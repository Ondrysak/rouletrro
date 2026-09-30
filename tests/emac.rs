//! EMAC semantics, pinned to the ColdFire Programmer's Reference Manual
//! (chapter 6) and MCF5441x RM chapter 5. Cases ported from the reference
//! project's tests/test_unicorn_emac.py, whose expected values come from
//! the manuals rather than from any emulator.

use dtemu::bus::Bus;
use dtemu::cpu::Cpu;

const CODE: u32 = 0x4010_0000;
const DATA: u32 = 0x4020_0000;

fn reg<'a>(c: &'a mut Cpu, name: &str) -> &'a mut u32 {
    let n: usize = name[1..].parse().unwrap();
    if name.starts_with('D') {
        &mut c.d[n]
    } else {
        &mut c.a[n]
    }
}

fn run(words: &[u16], init: &[(&str, u32)], mem: &[(u32, u32)]) -> Cpu {
    let mut c = Cpu::new(Bus::new(128));
    let code: Vec<u8> = words.iter().flat_map(|w| w.to_be_bytes()).collect();
    c.bus.poke_bytes(CODE, &code);
    for &(a, v) in mem {
        c.bus.poke32(a, v);
    }
    c.sr = 0x2700;
    for &(r, v) in init {
        *reg(&mut c, r) = v;
    }
    c.pc = CODE;
    let end = CODE + code.len() as u32;
    let mut n = 0;
    while c.pc != end {
        c.step();
        n += 1;
        assert!(n < 100, "ran away at {:#x}", c.pc);
    }
    c
}

fn check(name: &str, c: &mut Cpu, expect: &[(&str, u32)]) {
    for &(r, v) in expect {
        let got = *reg(c, r);
        assert_eq!(got, v, "{name}: {r} = {got:#010x}, want {v:#010x}");
    }
}

#[test]
fn basic_cases() {
    let mut c = run(&[0xa907, 0xa103, 0xa1c4, 0xa185], &[("D3", 0xDEADBEEF), ("D7", 0)], &[]);
    check("movclr", &mut c, &[("D4", 0xDEADBEEF), ("D5", 0)]);

    let mut c = run(&[0xad02, 0xad86], &[("D2", 0x1234ABCD)], &[]);
    check("mask", &mut c, &[("D6", 0xFFFFABCD)]);

    let mut c = run(&[0xa906, 0xad07, 0xa106, 0xa401, 0x0800, 0xa180], &[("D1", 5), ("D2", 7), ("D6", 0), ("D7", 0xFFFF)], &[]);
    check("mac.l", &mut c, &[("D0", 35)]);

    let words = [
        0xa100, 0xa301, 0xa502, 0xa703, 0xab04, 0xaf05, 0xad06, 0xa907, 0xa988, 0xa93c, 0x0000, 0x0000, 0xab84, 0xaf85,
        0xa1c0, 0xa3c1, 0xa5c2, 0xa7c3, 0xad86, 0xa189, 0xa38a, 0xa58b, 0xa78c,
    ];
    let init = [
        ("D0", 0x12345678), ("D1", 1), ("D2", 0xFFFFFFFF), ("D3", 0x7FFFFFFF), ("D4", 0x11223344), ("D5", 0x55667788),
        ("D6", 0xFF), ("D7", 0xC21),
    ];
    let mut c = run(&words, &init, &[]);
    check(
        "handler prologue",
        &mut c,
        &[
            ("A0", 0xC21), ("D0", 0x12345678), ("D1", 1), ("D2", 0xFFFFFFFF), ("D3", 0x7FFFFFFF), ("D4", 0x11223344),
            ("D5", 0x55667788), ("D6", 0xFFFF00FF), ("A1", 0), ("A2", 0), ("A3", 0), ("A4", 0),
        ],
    );
}

#[test]
fn load_cases() {
    let mut c = run(
        &[0xa907, 0xa105, 0xa891, 0x00c6, 0xa181],
        &[("D7", 0), ("D5", 0), ("D6", 0x00030002), ("D0", 0x00050004), ("D2", 0x00090008), ("D4", 0xAAAAAAAA), ("A1", DATA)],
        &[(DATA, 0x2A)],
    );
    check("mac.w load", &mut c, &[("D1", 15), ("D4", 0x2A), ("A1", DATA)]);

    let mut c = run(&[0xa907, 0xa105, 0xaa91, 0x0804, 0xa181], &[("D7", 0), ("D5", 0), ("D0", 5), ("D2", 9), ("D4", 7), ("A1", DATA)], &[(DATA, 0x2A)]);
    check("mac.l load", &mut c, &[("D1", 35), ("D5", 0x2A)]);

    let mut c = run(
        &[0xa907, 0xa105, 0xad01, 0xaa9a, 0x01e4, 0xa181],
        &[("D7", 0), ("D5", 100), ("D1", 0xFFF), ("D0", 0x00050004), ("D2", 0x00090008), ("D4", 0x00070006), ("A2", DATA + 0x1010)],
        &[(DATA + 0x10, 0x11223344)],
    );
    check("msac.w masked", &mut c, &[("D1", 65), ("D5", 0x11223344)]);

    let mut c = run(
        &[0xa907, 0xa105, 0xaaa2, 0x0006, 0xa181],
        &[("D7", 0), ("D5", 0), ("D6", 0x00030002), ("D0", 0x00050004), ("D2", 0x00090008), ("A2", DATA + 0x20)],
        &[(DATA + 0x1C, 0x11223344)],
    );
    check("mac.w predec", &mut c, &[("D1", 8), ("D5", 0x11223344), ("A2", DATA + 0x1C)]);
}

const FRAC: [u16; 5] = [0xa907, 0xa1c1, 0xa805, 0x0800, 0xa1c1];

#[test]
fn fractional_cases() {
    let cases: &[(&str, u32, u32, u32, u32)] = &[
        ("0.970*0.5", 0xA0, 0x7C290000, 0x40000000, 0x3E148000),
        ("-0.5*0.5", 0xA0, 0xC0000000, 0x40000000, 0xE0000000),
        ("-0.5*-0.5", 0xA0, 0xC0000000, 0xC0000000, 0x20000000),
        ("-1*-1 OMC", 0xA0, 0x80000000, 0x80000000, 0x7FFFFFFF),
        ("-1*-1 no OMC", 0x20, 0x80000000, 0x80000000, 0x80000000),
        ("R/T rounds", 0x30, 0x7FE00000, 1, 1),
        ("R/T clear", 0x20, 0x7FE00000, 1, 0),
        ("S/U 16-bit", 0x60, 0x7C290000, 0x40000000, 0x3E14),
        ("S/U negative", 0x60, 0xC0000000, 0x40000000, 0xE000),
    ];
    for &(name, macsr, d5, d4, want) in cases {
        let mut c = run(&FRAC, &[("D7", macsr), ("D5", d5), ("D4", d4)], &[]);
        check(name, &mut c, &[("D1", want)]);
    }
}

#[test]
fn smoother_step() {
    let mut c = run(
        &[0xa907, 0xa1c1, 0xa891, 0x00c6, 0xa805, 0x0800, 0xa1c1],
        &[("D7", 0xA0), ("D6", 0x03D70000), ("D5", 0x7C290000), ("D0", 0x40000000), ("A1", DATA)],
        &[(DATA, 0x40000000)],
    );
    check("smoother", &mut c, &[("D1", 0x40000000), ("D4", 0x40000000)]);
}

#[test]
fn integer_and_scale_cases() {
    let mac_w_low = [0xa907, 0xa1c1, 0xa805, 0x0000, 0xa1c1];
    let mac_l = FRAC;
    let mac_l_sr1 = [0xa907, 0xa1c1, 0xa805, 0x0e00, 0xa1c1];
    let mac_l_sl1 = [0xa907, 0xa1c1, 0xa805, 0x0a00, 0xa1c1];
    let msac_l = [0xa907, 0xa1c1, 0xa805, 0x0900, 0xa1c1];
    let msac_l_sr1 = [0xa907, 0xa1c1, 0xa805, 0x0f00, 0xa1c1];
    // (name, program, MACSR (via D7), D5, D4, expected D1)
    type Case<'a> = (&'a str, &'a [u16; 5], u32, u32, u32, u32);
    let cases: &[Case] = &[
        ("signed word -3*5", &mac_w_low, 0x00, 0xFFFD, 5, 0xFFFFFFF1),
        ("signed OMC -1000*1", &mac_l, 0x80, 0xFFFFFC18, 1, 0xFFFFFC18),
        ("signed OMC sat +", &mac_l, 0x80, 0x7FFFFFFF, 2, 0x7FFFFFFF),
        ("signed OMC sat -", &mac_l, 0x80, 0x80000000, 2, 0x80000000),
        ("signed >>1", &mac_l_sr1, 0x00, 0xFFFFFFFA, 1, 0xFFFFFFFD),
        ("unsigned word", &mac_w_low, 0x40, 0xFFFD, 5, 0xFFFD * 5),
        ("unsigned long OMC", &mac_l, 0xC0, 0xFFFFFFFD, 1, 0xFFFFFFFD),
        ("signed <<1", &mac_l_sl1, 0x00, 3, 5, 30),
        ("unsigned <<1", &mac_l_sl1, 0x40, 3, 5, 30),
        ("fractional ignores the scale", &mac_l_sl1, 0xA0, 0x40000000, 0x40000000, 0x20000000),
        ("fractional msac", &msac_l, 0xA0, 0x40000000, 0x40000000, 0xE0000000),
        ("signed msac", &msac_l, 0x00, 3, 5, 0xFFFFFFF1),
        ("signed msac >>1", &msac_l_sr1, 0x00, 0xFFFFFFFA, 1, 3),
        ("signed msac OMC sat", &msac_l, 0x80, 0x80000000, 2, 0x7FFFFFFF),
    ];
    for &(name, words, macsr, d5, d4, want) in cases {
        let mut c = run(words, &[("D7", macsr), ("D5", d5), ("D4", d4)], &[]);
        check(name, &mut c, &[("D1", want)]);
    }
}

#[test]
fn mode_switch_keeps_the_accumulator() {
    for (old, new) in [(0x20, 0x00), (0x00, 0x20), (0x00, 0x40)] {
        let mut c = run(&[0xa907, 0xa101, 0xa906, 0xa182], &[("D7", old), ("D6", new), ("D1", 0x12345678)], &[]);
        check("mode switch", &mut c, &[("D2", 0x12345678)]);
    }
}

#[test]
fn macsr_read_clears_high_bits() {
    // p.6-12: MOVE MACSR,Rx clears Rx[31:12].
    let mut c = run(&[0xa93c, 0xffff, 0xffff, 0xa980], &[], &[]);
    check("macsr", &mut c, &[("D0", 0xFFF)]);
}

#[test]
fn move_acc_to_acc() {
    // p.6-14: MOVE.L ACCy,ACCx.
    let mut c = run(&[0xa907, 0xa301, 0xa502, 0xa511, 0xa583], &[("D1", 0x11111111), ("D2", 0x22222222), ("D7", 0)], &[]);
    check("acc to acc", &mut c, &[("D3", 0x11111111)]);
}

/// The predecoded MAC path (taken by `step`) against the interpreter's
/// `mac_core`, over random operands, modes, accumulators and load forms.
#[test]
fn fast_mac_matches_interpreter() {
    let mut seed: u64 = 0x9E3779B97F4A7C15;
    let mut rnd = move || {
        seed ^= seed << 13;
        seed ^= seed >> 7;
        seed ^= seed << 17;
        seed
    };
    for case in 0..50_000 {
        // MAC/MSAC opcode: 1010 Rx 0 acc Rxa mode(bits 5-3) reg
        let load = rnd() % 2 == 0;
        let mode = if load { [2u16, 3, 4, 5][(rnd() % 4) as usize] } else { 0 };
        let reg = (rnd() % 6) as u16; // a0-a5 point at data
        let mut op: u16 = 0xA000 | (((rnd() % 8) as u16) << 9) | (((rnd() % 2) as u16) << 7) | (((rnd() % 2) as u16) << 6);
        if load {
            op |= (mode << 3) | reg;
        } else {
            op |= (rnd() % 16) as u16; // Ry register, D or A
        }
        let mut ext = (rnd() & 0xFFFF) as u16;
        ext &= !0x0600 | if rnd() % 3 == 0 { 0x0600 } else { 0 }; // mostly no scale
        let macsr = [0x00u32, 0x20, 0x80, 0xA0, 0x10, 0x40, 0x60, 0xB0][(rnd() % 8) as usize] | ((rnd() % 16) as u32) << 8 & 0xF00;
        let disp = ((rnd() % 64) as i16 * 4) as u16;
        let mut words = vec![op, ext];
        if load && mode == 5 {
            words.push(disp);
        }
        let mut regs = [0u32; 16];
        for (i, r) in regs.iter_mut().enumerate() {
            *r = match rnd() % 4 {
                0 => 0x8000_0000,
                1 => (rnd() as u32) & 0xFFFF,
                _ => rnd() as u32,
            };
            if (8..14).contains(&i) {
                *r = DATA + 0x1000 + ((rnd() % 256) as u32) * 4;
            }
        }
        let accs: Vec<i64> = (0..4).map(|_| ((rnd() as i64) << 16) >> 16).collect();
        let setup = |c: &mut Cpu| {
            c.d.copy_from_slice(&regs[..8]);
            c.a.copy_from_slice(&regs[8..]);
            c.macsr = macsr;
            c.mask = 0xFFFF_0000 | (rnd_mask(case) as u32);
            for (i, &v) in accs.iter().enumerate() {
                c.acc_set(i, v);
            }
        };
        let code: Vec<u8> = words.iter().flat_map(|w| w.to_be_bytes()).collect();
        // Fast path.
        let mut f = Cpu::new(Bus::new(128));
        f.bus.poke_bytes(CODE, &code);
        for k in 0..0x800u32 {
            f.bus.poke32(DATA + k * 4, k.wrapping_mul(0x9E37_79B9));
        }
        setup(&mut f);
        f.pc = CODE;
        f.sr = 0x2700;
        f.step();
        // Interpreter.
        let mut s = Cpu::new(Bus::new(128));
        for k in 0..0x800u32 {
            s.bus.poke32(DATA + k * 4, k.wrapping_mul(0x9E37_79B9));
        }
        setup(&mut s);
        s.mac_core(op, ext, disp as i16 as i32 as u32);
        assert_eq!((f.d, f.a, f.accv, f.macsr), (s.d, s.a, s.accv, s.macsr), "case {case}: op {op:04x} ext {ext:04x} macsr {macsr:03x}");
    }
}

fn rnd_mask(case: i32) -> u16 {
    if case % 5 == 0 {
        0x0FFF
    } else {
        0xFFFF
    }
}
