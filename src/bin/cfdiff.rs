//! Differential test runner for the CPU core (see tools/cfdiff.py).
//!
//! stdin, one case per line:  CODEHEX D0..D7 A0..A7 SR   (hex words)
//! stdout, one per line:      D0..D7 A0..A7 PC MEMSUM    or  EXC <vec>

use dtemu::bus::Bus;
use dtemu::cpu::{Cpu, Stop};
use std::io::{BufRead, Write};

const CODE: u32 = 0x4000_1000;
const DATA: u32 = 0x4010_0000;
const DATA_LEN: usize = 0x1_0000;

fn pattern(i: usize) -> u8 {
    ((i * 37 + 11) & 0xFF) as u8
}

fn main() {
    let mut cpu = Cpu::new(Bus::new(128));
    let data: Vec<u8> = (0..DATA_LEN).map(pattern).collect();
    // Every vector points at a marker so an exception is visible.
    cpu.vbr = 0x4000_0000;
    for v in 0..256u32 {
        cpu.bus.poke32(0x4000_0000 + v * 4, 0x4000_0800 + v * 2);
    }
    let stdin = std::io::stdin();
    let mut out = std::io::BufWriter::new(std::io::stdout());
    for line in stdin.lock().lines() {
        let line = line.unwrap();
        let f: Vec<&str> = line.split_whitespace().collect();
        if f.len() < 18 {
            continue;
        }
        let code: Vec<u8> = (0..f[0].len() / 2)
            .map(|i| u8::from_str_radix(&f[0][2 * i..2 * i + 2], 16).unwrap())
            .collect();
        cpu.bus.poke_bytes(CODE, &[0u8; 256]);
        cpu.bus.poke_bytes(CODE, &code);
        cpu.bus.poke_bytes(DATA, &data);
        for i in 0..8 {
            cpu.d[i] = u32::from_str_radix(f[1 + i], 16).unwrap();
            cpu.a[i] = u32::from_str_radix(f[9 + i], 16).unwrap();
        }
        cpu.sr = u16::from_str_radix(f[17], 16).unwrap();
        cpu.macsr = 0;
        cpu.pc = CODE;
        let end = CODE + code.len() as u32;
        let mut exc = None;
        for _ in 0..64 {
            if cpu.pc == end {
                break;
            }
            cpu.step();
            if (0x4000_0800..0x4000_0A00).contains(&cpu.pc) {
                exc = Some((cpu.pc - 0x4000_0800) / 2);
                break;
            }
        }
        let _ = Stop::Budget;
        if let Some(v) = exc {
            writeln!(out, "EXC {v}").unwrap();
            continue;
        }
        let mut sum: u32 = 0;
        for i in 0..DATA_LEN as u32 {
            sum = sum.wrapping_mul(31).wrapping_add(cpu.bus.peek8(DATA + i) as u32);
        }
        let regs: Vec<String> = cpu.d.iter().chain(cpu.a.iter()).map(|v| format!("{v:08x}")).collect();
        writeln!(out, "{} {:08x} {:08x}", regs.join(" "), cpu.pc, sum).unwrap();
    }
}
