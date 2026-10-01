//! Hold the block compiler to the interpreter: resume one snapshot on two
//! machines, one compiling hot blocks and one not, run both to the same
//! clocks, and compare everything at each: registers, the clock, DDR, SRAM
//! and the peripherals. The machine is deterministic, so any difference is
//! a compiler bug, found within one checkpoint interval.
//!
//!     jitcheck SNAPSHOT [--instr N] [--every N] [--from N]
//!     jitcheck --compare A.snap B.snap
//!
//! With no snapshot (`-`), both cold-boot from reset. `--compare` holds two
//! saved machines to the same test, e.g. one build's against another's.

use dtemu::firmware::Firmware;
use dtemu::machine::Machine;
use std::time::Instant;

fn num(s: &str) -> u64 {
    let s = s.replace('_', "");
    let (n, mul) = match s.chars().last() {
        Some('M') => (&s[..s.len() - 1], 1_000_000),
        Some('G') => (&s[..s.len() - 1], 1_000_000_000),
        Some('k') => (&s[..s.len() - 1], 1_000),
        _ => (&s[..], 1),
    };
    n.parse::<f64>().map(|v| (v * mul as f64) as u64).expect("number")
}

/// Run `m` to clock `t`. -> why it could not, if it could not.
fn run_to(m: &mut Machine, t: u64) -> Option<String> {
    let mut last = m.now();
    while m.now() < t {
        let s = m.run_until(t);
        if m.now() == last {
            return Some(format!("stuck at clock {} ({s:?})", m.now()));
        }
        last = m.now();
    }
    None
}

fn machine(fw: &Firmware, snap: &str, jit: bool) -> Machine {
    let mut m = Machine::new(fw, 128).unwrap();
    if !jit {
        m.cpu.jit = None;
        m.cpu.jitw = None;
    } else if m.cpu.jit.is_none() && m.cpu.jitw.is_none() {
        m.cpu.jitw = Some(dtemu::jit::Worker::new());
    }
    if snap != "-" {
        dtemu::snapshot::load(&mut m, std::path::Path::new(snap)).unwrap();
    }
    m
}

mod erased {
    /// Serialize anything serde can, for comparing.
    pub trait Ser {
        fn bytes(&self) -> Vec<u8>;
    }
    impl<T: serde::Serialize> Ser for T {
        fn bytes(&self) -> Vec<u8> {
            bincode::serialize(self).unwrap()
        }
    }
}

/// -> what differs between the two machines, if anything.
fn diff(a: &Machine, b: &Machine) -> Vec<String> {
    let (x, y) = (&a.cpu, &b.cpu);
    let mut out = Vec::new();
    let mut reg = |name: &str, p: u64, q: u64| {
        if p != q {
            out.push(format!("{name}: interp {p:x} jit {q:x}"));
        }
    };
    for i in 0..8 {
        reg(&format!("d{i}"), x.d[i] as u64, y.d[i] as u64);
        reg(&format!("a{i}"), x.a[i] as u64, y.a[i] as u64);
    }
    reg("pc", x.pc as u64, y.pc as u64);
    reg("sr", x.sr as u64, y.sr as u64);
    reg("other_sp", x.other_sp as u64, y.other_sp as u64);
    reg("macsr", x.macsr as u64, y.macsr as u64);
    reg("mask", x.mask as u64, y.mask as u64);
    for i in 0..4 {
        reg(&format!("acc{i}"), x.accv[i] as u64, y.accv[i] as u64);
    }
    reg("clock", x.bus.io.now, y.bus.io.now);
    reg("stopped", x.stopped as u64, y.stopped as u64);
    if x.bus.ddr != y.bus.ddr {
        let first = x.bus.ddr.iter().zip(&y.bus.ddr).position(|(p, q)| p != q).unwrap();
        out.push(format!(
            "DDR differs, first at 0x{:08x}: interp {:02x} jit {:02x}",
            0x4000_0000 + first,
            x.bus.ddr[first],
            y.bus.ddr[first]
        ));
    }
    if x.bus.sram != y.bus.sram {
        out.push("SRAM differs".into());
    }
    let (p, q) = (&x.bus.io, &y.bus.io);
    let mut part = |name: &str, s: Vec<u8>, t: Vec<u8>| {
        if s != t {
            out.push(format!("peripheral state differs: {name}"));
        }
    };
    let ser = |v: &dyn erased::Ser| v.bytes();
    part("intc", ser(&p.intc), ser(&q.intc));
    part("pit", ser(&p.pit), ser(&q.pit));
    part("dtim", ser(&p.dtim), ser(&q.dtim));
    part("uart8", ser(&p.uart8), ser(&q.uart8));
    part("uart9", ser(&p.uart9), ser(&q.uart9));
    part("panel", ser(&p.panel), ser(&q.panel));
    part("edma", ser(&p.edma), ser(&q.edma));
    part("esdhc", ser(&p.esdhc), ser(&q.esdhc));
    part("ssi", ser(&p.ssi), ser(&q.ssi));
    part("irq", ser(&(p.irq_level, p.deadline, p.gpio_d4, p.pit_hold)), ser(&(q.irq_level, q.deadline, q.gpio_d4, q.pit_hold)));
    part("irq_taken", ser(&p.irq_taken.to_vec()), ser(&q.irq_taken.to_vec()));
    if p.forced != q.forced {
        out.push("peripheral state differs: forced".into());
    }
    out
}

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    if args.is_empty() || args[0] == "-h" || args[0] == "--help" {
        println!("usage: jitcheck SNAPSHOT|- [--instr N] [--every N] [--from N]\n       jitcheck --compare A.snap B.snap");
        return;
    }
    if args[0] == "--compare" {
        let fw = Firmware::load(std::path::Path::new("fw/Digitakt_OS1.53.syx"), None).unwrap();
        let (a, b) = (machine(&fw, &args[1], false), machine(&fw, &args[2], false));
        let d = diff(&a, &b);
        if d.is_empty() {
            println!("identical");
            return;
        }
        for l in d.iter().take(30) {
            println!("  {}", l.replace("interp", "A").replace("jit", "B"));
        }
        std::process::exit(1);
    }
    let snap = args[0].clone();
    let (mut instr, mut every, mut from) = (200_000_000u64, 10_000_000u64, 0u64);
    let mut it = args[1..].iter();
    while let Some(a) = it.next() {
        match a.as_str() {
            "--instr" => instr = num(it.next().unwrap()),
            "--every" => every = num(it.next().unwrap()),
            "--from" => from = num(it.next().unwrap()),
            x => panic!("unknown option {x}"),
        }
    }
    let fw = Firmware::load(std::path::Path::new("fw/Digitakt_OS1.53.syx"), None).unwrap();
    let mut a = machine(&fw, &snap, false);
    let mut b = machine(&fw, &snap, true);
    let start = a.now();
    if from > 0 {
        run_to(&mut a, start + from);
        run_to(&mut b, start + from);
    }
    let (mut ta, mut tb) = (0.0, 0.0);
    let mut t = a.now();
    let end = start + from + instr;
    while t < end {
        t = (t + every).min(end);
        let s = Instant::now();
        let wa = run_to(&mut a, t);
        ta += s.elapsed().as_secs_f64();
        let s = Instant::now();
        let wb = run_to(&mut b, t);
        tb += s.elapsed().as_secs_f64();
        if wa.is_some() || wb.is_some() {
            println!("interp: {wa:?}; jit: {wb:?}");
        }
        let d = diff(&a, &b);
        if !d.is_empty() {
            println!("MISMATCH by clock {t} (previous checkpoint {}):", t - every);
            for l in d.iter().take(30) {
                println!("  {l}");
            }
            std::process::exit(1);
        }
    }
    println!("idle-skipped: interp {} jit {}", a.stats.idle_skipped, b.stats.idle_skipped);
    let (compiled, failed, secs) = b.cpu.jit_stats();
    println!(
        "identical through clock {t}: {} instructions; interp {ta:.2}s, jit {tb:.2}s ({:.2}x); {} blocks compiled in {:.2}s, {} declined; {} cache flushes; {} entries into compiled code",
        t - start,
        ta / tb.max(1e-9),
        compiled,
        secs,
        failed,
        b.cpu.bus.icache_flushes,
        b.cpu.jit_runs
    );
}
