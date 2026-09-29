//! Headless boot: run the firmware and report progress.
//!
//!     dtboot FIRMWARE.syx [--instr N] [--every N] [--ips N] [--no-masks]

use dtemu::cpu::Stop;
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

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let mut syx = None;
    let mut instr = 100_000_000u64;
    let mut every = 10_000_000u64;
    let mut ips = None;
    let mut no_masks = false;
    let mut watch: Option<u32> = None;
    let mut breaks: Vec<u32> = Vec::new();
    let mut trace = 0usize;
    let mut break_after = 0u64;
    let mut counts: Vec<u32> = Vec::new();
    let mut profile_after: Option<u64> = None;
    let mut save: Option<String> = None;
    let mut load: Option<String> = None;
    let mut it = args.iter();
    while let Some(a) = it.next() {
        match a.as_str() {
            "--instr" => instr = num(it.next().unwrap()),
            "--every" => every = num(it.next().unwrap()),
            "--ips" => ips = Some(num(it.next().unwrap())),
            "--no-masks" => no_masks = true,
            "--break" => {
                let a = u32::from_str_radix(it.next().unwrap().trim_start_matches("0x"), 16).unwrap();
                breaks.push(a);
            }
            "--save" => save = it.next().cloned(),
            "--load" => load = it.next().cloned(),
            "--profile-after" => profile_after = Some(num(it.next().unwrap())),
            "--count" => {
                let a = u32::from_str_radix(it.next().unwrap().trim_start_matches("0x"), 16).unwrap();
                counts.push(a);
            }
            "--break-after" => break_after = num(it.next().unwrap()),
            "--trace" => trace = num(it.next().unwrap()) as usize,
            "--watch" => {
                let a = u32::from_str_radix(it.next().unwrap().trim_start_matches("0x"), 16).unwrap();
                watch = Some(a);
            }
            _ => syx = Some(a.clone()),
        }
    }
    let syx = syx.unwrap_or_else(|| "fw/Digitakt_OS1.53.syx".into());
    let fw = Firmware::from_file(std::path::Path::new(&syx)).unwrap();
    let mut m = Machine::new(&fw, 128).unwrap();
    if let Some(i) = ips {
        m.cpu.bus.io.ips = i as f64;
    }
    m.cpu.bus.io.ignore_masks = no_masks;
    if !breaks.is_empty() {
        m.cpu.history = Some((vec![0; 48], 0));
    }
    for c in &counts {
        m.add_counter(*c);
    }
    if break_after > 0 {
        m.run_until(break_after);
    }
    for b in &breaks {
        m.add_breakpoint(*b);
    }
    if let Some(a) = watch {
        let o = a & 0x07FF_FFFF;
        m.cpu.bus.watch = Some((o, o + 4));
    }
    if let Some(p) = &load {
        dtemu::snapshot::load(&mut m, std::path::Path::new(p)).unwrap();
        println!("restored {p} at clock {}", m.now());
        instr += m.now();
    }
    let t0 = Instant::now();
    let mut logged = 0;
    let mut target = m.now();
    while target < instr {
        target = (target + every).min(instr);
        if let Some(pa) = profile_after {
            if m.now() >= pa && m.cpu.profile.is_none() {
                m.cpu.profile = Some(Default::default());
            }
        }
        let stop = m.run_until(target);
        for l in &m.log[logged..] {
            println!("{l}");
        }
        logged = m.log.len();
        let io = &m.cpu.bus.io;
        let mut top: Vec<(usize, u64)> = io.irq_taken.iter().copied().enumerate().filter(|x| x.1 > 0).collect();
        top.sort_by(|a, b| b.1.cmp(&a.1));
        let irqs: Vec<String> = top.iter().take(8).map(|(v, n)| format!("v{v}:{n}")).collect();
        let lit = m.frame.as_ref().map(|f| Machine::lit(f)).unwrap_or(0);
        println!(
            "{:>6.0}M pc={:08x} sr={:04x} idle={} frames={} lit={} tasks={} mainloop={} irq[{}] {:.1}s {}",
            m.now() as f64 / 1e6,
            m.cpu.pc,
            m.cpu.sr,
            m.stats.idle_hits,
            m.stats.frames,
            lit,
            m.stats.tasks.len(),
            m.stats.mainloop,
            irqs.join(" "),
            t0.elapsed().as_secs_f64(),
            m.counters.iter().map(|(a, n)| format!("{a:08x}:{n}")).collect::<Vec<_>>().join(" ")
        );
        if let Stop::Fault(v, p) = stop {
            println!("FAULT vector {v} at 0x{p:08x}");
            break;
        }
        if let Stop::Hook(pc) = stop {
            println!("BREAK at 0x{pc:08x}");
            if let Some((h, i)) = &m.cpu.history {
                let n = h.len();
                let from = i.saturating_sub(n);
                let pcs: Vec<String> = (from..*i).map(|k| format!("{:08x}", h[k % n])).collect();
                println!("history: {}", pcs.join(" "));
            }
            for _ in 0..trace {
                let c = &m.cpu;
                let op = c.bus.peek16(c.pc);
                println!("  {:08x} {:04x}  sr={:04x} d0={:08x} d1={:08x} a0={:08x} a7={:08x}", c.pc, op, c.sr, c.d[0], c.d[1], c.a[0], c.a[7]);
                m.cpu.step();
            }
            break;
        }
        if let Stop::Stopped = stop {
            println!("STOPPED with nothing to wake it at pc=0x{:08x}", m.cpu.pc);
            break;
        }
    }
    if let Some(p) = &save {
        dtemu::snapshot::save(&m, std::path::Path::new(p)).unwrap();
        println!("saved {p} ({} bytes)", std::fs::metadata(p).unwrap().len());
    }
    let secs = t0.elapsed().as_secs_f64();
    let executed = m.now().saturating_sub(m.stats.idle_skipped);
    println!(
        "{} instructions ({} executed, {} idle-skipped) in {:.2}s = {:.1} MIPS executed",
        m.now(),
        executed,
        m.stats.idle_skipped,
        secs,
        executed as f64 / secs / 1e6
    );
    let exc: Vec<String> = m.cpu.exc_counts.iter().enumerate().filter(|x| *x.1 > 0 && x.0 < 64).map(|(v, n)| format!("{v}:{n}")).collect();
    println!("cpu exceptions: {}", exc.join(" "));
    for (v, pc, op, t) in &m.cpu.fault_log {
        println!("  fault vector {v} at 0x{pc:08x} opcode {op:04x} (clock {t})");
    }
    let pages: Vec<String> = m.cpu.bus.unmapped_touch.iter().filter(|(p, _)| **p >= 0xE000_0000).map(|(p, (pc, n))| format!("{p:08x}(pc {pc:08x} x{n})")).collect();
    println!("unmodelled io pages: {}", pages.join(" "));
    print!("{}", m.task_report());
    let vt: Vec<String> = [97u32, 99, 154, 155, 180, 191, 192, 205, 207, 208, 223].iter().map(|v| format!("v{v}={:08x}", m.cpu.bus.peek32(m.cpu.vbr + v * 4))).collect();
    println!("vectors: {}", vt.join(" "));
    if let Some(p) = &m.cpu.profile {
        let total: u64 = p.values().sum();
        let mut by_fn: std::collections::HashMap<u32, u64> = Default::default();
        for (pc, n) in p {
            *by_fn.entry(pc & !0xFF).or_insert(0) += n;
        }
        let mut v: Vec<_> = by_fn.into_iter().collect();
        v.sort_by(|a, b| b.1.cmp(&a.1));
        let mut ops: std::collections::HashMap<u16, u64> = Default::default();
        for (pc, n) in p {
            *ops.entry(m.cpu.bus.peek16(*pc)).or_insert(0) += n;
        }
        let mut ov: Vec<_> = ops.into_iter().collect();
        ov.sort_by(|a, b| b.1.cmp(&a.1));
        let mut acc = 0.0;
        println!("opcodes:");
        for (o, n) in ov.iter().take(40) {
            let f = *n as f64 * 100.0 / total as f64;
            acc += f;
            println!("  {o:04x} {f:5.2}% (cum {acc:5.1}%)");
        }
        println!("distinct opcodes: {}", ov.len());
        println!("guest profile ({total} samples), by 256-byte region:");
        for (a, n) in v.iter().take(25) {
            println!("  {a:08x} {:5.1}%", *n as f64 * 100.0 / total as f64);
        }
    }
    println!("icache: {} decodes, {} invalidating stores", m.cpu.bus.icache_decodes, m.cpu.bus.icache_invalidations);
    let c = &m.cpu;
    for (pc, a, n, v) in c.bus.watch_log.iter().take(40) {
        println!("watch: pc={pc:08x} [{a:08x}].{n} <- {v:08x}");
    }
    println!("d: {}", c.d.iter().map(|v| format!("{v:08x}")).collect::<Vec<_>>().join(" "));
    println!("a: {}", c.a.iter().map(|v| format!("{v:08x}")).collect::<Vec<_>>().join(" "));
    let sp = c.a[7];
    println!("stack: {}", (0..24).map(|i| format!("{:08x}", c.bus.peek32(sp + 4 * i))).collect::<Vec<_>>().join(" "));
    println!("uart8 tx ({} bytes): {}", c.bus.io.uart8.tx.len(), c.bus.io.uart8.tx.iter().take(64).map(|b| format!("{b:02x}")).collect::<Vec<_>>().join(" "));
    let el = &c.bus.io.esdhc.log;
    let log: Vec<String> = el.iter().skip(el.len().saturating_sub(24)).map(|(i, a)| format!("CMD{i}({a:x})")).collect();
    println!("esdhc: {} commands: {}", c.bus.io.esdhc.log.len(), log.join(" "));
    for (i, ic) in c.bus.io.intc.iter().enumerate() {
        let lv: Vec<String> = ic.icr.iter().enumerate().filter(|x| *x.1 != 0).map(|(s, l)| format!("{s}:{l}")).collect();
        println!("intc{i}: imr={:016x} frc={:016x} lines={:016x} icr {}", ic.imr, ic.frc, ic.lines, lv.join(" "));
    }
    for (i, p) in c.bus.io.pit.iter().enumerate() {
        println!("pit{i}: pcsr={:04x} pmr={:04x} fired={}", p.pcsr, p.pmr, p.fired);
    }
    for (i, d) in c.bus.io.dtim.iter().enumerate() {
        println!("dtim{i}: dtmr={:04x} dter={:02x} dtrr={:08x} fired={}", d.dtmr, d.dter, d.dtrr, d.fired);
    }
    let ssi = &c.bus.io.ssi;
    let nz = ssi.out.iter().filter(|&&x| x != 0).count();
    let peak = ssi.out.iter().map(|x| x.unsigned_abs()).max().unwrap_or(0);
    println!("ssi: frames={} buffered={} nonzero={} peak={} tx_pending={}", ssi.frames, ssi.out.len(), nz, peak, ssi.tx_pending);
    println!("edma: erq={:016x} int={:016x} majors[34]={} [35]={} [59]={}", c.bus.io.edma.erq, c.bus.io.edma.int, c.bus.io.edma.majors[34], c.bus.io.edma.majors[35], c.bus.io.edma.majors[59]);
    if let Some(f) = &m.frame {
        print!("{}", Machine::frame_ascii(f));
    }
}
