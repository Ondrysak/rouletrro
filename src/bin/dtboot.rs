//! Headless boot: run the firmware and report progress.
//!
//!     dtboot [FIRMWARE.syx] [options]   (see --help)

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

const USAGE: &str = "\
usage: dtboot [FIRMWARE.syx] [options]

Runs the firmware headless and reports progress. Clocks are instruction
counts (suffixes k, M, G); after --load they count from the snapshot's.

  --instr N            run N instructions (default 100M)
  --every N            report every N (default 10M)
  --ips N              instructions per emulated second (default 200M)
  --main-os IMAGE.bin  run this MAIN OS (raw, as fwinfo -o extracts it)
                       in place of the update's
  --load SNAP          resume a snapshot
  --save SNAP          save a snapshot at the end
  --card IMAGE         back the eMMC with an image file (read only: changes
                       are not written back; a missing image starts formatted)
  --press CODE@AT[:HOLD]
                       press key CODE at clock AT for HOLD (default 30M)
  --turn ENC:DELTA@ATxCOUNT[/SPACING]
                       turn encoder ENC by DELTA, COUNT times, SPACING apart
  --wav OUT.wav        record the audio output
  --break ADDR         stop at a PC (hex); --break-after N arms it after N
  --watch ADDR         log writes to a DDR address (hex)
  --count ADDR         count visits to a PC (hex)
  --trace N            keep the last N PCs for fault reports
  --profile-after N    sample guest PCs from clock N on
  --no-masks           ignore interrupt masks

Environment:
  DTBOOT_OLED          end with the panel MCU's OLED, not the last captured frame
  DTBOOT_STEPS         after each key release, settle 40M and show the screen
                       (as PNGs in DTBOOT_PNG_DIR if set)
  DTBOOT_TURN_PNG=DIR  a PNG after each turn, DTBOOT_TURN_SETTLE (3M) later
";

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let mut syx = None;
    let mut main_os: Option<String> = None;
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
    let mut wav: Option<String> = None;
    let mut card: Option<String> = None;
    let mut pcm: Vec<i32> = Vec::new();
    let mut presses: Vec<(u8, u64, u64)> = Vec::new();
    let mut step_no = 0;
    let mut turns: Vec<(u64, u8, i8)> = Vec::new();
    let mut load: Option<String> = None;
    let mut it = args.iter();
    while let Some(a) = it.next() {
        match a.as_str() {
            "--main-os" => main_os = it.next().cloned(),
            "--instr" => instr = num(it.next().unwrap()),
            "--every" => every = num(it.next().unwrap()),
            "--ips" => ips = Some(num(it.next().unwrap())),
            "--no-masks" => no_masks = true,
            "-h" | "--help" => {
                print!("{USAGE}");
                return;
            }
            "--break" => {
                let a = u32::from_str_radix(it.next().unwrap().trim_start_matches("0x"), 16).unwrap();
                breaks.push(a);
            }
            "--press" => {
                // --press CODE@CLOCK[:HOLD]  e.g. 6@4100M:50M
                let v = it.next().unwrap();
                let (code, rest) = v.split_once('@').unwrap();
                let (at, hold) = rest.split_once(':').unwrap_or((rest, "30M"));
                presses.push((code.parse::<u8>().unwrap(), num(at), num(hold)));
            }
            "--wav" => wav = it.next().cloned(),
            "--card" => card = it.next().cloned(),
            "--turn" => {
                // --turn ENC:DELTA@CLOCKxCOUNT[/SPACING], one turn every SPACING (4M)
                let v = it.next().unwrap();
                let (ed, rest) = v.split_once('@').unwrap();
                let (e, d) = ed.split_once(':').unwrap();
                let (rest, spacing) = rest.split_once('/').map(|(a, b)| (a, num(b))).unwrap_or((rest, 4_000_000));
                let (at, count) = rest.split_once('x').unwrap_or((rest, "1"));
                for k in 0..count.parse::<u64>().unwrap() {
                    turns.push((num(at) + k * spacing, e.parse().unwrap(), d.parse().unwrap()));
                }
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
    let fw = Firmware::load(std::path::Path::new(&syx), main_os.as_deref().map(std::path::Path::new)).unwrap();
    let mut m = Machine::new(&fw, 128).unwrap();
    for l in m.profile_notes() {
        println!("{l}");
    }
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
    if let Some(p) = &card {
        let fresh = m.attach_card(std::path::Path::new(p)).unwrap();
        println!("card {p}: {} sectors in use{}", m.cpu.bus.io.esdhc.card.sectors.len(), if fresh { " (new, sample area formatted)" } else { "" });
    }
    if let Some(p) = &load {
        if let Err(e) = dtemu::snapshot::load(&mut m, std::path::Path::new(p)) {
            eprintln!("{p}: {e}");
            std::process::exit(1);
        }
        println!("restored {p} at clock {}", m.now());
        // Every clock given counts from the snapshot's.
        let base = m.now();
        instr += base;
        for p in &mut presses {
            p.1 += base;
        }
        for t in &mut turns {
            t.0 += base;
        }
        if let Some(pa) = &mut profile_after {
            *pa += base;
        }
    }
    let t0 = Instant::now();
    let (clock0, idle0) = (m.now(), m.stats.idle_skipped);
    let mut logged = 0;
    let mut target = m.now();
    while target < instr {
        target = (target + every).min(instr);
        if let Some(pa) = profile_after {
            if m.now() >= pa && m.cpu.profile.is_none() {
                m.cpu.profile = Some(Default::default());
            }
        }
        // Key presses and encoder turns scheduled inside this chunk, in time order.
        // kind: 0 = key down, 1 = key up, 2 = turn
        let mut events: Vec<(u64, u8, u8, i8)> = Vec::new();
        for &(code, at, hold) in &presses {
            events.push((at, 0, code, 0));
            events.push((at + hold, 1, code, 0));
        }
        for &(at, e, d) in &turns {
            events.push((at, 2, e, d));
        }
        events.sort();
        for (at, kind, code, d) in events {
            if at < m.now() || at >= target {
                continue;
            }
            m.run_until(at);
            if kind == 2 {
                m.turn(code, d);
                if let Some(dir) = std::env::var_os("DTBOOT_TURN_PNG") {
                    let settle = m.now() + std::env::var("DTBOOT_TURN_SETTLE").ok().and_then(|v| v.parse().ok()).unwrap_or(3_000_000);
                    m.run_until(settle);
                    step_no += 1;
                    oled_png(&m, &std::path::Path::new(&dir).join(format!("turn{step_no:02}.png")));
                }
                continue;
            }
            let down = kind == 0;
            m.key(code, down);
            println!("[{:>12}] key {} {}", m.now(), dtemu::panel::key_name(code).unwrap_or("?"), if down { "down" } else { "up" });
            if !down && std::env::var_os("DTBOOT_STEPS").is_some() {
                let settle = m.now() + 40_000_000;
                m.run_until(settle);
                if let Some(dir) = std::env::var_os("DTBOOT_PNG_DIR") {
                    step_no += 1;
                    oled_png(&m, &std::path::Path::new(&dir).join(format!("step{step_no:02}_{}.png", dtemu::panel::key_name(code).unwrap_or("K"))));
                } else {
                    print!("{}", Machine::frame_ascii(&m.cpu.bus.io.panel.oled));
                }
            }
        }
        let stop = m.run_until(target);
        if wav.is_some() {
            pcm.extend(m.cpu.bus.io.ssi.out.drain(..));
        }
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
    if let Some(p) = &wav {
        write_wav(p, &pcm);
        let peak = pcm.iter().map(|x| x.unsigned_abs()).max().unwrap_or(0);
        let nz = pcm.iter().filter(|&&x| x != 0).count();
        println!("wrote {p}: {} frames, peak {peak} (of 8388607), {nz} nonzero samples", pcm.len() / 2);
    }
    if let Some(p) = &save {
        dtemu::snapshot::save(&m, std::path::Path::new(p)).unwrap();
        println!("saved {p} ({} bytes)", std::fs::metadata(p).unwrap().len());
    }
    let secs = t0.elapsed().as_secs_f64();
    let executed = (m.now() - clock0) - (m.stats.idle_skipped - idle0);
    println!(
        "{} instructions ({} executed, {} idle-skipped) in {:.2}s = {:.1} MIPS executed, {:.0}% of real time",
        m.now() - clock0,
        executed,
        m.stats.idle_skipped - idle0,
        secs,
        executed as f64 / secs / 1e6,
        (m.now() - clock0) as f64 / m.cpu.bus.io.ips / secs * 100.0
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
        let mut blocks: Vec<(u32, u64)> = p.iter().map(|(a, n)| (*a, *n)).collect();
        blocks.sort_by(|a, b| b.1.cmp(&a.1));
        println!("hottest blocks (ops, of which the compiler calls a handler for):");
        let mut calls: std::collections::HashMap<u16, f64> = Default::default();
        let mut call_share = 0.0;
        for (rank, (a, n)) in blocks.iter().enumerate() {
            let share = *n as f64 * 100.0 / total as f64;
            let ops = m.cpu.bus.block_ops(*a).unwrap_or_default();
            let called: Vec<u16> = ops.iter().filter(|o| matches!(o.k, dtemu::fast::Kind::Call)).map(|o| o.op).collect();
            if !ops.is_empty() {
                for &o in &called {
                    *calls.entry(o).or_insert(0.0) += share / ops.len() as f64;
                }
                call_share += share * called.len() as f64 / ops.len() as f64;
            }
            if rank < 25 {
                let c: Vec<String> = called.iter().map(|o| format!("{o:04x}")).collect();
                println!("  {a:08x} {share:5.1}%  {:>3} ops, {} calls {}", ops.len(), called.len(), c.join(" "));
            }
        }
        let mut cv: Vec<_> = calls.into_iter().collect();
        cv.sort_by(|a, b| b.1.total_cmp(&a.1));
        println!("ops run as handler calls: about {call_share:.1}% of instructions; by opcode:");
        for (o, f) in cv.iter().take(30) {
            println!("  {o:04x} {f:5.2}%");
        }
        println!("guest profile ({total} samples), by 256-byte region:");
        for (a, n) in v.iter().take(25) {
            println!("  {a:08x} {:5.1}%", *n as f64 * 100.0 / total as f64);
        }
    }
    println!("+Drive mounted: {}", m.drive_mounted().map_or("unknown (flag not found in this build)".into(), |v| v.to_string()));
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
    if std::env::var_os("DTBOOT_OLED").is_some() {
        println!("panel MCU OLED ({} frames):", m.cpu.bus.io.panel.oled_frames);
        print!("{}", Machine::frame_ascii(&m.cpu.bus.io.panel.oled));
    } else if let Some(f) = &m.frame {
        print!("{}", Machine::frame_ascii(f));
    }
}

/// 16-bit stereo WAV at 48 kHz from 24-bit samples.
fn write_wav(path: &str, pcm: &[i32]) {
    let data: Vec<u8> = pcm.iter().flat_map(|&s| ((s >> 8) as i16).to_le_bytes()).collect();
    let mut f = Vec::new();
    f.extend_from_slice(b"RIFF");
    f.extend_from_slice(&(36 + data.len() as u32).to_le_bytes());
    f.extend_from_slice(b"WAVEfmt ");
    f.extend_from_slice(&16u32.to_le_bytes());
    f.extend_from_slice(&1u16.to_le_bytes());
    f.extend_from_slice(&2u16.to_le_bytes());
    f.extend_from_slice(&48_000u32.to_le_bytes());
    f.extend_from_slice(&(48_000u32 * 4).to_le_bytes());
    f.extend_from_slice(&4u16.to_le_bytes());
    f.extend_from_slice(&16u16.to_le_bytes());
    f.extend_from_slice(b"data");
    f.extend_from_slice(&(data.len() as u32).to_le_bytes());
    f.extend_from_slice(&data);
    std::fs::write(path, f).unwrap();
}

fn oled_png(m: &Machine, path: &std::path::Path) {
    let mut cv = dtemu::gui::Canvas::new(128 * 3, 64 * 3);
    let oled = &m.cpu.bus.io.panel.oled;
    for y in 0..64usize {
        for x in 0..128usize {
            let on = (oled[(7 - y / 8) + 8 * x] >> (y % 8)) & 1 != 0;
            cv.fill(x as i32 * 3, y as i32 * 3, 3, 3, if on { 0xFFFFFF } else { 0 });
        }
    }
    dtemu::gui::save_png(&cv, path).unwrap();
}
