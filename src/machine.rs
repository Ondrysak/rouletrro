//! The Digitakt: CPU, memory, peripherals, and the few high-level stand-ins
//! for hardware that is not modelled at register level.

use crate::bus::Bus;
use crate::cpu::{Cpu, Stop};
use crate::firmware::Firmware;
use crate::symbols::{Profile, MAIN_LOAD};

/// Where the OS finds its own container in the SPI flash: the updater writes
/// it there and the OS's flash reads (section tables, the panel MCU image)
/// look for it there.
pub const FLASH_SLOT: usize = 0x8_0000;
pub const FLASH_SIZE: usize = 16 << 20;
pub const INITIAL_SP: u32 = 0x4080_0000;
pub const DEFAULT_VBR: u32 = 0x4000_0000;
/// The argument the bootstrap passes MAIN OS when a panel answered: bit 20
/// always, bit 18 from the bootstrap's own check (reference FINDINGS, "The
/// boot flags"). The OS stores it at 0x401F55C0 and tests it in more than
/// twenty places; 0x40|0x20 would mean "no panel" and park the OS.
pub const BOOT_FLAGS: u32 = 0x0014_0000;

#[derive(Default, Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct Stats {
    pub flash_reads: u64,
    pub idle_hits: u64,
    pub idle_skipped: u64,
    pub tasks: Vec<(u32, u32, u32)>,
    pub frames: u64,
    pub intro_done_at: Option<u64>,
    pub abort_at: Option<(u64, u32)>,
    pub mainloop: u64,
}

pub struct Options {
    /// Treat an idle spin as wait-for-interrupt: jump the clock to the next
    /// event instead of executing the spin.
    pub fast_idle: bool,
}

impl Default for Options {
    fn default() -> Self {
        Options { fast_idle: true }
    }
}

pub struct Machine {
    pub cpu: Cpu,
    pub prof: Profile,
    pub flash: Vec<u8>,
    pub stats: Stats,
    pub opts: Options,
    /// The last complete frame the firmware handed to the panel.
    pub frame: Option<Vec<u8>>,
    pub frame_seq: u64,
    pub log: Vec<String>,
    /// Addresses at which `run_until` stops and returns `Stop::Hook`.
    pub breakpoints: Vec<u32>,
    /// Hit counters: address -> count.
    pub counters: Vec<(u32, u64)>,
}

impl Machine {
    pub fn new(fw: &Firmware, ddr_mb: usize) -> Result<Machine, String> {
        let os = fw.main_os().map_err(|e| e.to_string())?;
        if os.entry.dest != MAIN_LOAD {
            return Err(format!("MAIN OS loads at 0x{:08x}, expected 0x{MAIN_LOAD:08x}", os.entry.dest));
        }
        let prof = Profile::for_image(&os.data)?;
        let mut bus = Bus::new(ddr_mb);
        bus.poke_bytes(MAIN_LOAD, &os.data);
        if std::env::var_os("DTEMU_NO_ICACHE").is_none() {
            bus.icache_enable(os.data.len() as u32);
        }
        let mut cpu = Cpu::new(bus);
        // The block compiler is opt-in while it is being built out.
        if std::env::var_os("DTEMU_JIT").is_some() {
            cpu.jit = Some(Box::new(crate::jit::Jit::new(cpu.bus.icache_flushes)));
        }
        cpu.pc = prof.entry;
        cpu.sr = 0x2700;
        // The OS entry reads its argument at 4(a7).
        cpu.a[7] = INITIAL_SP - 8;
        cpu.bus.poke32(INITIAL_SP - 4, BOOT_FLAGS);
        cpu.vbr = DEFAULT_VBR;

        let mut flash = vec![0u8; FLASH_SIZE];
        let c = &fw.container;
        flash[FLASH_SLOT..FLASH_SLOT + c.len()].copy_from_slice(c);

        let mut m = Machine {
            cpu,
            prof,
            flash,
            stats: Stats::default(),
            opts: Options::default(),
            frame: None,
            frame_seq: 0,
            log: Vec::new(),
            breakpoints: Vec::new(),
            counters: Vec::new(),
        };
        m.install_hooks();
        Ok(m)
    }

    fn install_hooks(&mut self) {
        let p = self.prof.clone();
        for a in [p.flash_read, p.task_create, p.intro_done, p.panel_diff, p.abort_loop, p.mainloop] {
            self.cpu.set_hook(a, true);
        }
        for &a in &p.idle_spins {
            self.cpu.set_hook(a, true);
        }
    }

    pub fn add_counter(&mut self, pc: u32) {
        self.counters.push((pc, 0));
        self.cpu.set_hook(pc, true);
    }

    pub fn add_breakpoint(&mut self, pc: u32) {
        self.breakpoints.push(pc);
        self.cpu.set_hook(pc, true);
    }

    pub fn now(&self) -> u64 {
        self.cpu.icount()
    }

    fn note(&mut self, s: String) {
        if self.log.len() < 10_000 {
            self.log.push(format!("[{:>12}] {s}", self.now()));
        }
    }

    /// Handle a hooked PC. -> true to let the instruction there execute.
    fn on_hook(&mut self, pc: u32, until: u64) -> bool {
        let p = &self.prof;
        if pc == p.flash_read {
            // flash_read(offset, length, dest) -> 0, served from the image.
            let sp = self.cpu.a[7];
            let bus = &mut self.cpu.bus;
            let ret = bus.peek32(sp);
            let off = bus.peek32(sp + 4) as usize;
            let len = bus.peek32(sp + 8) as usize;
            let dest = bus.peek32(sp + 12);
            if len != 0 && dest != 0 && off + len <= self.flash.len() {
                let data = self.flash[off..off + len].to_vec();
                self.cpu.bus.poke_bytes(dest, &data);
            }
            self.stats.flash_reads += 1;
            self.cpu.d[0] = 0;
            self.cpu.a[7] = sp + 4;
            self.cpu.pc = ret;
            return false;
        }
        if p.idle_spins.contains(&pc) {
            self.stats.idle_hits += 1;
            if self.opts.fast_idle {
                // Nothing can change until an interrupt: skip to the next
                // event (or the end of the budget), as a WFI would.
                let io = &mut self.cpu.bus.io;
                let target = io.deadline.min(until);
                if target > io.now {
                    self.stats.idle_skipped += target - io.now;
                    io.now = target;
                }
            }
            return true;
        }
        if pc == p.task_create {
            let sp = self.cpu.a[7];
            let b = &self.cpu.bus;
            let (tcb, entry, prio) = (b.peek32(sp + 4), b.peek32(sp + 8), b.peek32(sp + 12));
            self.stats.tasks.push((entry, prio, tcb));
            self.note(format!("task_create entry=0x{entry:08x} prio={prio} tcb=0x{tcb:08x}"));
            return true;
        }
        if pc == p.intro_done {
            if self.stats.intro_done_at.is_none() {
                self.stats.intro_done_at = Some(self.now());
                self.note("intro done".into());
            }
            return true;
        }
        if pc == p.panel_diff {
            let ptr = self.cpu.bus.peek32(p.fb_front);
            if (0x4000_0000..0x5000_0000).contains(&ptr) {
                self.frame = Some(self.cpu.bus.peek_bytes(ptr, 1024));
                self.frame_seq += 1;
                self.stats.frames += 1;
            }
            return true;
        }
        if pc == p.abort_loop {
            if self.stats.abort_at.is_none() {
                self.stats.abort_at = Some((self.now(), self.cpu.bus.peek32(self.cpu.a[7])));
                let caller = self.cpu.bus.peek32(self.cpu.a[7]);
                self.note(format!("ABORT: reached the abort loop, return address 0x{caller:08x}"));
            }
            return true;
        }
        if pc == p.mainloop {
            self.stats.mainloop += 1;
            return true;
        }
        true
    }

    /// Run until the clock reaches `until` or something stops the machine.
    pub fn run_until(&mut self, until: u64) -> Stop {
        self.cpu.bus.io.service();
        loop {
            match self.cpu.run(until) {
                Stop::Budget => return Stop::Budget,
                Stop::Event => {
                    self.cpu.bus.io.service();
                    self.cpu.bus.dma_service();
                }
                Stop::Stopped => {
                    // STOP: sleep until the next event.
                    let io = &mut self.cpu.bus.io;
                    let target = io.deadline.min(until);
                    if target > io.now {
                        io.now = target;
                    }
                    if io.now >= until {
                        return Stop::Budget;
                    }
                    self.cpu.bus.io.service();
                    self.cpu.bus.dma_service();
                    // An interrupt at or below the mask cannot wake it; the
                    // CPU takes any that can in `run`.
                    if self.cpu.bus.io.deadline == u64::MAX && self.cpu.bus.io.irq_level <= self.cpu.ipl() {
                        return Stop::Stopped;
                    }
                }
                Stop::Hook(pc) => {
                    if self.breakpoints.contains(&pc) {
                        self.breakpoints.retain(|&b| b != pc);
                        self.cpu.skip_hook = Some(pc);
                        return Stop::Hook(pc);
                    }
                    for c in self.counters.iter_mut() {
                        if c.0 == pc {
                            c.1 += 1;
                        }
                    }
                    if self.on_hook(pc, until) {
                        self.cpu.skip_hook = Some(pc);
                    }
                }
                f @ Stop::Fault(..) => return f,
            }
        }
    }

    /// Every task seen created: (entry, prio, tcb, parked pc, stack words).
    pub fn task_report(&self) -> String {
        let b = &self.cpu.bus;
        let cur = b.peek32(self.prof.current_tcb);
        let mut out = String::new();
        for &(entry, prio, tcb) in &self.stats.tasks {
            let sp = b.peek32(tcb + 0x48);
            let pc = b.peek32(sp + 4);
            let words: Vec<String> = (2..14).map(|i| format!("{:08x}", b.peek32(sp + 4 * i))).collect();
            out += &format!(
                "  prio {prio:>2} entry {entry:08x} tcb {tcb:08x}{} sp {sp:08x} pc {pc:08x} | {}\n",
                if tcb == cur { " (running)" } else { "" },
                words.join(" ")
            );
        }
        out
    }

    /// Attach a +Drive card image (read now; `flush_card` writes it back).
    /// A missing or empty image becomes a fresh card with the sample area
    /// formatted, as a factory unit's is (the OS never creates it itself).
    /// -> whether the card is fresh.
    pub fn attach_card(&mut self, path: &std::path::Path) -> std::io::Result<bool> {
        let mut card = crate::esdhc::Card::open(path)?;
        let fresh = card.sectors.is_empty();
        if fresh {
            crate::ekfs::format(&mut card);
        }
        self.cpu.bus.io.esdhc.card = card;
        Ok(fresh)
    }

    pub fn flush_card(&mut self) -> std::io::Result<()> {
        self.cpu.bus.io.esdhc.card.flush()
    }

    /// The firmware's own "+Drive mounted" flag.
    pub fn drive_mounted(&self) -> bool {
        self.cpu.bus.peek32(self.prof.mounted) == 1
    }

    /// Press or release a key by control code (see `panel::key_name`).
    pub fn key(&mut self, code: u8, down: bool) {
        if let Some(msg) = self.cpu.bus.io.panel.key(code, down) {
            self.panel_send(&msg);
        }
    }

    /// Turn an encoder (0..8: A..H, then LEVEL/DATA) by `delta` counts.
    pub fn turn(&mut self, enc: u8, delta: i8) {
        let msg = crate::panel::PanelMcu::encoder(enc, delta);
        self.panel_send(&msg);
    }

    pub fn release_all_keys(&mut self) {
        let msg = self.cpu.bus.io.panel.release_all();
        if !msg.is_empty() {
            self.panel_send(&msg);
        }
    }

    /// Bytes from the panel MCU to the ColdFire, on UART8.
    pub fn panel_send(&mut self, bytes: &[u8]) {
        self.cpu.bus.io.uart_receive(8, bytes);
        self.cpu.bus.run_dma();
    }

    /// Panel frame as rows of pixels (true = lit), y = 0 at the top.
    pub fn frame_pixels(buf: &[u8]) -> Vec<Vec<bool>> {
        // SSD1306-style pages, page 0 at the bottom (a remapped COM scan):
        // byte = page + 8 * column, bit n = row 8 * (7 - page) + n.
        let mut rows = vec![vec![false; 128]; 64];
        for (y, row) in rows.iter_mut().enumerate() {
            for (x, px) in row.iter_mut().enumerate() {
                let b = buf[(7 - y / 8) + 8 * x];
                *px = (b >> (y % 8)) & 1 != 0;
            }
        }
        rows
    }

    pub fn frame_ascii(buf: &[u8]) -> String {
        let rows = Self::frame_pixels(buf);
        let mut s = String::new();
        // Two pixel rows per text line.
        for y in (0..64).step_by(2) {
            for (&a, &b) in rows[y].iter().zip(&rows[y + 1]) {
                s.push(match (a, b) {
                    (true, true) => '█',
                    (true, false) => '▀',
                    (false, true) => '▄',
                    _ => ' ',
                });
            }
            s.push('\n');
        }
        s
    }

    pub fn lit(buf: &[u8]) -> usize {
        buf.iter().map(|b| b.count_ones() as usize).sum()
    }
}
