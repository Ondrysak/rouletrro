//! Peripheral models and their dispatch.
//!
//! Time is the CPU's instruction count (`now`). Peripheral clocks convert
//! through `ips`, emulated instructions per emulated second, against the
//! 132 MHz bus clock the reference established from three independent PIT
//! rates landing on round numbers (50 Hz, 59.998 Hz, 14.9996 Hz).
//!
//! Interrupt lines are level-sensitive, as on the silicon: a source stays
//! asserted until the firmware's handler clears the flag that raised it.

use std::collections::{HashMap, VecDeque};

pub const F_BUS: f64 = 132_000_000.0;

pub const INTC_BASE: [u32; 3] = [0xFC04_8000, 0xFC04_C000, 0xFC05_0000];
pub const PIT_BASE: u32 = 0xFC08_0000;
pub const DTIM_BASE: u32 = 0xFC07_0000;
pub const EDMA_BASE: u32 = 0xFC04_4000;
pub const TCD_BASE: u32 = 0xFC04_5000;
pub const UART8_BASE: u32 = 0xEC07_0000;
pub const UART9_BASE: u32 = 0xEC07_4000;
pub const GPIO_BASE: u32 = 0xEC09_4000;

/// An interrupt source: (controller, source number).
pub type Src = (usize, u32);

pub const SRC_PIT: [Src; 4] = [(2, 13), (2, 14), (2, 15), (2, 16)];
pub const SRC_DTIM: [Src; 4] = [(0, 32), (0, 33), (0, 34), (0, 35)];
pub const SRC_UART8: Src = (1, 52);
pub const SRC_UART9: Src = (1, 53);
pub const SRC_ESDHC: Src = (2, 31);

// ---------------------------------------------------------------------------

#[derive(Clone)]
pub struct Intc {
    pub imr: u64,
    pub frc: u64,
    pub icr: [u8; 64],
    pub iconfig: u16,
    /// Device lines, recomputed by `Io::update_irq`.
    pub lines: u64,
}

impl Intc {
    fn new() -> Intc {
        Intc { imr: u64::MAX, frc: 0, icr: [0; 64], iconfig: 0, lines: 0 }
    }
    fn pending(&self) -> u64 {
        self.lines | self.frc
    }
}

#[derive(Clone, Default)]
pub struct Pit {
    pub pcsr: u16,
    pub pmr: u16,
    /// Instruction time the counter was (re)loaded, and with what.
    t0: f64,
    load: u32,
    /// When PIF next sets, while enabled.
    pub next: Option<f64>,
    pub fired: u64,
}

const PIT_EN: u16 = 0x01;
const PIT_RLD: u16 = 0x02;
const PIT_PIF: u16 = 0x04;
const PIT_PIE: u16 = 0x08;
const PIT_OVW: u16 = 0x10;

#[derive(Clone, Default)]
pub struct Dtim {
    pub dtmr: u16,
    pub dtxmr: u8,
    pub dter: u8,
    pub dtrr: u32,
    pub dtcr: u32,
    t0: f64,
    pub next: Option<f64>,
    pub fired: u64,
}

#[derive(Clone, Default)]
pub struct Uart {
    pub umr: [u8; 2],
    pub umr_ptr: usize,
    pub uimr: u8,
    pub ucsr: u8,
    pub rx: VecDeque<u8>,
    pub tx: Vec<u8>,
    pub rx_enabled: bool,
    pub tx_enabled: bool,
}

/// eDMA controller registers; transfers run in `crate::edma` on the bus.
#[derive(Clone)]
pub struct Edma {
    pub cr: u32,
    pub erq: u64,
    pub eei: u64,
    pub int: u64,
    pub err: u64,
    pub tcd: Vec<u8>,
    /// Channels with a software start or hardware request waiting.
    pub kick: u64,
    /// Channel -> instruction time before which it may not run again.
    pub busy_until: [u64; 64],
    /// Channels that asked while busy, re-offered by the service loop.
    pub deferred: u64,
    pub majors: [u64; 64],
}

pub struct Io {
    pub now: u64,
    /// The CPU stops when `now` reaches this; lowered by any event.
    pub deadline: u64,
    pub ips: f64,
    pub irq_level: u8,
    pub intc: [Intc; 3],
    pub pit: [Pit; 4],
    pub dtim: [Dtim; 4],
    pub uart8: Uart,
    pub panel: crate::panel::PanelMcu,
    pub uart9: Uart,
    pub edma: Edma,
    pub esdhc: crate::esdhc::Esdhc,
    /// Port D bit 4 drives port C bit 3 on the board (the SD gate).
    pub gpio_d4: bool,
    /// Registers that read as a fixed value (status bits nothing else sets).
    pub forced: HashMap<u32, u32>,
    /// Ignore IMR masking, as the reference emulator effectively did.
    pub ignore_masks: bool,
    /// PIT channels whose interrupts are held off (the intro policy).
    pub pit_hold: u8,
    pub irq_taken: [u64; 256],
    /// Bytes per UART character time, in instructions.
    pub uart_byte_instr: u64,
}

impl Default for Io {
    fn default() -> Self {
        Self::new()
    }
}

impl Io {
    pub fn new() -> Io {
        let mut forced = HashMap::new();
        // DSPI0 SR: TCF|EOQF|RFDF set, RXCTR nonzero -- transfers complete at
        // once. From the reference's boot harness.
        forced.insert(0xFC05_C02C, 0x1000_00F0);
        // A serial block's TX-done status bit.
        forced.insert(0xEC03_802C, 0x8000_0000);
        Io {
            now: 0,
            deadline: u64::MAX,
            ips: 100_000_000.0,
            irq_level: 0,
            intc: [Intc::new(), Intc::new(), Intc::new()],
            pit: Default::default(),
            dtim: Default::default(),
            uart8: Uart::default(),
            panel: Default::default(),
            uart9: Uart::default(),
            edma: Edma {
                cr: 0,
                erq: 0,
                eei: 0,
                int: 0,
                err: 0,
                tcd: vec![0; 64 * 32],
                kick: 0,
                busy_until: [0; 64],
                deferred: 0,
                majors: [0; 64],
            },
            esdhc: crate::esdhc::Esdhc::new(crate::esdhc::Card::new()),
            gpio_d4: false,
            forced,
            ignore_masks: false,
            pit_hold: 0,
            irq_taken: [0; 256],
            uart_byte_instr: 2000,
        }
    }

    fn cyc(&self, cycles: f64) -> f64 {
        cycles / F_BUS * self.ips
    }

    fn lower_deadline(&mut self, t: f64) {
        let t = if t < self.now as f64 { self.now } else { t.ceil() as u64 };
        if t < self.deadline {
            self.deadline = t;
        }
    }

    // -- interrupt controller ----------------------------------------------

    fn set_line(&mut self, s: Src, on: bool) {
        let bit = 1u64 << s.1;
        if on {
            self.intc[s.0].lines |= bit;
        } else {
            self.intc[s.0].lines &= !bit;
        }
    }

    /// Recompute every device line and the level the CPU sees.
    pub fn update_irq(&mut self) {
        for ch in 0..4 {
            let p = &self.pit[ch];
            let on = p.pcsr & PIT_PIF != 0 && p.pcsr & PIT_PIE != 0 && self.pit_hold & (1 << ch) == 0;
            self.set_line(SRC_PIT[ch], on);
            let d = &self.dtim[ch];
            let on = (d.dter & 0x02 != 0 && d.dtmr & 0x10 != 0 && d.dtxmr & 0x80 == 0)
                || (d.dter & 0x01 != 0 && d.dtmr & 0xC0 != 0);
            self.set_line(SRC_DTIM[ch], on);
        }
        let u8l = self.uart_isr(8) & self.uart8.uimr != 0;
        self.set_line(SRC_UART8, u8l);
        let u9l = self.uart_isr(9) & self.uart9.uimr != 0;
        self.set_line(SRC_UART9, u9l);
        let sd = self.esdhc.line();
        self.set_line(SRC_ESDHC, sd);
        // eDMA: channels 0-15 on INTC0 sources 8-23, 16-55 on INTC1 sources
        // 8-47 (source = channel - 8), 56-63 OR-ed onto INTC2 source 0.
        let int = self.edma.int;
        let lo = (int & 0xFFFF) << 8;
        self.intc[0].lines = (self.intc[0].lines & !(0xFFFFu64 << 8)) | lo;
        let mid = ((int >> 16) & ((1u64 << 40) - 1)) << 8;
        self.intc[1].lines = (self.intc[1].lines & !(((1u64 << 40) - 1) << 8)) | mid;
        self.set_line((2, 0), int >> 56 != 0);

        let mut best = 0u8;
        for c in &self.intc {
            let mut p = c.pending();
            if !self.ignore_masks {
                p &= !c.imr;
            }
            while p != 0 {
                let s = p.trailing_zeros();
                p &= p - 1;
                let l = c.icr[s as usize] & 7;
                if l > best {
                    best = l;
                }
            }
        }
        self.irq_level = best;
    }

    /// The CPU takes an interrupt: -> (vector, level) of the highest pending
    /// source above `ipl`. Ties go to the highest source number.
    pub fn ack_irq(&mut self, ipl: u8) -> Option<(u8, u8)> {
        let mut best: Option<(u8, usize, u32)> = None;
        for (ci, c) in self.intc.iter().enumerate() {
            let mut p = c.pending();
            if !self.ignore_masks {
                p &= !c.imr;
            }
            while p != 0 {
                let s = 63 - p.leading_zeros();
                p &= !(1u64 << s);
                let l = c.icr[s as usize] & 7;
                if l == 0 || (l <= ipl && l != 7) {
                    continue;
                }
                let better = match best {
                    None => true,
                    Some((bl, bc, bs)) => l > bl || (l == bl && (ci, s) > (bc, bs)),
                };
                if better {
                    best = Some((l, ci, s));
                }
            }
        }
        let (l, c, s) = best?;
        let vec = (64 + 64 * c as u32 + s) as u8;
        self.irq_taken[vec as usize] += 1;
        Some((vec, l))
    }

    fn intc_read(&mut self, c: usize, off: u32, size: u32) -> Option<u32> {
        let i = &self.intc[c];
        let p = i.pending();
        let v32 = |w: u64| w as u32;
        let r = match (off, size) {
            (0x00, 4) => v32(p >> 32),
            (0x04, 4) => v32(p),
            (0x08, 4) => v32(i.imr >> 32),
            (0x0C, 4) => v32(i.imr),
            (0x10, 4) => v32(i.frc >> 32),
            (0x14, 4) => v32(i.frc),
            (0x18, 2) => i.iconfig as u32,
            (0x40..=0x7F, 1) => i.icr[(off - 0x40) as usize] as u32,
            (0x1C..=0x1F, 1) => 0,
            _ => return None,
        };
        Some(r)
    }

    fn intc_write(&mut self, c: usize, off: u32, size: u32, v: u32) -> bool {
        let i = &mut self.intc[c];
        match (off, size) {
            (0x08, 4) => i.imr = (i.imr & 0xFFFF_FFFF) | ((v as u64) << 32),
            (0x0C, 4) => i.imr = (i.imr & !0xFFFF_FFFF) | v as u64,
            (0x10, 4) => i.frc = (i.frc & 0xFFFF_FFFF) | ((v as u64) << 32),
            (0x14, 4) => i.frc = (i.frc & !0xFFFF_FFFF) | v as u64,
            (0x18, 2) => i.iconfig = v as u16,
            // SIMR / CIMR: an index, not a mask; bit 6 means every source.
            (0x1C, 1) => {
                if v & 0x40 != 0 {
                    i.imr = u64::MAX;
                } else {
                    i.imr |= 1u64 << (v & 0x3F);
                }
            }
            (0x1D, 1) => {
                if v & 0x40 != 0 {
                    i.imr = 0;
                } else {
                    i.imr &= !(1u64 << (v & 0x3F));
                }
            }
            (0x1E..=0x1F, 1) => {}
            (0x40..=0x7F, 1) => i.icr[(off - 0x40) as usize] = v as u8,
            (0x40..=0x7F, 2) => {
                i.icr[(off - 0x40) as usize] = (v >> 8) as u8;
                if off < 0x7F {
                    i.icr[(off - 0x3F) as usize] = v as u8;
                }
            }
            (0x40..=0x7F, 4) => {
                for k in 0..4 {
                    if off + k <= 0x7F {
                        i.icr[(off - 0x40 + k) as usize] = (v >> (24 - 8 * k)) as u8;
                    }
                }
            }
            (0x00..=0x07, _) => {}
            // Byte and word writes into the 32-bit registers.
            (0x08..=0x17, _) => {
                let reg = off & !3;
                let cur = self.intc_read(c, reg, 4).unwrap_or(0);
                let shift = 8 * (4 - size - (off & 3));
                let m = if size == 4 { u32::MAX } else { ((1u32 << (8 * size)) - 1) << shift };
                let nv = (cur & !m) | ((v << shift) & m);
                return self.intc_write(c, reg, 4, nv);
            }
            _ => return false,
        }
        true
    }

    // -- PIT ---------------------------------------------------------------

    fn pit_tick(&self, ch: usize) -> f64 {
        let pre = 1u32 << (((self.pit[ch].pcsr >> 8) & 0xF) + 1);
        self.cyc(pre as f64)
    }

    fn pit_restart(&mut self, ch: usize, load: u32) {
        let now = self.now as f64;
        let tick = self.pit_tick(ch);
        let p = &mut self.pit[ch];
        p.t0 = now;
        p.load = load;
        if p.pcsr & PIT_EN != 0 {
            let t = now + (load as f64 + 1.0) * tick;
            p.next = Some(t);
            self.lower_deadline(t);
        } else {
            p.next = None;
        }
    }

    fn pit_counter(&self, ch: usize) -> u32 {
        let p = &self.pit[ch];
        match p.next {
            Some(t) => {
                let left = ((t - self.now as f64) / self.pit_tick(ch)).floor();
                (left.max(0.0) as u32).saturating_sub(0).min(0xFFFF)
            }
            None => p.load & 0xFFFF,
        }
    }

    fn pit_read(&mut self, ch: usize, off: u32, size: u32) -> Option<u32> {
        let p = &self.pit[ch];
        Some(match (off, size) {
            (0, 2) => p.pcsr as u32,
            (0, 4) => (p.pcsr as u32) << 16 | p.pmr as u32,
            (1, 1) => p.pcsr as u32 & 0xFF,
            (0, 1) => (p.pcsr >> 8) as u32,
            (2, 2) => p.pmr as u32,
            (4, 2) => self.pit_counter(ch),
            (4, 4) => self.pit_counter(ch) << 16,
            _ => return None,
        })
    }

    fn pit_write(&mut self, ch: usize, off: u32, size: u32, v: u32) -> bool {
        match (off, size) {
            (0, 4) => {
                self.pit_write(ch, 0, 2, v >> 16);
                self.pit_write(ch, 2, 2, v & 0xFFFF);
            }
            (0, 2) | (1, 1) | (0, 1) => {
                let v = match (off, size) {
                    (1, 1) => (self.pit[ch].pcsr & 0xFF00) as u32 | (v & 0xFF),
                    (0, 1) => (self.pit[ch].pcsr & 0x00FF) as u32 | ((v & 0xFF) << 8),
                    _ => v,
                } as u16;
                let old = self.pit[ch].pcsr;
                // PIF is write-one-to-clear.
                let pif = if v & PIT_PIF != 0 { 0 } else { old & PIT_PIF };
                let new = (v & !PIT_PIF) | pif;
                self.pit[ch].pcsr = new;
                let restart = (new & PIT_EN != 0 && old & PIT_EN == 0) || ((new ^ old) & 0x0F00 != 0);
                if new & PIT_EN == 0 {
                    self.pit[ch].next = None;
                } else if restart {
                    let pmr = self.pit[ch].pmr as u32;
                    self.pit_restart(ch, pmr);
                }
            }
            (2, 2) => {
                self.pit[ch].pmr = v as u16;
                let p = &self.pit[ch];
                if p.pcsr & PIT_OVW != 0 || p.pcsr & PIT_EN == 0 {
                    self.pit_restart(ch, v & 0xFFFF);
                }
            }
            _ => return false,
        }
        true
    }

    // -- DMA timers ----------------------------------------------------------

    fn dtim_tick(&self, ch: usize) -> Option<f64> {
        let d = &self.dtim[ch];
        if d.dtmr & 1 == 0 {
            return None;
        }
        let div = match (d.dtmr >> 1) & 3 {
            1 => 1.0,
            2 => 16.0,
            _ => return None,
        };
        Some(self.cyc(((d.dtmr >> 8) as f64 + 1.0) * div))
    }

    fn dtim_schedule(&mut self, ch: usize) {
        let tick = self.dtim_tick(ch);
        let d = &mut self.dtim[ch];
        d.next = tick.map(|t| d.t0 + (d.dtrr as f64 + 1.0) * t);
        if let Some(t) = d.next {
            self.lower_deadline(t);
        }
    }

    fn dtim_counter(&self, ch: usize) -> u32 {
        match self.dtim_tick(ch) {
            None => 0,
            Some(t) => {
                let d = &self.dtim[ch];
                let n = ((self.now as f64 - d.t0) / t).max(0.0) as u64;
                if d.dtmr & 0x08 != 0 {
                    (n % (d.dtrr as u64 + 1)) as u32
                } else {
                    n as u32
                }
            }
        }
    }

    fn dtim_read(&mut self, ch: usize, off: u32, size: u32) -> Option<u32> {
        let d = &self.dtim[ch];
        Some(match (off, size) {
            (0, 2) => d.dtmr as u32,
            (0, 4) => (d.dtmr as u32) << 16 | (d.dtxmr as u32) << 8 | d.dter as u32,
            (2, 1) => d.dtxmr as u32,
            (3, 1) => d.dter as u32,
            (2, 2) => (d.dtxmr as u32) << 8 | d.dter as u32,
            (4, 4) => d.dtrr,
            (8, 4) => d.dtcr,
            (0xC, 4) => self.dtim_counter(ch),
            _ => return None,
        })
    }

    fn dtim_write(&mut self, ch: usize, off: u32, size: u32, v: u32) -> bool {
        let now = self.now as f64;
        match (off, size) {
            (0, 4) => {
                self.dtim_write(ch, 0, 2, v >> 16);
                self.dtim_write(ch, 2, 1, (v >> 8) & 0xFF);
                self.dtim_write(ch, 3, 1, v & 0xFF);
            }
            (0, 2) => {
                let old = self.dtim[ch].dtmr;
                let new = v as u16;
                self.dtim[ch].dtmr = new;
                if new & 1 == 0 || old & 1 == 0 || (old ^ new) & 0xFF06 != 0 {
                    self.dtim[ch].t0 = now;
                }
                self.dtim_schedule(ch);
            }
            (2, 1) => self.dtim[ch].dtxmr = v as u8,
            (3, 1) => self.dtim[ch].dter &= !(v as u8),
            (2, 2) => {
                self.dtim[ch].dtxmr = (v >> 8) as u8;
                self.dtim[ch].dter &= !(v as u8);
            }
            (4, 4) => {
                self.dtim[ch].dtrr = v;
                self.dtim_schedule(ch);
            }
            (8, 4) => self.dtim[ch].dtcr = v,
            (0xC, 4) => {
                self.dtim[ch].t0 = now;
                self.dtim_schedule(ch);
            }
            _ => return false,
        }
        true
    }

    // -- UART ----------------------------------------------------------------

    fn uart(&mut self, n: u32) -> &mut Uart {
        if n == 8 {
            &mut self.uart8
        } else {
            &mut self.uart9
        }
    }

    fn uart_usr(&self, n: u32) -> u32 {
        let u = if n == 8 { &self.uart8 } else { &self.uart9 };
        // TXEMP | TXRDY always: transmission completes instantly.
        let mut v = 0x0C;
        if !u.rx.is_empty() {
            v |= 0x01;
        }
        if u.rx.len() >= 4 {
            v |= 0x02;
        }
        v
    }

    fn uart_isr(&self, n: u32) -> u8 {
        let u = if n == 8 { &self.uart8 } else { &self.uart9 };
        let mut v = 0x01; // TXRDY
        if !u.rx.is_empty() {
            v |= 0x02;
        }
        v
    }

    fn uart_read(&mut self, n: u32, off: u32) -> Option<u32> {
        Some(match off {
            0x00 => {
                let u = self.uart(n);
                let v = u.umr[u.umr_ptr];
                u.umr_ptr = 1;
                v as u32
            }
            0x04 => self.uart_usr(n),
            0x0C => {
                let v = self.uart(n).rx.pop_front().unwrap_or(0);
                self.update_irq();
                v as u32
            }
            0x14 => self.uart_isr(n) as u32,
            0x10 => 0,
            _ => return None,
        })
    }

    fn uart_write(&mut self, n: u32, off: u32, v: u32) -> bool {
        let v = v as u8;
        match off {
            0x00 => {
                let u = self.uart(n);
                let p = u.umr_ptr;
                u.umr[p] = v;
                u.umr_ptr = 1;
            }
            0x04 => self.uart(n).ucsr = v,
            0x08 => {
                let u = self.uart(n);
                match (v >> 4) & 7 {
                    1 => u.umr_ptr = 0,
                    2 => u.rx.clear(),
                    _ => {}
                }
                match v & 3 {
                    1 => u.rx_enabled = true,
                    2 => u.rx_enabled = false,
                    _ => {}
                }
                match (v >> 2) & 3 {
                    1 => u.tx_enabled = true,
                    2 => u.tx_enabled = false,
                    _ => {}
                }
            }
            0x0C => {
                let u = self.uart(n);
                if u.tx.len() < (1 << 16) {
                    u.tx.push(v);
                }
                if n == 8 {
                    let mut reply = Vec::new();
                    self.panel.from_host(v, &mut reply);
                    if !reply.is_empty() {
                        self.uart_receive(8, &reply);
                    }
                }
            }
            0x14 => {
                self.uart(n).uimr = v;
                self.update_irq();
            }
            _ => return false,
        }
        true
    }

    /// Bytes arriving on a UART's receive line.
    pub fn uart_receive(&mut self, n: u32, data: &[u8]) {
        self.uart(n).rx.extend(data.iter().copied());
        // UART8 receive is DMA-driven (channel 34) when the firmware has
        // enabled that channel; the bus drains it.
        if n == 8 && self.edma.erq & (1 << 34) != 0 {
            self.edma.kick |= 1 << 34;
        }
        self.update_irq();
    }

    // -- eDMA registers ------------------------------------------------------

    fn edma_read(&mut self, off: u32, size: u32) -> Option<u32> {
        let e = &self.edma;
        Some(match (off, size) {
            (0x00, 4) => e.cr,
            (0x04, 4) => 0,
            (0x08, 4) => (e.erq >> 32) as u32,
            (0x0C, 4) => e.erq as u32,
            (0x10, 4) => (e.eei >> 32) as u32,
            (0x14, 4) => e.eei as u32,
            (0x18..=0x1F, 1) => 0,
            (0x20, 4) => (e.int >> 32) as u32,
            (0x24, 4) => e.int as u32,
            (0x28, 4) => (e.err >> 32) as u32,
            (0x2C, 4) => e.err as u32,
            (0x30, 4) | (0x34, 4) => 0,
            _ => return None,
        })
    }

    fn edma_write(&mut self, off: u32, size: u32, v: u32) -> bool {
        let e = &mut self.edma;
        let sel = |v: u32| -> u64 {
            if v & 0x40 != 0 {
                u64::MAX
            } else {
                1u64 << (v & 0x3F)
            }
        };
        match (off, size) {
            (0x00, 4) => e.cr = v,
            (0x08, 4) => e.erq = (e.erq & 0xFFFF_FFFF) | ((v as u64) << 32),
            (0x0C, 4) => e.erq = (e.erq & !0xFFFF_FFFF) | v as u64,
            (0x10, 4) => e.eei = (e.eei & 0xFFFF_FFFF) | ((v as u64) << 32),
            (0x14, 4) => e.eei = (e.eei & !0xFFFF_FFFF) | v as u64,
            (0x18, 1) => {
                e.erq |= sel(v);
                let req = self.dma_requesting();
                self.edma.kick |= sel(v) & req;
            }
            (0x19, 1) => self.edma.erq &= !sel(v),
            (0x1A, 1) => self.edma.eei |= sel(v),
            (0x1B, 1) => self.edma.eei &= !sel(v),
            (0x1C, 1) => {
                self.edma.int &= !sel(v);
                self.update_irq();
            }
            (0x1D, 1) => self.edma.err &= !sel(v),
            (0x1E, 1) => {
                // SSRT: software start.
                let m = sel(v);
                for ch in 0..64 {
                    if m & (1 << ch) != 0 {
                        let o = ch * 32 + 0x1E;
                        self.edma.tcd[o + 1] |= 0x01;
                    }
                }
                self.edma.kick |= m;
            }
            (0x1F, 1) => {
                let m = sel(v);
                for ch in 0..64 {
                    if m & (1 << ch) != 0 {
                        self.edma.tcd[ch * 32 + 0x1F] &= !0x80;
                    }
                }
            }
            (0x18, 4) => {
                for k in 0..4 {
                    self.edma_write(0x18 + k, 1, (v >> (24 - 8 * k)) & 0xFF);
                }
            }
            (0x1C, 4) => {
                for k in 0..4 {
                    self.edma_write(0x1C + k, 1, (v >> (24 - 8 * k)) & 0xFF);
                }
            }
            (0x20, 4) => {
                self.edma.int &= !((v as u64) << 32);
                self.update_irq();
            }
            (0x24, 4) => {
                self.edma.int &= !(v as u64);
                self.update_irq();
            }
            (0x28, 4) => self.edma.err &= !((v as u64) << 32),
            (0x2C, 4) => self.edma.err &= !(v as u64),
            (0x100..=0x13F, 1) => {}
            _ => return false,
        }
        true
    }

    /// Channels whose peripheral is asking for service right now.
    pub fn dma_requesting(&self) -> u64 {
        let mut m = 0u64;
        // UART8 TX is always ready; UART8 RX when it has bytes.
        m |= 1 << 35;
        if !self.uart8.rx.is_empty() {
            m |= 1 << 34;
        }
        if self.esdhc.dma_request() {
            m |= 1 << crate::esdhc::DMA_CHAN;
        }
        m
    }

    fn tcd_write(&mut self, off: u32, size: u32, v: u32) {
        let o = off as usize;
        for k in 0..size as usize {
            self.edma.tcd[o + k] = (v >> (8 * (size as usize - 1 - k))) as u8;
        }
        // Writing CSR with START set starts the channel.
        let ch = o / 32;
        let csr_lo = ch * 32 + 0x1F;
        if (o..o + size as usize).contains(&csr_lo) && self.edma.tcd[csr_lo] & 0x01 != 0 {
            self.edma.kick |= 1 << ch;
        }
    }

    fn tcd_read(&self, off: u32, size: u32) -> u32 {
        let o = off as usize;
        let mut v = 0u32;
        for k in 0..size as usize {
            v = v << 8 | self.edma.tcd[o + k] as u32;
        }
        v
    }

    // -- dispatch ------------------------------------------------------------

    /// -> Some(value) when a model answers the read.
    pub fn read(&mut self, a: u32, size: u32, _plain: u32, _pc: u32, _ddr: &mut [u8]) -> Option<u32> {
        if let Some(&v) = self.forced.get(&(a & !3)) {
            let sh = 8 * (4 - size - (a & 3));
            let m = if size == 4 { u32::MAX } else { (1u32 << (8 * size)) - 1 };
            return Some((v >> sh) & m);
        }
        match a & 0xFFFF_C000 {
            0xFC04_8000 => return self.intc_read(0, a & 0x3FFF, size),
            0xFC04_C000 => return self.intc_read(1, a & 0x3FFF, size),
            0xFC05_0000 => return self.intc_read(2, a & 0x3FFF, size),
            0xFC08_0000 | 0xFC08_4000 | 0xFC08_8000 | 0xFC08_C000 => {
                return self.pit_read(((a >> 14) & 3) as usize, a & 0x3FFF, size)
            }
            0xFC07_0000 | 0xFC07_4000 | 0xFC07_8000 | 0xFC07_C000 => {
                return self.dtim_read(((a >> 14) & 3) as usize, a & 0x3FFF, size)
            }
            0xFC04_4000 => {
                let off = a & 0x3FFF;
                if (0x1000..0x1800).contains(&off) {
                    return Some(self.tcd_read(off - 0x1000, size));
                }
                return self.edma_read(off, size);
            }
            0xFC0C_C000 => {
                let r = self.esdhc.read(a & 0x3FFF, size);
                if r.is_some() {
                    // A DATPORT read may complete the transfer.
                    self.update_irq();
                }
                return r;
            }
            0xEC07_0000 if size == 1 && a & 3 == 0 => return self.uart_read(8, a & 0x3F),
            0xEC07_4000 if size == 1 && a & 3 == 0 => return self.uart_read(9, a & 0x3F),
            0xEC09_4000 => {
                // PPDSDR_C: bit 3 follows the port D bit 4 the board loops back.
                if a == 0xEC09_401A && size == 1 {
                    return Some(if self.gpio_d4 { 0x08 } else { 0 } | (_plain & !0x08));
                }
            }
            _ => {}
        }
        None
    }

    /// -> true when a model consumed the write.
    pub fn write(&mut self, a: u32, size: u32, v: u32, _pc: u32, _ddr: &mut [u8]) -> bool {
        let r = match a & 0xFFFF_C000 {
            0xFC04_8000 => self.intc_write(0, a & 0x3FFF, size, v),
            0xFC04_C000 => self.intc_write(1, a & 0x3FFF, size, v),
            0xFC05_0000 => self.intc_write(2, a & 0x3FFF, size, v),
            0xFC08_0000 | 0xFC08_4000 | 0xFC08_8000 | 0xFC08_C000 => {
                self.pit_write(((a >> 14) & 3) as usize, a & 0x3FFF, size, v)
            }
            0xFC07_0000 | 0xFC07_4000 | 0xFC07_8000 | 0xFC07_C000 => {
                self.dtim_write(((a >> 14) & 3) as usize, a & 0x3FFF, size, v)
            }
            0xFC04_4000 => {
                let off = a & 0x3FFF;
                if (0x1000..0x1800).contains(&off) {
                    self.tcd_write(off - 0x1000, size, v);
                    true
                } else {
                    self.edma_write(off, size, v)
                }
            }
            0xFC0C_C000 => {
                let r = self.esdhc.write(a & 0x3FFF, size, v);
                // Issuing a data command asks channel 59 for service.
                if self.edma.erq & (1 << crate::esdhc::DMA_CHAN) != 0 && self.esdhc.dma_request() {
                    self.edma.kick |= 1 << crate::esdhc::DMA_CHAN;
                }
                r
            }
            0xEC07_0000 if size == 1 && a & 3 == 0 => self.uart_write(8, a & 0x3F, v),
            0xEC07_4000 if size == 1 && a & 3 == 0 => self.uart_write(9, a & 0x3F, v),
            0xEC09_4000 => {
                // PPDSDR_D: writing 1 sets; PCLRR_D: writing 0 clears.
                if a == 0xEC09_401B && size == 1 && v & 0x10 != 0 {
                    self.gpio_d4 = true;
                }
                if a == 0xEC09_4027 && size == 1 && v & 0x10 == 0 {
                    self.gpio_d4 = false;
                }
                false
            }
            _ => false,
        };
        if r {
            self.update_irq();
        }
        r
    }

    /// Advance timers to `now`; set flags for everything due. Then set
    /// `deadline` to the next event.
    pub fn service(&mut self) {
        let now = self.now as f64;
        let mut next = f64::INFINITY;
        for ch in 0..4 {
            if let Some(t) = self.pit[ch].next {
                if now >= t {
                    let tick = self.pit_tick(ch);
                    let p = &mut self.pit[ch];
                    p.pcsr |= PIT_PIF;
                    p.fired += 1;
                    let reload = if p.pcsr & PIT_RLD != 0 { p.pmr as u32 } else { 0xFFFF };
                    p.load = reload;
                    let mut nt = t + (reload as f64 + 1.0) * tick;
                    if nt <= now {
                        nt = now + (reload as f64 + 1.0) * tick;
                    }
                    p.t0 = t;
                    p.next = Some(nt);
                }
                next = next.min(self.pit[ch].next.unwrap());
            }
            if let Some(t) = self.dtim[ch].next {
                if now >= t {
                    let tick = self.dtim_tick(ch).unwrap_or(1.0);
                    let d = &mut self.dtim[ch];
                    d.dter |= 0x02;
                    d.fired += 1;
                    let period = (d.dtrr as f64 + 1.0) * tick;
                    if d.dtmr & 0x08 != 0 {
                        let mut nt = t + period;
                        if nt <= now {
                            nt = now + period;
                        }
                        d.t0 = nt - period;
                        d.next = Some(nt);
                    } else {
                        // Free run: the counter goes round all 32 bits.
                        d.next = Some(t + 4_294_967_296.0 * tick);
                    }
                }
                next = next.min(self.dtim[ch].next.unwrap());
            }
        }
        for ch in 0..64 {
            let b = self.edma.busy_until[ch];
            if b > self.now {
                next = next.min(b as f64);
            }
        }
        self.update_irq();
        self.deadline = if next.is_finite() { next.ceil().max(now + 1.0) as u64 } else { u64::MAX };
    }
}
