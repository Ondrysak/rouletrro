//! ColdFire V4e integer core (ISA_A + ISA_B + ISA_C) with the EMAC unit, as
//! found in the MCF5441x. No FPU and no MMU: the MCF54418 has neither, and a
//! float opcode raises a line-F exception as it would on the device.
//!
//! Encodings and semantics follow the ColdFire Family Programmer's Reference
//! Manual (CFPRM) and the MCF5441x Reference Manual chapter 5 (EMAC). The
//! EMAC is modelled on its physical registers, ACCn and ACCextn, so a MACSR
//! mode change reinterprets them rather than repacking anything -- which is
//! where the reference emulator's patched Unicorn went wrong three times
//! (docs: "the patched Unicorn got the EMAC's arithmetic wrong in both
//! modes the audio engine uses").

use crate::bus::Bus;

pub const CF_C: u16 = 0x01;
pub const CF_V: u16 = 0x02;
pub const CF_Z: u16 = 0x04;
pub const CF_N: u16 = 0x08;
pub const CF_X: u16 = 0x10;
pub const SR_S: u16 = 0x2000;
pub const SR_T: u16 = 0x8000;

pub const MACSR_EV: u32 = 0x001;
pub const MACSR_V: u32 = 0x002;
pub const MACSR_Z: u32 = 0x004;
pub const MACSR_N: u32 = 0x008;
pub const MACSR_RT: u32 = 0x010;
pub const MACSR_FI: u32 = 0x020;
pub const MACSR_SU: u32 = 0x040;
pub const MACSR_OMC: u32 = 0x080;
pub const MACSR_PAV0: u32 = 0x100;

/// Exception vectors the core raises itself.
pub const VEC_ACCESS: u8 = 2;
pub const VEC_ADDRESS: u8 = 3;
pub const VEC_ILLEGAL: u8 = 4;
pub const VEC_DIVZERO: u8 = 5;
pub const VEC_PRIV: u8 = 8;
pub const VEC_TRACE: u8 = 9;
pub const VEC_LINEA: u8 = 10;
pub const VEC_LINEF: u8 = 11;
pub const VEC_FORMAT: u8 = 14;
pub const VEC_TRAP0: u8 = 32;

/// Why `run` returned.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Stop {
    /// The instruction budget ran out.
    Budget,
    /// PC reached an address with a hook set; nothing at PC has executed.
    Hook(u32),
    /// The core is in STOP and no interrupt can wake it before the budget.
    Stopped,
    /// A peripheral event is due (`io.deadline` reached).
    Event,
    /// An exception whose vector is empty or points nowhere sensible.
    Fault(u8, u32),
}

#[derive(Clone, Copy)]
enum Loc {
    D(usize),
    A(usize),
    M(u32),
    I(u32),
}

pub struct Cpu {
    pub d: [u32; 8],
    pub a: [u32; 8],
    pub pc: u32,
    pub sr: u16,
    /// The inactive stack pointer, when CACR[EUSP] banks A7.
    pub other_sp: u32,
    pub vbr: u32,
    pub cacr: u32,
    /// Every other MOVEC register, by Rc.
    pub ctl: std::collections::BTreeMap<u16, u32>,

    pub macsr: u32,
    pub mask: u32,
    /// ACC0-3, physical.
    pub acc: [u32; 4],
    /// ACCext0-3, physical: upper byte in bits 15-8, lower byte in 7-0.
    pub accext: [u16; 4],

    pub stopped: bool,
    /// Address of the instruction being executed.
    pub op_pc: u32,
    pub bus: Bus,

    /// One bit per halfword of the hooked code window.
    hook_bits: Vec<u64>,
    hook_base: u32,
    /// Let the instruction at this hooked PC execute once without stopping.
    /// Tied to the address, so an interrupt taken first cannot consume it.
    pub skip_hook: Option<u32>,
    pub last_fault: Option<(u8, u32)>,
    /// Count of exceptions raised by vector, for diagnostics.
    pub exc_counts: [u64; 256],
    /// Guest PC samples, one per 1024 instructions, when enabled.
    pub profile: Option<std::collections::HashMap<u32, u64>>,
    /// A ring of the last executed PCs, when enabled (debugging).
    pub history: Option<(Vec<u32>, usize)>,
    /// The first few faults: (vector, pc, opcode, clock).
    pub fault_log: Vec<(u8, u32, u16, u64)>,
}

const HOOK_WINDOW: u32 = 0x0080_0000; // 8 MB of code from hook_base

#[inline(always)]
fn sz_mask(sz: u32) -> u32 {
    match sz {
        1 => 0xFF,
        2 => 0xFFFF,
        _ => 0xFFFF_FFFF,
    }
}

#[inline(always)]
fn sz_msb(sz: u32) -> u32 {
    match sz {
        1 => 0x80,
        2 => 0x8000,
        _ => 0x8000_0000,
    }
}

#[inline(always)]
fn sext(v: u32, sz: u32) -> u32 {
    match sz {
        1 => v as u8 as i8 as i32 as u32,
        2 => v as u16 as i16 as i32 as u32,
        _ => v,
    }
}

impl Cpu {
    pub fn new(bus: Bus) -> Cpu {
        Cpu {
            d: [0; 8],
            a: [0; 8],
            pc: 0,
            sr: 0x2700,
            other_sp: 0,
            vbr: 0,
            cacr: 0,
            ctl: Default::default(),
            macsr: 0,
            mask: 0xFFFF_FFFF,
            acc: [0; 4],
            accext: [0; 4],
            stopped: false,
            op_pc: 0,
            bus,
            hook_bits: vec![0; (HOOK_WINDOW / 2 / 64) as usize],
            hook_base: 0x4000_0000,
            skip_hook: None,
            last_fault: None,
            exc_counts: [0; 256],
            fault_log: Vec::new(),
            history: None,
            profile: None,
        }
    }

    // -- hooks ------------------------------------------------------------

    pub fn set_hook(&mut self, pc: u32, on: bool) {
        let o = pc.wrapping_sub(self.hook_base);
        assert!(o < HOOK_WINDOW, "hook 0x{pc:08x} outside the hook window");
        let bit = (o >> 1) as usize;
        if on {
            self.hook_bits[bit >> 6] |= 1 << (bit & 63);
        } else {
            self.hook_bits[bit >> 6] &= !(1 << (bit & 63));
        }
    }

    #[inline(always)]
    fn hooked(&self, pc: u32) -> bool {
        let o = pc.wrapping_sub(self.hook_base);
        if o >= HOOK_WINDOW {
            return false;
        }
        let bit = (o >> 1) as usize;
        self.hook_bits[bit >> 6] & (1 << (bit & 63)) != 0
    }

    // -- status register --------------------------------------------------

    #[inline(always)]
    pub fn ipl(&self) -> u8 {
        ((self.sr >> 8) & 7) as u8
    }

    #[inline(always)]
    pub fn supervisor(&self) -> bool {
        self.sr & SR_S != 0
    }

    /// Install a new SR, banking A7 if CACR[EUSP] says the core has two.
    pub fn set_sr(&mut self, v: u16) {
        let v = v & 0xB71F;
        if self.cacr & 0x20 != 0 && (v ^ self.sr) & SR_S != 0 {
            std::mem::swap(&mut self.a[7], &mut self.other_sp);
        }
        self.sr = v;
    }

    #[inline(always)]
    fn set_ccr(&mut self, v: u16) {
        self.sr = (self.sr & 0xFF00) | (v & 0x1F);
    }

    #[inline(always)]
    fn flag(&self, f: u16) -> bool {
        self.sr & f != 0
    }

    #[inline(always)]
    fn set_nz(&mut self, v: u32, sz: u32) {
        let m = sz_mask(sz);
        let mut c = self.sr & !(CF_N | CF_Z | CF_V | CF_C);
        if v & m == 0 {
            c |= CF_Z;
        }
        if v & sz_msb(sz) != 0 {
            c |= CF_N;
        }
        self.sr = c;
    }

    fn cond(&self, cc: u16) -> bool {
        let c = self.flag(CF_C);
        let v = self.flag(CF_V);
        let z = self.flag(CF_Z);
        let n = self.flag(CF_N);
        match cc & 15 {
            0 => true,
            1 => false,
            2 => !c && !z,
            3 => c || z,
            4 => !c,
            5 => c,
            6 => !z,
            7 => z,
            8 => !v,
            9 => v,
            10 => !n,
            11 => n,
            12 => n == v,
            13 => n != v,
            14 => !z && n == v,
            _ => z || n != v,
        }
    }

    // -- memory -----------------------------------------------------------

    #[inline(always)]
    pub fn rd(&mut self, a: u32, sz: u32) -> u32 {
        match sz {
            1 => self.bus.read8(a) as u32,
            2 => self.bus.read16(a) as u32,
            _ => self.bus.read32(a),
        }
    }

    #[inline(always)]
    pub fn wr(&mut self, a: u32, sz: u32, v: u32) {
        match sz {
            1 => self.bus.write8(a, v as u8),
            2 => self.bus.write16(a, v as u16),
            _ => self.bus.write32(a, v),
        }
    }

    #[inline(always)]
    fn fetch16(&mut self) -> u16 {
        let v = self.bus.read16(self.pc);
        self.pc = self.pc.wrapping_add(2);
        v
    }

    #[inline(always)]
    fn fetch32(&mut self) -> u32 {
        let v = self.bus.read32(self.pc);
        self.pc = self.pc.wrapping_add(4);
        v
    }

    pub fn push32(&mut self, v: u32) {
        self.a[7] = self.a[7].wrapping_sub(4);
        let sp = self.a[7];
        self.bus.write32(sp, v);
    }

    pub fn pop32(&mut self) -> u32 {
        let sp = self.a[7];
        let v = self.bus.read32(sp);
        self.a[7] = sp.wrapping_add(4);
        v
    }

    // -- effective addresses ---------------------------------------------

    /// The brief extension word format: (d8, base, Xi.size*scale).
    fn index_ea(&mut self, base: u32) -> u32 {
        let ext = self.fetch16();
        let r = ((ext >> 12) & 7) as usize;
        let xi = if ext & 0x8000 != 0 { self.a[r] } else { self.d[r] };
        let xi = if ext & 0x0800 != 0 { xi } else { xi as u16 as i16 as i32 as u32 };
        let scale = (ext >> 9) & 3;
        base.wrapping_add(ext as u8 as i8 as i32 as u32)
            .wrapping_add(xi << scale)
    }

    /// Resolve an effective address for an operand of `sz` bytes. Applies
    /// (An)+ and -(An) side effects, so call it exactly once per operand.
    fn ea(&mut self, mode: u16, reg: u16, sz: u32) -> Loc {
        let r = reg as usize;
        match mode {
            0 => Loc::D(r),
            1 => Loc::A(r),
            2 => Loc::M(self.a[r]),
            // ColdFire keeps no word alignment for A7: a byte access through
            // (A7)+ or -(A7) moves it by one (the 680x0 rule of two does not
            // apply; QEMU gates it on the 680x0 feature).
            3 => {
                let a = self.a[r];
                self.a[r] = a.wrapping_add(sz);
                Loc::M(a)
            }
            4 => {
                let a = self.a[r].wrapping_sub(sz);
                self.a[r] = a;
                Loc::M(a)
            }
            5 => {
                let d = self.fetch16() as i16 as i32 as u32;
                Loc::M(self.a[r].wrapping_add(d))
            }
            6 => {
                let b = self.a[r];
                Loc::M(self.index_ea(b))
            }
            _ => match reg {
                0 => Loc::M(self.fetch16() as i16 as i32 as u32),
                1 => Loc::M(self.fetch32()),
                2 => {
                    let base = self.pc;
                    let d = self.fetch16() as i16 as i32 as u32;
                    Loc::M(base.wrapping_add(d))
                }
                3 => {
                    let base = self.pc;
                    Loc::M(self.index_ea(base))
                }
                4 => {
                    let v = match sz {
                        1 => self.fetch16() as u32 & 0xFF,
                        2 => self.fetch16() as u32,
                        _ => self.fetch32(),
                    };
                    Loc::I(v)
                }
                _ => Loc::I(0),
            },
        }
    }

    /// The address of a control-mode EA (LEA, PEA, JMP, JSR, MOVEM).
    fn ea_addr(&mut self, mode: u16, reg: u16) -> Option<u32> {
        match mode {
            2 | 5 | 6 => match self.ea(mode, reg, 4) {
                Loc::M(a) => Some(a),
                _ => None,
            },
            7 if reg <= 3 => match self.ea(mode, reg, 4) {
                Loc::M(a) => Some(a),
                _ => None,
            },
            _ => None,
        }
    }

    #[inline(always)]
    fn read_loc(&mut self, l: Loc, sz: u32) -> u32 {
        match l {
            Loc::D(r) => self.d[r] & sz_mask(sz),
            Loc::A(r) => self.a[r] & sz_mask(sz),
            Loc::M(a) => self.rd(a, sz),
            Loc::I(v) => v & sz_mask(sz),
        }
    }

    #[inline(always)]
    fn write_loc(&mut self, l: Loc, sz: u32, v: u32) {
        match l {
            Loc::D(r) => {
                let m = sz_mask(sz);
                self.d[r] = (self.d[r] & !m) | (v & m);
            }
            Loc::A(r) => self.a[r] = v,
            Loc::M(a) => self.wr(a, sz, v),
            Loc::I(_) => {}
        }
    }

    // -- exceptions -------------------------------------------------------

    /// Take exception `vec` with `pc` in the frame. `level` raises the
    /// interrupt mask (interrupts); synchronous exceptions leave it.
    pub fn exception(&mut self, vec: u8, pc: u32, level: Option<u8>) {
        self.exc_counts[vec as usize] += 1;
        let old_sr = self.sr;
        let mut new_sr = (old_sr | SR_S) & !SR_T;
        if let Some(l) = level {
            new_sr = (new_sr & !0x0700) | ((l as u16 & 7) << 8);
        }
        self.set_sr(new_sr);
        // The frame is longword aligned; the format field records by how
        // much the stack pointer was misaligned so RTE can undo it.
        let sp = self.a[7];
        let fmt = 4 + (sp & 3);
        self.a[7] = sp & !3;
        self.push32(pc);
        let w0 = (fmt << 28) | ((vec as u32) << 18) | old_sr as u32;
        self.push32(w0);
        let handler = self.bus.read32(self.vbr.wrapping_add(vec as u32 * 4));
        self.pc = handler;
        self.stopped = false;
    }

    fn rte(&mut self) {
        let sp = self.a[7];
        let w0 = self.bus.read32(sp);
        let pc = self.bus.read32(sp.wrapping_add(4));
        let fmt = w0 >> 28;
        let adj = if (4..=7).contains(&fmt) { fmt - 4 } else { 0 };
        self.a[7] = sp.wrapping_add(8 + adj);
        self.set_sr(w0 as u16);
        self.pc = pc;
    }

    /// Deliver an interrupt now if the mask allows it. -> taken.
    pub fn interrupt(&mut self, vec: u8, level: u8) -> bool {
        if level < 7 && level <= self.ipl() {
            return false;
        }
        let pc = self.pc;
        self.exception(vec, pc, Some(level));
        true
    }

    fn illegal(&mut self, vec: u8) {
        let pc = self.op_pc;
        if self.fault_log.len() < 32 {
            let op = self.bus.peek16(pc);
            self.fault_log.push((vec, pc, op, self.bus.io.now));
        }
        self.last_fault = Some((vec, pc));
        self.exception(vec, pc, None);
    }

    // -- the run loop -----------------------------------------------------

    /// Instructions executed so far: the machine's clock.
    #[inline(always)]
    pub fn icount(&self) -> u64 {
        self.bus.io.now
    }

    /// Execute until the clock reaches `until`, a peripheral event falls
    /// due, a hook is hit, or the core stops with nothing to wake it.
    pub fn run(&mut self, until: u64) -> Stop {
        loop {
            // Interrupts are sampled between instructions.
            let lvl = self.bus.io.irq_level;
            if lvl != 0 && (lvl == 7 || lvl > self.ipl()) {
                if let Some((vec, level)) = self.bus.io.ack_irq(self.ipl()) {
                    let pc = self.pc;
                    self.exception(vec, pc, Some(level));
                }
            }
            let now = self.bus.io.now;
            if now >= until {
                return Stop::Budget;
            }
            if now >= self.bus.io.deadline {
                return Stop::Event;
            }
            if self.stopped {
                return Stop::Stopped;
            }
            let pc = self.pc;
            if self.hooked(pc) {
                if self.skip_hook != Some(pc) {
                    return Stop::Hook(pc);
                }
                self.skip_hook = None;
            }
            self.op_pc = pc;
            self.bus.pc = pc;
            self.bus.io.now = now + 1;
            if now & 0x3FF == 0 {
                if let Some(p) = &mut self.profile {
                    *p.entry(pc).or_insert(0) += 1;
                }
            }
            if let Some((h, i)) = &mut self.history {
                let n = h.len();
                h[*i % n] = pc;
                *i += 1;
            }
            self.execute();
            if let Some((v, p)) = self.last_fault.take() {
                let h = self.bus.peek32(self.vbr.wrapping_add(v as u32 * 4));
                if h == 0 || h == 0xFFFF_FFFF {
                    return Stop::Fault(v, p);
                }
            }
        }
    }

    /// Execute exactly one instruction.
    pub fn step(&mut self) {
        let pc = self.pc;
        self.op_pc = pc;
        self.bus.pc = pc;
        self.bus.io.now += 1;
        self.execute();
    }

    fn execute(&mut self) {
        let op = self.fetch16();
        match op >> 12 {
            0x0 => self.line0(op),
            0x1 => self.op_move(op, 1),
            0x2 => self.op_move(op, 4),
            0x3 => self.op_move(op, 2),
            0x4 => self.line4(op),
            0x5 => self.line5(op),
            0x6 => self.line6(op),
            0x7 => self.line7(op),
            0x8 => self.line8(op),
            0x9 => self.line9d(op, false),
            0xA => self.linea(op),
            0xB => self.lineb(op),
            0xC => self.linec(op),
            0xD => self.line9d(op, true),
            0xE => self.linee(op),
            _ => self.linef(op),
        }
    }

    // -- arithmetic helpers -----------------------------------------------

    fn add_flags(&mut self, s: u32, d: u32, sz: u32, x: bool) -> u32 {
        let m = sz_mask(sz);
        let (s, d) = (s & m, d & m);
        let wide = s as u64 + d as u64 + x as u64;
        let r = (wide as u32) & m;
        let msb = sz_msb(sz);
        let carry = if sz == 4 { wide >> 32 != 0 } else { wide as u32 & (m + 1) != 0 };
        let over = (s ^ r) & (d ^ r) & msb != 0;
        let mut c = self.sr & !0x1F;
        if carry {
            c |= CF_C | CF_X;
        }
        if over {
            c |= CF_V;
        }
        if r & msb != 0 {
            c |= CF_N;
        }
        if r == 0 {
            c |= CF_Z;
        }
        self.sr = c;
        r
    }

    /// d - s - x, all condition codes including X.
    fn sub_flags(&mut self, s: u32, d: u32, sz: u32, x: bool) -> u32 {
        let m = sz_mask(sz);
        let (s, d) = (s & m, d & m);
        let r = d.wrapping_sub(s).wrapping_sub(x as u32) & m;
        let msb = sz_msb(sz);
        let borrow = (s as u64 + x as u64) > d as u64;
        let over = (s ^ d) & (r ^ d) & msb != 0;
        let mut c = self.sr & !0x1F;
        if borrow {
            c |= CF_C | CF_X;
        }
        if over {
            c |= CF_V;
        }
        if r & msb != 0 {
            c |= CF_N;
        }
        if r == 0 {
            c |= CF_Z;
        }
        self.sr = c;
        r
    }

    fn cmp_flags(&mut self, s: u32, d: u32, sz: u32) {
        let x = self.sr & CF_X;
        self.sub_flags(s, d, sz, false);
        self.sr = (self.sr & !CF_X) | x;
    }

    // -- line 0: bit operations, immediates, ISA_C extras ------------------

    fn line0(&mut self, op: u16) {
        let mode = (op >> 3) & 7;
        let reg = op & 7;
        if op & 0x0100 != 0 {
            // Dynamic bit ops: 0000 ddd1 tt mmmrrr
            let bit = self.d[((op >> 9) & 7) as usize];
            self.bitop((op >> 6) & 3, bit, mode, reg);
            return;
        }
        match op & 0xFFC0 {
            0x0800 | 0x0840 | 0x0880 | 0x08C0 => {
                let bit = self.fetch16() as u32 & 0xFF;
                self.bitop((op >> 6) & 3, bit, mode, reg);
                return;
            }
            _ => {}
        }
        match op & 0xFFF8 {
            0x00C0 => {
                // BITREV
                let r = reg as usize;
                self.d[r] = self.d[r].reverse_bits();
            }
            0x02C0 => {
                // BYTEREV
                let r = reg as usize;
                self.d[r] = self.d[r].swap_bytes();
            }
            0x04C0 => {
                // FF1: the offset of the first set bit from the msb.
                let r = reg as usize;
                let v = self.d[r];
                self.set_nz(v, 4);
                self.d[r] = v.leading_zeros();
            }
            0x0080 | 0x0280 | 0x0480 | 0x0680 | 0x0A80 => {
                let imm = self.fetch32();
                let r = reg as usize;
                let dv = self.d[r];
                let res = match op & 0xFFF8 {
                    0x0080 => {
                        let v = dv | imm;
                        self.set_nz(v, 4);
                        v
                    }
                    0x0280 => {
                        let v = dv & imm;
                        self.set_nz(v, 4);
                        v
                    }
                    0x0480 => self.sub_flags(imm, dv, 4, false),
                    0x0680 => self.add_flags(imm, dv, 4, false),
                    _ => {
                        let v = dv ^ imm;
                        self.set_nz(v, 4);
                        v
                    }
                };
                self.d[r] = res;
            }
            0x0C00 | 0x0C40 | 0x0C80 => {
                // CMPI.sz #imm, Dn
                let sz = [1, 2, 4][((op >> 6) & 3) as usize];
                let imm = match sz {
                    4 => self.fetch32(),
                    _ => self.fetch16() as u32 & sz_mask(sz),
                };
                let dv = self.d[reg as usize];
                self.cmp_flags(imm, dv, sz);
            }
            _ => self.illegal(VEC_ILLEGAL),
        }
    }

    fn bitop(&mut self, kind: u16, bit: u32, mode: u16, reg: u16) {
        if mode == 0 {
            let r = reg as usize;
            let m = 1u32 << (bit & 31);
            let v = self.d[r];
            self.sr = (self.sr & !CF_Z) | if v & m == 0 { CF_Z } else { 0 };
            match kind {
                1 => self.d[r] = v ^ m,
                2 => self.d[r] = v & !m,
                3 => self.d[r] = v | m,
                _ => {}
            }
            return;
        }
        let l = self.ea(mode, reg, 1);
        let m = 1u32 << (bit & 7);
        let v = self.read_loc(l, 1);
        self.sr = (self.sr & !CF_Z) | if v & m == 0 { CF_Z } else { 0 };
        let nv = match kind {
            1 => v ^ m,
            2 => v & !m,
            3 => v | m,
            _ => return,
        };
        self.write_loc(l, 1, nv);
    }

    // -- MOVE ------------------------------------------------------------

    fn op_move(&mut self, op: u16, sz: u32) {
        let src = self.ea((op >> 3) & 7, op & 7, sz);
        let v = self.read_loc(src, sz);
        let dmode = (op >> 6) & 7;
        let dreg = (op >> 9) & 7;
        if dmode == 1 {
            // MOVEA: word sources sign-extend; no flags.
            self.a[dreg as usize] = sext(v, sz);
            return;
        }
        let dst = self.ea(dmode, dreg, sz);
        self.write_loc(dst, sz, v);
        self.set_nz(v, sz);
    }

    // -- line 4: miscellaneous --------------------------------------------

    fn line4(&mut self, op: u16) {
        let mode = (op >> 3) & 7;
        let reg = op & 7;
        let r = reg as usize;
        // LEA: 0100 aaa1 11 ea (0x49C0 with mode 0 is EXTB.L)
        if op & 0x01C0 == 0x01C0 {
            if op & 0xFFF8 == 0x49C0 {
                let v = self.d[r] as u8 as i8 as i32 as u32;
                self.d[r] = v;
                self.set_nz(v, 4);
                return;
            }
            match self.ea_addr(mode, reg) {
                Some(a) => self.a[((op >> 9) & 7) as usize] = a,
                None => self.illegal(VEC_ILLEGAL),
            }
            return;
        }
        match op {
            0x4E71 => return, // NOP
            0x4E73 => {
                if !self.supervisor() {
                    return self.illegal(VEC_PRIV);
                }
                self.rte();
                return;
            }
            0x4E75 => {
                self.pc = self.pop32();
                return;
            }
            0x4E72 => {
                let imm = self.fetch16();
                if !self.supervisor() {
                    return self.illegal(VEC_PRIV);
                }
                self.set_sr(imm);
                self.stopped = true;
                return;
            }
            0x4E7A | 0x4E7B => {
                let ext = self.fetch16();
                if !self.supervisor() {
                    return self.illegal(VEC_PRIV);
                }
                let rn = ((ext >> 12) & 7) as usize;
                let is_a = ext & 0x8000 != 0;
                let rc = ext & 0x0FFF;
                if op == 0x4E7B {
                    let v = if is_a { self.a[rn] } else { self.d[rn] };
                    self.movec_write(rc, v);
                } else {
                    let v = self.movec_read(rc);
                    if is_a {
                        self.a[rn] = v;
                    } else {
                        self.d[rn] = v;
                    }
                }
                return;
            }
            0x4AC8 => {
                // HALT: a debug halt. Treat it as a stop.
                self.stopped = true;
                return;
            }
            0x4ACC => return, // PULSE
            0x4AFC => return self.illegal(VEC_ILLEGAL),
            0x40E7 => {
                // STRLDSR #imm: 0x40E7 0x46FC imm
                if self.bus.read16(self.pc) == 0x46FC {
                    self.pc = self.pc.wrapping_add(2);
                    let imm = self.fetch16();
                    if !self.supervisor() {
                        return self.illegal(VEC_PRIV);
                    }
                    let sr = self.sr as u32;
                    self.a[7] = self.a[7].wrapping_sub(4);
                    let sp = self.a[7];
                    self.bus.write32(sp, sr);
                    self.set_sr(imm);
                    return;
                }
                return self.illegal(VEC_ILLEGAL);
            }
            _ => {}
        }
        match op & 0xFFF0 {
            0x4E40 => {
                let vec = VEC_TRAP0 + (op & 15) as u8;
                let pc = self.pc;
                self.exception(vec, pc, None);
                return;
            }
            0x4E50 => {
                if op & 8 == 0 {
                    // LINK An,#d16
                    let d = self.fetch16() as i16 as i32 as u32;
                    let v = self.a[r];
                    self.push32(v);
                    self.a[r] = self.a[7];
                    self.a[7] = self.a[7].wrapping_add(d);
                } else {
                    // UNLK An
                    self.a[7] = self.a[r];
                    let v = self.pop32();
                    self.a[r] = v;
                }
                return;
            }
            0x4E60 => {
                if !self.supervisor() {
                    return self.illegal(VEC_PRIV);
                }
                if op & 8 == 0 {
                    self.other_sp = self.a[r];
                } else {
                    self.a[r] = self.other_sp;
                }
                return;
            }
            _ => {}
        }
        match op & 0xFFC0 {
            0x4E80 => {
                // JSR
                match self.ea_addr(mode, reg) {
                    Some(t) => {
                        let ret = self.pc;
                        self.push32(ret);
                        self.pc = t;
                    }
                    None => self.illegal(VEC_ILLEGAL),
                }
                return;
            }
            0x4EC0 => {
                match self.ea_addr(mode, reg) {
                    Some(t) => self.pc = t,
                    None => self.illegal(VEC_ILLEGAL),
                }
                return;
            }
            0x40C0 => {
                // MOVE SR,Dn
                if !self.supervisor() {
                    return self.illegal(VEC_PRIV);
                }
                if mode == 0 {
                    self.d[r] = (self.d[r] & 0xFFFF_0000) | self.sr as u32;
                } else {
                    self.illegal(VEC_ILLEGAL);
                }
                return;
            }
            0x42C0 => {
                // MOVE CCR,Dn
                if mode == 0 {
                    self.d[r] = (self.d[r] & 0xFFFF_0000) | (self.sr & 0x1F) as u32;
                } else {
                    self.illegal(VEC_ILLEGAL);
                }
                return;
            }
            0x44C0 => {
                // MOVE <ea>,CCR
                let l = self.ea(mode, reg, 2);
                let v = self.read_loc(l, 2);
                self.set_ccr(v as u16);
                return;
            }
            0x46C0 => {
                // MOVE <ea>,SR
                let l = self.ea(mode, reg, 2);
                let v = self.read_loc(l, 2);
                if !self.supervisor() {
                    return self.illegal(VEC_PRIV);
                }
                self.set_sr(v as u16);
                return;
            }
            0x4840 => {
                if mode == 0 {
                    // SWAP
                    let v = self.d[r].rotate_left(16);
                    self.d[r] = v;
                    self.set_nz(v, 4);
                } else {
                    // PEA
                    match self.ea_addr(mode, reg) {
                        Some(a) => self.push32(a),
                        None => self.illegal(VEC_ILLEGAL),
                    }
                }
                return;
            }
            0x4880 => {
                if mode == 0 {
                    // EXT.W
                    let v = self.d[r] as u8 as i8 as i16 as u16 as u32;
                    self.d[r] = (self.d[r] & 0xFFFF_0000) | v;
                    self.set_nz(v, 2);
                } else {
                    self.illegal(VEC_ILLEGAL);
                }
                return;
            }
            0x48C0 => {
                if mode == 0 {
                    // EXT.L
                    let v = self.d[r] as u16 as i16 as i32 as u32;
                    self.d[r] = v;
                    self.set_nz(v, 4);
                } else {
                    self.movem(mode, reg, true);
                }
                return;
            }
            0x4CC0 => {
                self.movem(mode, reg, false);
                return;
            }
            0x4C00 => return self.mul_l(mode, reg),
            0x4C40 => return self.div_l(mode, reg),
            0x4C80 => {
                if mode == 0 {
                    // SATS.L
                    if self.flag(CF_V) {
                        self.d[r] = if self.d[r] & 0x8000_0000 != 0 { 0x7FFF_FFFF } else { 0x8000_0000 };
                    }
                    let v = self.d[r];
                    self.set_nz(v, 4);
                } else {
                    self.illegal(VEC_ILLEGAL);
                }
                return;
            }
            0x4AC0 => {
                // TAS.B
                let l = self.ea(mode, reg, 1);
                let v = self.read_loc(l, 1);
                self.set_nz(v, 1);
                self.write_loc(l, 1, v | 0x80);
                return;
            }
            _ => {}
        }
        // Size-encoded single-operand group: 0100 xxx0 ss mmmrrr
        let ss = (op >> 6) & 3;
        match op & 0xFF00 {
            0x4000 if ss == 2 && mode == 0 => {
                // NEGX.L Dn
                let x = self.flag(CF_X);
                let z = self.sr & CF_Z;
                let v = self.sub_flags(self.d[r], 0, 4, x);
                if v != 0 {
                    self.sr &= !CF_Z;
                } else {
                    self.sr = (self.sr & !CF_Z) | z;
                }
                self.d[r] = v;
            }
            0x4200 if ss != 3 => {
                // CLR
                let sz = [1, 2, 4][ss as usize];
                let l = self.ea(mode, reg, sz);
                self.write_loc(l, sz, 0);
                self.sr = (self.sr & !0x0F) | CF_Z;
            }
            0x4400 if ss == 2 && mode == 0 => {
                let v = self.sub_flags(self.d[r], 0, 4, false);
                self.d[r] = v;
            }
            0x4600 if ss == 2 && mode == 0 => {
                let v = !self.d[r];
                self.d[r] = v;
                self.set_nz(v, 4);
            }
            0x4A00 if ss != 3 => {
                // TST
                let sz = [1, 2, 4][ss as usize];
                let l = self.ea(mode, reg, sz);
                let v = self.read_loc(l, sz);
                self.set_nz(v, sz);
            }
            _ => self.illegal(VEC_ILLEGAL),
        }
    }

    fn movem(&mut self, mode: u16, reg: u16, to_mem: bool) {
        let mask = self.fetch16();
        let base = match mode {
            2 => self.a[reg as usize],
            5 => {
                let d = self.fetch16() as i16 as i32 as u32;
                self.a[reg as usize].wrapping_add(d)
            }
            _ => return self.illegal(VEC_ILLEGAL),
        };
        let mut a = base;
        for i in 0..16 {
            if mask & (1 << i) == 0 {
                continue;
            }
            if to_mem {
                let v = if i < 8 { self.d[i] } else { self.a[i - 8] };
                self.bus.write32(a, v);
            } else {
                let v = self.bus.read32(a);
                if i < 8 {
                    self.d[i] = v;
                } else {
                    self.a[i - 8] = v;
                }
            }
            a = a.wrapping_add(4);
        }
    }

    fn mul_l(&mut self, mode: u16, reg: u16) {
        let ext = self.fetch16();
        let l = self.ea(mode, reg, 4);
        let s = self.read_loc(l, 4);
        let dl = ((ext >> 12) & 7) as usize;
        let res = if ext & 0x0800 != 0 {
            (self.d[dl] as i32).wrapping_mul(s as i32) as u32
        } else {
            self.d[dl].wrapping_mul(s)
        };
        self.d[dl] = res;
        self.set_nz(res, 4);
    }

    fn div_l(&mut self, mode: u16, reg: u16) {
        let ext = self.fetch16();
        let l = self.ea(mode, reg, 4);
        let s = self.read_loc(l, 4);
        let dq = ((ext >> 12) & 7) as usize;
        let dr = (ext & 7) as usize;
        let signed = ext & 0x0800 != 0;
        if s == 0 {
            let pc = self.pc;
            self.exception(VEC_DIVZERO, pc, None);
            return;
        }
        let dividend = self.d[dq];
        let (q, rem, over) = if signed {
            let (a, b) = (dividend as i32, s as i32);
            if a == i32::MIN && b == -1 {
                (0, 0, true)
            } else {
                ((a / b) as u32, (a % b) as u32, false)
            }
        } else {
            (dividend / s, dividend % s, false)
        };
        if over {
            self.div_overflow();
            return;
        }
        // The condition codes describe the quotient, for REM as for DIV.
        self.set_nz(q, 4);
        if dq == dr {
            self.d[dq] = q;
        } else {
            // REMS/REMU: the remainder goes to Dw, the dividend stays.
            self.d[dr] = rem;
        }
    }

    /// Division overflow: the destination is untouched, V set, C and Z
    /// clear, N kept (as the 68040 and QEMU's ColdFire model do).
    fn div_overflow(&mut self) {
        self.sr = (self.sr & !(CF_V | CF_C | CF_Z)) | CF_V;
    }

    fn movec_write(&mut self, rc: u16, v: u32) {
        match rc {
            0x002 => {
                let old_eusp = self.cacr & 0x20 != 0;
                self.cacr = v;
                let _ = old_eusp;
            }
            0x800 => self.other_sp = v,
            0x801 => self.vbr = v & 0xFFF0_0000,
            _ => {
                self.ctl.insert(rc, v);
            }
        }
    }

    fn movec_read(&mut self, rc: u16) -> u32 {
        match rc {
            0x002 => self.cacr,
            0x800 => self.other_sp,
            0x801 => self.vbr,
            _ => self.ctl.get(&rc).copied().unwrap_or(0),
        }
    }

    // -- line 5: ADDQ, SUBQ, Scc, TPF --------------------------------------

    fn line5(&mut self, op: u16) {
        let mode = (op >> 3) & 7;
        let reg = op & 7;
        if (op >> 6) & 3 == 3 {
            match op & 0x3F {
                0x3A => {
                    self.pc = self.pc.wrapping_add(2);
                    return;
                }
                0x3B => {
                    self.pc = self.pc.wrapping_add(4);
                    return;
                }
                0x3C => return,
                _ => {}
            }
            if mode == 0 {
                let v = if self.cond((op >> 8) & 15) { 0xFF } else { 0 };
                let r = reg as usize;
                self.d[r] = (self.d[r] & !0xFF) | v;
            } else {
                self.illegal(VEC_ILLEGAL);
            }
            return;
        }
        let mut q = ((op >> 9) & 7) as u32;
        if q == 0 {
            q = 8;
        }
        let sz = [1, 2, 4][((op >> 6) & 3) as usize];
        if mode == 1 {
            let r = reg as usize;
            self.a[r] = if op & 0x100 != 0 { self.a[r].wrapping_sub(q) } else { self.a[r].wrapping_add(q) };
            return;
        }
        let l = self.ea(mode, reg, sz);
        let dv = self.read_loc(l, sz);
        let res = if op & 0x100 != 0 {
            self.sub_flags(q, dv, sz, false)
        } else {
            self.add_flags(q, dv, sz, false)
        };
        self.write_loc(l, sz, res);
    }

    // -- line 6: branches -------------------------------------------------

    fn line6(&mut self, op: u16) {
        let base = self.pc;
        let d8 = op & 0xFF;
        let disp = match d8 {
            0 => self.fetch16() as i16 as i32 as u32,
            0xFF => self.fetch32(),
            _ => d8 as u8 as i8 as i32 as u32,
        };
        let cc = (op >> 8) & 15;
        let target = base.wrapping_add(disp);
        match cc {
            0 => self.pc = target,
            1 => {
                let ret = self.pc;
                self.push32(ret);
                self.pc = target;
            }
            _ => {
                if self.cond(cc) {
                    self.pc = target;
                }
            }
        }
    }

    // -- line 7: MOVEQ, MVS, MVZ --------------------------------------------

    fn line7(&mut self, op: u16) {
        let dn = ((op >> 9) & 7) as usize;
        if op & 0x100 == 0 {
            let v = op as u8 as i8 as i32 as u32;
            self.d[dn] = v;
            self.set_nz(v, 4);
            return;
        }
        let ss = (op >> 6) & 3;
        let sz = if ss & 1 == 0 { 1 } else { 2 };
        let l = self.ea((op >> 3) & 7, op & 7, sz);
        let v = self.read_loc(l, sz);
        let v = if ss < 2 { sext(v, sz) } else { v };
        self.d[dn] = v;
        self.set_nz(v, 4);
    }

    // -- line 8: OR, DIVU.W, DIVS.W -----------------------------------------

    fn line8(&mut self, op: u16) {
        let dn = ((op >> 9) & 7) as usize;
        let opmode = (op >> 6) & 7;
        let mode = (op >> 3) & 7;
        let reg = op & 7;
        match opmode {
            3 | 7 => {
                let l = self.ea(mode, reg, 2);
                let s = self.read_loc(l, 2);
                self.div_w(dn, s, opmode == 7);
            }
            0..=2 => {
                let sz = [1, 2, 4][opmode as usize];
                let l = self.ea(mode, reg, sz);
                let v = self.read_loc(l, sz) | self.d[dn];
                self.write_loc(Loc::D(dn), sz, v);
                self.set_nz(v, sz);
            }
            _ => {
                let sz = [1, 2, 4][(opmode - 4) as usize];
                let l = self.ea(mode, reg, sz);
                let v = self.read_loc(l, sz) | self.d[dn];
                self.write_loc(l, sz, v);
                self.set_nz(v, sz);
            }
        }
    }

    fn div_w(&mut self, dn: usize, s: u32, signed: bool) {
        let s16 = s & 0xFFFF;
        if s16 == 0 {
            let pc = self.pc;
            self.exception(VEC_DIVZERO, pc, None);
            return;
        }
        let dv = self.d[dn];
        let (q, r, over) = if signed {
            let (a, b) = (dv as i32 as i64, s16 as u16 as i16 as i64);
            let q = a / b;
            let r = a % b;
            (q as u32, r as u32, q < -32768 || q > 32767)
        } else {
            let q = dv / s16;
            (q, dv % s16, q > 0xFFFF)
        };
        if over {
            self.div_overflow();
            return;
        }
        self.d[dn] = (r << 16) | (q & 0xFFFF);
        self.set_nz(q, 2);
    }

    // -- lines 9 and D: SUB/ADD family ---------------------------------------

    fn line9d(&mut self, op: u16, add: bool) {
        let dn = ((op >> 9) & 7) as usize;
        let opmode = (op >> 6) & 7;
        let mode = (op >> 3) & 7;
        let reg = op & 7;
        match opmode {
            3 | 7 => {
                // ADDA/SUBA
                let sz = if opmode == 3 { 2 } else { 4 };
                let l = self.ea(mode, reg, sz);
                let s = sext(self.read_loc(l, sz), sz);
                self.a[dn] = if add { self.a[dn].wrapping_add(s) } else { self.a[dn].wrapping_sub(s) };
            }
            0..=2 => {
                let sz = [1, 2, 4][opmode as usize];
                let l = self.ea(mode, reg, sz);
                let s = self.read_loc(l, sz);
                let d = self.d[dn];
                let r = if add { self.add_flags(s, d, sz, false) } else { self.sub_flags(s, d, sz, false) };
                self.write_loc(Loc::D(dn), sz, r);
            }
            _ => {
                let sz = [1, 2, 4][(opmode - 4) as usize];
                if mode == 0 {
                    // ADDX/SUBX Dy,Dx
                    let ry = reg as usize;
                    let x = self.flag(CF_X);
                    let z = self.sr & CF_Z;
                    let (s, d) = (self.d[ry], self.d[dn]);
                    let r = if add { self.add_flags(s, d, sz, x) } else { self.sub_flags(s, d, sz, x) };
                    if r & sz_mask(sz) == 0 {
                        self.sr = (self.sr & !CF_Z) | z;
                    }
                    self.write_loc(Loc::D(dn), sz, r);
                    return;
                }
                if mode == 1 {
                    return self.illegal(VEC_ILLEGAL);
                }
                let l = self.ea(mode, reg, sz);
                let d = self.read_loc(l, sz);
                let s = self.d[dn];
                let r = if add { self.add_flags(s, d, sz, false) } else { self.sub_flags(s, d, sz, false) };
                self.write_loc(l, sz, r);
            }
        }
    }

    // -- line B: CMP, CMPA, EOR ----------------------------------------------

    fn lineb(&mut self, op: u16) {
        let dn = ((op >> 9) & 7) as usize;
        let opmode = (op >> 6) & 7;
        let mode = (op >> 3) & 7;
        let reg = op & 7;
        match opmode {
            0..=2 => {
                let sz = [1, 2, 4][opmode as usize];
                let l = self.ea(mode, reg, sz);
                let s = self.read_loc(l, sz);
                let d = self.d[dn];
                self.cmp_flags(s, d, sz);
            }
            3 | 7 => {
                let sz = if opmode == 3 { 2 } else { 4 };
                let l = self.ea(mode, reg, sz);
                let s = sext(self.read_loc(l, sz), sz);
                let d = self.a[dn];
                self.cmp_flags(s, d, 4);
            }
            _ => {
                let sz = [1, 2, 4][(opmode - 4) as usize];
                let l = self.ea(mode, reg, sz);
                let v = self.read_loc(l, sz) ^ self.d[dn];
                self.write_loc(l, sz, v);
                self.set_nz(v, sz);
            }
        }
    }

    // -- line C: AND, MULU.W, MULS.W -----------------------------------------

    fn linec(&mut self, op: u16) {
        let dn = ((op >> 9) & 7) as usize;
        let opmode = (op >> 6) & 7;
        let mode = (op >> 3) & 7;
        let reg = op & 7;
        match opmode {
            3 | 7 => {
                let l = self.ea(mode, reg, 2);
                let s = self.read_loc(l, 2);
                let r = if opmode == 7 {
                    (self.d[dn] as u16 as i16 as i32).wrapping_mul(s as u16 as i16 as i32) as u32
                } else {
                    (self.d[dn] & 0xFFFF).wrapping_mul(s & 0xFFFF)
                };
                self.d[dn] = r;
                self.set_nz(r, 4);
            }
            0..=2 => {
                let sz = [1, 2, 4][opmode as usize];
                let l = self.ea(mode, reg, sz);
                let v = self.read_loc(l, sz) & self.d[dn];
                self.write_loc(Loc::D(dn), sz, v);
                self.set_nz(v, sz);
            }
            _ => {
                let sz = [1, 2, 4][(opmode - 4) as usize];
                let l = self.ea(mode, reg, sz);
                let v = self.read_loc(l, sz) & self.d[dn];
                self.write_loc(l, sz, v);
                self.set_nz(v, sz);
            }
        }
    }

    // -- line E: shifts (register, long only on ColdFire) --------------------

    fn linee(&mut self, op: u16) {
        let r = (op & 7) as usize;
        let cnt_field = ((op >> 9) & 7) as u32;
        let count = if op & 0x20 != 0 {
            self.d[cnt_field as usize] & 63
        } else if cnt_field == 0 {
            8
        } else {
            cnt_field
        };
        let left = op & 0x100 != 0;
        let kind = (op >> 3) & 3;
        if (op >> 6) & 3 != 2 || kind > 1 {
            return self.illegal(VEC_ILLEGAL);
        }
        let v = self.d[r];
        let mut ccr = self.sr & !0x1F;
        let res;
        if count == 0 {
            res = v;
            ccr |= self.sr & CF_X;
        } else if left {
            res = if count >= 32 { 0 } else { v << count };
            let c = if count > 32 { false } else { (v >> (32 - count)) & 1 != 0 };
            if c {
                ccr |= CF_C | CF_X;
            }
        } else if kind == 1 {
            // LSR
            res = if count >= 32 { 0 } else { v >> count };
            let c = if count > 32 { false } else { (v >> (count - 1)) & 1 != 0 };
            if c {
                ccr |= CF_C | CF_X;
            }
        } else {
            // ASR
            let sv = v as i32;
            res = if count >= 32 { (sv >> 31) as u32 } else { (sv >> count) as u32 };
            let c = if count >= 32 { sv < 0 } else { (v >> (count - 1)) & 1 != 0 };
            if c {
                ccr |= CF_C | CF_X;
            }
        }
        if res == 0 {
            ccr |= CF_Z;
        }
        if res & 0x8000_0000 != 0 {
            ccr |= CF_N;
        }
        self.sr = ccr;
        self.d[r] = res;
    }

    // -- line F: cache maintenance, debug, and the missing FPU ---------------

    fn linef(&mut self, op: u16) {
        match op & 0xFF00 {
            0xF400 => {
                // CPUSHL / INTOUCH: no cache is modelled.
                if !self.supervisor() {
                    return self.illegal(VEC_PRIV);
                }
            }
            0xFB00 => {
                // WDDATA / WDEBUG: compute the EA for its length only.
                let mode = (op >> 3) & 7;
                let reg = op & 7;
                if op & 0x00C0 == 0x00C0 {
                    let _ = self.ea_addr(mode, reg);
                    self.pc = self.pc.wrapping_add(2);
                } else {
                    let sz = [1, 2, 4, 4][((op >> 6) & 3) as usize];
                    let _ = self.ea(mode, reg, sz);
                }
            }
            _ => self.illegal(VEC_LINEF),
        }
    }

    // -- line A: EMAC and MOV3Q ----------------------------------------------

    /// The 48-bit accumulator value in the current mode, sign- or
    /// zero-extended to 64 bits.
    pub fn acc_get(&self, i: usize) -> i64 {
        let acc = self.acc[i] as u64;
        let hi = (self.accext[i] >> 8) as u64 & 0xFF;
        let lo = self.accext[i] as u64 & 0xFF;
        if self.macsr & MACSR_FI != 0 {
            let raw = (hi << 40) | (acc << 8) | lo;
            ((raw << 16) as i64) >> 16
        } else {
            let raw = (hi << 40) | (lo << 32) | acc;
            if self.macsr & MACSR_SU == 0 {
                ((raw << 16) as i64) >> 16
            } else {
                raw as i64
            }
        }
    }

    pub fn acc_set(&mut self, i: usize, v: i64) {
        let raw = v as u64;
        if self.macsr & MACSR_FI != 0 {
            self.acc[i] = (raw >> 8) as u32;
            self.accext[i] = ((((raw >> 40) & 0xFF) << 8) | (raw & 0xFF)) as u16;
        } else {
            self.acc[i] = raw as u32;
            self.accext[i] = ((((raw >> 40) & 0xFF) << 8) | ((raw >> 32) & 0xFF)) as u16;
        }
    }

    fn mac_signed_int(&self) -> bool {
        self.macsr & (MACSR_FI | MACSR_SU) == 0
    }

    fn mac_word(&self, v: u32, upper: bool) -> u32 {
        if self.macsr & MACSR_FI != 0 {
            if upper { v & 0xFFFF_0000 } else { v << 16 }
        } else if self.mac_signed_int() {
            if upper { ((v as i32) >> 16) as u32 } else { v as u16 as i16 as i32 as u32 }
        } else if upper {
            v >> 16
        } else {
            v & 0xFFFF
        }
    }

    fn mac_product(&mut self, x: u32, y: u32) -> i64 {
        if self.macsr & MACSR_FI != 0 {
            let p = (((x as i32 as i64) * (y as i32 as i64)) as u64) << 1;
            let rem = p & 0xFF_FFFF;
            let mut f = p >> 24;
            if self.macsr & MACSR_RT != 0 && (rem > 0x80_0000 || (rem == 0x80_0000 && f & 1 != 0)) {
                f = f.wrapping_add(1);
            }
            f &= (1u64 << 40) - 1;
            if x == 0x8000_0000 && y == 0x8000_0000 {
                return f as i64;
            }
            ((f << 24) as i64) >> 24
        } else if self.mac_signed_int() {
            let p = (x as i32 as i64) * (y as i32 as i64);
            let res = (p << 24) >> 24;
            if res != p {
                self.macsr |= MACSR_V;
                if self.macsr & MACSR_OMC != 0 {
                    return if p < 0 { !(1i64 << 50) } else { 1i64 << 50 };
                }
            }
            res
        } else {
            let mut p = (x as u64) * (y as u64);
            if p & (0xFF_FFFFu64 << 40) != 0 {
                self.macsr |= MACSR_V;
                if self.macsr & MACSR_OMC != 0 {
                    p = 1u64 << 50;
                } else {
                    p &= (1u64 << 40) - 1;
                }
            }
            p as i64
        }
    }

    fn mac_saturate(&mut self, i: usize, sum: i64) -> i64 {
        let pav = MACSR_PAV0 << i;
        if self.macsr & MACSR_FI != 0 {
            let mut r = (sum << 16) >> 16;
            if r != sum {
                self.macsr |= MACSR_V;
            }
            if self.macsr & MACSR_V != 0 {
                self.macsr |= pav;
                if self.macsr & MACSR_OMC != 0 {
                    r = if sum < 0 { 0xFFFF_FF80_0000_0000u64 as i64 } else { 0x007F_FFFF_FF00 };
                }
            }
            r
        } else if self.mac_signed_int() {
            let mut r = (sum << 16) >> 16;
            if r != sum {
                self.macsr |= MACSR_V;
            }
            if self.macsr & MACSR_V != 0 {
                self.macsr |= pav;
                if self.macsr & MACSR_OMC != 0 {
                    r = if sum < 0 { -0x8000_0000 } else { 0x7FFF_FFFF };
                }
            }
            r
        } else {
            let mut v = sum as u64;
            if v & (0xFFFFu64 << 48) != 0 {
                self.macsr |= MACSR_V;
            }
            if self.macsr & MACSR_V != 0 {
                self.macsr |= pav;
                if self.macsr & MACSR_OMC != 0 {
                    v = if v > (1u64 << 53) { 0 } else { (1u64 << 48) - 1 };
                } else {
                    v &= (1u64 << 48) - 1;
                }
            }
            v as i64
        }
    }

    fn mac_set_flags(&mut self, i: usize) {
        let v = self.acc_get(i);
        if v == 0 {
            self.macsr |= MACSR_Z;
        } else if (v as u64) & (1u64 << 47) != 0 {
            self.macsr |= MACSR_N;
        }
        if self.macsr & (MACSR_PAV0 << i) != 0 {
            self.macsr |= MACSR_V;
        }
        let ev = if self.macsr & MACSR_FI != 0 {
            let t = v >> 39;
            t != 0 && t != -1
        } else if self.mac_signed_int() {
            let t = v >> 31;
            t != 0 && t != -1
        } else {
            (v as u64) >> 32 != 0
        };
        if ev {
            self.macsr |= MACSR_EV;
        }
    }

    fn mac_clear_flags(&mut self) {
        self.macsr &= !(MACSR_V | MACSR_Z | MACSR_N | MACSR_EV);
    }

    /// The value MOVE ACCn,Rx reads.
    fn mac_read(&self, i: usize) -> u32 {
        let v = self.acc_get(i);
        if self.macsr & MACSR_FI != 0 {
            if self.macsr & MACSR_SU != 0 {
                let rem = (v & 0xFF_FFFF) as u32;
                let mut q = v >> 24;
                if rem > 0x80_0000 || (rem == 0x80_0000 && q & 1 != 0) {
                    q += 1;
                }
                if self.macsr & MACSR_OMC != 0 && q != q as i16 as i64 {
                    q = if v < 0 { -0x8000 } else { 0x7FFF };
                }
                return (q as u32) & 0xFFFF;
            }
            let rem = (v & 0xFF) as u32;
            let mut q = v >> 8;
            if self.macsr & MACSR_RT != 0 && (rem > 0x80 || (rem == 0x80 && q & 1 != 0)) {
                q += 1;
            }
            if self.macsr & MACSR_OMC != 0 && q != q as i32 as i64 {
                q = if v < 0 { i32::MIN as i64 } else { i32::MAX as i64 };
            }
            return q as u32;
        }
        if self.macsr & MACSR_OMC == 0 {
            return v as u32;
        }
        if self.mac_signed_int() {
            if v == v as i32 as i64 {
                v as u32
            } else if v < 0 {
                0x8000_0000
            } else {
                0x7FFF_FFFF
            }
        } else if (v as u64) >> 32 == 0 {
            v as u32
        } else {
            0xFFFF_FFFF
        }
    }

    fn linea(&mut self, op: u16) {
        if op & 0x0100 == 0 {
            return self.mac(op);
        }
        let mode = (op >> 3) & 7;
        let reg = op & 7;
        if op & 0xF1C0 == 0xA140 {
            // MOV3Q #imm,<ea>
            let mut v = ((op >> 9) & 7) as u32;
            if v == 0 {
                v = 0xFFFF_FFFF;
            }
            let l = self.ea(mode, reg, 4);
            self.write_loc(l, 4, v);
            self.set_nz(v, 4);
            return;
        }
        if op == 0xA9C0 {
            // MOVE MACSR,CCR
            let m = self.macsr;
            let mut c = 0;
            if m & MACSR_N != 0 {
                c |= CF_N;
            }
            if m & MACSR_Z != 0 {
                c |= CF_Z;
            }
            if m & MACSR_V != 0 {
                c |= CF_V;
            }
            self.set_ccr(c);
            return;
        }
        let rx = |s: &Cpu, n: u16| -> u32 {
            let r = (n & 7) as usize;
            if n & 8 != 0 { s.a[r] } else { s.d[r] }
        };
        let set_rx = |s: &mut Cpu, n: u16, v: u32| {
            let r = (n & 7) as usize;
            if n & 8 != 0 {
                s.a[r] = v
            } else {
                s.d[r] = v
            }
        };
        if op & 0xF9B0 == 0xA180 {
            // MOVE.L ACCy,Rx / MOVCLR.L
            let i = ((op >> 9) & 3) as usize;
            let v = self.mac_read(i);
            set_rx(self, op & 0xF, v);
            if op & 0x40 != 0 {
                self.acc[i] = 0;
                self.accext[i] = 0;
                self.macsr &= !(MACSR_PAV0 << i);
            }
            return;
        }
        if op & 0xF9FC == 0xA110 {
            // MOVE.L ACCy,ACCx
            let src = (op & 3) as usize;
            let dst = ((op >> 9) & 3) as usize;
            self.acc[dst] = self.acc[src];
            self.accext[dst] = self.accext[src];
            self.mac_clear_flags();
            let pav_src = self.macsr & (MACSR_PAV0 << src) != 0;
            self.macsr &= !(MACSR_PAV0 << dst);
            if pav_src {
                self.macsr |= MACSR_PAV0 << dst;
            }
            self.mac_set_flags(dst);
            return;
        }
        if op & 0xFFF0 == 0xA980 {
            let v = self.macsr;
            set_rx(self, op & 0xF, v);
            return;
        }
        if op & 0xFFF0 == 0xAD80 {
            let v = self.mask;
            set_rx(self, op & 0xF, v);
            return;
        }
        if op & 0xFBF0 == 0xAB80 {
            // MOVE.L ACCext01/23,Rx
            let i = if op & 0x400 != 0 { 2 } else { 0 };
            let v = ((self.accext[i] as u32) << 16) | self.accext[i + 1] as u32;
            set_rx(self, op & 0xF, v);
            return;
        }
        // The rest take a source EA: Dn, An or #imm.
        let l = self.ea(mode, reg, 4);
        let val = self.read_loc(l, 4);
        let _ = rx;
        if op & 0xF9C0 == 0xA100 {
            // MOVE.L <ea>,ACCx
            let i = ((op >> 9) & 3) as usize;
            let v = if self.macsr & MACSR_FI != 0 {
                (val as i32 as i64) << 8
            } else if self.mac_signed_int() {
                val as i32 as i64
            } else {
                val as i64
            };
            self.acc_set(i, v);
            self.macsr &= !(MACSR_PAV0 << i);
            self.mac_clear_flags();
            self.mac_set_flags(i);
        } else if op & 0xFFC0 == 0xA900 {
            self.macsr = val & 0xFFF;
        } else if op & 0xFBC0 == 0xAB00 {
            let i = if op & 0x400 != 0 { 2 } else { 0 };
            self.accext[i] = (val >> 16) as u16;
            self.accext[i + 1] = val as u16;
        } else if op & 0xFFC0 == 0xAD00 {
            self.mask = val | 0xFFFF_0000;
        } else {
            self.illegal(VEC_LINEA);
        }
    }

    /// MAC / MSAC, with or without a parallel load.
    fn mac(&mut self, op: u16) {
        let ext = self.fetch16();
        let mut acc = (((op >> 7) & 1) | ((ext >> 3) & 2)) as usize;
        let load = op & 0x30 != 0;
        let (rx, ry, mut loadval, mut addr) = if load {
            let mode = (op >> 3) & 7;
            let reg = op & 7;
            // The EA without (An)+/-(An) side effects; applied after the MAC.
            let base = match mode {
                2 | 3 => self.a[reg as usize],
                4 => self.a[reg as usize].wrapping_sub(4),
                5 => {
                    let d = self.fetch16() as i16 as i32 as u32;
                    self.a[reg as usize].wrapping_add(d)
                }
                _ => return self.illegal(VEC_LINEA),
            };
            let a = if ext & 0x20 != 0 { base & self.mask } else { base };
            let v = self.bus.read32(a);
            acc ^= 1;
            let rxn = (ext >> 12) & 7;
            let rx = if ext & 0x8000 != 0 { self.a[rxn as usize] } else { self.d[rxn as usize] };
            let ry = if ext & 8 != 0 { self.a[(ext & 7) as usize] } else { self.d[(ext & 7) as usize] };
            (rx, ry, v, a)
        } else {
            let rxn = ((op >> 9) & 7) as usize;
            let rx = if op & 0x40 != 0 { self.a[rxn] } else { self.d[rxn] };
            let ryn = (op & 7) as usize;
            let ry = if op & 8 != 0 { self.a[ryn] } else { self.d[ryn] };
            (rx, ry, 0, 0)
        };
        let skip = self.macsr & MACSR_OMC != 0 && self.macsr & (MACSR_PAV0 << acc) != 0;
        if !skip {
            let (x, y) = if ext & 0x0800 == 0 {
                (self.mac_word(rx, ext & 0x80 != 0), self.mac_word(ry, ext & 0x40 != 0))
            } else {
                (rx, ry)
            };
            self.mac_clear_flags();
            let mut p = self.mac_product(x, y);
            match (ext >> 9) & 3 {
                1 => p = p.wrapping_shl(1),
                3 => {
                    p = if self.macsr & MACSR_SU != 0 && self.macsr & MACSR_FI == 0 {
                        ((p as u64) >> 1) as i64
                    } else {
                        p >> 1
                    }
                }
                _ => {}
            }
            let cur = self.acc_get(acc);
            let sum = if ext & 0x100 != 0 { cur.wrapping_sub(p) } else { cur.wrapping_add(p) };
            let r = self.mac_saturate(acc, sum);
            self.acc_set(acc, r);
            self.mac_set_flags(acc);
        }
        if load {
            let rw = ((op >> 9) & 7) as usize;
            if op & 0x40 != 0 {
                self.a[rw] = loadval;
            } else {
                self.d[rw] = loadval;
            }
            let reg = (op & 7) as usize;
            match (op >> 3) & 7 {
                3 => self.a[reg] = addr.wrapping_add(4),
                4 => self.a[reg] = addr,
                _ => {}
            }
            loadval = 0;
            addr = 0;
        }
        let _ = (loadval, addr);
    }
}
