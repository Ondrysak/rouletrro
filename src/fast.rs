//! A predecoded instruction cache for the MAIN OS text.
//!
//! Each instruction in the image is decoded once into an `Op`: a handler
//! and its operands resolved as far as they can be without running it --
//! extension words fetched, PC-relative addresses and branch targets made
//! absolute, immediates read. Executing it is then one indirect call.
//!
//! Correctness rests on the reference interpreter in `cpu.rs`: anything the
//! decoder does not recognise gets `h_slow`, which runs the instruction
//! there, and single-word instructions go straight to the interpreter's own
//! dispatch (they have nothing to predecode). A store into the cached range
//! invalidates the entries it could overlap (`Bus::check_code`).

use crate::cpu::{Cpu, Loc, CF_C, CF_N, CF_V, CF_X, CF_Z};

pub type Handler = fn(&mut Cpu, &Op);

#[derive(Clone, Copy, Debug)]
pub enum Ea {
    D(u8),
    A(u8),
    Ind(u8),
    Post(u8),
    Pre(u8),
    Disp(u8, i32),
    /// (d8, An, Xi) with the brief extension word.
    Idx(u8, u16),
    /// (d8, PC, Xi): the PC base, already absolute, and the extension word.
    PcIdx(u32, u16),
    Abs(u32),
    Imm(u32),
}

#[derive(Clone, Copy)]
pub struct Op {
    pub h: Handler,
    /// Length in bytes; the handler runs with PC already past it.
    pub len: u8,
    /// Ends its block: control flow, an SR write, or an interpreter fallback.
    pub end: bool,
    pub r: u8,
    pub op: u16,
    pub a: Ea,
    pub b: Ea,
    pub x: u32,
}

/// Anything not predecoded: the reference interpreter, from the top.
fn h_slow(c: &mut Cpu, _o: &Op) {
    c.pc = c.op_pc;
    c.execute();
}

/// A single-word instruction: the interpreter's dispatch without the fetch.
fn h_word(c: &mut Cpu, o: &Op) {
    c.dispatch(o.op);
}

// -- operand access -----------------------------------------------------------

#[inline(always)]
fn mask(sz: u32) -> u32 {
    match sz {
        1 => 0xFF,
        2 => 0xFFFF,
        _ => 0xFFFF_FFFF,
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

#[inline(always)]
fn idx(c: &Cpu, base: u32, ext: u16) -> u32 {
    let r = ((ext >> 12) & 7) as usize;
    let xi = if ext & 0x8000 != 0 { c.a[r] } else { c.d[r] };
    let xi = if ext & 0x0800 != 0 { xi } else { xi as u16 as i16 as i32 as u32 };
    base.wrapping_add(ext as u8 as i8 as i32 as u32).wrapping_add(xi << ((ext >> 9) & 3))
}

/// Resolve to a location, applying (An)+ / -(An).
#[inline(always)]
fn loc(c: &mut Cpu, e: Ea, sz: u32) -> Loc {
    match e {
        Ea::D(r) => Loc::D(r as usize),
        Ea::A(r) => Loc::A(r as usize),
        Ea::Ind(r) => Loc::M(c.a[r as usize]),
        Ea::Post(r) => {
            let a = c.a[r as usize];
            c.a[r as usize] = a.wrapping_add(sz);
            Loc::M(a)
        }
        Ea::Pre(r) => {
            let a = c.a[r as usize].wrapping_sub(sz);
            c.a[r as usize] = a;
            Loc::M(a)
        }
        Ea::Disp(r, d) => Loc::M(c.a[r as usize].wrapping_add(d as u32)),
        Ea::Idx(r, ext) => Loc::M(idx(c, c.a[r as usize], ext)),
        Ea::PcIdx(base, ext) => Loc::M(idx(c, base, ext)),
        Ea::Abs(a) => Loc::M(a),
        Ea::Imm(v) => Loc::I(v),
    }
}

#[inline(always)]
fn addr_of(c: &Cpu, e: Ea) -> u32 {
    match e {
        Ea::Ind(r) => c.a[r as usize],
        Ea::Disp(r, d) => c.a[r as usize].wrapping_add(d as u32),
        Ea::Idx(r, ext) => idx(c, c.a[r as usize], ext),
        Ea::PcIdx(base, ext) => idx(c, base, ext),
        Ea::Abs(a) => a,
        _ => 0,
    }
}

#[inline(always)]
fn read(c: &mut Cpu, e: Ea, sz: u32) -> u32 {
    match e {
        Ea::D(r) => c.d[r as usize] & mask(sz),
        Ea::A(r) => c.a[r as usize] & mask(sz),
        Ea::Imm(v) => v,
        _ => {
            let l = loc(c, e, sz);
            c.read_loc(l, sz)
        }
    }
}

// -- handlers -------------------------------------------------------------------

fn h_move<const SZ: u32>(c: &mut Cpu, o: &Op) {
    let v = read(c, o.a, SZ);
    let l = loc(c, o.b, SZ);
    c.write_loc(l, SZ, v);
    c.set_nz(v, SZ);
}

fn h_movea<const SZ: u32>(c: &mut Cpu, o: &Op) {
    let v = read(c, o.a, SZ);
    c.a[o.r as usize] = sext(v, SZ);
}

fn h_mvs<const SZ: u32>(c: &mut Cpu, o: &Op) {
    let v = sext(read(c, o.a, SZ), SZ);
    c.d[o.r as usize] = v;
    c.set_nz(v, 4);
}

fn h_mvz<const SZ: u32>(c: &mut Cpu, o: &Op) {
    let v = read(c, o.a, SZ);
    c.d[o.r as usize] = v;
    c.set_nz(v, 4);
}

fn h_moveq(c: &mut Cpu, o: &Op) {
    c.d[o.r as usize] = o.x;
    c.set_nz(o.x, 4);
}

fn h_mov3q(c: &mut Cpu, o: &Op) {
    let l = loc(c, o.b, 4);
    c.write_loc(l, 4, o.x);
    c.set_nz(o.x, 4);
}

fn h_lea(c: &mut Cpu, o: &Op) {
    c.a[o.r as usize] = addr_of(c, o.a);
}

fn h_pea(c: &mut Cpu, o: &Op) {
    let a = addr_of(c, o.a);
    c.push32(a);
}

fn h_jsr(c: &mut Cpu, o: &Op) {
    let t = addr_of(c, o.a);
    let ret = c.pc;
    c.push32(ret);
    c.pc = t;
}

fn h_jmp(c: &mut Cpu, o: &Op) {
    c.pc = addr_of(c, o.a);
}

fn h_bra(c: &mut Cpu, o: &Op) {
    c.pc = o.x;
}

fn h_bsr(c: &mut Cpu, o: &Op) {
    let ret = c.pc;
    c.push32(ret);
    c.pc = o.x;
}

fn h_bcc(c: &mut Cpu, o: &Op) {
    if c.cond(o.r as u16) {
        c.pc = o.x;
    }
}

fn h_link(c: &mut Cpu, o: &Op) {
    let r = o.r as usize;
    let v = c.a[r];
    c.push32(v);
    c.a[r] = c.a[7];
    c.a[7] = c.a[7].wrapping_add(o.x);
}

fn h_clr<const SZ: u32>(c: &mut Cpu, o: &Op) {
    let l = loc(c, o.b, SZ);
    c.write_loc(l, SZ, 0);
    c.sr = (c.sr & !0x0F) | CF_Z;
}

fn h_tst<const SZ: u32>(c: &mut Cpu, o: &Op) {
    let v = read(c, o.a, SZ);
    c.set_nz(v, SZ);
}

fn h_movem(c: &mut Cpu, o: &Op) {
    let mut a = addr_of(c, o.a);
    let m = o.x as u16;
    let to_mem = o.r != 0;
    for i in 0..16 {
        if m & (1 << i) == 0 {
            continue;
        }
        if to_mem {
            let v = if i < 8 { c.d[i] } else { c.a[i - 8] };
            c.bus.write32(a, v);
        } else {
            let v = c.bus.read32(a);
            if i < 8 {
                c.d[i] = v;
            } else {
                c.a[i - 8] = v;
            }
        }
        a = a.wrapping_add(4);
    }
}

/// MAC/MSAC. `x` packs the extension word (bits 15-0), Rx and Ry as
/// register numbers 0-15 (bits 19-16, 23-20) and the accumulator (25-24).
/// The two modes the audio engine runs in -- signed fractional and signed
/// integer, with or without OMC, no rounding -- are computed inline; the
/// rest go to the general `mac_core`.
fn h_mac(c: &mut Cpu, o: &Op) {
    use crate::cpu::{MACSR_FI, MACSR_OMC, MACSR_PAV0, MACSR_RT, MACSR_SU, MACSR_V};
    let m = c.macsr;
    let fast_mode = m & (MACSR_RT | MACSR_SU) == 0;
    if !fast_mode {
        return c.mac_core(o.op, o.x as u16, disp_of(o));
    }
    let ext = o.x as u16;
    let op = o.op;
    let acc = ((o.x >> 24) & 3) as usize;
    let rxn = ((o.x >> 16) & 15) as usize;
    let ryn = ((o.x >> 20) & 15) as usize;
    let rx = if rxn >= 8 { c.a[rxn - 8] } else { c.d[rxn] };
    let ry = if ryn >= 8 { c.a[ryn - 8] } else { c.d[ryn] };
    let load = op & 0x30 != 0;
    let mut addr = 0;
    let mut lv = 0;
    if load {
        let reg = (op & 7) as usize;
        let base = match (op >> 3) & 7 {
            2 | 3 => c.a[reg],
            4 => c.a[reg].wrapping_sub(4),
            _ => c.a[reg].wrapping_add(disp_of(o)),
        };
        addr = if ext & 0x20 != 0 { base & c.mask } else { base };
        lv = c.bus.read32(addr);
    }
    if !(m & MACSR_OMC != 0 && m & (MACSR_PAV0 << acc) != 0) {
        let fi = m & MACSR_FI != 0;
        let (x, y) = if ext & 0x0800 == 0 {
            if fi {
                (
                    if ext & 0x80 != 0 { rx & 0xFFFF_0000 } else { rx << 16 },
                    if ext & 0x40 != 0 { ry & 0xFFFF_0000 } else { ry << 16 },
                )
            } else {
                (
                    if ext & 0x80 != 0 { ((rx as i32) >> 16) as u32 } else { rx as u16 as i16 as i32 as u32 },
                    if ext & 0x40 != 0 { ((ry as i32) >> 16) as u32 } else { ry as u16 as i16 as i32 as u32 },
                )
            }
        } else {
            (rx, ry)
        };
        let mut flags = m & !0xF; // N Z V EV cleared
        let p: i64 = if fi {
            // (Y * X) << 1, truncated to product[63:24], sign-extended --
            // except -1 * -1, whose +1.0 is zero-filled.
            if x == 0x8000_0000 && y == 0x8000_0000 {
                1i64 << 39
            } else {
                (((x as i32 as i64) * (y as i32 as i64)) << 1) >> 24
            }
        } else {
            let p = (x as i32 as i64) * (y as i32 as i64);
            let mut p2 = (p << 24) >> 24;
            if p2 != p {
                flags |= MACSR_V;
                if m & MACSR_OMC != 0 {
                    p2 = if p < 0 { !(1i64 << 50) } else { 1i64 << 50 };
                }
            }
            match (ext >> 9) & 3 {
                1 => p2.wrapping_shl(1),
                3 => p2 >> 1,
                _ => p2,
            }
        };
        let cur = c.accv[acc];
        let sum = if ext & 0x100 != 0 { cur.wrapping_sub(p) } else { cur.wrapping_add(p) };
        let mut r = (sum << 16) >> 16;
        if r != sum {
            flags |= MACSR_V;
        }
        if flags & MACSR_V != 0 {
            flags |= MACSR_PAV0 << acc;
            if m & MACSR_OMC != 0 {
                r = if fi {
                    if sum < 0 { 0xFFFF_FF80_0000_0000u64 as i64 } else { 0x007F_FFFF_FF00 }
                } else if sum < 0 {
                    -0x8000_0000
                } else {
                    0x7FFF_FFFF
                };
            }
        }
        let v = (r << 16) >> 16;
        c.accv[acc] = v;
        if v == 0 {
            flags |= 0x4; // Z
        } else if v & (1i64 << 47) != 0 {
            flags |= 0x8; // N
        }
        if flags & (MACSR_PAV0 << acc) != 0 {
            flags |= MACSR_V;
        }
        let t = if fi { v >> 39 } else { v >> 31 };
        if t != 0 && t != -1 {
            flags |= 0x1; // EV
        }
        c.macsr = flags;
    }
    if load {
        let rw = ((op >> 9) & 7) as usize;
        if op & 0x40 != 0 {
            c.a[rw] = lv;
        } else {
            c.d[rw] = lv;
        }
        let reg = (op & 7) as usize;
        match (op >> 3) & 7 {
            3 => c.a[reg] = addr.wrapping_add(4),
            4 => c.a[reg] = addr,
            _ => {}
        }
    }
}

#[inline(always)]
fn disp_of(o: &Op) -> u32 {
    match o.a {
        Ea::Imm(d) => d,
        _ => 0,
    }
}

// The ALU families. OP: 0 add, 1 sub, 2 and, 3 or, 4 eor, 5 cmp.
#[inline(always)]
fn alu(c: &mut Cpu, op: u8, s: u32, d: u32, sz: u32) -> u32 {
    match op {
        0 => c.add_flags(s, d, sz, false),
        1 => c.sub_flags(s, d, sz, false),
        2 => {
            let v = s & d;
            c.set_nz(v, sz);
            v
        }
        3 => {
            let v = s | d;
            c.set_nz(v, sz);
            v
        }
        4 => {
            let v = s ^ d;
            c.set_nz(v, sz);
            v
        }
        _ => {
            c.cmp_flags(s, d, sz);
            d
        }
    }
}

/// <ea>,Dn
fn h_alu_ea_dn<const OP: u8, const SZ: u32>(c: &mut Cpu, o: &Op) {
    let s = read(c, o.a, SZ);
    let dn = o.r as usize;
    let r = alu(c, OP, s, c.d[dn], SZ);
    if OP != 5 {
        let m = mask(SZ);
        c.d[dn] = (c.d[dn] & !m) | (r & m);
    }
}

/// Dn,<ea>
fn h_alu_dn_ea<const OP: u8, const SZ: u32>(c: &mut Cpu, o: &Op) {
    let l = loc(c, o.b, SZ);
    let d = c.read_loc(l, SZ);
    let s = c.d[o.r as usize];
    let r = alu(c, OP, s, d, SZ);
    c.write_loc(l, SZ, r);
}

/// #imm,<ea> (ADDI/SUBI/ANDI/ORI/EORI/CMPI on Dn, ADDQ/SUBQ on any).
fn h_alu_imm<const OP: u8, const SZ: u32>(c: &mut Cpu, o: &Op) {
    let l = loc(c, o.b, SZ);
    let d = c.read_loc(l, SZ);
    let r = alu(c, OP, o.x, d, SZ);
    if OP != 5 {
        c.write_loc(l, SZ, r);
    }
}

/// ADDA/SUBA/CMPA <ea>,An. OP: 0 add, 1 sub, 5 cmp.
fn h_alu_an<const OP: u8, const SZ: u32>(c: &mut Cpu, o: &Op) {
    let s = sext(read(c, o.a, SZ), SZ);
    let an = o.r as usize;
    match OP {
        0 => c.a[an] = c.a[an].wrapping_add(s),
        1 => c.a[an] = c.a[an].wrapping_sub(s),
        _ => {
            let d = c.a[an];
            c.cmp_flags(s, d, 4);
        }
    }
}

fn h_quick_an(c: &mut Cpu, o: &Op) {
    let r = o.r as usize;
    c.a[r] = c.a[r].wrapping_add(o.x);
}

fn h_mul_w<const SIGNED: bool>(c: &mut Cpu, o: &Op) {
    let s = read(c, o.a, 2);
    let dn = o.r as usize;
    let r = if SIGNED {
        (c.d[dn] as u16 as i16 as i32).wrapping_mul(s as u16 as i16 as i32) as u32
    } else {
        (c.d[dn] & 0xFFFF).wrapping_mul(s & 0xFFFF)
    };
    c.d[dn] = r;
    c.set_nz(r, 4);
}

fn h_mul_l(c: &mut Cpu, o: &Op) {
    let s = read(c, o.a, 4);
    let dl = o.r as usize;
    let r = if o.x & 0x0800 != 0 {
        (c.d[dl] as i32).wrapping_mul(s as i32) as u32
    } else {
        c.d[dl].wrapping_mul(s)
    };
    c.d[dl] = r;
    c.set_nz(r, 4);
}

fn h_cmpi_dn<const SZ: u32>(c: &mut Cpu, o: &Op) {
    let d = c.d[o.r as usize];
    c.cmp_flags(o.x, d, SZ);
}

// -- decoding -------------------------------------------------------------------

struct Rd<'a> {
    c: &'a Cpu,
    p: u32,
}

impl Rd<'_> {
    fn w(&mut self) -> u16 {
        let v = self.c.bus.peek16(self.p);
        self.p = self.p.wrapping_add(2);
        v
    }
    fn l(&mut self) -> u32 {
        let v = self.c.bus.peek32(self.p);
        self.p = self.p.wrapping_add(4);
        v
    }
}

/// Decode an effective address. None for modes this cache does not handle.
fn ea(rd: &mut Rd, mode: u16, reg: u16, sz: u32) -> Option<Ea> {
    let r = reg as u8;
    Some(match mode {
        0 => Ea::D(r),
        1 => Ea::A(r),
        2 => Ea::Ind(r),
        3 => Ea::Post(r),
        4 => Ea::Pre(r),
        5 => Ea::Disp(r, rd.w() as i16 as i32),
        6 => Ea::Idx(r, rd.w()),
        _ => match reg {
            0 => Ea::Abs(rd.w() as i16 as i32 as u32),
            1 => Ea::Abs(rd.l()),
            2 => {
                let base = rd.p;
                Ea::Abs(base.wrapping_add(rd.w() as i16 as i32 as u32))
            }
            3 => {
                let base = rd.p;
                Ea::PcIdx(base, rd.w())
            }
            4 => Ea::Imm(match sz {
                1 => rd.w() as u32 & 0xFF,
                2 => rd.w() as u32,
                _ => rd.l(),
            }),
            _ => return None,
        },
    })
}

fn is_mem(e: Ea) -> bool {
    !matches!(e, Ea::D(_) | Ea::A(_) | Ea::Imm(_))
}

fn is_control(e: Ea) -> bool {
    matches!(e, Ea::Ind(_) | Ea::Disp(..) | Ea::Idx(..) | Ea::PcIdx(..) | Ea::Abs(_))
}

/// ColdFire moves A7 by the operand size for byte (A7)+/-(A7): nothing to adjust.
fn szf(s: u16) -> u32 {
    [1, 2, 4, 0][(s & 3) as usize]
}

pub fn decode(c: &Cpu, pc: u32) -> Op {
    let mut rd = Rd { c, p: pc };
    let op = rd.w();
    match decode_inner(&mut rd, op) {
        Some(mut o) => {
            o.op = op;
            o.len = (rd.p.wrapping_sub(pc)) as u8;
            o.end |= matches!(op >> 12, 0x6) || matches!(op & 0xFFC0, 0x4E80 | 0x4EC0);
            o
        }
        None if single_word(op) => {
            let mut o = mk(h_word);
            o.op = op;
            o.len = 2;
            // Returns, traps and SR writes change the flow or the mask.
            o.end = matches!(op, 0x4E73 | 0x4E75) || op & 0xFFF0 == 0x4E40 || op & 0xFFF8 == 0x46C0;
            o
        }
        None => {
            let mut o = mk(h_slow);
            o.op = op;
            o.len = 2;
            o.end = true;
            o
        }
    }
}

/// Decode a basic block at `pc` and store it. -> the block.
pub fn build_block(c: &mut Cpu, pc: u32) -> *const [Op] {
    let mut ops = Vec::with_capacity(8);
    let mut p = pc;
    loop {
        let op = decode(c, p);
        p = p.wrapping_add(op.len as u32);
        let end = op.end;
        ops.push(op);
        if end || ops.len() >= 32 || !c.bus.in_code_window(p) || c.hooked(p) {
            break;
        }
    }
    c.bus.block_store(pc, p, ops.into_boxed_slice())
}

fn mk(h: Handler) -> Op {
    Op { h, len: 0, end: false, r: 0, op: 0, a: Ea::D(0), b: Ea::D(0), x: 0 }
}

fn single_word(op: u16) -> bool {
    let mode = (op >> 3) & 7;
    match op >> 12 {
        0x0 => (op & 0x0100 != 0 && mode <= 4) || matches!(op & 0xFFF8, 0x00C0 | 0x02C0 | 0x04C0),
        0x5 => mode <= 4 && !matches!(op & 0x3F, 0x3A | 0x3B),
        0x7 => op & 0x100 == 0 || mode <= 4,
        0xE => true,
        0x8 | 0x9 | 0xB | 0xC | 0xD => mode <= 4,
        0x1..=0x3 => mode <= 4 && ((op >> 6) & 7) <= 4,
        0x4 => {
            matches!(op, 0x4E75 | 0x4E71 | 0x4E73)
                || op & 0xFFF8 == 0x4E58
                || op & 0xFFF0 == 0x4E40
                || matches!(op & 0xFFF8, 0x4840 | 0x4880 | 0x48C0 | 0x49C0 | 0x4480 | 0x4680 | 0x4080 | 0x40C0 | 0x42C0 | 0x44C0 | 0x46C0 | 0x4C80)
                || (matches!(op & 0xFF00, 0x4200 | 0x4A00) && (op >> 6) & 3 != 3 && mode <= 4)
        }
        0xA => op & 0xF9B0 == 0xA180 || (op & 0xF9C0 == 0xA100 && mode <= 1),
        _ => false,
    }
}

fn decode_inner(rd: &mut Rd, op: u16) -> Option<Op> {
    let mode = (op >> 3) & 7;
    let reg = op & 7;
    let rn = ((op >> 9) & 7) as u8;
    match op >> 12 {
        0x1..=0x3 => {
            let sz = match op >> 12 {
                1 => 1,
                2 => 4,
                _ => 2,
            };
            let a = ea(rd, mode, reg, sz)?;
            let dmode = (op >> 6) & 7;
            if dmode == 1 {
                let mut o = mk(if sz == 2 { h_movea::<2> } else { h_movea::<4> });
                o.a = a;
                o.r = rn;
                return Some(o);
            }
            let b = ea(rd, dmode, rn as u16, sz)?;
            if matches!(b, Ea::Imm(_) | Ea::A(_)) {
                return None;
            }
            let mut o = mk(match sz {
                1 => h_move::<1>,
                2 => h_move::<2>,
                _ => h_move::<4>,
            });
            o.a = a;
            o.b = b;
            return Some(o);
        }
        0x6 => {
            let base = rd.p;
            let d8 = op & 0xFF;
            let disp = match d8 {
                0 => rd.w() as i16 as i32 as u32,
                0xFF => rd.l(),
                _ => d8 as u8 as i8 as i32 as u32,
            };
            let cc = ((op >> 8) & 15) as u8;
            let mut o = mk(match cc {
                0 => h_bra,
                1 => h_bsr,
                _ => h_bcc,
            });
            o.x = base.wrapping_add(disp);
            o.r = cc;
            return Some(o);
        }
        _ => {}
    }
    match op >> 12 {
        0x0 => {
            let f = op & 0xFFF8;
            let r = reg as u8;
            let (h, imm): (Handler, u32) = match f {
                0x0080 => (h_alu_imm::<3, 4>, rd.l()),
                0x0280 => (h_alu_imm::<2, 4>, rd.l()),
                0x0480 => (h_alu_imm::<1, 4>, rd.l()),
                0x0680 => (h_alu_imm::<0, 4>, rd.l()),
                0x0A80 => (h_alu_imm::<4, 4>, rd.l()),
                0x0C00 => (h_cmpi_dn::<1>, rd.w() as u32 & 0xFF),
                0x0C40 => (h_cmpi_dn::<2>, rd.w() as u32),
                0x0C80 => (h_cmpi_dn::<4>, rd.l()),
                _ => return None,
            };
            let mut o = mk(h);
            o.x = imm;
            o.r = r;
            o.b = Ea::D(r);
            Some(o)
        }
        0x4 => {
            if op & 0x01C0 == 0x01C0 && op & 0xFFF8 != 0x49C0 {
                let a = ea(rd, mode, reg, 4)?;
                if !is_control(a) {
                    return None;
                }
                let mut o = mk(h_lea);
                o.a = a;
                o.r = rn;
                return Some(o);
            }
            match op & 0xFFC0 {
                0x4E80 | 0x4EC0 | 0x4840 => {
                    let a = ea(rd, mode, reg, 4)?;
                    if !is_control(a) {
                        return None;
                    }
                    let mut o = mk(match op & 0xFFC0 {
                        0x4E80 => h_jsr,
                        0x4EC0 => h_jmp,
                        _ => h_pea,
                    });
                    o.a = a;
                    return Some(o);
                }
                0x48C0 | 0x4CC0 if mode == 2 || mode == 5 => {
                    let m = rd.w();
                    let a = ea(rd, mode, reg, 4)?;
                    let mut o = mk(h_movem);
                    o.x = m as u32;
                    o.a = a;
                    o.r = (op & 0xFFC0 == 0x48C0) as u8;
                    return Some(o);
                }
                0x4C00 => {
                    let ext = rd.w();
                    if ext & 0x0400 != 0 {
                        return None;
                    }
                    let a = ea(rd, mode, reg, 4)?;
                    let mut o = mk(h_mul_l);
                    o.a = a;
                    o.x = ext as u32;
                    o.r = ((ext >> 12) & 7) as u8;
                    return Some(o);
                }
                _ => {}
            }
            if op & 0xFFF8 == 0x4E50 {
                let mut o = mk(h_link);
                o.r = reg as u8;
                o.x = rd.w() as i16 as i32 as u32;
                return Some(o);
            }
            let ss = (op >> 6) & 3;
            if ss != 3 && matches!(op & 0xFF00, 0x4200 | 0x4A00) {
                let sz = szf(ss);
                let e = ea(rd, mode, reg, sz)?;
                if matches!(e, Ea::A(_) | Ea::Imm(_)) {
                    return None;
                }
                let mut o = mk(match (op & 0xFF00, sz) {
                    (0x4200, 1) => h_clr::<1>,
                    (0x4200, 2) => h_clr::<2>,
                    (0x4200, _) => h_clr::<4>,
                    (_, 1) => h_tst::<1>,
                    (_, 2) => h_tst::<2>,
                    _ => h_tst::<4>,
                });
                o.a = e;
                o.b = e;
                return Some(o);
            }
            None
        }
        0x5 => {
            // ADDQ/SUBQ with an extension-word EA.
            if (op >> 6) & 3 == 3 {
                return None;
            }
            let sz = szf((op >> 6) & 3);
            let mut q = rn as u32;
            if q == 0 {
                q = 8;
            }
            let b = ea(rd, mode, reg, sz)?;
            if matches!(b, Ea::Imm(_)) {
                return None;
            }
            let sub = op & 0x100 != 0;
            if let Ea::A(r) = b {
                let mut o = mk(h_quick_an);
                o.r = r;
                o.x = if sub { q.wrapping_neg() } else { q };
                return Some(o);
            }
            let mut o = mk(match (sub, sz) {
                (false, 1) => h_alu_imm::<0, 1>,
                (false, 2) => h_alu_imm::<0, 2>,
                (false, _) => h_alu_imm::<0, 4>,
                (true, 1) => h_alu_imm::<1, 1>,
                (true, 2) => h_alu_imm::<1, 2>,
                (true, _) => h_alu_imm::<1, 4>,
            });
            o.x = q;
            o.b = b;
            Some(o)
        }
        0x7 => {
            if op & 0x100 == 0 {
                let mut o = mk(h_moveq);
                o.r = rn;
                o.x = op as u8 as i8 as i32 as u32;
                return Some(o);
            }
            let ss = (op >> 6) & 3;
            let sz = if ss & 1 == 0 { 1 } else { 2 };
            let a = ea(rd, mode, reg, sz)?;
            let mut o = mk(match ss {
                0 => h_mvs::<1>,
                1 => h_mvs::<2>,
                2 => h_mvz::<1>,
                _ => h_mvz::<2>,
            });
            o.a = a;
            o.r = rn;
            Some(o)
        }
        0x8 | 0x9 | 0xB | 0xC | 0xD => {
            let line = op >> 12;
            let opmode = (op >> 6) & 7;
            match (line, opmode) {
                (0xC, 3) | (0xC, 7) => {
                    let a = ea(rd, mode, reg, 2)?;
                    let mut o = mk(if opmode == 7 { h_mul_w::<true> } else { h_mul_w::<false> });
                    o.a = a;
                    o.r = rn;
                    return Some(o);
                }
                (0x8, 3) | (0x8, 7) => return None,
                _ => {}
            }
            let alu_op: u8 = match line {
                0xD => 0,
                0x9 => 1,
                0xC => 2,
                0x8 => 3,
                _ => 5,
            };
            if opmode == 3 || opmode == 7 {
                // ADDA/SUBA/CMPA
                if line == 0x8 || line == 0xC {
                    return None;
                }
                let sz = if opmode == 3 { 2 } else { 4 };
                let a = ea(rd, mode, reg, sz)?;
                let mut o = mk(match (alu_op, sz) {
                    (0, 2) => h_alu_an::<0, 2>,
                    (0, _) => h_alu_an::<0, 4>,
                    (1, 2) => h_alu_an::<1, 2>,
                    (1, _) => h_alu_an::<1, 4>,
                    (_, 2) => h_alu_an::<5, 2>,
                    _ => h_alu_an::<5, 4>,
                });
                o.a = a;
                o.r = rn;
                return Some(o);
            }
            if opmode <= 2 {
                let sz = szf(opmode);
                let a = ea(rd, mode, reg, sz)?;
                let h: Handler = match (alu_op, sz) {
                    (0, 1) => h_alu_ea_dn::<0, 1>,
                    (0, 2) => h_alu_ea_dn::<0, 2>,
                    (0, _) => h_alu_ea_dn::<0, 4>,
                    (1, 1) => h_alu_ea_dn::<1, 1>,
                    (1, 2) => h_alu_ea_dn::<1, 2>,
                    (1, _) => h_alu_ea_dn::<1, 4>,
                    (2, 1) => h_alu_ea_dn::<2, 1>,
                    (2, 2) => h_alu_ea_dn::<2, 2>,
                    (2, _) => h_alu_ea_dn::<2, 4>,
                    (3, 1) => h_alu_ea_dn::<3, 1>,
                    (3, 2) => h_alu_ea_dn::<3, 2>,
                    (3, _) => h_alu_ea_dn::<3, 4>,
                    (_, 1) => h_alu_ea_dn::<5, 1>,
                    (_, 2) => h_alu_ea_dn::<5, 2>,
                    _ => h_alu_ea_dn::<5, 4>,
                };
                let mut o = mk(h);
                o.a = a;
                o.r = rn;
                return Some(o);
            }
            // Dn,<ea> (opmode 4-6); line B is EOR.
            let sz = szf(opmode - 4);
            if mode <= 1 {
                return None;
            }
            let b = ea(rd, mode, reg, sz)?;
            if !is_mem(b) {
                return None;
            }
            let op2 = if line == 0xB { 4 } else { alu_op };
            let h: Handler = match (op2, sz) {
                (0, 1) => h_alu_dn_ea::<0, 1>,
                (0, 2) => h_alu_dn_ea::<0, 2>,
                (0, _) => h_alu_dn_ea::<0, 4>,
                (1, 1) => h_alu_dn_ea::<1, 1>,
                (1, 2) => h_alu_dn_ea::<1, 2>,
                (1, _) => h_alu_dn_ea::<1, 4>,
                (2, 1) => h_alu_dn_ea::<2, 1>,
                (2, 2) => h_alu_dn_ea::<2, 2>,
                (2, _) => h_alu_dn_ea::<2, 4>,
                (3, 1) => h_alu_dn_ea::<3, 1>,
                (3, 2) => h_alu_dn_ea::<3, 2>,
                (3, _) => h_alu_dn_ea::<3, 4>,
                (_, 1) => h_alu_dn_ea::<4, 1>,
                (_, 2) => h_alu_dn_ea::<4, 2>,
                _ => h_alu_dn_ea::<4, 4>,
            };
            let mut o = mk(h);
            o.b = b;
            o.r = rn;
            Some(o)
        }
        0xA => {
            if op & 0x0100 == 0 {
                // MAC/MSAC: the extension word, and a (d16,An) displacement.
                let ext = rd.w();
                let load = op & 0x30 != 0;
                if load && !matches!(mode, 2..=5) {
                    return None;
                }
                let disp = if load && mode == 5 { rd.w() as i16 as i32 as u32 } else { 0 };
                let mut acc = ((op >> 7) & 1) | ((ext >> 3) & 2);
                let (rx, ry) = if load {
                    acc ^= 1;
                    (((ext >> 12) & 7) | if ext & 0x8000 != 0 { 8 } else { 0 }, (ext & 7) | if ext & 8 != 0 { 8 } else { 0 })
                } else {
                    (((op >> 9) & 7) | if op & 0x40 != 0 { 8 } else { 0 }, (op & 7) | if op & 8 != 0 { 8 } else { 0 })
                };
                let mut o = mk(h_mac);
                o.x = ext as u32 | (rx as u32) << 16 | (ry as u32) << 20 | (acc as u32) << 24;
                o.a = Ea::Imm(disp);
                return Some(o);
            }
            if op & 0xF1C0 == 0xA140 {
                let b = ea(rd, mode, reg, 4)?;
                if matches!(b, Ea::Imm(_)) {
                    return None;
                }
                let mut v = rn as u32;
                if v == 0 {
                    v = 0xFFFF_FFFF;
                }
                let mut o = mk(h_mov3q);
                o.b = b;
                o.x = v;
                return Some(o);
            }
            None
        }
        _ => None,
    }
}

// Flags used above, re-exported so the handlers read like the interpreter.
#[allow(dead_code)]
const _FLAGS: [u16; 5] = [CF_C, CF_V, CF_Z, CF_N, CF_X];
