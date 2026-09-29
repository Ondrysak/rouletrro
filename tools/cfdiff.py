#!/usr/bin/env python3
"""Differential test of the Rust ColdFire core against Unicorn's CFV4E.

    python3 tools/cfdiff.py [N] [--seed S] [--only PREFIX]

Generates N random single-instruction cases from encoding templates, runs
each on Unicorn and on target/release/cfdiff, and reports disagreements.
Condition codes are read by the guest (`move.w sr,d7` appended to every
case), because Unicorn's SR register read returns stale lazy flags.

Unicorn is an oracle with known holes, not ground truth: it lacks FF1,
BITREV and BYTEREV on CFV4E, and its EMAC is wrong (see the reference
emulator's patches). Those instructions are not generated here; the EMAC
has its own tests in the crate.
"""
import random
import struct
import subprocess
import sys
from collections import Counter

from unicorn import Uc, UcError, UC_ARCH_M68K, UC_MODE_BIG_ENDIAN, UC_HOOK_INTR
from unicorn.m68k_const import *

CODE, DATA, DATA_LEN = 0x40001000, 0x40100000, 0x10000
STACK = 0x40108000
MARK = 0x40000800
DREGS = [UC_M68K_REG_D0 + i for i in range(8)]
AREGS = [UC_M68K_REG_A0 + i for i in range(8)]

SPECIAL = [0, 1, 2, 0xFFFFFFFF, 0x80000000, 0x7FFFFFFF, 0xFFFF, 0x8000, 0x7FFF,
           0xFF, 0x80, 0x7F, 0x10000, 0xFFFF8000, 0xFFFFFF80, 31, 32, 33, 63]


def rnd32(r):
    k = r.random()
    if k < 0.35:
        return r.choice(SPECIAL)
    if k < 0.6:
        return r.randrange(0, 256)
    return r.getrandbits(32)


def w(v):
    return struct.pack('>H', v & 0xFFFF)


def l(v):
    return struct.pack('>I', v & 0xFFFFFFFF)


class Gen:
    """Builds one instruction. Address registers used as bases always point
    into the data window, so every memory operand is mapped."""

    def __init__(self, r):
        self.r = r

    def ea(self, sz, allow=('d', 'a', 'ind', 'post', 'pre', 'd16', 'idx', 'imm', 'absw', 'absl', 'pcd16')):
        r = self.r
        kind = r.choice(allow)
        reg = r.randrange(8)
        if kind in ('ind', 'post', 'pre', 'd16', 'idx'):
            reg = r.choice([0, 1, 2, 3, 4, 5, 7])   # a0-a5 and the stack point at data
        if kind == 'd':
            return (0 << 3) | reg, b''
        if kind == 'a':
            return (1 << 3) | reg, b''
        if kind == 'ind':
            return (2 << 3) | reg, b''
        if kind == 'post':
            return (3 << 3) | reg, b''
        if kind == 'pre':
            return (4 << 3) | reg, b''
        if kind == 'd16':
            return (5 << 3) | reg, w(r.randrange(-0x80, 0x80) & ~1)
        if kind == 'idx':
            xi = r.randrange(8) & 0x7          # d0-d7, small values chosen below
            ext = (xi << 12) | 0x0800 | (r.randrange(3) << 9) | (r.randrange(-0x40, 0x40) & 0xFF)
            return (6 << 3) | reg, w(ext)
        if kind == 'absw':
            return (7 << 3) | 0, w(0x8000)     # 0xFFFF8000: not mapped -> skipped
        if kind == 'absl':
            return (7 << 3) | 1, l(DATA + r.randrange(0, 0x100, 2))
        if kind == 'pcd16':
            return (7 << 3) | 2, w(r.randrange(0, 0x40, 2))
        if kind == 'imm':
            if sz == 4:
                return (7 << 3) | 4, l(rnd32(r))
            return (7 << 3) | 4, w(rnd32(r) & (0xFF if sz == 1 else 0xFFFF))
        raise ValueError(kind)

    MEM = ('ind', 'post', 'pre', 'd16', 'idx', 'absl')
    ALT = ('d',) + MEM
    ANY = ('d', 'a') + MEM + ('imm', 'pcd16')
    DATA_ = ('d',) + MEM + ('imm', 'pcd16')

    def one(self):
        r = self.r
        t = r.choice(self.TEMPLATES)
        return t(self)

    def t_move(self):
        r = self.r
        sz = r.choice([1, 2, 4])
        szf = {1: 1, 2: 3, 4: 2}[sz]
        s, se = self.ea(sz, self.ANY if sz != 1 else self.DATA_)
        dmode = r.choice([0, 2, 3, 4, 5])
        dreg = r.randrange(8) if dmode == 0 else r.randrange(6)
        de = b''
        if dmode == 5:
            de = w(r.randrange(-0x40, 0x40) & ~1)
        return 'move', w((szf << 12) | (dreg << 9) | (dmode << 6) | s) + se + de

    def t_movea(self):
        r = self.r
        sz = r.choice([2, 4])
        szf = {2: 3, 4: 2}[sz]
        s, se = self.ea(sz, self.ANY)
        return 'movea', w((szf << 12) | (r.randrange(8) << 9) | (1 << 6) | s) + se

    def t_moveq(self):
        return 'moveq', w(0x7000 | (self.r.randrange(8) << 9) | self.r.randrange(256))

    def t_mvsz(self):
        r = self.r
        ss = r.randrange(4)
        s, se = self.ea(1 if ss in (0, 2) else 2, self.DATA_)
        return 'mvsz', w(0x7100 | (r.randrange(8) << 9) | (ss << 6) | s) + se

    def t_mov3q(self):
        r = self.r
        s, se = self.ea(4, self.ALT + ('a',))
        return 'mov3q', w(0xA140 | (r.randrange(8) << 9) | s) + se

    def t_arith(self):
        r = self.r
        line = r.choice([0x8, 0x9, 0xB, 0xC, 0xD])
        dn = r.randrange(8)
        if line == 0xB:
            opm = r.choice([0, 1, 2, 3, 7, 6])       # cmp.b/w/l cmpa.w/l eor.l
        elif line in (0x9, 0xD):
            opm = r.choice([0, 1, 2, 3, 7, 6])       # ea->Dn (b/w/l), adda/suba.w/l, Dn->ea .l
        else:
            opm = r.choice([0, 1, 2, 6])
        if opm in (0, 1, 2):
            sz = [1, 2, 4][opm]
            s, se = self.ea(sz, self.ANY if sz != 1 else self.DATA_)
        elif opm in (3, 7):
            sz = 2 if opm == 3 else 4
            s, se = self.ea(sz, self.ANY)
        else:
            s, se = self.ea(4, self.MEM if line != 0xB else self.ALT)
        return 'line%x.%d' % (line, opm), w((line << 12) | (dn << 9) | (opm << 6) | s) + se

    def t_addsubx(self):
        r = self.r
        line = r.choice([0x9, 0xD])
        return 'addsubx', w((line << 12) | (r.randrange(8) << 9) | 0x180 | r.randrange(8))

    def t_immop(self):
        r = self.r
        base = r.choice([0x0080, 0x0280, 0x0480, 0x0680, 0x0A80])
        return 'imm%04x' % base, w(base | r.randrange(8)) + l(rnd32(r))

    def t_cmpi(self):
        r = self.r
        ss = r.randrange(3)
        imm = l(rnd32(r)) if ss == 2 else w(rnd32(r) & (0xFF if ss == 0 else 0xFFFF))
        return 'cmpi', w(0x0C00 | (ss << 6) | r.randrange(8)) + imm

    def t_quick(self):
        r = self.r
        sub = r.randrange(2)
        s, se = self.ea(4, self.ALT + ('a',))
        return 'addsubq', w(0x5000 | (r.randrange(8) << 9) | (sub << 8) | (2 << 6) | s) + se

    def t_unary(self):
        r = self.r
        op = r.choice([0x4080, 0x4480, 0x4680, 0x4840, 0x4880, 0x48C0, 0x49C0])
        return 'unary%04x' % op, w(op | r.randrange(8))

    def t_clrtst(self):
        r = self.r
        base = r.choice([0x4200, 0x4A00])
        ss = r.randrange(3)
        s, se = self.ea([1, 2, 4][ss], self.ALT)
        return 'clrtst', w(base | (ss << 6) | s) + se

    def t_shift(self):
        r = self.r
        ir = r.randrange(2)
        return 'shift', w(0xE000 | (r.randrange(8) << 9) | (r.randrange(2) << 8) | (2 << 6) |
                         (ir << 5) | (r.randrange(2) << 3) | r.randrange(8))

    def t_mulw(self):
        r = self.r
        s, se = self.ea(2, self.DATA_)
        return 'mulw', w(0xC000 | (r.randrange(8) << 9) | (r.choice([3, 7]) << 6) | s) + se

    def t_divw(self):
        r = self.r
        s, se = self.ea(2, ('d', 'imm'))
        return 'divw', w(0x8000 | (r.randrange(8) << 9) | (r.choice([3, 7]) << 6) | s) + se

    def t_mull(self):
        r = self.r
        s, se = self.ea(4, ('d', 'ind', 'd16', 'post', 'pre'))
        ext = (r.randrange(8) << 12) | (r.randrange(2) << 11)
        return 'mull', w(0x4C00 | s) + w(ext) + se

    def t_divl(self):
        r = self.r
        s, se = self.ea(4, ('d', 'ind', 'd16'))
        dq = r.randrange(8)
        dr = dq if r.random() < 0.5 else r.randrange(8)
        ext = (dq << 12) | (r.randrange(2) << 11) | dr
        return 'divl', w(0x4C40 | s) + w(ext) + se

    def t_bit(self):
        r = self.r
        kind = r.randrange(4)
        s, se = self.ea(1, self.ALT if kind else self.DATA_[:-2])
        if r.randrange(2):
            return 'bitdyn', w(0x0100 | (r.randrange(8) << 9) | (kind << 6) | s) + se
        return 'bitimm', w(0x0800 | (kind << 6) | s) + w(r.randrange(64)) + se

    def t_scc(self):
        r = self.r
        return 'scc', w(0x50C0 | (r.randrange(16) << 8) | r.randrange(8))

    def t_lea(self):
        r = self.r
        s, se = self.ea(4, ('ind', 'd16', 'idx', 'absl', 'pcd16'))
        return 'lea', w(0x41C0 | (r.randrange(8) << 9) | s) + se

    def t_pea(self):
        s, se = self.ea(4, ('ind', 'd16', 'idx', 'absl', 'pcd16'))
        return 'pea', w(0x4840 | s) + se

    def t_link(self):
        r = self.r
        reg = r.randrange(6)
        return 'linkunlk', w(0x4E50 | reg) + w(-16 & 0xFFFF) + w(0x4E58 | reg)

    def t_movem(self):
        r = self.r
        s, se = self.ea(4, ('ind', 'd16'))
        mask = r.getrandbits(16) & ~(0x3F00 & (0xFFFF))    # never load a0-a5... keep bases
        mask &= 0xC0FF if r.randrange(2) else 0xFFFF
        dirn = r.randrange(2)
        if dirn:
            mask &= 0x00FF | 0x4000     # loading into An would break later bases: d-regs + a6
        return 'movem', w((0x4CC0 if dirn else 0x48C0) | s) + w(mask) + se

    def t_tas(self):
        s, se = self.ea(1, self.MEM)
        return 'tas', w(0x4AC0 | s) + se

    def t_ccr(self):
        r = self.r
        if r.randrange(2):
            s, se = self.ea(2, ('d', 'imm'))
            return 'toccr', w(0x44C0 | s) + se
        return 'fromccr', w(0x42C0 | r.randrange(8))

    def t_sats(self):
        return 'sats', w(0x4C80 | self.r.randrange(8))

    def t_bcc(self):
        r = self.r
        cc = r.randrange(2, 16)
        # taken -> skip a moveq that would set d6 to 0x55
        return 'bcc', w(0x6000 | (cc << 8) | 2) + w(0x7C55)

    TEMPLATES = [t_move, t_move, t_movea, t_moveq, t_mvsz, t_mov3q, t_arith, t_arith,
                 t_arith, t_addsubx, t_immop, t_cmpi, t_quick, t_unary, t_clrtst, t_shift,
                 t_mulw, t_divw, t_mull, t_divl, t_bit, t_scc, t_lea, t_pea, t_link,
                 t_movem, t_tas, t_ccr, t_sats, t_bcc]


def pattern_bytes():
    return bytes(((i * 37 + 11) & 0xFF) for i in range(DATA_LEN))


def make_case(r):
    g = Gen(r)
    name, code = g.one()
    code += w(0x40C7)                        # move.w sr,d7
    d = [rnd32(r) for _ in range(8)]
    # Index registers used by (d8,An,Xi) stay small so the address is mapped.
    for i in range(8):
        if r.random() < 0.5:
            d[i] = r.randrange(0, 0x40) & ~1
    a = [DATA + 0x4000 + r.randrange(0, 0x100) * 4 for _ in range(6)] + \
        [DATA + 0x6000 + r.randrange(0, 0x40) * 4, STACK]
    sr = 0x2700 | r.randrange(32)
    return name, code, d, a, sr


def run_unicorn(code, d, a, sr, data):
    uc = Uc(UC_ARCH_M68K, UC_MODE_BIG_ENDIAN)
    uc.ctl_set_cpu_model(UC_CPU_M68K_CFV4E)
    uc.mem_map(0x40000000, 0x200000)
    uc.mem_write(DATA, data)
    uc.mem_write(CODE, code)
    exc = []
    uc.hook_add(UC_HOOK_INTR, lambda u, n, _: (exc.append(n), u.emu_stop()))
    # SR before A7: Unicorn banks SSP/USP, so switching mode after setting
    # the stack pointer writes the register the CPU is about to stop using.
    uc.reg_write(UC_M68K_REG_SR, sr)
    for i in range(8):
        uc.reg_write(DREGS[i], d[i])
        uc.reg_write(AREGS[i], a[i])
    try:
        uc.emu_start(CODE, CODE + len(code), count=64)
    except UcError as e:
        return ('ERR', str(e))
    if exc:
        return ('EXC', exc[0])
    regs = [uc.reg_read(r) & 0xFFFFFFFF for r in DREGS + AREGS]
    pc = uc.reg_read(UC_M68K_REG_PC)
    mem = uc.mem_read(DATA, DATA_LEN)
    s = 0
    for b in mem:
        s = (s * 31 + b) & 0xFFFFFFFF
    return ('OK', regs, pc, s)


def main():
    n = 3000
    seed = 1
    only = None
    args = sys.argv[1:]
    if args and not args[0].startswith('-'):
        n = int(args[0])
    if '--seed' in args:
        seed = int(args[args.index('--seed') + 1])
    if '--only' in args:
        only = args[args.index('--only') + 1]
    r = random.Random(seed)
    cases = []
    while len(cases) < n:
        c = make_case(r)
        if only and not c[0].startswith(only):
            continue
        cases.append(c)
    data = pattern_bytes()
    lines = []
    for name, code, d, a, sr in cases:
        lines.append('%s %s %s %04x' % (code.hex(), ' '.join('%08x' % v for v in d),
                                        ' '.join('%08x' % v for v in a), sr))
    out = subprocess.run(['target/release/cfdiff'], input='\n'.join(lines) + '\n',
                         capture_output=True, text=True, check=True).stdout.splitlines()
    bad = Counter()
    shown = Counter()
    skipped = 0
    for (name, code, d, a, sr), got in zip(cases, out):
        ref = run_unicorn(code, d, a, sr, data)
        if ref[0] in ('ERR', 'EXC'):
            skipped += 1
            continue
        _, regs, pc, s = ref
        want = '%s %08x %08x' % (' '.join('%08x' % v for v in regs), pc, s)
        if got != want:
            bad[name] += 1
            if shown[name] < 3:
                shown[name] += 1
                print('MISMATCH %s code=%s sr=%04x' % (name, code.hex(), sr))
                print('   in  d=%s a=%s' % (' '.join('%08x' % v for v in d), ' '.join('%08x' % v for v in a)))
                g, wv = got.split(), want.split()
                names = ['d%d' % i for i in range(8)] + ['a%d' % i for i in range(8)] + ['pc', 'mem']
                diffs = ['%s: rust=%s uc=%s' % (nm, x, y) for nm, x, y in zip(names, g, wv) if x != y]
                print('   ' + '; '.join(diffs) if len(g) == len(wv) else '   rust=%s' % got)
    total = len(cases) - skipped
    print('%d cases, %d skipped (unicorn fault/exception), %d mismatches' % (total, skipped, sum(bad.values())))
    for k, v in bad.most_common():
        print('  %-14s %d' % (k, v))


if __name__ == '__main__':
    main()
