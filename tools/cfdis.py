#!/usr/bin/env python3
"""Disassemble MAIN OS: python3 tools/cfdis.py START [END|+N]  (hex addresses)

Capstone's m68k decoder with the ColdFire-only opcodes it lacks (MVS/MVZ,
FF1, MOV3Q, the EMAC) patched in, so a sweep does not desynchronise.
"""
import struct, sys
from capstone import Cs, CS_ARCH_M68K, CS_MODE_BIG_ENDIAN, CS_MODE_M68K_040

IMG = open('sections/section_3_MAIN_OS.bin', 'rb').read()
BASE = 0x40000400
md = Cs(CS_ARCH_M68K, CS_MODE_BIG_ENDIAN | CS_MODE_M68K_040)


def w(a):
    return struct.unpack_from('>H', IMG, a - BASE)[0]


def cf_special(a):
    op = w(a)
    if op & 0xF100 == 0x7100:
        ss = (op >> 6) & 3
        mn = ['mvs.b', 'mvs.w', 'mvz.b', 'mvz.w'][ss]
        mode, reg = (op >> 3) & 7, op & 7
        dn = (op >> 9) & 7
        if mode == 0:
            return mn, 'd%d,d%d' % (reg, dn), 2
        if mode == 5:
            return mn, '$%x(a%d),d%d' % (struct.unpack('>h', struct.pack('>H', w(a + 2)))[0], reg, dn), 4
        if mode == 7 and reg == 1:
            return mn, '$%08x,d%d' % ((w(a + 2) << 16) | w(a + 4), dn), 6
        if mode == 7 and reg == 4:
            return mn, '#$%x,d%d' % (w(a + 2), dn), 4
        if mode in (2, 3, 4):
            return mn, ['', '', '(a%d)', '(a%d)+', '-(a%d)'][mode] % reg + ',d%d' % dn, 2
    if op & 0xFFF8 == 0x04C0:
        return 'ff1', 'd%d' % (op & 7), 2
    if op & 0xF1C0 == 0xA140:
        return 'mov3q', '#%d,<ea %02x>' % ((op >> 9) & 7 or -1, op & 0x3F), 2
    if op & 0xF000 == 0xA000:
        return 'emac', '$%04x %04x' % (op, w(a + 2)), 4
    return None


def dis(start, end):
    a = start
    while a < end:
        sp = cf_special(a)
        if sp:
            print('%08x  %-9s %s' % (a, sp[0], sp[1]))
            a += sp[2]
            continue
        off = a - BASE
        ins = next(md.disasm(IMG[off:off + 10], a), None)
        if ins is None:
            print('%08x  .word     $%04x' % (a, w(a)))
            a += 2
            continue
        print('%08x  %-9s %s' % (a, ins.mnemonic, ins.op_str))
        a += ins.size


if __name__ == '__main__':
    s = int(sys.argv[1], 16)
    e = sys.argv[2] if len(sys.argv) > 2 else '+60'
    e = s + int(e[1:], 16) if e.startswith('+') else int(e, 16)
    dis(s, e)
