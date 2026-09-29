//! The address space: DDR, internal SRAM, sparse backing for everything else,
//! and the MMIO dispatch into `crate::io::Io`.
//!
//! MCF54418RM Table 1-2 gives the DDR controller 0x4000_0000-0x7FFF_FFFF.
//! The controller decodes only the address bits the fitted part needs, so the
//! 128 MB repeats through the whole window, and the firmware relies on it:
//! it keeps its stack at 0x47FFxxxx and reads DMA buffers through 0x48000000
//! and up, which ACR0 leaves cache-inhibited -- an uncached view of the same
//! memory. The 64 KB internal SRAM likewise repeats through 0x80000000-
//! 0x8BFFFFFF.
//!
//! Every other address that no peripheral model claims reads back what was
//! last written to it (zero at first), a page at a time. That matches how the
//! reference emulator ran: an unmodelled register page is plain memory. The
//! pages touched that way are recorded, because an unmodelled register the
//! firmware waits on looks exactly like a firmware bug.

use crate::io::Io;
use std::collections::{BTreeMap, HashMap};

pub const DDR_BASE: u32 = 0x4000_0000;
pub const SRAM_BASE: u32 = 0x8000_0000;
pub const SRAM_SIZE: usize = 0x1_0000;
const SPARSE_PAGE: u32 = 0x1000;

pub struct Bus {
    pub ddr: Vec<u8>,
    ddr_mask: u32,
    pub sram: Vec<u8>,
    pub sparse: HashMap<u32, Box<[u8; SPARSE_PAGE as usize]>>,
    /// Sparse pages first touched, page base -> (first pc, access count).
    pub unmapped_touch: BTreeMap<u32, (u32, u64)>,
    pub io: Io,
    /// The PC of the instruction being executed, for diagnostics.
    pub pc: u32,
    /// Set when a write lands in watched code, so any decoded cache is flushed.
    pub watch: Option<(u32, u32)>,
    pub watch_hits: u64,
}

impl Bus {
    pub fn new(ddr_mb: usize) -> Bus {
        let size = ddr_mb << 20;
        assert!(size.is_power_of_two());
        Bus {
            ddr: vec![0; size],
            ddr_mask: (size - 1) as u32,
            sram: vec![0; SRAM_SIZE],
            sparse: HashMap::new(),
            unmapped_touch: BTreeMap::new(),
            io: Io::new(),
            pc: 0,
            watch: None,
            watch_hits: 0,
        }
    }

    #[inline(always)]
    pub fn ddr_off(&self, addr: u32) -> Option<usize> {
        if addr & 0xC000_0000 == 0x4000_0000 {
            Some((addr & self.ddr_mask) as usize)
        } else {
            None
        }
    }

    #[inline(always)]
    fn is_io(addr: u32) -> bool {
        addr >= 0xE000_0000
    }

    fn sparse_page(&mut self, addr: u32) -> &mut [u8; SPARSE_PAGE as usize] {
        let base = addr & !(SPARSE_PAGE - 1);
        let pc = self.pc;
        self.sparse.entry(base).or_insert_with(|| Box::new([0; SPARSE_PAGE as usize]))
            as &mut _;
        let e = self.unmapped_touch.entry(base & !0xFFF).or_insert((pc, 0));
        e.1 += 1;
        self.sparse.get_mut(&base).unwrap()
    }

    /// Plain-memory byte read, bypassing peripherals.
    pub fn peek8(&self, addr: u32) -> u8 {
        if let Some(o) = self.ddr_off(addr) {
            return self.ddr[o];
        }
        if (SRAM_BASE..0x8C00_0000).contains(&addr) {
            return self.sram[(addr as usize) & (SRAM_SIZE - 1)];
        }
        let base = addr & !(SPARSE_PAGE - 1);
        match self.sparse.get(&base) {
            Some(p) => p[(addr - base) as usize],
            None => 0,
        }
    }

    /// Plain-memory byte write, bypassing peripherals.
    pub fn poke8(&mut self, addr: u32, v: u8) {
        if let Some(o) = self.ddr_off(addr) {
            self.ddr[o] = v;
            return;
        }
        if (SRAM_BASE..0x8C00_0000).contains(&addr) {
            self.sram[(addr as usize) & (SRAM_SIZE - 1)] = v;
            return;
        }
        let base = addr & !(SPARSE_PAGE - 1);
        let p = self.sparse.entry(base).or_insert_with(|| Box::new([0; SPARSE_PAGE as usize]));
        p[(addr - base) as usize] = v;
    }

    pub fn peek16(&self, a: u32) -> u16 {
        (self.peek8(a) as u16) << 8 | self.peek8(a.wrapping_add(1)) as u16
    }

    pub fn peek32(&self, a: u32) -> u32 {
        if let Some(o) = self.ddr_off(a) {
            if o + 4 <= self.ddr.len() {
                return u32::from_be_bytes(self.ddr[o..o + 4].try_into().unwrap());
            }
        }
        (self.peek16(a) as u32) << 16 | self.peek16(a.wrapping_add(2)) as u32
    }

    pub fn poke16(&mut self, a: u32, v: u16) {
        self.poke8(a, (v >> 8) as u8);
        self.poke8(a.wrapping_add(1), v as u8);
    }

    pub fn poke32(&mut self, a: u32, v: u32) {
        self.poke16(a, (v >> 16) as u16);
        self.poke16(a.wrapping_add(2), v as u16);
    }

    pub fn peek_bytes(&self, a: u32, n: usize) -> Vec<u8> {
        (0..n as u32).map(|i| self.peek8(a.wrapping_add(i))).collect()
    }

    pub fn poke_bytes(&mut self, a: u32, data: &[u8]) {
        if let Some(o) = self.ddr_off(a) {
            if o + data.len() <= self.ddr.len() {
                self.ddr[o..o + data.len()].copy_from_slice(data);
                return;
            }
        }
        for (i, &b) in data.iter().enumerate() {
            self.poke8(a.wrapping_add(i as u32), b);
        }
    }

    // -- CPU accesses --------------------------------------------------------

    #[inline(always)]
    pub fn read8(&mut self, a: u32) -> u8 {
        if let Some(o) = self.ddr_off(a) {
            return self.ddr[o];
        }
        self.read_slow(a, 1) as u8
    }

    #[inline(always)]
    pub fn read16(&mut self, a: u32) -> u16 {
        if let Some(o) = self.ddr_off(a) {
            if o + 2 <= self.ddr.len() {
                return u16::from_be_bytes([self.ddr[o], self.ddr[o + 1]]);
            }
        }
        self.read_slow(a, 2) as u16
    }

    #[inline(always)]
    pub fn read32(&mut self, a: u32) -> u32 {
        if let Some(o) = self.ddr_off(a) {
            if o + 4 <= self.ddr.len() {
                return u32::from_be_bytes(self.ddr[o..o + 4].try_into().unwrap());
            }
        }
        self.read_slow(a, 4)
    }

    #[inline(always)]
    pub fn write8(&mut self, a: u32, v: u8) {
        if let Some(o) = self.ddr_off(a) {
            self.check_watch(a, 1);
            self.ddr[o] = v;
            return;
        }
        self.write_slow(a, 1, v as u32)
    }

    #[inline(always)]
    pub fn write16(&mut self, a: u32, v: u16) {
        if let Some(o) = self.ddr_off(a) {
            if o + 2 <= self.ddr.len() {
                self.check_watch(a, 2);
                self.ddr[o..o + 2].copy_from_slice(&v.to_be_bytes());
                return;
            }
        }
        self.write_slow(a, 2, v as u32)
    }

    #[inline(always)]
    pub fn write32(&mut self, a: u32, v: u32) {
        if let Some(o) = self.ddr_off(a) {
            if o + 4 <= self.ddr.len() {
                self.check_watch(a, 4);
                self.ddr[o..o + 4].copy_from_slice(&v.to_be_bytes());
                return;
            }
        }
        self.write_slow(a, 4, v)
    }

    #[inline(always)]
    fn check_watch(&mut self, a: u32, n: u32) {
        if let Some((lo, hi)) = self.watch {
            let o = a & self.ddr_mask;
            if o + n > lo && o < hi {
                self.watch_hits += 1;
            }
        }
    }

    fn plain_read(&mut self, a: u32, size: u32) -> u32 {
        let mut v = 0u32;
        for i in 0..size {
            let x = a.wrapping_add(i);
            let b = if let Some(o) = self.ddr_off(x) {
                self.ddr[o]
            } else if (SRAM_BASE..0x8C00_0000).contains(&x) {
                self.sram[(x as usize) & (SRAM_SIZE - 1)]
            } else {
                let p = self.sparse_page(x);
                p[(x & (SPARSE_PAGE - 1)) as usize]
            };
            v = v << 8 | b as u32;
        }
        v
    }

    fn plain_write(&mut self, a: u32, size: u32, v: u32) {
        for i in 0..size {
            let x = a.wrapping_add(i);
            let b = (v >> (8 * (size - 1 - i))) as u8;
            if let Some(o) = self.ddr_off(x) {
                self.ddr[o] = b;
            } else if (SRAM_BASE..0x8C00_0000).contains(&x) {
                self.sram[(x as usize) & (SRAM_SIZE - 1)] = b;
            } else {
                let p = self.sparse_page(x);
                p[(x & (SPARSE_PAGE - 1)) as usize] = b;
            }
        }
    }

    #[cold]
    fn read_slow(&mut self, a: u32, size: u32) -> u32 {
        if Self::is_io(a) {
            // A model may answer; otherwise the register reads back as memory.
            let plain = self.plain_read(a, size);
            let pc = self.pc;
            let (io, ddr) = (&mut self.io, &mut self.ddr);
            if let Some(v) = io.read(a, size, plain, pc, ddr) {
                return v;
            }
            return plain;
        }
        self.plain_read(a, size)
    }

    #[cold]
    fn write_slow(&mut self, a: u32, size: u32, v: u32) {
        if Self::is_io(a) {
            let pc = self.pc;
            let (io, ddr) = (&mut self.io, &mut self.ddr);
            if io.write(a, size, v, pc, ddr) {
                return;
            }
        }
        self.plain_write(a, size, v)
    }
}
