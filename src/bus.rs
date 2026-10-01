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

use crate::fast::Op;
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
    /// A DDR write watchpoint: [lo, hi) as physical offsets. Hits are logged
    /// as (pc, address, size, value).
    pub watch: Option<(u32, u32)>,
    pub watch_hits: u64,
    pub watch_log: Vec<(u32, u32, u32, u32)>,
    /// Set while the eDMA engine runs, so a register write it makes cannot
    /// start the engine again underneath itself.
    pub in_dma: bool,
    /// The block cache over [CODE_BASE, CODE_BASE + span): halfword ->
    /// block id (0 = none), the blocks, and which halfwords hold code.
    blk_map: Vec<u32>,
    pub(crate) blk_arena: Vec<Block>,
    code_bits: Vec<u64>,
    pub(crate) flush_pending: bool,
    /// Compiled code's page tables (see `jit_pages`): per 64 KB of the
    /// address space, the host address of plain memory there or 0, for
    /// reads and for writes; and what they were built for.
    jit_rd_pages: Vec<usize>,
    jit_wr_pages: Vec<usize>,
    jit_pages_for: (usize, usize, u32),
    pub icache_flushes: u64,
    icache_span: u32,
    pub icache_on: bool,
    pub icache_decodes: u64,
    pub icache_invalidations: u64,
}

/// A predecoded block, and its compiled code once it ran hot.
pub struct Block {
    pub ops: Box<[Op]>,
    pub hits: u32,
    pub code: Option<crate::jit::BlockFn>,
    /// The compiler declined it; it stays interpreted.
    pub no_jit: bool,
    /// Sent to the background compiler.
    pub queued: bool,
}

/// Where MAIN OS's text starts; the cache covers the image from here.
pub const CODE_BASE: u32 = 0x4000_0400;

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
            watch_log: Vec::new(),
            in_dma: false,
            blk_map: Vec::new(),
            jit_rd_pages: Vec::new(),
            jit_wr_pages: Vec::new(),
            jit_pages_for: (0, 0, 0),
            blk_arena: Vec::new(),
            code_bits: Vec::new(),
            flush_pending: false,
            icache_flushes: 0,
            icache_span: 0,
            icache_on: false,
            icache_decodes: 0,
            icache_invalidations: 0,
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

    /// Cover `span` bytes of code from CODE_BASE with the block cache.
    pub fn icache_enable(&mut self, span: u32) {
        let n = (span / 2) as usize + 1;
        self.blk_map = vec![0; n];
        self.code_bits = vec![0; n / 64 + 1];
        self.blk_arena.clear();
        self.icache_span = span;
        self.icache_on = true;
    }

    /// Drop every block (at the next block boundary, never under one).
    pub fn icache_flush(&mut self) {
        self.flush_pending = true;
    }

    /// Apply a pending flush. Only between blocks.
    #[inline(always)]
    pub fn icache_settle(&mut self) {
        if self.flush_pending {
            self.flush_pending = false;
            self.blk_map.iter_mut().for_each(|x| *x = 0);
            self.code_bits.iter_mut().for_each(|x| *x = 0);
            self.blk_arena.clear();
            self.icache_flushes += 1;
        }
    }

    /// -> the index of the block starting at `pc`, if one is built.
    #[inline(always)]
    pub fn block_at(&self, pc: u32) -> Option<usize> {
        let o = pc.wrapping_sub(CODE_BASE);
        if o >= self.icache_span || o & 1 != 0 {
            return None;
        }
        let id = self.blk_map[(o >> 1) as usize];
        if id == 0 {
            return None;
        }
        Some((id - 1) as usize)
    }

    /// DDR offset of `[a, a + len)` when all of it is plain DDR that no
    /// watchpoint or cached code covers, so it can be moved directly.
    #[inline(always)]
    pub fn ddr_plain(&self, a: u32, len: u32) -> Option<usize> {
        let o = self.ddr_off(a)?;
        let end = a.wrapping_add(len - 1);
        if self.ddr_off(end)? != o + len as usize - 1 || self.watch.is_some() {
            return None;
        }
        // Wholly below or above the cached code, or a store could hit it.
        let (lo, hi) = (CODE_BASE - 0x4000_0000, CODE_BASE - 0x4000_0000 + self.icache_span);
        let o32 = o as u32;
        if o32 + len > lo && o32 < hi {
            return None;
        }
        Some(o)
    }

    /// Page tables for compiled code: entry `a >> 16` is the host address
    /// of the 64 KB page holding `a` when it is plain memory -- DDR (any
    /// alias) or SRAM -- and 0 otherwise. The write table leaves out DDR
    /// pages that overlap cached code, whose stores `check_code` must see.
    /// An access through a non-zero entry that stays inside its page is
    /// one `read*` / `write*` would make directly. Rebuilt when DDR or SRAM
    /// move (a snapshot load) or the code window changes. -> (reads, writes)
    pub fn jit_pages(&mut self) -> (*const usize, *const usize) {
        let key = (self.ddr.as_ptr() as usize, self.sram.as_ptr() as usize, self.icache_span);
        if self.jit_rd_pages.is_empty() || self.jit_pages_for != key {
            let (lo, hi) = (CODE_BASE - 0x4000_0000, CODE_BASE - 0x4000_0000 + self.icache_span);
            let mut rd = vec![0usize; 0x1_0000];
            let mut wr = vec![0usize; 0x1_0000];
            for page in 0..0x1_0000u32 {
                let a = page << 16;
                if let Some(o) = self.ddr_off(a) {
                    if o + 0x1_0000 <= self.ddr.len() {
                        let host = self.ddr.as_ptr() as usize + o;
                        rd[page as usize] = host;
                        let (s, e) = (o as u32, o as u32 + 0x1_0000);
                        if self.icache_span == 0 || e <= lo || s >= hi {
                            wr[page as usize] = host;
                        }
                    }
                } else if Self::sram_off(a).is_some() {
                    rd[page as usize] = self.sram.as_ptr() as usize;
                    wr[page as usize] = self.sram.as_ptr() as usize;
                }
            }
            self.jit_rd_pages = rd;
            self.jit_wr_pages = wr;
            self.jit_pages_for = key;
        }
        (self.jit_rd_pages.as_ptr(), self.jit_wr_pages.as_ptr())
    }

    #[inline(always)]
    pub fn in_code_window(&self, pc: u32) -> bool {
        self.icache_on && pc.wrapping_sub(CODE_BASE) < self.icache_span && pc & 1 == 0
    }

    /// Record a block that starts at `pc` and covers [pc, end).
    pub fn block_store(&mut self, pc: u32, end: u32, ops: Box<[Op]>) -> usize {
        let o = pc.wrapping_sub(CODE_BASE);
        self.blk_arena.push(Block { ops, hits: 0, code: None, no_jit: false, queued: false });
        let id = self.blk_arena.len() as u32;
        self.blk_map[(o >> 1) as usize] = id;
        let e = end.wrapping_sub(CODE_BASE).min(self.icache_span);
        for h in (o >> 1)..e.div_ceil(2) {
            let h = h as usize;
            self.code_bits[h >> 6] |= 1 << (h & 63);
        }
        self.icache_decodes += 1;
        self.blk_arena.len() - 1
    }

    /// A store to physical DDR offset `o`: if it lands on code some block
    /// was built from, every block goes (at the next block boundary).
    #[inline(always)]
    fn check_code(&mut self, o: u32, n: u32) {
        let rel = o.wrapping_sub(CODE_BASE - 0x4000_0000);
        if rel < self.icache_span {
            let lo = rel >> 1;
            let hi = ((rel + n).min(self.icache_span) + 1) >> 1;
            for i in lo..hi {
                let i = i as usize;
                if self.code_bits[i >> 6] & (1 << (i & 63)) != 0 {
                    self.icache_invalidations += 1;
                    self.flush_pending = true;
                    return;
                }
            }
        }
    }

    /// Offset into SRAM for an address in its 0x80000000-0x8BFFFFFF window.
    #[inline(always)]
    fn sram_off(addr: u32) -> Option<usize> {
        if addr.wrapping_sub(SRAM_BASE) < 0x0C00_0000 {
            Some((addr as usize) & (SRAM_SIZE - 1))
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
        if !self.sparse.contains_key(&base) {
            // First touch of a page nothing models: worth knowing about.
            self.unmapped_touch.insert(base, (self.pc, 1));
            self.sparse.insert(base, Box::new([0; SPARSE_PAGE as usize]));
        }
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
            self.check_code(o as u32, 1);
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
                if self.icache_on {
                    for k in (0..data.len()).step_by(2) {
                        self.check_code((o + k) as u32, 2);
                    }
                }
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
        if let Some(o) = Self::sram_off(a) {
            return self.sram[o];
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
        if let Some(o) = Self::sram_off(a) {
            if o + 2 <= SRAM_SIZE {
                return u16::from_be_bytes([self.sram[o], self.sram[o + 1]]);
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
        if let Some(o) = Self::sram_off(a) {
            if o + 4 <= SRAM_SIZE {
                return u32::from_be_bytes(self.sram[o..o + 4].try_into().unwrap());
            }
        }
        self.read_slow(a, 4)
    }

    #[inline(always)]
    pub fn write8(&mut self, a: u32, v: u8) {
        if let Some(o) = self.ddr_off(a) {
            self.check_watch(a, 1, v as u32);
            self.check_code(o as u32, 1);
            self.ddr[o] = v;
            return;
        }
        if let Some(o) = Self::sram_off(a) {
            self.sram[o] = v;
            return;
        }
        self.write_slow(a, 1, v as u32)
    }

    #[inline(always)]
    pub fn write16(&mut self, a: u32, v: u16) {
        if let Some(o) = self.ddr_off(a) {
            if o + 2 <= self.ddr.len() {
                self.check_watch(a, 2, v as u32);
                self.check_code(o as u32, 2);
                self.ddr[o..o + 2].copy_from_slice(&v.to_be_bytes());
                return;
            }
        }
        if let Some(o) = Self::sram_off(a) {
            if o + 2 <= SRAM_SIZE {
                self.sram[o..o + 2].copy_from_slice(&v.to_be_bytes());
                return;
            }
        }
        self.write_slow(a, 2, v as u32)
    }

    #[inline(always)]
    pub fn write32(&mut self, a: u32, v: u32) {
        if let Some(o) = self.ddr_off(a) {
            if o + 4 <= self.ddr.len() {
                self.check_watch(a, 4, v);
                self.check_code(o as u32, 4);
                self.ddr[o..o + 4].copy_from_slice(&v.to_be_bytes());
                return;
            }
        }
        if let Some(o) = Self::sram_off(a) {
            if o + 4 <= SRAM_SIZE {
                self.sram[o..o + 4].copy_from_slice(&v.to_be_bytes());
                return;
            }
        }
        self.write_slow(a, 4, v)
    }

    #[inline(always)]
    fn check_watch(&mut self, a: u32, n: u32, v: u32) {
        if let Some((lo, hi)) = self.watch {
            let o = a & self.ddr_mask;
            if o + n > lo && o < hi {
                self.watch_hits += 1;
                if self.watch_log.len() < 4096 {
                    self.watch_log.push((self.pc, a, n, v));
                }
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
            let pc = self.pc;
            let (io, ddr) = (&mut self.io, &mut self.ddr);
            if let Some(v) = io.read(a, size, 0, pc, ddr) {
                return v;
            }
        }
        self.plain_read(a, size)
    }

    #[cold]
    fn write_slow(&mut self, a: u32, size: u32, v: u32) {
        if Self::is_io(a) {
            let pc = self.pc;
            let (io, ddr) = (&mut self.io, &mut self.ddr);
            if io.write(a, size, v, pc, ddr) {
                if self.io.edma.kick != 0 && !self.in_dma {
                    self.run_dma();
                }
                return;
            }
        }
        self.plain_write(a, size, v)
    }
}
