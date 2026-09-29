//! The eSDHC controller (0xFC0CC000, RM chapter 25) and the eMMC behind it.
//!
//! The firmware drives the controller in PIO mode for commands and moves
//! bulk data through SoC eDMA channel 59, reading or writing DATPORT; the
//! controller's own DMA (XFERTYP[DMAEN]) is never used. Completion reaches
//! the firmware through its own interrupt handlers: the controller's
//! (vector 223, INTC2 source 31) on IRQSTAT & IRQSIGEN, and channel 59's
//! major-loop interrupt (vector 192). Nothing here writes driver state.
//!
//! The card must be one the firmware recognises, or it will not mount the
//! +Drive: `FUN_400e253e` matches the CID's manufacturer and product name
//! against a seven-row table at 0x4020c2c4, and `FUN_400e26f2` checks three
//! EXT_CSD identity bytes and the sector count against the same row. The
//! values below are that table's own (reference project, esdhc.py `PART`).

use serde::{Deserialize, Serialize};
use serde_big_array::BigArray;
use std::collections::{BTreeMap, VecDeque};
use std::io::{Read, Seek, SeekFrom, Write};

pub const BASE: u32 = 0xFC0C_C000;
pub const DMA_CHAN: usize = 59;

const CC: u32 = 1 << 0;
const TC: u32 = 1 << 1;
const BWR: u32 = 1 << 4;
const BRR: u32 = 1 << 5;

const CIHB: u32 = 1 << 0;
const CDIHB: u32 = 1 << 1;
const DLA: u32 = 1 << 2;
const BWEN: u32 = 1 << 10;
const BREN: u32 = 1 << 11;
const CINS: u32 = 1 << 16;

const SYSCTL_SELFCLEAR: u32 = 0x0F00_0000;

pub const MANUFACTURER: u32 = 0x11;
pub const PRODUCT: &[u8; 6] = b"004GE0";
pub const SECTORS_SLC: u32 = 0x003B_0000;
const ID_MULT: u32 = 0x0001D8;
const ID_B: u8 = 0x01;
const ID_C: u8 = 0x08;

/// Sparse sector storage with an optional backing file.
#[derive(Serialize, Deserialize)]
pub struct Card {
    #[serde(with = "sector_map")]
    pub sectors: BTreeMap<u32, Box<[u8; 512]>>,
    pub capacity: u32,
    #[serde(with = "BigArray")]
    pub ext_csd: [u8; 512],
    pub cid: [u32; 4],
    pub rca: u32,
    pub path: Option<std::path::PathBuf>,
    pub dirty: bool,
    erase_lo: Option<u32>,
    erase_hi: Option<u32>,
    pub reads: u64,
    pub writes: u64,
}

impl Card {
    pub fn new() -> Card {
        let mut ext = [0u8; 512];
        ext[0x98] = 1; // SLC mode: what slc_status_predicate wants
        ext[0xD4..0xD8].copy_from_slice(&SECTORS_SLC.to_be_bytes()); // read big-endian
        ext[0xAF] = 1;
        ext[0xB7] = 1;
        ext[0xB9] = 1;
        ext[0x9C..0x9F].copy_from_slice(&ID_MULT.to_be_bytes()[1..]);
        ext[0xDE] = ID_B;
        ext[0xE3] = ID_C;
        let n = PRODUCT;
        // R2 order {RSP3[23:0], RSP2, RSP1, RSP0} over CID[127:8].
        let cid = [
            0,
            (n[4] as u32) << 24 | (n[5] as u32) << 16,
            (n[0] as u32) << 24 | (n[1] as u32) << 16 | (n[2] as u32) << 8 | n[3] as u32,
            MANUFACTURER << 16,
        ];
        Card {
            sectors: BTreeMap::new(),
            capacity: SECTORS_SLC,
            ext_csd: ext,
            cid,
            rca: 0,
            path: None,
            dirty: false,
            erase_lo: None,
            erase_hi: None,
            reads: 0,
            writes: 0,
        }
    }

    /// Back the card with an image file (read now, written by `flush`).
    pub fn open(path: &std::path::Path) -> std::io::Result<Card> {
        let mut c = Card::new();
        c.path = Some(path.to_path_buf());
        if let Ok(mut f) = std::fs::File::open(path) {
            let len = f.metadata()?.len();
            let mut buf = [0u8; 512];
            let mut s = 0u32;
            let mut off = 0u64;
            // Only keep sectors that are not all zero.
            let mut chunk = vec![0u8; 1 << 20];
            while off < len {
                let n = f.read(&mut chunk)?;
                if n == 0 {
                    break;
                }
                for part in chunk[..n].chunks(512) {
                    if part.iter().any(|&b| b != 0) {
                        buf[..part.len()].copy_from_slice(part);
                        buf[part.len()..].fill(0);
                        c.sectors.insert(s, Box::new(buf));
                    }
                    s += 1;
                }
                off += n as u64;
            }
        }
        Ok(c)
    }

    pub fn flush(&mut self) -> std::io::Result<()> {
        let Some(p) = self.path.clone() else { return Ok(()) };
        if !self.dirty {
            return Ok(());
        }
        let mut f = std::fs::OpenOptions::new().create(true).write(true).truncate(true).open(&p)?;
        let zero = [0u8; 512];
        let mut last = 0u32;
        for (&s, data) in &self.sectors {
            f.seek(SeekFrom::Start(s as u64 * 512))?;
            f.write_all(&data[..])?;
            last = last.max(s + 1);
        }
        let _ = zero;
        f.set_len(last as u64 * 512)?;
        self.dirty = false;
        Ok(())
    }

    pub fn read(&mut self, sector: u32, out: &mut Vec<u8>) {
        self.reads += 1;
        match self.sectors.get(&sector) {
            Some(d) => out.extend_from_slice(&d[..]),
            None => out.extend_from_slice(&[0u8; 512]),
        }
    }

    pub fn write(&mut self, sector: u32, data: &[u8]) {
        self.writes += 1;
        self.dirty = true;
        if data.iter().all(|&b| b == 0) {
            self.sectors.remove(&sector);
            return;
        }
        let mut b = [0u8; 512];
        b[..data.len().min(512)].copy_from_slice(&data[..data.len().min(512)]);
        self.sectors.insert(sector, Box::new(b));
    }

    fn erase(&mut self, lo: u32, hi_incl: u32) {
        let keys: Vec<u32> = self.sectors.range(lo..=hi_incl).map(|(k, _)| *k).collect();
        for k in keys {
            self.sectors.remove(&k);
        }
        self.dirty = true;
    }
}

impl Default for Card {
    fn default() -> Self {
        Self::new()
    }
}

#[derive(PartialEq, Eq, Clone, Copy, Debug, Serialize, Deserialize)]
enum Phase {
    Idle,
    /// Card -> host: `rx` holds what is left to read.
    Read,
    /// Host -> card: `wx` collects until `want` bytes arrive.
    Write,
}

#[derive(Serialize, Deserialize)]
pub struct Esdhc {
    #[serde(with = "BigArray")]
    pub regs: [u32; 0x40],
    pub card: Card,
    rx: VecDeque<u8>,
    wx: Vec<u8>,
    want: usize,
    phase: Phase,
    cmd: u32,
    arg: u32,
    pattern: u32,
    pub log: Vec<(u32, u32)>,
}

impl Esdhc {
    pub fn new(card: Card) -> Esdhc {
        let mut regs = [0u32; 0x40];
        regs[0x04 / 4] = 0x0001_0000; // BLKATTR
        regs[0x24 / 4] = 0xFF88_00F8 | CINS; // PRSSTAT, a card inserted
        regs[0x28 / 4] = 0x0000_0020; // PROCTL
        regs[0x2C / 4] = 0x0000_8008; // SYSCTL
        regs[0x34 / 4] = 0x117F_013F; // IRQSTATEN
        regs[0x40 / 4] = 0x07F3_0000; // HOSTCAPBLT
        regs[0x44 / 4] = 0x0810_0810; // WML
        regs[0xC0 / 4] = 1; // VENDOR
        regs[0xFC / 4] = 0x0000_1201; // HOSTVER
        Esdhc {
            regs,
            card,
            rx: VecDeque::new(),
            wx: Vec::new(),
            want: 0,
            phase: Phase::Idle,
            cmd: 0,
            arg: 0,
            pattern: 0,
            log: Vec::new(),
        }
    }

    fn r(&self, off: u32) -> u32 {
        self.regs[(off / 4) as usize]
    }

    fn set(&mut self, off: u32, v: u32) {
        self.regs[(off / 4) as usize] = v;
    }

    fn irq_status(&mut self, bits: u32) {
        let en = self.r(0x34);
        let v = self.r(0x30) | (bits & en);
        self.set(0x30, v);
    }

    /// The controller's interrupt line.
    pub fn line(&self) -> bool {
        self.r(0x30) & self.r(0x38) != 0
    }

    /// Asking eDMA channel 59 for service.
    pub fn dma_request(&self) -> bool {
        match self.phase {
            Phase::Read => !self.rx.is_empty(),
            Phase::Write => self.wx.len() < self.want,
            Phase::Idle => false,
        }
    }

    fn block_bytes(&self) -> usize {
        let blkattr = self.r(0x04);
        let size = (blkattr & 0x1FFF) as usize;
        let size = if size == 0 { 512 } else { size };
        let xfer = self.r(0x0C);
        let multi = xfer & (1 << 5) != 0; // MSBSEL
        let count = if multi && xfer & (1 << 1) != 0 { (blkattr >> 16) as usize } else { 1 };
        size * count.max(1)
    }

    fn issue(&mut self, xfer: u32) {
        self.set(0x0C, xfer);
        let idx = (xfer >> 24) & 0x3F;
        let arg = self.r(0x08);
        if self.log.len() < 100_000 {
            self.log.push((idx, arg));
        }
        self.cmd = idx;
        self.arg = arg;
        let r1 = 0x0000_0900; // transfer state, READY_FOR_DATA
        let resp: [u32; 4] = match idx {
            0 => [0; 4],
            1 => [0xC0FF_8080, 0, 0, 0],
            2 | 10 => self.card.cid,
            9 => [0; 4],
            3 => {
                self.card.rca = arg >> 16;
                [r1, 0, 0, 0]
            }
            35 => {
                self.card.erase_lo = Some(arg);
                [r1, 0, 0, 0]
            }
            36 => {
                self.card.erase_hi = Some(arg);
                [r1, 0, 0, 0]
            }
            38 => {
                if let (Some(lo), Some(hi)) = (self.card.erase_lo, self.card.erase_hi) {
                    self.card.erase(lo, hi);
                }
                self.card.erase_lo = None;
                self.card.erase_hi = None;
                [r1, 0, 0, 0]
            }
            _ => [r1, 0, 0, 0],
        };
        for (i, v) in resp.iter().enumerate() {
            self.set(0x10 + 4 * i as u32, *v);
        }
        let mut prs = self.r(0x24) & !(CIHB | CDIHB | DLA | BREN | BWEN);
        self.rx.clear();
        self.wx.clear();
        self.phase = Phase::Idle;
        let mut status = CC;
        if xfer & (1 << 21) != 0 {
            // A data command.
            if xfer & (1 << 4) != 0 {
                // Card -> host.
                let n = self.block_bytes();
                let mut data = Vec::with_capacity(n);
                match idx {
                    8 => data.extend_from_slice(&self.card.ext_csd),
                    14 => data.extend_from_slice(&(!self.pattern).to_be_bytes()),
                    17 | 18 => {
                        let blocks = n.div_ceil(512);
                        for s in 0..blocks as u32 {
                            self.card.read(arg + s, &mut data);
                        }
                    }
                    _ => {}
                }
                data.resize(n.max(4), 0);
                self.rx.extend(data);
                self.phase = Phase::Read;
                prs |= BREN;
                // The card has put the whole transfer into the buffer: the
                // transaction is complete from the card's side.
                status |= BRR | TC;
            } else {
                self.want = self.block_bytes();
                self.phase = Phase::Write;
                prs |= BWEN | DLA | CDIHB;
                status |= BWR;
                if idx == 19 {
                    self.want = 4;
                }
            }
        }
        self.set(0x24, prs);
        self.irq_status(status);
    }

    fn finish_data(&mut self) {
        let mut prs = self.r(0x24) & !(CDIHB | DLA | BREN | BWEN);
        if self.phase == Phase::Write {
            match self.cmd {
                24 | 25 => {
                    let data = std::mem::take(&mut self.wx);
                    for (i, chunk) in data.chunks(512).enumerate() {
                        self.card.write(self.arg + i as u32, chunk);
                    }
                }
                19 => {
                    if self.wx.len() >= 4 {
                        self.pattern = u32::from_be_bytes(self.wx[..4].try_into().unwrap());
                    } else if let Some(&b) = self.wx.first() {
                        self.pattern = b as u32;
                    }
                }
                _ => {}
            }
        }
        self.phase = Phase::Idle;
        prs &= !(CIHB);
        self.set(0x24, prs);
        self.irq_status(TC);
    }

    pub fn read(&mut self, off: u32, size: u32) -> Option<u32> {
        if off >= 0x100 || off & 3 != 0 || size != 4 {
            // Byte/word reads of a register: from the register image.
            if off < 0x100 {
                let w = self.r(off & !3);
                let sh = 8 * (4 - size - (off & 3));
                let m = if size == 4 { u32::MAX } else { (1u32 << (8 * size)) - 1 };
                return Some((w >> sh) & m);
            }
            return None;
        }
        if off == 0x20 {
            // DATPORT: the next word of the transfer.
            let mut v = 0u32;
            for _ in 0..4 {
                v = v << 8 | self.rx.pop_front().unwrap_or(0) as u32;
            }
            if self.phase == Phase::Read && self.rx.is_empty() {
                self.phase = Phase::Idle;
                let prs = self.r(0x24) & !BREN;
                self.set(0x24, prs);
            }
            return Some(v);
        }
        Some(self.r(off))
    }

    pub fn write(&mut self, off: u32, size: u32, v: u32) -> bool {
        if off >= 0x100 {
            return false;
        }
        let (off, v) = if size == 4 {
            (off, v)
        } else {
            // Merge a narrow write into the register.
            let reg = off & !3;
            let sh = 8 * (4 - size - (off & 3));
            let m = if size == 4 { u32::MAX } else { ((1u32 << (8 * size)) - 1) << sh };
            (reg, (self.r(reg) & !m) | ((v << sh) & m))
        };
        match off {
            0x0C => self.issue(v),
            0x20 => {
                if self.phase == Phase::Write {
                    self.wx.extend_from_slice(&v.to_be_bytes());
                    if self.wx.len() >= self.want {
                        self.finish_data();
                    }
                } else {
                    self.pattern = v;
                }
            }
            0x2C => {
                // SYSCTL: INITA and the resets complete at once.
                if v & (1 << 24) != 0 {
                    self.phase = Phase::Idle;
                    self.rx.clear();
                    self.wx.clear();
                    self.set(0x30, 0);
                }
                self.set(0x2C, v & !SYSCTL_SELFCLEAR);
            }
            0x30 => {
                let cur = self.r(0x30);
                self.set(0x30, cur & !v);
            }
            0x24 | 0x10 | 0x14 | 0x18 | 0x1C | 0x40 | 0xFC => {}
            _ => self.set(off, v),
        }
        true
    }
}

mod sector_map {
    use serde::{Deserialize, Deserializer, Serialize, Serializer};
    use std::collections::BTreeMap;

    pub fn serialize<S: Serializer>(m: &BTreeMap<u32, Box<[u8; 512]>>, s: S) -> Result<S::Ok, S::Error> {
        let v: Vec<(u32, &[u8])> = m.iter().map(|(k, d)| (*k, &d[..])).collect();
        v.serialize(s)
    }

    pub fn deserialize<'de, D: Deserializer<'de>>(d: D) -> Result<BTreeMap<u32, Box<[u8; 512]>>, D::Error> {
        let v: Vec<(u32, Vec<u8>)> = Vec::deserialize(d)?;
        Ok(v.into_iter()
            .map(|(k, d)| {
                let mut b = Box::new([0u8; 512]);
                b[..d.len().min(512)].copy_from_slice(&d[..d.len().min(512)]);
                (k, b)
            })
            .collect())
    }
}
