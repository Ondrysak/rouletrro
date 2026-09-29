//! The +Drive's sample filesystem, "ekFS": format a card and write samples
//! into it, offline.
//!
//! A port of the reference project's emu/ekfsformat.py, where every detail
//! was recovered from the firmware's own code and checked against a card the
//! firmware formatted itself. In brief:
//!
//! * The sample region starts at sector 0x1C0000. Superblock at +0, inode
//!   bitmap +0x10, block bitmap +0x30, inode table +0x50, data +0x4050 (all
//!   in sectors); blocks are 16 KB. A copy of the superblock sits just past
//!   the block bitmap.
//! * The superblock checksum is Bob Jenkins' lookup3 hashbig seeded with
//!   0x31323334 (the length is not folded into the seed). The mount refuses
//!   a volume whose checksum does not match.
//! * Inodes are 128 bytes: type (1 = directory), 2, link count, size, parent,
//!   content hash, serial, extents (logical, count, physical) from +0x20.
//! * A directory's second extent (logical 0x20000) holds three sorted
//!   indexes -- by dx_hack_hash, by name, by inode -- that the firmware lists
//!   and searches through; the entry block alone is invisible to it.
//! * Every finished file carries lookup3("eLeK" seed) | 1 in inode +0x0C and
//!   in a table at block 64; without it a sample will not load.
//! * A sample is a 64-byte header, big-endian 16-bit mono PCM, 16 zeros.

use crate::esdhc::Card;

pub const SECTOR: usize = 512;
pub const REGION: u32 = 0x1C_0000;
const INODE_SIZE: usize = 0x80;
const INODES_PER_CHUNK: u32 = 128;
const CHUNK_SECTORS: u32 = 0x20;
pub const BLOCK_BYTES: usize = CHUNK_SECTORS as usize * SECTOR;
const LOG_BLOCK_SHIFT: u32 = 14;
const SEED: u32 = 0x3132_3334;
pub const ROOT_INODE: u32 = 2;
const RAM_INODE: u32 = 0x0100_0000;
pub const TYPE_DIR: u8 = 1;
pub const TYPE_FILE: u8 = 0;
const MAX_INLINE_EXTENTS: usize = (INODE_SIZE - 0x20) / 12;
const INDEX_LOGICAL: [u32; 3] = [0x20000, 0x20001, 0x20002];
const MAX_DIR_ENTRIES: usize = 2000;
const INDEX_HEADER: usize = 8;
const SAMPLE_HEADER: usize = 0x40;
const SAMPLE_TRAILER: usize = 0x10;
const MAX_SAMPLE_PCM: usize = 0x400_0000;
const SAMPLE_HASH_SEED: u32 = 0x654C_654B;
const HASH_TABLE_BLOCK: u32 = 64;
const SB_BACKUP_OFF: usize = 0x1E00;
const RESERVED_BLOCKS: u32 = 96;
const DIR_SIZE: u32 = 0x4000;

#[derive(Debug)]
pub struct Error(pub String);

impl std::fmt::Display for Error {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(&self.0)
    }
}

impl std::error::Error for Error {}

fn err<T>(s: impl Into<String>) -> Result<T, Error> {
    Err(Error(s.into()))
}

fn be32(b: &[u8], o: usize) -> u32 {
    u32::from_be_bytes(b[o..o + 4].try_into().unwrap())
}

fn put32(b: &mut [u8], o: usize, v: u32) {
    b[o..o + 4].copy_from_slice(&v.to_be_bytes());
}

fn put16(b: &mut [u8], o: usize, v: u16) {
    b[o..o + 2].copy_from_slice(&v.to_be_bytes());
}

fn be16(b: &[u8], o: usize) -> u16 {
    u16::from_be_bytes(b[o..o + 2].try_into().unwrap())
}

// -- hashes ----------------------------------------------------------------------

/// lookup3 hashbig, seeded the way this firmware seeds it.
pub fn ekfs_hash(data: &[u8], initval: u32) -> u32 {
    let rot = |x: u32, k: u32| x.rotate_left(k);
    let mut a = 0xDEAD_BEEFu32.wrapping_add(initval);
    let mut b = a;
    let mut c = a;
    let mut i = 0;
    let n = data.len();
    while n - i > 12 {
        a = a.wrapping_add(be32(data, i));
        b = b.wrapping_add(be32(data, i + 4));
        c = c.wrapping_add(be32(data, i + 8));
        a = a.wrapping_sub(c); a ^= rot(c, 4); c = c.wrapping_add(b);
        b = b.wrapping_sub(a); b ^= rot(a, 6); a = a.wrapping_add(c);
        c = c.wrapping_sub(b); c ^= rot(b, 8); b = b.wrapping_add(a);
        a = a.wrapping_sub(c); a ^= rot(c, 16); c = c.wrapping_add(b);
        b = b.wrapping_sub(a); b ^= rot(a, 19); a = a.wrapping_add(c);
        c = c.wrapping_sub(b); c ^= rot(b, 4); b = b.wrapping_add(a);
        i += 12;
    }
    let left = n - i;
    if left == 0 {
        return c;
    }
    let mut pad = [0u8; 12];
    pad[..left].copy_from_slice(&data[i..]);
    let (k0, k1, k2) = (be32(&pad, 0), be32(&pad, 4), be32(&pad, 8));
    // Bytes past the end are zero in `pad`, which is what the masks do.
    a = a.wrapping_add(k0);
    if left > 4 {
        b = b.wrapping_add(k1);
    }
    if left > 8 {
        c = c.wrapping_add(k2);
    }
    c ^= b; c = c.wrapping_sub(rot(b, 14));
    a ^= c; a = a.wrapping_sub(rot(c, 11));
    b ^= a; b = b.wrapping_sub(rot(a, 25));
    c ^= b; c = c.wrapping_sub(rot(b, 16));
    a ^= c; a = a.wrapping_sub(rot(c, 4));
    b ^= a; b = b.wrapping_sub(rot(a, 14));
    c ^= b; c = c.wrapping_sub(rot(b, 24));
    c
}

/// FUN_400cccb6: ext3's legacy dx_hack_hash.
pub fn dx_hack_hash(name: &[u8]) -> u32 {
    let (mut h0, mut h1) = (0x12A3_FE2Du32, 0x37AB_E8F9u32);
    for &ch in name {
        let mut h = h1.wrapping_add(h0 ^ (ch as u32).wrapping_mul(0x6D_22F5));
        if h & 0x8000_0000 != 0 {
            h = h.wrapping_add(0x8000_0001);
        }
        h1 = h0;
        h0 = h;
    }
    h0 << 1
}

/// FUN_400e8606: natural-order, case-insensitive, with its quirk (the byte
/// after an equal digit run is skipped on both sides unread).
pub fn natcmp(a: &[u8], b: &[u8]) -> i32 {
    let upper = |c: u8| if c.is_ascii_lowercase() { c - 0x20 } else { c } as i8 as i32;
    let digit = |c: u8| c.is_ascii_digit();
    let (mut i, mut j) = (0, 0);
    loop {
        if i >= a.len() {
            return if j < b.len() { -1 } else { 0 };
        }
        if j >= b.len() {
            return 1;
        }
        let (ca, cb) = (a[i], b[j]);
        if digit(ca) && digit(cb) {
            let (i0, j0) = (i, j);
            while i < a.len() && a[i] == b'0' {
                i += 1;
            }
            while j < b.len() && b[j] == b'0' {
                j += 1;
            }
            let (za, zb) = (i - i0, j - j0);
            let (mut ie, mut je) = (i, j);
            while ie < a.len() && digit(a[ie]) {
                ie += 1;
            }
            while je < b.len() && digit(b[je]) {
                je += 1;
            }
            if ie - i != je - j {
                return if ie - i > je - j { 1 } else { -1 };
            }
            while i < ie {
                if a[i] != b[j] {
                    return if a[i] < b[j] { -1 } else { 1 };
                }
                i += 1;
                j += 1;
            }
            if za != zb {
                return if za > zb { 1 } else { -1 };
            }
            i += 1;
            j += 1;
            continue;
        }
        let (ua, ub) = (upper(ca), upper(cb));
        if ua != ub {
            return if ua < ub { -1 } else { 1 };
        }
        i += 1;
        j += 1;
    }
}

fn name_key(name: &[u8]) -> u32 {
    match name {
        b"." => 0x2E00_534F,
        b".." => 0x2E2E_0053,
        _ => {
            let mut k = [0u8; 4];
            for (d, s) in k.iter_mut().zip(name.iter()) {
                *d = *s;
            }
            u32::from_be_bytes(k)
        }
    }
}

/// A directory entry: (location, inode, name, type).
type Entry = (u32, u32, Vec<u8>, u8);

/// The three index blocks (hash, name, inode), built by replaying the
/// firmware's insertion loop (FUN_400cd04c) one entry at a time.
fn build_indexes(entries: &[Entry]) -> Result<[Vec<u8>; 3], Error> {
    if entries.len() > MAX_DIR_ENTRIES {
        return err(format!("{} entries; the firmware allows {MAX_DIR_ENTRIES}", entries.len()));
    }
    let mut by_hash: Vec<(u32, u32)> = Vec::new();
    let mut by_name: Vec<(u32, u32, Vec<u8>, u8)> = Vec::new();
    let mut by_inode: Vec<(u32, u32)> = Vec::new();
    for (loc, ino, name, typ) in entries {
        let h = dx_hack_hash(name);
        let i = by_hash.iter().take_while(|x| x.0 <= h).count();
        by_hash.insert(i, (h, *loc));

        let mut i = if by_name.len() < 2 { 0 } else { 2 };
        while i < by_name.len() {
            let (xn, xt) = (&by_name[i].2, by_name[i].3);
            if xt != TYPE_DIR {
                if *typ == TYPE_DIR {
                    break;
                }
            } else if *typ != TYPE_DIR {
                i += 1;
                continue;
            }
            if natcmp(xn, name) > 0 {
                break;
            }
            i += 1;
        }
        by_name.insert(i, (name_key(name), *loc, name.clone(), *typ));

        let i = by_inode.iter().take_while(|x| x.0 <= *ino).count();
        by_inode.insert(i, (*ino, *loc));
    }
    let make = |recs: Vec<(u32, u32)>| {
        let mut buf = vec![0u8; BLOCK_BYTES];
        put16(&mut buf, 0, recs.len() as u16);
        for (n, (k, l)) in recs.iter().enumerate() {
            put32(&mut buf, INDEX_HEADER + 8 * n, *k);
            put32(&mut buf, INDEX_HEADER + 8 * n + 4, *l);
        }
        buf
    };
    Ok([make(by_hash), make(by_name.into_iter().map(|x| (x.0, x.1)).collect()), make(by_inode)])
}

// -- samples ---------------------------------------------------------------------

pub struct WavInfo {
    pub rate: u32,
    pub frames: usize,
    pub channels: u16,
    pub bits: u16,
}

/// RIFF/WAVE bytes -> (+Drive sample file, info). Integer PCM of 8/16/24/32
/// bits or 32-bit float, any channel count (averaged to mono -- the mk1
/// plays mono). The rate is kept, not resampled: the header carries it.
pub fn wav_to_sample(data: &[u8]) -> Result<(Vec<u8>, WavInfo), Error> {
    if data.len() < 12 || &data[..4] != b"RIFF" || &data[8..12] != b"WAVE" {
        return err("not a RIFF/WAVE file");
    }
    let (mut fmt, mut pcm) = (None, None);
    let mut o = 12;
    while o + 8 <= data.len() {
        let id = &data[o..o + 4];
        let size = u32::from_le_bytes(data[o + 4..o + 8].try_into().unwrap()) as usize;
        let body = &data[o + 8..(o + 8 + size).min(data.len())];
        if id == b"fmt " {
            fmt = Some(body);
        } else if id == b"data" {
            pcm = Some(body);
        }
        o += 8 + size + (size & 1);
    }
    let (Some(fmt), Some(pcm)) = (fmt, pcm) else { return err("WAV has no fmt or data chunk") };
    if fmt.len() < 16 {
        return err("WAV fmt chunk too short");
    }
    let le16 = |b: &[u8], o: usize| u16::from_le_bytes([b[o], b[o + 1]]);
    let mut tag = le16(fmt, 0);
    let chans = le16(fmt, 2);
    let rate = u32::from_le_bytes(fmt[4..8].try_into().unwrap());
    let bits = le16(fmt, 14);
    if tag == 0xFFFE && fmt.len() >= 26 {
        tag = le16(fmt, 24);
    }
    if !(tag == 1 || tag == 3) || chans < 1 {
        return err(format!("unsupported WAV format tag {tag}, {chans} channel(s)"));
    }
    if (tag == 3 && bits != 32) || (tag == 1 && ![8, 16, 24, 32].contains(&bits)) {
        return err(format!("unsupported {bits}-bit WAV"));
    }
    let width = (bits / 8) as usize;
    let frame = width * chans as usize;
    let frames = pcm.len() / frame;
    if frames * 2 > MAX_SAMPLE_PCM {
        return err(format!("{} bytes of PCM; the loader takes at most {MAX_SAMPLE_PCM}", frames * 2));
    }
    let mut out = vec![0u8; frames * 2];
    for i in 0..frames {
        let mut acc = 0.0f64;
        for ch in 0..chans as usize {
            let p = i * frame + ch * width;
            acc += match (tag, width) {
                (3, _) => f32::from_le_bytes(pcm[p..p + 4].try_into().unwrap()) as f64,
                (_, 1) => (pcm[p] as f64 - 128.0) / 128.0,
                (_, 2) => i16::from_le_bytes([pcm[p], pcm[p + 1]]) as f64 / 32768.0,
                (_, 3) => ((i32::from_le_bytes([0, pcm[p], pcm[p + 1], pcm[p + 2]]) >> 8) as f64) / 8_388_608.0,
                _ => i32::from_le_bytes(pcm[p..p + 4].try_into().unwrap()) as f64 / 2_147_483_648.0,
            };
        }
        let s = (acc / chans as f64 * 32767.0).round().clamp(-32768.0, 32767.0) as i16;
        out[2 * i..2 * i + 2].copy_from_slice(&s.to_be_bytes());
    }
    let mut file = vec![0u8; SAMPLE_HEADER];
    put32(&mut file, 0x04, out.len() as u32);
    put32(&mut file, 0x08, rate);
    file[0x14] = 0x7F;
    file.extend_from_slice(&out);
    file.extend_from_slice(&[0u8; SAMPLE_TRAILER]);
    Ok((file, WavInfo { rate, frames, channels: chans, bits }))
}

/// A WAV's name on the card: no extension (the content is not a WAV any
/// more; the firmware's recorder writes none).
pub fn sample_name(path: &std::path::Path) -> String {
    let ext = path.extension().map(|e| e.to_ascii_lowercase());
    let is_wav = matches!(ext.as_deref().and_then(|e| e.to_str()), Some("wav") | Some("wave"));
    let s = if is_wav { path.file_stem() } else { path.file_name() };
    s.map(|x| x.to_string_lossy().into_owned()).unwrap_or_default()
}

// -- the filesystem --------------------------------------------------------------

pub struct Ekfs<'a> {
    pub card: &'a mut Card,
    pub base: u32,
    pub sb: Vec<u8>,
}

impl<'a> Ekfs<'a> {
    fn sectors(card: &mut Card, sector: u32, n: u32) -> Vec<u8> {
        let mut out = Vec::with_capacity(n as usize * SECTOR);
        for s in 0..n {
            match card.sectors.get(&(sector + s)) {
                Some(d) => out.extend_from_slice(&d[..]),
                None => out.extend_from_slice(&[0u8; SECTOR]),
            }
        }
        out
    }

    fn put_sectors(&mut self, sector: u32, data: &[u8]) {
        for (k, chunk) in data.chunks(SECTOR).enumerate() {
            self.card.write(sector + k as u32, chunk);
        }
    }

    /// Open the ekFS on `card`, if there is one.
    pub fn open(card: &'a mut Card) -> Result<Ekfs<'a>, Error> {
        let sb = Self::sectors(card, REGION, 1);
        if &sb[..4] != b"ekFS" {
            return err("no ekFS superblock in the sample region");
        }
        Ok(Ekfs { card, base: REGION, sb })
    }

    fn u(&self, o: usize) -> u32 {
        be32(&self.sb, o)
    }

    pub fn checksum_ok(&self) -> bool {
        ekfs_hash(&self.sb[..0x1FC], SEED) == self.u(0x1FC)
    }

    fn bitmap_bytes(&self) -> u32 {
        self.u(0x08)
    }

    fn bitmap(&mut self, inode: bool) -> Vec<u8> {
        let off = if inode { self.u(0x14) } else { self.u(0x18) };
        let n = self.bitmap_bytes() / SECTOR as u32;
        Self::sectors(self.card, self.base + off, n)
    }

    fn put_bitmap(&mut self, inode: bool, raw: &[u8]) {
        let off = if inode { self.u(0x14) } else { self.u(0x18) };
        self.put_sectors(self.base + off, raw);
    }

    fn bit(raw: &[u8], n: u32) -> bool {
        be32(raw, (n as usize >> 5) * 4) >> (n & 31) & 1 != 0
    }

    fn set_bit(raw: &mut [u8], n: u32) {
        let w = (n as usize >> 5) * 4;
        let v = be32(raw, w) | 1 << (n & 31);
        put32(raw, w, v);
    }

    pub fn used(&mut self, inode: bool) -> u32 {
        self.bitmap(inode).iter().map(|b| b.count_ones()).sum()
    }

    pub fn reseal(&mut self) {
        let h = ekfs_hash(&self.sb[..0x1FC], SEED);
        put32(&mut self.sb, 0x1FC, h);
        let sb = self.sb.clone();
        self.put_sectors(self.base, &sb);
        let mut raw = self.bitmap(false);
        raw[SB_BACKUP_OFF..SB_BACKUP_OFF + SECTOR].copy_from_slice(&sb);
        self.put_bitmap(false, &raw);
    }

    fn alloc_inode(&mut self) -> Result<u32, Error> {
        let mut raw = self.bitmap(true);
        for n in 2..self.u(0x0C) {
            if !Self::bit(&raw, n) {
                Self::set_bit(&mut raw, n);
                self.put_bitmap(true, &raw);
                return Ok(n);
            }
        }
        err("no free inode")
    }

    fn alloc_blocks(&mut self, count: u32) -> Result<Vec<(u32, u32)>, Error> {
        let mut raw = self.bitmap(false);
        let total = self.u(0x10);
        let (mut runs, mut need, mut n) = (Vec::new(), count, 0);
        while need > 0 && n < total {
            if Self::bit(&raw, n) {
                n += 1;
                continue;
            }
            let (start, mut run) = (n, 0);
            while run < need && n < total && !Self::bit(&raw, n) {
                Self::set_bit(&mut raw, n);
                n += 1;
                run += 1;
            }
            runs.push((start, run));
            need -= run;
        }
        if need > 0 {
            return err(format!("no room: {need} blocks short"));
        }
        self.put_bitmap(false, &raw);
        Ok(runs)
    }

    fn inode_loc(&self, n: u32) -> (u32, usize) {
        let chunk = n / INODES_PER_CHUNK;
        (self.base + self.u(0x1C) + chunk * CHUNK_SECTORS, (n % INODES_PER_CHUNK) as usize * INODE_SIZE)
    }

    pub fn inode(&mut self, n: u32) -> Vec<u8> {
        let (s, o) = self.inode_loc(n);
        let chunk = Self::sectors(self.card, s, CHUNK_SECTORS);
        chunk[o..o + INODE_SIZE].to_vec()
    }

    fn put_inode(&mut self, n: u32, raw: &[u8]) {
        let (s, o) = self.inode_loc(n);
        let mut chunk = Self::sectors(self.card, s, CHUNK_SECTORS);
        chunk[o..o + INODE_SIZE].copy_from_slice(raw);
        self.put_sectors(s, &chunk);
    }

    fn extents(raw: &[u8]) -> Vec<(u32, u32, u32)> {
        let n = be16(raw, 0x1E) as usize;
        (0..n.min(MAX_INLINE_EXTENTS)).map(|i| {
            let o = 0x20 + i * 12;
            (be32(raw, o), be32(raw, o + 4), be32(raw, o + 8))
        }).collect()
    }

    fn physical(raw: &[u8], logical: u32) -> Option<u32> {
        Self::extents(raw).into_iter().find(|&(f, c, _)| f <= logical && logical < f + c).map(|(f, _, p)| p + logical - f)
    }

    fn next_serial(&mut self) -> u32 {
        let raw = self.bitmap(true);
        let mut best = 0;
        for n in 2..self.u(0x0C).min(4096) {
            if Self::bit(&raw, n) {
                best = best.max(be32(&self.inode(n), 0x10));
            }
        }
        best + 1
    }

    pub fn block(&mut self, n: u32) -> Vec<u8> {
        let s = self.base + self.u(0x20) + n * CHUNK_SECTORS;
        Self::sectors(self.card, s, CHUNK_SECTORS)
    }

    fn put_block(&mut self, n: u32, data: &[u8]) {
        let s = self.base + self.u(0x20) + n * CHUNK_SECTORS;
        self.put_sectors(s, data);
    }

    fn reclen(name: &[u8]) -> usize {
        8 + name.len().div_ceil(4) * 4
    }

    /// Entries of one directory block: (offset, reclen, inode, name, type).
    fn parse_dir(buf: &[u8]) -> Vec<(usize, usize, u32, Vec<u8>, u8)> {
        let mut out = Vec::new();
        let mut o = 0;
        while o + 8 <= buf.len() {
            let ino = be32(buf, o);
            let rec = be16(buf, o + 4) as usize;
            let nlen = buf[o + 6] as usize;
            let typ = buf[o + 7];
            if rec < 8 || o + rec > buf.len() {
                break;
            }
            out.push((o, rec, ino, buf[o + 8..o + 8 + nlen].to_vec(), typ));
            o += rec;
        }
        out
    }

    /// Entries in entry order, the way FUN_400cca5c walks them.
    pub fn dir_entries(&mut self, dir: u32) -> Result<Vec<Entry>, Error> {
        let raw = self.inode(dir);
        if raw[0] != TYPE_DIR {
            return err(format!("inode {dir} is not a directory"));
        }
        let size = be32(&raw, 4) as usize;
        let mut out = Vec::new();
        for lb in 0..(size / BLOCK_BYTES).max(1) as u32 {
            let Some(p) = Self::physical(&raw, lb) else { return err(format!("inode {dir}: no block for logical {lb}")) };
            for (off, _r, ino, name, typ) in Self::parse_dir(&self.block(p)) {
                if ino != 0 {
                    out.push(((lb << LOG_BLOCK_SHIFT) | off as u32, ino, name, typ));
                }
            }
        }
        Ok(out)
    }

    fn rebuild_indexes(&mut self, dir: u32) -> Result<(), Error> {
        let raw = self.inode(dir);
        let mut blocks = [0u32; 3];
        for (k, lg) in INDEX_LOGICAL.iter().enumerate() {
            blocks[k] = Self::physical(&raw, *lg).ok_or_else(|| Error(format!("inode {dir} has no index extent")))?;
        }
        let entries = self.dir_entries(dir)?;
        let idx = build_indexes(&entries)?;
        for (b, buf) in blocks.iter().zip(idx.iter()) {
            self.put_block(*b, buf);
        }
        Ok(())
    }

    fn add_dir_entry(&mut self, dir: u32, name: &[u8], target: u32, typ: u8) -> Result<(), Error> {
        let raw = self.inode(dir);
        let ext = Self::extents(&raw);
        let Some(&(_, _, blk)) = ext.first() else { return err(format!("inode {dir} has no extents")) };
        if self.dir_entries(dir)?.len() >= MAX_DIR_ENTRIES {
            return err("directory is full");
        }
        let mut buf = self.block(blk);
        let entries = Self::parse_dir(&buf);
        if entries.iter().any(|e| e.3 == name) {
            return err(format!("{} already exists", String::from_utf8_lossy(name)));
        }
        let Some((last_off, last_rec, _, last_name, _)) = entries.last().cloned() else { return err("not a directory") };
        let natural = Self::reclen(&last_name);
        let need = Self::reclen(name);
        if last_rec - natural < need {
            return err("no room in the directory block");
        }
        put16(&mut buf, last_off + 4, natural as u16);
        let o = last_off + natural;
        let rest = buf.len() - o;
        put32(&mut buf, o, target);
        put16(&mut buf, o + 4, rest as u16);
        buf[o + 6] = name.len() as u8;
        buf[o + 7] = typ;
        buf[o + 8..o + 8 + name.len()].copy_from_slice(name);
        let end = buf.len();
        buf[o + 8 + name.len()..end].fill(0);
        self.put_block(blk, &buf);
        self.rebuild_indexes(dir)
    }

    fn set_file_hash(&mut self, ino: u32, data: &[u8]) {
        let h = ekfs_hash(data, SAMPLE_HASH_SEED) | 1;
        let blk = HASH_TABLE_BLOCK + (ino >> 12);
        let mut table = self.block(blk);
        put32(&mut table, (ino as usize & 0xFFF) * 4, h);
        self.put_block(blk, &table);
        let mut raw = self.inode(ino);
        put32(&mut raw, 0x0C, h);
        self.put_inode(ino, &raw);
    }

    /// Add a file under directory inode `parent`. -> its inode.
    pub fn add_file(&mut self, parent: u32, name: &[u8], data: &[u8], typ: u8) -> Result<u32, Error> {
        let nblocks = (data.len().div_ceil(BLOCK_BYTES)).max(1) as u32;
        let ino = self.alloc_inode()?;
        let runs = self.alloc_blocks(nblocks)?;
        if runs.len() > MAX_INLINE_EXTENTS {
            return err(format!("needs {} extents; only {MAX_INLINE_EXTENTS} fit", runs.len()));
        }
        let mut padded = data.to_vec();
        padded.resize(nblocks as usize * BLOCK_BYTES, 0);
        let mut raw = vec![0u8; INODE_SIZE];
        raw[0] = typ;
        raw[1] = 2;
        put16(&mut raw, 0x02, 1);
        put32(&mut raw, 0x04, data.len() as u32);
        put32(&mut raw, 0x08, parent);
        let serial = self.next_serial();
        put32(&mut raw, 0x10, serial);
        put16(&mut raw, 0x1E, runs.len() as u16);
        let (mut logical, mut i) = (0u32, 0usize);
        for (k, &(first, run)) in runs.iter().enumerate() {
            put32(&mut raw, 0x20 + k * 12, logical);
            put32(&mut raw, 0x24 + k * 12, run);
            put32(&mut raw, 0x28 + k * 12, first);
            for j in 0..run {
                self.put_block(first + j, &padded[i..i + BLOCK_BYTES]);
                i += BLOCK_BYTES;
            }
            logical += run;
        }
        self.put_inode(ino, &raw);
        self.set_file_hash(ino, data);
        self.add_dir_entry(parent, name, ino, typ)?;
        self.reseal();
        Ok(ino)
    }

    /// Convert a WAV and add it as a sample. -> (inode, name, info).
    pub fn add_sample(&mut self, parent: u32, path: &std::path::Path, wav: &[u8]) -> Result<(u32, String, WavInfo), Error> {
        let (content, info) = wav_to_sample(wav)?;
        let name = sample_name(path);
        let ino = self.add_file(parent, name.as_bytes(), &content, TYPE_FILE)?;
        Ok((ino, name, info))
    }

    /// The inode of a directory named `name` under the root.
    pub fn find_dir(&mut self, name: &str) -> Option<u32> {
        let entries = self.dir_entries(ROOT_INODE).ok()?;
        entries.into_iter().find(|e| e.2 == name.as_bytes() && e.3 == TYPE_DIR && e.1 < RAM_INODE).map(|e| e.1)
    }

    /// (name, inode, type, size) under a directory.
    pub fn list(&mut self, dir: u32) -> Result<Vec<(String, u32, u8, u32)>, Error> {
        let e = self.dir_entries(dir)?;
        Ok(e.into_iter()
            .map(|(_, ino, name, typ)| {
                let size = if ino < RAM_INODE { be32(&self.inode(ino), 4) } else { 0 };
                (String::from_utf8_lossy(&name).into_owned(), ino, typ, size)
            })
            .collect())
    }
}

/// Lay down a fresh ekFS in the sample region, as the firmware's FORMAT
/// +DRIVE does (FUN_400d0cb8): root with 'factory' (a RAM directory) and
/// /incoming.
pub fn format(card: &mut Card) {
    const SB_DEFAULTS: [(usize, u32); 12] = [
        (0x04, 2), (0x08, 0x4000), (0x0C, 0x10000), (0x10, 0xEFE0), (0x14, 0x10), (0x18, 0x30),
        (0x1C, 0x50), (0x20, 0x4050), (0x24, 0x10), (0x28, 0x40), (0x2C, 0x40), (0x30, 0x50),
    ];
    let mut sb = vec![0u8; SECTOR];
    sb[..4].copy_from_slice(b"ekFS");
    for (o, v) in SB_DEFAULTS {
        put32(&mut sb, o, v);
    }
    let mut fs = Ekfs { card, base: REGION, sb };
    let bm = vec![0u8; fs.bitmap_bytes() as usize];
    fs.put_bitmap(true, &bm);
    fs.put_bitmap(false, &bm);
    let blank = vec![0u8; CHUNK_SECTORS as usize * SECTOR];
    for chunk in 0..8 {
        let s = fs.base + fs.u(0x1C) + chunk * CHUNK_SECTORS;
        fs.put_sectors(s, &blank);
    }
    let mut ib = fs.bitmap(true);
    Ekfs::set_bit(&mut ib, 0);
    Ekfs::set_bit(&mut ib, 1);
    let mut bb = fs.bitmap(false);
    let zero = vec![0u8; BLOCK_BYTES];
    for b in 0..RESERVED_BLOCKS {
        Ekfs::set_bit(&mut bb, b);
        fs.put_block(b, &zero);
    }
    let dir_block = |entries: &[(u32, &str, u8)]| {
        let mut buf = vec![0u8; BLOCK_BYTES];
        let mut o = 0;
        for (k, (ino, name, typ)) in entries.iter().enumerate() {
            let rec = if k == entries.len() - 1 { BLOCK_BYTES - o } else { Ekfs::reclen(name.as_bytes()) };
            put32(&mut buf, o, *ino);
            put16(&mut buf, o + 4, rec as u16);
            buf[o + 6] = name.len() as u8;
            buf[o + 7] = *typ;
            buf[o + 8..o + 8 + name.len()].copy_from_slice(name.as_bytes());
            o += rec;
        }
        buf
    };
    let make_dir = |fs: &mut Ekfs, ib: &mut Vec<u8>, bb: &mut Vec<u8>, ino: u32, parent: u32, first: u32, serial: u32, entries: &[(u32, &str, u8)], links: u16| {
        for b in first..first + 4 {
            Ekfs::set_bit(bb, b);
        }
        let mut raw = vec![0u8; INODE_SIZE];
        raw[0] = TYPE_DIR;
        raw[1] = 2;
        put16(&mut raw, 0x02, links);
        put32(&mut raw, 0x04, DIR_SIZE);
        put32(&mut raw, 0x08, 3);
        put32(&mut raw, 0x10, serial);
        put32(&mut raw, 0x1C, parent);
        put16(&mut raw, 0x1E, 2);
        put32(&mut raw, 0x20, 0);
        put32(&mut raw, 0x24, 1);
        put32(&mut raw, 0x28, first);
        put32(&mut raw, 0x2C, 0x20000);
        put32(&mut raw, 0x30, 3);
        put32(&mut raw, 0x34, first + 1);
        Ekfs::set_bit(ib, ino);
        fs.put_inode(ino, &raw);
        fs.put_block(first, &dir_block(entries));
        let mut located = Vec::new();
        let mut o = 0;
        for (e_ino, e_name, e_typ) in entries {
            located.push((o as u32, *e_ino, e_name.as_bytes().to_vec(), *e_typ));
            o += Ekfs::reclen(e_name.as_bytes());
        }
        let idx = build_indexes(&located).unwrap();
        for (k, buf) in idx.iter().enumerate() {
            fs.put_block(first + 1 + k as u32, buf);
        }
    };
    make_dir(&mut fs, &mut ib, &mut bb, 2, 2, RESERVED_BLOCKS, 2, &[(2, ".", TYPE_DIR), (2, "..", TYPE_DIR), (RAM_INODE, "factory", TYPE_DIR), (3, "incoming", TYPE_DIR)], 3);
    make_dir(&mut fs, &mut ib, &mut bb, 3, 2, RESERVED_BLOCKS + 4, 3, &[(3, ".", TYPE_DIR), (2, "..", TYPE_DIR)], 2);
    fs.put_bitmap(true, &ib);
    fs.put_bitmap(false, &bb);
    fs.reseal();
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn lookup3_known_value() {
        // lookup3 hashbig of the empty string is the seed-derived c.
        assert_eq!(ekfs_hash(b"", 0), 0xDEAD_BEEF);
    }

    #[test]
    fn natcmp_quirk() {
        assert_eq!(natcmp(b"a1b", b"a1c"), 0);
        assert!(natcmp(b"a2", b"a10") < 0);
        assert!(natcmp(b"ABC", b"abd") < 0);
    }

    #[test]
    fn format_then_add() {
        let mut card = Card::new();
        format(&mut card);
        let mut fs = Ekfs::open(&mut card).unwrap();
        assert!(fs.checksum_ok());
        let dir = fs.find_dir("incoming").unwrap();
        assert_eq!(dir, 3);
        // A tiny 16-bit mono WAV.
        let mut wav = Vec::new();
        let pcm: Vec<u8> = (0..480i16).flat_map(|i| (i * 60).to_le_bytes()).collect();
        wav.extend_from_slice(b"RIFF");
        wav.extend_from_slice(&(36 + pcm.len() as u32).to_le_bytes());
        wav.extend_from_slice(b"WAVEfmt ");
        wav.extend_from_slice(&16u32.to_le_bytes());
        wav.extend_from_slice(&1u16.to_le_bytes());
        wav.extend_from_slice(&1u16.to_le_bytes());
        wav.extend_from_slice(&48000u32.to_le_bytes());
        wav.extend_from_slice(&96000u32.to_le_bytes());
        wav.extend_from_slice(&2u16.to_le_bytes());
        wav.extend_from_slice(&16u16.to_le_bytes());
        wav.extend_from_slice(b"data");
        wav.extend_from_slice(&(pcm.len() as u32).to_le_bytes());
        wav.extend_from_slice(&pcm);
        let (ino, name, info) = fs.add_sample(dir, std::path::Path::new("ramp.wav"), &wav).unwrap();
        assert_eq!(name, "ramp");
        assert_eq!(info.frames, 480);
        assert!(fs.checksum_ok());
        let list = fs.list(dir).unwrap();
        assert!(list.iter().any(|(n, i, _, s)| n == "ramp" && *i == ino && *s == 64 + 960 + 16));
    }
}
