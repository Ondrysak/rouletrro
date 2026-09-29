//! Elektron OS `.syx` → ELE3 container → decompressed sections.
//!
//! Layer cake, outermost first:
//!   1. MIDI SysEx  -- `F0 00 20 3C <dev> 00 <cmd> ... F7` messages
//!   2. 8-in-7      -- each group of 7 data bytes is preceded by a byte
//!                     holding their high bits, MSB first
//!   3. preamble    -- 8 bytes; bytes 4..8 are a 32-bit content checksum
//!   4. ELE3        -- magic, then a section table of 16-byte entries
//!
//! A section's payload is packed when its 8-byte header
//! `[u32 len][u32 byte sum]` is self-consistent; otherwise it is stored raw
//! (the updater, id 4, has a header with sum 0; the build stamp, id 5, has
//! none). Nothing is encrypted or signed.

use std::fmt;

const COUNT_OFF: usize = 0x1C;
const TABLE_OFF: usize = 0x20;
const ENTRY_SZ: usize = 16;

/// Section ids with a known role.
pub const SEC_BOOTSTRAP: u32 = 2;
pub const SEC_MAIN_OS: u32 = 3;
pub const SEC_UPDATER: u32 = 4;
pub const SEC_META: u32 = 5;

#[derive(Debug)]
pub enum Error {
    Syx(String),
    Container(String),
    Depack(String),
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Error::Syx(s) => write!(f, "sysex: {s}"),
            Error::Container(s) => write!(f, "container: {s}"),
            Error::Depack(s) => write!(f, "depack: {s}"),
        }
    }
}

impl std::error::Error for Error {}

pub type Result<T> = std::result::Result<T, Error>;

/// Raw `.syx` bytes → the decoded byte stream (preamble + container).
pub fn decode_syx(d: &[u8]) -> Result<Vec<u8>> {
    let mut out = Vec::with_capacity(d.len());
    let mut i = 0;
    while i < d.len() {
        if d[i] != 0xF0 {
            return Err(Error::Syx(format!("expected F0 at offset {i}")));
        }
        let j = d[i..]
            .iter()
            .position(|&b| b == 0xF7)
            .map(|p| i + p)
            .ok_or_else(|| Error::Syx(format!("unterminated message at {i}")))?;
        let body = &d[i + 1..j];
        i = j + 1;
        // 14-byte bodies are the start/end markers; data messages are 126:
        // a 9-byte header, 116 bytes of 8-in-7 payload, 1 checksum byte.
        if body.len() != 126 {
            continue;
        }
        let payload = &body[9..125];
        let mut k = 0;
        while k < payload.len() {
            let ms = payload[k];
            k += 1;
            for n in 0..7 {
                if k >= payload.len() {
                    break;
                }
                let hi = if (ms >> (6 - n)) & 1 != 0 { 0x80 } else { 0 };
                out.push(payload[k] | hi);
                k += 1;
            }
        }
    }
    Ok(out)
}

#[derive(Clone, Debug)]
pub struct SectionEntry {
    pub id: u32,
    pub offset: u32,
    pub comp_len: u32,
    /// Load address for code sections; for the bootstrap a version word.
    pub dest: u32,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Storage {
    Packed,
    Raw,
}

#[derive(Clone)]
pub struct Section {
    pub entry: SectionEntry,
    pub storage: Storage,
    pub data: Vec<u8>,
}

impl Section {
    pub fn name(&self) -> &'static str {
        match self.entry.id {
            2 => "BOOTSTRAP",
            3 => "MAIN_OS",
            4 => "UPDATER",
            5 => "META",
            8 => "PANEL_MCU",
            _ => "SECTION",
        }
    }
}

pub struct Firmware {
    /// The ELE3 container, starting at its magic.
    pub container: Vec<u8>,
    pub build: String,
    pub version: String,
    pub sections: Vec<Section>,
}

fn be32(b: &[u8], off: usize) -> u32 {
    u32::from_be_bytes([b[off], b[off + 1], b[off + 2], b[off + 3]])
}

/// Decide how a section is stored and return its payload.
///
/// Packed only when the header is self-consistent: the declared length fits
/// and the bytes after it sum to the declared sum. A header with sum 0 is a
/// raw payload with a header to strip; anything else is raw as-is.
pub fn classify(stream: &[u8]) -> (Storage, &[u8]) {
    if stream.len() >= 8 {
        let ln = be32(stream, 0) as usize;
        let sm = be32(stream, 4);
        if ln.checked_add(8).is_some_and(|e| e <= stream.len()) {
            let sum = stream[8..8 + ln]
                .iter()
                .fold(0u32, |a, &b| a.wrapping_add(b as u32));
            if sum == sm {
                return (Storage::Packed, stream);
            }
        }
        if sm == 0 {
            return (Storage::Raw, &stream[8..]);
        }
    }
    (Storage::Raw, stream)
}

impl Firmware {
    pub fn from_syx_bytes(syx: &[u8]) -> Result<Firmware> {
        let dec = decode_syx(syx)?;
        let off = dec
            .windows(4)
            .position(|w| w == b"ELE3")
            .ok_or_else(|| Error::Container("no ELE3 magic".into()))?;
        let c = dec[off..].to_vec();
        if c.len() < TABLE_OFF {
            return Err(Error::Container("truncated header".into()));
        }
        let n = be32(&c, COUNT_OFF) as usize;
        if TABLE_OFF + n * ENTRY_SZ > c.len() {
            return Err(Error::Container(format!("section table of {n} does not fit")));
        }
        let build = String::from_utf8_lossy(&c[0x08..0x14]).trim().to_string();
        let version = String::from_utf8_lossy(&c[0x14..0x18]).to_string();
        let mut sections = Vec::with_capacity(n);
        for k in 0..n {
            let t = TABLE_OFF + ENTRY_SZ * k;
            let entry = SectionEntry {
                id: be32(&c, t),
                offset: be32(&c, t + 4),
                comp_len: be32(&c, t + 8),
                dest: be32(&c, t + 12),
            };
            let (s, e) = (entry.offset as usize, (entry.offset + entry.comp_len) as usize);
            if e > c.len() {
                return Err(Error::Container(format!("section {} overruns container", entry.id)));
            }
            let (storage, payload) = classify(&c[s..e]);
            let data = match storage {
                Storage::Packed => depack_section(payload)?,
                Storage::Raw => payload.to_vec(),
            };
            sections.push(Section { entry, storage, data });
        }
        Ok(Firmware { container: c, build, version, sections })
    }

    pub fn from_file(path: &std::path::Path) -> Result<Firmware> {
        let d = std::fs::read(path)
            .map_err(|e| Error::Syx(format!("cannot read {}: {e}", path.display())))?;
        Self::from_syx_bytes(&d)
    }

    pub fn section(&self, id: u32) -> Option<&Section> {
        self.sections.iter().find(|s| s.entry.id == id)
    }

    pub fn main_os(&self) -> Result<&Section> {
        self.section(SEC_MAIN_OS)
            .ok_or_else(|| Error::Container("no MAIN OS section (id 3)".into()))
    }
}

// ---------------------------------------------------------------------------
// The LZ codec (aPLib-style):
//
//  * control bits are read MSB first from tag bytes fetched on demand;
//    1 = literal byte, 0 = match;
//  * a match begins with an interlaced Elias-gamma g. g == 2 reuses the last
//    offset; otherwise raw = (g << 8) + byte, raw == 767 ends the stream, and
//    offset = raw - 767;
//  * two bits give a short length 1..3; 00 means gamma + 2 follows. Past
//    offset 3328 the length gains 1. The copy count is length + 1.

const BIAS: u32 = 767;
const REUSE: u32 = 2;
const FAR: u32 = 3328;

struct Bits<'a> {
    d: &'a [u8],
    p: usize,
    end: usize,
    tag: u32,
}

impl Bits<'_> {
    fn byte(&mut self) -> Result<u8> {
        if self.p >= self.end {
            return Err(Error::Depack("stream ended early".into()));
        }
        let v = self.d[self.p];
        self.p += 1;
        Ok(v)
    }

    fn bit(&mut self) -> Result<u32> {
        self.tag = (self.tag << 1) & 0x1FF;
        if self.tag & 0xFF == 0 {
            let b = self.byte()? as u32;
            self.tag = (b << 1) | 1;
            return Ok(b >> 7);
        }
        Ok(self.tag >> 8)
    }

    fn gamma(&mut self) -> Result<u32> {
        let mut v = 1u32;
        loop {
            v = (v << 1) | self.bit()?;
            if self.bit()? != 0 {
                return Ok(v);
            }
            if v > 0x0200_0000 {
                return Err(Error::Depack("gamma overflow".into()));
            }
        }
    }
}

/// -> (output, input position after the end marker)
pub fn depack(data: &[u8], pos: usize, end: usize) -> Result<(Vec<u8>, usize)> {
    let mut s = Bits { d: data, p: pos, end, tag: 0 };
    let mut out: Vec<u8> = Vec::with_capacity((end - pos) * 3);
    let mut last = 1u32;
    loop {
        if s.bit()? != 0 {
            out.push(s.byte()?);
            continue;
        }
        let g = s.gamma()?;
        let off = if g == REUSE {
            last
        } else {
            let raw = (g << 8).wrapping_add(s.byte()? as u32);
            if raw == BIAS {
                return Ok((out, s.p));
            }
            let off = raw.wrapping_sub(BIAS);
            last = off;
            off
        };
        let short = 2 * s.bit()? + s.bit()?;
        let mut length = if short != 0 { short } else { s.gamma()? + 2 };
        if off > FAR {
            length += 1;
        }
        let off = off as usize;
        if off == 0 || off > out.len() {
            return Err(Error::Depack(format!(
                "offset {off} outside {} bytes of output",
                out.len()
            )));
        }
        for _ in 0..=length {
            let b = out[out.len() - off];
            out.push(b);
        }
    }
}

/// Depack a section that starts with its `[u32 length][u32 sum]` header.
pub fn depack_section(stream: &[u8]) -> Result<Vec<u8>> {
    let length = be32(stream, 0) as usize;
    let (out, end) = depack(stream, 8, 8 + length)?;
    if end != 8 + length {
        return Err(Error::Depack(format!(
            "end marker at {} of {} stream bytes",
            end - 8,
            length
        )));
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn classify_raw_with_zero_sum_header() {
        let s = [0, 0, 0, 4, 0, 0, 0, 0, 1, 2, 3, 4];
        let (k, p) = classify(&s);
        // The declared length fits but the sum (10) is not 0, so: raw.
        assert_eq!(k, Storage::Raw);
        assert_eq!(p, &[1, 2, 3, 4]);
    }

    #[test]
    fn eight_in_seven() {
        // One 126-byte data message whose first group sets every high bit.
        let mut body = vec![0u8; 126];
        body[9] = 0x7F;
        for b in &mut body[10..17] {
            *b = 0x01;
        }
        let mut syx = vec![0xF0];
        syx.extend(&body);
        syx.push(0xF7);
        let out = decode_syx(&syx).unwrap();
        assert_eq!(&out[..7], &[0x81; 7]);
    }
}
