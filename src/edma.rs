//! The eDMA engine: moves bytes for channels the firmware has started.
//!
//! Register state lives in `io::Edma`; transfers run here, on the bus, so a
//! descriptor can read and write peripheral registers (UART data registers,
//! SSI FIFOs) as the hardware would.
//!
//! TCD layout on this SoC (not the Kinetis order): SADDR +0, ATTR +4,
//! SOFF +6, NBYTES +8, SLAST +C, DADDR +10, CITER +14, DOFF +16,
//! DLAST_SGA +18, BITER +1C, CSR +1E.
//!
//! A software START runs the whole major loop at once, following minor and
//! major channel links. A hardware request runs one minor loop per request,
//! except UART transmit, whose request line never drops, so it too runs to
//! major completion; the channel is then busy for as long as the bytes take
//! on the wire, which is when the firmware's completion handler would run.

use crate::bus::Bus;

const CSR_START: u16 = 0x0001;
const CSR_INTMAJOR: u16 = 0x0002;
const CSR_INTHALF: u16 = 0x0004;
const CSR_DREQ: u16 = 0x0008;
const CSR_ESG: u16 = 0x0010;
const CSR_MAJORELINK: u16 = 0x0020;
const CSR_ACTIVE: u16 = 0x0040;
const CSR_DONE: u16 = 0x0080;

fn size_of(code: u16) -> u32 {
    match code & 7 {
        0 => 1,
        1 => 2,
        2 => 4,
        4 => 16,
        5 => 32,
        _ => 1,
    }
}

fn modulo_add(addr: u32, delta: u32, m: u16) -> u32 {
    if m == 0 {
        return addr.wrapping_add(delta);
    }
    let mask = (1u32 << m) - 1;
    (addr & !mask) | (addr.wrapping_add(delta) & mask)
}

#[derive(Clone, Copy, Debug, Default)]
pub struct Tcd {
    pub saddr: u32,
    pub attr: u16,
    pub soff: u16,
    pub nbytes: u32,
    pub slast: u32,
    pub daddr: u32,
    pub citer: u16,
    pub doff: u16,
    pub dlast: u32,
    pub biter: u16,
    pub csr: u16,
}

impl Tcd {
    pub fn from_bytes(b: &[u8]) -> Tcd {
        let u32_ = |o: usize| u32::from_be_bytes(b[o..o + 4].try_into().unwrap());
        let u16_ = |o: usize| u16::from_be_bytes(b[o..o + 2].try_into().unwrap());
        Tcd {
            saddr: u32_(0),
            attr: u16_(4),
            soff: u16_(6),
            nbytes: u32_(8),
            slast: u32_(0xC),
            daddr: u32_(0x10),
            citer: u16_(0x14),
            doff: u16_(0x16),
            dlast: u32_(0x18),
            biter: u16_(0x1C),
            csr: u16_(0x1E),
        }
    }

    pub fn to_bytes(&self, b: &mut [u8]) {
        b[0..4].copy_from_slice(&self.saddr.to_be_bytes());
        b[4..6].copy_from_slice(&self.attr.to_be_bytes());
        b[6..8].copy_from_slice(&self.soff.to_be_bytes());
        b[8..12].copy_from_slice(&self.nbytes.to_be_bytes());
        b[12..16].copy_from_slice(&self.slast.to_be_bytes());
        b[16..20].copy_from_slice(&self.daddr.to_be_bytes());
        b[20..22].copy_from_slice(&self.citer.to_be_bytes());
        b[22..24].copy_from_slice(&self.doff.to_be_bytes());
        b[24..28].copy_from_slice(&self.dlast.to_be_bytes());
        b[28..30].copy_from_slice(&self.biter.to_be_bytes());
        b[30..32].copy_from_slice(&self.csr.to_be_bytes());
    }

    fn count(raw: u16) -> u16 {
        if raw & 0x8000 != 0 {
            raw & 0x01FF
        } else {
            raw & 0x7FFF
        }
    }

    fn link(raw: u16) -> Option<usize> {
        if raw & 0x8000 != 0 {
            Some(((raw >> 9) & 0x3F) as usize)
        } else {
            None
        }
    }
}

impl Bus {
    pub fn tcd(&self, ch: usize) -> Tcd {
        Tcd::from_bytes(&self.io.edma.tcd[ch * 32..ch * 32 + 32])
    }

    fn set_tcd(&mut self, ch: usize, t: &Tcd) {
        t.to_bytes(&mut self.io.edma.tcd[ch * 32..ch * 32 + 32]);
    }

    fn dma_read(&mut self, a: u32, n: u32) -> u32 {
        match n {
            1 => self.read8(a) as u32,
            2 => self.read16(a) as u32,
            _ => self.read32(a),
        }
    }

    fn dma_write(&mut self, a: u32, n: u32, v: u32) {
        match n {
            1 => self.write8(a, v as u8),
            2 => self.write16(a, v as u16),
            _ => self.write32(a, v),
        }
    }

    /// One minor loop. -> false if the channel has nothing left to do.
    fn dma_minor(&mut self, ch: usize) -> bool {
        let t = self.tcd(ch);
        let citer = Tcd::count(t.citer);
        if citer == 0 {
            return false;
        }
        let ssize = size_of(t.attr >> 8);
        let dsize = size_of(t.attr);
        let smod = (t.attr >> 11) & 0x1F;
        let dmod = (t.attr >> 3) & 0x1F;
        let mut nbytes = t.nbytes & 0x3FFF_FFFF;
        if nbytes == 0 {
            nbytes = 1 << 30;
        }
        let nbytes = nbytes.min(1 << 22);
        let soff = t.soff as i16 as i32 as u32;
        let doff = t.doff as i16 as i32 as u32;
        let (mut s, mut d) = (t.saddr, t.daddr);
        if ssize == dsize {
            // Equal sizes: each read goes straight to its write.
            let unit = ssize.min(4);
            let mut moved = 0;
            while moved < nbytes {
                for k in 0..ssize / unit {
                    let v = self.dma_read(s.wrapping_add(k * unit), unit);
                    self.dma_write(d.wrapping_add(k * unit), unit, v);
                }
                moved += ssize;
                s = modulo_add(s, soff, smod);
                d = modulo_add(d, doff, dmod);
            }
            return self.dma_minor_done(ch, citer, s, d);
        }
        // Reads round up to whole source units; the audio's loops fit on
        // the stack.
        let cap = (nbytes + ssize) as usize;
        let mut stack = [0u8; 96];
        let mut heap = Vec::new();
        let buf: &mut [u8] = if cap <= stack.len() {
            &mut stack
        } else {
            heap.resize(cap, 0);
            &mut heap
        };
        let mut len = 0usize;
        let mut moved = 0;
        while moved < nbytes {
            // A 16- or 32-byte burst moves as a run of longwords.
            let unit = ssize.min(4);
            for k in 0..ssize / unit {
                let v = self.dma_read(s.wrapping_add(k * unit), unit);
                let u = unit as usize;
                buf[len..len + u].copy_from_slice(&v.to_be_bytes()[4 - u..]);
                len += u;
            }
            moved += ssize;
            s = modulo_add(s, soff, smod);
        }
        let mut written = 0;
        while written + dsize <= len as u32 {
            let unit = dsize.min(4);
            for k in 0..dsize / unit {
                let off = (written + k * unit) as usize;
                let mut w = [0u8; 4];
                let u = unit as usize;
                w[4 - u..].copy_from_slice(&buf[off..off + u]);
                self.dma_write(d.wrapping_add(k * unit), unit, u32::from_be_bytes(w));
            }
            written += dsize;
            d = modulo_add(d, doff, dmod);
        }
        self.dma_minor_done(ch, citer, s, d)
    }

    /// Where `[a, a + n)` is plain memory a transfer may touch directly --
    /// DDR no watchpoint or cached code covers, or SRAM -- its first byte.
    fn plain_ptr(&mut self, a: u32, n: u32) -> Option<*mut u8> {
        if let Some(o) = self.ddr_plain(a, n) {
            return Some(self.ddr[o..].as_mut_ptr());
        }
        let o = Self::sram_offset(a)?;
        if o + n as usize <= crate::bus::SRAM_SIZE && Self::sram_offset(a.wrapping_add(n - 1)) == Some(o + n as usize - 1) {
            return Some(self.sram[o..].as_mut_ptr());
        }
        None
    }

    /// Run channel `ch`'s minor loops, from the first, for as long as each
    /// moves plain memory only, and without a minor link elsewhere: what `dma_minor`
    /// would do loop by loop, with the descriptor read once and written
    /// back once, and each loop a direct copy. Stops after the major loop
    /// (whose completion it runs) or before a loop it cannot copy, which
    /// `dma_minor` then runs. -> loops run.
    fn dma_minors_plain(&mut self, ch: usize, max: u32) -> u32 {
        let t = self.tcd(ch);
        // A minor link to the channel itself does nothing (as in
        // `dma_minor_done`); one to another channel runs there.
        if Tcd::link(t.citer).is_some_and(|l| l != ch) {
            return 0;
        }
        let mut count = Tcd::count(t.citer);
        let ssize = size_of(t.attr >> 8);
        let dsize = size_of(t.attr);
        let (smod, dmod) = ((t.attr >> 11) & 0x1F, (t.attr >> 3) & 0x1F);
        let nbytes = t.nbytes & 0x3FFF_FFFF;
        // Small loops only; a source chunk at most rounds a loop up.
        if nbytes == 0 || nbytes + ssize > 256 || nbytes.div_ceil(ssize) > 32 || (nbytes + ssize) / dsize > 64 {
            return 0;
        }
        let soff = t.soff as i16 as i32 as u32;
        let doff = t.doff as i16 as i32 as u32;
        let half = Tcd::count(t.biter) / 2;
        let (mut s, mut d) = (t.saddr, t.daddr);
        let mut done = 0;
        let mut src = [std::ptr::null_mut::<u8>(); 32];
        let mut dst = [std::ptr::null_mut::<u8>(); 64];
        let mut buf = [0u8; 256];
        'minor: while count > 0 && done < max {
            // Every chunk first: nothing moves unless all are plain.
            let (mut s2, mut d2) = (s, d);
            let mut ns = 0;
            let mut moved = 0;
            while moved < nbytes {
                match self.plain_ptr(s2, ssize) {
                    Some(p) => src[ns] = p,
                    None => break 'minor,
                }
                ns += 1;
                moved += ssize;
                s2 = modulo_add(s2, soff, smod);
            }
            let len = ns as u32 * ssize;
            let mut nd = 0;
            let mut written = 0;
            while written + dsize <= len {
                match self.plain_ptr(d2, dsize) {
                    Some(p) => dst[nd] = p,
                    None => break 'minor,
                }
                nd += 1;
                written += dsize;
                d2 = modulo_add(d2, doff, dmod);
            }
            // SAFETY: each pointer covers its chunk in DDR or SRAM, which
            // nothing resizes while the bus is borrowed here.
            unsafe {
                if ssize == dsize {
                    // As `dma_minor`: each unit (at most a longword) read,
                    // then written.
                    let unit = ssize.min(4) as usize;
                    for (&a, &b) in src[..ns].iter().zip(&dst[..nd]) {
                        for k in (0..ssize as usize).step_by(unit) {
                            let mut v = [0u8; 4];
                            std::ptr::copy_nonoverlapping(a.add(k), v.as_mut_ptr(), unit);
                            std::ptr::copy_nonoverlapping(v.as_ptr(), b.add(k), unit);
                        }
                    }
                } else {
                    // All reads, then all writes.
                    for (i, &a) in src[..ns].iter().enumerate() {
                        std::ptr::copy_nonoverlapping(a, buf.as_mut_ptr().add(i * ssize as usize), ssize as usize);
                    }
                    for (i, &b) in dst[..nd].iter().enumerate() {
                        std::ptr::copy_nonoverlapping(buf.as_ptr().add(i * dsize as usize), b, dsize as usize);
                    }
                }
            }
            s = s2;
            d = d2;
            count -= 1;
            done += 1;
            if t.csr & CSR_INTHALF != 0 && count == half && half != 0 {
                self.io.edma.int |= 1 << ch;
            }
        }
        if done == 0 {
            return 0;
        }
        // As `dma_minor_done` leaves it after the last loop.
        let b = &mut self.io.edma.tcd[ch * 32..ch * 32 + 32];
        let csr = ((t.csr & !CSR_START) | CSR_ACTIVE) & !CSR_DONE;
        b[0..4].copy_from_slice(&s.to_be_bytes());
        b[0x10..0x14].copy_from_slice(&d.to_be_bytes());
        let mask = if t.citer & 0x8000 != 0 { 0x01FF } else { 0x7FFF };
        b[0x14..0x16].copy_from_slice(&((t.citer & !mask) | count).to_be_bytes());
        b[0x1E..0x20].copy_from_slice(&csr.to_be_bytes());
        if count == 0 {
            self.dma_major_done(ch);
        }
        done
    }

    /// Minor loop bookkeeping: the new addresses and count, the half and
    /// major completions, the minor link.
    fn dma_minor_done(&mut self, ch: usize, citer: u16, s: u32, d: u32) -> bool {
        // Only the addresses, the count and CSR change: update them in
        // place (re-reading the rest, which the transfer may have written).
        let b = &mut self.io.edma.tcd[ch * 32..ch * 32 + 32];
        let raw_citer = u16::from_be_bytes([b[0x14], b[0x15]]);
        let biter = u16::from_be_bytes([b[0x1C], b[0x1D]]);
        let csr = u16::from_be_bytes([b[0x1E], b[0x1F]]);
        let new_count = citer - 1;
        let new_citer = (raw_citer & !(if raw_citer & 0x8000 != 0 { 0x01FF } else { 0x7FFF })) | new_count;
        let new_csr = ((csr & !CSR_START) | CSR_ACTIVE) & !CSR_DONE;
        b[0..4].copy_from_slice(&s.to_be_bytes());
        b[0x10..0x14].copy_from_slice(&d.to_be_bytes());
        b[0x14..0x16].copy_from_slice(&new_citer.to_be_bytes());
        b[0x1E..0x20].copy_from_slice(&new_csr.to_be_bytes());
        let half = Tcd::count(biter) / 2;
        if new_csr & CSR_INTHALF != 0 && new_count == half && half != 0 {
            self.io.edma.int |= 1 << ch;
        }
        if new_count == 0 {
            self.dma_major_done(ch);
        } else if let Some(l) = Tcd::link(new_citer) {
            if l != ch {
                self.dma_start_linked(l);
            }
        }
        true
    }

    fn dma_major_done(&mut self, ch: usize) {
        let mut t = self.tcd(ch);
        self.io.edma.majors[ch] += 1;
        let smod = (t.attr >> 11) & 0x1F;
        let dmod = (t.attr >> 3) & 0x1F;
        t.saddr = modulo_add(t.saddr, t.slast, smod);
        let csr = t.csr;
        if csr & CSR_ESG != 0 {
            // Scatter/gather: DLAST_SGA points at the next descriptor.
            let mut nb = [0u8; 32];
            for (i, b) in nb.iter_mut().enumerate() {
                *b = self.read8(t.dlast.wrapping_add(i as u32));
            }
            // The channel now holds the next descriptor, CSR included -- which
            // is how E_SG clears at the end of a chain. One loaded with START
            // set runs on.
            let n = Tcd::from_bytes(&nb);
            self.set_tcd(ch, &n);
            if n.csr & CSR_START != 0 {
                self.io.edma.kick |= 1 << ch;
            }
        } else {
            t.daddr = modulo_add(t.daddr, t.dlast, dmod);
            t.citer = t.biter;
            t.csr = (t.csr & !(CSR_ACTIVE | CSR_START)) | CSR_DONE;
            self.set_tcd(ch, &t);
        }
        if csr & CSR_INTMAJOR != 0 {
            self.io.edma.int |= 1 << ch;
        }
        if csr & CSR_DREQ != 0 {
            self.io.edma.erq &= !(1 << ch);
        }
        if csr & CSR_MAJORELINK != 0 {
            let l = ((csr >> 8) & 0x3F) as usize;
            if l != ch {
                self.dma_start_linked(l);
            }
        }
    }

    fn dma_start_linked(&mut self, ch: usize) {
        let o = ch * 32 + 0x1F;
        self.io.edma.tcd[o] |= CSR_START as u8;
        self.io.edma.kick |= 1 << ch;
    }

    /// Run every channel with work waiting. Called after I/O writes and
    /// from the machine's service loop.
    pub fn run_dma(&mut self) {
        if self.in_dma {
            return;
        }
        self.in_dma = true;
        self.run_dma_inner();
        self.in_dma = false;
        self.io.update_irq();
    }

    fn run_dma_inner(&mut self) {
        let mut guard = 0;
        while self.io.edma.kick != 0 && guard < 4096 {
            guard += 1;
            let ch = self.io.edma.kick.trailing_zeros() as usize;
            if self.io.edma.busy_until[ch] > self.io.now {
                // Still busy on the wire; the service loop comes back.
                let k = self.io.edma.kick & !(1 << ch);
                let rest = self.io.edma.kick & (1 << ch);
                self.io.edma.kick = k;
                self.io.edma.deferred |= rest;
                continue;
            }
            self.io.edma.kick &= !(1 << ch);
            let t = self.tcd(ch);
            let sw = t.csr & CSR_START != 0;
            let hw = self.io.edma.erq & (1 << ch) != 0 && self.io.dma_requesting() & (1 << ch) != 0;
            if !sw && !hw {
                continue;
            }
            if sw {
                // A software start runs the whole major loop.
                let before = self.io.edma.majors[ch];
                let mut n = self.dma_minors_plain(ch, 0x10001);
                while self.io.edma.majors[ch] == before && n <= 0x10000 && self.dma_minor(ch) {
                    n += 1;
                    if self.io.edma.majors[ch] != before || n > 0x10000 {
                        break;
                    }
                }
            } else if ch == 35 {
                // UART8 transmit: its request never drops, so run to the end
                // of the major loop; the channel is then busy for as long as
                // the bytes take on the wire.
                let mut bytes = 0u64;
                loop {
                    let nb = self.tcd(ch).nbytes as u64;
                    if !self.dma_minor(ch) {
                        break;
                    }
                    bytes += nb;
                    if self.tcd(ch).csr & CSR_DONE != 0 || bytes > 1 << 20 {
                        break;
                    }
                }
                self.io.edma.busy_until[ch] = self.io.now + bytes * self.io.uart_byte_instr;
                if self.io.edma.busy_until[ch] < self.io.deadline {
                    self.io.deadline = self.io.edma.busy_until[ch];
                }
            } else {
                // A minor loop per request, for as long as the peripheral
                // keeps asking (a receive FIFO with data, a storage transfer
                // with blocks left).
                let mut n = 0;
                while self.io.edma.erq & (1 << ch) != 0 && self.io.dma_requesting() & (1 << ch) != 0 {
                    // An SSI request is one frame: one minor loop each.
                    match ch {
                        crate::io::SSI_TX_CHAN => self.io.ssi.tx_pending -= 1,
                        crate::io::SSI_RX_CHAN => self.io.ssi.rx_pending -= 1,
                        _ => {}
                    }
                    if !self.dma_minor(ch) {
                        break;
                    }
                    n += 1;
                    if n > 1 << 20 {
                        break;
                    }
                }
            }
        }
        self.io.update_irq();
    }

    /// Re-offer channels that were busy; called when time has advanced.
    pub fn dma_service(&mut self) {
        let now = self.io.now;
        let mut again = 0u64;
        for ch in 0..64 {
            if self.io.edma.busy_until[ch] != 0 && self.io.edma.busy_until[ch] <= now {
                self.io.edma.busy_until[ch] = 0;
                if self.io.edma.erq & (1 << ch) != 0 {
                    again |= 1 << ch;
                }
            }
        }
        again |= self.io.edma.deferred;
        self.io.edma.deferred = 0;
        again &= self.io.dma_requesting() | self.io.edma.kick;
        self.io.edma.kick |= again;
        if self.io.edma.kick != 0 {
            self.run_dma();
        }
    }
}
