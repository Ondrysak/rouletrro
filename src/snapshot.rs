//! Save and restore the whole machine.
//!
//! A snapshot holds the CPU registers, every peripheral model, DDR (LZ4
//! compressed; most of 128 MB is zeros), the SRAM, the sparse register
//! pages, the card's sectors, and the machine's own bookkeeping. It is tied
//! to the MAIN OS image by hash: restoring onto a different build would run
//! one firmware's RAM against another's code.

use crate::io::Io;
use crate::machine::{Machine, Stats};
use crate::symbols::MAIN_OS_SHA256;
use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;
use std::io::{Read, Write};
use std::path::Path;

const MAGIC: &[u8; 8] = b"DTEMUSNP";
const VERSION: u32 = 1;

#[derive(Serialize, Deserialize)]
struct CpuState {
    d: [u32; 8],
    a: [u32; 8],
    pc: u32,
    sr: u16,
    other_sp: u32,
    vbr: u32,
    cacr: u32,
    ctl: BTreeMap<u16, u32>,
    macsr: u32,
    mask: u32,
    acc: [u32; 4],
    accext: [u16; 4],
    stopped: bool,
    exc_counts: Vec<u64>,
}

#[derive(Serialize)]
struct SnapRef<'a> {
    main_sha256: String,
    cpu: CpuState,
    io: &'a Io,
    sram: &'a [u8],
    sparse: Vec<(u32, &'a [u8])>,
    ddr_lz4: Vec<u8>,
    stats: &'a Stats,
    frame: &'a Option<Vec<u8>>,
    frame_seq: u64,
}

#[derive(Deserialize)]
struct Snap {
    main_sha256: String,
    cpu: CpuState,
    io: Io,
    sram: Vec<u8>,
    sparse: Vec<(u32, Vec<u8>)>,
    ddr_lz4: Vec<u8>,
    stats: Stats,
    frame: Option<Vec<u8>>,
    frame_seq: u64,
}

pub fn save(m: &Machine, path: &Path) -> std::io::Result<()> {
    let c = &m.cpu;
    let cpu = CpuState {
        d: c.d,
        a: c.a,
        pc: c.pc,
        sr: c.sr,
        other_sp: c.other_sp,
        vbr: c.vbr,
        cacr: c.cacr,
        ctl: c.ctl.clone(),
        macsr: c.macsr,
        mask: c.mask,
        acc: [0, 1, 2, 3].map(|i| c.acc_phys(i).0),
        accext: [0, 1, 2, 3].map(|i| c.acc_phys(i).1),
        stopped: c.stopped,
        exc_counts: c.exc_counts.to_vec(),
    };
    let mut sparse: Vec<(u32, &[u8])> = c.bus.sparse.iter().map(|(k, v)| (*k, &v[..])).collect();
    sparse.sort_by_key(|x| x.0);
    let snap = SnapRef {
        main_sha256: MAIN_OS_SHA256.to_string(),
        cpu,
        io: &c.bus.io,
        sram: &c.bus.sram,
        sparse,
        ddr_lz4: lz4_flex::compress_prepend_size(&c.bus.ddr),
        stats: &m.stats,
        frame: &m.frame,
        frame_seq: m.frame_seq,
    };
    let body = bincode::serialize(&snap).map_err(std::io::Error::other)?;
    let tmp = path.with_extension("tmp");
    {
        let mut f = std::fs::File::create(&tmp)?;
        f.write_all(MAGIC)?;
        f.write_all(&VERSION.to_le_bytes())?;
        f.write_all(&body)?;
    }
    std::fs::rename(tmp, path)
}

pub fn load(m: &mut Machine, path: &Path) -> Result<(), String> {
    let mut f = std::fs::File::open(path).map_err(|e| format!("{}: {e}", path.display()))?;
    let mut head = [0u8; 12];
    f.read_exact(&mut head).map_err(|e| e.to_string())?;
    if &head[..8] != MAGIC {
        return Err(format!("{} is not a snapshot", path.display()));
    }
    let ver = u32::from_le_bytes(head[8..12].try_into().unwrap());
    if ver != VERSION {
        return Err(format!("snapshot version {ver}, this build reads {VERSION}"));
    }
    let mut body = Vec::new();
    f.read_to_end(&mut body).map_err(|e| e.to_string())?;
    let s: Snap = bincode::deserialize(&body).map_err(|e| e.to_string())?;
    if s.main_sha256 != MAIN_OS_SHA256 {
        return Err("snapshot was taken on a different MAIN OS".into());
    }
    let ddr = lz4_flex::decompress_size_prepended(&s.ddr_lz4).map_err(|e| e.to_string())?;
    if ddr.len() != m.cpu.bus.ddr.len() {
        return Err(format!("snapshot DDR is {} bytes, machine has {}", ddr.len(), m.cpu.bus.ddr.len()));
    }
    let c = &mut m.cpu;
    c.bus.ddr = ddr;
    c.bus.icache_flush();
    c.bus.sram = s.sram;
    c.bus.sparse = s
        .sparse
        .into_iter()
        .map(|(k, v)| {
            let mut b = Box::new([0u8; 4096]);
            b[..v.len().min(4096)].copy_from_slice(&v[..v.len().min(4096)]);
            (k, b)
        })
        .collect();
    // Host-side settings are the running machine's, not the snapshot's.
    let (ips, ignore_masks, trace) = (c.bus.io.ips, c.bus.io.ignore_masks, c.bus.io.trace_intc);
    let card_path = c.bus.io.esdhc.card.path.clone();
    c.bus.io = s.io;
    c.bus.io.ips = ips;
    c.bus.io.ignore_masks = ignore_masks;
    c.bus.io.trace_intc = trace;
    c.bus.io.esdhc.card.path = card_path;
    c.bus.io.deadline = c.bus.io.now;
    let r = s.cpu;
    c.d = r.d;
    c.a = r.a;
    c.pc = r.pc;
    c.sr = r.sr;
    c.other_sp = r.other_sp;
    c.vbr = r.vbr;
    c.cacr = r.cacr;
    c.ctl = r.ctl;
    c.macsr = r.macsr;
    c.mask = r.mask;
    for i in 0..4 {
        c.set_acc_phys(i, r.acc[i], r.accext[i]);
    }
    c.stopped = r.stopped;
    for (i, v) in r.exc_counts.iter().enumerate().take(256) {
        c.exc_counts[i] = *v;
    }
    c.skip_hook = None;
    m.stats = s.stats;
    m.frame = s.frame;
    m.frame_seq = s.frame_seq;
    Ok(())
}
