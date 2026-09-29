//! Print a firmware's container and sections, optionally extracting them.
//!
//!     fwinfo FIRMWARE.syx [-o DIR]

use dtemu::firmware::Firmware;
use sha2::{Digest, Sha256};
use std::path::PathBuf;

fn hex(b: &[u8]) -> String {
    b.iter().map(|x| format!("{x:02x}")).collect()
}

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let mut syx = None;
    let mut out: Option<PathBuf> = None;
    let mut it = args.iter();
    while let Some(a) = it.next() {
        match a.as_str() {
            "-o" => out = it.next().map(PathBuf::from),
            "-h" | "--help" => {
                println!("usage: fwinfo FIRMWARE.syx [-o DIR]");
                return;
            }
            _ => syx = Some(PathBuf::from(a)),
        }
    }
    let Some(syx) = syx else {
        eprintln!("usage: fwinfo FIRMWARE.syx [-o DIR]");
        std::process::exit(2);
    };
    let raw = std::fs::read(&syx).unwrap_or_else(|e| panic!("{}: {e}", syx.display()));
    println!("file sha256 {}", hex(&Sha256::digest(&raw)));
    let fw = Firmware::from_syx_bytes(&raw).unwrap_or_else(|e| {
        eprintln!("{e}");
        std::process::exit(1)
    });
    println!("build {:?} version {:?}, {} sections", fw.build, fw.version, fw.sections.len());
    for s in &fw.sections {
        println!(
            "  id={} {:<10} {:?} off=0x{:06x} stored={:>7} expanded={:>8} dest=0x{:08x} sha256={}",
            s.entry.id,
            s.name(),
            s.storage,
            s.entry.offset,
            s.entry.comp_len,
            s.data.len(),
            s.entry.dest,
            hex(&Sha256::digest(&s.data))
        );
    }
    if let Some(dir) = out {
        std::fs::create_dir_all(&dir).unwrap();
        for s in &fw.sections {
            let p = dir.join(format!("section_{}_{}.bin", s.entry.id, s.name()));
            std::fs::write(&p, &s.data).unwrap();
        }
        println!("wrote {}", dir.display());
    }
}
