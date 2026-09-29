//! The +Drive card image: format it, add samples, list it.
//!
//!     dtcard IMAGE format
//!     dtcard IMAGE add FILE.wav... [--dir incoming]
//!     dtcard IMAGE list [--dir incoming]
//!     dtcard IMAGE check
//!
//! The firmware reads the +Drive tree only when it mounts it, at boot, so
//! samples added here appear after the next cold boot (not on a snapshot
//! resume, which carries the drive the snapshot saw).

use dtemu::ekfs::{self, Ekfs};
use dtemu::esdhc::Card;
use std::path::Path;

fn main() {
    let a: Vec<String> = std::env::args().skip(1).collect();
    if a.len() < 2 {
        eprintln!("usage: dtcard IMAGE format | add FILES.. [--dir D] | list [--dir D] | check | hash HEXSEED STRING");
        std::process::exit(2);
    }
    if a[1] == "hash" {
        let seed = u32::from_str_radix(&a[2], 16).unwrap();
        println!("{:08x} {:08x}", ekfs::ekfs_hash(a[3].as_bytes(), seed), ekfs::dx_hack_hash(a[3].as_bytes()));
        return;
    }
    let path = Path::new(&a[0]);
    let mut card = Card::open(path).unwrap_or_else(|e| {
        eprintln!("{}: {e}", path.display());
        std::process::exit(1)
    });
    let dir_name = a.iter().position(|x| x == "--dir").map(|i| a[i + 1].clone()).unwrap_or_else(|| "incoming".into());
    match a[1].as_str() {
        "format" => {
            ekfs::format(&mut card);
            card.dirty = true;
            println!("formatted the sample region");
        }
        "check" | "list" | "add" => {
            let mut fs = Ekfs::open(&mut card).unwrap_or_else(|e| {
                eprintln!("{e} (run `dtcard {} format` first)", path.display());
                std::process::exit(1)
            });
            println!("ekFS: {} inodes used, {} blocks used, checksum {}", fs.used(true), fs.used(false), if fs.checksum_ok() { "OK" } else { "MISMATCH" });
            let Some(dir) = fs.find_dir(&dir_name) else {
                eprintln!("no directory {dir_name:?} under the root");
                std::process::exit(1)
            };
            if a[1] == "add" {
                for f in a[2..].iter().take_while(|x| *x != "--dir") {
                    let data = std::fs::read(f).unwrap();
                    match fs.add_sample(dir, Path::new(f), &data) {
                        Ok((ino, name, info)) => println!("  {name:<32} {} frames at {} Hz ({} ch, {} bit) -> inode {ino}", info.frames, info.rate, info.channels, info.bits),
                        Err(e) => eprintln!("  {f}: {e}"),
                    }
                }
            }
            for (name, ino, typ, size) in fs.list(dir).unwrap() {
                println!("  {:<32} {} inode {:>8} {:>10} bytes", name, if typ == ekfs::TYPE_DIR { "dir " } else { "file" }, ino, size);
            }
        }
        c => {
            eprintln!("unknown command {c}");
            std::process::exit(2);
        }
    }
    card.flush().unwrap();
}
