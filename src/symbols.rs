//! Guest addresses in the MAIN OS, found in the image itself.
//!
//! Nothing here is a fixed address: each symbol is located by the shape of
//! its code (a masked instruction pattern, with call targets and DDR
//! addresses left open), and data addresses are read from the operands of
//! the code that uses them. So a rebuilt or patched OS works as long as the
//! code the emulator relies on keeps its shape. Only two are needed to run
//! at all: the entry point and the SPI flash read the emulator serves from
//! the update image. The rest feed the debugging tools and are dropped,
//! with a note, when they cannot be found.

pub const MAIN_LOAD: u32 = 0x4000_0400;

/// OS 1.53's MAIN OS, the build everything here was worked out on.
pub const OS_153_SHA256: &str = "4b47a9507758ca5669ca02ab2c0374d2c04c98aece445408295cc1dcb265c5df";

#[derive(Clone, Debug)]
pub struct Profile {
    /// sha256 of the MAIN OS image, hex.
    pub sha256: String,
    pub entry: u32,
    /// flash_read(offset, length, dest): served from the update image.
    pub flash_read: u32,
    /// Every `bra.b *` in the image: the RTOS idle points.
    pub idle_spins: Vec<u32>,
    pub task_create: Option<u32>,
    pub intro_done: Option<u32>,
    /// The panel task's frame diff; `fb_front` points at the frame it sends.
    pub panel_diff: Option<u32>,
    pub fb_front: Option<u32>,
    /// The `bra.b *` that fatal errors end in.
    pub abort_loop: Option<u32>,
    pub mainloop: Option<u32>,
    pub current_tcb: Option<u32>,
    /// The +Drive's mounted flag.
    pub mounted: Option<u32>,
    /// Symbols not found in this image.
    pub missing: Vec<&'static str>,
}

/// A code pattern: hex bytes, `??` for any byte, whitespace ignored.
struct Pat(Vec<Option<u8>>);

impl Pat {
    fn new(s: &str) -> Pat {
        let h: Vec<u8> = s.bytes().filter(|c| !c.is_ascii_whitespace()).collect();
        Pat(h
            .chunks(2)
            .map(|c| match c {
                b"??" => None,
                _ => Some(u8::from_str_radix(std::str::from_utf8(c).unwrap(), 16).unwrap()),
            })
            .collect())
    }

    /// -> the image offsets (word aligned) where it matches.
    fn find(&self, img: &[u8]) -> Vec<usize> {
        let p = &self.0;
        let first = [p[0].expect("pattern starts with a byte"), p[1].unwrap_or(0)];
        let any2 = p[1].is_none();
        let end = img.len().saturating_sub(p.len());
        // A quick scan for the first word, then the whole pattern.
        img[..end]
            .chunks_exact(2)
            .enumerate()
            .filter(|(_, w)| w[0] == first[0] && (any2 || w[1] == first[1]))
            .map(|(i, _)| 2 * i)
            .filter(|&o| p.iter().zip(&img[o..]).all(|(q, &b)| q.is_none_or(|q| q == b)))
            .collect()
    }
}

fn be32(img: &[u8], o: usize) -> u32 {
    u32::from_be_bytes(img[o..o + 4].try_into().unwrap())
}

/// The address of the one place `pat` matches, plus `at`.
fn code(img: &[u8], pat: &str, at: usize) -> Option<u32> {
    match Pat::new(pat).find(img)[..] {
        [o] => Some(MAIN_LOAD + (o + at) as u32),
        _ => None,
    }
}

/// The 32-bit operand at `at` in every match of `pat`, if they all agree.
fn operand(img: &[u8], pat: &str, at: usize) -> Option<u32> {
    let vals: Vec<u32> = Pat::new(pat).find(img).iter().map(|&o| be32(img, o + at)).collect();
    match vals.first() {
        Some(&v) if vals.iter().all(|&w| w == v) => Some(v),
        _ => None,
    }
}

// The patterns. Each names what it matches; `??` covers call targets,
// pc-relative displacements and DDR addresses, which move between builds.
// Peripheral addresses and constants stay: the hardware does not move.

/// flash_read: takes the SPI lock, then programs the DSPI (0xfc05c000)
/// for a READ command.
const FLASH_READ: &str = "4fef fff4 48d7 040c 242f 0010 262f 0014 246f 0018 4eba ????
    4879 ???????? 4eb9 ???????? 4878 0001 4eb9 ???????? 508f 2039 fc05 c000 7204";
/// task_create(tcb, prio, stack, size, ...): links the TCB into its
/// priority list (prio * 4 + the list heads).
const TASK_CREATE: &str = "202f 000c 72fc c2af 0014 206f 0004 e588 226f 0010 d3c1 43e9 fff4 0680";
/// The intro task, after the intro: stops the intro's timer and pends.
const INTRO_DONE: &str = "588f 4879 ???????? 4241 7810 45f9 ???????? 33c1 fc08 c000 13c4 fc05 001c";
/// The panel frame diff: loads the front and back frame pointers.
const PANEL_DIFF: &str = "4fef ffd8 48d7 1c7c 2479 ???????? 4283 2679 ???????? 49fa";
/// The fatal-error loop: `bra.b *` ahead of a `return -1`.
const ABORT_LOOP: &str = "60fe 70ff 4e75 700c 23c0";
/// The main task's message loop: receive, then dispatch on the type.
const MAINLOOP: &str = "4879 ???????? 4eb9 ???????? 588f 7225 2440 7192 b280 65e8";
/// The context switch saves the registers into the current TCB.
const CTX_SWITCH: &str = "46fc 2700 2f48 fffc 2079 ???????? 48e8";
/// The +Drive calls refuse with -1 unless the drive is mounted.
const MOUNTED_TEST: &str = "4ab9 ???????? 6604 70ff 60";

impl Profile {
    pub fn for_image(img: &[u8]) -> Result<Profile, String> {
        use sha2::{Digest, Sha256};
        if img.len() < 0x100 {
            return Err("MAIN OS image too short".into());
        }
        let sha256: String = Sha256::digest(img).iter().map(|b| format!("{b:02x}")).collect();
        // The image starts with its entry point.
        let entry = be32(img, 0);
        if !(MAIN_LOAD..MAIN_LOAD + img.len() as u32).contains(&entry) || entry & 1 != 0 {
            return Err(format!("MAIN OS entry 0x{entry:08x} is not inside the image"));
        }
        let flash_read = code(img, FLASH_READ, 0)
            .ok_or("MAIN OS: flash_read not found (the emulator serves the OS's SPI flash reads through it)")?;
        let idle_spins = (0..img.len() - 1)
            .step_by(2)
            .filter(|&o| img[o] == 0x60 && img[o + 1] == 0xFE)
            .map(|o| MAIN_LOAD + o as u32)
            .collect();
        let mut p = Profile {
            sha256,
            entry,
            flash_read,
            idle_spins,
            task_create: code(img, TASK_CREATE, 0),
            intro_done: code(img, INTRO_DONE, 0),
            panel_diff: code(img, PANEL_DIFF, 0),
            fb_front: operand(img, PANEL_DIFF, 10),
            abort_loop: code(img, ABORT_LOOP, 0),
            mainloop: code(img, MAINLOOP, 0),
            current_tcb: operand(img, CTX_SWITCH, 10),
            mounted: operand(img, MOUNTED_TEST, 2),
            missing: Vec::new(),
        };
        for (name, v) in [
            ("task_create", p.task_create),
            ("intro_done", p.intro_done),
            ("panel_diff", p.panel_diff),
            ("fb_front", p.fb_front),
            ("abort_loop", p.abort_loop),
            ("mainloop", p.mainloop),
            ("current_tcb", p.current_tcb),
            ("mounted", p.mounted),
        ] {
            if v.is_none() {
                p.missing.push(name);
            }
        }
        Ok(p)
    }

    pub fn is_os_153(&self) -> bool {
        self.sha256 == OS_153_SHA256
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn patterns() {
        let img = [0x4a, 0xb9, 0x12, 0x34, 0x56, 0x78, 0x66, 0x04, 0x70, 0xff, 0x60, 0x00, 0, 0];
        assert_eq!(Pat::new(MOUNTED_TEST).find(&img), vec![0]);
        assert_eq!(operand(&img, MOUNTED_TEST, 2), Some(0x1234_5678));
        assert_eq!(code(&img, "4ab9 ????", 0), Some(MAIN_LOAD));
        assert_eq!(code(&img, "70ff", 0), Some(MAIN_LOAD + 8));
        assert_eq!(code(&img, "ffff", 0), None);
    }
}
