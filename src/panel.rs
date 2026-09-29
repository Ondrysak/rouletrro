//! The front-panel microcontroller at the far end of UART8.
//!
//! A separate MCU scans the keys and encoders and drives the OLED and the
//! key LEDs; the ColdFire talks to it over UART8. This models it from the
//! stream, as the MCU sees it (reference project, emu/panelleds.py, read
//! from both the ColdFire's builders and the MCU's own parser):
//!
//! ```text
//!   0x            1  ignored
//!   1p cc d0..d7 10  OLED tile: page p, columns cc..cc+7
//!   2g ss         2  LED selectors for LEDs 4g..4g+3, 2 bits each
//!   Bs id vv      3  slot s (0..3) of LED id := palette index vv
//!   B4 id v0..v3  6  all four slots
//!   B5 i r g b    5  palette entry i := RGB, 0..31 each
//!   B6 ..         3  accepted, unused
//!   B7 x          2  OLED contrast
//!   B8            1  end of OLED frame
//!   anything else 2
//! ```
//!
//! Queries the MCU answers:
//!
//! ```text
//!   60 01        every button group: [0x20 | group, mask] x 6
//!   70 00/71 00  the UI card: 70, type 4 (Digitakt), UI fw 1.2.0, tested
//!   74 00        the serial number: 70 then 9 bytes
//! ```
//!
//! and what it sends on its own: `[0x20 | channel, mask]` for a button
//! group's state (a bitmask, the firmware derives edges), `[0x30 | encoder,
//! delta]` for an encoder turn (signed).

use serde::{Deserialize, Serialize};
use serde_big_array::BigArray;

pub const GROUPS: u8 = 6;
pub const CARD: [u8; 4] = [0x04, 0x02, 0x00, 0x01];
pub const SERIAL: &[u8; 12] = b"EMULATED0000";
pub const LEDS: usize = 44;
pub const LED_GROUPS: usize = 11;
pub const PALETTE: usize = 41;

/// Control code -> (wire channel, bit), measured on the device firmware
/// (reference devices/digitakt.toml `[panel.exceptions]`).
pub const WIRE: [(u8, u8, u8); 48] = [
    (24, 0, 0), (25, 0, 1), (26, 0, 2), (27, 0, 3), (28, 0, 4), (29, 0, 5), (30, 0, 6), (31, 0, 7),
    (32, 1, 0), (33, 1, 1), (34, 1, 2), (35, 1, 3), (36, 1, 4), (37, 1, 5), (38, 1, 6), (39, 1, 7),
    (2, 2, 0), (1, 2, 1), (19, 2, 2), (20, 2, 3), (0, 2, 4), (21, 2, 5), (22, 2, 6), (23, 2, 7),
    (6, 3, 0), (7, 3, 1), (8, 3, 2), (13, 3, 3), (16, 3, 4), (15, 3, 5), (17, 3, 6), (18, 3, 7),
    (4, 4, 0), (3, 4, 1), (5, 4, 2), (9, 4, 3), (10, 4, 4), (11, 4, 5), (12, 4, 6), (14, 4, 7),
    (46, 5, 0), (45, 5, 1), (44, 5, 2), (41, 5, 3), (40, 5, 4), (43, 5, 5), (42, 5, 6), (47, 5, 7),
];

/// Measured key names (reference devices/digitakt.toml `[panel.labels]`),
/// by control code.
pub fn key_name(code: u8) -> Option<&'static str> {
    Some(match code {
        1 => "FUNC",
        2 => "TRK",
        3 => "PTN",
        4 => "BANK",
        5 => "SONG",
        6 => "GLOBAL",
        7 => "SAMPLE",
        8 => "TEMPO",
        9 => "RECORD",
        10 => "PLAY",
        11 => "STOP",
        12 => "YES",
        13 => "NO",
        14 => "UP",
        15 => "DOWN",
        16 => "LEFT",
        17 => "RIGHT",
        18 => "PAGE",
        19 => "TRIG",
        20 => "SRC",
        21 => "FLTR",
        22 => "AMP",
        23 => "LFO",
        24..=39 => ["1", "2", "3", "4", "5", "6", "7", "8", "9", "10", "11", "12", "13", "14", "15", "16"][(code - 24) as usize],
        40..=47 => ["A", "B", "C", "D", "E", "F", "G", "H"][(code - 40) as usize],
        _ => return None,
    })
}

/// The LED that lights key `code`, if it has one (reference `[panel.leds]`).
pub fn led_for_key(code: u8) -> Option<usize> {
    const MAP: [(u8, u8); 39] = [
        (0, 24), (1, 25), (2, 26), (3, 27), (4, 28), (5, 29), (6, 30), (7, 31), (8, 32), (9, 33), (10, 34), (11, 35),
        (12, 36), (13, 37), (14, 38), (15, 39), (16, 4), (17, 3), (18, 19), (19, 20), (20, 21), (21, 22), (22, 23),
        (23, 18), (24, 11), (25, 8), (26, 12), (27, 13), (28, 14), (29, 16), (30, 15), (31, 17), (32, 1), (33, 5),
        (34, 6), (35, 7), (36, 9), (37, 10), (38, 2),
    ];
    MAP.iter().find(|(_, k)| *k == code).map(|(l, _)| *l as usize)
}

/// The four pattern-page LEDs, page 1 first.
pub const PAGE_LEDS: [usize; 4] = [43, 42, 41, 40];

#[derive(Clone, Serialize, Deserialize)]
pub struct PanelMcu {
    /// Bytes of a message still arriving.
    partial: Vec<u8>,
    /// Keys held, per wire channel.
    pub held: [u8; 8],
    /// The OLED as the MCU holds it: byte = page + 8 * column, bit n = row
    /// 8 * (7 - page) + n (page 0 is the bottom band).
    #[serde(with = "BigArray")]
    pub oled: [u8; 1024],
    /// Frames completed (B8 seen).
    pub oled_frames: u64,
    pub contrast: u8,
    /// Per LED, per slot: a palette index (0xFF = undefined).
    pub slots: Vec<[u8; 4]>,
    /// Per selector group: the last selector byte, once one was sent.
    pub selectors: Vec<Option<u8>>,
    /// RGB, 0..31 each; None until defined.
    pub palette: Vec<Option<(u8, u8, u8)>>,
    pub queries: u64,
    pub unknown: u64,
}

impl Default for PanelMcu {
    fn default() -> Self {
        PanelMcu {
            partial: Vec::new(),
            held: [0; 8],
            oled: [0; 1024],
            oled_frames: 0,
            contrast: 0,
            slots: vec![[0xFF; 4]; 256],
            selectors: vec![None; LED_GROUPS],
            palette: vec![None; PALETTE + 1],
            queries: 0,
            unknown: 0,
        }
    }
}

fn msg_len(h: u8) -> usize {
    match h >> 4 {
        0x0 => 1,
        0x1 => 10,
        0xB => match h {
            0xB0..=0xB3 | 0xB6 => 3,
            0xB4 => 6,
            0xB5 => 5,
            0xB7 => 2,
            0xB8 => 1,
            _ => 2,
        },
        _ => 2,
    }
}

impl PanelMcu {
    /// A byte from the ColdFire. Replies are appended to `reply`.
    pub fn from_host(&mut self, b: u8, reply: &mut Vec<u8>) {
        self.partial.push(b);
        let n = msg_len(self.partial[0]);
        if self.partial.len() < n {
            return;
        }
        let m = std::mem::take(&mut self.partial);
        self.message(&m, reply);
    }

    fn message(&mut self, m: &[u8], reply: &mut Vec<u8>) {
        let h = m[0];
        match h >> 4 {
            0x0 => {}
            0x1 => {
                let page = (h & 7) as usize;
                let col = m[1] as usize;
                for (k, &d) in m[2..10].iter().enumerate() {
                    let c = col + k;
                    if c < 128 {
                        self.oled[page + 8 * c] = d;
                    }
                }
            }
            0x2 => {
                let g = (h & 0xF) as usize;
                if g < self.selectors.len() {
                    self.selectors[g] = Some(m[1]);
                }
            }
            0x6 if m[1] == 0x01 && h == 0x60 => {
                self.queries += 1;
                for g in 0..GROUPS {
                    reply.push(0x20 | g);
                    reply.push(self.held[g as usize]);
                }
            }
            0x7 if m[1] == 0x00 && (h == 0x70 || h == 0x71) => {
                self.queries += 1;
                reply.push(0x70);
                reply.extend_from_slice(&CARD);
            }
            0x7 if m[1] == 0x00 && h == 0x74 => {
                self.queries += 1;
                reply.push(0x70);
                reply.extend_from_slice(&SERIAL[..9]);
            }
            0xB => match h {
                0xB0..=0xB3 => self.slots[m[1] as usize][(h & 3) as usize] = m[2],
                0xB4 => self.slots[m[1] as usize].copy_from_slice(&m[2..6]),
                0xB5 => {
                    let i = m[1] as usize;
                    if i < self.palette.len() {
                        self.palette[i] = Some((m[2], m[3], m[4]));
                    }
                }
                0xB7 => self.contrast = m[1],
                0xB8 => self.oled_frames += 1,
                _ => {}
            },
            _ => self.unknown += 1,
        }
    }

    /// The colour LED `led` shows, as 8-bit RGB, if defined.
    pub fn led_rgb(&self, led: usize) -> Option<(u8, u8, u8)> {
        let s = (*self.selectors.get(led >> 2)?)?;
        let slot = (s >> ((led & 3) * 2)) & 3;
        let idx = self.slots[led][slot as usize];
        let (r, g, b) = (*self.palette.get(idx as usize)?)?;
        let c = |v: u8| (v.min(31) as u16 * 255 / 31) as u8;
        Some((c(r), c(g), c(b)))
    }

    /// Set a key's state. -> the wire message to send, if it changed.
    pub fn key(&mut self, code: u8, down: bool) -> Option<[u8; 2]> {
        let &(_, ch, bit) = WIRE.iter().find(|w| w.0 == code)?;
        let old = self.held[ch as usize];
        let new = if down { old | (1 << bit) } else { old & !(1 << bit) };
        self.held[ch as usize] = new;
        (new != old).then_some([0x20 | ch, new])
    }

    /// Everything held, released. -> the wire messages.
    pub fn release_all(&mut self) -> Vec<u8> {
        let mut out = Vec::new();
        for ch in 0..8u8 {
            if self.held[ch as usize] != 0 {
                self.held[ch as usize] = 0;
                out.extend_from_slice(&[0x20 | ch, 0]);
            }
        }
        out
    }

    /// Turn encoder `ch` (0..8: A..H, then LEVEL/DATA) by `delta` counts.
    pub fn encoder(ch: u8, delta: i8) -> [u8; 2] {
        [0x30 | (ch & 0xF), delta as u8]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn selectors_and_palette() {
        let mut p = PanelMcu::default();
        let mut r = Vec::new();
        for &b in &[0xb8u8, 0x20, 0x01, 0x21, 0x02, 0xb5, 0x03, 0x1f, 0x00, 0x00, 0xb1, 0x00, 0x03] {
            p.from_host(b, &mut r);
        }
        assert_eq!(p.selectors[0], Some(0x01));
        assert_eq!(p.selectors[1], Some(0x02));
        assert_eq!(p.palette[3], Some((31, 0, 0)));
        // LED 0: selector 0 bits 1:0 = 1 -> slot 1 -> palette 3.
        assert_eq!(p.led_rgb(0), Some((255, 0, 0)));
    }
}
