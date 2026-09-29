//! The front-panel microcontroller at the far end of UART8.
//!
//! A separate MCU scans the keys and encoders and drives the OLED and LEDs;
//! the ColdFire talks to it over UART8. What it answers here is what the
//! firmware asks for:
//!
//!   `60 01`        report every button group: `[0x20 | group, mask]` x 6
//!   `70 00`/`71 00` the UI card: `70` then type 4 (a Digitakt), UI firmware
//!                  1.2.0, a nonzero "tested" flag
//!   `74 00`        the serial number, `70` then 9 bytes
//!
//! Everything else the ColdFire sends is display and LED traffic, kept in
//! `out` for the display decoder.

use std::collections::VecDeque;

pub const GROUPS: u8 = 6;
pub const CARD: [u8; 4] = [0x04, 0x02, 0x00, 0x01];
pub const SERIAL: &[u8; 12] = b"EMULATED0000";

#[derive(Default, Clone)]
pub struct PanelMcu {
    last: [u8; 2],
    /// Keys held, per group.
    pub held: [u8; 8],
    /// Everything the ColdFire sent, for the display/LED decoder.
    pub out: VecDeque<u8>,
    pub queries: u64,
}

impl PanelMcu {
    /// A byte from the ColdFire. -> bytes to send back.
    pub fn from_host(&mut self, b: u8, reply: &mut Vec<u8>) {
        self.last = [self.last[1], b];
        if self.out.len() < (1 << 20) {
            self.out.push_back(b);
        }
        match self.last {
            [0x60, 0x01] => {
                self.queries += 1;
                for g in 0..GROUPS {
                    reply.push(0x20 | g);
                    reply.push(self.held[g as usize]);
                }
            }
            [0x70, 0x00] | [0x71, 0x00] => {
                self.queries += 1;
                reply.push(0x70);
                reply.extend_from_slice(&CARD);
            }
            [0x74, 0x00] => {
                self.queries += 1;
                reply.push(0x70);
                reply.extend_from_slice(&SERIAL[..9]);
            }
            _ => {}
        }
    }
}
