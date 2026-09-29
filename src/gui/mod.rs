//! The panel window's drawing: a software canvas, the Digitakt's controls
//! laid out on it, and hit-testing for the mouse.

pub mod font;

use crate::panel;

pub const W: usize = 1180;
pub const H: usize = 600;

pub const BG: u32 = 0x1E1F22;
const PANEL: u32 = 0x2A2C30;
const KEY: u32 = 0x3A3D42;
const KEY_EDGE: u32 = 0x55595F;
const TEXT: u32 = 0xD8DADF;
const DIM: u32 = 0x8A8F98;
const OLED_ON: u32 = 0xE8F4FF;
const OLED_OFF: u32 = 0x05070A;

pub struct Canvas {
    pub px: Vec<u32>,
    pub w: usize,
    pub h: usize,
}

impl Canvas {
    pub fn new(w: usize, h: usize) -> Canvas {
        Canvas { px: vec![BG; w * h], w, h }
    }

    #[inline]
    pub fn put(&mut self, x: i32, y: i32, c: u32) {
        if x >= 0 && y >= 0 && (x as usize) < self.w && (y as usize) < self.h {
            self.px[y as usize * self.w + x as usize] = c;
        }
    }

    pub fn fill(&mut self, x: i32, y: i32, w: i32, h: i32, c: u32) {
        for yy in y.max(0)..(y + h).min(self.h as i32) {
            for xx in x.max(0)..(x + w).min(self.w as i32) {
                self.px[yy as usize * self.w + xx as usize] = c;
            }
        }
    }

    pub fn frame(&mut self, x: i32, y: i32, w: i32, h: i32, c: u32) {
        self.fill(x, y, w, 1, c);
        self.fill(x, y + h - 1, w, 1, c);
        self.fill(x, y, 1, h, c);
        self.fill(x + w - 1, y, 1, h, c);
    }

    pub fn disc(&mut self, cx: i32, cy: i32, r: i32, c: u32) {
        for dy in -r..=r {
            for dx in -r..=r {
                if dx * dx + dy * dy <= r * r {
                    self.put(cx + dx, cy + dy, c);
                }
            }
        }
    }

    pub fn ring(&mut self, cx: i32, cy: i32, r: i32, c: u32) {
        for dy in -r..=r {
            for dx in -r..=r {
                let d = dx * dx + dy * dy;
                if d <= r * r && d >= (r - 2) * (r - 2) {
                    self.put(cx + dx, cy + dy, c);
                }
            }
        }
    }

    /// Text in the 5x7 font at `scale`; -> the width drawn.
    pub fn text(&mut self, x: i32, y: i32, s: &str, scale: i32, c: u32) -> i32 {
        let mut cx = x;
        for ch in s.chars() {
            let g = font::glyph(ch);
            for (col, bits) in g.iter().enumerate() {
                for row in 0..7 {
                    if bits >> row & 1 != 0 {
                        self.fill(cx + col as i32 * scale, y + row * scale, scale, scale, c);
                    }
                }
            }
            cx += 6 * scale;
        }
        cx - x
    }

    pub fn text_centered(&mut self, cx: i32, y: i32, s: &str, scale: i32, c: u32) {
        let w = s.chars().count() as i32 * 6 * scale - scale;
        self.text(cx - w / 2, y, s, scale, c);
    }
}

#[derive(Clone, Copy)]
pub enum Control {
    Key(u8),
    /// Encoder index for turns (0..7 = A..H, 8 = LEVEL/DATA), and the key
    /// code of its push switch, if it has one.
    Knob(u8, Option<u8>),
}

#[derive(Clone, Copy)]
pub struct Place {
    pub c: Control,
    pub x: i32,
    pub y: i32,
    pub w: i32,
    pub h: i32,
}

impl Place {
    pub fn hit(&self, x: i32, y: i32) -> bool {
        x >= self.x && x < self.x + self.w && y >= self.y && y < self.y + self.h
    }
}

pub const OLED_X: i32 = 170;
pub const OLED_Y: i32 = 44;
pub const OLED_SCALE: i32 = 4;

/// Where every control sits.
pub fn layout() -> Vec<Place> {
    let mut v = Vec::new();
    let key = |code: u8, x: i32, y: i32, w: i32| Place { c: Control::Key(code), x, y, w, h: 40 };
    // The LEVEL/DATA knob, left of the screen.
    v.push(Place { c: Control::Knob(8, None), x: 40, y: 70, w: 90, h: 90 });
    // Knobs A..H, two rows right of the screen.
    for i in 0..8u8 {
        let col = (i % 4) as i32;
        let row = (i / 4) as i32;
        v.push(Place { c: Control::Knob(i, Some(40 + i)), x: 720 + col * 110, y: 50 + row * 130, w: 80, h: 80 });
    }
    // Under the LEVEL knob.
    v.push(key(2, 20, 190, 64)); // TRK
    v.push(key(3, 90, 190, 64)); // PTN
    v.push(key(4, 20, 240, 64)); // BANK
    v.push(key(5, 90, 240, 64)); // SONG
    // Menu keys and the cursor cluster, below the screen.
    let y1 = 330;
    let menus = [6u8, 7, 8, 18]; // GLOBAL SAMPLE TEMPO PAGE
    for (i, &c) in menus.iter().enumerate() {
        v.push(key(c, 164 + i as i32 * 88, y1, 82));
    }
    v.push(key(13, 540, y1, 64)); // NO
    v.push(key(12, 610, y1, 64)); // YES
    // Arrows: UP above DOWN, LEFT and RIGHT either side.
    v.push(key(14, 610, y1 + 50, 64)); // UP
    v.push(key(16, 540, y1 + 100, 64)); // LEFT
    v.push(key(15, 610, y1 + 100, 64)); // DOWN
    v.push(key(17, 680, y1 + 100, 64)); // RIGHT
    // Page keys, right of the cursor cluster.
    let pages = [19u8, 20, 21, 22, 23]; // TRIG SRC FLTR AMP LFO
    for (i, &c) in pages.iter().enumerate() {
        v.push(key(c, 770 + i as i32 * 80, y1 + 50, 74));
    }
    // Bottom row: FUNC, transport, the sixteen trigs.
    let y2 = 520;
    v.push(key(1, 20, y2, 76)); // FUNC
    v.push(key(9, 102, y2, 80)); // RECORD
    v.push(key(10, 188, y2, 76)); // PLAY
    v.push(key(11, 270, y2, 76)); // STOP
    for i in 0..16u8 {
        v.push(key(24 + i, 360 + i as i32 * 50, y2, 45));
    }
    v
}

/// What the window shows, published by the emulator thread.
#[derive(Clone)]
pub struct View {
    pub oled: Vec<u8>,
    pub leds: Vec<Option<(u8, u8, u8)>>,
    pub status: String,
}

/// Light added to a key face by its LED.
fn glow(base: u32, (r, g, b): (u8, u8, u8)) -> u32 {
    let add = |x: u32, y: u8| (x + (y as u32 * 3 / 4)).min(255);
    add((base >> 16) & 0xFF, r) << 16 | add((base >> 8) & 0xFF, g) << 8 | add(base & 0xFF, b)
}

fn blend(base: u32, (r, g, b): (u8, u8, u8), a: u32) -> u32 {
    let br = (base >> 16) & 0xFF;
    let bg = (base >> 8) & 0xFF;
    let bb = base & 0xFF;
    let mix = |x: u32, y: u32| (x * (256 - a) + y * a) >> 8;
    mix(br, r as u32) << 16 | mix(bg, g as u32) << 8 | mix(bb, b as u32)
}

pub fn draw(cv: &mut Canvas, view: &View, places: &[Place], held: &[u8], latched: &[u8]) {
    cv.fill(0, 0, cv.w as i32, cv.h as i32, BG);
    cv.fill(8, 30, cv.w as i32 - 16, cv.h as i32 - 38, PANEL);
    cv.text(12, 8, "DIGITAKT", 2, TEXT);
    cv.text(130, 12, &view.status, 1, DIM);
    // The OLED.
    cv.fill(OLED_X - 6, OLED_Y - 6, 128 * OLED_SCALE + 12, 64 * OLED_SCALE + 12, 0x000000);
    for y in 0..64usize {
        for x in 0..128usize {
            let b = view.oled[(7 - y / 8) + 8 * x];
            let on = (b >> (y % 8)) & 1 != 0;
            cv.fill(OLED_X + x as i32 * OLED_SCALE, OLED_Y + y as i32 * OLED_SCALE, OLED_SCALE, OLED_SCALE, if on { OLED_ON } else { OLED_OFF });
        }
    }
    // The four pattern-page LEDs, beside PAGE.
    for (i, &led) in panel::PAGE_LEDS.iter().enumerate() {
        let c = view.leds.get(led).copied().flatten().map(|rgb| blend(0x202020, rgb, 256)).unwrap_or(0x202020);
        cv.fill(438 + i as i32 * 16, 376, 12, 6, c);
    }
    for p in places {
        match p.c {
            Control::Key(code) => {
                let mut face = KEY;
                if let Some(led) = panel::led_for_key(code) {
                    if let Some(rgb) = view.leds.get(led).copied().flatten() {
                        face = glow(KEY, rgb);
                    }
                }
                let down = held.contains(&code);
                cv.fill(p.x, p.y, p.w, p.h, if down { blend(face, (255, 255, 255), 60) } else { face });
                cv.frame(p.x, p.y, p.w, p.h, if latched.contains(&code) { 0xFFD050 } else { KEY_EDGE });
                let label = panel::key_name(code).unwrap_or("?");
                let lum = ((face >> 16) & 0xFF) * 3 + ((face >> 8) & 0xFF) * 6 + (face & 0xFF);
                let ink = if lum > 1600 { 0x202226 } else { TEXT };
                let scale = if label.len() as i32 * 12 - 2 <= p.w - 6 { 2 } else { 1 };
                cv.text_centered(p.x + p.w / 2, p.y + p.h / 2 - 7 * scale / 2, label, scale, ink);
            }
            Control::Knob(enc, push) => {
                let (cx, cy) = (p.x + p.w / 2, p.y + p.h / 2 - 8);
                let r = p.w.min(p.h) / 2 - 10;
                let pressed = push.is_some_and(|c| held.contains(&c));
                cv.disc(cx, cy, r, if pressed { 0x70757D } else { 0x4A4E55 });
                cv.ring(cx, cy, r, 0x8C9199);
                cv.fill(cx - 1, cy - r + 3, 3, r / 2, 0xE0E0E0);
                let name = if enc == 8 { "LEVEL/DATA".to_string() } else { ((b'A' + enc) as char).to_string() };
                cv.text_centered(cx, p.y + p.h - 12, &name, if enc == 8 { 1 } else { 2 }, TEXT);
            }
        }
    }
    cv.text(12, cv.h as i32 - 18, "CLICK: PRESS   SHIFT-CLICK: LATCH   ESC: RELEASE ALL   WHEEL OR DRAG ON A KNOB: TURN   CLICK A KNOB: PUSH", 1, DIM);
}

/// Write the canvas as a PNG.
pub fn save_png(cv: &Canvas, path: &std::path::Path) -> std::io::Result<()> {
    let f = std::fs::File::create(path)?;
    let mut enc = png::Encoder::new(std::io::BufWriter::new(f), cv.w as u32, cv.h as u32);
    enc.set_color(png::ColorType::Rgb);
    enc.set_depth(png::BitDepth::Eight);
    let mut w = enc.write_header().map_err(std::io::Error::other)?;
    let mut data = Vec::with_capacity(cv.w * cv.h * 3);
    for &p in &cv.px {
        data.extend_from_slice(&[(p >> 16) as u8, (p >> 8) as u8, p as u8]);
    }
    w.write_image_data(&data).map_err(std::io::Error::other)
}
