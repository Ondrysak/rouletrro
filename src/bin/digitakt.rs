//! The Digitakt, in a window, with sound.
//!
//!     digitakt [FIRMWARE.syx] [--snapshot FILE] [--ips N] [--no-audio]
//!              [--headless --after SECS --screenshot OUT.png] [--press CODE@SECS]
//!
//! With `--snapshot`, the window resumes from FILE when it exists (F5 saves
//! it, and it is saved again on exit); otherwise the firmware boots from
//! reset, intro and all, and a blank +Drive is formatted on first boot.
//!
//! The emulator runs on its own thread. With audio it is paced by the sound
//! card: it renders ahead until about 60 ms are buffered and then waits, so
//! emulated time follows the card's clock. Without audio it follows the wall
//! clock.

use dtemu::firmware::Firmware;
use dtemu::gui::{self, Canvas, Control, View};
use dtemu::machine::Machine;
use std::collections::VecDeque;
use std::path::PathBuf;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::mpsc;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

enum Ev {
    Key(u8, bool),
    Turn(u8, i8),
    ReleaseAll,
    Save,
}

struct Audio {
    ring: Mutex<VecDeque<f32>>,
    dropouts: AtomicU64,
    rate: AtomicU64,
    live: AtomicBool,
}

struct Opts {
    syx: PathBuf,
    snapshot: Option<PathBuf>,
    ips: f64,
    audio: bool,
    headless: bool,
    after: f64,
    screenshot: Option<PathBuf>,
    presses: Vec<(u8, f64)>,
}

fn parse() -> Opts {
    let mut o = Opts {
        syx: PathBuf::from("fw/Digitakt_OS1.53.syx"),
        snapshot: None,
        ips: 250e6,
        audio: true,
        headless: false,
        after: 5.0,
        screenshot: None,
        presses: Vec::new(),
    };
    let args: Vec<String> = std::env::args().skip(1).collect();
    let mut it = args.iter();
    while let Some(a) = it.next() {
        match a.as_str() {
            "--snapshot" => o.snapshot = it.next().map(PathBuf::from),
            "--ips" => o.ips = it.next().unwrap().replace('M', "e6").parse().expect("--ips"),
            "--no-audio" => o.audio = false,
            "--headless" => o.headless = true,
            "--after" => o.after = it.next().unwrap().parse().expect("--after"),
            "--screenshot" => o.screenshot = it.next().map(PathBuf::from),
            "--press" => {
                let v = it.next().unwrap();
                let (c, t) = v.split_once('@').expect("--press CODE@SECS");
                o.presses.push((c.parse().unwrap(), t.parse().unwrap()));
            }
            "-h" | "--help" => {
                println!("usage: digitakt [FIRMWARE.syx] [--snapshot FILE] [--ips N] [--no-audio] [--headless --after SECS --screenshot OUT.png] [--press CODE@SECS]");
                std::process::exit(0);
            }
            _ => o.syx = PathBuf::from(a),
        }
    }
    o
}

fn view_of(m: &Machine, status: String) -> View {
    let p = &m.cpu.bus.io.panel;
    View {
        oled: p.oled.to_vec(),
        leds: (0..dtemu::panel::LEDS).map(|i| p.led_rgb(i)).collect(),
        status,
    }
}

/// The emulator loop. Runs until `quit`.
fn emulate(
    mut m: Machine,
    o: &Opts,
    rx: mpsc::Receiver<Ev>,
    view: Arc<Mutex<View>>,
    audio: Arc<Audio>,
    quit: Arc<AtomicBool>,
) -> Machine {
    let ips = o.ips;
    m.cpu.bus.io.ips = ips;
    let slice = (ips / 500.0) as u64; // 2 ms of emulated time
    let target_ms = 60.0;
    let t0 = Instant::now();
    let emu0 = m.now();
    let mut last_pub = Instant::now();
    let mut last_speed = (Instant::now(), m.now());
    let mut speed = 0.0;
    let mut phase = 0.0f64;
    while !quit.load(Ordering::Relaxed) {
        while let Ok(ev) = rx.try_recv() {
            match ev {
                Ev::Key(c, d) => m.key(c, d),
                Ev::Turn(e, d) => m.turn(e, d),
                Ev::ReleaseAll => m.release_all_keys(),
                Ev::Save => {
                    if let Some(p) = &o.snapshot {
                        match dtemu::snapshot::save(&m, p) {
                            Ok(()) => eprintln!("saved {}", p.display()),
                            Err(e) => eprintln!("save failed: {e}"),
                        }
                    }
                }
            }
        }
        let rate = audio.rate.load(Ordering::Relaxed) as f64;
        let live = audio.live.load(Ordering::Relaxed);
        if live {
            let buffered = audio.ring.lock().unwrap().len() as f64 / 2.0 / rate * 1000.0;
            if buffered > target_ms {
                std::thread::sleep(Duration::from_micros(500));
                continue;
            }
        } else {
            let emu_t = (m.now() - emu0) as f64 / ips;
            if emu_t > t0.elapsed().as_secs_f64() + 0.005 {
                std::thread::sleep(Duration::from_micros(500));
                continue;
            }
        }
        m.run_until(m.now() + slice);
        // Audio out: 24-bit samples, stereo, at the SSI's 48 kHz.
        let out = &mut m.cpu.bus.io.ssi.out;
        if live && out.len() >= 2 {
            let mut ring = audio.ring.lock().unwrap();
            let step = 48_000.0 / rate;
            let frames: Vec<(f32, f32)> = out
                .drain(..out.len() & !1)
                .collect::<Vec<i32>>()
                .chunks(2)
                .map(|p| (p[0] as f32 / 8_388_608.0, p[1] as f32 / 8_388_608.0))
                .collect();
            // Resample by stepping through the source at the rate ratio.
            while phase < frames.len() as f64 {
                let (l, r) = frames[phase as usize];
                ring.push_back(l);
                ring.push_back(r);
                phase += step;
            }
            phase -= frames.len() as f64;
        } else if !live {
            out.clear();
        }
        if last_pub.elapsed() > Duration::from_millis(15) {
            last_pub = Instant::now();
            let (t, n) = last_speed;
            if t.elapsed() > Duration::from_millis(500) {
                speed = (m.now() - n) as f64 / ips / t.elapsed().as_secs_f64();
                last_speed = (Instant::now(), m.now());
            }
            let audio_s = if live {
                let ms = audio.ring.lock().unwrap().len() as f64 / 2.0 / rate * 1000.0;
                format!("AUDIO LIVE {:.0} KHZ  {:3.0} MS BUFFERED  {} DROPOUTS", rate / 1000.0, ms, audio.dropouts.load(Ordering::Relaxed))
            } else {
                "AUDIO OFF".to_string()
            };
            let status = format!("OS 1.53   {audio_s}   SPEED {:3.0}%   T {:.1} S", speed * 100.0, m.now() as f64 / ips);
            *view.lock().unwrap() = view_of(&m, status);
        }
    }
    m
}

fn start_audio(audio: Arc<Audio>) -> Option<cpal::Stream> {
    use cpal::traits::{DeviceTrait, HostTrait, StreamTrait};
    let host = cpal::default_host();
    let dev = host.default_output_device()?;
    let supported = dev.supported_output_configs().ok()?.collect::<Vec<_>>();
    // Prefer 48 kHz stereo f32; take the device default otherwise.
    let cfg = supported
        .iter()
        .find(|c| c.channels() == 2 && c.min_sample_rate().0 <= 48_000 && c.max_sample_rate().0 >= 48_000 && c.sample_format() == cpal::SampleFormat::F32)
        .map(|c| c.clone().with_sample_rate(cpal::SampleRate(48_000)))
        .or_else(|| dev.default_output_config().ok())?;
    if cfg.sample_format() != cpal::SampleFormat::F32 {
        eprintln!("audio: the device's format is {:?}, not f32; running without sound", cfg.sample_format());
        return None;
    }
    let channels = cfg.channels() as usize;
    audio.rate.store(cfg.sample_rate().0 as u64, Ordering::Relaxed);
    let a = audio.clone();
    let stream = dev
        .build_output_stream(
            &cfg.config(),
            move |data: &mut [f32], _| {
                let mut ring = a.ring.lock().unwrap();
                let frames = data.len() / channels;
                if ring.len() < frames * 2 {
                    a.dropouts.fetch_add(1, Ordering::Relaxed);
                }
                for f in data.chunks_mut(channels) {
                    let l = ring.pop_front().unwrap_or(0.0);
                    let r = ring.pop_front().unwrap_or(0.0);
                    for (i, s) in f.iter_mut().enumerate() {
                        *s = match (channels, i) {
                            (1, _) => (l + r) * 0.5,
                            (_, 0) => l,
                            (_, 1) => r,
                            _ => 0.0,
                        };
                    }
                }
            },
            |e| eprintln!("audio: {e}"),
            None,
        )
        .ok()?;
    stream.play().ok()?;
    audio.live.store(true, Ordering::Relaxed);
    Some(stream)
}

fn main() {
    let o = parse();
    let fw = Firmware::from_file(&o.syx).unwrap_or_else(|e| {
        eprintln!("{}: {e}", o.syx.display());
        std::process::exit(1)
    });
    let mut m = Machine::new(&fw, 128).unwrap_or_else(|e| {
        eprintln!("{e}");
        std::process::exit(1)
    });
    if let Some(p) = &o.snapshot {
        if p.exists() {
            match dtemu::snapshot::load(&mut m, p) {
                Ok(()) => eprintln!("resumed {}", p.display()),
                Err(e) => eprintln!("{}: {e}; booting from reset", p.display()),
            }
        }
    }
    let view = Arc::new(Mutex::new(view_of(&m, String::from("STARTING"))));
    let audio = Arc::new(Audio {
        ring: Mutex::new(VecDeque::new()),
        dropouts: AtomicU64::new(0),
        rate: AtomicU64::new(48_000),
        live: AtomicBool::new(false),
    });
    let _stream = if o.audio && !o.headless { start_audio(audio.clone()) } else { None };
    if o.audio && !o.headless && _stream.is_none() {
        eprintln!("audio: no output device; running without sound");
    }
    let quit = Arc::new(AtomicBool::new(false));
    let (tx, rx) = mpsc::channel();
    let opts = Arc::new(o);
    let th = {
        let (view, audio, quit, opts) = (view.clone(), audio.clone(), quit.clone(), opts.clone());
        std::thread::spawn(move || emulate(m, &opts, rx, view, audio, quit))
    };
    let places = gui::layout();
    let mut cv = Canvas::new(gui::W, gui::H);
    if opts.headless {
        let t0 = Instant::now();
        let mut presses = opts.presses.clone();
        presses.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap());
        let mut pending: Vec<(u8, f64)> = Vec::new();
        while t0.elapsed().as_secs_f64() < opts.after {
            let t = t0.elapsed().as_secs_f64();
            while let Some(&(c, at)) = presses.first() {
                if at > t {
                    break;
                }
                presses.remove(0);
                let _ = tx.send(Ev::Key(c, true));
                pending.push((c, at + 0.15));
            }
            pending.retain(|&(c, up)| {
                if up <= t {
                    let _ = tx.send(Ev::Key(c, false));
                    false
                } else {
                    true
                }
            });
            std::thread::sleep(Duration::from_millis(10));
        }
        quit.store(true, Ordering::Relaxed);
        let m = th.join().unwrap();
        let v = view_of(&m, format!("HEADLESS, T {:.1} S", m.now() as f64 / opts.ips));
        gui::draw(&mut cv, &v, &places, &[], &[]);
        if let Some(p) = &opts.screenshot {
            gui::save_png(&cv, p).unwrap();
            eprintln!("wrote {}", p.display());
        }
        return;
    }
    run_window(&opts, tx, view, quit.clone(), &places, &mut cv);
    quit.store(true, Ordering::Relaxed);
    let m = th.join().unwrap();
    if let Some(p) = &opts.snapshot {
        match dtemu::snapshot::save(&m, p) {
            Ok(()) => eprintln!("saved {}", p.display()),
            Err(e) => eprintln!("save failed: {e}"),
        }
    }
}

fn run_window(o: &Opts, tx: mpsc::Sender<Ev>, view: Arc<Mutex<View>>, _quit: Arc<AtomicBool>, places: &[gui::Place], cv: &mut Canvas) {
    use minifb::{Key, KeyRepeat, MouseButton, MouseMode, Window, WindowOptions};
    let mut win = match Window::new("Digitakt (OS 1.53) - dtemu", gui::W, gui::H, WindowOptions::default()) {
        Ok(w) => w,
        Err(e) => {
            eprintln!("cannot open a window: {e}");
            return;
        }
    };
    win.set_target_fps(60);
    let mut held: Vec<u8> = Vec::new(); // momentary, under the mouse
    let mut latched: Vec<u8> = Vec::new();
    let mut was_down = false;
    let mut drag: Option<(u8, f32)> = None;
    let mut shot_at = o.screenshot.as_ref().map(|_| Instant::now() + Duration::from_secs_f64(o.after));
    while win.is_open() {
        let (mx, my) = win.get_mouse_pos(MouseMode::Discard).unwrap_or((-1.0, -1.0));
        let down = win.get_mouse_down(MouseButton::Left);
        let shift = win.is_key_down(Key::LeftShift) || win.is_key_down(Key::RightShift);
        let hit = places.iter().find(|p| p.hit(mx as i32, my as i32)).copied();
        if down && !was_down {
            match hit.map(|p| p.c) {
                Some(Control::Key(c)) | Some(Control::Knob(_, Some(c))) if shift => {
                    if let Some(i) = latched.iter().position(|&k| k == c) {
                        latched.remove(i);
                        let _ = tx.send(Ev::Key(c, false));
                    } else {
                        latched.push(c);
                        let _ = tx.send(Ev::Key(c, true));
                    }
                }
                Some(Control::Key(c)) => {
                    held.push(c);
                    let _ = tx.send(Ev::Key(c, true));
                }
                Some(Control::Knob(e, push)) => {
                    drag = Some((e, my));
                    if let Some(c) = push {
                        held.push(c);
                        let _ = tx.send(Ev::Key(c, true));
                    }
                }
                None => {}
            }
        }
        if down {
            if let Some((e, y0)) = drag {
                let steps = ((y0 - my) / 6.0) as i32;
                if steps != 0 {
                    let _ = tx.send(Ev::Turn(e, steps.clamp(-127, 127) as i8));
                    drag = Some((e, y0 - steps as f32 * 6.0));
                    // A drag is a turn, not a push.
                    for c in held.drain(..) {
                        let _ = tx.send(Ev::Key(c, false));
                    }
                }
            }
        }
        if !down && was_down {
            for c in held.drain(..) {
                let _ = tx.send(Ev::Key(c, false));
            }
            drag = None;
        }
        was_down = down;
        if let Some((_, wy)) = win.get_scroll_wheel() {
            if let Some(Control::Knob(e, _)) = hit.map(|p| p.c) {
                let n = if wy > 0.0 { 4 } else { -4 };
                let _ = tx.send(Ev::Turn(e, n));
            }
        }
        for k in win.get_keys_pressed(KeyRepeat::No) {
            match k {
                Key::Escape => {
                    latched.clear();
                    let _ = tx.send(Ev::ReleaseAll);
                }
                Key::F5 => {
                    let _ = tx.send(Ev::Save);
                }
                _ => {}
            }
        }
        let v = view.lock().unwrap().clone();
        let mut shown = held.clone();
        shown.extend(latched.iter());
        gui::draw(cv, &v, places, &shown, &latched);
        if let Some(t) = shot_at {
            if Instant::now() >= t {
                if let Some(p) = &o.screenshot {
                    let _ = gui::save_png(cv, p);
                    eprintln!("wrote {}", p.display());
                }
                shot_at = None;
            }
        }
        if win.update_with_buffer(&cv.px, cv.w, cv.h).is_err() {
            break;
        }
    }
}
