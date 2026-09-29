//! Inspect a snapshot: registers, TCDs, memory.
//!
//!     dtpeek SNAPSHOT tcd N | mem ADDR LEN | leds | uart [FROM] | replay | fb

use dtemu::firmware::Firmware;
use dtemu::machine::Machine;

fn main() {
    let a: Vec<String> = std::env::args().skip(1).collect();
    if a.len() < 2 || a[0] == "-h" || a[0] == "--help" {
        println!("usage: dtpeek SNAPSHOT tcd N | mem ADDR LEN | leds | uart [FROM] | replay | fb");
        return;
    }
    let fw = Firmware::from_file(std::path::Path::new("fw/Digitakt_OS1.53.syx")).unwrap();
    let mut m = Machine::new(&fw, 128).unwrap();
    dtemu::snapshot::load(&mut m, std::path::Path::new(&a[0])).unwrap();
    match a[1].as_str() {
        "tcd" => {
            let n: usize = a[2].parse().unwrap();
            let t = m.cpu.bus.tcd(n);
            println!("TCD{n}: {t:x?}");
            println!("erq={} int={}", m.cpu.bus.io.edma.erq >> n & 1, m.cpu.bus.io.edma.int >> n & 1);
        }
        "mem" => {
            let addr = u32::from_str_radix(a[2].trim_start_matches("0x"), 16).unwrap();
            let len: u32 = a[3].parse().unwrap();
            for row in (0..len).step_by(16) {
                let bytes: Vec<String> = (0..16.min(len - row)).map(|i| format!("{:02x}", m.cpu.bus.peek8(addr + row + i))).collect();
                println!("{:08x}: {}", addr + row, bytes.join(" "));
            }
        }
        "leds" => {
            let p = &m.cpu.bus.io.panel;
            println!("selectors: {:02x?}", p.selectors);
            println!("palette: {:?}", p.palette.iter().enumerate().filter(|x| x.1.is_some()).map(|(i, c)| (i, c.unwrap())).collect::<Vec<_>>());
            for led in 0..44 {
                println!("led {led}: slots {:02x?} -> {:?}", p.slots[led], p.led_rgb(led));
            }
            println!("oled frames {} unknown {}", p.oled_frames, p.unknown);
        }
        "uart" => {
            let tx = &m.cpu.bus.io.uart8.tx;
            println!("{} bytes", tx.len());
            let from: usize = a.get(2).map(|x| x.parse().unwrap()).unwrap_or(0);
            for row in tx[from.min(tx.len())..].chunks(32).take(40) {
                println!("{}", row.iter().map(|b| format!("{b:02x}")).collect::<Vec<_>>().join(" "));
            }
        }
        "replay" => {
            let mut p = dtemu::panel::PanelMcu::default();
            let mut r = Vec::new();
            let tx = &m.cpu.bus.io.uart8.tx;
            let mut i = 0;
            let mut sels = 0;
            while i < tx.len() {
                let h = tx[i];
                let n = match h >> 4 { 0 => 1, 1 => 10, 0xB => match h { 0xB0..=0xB3 | 0xB6 => 3, 0xB4 => 6, 0xB5 => 5, 0xB7 => 2, 0xB8 => 1, _ => 2 }, _ => 2 };
                if h >> 4 == 2 && sels < 40 {
                    println!("@{i}: {:02x?}", &tx[i..(i + n).min(tx.len())]);
                    sels += 1;
                }
                i += n;
            }
            for &b in tx.iter() {
                p.from_host(b, &mut r);
            }
            println!("selectors {:02x?} unknown {} frames {}", p.selectors, p.unknown, p.oled_frames);
            println!("live selectors {:02x?}", m.cpu.bus.io.panel.selectors);
        }
        "fb" => {
            if let Some(f) = &m.frame {
                print!("{}", Machine::frame_ascii(f));
            }
        }
        _ => eprintln!("unknown command"),
    }
}
