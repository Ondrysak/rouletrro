//! The signatures find, in OS 1.53, exactly the addresses worked out on it
//! by hand. Needs the firmware; skipped without it.

use dtemu::firmware::Firmware;
use dtemu::symbols::Profile;

#[test]
fn os_153_symbols() {
    let path = std::path::Path::new("fw/Digitakt_OS1.53.syx");
    if !path.exists() {
        eprintln!("skipped: no {}", path.display());
        return;
    }
    let fw = Firmware::from_file(path).unwrap();
    let p = Profile::for_image(&fw.main_os().unwrap().data).unwrap();
    assert!(p.is_os_153());
    assert_eq!(p.entry, 0x4000_04E8);
    assert_eq!(p.flash_read, 0x400E_94F2);
    assert_eq!(p.task_create, Some(0x4000_15AC));
    assert_eq!(p.intro_done, Some(0x4006_CB92));
    assert_eq!(p.panel_diff, Some(0x400E_60E2));
    assert_eq!(p.fb_front, Some(0x4020_D8F8));
    assert_eq!(p.abort_loop, Some(0x400E_E3F2));
    assert_eq!(p.mainloop, Some(0x4000_B6E4));
    assert_eq!(p.current_tcb, Some(0x4399_D798));
    assert_eq!(p.mounted, Some(0x420E_DC50));
    assert!(p.missing.is_empty());
    assert!(p.idle_spins.contains(&0x400E_E3F2));
}
