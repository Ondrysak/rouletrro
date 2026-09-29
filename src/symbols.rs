//! Guest addresses in Digitakt OS 1.53's MAIN OS.
//!
//! Every value here was resolved from the image by the reference project's
//! signature rules (docs/mk1/06-symbols.md there) and is only valid for the
//! image with `MAIN_OS_SHA256`. `Profile::for_image` refuses anything else,
//! so a different build fails loudly instead of hooking the wrong code.

pub const MAIN_OS_SHA256: &str = "4b47a9507758ca5669ca02ab2c0374d2c04c98aece445408295cc1dcb265c5df";
pub const MAIN_LOAD: u32 = 0x4000_0400;

#[derive(Clone, Debug)]
pub struct Profile {
    pub entry: u32,
    pub flash_read: u32,
    pub ctx_switch: u32,
    pub current_tcb: u32,
    pub prio_heads: u32,
    pub task_create: u32,
    pub sem_pend: u32,
    pub pend_b: u32,
    pub queue_send: u32,
    pub queue_recv: u32,
    pub mainloop: u32,
    pub panel_diff: u32,
    pub fb_front: u32,
    pub fb_back: u32,
    pub intro_pit3_isr: u32,
    pub intro_done: u32,
    pub intro_park: u32,
    pub frame_sem: u32,
    pub display_sem: u32,
    pub completion_sem: u32,
    pub pend_call: u32,
    pub depack_copy: u32,
    pub sd_flag: u32,
    pub sd_status: u32,
    pub sd_cmd_sem: u32,
    pub sd_data_sem: u32,
    pub sd_dma_sem: u32,
    pub sd_capacity: u32,
    pub uart8_ring_ptr: u32,
    pub uart8_consume_idx: u32,
    pub transport_state: u32,
    pub panel_button_names: u32,
    pub mounted: u32,
    pub first_boot_error: u32,
    pub progress_done: u32,
    pub progress_total: u32,
    pub abort_loop: u32,
    /// Every `bra.b *` in the image: the RTOS idle points.
    pub idle_spins: Vec<u32>,
}

impl Profile {
    pub fn for_image(img: &[u8]) -> Result<Profile, String> {
        use sha2::{Digest, Sha256};
        let h: String = Sha256::digest(img).iter().map(|b| format!("{b:02x}")).collect();
        if h != MAIN_OS_SHA256 {
            return Err(format!(
                "MAIN OS sha256 {h} is not OS 1.53's; this emulator's symbol table is for {MAIN_OS_SHA256}"
            ));
        }
        let idle_spins = (0..img.len() - 1)
            .step_by(2)
            .filter(|&o| img[o] == 0x60 && img[o + 1] == 0xFE)
            .map(|o| MAIN_LOAD + o as u32)
            .collect();
        Ok(Profile {
            entry: 0x4000_04E8,
            flash_read: 0x400E_94F2,
            ctx_switch: 0x4000_0410,
            current_tcb: 0x4399_D798,
            prio_heads: 0x4399_D758,
            task_create: 0x4000_15AC,
            sem_pend: 0x4000_16FE,
            pend_b: 0x4000_168A,
            queue_send: 0x4000_1B7A,
            queue_recv: 0x4000_1C2A,
            mainloop: 0x4000_B6E4,
            panel_diff: 0x400E_60E2,
            fb_front: 0x4020_D8F8,
            fb_back: 0x4020_D8FC,
            intro_pit3_isr: 0x4006_C154,
            intro_done: 0x4006_CB92,
            intro_park: 0x4006_CBC0,
            frame_sem: 0x4198_8BE4,
            display_sem: 0x421C_D06C,
            completion_sem: 0x421E_D5C8,
            pend_call: 0x400E_8AFC,
            depack_copy: 0x400E_AAAC,
            sd_flag: 0x421C_CA10,
            sd_status: 0x421C_CA40,
            sd_cmd_sem: 0x421C_CA5C,
            sd_data_sem: 0x421C_CA54,
            sd_dma_sem: 0x421C_CA4C,
            sd_capacity: 0x421C_CA34,
            uart8_ring_ptr: 0x4060_E820,
            uart8_consume_idx: 0x4060_E840,
            transport_state: 0x4199_DC2C,
            panel_button_names: 0x4018_E254,
            mounted: 0x420E_DC50,
            first_boot_error: 0x4064_80E0,
            progress_done: 0x421C_D4F4,
            progress_total: 0x421C_D4F0,
            abort_loop: 0x400E_E3F2,
            idle_spins,
        })
    }
}
