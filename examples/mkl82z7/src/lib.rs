//! Shared bits for the MKL82Z7 (FRDM-KL82Z) examples.
#![no_std]

use defmt_rtt as _;
use nxp_pac::WDOG;
use panic_probe as _;

/// Kinetis flash configuration field, placed at 0x400 by `memory.x`.
///
/// Bytes 0..8: backdoor comparison key (unused).
/// Bytes 8..12: FPROT3..FPROT0, 0xFF = no flash protection.
/// Byte 12: FSEC = 0xFE, security disabled, mass erase enabled.
/// Byte 13: FOPT = 0x3D, boot from flash (BOOTSRC_SEL = 0), NMI disabled,
///          RESET pin enabled, fast init, LPBOOT = full speed.
///          The reset value 0xFF would select the boot ROM instead of flash.
/// Bytes 14..16: reserved.
#[used]
#[unsafe(link_section = ".flash_config")]
pub static FLASH_CONFIG: [u8; 16] = [
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, // backdoor key
    0xFF, 0xFF, 0xFF, 0xFF, // FPROT3..FPROT0
    0xFE, // FSEC
    0x3D, // FOPT
    0xFF, 0xFF,
];

/// Board bring-up that every example needs. Call it first thing in `main`.
///
/// The KL82 watchdog is enabled out of reset and runs from the bus clock with
/// a timeout of about half a second, so it has to be disabled (or serviced)
/// early or the chip silently resets. The unlock sequence must complete within
/// 20 bus cycles and the configuration write within 256 bus cycles after it.
pub fn init() {
    // Make sure the flash configuration field is never optimised away even
    // when nothing else references this crate.
    core::hint::black_box(&FLASH_CONFIG);

    cortex_m::interrupt::free(|_| {
        WDOG.unlock().write(|w| w.set_wdogunlock(0xC520));
        WDOG.unlock().write(|w| w.set_wdogunlock(0xD928));
        WDOG.stctrlh().modify(|w| w.set_wdogen(false));
    });
}

/// Terminates the `probe-rs run` session with a success exit code.
///
/// Only meaningful with a debugger attached; without one the semihosting
/// breakpoint escalates to a HardFault.
pub fn exit() -> ! {
    loop {
        cortex_m_semihosting::debug::exit(cortex_m_semihosting::debug::EXIT_SUCCESS);
    }
}
