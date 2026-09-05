//! Support for FRDM-KL82Z hardware tests.
#![no_std]

#[cfg(not(any(feature = "time-driver-tpm", feature = "time-driver-lptmr")))]
compile_error!("select time-driver-tpm or time-driver-lptmr");

use defmt_rtt as _;
use panic_probe as _;

pub mod spi_link;

/// Flash configuration: unprotected, unsecured, NMI disabled, boot from flash in RUN.
#[used]
#[unsafe(no_mangle)]
#[unsafe(link_section = ".flash_config")]
pub static FLASH_CONFIG: [u8; 16] = [
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0x3b, 0xff, 0xff,
];

/// Report successful completion after all assertions have run.
pub fn pass() -> ! {
    embassy_nxp::power::set_sleep_mode(embassy_nxp::power::SleepMode::Wait);
    defmt::info!("Test OK");
    hil_test_passed()
}

/// Stable completion breakpoint for hosts that cannot stream RTT through VLPS.
#[inline(never)]
#[unsafe(no_mangle)]
pub extern "C" fn hil_test_passed() -> ! {
    cortex_m::asm::bkpt();
    loop {
        cortex_m::asm::wfi();
    }
}
