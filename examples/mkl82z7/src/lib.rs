//! Shared bits for the MKL82Z7 (FRDM-KL82Z) examples.
#![no_std]

use defmt_rtt as _;
use panic_probe as _;

pub mod spi_link;

/// Kinetis flash configuration field, placed at 0x400 by `memory.x`, which also
/// forces this object into the link with `EXTERN(FLASH_CONFIG)`.
///
/// Bytes 0..8: backdoor comparison key (unused).
/// Bytes 8..12: FPROT3..FPROT0, 0xFF = no flash protection.
/// Byte 12: FSEC = 0xFE, security disabled, mass erase enabled.
/// Byte 13: FOPT = 0x3D. BOOTSRC_SEL selects internal flash, BOOTPIN_OPT lets
///          BOOTCFG0 request the ROM updater, NMI is enabled, initialization is
///          fast, and LPBOOT selects RUN after reset.
/// Bytes 14..16: reserved.
#[used]
#[unsafe(no_mangle)]
#[unsafe(link_section = ".flash_config")]
pub static FLASH_CONFIG: [u8; 16] = [
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, // backdoor key
    0xFF, 0xFF, 0xFF, 0xFF, // FPROT3..FPROT0
    0xFE, // FSEC
    0x3D, // FOPT
    0xFF, 0xFF,
];

/// Runs a fixed busy loop and returns how long it took in microseconds, as
/// measured by `embassy-time`. The time driver runs from the internal
/// reference clock, so the result scales with the core clock.
#[inline(never)]
pub fn spin_benchmark() -> u64 {
    let start = embassy_time::Instant::now();
    for _ in 0..200_000u32 {
        cortex_m::asm::nop();
    }
    start.elapsed().as_micros()
}

/// Terminates a semihosting debugger session with a success exit code.
///
/// Only meaningful with a debugger attached; without one the semihosting
/// breakpoint escalates to a HardFault.
pub fn exit() -> ! {
    loop {
        cortex_m_semihosting::debug::exit(cortex_m_semihosting::debug::EXIT_SUCCESS);
    }
}
