//! Shared bits for the MKL82Z7 (FRDM-KL82Z) examples.
#![no_std]

use defmt_rtt as _;
use panic_probe as _;

/// Kinetis flash configuration field, placed at 0x400 by `memory.x`, which also
/// forces this object into the link with `EXTERN(FLASH_CONFIG)`.
///
/// Bytes 0..8: backdoor comparison key (unused).
/// Bytes 8..12: FPROT3..FPROT0, 0xFF = no flash protection.
/// Byte 12: FSEC = 0xFE, security disabled, mass erase enabled.
/// Byte 13: FOPT = 0x3D, boot from flash (BOOTSRC_SEL = 0), NMI disabled,
///          RESET pin enabled, fast init, LPBOOT = full speed.
///          The reset value 0xFF would select the boot ROM instead of flash.
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

/// Terminates the `probe-rs run` session with a success exit code.
///
/// Only meaningful with a debugger attached; without one the semihosting
/// breakpoint escalates to a HardFault.
pub fn exit() -> ! {
    loop {
        cortex_m_semihosting::debug::exit(cortex_m_semihosting::debug::EXIT_SUCCESS);
    }
}
