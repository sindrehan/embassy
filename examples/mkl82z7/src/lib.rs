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
/// Byte 13: FOPT = 0x3B. BOOTSRC_SEL selects internal flash, NMI is disabled,
///          initialization is fast, and LPBOOT selects RUN after reset.
/// Bytes 14..16: reserved.
#[used]
#[unsafe(no_mangle)]
#[unsafe(link_section = ".flash_config")]
pub static FLASH_CONFIG: [u8; 16] = [
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, // backdoor key
    0xFF, 0xFF, 0xFF, 0xFF, // FPROT3..FPROT0
    0xFE, // FSEC
    0x3B, // FOPT
    0xFF, 0xFF,
];
