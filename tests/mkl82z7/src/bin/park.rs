//! Idle fixture with no driven GPIOs. Use before changing test wiring.
#![no_std]
#![no_main]

use embassy_nxp_mkl82z7_tests as _;

teleprobe_meta::target!(b"frdm-kl82z");

#[cortex_m_rt::entry]
fn main() -> ! {
    embassy_nxp::init(Default::default());
    loop {
        cortex_m::asm::wfi();
    }
}
