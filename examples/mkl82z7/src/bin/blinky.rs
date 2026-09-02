//! Blinks the red LED (PTC1, active low) on the FRDM-KL82Z and logs each
//! toggle over RTT.
#![no_std]
#![no_main]

use cortex_m::asm;
use cortex_m_rt::entry;
use embassy_nxp::gpio::{Level, Output};
use embassy_nxp::pac::SIM;
use embassy_nxp_mkl82z7_examples as _;

#[entry]
fn main() -> ! {
    let p = embassy_nxp::init(Default::default());

    defmt::info!("blinky: FRDM-KL82Z, SDID = {:#010x}", SIM.sdid().read().0);

    // Active low: start with the LED off.
    let mut led = Output::new(p.PTC1, Level::High);

    loop {
        led.toggle();
        let level = led.level();
        defmt::info!("PTC1 = {}, LED {}", level == Level::High, if level == Level::High { "off" } else { "on" });

        // Reset clock configuration is FEI mode at ~21 MHz core clock.
        for _ in 0..500_000 {
            asm::nop();
        }
    }
}
