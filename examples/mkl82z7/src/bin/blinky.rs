//! Blinks the red LED (PTC1, active low) on the FRDM-KL82Z and logs each
//! toggle over RTT.
#![no_std]
#![no_main]

use cortex_m::asm;
use cortex_m_rt::entry;
use embassy_nxp_mkl82z7_examples as _;
use nxp_pac::port::vals::Mux;
use nxp_pac::{GPIOC, PORTC, SIM};

const LED_PIN: usize = 1;

#[entry]
fn main() -> ! {
    embassy_nxp_mkl82z7_examples::init();

    defmt::info!("blinky: FRDM-KL82Z, SDID = {:#010x}", SIM.sdid().read().0);

    // Clock the PORTC pin control registers.
    SIM.scgc5().modify(|w| w.set_ptc(true));

    // PTC1 as GPIO (ALT1), output, initially high (LED off).
    PORTC.pcr(LED_PIN).write(|w| w.set_mux(Mux::Mux1));
    GPIOC.psor().write(|w| w.set_ptso(LED_PIN, true));
    GPIOC.pddr().modify(|w| w.set_pdd(LED_PIN, true));

    loop {
        GPIOC.ptor().write(|w| w.set_ptto(LED_PIN, true));
        let level = GPIOC.pdir().read().pdi(LED_PIN);
        defmt::info!("PTC1 = {}, LED {}", level as u8, if level { "off" } else { "on" });

        // Reset clock configuration is FEI mode at ~21 MHz core clock.
        for _ in 0..500_000 {
            asm::nop();
        }
    }
}
