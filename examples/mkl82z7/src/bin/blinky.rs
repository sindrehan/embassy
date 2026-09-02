//! Blinks the red LED (PTC1, active low) on the FRDM-KL82Z every 500 ms using
//! `embassy-time`, and logs each toggle over RTT.
#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_nxp::gpio::{Level, Output};
use embassy_nxp_mkl82z7_examples as _;
use embassy_time::Timer;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_nxp::init(Default::default());
    defmt::info!("blinky: FRDM-KL82Z");

    // Active low: start with the LED off.
    let mut led = Output::new(p.PTC1, Level::High);

    loop {
        led.toggle();
        defmt::info!("LED {}", if led.level() == Level::High { "off" } else { "on" });
        Timer::after_millis(500).await;
    }
}
