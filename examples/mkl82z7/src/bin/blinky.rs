//! Blinks the red LED (PTC1, active low) on the FRDM-KL82Z every 500 ms using
//! `embassy-time`, with the core clocked at 72 MHz from the 12 MHz crystal.
#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_nxp::clocks::{ClockConfig, ExternalClock, ExternalSource};
use embassy_nxp::gpio::{Level, Output};
use embassy_nxp_mkl82z7_examples::{self as _, spin_benchmark};
use embassy_time::Timer;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let mut config = embassy_nxp::config::Config::default();
    // 12 MHz crystal, 22 pF load: PLL at 144 MHz, core 72 MHz, bus and flash 24 MHz.
    config.clocks = ClockConfig::pll(
        ExternalClock {
            frequency: 12_000_000,
            source: ExternalSource::Crystal {
                high_gain: false,
                load_capacitance_pf: 22,
            },
        },
        1,
        24,
    );
    let p = embassy_nxp::init(config);

    defmt::info!("blinky: FRDM-KL82Z, clocks {:?}", embassy_nxp::clocks::clocks());
    defmt::info!("spin benchmark: {} us", spin_benchmark());

    // Active low: start with the LED off.
    let mut led = Output::new(p.PTC1, Level::High);

    loop {
        led.toggle();
        defmt::info!("LED {}", if led.level() == Level::High { "off" } else { "on" });
        Timer::after_millis(500).await;
    }
}
