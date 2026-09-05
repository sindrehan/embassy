//! Blinks the red LED while the executor idles in VLPS.
//! Flash and reset without live RTT logging: VLPS can disrupt debugger access.
#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_nxp::clocks::ClockConfig;
use embassy_nxp::gpio::{Level, Output};
use embassy_nxp::power::SleepMode;
use embassy_nxp_mkl82z7_examples as _;
use embassy_time::Timer;

#[embassy_executor::main(executor = "embassy_nxp::executor::Executor", entry = "cortex_m_rt::entry")]
async fn main(_spawner: Spawner) {
    let mut config = embassy_nxp::config::Config {
        clocks: ClockConfig::vlpr(),
        ..Default::default()
    };
    config.power.sleep_mode = SleepMode::VeryLowPowerStop;
    let p = embassy_nxp::init(config);
    let mut led = Output::new(p.PTC1, Level::High);

    loop {
        led.set_low();
        Timer::after_millis(20).await;
        led.set_high();
        Timer::after_secs(2).await;
    }
}
