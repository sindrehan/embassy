//! Blinks the red LED (PTC1, active low) on the FRDM-KL82Z.
#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_nxp::gpio::{Level, Output};
use embassy_nxp_mkl82z7_examples as _;
use embassy_time::Timer;

#[embassy_executor::main(executor = "embassy_nxp::executor::Executor", entry = "cortex_m_rt::entry")]
async fn main(_spawner: Spawner) {
    let p = embassy_nxp::init(Default::default());
    let mut led = Output::new(p.PTC1, Level::High);

    loop {
        led.toggle();
        Timer::after_millis(500).await;
    }
}
