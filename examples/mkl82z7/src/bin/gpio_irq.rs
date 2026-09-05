//! Toggles the red LED when SW3 (PTD0, active low) is pressed.
#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_nxp::gpio::{Input, Level, Output, Pull};
use embassy_nxp_mkl82z7_examples as _;
use embassy_time::Timer;

#[embassy_executor::main(executor = "embassy_nxp::executor::Executor", entry = "cortex_m_rt::entry")]
async fn main(_spawner: Spawner) {
    let p = embassy_nxp::init(Default::default());
    let mut led = Output::new(p.PTC1, Level::High);
    let mut button = Input::new(p.PTD0, Pull::Up);

    loop {
        button.wait_for_low().await;
        led.toggle();
        Timer::after_millis(20).await;
        button.wait_for_high().await;
        Timer::after_millis(20).await;
    }
}
