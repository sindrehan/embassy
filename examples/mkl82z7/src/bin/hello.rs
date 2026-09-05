//! Prints a greeting over RTT once per second.
#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_nxp_mkl82z7_examples as _;
use embassy_time::Timer;

#[embassy_executor::main(executor = "embassy_nxp::executor::Executor", entry = "cortex_m_rt::entry")]
async fn main(_spawner: Spawner) {
    let _p = embassy_nxp::init(Default::default());

    loop {
        defmt::info!("Hello from the FRDM-KL82Z!");
        Timer::after_secs(1).await;
    }
}
