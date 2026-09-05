//! A DMA-only constructor must support in-place transfers without an SPI interrupt binding.
//! No loopback jumper is needed: this checks completion, not received data.
#![no_std]
#![no_main]

teleprobe_meta::target!(b"frdm-kl82z");

use embassy_executor::Spawner;
use embassy_nxp::spi::{Config, Spi};
use embassy_nxp_mkl82z7_tests as _;
use embassy_time::{Duration, with_timeout};

#[embassy_executor::main(executor = "embassy_nxp::executor::Executor", entry = "cortex_m_rt::entry")]
async fn main(_spawner: Spawner) {
    let p = embassy_nxp::init(Default::default());
    let mut config = Config::default();
    // The first receive must take long enough for the transfer future to yield.
    config.frequency = 1000;
    let mut spi = Spi::new_with_dma(p.SPI0, p.PTC5, p.PTC6, p.PTC7, p.DMA_CH0, p.DMA_CH1, config);
    let mut data = [0x5a; 16];
    with_timeout(Duration::from_secs(1), spi.transfer_in_place(&mut data))
        .await
        .expect("DMA-backed in-place SPI transfer did not wake its task")
        .unwrap();
    embassy_nxp_mkl82z7_tests::pass()
}
