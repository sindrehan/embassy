//! Two-board DMA SPI test: in-place, unequal-length and staging-buffer-boundary transfers.
#![no_std]
#![no_main]

teleprobe_meta::target!(b"frdm-kl82z");
teleprobe_meta::timeout!(40);

use embassy_executor::Spawner;
use embassy_nxp::gpio::{Level, Output};
use embassy_nxp::spi::{self, Spi};
use embassy_nxp_mkl82z7_tests as _;
use embassy_nxp_mkl82z7_tests::spi_link::{FRAME_LEN, reply, request};
use embassy_time::Timer;

#[embassy_executor::main(executor = "embassy_nxp::executor::Executor", entry = "cortex_m_rt::entry")]
async fn main(_spawner: Spawner) {
    let p = embassy_nxp::init(Default::default());
    let mut select = Output::new(p.PTC4, Level::High);
    let mut config = spi::Config::default();
    config.frequency = 100_000;
    let mut spi = Spi::new_with_dma(p.SPI0, p.PTC5, p.PTC6, p.PTC7, p.DMA_CH0, p.DMA_CH1, config);
    Timer::after_secs(2).await;

    let mut expected = None;
    for transaction in 0..260 {
        let tx = request(transaction as u8);
        let mut outgoing = [0xff; 130];
        outgoing[..FRAME_LEN].copy_from_slice(&tx);
        let mut rx = [0; 130];

        select.set_low();
        Timer::after_micros(10).await;
        match transaction % 4 {
            0 => spi.transfer(&mut rx[..FRAME_LEN], &tx).await.unwrap(),
            1 => {
                spi.transfer_in_place(&mut outgoing).await.unwrap();
                rx = outgoing;
            }
            2 => spi.transfer(&mut rx, &tx).await.unwrap(),
            _ => spi.transfer(&mut rx[..FRAME_LEN], &outgoing).await.unwrap(),
        }
        Timer::after_micros(10).await;
        select.set_high();

        if let Some(expected) = expected {
            assert_eq!(rx[..FRAME_LEN], expected);
            if matches!(transaction % 4, 1 | 2) {
                assert!(rx[FRAME_LEN..].iter().all(|&byte| byte == 0));
            }
        }
        expected = reply(&tx);
        Timer::after_millis(50).await;
    }
    embassy_nxp_mkl82z7_tests::pass()
}
