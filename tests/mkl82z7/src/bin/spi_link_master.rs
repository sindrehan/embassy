//! Master half of the two-board FRDM-KL82Z SPI test.
//!
//! Flash `spi_link_slave` to the other board and connect the boards as described in the test
//! README. The first exchange primes the slave's reply; every later exchange checks the reply to
//! the previous request.
#![no_std]
#![no_main]

teleprobe_meta::target!(b"frdm-kl82z");

use embassy_executor::Spawner;
use embassy_nxp::gpio::{Level, Output};
use embassy_nxp::spi::{self, Spi};
use embassy_nxp::{bind_interrupts, peripherals};
use embassy_nxp_mkl82z7_tests as _;
use embassy_nxp_mkl82z7_tests::spi_link::{FRAME_LEN, reply, request};
use embassy_time::Timer;

bind_interrupts!(struct Irqs {
    SPI0 => spi::InterruptHandler<peripherals::SPI0>;
});

#[embassy_executor::main(executor = "embassy_nxp::executor::Executor", entry = "cortex_m_rt::entry")]
async fn main(_spawner: Spawner) {
    let p = embassy_nxp::init(Default::default());
    let mut led = Output::new(p.PTC1, Level::High);
    let mut select = Output::new(p.PTC4, Level::High);

    let mut config = spi::Config::default();
    config.frequency = 500_000;
    let mut spi = Spi::new(p.SPI0, p.PTC5, p.PTC6, p.PTC7, Irqs, config);

    defmt::info!("SPI link master ready at 500 kHz");
    Timer::after_secs(2).await;

    let mut sequence = 0u8;
    let mut expected = None;
    // Cross the sequence-number wrap and check every reply after priming.
    for _ in 0..260 {
        let tx = request(sequence);
        let mut rx = [0xff; FRAME_LEN];

        select.set_low();
        Timer::after_micros(10).await;
        let result = spi.transfer(&mut rx, &tx).await;
        Timer::after_micros(10).await;
        select.set_high();
        result.unwrap();

        if let Some(expected) = expected {
            defmt::assert_eq!(rx, expected, "bad reply for sequence {}", sequence.wrapping_sub(1));
            led.toggle();
        } else {
            defmt::info!("link primed");
        }

        expected = reply(&tx);
        sequence = sequence.wrapping_add(1);
        Timer::after_millis(50).await;
    }
    embassy_nxp_mkl82z7_tests::pass()
}
