//! Sends a counter to the other board and reads its previous reply.
//! See the README for the two-board wiring.
#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_nxp::gpio::{Level, Output};
use embassy_nxp::spi::{self, Spi};
use embassy_nxp::{bind_interrupts, peripherals};
use embassy_nxp_mkl82z7_examples as _;
use embassy_time::Timer;

bind_interrupts!(struct Irqs {
    SPI0 => spi::InterruptHandler<peripherals::SPI0>;
});

#[embassy_executor::main(executor = "embassy_nxp::executor::Executor", entry = "cortex_m_rt::entry")]
async fn main(_spawner: Spawner) {
    let p = embassy_nxp::init(Default::default());
    let mut select = Output::new(p.PTC4, Level::High);
    let mut config = spi::Config::default();
    config.frequency = 500_000;
    let mut spi = Spi::new(p.SPI0, p.PTC5, p.PTC6, p.PTC7, Irqs, config);

    Timer::after_secs(2).await;
    let mut counter = 0u8;
    loop {
        let mut reply = [0; 1];
        select.set_low();
        spi.transfer(&mut reply, &[counter]).await.unwrap();
        select.set_high();
        defmt::info!("Sent {}, previous reply {}", counter, reply[0]);
        counter = counter.wrapping_add(1);
        Timer::after_millis(500).await;
    }
}
