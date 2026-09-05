//! Sends bytes through SPI0 and reads them back. Connect D11 (PTC6) to D12
//! (PTC7); disconnect any other device from these pins first.
#![no_std]
#![no_main]

use embassy_executor::Spawner;
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
    let mut spi = Spi::new(p.SPI0, p.PTC5, p.PTC6, p.PTC7, Irqs, spi::Config::default());

    loop {
        let mut received = [0; 4];
        spi.transfer(&mut received, &[1, 2, 3, 4]).await.unwrap();
        defmt::info!("Received: {:#04x}", received);
        Timer::after_secs(1).await;
    }
}
