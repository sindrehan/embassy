//! Replies with the previous request plus one. SPI shifts both directions
//! simultaneously, so each reply is read during the next exchange.
//! See the README for the two-board wiring.
#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_nxp::gpio::{Level, Output};
use embassy_nxp::spis::{self, Spis};
use embassy_nxp::{bind_interrupts, peripherals};
use embassy_nxp_mkl82z7_examples as _;

bind_interrupts!(struct Irqs {
    INTMUX0_0 => spis::InterruptHandler<peripherals::SPI1>;
});

#[embassy_executor::main(executor = "embassy_nxp::executor::Executor", entry = "cortex_m_rt::entry")]
async fn main(_spawner: Spawner) {
    let p = embassy_nxp::init(Default::default());
    let mut led = Output::new(p.PTC1, Level::High);
    let mut spi = Spis::new(p.SPI1, p.PTD5, p.PTD6, p.PTD7, p.PTD4, Irqs, spis::Config::default());
    let mut reply = [0; 1];

    loop {
        let mut request = [0; 1];
        let (received, _) = spi.transfer(&mut request, &reply).await.unwrap();
        if received == 1 {
            reply[0] = request[0].wrapping_add(1);
            led.toggle();
        }
    }
}
