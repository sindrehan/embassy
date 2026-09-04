//! Slave half of the two-board FRDM-KL82Z SPI link example.
//!
//! Flash `spi_link_master` to the other board and connect the boards as described in the example
//! README. Each valid request prepares a reply that the master reads during its next exchange.
#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_nxp::gpio::{Level, Output};
use embassy_nxp::spis::{self, Spis};
use embassy_nxp::{bind_interrupts, peripherals};
use embassy_nxp_mkl82z7_examples as _;
use embassy_nxp_mkl82z7_examples::spi_link::{FRAME_LEN, reply};

bind_interrupts!(struct Irqs {
    INTMUX0_0 => spis::InterruptHandler<peripherals::SPI1>;
});

#[embassy_executor::main(executor = "embassy_nxp::executor::Executor", entry = "cortex_m_rt::entry")]
async fn main(_spawner: Spawner) {
    let p = embassy_nxp::init(Default::default());
    let mut led = Output::new(p.PTC1, Level::High);
    let mut spi = Spis::new(p.SPI1, p.PTD5, p.PTD6, p.PTD7, p.PTD4, Irqs, spis::Config::default());

    defmt::info!("SPI1 link slave ready");
    let mut tx = [0xff; FRAME_LEN];
    loop {
        let mut rx = [0xff; FRAME_LEN];
        match spi.transfer(&mut rx, &tx).await {
            Ok((received, _)) if received == FRAME_LEN => match reply(&rx) {
                Some(next) => {
                    tx = next;
                    led.toggle();
                    defmt::info!("received request sequence {}", rx[1]);
                }
                None => {
                    tx = [0xff; FRAME_LEN];
                    defmt::warn!("invalid request: {:#04x}", rx);
                }
            },
            Ok((received, _)) => {
                tx = [0xff; FRAME_LEN];
                defmt::warn!("short SPI transaction: {} bytes", received);
            }
            Err(error) => {
                tx = [0xff; FRAME_LEN];
                defmt::warn!("SPI transfer failed: {:?}", error);
            }
        }
    }
}
