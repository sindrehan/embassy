//! Slave half of the two-board FRDM-KL82Z SPI link example.
//!
//! Flash `spi_link_master` to the other board and connect the boards as described in the example
//! README. Each valid request prepares a reply that the master reads during its next exchange.
//! Between exchanges, the slave disconnects SPI1 and sleeps for 20 ms before reinitializing it.
//! The master must leave at least this much time between exchanges.
#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_nxp::gpio::{Level, Output};
use embassy_nxp::power::{self, SleepMode};
use embassy_nxp::spis::{self, Spis};
use embassy_nxp::{bind_interrupts, peripherals};
use embassy_nxp_mkl82z7_examples as _;
use embassy_nxp_mkl82z7_examples::spi_link::{FRAME_LEN, reply};
use embassy_time::Timer;

bind_interrupts!(struct Irqs {
    INTMUX0_0 => spis::InterruptHandler<peripherals::SPI1>;
});

#[embassy_executor::main(executor = "embassy_nxp::executor::Executor", entry = "cortex_m_rt::entry")]
async fn main(_spawner: Spawner) {
    let mut p = embassy_nxp::init(Default::default());
    let mut led = Output::new(p.PTC1, Level::High);
    power::set_sleep_mode(SleepMode::VeryLowPowerStop);

    defmt::info!("SPI1 link slave ready");
    let mut tx = [0xff; FRAME_LEN];
    loop {
        let mut spi = Spis::new(
            p.SPI1.reborrow(),
            p.PTD5.reborrow(),
            p.PTD6.reborrow(),
            p.PTD7.reborrow(),
            p.PTD4.reborrow(),
            Irqs,
            spis::Config::default(),
        );
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
        drop(spi);
        assert!(!embassy_nxp::clocks::is_enabled::<peripherals::SPI1>());
        let scb = unsafe { &*cortex_m::peripheral::SCB::PTR };
        assert_ne!(scb.scr.read() & (1 << 2), 0);
        Timer::after_millis(20).await;
    }
}
