//! An interrupt-driven SPI master must prevent deep sleep until completion or cancellation.
//! No loopback jumper is required.
#![no_std]
#![no_main]

teleprobe_meta::target!(b"frdm-kl82z");

use core::pin::pin;

use embassy_executor::Spawner;
use embassy_futures::poll_once;
use embassy_nxp::power::{self, SleepMode};
use embassy_nxp::spi::{self, Spi};
use embassy_nxp::{bind_interrupts, peripherals};
use embassy_nxp_mkl82z7_tests as _;

bind_interrupts!(struct Irqs {
    SPI0 => spi::InterruptHandler<peripherals::SPI0>;
});

#[embassy_executor::main(executor = "embassy_nxp::executor::Executor", entry = "cortex_m_rt::entry")]
async fn main(_spawner: Spawner) {
    let p = embassy_nxp::init(Default::default());
    let mut config = spi::Config::default();
    config.frequency = 1000;
    let mut spi = Spi::new(p.SPI0, p.PTC5, p.PTC6, p.PTC7, Irqs, config);
    power::set_sleep_mode(SleepMode::VeryLowPowerStop);
    let scb = unsafe { &*cortex_m::peripheral::SCB::PTR };
    {
        let bytes = [0x5a; 16];
        let mut transfer = pin!(spi.write(&bytes));
        assert!(poll_once(transfer.as_mut()).is_pending());
        assert_eq!(scb.scr.read() & 4, 0, "SPI transfer permits deep sleep");
    }
    assert_ne!(scb.scr.read() & 4, 0, "SPI cancellation retained its wake guard");
    embassy_nxp_mkl82z7_tests::pass()
}
