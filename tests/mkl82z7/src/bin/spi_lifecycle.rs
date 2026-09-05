//! SPI master cancellation, teardown and reuse. No loopback jumper is required.
#![no_std]
#![no_main]

teleprobe_meta::target!(b"frdm-kl82z");

use core::pin::pin;

use embassy_executor::Spawner;
use embassy_futures::poll_once;
use embassy_hal_internal::interrupt::InterruptExt;
use embassy_nxp::power::{self, SleepMode};
use embassy_nxp::spi::{self, Spi};
use embassy_nxp::{Async, bind_interrupts, clocks, pac, peripherals};
use embassy_nxp_mkl82z7_tests as _;
use embassy_time::{Duration, with_timeout};

bind_interrupts!(struct Irqs {
    SPI0 => spi::InterruptHandler<peripherals::SPI0>;
});

fn disconnected() {
    assert!(!clocks::is_enabled::<peripherals::SPI0>());
    assert!(!pac::Interrupt::SPI0.is_enabled());
    assert!(!pac::Interrupt::SPI0.is_pending());
    for pin in 5..=7 {
        assert_eq!(pac::PORTC.pcr(pin).read().0, 0);
    }
}

async fn check(mut spi: Spi<'_, Async>) {
    {
        let data = [0x5a; 130];
        let mut transfer = pin!(spi.write(&data));
        assert!(poll_once(transfer.as_mut()).is_pending());
    }
    assert_eq!(pac::SPI0.rser().read().0, 0);
    assert!(!pac::SPI0.sr().read().txrxs());
    assert_eq!(pac::SPI0.sr().read().txctr(), 0);
    assert_eq!(pac::SPI0.sr().read().rxctr(), 0);
    assert_eq!(pac::DMA.erq().read().0, 0);
    let mut data = [0xa5; 8];
    with_timeout(Duration::from_millis(200), spi.transfer_in_place(&mut data))
        .await
        .unwrap()
        .unwrap();
    drop(spi);
    disconnected();
}

#[embassy_executor::main(executor = "embassy_nxp::executor::Executor", entry = "cortex_m_rt::entry")]
async fn main(_spawner: Spawner) {
    let mut p = embassy_nxp::init(Default::default());
    power::set_sleep_mode(SleepMode::VeryLowPowerStop);
    let mut config = spi::Config::default();
    config.frequency = 1000;
    for _ in 0..3 {
        check(Spi::new(
            p.SPI0.reborrow(),
            p.PTC5.reborrow(),
            p.PTC6.reborrow(),
            p.PTC7.reborrow(),
            Irqs,
            config.clone(),
        ))
        .await;
        check(Spi::new_with_dma(
            p.SPI0.reborrow(),
            p.PTC5.reborrow(),
            p.PTC6.reborrow(),
            p.PTC7.reborrow(),
            p.DMA_CH0.reborrow(),
            p.DMA_CH1.reborrow(),
            config.clone(),
        ))
        .await;
        let mut spi = Spi::new_blocking(
            p.SPI0.reborrow(),
            p.PTC5.reborrow(),
            p.PTC6.reborrow(),
            p.PTC7.reborrow(),
            config.clone(),
        );
        spi.blocking_write(&[0x42]).unwrap();
        drop(spi);
        disconnected();
    }
    embassy_nxp_mkl82z7_tests::pass()
}
