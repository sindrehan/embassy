//! Checks SPI-slave cancellation, pin reuse and interrupt cleanup without an SPI master.
//! Leave PTC4..PTC7, PTC10/PTC11 and PTD4..PTD7 unconnected.
#![no_std]
#![no_main]

use core::pin::pin;

use embassy_executor::Spawner;
use embassy_futures::poll_once;
use embassy_hal_internal::interrupt::InterruptExt;
use embassy_nxp::adc::{Adc, Averaging, Config as AdcConfig, InternalChannel, Resolution};
use embassy_nxp::gpio::{Input, Pull};
use embassy_nxp::i2c::{self, I2c};
use embassy_nxp::power::{self, SleepMode};
use embassy_nxp::spis::{self, Spis};
use embassy_nxp::{Async, bind_interrupts, clocks, pac, peripherals};
use embassy_nxp_mkl82z7_examples as _;
use embassy_time::{Duration, Timer, with_timeout};

bind_interrupts!(struct Irqs {
    ADC0 => embassy_nxp::adc::InterruptHandler<peripherals::ADC0>;
    SPI0 => spis::InterruptHandler<peripherals::SPI0>;
    INTMUX0_0 => spis::InterruptHandler<peripherals::SPI1>, i2c::InterruptHandler<peripherals::I2C1>;
});

fn deep_sleep_enabled() -> bool {
    let scb = unsafe { &*cortex_m::peripheral::SCB::PTR };
    scb.scr.read() & (1 << 2) != 0
}

async fn check_cancellation(
    spi: &mut Spis<'_, Async>,
    adc: &mut Adc<'_, peripherals::ADC0, Async>,
    regs: pac::spi::Spi,
) {
    let mut rx = [0; 4];
    {
        let mut transfer = pin!(spi.read(&mut rx));
        assert!(poll_once(transfer.as_mut()).is_pending());
        assert!(!deep_sleep_enabled());

        // Changing the idle policy must not override an active transfer's guard.
        power::set_sleep_mode(SleepMode::Wait);
        power::set_sleep_mode(SleepMode::VeryLowPowerStop);
        assert!(!deep_sleep_enabled());

        with_timeout(Duration::from_millis(100), adc.read_internal(InternalChannel::VrefLow))
            .await
            .unwrap();
        assert!(!deep_sleep_enabled(), "ADC completion released the SPI wake guard");
    }
    assert!(deep_sleep_enabled());
    assert!(!regs.sr().read().txrxs());
    assert_eq!(regs.rser().read().0, 0);
    assert_eq!(regs.sr().read().txctr(), 0);
    assert_eq!(regs.sr().read().rxctr(), 0);

    // A second wait can be armed and cancelled by a deadline.
    assert!(
        with_timeout(Duration::from_millis(10), spi.read(&mut rx))
            .await
            .is_err()
    );
    assert!(deep_sleep_enabled());
}

fn check_disconnected(port: pac::port::Port) {
    for pin in 4..=7 {
        assert_eq!(port.pcr(pin).read().0, 0);
    }
}

#[embassy_executor::main(executor = "embassy_nxp::executor::Executor", entry = "cortex_m_rt::entry")]
async fn main(_spawner: Spawner) {
    let mut p = embassy_nxp::init(Default::default());
    let mut adc = Adc::new(p.ADC0, Irqs, AdcConfig::new(Resolution::Bits12, Averaging::Samples32)).unwrap();
    power::set_sleep_mode(SleepMode::VeryLowPowerStop);

    // I2C1 shares INTMUX0_0 with SPI1 and must keep working after SPI1 is dropped.
    let mut config = i2c::Config::default();
    config.internal_pullup = true;
    config.timeout = Duration::from_millis(100);
    let mut i2c = I2c::new(p.I2C1, p.PTC10, p.PTC11, Irqs, config);
    let other_sources = pac::INTMUX0.ch_ier_31_0(0).read().0;

    for _ in 0..4 {
        // Keep PCS0 inactive without an external master. The input's pull survives its drop.
        {
            let select = Input::new(p.PTC4.reborrow(), Pull::Up);
            assert!(select.is_high(), "SPI0 PCS0 must be inactive");
        }
        let mut spi = Spis::new(
            p.SPI0.reborrow(),
            p.PTC5.reborrow(),
            p.PTC6.reborrow(),
            p.PTC7.reborrow(),
            p.PTC4.reborrow(),
            Irqs,
            spis::Config::default(),
        );
        check_cancellation(&mut spi, &mut adc, pac::SPI0).await;
        drop(spi);
        assert!(!clocks::is_enabled::<peripherals::SPI0>());
        assert!(!pac::Interrupt::SPI0.is_enabled());
        assert!(!pac::Interrupt::SPI0.is_pending());
        check_disconnected(pac::PORTC);

        {
            let select = Input::new(p.PTD4.reborrow(), Pull::Up);
            assert!(select.is_high(), "SPI1 PCS0 must be inactive");
        }
        let mut spi = cortex_m::interrupt::free(|_| {
            pac::Interrupt::INTMUX0_0.pend();
            let spi = Spis::new(
                p.SPI1.reborrow(),
                p.PTD5.reborrow(),
                p.PTD6.reborrow(),
                p.PTD7.reborrow(),
                p.PTD4.reborrow(),
                Irqs,
                spis::Config::default(),
            );
            assert!(pac::Interrupt::INTMUX0_0.is_pending());
            spi
        });
        check_cancellation(&mut spi, &mut adc, pac::SPI1).await;
        // A pending shared interrupt belongs to the other peripherals too.
        cortex_m::interrupt::free(|_| {
            pac::Interrupt::INTMUX0_0.pend();
            drop(spi);
            assert!(pac::Interrupt::INTMUX0_0.is_pending());
        });
        assert!(!clocks::is_enabled::<peripherals::SPI1>());
        assert!(pac::Interrupt::INTMUX0_0.is_enabled());
        assert_eq!(pac::INTMUX0.ch_ier_31_0(0).read().0, other_sources);
        check_disconnected(pac::PORTD);
        assert_eq!(i2c.write(0x7e, &[0]).await, Err(i2c::Error::AddressNack));

        // Reuse the same peripheral and pins in blocking mode, then disconnect them again.
        let spi = Spis::new_blocking(
            p.SPI1.reborrow(),
            p.PTD5.reborrow(),
            p.PTD6.reborrow(),
            p.PTD7.reborrow(),
            p.PTD4.reborrow(),
            spis::Config::default(),
        );
        assert!(clocks::is_enabled::<peripherals::SPI1>());
        drop(spi);
        assert!(!clocks::is_enabled::<peripherals::SPI1>());
        check_disconnected(pac::PORTD);
        assert_eq!(pac::INTMUX0.ch_ier_31_0(0).read().0, other_sources);
        assert!(deep_sleep_enabled());
        Timer::after_millis(20).await;
    }

    power::set_sleep_mode(SleepMode::Wait);
    defmt::info!("SPI slave cancellation and lifecycle checks passed");
    embassy_nxp_mkl82z7_examples::exit()
}
