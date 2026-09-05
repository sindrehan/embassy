//! Checks ADC completion and cancellation with VLPS selected. No external connections required.
#![no_std]
#![no_main]

teleprobe_meta::target!(b"frdm-kl82z");

use core::pin::pin;

use embassy_executor::Spawner;
use embassy_futures::poll_once;
use embassy_nxp::adc::{Adc, Averaging, Config, InternalChannel, Resolution};
use embassy_nxp::clocks::{ClockConfig, ExternalClock, ExternalSource};
use embassy_nxp::power::{self, SleepMode};
use embassy_nxp::{bind_interrupts, pac, peripherals};
use embassy_nxp_mkl82z7_tests as _;
use embassy_time::{Duration, Timer, with_timeout};

bind_interrupts!(struct Irqs {
    ADC0 => embassy_nxp::adc::InterruptHandler<peripherals::ADC0>;
});

fn deep_sleep_enabled() -> bool {
    let scb = unsafe { &*cortex_m::peripheral::SCB::PTR };
    scb.scr.read() & (1 << 2) != 0
}

#[embassy_executor::main(executor = "embassy_nxp::executor::Executor", entry = "cortex_m_rt::entry")]
async fn main(_spawner: Spawner) {
    let config = embassy_nxp::config::Config {
        clocks: ClockConfig::pll(
            ExternalClock {
                frequency: 12_000_000,
                source: ExternalSource::Crystal {
                    high_gain: false,
                    load_capacitance_pf: 0,
                },
            },
            1,
            24,
        ),
        ..Default::default()
    };
    let p = embassy_nxp::init(config);
    let mut adc = Adc::new(p.ADC0, Irqs, Config::new(Resolution::Bits12, Averaging::Samples32)).unwrap();
    power::set_sleep_mode(SleepMode::VeryLowPowerStop);

    for _ in 0..16 {
        // Mask the ADC interrupt so the first poll cannot complete before cancellation.
        cortex_m::interrupt::free(|_| {
            let mut conversion = pin!(adc.read_internal(InternalChannel::VrefLow));
            assert!(poll_once(conversion.as_mut()).is_pending());
            assert!(!deep_sleep_enabled());
        });
        assert!(deep_sleep_enabled());
        assert_eq!(pac::ADC0.sc1(0).read().adch().to_bits(), 31);
        assert!(!pac::ADC0.sc1(0).read().aien());

        Timer::after_millis(20).await;
        let low = with_timeout(Duration::from_millis(100), adc.read_internal(InternalChannel::VrefLow))
            .await
            .expect("ADC did not complete after VLPS");
        assert!(deep_sleep_enabled());
        let high = with_timeout(Duration::from_millis(100), adc.read_internal(InternalChannel::VrefHigh))
            .await
            .expect("ADC did not complete after cancellation");
        assert!(deep_sleep_enabled());
        assert!(low < 32 && high > 4063);
    }

    power::set_sleep_mode(SleepMode::Wait);
    defmt::info!("ADC VLPS and cancellation checks passed");
    embassy_nxp_mkl82z7_tests::pass()
}
