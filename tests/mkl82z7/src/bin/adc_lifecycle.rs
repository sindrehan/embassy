//! ADC cancellation, clock cleanup, analog pin setup and peripheral reuse.
#![no_std]
#![no_main]

teleprobe_meta::target!(b"frdm-kl82z");

use core::pin::pin;

use embassy_executor::Spawner;
use embassy_futures::poll_once;
use embassy_hal_internal::interrupt::InterruptExt;
use embassy_nxp::adc::{self, Adc, Averaging, Config, InternalChannel, Resolution};
use embassy_nxp::gpio::{Input, Pull};
use embassy_nxp::{bind_interrupts, clocks, pac, peripherals};
use embassy_nxp_mkl82z7_tests as _;
use embassy_time::{Duration, with_timeout};

bind_interrupts!(struct Irqs {
    ADC0 => adc::InterruptHandler<peripherals::ADC0>;
});

fn stopped() {
    assert!(!clocks::is_enabled::<peripherals::ADC0>());
    assert!(!pac::Interrupt::ADC0.is_enabled());
    assert!(!pac::Interrupt::ADC0.is_pending());
}

#[embassy_executor::main(executor = "embassy_nxp::executor::Executor", entry = "cortex_m_rt::entry")]
async fn main(_spawner: Spawner) {
    let mut p = embassy_nxp::init(Default::default());
    let config = Config::new(Resolution::Bits16, Averaging::Samples32);
    for _ in 0..3 {
        let mut adc = Adc::new(p.ADC0.reborrow(), Irqs, config).unwrap();
        {
            let mut read = pin!(adc.read_internal(InternalChannel::VrefHigh));
            assert!(cortex_m::interrupt::free(|_| poll_once(read.as_mut()).is_pending()));
        }
        assert_eq!(pac::ADC0.sc1(0).read().adch().to_bits(), 31);
        assert!(!pac::ADC0.sc1(0).read().aien());
        assert!(
            with_timeout(Duration::from_millis(100), adc.read_internal(InternalChannel::VrefHigh))
                .await
                .unwrap()
                > 64000
        );
        {
            let _pin = Input::new(p.PTB0.reborrow(), Pull::Up);
        }
        adc.read(&mut p.PTB0).await;
        assert_eq!(
            pac::PORTB.pcr(0).read().0,
            0,
            "ADC retained digital pull/interrupt settings"
        );
        // A sampled pin is no longer borrowed. Dropping the ADC must not undo its new use.
        let input = Input::new(p.PTB0.reborrow(), Pull::Up);
        drop(adc);
        stopped();
        assert!(input.is_high());
        let mut adc = Adc::new_blocking(p.ADC0.reborrow(), config).unwrap();
        assert!(adc.blocking_read_internal(InternalChannel::VrefLow) < 655);
        drop(adc);
        stopped();
    }
    embassy_nxp_mkl82z7_tests::pass()
}
