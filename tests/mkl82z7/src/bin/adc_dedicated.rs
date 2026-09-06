//! Dedicated SE22 input selection, cancellation, averaging and peripheral reuse.
#![no_std]
#![no_main]

teleprobe_meta::target!(b"frdm-kl82z");

use core::pin::pin;

use embassy_executor::Spawner;
use embassy_futures::poll_once;
use embassy_nxp::adc::{self, Adc, Averaging, Config, Resolution};
use embassy_nxp::power::{self, SleepMode};
use embassy_nxp::{bind_interrupts, pac, peripherals};
use embassy_nxp_mkl82z7_tests as _;
use embassy_time::{Duration, with_timeout};

bind_interrupts!(struct Irqs {
    ADC0 => adc::InterruptHandler<peripherals::ADC0>;
});

fn deep_sleep() -> bool {
    unsafe { &*cortex_m::peripheral::SCB::PTR }.scr.read() & 4 != 0
}

#[embassy_executor::main(executor = "embassy_nxp::executor::Executor", entry = "cortex_m_rt::entry")]
async fn main(_spawner: Spawner) {
    let mut p = embassy_nxp::init(Default::default());
    power::set_sleep_mode(SleepMode::VeryLowPowerStop);
    let config = Config::new(Resolution::Bits10, Averaging::Samples8);
    for _ in 0..3 {
        let mut adc = Adc::new(p.ADC0.reborrow(), Irqs, config).unwrap();
        {
            let mut conversion = pin!(adc.read(&mut p.VREF_OUT));
            cortex_m::interrupt::free(|_| {
                assert!(poll_once(conversion.as_mut()).is_pending());
                assert_eq!(pac::ADC0.sc1(0).read().adch().to_bits(), 22);
                assert!(pac::ADC0.sc1(0).read().aien());
                assert!(!deep_sleep());
            });
        }
        assert_eq!(pac::ADC0.sc1(0).read().adch().to_bits(), 31);
        assert!(!pac::ADC0.sc1(0).read().aien());
        assert!(deep_sleep());
        let value = with_timeout(Duration::from_millis(100), adc.read(&mut p.VREF_OUT))
            .await
            .unwrap();
        // The devkit input may float; verify conversion completion, not its voltage.
        assert!(value <= 1023);
        assert!(pac::ADC0.sc3().read().avge());
        assert_eq!(pac::ADC0.sc3().read().avgs().to_bits(), 1);
        assert!(deep_sleep());
        drop(adc);
        let mut adc = Adc::new_blocking(p.ADC0.reborrow(), config).unwrap();
        assert!(adc.blocking_read(&mut p.VREF_OUT) <= 1023);
        drop(adc);
    }
    embassy_nxp_mkl82z7_tests::pass()
}
