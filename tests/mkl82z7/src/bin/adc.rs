//! Calibration and hardware averaging against the internal reference rails.
#![no_std]
#![no_main]

teleprobe_meta::target!(b"frdm-kl82z");

use embassy_executor::Spawner;
use embassy_nxp::adc::{Adc, Averaging, Config, InternalChannel, Resolution};
use embassy_nxp::{bind_interrupts, peripherals};
use embassy_nxp_mkl82z7_tests as _;
use embassy_time::{Duration, with_timeout};

bind_interrupts!(struct Irqs {
    ADC0 => embassy_nxp::adc::InterruptHandler<peripherals::ADC0>;
});

#[embassy_executor::main(executor = "embassy_nxp::executor::Executor", entry = "cortex_m_rt::entry")]
async fn main(_spawner: Spawner) {
    let p = embassy_nxp::init(Default::default());
    let mut adc = Adc::new(p.ADC0, Irqs, Config::default()).unwrap();
    for (resolution, maximum) in [
        (Resolution::Bits8, 255),
        (Resolution::Bits10, 1023),
        (Resolution::Bits12, 4095),
        (Resolution::Bits16, 65535),
    ] {
        adc.set_resolution(resolution);
        for averaging in [
            Averaging::Disabled,
            Averaging::Samples4,
            Averaging::Samples8,
            Averaging::Samples16,
            Averaging::Samples32,
        ] {
            adc.set_averaging(averaging);
            adc.calibrate().unwrap();
            let low = adc.blocking_read_internal(InternalChannel::VrefLow);
            let high = with_timeout(Duration::from_millis(100), adc.read_internal(InternalChannel::VrefHigh))
                .await
                .unwrap();
            assert!(u32::from(low) <= maximum / 100 + 1);
            assert!(u32::from(high) >= maximum * 99 / 100);
        }
    }
    embassy_nxp_mkl82z7_tests::pass()
}
