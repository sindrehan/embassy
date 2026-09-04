//! Samples PTB0 and the ADC reference rails with 32-sample hardware averaging.
//!
//! PTB0 is available on J4 pin 12 (B6). Connect it to GND or 3.3 V to check the endpoint values.
#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_nxp::adc::{Adc, Averaging, Config, InternalChannel};
use embassy_nxp::{bind_interrupts, peripherals};
use embassy_nxp_mkl82z7_examples as _;
use embassy_time::Timer;

bind_interrupts!(struct Irqs {
    ADC0 => embassy_nxp::adc::InterruptHandler<peripherals::ADC0>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_nxp::init(Default::default());
    let mut input = p.PTB0;
    let mut config = Config::default();
    config.averaging = Averaging::Samples32;
    let mut adc = Adc::new(p.ADC0, Irqs, config).unwrap();

    loop {
        let input = adc.read(&mut input).await;
        let low = adc.read_internal(InternalChannel::VrefLow).await;
        let high = adc.read_internal(InternalChannel::VrefHigh).await;
        defmt::info!("PTB0: {=u16}, VREFL: {=u16}, VREFH: {=u16}", input, low, high);
        Timer::after_secs(1).await;
    }
}
