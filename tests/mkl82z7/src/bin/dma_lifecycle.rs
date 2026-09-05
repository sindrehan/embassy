//! DMA transfers hold independent wake guards and stop before releasing their channels.
#![no_std]
#![no_main]

teleprobe_meta::target!(b"frdm-kl82z");

use embassy_executor::Spawner;
use embassy_nxp::lpuart::{Config, Lpuart};
use embassy_nxp::power::{self, SleepMode};
use embassy_nxp::{dma, pac, peripherals};
use embassy_nxp_mkl82z7_tests as _;
use embassy_time::{Duration, with_timeout};

fn deep_sleep() -> bool {
    unsafe { &*cortex_m::peripheral::SCB::PTR }.scr.read() & 4 != 0
}

fn stopped(channel: usize) {
    assert!(!pac::DMA.erq().read().erq(channel));
    assert!(!pac::DMA.tcd_csr(channel).read().active());
    assert!(!pac::DMAMUX.chcfg(channel).read().enbl());
    assert!(!pac::DMA.int().read().int(channel));
}

#[embassy_executor::main(executor = "embassy_nxp::executor::Executor", entry = "cortex_m_rt::entry")]
async fn main(_spawner: Spawner) {
    let mut p = embassy_nxp::init(Default::default());
    let mut uart = Lpuart::new_blocking(p.LPUART0, p.PTB17, p.PTB16, Config::default());
    let data = [0x5a];
    let register = pac::LPUART0.data().as_ptr() as *mut u8;
    power::set_sleep_mode(SleepMode::VeryLowPowerStop);

    // Source zero disables peripheral requests; software starts the completion check below.
    let first = unsafe { dma::write(p.DMA_CH0.reborrow(), 0, &data[..], register) };
    assert!(!deep_sleep());
    let second = unsafe { dma::write(p.DMA_CH1.reborrow(), 0, &data[..], register) };
    drop(first);
    stopped(0);
    assert!(!deep_sleep(), "one channel released another channel's guard");
    drop(second);
    stopped(1);
    assert!(deep_sleep());

    let transfer = unsafe { dma::write(p.DMA_CH0.reborrow(), 0, &data[..], register) };
    pac::DMA.tcd_csr(0).modify(|w| w.set_start(true));
    with_timeout(Duration::from_millis(100), transfer)
        .await
        .unwrap()
        .unwrap();
    stopped(0);
    assert!(deep_sleep());
    uart.blocking_flush().unwrap();
    assert!(embassy_nxp::clocks::is_enabled::<peripherals::LPUART0>());
    embassy_nxp_mkl82z7_tests::pass()
}
