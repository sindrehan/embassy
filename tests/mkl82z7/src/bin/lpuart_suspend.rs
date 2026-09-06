//! UART pin isolation, stale-data cleanup, cancellation and split-half resume.
#![no_std]
#![no_main]

teleprobe_meta::target!(b"frdm-kl82z");

use core::pin::pin;

use embassy_executor::Spawner;
use embassy_futures::poll_once;
use embassy_nxp::lpuart::{self, Error, Lpuart};
use embassy_nxp::power::{self, SleepMode};
use embassy_nxp::{Async, bind_interrupts, pac, peripherals};
use embassy_nxp_mkl82z7_tests as _;

bind_interrupts!(struct Irqs {
    LPUART0 => lpuart::InterruptHandler<peripherals::LPUART0>;
});

fn loopback() {
    let regs = pac::LPUART0;
    regs.ctrl().modify(|w| w.set_re(false));
    while regs.ctrl().read().re() {}
    regs.ctrl().modify(|w| {
        w.set_loops(true);
        w.set_rsrc(false);
    });
    regs.fifo().modify(|w| w.set_rxflush(true));
    regs.ctrl().modify(|w| w.set_re(true));
}

fn isolated() {
    assert_eq!(pac::PORTB.pcr(16).read().0, 0);
    assert_eq!(pac::PORTB.pcr(17).read().0, 0);
    assert!(!pac::LPUART0.ctrl().read().re());
    assert!(!pac::LPUART0.ctrl().read().te());
    assert_eq!(pac::DMA.erq().read().0, 0);
    assert_ne!(unsafe { &*cortex_m::peripheral::SCB::PTR }.scr.read() & 4, 0);
}

async fn check(mut uart: Lpuart<'_, Async>) {
    loopback();
    for _ in 0..3 {
        uart.write(&[0x42]).await.unwrap();
        {
            let mut suspend = pin!(uart.suspend());
            assert!(poll_once(suspend.as_mut()).is_pending());
        }
        assert!(!uart.is_suspended());
        assert_eq!(pac::PORTB.pcr(17).read().mux().to_bits(), 3);
        uart.suspend().await;
        uart.suspend().await;
        isolated();
        assert_eq!(uart.write(&[0]).await, Err(Error::Suspended));
        assert_eq!(uart.read(&mut [0]).await, Err(Error::Suspended));
        uart.resume();
        uart.resume();
        assert_eq!(uart.try_read().unwrap(), None);
        uart.write(&[0x71]).await.unwrap();
        let mut byte = [0];
        uart.read(&mut byte).await.unwrap();
        assert_eq!(byte, [0x71]);
        uart.flush().await.unwrap();
    }
    let (mut tx, mut rx) = uart.split();
    {
        let mut buffer = [0; 16];
        let mut read = pin!(rx.read(&mut buffer));
        assert!(poll_once(read.as_mut()).is_pending());
    }
    rx.suspend();
    assert!(pac::LPUART0.ctrl().read().te());
    tx.write(&[0x12]).await.unwrap();
    tx.suspend().await;
    isolated();
    rx.resume();
    assert!(!pac::LPUART0.ctrl().read().te());
    tx.resume();
    tx.write(&[0x23]).await.unwrap();
    tx.flush().await.unwrap();
    assert_eq!(rx.try_read().unwrap(), Some(0x23));
    rx.suspend();
    tx.suspend().await;
    drop(rx);
    drop(tx);
    assert!(!embassy_nxp::clocks::is_enabled::<peripherals::LPUART0>());
    assert_eq!(pac::PORTB.pcr(16).read().0, 0);
    assert_eq!(pac::PORTB.pcr(17).read().0, 0);
}

#[embassy_executor::main(executor = "embassy_nxp::executor::Executor", entry = "cortex_m_rt::entry")]
async fn main(_spawner: Spawner) {
    let mut p = embassy_nxp::init(Default::default());
    power::set_sleep_mode(SleepMode::VeryLowPowerStop);
    let mut config = lpuart::Config::default();
    config.baudrate = 9600;
    check(Lpuart::new(
        p.LPUART0.reborrow(),
        p.PTB17.reborrow(),
        p.PTB16.reborrow(),
        Irqs,
        config.clone(),
    ))
    .await;
    check(Lpuart::new_with_dma(
        p.LPUART0.reborrow(),
        p.PTB17.reborrow(),
        p.PTB16.reborrow(),
        Irqs,
        p.DMA_CH0,
        p.DMA_CH1,
        config.clone(),
    ))
    .await;
    let mut uart = Lpuart::new_blocking(p.LPUART0, p.PTB17, p.PTB16, config);
    loopback();
    uart.blocking_write(&[0x34]).unwrap();
    uart.blocking_suspend();
    isolated();
    assert_eq!(uart.blocking_read(&mut [0]), Err(Error::Suspended));
    uart.resume();
    assert_eq!(uart.try_read().unwrap(), None);
    uart.blocking_write(&[0x45]).unwrap();
    uart.blocking_flush().unwrap();
    let mut byte = [0];
    uart.blocking_read(&mut byte).unwrap();
    assert_eq!(byte, [0x45]);
    drop(uart);
    embassy_nxp_mkl82z7_tests::pass()
}
