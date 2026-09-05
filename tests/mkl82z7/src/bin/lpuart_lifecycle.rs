//! UART cancellation, deep-sleep guards, split-half teardown and reuse via internal loopback.
#![no_std]
#![no_main]

teleprobe_meta::target!(b"frdm-kl82z");

use core::pin::pin;

use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_futures::poll_once;
use embassy_hal_internal::interrupt::InterruptExt;
use embassy_nxp::lpuart::{self, Lpuart, LpuartRx, LpuartTx};
use embassy_nxp::power::{self, SleepMode};
use embassy_nxp::{Async, bind_interrupts, clocks, pac, peripherals};
use embassy_nxp_mkl82z7_tests as _;
use embassy_time::{Duration, Timer, with_timeout};

bind_interrupts!(struct Irqs {
    LPUART0 => lpuart::InterruptHandler<peripherals::LPUART0>;
});

fn deep_sleep() -> bool {
    unsafe { &*cortex_m::peripheral::SCB::PTR }.scr.read() & 4 != 0
}

fn loopback() {
    let regs = pac::LPUART0;
    regs.ctrl().modify(|w| w.set_re(false));
    while regs.ctrl().read().re() {}
    regs.ctrl().modify(|w| {
        w.set_loops(true);
        w.set_rsrc(false);
    });
    regs.fifo().modify(|w| w.set_rxflush(true));
    regs.stat().modify(|_| {});
    regs.ctrl().modify(|w| w.set_re(true));
}

fn stopped() {
    assert!(!clocks::is_enabled::<peripherals::LPUART0>());
    assert!(!pac::Interrupt::LPUART0.is_enabled());
    assert!(!pac::Interrupt::LPUART0.is_pending());
    assert_eq!(pac::PORTB.pcr(16).read().0, 0);
    assert_eq!(pac::PORTB.pcr(17).read().0, 0);
    assert!(deep_sleep());
}

async fn check(uart: Lpuart<'_, Async>) {
    let regs = pac::LPUART0;
    loopback();
    let (mut tx, mut rx) = uart.split();
    let mut received = [0; 64];
    {
        let mut read = pin!(rx.read(&mut received));
        assert!(poll_once(read.as_mut()).is_pending());
        assert!(!deep_sleep());
    }
    assert!(!regs.ctrl().read().rie());
    assert!(!regs.ctrl().read().orie());
    assert!(!regs.baud().read().rdmae());
    assert_eq!(pac::DMA.erq().read().0, 0);
    assert!(deep_sleep());

    // Queuing a byte is not transmission completion. The TC interrupt releases the guard.
    tx.write(&[0x42]).await.unwrap();
    assert!(!deep_sleep());
    {
        let mut flush = pin!(tx.flush());
        assert!(poll_once(flush.as_mut()).is_pending());
    }
    assert!(!deep_sleep(), "cancelled flush released the transmission guard");
    Timer::after_millis(20).await;
    assert!(regs.stat().read().tc());
    assert!(deep_sleep());
    assert_eq!(rx.try_read().unwrap(), Some(0x42));

    {
        let bytes = [0x5a; 128];
        let mut write = pin!(tx.write(&bytes));
        assert!(poll_once(write.as_mut()).is_pending());
        assert!(!deep_sleep());
    }
    assert!(!regs.ctrl().read().tie());
    assert!(!regs.baud().read().tdmae());
    assert_eq!(pac::DMA.erq().read().0, 0);
    assert!(!deep_sleep(), "cancelled write abandoned queued bytes");
    Timer::after_millis(20).await;
    assert!(deep_sleep());
    loopback();

    let pattern = core::array::from_fn::<_, 64, _>(|i| i as u8);
    let (written, read) = with_timeout(
        Duration::from_millis(200),
        join(tx.write(&pattern), rx.read(&mut received)),
    )
    .await
    .unwrap();
    written.unwrap();
    read.unwrap();
    assert_eq!(received, pattern);
    tx.flush().await.unwrap();
    assert!(deep_sleep());

    drop(rx);
    assert!(clocks::is_enabled::<peripherals::LPUART0>());
    assert!(pac::Interrupt::LPUART0.is_enabled());
    assert!(!regs.ctrl().read().re());
    assert!(regs.ctrl().read().te());
    assert_eq!(pac::PORTB.pcr(16).read().0, 0);
    tx.write(&[0x33; 16]).await.unwrap();
    tx.flush().await.unwrap();
    drop(tx);
    stopped();
}

#[embassy_executor::main(executor = "embassy_nxp::executor::Executor", entry = "cortex_m_rt::entry")]
async fn main(_spawner: Spawner) {
    let mut p = embassy_nxp::init(Default::default());
    power::set_sleep_mode(SleepMode::VeryLowPowerStop);
    let mut config = lpuart::Config::default();
    config.baudrate = 9600;
    for _ in 0..2 {
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
            p.DMA_CH0.reborrow(),
            p.DMA_CH1.reborrow(),
            config.clone(),
        ))
        .await;
    }

    let uart = Lpuart::new(
        p.LPUART0.reborrow(),
        p.PTB17.reborrow(),
        p.PTB16.reborrow(),
        Irqs,
        config.clone(),
    );
    loopback();
    let (tx, mut rx) = uart.split();
    drop(tx);
    assert!(clocks::is_enabled::<peripherals::LPUART0>());
    assert!(pac::Interrupt::LPUART0.is_enabled());
    assert!(pac::LPUART0.ctrl().read().re());
    assert!(!pac::LPUART0.ctrl().read().te());
    assert_eq!(pac::PORTB.pcr(17).read().0, 0);
    // Supply an internal loopback stimulus with TX's pin still disconnected.
    pac::LPUART0.ctrl().modify(|w| w.set_te(true));
    pac::LPUART0.data().write(|w| w.0 = 0x71);
    let mut byte = [0];
    with_timeout(Duration::from_millis(100), rx.read(&mut byte))
        .await
        .unwrap()
        .unwrap();
    assert_eq!(byte, [0x71]);
    pac::LPUART0.ctrl().modify(|w| w.set_te(false));
    while pac::LPUART0.ctrl().read().te() {}
    drop(rx);
    stopped();

    let mut tx = LpuartTx::new_blocking(p.LPUART0.reborrow(), p.PTB17.reborrow(), config.clone());
    tx.blocking_write(&[0x32]).unwrap();
    assert!(!deep_sleep());
    tx.blocking_flush().unwrap();
    assert!(deep_sleep());
    drop(tx);
    stopped();
    let rx = LpuartRx::new(p.LPUART0.reborrow(), p.PTB16.reborrow(), Irqs, config);
    drop(rx);
    stopped();
    embassy_nxp_mkl82z7_tests::pass()
}
