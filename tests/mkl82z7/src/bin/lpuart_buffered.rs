//! Buffered RX through executor stalls, ring overflow, cancellation and suspend/resume.
#![no_std]
#![no_main]

teleprobe_meta::target!(b"frdm-kl82z");

use core::pin::pin;

use embassy_executor::Spawner;
use embassy_futures::poll_once;
use embassy_nxp::lpuart::{self, Error, Lpuart};
use embassy_nxp::power::{self, SleepMode};
use embassy_nxp::{bind_interrupts, pac, peripherals};
use embassy_nxp_mkl82z7_tests as _;
use embassy_time::{Duration, with_timeout};

bind_interrupts!(struct Irqs {
    LPUART0 => lpuart::InterruptHandler<peripherals::LPUART0>;
});

fn deep_sleep() -> bool {
    unsafe { &*cortex_m::peripheral::SCB::PTR }.scr.read() & 4 != 0
}

#[embassy_executor::main(executor = "embassy_nxp::executor::Executor", entry = "cortex_m_rt::entry")]
async fn main(_spawner: Spawner) {
    let mut p = embassy_nxp::init(Default::default());
    power::set_sleep_mode(SleepMode::VeryLowPowerStop);
    let uart = Lpuart::new(
        p.LPUART0.reborrow(),
        p.PTB17.reborrow(),
        p.PTB16.reborrow(),
        Irqs,
        Default::default(),
    );
    let regs = pac::LPUART0;
    regs.ctrl().modify(|w| w.set_re(false));
    while regs.ctrl().read().re() {}
    regs.ctrl().modify(|w| {
        w.set_loops(true);
        w.set_rsrc(false);
    });
    regs.ctrl().modify(|w| w.set_re(true));
    let (mut tx, rx) = uart.split();
    let storage = cortex_m::singleton!(: [u8; 128] = [0; 128]).unwrap();
    let mut rx = rx.into_buffered(storage);
    assert!(!deep_sleep());
    let pattern = core::array::from_fn::<_, 96, _>(|i| i as u8);
    let mut received = [0; 96];
    for _ in 0..5 {
        // No await: only interrupts can move received bytes while this task occupies the core.
        tx.blocking_write(&pattern).unwrap();
        tx.blocking_flush().unwrap();
        with_timeout(
            Duration::from_millis(100),
            embedded_io_async::Read::read_exact(&mut rx, &mut received),
        )
        .await
        .unwrap()
        .unwrap();
        assert_eq!(received, pattern);
        assert!(!deep_sleep());
    }
    {
        let mut byte = [0xa5];
        {
            let mut read = pin!(rx.read(&mut byte));
            assert!(poll_once(read.as_mut()).is_pending());
        }
        assert_eq!(byte, [0xa5]);
    }
    assert!(regs.ctrl().read().rie());
    assert!(!deep_sleep());
    tx.blocking_write(&[0x55; 160]).unwrap();
    tx.blocking_flush().unwrap();
    assert_eq!(rx.read(&mut received).await, Err(Error::Overrun));
    assert_eq!(rx.read(&mut received).await, Err(Error::Overrun));
    rx.recover();
    assert_eq!(rx.try_read(&mut received).unwrap(), 0);
    tx.blocking_write(&[0x31]).unwrap();
    tx.blocking_flush().unwrap();
    assert_eq!(rx.read(&mut received).await.unwrap(), 1);
    assert_eq!(received[0], 0x31);

    // Hardware overrun while interrupts are masked must also latch an error.
    cortex_m::interrupt::free(|_| {
        tx.blocking_write(&[0x66; 32]).unwrap();
        tx.blocking_flush().unwrap();
    });
    assert_eq!(rx.read(&mut received).await, Err(Error::Overrun));
    rx.recover();
    regs.ctrl().modify(|w| w.set_sbk(true));
    regs.ctrl().modify(|w| w.set_sbk(false));
    tx.blocking_flush().unwrap();
    assert_eq!(rx.read(&mut received).await, Err(Error::Framing));
    rx.suspend();
    assert!(deep_sleep());
    assert_eq!(pac::PORTB.pcr(16).read().0, 0);
    assert_eq!(rx.read(&mut received).await, Err(Error::Suspended));
    tx.blocking_write(&[0x77]).unwrap();
    tx.blocking_flush().unwrap();
    rx.resume();
    assert!(!deep_sleep());
    assert_eq!(rx.try_read(&mut received).unwrap(), 0);
    tx.blocking_write(&[0x42]).unwrap();
    tx.blocking_flush().unwrap();
    assert_eq!(rx.read(&mut received).await.unwrap(), 1);
    assert_eq!(received[0], 0x42);
    let (rx, storage) = rx.into_inner();
    assert!(deep_sleep());
    assert!(rx.is_suspended());
    let mut rx = rx.into_buffered(storage);
    assert!(rx.is_suspended());
    rx.resume();
    tx.blocking_write(&[0x43]).unwrap();
    tx.blocking_flush().unwrap();
    assert_eq!(rx.read(&mut received).await.unwrap(), 1);
    assert_eq!(received[0], 0x43);
    drop(rx);
    assert!(deep_sleep());
    assert!(regs.ctrl().read().te());
    drop(tx);
    // A forgotten receiver may outlive a peripheral reborrow; reinitialization must retire its
    // static ring and wake guard before using the unbuffered interrupt path again.
    let rx = lpuart::LpuartRx::new(p.LPUART0.reborrow(), p.PTB16.reborrow(), Irqs, Default::default());
    let storage = cortex_m::singleton!(: [u8; 16] = [0; 16]).unwrap();
    core::mem::forget(rx.into_buffered(storage));
    assert!(!deep_sleep());
    let uart = Lpuart::new(p.LPUART0, p.PTB17, p.PTB16, Irqs, Default::default());
    assert!(deep_sleep());
    regs.ctrl().modify(|w| w.set_re(false));
    while regs.ctrl().read().re() {}
    regs.ctrl().modify(|w| w.set_loops(true));
    regs.ctrl().modify(|w| w.set_re(true));
    let (mut tx, mut rx) = uart.split();
    tx.blocking_write(&[0x91]).unwrap();
    tx.blocking_flush().unwrap();
    rx.read(&mut received[..1]).await.unwrap();
    assert_eq!(received[0], 0x91);
    drop(rx);
    drop(tx);
    embassy_nxp_mkl82z7_tests::pass()
}
