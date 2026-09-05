//! Standard async I/O traits: partial reads, exact reads, cancellation and serial errors.
#![no_std]
#![no_main]

teleprobe_meta::target!(b"frdm-kl82z");

use core::pin::pin;

use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_futures::poll_once;
use embassy_nxp::lpuart::{self, Error, Lpuart};
use embassy_nxp::power::{self, SleepMode};
use embassy_nxp::{Async, bind_interrupts, pac, peripherals};
use embassy_nxp_mkl82z7_tests as _;
use embassy_time::{Duration, Timer, with_timeout};
use embedded_io_async::{Read, Write};

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

fn deep_sleep() -> bool {
    unsafe { &*cortex_m::peripheral::SCB::PTR }.scr.read() & 4 != 0
}

async fn partial<U: Read<Error = Error> + Write<Error = Error>>(uart: &mut U) {
    assert_eq!(uart.read(&mut []).await.unwrap(), 0);
    assert_eq!(uart.write(&[]).await.unwrap(), 0);
    uart.flush().await.unwrap();
    uart.write_all(&[0x31, 0x32, 0x33]).await.unwrap();
    uart.flush().await.unwrap();
    let mut buffer = [0xa5; 32];
    assert_eq!(
        with_timeout(Duration::from_millis(100), uart.read(&mut buffer))
            .await
            .unwrap()
            .unwrap(),
        3
    );
    assert_eq!(&buffer[..3], &[0x31, 0x32, 0x33]);
    assert!(buffer[3..].iter().all(|&b| b == 0xa5));
}

async fn check(mut uart: Lpuart<'_, Async>) {
    loopback();
    partial(&mut uart).await;
    assert!(deep_sleep());
    let (mut tx, mut rx) = uart.split();
    let mut buffer = [0xa5; 32];
    {
        let mut read = pin!(Read::read(&mut rx, &mut buffer));
        assert!(poll_once(read.as_mut()).is_pending());
        assert!(!deep_sleep());
    }
    assert!(deep_sleep());
    assert_eq!(buffer, [0xa5; 32]);
    assert!(!pac::LPUART0.ctrl().read().rie());
    assert!(!pac::LPUART0.ctrl().read().orie());
    assert!(!pac::LPUART0.baud().read().rdmae());

    // A one-byte message must complete a read of a much larger buffer.
    let send = async {
        Timer::after_millis(10).await;
        Write::write_all(&mut tx, &[0x62]).await.unwrap();
        Write::flush(&mut tx).await.unwrap();
    };
    let (received, ()) = with_timeout(Duration::from_millis(100), join(Read::read(&mut rx, &mut buffer), send))
        .await
        .unwrap();
    assert_eq!(received.unwrap(), 1);
    assert_eq!(buffer[0], 0x62);
    assert!(buffer[1..].iter().all(|&b| b == 0xa5));

    let pattern = core::array::from_fn::<_, 64, _>(|i| i as u8);
    let mut received = [0; 64];
    let (read, written) = with_timeout(
        Duration::from_millis(200),
        join(
            Read::read_exact(&mut rx, &mut received),
            Write::write_all(&mut tx, &pattern),
        ),
    )
    .await
    .unwrap();
    read.unwrap();
    written.unwrap();
    Write::flush(&mut tx).await.unwrap();
    assert_eq!(received, pattern);
    assert!(deep_sleep());

    // A good character followed by a break must report the bytes before the framing error.
    Write::write_all(&mut tx, &[0x45]).await.unwrap();
    Write::flush(&mut tx).await.unwrap();
    pac::LPUART0.ctrl().modify(|w| w.set_sbk(true));
    pac::LPUART0.ctrl().modify(|w| w.set_sbk(false));
    tx.blocking_flush().unwrap();
    assert_eq!(Read::read(&mut rx, &mut buffer).await.unwrap(), 1);
    assert_eq!(buffer[0], 0x45);
    assert_eq!(Read::read(&mut rx, &mut buffer).await, Err(Error::Framing));
    assert!(deep_sleep());
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
    embedded_io::Write::write_all(&mut uart, &[0x78]).unwrap();
    embedded_io::Write::flush(&mut uart).unwrap();
    pac::LPUART0.ctrl().modify(|w| w.set_sbk(true));
    pac::LPUART0.ctrl().modify(|w| w.set_sbk(false));
    uart.blocking_flush().unwrap();
    let mut buffer = [0; 8];
    assert_eq!(embedded_io::Read::read(&mut uart, &mut buffer).unwrap(), 1);
    assert_eq!(buffer[0], 0x78);
    assert_eq!(embedded_io::Read::read(&mut uart, &mut buffer), Err(Error::Framing));
    drop(uart);
    assert!(deep_sleep());
    embassy_nxp_mkl82z7_tests::pass()
}
