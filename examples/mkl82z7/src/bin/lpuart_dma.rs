//! LPUART over DMA, checked through internal loopback. LPUART2 (the instance
//! with a 1-byte FIFO, which overruns at 115200 in interrupt mode) runs 256
//! bytes at 115200 with DMA on both directions; LPUART0 then does 1024 bytes
//! at 1 Mbaud. LPUART2's pins PTD3/PTD2 are the accelerometer's I2C0 pins, so
//! the sensor sees some traffic; it recovers at its next START.
#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_nxp::lpuart::{self, Lpuart};
use embassy_nxp::{bind_interrupts, pac, peripherals};
use embassy_nxp_mkl82z7_examples as _;
use embassy_time::{Duration, Instant, with_timeout};

bind_interrupts!(struct Irqs {
    INTMUX0_0 => lpuart::InterruptHandler<peripherals::LPUART2>;
    LPUART0 => lpuart::InterruptHandler<peripherals::LPUART0>;
});

fn loopback(regs: pac::lpuart::Lpuart) {
    regs.ctrl().modify(|w| w.set_re(false));
    regs.ctrl().modify(|w| {
        w.set_loops(true);
        w.set_rsrc(false);
    });
    regs.fifo().modify(|w| w.set_rxflush(true));
    regs.ctrl().modify(|w| w.set_re(true));
}

async fn check<'d>(
    name: &str,
    uart: Lpuart<'d, embassy_nxp::Async>,
    regs: pac::lpuart::Lpuart,
    tx_buf: &[u8],
    rx_buf: &mut [u8],
) {
    loopback(regs);
    let (mut tx, mut rx) = uart.split();
    let start = Instant::now();
    let (written, read) = with_timeout(Duration::from_secs(2), join(tx.write(tx_buf), rx.read(rx_buf)))
        .await
        .expect("DMA transfer timed out");
    written.unwrap();
    read.unwrap();
    let elapsed = start.elapsed();
    defmt::assert_eq!(rx_buf, tx_buf, "loopback mismatch");
    defmt::info!("{}: {} bytes ok in {} us", name, tx_buf.len(), elapsed.as_micros());
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_nxp::init(Default::default());
    defmt::info!("lpuart dma");

    let mut pattern = [0u8; 1024];
    for (i, b) in pattern.iter_mut().enumerate() {
        *b = (i as u8).wrapping_mul(31).wrapping_add(5);
    }
    let mut got = [0u8; 1024];

    let uart = Lpuart::new_with_dma(
        p.LPUART2,
        p.PTD3,
        p.PTD2,
        Irqs,
        p.DMA_CH0,
        p.DMA_CH1,
        lpuart::Config::default(),
    );
    check(
        "LPUART2 at 115200",
        uart,
        pac::LPUART2,
        &pattern[..256],
        &mut got[..256],
    )
    .await;

    let mut config = lpuart::Config::default();
    config.baudrate = 1_000_000;
    let uart = Lpuart::new_with_dma(p.LPUART0, p.PTB17, p.PTB16, Irqs, p.DMA_CH2, p.DMA_CH3, config);
    check("LPUART0 at 1 Mbaud", uart, pac::LPUART0, &pattern, &mut got).await;

    defmt::info!("lpuart dma passed");
    embassy_nxp_mkl82z7_examples::exit()
}
