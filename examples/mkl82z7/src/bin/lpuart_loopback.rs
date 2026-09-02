//! Self test of the LPUART driver without any wiring: LPUART0 is put in
//! internal loopback (CTRL[LOOPS], TX fed back into RX), a pattern is sent
//! with the async API and read back, then the same with the blocking API.
//! Exits with success when both match.
#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_nxp::lpuart::{Config, InterruptHandler, Lpuart};
use embassy_nxp::{bind_interrupts, pac, peripherals};
use embassy_nxp_mkl82z7_examples as _;
use embassy_time::{Duration, with_timeout};

bind_interrupts!(struct Irqs {
    LPUART0 => InterruptHandler<peripherals::LPUART0>;
});

/// Switch LPUART0 to internal loopback. Flipping LOOPS with the receiver running produces a
/// glitch byte, so stop the receiver, switch, flush and restart it.
fn loopback() {
    let regs = pac::LPUART0;
    regs.ctrl().modify(|w| w.set_re(false));
    regs.ctrl().modify(|w| {
        w.set_loops(true);
        w.set_rsrc(false);
    });
    regs.fifo().modify(|w| w.set_rxflush(true));
    regs.stat().modify(|_| {});
    regs.ctrl().modify(|w| w.set_re(true));
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_nxp::init(Default::default());
    defmt::info!("lpuart loopback: LPUART0 at 115200");

    let mut pattern = [0u8; 64];
    for (i, b) in pattern.iter_mut().enumerate() {
        *b = (i as u8).wrapping_mul(37).wrapping_add(11);
    }

    // Async: write and read concurrently, since the receive FIFO holds only 8 bytes.
    {
        let uart = Lpuart::new(p.LPUART0, p.PTB17, p.PTB16, Irqs, Config::default());
        loopback();
        let (mut tx, mut rx) = uart.split();
        let mut got = [0u8; 64];
        let (written, read) = with_timeout(Duration::from_millis(200), join(tx.write(&pattern), rx.read(&mut got)))
            .await
            .expect("async loopback timed out");
        written.unwrap();
        read.unwrap();
        defmt::assert_eq!(got, pattern, "async loopback mismatch");
        tx.flush().await.unwrap();
        defmt::info!("async: 64 bytes ok");
    }

    // Blocking, on a fresh driver over the same instance and pins.
    {
        // The async driver above has been dropped, so stealing the singletons again is sound.
        let (lpuart, tx, rx) = unsafe {
            (
                peripherals::LPUART0::steal(),
                peripherals::PTB17::steal(),
                peripherals::PTB16::steal(),
            )
        };
        let mut config = Config::default();
        config.baudrate = 9600;
        let mut uart = Lpuart::new_blocking(lpuart, tx, rx, config);
        loopback();
        let mut got = [0u8; 64];
        // blocking_write returns once the bytes are queued, so go FIFO-sized chunk by chunk to
        // keep the 8-byte receive FIFO from overflowing.
        for (out, inp) in pattern.chunks(8).zip(got.chunks_mut(8)) {
            uart.blocking_write(out).unwrap();
            uart.blocking_read(inp).unwrap();
        }
        defmt::assert_eq!(got, pattern, "blocking loopback mismatch");
        defmt::info!("blocking: 64 bytes ok at 9600 baud");
    }

    defmt::info!("lpuart loopback passed");
    embassy_nxp_mkl82z7_examples::exit()
}
