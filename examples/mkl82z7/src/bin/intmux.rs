//! Interrupts through INTMUX0: I2C1 and LPUART2 have no NVIC line of their
//! own, both are routed through channel 0 and share one `bind_interrupts!`
//! line. I2C1 (PTC10 SCL, PTC11 SDA, nothing attached) probes an empty address
//! and must get a NACK back through the interrupt; LPUART2 (PTD3 TX, PTD2 RX)
//! runs 64 bytes through internal loopback at 9600 baud. Both are given deadlines so a
//! missing interrupt shows up as a timeout instead of a hang.
//!
//! LPUART2's only pins are the accelerometer's I2C0 pins, so the sensor sees
//! some traffic while this runs; it recovers at its next START.
#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_nxp::i2c::{self, I2c};
use embassy_nxp::lpuart::{self, Lpuart};
use embassy_nxp::{bind_interrupts, pac, peripherals};
use embassy_nxp_mkl82z7_examples as _;
use embassy_time::{Duration, with_timeout};

bind_interrupts!(struct Irqs {
    INTMUX0_0 => i2c::InterruptHandler<peripherals::I2C1>, lpuart::InterruptHandler<peripherals::LPUART2>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_nxp::init(Default::default());
    defmt::info!("intmux: I2C1 and LPUART2 on INTMUX0 channel 0");

    let mut i2c = I2c::new(p.I2C1, p.PTC10, p.PTC11, Irqs, i2c::Config::default());
    let result = with_timeout(Duration::from_millis(100), i2c.write(0x7E, &[0]))
        .await
        .expect("I2C1 interrupt never arrived");
    defmt::assert_eq!(result, Err(i2c::Error::AddressNack));
    defmt::info!("I2C1: address NACK delivered through INTMUX0");

    // LPUART2 has a 1-byte receive buffer, so on the 21 MHz reset clock the per-byte interrupt
    // path only keeps up at modest baud rates; 9600 is comfortable.
    let mut config = lpuart::Config::default();
    config.baudrate = 9600;
    let uart = Lpuart::new(p.LPUART2, p.PTD3, p.PTD2, Irqs, config);
    let regs = pac::LPUART2;
    regs.ctrl().modify(|w| w.set_re(false));
    regs.ctrl().modify(|w| {
        w.set_loops(true);
        w.set_rsrc(false);
    });
    regs.fifo().modify(|w| w.set_rxflush(true));
    regs.ctrl().modify(|w| w.set_re(true));

    let (mut tx, mut rx) = uart.split();
    let mut pattern = [0u8; 64];
    for (i, b) in pattern.iter_mut().enumerate() {
        *b = (i as u8).wrapping_mul(29).wrapping_add(3);
    }
    let mut got = [0u8; 64];
    let (written, read) = with_timeout(Duration::from_millis(200), join(tx.write(&pattern), rx.read(&mut got)))
        .await
        .expect("LPUART2 interrupt never arrived");
    written.unwrap();
    read.unwrap();
    defmt::assert_eq!(got, pattern, "LPUART2 loopback mismatch");
    defmt::info!("LPUART2: 64 bytes looped back through INTMUX0");

    let vec = pac::INTMUX0.ch_vec(0).read().vecn();
    defmt::info!("INTMUX0 CH0 IER = {:#010x}, VEC = {}", pac::INTMUX0.ch_ier_31_0(0).read().0, vec);
    defmt::info!("intmux passed");
    embassy_nxp_mkl82z7_examples::exit()
}
