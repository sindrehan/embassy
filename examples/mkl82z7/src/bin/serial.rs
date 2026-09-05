//! Echoes bytes on the OpenSDA virtual serial port at 115200 baud, 8N1.
//! LPUART0 uses PTB17 (TX / D1) and PTB16 (RX / D0).
#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_nxp::lpuart::{Config, InterruptHandler, Lpuart};
use embassy_nxp::{bind_interrupts, peripherals};
use embassy_nxp_mkl82z7_examples as _;

bind_interrupts!(struct Irqs {
    LPUART0 => InterruptHandler<peripherals::LPUART0>;
});

#[embassy_executor::main(executor = "embassy_nxp::executor::Executor", entry = "cortex_m_rt::entry")]
async fn main(_spawner: Spawner) {
    let p = embassy_nxp::init(Default::default());
    let mut uart = Lpuart::new(p.LPUART0, p.PTB17, p.PTB16, Irqs, Config::default());
    uart.write(b"KL82 echo ready\r\n").await.unwrap();

    let mut byte = [0; 1];
    loop {
        uart.read(&mut byte).await.unwrap();
        uart.write(&byte).await.unwrap();
    }
}
