//! LPUART0 echo on the FRDM-KL82Z: PTB17 = LPUART0_TX and PTB16 = LPUART0_RX
//! go to the OpenSDA virtual COM port (J-Link CDC, `/dev/ttyACM0` on Linux
//! once `cdc_acm` is loaded) and to Arduino D1/D0. 115200 8N1. Everything
//! received is echoed back and a heartbeat line goes out every second.
//!
//! For an external 3.3 V adapter on free header pins use LPUART1 instead:
//! PTC4 = LPUART1_TX (Arduino D10, J2 pin 6) to the adapter's RX and
//! PTC3 = LPUART1_RX (Arduino D6, J1 pin 14) to the adapter's TX.
#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_futures::select::{Either, select};
use embassy_nxp::lpuart::{Config, InterruptHandler, Lpuart};
use embassy_nxp::{bind_interrupts, peripherals};
use embassy_nxp_mkl82z7_examples as _;
use embassy_time::{Duration, Ticker};

bind_interrupts!(struct Irqs {
    LPUART0 => InterruptHandler<peripherals::LPUART0>;
});

#[embassy_executor::main(executor = "embassy_nxp::executor::Executor", entry = "cortex_m_rt::entry")]
async fn main(_spawner: Spawner) {
    let p = embassy_nxp::init(Default::default());
    defmt::info!("serial: LPUART0 on PTB17 (TX) / PTB16 (RX), 115200 8N1");

    let uart = Lpuart::new(p.LPUART0, p.PTB17, p.PTB16, Irqs, Config::default());
    let (mut tx, mut rx) = uart.split();
    tx.write(b"\r\nKL82 LPUART0 echo ready\r\n").await.unwrap();

    let mut ticker = Ticker::every(Duration::from_secs(1));
    let mut heartbeat = 0u32;
    let mut byte = [0u8; 1];
    loop {
        match select(rx.read(&mut byte), ticker.next()).await {
            Either::First(Ok(())) => {
                defmt::info!("rx {:#04x}", byte[0]);
                tx.write(&byte).await.unwrap();
            }
            Either::First(Err(e)) => defmt::warn!("rx error {:?}", e),
            Either::Second(()) => {
                heartbeat += 1;
                let mut line = heapless::String::<32>::new();
                use core::fmt::Write;
                write!(line, "heartbeat {}\r\n", heartbeat).unwrap();
                tx.write(line.as_bytes()).await.unwrap();
            }
        }
    }
}
