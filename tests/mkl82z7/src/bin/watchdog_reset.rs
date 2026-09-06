//! Verify an unrefreshed watchdog resets the MCU; verdict is reported over UART.
#![no_std]
#![no_main]

teleprobe_meta::target!(b"frdm-kl82z");

use embassy_executor::Spawner;
use embassy_nxp::lpuart::{self, Lpuart};
use embassy_nxp::wdog::{Config, Watchdog};
use embassy_nxp_mkl82z7_tests as _;
use embassy_time::Timer;

#[embassy_executor::main(executor = "embassy_nxp::executor::Executor", entry = "cortex_m_rt::entry")]
async fn main(_spawner: Spawner) {
    let watchdog_reset = embassy_nxp::pac::RCM.srs0().read().wdog();
    let p = embassy_nxp::init(Default::default());
    let mut uart = Lpuart::new_blocking(p.LPUART0, p.PTB17, p.PTB16, lpuart::Config::default());
    if watchdog_reset {
        // The host accepts this only after observing the arming message from this run.
        uart.blocking_write(b"WDOG RESET OK\r\n").unwrap();
        uart.blocking_flush().unwrap();
        loop {
            cortex_m::asm::nop();
        }
    }
    Timer::after_millis(1000).await;
    assert!(!cortex_m::peripheral::DCB::is_debugger_attached());
    let mut config = Config::default();
    config.timeout_ticks = 500;
    let mut watchdog = Watchdog::new(p.WDOG, config).unwrap();
    uart.blocking_write(b"WDOG ARMED\r\n").unwrap();
    uart.blocking_flush().unwrap();
    watchdog.feed();
    loop {
        cortex_m::asm::nop();
    }
}
