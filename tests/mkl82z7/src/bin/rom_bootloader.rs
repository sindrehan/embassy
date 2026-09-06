//! ROM entry fixture. The host verifies its UART ping response, not a breakpoint.
#![no_std]
#![no_main]

teleprobe_meta::target!(b"frdm-kl82z");

use embassy_executor::Spawner;
use embassy_nxp::clocks::{ClockConfig, ExternalClock, ExternalSource};
use embassy_nxp::lpuart::{self, Lpuart};
use embassy_nxp::wdog::{Config, Watchdog};
use embassy_nxp_mkl82z7_tests as _;
use embassy_time::Timer;

#[used]
#[unsafe(link_section = ".bootloader_config")]
static BOOTLOADER_CONFIG: [u8; 64] = {
    let mut b = [0xff; 64];
    b[0] = b'k';
    b[1] = b'c';
    b[2] = b'f';
    b[3] = b'g';
    b[0x10] = 1; // UART only; leave SPI and I2C fixtures undriven.
    b[0x1c] = 0xfe; // Use the ROM's high-speed clock configuration.
    b[0x1d] = !2; // Divide the 48 MHz clock by two.
    b
};

#[embassy_executor::main(executor = "embassy_nxp::executor::Executor", entry = "cortex_m_rt::entry")]
async fn main(_spawner: Spawner) {
    let config = embassy_nxp::config::Config {
        clocks: ClockConfig::pll(
            ExternalClock {
                frequency: 12_000_000,
                source: ExternalSource::Crystal {
                    high_gain: false,
                    load_capacitance_pf: 0,
                },
            },
            1,
            24,
        ),
        ..Default::default()
    };
    let p = embassy_nxp::init(config);
    let mut watchdog = Watchdog::new(p.WDOG, Config::default()).unwrap();
    let mut uart = Lpuart::new_blocking(p.LPUART0, p.PTB17, p.PTB16, lpuart::Config::default());
    Timer::after_millis(100).await;
    uart.blocking_write(b"ROM entry\r\n").unwrap();
    uart.blocking_flush().unwrap();
    drop(uart);
    watchdog.disable().unwrap();
    // No DMA or other bus masters are active. RUN/PLL, privileged main-stack thread mode;
    // only UART interfaces are enabled in the BCA, with the UART pins released above.
    unsafe { embassy_nxp::rom::enter_bootloader() }
}
