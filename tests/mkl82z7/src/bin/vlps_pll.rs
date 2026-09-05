//! Runs from the 72 MHz PLL and enters VLPS whenever the executor is idle.
//!
//! Each of 32 timer wakes must restore PEE before the task resumes.
#![no_std]
#![no_main]

teleprobe_meta::target!(b"frdm-kl82z");

use core::fmt::Write as _;

use embassy_executor::Spawner;
use embassy_nxp::clocks::{ClockConfig, ExternalClock, ExternalSource};
use embassy_nxp::gpio::{Level, Output};
use embassy_nxp::lpuart::{self, Lpuart};
use embassy_nxp::pac::mcg::vals::Clkst;
use embassy_nxp::power::SleepMode;
use embassy_nxp::{Blocking, bind_interrupts, peripherals};
use embassy_nxp_mkl82z7_tests as _;
use embassy_time::{Instant, Timer};

bind_interrupts!(struct Irqs {
    LPUART0 => lpuart::InterruptHandler<peripherals::LPUART0>;
});

fn log(uart: &mut Lpuart<'_, Blocking>, args: core::fmt::Arguments<'_>) {
    let mut line = heapless::String::<128>::new();
    let _ = line.write_fmt(args);
    let _ = line.push_str("\r\n");
    uart.blocking_write(line.as_bytes()).unwrap();
    uart.blocking_flush().unwrap();
}

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
        power: embassy_nxp::power::Config {
            sleep_mode: SleepMode::VeryLowPowerStop,
        },
    };

    let p = embassy_nxp::init(config);
    let mut led = Output::new(p.PTC1, Level::High);
    let mut uart = Lpuart::new_blocking(p.LPUART0, p.PTB17, p.PTB16, lpuart::Config::default());
    let start = Instant::now();
    let mut wakes = 0u32;

    log(&mut uart, format_args!("72 MHz PEE; executor idle enters VLPS"));

    for _ in 0..32 {
        Timer::after_millis(20).await;

        let status = embassy_nxp::pac::MCG.s().read();
        assert_eq!(status.clkst(), Clkst::_11);
        assert!(status.lock0());

        wakes += 1;
        led.toggle();
        log(
            &mut uart,
            format_args!(
                "wake {} at {} ms: PLL locked, MCG in PEE",
                wakes,
                start.elapsed().as_millis()
            ),
        );
    }
    embassy_nxp_mkl82z7_tests::pass()
}
