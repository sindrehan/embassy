//! Walks the idle sleep modes and checks that embassy-time keeps waking the
//! core in each: ten 250 ms timers should take 2500 ms. Logs over LPUART0
//! (OpenSDA serial port) because the debugger loses the core in the deep
//! modes. Use the test runner to check completion. Enable `vlpr` to run the
//! same walk from very low power run.
#![no_std]
#![no_main]

teleprobe_meta::target!(b"frdm-kl82z");

use core::fmt::Write as _;

use embassy_executor::Spawner;
use embassy_nxp::clocks::ClockConfig;
use embassy_nxp::gpio::{Level, Output};
use embassy_nxp::lpuart::{self, Lpuart};
use embassy_nxp::power::{self, SleepMode};
use embassy_nxp::{Blocking, pac};
use embassy_nxp_mkl82z7_tests as _;
use embassy_time::{Instant, Timer};

const VLPR: bool = cfg!(feature = "vlpr");

fn log(uart: &mut Lpuart<'_, Blocking>, args: core::fmt::Arguments<'_>) {
    let mut line = heapless::String::<160>::new();
    let _ = line.write_fmt(args);
    let _ = line.push_str("\r\n");
    uart.blocking_write(line.as_bytes()).unwrap();
    uart.blocking_flush().unwrap();
}

#[embassy_executor::main(executor = "embassy_nxp::executor::Executor", entry = "cortex_m_rt::entry")]
async fn main(_spawner: Spawner) {
    let mut config = embassy_nxp::config::Config::default();
    if VLPR {
        config.clocks = ClockConfig::vlpr();
    }
    let p = embassy_nxp::init(config);
    let mut led = Output::new(p.PTC1, Level::High);
    let mut uart = Lpuart::new_blocking(p.LPUART0, p.PTB17, p.PTB16, lpuart::Config::default());
    let c = embassy_nxp::clocks::clocks();
    log(
        &mut uart,
        format_args!(
            "sleep modes: vlpr={} core {} lpuart {} PMSTAT={:#04x}",
            VLPR,
            c.core,
            c.lpuart,
            pac::SMC.pmstat().read().pmstat()
        ),
    );

    for mode in [
        SleepMode::Wait,
        SleepMode::PartialStop2,
        SleepMode::PartialStop1,
        SleepMode::Stop,
        SleepMode::VeryLowPowerStop,
        SleepMode::Wait,
    ] {
        power::set_sleep_mode(mode);
        log(
            &mut uart,
            format_args!("mode {:?} (reads back {:?}): start", mode, power::sleep_mode()),
        );
        let start = Instant::now();
        for _ in 0..10 {
            led.toggle();
            Timer::after_millis(250).await;
        }
        assert!((2500..2600).contains(&start.elapsed().as_millis()));
        log(
            &mut uart,
            format_args!(
                "mode {:?}: 10 x 250 ms took {} ms, STOPA={}",
                mode,
                start.elapsed().as_millis(),
                pac::SMC.pmctrl().read().stopa()
            ),
        );
    }
    log(&mut uart, format_args!("sleep modes done"));
    embassy_nxp_mkl82z7_tests::pass()
}
