//! LPTMR-backed `embassy-time` in VLPR and VLPS.
//!
//! The short race exercises moving an armed alarm earlier. The 34-second run crosses the
//! 16-bit time counter's half-period checkpoint before the LED continues blinking once a second.
#![no_std]
#![no_main]

use core::fmt::Write as _;

use embassy_executor::Spawner;
use embassy_futures::select::{Either, select};
use embassy_nxp::Blocking;
use embassy_nxp::clocks::ClockConfig;
use embassy_nxp::gpio::{Level, Output};
use embassy_nxp::lpuart::{self, Lpuart};
use embassy_nxp::power::SleepMode;
use embassy_nxp_mkl82z7_examples as _;
use embassy_time::{Duration, Instant, Timer};

fn log(uart: &mut Lpuart<'_, Blocking>, args: core::fmt::Arguments<'_>) {
    let mut line = heapless::String::<128>::new();
    let _ = line.write_fmt(args);
    let _ = line.push_str("\r\n");
    uart.blocking_write(line.as_bytes()).unwrap();
    uart.blocking_flush().unwrap();
}

#[embassy_executor::main(executor = "embassy_nxp::executor::Executor", entry = "cortex_m_rt::entry")]
async fn main(_spawner: Spawner) {
    let mut config = embassy_nxp::config::Config::default();
    config.clocks = ClockConfig::vlpr();
    config.power.sleep_mode = SleepMode::VeryLowPowerStop;
    let p = embassy_nxp::init(config);

    let mut led = Output::new(p.PTC1, Level::High);
    let mut uart = Lpuart::new_blocking(p.LPUART0, p.PTB17, p.PTB16, lpuart::Config::default());
    log(&mut uart, format_args!("LPTMR time driver: VLPR, idle VLPS"));

    let race_start = Instant::now();
    match select(Timer::after_millis(200), Timer::after_millis(20)).await {
        Either::First(_) => panic!("later timer fired first"),
        Either::Second(_) => {}
    }
    let race_elapsed = race_start.elapsed().as_millis();
    assert!((20..50).contains(&race_elapsed));
    log(&mut uart, format_args!("earlier alarm fired after {} ms", race_elapsed));

    let checkpoint_start = Instant::now();
    for _ in 0..34 {
        Timer::after_secs(1).await;
        led.toggle();
    }
    let checkpoint_elapsed = checkpoint_start.elapsed().as_millis();
    assert!((34_000..34_200).contains(&checkpoint_elapsed));
    log(&mut uart, format_args!("34 timer wakes took {} ms", checkpoint_elapsed));

    loop {
        led.toggle();
        Timer::after(Duration::from_secs(1)).await;
    }
}
