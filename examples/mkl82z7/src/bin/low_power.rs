//! Power modes on the FRDM-KL82Z. Runs in VLPR (4 MHz from the fast IRC) with
//! VLPS as the idle sleep, blinks the red LED on embassy-time for a few
//! seconds, then enters LLS3 until SW3 (PTD0, LLWU_P12) is pressed or a 5 s
//! LPTMR timeout fires, and finally goes to VLLS3 with the same wake sources;
//! the wakeup from VLLS is a reset, which the start of `main` reports.
//!
//! The debugger loses the core while it sleeps in VLPS, LLS and VLLS, and with
//! it the RTT output, so this example also logs over LPUART0 (PTB17/PTB16, the
//! OpenSDA serial port) at 115200, flushing before every sleep.
#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_nxp::clocks::ClockConfig;
use core::fmt::Write as _;

use embassy_nxp::gpio::{Input, Level, Output, Pull};
use embassy_nxp::lpuart::{self, Lpuart};
use embassy_nxp::power::{self, LeakageMode, SleepMode, Wake, WakeEdge};
use embassy_nxp::{Blocking, bind_interrupts, peripherals};
use embassy_nxp_mkl82z7_examples as _;
use embassy_time::{Duration, Timer};

bind_interrupts!(struct Irqs {
    LPUART0 => lpuart::InterruptHandler<peripherals::LPUART0>;
});

/// Log a line over the serial port and wait until it has left the chip.
fn log(uart: &mut Lpuart<'_, Blocking>, args: core::fmt::Arguments<'_>) {
    let mut line = heapless::String::<160>::new();
    let _ = line.write_fmt(args);
    let _ = line.push_str("\r\n");
    uart.blocking_write(line.as_bytes()).unwrap();
    uart.blocking_flush().unwrap();
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let mut config = embassy_nxp::config::Config::default();
    config.clocks = ClockConfig::vlpr();
    config.power.sleep_mode = SleepMode::VeryLowPowerStop;
    let p = embassy_nxp::init(config);
    let woke = power::woke_from_vlls();
    let mut led = Output::new(p.PTC1, Level::High);
    let button = Input::new(p.PTD0, Pull::Up);
    let mut uart = Lpuart::new_blocking(p.LPUART0, p.PTB17, p.PTB16, lpuart::Config::default());
    if woke {
        // Pins were frozen from before the sleep until now; they are set up again, so release.
        power::release_io_after_vlls();
        defmt::info!("woke from VLLS through a reset");
        log(&mut uart, format_args!("woke from VLLS through a reset, RCM says wakeup"));
    }

    let clocks = embassy_nxp::clocks::clocks();
    defmt::info!("low power: VLPR, idle sleep VLPS, clocks {:?}", clocks);
    log(
        &mut uart,
        format_args!(
            "low power: VLPR, idle VLPS, core {} Hz, bus {} Hz, flash {} Hz, lpuart {} Hz",
            clocks.core, clocks.bus, clocks.flash, clocks.lpuart
        ),
    );

    // The executor sleeps in VLPS between these toggles; the TPM tick wakes it.
    let start = embassy_time::Instant::now();
    for _ in 0..10 {
        led.toggle();
        Timer::after_millis(250).await;
    }
    led.set_high();
    log(&mut uart, format_args!("10 x 250 ms timer wakes from VLPS took {} ms", start.elapsed().as_millis()));

    defmt::info!("entering LLS3: press SW3 or wait 5 s");
    log(&mut uart, format_args!("entering LLS3: press SW3 or wait 5 s"));
    let reason = power::stop(
        LeakageMode::LowLeakageStop,
        &[
            Wake::Pin(&button, WakeEdge::Falling),
            Wake::Timeout(core::time::Duration::from_secs(5)),
        ],
    );
    defmt::info!("LLS3 ended: {:?}", reason);
    log(&mut uart, format_args!("LLS3 ended: {:?}", reason));
    for _ in 0..4 {
        led.toggle();
        Timer::after_millis(100).await;
    }

    defmt::info!("entering VLLS3: press SW3 or wait 5 s, the wakeup is a reset");
    log(&mut uart, format_args!("entering VLLS3: press SW3 or wait 5 s, the wakeup is a reset"));
    Timer::after(Duration::from_millis(50)).await;
    power::stop(
        LeakageMode::Vlls3,
        &[
            Wake::Pin(&button, WakeEdge::Falling),
            Wake::Timeout(core::time::Duration::from_secs(5)),
        ],
    );
    defmt::unreachable!("VLLS exit is a reset");
}
