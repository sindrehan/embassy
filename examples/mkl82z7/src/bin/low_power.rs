//! Power modes on the FRDM-KL82Z. Runs in VLPR (4 MHz from the fast IRC) with
//! VLPS as the idle sleep, blinks the red LED on embassy-time for a few
//! seconds, then enters LLS3 until SW3 (PTD0, LLWU_P12) is pressed or a 5 s
//! LPTMR timeout fires, and finally goes to VLLS3 with the same wake sources;
//! the wakeup from VLLS is a reset, which the start of `main` reports.
//!
//! The debugger loses the core while it sleeps in VLPS, LLS and VLLS; RTT
//! output resumes when it wakes. Use `probe-rs attach` afterwards, or read the
//! LED.
#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_nxp::clocks::ClockConfig;
use embassy_nxp::gpio::{Input, Level, Output, Pull};
use embassy_nxp::power::{self, LeakageMode, SleepMode, Wake, WakeEdge};
use embassy_nxp_mkl82z7_examples as _;
use embassy_time::{Duration, Timer};

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let mut config = embassy_nxp::config::Config::default();
    config.clocks = ClockConfig::vlpr();
    config.power.sleep_mode = SleepMode::VeryLowPowerStop;
    let p = embassy_nxp::init(config);

    if power::woke_from_vlls() {
        // Pins are still frozen from before the sleep; set them up, then release them.
        let _led = Output::new(unsafe { embassy_nxp::peripherals::PTC1::steal() }, Level::High);
        power::release_io_after_vlls();
        defmt::info!("woke from VLLS through a reset");
    }

    defmt::info!("low power: VLPR, idle sleep VLPS, clocks {:?}", embassy_nxp::clocks::clocks());
    let mut led = Output::new(p.PTC1, Level::High);
    let button = Input::new(p.PTD0, Pull::Up);

    // The executor sleeps in VLPS between these toggles; the TPM tick wakes it.
    for _ in 0..10 {
        led.toggle();
        Timer::after_millis(250).await;
    }
    led.set_high();

    defmt::info!("entering LLS3: press SW3 or wait 5 s");
    let reason = power::stop(
        LeakageMode::LowLeakageStop,
        &[
            Wake::Pin(&button, WakeEdge::Falling),
            Wake::Timeout(core::time::Duration::from_secs(5)),
        ],
    );
    defmt::info!("LLS3 ended: {:?}", reason);
    for _ in 0..4 {
        led.toggle();
        Timer::after_millis(100).await;
    }

    defmt::info!("entering VLLS3: press SW3 or wait 5 s, the wakeup is a reset");
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
