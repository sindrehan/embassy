//! Watchdog configuration, refresh, WAIT/VLPS policy and explicit disable.
#![no_std]
#![no_main]

teleprobe_meta::target!(b"frdm-kl82z");

use embassy_executor::Spawner;
use embassy_nxp::lpuart::{self, Lpuart};
use embassy_nxp::pac;
use embassy_nxp::power::{self, SleepMode};
use embassy_nxp::wdog::{Config, Error, Watchdog};
use embassy_nxp_mkl82z7_tests as _;
use embassy_time::Timer;

fn count() -> u32 {
    loop {
        let high = pac::WDOG.tmrouth().read().0;
        let low = pac::WDOG.tmroutl().read().0;
        if high == pac::WDOG.tmrouth().read().0 {
            return (u32::from(high) << 16) | u32::from(low);
        }
    }
}

#[embassy_executor::main(executor = "embassy_nxp::executor::Executor", entry = "cortex_m_rt::entry")]
async fn main(_spawner: Spawner) {
    let mut hal_config = embassy_nxp::config::Config::default();
    if cfg!(feature = "vlpr") {
        hal_config.clocks = embassy_nxp::clocks::ClockConfig::vlpr();
        // Exercise configuration windows at the lowest supported core and bus frequencies.
        hal_config.clocks.core_div = 16;
        hal_config.clocks.bus_div = 16;
        hal_config.clocks.flash_div = 16;
        hal_config.clocks.qspi_div = 16;
    }
    let p = embassy_nxp::init(hal_config);
    // Give the flashing process time to release debug power requests before testing STOP.
    Timer::after_millis(1000).await;
    assert!(!cortex_m::peripheral::DCB::is_debugger_attached());
    let mut uart = Lpuart::new_blocking(p.LPUART0, p.PTB17, p.PTB16, lpuart::Config::default());
    let mut config = Config::default();
    config.timeout_ticks = 250;
    let mut wdog = Watchdog::new(p.WDOG, config).unwrap();
    assert!(wdog.is_enabled());
    assert_eq!(pac::WDOG.tovall().read().0, 250);
    for _ in 0..8 {
        wdog.feed();
        Timer::after_millis(100).await;
        assert!(count() > 50 && count() < 180);
    }
    let old = pac::WDOG.stctrlh().read();
    let mut invalid = config;
    invalid.timeout_ticks = 0;
    assert_eq!(wdog.configure(invalid), Err(Error::TimeoutTooShort));
    assert_eq!(pac::WDOG.stctrlh().read().0, old.0);

    config.timeout_ticks = 64;
    wdog.configure(config).unwrap();
    for _ in 0..8 {
        wdog.feed();
        Timer::after_millis(10).await;
    }
    config.timeout_ticks = 0x1_00fa;
    wdog.configure(config).unwrap();
    assert_eq!(pac::WDOG.tovalh().read().0, 1);
    assert_eq!(pac::WDOG.tovall().read().0, 250);
    config.timeout_ticks = 250;

    config.run_in_wait = false;
    wdog.configure(config).unwrap();
    let before = count();
    Timer::after_millis(100).await;
    assert!(count().abs_diff(before) < 20);
    config.run_in_wait = true;
    wdog.configure(config).unwrap();
    power::set_sleep_mode(SleepMode::VeryLowPowerStop);
    let before = count();
    Timer::after_millis(100).await;
    cortex_m::asm::delay(embassy_nxp::clocks::clocks().core / 1000);
    assert!(count().abs_diff(before) < 30);
    config.run_in_stop = true;
    wdog.configure(config).unwrap();
    Timer::after_millis(100).await;
    cortex_m::asm::delay(embassy_nxp::clocks::clocks().core / 1000);
    assert!(count() > 50);
    wdog.disable().unwrap();
    assert!(!wdog.is_enabled());
    Timer::after_millis(600).await;
    assert_eq!(count(), 0);
    wdog.enable().unwrap();
    assert!(wdog.is_enabled());
    wdog.feed();
    wdog.disable().unwrap();
    wdog.lock().unwrap();
    assert_eq!(wdog.enable(), Err(Error::Locked));
    uart.blocking_write(b"WATCHDOG OK\r\n").unwrap();
    uart.blocking_flush().unwrap();
    loop {
        cortex_m::asm::nop();
    }
}
