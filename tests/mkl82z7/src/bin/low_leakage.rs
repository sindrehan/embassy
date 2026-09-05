//! Checks LLS3 timeout wakeup and VLLS3 reset wakeup. Requires the TPM time
//! driver because LPTMR0 supplies the LLWU wake source.
#![no_std]
#![no_main]

teleprobe_meta::target!(b"frdm-kl82z");
teleprobe_meta::timeout!(30);

use embassy_executor::Spawner;
use embassy_nxp::clocks::ClockConfig;
use embassy_nxp::power::{self, LeakageMode, Wake, WakeReason};
use embassy_nxp_mkl82z7_tests as _;

#[embassy_executor::main(executor = "embassy_nxp::executor::Executor", entry = "cortex_m_rt::entry")]
async fn main(_spawner: Spawner) {
    let config = embassy_nxp::config::Config {
        clocks: ClockConfig::vlpr(),
        ..Default::default()
    };
    let mut p = embassy_nxp::init(config);

    if let Some(reason) = power::vlls_wake_reason(&mut p.LLWU, Some(&mut p.LPTMR0)) {
        power::release_io_after_vlls();
        defmt::assert_eq!(reason, WakeReason::Timeout);
        embassy_nxp_mkl82z7_tests::pass();
    }

    let wake = [Wake::Timeout(core::time::Duration::from_secs(2))];
    let reason = power::stop(&mut p.LLWU, Some(&mut p.LPTMR0), LeakageMode::LowLeakageStop, &wake);
    defmt::assert_eq!(reason, WakeReason::Timeout);
    power::stop(&mut p.LLWU, Some(&mut p.LPTMR0), LeakageMode::Vlls3, &wake);
    defmt::unreachable!("VLLS exit must reset the core");
}
