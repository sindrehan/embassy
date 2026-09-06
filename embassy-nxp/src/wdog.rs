//! Kinetis watchdog using the independent 1 kHz low-power oscillator.
//!
//! The runtime disables the watchdog before RAM initialization. This driver enables it with
//! explicit WAIT, STOP/VLPS and debugger policies. LLS pauses the watchdog; VLLS powers it off.
//! An enabled watchdog always counts in RUN.
//! In STOP/VLPS, the backup reset circuit requires two timeouts before resetting the chip.
//! Timeout durations inherit the LPO's frequency tolerance (900–1100 us per tick on MKL82).
//!
//! Configuration and enable/disable operations block while updates synchronize. The supported
//! timeout starts at 64 LPO ticks, leaving margin for this synchronization. No prescaling,
//! windowed refresh, interrupt-before-reset or functional test mode is enabled.

use crate::pac::WDOG;
use crate::pac::wdog::regs::Stctrlh;
use crate::{Peri, peripherals};

/// Watchdog operation policy.
#[non_exhaustive]
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Config {
    /// Timeout in nominal milliseconds (1 kHz LPO ticks), at least 64.
    pub timeout_ticks: u32,
    /// Count while the CPU is in WAIT or VLPW.
    pub run_in_wait: bool,
    /// Count in STOP/VLPS. Reset there takes two timeout periods.
    pub run_in_stop: bool,
    /// Count while the core is halted by a debugger.
    pub run_in_debug: bool,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            timeout_ticks: 2500,
            run_in_wait: true,
            run_in_stop: false,
            run_in_debug: false,
        }
    }
}

/// Configuration error. Invalid configuration leaves the hardware unchanged.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    /// The timeout is too short for configuration synchronization.
    TimeoutTooShort,
    /// Hardware configuration was locked until reset.
    Locked,
}

impl core::fmt::Display for Error {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        core::fmt::Debug::fmt(self, f)
    }
}
impl core::error::Error for Error {}

/// An owned watchdog. Dropping it does not stop it or refresh it.
///
/// Keep this value alive and call [`feed`](Self::feed) before the timeout. Use
/// [`disable`](Self::disable) explicitly when stopping supervision is intended.
#[must_use]
pub struct Watchdog<'d> {
    _peri: Peri<'d, peripherals::WDOG>,
    config: Config,
    enabled: bool,
}

fn control(config: &Config, enabled: bool, updates: bool) -> Stctrlh {
    let mut ctrl = Stctrlh(0x100); // Preserve the reset value of reserved bit 8.
    ctrl.set_wdogen(enabled);
    ctrl.set_allowupdate(updates);
    ctrl.set_waiten(config.run_in_wait);
    ctrl.set_stopen(config.run_in_stop);
    ctrl.set_dbgen(config.run_in_debug);
    ctrl.set_distestwdog(true);
    ctrl
}

impl<'d> Watchdog<'d> {
    /// Configure and start the watchdog. Requires HAL initialization and runtime pre-init.
    /// Feed any already-running watchdog before calling; updating does not pause its old timer.
    pub fn new(peri: Peri<'d, peripherals::WDOG>, config: Config) -> Result<Self, Error> {
        let mut this = Self {
            _peri: peri,
            config,
            enabled: false,
        };
        this.configure(config)?;
        Ok(this)
    }

    /// Reconfigure and enable the watchdog, restarting its timeout.
    /// Blocks for configuration synchronization; do not call close to an existing timeout.
    pub fn configure(&mut self, config: Config) -> Result<(), Error> {
        if config.timeout_ticks < 64 {
            return Err(Error::TimeoutTooShort);
        }
        self.update(config, true, true)?;
        self.config = config;
        self.enabled = true;
        Ok(())
    }

    /// Restart the timeout. Does nothing when explicitly disabled.
    pub fn feed(&mut self) {
        if self.enabled {
            critical_section::with(|_| {
                // Consecutive halfword stores guarantee the 20-bus-cycle refresh window,
                // including unoptimized builds.
                #[cfg(target_arch = "arm")]
                unsafe {
                    core::arch::asm!(
                        "strh {first}, [{reg}]",
                        "strh {second}, [{reg}]",
                        reg = in(reg) WDOG.refresh().as_ptr(),
                        first = in(reg) 0xa602u32,
                        second = in(reg) 0xb480u32,
                        options(nostack, preserves_flags),
                    );
                }
            });
        }
    }

    /// Stop the watchdog and wait for the disable to synchronize. Ownership is retained.
    pub fn disable(&mut self) -> Result<(), Error> {
        self.update(self.config, false, true)?;
        self.enabled = false;
        Ok(())
    }

    /// Re-enable the saved configuration and restart the timeout.
    pub fn enable(&mut self) -> Result<(), Error> {
        self.configure(self.config)
    }

    /// Prevent further configuration changes until reset. Feeding remains available.
    pub fn lock(&mut self) -> Result<(), Error> {
        self.update(self.config, self.enabled, false)
    }

    /// Whether this driver has enabled the watchdog.
    pub fn is_enabled(&self) -> bool {
        self.enabled
    }

    fn update(&mut self, config: Config, enabled: bool, updates: bool) -> Result<(), Error> {
        if !WDOG.stctrlh().read().allowupdate() {
            return Err(Error::Locked);
        }
        let _guard = crate::power::wake_guard();
        let ctrl = control(&config, enabled, updates).0;
        // Supply every value before unlocking. No compiler-generated work is allowed in WCT.
        #[cfg(target_arch = "arm")]
        critical_section::with(|_| unsafe {
            core::arch::asm!(
                "strh r3, [r0, #14]",
                "strh r4, [r0, #14]",
                "ldrh r5, [r0, #20]", // One bus access before updating configuration.
                "lsrs r5, r1, #16",
                "strh r5, [r0, #4]",
                "strh r1, [r0, #6]",
                "movs r5, #0",
                "strh r5, [r0, #8]",
                "strh r5, [r0, #10]",
                "strh r5, [r0, #22]",
                "strh r2, [r0]",
                in("r0") WDOG.as_ptr(),
                in("r1") config.timeout_ticks,
                in("r2") u32::from(ctrl),
                in("r3") 0xc520u32,
                in("r4") 0xd928u32,
                out("r5") _,
                options(nostack),
            );
        });
        #[cfg(not(target_arch = "arm"))]
        let _ = ctrl;

        // Each read spans at least one bus cycle. Keep this loop short even in unoptimized
        // builds at the lowest supported core clock; refresh/unlock during WCT is ignored.
        #[cfg(target_arch = "arm")]
        unsafe {
            core::arch::asm!(
                "2:",
                "ldrh r2, [r0, #20]",
                "subs r1, #1",
                "bne 2b",
                in("r0") WDOG.as_ptr(),
                inout("r1") 256u32 => _,
                out("r2") _,
                options(nostack),
            );
        }
        // KL82 RM 28.10: allow clock switching and reload synchronization after WCT.
        // Eight maximum LPO periods (data sheet Table 42: 1100 us) also cover disable/enable.
        let cycles = (u64::from(crate::clocks::clocks().core) * 8800).div_ceil(1_000_000);
        cortex_m::asm::delay(cycles as u32);
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn policy_bits() {
        let config = Config::default();
        let ctrl = control(&config, true, true);
        assert!(ctrl.wdogen() && ctrl.waiten() && ctrl.allowupdate());
        assert!(!ctrl.clksrc() && !ctrl.stopen() && !ctrl.dbgen() && !ctrl.winen());
        assert!(ctrl.distestwdog());
        let config = Config {
            run_in_wait: false,
            run_in_stop: true,
            run_in_debug: true,
            ..config
        };
        let ctrl = control(&config, false, false);
        assert!(!ctrl.wdogen() && !ctrl.waiten() && !ctrl.allowupdate());
        assert!(ctrl.stopen() && ctrl.dbgen());
    }
}
