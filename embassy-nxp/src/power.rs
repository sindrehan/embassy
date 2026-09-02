//! Power modes (Kinetis SMC, LLWU, LPTMR).
//!
//! Three layers:
//!
//! - **Run modes.** RUN, HSRUN (entered by [`clocks`](crate::clocks) when the core clock needs it)
//!   and VLPR, selected through [`ClockConfig::run_mode`](crate::clocks::ClockConfig) because VLPR
//!   restricts the clocks: BLPI on the fast IRC, core and bus at most 4 MHz, flash at most 1 MHz.
//! - **Idle sleep.** [`Config::sleep_mode`] picks what the executor's idle `WFE` enters: WAIT,
//!   a partial stop, STOP or VLPS. The time driver keeps ticking in all of them (its TPM clock,
//!   the fast IRC, is kept running in stop), so `embassy-time` wakes the core as usual. Bus
//!   peripherals only keep working while asleep in WAIT and partial stop 2; in STOP and VLPS
//!   their clocks stop and, for example, incoming LPUART bytes are lost until the next wake.
//!   The debugger may lose its connection while the core is in STOP or VLPS.
//! - **Low leakage.** [`stop`] enters LLS or VLLS with LLWU pin and LPTMR timeout wake sources.
//!   `embassy-time` does not advance while in these modes. VLLS exits through a reset; see
//!   [`woke_from_vlls`] and [`release_io_after_vlls`].

use core::time::Duration;

use embassy_hal_internal::interrupt::InterruptExt;

use crate::clocks::{ClockConfig, McgMode, RunMode};
use crate::gpio::{Bank, Input};
use crate::pac::llwu::vals::Wupe;
use crate::pac::lptmr::vals::Pcs;
use crate::pac::smc::vals::{Llsm, Pstopo, Runm, Stopm};
use crate::pac::{Interrupt, LLWU, LPTMR0, MCG, PMC, RCM, SMC};
use crate::peripherals;

/// What the core enters when the executor idles.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum SleepMode {
    /// Core clock gated, everything else running. VLPW when in VLPR.
    #[default]
    Wait,
    /// System clock stopped, bus clock running: peripherals and DMA keep working.
    PartialStop2,
    /// System and bus clocks stopped, clock sources running: fast wakeup.
    PartialStop1,
    /// Normal STOP. The fast IRC (time driver) and, in PEE, the PLL are kept running.
    Stop,
    /// Very low power stop. Not available with a PLL clock configuration.
    VeryLowPowerStop,
}

/// Power configuration.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Config {
    pub sleep_mode: SleepMode,
}

/// The low leakage modes of [`stop`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum LeakageMode {
    /// LLS3: full RAM retention, returns from [`stop`] on wake.
    LowLeakageStop,
    /// VLLS3: full RAM retention, exits through a wakeup reset.
    Vlls3,
    /// VLLS2: partial RAM retention, exits through a wakeup reset.
    Vlls2,
    /// VLLS1: no RAM retention, exits through a wakeup reset.
    Vlls1,
    /// VLLS0: as VLLS1 with the 1 kHz LPO off, so only pins can wake it.
    Vlls0,
}

/// Edge on an LLWU pin that ends a low leakage stop.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum WakeEdge {
    Rising,
    Falling,
    Any,
}

/// A wake source for [`stop`].
pub enum Wake<'a> {
    /// An edge on an LLWU capable pin (see the chip's LLWU_Pn assignments). Panics for others.
    Pin(&'a Input<'a>, WakeEdge),
    /// LPTMR0 on the 1 kHz LPO, 1 ms to 65535 ms. Not available in VLLS0.
    Timeout(Duration),
}

/// Why a low leakage stop ended.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum WakeReason {
    /// The LLWU pin with this input number.
    Pin(u8),
    /// The LPTMR timeout.
    Timeout,
    /// Something else, for example another module routed to the LLWU.
    Other,
}

/// Allow every power mode. The protection register is write-once after reset, so this happens
/// exactly once, before anything decides on a mode.
pub(crate) fn init_protection() {
    SMC.pmprot().write(|w| {
        w.set_avlp(true);
        w.set_alls(true);
        w.set_avlls(true);
        w.set_ahsrun(true);
    });
}

/// Applies the idle sleep mode and, last of all, VLPR.
pub(crate) fn init(config: &Config, clocks: &ClockConfig) {
    set_sleep_mode(config.sleep_mode, clocks);

    if clocks.run_mode == RunMode::VeryLowPower {
        // Everything that touches the MCG or the clock dividers has run by now; from here on
        // the clock tree must stay as it is.
        SMC.pmctrl().modify(|w| w.set_runm(Runm::_10));
        // PMSTAT: 0x04 = VLPR.
        while SMC.pmstat().read().pmstat() != 0x04 {}
        debug!("Entered VLPR");
    }
}

fn set_sleep_mode(mode: SleepMode, clocks: &ClockConfig) {
    let deep = mode != SleepMode::Wait;
    if mode == SleepMode::VeryLowPowerStop {
        assert!(
            !matches!(clocks.mcg, McgMode::Pee { .. }),
            "VLPS is not available with a PLL clock configuration: the MCG drops to PBE on exit"
        );
    }

    // Keep the selected IRC (the time driver's clock) and, in PEE, the PLL alive through stop.
    MCG.c1().modify(|w| w.set_irefsten(deep));
    if matches!(clocks.mcg, McgMode::Pee { .. }) {
        MCG.c5().modify(|w| w.set_pllsten(deep));
    }

    let (stopm, pstopo) = match mode {
        SleepMode::Wait | SleepMode::Stop => (Stopm::_000, Pstopo::_00),
        SleepMode::PartialStop1 => (Stopm::_000, Pstopo::_01),
        SleepMode::PartialStop2 => (Stopm::_000, Pstopo::_10),
        SleepMode::VeryLowPowerStop => (Stopm::_010, Pstopo::_00),
    };
    SMC.stopctrl().modify(|w| w.set_pstopo(pstopo));
    SMC.pmctrl().modify(|w| w.set_stopm(stopm));
    // The SMC wants the write to land before a WFI/WFE.
    let _ = SMC.pmctrl().read();
    set_sleepdeep(deep);
}

fn set_sleepdeep(deep: bool) {
    // SCB_SCR[SLEEPDEEP], bit 2.
    let scb = unsafe { &*cortex_m::peripheral::SCB::PTR };
    unsafe {
        scb.scr.modify(|v| if deep { v | (1 << 2) } else { v & !(1 << 2) });
    }
}

/// LLWU input number of a pin, from the generated table.
fn llwu_input(bank: Bank, pin: u8) -> Option<u8> {
    crate::chip::LLWU_PINS
        .iter()
        .find(|(b, p, _)| *b == bank && *p == pin)
        .map(|(_, _, input)| *input)
}

/// Enter a low leakage stop until one of `wake` fires.
///
/// LLS returns with the reason. The VLLS modes exit through a wakeup reset and never return;
/// after that reset [`woke_from_vlls`] is true and the pins stay frozen until
/// [`release_io_after_vlls`]. Not available with a PLL clock configuration.
///
/// The executor is not running while this blocks, and `embassy-time` does not advance.
pub fn stop(mode: LeakageMode, wake: &[Wake<'_>]) -> WakeReason {
    let sleep_before = (SMC.pmctrl().read().stopm(), SMC.stopctrl().read().pstopo());
    let (stopm, llsm) = match mode {
        LeakageMode::LowLeakageStop => (Stopm::_011, Llsm::_011),
        LeakageMode::Vlls3 => (Stopm::_100, Llsm::_011),
        LeakageMode::Vlls2 => (Stopm::_100, Llsm::_010),
        LeakageMode::Vlls1 => (Stopm::_100, Llsm::_001),
        LeakageMode::Vlls0 => (Stopm::_100, Llsm::_000),
    };
    assert!(
        MCG.s().read().clkst() != crate::pac::mcg::vals::Clkst::_11,
        "low leakage stop is not available with a PLL clock configuration"
    );

    // Wake sources.
    let mut timeout = false;
    for source in wake {
        match source {
            Wake::Pin(input, edge) => {
                let input_number = llwu_input(input.pin.pin_bank(), input.pin.pin_number())
                    .unwrap_or_else(|| panic!("pin is not an LLWU wakeup input"));
                let wupe = match edge {
                    WakeEdge::Rising => Wupe::_01,
                    WakeEdge::Falling => Wupe::_10,
                    WakeEdge::Any => Wupe::_11,
                };
                LLWU.pe(input_number as usize / 4)
                    .modify(|w| w.set_wupe(input_number as usize % 4, wupe));
            }
            Wake::Timeout(duration) => {
                assert!(mode != LeakageMode::Vlls0, "the LPTMR has no clock in VLLS0");
                let ms = duration.as_millis().clamp(1, u16::MAX as u128) as u16;
                crate::clocks::enable::<peripherals::LPTMR0>();
                LPTMR0.csr().write(|_| {});
                LPTMR0.psr().write(|w| {
                    // Clock 1 is the 1 kHz LPO; bypass the prescaler for 1 ms ticks.
                    w.set_pcs(Pcs::_01);
                    w.set_pbyp(true);
                });
                LPTMR0.cmr().write(|w| w.set_compare(ms));
                LPTMR0.csr().write(|w| {
                    w.set_tie(true);
                    w.set_ten(true);
                });
                LLWU.me().modify(|w| w.set_wume(0, true));
                timeout = true;
            }
        }
    }
    // Clear stale pin flags (write 1 to clear).
    for i in 0..4 {
        LLWU.pf(i).write(|w| w.0 = 0xFF);
    }

    SMC.stopctrl().modify(|w| {
        w.set_llsm(llsm);
        w.set_pstopo(Pstopo::_00);
    });
    SMC.pmctrl().modify(|w| w.set_stopm(stopm));
    let _ = SMC.pmctrl().read();
    set_sleepdeep(true);

    // Sleep with interrupts masked: the LLWU interrupt still ends the stop (WFI wakes on any
    // enabled pending interrupt) but its handler does not run, so the flags are read here.
    let reason = critical_section::with(|_| {
        Interrupt::LLWU.unpend();
        unsafe { Interrupt::LLWU.enable() };
        cortex_m::asm::dsb();
        cortex_m::asm::wfi();
        cortex_m::asm::isb();

        let mut reason = WakeReason::Other;
        for i in 0..4 {
            let flags = LLWU.pf(i).read();
            for bit in 0..8 {
                if flags.wuf(bit) {
                    reason = WakeReason::Pin((i * 8 + bit) as u8);
                }
            }
            LLWU.pf(i).write(|w| w.0 = 0xFF);
        }
        if LLWU.mf5().read().mwuf(0) {
            reason = WakeReason::Timeout;
        }
        Interrupt::LLWU.disable();
        Interrupt::LLWU.unpend();
        reason
    });

    // Disarm everything and restore the idle sleep configuration.
    for i in 0..8 {
        LLWU.pe(i).write(|_| {});
    }
    LLWU.me().write(|_| {});
    if timeout {
        LPTMR0.csr().write(|_| {});
    }
    SMC.stopctrl().modify(|w| w.set_pstopo(sleep_before.1));
    SMC.pmctrl().modify(|w| w.set_stopm(sleep_before.0));
    let _ = SMC.pmctrl().read();
    set_sleepdeep(sleep_before.0 != Stopm::_000 || sleep_before.1 != Pstopo::_00);

    reason
}

/// Whether the last reset was the wakeup from a VLLS mode.
pub fn woke_from_vlls() -> bool {
    RCM.srs0().read().wakeup()
}

/// After a VLLS wakeup reset the pins stay frozen in their pre-sleep state until this is called.
/// Configure the pins as needed first, then release them.
pub fn release_io_after_vlls() {
    if PMC.regsc().read().ackiso() {
        PMC.regsc().modify(|w| w.set_ackiso(true));
    }
}
