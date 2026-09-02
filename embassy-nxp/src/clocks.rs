//! Clocks: system clock configuration and peripheral clock gating.
//!
//! [`ClockConfig`] selects the MCG mode and the SIM dividers; [`init`](crate::init) applies it
//! and [`clocks`] reports the resulting frequencies.
//!
//! On Kinetis every peripheral has a clock gate bit in one of the `SIM_SCGCx` registers, and its
//! registers are inaccessible (a bus fault) while the gate is closed. The gate bit for each
//! peripheral singleton is generated from the `nxp-pac` metadata.
#![macro_use]

pub(crate) trait SealedClockGate {
    fn enable_clock();
    fn disable_clock();
    fn is_clock_enabled() -> bool;
}

/// A peripheral whose clock can be gated.
#[allow(private_bounds)]
pub trait ClockGate: SealedClockGate {}

/// Open the clock gate of peripheral `T`.
///
/// ```rust,ignore
/// embassy_nxp::clocks::enable::<peripherals::LPUART0>();
/// ```
#[inline]
pub fn enable<T: ClockGate>() {
    T::enable_clock();
}

/// Close the clock gate of peripheral `T`. Its registers must not be touched afterwards.
#[inline]
pub fn disable<T: ClockGate>() {
    T::disable_clock();
}

/// Whether the clock gate of peripheral `T` is open.
#[inline]
pub fn is_enabled<T: ClockGate>() -> bool {
    T::is_clock_enabled()
}

macro_rules! impl_clock_gate {
    ($name:ident, $reg:ident, $get:ident, $set:ident) => {
        impl crate::clocks::SealedClockGate for peripherals::$name {
            fn enable_clock() {
                // The SCGC registers are shared between peripherals: read-modify-write atomically.
                critical_section::with(|_| crate::pac::SIM.$reg().modify(|w| w.$set(true)));
            }

            fn disable_clock() {
                critical_section::with(|_| crate::pac::SIM.$reg().modify(|w| w.$set(false)));
            }

            fn is_clock_enabled() -> bool {
                crate::pac::SIM.$reg().read().$get()
            }
        }

        impl crate::clocks::ClockGate for peripherals::$name {}
    };
}

// ---------------------------------------------------------------------------------------------
// Clock configuration (MCG, OSC, SIM dividers)
// ---------------------------------------------------------------------------------------------

use core::cell::Cell;

use critical_section::Mutex;

use crate::pac::mcg::vals::{Clks, Clkst, DrstDrs, Fcrdiv, Frdiv, Range};
use crate::pac::sim::vals::{Outdiv1, Outdiv2, Outdiv4, Outdiv5, Pllfllsel};
use crate::pac::smc::vals::Runm;
use crate::pac::{MCG, OSC, SIM, SMC};

/// Slow internal reference clock.
const SLOW_IRC_HZ: u32 = 32_768;
/// Fast internal reference clock, before `FCRDIV`.
const FAST_IRC_HZ: u32 = 4_000_000;
/// FLL multiplier with `DRST_DRS = 0` and `DMX32 = 0` (the reset configuration).
const FLL_FACTOR_LOW: u32 = 640;
/// Core clock limits in Run and High Speed Run mode.
const RUN_MAX_CORE_HZ: u32 = 72_000_000;
const HSRUN_MAX_CORE_HZ: u32 = 96_000_000;
/// Bus and flash clock limit.
const MAX_BUS_HZ: u32 = 24_000_000;
/// 48 MHz internal reference, selected as the PLLFLLSEL peripheral clock.
const IRC48M_HZ: u32 = 48_000_000;

/// The source of the external reference clock on `EXTAL0`/`XTAL0`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ExternalSource {
    /// A crystal or resonator driven by the internal oscillator.
    Crystal {
        /// Run the oscillator in high gain mode instead of low power mode.
        high_gain: bool,
        /// Internal load capacitance in pF, an even number from 0 to 30.
        load_capacitance_pf: u8,
    },
    /// A square wave clock fed into `EXTAL0`.
    Clock,
}

/// The external reference clock.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct ExternalClock {
    /// Frequency in Hz.
    pub frequency: u32,
    pub source: ExternalSource,
}

/// The Multipurpose Clock Generator mode, which decides what `MCGOUTCLK` is.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum McgMode {
    /// FLL engaged internal: the FLL multiplies the 32.768 kHz slow internal reference by 640,
    /// giving 20.97 MHz. This is the reset configuration and needs no external components.
    Fei,
    /// PLL engaged external: `MCGOUTCLK = external / prdiv * vdiv / 2`.
    Pee {
        external: ExternalClock,
        /// PLL reference divider, 1 to 8. The divided reference must be 8 to 16 MHz.
        prdiv: u8,
        /// VCO multiplier, 16 to 47.
        vdiv: u8,
    },
    /// Bypassed low power internal: the 4 MHz fast internal reference drives `MCGOUTCLK`
    /// directly and the FLL and PLL are off.
    Blpi,
}

/// System clock configuration.
///
/// The bus, flash and QuadSPI clocks are `MCGOUTCLK` divided by their dividers. The reference
/// manual requires the core clock to be at most 72 MHz in Run mode (96 MHz in High Speed Run
/// mode, entered automatically), the bus and flash clocks to be at most 24 MHz and at most an
/// eighth of the core clock, and the flash clock to be at most the bus clock. Violations panic
/// in `init`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct ClockConfig {
    pub mcg: McgMode,
    /// Core and system clock divider (`OUTDIV1`), 1 to 16.
    pub core_div: u8,
    /// Bus clock divider (`OUTDIV2`), 1 to 16. Must be a multiple of `core_div`.
    pub bus_div: u8,
    /// Flash clock divider (`OUTDIV4`), 1 to 16. Must be a multiple of `core_div`.
    pub flash_div: u8,
    /// QuadSPI clock divider (`OUTDIV5`), 1 to 16.
    pub qspi_div: u8,
}

impl Default for ClockConfig {
    /// The reset configuration: FEI at 20.97 MHz core and bus, 10.49 MHz flash.
    fn default() -> Self {
        Self {
            mcg: McgMode::Fei,
            core_div: 1,
            bus_div: 1,
            flash_div: 2,
            qspi_div: 1,
        }
    }
}

impl ClockConfig {
    /// PLL from an external clock or crystal, with `MCGOUTCLK = frequency / prdiv * vdiv / 2`
    /// and the dividers picked for the fastest legal core, bus and flash clocks.
    ///
    /// For the 12 MHz crystal on the FRDM-KL82Z, `prdiv = 1` and `vdiv = 24` give a 144 MHz
    /// `MCGOUTCLK`, a 72 MHz core clock and 24 MHz bus and flash clocks.
    pub const fn pll(external: ExternalClock, prdiv: u8, vdiv: u8) -> Self {
        let mcgout = external.frequency / prdiv as u32 * vdiv as u32 / 2;
        let core_div = mcgout.div_ceil(RUN_MAX_CORE_HZ) as u8;
        let bus_div = {
            // Smallest multiple of core_div that keeps the bus at or below 24 MHz.
            let mut d = core_div;
            while mcgout / d as u32 > MAX_BUS_HZ {
                d += core_div;
            }
            d
        };
        Self {
            mcg: McgMode::Pee { external, prdiv, vdiv },
            core_div,
            bus_div,
            flash_div: bus_div,
            qspi_div: core_div,
        }
    }

    /// `MCGOUTCLK` in Hz for this configuration.
    pub const fn mcgout_hz(&self) -> u32 {
        match self.mcg {
            McgMode::Fei => SLOW_IRC_HZ * FLL_FACTOR_LOW,
            McgMode::Pee { external, prdiv, vdiv } => external.frequency / prdiv as u32 * vdiv as u32 / 2,
            McgMode::Blpi => FAST_IRC_HZ,
        }
    }

    /// The clock frequencies this configuration produces.
    pub const fn clocks(&self) -> Clocks {
        let mcgout = self.mcgout_hz();
        Clocks {
            mcgout,
            core: mcgout / self.core_div as u32,
            bus: mcgout / self.bus_div as u32,
            flash: mcgout / self.flash_div as u32,
            qspi: mcgout / self.qspi_div as u32,
            pllfll: IRC48M_HZ,
        }
    }

    fn validate(&self) {
        let c = self.clocks();
        let div_ok = |d: u8| (1..=16).contains(&d);
        assert!(
            div_ok(self.core_div) && div_ok(self.bus_div) && div_ok(self.flash_div) && div_ok(self.qspi_div),
            "clock dividers must be 1 to 16"
        );
        assert!(c.core <= HSRUN_MAX_CORE_HZ, "core clock above 96 MHz");
        assert!(c.bus <= MAX_BUS_HZ, "bus clock above 24 MHz");
        assert!(c.flash <= MAX_BUS_HZ && c.flash <= c.bus, "flash clock above 24 MHz or above the bus clock");
        assert!(
            self.bus_div % self.core_div == 0 && self.flash_div % self.core_div == 0,
            "bus and flash clocks must be integer divisions of the core clock"
        );
        assert!(
            self.bus_div / self.core_div <= 8 && self.flash_div / self.core_div <= 8,
            "core to bus and core to flash ratios are limited to 8"
        );
        if let McgMode::Pee { external, prdiv, vdiv } = self.mcg {
            assert!((1..=8).contains(&prdiv), "prdiv must be 1 to 8");
            assert!((16..=47).contains(&vdiv), "vdiv must be 16 to 47");
            let reference = external.frequency / prdiv as u32;
            assert!(
                (8_000_000..=16_000_000).contains(&reference),
                "PLL reference (external / prdiv) must be 8 to 16 MHz"
            );
            if let ExternalSource::Crystal { load_capacitance_pf, .. } = external.source {
                assert!(
                    load_capacitance_pf <= 30 && load_capacitance_pf % 2 == 0,
                    "load capacitance must be an even number of pF up to 30"
                );
            }
        }
    }
}

/// The clock frequencies in Hz, as configured by [`init`](crate::init).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Clocks {
    /// MCG output clock, the input to all the dividers.
    pub mcgout: u32,
    /// Core and system clock.
    pub core: u32,
    /// Bus clock.
    pub bus: u32,
    /// Flash clock.
    pub flash: u32,
    /// QuadSPI clock.
    pub qspi: u32,
    /// The `SIM_SOPT2[PLLFLLSEL]` peripheral clock offered to LPUART, TPM, FlexIO, EMVSIM and
    /// USB. `init` points it at the 48 MHz IRC48M.
    pub pllfll: u32,
}

static CLOCKS: Mutex<Cell<Clocks>> = Mutex::new(Cell::new(Clocks {
    mcgout: 0,
    core: 0,
    bus: 0,
    flash: 0,
    qspi: 0,
    pllfll: 0,
}));

/// The clock frequencies configured by [`init`](crate::init).
pub fn clocks() -> Clocks {
    critical_section::with(|cs| CLOCKS.borrow(cs).get())
}

pub(crate) fn init(config: ClockConfig) {
    config.validate();
    let clocks = config.clocks();

    let current = current_mcgout_hz();
    let high_speed = clocks.core > RUN_MAX_CORE_HZ;

    if high_speed {
        SMC.pmprot().modify(|w| w.set_ahsrun(true));
        SMC.pmctrl().modify(|w| w.set_runm(Runm::_11));
        // PMSTAT: 0x80 = HSRUN.
        while SMC.pmstat().read().pmstat() != 0x80 {}
    }

    // Dividers go in before speeding up and after slowing down, so no clock ever overshoots.
    if clocks.mcgout >= current {
        set_dividers(&config);
    }

    match config.mcg {
        McgMode::Fei => {}
        McgMode::Pee { external, prdiv, vdiv } => enter_pee(external, prdiv, vdiv),
        McgMode::Blpi => enter_blpi(),
    }

    if clocks.mcgout < current {
        set_dividers(&config);
    }

    // Selecting the IRC48M here also enables it. The fractional divider (CLKDIV3) is /1 at reset.
    critical_section::with(|_| SIM.sopt2().modify(|w| w.set_pllfllsel(Pllfllsel::_11)));

    critical_section::with(|cs| CLOCKS.borrow(cs).set(clocks));
    debug!("Clocks: {:?}", clocks);
}

/// `MCGOUTCLK` as the hardware has it right now. Only the reset configuration is expected
/// here, so anything but FEI is reported as the fast IRC.
fn current_mcgout_hz() -> u32 {
    match MCG.s().read().clkst() {
        Clkst::_00 => SLOW_IRC_HZ * FLL_FACTOR_LOW,
        _ => FAST_IRC_HZ,
    }
}

fn set_dividers(config: &ClockConfig) {
    SIM.clkdiv1().write(|w| {
        w.set_outdiv1(Outdiv1::from_bits(config.core_div - 1));
        w.set_outdiv2(Outdiv2::from_bits(config.bus_div - 1));
        w.set_outdiv4(Outdiv4::from_bits(config.flash_div - 1));
        w.set_outdiv5(Outdiv5::from_bits(config.qspi_div - 1));
    });
}

/// FEI -> FBE -> PBE -> PEE, following the reference manual's mode transition rules and the
/// MCUXpresso SDK's `CLOCK_BootToPeeMode`.
fn enter_pee(external: ExternalClock, prdiv: u8, vdiv: u8) {
    // Oscillator setup. RANGE: 0 = low (32 kHz), 1 = high (3 to 8 MHz), 2 = very high (8 to
    // 32 MHz); the PAC marks 2 as reserved because the SVD lacks its description.
    let range = if external.frequency <= 39_063 {
        0
    } else if external.frequency <= 8_000_000 {
        1
    } else {
        2
    };
    let crystal = match external.source {
        ExternalSource::Crystal {
            high_gain,
            load_capacitance_pf,
        } => {
            OSC.cr().modify(|w| {
                w.set_sc2p(load_capacitance_pf & 2 != 0);
                w.set_sc4p(load_capacitance_pf & 4 != 0);
                w.set_sc8p(load_capacitance_pf & 8 != 0);
                w.set_sc16p(load_capacitance_pf & 16 != 0);
                w.set_erclken(true);
            });
            Some(high_gain)
        }
        ExternalSource::Clock => {
            OSC.cr().modify(|w| w.set_erclken(true));
            None
        }
    };
    MCG.c2().modify(|w| {
        w.set_range(Range::from_bits(range));
        w.set_hgo(crystal == Some(true));
        w.set_erefs(crystal.is_some());
    });
    if crystal.is_some() {
        while !MCG.s().read().oscinit0() {}
    }

    // FBE. Errata ERR007993: flip the DRST_DRS LSB while the FLL reference changes from the
    // internal to the external clock, then restore it.
    let c4 = MCG.c4().read();
    MCG.c4()
        .modify(|w| w.set_drst_drs(DrstDrs::from_bits(c4.drst_drs().to_bits() ^ 1)));
    MCG.c1().modify(|w| {
        w.set_clks(Clks::_10);
        // The FLL is not used in PEE, so its reference divider does not matter.
        w.set_frdiv(Frdiv::_000);
        w.set_irefs(false);
    });
    while MCG.s().read().irefst() || MCG.s().read().clkst() != Clkst::_10 {}
    MCG.c4().write_value(c4);

    // PBE: configure and lock the PLL, then hand the FLL/PLL mux to the PLL.
    MCG.c6().modify(|w| w.set_plls(false));
    while MCG.s().read().pllst() {}
    MCG.c5().write(|w| w.set_prdiv(prdiv - 1));
    MCG.c6().modify(|w| w.set_vdiv(vdiv - 16));
    MCG.c5().modify(|w| w.set_pllclken(true));
    while !MCG.s().read().lock0() {}
    MCG.c6().modify(|w| w.set_plls(true));
    while !MCG.s().read().pllst() {}

    // PEE.
    MCG.c1().modify(|w| w.set_clks(Clks::_00));
    while MCG.s().read().clkst() != Clkst::_11 {}
}

/// FEI -> FBI -> BLPI on the 4 MHz fast IRC.
fn enter_blpi() {
    // FCRDIV may only change while the fast IRC is not in use; at reset it is not.
    MCG.sc().modify(|w| w.set_fcrdiv(Fcrdiv::_000));
    MCG.c2().modify(|w| w.set_ircs(true));
    MCG.c1().modify(|w| {
        w.set_clks(Clks::_01);
        w.set_irefs(true);
    });
    while !MCG.s().read().ircst() || MCG.s().read().clkst() != Clkst::_01 {}
    MCG.c2().modify(|w| w.set_lp(true));
}
