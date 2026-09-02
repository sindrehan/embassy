//! Peripheral clock gating.
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
