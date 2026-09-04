//! Interrupt multiplexer (INTMUX0).
//!
//! The Cortex-M0+ on the Kinetis L parts has 32 NVIC lines, and the peripherals that did not get
//! one reach the core through INTMUX0 instead: four channels, each ORing up to 32 sources into
//! one NVIC line (`INTMUX0_0` to `INTMUX0_3`).
//!
//! The drivers keep the usual typed interrupt binding: a multiplexed instance names the NVIC
//! channel interrupt as its [`Interrupt`](crate::interrupt::typelevel::Interrupt) and its
//! constructor opens the source bit here. All multiplexed instances share [`CHANNEL`], so several
//! drivers bind their handlers to the same line and each one checks its own peripheral's flags:
//!
//! ```rust,ignore
//! bind_interrupts!(struct Irqs {
//!     INTMUX0_0 => i2c::InterruptHandler<peripherals::I2C1>, lpuart::InterruptHandler<peripherals::LPUART2>;
//! });
//! ```

use core::sync::atomic::{AtomicBool, Ordering};

use crate::pac::INTMUX0;
use crate::peripherals;

/// The INTMUX channel used for every multiplexed peripheral, NVIC interrupt `INTMUX0_0`.
pub const CHANNEL: usize = 0;

static CLOCKED: AtomicBool = AtomicBool::new(false);

/// Route `source` (the peripheral's input number on the multiplexer) to `channel`.
///
/// The channel is left in its reset configuration: OR mode, interrupt request while any enabled
/// source is pending. The request clears once the source deasserts, so handlers need nothing
/// beyond servicing their peripheral.
pub(crate) fn enable_source(channel: usize, source: u8) {
    critical_section::with(|_| {
        // Inside the critical section a load and store are enough; thumbv6m has no swap.
        if !CLOCKED.load(Ordering::Relaxed) {
            CLOCKED.store(true, Ordering::Relaxed);
            crate::clocks::enable::<peripherals::INTMUX0>();
        }
        INTMUX0.ch_ier_31_0(channel).modify(|w| w.0 |= 1 << source);
    });
}

/// Stop routing `source` to `channel` without disturbing the other sources on the channel.
pub(crate) fn disable_source(channel: usize, source: u8) {
    critical_section::with(|_| {
        INTMUX0.ch_ier_31_0(channel).modify(|w| w.0 &= !(1 << source));
    });
}
