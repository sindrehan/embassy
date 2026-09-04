#![no_std]
#![allow(unsafe_op_in_unsafe_fn)]

// This mod MUST go first, so that the others see its macros.
pub(crate) mod fmt;

#[cfg(lpc55)]
pub mod adc;
#[cfg(kinetis)]
pub mod clocks;
#[cfg(any(lpc55, kinetis))]
pub mod dma;
pub mod gpio;
#[cfg(kinetis)]
pub mod i2c;
#[cfg(kinetis)]
pub mod intmux;
#[cfg(kinetis)]
pub mod lpuart;
#[cfg(lpc55)]
pub mod pint;
#[cfg(kinetis)]
pub mod power;
#[cfg(lpc55)]
pub mod pwm;
#[cfg(lpc55)]
pub mod sct;
#[cfg(any(lpc55, kinetis))]
pub mod spi;
#[cfg(lpc55)]
pub mod usart;

#[cfg(rt1xxx)]
mod iomuxc;

#[cfg(feature = "_time_driver")]
#[cfg_attr(feature = "time-driver-pit", path = "time_driver/pit.rs")]
#[cfg_attr(feature = "time-driver-rtc", path = "time_driver/rtc.rs")]
#[cfg_attr(feature = "time-driver-tpm", path = "time_driver/tpm.rs")]
mod time_driver;

// This mod MUST go last, so that it sees all the `impl_foo!` macros
#[cfg_attr(lpc55, path = "chips/lpc55.rs")]
#[cfg_attr(feature = "mimxrt1011", path = "chips/mimxrt1011.rs")]
#[cfg_attr(feature = "mimxrt1062", path = "chips/mimxrt1062.rs")]
#[cfg_attr(feature = "mkl82z7", path = "chips/mkl82z7.rs")]
mod chip;

pub use chip::{Peripherals, interrupt, peripherals};
pub use embassy_hal_internal::{Peri, PeripheralType};
#[cfg(feature = "unstable-pac")]
pub use nxp_pac as pac;
#[cfg(not(feature = "unstable-pac"))]
pub(crate) use nxp_pac as pac;

/// Macro to bind interrupts to handlers.
/// (Copied from `embassy-rp`)
/// This defines the right interrupt handlers, and creates a unit struct (like `struct Irqs;`)
/// and implements the right [`Binding`]s for it. You can pass this struct to drivers to
/// prove at compile-time that the right interrupts have been bound.
///
/// Example of how to bind one interrupt:
///
/// ```rust,ignore
/// use embassy_nxp::{bind_interrupts, usart, peripherals};
///
/// bind_interrupts!(
///     /// Binds the USART Interrupts.
///     struct Irqs {
///         FLEXCOMM0 => usart::InterruptHandler<peripherals::USART0>;
///     }
/// );
/// ```
#[macro_export]
macro_rules! bind_interrupts {
    ($(#[$attr:meta])* $vis:vis struct $name:ident {
        $(
            $(#[cfg($cond_irq:meta)])?
            $irq:ident => $(
                $(#[cfg($cond_handler:meta)])?
                $handler:ty
            ),*;
        )*
    }) => {
        #[derive(Copy, Clone)]
        $(#[$attr])*
        $vis struct $name;

        $(
            #[allow(non_snake_case)]
            #[unsafe(no_mangle)]
            $(#[cfg($cond_irq)])?
            unsafe extern "C" fn $irq() {
                unsafe {
                    $(
                        $(#[cfg($cond_handler)])?
                        <$handler as $crate::interrupt::typelevel::Handler<$crate::interrupt::typelevel::$irq>>::on_interrupt();

                    )*
                }
            }

            $(#[cfg($cond_irq)])?
            $crate::bind_interrupts!(@inner
                $(
                    $(#[cfg($cond_handler)])?
                    unsafe impl $crate::interrupt::typelevel::Binding<$crate::interrupt::typelevel::$irq, $handler> for $name {}
                )*
            );
        )*
    };
    (@inner $($t:tt)*) => {
        $($t)*
    }
}

/// Initialize the `embassy-nxp` HAL with the provided configuration.
///
/// This returns the peripheral singletons that can be used for creating drivers.
///
/// This should only be called once and at startup, otherwise it panics.
pub fn init(_config: config::Config) -> Peripherals {
    // Do this first, so that it panics if user is calling `init` a second time
    // before doing anything important.
    let peripherals = Peripherals::take();

    #[cfg(feature = "mimxrt1011")]
    {
        // The RT1010 Reference manual states that core clock root must be switched before
        // reprogramming PLL2.
        pac::CCM.cbcdr().modify(|w| {
            w.set_periph_clk_sel(pac::ccm::vals::PeriphClkSel::PeriphClkSel1);
        });

        while matches!(
            pac::CCM.cdhipr().read().periph_clk_sel_busy(),
            pac::ccm::vals::PeriphClkSelBusy::PeriphClkSelBusy1
        ) {}

        info!("Core clock root switched");

        // 480 * 18 / 24 = 360
        pac::CCM_ANALOG.pfd_480().modify(|x| x.set_pfd2_frac(12));

        //480*18/24(pfd0)/4
        pac::CCM_ANALOG.pfd_480().modify(|x| x.set_pfd0_frac(24));
        pac::CCM.cscmr1().modify(|x| x.set_flexspi_podf(3.into()));

        // CPU Core
        pac::CCM_ANALOG.pfd_528().modify(|x| x.set_pfd3_frac(18));
        cortex_m::asm::delay(500_000);

        // Clock core clock with PLL 2.
        pac::CCM
            .cbcdr()
            .modify(|x| x.set_periph_clk_sel(pac::ccm::vals::PeriphClkSel::PeriphClkSel0)); // false

        while matches!(
            pac::CCM.cdhipr().read().periph_clk_sel_busy(),
            pac::ccm::vals::PeriphClkSelBusy::PeriphClkSelBusy1
        ) {}

        pac::CCM
            .cbcmr()
            .write(|v| v.set_pre_periph_clk_sel(pac::ccm::vals::PrePeriphClkSel::PrePeriphClkSel0));

        // TODO: Some for USB PLLs

        // DCDC clock?
        pac::CCM.ccgr6().modify(|v| v.set_cg0(1));
    }

    #[cfg(kinetis)]
    power::init_protection();

    #[cfg(kinetis)]
    clocks::init(_config.clocks);

    #[cfg(any(lpc55, rt1xxx, kinetis))]
    gpio::init();

    #[cfg(lpc55)]
    {
        pint::init();
        pwm::Pwm::reset();
    }

    #[cfg(feature = "_time_driver")]
    time_driver::init();

    #[cfg(any(lpc55, kinetis))]
    dma::init();

    // Last: VLPR forbids clock changes afterwards, and the idle sleep mode wants the final
    // clock configuration.
    #[cfg(kinetis)]
    power::init(&_config.power, &_config.clocks);

    peripherals
}

// Disable the watchdog before cortex-m-rt initializes RAM. This must be assembly: a Rust
// function may use the uninitialized stack even when its source does not contain local variables.
#[cfg(all(kinetis, feature = "rt"))]
core::arch::global_asm!(
    r#"
    .syntax unified
    .section .text.__pre_init, "ax"
    .global __pre_init
    .type __pre_init, %function
    .thumb_func
__pre_init:
    ldr r0, =0x4005200e
    ldr r1, =0xc520
    ldr r2, =0xd928
    strh r1, [r0]
    strh r2, [r0]

    subs r0, r0, #14
    ldrh r1, [r0]
    movs r2, #1
    bics r1, r2
    strh r1, [r0]
    bx lr
    .size __pre_init, . - __pre_init
"#
);

/// HAL configuration for the NXP board.
pub mod config {
    #[derive(Default)]
    pub struct Config {
        /// System clock configuration.
        #[cfg(kinetis)]
        pub clocks: crate::clocks::ClockConfig,
        /// Power configuration.
        #[cfg(kinetis)]
        pub power: crate::power::Config,
    }
}

#[allow(unused)]
struct BitIter(u32);

impl Iterator for BitIter {
    type Item = u32;

    fn next(&mut self) -> Option<Self::Item> {
        match self.0.trailing_zeros() {
            32 => None,
            b => {
                self.0 &= !(1 << b);
                Some(b)
            }
        }
    }
}

trait SealedMode {}

/// UART mode.
#[allow(private_bounds)]
pub trait Mode: SealedMode {}

macro_rules! impl_mode {
    ($name:ident) => {
        impl SealedMode for $name {}
        impl Mode for $name {}
    };
}

/// Blocking mode.
pub struct Blocking;
/// Asynchronous mode.
pub struct Async;

impl_mode!(Blocking);
impl_mode!(Async);
