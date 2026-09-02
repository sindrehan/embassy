#![macro_use]

use embassy_hal_internal::{PeripheralType, impl_peripheral};

use crate::Peri;
use crate::pac::common::{RW, Reg};
use crate::pac::port::vals::Mux;
use crate::pac::{GPIOA, GPIOB, GPIOC, GPIOD, GPIOE, PORTA, PORTB, PORTC, PORTD, PORTE, gpio, port};
use crate::peripherals;

pub(crate) fn init() {
    // The pin control registers (PORTx_PCRn) bus-fault until the port clock is gated on.
    crate::clocks::enable::<peripherals::PORTA>();
    crate::clocks::enable::<peripherals::PORTB>();
    crate::clocks::enable::<peripherals::PORTC>();
    crate::clocks::enable::<peripherals::PORTD>();
    crate::clocks::enable::<peripherals::PORTE>();
    info!("GPIO initialized");
}

/// The GPIO pin level.
#[derive(Debug, Eq, PartialEq, Clone, Copy)]
pub enum Level {
    /// Logical low. Corresponds to 0V.
    Low,
    /// Logical high. Corresponds to VDD.
    High,
}

/// Pull setting for a GPIO input.
#[derive(Debug, Clone, Copy, Eq, PartialEq)]
pub enum Pull {
    /// No pull.
    None,
    /// Internal pull-up resistor.
    Up,
    /// Internal pull-down resistor.
    Down,
}

/// A GPIO port. Each Kinetis port pairs a `PORTx` pin control block (mux, pull, interrupts) with
/// a `GPIOx` data block (direction, input, output).
#[derive(Debug, Eq, PartialEq, Clone, Copy)]
pub enum Bank {
    GpioA,
    GpioB,
    GpioC,
    GpioD,
    GpioE,
}

impl Bank {
    pub(crate) const fn port(self) -> port::Port {
        match self {
            Bank::GpioA => PORTA,
            Bank::GpioB => PORTB,
            Bank::GpioC => PORTC,
            Bank::GpioD => PORTD,
            Bank::GpioE => PORTE,
        }
    }

    pub(crate) const fn gpio(self) -> gpio::Gpio {
        match self {
            Bank::GpioA => GPIOA,
            Bank::GpioB => GPIOB,
            Bank::GpioC => GPIOC,
            Bank::GpioD => GPIOD,
            Bank::GpioE => GPIOE,
        }
    }
}

/// GPIO output driver. Internally, this is a specialized [Flex] pin.
pub struct Output<'d> {
    pub(crate) pin: Flex<'d>,
}

impl<'d> Output<'d> {
    /// Create GPIO output driver for a [Pin] with the provided [initial output](Level).
    #[inline]
    pub fn new(pin: Peri<'d, impl Pin>, initial_output: Level) -> Self {
        let mut pin = Flex::new(pin);
        let mut result = Self { pin: Flex { pin: unsafe { AnyPin::steal(pin.pin_bank(), pin.pin_number()) } } };

        // Set the level before switching to output so the pin never glitches.
        match initial_output {
            Level::High => result.set_high(),
            Level::Low => result.set_low(),
        };
        pin.set_as_output();
        result.pin = pin;

        result
    }

    pub fn set_high(&mut self) {
        self.pin.gpio().psor().write(|w| w.set_ptso(self.pin.pin_number() as usize, true));
    }

    pub fn set_low(&mut self) {
        self.pin.gpio().pcor().write(|w| w.set_ptco(self.pin.pin_number() as usize, true));
    }

    pub fn toggle(&mut self) {
        self.pin.gpio().ptor().write(|w| w.set_ptto(self.pin.pin_number() as usize, true));
    }

    /// Get the current output level of the pin. Note that the value returned by this function is
    /// the voltage level reported by the pin, not the value set by the output driver.
    pub fn level(&self) -> Level {
        self.pin.level()
    }
}

/// GPIO input driver. Internally, this is a specialized [Flex] pin.
pub struct Input<'d> {
    pub(crate) pin: Flex<'d>,
}

impl<'d> Input<'d> {
    /// Create GPIO input driver for a [Pin] with the provided [Pull].
    #[inline]
    pub fn new(pin: Peri<'d, impl Pin>, pull: Pull) -> Self {
        let mut pin = Flex::new(pin);
        pin.set_as_input();
        let mut result = Self { pin };
        result.set_pull(pull);

        result
    }

    /// Set the pull configuration for the pin. To disable the pull, use [Pull::None].
    pub fn set_pull(&mut self, pull: Pull) {
        self.pin.set_pull(pull);
    }

    /// Get the current input level of the pin.
    pub fn read(&self) -> Level {
        self.pin.level()
    }
}

/// A flexible GPIO pin whose mode is not yet determined. Under the hood, this is a reference to a
/// type-erased pin called ["AnyPin"](AnyPin).
pub struct Flex<'d> {
    pub(crate) pin: Peri<'d, AnyPin>,
}

impl<'d> Flex<'d> {
    /// Wrap the pin in a `Flex`.
    ///
    /// Note: the pin keeps whatever mux setting it had; it is not switched to GPIO until
    /// [`set_as_input`](Self::set_as_input) or [`set_as_output`](Self::set_as_output).
    #[inline]
    pub fn new(pin: Peri<'d, impl Pin>) -> Self {
        Self { pin: pin.into() }
    }

    /// Get the bank of this pin. See also [Bank].
    pub fn pin_bank(&self) -> Bank {
        self.pin.pin_bank()
    }

    /// Get the number of this pin within its bank.
    pub fn pin_number(&self) -> u8 {
        self.pin.pin_number()
    }

    /// Get the bit mask for this pin. PTx0 is bit 0, PTx1 is bit 1, etc.
    pub fn bit(&self) -> u32 {
        1 << self.pin.pin_number()
    }

    fn gpio(&self) -> gpio::Gpio {
        self.pin.pin_bank().gpio()
    }

    /// Set the pull configuration for the pin. To disable the pull, use [Pull::None].
    pub fn set_pull(&mut self, pull: Pull) {
        self.pin.pcr().modify(|w| {
            w.set_pe(pull != Pull::None);
            w.set_ps(pull == Pull::Up);
        });
    }

    /// Route the pin to the GPIO function (ALT1).
    fn set_as_gpio(&mut self) {
        self.pin.pcr().modify(|w| w.set_mux(Mux::Mux1));
    }

    /// Set the pin in output mode. This also routes the pin to the GPIO function.
    pub fn set_as_output(&mut self) {
        self.set_as_gpio();
        self.gpio()
            .pddr()
            .modify(|w| w.set_pdd(self.pin.pin_number() as usize, true));
    }

    /// Set the pin in input mode. This also routes the pin to the GPIO function.
    pub fn set_as_input(&mut self) {
        self.set_as_gpio();
        self.gpio()
            .pddr()
            .modify(|w| w.set_pdd(self.pin.pin_number() as usize, false));
    }

    /// Get the current level of the pin, as seen by the input buffer.
    pub fn level(&self) -> Level {
        if self.gpio().pdir().read().pdi(self.pin.pin_number() as usize) {
            Level::High
        } else {
            Level::Low
        }
    }
}

/// Sealed trait for pins. This trait is sealed and cannot be implemented outside of this crate.
pub(crate) trait SealedPin: Sized {
    fn pin_bank(&self) -> Bank;
    fn pin_number(&self) -> u8;

    /// The pin control register (`PORTx_PCRn`) of this pin.
    #[inline]
    fn pcr(&self) -> Reg<port::regs::Pcr, RW> {
        self.pin_bank().port().pcr(self.pin_number() as usize)
    }
}

/// Interface for a Pin that can be configured by an [Input] or [Output] driver, or converted to an
/// [AnyPin]. By default, this trait is sealed and cannot be implemented outside of the
/// `embassy-nxp` crate due to the [SealedPin] trait.
#[allow(private_bounds)]
pub trait Pin: PeripheralType + Into<AnyPin> + SealedPin + Sized + 'static {
    /// Returns the pin number within a bank
    #[inline]
    fn pin(&self) -> u8 {
        self.pin_number()
    }

    /// Returns the bank of this pin
    #[inline]
    fn bank(&self) -> Bank {
        self.pin_bank()
    }
}

/// Type-erased GPIO pin.
pub struct AnyPin {
    pub(crate) pin_bank: Bank,
    pub(crate) pin_number: u8,
}

impl AnyPin {
    /// Unsafely create a new type-erased pin.
    ///
    /// # Safety
    ///
    /// You must ensure that you’re only using one instance of this type at a time.
    pub unsafe fn steal(pin_bank: Bank, pin_number: u8) -> Peri<'static, Self> {
        Peri::new_unchecked(Self { pin_bank, pin_number })
    }
}

impl_peripheral!(AnyPin);

impl Pin for AnyPin {}
impl SealedPin for AnyPin {
    #[inline]
    fn pin_bank(&self) -> Bank {
        self.pin_bank
    }

    #[inline]
    fn pin_number(&self) -> u8 {
        self.pin_number
    }
}

macro_rules! impl_pin {
    ($name:ident, $bank:ident, $pin_num:expr) => {
        impl crate::gpio::Pin for peripherals::$name {}
        impl crate::gpio::SealedPin for peripherals::$name {
            #[inline]
            fn pin_bank(&self) -> crate::gpio::Bank {
                crate::gpio::Bank::$bank
            }

            #[inline]
            fn pin_number(&self) -> u8 {
                $pin_num
            }
        }

        impl From<peripherals::$name> for crate::gpio::AnyPin {
            fn from(val: peripherals::$name) -> Self {
                use crate::gpio::SealedPin;

                Self {
                    pin_bank: val.pin_bank(),
                    pin_number: val.pin_number(),
                }
            }
        }
    };
}
