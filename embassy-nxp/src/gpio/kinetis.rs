#![macro_use]

use core::future::Future;
use core::task::{Context, Poll};

use embassy_hal_internal::interrupt::InterruptExt;
use embassy_hal_internal::{PeripheralType, impl_peripheral};
use embassy_sync::waitqueue::AtomicWaker;

use crate::pac::common::{RW, Reg};
#[cfg(feature = "rt")]
use crate::pac::interrupt;
use crate::pac::port::vals::{Irqc, Mux};
use crate::pac::{GPIOA, GPIOB, GPIOC, GPIOD, GPIOE, Interrupt, PORTA, PORTB, PORTC, PORTD, PORTE, gpio, port};
use crate::{Peri, peripherals};

const PORT_COUNT: usize = 5;
const PINS_PER_PORT: usize = 32;

/// One waker per pin, indexed by port then pin number.
static WAKERS: [[AtomicWaker; PINS_PER_PORT]; PORT_COUNT] =
    [const { [const { AtomicWaker::new() }; PINS_PER_PORT] }; PORT_COUNT];

pub(crate) fn init() {
    // The pin control registers (PORTx_PCRn) bus-fault until the port clock is gated on.
    crate::clocks::enable::<peripherals::PORTA>();
    crate::clocks::enable::<peripherals::PORTB>();
    crate::clocks::enable::<peripherals::PORTC>();
    crate::clocks::enable::<peripherals::PORTD>();
    crate::clocks::enable::<peripherals::PORTE>();

    // One NVIC line per port carries every pin interrupt; the HAL owns them.
    unsafe {
        Interrupt::PORTA.enable();
        Interrupt::PORTB.enable();
        Interrupt::PORTC.enable();
        Interrupt::PORTD.enable();
        Interrupt::PORTE.enable();
    }
    info!("GPIO initialized");
}

/// Port interrupt: for every pin with its flag set, switch the pin's interrupt off (a level
/// condition would otherwise fire again immediately), clear the flag and wake the waiter, which
/// recognises completion by the interrupt being off.
#[cfg(feature = "rt")]
fn on_port_interrupt(bank: Bank) {
    let port = bank.port();
    let flags = port.isfr().read().0;
    for (pin, waker) in WAKERS[bank as usize].iter().enumerate() {
        if flags & (1 << pin) != 0 {
            port.pcr(pin).modify(|w| {
                w.set_irqc(Irqc::_0000);
                w.set_isf(true);
            });
            waker.wake();
        }
    }
}

#[cfg(feature = "rt")]
#[interrupt]
fn PORTA() {
    on_port_interrupt(Bank::GpioA);
}

#[cfg(feature = "rt")]
#[interrupt]
fn PORTB() {
    on_port_interrupt(Bank::GpioB);
}

#[cfg(feature = "rt")]
#[interrupt]
fn PORTC() {
    on_port_interrupt(Bank::GpioC);
}

#[cfg(feature = "rt")]
#[interrupt]
fn PORTD() {
    on_port_interrupt(Bank::GpioD);
}

#[cfg(feature = "rt")]
#[interrupt]
fn PORTE() {
    on_port_interrupt(Bank::GpioE);
}

/// The GPIO pin level.
#[derive(Debug, Eq, PartialEq, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Level {
    /// Logical low. Corresponds to 0V.
    Low,
    /// Logical high. Corresponds to VDD.
    High,
}

/// Pull setting for a GPIO input.
#[derive(Debug, Clone, Copy, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
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
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Bank {
    GpioA = 0,
    GpioB = 1,
    GpioC = 2,
    GpioD = 3,
    GpioE = 4,
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

        // Set the level before switching to output so the pin never glitches.
        match initial_output {
            Level::High => pin.set_high(),
            Level::Low => pin.set_low(),
        };
        pin.set_as_output();

        Self { pin }
    }

    pub fn set_high(&mut self) {
        self.pin
            .gpio()
            .psor()
            .write(|w| w.set_ptso(self.pin.pin_number() as usize, true));
    }

    pub fn set_low(&mut self) {
        self.pin
            .gpio()
            .pcor()
            .write(|w| w.set_ptco(self.pin.pin_number() as usize, true));
    }

    pub fn toggle(&mut self) {
        self.pin
            .gpio()
            .ptor()
            .write(|w| w.set_ptto(self.pin.pin_number() as usize, true));
    }

    /// Get the current output level of the pin. Note that the value returned by this function is
    /// the voltage level reported by the pin, not the value set by the output driver.
    pub fn level(&self) -> Level {
        self.pin.level()
    }

    /// Whether the output is driven high.
    pub fn is_set_high(&self) -> bool {
        self.pin.is_set_high()
    }

    /// Whether the output is driven low.
    pub fn is_set_low(&self) -> bool {
        !self.is_set_high()
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

    /// Whether the input is high.
    pub fn is_high(&self) -> bool {
        self.read() == Level::High
    }

    /// Whether the input is low.
    pub fn is_low(&self) -> bool {
        self.read() == Level::Low
    }

    /// Wait until the pin is high. Returns immediately if it already is.
    pub async fn wait_for_high(&mut self) {
        self.pin.wait_for_high().await
    }

    /// Wait until the pin is low. Returns immediately if it already is.
    pub async fn wait_for_low(&mut self) {
        self.pin.wait_for_low().await
    }

    /// Wait for a low to high transition.
    pub async fn wait_for_rising_edge(&mut self) {
        self.pin.wait_for_rising_edge().await
    }

    /// Wait for a high to low transition.
    pub async fn wait_for_falling_edge(&mut self) {
        self.pin.wait_for_falling_edge().await
    }

    /// Wait for a transition in either direction.
    pub async fn wait_for_any_edge(&mut self) {
        self.pin.wait_for_any_edge().await
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

    /// Whether the pin is high.
    pub fn is_high(&self) -> bool {
        self.level() == Level::High
    }

    /// Whether the pin is low.
    pub fn is_low(&self) -> bool {
        self.level() == Level::Low
    }

    /// Whether the output register drives the pin high (regardless of direction).
    pub fn is_set_high(&self) -> bool {
        self.gpio().pdor().read().pdo(self.pin.pin_number() as usize)
    }

    pub fn set_high(&mut self) {
        self.gpio()
            .psor()
            .write(|w| w.set_ptso(self.pin.pin_number() as usize, true));
    }

    pub fn set_low(&mut self) {
        self.gpio()
            .pcor()
            .write(|w| w.set_ptco(self.pin.pin_number() as usize, true));
    }

    pub fn toggle(&mut self) {
        self.gpio()
            .ptor()
            .write(|w| w.set_ptto(self.pin.pin_number() as usize, true));
    }

    /// Wait until the pin is high. Returns immediately if it already is.
    pub async fn wait_for_high(&mut self) {
        if self.is_high() {
            return;
        }
        InputFuture::new(self, Irqc::_1100).await
    }

    /// Wait until the pin is low. Returns immediately if it already is.
    pub async fn wait_for_low(&mut self) {
        if self.is_low() {
            return;
        }
        InputFuture::new(self, Irqc::_1000).await
    }

    /// Wait for a low to high transition.
    pub async fn wait_for_rising_edge(&mut self) {
        InputFuture::new(self, Irqc::_1001).await
    }

    /// Wait for a high to low transition.
    pub async fn wait_for_falling_edge(&mut self) {
        InputFuture::new(self, Irqc::_1010).await
    }

    /// Wait for a transition in either direction.
    pub async fn wait_for_any_edge(&mut self) {
        InputFuture::new(self, Irqc::_1011).await
    }
}

/// Completes when the pin interrupt configured with `irqc` has fired. The port handler switches
/// the pin's interrupt off when it fires, which is what the future looks for.
#[must_use = "futures do nothing unless you `.await` or poll them"]
struct InputFuture<'a> {
    bank: Bank,
    pin: u8,
    _lifetime: core::marker::PhantomData<&'a mut ()>,
}

impl<'a> InputFuture<'a> {
    fn new(flex: &'a mut Flex<'_>, irqc: Irqc) -> Self {
        let bank = flex.pin_bank();
        let pin = flex.pin_number();
        // Clear a stale flag and arm. Level modes raise the interrupt at once if the condition
        // already holds, so no event between the caller's check and this point is lost.
        critical_section::with(|_| {
            bank.port().pcr(pin as usize).modify(|w| {
                w.set_isf(true);
                w.set_irqc(irqc);
            });
        });
        Self {
            bank,
            pin,
            _lifetime: core::marker::PhantomData,
        }
    }
}

impl<'a> Future for InputFuture<'a> {
    type Output = ();

    fn poll(self: core::pin::Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<()> {
        WAKERS[self.bank as usize][self.pin as usize].register(cx.waker());
        if self.bank.port().pcr(self.pin as usize).read().irqc() == Irqc::_0000 {
            Poll::Ready(())
        } else {
            Poll::Pending
        }
    }
}

impl<'a> Drop for InputFuture<'a> {
    fn drop(&mut self) {
        critical_section::with(|_| {
            self.bank.port().pcr(self.pin as usize).modify(|w| {
                w.set_irqc(Irqc::_0000);
                w.set_isf(true);
            });
        });
    }
}

// embedded-hal digital traits.

impl<'d> embedded_hal_1::digital::ErrorType for Input<'d> {
    type Error = core::convert::Infallible;
}

impl<'d> embedded_hal_1::digital::InputPin for Input<'d> {
    fn is_high(&mut self) -> Result<bool, Self::Error> {
        Ok(Input::is_high(self))
    }

    fn is_low(&mut self) -> Result<bool, Self::Error> {
        Ok(Input::is_low(self))
    }
}

impl<'d> embedded_hal_async::digital::Wait for Input<'d> {
    async fn wait_for_high(&mut self) -> Result<(), Self::Error> {
        Input::wait_for_high(self).await;
        Ok(())
    }

    async fn wait_for_low(&mut self) -> Result<(), Self::Error> {
        Input::wait_for_low(self).await;
        Ok(())
    }

    async fn wait_for_rising_edge(&mut self) -> Result<(), Self::Error> {
        Input::wait_for_rising_edge(self).await;
        Ok(())
    }

    async fn wait_for_falling_edge(&mut self) -> Result<(), Self::Error> {
        Input::wait_for_falling_edge(self).await;
        Ok(())
    }

    async fn wait_for_any_edge(&mut self) -> Result<(), Self::Error> {
        Input::wait_for_any_edge(self).await;
        Ok(())
    }
}

impl<'d> embedded_hal_1::digital::ErrorType for Output<'d> {
    type Error = core::convert::Infallible;
}

impl<'d> embedded_hal_1::digital::OutputPin for Output<'d> {
    fn set_high(&mut self) -> Result<(), Self::Error> {
        Output::set_high(self);
        Ok(())
    }

    fn set_low(&mut self) -> Result<(), Self::Error> {
        Output::set_low(self);
        Ok(())
    }
}

impl<'d> embedded_hal_1::digital::StatefulOutputPin for Output<'d> {
    fn is_set_high(&mut self) -> Result<bool, Self::Error> {
        Ok(Output::is_set_high(self))
    }

    fn is_set_low(&mut self) -> Result<bool, Self::Error> {
        Ok(Output::is_set_low(self))
    }
}

impl<'d> embedded_hal_1::digital::ErrorType for Flex<'d> {
    type Error = core::convert::Infallible;
}

impl<'d> embedded_hal_1::digital::InputPin for Flex<'d> {
    fn is_high(&mut self) -> Result<bool, Self::Error> {
        Ok(Flex::is_high(self))
    }

    fn is_low(&mut self) -> Result<bool, Self::Error> {
        Ok(Flex::is_low(self))
    }
}

impl<'d> embedded_hal_1::digital::OutputPin for Flex<'d> {
    fn set_high(&mut self) -> Result<(), Self::Error> {
        Flex::set_high(self);
        Ok(())
    }

    fn set_low(&mut self) -> Result<(), Self::Error> {
        Flex::set_low(self);
        Ok(())
    }
}

impl<'d> embedded_hal_1::digital::StatefulOutputPin for Flex<'d> {
    fn is_set_high(&mut self) -> Result<bool, Self::Error> {
        Ok(Flex::is_set_high(self))
    }

    fn is_set_low(&mut self) -> Result<bool, Self::Error> {
        Ok(!Flex::is_set_high(self))
    }
}

impl<'d> embedded_hal_async::digital::Wait for Flex<'d> {
    async fn wait_for_high(&mut self) -> Result<(), Self::Error> {
        Flex::wait_for_high(self).await;
        Ok(())
    }

    async fn wait_for_low(&mut self) -> Result<(), Self::Error> {
        Flex::wait_for_low(self).await;
        Ok(())
    }

    async fn wait_for_rising_edge(&mut self) -> Result<(), Self::Error> {
        Flex::wait_for_rising_edge(self).await;
        Ok(())
    }

    async fn wait_for_falling_edge(&mut self) -> Result<(), Self::Error> {
        Flex::wait_for_falling_edge(self).await;
        Ok(())
    }

    async fn wait_for_any_edge(&mut self) -> Result<(), Self::Error> {
        Flex::wait_for_any_edge(self).await;
        Ok(())
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
