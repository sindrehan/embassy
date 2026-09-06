//! ADC16 driver for Kinetis MCUs.
//!
//! Conversions are single-ended. The ADC is calibrated when the driver is created, using
//! 32-sample hardware averaging and an ADC clock at or below 4 MHz.
//! Asynchronous conversions keep the executor in WAIT until they complete or are cancelled.
#![macro_use]

use core::future::poll_fn;
use core::sync::atomic::{AtomicU32, Ordering};
use core::task::Poll;

use embassy_hal_internal::drop::OnDrop;
use embassy_hal_internal::{Peri, PeripheralType};
use embassy_sync::waitqueue::AtomicWaker;

use crate::interrupt::typelevel::{Binding, Interrupt};
use crate::pac::adc::Adc as Regs;
use crate::pac::adc::vals::{Adch, Adiclk, Adiv, Avgs, Mode as ConversionMode, Refsel};
use crate::{Async, Blocking, Mode};

const MAX_CALIBRATION_CLOCK_HZ: u32 = 4_000_000;
const NO_RESULT: u32 = u32::MAX;

/// ADC resolution.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Resolution {
    /// 8-bit single-ended conversions.
    Bits8,
    /// 10-bit single-ended conversions.
    Bits10,
    /// 12-bit single-ended conversions.
    Bits12,
    /// 16-bit single-ended conversions.
    Bits16,
}

/// Number of samples averaged by the ADC for each result.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Averaging {
    /// Hardware averaging disabled.
    Disabled,
    /// Average 4 samples.
    Samples4,
    /// Average 8 samples.
    Samples8,
    /// Average 16 samples.
    Samples16,
    /// Average 32 samples.
    Samples32,
}

/// Internal ADC input.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum InternalChannel {
    /// Internal temperature sensor.
    Temperature,
    /// High side of the selected voltage reference.
    VrefHigh,
    /// Low side of the selected voltage reference.
    VrefLow,
}

impl InternalChannel {
    fn channel(self) -> u8 {
        match self {
            Self::Temperature => 26,
            Self::VrefHigh => 29,
            Self::VrefLow => 30,
        }
    }
}

/// ADC configuration.
#[non_exhaustive]
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Config {
    /// Conversion resolution.
    pub resolution: Resolution,
    /// Hardware averaging applied to each result.
    pub averaging: Averaging,
}

impl Config {
    /// Create an ADC configuration.
    pub const fn new(resolution: Resolution, averaging: Averaging) -> Self {
        Self { resolution, averaging }
    }
}

impl Default for Config {
    fn default() -> Self {
        Self {
            resolution: Resolution::Bits16,
            averaging: Averaging::Disabled,
        }
    }
}

/// ADC error.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum Error {
    /// The ADC self-calibration sequence failed.
    CalibrationFailed,
}

impl core::fmt::Display for Error {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        core::fmt::Debug::fmt(self, f)
    }
}

impl core::error::Error for Error {}

/// Per-instance constants.
pub struct Info {
    pub(crate) regs: Regs,
}

/// Per-instance async state.
pub struct State {
    waker: AtomicWaker,
    result: AtomicU32,
}

impl State {
    /// Create empty ADC state.
    pub const fn new() -> Self {
        Self {
            waker: AtomicWaker::new(),
            result: AtomicU32::new(NO_RESULT),
        }
    }
}

impl Default for State {
    fn default() -> Self {
        Self::new()
    }
}

/// ADC driver.
///
/// Dropping the driver stops conversions and disables its clock and interrupt. Pins are borrowed
/// only during reads; they remain in analog mode and may be reconfigured by their owner.
pub struct Adc<'d, T: Instance, M: Mode> {
    _peri: Peri<'d, T>,
    info: &'static Info,
    state: &'static State,
    resolution: Resolution,
    averaging: Averaging,
    disable_interrupt: Option<fn()>,
    _mode: core::marker::PhantomData<M>,
}

fn clock_divider(bus_hz: u32) -> Adiv {
    assert!(bus_hz != 0, "ADC bus clock is not running");
    match bus_hz.div_ceil(MAX_CALIBRATION_CLOCK_HZ) {
        0 | 1 => Adiv::_00,
        2 => Adiv::_01,
        3..=4 => Adiv::_10,
        5..=8 => Adiv::_11,
        _ => panic!("ADC bus clock is too fast"),
    }
}

fn conversion_mode(resolution: Resolution) -> ConversionMode {
    match resolution {
        Resolution::Bits8 => ConversionMode::_00,
        Resolution::Bits10 => ConversionMode::_10,
        Resolution::Bits12 => ConversionMode::_01,
        Resolution::Bits16 => ConversionMode::_11,
    }
}

fn apply_averaging(regs: Regs, averaging: Averaging) {
    regs.sc3().write(|w| match averaging {
        Averaging::Disabled => w.set_avge(false),
        Averaging::Samples4 => {
            w.set_avge(true);
            w.set_avgs(Avgs::_00);
        }
        Averaging::Samples8 => {
            w.set_avge(true);
            w.set_avgs(Avgs::_01);
        }
        Averaging::Samples16 => {
            w.set_avge(true);
            w.set_avgs(Avgs::_10);
        }
        Averaging::Samples32 => {
            w.set_avge(true);
            w.set_avgs(Avgs::_11);
        }
    });
}

fn calibrate(regs: Regs, averaging: Averaging) -> Result<(), Error> {
    // CALF is write-one-to-clear. Calibration must use software triggering and must not be
    // interrupted by any ADC register write.
    regs.sc3().write(|w| w.set_calf(true));
    regs.sc3().write(|w| {
        w.set_avge(true);
        w.set_avgs(Avgs::_11);
        w.set_cal(true);
    });
    while regs.sc3().read().cal() {}

    let failed = regs.sc3().read().calf();
    let _ = regs.r(0).read();
    if failed {
        regs.sc3().write(|w| w.set_calf(true));
        apply_averaging(regs, averaging);
        return Err(Error::CalibrationFailed);
    }

    let plus = u32::from(regs.clp0().read().clp0())
        + u32::from(regs.clp1().read().clp1())
        + u32::from(regs.clp2().read().clp2())
        + u32::from(regs.clp3().read().clp3())
        + u32::from(regs.clp4().read().clp4())
        + u32::from(regs.clps().read().clps());
    regs.pg().write(|w| w.set_pg(((plus >> 1) as u16) | 0x8000));

    let minus = u32::from(regs.clm0().read().clm0())
        + u32::from(regs.clm1().read().clm1())
        + u32::from(regs.clm2().read().clm2())
        + u32::from(regs.clm3().read().clm3())
        + u32::from(regs.clm4().read().clm4())
        + u32::from(regs.clms().read().clms());
    regs.mg().write(|w| w.set_mg(((minus >> 1) as u16) | 0x8000));

    apply_averaging(regs, averaging);
    Ok(())
}

fn init<T: Instance>(config: Config) -> Result<(), Error> {
    T::enable_clock();
    let on_error = OnDrop::new(deinit::<T>);
    let regs = T::info().regs;

    regs.sc1(0).write(|w| w.set_adch(Adch::_11111));
    regs.sc1(1).write(|w| w.set_adch(Adch::_11111));
    regs.cfg1().write(|w| {
        w.set_adiclk(Adiclk::_00);
        w.set_mode(conversion_mode(config.resolution));
        w.set_adiv(clock_divider(crate::clocks::clocks().bus));
    });
    regs.cfg2().write(|_| {});
    regs.sc2().write(|w| w.set_refsel(Refsel::_00));

    calibrate(regs, config.averaging)?;
    on_error.defuse();
    Ok(())
}

fn deinit<T: Instance>() {
    let regs = T::info().regs;
    regs.sc1(0).write(|w| w.set_adch(Adch::_11111));
    regs.sc1(1).write(|w| w.set_adch(Adch::_11111));
    regs.sc2().write(|_| {});
    regs.sc3().write(|w| w.set_calf(true));
    T::state().result.store(NO_RESULT, Ordering::Relaxed);
    T::disable_clock();
}

fn disable_interrupt<T: InterruptInstance>() {
    T::Interrupt::disable();
    T::Interrupt::unpend();
}

impl<T: Instance, M: Mode> Drop for Adc<'_, T, M> {
    fn drop(&mut self) {
        critical_section::with(|_| {
            if let Some(disable) = self.disable_interrupt {
                disable();
            }
            deinit::<T>();
        });
    }
}

fn start_conversion(regs: Regs, channel: u8, mux_b: bool, interrupt: bool) {
    regs.cfg2().modify(|w| w.set_muxsel(mux_b));
    regs.sc1(0).write(|w| {
        w.set_adch(Adch::from_bits(channel));
        w.set_diff(false);
        w.set_aien(interrupt);
    });
}

fn stop_conversion<T: InterruptInstance>() {
    critical_section::with(|_| {
        T::info().regs.sc1(0).write(|w| w.set_adch(Adch::_11111));
        T::state().result.store(NO_RESULT, Ordering::Relaxed);
        T::Interrupt::unpend();
    });
}

impl<'d, T: Instance> Adc<'d, T, Blocking> {
    /// Create and calibrate a blocking ADC driver.
    pub fn new_blocking(peri: Peri<'d, T>, config: Config) -> Result<Self, Error> {
        init::<T>(config)?;
        Ok(Self {
            _peri: peri,
            info: T::info(),
            state: T::state(),
            resolution: config.resolution,
            averaging: config.averaging,
            disable_interrupt: None,
            _mode: core::marker::PhantomData,
        })
    }
}

impl<'d, T: InterruptInstance> Adc<'d, T, Async> {
    /// Create and calibrate an interrupt-driven ADC driver.
    pub fn new(
        peri: Peri<'d, T>,
        _irq: impl Binding<T::Interrupt, InterruptHandler<T>>,
        config: Config,
    ) -> Result<Self, Error> {
        init::<T>(config)?;
        T::Interrupt::unpend();
        unsafe { T::Interrupt::enable() };
        Ok(Self {
            _peri: peri,
            info: T::info(),
            state: T::state(),
            resolution: config.resolution,
            averaging: config.averaging,
            disable_interrupt: Some(disable_interrupt::<T>),
            _mode: core::marker::PhantomData,
        })
    }

    async fn read_channel(&mut self, channel: u8, mux_b: bool) -> u16 {
        let _wake_guard = crate::power::wake_guard();
        let regs = self.info.regs;
        let state = self.state;
        critical_section::with(|_| {
            state.result.store(NO_RESULT, Ordering::Relaxed);
            start_conversion(regs, channel, mux_b, true);
        });

        let on_drop = OnDrop::new(stop_conversion::<T>);
        let result = poll_fn(|cx| {
            state.waker.register(cx.waker());
            match state.result.load(Ordering::Acquire) {
                NO_RESULT => Poll::Pending,
                result => Poll::Ready(result as u16),
            }
        })
        .await;
        stop_conversion::<T>();
        on_drop.defuse();
        result
    }

    /// Read an ADC pin asynchronously.
    pub async fn read(&mut self, pin: &mut Peri<'_, impl AdcPin<T>>) -> u16 {
        pin.configure_for_adc();
        self.read_channel(pin.channel(), pin.mux_b()).await
    }

    /// Read an internal ADC input asynchronously.
    pub async fn read_internal(&mut self, channel: InternalChannel) -> u16 {
        self.read_channel(channel.channel(), false).await
    }
}

impl<'d, T: Instance, M: Mode> Adc<'d, T, M> {
    /// Read an ADC pin, waiting for the conversion to finish.
    pub fn blocking_read(&mut self, pin: &mut Peri<'_, impl AdcPin<T>>) -> u16 {
        pin.configure_for_adc();
        self.blocking_read_channel(pin.channel(), pin.mux_b())
    }

    /// Read an internal ADC input, waiting for the conversion to finish.
    pub fn blocking_read_internal(&mut self, channel: InternalChannel) -> u16 {
        self.blocking_read_channel(channel.channel(), false)
    }

    fn blocking_read_channel(&mut self, channel: u8, mux_b: bool) -> u16 {
        let regs = self.info.regs;
        start_conversion(regs, channel, mux_b, false);
        while !regs.sc1(0).read().coco() {}
        regs.r(0).read().d()
    }

    /// Run ADC self-calibration again.
    pub fn calibrate(&mut self) -> Result<(), Error> {
        self.info.regs.sc1(0).write(|w| w.set_adch(Adch::_11111));
        calibrate(self.info.regs, self.averaging)
    }

    /// Return the conversion resolution.
    pub fn resolution(&self) -> Resolution {
        self.resolution
    }

    /// Set the conversion resolution.
    pub fn set_resolution(&mut self, resolution: Resolution) {
        self.info
            .regs
            .cfg1()
            .modify(|w| w.set_mode(conversion_mode(resolution)));
        self.resolution = resolution;
    }

    /// Return the hardware averaging setting.
    pub fn averaging(&self) -> Averaging {
        self.averaging
    }

    /// Set the number of samples averaged for each result.
    pub fn set_averaging(&mut self, averaging: Averaging) {
        apply_averaging(self.info.regs, averaging);
        self.averaging = averaging;
    }
}

/// Interrupt handler. Bind it with [`bind_interrupts!`](crate::bind_interrupts).
pub struct InterruptHandler<T: InterruptInstance> {
    _phantom: core::marker::PhantomData<T>,
}

impl<T: InterruptInstance> crate::interrupt::typelevel::Handler<T::Interrupt> for InterruptHandler<T> {
    unsafe fn on_interrupt() {
        let regs = T::info().regs;
        if regs.sc1(0).read().aien() && regs.sc1(0).read().coco() {
            let result = regs.r(0).read().d();
            T::state().result.store(u32::from(result), Ordering::Release);
            T::state().waker.wake();
        }
    }
}

pub(crate) trait SealedInstance {
    fn info() -> &'static Info;
    fn state() -> &'static State;
    fn enable_clock();
    fn disable_clock();
}

/// An ADC instance.
#[allow(private_bounds)]
pub trait Instance: SealedInstance + PeripheralType {}

/// An ADC instance with an interrupt, usable in async mode.
pub trait InterruptInstance: Instance {
    /// NVIC interrupt for this instance.
    type Interrupt: Interrupt;
}

macro_rules! impl_adc_instance {
    ($inst:ident) => {
        impl crate::adc::SealedInstance for crate::peripherals::$inst {
            fn info() -> &'static crate::adc::Info {
                static INFO: crate::adc::Info = crate::adc::Info {
                    regs: crate::pac::$inst,
                };
                &INFO
            }

            fn state() -> &'static crate::adc::State {
                static STATE: crate::adc::State = crate::adc::State::new();
                &STATE
            }

            fn enable_clock() {
                crate::clocks::enable::<crate::peripherals::$inst>();
            }

            fn disable_clock() {
                crate::clocks::disable::<crate::peripherals::$inst>();
            }
        }

        impl crate::adc::Instance for crate::peripherals::$inst {}
    };
}

macro_rules! impl_adc_interrupt {
    ($inst:ident, $irq:ident) => {
        impl crate::adc::InterruptInstance for crate::peripherals::$inst {
            type Interrupt = crate::interrupt::typelevel::$irq;
        }
    };
}

pub(crate) trait SealedAdcPin<T: Instance>: PeripheralType {
    fn channel(&self) -> u8;
    fn mux_b(&self) -> bool;
    fn configure_for_adc(&self);
}

/// A pin that can be sampled by an ADC instance.
///
/// Includes dedicated analog pins as well as GPIO-backed inputs. On MKL82, use
/// `peripherals::VREF_OUT` for ADC0_SE22 and `peripherals::DAC0_OUT` for ADC0_SE23.
/// Dedicated inputs use the same calibrated, cancellation-safe conversion path as GPIO pins;
/// they do not require a pin-mux change. Disable any other analog function sharing the input.
#[allow(private_bounds)]
pub trait AdcPin<T: Instance>: SealedAdcPin<T> + PeripheralType {}

macro_rules! impl_adc_gpio_pin {
    ($pin:ident, $inst:ident, $channel:expr, $mux_b:expr) => {
        impl crate::adc::SealedAdcPin<crate::peripherals::$inst> for crate::peripherals::$pin {
            fn channel(&self) -> u8 {
                $channel
            }

            fn mux_b(&self) -> bool {
                $mux_b
            }

            fn configure_for_adc(&self) {
                crate::gpio::SealedPin::pcr(self).write(|w| w.set_isf(true));
            }
        }

        impl crate::adc::AdcPin<crate::peripherals::$inst> for crate::peripherals::$pin {}
    };
}

macro_rules! impl_adc_fixed_pin {
    ($pin:ident, $inst:ident, $channel:expr, $mux_b:expr) => {
        impl crate::adc::SealedAdcPin<crate::peripherals::$inst> for crate::peripherals::$pin {
            fn channel(&self) -> u8 {
                $channel
            }

            fn mux_b(&self) -> bool {
                $mux_b
            }

            fn configure_for_adc(&self) {}
        }

        impl crate::adc::AdcPin<crate::peripherals::$inst> for crate::peripherals::$pin {}
    };
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn calibration_clock_does_not_exceed_four_mhz() {
        for bus_hz in [4_000_000, 8_000_000, 16_000_000, 24_000_000, 32_000_000] {
            let divider = 1u32 << clock_divider(bus_hz).to_bits();
            assert!(bus_hz / divider <= MAX_CALIBRATION_CLOCK_HZ);
        }
    }
}
