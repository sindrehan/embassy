//! Pulse-width modulation using the Kinetis Timer/PWM Module (TPM) or FlexIO timers.
//!
//! All channels of a TPM instance share one counter and therefore one frequency. Each output pin
//! is configured separately with [`Pwm::enable_channel`], then controlled directly or through an
//! [`embedded_hal_1::pwm::SetDutyCycle`] channel returned by [`Pwm::channel`].
//!
//! [`FlexioPwm`] provides independent PWM outputs from FlexIO's dual 8-bit timer mode. It owns the
//! complete FlexIO instance, including timers and shifters.

#![macro_use]

use core::convert::Infallible;
use core::marker::PhantomData;

use embassy_hal_internal::{Peri, PeripheralType};

use crate::gpio::{AnyPin, SealedPin};
use crate::pac::flexio::Flexio as FlexioRegisters;
use crate::pac::flexio::vals::{TimctlPincfg, Timdec, Timdis, Timena, Timod, Timout, Timrst};
use crate::pac::port::vals::Mux;
use crate::pac::tpm::vals::Ps;

const MAX_CHANNELS: usize = 6;
const FLEXIO_TIMERS: usize = 8;

/// PWM output polarity.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Polarity {
    /// The output is high during the active portion of the cycle.
    ActiveHigh,
    /// The output is low during the active portion of the cycle.
    ActiveLow,
}

/// PWM configuration.
#[non_exhaustive]
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Config {
    /// Requested output frequency in hertz.
    ///
    /// The closest frequency supported by the 4 MHz TPM clock is used. Frequencies from 1 Hz to
    /// 2 MHz are accepted.
    pub frequency: u32,
}

impl Default for Config {
    fn default() -> Self {
        Self { frequency: 1_000 }
    }
}

/// PWM driver for one TPM instance.
pub struct Pwm<'d, T: Instance> {
    _peri: Peri<'d, T>,
    pins: [Option<Peri<'d, AnyPin>>; MAX_CHANNELS],
    period_ticks: u16,
    frequency: u32,
}

impl<'d, T: Instance> Pwm<'d, T> {
    /// Create a PWM driver.
    ///
    /// The TPM uses edge-aligned PWM. Outputs remain disabled until configured with
    /// [`enable_channel`](Self::enable_channel).
    pub fn new(peri: Peri<'d, T>, config: Config) -> Self {
        let (prescaler, period_ticks, frequency) = timing(config.frequency);

        crate::clocks::enable_tpm_clock();
        T::enable_clock();
        T::configure(prescaler, period_ticks);

        Self {
            _peri: peri,
            pins: core::array::from_fn(|_| None),
            period_ticks,
            frequency,
        }
    }

    /// Return the actual PWM frequency in hertz.
    pub fn frequency(&self) -> u32 {
        self.frequency
    }

    /// Return the duty value corresponding to a 100% duty cycle.
    pub fn max_duty_cycle(&self) -> u16 {
        self.period_ticks
    }

    /// Configure and enable a TPM channel on `pin`, initially at 0% duty.
    pub fn enable_channel<const C: usize>(&mut self, pin: Peri<'d, impl ChannelPin<T, C>>, polarity: Polarity) {
        assert!(C < T::CHANNELS, "channel does not exist on this TPM instance");
        assert!(self.pins[C].is_none(), "PWM channel is already enabled");

        // Configure the inactive level before connecting the peripheral to the pin. In
        // edge-aligned PWM, MSB:MSA=10 selects PWM and ELSB:ELSA selects its polarity.
        T::configure_channel(C, polarity);
        pin.pcr().modify(|w| w.set_mux(pin.alt()));
        self.pins[C] = Some(pin.into());
    }

    /// Disable a channel and disconnect its pin from the TPM.
    pub fn disable_channel<const C: usize>(&mut self) {
        assert!(C < T::CHANNELS, "channel does not exist on this TPM instance");
        T::disable_channel(C);
        if let Some(pin) = self.pins[C].take() {
            pin.pcr().modify(|w| w.set_mux(Mux::Mux0));
        }
    }

    /// Set the duty cycle for a configured channel.
    ///
    /// `0` is always inactive and [`max_duty_cycle`](Self::max_duty_cycle) is always active.
    pub fn set_duty_cycle<const C: usize>(&mut self, duty: u16) {
        self.check_channel::<C>();
        assert!(duty <= self.period_ticks, "PWM duty cycle exceeds its maximum");
        T::set_duty_cycle(C, duty);
    }

    /// Return the current duty cycle for a configured channel.
    pub fn duty_cycle<const C: usize>(&self) -> u16 {
        self.check_channel::<C>();
        T::duty_cycle(C)
    }

    /// Borrow a configured output as an embedded-hal PWM channel.
    pub fn channel<const C: usize>(&mut self) -> PwmChannel<'_, 'd, T, C> {
        self.check_channel::<C>();
        PwmChannel {
            pwm: self,
            _instance: PhantomData,
        }
    }

    fn check_channel<const C: usize>(&self) {
        assert!(C < T::CHANNELS, "channel does not exist on this TPM instance");
        assert!(self.pins[C].is_some(), "PWM channel is not enabled");
    }
}

impl<'d, T: Instance> Drop for Pwm<'d, T> {
    fn drop(&mut self) {
        T::stop();
        for channel in 0..T::CHANNELS {
            if let Some(pin) = self.pins[channel].take() {
                pin.pcr().modify(|w| w.set_mux(Mux::Mux0));
            }
        }
        T::disable_clock();
    }
}

/// A borrowed PWM output implementing the embedded-hal duty-cycle API.
pub struct PwmChannel<'a, 'd, T: Instance, const C: usize> {
    pwm: &'a mut Pwm<'d, T>,
    _instance: PhantomData<T>,
}

impl<'a, 'd, T: Instance, const C: usize> PwmChannel<'a, 'd, T, C> {
    /// Return the current duty cycle.
    pub fn duty_cycle(&self) -> u16 {
        self.pwm.duty_cycle::<C>()
    }
}

impl<'a, 'd, T: Instance, const C: usize> embedded_hal_1::pwm::ErrorType for PwmChannel<'a, 'd, T, C> {
    type Error = Infallible;
}

impl<'a, 'd, T: Instance, const C: usize> embedded_hal_1::pwm::SetDutyCycle for PwmChannel<'a, 'd, T, C> {
    fn max_duty_cycle(&self) -> u16 {
        self.pwm.max_duty_cycle()
    }

    fn set_duty_cycle(&mut self, duty: u16) -> Result<(), Self::Error> {
        self.pwm.set_duty_cycle::<C>(duty);
        Ok(())
    }
}

/// FlexIO PWM configuration.
#[non_exhaustive]
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct FlexioConfig {
    /// Requested output frequency in hertz.
    ///
    /// FlexIO's PWM mode has two 8-bit counters. Frequencies from 15,625 Hz to 2 MHz preserve the
    /// full duty-cycle range when clocked from the 4 MHz fast internal reference.
    pub frequency: u32,
}

impl Default for FlexioConfig {
    fn default() -> Self {
        Self { frequency: 20_000 }
    }
}

/// PWM outputs generated by the FlexIO timers.
///
/// Each of the eight timers has an independent frequency and can drive any pin connected to the
/// FlexIO instance. The driver owns the entire FlexIO instance.
pub struct FlexioPwm<'d, T: FlexioInstance> {
    _peri: Peri<'d, T>,
    pins: [Option<Peri<'d, AnyPin>>; FLEXIO_TIMERS],
    pin_muxes: [Mux; FLEXIO_TIMERS],
    pin_numbers: [u8; FLEXIO_TIMERS],
    polarities: [Polarity; FLEXIO_TIMERS],
    period_ticks: [u16; FLEXIO_TIMERS],
    frequencies: [u32; FLEXIO_TIMERS],
    duties: [u16; FLEXIO_TIMERS],
}

impl<'d, T: FlexioInstance> FlexioPwm<'d, T> {
    /// Create a FlexIO PWM driver with all outputs disabled.
    pub fn new(peri: Peri<'d, T>) -> Self {
        crate::clocks::enable_flexio_clock();
        T::enable_clock();

        let regs = T::regs();
        regs.ctrl().write(|w| w.set_swrst(true));
        regs.ctrl().write(|w| {
            w.set_flexen(true);
            w.set_dbge(true);
        });

        Self {
            _peri: peri,
            pins: core::array::from_fn(|_| None),
            pin_muxes: [Mux::Mux0; FLEXIO_TIMERS],
            pin_numbers: [0; FLEXIO_TIMERS],
            polarities: [Polarity::ActiveHigh; FLEXIO_TIMERS],
            period_ticks: [0; FLEXIO_TIMERS],
            frequencies: [0; FLEXIO_TIMERS],
            duties: [0; FLEXIO_TIMERS],
        }
    }

    /// Configure a timer channel on `pin`, initially at 0% duty.
    pub fn enable_channel<const C: usize>(
        &mut self,
        pin: Peri<'d, impl FlexioPin<T>>,
        polarity: Polarity,
        config: FlexioConfig,
    ) {
        assert!(C < T::TIMERS, "timer does not exist on this FlexIO instance");
        assert!(self.pins[C].is_none(), "FlexIO PWM channel is already enabled");

        let (period_ticks, frequency) = flexio_timing(config.frequency);
        let pin_number = pin.flexio_pin();
        let pin_mux = pin.alt();

        // Keep the output inactive as GPIO until a nonzero PWM duty is requested.
        set_gpio_level(&*pin, polarity == Polarity::ActiveLow);
        pin.pcr().modify(|w| w.set_mux(Mux::Mux1));

        let regs = T::regs();
        regs.timctl(C).write(|w| w.set_timod(Timod::_00));
        regs.timcfg(C).write(|w| {
            w.set_timout(Timout::_00);
            w.set_timdec(Timdec::_00);
            w.set_timrst(Timrst::_000);
            w.set_timdis(Timdis::_000);
            w.set_timena(Timena::_000);
        });

        self.pin_muxes[C] = pin_mux;
        self.pin_numbers[C] = pin_number;
        self.polarities[C] = polarity;
        self.period_ticks[C] = period_ticks;
        self.frequencies[C] = frequency;
        self.duties[C] = 0;
        self.pins[C] = Some(pin.into());
    }

    /// Disable a timer channel and disconnect its pin from FlexIO.
    pub fn disable_channel<const C: usize>(&mut self) {
        self.check_flexio_channel::<C>();
        T::regs().timctl(C).write(|w| w.set_timod(Timod::_00));
        if let Some(pin) = self.pins[C].take() {
            pin.pcr().modify(|w| w.set_mux(Mux::Mux0));
        }
        self.period_ticks[C] = 0;
    }

    /// Return the actual frequency of a configured channel in hertz.
    pub fn frequency<const C: usize>(&self) -> u32 {
        self.check_flexio_channel::<C>();
        self.frequencies[C]
    }

    /// Return the duty value corresponding to 100% for a configured channel.
    pub fn max_duty_cycle<const C: usize>(&self) -> u16 {
        self.check_flexio_channel::<C>();
        self.period_ticks[C]
    }

    /// Set the duty cycle for a configured channel.
    pub fn set_duty_cycle<const C: usize>(&mut self, duty: u16) {
        self.check_flexio_channel::<C>();
        let period = self.period_ticks[C];
        assert!(duty <= period, "PWM duty cycle exceeds its maximum");

        let regs = T::regs();
        let pin = self.pins[C].as_ref().unwrap();
        if duty == 0 || duty == period {
            let active = duty == period;
            let high = match self.polarities[C] {
                Polarity::ActiveHigh => active,
                Polarity::ActiveLow => !active,
            };
            set_gpio_level(&**pin, high);
            pin.pcr().modify(|w| w.set_mux(Mux::Mux1));
            regs.timctl(C).modify(|w| {
                w.set_timod(Timod::_00);
                w.set_pincfg(TimctlPincfg::_00);
            });
        } else {
            let active_ticks = duty;
            let inactive_ticks = period - duty;
            let compare = ((inactive_ticks - 1) << 8) | (active_ticks - 1);
            regs.timcmp(C).write(|w| w.set_cmp(compare));

            if self.duties[C] == 0 || self.duties[C] == period {
                // TIMCFG was written when the channel was enabled. Set TIMOD last, as required
                // by the FlexIO programming sequence.
                regs.timctl(C).write(|w| {
                    w.set_pinsel(self.pin_numbers[C]);
                    w.set_pinpol(self.polarities[C] == Polarity::ActiveLow);
                    w.set_pincfg(TimctlPincfg::_11);
                    w.set_timod(Timod::_10);
                });
                pin.pcr().modify(|w| w.set_mux(self.pin_muxes[C]));
            }
        }
        self.duties[C] = duty;
    }

    /// Return the current duty cycle for a configured channel.
    pub fn duty_cycle<const C: usize>(&self) -> u16 {
        self.check_flexio_channel::<C>();
        self.duties[C]
    }

    /// Borrow a configured output as an embedded-hal PWM channel.
    pub fn channel<const C: usize>(&mut self) -> FlexioPwmChannel<'_, 'd, T, C> {
        self.check_flexio_channel::<C>();
        FlexioPwmChannel { pwm: self }
    }

    fn check_flexio_channel<const C: usize>(&self) {
        assert!(C < T::TIMERS, "timer does not exist on this FlexIO instance");
        assert!(self.pins[C].is_some(), "FlexIO PWM channel is not enabled");
    }
}

impl<'d, T: FlexioInstance> Drop for FlexioPwm<'d, T> {
    fn drop(&mut self) {
        let regs = T::regs();
        regs.ctrl().write(|w| w.set_swrst(true));
        regs.ctrl().write(|_| {});
        for pin in self.pins.iter_mut().filter_map(Option::take) {
            pin.pcr().modify(|w| w.set_mux(Mux::Mux0));
        }
        T::disable_clock();
    }
}

/// A borrowed FlexIO PWM output implementing the embedded-hal duty-cycle API.
pub struct FlexioPwmChannel<'a, 'd, T: FlexioInstance, const C: usize> {
    pwm: &'a mut FlexioPwm<'d, T>,
}

impl<'a, 'd, T: FlexioInstance, const C: usize> embedded_hal_1::pwm::ErrorType for FlexioPwmChannel<'a, 'd, T, C> {
    type Error = Infallible;
}

impl<'a, 'd, T: FlexioInstance, const C: usize> embedded_hal_1::pwm::SetDutyCycle for FlexioPwmChannel<'a, 'd, T, C> {
    fn max_duty_cycle(&self) -> u16 {
        self.pwm.max_duty_cycle::<C>()
    }

    fn set_duty_cycle(&mut self, duty: u16) -> Result<(), Self::Error> {
        self.pwm.set_duty_cycle::<C>(duty);
        Ok(())
    }
}

fn set_gpio_level(pin: &impl SealedPin, high: bool) {
    let gpio = pin.pin_bank().gpio();
    if high {
        gpio.psor().write(|w| w.set_ptso(pin.pin_number() as usize, true));
    } else {
        gpio.pcor().write(|w| w.set_ptco(pin.pin_number() as usize, true));
    }
    gpio.pddr().modify(|w| w.set_pdd(pin.pin_number() as usize, true));
}

fn flexio_timing(requested: u32) -> (u16, u32) {
    assert!(
        (15_625..=2_000_000).contains(&requested),
        "FlexIO PWM frequency must be from 15,625 Hz to 2 MHz"
    );

    let floor = crate::clocks::FLEXIO_CLOCK_HZ / requested;
    let mut best: Option<(u64, u16)> = None;
    for ticks in [floor, floor + 1] {
        let ticks = ticks.clamp(2, 256) as u16;
        let error = u64::from(crate::clocks::FLEXIO_CLOCK_HZ).abs_diff(u64::from(requested) * u64::from(ticks));
        if best.is_none_or(|(best_error, best_ticks)| {
            error * u64::from(best_ticks) < best_error * u64::from(ticks)
                || (error * u64::from(best_ticks) == best_error * u64::from(ticks) && ticks > best_ticks)
        }) {
            best = Some((error, ticks));
        }
    }
    let (_, ticks) = best.unwrap();
    (ticks, crate::clocks::FLEXIO_CLOCK_HZ / u32::from(ticks))
}

pub(crate) trait SealedFlexioInstance {
    const TIMERS: usize;
    fn regs() -> FlexioRegisters;
    fn enable_clock();
    fn disable_clock();
}

/// A FlexIO instance usable for PWM generation.
#[allow(private_bounds)]
pub trait FlexioInstance: SealedFlexioInstance + PeripheralType {}

macro_rules! impl_flexio_pwm_instance {
    ($inst:ident, $timers:expr) => {
        impl crate::pwm::SealedFlexioInstance for crate::peripherals::$inst {
            const TIMERS: usize = $timers;

            fn regs() -> crate::pac::flexio::Flexio {
                crate::pac::$inst
            }

            fn enable_clock() {
                crate::clocks::enable::<crate::peripherals::$inst>();
            }

            fn disable_clock() {
                crate::clocks::disable::<crate::peripherals::$inst>();
            }
        }

        impl crate::pwm::FlexioInstance for crate::peripherals::$inst {}
    };
}

pub(crate) trait SealedFlexioPin<T: FlexioInstance>: crate::gpio::Pin {
    fn flexio_pin(&self) -> u8;
    fn alt(&self) -> Mux;
}

/// A pin connected to a FlexIO instance.
#[allow(private_bounds)]
pub trait FlexioPin<T: FlexioInstance>: SealedFlexioPin<T> + crate::gpio::Pin {}

macro_rules! impl_flexio_pwm_pin {
    ($pin:ident, $inst:ident, $number:expr, $alt:ident) => {
        impl crate::pwm::SealedFlexioPin<crate::peripherals::$inst> for crate::peripherals::$pin {
            fn flexio_pin(&self) -> u8 {
                $number
            }

            fn alt(&self) -> crate::pac::port::vals::Mux {
                crate::pac::port::vals::Mux::$alt
            }
        }

        impl crate::pwm::FlexioPin<crate::peripherals::$inst> for crate::peripherals::$pin {}
    };
}

fn timing(requested: u32) -> (Ps, u16, u32) {
    assert!(
        (1..=2_000_000).contains(&requested),
        "PWM frequency must be from 1 Hz to 2 MHz"
    );

    // Compare exact rational errors. For each prescaler only the two periods around the ideal
    // value can be best; prefer the larger period when errors are equal for greater resolution.
    let mut best: Option<(u64, u64, u16, u8)> = None;
    for ps_bits in 0..=7u8 {
        let divider = 1u64 << ps_bits;
        let target_period = crate::clocks::TPM_CLOCK_HZ as u64 / (requested as u64 * divider);
        for ticks in [target_period, target_period + 1] {
            let ticks = ticks.clamp(2, u16::MAX as u64);
            let denominator = divider * ticks;
            let target = requested as u64 * denominator;
            let error = (crate::clocks::TPM_CLOCK_HZ as u64).abs_diff(target);
            let better = best.is_none_or(|(best_error, best_denominator, best_ticks, _)| {
                error as u128 * (best_denominator as u128) < best_error as u128 * denominator as u128
                    || (error as u128 * (best_denominator as u128) == best_error as u128 * denominator as u128
                        && ticks > best_ticks as u64)
            });
            if better {
                best = Some((error, denominator, ticks as u16, ps_bits));
            }
        }
    }

    let (_, denominator, ticks, ps_bits) = best.unwrap();
    (
        Ps::from_bits(ps_bits),
        ticks,
        crate::clocks::TPM_CLOCK_HZ / denominator as u32,
    )
}

pub(crate) trait SealedInstance {
    const CHANNELS: usize;
    fn enable_clock();
    fn disable_clock();
    fn configure(prescaler: Ps, period_ticks: u16);
    fn stop();
    fn configure_channel(channel: usize, polarity: Polarity);
    fn disable_channel(channel: usize);
    fn set_duty_cycle(channel: usize, duty: u16);
    fn duty_cycle(channel: usize) -> u16;
}

/// A Timer/PWM Module instance.
#[allow(private_bounds)]
pub trait Instance: SealedInstance + PeripheralType {}

macro_rules! impl_pwm_instance {
    ($inst:ident, $channels:expr) => {
        impl crate::pwm::SealedInstance for crate::peripherals::$inst {
            const CHANNELS: usize = $channels;

            fn enable_clock() {
                crate::clocks::enable::<crate::peripherals::$inst>();
            }

            fn disable_clock() {
                crate::clocks::disable::<crate::peripherals::$inst>();
            }

            fn configure(prescaler: crate::pac::tpm::vals::Ps, period_ticks: u16) {
                let regs = crate::pac::$inst;
                // Stop the counter while setting the shared period and clearing old channel
                // modes.
                regs.sc().write(|w| {
                    w.set_cmod(crate::pac::tpm::vals::Cmod::_00);
                    w.set_tof(true);
                });
                for channel in 0..Self::CHANNELS {
                    regs.csc(channel).write(|w| w.set_chf(true));
                    regs.cv(channel).write(|w| w.set_val(0));
                }
                regs.cnt().write(|w| w.set_count(0));
                regs.mod_().write(|w| w.set_mod_(period_ticks - 1));
                regs.sc().write(|w| {
                    w.set_ps(prescaler);
                    w.set_cmod(crate::pac::tpm::vals::Cmod::_01);
                });
            }

            fn stop() {
                let regs = crate::pac::$inst;
                regs.sc().modify(|w| w.set_cmod(crate::pac::tpm::vals::Cmod::_00));
                for channel in 0..Self::CHANNELS {
                    regs.csc(channel).write(|w| w.set_chf(true));
                }
            }

            fn configure_channel(channel: usize, polarity: crate::pwm::Polarity) {
                let regs = crate::pac::$inst;
                regs.cv(channel).write(|w| w.set_val(0));
                regs.csc(channel).write(|w| {
                    w.set_msb(true);
                    match polarity {
                        crate::pwm::Polarity::ActiveHigh => w.set_elsb(true),
                        crate::pwm::Polarity::ActiveLow => w.set_elsa(true),
                    }
                    w.set_chf(true);
                });
            }

            fn disable_channel(channel: usize) {
                crate::pac::$inst.csc(channel).write(|w| w.set_chf(true));
            }

            fn set_duty_cycle(channel: usize, duty: u16) {
                crate::pac::$inst.cv(channel).write(|w| w.set_val(duty));
            }

            fn duty_cycle(channel: usize) -> u16 {
                crate::pac::$inst.cv(channel).read().val()
            }
        }

        impl crate::pwm::Instance for crate::peripherals::$inst {}
    };
}

pub(crate) trait SealedChannelPin<T: Instance, const C: usize>: crate::gpio::Pin {
    fn alt(&self) -> Mux;
}

/// A pin that can carry channel `C` of TPM instance `T`.
#[allow(private_bounds)]
pub trait ChannelPin<T: Instance, const C: usize>: SealedChannelPin<T, C> + crate::gpio::Pin {}

macro_rules! impl_pwm_pin {
    ($pin:ident, $inst:ident, $channel:expr, $alt:ident) => {
        impl crate::pwm::SealedChannelPin<crate::peripherals::$inst, $channel> for crate::peripherals::$pin {
            fn alt(&self) -> crate::pac::port::vals::Mux {
                crate::pac::port::vals::Mux::$alt
            }
        }

        impl crate::pwm::ChannelPin<crate::peripherals::$inst, $channel> for crate::peripherals::$pin {}
    };
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn exact_frequency_prefers_highest_resolution() {
        let (prescaler, ticks, actual) = timing(1_000);
        assert_eq!(prescaler, Ps::_000);
        assert_eq!(ticks, 4_000);
        assert_eq!(actual, 1_000);
    }

    #[test]
    fn reaches_frequency_limits() {
        assert_eq!(timing(1).2, 1);
        assert_eq!(timing(2_000_000).2, 2_000_000);
    }

    #[test]
    fn flexio_pwm_uses_full_eight_bit_period() {
        assert_eq!(flexio_timing(15_625), (256, 15_625));
        assert_eq!(flexio_timing(2_000_000), (2, 2_000_000));
    }
}
