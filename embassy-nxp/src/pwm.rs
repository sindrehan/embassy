#![macro_use]

//! Pulse-Width Modulation (PWM) driver.

#[cfg_attr(lpc55, path = "./pwm/lpc55.rs")]
#[cfg_attr(kinetis, path = "./pwm/kinetis.rs")]
mod inner;
pub use inner::*;
