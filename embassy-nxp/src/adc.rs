#![macro_use]
//! Analog to digital converter (ADC) driver

#[cfg_attr(lpc55, path = "./adc/lpc55.rs")]
#[cfg_attr(kinetis, path = "./adc/kinetis.rs")]
mod inner;
pub use inner::*;
