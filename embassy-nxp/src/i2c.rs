//! Inter-Integrated Circuit (I2C) driver.
#![macro_use]

#[cfg_attr(kinetis, path = "./i2c/kinetis.rs")]
mod inner;
pub use inner::*;
