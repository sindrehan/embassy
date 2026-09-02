//! Serial Peripheral Interface (SPI) driver.
#![macro_use]

#[cfg_attr(lpc55, path = "./spi/lpc55.rs")]
#[cfg_attr(kinetis, path = "./spi/kinetis.rs")]
mod inner;
pub use inner::*;
