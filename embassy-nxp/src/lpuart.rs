//! Low Power UART (LPUART) driver.
#![macro_use]

#[cfg_attr(kinetis, path = "./lpuart/kinetis.rs")]
mod inner;
pub use inner::*;
