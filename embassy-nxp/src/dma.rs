#![macro_use]
//! Direct Memory Access (DMA) driver.

#[cfg_attr(lpc55, path = "./dma/lpc55.rs")]
#[cfg_attr(kinetis, path = "./dma/kinetis.rs")]
mod inner;
pub use inner::*;
