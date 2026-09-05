//! Initializing a peripheral must preserve another source's pending INTMUX interrupt.
#![no_std]
#![no_main]

teleprobe_meta::target!(b"frdm-kl82z");

use embassy_executor::Spawner;
use embassy_hal_internal::interrupt::InterruptExt;
use embassy_nxp::lpuart::{self, Lpuart};
use embassy_nxp::spi::{self, Spi};
use embassy_nxp::{bind_interrupts, pac, peripherals};
use embassy_nxp_mkl82z7_tests as _;

bind_interrupts!(struct Irqs {
    INTMUX0_0 => spi::InterruptHandler<peripherals::SPI1>, lpuart::InterruptHandler<peripherals::LPUART2>;
});

#[embassy_executor::main(executor = "embassy_nxp::executor::Executor", entry = "cortex_m_rt::entry")]
async fn main(_spawner: Spawner) {
    let p = embassy_nxp::init(Default::default());
    let (_spi, _uart) = cortex_m::interrupt::free(|_| {
        pac::Interrupt::INTMUX0_0.pend();
        let spi = Spi::new(p.SPI1, p.PTD5, p.PTD6, p.PTD7, Irqs, spi::Config::default());
        assert!(
            pac::Interrupt::INTMUX0_0.is_pending(),
            "SPI cleared a shared pending interrupt"
        );
        let uart = Lpuart::new(p.LPUART2, p.PTD3, p.PTD2, Irqs, lpuart::Config::default());
        assert!(
            pac::Interrupt::INTMUX0_0.is_pending(),
            "UART cleared a shared pending interrupt"
        );
        (spi, uart)
    });
    embassy_nxp_mkl82z7_tests::pass()
}
