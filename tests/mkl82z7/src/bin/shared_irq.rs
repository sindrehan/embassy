//! Initializing or dropping a peripheral preserves other sources on the shared INTMUX line.
#![no_std]
#![no_main]

teleprobe_meta::target!(b"frdm-kl82z");

use embassy_executor::Spawner;
use embassy_hal_internal::interrupt::InterruptExt;
use embassy_nxp::lpuart::{self, Lpuart};
use embassy_nxp::spi::{self, Spi};
use embassy_nxp::{bind_interrupts, clocks, pac, peripherals};
use embassy_nxp_mkl82z7_tests as _;

bind_interrupts!(struct Irqs {
    INTMUX0_0 => spi::InterruptHandler<peripherals::SPI1>, lpuart::InterruptHandler<peripherals::LPUART2>;
});

#[embassy_executor::main(executor = "embassy_nxp::executor::Executor", entry = "cortex_m_rt::entry")]
async fn main(_spawner: Spawner) {
    let mut p = embassy_nxp::init(Default::default());
    for drop_spi_first in [true, false] {
        let (spi, uart) = cortex_m::interrupt::free(|_| {
            pac::Interrupt::INTMUX0_0.pend();
            let spi = Spi::new(
                p.SPI1.reborrow(),
                p.PTD5.reborrow(),
                p.PTD6.reborrow(),
                p.PTD7.reborrow(),
                Irqs,
                spi::Config::default(),
            );
            assert!(
                pac::Interrupt::INTMUX0_0.is_pending(),
                "SPI cleared a shared pending interrupt"
            );
            let uart = Lpuart::new(
                p.LPUART2.reborrow(),
                p.PTD3.reborrow(),
                p.PTD2.reborrow(),
                Irqs,
                lpuart::Config::default(),
            );
            assert!(
                pac::Interrupt::INTMUX0_0.is_pending(),
                "UART cleared a shared pending interrupt"
            );
            (spi, uart)
        });
        cortex_m::interrupt::free(|_| {
            let mut uart = uart;
            let mut spi = spi;
            pac::Interrupt::INTMUX0_0.pend();
            if drop_spi_first {
                drop(spi);
                assert!(clocks::is_enabled::<peripherals::LPUART2>());
                uart.blocking_write(&[0x5a]).unwrap();
                uart.blocking_flush().unwrap();
                drop(uart);
            } else {
                drop(uart);
                assert!(clocks::is_enabled::<peripherals::SPI1>());
                spi.blocking_write(&[0xa5]).unwrap();
                drop(spi);
            }
            assert!(pac::Interrupt::INTMUX0_0.is_pending());
            assert!(pac::Interrupt::INTMUX0_0.is_enabled());
            assert_eq!(pac::INTMUX0.ch_ier_31_0(0).read().0, 0);
        });
        assert!(!clocks::is_enabled::<peripherals::LPUART2>());
        assert!(!clocks::is_enabled::<peripherals::SPI1>());
    }
    embassy_nxp_mkl82z7_tests::pass()
}
