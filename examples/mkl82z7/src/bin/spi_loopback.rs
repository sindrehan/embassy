//! SPI0 loopback on the FRDM-KL82Z Arduino header: SCK = PTC5 (D13),
//! SOUT = PTC6 (D11), SIN = PTC7 (D12). Jumper D11 to D12 so what goes out
//! comes back. Runs the async API at 1 MHz and the blocking API at 8 MHz, plus
//! write-only and read-only calls, and exits with success when the data
//! matches. Without the jumper it reports the mismatch and exits with failure.
#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_nxp::spi::{self, Spi};
use embassy_nxp::{bind_interrupts, peripherals};
use embassy_nxp_mkl82z7_examples as _;
use embassy_time::{Duration, with_timeout};

bind_interrupts!(struct Irqs {
    SPI0 => spi::InterruptHandler<peripherals::SPI0>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_nxp::init(Default::default());
    defmt::info!("spi loopback: SPI0 on PTC5/PTC6/PTC7 (D13/D11/D12), jumper D11-D12");

    let mut pattern = [0u8; 64];
    for (i, b) in pattern.iter_mut().enumerate() {
        *b = (i as u8).wrapping_mul(53).wrapping_add(7);
    }
    let mut ok = true;

    let mut spi = Spi::new(p.SPI0, p.PTC5, p.PTC6, p.PTC7, Irqs, spi::Config::default());
    let mut got = [0u8; 64];
    with_timeout(Duration::from_millis(200), spi.transfer(&mut got, &pattern))
        .await
        .expect("SPI0 interrupt never arrived")
        .unwrap();
    if got == pattern {
        defmt::info!("async transfer, 64 bytes at 1 MHz: ok");
    } else {
        defmt::error!("async transfer mismatch, first bytes {:#04x} (jumper D11-D12 missing?)", got[..4]);
        ok = false;
    }

    // In-place, then write-only and read-only (read-only clocks 0xFF out, so it reads back 0xFF).
    let mut inplace = pattern;
    spi.transfer_in_place(&mut inplace).await.unwrap();
    if inplace != pattern {
        defmt::error!("async in-place mismatch");
        ok = false;
    }
    spi.write(&pattern[..8]).await.unwrap();
    let mut ff = [0u8; 8];
    spi.read(&mut ff).await.unwrap();
    if ff != [0xFF; 8] {
        defmt::error!("read-only expected 0xFF, got {:#04x}", ff);
        ok = false;
    }
    drop(spi);

    // Blocking at 8 MHz on a fresh driver.
    let (spi0, sck, mosi, miso) = unsafe {
        (
            peripherals::SPI0::steal(),
            peripherals::PTC5::steal(),
            peripherals::PTC6::steal(),
            peripherals::PTC7::steal(),
        )
    };
    let mut config = spi::Config::default();
    config.frequency = 8_000_000;
    let mut spi = Spi::new_blocking(spi0, sck, mosi, miso, config);
    let mut got = [0u8; 64];
    spi.blocking_transfer(&mut got, &pattern).unwrap();
    if got == pattern {
        defmt::info!("blocking transfer, 64 bytes at 8 MHz: ok");
    } else {
        defmt::error!("blocking transfer mismatch");
        ok = false;
    }

    if ok {
        defmt::info!("spi loopback passed");
        embassy_nxp_mkl82z7_examples::exit()
    } else {
        defmt::panic!("spi loopback failed");
    }
}
