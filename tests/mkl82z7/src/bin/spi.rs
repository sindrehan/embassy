//! SPI0 loopback on the FRDM-KL82Z Arduino header: SCK = PTC5 (D13),
//! SOUT = PTC6 (D11), SIN = PTC7 (D12). Jumper D11 to D12 so what goes out
//! comes back. Runs the async API at 1 MHz and the blocking API at 8 MHz, plus
//! write-only and read-only calls, then the DMA-fed driver with 64 bytes at
//! 1 MHz and 1024 bytes at 8 MHz, and exits with success when the data
//! matches. Without the jumper it reports the mismatch and exits with failure.
#![no_std]
#![no_main]

teleprobe_meta::target!(b"frdm-kl82z");

use embassy_executor::Spawner;
use embassy_nxp::spi::{self, Spi};
use embassy_nxp::{bind_interrupts, peripherals};
use embassy_nxp_mkl82z7_tests as _;
use embassy_time::{Duration, with_timeout};

bind_interrupts!(struct Irqs {
    SPI0 => spi::InterruptHandler<peripherals::SPI0>;
});

#[embassy_executor::main(executor = "embassy_nxp::executor::Executor", entry = "cortex_m_rt::entry")]
async fn main(_spawner: Spawner) {
    let mut p = embassy_nxp::init(Default::default());
    defmt::info!("spi loopback: SPI0 on PTC5/PTC6/PTC7 (D13/D11/D12), jumper D11-D12");

    let mut pattern = [0u8; 64];
    for (i, b) in pattern.iter_mut().enumerate() {
        *b = (i as u8).wrapping_mul(53).wrapping_add(7);
    }
    let mut ok = true;

    {
        let mut spi = Spi::new(
            p.SPI0.reborrow(),
            p.PTC5.reborrow(),
            p.PTC6.reborrow(),
            p.PTC7.reborrow(),
            Irqs,
            spi::Config::default(),
        );
        let mut got = [0u8; 64];
        with_timeout(Duration::from_millis(200), spi.transfer(&mut got, &pattern))
            .await
            .expect("SPI0 interrupt never arrived")
            .unwrap();
        if got == pattern {
            defmt::info!("async transfer, 64 bytes at 1 MHz: ok");
        } else {
            defmt::error!(
                "async transfer mismatch, first bytes {:#04x} (jumper D11-D12 missing?)",
                got[..4]
            );
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
    }

    // Blocking at 8 MHz on a fresh driver.
    let mut config = spi::Config::default();
    config.frequency = 8_000_000;
    {
        let mut spi = Spi::new_blocking(
            p.SPI0.reborrow(),
            p.PTC5.reborrow(),
            p.PTC6.reborrow(),
            p.PTC7.reborrow(),
            config,
        );
        let mut got = [0u8; 64];
        spi.blocking_transfer(&mut got, &pattern).unwrap();
        if got == pattern {
            defmt::info!("blocking transfer, 64 bytes at 8 MHz: ok");
        } else {
            defmt::error!("blocking transfer mismatch");
            ok = false;
        }
    }

    // DMA on both directions.
    {
        let mut spi = Spi::new_with_dma(
            p.SPI0.reborrow(),
            p.PTC5.reborrow(),
            p.PTC6.reborrow(),
            p.PTC7.reborrow(),
            p.DMA_CH0.reborrow(),
            p.DMA_CH1.reborrow(),
            spi::Config::default(),
        );
        let mut got = [0u8; 64];
        with_timeout(Duration::from_millis(200), spi.transfer(&mut got, &pattern))
            .await
            .expect("SPI0 DMA transfer timed out")
            .unwrap();
        if got == pattern {
            defmt::info!("dma transfer, 64 bytes at 1 MHz: ok");
        } else {
            defmt::error!("dma transfer mismatch, first bytes {:#04x}", got[..4]);
            ok = false;
        }
        spi.write(&pattern[..8]).await.unwrap();
        let mut ff = [0u8; 8];
        spi.read(&mut ff).await.unwrap();
        if ff != [0xFF; 8] {
            defmt::error!("dma read-only expected 0xFF, got {:#04x}", ff);
            ok = false;
        }
    }

    let mut config = spi::Config::default();
    config.frequency = 8_000_000;
    let mut spi = Spi::new_with_dma(p.SPI0, p.PTC5, p.PTC6, p.PTC7, p.DMA_CH0, p.DMA_CH1, config);
    let mut big = [0u8; 1024];
    for (i, b) in big.iter_mut().enumerate() {
        *b = (i as u8).wrapping_mul(17).wrapping_add(1);
    }
    let mut big_got = [0u8; 1024];
    let start = embassy_time::Instant::now();
    spi.transfer(&mut big_got, &big).await.unwrap();
    let elapsed = start.elapsed().as_micros();
    if big_got == big {
        defmt::info!("dma transfer, 1024 bytes at 8 MHz: ok in {} us", elapsed);
    } else {
        defmt::error!("dma 1024-byte transfer mismatch");
        ok = false;
    }

    if ok {
        defmt::info!("spi loopback passed");
        embassy_nxp_mkl82z7_tests::pass()
    } else {
        defmt::panic!("spi loopback failed");
    }
}
