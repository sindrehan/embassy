//! Slow SPI frames must not mask timer interrupts or block the executor while shifting.
#![no_std]
#![no_main]

teleprobe_meta::target!(b"frdm-kl82z");

use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_nxp::spi::{self, Spi};
use embassy_nxp::{bind_interrupts, peripherals};
use embassy_nxp_mkl82z7_tests as _;
use embassy_time::{Duration, Instant, Timer, with_timeout};

bind_interrupts!(struct Irqs {
    SPI0 => spi::InterruptHandler<peripherals::SPI0>;
});

#[embassy_executor::main(executor = "embassy_nxp::executor::Executor", entry = "cortex_m_rt::entry")]
async fn main(_spawner: Spawner) {
    let p = embassy_nxp::init(Default::default());
    let mut config = spi::Config::default();
    config.frequency = 100;
    let mut spi = Spi::new(p.SPI0, p.PTC5, p.PTC6, p.PTC7, Irqs, config);
    let tick = async {
        for _ in 0..100 {
            let start = Instant::now();
            Timer::after_millis(5).await;
            assert!(
                start.elapsed() < Duration::from_millis(25),
                "SPI blocked the executor or timer IRQ"
            );
        }
    };
    let (written, ()) = with_timeout(Duration::from_secs(2), join(spi.write(&[0x5a; 4]), tick))
        .await
        .unwrap();
    written.unwrap();
    embassy_nxp_mkl82z7_tests::pass()
}
