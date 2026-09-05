//! Reads the onboard FXOS8700CQ accelerometer over I2C0 (PTD2 SCL, PTD3 SDA).
#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_nxp::i2c::{Config, I2c, InterruptHandler};
use embassy_nxp::{bind_interrupts, peripherals};
use embassy_nxp_mkl82z7_examples as _;
use embassy_time::Timer;

bind_interrupts!(struct Irqs {
    I2C0 => InterruptHandler<peripherals::I2C0>;
});

const ADDRESS: u8 = 0x1c;
const OUT_X_MSB: u8 = 0x01;
const WHO_AM_I: u8 = 0x0d;
const XYZ_DATA_CFG: u8 = 0x0e;
const CTRL_REG1: u8 = 0x2a;

#[embassy_executor::main(executor = "embassy_nxp::executor::Executor", entry = "cortex_m_rt::entry")]
async fn main(_spawner: Spawner) {
    let p = embassy_nxp::init(Default::default());
    let mut i2c = I2c::new(p.I2C0, p.PTD2, p.PTD3, Irqs, Config::default());

    let mut id = [0; 1];
    i2c.write_read(ADDRESS, &[WHO_AM_I], &mut id).await.unwrap();
    defmt::info!("FXOS8700CQ ID: {:#04x}", id[0]);

    // Configure the range while in standby, then enable measurements.
    i2c.write(ADDRESS, &[CTRL_REG1, 0x00]).await.unwrap();
    i2c.write(ADDRESS, &[XYZ_DATA_CFG, 0x00]).await.unwrap(); // ±2 g
    i2c.write(ADDRESS, &[CTRL_REG1, 0x01]).await.unwrap();

    loop {
        Timer::after_millis(200).await;
        let mut raw = [0; 6];
        i2c.write_read(ADDRESS, &[OUT_X_MSB], &mut raw).await.unwrap();
        let axis = |i: usize| (i16::from_be_bytes([raw[i], raw[i + 1]]) >> 2) as i32 * 244 / 1000;
        defmt::info!("Acceleration (mg): x={} y={} z={}", axis(0), axis(2), axis(4));
    }
}
