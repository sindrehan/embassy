//! Talks to the FXOS8700CQ accelerometer/magnetometer on the FRDM-KL82Z over
//! I2C0 (PTD2 = SCL, PTD3 = SDA, address 0x1C): checks WHO_AM_I, then streams
//! acceleration samples, first with the interrupt-driven driver and then with
//! the DMA-assisted one.
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

const FXOS8700: u8 = 0x1C;
const REG_OUT_X_MSB: u8 = 0x01;
const REG_SYSMOD: u8 = 0x0B;
const REG_WHO_AM_I: u8 = 0x0D;
const REG_XYZ_DATA_CFG: u8 = 0x0E;
const REG_CTRL_REG1: u8 = 0x2A;
const WHO_AM_I: u8 = 0xC7;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_nxp::init(Default::default());
    defmt::info!("i2c: FXOS8700CQ on I2C0 (PTD2 SCL, PTD3 SDA) at 100 kHz");

    let mut i2c = I2c::new(p.I2C0, p.PTD2, p.PTD3, Irqs, Config::default());

    // Blocking and async paths against the same register.
    let mut id = [0u8; 1];
    i2c.blocking_write_read(FXOS8700, &[REG_WHO_AM_I], &mut id).unwrap();
    defmt::info!("WHO_AM_I (blocking) = {:#04x}", id[0]);
    i2c.write_read(FXOS8700, &[REG_WHO_AM_I], &mut id).await.unwrap();
    defmt::info!("WHO_AM_I (async)    = {:#04x}", id[0]);
    defmt::assert_eq!(id[0], WHO_AM_I, "unexpected WHO_AM_I");

    // A NACK from an empty address must come back as an error, not hang.
    match i2c.write(0x7E, &[0]).await {
        Err(e) => defmt::info!("probe of 0x7E: {:?} (expected)", e),
        Ok(()) => defmt::warn!("probe of 0x7E unexpectedly acknowledged"),
    }

    // Standby, +-2 g, active at the default 800 Hz.
    i2c.write(FXOS8700, &[REG_CTRL_REG1, 0x00]).await.unwrap();
    i2c.write(FXOS8700, &[REG_XYZ_DATA_CFG, 0x00]).await.unwrap();
    i2c.write(FXOS8700, &[REG_CTRL_REG1, 0x01]).await.unwrap();

    for _ in 0..3 {
        Timer::after_millis(200).await;
        let mut raw = [0u8; 6];
        i2c.write_read(FXOS8700, &[REG_OUT_X_MSB], &mut raw).await.unwrap();
        // 14-bit left justified, 0.244 mg/LSB at +-2 g.
        let axis = |i: usize| (i16::from_be_bytes([raw[i], raw[i + 1]]) >> 2) as i32 * 244 / 1000;
        defmt::info!("accel mg: x={} y={} z={}", axis(0), axis(2), axis(4));
    }
    drop(i2c);

    // The same over DMA: 2-byte writes send the second byte by DMA, the 6-byte read moves four
    // bytes by DMA and the last two by hand.
    let (i2c0, scl, sda) = unsafe { (peripherals::I2C0::steal(), peripherals::PTD2::steal(), peripherals::PTD3::steal()) };
    let mut i2c = I2c::new_with_dma(i2c0, scl, sda, Irqs, p.DMA_CH0, Config::default());
    let mut id = [0u8; 1];
    i2c.write_read(FXOS8700, &[REG_WHO_AM_I], &mut id).await.unwrap();
    defmt::assert_eq!(id[0], WHO_AM_I, "unexpected WHO_AM_I over DMA driver");
    i2c.write(FXOS8700, &[REG_CTRL_REG1, 0x00]).await.unwrap();
    i2c.write(FXOS8700, &[REG_CTRL_REG1, 0x01]).await.unwrap();
    let mut ctrl = [0u8; 1];
    i2c.write_read(FXOS8700, &[REG_CTRL_REG1], &mut ctrl).await.unwrap();
    defmt::assert_eq!(ctrl[0] & 1, 1, "CTRL_REG1 write over DMA not applied");
    for _ in 0..3 {
        Timer::after_millis(200).await;
        let mut raw = [0u8; 6];
        i2c.write_read(FXOS8700, &[REG_OUT_X_MSB], &mut raw).await.unwrap();
        let axis = |i: usize| (i16::from_be_bytes([raw[i], raw[i + 1]]) >> 2) as i32 * 244 / 1000;
        defmt::info!("accel mg over dma: x={} y={} z={}", axis(0), axis(2), axis(4));
    }
    // A longer read to give the DMA a real run: 16 bytes from SYSMOD (0x0B) onwards. The
    // address pointer only wraps inside the output data registers, so from here it increments
    // linearly and WHO_AM_I (0x0D) lands at offset 2, XYZ_DATA_CFG (0x0E) at offset 3.
    let mut block = [0u8; 16];
    i2c.write_read(FXOS8700, &[REG_SYSMOD], &mut block).await.unwrap();
    defmt::assert_eq!(block[2], WHO_AM_I, "WHO_AM_I in the block read");
    defmt::assert_eq!(block[3], 0x00, "XYZ_DATA_CFG in the block read");
    defmt::info!("16-byte block read over dma ok: {:#04x}", block);

    defmt::info!("i2c accel passed");
    embassy_nxp_mkl82z7_examples::exit()
}
