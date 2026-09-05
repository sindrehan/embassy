# MKL82Z7 examples

These examples target the NXP FRDM-KL82Z board. The default runner uses
probe-rs with the onboard J-Link-compatible OpenSDA probe:

```sh
cargo run --bin blinky
```

The runner connects under reset so that firmware which enters a low-power mode
can still be replaced.

Async examples use the Kinetis executor so clock recovery completes before an
interrupt handler runs after waking from VLPS.

The examples use the 1 MHz TPM time driver by default. To use the 1 kHz LPTMR
driver instead, disable the default feature and enable `time-driver-lptmr`:

```sh
cargo run --release --no-default-features --features time-driver-lptmr --bin lptmr_time
```

## Board connections

- The RGB LED is active low: red PTC1, green PTC2, blue PTC0.
- SW2 is active low on PTA4/NMI.
- SW3 is active low on PTD0/LLWU_P12.
- The OpenSDA virtual serial port is connected to LPUART0 on PTB17 (TX) and
  PTB16 (RX).
- The FXOS8700CQ accelerometer is connected to I2C0 on PTD2 (SCL) and PTD3
  (SDA), at address `0x1c`.
- SPI0 is available on PTC5 (SCK, D13), PTC6 (SOUT, D11), and PTC7 (SIN,
  D12). PTC4 (D10) is SPI0_PCS0.

The board uses a 12 MHz crystal. Examples which call `ClockConfig::pll()` run
the core at 72 MHz and the bus and flash clocks at 24 MHz. Other examples use
the reset clock configuration.

## Examples

- `blinky`: blinks the RGB LED with the PLL clock configuration.
- `pwm_rgb`: fades the RGB LED between colors. TPM0 drives red and green, and
  FlexIO timer 0 drives blue on PTC0/FXIO0_D12.
- `adc`: samples PTB0, VREFL, and VREFH with calibrated 16-bit conversions and
  32-sample hardware averaging. PTB0 is on J4 pin 12 (B6).
- `adc_low_power`: checks ADC completion after VLPS and wake-guard release on
  completion and cancellation. No external connections are required.
- `hello`: reports the configured clocks and runs a short CPU benchmark.
- `gpio_irq`: exercises GPIO edge and level waits; connect D11 to D12.
- `serial`: echoes bytes on the OpenSDA virtual serial port at 115200 baud.
- `lpuart_loopback`: tests interrupt-driven and blocking LPUART transfers with
  internal loopback enabled.
- `lpuart_dma`: tests DMA-backed LPUART transfers with internal loopback.
- `i2c_accel`: checks timeout recovery, then reads the onboard accelerometer
  with blocking, interrupt-driven, and DMA-backed I2C transfers.
- `spi_loopback`: tests blocking, interrupt-driven, and DMA-backed SPI
  transfers; connect D11 to D12.
- `spi_link_master` and `spi_link_slave`: exchange checked frames between two
  FRDM-KL82Z boards using SPI0 and SPI1.
- `intmux`: routes I2C1 and LPUART2 through an INTMUX channel.
- `sleep_modes`: exercises the idle sleep modes in RUN and VLPR.
- `vlps_pll`: runs at 72 MHz from the PLL, idles in VLPS, and verifies that
  each timer wake restores PEE before application code runs.
- `lptmr_time`: checks short alarms and the 16-bit counter extension while the
  executor idles in VLPS. Requires the `time-driver-lptmr` feature.
- `low_power`: enters VLPS, LLS3, and VLLS3, waking from SW3 or an LPTMR
  timeout. VLLS wakeup resets the MCU, and the example reports the retained
  wake source after restart. Requires the default `time-driver-tpm` feature
  because the LPTMR time driver owns both LPTMR instances.

The LPUART clock must remain active for asynchronous serial reception in STOP
or VLPS. The supplied configurations use the fast internal reference clock
when appropriate.

## Two-board SPI link

Remove the D11-to-D12 loopback jumper, if fitted, then connect the boards as
follows. Signal names are from the KL82 peripheral's point of view, so the
master and slave data pins cross.

| Master board | Slave board |
| --- | --- |
| GND | GND |
| D13 / PTC5 / SPI0_SCK | J22 pin 5 / PTD5 / SPI1_SCK |
| D11 / PTC6 / SPI0_SOUT | J22 pin 7 / PTD7 / SPI1_SIN |
| D12 / PTC7 / SPI0_SIN | J22 pin 6 / PTD6 / SPI1_SOUT |
| D10 / PTC4 / GPIO select | J22 pin 4 / PTD4 / SPI1_PCS0 |

Run `spi_link_slave` before `spi_link_master`, using a separate terminal for
each board:

```sh
cargo run --release --bin spi_link_slave
cargo run --release --bin spi_link_master
```

When both probes are connected, select each one by including its serial number
in `PROBE_RS_PROBE`, using the form `1366:1015:<serial>`.

The red LED toggles for every valid frame. RTT logs on the master report each
verified reply; replies are returned one exchange after their requests because
SPI shifts both directions at the same time.

## Flash configuration

`memory.x` reserves the flash configuration field at `0x400..0x40f`. The value
in `src/lib.rs` leaves the device unsecured, enables mass erase, boots from
internal flash, disables NMI, selects fast initialization, and enters RUN after
reset.
