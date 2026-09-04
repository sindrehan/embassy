# MKL82Z7 examples

These examples target the NXP FRDM-KL82Z board. The default runner uses
probe-rs with the onboard J-Link-compatible OpenSDA probe:

```sh
cargo run --bin blinky
```

The runner connects under reset so that firmware which enters a low-power mode
can still be replaced.

## Board connections

- The RGB LED is active low: red PTC1, green PTC2, blue PTC0.
- SW2 is active low on PTA4/NMI.
- SW3 is active low on PTD0/LLWU_P12.
- The OpenSDA virtual serial port is connected to LPUART0 on PTB17 (TX) and
  PTB16 (RX).
- The FXOS8700CQ accelerometer is connected to I2C0 on PTD2 (SCL) and PTD3
  (SDA), at address `0x1c`.
- SPI0 is available on PTC5 (SCK, D13), PTC6 (SOUT, D11), and PTC7 (SIN,
  D12). Connect D11 to D12 for the loopback example.

The board uses a 12 MHz crystal. Examples which call `ClockConfig::pll()` run
the core at 72 MHz and the bus and flash clocks at 24 MHz. Other examples use
the reset clock configuration.

## Examples

- `blinky`: blinks the RGB LED with the PLL clock configuration.
- `pwm_rgb`: fades the RGB LED between colors. TPM0 drives red and green, and
  FlexIO timer 0 drives blue on PTC0/FXIO0_D12.
- `hello`: reports the configured clocks and runs a short CPU benchmark.
- `gpio_irq`: exercises GPIO edge and level waits; connect D11 to D12.
- `serial`: echoes bytes on the OpenSDA virtual serial port at 115200 baud.
- `lpuart_loopback`: tests interrupt-driven and blocking LPUART transfers with
  internal loopback enabled.
- `lpuart_dma`: tests DMA-backed LPUART transfers with internal loopback.
- `i2c_accel`: reads the onboard accelerometer with blocking, interrupt-driven,
  and DMA-backed I2C transfers.
- `spi_loopback`: tests blocking, interrupt-driven, and DMA-backed SPI
  transfers; connect D11 to D12.
- `intmux`: routes I2C1 and LPUART2 through an INTMUX channel.
- `sleep_modes`: exercises the idle sleep modes in RUN and VLPR.
- `low_power`: enters VLPS, LLS3, and VLLS3, waking from SW3 or an LPTMR
  timeout. VLLS wakeup resets the MCU, and the example reports the retained
  wake source after restart.

The LPUART clock must remain active for asynchronous serial reception in STOP
or VLPS. The supplied configurations use the fast internal reference clock
when appropriate.

## Flash configuration

`memory.x` reserves the flash configuration field at `0x400..0x40f`. The value
in `src/lib.rs` leaves the device unsecured, enables mass erase, boots from
internal flash unless BOOTCFG0 requests the ROM updater, enables NMI, selects
fast initialization, and enters RUN after reset.
