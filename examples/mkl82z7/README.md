# FRDM-KL82Z examples

Small examples for the NXP MKL82Z128VLK7 and Embassy.

## Run

Install the `thumbv6m-none-eabi` Rust target and a `probe-rs` build with MKL82
support. Connect the board's OpenSDA USB port, then:

```sh
cd examples/mkl82z7
cargo run --release --bin blinky
```

The runner connects over SWD under reset. With multiple probes connected, use
`probe-rs list` and set `PROBE_RS_PROBE=VID:PID:SERIAL` to select the board.

The default time driver uses TPM at 1 MHz. To use LPTMR at 1 kHz:

```sh
cargo run --release --no-default-features --features time-driver-lptmr --bin blinky
```

Async examples use the Kinetis executor, which restores the clocks before
interrupts run after a VLPS wakeup.

## Examples

| Example | Demonstrates | Connections |
| --- | --- | --- |
| `hello` | Periodic RTT logging | None |
| `blinky` | GPIO output and async delay | Onboard red LED |
| `gpio_irq` | Async button input | Onboard SW3 |
| `pwm_rgb` | RGB color fading with TPM and FlexIO PWM | Onboard RGB LED |
| `adc` | Calibrated ADC with 32-sample averaging | PTB0 on J4 pin 12: GND or 3.3 V |
| `serial` | Async UART echo at 115200 baud, 8N1 | OpenSDA virtual serial port |
| `i2c_accel` | Async accelerometer reads | Onboard FXOS8700CQ |
| `spi_loopback` | Async SPI transfer | Jumper D11 to D12; disconnect other SPI devices |
| `spi_link_master`, `spi_link_slave` | Two-board SPI request/reply | See below |
| `low_power` | VLPR with VLPS between timer wakes | Onboard red LED |

Hardware assertions, stress tests, DMA loopback and power-mode verification live
in [tests/mkl82z7](../../tests/mkl82z7).

## Board signals

- RGB LED, active low: red PTC1, green PTC2, blue PTC0.
- SW3, active low: PTD0.
- OpenSDA serial: PTB17 TX, PTB16 RX.
- FXOS8700CQ: I2C0 on PTD2 SCL and PTD3 SDA, address `0x1c`.

The board has a 12 MHz crystal. The default HAL configuration uses the reset
clock; `pwm_rgb` configures the PLL for a 72 MHz core. The shared flash
configuration leaves the device unsecured and boots internal flash in RUN with
NMI disabled.

## Two-board SPI link

Remove any D11-to-D12 loopback jumper, then connect:

| Master board | Slave board |
| --- | --- |
| GND | GND |
| D13 / PTC5 / SCK | J22 pin 5 / PTD5 / SCK |
| D11 / PTC6 / SOUT | J22 pin 7 / PTD7 / SIN |
| D12 / PTC7 / SIN | J22 pin 6 / PTD6 / SOUT |
| D10 / PTC4 / GPIO select | J22 pin 4 / PTD4 / PCS0 |

Flash `spi_link_slave` to the slave first, then run `spi_link_master` on the
master. Use a separate terminal with the appropriate `PROBE_RS_PROBE` for each
board. The master sends a counter; the slave adds one and returns it on the
**next** exchange, because SPI shifts both directions simultaneously. The first
reply is zero.

## Low power

VLPS can interrupt live SWD/RTT access. Build, flash and reset without a logging
session:

```sh
cargo build --release --bin low_power
probe-rs download --chip MKL82Z128VLK7 --protocol swd --connect-under-reset \
  target/thumbv6m-none-eabi/release/low_power
probe-rs reset --chip MKL82Z128VLK7 --protocol swd --connect-under-reset
```

The LED flashes briefly every two seconds. Disconnect the debugger before
measuring current.
