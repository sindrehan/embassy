# FRDM-KL82Z hardware tests

Regression tests for `embassy-nxp` on the MKL82Z128VLK7. Each test is a separate
flash-linked image with assertions and a bounded host-side run. Successful tests
log `Test OK` and reach `hil_test_passed`; a panic, fault, lost connection or
timeout is a failure. The `park` and `spi_link_slave` binaries are fixtures, not
standalone tests.

## Run locally

Install Rust's `thumbv6m-none-eabi` target, `uv`, `probe-rs` with MKL82 support,
SEGGER's `JLinkGDBServerCLExe`, and `arm-none-eabi-gdb`. Put them on `PATH`.
The local runner requires Linux and a J-Link probe; it flashes with probe-rs and
checks completion with GDB because VLPS can interrupt live RTT access.

```sh
probe-rs list
uv run tests/mkl82z7/run.py --list
uv run tests/mkl82z7/run.py --probe VID:PID:SERIAL --group onboard
uv run tests/mkl82z7/run.py --probe VID:PID:SERIAL --group low-power
```

Replace the selectors with the full J-Link selectors from `probe-rs list`.
No probe is chosen implicitly. Each command overwrites the selected board's
firmware. Both TPM and LPTMR time drivers are tested by default; select one with
`--time-driver tpm` or `--time-driver lptmr`. Individual test names override the
group, for example:

```sh
uv run tests/mkl82z7/run.py adc adc_low_power --probe VID:PID:SERIAL
uv run tests/mkl82z7/run.py sleep_modes --vlpr --probe VID:PID:SERIAL
```

Build, flash, GDB and server logs, the exact ELF images, and a `summary.json` are
written under `tests/mkl82z7/target/hil/`. Any failure produces a nonzero exit
status. GDB logs contain a backtrace for faults; the runner does not stream RTT.
It terminates its debugger processes before returning. Leave the debug wiring
connected throughout functional tests; these tests do not measure sleep current.

## Fixtures and coverage

Disconnect unrelated peripherals and adapters. Select a group only after checking
its wiring; there is no combined group because the fixtures are incompatible.

| Group | Wiring | Checks |
| --- | --- | --- |
| `onboard` | Powered, unmodified devkit; PTB0 and PTC10/PTC11 free | ADC calibration/resolution/averaging and lifecycle, I²C transfers/recovery, UART IRQ/DMA loopback, split-half lifecycle and async I/O traits, DMA guards/reuse, INTMUX routing |
| `low-power` | No external signals on UART or LED pins | ADC wake guards, PLL restoration, idle sleep modes, LPTMR alarms and full counter wrap |
| `isolated` | PTC4–PTC7, PTC10/PTC11 and PTD4–PTD7 undriven | SPI master/slave cancellation and lifecycle, shared IRQ preservation, DMA-only SPI in-place completion, wake guards and slow-frame interrupt latency |
| `loopback` | Jumper D11/PTC6 to D12/PTC7; remove other SPI wiring | GPIO waits/cancellation, SPI blocking/IRQ/DMA transfers |
| `link` | Two boards wired below; no loopback jumper | IRQ and DMA masters, 260 exchanges each, in-place and unequal-length buffers, slave teardown/recreation and VLPS between frames |
| `low-leakage` | No external wake signals; TPM driver only | LLS3 timeout wake and VLLS3 reset wake |

The `low-leakage` test resets the core on VLLS exit. A debugger that cannot
preserve the completion breakpoint across that reset cannot verify this test;
a disconnect or timeout is not evidence of success.
After low-leakage debug loss, reflashing may require a target power cycle or
recovery with SEGGER's J-Link tools.

When a second board is wired in, pass `--peer-probe VID:PID:SERIAL`. The runner
first flashes an idle, high-impedance fixture to that board. This permits the
`isolated` tests with the peer still wired, provided there are no other drivers
or external pulls. To leave both boards idle:

```sh
uv run tests/mkl82z7/run.py --probe VID:PID:MASTER --peer-probe VID:PID:SLAVE --park
```

## Two-board SPI test

| Master (`--probe`) | Slave (`--peer-probe`) |
| --- | --- |
| GND | GND |
| D13 / PTC5 / SCK | J22 pin 5 / PTD5 / SCK |
| D11 / PTC6 / SOUT | J22 pin 7 / PTD7 / SIN |
| D12 / PTC7 / SIN | J22 pin 6 / PTD6 / SOUT |
| D10 / PTC4 / GPIO select | J22 pin 4 / PTD4 / PCS0 |

```sh
uv run tests/mkl82z7/run.py --group link \
  --probe VID:PID:MASTER --peer-probe VID:PID:SLAVE
```

The runner starts the slave before the master and parks both boards afterwards,
including after a test failure. Only the master reports the verdict. Replies
arrive one transaction after their requests; every reply after priming is checked.

## Build and CI

```sh
cargo build --release --manifest-path tests/mkl82z7/Cargo.toml \
  --target thumbv6m-none-eabi --features time-driver-tpm --bins
cargo build --release --manifest-path tests/mkl82z7/Cargo.toml \
  --target thumbv6m-none-eabi --features time-driver-lptmr --bins
uv run --python 3.11 python -m unittest discover -s tests/mkl82z7 -p test_runner.py
```

The Embassy build metadata covers both time drivers and the VLPR variant. The
binaries carry Teleprobe target metadata and use the usual `Test OK`/breakpoint
completion convention. The default Cargo runner is Teleprobe for single-image
tests with an appropriate fixture; use the Python runner for two-board sequencing
and J-Link low-power tests. The KL82 is not in the HIL farm, so `ci.sh` builds but
does not submit these artifacts for execution.

User-oriented examples are in [examples/mkl82z7](../../examples/mkl82z7).
