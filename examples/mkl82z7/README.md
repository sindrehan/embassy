# MKL82Z7 (Kinetis KL82) examples

Tested on the FRDM-KL82Z. The onboard OpenSDA debugger runs SEGGER J-Link
firmware (USB 1366:1015). `cargo run` uses probe-rs, which needs the MKL82Z7
family target (added to probe-rs in September 2026):

```sh
cargo run --bin blinky
cargo run --bin hello
```

The examples go through `embassy-nxp` with the `mkl82z7` feature. The HAL
disables the watchdog in `__pre_init`, `init` gates on the PORT clocks, and other peripheral
clocks are opened with `embassy_nxp::clocks::enable::<peripherals::X>()`. The
`time-driver-tpm` feature runs `embassy-time` at 1 MHz from TPM0, clocked by the
4 MHz fast internal reference, so it is independent of the core clock setup.

`Config::clocks` selects the system clocks. `hello` stays on the reset
configuration (FLL from the slow internal reference, 21 MHz core). `blinky`
uses `ClockConfig::pll` with the board's 12 MHz crystal: PLL at 144 MHz, 72 MHz
core, 24 MHz bus and flash. Both print the resulting clocks and a busy-loop
benchmark, which runs about 3.4 times faster on the PLL (72 over 21 MHz).

## Serial

`serial` echoes on LPUART0 (PTB17 TX, PTB16 RX, 115200 8N1), which the board
routes to the OpenSDA virtual COM port and to Arduino D1/D0. The J-Link OpenSDA
firmware enumerates a CDC ACM interface, `/dev/ttyACM0` on Linux (it needs the
`cdc_acm` module, which is missing until a reboot after a kernel update). A
quick check with pyserial at 115200: a `heartbeat N` line arrives every second
and anything sent comes back. For an external 3.3 V adapter
use LPUART1 on PTC4 (TX, Arduino D10 / J2 pin 6) and PTC3 (RX, Arduino D6 /
J1 pin 14) instead; the example says which two lines to change.

`lpuart_loopback` needs no wiring: it puts LPUART0 in internal loopback and
checks 64 bytes through the async API at 115200 and the blocking API at 9600,
then exits. LPUART2 has no NVIC vector of its own (it sits behind INTMUX0), so
it is blocking only.

## Chip quirks handled here

- **Flash configuration field at 0x400..0x40F.** `memory.x` places the
  `.flash_config` section from `src/lib.rs` there (FSEC = 0xFE unsecured,
  FOPT = 0x3D boot from flash) and starts `.text` at 0x410 so code never spills
  into it. FOPT = 0xFF would boot from the ROM bootloader; an FSEC other than
  0xFE secures the part.
- **Watchdog runs out of reset** with roughly a 0.5 s timeout, and there is a
  much tighter rule on top: after a debugger halts the core at the reset vector
  and resumes it, the first unlock word must be written within the 256 bus cycle
  watchdog configuration time or the chip resets (RM 28.4.2, "WCT"). Disabling
  it from `main` is already too late, so `embassy-nxp` does it from
  cortex-m-rt's `__pre_init` hook, before RAM initialisation. probe-rs exposes
  this; J-Link hides it because it disables the watchdog itself after reset.
- **RAM starts at 0x1FFFA000.** SRAM_L and SRAM_U are contiguous 96 KiB.
- **Recovering a reset-looping chip.** probe-rs has no Kinetis MDM-AP debug
  sequence, so while the chip keeps resetting (for example firmware that forgot
  the watchdog) the AHB-AP answers FAULT and probe-rs cannot connect. J-Link can:

  ```sh
  printf 'connect\nhalt\nloadfile target/thumbv6m-none-eabi/debug/hello 0x0\nr\ng\nexit\n' > /tmp/jl.txt
  JLinkExe -device MKL82Z128xxx7 -if SWD -speed 4000 -autoconnect 1 -CommandFile /tmp/jl.txt
  ```

## FRDM-KL82Z board

- RGB LED, active low: red PTC1, green PTC2, blue PTC0.
- SW2 on PTA4 (NMI), SW3 on PTD0 (LLWU_P12).
