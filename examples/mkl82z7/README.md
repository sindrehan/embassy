# MKL82Z7 (Kinetis KL82) examples

Tested on the FRDM-KL82Z. The onboard OpenSDA debugger runs SEGGER J-Link
firmware (USB 1366:1015). probe-rs has no built-in KL82 target, so
`MKL82Z7.yaml` carries the chip description and `.cargo/config.toml` points
`cargo run` at it:

```sh
cargo run --bin blinky
cargo run --bin hello
```

These examples talk to the PAC directly. `embassy-nxp` cannot be used yet: its
build script needs the nxp-pac `metadata` (pins, signals, clock gates), which
does not exist for this chip so far.

## Chip quirks handled here

- **Flash configuration field at 0x400..0x40F.** `memory.x` places the
  `.flash_config` section from `src/lib.rs` there (FSEC = 0xFE unsecured,
  FOPT = 0x3D boot from flash) and starts `.text` at 0x410 so code never spills
  into it. FOPT = 0xFF would boot from the ROM bootloader; an FSEC other than
  0xFE secures the part.
- **Watchdog runs out of reset** from the bus clock with roughly a 0.5 s
  timeout. `init()` unlocks and disables it; every example calls it first.
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
