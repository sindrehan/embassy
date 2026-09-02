# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

<!-- next-header -->
## Unreleased - ReleaseDate
- MKL82Z7 (Kinetis KL82): chip support with clock gating (`clocks`), GPIO, watchdog disable, a TPM time driver (`time-driver-tpm`) system clock configuration (FEI, PLL, BLPI) LPUART, I2C master and SPI (DSPI) master drivers (blocking and async, INTMUX0 for the instances without an NVIC line), eDMA with DMA transfers for LPUART, SPI and I2C, GPIO pin interrupts (`wait_for_*`, embedded-hal `Wait`), power modes (VLPR, idle STOP/VLPS, LLS/VLLS with LLWU wakeups)
- LPC55: blocking version of SPI
- Codegen using `nxp-pac` metadata
- LPC55: PWM simple
- LPC55: Move ALT definitions for USART to TX/RX pin impls. 
- LPC55: Remove internal match_iocon macro
- LPC55: DMA Controller and asynchronous version of USART
- Moved NXP LPC55S69 from `lpc55-pac` to `nxp-pac`
- First release with changelog.
