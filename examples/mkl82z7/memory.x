MEMORY
{
  /* MKL82Z128: 128 KiB program flash, 96 KiB SRAM.
     SRAM_L (0x1FFFA000..0x20000000) and SRAM_U (0x20000000..0x20012000) are
     contiguous, so they form one region. */
  FLASH : ORIGIN = 0x00000000, LENGTH = 128K
  RAM   : ORIGIN = 0x1FFFA000, LENGTH = 96K
}

/* Kinetis flash configuration field. The 16 bytes at 0x400..0x40F hold the
   backdoor key, flash protection, FSEC and FOPT. Whatever ends up here is
   what the chip boots with, so this section pins it to a known-good value
   (see src/lib.rs) instead of letting .text spill into it. */
_stext = ORIGIN(FLASH) + 0x410;

SECTIONS
{
  .flash_config ORIGIN(FLASH) + 0x400 :
  {
    KEEP(*(.flash_config));
  } > FLASH
} INSERT AFTER .vector_table;
