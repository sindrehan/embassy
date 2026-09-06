//! MKL82 ROM bootloader entry.

/// Transfer control to the ROM bootloader using its API tree (KL82 RM 7.3.5).
///
/// Disables external interrupts and SysTick, clears pending exceptions and deep sleep,
/// then calls `runBootloader(NULL)`. The ROM takes over RAM, clocks, pins and peripherals.
/// Its behavior, including enabled interfaces and detection timeout, is selected by the
/// Bootloader Configuration Area at flash offset `0x3c0`. It may restart the application
/// when detection times out. This function never resumes the calling application context.
/// An invalid BCA enables all ROM interfaces by default.
///
/// # Safety
///
/// - Call from privileged thread mode using the main stack, not an interrupt handler.
/// - Stop all DMA and other bus masters, and quiesce peripheral operations first. Disable
///   the watchdog. Interrupt masking alone does not stop DMA or peripheral outputs.
/// - Put external hardware in a safe state and release pins used by the enabled ROM
///   interfaces. The ROM can overwrite application RAM and change pin functions.
/// - Enter from RUN with a clock configuration supported by the ROM/BCA. In particular,
///   leave VLPR/HSRUN before calling. This helper does not restore reset clock settings.
pub unsafe fn enter_bootloader() -> ! {
    use cortex_m::peripheral::{NVIC, SCB, SYST};

    unsafe {
        cortex_m::interrupt::disable();
        // MKL82's Cortex-M0+ implements one bank of 32 external interrupts.
        (*NVIC::PTR).icer[0].write(u32::MAX);
        (*NVIC::PTR).icpr[0].write(u32::MAX);
        (*SYST::PTR).csr.write(0);
        SCB::clear_pendst();
        SCB::clear_pendsv();
        (*SCB::PTR).scr.write(0);
        cortex_m::asm::dsb();
        cortex_m::asm::isb();

        let tree = core::ptr::read_volatile(0x1c00_001c as *const *const usize);
        let entry = core::ptr::read_volatile(tree);
        let run: unsafe extern "C" fn(*const ()) = core::mem::transmute(entry);
        run(core::ptr::null());
    }
    SCB::sys_reset()
}
