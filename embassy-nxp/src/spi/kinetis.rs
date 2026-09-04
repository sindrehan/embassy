//! SPI master driver for the Kinetis DSPI module.
//!
//! Master mode, 8-bit frames, clocked from the bus clock. Chip select is not driven by the
//! module; use a GPIO, for example through `embassy_embedded_hal`'s `SpiDevice`. SPI0 has a
//! 4-entry FIFO, SPI1 a single entry. The blocking and async APIs share one transfer routine:
//! the async driver sleeps on the "receive FIFO not empty" interrupt, the blocking driver
//! busy-polls the same future with [`embassy_futures::block_on`]. With two DMA channels
//! ([`Spi::new_with_dma`]) the FIFOs are fed and drained by DMA instead. SPI1 reaches the NVIC
//! through [INTMUX0](crate::intmux), so its handler is bound to `INTMUX0_0`.
#![macro_use]

use core::future::poll_fn;
use core::marker::PhantomData;
use core::task::Poll;

use embassy_hal_internal::drop::OnDrop;
use embassy_hal_internal::{Peri, PeripheralType};
use embassy_sync::waitqueue::AtomicWaker;
pub use embedded_hal_1::spi::{Phase, Polarity};

use crate::dma::{AnyChannel, Channel, MAX_TRANSFER};
use crate::interrupt::typelevel::{Binding, Interrupt};
use crate::pac::common::{RW, Reg};
use crate::pac::port::regs::Pcr;
use crate::pac::port::vals::Mux;
use crate::pac::spi::Spi as Regs;
use crate::pac::spi::regs::Sr;
use crate::pac::spi::vals::{Ctas, Pbr, Pcs};
use crate::{Async, Blocking, Mode};

/// Write-1-to-clear flags in SR: TCF, EOQF, TFUF, TFFF, RFOF, RFDF.
const SR_W1C: u32 = 0x9A0A_0000;
/// Byte clocked out when there is nothing to send.
const DUMMY: u8 = 0xFF;
/// Baud rate prescaler (`PBR`) and scaler (`BR`) values.
const PRESCALERS: [u32; 4] = [2, 3, 5, 7];
const SCALERS: [u32; 16] = [
    2, 4, 6, 8, 16, 32, 64, 128, 256, 512, 1024, 2048, 4096, 8192, 16384, 32768,
];

/// SPI error.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum Error {
    /// The receive FIFO overflowed.
    Overrun,
    /// The DMA controller reported an error moving the data.
    Dma,
}

impl embedded_hal_1::spi::Error for Error {
    fn kind(&self) -> embedded_hal_1::spi::ErrorKind {
        match self {
            Error::Overrun => embedded_hal_1::spi::ErrorKind::Overrun,
            Error::Dma => embedded_hal_1::spi::ErrorKind::Other,
        }
    }
}

impl core::fmt::Display for Error {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        core::fmt::Debug::fmt(self, f)
    }
}

impl core::error::Error for Error {}

/// Bit order.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum BitOrder {
    MsbFirst,
    LsbFirst,
}

/// SPI configuration.
#[non_exhaustive]
#[derive(Clone, Debug)]
pub struct Config {
    /// SCK frequency in Hz. The closest rate not above it is used.
    pub frequency: u32,
    pub phase: Phase,
    pub polarity: Polarity,
    pub bit_order: BitOrder,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            frequency: 1_000_000,
            phase: Phase::CaptureOnFirstTransition,
            polarity: Polarity::IdleLow,
            bit_order: BitOrder::MsbFirst,
        }
    }
}

/// Per-instance constants.
pub struct Info {
    pub(crate) regs: Regs,
    pub(crate) fifo_depth: u8,
}

/// Per-instance waker.
pub struct State {
    waker: AtomicWaker,
}

impl State {
    pub const fn new() -> Self {
        Self {
            waker: AtomicWaker::new(),
        }
    }
}

impl Default for State {
    fn default() -> Self {
        Self::new()
    }
}

/// SPI master driver.
pub struct Spi<'d, M: Mode> {
    info: &'static Info,
    state: &'static State,
    is_async: bool,
    dma: Option<Dma<'d>>,
    _phantom: PhantomData<(&'d (), M)>,
}

struct Dma<'d> {
    tx: Peri<'d, AnyChannel>,
    tx_request: u8,
    rx: Peri<'d, AnyChannel>,
    rx_request: u8,
}

/// `DBR`, `PBR`, `BR` for the highest SCK rate not above `frequency`, like the SDK.
fn baud_divisors(bus_hz: u32, frequency: u32) -> (bool, Pbr, u8) {
    assert!(frequency != 0, "SPI frequency must be greater than zero");
    let mut best: Option<(u32, bool, u8, u8)> = None;
    for (pbr, &prescaler) in PRESCALERS.iter().enumerate() {
        for (br, &scaler) in SCALERS.iter().enumerate() {
            for dbr in 1..=2u32 {
                let rate = (bus_hz as u64 * dbr as u64 / (prescaler as u64 * scaler as u64)) as u32;
                if rate <= frequency {
                    let error = frequency - rate;
                    if best.is_none_or(|(best_error, ..)| error < best_error) {
                        best = Some((error, dbr == 2, pbr as u8, br as u8));
                    }
                }
            }
        }
    }
    let best = best.unwrap_or_else(|| panic!("requested SPI frequency is below the hardware minimum"));
    (best.1, Pbr::from_bits(best.2), best.3)
}

fn init<T: Instance>(
    sck: (Reg<Pcr, RW>, Mux),
    mosi: Option<(Reg<Pcr, RW>, Mux)>,
    miso: Option<(Reg<Pcr, RW>, Mux)>,
    config: &Config,
) {
    let regs = T::info().regs;
    T::enable_clock();

    // Master, module enabled, FIFOs enabled, halted while configuring.
    regs.mcr().write(|w| {
        w.set_mstr(true);
        w.set_mdis(false);
        w.set_halt(true);
        w.set_clr_txf(true);
        w.set_clr_rxf(true);
    });

    let (dbr, pbr, br) = baud_divisors(crate::clocks::clocks().bus, config.frequency);
    regs.ctar(0).write(|w| {
        w.set_fmsz(7);
        w.set_cpol(config.polarity == Polarity::IdleHigh);
        w.set_cpha(config.phase == Phase::CaptureOnSecondTransition);
        w.set_lsbfe(config.bit_order == BitOrder::LsbFirst);
        w.set_dbr(dbr);
        w.set_pbr(pbr);
        w.set_br(br);
    });

    for (pcr, mux) in [Some(sck), mosi, miso].into_iter().flatten() {
        pcr.modify(|w| w.set_mux(mux));
    }

    regs.sr().write_value(Sr(SR_W1C));
    regs.mcr().modify(|w| w.set_halt(false));
}

impl<'d> Spi<'d, Blocking> {
    /// Create a blocking full-duplex master.
    pub fn new_blocking<T: Instance>(
        _peri: Peri<'d, T>,
        sck: Peri<'d, impl SckPin<T>>,
        mosi: Peri<'d, impl MosiPin<T>>,
        miso: Peri<'d, impl MisoPin<T>>,
        config: Config,
    ) -> Self {
        init::<T>(
            (sck.pcr(), sck.alt()),
            Some((mosi.pcr(), mosi.alt())),
            Some((miso.pcr(), miso.alt())),
            &config,
        );
        Self::new_inner::<T>(false)
    }

    /// Create a blocking transmit-only master (no MISO pin).
    pub fn new_blocking_txonly<T: Instance>(
        _peri: Peri<'d, T>,
        sck: Peri<'d, impl SckPin<T>>,
        mosi: Peri<'d, impl MosiPin<T>>,
        config: Config,
    ) -> Self {
        init::<T>((sck.pcr(), sck.alt()), Some((mosi.pcr(), mosi.alt())), None, &config);
        Self::new_inner::<T>(false)
    }

    /// Create a blocking receive-only master (no MOSI pin).
    pub fn new_blocking_rxonly<T: Instance>(
        _peri: Peri<'d, T>,
        sck: Peri<'d, impl SckPin<T>>,
        miso: Peri<'d, impl MisoPin<T>>,
        config: Config,
    ) -> Self {
        init::<T>((sck.pcr(), sck.alt()), None, Some((miso.pcr(), miso.alt())), &config);
        Self::new_inner::<T>(false)
    }
}

impl<'d> Spi<'d, Async> {
    /// Create an async full-duplex master.
    pub fn new<T: InterruptInstance>(
        _peri: Peri<'d, T>,
        sck: Peri<'d, impl SckPin<T>>,
        mosi: Peri<'d, impl MosiPin<T>>,
        miso: Peri<'d, impl MisoPin<T>>,
        _irq: impl Binding<T::Interrupt, InterruptHandler<T>>,
        config: Config,
    ) -> Self {
        init::<T>(
            (sck.pcr(), sck.alt()),
            Some((mosi.pcr(), mosi.alt())),
            Some((miso.pcr(), miso.alt())),
            &config,
        );
        enable_interrupt::<T>();
        Self::new_inner::<T>(true)
    }

    /// Create an async transmit-only master (no MISO pin).
    pub fn new_txonly<T: InterruptInstance>(
        _peri: Peri<'d, T>,
        sck: Peri<'d, impl SckPin<T>>,
        mosi: Peri<'d, impl MosiPin<T>>,
        _irq: impl Binding<T::Interrupt, InterruptHandler<T>>,
        config: Config,
    ) -> Self {
        init::<T>((sck.pcr(), sck.alt()), Some((mosi.pcr(), mosi.alt())), None, &config);
        enable_interrupt::<T>();
        Self::new_inner::<T>(true)
    }

    /// Create an async receive-only master (no MOSI pin).
    pub fn new_rxonly<T: InterruptInstance>(
        _peri: Peri<'d, T>,
        sck: Peri<'d, impl SckPin<T>>,
        miso: Peri<'d, impl MisoPin<T>>,
        _irq: impl Binding<T::Interrupt, InterruptHandler<T>>,
        config: Config,
    ) -> Self {
        init::<T>((sck.pcr(), sck.alt()), None, Some((miso.pcr(), miso.alt())), &config);
        enable_interrupt::<T>();
        Self::new_inner::<T>(true)
    }

    /// Create an async full-duplex master whose FIFOs are fed and drained by DMA. No interrupt
    /// binding is needed: completion comes from the DMA controller.
    pub fn new_with_dma<T: Instance>(
        _peri: Peri<'d, T>,
        sck: Peri<'d, impl SckPin<T>>,
        mosi: Peri<'d, impl MosiPin<T>>,
        miso: Peri<'d, impl MisoPin<T>>,
        tx_dma: Peri<'d, impl Channel>,
        rx_dma: Peri<'d, impl Channel>,
        config: Config,
    ) -> Self {
        init::<T>(
            (sck.pcr(), sck.alt()),
            Some((mosi.pcr(), mosi.alt())),
            Some((miso.pcr(), miso.alt())),
            &config,
        );
        let mut spi = Self::new_inner::<T>(true);
        spi.dma = Some(Dma {
            tx: tx_dma.into(),
            tx_request: T::TX_DMA_REQUEST,
            rx: rx_dma.into(),
            rx_request: T::RX_DMA_REQUEST,
        });
        spi
    }

    /// Clock `write` out and `read` in at the same time. The longer slice sets the length; the
    /// shorter one is padded with 0xFF on the way out or dropped on the way in.
    pub async fn transfer(&mut self, read: &mut [u8], write: &[u8]) -> Result<(), Error> {
        self.transfer_inner(read, write).await
    }

    /// Clock `data` out, replacing it with what comes in.
    pub async fn transfer_in_place(&mut self, data: &mut [u8]) -> Result<(), Error> {
        self.transfer_in_place_inner(data).await
    }

    /// Clock `data` out, discarding what comes in.
    pub async fn write(&mut self, data: &[u8]) -> Result<(), Error> {
        self.transfer_inner(&mut [], data).await
    }

    /// Clock 0xFF out and fill `data` with what comes in.
    pub async fn read(&mut self, data: &mut [u8]) -> Result<(), Error> {
        self.transfer_inner(data, &[]).await
    }
}

impl<'d, M: Mode> Spi<'d, M> {
    fn new_inner<T: Instance>(is_async: bool) -> Self {
        Self {
            info: T::info(),
            state: T::state(),
            is_async,
            dma: None,
            _phantom: PhantomData,
        }
    }

    /// See [`Spi::<Async>::transfer`](Spi::transfer).
    pub fn blocking_transfer(&mut self, read: &mut [u8], write: &[u8]) -> Result<(), Error> {
        embassy_futures::block_on(self.transfer_inner(read, write))
    }

    /// See [`Spi::<Async>::transfer_in_place`](Spi::transfer_in_place).
    pub fn blocking_transfer_in_place(&mut self, data: &mut [u8]) -> Result<(), Error> {
        embassy_futures::block_on(self.transfer_in_place_inner(data))
    }

    /// See [`Spi::<Async>::write`](Spi::write).
    pub fn blocking_write(&mut self, data: &[u8]) -> Result<(), Error> {
        embassy_futures::block_on(self.transfer_inner(&mut [], data))
    }

    /// See [`Spi::<Async>::read`](Spi::read).
    pub fn blocking_read(&mut self, data: &mut [u8]) -> Result<(), Error> {
        embassy_futures::block_on(self.transfer_inner(data, &[]))
    }

    /// Wait until the transmit FIFO has drained. Transfers already wait for every frame to come
    /// back, so this returns immediately after them.
    pub fn flush(&mut self) -> Result<(), Error> {
        while self.info.regs.sr().read().txctr() != 0 {}
        Ok(())
    }

    /// Wait for the receive FIFO to hold at least one frame.
    async fn wait_rx(&mut self) {
        let regs = self.info.regs;
        poll_fn(|cx| {
            if regs.sr().read().rxctr() != 0 {
                return Poll::Ready(());
            }
            if self.is_async {
                self.state.waker.register(cx.waker());
                // RFDF is level sensitive (receive FIFO not empty), so a frame that landed
                // between the check and this write still raises the interrupt.
                critical_section::with(|_| regs.rser().modify(|w| w.set_rfdf_re(true)));
            }
            Poll::Pending
        })
        .await
    }

    async fn transfer_in_place_inner(&mut self, data: &mut [u8]) -> Result<(), Error> {
        let regs = self.info.regs;
        let depth = self.info.fifo_depth as usize;
        let len = data.len();
        if len == 0 {
            return Ok(());
        }

        regs.mcr().modify(|w| {
            w.set_clr_txf(true);
            w.set_clr_rxf(true);
        });
        regs.sr().write_value(Sr(SR_W1C));

        // Keep the accesses sequenced through the one mutable slice. A received byte cannot
        // overwrite a byte that has not already been queued for transmission.
        let mut sent = 0;
        let mut received = 0;
        while received < len {
            while sent < len && sent - received < depth && (regs.sr().read().txctr() as usize) < depth {
                let byte = data[sent];
                regs.pushr().write(|w| {
                    w.set_txdata(byte as u16);
                    w.set_pcs(Pcs::from_bits(0));
                    w.set_ctas(Ctas::_000);
                });
                sent += 1;
            }

            self.wait_rx().await;

            while received < sent && regs.sr().read().rxctr() != 0 {
                data[received] = regs.popr().read().rxdata() as u8;
                regs.sr().write_value(Sr(0).with_rfdf());
                received += 1;
            }
        }

        if regs.sr().read().rfof() {
            regs.sr().write_value(Sr(0).with_rfof());
            return Err(Error::Overrun);
        }
        Ok(())
    }

    /// Full-duplex transfer with the DMA channels: one channel feeds PUSHR's data byte on every
    /// TFFF request, the other drains POPR on every RFDF request. Slices shorter than `len` are
    /// padded with 0xFF from a fixed source or drained into a sink.
    async fn transfer_dma(&mut self, read: &mut [u8], write: &[u8], len: usize) -> Result<(), Error> {
        let regs = self.info.regs;
        let dma = self.dma.as_mut().unwrap();

        // The command half of PUSHR (PCS, CTAS, CONT: all zero) persists across byte writes to
        // its data half, which is what the DMA does.
        unsafe { (regs.pushr().as_ptr() as *mut u16).add(1).write_volatile(0) };
        let pushr = regs.pushr().as_ptr() as *mut u8;
        let popr = regs.popr().as_ptr() as *const u8;
        static DUMMY_BYTE: u8 = DUMMY;

        let rx = unsafe {
            if read.len() == len {
                crate::dma::read(dma.rx.reborrow(), dma.rx_request, popr, read)
            } else {
                debug_assert!(read.is_empty());
                crate::dma::read_discard::<_, u8>(dma.rx.reborrow(), dma.rx_request, popr, len)
            }
        };
        let tx = unsafe {
            if write.len() == len {
                crate::dma::write(dma.tx.reborrow(), dma.tx_request, write, pushr)
            } else {
                debug_assert!(write.is_empty());
                crate::dma::write_repeated(dma.tx.reborrow(), dma.tx_request, &raw const DUMMY_BYTE, pushr, len)
            }
        };

        regs.rser().write(|w| {
            w.set_rfdf_re(true);
            w.set_rfdf_dirs(true);
            w.set_tfff_re(true);
            w.set_tfff_dirs(true);
        });
        let on_drop = OnDrop::new(move || regs.rser().write(|_| {}));
        let (rx, tx) = embassy_futures::join::join(rx, tx).await;
        regs.rser().write(|_| {});
        on_drop.defuse();

        if rx.is_err() || tx.is_err() {
            return Err(Error::Dma);
        }
        if regs.sr().read().rfof() {
            regs.sr().write_value(Sr(0).with_rfof());
            return Err(Error::Overrun);
        }
        Ok(())
    }

    async fn transfer_inner(&mut self, read: &mut [u8], write: &[u8]) -> Result<(), Error> {
        let regs = self.info.regs;
        let depth = self.info.fifo_depth as usize;
        let len = read.len().max(write.len());
        if len == 0 {
            return Ok(());
        }

        regs.mcr().modify(|w| {
            w.set_clr_txf(true);
            w.set_clr_rxf(true);
        });
        regs.sr().write_value(Sr(SR_W1C));

        if self.dma.is_some() {
            let mut offset = 0;
            while offset < len {
                let mut chunk_len = (len - offset).min(MAX_TRANSFER);
                if offset < read.len() {
                    chunk_len = chunk_len.min(read.len() - offset);
                }
                if offset < write.len() {
                    chunk_len = chunk_len.min(write.len() - offset);
                }

                let read_chunk = if offset < read.len() {
                    &mut read[offset..offset + chunk_len]
                } else {
                    &mut []
                };
                let write_chunk = if offset < write.len() {
                    &write[offset..offset + chunk_len]
                } else {
                    &[]
                };
                self.transfer_dma(read_chunk, write_chunk, chunk_len).await?;
                offset += chunk_len;
            }
            return Ok(());
        }

        let mut sent = 0;
        let mut received = 0;
        while received < len {
            // Keep at most `depth` frames in flight so the receive FIFO cannot overflow.
            while sent < len && sent - received < depth && (regs.sr().read().txctr() as usize) < depth {
                let byte = write.get(sent).copied().unwrap_or(DUMMY);
                regs.pushr().write(|w| {
                    w.set_txdata(byte as u16);
                    w.set_pcs(Pcs::from_bits(0));
                    w.set_ctas(Ctas::_000);
                });
                sent += 1;
            }

            self.wait_rx().await;

            while received < sent && regs.sr().read().rxctr() != 0 {
                let byte = regs.popr().read().rxdata() as u8;
                regs.sr().write_value(Sr(0).with_rfdf());
                if let Some(slot) = read.get_mut(received) {
                    *slot = byte;
                }
                received += 1;
            }
        }

        if regs.sr().read().rfof() {
            regs.sr().write_value(Sr(0).with_rfof());
            return Err(Error::Overrun);
        }
        Ok(())
    }
}

fn enable_interrupt<T: InterruptInstance>() {
    if let Some(source) = T::INTMUX_SOURCE {
        crate::intmux::enable_source(crate::intmux::CHANNEL, source);
    }
    T::Interrupt::unpend();
    unsafe { T::Interrupt::enable() };
}

/// Interrupt handler. Bind it with [`bind_interrupts!`](crate::bind_interrupts).
pub struct InterruptHandler<T: InterruptInstance> {
    _phantom: PhantomData<T>,
}

impl<T: InterruptInstance> crate::interrupt::typelevel::Handler<T::Interrupt> for InterruptHandler<T> {
    unsafe fn on_interrupt() {
        // On a shared INTMUX line this runs for other peripherals' interrupts too, possibly
        // before this instance exists; its registers bus-fault while the clock gate is closed.
        if T::INTMUX_SOURCE.is_some() && !T::clock_enabled() {
            return;
        }
        let regs = T::info().regs;
        if regs.rser().read().rfdf_re() && regs.sr().read().rfdf() {
            // The thread side only touches RSER inside a critical section, so this cannot race.
            regs.rser().modify(|w| w.set_rfdf_re(false));
            T::state().waker.wake();
        }
    }
}

trait SrWith {
    fn with_rfdf(self) -> Self;
    fn with_rfof(self) -> Self;
}

impl SrWith for Sr {
    fn with_rfdf(mut self) -> Self {
        self.set_rfdf(true);
        self
    }

    fn with_rfof(mut self) -> Self {
        self.set_rfof(true);
        self
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn baud_rate_does_not_exceed_request() {
        let bus_hz = 24_000_000;
        let requested = 1_000_000;
        let (dbr, pbr, br) = baud_divisors(bus_hz, requested);
        let rate = bus_hz * if dbr { 2 } else { 1 } / (PRESCALERS[pbr.to_bits() as usize] * SCALERS[br as usize]);
        assert!(rate <= requested);
    }

    #[test]
    #[should_panic(expected = "below the hardware minimum")]
    fn rejects_unrepresentable_low_baud_rate() {
        baud_divisors(24_000_000, 100);
    }
}

impl<'d, M: Mode> embedded_hal_1::spi::ErrorType for Spi<'d, M> {
    type Error = Error;
}

impl<'d, M: Mode> embedded_hal_1::spi::SpiBus<u8> for Spi<'d, M> {
    fn read(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
        self.blocking_read(words)
    }

    fn write(&mut self, words: &[u8]) -> Result<(), Self::Error> {
        self.blocking_write(words)
    }

    fn transfer(&mut self, read: &mut [u8], write: &[u8]) -> Result<(), Self::Error> {
        self.blocking_transfer(read, write)
    }

    fn transfer_in_place(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
        self.blocking_transfer_in_place(words)
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        Spi::flush(self)
    }
}

impl<'d> embedded_hal_async::spi::SpiBus<u8> for Spi<'d, Async> {
    async fn read(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
        self.transfer_inner(words, &[]).await
    }

    async fn write(&mut self, words: &[u8]) -> Result<(), Self::Error> {
        self.transfer_inner(&mut [], words).await
    }

    async fn transfer(&mut self, read: &mut [u8], write: &[u8]) -> Result<(), Self::Error> {
        self.transfer_inner(read, write).await
    }

    async fn transfer_in_place(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
        self.transfer_in_place_inner(words).await
    }

    async fn flush(&mut self) -> Result<(), Self::Error> {
        Spi::flush(self)
    }
}

pub(crate) trait SealedInstance {
    /// DMAMUX request source for received frames.
    const RX_DMA_REQUEST: u8;
    /// DMAMUX request source for transmit frames.
    const TX_DMA_REQUEST: u8;
    fn info() -> &'static Info;
    fn state() -> &'static State;
    fn enable_clock();
    fn clock_enabled() -> bool;
}

/// An SPI instance.
#[allow(private_bounds)]
pub trait Instance: SealedInstance + PeripheralType {}

/// An SPI instance that can raise an interrupt, usable in async mode.
pub trait InterruptInstance: Instance {
    /// NVIC interrupt for this instance: its own line, or the INTMUX channel it is routed through.
    type Interrupt: Interrupt;
    /// Input number on INTMUX0 for instances without an NVIC line of their own.
    const INTMUX_SOURCE: Option<u8> = None;
}

macro_rules! impl_spi_instance {
    ($inst:ident, $fifo_depth:expr, $rx_request:expr, $tx_request:expr) => {
        impl crate::spi::SealedInstance for crate::peripherals::$inst {
            const RX_DMA_REQUEST: u8 = $rx_request;
            const TX_DMA_REQUEST: u8 = $tx_request;

            fn info() -> &'static crate::spi::Info {
                static INFO: crate::spi::Info = crate::spi::Info {
                    regs: crate::pac::$inst,
                    fifo_depth: $fifo_depth,
                };
                &INFO
            }

            fn state() -> &'static crate::spi::State {
                static STATE: crate::spi::State = crate::spi::State::new();
                &STATE
            }

            fn enable_clock() {
                crate::clocks::enable::<crate::peripherals::$inst>();
            }

            fn clock_enabled() -> bool {
                crate::clocks::is_enabled::<crate::peripherals::$inst>()
            }
        }

        impl crate::spi::Instance for crate::peripherals::$inst {}
    };
}

macro_rules! impl_spi_interrupt {
    ($inst:ident, $irq:ident) => {
        impl crate::spi::InterruptInstance for crate::peripherals::$inst {
            type Interrupt = crate::interrupt::typelevel::$irq;
        }
    };
    ($inst:ident, $irq:ident, $source:expr) => {
        impl crate::spi::InterruptInstance for crate::peripherals::$inst {
            type Interrupt = crate::interrupt::typelevel::$irq;
            const INTMUX_SOURCE: Option<u8> = Some($source);
        }
    };
}

pub(crate) trait SealedSckPin<T: Instance>: crate::gpio::Pin {
    fn alt(&self) -> Mux;
}
pub(crate) trait SealedMosiPin<T: Instance>: crate::gpio::Pin {
    fn alt(&self) -> Mux;
}
pub(crate) trait SealedMisoPin<T: Instance>: crate::gpio::Pin {
    fn alt(&self) -> Mux;
}

/// A pin that can carry `SPIn_SCK`.
#[allow(private_bounds)]
pub trait SckPin<T: Instance>: SealedSckPin<T> + crate::gpio::Pin {}
/// A pin that can carry `SPIn_SOUT` (MOSI).
#[allow(private_bounds)]
pub trait MosiPin<T: Instance>: SealedMosiPin<T> + crate::gpio::Pin {}
/// A pin that can carry `SPIn_SIN` (MISO).
#[allow(private_bounds)]
pub trait MisoPin<T: Instance>: SealedMisoPin<T> + crate::gpio::Pin {}

macro_rules! impl_spi_pin {
    ($sealed:ident, $trait:ident, $pin:ident, $inst:ident, $alt:expr) => {
        impl crate::spi::$sealed<crate::peripherals::$inst> for crate::peripherals::$pin {
            fn alt(&self) -> crate::pac::port::vals::Mux {
                crate::pac::port::vals::Mux::from_bits($alt)
            }
        }
        impl crate::spi::$trait<crate::peripherals::$inst> for crate::peripherals::$pin {}
    };
}
macro_rules! impl_spi_sck_pin {
    ($pin:ident, $inst:ident, $alt:expr) => {
        impl_spi_pin!(SealedSckPin, SckPin, $pin, $inst, $alt);
    };
}
macro_rules! impl_spi_mosi_pin {
    ($pin:ident, $inst:ident, $alt:expr) => {
        impl_spi_pin!(SealedMosiPin, MosiPin, $pin, $inst, $alt);
    };
}
macro_rules! impl_spi_miso_pin {
    ($pin:ident, $inst:ident, $alt:expr) => {
        impl_spi_pin!(SealedMisoPin, MisoPin, $pin, $inst, $alt);
    };
}
