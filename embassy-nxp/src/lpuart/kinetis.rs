//! LPUART driver for Kinetis.
//!
//! All LPUART instances are clocked from the 48 MHz IRC48M through `SIM_SOPT2[PLLFLLSEL]`, which
//! [`init`](crate::init) selects, so baud rates do not depend on the core clock configuration.
//!
//! The async API is interrupt driven: TX refills the FIFO from the `TDRE` interrupt and RX wakes
//! on `RDRF`, one interrupt per received byte. With DMA channels ([`Lpuart::new_with_dma`]) whole
//! buffers move without per-byte interrupts instead. LPUART0 and LPUART1 have 8-byte FIFOs;
//! LPUART2 has a single buffer, so in interrupt mode it only sustains reception at modest baud
//! rates (9600 is safe on the 21 MHz reset clock, 115200 overruns), while with DMA it keeps up.
//! LPUART2 reaches the NVIC through [INTMUX0](crate::intmux), so its handler is bound to
//! `INTMUX0_0`.
#![macro_use]

use core::future::poll_fn;
use core::marker::PhantomData;
use core::task::Poll;

use embassy_hal_internal::{Peri, PeripheralType};
use embassy_sync::waitqueue::AtomicWaker;
use embedded_io::ErrorKind;

use crate::interrupt::typelevel::{Binding, Interrupt};
use crate::pac::common::{RW, Reg};
use crate::pac::lpuart::Lpuart as Regs;
use crate::pac::port::regs::Pcr;
use crate::pac::SIM;
use crate::pac::lpuart::regs::{Data, Stat};
use crate::pac::port::vals::Mux;
use crate::dma::{AnyChannel, Channel};
use crate::pac::sim::vals::Lpuartsrc;
use crate::{Async, Blocking, Mode};

/// Write-1-to-clear flags in STAT: LBKDIF, RXEDGIF, IDLE, OR, NF, FE, PF, MA1F, MA2F.
const STAT_W1C: u32 = 0xC01F_C000;

/// Serial error.
#[derive(Debug, Eq, PartialEq, Copy, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum Error {
    /// The receive FIFO overflowed and data was lost.
    Overrun,
    /// The received character's parity did not match the configuration.
    Parity,
    /// The received character had no valid stop bit (also raised by a break).
    Framing,
    /// The receiver detected noise on the line.
    Noise,
    /// The DMA controller reported an error moving the data.
    Dma,
}

impl embedded_io::Error for Error {
    fn kind(&self) -> ErrorKind {
        match self {
            Error::Overrun => ErrorKind::Other,
            Error::Parity => ErrorKind::InvalidData,
            Error::Framing => ErrorKind::InvalidData,
            Error::Noise => ErrorKind::Other,
            Error::Dma => ErrorKind::Other,
        }
    }
}

impl core::fmt::Display for Error {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        core::fmt::Debug::fmt(self, f)
    }
}

impl core::error::Error for Error {}

/// Parity bit.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Parity {
    /// No parity.
    ParityNone,
    /// Even parity.
    ParityEven,
    /// Odd parity.
    ParityOdd,
}

/// Stop bits.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum StopBits {
    /// 1 stop bit.
    Stop1,
    /// 2 stop bits.
    Stop2,
}

/// UART configuration. Frames always carry 8 data bits.
#[non_exhaustive]
#[derive(Clone, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Config {
    /// Baud rate. `init` panics if the closest achievable rate is more than 3% off.
    pub baudrate: u32,
    /// Stop bits.
    pub stop_bits: StopBits,
    /// Parity bit.
    pub parity: Parity,
    /// Invert the TX pin output.
    pub invert_tx: bool,
    /// Invert the RX pin input.
    pub invert_rx: bool,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            baudrate: 115200,
            stop_bits: StopBits::Stop1,
            parity: Parity::ParityNone,
            invert_tx: false,
            invert_rx: false,
        }
    }
}

/// Per-instance constants.
pub struct Info {
    pub(crate) regs: Regs,
}

/// Per-instance wakers.
pub struct State {
    tx_waker: AtomicWaker,
    rx_waker: AtomicWaker,
}

impl State {
    pub const fn new() -> Self {
        Self {
            tx_waker: AtomicWaker::new(),
            rx_waker: AtomicWaker::new(),
        }
    }
}

/// Bidirectional LPUART driver.
pub struct Lpuart<'d, M: Mode> {
    tx: LpuartTx<'d, M>,
    rx: LpuartRx<'d, M>,
}

/// Transmit half of an LPUART.
pub struct LpuartTx<'d, M: Mode> {
    info: &'static Info,
    state: &'static State,
    dma: Option<(Peri<'d, AnyChannel>, u8)>,
    _phantom: PhantomData<(&'d (), M)>,
}

/// Receive half of an LPUART.
pub struct LpuartRx<'d, M: Mode> {
    info: &'static Info,
    state: &'static State,
    dma: Option<(Peri<'d, AnyChannel>, u8)>,
    _phantom: PhantomData<(&'d (), M)>,
}

/// Oversampling ratio and baud rate modulo divisor for `baudrate` from the `src` clock, picking
/// the pair with the smallest error like the MCUXpresso SDK does. `None` if the error exceeds 3%.
fn baud_divisors(src: u32, baudrate: u32) -> Option<(u8, u16)> {
    let mut best: Option<(u32, u8, u16)> = None;

    for osr in 4..=32u32 {
        let mut sbr = (src / (baudrate * osr)).max(1);
        let rate = |sbr: u32| src / (osr * sbr);
        let mut diff = rate(sbr).abs_diff(baudrate);
        let diff_next = rate(sbr + 1).abs_diff(baudrate);
        if diff_next < diff {
            sbr += 1;
            diff = diff_next;
        }
        // SBR is a 13-bit field.
        if sbr > 0x1FFF {
            continue;
        }
        if best.is_none_or(|(best_diff, _, _)| diff <= best_diff) {
            best = Some((diff, osr as u8, sbr as u16));
        }
    }

    let (diff, osr, sbr) = best?;
    (diff <= baudrate / 100 * 3).then_some((osr, sbr))
}

/// Clock, baud, frame and pin setup shared by all constructors.
fn init<T: Instance>(tx: Option<(Reg<Pcr, RW>, Mux)>, rx: Option<(Reg<Pcr, RW>, Mux)>, config: &Config) {
    let regs = T::info().regs;

    // All LPUARTs share the clock select: the PLLFLLSEL clock, which `clocks::init` points at the
    // 48 MHz IRC48M.
    critical_section::with(|_| SIM.sopt2().modify(|w| w.set_lpuartsrc(Lpuartsrc::_01)));
    T::enable_clock();
    let src = crate::clocks::clocks().pllfll;

    let (osr, sbr) = match baud_divisors(src, config.baudrate) {
        Some(divisors) => divisors,
        None => panic!("LPUART: baud rate {} not achievable within 3%", config.baudrate),
    };

    // Everything off while configuring.
    regs.ctrl().write(|_| {});

    regs.baud().write(|w| {
        w.set_osr(osr - 1);
        w.set_sbr(sbr);
        // Sampling on both edges is required for oversampling ratios below 8.
        w.set_bothedge(osr < 8);
        w.set_sbns(config.stop_bits == StopBits::Stop2);
    });

    let mut stat = Stat(STAT_W1C);
    stat.set_rxinv(config.invert_rx);
    regs.stat().write_value(stat);

    regs.fifo().modify(|w| {
        w.set_txfe(true);
        w.set_rxfe(true);
        w.set_txflush(true);
        w.set_rxflush(true);
    });
    // TDRE when the TX FIFO is empty, RDRF as soon as one byte is in the RX FIFO.
    regs.water().write(|w| {
        w.set_txwater(0);
        w.set_rxwater(0);
    });

    if let Some((pcr, mux)) = tx {
        pcr.modify(|w| w.set_mux(mux));
    }
    if let Some((pcr, mux)) = rx {
        pcr.modify(|w| w.set_mux(mux));
    }

    regs.ctrl().write(|w| {
        // With parity the frame grows to 9 bits: 8 data plus the parity bit.
        w.set_m(config.parity != Parity::ParityNone);
        w.set_pe(config.parity != Parity::ParityNone);
        w.set_pt(config.parity == Parity::ParityOdd);
        w.set_txinv(config.invert_tx);
        w.set_te(tx.is_some());
        w.set_re(rx.is_some());
    });
}

/// Clear the write-1-to-clear STAT flags selected by `flags` without disturbing the others.
fn clear_stat(regs: Regs, flags: impl FnOnce(&mut Stat)) {
    let mut mask = Stat(0);
    flags(&mut mask);
    let mut stat = regs.stat().read();
    stat.0 = (stat.0 & !STAT_W1C) | (mask.0 & STAT_W1C);
    regs.stat().write_value(stat);
}

fn tx_fifo_size(regs: Regs) -> u8 {
    match regs.fifo().read().txfifosize().to_bits() {
        0 => 1,
        n => 1 << (n + 1),
    }
}

impl<'d, M: Mode> LpuartTx<'d, M> {
    fn new_inner<T: Instance>(dma: Option<(Peri<'d, AnyChannel>, u8)>) -> Self {
        Self {
            info: T::info(),
            state: T::state(),
            dma,
            _phantom: PhantomData,
        }
    }

    /// Write all bytes, blocking while the FIFO is full.
    pub fn blocking_write(&mut self, buffer: &[u8]) -> Result<(), Error> {
        let regs = self.info.regs;
        let size = tx_fifo_size(regs) as usize;
        let mut written = 0;

        while written < buffer.len() {
            while !regs.stat().read().tdre() {}
            let chunk = size.min(buffer.len() - written);
            for &byte in &buffer[written..written + chunk] {
                regs.data().write_value(Data(byte as u32));
            }
            written += chunk;
        }

        Ok(())
    }

    /// Block until every queued byte has left the shift register.
    pub fn blocking_flush(&mut self) -> Result<(), Error> {
        while !self.info.regs.stat().read().tc() {}
        Ok(())
    }
}

impl<'d> LpuartTx<'d, Blocking> {
    /// Create a transmit-only blocking driver.
    pub fn new_blocking<T: Instance>(_peri: Peri<'d, T>, tx: Peri<'d, impl TxPin<T>>, config: Config) -> Self {
        init::<T>(Some((tx.pcr(), tx.alt())), None, &config);
        Self::new_inner::<T>(None)
    }
}

impl<'d> LpuartTx<'d, Async> {
    /// Create a transmit-only async driver.
    pub fn new<T: InterruptInstance>(
        _peri: Peri<'d, T>,
        tx: Peri<'d, impl TxPin<T>>,
        _irq: impl Binding<T::Interrupt, InterruptHandler<T>>,
        config: Config,
    ) -> Self {
        init::<T>(Some((tx.pcr(), tx.alt())), None, &config);
        enable_interrupt::<T>();
        Self::new_inner::<T>(None)
    }

    /// Create a transmit-only async driver that moves data with a DMA channel.
    pub fn new_with_dma<T: InterruptInstance>(
        _peri: Peri<'d, T>,
        tx: Peri<'d, impl TxPin<T>>,
        _irq: impl Binding<T::Interrupt, InterruptHandler<T>>,
        tx_dma: Peri<'d, impl Channel>,
        config: Config,
    ) -> Self {
        init::<T>(Some((tx.pcr(), tx.alt())), None, &config);
        enable_interrupt::<T>();
        Self::new_inner::<T>(Some((tx_dma.into(), T::TX_DMA_REQUEST)))
    }

    /// Write all bytes: through the DMA channel if one was given, otherwise waiting on the TX
    /// FIFO interrupt while it is full.
    pub async fn write(&mut self, buffer: &[u8]) -> Result<(), Error> {
        let regs = self.info.regs;

        if let Some((channel, request)) = &mut self.dma {
            if buffer.is_empty() {
                return Ok(());
            }
            regs.baud().modify(|w| w.set_tdmae(true));
            let transfer = unsafe { crate::dma::write(channel.reborrow(), *request, buffer, regs.data().as_ptr() as *mut u8) };
            let result = transfer.await;
            regs.baud().modify(|w| w.set_tdmae(false));
            return result.map_err(|_| Error::Dma);
        }

        let size = tx_fifo_size(regs);
        let mut written = 0;

        while written < buffer.len() {
            while written < buffer.len() && regs.water().read().txcount() < size {
                regs.data().write_value(Data(buffer[written] as u32));
                written += 1;
            }
            if written == buffer.len() {
                break;
            }

            poll_fn(|cx| {
                self.state.tx_waker.register(cx.waker());
                if regs.stat().read().tdre() {
                    Poll::Ready(())
                } else {
                    // TDRE is level sensitive, so a FIFO that drained between the check and the
                    // arm still raises the interrupt right away. The handler masks it again.
                    critical_section::with(|_| regs.ctrl().modify(|w| w.set_tie(true)));
                    Poll::Pending
                }
            })
            .await;
        }

        Ok(())
    }

    /// Wait until every queued byte has left the shift register.
    pub async fn flush(&mut self) -> Result<(), Error> {
        let regs = self.info.regs;
        poll_fn(|cx| {
            self.state.tx_waker.register(cx.waker());
            if regs.stat().read().tc() {
                Poll::Ready(())
            } else {
                critical_section::with(|_| regs.ctrl().modify(|w| w.set_tcie(true)));
                Poll::Pending
            }
        })
        .await;
        Ok(())
    }
}

impl<'d, M: Mode> LpuartRx<'d, M> {
    fn new_inner<T: Instance>(dma: Option<(Peri<'d, AnyChannel>, u8)>) -> Self {
        Self {
            info: T::info(),
            state: T::state(),
            dma,
            _phantom: PhantomData,
        }
    }

    /// One byte from the FIFO if there is one, or a pending error.
    fn try_read_byte(&mut self) -> Option<Result<u8, Error>> {
        let regs = self.info.regs;

        if regs.stat().read().or() {
            clear_stat(regs, |s| s.set_or(true));
            return Some(Err(Error::Overrun));
        }
        if regs.water().read().rxcount() == 0 {
            return None;
        }

        // The error flags travel with each character through the FIFO.
        let data = regs.data().read();
        let error = if data.fretsc() {
            Some(Error::Framing)
        } else if data.paritye() {
            Some(Error::Parity)
        } else if data.noisy() {
            Some(Error::Noise)
        } else {
            None
        };
        if error.is_some() {
            clear_stat(regs, |s| {
                s.set_fe(true);
                s.set_pf(true);
                s.set_nf(true);
            });
        }

        Some(match error {
            Some(e) => Err(e),
            None => Ok(data.0 as u8),
        })
    }

    /// Fill the buffer, blocking until every byte has arrived.
    pub fn blocking_read(&mut self, buffer: &mut [u8]) -> Result<(), Error> {
        for slot in buffer {
            *slot = loop {
                if let Some(byte) = self.try_read_byte() {
                    break byte?;
                }
            };
        }
        Ok(())
    }
}

impl<'d> LpuartRx<'d, Blocking> {
    /// Create a receive-only blocking driver.
    pub fn new_blocking<T: Instance>(_peri: Peri<'d, T>, rx: Peri<'d, impl RxPin<T>>, config: Config) -> Self {
        init::<T>(None, Some((rx.pcr(), rx.alt())), &config);
        Self::new_inner::<T>(None)
    }
}

impl<'d> LpuartRx<'d, Async> {
    /// Create a receive-only async driver.
    pub fn new<T: InterruptInstance>(
        _peri: Peri<'d, T>,
        rx: Peri<'d, impl RxPin<T>>,
        _irq: impl Binding<T::Interrupt, InterruptHandler<T>>,
        config: Config,
    ) -> Self {
        init::<T>(None, Some((rx.pcr(), rx.alt())), &config);
        enable_interrupt::<T>();
        Self::new_inner::<T>(None)
    }

    /// Create a receive-only async driver that moves data with a DMA channel.
    pub fn new_with_dma<T: InterruptInstance>(
        _peri: Peri<'d, T>,
        rx: Peri<'d, impl RxPin<T>>,
        _irq: impl Binding<T::Interrupt, InterruptHandler<T>>,
        rx_dma: Peri<'d, impl Channel>,
        config: Config,
    ) -> Self {
        init::<T>(None, Some((rx.pcr(), rx.alt())), &config);
        enable_interrupt::<T>();
        Self::new_inner::<T>(Some((rx_dma.into(), T::RX_DMA_REQUEST)))
    }

    /// Fill the buffer: through the DMA channel if one was given, otherwise waiting on the RX
    /// interrupt for each byte. With DMA the per-character error flags are not available, so
    /// receive errors are reported from the status register once the buffer is full.
    pub async fn read(&mut self, buffer: &mut [u8]) -> Result<(), Error> {
        let regs = self.info.regs;

        if let Some((channel, request)) = &mut self.dma {
            if buffer.is_empty() {
                return Ok(());
            }
            clear_stat(regs, |s| {
                s.set_or(true);
                s.set_fe(true);
                s.set_pf(true);
                s.set_nf(true);
            });
            regs.baud().modify(|w| w.set_rdmae(true));
            let transfer = unsafe { crate::dma::read(channel.reborrow(), *request, regs.data().as_ptr() as *const u8, buffer) };
            let result = transfer.await;
            regs.baud().modify(|w| w.set_rdmae(false));
            result.map_err(|_| Error::Dma)?;
            let stat = regs.stat().read();
            let error = if stat.or() {
                Some(Error::Overrun)
            } else if stat.fe() {
                Some(Error::Framing)
            } else if stat.pf() {
                Some(Error::Parity)
            } else if stat.nf() {
                Some(Error::Noise)
            } else {
                None
            };
            if let Some(e) = error {
                clear_stat(regs, |s| {
                    s.set_or(true);
                    s.set_fe(true);
                    s.set_pf(true);
                    s.set_nf(true);
                });
                return Err(e);
            }
            return Ok(());
        }

        let mut filled = 0;

        while filled < buffer.len() {
            if let Some(byte) = self.try_read_byte() {
                buffer[filled] = byte?;
                filled += 1;
                continue;
            }

            poll_fn(|cx| {
                self.state.rx_waker.register(cx.waker());
                let stat = regs.stat().read();
                if stat.rdrf() || stat.or() {
                    Poll::Ready(())
                } else {
                    // Both flags are level sensitive; see `LpuartTx::write`.
                    critical_section::with(|_| {
                        regs.ctrl().modify(|w| {
                            w.set_rie(true);
                            w.set_orie(true);
                        })
                    });
                    Poll::Pending
                }
            })
            .await;
        }

        Ok(())
    }
}

/// Read one byte if one is waiting, without blocking. `Ok(None)` when the FIFO is empty.
impl<'d, M: Mode> LpuartRx<'d, M> {
    pub fn try_read(&mut self) -> Result<Option<u8>, Error> {
        self.try_read_byte().transpose()
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
        let state = T::state();
        let stat = regs.stat().read();
        let ctrl = regs.ctrl().read();

        // Every enable is masked again here; the waiting task re-arms what it still needs. The
        // thread side only modifies CTRL inside a critical section, so this read-modify-write
        // cannot race with it.
        if (ctrl.rie() && stat.rdrf()) || (ctrl.orie() && stat.or()) {
            regs.ctrl().modify(|w| {
                w.set_rie(false);
                w.set_orie(false);
            });
            state.rx_waker.wake();
        }
        if (ctrl.tie() && stat.tdre()) || (ctrl.tcie() && stat.tc()) {
            regs.ctrl().modify(|w| {
                w.set_tie(false);
                w.set_tcie(false);
            });
            state.tx_waker.wake();
        }
    }
}

impl<'d> Lpuart<'d, Blocking> {
    /// Create a blocking driver.
    pub fn new_blocking<T: Instance>(
        _peri: Peri<'d, T>,
        tx: Peri<'d, impl TxPin<T>>,
        rx: Peri<'d, impl RxPin<T>>,
        config: Config,
    ) -> Self {
        init::<T>(Some((tx.pcr(), tx.alt())), Some((rx.pcr(), rx.alt())), &config);
        Self {
            tx: LpuartTx::new_inner::<T>(None),
            rx: LpuartRx::new_inner::<T>(None),
        }
    }
}

impl<'d> Lpuart<'d, Async> {
    /// Create an async driver.
    pub fn new<T: InterruptInstance>(
        _peri: Peri<'d, T>,
        tx: Peri<'d, impl TxPin<T>>,
        rx: Peri<'d, impl RxPin<T>>,
        _irq: impl Binding<T::Interrupt, InterruptHandler<T>>,
        config: Config,
    ) -> Self {
        init::<T>(Some((tx.pcr(), tx.alt())), Some((rx.pcr(), rx.alt())), &config);
        enable_interrupt::<T>();
        Self {
            tx: LpuartTx::new_inner::<T>(None),
            rx: LpuartRx::new_inner::<T>(None),
        }
    }

    /// Create an async driver that moves data with DMA channels, one per direction.
    pub fn new_with_dma<T: InterruptInstance>(
        _peri: Peri<'d, T>,
        tx: Peri<'d, impl TxPin<T>>,
        rx: Peri<'d, impl RxPin<T>>,
        _irq: impl Binding<T::Interrupt, InterruptHandler<T>>,
        tx_dma: Peri<'d, impl Channel>,
        rx_dma: Peri<'d, impl Channel>,
        config: Config,
    ) -> Self {
        init::<T>(Some((tx.pcr(), tx.alt())), Some((rx.pcr(), rx.alt())), &config);
        enable_interrupt::<T>();
        Self {
            tx: LpuartTx::new_inner::<T>(Some((tx_dma.into(), T::TX_DMA_REQUEST))),
            rx: LpuartRx::new_inner::<T>(Some((rx_dma.into(), T::RX_DMA_REQUEST))),
        }
    }

    /// See [`LpuartTx::write`].
    pub async fn write(&mut self, buffer: &[u8]) -> Result<(), Error> {
        self.tx.write(buffer).await
    }

    /// See [`LpuartTx::flush`].
    pub async fn flush(&mut self) -> Result<(), Error> {
        self.tx.flush().await
    }

    /// See [`LpuartRx::read`].
    pub async fn read(&mut self, buffer: &mut [u8]) -> Result<(), Error> {
        self.rx.read(buffer).await
    }
}

impl<'d, M: Mode> Lpuart<'d, M> {
    /// See [`LpuartTx::blocking_write`].
    pub fn blocking_write(&mut self, buffer: &[u8]) -> Result<(), Error> {
        self.tx.blocking_write(buffer)
    }

    /// See [`LpuartTx::blocking_flush`].
    pub fn blocking_flush(&mut self) -> Result<(), Error> {
        self.tx.blocking_flush()
    }

    /// See [`LpuartRx::blocking_read`].
    pub fn blocking_read(&mut self, buffer: &mut [u8]) -> Result<(), Error> {
        self.rx.blocking_read(buffer)
    }

    /// See [`LpuartRx::try_read`].
    pub fn try_read(&mut self) -> Result<Option<u8>, Error> {
        self.rx.try_read()
    }

    /// Split into independently usable transmit and receive halves.
    pub fn split(self) -> (LpuartTx<'d, M>, LpuartRx<'d, M>) {
        (self.tx, self.rx)
    }

    /// Borrow the transmit and receive halves.
    pub fn split_ref(&mut self) -> (&mut LpuartTx<'d, M>, &mut LpuartRx<'d, M>) {
        (&mut self.tx, &mut self.rx)
    }
}

impl<'d> embedded_io::ErrorType for LpuartTx<'d, Blocking> {
    type Error = Error;
}

impl<'d> embedded_io::Write for LpuartTx<'d, Blocking> {
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        self.blocking_write(buf).map(|_| buf.len())
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        self.blocking_flush()
    }
}

impl<'d> embedded_io::ErrorType for LpuartRx<'d, Blocking> {
    type Error = Error;
}

impl<'d> embedded_io::Read for LpuartRx<'d, Blocking> {
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        if buf.is_empty() {
            return Ok(0);
        }
        // Block for the first byte, then take whatever else is already in the FIFO.
        self.blocking_read(&mut buf[..1])?;
        let mut n = 1;
        while n < buf.len() {
            match self.try_read_byte() {
                Some(Ok(byte)) => {
                    buf[n] = byte;
                    n += 1;
                }
                Some(Err(e)) => return Err(e),
                None => break,
            }
        }
        Ok(n)
    }
}

impl<'d> embedded_io::ErrorType for Lpuart<'d, Blocking> {
    type Error = Error;
}

impl<'d> embedded_io::Write for Lpuart<'d, Blocking> {
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        embedded_io::Write::write(&mut self.tx, buf)
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        embedded_io::Write::flush(&mut self.tx)
    }
}

impl<'d> embedded_io::Read for Lpuart<'d, Blocking> {
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        embedded_io::Read::read(&mut self.rx, buf)
    }
}

pub(crate) trait SealedInstance {
    /// DMAMUX request source for received data.
    const RX_DMA_REQUEST: u8;
    /// DMAMUX request source for transmit data.
    const TX_DMA_REQUEST: u8;
    fn info() -> &'static Info;
    fn state() -> &'static State;
    fn enable_clock();
    fn clock_enabled() -> bool;
}

/// An LPUART instance.
#[allow(private_bounds)]
pub trait Instance: SealedInstance + PeripheralType {}

/// An LPUART instance that can raise an interrupt, usable in async mode.
pub trait InterruptInstance: Instance {
    /// NVIC interrupt for this instance: its own line, or the INTMUX channel it is routed through.
    type Interrupt: Interrupt;
    /// Input number on INTMUX0 for instances without an NVIC line of their own.
    const INTMUX_SOURCE: Option<u8> = None;
}

macro_rules! impl_lpuart_instance {
    ($inst:ident, $rx_request:expr, $tx_request:expr) => {
        impl crate::lpuart::SealedInstance for crate::peripherals::$inst {
            const RX_DMA_REQUEST: u8 = $rx_request;
            const TX_DMA_REQUEST: u8 = $tx_request;

            fn info() -> &'static crate::lpuart::Info {
                static INFO: crate::lpuart::Info = crate::lpuart::Info {
                    regs: crate::pac::$inst,
                };
                &INFO
            }

            fn state() -> &'static crate::lpuart::State {
                static STATE: crate::lpuart::State = crate::lpuart::State::new();
                &STATE
            }

            fn enable_clock() {
                crate::clocks::enable::<crate::peripherals::$inst>();
            }

            fn clock_enabled() -> bool {
                crate::clocks::is_enabled::<crate::peripherals::$inst>()
            }
        }

        impl crate::lpuart::Instance for crate::peripherals::$inst {}
    };
}

macro_rules! impl_lpuart_interrupt {
    ($inst:ident, $irq:ident) => {
        impl crate::lpuart::InterruptInstance for crate::peripherals::$inst {
            type Interrupt = crate::interrupt::typelevel::$irq;
        }
    };
    ($inst:ident, $irq:ident, $source:expr) => {
        impl crate::lpuart::InterruptInstance for crate::peripherals::$inst {
            type Interrupt = crate::interrupt::typelevel::$irq;
            const INTMUX_SOURCE: Option<u8> = Some($source);
        }
    };
}

pub(crate) trait SealedTxPin<T: Instance>: crate::gpio::Pin {
    fn alt(&self) -> Mux;
}

pub(crate) trait SealedRxPin<T: Instance>: crate::gpio::Pin {
    fn alt(&self) -> Mux;
}

/// A pin that can carry `LPUARTn_TX`.
#[allow(private_bounds)]
pub trait TxPin<T: Instance>: SealedTxPin<T> + crate::gpio::Pin {}

/// A pin that can carry `LPUARTn_RX`.
#[allow(private_bounds)]
pub trait RxPin<T: Instance>: SealedRxPin<T> + crate::gpio::Pin {}

macro_rules! impl_lpuart_tx_pin {
    ($pin:ident, $inst:ident, $alt:expr) => {
        impl crate::lpuart::SealedTxPin<crate::peripherals::$inst> for crate::peripherals::$pin {
            fn alt(&self) -> crate::pac::port::vals::Mux {
                crate::pac::port::vals::Mux::from_bits($alt)
            }
        }
        impl crate::lpuart::TxPin<crate::peripherals::$inst> for crate::peripherals::$pin {}
    };
}

macro_rules! impl_lpuart_rx_pin {
    ($pin:ident, $inst:ident, $alt:expr) => {
        impl crate::lpuart::SealedRxPin<crate::peripherals::$inst> for crate::peripherals::$pin {
            fn alt(&self) -> crate::pac::port::vals::Mux {
                crate::pac::port::vals::Mux::from_bits($alt)
            }
        }
        impl crate::lpuart::RxPin<crate::peripherals::$inst> for crate::peripherals::$pin {}
    };
}
