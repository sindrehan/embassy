//! Kinetis DSPI slave driver.
//!
//! Each transfer covers one active-low PCS0 assertion and completes when PCS0 is deasserted.
//! Receive and transmit buffers are upper bounds: extra received bytes are discarded, and the
//! configured over-read character is sent after the transmit buffer is exhausted.
#![macro_use]

use core::cell::RefCell;
use core::marker::PhantomData;

use critical_section::Mutex;
use embassy_hal_internal::drop::OnDrop;
use embassy_hal_internal::{Peri, PeripheralType};
pub use embedded_hal_1::spi::{Phase, Polarity};

use crate::gpio::Flex;
use crate::interrupt::typelevel::{Binding, Interrupt};
use crate::pac::common::{RW, Reg};
use crate::pac::port::regs::Pcr;
use crate::pac::port::vals::Mux;
use crate::pac::spi::Spi as Regs;
use crate::pac::spi::regs::Sr;
use crate::pac::spi::vals::Pcsis;
use crate::{Async, Blocking, Mode};

/// Write-1-to-clear flags in SR: TCF, EOQF, TFUF, TFFF, RFOF, RFDF.
const SR_W1C: u32 = 0x9A0A_0000;

/// SPI slave error.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum Error {
    /// The receive FIFO overflowed.
    Overrun,
    /// The transmit FIFO ran empty while the master clocked a frame.
    Underrun,
}

impl core::fmt::Display for Error {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        core::fmt::Debug::fmt(self, f)
    }
}

impl core::error::Error for Error {}

/// SPI slave configuration.
#[non_exhaustive]
#[derive(Clone, Debug)]
pub struct Config {
    /// Clock phase.
    pub phase: Phase,
    /// Clock polarity.
    pub polarity: Polarity,
    /// Byte sent after all bytes in the transmit buffer have been sent.
    pub orc: u8,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            phase: Phase::CaptureOnFirstTransition,
            polarity: Polarity::IdleLow,
            orc: 0,
        }
    }
}

pub(crate) struct Info {
    pub(crate) regs: Regs,
    pub(crate) fifo_depth: u8,
}

pub(crate) struct State {
    transfer: Mutex<RefCell<Transfer>>,
}

impl State {
    pub(crate) const fn new() -> Self {
        Self {
            transfer: Mutex::new(RefCell::new(Transfer::new())),
        }
    }
}

struct Transfer {
    active: bool,
    tx: usize,
    tx_len: usize,
    rx: usize,
    rx_len: usize,
    queued: usize,
    received: usize,
    orc: u8,
    error: Option<Error>,
}

impl Transfer {
    const fn new() -> Self {
        Self {
            active: false,
            tx: 0,
            tx_len: 0,
            rx: 0,
            rx_len: 0,
            queued: 0,
            received: 0,
            orc: 0,
            error: None,
        }
    }
}

/// Serial Peripheral Interface in slave mode.
///
/// The KL82 DSPI peripheral supports 8-bit, MSB-first slave transfers using PCS0 as its
/// active-low slave-select input. There is no embedded-hal SPI slave trait, so transfers use this
/// type's inherent methods.
pub struct Spis<'d, M: Mode> {
    info: &'static Info,
    state: &'static State,
    select: Flex<'d>,
    orc: u8,
    _phantom: PhantomData<M>,
}

impl<'d> Spis<'d, Blocking> {
    /// Create a blocking SPI slave.
    pub fn new_blocking<T: Instance>(
        _peri: Peri<'d, T>,
        sck: Peri<'d, impl SckPin<T>>,
        sout: Peri<'d, impl SoutPin<T>>,
        sin: Peri<'d, impl SinPin<T>>,
        pcs: Peri<'d, impl PcsPin<T>>,
        config: Config,
    ) -> Self {
        init::<T>(
            (sck.pcr(), sck.alt()),
            (sout.pcr(), sout.alt()),
            (sin.pcr(), sin.alt()),
            (pcs.pcr(), pcs.alt()),
            &config,
        );
        Self::new_inner::<T>(pcs, config.orc)
    }

    /// Receive and transmit during one PCS0 assertion.
    ///
    /// Returns the number of bytes stored in `read` and consumed from `write`.
    pub fn blocking_transfer(&mut self, read: &mut [u8], write: &[u8]) -> Result<(usize, usize), Error> {
        self.blocking_inner(
            read.as_mut_ptr() as usize,
            read.len(),
            write.as_ptr() as usize,
            write.len(),
        )
    }

    /// Receive during one PCS0 assertion while transmitting the over-read character.
    pub fn blocking_read(&mut self, data: &mut [u8]) -> Result<usize, Error> {
        self.blocking_transfer(data, &[]).map(|n| n.0)
    }

    /// Transmit during one PCS0 assertion and discard received bytes.
    pub fn blocking_write(&mut self, data: &[u8]) -> Result<usize, Error> {
        self.blocking_transfer(&mut [], data).map(|n| n.1)
    }

    /// Receive and transmit in place during one PCS0 assertion.
    pub fn blocking_transfer_in_place(&mut self, data: &mut [u8]) -> Result<usize, Error> {
        self.blocking_inner(
            data.as_mut_ptr() as usize,
            data.len(),
            data.as_ptr() as usize,
            data.len(),
        )
        .map(|n| n.0)
    }

    fn blocking_inner(&mut self, rx: usize, rx_len: usize, tx: usize, tx_len: usize) -> Result<(usize, usize), Error> {
        prepare(self.info, self.state, rx, rx_len, tx, tx_len, self.orc);
        self.select.arm_rising_edge_flag();
        run(self.info);

        let on_drop = OnDrop::new(|| abort(self.info, self.state));
        while !self.select.rising_edge_flag() {
            service(self.info, self.state);
        }
        self.select.disarm_edge();
        let result = finish(self.info, self.state);
        on_drop.defuse();
        result
    }
}

impl<'d> Spis<'d, Async> {
    /// Create an async SPI slave.
    pub fn new<T: InterruptInstance>(
        _peri: Peri<'d, T>,
        sck: Peri<'d, impl SckPin<T>>,
        sout: Peri<'d, impl SoutPin<T>>,
        sin: Peri<'d, impl SinPin<T>>,
        pcs: Peri<'d, impl PcsPin<T>>,
        _irq: impl Binding<T::Interrupt, InterruptHandler<T>>,
        config: Config,
    ) -> Self {
        init::<T>(
            (sck.pcr(), sck.alt()),
            (sout.pcr(), sout.alt()),
            (sin.pcr(), sin.alt()),
            (pcs.pcr(), pcs.alt()),
            &config,
        );
        enable_interrupt::<T>();
        Self::new_inner::<T>(pcs, config.orc)
    }

    /// Receive and transmit during one PCS0 assertion.
    ///
    /// The buffers limit how many bytes are stored or supplied; they do not set the transaction
    /// length. Extra received bytes are discarded and extra clocks transmit [`Config::orc`]. The
    /// transfer completes when PCS0 is deasserted and returns `(received, transmitted)` counts.
    pub async fn transfer(&mut self, read: &mut [u8], write: &[u8]) -> Result<(usize, usize), Error> {
        self.async_inner(
            read.as_mut_ptr() as usize,
            read.len(),
            write.as_ptr() as usize,
            write.len(),
        )
        .await
    }

    /// Receive during one PCS0 assertion while transmitting the over-read character.
    pub async fn read(&mut self, data: &mut [u8]) -> Result<usize, Error> {
        self.transfer(data, &[]).await.map(|n| n.0)
    }

    /// Transmit during one PCS0 assertion and discard received bytes.
    pub async fn write(&mut self, data: &[u8]) -> Result<usize, Error> {
        self.transfer(&mut [], data).await.map(|n| n.1)
    }

    /// Receive and transmit in place during one PCS0 assertion.
    pub async fn transfer_in_place(&mut self, data: &mut [u8]) -> Result<usize, Error> {
        self.async_inner(
            data.as_mut_ptr() as usize,
            data.len(),
            data.as_ptr() as usize,
            data.len(),
        )
        .await
        .map(|n| n.0)
    }

    async fn async_inner(
        &mut self,
        rx: usize,
        rx_len: usize,
        tx: usize,
        tx_len: usize,
    ) -> Result<(usize, usize), Error> {
        let info = self.info;
        let state = self.state;
        prepare(info, state, rx, rx_len, tx, tx_len, self.orc);

        // Arm the edge detector before starting DSPI. DSPI itself is already configured for
        // hardware PCS0 gating, so no first clock is lost while software observes slave select.
        let select_rise = self.select.rising_edge_future();
        run(info);

        let on_drop = OnDrop::new(move || abort(info, state));
        select_rise.await;
        let result = finish(info, state);
        on_drop.defuse();
        result
    }
}

impl<'d, M: Mode> Spis<'d, M> {
    fn new_inner<T: Instance>(pcs: Peri<'d, impl PcsPin<T>>, orc: u8) -> Self {
        Self {
            info: T::info(),
            state: T::state(),
            select: Flex::new(pcs),
            orc,
            _phantom: PhantomData,
        }
    }
}

fn init<T: Instance>(
    sck: (Reg<Pcr, RW>, Mux),
    sout: (Reg<Pcr, RW>, Mux),
    sin: (Reg<Pcr, RW>, Mux),
    pcs: (Reg<Pcr, RW>, Mux),
    config: &Config,
) {
    let regs = T::info().regs;
    T::enable_clock();

    // PCSIS[0] = 1 configures PCS0 as active low. Keep the module stopped until a transfer is
    // armed so configuration and interrupt-request writes happen only in the stopped state.
    regs.mcr().write(|w| {
        w.set_mstr(false);
        w.set_mdis(false);
        w.set_halt(true);
        w.set_pcsis(Pcsis::from_bits(1));
        w.set_clr_txf(true);
        w.set_clr_rxf(true);
    });
    regs.ctar_slave().write(|w| {
        w.set_fmsz(7);
        w.set_cpol(config.polarity == Polarity::IdleHigh);
        w.set_cpha(config.phase == Phase::CaptureOnSecondTransition);
    });

    for (pcr, mux) in [sck, sout, sin, pcs] {
        pcr.modify(|w| w.set_mux(mux));
    }

    regs.rser().write(|_| {});
    regs.sr().write_value(Sr(SR_W1C));
}

fn prepare(info: &'static Info, state: &'static State, rx: usize, rx_len: usize, tx: usize, tx_len: usize, orc: u8) {
    halt(info);
    let regs = info.regs;
    regs.mcr().modify(|w| {
        w.set_clr_txf(true);
        w.set_clr_rxf(true);
    });
    regs.sr().write_value(Sr(SR_W1C));

    critical_section::with(|cs| {
        let mut transfer = state.transfer.borrow(cs).borrow_mut();
        debug_assert!(!transfer.active);
        *transfer = Transfer {
            active: true,
            tx,
            tx_len,
            rx,
            rx_len,
            queued: 0,
            received: 0,
            orc,
            error: None,
        };
        fill_tx(info, &mut transfer);
    });
}

fn run(info: &'static Info) {
    let regs = info.regs;
    regs.rser().write(|w| {
        w.set_tfff_re(true);
        w.set_tfuf_re(true);
        w.set_rfof_re(true);
        w.set_rfdf_re(true);
    });
    regs.mcr().modify(|w| w.set_halt(false));
}

fn halt(info: &'static Info) {
    let regs = info.regs;
    regs.mcr().modify(|w| w.set_halt(true));
    while regs.sr().read().txrxs() {}
    regs.rser().write(|_| {});
}

fn fill_tx(info: &'static Info, transfer: &mut Transfer) {
    let regs = info.regs;
    while regs.sr().read().txctr() < info.fifo_depth {
        let byte = if transfer.queued < transfer.tx_len {
            // SAFETY: The transfer method holds the transmit slice borrowed until completion or
            // abort, and queued is checked against its length.
            unsafe { *((transfer.tx + transfer.queued) as *const u8) }
        } else {
            transfer.orc
        };
        regs.pushr_slave().write(|w| w.set_txdata(byte as u16));
        regs.sr().write_value(Sr(0).with_tfff());
        transfer.queued = transfer.queued.saturating_add(1);
    }
}

fn service(info: &'static Info, state: &'static State) {
    let regs = info.regs;

    critical_section::with(|cs| {
        let mut transfer = state.transfer.borrow(cs).borrow_mut();
        if !transfer.active {
            return;
        }

        let sr = regs.sr().read();
        if sr.rfof() {
            regs.sr().write_value(Sr(0).with_rfof());
            transfer.error.get_or_insert(Error::Overrun);
        }
        if sr.tfuf() {
            regs.sr().write_value(Sr(0).with_tfuf());
            transfer.error.get_or_insert(Error::Underrun);
        }

        while regs.sr().read().rxctr() != 0 {
            let byte = regs.popr().read().rxdata() as u8;
            regs.sr().write_value(Sr(0).with_rfdf());
            if transfer.received < transfer.rx_len {
                // SAFETY: The transfer method holds the receive slice borrowed until completion
                // or abort, and received is checked against its length.
                unsafe { *((transfer.rx + transfer.received) as *mut u8) = byte };
            }
            transfer.received = transfer.received.saturating_add(1);
        }

        fill_tx(info, &mut transfer);
    });
}

fn finish(info: &'static Info, state: &'static State) -> Result<(usize, usize), Error> {
    halt(info);
    service(info, state);

    let result = critical_section::with(|cs| {
        let mut transfer = state.transfer.borrow(cs).borrow_mut();
        let counts = (
            transfer.received.min(transfer.rx_len),
            transfer.received.min(transfer.tx_len),
        );
        let result = transfer.error.map_or(Ok(counts), Err);
        *transfer = Transfer::new();
        result
    });

    let regs = info.regs;
    regs.mcr().modify(|w| {
        w.set_clr_txf(true);
        w.set_clr_rxf(true);
    });
    regs.sr().write_value(Sr(SR_W1C));
    result
}

fn abort(info: &'static Info, state: &'static State) {
    halt(info);
    let regs = info.regs;
    regs.mcr().modify(|w| {
        w.set_clr_txf(true);
        w.set_clr_rxf(true);
    });
    regs.sr().write_value(Sr(SR_W1C));
    critical_section::with(|cs| *state.transfer.borrow(cs).borrow_mut() = Transfer::new());
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
        // An INTMUX channel may run for another source before this instance's clock is enabled.
        if T::INTMUX_SOURCE.is_some() && !T::clock_enabled() {
            return;
        }
        service(T::info(), T::state());
    }
}

trait SrWith {
    fn with_rfdf(self) -> Self;
    fn with_rfof(self) -> Self;
    fn with_tfff(self) -> Self;
    fn with_tfuf(self) -> Self;
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

    fn with_tfff(mut self) -> Self {
        self.set_tfff(true);
        self
    }

    fn with_tfuf(mut self) -> Self {
        self.set_tfuf(true);
        self
    }
}

pub(crate) trait SealedInstance {
    fn info() -> &'static Info;
    fn state() -> &'static State;
    fn enable_clock();
    fn clock_enabled() -> bool;
}

/// A DSPI instance usable in slave mode.
#[allow(private_bounds)]
pub trait Instance: SealedInstance + PeripheralType {}

/// A DSPI slave instance that can raise an interrupt.
pub trait InterruptInstance: Instance {
    /// NVIC interrupt for this instance: its own line, or the INTMUX channel it is routed through.
    type Interrupt: Interrupt;
    /// Input number on INTMUX0 for instances without an NVIC line of their own.
    const INTMUX_SOURCE: Option<u8> = None;
}

macro_rules! impl_spis_instance {
    ($inst:ident, $fifo_depth:expr) => {
        impl crate::spis::SealedInstance for crate::peripherals::$inst {
            fn info() -> &'static crate::spis::Info {
                static INFO: crate::spis::Info = crate::spis::Info {
                    regs: crate::pac::$inst,
                    fifo_depth: $fifo_depth,
                };
                &INFO
            }

            fn state() -> &'static crate::spis::State {
                static STATE: crate::spis::State = crate::spis::State::new();
                &STATE
            }

            fn enable_clock() {
                crate::clocks::enable::<crate::peripherals::$inst>();
            }

            fn clock_enabled() -> bool {
                crate::clocks::is_enabled::<crate::peripherals::$inst>()
            }
        }

        impl crate::spis::Instance for crate::peripherals::$inst {}
    };
}

macro_rules! impl_spis_interrupt {
    ($inst:ident, $irq:ident) => {
        impl crate::spis::InterruptInstance for crate::peripherals::$inst {
            type Interrupt = crate::interrupt::typelevel::$irq;
        }
    };
    ($inst:ident, $irq:ident, $source:expr) => {
        impl crate::spis::InterruptInstance for crate::peripherals::$inst {
            type Interrupt = crate::interrupt::typelevel::$irq;
            const INTMUX_SOURCE: Option<u8> = Some($source);
        }
    };
}

pub(crate) trait SealedSckPin<T: Instance>: crate::gpio::Pin {
    fn alt(&self) -> Mux;
}
pub(crate) trait SealedSoutPin<T: Instance>: crate::gpio::Pin {
    fn alt(&self) -> Mux;
}
pub(crate) trait SealedSinPin<T: Instance>: crate::gpio::Pin {
    fn alt(&self) -> Mux;
}
pub(crate) trait SealedPcsPin<T: Instance>: crate::gpio::Pin {
    fn alt(&self) -> Mux;
}

/// A pin that can carry `SPIn_SCK`.
#[allow(private_bounds)]
pub trait SckPin<T: Instance>: SealedSckPin<T> + crate::gpio::Pin {}
/// A pin that can carry `SPIn_SOUT`.
#[allow(private_bounds)]
pub trait SoutPin<T: Instance>: SealedSoutPin<T> + crate::gpio::Pin {}
/// A pin that can carry `SPIn_SIN`.
#[allow(private_bounds)]
pub trait SinPin<T: Instance>: SealedSinPin<T> + crate::gpio::Pin {}
/// A pin that can carry `SPIn_PCS0` as a slave-select input.
#[allow(private_bounds)]
pub trait PcsPin<T: Instance>: SealedPcsPin<T> + crate::gpio::Pin {}

macro_rules! impl_spis_pin {
    ($sealed:ident, $trait:ident, $pin:ident, $inst:ident, $alt:expr) => {
        impl crate::spis::$sealed<crate::peripherals::$inst> for crate::peripherals::$pin {
            fn alt(&self) -> crate::pac::port::vals::Mux {
                crate::pac::port::vals::Mux::from_bits($alt)
            }
        }
        impl crate::spis::$trait<crate::peripherals::$inst> for crate::peripherals::$pin {}
    };
}

macro_rules! impl_spis_sck_pin {
    ($pin:ident, $inst:ident, $alt:expr) => {
        impl_spis_pin!(SealedSckPin, SckPin, $pin, $inst, $alt);
    };
}

macro_rules! impl_spis_sout_pin {
    ($pin:ident, $inst:ident, $alt:expr) => {
        impl_spis_pin!(SealedSoutPin, SoutPin, $pin, $inst, $alt);
    };
}

macro_rules! impl_spis_sin_pin {
    ($pin:ident, $inst:ident, $alt:expr) => {
        impl_spis_pin!(SealedSinPin, SinPin, $pin, $inst, $alt);
    };
}

macro_rules! impl_spis_pcs_pin {
    ($pin:ident, $inst:ident, $alt:expr) => {
        impl_spis_pin!(SealedPcsPin, PcsPin, $pin, $inst, $alt);
    };
}
