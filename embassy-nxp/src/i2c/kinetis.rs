//! I2C master driver for the Kinetis I2C module.
//!
//! The module is clocked from the bus clock. The blocking and async APIs share one transfer
//! state machine that waits for the per-byte `IICIF` flag: the async driver sleeps on the
//! interrupt, the blocking one busy-polls the same future with [`embassy_futures::block_on`].
//!
//! With a DMA channel ([`I2c::new_with_dma`]) the bulk of each read or write run moves by DMA
//! while the address, the first written byte and the last two read bytes, which steer ACK and
//! STOP, stay with the state machine. I2C1 reaches the NVIC through [INTMUX0](crate::intmux),
//! so its handler is bound to `INTMUX0_0`. There are no timeouts: a slave holding the bus
//! stalls the transfer.
#![macro_use]

use core::future::poll_fn;
use core::marker::PhantomData;
use core::task::Poll;

use embassy_hal_internal::{Peri, PeripheralType};
use embassy_sync::waitqueue::AtomicWaker;
use embedded_hal_1::i2c::Operation;

use crate::dma::{AnyChannel, Channel};

use crate::interrupt::typelevel::{Binding, Interrupt};
use crate::pac::common::{RW, Reg};
use crate::pac::i2c::I2c as Regs;
use crate::pac::i2c::regs::S;
use crate::pac::i2c::vals::{Flt, Mult};
use crate::pac::port::regs::Pcr;
use crate::pac::port::vals::Mux;
use crate::{Async, Blocking, Mode};

/// SCL divider for each `F[ICR]` value (reference manual, "I2C divider and hold values").
const DIVIDERS: [u16; 64] = [
    20, 22, 24, 26, 28, 30, 34, 40, 28, 32, 36, 40, 44, 48, 56, 68, 48, 56, 64, 72, 80, 88, 104, 128, 80, 96, 112, 128,
    144, 160, 192, 240, 160, 192, 224, 256, 288, 320, 384, 480, 320, 384, 448, 512, 576, 640, 768, 960, 640, 768, 896,
    1024, 1152, 1280, 1536, 1920, 1280, 1536, 1792, 2048, 2304, 2560, 3072, 3840,
];

/// I2C error.
#[derive(Debug, Eq, PartialEq, Copy, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum Error {
    /// The bus was busy when the transfer should have started.
    Bus,
    /// Arbitration was lost to another master.
    Arbitration,
    /// The address byte was not acknowledged.
    AddressNack,
    /// A data byte was not acknowledged.
    DataNack,
    /// A read of zero bytes was requested, which the hardware cannot do.
    InvalidReadBufferLength,
    /// The DMA controller reported an error moving the data.
    Dma,
}

impl embedded_hal_1::i2c::Error for Error {
    fn kind(&self) -> embedded_hal_1::i2c::ErrorKind {
        use embedded_hal_1::i2c::{ErrorKind, NoAcknowledgeSource};
        match self {
            Error::Bus => ErrorKind::Bus,
            Error::Arbitration => ErrorKind::ArbitrationLoss,
            Error::AddressNack => ErrorKind::NoAcknowledge(NoAcknowledgeSource::Address),
            Error::DataNack => ErrorKind::NoAcknowledge(NoAcknowledgeSource::Data),
            Error::InvalidReadBufferLength => ErrorKind::Other,
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

/// I2C configuration.
#[non_exhaustive]
#[derive(Clone, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Config {
    /// SCL frequency in Hz. The closest divider of the bus clock is used.
    pub frequency: u32,
    /// Enable the internal pull-ups on SCL and SDA.
    pub internal_pullup: bool,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            frequency: 100_000,
            internal_pullup: true,
        }
    }
}

/// Per-instance constants.
pub struct Info {
    pub(crate) regs: Regs,
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

/// I2C master driver.
pub struct I2c<'d, M: Mode> {
    info: &'static Info,
    state: &'static State,
    is_async: bool,
    dma: Option<(Peri<'d, AnyChannel>, u8)>,
    _phantom: PhantomData<(&'d (), M)>,
}

/// `F[MULT]` and `F[ICR]` giving the SCL frequency closest to `frequency` from `bus_hz`.
fn baud_divisors(bus_hz: u32, frequency: u32) -> (Mult, u8) {
    let mut best = (u32::MAX, 0u8, 0u8);
    for mult in 0..=2u8 {
        for (icr, &divider) in DIVIDERS.iter().enumerate() {
            let rate = bus_hz / ((1u32 << mult) * divider as u32);
            let error = rate.abs_diff(frequency);
            if error < best.0 {
                best = (error, mult, icr as u8);
            }
        }
    }
    (Mult::from_bits(best.1), best.2)
}

fn init<T: Instance>(scl: (Reg<Pcr, RW>, Mux), sda: (Reg<Pcr, RW>, Mux), config: &Config) {
    let regs = T::info().regs;
    T::enable_clock();

    // The reset state, with the start/stop detect flags cleared, then the divider.
    regs.a1().write(|_| {});
    regs.f().write(|_| {});
    regs.c1().write(|_| {});
    regs.s().write_value(S(0xFF));
    regs.c2().write(|_| {});
    regs.flt().write(|w| {
        w.set_startf(true);
        w.set_stopf(true);
        w.set_flt(Flt::from_bits(0));
    });
    regs.ra().write(|_| {});

    let (mult, icr) = baud_divisors(crate::clocks::clocks().bus, config.frequency);
    regs.f().write(|w| {
        w.set_mult(mult);
        w.set_icr(icr);
    });

    for (pcr, mux) in [scl, sda] {
        pcr.modify(|w| {
            w.set_mux(mux);
            w.set_ode(true);
            w.set_pe(config.internal_pullup);
            w.set_ps(true);
        });
    }

    regs.c1().write(|w| w.set_iicen(true));
}

impl<'d> I2c<'d, Blocking> {
    /// Create a blocking I2C master.
    pub fn new_blocking<T: Instance>(
        _peri: Peri<'d, T>,
        scl: Peri<'d, impl SclPin<T>>,
        sda: Peri<'d, impl SdaPin<T>>,
        config: Config,
    ) -> Self {
        init::<T>((scl.pcr(), scl.alt()), (sda.pcr(), sda.alt()), &config);
        Self {
            info: T::info(),
            state: T::state(),
            is_async: false,
            dma: None,
            _phantom: PhantomData,
        }
    }
}

impl<'d> I2c<'d, Async> {
    /// Create an async I2C master.
    pub fn new<T: InterruptInstance>(
        _peri: Peri<'d, T>,
        scl: Peri<'d, impl SclPin<T>>,
        sda: Peri<'d, impl SdaPin<T>>,
        _irq: impl Binding<T::Interrupt, InterruptHandler<T>>,
        config: Config,
    ) -> Self {
        init::<T>((scl.pcr(), scl.alt()), (sda.pcr(), sda.alt()), &config);
        if let Some(source) = T::INTMUX_SOURCE {
            crate::intmux::enable_source(crate::intmux::CHANNEL, source);
        }
        T::Interrupt::unpend();
        unsafe { T::Interrupt::enable() };
        Self {
            info: T::info(),
            state: T::state(),
            is_async: true,
            dma: None,
            _phantom: PhantomData,
        }
    }

    /// Create an async I2C master that moves the bulk of each transfer with a DMA channel.
    pub fn new_with_dma<T: InterruptInstance>(
        _peri: Peri<'d, T>,
        scl: Peri<'d, impl SclPin<T>>,
        sda: Peri<'d, impl SdaPin<T>>,
        _irq: impl Binding<T::Interrupt, InterruptHandler<T>>,
        dma: Peri<'d, impl Channel>,
        config: Config,
    ) -> Self {
        let mut i2c = Self::new(_peri, scl, sda, _irq, config);
        i2c.dma = Some((dma.into(), T::DMA_REQUEST));
        i2c
    }

    /// Read into `buffer` from the device at `address`.
    pub async fn read(&mut self, address: u8, buffer: &mut [u8]) -> Result<(), Error> {
        self.transaction_inner(address, &mut [Operation::Read(buffer)]).await
    }

    /// Write `bytes` to the device at `address`.
    pub async fn write(&mut self, address: u8, bytes: &[u8]) -> Result<(), Error> {
        self.transaction_inner(address, &mut [Operation::Write(bytes)]).await
    }

    /// Write `bytes`, then read into `buffer` after a repeated start.
    pub async fn write_read(&mut self, address: u8, bytes: &[u8], buffer: &mut [u8]) -> Result<(), Error> {
        self.transaction_inner(address, &mut [Operation::Write(bytes), Operation::Read(buffer)])
            .await
    }

    /// Run a sequence of operations as one transaction, see [`embedded_hal_1::i2c::I2c::transaction`].
    pub async fn transaction(&mut self, address: u8, operations: &mut [Operation<'_>]) -> Result<(), Error> {
        self.transaction_inner(address, operations).await
    }
}

impl<'d, M: Mode> I2c<'d, M> {
    /// Read into `buffer` from the device at `address`.
    pub fn blocking_read(&mut self, address: u8, buffer: &mut [u8]) -> Result<(), Error> {
        embassy_futures::block_on(self.transaction_inner(address, &mut [Operation::Read(buffer)]))
    }

    /// Write `bytes` to the device at `address`.
    pub fn blocking_write(&mut self, address: u8, bytes: &[u8]) -> Result<(), Error> {
        embassy_futures::block_on(self.transaction_inner(address, &mut [Operation::Write(bytes)]))
    }

    /// Write `bytes`, then read into `buffer` after a repeated start.
    pub fn blocking_write_read(&mut self, address: u8, bytes: &[u8], buffer: &mut [u8]) -> Result<(), Error> {
        embassy_futures::block_on(
            self.transaction_inner(address, &mut [Operation::Write(bytes), Operation::Read(buffer)]),
        )
    }

    /// Run a sequence of operations as one transaction, see [`embedded_hal_1::i2c::I2c::transaction`].
    pub fn blocking_transaction(&mut self, address: u8, operations: &mut [Operation<'_>]) -> Result<(), Error> {
        embassy_futures::block_on(self.transaction_inner(address, operations))
    }

    /// Wait for `IICIF`, returning the status. Async mode sleeps on the interrupt, blocking mode
    /// is polled in a busy loop by `block_on`.
    async fn wait_iicif(&mut self) -> S {
        let regs = self.info.regs;
        poll_fn(|cx| {
            let s = regs.s().read();
            if s.iicif() {
                return Poll::Ready(s);
            }
            if self.is_async {
                self.state.waker.register(cx.waker());
                // IICIF is level sensitive: a flag set between the read above and this write still
                // raises the interrupt. The handler masks IICIE again before waking.
                critical_section::with(|_| regs.c1().modify(|w| w.set_iicie(true)));
            }
            Poll::Pending
        })
        .await
    }

    /// After a DMA run: the byte that the DMA's last register access started is in flight or
    /// done, and IICIF may be stale from the DMA'd bytes. Return the status once it has finished.
    async fn wait_after_dma(&mut self) -> S {
        let regs = self.info.regs;
        let s = regs.s().read();
        if s.tcf() {
            return s;
        }
        regs.s().write_value(S(0).with_iicif());
        let s = regs.s().read();
        if s.tcf() {
            return s;
        }
        self.wait_iicif().await
    }

    /// Wait for the byte in flight, clear the flag and report arbitration loss or a NACK.
    async fn finish_byte(&mut self, nack: Error) -> Result<(), Error> {
        self.finish_byte_with(false, nack).await
    }

    async fn finish_byte_with(&mut self, after_dma: bool, nack: Error) -> Result<(), Error> {
        let regs = self.info.regs;
        let s = if after_dma {
            self.wait_after_dma().await
        } else {
            self.wait_iicif().await
        };
        regs.s().write_value(S(0).with_iicif());
        if s.arbl() {
            regs.s().write_value(S(0).with_arbl());
            // Arbitration loss clears MST in hardware; nothing to stop.
            return Err(Error::Arbitration);
        }
        if s.rxak() {
            self.stop().await;
            return Err(nack);
        }
        Ok(())
    }

    /// START (or repeated START) followed by the address byte.
    async fn start(&mut self, address: u8, read: bool, repeated: bool) -> Result<(), Error> {
        let regs = self.info.regs;

        if repeated {
            // Errata: a repeated start is not generated reliably with F[MULT] != 0.
            let f = regs.f().read();
            regs.f().write(|w| {
                w.set_icr(f.icr());
                w.set_mult(Mult::_00);
            });
            regs.c1().modify(|w| {
                w.set_rsta(true);
                w.set_tx(true);
            });
            regs.f().write_value(f);
            cortex_m::asm::delay(6);
        } else {
            if regs.s().read().busy() {
                return Err(Error::Bus);
            }
            regs.c1().modify(|w| {
                w.set_mst(true);
                w.set_tx(true);
            });
        }

        while !regs.s2().read().empty() {}
        regs.d().write(|w| w.set_data((address << 1) | read as u8));
        self.finish_byte(Error::AddressNack).await
    }

    /// STOP, then wait for the bus to go idle.
    async fn stop(&mut self) {
        let regs = self.info.regs;
        regs.c1().modify(|w| {
            w.set_mst(false);
            w.set_tx(false);
            w.set_txak(false);
        });
        while regs.s().read().busy() {}
    }

    async fn transaction_inner(&mut self, address: u8, operations: &mut [Operation<'_>]) -> Result<(), Error> {
        let regs = self.info.regs;
        let count = operations.len();
        let mut index = 0;
        let mut started = false;
        let mut stopped = false;

        while index < count {
            match operations[index] {
                Operation::Write(_) => {
                    self.start(address, false, started).await?;
                    started = true;
                    regs.c1().modify(|w| w.set_tx(true));
                    // Consecutive writes are one stream of bytes.
                    while let Some(Operation::Write(bytes)) = operations.get(index) {
                        if bytes.len() >= 2 && self.dma.is_some() {
                            // First byte by hand with DMAEN set; each completion then requests
                            // the next byte from the DMA until the slice is done.
                            let (channel, request) = self.dma.as_mut().unwrap();
                            regs.c1().modify(|w| w.set_dmaen(true));
                            regs.d().write(|w| w.set_data(bytes[0]));
                            let transfer = unsafe {
                                crate::dma::write(channel.reborrow(), *request, &bytes[1..], regs.d().as_ptr() as *mut u8)
                            };
                            let result = transfer.await;
                            regs.c1().modify(|w| w.set_dmaen(false));
                            if result.is_err() {
                                self.stop().await;
                                return Err(Error::Dma);
                            }
                            self.finish_byte_with(true, Error::DataNack).await?;
                        } else {
                            for &byte in *bytes {
                                regs.d().write(|w| w.set_data(byte));
                                self.finish_byte(Error::DataNack).await?;
                            }
                        }
                        index += 1;
                    }
                }
                Operation::Read(_) => {
                    // Consecutive reads are one stream too; the last byte of the run is NACKed.
                    let run_end = index
                        + operations[index..]
                            .iter()
                            .take_while(|op| matches!(op, Operation::Read(_)))
                            .count();
                    let mut remaining: usize = operations[index..run_end]
                        .iter()
                        .map(|op| match op {
                            Operation::Read(buffer) => buffer.len(),
                            Operation::Write(_) => 0,
                        })
                        .sum();
                    if remaining == 0 {
                        if started {
                            self.stop().await;
                        }
                        return Err(Error::InvalidReadBufferLength);
                    }
                    let last_run = run_end == count;

                    self.start(address, true, started).await?;
                    started = true;

                    // Switch to receive; the dummy read clocks in the first byte.
                    regs.c1().modify(|w| {
                        w.set_tx(false);
                        w.set_txak(remaining == 1);
                    });
                    let _ = regs.d().read();

                    // A single read of three or more bytes moves all but the last two by DMA;
                    // those two set the NACK and the STOP.
                    let mut after_dma = false;
                    let mut skip = 0;
                    if run_end == index + 1 && remaining >= 3 && self.dma.is_some() {
                        let Operation::Read(buffer) = &mut operations[index] else { unreachable!() };
                        let (channel, request) = self.dma.as_mut().unwrap();
                        let count = remaining - 2;
                        regs.c1().modify(|w| w.set_dmaen(true));
                        let transfer = unsafe {
                            crate::dma::read(channel.reborrow(), *request, regs.d().as_ptr() as *const u8, &mut buffer[..count])
                        };
                        let result = transfer.await;
                        regs.c1().modify(|w| w.set_dmaen(false));
                        if result.is_err() {
                            self.stop().await;
                            return Err(Error::Dma);
                        }
                        skip = count;
                        remaining -= count;
                        after_dma = true;
                    }

                    for op in &mut operations[index..run_end] {
                        let Operation::Read(buffer) = op else { unreachable!() };
                        for slot in buffer.iter_mut().skip(skip) {
                            let s = if core::mem::take(&mut after_dma) {
                                self.wait_after_dma().await
                            } else {
                                self.wait_iicif().await
                            };
                            regs.s().write_value(S(0).with_iicif());
                            if s.arbl() {
                                regs.s().write_value(S(0).with_arbl());
                                return Err(Error::Arbitration);
                            }
                            match remaining {
                                // Last byte: STOP (or hold the bus for a repeated start) before
                                // reading it out, so no further byte is clocked.
                                1 if last_run => {
                                    self.stop().await;
                                    stopped = true;
                                }
                                1 => regs.c1().modify(|w| w.set_tx(true)),
                                // Second to last: NACK the next one.
                                2 => regs.c1().modify(|w| w.set_txak(true)),
                                _ => {}
                            }
                            *slot = regs.d().read().data();
                            remaining -= 1;
                        }
                        skip = 0;
                    }
                    index = run_end;
                }
            }
        }

        if started && !stopped {
            self.stop().await;
        }
        Ok(())
    }
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
        if regs.c1().read().iicie() && regs.s().read().iicif() {
            // The thread side only touches C1 inside a critical section, so this cannot race.
            regs.c1().modify(|w| w.set_iicie(false));
            T::state().waker.wake();
        }
    }
}

impl<'d, M: Mode> embedded_hal_1::i2c::ErrorType for I2c<'d, M> {
    type Error = Error;
}

impl<'d, M: Mode> embedded_hal_1::i2c::I2c for I2c<'d, M> {
    fn transaction(&mut self, address: u8, operations: &mut [Operation<'_>]) -> Result<(), Self::Error> {
        self.blocking_transaction(address, operations)
    }
}

impl<'d> embedded_hal_async::i2c::I2c for I2c<'d, Async> {
    async fn transaction(&mut self, address: u8, operations: &mut [Operation<'_>]) -> Result<(), Self::Error> {
        self.transaction_inner(address, operations).await
    }
}

trait SWith {
    fn with_iicif(self) -> Self;
    fn with_arbl(self) -> Self;
}

impl SWith for S {
    fn with_iicif(mut self) -> Self {
        self.set_iicif(true);
        self
    }

    fn with_arbl(mut self) -> Self {
        self.set_arbl(true);
        self
    }
}

pub(crate) trait SealedInstance {
    /// DMAMUX request source (shared by both directions).
    const DMA_REQUEST: u8;
    fn info() -> &'static Info;
    fn state() -> &'static State;
    fn enable_clock();
    fn clock_enabled() -> bool;
}

/// An I2C instance.
#[allow(private_bounds)]
pub trait Instance: SealedInstance + PeripheralType {}

/// An I2C instance that can raise an interrupt, usable in async mode.
pub trait InterruptInstance: Instance {
    /// NVIC interrupt for this instance: its own line, or the INTMUX channel it is routed through.
    type Interrupt: Interrupt;
    /// Input number on INTMUX0 for instances without an NVIC line of their own.
    const INTMUX_SOURCE: Option<u8> = None;
}

macro_rules! impl_i2c_instance {
    ($inst:ident, $request:expr) => {
        impl crate::i2c::SealedInstance for crate::peripherals::$inst {
            const DMA_REQUEST: u8 = $request;

            fn info() -> &'static crate::i2c::Info {
                static INFO: crate::i2c::Info = crate::i2c::Info {
                    regs: crate::pac::$inst,
                };
                &INFO
            }

            fn state() -> &'static crate::i2c::State {
                static STATE: crate::i2c::State = crate::i2c::State::new();
                &STATE
            }

            fn enable_clock() {
                crate::clocks::enable::<crate::peripherals::$inst>();
            }

            fn clock_enabled() -> bool {
                crate::clocks::is_enabled::<crate::peripherals::$inst>()
            }
        }

        impl crate::i2c::Instance for crate::peripherals::$inst {}
    };
}

macro_rules! impl_i2c_interrupt {
    ($inst:ident, $irq:ident) => {
        impl crate::i2c::InterruptInstance for crate::peripherals::$inst {
            type Interrupt = crate::interrupt::typelevel::$irq;
        }
    };
    ($inst:ident, $irq:ident, $source:expr) => {
        impl crate::i2c::InterruptInstance for crate::peripherals::$inst {
            type Interrupt = crate::interrupt::typelevel::$irq;
            const INTMUX_SOURCE: Option<u8> = Some($source);
        }
    };
}

pub(crate) trait SealedSclPin<T: Instance>: crate::gpio::Pin {
    fn alt(&self) -> Mux;
}

pub(crate) trait SealedSdaPin<T: Instance>: crate::gpio::Pin {
    fn alt(&self) -> Mux;
}

/// A pin that can carry `I2Cn_SCL`.
#[allow(private_bounds)]
pub trait SclPin<T: Instance>: SealedSclPin<T> + crate::gpio::Pin {}

/// A pin that can carry `I2Cn_SDA`.
#[allow(private_bounds)]
pub trait SdaPin<T: Instance>: SealedSdaPin<T> + crate::gpio::Pin {}

macro_rules! impl_i2c_scl_pin {
    ($pin:ident, $inst:ident, $alt:expr) => {
        impl crate::i2c::SealedSclPin<crate::peripherals::$inst> for crate::peripherals::$pin {
            fn alt(&self) -> crate::pac::port::vals::Mux {
                crate::pac::port::vals::Mux::from_bits($alt)
            }
        }
        impl crate::i2c::SclPin<crate::peripherals::$inst> for crate::peripherals::$pin {}
    };
}

macro_rules! impl_i2c_sda_pin {
    ($pin:ident, $inst:ident, $alt:expr) => {
        impl crate::i2c::SealedSdaPin<crate::peripherals::$inst> for crate::peripherals::$pin {
            fn alt(&self) -> crate::pac::port::vals::Mux {
                crate::pac::port::vals::Mux::from_bits($alt)
            }
        }
        impl crate::i2c::SdaPin<crate::peripherals::$inst> for crate::peripherals::$pin {}
    };
}
