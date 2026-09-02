//! eDMA driver for Kinetis.
//!
//! Eight channels, each fed by DMAMUX0 from one of 64 peripheral request sources; any source can
//! go to any channel, so drivers take any channel and supply their request number. Transfers are
//! futures completing on the major loop interrupt. The HAL owns the DMA interrupts (channel pairs
//! share the four `DMAn_DMAn+4` lines, plus `DMA_ERROR`); nothing needs binding by the user.
#![macro_use]

use core::future::Future;
use core::pin::Pin;
use core::sync::atomic::{AtomicU8, Ordering, compiler_fence};
use core::task::{Context, Poll};

use embassy_hal_internal::interrupt::InterruptExt;
use embassy_hal_internal::{PeripheralType, impl_peripheral};
use embassy_sync::waitqueue::AtomicWaker;

use crate::Peri;
use crate::pac::dma::vals::Ssize;
#[cfg(feature = "rt")]
use crate::pac::interrupt;
use crate::pac::{DMA, DMAMUX, Interrupt, SIM};
use crate::peripherals;

pub(crate) const CHANNEL_COUNT: usize = 8;
/// Largest major loop count without channel linking (`CITER` is 15 bits).
const MAX_TRANSFER: usize = 0x7FFF;

static WAKERS: [AtomicWaker; CHANNEL_COUNT] = [const { AtomicWaker::new() }; CHANNEL_COUNT];
/// Channels whose transfer stopped with an error, one bit each. Only touched in critical sections.
static ERRORS: AtomicU8 = AtomicU8::new(0);

/// DMA error.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum Error {
    /// The controller reported a bus or configuration error on the channel.
    Bus,
}

#[cfg(feature = "rt")]
fn on_channel(channel: usize) {
    if DMA.int().read().int(channel) {
        DMA.cint().write(|w| w.set_cint(channel as u8));
        WAKERS[channel].wake();
    }
}

#[cfg(feature = "rt")]
#[interrupt]
fn DMA0_DMA4() {
    on_channel(0);
    on_channel(4);
}

#[cfg(feature = "rt")]
#[interrupt]
fn DMA1_DMA5() {
    on_channel(1);
    on_channel(5);
}

#[cfg(feature = "rt")]
#[interrupt]
fn DMA2_DMA6() {
    on_channel(2);
    on_channel(6);
}

#[cfg(feature = "rt")]
#[interrupt]
fn DMA3_DMA7() {
    on_channel(3);
    on_channel(7);
}

#[cfg(feature = "rt")]
#[interrupt]
fn DMA_ERROR() {
    let err = DMA.err().read().0 as u8;
    critical_section::with(|_| {
        ERRORS.store(ERRORS.load(Ordering::Relaxed) | err, Ordering::Relaxed);
    });
    for channel in 0..CHANNEL_COUNT {
        if err & (1 << channel) != 0 {
            DMA.cerq().write(|w| w.set_cerq(channel as u8));
            DMA.cerr().write(|w| w.set_cerr(channel as u8));
            WAKERS[channel].wake();
        }
    }
}

pub(crate) fn init() {
    critical_section::with(|_| SIM.scgc7().modify(|w| w.set_dma(true)));
    crate::clocks::enable::<peripherals::DMAMUX>();

    DMA.erq().write(|_| {});
    DMA.int().write(|w| w.0 = 0xFF);
    DMA.err().write(|w| w.0 = 0xFF);
    for channel in 0..CHANNEL_COUNT {
        DMAMUX.chcfg(channel).write(|_| {});
    }
    // Error interrupts for every channel.
    DMA.seei().write(|w| w.set_saee(true));

    unsafe {
        Interrupt::DMA0_DMA4.enable();
        Interrupt::DMA1_DMA5.enable();
        Interrupt::DMA2_DMA6.enable();
        Interrupt::DMA3_DMA7.enable();
        Interrupt::DMA_ERROR.enable();
    }
    info!("DMA initialized");
}

/// Peripheral to memory: `to.len()` words from the register at `from` into `to`, one word per
/// request from DMAMUX source `request`.
///
/// # Safety
///
/// `from` must be a peripheral data register and `to` valid, unaliased memory for the whole
/// transfer. The peripheral must be set up to raise the request.
pub unsafe fn read<'a, C: Channel, W: Word>(ch: Peri<'a, C>, request: u8, from: *const W, to: *mut [W]) -> Transfer<'a, C> {
    transfer_inner(ch, request, from as u32, to as *mut W as u32, to.len(), W::SIZE, false, true)
}

/// Memory to peripheral: `from.len()` words from `from` into the register at `to`, one word per
/// request from DMAMUX source `request`.
///
/// # Safety
///
/// `to` must be a peripheral data register and `from` valid memory that stays unchanged for the
/// whole transfer. The peripheral must be set up to raise the request.
pub unsafe fn write<'a, C: Channel, W: Word>(ch: Peri<'a, C>, request: u8, from: *const [W], to: *mut W) -> Transfer<'a, C> {
    transfer_inner(ch, request, from as *const W as u32, to as u32, from.len(), W::SIZE, true, false)
}

/// Peripheral to nowhere: `count` words from the register at `from`, discarded. Useful to drain
/// a receive register while only transmitting.
///
/// # Safety
///
/// `from` must be a peripheral data register set up to raise the request.
pub unsafe fn read_discard<'a, C: Channel, W: Word>(ch: Peri<'a, C>, request: u8, from: *const W, count: usize) -> Transfer<'a, C> {
    transfer_inner(ch, request, from as u32, &raw const SINK as u32, count, W::SIZE, false, false)
}

/// One fixed word to a peripheral, `count` times. Useful to clock a receive-only transfer.
///
/// # Safety
///
/// `to` must be a peripheral data register set up to raise the request; `from` must stay valid
/// and unchanged for the whole transfer.
pub unsafe fn write_repeated<'a, C: Channel, W: Word>(ch: Peri<'a, C>, request: u8, from: *const W, to: *mut W, count: usize) -> Transfer<'a, C> {
    transfer_inner(ch, request, from as u32, to as u32, count, W::SIZE, false, false)
}

/// Destination of [`read_discard`]. Written by DMA only, never read.
static mut SINK: u32 = 0;

#[allow(clippy::too_many_arguments)]
fn transfer_inner<'a, C: Channel>(
    ch: Peri<'a, C>,
    request: u8,
    from: u32,
    to: u32,
    len: usize,
    size: Ssize,
    incr_src: bool,
    incr_dst: bool,
) -> Transfer<'a, C> {
    assert!(len > 0 && len <= MAX_TRANSFER, "DMA: transfer length must be 1 to 32767 words");
    let n = ch.number() as usize;
    let bytes = 1u16 << size.to_bits();

    // Quiesce the channel before touching its TCD.
    DMA.cerq().write(|w| w.set_cerq(n as u8));
    DMAMUX.chcfg(n).write(|_| {});
    while DMA.tcd_csr(n).read().active() {}
    DMA.cdne().write(|w| w.set_cdne(n as u8));
    DMA.cint().write(|w| w.set_cint(n as u8));
    critical_section::with(|_| ERRORS.store(ERRORS.load(Ordering::Relaxed) & !(1 << n), Ordering::Relaxed));

    // One word per minor loop, one minor loop per request, `len` major iterations, and the
    // request disabled by hardware when the major loop finishes.
    DMA.tcd_saddr(n).write(|w| w.set_saddr(from));
    DMA.tcd_soff(n).write(|w| w.set_soff(if incr_src { bytes } else { 0 }));
    DMA.tcd_attr(n).write(|w| {
        w.set_ssize(size);
        w.set_dsize(size.to_bits());
    });
    DMA.tcd_nbytes_mlno(n).write(|w| w.set_nbytes(bytes as u32));
    DMA.tcd_slast(n).write(|w| w.set_slast(0));
    DMA.tcd_daddr(n).write(|w| w.set_daddr(to));
    DMA.tcd_doff(n).write(|w| w.set_doff(if incr_dst { bytes } else { 0 }));
    DMA.tcd_citer_elinkno(n).write(|w| w.set_citer(len as u16));
    DMA.tcd_biter_elinkno(n).write(|w| w.set_biter(len as u16));
    DMA.tcd_dlastsga(n).write(|w| w.set_dlastsga(0));
    DMA.tcd_csr(n).write(|w| {
        w.set_intmajor(true);
        w.set_dreq(true);
    });

    DMAMUX.chcfg(n).write(|w| {
        w.set_source(crate::pac::dmamux::vals::Source::from_bits(request));
        w.set_enbl(true);
    });

    compiler_fence(Ordering::SeqCst);
    DMA.serq().write(|w| w.set_serq(n as u8));
    compiler_fence(Ordering::SeqCst);

    Transfer { channel: ch }
}

/// A running DMA transfer. Dropping it stops the channel.
#[must_use = "futures do nothing unless you `.await` or poll them"]
pub struct Transfer<'a, C: Channel> {
    channel: Peri<'a, C>,
}

impl<'a, C: Channel> Drop for Transfer<'a, C> {
    fn drop(&mut self) {
        let n = self.channel.number() as usize;
        DMA.cerq().write(|w| w.set_cerq(n as u8));
        DMAMUX.chcfg(n).write(|_| {});
        while DMA.tcd_csr(n).read().active() {}
        compiler_fence(Ordering::SeqCst);
    }
}

impl<'a, C: Channel> Unpin for Transfer<'a, C> {}
impl<'a, C: Channel> Future for Transfer<'a, C> {
    type Output = Result<(), Error>;

    fn poll(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
        let n = self.channel.number() as usize;
        WAKERS[n].register(cx.waker());

        let failed = critical_section::with(|_| {
            let errors = ERRORS.load(Ordering::Relaxed);
            ERRORS.store(errors & !(1 << n), Ordering::Relaxed);
            errors & (1 << n) != 0
        });
        if failed {
            return Poll::Ready(Err(Error::Bus));
        }
        if DMA.tcd_csr(n).read().done() {
            compiler_fence(Ordering::SeqCst);
            Poll::Ready(Ok(()))
        } else {
            Poll::Pending
        }
    }
}

pub(crate) trait SealedChannel {}
trait SealedWord {}

/// A DMA channel.
#[allow(private_bounds)]
pub trait Channel: PeripheralType + SealedChannel + Into<AnyChannel> + Sized + 'static {
    /// Channel number, 0 to 7.
    fn number(&self) -> u8;
}

/// A DMA transfer word.
#[allow(private_bounds)]
pub trait Word: SealedWord {
    /// Transfer size code for the TCD.
    const SIZE: Ssize;
}

impl SealedWord for u8 {}
impl Word for u8 {
    const SIZE: Ssize = Ssize::from_bits(0);
}
impl SealedWord for u16 {}
impl Word for u16 {
    const SIZE: Ssize = Ssize::from_bits(1);
}
impl SealedWord for u32 {}
impl Word for u32 {
    const SIZE: Ssize = Ssize::from_bits(2);
}

/// Type-erased DMA channel.
pub struct AnyChannel {
    pub(crate) number: u8,
}

impl_peripheral!(AnyChannel);

impl SealedChannel for AnyChannel {}
impl Channel for AnyChannel {
    fn number(&self) -> u8 {
        self.number
    }
}

macro_rules! impl_dma_channel {
    ($instance:ident, $name:ident, $num:expr) => {
        impl crate::dma::SealedChannel for crate::peripherals::$name {}
        impl crate::dma::Channel for crate::peripherals::$name {
            fn number(&self) -> u8 {
                $num
            }
        }

        impl From<peripherals::$name> for crate::dma::AnyChannel {
            fn from(val: peripherals::$name) -> Self {
                use crate::dma::Channel;

                Self { number: val.number() }
            }
        }
    };
}
