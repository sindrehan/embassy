//! Time driver using the Kinetis low-power timers.
//!
//! LPTMR1 is a free-running 16-bit counter clocked at 1 kHz from the LPO. LPTMR0 provides the
//! alarm interrupt. Keeping the clock and alarm in separate modules matters because an enabled
//! LPTMR compare register may only be changed after a compare; restarting LPTMR0 to bring an
//! alarm forward therefore does not disturb the monotonic counter.
//!
//! Software extends LPTMR1 at each half-period. Even when no application timer is queued,
//! LPTMR0 raises a checkpoint interrupt every 32.768 seconds. Both modules and the LPO continue
//! operating in Stop and VLPS.

use core::cell::RefCell;
use core::cmp::min;
use core::sync::atomic::{AtomicU32, Ordering, compiler_fence};
use core::task::Waker;

use critical_section::Mutex;
use embassy_hal_internal::interrupt::InterruptExt;
use embassy_time_driver::Driver as _;
use embassy_time_queue_utils::Queue;

use crate::pac::lptmr::vals::Pcs;
use crate::pac::{LPTMR0, LPTMR1, interrupt};
use crate::peripherals;

const HALF_PERIOD: u64 = 1 << 15;

struct Driver {
    /// Number of 2^15-tick periods elapsed since boot.
    period: AtomicU32,
    queue: Mutex<RefCell<Queue>>,
}

fn read_counter() -> u16 {
    // A write latches the asynchronous counter into the value returned by the following read.
    LPTMR1.cnr().write(|_| {});
    LPTMR1.cnr().read().counter()
}

fn calc_now(period: u32, counter: u16) -> u64 {
    ((period as u64) << 15) + ((counter as u32 ^ ((period & 1) << 15)) as u64)
}

impl embassy_time_driver::Driver for Driver {
    fn now(&self) -> u64 {
        let period = self.period.load(Ordering::Relaxed);
        compiler_fence(Ordering::Acquire);
        calc_now(period, read_counter())
    }

    fn schedule_wake(&self, at: u64, waker: &Waker) {
        critical_section::with(|cs| {
            let mut queue = self.queue.borrow(cs).borrow_mut();

            if queue.schedule_wake(at, waker) {
                let mut next = queue.next_expiration(self.now());
                while !self.set_alarm(next) {
                    next = queue.next_expiration(self.now());
                }
            }
        })
    }
}

impl Driver {
    fn init(&'static self) {
        crate::clocks::enable::<peripherals::LPTMR0>();
        crate::clocks::enable::<peripherals::LPTMR1>();

        // LPTMR registers survive warm resets. Configure CSR with each timer disabled before
        // touching PSR or CMR.
        LPTMR0.csr().write(|_| {});
        LPTMR1.csr().write(|w| w.set_tfc(true));

        for timer in [LPTMR0, LPTMR1] {
            timer.psr().write(|w| {
                // Clock 1 is the 1 kHz LPO. It remains available through VLPS and LLS.
                w.set_pcs(Pcs::_01);
                w.set_pbyp(true);
            });
        }

        // LPTMR1 is the monotonic clock. In free-running mode it resets only at 16-bit overflow.
        LPTMR1.cmr().write(|w| w.set_compare(u16::MAX));
        // Keep CSR[5:1] unchanged when TEN changes from zero to one.
        LPTMR1.csr().write(|w| {
            w.set_tfc(true);
            w.set_ten(true);
        });

        unsafe { interrupt::LPTMR0.enable() };
        self.arm_for(HALF_PERIOD);
    }

    /// Update the software extension after LPTMR1 crosses a half-period boundary.
    fn update_period(&self) {
        let counter_half = (read_counter() >> 15) as u32;
        let period = self.period.load(Ordering::Relaxed);
        if counter_half != period & 1 {
            self.period.store(period.wrapping_add(1), Ordering::Relaxed);
        }
    }

    /// Program LPTMR0 for `timestamp` or the next extension checkpoint, whichever comes first.
    fn arm_for(&self, timestamp: u64) {
        let now = self.now();
        let checkpoint = ((now >> 15) + 1) << 15;
        let wake_at = min(timestamp, checkpoint);
        let ticks = wake_at.saturating_sub(now).clamp(1, u16::MAX as u64 + 1) as u32;

        // Disabling LPTMR0 resets its counter and makes CMR writable. CMR holds ticks - 1.
        LPTMR0.csr().write(|_| {});
        LPTMR0.cmr().write(|w| w.set_compare((ticks - 1) as u16));
        LPTMR0.csr().write(|w| w.set_ten(true));
        // The reference manual requires TIE to be set as the final initialization step.
        LPTMR0.csr().modify(|w| w.set_tie(true));
    }

    fn set_alarm(&self, timestamp: u64) -> bool {
        if timestamp <= self.now() {
            return false;
        }

        self.arm_for(timestamp);

        // Do not leave a compare armed for an application deadline that elapsed while programming
        // it. The caller wakes expired queue entries and tries again.
        if timestamp <= self.now() {
            LPTMR0.csr().write(|_| {});
            return false;
        }

        true
    }

    fn on_interrupt(&self) {
        critical_section::with(|cs| {
            if !LPTMR0.csr().read().tcf() {
                return;
            }

            // Disabling clears TCF and deasserts the interrupt request.
            LPTMR0.csr().write(|_| {});
            self.update_period();

            let mut next = self.queue.borrow_ref_mut(cs).next_expiration(self.now());
            while !self.set_alarm(next) {
                next = self.queue.borrow_ref_mut(cs).next_expiration(self.now());
            }
        });
    }
}

embassy_time_driver::time_driver_impl!(static DRIVER: Driver = Driver {
    period: AtomicU32::new(0),
    queue: Mutex::new(RefCell::new(Queue::new())),
});

pub(crate) fn init() {
    DRIVER.init();
}

#[cfg(feature = "rt")]
#[interrupt]
fn LPTMR0() {
    DRIVER.on_interrupt();
}
