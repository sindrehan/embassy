//! Time driver using a Timer/PWM Module (TPM).
//!
//! This driver is used with the Kinetis parts. TPM0 runs as a free-running 16-bit counter at
//! 1 MHz, clocked from the 4 MHz fast internal reference clock (MCGIRCLK) through the /4
//! prescaler, so the tick rate does not depend on the core or bus clock configuration.
//!
//! The 16-bit counter is extended to 64 bits in software the same way the `embassy-stm32` 16-bit
//! timer driver does it: a `period` counter is incremented on every overflow and on every half
//! overflow (channel 1 compares at 0x8000), and `now()` combines the two. Channel 0 is the alarm
//! compare; it is only armed when the alarm falls within the next three quarter periods, otherwise
//! `next_period` arms it later.

use core::cell::{Cell, RefCell};
use core::sync::atomic::{AtomicU32, Ordering, compiler_fence};
use core::task::Waker;

use critical_section::{CriticalSection, Mutex};
use embassy_hal_internal::interrupt::InterruptExt;
use embassy_time_driver::Driver as _;
use embassy_time_queue_utils::Queue;

use crate::pac::tpm::vals::{Cmod, Dbgmode, Ps};
use crate::pac::{MCG, SIM, TPM0, interrupt, mcg, sim};

/// Alarm compare channel.
const ALARM_CH: usize = 0;
/// Half period compare channel.
const HALF_CH: usize = 1;

struct Driver {
    /// Number of 2^15 periods elapsed since boot.
    period: AtomicU32,
    alarm: Mutex<Cell<u64>>,
    queue: Mutex<RefCell<Queue>>,
}

fn calc_now(period: u32, counter: u16) -> u64 {
    ((period as u64) << 15) + ((counter as u32 ^ ((period & 1) << 15)) as u64)
}

impl embassy_time_driver::Driver for Driver {
    fn now(&self) -> u64 {
        let period = self.period.load(Ordering::Relaxed);
        compiler_fence(Ordering::Acquire);
        let counter = TPM0.cnt().read().count();
        calc_now(period, counter)
    }

    fn schedule_wake(&self, at: u64, waker: &Waker) {
        critical_section::with(|cs| {
            let mut queue = self.queue.borrow(cs).borrow_mut();

            if queue.schedule_wake(at, waker) {
                let mut next = queue.next_expiration(self.now());

                while !self.set_alarm(cs, next) {
                    next = queue.next_expiration(self.now());
                }
            }
        })
    }
}

impl Driver {
    fn init(&'static self) {
        // 4 MHz fast IRC on MCGIRCLK. FCRDIV resets to /2 and may only be changed while the fast
        // IRC is not in use, so set it before enabling the clock output.
        MCG.sc().modify(|w| w.set_fcrdiv(mcg::vals::Fcrdiv::_000));
        MCG.c2().modify(|w| w.set_ircs(true));
        MCG.c1().modify(|w| w.set_irclken(true));
        while !MCG.s().read().ircst() {}

        critical_section::with(|_| {
            SIM.scgc6().modify(|w| w.set_tpm0(true));
            SIM.sopt2().modify(|w| w.set_tpmsrc(sim::vals::Tpmsrc::_11));
        });

        // Configure with the counter stopped so MOD and CnV writes take effect immediately.
        TPM0.sc().write(|w| {
            w.set_cmod(Cmod::_00);
            w.set_tof(true);
        });
        TPM0.mod_().write(|w| w.set_mod_(u16::MAX));
        TPM0.cnt().write(|w| w.set_count(0));

        // Half period compare: software output compare, interrupt enabled.
        TPM0.cv(HALF_CH).write(|w| w.set_val(0x8000));
        TPM0.csc(HALF_CH).write(|w| {
            w.set_msa(true);
            w.set_chie(true);
            w.set_chf(true);
        });

        // Alarm compare: software output compare, armed by `set_alarm`.
        TPM0.csc(ALARM_CH).write(|w| {
            w.set_msa(true);
            w.set_chf(true);
        });

        TPM0.conf().write(|w| w.set_dbgmode(Dbgmode::_11));

        unsafe { interrupt::TPM0.enable() };

        TPM0.sc().write(|w| {
            w.set_ps(Ps::_010);
            w.set_toie(true);
            w.set_cmod(Cmod::_01);
        });
    }

    fn arm_alarm(&self, armed: bool) {
        // Writing CHF as 1 clears a stale flag from an earlier pass over the same counter value.
        TPM0.csc(ALARM_CH).write(|w| {
            w.set_msa(true);
            w.set_chie(armed);
            w.set_chf(true);
        });
    }

    fn set_alarm(&self, cs: CriticalSection, timestamp: u64) -> bool {
        let alarm = self.alarm.borrow(cs);
        alarm.set(timestamp);

        let t = self.now();
        if timestamp <= t {
            self.arm_alarm(false);
            alarm.set(u64::MAX);
            return false;
        }

        // Write the compare value regardless of whether it is armed now. `next_period` arms it
        // later if the alarm is too far away.
        TPM0.cv(ALARM_CH).write(|w| w.set_val(timestamp as u16));
        let diff = timestamp - t;
        self.arm_alarm(diff < 0xc000);

        // A CnV write only takes effect on the next counter increment, and a match on that same
        // increment is not guaranteed to be seen, so the alarm must still be at least two ticks
        // out. Otherwise report it as possibly missed; the caller re-evaluates the queue.
        let t = self.now();
        if timestamp <= t + 1 {
            self.arm_alarm(false);
            alarm.set(u64::MAX);
            return false;
        }

        true
    }

    fn trigger_alarm(&self, cs: CriticalSection) {
        let mut next = self.queue.borrow_ref_mut(cs).next_expiration(self.now());

        while !self.set_alarm(cs, next) {
            next = self.queue.borrow_ref_mut(cs).next_expiration(self.now());
        }
    }

    fn next_period(&self) {
        // Only the interrupt modifies `period`, so this cannot race.
        let period = self.period.load(Ordering::Relaxed) + 1;
        self.period.store(period, Ordering::Relaxed);
        let t = (period as u64) << 15;

        critical_section::with(|cs| {
            let at = self.alarm.borrow(cs).get();
            if at < t + 0xc000 {
                // The compare value was already written by `set_alarm`. A stale CHF may raise
                // one early interrupt, which `trigger_alarm` handles by re-arming.
                TPM0.csc(ALARM_CH).modify(|w| w.set_chie(true));
            }
        })
    }

    fn on_interrupt(&self) {
        critical_section::with(|cs| {
            // Flags are write-1-to-clear: writing back what was read clears exactly those.
            let status = TPM0.status().read();
            TPM0.status().write_value(status);

            if status.tof() {
                self.next_period();
            }

            if status.chf(HALF_CH) {
                self.next_period();
            }

            if status.chf(ALARM_CH) && TPM0.csc(ALARM_CH).read().chie() {
                self.trigger_alarm(cs);
            }
        })
    }
}

embassy_time_driver::time_driver_impl!(static DRIVER: Driver = Driver {
    period: AtomicU32::new(0),
    alarm: Mutex::new(Cell::new(u64::MAX)),
    queue: Mutex::new(RefCell::new(Queue::new()))
});

pub(crate) fn init() {
    DRIVER.init();
}

#[cfg(feature = "rt")]
#[interrupt]
fn TPM0() {
    DRIVER.on_interrupt();
}
