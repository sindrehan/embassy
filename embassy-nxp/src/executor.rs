//! Kinetis thread-mode executor with low-power sleep support.
//!
//! Enable the `executor-thread` feature on `embassy-nxp`, do not enable an
//! `embassy-executor` `platform-*` feature, and select this executor in the main macro:
//!
//! ```rust,ignore
//! #[embassy_executor::main(
//!     executor = "embassy_nxp::executor::Executor",
//!     entry = "cortex_m_rt::entry"
//! )]
//! async fn main(_spawner: embassy_executor::Spawner) {
//!     // ...
//! }
//! ```

use core::marker::PhantomData;
use core::sync::atomic::{AtomicBool, Ordering};

use embassy_executor::{Spawner, raw};

const THREAD_PENDER: usize = usize::MAX;
static WORK_PENDING: AtomicBool = AtomicBool::new(false);

struct KinetisPender;

embassy_executor::pender_impl!(KinetisPender);

impl embassy_executor::pender::Pender for KinetisPender {
    fn pend(context: *mut ()) {
        debug_assert_eq!(context as usize, THREAD_PENDER);
        WORK_PENDING.store(true, Ordering::SeqCst);
    }
}

/// Thread-mode executor that enters the configured Kinetis power mode when idle.
pub struct Executor {
    inner: raw::Executor,
    not_send: PhantomData<*mut ()>,
}

impl Executor {
    /// Create an executor.
    pub fn new() -> Self {
        Self {
            inner: raw::Executor::new(THREAD_PENDER as *mut ()),
            not_send: PhantomData,
        }
    }

    /// Run the executor.
    pub fn run(&'static mut self, init: impl FnOnce(Spawner)) -> ! {
        init(self.inner.spawner());

        loop {
            unsafe {
                self.inner.poll();

                critical_section::with(|cs| {
                    if WORK_PENDING.load(Ordering::SeqCst) {
                        WORK_PENDING.store(false, Ordering::SeqCst);
                    } else {
                        embassy_executor::trace_idle();
                        crate::power::sleep(cs);
                    }
                });
            }
        }
    }
}

impl Default for Executor {
    fn default() -> Self {
        Self::new()
    }
}
