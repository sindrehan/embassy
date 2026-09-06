//! Interrupt-buffered reception independent of executor polling.

use super::*;

pub(super) struct RxBuffer {
    storage: &'static mut [u8],
    head: usize,
    len: usize,
    error: Option<Error>,
    guard: Option<crate::power::WakeGuard>,
}

impl RxBuffer {
    fn clear(&mut self) {
        self.head = 0;
        self.len = 0;
        self.error = None;
    }

    fn push(&mut self, byte: u8) {
        if self.len == self.storage.len() {
            self.fail(Error::Overrun);
        } else {
            self.storage[(self.head + self.len) % self.storage.len()] = byte;
            self.len += 1;
        }
    }

    fn fail(&mut self, error: Error) {
        self.clear();
        self.error = Some(error);
    }

    fn read(&mut self, output: &mut [u8]) -> Result<usize, Error> {
        if let Some(error) = self.error {
            return Err(error);
        }
        // Bound the interrupt-masked copy, including when the application supplies a large buffer.
        let n = output.len().min(self.len).min(32);
        for byte in &mut output[..n] {
            *byte = self.storage[self.head];
            self.head = (self.head + 1) % self.storage.len();
        }
        self.len -= n;
        Ok(n)
    }
}

/// An interrupt-driven receive ring that continues filling while the executor is busy.
///
/// Construct with [`LpuartRx::into_buffered`]. RX keeps the executor in WAIT until suspended or
/// dropped, even when no read is pending. Interrupts must still be serviced promptly; neither
/// this ring nor the hardware FIFO can tolerate unlimited interrupt masking or sustained input
/// faster than the consumer. Size the buffer for the longest expected task stall.
///
/// Hardware errors or a full ring discard the buffered data and latch an error. Reads return that
/// error until [`recover`](Self::recover) or a suspend/resume cycle discards stale input and restarts
/// reception. The HAL does not identify protocol frame boundaries after data loss.
///
/// Read cancellation consumes no bytes and leaves background reception running. Dropping this
/// driver stops background reception before releasing the underlying UART half.
pub struct BufferedLpuartRx<'d> {
    rx: Option<LpuartRx<'d, Async>>,
}

impl<'d> LpuartRx<'d, Async> {
    /// Start interrupt-buffered reception. Unread FIFO data and errors are discarded.
    ///
    /// The nonempty buffer must be static because interrupts can access it even if the driver is
    /// forgotten. [`BufferedLpuartRx::into_inner`] returns it for reuse. A configured DMA channel
    /// remains owned but is not used by this receiver. Uses the existing [`InterruptHandler`].
    pub fn into_buffered(mut self, buffer: &'static mut [u8]) -> BufferedLpuartRx<'d> {
        assert!(!buffer.is_empty(), "RX buffer must not be empty");
        let suspended = self.is_suspended();
        self.suspend();
        critical_section::with(|cs| {
            self.state.resources.borrow(cs).borrow_mut().rx_buffer = Some(RxBuffer {
                storage: buffer,
                head: 0,
                len: 0,
                error: None,
                guard: None,
            });
        });
        let mut buffered = BufferedLpuartRx { rx: Some(self) };
        if !suspended {
            buffered.resume();
        }
        buffered
    }
}

impl<'d> BufferedLpuartRx<'d> {
    /// Read available bytes, or wait for at least one byte. Cancellation consumes no data.
    pub async fn read(&mut self, output: &mut [u8]) -> Result<usize, Error> {
        if output.is_empty() {
            return Ok(0);
        }
        poll_fn(|cx| {
            self.rx.as_ref().unwrap().state.rx_waker.register(cx.waker());
            match self.try_read(output) {
                Ok(0) => Poll::Pending,
                result => Poll::Ready(result),
            }
        })
        .await
    }

    /// Read buffered bytes without waiting. Returns zero when empty.
    pub fn try_read(&mut self, output: &mut [u8]) -> Result<usize, Error> {
        if output.is_empty() {
            return Ok(0);
        }
        let rx = self.rx.as_ref().unwrap();
        if rx.is_suspended() {
            return Err(Error::Suspended);
        }
        critical_section::with(|cs| {
            rx.state
                .resources
                .borrow(cs)
                .borrow_mut()
                .rx_buffer
                .as_mut()
                .unwrap()
                .read(output)
        })
    }

    /// Stop reception, disconnect RX and discard buffered input. Drop pending read futures first.
    pub fn suspend(&mut self) {
        let rx = self.rx.as_mut().unwrap();
        rx.suspend();
        critical_section::with(|cs| {
            let mut resources = rx.state.resources.borrow(cs).borrow_mut();
            let ring = resources.rx_buffer.as_mut().unwrap();
            ring.clear();
            ring.guard.take();
        });
    }

    /// Discard stale input and resume background reception. Does nothing unless suspended.
    pub fn resume(&mut self) {
        let rx = self.rx.as_mut().unwrap();
        if rx.is_suspended() {
            critical_section::with(|cs| {
                let mut resources = rx.state.resources.borrow(cs).borrow_mut();
                let ring = resources.rx_buffer.as_mut().unwrap();
                ring.clear();
                ring.guard = Some(crate::power::wake_guard());
                rx.resume();
                arm(rx.info.regs);
            });
        }
    }

    /// Whether reception is suspended and RX is disconnected.
    pub fn is_suspended(&self) -> bool {
        self.rx.as_ref().unwrap().is_suspended()
    }

    /// Discard all queued data and errors and restart reception, unless suspended.
    /// Coordinate with the sender or resynchronize the protocol after calling this.
    pub fn recover(&mut self) {
        let suspended = self.is_suspended();
        self.suspend();
        if !suspended {
            self.resume();
        }
    }

    /// Stop buffering and return the suspended UART half and its buffer for reuse.
    pub fn into_inner(mut self) -> (LpuartRx<'d, Async>, &'static mut [u8]) {
        self.suspend();
        let rx = self.rx.take().unwrap();
        let buffer = critical_section::with(|cs| {
            rx.state
                .resources
                .borrow(cs)
                .borrow_mut()
                .rx_buffer
                .take()
                .unwrap()
                .storage
        });
        (rx, buffer)
    }
}

impl Drop for BufferedLpuartRx<'_> {
    fn drop(&mut self) {
        if self.rx.is_some() {
            self.suspend();
            critical_section::with(|cs| {
                self.rx
                    .as_ref()
                    .unwrap()
                    .state
                    .resources
                    .borrow(cs)
                    .borrow_mut()
                    .rx_buffer
                    .take();
            });
        }
    }
}

impl embedded_io::ErrorType for BufferedLpuartRx<'_> {
    type Error = Error;
}

impl embedded_io_async::Read for BufferedLpuartRx<'_> {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Error> {
        self.read(buf).await
    }
}

fn arm(regs: Regs) {
    regs.ctrl().modify(|w| {
        w.set_rie(true);
        w.set_orie(true);
    });
}

pub(super) fn on_interrupt(regs: Regs, state: &State) -> bool {
    critical_section::with(|cs| {
        let mut resources = state.resources.borrow(cs).borrow_mut();
        let Some(ring) = &mut resources.rx_buffer else {
            return false;
        };
        let ctrl = regs.ctrl().read();
        if !ctrl.rie() && !ctrl.orie() {
            return true;
        }
        if regs.stat().read().or() {
            ring.fail(Error::Overrun);
        }
        // Bound each invocation so other interrupts can run between FIFO batches.
        for _ in 0..8 {
            if ring.error.is_some() || regs.water().read().rxcount() == 0 {
                break;
            }
            let data = regs.data().read();
            if data.fretsc() {
                ring.fail(Error::Framing);
            } else if data.paritye() {
                ring.fail(Error::Parity);
            } else if data.noisy() {
                ring.fail(Error::Noise);
            } else {
                ring.push(data.0 as u8);
            }
        }
        if ring.error.is_some() {
            stop_read(regs);
        }
        if ring.len != 0 || ring.error.is_some() {
            state.rx_waker.wake();
        }
        true
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    extern crate std;

    fn ring(n: usize) -> RxBuffer {
        RxBuffer {
            storage: std::boxed::Box::leak(std::vec![0; n].into_boxed_slice()),
            head: 0,
            len: 0,
            error: None,
            guard: None,
        }
    }

    #[test]
    fn wrap_and_recovery() {
        let mut ring = ring(3);
        ring.push(1);
        ring.push(2);
        let mut byte = [0];
        assert_eq!(ring.read(&mut byte), Ok(1));
        assert_eq!(byte, [1]);
        ring.push(3);
        ring.push(4);
        let mut bytes = [0; 3];
        assert_eq!(ring.read(&mut bytes), Ok(3));
        assert_eq!(bytes, [2, 3, 4]);
        for n in 0..4 {
            ring.push(n);
        }
        assert_eq!(ring.read(&mut bytes), Err(Error::Overrun));
        assert_eq!(ring.read(&mut bytes), Err(Error::Overrun));
        ring.clear();
        ring.push(9);
        assert_eq!(ring.read(&mut bytes), Ok(1));
        assert_eq!(bytes[0], 9);
    }
}
