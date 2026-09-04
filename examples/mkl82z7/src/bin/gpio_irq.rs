//! Pin interrupts, checked through the D11 to D12 jumper: PTC6 (D11) drives,
//! PTC7 (D12) waits. Each wait is raced against a timer that flips the output
//! after 50 ms, so the measured latency shows the wait really woke on the
//! event; a wait that must not fire is given a deadline instead.
#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_nxp::gpio::{Input, Level, Output, Pull};
use embassy_nxp_mkl82z7_examples as _;
use embassy_time::{Duration, Instant, Timer, with_timeout};

async fn flip_after(out: &mut Output<'_>, level: Level, ms: u64) {
    Timer::after_millis(ms).await;
    match level {
        Level::High => out.set_high(),
        Level::Low => out.set_low(),
    }
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_nxp::init(Default::default());
    defmt::info!("gpio irq: PTC6 (D11) drives PTC7 (D12) through the jumper");

    let mut out = Output::new(p.PTC6, Level::Low);
    let mut inp = Input::new(p.PTC7, Pull::None);
    Timer::after_millis(1).await;
    defmt::assert!(
        inp.is_low(),
        "jumper D11-D12 missing? input should follow the low output"
    );

    // Level waits that already hold return at once.
    let t = Instant::now();
    inp.wait_for_low().await;
    defmt::assert!(t.elapsed() < Duration::from_millis(1));
    defmt::info!("wait_for_low on a low pin: immediate");

    // Level and edge waits woken by the output flipping 50 ms later.
    for (name, level, wait) in [
        ("wait_for_high", Level::High, 0u8),
        ("wait_for_falling_edge", Level::Low, 1),
        ("wait_for_rising_edge", Level::High, 2),
        ("wait_for_any_edge", Level::Low, 3),
        ("wait_for_low", Level::Low, 4),
    ] {
        if wait == 4 {
            out.set_high();
            Timer::after_millis(1).await;
        }
        let t = Instant::now();
        let waiter = async {
            match wait {
                0 => inp.wait_for_high().await,
                1 => inp.wait_for_falling_edge().await,
                2 => inp.wait_for_rising_edge().await,
                3 => inp.wait_for_any_edge().await,
                _ => inp.wait_for_low().await,
            }
        };
        with_timeout(
            Duration::from_millis(500),
            join(flip_after(&mut out, level, 50), waiter),
        )
        .await
        .expect("pin interrupt never fired");
        let ms = t.elapsed().as_millis();
        defmt::assert!((49..=52).contains(&ms), "{} woke after {} ms, expected ~50", name, ms);
        defmt::assert_eq!(inp.read(), level);
        defmt::info!("{}: woke after {} ms", name, ms);
    }

    // No spurious wake: a rising edge cannot happen while the line stays low.
    out.set_low();
    Timer::after_millis(1).await;
    let r = with_timeout(Duration::from_millis(100), inp.wait_for_rising_edge()).await;
    defmt::assert!(r.is_err(), "rising edge reported without a transition");
    defmt::info!("no spurious wake in 100 ms");

    // The dropped future left the pin interrupt disarmed; a later wait still works.
    let t = Instant::now();
    with_timeout(
        Duration::from_millis(500),
        join(flip_after(&mut out, Level::High, 50), inp.wait_for_rising_edge()),
    )
    .await
    .expect("pin interrupt never fired after a cancelled wait");
    defmt::info!("wait after cancel: woke after {} ms", t.elapsed().as_millis());

    defmt::info!("gpio irq passed");
    embassy_nxp_mkl82z7_examples::exit()
}
