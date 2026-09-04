//! Fades the FRDM-KL82Z RGB LED between red, green, and blue.
//!
//! TPM0 drives the active-low red and green LEDs on PTC1/TPM0_CH0 and PTC2/TPM0_CH1. The blue LED
//! is driven by FlexIO timer 0 on PTC0/FXIO0_D12.
#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_nxp::peripherals;
use embassy_nxp::pwm::{Config, FlexioConfig, FlexioPwm, Polarity, Pwm};
use embassy_nxp_mkl82z7_examples as _;
use embassy_time::Timer;

const PWM_FREQUENCY: u32 = 15_625;
const FADE_STEPS: u16 = 256;
const STEP_DELAY_MS: u64 = 8;

fn gamma_correct(value: u16) -> u16 {
    (u32::from(value) * u32::from(value) / u32::from(FADE_STEPS)) as u16
}

fn mix(from: u16, to: u16, step: u16) -> u16 {
    let from_weight = u32::from(FADE_STEPS - step);
    let to_weight = u32::from(step);
    ((u32::from(from) * from_weight + u32::from(to) * to_weight) / u32::from(FADE_STEPS)) as u16
}

fn show_color(
    pwm: &mut Pwm<'_, peripherals::TPM0>,
    flexio_pwm: &mut FlexioPwm<'_, peripherals::FLEXIO0>,
    red: u16,
    green: u16,
    blue_level: u16,
) {
    pwm.set_duty_cycle::<0>(gamma_correct(red));
    pwm.set_duty_cycle::<1>(gamma_correct(green));
    flexio_pwm.set_duty_cycle::<0>(gamma_correct(blue_level));
}

async fn fade(
    pwm: &mut Pwm<'_, peripherals::TPM0>,
    flexio_pwm: &mut FlexioPwm<'_, peripherals::FLEXIO0>,
    from: (u16, u16, u16),
    to: (u16, u16, u16),
) {
    for step in 0..=FADE_STEPS {
        show_color(
            pwm,
            flexio_pwm,
            mix(from.0, to.0, step),
            mix(from.1, to.1, step),
            mix(from.2, to.2, step),
        );
        Timer::after_millis(STEP_DELAY_MS).await;
    }
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_nxp::init(Default::default());

    let mut pwm_config = Config::default();
    pwm_config.frequency = PWM_FREQUENCY;
    let mut pwm = Pwm::new(p.TPM0, pwm_config);
    pwm.enable_channel::<0>(p.PTC1, Polarity::ActiveLow);
    pwm.enable_channel::<1>(p.PTC2, Polarity::ActiveLow);

    let mut flexio_config = FlexioConfig::default();
    flexio_config.frequency = PWM_FREQUENCY;
    let mut flexio_pwm = FlexioPwm::new(p.FLEXIO0);
    flexio_pwm.enable_channel::<0>(p.PTC0, Polarity::ActiveLow, flexio_config);

    defmt::info!(
        "RGB fade: TPM PWM at {} Hz, FlexIO PWM at {} Hz",
        pwm.frequency(),
        flexio_pwm.frequency::<0>()
    );

    const RED: (u16, u16, u16) = (FADE_STEPS, 0, 0);
    const GREEN: (u16, u16, u16) = (0, FADE_STEPS, 0);
    const BLUE: (u16, u16, u16) = (0, 0, FADE_STEPS);

    loop {
        fade(&mut pwm, &mut flexio_pwm, RED, GREEN).await;
        fade(&mut pwm, &mut flexio_pwm, GREEN, BLUE).await;
        fade(&mut pwm, &mut flexio_pwm, BLUE, RED).await;
    }
}
