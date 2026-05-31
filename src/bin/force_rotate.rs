#![no_std]
#![no_main]

use core::f32::consts::PI;

use defmt::{info, warn};
use embassy_executor::Spawner;
use embassy_stm32::gpio::{Input, Level, Output, OutputType, Pull, Speed};
use embassy_stm32::time::khz;
use embassy_stm32::timer::low_level::CountingMode;
use embassy_stm32::timer::simple_pwm::{PwmPin, SimplePwm};
use embassy_time::{Duration, Ticker};
use m5stack_roller485::rcc;
use {defmt_rtt as _, panic_probe as _};

const PWM_KHZ: u32 = 42;
const LOOP_HZ: u64 = 1_000;
const DUTY_AMPLITUDE: f32 = 0.22;
const START_ELECTRICAL_HZ: f32 = 0.8;
const TARGET_ELECTRICAL_HZ: f32 = 3.0;
const RAMP_SECONDS: f32 = 2.0;
const ENABLE_SETTLE_MS: u64 = 50;
const FAULT_DEBOUNCE_TICKS: u8 = 20;
const TWO_PI: f32 = 2.0 * PI;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_stm32::init(rcc::config());
    info!("force rotate demo start");

    let mut drv_en = Output::new(p.PB1, Level::Low, Speed::VeryHigh);
    let mut pwm_en = Output::new(p.PB2, Level::Low, Speed::VeryHigh);
    let drv_fault = Input::new(p.PB0, Pull::Up);

    let ch1_pin = PwmPin::new(p.PA8, OutputType::PushPull);
    let ch2_pin = PwmPin::new(p.PA9, OutputType::PushPull);
    let ch3_pin = PwmPin::new(p.PA10, OutputType::PushPull);
    let pwm = SimplePwm::new(
        p.TIM1,
        Some(ch1_pin),
        Some(ch2_pin),
        Some(ch3_pin),
        None,
        khz(PWM_KHZ),
        CountingMode::CenterAlignedDownInterrupts,
    );

    let channels = pwm.split();
    let mut pwm_c = channels.ch1;
    let mut pwm_b = channels.ch2;
    let mut pwm_a = channels.ch3;

    pwm_c.enable();
    pwm_b.enable();
    pwm_a.enable();

    set_center(&mut pwm_a, &mut pwm_b, &mut pwm_c);
    drv_en.set_high();
    embassy_time::Timer::after(Duration::from_millis(ENABLE_SETTLE_MS)).await;
    pwm_en.set_high();
    embassy_time::Timer::after(Duration::from_millis(ENABLE_SETTLE_MS)).await;

    let mut theta = 0.0f32;
    let mut electrical_hz = START_ELECTRICAL_HZ;
    let ramp_step = (TARGET_ELECTRICAL_HZ - START_ELECTRICAL_HZ) / (RAMP_SECONDS * LOOP_HZ as f32);
    let mut ticker = Ticker::every(Duration::from_hz(LOOP_HZ));
    let mut fault_low_ticks = 0u8;
    let mut pwm_outputs_enabled = true;

    loop {
        ticker.next().await;

        if drv_fault.is_low() {
            fault_low_ticks = fault_low_ticks.saturating_add(1);
            if fault_low_ticks >= FAULT_DEBOUNCE_TICKS && pwm_outputs_enabled {
                warn!("driver fault asserted after debounce, pwm disabled");
                pwm_en.set_low();
                pwm_outputs_enabled = false;
                set_center(&mut pwm_a, &mut pwm_b, &mut pwm_c);
            }
            continue;
        }
        fault_low_ticks = 0;
        if !pwm_outputs_enabled {
            info!("driver fault released, pwm re-enabled");
            pwm_en.set_high();
            pwm_outputs_enabled = true;
            embassy_time::Timer::after(Duration::from_millis(ENABLE_SETTLE_MS)).await;
        }

        if electrical_hz < TARGET_ELECTRICAL_HZ {
            electrical_hz = (electrical_hz + ramp_step).min(TARGET_ELECTRICAL_HZ);
        }

        theta = wrap_0_2pi(theta + TWO_PI * electrical_hz / LOOP_HZ as f32);
        let (da, db, dc) = open_loop_duty(theta);
        set_duty(&mut pwm_a, &mut pwm_b, &mut pwm_c, da, db, dc);
    }
}

fn open_loop_duty(theta: f32) -> (f32, f32, f32) {
    let a = 0.5 + DUTY_AMPLITUDE * libm::sinf(theta);
    let b = 0.5 + DUTY_AMPLITUDE * libm::sinf(theta - TWO_PI / 3.0);
    let c = 0.5 + DUTY_AMPLITUDE * libm::sinf(theta + TWO_PI / 3.0);
    (a, b, c)
}

fn set_center(
    pwm_a: &mut embassy_stm32::timer::simple_pwm::SimplePwmChannel<
        '_,
        embassy_stm32::peripherals::TIM1,
    >,
    pwm_b: &mut embassy_stm32::timer::simple_pwm::SimplePwmChannel<
        '_,
        embassy_stm32::peripherals::TIM1,
    >,
    pwm_c: &mut embassy_stm32::timer::simple_pwm::SimplePwmChannel<
        '_,
        embassy_stm32::peripherals::TIM1,
    >,
) {
    set_duty(pwm_a, pwm_b, pwm_c, 0.5, 0.5, 0.5);
}

fn set_duty(
    pwm_a: &mut embassy_stm32::timer::simple_pwm::SimplePwmChannel<
        '_,
        embassy_stm32::peripherals::TIM1,
    >,
    pwm_b: &mut embassy_stm32::timer::simple_pwm::SimplePwmChannel<
        '_,
        embassy_stm32::peripherals::TIM1,
    >,
    pwm_c: &mut embassy_stm32::timer::simple_pwm::SimplePwmChannel<
        '_,
        embassy_stm32::peripherals::TIM1,
    >,
    da: f32,
    db: f32,
    dc: f32,
) {
    let max_duty = pwm_a.max_duty_cycle() as f32;
    pwm_a.set_duty_cycle((clamp01(da) * max_duty) as u32);
    pwm_b.set_duty_cycle((clamp01(db) * max_duty) as u32);
    pwm_c.set_duty_cycle((clamp01(dc) * max_duty) as u32);
}

fn clamp01(value: f32) -> f32 {
    if value < 0.0 {
        0.0
    } else if value > 1.0 {
        1.0
    } else {
        value
    }
}

fn wrap_0_2pi(theta: f32) -> f32 {
    if theta >= TWO_PI {
        theta - TWO_PI
    } else {
        theta
    }
}
