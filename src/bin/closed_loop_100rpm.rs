#![no_std]
#![no_main]

use core::f32::consts::PI;

use defmt::{info, warn};
use embassy_executor::Spawner;
use embassy_stm32::gpio::{Input, Level, Output, OutputType, Pull, Speed};
use embassy_stm32::time::khz;
use embassy_stm32::timer::low_level::CountingMode;
use embassy_stm32::timer::simple_pwm::{PwmPin, SimplePwm, SimplePwmChannel};
use m5stack_roller485::driver::current::{
    CURRENT_DMA_BUF_WORDS, CURRENT_FRAME_WORDS, CURRENT_READ_WORDS, CurrentConfig,
    OffsetCalibrator, SyncedCurrentSampler, configure_tim1_trgo2_update,
};
use m5stack_roller485::driver::tli5012b::Tli5012b;
use m5stack_roller485::resources::*;
use m5stack_roller485::{rcc, split_resources};
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

const PWM_KHZ: u32 = 42;
const PWM_HZ: f32 = PWM_KHZ as f32 * 1_000.0;
const CONTROL_DT: f32 = (CURRENT_READ_WORDS / CURRENT_FRAME_WORDS) as f32 / PWM_HZ;
const TWO_PI: f32 = 2.0 * PI;

const TARGET_RPM: f32 = 100.0;
const POLE_PAIRS_FALLBACK: f32 = 7.0;
const SENSOR_DIR_FALLBACK: f32 = 1.0;
const ELECTRICAL_OFFSET: f32 = 0.0;
const USE_CURRENT_LOOP: bool = false;

const IQ_MAX_A: f32 = 0.35;
const VOLTAGE_LIMIT_RATIO: f32 = 0.18;
const VOLTAGE_MODE_START_VQ: f32 = 0.6;
const OFFSET_SAMPLES: u32 = 256;
const ALIGN_SECONDS: f32 = 0.6;
const ALIGN_VOLTAGE: f32 = 0.45;
const IDENTIFY_SECONDS: f32 = 2.0;
const IDENTIFY_ELECTRICAL_TURNS: f32 = 2.0;
const IDENTIFY_MIN_MECH_DELTA: f32 = 0.15;
const TELEMETRY_PERIOD_TICKS: u32 = 500;

static ADC_DMA_BUF: StaticCell<[u16; CURRENT_DMA_BUF_WORDS]> = StaticCell::new();

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_stm32::init(rcc::config());
    let resources = split_resources!(p);
    info!("closed-loop 100rpm demo start");

    let mut drv_en = Output::new(resources.motor_io.drv_en, Level::Low, Speed::VeryHigh);
    let mut pwm_en = Output::new(resources.motor_io.pwm_en, Level::Low, Speed::VeryHigh);
    let drv_fault = Input::new(resources.motor_io.drv_fault, Pull::Up);

    let mut sensor = Tli5012b::new(resources.angle_sensor);
    let pwm = SimplePwm::new(
        resources.motor_pwm.pwm_timer,
        Some(PwmPin::new(
            resources.motor_pwm.phase_c_pwm,
            OutputType::PushPull,
        )),
        Some(PwmPin::new(
            resources.motor_pwm.phase_b_pwm,
            OutputType::PushPull,
        )),
        Some(PwmPin::new(
            resources.motor_pwm.phase_a_pwm,
            OutputType::PushPull,
        )),
        None,
        khz(PWM_KHZ),
        CountingMode::CenterAlignedDownInterrupts,
    );
    configure_tim1_trgo2_update();

    let channels = pwm.split();
    let mut pwm_c = channels.ch1;
    let mut pwm_b = channels.ch2;
    let mut pwm_a = channels.ch3;
    pwm_a.enable();
    pwm_b.enable();
    pwm_c.enable();
    set_duty(&mut pwm_a, &mut pwm_b, &mut pwm_c, 0.5, 0.5, 0.5);

    let adc_dma_buf = ADC_DMA_BUF.init([0; CURRENT_DMA_BUF_WORDS]);
    let mut current =
        SyncedCurrentSampler::new(resources.current, adc_dma_buf, CurrentConfig::default());
    let mut current_buf = [0u16; CURRENT_READ_WORDS];

    info!("stage: current offset calibration");
    let mut offset_cal = OffsetCalibrator::new(OFFSET_SAMPLES);
    let offsets = loop {
        let raw = match current.read_latest_raw(&mut current_buf).await {
            Ok(raw) => raw,
            Err(_) => {
                current.clear();
                continue;
            }
        };

        if let Some(offsets) = offset_cal.push(raw) {
            break offsets;
        }
    };
    current.set_offsets(offsets);
    info!(
        "current offsets ia={} ib={} ic={}",
        offsets[0], offsets[1], offsets[2]
    );

    info!("stage: enable driver");
    drv_en.set_high();
    embassy_time::Timer::after_millis(50).await;
    pwm_en.set_high();
    embassy_time::Timer::after_millis(50).await;

    info!("stage: rotor align");
    let align_ticks = (ALIGN_SECONDS / CONTROL_DT) as u32;
    for _ in 0..align_ticks {
        let sample = match current.read_latest(&mut current_buf).await {
            Ok(sample) => sample,
            Err(_) => {
                current.clear();
                continue;
            }
        };
        apply_voltage_vector(
            &mut pwm_a,
            &mut pwm_b,
            &mut pwm_c,
            ALIGN_VOLTAGE,
            0.0,
            0.0,
            sample.vbus.max(1.0),
        );
    }

    info!("stage: identify direction and pole pairs");
    let identify_ticks = (IDENTIFY_SECONDS / CONTROL_DT) as u32;
    let identify_electrical_delta = TWO_PI * IDENTIFY_ELECTRICAL_TURNS;
    let mut mech_delta = 0.0;
    let mut last_identify_theta = read_mech_theta(&mut sensor).unwrap_or(0.0);
    for tick in 0..identify_ticks {
        let sample = match current.read_latest(&mut current_buf).await {
            Ok(sample) => sample,
            Err(_) => {
                current.clear();
                continue;
            }
        };

        let progress = (tick + 1) as f32 / identify_ticks.max(1) as f32;
        let theta_e = identify_electrical_delta * progress;
        apply_voltage_vector(
            &mut pwm_a,
            &mut pwm_b,
            &mut pwm_c,
            ALIGN_VOLTAGE,
            0.0,
            theta_e,
            sample.vbus.max(1.0),
        );

        if let Some(theta) = read_mech_theta(&mut sensor) {
            mech_delta += shortest_angle(theta - last_identify_theta);
            last_identify_theta = theta;
        }
    }

    let raw_pole_pairs = raw_pole_pairs(identify_electrical_delta, mech_delta);
    let (sensor_dir, pole_pairs) = identify_mapping(identify_electrical_delta, mech_delta);
    info!(
        "identify result: electrical_delta={} mech_delta={} raw_pole_pairs={} sensor_dir={} pole_pairs={}",
        identify_electrical_delta, mech_delta, raw_pole_pairs, sensor_dir, pole_pairs
    );

    info!("stage: rotor realign");
    for _ in 0..align_ticks {
        let sample = match current.read_latest(&mut current_buf).await {
            Ok(sample) => sample,
            Err(_) => {
                current.clear();
                continue;
            }
        };
        apply_voltage_vector(
            &mut pwm_a,
            &mut pwm_b,
            &mut pwm_c,
            ALIGN_VOLTAGE,
            0.0,
            0.0,
            sample.vbus.max(1.0),
        );
    }

    let theta_align = read_mech_theta(&mut sensor).unwrap_or(0.0);
    let electrical_offset = wrap_angle(-sensor_dir * pole_pairs * theta_align + ELECTRICAL_OFFSET);
    info!(
        "stage: run 100rpm theta_align={} electrical_offset={} current_loop={}",
        theta_align, electrical_offset, USE_CURRENT_LOOP
    );

    let mut last_theta_m = read_mech_theta(&mut sensor).unwrap_or(0.0);
    let mut vel_pi = if USE_CURRENT_LOOP {
        Pi::new(0.02, 0.3, IQ_MAX_A)
    } else {
        Pi::new(0.04, 0.8, 2.0)
    };
    let mut id_pi = Pi::new(0.25, 30.0, 2.0);
    let mut iq_pi = Pi::new(0.25, 30.0, 2.0);
    let target_omega = TARGET_RPM * TWO_PI / 60.0;
    let mut fault_low_ticks = 0u8;
    let mut telemetry_ticks = 0u32;
    let mut overrun_count = 0u32;
    let mut run_probe = 0u8;

    loop {
        let sample = match current.read_latest(&mut current_buf).await {
            Ok(sample) => sample,
            Err(_) => {
                overrun_count = overrun_count.wrapping_add(1);
                if overrun_count % 100 == 0 {
                    warn!("current sampler overrun count={}", overrun_count);
                }
                current.clear();
                continue;
            }
        };
        if run_probe == 0 {
            info!("run probe: first current sample vbus={}", sample.vbus);
            run_probe = 1;
        }

        if drv_fault.is_low() {
            fault_low_ticks = fault_low_ticks.saturating_add(1);
            if fault_low_ticks >= 20 {
                warn!("driver fault, pwm disabled");
                pwm_en.set_low();
                set_duty(&mut pwm_a, &mut pwm_b, &mut pwm_c, 0.5, 0.5, 0.5);
            }
            continue;
        }
        fault_low_ticks = 0;

        let theta_m = match read_mech_theta(&mut sensor) {
            Some(theta) => theta,
            None => continue,
        };
        if run_probe == 1 {
            info!("run probe: first angle theta_m={}", theta_m);
            run_probe = 2;
        }
        let omega_m = sensor_dir * shortest_angle(theta_m - last_theta_m) / CONTROL_DT;
        last_theta_m = theta_m;

        let theta_e = wrap_angle(sensor_dir * pole_pairs * theta_m + electrical_offset);
        let (sin_t, cos_t) = (libm::sinf(theta_e), libm::cosf(theta_e));

        let (i_alpha, i_beta) = clarke(sample.phase.ia, sample.phase.ib);
        let id = i_alpha * cos_t + i_beta * sin_t;
        let iq = -i_alpha * sin_t + i_beta * cos_t;

        let v_limit = (sample.vbus * VOLTAGE_LIMIT_RATIO).clamp(1.0, 6.0);
        id_pi.set_limit(v_limit);
        iq_pi.set_limit(v_limit);

        let speed_cmd = vel_pi.step(target_omega - omega_m, CONTROL_DT);
        let (iq_ref, vd, vq) = if USE_CURRENT_LOOP {
            (
                speed_cmd,
                id_pi.step(-id, CONTROL_DT),
                iq_pi.step(speed_cmd - iq, CONTROL_DT),
            )
        } else {
            (
                0.0,
                0.0,
                (target_omega.signum() * VOLTAGE_MODE_START_VQ + speed_cmd)
                    .clamp(-v_limit, v_limit),
            )
        };

        let v_alpha = vd * cos_t - vq * sin_t;
        let v_beta = vd * sin_t + vq * cos_t;
        let (da, db, dc) = svpwm(v_alpha, v_beta, sample.vbus.max(1.0));
        set_duty(&mut pwm_a, &mut pwm_b, &mut pwm_c, da, db, dc);
        if run_probe == 2 {
            info!("run probe: first pwm update done");
            run_probe = 3;
        }

        telemetry_ticks = telemetry_ticks.wrapping_add(1);
        if telemetry_ticks >= TELEMETRY_PERIOD_TICKS {
            telemetry_ticks = 0;
            info!(
                "run rpm={} speed_cmd={} iq_ref={} id={} iq={} vq={} vbus={}",
                omega_m * 60.0 / TWO_PI,
                speed_cmd,
                iq_ref,
                id,
                iq,
                vq,
                sample.vbus
            );
        }
    }
}

fn identify_mapping(electrical_delta: f32, mech_delta: f32) -> (f32, f32) {
    if libm::fabsf(mech_delta) < IDENTIFY_MIN_MECH_DELTA {
        warn!(
            "identify movement too small, fallback sensor_dir={} pole_pairs={}",
            SENSOR_DIR_FALLBACK, POLE_PAIRS_FALLBACK
        );
        return (SENSOR_DIR_FALLBACK, POLE_PAIRS_FALLBACK);
    }

    let sensor_dir = if mech_delta >= 0.0 { 1.0 } else { -1.0 };
    let identified = electrical_delta / libm::fabsf(mech_delta);
    let rounded = libm::floorf(identified + 0.5);

    if !(1.0..=30.0).contains(&rounded) {
        warn!(
            "identified pole_pairs out of range, raw={} fallback={}",
            identified, POLE_PAIRS_FALLBACK
        );
        return (sensor_dir, POLE_PAIRS_FALLBACK);
    }

    (sensor_dir, rounded)
}

fn raw_pole_pairs(electrical_delta: f32, mech_delta: f32) -> f32 {
    if libm::fabsf(mech_delta) < IDENTIFY_MIN_MECH_DELTA {
        0.0
    } else {
        electrical_delta / libm::fabsf(mech_delta)
    }
}

fn apply_voltage_vector(
    pwm_a: &mut SimplePwmChannel<'_, embassy_stm32::peripherals::TIM1>,
    pwm_b: &mut SimplePwmChannel<'_, embassy_stm32::peripherals::TIM1>,
    pwm_c: &mut SimplePwmChannel<'_, embassy_stm32::peripherals::TIM1>,
    vd: f32,
    vq: f32,
    theta: f32,
    vbus: f32,
) {
    let (sin_t, cos_t) = (libm::sinf(theta), libm::cosf(theta));
    let v_alpha = vd * cos_t - vq * sin_t;
    let v_beta = vd * sin_t + vq * cos_t;
    let (da, db, dc) = svpwm(v_alpha, v_beta, vbus);
    set_duty(pwm_a, pwm_b, pwm_c, da, db, dc);
}

fn read_mech_theta(sensor: &mut Tli5012b) -> Option<f32> {
    sensor
        .read_angle()
        .ok()
        .map(|sample| sample.angle_deg * (PI / 180.0))
}

fn clarke(ia: f32, ib: f32) -> (f32, f32) {
    const INV_SQRT_3: f32 = 0.577_350_26;
    (ia, (ia + 2.0 * ib) * INV_SQRT_3)
}

fn svpwm(v_alpha: f32, v_beta: f32, vbus: f32) -> (f32, f32, f32) {
    const SQRT_3: f32 = 1.732_050_8;

    let va = v_alpha;
    let vb = -0.5 * v_alpha + 0.5 * SQRT_3 * v_beta;
    let vc = -0.5 * v_alpha - 0.5 * SQRT_3 * v_beta;

    let vmax = va.max(vb).max(vc);
    let vmin = va.min(vb).min(vc);
    let offset = -0.5 * (vmax + vmin);

    (
        clamp01(0.5 + (va + offset) / vbus),
        clamp01(0.5 + (vb + offset) / vbus),
        clamp01(0.5 + (vc + offset) / vbus),
    )
}

fn set_duty(
    pwm_a: &mut SimplePwmChannel<'_, embassy_stm32::peripherals::TIM1>,
    pwm_b: &mut SimplePwmChannel<'_, embassy_stm32::peripherals::TIM1>,
    pwm_c: &mut SimplePwmChannel<'_, embassy_stm32::peripherals::TIM1>,
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
    value.clamp(0.0, 1.0)
}

fn wrap_angle(angle: f32) -> f32 {
    let mut out = angle;
    while out >= PI {
        out -= TWO_PI;
    }
    while out < -PI {
        out += TWO_PI;
    }
    out
}

fn shortest_angle(angle: f32) -> f32 {
    wrap_angle(angle)
}

struct Pi {
    kp: f32,
    ki: f32,
    integ: f32,
    limit: f32,
}

impl Pi {
    fn new(kp: f32, ki: f32, limit: f32) -> Self {
        Self {
            kp,
            ki,
            integ: 0.0,
            limit,
        }
    }

    fn set_limit(&mut self, limit: f32) {
        self.limit = limit;
        self.integ = self.integ.clamp(-limit, limit);
    }

    fn step(&mut self, err: f32, dt: f32) -> f32 {
        self.integ = (self.integ + err * self.ki * dt).clamp(-self.limit, self.limit);
        (self.kp * err + self.integ).clamp(-self.limit, self.limit)
    }
}
