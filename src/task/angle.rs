use core::f32::consts::PI;

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::{Duration, Instant, Timer};

use crate::driver::tli5012b::Tli5012b;

const TWO_PI: f32 = 2.0 * PI;

pub static STATE: Mutex<CriticalSectionRawMutex, AngleState> = Mutex::new(AngleState::new());

#[derive(Clone, Copy, Debug)]
pub struct AngleState {
    pub theta: f32,
    pub omega: f32,
    pub updated_at_us: u64,
    last_theta: f32,
    initialized: bool,
}

impl AngleState {
    pub const fn new() -> Self {
        Self {
            theta: 0.0,
            omega: 0.0,
            updated_at_us: 0,
            last_theta: 0.0,
            initialized: false,
        }
    }

    pub fn predicted(&self, now_us: u64) -> f32 {
        let dt = now_us.saturating_sub(self.updated_at_us) as f32 * 1.0e-6;
        wrap_angle(self.theta + self.omega * dt)
    }

    fn update(&mut self, theta: f32, now_us: u64) {
        if self.initialized {
            let dt = now_us.saturating_sub(self.updated_at_us).max(1) as f32 * 1.0e-6;
            self.omega = shortest_angle(theta - self.last_theta) / dt;
        } else {
            self.initialized = true;
        }

        self.theta = theta;
        self.last_theta = theta;
        self.updated_at_us = now_us;
    }
}

impl Default for AngleState {
    fn default() -> Self {
        Self::new()
    }
}

pub async fn predicted_now() -> f32 {
    let state = STATE.lock().await;
    state.predicted(now_us())
}

#[embassy_executor::task]
pub async fn task(mut sensor: Tli5012b, period: Duration) {
    loop {
        if let Ok(sample) = sensor.read_angle() {
            let theta = sample.angle_deg * (PI / 180.0);
            STATE.lock().await.update(theta, now_us());
        }

        Timer::after(period).await;
    }
}

fn now_us() -> u64 {
    Instant::now().as_micros() as u64
}

fn shortest_angle(angle: f32) -> f32 {
    wrap_angle(angle)
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
