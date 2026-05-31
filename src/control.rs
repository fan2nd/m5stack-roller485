use core::f32::consts::PI;

const SQRT_3: f32 = 1.732_050_8;
const INV_SQRT_3: f32 = 0.577_350_26;
const TWO_PI: f32 = 2.0 * PI;

pub const FAULT_DRV: u32 = 1 << 0;
pub const FAULT_OVERCURRENT: u32 = 1 << 1;
pub const FAULT_COMM_TIMEOUT: u32 = 1 << 2;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ControlMode {
    Speed,
    Position,
    Damping,
    Knob,
    Weightless,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DriveState {
    Boot,
    Calibrating,
    Ready,
    Enabled,
    Fault,
}

#[derive(Clone, Copy, Debug)]
pub struct Gains {
    pub id_kp: f32,
    pub id_ki: f32,
    pub iq_kp: f32,
    pub iq_ki: f32,
    pub vel_kp: f32,
    pub vel_ki: f32,
    pub pos_kp: f32,
    pub pos_kd: f32,
}

impl Default for Gains {
    fn default() -> Self {
        Self {
            id_kp: 1.2,
            id_ki: 120.0,
            iq_kp: 1.2,
            iq_ki: 120.0,
            vel_kp: 0.08,
            vel_ki: 2.0,
            pos_kp: 3.0,
            pos_kd: 0.06,
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct Limits {
    pub iq_max: f32,
    pub vel_max: f32,
    pub acc_max: f32,
}

impl Default for Limits {
    fn default() -> Self {
        Self {
            iq_max: 4.0,
            vel_max: 40.0,
            acc_max: 120.0,
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct Effects {
    pub damping_k: f32,
    pub detent_step_rad: f32,
    pub detent_k: f32,
    pub weightless_friction: f32,
}

impl Default for Effects {
    fn default() -> Self {
        Self {
            damping_k: 0.08,
            detent_step_rad: PI / 18.0,
            detent_k: 0.7,
            weightless_friction: 0.02,
        }
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub struct PhaseCurrents {
    pub ia: f32,
    pub ib: f32,
    pub ic: f32,
}

#[derive(Clone, Copy, Debug)]
pub struct Telemetry {
    pub mode: ControlMode,
    pub state: DriveState,
    pub theta: f32,
    pub omega: f32,
    pub id: f32,
    pub iq: f32,
    pub iq_ref: f32,
    pub vbus: f32,
    pub temp_c: f32,
    pub fault_bits: u32,
}

#[derive(Clone, Copy, Debug)]
pub struct DutyCycles {
    pub a: f32,
    pub b: f32,
    pub c: f32,
}

#[derive(Clone, Copy, Debug)]
pub struct ControlTick {
    pub duty: DutyCycles,
    pub telem: Telemetry,
}

#[derive(Clone, Copy, Debug)]
struct Pi {
    kp: f32,
    ki: f32,
    integ: f32,
    integ_limit: f32,
    out_limit: f32,
}

impl Pi {
    fn new(kp: f32, ki: f32, out_limit: f32) -> Self {
        Self {
            kp,
            ki,
            integ: 0.0,
            integ_limit: out_limit,
            out_limit,
        }
    }

    fn set_gains(&mut self, kp: f32, ki: f32) {
        self.kp = kp;
        self.ki = ki;
    }

    fn reset(&mut self) {
        self.integ = 0.0;
    }

    fn step(&mut self, err: f32, dt: f32) -> f32 {
        self.integ += err * self.ki * dt;
        self.integ = clamp(self.integ, -self.integ_limit, self.integ_limit);

        let out = self.kp * err + self.integ;
        clamp(out, -self.out_limit, self.out_limit)
    }
}

pub struct FocController {
    pub mode: ControlMode,
    pub state: DriveState,
    pub target: f32,
    pub gains: Gains,
    pub limits: Limits,
    pub effects: Effects,
    pub vbus_divider_ratio: f32,

    pub fault_bits: u32,

    adc_offsets: [f32; 3],
    offset_acc: [f32; 3],
    cal_samples: u32,

    theta_est: f32,
    omega_est: f32,

    id_pi: Pi,
    iq_pi: Pi,
    vel_pi: Pi,

    last_iq_ref: f32,
}

impl Default for FocController {
    fn default() -> Self {
        let gains = Gains::default();
        let limits = Limits::default();
        Self {
            mode: ControlMode::Weightless,
            state: DriveState::Boot,
            target: 0.0,
            gains,
            limits,
            effects: Effects::default(),
            vbus_divider_ratio: 2.2 / (1.2 + 12.0),
            fault_bits: 0,
            adc_offsets: [2048.0; 3],
            offset_acc: [0.0; 3],
            cal_samples: 0,
            theta_est: 0.0,
            omega_est: 0.0,
            id_pi: Pi::new(gains.id_kp, gains.id_ki, 8.0),
            iq_pi: Pi::new(gains.iq_kp, gains.iq_ki, 8.0),
            vel_pi: Pi::new(gains.vel_kp, gains.vel_ki, limits.iq_max),
            last_iq_ref: 0.0,
        }
    }
}

impl FocController {
    pub fn boot_to_calibrating(&mut self) {
        if self.state == DriveState::Boot {
            self.state = DriveState::Calibrating;
        }
    }

    pub fn request_enable(&mut self) {
        if self.state == DriveState::Ready {
            self.state = DriveState::Enabled;
            self.id_pi.reset();
            self.iq_pi.reset();
            self.vel_pi.reset();
        }
    }

    pub fn request_disable(&mut self) {
        if self.state != DriveState::Fault {
            self.state = DriveState::Ready;
        }
        self.last_iq_ref = 0.0;
    }

    pub fn clear_fault(&mut self) {
        self.fault_bits = 0;
        self.state = DriveState::Ready;
        self.last_iq_ref = 0.0;
        self.id_pi.reset();
        self.iq_pi.reset();
        self.vel_pi.reset();
    }

    pub fn set_fault(&mut self, bits: u32) {
        self.fault_bits |= bits;
        self.state = DriveState::Fault;
    }

    pub fn set_mode(&mut self, mode: ControlMode) {
        self.mode = mode;
    }

    pub fn set_target(&mut self, value: f32) {
        self.target = value;
    }

    pub fn set_gains(&mut self, gains: Gains) {
        self.gains = gains;
        self.id_pi.set_gains(gains.id_kp, gains.id_ki);
        self.iq_pi.set_gains(gains.iq_kp, gains.iq_ki);
        self.vel_pi.set_gains(gains.vel_kp, gains.vel_ki);
    }

    pub fn set_limits(&mut self, limits: Limits) {
        self.limits = limits;
        self.vel_pi.out_limit = limits.iq_max;
        self.vel_pi.integ_limit = limits.iq_max;
    }

    pub fn ingest_offsets(&mut self, ia_adc: u16, ib_adc: u16, ic_adc: u16) {
        if self.state != DriveState::Calibrating {
            return;
        }

        self.offset_acc[0] += ia_adc as f32;
        self.offset_acc[1] += ib_adc as f32;
        self.offset_acc[2] += ic_adc as f32;
        self.cal_samples += 1;

        if self.cal_samples >= 64 {
            let n = self.cal_samples as f32;
            self.adc_offsets[0] = self.offset_acc[0] / n;
            self.adc_offsets[1] = self.offset_acc[1] / n;
            self.adc_offsets[2] = self.offset_acc[2] / n;
            self.state = DriveState::Ready;
        }
    }

    pub fn step(
        &mut self,
        adc_vbus: u16,
        adc_ia: u16,
        adc_ib: u16,
        adc_ic: u16,
        temp_raw: u16,
        dt_s: f32,
    ) -> ControlTick {
        let phase = self.decode_phase_currents(adc_ia, adc_ib, adc_ic);
        let vbus = self.decode_vbus(adc_vbus);
        let temp_c = self.decode_temp(temp_raw);

        // Placeholder observer: in production this should come from SPI angle sensor.
        let accel = (self.last_iq_ref * 8.0) - self.omega_est * 0.25;
        self.omega_est = clamp(
            self.omega_est + accel * dt_s,
            -self.limits.vel_max,
            self.limits.vel_max,
        );
        self.theta_est = wrap_angle(self.theta_est + self.omega_est * dt_s);

        let (i_alpha, i_beta) = clarke(phase);
        let (sin_t, cos_t) = (libm::sinf(self.theta_est), libm::cosf(self.theta_est));
        let id = i_alpha * cos_t + i_beta * sin_t;
        let iq = -i_alpha * sin_t + i_beta * cos_t;

        let iq_ref = self.compute_iq_ref(dt_s);
        let id_ref = 0.0;

        let vd = self.id_pi.step(id_ref - id, dt_s);
        let vq = self.iq_pi.step(iq_ref - iq, dt_s);

        let v_alpha = vd * cos_t - vq * sin_t;
        let v_beta = vd * sin_t + vq * cos_t;

        let duty = if self.state == DriveState::Enabled {
            svpwm(v_alpha, v_beta, vbus.max(1.0))
        } else {
            DutyCycles {
                a: 0.5,
                b: 0.5,
                c: 0.5,
            }
        };

        if phase.ia.abs() > self.limits.iq_max * 1.5
            || phase.ib.abs() > self.limits.iq_max * 1.5
            || phase.ic.abs() > self.limits.iq_max * 1.5
        {
            self.set_fault(FAULT_OVERCURRENT);
        }

        ControlTick {
            duty,
            telem: Telemetry {
                mode: self.mode,
                state: self.state,
                theta: self.theta_est,
                omega: self.omega_est,
                id,
                iq,
                iq_ref,
                vbus,
                temp_c,
                fault_bits: self.fault_bits,
            },
        }
    }

    fn decode_phase_currents(&self, adc_ia: u16, adc_ib: u16, adc_ic: u16) -> PhaseCurrents {
        // This scale is intentionally conservative for first bring-up.
        // Fine calibration can be exposed later over CLI if needed.
        const AMP_PER_LSB: f32 = 0.002;

        let ia = (adc_ia as f32 - self.adc_offsets[0]) * AMP_PER_LSB;
        let ib = (adc_ib as f32 - self.adc_offsets[1]) * AMP_PER_LSB;
        let ic = (adc_ic as f32 - self.adc_offsets[2]) * AMP_PER_LSB;

        // Fixed same-polarity mapping requested by hardware owner.
        PhaseCurrents { ia, ib, ic }
    }

    fn decode_vbus(&self, adc_vbus: u16) -> f32 {
        let v_adc = (adc_vbus as f32) * (3.3 / 4095.0);
        let ratio = self.vbus_divider_ratio.max(0.001);
        v_adc / ratio
    }

    fn decode_temp(&self, temp_raw: u16) -> f32 {
        // Placeholder linear mapping for telemetry sanity.
        (temp_raw as f32) * (100.0 / 4095.0)
    }

    fn compute_iq_ref(&mut self, dt_s: f32) -> f32 {
        let raw = match self.mode {
            ControlMode::Speed => {
                let vel_err = self.target - self.omega_est;
                self.vel_pi.step(vel_err, dt_s)
            }
            ControlMode::Position => {
                let pos_err = shortest_angle_error(self.target, self.theta_est);
                let vel_target = clamp(
                    self.gains.pos_kp * pos_err - self.gains.pos_kd * self.omega_est,
                    -self.limits.vel_max,
                    self.limits.vel_max,
                );
                self.vel_pi.step(vel_target - self.omega_est, dt_s)
            }
            ControlMode::Damping => -self.effects.damping_k * self.omega_est,
            ControlMode::Knob => {
                let step = self.effects.detent_step_rad.max(0.01);
                let detent = libm::roundf(self.theta_est / step) * step;
                self.effects.detent_k * shortest_angle_error(detent, self.theta_est)
                    - self.effects.damping_k * self.omega_est
            }
            ControlMode::Weightless => {
                let friction = self.effects.weightless_friction;
                -friction * self.omega_est.signum() - 0.02 * self.omega_est
            }
        };

        // Smooth transitions when mode changes over RS485.
        let blended = self.last_iq_ref + (raw - self.last_iq_ref) * clamp(dt_s * 20.0, 0.0, 1.0);
        self.last_iq_ref = clamp(blended, -self.limits.iq_max, self.limits.iq_max);
        self.last_iq_ref
    }
}

fn clarke(i: PhaseCurrents) -> (f32, f32) {
    let i_alpha = i.ia;
    let i_beta = (i.ia + 2.0 * i.ib) * INV_SQRT_3;
    (i_alpha, i_beta)
}

fn svpwm(v_alpha: f32, v_beta: f32, vbus: f32) -> DutyCycles {
    let va = v_alpha;
    let vb = -0.5 * v_alpha + 0.5 * SQRT_3 * v_beta;
    let vc = -0.5 * v_alpha - 0.5 * SQRT_3 * v_beta;

    let vmax = va.max(vb).max(vc);
    let vmin = va.min(vb).min(vc);
    let v_offset = -0.5 * (vmax + vmin);

    let da = clamp(0.5 + (va + v_offset) / vbus, 0.0, 1.0);
    let db = clamp(0.5 + (vb + v_offset) / vbus, 0.0, 1.0);
    let dc = clamp(0.5 + (vc + v_offset) / vbus, 0.0, 1.0);

    DutyCycles {
        a: da,
        b: db,
        c: dc,
    }
}

fn clamp(v: f32, min_v: f32, max_v: f32) -> f32 {
    if v < min_v {
        min_v
    } else if v > max_v {
        max_v
    } else {
        v
    }
}

fn wrap_angle(a: f32) -> f32 {
    let mut out = a;
    while out >= PI {
        out -= TWO_PI;
    }
    while out < -PI {
        out += TWO_PI;
    }
    out
}

fn shortest_angle_error(target: f32, current: f32) -> f32 {
    wrap_angle(target - current)
}

impl Default for Telemetry {
    fn default() -> Self {
        Self {
            mode: ControlMode::Weightless,
            state: DriveState::Boot,
            theta: 0.0,
            omega: 0.0,
            id: 0.0,
            iq: 0.0,
            iq_ref: 0.0,
            vbus: 0.0,
            temp_c: 0.0,
            fault_bits: 0,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn phase_mapping_is_stable() {
        let mut ctrl = FocController::default();
        ctrl.state = DriveState::Ready;
        let i = ctrl.decode_phase_currents(2100, 2200, 2300);
        assert!(i.ia < i.ib && i.ib < i.ic);
    }

    #[test]
    fn svpwm_outputs_are_bounded() {
        let d = svpwm(1.2, -0.7, 12.0);
        assert!((0.0..=1.0).contains(&d.a));
        assert!((0.0..=1.0).contains(&d.b));
        assert!((0.0..=1.0).contains(&d.c));
    }
}
