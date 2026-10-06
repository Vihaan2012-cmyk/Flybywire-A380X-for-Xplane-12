const NOMINAL_STEERING_RATE_DEG_S: f64 = 30.0;
const MAX_NOSE_ANGLE_DEG: f64 = 70.0;
const MAX_BODY_ANGLE_DEG: f64 = 15.0;
const BODY_GAIN: f64 = -0.30;
const BODY_PHASE_OUT_SPEED_MS: f64 = 15.0;

const I_EFF: f64 = 40.0;
const K_EFF: f64 = 60_000.0;
const C_MECHANICAL_HEALTHY: f64 = 20_000.0;
const C_RESIDUAL_FAILED: f64 = 500.0;
const DESTAB_GAIN: f64 = 150.0;
const DISTURBANCE_NM: f64 = 50.0;
const MAX_PERTURBATION_DEG: f64 = 8.0;

const SUB_STEP_S: f64 = 0.005;

fn to_deg(rad: f64) -> f64 {
    rad * 180.0 / std::f64::consts::PI
}
fn to_rad(deg: f64) -> f64 {
    deg * std::f64::consts::PI / 180.0
}

pub fn body_steering_angle_deg(nose_commanded_angle_deg: f64, groundspeed_ms: f64) -> f64 {
    let phase = (1.0 - groundspeed_ms.max(0.0) / BODY_PHASE_OUT_SPEED_MS).clamp(0.0, 1.0);
    (BODY_GAIN * nose_commanded_angle_deg * phase).clamp(-MAX_BODY_ANGLE_DEG, MAX_BODY_ANGLE_DEG)
}

pub(crate) const DISC_MECHANISM_FAIL_THRESHOLD: f64 = 0.5;
const OVERTRAVEL_FAIL_THRESHOLD: f64 = 0.5;

#[derive(Clone, Copy, Debug, Default)]
pub struct SteeringFaults {
    pub shimmy_damper_fail: f64,
    pub actuator_leak: f64,
    pub disc_mechanism_fail: f64,
    pub steer_overtravel_fail: f64,
}

#[derive(Clone, Copy, Debug)]
pub struct SteeringInputs {
    pub commanded_angle_deg: f64,
    pub groundspeed_ms: f64,
    pub dt_s: f64,
    pub disconnect_selected: bool,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct SteeringOutputs {
    pub angle_deg: f64,
    pub base_angle_deg: f64,
    pub shimmy_deg: f64,
    pub shimmy_unstable: bool,
    pub disconnected: bool,
}

pub struct SteeringActuator {
    max_angle_deg: f64,
    base_angle_deg: f64,
    perturbation_deg: f64,
    perturbation_rate: f64,
    disconnected: bool,
}

impl SteeringActuator {
    pub fn new_nose() -> Self {
        Self { max_angle_deg: MAX_NOSE_ANGLE_DEG, base_angle_deg: 0.0, perturbation_deg: 0.0, perturbation_rate: 0.0, disconnected: false }
    }

    pub fn new_body() -> Self {
        Self { max_angle_deg: MAX_BODY_ANGLE_DEG, base_angle_deg: 0.0, perturbation_deg: 0.0, perturbation_rate: 0.0, disconnected: false }
    }

    pub fn critical_speed_ms(shimmy_damper_fail: f64) -> f64 {
        let c_mech = C_RESIDUAL_FAILED + (C_MECHANICAL_HEALTHY - C_RESIDUAL_FAILED) * (1.0 - shimmy_damper_fail.clamp(0.0, 1.0));
        c_mech / DESTAB_GAIN
    }

    pub fn step(&mut self, inputs: &SteeringInputs, faults: &SteeringFaults) -> SteeringOutputs {
        let dt = inputs.dt_s.max(0.0);

        if faults.disc_mechanism_fail < DISC_MECHANISM_FAIL_THRESHOLD {
            self.disconnected = inputs.disconnect_selected;
        }

        let overtravel = faults.steer_overtravel_fail >= OVERTRAVEL_FAIL_THRESHOLD;
        let target = if overtravel { inputs.commanded_angle_deg } else { inputs.commanded_angle_deg.clamp(-self.max_angle_deg, self.max_angle_deg) };
        let rate = NOMINAL_STEERING_RATE_DEG_S * (1.0 - faults.actuator_leak.clamp(0.0, 1.0)).max(0.0);
        let diff = target - self.base_angle_deg;
        let step = diff.signum() * (rate * dt).min(diff.abs());
        self.base_angle_deg += step;

        let c_mech = C_RESIDUAL_FAILED + (C_MECHANICAL_HEALTHY - C_RESIDUAL_FAILED) * (1.0 - faults.shimmy_damper_fail.clamp(0.0, 1.0));
        let c_net = c_mech - DESTAB_GAIN * inputs.groundspeed_ms.max(0.0);

        let mut delta = to_rad(self.perturbation_deg);
        let mut delta_dot = self.perturbation_rate;
        let mut remaining = dt;
        while remaining > 1e-9 {
            let h = SUB_STEP_S.min(remaining);
            let forcing = (DISTURBANCE_NM - K_EFF * delta) / I_EFF;
            delta_dot = (delta_dot + forcing * h) / (1.0 + c_net * h / I_EFF).max(1e-6);
            delta += delta_dot * h;
            remaining -= h;
        }
        if !delta.is_finite() || !delta_dot.is_finite() {
            delta = 0.0;
            delta_dot = 0.0;
        }
        self.perturbation_deg = to_deg(delta).clamp(-MAX_PERTURBATION_DEG, MAX_PERTURBATION_DEG);
        self.perturbation_rate = delta_dot;

        let bound = self.max_angle_deg + MAX_PERTURBATION_DEG;
        let angle_deg = if overtravel { self.base_angle_deg + self.perturbation_deg } else { (self.base_angle_deg + self.perturbation_deg).clamp(-bound, bound) };

        SteeringOutputs {
            angle_deg,
            base_angle_deg: self.base_angle_deg,
            shimmy_deg: self.perturbation_deg,
            shimmy_unstable: c_net < 0.0,
            disconnected: self.disconnected,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn healthy() -> SteeringFaults {
        SteeringFaults::default()
    }

    #[test]
    fn a_healthy_damper_never_goes_unstable_at_realistic_ground_speeds() {
        let v_crit = SteeringActuator::critical_speed_ms(0.0);
        assert!(v_crit > 90.0, "healthy critical speed should be far above realistic taxi/rollout speeds: {v_crit} m/s");
    }

    #[test]
    fn a_fully_failed_damper_has_a_low_critical_speed() {
        let v_crit = SteeringActuator::critical_speed_ms(1.0);
        assert!(v_crit < 10.0, "a fully failed damper should go unstable at a realistic taxi speed: {v_crit} m/s");
    }

    #[test]
    fn a_healthy_damper_at_rest_stays_finite_and_settles() {
        let mut s = SteeringActuator::new_nose();
        let inputs = SteeringInputs { commanded_angle_deg: 0.0, groundspeed_ms: 0.0, dt_s: 1.0 / 36.0, disconnect_selected: false };
        let mut out = SteeringOutputs::default();
        for _ in 0..3_600 {
            out = s.step(&inputs, &healthy());
            assert!(out.angle_deg.is_finite(), "steering angle went non-finite: {}", out.angle_deg);
        }
        assert!(!out.shimmy_unstable);
        assert!(out.shimmy_deg.abs() < 0.1, "at rest the perturbation must settle near its static offset, got {}", out.shimmy_deg);
    }

    #[test]
    fn a_healthy_damper_settles_and_stays_bounded_at_high_speed() {
        let mut s = SteeringActuator::new_nose();
        let inputs = SteeringInputs { commanded_angle_deg: 0.0, groundspeed_ms: 90.0, dt_s: 0.01, disconnect_selected: false };
        let mut out = SteeringOutputs::default();
        for _ in 0..2_000 {
            out = s.step(&inputs, &healthy());
        }
        assert!(!out.shimmy_unstable);
        assert!(out.shimmy_deg.abs() < 1.0, "a healthy damper must settle to a small bounded offset, not grow: {}", out.shimmy_deg);
    }

    #[test]
    fn a_failed_damper_lets_the_perturbation_grow_at_a_speed_just_above_its_critical_speed() {
        let mut s = SteeringActuator::new_nose();
        let faults = SteeringFaults { shimmy_damper_fail: 1.0, ..SteeringFaults::default() };
        let inputs = SteeringInputs { commanded_angle_deg: 0.0, groundspeed_ms: 4.0, dt_s: 0.01, disconnect_selected: false };
        let mut early = 0.0;
        let mut out = SteeringOutputs::default();
        for i in 0..3_000 {
            out = s.step(&inputs, &faults);
            if i == 200 {
                early = out.shimmy_deg.abs();
            }
        }
        assert!(out.shimmy_unstable, "4 m/s is above the fully-failed critical speed and must be flagged unstable");
        assert!(out.shimmy_deg.abs() > early + 0.1, "the perturbation must have grown from its early value ({early}) to its later one ({})", out.shimmy_deg);
    }

    #[test]
    fn the_actuator_tracks_a_commanded_angle_within_its_travel_limit() {
        let mut s = SteeringActuator::new_body();
        let inputs = SteeringInputs { commanded_angle_deg: 45.0, groundspeed_ms: 0.0, dt_s: 0.1, disconnect_selected: false };
        let mut out = SteeringOutputs::default();
        for _ in 0..50 {
            out = s.step(&inputs, &healthy());
        }
        assert!((out.base_angle_deg - MAX_BODY_ANGLE_DEG).abs() < 0.5, "commanding past the body axle's travel limit must clamp to it: {}", out.base_angle_deg);
    }

    #[test]
    fn body_schedule_opposes_nose_angle_at_low_speed_and_phases_out_at_speed() {
        let low_speed = body_steering_angle_deg(30.0, 0.0);
        assert!(low_speed < 0.0, "the body axle must steer opposite the nose at low speed");
        let high_speed = body_steering_angle_deg(30.0, 40.0);
        assert_eq!(high_speed, 0.0, "the assist must have fully phased out well above BODY_PHASE_OUT_SPEED_MS");
    }

    #[test]
    fn numerically_safe_at_rest_and_dt_zero() {
        let mut s = SteeringActuator::new_nose();
        let inputs = SteeringInputs { commanded_angle_deg: 0.0, groundspeed_ms: 0.0, dt_s: 0.0, disconnect_selected: false };
        let out = s.step(&inputs, &healthy());
        assert!(out.angle_deg.is_finite());
        assert!(!out.shimmy_deg.is_nan());
    }

    #[test]
    fn steer_overtravel_fail_lets_a_commanded_angle_exceed_the_healthy_clamp() {
        let mut s = SteeringActuator::new_nose();
        let big_command = SteeringInputs { commanded_angle_deg: MAX_NOSE_ANGLE_DEG + 40.0, groundspeed_ms: 0.0, dt_s: 0.5, disconnect_selected: false };

        let mut healthy_actuator = SteeringActuator::new_nose();
        let mut healthy_out = SteeringOutputs::default();
        for _ in 0..50 {
            healthy_out = healthy_actuator.step(&big_command, &healthy());
        }
        assert!(healthy_out.angle_deg.abs() <= MAX_NOSE_ANGLE_DEG + MAX_PERTURBATION_DEG + 1e-6, "the healthy clamp must hold: {}", healthy_out.angle_deg);

        let faults = SteeringFaults { steer_overtravel_fail: 1.0, ..SteeringFaults::default() };
        let mut out = SteeringOutputs::default();
        for _ in 0..50 {
            out = s.step(&big_command, &faults);
        }
        assert!(out.angle_deg.abs() > MAX_NOSE_ANGLE_DEG + MAX_PERTURBATION_DEG, "an armed overtravel failure must let the angle exceed the healthy clamp, got {}", out.angle_deg);
    }

    #[test]
    fn disc_mechanism_fail_freezes_the_disconnect_state_against_a_new_selection() {
        let mut s = SteeringActuator::new_nose();
        let not_selected = SteeringInputs { commanded_angle_deg: 0.0, groundspeed_ms: 0.0, dt_s: 0.1, disconnect_selected: false };
        let out = s.step(&not_selected, &healthy());
        assert!(!out.disconnected, "must start connected");

        let selected = SteeringInputs { disconnect_selected: true, ..not_selected };
        let out = s.step(&selected, &healthy());
        assert!(out.disconnected, "a healthy mechanism must respond to a new selection");

        let faults = SteeringFaults { disc_mechanism_fail: 1.0, ..SteeringFaults::default() };
        let out = s.step(&not_selected, &faults);
        assert!(out.disconnected, "a jammed mechanism must stay stuck disconnected against a new deselection");
    }
}
