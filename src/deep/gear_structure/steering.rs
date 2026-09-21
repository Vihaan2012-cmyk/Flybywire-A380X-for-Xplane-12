//! Nosewheel and body-gear (rear-axle) steering, with a shimmy damper.
//!
//! # Steering actuator
//! A rate-limited hydraulic position servo tracking a commanded tiller/pedal
//! angle (`NOMINAL_STEERING_RATE_DEG_S`, GENERIC, order-of-magnitude for a
//! large transport's nosewheel steering slew rate; no A380-specific figure
//! is public). The A380's body gear bogies steer their rearmost axle a
//! smaller amount, opposing the nose angle at low speed to reduce tyre
//! scrub in a tight turn and phasing out toward zero at higher speed for
//! directional stability -- a well documented general design concept on
//! multi-bogie large aircraft (747/A340/A380-class main gear), not a cited
//! A380 schedule; `body_steering_angle_deg` implements it as a GENERIC,
//! documented schedule.
//!
//! # Shimmy
//! A steerable wheel is a rotational mass-spring-damper about its
//! steering/caster axis; a torque proportional to groundspeed that opposes
//! the mechanical damper (the classic destabilising mechanism: tyre
//! cornering-force lag interacting with caster geometry, standard published
//! shimmy theory, e.g. NASA/FAA shimmy analyses -- not A380-specific data)
//! reduces the net effective damping as speed rises:
//!
//!   I*delta'' + c_net(v)*delta' + K*delta = disturbance
//!   c_net(v) = c_mechanical(fault) - DESTAB_GAIN*v
//!
//! `c_mechanical` falls from a healthy value (GENERIC, sized so the
//! resulting critical speed -- where `c_net` reaches zero -- sits far above
//! any realistic ground speed) toward a small residual structural value as
//! `shimmy_damper_fail` rises (GENERIC, sized so a fully failed damper's
//! critical speed sits in the realistic taxi/rollout range): below the
//! critical speed the system is a damped oscillator settling to a small
//! offset from a constant disturbance torque (GENERIC, representing runway
//! surface roughness); above it, the same linear system is unstable and the
//! perturbation grows -- shimmy, an emergent consequence of the equation,
//! not a scripted symptom. All of `I`, `K`, `c_mechanical`, `DESTAB_GAIN`
//! and the disturbance magnitude are GENERIC, chosen only to place the
//! healthy/failed critical speeds in the right qualitative regimes (checked
//! by this file's own tests), since no public A380 nosewheel torsional data
//! exists.

/// Steering actuator rate at full hydraulic effort, deg/s (GENERIC).
const NOMINAL_STEERING_RATE_DEG_S: f64 = 30.0;
/// Nosewheel maximum commanded angle, deg (GENERIC, order-of-magnitude
/// large-transport tiller range).
const MAX_NOSE_ANGLE_DEG: f64 = 70.0;
/// Body-gear rear-axle maximum steering angle, deg (GENERIC: a small
/// scrub-reduction angle, not primary steering).
const MAX_BODY_ANGLE_DEG: f64 = 15.0;
/// Body-axle angle as a fraction of the nosewheel's own commanded angle at
/// zero groundspeed, opposing it (GENERIC).
const BODY_GAIN: f64 = -0.30;
/// Groundspeed at which the body-axle assist has fully phased out to zero
/// (GENERIC).
const BODY_PHASE_OUT_SPEED_MS: f64 = 15.0;

/// Torsional inertia about the steering axis, kg*m^2 (GENERIC).
const I_EFF: f64 = 40.0;
/// Torsional (centering) stiffness, N*m/rad (GENERIC).
const K_EFF: f64 = 60_000.0;
/// Healthy mechanical shimmy-damper coefficient, N*m*s/rad (GENERIC, sized
/// so the healthy critical speed is far above any realistic ground speed --
/// see `critical_speed_ms`'s test).
const C_MECHANICAL_HEALTHY: f64 = 20_000.0;
/// Residual structural damping with the damper fully failed, N*m*s/rad
/// (GENERIC: never literally zero friction).
const C_RESIDUAL_FAILED: f64 = 500.0;
/// Destabilising gain, N*m*s/rad per m/s of groundspeed (GENERIC).
const DESTAB_GAIN: f64 = 150.0;
/// Small constant seed disturbance torque, N*m (GENERIC: runway surface
/// roughness/joint texture).
const DISTURBANCE_NM: f64 = 50.0;
/// Perturbation magnitude beyond which nonlinear mechanical limits this
/// module does not otherwise model (stops, tyre scrub) are assumed to cap
/// further growth (GENERIC).
const MAX_PERTURBATION_DEG: f64 = 8.0;

const SUB_STEP_S: f64 = 0.005;

fn to_deg(rad: f64) -> f64 {
    rad * 180.0 / std::f64::consts::PI
}
fn to_rad(deg: f64) -> f64 {
    deg * std::f64::consts::PI / 180.0
}

/// The commanded (low-speed / rear-axle) steering schedule for the body
/// gear's rear axle, as a fraction-of-nose-angle assist phasing out with
/// speed (see module doc).
pub fn body_steering_angle_deg(nose_commanded_angle_deg: f64, groundspeed_ms: f64) -> f64 {
    let phase = (1.0 - groundspeed_ms.max(0.0) / BODY_PHASE_OUT_SPEED_MS).clamp(0.0, 1.0);
    (BODY_GAIN * nose_commanded_angle_deg * phase).clamp(-MAX_BODY_ANGLE_DEG, MAX_BODY_ANGLE_DEG)
}

/// Faults this steering system carries, each a fraction 0 (healthy) .. 1
/// (fully failed).
#[derive(Clone, Copy, Debug, Default)]
pub struct SteeringFaults {
    /// Shimmy damper degraded/failed.
    pub shimmy_damper_fail: f64,
    /// Steering actuator internal leak: reduces slew rate.
    pub actuator_leak: f64,
}

#[derive(Clone, Copy, Debug)]
pub struct SteeringInputs {
    /// Commanded angle, deg (already scheduled/limited by whoever drives
    /// the tiller or pedals; this module does not itself impose the
    /// high-speed pedal-steering-only limits some types apply).
    pub commanded_angle_deg: f64,
    pub groundspeed_ms: f64,
    pub dt_s: f64,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct SteeringOutputs {
    /// True wheel angle including the shimmy perturbation, deg.
    pub angle_deg: f64,
    /// The commanded (base) angle actually tracked so far, deg.
    pub base_angle_deg: f64,
    /// The shimmy perturbation alone, deg.
    pub shimmy_deg: f64,
    /// The net damping at the current speed/fault state has gone unstable
    /// (a genuine, ongoing shimmy tendency, not just noise).
    pub shimmy_unstable: bool,
}

pub struct SteeringActuator {
    max_angle_deg: f64,
    base_angle_deg: f64,
    perturbation_deg: f64,
    perturbation_rate: f64,
}

impl SteeringActuator {
    pub fn new_nose() -> Self {
        Self { max_angle_deg: MAX_NOSE_ANGLE_DEG, base_angle_deg: 0.0, perturbation_deg: 0.0, perturbation_rate: 0.0 }
    }

    pub fn new_body() -> Self {
        Self { max_angle_deg: MAX_BODY_ANGLE_DEG, base_angle_deg: 0.0, perturbation_deg: 0.0, perturbation_rate: 0.0 }
    }

    /// The groundspeed at which the net effective damping reaches zero for
    /// the given fault severity: below it the shimmy mode is damped, above
    /// it the same linear equation is unstable.
    pub fn critical_speed_ms(shimmy_damper_fail: f64) -> f64 {
        let c_mech = C_RESIDUAL_FAILED + (C_MECHANICAL_HEALTHY - C_RESIDUAL_FAILED) * (1.0 - shimmy_damper_fail.clamp(0.0, 1.0));
        c_mech / DESTAB_GAIN
    }

    pub fn step(&mut self, inputs: &SteeringInputs, faults: &SteeringFaults) -> SteeringOutputs {
        let dt = inputs.dt_s.max(0.0);
        let target = inputs.commanded_angle_deg.clamp(-self.max_angle_deg, self.max_angle_deg);
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
            // The damping term is taken implicitly. Explicitly, the rate is
            // multiplied by (1 - c_net*h/I_EFF) each sub-step, which for the
            // healthy damper at rest is -1.5: the perturbation grew without
            // bound in a few ticks, overflowed, and the NaN stayed in the
            // state for the rest of the session (seen live as
            // NW_STEER_ANGLE_DEG = NaN every frame on a parked aircraft).
            // Dividing by (1 + c_net*h/I_EFF) instead is the exact solution
            // of the linear damping over the sub-step and is stable for any
            // positive damping and any h; the stiffness and disturbance
            // terms are unchanged. Negative c_net (an unstable, failed
            // damper above its critical speed) still grows, as it should,
            // and the clamp below still caps it.
            let forcing = (DISTURBANCE_NM - K_EFF * delta) / I_EFF;
            delta_dot = (delta_dot + forcing * h) / (1.0 + c_net * h / I_EFF).max(1e-6);
            delta += delta_dot * h;
            remaining -= h;
        }
        // A non-finite state cannot recover on its own and would poison
        // every frame after it; start the perturbation over instead.
        if !delta.is_finite() || !delta_dot.is_finite() {
            delta = 0.0;
            delta_dot = 0.0;
        }
        self.perturbation_deg = to_deg(delta).clamp(-MAX_PERTURBATION_DEG, MAX_PERTURBATION_DEG);
        self.perturbation_rate = delta_dot;

        SteeringOutputs {
            angle_deg: (self.base_angle_deg + self.perturbation_deg).clamp(-self.max_angle_deg - MAX_PERTURBATION_DEG, self.max_angle_deg + MAX_PERTURBATION_DEG),
            base_angle_deg: self.base_angle_deg,
            shimmy_deg: self.perturbation_deg,
            shimmy_unstable: c_net < 0.0,
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
        // The live defect: parked, healthy damper, the explicit damping
        // step diverged (multiplier -1.5 per sub-step) and the angle was NaN
        // from the first seconds of the session onward.
        let mut s = SteeringActuator::new_nose();
        let inputs = SteeringInputs { commanded_angle_deg: 0.0, groundspeed_ms: 0.0, dt_s: 1.0 / 36.0 };
        let mut out = SteeringOutputs::default();
        for _ in 0..3_600 {
            out = s.step(&inputs, &healthy());
            assert!(out.angle_deg.is_finite(), "steering angle went non-finite: {}", out.angle_deg);
        }
        assert!(!out.shimmy_unstable);
        // 50 N*m against 60 kN*m/rad is a 0.048 degree static offset.
        assert!(out.shimmy_deg.abs() < 0.1, "at rest the perturbation must settle near its static offset, got {}", out.shimmy_deg);
    }

    #[test]
    fn a_healthy_damper_settles_and_stays_bounded_at_high_speed() {
        let mut s = SteeringActuator::new_nose();
        let inputs = SteeringInputs { commanded_angle_deg: 0.0, groundspeed_ms: 90.0, dt_s: 0.01 };
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
        let faults = SteeringFaults { shimmy_damper_fail: 1.0, actuator_leak: 0.0 };
        // 4 m/s is just above `critical_speed_ms(1.0)` (3.33 m/s): a mild,
        // gradual instability, not an instant slam into the perturbation
        // cap, so "early" and "late" amplitudes stay distinguishable.
        let inputs = SteeringInputs { commanded_angle_deg: 0.0, groundspeed_ms: 4.0, dt_s: 0.01 };
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
        let inputs = SteeringInputs { commanded_angle_deg: 45.0, groundspeed_ms: 0.0, dt_s: 0.1 };
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
        let inputs = SteeringInputs { commanded_angle_deg: 0.0, groundspeed_ms: 0.0, dt_s: 0.0 };
        let out = s.step(&inputs, &healthy());
        assert!(out.angle_deg.is_finite());
        assert!(!out.shimmy_deg.is_nan());
    }
}
