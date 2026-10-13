//! Synthetic angle-of-sideslip: the A380, like the rest of the Airbus FBW
//! family, has **no dedicated sideslip vane** -- sideslip is a *derived*
//! quantity the flight control computers estimate from other sensors
//! (lateral acceleration, yaw rate, roll/pitch attitude, airspeed), not a
//! physical part with its own failure modes. That is exactly why this
//! module has no `Faults` struct and is not registered as a component or
//! failure in `registry.rs`: `docs/deep/BRIEF.md`'s own rule is "a failure
//! only if it changes a modelled output", and there is no physical sensor
//! here to fail -- any inaccuracy in the estimate below is inherited
//! entirely from whichever real sensors feed it (the accelerometer/gyro
//! bias and noise models already exist in `src/physics/adirs.rs`, which
//! this directory does not own or duplicate).
//!
//! This module exists to show *how* that derivation works and that its
//! errors are inherited, not invented: the standard, textbook small-angle
//! body-axis sideslip kinematic relation (found in any flight-dynamics
//! text covering lateral-directional equations of motion, e.g. Stevens &
//! Lewis, *Aircraft Control and Simulation*, or McRuer, Ashkenas & Graham,
//! *Aircraft Dynamics and Automatic Control*) is:
//!
//! `beta_dot = a_y/V - r + (g/V)*sin(phi)*cos(theta)`
//!
//! where `a_y` is body-axis lateral acceleration, `r` yaw rate, `V` true
//! airspeed, and `phi`/`theta` roll/pitch. This is GENERIC textbook
//! flight-dynamics physics, not the A380's actual (proprietary,
//! unpublished) flight-control-law sideslip estimation algorithm -- it is
//! used here only to demonstrate the *mechanism* (and that a coordinated
//! turn, `r ~= (g/V)*tan(phi)`, correctly gives `beta_dot ~= 0`).
//!
//! A pure open-loop integration of this rate drifts without bound from
//! any uncorrected input bias, exactly like the free-inertial position
//! integration in `src/physics/adirs.rs` -- real synthetic estimators
//! counter this with a slow "washout" back toward zero (a standard
//! control-engineering technique for rejecting long-term integrator
//! drift while passing genuine, sustained sideslip through more slowly
//! than a hard reset would), which this module applies with a GENERIC,
//! minutes-scale time constant.

const STANDARD_GRAVITY_MS2: f64 = 9.806_65;
/// Washout time constant, seconds: slow enough not to reject a real,
/// sustained sideslip (e.g. an engine-out asymmetric-thrust condition)
/// within the timescale a pilot would notice it, fast enough to bound
/// integrator drift over a flight. GENERIC.
const WASHOUT_TAU_S: f64 = 300.0;
/// Below this true airspeed, `1/V` blows up (ground/very low speed); the
/// estimate is clamped to airspeeds above this floor.
const MIN_TAS_MS: f64 = 5.0;

#[derive(Clone, Copy, Debug, Default)]
pub struct SideslipEstimator {
    beta_deg: f64,
}

impl SideslipEstimator {
    pub fn new() -> Self {
        Self { beta_deg: 0.0 }
    }

    /// `lateral_accel_ms2`: body-axis lateral (Y) acceleration. `yaw_rate_rad_s`:
    /// body-axis yaw rate. `roll_deg`, `pitch_deg`: attitude. `tas_ms`: true
    /// airspeed. Returns the current synthetic sideslip estimate, degrees
    /// (positive = nose points left of the velocity vector, the standard
    /// aerodynamic sign convention).
    pub fn step(&mut self, lateral_accel_ms2: f64, yaw_rate_rad_s: f64, roll_deg: f64, pitch_deg: f64, tas_ms: f64, dt_s: f64) -> f64 {
        let dt = dt_s.max(0.0);
        let v = tas_ms.max(MIN_TAS_MS);
        let (roll_rad, pitch_rad) = (roll_deg.to_radians(), pitch_deg.to_radians());
        let beta_dot_rad_s = lateral_accel_ms2 / v - yaw_rate_rad_s + (STANDARD_GRAVITY_MS2 / v) * roll_rad.sin() * pitch_rad.cos();
        self.beta_deg += beta_dot_rad_s.to_degrees() * dt;
        // Washout: exact exponential decay toward zero, dt-scaled (see
        // `static_port.rs`'s fixed-blend-fraction bug and fix for why this
        // matters).
        self.beta_deg *= (-dt / WASHOUT_TAU_S).exp();
        self.beta_deg
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn zero_inputs_give_zero_sideslip() {
        let mut e = SideslipEstimator::new();
        let beta = e.step(0.0, 0.0, 0.0, 0.0, 150.0, 1.0);
        assert_eq!(beta, 0.0);
    }

    #[test]
    fn a_turn_with_zero_lateral_accel_and_matching_yaw_rate_gives_zero_sideslip_rate() {
        // This model's own equilibrium (beta_dot = 0 with a_y = 0, theta =
        // 0): r = (g/V)*sin(phi) -- the self-consistent "coordinated turn"
        // condition for *this* kinematic relation (a full turn-geometry
        // derivation of r independently gives (g/V)*tan(phi); the two
        // agree only in the small-bank-angle limit, since this equation is
        // the standard small-angle form -- see module docs). Feeding in
        // this equation's own equilibrium checks the equation is wired up
        // correctly, not a claim that real coordinated-turn yaw rate uses
        // sin rather than tan.
        let mut e = SideslipEstimator::new();
        let v = 150.0;
        let roll_deg: f64 = 10.0;
        let r = (STANDARD_GRAVITY_MS2 / v) * roll_deg.to_radians().sin();
        let mut beta = 0.0;
        for _ in 0..50 {
            beta = e.step(0.0, r, roll_deg, 0.0, v, 0.1);
        }
        assert!(beta.abs() < 1e-6, "{beta}");
    }

    #[test]
    fn a_sustained_lateral_acceleration_builds_up_sideslip_then_it_washes_out() {
        let mut e = SideslipEstimator::new();
        let mut beta = 0.0;
        // A short burst of uncoordinated lateral acceleration (e.g. a gust
        // or asymmetric thrust step), then it stops.
        for _ in 0..20 {
            beta = e.step(2.0, 0.0, 0.0, 0.0, 150.0, 0.1);
        }
        assert!(beta.abs() > 0.01, "expected a measurable sideslip build-up, got {beta}");
        let built_up = beta;
        // Inputs return to zero: the washout should decay the estimate
        // back toward zero over the long term.
        for _ in 0..30_000 {
            beta = e.step(0.0, 0.0, 0.0, 0.0, 150.0, 0.1);
        }
        assert!(beta.abs() < built_up.abs() * 0.1, "built up {built_up}, after washout {beta}");
    }

    #[test]
    fn no_nan_at_zero_dt_or_zero_airspeed() {
        let mut e = SideslipEstimator::new();
        let beta = e.step(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        assert!(beta.is_finite());
    }
}
