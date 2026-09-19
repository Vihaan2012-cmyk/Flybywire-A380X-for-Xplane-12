//! A generic rate-limited control-surface actuator, shared by every flight
//! control surface in `aerodynamics.rs` (elevator, aileron, rudder,
//! spoiler), with the three actuator failure modes a flight-controls agent
//! needs somewhere real to act on, each expressed the same way every other
//! model in this project expresses a fault -- a fraction `0.0 = healthy ..
//! 1.0 = fully failed` (`registry.rs` registers one `FailureDef` per mode,
//! per surface):
//!
//! - **jam** (`jam_fraction`): the mechanism progressively binds up,
//!   scaling the achievable rate down to zero; at `1.0` the surface is
//!   fully seized and freezes wherever it happens to be (not at some
//!   externally chosen position -- that is what a real jam does: it stops
//!   the surface at *its* position when the seizure completes, not at a
//!   commanded one).
//! - **runaway** (`runaway_rate_rad_s`): a failed control valve/amplifier
//!   drives the surface at this signed rate regardless of command, until
//!   it hits its travel limit (registered as two failures per surface,
//!   toward each limit, since "elevator runs away nose up" and "...nose
//!   down" are distinct physical failure modes with different
//!   consequences -- see `registry.rs`).
//! - **float** (`float_fraction`): partial-to-total loss of actuator
//!   authority (e.g. hydraulic pressure loss on an unpowered-when-idle
//!   surface); the commanded target blends toward zero deflection as the
//!   surface aerodynamically streamlines, at `1.0` the actuator contributes
//!   nothing and the surface fully floats.
//!
//! With every fault at its default (healthy), this is a simple rate- and
//! position-limited first-order actuator: real hydraulic/electric flight
//! control actuators are rate-limited (a fixed deg/s slew, not an
//! instantaneous position), so a step command still takes real time to
//! reach, exactly like the physical A380 EHA/EBHA actuators the FBW
//! systems this project ports use.

#[derive(Clone, Copy, Debug, Default)]
pub struct SurfaceFault {
    /// 0 = free to move at full rate; 1 = fully seized (frozen wherever it
    /// is). Values between linearly reduce the achievable rate.
    pub jam_fraction: f64,
    /// Signed rate, rad/s: nonzero drives the surface at this rate
    /// regardless of command (a runaway), clamped to the travel limit.
    /// Zero = no runaway.
    pub runaway_rate_rad_s: f64,
    /// 0 = full actuator authority; 1 = fully floating (no authority,
    /// commanded target fully blended to zero deflection). Values between
    /// linearly blend the commanded target toward zero.
    pub float_fraction: f64,
}

#[derive(Clone, Copy, Debug)]
pub struct Actuator {
    position_rad: f64,
}

impl Actuator {
    pub fn new() -> Self {
        Self { position_rad: 0.0 }
    }

    pub fn position_rad(&self) -> f64 {
        self.position_rad
    }

    /// Advances the actuator one tick toward `commanded_rad`, subject to
    /// `rate_limit_rad_s` and `limit_rad` (symmetric travel limit), per the
    /// module doc's fault handling. Returns the new position.
    pub fn step(&mut self, commanded_rad: f64, fault: &SurfaceFault, rate_limit_rad_s: f64, limit_rad: f64, dt_s: f64) -> f64 {
        let dt = dt_s.max(0.0);
        let float = fault.float_fraction.clamp(0.0, 1.0);
        let blended_target = commanded_rad * (1.0 - float);
        let runaway = fault.runaway_rate_rad_s;
        let (target, base_rate) = if runaway.abs() > 1e-9 {
            // One tick's worth of travel beyond the limit in the runaway's
            // direction is always "further than the limit", so the rate
            // clamp below is what actually stops it there.
            (self.position_rad + runaway.signum() * (limit_rad.max(1e-9) * 2.0), runaway.abs())
        } else {
            (blended_target, rate_limit_rad_s.max(0.0))
        };
        let jam = fault.jam_fraction.clamp(0.0, 1.0);
        let rate = base_rate * (1.0 - jam);
        let max_step = rate * dt;
        let delta = (target - self.position_rad).clamp(-max_step, max_step);
        self.position_rad = (self.position_rad + delta).clamp(-limit_rad, limit_rad);
        self.position_rad
    }
}

impl Default for Actuator {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_healthy_actuator_slews_toward_command_and_settles_exactly_on_it() {
        let mut a = Actuator::new();
        let fault = SurfaceFault::default();
        for _ in 0..1000 {
            a.step(0.3, &fault, 0.5, 0.6, 0.05);
        }
        assert!((a.position_rad() - 0.3).abs() < 1e-6);
    }

    #[test]
    fn rate_limit_is_respected_on_a_step_command() {
        let mut a = Actuator::new();
        let fault = SurfaceFault::default();
        let p = a.step(1.0, &fault, 0.2, 0.6, 0.05); // 0.2 rad/s * 0.05 s = 0.01 rad max
        assert!((p - 0.01).abs() < 1e-9);
    }

    #[test]
    fn a_full_jam_freezes_it_wherever_it_is_regardless_of_command() {
        let mut a = Actuator::new();
        a.step(0.15, &SurfaceFault::default(), 1.0, 0.6, 1.0); // settle at 0.15 first
        let frozen_at = a.position_rad();
        let fault = SurfaceFault { jam_fraction: 1.0, ..Default::default() };
        for _ in 0..200 {
            a.step(-0.5, &fault, 0.5, 0.6, 0.05);
        }
        assert!((a.position_rad() - frozen_at).abs() < 1e-9);
    }

    #[test]
    fn a_partial_jam_slows_but_does_not_stop_it() {
        let mut a = Actuator::new();
        let fault = SurfaceFault { jam_fraction: 0.9, ..Default::default() };
        let healthy_step = a.step(1.0, &SurfaceFault::default(), 1.0, 1.0, 0.05);
        let mut b = Actuator::new();
        let jammed_step = b.step(1.0, &fault, 1.0, 1.0, 0.05);
        assert!(jammed_step > 0.0 && jammed_step < healthy_step);
    }

    #[test]
    fn a_runaway_drives_to_the_travel_limit_ignoring_command() {
        let mut a = Actuator::new();
        let fault = SurfaceFault { runaway_rate_rad_s: -0.3, ..Default::default() };
        for _ in 0..500 {
            a.step(0.5, &fault, 0.5, 0.6, 0.05); // command asks nose-up; runaway drives the other way
        }
        assert!((a.position_rad() + 0.6).abs() < 1e-6, "should hardover to -limit: {}", a.position_rad());
    }

    #[test]
    fn a_fully_floating_surface_relaxes_to_zero_even_with_a_nonzero_command() {
        let mut a = Actuator::new();
        a.step(0.4, &SurfaceFault::default(), 1.0, 0.6, 0.05);
        let mut floated = a;
        let float = SurfaceFault { float_fraction: 1.0, ..Default::default() };
        for _ in 0..200 {
            floated.step(0.4, &float, 1.0, 0.6, 0.05);
        }
        assert!(floated.position_rad().abs() < 1e-6);
    }

    #[test]
    fn zero_dt_never_moves_or_produces_nan() {
        let mut a = Actuator::new();
        let p = a.step(0.4, &SurfaceFault::default(), 1.0, 0.6, 0.0);
        assert_eq!(p, 0.0);
        assert!(p.is_finite());
    }
}
