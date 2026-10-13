//! A generic rate-limited actuator position with a jam fault, shared by the
//! load compressor's inlet guide vanes and surge control valve
//! (`load_compressor.rs`) and the inlet door (`inlet_door.rs`), rather than
//! duplicating the same rate-limit-plus-jam logic three times.
//!
//! Every actuated position in this directory moves toward its commanded
//! value at a bounded rate (a real actuator cannot jump instantly), and a
//! jam/seizure fault (0 healthy .. 1 fully seized) scales that rate toward
//! zero: at 1.0 the actuator is frozen wherever it happens to be when the
//! fault reaches full severity -- the physical definition of "stuck", not a
//! separate scripted "stuck open/closed" state. Which position it freezes
//! at falls naturally out of whatever the commanded position was doing at
//! the moment the fault engaged.

#[derive(Clone, Copy, Debug)]
pub struct Actuator {
    position_frac: f64,
    full_travel_rate_per_s: f64,
}

impl Actuator {
    pub fn new(initial_frac: f64, full_travel_rate_per_s: f64) -> Self {
        Self {
            position_frac: initial_frac.clamp(0.0, 1.0),
            full_travel_rate_per_s: full_travel_rate_per_s.max(1e-6),
        }
    }

    pub fn position_frac(&self) -> f64 {
        self.position_frac
    }

    /// Moves toward `commanded_frac` at this actuator's rate, scaled down by
    /// `jam_frac` (0 healthy .. 1 fully seized). Returns the new position.
    pub fn step(&mut self, commanded_frac: f64, jam_frac: f64, dt_s: f64) -> f64 {
        let rate = self.full_travel_rate_per_s * (1.0 - jam_frac.clamp(0.0, 1.0));
        let target = commanded_frac.clamp(0.0, 1.0);
        let max_delta = rate * dt_s.max(0.0);
        let delta = (target - self.position_frac).clamp(-max_delta, max_delta);
        self.position_frac = (self.position_frac + delta).clamp(0.0, 1.0);
        self.position_frac
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_healthy_actuator_reaches_its_command_given_enough_time() {
        let mut a = Actuator::new(0.0, 0.5);
        for _ in 0..10 {
            a.step(1.0, 0.0, 0.5);
        }
        assert!((a.position_frac() - 1.0).abs() < 1e-6);
    }

    #[test]
    fn it_never_moves_faster_than_its_rate_limit() {
        let mut a = Actuator::new(0.0, 0.5);
        let p = a.step(1.0, 0.0, 1.0);
        assert!((p - 0.5).abs() < 1e-9, "{p}");
    }

    #[test]
    fn a_fully_jammed_actuator_never_moves() {
        let mut a = Actuator::new(0.3, 0.5);
        for _ in 0..20 {
            a.step(1.0, 1.0, 1.0);
        }
        assert!((a.position_frac() - 0.3).abs() < 1e-9);
    }

    #[test]
    fn a_partially_jammed_actuator_moves_slower_but_still_moves() {
        let mut healthy = Actuator::new(0.0, 0.5);
        let mut stiff = Actuator::new(0.0, 0.5);
        healthy.step(1.0, 0.0, 1.0);
        stiff.step(1.0, 0.6, 1.0);
        assert!(stiff.position_frac() > 0.0);
        assert!(stiff.position_frac() < healthy.position_frac());
    }

    #[test]
    fn zero_dt_never_moves_and_never_produces_nan() {
        let mut a = Actuator::new(0.2, 0.5);
        let p = a.step(1.0, 0.0, 0.0);
        assert!((p - 0.2).abs() < 1e-12);
        assert!(p.is_finite());
    }
}
