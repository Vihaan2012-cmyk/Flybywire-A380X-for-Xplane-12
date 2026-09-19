//! Spool dynamics: shaft torque balance and inertia, integrated with fixed
//! sub-steps so the result never depends on the caller's frame rate and
//! never explodes at a large or paused `dt`.

/// A small positive speed floor (rad/s) used only to keep `power/omega`
/// finite at literally zero rotation; below it, torque-based (not
/// power-based) sources such as the starter are what actually turns the
/// spool (see `starter.rs`).
const OMEGA_FLOOR_RAD_S: f64 = 0.5;

/// One rigid rotor: an inertia and its current speed.
#[derive(Clone, Copy, Debug)]
pub struct Spool {
    pub rpm: f64,
    pub inertia_kg_m2: f64,
}

impl Spool {
    pub fn new(inertia_kg_m2: f64) -> Self {
        Self { rpm: 0.0, inertia_kg_m2 }
    }

    pub fn omega_rad_s(&self) -> f64 {
        self.rpm * std::f64::consts::PI / 30.0
    }

    /// Converts a power (turbine positive, compressor/accessory negative)
    /// to a torque at the current speed, floored so it stays finite near
    /// zero rotation instead of blowing up.
    pub fn torque_from_power(&self, power_w: f64) -> f64 {
        power_w / self.omega_rad_s().max(OMEGA_FLOOR_RAD_S)
    }

    /// Integrates one net torque (N·m) over `dt` seconds using fixed
    /// sub-steps of at most `max_substep_s`, capped at `max_substeps` so a
    /// paused sim or a huge frame time cannot take an unbounded number of
    /// steps: cost stays bounded, at the cost of the spool lagging behind
    /// real time on an extreme frame spike rather than ever going unstable.
    pub fn integrate(&mut self, net_torque_n_m: f64, dt_s: f64, max_substep_s: f64, max_substeps: u32) {
        if !dt_s.is_finite() || dt_s <= 0.0 || !net_torque_n_m.is_finite() {
            return;
        }
        let substeps = ((dt_s / max_substep_s).ceil() as u32).clamp(1, max_substeps);
        let sub_dt = dt_s / substeps as f64;
        for _ in 0..substeps {
            let alpha_rad_s2 = net_torque_n_m / self.inertia_kg_m2; // angular acceleration
            let d_rpm = alpha_rad_s2 * (30.0 / std::f64::consts::PI) * sub_dt;
            self.rpm = (self.rpm + d_rpm).max(0.0);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn positive_torque_accelerates_the_spool() {
        let mut s = Spool::new(100.0);
        s.integrate(500.0, 1.0, 0.01, 200);
        assert!(s.rpm > 0.0);
    }

    #[test]
    fn negative_torque_never_takes_speed_below_zero() {
        let mut s = Spool { rpm: 5.0, inertia_kg_m2: 100.0 };
        s.integrate(-1e9, 1.0, 0.01, 200);
        assert_eq!(s.rpm, 0.0);
    }

    #[test]
    fn a_huge_or_paused_frame_time_never_produces_nan_or_infinity() {
        let mut s = Spool::new(50.0);
        s.integrate(10_000.0, 5000.0, 0.005, 50);
        assert!(s.rpm.is_finite());
        let mut zero_dt = Spool::new(50.0);
        zero_dt.integrate(10_000.0, 0.0, 0.005, 50);
        assert_eq!(zero_dt.rpm, 0.0);
    }

    #[test]
    fn substeps_do_not_change_the_answer_much_versus_one_big_step() {
        // Sanity check that sub-stepping converges rather than drifting.
        let mut fine = Spool::new(100.0);
        fine.integrate(300.0, 2.0, 0.001, 5000);
        let mut coarse = Spool::new(100.0);
        coarse.integrate(300.0, 2.0, 0.05, 40);
        assert!((fine.rpm - coarse.rpm).abs() / fine.rpm.max(1.0) < 0.02);
    }
}
