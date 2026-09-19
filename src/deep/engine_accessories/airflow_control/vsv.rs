//! IP compressor variable stator vanes (VSV): the actuator ring and its
//! schedule that re-pitch the IP compressor's stator rows with corrected
//! spool speed, the way every modern multi-stage axial compressor keeps
//! its rotor blades at a sensible incidence angle across a speed range far
//! wider than fixed geometry could tolerate without stalling at low speed
//! or choking at high speed. The actuator is a rate-limited position
//! servo like every other valve/vane actuator in this directory; what is
//! new here is the *consequence* of a position error: this module does not
//! touch the compressor map itself (that lives in the lead's gas path,
//! `physics::engine::compressor`, never edited here), it only computes the
//! stall-margin penalty an off-schedule vane angle would cost, as a
//! documented output for that model to consume -- exactly the "effect
//! output" this area's backlog calls for, not a model of the compressor
//! map.
//!
//! No Trent-900 VSV schedule or travel range is public. The schedule shape
//! (a monotonic angle-vs-corrected-speed line) and travel range
//! (**GENERIC**, ~40 degrees, the order of magnitude commonly quoted for
//! large axial compressor variable stators) and the quadratic stall-margin
//! penalty coefficient (**GENERIC**, sized so a schedule error of the full
//! travel range would cost stall margin on the order of what would actually
//! surge a compressor, without a published Trent map to calibrate against)
//! are typical/derived, not measured.

/// Vane travel range, degrees closed-to-open (**GENERIC**).
const MIN_ANGLE_DEG: f64 = -20.0;
const MAX_ANGLE_DEG: f64 = 20.0;
/// Actuator rate limit, degrees/s (**GENERIC**, typical of a hydromechanical
/// VSV actuation ring).
const ACTUATOR_RATE_DEG_S: f64 = 15.0;
/// Stall-margin penalty coefficient, %/degree^2 (**GENERIC**, see module
/// docs): margin lost is symmetric around the schedule (too open or too
/// closed both cost margin, in opposite physical ways -- too closed raises
/// incidence and risks rotating stall, too open reduces work done and
/// risks the following stage choking).
const STALL_MARGIN_COEFF_PCT_PER_DEG2: f64 = 0.02;

/// The schedule: vane angle commanded at a given IP-spool corrected speed
/// fraction (0..~1.2). **GENERIC** linear schedule (see module docs); real
/// schedules are typically a multi-segment curve, not published for this
/// engine.
pub fn schedule_angle_deg(n2_corrected_frac: f64) -> f64 {
    let f = n2_corrected_frac.clamp(0.0, 1.2).min(1.0);
    MIN_ANGLE_DEG + (MAX_ANGLE_DEG - MIN_ANGLE_DEG) * f
}

/// Faults the VSV system can carry, 0 (healthy) .. 1 (fully failed).
#[derive(Clone, Copy, Debug, Default)]
pub struct VsvFaults {
    /// Actuator jam: scales the rate limit to zero, freezing the vane ring
    /// wherever it is regardless of what the schedule now calls for.
    pub jam: f64,
    /// Rigging/feedback error, degrees: a persistent offset between where
    /// the actuator believes it is and the vanes' true position (a
    /// miscalibrated or slipped feedback linkage), signed.
    pub rigging_bias_deg: f64,
}

#[derive(Clone, Copy, Debug)]
pub struct Vsv {
    angle_deg: f64,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct VsvState {
    pub angle_deg: f64,
    /// True angle minus the ideal schedule angle (not the rigging-biased
    /// target the actuator was chasing) -- the physically meaningful error.
    pub schedule_error_deg: f64,
    /// Stall-margin change the gas path's compressor model should apply
    /// this frame, percentage points (negative = margin lost). A
    /// documented interface; this module does not touch the compressor map.
    pub stall_margin_delta_pct: f64,
}

impl Vsv {
    pub fn new(n2_corrected_frac: f64) -> Self {
        Self { angle_deg: schedule_angle_deg(n2_corrected_frac) }
    }

    /// One step. `n2_corrected_frac` is the IP spool's corrected speed
    /// fraction (0..~1.2) this frame.
    pub fn step(&mut self, n2_corrected_frac: f64, faults: &VsvFaults, dt_s: f64) -> VsvState {
        let dt = dt_s.max(0.0);
        let jam = faults.jam.clamp(0.0, 1.0);
        let ideal_target = schedule_angle_deg(n2_corrected_frac);
        let actuator_target = (ideal_target + faults.rigging_bias_deg).clamp(MIN_ANGLE_DEG, MAX_ANGLE_DEG);

        let rate = ACTUATOR_RATE_DEG_S * (1.0 - jam);
        let max_step = rate * dt;
        let error = actuator_target - self.angle_deg;
        self.angle_deg += error.clamp(-max_step, max_step);
        self.angle_deg = self.angle_deg.clamp(MIN_ANGLE_DEG, MAX_ANGLE_DEG);

        let schedule_error_deg = self.angle_deg - ideal_target;
        let stall_margin_delta_pct = -STALL_MARGIN_COEFF_PCT_PER_DEG2 * schedule_error_deg * schedule_error_deg;

        VsvState { angle_deg: self.angle_deg, schedule_error_deg, stall_margin_delta_pct }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_healthy_system_settles_on_schedule_with_no_margin_penalty() {
        let mut vsv = Vsv::new(0.0);
        let mut s = VsvState::default();
        for _ in 0..200 {
            s = vsv.step(0.6, &VsvFaults::default(), 0.1);
        }
        assert!((s.angle_deg - schedule_angle_deg(0.6)).abs() < 0.5);
        assert!(s.schedule_error_deg.abs() < 0.5);
        assert!(s.stall_margin_delta_pct > -0.01, "{}", s.stall_margin_delta_pct);
    }

    #[test]
    fn a_fully_jammed_actuator_never_moves_from_its_starting_angle() {
        let mut vsv = Vsv::new(0.0);
        let start = schedule_angle_deg(0.0);
        let s = vsv.step(1.0, &VsvFaults { jam: 1.0, ..Default::default() }, 5.0);
        assert!((s.angle_deg - start).abs() < 1e-9);
    }

    #[test]
    fn a_jammed_vane_far_off_schedule_costs_stall_margin() {
        let mut vsv = Vsv::new(0.0);
        for _ in 0..300 {
            vsv.step(1.0, &VsvFaults { jam: 1.0, ..Default::default() }, 0.1);
        }
        let s = vsv.step(1.0, &VsvFaults { jam: 1.0, ..Default::default() }, 0.1);
        assert!(s.schedule_error_deg.abs() > 30.0, "{}", s.schedule_error_deg);
        assert!(s.stall_margin_delta_pct < -10.0, "{}", s.stall_margin_delta_pct);
    }

    #[test]
    fn a_rigging_bias_settles_off_schedule_by_the_bias_amount() {
        let mut vsv = Vsv::new(0.5);
        let mut s = VsvState::default();
        for _ in 0..300 {
            s = vsv.step(0.5, &VsvFaults { rigging_bias_deg: 8.0, ..Default::default() }, 0.1);
        }
        assert!((s.schedule_error_deg - 8.0).abs() < 0.5, "{}", s.schedule_error_deg);
        assert!(s.stall_margin_delta_pct < 0.0);
    }

    #[test]
    fn zero_dt_gives_no_nan() {
        let mut vsv = Vsv::new(0.0);
        let s = vsv.step(0.5, &VsvFaults::default(), 0.0);
        assert!(!s.angle_deg.is_nan() && !s.stall_margin_delta_pct.is_nan());
    }
}
