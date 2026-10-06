const MIN_ANGLE_DEG: f64 = -20.0;
const MAX_ANGLE_DEG: f64 = 20.0;
const ACTUATOR_RATE_DEG_S: f64 = 15.0;
const STALL_MARGIN_COEFF_PCT_PER_DEG2: f64 = 0.02;

pub fn schedule_angle_deg(n2_corrected_frac: f64) -> f64 {
    let f = n2_corrected_frac.clamp(0.0, 1.2).min(1.0);
    MIN_ANGLE_DEG + (MAX_ANGLE_DEG - MIN_ANGLE_DEG) * f
}

#[derive(Clone, Copy, Debug, Default)]
pub struct VsvFaults {
    pub jam: f64,
    pub rigging_bias_deg: f64,
}

#[derive(Clone, Copy, Debug)]
pub struct Vsv {
    angle_deg: f64,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct VsvState {
    pub angle_deg: f64,
    pub schedule_error_deg: f64,
    pub stall_margin_delta_pct: f64,
}

impl Vsv {
    pub fn new(n2_corrected_frac: f64) -> Self {
        Self { angle_deg: schedule_angle_deg(n2_corrected_frac) }
    }

    pub fn step(&mut self, n2_corrected_frac: f64, faults: &VsvFaults, dt_s: f64) -> VsvState {
        let dt = dt_s.max(0.0);
        let jam = faults.jam.clamp(0.0, 1.0);
        let ideal_target = schedule_angle_deg(n2_corrected_frac);
        let actuator_target = (ideal_target + faults.rigging_bias_deg).clamp(MIN_ANGLE_DEG, MAX_ANGLE_DEG);

        let stiction_deg = (MAX_ANGLE_DEG - MIN_ANGLE_DEG) * jam;
        let error = actuator_target - self.angle_deg;
        if error.abs() > stiction_deg {
            let max_step = ACTUATOR_RATE_DEG_S * dt;
            let closing = (error.abs() - stiction_deg).min(max_step);
            self.angle_deg += closing * error.signum();
        }
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
