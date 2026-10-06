const STANDARD_GRAVITY_MS2: f64 = 9.806_65;
const WASHOUT_TAU_S: f64 = 300.0;
const MIN_TAS_MS: f64 = 5.0;

#[derive(Clone, Copy, Debug, Default)]
pub struct SideslipEstimator {
    beta_deg: f64,
}

impl SideslipEstimator {
    pub fn new() -> Self {
        Self { beta_deg: 0.0 }
    }

    pub fn step(&mut self, lateral_accel_ms2: f64, yaw_rate_rad_s: f64, roll_deg: f64, pitch_deg: f64, tas_ms: f64, dt_s: f64) -> f64 {
        let dt = dt_s.max(0.0);
        let v = tas_ms.max(MIN_TAS_MS);
        let (roll_rad, pitch_rad) = (roll_deg.to_radians(), pitch_deg.to_radians());
        let beta_dot_rad_s = lateral_accel_ms2 / v - yaw_rate_rad_s + (STANDARD_GRAVITY_MS2 / v) * roll_rad.sin() * pitch_rad.cos();
        self.beta_deg += beta_dot_rad_s.to_degrees() * dt;
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
        for _ in 0..20 {
            beta = e.step(2.0, 0.0, 0.0, 0.0, 150.0, 0.1);
        }
        assert!(beta.abs() > 0.01, "expected a measurable sideslip build-up, got {beta}");
        let built_up = beta;
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
