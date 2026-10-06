use super::turbine::PEAK_POWER_W;

const THERMAL_MASS_J_K: f64 = 45_000.0;
const LOSS_W_K: f64 = 90.0;
const HEAT_FRACTION: f64 = 0.15;
const OVERHEAT_RISE_K: f64 = 120.0;

#[derive(Clone, Copy, Debug)]
pub struct DutyCycleHeat {
    rise_k: f64,
}

impl DutyCycleHeat {
    pub fn new() -> Self {
        Self { rise_k: 0.0 }
    }

    pub fn rise_k(&self) -> f64 {
        self.rise_k
    }

    pub fn overheated(&self) -> bool {
        self.rise_k >= OVERHEAT_RISE_K
    }

    pub fn step(&mut self, cranking_power_w: f64, dt_s: f64) {
        let dt = dt_s.max(0.0);
        let heat_in_w = cranking_power_w.max(0.0) * HEAT_FRACTION;
        let target = heat_in_w / LOSS_W_K;
        let k = LOSS_W_K / THERMAL_MASS_J_K;
        self.rise_k = target + (self.rise_k - target) * (-k * dt).exp();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn starting_from_cold_gives_zero_rise_and_no_nan() {
        let h = DutyCycleHeat::new();
        assert_eq!(h.rise_k(), 0.0);
        assert!(!h.overheated());
    }

    #[test]
    fn a_single_normal_length_crank_does_not_overheat_it() {
        let mut h = DutyCycleHeat::new();
        for _ in 0..(60.0 / 0.1) as usize {
            h.step(PEAK_POWER_W * 0.6, 0.1);
        }
        assert!(!h.overheated(), "{} K rise after one start", h.rise_k());
        assert!(h.rise_k() > 0.0);
    }

    #[test]
    fn repeated_back_to_back_cranks_without_cooldown_eventually_overheat_it() {
        let mut h = DutyCycleHeat::new();
        for _ in 0..(600.0 / 0.1) as usize {
            h.step(PEAK_POWER_W * 0.6, 0.1);
        }
        assert!(h.overheated(), "{} K rise after ten minutes continuous cranking", h.rise_k());
    }

    #[test]
    fn it_cools_back_down_once_cranking_stops() {
        let mut h = DutyCycleHeat::new();
        for _ in 0..(60.0 / 0.1) as usize {
            h.step(PEAK_POWER_W * 0.6, 0.1);
        }
        let after_crank = h.rise_k();
        for _ in 0..(600.0 / 0.5) as usize {
            h.step(0.0, 0.5);
        }
        assert!(h.rise_k() < after_crank * 0.5, "should have cooled substantially in ten minutes at rest");
    }

    #[test]
    fn zero_dt_never_produces_nan() {
        let mut h = DutyCycleHeat::new();
        h.step(PEAK_POWER_W, 0.0);
        assert!(!h.rise_k().is_nan());
    }
}
