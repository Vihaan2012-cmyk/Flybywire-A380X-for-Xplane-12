#[derive(Clone, Copy, Debug)]
pub struct SpoolSpec {
    pub omega_n_rad_s: f64,
    pub stiffness_n_m: f64,
    pub zeta: f64,
    pub reference_unbalance_kg_m: f64,
    pub mm_per_index_unit: f64,
    pub filter_tau_s: f64,
}

pub const FAN_SPEC: SpoolSpec = SpoolSpec { omega_n_rad_s: 45.0, stiffness_n_m: 6.0e7, zeta: 0.12, reference_unbalance_kg_m: 12.0, mm_per_index_unit: 0.08, filter_tau_s: 0.6 };
pub const IP_SPEC: SpoolSpec = SpoolSpec { omega_n_rad_s: 120.0, stiffness_n_m: 1.2e8, zeta: 0.1, reference_unbalance_kg_m: 3.0, mm_per_index_unit: 0.05, filter_tau_s: 0.5 };
pub const HP_SPEC: SpoolSpec = SpoolSpec { omega_n_rad_s: 220.0, stiffness_n_m: 2.5e8, zeta: 0.08, reference_unbalance_kg_m: 1.0, mm_per_index_unit: 0.03, filter_tau_s: 0.4 };

#[derive(Clone, Copy, Debug, Default)]
pub struct ImbalanceFaults {
    pub blade_loss_frac: f64,
    pub ice_frac: f64,
    pub bird_strike_frac: f64,
}

#[derive(Clone, Copy, Debug)]
pub struct SpoolVibration {
    spec: SpoolSpec,
    filtered_index: f64,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct VibrationState {
    pub index: f64,
    pub instantaneous_index: f64,
}

impl SpoolVibration {
    pub fn new(spec: SpoolSpec) -> Self {
        Self { spec, filtered_index: 0.0 }
    }

    pub fn step(&mut self, omega_rad_s: f64, faults: &ImbalanceFaults, dt_s: f64) -> VibrationState {
        let omega = omega_rad_s.max(0.0);
        let unbalance_kg_m = self.spec.reference_unbalance_kg_m
            * (faults.blade_loss_frac.clamp(0.0, 1.0) + faults.ice_frac.clamp(0.0, 1.0) + faults.bird_strike_frac.clamp(0.0, 1.0));
        let force_n = unbalance_kg_m * omega * omega;
        let r = omega / self.spec.omega_n_rad_s;
        let denom = ((1.0 - r * r).powi(2) + (2.0 * self.spec.zeta * r).powi(2)).sqrt().max(1e-9);
        let displacement_m = force_n / self.spec.stiffness_n_m / denom;
        let instantaneous_index = (displacement_m * 1000.0) / self.spec.mm_per_index_unit;

        let dt = dt_s.max(0.0);
        let k = 1.0 - (-dt / self.spec.filter_tau_s).exp();
        self.filtered_index += (instantaneous_index - self.filtered_index) * k;

        VibrationState { index: self.filtered_index, instantaneous_index }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn settle(spec: SpoolSpec, omega: f64, faults: &ImbalanceFaults, seconds: f64) -> VibrationState {
        let mut v = SpoolVibration::new(spec);
        let dt = 0.02;
        let mut out = VibrationState::default();
        for _ in 0..(seconds / dt) as usize {
            out = v.step(omega, faults, dt);
        }
        out
    }

    #[test]
    fn a_balanced_rotor_reads_zero_vibration_at_any_speed_no_nan() {
        let s = settle(FAN_SPEC, 300.0, &ImbalanceFaults::default(), 5.0);
        assert_eq!(s.index, 0.0);
        assert!(!s.index.is_nan());
    }

    #[test]
    fn zero_speed_gives_zero_vibration_even_with_a_full_fault() {
        let s = settle(FAN_SPEC, 0.0, &ImbalanceFaults { blade_loss_frac: 1.0, ..Default::default() }, 5.0);
        assert_eq!(s.index, 0.0);
    }

    #[test]
    fn vibration_rises_with_the_square_of_speed_away_from_resonance() {
        let low = settle(FAN_SPEC, 5.0, &ImbalanceFaults { blade_loss_frac: 1.0, ..Default::default() }, 5.0);
        let high = settle(FAN_SPEC, 10.0, &ImbalanceFaults { blade_loss_frac: 1.0, ..Default::default() }, 5.0);
        assert!((high.index / low.index - 4.0).abs() < 0.5, "ratio {}", high.index / low.index);
    }

    #[test]
    fn a_blade_loss_gives_far_more_vibration_than_healthy() {
        let healthy = settle(FAN_SPEC, 300.0, &ImbalanceFaults::default(), 5.0);
        let damaged = settle(FAN_SPEC, 300.0, &ImbalanceFaults { blade_loss_frac: 1.0, ..Default::default() }, 5.0);
        assert!(damaged.index > healthy.index + 1.0);
    }

    #[test]
    fn multiple_simultaneous_causes_add_up() {
        let one = settle(FAN_SPEC, 300.0, &ImbalanceFaults { ice_frac: 0.5, ..Default::default() }, 5.0);
        let two = settle(FAN_SPEC, 300.0, &ImbalanceFaults { ice_frac: 0.5, bird_strike_frac: 0.5, ..Default::default() }, 5.0);
        assert!(two.index > one.index);
    }

    #[test]
    fn the_tracking_filter_lags_a_sudden_imbalance_rather_than_jumping_instantly() {
        let mut v = SpoolVibration::new(FAN_SPEC);
        v.step(300.0, &ImbalanceFaults::default(), 1.0);
        let s = v.step(300.0, &ImbalanceFaults { blade_loss_frac: 1.0, ..Default::default() }, 0.05);
        assert!(s.index > 0.0 && s.index < s.instantaneous_index, "the filtered reading should lag the instantaneous one");
    }
}
