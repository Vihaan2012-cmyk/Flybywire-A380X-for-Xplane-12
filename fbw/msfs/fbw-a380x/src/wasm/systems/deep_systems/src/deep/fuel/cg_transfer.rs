#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct TransferFaults {
    pub valve_stuck_fraction: f64,
    pub pump_degradation_fraction: f64,
    pub gallery_leak_fraction: f64,
}

pub fn achieved_transfer_rate_kg_s(nominal_rate_kg_s: f64, faults: &TransferFaults) -> f64 {
    let valve_factor = (1.0 - faults.valve_stuck_fraction.clamp(0.0, 1.0)).max(0.0);
    let pump_factor = (1.0 - faults.pump_degradation_fraction.clamp(0.0, 1.0)).max(0.0);
    let gallery_factor = (1.0 - faults.gallery_leak_fraction.clamp(0.0, 1.0)).max(0.0);
    (nominal_rate_kg_s.max(0.0) * valve_factor * pump_factor * gallery_factor).max(0.0)
}

#[derive(Clone, Copy, Debug, Default)]
pub struct TransferFaultDetector {
    shortfall_held_s: f64,
}
impl TransferFaultDetector {
    pub fn update(&mut self, required_rate_kg_s: f64, achieved_rate_kg_s: f64, tolerance_fraction: f64, confirm_s: f64, dt_s: f64) -> bool {
        let shortfall = required_rate_kg_s > 1e-6 && achieved_rate_kg_s < required_rate_kg_s * (1.0 - tolerance_fraction.clamp(0.0, 1.0));
        self.shortfall_held_s = if shortfall { self.shortfall_held_s + dt_s.max(0.0) } else { 0.0 };
        self.shortfall_held_s >= confirm_s
    }
}

pub fn wing_bending_relief_nm(mass_kg: f64, span_m: f64) -> f64 {
    mass_kg.max(0.0) * super::geometry::G * span_m.abs()
}

pub fn outer_tank_retention_active(inner_fill_fraction: f64, mid_fill_fraction: f64, retain_until_fraction: f64) -> bool {
    inner_fill_fraction > retain_until_fraction || mid_fill_fraction > retain_until_fraction
}

pub fn wing_balance_transfer_needed(left_total_kg: f64, right_total_kg: f64, imbalance_limit_kg: f64) -> bool {
    (left_total_kg - right_total_kg).abs() > imbalance_limit_kg.max(0.0)
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum HeavySide {
    Left,
    Right,
}
pub fn heavy_side(left_total_kg: f64, right_total_kg: f64, imbalance_limit_kg: f64) -> Option<HeavySide> {
    if !wing_balance_transfer_needed(left_total_kg, right_total_kg, imbalance_limit_kg) {
        return None;
    }
    Some(if left_total_kg > right_total_kg { HeavySide::Left } else { HeavySide::Right })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_healthy_path_delivers_the_full_nominal_rate() {
        assert_eq!(achieved_transfer_rate_kg_s(10.0, &TransferFaults::default()), 10.0);
    }

    #[test]
    fn a_half_stuck_valve_halves_the_delivered_rate() {
        let f = TransferFaults { valve_stuck_fraction: 0.5, ..Default::default() };
        assert!((achieved_transfer_rate_kg_s(10.0, &f) - 5.0).abs() < 1e-9);
    }

    #[test]
    fn faults_combine_multiplicatively_and_never_go_negative() {
        let f = TransferFaults { valve_stuck_fraction: 0.5, pump_degradation_fraction: 0.5, gallery_leak_fraction: 0.5 };
        let rate = achieved_transfer_rate_kg_s(8.0, &f);
        assert!((rate - 1.0).abs() < 1e-9, "{rate}");
        let full_fail = TransferFaults { valve_stuck_fraction: 1.0, ..Default::default() };
        assert_eq!(achieved_transfer_rate_kg_s(8.0, &full_fail), 0.0);
    }

    #[test]
    fn no_nan_at_zero_nominal_rate_or_zero_dt() {
        let f = TransferFaults::default();
        assert_eq!(achieved_transfer_rate_kg_s(0.0, &f), 0.0);
        let mut det = TransferFaultDetector::default();
        assert!(!det.update(0.0, 0.0, 0.1, 5.0, 0.0));
    }

    #[test]
    fn a_sustained_shortfall_confirms_after_its_timer_and_a_brief_one_does_not() {
        let mut det = TransferFaultDetector::default();
        assert!(!det.update(10.0, 2.0, 0.1, 5.0, 4.0));
        assert!(det.update(10.0, 2.0, 0.1, 5.0, 2.0));
        let mut det2 = TransferFaultDetector::default();
        assert!(!det2.update(10.0, 2.0, 0.1, 5.0, 1.0));
        assert!(!det2.update(10.0, 9.5, 0.1, 5.0, 10.0), "recovering the rate resets the timer");
    }

    #[test]
    fn bending_relief_scales_with_mass_and_span_and_ignores_sign() {
        let inboard = wing_bending_relief_nm(1000.0, 10.0);
        let outboard = wing_bending_relief_nm(1000.0, 30.0);
        assert!(outboard > inboard, "fuel further out relieves more root bending moment");
        assert_eq!(wing_bending_relief_nm(1000.0, -30.0), outboard);
        assert_eq!(wing_bending_relief_nm(0.0, 30.0), 0.0);
    }

    #[test]
    fn outer_tanks_are_retained_while_inner_or_mid_still_hold_fuel() {
        assert!(outer_tank_retention_active(0.6, 0.1, 0.3));
        assert!(outer_tank_retention_active(0.1, 0.6, 0.3));
        assert!(!outer_tank_retention_active(0.1, 0.1, 0.3));
    }

    #[test]
    fn wing_balance_flags_only_past_the_limit_and_names_the_heavy_side() {
        assert!(!wing_balance_transfer_needed(50_000.0, 49_600.0, 500.0));
        assert!(wing_balance_transfer_needed(50_000.0, 49_000.0, 500.0));
        assert_eq!(heavy_side(50_000.0, 49_000.0, 500.0), Some(HeavySide::Left));
        assert_eq!(heavy_side(49_000.0, 50_000.0, 500.0), Some(HeavySide::Right));
        assert_eq!(heavy_side(50_000.0, 49_600.0, 500.0), None);
    }
}
