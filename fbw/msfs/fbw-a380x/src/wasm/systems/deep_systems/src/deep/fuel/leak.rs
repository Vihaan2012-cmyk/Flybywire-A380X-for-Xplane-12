use super::jettison::{head_pressure_pa, orifice_flow_m3_s};

pub fn tank_wall_leak_kg_s(orifice_area_m2: f64, liquid_depth_above_hole_m: f64, density_kg_m3: f64) -> f64 {
    let dp = head_pressure_pa(liquid_depth_above_hole_m, density_kg_m3);
    orifice_flow_m3_s(orifice_area_m2, dp, density_kg_m3) * density_kg_m3.max(0.0)
}

pub fn gallery_leak_kg_s(orifice_area_m2: f64, line_pressure_pa: f64, ambient_pressure_pa: f64, density_kg_m3: f64) -> f64 {
    let dp = line_pressure_pa - ambient_pressure_pa;
    orifice_flow_m3_s(orifice_area_m2, dp, density_kg_m3) * density_kg_m3.max(0.0)
}

pub fn leak_area_m2(magnitude_0_1: f64, max_area_m2: f64) -> f64 {
    magnitude_0_1.clamp(0.0, 1.0) * max_area_m2.max(0.0)
}

#[derive(Clone, Debug, Default)]
pub struct LeakDetector {
    fuel_used_accum_kg: f64,
    window_start_quantity_kg: Option<f64>,
    window_elapsed_s: f64,
    consecutive_over_threshold_windows: u32,
}
impl LeakDetector {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn update(&mut self, indicated_total_kg: f64, fuel_flow_kg_s: f64, dt_s: f64, window_s: f64, threshold_kg: f64, confirm_windows: u32) -> bool {
        let start = *self.window_start_quantity_kg.get_or_insert(indicated_total_kg);
        self.fuel_used_accum_kg += fuel_flow_kg_s.max(0.0) * dt_s.max(0.0);
        self.window_elapsed_s += dt_s.max(0.0);
        if self.window_elapsed_s < window_s.max(1e-6) {
            return self.consecutive_over_threshold_windows >= confirm_windows.max(1);
        }
        let actual_decrease_kg = (start - indicated_total_kg).max(0.0);
        let discrepancy_kg = actual_decrease_kg - self.fuel_used_accum_kg;
        if discrepancy_kg > threshold_kg.max(0.0) {
            self.consecutive_over_threshold_windows += 1;
        } else {
            self.consecutive_over_threshold_windows = 0;
        }
        self.window_start_quantity_kg = Some(indicated_total_kg);
        self.fuel_used_accum_kg = 0.0;
        self.window_elapsed_s = 0.0;
        self.consecutive_over_threshold_windows >= confirm_windows.max(1)
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Side {
    Left,
    Right,
}

pub fn likely_leaking_side(left_discrepancy_kg: f64, right_discrepancy_kg: f64, threshold_kg: f64) -> Option<Side> {
    let l = left_discrepancy_kg > threshold_kg.max(0.0);
    let r = right_discrepancy_kg > threshold_kg.max(0.0);
    match (l, r) {
        (true, false) => Some(Side::Left),
        (false, true) => Some(Side::Right),
        (true, true) => {
            if left_discrepancy_kg >= right_discrepancy_kg {
                Some(Side::Left)
            } else {
                Some(Side::Right)
            }
        }
        (false, false) => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const RHO: f64 = 800.0;

    #[test]
    fn no_leak_flow_from_an_empty_tank_or_a_closed_orifice() {
        assert_eq!(tank_wall_leak_kg_s(0.0001, 0.0, RHO), 0.0);
        assert_eq!(tank_wall_leak_kg_s(0.0, 2.0, RHO), 0.0);
    }

    #[test]
    fn a_deeper_tank_leaks_faster_than_a_shallow_one() {
        let shallow = tank_wall_leak_kg_s(0.0001, 0.5, RHO);
        let deep = tank_wall_leak_kg_s(0.0001, 3.0, RHO);
        assert!(deep > shallow);
    }

    #[test]
    fn a_gallery_leak_needs_positive_line_pressure_over_ambient() {
        assert_eq!(gallery_leak_kg_s(0.0001, 50_000.0, 101_325.0, RHO), 0.0, "gauge pressure below ambient leaks nothing");
        assert!(gallery_leak_kg_s(0.0001, 250_000.0, 101_325.0, RHO) > 0.0);
    }

    #[test]
    fn leak_area_scales_linearly_with_magnitude() {
        assert_eq!(leak_area_m2(0.0, 0.001), 0.0);
        assert_eq!(leak_area_m2(1.0, 0.001), 0.001);
        assert!((leak_area_m2(0.5, 0.001) - 0.0005).abs() < 1e-12);
    }

    #[test]
    fn no_leak_flagged_when_quantity_loss_matches_fuel_used() {
        let mut det = LeakDetector::new();
        let mut qty = 10_000.0;
        let flow = 1.0;
        let dt = 1.0;
        let mut flagged = false;
        for _ in 0..600 {
            qty -= flow * dt;
            flagged = det.update(qty, flow, dt, 60.0, 50.0, 2) || flagged;
        }
        assert!(!flagged);
    }

    #[test]
    fn a_sustained_unmetered_loss_is_flagged_after_confirm_windows() {
        let mut det = LeakDetector::new();
        let mut qty = 10_000.0;
        let flow = 1.0;
        let leak = 0.5;
        let dt = 1.0;
        let mut flagged = false;
        for _ in 0..130 {
            qty -= (flow + leak) * dt;
            flagged = det.update(qty, flow, dt, 60.0, 20.0, 2);
        }
        assert!(flagged, "a steady unmetered 0.5 kg/s loss over two 60 s windows should exceed a 20 kg threshold");
    }

    #[test]
    fn a_single_noisy_window_alone_does_not_confirm_with_confirm_windows_above_one() {
        let mut det = LeakDetector::new();
        assert!(!det.update(9_000.0, 0.0, 60.0, 60.0, 20.0, 2));
        assert!(!det.update(9_000.0, 0.0, 60.0, 60.0, 20.0, 2));
    }

    #[test]
    fn side_isolation_names_the_heavier_discrepancy_and_none_below_threshold() {
        assert_eq!(likely_leaking_side(5.0, 100.0, 20.0), Some(Side::Right));
        assert_eq!(likely_leaking_side(100.0, 5.0, 20.0), Some(Side::Left));
        assert_eq!(likely_leaking_side(5.0, 5.0, 20.0), None);
        assert_eq!(likely_leaking_side(100.0, 90.0, 20.0), Some(Side::Left));
    }
}
