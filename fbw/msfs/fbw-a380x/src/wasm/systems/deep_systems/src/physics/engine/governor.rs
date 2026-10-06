use super::params::{
    MAX_COMBUSTOR_FUEL_AIR_RATIO, MAX_N1_PROTECTION_PCT, MAX_N3_PROTECTION_PCT, MIN_N1_FOR_COMBUSTION_PCT,
    MIN_N3_FOR_COMBUSTION_PCT, STATIC_THRUST_N,
};

pub fn generic_sfc_reference_wf_kg_s() -> f64 {
    const SFC_LB_PER_LBF_HR: f64 = 0.33;
    let static_thrust_lbf = STATIC_THRUST_N / 4.448_221_615_3;
    let wf_lb_hr = SFC_LB_PER_LBF_HR * static_thrust_lbf;
    wf_lb_hr * 0.453_593_4 / 3600.0
}

const KP: f64 = 0.06;
const KI: f64 = 0.02;
const INTEGRAL_LIMIT: f64 = 150.0;

fn accel_schedule_max_wf_kg_s(n3_corrected_pct: f64, design_wf_kg_s: f64) -> f64 {
    let n3_frac = (n3_corrected_pct / 100.0).clamp(0.0, 1.2);
    let multiple = ACCEL_SCHEDULE_BASE + ACCEL_SCHEDULE_SLOPE * n3_frac;
    design_wf_kg_s * multiple
}

pub const ACCEL_FAR_MARGIN: f64 = 2.30;

fn accel_far_multiple(n3_corrected_pct: f64) -> f64 {
    let n3 = (n3_corrected_pct / 100.0).clamp(0.0, 1.0);
    n3 * n3 * ACCEL_FAR_MARGIN
}

const ACCEL_SCHEDULE_BASE: f64 = 1.3;
const ACCEL_SCHEDULE_SLOPE: f64 = 0.9;

#[derive(Clone, Copy, Debug, Default)]
pub struct Governor {
    integral: f64,
    design_far: Option<f64>,
    ramp: Option<f64>,
}

const POST_START_N1_RAMP_PCT_S: f64 = 2.0;

impl Governor {
    pub fn new() -> Self {
        Self { integral: 0.0, ramp: None, design_far: None }
    }

    pub fn with_design_far(far: f64) -> Self {
        Self { design_far: Some(far), ..Self::new() }
    }

    pub fn track(&mut self, wf_kg_s: f64, measured_n1_corrected_pct: f64, design_wf_kg_s: f64) {
        let feedforward = design_wf_kg_s * (measured_n1_corrected_pct / 100.0).max(0.0).powi(3);
        self.integral = ((wf_kg_s - feedforward) / KI).clamp(-INTEGRAL_LIMIT, INTEGRAL_LIMIT);
        self.ramp = Some(measured_n1_corrected_pct);
    }

    pub fn combustion_floor_met(n1_corrected_pct: f64, n3_corrected_pct: f64) -> bool {
        n1_corrected_pct >= MIN_N1_FOR_COMBUSTION_PCT || n3_corrected_pct >= MIN_N3_FOR_COMBUSTION_PCT
    }

    pub fn step(
        &mut self,
        target_n1_corrected_pct: f64,
        measured_n1_corrected_pct: f64,
        measured_n3_corrected_pct: f64,
        fuel_valve_open: bool,
        mdot_air_to_combustor_kg_s: f64,
        design_wf_kg_s: f64,
        dt_s: f64,
    ) -> f64 {
        if !fuel_valve_open || !Self::combustion_floor_met(measured_n1_corrected_pct, measured_n3_corrected_pct) {
            self.integral = 0.0;
            return 0.0;
        }

        let far_limit = mdot_air_to_combustor_kg_s.max(0.0) * MAX_COMBUSTOR_FUEL_AIR_RATIO;
        let accel_limit = match self.design_far {
            Some(far) => mdot_air_to_combustor_kg_s.max(0.0) * far * accel_far_multiple(measured_n3_corrected_pct),
            None => accel_schedule_max_wf_kg_s(measured_n3_corrected_pct, design_wf_kg_s),
        };

        let target_n1_corrected_pct = match self.ramp {
            Some(r) => {
                let next = (r + POST_START_N1_RAMP_PCT_S * dt_s).min(target_n1_corrected_pct);
                self.ramp = if next >= target_n1_corrected_pct { None } else { Some(next) };
                next
            }
            None => target_n1_corrected_pct,
        };
        let error = target_n1_corrected_pct - measured_n1_corrected_pct;
        let feedforward = design_wf_kg_s * (target_n1_corrected_pct / 100.0).max(0.0).powi(3);

        let ceiling = accel_limit.min(far_limit);
        let trial = self.integral + error * dt_s;
        let unclamped = feedforward + KP * error + KI * trial;
        let pinned = (unclamped > ceiling && error > 0.0) || (unclamped < 0.0 && error < 0.0);
        if !pinned {
            self.integral = trial.clamp(-INTEGRAL_LIMIT, INTEGRAL_LIMIT);
        }
        let mut wf = feedforward + KP * error + KI * self.integral;

        if measured_n1_corrected_pct > MAX_N1_PROTECTION_PCT || measured_n3_corrected_pct > MAX_N3_PROTECTION_PCT {
            wf = wf.min(design_wf_kg_s * 0.5);
        }

        wf.clamp(0.0, ceiling)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn no_fuel_below_the_combustion_floor() {
        assert!(!Governor::combustion_floor_met(5.0, 10.0));
        assert!(Governor::combustion_floor_met(12.0, 10.0));
        assert!(Governor::combustion_floor_met(5.0, 25.0));
    }

    const AMPLE_MDOT_KG_S: f64 = 200.0;

    #[test]
    fn a_closed_fuel_valve_gives_zero_flow() {
        let mut g = Governor::new();
        let wf = g.step(90.0, 40.0, 60.0, false, AMPLE_MDOT_KG_S, generic_sfc_reference_wf_kg_s(), 0.1);
        assert_eq!(wf, 0.0);
    }

    #[test]
    fn a_low_target_below_the_combustion_floor_gives_no_fuel() {
        let mut g = Governor::new();
        let wf = g.step(5.0, 2.0, 5.0, true, AMPLE_MDOT_KG_S, generic_sfc_reference_wf_kg_s(), 0.1);
        assert_eq!(wf, 0.0);
    }

    #[test]
    fn a_higher_target_commands_more_fuel_once_lit() {
        let mut idle_gov = Governor::new();
        let mut toga_gov = Governor::new();
        let idle_wf = idle_gov.step(25.0, 25.0, 60.0, true, AMPLE_MDOT_KG_S, generic_sfc_reference_wf_kg_s(), 0.1);
        let toga_wf = toga_gov.step(100.0, 25.0, 60.0, true, AMPLE_MDOT_KG_S, generic_sfc_reference_wf_kg_s(), 0.1);
        assert!(toga_wf > idle_wf, "{idle_wf} {toga_wf}");
    }

    #[test]
    fn overspeed_pulls_fuel_back() {
        let mut g = Governor::new();
        let wf = g.step(100.0, 105.0, 60.0, true, AMPLE_MDOT_KG_S, generic_sfc_reference_wf_kg_s(), 0.1);
        assert!(wf <= generic_sfc_reference_wf_kg_s() * 0.5 + 1e-9);
    }

    #[test]
    fn the_feedforward_fuel_flow_is_a_plausible_order_of_magnitude() {
        let wf = generic_sfc_reference_wf_kg_s();
        assert!(wf > 1.0 && wf < 6.0, "{wf}");
    }

    #[test]
    fn an_early_light_off_fuel_flow_is_bounded_by_the_fuel_air_ratio_not_unbounded_by_n1_error() {
        let mut g = Governor::new();
        let idle_n1_target_pct = 19.0;
        let core_mdot_kg_s = 5.0;
        let wf = g.step(idle_n1_target_pct, 14.0, 33.0, true, core_mdot_kg_s, generic_sfc_reference_wf_kg_s(), 0.05);
        let far_limit = core_mdot_kg_s * super::MAX_COMBUSTOR_FUEL_AIR_RATIO;
        assert!(wf <= far_limit + 1e-9, "wf {wf} exceeded the fuel-air-ratio backstop {far_limit}");
        assert!(wf * 3600.0 < 2000.0, "{} kg/h, expected well under the field report's ~3000 kg/h", wf * 3600.0);
    }

    #[test]
    fn the_fuel_air_ratio_backstop_still_bounds_a_pathological_error() {
        let mut g = Governor::new();
        let core_mdot_kg_s = 2.0;
        let far_limit = core_mdot_kg_s * super::MAX_COMBUSTOR_FUEL_AIR_RATIO;
        let mut wf = 0.0;
        for _ in 0..50 {
            wf = g.step(100.0, 5.0, 25.0, true, core_mdot_kg_s, generic_sfc_reference_wf_kg_s(), 0.1);
        }
        assert!(wf <= far_limit + 1e-9, "wf {wf} exceeded the fuel-air-ratio backstop {far_limit}");
    }
}
