#[derive(Clone, Copy, Debug)]
pub struct IceCrystalInputs {
    pub ice_water_content_g_m3: f64,
    pub core_mass_flow_kg_s: f64,
    pub warm_surface_temp_c: f64,
    pub dt_s: f64,
}

#[derive(Clone, Copy, Debug)]
pub struct IceCrystalOutputs {
    pub accreted_mass_kg: f64,
    pub flow_capacity_loss_frac: f64,
    pub rollback_risk_frac: f64,
    pub flameout_risk_frac: f64,
    pub shedding_event: bool,
}

const ICE_DENSITY_KG_M3: f64 = 700.0;
const CAPTURE_FRACTION: f64 = 0.02;
const STICKING_WINDOW_C: f64 = 4.0;
const FULL_BLOCKAGE_MASS_KG: f64 = 0.5;
const SHED_THRESHOLD_FRAC: f64 = 0.6;
const SHED_RETAINED_FRACTION: f64 = 0.1;

fn sticking_efficiency(surface_temp_c: f64) -> f64 {
    let x = surface_temp_c / STICKING_WINDOW_C;
    (1.0 - x * x).max(0.0)
}

#[derive(Clone, Copy, Debug, Default)]
pub struct IceCrystalIcingState {
    accreted_mass_kg: f64,
}

impl IceCrystalIcingState {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn step(&mut self, i: &IceCrystalInputs) -> IceCrystalOutputs {
        let dt = i.dt_s.max(0.0);
        let iwc_kg_m3 = i.ice_water_content_g_m3.max(0.0) * 1e-3;
        let ice_mass_flow_kg_s = iwc_kg_m3 * i.core_mass_flow_kg_s.max(0.0);
        let efficiency = sticking_efficiency(i.warm_surface_temp_c);
        let accrete_rate_kg_s = ice_mass_flow_kg_s * CAPTURE_FRACTION * efficiency;
        self.accreted_mass_kg += accrete_rate_kg_s * dt;

        let mut flow_capacity_loss_frac = (self.accreted_mass_kg / FULL_BLOCKAGE_MASS_KG).min(1.0);
        let mut shedding_event = false;
        if flow_capacity_loss_frac >= SHED_THRESHOLD_FRAC {
            self.accreted_mass_kg *= SHED_RETAINED_FRACTION;
            flow_capacity_loss_frac = (self.accreted_mass_kg / FULL_BLOCKAGE_MASS_KG).min(1.0);
            shedding_event = true;
        }

        let rollback_risk_frac = flow_capacity_loss_frac.powi(2);
        let flameout_risk_frac = (rollback_risk_frac + if shedding_event { 0.4 } else { 0.0 }).min(1.0);

        IceCrystalOutputs { accreted_mass_kg: self.accreted_mass_kg, flow_capacity_loss_frac, rollback_risk_frac, flameout_risk_frac, shedding_event }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn inputs(iwc_g_m3: f64, surface_temp_c: f64, dt_s: f64) -> IceCrystalInputs {
        IceCrystalInputs { ice_water_content_g_m3: iwc_g_m3, core_mass_flow_kg_s: 15.0, warm_surface_temp_c: surface_temp_c, dt_s }
    }

    #[test]
    fn no_ice_water_or_zero_dt_causes_no_accretion_and_no_nan() {
        let mut s = IceCrystalIcingState::new();
        let out = s.step(&inputs(0.0, 0.0, 10.0));
        assert_eq!(out.accreted_mass_kg, 0.0);
        assert_eq!(out.flow_capacity_loss_frac, 0.0);
        assert!(!out.rollback_risk_frac.is_nan());
        let out2 = s.step(&inputs(5.0, 0.0, 0.0));
        assert_eq!(out2.accreted_mass_kg, 0.0);
    }

    #[test]
    fn sticking_efficiency_peaks_at_zero_and_vanishes_outside_the_window() {
        assert!((sticking_efficiency(0.0) - 1.0).abs() < 1e-9);
        assert_eq!(sticking_efficiency(-20.0), 0.0);
        assert_eq!(sticking_efficiency(20.0), 0.0);
        assert!(sticking_efficiency(0.0) > sticking_efficiency(2.0));
    }

    #[test]
    fn cold_dry_or_hot_wet_surfaces_accrete_nothing_despite_high_ice_water_content() {
        let mut cold = IceCrystalIcingState::new();
        let cold_out = cold.step(&inputs(6.0, -30.0, 120.0));
        assert_eq!(cold_out.accreted_mass_kg, 0.0);

        let mut hot = IceCrystalIcingState::new();
        let hot_out = hot.step(&inputs(6.0, 40.0, 120.0));
        assert_eq!(hot_out.accreted_mass_kg, 0.0);
    }

    #[test]
    fn a_near_freezing_surface_accretes_ice_and_raises_rollback_risk() {
        let mut s = IceCrystalIcingState::new();
        let a = s.step(&inputs(6.0, 0.0, 60.0));
        let b = s.step(&inputs(6.0, 0.0, 60.0));
        assert!(a.accreted_mass_kg > 0.0);
        assert!(b.accreted_mass_kg > a.accreted_mass_kg || b.shedding_event);
        assert!(b.rollback_risk_frac > 0.0);
    }

    #[test]
    fn accretion_sheds_once_the_threshold_is_crossed_and_the_mass_drops() {
        let mut s = IceCrystalIcingState::new();
        let mut shed_seen = false;
        let mut last_mass = 0.0;
        for _ in 0..200 {
            let out = s.step(&inputs(8.0, 0.0, 30.0));
            if out.shedding_event {
                shed_seen = true;
                assert!(out.accreted_mass_kg < last_mass, "mass should drop on shedding");
                assert!(out.flameout_risk_frac >= 0.4);
                break;
            }
            last_mass = out.accreted_mass_kg;
        }
        assert!(shed_seen, "expected at least one shedding event over a sustained encounter");
    }

    #[test]
    fn flow_capacity_loss_never_exceeds_one() {
        let mut s = IceCrystalIcingState::new();
        let mut out = s.step(&inputs(8.0, 0.0, 1.0));
        for _ in 0..1000 {
            out = s.step(&inputs(8.0, 0.0, 30.0));
            assert!(out.flow_capacity_loss_frac <= 1.0);
        }
    }
}
