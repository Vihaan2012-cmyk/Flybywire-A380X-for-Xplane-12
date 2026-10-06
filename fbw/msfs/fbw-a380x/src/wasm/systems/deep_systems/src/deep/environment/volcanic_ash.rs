#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AshConcentrationClass {
    None,
    Low,
    Medium,
    High,
}

pub fn classify(concentration_mg_m3: f64) -> AshConcentrationClass {
    if concentration_mg_m3 <= 0.0 {
        AshConcentrationClass::None
    } else if concentration_mg_m3 < 2.0 {
        AshConcentrationClass::Low
    } else if concentration_mg_m3 < 4.0 {
        AshConcentrationClass::Medium
    } else {
        AshConcentrationClass::High
    }
}

const MELT_START_C: f64 = 1000.0;
const MELT_FULL_C: f64 = 1200.0;

fn molten_fraction(gas_temp_c: f64) -> f64 {
    ((gas_temp_c - MELT_START_C) / (MELT_FULL_C - MELT_START_C)).clamp(0.0, 1.0)
}

const DEPOSITION_EFFICIENCY: f64 = 0.5;
const FULL_BLOCKAGE_MASS_KG: f64 = 2.0;

const EROSION_RATE_SCALE: f64 = 4.0e-8;
const EROSION_VELOCITY_EXPONENT: f64 = 2.5;
const MAX_EROSION_EFFICIENCY_LOSS: f64 = 0.25;

const WINDSHIELD_ABRASION_RATE: f64 = 6.0e-9;
const PITOT_BLOCKAGE_RATE: f64 = 0.02;
const ODOUR_PER_MG_M3: f64 = 0.3;

#[derive(Clone, Copy, Debug)]
pub struct AshInputs {
    pub concentration_mg_m3: f64,
    pub core_mass_flow_kg_s: f64,
    pub ngv_gas_temp_c: f64,
    pub compressor_velocity_ms: f64,
    pub tas_ms: f64,
    pub dt_s: f64,
}

#[derive(Clone, Copy, Debug)]
pub struct AshOutputs {
    pub class: AshConcentrationClass,
    pub flow_capacity_loss_frac: f64,
    pub compressor_efficiency_loss_frac: f64,
    pub windshield_visibility_loss_frac: f64,
    pub pitot_blockage_frac: f64,
    pub cabin_odor_intensity: f64,
    pub flameout_risk_frac: f64,
    pub relight_possible: bool,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct VolcanicAshState {
    accreted_mass_kg: f64,
    compressor_efficiency_loss_frac: f64,
    windshield_visibility_loss_frac: f64,
    pitot_blockage_frac: f64,
}

impl VolcanicAshState {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn step(&mut self, i: &AshInputs) -> AshOutputs {
        let dt = i.dt_s.max(0.0);
        let concentration_kg_m3 = (i.concentration_mg_m3.max(0.0)) * 1e-6;
        let class = classify(i.concentration_mg_m3.max(0.0));

        let ash_mass_flow_kg_s = concentration_kg_m3 * i.core_mass_flow_kg_s.max(0.0);
        let molten = molten_fraction(i.ngv_gas_temp_c);
        let deposit_rate_kg_s = ash_mass_flow_kg_s * molten * DEPOSITION_EFFICIENCY;
        self.accreted_mass_kg += deposit_rate_kg_s * dt;
        let flow_capacity_loss_frac = (self.accreted_mass_kg / FULL_BLOCKAGE_MASS_KG).min(1.0);

        let solid_mass_flow_kg_s = ash_mass_flow_kg_s * (1.0 - molten);
        let erosion_rate = EROSION_RATE_SCALE * solid_mass_flow_kg_s * i.compressor_velocity_ms.max(0.0).powf(EROSION_VELOCITY_EXPONENT);
        self.compressor_efficiency_loss_frac = (self.compressor_efficiency_loss_frac + erosion_rate * dt).min(MAX_EROSION_EFFICIENCY_LOSS);

        self.windshield_visibility_loss_frac = (self.windshield_visibility_loss_frac + WINDSHIELD_ABRASION_RATE * concentration_kg_m3 * i.tas_ms.max(0.0) * dt).min(1.0);

        self.pitot_blockage_frac = (self.pitot_blockage_frac + PITOT_BLOCKAGE_RATE * concentration_kg_m3 * 1000.0 * dt).min(1.0);

        let cabin_odor_intensity = (i.concentration_mg_m3.max(0.0) * ODOUR_PER_MG_M3).min(1.0);

        let flameout_risk_frac = (i.concentration_mg_m3.max(0.0) / 4.0 * 0.5 + flow_capacity_loss_frac * 0.5).min(1.0);

        AshOutputs {
            class,
            flow_capacity_loss_frac,
            compressor_efficiency_loss_frac: self.compressor_efficiency_loss_frac,
            windshield_visibility_loss_frac: self.windshield_visibility_loss_frac,
            pitot_blockage_frac: self.pitot_blockage_frac,
            cabin_odor_intensity,
            flameout_risk_frac,
            relight_possible: i.concentration_mg_m3 <= 0.01,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn inputs(concentration_mg_m3: f64, gas_temp_c: f64, dt_s: f64) -> AshInputs {
        AshInputs { concentration_mg_m3, core_mass_flow_kg_s: 15.0, ngv_gas_temp_c: gas_temp_c, compressor_velocity_ms: 200.0, tas_ms: 230.0, dt_s }
    }

    #[test]
    fn classification_matches_the_published_bands() {
        assert_eq!(classify(0.0), AshConcentrationClass::None);
        assert_eq!(classify(1.0), AshConcentrationClass::Low);
        assert_eq!(classify(3.0), AshConcentrationClass::Medium);
        assert_eq!(classify(5.0), AshConcentrationClass::High);
    }

    #[test]
    fn zero_concentration_causes_no_wear_no_nan_and_allows_relight() {
        let mut s = VolcanicAshState::new();
        let out = s.step(&inputs(0.0, 1500.0, 1.0));
        assert_eq!(out.flow_capacity_loss_frac, 0.0);
        assert_eq!(out.compressor_efficiency_loss_frac, 0.0);
        assert_eq!(out.cabin_odor_intensity, 0.0);
        assert!(out.relight_possible);
        assert!(!out.flameout_risk_frac.is_nan());
    }

    #[test]
    fn hotter_gas_glasses_faster_than_cooler_gas_at_the_same_concentration() {
        let mut hot = VolcanicAshState::new();
        let mut cool = VolcanicAshState::new();
        let hot_out = hot.step(&inputs(4.0, 1300.0, 60.0));
        let cool_out = cool.step(&inputs(4.0, 900.0, 60.0));
        assert!(hot_out.flow_capacity_loss_frac > cool_out.flow_capacity_loss_frac);
        assert_eq!(cool_out.flow_capacity_loss_frac, 0.0, "below the melt threshold nothing glasses");
    }

    #[test]
    fn cooler_gas_erodes_the_compressor_faster_since_more_ash_stays_solid() {
        let mut hot = VolcanicAshState::new();
        let mut cool = VolcanicAshState::new();
        let hot_out = hot.step(&inputs(4.0, 1300.0, 60.0));
        let cool_out = cool.step(&inputs(4.0, 900.0, 60.0));
        assert!(cool_out.compressor_efficiency_loss_frac > hot_out.compressor_efficiency_loss_frac);
    }

    #[test]
    fn wear_accumulates_and_is_monotonic_over_time() {
        let mut s = VolcanicAshState::new();
        let a = s.step(&inputs(3.0, 1100.0, 30.0));
        let b = s.step(&inputs(3.0, 1100.0, 30.0));
        assert!(b.compressor_efficiency_loss_frac >= a.compressor_efficiency_loss_frac);
        assert!(b.windshield_visibility_loss_frac >= a.windshield_visibility_loss_frac);
        assert!(b.pitot_blockage_frac >= a.pitot_blockage_frac);
        assert!(b.flow_capacity_loss_frac >= a.flow_capacity_loss_frac);
    }

    #[test]
    fn pitot_blockage_and_erosion_saturate_and_never_exceed_their_ceilings() {
        let mut s = VolcanicAshState::new();
        let mut out = s.step(&inputs(10.0, 1300.0, 1.0));
        for _ in 0..2000 {
            out = s.step(&inputs(10.0, 1300.0, 60.0));
        }
        assert!(out.pitot_blockage_frac <= 1.0);
        assert!(out.compressor_efficiency_loss_frac <= MAX_EROSION_EFFICIENCY_LOSS + 1e-9);
        assert!(out.flow_capacity_loss_frac <= 1.0);
    }

    #[test]
    fn relight_is_possible_again_once_clear_even_with_damage_already_done() {
        let mut s = VolcanicAshState::new();
        let inside = s.step(&inputs(6.0, 1100.0, 120.0));
        assert!(!inside.relight_possible);
        let clear = s.step(&inputs(0.0, 400.0, 1.0));
        assert!(clear.relight_possible);
        assert!(clear.compressor_efficiency_loss_frac > 0.0);
        assert!(clear.flow_capacity_loss_frac > 0.0);
    }

    #[test]
    fn flameout_risk_rises_with_concentration_and_accumulated_blockage() {
        let mut fresh = VolcanicAshState::new();
        let mild = fresh.step(&inputs(1.0, 1300.0, 1.0));
        let mut worn = VolcanicAshState::new();
        worn.step(&inputs(8.0, 1300.0, 600.0));
        let severe = worn.step(&inputs(8.0, 1300.0, 1.0));
        assert!(severe.flameout_risk_frac > mild.flameout_risk_frac);
    }

    #[test]
    fn cabin_odour_scales_with_concentration_and_is_not_cumulative() {
        let mut s = VolcanicAshState::new();
        let first = s.step(&inputs(2.0, 1000.0, 60.0));
        let second = s.step(&inputs(2.0, 1000.0, 60.0));
        assert!((first.cabin_odor_intensity - second.cabin_odor_intensity).abs() < 1e-9);
        assert!(first.cabin_odor_intensity > 0.0);
    }
}
