use super::duct::{orifice_mass_flow_kg_s, DuctSection, DuctSectionFaults, CP_AIR_J_KG_K};

pub const LEAK_AREA_FRACTION_OF_BORE: f64 = 0.02;
pub const LEAK_DISCHARGE_COEFFICIENT: f64 = 0.65;
const RUPTURE_JET_ONSET: f64 = 0.3;
const DIFFUSE_EFFECTIVENESS: f64 = 0.3;
const IMPINGEMENT_EFFECTIVENESS: f64 = 0.9;

const JET_IMPACT_AREA_MULTIPLIER: f64 = 3.0;

#[derive(Clone, Copy, Debug, Default)]
pub struct LeakResult {
    pub mass_flow_kg_s: f64,
    pub heat_to_zone_w: f64,
    pub is_impinging_jet: bool,
    pub jet_impact_flux_w_m2: f64,
}

pub fn step(section: &DuctSection, zone_ambient_pa: f64, zone_air_k: f64, faults: &DuctSectionFaults) -> LeakResult {
    let bore = section.full_bore_area_m2();
    let leak_area = bore * LEAK_AREA_FRACTION_OF_BORE * faults.leak.clamp(0.0, 1.0);
    let rupture = faults.rupture.clamp(0.0, 1.0);
    let rupture_area = bore * rupture;
    let area = leak_area + rupture_area;
    if area <= 0.0 {
        return LeakResult::default();
    }

    let mdot = orifice_mass_flow_kg_s(LEAK_DISCHARGE_COEFFICIENT, area, section.gas.pressure_pa(), section.gas.temp_k(), zone_ambient_pa);

    let enthalpy_bound_w = mdot * CP_AIR_J_KG_K * (section.gas.temp_k() - zone_air_k).max(0.0);
    let jet_progress = ((rupture - RUPTURE_JET_ONSET) / (1.0 - RUPTURE_JET_ONSET)).clamp(0.0, 1.0);
    let effectiveness = DIFFUSE_EFFECTIVENESS + (IMPINGEMENT_EFFECTIVENESS - DIFFUSE_EFFECTIVENESS) * jet_progress;
    let heat_to_zone_w = enthalpy_bound_w * effectiveness;
    let is_impinging_jet = rupture > RUPTURE_JET_ONSET;
    let jet_impact_flux_w_m2 = if is_impinging_jet {
        heat_to_zone_w / (area * JET_IMPACT_AREA_MULTIPLIER).max(1e-9)
    } else {
        0.0
    };

    LeakResult {
        mass_flow_kg_s: mdot,
        heat_to_zone_w,
        is_impinging_jet,
        jet_impact_flux_w_m2,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn hot_section() -> DuctSection {
        DuctSection::new("PYLON_1", 0.1, 0.08, 4.0, 300_000.0, 473.15)
    }

    #[test]
    fn no_fault_means_no_leak() {
        let s = hot_section();
        let r = step(&s, 40_000.0, 288.15, &DuctSectionFaults::default());
        assert_eq!(r.mass_flow_kg_s, 0.0);
        assert_eq!(r.heat_to_zone_w, 0.0);
    }

    #[test]
    fn a_bigger_leak_flows_and_heats_more() {
        let s = hot_section();
        let small = step(&s, 40_000.0, 288.15, &DuctSectionFaults { leak: 0.2, ..Default::default() });
        let big = step(&s, 40_000.0, 288.15, &DuctSectionFaults { leak: 1.0, ..Default::default() });
        assert!(big.mass_flow_kg_s > small.mass_flow_kg_s);
        assert!(big.heat_to_zone_w > small.heat_to_zone_w);
    }

    #[test]
    fn rupture_flows_far_more_than_a_full_severity_leak() {
        let s = hot_section();
        let leak = step(&s, 40_000.0, 288.15, &DuctSectionFaults { leak: 1.0, ..Default::default() });
        let rupture = step(&s, 40_000.0, 288.15, &DuctSectionFaults { rupture: 1.0, ..Default::default() });
        assert!(rupture.mass_flow_kg_s > leak.mass_flow_kg_s * 10.0, "a full duct severance must dwarf a full-severity crack");
    }

    #[test]
    fn heat_never_exceeds_the_enthalpy_bound() {
        let s = hot_section();
        let r = step(&s, 40_000.0, 288.15, &DuctSectionFaults { rupture: 1.0, ..Default::default() });
        let bound = r.mass_flow_kg_s * CP_AIR_J_KG_K * (473.15 - 288.15);
        assert!(r.heat_to_zone_w <= bound + 1e-6, "effectiveness must not manufacture energy beyond the enthalpy bound");
    }

    #[test]
    fn a_rupture_is_flagged_as_an_impinging_jet_and_transfers_heat_more_effectively_per_kg_s() {
        let s = hot_section();
        let leak = step(&s, 40_000.0, 288.15, &DuctSectionFaults { leak: 1.0, ..Default::default() });
        let rupture = step(&s, 40_000.0, 288.15, &DuctSectionFaults { rupture: 1.0, ..Default::default() });
        assert!(!leak.is_impinging_jet);
        assert!(rupture.is_impinging_jet);
        assert_eq!(leak.jet_impact_flux_w_m2, 0.0, "a diffuse leak has no concentrated spot to report");
        assert!(rupture.jet_impact_flux_w_m2 > 0.0, "an impinging rupture must report a real local flux");
        let leak_w_per_kg_s = leak.heat_to_zone_w / leak.mass_flow_kg_s;
        let rupture_w_per_kg_s = rupture.heat_to_zone_w / rupture.mass_flow_kg_s;
        assert!(rupture_w_per_kg_s > leak_w_per_kg_s, "impingement effectiveness must exceed diffuse effectiveness per unit flow");
    }

    #[test]
    fn jet_impact_flux_reports_the_same_total_energy_not_extra() {
        let s = hot_section();
        let r = step(&s, 40_000.0, 288.15, &DuctSectionFaults { rupture: 1.0, ..Default::default() });
        let impact_area = s.full_bore_area_m2() * JET_IMPACT_AREA_MULTIPLIER;
        let implied_total_w = r.jet_impact_flux_w_m2 * impact_area;
        assert!((implied_total_w - r.heat_to_zone_w).abs() < 1e-6, "flux * impact area must reconstruct the exact same heat_to_zone_w, not a different (invented) energy total");
    }

    #[test]
    fn a_duct_colder_than_its_zone_gives_no_negative_heat() {
        let s = DuctSection::new("WING_ROOT", 0.1, 0.08, 4.0, 300_000.0, 260.0);
        let r = step(&s, 40_000.0, 288.15, &DuctSectionFaults { leak: 1.0, ..Default::default() });
        assert_eq!(r.heat_to_zone_w, 0.0);
        assert!(r.mass_flow_kg_s.is_finite());
    }
}
