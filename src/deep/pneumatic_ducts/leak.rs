//! Per-duct-section leak and rupture: an orifice from the section's own gas
//! to its surrounding airframe zone, using the same `orifice_mass_flow_kg_s`
//! law every other flow path in this module uses (`duct.rs`).
//!
//! Two severities of the same physical fault, both continuous 0.0..1.0:
//! - **leak**: a crack/seal failure. Orifice area scales up to a small
//!   fraction of the duct's own cross-section
//!   (`LEAK_AREA_FRACTION_OF_BORE`, **GENERIC** -- sized, like
//!   `physics::bays::LEAK_AREA_MAX_M2`'s own derivation note, "large enough
//!   to dominate the local heat balance ... not so large it instantly
//!   saturates" while staying well below a full severance).
//! - **rupture**: the duct parts entirely. Orifice area scales up to the
//!   section's own full bore area (`DuctSection::full_bore_area_m2`), i.e.
//!   `rupture == 1.0` is a complete severance.
//!
//! **Energy conservation bound.** The maximum heat the escaping gas can
//! give up to the zone is its own sensible enthalpy above the zone's
//! temperature, `mdot * cp * (T_duct - T_zone)` -- the same form
//! `physics::bays::leak_heat_w` already uses for its interim fixed-condition
//! placeholder (`bays.rs:543-545`). This module reuses that *bound* but adds
//! a real **effectiveness** term (matching this crate's existing heat-
//! exchanger-effectiveness convention, e.g. `physics::engine::oil`'s
//! `FCOC_EFFECTIVENESS`/`ACOC_EFFECTIVENESS`): a small pinhole leak's air
//! only partially thermalises against the zone before dilution/ventilation
//! carries it off, while a rupture fires a concentrated, still-near-sonic
//! jet that impinges directly on nearby structure -- real bleed-duct-
//! failure investigations (FAA airworthiness directives on wide-body bleed
//! duct/manifold failures, and public NTSB/AAIB reports on wing-root bleed
//! leaks damaging adjacent structure) consistently describe a rupture's
//! concentrated, fast, localised heat/damage against a diffuse leak's slow
//! bay heating -- because a jet in contact with a surface transfers heat by
//! forced convection at a far higher coefficient than air simply diluting
//! into a compartment's bulk air. A higher heat-transfer coefficient cannot
//! extract *more* energy than the flow carries (that would violate
//! conservation); it only extracts a *larger fraction of the same bound*
//! before the gas leaves the zone -- exactly what "effectiveness" already
//! means for every other heat exchanger in this crate. Never invents energy
//! beyond the bound above.

use super::duct::{orifice_mass_flow_kg_s, DuctSection, DuctSectionFaults, CP_AIR_J_KG_K};

/// GENERIC: a leak's full-severity crack area as a fraction of the duct's
/// own bore area. Chosen so a full-severity *leak* (a seal/flange crack)
/// stays a full order of magnitude below a full-severity *rupture*'s flow
/// (`= 1.0 x` bore area) -- matching how "leak" (detectable, manageable,
/// crew can often complete the flight) and "rupture" (immediate isolation)
/// are operationally distinct severities, not the same fault scaled
/// linearly to 100%.
pub const LEAK_AREA_FRACTION_OF_BORE: f64 = 0.02;
/// Orifice discharge coefficient: matches `physics::bays::
/// LEAK_DISCHARGE_COEFFICIENT` / `docs/physics/air.md`'s cited duct/valve
/// figure, reused rather than inventing a second one for the same class of
/// opening.
pub const LEAK_DISCHARGE_COEFFICIENT: f64 = 0.65;
/// Below this rupture magnitude the escaping jet is still small/diffuse
/// enough to treat as ordinary dilution rather than a concentrated jet.
/// **GENERIC**.
const RUPTURE_JET_ONSET: f64 = 0.3;
/// **GENERIC**: fraction of the enthalpy bound a slow, diffuse leak
/// actually delivers to the zone before ventilation/dilution carries the
/// rest away (never all of it -- the zone is not a closed calorimeter).
const DIFFUSE_EFFECTIVENESS: f64 = 0.3;
/// **GENERIC**: fraction of the enthalpy bound a concentrated, impinging
/// rupture jet delivers -- high, matching the well-established fact that
/// stagnation-region jet-impingement heat transfer coefficients are much
/// higher than parallel/diffuse flow at the same conditions (general
/// jet-impingement heat-transfer literature, e.g. Martin's correlation for
/// impinging jet Nusselt numbers), so most of the available enthalpy is
/// transferred before the jet loses coherence.
const IMPINGEMENT_EFFECTIVENESS: f64 = 0.9;

/// A jet's cross-section spreads somewhat past the orifice itself by the
/// time it reaches nearby structure/wiring at a short standoff distance.
/// **GENERIC**: no public figure exists for A380 bleed-bay geometry;
/// `3x` the orifice area is a representative, conservative (i.e. flux-
/// diluting, not flux-concentrating) order-of-magnitude spread for a
/// free jet a few orifice-diameters downstream (general free-jet spreading
/// behaviour, e.g. any jet-impingement heat-transfer reference).
const JET_IMPACT_AREA_MULTIPLIER: f64 = 3.0;

/// One tick's result of a duct section's leak/rupture faults.
#[derive(Clone, Copy, Debug, Default)]
pub struct LeakResult {
    /// Mass flow leaving the duct through the fault, kg/s (>= 0).
    pub mass_flow_kg_s: f64,
    /// Heat delivered to the zone this tick, W (>= 0; this module never
    /// models the zone being hotter than the duct through a leak path).
    pub heat_to_zone_w: f64,
    /// True once `rupture` is past the jet-impingement onset -- a caller
    /// (e.g. a future fire/structural-damage model) can use this to flag
    /// "this is a jet, not a diffuse leak" without recomputing the ramp.
    pub is_impinging_jet: bool,
    /// **Structural/wiring damage interface.** The same `heat_to_zone_w`
    /// energy, expressed as a local flux (W/m^2) at the jet's own
    /// impingement spot rather than spread across the whole zone's bulk
    /// air -- zero below the jet-impingement onset (a diffuse leak has no
    /// concentrated spot to report). This does not add energy beyond
    /// `heat_to_zone_w` (conservation, module docs): it is the same total
    /// watts, reported at the spatial concentration a wiring-harness/
    /// structural-burn-through damage model actually needs (a real
    /// impinging bleed leak is documented, e.g. in FAA ADs on wide-body
    /// bleed duct failures, as burning through *local* structure/looms
    /// long before it noticeably heats the whole surrounding bay -- a
    /// per-zone bulk-average watt figure alone cannot drive that kind of
    /// damage model).
    pub jet_impact_flux_w_m2: f64,
}

/// Evaluate (but do not apply) this tick's leak/rupture flow and heat for
/// `section`, given the zone's own current static air/ambient conditions.
/// The caller (`network.rs`) is responsible for removing `mass_flow_kg_s *
/// dt_s` from `section.gas` (via `DuctVolume::add_mass`, negative) and for
/// crediting `heat_to_zone_w` to the zone's own thermal model.
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
        DuctSection::new("PYLON_1", 0.1, 0.08, 4.0, 300_000.0, 473.15) // ~200 C bleed
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
        let s = DuctSection::new("WING_ROOT", 0.1, 0.08, 4.0, 300_000.0, 260.0); // colder than the zone
        let r = step(&s, 40_000.0, 288.15, &DuctSectionFaults { leak: 1.0, ..Default::default() });
        assert_eq!(r.heat_to_zone_w, 0.0);
        assert!(r.mass_flow_kg_s.is_finite());
    }
}
