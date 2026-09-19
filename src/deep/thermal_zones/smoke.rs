//! Smoke transport (task step 5): a zone's smoke content is tracked as an
//! absolute mass (kg) in its well-mixed air node, advected between zones
//! exactly like the enthalpy exchange `network::ThermalNetwork::step`
//! already performs for temperature -- the same "flow carries whatever
//! concentration its source currently holds" continuously-stirred-tank
//! (CSTR) simplification standard multizone contaminant-transport network
//! models use (e.g. ASHRAE Handbook -- Fundamentals, multizone airflow/
//! contaminant transport chapter; also the standard simplifying
//! assumption behind aircraft smoke-detector "time to detect" estimates).
//! For a `flow_kg_s` of air moving at concentration `c` (kg smoke / kg
//! air), `flow_kg_s * c` kg/s of smoke moves with it.

/// Smoke mass fraction (kg smoke / kg air) in a well-mixed zone of
/// `air_mass_kg` holding `smoke_kg` of smoke. Never negative; a
/// non-positive air mass (should not happen -- `Zone::new` floors it)
/// reads as clean air rather than dividing by zero.
pub fn concentration_kg_per_kg(smoke_kg: f64, air_mass_kg: f64) -> f64 {
    if air_mass_kg <= 0.0 {
        return 0.0;
    }
    (smoke_kg / air_mass_kg).max(0.0)
}

/// Smoke mass flux (kg/s) carried by a ventilation flow at
/// `source_concentration_kg_per_kg`. Negative flow is treated as zero
/// (no flow, no advection) rather than reversing direction; callers that
/// need a two-way link add two of these instead.
pub fn advected_smoke_flux_kg_s(flow_kg_s: f64, source_concentration_kg_per_kg: f64) -> f64 {
    flow_kg_s.max(0.0) * source_concentration_kg_per_kg.max(0.0)
}

/// One tick of a zone's own smoke mass balance: production (fire/overheat
/// injection) plus the net advected flux already computed by the caller
/// (inflow from a reference minus this zone's own outflow, see
/// `network::ThermalNetwork::substep`), minus a settling/filtration loss
/// proportional to standing mass (`decay_per_s` -- an ECS recirculation
/// filter or simple gravitational settling; **GENERIC**, 0 unless a
/// caller sets one). Floored at zero: smoke mass cannot go negative
/// (matches every other physical quantity's floor convention in this
/// crate, e.g. `physics::tyre`'s leak masses).
pub fn step_smoke_kg(smoke_kg: f64, produced_kg_s: f64, net_flux_kg_s: f64, decay_per_s: f64, dt_s: f64) -> f64 {
    if dt_s <= 0.0 {
        return smoke_kg.max(0.0);
    }
    let decay_kg_s = decay_per_s.max(0.0) * smoke_kg.max(0.0);
    let raw = smoke_kg + (produced_kg_s.max(0.0) + net_flux_kg_s - decay_kg_s) * dt_s;
    raw.max(0.0)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn concentration_is_zero_for_zero_or_negative_air_mass() {
        assert_eq!(concentration_kg_per_kg(1.0, 0.0), 0.0);
        assert_eq!(concentration_kg_per_kg(1.0, -5.0), 0.0);
    }

    #[test]
    fn concentration_scales_linearly_with_smoke_mass() {
        let c1 = concentration_kg_per_kg(0.1, 10.0);
        let c2 = concentration_kg_per_kg(0.2, 10.0);
        assert!((c2 / c1 - 2.0).abs() < 1e-9);
    }

    #[test]
    fn advected_flux_is_zero_for_reverse_or_zero_flow() {
        assert_eq!(advected_smoke_flux_kg_s(-1.0, 0.5), 0.0);
        assert_eq!(advected_smoke_flux_kg_s(0.0, 0.5), 0.0);
        assert!(advected_smoke_flux_kg_s(1.0, 0.5) > 0.0);
    }

    #[test]
    fn step_smoke_accumulates_production_over_time() {
        let mut kg = 0.0;
        for _ in 0..100 {
            kg = step_smoke_kg(kg, 0.01, 0.0, 0.0, 1.0);
        }
        assert!((kg - 1.0).abs() < 1e-9);
    }

    #[test]
    fn step_smoke_never_goes_negative_even_with_large_outflow() {
        let kg = step_smoke_kg(0.01, 0.0, -10.0, 0.0, 1.0);
        assert_eq!(kg, 0.0);
    }

    #[test]
    fn step_smoke_decay_reduces_standing_mass() {
        let mut kg = 1.0;
        for _ in 0..100 {
            kg = step_smoke_kg(kg, 0.0, 0.0, 0.05, 1.0);
        }
        assert!(kg < 1.0 && kg > 0.0, "decay should reduce but not zero out a finite-rate exponential decay: {kg}");
    }

    #[test]
    fn step_smoke_at_zero_dt_is_a_no_op() {
        assert_eq!(step_smoke_kg(0.5, 1.0, 1.0, 1.0, 0.0), 0.5);
    }
}
