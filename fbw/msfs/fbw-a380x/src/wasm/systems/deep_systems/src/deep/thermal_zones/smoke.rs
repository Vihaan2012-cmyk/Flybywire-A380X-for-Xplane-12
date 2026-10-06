pub fn concentration_kg_per_kg(smoke_kg: f64, air_mass_kg: f64) -> f64 {
    if air_mass_kg <= 0.0 {
        return 0.0;
    }
    (smoke_kg / air_mass_kg).max(0.0)
}

pub fn advected_smoke_flux_kg_s(flow_kg_s: f64, source_concentration_kg_per_kg: f64) -> f64 {
    flow_kg_s.max(0.0) * source_concentration_kg_per_kg.max(0.0)
}

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
