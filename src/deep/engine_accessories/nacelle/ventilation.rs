//! Nacelle ventilation: the airflow through the fan-cowl/core-cowl
//! compartments that carries away accessory-gearbox heat and, just as
//! importantly, keeps any leaked fuel or oil vapour below a flammable
//! concentration -- the actual reason certification requires a minimum
//! nacelle ventilation rate (14 CFR/CS-25 nacelle fire-protection
//! requirements call for ventilation adequate to preclude a hazardous
//! accumulation of flammable vapours). Two independent physical drivers
//! combine additively: ram air, which grows with the square of airspeed
//! (dynamic pressure) through fixed NACA-type inlet/outlet scoops, and an
//! eductor (venturi) driven by a small bleed of hot engine air that
//! entrains cooling air even with the aircraft stationary, when ram alone
//! gives nothing. A blocked scoop or a jammed/leaking eductor line reduces
//! whichever term it acts on; below the certification-style minimum this
//! module flags a vapour-accumulation risk as its documented output for a
//! fire/ice or performance consumer, rather than silently doing nothing.
//!
//! No Trent-900/A380 nacelle ventilation figures are public. The ram and
//! eductor coefficients and the minimum-flow threshold are **GENERIC**,
//! sized so a healthy nacelle comfortably clears the threshold in both
//! cruise (ram-dominated) and ground idle (eductor-dominated) conditions.

/// Ram contribution: `k_ram * dynamic_pressure_pa` (**GENERIC**, an
/// effective scoop area/discharge-coefficient product).
const RAM_COEFF_KG_S_PER_PA: f64 = 4.0e-4;
/// Eductor contribution: entrained flow per unit of motive bleed flow
/// (**GENERIC**, a typical ejector entrainment ratio for a small motive
/// flow).
const EDUCTOR_ENTRAINMENT_RATIO: f64 = 3.0;
/// Motive bleed flow feeding the eductor while the engine runs, kg/s
/// (**GENERIC**, a small tap, much smaller than the handling bleeds).
const EDUCTOR_MOTIVE_KG_S: f64 = 0.05;
/// Minimum ventilation flow below which vapour could accumulate
/// (**GENERIC**, sized so both driving terms above clear it with margin
/// when healthy).
const MIN_SAFE_FLOW_KG_S: f64 = 0.1;

#[derive(Clone, Copy, Debug, Default)]
pub struct VentilationFaults {
    /// Inlet/outlet scoop blocked (ice, debris), 0 clear .. 1 fully
    /// blocked: derates the ram term.
    pub scoop_blockage: f64,
    /// Eductor line blocked or leaking before reaching the venturi, 0
    /// healthy .. 1 fully lost: derates the eductor term.
    pub eductor_blockage: f64,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct VentilationState {
    pub flow_kg_s: f64,
    pub vapour_accumulation_risk: bool,
}

/// One evaluation (no internal state -- purely a function of current flight
/// condition and engine-running status, like `rotor_dynamics::bearing`).
/// `dynamic_pressure_pa` is `0.5 * rho * V^2` at the nacelle; `engine_running`
/// gates the eductor's motive flow.
pub fn step(dynamic_pressure_pa: f64, engine_running: bool, faults: &VentilationFaults) -> VentilationState {
    let scoop_open = 1.0 - faults.scoop_blockage.clamp(0.0, 1.0);
    let eductor_open = 1.0 - faults.eductor_blockage.clamp(0.0, 1.0);

    let ram_kg_s = RAM_COEFF_KG_S_PER_PA * dynamic_pressure_pa.max(0.0) * scoop_open;
    let eductor_kg_s = if engine_running { EDUCTOR_MOTIVE_KG_S * EDUCTOR_ENTRAINMENT_RATIO * eductor_open } else { 0.0 };
    let flow_kg_s = ram_kg_s + eductor_kg_s;

    VentilationState { flow_kg_s, vapour_accumulation_risk: flow_kg_s < MIN_SAFE_FLOW_KG_S }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn stationary_engine_off_gives_no_flow_and_flags_risk_no_nan() {
        let s = step(0.0, false, &VentilationFaults::default());
        assert_eq!(s.flow_kg_s, 0.0);
        assert!(s.vapour_accumulation_risk);
        assert!(!s.flow_kg_s.is_nan());
    }

    #[test]
    fn engine_running_on_the_ground_clears_the_minimum_via_the_eductor_alone() {
        let s = step(0.0, true, &VentilationFaults::default());
        assert!(s.flow_kg_s >= MIN_SAFE_FLOW_KG_S);
        assert!(!s.vapour_accumulation_risk);
    }

    #[test]
    fn cruise_dynamic_pressure_clears_the_minimum_via_ram_alone() {
        // ~250 kt EAS at low altitude, dynamic pressure roughly a few kPa.
        let s = step(5_000.0, false, &VentilationFaults::default());
        assert!(s.flow_kg_s >= MIN_SAFE_FLOW_KG_S);
    }

    #[test]
    fn a_fully_blocked_eductor_on_the_ground_reintroduces_the_risk() {
        let s = step(0.0, true, &VentilationFaults { eductor_blockage: 1.0, ..Default::default() });
        assert!(s.vapour_accumulation_risk);
        assert_eq!(s.flow_kg_s, 0.0);
    }

    #[test]
    fn a_fully_blocked_scoop_in_cruise_still_gets_eductor_flow_if_running() {
        let s = step(5_000.0, true, &VentilationFaults { scoop_blockage: 1.0, ..Default::default() });
        assert!(s.flow_kg_s > 0.0);
        assert!(s.flow_kg_s < step(5_000.0, true, &VentilationFaults::default()).flow_kg_s);
    }
}
