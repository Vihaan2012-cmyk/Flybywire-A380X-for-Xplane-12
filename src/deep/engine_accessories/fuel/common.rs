//! Shared Jet A-1 fuel properties used across the fuel-system models in this
//! directory (LP pump, filter, HP pump, FMU, manifold). Public reference
//! values only; nothing here is Trent-900-specific.

/// Nominal Jet A-1 density at 15 C, kg/m^3. The DEF STAN 91-091 / ASTM D1655
/// specification band is 775-840 kg/m^3 at 15 C; 800 kg/m^3 is the
/// commonly-quoted mid-band nominal used for aviation fuel-system sizing.
pub const FUEL_DENSITY_KG_M3: f64 = 800.0;

/// Specific heat, J/(kg K). Typical Jet A-1 figure (matches
/// `physics::engine::oil`'s own `FUEL_CP`, CRC Handbook of Aviation Fuel
/// Properties).
pub const FUEL_CP_J_KGK: f64 = 2010.0;

/// Kinematic viscosity, cSt, vs temperature: a simple Arrhenius-type fit
/// anchored on two commonly-cited Jet A-1 figures -- the DEF STAN 91-091
/// specification ceiling of 8 mm^2/s at -20 C, and a typical ~1.3 mm^2/s at
/// 20 C (CRC Handbook of Aviation Fuel Properties) -- rather than a
/// temperature-independent constant. **GENERIC** fit, not measured data:
/// real Jet A-1 viscosity-temperature curves are mildly non-Arrhenius, but
/// this captures the right order of magnitude and the right sign (colder
/// fuel is far more viscous) for the filter/pump models that use it.
pub fn viscosity_cst(temp_k: f64) -> f64 {
    const T_REF_K: f64 = 293.15;
    const V_REF_CST: f64 = 1.3;
    // Solved from the two anchor points above: k = ln(8.0/1.3) / 60.0 K.
    const K_PER_K: f64 = 0.030_295;
    let t = temp_k.clamp(200.0, 400.0);
    V_REF_CST * (-K_PER_K * (t - T_REF_K)).exp()
}

/// Fuel vapour pressure, Pa, vs temperature. Jet A-1's Reid vapour pressure
/// is very low (well under atmospheric) at normal fuel temperatures, unlike
/// gasoline; this is a **GENERIC** exponential rise anchored on the
/// commonly-cited order of magnitude (a few hundred Pa near 20 C, low kPa by
/// 60 C) used only to give the LP pump's cavitation (NPSH) check a
/// physically-reasonable, temperature-sensitive threshold -- no
/// Trent-900-specific fuel spec is public or needed here.
pub fn vapour_pressure_pa(temp_k: f64) -> f64 {
    const T_REF_K: f64 = 293.15;
    const P_REF_PA: f64 = 300.0;
    const K_PER_K: f64 = 0.06;
    let t = temp_k.clamp(200.0, 400.0);
    P_REF_PA * (K_PER_K * (t - T_REF_K)).exp()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn viscosity_matches_its_two_anchor_points() {
        assert!((viscosity_cst(253.15) - 8.0).abs() < 0.05);
        assert!((viscosity_cst(293.15) - 1.3).abs() < 0.01);
    }

    #[test]
    fn viscosity_rises_as_fuel_cools() {
        assert!(viscosity_cst(233.15) > viscosity_cst(293.15));
    }

    #[test]
    fn vapour_pressure_rises_with_temperature_and_stays_positive() {
        assert!(vapour_pressure_pa(200.0) > 0.0);
        assert!(vapour_pressure_pa(330.0) > vapour_pressure_pa(290.0));
    }
}
