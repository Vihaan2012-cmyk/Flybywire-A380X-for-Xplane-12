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

/// Kinematic viscosity, mm^2/s (= cSt), vs temperature, by the Walther
/// relation (ASTM D341, the standard viscosity-temperature relation for
/// petroleum liquids):
///
///   `log10(log10(nu + 0.7)) = A - B * log10(T_kelvin)`
///
/// anchored on the two published Jet A-1 figures this model has to sit
/// between:
///
/// - **8.0 mm^2/s at -20 C** (253.15 K) -- the DEF STAN 91-091 / ASTM D1655
///   specification ceiling, the coldest viscosity the specification pins
///   down;
/// - **1.25 mm^2/s at 40 C** (313.15 K) -- the typical Jet A-1 figure (CRC
///   *Handbook of Aviation Fuel Properties*, Report No. 635).
///
/// `A` and `B` are solved from those two points by hand, not fitted:
///
///   `Z  = log10(log10(nu + 0.7))`
///   `Z1 = log10(log10(8.70)) = -0.0270943`,  `log10(253.15) = 2.4033779`
///   `Z2 = log10(log10(1.95)) = -0.5375502`,  `log10(313.15) = 2.4957524`
///   `B  = (Z1 - Z2)/(log10 T2 - log10 T1) = 0.5104559/0.0923745 = 5.5259400`
///   `A  = Z1 + B*log10 T1 = -0.0270943 + 13.2809222 = 13.2538279`
///
/// The Walther form replaces an earlier single-exponential (Arrhenius) fit
/// which, besides mis-deriving its own constant, cannot hold both anchors
/// *and* the curvature between them: fitted to these two points, a plain
/// exponential runs about a third low through the middle of the range,
/// which is exactly where the filter and pump models operate.
///
/// **This is therefore a specification-worst-case curve, not a typical
/// batch.** Its cold anchor is the number a batch is allowed to be no
/// worse than, so off the anchors it reads about 1.92 mm^2/s at 20 C,
/// against roughly 1.7 mm^2/s often quoted for a typical batch -- about
/// 13% viscous. Searched for a citable *typical* (as opposed to limiting)
/// Jet A-1 kinematic viscosity at a cold anchor -- CGSB 3.23 / ASTM D1655 /
/// DEF STAN 91-091 tables, CRC Report 635, and fuel suppliers' own data
/// sheets: every public source pins only the -20 C maximum of 8.0 mm^2/s,
/// so no typical cold-end figure is available to re-anchor on. The ~1.7
/// mm^2/s at 20 C used as the comparison above is likewise a commonly
/// repeated figure rather than a cited one.
///
/// Anchoring on the ceiling is left in place deliberately: for the filter
/// (`filter.rs` scales its element drop by the viscosity ratio) and for the
/// pumps, a high viscosity is the demanding direction, so the error runs
/// toward predicting filter differential pressure and pump load a little
/// early rather than a little late.
pub fn viscosity_cst(temp_k: f64) -> f64 {
    const WALTHER_A: f64 = 13.253_827_9;
    const WALTHER_B: f64 = 5.525_940_0;
    let t = temp_k.clamp(200.0, 400.0);
    10f64.powf(10f64.powf(WALTHER_A - WALTHER_B * t.log10())) - 0.7
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
        // DEF STAN 91-091 / ASTM D1655 ceiling at -20 C ...
        assert!((viscosity_cst(253.15) - 8.0).abs() < 0.01, "{}", viscosity_cst(253.15));
        // ... and the typical CRC figure at 40 C.
        assert!((viscosity_cst(313.15) - 1.25).abs() < 0.01, "{}", viscosity_cst(313.15));
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
