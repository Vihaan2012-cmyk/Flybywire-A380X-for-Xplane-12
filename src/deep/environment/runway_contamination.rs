//! Runway contamination: friction coefficient by contaminant type and
//! depth (public EASA/FAA runway condition codes), and dynamic
//! hydroplaning speed.
//!
//! ## Sources
//! - Runway condition codes (RWYCC 0-6) and their qualitative braking-
//!   action terms (GOOD, GOOD TO MEDIUM, MEDIUM, MEDIUM TO POOR, POOR,
//!   NIL) are the public Runway Condition Assessment Matrix (RCAM), ICAO
//!   Doc 9981 (PANS-Aerodromes) / FAA AC 150/5200-30D, built from
//!   contaminant type, depth (the RCAM's own 3 mm / 1/8 in threshold
//!   between its "good" and "medium" bands) and, for compacted snow,
//!   outside air temperature (its -15 C threshold). The RCAM deliberately
//!   reports *qualitative* braking action rather than a numeric friction
//!   coefficient (ICAO found no consistent correlation between continuous-
//!   friction-measuring-equipment mu and actual braking performance on a
//!   contaminated runway) -- so `generic_mu` below assigns a `GENERIC`
//!   representative coefficient per band for use in a performance
//!   calculation, not a published number.
//! - Dynamic hydroplaning speed: Horne's NASA formula, `v_p (kt) = 9 *
//!   sqrt(tire pressure, psi)`, from NASA's tyre-hydroplaning research
//!   (NASA TN D-2056 and follow-on work); widely republished (e.g. a 50
//!   psi tyre hydroplanes at about 64 kt).
//! - The hydroplaning-onset transition (smoothly discounting friction over
//!   the last 30% of the approach to `v_p`) and the residual friction
//!   once hydroplaning is fully established are `GENERIC`.

/// A runway surface condition, matching the RCAM's own categories.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Contaminant {
    Dry,
    Frost,
    Water { depth_mm: f64 },
    Slush { depth_mm: f64 },
    DrySnow { depth_mm: f64 },
    WetSnow { depth_mm: f64 },
    CompactedSnow { oat_c: f64 },
    Ice,
    /// RCAM's "wet ice / water on top of compacted snow / dry or wet snow
    /// over ice": the NIL band, never reported in a FICON NOTAM as a
    /// runway condition code but retained here as the physical floor.
    WaterOverIceOrCompactedSnow,
}

/// RCAM's own 3 mm (1/8 in) good/medium depth threshold.
const DEPTH_THRESHOLD_MM: f64 = 3.0;
/// RCAM's own compacted-snow temperature threshold, C.
const COMPACTED_SNOW_COLD_THRESHOLD_C: f64 = -15.0;

pub fn runway_condition_code(c: Contaminant) -> u8 {
    match c {
        Contaminant::Dry => 6,
        Contaminant::Frost => 5,
        Contaminant::Water { depth_mm } | Contaminant::Slush { depth_mm } | Contaminant::DrySnow { depth_mm } | Contaminant::WetSnow { depth_mm } => {
            if depth_mm <= DEPTH_THRESHOLD_MM {
                5
            } else {
                3
            }
        }
        Contaminant::CompactedSnow { oat_c } => {
            if oat_c <= COMPACTED_SNOW_COLD_THRESHOLD_C {
                4
            } else {
                3
            }
        }
        Contaminant::Ice => 1,
        Contaminant::WaterOverIceOrCompactedSnow => 0,
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BrakingAction {
    Good,
    GoodToMedium,
    Medium,
    MediumToPoor,
    Poor,
    Nil,
}

pub fn braking_action(rwy_cc: u8) -> BrakingAction {
    match rwy_cc {
        6 | 5 => BrakingAction::Good,
        4 => BrakingAction::GoodToMedium,
        3 => BrakingAction::Medium,
        2 => BrakingAction::MediumToPoor,
        1 => BrakingAction::Poor,
        _ => BrakingAction::Nil,
    }
}

/// GENERIC representative dry-equivalent friction coefficient per RCAM
/// band (see module doc: the RCAM itself publishes no numeric mu).
fn generic_mu(rwy_cc: u8) -> f64 {
    match rwy_cc {
        6 => 0.40,
        5 => 0.38,
        4 => 0.33,
        3 => 0.28,
        2 => 0.22,
        1 => 0.16,
        _ => 0.05,
    }
}

fn has_fluid_film(c: Contaminant) -> bool {
    matches!(c, Contaminant::Water { .. } | Contaminant::Slush { .. } | Contaminant::WetSnow { .. } | Contaminant::WaterOverIceOrCompactedSnow)
}

/// Horne's NASA dynamic hydroplaning speed, knots, for a tyre pressure in
/// psi (see module doc).
pub fn hydroplane_speed_kt(tire_pressure_psi: f64) -> f64 {
    9.0 * tire_pressure_psi.max(0.0).sqrt()
}

/// GENERIC: friction remaining once dynamic hydroplaning is fully
/// established (the tyre rides on the fluid film, essentially unbraked).
const MU_HYDROPLANE_RESIDUAL: f64 = 0.05;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RunwayFrictionOutput {
    pub contaminant: Contaminant,
    pub rwy_cc: u8,
    pub braking_action: BrakingAction,
    pub mu_effective: f64,
    pub hydroplaning: bool,
    pub hydroplane_speed_kt: f64,
}

/// The braking side's main entry point: current contaminant, this gear's
/// tyre pressure and the aircraft's groundspeed in, an effective friction
/// coefficient out.
pub fn friction(contaminant: Contaminant, tire_pressure_psi: f64, groundspeed_kt: f64) -> RunwayFrictionOutput {
    let rwy_cc = runway_condition_code(contaminant);
    let mu_dry_equivalent = generic_mu(rwy_cc);
    let v_p = hydroplane_speed_kt(tire_pressure_psi);
    let (mu_effective, hydroplaning) = if has_fluid_film(contaminant) && v_p > 0.0 {
        // GENERIC: friction is unaffected below 70% of v_p, then ramps
        // linearly down to the hydroplaning residual by v_p itself.
        let ratio = (groundspeed_kt.max(0.0) / v_p).clamp(0.0, 2.0);
        let onset = ((ratio - 0.7) / 0.3).clamp(0.0, 1.0);
        (mu_dry_equivalent + (MU_HYDROPLANE_RESIDUAL - mu_dry_equivalent) * onset, ratio >= 1.0)
    } else {
        (mu_dry_equivalent, false)
    };
    RunwayFrictionOutput { contaminant, rwy_cc, braking_action: braking_action(rwy_cc), mu_effective, hydroplaning, hydroplane_speed_kt: v_p }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn dry_and_ice_map_to_the_published_codes_and_bands() {
        let dry = friction(Contaminant::Dry, 200.0, 140.0);
        assert_eq!(dry.rwy_cc, 6);
        assert_eq!(dry.braking_action, BrakingAction::Good);
        let ice = friction(Contaminant::Ice, 200.0, 30.0);
        assert_eq!(ice.rwy_cc, 1);
        assert_eq!(ice.braking_action, BrakingAction::Poor);
        let nil = friction(Contaminant::WaterOverIceOrCompactedSnow, 200.0, 30.0);
        assert_eq!(nil.rwy_cc, 0);
        assert_eq!(nil.braking_action, BrakingAction::Nil);
    }

    #[test]
    fn deeper_contaminant_of_the_same_type_gives_a_lower_friction_coefficient() {
        let shallow = friction(Contaminant::Water { depth_mm: 2.0 }, 200.0, 20.0);
        let deep = friction(Contaminant::Water { depth_mm: 6.0 }, 200.0, 20.0);
        assert!(deep.mu_effective < shallow.mu_effective);
    }

    #[test]
    fn colder_compacted_snow_brakes_better_than_warmer_compacted_snow() {
        let cold = friction(Contaminant::CompactedSnow { oat_c: -20.0 }, 200.0, 20.0);
        let warm = friction(Contaminant::CompactedSnow { oat_c: -5.0 }, 200.0, 20.0);
        assert!(cold.mu_effective > warm.mu_effective);
    }

    #[test]
    fn hydroplane_speed_matches_hornes_worked_example() {
        // A widely republished worked example of Horne's formula: 50 psi
        // hydroplanes at about 64 kt.
        assert!((hydroplane_speed_kt(50.0) - 63.6).abs() < 0.5);
    }

    #[test]
    fn exceeding_hydroplane_speed_on_a_wet_runway_collapses_friction() {
        let slow = friction(Contaminant::Water { depth_mm: 10.0 }, 180.0, 50.0);
        let fast = friction(Contaminant::Water { depth_mm: 10.0 }, 180.0, 160.0);
        let v_p = hydroplane_speed_kt(180.0);
        assert!(160.0 > v_p, "test setup: {v_p}");
        assert!(!slow.hydroplaning);
        assert!(fast.hydroplaning);
        assert!(fast.mu_effective < slow.mu_effective);
        assert!((fast.mu_effective - MU_HYDROPLANE_RESIDUAL).abs() < 1e-9);
    }

    #[test]
    fn a_dry_runway_never_hydroplanes_regardless_of_speed() {
        let out = friction(Contaminant::Dry, 200.0, 999.0);
        assert!(!out.hydroplaning);
        assert_eq!(out.mu_effective, generic_mu(6));
    }

    #[test]
    fn zero_tyre_pressure_is_numerically_safe() {
        let out = friction(Contaminant::Water { depth_mm: 10.0 }, 0.0, 100.0);
        assert!(!out.mu_effective.is_nan());
        assert!(!out.hydroplaning);
        assert_eq!(out.hydroplane_speed_kt, 0.0);
    }
}
