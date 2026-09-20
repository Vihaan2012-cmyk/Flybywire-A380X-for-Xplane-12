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
//!   contaminated runway).
//! - **Numeric wheel braking coefficients: FAA AC 25-31, "Takeoff
//!   Performance Data for Operations on Contaminated Runways"
//!   (12/22/15), Table 2, "Wheel Braking Coefficients as a Function of
//!   Runway Surface Condition".** The RCAM publishes no mu, but this AC
//!   does, and it is the standard data providers are told to build
//!   contaminated-runway performance data from. Its values are used here
//!   verbatim (see [`wheel_braking_coefficient`]). Only the conditions the
//!   AC hands back to 14 CFR 25.109(c)'s smooth-wet-runway polynomial --
//!   frost, damp/wet, and 3 mm or less of slush or snow -- remain GENERIC
//!   here, because 25.109(c)'s coefficient tables are published only as
//!   graphics and could not be transcribed from a public source; see
//!   [`generic_mu_pending_25_109c`]. Dry is likewise GENERIC: AC 25-31 is
//!   about contaminated runways and has no dry row.
//! - Dynamic hydroplaning speed: Horne's NASA formula, `v_p (kt) = 9 *
//!   sqrt(tire pressure, psi)`, from NASA's tyre-hydroplaning research
//!   (NASA TN D-2056 and follow-on work); widely republished (e.g. a 50
//!   psi tyre hydroplanes at about 64 kt). AC 25-31's Table 2 footnote 2
//!   states the identical formula ("VP = 9 sqrt(P), where VP is the ground
//!   speed in knots and P is the tire pressure in lb/in2"), so the two
//!   agree exactly.
//! - The hydroplaning residual friction, **0.05**, is no longer GENERIC
//!   either: it is AC 25-31 Table 2's own value for water or slush deeper
//!   than 3 mm "for speeds at 85% of the hydroplaning speed and above".
//!   The AC's 70%-to-85% *ramp* toward it is this module's own smoothing of
//!   the AC's step change, deliberately in the conservative direction --
//!   see [`friction`].

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

/// AC 25-31 Table 2's value for compacted snow at -15 C and colder.
pub const MU_COMPACTED_SNOW_COLD: f64 = 0.20;
/// AC 25-31 Table 2's value for the band that covers wet ("slippery when
/// wet") runways, dry or wet snow of any depth over compacted snow, more
/// than 3 mm of dry or wet snow, and compacted snow warmer than -15 C.
pub const MU_SNOW_OR_SLIPPERY_WET: f64 = 0.16;
/// AC 25-31 Table 2's value for ice.
pub const MU_ICE: f64 = 0.08;

/// **GENERIC** fallback for the conditions AC 25-31 Table 2 does not give a
/// number for, but hands to 14 CFR 25.109(c)'s smooth-wet-runway
/// coefficient instead: frost, damp/wet (3 mm or less of water) and 3 mm or
/// less of slush, dry snow or wet snow. 25.109(c)(1) defines that
/// coefficient as a polynomial in true ground speed, tabulated for tyre
/// pressures of 50/100/200/300 psi with linear interpolation between them,
/// and then multiplied by an anti-skid efficiency of 0.80 for a fully
/// modulating system (25.109(c)(2)). Its coefficient tables are published
/// in the CFR only as graphics; they could not be transcribed from any
/// public text source, so they are not implemented and these values stand
/// in. Searched: eCFR 25.109 (tables render as images), AC 25-7 series,
/// EASA CS-25 AMC 25.109.
///
/// The values are a monotonic band by RCAM code, floored at AC 25-31's own
/// sourced numbers so they can never be more optimistic than the AC allows
/// for a comparable surface.
fn generic_mu_pending_25_109c(rwy_cc: u8) -> f64 {
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

/// The wheel braking coefficient for a contaminant, from AC 25-31 Table 2
/// where the AC gives one, and from [`generic_mu_pending_25_109c`] where it
/// defers to 25.109(c).
///
/// AC 25-31's values "assume a fully modulating anti-skid system"; the A380
/// has one, so no anti-skid multiplier applies here. (The AC's own
/// multipliers for lesser systems are 0.625 for quasi-modulating and 0.375
/// for on-off -- recorded in case a failed-antiskid case ever wants them,
/// but not applied.)
///
/// The deep-water/slush case is speed dependent and is handled in
/// [`friction`], since it needs the hydroplaning speed; this returns its
/// low-speed value.
pub fn wheel_braking_coefficient(c: Contaminant) -> f64 {
    match c {
        // AC 25-31 has no dry row: GENERIC.
        Contaminant::Dry => generic_mu_pending_25_109c(6),
        // "Per method defined in 25.109(c)".
        Contaminant::Frost => generic_mu_pending_25_109c(runway_condition_code(c)),
        Contaminant::Water { depth_mm } | Contaminant::Slush { depth_mm } => {
            if depth_mm <= DEPTH_THRESHOLD_MM {
                // 3 mm or less: per 25.109(c).
                generic_mu_pending_25_109c(runway_condition_code(c))
            } else {
                // Greater than 3 mm: "50% of the wheel braking coefficient
                // determined in accordance with 25.109(c), but no greater
                // than 0.16" below 85% of the hydroplaning speed.
                (0.5 * generic_mu_pending_25_109c(runway_condition_code(c))).min(MU_SNOW_OR_SLIPPERY_WET)
            }
        }
        Contaminant::DrySnow { depth_mm } | Contaminant::WetSnow { depth_mm } => {
            if depth_mm <= DEPTH_THRESHOLD_MM {
                generic_mu_pending_25_109c(runway_condition_code(c))
            } else {
                MU_SNOW_OR_SLIPPERY_WET
            }
        }
        Contaminant::CompactedSnow { oat_c } => {
            if oat_c <= COMPACTED_SNOW_COLD_THRESHOLD_C {
                MU_COMPACTED_SNOW_COLD
            } else {
                MU_SNOW_OR_SLIPPERY_WET
            }
        }
        Contaminant::Ice => MU_ICE,
        // The NIL band is below anything AC 25-31 tabulates (the AC's
        // lowest number is ice at 0.08, and this surface is explicitly
        // worse than ice). GENERIC, taken at the hydroplaning residual --
        // the AC's own floor value for a tyre riding on a fluid film.
        Contaminant::WaterOverIceOrCompactedSnow => MU_HYDROPLANE_RESIDUAL,
    }
}

/// Which contaminants can put the tyre up on a fluid film. AC 25-31 Table 2
/// applies its hydroplaning clause to **water and slush deeper than 3 mm**
/// only; dry or wet snow of any depth gets a flat 0.16 with no speed term,
/// and the NIL surface is already at the residual. An earlier revision of
/// this file also hydroplaned wet snow, which the AC does not.
fn has_fluid_film(c: Contaminant) -> bool {
    match c {
        Contaminant::Water { depth_mm } | Contaminant::Slush { depth_mm } => depth_mm > DEPTH_THRESHOLD_MM,
        _ => false,
    }
}

/// Horne's NASA dynamic hydroplaning speed, knots, for a tyre pressure in
/// psi (see module doc).
pub fn hydroplane_speed_kt(tire_pressure_psi: f64) -> f64 {
    9.0 * tire_pressure_psi.max(0.0).sqrt()
}

/// Friction remaining once dynamic hydroplaning is established (the tyre
/// rides on the fluid film, essentially unbraked). **Sourced**: AC 25-31
/// Table 2, water or slush deeper than 3 mm, "(2) For speeds at 85% of the
/// hydroplaning speed and above: 0.05".
pub const MU_HYDROPLANE_RESIDUAL: f64 = 0.05;
/// Fraction of the hydroplaning speed at and above which AC 25-31 Table 2
/// applies [`MU_HYDROPLANE_RESIDUAL`] -- the AC's own 85%.
const HYDROPLANE_FULL_FRACTION: f64 = 0.85;
/// Fraction of the hydroplaning speed at which this model *starts* backing
/// friction off toward the residual. **This 70% is the module's own**: AC
/// 25-31 states a step change at 85%, which is not something a real-time
/// friction model can integrate through cleanly. Ramping 70% -> 85%
/// reproduces the AC exactly at and above 85% while being *more*
/// pessimistic than the AC below it, which is the safe direction for a
/// stopping-distance model.
const HYDROPLANE_ONSET_FRACTION: f64 = 0.70;

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
    let mu_low_speed = wheel_braking_coefficient(contaminant);
    let v_p = hydroplane_speed_kt(tire_pressure_psi);
    let (mu_effective, hydroplaning) = if has_fluid_film(contaminant) && v_p > 0.0 {
        // AC 25-31 Table 2, deep water/slush: the low-speed coefficient up
        // to 85% of v_p, then 0.05 at and above it. Ramped from 70% so the
        // model has no step (see HYDROPLANE_ONSET_FRACTION): at and above
        // 85% this returns exactly the AC's 0.05.
        let ratio = (groundspeed_kt.max(0.0) / v_p).clamp(0.0, 2.0);
        let span = HYDROPLANE_FULL_FRACTION - HYDROPLANE_ONSET_FRACTION;
        let onset = ((ratio - HYDROPLANE_ONSET_FRACTION) / span).clamp(0.0, 1.0);
        (
            mu_low_speed + (MU_HYDROPLANE_RESIDUAL - mu_low_speed) * onset,
            ratio >= HYDROPLANE_FULL_FRACTION,
        )
    } else {
        (mu_low_speed, false)
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
        assert_eq!(out.mu_effective, wheel_braking_coefficient(Contaminant::Dry));
    }

    #[test]
    fn zero_tyre_pressure_is_numerically_safe() {
        let out = friction(Contaminant::Water { depth_mm: 10.0 }, 0.0, 100.0);
        assert!(!out.mu_effective.is_nan());
        assert!(!out.hydroplaning);
        assert_eq!(out.hydroplane_speed_kt, 0.0);
    }

    /// Every value AC 25-31 Table 2 actually states, checked against the
    /// table. If a future edit "tidies" one of these it fails here.
    #[test]
    fn the_sourced_coefficients_are_exactly_ac_25_31_table_2() {
        // "-15 C and colder outside air temperature: compacted snow" -> 0.20
        assert_eq!(wheel_braking_coefficient(Contaminant::CompactedSnow { oat_c: -15.0 }), 0.20);
        assert_eq!(wheel_braking_coefficient(Contaminant::CompactedSnow { oat_c: -40.0 }), 0.20);
        // "Warmer than -15 C: compacted snow" -> 0.16
        assert_eq!(wheel_braking_coefficient(Contaminant::CompactedSnow { oat_c: -14.9 }), 0.16);
        // "Greater than 3 mm depth of dry snow / wet snow" -> 0.16
        assert_eq!(wheel_braking_coefficient(Contaminant::DrySnow { depth_mm: 3.1 }), 0.16);
        assert_eq!(wheel_braking_coefficient(Contaminant::WetSnow { depth_mm: 50.0 }), 0.16);
        // "Ice" -> 0.08
        assert_eq!(wheel_braking_coefficient(Contaminant::Ice), 0.08);
        // Deep water/slush low-speed value: "50% of the coefficient
        // determined in accordance with 25.109(c), but no greater than
        // 0.16". The 25.109(c) stand-in at RWYCC 3 is 0.28, half of which
        // is 0.14, which is under the 0.16 cap -- so the cap is not what
        // binds here and 0.14 is the value.
        assert!((wheel_braking_coefficient(Contaminant::Water { depth_mm: 10.0 }) - 0.14).abs() < 1e-12);
        assert!(wheel_braking_coefficient(Contaminant::Water { depth_mm: 10.0 }) <= 0.16);
    }

    /// AC 25-31 puts the residual in at 85% of the hydroplaning speed, not
    /// at 100%. The ramp must therefore be finished by 85%.
    #[test]
    fn the_residual_is_reached_at_eighty_five_percent_of_the_hydroplaning_speed() {
        let p_psi = 200.0;
        let v_p = hydroplane_speed_kt(p_psi); // 9 * sqrt(200) = 127.3 kt
        let at_85 = friction(Contaminant::Water { depth_mm: 10.0 }, p_psi, 0.85 * v_p);
        assert!((at_85.mu_effective - MU_HYDROPLANE_RESIDUAL).abs() < 1e-9, "{}", at_85.mu_effective);
        assert!(at_85.hydroplaning);
        // Just below the onset the AC's low-speed value still applies in
        // full.
        let at_69 = friction(Contaminant::Water { depth_mm: 10.0 }, p_psi, 0.69 * v_p);
        assert!((at_69.mu_effective - wheel_braking_coefficient(Contaminant::Water { depth_mm: 10.0 })).abs() < 1e-9);
        assert!(!at_69.hydroplaning);
        // Halfway through the ramp it is between the two, and the ramp is
        // monotonic.
        let mut last = f64::INFINITY;
        for i in 0..=20 {
            let v = v_p * (0.70 + 0.15 * (i as f64) / 20.0);
            let mu = friction(Contaminant::Water { depth_mm: 10.0 }, p_psi, v).mu_effective;
            assert!(mu <= last + 1e-12, "friction must not rise with speed: {mu} after {last}");
            last = mu;
        }
    }

    /// AC 25-31 gives snow a flat coefficient with no speed term; only
    /// water and slush deeper than 3 mm hydroplane.
    #[test]
    fn snow_does_not_hydroplane_but_deep_water_and_slush_do() {
        let fast_snow = friction(Contaminant::WetSnow { depth_mm: 20.0 }, 200.0, 200.0);
        assert!(!fast_snow.hydroplaning);
        assert_eq!(fast_snow.mu_effective, MU_SNOW_OR_SLIPPERY_WET);

        for c in [Contaminant::Water { depth_mm: 10.0 }, Contaminant::Slush { depth_mm: 10.0 }] {
            assert!(friction(c, 200.0, 200.0).hydroplaning, "{c:?} must hydroplane");
        }
        // 3 mm or less is in the 25.109(c) band, not the deep band, so it
        // does not hydroplane either.
        assert!(!friction(Contaminant::Water { depth_mm: 3.0 }, 200.0, 200.0).hydroplaning);
    }

    /// Nothing in the table may be more slippery than the NIL surface, and
    /// nothing may be less slippery than dry.
    #[test]
    fn the_coefficient_ordering_holds_across_every_contaminant() {
        let nil = wheel_braking_coefficient(Contaminant::WaterOverIceOrCompactedSnow);
        let dry = wheel_braking_coefficient(Contaminant::Dry);
        for c in [
            Contaminant::Dry,
            Contaminant::Frost,
            Contaminant::Water { depth_mm: 1.0 },
            Contaminant::Water { depth_mm: 20.0 },
            Contaminant::Slush { depth_mm: 1.0 },
            Contaminant::Slush { depth_mm: 20.0 },
            Contaminant::DrySnow { depth_mm: 1.0 },
            Contaminant::DrySnow { depth_mm: 20.0 },
            Contaminant::WetSnow { depth_mm: 1.0 },
            Contaminant::WetSnow { depth_mm: 20.0 },
            Contaminant::CompactedSnow { oat_c: -30.0 },
            Contaminant::CompactedSnow { oat_c: 0.0 },
            Contaminant::Ice,
            Contaminant::WaterOverIceOrCompactedSnow,
        ] {
            let mu = wheel_braking_coefficient(c);
            assert!(mu >= nil - 1e-12 && mu <= dry + 1e-12, "{c:?} -> {mu}");
            assert!(mu > 0.0);
        }
    }
}
