//! Air Data Reference computation: turns the physical pressures/temperature
//! the probes in this directory actually sense into computed airspeed
//! (CAS), Mach, pressure altitude and true airspeed/SAT, using the standard
//! public ICAO/FAA air-data-computer relations -- and a generic 3-way
//! voter/monitor for combining three independent ADRs' outputs, the kind of
//! redundancy-management technique any triplex air-data system uses.
//!
//! ## Compressible pitot-static inversion
//! Standard subsonic relations (ICAO Doc 7488, *Manual of the ICAO Standard
//! Atmosphere*; and the standard air-data-computer formulae reproduced in
//! any flight-test or air-data textbook, e.g. Gracey, NASA RP-1046):
//! `qc = Ps * ((1 + 0.2 M^2)^3.5 - 1)` (impact pressure from Mach),
//! inverted here as
//! `M = sqrt(5 * ((qc/Ps + 1)^(2/7) - 1))`, and
//! `CAS = a0 * sqrt(5 * ((qc/P0 + 1)^(2/7) - 1))` using sea-level reference
//! conditions `P0`/`a0`. These are the same public formulae
//! `src/physics/adirs.rs` cites; independently re-derived here per this
//! brief's no-cross-module-dependency rule.
//!
//! ## ICAO Standard Atmosphere (pressure -> altitude)
//! Troposphere (0-11 km): `T = T0 - L*h`, `P = P0*(T/T0)^(g0/(R*L))`,
//! inverted as `h = T0/L * (1 - (P/P0)^(R*L/g0))`. Above 11 km (to 20 km,
//! the tropopause-to-second-layer isothermal region): `T = T1`,
//! `P = P1*exp(-g0*(h-11000)/(R*T1))`, inverted as
//! `h = 11000 - R*T1/g0 * ln(P/P1)`. ICAO Doc 7488 / any standard-atmosphere
//! reference.
//!
//! ## Voting/monitoring
//! Real triplex air-data systems compare all three ADRs and flag
//! disagreement rather than blindly averaging (a silently-averaged wrong
//! value is worse than a flagged disagreement). The specific comparison
//! logic and thresholds an A380 FWC actually implements are proprietary and
//! not read here; what is used is the standard, publicly described
//! redundancy-management technique for triplex analogue-ish channels --
//! median select (immune to any *single* outlier, unlike a mean) with a
//! disagreement flag when the spread exceeds a threshold (the general
//! approach described in the fault-tolerant-systems literature, e.g. Lala &
//! Harper, "Architectural Principles for Safety-Critical Real-Time
//! Applications", Proc. IEEE, 1994, section on voting). Threshold value is
//! GENERIC.

const GAMMA_AIR: f64 = 1.4;
const R_AIR: f64 = 287.052_87;
const P0_PA: f64 = 101_325.0;
const A0_MS: f64 = 340.294;
const TAT_RECOVERY_FACTOR: f64 = 0.99;

// ICAO Standard Atmosphere constants.
const T0_K: f64 = 288.15;
const L_K_PER_M: f64 = 0.0065;
const G0: f64 = 9.80665;
const T1_K: f64 = 216.65;
const H1_M: f64 = 11_000.0;
/// Pressure at 11 km in the standard atmosphere, computed from the
/// troposphere relation at `h = H1_M` (Pa).
fn p1_pa() -> f64 {
    P0_PA * (T1_K / T0_K).powf(G0 / (R_AIR * L_K_PER_M))
}

/// A computed-airspeed/Mach/altitude/TAS/SAT bundle from one ADR channel's
/// physical inputs.
#[derive(Clone, Copy, Debug, Default)]
pub struct AdrOutputs {
    pub cas_ms: f64,
    pub mach: f64,
    pub pressure_altitude_m: f64,
    pub tas_ms: f64,
    pub sat_c: f64,
}

/// Computes CAS/Mach/altitude/TAS/SAT from the sensed total/static pressures
/// and sensed TAT. `total_pa` and `static_pa` are what a (possibly faulted)
/// [`super::pitot::PitotProbe`]/[`super::static_port::StaticPort`] pair
/// reports; `tat_c` is what a (possibly faulted)
/// [`super::tat_probe::TatProbe`] reports.
pub fn compute(total_pa: f64, static_pa: f64, tat_c: f64) -> AdrOutputs {
    let static_pa = static_pa.max(1.0);
    let qc = (total_pa - static_pa).max(0.0);

    let mach = (5.0 * ((qc / static_pa + 1.0).powf(2.0 / 7.0) - 1.0)).max(0.0).sqrt();
    let cas_ms = A0_MS * (5.0 * ((qc / P0_PA + 1.0).powf(2.0 / 7.0) - 1.0)).max(0.0).sqrt();

    // Invert the TAT recovery relation for SAT: Tt = Ts*(1 + r*(g-1)/2*M^2).
    let tat_k = tat_c + 273.15;
    let sat_k = tat_k / (1.0 + TAT_RECOVERY_FACTOR * (GAMMA_AIR - 1.0) / 2.0 * mach * mach);
    let sat_c = sat_k - 273.15;
    let speed_of_sound_ms = (GAMMA_AIR * R_AIR * sat_k.max(1.0)).sqrt();
    let tas_ms = mach * speed_of_sound_ms;

    let pressure_altitude_m = if static_pa >= p1_pa() {
        T0_K / L_K_PER_M * (1.0 - (static_pa / P0_PA).powf(R_AIR * L_K_PER_M / G0))
    } else {
        H1_M - R_AIR * T1_K / G0 * (static_pa / p1_pa()).ln()
    };

    AdrOutputs { cas_ms, mach, pressure_altitude_m, tas_ms, sat_c }
}

/// The result of voting three channels of the same quantity.
#[derive(Clone, Copy, Debug, Default)]
pub struct VoteResult {
    /// The median (middle) value -- immune to any single outlier.
    pub value: f64,
    /// The spread (max - min) exceeded the disagreement threshold.
    pub disagree: bool,
    /// Which channel(s) sit furthest from the median (for annunciation);
    /// `[false;3]` when all three agree.
    pub outlier: [bool; 3],
}

/// Median-select voter with a disagreement flag (see module docs).
/// `threshold` is in the same units as `a`/`b`/`c` (e.g. Pa for pressures,
/// m/s for speeds, degrees for angles) -- callers choose a threshold
/// appropriate to the quantity and its normal cross-channel scatter.
pub fn vote3(a: f64, b: f64, c: f64, threshold: f64) -> VoteResult {
    let mut vals = [a, b, c];
    vals.sort_by(|x, y| x.partial_cmp(y).unwrap_or(std::cmp::Ordering::Equal));
    let median = vals[1];
    let spread = vals[2] - vals[0];
    let disagree = spread > threshold.max(0.0);
    // Flag whichever of the three original channels sits furthest from the
    // median as the most likely outlier (only meaningful when disagreeing).
    let dist = |v: f64| (v - median).abs();
    let (da, db, dc) = (dist(a), dist(b), dist(c));
    let max_dist = da.max(db).max(dc);
    let outlier = if disagree && max_dist > 1e-9 {
        [
            (da - max_dist).abs() < 1e-9,
            (db - max_dist).abs() < 1e-9,
            (dc - max_dist).abs() < 1e-9,
        ]
    } else {
        [false; 3]
    };
    VoteResult { value: median, disagree, outlier }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn zero_impact_pressure_is_zero_speed() {
        let out = compute(P0_PA, P0_PA, 15.0);
        assert!(out.cas_ms.abs() < 1e-6);
        assert!(out.mach.abs() < 1e-6);
    }

    #[test]
    fn sea_level_standard_pressure_gives_zero_altitude() {
        let out = compute(P0_PA, P0_PA, 15.0);
        assert!(out.pressure_altitude_m.abs() < 1.0, "{}", out.pressure_altitude_m);
    }

    #[test]
    fn standard_pressure_at_eleven_km_gives_eleven_km_altitude() {
        let out = compute(p1_pa(), p1_pa(), -56.5);
        assert!((out.pressure_altitude_m - H1_M).abs() < 5.0, "{}", out.pressure_altitude_m);
    }

    #[test]
    fn standard_pressure_at_twenty_km_is_above_the_tropopause_branch() {
        // ICAO standard atmosphere at 20 km: ~5474.9 Pa.
        let p20 = 5474.9;
        let out = compute(p20, p20, -56.5);
        assert!((out.pressure_altitude_m - 20_000.0).abs() < 100.0, "{}", out.pressure_altitude_m);
    }

    #[test]
    fn cas_round_trips_through_the_compressible_relations_at_a_known_point() {
        // At sea level, M and CAS coincide (ISA sea-level static = P0/a0).
        let static_pa = P0_PA;
        let mach_in = 0.3;
        let qc = static_pa * ((1.0_f64 + 0.2 * mach_in * mach_in).powf(3.5) - 1.0);
        let out = compute(static_pa + qc, static_pa, 15.0);
        assert!((out.mach - mach_in).abs() < 1e-6, "{}", out.mach);
        assert!((out.cas_ms - mach_in * A0_MS).abs() < 0.5, "{}", out.cas_ms);
    }

    #[test]
    fn sat_recovers_from_tat_at_a_known_mach() {
        let sat_c = -50.0;
        let mach = 0.85;
        let sat_k = sat_c + 273.15;
        let tat_k = sat_k * (1.0 + TAT_RECOVERY_FACTOR * 0.2 * mach * mach);
        // Build a total/static pair that yields exactly this Mach.
        let static_pa = 22_600.0; // ~ FL350 ISA
        let qc = static_pa * ((1.0 + 0.2 * mach * mach).powf(3.5) - 1.0);
        let out = compute(static_pa + qc, static_pa, tat_k - 273.15);
        assert!((out.sat_c - sat_c).abs() < 0.05, "{}", out.sat_c);
    }

    #[test]
    fn voter_picks_the_median_and_ignores_a_single_bad_channel() {
        let result = vote3(250.0, 251.0, 400.0, 5.0);
        assert!((result.value - 251.0).abs() < 1e-9);
        assert!(result.disagree);
        assert_eq!(result.outlier, [false, false, true]);
    }

    #[test]
    fn voter_reports_no_disagreement_within_threshold() {
        let result = vote3(250.0, 251.0, 249.5, 5.0);
        assert!(!result.disagree);
        assert_eq!(result.outlier, [false; 3]);
    }

    #[test]
    fn no_nan_at_zero_pressures() {
        let out = compute(0.0, 0.0, 15.0);
        assert!(out.cas_ms.is_finite());
        assert!(out.pressure_altitude_m.is_finite());
    }
}
