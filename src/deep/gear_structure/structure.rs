//! Structural consequences that fall out of the per-leg strut model:
//! tail-strike geometry coupled to actual (not just nominal) body-gear
//! compression, a wing-root bending fatigue index from the wing legs' own
//! landing loads, an overweight-landing inspection trigger, and a combined
//! hard-landing load report tying all five legs together.
//!
//! # Tailstrike geometry
//! `physics/damage.rs` already derives a *static* tailstrike pitch angle
//! (12.99 deg) from FlyByWire's own published contact-point geometry
//! (`flight_model.cfg`): the aft body gear (`point.1`/`point.2`, z = -5.7 ft,
//! y = -15.4 ft) as the pivot, the tailstrike point (`point.17`, z =
//! -72.402222 ft, y = 0.002656 ft) 66.702 ft further aft and 15.397 ft
//! higher, `theta = atan(15.397/66.702)`. This module re-derives that same
//! public geometry independently (it cannot depend on `physics::damage`,
//! a crate-internal module, per this workstream's self-containment rule)
//! and extends it: a body leg sitting lower than its own nominal static
//! compression (an under-serviced strut, or mid-touchdown dynamic
//! compression beyond the static value) measurably reduces the aft
//! clearance, so the critical pitch is not a fixed constant here but a
//! function of the body leg's *actual* compression fraction -- a genuine
//! coupling to `strut.rs`'s own state, not a duplicate constant.
//!
//! # Wing bending fatigue
//! Each wing leg's own Miner's-rule landing-cycle peak force (from
//! `strut::Strut`'s `cycle_completed`/`peak_force_last_cycle_n` outputs) is
//! also a wing-root bending load: the wing-mounted gear reacts its landing
//! impact directly into the wing structure. `WingFatigueTracker` applies
//! the same Miner's-rule form as `strut.rs`'s own fatigue tracking, with its
//! own GENERIC exponent (wing structure is aluminium, not the gear leg's
//! forged steel, so a different, softer textbook S-N exponent) and its own
//! GENERIC reference cycle count, to the wing legs' cycle peaks specifically
//! -- deliberately not the nose or body legs, which do not react through
//! the wing.
//!
//! # Overweight landing / hard-landing report
//! `overweight_landing_check` and `build_hard_landing_report` bring the
//! touchdown mass, the tailstrike margin and every leg's own load
//! utilisation (already computed by `strut::Strut`) together into one
//! structured record -- richer than a single pass/fail flag, and built
//! entirely from this workstream's own physics.

use super::{strut, LegKind, MLW_KG};

/// Exact definition, m per foot.
const FT_TO_M: f64 = 0.3048;
/// FlyByWire `flight_model.cfg` contact-point geometry (see module doc):
/// height difference between the tailstrike point and the aft body gear
/// pivot, ft.
const TAILSTRIKE_HEIGHT_DIFF_FT: f64 = 15.397;
/// Same geometry: horizontal distance aft of the pivot, ft.
const TAILSTRIKE_DISTANCE_AFT_FT: f64 = 66.702;

/// The pitch angle (deg) at which the tail would strike, given how far the
/// aft body gear leg is currently compressed beyond (or short of) its own
/// nominal static compression. `body_leg_compression_frac` is
/// `strut::StrutOutputs::compression_frac` for a body leg.
pub fn tailstrike_margin_deg(pitch_deg: f64, body_leg_compression_frac: f64) -> f64 {
    let height_diff_m = TAILSTRIKE_HEIGHT_DIFF_FT * FT_TO_M;
    let distance_m = TAILSTRIKE_DISTANCE_AFT_FT * FT_TO_M;
    let extra_compression_m = (body_leg_compression_frac - strut::nominal_compression_frac()) * LegKind::Body.stroke_m();
    let effective_height_diff_m = (height_diff_m - extra_compression_m).max(0.0);
    let critical_pitch_deg = effective_height_diff_m.atan2(distance_m).to_degrees();
    critical_pitch_deg - pitch_deg
}

/// GENERIC Basquin-type S-N exponent for a wing-root aluminium structure
/// (textbook mid-range, softer than the gear leg's own forged-steel
/// exponent -- aluminium alloys typically show a lower 1/b than
/// high-strength steel).
const WING_FATIGUE_EXPONENT: f64 = 5.0;
/// GENERIC reference cycle count for the wing structure's landing-load
/// fatigue budget (loosely informed by publicly discussed large-transport
/// design-service landing counts; not cited to Airbus).
const WING_FATIGUE_REFERENCE_CYCLES: f64 = 30_000.0;

/// Accumulates a wing-root bending fatigue index from the wing legs' own
/// landing-cycle peak loads (Miner's rule, the same form `strut.rs` applies
/// to the leg itself, with its own exponent/reference here).
#[derive(Clone, Copy, Debug, Default)]
pub struct WingFatigueTracker {
    pub fatigue_index: f64,
}

impl WingFatigueTracker {
    pub fn new() -> Self {
        Self::default()
    }

    /// Call once per completed wing-leg ground-contact cycle (i.e. whenever
    /// that leg's `strut::StrutOutputs::cycle_completed` is true), with that
    /// cycle's own peak force and the leg's static reference load.
    pub fn note_wing_leg_cycle(&mut self, peak_force_n: f64, static_reference_n: f64) {
        let ratio = (peak_force_n / static_reference_n.max(1.0)).max(0.0);
        self.fatigue_index += ratio.powf(WING_FATIGUE_EXPONENT) / WING_FATIGUE_REFERENCE_CYCLES;
    }
}

/// GENERIC transport-category maintenance-manual-style inspection tiers by
/// how far touchdown mass exceeded MLW (no public A380-specific figure
/// found; matches `physics/damage.rs`'s own documented convention of
/// flagging such thresholds as generic rather than cited).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum InspectionLevel {
    None,
    LogEntry,
    DetailedInspection,
    MajorInspection,
}

const LOG_ENTRY_EXCESS_FRACTION: f64 = 0.0;
const DETAILED_INSPECTION_EXCESS_FRACTION: f64 = 0.03;
const MAJOR_INSPECTION_EXCESS_FRACTION: f64 = 0.10;

#[derive(Clone, Copy, Debug)]
pub struct OverweightLandingResult {
    pub excess_fraction: f64,
    pub inspection_level: InspectionLevel,
}

/// Checks a touchdown mass against MLW and returns the excess fraction and
/// the resulting (GENERIC) inspection tier.
pub fn overweight_landing_check(mass_kg: f64) -> OverweightLandingResult {
    let excess_fraction = ((mass_kg - MLW_KG) / MLW_KG).max(0.0);
    let inspection_level = if excess_fraction <= LOG_ENTRY_EXCESS_FRACTION {
        InspectionLevel::None
    } else if excess_fraction < DETAILED_INSPECTION_EXCESS_FRACTION {
        InspectionLevel::LogEntry
    } else if excess_fraction < MAJOR_INSPECTION_EXCESS_FRACTION {
        InspectionLevel::DetailedInspection
    } else {
        InspectionLevel::MajorInspection
    };
    OverweightLandingResult { excess_fraction, inspection_level }
}

/// One leg's contribution to a hard-landing report.
#[derive(Clone, Debug)]
pub struct LegLoadSummary {
    pub name: &'static str,
    pub peak_force_n: f64,
    /// Peak force as a fraction of that leg's own limit load.
    pub utilization: f64,
    pub overload: bool,
    pub collapsed: bool,
}

#[derive(Clone, Debug)]
pub struct HardLandingReport {
    pub legs: Vec<LegLoadSummary>,
    pub overweight: OverweightLandingResult,
    pub tailstrike_margin_deg: f64,
    pub inspection_required: bool,
}

/// Builds the combined report from each leg's own already-computed load
/// summary plus the touchdown mass and pitch.
pub fn build_hard_landing_report(legs: Vec<LegLoadSummary>, mass_kg: f64, pitch_deg: f64, body_leg_compression_frac: f64) -> HardLandingReport {
    let overweight = overweight_landing_check(mass_kg);
    let margin = tailstrike_margin_deg(pitch_deg, body_leg_compression_frac);
    let inspection_required = overweight.inspection_level != InspectionLevel::None || margin <= 0.0 || legs.iter().any(|l| l.overload || l.collapsed);
    HardLandingReport { legs, overweight, tailstrike_margin_deg: margin, inspection_required }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn at_nominal_compression_the_critical_pitch_matches_the_published_geometry() {
        let nominal = strut::nominal_compression_frac();
        // With no extra sag, margin at pitch = 0 is exactly the critical
        // angle; compare against the independently published figure
        // `physics/damage.rs` already cites from the same public geometry
        // (12.99 deg), without depending on that crate-internal module.
        let margin_at_zero_pitch = tailstrike_margin_deg(0.0, nominal);
        assert!((margin_at_zero_pitch - 12.99).abs() < 0.01, "{margin_at_zero_pitch}");
    }

    #[test]
    fn a_pitch_past_the_critical_angle_leaves_no_margin() {
        let nominal = strut::nominal_compression_frac();
        let margin = tailstrike_margin_deg(13.5, nominal);
        assert!(margin < 0.0, "past the critical angle the margin must be negative: {margin}");
    }

    #[test]
    fn sagging_further_than_nominal_reduces_the_tailstrike_margin() {
        let nominal = strut::nominal_compression_frac();
        let sagging = (nominal + 0.15).min(0.99);
        let margin_nominal = tailstrike_margin_deg(10.0, nominal);
        let margin_sagging = tailstrike_margin_deg(10.0, sagging);
        assert!(margin_sagging < margin_nominal, "a body leg sitting lower than nominal must reduce the tailstrike margin: {margin_sagging} vs {margin_nominal}");
    }

    #[test]
    fn wing_fatigue_accumulates_more_from_a_higher_peak_load_ratio() {
        let mut gentle = WingFatigueTracker::new();
        let mut hard = WingFatigueTracker::new();
        gentle.note_wing_leg_cycle(0.5e6, 1.0e6);
        hard.note_wing_leg_cycle(2.0e6, 1.0e6);
        assert!(hard.fatigue_index > gentle.fatigue_index);
        assert!(gentle.fatigue_index >= 0.0);
    }

    #[test]
    fn overweight_landing_tiers_escalate_with_excess_mass() {
        let at_mlw = overweight_landing_check(MLW_KG);
        assert_eq!(at_mlw.inspection_level, InspectionLevel::None);
        assert_eq!(at_mlw.excess_fraction, 0.0);

        let slightly_over = overweight_landing_check(MLW_KG * 1.01);
        assert_eq!(slightly_over.inspection_level, InspectionLevel::LogEntry);

        let moderately_over = overweight_landing_check(MLW_KG * 1.05);
        assert_eq!(moderately_over.inspection_level, InspectionLevel::DetailedInspection);

        let well_over = overweight_landing_check(MLW_KG * 1.20);
        assert_eq!(well_over.inspection_level, InspectionLevel::MajorInspection);
    }

    #[test]
    fn a_report_with_a_collapsed_leg_always_requires_inspection() {
        let legs = vec![LegLoadSummary { name: "left wing", peak_force_n: 1.0, utilization: 0.1, overload: false, collapsed: true }];
        let nominal = strut::nominal_compression_frac();
        let report = build_hard_landing_report(legs, MLW_KG, 0.0, nominal);
        assert!(report.inspection_required);
    }

    #[test]
    fn a_clean_normal_landing_report_needs_no_inspection() {
        let legs = vec![LegLoadSummary { name: "left wing", peak_force_n: 1.0, utilization: 0.3, overload: false, collapsed: false }];
        let nominal = strut::nominal_compression_frac();
        let report = build_hard_landing_report(legs, MLW_KG * 0.9, 3.0, nominal);
        assert!(!report.inspection_required);
    }

    #[test]
    fn numerically_safe_at_extremes() {
        let m = tailstrike_margin_deg(0.0, 1.0);
        assert!(m.is_finite());
        let m2 = tailstrike_margin_deg(0.0, 0.0);
        assert!(m2.is_finite());
    }
}
