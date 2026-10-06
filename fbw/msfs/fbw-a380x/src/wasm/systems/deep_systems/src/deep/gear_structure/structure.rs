use super::{strut, LegKind, MLW_KG};

const FT_TO_M: f64 = 0.3048;
const TAILSTRIKE_HEIGHT_DIFF_FT: f64 = 15.397;
const TAILSTRIKE_DISTANCE_AFT_FT: f64 = 66.702;

pub fn tailstrike_margin_deg(pitch_deg: f64, body_leg_compression_frac: f64) -> f64 {
    let height_diff_m = TAILSTRIKE_HEIGHT_DIFF_FT * FT_TO_M;
    let distance_m = TAILSTRIKE_DISTANCE_AFT_FT * FT_TO_M;
    let extra_compression_m = (body_leg_compression_frac - strut::nominal_compression_frac()) * LegKind::Body.stroke_m();
    let effective_height_diff_m = (height_diff_m - extra_compression_m).max(0.0);
    let critical_pitch_deg = effective_height_diff_m.atan2(distance_m).to_degrees();
    critical_pitch_deg - pitch_deg
}

const WING_FATIGUE_EXPONENT: f64 = 5.0;
const WING_FATIGUE_REFERENCE_CYCLES: f64 = 30_000.0;

#[derive(Clone, Copy, Debug, Default)]
pub struct WingFatigueTracker {
    pub fatigue_index: f64,
}

impl WingFatigueTracker {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn note_wing_leg_cycle(&mut self, peak_force_n: f64, static_reference_n: f64) {
        let ratio = (peak_force_n / static_reference_n.max(1.0)).max(0.0);
        self.fatigue_index += ratio.powf(WING_FATIGUE_EXPONENT) / WING_FATIGUE_REFERENCE_CYCLES;
    }
}

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

#[derive(Clone, Debug)]
pub struct LegLoadSummary {
    pub name: &'static str,
    pub peak_force_n: f64,
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
