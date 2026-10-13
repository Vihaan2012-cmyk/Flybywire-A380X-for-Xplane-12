//! Fuel quantity gauging (backlog item 2): capacitance probes per tank,
//! densitometer, per-probe failures and how the FQMS (Fuel Quantity
//! Management System) computes an indicated quantity from whichever probes
//! have survived.
//!
//! The existing plugin-wide model already has the core physics of a
//! multi-probe capacitance FQI:
//! `src/physics/fluids.rs::probe_indicated_fill_fraction` averages
//! `probe_count` probes spaced along a tank's span, each clipped to the
//! tank's own envelope, and shows that indication is exact until tilt clips
//! a probe -- genuinely reproducing why real aircraft use discrete probes
//! rather than one ideal totaliser. What it does not do -- and what a real
//! FQMS's BITE (Built-In Test Equipment) does -- is notice when a probe
//! itself has failed (open/shorted/drifted) and exclude or down-weight it,
//! and it has no densitometer at all (real capacitance probes measure
//! permittivity, which the FQMS converts to *volume*; mass requires a
//! measured fuel density from a densitometer, not an assumed constant).
//! This module adds exactly those two gaps: per-probe failure state, and a
//! densitometer with its own failure mode, without re-deriving the
//! tilt-vs-probe-count accuracy result the existing function already
//! establishes (this module's `local_reading` uses the same evenly-spaced,
//! clipped-to-envelope construction, extended with the perturbing probe
//! faults `probe_indicated_fill_fraction` does not have inputs for).
//!
//! Probe counts are `GENERIC`, scaled by tank size (no public source gives
//! the A380 FQMS's exact per-tank probe count): larger tanks carry more
//! probes, one probe per ~1200 US gal of capacity (rounded, minimum 2 so
//! every tank can detect a tilt gradient at all).

/// A tank's probes: `probe_count` scaled `GENERIC`ally from its capacity
/// (see module doc), so the biggest tank (left/right inner, 12 189.4 US gal,
/// `geometry::TankShape::of(Tank::LeftInner)`) carries about ten probes and
/// the smallest (an outer tank, 2731.5 US gal) about two or three.
pub fn probe_count_for_capacity_gal(capacity_gal: f64) -> u32 {
    ((capacity_gal / 1200.0).round() as i64).max(2) as u32
}

/// One probe's own failure state, 0.0 healthy .. 1.0 fully failed (dead),
/// per `docs/deep/BRIEF.md`'s fault-fraction convention.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ProbeFault {
    pub failure_fraction: f64,
}

/// A probe whose `failure_fraction` reaches this is treated by the FQMS's
/// BITE as failed outright (open circuit or shorted to a rail) and excluded
/// from the average, rather than merely biased -- matching how a real
/// capacitance-probe FQI reports a probe as failed rather than quietly
/// averaging in a wildly wrong number once its self-test disagrees enough
/// with its neighbours (`GENERIC` threshold: no public BITE detection
/// threshold is published).
pub const PROBE_DEAD_THRESHOLD: f64 = 0.9;
/// A probe below the dead threshold still drifts before it dies: up to this
/// fraction of full scale, scaled by how far through its failure range it
/// is. `GENERIC`.
pub const MAX_PROBE_BIAS_FRACTION: f64 = 0.15;

/// One probe's own local reading before failure is applied: the same
/// evenly-spaced, envelope-clipped construction
/// `physics::fluids::probe_indicated_fill_fraction` sums over internally,
/// exposed per-probe here so a failed probe can be excluded rather than
/// blindly averaged.
fn local_reading(fill_fraction: f64, tilt_fraction: f64, probe_index: u32, probe_count: u32) -> f64 {
    let n = probe_count.max(1);
    let s = (probe_index as f64 + 0.5) / n as f64 - 0.5;
    (fill_fraction + tilt_fraction * s).clamp(0.0, 1.0)
}

/// The FQMS's computed fill fraction from `probes` (indexed identically to
/// `local_reading`'s `probe_index`), and the surviving fraction of the full
/// array (1.0 = every probe healthy, 0.0 = every probe dead). A probe past
/// [`PROBE_DEAD_THRESHOLD`] is dropped entirely; a probe below it still
/// contributes but with a bias proportional to how failed it is, alternating
/// sign by index so a single degrading probe pulls the average off centre
/// rather than every probe conveniently erring the same way.
///
/// Losing probes degrades accuracy exactly the way losing *sampling
/// resolution* does in `probe_indicated_fill_fraction`: fewer surviving
/// probes average out less of a real tilt gradient, so the returned
/// `(indication, confidence)` should be read together -- a caller wanting
/// the accuracy penalty from fewer probes can re-run the existing
/// `physics::fluids::probe_indicated_fill_fraction` with
/// `surviving_count` in place of the nominal `probe_count`.
pub fn fqms_indicated_fraction(fill_fraction: f64, tilt_fraction: f64, probes: &[ProbeFault]) -> (f64, f64) {
    let n = probes.len().max(1) as u32;
    let mut sum = 0.0;
    let mut alive = 0u32;
    for (i, p) in probes.iter().enumerate() {
        if p.failure_fraction >= PROBE_DEAD_THRESHOLD {
            continue;
        }
        let reading = local_reading(fill_fraction, tilt_fraction, i as u32, n);
        let bias_sign = if i % 2 == 0 { 1.0 } else { -1.0 };
        let bias = (p.failure_fraction / PROBE_DEAD_THRESHOLD).clamp(0.0, 1.0) * MAX_PROBE_BIAS_FRACTION * bias_sign;
        sum += (reading + bias).clamp(0.0, 1.0);
        alive += 1;
    }
    if alive == 0 {
        // Every probe dead: the FQMS has nothing to average and freezes on
        // whatever fraction was last valid; the caller (which owns the
        // "last valid" state) is expected to hold its own last output
        // rather than this function inventing one. Report the raw true
        // fraction with zero confidence so a caller can tell the two cases
        // apart from a nominal, fully confident reading.
        return (fill_fraction, 0.0);
    }
    (sum / alive as f64, alive as f64 / n as f64)
}

/// A densitometer's own failure: `false` reads the true density, `true`
/// makes the FQMS fall back to a fixed reference density
/// (`default_density_kg_m3`) for its volume-to-mass conversion, which is
/// wrong whenever the true density has drifted from that reference (cold
/// fuel is denser, per `physics::fluids::jet_a_density_kg_m3`'s own
/// documented ~9e-4 /K thermal expansion coefficient -- reused here only as
/// a citation, not called, per this directory's self-containment rule).
pub fn indicated_mass_kg(volume_fraction: f64, capacity_gal: f64, true_density_kg_m3: f64, densitometer_failed: bool, default_density_kg_m3: f64) -> f64 {
    let volume_m3 = volume_fraction.clamp(0.0, 1.0) * capacity_gal * super::geometry::GAL_TO_M3;
    let density = if densitometer_failed { default_density_kg_m3 } else { true_density_kg_m3 };
    volume_m3 * density.max(0.0)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn probe_count_scales_with_capacity_and_never_drops_below_two() {
        assert_eq!(probe_count_for_capacity_gal(2731.5), 2);
        assert!(probe_count_for_capacity_gal(12189.4) > probe_count_for_capacity_gal(2731.5));
        assert_eq!(probe_count_for_capacity_gal(1.0), 2);
    }

    #[test]
    fn with_every_probe_healthy_the_indication_matches_the_existing_ideal_average() {
        let n = 6;
        let probes = vec![ProbeFault::default(); n as usize];
        let (indicated, confidence) = fqms_indicated_fraction(0.5, 0.2, &probes);
        // No tilt clipping at 50% fill with modest tilt: should recover the
        // true fraction closely, exactly as
        // `probe_indicated_fill_fraction` does at zero/low tilt.
        assert!((indicated - 0.5).abs() < 0.05, "{indicated}");
        assert_eq!(confidence, 1.0);
    }

    #[test]
    fn a_single_dead_probe_is_excluded_and_confidence_drops() {
        let mut probes = vec![ProbeFault::default(); 4];
        probes[0].failure_fraction = 1.0;
        let (_, confidence) = fqms_indicated_fraction(0.6, 0.0, &probes);
        assert_eq!(confidence, 0.75);
    }

    #[test]
    fn every_probe_dead_reports_zero_confidence() {
        let probes = vec![ProbeFault { failure_fraction: 1.0 }; 3];
        let (_, confidence) = fqms_indicated_fraction(0.5, 0.0, &probes);
        assert_eq!(confidence, 0.0);
    }

    #[test]
    fn a_drifting_but_not_dead_probe_biases_the_average_away_from_truth() {
        let mut probes = vec![ProbeFault::default(); 4];
        probes[0].failure_fraction = 0.8;
        let (healthy_indication, _) = fqms_indicated_fraction(0.5, 0.0, &vec![ProbeFault::default(); 4]);
        let (biased_indication, _) = fqms_indicated_fraction(0.5, 0.0, &probes);
        assert!((biased_indication - healthy_indication).abs() > 1e-6);
    }

    #[test]
    fn a_healthy_densitometer_uses_true_density_a_failed_one_the_default() {
        let true_density = 800.0;
        let default_density = 785.0;
        let ok = indicated_mass_kg(1.0, 1000.0, true_density, false, default_density);
        let failed = indicated_mass_kg(1.0, 1000.0, true_density, true, default_density);
        assert!(ok > failed, "denser true fuel should mass more once measured correctly");
        let volume_m3 = 1000.0 * super::super::geometry::GAL_TO_M3;
        assert!((ok - volume_m3 * true_density).abs() < 1e-6);
        assert!((failed - volume_m3 * default_density).abs() < 1e-6);
    }

    #[test]
    fn mass_is_never_negative_at_zero_volume() {
        assert_eq!(indicated_mass_kg(0.0, 1000.0, 800.0, false, 785.0), 0.0);
    }
}
