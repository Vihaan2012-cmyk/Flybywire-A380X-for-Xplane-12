pub fn probe_count_for_capacity_gal(capacity_gal: f64) -> u32 {
    ((capacity_gal / 1200.0).round() as i64).max(2) as u32
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ProbeFault {
    pub failure_fraction: f64,
}

pub const PROBE_DEAD_THRESHOLD: f64 = 0.9;
pub const MAX_PROBE_BIAS_FRACTION: f64 = 0.15;

fn local_reading(fill_fraction: f64, tilt_fraction: f64, probe_index: u32, probe_count: u32) -> f64 {
    let n = probe_count.max(1);
    let s = (probe_index as f64 + 0.5) / n as f64 - 0.5;
    (fill_fraction + tilt_fraction * s).clamp(0.0, 1.0)
}

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
        return (fill_fraction, 0.0);
    }
    (sum / alive as f64, alive as f64 / n as f64)
}

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
