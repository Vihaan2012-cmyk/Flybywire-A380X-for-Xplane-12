use super::common::FUEL_DENSITY_KG_M3;

pub const NUM_GROUPS: usize = 8;
const GROUP_AREA_M2: f64 = 4.2e-6;
const CD: f64 = 0.75;

#[derive(Clone, Copy, Debug)]
pub struct ManifoldFaults {
    pub group_blockage: [f64; NUM_GROUPS],
}

impl Default for ManifoldFaults {
    fn default() -> Self {
        Self { group_blockage: [0.0; NUM_GROUPS] }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct ManifoldState {
    pub manifold_gauge_pa: f64,
    pub manifold_absolute_pa: f64,
    pub group_flow_kg_s: [f64; NUM_GROUPS],
    pub total_flow_kg_s: f64,
    pub hot_streak_severity: f64,
}

pub fn step(metered_kg_s: f64, combustor_pa: f64, faults: &ManifoldFaults) -> ManifoldState {
    let total_q = (metered_kg_s.max(0.0) / FUEL_DENSITY_KG_M3).max(0.0);
    let open_area = |i: usize| GROUP_AREA_M2 * (1.0 - faults.group_blockage[i].clamp(0.0, 1.0)).max(0.0);
    let flow_at = |dp: f64| -> [f64; NUM_GROUPS] {
        let mut f = [0.0; NUM_GROUPS];
        let coeff = (2.0 * dp.max(0.0) / FUEL_DENSITY_KG_M3).sqrt();
        for i in 0..NUM_GROUPS {
            f[i] = CD * open_area(i) * coeff;
        }
        f
    };
    let (mut lo, mut hi) = (0.0, 1.0e9_f64);
    for _ in 0..60 {
        let mid = 0.5 * (lo + hi);
        let sum: f64 = flow_at(mid).iter().sum();
        if sum > total_q {
            hi = mid;
        } else {
            lo = mid;
        }
    }
    let dp = lo;
    let group_flow_m3_s = flow_at(dp);
    let mut group_flow_kg_s = [0.0; NUM_GROUPS];
    for i in 0..NUM_GROUPS {
        group_flow_kg_s[i] = group_flow_m3_s[i] * FUEL_DENSITY_KG_M3;
    }
    let total_flow_kg_s: f64 = group_flow_kg_s.iter().sum();
    let mean = total_flow_kg_s / NUM_GROUPS as f64;
    let hot_streak_severity = if mean > 1e-9 {
        let variance: f64 = group_flow_kg_s.iter().map(|q| (q - mean).powi(2)).sum::<f64>() / NUM_GROUPS as f64;
        variance.sqrt() / mean
    } else {
        0.0
    };

    ManifoldState { manifold_gauge_pa: dp, manifold_absolute_pa: combustor_pa.max(0.0) + dp, group_flow_kg_s, total_flow_kg_s, hot_streak_severity }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn zero_flow_gives_zero_pressure_and_no_nan() {
        let s = step(0.0, 2.0e6, &ManifoldFaults::default());
        assert_eq!(s.manifold_gauge_pa, 0.0);
        assert!(!s.hot_streak_severity.is_nan());
    }

    #[test]
    fn healthy_groups_split_flow_evenly_with_zero_hot_streak() {
        let s = step(3.4, 2.0e6, &ManifoldFaults::default());
        assert!((s.total_flow_kg_s - 3.4).abs() < 0.01);
        assert!(s.hot_streak_severity < 1e-6);
        for q in s.group_flow_kg_s {
            assert!((q - 3.4 / NUM_GROUPS as f64).abs() < 1e-6);
        }
    }

    #[test]
    fn one_coked_group_starves_and_its_neighbours_take_up_the_flow() {
        let mut faults = ManifoldFaults::default();
        faults.group_blockage[0] = 0.9;
        let s = step(3.4, 2.0e6, &faults);
        assert!((s.total_flow_kg_s - 3.4).abs() < 0.01, "total flow must be conserved");
        assert!(s.group_flow_kg_s[0] < s.group_flow_kg_s[1]);
        assert!(s.group_flow_kg_s[1] > 3.4 / NUM_GROUPS as f64, "neighbours must pick up the difference");
        assert!(s.hot_streak_severity > 0.05);
    }

    #[test]
    fn a_fully_blocked_group_passes_no_flow_at_all() {
        let mut faults = ManifoldFaults::default();
        faults.group_blockage[3] = 1.0;
        let s = step(3.4, 2.0e6, &faults);
        assert_eq!(s.group_flow_kg_s[3], 0.0);
        assert!((s.total_flow_kg_s - 3.4).abs() < 0.01);
    }

    #[test]
    fn absolute_manifold_pressure_rides_on_top_of_combustor_pressure() {
        let low_pback = step(2.48, 1.0e6, &ManifoldFaults::default());
        let high_pback = step(2.48, 3.0e6, &ManifoldFaults::default());
        assert!((low_pback.manifold_gauge_pa - high_pback.manifold_gauge_pa).abs() < 1.0);
        assert!((high_pback.manifold_absolute_pa - low_pback.manifold_absolute_pa - 2.0e6).abs() < 1.0);
    }

    #[test]
    fn more_flow_needs_a_higher_manifold_pressure() {
        let low = step(1.0, 2.0e6, &ManifoldFaults::default());
        let high = step(3.4, 2.0e6, &ManifoldFaults::default());
        assert!(high.manifold_gauge_pa > low.manifold_gauge_pa);
    }
}
