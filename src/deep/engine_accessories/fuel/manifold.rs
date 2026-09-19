//! Burner manifold and fuel nozzles: the last element of the fuel path,
//! distributing the metered flow (`fmu.rs`, gated by the HP SOV,
//! `shutoff_valve.rs`) around the annular combustor through a ring of
//! spray nozzles grouped into sectors.
//!
//! The manifold is a single common gallery at one pressure; each nozzle
//! group is an orifice off it (`Q = Cd * A * sqrt(2*rho*dP)`), so the
//! *manifold* pressure is whatever makes every group's flow sum to the
//! total metered flow (solved by bisection, the same "conserve flow, solve
//! for the pressure that does it" approach `physics::engine::oil`'s filter/
//! relief-valve hydraulics uses). Coking (carbon deposit) or mechanical
//! blockage on one group's nozzles shrinks that group's effective area; the
//! manifold being common means the fuel that group can no longer pass does
//! not simply disappear -- the shared pressure rises until the *other*
//! groups pass the difference, so a partly-coked sector runs lean while its
//! neighbours run rich on the same total fuel flow. That flow imbalance
//! (this module's `hot_streak_severity`, the coefficient of variation of
//! flow across groups) is the physical quantity a combustor/hot-section
//! model would need to place a local hot streak in the gas temperature
//! profile it hands to the turbine -- exposed here as a documented output
//! for that consumer, not itself modelled (this directory does not touch
//! `physics::engine::hot_section`).
//!
//! No Trent-900 nozzle count or sizing is public. `NUM_GROUPS` (**GENERIC**:
//! 8, a plausible sectorisation of a large annular combustor's nozzle ring
//! for pattern-factor purposes) and each group's discharge coefficient/area
//! are **GENERIC**, sized so healthy full-power flow needs a manifold
//! pressure in the range a real fuel injector operates at (order 10-30 bar
//! over combustor pressure).

use super::common::FUEL_DENSITY_KG_M3;

pub const NUM_GROUPS: usize = 8;
/// Each group's healthy orifice area, m^2, equal shares of a total sized so
/// the lead's gas path's ~2.48 kg/s SLS design-point fuel flow
/// (`physics::engine::gas_path`) needs a manifold pressure rise on the
/// order of 15 bar over combustor pressure (**GENERIC**).
const GROUP_AREA_M2: f64 = 4.2e-6;
const CD: f64 = 0.75;

/// Faults: each group's fuel nozzle coking/blockage, 0 (clean) .. 1 (fully
/// blocked).
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
    /// The differential the nozzles spray across, Pa (manifold minus
    /// `combustor_pa`).
    pub manifold_gauge_pa: f64,
    /// The manifold's own absolute pressure, Pa (`combustor_pa +
    /// manifold_gauge_pa`) -- what a manifold-pressure transducer would read.
    pub manifold_absolute_pa: f64,
    pub group_flow_kg_s: [f64; NUM_GROUPS],
    pub total_flow_kg_s: f64,
    /// Coefficient of variation of the per-group flow (0 = perfectly even
    /// distribution across all groups, higher = a growing local hot streak
    /// as blocked groups starve and their neighbours take up the flow).
    /// Documented interface for a combustor/hot-section consumer; not
    /// itself used to alter temperatures by this module.
    pub hot_streak_severity: f64,
}

/// One step. `metered_kg_s` is the fuel mass flow arriving at the manifold
/// (already gated by the HP SOV); `combustor_pa` the pressure the nozzles
/// spray against.
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
    // Bisect the common differential so total group flow matches the
    // metered flow supplied.
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
        // Same flow, same nozzles -> same differential, but the absolute
        // pressure sits on top of whatever the combustor is running at.
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
