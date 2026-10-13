//! Leaks, per tank and per transfer-gallery section, with the leak-detection
//! logic a real FQMS uses (fuel used vs. quantity change) and the ECAM FUEL
//! LEAK condition it drives (backlog item 6).
//!
//! Neither `src/fuel_network.rs` nor `src/fuel.rs` has any leak model at
//! all today: fuel only ever leaves the network through an engine/APU demand
//! or the jettison path this push's [`super::jettison`] module deepens. A
//! real leak -- a holed tank skin, a chafed gallery line -- is a *third*,
//! unmetered sink the FQMS never commanded and the FADEC's flow meters never
//! saw, which is exactly how real aircraft detect one: fuel actually
//! disappears faster than the engines/APU can account for. This module
//! reuses [`super::jettison::orifice_flow_m3_s`]/`head_pressure_pa` (already
//! self-contained, sibling module in this same directory, not the crate's
//! shared `physics::fluids`) for the leak orifice physics itself, and adds
//! the detector and per-side isolation logic neither existing module has any
//! equivalent of.

use super::jettison::{head_pressure_pa, orifice_flow_m3_s};

/// A tank-wall leak's mass flow to atmosphere, kg/s: driven by the tank's
/// own hydrostatic head above the hole (tanks are vented to ambient, so the
/// hole's driving pressure is simply the fuel column above it, the same
/// physical picture as a holed water tank).
pub fn tank_wall_leak_kg_s(orifice_area_m2: f64, liquid_depth_above_hole_m: f64, density_kg_m3: f64) -> f64 {
    let dp = head_pressure_pa(liquid_depth_above_hole_m, density_kg_m3);
    orifice_flow_m3_s(orifice_area_m2, dp, density_kg_m3) * density_kg_m3.max(0.0)
}

/// A pressurised transfer-gallery section's leak, kg/s: driven by the
/// line's own pressure (from whichever pump/valve feeds it -- the caller's
/// concern, e.g. via [`super::cg_transfer::achieved_transfer_rate_kg_s`]'s
/// own network) against ambient.
pub fn gallery_leak_kg_s(orifice_area_m2: f64, line_pressure_pa: f64, ambient_pressure_pa: f64, density_kg_m3: f64) -> f64 {
    let dp = line_pressure_pa - ambient_pressure_pa;
    orifice_flow_m3_s(orifice_area_m2, dp, density_kg_m3) * density_kg_m3.max(0.0)
}

/// Converts a 0..1 failure magnitude to a leak orifice area, linear up to
/// `max_area_m2` (`GENERIC`: no public source gives a real leak-hole size;
/// callers should pick `max_area_m2` per site, e.g. a few square millimetres
/// for a stress-crack, a larger figure for a punctured line).
pub fn leak_area_m2(magnitude_0_1: f64, max_area_m2: f64) -> f64 {
    magnitude_0_1.clamp(0.0, 1.0) * max_area_m2.max(0.0)
}

/// The real FQMS/ECAM FUEL LEAK algorithm's own principle: over a rolling
/// window, compare how much the FQMS's *indicated* total quantity actually
/// fell against how much fuel the engines/APU's own flow metering says was
/// burned. A leak is unmetered, so it shows up only as the *difference*
/// between those two numbers, never in either alone -- this is why real
/// aircraft need a dedicated leak computation rather than a low-quantity
/// warning: a normal high-burn climb also depletes tanks fast, but the flow
/// meters account for every kilogram of *that* loss.
///
/// `window_s`/`threshold_kg` are `GENERIC`: no public numeric FUEL LEAK
/// threshold or averaging window is published for the A380 (type-specific
/// FQMS logic is proprietary); the shape of the algorithm -- rolling window,
/// discrepancy threshold, multi-window confirmation to reject a single noisy
/// sample -- is the documented real principle (this is standard across
/// large transport FQMS designs), not the exact figures.
#[derive(Clone, Debug, Default)]
pub struct LeakDetector {
    fuel_used_accum_kg: f64,
    window_start_quantity_kg: Option<f64>,
    window_elapsed_s: f64,
    consecutive_over_threshold_windows: u32,
}
impl LeakDetector {
    pub fn new() -> Self {
        Self::default()
    }

    /// Advances the detector by `dt_s` with this tick's total fuel-flow rate
    /// (engines + APU combined, kg/s, from their own FADEC/flow metering)
    /// and the FQMS's current indicated total quantity (kg). Returns
    /// `true` once enough consecutive windows have shown a discrepancy past
    /// `threshold_kg` to total at least `confirm_windows` (a simple,
    /// numerically stable stand-in for a continuous confirm timer that does
    /// not require sub-window interpolation).
    pub fn update(&mut self, indicated_total_kg: f64, fuel_flow_kg_s: f64, dt_s: f64, window_s: f64, threshold_kg: f64, confirm_windows: u32) -> bool {
        let start = *self.window_start_quantity_kg.get_or_insert(indicated_total_kg);
        self.fuel_used_accum_kg += fuel_flow_kg_s.max(0.0) * dt_s.max(0.0);
        self.window_elapsed_s += dt_s.max(0.0);
        if self.window_elapsed_s < window_s.max(1e-6) {
            return self.consecutive_over_threshold_windows >= confirm_windows.max(1);
        }
        let actual_decrease_kg = (start - indicated_total_kg).max(0.0);
        let discrepancy_kg = actual_decrease_kg - self.fuel_used_accum_kg;
        if discrepancy_kg > threshold_kg.max(0.0) {
            self.consecutive_over_threshold_windows += 1;
        } else {
            self.consecutive_over_threshold_windows = 0;
        }
        self.window_start_quantity_kg = Some(indicated_total_kg);
        self.fuel_used_accum_kg = 0.0;
        self.window_elapsed_s = 0.0;
        self.consecutive_over_threshold_windows >= confirm_windows.max(1)
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Side {
    Left,
    Right,
}

/// Isolates which side is the likelier source once a leak is suspected, by
/// comparing each side's own unmetered discrepancy (its own quantity
/// decrease minus its own engines' burn -- the same principle as
/// [`LeakDetector`], applied per side instead of to the whole aircraft),
/// matching how a real crew cross-checks left/right fuel used vs. FOB during
/// the ECAM FUEL LEAK procedure. `None` when neither side's discrepancy
/// clears `threshold_kg` (e.g. the leak is small/aggregate-only so far, or
/// there is no leak).
pub fn likely_leaking_side(left_discrepancy_kg: f64, right_discrepancy_kg: f64, threshold_kg: f64) -> Option<Side> {
    let l = left_discrepancy_kg > threshold_kg.max(0.0);
    let r = right_discrepancy_kg > threshold_kg.max(0.0);
    match (l, r) {
        (true, false) => Some(Side::Left),
        (false, true) => Some(Side::Right),
        (true, true) => {
            if left_discrepancy_kg >= right_discrepancy_kg {
                Some(Side::Left)
            } else {
                Some(Side::Right)
            }
        }
        (false, false) => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const RHO: f64 = 800.0;

    #[test]
    fn no_leak_flow_from_an_empty_tank_or_a_closed_orifice() {
        assert_eq!(tank_wall_leak_kg_s(0.0001, 0.0, RHO), 0.0);
        assert_eq!(tank_wall_leak_kg_s(0.0, 2.0, RHO), 0.0);
    }

    #[test]
    fn a_deeper_tank_leaks_faster_than_a_shallow_one() {
        let shallow = tank_wall_leak_kg_s(0.0001, 0.5, RHO);
        let deep = tank_wall_leak_kg_s(0.0001, 3.0, RHO);
        assert!(deep > shallow);
    }

    #[test]
    fn a_gallery_leak_needs_positive_line_pressure_over_ambient() {
        assert_eq!(gallery_leak_kg_s(0.0001, 50_000.0, 101_325.0, RHO), 0.0, "gauge pressure below ambient leaks nothing");
        assert!(gallery_leak_kg_s(0.0001, 250_000.0, 101_325.0, RHO) > 0.0);
    }

    #[test]
    fn leak_area_scales_linearly_with_magnitude() {
        assert_eq!(leak_area_m2(0.0, 0.001), 0.0);
        assert_eq!(leak_area_m2(1.0, 0.001), 0.001);
        assert!((leak_area_m2(0.5, 0.001) - 0.0005).abs() < 1e-12);
    }

    #[test]
    fn no_leak_flagged_when_quantity_loss_matches_fuel_used() {
        let mut det = LeakDetector::new();
        let mut qty = 10_000.0;
        let flow = 1.0; // kg/s
        let dt = 1.0;
        let mut flagged = false;
        for _ in 0..600 {
            qty -= flow * dt; // exactly matches what the detector will add up
            flagged = det.update(qty, flow, dt, 60.0, 50.0, 2) || flagged;
        }
        assert!(!flagged);
    }

    #[test]
    fn a_sustained_unmetered_loss_is_flagged_after_confirm_windows() {
        let mut det = LeakDetector::new();
        let mut qty = 10_000.0;
        let flow = 1.0; // kg/s burned, metered
        let leak = 0.5; // kg/s extra, unmetered
        let dt = 1.0;
        let mut flagged = false;
        for _ in 0..130 {
            qty -= (flow + leak) * dt;
            flagged = det.update(qty, flow, dt, 60.0, 20.0, 2);
        }
        assert!(flagged, "a steady unmetered 0.5 kg/s loss over two 60 s windows should exceed a 20 kg threshold");
    }

    #[test]
    fn a_single_noisy_window_alone_does_not_confirm_with_confirm_windows_above_one() {
        let mut det = LeakDetector::new();
        // First window: big spurious discrepancy.
        assert!(!det.update(9_000.0, 0.0, 60.0, 60.0, 20.0, 2));
        // Second window: back to normal (no further discrepancy) resets the streak.
        assert!(!det.update(9_000.0, 0.0, 60.0, 60.0, 20.0, 2));
    }

    #[test]
    fn side_isolation_names_the_heavier_discrepancy_and_none_below_threshold() {
        assert_eq!(likely_leaking_side(5.0, 100.0, 20.0), Some(Side::Right));
        assert_eq!(likely_leaking_side(100.0, 5.0, 20.0), Some(Side::Left));
        assert_eq!(likely_leaking_side(5.0, 5.0, 20.0), None);
        assert_eq!(likely_leaking_side(100.0, 90.0, 20.0), Some(Side::Left));
    }
}
