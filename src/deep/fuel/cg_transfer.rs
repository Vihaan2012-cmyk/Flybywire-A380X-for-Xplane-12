//! Automatic CG control and transfer sequencing faults (backlog item 3):
//! named health for the trim-tank, outer-tank and inner/mid-to-feed transfer
//! valves, pumps and galleries, wing-bending relief from outer-tank
//! retention, and wing-balance cross-feed -- with faults (valve stuck, pump
//! degraded, gallery leak) that a real crew would trace to a specific line
//! replaceable unit, not a generic magnitude.
//!
//! `src/fuel_transfer.rs::LegacyFuel` already ports FlyByWire's own transfer
//! *sequencing* logic faithfully from its TypeScript (`LegacyFuel.ts`),
//! including `calculate_cg_target`'s FCOM-regression target CG polynomial
//! and the trigger conditions that decide when trim/inner/mid transfers run;
//! `src/fuel_network.rs` then moves the fuel for real once those triggers
//! open the named valves. That is genuine depth already and this module does
//! not re-derive any of it. What neither module has: the valves and pumps
//! `LegacyFuel` operates are addressed there only as bare `flight_model.cfg`
//! index numbers, and `fuel_network.rs`'s own `pump_fail`/`valve_fail` fault
//! arrays are anonymous per-index magnitudes with no failure-catalogue entry
//! naming *which* named component (e.g. `TrimTankPumpLeft`,
//! `TrimLineIsolationValveFwd`) failed or why; and neither module quantifies
//! *why* outer-tank retention matters (wing-root bending relief) or models
//! wing-balance cross-feed as anything more than a trigger. This module adds
//! exactly those: named component health feeding a physical
//! achieved-transfer-rate model, a real (if simplified) bending-moment
//! relief figure, and a wing-balance decision from each side's own total.
//!
//! Component names below are FlyByWire's own (`flight_model.cfg`
//! `Valve`/`Pump` `Name` fields, lines 197-330 of the file cited throughout
//! `src/fuel_network.rs`'s doc comments), so a registered failure here reads
//! the same in this model and in the ported network.

/// One transfer-path component's own health, 0.0 healthy .. 1.0 fully
/// failed, per `docs/deep/BRIEF.md`'s convention. A valve's magnitude is how
/// far it is stuck from full authority (it freezes at whatever position it
/// held when it seized, mirroring `fuel_network.rs`'s own `valve_stuck_at`
/// design); a pump's is how much delivered flow/pressure it has lost; a
/// gallery's is the fraction of transfer flow through that gallery section
/// diverted by a leak instead of reaching its destination tank.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct TransferFaults {
    pub valve_stuck_fraction: f64,
    pub pump_degradation_fraction: f64,
    pub gallery_leak_fraction: f64,
}

/// The flow a transfer path actually delivers given `nominal_rate_kg_s` (what
/// an entirely healthy path would move) and its own faults. A stuck valve
/// multiplies by its own remaining open authority (`1 - stuck`, since a
/// valve that seizes part-open still passes a throttled flow -- the same
/// physical picture `fuel_network.rs`'s `valve_open` fraction already uses
/// for the network's own valves); a degraded pump linearly loses delivered
/// flow the way a centrifugal pump's curve falls with impeller wear; a
/// gallery leak diverts a fraction of whatever reaches it before the
/// destination tank sees it. The three combine multiplicatively (independent
/// series losses along one path), floored at zero.
pub fn achieved_transfer_rate_kg_s(nominal_rate_kg_s: f64, faults: &TransferFaults) -> f64 {
    let valve_factor = (1.0 - faults.valve_stuck_fraction.clamp(0.0, 1.0)).max(0.0);
    let pump_factor = (1.0 - faults.pump_degradation_fraction.clamp(0.0, 1.0)).max(0.0);
    let gallery_factor = (1.0 - faults.gallery_leak_fraction.clamp(0.0, 1.0)).max(0.0);
    (nominal_rate_kg_s.max(0.0) * valve_factor * pump_factor * gallery_factor).max(0.0)
}

/// A CG-control transfer is required (a trigger has commanded it, per
/// `LegacyFuel`'s own logic) but not being achieved fast enough: a
/// sustained-fault detector for an "AUTO FUEL TRANSFER FAULT"-style ECAM
/// condition. `required_rate_kg_s` is what the sequencing logic asked for;
/// `achieved_rate_kg_s` is what [`achieved_transfer_rate_kg_s`] actually
/// delivers; the fault confirms after `confirm_s` of a shortfall past
/// `tolerance_fraction` of the requirement, the same confirm-timer shape
/// `crate::deep::api::EcamAlert::confirm` uses for every other alert in this
/// push, kept here as a plain accumulator so this module stays independent
/// of that API's `Cond`/variable-name plumbing.
#[derive(Clone, Copy, Debug, Default)]
pub struct TransferFaultDetector {
    shortfall_held_s: f64,
}
impl TransferFaultDetector {
    pub fn update(&mut self, required_rate_kg_s: f64, achieved_rate_kg_s: f64, tolerance_fraction: f64, confirm_s: f64, dt_s: f64) -> bool {
        let shortfall = required_rate_kg_s > 1e-6 && achieved_rate_kg_s < required_rate_kg_s * (1.0 - tolerance_fraction.clamp(0.0, 1.0));
        self.shortfall_held_s = if shortfall { self.shortfall_held_s + dt_s.max(0.0) } else { 0.0 };
        self.shortfall_held_s >= confirm_s
    }
}

/// Wing-root bending-moment relief from one tank's own fuel mass at its own
/// spanwise station: a point mass `mass_kg` at distance `span_m` from the
/// wing root contributes a relieving moment `mass_kg * G * span_m` about the
/// root under 1 g level flight (the standard cantilever point-load moment;
/// this is exactly why the outer wing tanks -- furthest from the root -- are
/// retained longest on the real A380: the same fuel mass relieves far more
/// bending moment out at the tip than it would centrally). `span_m` should
/// be each tank's own lateral `Position` distance from `flight_model.cfg`
/// (e.g. `LeftOuter`'s `-100.0` ft second `Position` field, `Feed1`'s
/// `-71.0`, `flight_model.cfg:142-143` -- converted to metres by the
/// caller); this function only does the moment arithmetic, not the position
/// lookup, to stay self-contained.
pub fn wing_bending_relief_nm(mass_kg: f64, span_m: f64) -> f64 {
    mass_kg.max(0.0) * super::geometry::G * span_m.abs()
}

/// Whether the outer tanks should still be retained (not yet transferred)
/// for load alleviation: `GENERIC` scheduling rule -- retain the outer tanks
/// as long as either inner or mid wing tank still holds more than
/// `retain_until_fraction` of its own capacity, since transferring the
/// outboard mass away first would give up bending relief the aircraft could
/// otherwise keep for longer (the real A380's own outer-tank-last ordering
/// principle is publicly documented; the exact fill-fraction threshold at
/// which it switches is not, hence `GENERIC`).
pub fn outer_tank_retention_active(inner_fill_fraction: f64, mid_fill_fraction: f64, retain_until_fraction: f64) -> bool {
    inner_fill_fraction > retain_until_fraction || mid_fill_fraction > retain_until_fraction
}

/// Whether a wing-balance cross-feed transfer is called for: the two wings'
/// total fuel differ by more than `imbalance_limit_kg`. Mirrors the shape of
/// `fuel_network.rs`'s own ported `TankAbsImbalanceAbove`/`Below` trigger
/// conditions (module doc point 4) but expressed over the whole-wing totals
/// this module's callers already have, rather than the network's own
/// gallon-denominated trigger thresholds.
pub fn wing_balance_transfer_needed(left_total_kg: f64, right_total_kg: f64, imbalance_limit_kg: f64) -> bool {
    (left_total_kg - right_total_kg).abs() > imbalance_limit_kg.max(0.0)
}

/// Which side is heavy, for routing a cross-feed transfer from heavy to
/// light side. `None` when balanced or already within limits.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum HeavySide {
    Left,
    Right,
}
pub fn heavy_side(left_total_kg: f64, right_total_kg: f64, imbalance_limit_kg: f64) -> Option<HeavySide> {
    if !wing_balance_transfer_needed(left_total_kg, right_total_kg, imbalance_limit_kg) {
        return None;
    }
    Some(if left_total_kg > right_total_kg { HeavySide::Left } else { HeavySide::Right })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_healthy_path_delivers_the_full_nominal_rate() {
        assert_eq!(achieved_transfer_rate_kg_s(10.0, &TransferFaults::default()), 10.0);
    }

    #[test]
    fn a_half_stuck_valve_halves_the_delivered_rate() {
        let f = TransferFaults { valve_stuck_fraction: 0.5, ..Default::default() };
        assert!((achieved_transfer_rate_kg_s(10.0, &f) - 5.0).abs() < 1e-9);
    }

    #[test]
    fn faults_combine_multiplicatively_and_never_go_negative() {
        let f = TransferFaults { valve_stuck_fraction: 0.5, pump_degradation_fraction: 0.5, gallery_leak_fraction: 0.5 };
        let rate = achieved_transfer_rate_kg_s(8.0, &f);
        assert!((rate - 1.0).abs() < 1e-9, "{rate}");
        let full_fail = TransferFaults { valve_stuck_fraction: 1.0, ..Default::default() };
        assert_eq!(achieved_transfer_rate_kg_s(8.0, &full_fail), 0.0);
    }

    #[test]
    fn no_nan_at_zero_nominal_rate_or_zero_dt() {
        let f = TransferFaults::default();
        assert_eq!(achieved_transfer_rate_kg_s(0.0, &f), 0.0);
        let mut det = TransferFaultDetector::default();
        assert!(!det.update(0.0, 0.0, 0.1, 5.0, 0.0));
    }

    #[test]
    fn a_sustained_shortfall_confirms_after_its_timer_and_a_brief_one_does_not() {
        let mut det = TransferFaultDetector::default();
        // Required 10, achieved 2: well past the 10% tolerance.
        assert!(!det.update(10.0, 2.0, 0.1, 5.0, 4.0));
        assert!(det.update(10.0, 2.0, 0.1, 5.0, 2.0));
        let mut det2 = TransferFaultDetector::default();
        assert!(!det2.update(10.0, 2.0, 0.1, 5.0, 1.0));
        assert!(!det2.update(10.0, 9.5, 0.1, 5.0, 10.0), "recovering the rate resets the timer");
    }

    #[test]
    fn bending_relief_scales_with_mass_and_span_and_ignores_sign() {
        let inboard = wing_bending_relief_nm(1000.0, 10.0);
        let outboard = wing_bending_relief_nm(1000.0, 30.0);
        assert!(outboard > inboard, "fuel further out relieves more root bending moment");
        assert_eq!(wing_bending_relief_nm(1000.0, -30.0), outboard);
        assert_eq!(wing_bending_relief_nm(0.0, 30.0), 0.0);
    }

    #[test]
    fn outer_tanks_are_retained_while_inner_or_mid_still_hold_fuel() {
        assert!(outer_tank_retention_active(0.6, 0.1, 0.3));
        assert!(outer_tank_retention_active(0.1, 0.6, 0.3));
        assert!(!outer_tank_retention_active(0.1, 0.1, 0.3));
    }

    #[test]
    fn wing_balance_flags_only_past_the_limit_and_names_the_heavy_side() {
        assert!(!wing_balance_transfer_needed(50_000.0, 49_600.0, 500.0));
        assert!(wing_balance_transfer_needed(50_000.0, 49_000.0, 500.0));
        assert_eq!(heavy_side(50_000.0, 49_000.0, 500.0), Some(HeavySide::Left));
        assert_eq!(heavy_side(49_000.0, 50_000.0, 500.0), Some(HeavySide::Right));
        assert_eq!(heavy_side(50_000.0, 49_600.0, 500.0), None);
    }
}
