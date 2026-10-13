//! Arc fault model: the current and heat an arcing fault (a chafe or a
//! bundle-overheat crosstalk short mid-severity, before it becomes a bolted
//! short) actually delivers, and why a conventional thermal/magnetic
//! breaker can fail to trip on an *intermittent* arc even though a real
//! short exists.
//!
//! **Current.** A bolted (zero-impedance) short draws whatever the source
//! and wiring resistance allow, `I = V / R_wire`. A genuine arcing fault is
//! not zero-impedance: the arc itself sustains an almost current-independent
//! voltage drop (`faults::ARC_VOLTAGE_DROP_V`, cited there), so the arc
//! current solves `I = (V_source - V_arc) / R_wire_to_fault` -- lower than
//! the bolted-short current by exactly the arc's own voltage drop, an
//! elementary circuit relation (KVL around the source/wiring/arc loop),
//! clamped at zero if the arc voltage alone exceeds the source (the arc
//! cannot sustain and self-extinguishes).
//!
//! **Heat into the zone.** The arc dissipates `P = I_arc * V_arc` right at
//! the fault point -- a localized heat source for whatever thermal-zone
//! model owns that physical location (`ThermalNetwork::inject_heat_w`-style
//! interface; this module returns a plain wattage, never calling into
//! another workstream's code, this push's plain-data-interface rule).
//!
//! **Why a thermal breaker can miss it.** A conventional breaker's I^2t
//! thermal element integrates the *time-averaged* heating current. An arc
//! that only makes contact intermittently (a chafe not yet continuous,
//! vibration opening and closing the gap) draws its peak current only a
//! fraction `duty` of the time; the RMS heating-equivalent current a
//! thermal element actually sees is `I_arc * sqrt(duty)` (RMS of a
//! rectangular pulse train, textbook). If that RMS-equivalent current stays
//! under the breaker's rated current, the thermal element never accumulates
//! enough I^2t to trip -- a real, well-documented limitation of thermal/
//! magnetic breakers against arcing faults (the public motivation for arc-
//! fault-specific protection, e.g. UL 1699 arc-fault-circuit-interrupter
//! testing, and Boeing's own published rationale for the 787's arc-fault
//! circuit breakers), reproduced here as a plain ratio, not asserted.

/// Arc current, A, for a source at `source_v` through wiring resistance
/// `r_wire_ohm` to the fault point, given the arc's own voltage drop.
/// Clamped at zero (an arc that cannot sustain).
pub fn arc_current_a(source_v: f64, r_wire_ohm: f64, arc_voltage_v: f64) -> f64 {
    if r_wire_ohm <= 0.0 {
        return 0.0;
    }
    ((source_v - arc_voltage_v) / r_wire_ohm).max(0.0)
}

/// Power the arc itself dissipates at the fault point, W -- the heat term
/// to hand to a thermal-zone model.
pub fn arc_heat_w(arc_current_a: f64, arc_voltage_v: f64) -> f64 {
    (arc_current_a * arc_voltage_v).max(0.0)
}

/// The RMS-equivalent current a conventional thermal (I^2t) breaker element
/// sees from an arc that only makes contact a fraction `duty` (0..1) of the
/// time. `duty >= 1.0` degenerates to the continuous-arc case (`I_arc`
/// itself, no reduction).
pub fn thermal_equivalent_current_a(arc_current_a: f64, duty: f64) -> f64 {
    arc_current_a * duty.clamp(0.0, 1.0).sqrt()
}

/// Whether a conventional thermal/magnetic breaker rated `rated_a`,
/// carrying its own normal load current `baseline_load_a` in addition to
/// the intermittent arc, would see enough *additional* heating current to
/// ever cross its rated current (the necessary condition for its I^2t
/// element to accumulate toward a trip at all -- a ratio below 1 here means
/// the breaker's thermal element sees a sub-rated average and, per the
/// module doc, may never trip on this arc no matter how long it persists).
pub fn thermal_breaker_sees_ratio(baseline_load_a: f64, arc_current_a: f64, duty: f64, rated_a: f64) -> f64 {
    if rated_a <= 0.0 {
        return 0.0;
    }
    (baseline_load_a + thermal_equivalent_current_a(arc_current_a, duty)) / rated_a
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn arc_current_is_lower_than_a_bolted_short_by_the_arcs_own_voltage_drop() {
        let bolted = 115.0 / 0.5; // I = V/R, no arc voltage term
        let arcing = arc_current_a(115.0, 0.5, 30.0);
        assert!(arcing < bolted);
        assert!((arcing - (115.0 - 30.0) / 0.5).abs() < 1e-9);
    }

    #[test]
    fn an_arc_voltage_at_or_above_the_source_cannot_sustain() {
        assert_eq!(arc_current_a(28.0, 0.1, 30.0), 0.0, "28 V source cannot sustain a 30 V arc");
    }

    #[test]
    fn arc_heat_is_zero_at_zero_current_and_positive_otherwise() {
        assert_eq!(arc_heat_w(0.0, 30.0), 0.0);
        assert!(arc_heat_w(10.0, 30.0) > 0.0);
    }

    #[test]
    fn low_duty_cycle_can_keep_an_intermittent_arc_under_the_breakers_trip_threshold() {
        // A 115 V AC feeder with 1.7 ohm of wiring resistance between the bus
        // and a chafe point, arcing at the 30 V arc drop:
        //   I_arc = (115 - 30) / 1.7 = 85 / 1.7 = 50 A.
        let i_arc = arc_current_a(115.0, 1.7, 30.0);
        assert!((i_arc - 50.0).abs() < 1e-9);
        // The feeder is protected by a 35 A breaker (a standard aviation
        // rating) and carries its own 20 A of normal load.
        //
        // Intermittent chafe, contact only 4% of the time (vibration driven):
        //   I_rms_equiv = 50 * sqrt(0.04) = 50 * 0.2 = 10 A
        //   ratio       = (20 + 10) / 35 = 30/35 = 0.857
        // -- under rated, so the I^2t element never accumulates a trip.
        let ratio = thermal_breaker_sees_ratio(20.0, i_arc, 0.04, 35.0);
        assert!((ratio - 30.0 / 35.0).abs() < 1e-9);
        assert!(ratio < 1.0, "ratio {ratio} should stay under 1.0 -- the documented thermal-breaker blind spot");
        // The identical arc made continuous (duty 1.0) and nothing else
        // changed:
        //   ratio = (20 + 50) / 35 = 70/35 = 2.0
        // -- twice rated, which the thermal element does integrate to a trip.
        // Only the duty cycle separates the two cases.
        let continuous_ratio = thermal_breaker_sees_ratio(20.0, i_arc, 1.0, 35.0);
        assert!((continuous_ratio - 2.0).abs() < 1e-9);
        assert!(continuous_ratio > 1.0);
    }

    #[test]
    fn duty_above_one_is_clamped_not_amplified() {
        let normal = thermal_equivalent_current_a(10.0, 1.0);
        let clamped = thermal_equivalent_current_a(10.0, 5.0);
        assert!((normal - clamped).abs() < 1e-9);
    }

    #[test]
    fn no_nan_at_zero_resistance_or_zero_rating() {
        assert_eq!(arc_current_a(115.0, 0.0, 30.0), 0.0);
        assert_eq!(thermal_breaker_sees_ratio(1.0, 1.0, 1.0, 0.0), 0.0);
    }
}
