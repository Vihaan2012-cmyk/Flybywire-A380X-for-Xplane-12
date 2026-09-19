//! Nacelle fire/overheat detection sensing element: two independent
//! continuous-loop detectors (A/B) per fire zone, the standard large-
//! transport arrangement (e.g. Kidde/Meggitt-type continuous pneumatic-
//! averaging loops) precisely so a single loop's own fault can never by
//! itself declare -- or fail to declare -- a fire. Two zones per engine
//! are modelled (core/turbine and fan/accessory), a typical large-turbofan
//! nacelle fire-zone split.
//!
//! **This module only senses.** It takes a true zone temperature as input
//! (owned by whichever thermal/fire-propagation model actually represents
//! a fire or overheat event -- not built in this directory) and produces a
//! `FireZoneReading` -- confirmed detection (both loops agree above
//! threshold) and a loop-disagree flag (maintenance/annunciation) -- as a
//! documented interface for the dedicated fire-system agent to consume.
//! Nothing here decides what to do about a fire: no suppression, no bottle
//! discharge, no shutdown logic, exactly the hand-off boundary the shared
//! brief calls for.
//!
//! No Trent-900/A380 detection-loop trip temperature is public. The trip
//! threshold is **GENERIC**, the commonly-cited order of magnitude for a
//! continuous-loop engine fire detector (several hundred degrees C, well
//! above any expected nacelle operating temperature but well below actual
//! fire temperatures).

/// Detection trip temperature, K (**GENERIC**: ~450 C, a representative
/// continuous-loop engine fire detector setpoint).
pub const TRIP_K: f64 = 273.15 + 450.0;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Zone {
    Core,
    FanAccessory,
}

/// One loop's fault state, 0 (healthy) .. 1 (fully failed).
#[derive(Clone, Copy, Debug, Default)]
pub struct LoopFaults {
    /// Depressurised/broken sensing element: never trips regardless of
    /// actual temperature (a dangerous failure direction, the reason a
    /// second independent loop exists).
    pub fails_to_detect: f64,
    /// Chafed/shorted element: trips regardless of actual temperature (a
    /// nuisance failure direction; caught by the other loop disagreeing).
    pub false_trip: f64,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct FireZoneReading {
    pub loop_a_tripped: bool,
    pub loop_b_tripped: bool,
    /// Both loops agree above threshold: the only condition this module
    /// calls "confirmed", for a fire-system consumer to act on.
    pub confirmed: bool,
    /// The loops disagree: a loop fault, not (yet) a confirmed fire.
    pub loop_disagree: bool,
}

fn loop_reading(true_temp_k: f64, faults: &LoopFaults) -> bool {
    if faults.false_trip.clamp(0.0, 1.0) >= 0.5 {
        true
    } else if faults.fails_to_detect.clamp(0.0, 1.0) >= 0.5 {
        false
    } else {
        true_temp_k >= TRIP_K
    }
}

/// One evaluation (no internal state; the loops themselves are effectively
/// instantaneous pneumatic-average sensors at this model's timescale).
pub fn read(zone_true_temp_k: f64, loop_a: &LoopFaults, loop_b: &LoopFaults) -> FireZoneReading {
    let a = loop_reading(zone_true_temp_k, loop_a);
    let b = loop_reading(zone_true_temp_k, loop_b);
    FireZoneReading { loop_a_tripped: a, loop_b_tripped: b, confirmed: a && b, loop_disagree: a != b }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_cool_zone_with_healthy_loops_reads_no_fire_no_nan() {
        let r = read(300.0, &LoopFaults::default(), &LoopFaults::default());
        assert!(!r.confirmed && !r.loop_disagree);
        assert!(!r.loop_a_tripped && !r.loop_b_tripped);
    }

    #[test]
    fn a_hot_zone_above_trip_with_healthy_loops_confirms() {
        let r = read(TRIP_K + 50.0, &LoopFaults::default(), &LoopFaults::default());
        assert!(r.confirmed);
        assert!(!r.loop_disagree);
    }

    #[test]
    fn one_loop_failing_to_detect_alone_does_not_confirm_a_real_fire() {
        let faulty = LoopFaults { fails_to_detect: 1.0, ..Default::default() };
        let r = read(TRIP_K + 50.0, &faulty, &LoopFaults::default());
        assert!(!r.confirmed, "the other loop alone is not enough to confirm -- both must agree");
        assert!(r.loop_disagree, "but the disagreement itself is flagged");
    }

    #[test]
    fn one_loop_false_tripping_alone_does_not_confirm_a_fire_in_a_cool_zone() {
        let faulty = LoopFaults { false_trip: 1.0, ..Default::default() };
        let r = read(300.0, &faulty, &LoopFaults::default());
        assert!(!r.confirmed, "a lone false trip cannot confirm a fire");
        assert!(r.loop_disagree);
    }

    #[test]
    fn both_loops_false_tripping_together_confirms_even_in_a_cool_zone() {
        // A real limitation of loop-agreement voting: it cannot distinguish
        // a real fire from two coincident false trips, hence the interface
        // to a dedicated fire-system agent that may bring other cues.
        let faulty = LoopFaults { false_trip: 1.0, ..Default::default() };
        let r = read(300.0, &faulty, &faulty);
        assert!(r.confirmed);
        assert!(!r.loop_disagree);
    }
}
