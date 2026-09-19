//! Overheat Detection Loop System (ODLS): the pneumatic bleed-duct overheat
//! detection loops that run the length of each duct run through its
//! airframe zone -- a real, distinct ATA 36 system from engine/APU fire
//! detection (`deep::fire_ice::fire_loops`, ATA 26, different zones: engine
//! core/nacelle, APU bay, gear bay, cargo, avionics). ODLS instead watches
//! the zones a *bleed duct itself* passes through (wing-to-body/wing-root
//! fairing, pylon duct run, wing leading edge for wing anti-ice) so a duct
//! leak or rupture is caught and isolated before it damages structure --
//! the public description of this class of system (wide-body wing/pylon
//! bleed-leak detection loops distinct from engine fire loops) appears
//! across multiple public FAA airworthiness directives on 747/767/A330-
//! family bleed duct overheat detectors.
//!
//! **Two independent sensing loops per zone (A/B)**, the standard fail-
//! safe redundancy for this class of continuous detection loop (the same
//! general principle `deep::fire_ice::fire_loops` cites from FAA-H-8083-31
//! ch. 15, applied here to a second, independently-modelled real system,
//! not imported from it, per this push's self-contained-module rule).
//! Each loop is a **sensing element with thermal lag** (a small sheathed
//! thermal element does not read the zone's bulk air instantaneously):
//! modelled the same way this crate's other temperature probes are
//! (`physics::engine::hot_section`'s TGT probe) -- an exact first-order
//! exponential relaxation toward the zone's own air temperature, with a
//! short time constant (`SENSOR_TIME_CONSTANT_S`, **GENERIC**: a few
//! seconds, typical of a small sheathed element, much faster than a bay's
//! own bulk thermal mass so the loop still catches a fast leak).
//!
//! **Fault taxonomy**, matching the well-documented real failure modes of a
//! continuous resistance-loop detector (same general principle
//! `fire_loops.rs` cites: a short reads as a false hot indication, an open
//! reads as a loop fault with no valid data):
//! - `loop_a_open`/`loop_b_open`: that loop's element circuit is broken --
//!   it reports no valid reading (`LoopFault`), not a false "cool" (a
//!   silently-masked overheat would be the more dangerous failure, so a
//!   real detector distinguishes "no data" from "reads cool").
//! - `loop_a_short`/`loop_b_short`: that loop's element is shorted -- it
//!   pegs at a fixed high reading (a real false-hot indication from that
//!   specific loop, distinct from the *system*-level `false_detection`
//!   below).
//! - `false_detection`: an independent spurious-trip fault not tied to a
//!   specific loop's wiring (e.g. loop chafe against structure grounding
//!   intermittently, or a processing fault in the detection controller) --
//!   modelled as the loop *reporting* extra fictitious degrees on top of
//!   whatever it actually senses, so the same threshold/confirm-delay logic
//!   below applies uniformly ("the sensor lies, the isolation logic still
//!   acts on what it is told", `physics::damage`'s sensor-fault
//!   convention) rather than a special-cased trip path.
//!
//! **Detection**: a valid loop reading must sit above `ambient + THRESHOLD`
//! continuously for `CONFIRM_TIME_S` before the zone trips. **GENERIC**
//! threshold: 2000-series aircraft aluminium structure begins losing
//! temper strength above roughly 150 C, so a detector is conventionally set
//! with margin below that; `THRESHOLD_ABOVE_AMBIENT_K = 100.0` sits clearly
//! below that structural limit and clearly above any temperature this
//! crate's zone thermal models reach in healthy operation. `CONFIRM_TIME_S`
//! (**GENERIC**, a few seconds) rejects brief hot-air transients (e.g. pack
//! trim-air cycling) the way a real detector's debounce does, while still
//! being short next to the timescale structural damage takes to develop.
//!
//! **Isolation logic input**: [`OverheatDetectionLoop::trip`] is the single
//! boolean a bleed isolation valve controller reads (`network.rs` closes
//! that zone's engine/APU bleed isolation valve on trip -- the real causal
//! chain this system exists for). [`OverheatDetectionLoop::loop_fault`]
//! flags "no valid detection available" (both loops open/shorted) for a
//! FAULT annunciation, distinct from an actual trip.

/// A single sensing element: first-order lag toward the zone's own air
/// temperature.
#[derive(Clone, Copy, Debug)]
struct SensingElement {
    element_k: f64,
}
impl SensingElement {
    /// **GENERIC**: a small sheathed thermal element's own thermal time
    /// constant, seconds.
    const TIME_CONSTANT_S: f64 = 2.0;

    fn new(start_k: f64) -> Self {
        Self { element_k: start_k.max(1.0) }
    }
    fn step(&mut self, zone_k: f64, dt_s: f64) -> f64 {
        let dt = dt_s.max(0.0);
        let k = 1.0 / Self::TIME_CONSTANT_S;
        self.element_k = zone_k + (self.element_k - zone_k) * (-k * dt).exp();
        self.element_k
    }
}

/// One loop's electrical health, driving how its raw sensed value is
/// interpreted (module docs).
enum LoopReading {
    Valid(f64),
    Fault,
}
fn interpret(sensed_k: f64, ambient_k: f64, open: f64, short: f64) -> LoopReading {
    if open.clamp(0.0, 1.0) >= 0.5 {
        LoopReading::Fault
    } else if short.clamp(0.0, 1.0) >= 0.5 {
        // Pegged well past the trip threshold: a shorted loop reads hot.
        LoopReading::Valid(ambient_k + OverheatDetectionLoop::THRESHOLD_ABOVE_AMBIENT_K * 2.0)
    } else {
        LoopReading::Valid(sensed_k)
    }
}

/// Faults on one zone's ODLS (0.0 = healthy .. 1.0 = fully failed).
#[derive(Clone, Copy, Debug, Default)]
pub struct OdlsFaults {
    pub loop_a_open: f64,
    pub loop_a_short: f64,
    pub loop_b_open: f64,
    pub loop_b_short: f64,
    /// System-level spurious trip, independent of either loop's own wiring
    /// (module docs).
    pub false_detection: f64,
}

/// This tick's ODLS outputs for one zone.
#[derive(Clone, Copy, Debug, Default)]
pub struct OdlsOutputs {
    /// The isolation logic input: true once a real (or falsely detected)
    /// overheat has been confirmed for `CONFIRM_TIME_S`.
    pub trip: bool,
    /// True when neither loop can provide a valid reading (a FAULT
    /// annunciation, not itself a trip).
    pub loop_fault: bool,
    /// How long the trip condition has been continuously present, s (0
    /// while not present) -- exposed for the Study panel / ECAM confirm
    /// countdown.
    pub confirming_s: f64,
}

/// One zone's dual-loop overheat detection.
#[derive(Clone, Copy, Debug)]
pub struct OverheatDetectionLoop {
    loop_a: SensingElement,
    loop_b: SensingElement,
    confirming_s: f64,
}
impl OverheatDetectionLoop {
    pub const THRESHOLD_ABOVE_AMBIENT_K: f64 = 100.0;
    /// **GENERIC**: debounce against brief hot-air transients.
    pub const CONFIRM_TIME_S: f64 = 3.0;
    /// **GENERIC**: false_detection == 1.0 alone reports this many
    /// fictitious degrees on top of whatever is really sensed -- comfortably
    /// past the trip threshold on its own regardless of the real zone
    /// temperature.
    const FALSE_DETECTION_FULL_K: f64 = 150.0;

    pub fn new(start_k: f64) -> Self {
        Self { loop_a: SensingElement::new(start_k), loop_b: SensingElement::new(start_k), confirming_s: 0.0 }
    }

    pub fn step(&mut self, zone_air_k: f64, ambient_k: f64, dt_s: f64, faults: &OdlsFaults) -> OdlsOutputs {
        let a_sensed = self.loop_a.step(zone_air_k, dt_s);
        let b_sensed = self.loop_b.step(zone_air_k, dt_s);
        let a = interpret(a_sensed, ambient_k, faults.loop_a_open, faults.loop_a_short);
        let b = interpret(b_sensed, ambient_k, faults.loop_b_open, faults.loop_b_short);

        let mut valid_readings: Vec<f64> = Vec::with_capacity(2);
        for r in [a, b] {
            if let LoopReading::Valid(v) = r {
                valid_readings.push(v);
            }
        }
        let loop_fault = valid_readings.is_empty();

        // Fail-safe voting: either loop alone reading hot is enough to
        // trip (a faulted loop must never mask a real reading from the
        // other), so the hottest valid reading governs.
        let hottest = valid_readings.iter().cloned().fold(f64::MIN, f64::max);
        let false_bump = faults.false_detection.clamp(0.0, 1.0) * Self::FALSE_DETECTION_FULL_K;
        let effective_reading = if loop_fault { ambient_k + false_bump } else { hottest + false_bump };

        let over_threshold = effective_reading - ambient_k > Self::THRESHOLD_ABOVE_AMBIENT_K;
        self.confirming_s = if over_threshold { self.confirming_s + dt_s.max(0.0) } else { 0.0 };

        OdlsOutputs {
            trip: self.confirming_s >= Self::CONFIRM_TIME_S,
            loop_fault,
            confirming_s: self.confirming_s,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn run_for(zone_k: f64, ambient_k: f64, faults: &OdlsFaults, seconds: f64) -> OdlsOutputs {
        let mut o = OverheatDetectionLoop::new(ambient_k);
        let dt = 0.5;
        let mut out = OdlsOutputs::default();
        let mut t = 0.0;
        while t < seconds {
            out = o.step(zone_k, ambient_k, dt, faults);
            t += dt;
        }
        out
    }

    #[test]
    fn a_healthy_zone_never_trips() {
        let out = run_for(300.0, 288.0, &OdlsFaults::default(), 600.0);
        assert!(!out.trip);
        assert!(!out.loop_fault);
    }

    #[test]
    fn a_sustained_overheat_trips_after_the_confirm_delay_not_instantly() {
        let mut o = OverheatDetectionLoop::new(288.0);
        let faults = OdlsFaults::default();
        let hot = 288.0 + OverheatDetectionLoop::THRESHOLD_ABOVE_AMBIENT_K + 20.0;
        // Immediately after the jump the lagged element has not caught up
        // and the confirm timer has not elapsed: must not trip yet.
        let first = o.step(hot, 288.0, 0.1, &faults);
        assert!(!first.trip);
        let mut out = first;
        for _ in 0..200 {
            out = o.step(hot, 288.0, 0.1, &faults);
        }
        assert!(out.trip, "a sustained real overheat must eventually trip");
    }

    #[test]
    fn a_brief_transient_does_not_trip() {
        let mut o = OverheatDetectionLoop::new(288.0);
        let faults = OdlsFaults::default();
        let hot = 288.0 + OverheatDetectionLoop::THRESHOLD_ABOVE_AMBIENT_K + 50.0;
        let out = o.step(hot, 288.0, OverheatDetectionLoop::CONFIRM_TIME_S * 0.3, &faults);
        assert!(!out.trip, "a transient shorter than the confirm delay must not trip");
    }

    #[test]
    fn both_loops_open_reports_a_fault_with_no_trip_from_a_healthy_zone() {
        let faults = OdlsFaults { loop_a_open: 1.0, loop_b_open: 1.0, ..Default::default() };
        let out = run_for(300.0, 288.0, &faults, 60.0);
        assert!(out.loop_fault);
        assert!(!out.trip);
    }

    #[test]
    fn one_loop_shorted_alone_still_trips_even_with_the_other_loop_open() {
        let faults = OdlsFaults { loop_a_open: 1.0, loop_b_short: 1.0, ..Default::default() };
        let out = run_for(288.0, 288.0, &faults, 60.0); // zone itself is cold
        assert!(out.trip, "a shorted loop must be able to trip on its own");
        assert!(!out.loop_fault, "one loop still provides a (if false) reading");
    }

    #[test]
    fn false_detection_alone_trips_a_cold_zone() {
        let faults = OdlsFaults { false_detection: 1.0, ..Default::default() };
        let out = run_for(288.0, 288.0, &faults, 60.0);
        assert!(out.trip);
    }

    #[test]
    fn no_nan_at_dt_zero() {
        let mut o = OverheatDetectionLoop::new(288.0);
        let out = o.step(500.0, 288.0, 0.0, &OdlsFaults::default());
        assert!(!out.trip);
        assert_eq!(out.confirming_s, 0.0);
    }
}
