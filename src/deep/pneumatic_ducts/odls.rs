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
//! **Detection**: a valid loop reading must sit above an **absolute** set
//! point continuously for `CONFIRM_TIME_S` before the zone trips.
//!
//! This used to be a margin *above ambient*
//! (`THRESHOLD_ABOVE_AMBIENT_K = 100.0`), which
//! `thermal_zones::PROGRESS.md`'s 2026-09-20 pylon-bleed-duct-leak
//! investigation found was the wrong *form*, not just the wrong number: a
//! real continuous-loop detector (a eutectic-salt or similar element) is
//! set at a fixed alarm temperature chosen for its own compartment's
//! structural/wiring limit, not relative to whatever the outside air
//! happens to be doing. The old relative rule tripped at 115 C at 15 C
//! ambient, at 145 C on a 45 C ramp (above the very structural margin its
//! own doc cited), and at only +45 C at cruise with SAT -55 C -- the last
//! being *below* what a pylon bay normally sits at from engine proximity
//! alone, i.e. a nuisance trip waiting to happen. It also meant a real
//! duct leak that heated a bay by a large, genuine amount (e.g. the pylon
//! case's own measured 73 K rise at take-off power) still could not trip
//! it, because the margin it had to clear scaled with ambient rather than
//! being fixed.
//!
//! Two absolute set points now cover the two compartment classes this
//! network's own zones fall into (`network.rs`'s own zone-to-threshold
//! assignment): [`OverheatDetectionLoop::THRESHOLD_WING_FUSELAGE_K`]
//! (~124 C) for the wing-leading-edge/fuselage duct runs, and
//! [`OverheatDetectionLoop::THRESHOLD_PYLON_STRUT_K`] (~200 C) for the
//! pylon/APU-bay duct runs -- higher precisely because those bays run hot
//! in normal operation from engine/APU proximity. Both are **GENERIC**
//! (no A380-specific alarm temperature is public), but the *form* -- fixed
//! per compartment class, not relative to ambient -- is the documented
//! real one. `CONFIRM_TIME_S` (**GENERIC**, a few seconds) rejects brief
//! hot-air transients (e.g. pack trim-air cycling) the way a real
//! detector's debounce does, while still being short next to the
//! timescale structural damage takes to develop.
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
fn interpret(sensed_k: f64, threshold_abs_k: f64, open: f64, short: f64) -> LoopReading {
    if open.clamp(0.0, 1.0) >= 0.5 {
        LoopReading::Fault
    } else if short.clamp(0.0, 1.0) >= 0.5 {
        // Pegged well past this zone's own absolute trip threshold: a
        // shorted loop reads hot.
        LoopReading::Valid(threshold_abs_k + OverheatDetectionLoop::SHORT_PEG_MARGIN_K)
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
    /// This specific loop's own health: true when *this* loop alone has no
    /// valid reading (`loop_a_open`/`loop_b_open` past their own
    /// threshold). Exposed per loop -- not just the aggregate `loop_fault`
    /// -- because the fail-safe voting below means one loop failing open
    /// while its twin stays healthy is, by design, invisible to `trip` and
    /// to the aggregate fault: the same dual-loop masking
    /// `deep::fire_ice::fire_loops` already documents for its own A/B
    /// loops. A real BITE panel still shows which specific loop failed;
    /// this is that reading.
    pub loop_a_fault: bool,
    pub loop_b_fault: bool,
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
    /// This zone's own absolute alarm temperature, K (module doc): fixed at
    /// construction by which compartment class the zone belongs to, not
    /// recomputed from ambient every tick.
    threshold_abs_k: f64,
}
impl OverheatDetectionLoop {
    /// Representative continuous-loop alarm temperature for a wing-leading-
    /// edge or fuselage duct run, K (124 C, **GENERIC**: no A380-specific
    /// figure is public; see module doc for the derivation). 124 C + 273.15.
    pub const THRESHOLD_WING_FUSELAGE_K: f64 = 124.0 + 273.15;
    /// Representative continuous-loop alarm temperature for a pylon/strut
    /// (or APU-bay) duct run, K (200 C, **GENERIC**) -- higher because that
    /// bay runs hot in normal operation from engine/APU proximity alone
    /// (module doc).
    pub const THRESHOLD_PYLON_STRUT_K: f64 = 200.0 + 273.15;
    /// **GENERIC**: debounce against brief hot-air transients.
    pub const CONFIRM_TIME_S: f64 = 3.0;
    /// **GENERIC**: a shorted loop pegs this many degrees past its own
    /// zone's absolute threshold -- comfortably past the trip point on its
    /// own regardless of the real zone temperature.
    const SHORT_PEG_MARGIN_K: f64 = 50.0;
    /// **GENERIC**: false_detection == 1.0 alone reports this many
    /// fictitious degrees on top of whatever is really sensed -- large
    /// enough to cross even the higher (pylon/strut) absolute threshold
    /// from a genuinely cold zone (e.g. a cruise SAT of -55 C, 218 K,
    /// against the 473 K pylon threshold is a 255 K gap), so a false trip
    /// stays independent of the real zone temperature the way the fault it
    /// represents (loop chafe/processing fault, module doc) actually is.
    const FALSE_DETECTION_FULL_K: f64 = 300.0;

    pub fn new(start_k: f64, threshold_abs_k: f64) -> Self {
        Self { loop_a: SensingElement::new(start_k), loop_b: SensingElement::new(start_k), confirming_s: 0.0, threshold_abs_k }
    }

    pub fn step(&mut self, zone_air_k: f64, dt_s: f64, faults: &OdlsFaults) -> OdlsOutputs {
        let a_sensed = self.loop_a.step(zone_air_k, dt_s);
        let b_sensed = self.loop_b.step(zone_air_k, dt_s);
        let a = interpret(a_sensed, self.threshold_abs_k, faults.loop_a_open, faults.loop_a_short);
        let b = interpret(b_sensed, self.threshold_abs_k, faults.loop_b_open, faults.loop_b_short);

        let loop_a_fault = matches!(a, LoopReading::Fault);
        let loop_b_fault = matches!(b, LoopReading::Fault);

        let mut valid_readings: Vec<f64> = Vec::with_capacity(2);
        for r in [a, b] {
            if let LoopReading::Valid(v) = r {
                valid_readings.push(v);
            }
        }
        let loop_fault = valid_readings.is_empty();
        debug_assert_eq!(loop_fault, loop_a_fault && loop_b_fault);

        // Fail-safe voting: either loop alone reading hot is enough to
        // trip (a faulted loop must never mask a real reading from the
        // other), so the hottest valid reading governs.
        let hottest = valid_readings.iter().cloned().fold(f64::MIN, f64::max);
        let false_bump = faults.false_detection.clamp(0.0, 1.0) * Self::FALSE_DETECTION_FULL_K;
        // No valid sensor data at all: there is nothing real to compare to
        // the threshold, so the baseline sits just under it (no trip
        // without an independent false_detection fault on top) rather than
        // inventing a "reads cool" temperature this zone's own real air
        // never entered into the calculation.
        let effective_reading = if loop_fault { self.threshold_abs_k - 1.0 + false_bump } else { hottest + false_bump };

        let over_threshold = effective_reading > self.threshold_abs_k;
        self.confirming_s = if over_threshold { self.confirming_s + dt_s.max(0.0) } else { 0.0 };

        OdlsOutputs {
            trip: self.confirming_s >= Self::CONFIRM_TIME_S,
            loop_fault,
            loop_a_fault,
            loop_b_fault,
            confirming_s: self.confirming_s,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Every generic-mechanism test below runs against the wing/fuselage
    /// class threshold; the zone-classification itself (which real zone
    /// gets which threshold) is `network.rs`'s concern, not this file's.
    const T: f64 = OverheatDetectionLoop::THRESHOLD_WING_FUSELAGE_K;

    fn run_for(zone_k: f64, threshold_abs_k: f64, faults: &OdlsFaults, seconds: f64) -> OdlsOutputs {
        let mut o = OverheatDetectionLoop::new(288.0, threshold_abs_k);
        let dt = 0.5;
        let mut out = OdlsOutputs::default();
        let mut t = 0.0;
        while t < seconds {
            out = o.step(zone_k, dt, faults);
            t += dt;
        }
        out
    }

    #[test]
    fn a_healthy_zone_never_trips() {
        let out = run_for(300.0, T, &OdlsFaults::default(), 600.0);
        assert!(!out.trip);
        assert!(!out.loop_fault);
        assert!(!out.loop_a_fault);
        assert!(!out.loop_b_fault);
    }

    #[test]
    fn a_sustained_overheat_trips_after_the_confirm_delay_not_instantly() {
        let mut o = OverheatDetectionLoop::new(288.0, T);
        let faults = OdlsFaults::default();
        let hot = T + 20.0;
        // Immediately after the jump the lagged element has not caught up
        // and the confirm timer has not elapsed: must not trip yet.
        let first = o.step(hot, 0.1, &faults);
        assert!(!first.trip);
        let mut out = first;
        for _ in 0..200 {
            out = o.step(hot, 0.1, &faults);
        }
        assert!(out.trip, "a sustained real overheat must eventually trip");
    }

    #[test]
    fn a_brief_transient_does_not_trip() {
        let mut o = OverheatDetectionLoop::new(288.0, T);
        let faults = OdlsFaults::default();
        let hot = T + 50.0;
        let out = o.step(hot, OverheatDetectionLoop::CONFIRM_TIME_S * 0.3, &faults);
        assert!(!out.trip, "a transient shorter than the confirm delay must not trip");
    }

    #[test]
    fn both_loops_open_reports_a_fault_with_no_trip_from_a_healthy_zone() {
        let faults = OdlsFaults { loop_a_open: 1.0, loop_b_open: 1.0, ..Default::default() };
        let out = run_for(300.0, T, &faults, 60.0);
        assert!(out.loop_fault);
        assert!(out.loop_a_fault);
        assert!(out.loop_b_fault);
        assert!(!out.trip);
    }

    /// The dual-loop masking this pass closed: a single loop failing open
    /// must still be visible on its own, even though (correctly) it cannot
    /// move the aggregate `loop_fault` or `trip` on its own.
    #[test]
    fn a_single_open_loop_reports_on_its_own_without_masking_or_tripping() {
        let faults = OdlsFaults { loop_a_open: 1.0, ..Default::default() };
        let out = run_for(300.0, T, &faults, 60.0);
        assert!(out.loop_a_fault, "loop A's own open failure must be visible");
        assert!(!out.loop_b_fault, "loop B is healthy");
        assert!(!out.loop_fault, "one surviving loop is not a system fault");
        assert!(!out.trip, "a healthy zone with one open loop must not trip");
    }

    #[test]
    fn one_loop_shorted_alone_still_trips_even_with_the_other_loop_open() {
        let faults = OdlsFaults { loop_a_open: 1.0, loop_b_short: 1.0, ..Default::default() };
        let out = run_for(288.0, T, &faults, 60.0); // zone itself is cold
        assert!(out.trip, "a shorted loop must be able to trip on its own");
        assert!(!out.loop_fault, "one loop still provides a (if false) reading");
    }

    #[test]
    fn false_detection_alone_trips_a_cold_zone() {
        let faults = OdlsFaults { false_detection: 1.0, ..Default::default() };
        // The higher (pylon/strut) threshold is the harder case: the bump
        // must clear it too, from a genuinely cold zone.
        let out = run_for(218.0, OverheatDetectionLoop::THRESHOLD_PYLON_STRUT_K, &faults, 60.0);
        assert!(out.trip);
    }

    /// The absolute form's whole point: the same real zone temperature
    /// trips a wing/fuselage-class loop but not a pylon/strut-class one,
    /// because the two compartments have different structural/wiring
    /// limits, not different distances from whatever ambient happens to be.
    #[test]
    fn the_same_real_temperature_trips_one_compartment_class_but_not_the_other() {
        let hot_for_wing_not_pylon = OverheatDetectionLoop::THRESHOLD_WING_FUSELAGE_K + 10.0;
        let wing = run_for(hot_for_wing_not_pylon, OverheatDetectionLoop::THRESHOLD_WING_FUSELAGE_K, &OdlsFaults::default(), 60.0);
        let pylon = run_for(hot_for_wing_not_pylon, OverheatDetectionLoop::THRESHOLD_PYLON_STRUT_K, &OdlsFaults::default(), 60.0);
        assert!(wing.trip, "past the wing/fuselage threshold must trip a wing/fuselage-class loop");
        assert!(!pylon.trip, "the same temperature is normal for a pylon/strut-class bay and must not trip it");
    }

    #[test]
    fn no_nan_at_dt_zero() {
        let mut o = OverheatDetectionLoop::new(288.0, T);
        let out = o.step(500.0, 0.0, &OdlsFaults::default());
        assert!(!out.trip);
        assert_eq!(out.confirming_s, 0.0);
    }
}
