//! Fire/overheat detection: two independent continuous sensing loops per
//! protected zone, matching the well-documented public principle behind
//! transport-category continuous-loop fire detection (e.g. Kidde/Meggitt
//! continuous-loop fire-and-overheat detector product literature, and the
//! general "two sensing technologies" and "average + discrete response"
//! description reproduced widely in type-training material and in FAA
//! *Aviation Maintenance Technician Handbook -- Airframe* (FAA-H-8083-31),
//! ch. 15 "Fire Protection Systems"). The exact A380 detector part numbers
//! and trip thresholds are not public; every number below is **GENERIC**,
//! sized to plausible transport-aircraft parameters and cited by
//! technology, not by airframe.
//!
//! Nine zones (`docs/deep/BRIEF.md` backlog item 1's list): the four engine
//! nacelle/pylon fire zones (matching FBW's own `fire_and_smoke_
//! protection.rs`'s simplification of "one detection zone per engine"
//! quoted there, read as context, not depended on -- BRIEF rule 2), the
//! APU bay, the main gear bay, the two cargo compartments and the main
//! avionics bay.
//!
//! ## The two loop technologies
//! `Loop::A` on every zone is modelled as a **continuous resistance
//! (thermistor) element**: a semiconductor core whose resistance falls
//! exponentially as it gets hotter (`R = R0 * exp(B*(1/T - 1/T0))`, the
//! standard NTC-thermistor law, e.g. any NTC thermistor datasheet;
//! `B ~= 3435 K` is a common commodity NTC beta value, GENERIC here as a
//! representative order of magnitude, not a cited A380 part). `Loop::B` is
//! a **continuous pneumatic (gas-filled tube) element**: a sealed
//! constant-volume tube whose internal pressure follows Amontons' law
//! (`P/T = const`, ideal gas at constant volume) for its *average*
//! response, plus a **discrete** local getter-material response (a small
//! amount of hydrogen-absorbing material along the tube that releases gas
//! once a fixed local hot-spot temperature is reached, giving a real
//! localized-fire response even when the bulk average is still low) --
//! this average-plus-discrete combination is the actual, publicly
//! documented operating principle of Kidde-type continuous-loop pneumatic
//! detectors (same handbook chapter above, "Continuous-Loop Detector
//! Systems").
//!
//! ## Why a short reads as a false fire and an open reads as a fault
//! An NTC thermistor's resistance falls with *either* rising temperature
//! *or* a dead short across the element -- the two are electrically
//! indistinguishable from the loop's own resistance reading alone, so a
//! shorted loop genuinely produces a false fire indication, not a scripted
//! one. An open circuit instead drives the reading to (near-)infinite
//! resistance / near-zero pressure, a value no real fire or cold-soak
//! condition can produce, which is exactly how a real fire-detection unit
//! distinguishes "faulted loop" from "loop reading cold" -- reproduced
//! here as a loop-fault threshold set beyond the coldest physically
//! expected reading (-65 C cold-soak, a standard avionics/equipment
//! cold-temperature design point).

use super::util::clamp01;

/// The nine protected zones this module detects fire/overheat in.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Zone {
    Engine(u8),
    Apu,
    MainGearBay,
    CargoFwd,
    CargoAft,
    Avionics,
}

pub const ZONES: [Zone; 9] = [
    Zone::Engine(1),
    Zone::Engine(2),
    Zone::Engine(3),
    Zone::Engine(4),
    Zone::Apu,
    Zone::MainGearBay,
    Zone::CargoFwd,
    Zone::CargoAft,
    Zone::Avionics,
];

// ---------------------------------------------------------------------------
// Thermistor loop (Loop A)
// ---------------------------------------------------------------------------

/// NTC thermistor beta constant, K. GENERIC: a common commodity NTC value
/// (see module doc), representative order of magnitude for a semiconductor
/// continuous-loop sensing core.
const THERMISTOR_BETA_K: f64 = 3435.0;
const THERMISTOR_T0_K: f64 = 298.15; // 25 C reference
const THERMISTOR_R0_OHM: f64 = 10_000.0; // GENERIC reference resistance at T0

/// Average-loop temperature, deg C, above which the fire-detection unit
/// declares a fire/overheat from the thermistor loop. GENERIC: set well
/// above any normal operating temperature these zones see (nacelle bleed
/// leak ~200 C, APU bay hot-section proximity) and well below the several-
/// hundred-degree temperatures an actual open flame produces, matching the
/// intent of a real continuous-loop fire threshold.
pub const FIRE_TRIP_C: f64 = 200.0;
/// Coldest temperature these loops are ever expected to read (a cold-soak
/// avionics/equipment bay design point), used to place the loop-fault
/// (open-circuit) threshold beyond any real reading.
const COLD_SOAK_C: f64 = -65.0;

fn thermistor_resistance_ohm(temp_c: f64) -> f64 {
    let t_k = (temp_c + 273.15).max(1.0);
    THERMISTOR_R0_OHM * (THERMISTOR_BETA_K * (1.0 / t_k - 1.0 / THERMISTOR_T0_K)).exp()
}

fn thermistor_trip_resistance_ohm() -> f64 {
    thermistor_resistance_ohm(FIRE_TRIP_C)
}

fn thermistor_fault_resistance_ohm() -> f64 {
    // Well beyond the coldest expected in-range reading: a real open-loop
    // detection compares against a value no real temperature can produce.
    thermistor_resistance_ohm(COLD_SOAK_C) * 5.0
}

// ---------------------------------------------------------------------------
// Pneumatic loop (Loop B)
// ---------------------------------------------------------------------------

/// Sealed-tube reference pressure at the reference temperature, Pa.
/// GENERIC: representative sealed-tube charge pressure for this class of
/// detector, not an A380-specific figure.
const PNEUMATIC_P0_PA: f64 = 150_000.0;
const PNEUMATIC_T0_K: f64 = 298.15;
/// Extra pressure the discrete (getter-material) response contributes once
/// a local hot spot exceeds its own fixed release temperature. GENERIC,
/// sized so a hot spot partway through its release span (see
/// `PNEUMATIC_DISCRETE_SPAN_C`) already pushes the loop over
/// `PNEUMATIC_TRIP_PA` even with the rest of the zone at a normal ambient
/// temperature -- the discrete element exists specifically to catch a
/// localized flame before the bulk average heats up.
const PNEUMATIC_DISCRETE_BONUS_PA: f64 = 200_000.0;
/// The discrete element's own fixed release temperature, deg C -- lower
/// than the bulk average fire threshold, since it exists precisely to
/// catch a localized flame before the whole loop's average heats up.
const PNEUMATIC_DISCRETE_RELEASE_C: f64 = 150.0;
const PNEUMATIC_DISCRETE_SPAN_C: f64 = 20.0;

pub const PNEUMATIC_TRIP_PA: f64 = PNEUMATIC_P0_PA * (FIRE_TRIP_C + 273.15) / PNEUMATIC_T0_K;

fn pneumatic_pressure_pa(avg_c: f64, hot_spot_c: f64) -> f64 {
    let avg_k = (avg_c + 273.15).max(1.0);
    let average_response = PNEUMATIC_P0_PA * avg_k / PNEUMATIC_T0_K;
    let discrete_weight = clamp01((hot_spot_c - PNEUMATIC_DISCRETE_RELEASE_C) / PNEUMATIC_DISCRETE_SPAN_C);
    average_response + PNEUMATIC_DISCRETE_BONUS_PA * discrete_weight
}

fn pneumatic_fault_pressure_pa() -> f64 {
    // Below what a real cold-soak tube ever reads (near-total gas loss from
    // a ruptured tube), the pneumatic analogue of the thermistor's
    // out-of-range open-circuit threshold.
    PNEUMATIC_P0_PA * (COLD_SOAK_C + 273.15) / PNEUMATIC_T0_K * 0.3
}

// ---------------------------------------------------------------------------
// Faults and readings
// ---------------------------------------------------------------------------

/// Fractional faults on one physical loop, 0 = healthy .. 1 = fully failed.
#[derive(Clone, Copy, Debug, Default)]
pub struct LoopFaults {
    /// Broken conductor / ruptured pneumatic tube: reading driven toward
    /// the out-of-physical-range fault value.
    pub open_circuit: f64,
    /// Dead short (thermistor) / internal gas leak into the sensing tube
    /// (pneumatic): reading driven toward a value indistinguishable from a
    /// real fire.
    pub short_circuit: f64,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Technology {
    Thermistor,
    Pneumatic,
}

/// One physical continuous-loop sensing channel.
#[derive(Clone, Copy, Debug)]
pub struct DetectorLoop {
    pub technology: Technology,
}

/// What one loop reports this tick.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LoopReading {
    /// Raw sensed physical quantity (ohms for a thermistor loop, Pa for a
    /// pneumatic loop) after faults are applied, for diagnostics/display.
    pub raw: f64,
    /// The loop's own opinion that there is a fire/overheat.
    pub fire_signal: bool,
    /// The loop itself is faulted (reading outside any physically possible
    /// temperature/pressure), independent of `fire_signal`.
    pub loop_fault: bool,
}

impl DetectorLoop {
    pub fn new(technology: Technology) -> Self {
        Self { technology }
    }

    /// Sense the zone. A continuous element's overall reading is dominated
    /// by its hottest point along the loop (whichever short length of wire
    /// sits over an actual flame or hot spot), so both technologies react
    /// to `hot_spot_c.max(average_c)`; the pneumatic loop's discrete
    /// response additionally needs the *localized* temperature specifically
    /// (see module doc), so it is passed separately.
    pub fn sense(&self, average_zone_c: f64, hot_spot_c: f64, faults: LoopFaults) -> LoopReading {
        let sensed_c = average_zone_c.max(hot_spot_c);
        let open = clamp01(faults.open_circuit);
        let short = clamp01(faults.short_circuit);

        match self.technology {
            Technology::Thermistor => {
                let healthy = thermistor_resistance_ohm(sensed_c);
                // A short pulls resistance toward ~0 ohm (physically
                // indistinguishable from an extremely hot reading); an
                // open pulls it toward a value beyond any real reading.
                let with_short = healthy * (1.0 - short);
                let raw = with_short + (thermistor_fault_resistance_ohm() - with_short) * open;
                let fire_signal = raw < thermistor_trip_resistance_ohm();
                let loop_fault = raw > thermistor_resistance_ohm(COLD_SOAK_C) * 1.5;
                LoopReading { raw, fire_signal, loop_fault }
            }
            Technology::Pneumatic => {
                let healthy = pneumatic_pressure_pa(average_zone_c, hot_spot_c);
                // A short (internal leak) drives pressure toward a
                // fire-mimicking high reading; an open (ruptured tube)
                // drives it toward the below-cold-soak fault floor.
                let with_short = healthy + (PNEUMATIC_P0_PA * 3.0 - healthy) * short;
                let raw = with_short + (pneumatic_fault_pressure_pa() - with_short) * open;
                let fire_signal = raw > PNEUMATIC_TRIP_PA;
                let loop_fault = raw < PNEUMATIC_P0_PA * (COLD_SOAK_C + 273.15) / PNEUMATIC_T0_K * 0.6;
                LoopReading { raw, fire_signal, loop_fault }
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Zone-level AND/OR combination with single-loop-fault fallback
// ---------------------------------------------------------------------------

/// How the fire-detection unit combines two healthy loops. A real unit
/// mostly uses `And` (agreement between loops suppresses nuisance trips
/// from a single spurious reading); `Or` trades that away for maximum
/// sensitivity. Either way, once one loop is *faulted*, the unit falls
/// back to trusting the other loop alone -- the same fallback principle
/// read (not depended on, BRIEF rule 2) in FBW's own
/// `fire_and_smoke_protection.rs`'s `fire_detection_determination`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LoopLogic {
    And,
    Or,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ZoneFireStatus {
    pub fire: bool,
    pub loop_a_fault: bool,
    pub loop_b_fault: bool,
}

pub struct ZoneDetector {
    pub loop_a: DetectorLoop,
    pub loop_b: DetectorLoop,
    pub logic: LoopLogic,
}

impl ZoneDetector {
    pub fn new(logic: LoopLogic) -> Self {
        Self { loop_a: DetectorLoop::new(Technology::Thermistor), loop_b: DetectorLoop::new(Technology::Pneumatic), logic }
    }

    pub fn evaluate(&self, average_zone_c: f64, hot_spot_c: f64, faults_a: LoopFaults, faults_b: LoopFaults) -> ZoneFireStatus {
        let a = self.loop_a.sense(average_zone_c, hot_spot_c, faults_a);
        let b = self.loop_b.sense(average_zone_c, hot_spot_c, faults_b);

        let fire = if a.loop_fault != b.loop_fault {
            // Exactly one loop faulted: trust the other loop alone.
            (a.fire_signal && !a.loop_fault) || (b.fire_signal && !b.loop_fault)
        } else if a.loop_fault && b.loop_fault {
            // Both faulted simultaneously: no reliable reading remains at
            // all, so the unit fails toward the conservative assumption
            // (presumed fire) rather than silently going blind -- the same
            // real, documented failsafe philosophy behind the "both loops
            // lose power/fail together -> declare fire" convention (read,
            // not depended on, in FBW's own `fire_and_smoke_protection.rs`
            // test `unpowering_both_loops_simultaneously_triggers_fire_
            // detection`). A staggered failure (one loop faulted well
            // before the other) is handled by the branch above instead,
            // trusting whichever loop is still reliable.
            true
        } else {
            match self.logic {
                LoopLogic::And => a.fire_signal && b.fire_signal,
                LoopLogic::Or => a.fire_signal || b.fire_signal,
            }
        };

        ZoneFireStatus { fire, loop_a_fault: a.loop_fault, loop_b_fault: b.loop_fault }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn thermistor_resistance_falls_as_temperature_rises() {
        assert!(thermistor_resistance_ohm(20.0) > thermistor_resistance_ohm(200.0));
    }

    #[test]
    fn healthy_thermistor_loop_trips_at_fire_temperature_and_not_below() {
        let l = DetectorLoop::new(Technology::Thermistor);
        let cold = l.sense(20.0, 20.0, LoopFaults::default());
        assert!(!cold.fire_signal && !cold.loop_fault);
        let hot = l.sense(FIRE_TRIP_C + 50.0, FIRE_TRIP_C + 50.0, LoopFaults::default());
        assert!(hot.fire_signal && !hot.loop_fault);
    }

    #[test]
    fn thermistor_short_circuit_produces_a_false_fire_not_a_fault() {
        let l = DetectorLoop::new(Technology::Thermistor);
        let shorted = l.sense(20.0, 20.0, LoopFaults { short_circuit: 1.0, ..Default::default() });
        assert!(shorted.fire_signal, "a dead short must read as fire");
        assert!(!shorted.loop_fault, "a short is not distinguishable from real heat, so must not read as a fault");
    }

    #[test]
    fn thermistor_open_circuit_produces_a_fault_not_a_fire() {
        let l = DetectorLoop::new(Technology::Thermistor);
        let open = l.sense(20.0, 20.0, LoopFaults { open_circuit: 1.0, ..Default::default() });
        assert!(open.loop_fault);
        assert!(!open.fire_signal);
    }

    #[test]
    fn pneumatic_loop_average_response_rises_with_temperature() {
        assert!(pneumatic_pressure_pa(20.0, 20.0) < pneumatic_pressure_pa(FIRE_TRIP_C, FIRE_TRIP_C));
    }

    #[test]
    fn pneumatic_discrete_response_trips_on_a_local_hot_spot_below_the_average_fire_threshold() {
        // A hot spot at 160 C is well below FIRE_TRIP_C (200 C), but above
        // the discrete release temperature (150 C): the discrete bonus
        // must be enough to trip on its own via a localized flame that
        // hasn't yet heated the whole zone.
        let l = DetectorLoop::new(Technology::Pneumatic);
        let localized = l.sense(30.0, 160.0, LoopFaults::default());
        assert!(localized.fire_signal, "{:?}", localized);
    }

    #[test]
    fn pneumatic_short_is_false_fire_and_open_is_fault() {
        let l = DetectorLoop::new(Technology::Pneumatic);
        let shorted = l.sense(20.0, 20.0, LoopFaults { short_circuit: 1.0, ..Default::default() });
        assert!(shorted.fire_signal && !shorted.loop_fault);
        let open = l.sense(20.0, 20.0, LoopFaults { open_circuit: 1.0, ..Default::default() });
        assert!(open.loop_fault && !open.fire_signal);
    }

    #[test]
    fn and_logic_needs_both_loops_while_or_logic_needs_only_one() {
        // Force loop A (thermistor) into false fire via a short, loop B
        // (pneumatic) stays healthy and cold -- with both loops
        // *unfaulted* (short is not a fault, previous test), AND requires
        // agreement and should not trip; OR should.
        let and_zone = ZoneDetector::new(LoopLogic::And);
        let or_zone = ZoneDetector::new(LoopLogic::Or);
        let faults_a = LoopFaults { short_circuit: 1.0, ..Default::default() };
        let and_status = and_zone.evaluate(20.0, 20.0, faults_a, LoopFaults::default());
        let or_status = or_zone.evaluate(20.0, 20.0, faults_a, LoopFaults::default());
        assert!(!and_status.fire, "AND logic must not trip on a single disagreeing loop");
        assert!(or_status.fire, "OR logic must trip on either loop alone");
    }

    #[test]
    fn a_faulted_loop_falls_back_to_trusting_the_other_loop_even_under_and_logic() {
        let zone = ZoneDetector::new(LoopLogic::And);
        let faults_a = LoopFaults { open_circuit: 1.0, ..Default::default() }; // loop A faulted (not firing)
        let status = zone.evaluate(FIRE_TRIP_C + 50.0, FIRE_TRIP_C + 50.0, faults_a, LoopFaults::default());
        assert!(status.fire, "with A faulted, a real fire on healthy loop B alone must still be declared");
        assert!(status.loop_a_fault && !status.loop_b_fault);
    }

    #[test]
    fn both_loops_faulted_simultaneously_fails_toward_declaring_fire() {
        let zone = ZoneDetector::new(LoopLogic::And);
        let faults_a = LoopFaults { open_circuit: 1.0, ..Default::default() };
        let faults_b = LoopFaults { open_circuit: 1.0, ..Default::default() };
        let status = zone.evaluate(20.0, 20.0, faults_a, faults_b);
        assert!(status.loop_a_fault && status.loop_b_fault);
        assert!(status.fire, "total simultaneous loss of both loops must fail safe toward presumed fire");
    }

    #[test]
    fn zones_constant_lists_four_engines_and_five_other_zones() {
        let engines = ZONES.iter().filter(|z| matches!(z, Zone::Engine(_))).count();
        assert_eq!(engines, 4);
        assert_eq!(ZONES.len(), 9);
    }
}
