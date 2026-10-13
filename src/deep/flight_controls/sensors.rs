//! Position transducers (LVDT for a linear ram, RVDT for a rotary one):
//! the electrical position instrumentation a flight control computer reads
//! to know where a surface actually is, separate from whatever feedback
//! (often a purely mechanical follow-up linkage in a real hydraulic servo
//! valve, which needs no electronics at all to close its own loop --
//! `actuator::PowerControlUnit`'s own `transducer_frozen`/`transducer_bias_rad`
//! faults model *that* inner-loop feedback) a PCU uses to servo itself.
//! Real FBW/A380 actuators carry more than one of these per actuator
//! specifically so a computer can cross-check them against each other
//! before trusting a position for monitoring, law computation, or
//! consolidation; neither `a380_systems/src/hydraulic/mod.rs` nor
//! `fbw-common`'s `linear_actuator.rs` models the transducers themselves
//! (confirmed by search: no LVDT/RVDT/transducer/sensor-fault code exists
//! there), so this module is new physical modelling, not a port.
//!
//! Three failure modes a real LVDT/RVDT channel can show, all publicly
//! documented aerospace sensor failure categories (e.g. SAE ARP4761's
//! sensor-failure taxonomy: drift, loss-of-signal/open, and intermittent
//! connection):
//! - drift: a slowly growing bias, from demodulator/excitation electronics
//!   degrading;
//! - open circuit: a broken winding or connector pin loses the signal
//!   entirely, and a real LVDT's demodulated output characteristically
//!   rails toward zero (loss of excitation) rather than holding its last
//!   value;
//! - intermittent: a marginal connector/wire only carries signal part of
//!   the time -- modelled here as a deterministic duty cycle (GENERIC) so
//!   the whole crate stays reproducible with no RNG, rather than as true
//!   randomness.

/// Faults one transducer channel can carry.
#[derive(Clone, Copy, Debug, Default)]
pub struct TransducerFaults {
    /// 0 healthy .. 1 drifting at the modelled maximum rate.
    pub drift: f64,
    /// 0 healthy .. 1 fully open (no signal).
    pub open_circuit: f64,
    /// 0 none .. 1 signal dropped essentially all the time.
    pub intermittent: f64,
}

/// One reading: the value if `valid`, otherwise a fail-safe placeholder a
/// monitor should never use as a real position.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct TransducerReading {
    pub value_rad: f64,
    pub valid: bool,
}

/// GENERIC maximum drift rate at `drift == 1.0`: a demodulator degrading
/// enough to matter over tens of seconds, not the sudden failures the other
/// two modes are.
const MAX_DRIFT_RATE_RAD_S: f64 = 0.0015;
/// GENERIC intermittent-connection cycle period: representative of a
/// vibration/thermal-cycling-correlated marginal contact, not literal
/// randomness (see module doc comment).
const INTERMITTENT_PERIOD_S: f64 = 0.7;

pub struct PositionTransducer {
    drift_bias_rad: f64,
    elapsed_s: f64,
}

impl PositionTransducer {
    pub fn new() -> Self {
        Self { drift_bias_rad: 0.0, elapsed_s: 0.0 }
    }

    pub fn step(&mut self, true_angle_rad: f64, faults: &TransducerFaults, dt_s: f64) -> TransducerReading {
        let dt = dt_s.max(0.0);
        self.elapsed_s += dt;

        let drift = faults.drift.clamp(0.0, 1.0);
        self.drift_bias_rad += MAX_DRIFT_RATE_RAD_S * drift * dt;

        let open = faults.open_circuit.clamp(0.0, 1.0);
        if open > 0.5 {
            // Loss of excitation: the demodulated output rails toward zero,
            // not toward the true angle or a frozen one.
            return TransducerReading { value_rad: 0.0, valid: false };
        }

        let intermittent = faults.intermittent.clamp(0.0, 1.0);
        if intermittent > 0.0 {
            let phase = (self.elapsed_s % INTERMITTENT_PERIOD_S) / INTERMITTENT_PERIOD_S;
            if phase < intermittent {
                return TransducerReading { value_rad: 0.0, valid: false };
            }
        }

        TransducerReading { value_rad: true_angle_rad + self.drift_bias_rad, valid: true }
    }
}

impl Default for PositionTransducer {
    fn default() -> Self {
        Self::new()
    }
}

/// Two independent channels on the same actuator (real practice: an A- and
/// B-channel LVDT/RVDT), plus a debounced disagreement monitor -- the same
/// debounce logic as `surface::AsymmetryMonitor`, applied here to one
/// actuator's own two channels instead of two actuators' positions.
pub struct DualTransducer {
    pub a: PositionTransducer,
    pub b: PositionTransducer,
    disagree: super::surface::AsymmetryMonitor,
}

/// What a flight control computer would see from one actuator's dual
/// transducer this tick.
#[derive(Clone, Copy, Debug, Default)]
pub struct DualTransducerOutput {
    pub a: TransducerReading,
    pub b: TransducerReading,
    /// The value the computer should actually use: the healthy channel if
    /// exactly one is valid, channel A's if both are (arbitrary but fixed
    /// tie-break), or `None` if neither is valid.
    pub consolidated_rad: Option<f64>,
    /// True once the two channels have disagreed by more than the monitor's
    /// threshold for long enough to be declared a monitoring fault (not
    /// just noise) -- the signal a real PRIM/SEC would use to distrust this
    /// actuator's position feedback for law computation, independent of
    /// whatever the actuator's own mechanical feedback is still doing.
    pub monitoring_fault: bool,
}

impl DualTransducer {
    pub fn new(disagree_threshold_timer_s: f64) -> Self {
        Self { a: PositionTransducer::new(), b: PositionTransducer::new(), disagree: super::surface::AsymmetryMonitor::new(disagree_threshold_timer_s) }
    }

    pub fn step(&mut self, true_angle_rad: f64, faults_a: &TransducerFaults, faults_b: &TransducerFaults, disagree_threshold_rad: f64, dt_s: f64) -> DualTransducerOutput {
        let a = self.a.step(true_angle_rad, faults_a, dt_s);
        let b = self.b.step(true_angle_rad, faults_b, dt_s);

        // Only meaningful to compare when both are actually reporting;
        // while either is known-invalid there is nothing to "disagree".
        let disagreeing = a.valid && b.valid && self.disagree.step(a.value_rad, b.value_rad, disagree_threshold_rad, dt_s);

        let consolidated_rad = match (a.valid, b.valid) {
            (true, true) => Some(a.value_rad),
            (true, false) => Some(a.value_rad),
            (false, true) => Some(b.value_rad),
            (false, false) => None,
        };

        DualTransducerOutput { a, b, consolidated_rad, monitoring_fault: disagreeing }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const DT: f64 = 0.01;

    #[test]
    fn no_nan_at_rest_or_zero_dt() {
        let mut t = PositionTransducer::new();
        let r = t.step(0.0, &TransducerFaults::default(), 0.0);
        assert!(r.value_rad.is_finite());
        let mut d = DualTransducer::new(1.0);
        let out = d.step(0.0, &TransducerFaults::default(), &TransducerFaults::default(), 0.01, 0.0);
        assert!(out.consolidated_rad.unwrap().is_finite());
    }

    #[test]
    fn a_healthy_channel_reports_the_true_angle_exactly() {
        let mut t = PositionTransducer::new();
        let r = t.step(0.3, &TransducerFaults::default(), DT);
        assert!(r.valid);
        assert_eq!(r.value_rad, 0.3);
    }

    #[test]
    fn drift_accumulates_over_time_and_scales_with_severity() {
        let mut mild = PositionTransducer::new();
        let mut severe = PositionTransducer::new();
        let mut mild_r = TransducerReading::default();
        let mut severe_r = TransducerReading::default();
        for _ in 0..10_000 {
            mild_r = mild.step(0.0, &TransducerFaults { drift: 0.1, ..Default::default() }, DT);
            severe_r = severe.step(0.0, &TransducerFaults { drift: 1.0, ..Default::default() }, DT);
        }
        assert!(mild_r.value_rad > 0.0);
        assert!(severe_r.value_rad > mild_r.value_rad * 5.0);
    }

    #[test]
    fn an_open_circuit_reports_invalid_and_rails_to_zero_not_the_true_angle() {
        let mut t = PositionTransducer::new();
        let r = t.step(0.5, &TransducerFaults { open_circuit: 1.0, ..Default::default() }, DT);
        assert!(!r.valid);
        assert_eq!(r.value_rad, 0.0);
    }

    #[test]
    fn intermittent_drops_the_signal_only_some_of_the_time() {
        let mut t = PositionTransducer::new();
        let faults = TransducerFaults { intermittent: 0.5, ..Default::default() };
        let mut valid_count = 0;
        let mut invalid_count = 0;
        for _ in 0..1000 {
            if t.step(1.0, &faults, DT).valid {
                valid_count += 1;
            } else {
                invalid_count += 1;
            }
        }
        assert!(valid_count > 0 && invalid_count > 0, "valid {valid_count} invalid {invalid_count}");
    }

    #[test]
    fn dual_channel_consolidation_prefers_whichever_channel_is_valid() {
        let mut d = DualTransducer::new(1.0);
        let dead_b = TransducerFaults { open_circuit: 1.0, ..Default::default() };
        let out = d.step(0.42, &TransducerFaults::default(), &dead_b, 0.05, DT);
        assert_eq!(out.consolidated_rad, Some(0.42));
        assert!(!out.monitoring_fault, "one channel being open is not the same as the two disagreeing");
    }

    #[test]
    fn both_channels_dead_leaves_no_consolidated_value() {
        let mut d = DualTransducer::new(1.0);
        let dead = TransducerFaults { open_circuit: 1.0, ..Default::default() };
        let out = d.step(0.42, &dead, &dead, 0.05, DT);
        assert_eq!(out.consolidated_rad, None);
    }

    #[test]
    fn a_persistent_disagreement_between_channels_trips_the_monitoring_fault() {
        let mut d = DualTransducer::new(0.2);
        // Channel B drifts hard while A stays healthy: eventually the two
        // disagree by more than the threshold for long enough to trip.
        let drifting_b = TransducerFaults { drift: 1.0, ..Default::default() };
        let mut tripped = false;
        for _ in 0..10_000 {
            let out = d.step(0.0, &TransducerFaults::default(), &drifting_b, 0.02, DT);
            tripped |= out.monitoring_fault;
        }
        assert!(tripped);
    }
}
