//! Smoke detector (cargo compartment / lavatory): a photoelectric
//! light-scattering detector, the principle used by the TSO-C1d-class smoke
//! detectors fitted to transport-category cargo compartments and
//! lavatories -- a light source and photodetector pair where smoke
//! particles scatter light into the photodetector, raising its signal in
//! proportion to smoke density (conventionally expressed as percent
//! obscuration per foot, the standard aviation smoke-density unit used in
//! TSO-C1d/related smoke-detector certification testing).
//!
//! ## Faults
//! - **Dirty/degraded optics** (dust and contamination accumulating on the
//!   lens over time, a real, commonly cited reason aircraft smoke detectors
//!   need periodic cleaning/replacement) attenuates the light reaching the
//!   photodetector, reducing the *apparent* obscuration for a given true
//!   smoke density -- a false-negative risk (delayed or missed detection),
//!   modelled as a multiplicative sensitivity loss.
//! - **Contamination/electrical fault** can instead add a spurious signal
//!   with no smoke present -- a false-alarm risk, modelled as a signed
//!   bias, physically distinct from sensitivity loss (one attenuates real
//!   smoke's signal, the other fabricates a signal with none present).
//! - **Stuck output** freezes the reading regardless of true smoke density.
//! - **Circuit/self-test fault** (ECAM completeness pass, ATA 26 `* DET
//!   FAULT` procedures): real smoke detectors are continuously
//!   self-monitored (loop continuity/power) *separately* from their
//!   optical sensing element -- the same distinction this crate already
//!   draws elsewhere for exactly this class of monitored quantity
//!   (`fire_and_smoke_protection.rs`'s dual detection loop A/B vs. the
//!   zone's actual fire state, and this crate's own `BKR_<id>_STATUS`
//!   discrete). Modelled as a fourth, independent fault field: it does not
//!   touch the reading or the alarm logic at all (`SmokeDetector::step` is
//!   unaffected), it only drives [`SmokeDetectorOutput::circuit_fault`], a
//!   direct pass-through with no threshold invented -- the same mapping
//!   `BKR_<id>_STATUS` already uses.

#[derive(Clone, Copy, Debug, Default)]
pub struct SmokeDetectorFaults {
    /// Optics degradation reducing sensitivity to real smoke, `0.0` clean
    /// .. `1.0` fully desensitised (real smoke produces no reading at all).
    pub sensitivity_loss: f64,
    /// Spurious added signal (contamination/electrical fault), same units
    /// as the obscuration reading (percent obscuration per foot).
    pub false_bias_pct_per_ft: f64,
    /// Frozen output: `1.0` fully stuck at the last reading.
    pub stuck: f64,
    /// Detector circuit/self-test fault, `0.0` healthy .. `1.0` fully
    /// faulted -- orthogonal to the three sensing faults above (module
    /// doc). Drives only [`SmokeDetectorOutput::circuit_fault`].
    pub circuit_fault: f64,
}

/// TSO-C1d-class cargo/lavatory smoke detectors commonly alarm in the
/// low single digits of percent obscuration per foot; 2.0 %/ft is used
/// here as a representative mid-range figure -- GENERIC (no single public
/// number applies across all installations/sensitivity settings).
const ALARM_THRESHOLD_PCT_PER_FT: f64 = 2.0;

#[derive(Clone, Copy, Debug, Default)]
pub struct SmokeDetectorOutput {
    pub reading_pct_per_ft: f64,
    pub alarm: bool,
    /// The detector's own monitored circuit/self-test fault (module doc):
    /// `faults.circuit_fault > 0.0`, independent of `reading_pct_per_ft`/
    /// `alarm`.
    pub circuit_fault: bool,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct SmokeDetector {
    last_reading: f64,
}

impl SmokeDetector {
    pub fn new() -> Self {
        Self { last_reading: 0.0 }
    }

    pub fn step(&mut self, true_obscuration_pct_per_ft: f64, faults: &SmokeDetectorFaults) -> SmokeDetectorOutput {
        let sensitivity = (1.0 - faults.sensitivity_loss.clamp(0.0, 1.0)).max(0.0);
        let healthy_reading = true_obscuration_pct_per_ft.max(0.0) * sensitivity + faults.false_bias_pct_per_ft.max(0.0);
        let stuck = faults.stuck.clamp(0.0, 1.0);
        self.last_reading = healthy_reading * (1.0 - stuck) + self.last_reading * stuck;
        SmokeDetectorOutput { reading_pct_per_ft: self.last_reading, alarm: self.last_reading >= ALARM_THRESHOLD_PCT_PER_FT, circuit_fault: faults.circuit_fault > 0.0 }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn clear_air_never_alarms() {
        let mut d = SmokeDetector::new();
        let out = d.step(0.0, &SmokeDetectorFaults::default());
        assert!(!out.alarm);
    }

    #[test]
    fn healthy_detector_alarms_above_threshold() {
        let mut d = SmokeDetector::new();
        let out = d.step(3.0, &SmokeDetectorFaults::default());
        assert!(out.alarm);
        assert!((out.reading_pct_per_ft - 3.0).abs() < 1e-9);
    }

    #[test]
    fn desensitised_detector_misses_real_smoke_that_would_otherwise_alarm() {
        let mut d = SmokeDetector::new();
        let faults = SmokeDetectorFaults { sensitivity_loss: 0.9, ..Default::default() };
        let out = d.step(3.0, &faults);
        assert!(!out.alarm, "reading {}", out.reading_pct_per_ft);
    }

    #[test]
    fn false_bias_alarms_with_no_smoke_present() {
        let mut d = SmokeDetector::new();
        let faults = SmokeDetectorFaults { false_bias_pct_per_ft: 5.0, ..Default::default() };
        let out = d.step(0.0, &faults);
        assert!(out.alarm);
    }

    #[test]
    fn stuck_detector_freezes_its_reading() {
        let mut d = SmokeDetector::new();
        let first = d.step(0.0, &SmokeDetectorFaults::default());
        assert!(!first.alarm);
        let faults = SmokeDetectorFaults { stuck: 1.0, ..Default::default() };
        let out = d.step(10.0, &faults);
        assert!(!out.alarm, "a stuck-clear detector must not suddenly alarm");
        assert_eq!(out.reading_pct_per_ft, 0.0);
    }

    #[test]
    fn no_nan_at_zero_inputs() {
        let mut d = SmokeDetector::new();
        let out = d.step(0.0, &SmokeDetectorFaults::default());
        assert!(out.reading_pct_per_ft.is_finite());
    }

    #[test]
    fn a_healthy_detector_never_reports_a_circuit_fault() {
        let mut d = SmokeDetector::new();
        let out = d.step(3.0, &SmokeDetectorFaults::default());
        assert!(!out.circuit_fault);
        assert!(out.alarm, "the reading/alarm path must be unaffected by this field's absence");
    }

    #[test]
    fn a_circuit_fault_is_independent_of_the_reading_and_alarm() {
        let mut d = SmokeDetector::new();
        let faults = SmokeDetectorFaults { circuit_fault: 1.0, ..Default::default() };
        // No real smoke present: the fault must raise its own discrete
        // without fabricating an alarm.
        let out = d.step(0.0, &faults);
        assert!(out.circuit_fault);
        assert!(!out.alarm);
        // Real smoke present: the circuit fault does not silence a
        // genuine alarm either -- the two are orthogonal (module doc).
        let out2 = d.step(3.0, &faults);
        assert!(out2.circuit_fault);
        assert!(out2.alarm);
    }

    #[test]
    fn magnitude_zero_on_every_fault_field_is_byte_identical_to_default() {
        let mut a = SmokeDetector::new();
        let mut b = SmokeDetector::new();
        let out_a = a.step(1.5, &SmokeDetectorFaults::default());
        let out_b = b.step(1.5, &SmokeDetectorFaults { sensitivity_loss: 0.0, false_bias_pct_per_ft: 0.0, stuck: 0.0, circuit_fault: 0.0 });
        assert_eq!(out_a.reading_pct_per_ft, out_b.reading_pct_per_ft);
        assert_eq!(out_a.alarm, out_b.alarm);
        assert_eq!(out_a.circuit_fault, out_b.circuit_fault);
        assert!(!out_a.circuit_fault);
    }
}
