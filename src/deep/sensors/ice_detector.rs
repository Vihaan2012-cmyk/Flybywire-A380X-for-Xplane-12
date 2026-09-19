//! Ice detector: a small vibrating probe (the public operating principle of
//! Rosemount/Goodrich-style magnetostrictive ice detectors such as the
//! 0871LH1, widely described in icing-instrumentation literature) that is
//! driven at its mechanical resonant frequency. Ice accreting on the probe
//! adds mass, which lowers that resonant frequency (basic vibration
//! mechanics: for a resonant mass-spring-like system, `f = f0*sqrt(k/(m0+dm))`,
//! so added mass mass-loads the probe and pulls its frequency down -- the
//! same physical principle a quartz-crystal microbalance uses to sense
//! deposited mass). When the frequency has dropped by a calibrated amount
//! (corresponding to a small, known ice thickness -- commonly cited public
//! figures for this class of detector are a fraction of a millimetre), the
//! detector reports ICE DETECTED and fires its own internal heater to shed
//! the ice and reset, then resumes monitoring -- a duty-cycled detect/deice
//! loop, not a one-shot latch.
//!
//! ## Faults
//! - A failed deice heater lets ice keep accreting once detected instead of
//!   shedding: the frequency keeps falling and the discrete output latches
//!   ICE DETECTED indefinitely while icing conditions persist (a real,
//!   physically distinct failure signature from a healthy duty cycle).
//! - Frequency-sensing electronics drift/failure biases the *apparent*
//!   frequency shift, which can either mask real accretion (false negative)
//!   or fabricate a shift with no ice present (false positive) --
//!   modelled as a signed bias on the sensed shift.
//! - Physical probe damage (bent probe, corrosion) shifts the *undamaged*
//!   baseline resonant frequency the detector calibrates against, which
//!   the detection threshold logic below shows as a permanent offset.

/// Mass-loading sensitivity: fractional frequency drop per fractional mass
/// added, for a resonant system with the stiffness held constant
/// (`f = f0*sqrt(m0/(m0+dm))`, linearised for small `dm/m0`:
/// `df/f0 ~= -0.5*dm/m0`). This is the standard small-signal result for any
/// resonant mass-sensing device (the same relation a quartz-crystal
/// microbalance uses), not GENERIC.
fn fractional_frequency_drop(added_mass_kg: f64, effective_probe_mass_kg: f64) -> f64 {
    0.5 * added_mass_kg / effective_probe_mass_kg.max(1e-9)
}

/// Effective vibrating mass of the probe tip, kg. GENERIC: a small strut,
/// order a few grams (no public figure for the exact mass of this specific
/// component).
const PROBE_EFFECTIVE_MASS_KG: f64 = 0.003;
/// Ice mass, as a fraction of the probe's own effective mass, that trips
/// detection. GENERIC: chosen so the detector trips at a small, sub-
/// millimetre-scale accretion (consistent with the publicly cited class
/// figure in the module docs) well before the probe's own mass has
/// meaningfully changed its dynamics.
const DETECTION_MASS_FRACTION: f64 = 0.05;
/// Icing accretion rate onto the small probe, kg/s per (g/m^3 LWC) per
/// (m/s TAS), scaled down from the pitot's own frontal-catch relation by
/// the much smaller probe area exposed. GENERIC (same frontal-catch-area
/// reasoning as `pitot.rs`/`aoa_vane.rs`, independently sized for a probe
/// this small).
const ACCUM_KG_S_PER_LWC_TAS: f64 = 4e-7;
/// Deice heater duty: once tripped, the probe sheds its ice over this many
/// seconds (a real deice heater is sized to clear the small probe quickly
/// -- GENERIC, order of a few seconds, much faster than the pitot/AoA
/// probes' own heaters need to prevent accretion in the first place, since
/// this heater only has to clear a tiny mass rather than continuously
/// resist icing).
const DEICE_SECONDS: f64 = 3.0;

#[derive(Clone, Copy, Debug, Default)]
pub struct IceDetectorFaults {
    /// The deice heater cannot shed accreted ice: `1.0` fully failed (no
    /// deicing at all once tripped).
    pub heater_failure: f64,
    /// Signed bias on the sensed fractional frequency shift (electronics
    /// drift/failure): positive fabricates a shift (false positive risk),
    /// negative masks one (false negative risk).
    pub frequency_sensor_bias: f64,
    /// Physical probe damage offsetting the calibrated baseline, same sign
    /// convention as `frequency_sensor_bias`.
    pub probe_damage_bias: f64,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct IceDetectorOutput {
    pub ice_detected: bool,
    /// Accreted ice mass on the probe, kg (diagnostic).
    pub ice_kg: f64,
    /// Currently running its deice heater.
    pub deicing: bool,
}

#[derive(Clone, Copy, Debug)]
pub struct IceDetector {
    ice_kg: f64,
    deice_remaining_s: f64,
}

impl IceDetector {
    pub fn new() -> Self {
        Self { ice_kg: 0.0, deice_remaining_s: 0.0 }
    }

    /// `sat_c`, `tas_ms`, `lwc_gm3`: the local icing conditions at the
    /// probe (same convention as the other probes in this directory).
    pub fn step(&mut self, sat_c: f64, tas_ms: f64, lwc_gm3: f64, faults: &IceDetectorFaults, dt_s: f64) -> IceDetectorOutput {
        let dt = dt_s.max(0.0);

        if self.deice_remaining_s > 0.0 {
            // Deicing: sheds mass linearly over the deice cycle rather than
            // an instant reset, so a `step()` called mid-cycle sees partial
            // clearing.
            let clear_frac = (dt / DEICE_SECONDS.max(1e-6)).min(1.0);
            self.ice_kg = (self.ice_kg * (1.0 - clear_frac)).max(0.0);
            self.deice_remaining_s = (self.deice_remaining_s - dt).max(0.0);
        } else if sat_c < 0.0 && lwc_gm3 > 0.0 {
            self.ice_kg += ACCUM_KG_S_PER_LWC_TAS * lwc_gm3.max(0.0) * tas_ms.max(0.0) * dt;
        }

        let true_shift = fractional_frequency_drop(self.ice_kg, PROBE_EFFECTIVE_MASS_KG);
        let sensed_shift = true_shift + faults.frequency_sensor_bias + faults.probe_damage_bias;
        let ice_detected = sensed_shift >= DETECTION_MASS_FRACTION * 0.5;

        if ice_detected && self.deice_remaining_s <= 0.0 {
            let heater_ok = faults.heater_failure.clamp(0.0, 1.0) < 0.98;
            if heater_ok {
                self.deice_remaining_s = DEICE_SECONDS;
            }
            // A failed heater simply never starts a deice cycle: ice_kg
            // keeps accumulating next tick (handled by the branch above),
            // so `ice_detected` latches true for as long as icing
            // conditions persist -- the documented failure signature.
        }

        IceDetectorOutput { ice_detected, ice_kg: self.ice_kg, deicing: self.deice_remaining_s > 0.0 }
    }
}

impl Default for IceDetector {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn no_icing_conditions_never_trips() {
        let mut d = IceDetector::new();
        let mut out = IceDetectorOutput::default();
        for _ in 0..1000 {
            out = d.step(15.0, 200.0, 0.0, &IceDetectorFaults::default(), 0.5);
        }
        assert!(!out.ice_detected);
        assert_eq!(out.ice_kg, 0.0);
    }

    #[test]
    fn healthy_detector_cycles_detect_then_deice() {
        let mut d = IceDetector::new();
        let faults = IceDetectorFaults::default();
        let mut tripped = false;
        let mut deiced_after_trip = false;
        for _ in 0..2000 {
            let out = d.step(-20.0, 200.0, 0.8, &faults, 0.1);
            if out.ice_detected {
                tripped = true;
            }
            if tripped && out.deicing {
                deiced_after_trip = true;
            }
        }
        assert!(tripped, "expected the detector to trip in heavy icing");
        assert!(deiced_after_trip, "expected the healthy detector to run its deice cycle after tripping");
    }

    #[test]
    fn failed_heater_latches_ice_detected_instead_of_cycling() {
        let mut d = IceDetector::new();
        let faults = IceDetectorFaults { heater_failure: 1.0, ..Default::default() };
        let mut out = IceDetectorOutput::default();
        for _ in 0..2000 {
            out = d.step(-20.0, 200.0, 0.8, &faults, 0.1);
        }
        assert!(out.ice_detected);
        assert!(!out.deicing, "a fully failed heater never starts a deice cycle");
        assert!(out.ice_kg > 0.0);
    }

    #[test]
    fn frequency_sensor_negative_bias_can_mask_real_icing() {
        let mut masked = IceDetector::new();
        let mut healthy = IceDetector::new();
        let masking_faults = IceDetectorFaults { frequency_sensor_bias: -1.0, ..Default::default() };
        let mut masked_out = IceDetectorOutput::default();
        let mut healthy_out = IceDetectorOutput::default();
        for _ in 0..50 {
            masked_out = masked.step(-20.0, 200.0, 0.8, &masking_faults, 0.1);
            healthy_out = healthy.step(-20.0, 200.0, 0.8, &IceDetectorFaults::default(), 0.1);
        }
        assert!(!masked_out.ice_detected);
        assert!(healthy_out.ice_detected);
    }

    #[test]
    fn probe_damage_bias_can_cause_a_false_positive_with_no_ice() {
        let mut d = IceDetector::new();
        let faults = IceDetectorFaults { probe_damage_bias: 1.0, ..Default::default() };
        // Warm, dry conditions: no physical icing at all.
        let out = d.step(20.0, 200.0, 0.0, &faults, 0.1);
        assert!(out.ice_detected);
        assert_eq!(out.ice_kg, 0.0);
    }

    #[test]
    fn no_nan_at_zero_dt_or_rest() {
        let mut d = IceDetector::new();
        let out = d.step(0.0, 0.0, 0.0, &IceDetectorFaults::default(), 0.0);
        assert!(out.ice_kg.is_finite());
    }
}
