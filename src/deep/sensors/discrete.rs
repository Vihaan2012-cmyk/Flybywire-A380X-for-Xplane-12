//! Discrete/analogue-secondary sensors that don't fit the air-data family:
//! inductive proximity sensors (gear/door position), capacitance fuel
//! quantity probes, resistance temperature sensors, and pressure
//! transducers. Each is a small, physically grounded model of one common,
//! documented failure mechanism for its sensor class.

// ---------------------------------------------------------------------
// Proximity sensor (inductive target sensing: gear, doors, thrust reverser
// locks).
// ---------------------------------------------------------------------

/// An aircraft inductive proximity sensor switches state when a steel target
/// passes within its sensing gap -- typically adjustable/set during
/// rigging, with published generic figures for this sensor class in the
/// few-mm to ~1 cm range depending on target size (GENERIC: no A380-specific
/// rigging figure is public; 8 mm nominal is a representative generic
/// value). Two well documented real failure modes:
/// - the gap itself drifts out of adjustment (vibration, mounting wear),
///   moving the switch point without the sensor "failing" outright -- this
///   is exactly the kind of defect aircraft maintenance manuals call out
///   proximity-sensor "gap" checks for;
/// - the sensor electronics fail stuck, always reporting near or always
///   reporting far regardless of the real target position.
/// A small hysteresis band (a real inductive sensor's switch point differs
/// slightly for approaching vs. receding targets, to avoid chatter right at
/// the threshold) is included -- GENERIC magnitude.
const NOMINAL_SENSING_GAP_MM: f64 = 8.0;
const HYSTERESIS_MM: f64 = 0.5;

#[derive(Clone, Copy, Debug, Default)]
pub struct ProximitySensorFaults {
    /// Rigging/wear error added to the nominal sensing gap, mm (signed: a
    /// positive value means the sensor triggers "near" later than it
    /// should, i.e. the effective gap it needs is smaller than rigged for).
    pub gap_error_mm: f64,
    /// Stuck reporting "near" (target detected) regardless of the real gap.
    pub stuck_near: f64,
    /// Stuck reporting "far" (no target) regardless of the real gap.
    pub stuck_far: f64,
}

#[derive(Clone, Copy, Debug)]
pub struct ProximitySensor {
    /// Last reported state, for the hysteresis band.
    near: bool,
}

impl ProximitySensor {
    pub fn new() -> Self {
        Self { near: false }
    }

    /// `target_gap_mm`: the true, current mechanical gap between the sensor
    /// and its target (e.g. from the gear/door position model). Returns
    /// `true` when the target is sensed as "near" (in position).
    pub fn sense(&mut self, target_gap_mm: f64, faults: &ProximitySensorFaults) -> bool {
        if faults.stuck_near.clamp(0.0, 1.0) >= 0.5 {
            self.near = true;
            return true;
        }
        if faults.stuck_far.clamp(0.0, 1.0) >= 0.5 {
            self.near = false;
            return false;
        }
        let effective_gap_mm = NOMINAL_SENSING_GAP_MM + faults.gap_error_mm;
        let threshold = if self.near { effective_gap_mm + HYSTERESIS_MM } else { effective_gap_mm - HYSTERESIS_MM };
        self.near = target_gap_mm <= threshold.max(0.0);
        self.near
    }
}

impl Default for ProximitySensor {
    fn default() -> Self {
        Self::new()
    }
}

// ---------------------------------------------------------------------
// Fuel quantity capacitance probe.
// ---------------------------------------------------------------------

/// A capacitance fuel probe is two concentric tubes forming a capacitor;
/// fuel between them raises the capacitance relative to air in proportion
/// to fuel's relative permittivity (Jet A/A-1: commonly cited ~2.1),
/// so the indicating system converts measured capacitance to quantity
/// assuming that known permittivity. Water is far more polar
/// (relative permittivity ~80, standard physical chemistry data) and, being
/// denser than fuel, settles at the bottom of the tank and thus at the
/// bottom of the probe -- so any water content raises the *measured*
/// capacitance (and therefore the *indicated* quantity) above the true
/// liquid volume, a real and documented capacitance-fuel-gauging error
/// mode. Modelled directly from the physics: capacitance is proportional to
/// a permittivity-weighted sum of the water and fuel columns, and the
/// indicating system's fixed fuel-permittivity assumption is what turns that
/// into an over-reading.
const FUEL_RELATIVE_PERMITTIVITY: f64 = 2.1;
const WATER_RELATIVE_PERMITTIVITY: f64 = 80.0;
/// MIL-PRF-23699 turbine oil (e.g. Mobil Jet Oil II) relative permittivity:
/// commonly cited in the 2.1-2.3 range for synthetic ester turbine oils,
/// the same order as jet fuel -- 2.2 is the mid value used here.
const OIL_RELATIVE_PERMITTIVITY: f64 = 2.2;

#[derive(Clone, Copy, Debug, Default)]
pub struct CapacitanceProbeFaults {
    /// Fraction of the wetted probe length that is the contaminant (water)
    /// rather than the working liquid (water settles below a less dense
    /// liquid, so this is the bottom fraction of the liquid column).
    pub contamination_frac: f64,
    /// The probe or its wiring is open-circuit: the capacitance bridge
    /// reads no signal, and the indicating system defaults to zero rather
    /// than extrapolating -- the standard fail-safe behaviour for an open
    /// capacitance sensor input.
    pub open_circuit: f64,
}

/// The same alias under the fuel-specific name used before this was
/// generalised (kept so `registry.rs`'s `model_field` citations referring
/// to `FuelProbeFaults` and existing tests both stay valid).
pub type FuelProbeFaults = CapacitanceProbeFaults;

/// Reports the *indicated* liquid level fraction (what the cockpit gauge
/// would show, `0.0..~1.2` -- it can read above `1.0` with enough
/// contamination) for a probe whose true wetted length fraction is
/// `true_level_frac`, calibrated for a liquid of `own_permittivity` and
/// contaminated by a substance of `contaminant_permittivity`. This is the
/// shared physics behind both [`fuel_probe_indicated_level`] (fuel,
/// contaminated by water) and [`oil_probe_indicated_level`] (oil,
/// contaminated by water -- a real, documented issue when a failed
/// fuel-cooled/air-cooled oil cooler or a breached seal lets water into an
/// oil system).
fn capacitance_probe_indicated_level(
    true_level_frac: f64,
    own_permittivity: f64,
    contaminant_permittivity: f64,
    faults: &CapacitanceProbeFaults,
) -> f64 {
    if faults.open_circuit.clamp(0.0, 1.0) >= 0.98 {
        return 0.0;
    }
    let level = true_level_frac.clamp(0.0, 1.0);
    let contamination_frac = faults.contamination_frac.clamp(0.0, 1.0);
    // Effective permittivity of the wetted column: a contamination_frac
    // share at the contaminant's permittivity, the rest at the working
    // liquid's own.
    let effective_permittivity = contamination_frac * contaminant_permittivity + (1.0 - contamination_frac) * own_permittivity;
    // The indicating system's calibration assumes the pure working liquid;
    // it converts measured capacitance back to a length using
    // `own_permittivity`, so any higher effective permittivity is read
    // back as a proportionally longer (higher) wetted column.
    level * (effective_permittivity / own_permittivity)
}

/// Fuel quantity capacitance probe, contaminated by water.
pub fn fuel_probe_indicated_level(true_level_frac: f64, faults: &FuelProbeFaults) -> f64 {
    capacitance_probe_indicated_level(true_level_frac, FUEL_RELATIVE_PERMITTIVITY, WATER_RELATIVE_PERMITTIVITY, faults)
}

/// Engine oil quantity capacitance probe, contaminated by water (a failed
/// oil cooler or breached seal letting coolant/water into the oil system is
/// a real, documented cause of this same capacitance-gauging error mode as
/// the fuel probe above, just on a different liquid).
pub fn oil_probe_indicated_level(true_level_frac: f64, faults: &CapacitanceProbeFaults) -> f64 {
    capacitance_probe_indicated_level(true_level_frac, OIL_RELATIVE_PERMITTIVITY, WATER_RELATIVE_PERMITTIVITY, faults)
}

// ---------------------------------------------------------------------
// Resistance temperature sensor (RTD-style: fuel, oil, hydraulic, bleed
// temperature probes).
// ---------------------------------------------------------------------

/// A platinum RTD's resistance varies close to linearly with temperature:
/// `R = R0 * (1 + alpha*(T - T0))`, the standard simplified Callendar-Van
/// Dusen relation; `alpha = 0.00385 ohm/ohm/C` is the published IEC 60751
/// Pt100 temperature coefficient (a public, standardised sensor-industry
/// figure, not GENERIC). `R0 = 100 ohm` at 0 C is the Pt100's defining
/// property. An open-circuit fault drives the effective resistance toward
/// infinity (no current path); a short-circuit fault drives it toward zero.
/// Whatever converts that resistance back to a temperature for the cockpit
/// gauge naturally reports a pegged extreme in either case -- exactly the
/// documented real behaviour of resistance-based temperature indications
/// failing open or shorted (an open RTD input on real aircraft indicating
/// systems commonly pegs the gauge to a fixed off-scale value, and a short
/// pegs it the other way).
const PT100_R0_OHM: f64 = 100.0;
const PT100_ALPHA: f64 = 0.00385;

#[derive(Clone, Copy, Debug, Default)]
pub struct TemperatureSensorFaults {
    pub open_circuit: f64,
    pub short_circuit: f64,
}

/// Reports the resistance-derived temperature, C, clamped to
/// `sensor_range_c` (the indicating system's own display/electrical
/// range -- an open/shorted input cannot report outside what its
/// conversion electronics can represent).
pub fn temperature_sensor_reading_c(true_c: f64, sensor_range_c: (f64, f64), faults: &TemperatureSensorFaults) -> f64 {
    let healthy_r = PT100_R0_OHM * (1.0 + PT100_ALPHA * true_c);
    let open = faults.open_circuit.clamp(0.0, 1.0);
    let short = faults.short_circuit.clamp(0.0, 1.0);
    // Open dominates when both are present (a fully open circuit has no
    // resistance path left to short).
    let effective_r = if open >= short {
        healthy_r / (1.0 - open).max(1e-6)
    } else {
        healthy_r * (1.0 - short)
    };
    let reading_c = (effective_r / PT100_R0_OHM - 1.0) / PT100_ALPHA;
    reading_c.clamp(sensor_range_c.0, sensor_range_c.1)
}

// ---------------------------------------------------------------------
// Pressure transducer (hydraulic, pneumatic, oil -- anywhere a strain-gauge
// or capacitive transducer reports pressure electrically rather than
// mechanically).
// ---------------------------------------------------------------------

/// Two common transducer failure modes: a slow zero-drift as the sensing
/// element (diaphragm/strain gauge) fatigues or its bonding degrades with
/// thermal cycling (a real, documented long-term transducer aging
/// mechanism), and a stuck-output fault (a seized diaphragm or a failed
/// output stage holds the last value regardless of the true pressure).
#[derive(Clone, Copy, Debug, Default)]
pub struct PressureTransducerFaults {
    /// Zero-drift rate, Pa per hour (signed). GENERIC: no published
    /// long-term drift spec for a generic aircraft transducer; this is a
    /// slow-aging bias, not sensor noise.
    pub drift_rate_pa_per_hr: f64,
    /// Diaphragm/output stuck: `1.0` fully frozen at the last output.
    pub stuck: f64,
}

#[derive(Clone, Copy, Debug)]
pub struct PressureTransducer {
    drift_bias_pa: f64,
    last_output_pa: f64,
}

impl PressureTransducer {
    pub fn new(initial_pa: f64) -> Self {
        Self { drift_bias_pa: 0.0, last_output_pa: initial_pa }
    }

    pub fn step(&mut self, true_pa: f64, faults: &PressureTransducerFaults, dt_s: f64) -> f64 {
        let dt = dt_s.max(0.0);
        self.drift_bias_pa += faults.drift_rate_pa_per_hr * dt / 3600.0;
        let healthy_output_pa = true_pa + self.drift_bias_pa;
        let stuck = faults.stuck.clamp(0.0, 1.0);
        self.last_output_pa = healthy_output_pa * (1.0 - stuck) + self.last_output_pa * stuck;
        self.last_output_pa
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // ---- Proximity sensor.

    #[test]
    fn proximity_sensor_reports_near_when_gap_closes_and_far_when_it_opens() {
        let mut sensor = ProximitySensor::new();
        assert!(!sensor.sense(50.0, &ProximitySensorFaults::default()));
        assert!(sensor.sense(2.0, &ProximitySensorFaults::default()));
        assert!(!sensor.sense(50.0, &ProximitySensorFaults::default()));
    }

    #[test]
    fn proximity_sensor_hysteresis_avoids_chatter_right_at_the_threshold() {
        let mut sensor = ProximitySensor::new();
        assert!(sensor.sense(2.0, &ProximitySensorFaults::default()));
        // Just above the nominal gap but still inside the hysteresis band:
        // should still read near since it was already near.
        assert!(sensor.sense(NOMINAL_SENSING_GAP_MM + 0.2, &ProximitySensorFaults::default()));
        // Clearly open: now reads far.
        assert!(!sensor.sense(NOMINAL_SENSING_GAP_MM + 5.0, &ProximitySensorFaults::default()));
    }

    #[test]
    fn gap_error_shifts_the_switch_point() {
        let mut miscalibrated = ProximitySensor::new();
        let faults = ProximitySensorFaults { gap_error_mm: -5.0, ..Default::default() };
        // A gap that would trigger a healthy sensor no longer does with a
        // tighter effective threshold.
        assert!(!miscalibrated.sense(6.0, &faults));
        let mut healthy = ProximitySensor::new();
        assert!(healthy.sense(6.0, &ProximitySensorFaults::default()));
    }

    #[test]
    fn stuck_near_and_stuck_far_ignore_the_real_gap() {
        let mut stuck_near = ProximitySensor::new();
        let near_faults = ProximitySensorFaults { stuck_near: 1.0, ..Default::default() };
        assert!(stuck_near.sense(500.0, &near_faults));

        let mut stuck_far = ProximitySensor::new();
        let far_faults = ProximitySensorFaults { stuck_far: 1.0, ..Default::default() };
        assert!(!stuck_far.sense(0.0, &far_faults));
    }

    // ---- Fuel probe.

    #[test]
    fn healthy_fuel_probe_reads_true_level() {
        let level = fuel_probe_indicated_level(0.5, &FuelProbeFaults::default());
        assert!((level - 0.5).abs() < 1e-9);
    }

    #[test]
    fn water_contamination_reads_high() {
        let faults = FuelProbeFaults { contamination_frac: 0.2, ..Default::default() };
        let level = fuel_probe_indicated_level(0.5, &faults);
        assert!(level > 0.5, "{level}");
    }

    #[test]
    fn open_circuit_fuel_probe_reads_zero() {
        let faults = FuelProbeFaults { open_circuit: 1.0, ..Default::default() };
        assert_eq!(fuel_probe_indicated_level(0.8, &faults), 0.0);
    }

    // ---- Oil probe (shares the fuel probe's capacitance physics, own
    // permittivity, a real distinct instance -- water-in-oil contamination
    // is a documented issue independent of water-in-fuel).

    #[test]
    fn healthy_oil_probe_reads_true_level() {
        let level = oil_probe_indicated_level(0.6, &CapacitanceProbeFaults::default());
        assert!((level - 0.6).abs() < 1e-9);
    }

    #[test]
    fn water_in_oil_reads_high_same_as_water_in_fuel() {
        let faults = CapacitanceProbeFaults { contamination_frac: 0.2, ..Default::default() };
        let level = oil_probe_indicated_level(0.6, &faults);
        assert!(level > 0.6, "{level}");
    }

    // ---- Temperature sensor.

    #[test]
    fn healthy_rtd_reads_true_temperature() {
        let reading = temperature_sensor_reading_c(85.0, (-60.0, 200.0), &TemperatureSensorFaults::default());
        assert!((reading - 85.0).abs() < 1e-6, "{reading}");
    }

    #[test]
    fn open_circuit_pegs_the_reading_to_the_top_of_range() {
        let faults = TemperatureSensorFaults { open_circuit: 1.0, ..Default::default() };
        let reading = temperature_sensor_reading_c(85.0, (-60.0, 200.0), &faults);
        assert_eq!(reading, 200.0);
    }

    #[test]
    fn short_circuit_pegs_the_reading_to_the_bottom_of_range() {
        let faults = TemperatureSensorFaults { short_circuit: 1.0, ..Default::default() };
        let reading = temperature_sensor_reading_c(85.0, (-60.0, 200.0), &faults);
        assert_eq!(reading, -60.0);
    }

    #[test]
    fn partial_open_circuit_reads_high_but_not_pegged() {
        let faults = TemperatureSensorFaults { open_circuit: 0.5, ..Default::default() };
        let reading = temperature_sensor_reading_c(20.0, (-60.0, 200.0), &faults);
        assert!(reading > 20.0 && reading < 200.0, "{reading}");
    }

    // ---- Pressure transducer.

    #[test]
    fn healthy_transducer_tracks_true_pressure() {
        let mut t = PressureTransducer::new(3000.0);
        let out = t.step(3500.0, &PressureTransducerFaults::default(), 1.0);
        assert!((out - 3500.0).abs() < 1e-6);
    }

    #[test]
    fn drift_accumulates_over_time() {
        let mut t = PressureTransducer::new(3000.0);
        let faults = PressureTransducerFaults { drift_rate_pa_per_hr: 3600.0, ..Default::default() };
        let out = t.step(3000.0, &faults, 3600.0); // one hour
        assert!((out - 3000.0 - 3600.0).abs() < 1e-6, "{out}");
    }

    #[test]
    fn stuck_transducer_freezes_its_output() {
        let mut t = PressureTransducer::new(3000.0);
        let out1 = t.step(3000.0, &PressureTransducerFaults::default(), 1.0);
        let faults = PressureTransducerFaults { stuck: 1.0, ..Default::default() };
        let out2 = t.step(9000.0, &faults, 1.0);
        assert_eq!(out1, out2);
    }

    #[test]
    fn no_nan_at_zero_dt_or_rest() {
        let mut t = PressureTransducer::new(0.0);
        let out = t.step(0.0, &PressureTransducerFaults::default(), 0.0);
        assert!(out.is_finite());
        let reading = temperature_sensor_reading_c(0.0, (-60.0, 200.0), &TemperatureSensorFaults::default());
        assert!(reading.is_finite());
        let level = fuel_probe_indicated_level(0.0, &FuelProbeFaults::default());
        assert!(level.is_finite());
    }
}
