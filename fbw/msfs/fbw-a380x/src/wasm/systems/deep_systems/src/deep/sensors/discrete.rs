const NOMINAL_SENSING_GAP_MM: f64 = 8.0;
const HYSTERESIS_MM: f64 = 0.5;

#[derive(Clone, Copy, Debug, Default)]
pub struct ProximitySensorFaults {
    pub gap_error_mm: f64,
    pub stuck_near: f64,
    pub stuck_far: f64,
}

#[derive(Clone, Copy, Debug)]
pub struct ProximitySensor {
    near: bool,
}

impl ProximitySensor {
    pub fn new() -> Self {
        Self { near: false }
    }

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

const FUEL_RELATIVE_PERMITTIVITY: f64 = 2.1;
const WATER_RELATIVE_PERMITTIVITY: f64 = 80.0;
const OIL_RELATIVE_PERMITTIVITY: f64 = 2.2;

#[derive(Clone, Copy, Debug, Default)]
pub struct CapacitanceProbeFaults {
    pub contamination_frac: f64,
    pub open_circuit: f64,
}

pub type FuelProbeFaults = CapacitanceProbeFaults;

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
    let effective_permittivity = contamination_frac * contaminant_permittivity + (1.0 - contamination_frac) * own_permittivity;
    level * (effective_permittivity / own_permittivity)
}

pub fn fuel_probe_indicated_level(true_level_frac: f64, faults: &FuelProbeFaults) -> f64 {
    capacitance_probe_indicated_level(true_level_frac, FUEL_RELATIVE_PERMITTIVITY, WATER_RELATIVE_PERMITTIVITY, faults)
}

pub fn oil_probe_indicated_level(true_level_frac: f64, faults: &CapacitanceProbeFaults) -> f64 {
    capacitance_probe_indicated_level(true_level_frac, OIL_RELATIVE_PERMITTIVITY, WATER_RELATIVE_PERMITTIVITY, faults)
}

const PT100_R0_OHM: f64 = 100.0;
const PT100_ALPHA: f64 = 0.00385;

#[derive(Clone, Copy, Debug, Default)]
pub struct TemperatureSensorFaults {
    pub open_circuit: f64,
    pub short_circuit: f64,
}

pub fn temperature_sensor_reading_c(true_c: f64, sensor_range_c: (f64, f64), faults: &TemperatureSensorFaults) -> f64 {
    let healthy_r = PT100_R0_OHM * (1.0 + PT100_ALPHA * true_c);
    let open = faults.open_circuit.clamp(0.0, 1.0);
    let short = faults.short_circuit.clamp(0.0, 1.0);
    let effective_r = if open >= short {
        healthy_r / (1.0 - open).max(1e-6)
    } else {
        healthy_r * (1.0 - short)
    };
    let reading_c = (effective_r / PT100_R0_OHM - 1.0) / PT100_ALPHA;
    reading_c.clamp(sensor_range_c.0, sensor_range_c.1)
}

#[derive(Clone, Copy, Debug, Default)]
pub struct PressureTransducerFaults {
    pub drift_rate_pa_per_hr: f64,
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
        assert!(sensor.sense(NOMINAL_SENSING_GAP_MM + 0.2, &ProximitySensorFaults::default()));
        assert!(!sensor.sense(NOMINAL_SENSING_GAP_MM + 5.0, &ProximitySensorFaults::default()));
    }

    #[test]
    fn gap_error_shifts_the_switch_point() {
        let mut miscalibrated = ProximitySensor::new();
        let faults = ProximitySensorFaults { gap_error_mm: -5.0, ..Default::default() };
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
        let faults = TemperatureSensorFaults { open_circuit: 0.25, ..Default::default() };
        let reading = temperature_sensor_reading_c(20.0, (-60.0, 200.0), &faults);
        assert!(reading > 20.0 && reading < 200.0, "{reading}");
        assert!((reading - 113.2).abs() < 0.5, "{reading}");
        let worse = TemperatureSensorFaults { open_circuit: 0.5, ..Default::default() };
        assert_eq!(temperature_sensor_reading_c(20.0, (-60.0, 200.0), &worse), 200.0);
    }

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
        let out = t.step(3000.0, &faults, 3600.0);
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
