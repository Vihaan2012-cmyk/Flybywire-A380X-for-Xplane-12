#[derive(Clone, Copy, Debug, Default)]
pub struct TransducerFaults {
    pub drift: f64,
    pub open_circuit: f64,
    pub intermittent: f64,
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct TransducerReading {
    pub value_rad: f64,
    pub valid: bool,
}

const MAX_DRIFT_RATE_RAD_S: f64 = 0.0015;
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

pub struct DualTransducer {
    pub a: PositionTransducer,
    pub b: PositionTransducer,
    disagree: super::surface::AsymmetryMonitor,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct DualTransducerOutput {
    pub a: TransducerReading,
    pub b: TransducerReading,
    pub consolidated_rad: Option<f64>,
    pub monitoring_fault: bool,
}

impl DualTransducer {
    pub fn new(disagree_threshold_timer_s: f64) -> Self {
        Self { a: PositionTransducer::new(), b: PositionTransducer::new(), disagree: super::surface::AsymmetryMonitor::new(disagree_threshold_timer_s) }
    }

    pub fn step(&mut self, true_angle_rad: f64, faults_a: &TransducerFaults, faults_b: &TransducerFaults, disagree_threshold_rad: f64, dt_s: f64) -> DualTransducerOutput {
        let a = self.a.step(true_angle_rad, faults_a, dt_s);
        let b = self.b.step(true_angle_rad, faults_b, dt_s);

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
        let drifting_b = TransducerFaults { drift: 1.0, ..Default::default() };
        let mut tripped = false;
        for _ in 0..10_000 {
            let out = d.step(0.0, &TransducerFaults::default(), &drifting_b, 0.02, DT);
            tripped |= out.monitoring_fault;
        }
        assert!(tripped);
    }
}
