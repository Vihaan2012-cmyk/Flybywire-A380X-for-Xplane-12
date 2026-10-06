const BASE_TAU_S: f64 = 5.0;

#[derive(Clone, Copy, Debug, Default)]
pub struct BrakeWearPinFaults {
    pub pin_binding: f64,
    pub sender_bias: f64,
    pub open_circuit: f64,
}

#[derive(Clone, Copy, Debug)]
pub struct BrakeWearPin {
    indicated_remaining_frac: f64,
}

impl BrakeWearPin {
    pub fn new(initial_remaining_frac: f64) -> Self {
        Self { indicated_remaining_frac: initial_remaining_frac.clamp(0.0, 1.0) }
    }

    pub fn step(&mut self, true_remaining_frac: f64, faults: &BrakeWearPinFaults, dt_s: f64) -> f64 {
        if faults.open_circuit.clamp(0.0, 1.0) >= 0.98 {
            self.indicated_remaining_frac = 0.0;
            return 0.0;
        }
        let dt = dt_s.max(0.0);
        let true_frac = true_remaining_frac.clamp(0.0, 1.0);
        let binding = faults.pin_binding.clamp(0.0, 1.0);
        if binding < 0.98 {
            let tau = BASE_TAU_S / (1.0 - binding);
            let k = (-dt / tau).exp();
            self.indicated_remaining_frac = true_frac + (self.indicated_remaining_frac - true_frac) * k;
        }
        (self.indicated_remaining_frac + faults.sender_bias).clamp(0.0, 1.0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn healthy_sensor_tracks_true_wear_closely() {
        let mut pin = BrakeWearPin::new(1.0);
        let mut out = 1.0;
        for i in 0..200 {
            let true_frac = 1.0 - i as f64 / 400.0;
            out = pin.step(true_frac, &BrakeWearPinFaults::default(), 1.0);
        }
        assert!((out - 0.5).abs() < 0.02, "{out}");
    }

    #[test]
    fn a_fully_bound_pin_freezes_while_the_real_brake_keeps_wearing() {
        let mut pin = BrakeWearPin::new(0.8);
        let faults = BrakeWearPinFaults { pin_binding: 1.0, ..Default::default() };
        let frozen = pin.step(0.8, &faults, 1.0);
        let mut out = frozen;
        for _ in 0..500 {
            out = pin.step(0.0, &faults, 1.0);
        }
        assert!((out - frozen).abs() < 1e-6, "frozen {frozen} now {out}");
        assert!(out > 0.5, "the dangerous case: indicated life stays high while real life is gone");
    }

    #[test]
    fn partial_binding_makes_indicated_wear_lag_behind_and_overstate_remaining_life() {
        let mut healthy = BrakeWearPin::new(1.0);
        let mut bound = BrakeWearPin::new(1.0);
        let bound_faults = BrakeWearPinFaults { pin_binding: 0.9, ..Default::default() };
        let (mut h, mut b) = (1.0, 1.0);
        for i in 0..300 {
            let true_frac = (1.0 - i as f64 / 300.0).max(0.0);
            h = healthy.step(true_frac, &BrakeWearPinFaults::default(), 1.0);
            b = bound.step(true_frac, &bound_faults, 1.0);
        }
        assert!(b > h, "bound indication {b} should overstate remaining life vs healthy {h}");
    }

    #[test]
    fn sender_bias_offsets_the_reading() {
        let mut pin = BrakeWearPin::new(0.5);
        let faults = BrakeWearPinFaults { sender_bias: -0.1, ..Default::default() };
        let mut out = 0.0;
        for _ in 0..50 {
            out = pin.step(0.5, &faults, 1.0);
        }
        assert!((out - 0.4).abs() < 0.01, "{out}");
    }

    #[test]
    fn open_circuit_reads_a_conservative_zero() {
        let mut pin = BrakeWearPin::new(1.0);
        let faults = BrakeWearPinFaults { open_circuit: 1.0, ..Default::default() };
        assert_eq!(pin.step(1.0, &faults, 1.0), 0.0);
    }

    #[test]
    fn no_nan_at_zero_dt_or_rest() {
        let mut pin = BrakeWearPin::new(0.0);
        let out = pin.step(0.0, &BrakeWearPinFaults::default(), 0.0);
        assert!(out.is_finite());
    }
}
