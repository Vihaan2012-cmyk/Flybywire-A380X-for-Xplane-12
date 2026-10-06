const TIME_CONSTANT_S: f64 = 0.15;
const PICKOFF_BIAS_MAX_KG_S: f64 = 5.0;

#[derive(Clone, Copy, Debug, Default)]
pub struct PickoffFaults {
    pub bias_frac_of_design: f64,
    pub frozen: f64,
}

#[derive(Clone, Copy, Debug)]
pub struct FlowTransmitter {
    rotor_kg_s: f64,
    channel_a_hold: f64,
    channel_b_hold: f64,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct FlowReading {
    pub true_flow_kg_s: f64,
    pub channel_a_kg_s: f64,
    pub channel_b_kg_s: f64,
}

impl FlowTransmitter {
    pub fn new() -> Self {
        Self { rotor_kg_s: 0.0, channel_a_hold: 0.0, channel_b_hold: 0.0 }
    }

    pub fn step(&mut self, actual_kg_s: f64, channel_a: &PickoffFaults, channel_b: &PickoffFaults, dt_s: f64) -> FlowReading {
        let dt = dt_s.max(0.0);
        let k = 1.0 - (-dt / TIME_CONSTANT_S).exp();
        self.rotor_kg_s += (actual_kg_s.max(0.0) - self.rotor_kg_s) * k;
        let a = self.pickoff_channel(true, self.rotor_kg_s, channel_a);
        let b = self.pickoff_channel(false, self.rotor_kg_s, channel_b);
        FlowReading { true_flow_kg_s: self.rotor_kg_s, channel_a_kg_s: a, channel_b_kg_s: b }
    }

    fn pickoff_channel(&mut self, is_a: bool, rotor_kg_s: f64, faults: &PickoffFaults) -> f64 {
        let biased = rotor_kg_s + faults.bias_frac_of_design.clamp(-1.0, 1.0) * PICKOFF_BIAS_MAX_KG_S;
        let frozen = faults.frozen.clamp(0.0, 1.0);
        let hold = if is_a { &mut self.channel_a_hold } else { &mut self.channel_b_hold };
        *hold = *hold * frozen + biased * (1.0 - frozen);
        *hold
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_rotor_settles_to_the_true_flow_with_no_faults() {
        let mut ft = FlowTransmitter::new();
        let mut r = FlowReading::default();
        for _ in 0..200 {
            r = ft.step(3.4, &PickoffFaults::default(), &PickoffFaults::default(), 0.02);
        }
        assert!((r.channel_a_kg_s - 3.4).abs() < 0.05);
        assert!((r.channel_b_kg_s - 3.4).abs() < 0.05);
        assert!((r.true_flow_kg_s - 3.4).abs() < 0.05);
    }

    #[test]
    fn zero_flow_at_rest_gives_no_nan() {
        let mut ft = FlowTransmitter::new();
        let r = ft.step(0.0, &PickoffFaults::default(), &PickoffFaults::default(), 0.0);
        assert_eq!(r.channel_a_kg_s, 0.0);
        assert!(!r.channel_a_kg_s.is_nan());
    }

    #[test]
    fn a_frozen_channel_holds_its_last_reading_while_the_other_tracks() {
        let mut ft = FlowTransmitter::new();
        for _ in 0..200 {
            ft.step(3.0, &PickoffFaults::default(), &PickoffFaults::default(), 0.02);
        }
        let frozen = PickoffFaults { frozen: 1.0, ..Default::default() };
        let mut last_a = 0.0;
        for _ in 0..200 {
            let r = ft.step(1.0, &frozen, &PickoffFaults::default(), 0.02);
            last_a = r.channel_a_kg_s;
        }
        let r = ft.step(1.0, &frozen, &PickoffFaults::default(), 0.02);
        assert!((r.channel_a_kg_s - last_a).abs() < 1e-9);
        assert!(r.channel_b_kg_s < last_a, "b should have tracked the drop to 1.0 kg/s");
    }

    #[test]
    fn a_biased_channel_disagrees_with_a_healthy_one() {
        let mut ft = FlowTransmitter::new();
        let biased = PickoffFaults { bias_frac_of_design: 1.0, ..Default::default() };
        let mut r = FlowReading::default();
        for _ in 0..200 {
            r = ft.step(3.0, &biased, &PickoffFaults::default(), 0.02);
        }
        assert!((r.channel_a_kg_s - r.channel_b_kg_s).abs() > 1.0);
    }
}
