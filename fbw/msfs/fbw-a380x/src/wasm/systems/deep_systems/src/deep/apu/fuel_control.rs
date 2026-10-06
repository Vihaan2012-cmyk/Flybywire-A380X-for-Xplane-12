use super::actuator::Actuator;
use super::params;

#[derive(Clone, Copy, Debug, Default)]
pub struct FuelControlFaults {
    pub metering_valve_jam: f64,
}

#[derive(Clone, Debug)]
pub struct FuelControl {
    metering_valve: Actuator,
    max_flow_kg_s: f64,
}

impl FuelControl {
    pub fn new(max_flow_kg_s: f64) -> Self {
        Self {
            metering_valve: Actuator::new(0.0, params::FUEL_METERING_VALVE_RATE_PER_S),
            max_flow_kg_s: max_flow_kg_s.max(0.0),
        }
    }

    pub fn metering_valve_position_frac(&self) -> f64 {
        self.metering_valve.position_frac()
    }

    pub fn step(
        &mut self,
        commanded_flow_kg_s: f64,
        solenoid_open: bool,
        faults: &FuelControlFaults,
        dt_s: f64,
    ) -> f64 {
        let commanded_frac = if self.max_flow_kg_s > 1e-9 {
            (commanded_flow_kg_s.max(0.0) / self.max_flow_kg_s).clamp(0.0, 1.0)
        } else {
            0.0
        };
        let pos = self.metering_valve.step(commanded_frac, faults.metering_valve_jam, dt_s);
        if solenoid_open {
            pos * self.max_flow_kg_s
        } else {
            0.0
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_closed_solenoid_delivers_no_fuel_regardless_of_command() {
        let mut fc = FuelControl::new(0.05);
        let flow = fc.step(0.05, false, &FuelControlFaults::default(), 1.0);
        assert_eq!(flow, 0.0);
    }

    #[test]
    fn a_healthy_valve_settles_on_the_commanded_flow() {
        let mut fc = FuelControl::new(0.05);
        let mut flow = 0.0;
        for _ in 0..20 {
            flow = fc.step(0.03, true, &FuelControlFaults::default(), 0.5);
        }
        assert!((flow - 0.03).abs() < 1e-6, "{flow}");
    }

    #[test]
    fn the_valve_cannot_jump_instantly_to_a_new_command() {
        let mut fc = FuelControl::new(0.05);
        let flow = fc.step(0.05, true, &FuelControlFaults::default(), 0.05);
        assert!(flow < 0.05, "{flow}");
        assert!(flow > 0.0);
    }

    #[test]
    fn a_jammed_valve_freezes_and_ignores_later_commands() {
        let mut fc = FuelControl::new(0.05);
        for _ in 0..20 {
            fc.step(0.02, true, &FuelControlFaults::default(), 0.5);
        }
        let frozen_at = fc.metering_valve_position_frac();
        let mut flow = 0.0;
        for _ in 0..20 {
            flow = fc.step(0.0, true, &FuelControlFaults { metering_valve_jam: 1.0 }, 0.5);
        }
        assert!((fc.metering_valve_position_frac() - frozen_at).abs() < 1e-9);
        assert!(flow > 0.0, "should still be delivering the frozen flow, got {flow}");
    }

    #[test]
    fn zero_max_flow_never_produces_nan() {
        let mut fc = FuelControl::new(0.0);
        let flow = fc.step(1.0, true, &FuelControlFaults::default(), 1.0);
        assert!(flow.is_finite());
        assert_eq!(flow, 0.0);
    }
}
