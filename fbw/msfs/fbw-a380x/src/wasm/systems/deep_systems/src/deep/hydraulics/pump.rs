pub const PSI_PA: f64 = 6894.757;
const CUBIC_INCH_M3: f64 = 1.6387064e-5;

fn interpolate9(breakpoints: &[f64; 9], values: &[f64; 9], x: f64) -> f64 {
    if x <= breakpoints[0] {
        return values[0];
    }
    if x >= breakpoints[8] {
        return values[8];
    }
    for i in 0..8 {
        if x >= breakpoints[i] && x <= breakpoints[i + 1] {
            let span = breakpoints[i + 1] - breakpoints[i];
            let t = if span > 0.0 { (x - breakpoints[i]) / span } else { 0.0 };
            return values[i] + t * (values[i + 1] - values[i]);
        }
    }
    values[8]
}

#[derive(Clone, Copy, Debug)]
pub struct PumpCharacteristics {
    pressure_breakpoints_psi: [f64; 9],
    displacement_in3: [f64; 9],
}
impl PumpCharacteristics {
    pub fn a380_edp() -> Self {
        Self {
            pressure_breakpoints_psi: [0.0, 500.0, 1000.0, 2900.0, 4790.0, 5150.0, 5225.0, 5350.0, 5500.0],
            displacement_in3: [2.8, 2.8, 2.8, 2.8, 2.6, 0.0, 0.0, 0.0, 0.0],
        }
    }
    pub fn a380_electric() -> Self {
        Self {
            pressure_breakpoints_psi: [0.0, 2000.0, 3000.0, 4000.0, 5000.0, 5100.0, 5200.0, 5300.0, 5350.0],
            displacement_in3: [0.294525, 0.28875, 0.2858625, 0.231, 0.17325, 0.0, 0.0, 0.0, 0.0],
        }
    }
    pub fn displacement_in3(&self, outlet_gauge_pa: f64) -> f64 {
        interpolate9(&self.pressure_breakpoints_psi, &self.displacement_in3, outlet_gauge_pa / PSI_PA)
    }
}

const AIR_PRESSURE_BREAKPTS_PSI: [f64; 9] = [0.0, 5.0, 10.0, 15.0, 20.0, 30.0, 50.0, 70.0, 100.0];
const CAVITATION_MAP_RATIO: [f64; 9] = [0.0, 0.1, 0.6, 0.8, 0.9, 1.0, 1.0, 1.0, 1.0];
pub fn cavitation_efficiency(inlet_gauge_pa: f64) -> f64 {
    interpolate9(&AIR_PRESSURE_BREAKPTS_PSI, &CAVITATION_MAP_RATIO, inlet_gauge_pa / PSI_PA)
}

pub const PUMP_MECHANICAL_EFFICIENCY: f64 = 0.90;
const HEALTHY_CASE_DRAIN_FRACTION: f64 = 0.03;
const STANDBY_LEAKAGE_M3_S_PER_PA: f64 = 2.5e-5 / 34.47e6;

#[derive(Clone, Copy, Debug, Default)]
pub struct PumpFaults {
    pub wear: f64,
    pub displacement_loss: f64,
    pub seizure: f64,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct PumpOutputs {
    pub flow_m3_s: f64,
    pub case_drain_m3_s: f64,
    pub shaft_power_w: f64,
    pub volumetric_efficiency: f64,
}

fn pump_common(
    displacement_in3: f64,
    rpm: f64,
    outlet_gauge_pa: f64,
    inlet_gauge_pa: f64,
    faults: &PumpFaults,
) -> PumpOutputs {
    let seizure = faults.seizure.clamp(0.0, 1.0);
    let wear = faults.wear.clamp(0.0, 1.0);
    let disp_loss = faults.displacement_loss.clamp(0.0, 1.0);
    let rpm = rpm.max(0.0) * (1.0 - seizure);
    let disp_m3 = (displacement_in3 * (1.0 - disp_loss)).max(0.0) * CUBIC_INCH_M3;
    let ideal_flow_m3_s = disp_m3 * (rpm / 60.0);
    let volumetric_efficiency = (1.0 - wear).clamp(0.0, 1.0);
    let cavitation = cavitation_efficiency(inlet_gauge_pa);
    let flow_m3_s = ideal_flow_m3_s * volumetric_efficiency * cavitation;
    let leaked_to_case = ideal_flow_m3_s * (1.0 - volumetric_efficiency) * cavitation;
    let standby_leak_m3_s = if rpm > 0.0 { STANDBY_LEAKAGE_M3_S_PER_PA * outlet_gauge_pa.max(0.0) } else { 0.0 };
    let case_drain_m3_s = (HEALTHY_CASE_DRAIN_FRACTION * ideal_flow_m3_s * cavitation + leaked_to_case + standby_leak_m3_s) * (1.0 - seizure);
    let shaft_power_w = outlet_gauge_pa.max(0.0) * (flow_m3_s + case_drain_m3_s) / PUMP_MECHANICAL_EFFICIENCY;
    PumpOutputs { flow_m3_s, case_drain_m3_s, shaft_power_w, volumetric_efficiency }
}

#[derive(Clone, Copy, Debug)]
pub struct EngineDrivenPump {
    characteristics: PumpCharacteristics,
}
impl EngineDrivenPump {
    pub fn a380() -> Self {
        Self { characteristics: PumpCharacteristics::a380_edp() }
    }
    pub fn step(&self, shaft_rpm: f64, outlet_gauge_pa: f64, inlet_gauge_pa: f64, faults: &PumpFaults) -> PumpOutputs {
        pump_common(self.characteristics.displacement_in3(outlet_gauge_pa), shaft_rpm, outlet_gauge_pa, inlet_gauge_pa, faults)
    }
}

#[derive(Clone, Copy, Debug)]
pub struct ElectricPump {
    characteristics: PumpCharacteristics,
    regulated_rpm: f64,
    spin_time_constant_s: f64,
    motor_efficiency: f64,
    speed_rpm: f64,
    lagged_displacement_in3: f64,
}
impl ElectricPump {
    const DISPLACEMENT_LAG_S: f64 = 0.15;

    pub fn a380_electric() -> Self {
        Self {
            characteristics: PumpCharacteristics::a380_electric(),
            regulated_rpm: 8000.0,
            spin_time_constant_s: 0.4,
            motor_efficiency: 0.85,
            speed_rpm: 0.0,
            lagged_displacement_in3: 0.0,
        }
    }

    pub fn speed_rpm(&self) -> f64 {
        self.speed_rpm
    }

    pub fn advance_speed(&mut self, powered: bool, dt_s: f64) {
        let target_rpm = if powered { self.regulated_rpm } else { 0.0 };
        let k = 1.0 / self.spin_time_constant_s.max(1e-3);
        let dt = dt_s.max(0.0);
        self.speed_rpm = target_rpm + (self.speed_rpm - target_rpm) * (-k * dt).exp();
    }

    fn blended_displacement_in3(&self, outlet_gauge_pa: f64, dt_s: f64) -> f64 {
        let target = self.characteristics.displacement_in3(outlet_gauge_pa);
        let k = (-dt_s.max(0.0) / Self::DISPLACEMENT_LAG_S).exp();
        self.lagged_displacement_in3 * k + target * (1.0 - k)
    }

    pub fn flow_at(&self, outlet_gauge_pa: f64, inlet_gauge_pa: f64, faults: &PumpFaults, dt_s: f64) -> PumpOutputs {
        pump_common(self.blended_displacement_in3(outlet_gauge_pa, dt_s), self.speed_rpm, outlet_gauge_pa, inlet_gauge_pa, faults)
    }

    pub fn commit_displacement(&mut self, outlet_gauge_pa: f64, dt_s: f64) {
        self.lagged_displacement_in3 = self.blended_displacement_in3(outlet_gauge_pa, dt_s);
    }

    pub fn current_a(&self, out: &PumpOutputs, powered: bool, bus_voltage_v: f64) -> f64 {
        if powered && bus_voltage_v > 0.0 && self.motor_efficiency > 0.0 {
            out.shaft_power_w / (self.motor_efficiency * bus_voltage_v)
        } else {
            0.0
        }
    }

    pub fn step(&mut self, powered: bool, outlet_gauge_pa: f64, inlet_gauge_pa: f64, bus_voltage_v: f64, faults: &PumpFaults, dt_s: f64) -> (PumpOutputs, f64) {
        self.advance_speed(powered, dt_s);
        let out = self.flow_at(outlet_gauge_pa, inlet_gauge_pa, faults, dt_s);
        self.commit_displacement(outlet_gauge_pa, dt_s);
        let current_a = self.current_a(&out, powered, bus_voltage_v);
        (out, current_a)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn edp_destrokes_to_zero_displacement_above_its_compensator_pressure() {
        let c = PumpCharacteristics::a380_edp();
        assert_eq!(c.displacement_in3(0.0), 2.8);
        assert_eq!(c.displacement_in3(2000.0 * PSI_PA), 2.8);
        assert_eq!(c.displacement_in3(5500.0 * PSI_PA), 0.0);
        assert!(c.displacement_in3(5000.0 * PSI_PA) < 2.8);
    }

    #[test]
    fn cavitation_efficiency_ramps_from_the_fbw_breakpoints() {
        assert_eq!(cavitation_efficiency(0.0), 0.0);
        assert_eq!(cavitation_efficiency(30.0 * PSI_PA), 1.0);
        assert!(cavitation_efficiency(10.0 * PSI_PA) > 0.0 && cavitation_efficiency(10.0 * PSI_PA) < 1.0);
    }

    #[test]
    fn a_healthy_edp_delivers_flow_proportional_to_speed() {
        let pump = EngineDrivenPump::a380();
        let low = pump.step(2000.0, 3000.0 * PSI_PA, 50.0 * PSI_PA, &PumpFaults::default());
        let high = pump.step(4000.0, 3000.0 * PSI_PA, 50.0 * PSI_PA, &PumpFaults::default());
        assert!((high.flow_m3_s / low.flow_m3_s - 2.0).abs() < 1e-6);
        assert!(low.flow_m3_s > 0.0);
        assert!(low.shaft_power_w > 0.0);
    }

    #[test]
    fn seizure_stops_flow_and_case_drain_both() {
        let pump = EngineDrivenPump::a380();
        let seized = PumpFaults { seizure: 1.0, ..Default::default() };
        let out = pump.step(4000.0, 3000.0 * PSI_PA, 50.0 * PSI_PA, &seized);
        assert_eq!(out.flow_m3_s, 0.0);
        assert_eq!(out.case_drain_m3_s, 0.0);
    }

    #[test]
    fn wear_raises_case_drain_and_lowers_delivered_flow() {
        let pump = EngineDrivenPump::a380();
        let healthy = pump.step(4000.0, 3000.0 * PSI_PA, 50.0 * PSI_PA, &PumpFaults::default());
        let worn = pump.step(4000.0, 3000.0 * PSI_PA, 50.0 * PSI_PA, &PumpFaults { wear: 0.5, ..Default::default() });
        assert!(worn.flow_m3_s < healthy.flow_m3_s);
        assert!(worn.case_drain_m3_s > healthy.case_drain_m3_s, "case drain should be the health indicator that rises with wear");
    }

    #[test]
    fn displacement_loss_reduces_flow_even_at_low_pressure() {
        let pump = EngineDrivenPump::a380();
        let healthy = pump.step(4000.0, 0.0, 50.0 * PSI_PA, &PumpFaults::default());
        let damaged = pump.step(4000.0, 0.0, 50.0 * PSI_PA, &PumpFaults { displacement_loss: 0.6, ..Default::default() });
        assert!((damaged.flow_m3_s / healthy.flow_m3_s - 0.4).abs() < 1e-6);
    }

    #[test]
    fn low_inlet_pressure_cavitates_the_pump_even_when_otherwise_healthy() {
        let pump = EngineDrivenPump::a380();
        let good_inlet = pump.step(4000.0, 3000.0 * PSI_PA, 50.0 * PSI_PA, &PumpFaults::default());
        let starved_inlet = pump.step(4000.0, 3000.0 * PSI_PA, 0.0, &PumpFaults::default());
        assert_eq!(starved_inlet.flow_m3_s, 0.0);
        assert!(good_inlet.flow_m3_s > 0.0);
    }

    #[test]
    fn electric_pump_spins_up_smoothly_and_draws_current_while_running() {
        let mut pump = ElectricPump::a380_electric();
        let mut last = 0.0;
        for _ in 0..200 {
            let (_, _current) = pump.step(true, 3000.0 * PSI_PA, 50.0 * PSI_PA, 115.0, &PumpFaults::default(), 0.02);
            assert!(pump.speed_rpm() >= last, "speed should ramp up monotonically toward the regulated speed");
            last = pump.speed_rpm();
        }
        assert!((pump.speed_rpm() - 8000.0).abs() < 1.0);
        let (out, current) = pump.step(true, 3000.0 * PSI_PA, 50.0 * PSI_PA, 115.0, &PumpFaults::default(), 0.02);
        assert!(out.flow_m3_s > 0.0);
        assert!(current > 0.0);
    }

    #[test]
    fn electric_pump_spins_down_when_unpowered() {
        let mut pump = ElectricPump::a380_electric();
        let mut running = None;
        for _ in 0..200 {
            running = Some(pump.step(true, 3000.0 * PSI_PA, 50.0 * PSI_PA, 115.0, &PumpFaults::default(), 0.02));
        }
        let (running_out, running_current) = running.unwrap();
        assert!(pump.speed_rpm() > 1000.0);
        assert!(running_out.flow_m3_s > 0.0 && running_current > 0.0);

        for _ in 0..200 {
            pump.step(false, 3000.0 * PSI_PA, 50.0 * PSI_PA, 115.0, &PumpFaults::default(), 0.02);
        }
        assert!(pump.speed_rpm() < 1.0);
        let (out, current) = pump.step(false, 3000.0 * PSI_PA, 50.0 * PSI_PA, 115.0, &PumpFaults::default(), 0.02);

        const SPUN_DOWN_FRACTION: f64 = 1.0e-4;
        assert!(out.flow_m3_s >= 0.0 && out.flow_m3_s < running_out.flow_m3_s * SPUN_DOWN_FRACTION, "flow {} must be a negligible fraction of the running {}", out.flow_m3_s, running_out.flow_m3_s);
        assert!(current >= 0.0 && current < running_current * SPUN_DOWN_FRACTION, "current {} must be a negligible fraction of the running {}", current, running_current);
    }

    #[test]
    fn no_nan_at_rest_or_zero_voltage() {
        let mut pump = ElectricPump::a380_electric();
        let (out, current) = pump.step(true, 0.0, 0.0, 0.0, &PumpFaults::default(), 0.0);
        assert!(out.flow_m3_s.is_finite());
        assert_eq!(current, 0.0);
    }
}
