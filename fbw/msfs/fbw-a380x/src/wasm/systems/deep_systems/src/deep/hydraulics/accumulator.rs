use super::fluid;

pub const GALLON_M3: f64 = 3.785_411_784e-3;
pub const PSI_PA: f64 = 6894.757;

#[derive(Clone, Copy, Debug, Default)]
pub struct AccumulatorFaults {
    pub precharge_loss: f64,
}

#[derive(Clone, Debug)]
pub struct Accumulator {
    shell_volume_m3: f64,
    rated_precharge_pa: f64,
    polytropic_n: f64,
    port_area_m2: f64,
    gas_volume_m3: f64,
}

impl Accumulator {
    pub fn new(shell_volume_m3: f64, precharge_pa: f64, polytropic_n: f64, port_area_m2: f64) -> Self {
        let shell_volume_m3 = shell_volume_m3.max(1e-9);
        Self { shell_volume_m3, rated_precharge_pa: precharge_pa.max(1.0), polytropic_n: polytropic_n.max(1.0), port_area_m2, gas_volume_m3: shell_volume_m3 }
    }

    pub fn a380() -> Self {
        Self::new(GALLON_M3 * 0.5, 2612.0 * PSI_PA, 1.2, 3.0e-5)
    }

    fn precharge_pa(&self, faults: &AccumulatorFaults) -> f64 {
        let loss = faults.precharge_loss.clamp(0.0, 1.0);
        self.rated_precharge_pa * (1.0 - loss) + fluid::ATM_PA * loss
    }

    pub fn pressure_pa(&self, faults: &AccumulatorFaults) -> f64 {
        let p0 = self.precharge_pa(faults);
        p0 * (self.shell_volume_m3 / self.gas_volume_m3.max(1e-9)).powf(self.polytropic_n)
    }

    pub fn fluid_volume_m3(&self) -> f64 {
        self.shell_volume_m3 - self.gas_volume_m3
    }

    pub fn step(&mut self, line_pressure_pa_gauge: f64, faults: &AccumulatorFaults, dt_s: f64) -> f64 {
        let dt = dt_s.max(0.0);
        if dt <= 0.0 {
            return 0.0;
        }
        const SUBSTEPS: usize = 20;
        let sub_dt = dt / SUBSTEPS as f64;
        let line_pa_abs = line_pressure_pa_gauge + fluid::ATM_PA;
        let density = fluid::density_kg_m3(60.0);
        let mut net_in_m3 = 0.0;
        for _ in 0..SUBSTEPS {
            let p_acc = self.pressure_pa(faults);
            let dp = line_pa_abs - p_acc;
            let sign = if dp >= 0.0 { 1.0 } else { -1.0 };
            let q = sign * 0.61 * self.port_area_m2.max(0.0) * (2.0 * dp.abs() / density).sqrt();
            let dv = q * sub_dt;
            let new_gas = (self.gas_volume_m3 - dv).clamp(1e-9, self.shell_volume_m3);
            net_in_m3 += self.gas_volume_m3 - new_gas;
            self.gas_volume_m3 = new_gas;
        }
        net_in_m3 / dt
    }

    pub fn exchange_flow_at(&self, line_pressure_pa_gauge: f64, faults: &AccumulatorFaults, dt_s: f64) -> f64 {
        let line_pa_abs = line_pressure_pa_gauge + fluid::ATM_PA;
        let density = fluid::density_kg_m3(60.0);
        let p_acc = self.pressure_pa(faults);
        let dp = line_pa_abs - p_acc;
        let sign = if dp >= 0.0 { 1.0 } else { -1.0 };
        let raw = sign * 0.61 * self.port_area_m2.max(0.0) * (2.0 * dp.abs() / density).sqrt();
        if dt_s <= 0.0 {
            return raw;
        }
        if raw > 0.0 {
            let max_in = (self.gas_volume_m3 - 1e-9).max(0.0) / dt_s;
            raw.min(max_in)
        } else {
            let max_out = (self.shell_volume_m3 - self.gas_volume_m3).max(0.0) / dt_s;
            raw.max(-max_out)
        }
    }

    pub fn advance(&mut self, committed_flow_m3_s: f64, dt_s: f64) -> f64 {
        let dt = dt_s.max(0.0);
        if dt <= 0.0 {
            return 0.0;
        }
        const SUBSTEPS: usize = 20;
        let sub_dt = dt / SUBSTEPS as f64;
        let dv_per_substep = committed_flow_m3_s * sub_dt;
        let mut net_in_m3 = 0.0;
        for _ in 0..SUBSTEPS {
            let new_gas = (self.gas_volume_m3 - dv_per_substep).clamp(1e-9, self.shell_volume_m3);
            net_in_m3 += self.gas_volume_m3 - new_gas;
            self.gas_volume_m3 = new_gas;
        }
        net_in_m3 / dt
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn faults_ok() -> AccumulatorFaults {
        AccumulatorFaults::default()
    }

    #[test]
    fn resting_pressure_equals_precharge_when_empty_of_fluid() {
        let acc = Accumulator::a380();
        assert!((acc.pressure_pa(&faults_ok()) - 2612.0 * PSI_PA).abs() < 1.0);
        assert_eq!(acc.fluid_volume_m3(), 0.0);
    }

    #[test]
    fn charges_when_line_pressure_exceeds_its_own() {
        let mut acc = Accumulator::a380();
        let line_pa = 5000.0 * PSI_PA;
        let mut flow_sum = 0.0;
        for _ in 0..500 {
            flow_sum += acc.step(line_pa, &faults_ok(), 0.02);
        }
        assert!(acc.fluid_volume_m3() > 0.0, "should have taken in fluid");
        assert!(flow_sum > 0.0, "net flow into the accumulator should be positive while charging");
        assert!(acc.pressure_pa(&faults_ok()) > 2612.0 * PSI_PA);
    }

    #[test]
    fn discharges_to_support_a_pressure_drop_and_never_exceeds_its_shell() {
        let mut acc = Accumulator::a380();
        for _ in 0..500 {
            acc.step(5000.0 * PSI_PA, &faults_ok(), 0.02);
        }
        let charged_fluid = acc.fluid_volume_m3();
        assert!(charged_fluid > 0.0);
        let mut gave_back = 0.0;
        for _ in 0..50 {
            gave_back += acc.step(0.0, &faults_ok(), 0.02) * 0.02;
        }
        assert!(gave_back < 0.0, "discharging should show as net negative flow (out of the accumulator)");
        assert!(acc.fluid_volume_m3() < charged_fluid);
        assert!(acc.fluid_volume_m3() >= 0.0);
    }

    #[test]
    fn a_fully_discharged_accumulator_clamps_without_going_negative_or_nan() {
        let mut acc = Accumulator::a380();
        for _ in 0..2000 {
            let flow = acc.step(-100_000.0, &faults_ok(), 0.02);
            assert!(flow.is_finite());
        }
        assert!(acc.fluid_volume_m3() >= 0.0);
        assert!(acc.pressure_pa(&faults_ok()).is_finite());
    }

    #[test]
    fn precharge_loss_lowers_both_resting_and_charged_pressure() {
        let ok = faults_ok();
        let leak_fault = AccumulatorFaults { precharge_loss: 1.0 };
        let mut healthy = Accumulator::a380();
        let mut leaked = Accumulator::a380();

        assert!((leaked.pressure_pa(&leak_fault) - fluid::ATM_PA).abs() < 1.0);
        assert!(leaked.pressure_pa(&leak_fault) < healthy.pressure_pa(&ok));

        for _ in 0..500 {
            healthy.step(5000.0 * PSI_PA, &ok, 0.02);
            leaked.step(5000.0 * PSI_PA, &leak_fault, 0.02);
        }
        let shell = GALLON_M3 * 0.5;
        assert!((healthy.fluid_volume_m3() / shell - 0.419).abs() < 0.01, "healthy charged fill fraction {}", healthy.fluid_volume_m3() / shell);
        assert!((leaked.fluid_volume_m3() / shell - 0.992).abs() < 0.01, "leaked charged fill fraction {}", leaked.fluid_volume_m3() / shell);

        for _ in 0..2000 {
            healthy.step(0.0, &ok, 0.02);
            leaked.step(0.0, &leak_fault, 0.02);
        }
        assert!((healthy.pressure_pa(&ok) - 2612.0 * PSI_PA).abs() / (2612.0 * PSI_PA) < 1e-9, "healthy relaxes back to exactly its precharge, got {}", healthy.pressure_pa(&ok));
        assert!((leaked.pressure_pa(&leak_fault) - fluid::ATM_PA).abs() / fluid::ATM_PA < 1e-9, "a fully bled accumulator relaxes to ambient, got {}", leaked.pressure_pa(&leak_fault));
        assert!(leaked.pressure_pa(&leak_fault) < healthy.pressure_pa(&ok));
    }

    #[test]
    fn polytropic_relation_holds_algebraically() {
        let mut acc = Accumulator::new(1.0e-3, 2000.0 * PSI_PA, 1.2, 1e-4);
        acc.gas_volume_m3 = 0.5e-3;
        let expected = 2000.0 * PSI_PA * 2.0f64.powf(1.2);
        assert!((acc.pressure_pa(&faults_ok()) - expected).abs() / expected < 1e-9);
    }

    #[test]
    fn no_nan_at_dt_zero() {
        let mut acc = Accumulator::a380();
        let flow = acc.step(5000.0 * PSI_PA, &faults_ok(), 0.0);
        assert_eq!(flow, 0.0);
        assert!(acc.pressure_pa(&faults_ok()).is_finite());
    }

    #[test]
    fn exchange_flow_at_is_pure_and_matches_the_port_formula() {
        let acc = Accumulator::a380();
        let faults = faults_ok();
        let line_pa = 5000.0 * PSI_PA;
        let flow = acc.exchange_flow_at(line_pa, &faults, 0.02);

        let density = fluid::density_kg_m3(60.0);
        let p_acc = acc.pressure_pa(&faults);
        let dp = (line_pa + fluid::ATM_PA) - p_acc;
        let expected = 0.61 * 3.0e-5 * (2.0 * dp / density).sqrt();
        assert!((flow - expected).abs() / expected < 1e-9, "flow {flow} expected {expected}");

        assert_eq!(acc.fluid_volume_m3(), 0.0, "exchange_flow_at must not mutate state");
    }

    #[test]
    fn a_fresh_accumulator_with_no_fluid_cannot_phantom_discharge_into_an_empty_line() {
        let acc = Accumulator::a380();
        let faults = faults_ok();
        assert_eq!(acc.fluid_volume_m3(), 0.0, "setup: fresh accumulator starts with no fluid");
        let flow = acc.exchange_flow_at(0.0, &faults, 0.02);
        assert_eq!(flow, 0.0, "no fluid on hand means no flow, whatever the raw formula would otherwise say: {flow}");
    }

    #[test]
    fn an_almost_full_accumulator_is_capped_to_its_remaining_ullage() {
        let mut acc = Accumulator::a380();
        let faults = faults_ok();
        acc.gas_volume_m3 = 1e-7;
        let p_acc = acc.pressure_pa(&faults);
        let line_pa = p_acc + 10.0 * PSI_PA - fluid::ATM_PA;
        let dt = 0.02;
        let max_in = (acc.gas_volume_m3 - 1e-9) / dt;
        let flow = acc.exchange_flow_at(line_pa, &faults, dt);
        assert!(flow <= max_in + 1e-15, "must not exceed the remaining ullage's own rate, got {flow} > {max_in}");
        assert!(flow >= 0.0, "still a charging flow, just capped: {flow}");
    }

    #[test]
    fn advance_applies_a_committed_flow_and_clamps_at_the_shell_both_ways() {
        let mut acc = Accumulator::a380();
        let shell = GALLON_M3 * 0.5;

        let avg = acc.advance(10.0, 1.0);
        assert!(acc.fluid_volume_m3() <= shell + 1e-12, "should not exceed the shell volume, got {}", acc.fluid_volume_m3());
        assert!(avg > 0.0, "reported average flow should still be positive (charging)");

        let before = acc.fluid_volume_m3();
        assert!(before > 0.0, "should have taken in fluid from the charge above");
        acc.advance(-10.0, 1.0);
        assert!(acc.fluid_volume_m3() >= 0.0, "should not go negative, got {}", acc.fluid_volume_m3());
        assert!(acc.fluid_volume_m3() <= before, "should have given fluid back, not gained more");
    }
}
