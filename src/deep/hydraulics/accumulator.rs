//! Hydraulic accumulator: a nitrogen-precharged piston/bladder accumulator
//! that stores fluid pushed in against the gas cushion's own compression,
//! smoothing pump ripple and covering transient demand peaks (gear
//! extension, brake application, reverser deployment) the pumps alone
//! cannot instantly match.
//!
//! State is the gas volume (fluid volume = shell volume - gas volume); the
//! nitrogen follows a polytropic process `P * V^n = const` (Merritt,
//! "Hydraulic Control Systems", ch. 2; `n` = 1.0 isothermal .. 1.4 adiabatic,
//! 1.2 used as a mid-range figure for the fast charge/discharge transients
//! an accumulator actually sees -- the same exponent `fluid.rs`'s
//! entrained-air correction uses, for the same reason). Precharge and
//! internal volume match FlyByWire's own A380 model
//! (`a380_systems/src/hydraulic/mod.rs`,
//! `A380HydraulicCircuitFactory::ACCUMULATOR_GAS_PRE_CHARGE_PSI` = 2612 psi,
//! `..._MAX_VOLUME_GALLONS` = 0.5 gal -- the real A380's own figures are not
//! public, so this reuses FBW's modelled values for the same aircraft rather
//! than inventing new ones).
//!
//! Coupled to the network across one tick's (mostly settled) line pressure
//! rather than folded into `network::Network`'s implicit node solve, and
//! stepped by fine sub-stepping instead -- the brief's explicitly allowed
//! alternative to a fully implicit solve. This is safe here because the
//! accumulator's own state (gas volume) is clamped to `(0, shell_volume]` by
//! construction every sub-step, so sub-stepping cannot run away regardless
//! of how large a single outer `dt` is asked for.

use super::fluid;

pub const GALLON_M3: f64 = 3.785_411_784e-3;
pub const PSI_PA: f64 = 6894.757;

/// Faults an accumulator can carry.
#[derive(Clone, Copy, Debug, Default)]
pub struct AccumulatorFaults {
    /// Nitrogen bled out through the gas valve/worn bladder over time: 0 the
    /// full rated precharge, 1 none left (the gas side has relaxed to
    /// ambient, so the accumulator can absorb fluid but delivers almost no
    /// stored pressure back out).
    pub precharge_loss: f64,
}

#[derive(Clone, Debug)]
pub struct Accumulator {
    shell_volume_m3: f64,
    rated_precharge_pa: f64,
    polytropic_n: f64,
    /// The accumulator's own internal port (a small orifice to the line it
    /// charges from/discharges to), m^2. Sized so the accumulator responds
    /// within a couple of seconds to a real demand transient rather than
    /// instantly (a real accumulator's inlet is a restricted union, not a
    /// wide-open port) -- GENERIC, no public A380 port size exists.
    port_area_m2: f64,
    /// State: current gas volume, m^3, in `(0, shell_volume_m3]`. Fluid
    /// volume held is `shell_volume_m3 - gas_volume_m3`.
    gas_volume_m3: f64,
}

impl Accumulator {
    pub fn new(shell_volume_m3: f64, precharge_pa: f64, polytropic_n: f64, port_area_m2: f64) -> Self {
        let shell_volume_m3 = shell_volume_m3.max(1e-9);
        Self { shell_volume_m3, rated_precharge_pa: precharge_pa.max(1.0), polytropic_n: polytropic_n.max(1.0), port_area_m2, gas_volume_m3: shell_volume_m3 }
    }

    /// FlyByWire's own A380 green/yellow accumulator sizing (see module doc).
    pub fn a380() -> Self {
        Self::new(GALLON_M3 * 0.5, 2612.0 * PSI_PA, 1.2, 3.0e-5)
    }

    fn precharge_pa(&self, faults: &AccumulatorFaults) -> f64 {
        let loss = faults.precharge_loss.clamp(0.0, 1.0);
        self.rated_precharge_pa * (1.0 - loss) + fluid::ATM_PA * loss
    }

    /// The gas cushion's own pressure at the current state (absolute Pa),
    /// from the polytropic relation `P0*V0^n = P*Vgas^n` with `V0` the shell
    /// volume (the reference state: no fluid in, gas fills the whole shell
    /// at the precharge pressure).
    pub fn pressure_pa(&self, faults: &AccumulatorFaults) -> f64 {
        let p0 = self.precharge_pa(faults);
        p0 * (self.shell_volume_m3 / self.gas_volume_m3.max(1e-9)).powf(self.polytropic_n)
    }

    pub fn fluid_volume_m3(&self) -> f64 {
        self.shell_volume_m3 - self.gas_volume_m3
    }

    /// Steps the accumulator against a representative line gauge pressure
    /// held constant across this tick, returning the average flow it drew
    /// from the line, m^3/s (positive = charging: fluid flowed line ->
    /// accumulator; the caller subtracts this from that line's node
    /// injection for this same tick, since it is fluid the network no
    /// longer has). `line_pressure_pa` is gauge; the accumulator's own
    /// pressure is absolute, so gauge is converted once here.
    pub fn step(&mut self, line_pressure_pa_gauge: f64, faults: &AccumulatorFaults, dt_s: f64) -> f64 {
        let dt = dt_s.max(0.0);
        if dt <= 0.0 {
            return 0.0;
        }
        const SUBSTEPS: usize = 20;
        let sub_dt = dt / SUBSTEPS as f64;
        let line_pa_abs = line_pressure_pa_gauge + fluid::ATM_PA;
        let density = fluid::density_kg_m3(60.0); // representative fluid temperature for the port's own small loss
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
        // Charge it up first.
        for _ in 0..500 {
            acc.step(5000.0 * PSI_PA, &faults_ok(), 0.02);
        }
        let charged_fluid = acc.fluid_volume_m3();
        assert!(charged_fluid > 0.0);
        // Now the line sags well below the accumulator's own pressure: it must give fluid back.
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
            let flow = acc.step(-100_000.0, &faults_ok(), 0.02); // way below its own pressure, tries to dump everything
            assert!(flow.is_finite());
        }
        assert!(acc.fluid_volume_m3() >= 0.0);
        assert!(acc.pressure_pa(&faults_ok()).is_finite());
    }

    #[test]
    fn precharge_loss_lowers_both_resting_and_charged_pressure() {
        let healthy = Accumulator::a380();
        let mut leaked = Accumulator::a380();
        let leak_fault = AccumulatorFaults { precharge_loss: 1.0 };
        assert!(leaked.pressure_pa(&leak_fault) < healthy.pressure_pa(&faults_ok()));
        for _ in 0..500 {
            leaked.step(5000.0 * PSI_PA, &leak_fault, 0.02);
        }
        // With no precharge, the same charging flow buys far less pressure rise for the same fluid in.
        assert!(leaked.pressure_pa(&leak_fault) < 5000.0 * PSI_PA);
    }

    #[test]
    fn polytropic_relation_holds_algebraically() {
        let mut acc = Accumulator::new(1.0e-3, 2000.0 * PSI_PA, 1.2, 1e-4);
        acc.gas_volume_m3 = 0.5e-3; // halve the gas volume directly
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
}
