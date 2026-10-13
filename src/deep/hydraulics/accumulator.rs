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

    /// Instantaneous flow the accumulator's own port would exchange with a
    /// line held at `line_pressure_pa_gauge`, evaluated at the accumulator's
    /// CURRENT state (m^3/s, positive = charging: line -> accumulator, the
    /// same sign convention `step`'s return already uses). The same
    /// port-orifice formula `step`'s own per-sub-step body uses, but a
    /// single pure evaluation: it does NOT touch `gas_volume_m3`, so unlike
    /// `step` it is safe to call many times in the same tick with different
    /// trial pressures -- which is exactly what a `network::Network`
    /// bisection residual does (see `topology.rs`'s `manifold_injection`
    /// closure, which folds this in alongside the EDPs/relief valve the
    /// same way `network::Network::step_with_pressure_dependent`'s own doc
    /// describes, fixes/W72.md's mechanism). This exists because freezing
    /// THIS accumulator's exchange flow for a whole tick, the way `step`
    /// alone used to be used for, raced MANIFOLD's own tiny capacitance
    /// exactly like the EDP/relief bug fixes/W72.md fixed -- producing the
    /// separate 2257/2892 psi two-level chatter fixes/W174.md's SECONDARY
    /// CHATTER section documents (Log-keep-123124.txt, t=1.3-9.6s):
    /// `HYD_GREEN_ACCUMULATOR_PRESSURE_PSI` alternating between its
    /// bled-dry precharge floor (2612.0 psi) and ~2890.5 psi, in lock-step
    /// with MANIFOLD/ESSENTIAL.
    /// (build fix, INT-P4) Takes `dt_s` and bounds the raw port-orifice
    /// value by what this accumulator's own fixed shell can physically
    /// exchange in that one tick -- the same bound `step`'s own per-
    /// substep `(gas_volume_m3 - dv).clamp(1e-9, shell_volume_m3)` already
    /// enforced on mainline. Without it, a bisection residual (this
    /// function's own reason for existing) can search trial pressures far
    /// from `self`'s actual state -- e.g. a cold, empty manifold (0 Pa)
    /// against a freshly-constructed accumulator sitting at its full
    /// precharge (`Accumulator::a380`'s ~2612 psi, all gas, zero fluid) --
    /// where the unbounded formula reports a large "discharge" the
    /// accumulator has no fluid left to give (`fluid_volume_m3() == 0`
    /// there), and the solver treats that phantom capacity as a real
    /// pressure source, converging MANIFOLD to a value nothing physically
    /// backs (confirmed by hand: `the_green_circuit_can_be_pressurised_
    /// on_its_electric_pumps_with_no_engines` jumped to ~1843 psi in the
    /// first 0.02 s tick with the electric pumps still at ~0 flow, purely
    /// from this). `advance` already self-corrects its OWN state this way;
    /// this brings the value the SOLVE sees into line with what `advance`
    /// will actually be able to commit afterward.
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
            // Charging (line -> accumulator): bounded by how much more gas
            // volume can still compress before `advance`'s own `1e-9` floor.
            let max_in = (self.gas_volume_m3 - 1e-9).max(0.0) / dt_s;
            raw.min(max_in)
        } else {
            // Discharging (accumulator -> line): bounded by the fluid
            // actually on hand (the ullage left before the shell ceiling).
            let max_out = (self.shell_volume_m3 - self.gas_volume_m3).max(0.0) / dt_s;
            raw.max(-max_out)
        }
    }

    /// Commits this tick's state change from a flow ALREADY DECIDED
    /// elsewhere (`committed_flow_m3_s`, the same sign convention
    /// `exchange_flow_at`/`step` both use: positive = charging), sub-stepped
    /// the same `SUBSTEPS` = 20 way `step` always sub-stepped its own
    /// internally re-derived flow. Away from the shell-volume clamp this is
    /// numerically identical to one un-sub-stepped update (20 equal slivers
    /// of the same constant rate integrate to the same total as one); the
    /// sub-stepping still earns its keep right at the clamp, where it lets
    /// `gas_volume_m3` ease up against `(1e-9, shell_volume_m3)` a
    /// twentieth of a tick at a time instead of jumping straight past it in
    /// one whole-`dt` step. The caller (`topology.rs`) is expected to pass
    /// `exchange_flow_at` evaluated at the NETWORK's now-converged pressure
    /// -- the same read-after-solve pattern `fixes/W72.md`'s EDIT5 already
    /// established for the EDPs' own telemetry read-back -- so the state
    /// this commits matches what the pressure solve actually resolved,
    /// rather than a pre-solve, lagged estimate.
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
        let ok = faults_ok();
        let leak_fault = AccumulatorFaults { precharge_loss: 1.0 };
        let mut healthy = Accumulator::a380();
        let mut leaked = Accumulator::a380();

        // -- Resting (no fluid in, gas filling the whole shell): a fully
        // bled accumulator's gas cushion has relaxed to atmospheric, so it
        // holds no stored pressure at all, against the healthy one's 2612
        // psi precharge.
        assert!((leaked.pressure_pa(&leak_fault) - fluid::ATM_PA).abs() < 1.0);
        assert!(leaked.pressure_pa(&leak_fault) < healthy.pressure_pa(&ok));

        // -- Charged from the same 5000 psi *gauge* line. Both end at the
        // line's own pressure (that is what charged means: port flow stops
        // when dp = 0), so what differs is how much fluid each had to
        // swallow to get there. Note the accumulator's own pressure is
        // absolute, so the charged value is 5000 psi + 1 atm -- the old
        // `< 5000 psi` assertion here was comparing an absolute pressure
        // against a gauge one. Hand solve from `P0*V0^n = P*Vgas^n`,
        // n = 1.2, line_abs = 5000*6894.757 + 101325 = 34.575 MPa:
        //   healthy (P0 = 2612 psi = 18.009 MPa):
        //     (V0/Vgas)^1.2 = 34.575/18.009 = 1.9199 -> V0/Vgas = 1.7220
        //     -> fluid held = 1 - 1/1.7220 = 0.419 of the shell
        //   leaked  (P0 = 1 atm = 0.10133 MPa):
        //     (V0/Vgas)^1.2 = 34.575/0.10133 = 341.2 -> V0/Vgas = 129.1
        //     -> fluid held = 1 - 1/129.1 = 0.992 of the shell
        // i.e. the bled unit ends up almost solid fluid at line pressure.
        for _ in 0..500 {
            healthy.step(5000.0 * PSI_PA, &ok, 0.02);
            leaked.step(5000.0 * PSI_PA, &leak_fault, 0.02);
        }
        let shell = GALLON_M3 * 0.5;
        assert!((healthy.fluid_volume_m3() / shell - 0.419).abs() < 0.01, "healthy charged fill fraction {}", healthy.fluid_volume_m3() / shell);
        assert!((leaked.fluid_volume_m3() / shell - 0.992).abs() < 0.01, "leaked charged fill fraction {}", leaked.fluid_volume_m3() / shell);

        // -- Delivering into a dead line (0 psi gauge): the gas can only
        // expand until it fills the shell again, so the healthy unit pushes
        // its last drop of fluid out at its full 2612 psi precharge while
        // the bled one runs out of push at atmospheric. That stored
        // delivery pressure is the whole function of a precharge, and it is
        // what `precharge_loss` takes away.
        // (2000 steps = 40 s: the gas spring's push falls off as
        // sqrt(dp) near the end, so the last few millilitres take ~12 s.)
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

    #[test]
    fn exchange_flow_at_is_pure_and_matches_the_port_formula() {
        let acc = Accumulator::a380();
        let faults = faults_ok();
        let line_pa = 5000.0 * PSI_PA;
        // A typical tick (0.02 s): the raw port flow at this dp is far
        // below what a whole tick's worth of this accumulator's own 0.5
        // gallon shell could absorb, so the (build fix, INT-P4) physical
        // capacity bound added to `exchange_flow_at` does not engage here
        // and the expected value below is unchanged.
        let flow = acc.exchange_flow_at(line_pa, &faults, 0.02);

        // Hand-computed from the same formula `step`'s own per-sub-step
        // body uses, at the resting state `Accumulator::a380` starts in
        // (full gas, no fluid, at its 2612 psi precharge): port area is
        // `Accumulator::a380`'s own 3.0e-5 m^2 constant (module doc).
        let density = fluid::density_kg_m3(60.0);
        let p_acc = acc.pressure_pa(&faults);
        let dp = (line_pa + fluid::ATM_PA) - p_acc;
        let expected = 0.61 * 3.0e-5 * (2.0 * dp / density).sqrt();
        assert!((flow - expected).abs() / expected < 1e-9, "flow {flow} expected {expected}");

        // Pure: must not have touched the accumulator's own state, unlike
        // `step`/`advance`.
        assert_eq!(acc.fluid_volume_m3(), 0.0, "exchange_flow_at must not mutate state");
    }

    /// (build fix, INT-P4) A freshly constructed accumulator has zero
    /// fluid on hand (all gas, at its own precharge pressure) -- it cannot
    /// discharge anything into a line sitting below that precharge, no
    /// matter how large the raw pressure-difference formula reports,
    /// because there is no fluid behind the port to push. Without the
    /// physical-capacity bound this is a phantom flow a bisection residual
    /// (`topology.rs`'s `manifold_injection`) would treat as a real
    /// pressure source, pulling a cold, empty manifold up toward the
    /// accumulator's own precharge in a single tick.
    #[test]
    fn a_fresh_accumulator_with_no_fluid_cannot_phantom_discharge_into_an_empty_line() {
        let acc = Accumulator::a380();
        let faults = faults_ok();
        assert_eq!(acc.fluid_volume_m3(), 0.0, "setup: fresh accumulator starts with no fluid");
        // An empty manifold: 0 Pa gauge, far below the ~2612 psi precharge,
        // so the raw (unbounded) formula would report a large discharge.
        let flow = acc.exchange_flow_at(0.0, &faults, 0.02);
        assert_eq!(flow, 0.0, "no fluid on hand means no flow, whatever the raw formula would otherwise say: {flow}");
    }

    /// The mirror case: an accumulator with only a sliver of gas ullage
    /// left to compress cannot take in more than that sliver in one tick,
    /// even against a line pressure high enough that the raw formula alone
    /// would ask for far more.
    #[test]
    fn an_almost_full_accumulator_is_capped_to_its_remaining_ullage() {
        let mut acc = Accumulator::a380();
        let faults = faults_ok();
        // A tiny sliver of gas ullage left (the shell is ~1.893e-3 m^3;
        // 1e-7 is a fraction of a percent of that) -- not the numerical
        // floor itself, so the polytropic relation still gives a finite
        // (if very high) accumulator pressure, letting a plausible high
        // line pressure still read as a charging attempt (dp > 0).
        acc.gas_volume_m3 = 1e-7;
        let p_acc = acc.pressure_pa(&faults);
        let line_pa = p_acc + 10.0 * PSI_PA - fluid::ATM_PA; // just above the accumulator's own pressure
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

        // A huge committed charging flow for a whole second: far more than
        // the shell can hold. `advance` must clamp, not overshoot into a
        // negative gas volume (which `fluid_volume_m3` would report as
        // exceeding the shell).
        let avg = acc.advance(10.0, 1.0);
        assert!(acc.fluid_volume_m3() <= shell + 1e-12, "should not exceed the shell volume, got {}", acc.fluid_volume_m3());
        assert!(avg > 0.0, "reported average flow should still be positive (charging)");

        // Symmetric check discharging from a charged state: a huge negative
        // committed flow must not drive fluid volume negative either.
        let before = acc.fluid_volume_m3();
        assert!(before > 0.0, "should have taken in fluid from the charge above");
        acc.advance(-10.0, 1.0);
        assert!(acc.fluid_volume_m3() >= 0.0, "should not go negative, got {}", acc.fluid_volume_m3());
        assert!(acc.fluid_volume_m3() <= before, "should have given fluid back, not gained more");
    }
}
