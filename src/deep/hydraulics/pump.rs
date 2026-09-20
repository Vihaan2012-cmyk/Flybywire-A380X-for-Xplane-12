//! Hydraulic pumps: engine-driven (pressure-compensated variable
//! displacement) and electric motor-driven, sharing the same displacement
//! and cavitation model.
//!
//! A pressure-compensated piston pump's swash plate destrokes as outlet
//! pressure nears the system's regulated pressure, so it delivers whatever
//! flow the circuit demands up to its rated displacement and then holds
//! pressure rather than overshooting it -- this is why the network's own
//! implicit solve does not itself need to regulate to a setpoint: the pumps
//! do that physically, exactly as the real hardware does. Displacement
//! tables are FlyByWire's own published A380 pump characteristics
//! (`fbw-common/src/wasm/systems/systems/src/hydraulic/pumps.rs`,
//! `PumpCharacteristics::a380_edp`/`a380_electric_pump`), the closest public
//! proxy to the real A380 EDP/electric pump curves (no Airbus AMM figures
//! are public), reproduced here as plain breakpoint tables since this
//! directory's code stays self-contained rather than depending on `uom` or
//! FBW's crate.

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

/// A pump's displacement-vs-outlet-pressure curve (the pressure compensator)
/// and its low-inlet-pressure cavitation derate.
#[derive(Clone, Copy, Debug)]
pub struct PumpCharacteristics {
    pressure_breakpoints_psi: [f64; 9],
    displacement_in3: [f64; 9],
}
impl PumpCharacteristics {
    /// `PumpCharacteristics::A380_EDP_DISPLACEMENT_BREAKPTS_PSI`/`_MAP_CUBIC_INCH`
    /// (`pumps.rs` lines 51-55): max 2.8 in^3 up to 2900 psi, destroking to
    /// zero by 5150 psi (the real A380 EDP's own destroke curve is not
    /// public; this is FBW's own modelled curve for the same pump).
    pub fn a380_edp() -> Self {
        Self {
            pressure_breakpoints_psi: [0.0, 500.0, 1000.0, 2900.0, 4790.0, 5150.0, 5225.0, 5350.0, 5500.0],
            displacement_in3: [2.8, 2.8, 2.8, 2.8, 2.6, 0.0, 0.0, 0.0, 0.0],
        }
    }
    /// `PumpCharacteristics::A380_EPUMP_DISPLACEMENT_BREAKPTS_PSI`/`_MAP_CUBIC_INCH`
    /// (`pumps.rs` lines 59-62): the yellow electric pump.
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

/// `PumpCharacteristics::AIR_PRESSURE_BREAKPTS_PSI`/`AIR_PRESSURE_CARAC_RATIO`
/// (`pumps.rs` lines 28-29): a pump's inlet needs a positive boost pressure
/// (from the reservoir's own bootstrap pressurisation, `reservoir.rs`) or it
/// cavitates -- efficiency ramps from 0 at 0 psi inlet gauge to 1.0 by
/// 30 psi.
const AIR_PRESSURE_BREAKPTS_PSI: [f64; 9] = [0.0, 5.0, 10.0, 15.0, 20.0, 30.0, 50.0, 70.0, 100.0];
const CAVITATION_MAP_RATIO: [f64; 9] = [0.0, 0.1, 0.6, 0.8, 0.9, 1.0, 1.0, 1.0, 1.0];
pub fn cavitation_efficiency(inlet_gauge_pa: f64) -> f64 {
    interpolate9(&AIR_PRESSURE_BREAKPTS_PSI, &CAVITATION_MAP_RATIO, inlet_gauge_pa / PSI_PA)
}

/// A positive-displacement pump's own mechanical/volumetric loss put back
/// onto its drive as extra shaft power for the flow it delivers -- this
/// crate's own `physics::hydraulics.rs` module doc already documents this
/// exact figure and rationale (aviation axial-piston-pump efficiency, used
/// there for `EngineDrivenPump::shaft_power()`); reused here rather than a
/// second, possibly-divergent number.
pub const PUMP_MECHANICAL_EFFICIENCY: f64 = 0.90;
/// Healthy piston pump internal (case drain) leakage as a fraction of ideal
/// flow -- GENERIC, a representative small-clearance figure; wear adds to
/// this (see `PumpFaults::wear` below), which is what makes case drain flow
/// the real-world health indicator the brief asks for.
const HEALTHY_CASE_DRAIN_FRACTION: f64 = 0.03;

/// Faults a pump can carry. `wear` is a slowly accumulated health parameter
/// (persisted on the component, not itself an instructor-triggered
/// failure); `displacement_loss` and `seizure` are the discrete failures.
#[derive(Clone, Copy, Debug, Default)]
pub struct PumpFaults {
    /// Internal clearance wear: 0 healthy .. 1 fully worn (no useful
    /// delivery, all displacement leaks past internally as case drain).
    pub wear: f64,
    /// Swash-plate/valve-plate damage: 0 healthy .. 1 no displacement at any
    /// pressure (independent of the pressure-compensator curve itself).
    pub displacement_loss: f64,
    /// Mechanical seizure: 0 healthy .. 1 fully seized (shaft will not turn;
    /// zero flow *and* zero case drain, since nothing is pumping at all).
    pub seizure: f64,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct PumpOutputs {
    pub flow_m3_s: f64,
    /// Case drain flow, m^3/s -- rises with `wear`, the real diagnostic a
    /// maintenance crew reads to catch a degrading pump before it fails
    /// outright.
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
    let case_drain_m3_s = (HEALTHY_CASE_DRAIN_FRACTION * ideal_flow_m3_s * cavitation + leaked_to_case) * (1.0 - seizure);
    let shaft_power_w = outlet_gauge_pa.max(0.0) * (flow_m3_s + case_drain_m3_s) / PUMP_MECHANICAL_EFFICIENCY;
    PumpOutputs { flow_m3_s, case_drain_m3_s, shaft_power_w, volumetric_efficiency }
}

/// An engine-driven pump: shaft speed is whatever the accessory gearbox
/// (another area's model) delivers this tick.
#[derive(Clone, Copy, Debug)]
pub struct EngineDrivenPump {
    characteristics: PumpCharacteristics,
}
impl EngineDrivenPump {
    pub fn a380() -> Self {
        Self { characteristics: PumpCharacteristics::a380_edp() }
    }
    /// `shaft_rpm` is the pump's own input shaft speed (engine HP spool
    /// through the accessory gearbox's fixed ratio, computed by the engine
    /// area's model); `outlet_gauge_pa`/`inlet_gauge_pa` are last step's
    /// converged network pressures at this pump's delivery port and the
    /// reservoir's own reported inlet pressure.
    pub fn step(&self, shaft_rpm: f64, outlet_gauge_pa: f64, inlet_gauge_pa: f64, faults: &PumpFaults) -> PumpOutputs {
        pump_common(self.characteristics.displacement_in3(outlet_gauge_pa), shaft_rpm, outlet_gauge_pa, inlet_gauge_pa, faults)
    }
}

/// An electric motor pump: constant-speed (its own motor governor holds
/// `regulated_rpm`), spinning up/down over `spin_time_constant_s` rather
/// than snapping instantly (an exact exponential step, unconditionally
/// stable regardless of `dt`, matching this crate's own established
/// first-order-lag convention, e.g. `physics::engine::oil.rs`'s chamber
/// temperatures).
#[derive(Clone, Copy, Debug)]
pub struct ElectricPump {
    characteristics: PumpCharacteristics,
    regulated_rpm: f64,
    spin_time_constant_s: f64,
    /// Motor electrical efficiency -- GENERIC, typical of an aviation
    /// brushless/induction hydraulic motor pump; same equation form as
    /// `physics::fluids::pump_current_a` (`I = P_hydraulic / (eff * V)`),
    /// restated here to stay self-contained.
    motor_efficiency: f64,
    speed_rpm: f64,
}
impl ElectricPump {
    /// FlyByWire's `A380_EPUMP_REGULATED_SPEED_RPM` = 8000 rpm (`pumps.rs`
    /// line 57). All four of the A380's electric pumps are the same unit --
    /// FlyByWire builds green A/B and yellow A/B from one
    /// `PumpCharacteristics::a380_electric_pump()`
    /// (`a380_systems/src/hydraulic/mod.rs:1934-1985`).
    pub fn a380_electric() -> Self {
        Self { characteristics: PumpCharacteristics::a380_electric(), regulated_rpm: 8000.0, spin_time_constant_s: 0.4, motor_efficiency: 0.85, speed_rpm: 0.0 }
    }

    pub fn speed_rpm(&self) -> f64 {
        self.speed_rpm
    }

    /// `powered`: the motor contactor is closed and bus voltage is present.
    /// Returns the pump's hydraulic outputs plus the motor's electrical
    /// current draw, A.
    pub fn step(&mut self, powered: bool, outlet_gauge_pa: f64, inlet_gauge_pa: f64, bus_voltage_v: f64, faults: &PumpFaults, dt_s: f64) -> (PumpOutputs, f64) {
        // Seizure is applied once, uniformly, inside `pump_common` (it acts
        // on the pump end, not the motor): the motor itself still spins up
        // to its normal regulated speed against a jammed pump end.
        let target_rpm = if powered { self.regulated_rpm } else { 0.0 };
        let k = 1.0 / self.spin_time_constant_s.max(1e-3);
        let dt = dt_s.max(0.0);
        self.speed_rpm = target_rpm + (self.speed_rpm - target_rpm) * (-k * dt).exp();
        let out = pump_common(self.characteristics.displacement_in3(outlet_gauge_pa), self.speed_rpm, outlet_gauge_pa, inlet_gauge_pa, faults);
        let current_a = if bus_voltage_v > 0.0 && self.motor_efficiency > 0.0 {
            out.shaft_power_w / (self.motor_efficiency * bus_voltage_v)
        } else {
            0.0
        };
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

        // The motor's spin-down is a first-order lag (`spin_time_constant_s`
        // = 0.4 s), and an exponential is asymptotic: it never reaches
        // exactly zero, so asserting `== 0.0` here would be asserting that
        // the model is *not* a first-order lag. Hand solve: 201 unpowered
        // steps of 0.02 s = 4.02 s = 10.05 time constants, so the shaft is
        // at exp(-10.05) = 4.3e-5 of the 8000 rpm it was turning, i.e.
        // ~0.34 rpm. A fixed-displacement pump's delivered flow, and hence
        // its shaft power and motor current, are all linear in shaft speed,
        // so each must be that same 4.3e-5 fraction of its running value.
        // Bound them at 1e-4 of the running value: about 2.3x the predicted
        // residual (so a correct 0.4 s lag passes with margin) but still
        // four decades below the running value, and in absolute terms well
        // under a millilitre per minute and a microamp -- nothing the rest
        // of the network can resolve. A pump that genuinely failed to spin
        // down would sit at ~1.0 of the running value and fail this by four
        // orders of magnitude.
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
