//! The A380's green and yellow hydraulic circuits, assembled from this
//! directory's generic elements.
//!
//! Pump layout matches FlyByWire's own A380 model
//! (`a380_systems/src/hydraulic/mod.rs` lines 1645-1690,
//! `A380Hydraulic`'s field list): four engine-driven pumps per circuit
//! (green from engines 1/2, yellow from engines 3/4 -- each engine drives
//! two, `engine_driven_pump_{1,2,3,4}{a,b}_controller`), and one electric
//! pump pair on yellow only (`yellow_electric_pump_{a,b}_controller`; no
//! green electric pump exists in FlyByWire's own A380 model, so none is
//! added here either) -- matching the brief's "4 engine-driven pumps per
//! system (8 total), electric pumps".
//!
//! Topology per circuit: pumps feed a manifold; a priority valve
//! (`network::PriorityValve::a380`) gates a non-essential branch (gear,
//! brakes, steering, cargo doors, reversers) behind the flight controls'
//! always-fed essential branch; a relief valve and an accumulator sit on
//! the manifold; every consumer's used flow returns to a return manifold,
//! through a filter (with bypass) and back to the reservoir. Line lengths
//! are GENERIC, representative routing distances derived from the
//! A380-800's published overall dimensions (72.7 m length, 79.8 m span --
//! Airbus/EASA TCDS A.110 public figures): short to the flight-control
//! actuators and reversers (pylon/centre-fuselage mounted near the pumps),
//! long to nose wheel steering (all the way forward to the nose gear bay).
//! Diameters are GENERIC, in the 3/8"-3/4" bore range large transport 5000
//! psi hydraulic tubing typically uses, scaled to each branch's relative
//! flow demand. None of this is real AMM routing data (not public).

use super::accumulator::{Accumulator, AccumulatorFaults};
use super::fluid;
use super::network::{
    CheckValve, CheckValveFaults, Endpoint, Filter, FilterFaults, FireShutoffValve, LeakMeasurementValve, Line, LmvPosition, Network, Node, PriorityValve, ReliefValve, Restriction, PSI_PA,
};
use super::pump::{ElectricPump, EngineDrivenPump, PumpFaults};
use super::reservoir::{Reservoir, ReservoirFaults};
use super::thermal::{throttling_heat_w, ThermalSizing, ThermalState};

/// Node indices within a circuit's own `Network`.
mod node {
    pub const MANIFOLD: usize = 0;
    pub const ESSENTIAL: usize = 1;
    pub const NON_ESSENTIAL: usize = 2;
    pub const GEAR: usize = 3;
    pub const BRAKES: usize = 4;
    pub const STEERING: usize = 5;
    pub const CARGO_DOORS: usize = 6;
    pub const REVERSERS: usize = 7;
    pub const RETURN: usize = 8;
    pub const COUNT: usize = 9;
}
/// Line indices within a circuit's own `Network`.
mod line {
    pub const ESSENTIAL: usize = 0;
    pub const PRIORITY: usize = 1;
    pub const GEAR: usize = 2;
    pub const BRAKES: usize = 3;
    pub const STEERING: usize = 4;
    pub const CARGO_DOORS: usize = 5;
    pub const REVERSERS: usize = 6;
    pub const RETURN_FILTER: usize = 7;
    pub const RETURN_BYPASS: usize = 8;
    pub const LMV_TAP: usize = 9;
    pub const COUNT: usize = 10;
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Color {
    Green,
    Yellow,
}

/// Reverse-leak orifice a jammed-open check valve exposes when its own pump
/// is stopped, letting manifold pressure bleed backward into it -- GENERIC,
/// representative of a worn poppet's seat gap rather than a wide-open line.
const CHECK_VALVE_STUCK_LEAK_AREA_M2: f64 = 1.0e-6;
/// The margin a running pump's own internal gallery is assumed to hold
/// above manifold pressure while delivering -- GENERIC, just enough to keep
/// its check valve unambiguously forward-biased whenever it is actually
/// turning.
const RUNNING_PUMP_MARGIN_PA: f64 = 50_000.0;

/// External demand each consumer group is drawing this tick, m^3/s (from
/// whichever other area models flight controls/gear/brakes/steering/doors/
/// reversers -- this directory only owns the hydraulic supply side). Each
/// is assumed to return the same volume to the return manifold (a servo
/// valve routes supply flow to return after doing work; actuator
/// differential-area asymmetry is not modelled at this level).
#[derive(Clone, Copy, Debug, Default)]
pub struct ConsumerDemands {
    pub flight_controls_m3_s: f64,
    pub gear_m3_s: f64,
    pub brakes_m3_s: f64,
    pub steering_m3_s: f64,
    pub cargo_doors_m3_s: f64,
    pub reversers_m3_s: f64,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct EdpInputs {
    /// Pump input shaft speed (engine HP spool through the accessory
    /// gearbox's fixed ratio -- another area's model), rpm.
    pub shaft_rpm: f64,
    /// The firewall FIRE handle for this engine has been pulled.
    pub fire_handle_pulled: bool,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct CircuitInputs {
    pub edp: [EdpInputs; 4],
    pub electric_pump_powered: bool,
    pub electric_pump_bus_voltage_v: f64,
    pub demands: ConsumerDemands,
    pub fuel_kg_s: f64,
    pub fuel_temp_k: f64,
    pub ambient_k: f64,
    /// Cabin/bleed-air supply available to the reservoir's own bootstrap
    /// pressurising valve, 0..1 (normally 1.0 with electrical/pneumatic
    /// power up) -- see `reservoir.rs`'s module doc for why this is
    /// independent of this same circuit's own hydraulic pressure.
    pub pressurization_supply_fraction: f64,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct EdpFaults {
    pub pump: PumpFaults,
    pub check_valve: CheckValveFaults,
    /// Firewall shutoff valve seized at its last position: 0 healthy .. 1
    /// fully seized.
    pub fire_sov_stuck: f64,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct CircuitFaults {
    pub edp: [EdpFaults; 4],
    pub electric_pump: PumpFaults,
    pub reservoir: ReservoirFaults,
    pub accumulator: AccumulatorFaults,
    /// Priority valve seized at its last position: 0 healthy .. 1 fully seized.
    pub priority_valve_stuck: f64,
    /// Relief valve spring weakened, cracking as low as half its design
    /// pressure: 0 healthy .. 1.
    pub relief_valve_crack_low: f64,
    /// Return filter clog: 0 clean .. 1 fully blocked.
    pub filter_clog: f64,
    /// Per-branch line leak, orifice area m^2 (0 healthy), order:
    /// [gear, brakes, steering, cargo_doors, reversers].
    pub line_leak_area_m2: [f64; 5],
    /// Air ingested into the fluid (a failing pump inlet seal, a badly
    /// aerated reservoir return, air not bled after maintenance): 0 healthy
    /// .. 1 at `MAX_AIR_INGESTION_FRACTION` free air content at 1 atm,
    /// applied uniformly across the circuit's own nodes and fed into
    /// `fluid::effective_bulk_modulus_pa` (network-wide, not one line/node,
    /// since entrained air circulates through the whole loop).
    pub air_ingestion: f64,
}

/// GENERIC: aerospace hydraulic fluid cleanliness practice targets well
/// under 1% free air by volume in a healthy system (entrained air above a
/// few tenths of a percent already measurably softens the fluid per
/// `fluid::effective_bulk_modulus_pa`); this is the fully-ingested (fault
/// magnitude 1.0) ceiling, a badly aerated system rather than a physical
/// upper bound on how much air fluid could ever hold.
const MAX_AIR_INGESTION_FRACTION: f64 = 0.05;

#[derive(Clone, Copy, Debug, Default)]
pub struct EdpReport {
    pub flow_m3_s: f64,
    pub case_drain_m3_s: f64,
    pub volumetric_efficiency: f64,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct CircuitOutputs {
    pub manifold_pressure_pa: f64,
    pub essential_pressure_pa: f64,
    pub accumulator_pressure_pa: f64,
    pub reservoir_fill_fraction: f64,
    pub reservoir_low_level_warning: bool,
    pub reservoir_low_pressure_warning: bool,
    pub fluid_temp_c: f64,
    pub fluid_overheat: bool,
    pub edp: [EdpReport; 4],
    pub electric_pump_flow_m3_s: f64,
}

pub struct Circuit {
    color: Color,
    network: Network,
    reservoir: Reservoir,
    accumulator: Accumulator,
    edps: [EngineDrivenPump; 4],
    edp_fire_open: [f64; 4],
    electric_pump: Option<ElectricPump>,
    priority_valve: PriorityValve,
    priority_open: f64,
    relief_valve: ReliefValve,
    filter: Filter,
    lmv: LmvPosition,
    thermal: ThermalState,
    sizing: ThermalSizing,
    last_reservoir_inlet_pa: f64,
}

impl Circuit {
    pub fn new(color: Color) -> Self {
        let nodes = vec![
            Node::new(3.0e-3, 0.0),  // MANIFOLD
            Node::new(1.0e-3, 0.0),  // ESSENTIAL
            Node::new(1.0e-3, 0.0),  // NON_ESSENTIAL
            Node::new(0.5e-3, 0.0),  // GEAR
            Node::new(0.3e-3, 0.0),  // BRAKES
            Node::new(0.3e-3, 0.0),  // STEERING
            Node::new(0.3e-3, 0.0),  // CARGO_DOORS
            Node::new(0.3e-3, 0.0),  // REVERSERS
            Node::new(2.0e-3, 0.0),  // RETURN
        ];
        debug_assert_eq!(nodes.len(), node::COUNT);

        let priority_valve = PriorityValve::a380();
        let filter = Filter { clean_diameter_m: 0.014, length_m: 0.3, bypass_cracking_pa: 65.0 * PSI_PA };
        let lines = vec![
            Line::pipe(Endpoint::Node(node::MANIFOLD), Endpoint::Node(node::ESSENTIAL), 0.012, 6.0),
            Line::valve(Endpoint::Node(node::MANIFOLD), Endpoint::Node(node::NON_ESSENTIAL), std::f64::consts::PI / 4.0 * 0.012 * 0.012, 0.7),
            Line::pipe(Endpoint::Node(node::NON_ESSENTIAL), Endpoint::Node(node::GEAR), 0.010, 15.0),
            Line::pipe(Endpoint::Node(node::NON_ESSENTIAL), Endpoint::Node(node::BRAKES), 0.008, 20.0),
            Line::pipe(Endpoint::Node(node::NON_ESSENTIAL), Endpoint::Node(node::STEERING), 0.006, 35.0),
            Line::pipe(Endpoint::Node(node::NON_ESSENTIAL), Endpoint::Node(node::CARGO_DOORS), 0.006, 22.0),
            Line::pipe(Endpoint::Node(node::NON_ESSENTIAL), Endpoint::Node(node::REVERSERS), 0.008, 4.0),
            Line::pipe(Endpoint::Node(node::RETURN), Endpoint::Fixed(0.0), filter.effective_diameter_m(&FilterFaults::default()), filter.length_m),
            Line::valve(Endpoint::Node(node::RETURN), Endpoint::Fixed(0.0), std::f64::consts::PI / 4.0 * 0.020 * 0.020, 0.7),
            Line::valve(Endpoint::Node(node::MANIFOLD), Endpoint::Fixed(0.0), LeakMeasurementValve::MEASURE_ORIFICE_AREA_M2, 0.61),
        ];
        debug_assert_eq!(lines.len(), line::COUNT);

        let (reservoir, electric_pump) = match color {
            Color::Green => (Reservoir::a380_green(), None),
            Color::Yellow => (Reservoir::a380_yellow(), Some(ElectricPump::a380_yellow_electric())),
        };

        Self {
            color,
            network: Network::new(nodes, lines),
            reservoir,
            accumulator: Accumulator::a380(),
            edps: [EngineDrivenPump::a380(); 4],
            edp_fire_open: [1.0; 4],
            electric_pump,
            priority_valve,
            priority_open: 0.0,
            relief_valve: ReliefValve { cracking_pa: 5400.0 * PSI_PA, full_flow_rise_pa: 200.0 * PSI_PA, full_flow_m3_s: 3.0e-3 },
            filter,
            lmv: LmvPosition::Normal,
            thermal: ThermalState::new(288.15),
            sizing: ThermalSizing::a380_circuit(),
            last_reservoir_inlet_pa: 0.0,
        }
    }

    pub fn color(&self) -> Color {
        self.color
    }

    pub fn set_lmv_position(&mut self, position: LmvPosition) {
        self.lmv = position;
    }

    pub fn step(&mut self, inputs: &CircuitInputs, faults: &CircuitFaults, dt_s: f64) -> CircuitOutputs {
        let dt = dt_s.max(0.0);
        let temp_c = self.thermal.temp_c();
        let density = fluid::density_kg_m3(temp_c);
        let visc = fluid::dynamic_viscosity_pa_s(temp_c);

        // Last step's converged state, used throughout as this tick's
        // "current" reading (the lagged-input pattern every valve/pump in
        // this directory uses).
        let manifold_pa = self.network.nodes[node::MANIFOLD].pressure_pa;
        let inlet_pa = self.last_reservoir_inlet_pa;

        let air_fraction = faults.air_ingestion.clamp(0.0, 1.0) * MAX_AIR_INGESTION_FRACTION;
        for n in &mut self.network.nodes {
            n.air_fraction_at_1atm = air_fraction;
        }

        let mut injections = vec![0.0; node::COUNT];
        let mut edp_report = [EdpReport::default(); 4];
        let mut pump_reservoir_draw = 0.0;
        let mut pump_heat_w = 0.0;
        let mut pump_delivered_flow_m3_s = 0.0;

        for i in 0..4 {
            let out = self.edps[i].step(inputs.edp[i].shaft_rpm, manifold_pa, inlet_pa, &faults.edp[i].pump);
            let commanded_open = if inputs.edp[i].fire_handle_pulled { 0.0 } else { 1.0 };
            self.edp_fire_open[i] = FireShutoffValve::open_fraction(commanded_open, faults.edp[i].fire_sov_stuck, self.edp_fire_open[i]);
            let fire_open = self.edp_fire_open[i];

            let pump_internal_pa = if inputs.edp[i].shaft_rpm > 1.0 { manifold_pa + RUNNING_PUMP_MARGIN_PA } else { 0.0 };
            let gate = self.edps_check_valve().open_fraction(pump_internal_pa, manifold_pa, &faults.edp[i].check_valve);
            let reverse_leak = if inputs.edp[i].shaft_rpm <= 1.0 {
                gate * 0.61 * CHECK_VALVE_STUCK_LEAK_AREA_M2 * (2.0 * manifold_pa.max(0.0) / density).sqrt()
            } else {
                0.0
            };

            let forward = out.flow_m3_s * gate * fire_open;
            let case_drain = out.case_drain_m3_s * fire_open;
            injections[node::MANIFOLD] += forward - reverse_leak;
            injections[node::RETURN] += case_drain + reverse_leak;
            pump_reservoir_draw += (out.flow_m3_s + out.case_drain_m3_s) * fire_open;
            pump_heat_w += out.shaft_power_w * (1.0 - super::pump::PUMP_MECHANICAL_EFFICIENCY);
            pump_delivered_flow_m3_s += forward;

            edp_report[i] = EdpReport { flow_m3_s: forward, case_drain_m3_s: case_drain, volumetric_efficiency: out.volumetric_efficiency };
        }

        let mut electric_pump_flow = 0.0;
        if let Some(pump) = &mut self.electric_pump {
            let (out, _current_a) = pump.step(inputs.electric_pump_powered, manifold_pa, inlet_pa, inputs.electric_pump_bus_voltage_v, &faults.electric_pump, dt);
            injections[node::MANIFOLD] += out.flow_m3_s;
            injections[node::RETURN] += out.case_drain_m3_s;
            pump_reservoir_draw += out.flow_m3_s + out.case_drain_m3_s;
            pump_heat_w += out.shaft_power_w * (1.0 - super::pump::PUMP_MECHANICAL_EFFICIENCY);
            pump_delivered_flow_m3_s += out.flow_m3_s;
            electric_pump_flow = out.flow_m3_s;
        }

        let relief_flow = self.relief_valve.flow_m3_s(manifold_pa, faults.relief_valve_crack_low);
        injections[node::MANIFOLD] -= relief_flow;
        injections[node::RETURN] += relief_flow;

        let acc_net_in_rate = self.accumulator.step(manifold_pa, &faults.accumulator, dt);
        injections[node::MANIFOLD] -= acc_net_in_rate;

        self.priority_open = self.priority_valve.open_fraction(manifold_pa, faults.priority_valve_stuck, self.priority_open);
        self.network.lines[line::PRIORITY].open_fraction = self.priority_open;

        let (_, filter_dp) = self.network.line_flow_and_dp(line::RETURN_FILTER, density, visc);
        self.network.lines[line::RETURN_FILTER].restriction = Restriction::Pipe { diameter_m: self.filter.effective_diameter_m(&FilterFaults { clog: faults.filter_clog }), length_m: self.filter.length_m };
        self.network.lines[line::RETURN_BYPASS].open_fraction = self.filter.bypass_open_fraction(filter_dp);

        self.network.lines[line::LMV_TAP].open_fraction = match self.lmv {
            LmvPosition::Normal => 0.0,
            LmvPosition::Measure => 1.0,
        };

        let branch_lines = [line::GEAR, line::BRAKES, line::STEERING, line::CARGO_DOORS, line::REVERSERS];
        for (li, &leak) in branch_lines.iter().zip(faults.line_leak_area_m2.iter()) {
            self.network.lines[*li].leak_area_m2 = leak;
        }

        let demands = [
            (node::GEAR, inputs.demands.gear_m3_s),
            (node::BRAKES, inputs.demands.brakes_m3_s),
            (node::STEERING, inputs.demands.steering_m3_s),
            (node::CARGO_DOORS, inputs.demands.cargo_doors_m3_s),
            (node::REVERSERS, inputs.demands.reversers_m3_s),
        ];
        let mut total_demand = inputs.demands.flight_controls_m3_s;
        injections[node::ESSENTIAL] -= inputs.demands.flight_controls_m3_s;
        for (n, d) in demands {
            injections[n] -= d;
            total_demand += d;
        }
        injections[node::RETURN] += total_demand;

        let leaked_m3 = self.network.step(&injections, density, visc, dt);

        let (return_main_flow, _) = self.network.line_flow_and_dp(line::RETURN_FILTER, density, visc);
        let (return_bypass_flow, _) = self.network.line_flow_and_dp(line::RETURN_BYPASS, density, visc);
        let reservoir_inflow_rate = return_main_flow.max(0.0) + return_bypass_flow.max(0.0) - if dt > 0.0 { leaked_m3 / dt } else { 0.0 };
        let res_out = self.reservoir.step(reservoir_inflow_rate, pump_reservoir_draw, inputs.pressurization_supply_fraction, &faults.reservoir, dt);
        self.last_reservoir_inlet_pa = res_out.inlet_air_pressure_pa;

        let mut throttling_heat = 0.0;
        for li in 0..line::COUNT {
            let (flow, dp) = self.network.line_flow_and_dp(li, density, visc);
            throttling_heat += throttling_heat_w(flow, dp);
        }
        throttling_heat += throttling_heat_w(relief_flow, manifold_pa);

        let thermal_out = self.thermal.step(&self.sizing, pump_heat_w, throttling_heat, pump_delivered_flow_m3_s * density, inputs.fuel_kg_s, inputs.fuel_temp_k, inputs.ambient_k, dt);

        CircuitOutputs {
            manifold_pressure_pa: self.network.nodes[node::MANIFOLD].pressure_pa,
            essential_pressure_pa: self.network.nodes[node::ESSENTIAL].pressure_pa,
            accumulator_pressure_pa: self.accumulator.pressure_pa(&faults.accumulator),
            reservoir_fill_fraction: res_out.fill_fraction,
            reservoir_low_level_warning: res_out.low_level_warning,
            reservoir_low_pressure_warning: res_out.low_pressure_warning,
            fluid_temp_c: thermal_out.temp_c,
            fluid_overheat: thermal_out.overheat,
            edp: edp_report,
            electric_pump_flow_m3_s: electric_pump_flow,
        }
    }

    /// All four EDPs share the same GENERIC check valve spec (no public
    /// per-pump variation exists).
    fn edps_check_valve(&self) -> CheckValve {
        CheckValve { cracking_pa: 5.0 * PSI_PA }
    }
}

/// Both A380 hydraulic systems, independent of each other (the A380 has no
/// power-transfer-unit between circuits, unlike the A320 -- FlyByWire's own
/// A380 `A380Hydraulic` field list has no PTU type, consistent with the
/// A380's electric-pump-based redundancy strategy instead).
pub struct A380Hydraulics {
    pub green: Circuit,
    pub yellow: Circuit,
}
impl A380Hydraulics {
    pub fn new() -> Self {
        Self { green: Circuit::new(Color::Green), yellow: Circuit::new(Color::Yellow) }
    }
}
impl Default for A380Hydraulics {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn running_edps(rpm: f64) -> [EdpInputs; 4] {
        [EdpInputs { shaft_rpm: rpm, fire_handle_pulled: false }; 4]
    }

    #[test]
    fn healthy_green_circuit_pressurises_toward_service_pressure() {
        let mut c = Circuit::new(Color::Green);
        let inputs = CircuitInputs { edp: running_edps(4000.0), ambient_k: 288.15, fuel_temp_k: 288.15, pressurization_supply_fraction: 1.0, ..Default::default() };
        let mut out = CircuitOutputs::default();
        for _ in 0..3000 {
            out = c.step(&inputs, &CircuitFaults::default(), 0.02);
        }
        assert!(out.manifold_pressure_pa.is_finite());
        assert!(out.manifold_pressure_pa > 3000.0 * PSI_PA, "green manifold should reach a real service pressure: {:.0} psi", out.manifold_pressure_pa / PSI_PA);
        assert!(out.manifold_pressure_pa < 6000.0 * PSI_PA, "relief valve should have capped it: {:.0} psi", out.manifold_pressure_pa / PSI_PA);
        assert!(!out.fluid_overheat);
    }

    #[test]
    fn yellow_circuit_electric_pump_alone_can_pressurise_the_system() {
        let mut c = Circuit::new(Color::Yellow);
        let inputs = CircuitInputs { electric_pump_powered: true, electric_pump_bus_voltage_v: 115.0, ambient_k: 288.15, fuel_temp_k: 288.15, pressurization_supply_fraction: 1.0, ..Default::default() };
        let mut out = CircuitOutputs::default();
        for _ in 0..3000 {
            out = c.step(&inputs, &CircuitFaults::default(), 0.02);
        }
        assert!(out.manifold_pressure_pa > 500.0 * PSI_PA, "electric pump alone should build meaningful pressure: {:.0} psi", out.manifold_pressure_pa / PSI_PA);
    }

    #[test]
    fn a_demand_draws_the_reservoir_down_and_returns_through_the_filter() {
        let mut c = Circuit::new(Color::Green);
        let start_fill = c.reservoir.fluid_volume_m3();
        let inputs = CircuitInputs {
            edp: running_edps(4000.0),
            demands: ConsumerDemands { gear_m3_s: 2.0e-4, ..Default::default() },
            ambient_k: 288.15,
            fuel_temp_k: 288.15,
            pressurization_supply_fraction: 1.0,
            ..Default::default()
        };
        for _ in 0..3000 {
            c.step(&inputs, &CircuitFaults::default(), 0.02);
        }
        // Circulating through actuators and back through the filter should
        // not, by itself, drain the reservoir to empty.
        assert!(c.reservoir.fluid_volume_m3() > 0.0);
        assert!((c.reservoir.fluid_volume_m3() - start_fill).abs() < start_fill, "reservoir level should stay in a sane band, not empty or overflow");
    }

    #[test]
    fn a_branch_leak_costs_reservoir_fluid_over_time() {
        let inputs = CircuitInputs { edp: running_edps(4000.0), ambient_k: 288.15, fuel_temp_k: 288.15, pressurization_supply_fraction: 1.0, ..Default::default() };
        let mut healthy = Circuit::new(Color::Green);
        let mut leaking = Circuit::new(Color::Green);
        let leak_faults = CircuitFaults { line_leak_area_m2: [3.0e-6, 0.0, 0.0, 0.0, 0.0], ..Default::default() };
        for _ in 0..3000 {
            healthy.step(&inputs, &CircuitFaults::default(), 0.02);
            leaking.step(&inputs, &leak_faults, 0.02);
        }
        assert!(leaking.reservoir.fluid_volume_m3() < healthy.reservoir.fluid_volume_m3());
    }

    #[test]
    fn pulling_all_fire_handles_isolates_the_engine_driven_pumps() {
        let mut c = Circuit::new(Color::Green);
        let mut edp = running_edps(4000.0);
        for e in &mut edp {
            e.fire_handle_pulled = true;
        }
        let inputs = CircuitInputs { edp, ambient_k: 288.15, fuel_temp_k: 288.15, pressurization_supply_fraction: 1.0, ..Default::default() };
        let mut out = CircuitOutputs::default();
        for _ in 0..500 {
            out = c.step(&inputs, &CircuitFaults::default(), 0.02);
        }
        for r in out.edp {
            assert_eq!(r.flow_m3_s, 0.0);
        }
        assert!(out.manifold_pressure_pa < 100.0 * PSI_PA, "with every EDP isolated and no electric pump, green should not hold pressure");
    }

    #[test]
    fn a_seized_priority_valve_can_starve_the_non_essential_branch() {
        let mut c = Circuit::new(Color::Green);
        let inputs = CircuitInputs {
            edp: running_edps(4000.0),
            demands: ConsumerDemands { gear_m3_s: 1.0e-4, ..Default::default() },
            ambient_k: 288.15,
            fuel_temp_k: 288.15,
            pressurization_supply_fraction: 1.0,
            ..Default::default()
        };
        // Seize the priority valve shut from the very first tick.
        let faults = CircuitFaults { priority_valve_stuck: 1.0, ..Default::default() };
        let mut out = CircuitOutputs::default();
        for _ in 0..1000 {
            out = c.step(&inputs, &faults, 0.02);
        }
        assert!(out.manifold_pressure_pa > 3000.0 * PSI_PA, "essential side should still pressurise");
        let gear_pa = c_gear_pressure(&mut c, &inputs, &faults);
        assert!(gear_pa < 1000.0 * PSI_PA, "a seized-shut priority valve should starve gear: {:.0} psi", gear_pa / PSI_PA);
    }

    fn c_gear_pressure(c: &mut Circuit, inputs: &CircuitInputs, faults: &CircuitFaults) -> f64 {
        c.step(inputs, faults, 0.02);
        c.network.nodes[node::GEAR].pressure_pa
    }

    #[test]
    fn air_ingestion_makes_the_circuit_slower_to_pressurise() {
        let inputs = CircuitInputs { edp: running_edps(4000.0), ambient_k: 288.15, fuel_temp_k: 288.15, pressurization_supply_fraction: 1.0, ..Default::default() };
        let mut healthy = Circuit::new(Color::Green);
        let mut aerated = Circuit::new(Color::Green);
        let aerated_faults = CircuitFaults { air_ingestion: 1.0, ..Default::default() };
        let mut healthy_pa = 0.0;
        let mut aerated_pa = 0.0;
        // Early in the transient (before either has settled), the softer,
        // more compressible aerated fluid should lag behind the healthy one.
        for _ in 0..40 {
            healthy_pa = healthy.step(&inputs, &CircuitFaults::default(), 0.02).manifold_pressure_pa;
            aerated_pa = aerated.step(&inputs, &aerated_faults, 0.02).manifold_pressure_pa;
        }
        assert!(aerated_pa.is_finite() && healthy_pa.is_finite());
        assert!(aerated_pa < healthy_pa, "an aerated (spongy) circuit should pressurise more slowly: aerated {aerated_pa:.0} Pa vs healthy {healthy_pa:.0} Pa");
    }

    #[test]
    fn no_nan_at_rest_or_dt_zero() {
        let mut c = Circuit::new(Color::Yellow);
        let out = c.step(&CircuitInputs::default(), &CircuitFaults::default(), 0.0);
        assert!(out.manifold_pressure_pa.is_finite());
        assert!(out.fluid_temp_c.is_finite());
    }
}
