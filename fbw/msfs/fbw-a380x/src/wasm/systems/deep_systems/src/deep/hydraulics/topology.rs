use super::accumulator::{Accumulator, AccumulatorFaults};
use super::fluid;
use super::network::{
    CheckValve, CheckValveFaults, Endpoint, Filter, FilterFaults, FireShutoffValve, LeakMeasurementValve, Line, LmvPosition, Network, Node, PriorityValve, ReliefValve, Restriction, PSI_PA,
};
use super::pump::{ElectricPump, EngineDrivenPump, PumpFaults};
use super::reservoir::{Reservoir, ReservoirFaults};
use super::thermal::{throttling_heat_w, ThermalSizing, ThermalState};

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

const CHECK_VALVE_STUCK_LEAK_AREA_M2: f64 = 1.0e-6;
const RUNNING_PUMP_MARGIN_PA: f64 = 50_000.0;

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
    pub shaft_rpm: f64,
    pub fire_handle_pulled: bool,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct CircuitInputs {
    pub edp: [EdpInputs; 4],
    pub electric_pump_powered: [bool; 2],
    pub electric_pump_bus_voltage_v: [f64; 2],
    pub demands: ConsumerDemands,
    pub fuel_kg_s: f64,
    pub fuel_temp_k: f64,
    pub ambient_k: f64,
    pub pressurization_supply_fraction: f64,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct EdpFaults {
    pub pump: PumpFaults,
    pub check_valve: CheckValveFaults,
    pub fire_sov_stuck: f64,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct CircuitFaults {
    pub edp: [EdpFaults; 4],
    pub electric_pump: [PumpFaults; 2],
    pub reservoir: ReservoirFaults,
    pub accumulator: AccumulatorFaults,
    pub priority_valve_stuck: f64,
    pub relief_valve_crack_low: f64,
    pub filter_clog: f64,
    pub line_leak_area_m2: [f64; 5],
    pub air_ingestion: f64,
}

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
    pub manifold_temp_c: f64,
    pub manifold_overheat: bool,
    pub edp: [EdpReport; 4],
    pub electric_pump_flow_m3_s: [f64; 2],
    pub node_pressures_pa: [f64; NODE_COUNT],
    pub reservoir_inflow_m3_s: f64,
    pub reservoir_outflow_m3_s: f64,
    pub network_leaked_m3_s: f64,
    pub reservoir_leaked_m3_s: f64,
    pub relief_flow_m3_s: f64,
    pub accumulator_flow_m3_s: f64,
    pub reservoir_inlet_pa: f64,
    pub conservation: ConservationDiag,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct ConservationDiag {
    pub solver_m3: f64,
    pub relief_lag_m3: f64,
    pub case_drain_lag_m3: f64,
    pub check_valve_m3: f64,
    pub return_clamp_m3: f64,
    pub reservoir_clamp_m3: f64,
    pub accumulator_clamp_m3: f64,
    pub lmv_tap_m3: f64,
}

impl ConservationDiag {
    pub fn total_m3(&self) -> f64 {
        self.solver_m3 + self.relief_lag_m3 + self.case_drain_lag_m3 + self.check_valve_m3 + self.return_clamp_m3 + self.reservoir_clamp_m3 + self.accumulator_clamp_m3 + self.lmv_tap_m3
    }
}

pub const NODE_COUNT: usize = node::COUNT;

pub struct Circuit {
    color: Color,
    network: Network,
    reservoir: Reservoir,
    accumulator: Accumulator,
    edps: [EngineDrivenPump; 4],
    edp_fire_open: [f64; 4],
    electric_pumps: [ElectricPump; 2],
    priority_valve: PriorityValve,
    priority_open: f64,
    relief_valve: ReliefValve,
    filter: Filter,
    lmv: LmvPosition,
    thermal: ThermalState,
    sizing: ThermalSizing,
    manifold_thermal: ThermalState,
    manifold_sizing: ThermalSizing,
    last_reservoir_inlet_pa: f64,
}

impl Circuit {
    pub fn new(color: Color) -> Self {
        let nodes = vec![
            Node::new(3.0e-3, 0.0),
            Node::new(1.0e-3, 0.0),
            Node::new(1.0e-3, 0.0),
            Node::new(0.5e-3, 0.0),
            Node::new(0.3e-3, 0.0),
            Node::new(0.3e-3, 0.0),
            Node::new(0.3e-3, 0.0),
            Node::new(0.3e-3, 0.0),
            Node::new(2.0e-3, 0.0),
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

        let reservoir = match color {
            Color::Green => Reservoir::a380_green(),
            Color::Yellow => Reservoir::a380_yellow(),
        };

        Self {
            color,
            network: Network::new(nodes, lines),
            reservoir,
            accumulator: Accumulator::a380(),
            edps: [EngineDrivenPump::a380(); 4],
            edp_fire_open: [1.0; 4],
            electric_pumps: [ElectricPump::a380_electric(); 2],
            priority_valve,
            priority_open: 0.0,
            relief_valve: ReliefValve { cracking_pa: 5400.0 * PSI_PA, full_flow_rise_pa: 200.0 * PSI_PA, full_flow_m3_s: 3.0e-3 },
            filter,
            lmv: LmvPosition::Normal,
            thermal: ThermalState::new(288.15),
            sizing: ThermalSizing::a380_circuit(),
            manifold_thermal: ThermalState::new(288.15),
            manifold_sizing: ThermalSizing::a380_manifold(),
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
        let mut case_drain_heat_w = 0.0;
        let mut case_drain_flow_m3_s = 0.0;
        let mut pump_delivered_flow_m3_s = 0.0;

        let mut case_drain_injected = 0.0;
        let mut edp_gate = [0.0; 4];
        let mut edp_fire_open_now = [0.0; 4];
        for i in 0..4 {
            let commanded_open = if inputs.edp[i].fire_handle_pulled { 0.0 } else { 1.0 };
            self.edp_fire_open[i] = FireShutoffValve::open_fraction(commanded_open, faults.edp[i].fire_sov_stuck, self.edp_fire_open[i]);
            edp_fire_open_now[i] = self.edp_fire_open[i];

            let pump_internal_pa = if inputs.edp[i].shaft_rpm > 1.0 { manifold_pa + RUNNING_PUMP_MARGIN_PA } else { 0.0 };
            edp_gate[i] = self.edps_check_valve().open_fraction(pump_internal_pa, manifold_pa, &faults.edp[i].check_valve);
            let reverse_leak = if inputs.edp[i].shaft_rpm <= 1.0 {
                edp_gate[i] * 0.61 * CHECK_VALVE_STUCK_LEAK_AREA_M2 * (2.0 * manifold_pa.max(0.0) / density).sqrt()
            } else {
                0.0
            };
            injections[node::MANIFOLD] -= reverse_leak;
            injections[node::RETURN] += reverse_leak;

            let case_out = self.edps[i].step(inputs.edp[i].shaft_rpm, manifold_pa, inlet_pa, &faults.edp[i].pump);
            let case_drain = case_out.case_drain_m3_s * edp_fire_open_now[i];
            case_drain_injected += case_drain;
            injections[node::RETURN] += case_drain;
            edp_report[i].case_drain_m3_s = case_drain;
        }

        for (i, pump) in self.electric_pumps.iter_mut().enumerate() {
            pump.advance_speed(inputs.electric_pump_powered[i], dt);
            let out = pump.flow_at(manifold_pa, inlet_pa, &faults.electric_pump[i], dt);
            case_drain_injected += out.case_drain_m3_s;
            injections[node::RETURN] += out.case_drain_m3_s;
        }

        let relief_flow_lagged = self.relief_valve.flow_m3_s(manifold_pa, faults.relief_valve_crack_low);
        injections[node::RETURN] += relief_flow_lagged;

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

        let edps = self.edps;
        let edp_shaft_rpm: [f64; 4] = std::array::from_fn(|i| inputs.edp[i].shaft_rpm);
        let edp_pump_faults: [PumpFaults; 4] = std::array::from_fn(|i| faults.edp[i].pump);
        let electric_pumps = self.electric_pumps;
        let electric_pump_faults = faults.electric_pump;
        let relief_valve = self.relief_valve;
        let relief_crack_low = faults.relief_valve_crack_low;
        let accumulator = self.accumulator.clone();
        let accumulator_faults = faults.accumulator;
        let manifold_injection = move |p_i: f64| -> f64 {
            let mut q = 0.0;
            for i in 0..4 {
                let out = edps[i].step(edp_shaft_rpm[i], p_i, inlet_pa, &edp_pump_faults[i]);
                q += out.flow_m3_s * edp_gate[i] * edp_fire_open_now[i];
            }
            for i in 0..2 {
                q += electric_pumps[i].flow_at(p_i, inlet_pa, &electric_pump_faults[i], dt).flow_m3_s;
            }
            q -= relief_valve.flow_m3_s(p_i, relief_crack_low);
            q -= accumulator.exchange_flow_at(p_i, &accumulator_faults, dt);
            q
        };
        let mut pressure_dependent: [Option<&dyn Fn(f64) -> f64>; node::COUNT] = [None; node::COUNT];
        pressure_dependent[node::MANIFOLD] = Some(&manifold_injection);
        let leaked_m3 = self.network.step_with_pressure_dependent(&injections, &pressure_dependent, density, visc, dt);

        let manifold_pa_converged = self.network.nodes[node::MANIFOLD].pressure_pa;
        let mut case_drain_drawn = 0.0;
        let mut check_valve_held = 0.0;
        for i in 0..4 {
            let out = self.edps[i].step(edp_shaft_rpm[i], manifold_pa_converged, inlet_pa, &edp_pump_faults[i]);
            let forward = out.flow_m3_s * edp_gate[i] * edp_fire_open_now[i];
            case_drain_drawn += out.case_drain_m3_s * edp_fire_open_now[i];
            check_valve_held += out.flow_m3_s * edp_fire_open_now[i] - forward;
            pump_reservoir_draw += (out.flow_m3_s + out.case_drain_m3_s) * edp_fire_open_now[i];
            pump_heat_w += out.shaft_power_w * (1.0 - super::pump::PUMP_MECHANICAL_EFFICIENCY);
            case_drain_heat_w += out.case_drain_m3_s * edp_fire_open_now[i] * (manifold_pa_converged - inlet_pa).max(0.0);
            case_drain_flow_m3_s += out.case_drain_m3_s * edp_fire_open_now[i];
            pump_delivered_flow_m3_s += forward;
            edp_report[i].flow_m3_s = forward;
            edp_report[i].volumetric_efficiency = out.volumetric_efficiency;
        }

        let mut electric_pump_flow = [0.0; 2];
        for (i, pump) in self.electric_pumps.iter_mut().enumerate() {
            let out = pump.flow_at(manifold_pa_converged, inlet_pa, &faults.electric_pump[i], dt);
            pump.commit_displacement(manifold_pa_converged, dt);
            case_drain_drawn += out.case_drain_m3_s;
            pump_reservoir_draw += out.flow_m3_s + out.case_drain_m3_s;
            pump_heat_w += out.shaft_power_w * (1.0 - super::pump::PUMP_MECHANICAL_EFFICIENCY);
            case_drain_heat_w += out.case_drain_m3_s * (manifold_pa_converged - inlet_pa).max(0.0);
            case_drain_flow_m3_s += out.case_drain_m3_s;
            pump_delivered_flow_m3_s += out.flow_m3_s;
            electric_pump_flow[i] = out.flow_m3_s;
        }

        let acc_committed_flow = self.accumulator.exchange_flow_at(manifold_pa_converged, &faults.accumulator, dt);
        let acc_applied_flow = self.accumulator.advance(acc_committed_flow, dt);

        let (return_main_flow, _) = self.network.line_flow_and_dp(line::RETURN_FILTER, density, visc);
        let (return_bypass_flow, _) = self.network.line_flow_and_dp(line::RETURN_BYPASS, density, visc);
        let reservoir_inflow_rate = return_main_flow.max(0.0) + return_bypass_flow.max(0.0);
        let reservoir_before = self.reservoir.fluid_volume_m3();
        let res_out = self.reservoir.step(reservoir_inflow_rate, pump_reservoir_draw, inputs.pressurization_supply_fraction, &faults.reservoir, dt);
        let (lmv_flow, _) = self.network.line_flow_and_dp(line::LMV_TAP, density, visc);
        let conservation = ConservationDiag {
            solver_m3: self.network.last_imbalance_m3,
            relief_lag_m3: (self.relief_valve.flow_m3_s(manifold_pa_converged, faults.relief_valve_crack_low) - relief_flow_lagged) * dt,
            case_drain_lag_m3: (case_drain_drawn - case_drain_injected) * dt,
            check_valve_m3: check_valve_held * dt,
            return_clamp_m3: (return_main_flow.min(0.0) + return_bypass_flow.min(0.0)) * dt,
            reservoir_clamp_m3: reservoir_before + (reservoir_inflow_rate - pump_reservoir_draw - res_out.leaked_m3_s) * dt - res_out.fluid_volume_m3,
            accumulator_clamp_m3: (acc_committed_flow - acc_applied_flow) * dt,
            lmv_tap_m3: lmv_flow * dt,
        };
        self.last_reservoir_inlet_pa = res_out.inlet_air_pressure_pa;

        let mut throttling_heat = 0.0;
        for li in 0..line::COUNT {
            let (flow, dp) = self.network.line_flow_and_dp(li, density, visc);
            throttling_heat += throttling_heat_w(flow, dp);
        }
        throttling_heat += throttling_heat_w(relief_flow_lagged, manifold_pa);

        let reservoir_k_before = self.thermal.temp_k();
        let thermal_out = self.thermal.step(&self.sizing, pump_heat_w + case_drain_heat_w, throttling_heat, (pump_delivered_flow_m3_s + case_drain_flow_m3_s) * density, inputs.fuel_kg_s, inputs.fuel_temp_k, inputs.ambient_k, dt);

        let manifold_out = self.manifold_thermal.step(&self.manifold_sizing, pump_heat_w, throttling_heat, 0.0, 0.0, 0.0, reservoir_k_before, dt);

        CircuitOutputs {
            manifold_pressure_pa: self.network.nodes[node::MANIFOLD].pressure_pa,
            essential_pressure_pa: self.network.nodes[node::ESSENTIAL].pressure_pa,
            accumulator_pressure_pa: self.accumulator.pressure_pa(&faults.accumulator),
            reservoir_fill_fraction: res_out.fill_fraction,
            reservoir_low_level_warning: res_out.low_level_warning,
            reservoir_low_pressure_warning: res_out.low_pressure_warning,
            fluid_temp_c: thermal_out.temp_c,
            fluid_overheat: thermal_out.overheat,
            manifold_temp_c: manifold_out.temp_c,
            manifold_overheat: manifold_out.overheat,
            edp: edp_report,
            electric_pump_flow_m3_s: electric_pump_flow,
            node_pressures_pa: std::array::from_fn(|i| self.network.nodes[i].pressure_pa),
            reservoir_inflow_m3_s: reservoir_inflow_rate,
            reservoir_outflow_m3_s: pump_reservoir_draw,
            network_leaked_m3_s: if dt > 0.0 { leaked_m3 / dt } else { 0.0 },
            reservoir_leaked_m3_s: res_out.leaked_m3_s,
            relief_flow_m3_s: relief_flow_lagged,
            accumulator_flow_m3_s: acc_committed_flow,
            reservoir_inlet_pa: res_out.inlet_air_pressure_pa,
            conservation,
        }
    }

    fn edps_check_valve(&self) -> CheckValve {
        CheckValve { cracking_pa: 5.0 * PSI_PA }
    }
}

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
    fn sustained_high_pump_duty_separates_the_manifold_temperature_from_the_reservoirs() {
        let mut c = Circuit::new(Color::Green);
        let inputs = CircuitInputs { edp: running_edps(4000.0), ambient_k: 288.15, fuel_temp_k: 288.15, pressurization_supply_fraction: 1.0, ..Default::default() };
        let mut out = CircuitOutputs::default();
        for _ in 0..6000 {
            out = c.step(&inputs, &CircuitFaults::default(), 0.02);
        }
        assert!(out.manifold_temp_c.is_finite());
        assert!(out.manifold_temp_c > out.fluid_temp_c, "sustained high pump duty should run the manifold measurably hotter than the reservoir: manifold {} vs reservoir {}", out.manifold_temp_c, out.fluid_temp_c);
    }

    #[test]
    fn at_rest_the_manifold_converges_to_the_same_temperature_as_the_reservoir() {
        let mut c = Circuit::new(Color::Green);
        let inputs = CircuitInputs { edp: running_edps(0.0), ambient_k: 288.15, fuel_temp_k: 288.15, pressurization_supply_fraction: 1.0, ..Default::default() };
        let mut out = CircuitOutputs::default();
        for _ in 0..20000 {
            out = c.step(&inputs, &CircuitFaults::default(), 0.1);
        }
        assert!((out.manifold_temp_c - out.fluid_temp_c).abs() < 1.0, "at rest the two should converge: manifold {} vs reservoir {}", out.manifold_temp_c, out.fluid_temp_c);
        assert!(!out.manifold_overheat);
    }

    #[test]
    fn an_edp_flow_step_never_needs_the_solver_bracket_and_stays_under_the_relief_cap() {
        let mut c = Circuit::new(Color::Green);
        let inputs = CircuitInputs { edp: running_edps(4000.0), ambient_k: 288.15, fuel_temp_k: 288.15, pressurization_supply_fraction: 1.0, ..Default::default() };
        for tick in 0..3000 {
            let out = c.step(&inputs, &CircuitFaults::default(), 0.02);
            assert!(out.manifold_pressure_pa.is_finite(), "tick {tick}: pressure went non-finite");
            assert!(
                out.manifold_pressure_pa < 6000.0 * PSI_PA,
                "tick {tick}: relief should cap this well under the solver's own search ceiling: {:.1} psi",
                out.manifold_pressure_pa / PSI_PA
            );
        }
    }

    #[test]
    fn accumulator_exchange_does_not_chatter_the_manifold_at_zero_edp_flow() {
        let mut c = Circuit::new(Color::Green);
        let inputs_running = CircuitInputs { edp: running_edps(4000.0), ambient_k: 288.15, fuel_temp_k: 288.15, pressurization_supply_fraction: 1.0, ..Default::default() };
        for _ in 0..3000 {
            c.step(&inputs_running, &CircuitFaults::default(), 0.02);
        }
        assert!(c.accumulator.fluid_volume_m3() > 0.0, "accumulator should have taken in fluid while the circuit was pressurised");

        let inputs_stopped = CircuitInputs { edp: running_edps(0.0), ambient_k: 288.15, fuel_temp_k: 288.15, pressurization_supply_fraction: 1.0, ..Default::default() };
        let mut last: Option<f64> = None;
        for tick in 0..500 {
            let out = c.step(&inputs_stopped, &CircuitFaults::default(), 0.02);
            assert!(out.manifold_pressure_pa.is_finite(), "tick {tick}: pressure went non-finite");
            if let Some(prev) = last {
                let step_change = (out.manifold_pressure_pa - prev).abs();
                assert!(
                    step_change < 300.0 * PSI_PA,
                    "tick {tick}: manifold pressure jumped {:.0} psi in one tick ({:.0} -> {:.0} psi), looks like the old chatter",
                    step_change / PSI_PA,
                    prev / PSI_PA,
                    out.manifold_pressure_pa / PSI_PA
                );
            }
            last = Some(out.manifold_pressure_pa);
        }
    }

    fn settle_on_electric_pumps(color: Color, powered: [bool; 2]) -> CircuitOutputs {
        let mut c = Circuit::new(color);
        let inputs = CircuitInputs {
            electric_pump_powered: powered,
            electric_pump_bus_voltage_v: powered.map(|on| if on { 115.0 } else { 0.0 }),
            ambient_k: 288.15,
            fuel_temp_k: 288.15,
            pressurization_supply_fraction: 1.0,
            ..Default::default()
        };
        let mut out = CircuitOutputs::default();
        for _ in 0..3000 {
            out = c.step(&inputs, &CircuitFaults::default(), 0.02);
        }
        out
    }

    #[test]
    fn the_green_circuit_can_be_pressurised_on_its_electric_pumps_with_no_engines() {
        let mut c = Circuit::new(Color::Green);
        let inputs = CircuitInputs {
            electric_pump_powered: [true, true],
            electric_pump_bus_voltage_v: [115.0, 115.0],
            ambient_k: 288.15,
            fuel_temp_k: 288.15,
            pressurization_supply_fraction: 1.0,
            ..Default::default()
        };
        c.step(&inputs, &CircuitFaults::default(), 0.02);
        let mid_flow = c.step(&inputs, &CircuitFaults::default(), 0.02).electric_pump_flow_m3_s;
        assert!(mid_flow.iter().all(|&f| f > 0.0), "both green pumps should be delivering while pressure is still building: {mid_flow:?}");
        let mut out = CircuitOutputs::default();
        for _ in 2..3000 {
            out = c.step(&inputs, &CircuitFaults::default(), 0.02);
        }
        assert!(
            out.manifold_pressure_pa > 500.0 * PSI_PA,
            "green on its own electric pumps should build meaningful pressure: {:.0} psi",
            out.manifold_pressure_pa / PSI_PA
        );
    }

    #[test]
    fn losing_one_ac_bus_leaves_its_circuit_the_other_electric_pump() {
        for color in [Color::Green, Color::Yellow] {
            let both = settle_on_electric_pumps(color, [true, true]);
            let one = settle_on_electric_pumps(color, [true, false]);
            let none = settle_on_electric_pumps(color, [false, false]);

            let mut c = Circuit::new(color);
            let inputs = CircuitInputs {
                electric_pump_powered: [true, false],
                electric_pump_bus_voltage_v: [115.0, 0.0],
                ambient_k: 288.15,
                fuel_temp_k: 288.15,
                pressurization_supply_fraction: 1.0,
                ..Default::default()
            };
            c.step(&inputs, &CircuitFaults::default(), 0.02);
            let early = c.step(&inputs, &CircuitFaults::default(), 0.02);
            assert!(early.electric_pump_flow_m3_s[0] > 0.0, "{color:?}: pump A should still run on its own bus");
            assert_eq!(one.electric_pump_flow_m3_s[1], 0.0, "{color:?}: pump B has no supply");
            assert!(
                one.manifold_pressure_pa > 500.0 * PSI_PA,
                "{color:?}: one pump must still pressurise the circuit, got {:.0} psi",
                one.manifold_pressure_pa / PSI_PA
            );
            const SETTLING_NOISE_PA: f64 = 1000.0;
            assert!(
                one.manifold_pressure_pa <= both.manifold_pressure_pa + SETTLING_NOISE_PA,
                "{color:?}: one pump cannot beat two: {:.0} psi vs {:.0} psi",
                one.manifold_pressure_pa / PSI_PA,
                both.manifold_pressure_pa / PSI_PA
            );
            assert!(
                none.manifold_pressure_pa < 100.0 * PSI_PA,
                "{color:?}: with neither pump powered and no engines there is no source, got {:.0} psi",
                none.manifold_pressure_pa / PSI_PA
            );
        }
    }

    #[test]
    fn yellow_circuit_electric_pump_alone_can_pressurise_the_system() {
        let mut c = Circuit::new(Color::Yellow);
        let inputs = CircuitInputs { electric_pump_powered: [true, true], electric_pump_bus_voltage_v: [115.0, 115.0], ambient_k: 288.15, fuel_temp_k: 288.15, pressurization_supply_fraction: 1.0, ..Default::default() };
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
    fn a_branch_leak_costs_the_reservoir_what_leaked_and_no_more() {
        let inputs = CircuitInputs { edp: running_edps(4000.0), ambient_k: 288.15, fuel_temp_k: 288.15, pressurization_supply_fraction: 1.0, ..Default::default() };
        let leak_faults = CircuitFaults { line_leak_area_m2: [3.0e-6, 0.0, 0.0, 0.0, 0.0], ..Default::default() };
        let mut c = Circuit::new(Color::Green);
        for _ in 0..1500 {
            c.step(&inputs, &leak_faults, 0.02);
        }
        let start = c.reservoir.fluid_volume_m3();
        let mut leaked = 0.0;
        for _ in 0..1500 {
            leaked += c.step(&inputs, &leak_faults, 0.02).network_leaked_m3_s * 0.02;
        }
        let lost = start - c.reservoir.fluid_volume_m3();
        assert!(leaked > 0.0, "setup: the branch must be leaking");
        assert!((lost / leaked - 1.0).abs() < 0.05, "the reservoir must lose what leaked overboard: lost {lost:.6} m3, leaked {leaked:.6} m3");
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

    #[test]
    fn steady_edp_operation_holds_the_reservoir_level() {
        let mut c = Circuit::new(Color::Green);
        let start_fill = c.reservoir.fluid_volume_m3();
        let inputs = CircuitInputs { edp: running_edps(4000.0), ambient_k: 288.15, fuel_temp_k: 288.15, pressurization_supply_fraction: 1.0, ..Default::default() };
        let mut min_fraction = 1.0_f64;
        for _ in 0..3000 {
            let out = c.step(&inputs, &CircuitFaults::default(), 0.02);
            min_fraction = min_fraction.min(out.reservoir_fill_fraction);
        }
        let end_fill = c.reservoir.fluid_volume_m3();
        assert!(
            end_fill > start_fill * 0.9,
            "a steadily running, undemanded circuit must not drain its own reservoir: start {start_fill:.6} m3, end {end_fill:.6} m3"
        );
        assert!(
            min_fraction > 0.9,
            "reservoir fraction must not dip well below full while nothing is consuming flow, got a low of {min_fraction:.4}"
        );
    }
}
