//! The A380's green and yellow hydraulic circuits, assembled from this
//! directory's generic elements.
//!
//! Pump layout matches FlyByWire's own A380 model
//! (`a380_systems/src/hydraulic/mod.rs` lines 1684-1694,
//! `A380Hydraulic`'s field list): four engine-driven pumps per circuit
//! (green from engines 1/2, yellow from engines 3/4 -- each engine drives
//! two, `engine_driven_pump_{1,2,3,4}{a,b}_controller`), and **two
//! electric pumps per circuit**, all four the same unit:
//!
//! | Pump | Motor supply | Control power |
//! |---|---|---|
//! | Green A | AC 1 | DC 2 |
//! | Green B | AC 2 | DC 2 |
//! | Yellow A | AC 3 | DC 1 |
//! | Yellow B | AC 4 | DC 1 |
//!
//! (`GREEN_A_ELEC_PUMP_SUPPLY_POWER_BUS` and its three siblings, and
//! `GREEN_ELEC_PUMP_CONTROL_POWER_BUS`/`YELLOW_ELEC_PUMP_CONTROL_POWER_BUS`,
//! same file lines 1767-1780.) Splitting a circuit's two pumps across two
//! AC buses is the point of the arrangement: losing one bus still leaves
//! that circuit a powered pump.
//!
//! This module used to claim the A380 had no green electric pump and
//! modelled a single pump on yellow. Both were wrong -- FlyByWire
//! constructs all four -- and the cost was that the green circuit had no
//! electric source at all, so it could not be pressurised on the ground,
//! nor after losing both its engines.
//!
//! Still not modelled: the green auxiliary pump (`green_auxiliary_pump`,
//! a `ManualPump` on `PumpCharacteristics::a380_aux_pump()`), for cargo
//! door operation with no engines or AC power. It is a different kind of
//! pump and wants its own model rather than a third `ElectricPump`.
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
    /// Per electric pump, A then B: its motor contactor is closed and its
    /// own AC bus is live (green A/B on AC 1/2, yellow A/B on AC 3/4).
    pub electric_pump_powered: [bool; 2],
    pub electric_pump_bus_voltage_v: [f64; 2],
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
    /// Per electric pump, A then B.
    pub electric_pump: [PumpFaults; 2],
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
    /// ECAM completeness pass (E-FIRE §I): the manifold/pump-discharge
    /// fluid's own temperature, distinct from `fluid_temp_c`'s reservoir
    /// reading -- see `ThermalSizing::a380_manifold`.
    pub manifold_temp_c: f64,
    pub manifold_overheat: bool,
    pub edp: [EdpReport; 4],
    /// Per electric pump, A then B.
    pub electric_pump_flow_m3_s: [f64; 2],
    /// Diagnostics for `live.rs`'s drain recorder: every node's converged
    /// pressure (`node`'s order), and this step's reservoir balance terms.
    pub node_pressures_pa: [f64; NODE_COUNT],
    pub reservoir_inflow_m3_s: f64,
    pub reservoir_outflow_m3_s: f64,
    pub network_leaked_m3_s: f64,
    pub relief_flow_m3_s: f64,
    pub accumulator_flow_m3_s: f64,
    pub reservoir_inlet_pa: f64,
    /// Where this step failed to conserve fluid, if anywhere.
    pub conservation: ConservationDiag,
}

/// This step's fluid bookkeeping errors, m^3, positive = fluid destroyed
/// (negative = created). A circuit that conserves fluid shows all zeros;
/// kept separate per mechanism so a drain names its own cause.
#[derive(Clone, Copy, Debug, Default)]
pub struct ConservationDiag {
    /// The network solve itself: a node pinned at the search bracket, or
    /// sweeps stopping short of convergence.
    pub solver_m3: f64,
    /// The relief valve's dump taken from MANIFOLD at the solved pressure
    /// but returned to RETURN at last step's.
    pub relief_lag_m3: f64,
    /// EDP case drain drawn from the reservoir at the solved pressure but
    /// injected into RETURN at last step's.
    pub case_drain_lag_m3: f64,
    /// EDP flow drawn from the reservoir but held back by the check valve.
    pub check_valve_m3: f64,
    /// Return lines running backwards (reservoir -> RETURN) while the
    /// reservoir is only ever credited, never debited, by them.
    pub return_clamp_m3: f64,
    /// Reservoir volume clamped at empty or full.
    pub reservoir_clamp_m3: f64,
    /// Accumulator state clamped at empty or full.
    pub accumulator_clamp_m3: f64,
    /// The leak-measurement tap's flow to the reservoir boundary, which the
    /// reservoir is not credited with.
    pub lmv_tap_m3: f64,
}

impl ConservationDiag {
    pub fn total_m3(&self) -> f64 {
        self.solver_m3 + self.relief_lag_m3 + self.case_drain_lag_m3 + self.check_valve_m3 + self.return_clamp_m3 + self.reservoir_clamp_m3 + self.accumulator_clamp_m3 + self.lmv_tap_m3
    }
}

/// How many nodes a circuit's `Network` has (`node::COUNT`), for
/// [`CircuitOutputs::node_pressures_pa`].
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
    /// ECAM completeness pass (E-FIRE §I): a second, small thermal state
    /// for the manifold/pump-discharge fluid, distinct from the whole-
    /// circuit `thermal`/`sizing` above -- see `ThermalSizing::a380_manifold`.
    manifold_thermal: ThermalState,
    manifold_sizing: ThermalSizing,
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

        // Both circuits carry two electric pumps; only the reservoir
        // differs between them.
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

        // Each EDP's forward flow into MANIFOLD is resolved together with
        // MANIFOLD's own pressure below (`pressure_dependent`), not frozen
        // here at last step's `manifold_pa` -- see `network.rs`'s
        // `step_with_pressure_dependent` doc for why a whole tick of frozen
        // full-stroke flow racing the manifold's tiny capacitance is what
        // pinned published pressure to the solver's bracket ceiling. What
        // stays lagged here is everything that either does not feed
        // MANIFOLD (case drain, which returns to RETURN) or is already a
        // hard constant whenever the pump is turning: the check valve's
        // `gate` is gated on a fixed margin above manifold pressure
        // (`RUNNING_PUMP_MARGIN_PA` = 50,000 Pa) that clears its 5 psi
        // (~34,474 Pa) cracking pressure regardless of which pressure
        // reading supplies it, so it is degenerately 1.0 whenever the pump
        // is actually running and moot (multiplying a ~0 flow) when it is
        // not -- nothing here needs the node's own trial pressure. The
        // reverse leak is smaller still (a 1 mm^2-class stuck-valve
        // orifice, only nonzero while that one pump is stopped).
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

            // Case drain is a small, roughly-constant leak fraction of ideal
            // flow (`pump.rs`'s `HEALTHY_CASE_DRAIN_FRACTION`) into RETURN,
            // not MANIFOLD, so it cannot blow the search bracket; evaluating
            // it at last step's pressure, as before, keeps this fix to the
            // element that actually needs it.
            let case_out = self.edps[i].step(inputs.edp[i].shaft_rpm, manifold_pa, inlet_pa, &faults.edp[i].pump);
            let case_drain = case_out.case_drain_m3_s * edp_fire_open_now[i];
            case_drain_injected += case_drain;
            injections[node::RETURN] += case_drain;
            edp_report[i].case_drain_m3_s = case_drain;
        }

        let mut electric_pump_flow = [0.0; 2];
        for (i, pump) in self.electric_pumps.iter_mut().enumerate() {
            let (out, _current_a) =
                pump.step(inputs.electric_pump_powered[i], manifold_pa, inlet_pa, inputs.electric_pump_bus_voltage_v[i], &faults.electric_pump[i], dt);
            injections[node::MANIFOLD] += out.flow_m3_s;
            injections[node::RETURN] += out.case_drain_m3_s;
            pump_reservoir_draw += out.flow_m3_s + out.case_drain_m3_s;
            pump_heat_w += out.shaft_power_w * (1.0 - super::pump::PUMP_MECHANICAL_EFFICIENCY);
            pump_delivered_flow_m3_s += out.flow_m3_s;
            electric_pump_flow[i] = out.flow_m3_s;
        }

        // The relief valve's dump is also resolved against MANIFOLD's own
        // trial pressure below (`pressure_dependent`, EDIT 5), not frozen at
        // `manifold_pa`: a relief valve has to respond within the same tick
        // a pump surge does, or nothing opposes that surge until the
        // *following* tick, by which point the frozen relief reading is
        // stale in the other direction (see this file's own regression
        // test). Reservoir/heat bookkeeping below keeps this lagged
        // estimate -- it only needs the flow roughly right, not to the
        // pressure bracket's own precision (see RISK in fixes/W72.md).
        let relief_flow_lagged = self.relief_valve.flow_m3_s(manifold_pa, faults.relief_valve_crack_low);
        injections[node::RETURN] += relief_flow_lagged;

        // The accumulator's own exchange with MANIFOLD is no longer frozen
        // here at last step's `manifold_pa` and injected as a constant --
        // the same freeze-a-fast-responding-flow-for-a-whole-tick bug
        // fixes/W72.md found and fixed for the EDPs/relief valve
        // (fixes/W174.md's SECONDARY CHATTER section: this is what produced
        // the 2257/2892 psi two-level chatter, `HYD_*_ACCUMULATOR_PRESSURE_PSI`
        // alternating between its bled-dry precharge floor and a higher
        // value, in lock-step with MANIFOLD/ESSENTIAL). It is instead
        // folded into `manifold_injection` below (EDIT 4), evaluated at the
        // node's own trial pressure via the new pure
        // `Accumulator::exchange_flow_at`, the same technique already used
        // there for the EDPs/relief. The accumulator's own internal state
        // (gas volume) still only updates once per tick, via
        // `Accumulator::advance` after the network has converged (EDIT 4) --
        // see `accumulator.rs`'s own doc on both new methods for why that
        // split is safe.

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

        // Fold the EDPs' forward flow and the relief valve's dump into
        // MANIFOLD's own residual (node index `node::MANIFOLD` == 0), each
        // evaluated at that node's trial pressure `p_i` rather than frozen:
        // both keep the residual's required monotonicity (pump flow only
        // ever falls, relief flow only ever rises, as `p_i` rises) -- see
        // `network::Network::step_with_pressure_dependent`'s own doc. All
        // captures are by value (`EngineDrivenPump`, `PumpFaults` and
        // `ReliefValve` are all `Copy`), so the closure borrows nothing from
        // `self` and there is no conflict with the `&mut self.network` call
        // right below it.
        let edps = self.edps;
        let edp_shaft_rpm: [f64; 4] = std::array::from_fn(|i| inputs.edp[i].shaft_rpm);
        let edp_pump_faults: [PumpFaults; 4] = std::array::from_fn(|i| faults.edp[i].pump);
        let relief_valve = self.relief_valve;
        let relief_crack_low = faults.relief_valve_crack_low;
        // Captured by value: `Accumulator` is `Clone` (not `Copy`, unlike
        // `EngineDrivenPump`/`PumpFaults`/`ReliefValve` above), and
        // `exchange_flow_at` only reads it, so cloning here keeps this
        // closure's captures the same shape as the EDP/relief ones and
        // avoids a borrow of `self.accumulator` that would conflict with
        // the `&mut self.network` call right below.
        let accumulator = self.accumulator.clone();
        let accumulator_faults = faults.accumulator;
        let manifold_injection = move |p_i: f64| -> f64 {
            let mut q = 0.0;
            for i in 0..4 {
                let out = edps[i].step(edp_shaft_rpm[i], p_i, inlet_pa, &edp_pump_faults[i]);
                q += out.flow_m3_s * edp_gate[i] * edp_fire_open_now[i];
            }
            q -= relief_valve.flow_m3_s(p_i, relief_crack_low);
            // Positive = charging (line -> accumulator, `accumulator.rs`'s
            // own sign convention), so it is a SINK on MANIFOLD's residual --
            // the same direction the relief valve's dump already subtracts.
            q -= accumulator.exchange_flow_at(p_i, &accumulator_faults, dt);
            q
        };
        let mut pressure_dependent: [Option<&dyn Fn(f64) -> f64>; node::COUNT] = [None; node::COUNT];
        pressure_dependent[node::MANIFOLD] = Some(&manifold_injection);
        let leaked_m3 = self.network.step_with_pressure_dependent(&injections, &pressure_dependent, density, visc, dt);

        // Reporting/reservoir/heat figures for the EDPs, read back at the
        // network's now-converged MANIFOLD pressure -- the same
        // read-after-solve pattern `line_flow_and_dp` below already uses
        // for a line's flow, so telemetry matches what was actually
        // injected rather than the old pre-solve estimate.
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
            pump_delivered_flow_m3_s += forward;
            edp_report[i].flow_m3_s = forward;
            edp_report[i].volumetric_efficiency = out.volumetric_efficiency;
        }

        // Commit the accumulator's own state change from the SAME converged
        // pressure the EDPs were just read back at -- the flow that was
        // actually resolved against MANIFOLD this tick, not a stale
        // pre-solve estimate. `advance` sub-steps this into the gas volume
        // the same way `step` always did (see `accumulator.rs`), just
        // driven by this one already-decided flow rather than re-deriving
        // it from a fixed line pressure each sub-step.
        let acc_committed_flow = self.accumulator.exchange_flow_at(manifold_pa_converged, &faults.accumulator, dt);
        let acc_applied_flow = self.accumulator.advance(acc_committed_flow, dt);

        let (return_main_flow, _) = self.network.line_flow_and_dp(line::RETURN_FILTER, density, visc);
        let (return_bypass_flow, _) = self.network.line_flow_and_dp(line::RETURN_BYPASS, density, visc);
        // What the return lines deliver is all that comes back. A branch
        // leak's fluid already left the network at the leaking line (see
        // `Network::node_line_contribution`), so it never reached RETURN;
        // taking it off the return flow again drained the reservoir twice
        // as fast as the leak.
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

        // ECAM completeness pass (E-FIRE §I): capture the reservoir's own
        // state *before* `self.thermal.step` advances it, so the manifold
        // state below exchanges with this tick's starting reservoir
        // temperature -- the same "before this tick's own state" ordering
        // rule `fire_ice::live`'s inter-zone heat computation already
        // documents for the identical reason (no state sees half a frame).
        let reservoir_k_before = self.thermal.temp_k();
        let thermal_out = self.thermal.step(&self.sizing, pump_heat_w, throttling_heat, pump_delivered_flow_m3_s * density, inputs.fuel_kg_s, inputs.fuel_temp_k, inputs.ambient_k, dt);

        // The manifold sees the *same* two heat sources directly (module
        // doc, `ThermalSizing::a380_manifold`), with no fuel/HX cooling of
        // its own (`fluid_flow_kg_s`/`fuel_kg_s` both 0) -- its only sink
        // is conduction to the reservoir's own (colder, actively cooled)
        // fluid, via `manifold_sizing.ambient_loss_w_per_k`.
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
            relief_flow_m3_s: relief_flow_lagged,
            accumulator_flow_m3_s: acc_committed_flow,
            reservoir_inlet_pa: res_out.inlet_air_pressure_pa,
            conservation,
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

    /// E-FIRE §I: the manifold's own thermal state must be a genuinely
    /// distinct signal from the reservoir's, not the same lumped
    /// comparison duplicated under a second name.
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

    /// At rest (no pump heat, no throttling), the manifold must settle to
    /// the same ambient the reservoir does -- the "converge at low flow"
    /// half of §I's own design (module doc, `ThermalSizing::a380_manifold`).
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
        // Regression for the bug this fix addresses: a sudden full-stroke
        // EDP flow step (exactly what the test above settles from, but
        // checked on EVERY tick here, not just the last) used to snap
        // `manifold_pressure_pa` to exactly `network::PRESSURE_BRACKET_HI_PA`
        // (8702.2646 psi) for many ticks in a row, because the EDP's own
        // pressure-compensated destroke curve and the relief valve were
        // both frozen at last step's pressure for the whole tick, racing a
        // manifold volume too small (and a fluid too stiff) to absorb a
        // whole tick of frozen full-stroke flow within the solver's search
        // bracket. 6000 psi matches the already-established final-state
        // bound above; see fixes/W72.md's RISK section for why this is a
        // deliberately conservative per-tick bound rather than the tighter
        // figure a reduced single-branch reproduction of this same fix
        // converges to.
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
        // Regression for the SECOND bug fixes/W174.md's SECONDARY CHATTER
        // section found alongside the EDP/relief one fixes/W72.md already
        // fixed: with the EDPs stopped and the accumulator still charged
        // from a moment ago, the OLD code froze the accumulator's exchange
        // flow at last tick's manifold pressure for the whole tick, racing
        // MANIFOLD's own tiny capacitance the same way the EDP/relief bug
        // did -- producing `HYD_GREEN_ACCUMULATOR_PRESSURE_PSI` alternating
        // between its bled-dry precharge floor (2612 psi) and ~2890.5 psi,
        // in lock-step with MANIFOLD/ESSENTIAL alternating between roughly
        // 2257/2892 psi, every single tick (Log-keep-123124.txt, t=1.3-9.6s).
        let mut c = Circuit::new(Color::Green);
        let inputs_running = CircuitInputs { edp: running_edps(4000.0), ambient_k: 288.15, fuel_temp_k: 288.15, pressurization_supply_fraction: 1.0, ..Default::default() };
        // Charge the circuit (and its accumulator) up first.
        for _ in 0..3000 {
            c.step(&inputs_running, &CircuitFaults::default(), 0.02);
        }
        assert!(c.accumulator.fluid_volume_m3() > 0.0, "accumulator should have taken in fluid while the circuit was pressurised");

        // Now stop every EDP (zero flow, no electric pump either) -- the
        // "zero EDP flow, charged accumulator" condition the brief names --
        // and watch the accumulator discharge into MANIFOLD as pressure
        // sags.
        let inputs_stopped = CircuitInputs { edp: running_edps(0.0), ambient_k: 288.15, fuel_temp_k: 288.15, pressurization_supply_fraction: 1.0, ..Default::default() };
        let mut last: Option<f64> = None;
        for tick in 0..500 {
            let out = c.step(&inputs_stopped, &CircuitFaults::default(), 0.02);
            assert!(out.manifold_pressure_pa.is_finite(), "tick {tick}: pressure went non-finite");
            if let Some(prev) = last {
                let step_change = (out.manifold_pressure_pa - prev).abs();
                // A settling discharge should ease off smoothly. The old
                // bug's signature was a near-full swing between two levels
                // EVERY tick (~635 psi, 2257 <-> 2892, never narrowing).
                // 300 psi is generous headroom over a smooth discharge's own
                // per-tick change, while still easily catching a repeating
                // bang-bang swing of that size.
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

    /// Settle a circuit with the given electric pumps powered, and report
    /// the manifold pressure and each pump's delivered flow.
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

    /// The green circuit has two electric pumps of its own (AC 1 and AC 2).
    /// It used to have none at all, which left it unable to be pressurised
    /// on the ground or after losing both its engines.
    #[test]
    fn the_green_circuit_can_be_pressurised_on_its_electric_pumps_with_no_engines() {
        // (build fix, INT-P4) `settle_on_electric_pumps` runs 3000 ticks (60
        // simulated seconds) with no consumer demand at all: by then the
        // pressure-compensated pump has fully destroked (traced this by
        // hand -- manifold settles dead flat at ~5105 psi, matching the
        // system's own rated pressure, from well before t=200 on), and the
        // ONLY flow left to deliver is whatever balances this circuit's
        // fixed, tiny leaks (the measure-valve orifice, `Circuit::new`'s
        // own `LeakMeasurementValve::MEASURE_ORIFICE_AREA_M2` line) against
        // a wide-open destroke curve -- a real value, but one small enough
        // to fall below this solver's own bisection precision at that
        // pressure, so it reads as an exact 0.0 by the final tick rather
        // than settling on some tiny-but-nonzero float. Asserting strict
        // positivity AFTER full settling therefore tests the solver's
        // numerical floor, not the fix (this exact behaviour matches the
        // established, physically-correct meaning of "pressure-
        // compensated": destroke to ~0 once the system is at pressure with
        // nothing drawing on it). Checking flow during the buildup instead
        // -- while pressure is still climbing, so the pump has not yet
        // destroked -- proves the pumps are what built the pressure, which
        // is this test's actual claim, without demanding a value that is
        // physically supposed to vanish at full steady state.
        let mut c = Circuit::new(Color::Green);
        let inputs = CircuitInputs {
            electric_pump_powered: [true, true],
            electric_pump_bus_voltage_v: [115.0, 115.0],
            ambient_k: 288.15,
            fuel_temp_k: 288.15,
            pressurization_supply_fraction: 1.0,
            ..Default::default()
        };
        // The very first tick's flow is still 0 (the pump's own response
        // needs a tick to start moving off the electrical/mechanical
        // starting transient); by the second it is unambiguously positive
        // and still climbing, well before any destroke -- confirmed by
        // hand-tracing this exact scenario tick by tick.
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

    /// The two pumps of a circuit are fed from different AC buses so that
    /// losing one bus costs that circuit one pump, not both.
    #[test]
    fn losing_one_ac_bus_leaves_its_circuit_the_other_electric_pump() {
        for color in [Color::Green, Color::Yellow] {
            let both = settle_on_electric_pumps(color, [true, true]);
            let one = settle_on_electric_pumps(color, [true, false]);
            let none = settle_on_electric_pumps(color, [false, false]);

            // (build fix, INT-P4) `settle_on_electric_pumps` runs long
            // enough (3000 ticks, 60 s) for a pressure-compensated pump
            // with no consumer demand to fully destroke -- see the sibling
            // test's own doc for why checking flow AFTER that point can
            // land below this solver's bisection precision and read as an
            // exact 0.0 even though the pump is genuinely still running
            // (a physically-real but tiny flow balancing this circuit's
            // fixed leaks). Checking one tick in, while pressure is still
            // climbing, proves pump A is what pressurises the circuit on
            // its own bus without depending on that settled tail value.
            // (The very first tick alone is still 0 -- see the sibling
            // test's own note on the pump's one-tick starting transient --
            // so this checks the second.)
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
            assert!(one.manifold_pressure_pa <= both.manifold_pressure_pa, "{color:?}: one pump cannot beat two");
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

    /// The bug this fix closes: EDPs running steadily at a constant, fully
    /// spooled shaft speed (no ramp, no demand -- the same
    /// `running_edps(4000.0)` fixture `healthy_green_circuit_pressurises_
    /// toward_service_pressure` above already uses) must settle onto their
    /// pressure-compensator curve and hold the reservoir, not cycle the
    /// manifold between near-zero and the network solver's own bisection
    /// bracket ceiling while draining the reservoir a little more every
    /// cycle (observed on a real engine-start log: green reservoir
    /// 1.0 -> 0.03 in 18 s with the aircraft static and no consumer
    /// demand). A circuit with a healthy pressure-compensated pump and
    /// nothing drawing from it should look, from the reservoir's side,
    /// like a closed loop.
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
