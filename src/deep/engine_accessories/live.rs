//! The live engine accessories: one owned fuel-and-control chain per
//! engine, stepped every frame from [`Truth`] and published under the
//! `A32NX_ENG_n_*` names `registry.rs` names in its ECAM triggers.
//!
//! What it owns, per engine, is the whole path fuel takes from the pylon to
//! the burners -- every stage hung off that engine's accessory gearbox:
//!
//! * the LP (boost) centrifugal pump and its inlet strainer;
//! * the fuel filter and its 35 psi bypass valve;
//! * the HP (gear) pump, on the same gearbox shaft as the LP stage;
//! * the fuel metering unit, its metering valve and its spill valve;
//! * the HP fuel shut-off valve;
//! * the dual pick-off fuel flow transmitter;
//! * the burner manifold and its eight nozzle groups;
//! * and the dual-channel EEC that meters against all of it.
//!
//! Four of each, one per Trent 972B-84.
//!
//! ## How the chain is closed
//!
//! The HP pump's delivery depends on the pressure the FMU's spill valve
//! regulates it against, and that differential depends on the flow the HP
//! pump delivers. The loop is closed with the previous frame's value --
//! the same one-frame lag `deep::live`'s own module doc sanctions between
//! areas, and far below the time constant of a fuel system that settles in
//! tenths of a second.
//!
//! ## Scope
//!
//! This area's `registry.rs` also covers ignition, starting, compressor
//! variable geometry, handling bleeds, rotor dynamics, the thrust reverser
//! and the nacelle. Those are not stepped here; see this module's
//! `UNCONSUMED` list, which the tests hold to exactly what is and is not
//! driven so the gap is declared rather than discovered.
//!
//! ## What is not in `Truth` yet
//!
//! `Truth` carries N1 and whether the engine is running, which is enough to
//! know the fan is turning but not enough to drive a fuel system: the pumps
//! are geared to N3, the FMU meters against a commanded fuel flow the
//! governor computes, and the nozzles spray against combustor pressure.
//! Those, the fuel's own inlet temperature and the cockpit's master/fire
//! switches are in [`EngineAccessoryCommands`].

use crate::deep::api::Registry;
use crate::deep::live::{Faults, Truth};

use super::eec::{ActiveChannel, Eec, EecFaults, EecState, SensorFaults, PARAMS};
use super::fuel::filter::{self, FilterFaults, FilterState};
use super::fuel::flow_transmitter::{FlowReading, FlowTransmitter, PickoffFaults};
use super::fuel::fmu::{FmuFaults, FmuState, FuelMeteringUnit};
use super::fuel::hp_pump::{self, HpPumpFaults, HpPumpState};
use super::fuel::lp_pump::{self, LpPumpFaults, LpPumpState};
use super::fuel::manifold::{self, ManifoldFaults, ManifoldState, NUM_GROUPS};
use super::fuel::shutoff_valve::{ShutoffValve, ShutoffValveFaults};

const N_ENGINES: usize = 4;
const N_EEC_PARAMS: usize = 5;

/// Pressure rise the aircraft's own tank boost pumps deliver into the
/// engine LP pump inlet, Pa. **GENERIC**: aircraft fuel boost pumps are
/// quoted in the 5-15 psi class, and 50 kPa (7.3 psi) sits in that band --
/// the same figure `deep::fuel::jettison`'s own
/// `NOMINAL_JETTISON_PUMP_RISE_PA` derives independently for the same class
/// of pump, restated here rather than imported so this area keeps no
/// dependency on another.
const FEED_BOOST_RISE_PA: f64 = 50_000.0;

/// How far the metered flow may sit from the flow the governor commanded
/// before the FADEC calls its own metering faulted, as a fraction of the
/// command. **GENERIC**: a metering unit that cannot hold 10% of a command
/// it has had time to settle on is not controlling the engine.
const METERING_TOLERANCE: f64 = 0.10;
/// The larger shortfall at which the thrust the engine is producing is
/// itself abnormal -- the condition the FADEC FUEL METERING FAULT
/// procedure's own `ENG n MASTER ... OFF` line is conditional on.
/// **GENERIC**, set at a third of the commanded flow: far beyond any
/// transient, and enough of a thrust error for the crew to act on.
const THRUST_ABNORMAL_TOLERANCE: f64 = 0.33;
/// How far the flow transmitter's indication may sit from the flow the FMU
/// says it metered before the two are reported as disagreeing. **GENERIC**,
/// in kg/s, matching `eec::FUEL_FLOW_DISAGREE_THRESHOLD_KG_S`'s own scale
/// so the meter-vs-metering-unit check and the channel-A-vs-B check call a
/// disagreement at the same size of error.
const FF_DISAGREE_KG_S: f64 = 1.0;
/// Nozzle-to-nozzle flow spread (the manifold's own
/// `hot_streak_severity`, a coefficient of variation) at which the FADEC
/// reports a nozzle imbalance. **GENERIC**: 5% spread across eight groups
/// is well outside build tolerance and is where a coked group starts
/// showing as a combustor pattern-factor problem.
const NOZZLE_IMBALANCE_SEVERITY: f64 = 0.05;
/// How far the HP SOV may sit from its commanded position before the
/// disagreement is reported, as a fraction of travel. **GENERIC**: set
/// outside the travel the healthy valve covers in a frame.
const SOV_DISAGREE_TOLERANCE: f64 = 0.05;
/// How far the HP pump's delivery may fall short of what the FMU needs to
/// pass the commanded flow before its own low-flow warning posts, as a
/// fraction. **GENERIC**: the pump is sized with real margin over the
/// metering valve, so 10% of shortfall already means it has lost that
/// margin.
const HP_PUMP_LOW_FLOW_TOLERANCE: f64 = 0.10;

/// `registry.rs` registers ignition (ATA 74), starting (80), compressor
/// variable geometry (72), handling bleeds (75), rotor dynamics (77), the
/// thrust reverser (78) and the nacelle (30/71/26) in this same area. This
/// live system drives the fuel system and the EEC (both ATA 73) only; the
/// ATA chapters listed here are registered but not yet stepped, and the
/// tests hold this list to exactly that set so it cannot quietly drift.
pub const UNCONSUMED_ATA: [u16; 9] = [26, 30, 71, 72, 74, 75, 77, 78, 80];

// ---------------------------------------------------------------------------
// Inputs that `Truth` does not carry yet.
// ---------------------------------------------------------------------------

/// Everything this area needs that is not in [`Truth`].
#[derive(Clone, Copy, Debug)]
pub struct EngineAccessoryCommands {
    /// HP spool speed as a fraction of its design speed, per engine. The
    /// accessory gearbox -- and so both fuel pumps -- is geared to N3, not
    /// to the fan; `Truth` carries only `engine_n1_frac`, and inferring one
    /// spool's speed from another's would be a second engine model.
    pub engine_n3_frac: [f64; N_ENGINES],
    /// IP spool speed, percent, and turbine gas temperature, K -- the EEC
    /// senses both on each channel, so they have to be real values for the
    /// channel-disagree monitor to mean anything.
    pub engine_n2_percent: [f64; N_ENGINES],
    pub engine_tgt_k: [f64; N_ENGINES],
    /// Compressor delivery (P30) pressure, Pa: what the burner nozzles
    /// spray against, and the EEC's fifth sensed parameter.
    pub engine_p30_pa: [f64; N_ENGINES],
    /// The fuel flow the engine's governor is commanding, kg/s. This is the
    /// output of the control law, not of this area's hardware; the FMU's
    /// job is to deliver it.
    pub wf_command_kg_s: [f64; N_ENGINES],
    /// Fuel temperature at the engine's LP pump inlet, K -- the feed tank's
    /// own bulk temperature, which `deep::fuel::live` computes but `Truth`
    /// does not yet carry. `None` falls back to ambient static air
    /// temperature, which is where a parked, cold-soaked aircraft's feed
    /// fuel genuinely sits.
    pub fuel_inlet_k: Option<f64>,
    /// The engine master switch and the fire handle, which between them
    /// command the HP shut-off valve.
    pub master_on: [bool; N_ENGINES],
    pub fire_handle_pulled: [bool; N_ENGINES],
}

impl Default for EngineAccessoryCommands {
    /// Four cold engines: nothing turning, nothing commanded, masters off.
    fn default() -> Self {
        Self {
            engine_n3_frac: [0.0; N_ENGINES],
            engine_n2_percent: [0.0; N_ENGINES],
            engine_tgt_k: [288.15; N_ENGINES],
            engine_p30_pa: [101_325.0; N_ENGINES],
            wf_command_kg_s: [0.0; N_ENGINES],
            fuel_inlet_k: None,
            master_on: [false; N_ENGINES],
            fire_handle_pulled: [false; N_ENGINES],
        }
    }
}

// ---------------------------------------------------------------------------
// Failure ids.
// ---------------------------------------------------------------------------

/// The ATA-73 failure ids this live system consumes, per engine, resolved
/// once at construction from the registry itself.
struct EngineIds {
    lp_wear: u64,
    lp_inlet_restriction: u64,
    filter_clog: u64,
    hp_wear: u64,
    hp_starvation: u64,
    fmu_sticking: u64,
    fmu_spill_open: u64,
    fmu_spill_closed: u64,
    sov_stuck: u64,
    ft_a_bias: u64,
    ft_a_frozen: u64,
    ft_b_bias: u64,
    ft_b_frozen: u64,
    nozzle: [u64; NUM_GROUPS],
    eec_channel_a: u64,
    eec_channel_b: u64,
    eec_sensor_a: [u64; N_EEC_PARAMS],
    eec_sensor_b: [u64; N_EEC_PARAMS],
}

/// The one failure on `component` whose registered `model_field` contains
/// `fragment`. A live system that cannot find the failure it is meant to
/// consume is a build error, not something to tolerate at runtime.
fn fid(reg: &Registry, component: &str, fragment: &str) -> u64 {
    let mut found = reg.failures.iter().filter(|f| f.component == component && f.model_field.contains(fragment));
    let first = found.next().unwrap_or_else(|| panic!("no failure on {component} whose model_field contains {fragment:?}"));
    assert!(found.next().is_none(), "more than one failure on {component} matches {fragment:?}");
    first.id
}

impl EngineIds {
    fn resolve(reg: &Registry, eng: usize) -> Self {
        let n = eng + 1;
        let lp = format!("73_fuel.lp_pump_{n}");
        let filter_id = format!("73_fuel.filter_{n}");
        let hp = format!("73_fuel.hp_pump_{n}");
        let fmu = format!("73_fuel.fmu_{n}");
        let sov = format!("73_fuel.hp_sov_{n}");
        let ft = format!("73_fuel.flow_transmitter_{n}");
        let man = format!("73_fuel.manifold_{n}");
        let eec = format!("73_eec.channels_{n}");

        let mut nozzle = [0u64; NUM_GROUPS];
        for (g, slot) in nozzle.iter_mut().enumerate() {
            *slot = fid(reg, &man, &format!("group_blockage[{g}]"));
        }
        let mut eec_sensor_a = [0u64; N_EEC_PARAMS];
        let mut eec_sensor_b = [0u64; N_EEC_PARAMS];
        for (i, p) in ["N1", "N2", "N3", "TGT", "P30"].iter().enumerate() {
            eec_sensor_a[i] = fid(reg, &eec, &format!("sensor_a[{p}]"));
            eec_sensor_b[i] = fid(reg, &eec, &format!("sensor_b[{p}]"));
        }

        Self {
            lp_wear: fid(reg, &lp, "LpPumpFaults.wear"),
            lp_inlet_restriction: fid(reg, &lp, "inlet_restriction"),
            filter_clog: fid(reg, &filter_id, "FilterFaults.clog"),
            hp_wear: fid(reg, &hp, "HpPumpFaults.wear"),
            hp_starvation: fid(reg, &hp, "inlet_starvation"),
            fmu_sticking: fid(reg, &fmu, "valve_sticking"),
            fmu_spill_open: fid(reg, &fmu, "spill_stuck_open"),
            fmu_spill_closed: fid(reg, &fmu, "spill_stuck_closed"),
            sov_stuck: fid(reg, &sov, "ShutoffValveFaults.stuck"),
            ft_a_bias: fid(reg, &ft, "bias_frac_of_design (channel A)"),
            ft_a_frozen: fid(reg, &ft, "frozen (channel A)"),
            ft_b_bias: fid(reg, &ft, "bias_frac_of_design (channel B)"),
            ft_b_frozen: fid(reg, &ft, "frozen (channel B)"),
            nozzle,
            eec_channel_a: fid(reg, &eec, "channel_a_fault"),
            eec_channel_b: fid(reg, &eec, "channel_b_fault"),
            eec_sensor_a,
            eec_sensor_b,
        }
    }
}

// ---------------------------------------------------------------------------
// One engine's fuel-and-control chain.
// ---------------------------------------------------------------------------

/// One engine's accessories: the whole fuel path plus its EEC.
struct EngineChain {
    ids: EngineIds,
    fmu: FuelMeteringUnit,
    sov: ShutoffValve,
    transmitter: FlowTransmitter,
    eec: Eec,

    // This frame's state, for publishing.
    lp: LpPumpState,
    filter: FilterState,
    hp: HpPumpState,
    fmu_state: FmuState,
    sov_position: f64,
    flow: FlowReading,
    manifold: ManifoldState,
    eec_state: EecState,
    /// Fuel actually reaching the manifold, kg/s: what the FMU metered,
    /// gated by the shut-off valve.
    delivered_kg_s: f64,
}

impl EngineChain {
    fn new(reg: &Registry, eng: usize) -> Self {
        Self {
            ids: EngineIds::resolve(reg, eng),
            fmu: FuelMeteringUnit::new(),
            // A cold engine's HP SOV is shut.
            sov: ShutoffValve::new(false),
            transmitter: FlowTransmitter::new(),
            eec: Eec::new(),
            lp: LpPumpState::default(),
            filter: FilterState::default(),
            hp: HpPumpState::default(),
            fmu_state: FmuState::default(),
            sov_position: 0.0,
            flow: FlowReading::default(),
            manifold: manifold::step(0.0, 101_325.0, &ManifoldFaults::default()),
            eec_state: EecState { selected: [0.0; N_EEC_PARAMS], disagree: [false; N_EEC_PARAMS], active: ActiveChannel::A, fuel_flow_disagree: false },
            delivered_kg_s: 0.0,
        }
    }

    /// One engine's chain, from the pylon to the burners.
    #[allow(clippy::too_many_arguments)]
    fn step(&mut self, eng: usize, truth: &Truth, faults: &Faults, commands: &EngineAccessoryCommands, dt: f64) {
        let n3 = commands.engine_n3_frac[eng].max(0.0);
        let fuel_k = commands.fuel_inlet_k.unwrap_or(truth.environment.sat_c + 273.15).max(1.0);
        let inlet_pa = truth.environment.ambient_pressure_pa.max(0.0) + FEED_BOOST_RISE_PA;
        let wf_command = commands.wf_command_kg_s[eng].max(0.0);

        // ---- HP pump: works against the differential the FMU's spill
        // valve regulated last frame (the loop is closed with a one-frame
        // lag; see the module doc).
        let starvation_from_lp = if self.hp.delivered_m3_s > 1e-9 { (1.0 - self.lp.flow_m3_s / self.hp.delivered_m3_s).clamp(0.0, 1.0) } else { 0.0 };
        let hp_faults = HpPumpFaults {
            wear: faults.get(self.ids.hp_wear),
            // `registry.rs`: inlet starvation is "fed forward from LP pump
            // cavitation", so the armed failure and the LP stage's own
            // inability to keep up are the same physical input, whichever
            // is worse.
            inlet_starvation: faults.get(self.ids.hp_starvation).max(starvation_from_lp),
        };
        self.hp = hp_pump::step(n3, self.fmu_state.differential_pa, &hp_faults);

        // ---- LP pump and filter feed it.
        let lp_faults = LpPumpFaults { wear: faults.get(self.ids.lp_wear), inlet_restriction: faults.get(self.ids.lp_inlet_restriction) };
        self.lp = lp_pump::step(n3, inlet_pa, fuel_k, self.hp.delivered_m3_s, &lp_faults);
        let filter_faults = FilterFaults { clog: faults.get(self.ids.filter_clog) };
        self.filter = filter::step(self.lp.outlet_pa, self.hp.delivered_m3_s, fuel_k, &filter_faults);

        // ---- FMU: meters the commanded flow out of what the HP pump
        // delivers at the pressure it delivers it.
        let fmu_faults = FmuFaults {
            valve_sticking: faults.get(self.ids.fmu_sticking),
            spill_stuck_open: faults.get(self.ids.fmu_spill_open),
            spill_stuck_closed: faults.get(self.ids.fmu_spill_closed),
        };
        let hp_supply_pa = self.filter.outlet_pa + self.fmu_state.differential_pa;
        self.fmu_state = self.fmu.step(wf_command, hp_supply_pa, self.hp.delivered_m3_s, &fmu_faults, dt);

        // ---- HP shut-off valve: open on the master switch unless the fire
        // handle has been pulled.
        let sov_faults = ShutoffValveFaults { stuck: faults.get(self.ids.sov_stuck) };
        let sov_commanded_open = commands.master_on[eng] && !commands.fire_handle_pulled[eng];
        self.sov_position = self.sov.step(sov_commanded_open, &sov_faults, dt);
        self.delivered_kg_s = self.fmu_state.metered_kg_s * self.sov_position;

        // ---- Flow transmitter, then the manifold it feeds.
        let a = PickoffFaults { bias_frac_of_design: faults.get(self.ids.ft_a_bias), frozen: faults.get(self.ids.ft_a_frozen) };
        let b = PickoffFaults { bias_frac_of_design: faults.get(self.ids.ft_b_bias), frozen: faults.get(self.ids.ft_b_frozen) };
        self.flow = self.transmitter.step(self.delivered_kg_s, &a, &b, dt);

        let manifold_faults = ManifoldFaults { group_blockage: std::array::from_fn(|g| faults.get(self.ids.nozzle[g])) };
        self.manifold = manifold::step(self.delivered_kg_s, commands.engine_p30_pa[eng], &manifold_faults);

        // ---- EEC.
        // One registered failure per (channel, parameter) covers both the
        // bias and the frozen mode of that sensor chain
        // (`registry.rs`: "0 healthy .. 1 max bias or fully frozen"), so
        // the magnitude drives both: a degrading pick-off drifts *and*
        // loses responsiveness together, reaching fully biased and fully
        // frozen -- a dead chain -- at 1.0.
        let sensor = |id: u64| {
            let m = faults.get(id);
            SensorFaults { bias: m, frozen: m }
        };
        let eec_faults = EecFaults {
            channel_a_fault: faults.get(self.ids.eec_channel_a),
            channel_b_fault: faults.get(self.ids.eec_channel_b),
            sensor_a: std::array::from_fn(|i| sensor(self.ids.eec_sensor_a[i])),
            sensor_b: std::array::from_fn(|i| sensor(self.ids.eec_sensor_b[i])),
        };
        let true_values = [
            truth.engine_n1_frac[eng] * 100.0,
            commands.engine_n2_percent[eng],
            n3 * 100.0,
            commands.engine_tgt_k[eng],
            commands.engine_p30_pa[eng],
        ];
        self.eec_state = self.eec.step(true_values, self.flow.channel_a_kg_s, self.flow.channel_b_kg_s, &eec_faults, dt);
    }

    /// The flow the FMU needs the HP pump to deliver to pass the commanded
    /// flow, m^3/s -- what the pump's own low-flow monitor compares against.
    fn required_pump_flow_m3_s(wf_command_kg_s: f64) -> f64 {
        wf_command_kg_s.max(0.0) / super::fuel::common::FUEL_DENSITY_KG_M3
    }

    fn hp_pump_low_flow(&self, wf_command_kg_s: f64) -> bool {
        let required = Self::required_pump_flow_m3_s(wf_command_kg_s);
        required > 0.0 && self.hp.delivered_m3_s < required * (1.0 - HP_PUMP_LOW_FLOW_TOLERANCE)
    }

    fn metering_error_fraction(&self, wf_command_kg_s: f64) -> f64 {
        if wf_command_kg_s <= 0.0 {
            return 0.0;
        }
        (self.fmu_state.metered_kg_s - wf_command_kg_s).abs() / wf_command_kg_s
    }
}

// ---------------------------------------------------------------------------
// The live system.
// ---------------------------------------------------------------------------

/// The live engine accessories: four engines' fuel chains and EECs.
pub struct EngineAccessoriesLive {
    engines: Vec<EngineChain>,
    /// Inputs `Truth` does not carry; see [`EngineAccessoryCommands`].
    pub commands: EngineAccessoryCommands,
}

impl Default for EngineAccessoriesLive {
    fn default() -> Self {
        Self::new()
    }
}

impl EngineAccessoriesLive {
    pub fn new() -> Self {
        let mut reg = Registry::default();
        super::registry::register(&mut reg);
        Self { engines: (0..N_ENGINES).map(|e| EngineChain::new(&reg, e)).collect(), commands: EngineAccessoryCommands::default() }
    }

    /// The fuel actually reaching engine `eng`'s burners, kg/s -- the one
    /// number `deep::fuel`'s own tanks need back from this area once
    /// `Truth` can carry it between them.
    pub fn delivered_fuel_kg_s(&self, eng: usize) -> f64 {
        self.engines[eng].delivered_kg_s
    }
}

impl crate::deep::live::Area for EngineAccessoriesLive {
    fn name(&self) -> &'static str {
        "engine_accessories"
    }

    fn tick(&mut self, truth: &Truth, faults: &Faults) {
        let dt = truth.dt_s.max(0.0);
        let commands = self.commands;
        for (eng, chain) in self.engines.iter_mut().enumerate() {
            chain.step(eng, truth, faults, &commands, dt);
        }
    }

    fn publish(&self, out: &mut dyn FnMut(&str, f64)) {
        let b = |x: bool| if x { 1.0 } else { 0.0 };

        for (eng, chain) in self.engines.iter().enumerate() {
            let n = eng + 1;
            let wf_command = self.commands.wf_command_kg_s[eng];

            // ---- Filter.
            out(&format!("A32NX_ENG_{n}_FUEL_FILTER_IMPENDING_BYPASS"), b(chain.filter.impending_bypass));
            out(&format!("A32NX_ENG_{n}_FUEL_FILTER_BYPASSED"), b(chain.filter.bypassed));
            out(&format!("A32NX_ENG_{n}_FUEL_FILTER_DP_PA"), chain.filter.differential_pa);

            // ---- Pumps.
            out(&format!("A32NX_ENG_{n}_HP_PUMP_LOW_FLOW"), b(chain.hp_pump_low_flow(wf_command)));
            out(&format!("A32NX_ENG_{n}_HP_PUMP_FLOW_KG_S"), chain.hp.delivered_kg_s);
            out(&format!("A32NX_ENG_{n}_LP_PUMP_OUTLET_PA"), chain.lp.outlet_pa);
            out(&format!("A32NX_ENG_{n}_LP_PUMP_CAVITATING"), b(chain.lp.cavitating));

            // ---- Metering.
            let error = chain.metering_error_fraction(wf_command);
            out(&format!("A32NX_ENG_{n}_FMU_FAULT"), b(error > METERING_TOLERANCE));
            out(&format!("A32NX_ENG_{n}_THRUST_ABNORMAL"), b(error > THRUST_ABNORMAL_TOLERANCE));
            out(&format!("A32NX_ENG_{n}_FMU_METERED_KG_S"), chain.fmu_state.metered_kg_s);
            out(&format!("A32NX_ENG_{n}_FMU_DP_PA"), chain.fmu_state.differential_pa);

            // ---- HP shut-off valve.
            let sov_target = if self.commands.master_on[eng] && !self.commands.fire_handle_pulled[eng] { 1.0 } else { 0.0 };
            out(&format!("A32NX_ENG_{n}_HP_SOV_DISAGREE"), b((chain.sov_position - sov_target).abs() > SOV_DISAGREE_TOLERANCE));
            out(&format!("A32NX_ENG_{n}_HP_SOV_POSITION"), chain.sov_position);

            // ---- Flow transmitter: channel against channel, and the
            // meter as a whole against what the FMU says it metered.
            out(&format!("A32NX_ENG_{n}_FF_CHANNEL_DISAGREE"), b(chain.eec_state.fuel_flow_disagree));
            // What the crew's fuel-flow indication reads: the mean of the
            // transmitter's two pick-off channels.
            let indicated = 0.5 * (chain.flow.channel_a_kg_s + chain.flow.channel_b_kg_s);
            out(&format!("A32NX_ENG_{n}_FF_DISAGREE"), b((indicated - chain.delivered_kg_s).abs() > FF_DISAGREE_KG_S));
            out(&format!("A32NX_ENG_{n}_FF_INDICATED_KG_S"), indicated);
            out(&format!("A32NX_ENG_{n}_FF_TRUE_KG_S"), chain.flow.true_flow_kg_s);

            // ---- Burner manifold.
            out(&format!("A32NX_ENG_{n}_NOZZLE_IMBALANCE"), b(chain.manifold.hot_streak_severity > NOZZLE_IMBALANCE_SEVERITY));
            out(&format!("A32NX_ENG_{n}_NOZZLE_HOT_STREAK"), chain.manifold.hot_streak_severity);
            out(&format!("A32NX_ENG_{n}_MANIFOLD_GAUGE_PA"), chain.manifold.manifold_gauge_pa);

            // ---- EEC.
            let channel_faulted = chain.eec_state.active != ActiveChannel::A;
            out(&format!("A32NX_ENG_{n}_EEC_CHANNEL_FAULT"), b(channel_faulted));
            out(&format!("A32NX_ENG_{n}_EEC_NO_VALID_CHANNEL"), b(chain.eec_state.active == ActiveChannel::None));
            out(&format!("A32NX_ENG_{n}_EEC_SENSOR_DISAGREE"), b(chain.eec_state.disagree.iter().any(|&d| d)));
            for (i, p) in PARAMS.iter().enumerate() {
                let key = match p {
                    super::eec::Param::N1 => "N1",
                    super::eec::Param::N2 => "N2",
                    super::eec::Param::N3 => "N3",
                    super::eec::Param::Tgt => "TGT",
                    super::eec::Param::P30 => "P30",
                };
                out(&format!("A32NX_ENG_{n}_EEC_{key}_SELECTED"), chain.eec_state.selected[i]);
                out(&format!("A32NX_ENG_{n}_EEC_{key}_DISAGREE"), b(chain.eec_state.disagree[i]));
            }
        }
    }
}

/// This area's live system.
pub fn live_system() -> Box<dyn crate::deep::live::Area> {
    Box::new(EngineAccessoriesLive::new())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::deep::fuel::live::test_support::collect_vars;
    use crate::deep::live::Area as _;
    use std::collections::BTreeMap;

    /// The fuel flow a Trent 972B-84 burns per engine at the gas path's own
    /// SLS design point (`physics::engine::gas_path`, cited throughout this
    /// area's own modules).
    const DESIGN_WF_KG_S: f64 = 2.48;

    fn published(area: &dyn crate::deep::live::Area) -> BTreeMap<String, f64> {
        let mut map = BTreeMap::new();
        area.publish(&mut |name, value| {
            map.insert(name.to_string(), value);
        });
        map
    }

    /// Four engines running at cruise power, fuel warm, masters on.
    fn running() -> (Truth, EngineAccessoryCommands) {
        let truth = Truth {
            dt_s: 0.02,
            engine_running: [true; 4],
            engine_n1_frac: [0.85; 4],
            ac_bus_volts: [115.0; 4],
            ..Truth::default()
        };
        let commands = EngineAccessoryCommands {
            engine_n3_frac: [0.9; 4],
            engine_n2_percent: [88.0; 4],
            engine_tgt_k: [900.0; 4],
            engine_p30_pa: [2.5e6; 4],
            wf_command_kg_s: [DESIGN_WF_KG_S; 4],
            fuel_inlet_k: Some(300.0),
            master_on: [true; 4],
            fire_handle_pulled: [false; 4],
        };
        (truth, commands)
    }

    fn run(live: &mut EngineAccessoriesLive, truth: &Truth, faults: &Faults, seconds: f64) -> BTreeMap<String, f64> {
        let steps = (seconds / truth.dt_s).ceil() as usize;
        for _ in 0..steps.max(1) {
            live.tick(truth, faults);
        }
        published(live)
    }

    fn settled(faults: &Faults) -> BTreeMap<String, f64> {
        let (truth, commands) = running();
        let mut live = EngineAccessoriesLive::new();
        live.commands = commands;
        run(&mut live, &truth, faults, 30.0)
    }

    #[test]
    fn four_healthy_engines_meter_what_the_governor_asks_and_raise_nothing() {
        let out = settled(&Faults::default());
        for n in 1..=4 {
            assert!(
                (out[&format!("A32NX_ENG_{n}_FMU_METERED_KG_S")] - DESIGN_WF_KG_S).abs() < 0.05,
                "engine {n} metered {} against a {DESIGN_WF_KG_S} command",
                out[&format!("A32NX_ENG_{n}_FMU_METERED_KG_S")]
            );
            for v in ["FMU_FAULT", "THRUST_ABNORMAL", "HP_PUMP_LOW_FLOW", "FUEL_FILTER_BYPASSED", "FUEL_FILTER_IMPENDING_BYPASS", "HP_SOV_DISAGREE", "FF_CHANNEL_DISAGREE", "FF_DISAGREE", "NOZZLE_IMBALANCE", "EEC_CHANNEL_FAULT", "EEC_NO_VALID_CHANNEL", "EEC_SENSOR_DISAGREE"] {
                assert_eq!(out.get(&format!("A32NX_ENG_{n}_{v}")), Some(&0.0), "engine {n} {v} should be healthy");
            }
        }
    }

    /// The ATA-73 alerts are the ones this live system owns; every
    /// variable they trigger on must be published by it.
    #[test]
    fn every_variable_this_areas_ata_73_alerts_trigger_on_is_published() {
        let mut reg = Registry::default();
        super::super::registry::register(&mut reg);
        let mut names = Vec::new();
        for alert in reg.alerts.iter().filter(|a| a.ata == 73) {
            collect_vars(&alert.trigger, &mut names);
            // Procedure lines' own conditions too: a line conditional on a
            // variable nobody publishes can never apply.
            for line in &alert.procedure {
                collect_vars(&line.applies_if, &mut names);
            }
        }
        let out = settled(&Faults::default());
        for name in names {
            // Cockpit controls and the fire handle belong to the plugin,
            // not to this area.
            if !name.starts_with("A32NX_ENG_") {
                continue;
            }
            assert!(out.contains_key(&name), "an ATA 73 alert reads {name}, which this live system does not publish");
        }
    }

    #[test]
    fn a_clogging_filter_warns_before_it_bypasses_and_then_bypasses() {
        // registry.rs: "differential pressure rises until the 35 psi bypass
        // valve cracks; beyond that, unfiltered fuel reaches the HP pump".
        let live = EngineAccessoriesLive::new();
        let id = live.engines[0].ids.filter_clog;

        let clean = settled(&Faults::default());
        assert_eq!(clean.get("A32NX_ENG_1_FUEL_FILTER_IMPENDING_BYPASS"), Some(&0.0));

        let mut worst_dp = 0.0;
        let mut impending_at = None;
        let mut bypass_at = None;
        for step in 0..=20 {
            let clog = step as f64 / 20.0;
            let out = settled(&Faults::from_pairs([(id, clog)]));
            let dp = out["A32NX_ENG_1_FUEL_FILTER_DP_PA"];
            assert!(dp >= worst_dp - 1e-9, "differential must rise monotonically with clog");
            worst_dp = dp;
            if impending_at.is_none() && out["A32NX_ENG_1_FUEL_FILTER_IMPENDING_BYPASS"] > 0.0 {
                impending_at = Some(clog);
            }
            if bypass_at.is_none() && out["A32NX_ENG_1_FUEL_FILTER_BYPASSED"] > 0.0 {
                bypass_at = Some(clog);
            }
        }
        let impending = impending_at.expect("a filter clogging toward fully blocked must warn");
        let bypass = bypass_at.expect("and must eventually bypass");
        assert!(impending < bypass, "the warning must come before the bypass: {impending} vs {bypass}");
    }

    #[test]
    fn an_fmu_spill_valve_stuck_open_starves_the_burners_and_reports_abnormal_thrust() {
        // registry.rs: "the regulated differential collapses toward zero;
        // metered flow starves even at full valve area (flameout risk)".
        let live = EngineAccessoriesLive::new();
        let id = live.engines[1].ids.fmu_spill_open;
        let out = settled(&Faults::from_pairs([(id, 1.0)]));
        assert!(out["A32NX_ENG_2_FMU_METERED_KG_S"] < 0.1 * DESIGN_WF_KG_S, "a collapsed differential must starve the burners");
        assert_eq!(out.get("A32NX_ENG_2_FMU_FAULT"), Some(&1.0));
        assert_eq!(out.get("A32NX_ENG_2_THRUST_ABNORMAL"), Some(&1.0));
        assert_eq!(out.get("A32NX_ENG_1_FMU_FAULT"), Some(&0.0), "the other engines are untouched");
    }

    #[test]
    fn an_fmu_metering_valve_stuck_shut_cannot_follow_a_command_at_all() {
        // registry.rs: "the valve tracks the commanded flow ever more
        // slowly and, fully stuck, freezes in place" -- and a cold engine's
        // valve starts shut.
        let live = EngineAccessoriesLive::new();
        let id = live.engines[0].ids.fmu_sticking;
        let out = settled(&Faults::from_pairs([(id, 1.0)]));
        assert!(out["A32NX_ENG_1_FMU_METERED_KG_S"] < 1e-6, "a fully seized valve cannot open");
        assert_eq!(out.get("A32NX_ENG_1_FMU_FAULT"), Some(&1.0));
    }

    #[test]
    fn a_worn_hp_pump_slips_and_eventually_cannot_supply_the_commanded_flow() {
        // registry.rs: "internal slip flow grows as 1/(1-wear)^2; delivered
        // flow falls short of the theoretical displacement flow".
        let live = EngineAccessoriesLive::new();
        let id = live.engines[2].ids.hp_wear;
        let healthy = settled(&Faults::default());
        let worn = settled(&Faults::from_pairs([(id, 0.9)]));
        assert!(
            worn["A32NX_ENG_3_HP_PUMP_FLOW_KG_S"] < healthy["A32NX_ENG_3_HP_PUMP_FLOW_KG_S"],
            "wear must cost delivered flow: {} vs {}",
            worn["A32NX_ENG_3_HP_PUMP_FLOW_KG_S"],
            healthy["A32NX_ENG_3_HP_PUMP_FLOW_KG_S"]
        );
        let starved = settled(&Faults::from_pairs([(live.engines[2].ids.hp_starvation, 1.0)]));
        assert_eq!(starved.get("A32NX_ENG_3_HP_PUMP_LOW_FLOW"), Some(&1.0), "a totally starved pump must post its low-flow warning");
        assert_eq!(starved.get("A32NX_ENG_4_HP_PUMP_LOW_FLOW"), Some(&0.0));
    }

    #[test]
    fn a_stuck_hp_shutoff_valve_cannot_follow_the_master_switch() {
        // registry.rs: "stuck open defeats a fire-handle shutdown, stuck
        // closed flames the engine out".
        let (truth, commands) = running();
        let mut live = EngineAccessoriesLive::new();
        live.commands = commands;
        let id = live.engines[0].ids.sov_stuck;

        // Healthy: the valve follows the master switch open, then follows
        // the fire handle shut.
        let open = run(&mut live, &truth, &Faults::default(), 10.0);
        assert!(open["A32NX_ENG_1_HP_SOV_POSITION"] > 0.95);
        assert_eq!(open.get("A32NX_ENG_1_HP_SOV_DISAGREE"), Some(&0.0));
        live.commands.fire_handle_pulled[0] = true;
        let shut = run(&mut live, &truth, &Faults::default(), 10.0);
        assert!(shut["A32NX_ENG_1_HP_SOV_POSITION"] < 0.05);

        // Seized open: the same fire handle now leaves it open and the
        // disagreement is reported.
        let mut live = EngineAccessoriesLive::new();
        live.commands = commands;
        run(&mut live, &truth, &Faults::default(), 10.0);
        live.commands.fire_handle_pulled[0] = true;
        let seized = run(&mut live, &truth, &Faults::from_pairs([(id, 1.0)]), 10.0);
        assert!(seized["A32NX_ENG_1_HP_SOV_POSITION"] > 0.95, "a seized valve does not move");
        assert_eq!(seized.get("A32NX_ENG_1_HP_SOV_DISAGREE"), Some(&1.0));
    }

    #[test]
    fn a_biased_flow_transmitter_channel_makes_the_two_channels_disagree() {
        let live = EngineAccessoriesLive::new();
        let id = live.engines[3].ids.ft_a_bias;
        let out = settled(&Faults::from_pairs([(id, 1.0)]));
        assert_eq!(out.get("A32NX_ENG_4_FF_CHANNEL_DISAGREE"), Some(&1.0));
        assert_eq!(out.get("A32NX_ENG_3_FF_CHANNEL_DISAGREE"), Some(&0.0));
        assert!(out["A32NX_ENG_4_FF_INDICATED_KG_S"] > out["A32NX_ENG_3_FF_INDICATED_KG_S"], "a positive bias must read high");
    }

    #[test]
    fn a_coked_nozzle_group_skews_the_manifold_and_raises_a_hot_streak() {
        // registry.rs: "shrinks that group's orifice area; the shared
        // manifold pressure rises until the other groups pass the
        // difference, raising hot_streak_severity".
        let live = EngineAccessoriesLive::new();
        let id = live.engines[0].ids.nozzle[3];
        let clean = settled(&Faults::default());
        let coked = settled(&Faults::from_pairs([(id, 0.8)]));
        assert_eq!(clean.get("A32NX_ENG_1_NOZZLE_IMBALANCE"), Some(&0.0));
        assert_eq!(coked.get("A32NX_ENG_1_NOZZLE_IMBALANCE"), Some(&1.0));
        assert!(coked["A32NX_ENG_1_NOZZLE_HOT_STREAK"] > clean["A32NX_ENG_1_NOZZLE_HOT_STREAK"]);
        assert!(coked["A32NX_ENG_1_MANIFOLD_GAUGE_PA"] > clean["A32NX_ENG_1_MANIFOLD_GAUGE_PA"], "the surviving groups must pass the difference at a higher pressure");
    }

    #[test]
    fn one_dead_eec_channel_hands_over_and_two_leave_no_valid_channel() {
        // registry.rs: "hands control to channel B if it is healthy; both
        // dead leaves no valid EEC channel".
        let live = EngineAccessoriesLive::new();
        let (a, b) = (live.engines[1].ids.eec_channel_a, live.engines[1].ids.eec_channel_b);

        let one = settled(&Faults::from_pairs([(a, 1.0)]));
        assert_eq!(one.get("A32NX_ENG_2_EEC_CHANNEL_FAULT"), Some(&1.0));
        assert_eq!(one.get("A32NX_ENG_2_EEC_NO_VALID_CHANNEL"), Some(&0.0), "channel B is still flying the engine");

        let both = settled(&Faults::from_pairs([(a, 1.0), (b, 1.0)]));
        assert_eq!(both.get("A32NX_ENG_2_EEC_NO_VALID_CHANNEL"), Some(&1.0));
        assert_eq!(both.get("A32NX_ENG_1_EEC_NO_VALID_CHANNEL"), Some(&0.0));
    }

    #[test]
    fn a_drifting_eec_sensor_is_caught_by_the_channel_disagree_monitor() {
        // registry.rs: "flagged against channel B once the disagreement
        // exceeds this parameter's threshold".
        let live = EngineAccessoriesLive::new();
        let tgt_a = live.engines[0].ids.eec_sensor_a[3]; // TGT, channel A
        let out = settled(&Faults::from_pairs([(tgt_a, 1.0)]));
        assert_eq!(out.get("A32NX_ENG_1_EEC_SENSOR_DISAGREE"), Some(&1.0));
        assert_eq!(out.get("A32NX_ENG_1_EEC_TGT_DISAGREE"), Some(&1.0));
        assert_eq!(out.get("A32NX_ENG_1_EEC_N1_DISAGREE"), Some(&0.0), "only the failed parameter disagrees");
        assert_eq!(out.get("A32NX_ENG_2_EEC_SENSOR_DISAGREE"), Some(&0.0));
    }

    #[test]
    fn nothing_divides_by_zero_on_four_cold_engines_at_zero_dt() {
        let mut live = EngineAccessoriesLive::new();
        live.tick(&Truth { dt_s: 0.0, ..Truth::default() }, &Faults::default());
        for (name, value) in published(&live) {
            assert!(value.is_finite(), "{name} is not finite");
        }
    }

    /// Exactly the ATA-73 failures are consumed, and the chapters this live
    /// system does not yet drive are the declared ones.
    #[test]
    fn every_ata_73_failure_is_consumed_and_the_rest_are_declared_unconsumed() {
        let mut reg = Registry::default();
        super::super::registry::register(&mut reg);

        let mut consumed: Vec<u64> = Vec::new();
        for eng in 0..N_ENGINES {
            let ids = EngineIds::resolve(&reg, eng);
            consumed.extend([
                ids.lp_wear,
                ids.lp_inlet_restriction,
                ids.filter_clog,
                ids.hp_wear,
                ids.hp_starvation,
                ids.fmu_sticking,
                ids.fmu_spill_open,
                ids.fmu_spill_closed,
                ids.sov_stuck,
                ids.ft_a_bias,
                ids.ft_a_frozen,
                ids.ft_b_bias,
                ids.ft_b_frozen,
                ids.eec_channel_a,
                ids.eec_channel_b,
            ]);
            consumed.extend(ids.nozzle);
            consumed.extend(ids.eec_sensor_a);
            consumed.extend(ids.eec_sensor_b);
        }
        consumed.sort_unstable();
        consumed.dedup();

        let ata_73: Vec<u64> = reg.failures.iter().filter(|f| f.ata == 73).map(|f| f.id).collect();
        for id in &ata_73 {
            assert!(consumed.contains(id), "ATA 73 failure {id} is registered but never read by the live system");
        }
        assert_eq!(consumed.len(), ata_73.len(), "the live system should consume exactly the ATA 73 failures");

        let mut other: Vec<u16> = reg.failures.iter().map(|f| f.ata).filter(|&a| a != 73).collect();
        other.sort_unstable();
        other.dedup();
        assert_eq!(other, UNCONSUMED_ATA, "the chapters this live system does not drive must be the declared ones");
    }

    #[test]
    fn resolving_the_ids_twice_gives_the_same_answer() {
        let mut reg = Registry::default();
        super::super::registry::register(&mut reg);
        let mut reg2 = Registry::default();
        super::super::registry::register(&mut reg2);
        for eng in 0..N_ENGINES {
            assert_eq!(EngineIds::resolve(&reg, eng).filter_clog, EngineIds::resolve(&reg2, eng).filter_clog);
        }
    }
}
