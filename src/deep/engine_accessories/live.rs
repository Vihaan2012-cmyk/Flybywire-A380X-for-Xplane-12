//! The live engine accessories: one owned accessory chain per engine,
//! stepped every frame from [`Truth`] and published under the
//! `A32NX_ENG_n_*` names `registry.rs` names in its ECAM triggers.
//!
//! What it owns, per engine, is everything hung off (or bolted to) that
//! engine's accessory gearbox and nacelle:
//!
//! * the whole fuel path -- LP (boost) pump and inlet strainer, filter and
//!   its 35 psi bypass valve, HP (gear) pump, fuel metering unit with its
//!   metering and spill valves, HP fuel shut-off valve, dual pick-off flow
//!   transmitter, burner manifold and its eight nozzle groups -- and the
//!   dual-channel EEC that meters against all of it (ATA 73);
//! * both ignition chains: exciter and igniter plug, A and B (ATA 74);
//! * the starter air valve, the air turbine starter with its sprag clutch,
//!   and the starter's own duty-cycle heating (ATA 80);
//! * the IP compressor's variable stator vanes (ATA 72) and the IP/HP
//!   handling (surge) bleed valves (ATA 75);
//! * rotor dynamics: three spools' synchronous imbalance vibration and all
//!   five main bearings' defect signatures and chip detectors (ATA 77);
//! * the thrust reverser, on engines 2 and 3 only (ATA 78);
//! * the nacelle anti-ice valve (ATA 30), nacelle ventilation (ATA 71) and
//!   both fire zones' detection loops (ATA 26).
//!
//! Four of each, one per Trent 972B-84 (two of the reverser).
//!
//! ## How the fuel chain is closed
//!
//! The HP pump's delivery depends on the pressure the FMU's spill valve
//! regulates it against, and that differential depends on the flow the HP
//! pump delivers. The loop is closed with the previous frame's value --
//! the same one-frame lag `deep::live`'s own module doc sanctions between
//! areas, and far below the time constant of a fuel system that settles in
//! tenths of a second.
//!
//! ## What the models are driven from
//!
//! Every input below is a real reading. `Truth` carries the three spool
//! speeds, the HP6 and customer bleed ports, the ambient state, the bus
//! voltages, the hydraulic pressures and the cockpit controls (masters,
//! fire handles, starter engagement, nacelle anti-ice). Two inputs come
//! from other areas through [`Truth::published`], one frame behind, which
//! is exactly what that channel exists for:
//!
//! * `DEEP_PNEU_ENG_n_START_DUCT_PRESSURE_PA` -- `deep::pneumatic_ducts`'
//!   own start-duct pressure at the starter air valve, which is what
//!   actually drives the air turbine starter;
//! * `THERMAL_ZONE_NACELLECOWLn_TEMPERATURE_C` -- `deep::thermal_zones`'
//!   own nacelle cowl zone air temperature, which is what the fire
//!   detection loops sense;
//! * `FUEL_TANK_TEMP_C:n` -- `deep::fuel`'s own feed-tank bulk temperature
//!   at the engine LP pump inlet.
//!
//! ## What is still not in `Truth`
//!
//! Two inputs have no real source anywhere in this port, and the models
//! that need them are wired but left at rest rather than fed an invented
//! number (see [`EngineAccessoryCommands`]):
//!
//! * TGT (turbine gas temperature), the EEC's fourth sensed parameter,
//!   measured downstream of the LP turbine -- a station neither this
//!   crate's engine model nor `Truth` publishes;
//! * the thrust reverser's own deploy command. The reverser's three locks
//!   and its actuator are fully modelled and stepped, but nothing in
//!   `Truth` or in any other area's published set carries a reverse-thrust
//!   selection, so the sleeve is never commanded out and the six lock
//!   faults and the actuator jam cannot show. The one reverser path that
//!   *is* live without a command is the one that matters most -- an
//!   uncommanded deployment needs no command by definition.
//!
//! The fuel flow the governor commands is **no longer** in that list: it is
//! `Truth::engine_fuel_flow_kg_s`, this crate's own engine model's real
//! fuel flow into the combustor. That is the flow the FADEC's control law
//! has settled on and the flow the metering unit's job is to deliver, so
//! it is what the FMU is asked for and what the metering monitors compare
//! against.

use std::f64::consts::PI;

use crate::deep::api::Registry;
use crate::deep::live::{Faults, Truth};

use super::airflow_control::bleed_valve::{self, BleedValve, BleedValveFaults, BleedValveSpec, BleedValveState, HP_HANDLING_BLEED, IP_HANDLING_BLEED};
use super::airflow_control::vsv::{Vsv, VsvFaults, VsvState};
use super::eec::{ActiveChannel, Eec, EecFaults, EecState, SensorFaults, PARAMS};
use super::fuel::filter::{self, FilterFaults, FilterState};
use super::fuel::flow_transmitter::{FlowReading, FlowTransmitter, PickoffFaults};
use super::fuel::fmu::{FmuFaults, FmuState, FuelMeteringUnit};
use super::fuel::hp_pump::{self, HpPumpFaults, HpPumpState};
use super::fuel::lp_pump::{self, LpPumpFaults, LpPumpState};
use super::fuel::manifold::{self, ManifoldFaults, ManifoldState, NUM_GROUPS};
use super::fuel::shutoff_valve::{ShutoffValve, ShutoffValveFaults};
use super::ignition::{self, IgnitionFaults, IgnitionState};
use super::nacelle::anti_ice::{AntiIceState, AntiIceValve, AntiIceValveFaults};
use super::nacelle::fire_detection::{self, FireZoneReading, LoopFaults};
use super::nacelle::ventilation::{self, VentilationFaults, VentilationState};
use super::rotor_dynamics::bearing::BearingFaults;
use super::rotor_dynamics::bearings::{BearingState, EngineBearingFaults, EngineBearings};
use super::rotor_dynamics::imbalance::{ImbalanceFaults, SpoolVibration, VibrationState, FAN_SPEC, HP_SPEC, IP_SPEC};
use super::starting::air_valve::{AirValve, AirValveFaults};
use super::starting::duty_cycle::DutyCycleHeat;
use super::starting::turbine::{AirTurbineStarter, AtsFaults, AtsState, FREE_SPEED_FRAC, N3_DESIGN_RPM};
use super::thrust_reverser::{LockFaults, ReverserFaults, ReverserState, ThrustReverser};

const N_ENGINES: usize = 4;
const N_EEC_PARAMS: usize = 5;
/// The three spools, in `N1, N2, N3` order everywhere in this file.
const N_SPOOLS: usize = 3;
/// The five main bearings, in `rotor_dynamics::bearings::BEARINGS` order.
const N_BEARINGS: usize = 5;
/// The two fire zones per engine, in `registry.rs`'s own `CORE, FAN` order.
const N_FIRE_ZONES: usize = 2;
/// The two ignition chains, and the two handling bleed valves (IP, HP).
const N_IGN_CHAINS: usize = 2;
const N_HANDLING_BLEEDS: usize = 2;
/// The three independent reverser locks (primary, secondary, tertiary).
const N_REV_LOCKS: usize = 3;

/// Fan/LP and IP spool design speeds, RPM. Restated from
/// `physics::engine::params`' own `N1_DESIGN_RPM`/`N2_DESIGN_RPM`, which
/// cite the package's `engines.cfg` header comments ("LP - Real N1 - Sim N1
/// - 2,900RPM", "IP - Real N2 - Sim XX - 8,300RPM"); restated rather than
/// imported so this directory keeps no dependency on the gas-path rebuild.
/// The HP spool's comes from `starting::turbine::N3_DESIGN_RPM`, which
/// restates the same file's 12,200 RPM.
const N1_DESIGN_RPM: f64 = 2900.0;
const N2_DESIGN_RPM: f64 = 8300.0;

/// ISA sea-level static temperature, K -- the reference every corrected
/// speed in this file is corrected to (`theta = T / 288.15`).
const ISA_SL_TEMP_K: f64 = 288.15;

/// Standard dry-air gas constant, J/(kg K) -- restated here for the nacelle
/// ventilation dynamic pressure, exactly as `airflow_control::bleed_valve`
/// restates it for the same reason.
const R_AIR: f64 = 287.05;

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

/// Bus voltage below which the ignition exciters are not energised, V.
/// **GENERIC**: half the 115 V AC bus the exciters charge from
/// (`ignition::V_BUS_V`), i.e. anything a real undervoltage cut-out would
/// already have dropped the exciters at.
const IGNITION_MIN_BUS_V: f64 = 0.5 * ignition::V_BUS_V;

/// Design start-air differential across the starter air valve, Pa: the
/// pressure rise the start duct is sized to deliver over the ambient the
/// air turbine starter exhausts into. **GENERIC**, 30 psi, the classic
/// large-transport start-duct design differential, and consistent with the
/// ~45 psia the APU's own bleed reaches at sea level in this port
/// (`deep::apu`'s PW980 bleed output).
const START_DESIGN_DP_PA: f64 = 207_000.0;
/// How far the starter air valve may sit from its commanded position
/// before the disagreement is reported, and how long after a change of
/// command the monitor waits before believing the difference, s.
/// **GENERIC**: the valve's own 3 s full travel plus half again, so a
/// healthy valve opening or closing is never called faulty on its way.
const START_VALVE_DISAGREE_TOLERANCE: f64 = 0.05;
const START_VALVE_SETTLE_S: f64 = 4.5;
/// How far above its own free-running speed the starter rotor must be
/// turning for the sprag to be reported as having failed to disengage, as
/// a fraction of `FREE_SPEED_FRAC`. **GENERIC**: a healthy sprag caps the
/// rotor at exactly its free speed, so any margin at all is a real
/// detection; 1% keeps it clear of floating-point noise.
const STARTER_OVERRUN_MARGIN: f64 = 1.01;

/// Rigging/feedback offset, degrees, that a fully-armed VSV rigging error
/// corresponds to. **GENERIC**: half the vane ring's own 40 degree travel
/// range (`airflow_control::vsv`), an offset a slipped feedback linkage can
/// plausibly reach.
///
/// The registered failure's magnitude is an unsigned 0..1 fraction while
/// the model's `rigging_bias_deg` is signed, so the magnitude drives the
/// **vane-closed** direction: that is the one that raises blade incidence
/// and costs the stall margin the failure's registered effect names, and
/// it is the direction that stays visible at high corrected speed (a
/// vane-open bias saturates against the ring's own open stop).
const MAX_RIGGING_BIAS_DEG: f64 = 20.0;

/// Representative core (IP/HP compressor) design mass flow the handling
/// bleeds' margin contribution is normalised against, kg/s. **Derived**
/// from the publicly quoted Trent 900 figures -- about 1200 kg/s total
/// intake flow at a bypass ratio of about 8.7 -- giving a core flow of
/// 1200 / 9.7 ~ 124 kg/s.
const CORE_DESIGN_FLOW_KG_S: f64 = 124.0;
/// How far a handling bleed valve may sit from its scheduled position
/// before the disagreement is reported, as a fraction of travel.
/// **GENERIC**: wider than the HP SOV's, because this valve chases a
/// continuous schedule rather than an open/shut command and legitimately
/// lags it through a spool transient -- which is what the alert's own 5 s
/// confirmation delay is there to ride out.
const HANDLING_BLEED_DISAGREE_TOLERANCE: f64 = 0.15;

/// How far the nacelle anti-ice valve may sit from its commanded position
/// before the disagreement is reported, as a fraction of travel.
/// **GENERIC**, the same tolerance the HP SOV uses; the valve's 2 s travel
/// sits well inside the alert's own 5 s confirmation delay.
const ANTI_ICE_DISAGREE_TOLERANCE: f64 = 0.05;

/// How far the reverser sleeve may sit from its commanded position before
/// the disagreement is reported, as a fraction of stroke. **GENERIC**,
/// inside the reverser's own 5% uncommanded-deployment threshold.
const REVERSER_DISAGREE_TOLERANCE: f64 = 0.05;
/// Nominal A380 hydraulic system pressure, Pa (5000 psi) -- what the
/// reverser's available actuation pressure fraction is measured against.
const HYDRAULIC_NOMINAL_PA: f64 = 34_474_000.0;

/// Nothing. Every ATA chapter this area registers is now stepped by this
/// live system; the test at the bottom of this file holds that to be true
/// rather than leaving it to be discovered.
pub const UNCONSUMED_ATA: [u16; 0] = [];

// ---------------------------------------------------------------------------
// Inputs that `Truth` does not carry yet.
// ---------------------------------------------------------------------------

/// Everything this area needs that is not in [`Truth`].
///
/// Everything that *is* in `Truth` has been taken out of here: the spool
/// speeds, P30, the master/fire-handle command, the starter engagement,
/// the nacelle anti-ice selection, and -- since this pass -- the fuel flow
/// the governor is commanding, which is `Truth::engine_fuel_flow_kg_s`.
/// What is left are the inputs with no source anywhere in this port.
#[derive(Clone, Copy, Debug)]
pub struct EngineAccessoryCommands {
    /// Turbine gas temperature, K -- the EEC's fourth sensed parameter.
    /// Sensed nowhere else in this port (TGT is measured downstream of the
    /// LP turbine, a station none of this crate's engine model or `Truth`
    /// publishes), so this stays a command rather than a `Truth` read.
    pub engine_tgt_k: [f64; N_ENGINES],
    /// An explicit override for the fuel flow the governor is commanding,
    /// kg/s. `None` -- the normal case -- reads `Truth::engine_fuel_flow_
    /// kg_s`, this crate's own engine model's real flow into the
    /// combustor, which is the flow the metering unit's job is to deliver.
    /// The override exists for tests that need a command the engine model
    /// is not producing.
    pub wf_command_kg_s: Option<[f64; N_ENGINES]>,
    /// Fuel temperature at the engine's LP pump inlet, K -- an explicit
    /// override for tests. `None` reads `deep::fuel::live`'s own published
    /// feed-tank bulk temperature one frame back (`FUEL_TANK_TEMP_C:n`,
    /// through `Truth::published`) and falls back further to ambient static
    /// air only if fuel has not published yet (a cold aircraft with no fuel
    /// area running), which is where a parked, cold-soaked aircraft's feed
    /// fuel genuinely sits.
    pub fuel_inlet_k: Option<f64>,
    /// Reverse thrust selected on engines 2 and 3, in that order.
    ///
    /// **No source exists.** Nothing in `Truth`, in `Controls` or in any
    /// other area's published set carries a reverse-thrust selection: the
    /// compiled FADEC bus exposes a thrust-limit mode and FlyByWire's own
    /// reverser model lives on the other side of `extra_backend_fbw`'s
    /// force path. The reverser is fully modelled and stepped anyway, so
    /// that the day a reverser lever reaches `Truth` this is one line, and
    /// so that the one deployment path that needs no command -- every lock
    /// failing to hold at once -- is live today. Tests drive this field
    /// directly; in the aircraft it is always `false`.
    pub reverser_deploy_commanded: [bool; 2],
}

impl Default for EngineAccessoryCommands {
    /// Four cold engines: nothing commanded, no TGT reading yet.
    fn default() -> Self {
        Self { engine_tgt_k: [ISA_SL_TEMP_K; N_ENGINES], wf_command_kg_s: None, fuel_inlet_k: None, reverser_deploy_commanded: [false; 2] }
    }
}

/// `deep::fuel::live`'s own tank numbering (`FEED` order in that area's
/// `feed_tank_index`, cited there as `flight_model.cfg`'s `Tank.2`/`Tank.5`/
/// `Tank.6`/`Tank.9`): engine 1's feed tank is tank 2 in that 1-based
/// numbering, and so on. Restated here rather than imported so this area
/// keeps no dependency on `deep::fuel`'s internals -- only on the variable
/// name it publishes.
const FEED_TANK_NUMBER: [u32; N_ENGINES] = [2, 5, 6, 9];

/// Which engines carry a thrust reverser, 1-based: the A380's two inboard
/// engines only (`thrust_reverser`'s own module doc, and FlyByWire's own
/// "engine 2/3 EBHA" reverser model).
const REVERSER_ENGINES: [usize; 2] = [2, 3];

/// The three spools' names as the ECAM trigger variables spell them.
const SPOOL_KEYS: [&str; N_SPOOLS] = ["N1", "N2", "N3"];
/// `registry.rs`'s own per-spool component key, in the same order.
const SPOOL_COMPONENT_KEYS: [&str; N_SPOOLS] = ["fan", "ip", "hp"];
/// The five bearings' component keys, in `BEARINGS` order.
const BEARING_KEYS: [&str; N_BEARINGS] = ["fan_front", "ip_front", "hp_turbine", "ip_turbine", "lp_turbine_rear"];
/// Which spool's name each bearing's own ECAM variable is prefixed with --
/// `registry.rs`'s own `var_suffix` column, which follows the spool the
/// bearing actually rides on.
const BEARING_VAR_SPOOL: [&str; N_BEARINGS] = ["N1", "N2", "N3", "N2", "N1"];
/// The two handling bleed valves' keys, IP first.
const BLEED_KEYS: [&str; N_HANDLING_BLEEDS] = ["IP", "HP"];
/// The two fire zones as `registry.rs` names them in its variables.
const FIRE_ZONE_KEYS: [&str; N_FIRE_ZONES] = ["CORE", "FAN"];
/// The two ignition chains' component keys.
const IGN_CHAIN_KEYS: [&str; N_IGN_CHAINS] = ["a", "b"];
/// The three reverser locks' keys as `registry.rs` spells them in the
/// failures' `model_field`.
const REV_LOCK_KEYS: [&str; N_REV_LOCKS] = ["lock_a", "lock_b", "lock_c"];

fn omega_rad_s(rpm: f64) -> f64 {
    rpm * PI / 30.0
}

// ---------------------------------------------------------------------------
// Failure ids.
// ---------------------------------------------------------------------------

/// The one failure on `component` whose registered `model_field` contains
/// `fragment`. A live system that cannot find the failure it is meant to
/// consume is a build error, not something to tolerate at runtime.
fn fid(reg: &Registry, component: &str, fragment: &str) -> u64 {
    let mut found = reg.failures.iter().filter(|f| f.component == component && f.model_field.contains(fragment));
    let first = found.next().unwrap_or_else(|| panic!("no failure on {component} whose model_field contains {fragment:?}"));
    assert!(found.next().is_none(), "more than one failure on {component} matches {fragment:?}");
    first.id
}

/// The ATA-73 failure ids the fuel chain and EEC consume, per engine.
struct FuelIds {
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

impl FuelIds {
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

    fn all(&self) -> Vec<u64> {
        let mut v = vec![
            self.lp_wear,
            self.lp_inlet_restriction,
            self.filter_clog,
            self.hp_wear,
            self.hp_starvation,
            self.fmu_sticking,
            self.fmu_spill_open,
            self.fmu_spill_closed,
            self.sov_stuck,
            self.ft_a_bias,
            self.ft_a_frozen,
            self.ft_b_bias,
            self.ft_b_frozen,
            self.eec_channel_a,
            self.eec_channel_b,
        ];
        v.extend(self.nozzle);
        v.extend(self.eec_sensor_a);
        v.extend(self.eec_sensor_b);
        v
    }
}

/// ATA 74: one exciter and one igniter plug failure per chain.
struct IgnitionIds {
    exciter: [u64; N_IGN_CHAINS],
    igniter: [u64; N_IGN_CHAINS],
}

impl IgnitionIds {
    fn resolve(reg: &Registry, eng: usize) -> Self {
        let n = eng + 1;
        Self {
            exciter: std::array::from_fn(|c| fid(reg, &format!("74_ignition.exciter_{}_{n}", IGN_CHAIN_KEYS[c]), "IgnitionFaults.exciter_")),
            igniter: std::array::from_fn(|c| fid(reg, &format!("74_ignition.igniter_{}_{n}", IGN_CHAIN_KEYS[c]), "IgnitionFaults.igniter_")),
        }
    }

    fn all(&self) -> Vec<u64> {
        let mut v = self.exciter.to_vec();
        v.extend(self.igniter);
        v
    }
}

/// ATA 80: the starter air valve and the two sprag-clutch faults.
struct StartIds {
    sav_stuck: u64,
    clutch_engage: u64,
    clutch_disengage: u64,
}

impl StartIds {
    fn resolve(reg: &Registry, eng: usize) -> Self {
        let n = eng + 1;
        let ats = format!("80_start.ats_{n}");
        Self {
            sav_stuck: fid(reg, &format!("80_start.air_valve_{n}"), "AirValveFaults.stuck"),
            clutch_engage: fid(reg, &ats, "clutch_fails_to_engage"),
            clutch_disengage: fid(reg, &ats, "clutch_fails_to_disengage"),
        }
    }

    fn all(&self) -> Vec<u64> {
        vec![self.sav_stuck, self.clutch_engage, self.clutch_disengage]
    }
}

/// ATA 72 (VSV) and ATA 75 (handling bleeds).
struct AirflowIds {
    vsv_jam: u64,
    vsv_rigging: u64,
    bleed_jam: [u64; N_HANDLING_BLEEDS],
}

impl AirflowIds {
    fn resolve(reg: &Registry, eng: usize) -> Self {
        let n = eng + 1;
        let vsv = format!("72_air.vsv_{n}");
        Self {
            vsv_jam: fid(reg, &vsv, "VsvFaults.jam"),
            vsv_rigging: fid(reg, &vsv, "VsvFaults.rigging_bias_deg"),
            bleed_jam: std::array::from_fn(|i| fid(reg, &format!("75_air.{}_handling_bleed_{n}", BLEED_KEYS[i].to_lowercase()), "BleedValveFaults.jam")),
        }
    }

    fn all(&self) -> Vec<u64> {
        let mut v = vec![self.vsv_jam, self.vsv_rigging];
        v.extend(self.bleed_jam);
        v
    }
}

/// ATA 77: three spools' imbalance causes and five bearings' four defects.
struct RotorIds {
    /// `[spool][blade loss, ice, bird strike]`.
    imbalance: [[u64; 3]; N_SPOOLS],
    /// `[bearing][outer race, inner race, rolling element, cage]`.
    bearing: [[u64; 4]; N_BEARINGS],
}

impl RotorIds {
    fn resolve(reg: &Registry, eng: usize) -> Self {
        let n = eng + 1;
        const CAUSES: [&str; 3] = ["blade_loss_frac", "ice_frac", "bird_strike_frac"];
        const DEFECTS: [&str; 4] = ["outer_race_spall", "inner_race_spall", "rolling_element_spall", "cage_wear"];
        Self {
            imbalance: std::array::from_fn(|s| {
                let rotor = format!("77_vib.{}_rotor_{n}", SPOOL_COMPONENT_KEYS[s]);
                std::array::from_fn(|c| fid(reg, &rotor, CAUSES[c]))
            }),
            bearing: std::array::from_fn(|b| {
                let bearing = format!("77_vib.bearing_{}_{n}", BEARING_KEYS[b]);
                std::array::from_fn(|d| fid(reg, &bearing, DEFECTS[d]))
            }),
        }
    }

    fn all(&self) -> Vec<u64> {
        let mut v = Vec::new();
        for s in self.imbalance {
            v.extend(s);
        }
        for b in self.bearing {
            v.extend(b);
        }
        v
    }
}

/// ATA 78: three locks x (fails to hold, jam), plus the actuator.
struct ReverserIds {
    lock_hold: [u64; N_REV_LOCKS],
    lock_jam: [u64; N_REV_LOCKS],
    actuator_jam: u64,
}

impl ReverserIds {
    fn resolve(reg: &Registry, eng_number: usize) -> Self {
        let id = format!("78_rev.reverser_{eng_number}");
        Self {
            lock_hold: std::array::from_fn(|l| fid(reg, &id, &format!("fails_to_hold ({})", REV_LOCK_KEYS[l]))),
            lock_jam: std::array::from_fn(|l| fid(reg, &id, &format!("jam ({})", REV_LOCK_KEYS[l]))),
            actuator_jam: fid(reg, &id, "ReverserFaults.actuator_jam"),
        }
    }

    fn all(&self) -> Vec<u64> {
        let mut v = self.lock_hold.to_vec();
        v.extend(self.lock_jam);
        v.push(self.actuator_jam);
        v
    }
}

/// ATA 30 (anti-ice), 71 (ventilation) and 26 (fire detection loops).
struct NacelleIds {
    anti_ice_stuck: u64,
    scoop: u64,
    eductor: u64,
    /// `[zone][loop A fails to detect, loop A false trip, loop B fails to
    /// detect, loop B false trip]`.
    fire: [[u64; 4]; N_FIRE_ZONES],
}

impl NacelleIds {
    fn resolve(reg: &Registry, eng: usize) -> Self {
        let n = eng + 1;
        let vent = format!("71_pwr.nacelle_ventilation_{n}");
        const LOOP_FRAGMENTS: [&str; 4] = ["fails_to_detect (loop A", "false_trip (loop A", "fails_to_detect (loop B", "false_trip (loop B"];
        Self {
            anti_ice_stuck: fid(reg, &format!("30_ice.nacelle_valve_{n}"), "AntiIceValveFaults.stuck"),
            scoop: fid(reg, &vent, "scoop_blockage"),
            eductor: fid(reg, &vent, "eductor_blockage"),
            fire: std::array::from_fn(|z| {
                let zone = format!("26_fire.zone_{}_{n}", FIRE_ZONE_KEYS[z].to_lowercase());
                std::array::from_fn(|l| fid(reg, &zone, LOOP_FRAGMENTS[l]))
            }),
        }
    }

    fn all(&self) -> Vec<u64> {
        let mut v = vec![self.anti_ice_stuck, self.scoop, self.eductor];
        for z in self.fire {
            v.extend(z);
        }
        v
    }
}

// ---------------------------------------------------------------------------
// Published variable names, built once.
// ---------------------------------------------------------------------------

/// Every name this engine publishes, and the two names it reads from other
/// areas, built once at construction.
///
/// `publish` runs every frame and `tick` reads two cross-area names every
/// frame; `format!`-ing about seventy names per engine at 30-60 Hz would
/// allocate and free some seventeen thousand short strings a second for
/// values whose *names* never change. `deep::pneumatic_ducts::live` already
/// takes exactly this approach for the same reason.
struct ChainNames {
    // Fuel chain.
    filter_impending_bypass: String,
    filter_bypassed: String,
    filter_dp_pa: String,
    oil_filter_bypassed: String,
    hp_pump_low_flow: String,
    hp_pump_flow: String,
    lp_pump_outlet_pa: String,
    lp_pump_cavitating: String,
    fmu_fault: String,
    thrust_abnormal: String,
    fmu_metered: String,
    fmu_dp_pa: String,
    wf_command: String,
    sov_disagree: String,
    sov_position: String,
    ff_channel_disagree: String,
    ff_disagree: String,
    ff_indicated: String,
    ff_true: String,
    nozzle_imbalance: String,
    nozzle_hot_streak: String,
    manifold_gauge_pa: String,
    // EEC.
    eec_channel_fault: String,
    eec_no_valid_channel: String,
    eec_sensor_disagree: String,
    eec_selected: [String; N_EEC_PARAMS],
    eec_disagree: [String; N_EEC_PARAMS],
    // Ignition.
    ign_spark_rate: [String; N_IGN_CHAINS],
    ign_powered: String,
    no_ignition_available: String,
    // Starting.
    start_valve_position: String,
    start_valve_disagree: String,
    starter_torque: String,
    starter_rotor_rpm: String,
    starter_disengage_fault: String,
    starter_disintegrated: String,
    starter_overheat: String,
    starter_housing_rise_k: String,
    // Compressor airflow control.
    vsv_angle: String,
    vsv_schedule_error: String,
    vsv_stall_margin: String,
    bleed_position: [String; N_HANDLING_BLEEDS],
    bleed_disagree: [String; N_HANDLING_BLEEDS],
    bleed_flow: [String; N_HANDLING_BLEEDS],
    bleed_stall_margin: [String; N_HANDLING_BLEEDS],
    // Rotor dynamics.
    vib_index: [String; N_SPOOLS],
    bearing_amplitude: [String; N_BEARINGS],
    bearing_chip: [String; N_BEARINGS],
    bearing_debris: [String; N_BEARINGS],
    // Nacelle.
    anti_ice_position: String,
    anti_ice_disagree: String,
    anti_ice_lip_temp_k: String,
    anti_ice_bleed_flow: String,
    nacelle_vent_flow: String,
    nacelle_vapour_risk: String,
    fire_loop_disagree: [String; N_FIRE_ZONES],
    fire_confirmed: [String; N_FIRE_ZONES],
    // Read from other areas, one frame behind.
    read_feed_tank_temp_c: String,
    read_start_duct_pa: String,
    read_nacelle_zone_temp_c: String,
}

impl ChainNames {
    fn new(eng: usize) -> Self {
        let n = eng + 1;
        let v = |suffix: &str| format!("A32NX_ENG_{n}_{suffix}");
        Self {
            filter_impending_bypass: v("FUEL_FILTER_IMPENDING_BYPASS"),
            filter_bypassed: v("FUEL_FILTER_BYPASSED"),
            filter_dp_pa: v("FUEL_FILTER_DP_PA"),
            oil_filter_bypassed: v("OIL_FILTER_BYPASSED"),
            hp_pump_low_flow: v("HP_PUMP_LOW_FLOW"),
            hp_pump_flow: v("HP_PUMP_FLOW_KG_S"),
            lp_pump_outlet_pa: v("LP_PUMP_OUTLET_PA"),
            lp_pump_cavitating: v("LP_PUMP_CAVITATING"),
            fmu_fault: v("FMU_FAULT"),
            thrust_abnormal: v("THRUST_ABNORMAL"),
            fmu_metered: v("FMU_METERED_KG_S"),
            fmu_dp_pa: v("FMU_DP_PA"),
            wf_command: v("WF_COMMAND_KG_S"),
            sov_disagree: v("HP_SOV_DISAGREE"),
            sov_position: v("HP_SOV_POSITION"),
            ff_channel_disagree: v("FF_CHANNEL_DISAGREE"),
            ff_disagree: v("FF_DISAGREE"),
            ff_indicated: v("FF_INDICATED_KG_S"),
            ff_true: v("FF_TRUE_KG_S"),
            nozzle_imbalance: v("NOZZLE_IMBALANCE"),
            nozzle_hot_streak: v("NOZZLE_HOT_STREAK"),
            manifold_gauge_pa: v("MANIFOLD_GAUGE_PA"),
            eec_channel_fault: v("EEC_CHANNEL_FAULT"),
            eec_no_valid_channel: v("EEC_NO_VALID_CHANNEL"),
            eec_sensor_disagree: v("EEC_SENSOR_DISAGREE"),
            eec_selected: std::array::from_fn(|i| v(&format!("EEC_{}_SELECTED", eec_param_key(i)))),
            eec_disagree: std::array::from_fn(|i| v(&format!("EEC_{}_DISAGREE", eec_param_key(i)))),
            ign_spark_rate: std::array::from_fn(|c| v(&format!("IGN_{}_SPARK_RATE_HZ", IGN_CHAIN_KEYS[c].to_uppercase()))),
            ign_powered: v("IGN_POWERED"),
            no_ignition_available: v("NO_IGNITION_AVAILABLE"),
            start_valve_position: v("START_VALVE_POSITION"),
            start_valve_disagree: v("START_VALVE_DISAGREE"),
            starter_torque: v("STARTER_TORQUE_NM"),
            starter_rotor_rpm: v("STARTER_ROTOR_RPM"),
            starter_disengage_fault: v("STARTER_DISENGAGE_FAULT"),
            starter_disintegrated: v("STARTER_DISINTEGRATED"),
            starter_overheat: v("STARTER_OVERHEAT"),
            starter_housing_rise_k: v("STARTER_HOUSING_RISE_K"),
            vsv_angle: v("VSV_ANGLE_DEG"),
            vsv_schedule_error: v("VSV_SCHEDULE_ERROR_DEG"),
            vsv_stall_margin: v("VSV_STALL_MARGIN_DELTA_PCT"),
            bleed_position: std::array::from_fn(|i| v(&format!("{}_HANDLING_BLEED_POSITION", BLEED_KEYS[i]))),
            bleed_disagree: std::array::from_fn(|i| v(&format!("{}_HANDLING_BLEED_DISAGREE", BLEED_KEYS[i]))),
            bleed_flow: std::array::from_fn(|i| v(&format!("{}_HANDLING_BLEED_KG_S", BLEED_KEYS[i]))),
            bleed_stall_margin: std::array::from_fn(|i| v(&format!("{}_HANDLING_BLEED_STALL_MARGIN_PCT", BLEED_KEYS[i]))),
            vib_index: std::array::from_fn(|s| v(&format!("{}_VIB_INDEX", SPOOL_KEYS[s]))),
            bearing_amplitude: std::array::from_fn(|b| v(&format!("{}_{}_DEFECT_AMPLITUDE_MM_S", BEARING_VAR_SPOOL[b], BEARING_KEYS[b].to_uppercase()))),
            bearing_chip: std::array::from_fn(|b| v(&format!("{}_CHIP_DETECTED", BEARING_KEYS[b].to_uppercase()))),
            bearing_debris: std::array::from_fn(|b| v(&format!("{}_DEBRIS_G", BEARING_KEYS[b].to_uppercase()))),
            anti_ice_position: v("ANTI_ICE_POSITION"),
            anti_ice_disagree: v("ANTI_ICE_DISAGREE"),
            anti_ice_lip_temp_k: v("ANTI_ICE_LIP_TEMP_K"),
            anti_ice_bleed_flow: v("ANTI_ICE_BLEED_KG_S"),
            nacelle_vent_flow: v("NACELLE_VENT_FLOW_KG_S"),
            nacelle_vapour_risk: v("NACELLE_VAPOUR_RISK"),
            fire_loop_disagree: std::array::from_fn(|z| v(&format!("{}_FIRE_LOOP_DISAGREE", FIRE_ZONE_KEYS[z]))),
            fire_confirmed: std::array::from_fn(|z| v(&format!("{}_FIRE_CONFIRMED", FIRE_ZONE_KEYS[z]))),
            read_feed_tank_temp_c: format!("FUEL_TANK_TEMP_C:{}", FEED_TANK_NUMBER[eng]),
            read_start_duct_pa: format!("DEEP_PNEU_ENG_{n}_START_DUCT_PRESSURE_PA"),
            read_nacelle_zone_temp_c: format!("THERMAL_ZONE_NACELLECOWL{n}_TEMPERATURE_C"),
        }
    }
}

fn eec_param_key(i: usize) -> &'static str {
    match PARAMS[i] {
        super::eec::Param::N1 => "N1",
        super::eec::Param::N2 => "N2",
        super::eec::Param::N3 => "N3",
        super::eec::Param::Tgt => "TGT",
        super::eec::Param::P30 => "P30",
    }
}

/// One reverser's published names -- only engines 2 and 3 have one.
struct ReverserNames {
    position: String,
    uncommanded: String,
    position_disagree: String,
}

impl ReverserNames {
    fn new(eng_number: usize) -> Self {
        Self {
            position: format!("A32NX_ENG_{eng_number}_REV_POSITION"),
            uncommanded: format!("A32NX_ENG_{eng_number}_REV_UNCOMMANDED"),
            position_disagree: format!("A32NX_ENG_{eng_number}_REV_POSITION_DISAGREE"),
        }
    }
}

/// One engine's thrust reverser: the model, its ids, its names and this
/// frame's state.
struct ReverserUnit {
    /// Which of the two reverser-carrying engines this is, indexing
    /// `EngineAccessoryCommands::reverser_deploy_commanded`.
    slot: usize,
    ids: ReverserIds,
    names: ReverserNames,
    model: ThrustReverser,
    state: ReverserState,
    /// What the sleeve was commanded to this frame -- cached because
    /// `publish` has no access to the commands.
    commanded: bool,
}

impl ReverserUnit {
    fn new(reg: &Registry, eng_number: usize) -> Self {
        Self {
            slot: REVERSER_ENGINES.iter().position(|e| *e == eng_number).expect("only engines 2 and 3 carry a reverser"),
            ids: ReverserIds::resolve(reg, eng_number),
            names: ReverserNames::new(eng_number),
            model: ThrustReverser::new(),
            state: ReverserState::default(),
            commanded: false,
        }
    }

    fn step(&mut self, commanded: bool, hydraulic_frac: f64, faults: &Faults, dt: f64) {
        let lock = |hold: u64, jam: u64| LockFaults { fails_to_hold: faults.get(hold), jam: faults.get(jam) };
        let rev_faults = ReverserFaults {
            lock_a: lock(self.ids.lock_hold[0], self.ids.lock_jam[0]),
            lock_b: lock(self.ids.lock_hold[1], self.ids.lock_jam[1]),
            lock_c: lock(self.ids.lock_hold[2], self.ids.lock_jam[2]),
            actuator_jam: faults.get(self.ids.actuator_jam),
        };
        self.commanded = commanded;
        self.state = self.model.step(commanded, hydraulic_frac, &rev_faults, dt);
    }
}

// ---------------------------------------------------------------------------
// One engine's accessories.
// ---------------------------------------------------------------------------

/// One engine's accessories: the whole fuel path, its EEC, both ignition
/// chains, the starter, the compressor's variable geometry and handling
/// bleeds, the rotors and bearings, the nacelle, and (engines 2 and 3) the
/// thrust reverser.
struct EngineChain {
    names: ChainNames,

    // ---- Fuel and control (ATA 73).
    fuel_ids: FuelIds,
    fmu: FuelMeteringUnit,
    sov: ShutoffValve,
    transmitter: FlowTransmitter,
    eec: Eec,
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
    /// The flow the governor asked for this frame, and whether the HP
    /// shut-off valve was commanded open -- cached because `publish`
    /// (unlike `step`) sees neither `Truth` nor the commands.
    wf_command_kg_s: f64,
    sov_commanded_open: bool,

    // ---- Ignition (ATA 74).
    ignition_ids: IgnitionIds,
    ignition_state: IgnitionState,
    ignition_powered: bool,

    // ---- Starting (ATA 80).
    start_ids: StartIds,
    start_valve: AirValve,
    starter: AirTurbineStarter,
    starter_heat: DutyCycleHeat,
    start_valve_position: f64,
    start_valve_commanded_open: bool,
    /// How long the starter air valve's command has been steady, s: a real
    /// valve-position monitor gives the valve its travel time before
    /// calling a difference a fault.
    start_valve_steady_s: f64,
    ats_state: AtsState,

    // ---- Compressor airflow control (ATA 72/75).
    airflow_ids: AirflowIds,
    vsv: Vsv,
    vsv_state: VsvState,
    handling_bleeds: [BleedValve; N_HANDLING_BLEEDS],
    handling_bleed_state: [BleedValveState; N_HANDLING_BLEEDS],
    handling_bleed_target: [f64; N_HANDLING_BLEEDS],

    // ---- Rotor dynamics (ATA 77).
    rotor_ids: RotorIds,
    spool_vibration: [SpoolVibration; N_SPOOLS],
    vibration_state: [VibrationState; N_SPOOLS],
    bearings: EngineBearings,
    bearing_state: [BearingState; N_BEARINGS],

    // ---- Nacelle (ATA 30/71/26).
    nacelle_ids: NacelleIds,
    anti_ice: AntiIceValve,
    anti_ice_state: AntiIceState,
    anti_ice_commanded_open: bool,
    ventilation_state: VentilationState,
    fire_reading: [FireZoneReading; N_FIRE_ZONES],
    /// Whether this engine was running this frame -- cached because
    /// `publish` does not see `Truth`.
    engine_running: bool,
    /// Whether the oil filter's own bypass valve is open this frame, cached
    /// from `Truth::engine_oil_filter_bypassed` (`physics::engine::oil`'s
    /// `OilState::filter_bypassed`, sourced through `engine_commands.rs`'s
    /// `ENGINE_OIL_FILTER_BYPASS:n`) for the same reason `engine_running`
    /// is cached: `publish` does not see `Truth`.
    oil_filter_bypassed: bool,

    // ---- Thrust reverser (ATA 78, engines 2 and 3 only).
    reverser: Option<ReverserUnit>,
}

impl EngineChain {
    fn new(reg: &Registry, eng: usize) -> Self {
        let n = eng + 1;
        Self {
            names: ChainNames::new(eng),

            fuel_ids: FuelIds::resolve(reg, eng),
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
            eec_state: EecState {
                selected: [0.0; N_EEC_PARAMS],
                disagree: [false; N_EEC_PARAMS],
                active: ActiveChannel::A,
                fuel_flow_disagree: false,
                channel_a_serviceable: true,
                channel_b_serviceable: true,
            },
            delivered_kg_s: 0.0,
            wf_command_kg_s: 0.0,
            sov_commanded_open: false,

            ignition_ids: IgnitionIds::resolve(reg, eng),
            ignition_state: IgnitionState::default(),
            ignition_powered: false,

            start_ids: StartIds::resolve(reg, eng),
            // A cold engine's starter air valve is shut.
            start_valve: AirValve::new(false),
            starter: AirTurbineStarter::new(),
            starter_heat: DutyCycleHeat::new(),
            start_valve_position: 0.0,
            start_valve_commanded_open: false,
            start_valve_steady_s: START_VALVE_SETTLE_S,
            ats_state: AtsState::default(),

            airflow_ids: AirflowIds::resolve(reg, eng),
            // A cold engine's vanes sit at the bottom of their schedule.
            vsv: Vsv::new(0.0),
            vsv_state: VsvState::default(),
            handling_bleeds: [BleedValve::new(); N_HANDLING_BLEEDS],
            handling_bleed_state: [BleedValveState::default(); N_HANDLING_BLEEDS],
            handling_bleed_target: [1.0; N_HANDLING_BLEEDS],

            rotor_ids: RotorIds::resolve(reg, eng),
            spool_vibration: [SpoolVibration::new(FAN_SPEC), SpoolVibration::new(IP_SPEC), SpoolVibration::new(HP_SPEC)],
            vibration_state: [VibrationState::default(); N_SPOOLS],
            bearings: EngineBearings::new(),
            bearing_state: [BearingState::default(); N_BEARINGS],

            nacelle_ids: NacelleIds::resolve(reg, eng),
            anti_ice: AntiIceValve::new(ISA_SL_TEMP_K),
            anti_ice_state: AntiIceState::default(),
            anti_ice_commanded_open: false,
            ventilation_state: VentilationState::default(),
            fire_reading: [FireZoneReading::default(); N_FIRE_ZONES],
            engine_running: false,
            oil_filter_bypassed: false,

            reverser: REVERSER_ENGINES.contains(&n).then(|| ReverserUnit::new(reg, n)),
        }
    }

    /// Every failure id this chain reads -- the list the test at the bottom
    /// of this file checks against the registry.
    fn consumed_ids(&self) -> Vec<u64> {
        let mut v = self.fuel_ids.all();
        v.extend(self.ignition_ids.all());
        v.extend(self.start_ids.all());
        v.extend(self.airflow_ids.all());
        v.extend(self.rotor_ids.all());
        v.extend(self.nacelle_ids.all());
        if let Some(rev) = &self.reverser {
            v.extend(rev.ids.all());
        }
        v
    }

    /// One engine's accessories, one frame.
    fn step(&mut self, eng: usize, truth: &Truth, faults: &Faults, commands: &EngineAccessoryCommands, dt: f64) {
        let ambient_pa = truth.environment.ambient_pressure_pa.max(0.0);
        let ambient_k = (truth.environment.sat_c + 273.15).max(1.0);
        // `theta`, the corrected-speed temperature ratio every compressor
        // schedule in this file is written against.
        let theta_sqrt = (ambient_k / ISA_SL_TEMP_K).sqrt().max(1e-6);

        let n1 = truth.engine_n1_frac[eng].max(0.0);
        let n2 = truth.engine_n2_frac[eng].max(0.0);
        let n3 = truth.engine_n3_frac[eng].max(0.0);

        self.step_fuel(eng, truth, faults, commands, dt, ambient_pa, n2, n3);
        self.step_ignition(eng, truth, faults);
        self.step_starting(eng, truth, faults, dt, ambient_pa, n3);
        self.step_airflow(eng, truth, faults, dt, ambient_pa, n2 / theta_sqrt, n3 / theta_sqrt);
        self.step_rotors(faults, dt, n1, n2, n3);
        self.step_nacelle(eng, truth, faults, dt, ambient_pa, ambient_k);
        self.step_reverser(truth, faults, commands, dt);
    }

    // ---- ATA 73: the fuel path and the EEC. -------------------------------

    #[allow(clippy::too_many_arguments)]
    fn step_fuel(&mut self, eng: usize, truth: &Truth, faults: &Faults, commands: &EngineAccessoryCommands, dt: f64, ambient_pa: f64, n2: f64, n3: f64) {
        // `deep::fuel::live` publishes each feed tank's own bulk
        // temperature one frame behind; a manual override (tests) wins,
        // then that published reading, then ambient static air for a cold
        // aircraft fuel has not run for yet.
        let published_fuel_k = truth.published.get(&self.names.read_feed_tank_temp_c).map(|c| c + 273.15);
        let fuel_k = commands.fuel_inlet_k.or(published_fuel_k).unwrap_or(truth.environment.sat_c + 273.15).max(1.0);
        let inlet_pa = ambient_pa + FEED_BOOST_RISE_PA;

        // The flow the governor is commanding: this crate's own engine
        // model's real fuel flow into the combustor, which is what the
        // FADEC's control law settled on and what the metering unit's job
        // is to deliver. A test override wins over it.
        let wf_command = commands.wf_command_kg_s.map_or(truth.engine_fuel_flow_kg_s[eng], |c| c[eng]).max(0.0);
        self.wf_command_kg_s = wf_command;

        // ---- HP pump: works against the differential the FMU's spill
        // valve regulated last frame (the loop is closed with a one-frame
        // lag; see the module doc). Both fuel pumps are geared to N3, not
        // to the fan.
        let starvation_from_lp = if self.hp.delivered_m3_s > 1e-9 { (1.0 - self.lp.flow_m3_s / self.hp.delivered_m3_s).clamp(0.0, 1.0) } else { 0.0 };
        let hp_faults = HpPumpFaults {
            wear: faults.get(self.fuel_ids.hp_wear),
            // `registry.rs`: inlet starvation is "fed forward from LP pump
            // cavitation", so the armed failure and the LP stage's own
            // inability to keep up are the same physical input, whichever
            // is worse.
            inlet_starvation: faults.get(self.fuel_ids.hp_starvation).max(starvation_from_lp),
        };
        self.hp = hp_pump::step(n3, self.fmu_state.differential_pa, &hp_faults);

        // ---- LP pump and filter feed it.
        let lp_faults = LpPumpFaults { wear: faults.get(self.fuel_ids.lp_wear), inlet_restriction: faults.get(self.fuel_ids.lp_inlet_restriction) };
        self.lp = lp_pump::step(n3, inlet_pa, fuel_k, self.hp.delivered_m3_s, &lp_faults);
        let filter_faults = FilterFaults { clog: faults.get(self.fuel_ids.filter_clog) };
        self.filter = filter::step(self.lp.outlet_pa, self.hp.delivered_m3_s, fuel_k, &filter_faults);

        // ---- FMU: meters the commanded flow out of what the HP pump
        // delivers at the pressure it delivers it.
        let fmu_faults = FmuFaults {
            valve_sticking: faults.get(self.fuel_ids.fmu_sticking),
            spill_stuck_open: faults.get(self.fuel_ids.fmu_spill_open),
            spill_stuck_closed: faults.get(self.fuel_ids.fmu_spill_closed),
        };
        let hp_supply_pa = self.filter.outlet_pa + self.fmu_state.differential_pa;
        self.fmu_state = self.fmu.step(wf_command, hp_supply_pa, self.hp.delivered_m3_s, &fmu_faults, dt);

        // ---- HP shut-off valve: open on the master switch unless the fire
        // handle has been pulled. Both are real cockpit controls.
        let sov_faults = ShutoffValveFaults { stuck: faults.get(self.fuel_ids.sov_stuck) };
        self.sov_commanded_open = truth.controls.engine_master_on[eng] && !truth.controls.fire_pb_released[eng];
        self.sov_position = self.sov.step(self.sov_commanded_open, &sov_faults, dt);
        self.delivered_kg_s = self.fmu_state.metered_kg_s * self.sov_position;

        // ---- Flow transmitter, then the manifold it feeds.
        let a = PickoffFaults { bias_frac_of_design: faults.get(self.fuel_ids.ft_a_bias), frozen: faults.get(self.fuel_ids.ft_a_frozen) };
        let b = PickoffFaults { bias_frac_of_design: faults.get(self.fuel_ids.ft_b_bias), frozen: faults.get(self.fuel_ids.ft_b_frozen) };
        self.flow = self.transmitter.step(self.delivered_kg_s, &a, &b, dt);

        // P30 (HP compressor delivery pressure) is the same station
        // `Truth::engine_hp_port_pressure_pa` already reads unconditionally
        // off this crate's own engine model's HP6 port.
        let p30_pa = truth.engine_hp_port_pressure_pa[eng];
        let manifold_faults = ManifoldFaults { group_blockage: std::array::from_fn(|g| faults.get(self.fuel_ids.nozzle[g])) };
        self.manifold = manifold::step(self.delivered_kg_s, p30_pa, &manifold_faults);

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
            channel_a_fault: faults.get(self.fuel_ids.eec_channel_a),
            channel_b_fault: faults.get(self.fuel_ids.eec_channel_b),
            sensor_a: std::array::from_fn(|i| sensor(self.fuel_ids.eec_sensor_a[i])),
            sensor_b: std::array::from_fn(|i| sensor(self.fuel_ids.eec_sensor_b[i])),
        };
        let true_values = [truth.engine_n1_frac[eng] * 100.0, n2 * 100.0, n3 * 100.0, commands.engine_tgt_k[eng], p30_pa];
        self.eec_state = self.eec.step(true_values, self.flow.channel_a_kg_s, self.flow.channel_b_kg_s, &eec_faults, dt);
    }

    // ---- ATA 74: ignition. ------------------------------------------------

    /// Both exciters are energised together whenever the start sequence has
    /// them or the master is on with the engine not yet running (light-up
    /// and relight), and only while the engine's own AC bus is live.
    ///
    /// Engine `n` is taken to be fed from AC bus `n`, the A380's own
    /// one-VFG-per-engine arrangement; `Truth::ac_bus_volts` carries all
    /// four. Which flight phases select *continuous* ignition is the
    /// FADEC's decision and is not modelled here, exactly as
    /// `ignition::step`'s own doc says.
    fn step_ignition(&mut self, eng: usize, truth: &Truth, faults: &Faults) {
        let bus_live = truth.ac_bus_volts[eng] >= IGNITION_MIN_BUS_V;
        let selected = truth.controls.starter_engaged[eng] || (truth.controls.engine_master_on[eng] && !truth.engine_running[eng]);
        self.ignition_powered = bus_live && selected;
        let ign_faults = IgnitionFaults {
            exciter_a_failure: faults.get(self.ignition_ids.exciter[0]),
            exciter_b_failure: faults.get(self.ignition_ids.exciter[1]),
            igniter_a_erosion: faults.get(self.ignition_ids.igniter[0]),
            igniter_b_erosion: faults.get(self.ignition_ids.igniter[1]),
        };
        self.ignition_state = ignition::step(self.ignition_powered, &ign_faults);
    }

    // ---- ATA 80: starting. ------------------------------------------------

    /// The starter air valve, the air turbine starter and its duty-cycle
    /// heating.
    ///
    /// The air actually available is `deep::pneumatic_ducts`' own start-duct
    /// pressure at this engine, read one frame behind through
    /// `Truth::published`; with nothing published yet it falls back to
    /// ambient, i.e. no start air at all, which is the honest answer for an
    /// aircraft whose pneumatic area is not running.
    fn step_starting(&mut self, eng: usize, truth: &Truth, faults: &Faults, dt: f64, ambient_pa: f64, n3: f64) {
        self.start_valve_commanded_open = truth.controls.starter_engaged[eng];
        let valve_faults = AirValveFaults { stuck: faults.get(self.start_ids.sav_stuck) };
        self.start_valve_position = self.start_valve.step(self.start_valve_commanded_open, &valve_faults, dt);

        let duct_pa = truth.published.get_or(&self.names.read_start_duct_pa, ambient_pa);
        let supply_ratio = ((duct_pa - ambient_pa) / START_DESIGN_DP_PA).clamp(0.0, 1.5);
        let supply_fraction = supply_ratio * self.start_valve_position;

        let ats_faults = AtsFaults {
            clutch_fails_to_engage: faults.get(self.start_ids.clutch_engage),
            clutch_fails_to_disengage: faults.get(self.start_ids.clutch_disengage),
        };
        self.ats_state = self.starter.step(supply_fraction, n3 * N3_DESIGN_RPM, &ats_faults);

        // The heat the gearbox actually takes: the mechanical power the
        // turbine is delivering through it (torque x the rotor's own
        // angular speed), zero whenever it is not driving.
        let rotor_omega = omega_rad_s(self.ats_state.rotor_rpm);
        let cranking_power_w = (self.ats_state.torque_n_m * rotor_omega).max(0.0);
        self.starter_heat.step(cranking_power_w, dt);
    }

    /// Whether the starter air valve is far enough from its command, for
    /// long enough, to be reported as disagreeing.
    fn start_valve_disagrees(&self) -> bool {
        let target = if self.start_valve_commanded_open { 1.0 } else { 0.0 };
        self.start_valve_steady_s >= START_VALVE_SETTLE_S && (self.start_valve_position - target).abs() > START_VALVE_DISAGREE_TOLERANCE
    }

    /// Whether the sprag clutch has failed to let the starter rotor
    /// freewheel: a healthy sprag caps the rotor at the turbine's own
    /// free-running speed however fast the spool turns, so a rotor above
    /// that speed is being dragged by the spool and nothing else.
    fn starter_disengage_fault(&self) -> bool {
        self.ats_state.rotor_rpm > N3_DESIGN_RPM * FREE_SPEED_FRAC * STARTER_OVERRUN_MARGIN
    }

    // ---- ATA 72/75: compressor variable geometry and handling bleeds. -----

    /// The IP compressor's variable stator vanes and both handling bleed
    /// valves, each on its own spool's corrected speed.
    #[allow(clippy::too_many_arguments)]
    fn step_airflow(&mut self, eng: usize, truth: &Truth, faults: &Faults, dt: f64, ambient_pa: f64, n2_corrected: f64, n3_corrected: f64) {
        // The registered magnitude is unsigned; it drives the vane-closed
        // direction, the one that costs stall margin (see
        // `MAX_RIGGING_BIAS_DEG`).
        let vsv_faults = VsvFaults { jam: faults.get(self.airflow_ids.vsv_jam), rigging_bias_deg: -MAX_RIGGING_BIAS_DEG * faults.get(self.airflow_ids.vsv_rigging) };
        self.vsv_state = self.vsv.step(n2_corrected, &vsv_faults, dt);

        // The IP valve bleeds the IP compressor's delivery -- the customer
        // bleed port this crate's engine model already exposes, which is
        // the only IP-station reading in `Truth`. The HP valve bleeds HP6,
        // which `Truth` carries unconditionally.
        let sources = [
            (truth.engine_bleed_pressure_pa[eng], truth.engine_bleed_temp_k[eng], n2_corrected, &IP_HANDLING_BLEED),
            (truth.engine_hp_port_pressure_pa[eng], truth.engine_hp_port_temp_k[eng], n3_corrected, &HP_HANDLING_BLEED),
        ];
        for i in 0..N_HANDLING_BLEEDS {
            let (upstream_pa, upstream_k, corrected, spec): (f64, f64, f64, &BleedValveSpec) = sources[i];
            self.handling_bleed_target[i] = bleed_valve::schedule_open_fraction(spec, corrected);
            let bleed_faults = BleedValveFaults { jam: faults.get(self.airflow_ids.bleed_jam[i]) };
            self.handling_bleed_state[i] = self.handling_bleeds[i].step(spec, corrected, upstream_pa, upstream_k, ambient_pa, CORE_DESIGN_FLOW_KG_S, &bleed_faults, dt);
        }
    }

    // ---- ATA 77: rotor imbalance and bearings. ----------------------------

    fn step_rotors(&mut self, faults: &Faults, dt: f64, n1: f64, n2: f64, n3: f64) {
        let design = [omega_rad_s(N1_DESIGN_RPM), omega_rad_s(N2_DESIGN_RPM), omega_rad_s(N3_DESIGN_RPM)];
        let omega = [design[0] * n1, design[1] * n2, design[2] * n3];

        for s in 0..N_SPOOLS {
            let ids = self.rotor_ids.imbalance[s];
            let imbalance = ImbalanceFaults { blade_loss_frac: faults.get(ids[0]), ice_frac: faults.get(ids[1]), bird_strike_frac: faults.get(ids[2]) };
            self.vibration_state[s] = self.spool_vibration[s].step(omega[s], &imbalance, dt);
        }

        let mut bearing_faults = EngineBearingFaults::default();
        for b in 0..N_BEARINGS {
            let ids = self.rotor_ids.bearing[b];
            bearing_faults.faults[b] =
                BearingFaults { outer_race_spall: faults.get(ids[0]), inner_race_spall: faults.get(ids[1]), rolling_element_spall: faults.get(ids[2]), cage_wear: faults.get(ids[3]) };
        }
        self.bearing_state = self.bearings.step(omega[0], omega[1], omega[2], design[0], design[1], design[2], &bearing_faults, dt);
    }

    // ---- ATA 30/71/26: the nacelle. ---------------------------------------

    fn step_nacelle(&mut self, eng: usize, truth: &Truth, faults: &Faults, dt: f64, ambient_pa: f64, ambient_k: f64) {
        // Anti-ice: a hot-bleed valve off the engine's own customer bleed
        // port, commanded by the overhead ENG ANTI ICE pushbutton.
        self.anti_ice_commanded_open = truth.controls.nacelle_anti_ice_selected[eng];
        let ai_faults = AntiIceValveFaults { stuck: faults.get(self.nacelle_ids.anti_ice_stuck) };
        self.anti_ice_state = self.anti_ice.step(
            self.anti_ice_commanded_open,
            truth.engine_bleed_pressure_pa[eng],
            truth.engine_bleed_temp_k[eng],
            ambient_k,
            &ai_faults,
            dt,
        );

        // Ventilation: ram air through the cowl scoops plus the eductor the
        // running engine drives. Dynamic pressure is the real one --
        // ambient density from the ambient pressure and temperature, times
        // true airspeed squared.
        let rho = ambient_pa / (R_AIR * ambient_k);
        let tas = truth.environment.tas_ms.max(0.0);
        let dynamic_pressure_pa = 0.5 * rho * tas * tas;
        let vent_faults = VentilationFaults { scoop_blockage: faults.get(self.nacelle_ids.scoop), eductor_blockage: faults.get(self.nacelle_ids.eductor) };
        self.engine_running = truth.engine_running[eng];
        self.oil_filter_bypassed = truth.engine_oil_filter_bypassed[eng];
        self.ventilation_state = ventilation::step(dynamic_pressure_pa, self.engine_running, &vent_faults);

        // Fire detection: both zones' loops sense the nacelle cowl zone
        // `deep::thermal_zones` owns, read one frame behind. That area
        // models one nacelle cowl zone per engine rather than splitting
        // core from fan, so both zones' loops sense the same temperature
        // today -- a documented limitation of the input, not of the loops,
        // which are independent per zone in every other respect.
        let zone_k = truth.published.get_or(&self.names.read_nacelle_zone_temp_c, truth.environment.sat_c) + 273.15;
        for z in 0..N_FIRE_ZONES {
            let ids = self.nacelle_ids.fire[z];
            let loop_a = LoopFaults { fails_to_detect: faults.get(ids[0]), false_trip: faults.get(ids[1]) };
            let loop_b = LoopFaults { fails_to_detect: faults.get(ids[2]), false_trip: faults.get(ids[3]) };
            self.fire_reading[z] = fire_detection::read(zone_k, &loop_a, &loop_b);
        }
    }

    // ---- ATA 78: the thrust reverser. -------------------------------------

    fn step_reverser(&mut self, truth: &Truth, faults: &Faults, commands: &EngineAccessoryCommands, dt: f64) {
        let Some(rev) = &mut self.reverser else { return };
        // The EBHA's actuation pressure: whichever of the two systems is
        // better off, against the A380's own 5000 psi. The electrical
        // back-up half of an EBHA is not modelled here -- `deep::hydraulics`
        // owns the pressures and `deep::electrical` the buses, and neither
        // publishes an EBHA-local pressure.
        let hydraulic_frac = (truth.hydraulic_pressure_pa[0].max(truth.hydraulic_pressure_pa[1]) / HYDRAULIC_NOMINAL_PA).clamp(0.0, 1.0);
        rev.step(commands.reverser_deploy_commanded[rev.slot], hydraulic_frac, faults, dt);
    }

    // ---- Monitors that `publish` reads. -----------------------------------

    /// The flow the FMU needs the HP pump to deliver to pass the commanded
    /// flow, m^3/s -- what the pump's own low-flow monitor compares against.
    fn required_pump_flow_m3_s(wf_command_kg_s: f64) -> f64 {
        wf_command_kg_s.max(0.0) / super::fuel::common::FUEL_DENSITY_KG_M3
    }

    fn hp_pump_low_flow(&self) -> bool {
        let required = Self::required_pump_flow_m3_s(self.wf_command_kg_s);
        required > 0.0 && self.hp.delivered_m3_s < required * (1.0 - HP_PUMP_LOW_FLOW_TOLERANCE)
    }

    fn metering_error_fraction(&self) -> f64 {
        if self.wf_command_kg_s <= 0.0 {
            return 0.0;
        }
        (self.fmu_state.metered_kg_s - self.wf_command_kg_s).abs() / self.wf_command_kg_s
    }

    /// The worst of one bearing's four defect tones, mm/s -- what a
    /// vibration monitoring unit reports for that bearing.
    fn bearing_amplitude_mm_s(&self, b: usize) -> f64 {
        let s = &self.bearing_state[b].signature;
        s.outer_race.1.max(s.inner_race.1).max(s.ball_spin.1).max(s.cage.1)
    }
}

// ---------------------------------------------------------------------------
// The live system.
// ---------------------------------------------------------------------------

/// The live engine accessories: four engines' worth of everything hung off
/// the accessory gearbox and the nacelle.
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

    /// Air bled overboard by engine `eng`'s two handling bleed valves and
    /// its nacelle anti-ice valve, kg/s -- core air the gas path no longer
    /// has, and the one number `physics::engine`'s compressor model would
    /// need back from this area.
    pub fn bleed_loss_kg_s(&self, eng: usize) -> f64 {
        let c = &self.engines[eng];
        c.handling_bleed_state.iter().map(|b| b.bled_kg_s).sum::<f64>() + c.anti_ice_state.bled_kg_s
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
            // The starter air valve's position monitor needs to know how
            // long the command has been steady, which only the caller of
            // `step` can see across frames.
            let was_commanded = chain.start_valve_commanded_open;
            let now_commanded = truth.controls.starter_engaged[eng];
            chain.start_valve_steady_s = if was_commanded == now_commanded { chain.start_valve_steady_s + dt } else { 0.0 };
            chain.step(eng, truth, faults, &commands, dt);
        }
    }

    fn publish(&self, out: &mut dyn FnMut(&str, f64)) {
        let b = |x: bool| if x { 1.0 } else { 0.0 };

        for chain in self.engines.iter() {
            let n = &chain.names;

            // ---- Fuel filter.
            out(&n.filter_impending_bypass, b(chain.filter.impending_bypass));
            out(&n.filter_bypassed, b(chain.filter.bypassed));
            out(&n.filter_dp_pa, chain.filter.differential_pa);
            out(&n.oil_filter_bypassed, b(chain.oil_filter_bypassed));

            // ---- Fuel pumps.
            out(&n.hp_pump_low_flow, b(chain.hp_pump_low_flow()));
            out(&n.hp_pump_flow, chain.hp.delivered_kg_s);
            out(&n.lp_pump_outlet_pa, chain.lp.outlet_pa);
            out(&n.lp_pump_cavitating, b(chain.lp.cavitating));

            // ---- Metering.
            let error = chain.metering_error_fraction();
            out(&n.fmu_fault, b(error > METERING_TOLERANCE));
            out(&n.thrust_abnormal, b(error > THRUST_ABNORMAL_TOLERANCE));
            out(&n.fmu_metered, chain.fmu_state.metered_kg_s);
            out(&n.fmu_dp_pa, chain.fmu_state.differential_pa);
            out(&n.wf_command, chain.wf_command_kg_s);

            // ---- HP shut-off valve.
            let sov_target = if chain.sov_commanded_open { 1.0 } else { 0.0 };
            out(&n.sov_disagree, b((chain.sov_position - sov_target).abs() > SOV_DISAGREE_TOLERANCE));
            out(&n.sov_position, chain.sov_position);

            // ---- Flow transmitter: channel against channel, and the
            // meter as a whole against what the FMU says it metered.
            out(&n.ff_channel_disagree, b(chain.eec_state.fuel_flow_disagree));
            // What the crew's fuel-flow indication reads: the mean of the
            // transmitter's two pick-off channels.
            let indicated = 0.5 * (chain.flow.channel_a_kg_s + chain.flow.channel_b_kg_s);
            out(&n.ff_disagree, b((indicated - chain.delivered_kg_s).abs() > FF_DISAGREE_KG_S));
            out(&n.ff_indicated, indicated);
            out(&n.ff_true, chain.flow.true_flow_kg_s);

            // ---- Burner manifold.
            out(&n.nozzle_imbalance, b(chain.manifold.hot_streak_severity > NOZZLE_IMBALANCE_SEVERITY));
            out(&n.nozzle_hot_streak, chain.manifold.hot_streak_severity);
            out(&n.manifold_gauge_pa, chain.manifold.manifold_gauge_pa);

            // ---- EEC.
            // "EEC CHANNEL FAULT" is single-channel operation, not "A is
            // not in control": losing the standby channel is the same
            // annunciated condition as losing the controlling one.
            out(&n.eec_channel_fault, b(!(chain.eec_state.channel_a_serviceable && chain.eec_state.channel_b_serviceable)));
            out(&n.eec_no_valid_channel, b(chain.eec_state.active == ActiveChannel::None));
            out(&n.eec_sensor_disagree, b(chain.eec_state.disagree.iter().any(|&d| d)));
            for i in 0..N_EEC_PARAMS {
                out(&n.eec_selected[i], chain.eec_state.selected[i]);
                out(&n.eec_disagree[i], b(chain.eec_state.disagree[i]));
            }

            // ---- Ignition.
            out(&n.ign_spark_rate[0], chain.ignition_state.chain_a_hz);
            out(&n.ign_spark_rate[1], chain.ignition_state.chain_b_hz);
            out(&n.ign_powered, b(chain.ignition_powered));
            out(&n.no_ignition_available, b(chain.ignition_state.no_ignition_available));

            // ---- Starting.
            out(&n.start_valve_position, chain.start_valve_position);
            out(&n.start_valve_disagree, b(chain.start_valve_disagrees()));
            out(&n.starter_torque, chain.ats_state.torque_n_m);
            out(&n.starter_rotor_rpm, chain.ats_state.rotor_rpm);
            out(&n.starter_disengage_fault, b(chain.starter_disengage_fault()));
            out(&n.starter_disintegrated, b(chain.ats_state.disintegrated));
            out(&n.starter_overheat, b(chain.starter_heat.overheated()));
            out(&n.starter_housing_rise_k, chain.starter_heat.rise_k());

            // ---- Compressor variable geometry.
            out(&n.vsv_angle, chain.vsv_state.angle_deg);
            // The alert reads a *magnitude*: the vanes can be off schedule
            // in either direction (a jam at low speed leaves them too
            // closed, a jam at high speed too open) and both cost margin,
            // which is why `VsvState::stall_margin_delta_pct` is symmetric
            // in the error too.
            out(&n.vsv_schedule_error, chain.vsv_state.schedule_error_deg.abs());
            out(&n.vsv_stall_margin, chain.vsv_state.stall_margin_delta_pct);

            // ---- Handling bleeds.
            for i in 0..N_HANDLING_BLEEDS {
                let s = &chain.handling_bleed_state[i];
                out(&n.bleed_position[i], s.position);
                out(&n.bleed_disagree[i], b((s.position - chain.handling_bleed_target[i]).abs() > HANDLING_BLEED_DISAGREE_TOLERANCE));
                out(&n.bleed_flow[i], s.bled_kg_s);
                out(&n.bleed_stall_margin[i], s.stall_margin_delta_pct);
            }

            // ---- Rotor dynamics.
            for s in 0..N_SPOOLS {
                out(&n.vib_index[s], chain.vibration_state[s].index);
            }
            for i in 0..N_BEARINGS {
                out(&n.bearing_amplitude[i], chain.bearing_amplitude_mm_s(i));
                out(&n.bearing_chip[i], b(chain.bearing_state[i].chip_detected));
                out(&n.bearing_debris[i], chain.bearing_state[i].debris_g);
            }

            // ---- Nacelle anti-ice.
            let ai_target = if chain.anti_ice_commanded_open { 1.0 } else { 0.0 };
            out(&n.anti_ice_position, chain.anti_ice_state.position);
            out(&n.anti_ice_disagree, b((chain.anti_ice_state.position - ai_target).abs() > ANTI_ICE_DISAGREE_TOLERANCE));
            out(&n.anti_ice_lip_temp_k, chain.anti_ice_state.lip_k);
            out(&n.anti_ice_bleed_flow, chain.anti_ice_state.bled_kg_s);

            // ---- Nacelle ventilation. The flow is what it is whatever
            // the engine is doing, but the *hazard* the ventilation
            // minimum exists against -- leaked fuel and oil vapour
            // accumulating in the compartment -- needs a running engine
            // to produce the vapour in the first place, which is why a
            // cold aircraft on stand has no ventilation flow and no
            // NACELLE VENT LO caution either.
            out(&n.nacelle_vent_flow, chain.ventilation_state.flow_kg_s);
            out(&n.nacelle_vapour_risk, b(chain.ventilation_state.vapour_accumulation_risk && chain.engine_running));

            // ---- Nacelle fire detection.
            for z in 0..N_FIRE_ZONES {
                out(&n.fire_loop_disagree[z], b(chain.fire_reading[z].loop_disagree));
                out(&n.fire_confirmed[z], b(chain.fire_reading[z].confirmed));
            }

            // ---- Thrust reverser (engines 2 and 3 only).
            if let Some(rev) = &chain.reverser {
                let target = if rev.commanded { 1.0 } else { 0.0 };
                out(&rev.names.position, rev.state.position);
                out(&rev.names.uncommanded, b(rev.state.uncommanded_deployment));
                out(&rev.names.position_disagree, b((rev.state.position - target).abs() > REVERSER_DISAGREE_TOLERANCE));
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
    use crate::deep::live::{Area as _, Controls, PublishedFrame};
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

    /// Four engines running at cruise power, fuel warm, masters on. The
    /// fuel flow is `Truth`'s own, which is what the metering unit is now
    /// asked for.
    fn running() -> (Truth, EngineAccessoryCommands) {
        let truth = Truth {
            dt_s: 0.02,
            engine_running: [true; 4],
            engine_n1_frac: [0.85; 4],
            engine_n2_frac: [0.88; 4],
            engine_n3_frac: [0.9; 4],
            engine_fuel_flow_kg_s: [DESIGN_WF_KG_S; 4],
            engine_bleed_pressure_pa: [300_000.0; 4],
            engine_bleed_temp_k: [520.0; 4],
            engine_hp_port_pressure_pa: [2.5e6; 4],
            engine_hp_port_temp_k: [700.0; 4],
            ac_bus_volts: [115.0; 4],
            hydraulic_pressure_pa: [HYDRAULIC_NOMINAL_PA; 2],
            controls: Controls { engine_master_on: [true; 4], ..Controls::default() },
            ..Truth::default()
        };
        let commands = EngineAccessoryCommands { engine_tgt_k: [900.0; 4], fuel_inlet_k: Some(300.0), ..EngineAccessoryCommands::default() };
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

    /// The registry, built once per test that needs to resolve ids.
    fn registry() -> Registry {
        let mut reg = Registry::default();
        super::super::registry::register(&mut reg);
        reg
    }

    // -----------------------------------------------------------------
    // The fuel chain.
    // -----------------------------------------------------------------

    #[test]
    fn the_governor_command_is_the_engines_own_fuel_flow_not_a_constant_zero() {
        // The whole fuel chain used to meter against a `wf_command` nothing
        // ever set, so `metering_error_fraction` returned 0 whatever the
        // FMU did and every nozzle, metering and flow-transmitter failure
        // was inert. It is `Truth::engine_fuel_flow_kg_s` now.
        let out = settled(&Faults::default());
        for n in 1..=4 {
            assert!((out[&format!("A32NX_ENG_{n}_WF_COMMAND_KG_S")] - DESIGN_WF_KG_S).abs() < 1e-9, "engine {n} should be asked for the engine model's own flow");
        }

        // An engine at a different power setting is asked for a different
        // flow, with nothing overriding anything.
        let (mut truth, commands) = running();
        truth.engine_fuel_flow_kg_s = [0.4, 1.0, 2.0, 3.0];
        let mut live = EngineAccessoriesLive::new();
        live.commands = commands;
        let out = run(&mut live, &truth, &Faults::default(), 30.0);
        for (i, wf) in [0.4, 1.0, 2.0, 3.0].into_iter().enumerate() {
            let n = i + 1;
            assert!((out[&format!("A32NX_ENG_{n}_WF_COMMAND_KG_S")] - wf).abs() < 1e-9);
            assert!((out[&format!("A32NX_ENG_{n}_FMU_METERED_KG_S")] - wf).abs() < 0.05, "engine {n} metered {} against a {wf} command", out[&format!("A32NX_ENG_{n}_FMU_METERED_KG_S")]);
        }
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

    /// Every variable every alert this area owns triggers on must be
    /// published by it -- an alert reading a variable nobody publishes can
    /// never fire, whatever its failures do.
    #[test]
    fn every_variable_this_areas_alerts_trigger_on_is_published() {
        let reg = registry();
        let mut names = Vec::new();
        for alert in &reg.alerts {
            collect_vars(&alert.trigger, &mut names);
            // Procedure lines' own conditions too: a line conditional on a
            // variable nobody publishes can never apply.
            for line in &alert.procedure {
                collect_vars(&line.applies_if, &mut names);
            }
        }
        assert!(names.len() > 100, "this area owns 104 alerts and more procedure conditions; got {}", names.len());
        let out = settled(&Faults::default());
        let mut checked = 0usize;
        for name in names {
            // Cockpit controls and the fire handle belong to the plugin,
            // not to this area.
            if !name.starts_with("A32NX_ENG_") {
                continue;
            }
            assert!(out.contains_key(&name), "an alert in this area reads {name}, which this live system does not publish");
            checked += 1;
        }
        assert!(checked > 100, "only {checked} trigger variables were checked");
    }

    #[test]
    fn a_clogging_filter_warns_before_it_bypasses_and_then_bypasses() {
        // registry.rs: "differential pressure rises until the 35 psi bypass
        // valve cracks; beyond that, unfiltered fuel reaches the HP pump".
        let live = EngineAccessoriesLive::new();
        let id = live.engines[0].fuel_ids.filter_clog;

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
        let id = live.engines[1].fuel_ids.fmu_spill_open;
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
        let id = live.engines[0].fuel_ids.fmu_sticking;
        let out = settled(&Faults::from_pairs([(id, 1.0)]));
        assert!(out["A32NX_ENG_1_FMU_METERED_KG_S"] < 1e-6, "a fully seized valve cannot open");
        assert_eq!(out.get("A32NX_ENG_1_FMU_FAULT"), Some(&1.0));
    }

    #[test]
    fn a_worn_hp_pump_slips_and_eventually_cannot_supply_the_commanded_flow() {
        // registry.rs: "internal slip flow grows as 1/(1-wear)^2; delivered
        // flow falls short of the theoretical displacement flow".
        let live = EngineAccessoriesLive::new();
        let id = live.engines[2].fuel_ids.hp_wear;
        let healthy = settled(&Faults::default());
        let worn = settled(&Faults::from_pairs([(id, 0.9)]));
        assert!(
            worn["A32NX_ENG_3_HP_PUMP_FLOW_KG_S"] < healthy["A32NX_ENG_3_HP_PUMP_FLOW_KG_S"],
            "wear must cost delivered flow: {} vs {}",
            worn["A32NX_ENG_3_HP_PUMP_FLOW_KG_S"],
            healthy["A32NX_ENG_3_HP_PUMP_FLOW_KG_S"]
        );
        let starved = settled(&Faults::from_pairs([(live.engines[2].fuel_ids.hp_starvation, 1.0)]));
        assert_eq!(starved.get("A32NX_ENG_3_HP_PUMP_LOW_FLOW"), Some(&1.0), "a totally starved pump must post its low-flow warning");
        assert_eq!(starved.get("A32NX_ENG_4_HP_PUMP_LOW_FLOW"), Some(&0.0));
    }

    #[test]
    fn a_stuck_hp_shutoff_valve_cannot_follow_the_master_switch() {
        // registry.rs: "stuck open defeats a fire-handle shutdown, stuck
        // closed flames the engine out".
        let (mut truth, commands) = running();
        let mut live = EngineAccessoriesLive::new();
        live.commands = commands;
        let id = live.engines[0].fuel_ids.sov_stuck;

        let open = run(&mut live, &truth, &Faults::default(), 10.0);
        assert!(open["A32NX_ENG_1_HP_SOV_POSITION"] > 0.95);
        assert_eq!(open.get("A32NX_ENG_1_HP_SOV_DISAGREE"), Some(&0.0));
        truth.controls.fire_pb_released[0] = true;
        let shut = run(&mut live, &truth, &Faults::default(), 10.0);
        assert!(shut["A32NX_ENG_1_HP_SOV_POSITION"] < 0.05);

        let (mut truth, commands) = running();
        let mut live = EngineAccessoriesLive::new();
        live.commands = commands;
        run(&mut live, &truth, &Faults::default(), 10.0);
        truth.controls.fire_pb_released[0] = true;
        let seized = run(&mut live, &truth, &Faults::from_pairs([(id, 1.0)]), 10.0);
        assert!(seized["A32NX_ENG_1_HP_SOV_POSITION"] > 0.95, "a seized valve does not move");
        assert_eq!(seized.get("A32NX_ENG_1_HP_SOV_DISAGREE"), Some(&1.0));
    }

    #[test]
    fn a_biased_flow_transmitter_channel_makes_the_two_channels_disagree() {
        let live = EngineAccessoriesLive::new();
        let id = live.engines[3].fuel_ids.ft_a_bias;
        let out = settled(&Faults::from_pairs([(id, 1.0)]));
        assert_eq!(out.get("A32NX_ENG_4_FF_CHANNEL_DISAGREE"), Some(&1.0));
        assert_eq!(out.get("A32NX_ENG_3_FF_CHANNEL_DISAGREE"), Some(&0.0));
        assert!(out["A32NX_ENG_4_FF_INDICATED_KG_S"] > out["A32NX_ENG_3_FF_INDICATED_KG_S"], "a positive bias must read high");
    }

    #[test]
    fn a_frozen_flow_transmitter_channel_stops_tracking_a_real_flow() {
        // Dead before this pass: with `wf_command` stuck at zero there was
        // no flow to stop tracking, so all eight frozen-channel failures
        // moved nothing at all.
        let live = EngineAccessoriesLive::new();
        let id = live.engines[0].fuel_ids.ft_a_frozen;
        let healthy = settled(&Faults::default());
        let frozen = settled(&Faults::from_pairs([(id, 1.0)]));
        assert!(healthy["A32NX_ENG_1_FF_INDICATED_KG_S"] > 0.5 * DESIGN_WF_KG_S);
        assert!(frozen["A32NX_ENG_1_FF_INDICATED_KG_S"] < healthy["A32NX_ENG_1_FF_INDICATED_KG_S"], "a frozen channel drags the mean indication down");
        assert_eq!(frozen.get("A32NX_ENG_1_FF_CHANNEL_DISAGREE"), Some(&1.0));
    }

    #[test]
    fn a_coked_nozzle_group_skews_the_manifold_and_raises_a_hot_streak() {
        // registry.rs: "shrinks that group's orifice area; the shared
        // manifold pressure rises until the other groups pass the
        // difference, raising hot_streak_severity".
        let live = EngineAccessoriesLive::new();
        let id = live.engines[0].fuel_ids.nozzle[3];
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
        let (a, b) = (live.engines[1].fuel_ids.eec_channel_a, live.engines[1].fuel_ids.eec_channel_b);

        let one = settled(&Faults::from_pairs([(a, 1.0)]));
        assert_eq!(one.get("A32NX_ENG_2_EEC_CHANNEL_FAULT"), Some(&1.0));
        assert_eq!(one.get("A32NX_ENG_2_EEC_NO_VALID_CHANNEL"), Some(&0.0), "channel B is still flying the engine");

        // The standby channel dying is the same annunciated condition:
        // the EEC is running single-channel either way. Before this the
        // alert only knew about channel A, so all four channel-B faults
        // moved nothing a pilot could see.
        let standby = settled(&Faults::from_pairs([(b, 1.0)]));
        assert_eq!(standby.get("A32NX_ENG_2_EEC_CHANNEL_FAULT"), Some(&1.0), "ENG 2 EEC CHANNEL FAULT must be reachable from channel B too");
        assert_eq!(standby.get("A32NX_ENG_2_EEC_NO_VALID_CHANNEL"), Some(&0.0));
        assert_eq!(standby.get("A32NX_ENG_1_EEC_CHANNEL_FAULT"), Some(&0.0));

        let both = settled(&Faults::from_pairs([(a, 1.0), (b, 1.0)]));
        assert_eq!(both.get("A32NX_ENG_2_EEC_NO_VALID_CHANNEL"), Some(&1.0));
        assert_eq!(both.get("A32NX_ENG_1_EEC_NO_VALID_CHANNEL"), Some(&0.0));
    }

    #[test]
    fn a_drifting_eec_sensor_is_caught_by_the_channel_disagree_monitor() {
        // registry.rs: "flagged against channel B once the disagreement
        // exceeds this parameter's threshold".
        let live = EngineAccessoriesLive::new();
        let tgt_a = live.engines[0].fuel_ids.eec_sensor_a[3]; // TGT, channel A
        let out = settled(&Faults::from_pairs([(tgt_a, 1.0)]));
        assert_eq!(out.get("A32NX_ENG_1_EEC_SENSOR_DISAGREE"), Some(&1.0));
        assert_eq!(out.get("A32NX_ENG_1_EEC_TGT_DISAGREE"), Some(&1.0));
        assert_eq!(out.get("A32NX_ENG_1_EEC_N1_DISAGREE"), Some(&0.0), "only the failed parameter disagrees");
        assert_eq!(out.get("A32NX_ENG_2_EEC_SENSOR_DISAGREE"), Some(&0.0));
    }

    // -----------------------------------------------------------------
    // Ignition.
    // -----------------------------------------------------------------

    /// A start on the ground: masters on, starters engaged, AC live.
    fn starting_truth() -> Truth {
        Truth {
            dt_s: 0.1,
            on_ground: true,
            engine_running: [false; 4],
            engine_n1_frac: [0.08; 4],
            engine_n2_frac: [0.25; 4],
            engine_n3_frac: [0.22; 4],
            engine_fuel_flow_kg_s: [0.05; 4],
            engine_bleed_pressure_pa: [130_000.0; 4],
            engine_bleed_temp_k: [400.0; 4],
            engine_hp_port_pressure_pa: [200_000.0; 4],
            engine_hp_port_temp_k: [450.0; 4],
            ac_bus_volts: [115.0; 4],
            apu_running: true,
            apu_bleed_pressure_pa: 310_000.0,
            controls: Controls { engine_master_on: [true; 4], starter_engaged: [true; 4], apu_bleed_pb_on: true, ..Controls::default() },
            // What `deep::pneumatic_ducts` would have published last frame:
            // the APU's bleed, at the starter air valve.
            published: PublishedFrame(
                (1..=4)
                    .map(|n| (format!("DEEP_PNEU_ENG_{n}_START_DUCT_PRESSURE_PA"), 310_000.0))
                    .collect(),
            ),
            ..Truth::default()
        }
    }

    fn start_run(faults: &Faults, seconds: f64) -> BTreeMap<String, f64> {
        let truth = starting_truth();
        let mut live = EngineAccessoriesLive::new();
        run(&mut live, &truth, faults, seconds)
    }

    #[test]
    fn both_ignition_chains_spark_during_a_start_and_neither_does_when_nothing_selects_them() {
        let out = start_run(&Faults::default(), 5.0);
        for n in 1..=4 {
            assert_eq!(out.get(&format!("A32NX_ENG_{n}_IGN_POWERED")), Some(&1.0));
            assert!(out[&format!("A32NX_ENG_{n}_IGN_A_SPARK_RATE_HZ")] > 0.0);
            assert!(out[&format!("A32NX_ENG_{n}_IGN_B_SPARK_RATE_HZ")] > 0.0);
            assert_eq!(out.get(&format!("A32NX_ENG_{n}_NO_IGNITION_AVAILABLE")), Some(&0.0));
        }

        // Running at cruise, nothing selecting ignition: the exciters are
        // not energised and there is no spark -- and, with nothing
        // selected, no "no ignition available" either.
        let cruise = settled(&Faults::default());
        assert_eq!(cruise.get("A32NX_ENG_1_IGN_POWERED"), Some(&0.0));
        assert_eq!(cruise.get("A32NX_ENG_1_IGN_A_SPARK_RATE_HZ"), Some(&0.0));
        assert_eq!(cruise.get("A32NX_ENG_1_NO_IGNITION_AVAILABLE"), Some(&0.0));
    }

    #[test]
    fn one_dead_exciter_halves_the_ignition_and_both_dead_raise_the_ignition_fault() {
        // registry.rs: "Charges more slowly (spark rate falls); fully dead,
        // that chain never reaches trigger voltage and never sparks."
        let reg = registry();
        let ids = IgnitionIds::resolve(&reg, 1);

        let one = start_run(&Faults::from_pairs([(ids.exciter[0], 1.0)]), 5.0);
        assert_eq!(one.get("A32NX_ENG_2_IGN_A_SPARK_RATE_HZ"), Some(&0.0));
        assert!(one["A32NX_ENG_2_IGN_B_SPARK_RATE_HZ"] > 0.0, "the other chain is untouched");
        assert_eq!(one.get("A32NX_ENG_2_NO_IGNITION_AVAILABLE"), Some(&0.0), "one chain alone still lights the engine");

        // A degrading (not dead) exciter sparks slower, not just weaker.
        let degraded = start_run(&Faults::from_pairs([(ids.exciter[0], 0.6)]), 5.0);
        let healthy = start_run(&Faults::default(), 5.0);
        assert!(degraded["A32NX_ENG_2_IGN_A_SPARK_RATE_HZ"] > 0.0);
        assert!(degraded["A32NX_ENG_2_IGN_A_SPARK_RATE_HZ"] < healthy["A32NX_ENG_2_IGN_A_SPARK_RATE_HZ"]);

        // ENG 2 IGNITION FAULT triggers on NO_IGNITION_AVAILABLE, and it
        // takes both chains to get there.
        let both = start_run(&Faults::from_pairs([(ids.exciter[0], 1.0), (ids.exciter[1], 1.0)]), 5.0);
        assert_eq!(both.get("A32NX_ENG_2_NO_IGNITION_AVAILABLE"), Some(&1.0));
        assert_eq!(both.get("A32NX_ENG_1_NO_IGNITION_AVAILABLE"), Some(&0.0));
    }

    #[test]
    fn a_fully_eroded_igniter_plug_stops_firing_and_two_of_them_lose_ignition() {
        // registry.rs: "once it exceeds the exciter's fixed peak output,
        // that chain stops firing outright".
        let reg = registry();
        let ids = IgnitionIds::resolve(&reg, 0);
        let one = start_run(&Faults::from_pairs([(ids.igniter[0], 1.0)]), 5.0);
        assert_eq!(one.get("A32NX_ENG_1_IGN_A_SPARK_RATE_HZ"), Some(&0.0));
        assert_eq!(one.get("A32NX_ENG_1_NO_IGNITION_AVAILABLE"), Some(&0.0));
        let both = start_run(&Faults::from_pairs([(ids.igniter[0], 1.0), (ids.igniter[1], 1.0)]), 5.0);
        assert_eq!(both.get("A32NX_ENG_1_NO_IGNITION_AVAILABLE"), Some(&1.0));
    }

    // -----------------------------------------------------------------
    // Starting.
    // -----------------------------------------------------------------

    #[test]
    fn a_healthy_start_opens_the_valve_drives_the_starter_and_reports_no_fault() {
        let out = start_run(&Faults::default(), 10.0);
        for n in 1..=4 {
            assert!(out[&format!("A32NX_ENG_{n}_START_VALVE_POSITION")] > 0.95, "the valve should be open for a start");
            assert_eq!(out.get(&format!("A32NX_ENG_{n}_START_VALVE_DISAGREE")), Some(&0.0));
            assert!(out[&format!("A32NX_ENG_{n}_STARTER_TORQUE_NM")] > 0.0, "the turbine should be driving the spool");
            assert_eq!(out.get(&format!("A32NX_ENG_{n}_STARTER_DISENGAGE_FAULT")), Some(&0.0));
            assert_eq!(out.get(&format!("A32NX_ENG_{n}_STARTER_DISINTEGRATED")), Some(&0.0));
            assert_eq!(out.get(&format!("A32NX_ENG_{n}_STARTER_OVERHEAT")), Some(&0.0));
            assert!(out[&format!("A32NX_ENG_{n}_STARTER_HOUSING_RISE_K")] > 0.0, "cranking heats the gearbox");
        }
    }

    #[test]
    fn a_stuck_starter_air_valve_never_opens_and_the_position_monitor_says_so() {
        // registry.rs: "stuck closed gives no start air at all".
        let reg = registry();
        let ids = StartIds::resolve(&reg, 2);
        let out = start_run(&Faults::from_pairs([(ids.sav_stuck, 1.0)]), 10.0);
        assert_eq!(out.get("A32NX_ENG_3_START_VALVE_POSITION"), Some(&0.0));
        assert_eq!(out.get("A32NX_ENG_3_START_VALVE_DISAGREE"), Some(&1.0), "ENG 3 START VALVE FAULT must be reachable");
        assert_eq!(out.get("A32NX_ENG_3_STARTER_TORQUE_NM"), Some(&0.0), "no air, no torque");
        assert_eq!(out.get("A32NX_ENG_4_START_VALVE_DISAGREE"), Some(&0.0));
    }

    #[test]
    fn the_valve_position_monitor_gives_a_healthy_valve_its_own_travel_time() {
        // A 3 s valve against a 3 s ECAM confirmation is too tight to
        // compare raw, so the monitor waits for the command to be steady.
        let truth = starting_truth();
        let mut live = EngineAccessoriesLive::new();
        let early = run(&mut live, &truth, &Faults::default(), 1.0);
        assert!(early["A32NX_ENG_1_START_VALVE_POSITION"] < 0.95, "the valve is still on its way");
        assert_eq!(early.get("A32NX_ENG_1_START_VALVE_DISAGREE"), Some(&0.0), "a valve still travelling is not a faulty valve");
    }

    #[test]
    fn a_clutch_that_will_not_engage_transmits_less_torque_to_the_spool() {
        // registry.rs: "Reduces torque transmitted to the spool during
        // cranking; a hung start with the turbine itself spinning
        // normally."
        let reg = registry();
        let ids = StartIds::resolve(&reg, 0);
        let healthy = start_run(&Faults::default(), 10.0);
        let hung = start_run(&Faults::from_pairs([(ids.clutch_engage, 0.9)]), 10.0);
        assert!(hung["A32NX_ENG_1_STARTER_TORQUE_NM"] < 0.2 * healthy["A32NX_ENG_1_STARTER_TORQUE_NM"], "{} vs {}", hung["A32NX_ENG_1_STARTER_TORQUE_NM"], healthy["A32NX_ENG_1_STARTER_TORQUE_NM"]);
    }

    #[test]
    fn a_clutch_that_will_not_disengage_drags_the_starter_at_spool_speed_and_then_bursts_it() {
        // registry.rs: "Couples spool speed back into the starter rotor
        // past the turbine's own free speed; drags the spool, and if the
        // spool later reaches the FADEC's own N3 overspeed setpoint with
        // the clutch still coupled, disintegrates the starter."
        let reg = registry();
        let ids = StartIds::resolve(&reg, 1);

        // At cruise the spool is well above the starter's own free speed,
        // so a coupled sprag is immediately visible.
        let (truth, commands) = running();
        let mut live = EngineAccessoriesLive::new();
        live.commands = commands;
        let out = run(&mut live, &truth, &Faults::from_pairs([(ids.clutch_disengage, 1.0)]), 2.0);
        assert_eq!(out.get("A32NX_ENG_2_STARTER_DISENGAGE_FAULT"), Some(&1.0), "ENG 2 STARTER FAULT must be reachable");
        assert!(out["A32NX_ENG_2_STARTER_TORQUE_NM"] < 0.0, "a coupled sprag drags the spool, it does not drive it");
        assert_eq!(out.get("A32NX_ENG_1_STARTER_DISENGAGE_FAULT"), Some(&0.0));
        assert_eq!(out.get("A32NX_ENG_2_STARTER_DISINTEGRATED"), Some(&0.0), "0.90 N3 is below the rotor's burst margin");

        // Take-off power with the same fault: the rotor is now forced past
        // its own burst margin.
        let mut takeoff = truth.clone();
        takeoff.engine_n3_frac = [1.0; 4];
        let mut live = EngineAccessoriesLive::new();
        let out = run(&mut live, &takeoff, &Faults::from_pairs([(ids.clutch_disengage, 1.0)]), 2.0);
        assert_eq!(out.get("A32NX_ENG_2_STARTER_DISINTEGRATED"), Some(&1.0));
        assert_eq!(out.get("A32NX_ENG_3_STARTER_DISINTEGRATED"), Some(&0.0), "only the engine whose clutch failed");
    }

    #[test]
    fn continuous_cranking_eventually_overheats_the_starter_and_a_normal_start_does_not() {
        let short = start_run(&Faults::default(), 60.0);
        assert_eq!(short.get("A32NX_ENG_1_STARTER_OVERHEAT"), Some(&0.0), "one normal-length crank must not overheat it");
        let long = start_run(&Faults::default(), 900.0);
        assert_eq!(long.get("A32NX_ENG_1_STARTER_OVERHEAT"), Some(&1.0), "ENG 1 STARTER OVERHEAT must be reachable");
        assert!(long["A32NX_ENG_1_STARTER_HOUSING_RISE_K"] > short["A32NX_ENG_1_STARTER_HOUSING_RISE_K"]);
    }

    // -----------------------------------------------------------------
    // Compressor variable geometry and handling bleeds.
    // -----------------------------------------------------------------

    #[test]
    fn healthy_vanes_and_bleeds_settle_on_their_schedules() {
        let out = settled(&Faults::default());
        for n in 1..=4 {
            assert!(out[&format!("A32NX_ENG_{n}_VSV_SCHEDULE_ERROR_DEG")] < 0.5);
            assert_eq!(out.get(&format!("A32NX_ENG_{n}_IP_HANDLING_BLEED_DISAGREE")), Some(&0.0));
            assert_eq!(out.get(&format!("A32NX_ENG_{n}_HP_HANDLING_BLEED_DISAGREE")), Some(&0.0));
            // At cruise power both handling bleeds are scheduled shut.
            assert!(out[&format!("A32NX_ENG_{n}_IP_HANDLING_BLEED_KG_S")] < 1e-6);
        }
    }

    #[test]
    fn a_jammed_vsv_actuator_drifts_off_schedule_and_costs_stall_margin() {
        // registry.rs: "Freezes the vane ring at whatever angle it was at;
        // the growing schedule error costs stall margin". The alert trips
        // above 10 degrees of error.
        let reg = registry();
        let ids = AirflowIds::resolve(&reg, 0);
        let out = settled(&Faults::from_pairs([(ids.vsv_jam, 1.0)]));
        assert!(out["A32NX_ENG_1_VSV_SCHEDULE_ERROR_DEG"] > 10.0, "ENG 1 VSV FAULT must be reachable: {}", out["A32NX_ENG_1_VSV_SCHEDULE_ERROR_DEG"]);
        assert!(out["A32NX_ENG_1_VSV_STALL_MARGIN_DELTA_PCT"] < -1.0, "margin must be lost, not gained");
        assert!(out["A32NX_ENG_2_VSV_SCHEDULE_ERROR_DEG"] < 0.5, "the other engines stay on schedule");
    }

    #[test]
    fn a_vsv_rigging_error_settles_off_schedule_by_the_bias_and_also_costs_margin() {
        // registry.rs: "A healthy, fully responsive actuator still settles
        // off the true schedule by the bias amount, costing stall margin
        // exactly as a jam does."
        let reg = registry();
        let ids = AirflowIds::resolve(&reg, 3);
        let half = settled(&Faults::from_pairs([(ids.vsv_rigging, 0.5)]));
        let full = settled(&Faults::from_pairs([(ids.vsv_rigging, 1.0)]));
        assert!(half["A32NX_ENG_4_VSV_SCHEDULE_ERROR_DEG"] > 5.0);
        assert!(full["A32NX_ENG_4_VSV_SCHEDULE_ERROR_DEG"] > half["A32NX_ENG_4_VSV_SCHEDULE_ERROR_DEG"], "a bigger rigging error is a bigger schedule error");
        assert!(full["A32NX_ENG_4_VSV_SCHEDULE_ERROR_DEG"] > 10.0, "ENG 4 VSV FAULT must be reachable from a rigging error too");
        assert!(full["A32NX_ENG_4_VSV_STALL_MARGIN_DELTA_PCT"] < half["A32NX_ENG_4_VSV_STALL_MARGIN_DELTA_PCT"]);
    }

    #[test]
    fn a_handling_bleed_jammed_open_at_cruise_keeps_dumping_core_air() {
        // registry.rs: "Jammed open at high power bleeds core air the
        // engine needs (thrust/fuel-air-ratio penalty)".
        let reg = registry();
        for (i, key) in BLEED_KEYS.iter().enumerate() {
            let ids = AirflowIds::resolve(&reg, 0);
            let out = settled(&Faults::from_pairs([(ids.bleed_jam[i], 1.0)]));
            assert_eq!(out.get(&format!("A32NX_ENG_1_{key}_HANDLING_BLEED_DISAGREE")), Some(&1.0), "ENG 1 {key} BLEED VALVE FAULT must be reachable");
            assert!(out[&format!("A32NX_ENG_1_{key}_HANDLING_BLEED_KG_S")] > 0.1, "a valve jammed open at cruise is still bleeding: {}", out[&format!("A32NX_ENG_1_{key}_HANDLING_BLEED_KG_S")]);
            assert_eq!(out.get(&format!("A32NX_ENG_2_{key}_HANDLING_BLEED_DISAGREE")), Some(&0.0));
        }
    }

    // -----------------------------------------------------------------
    // Rotor dynamics.
    // -----------------------------------------------------------------

    #[test]
    fn a_balanced_healthy_engine_reads_no_vibration_and_no_bearing_tone() {
        let out = settled(&Faults::default());
        for n in 1..=4 {
            for s in SPOOL_KEYS {
                assert_eq!(out.get(&format!("A32NX_ENG_{n}_{s}_VIB_INDEX")), Some(&0.0));
            }
            for (b, key) in BEARING_KEYS.iter().enumerate() {
                let upper = key.to_uppercase();
                assert_eq!(out.get(&format!("A32NX_ENG_{n}_{}_{upper}_DEFECT_AMPLITUDE_MM_S", BEARING_VAR_SPOOL[b])), Some(&0.0));
                assert_eq!(out.get(&format!("A32NX_ENG_{n}_{upper}_CHIP_DETECTED")), Some(&0.0));
            }
        }
    }

    #[test]
    fn a_blade_loss_raises_that_spools_vibration_index_past_its_alert_threshold() {
        // registry.rs: "Adds eccentric mass; synchronous vibration rises
        // with the square of shaft speed". ENG n N1 VIB HI trips above 4.
        let reg = registry();
        let ids = RotorIds::resolve(&reg, 0);
        for (s, key) in SPOOL_KEYS.iter().enumerate() {
            let out = settled(&Faults::from_pairs([(ids.imbalance[s][0], 1.0)]));
            assert!(out[&format!("A32NX_ENG_1_{key}_VIB_INDEX")] > 4.0, "ENG 1 {key} VIB HI must be reachable: {}", out[&format!("A32NX_ENG_1_{key}_VIB_INDEX")]);
            for other in SPOOL_KEYS.iter().filter(|o| *o != key) {
                assert_eq!(out.get(&format!("A32NX_ENG_1_{other}_VIB_INDEX")), Some(&0.0), "only the damaged spool vibrates");
            }
            assert_eq!(out.get(&format!("A32NX_ENG_2_{key}_VIB_INDEX")), Some(&0.0));
        }
    }

    #[test]
    fn ice_and_bird_strike_add_eccentric_mass_the_same_way_a_blade_loss_does() {
        let reg = registry();
        let ids = RotorIds::resolve(&reg, 1);
        let blade = settled(&Faults::from_pairs([(ids.imbalance[0][0], 0.5)]));
        let ice = settled(&Faults::from_pairs([(ids.imbalance[0][1], 0.5)]));
        let bird = settled(&Faults::from_pairs([(ids.imbalance[0][2], 0.5)]));
        let v = |m: &BTreeMap<String, f64>| m["A32NX_ENG_2_N1_VIB_INDEX"];
        assert!(v(&blade) > 0.0);
        assert!((v(&ice) - v(&blade)).abs() < 1e-9, "the same eccentric mass gives the same vibration whatever put it there");
        assert!((v(&bird) - v(&blade)).abs() < 1e-9);
    }

    #[test]
    fn each_bearing_defect_rings_on_its_own_bearing_and_trips_its_own_alert() {
        // registry.rs: each spall "rings at" its own defect frequency; the
        // bearing VIB alert trips above 2 mm/s.
        let reg = registry();
        let ids = RotorIds::resolve(&reg, 2);
        for (bi, key) in BEARING_KEYS.iter().enumerate() {
            for d in 0..4 {
                let out = settled(&Faults::from_pairs([(ids.bearing[bi][d], 1.0)]));
                let upper = key.to_uppercase();
                let name = format!("A32NX_ENG_3_{}_{upper}_DEFECT_AMPLITUDE_MM_S", BEARING_VAR_SPOOL[bi]);
                assert!(out[&name] > 2.0, "ENG 3 {key} defect {d} must reach its alert: {}", out[&name]);
                // The other four bearings stay silent.
                for (oj, other) in BEARING_KEYS.iter().enumerate().filter(|(oj, _)| *oj != bi) {
                    let other_name = format!("A32NX_ENG_3_{}_{}_DEFECT_AMPLITUDE_MM_S", BEARING_VAR_SPOOL[oj], other.to_uppercase());
                    assert_eq!(out.get(&other_name), Some(&0.0), "{other} should be silent");
                }
            }
        }
    }

    #[test]
    fn a_sustained_bearing_spall_sheds_metal_until_the_chip_detector_trips() {
        // registry.rs: "also feeds the oil system's chip detector".
        let reg = registry();
        let ids = RotorIds::resolve(&reg, 0);
        let (mut truth, commands) = running();
        // A minute at a time: the detector is a slow integrator, exactly
        // like the real magnetic plug it models.
        truth.dt_s = 60.0;
        let mut live = EngineAccessoriesLive::new();
        live.commands = commands;
        let faults = Faults::from_pairs([(ids.bearing[0][1], 1.0)]);

        let early = run(&mut live, &truth, &faults, 120.0);
        assert_eq!(early.get("A32NX_ENG_1_FAN_FRONT_CHIP_DETECTED"), Some(&0.0), "two minutes is not enough to bridge the gap");
        assert!(early["A32NX_ENG_1_FAN_FRONT_DEBRIS_G"] > 0.0, "but it is already shedding metal");

        let late = run(&mut live, &truth, &faults, 3600.0);
        assert_eq!(late.get("A32NX_ENG_1_FAN_FRONT_CHIP_DETECTED"), Some(&1.0), "ENG 1 FAN FRONT BRG CHIP DET must be reachable");
        assert_eq!(late.get("A32NX_ENG_1_IP_FRONT_CHIP_DETECTED"), Some(&0.0), "only the failed bearing's detector");
        assert!(late["A32NX_ENG_1_FAN_FRONT_DEBRIS_G"] > early["A32NX_ENG_1_FAN_FRONT_DEBRIS_G"]);
    }

    // -----------------------------------------------------------------
    // The nacelle.
    // -----------------------------------------------------------------

    /// Climbing in icing conditions with the nacelle anti-ice selected.
    fn icing_truth() -> Truth {
        let (mut truth, _) = running();
        truth.dt_s = 0.1;
        truth.environment.sat_c = -8.0;
        truth.environment.ambient_pressure_pa = 64_400.0;
        truth.environment.tas_ms = 170.0;
        truth.controls.nacelle_anti_ice_selected = [true; 4];
        truth
    }

    #[test]
    fn a_selected_anti_ice_valve_opens_warms_the_cowl_lip_and_agrees_with_its_command() {
        let truth = icing_truth();
        let mut live = EngineAccessoriesLive::new();
        live.commands = EngineAccessoryCommands { fuel_inlet_k: Some(280.0), ..Default::default() };
        let out = run(&mut live, &truth, &Faults::default(), 60.0);
        for n in 1..=4 {
            assert!(out[&format!("A32NX_ENG_{n}_ANTI_ICE_POSITION")] > 0.95);
            assert_eq!(out.get(&format!("A32NX_ENG_{n}_ANTI_ICE_DISAGREE")), Some(&0.0));
            assert!(out[&format!("A32NX_ENG_{n}_ANTI_ICE_BLEED_KG_S")] > 0.0);
            assert!(out[&format!("A32NX_ENG_{n}_ANTI_ICE_LIP_TEMP_K")] > 273.15, "the lip must be held above freezing: {}", out[&format!("A32NX_ENG_{n}_ANTI_ICE_LIP_TEMP_K")]);
        }
    }

    #[test]
    fn a_stuck_anti_ice_valve_leaves_the_cowl_lip_cold_and_disagrees() {
        // registry.rs: "Stuck closed leaves the cowl lip at ambient with no
        // ice protection".
        let reg = registry();
        let ids = NacelleIds::resolve(&reg, 1);
        let truth = icing_truth();
        let mut live = EngineAccessoriesLive::new();
        live.commands = EngineAccessoryCommands { fuel_inlet_k: Some(280.0), ..Default::default() };
        let out = run(&mut live, &truth, &Faults::from_pairs([(ids.anti_ice_stuck, 1.0)]), 60.0);
        assert_eq!(out.get("A32NX_ENG_2_ANTI_ICE_POSITION"), Some(&0.0));
        assert_eq!(out.get("A32NX_ENG_2_ANTI_ICE_DISAGREE"), Some(&1.0), "ENG 2 ANTI ICE FAULT must be reachable");
        assert!(out["A32NX_ENG_2_ANTI_ICE_LIP_TEMP_K"] < 273.15, "no bleed, no ice protection");
        assert!(out["A32NX_ENG_1_ANTI_ICE_LIP_TEMP_K"] > 273.15, "the other engines are protected");
    }

    #[test]
    fn a_blocked_nacelle_eductor_on_the_ground_reintroduces_the_vapour_risk() {
        // registry.rs: "matters most on the ground where ram gives nothing,
        // and can reintroduce a vapour-accumulation risk".
        let reg = registry();
        let ids = NacelleIds::resolve(&reg, 0);
        let (mut truth, commands) = running();
        truth.on_ground = true;
        truth.environment.tas_ms = 0.0;
        let mut live = EngineAccessoriesLive::new();
        live.commands = commands;
        let healthy = run(&mut live, &truth, &Faults::default(), 1.0);
        assert_eq!(healthy.get("A32NX_ENG_1_NACELLE_VAPOUR_RISK"), Some(&0.0), "the eductor alone clears the minimum");

        let mut live = EngineAccessoriesLive::new();
        let blocked = run(&mut live, &truth, &Faults::from_pairs([(ids.eductor, 1.0)]), 1.0);
        assert_eq!(blocked.get("A32NX_ENG_1_NACELLE_VAPOUR_RISK"), Some(&1.0), "ENG 1 NACELLE VENT LO must be reachable");
        assert_eq!(blocked.get("A32NX_ENG_2_NACELLE_VAPOUR_RISK"), Some(&0.0));
    }

    #[test]
    fn a_cold_aircraft_on_stand_raises_no_nacelle_vent_caution() {
        // Nothing is moving and nothing is running, so there is no
        // ventilation flow -- and no vapour to ventilate either. An
        // ungated risk flag here would put four amber NACELLE VENT LO
        // cautions on the ECAM of every cold aircraft that loads.
        let mut live = EngineAccessoriesLive::new();
        let out = run(&mut live, &Truth { dt_s: 0.5, ..Truth::default() }, &Faults::default(), 30.0);
        for n in 1..=4 {
            assert_eq!(out.get(&format!("A32NX_ENG_{n}_NACELLE_VENT_FLOW_KG_S")), Some(&0.0));
            assert_eq!(out.get(&format!("A32NX_ENG_{n}_NACELLE_VAPOUR_RISK")), Some(&0.0));
        }
    }

    #[test]
    fn a_blocked_nacelle_scoop_costs_ram_ventilation_in_flight() {
        let reg = registry();
        let ids = NacelleIds::resolve(&reg, 2);
        let truth = icing_truth();
        let mut live = EngineAccessoriesLive::new();
        let healthy = run(&mut live, &truth, &Faults::default(), 1.0);
        let mut live = EngineAccessoriesLive::new();
        let blocked = run(&mut live, &truth, &Faults::from_pairs([(ids.scoop, 1.0)]), 1.0);
        assert!(healthy["A32NX_ENG_3_NACELLE_VENT_FLOW_KG_S"] > blocked["A32NX_ENG_3_NACELLE_VENT_FLOW_KG_S"], "a blocked scoop must cost ram flow");
        assert!(blocked["A32NX_ENG_3_NACELLE_VENT_FLOW_KG_S"] > 0.0, "the eductor still runs");
    }

    #[test]
    fn a_single_false_tripping_fire_loop_disagrees_without_declaring_a_fire() {
        // registry.rs: "A lone false trip cannot confirm a fire in a cool
        // zone; flagged as a loop disagree."
        let reg = registry();
        let ids = NacelleIds::resolve(&reg, 3);
        for (z, zone) in FIRE_ZONE_KEYS.iter().enumerate() {
            // Loop A false trip, then loop B's.
            for l in [1usize, 3] {
                let out = settled(&Faults::from_pairs([(ids.fire[z][l], 1.0)]));
                assert_eq!(out.get(&format!("A32NX_ENG_4_{zone}_FIRE_LOOP_DISAGREE")), Some(&1.0), "ENG 4 {zone} FIRE DET FAULT must be reachable");
                assert_eq!(out.get(&format!("A32NX_ENG_4_{zone}_FIRE_CONFIRMED")), Some(&0.0), "one loop cannot declare a fire");
                assert_eq!(out.get(&format!("A32NX_ENG_3_{zone}_FIRE_LOOP_DISAGREE")), Some(&0.0));
            }
        }
    }

    #[test]
    fn a_hot_nacelle_zone_confirms_a_fire_and_a_broken_loop_stops_it_confirming() {
        // The loops sense `deep::thermal_zones`' own nacelle cowl zone
        // temperature, one frame behind. A loop that fails to detect is
        // correctly invisible until there is something to detect -- which
        // is the whole reason the second loop exists.
        let reg = registry();
        let ids = NacelleIds::resolve(&reg, 0);
        let (mut truth, commands) = running();
        let hot_c = fire_detection::TRIP_K - 273.15 + 50.0;
        truth.published = PublishedFrame((1..=4).map(|n| (format!("THERMAL_ZONE_NACELLECOWL{n}_TEMPERATURE_C"), hot_c)).collect());

        let mut live = EngineAccessoriesLive::new();
        live.commands = commands;
        let out = run(&mut live, &truth, &Faults::default(), 1.0);
        assert_eq!(out.get("A32NX_ENG_1_CORE_FIRE_CONFIRMED"), Some(&1.0), "both healthy loops above the trip temperature confirm");
        assert_eq!(out.get("A32NX_ENG_1_CORE_FIRE_LOOP_DISAGREE"), Some(&0.0));

        let mut live = EngineAccessoriesLive::new();
        let broken = run(&mut live, &truth, &Faults::from_pairs([(ids.fire[0][0], 1.0)]), 1.0);
        assert_eq!(broken.get("A32NX_ENG_1_CORE_FIRE_CONFIRMED"), Some(&0.0), "a broken loop A leaves the fire unconfirmed");
        assert_eq!(broken.get("A32NX_ENG_1_CORE_FIRE_LOOP_DISAGREE"), Some(&1.0), "and the disagreement is what the crew is told");
    }

    // -----------------------------------------------------------------
    // The thrust reverser.
    // -----------------------------------------------------------------

    #[test]
    fn only_the_inboard_engines_have_a_reverser_at_all() {
        let out = settled(&Faults::default());
        for n in [2, 3] {
            assert!(out.contains_key(&format!("A32NX_ENG_{n}_REV_POSITION")), "engine {n} carries a reverser");
        }
        for n in [1, 4] {
            assert!(!out.contains_key(&format!("A32NX_ENG_{n}_REV_POSITION")), "the A380 has no reverser on engine {n}");
        }
    }

    #[test]
    fn every_lock_failing_to_hold_at_once_deploys_the_sleeve_uncommanded() {
        // registry.rs: "all three failing this way together is what an
        // uncommanded deployment actually requires". This is the one
        // reverser path that needs no deploy command, and it is the one
        // that raises a red warning.
        let reg = registry();
        let ids = ReverserIds::resolve(&reg, 2);
        let (truth, commands) = running();

        let mut live = EngineAccessoriesLive::new();
        live.commands = commands;
        let one = run(&mut live, &truth, &Faults::from_pairs([(ids.lock_hold[0], 1.0)]), 60.0);
        assert_eq!(one.get("A32NX_ENG_2_REV_UNCOMMANDED"), Some(&0.0), "one failed lock: the other two still hold");
        assert_eq!(one.get("A32NX_ENG_2_REV_POSITION"), Some(&0.0));

        let mut live = EngineAccessoriesLive::new();
        live.commands = commands;
        let all = run(&mut live, &truth, &Faults::from_pairs([(ids.lock_hold[0], 1.0), (ids.lock_hold[1], 1.0), (ids.lock_hold[2], 1.0)]), 60.0);
        assert_eq!(all.get("A32NX_ENG_2_REV_UNCOMMANDED"), Some(&1.0), "ENG 2 REVERSER UNLOCKED must be reachable");
        assert_eq!(all.get("A32NX_ENG_2_REV_POSITION_DISAGREE"), Some(&1.0));
        assert_eq!(all.get("A32NX_ENG_3_REV_UNCOMMANDED"), Some(&0.0));
    }

    #[test]
    fn a_jammed_lock_blocks_a_commanded_deployment_and_the_position_disagrees() {
        // registry.rs: "Blocks legitimate deployment outright (every lock
        // must release)". Nothing in `Truth` commands reverse yet, so the
        // command comes from `EngineAccessoryCommands` here -- the one
        // input this area is still waiting for, wired and tested so that
        // the day it arrives nothing else has to change.
        let reg = registry();
        let ids = ReverserIds::resolve(&reg, 3);
        let (truth, commands) = running();
        let deploying = EngineAccessoryCommands { reverser_deploy_commanded: [true, true], ..commands };

        let mut live = EngineAccessoriesLive::new();
        live.commands = deploying;
        let healthy = run(&mut live, &truth, &Faults::default(), 10.0);
        assert!(healthy["A32NX_ENG_3_REV_POSITION"] > 0.95, "a healthy reverser deploys when commanded");
        assert_eq!(healthy.get("A32NX_ENG_3_REV_POSITION_DISAGREE"), Some(&0.0));

        for lock in 0..N_REV_LOCKS {
            let mut live = EngineAccessoriesLive::new();
            live.commands = deploying;
            let jammed = run(&mut live, &truth, &Faults::from_pairs([(ids.lock_jam[lock], 1.0)]), 10.0);
            assert_eq!(jammed.get("A32NX_ENG_3_REV_POSITION"), Some(&0.0), "lock {lock} jammed must block deployment");
            assert_eq!(jammed.get("A32NX_ENG_3_REV_POSITION_DISAGREE"), Some(&1.0), "ENG 3 REVERSER FAULT must be reachable");
        }

        let mut live = EngineAccessoriesLive::new();
        live.commands = deploying;
        let seized = run(&mut live, &truth, &Faults::from_pairs([(ids.actuator_jam, 1.0)]), 10.0);
        assert_eq!(seized.get("A32NX_ENG_3_REV_POSITION"), Some(&0.0));
        assert_eq!(seized.get("A32NX_ENG_3_REV_POSITION_DISAGREE"), Some(&1.0));
    }

    // -----------------------------------------------------------------
    // Coverage and cost.
    // -----------------------------------------------------------------

    #[test]
    fn nothing_divides_by_zero_on_four_cold_engines_at_zero_dt() {
        let mut live = EngineAccessoriesLive::new();
        live.tick(&Truth { dt_s: 0.0, ..Truth::default() }, &Faults::default());
        for (name, value) in published(&live) {
            assert!(value.is_finite(), "{name} is not finite");
        }
    }

    /// Every failure this area registers is read by the live system, and
    /// nothing it reads is unregistered.
    #[test]
    fn every_registered_failure_in_this_area_is_consumed_by_the_live_system() {
        let reg = registry();
        let live = EngineAccessoriesLive::new();

        let mut consumed: Vec<u64> = live.engines.iter().flat_map(|c| c.consumed_ids()).collect();
        consumed.sort_unstable();
        let before = consumed.len();
        consumed.dedup();
        assert_eq!(before, consumed.len(), "the live system resolved the same failure id twice");

        let registered: std::collections::BTreeSet<u64> = reg.failures.iter().map(|f| f.id).collect();
        for id in &consumed {
            assert!(registered.contains(id), "the live system reads {id}, which this area does not register");
        }
        for f in &reg.failures {
            assert!(consumed.contains(&f.id), "{} ({}, ata {}, {}) is registered but never read by the live system", f.id, f.name, f.ata, f.model_field);
        }
        assert_eq!(consumed.len(), registered.len());

        let mut ata: Vec<u16> = reg.failures.iter().map(|f| f.ata).collect();
        ata.sort_unstable();
        ata.dedup();
        assert_eq!(ata, [26, 30, 71, 72, 73, 74, 75, 77, 78, 80], "this area's chapters");
        assert!(UNCONSUMED_ATA.is_empty(), "every chapter is stepped now");
    }

    #[test]
    fn resolving_the_ids_twice_gives_the_same_answer() {
        let reg = registry();
        let reg2 = registry();
        for eng in 0..N_ENGINES {
            assert_eq!(FuelIds::resolve(&reg, eng).filter_clog, FuelIds::resolve(&reg2, eng).filter_clog);
            assert_eq!(RotorIds::resolve(&reg, eng).bearing[2][1], RotorIds::resolve(&reg2, eng).bearing[2][1]);
        }
    }

    /// What four engines' worth of accessories cost per frame.
    ///
    /// Printed rather than pinned to a number -- the absolute figure is the
    /// machine's, not the code's -- but bounded well below a frame at 60 Hz
    /// so that a change that made this area quadratic could not pass.
    #[test]
    fn four_engines_of_accessories_fit_comfortably_inside_a_frame() {
        let (truth, commands) = running();
        let mut live = EngineAccessoriesLive::new();
        live.commands = commands;
        let faults = Faults::default();

        // Warm up: the first frames allocate the published-name map in the
        // caller, not here, but the models' own first steps are cold.
        for _ in 0..100 {
            live.tick(&truth, &faults);
        }

        let frames = 2_000;
        let t0 = std::time::Instant::now();
        let mut sink = 0.0;
        for _ in 0..frames {
            live.tick(&truth, &faults);
            live.publish(&mut |_, v| sink += v);
        }
        let per_frame_us = t0.elapsed().as_secs_f64() * 1e6 / frames as f64;
        println!("engine_accessories: {per_frame_us:.1} us per frame (tick + publish, 4 engines), sink {sink}");
        assert!(per_frame_us < 1_000.0, "{per_frame_us:.1} us per frame is more than 6% of a 60 Hz frame");
    }
}
