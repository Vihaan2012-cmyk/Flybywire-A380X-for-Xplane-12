//! The live pneumatic duct system: one owned [`DuctNetwork`] stepped
//! every frame from [`Truth`], with every failure [`super::registry`]
//! registers driving the exact `DuctNetworkFaults` field that registry
//! entry names, and every variable its ECAM triggers read published back
//! out.
//!
//! Before this file the network was a type nothing instantiated: no duct
//! anywhere in the plugin held gas, `DEEP_PNEU_ODLS_*_TRIP` did not
//! exist, and the `AIR ENG n BLEED LEAK`/`AIR ENG n PRECOOLER OVHT`
//! alerts hanging off those names could never fire.
//!
//! ## How `Truth` drives it
//! - **Ambient**: `environment.ambient_pressure_pa` and `sat_c` are the
//!   pressure and temperature every leak discharges into, every relief
//!   valve references and every duct conducts to.
//! - **Engine bleed ports**: `engine_ip_port_pressure_pa`/`engine_ip_
//!   port_temp_k` are the IP8 tap's own upstream condition, read
//!   *unconditionally* -- not `engine_bleed_pressure_pa`/`_temp_k`, which
//!   already carry whichever of IP8/HP6 `engine_commands.rs:466`'s own
//!   switch picked for the customer bleed this tick (`deep::live`'s own
//!   doc on that pair) and so read as HP6's real hot condition whenever
//!   that *other*, unrelated switch has the shallow engine model's HP
//!   valve open. This area runs its own independent IP8-tap/HP-valve/
//!   PR-valve model with its own actuator lag (`network.rs`); feeding it
//!   the pre-switched pair used to double-switch the port and let a real
//!   HP6-hot slug reach the passive IP tap (`duct::passive_valve_open_
//!   fraction`, no lag of its own) unannounced -- the cause of a ~400 C
//!   all-engine precooler-outlet spike at TOGA, fixed in W91
//!   (`E:/fbw-debug/fixes/W91.md`). `engine_hp_port_pressure_pa`/`_temp_k`
//!   are the HP6 tap's own, read *unconditionally* (module doc on that
//!   pair in `deep::live`), so the HP valve's own stuck-valve failure
//!   (15_036_015) and the precooler's real hot source are both reachable
//!   now -- previously this branch was fed a fixed
//!   [`HP_PORT_UNAVAILABLE_PA`] (zero), which correctly held the valve
//!   shut but meant nothing behind it could ever be exercised.
//! - **Precooler cooling air**: the precooler is an air-to-air exchanger
//!   against engine fan-duct air. `Truth` has no bypass mass flow, so it
//!   is derived from `engine_n1_frac` and the ambient density at
//!   [`TRENT_900_BYPASS_MDOT_SLS_KG_S`] (public Trent 900 sea-level-static
//!   figures) -- a real relation between a real `Truth` input and the
//!   quantity the model needs, not a stand-in constant. The engine's own
//!   `bypass_mdot_kg_s` in `Truth` would be strictly better (still not
//!   sourced, `docs/deep/truth-requests.md`).
//! - **APU**: `apu_running` plus `apu_bleed_pressure_pa`, gated by the real
//!   `controls.apu_bleed_pb_on` pushbutton. The APU's bleed *temperature*
//!   is not in `Truth`, so it is computed from the pressure ratio the load
//!   compressor is actually achieving against ambient
//!   ([`APU_LOAD_COMPRESSOR_POLYTROPIC_EFFICIENCY`]) -- thermodynamics on
//!   a real input rather than a chosen number.
//! - **Cockpit controls** (`truth.controls`, real this pass): pack
//!   pushbuttons (both feed valves of a pack open together, since only the
//!   pushbutton is a real crew control -- the flow-control valve itself is
//!   this area's own modelled component, `docs/deep/truth-requests.md`),
//!   wing anti-ice selection (one pushbutton, both sides), the cross-bleed
//!   selector (raw 0 SHUT/1 AUTO/2 OPEN), the engine bleed pushbuttons
//!   (this area's own `network::NetworkInputs::engine_bleed_pb_auto`, the
//!   real shutoff the "ENG n BLEED" pushbutton *is*) and starter
//!   engagement, replacing the interim `ControlAssumptions` struct this
//!   area used to run on entirely (packs always open, wing anti-ice always
//!   off, starters always off, cross-bleed opened only by the same
//!   heuristic this area still uses for the selector's own AUTO position,
//!   since FBW's real AUTO logic is flight-deck software out of this plant
//!   model's scope, `network.rs`'s own module doc).
//! - **Zone air temperatures.** ODLS watches the temperature of the bay
//!   each duct runs through, which is `deep::thermal_zones`' own output.
//!   `Truth::published` now carries it (this pass's own contract fix, see
//!   `docs/deep/truth-requests.md`'s "Contract gap" section), so this area
//!   reads `THERMAL_ZONE_<NAME>_TEMPERATURE_C` back through
//!   `truth.published.get_or(...)` with the recovery temperature as the
//!   fallback for a frame nothing has published yet -- closing the leak ->
//!   bay overheat -> isolation chain this area exists for, previously cut
//!   in the middle because a duct leak's own heat (published here as
//!   `DEEP_PNEU_ZONE_<zone>_HEAT_W`) could never come back round to the
//!   loop that should trip on it.

use super::duct::DuctSectionFaults;
use super::network::{ApuBleedInput, DuctNetwork, DuctNetworkFaults, EngineBleedInput, NetworkInputs, NetworkOutputs, ODLS_ZONE_COUNT, ZONE_COUNT, ZONE_NAMES};
use super::odls::OdlsFaults;
use super::precooler::PrecoolerFaults;
use crate::deep::api::{failure_id, Area as RegArea};
use crate::deep::live::{DerivedFailure, Faults, Truth};

/// Ratio of specific heats for air, standard.
const GAMMA_AIR: f64 = 1.4;
/// ICAO standard atmosphere sea-level density, kg/m^3 (ICAO Doc 7488).
const SEA_LEVEL_DENSITY_KG_M3: f64 = 1.225;
/// Specific gas constant for dry air, J/(kg K).
const R_AIR_J_KG_K: f64 = 287.057_005;

/// Trent 900 sea-level-static bypass mass flow, kg/s. Public engine data
/// for the Trent 970/972 family gives about 1204 kg/s total intake flow at
/// take-off against a bypass ratio of about 8.7, i.e. 1204 * 8.7/9.7 of it
/// through the fan duct. The precooler's cooling air is a small bleed off
/// exactly that stream.
const TRENT_900_BYPASS_MDOT_SLS_KG_S: f64 = 1204.0 * 8.7 / 9.7;

/// GENERIC bay ventilation mass flow, kg/s, used by [`PneumaticDuctsLive::
/// own_zone_excess_k`] to translate this area's own duct-leak heat into a
/// local temperature excess. Reuses `thermal_zones::PROGRESS.md`'s own
/// cited 0.5 kg/s pylon-bay figure (CS/FAR 25.1187 fire-zone ventilation
/// minimum), applied to every ODLS zone alike for lack of a more specific
/// per-zone figure -- see that function's own doc.
const ZONE_VENTILATION_KG_S: f64 = 0.5;
/// Specific heat of air at the temperatures these bays run at, J/(kg*K).
const CP_AIR_J_KGK: f64 = 1005.0;
/// Time constant [`PneumaticDuctsLive::relax_own_zone_excess`] relaxes
/// toward its steady-state target over, s. **GENERIC**: order-of-magnitude
/// for a several-cubic-metre ventilated bay's own air thermal mass against
/// its ventilation flow (`m/mdot_vent`; a ~5 m^3 bay at typical density is
/// a few kg of air against 0.5 kg/s ventilation, single-digit seconds --
/// rounded up for stability margin, not for a closer physical match).
const ZONE_EXCESS_TIME_CONSTANT_S: f64 = 10.0;

/// Cooling air the APU's own fan/load-compressor stream makes available to
/// its precooler, kg/s, while it runs. **GENERIC**: no public figure
/// exists. Derived by the requirement a precooler has to meet to cool at
/// all -- its fan-air valve bleeds off 2% of the available stream at full
/// opening (`precooler::FAN_AIR_BLEED_OFF_FRACTION_AT_FULL_OPEN`), so the
/// stream must be about two orders of magnitude above the APU's own bleed
/// flow (order 0.5 kg/s through its bleed valve) for the cold side to
/// match the hot side.
const APU_COOLING_AIR_KG_S: f64 = 30.0;

/// Polytropic efficiency of the APU load compressor, used to get its
/// discharge temperature from the pressure ratio it is actually achieving
/// (module doc). **GENERIC**: 0.8 is the ordinary range for a single-stage
/// centrifugal load compressor; no PW980-specific figure is public.
const APU_LOAD_COMPRESSOR_POLYTROPIC_EFFICIENCY: f64 = 0.8;

fn f(ata: u16, n: u16) -> u64 {
    failure_id(RegArea::PneumaticDucts, ata, n)
}

// ---------------------------------------------------------------------
// Authority: this area's level-2 couplings into FlyByWire's own failures
// (`docs/deep/authority.md`).
//
// This is the overlap `authority.md` calls the most delicate of the three,
// and it turns out to be the narrowest. FlyByWire's A380 pneumatic system
// models eight bleed valves this model also has -- each engine's HP valve
// and its pressure-regulating (shut-off) valve -- and takes a *continuous*
// seizure input for each, `PNEU_VALVE_FAILED:n` (`a380_systems/src/
// pneumatic.rs:53-90`'s `ValveSeizure`: "a seized valve loses that fraction
// of its authority, measured from where it stood when it seized"). That is
// the same physical fault this model's own `upstream[n].hp_valve_stuck`/
// `pr_valve_stuck` is, with the same meaning and the same 0..1 scale, so
// the verdict passes across unrounded -- the one place in these three
// systems where level 2 keeps the deep model's own granularity.
//
// Everything else in this area is level 1. FlyByWire has no duct, no leak,
// no rupture, no insulation, no precooler, no overheat-detection loop and
// no bay temperature, so nothing competes with what this area publishes.
// The one genuine gap is in the other direction and is recorded in
// `authority.md`: when this area's ODLS trips and latches an engine's
// bleed isolated, there is no FlyByWire input that *shuts* a valve --
// seizure freezes a valve where it stands, which for an open valve would
// leave FlyByWire bleeding from an engine this model has isolated. Seizing
// it would not be the same statement, so it is not made.

/// The extra catalogue's "Engine n HP bleed valve stuck"
/// (`crate::failures::extra`, `PNEUMATIC_VALVES` valves 1-4).
const FBW_HP_VALVE: [u64; 4] = [36_008, 36_009, 36_010, 36_011];
/// "Engine n bleed valve stuck" -- the pressure regulating/shut-off valve
/// (`PNEUMATIC_VALVES` valves 5-8).
const FBW_PR_VALVE: [u64; 4] = [36_012, 36_013, 36_014, 36_015];

/// The extra catalogue's own "Engine n precooler fault" ids (36_004-
/// 36_007, `Effect::Hook { var: "FAIL_PRECOOLER_HOOK", owner: Owner::Air }`)
/// -- a crew-armable failure from the Study Failures page, with no
/// consumer at all until now (W50/W109). Independent of, and in a
/// different id space from, this area's own deep-catalogue precooler-
/// fouling id (`f(36, 4)`, `deep::api::failure_id`-hashed, armed from the
/// Study panel's separate deep-failures view and already covered by
/// `arming_precooler_fouling_leaves_the_delivered_bleed_hotter` below) --
/// the same "two independent sources of the same mechanism" relationship
/// `physics::bays.rs`'s `ENGINE_BLEED_LEAK_FAILURE_IDS` already has with
/// this area's own 36_000-36_003 duct-leak modelling.
const EXTRA_PRECOOLER_FAULT_IDS: [u64; 4] = [36_004, 36_005, 36_006, 36_007];

/// The registry component behind all eight. `registry.rs` catalogues one
/// id per distinct fault mechanism on a component *class* (its own module
/// doc: "x4 engines"), so the four engines share one component and one
/// deep failure id each for the HP and the PR valve -- which is why arming
/// either seizes that valve on all four engines, here and on FlyByWire's
/// side alike. Per-instance arming needs per-instance ids; the same gap
/// `apply_faults` already records.
const UPSTREAM_VALVE_COMPONENT: &str = "36_pneu.engine_upstream_valve_stage";

/// Why, per engine, for each half of the table.
const HP_VALVE_REASON: [&str; 4] = [
    "engine 1 HP bleed valve seized at its last position",
    "engine 2 HP bleed valve seized at its last position",
    "engine 3 HP bleed valve seized at its last position",
    "engine 4 HP bleed valve seized at its last position",
];
const PR_VALVE_REASON: [&str; 4] = [
    "engine 1 bleed PR/shutoff valve seized at its last position",
    "engine 2 bleed PR/shutoff valve seized at its last position",
    "engine 3 bleed PR/shutoff valve seized at its last position",
    "engine 4 bleed PR/shutoff valve seized at its last position",
];

/// Every level-2 coupling this area owns, as `(FlyByWire failure id, deep
/// component)`, in the order [`PneumaticDuctsLive::each_coupling`] emits
/// them.
fn coupling_table() -> Vec<(u64, &'static str)> {
    let mut v: Vec<(u64, &'static str)> = Vec::new();
    for i in 0..4 {
        v.push((FBW_HP_VALVE[i], UPSTREAM_VALVE_COMPONENT));
    }
    for i in 0..4 {
        v.push((FBW_PR_VALVE[i], UPSTREAM_VALVE_COMPONENT));
    }
    v
}

/// Every variable name this area publishes, built once (the `Area` trait
/// publishes by `&str` and these names never change).
struct VarNames {
    odls_trip: [String; ODLS_ZONE_COUNT],
    odls_fault: [String; ODLS_ZONE_COUNT],
    /// Each loop's own health, published separately from the aggregate
    /// `odls_fault` so a single loop's `loop_a_open`/`loop_b_open` failure
    /// is visible on its own -- see `odls::OdlsOutputs`'s own doc for why
    /// the aggregate alone cannot show it (the same dual-loop masking
    /// `deep::fire_ice::fire_loops` already documents for its A/B loops).
    odls_loop_a_fault: [String; ODLS_ZONE_COUNT],
    odls_loop_b_fault: [String; ODLS_ZONE_COUNT],
    zone_heat_w: [String; ZONE_COUNT],
    zone_jet_flux: [String; ZONE_COUNT],
    engine_precooler_ovht: [String; 4],
    engine_precooler_outlet_c: [String; 4],
    engine_isolation_open: [String; 4],
    engine_duct_pressure: [String; 4],
    engine_duct_temp_c: [String; 4],
    transfer_pipe_pressure: [String; 4],
    hp_valve_open: [String; 4],
    pr_valve_open: [String; 4],
    start_duct_pressure: [String; 4],
    pack_supply_pressure: [String; 2],
    pack_supply_temp_c: [String; 2],
    wai_duct_pressure: [String; 2],
    wai_duct_temp_c: [String; 2],
    wai_valve_open: [String; 2],
    hyd_reservoir_pressure: [String; 2],
    cross_bleed_open: [String; 3],
    // ---- ECAM-completeness additions (E-AIR-DESIGN.md, ATA 21 AIR/PRESS).
    // Independent LRU-style faults: each is a component that is broken or
    // not, no threshold invented, the same shape `fbw/ata24.rs`'s
    // `ELEC_GEN_n_FAULT` uses. They do not participate in the duct network
    // solve, so they live as their own fields rather than inside
    // `DuctNetworkFaults`/`NetworkOutputs`.
    pack_regul_fault: [String; 2],
    mixer_press_regul_fault: String,
    ram_air_door_fault: [String; 2],
    press_man_ctl_fault: String,
    cabin_air_extract_vlv_fault: String,
    /// 211800022: exactly one of a pack's two FDAC channels down (both down
    /// is 211800009/010, already wired). Bridged from FlyByWire's own real
    /// per-channel discretes (`Truth::fdac_channel_failure`), not a new
    /// failure.
    pack_regul_redundancy_fault: String,
    /// 213800015: all four OCSMs' own `BothChannelsFault` together.
    /// Bridged from FlyByWire's own real per-channel discretes
    /// (`Truth::ocsm_channel_failure`).
    outflw_vlv_ctl_fault_all: String,
    /// 211800013/014: each pack's own air cycle machine outlet temperature,
    /// C. Real physics on a real input (see [`PneumaticDuctsLive::tick`]'s
    /// own comment) rather than a bare boolean, so the FCOM's real 95 C
    /// trip (`E-AIR-FCOM.json` 211800013, FCOM p.4653) is applied in
    /// `fbw/ata21_22_23.rs`, the same way `ac_dead`'s 90 V is applied in
    /// `fbw/ata24.rs` rather than pre-baked into a boolean here.
    pack_acm_outlet_temp_c: [String; 2],
    /// 211800017-020: each pack's two flow-control valves (FCVs), one
    /// binary LRU fault each -- see [`PneumaticDuctsLive::tick`]'s own
    /// comment for why this is our own component rather than a bridge.
    pack_fcv_fault: [[String; 2]; 2],
}

fn per_engine(fmt: impl Fn(usize) -> String) -> [String; 4] {
    std::array::from_fn(|i| fmt(i + 1))
}

impl VarNames {
    fn new() -> Self {
        let side = ["L", "R"];
        let hyd = ["GREEN", "YELLOW"];
        let xbleed = ["L", "C", "R"];
        Self {
            odls_trip: std::array::from_fn(|z| format!("DEEP_PNEU_ODLS_{}_TRIP", ZONE_NAMES[z])),
            odls_fault: std::array::from_fn(|z| format!("DEEP_PNEU_ODLS_{}_FAULT", ZONE_NAMES[z])),
            odls_loop_a_fault: std::array::from_fn(|z| format!("DEEP_PNEU_ODLS_{}_LOOP_A_FAULT", ZONE_NAMES[z])),
            odls_loop_b_fault: std::array::from_fn(|z| format!("DEEP_PNEU_ODLS_{}_LOOP_B_FAULT", ZONE_NAMES[z])),
            zone_heat_w: std::array::from_fn(|z| format!("DEEP_PNEU_ZONE_{}_HEAT_W", ZONE_NAMES[z])),
            zone_jet_flux: std::array::from_fn(|z| format!("DEEP_PNEU_ZONE_{}_JET_FLUX_W_M2", ZONE_NAMES[z])),
            engine_precooler_ovht: per_engine(|n| format!("DEEP_PNEU_ENG_{n}_PRECOOLER_OVHT")),
            engine_precooler_outlet_c: per_engine(|n| format!("DEEP_PNEU_ENG_{n}_PRECOOLER_OUTLET_C")),
            engine_isolation_open: per_engine(|n| format!("DEEP_PNEU_ENG_{n}_ISOLATION_OPEN")),
            engine_duct_pressure: per_engine(|n| format!("DEEP_PNEU_ENG_{n}_DUCT_PRESSURE_PA")),
            engine_duct_temp_c: per_engine(|n| format!("DEEP_PNEU_ENG_{n}_DUCT_TEMPERATURE_C")),
            transfer_pipe_pressure: per_engine(|n| format!("DEEP_PNEU_ENG_{n}_TRANSFER_PRESSURE_PA")),
            hp_valve_open: per_engine(|n| format!("DEEP_PNEU_ENG_{n}_HP_VALVE_OPEN")),
            pr_valve_open: per_engine(|n| format!("DEEP_PNEU_ENG_{n}_PR_VALVE_OPEN")),
            start_duct_pressure: per_engine(|n| format!("DEEP_PNEU_ENG_{n}_START_DUCT_PRESSURE_PA")),
            pack_supply_pressure: std::array::from_fn(|i| format!("DEEP_PNEU_PACK_{}_SUPPLY_PRESSURE_PA", i + 1)),
            pack_supply_temp_c: std::array::from_fn(|i| format!("DEEP_PNEU_PACK_{}_SUPPLY_TEMPERATURE_C", i + 1)),
            wai_duct_pressure: std::array::from_fn(|i| format!("DEEP_PNEU_WAI_{}_DUCT_PRESSURE_PA", side[i])),
            wai_duct_temp_c: std::array::from_fn(|i| format!("DEEP_PNEU_WAI_{}_DUCT_TEMPERATURE_C", side[i])),
            wai_valve_open: std::array::from_fn(|i| format!("DEEP_PNEU_WAI_{}_VALVE_OPEN", side[i])),
            hyd_reservoir_pressure: std::array::from_fn(|i| format!("DEEP_PNEU_HYD_{}_RESERVOIR_PRESSURE_PA", hyd[i])),
            cross_bleed_open: std::array::from_fn(|i| format!("DEEP_PNEU_XBLEED_{}_OPEN", xbleed[i])),
            pack_regul_fault: std::array::from_fn(|i| format!("DEEP_PNEU_PACK_{}_REGUL_FAULT", i + 1)),
            mixer_press_regul_fault: "DEEP_PNEU_MIXER_PRESS_REGUL_FAULT".to_owned(),
            ram_air_door_fault: std::array::from_fn(|i| format!("DEEP_PNEU_RAM_AIR_{}_FAULT", i + 1)),
            press_man_ctl_fault: "DEEP_PNEU_PRESS_MAN_CTL_FAULT".to_owned(),
            cabin_air_extract_vlv_fault: "DEEP_PNEU_CABIN_AIR_EXTRACT_VLV_FAULT".to_owned(),
            pack_regul_redundancy_fault: "DEEP_PNEU_PACK_REGUL_REDUNDANCY_FAULT".to_owned(),
            outflw_vlv_ctl_fault_all: "DEEP_PNEU_OUTFLW_VLV_CTL_FAULT_ALL".to_owned(),
            pack_acm_outlet_temp_c: std::array::from_fn(|i| format!("DEEP_PNEU_PACK_{}_ACM_OUTLET_TEMPERATURE_C", i + 1)),
            pack_fcv_fault: std::array::from_fn(|p| std::array::from_fn(|v| format!("DEEP_PNEU_PACK_{}_FCV_{}_FAULT", p + 1, v + 1))),
        }
    }
}

pub struct PneumaticDuctsLive {
    network: DuctNetwork,
    faults: DuctNetworkFaults,
    out: NetworkOutputs,
    names: VarNames,
    /// One published name per entry of [`coupling_table`], in that order.
    derived_names: Vec<String>,
    /// [`Self::own_zone_excess_k`]'s own stored, relaxing state.
    own_zone_excess_state: [f64; ZONE_COUNT],
    // ---- ECAM-completeness additions. This tick's magnitude (0 healthy
    // .. 1 fully failed) of each independent LRU fault, and this tick's
    // verdict of each bridged FlyByWire condition.
    pack_regul_fault: [f64; 2],
    mixer_press_regul_fault: f64,
    ram_air_door_fault: [f64; 2],
    press_man_ctl_fault: f64,
    cabin_air_extract_vlv_fault: f64,
    pack_regul_redundancy_fault: bool,
    outflw_vlv_ctl_fault_all: bool,
    pack_acm_outlet_temp_c: [f64; 2],
    pack_fcv_fault: [[f64; 2]; 2],
    /// **Pending FlyByWire write.** 211800045 AIR PACK REGUL DEGRADED --
    /// see `Truth::pack_flow_insufficient_fwd_crg`'s own doc and
    /// `E:/fbw-debug/ecam/E-AIR-FBW-WRITES.md`. A plain passthrough
    /// (`Truth` -> published Var; a `Cond` cannot read `Truth` directly).
    pack_flow_insufficient_fwd_crg: bool,
}

impl Default for PneumaticDuctsLive {
    fn default() -> Self {
        Self::new()
    }
}

fn on(b: bool) -> f64 {
    if b {
        1.0
    } else {
        0.0
    }
}

impl PneumaticDuctsLive {
    pub fn new() -> Self {
        Self {
            network: DuctNetwork::new(),
            faults: DuctNetworkFaults::default(),
            out: NetworkOutputs::default(),
            names: VarNames::new(),
            derived_names: coupling_table().into_iter().map(|(id, _)| format!("DEEP_DERIVED_FBW_FAILURE_{id}")).collect(),
            own_zone_excess_state: [0.0; ZONE_COUNT],
            pack_regul_fault: [0.0; 2],
            mixer_press_regul_fault: 0.0,
            ram_air_door_fault: [0.0; 2],
            press_man_ctl_fault: 0.0,
            cabin_air_extract_vlv_fault: 0.0,
            pack_regul_redundancy_fault: false,
            outflw_vlv_ctl_fault_all: false,
            pack_acm_outlet_temp_c: [15.0; 2],
            pack_fcv_fault: [[0.0; 2]; 2],
            pack_flow_insufficient_fwd_crg: false,
        }
    }

    /// The last tick's outputs, for anything that wants the model's state
    /// rather than its published variables.
    pub fn outputs(&self) -> &NetworkOutputs {
        &self.out
    }

    /// Fan-duct mass flow available to one engine's precooler, kg/s
    /// (module doc). Fan flow is `rho * A * V` through a fixed annulus, so
    /// at a given air density it scales with fan speed; the density ratio
    /// carries the altitude dependence.
    fn bypass_mdot_kg_s(truth: &Truth, engine: usize) -> f64 {
        if !truth.engine_running[engine] {
            return 0.0;
        }
        let density = truth.environment.ambient_pressure_pa.max(1.0) / (R_AIR_J_KG_K * (truth.environment.sat_c + 273.15).max(1.0));
        TRENT_900_BYPASS_MDOT_SLS_KG_S * truth.engine_n1_frac[engine].clamp(0.0, 1.2) * (density / SEA_LEVEL_DENSITY_KG_M3)
    }

    /// Total (ram) air temperature, K: what the fan duct actually swallows
    /// and therefore the coldest the precooler's cooling side can be.
    fn ram_total_temp_k(truth: &Truth) -> f64 {
        let static_k = (truth.environment.sat_c + 273.15).max(1.0);
        let mach = truth.environment.mach();
        static_k * (1.0 + (GAMMA_AIR - 1.0) / 2.0 * mach * mach)
    }

    fn engine_inputs(truth: &Truth) -> [EngineBleedInput; 4] {
        let fan_air_k = Self::ram_total_temp_k(truth);
        std::array::from_fn(|i| EngineBleedInput {
            // Real, unswitched IP8 port condition, read unconditionally
            // (`Truth`'s own doc on this pair, W91) -- not
            // `engine_bleed_pressure_pa`/`_temp_k`, which already carry
            // whichever of IP8/HP6 the *shallow* engine model's own switch
            // (`engine_commands.rs:466`) picked for the customer bleed.
            // Wiring that pre-switched pair in here double-switched the
            // port and let a real HP6-hot slug through this area's own
            // passive (no actuator lag) IP tap unannounced the instant
            // that other, unrelated switch opened.
            ip_port_pressure_pa: truth.engine_ip_port_pressure_pa[i].max(0.0),
            ip_port_temp_k: truth.engine_ip_port_temp_k[i].max(1.0),
            // Real HP6 port condition, read unconditionally (`Truth`'s own
            // doc on this pair): below FlyByWire's HP-valve interlock
            // whenever the engine genuinely has no usable HP source, and a
            // real hot pressure once it does -- no longer a fixed zero.
            hp_port_pressure_pa: truth.engine_hp_port_pressure_pa[i].max(0.0),
            hp_port_temp_k: truth.engine_hp_port_temp_k[i].max(1.0),
            fan_air_available_kg_s: Self::bypass_mdot_kg_s(truth, i),
            fan_air_k,
        })
    }

    /// The APU's load-compressor discharge condition. Its temperature is
    /// the compression it is actually doing, from the pressure ratio
    /// `Truth` publishes (module doc): `T2 = T1 * PR^((g-1)/(g*eta))`.
    fn apu_input(truth: &Truth) -> ApuBleedInput {
        let ambient_pa = truth.environment.ambient_pressure_pa.max(1.0);
        let ambient_k = (truth.environment.sat_c + 273.15).max(1.0);
        let pressure_pa = if truth.apu_running { truth.apu_bleed_pressure_pa.max(0.0) } else { 0.0 };
        let ratio = (pressure_pa / ambient_pa).max(1.0);
        let exponent = (GAMMA_AIR - 1.0) / (GAMMA_AIR * APU_LOAD_COMPRESSOR_POLYTROPIC_EFFICIENCY);
        ApuBleedInput {
            pressure_pa,
            temp_k: ambient_k * ratio.powf(exponent),
            fan_air_available_kg_s: if truth.apu_running { APU_COOLING_AIR_KG_S } else { 0.0 },
            fan_air_k: ambient_k,
        }
    }

    /// Whether the APU is actually able to deliver bleed: the real APU
    /// bleed pushbutton on (`truth.controls.apu_bleed_pb_on`), the APU
    /// running, and its port genuinely above the air it would have to push
    /// into.
    fn apu_bleed_available(truth: &Truth) -> bool {
        truth.controls.apu_bleed_pb_on && truth.apu_running && truth.apu_bleed_pressure_pa > truth.environment.ambient_pressure_pa * 1.05
    }

    /// The three cross-bleed valves' commanded position from the real
    /// selector knob (`truth.controls.cross_bleed_selector`, raw 0 SHUT /
    /// 1 AUTO / 2 OPEN -- `Controls`' own doc). SHUT and OPEN are the
    /// selector's own literal positions; AUTO keeps this area's prior
    /// heuristic (open only when the APU is genuinely the sole bleed
    /// source available) because FlyByWire's own AUTO logic is flight-deck
    /// computer software, out of this self-contained plant model's scope
    /// (`network.rs`'s own module doc precedent for the upstream stage).
    fn cross_bleed_command(truth: &Truth) -> f64 {
        if truth.controls.cross_bleed_selector <= 0.5 {
            0.0 // SHUT
        } else if truth.controls.cross_bleed_selector >= 1.5 {
            1.0 // OPEN
        } else {
            on(Self::apu_bleed_available(truth)) // AUTO
        }
    }

    fn inputs(&self, truth: &Truth) -> NetworkInputs {
        let apu_available = Self::apu_bleed_available(truth);
        let cross = Self::cross_bleed_command(truth);
        let recovery_k = Self::recovery_temp_k(truth);
        NetworkInputs {
            dt_s: truth.dt_s,
            ambient_pa: truth.environment.ambient_pressure_pa.max(1.0),
            ambient_k: (truth.environment.sat_c + 273.15).max(1.0),
            engines: Self::engine_inputs(truth),
            apu: Self::apu_input(truth),
            apu_bleed_selected: apu_available,
            apu_bleed_valve_command: on(apu_available),
            cross_bleed_valve_command: [cross; 3],
            // Only the pushbutton is a real crew control (`docs/deep/
            // truth-requests.md`): both feed valves of a pack open
            // together once its own pushbutton is on.
            pack_valve_open: [[on(truth.controls.pack_pb_on[0]); 2], [on(truth.controls.pack_pb_on[1]); 2]],
            // One pushbutton, both sides (`Controls`' own doc).
            wai_selected: [truth.controls.wing_anti_ice_selected; 2],
            starter_engaged: truth.controls.starter_engaged,
            engine_bleed_pb_auto: truth.controls.engine_bleed_pb_auto,
            zone_air_k: self.zone_air_k(truth, recovery_k),
        }
    }

    /// Adiabatic-wall recovery temperature, K: what a bay's structure and
    /// the air washing through it actually sit at with no internal heat
    /// source, turbulent recovery factor 0.9.
    fn recovery_temp_k(truth: &Truth) -> f64 {
        const RECOVERY_FACTOR: f64 = 0.9;
        let static_k = (truth.environment.sat_c + 273.15).max(1.0);
        let mach = truth.environment.mach();
        static_k * (1.0 + RECOVERY_FACTOR * (GAMMA_AIR - 1.0) / 2.0 * mach * mach)
    }

    /// Each zone's air temperature this tick, K, indexed by `ZONE_NAMES`:
    /// `deep::thermal_zones`' own published `THERMAL_ZONE_<NAME>_
    /// TEMPERATURE_C` (previous frame, `Truth::published`'s documented
    /// one-frame lag) when it has published one, the same recovery
    /// temperature as before otherwise -- an unheated, ram-ventilated
    /// bay's own physically sane resting state, and what every zone reads
    /// on the first frame or if `thermal_zones` were ever absent from
    /// `all_areas()`. This is the coupling that lets a real duct leak's own
    /// heat (via `thermal_zones`' identically-named ATA 36/49 leak
    /// failures heating the same zones, `network.rs`'s own module doc)
    /// come back around and trip this area's own ODLS, instead of every
    /// zone being permanently pinned at recovery temperature regardless of
    /// what is actually leaking into it.
    ///
    /// **On top of that**, [`Self::own_zone_excess_k`] adds this area's
    /// *own* previous-tick `zone_heat_w` -- see that function's own doc for
    /// why: without it, this area's own registered ATA 36 duct leak/rupture
    /// failures could raise duct pressure and gas temperature (both
    /// already real and already published) but could never actually warm
    /// the bay their own ODLS watches, because `thermal_zones` only ever
    /// consumes *its own* separate, cruder placeholder leak failures, never
    /// this area's `zone_heat_w` output (`deep::integration::
    /// failure_audit`'s sweep: every duct leak/rupture in this catalogue
    /// was live -- pressure and gas temperature moved -- but could not
    /// reach its own alert).
    fn zone_air_k(&self, truth: &Truth, recovery_k: f64) -> [f64; ZONE_COUNT] {
        let recovery_c = recovery_k - 273.15;
        std::array::from_fn(|z| {
            let name = format!("THERMAL_ZONE_{}_TEMPERATURE_C", ZONE_NAMES[z].to_ascii_uppercase());
            truth.published.get_or(&name, recovery_c) + 273.15 + self.own_zone_excess_k(z)
        })
    }

    /// This area's own lagged temperature excess above whatever
    /// `thermal_zones` is publishing for this zone, driven by this area's
    /// own `zone_heat_w` (leak/rupture enthalpy + insulation loss,
    /// `network::NetworkOutputs::zone_heat_w`'s own doc). A stored,
    /// relaxing state (`Self::relax_own_zone_excess`), not recomputed fresh
    /// from one tick's heat each time -- see that function's own doc for
    /// why a memoryless algebraic version of this feedback is unstable.
    ///
    /// `thermal_zones` is the authoritative, detailed thermal network for
    /// every zone (conduction, structure mass, ventilation links, `deep::
    /// thermal_zones::network::ThermalNetwork`) and this module must not
    /// duplicate that -- but this area cannot call into `thermal_zones`
    /// either (self-containment, `docs/deep/BRIEF.md` hard rule 2), and
    /// `thermal_zones` does not consume this area's own `zone_heat_w` (see
    /// `Self::zone_air_k`'s doc). The honest middle ground, without
    /// inventing a second full thermal network: a single-term ventilation
    /// balance at steady state, `heat_in = mdot_vent*cp*excess_k`, i.e.
    /// `excess_k = zone_heat_w / (mdot_vent*cp)` -- exactly the arithmetic
    /// `thermal_zones::PROGRESS.md`'s own 2026-09-20 pylon-bleed-duct-leak
    /// investigation used to derive that a pylon bay's ventilation, not its
    /// thermal mass, sets its steady overheat (that investigation's own
    /// words: "ventilation is 95% of the steady-state conductance, so the
    /// structure terms cannot change the answer"). [`ZONE_VENTILATION_KG_S`]
    /// reuses that investigation's own cited 0.5 kg/s pylon figure
    /// (CS/FAR 25.1187 fire-zone ventilation minimum) as a **GENERIC**
    /// figure applied to every ODLS zone alike, for lack of a more specific
    /// per-zone number.
    fn own_zone_excess_k(&self, zone: usize) -> f64 {
        self.own_zone_excess_state[zone]
    }

    /// Relaxes [`Self::own_zone_excess_k`]'s stored state toward this
    /// tick's steady-state target with an exact exponential step
    /// (`docs/deep/BRIEF.md`'s own "exact exponential steps for first-order
    /// lags" convention), over [`ZONE_EXCESS_TIME_CONSTANT_S`].
    ///
    /// An earlier version of this computed the excess fresh from the
    /// *previous* tick's `zone_heat_w` every tick, with no state of its
    /// own -- algebraically reasonable (the same steady-state formula this
    /// one relaxes toward) but numerically unstable in the closed loop it
    /// sits in: `zone_air_k` feeds `leak::step`'s own delta-T, whose
    /// `heat_to_zone_w` output is exactly what the next tick's excess was
    /// computed from, with no damping between the two. A real rupture's
    /// heat spike (hundreds of kW for one tick while the duct itself is
    /// still charging) turned into a thousand-kelvin one-tick excess, which
    /// zeroed the delta-T (and so the heat) the *next* tick, which relaxed
    /// the excess back to zero the tick after that, reopening the delta-T
    /// -- an undamped bang-bang oscillation, confirmed by instrumenting the
    /// actual trajectory (946 kW / 0 W / 144 kW / 0 W before settling).
    /// A stored state that can only move a bounded fraction of the way to
    /// its target each tick cannot overshoot into that, regardless of how
    /// large or sudden the target swings.
    fn relax_own_zone_excess(&mut self, dt_s: f64) {
        let dt = dt_s.max(0.0);
        let a = (-dt / ZONE_EXCESS_TIME_CONSTANT_S).exp();
        for z in 0..ZONE_COUNT {
            let target = (self.out.zone_heat_w[z] / (ZONE_VENTILATION_KG_S * CP_AIR_J_KGK)).max(0.0);
            self.own_zone_excess_state[z] = target + (self.own_zone_excess_state[z] - target) * a;
        }
    }

    /// Every failure `registry.rs` registers, onto the exact model field
    /// it names.
    ///
    /// The catalogue registers **one id per distinct fault mechanism on a
    /// component class** (`registry.rs`'s own module doc: "Engine bleed
    /// duct, pylon run (x4 engines)"), not one per instance, so arming a
    /// duct-leak failure leaks every instance of that duct. Per-instance
    /// arming needs per-instance ids; noted in the report.
    fn apply_faults(&mut self, faults: &Faults) {
        let duct = |leak_id: u64, rupture_id: u64, insulation_id: u64| DuctSectionFaults {
            leak: faults.get(leak_id),
            rupture: faults.get(rupture_id),
            insulation_damage: faults.get(insulation_id),
        };
        let precooler = |fouling: u64, fav: u64, sensor: u64, check: u64| PrecoolerFaults {
            fouling: faults.get(fouling),
            fan_air_valve_stuck: faults.get(fav),
            temp_sensor_fault: faults.get(sensor),
            check_valve_failure: faults.get(check),
        };

        let engine_duct = duct(f(36, 1), f(36, 2), f(36, 3));
        let engine_precooler = precooler(f(36, 4), f(36, 5), f(36, 6), f(36, 7));
        let upstream_hp = faults.get(f(36, 15));
        let upstream_pr = faults.get(f(36, 16));
        let upstream_ip = faults.get(f(36, 17));
        let start_duct = duct(f(36, 21), f(36, 22), f(36, 23));
        let start_check_valve = faults.get(f(36, 24));
        for i in 0..4 {
            self.faults.engine_duct[i] = engine_duct;
            // `engine_precooler` above is this area's own deep-catalogue
            // fouling input, shared by all four engines (this fn's own doc:
            // `registry.rs` catalogues one id per component *class*, not
            // per instance). `EXTRA_PRECOOLER_FAULT_IDS[i]` is the extra
            // catalogue's *per-engine* id a crew can actually arm from the
            // Study Failures page; `max` so either source alone still
            // fouls the core and arming both is not double the fouling.
            self.faults.engine_precooler[i] = PrecoolerFaults {
                // (build fix, INT-P4) `faults.get`, not a direct
                // `crate::failures::magnitude` call: this is `deep::live`
                // area code, which must never touch `crate::failures`'
                // process-wide state directly (see W108's own build-fix
                // note in `deep/apu/live.rs::faults_from` for the full
                // reasoning -- the same bug, same fix, this area's own
                // pre-existing instance of it). `deep/plugin.rs::
                // DeepLayer::faults` now folds these 4 extra-catalogue ids
                // into the same `Faults` map production already builds.
                fouling: engine_precooler.fouling.max(faults.get(EXTRA_PRECOOLER_FAULT_IDS[i])),
                ..engine_precooler
            };
            self.faults.upstream[i].hp_valve_stuck = upstream_hp;
            self.faults.upstream[i].pr_valve_stuck = upstream_pr;
            self.faults.upstream[i].ip_check_valve_stuck_closed = upstream_ip;
            self.faults.start[i] = start_duct;
            self.faults.start_check_valve_failure[i] = start_check_valve;
        }

        self.faults.apu_duct = duct(f(36, 8), f(36, 9), f(36, 10));
        self.faults.apu_precooler = precooler(f(36, 11), f(36, 12), f(36, 13), f(36, 14));

        let pack_duct = duct(f(36, 18), f(36, 19), f(36, 20));
        let hyd_duct = duct(f(36, 25), f(36, 26), f(36, 27));
        let wai_duct = duct(f(30, 1), f(30, 2), f(30, 3));
        for i in 0..2 {
            self.faults.packs[i] = pack_duct;
            self.faults.hyd_reservoir[i] = hyd_duct;
            self.faults.wai[i] = wai_duct;
        }

        let odls = OdlsFaults {
            loop_a_open: faults.get(f(36, 28)),
            loop_a_short: faults.get(f(36, 29)),
            loop_b_open: faults.get(f(36, 30)),
            loop_b_short: faults.get(f(36, 31)),
            false_detection: faults.get(f(36, 32)),
        };
        for z in 0..ODLS_ZONE_COUNT {
            self.faults.odls[z] = odls;
        }
    }

    /// This area's whole level-2 coupling table, with this frame's verdict
    /// on each entry (`docs/deep/authority.md`).
    ///
    /// The magnitude is passed across **unrounded**: FlyByWire's
    /// `ValveSeizure` takes the same 0..1 loss of valve authority this
    /// model's own `hp_valve_stuck`/`pr_valve_stuck` is, so a half-seized
    /// valve is half-seized on both sides. This is the one coupling of the
    /// three systems where level 2 does not have to round to a trip.
    fn each_coupling(&self, out: &mut dyn FnMut(DerivedFailure)) {
        for i in 0..4 {
            out(DerivedFailure {
                fbw_id: FBW_HP_VALVE[i],
                magnitude: self.faults.upstream[i].hp_valve_stuck.clamp(0.0, 1.0),
                deep_component: UPSTREAM_VALVE_COMPONENT,
                reason: HP_VALVE_REASON[i],
            });
        }
        for i in 0..4 {
            out(DerivedFailure {
                fbw_id: FBW_PR_VALVE[i],
                magnitude: self.faults.upstream[i].pr_valve_stuck.clamp(0.0, 1.0),
                deep_component: UPSTREAM_VALVE_COMPONENT,
                reason: PR_VALVE_REASON[i],
            });
        }
    }
}

impl crate::deep::live::Area for PneumaticDuctsLive {
    fn name(&self) -> &'static str {
        "pneumatic_ducts"
    }

    fn tick(&mut self, truth: &Truth, faults: &Faults) {
        self.apply_faults(faults);
        let inputs = self.inputs(truth);
        self.out = self.network.step(&inputs, &self.faults);
        self.relax_own_zone_excess(truth.dt_s);

        // ---- ECAM-completeness additions (E-AIR-DESIGN.md). New ata=21
        // ids (this area previously only used 36/30): 1/2 pack regulation
        // train, 3 mixer unit pressure regulator, 4/5 ram-air door, 6
        // manual pressurisation control path, 7 cabin air extract valve.
        self.pack_regul_fault = [faults.get(f(21, 1)), faults.get(f(21, 2))];
        self.mixer_press_regul_fault = faults.get(f(21, 3));
        self.ram_air_door_fault = [faults.get(f(21, 4)), faults.get(f(21, 5))];
        self.press_man_ctl_fault = faults.get(f(21, 6));
        self.cabin_air_extract_vlv_fault = faults.get(f(21, 7));
        // 211800022, corrected against the real FCOM text (E-AIR-FCOM.json,
        // FCOM p.4662, "AIR PACK 1+2 REGUL REDUNDANCY LOST"): "the
        // performance of both packs is degraded due to several valve and
        // sensor failures... on EACH pack, AT LEAST ONE of [the ACM, the
        // altitude valve, the ACM isolating valve, the temperature control
        // valve, the turbine bypass valve, the ram-air inlet/outlet valves,
        // the temperature sensors] is failed" -- an AND across the two
        // packs of an OR across several real per-pack components, not the
        // narrower FDAC-channel XOR this field held before this pass (that
        // XOR is kept nowhere else; 211800022 is its only consumer). Built
        // from the three per-pack component faults this area already models
        // (the regulation train, the ram-air door, and either real FDAC
        // channel down) -- both down on the same pack is 211800009/010,
        // already wired, but a single down channel still counts here as
        // "at least one of the redundant systems failed".
        let pack_has_a_fault = |i: usize| self.pack_regul_fault[i] > 0.0 || self.ram_air_door_fault[i] > 0.0 || truth.fdac_channel_failure[i][0] || truth.fdac_channel_failure[i][1];
        self.pack_regul_redundancy_fault = pack_has_a_fault(0) && pack_has_a_fault(1);
        // 213800015: all four OCSMs' own `BothChannelsFault` (both real
        // channels down) together.
        self.outflw_vlv_ctl_fault_all = truth.ocsm_channel_failure.iter().all(|ch| ch[0] && ch[1]);

        // 211800013/014, ata=21 n=8/9: each pack's own air cycle machine
        // (ACM) outlet temperature. This network has no ACM thermal-cycle
        // physics (it moves bleed-air mass/pressure/temperature up to the
        // pack, `o.pack_supply_temp_k`, which is the pack's real, already-
        // computed INLET condition -- the design sheet's own `pneumatic.
        // pack_inlet_pressure/temperature`), so the ACM's own cooling is
        // modelled here as an effectiveness the failure degrades: at
        // magnitude 0 the ACM cools its hot inlet air fully to ambient
        // (`environment.sat_c`, a real Truth input); at magnitude 1 it
        // cools nothing and the outlet is the same hot inlet air the pack
        // was fed. **The only cited number is the FCOM's own 95 C trip**
        // (E-AIR-FCOM.json 211800013, FCOM p.4653), applied in
        // `fbw/ata21_22_23.rs`, not here -- this function only publishes
        // the real temperature the trip is compared against.
        let ambient_c = truth.environment.sat_c;
        for i in 0..2 {
            let inlet_c = self.out.pack_supply_temp_k[i] - 273.15;
            let acm_overheat = faults.get(f(21, 8 + i as u16));
            let cooling_effectiveness = (1.0 - acm_overheat).clamp(0.0, 1.0);
            self.pack_acm_outlet_temp_c[i] = inlet_c - cooling_effectiveness * (inlet_c - ambient_c);
        }

        // 211800017-020, ata=21 n=10-13: each pack's two flow-control
        // valves (FCVs), one binary LRU fault each. The design sheet's
        // original plan bridged FlyByWire's own already-computed
        // `FcvFault` (`full_digital_agu_controller.rs:310-394`), but that
        // fault is not written to any SimVar FlyByWire's own `write()`
        // already publishes (unlike the FDAC/OCSM *channel* discretes
        // above) -- only adding one would expose it, and editing
        // `fbw-aircraft` is out of this worktree's hard rules. So this is
        // this area's own component instead: a real FCV, modelled the same
        // component-broken-flag shape as `pack_regul_fault` above, no
        // threshold invented.
        for p in 0..2 {
            for v in 0..2 {
                self.pack_fcv_fault[p][v] = faults.get(f(21, 10 + (p * 2 + v) as u16));
            }
        }

        self.pack_flow_insufficient_fwd_crg = truth.pack_flow_insufficient_fwd_crg;
    }

    fn publish(&self, out: &mut dyn FnMut(&str, f64)) {
        let n = &self.names;
        let o = &self.out;
        for z in 0..ODLS_ZONE_COUNT {
            out(&n.odls_trip[z], on(o.odls_trip[z]));
            out(&n.odls_fault[z], on(o.odls_loop_fault[z]));
            out(&n.odls_loop_a_fault[z], on(o.odls_loop_a_fault[z]));
            out(&n.odls_loop_b_fault[z], on(o.odls_loop_b_fault[z]));
        }
        for z in 0..ZONE_COUNT {
            out(&n.zone_heat_w[z], o.zone_heat_w[z]);
            out(&n.zone_jet_flux[z], o.jet_impact_flux_w_m2[z]);
        }
        for i in 0..4 {
            out(&n.engine_precooler_ovht[i], on(o.engine_precooler_overtemp[i]));
            out(&n.engine_precooler_outlet_c[i], o.engine_precooler_outlet_k[i] - 273.15);
            out(&n.engine_isolation_open[i], on(!o.engine_isolated[i]));
            out(&n.engine_duct_pressure[i], o.engine_duct_pressure_pa[i]);
            out(&n.engine_duct_temp_c[i], o.engine_duct_temp_k[i] - 273.15);
            out(&n.transfer_pipe_pressure[i], o.transfer_pipe_pressure_pa[i]);
            out(&n.hp_valve_open[i], o.hp_valve_open[i]);
            out(&n.pr_valve_open[i], o.pr_valve_open[i]);
            out(&n.start_duct_pressure[i], o.start_duct_pressure_pa[i]);
        }
        out("DEEP_PNEU_APU_PRECOOLER_OVHT", on(o.apu_precooler_overtemp));
        out("DEEP_PNEU_APU_PRECOOLER_OUTLET_C", o.apu_precooler_outlet_k - 273.15);
        out("DEEP_PNEU_APU_ISOLATION_OPEN", on(!o.apu_isolated));
        out("DEEP_PNEU_APU_DUCT_TEMPERATURE_C", o.apu_duct_temp_k - 273.15);
        out("DEEP_PNEU_APU_BLEED_VALVE_OPEN", o.apu_bleed_valve_open);
        // Real APU load-compressor demand: `deep::apu` reads this exact
        // name (`Truth::published.get_or("PNEU_APU_BLEED_DEMAND_KG_S",
        // 0.0)`) to know its own load compressor is actually loaded --
        // without it, that area's own erosion/surge-control-valve/IGV
        // failures had nothing to act on.
        out("PNEU_APU_BLEED_DEMAND_KG_S", o.apu_bleed_demand_kg_s);
        for i in 0..2 {
            out(&n.pack_supply_pressure[i], o.pack_supply_pressure_pa[i]);
            out(&n.pack_supply_temp_c[i], o.pack_supply_temp_k[i] - 273.15);
            out(&n.wai_duct_pressure[i], o.wai_duct_pressure_pa[i]);
            out(&n.wai_duct_temp_c[i], o.wai_duct_temp_k[i] - 273.15);
            out(&n.wai_valve_open[i], o.wai_valve_open[i]);
            out(&n.hyd_reservoir_pressure[i], o.hyd_reservoir_pressure_pa[i]);
        }
        for i in 0..3 {
            out(&n.cross_bleed_open[i], o.cross_bleed_valve_open[i]);
        }

        // ---- ECAM-completeness additions (E-AIR-DESIGN.md).
        for i in 0..2 {
            out(&n.pack_regul_fault[i], self.pack_regul_fault[i]);
            out(&n.ram_air_door_fault[i], self.ram_air_door_fault[i]);
        }
        out(&n.mixer_press_regul_fault, self.mixer_press_regul_fault);
        out(&n.press_man_ctl_fault, self.press_man_ctl_fault);
        out(&n.cabin_air_extract_vlv_fault, self.cabin_air_extract_vlv_fault);
        out(&n.pack_regul_redundancy_fault, on(self.pack_regul_redundancy_fault));
        out(&n.outflw_vlv_ctl_fault_all, on(self.outflw_vlv_ctl_fault_all));
        for i in 0..2 {
            out(&n.pack_acm_outlet_temp_c[i], self.pack_acm_outlet_temp_c[i]);
            for v in 0..2 {
                out(&n.pack_fcv_fault[i][v], self.pack_fcv_fault[i][v]);
            }
        }
        out("DEEP_PNEU_PACK_FLOW_INSUFFICIENT_FWD_CRG", on(self.pack_flow_insufficient_fwd_crg));

        // The level-2 couplings, so a derived failure is never silent
        // (`docs/deep/authority.md`).
        let mut k = 0usize;
        self.each_coupling(&mut |d| {
            if let Some(name) = self.derived_names.get(k) {
                out(name, d.magnitude);
            }
            k += 1;
        });
    }

    fn derived_failures(&self, out: &mut dyn FnMut(DerivedFailure)) {
        self.each_coupling(out);
    }
}

pub fn live_system() -> Box<dyn crate::deep::live::Area> {
    Box::new(PneumaticDuctsLive::new())
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::BTreeMap;

    /// Four engines at cruise power with a real IP8 bleed condition at the
    /// pylon, at a cruise ambient.
    fn cruise_truth() -> Truth {
        Truth {
            dt_s: 1.0,
            environment: crate::deep::integration::weather_truth::EnvironmentTruth {
                sat_c: -40.0,
                leading_edge_c: -20.0,
                ambient_pressure_pa: 24_000.0,
                tas_ms: 240.0,
                precipitation_on_aircraft_ratio: 0.0,
                weather: None,
            },
            altitude_ft: 35_000.0,
            on_ground: false,
            engine_n1_frac: [0.85; 4],
            engine_running: [true; 4],
            engine_ip_port_pressure_pa: [260_000.0; 4],
            engine_ip_port_temp_k: [400.0; 4],
            ac_bus_volts: [115.0; 4],
            ..Truth::default()
        }
    }

    fn published(area: &dyn crate::deep::live::Area) -> BTreeMap<String, f64> {
        let mut map = BTreeMap::new();
        area.publish(&mut |name, value| {
            map.insert(name.to_string(), value);
        });
        map
    }

    fn run(area: &mut dyn crate::deep::live::Area, truth: &Truth, faults: &Faults, ticks: usize) {
        for _ in 0..ticks {
            area.tick(truth, faults);
        }
    }

    #[test]
    fn every_variable_the_registry_triggers_on_is_actually_published() {
        let area = live_system();
        let map = published(area.as_ref());
        let mut required: Vec<String> = Vec::new();
        for zone in ZONE_NAMES.iter().take(ODLS_ZONE_COUNT) {
            required.push(format!("DEEP_PNEU_ODLS_{zone}_TRIP"));
            required.push(format!("DEEP_PNEU_ODLS_{zone}_FAULT"));
        }
        for n in 1..=4 {
            required.push(format!("DEEP_PNEU_ENG_{n}_PRECOOLER_OVHT"));
        }
        // Named in `registry.rs`'s own "new Vars this model must publish"
        // list as well as its triggers.
        required.push("DEEP_PNEU_APU_PRECOOLER_OVHT".into());
        required.push("DEEP_PNEU_APU_ISOLATION_OPEN".into());
        for n in 1..=4 {
            required.push(format!("DEEP_PNEU_ENG_{n}_ISOLATION_OPEN"));
        }
        for name in required {
            assert!(map.contains_key(&name), "{name} is read by an ECAM trigger but never published");
        }
    }

    #[test]
    fn a_healthy_network_pressurises_its_ducts_and_trips_nothing() {
        let mut area = live_system();
        run(area.as_mut(), &cruise_truth(), &Faults::default(), 300);
        let map = published(area.as_ref());
        for n in 1..=4 {
            assert!(map[&format!("DEEP_PNEU_ENG_{n}_DUCT_PRESSURE_PA")] > 101_325.0, "engine {n} duct must pressurise, got {}", map[&format!("DEEP_PNEU_ENG_{n}_DUCT_PRESSURE_PA")]);
            assert_eq!(map[&format!("DEEP_PNEU_ENG_{n}_ISOLATION_OPEN")], 1.0);
        }
        assert!(map["DEEP_PNEU_PACK_1_SUPPLY_PRESSURE_PA"] > 101_325.0);
        for zone in ZONE_NAMES.iter().take(ODLS_ZONE_COUNT) {
            assert_eq!(map[&format!("DEEP_PNEU_ODLS_{zone}_TRIP")], 0.0, "{zone} must not trip with nothing wrong");
        }
    }

    #[test]
    fn arming_the_engine_bleed_duct_rupture_sags_the_duct_and_heats_the_pylon() {
        // Failure 15_036_002, effect: "Large mass flow escapes as a
        // near-sonic jet ... higher heat-transfer effectiveness into the
        // pylon zone" and (fault 1's shared mechanism) "manifold/
        // downstream pressure sags proportionally".
        //
        // The heat check runs early: this area's own `zone_heat_w` now
        // feeds back into the same `zone_air_k` its own leak law reads its
        // driving delta-T from (`PneumaticDuctsLive::own_zone_excess_k`,
        // added this pass), so a sustained rupture genuinely heats its own
        // bay toward the duct's own temperature over enough ventilation
        // cycles -- a real ceiling (the bay can never exceed the duct
        // feeding it), not a bug. By 100 s in that self-consistent
        // heating has already pulled the delta-T (and so the instantaneous
        // heat) back down, the same way `thermal_zones::PROGRESS.md`'s own
        // pylon investigation found ventilation sets the steady state, not
        // duct enthalpy alone. The claim this assertion actually makes --
        // "a rupture dumps real heat into its own pylon" -- is checked
        // before that self-consistent response has time to act.
        let truth = cruise_truth();
        let mut ruptured = live_system();
        let mut healthy = live_system();
        run(ruptured.as_mut(), &truth, &Faults::from_pairs([(f(36, 2), 1.0)]), 3);
        let early = published(ruptured.as_ref());
        assert!(early["DEEP_PNEU_ZONE_PylonEngine1_HEAT_W"] > 1000.0, "the escaping gas must dump real heat into its own pylon, got {}", early["DEEP_PNEU_ZONE_PylonEngine1_HEAT_W"]);
        assert!(early["DEEP_PNEU_ZONE_PylonEngine1_JET_FLUX_W_M2"] > 0.0, "a full rupture must report an impinging-jet flux");

        run(ruptured.as_mut(), &truth, &Faults::from_pairs([(f(36, 2), 1.0)]), 97);
        run(healthy.as_mut(), &truth, &Faults::default(), 100);
        let bad = published(ruptured.as_ref());
        let good = published(healthy.as_ref());
        assert!(
            bad["DEEP_PNEU_ENG_1_DUCT_PRESSURE_PA"] < good["DEEP_PNEU_ENG_1_DUCT_PRESSURE_PA"],
            "a ruptured duct must sag: {} vs {}",
            bad["DEEP_PNEU_ENG_1_DUCT_PRESSURE_PA"],
            good["DEEP_PNEU_ENG_1_DUCT_PRESSURE_PA"]
        );
        assert_eq!(good["DEEP_PNEU_ZONE_PylonEngine1_JET_FLUX_W_M2"], 0.0);
    }

    #[test]
    fn arming_the_odls_false_detection_trips_and_latches_that_zones_isolation() {
        // Failure 15_036_032, effect: "Trips and latches that zone's
        // isolation valve shut with no real overheat present". The
        // published trip is what `AIR ENG n BLEED LEAK` triggers on.
        let truth = cruise_truth();
        let mut area = live_system();
        let armed = Faults::from_pairs([(f(36, 32), 1.0)]);
        run(area.as_mut(), &truth, &armed, 30);
        let map = published(area.as_ref());
        assert_eq!(map["DEEP_PNEU_ODLS_PylonEngine1_TRIP"], 1.0, "a full-severity false detection must trip a cold zone");
        assert_eq!(map["DEEP_PNEU_ENG_1_ISOLATION_OPEN"], 0.0, "the trip must isolate that engine's bleed");
        assert!(map["DEEP_PNEU_ENG_1_PR_VALVE_OPEN"] < 0.01, "and drive its PR valve shut, got {}", map["DEEP_PNEU_ENG_1_PR_VALVE_OPEN"]);

        // Latching: clearing the fault does not un-trip the isolation.
        run(area.as_mut(), &truth, &Faults::default(), 30);
        let after = published(area.as_ref());
        assert_eq!(after["DEEP_PNEU_ENG_1_ISOLATION_OPEN"], 0.0, "a real ODLS trip needs a reset, not self-clearing");
    }

    #[test]
    fn arming_a_loop_open_circuit_reports_a_detection_fault_without_tripping() {
        // Failures 15_036_028/030 raise AIR BLEED LEAK DET FAULT: both
        // loops open leaves no valid detection at all.
        let truth = cruise_truth();
        let mut area = live_system();
        let armed = Faults::from_pairs([(f(36, 28), 1.0), (f(36, 30), 1.0)]);
        run(area.as_mut(), &truth, &armed, 30);
        let map = published(area.as_ref());
        assert_eq!(map["DEEP_PNEU_ODLS_PylonEngine1_FAULT"], 1.0);
        assert_eq!(map["DEEP_PNEU_ODLS_PylonEngine1_TRIP"], 0.0, "a loop fault is not a leak");
    }

    #[test]
    fn arming_the_extra_catalogues_precooler_fault_also_fouls_the_core() {
        // 36_004, "Engine 1 precooler fault" (`failures.rs` extra::
        // pneumatic()) -- a crew-armable failure from the Study Failures
        // page, independent of this area's own deep-catalogue fouling id
        // `f(36, 4)` (covered by the next test below). Global
        // `failures::STATE`, so serialised the same way `breakers.rs`/
        // `physics/bays.rs` already do for their own global-state tests.
        let _g = crate::failures::tests::serial();
        let _f = crate::failures::Failures::new();

        let mut truth = cruise_truth();
        // Adapted per PLUGIN-ORDER.md/W149/W168: after W91's rewire, the
        // upstream stage reads `engine_ip_port_pressure_pa`/`_temp_k`
        // (the real, unswitched IP8 port), not `engine_bleed_pressure_pa`/
        // `_temp_k` (W91's own pre-switched customer-bleed pair) -- so the
        // "high-power tap" this test needs is set on the field the
        // precooler's cooling-duty math (`Self::bypass_mdot_kg_s`) and the
        // upstream stage actually consume post-W91.
        truth.engine_ip_port_temp_k = [560.0; 4];
        truth.engine_ip_port_pressure_pa = [300_000.0; 4];
        // (INT-FIX root-cause fix) Packs off: `cruise_truth()` leaves
        // `Controls::pack_pb_on` at its real default, `[true; 2]`
        // (`deep/live.rs`), and `network.rs`'s own real, untouched topology
        // feeds Pack 1's supply duct from BOTH engine 1 and engine 2
        // (`pack1_from_1`/`pack1_from_2`, both `transfer_kg`'d into the same
        // `self.packs[0].gas` -- the real A380 architecture, pack 1 fed by
        // the inboard-left pair). `transfer_kg` is bidirectional (flow runs
        // whichever way pressure actually points), so with the packs on,
        // fouling engine 2's precooler genuinely, physically perturbs the
        // shared pack-1 manifold, which then feeds a small amount of that
        // perturbation back into engine 1's own duct -- confirmed by
        // instrumenting both runs tick by tick: every metric (duct
        // pressure/temperature, precooler outlet, zone heat) is bit-
        // identical between "engine 2 fouled" and "clean" for engine 1 with
        // the packs off, and diverges the moment they are on. That coupling
        // is real and intentional (unmodified by any integration batch),
        // not the fault-id mixup the isolation check below means to catch
        // -- so leaving the packs on (their real default) confounds this
        // specific check with an unrelated, correctly-modeled effect.
        // Packs off removes that confound and leaves the check testing
        // exactly what its comment says: that `EXTRA_PRECOOLER_FAULT_IDS[1]`
        // does not also land on `engine_precooler[0]`.
        truth.controls.pack_pb_on = [false, false];

        crate::failures::set_magnitude(36_004, 1.0);
        assert_eq!(crate::failures::magnitude(36_004), 1.0, "setup: the extra catalogue's fault must actually register");

        // (build fix, INT-P4) `apply_faults` now reads this id through the
        // `Faults` map it is handed, not a direct `crate::failures::
        // magnitude` call (see this file's own `faults.get(EXTRA_
        // PRECOOLER_FAULT_IDS[i])`, changed for the same reason W108's own
        // build-fix note in `deep/apu/live.rs::faults_from` explains: area
        // code must never touch `crate::failures`' process-wide state
        // directly, or the audit harness's `thread::scope` sweep workers,
        // which build their own local `Faults` and never hold `serial()`,
        // panic). `crate::failures::set_magnitude` above still exercises
        // the real global registration path (the "setup" assertion just
        // above proves it), but reaching this area now also needs the same
        // id in the `Faults` this test hands `run` directly, the same way
        // production's `DeepLayer::faults()` folds it in every frame.
        let mut faulted = live_system();
        let mut clean = live_system();
        run(faulted.as_mut(), &truth, &Faults::from_pairs([(36_004, 1.0)]), 60);
        crate::failures::set_magnitude(36_004, 0.0);
        run(clean.as_mut(), &truth, &Faults::default(), 60);
        let hot = published(faulted.as_ref())["DEEP_PNEU_ENG_1_DUCT_TEMPERATURE_C"];
        let cool = published(clean.as_ref())["DEEP_PNEU_ENG_1_DUCT_TEMPERATURE_C"];
        assert!(hot > cool + 3.0, "the extra catalogue's precooler fault must foul the core too: {hot} C vs {cool} C");

        // Engine 2's own id (36_005) must not touch engine 1's core.
        crate::failures::set_magnitude(36_005, 1.0);
        let mut eng2 = live_system();
        run(eng2.as_mut(), &truth, &Faults::from_pairs([(36_005, 1.0)]), 60);
        let eng1_untouched = published(eng2.as_ref())["DEEP_PNEU_ENG_1_DUCT_TEMPERATURE_C"];
        assert!((eng1_untouched - cool).abs() < 3.0, "engine 2's fault id must not foul engine 1's core: {eng1_untouched} C vs clean {cool} C");
        crate::failures::set_magnitude(36_005, 0.0);
    }

    #[test]
    fn arming_precooler_fouling_leaves_the_delivered_bleed_hotter() {
        // Failure 15_036_004, effect: "For the same cooling flow the
        // outlet runs hotter ... raising overtemperature-trip risk".
        // Read on the duct the precooler actually delivers into rather
        // than on `..._PRECOOLER_OUTLET_C`: with no consumer downstream
        // (see `ControlAssumptions`) the PR valve meters in bursts, and on
        // a tick with no flow through it a heat exchanger has nothing to
        // exchange, so its instantaneous outlet reads back as its own
        // source temperature. The duct's gas temperature is the integral
        // of what was actually delivered and is what the bay, the ODLS and
        // every consumer downstream really see.
        //
        // Needs a bleed hot enough for the precooler to have work to do at
        // all (its regulation target is 200 C), i.e. a high-power tap.
        let mut truth = cruise_truth();
        truth.engine_ip_port_temp_k = [560.0; 4];
        truth.engine_ip_port_pressure_pa = [300_000.0; 4];

        let mut fouled = live_system();
        let mut clean = live_system();
        run(fouled.as_mut(), &truth, &Faults::from_pairs([(f(36, 4), 1.0)]), 60);
        run(clean.as_mut(), &truth, &Faults::default(), 60);
        let hot = published(fouled.as_ref())["DEEP_PNEU_ENG_1_DUCT_TEMPERATURE_C"];
        let cool = published(clean.as_ref())["DEEP_PNEU_ENG_1_DUCT_TEMPERATURE_C"];
        assert!(hot > cool + 3.0, "a fouled core must leave the duct hotter: {hot} C vs {cool} C");
    }

    #[test]
    fn the_apu_feeds_engine_one_through_the_cross_bleed_when_the_engines_are_dead() {
        let mut truth = cruise_truth();
        truth.engine_running = [false; 4];
        truth.engine_n1_frac = [0.0; 4];
        truth.engine_ip_port_pressure_pa = [101_325.0; 4];
        truth.engine_ip_port_temp_k = [288.15; 4];
        truth.on_ground = true;
        truth.environment.sat_c = 15.0;
        truth.environment.ambient_pressure_pa = 101_325.0;
        truth.environment.tas_ms = 0.0;
        truth.apu_running = true;
        truth.apu_bleed_pressure_pa = 320_000.0;
        truth.controls.apu_bleed_pb_on = true;

        let mut area = live_system();
        run(area.as_mut(), &truth, &Faults::default(), 300);
        let map = published(area.as_ref());
        assert_eq!(map["DEEP_PNEU_APU_BLEED_VALVE_OPEN"], 1.0);
        assert!(map["DEEP_PNEU_ENG_1_DUCT_PRESSURE_PA"] > 120_000.0, "the APU must pressurise engine 1's duct, got {}", map["DEEP_PNEU_ENG_1_DUCT_PRESSURE_PA"]);
        assert!(map["DEEP_PNEU_APU_DUCT_TEMPERATURE_C"] > 15.0, "load-compressor discharge must be hotter than the air it drew in");
    }

    #[test]
    fn the_hp6_branch_now_uses_the_real_hp_port_instead_of_a_fixed_zero() {
        // Before this pass `Truth::engine_hp_port_pressure_pa`/`_temp_k`
        // were ignored entirely (this file's own old `HP_PORT_UNAVAILABLE_
        // PA` = 0), so the HP valve could never open regardless of what
        // the engine's own HP6 port was doing. With the real port wired
        // in, an engine isolated from every other source and starved of
        // IP8 must still pressurise through its own HP valve, exactly like
        // `network.rs`'s own `the_hp_valve_opens_when_ip8_alone_cannot_
        // hold_regulation` test proves the underlying model already can.
        let mut truth = cruise_truth();
        truth.controls.cross_bleed_selector = 0.0; // SHUT: no neighbour can help
        truth.controls.pack_pb_on = [false, false]; // no consumer to mask the source
        truth.engine_ip_port_pressure_pa[0] = 150_000.0; // below the 206.8 kPa IP8/HP6 switch-over
        truth.engine_ip_port_temp_k[0] = 400.0;
        truth.engine_hp_port_pressure_pa[0] = 500_000.0;
        truth.engine_hp_port_temp_k[0] = 600.0;

        let mut area = live_system();
        let mut peak_hp_open = 0.0_f64;
        for _ in 0..300 {
            area.tick(&truth, &Faults::default());
            peak_hp_open = peak_hp_open.max(published(area.as_ref())["DEEP_PNEU_ENG_1_HP_VALVE_OPEN"]);
        }
        assert!(peak_hp_open > 0.1, "the HP valve must open off the real HP6 port once IP8 alone cannot hold regulation, peak {peak_hp_open}");
    }

    /// W91: `engine_bleed_pressure_pa`/`_temp_k` carry whichever of IP8/HP6
    /// the *shallow* `physics::engine` model's own switch (`engine_
    /// commands.rs:466`) currently has feeding the customer bleed -- they
    /// can read scorching hot even while the real IP8 tap stays cool and
    /// the real HP6 port is reported unavailable (as it would be right
    /// after that other, unrelated switch has briefly, and wrongly for
    /// this area's own purposes, picked HP6). This area's own upstream
    /// stage must key off the real, unswitched pair, or a transient in
    /// that other switch injects an unregulated hot slug through this
    /// area's own passive (no actuator lag) IP tap.
    #[test]
    fn the_upstream_stage_follows_the_real_unswitched_ip8_port_not_the_pre_switched_customer_bleed_pair() {
        let mut truth = cruise_truth();
        truth.engine_bleed_pressure_pa = [900_000.0; 4]; // pre-switched pair: looks HP6-hot
        truth.engine_bleed_temp_k = [650.0; 4];
        truth.engine_ip_port_pressure_pa = [260_000.0; 4]; // real IP8: unchanged, moderate
        truth.engine_ip_port_temp_k = [400.0; 4];
        truth.engine_hp_port_pressure_pa = [0.0; 4]; // real HP6: genuinely unavailable
        truth.engine_hp_port_temp_k = [288.15; 4];

        let mut area = live_system();
        run(area.as_mut(), &truth, &Faults::default(), 60);
        let map = published(area.as_ref());
        assert!(
            map["DEEP_PNEU_ENG_1_PRECOOLER_OUTLET_C"] < 300.0,
            "must not read the pre-switched customer-bleed pair's HP6-hot temperature when the real IP8 tap is cool and the real HP6 port is unavailable, got {} C",
            map["DEEP_PNEU_ENG_1_PRECOOLER_OUTLET_C"]
        );
        assert_eq!(map["DEEP_PNEU_ENG_1_PRECOOLER_OVHT"], 0.0);
    }

    #[test]
    fn cross_bleed_selector_shut_overrides_the_apu_sole_source_heuristic() {
        let mut truth = cruise_truth();
        truth.engine_running = [false; 4];
        truth.engine_n1_frac = [0.0; 4];
        truth.engine_ip_port_pressure_pa = [101_325.0; 4];
        truth.engine_ip_port_temp_k = [288.15; 4];
        truth.on_ground = true;
        truth.environment.sat_c = 15.0;
        truth.environment.ambient_pressure_pa = 101_325.0;
        truth.environment.tas_ms = 0.0;
        truth.apu_running = true;
        truth.apu_bleed_pressure_pa = 320_000.0;
        truth.controls.apu_bleed_pb_on = true;
        truth.controls.cross_bleed_selector = 0.0; // SHUT
        // Pack 1's own dual feed (engines 1 *and* 2) is itself a second
        // bridge between their ducts, entirely independent of the
        // cross-bleed valves (`network.rs`'s own `closing_all_cross_bleed_
        // valves_stops_a_non_running_engine_from_pressurising` test notes
        // exactly this) -- shut here so this test isolates what the
        // cross-bleed selector itself controls.
        truth.controls.pack_pb_on = [false, false];

        let mut area = live_system();
        run(area.as_mut(), &truth, &Faults::default(), 300);
        let map = published(area.as_ref());
        assert_eq!(map["DEEP_PNEU_XBLEED_L_OPEN"], 0.0, "SHUT must override even the sole-source AUTO heuristic");
        // The APU's own bleed valve feeds engine 1's duct directly and is
        // not one of the three valves this selector controls (`Controls`'
        // own doc: "a single knob controls all three cross-bleed valves",
        // i.e. L/C/R, not the separate APU valve), so engine 1 still
        // pressurises; SHUT is proven by engine 2 -- reachable only
        // through the now-shut left cross-bleed valve -- staying unfed.
        assert!(map["DEEP_PNEU_ENG_1_DUCT_PRESSURE_PA"] > 120_000.0, "the APU's own valve into engine 1 is unrelated to the cross-bleed selector, got {}", map["DEEP_PNEU_ENG_1_DUCT_PRESSURE_PA"]);
        assert!(map["DEEP_PNEU_ENG_2_DUCT_PRESSURE_PA"] < 110_000.0, "with the cross-bleed selector SHUT, engine 2 must not be fed through the left valve, got {}", map["DEEP_PNEU_ENG_2_DUCT_PRESSURE_PA"]);
    }

    #[test]
    fn cross_bleed_selector_open_forces_every_valve_open_with_no_sole_source_condition() {
        let mut truth = cruise_truth();
        truth.controls.cross_bleed_selector = 2.0; // OPEN
        let mut area = live_system();
        run(area.as_mut(), &truth, &Faults::default(), 30);
        let map = published(area.as_ref());
        assert_eq!(map["DEEP_PNEU_XBLEED_L_OPEN"], 1.0);
        assert_eq!(map["DEEP_PNEU_XBLEED_C_OPEN"], 1.0);
        assert_eq!(map["DEEP_PNEU_XBLEED_R_OPEN"], 1.0);
    }

    #[test]
    fn switching_a_pack_pushbutton_off_stops_feeding_that_pack() {
        let truth = cruise_truth(); // default pack_pb_on = [true, true]
        let mut off_truth = cruise_truth();
        off_truth.controls.pack_pb_on[0] = false;

        let mut on_area = live_system();
        let mut off_area = live_system();
        run(on_area.as_mut(), &truth, &Faults::default(), 300);
        run(off_area.as_mut(), &off_truth, &Faults::default(), 300);
        let on_pressure = published(on_area.as_ref())["DEEP_PNEU_PACK_1_SUPPLY_PRESSURE_PA"];
        let off_pressure = published(off_area.as_ref())["DEEP_PNEU_PACK_1_SUPPLY_PRESSURE_PA"];
        assert!(on_pressure > off_pressure + 5000.0, "switching pack 1's pushbutton off must stop feeding it: on {on_pressure} vs off {off_pressure}");
    }

    #[test]
    fn a_thermal_areas_own_wing_duct_leak_heats_the_bay_enough_to_trip_this_areas_odls() {
        // End-to-end coupling test: `thermal_zones` carries its own
        // (pre-existing, interim) ATA 30 wing-anti-ice-duct-leak failure
        // that heats WingLeLeft's real air node directly
        // (`thermal_zones::live::apply_ice_and_duct_failures`). Before this
        // pass this area could never see that heat -- every zone was given
        // recovery temperature regardless (module doc's old "Contract
        // gap") -- so the leak -> bay overheat -> isolation chain this
        // area exists for was cut in the middle. With `Self::zone_air_k`
        // now reading `truth.published` instead, the other area's real
        // heat must reach this area's own ODLS and trip it.
        //
        // WingLeLeft (unlike a pylon) has no forced-ventilation link at
        // all in `topology_a380::build`, so a full-severity leak from a
        // duct at take-off bleed condition has to carry it past this area's
        // own absolute wing/fuselage ODLS threshold
        // (`odls::OverheatDetectionLoop::THRESHOLD_WING_FUSELAGE_K`).
        // `thermal_zones` used to inject a fixed 30 kW here regardless of
        // engine state (853 C on a cold aircraft); it now derives the heat
        // from the duct's real pressure and temperature, which is why this
        // test has to supply a running engine.
        // A ram-vented pylon's own 40 kW leak, by contrast, settles only
        // ~75 K above ambient under this area's own 0.5 kg/s pylon vent
        // and does not confirm a trip at the pylon/strut class's higher
        // (~200 C) threshold either way.
        let mut deep = crate::deep::live::Deep::new().with_area(live_system()).with_area(crate::deep::thermal_zones::live::live_system());
        let leak_id = f_thermal(30, 1); // thermal_zones' own WingLeLeft anti-ice duct leak id
        let armed = Faults::from_pairs([(leak_id, 1.0)]);
        // The leak's heat is derived from the duct's real condition now (a
        // choked crack fed from `engine_ip_port_pressure_pa`/`_temp_k`), so
        // a cold, unpowered aircraft leaks nothing -- the outcome the comment
        // above predicted once `thermal_zones` fixed its leak-model form.
        // Give it the running engine that area's own `takeoff_truth` uses:
        // Trent 972 IP8 at take-off power, 970 kPa / 590 K.
        //
        // (INT-FIX root-cause fix) `engine_bleed_pressure_pa`/`_temp_k` ARE
        // also set here, alongside the ip-port pair: this is a cross-area
        // test (`thermal_zones` is `with_area`'d in too), and
        // `thermal_zones::live::apply_ice_and_duct_failures` -- untouched by
        // any integration batch -- still reads `truth.engine_bleed_
        // pressure_pa`/`_temp_k` for its own wing-duct-leak heat, not the
        // new `engine_ip_port_*` pair W91 repointed *this* area's own
        // `engine_inputs()` to. W91's own test-fixture rename (Batch 2,
        // EDIT8-15) renamed this literal from `engine_bleed_*` to
        // `engine_ip_port_*` for every test in this file, correctly for the
        // ones that only exercise this area -- but this is the one test that
        // also feeds `thermal_zones`, and stripping `engine_bleed_*` left
        // that area's leak fed from `Truth::default()`'s 0 Pa / 0 K, so it
        // never leaked at all (confirmed: this test passes verbatim, field
        // names and all, against unmodified `D:/A380/fbw-xp-systems` main, where
        // `engine_ip_port_pressure_pa`/`_temp_k` do not exist and only
        // `engine_bleed_*` is set). Setting both fields to the same take-off
        // condition feeds each area the input it actually reads.
        let truth = Truth {
            dt_s: 1.0,
            engine_running: [true; 4],
            engine_n1_frac: [1.0; 4],
            engine_ip_port_pressure_pa: [970_000.0; 4],
            engine_ip_port_temp_k: [590.0; 4],
            engine_bleed_pressure_pa: [970_000.0; 4],
            engine_bleed_temp_k: [590.0; 4],
            ..Truth::default()
        };
        let mut published = BTreeMap::new();
        for _ in 0..600 {
            deep.tick(truth.clone(), &armed, &mut |name, value| {
                published.insert(name.to_string(), value);
            });
        }
        // The precondition for the trip below is the loop's own threshold,
        // not a number picked to pass: the bay has to be hotter than what
        // the ODLS is set to notice.
        let bay_k = published["THERMAL_ZONE_WINGLELEFT_TEMPERATURE_C"] + 273.15;
        let threshold_k = crate::deep::pneumatic_ducts::odls::OverheatDetectionLoop::THRESHOLD_WING_FUSELAGE_K;
        assert!(bay_k > threshold_k, "setup: the thermal area's own leak failure must heat the bay past the ODLS threshold ({threshold_k:.1} K), got {bay_k:.1} K");
        assert_eq!(published["DEEP_PNEU_ODLS_WingLeLeft_TRIP"], 1.0, "the real bay heat must now reach this area's own ODLS and trip it");

        // The right wing never had anything leak into it.
        assert_eq!(published["DEEP_PNEU_ODLS_WingLeRight_TRIP"], 0.0);
    }

    /// `thermal_zones` registers its ATA 30/36 duct-leak failures under
    /// `Area::ThermalZones`, a different (interim, pre-existing) failure
    /// id from this area's own equivalent faults -- both model a leak into
    /// the same physical bay, from two different areas' own components,
    /// per `network.rs`'s own module doc.
    fn f_thermal(ata: u16, n: u16) -> u64 {
        crate::deep::api::failure_id(RegArea::ThermalZones, ata, n)
    }

    /// The "bigger" gap this pass closes: this area's *own* registered
    /// ATA 36 duct-leak/rupture failures were live (pressure/gas
    /// temperature moved) but could never reach their own alert, because
    /// `thermal_zones` never consumes this area's `zone_heat_w`
    /// (`Self::own_zone_excess_k`'s own doc). A full-severity engine-1
    /// duct rupture, at a real high-pressure bleed condition, must now trip
    /// engine 1's own pylon ODLS using only this area's own physics -- no
    /// dependency on `thermal_zones` at all.
    #[test]
    fn this_areas_own_engine_duct_rupture_now_reaches_its_own_odls() {
        // The precooler regulates its outlet toward `OUTLET_TARGET_C`
        // (200 C, `precooler.rs`), which is why a leak/rupture cannot rely
        // on raw source temperature alone to cross a ~200 C pylon/strut
        // threshold: a *low-N1* condition starves the precooler of its own
        // cooling (fan-bypass) air (`bypass_mdot_kg_s` scales with N1), so
        // even a fully open FAV cannot hold the target against a real HP6
        // source -- the same real precooler-undersizing failure mode
        // `precooler.rs`'s own `a_stuck_closed_fav_cannot_cool_and_the_
        // bleed_stays_hot` test exercises directly, reached here through a
        // real airframe state instead.
        let mut truth = cruise_truth();
        truth.on_ground = true;
        truth.environment.ambient_pressure_pa = 101_325.0;
        truth.environment.sat_c = 15.0;
        truth.environment.tas_ms = 0.0;
        truth.engine_n1_frac = [0.05; 4]; // starves the precooler's own cooling air
        truth.engine_ip_port_pressure_pa = [900_000.0; 4]; // a strong bleed source to rupture
        truth.engine_ip_port_temp_k = [560.0; 4];
        truth.engine_hp_port_pressure_pa = [1_200_000.0; 4];
        truth.engine_hp_port_temp_k = [700.0; 4];
        let rupture_id = f(36, 2); // engine duct rupture, module doc's ordering (leak, rupture, insulation)

        let mut healthy = live_system();
        run(healthy.as_mut(), &truth, &Faults::default(), 10);
        let healthy_out = published(healthy.as_ref());
        assert_eq!(healthy_out["DEEP_PNEU_ODLS_PylonEngine1_TRIP"], 0.0, "a healthy duct must not trip its own bay");

        let mut ruptured = live_system();
        let rupture_faults = Faults::from_pairs([(rupture_id, 1.0)]);
        run(ruptured.as_mut(), &truth, &rupture_faults, 1);
        let heat_pulse = published(ruptured.as_ref())["DEEP_PNEU_ZONE_PylonEngine1_HEAT_W"];
        assert!(heat_pulse > 0.0, "a rupture must actually deliver heat to its own zone, got {heat_pulse}");

        // Continued mid-transient (module doc on `arming_the_engine_bleed_
        // duct_rupture_sags_the_duct_and_heats_the_pylon`): the rupture's
        // own heat pulse and this area's own zone feedback settle into a
        // self-consistent equilibrium over more ticks than this (heat
        // itself can transiently read back to zero once the lagged excess
        // has already pushed the bay close to the duct's own temperature),
        // so the trip is confirmed here while the bay is still genuinely
        // past its own absolute threshold from that earlier real heat, not
        // decades into that later settling.
        run(ruptured.as_mut(), &truth, &rupture_faults, 8);
        let ruptured_out = published(ruptured.as_ref());
        assert_eq!(ruptured_out["DEEP_PNEU_ODLS_PylonEngine1_TRIP"], 1.0, "that heat must now reach this area's own ODLS and trip it");
        // `f(36, 2)` registers one id per fault *mechanism* on the engine-
        // duct component *class* (`registry.rs`'s own module doc), so
        // arming it ruptures all four engine ducts alike -- every pylon
        // trips, not just engine 1's. A zone this fault cannot possibly
        // touch (the wing leading edge, a different duct entirely) must
        // still stay clear.
        assert_eq!(ruptured_out["DEEP_PNEU_ODLS_WingLeLeft_TRIP"], 0.0, "a zone this fault cannot reach must stay clear");
    }

    /// `deep::apu` reads `PNEU_APU_BLEED_DEMAND_KG_S` to know its own load
    /// compressor is actually loaded; without a real figure here its own
    /// erosion/surge-control-valve/IGV failures had nothing to act on.
    #[test]
    fn apu_bleed_demand_is_published_and_real_once_the_apu_is_bled() {
        let mut truth = Truth { dt_s: 0.2, on_ground: true, apu_running: true, apu_bleed_pressure_pa: 310_000.0, ..Truth::default() };
        truth.controls.apu_bleed_pb_on = true;

        let mut off = live_system();
        run(off.as_mut(), &Truth { dt_s: 0.2, ..Truth::default() }, &Faults::default(), 5);
        assert_eq!(published(off.as_ref())["PNEU_APU_BLEED_DEMAND_KG_S"], 0.0, "no bleed selected, no demand");

        let mut on = live_system();
        run(on.as_mut(), &truth, &Faults::default(), 5);
        assert!(published(on.as_ref())["PNEU_APU_BLEED_DEMAND_KG_S"] > 0.0, "a real APU bleed source must show a real, nonzero demand");
    }

    // -----------------------------------------------------------------
    // Authority (docs/deep/authority.md): the level-2 couplings.

    fn derived(area: &PneumaticDuctsLive) -> BTreeMap<u64, f64> {
        let mut out = BTreeMap::new();
        crate::deep::live::Area::derived_failures(area, &mut |d| {
            out.insert(d.fbw_id, d.magnitude);
        });
        out
    }

    #[test]
    fn the_coupling_table_matches_what_the_area_actually_emits() {
        let area = PneumaticDuctsLive::new();
        let table = coupling_table();
        let mut emitted: Vec<(u64, &'static str)> = Vec::new();
        area.each_coupling(&mut |d| emitted.push((d.fbw_id, d.deep_component)));
        assert_eq!(emitted, table);
        assert_eq!(area.derived_names.len(), table.len());
        assert_eq!(table.len(), 8, "four HP valves and four PR valves -- all FlyByWire models of this area's components");

        // These are the extra catalogue's ids, the ones
        // `extra::write_pneumatic_valves` turns into `PNEU_VALVE_FAILED:n`
        // every tick. The valve numbers must be FlyByWire's own: 1-4 the
        // HP valves, 5-8 the PR valves (`a380_systems/src/pneumatic.rs`).
        let valves: std::collections::BTreeMap<u64, usize> = crate::failures::extra::PNEUMATIC_VALVES.iter().copied().collect();
        for (i, id) in FBW_HP_VALVE.iter().enumerate() {
            assert_eq!(valves.get(id), Some(&(i + 1)), "{id} must be FlyByWire's HP valve {}", i + 1);
        }
        for (i, id) in FBW_PR_VALVE.iter().enumerate() {
            assert_eq!(valves.get(id), Some(&(i + 5)), "{id} must be FlyByWire's PR valve {}", i + 5);
        }
        let catalogue: std::collections::BTreeSet<u64> = crate::failures::extra::extra_failures().into_iter().map(|x| x.id).collect();
        for (id, _) in &table {
            assert!(catalogue.contains(id), "{id} is in no catalogue this plugin drives");
        }
    }

    #[test]
    fn a_healthy_network_tells_flybywire_nothing_at_all() {
        let mut area = PneumaticDuctsLive::new();
        crate::deep::live::Area::tick(&mut area, &cruise_truth(), &Faults::default());
        assert!(derived(&area).values().all(|&m| m == 0.0), "{:?}", derived(&area));
    }

    #[test]
    fn a_seized_bleed_valve_reaches_flybywires_own_valve_at_the_same_severity() {
        // `authority.md`'s one unrounded coupling: FlyByWire's
        // `ValveSeizure` takes the same 0..1 loss of authority this model's
        // own `pr_valve_stuck` is, so a 40%-seized valve crosses as 0.4 and
        // not as a trip.
        let mut area = PneumaticDuctsLive::new();
        crate::deep::live::Area::tick(&mut area, &cruise_truth(), &Faults::from_pairs([(f(36, 16), 0.4)]));
        let d = derived(&area);
        for id in FBW_PR_VALVE {
            assert!((d[&id] - 0.4).abs() < 1e-9, "PR valve {id} should cross at 0.4, got {}", d[&id]);
        }
        for id in FBW_HP_VALVE {
            assert_eq!(d[&id], 0.0, "the HP valves are a different valve and must be untouched");
        }
        assert!((published(&area)["DEEP_DERIVED_FBW_FAILURE_36012"] - 0.4).abs() < 1e-9, "and it must be visible");

        // And the HP valve's own id, the other way round.
        let mut hp = PneumaticDuctsLive::new();
        crate::deep::live::Area::tick(&mut hp, &cruise_truth(), &Faults::from_pairs([(f(36, 15), 1.0)]));
        let d = derived(&hp);
        for id in FBW_HP_VALVE {
            assert_eq!(d[&id], 1.0);
        }
        for id in FBW_PR_VALVE {
            assert_eq!(d[&id], 0.0);
        }
    }

    /// End to end: the deep model's verdict, through the same
    /// `PNEU_VALVE_FAILED:n` variable `extra::write_pneumatic_valves`
    /// writes, freezes FlyByWire's own valve.
    ///
    /// The global `crate::failures` registry is deliberately not touched
    /// (it is process-wide and shared with every other test): the
    /// magnitude is written straight into the variable, which is what
    /// `write_pneumatic_valves` does with it every tick.
    #[test]
    fn a_derived_valve_seizure_changes_flybywires_own_solve() {
        use crate::aspects::test_vars::TestVars;
        use std::time::Duration;
        use systems::simulation::{Simulation, StartState, VariableRegistry};

        let mut area = PneumaticDuctsLive::new();
        crate::deep::live::Area::tick(&mut area, &cruise_truth(), &Faults::from_pairs([(f(36, 16), 1.0)]));
        let verdict = derived(&area);
        assert_eq!(verdict[&36_012], 1.0, "setup: the deep model must have concluded engine 1's PR valve is seized");

        let run = |seized: bool| -> f64 {
            let mut vars = TestVars::default();
            let mut sim = Simulation::new(StartState::Cruise, a380_systems::A380::new, &mut vars);
            // A bare test bed reads every unset variable as zero: every
            // pushbutton off, every engine stopped, no generator on line
            // and so no power for the bleed control. Four running engines
            // (`TrentEngine`'s own variables), the batteries in AUTO and
            // the ENG n BLEED pushbuttons in AUTO are what an A380 in the
            // cruise actually has, and are what FlyByWire's own pneumatic
            // system needs before any valve moves at all. The IP8 port
            // condition is the `ENGINE_IP_PORT_*` pair `deep::plugin`
            // publishes from this crate's engine model every frame.
            for n in 1..=4 {
                vars.set(&format!("TURB ENG CORRECTED N1:{n}"), 85.0);
                vars.set(&format!("TURB ENG CORRECTED N2:{n}"), 90.0);
                vars.set(&format!("A32NX_ENGINE_N2:{n}"), 90.0);
                vars.set(&format!("A32NX_ENGINE_N3:{n}"), 90.0);
                vars.set(&format!("A32NX_ENGINE_STATE:{n}"), 1.0);
                vars.set(&format!("A32NX_OVHD_PNEU_ENG_{n}_BLEED_PB_IS_AUTO"), 1.0);
                vars.set(&format!("A32NX_OVHD_ELEC_ENG_GEN_{n}_PB_IS_ON"), 1.0);
                vars.set(&format!("A32NX_ENGINE_IP_PORT_PRESSURE_PA:{n}"), 300_000.0);
                vars.set(&format!("A32NX_ENGINE_IP_PORT_TEMP_K:{n}"), 500.0);
            }
            for bat in ["BAT_1", "BAT_2", "BAT_ESS", "BAT_APU"] {
                vars.set(&format!("A32NX_OVHD_ELEC_{bat}_PB_IS_AUTO"), 1.0);
            }
            // A real atmosphere: a gas model handed a vacuum divides by
            // zero, and the whole pneumatic solve comes back NaN.
            // `UpdateContext` reads these in MSFS's own units
            // (`update_context.rs`: ambient pressure in inHg, temperature
            // in C, density in slug/ft^3).
            vars.set("AMBIENT TEMPERATURE", 15.0);
            vars.set("AMBIENT PRESSURE", 29.92);
            vars.set("AMBIENT DENSITY", 0.002_377);
            vars.set("PRESSURE ALTITUDE", 0.0);
            vars.set("SIM ON GROUND", 1.0);
            vars.set("TOTAL WEIGHT", 1_200_000.0);
            if seized {
                // Exactly what `extra::write_pneumatic_valves` does with a
                // magnitude, for the four PR valves this verdict covers.
                // Written before the first tick, so FlyByWire's own valves
                // seize where they start -- shut -- and never open.
                for n in 5..=8 {
                    let id = vars.get(format!("PNEU_VALVE_FAILED:{n}"));
                    systems::simulation::SimulatorReaderWriter::write(&mut vars, &id, 1.0);
                }
            }
            for i in 0..60 {
                sim.tick(Duration::from_millis(50), 100.0 + i as f64 * 0.05, &mut vars);
            }
            vars.value("A32NX_PNEU_ENG_1_PR_VALVE_OPEN")
        };
        let free = run(false);
        let frozen = run(true);
        assert!(free > 0.0, "setup: FlyByWire's own PR valve opens in the cruise with the bleed running, got {free}");
        assert_eq!(frozen, 0.0, "seized where it stood, it must stay shut -- the deep model's verdict, in FlyByWire's own solve");
    }

    #[test]
    fn a_cold_dark_aircraft_publishes_finite_values_and_a_zero_dt_frame_changes_nothing() {
        let mut area = live_system();
        let truth = Truth::default();
        run(area.as_mut(), &truth, &Faults::default(), 100);
        for (name, value) in published(area.as_ref()) {
            assert!(value.is_finite(), "{name} went non-finite");
        }
        let still = Truth { dt_s: 0.0, ..Truth::default() };
        area.tick(&still, &Faults::default());
        let before = published(area.as_ref());
        area.tick(&still, &Faults::default());
        assert_eq!(before, published(area.as_ref()));
    }

    /// 211800045 AIR PACK REGUL DEGRADED (E-AIR-DESIGN.md): a plain
    /// passthrough of the pending-write `Truth::pack_flow_insufficient_fwd_crg`
    /// (see `E:/fbw-debug/ecam/E-AIR-FBW-WRITES.md`) -- reads `0.0` (healthy)
    /// today, and this proves the passthrough itself is wired correctly so
    /// the alert lights up the moment FlyByWire's write lands, with no
    /// further plugin change.
    #[test]
    fn pack_flow_insufficient_fwd_crg_is_a_plain_truth_passthrough() {
        let mut area = live_system();
        area.tick(&Truth::default(), &Faults::default());
        assert_eq!(published(area.as_ref())["DEEP_PNEU_PACK_FLOW_INSUFFICIENT_FWD_CRG"], 0.0);

        area.tick(&Truth { pack_flow_insufficient_fwd_crg: true, ..Truth::default() }, &Faults::default());
        assert_eq!(published(area.as_ref())["DEEP_PNEU_PACK_FLOW_INSUFFICIENT_FWD_CRG"], 1.0);
    }
}

