//! The duct network: the real IP/HP port -> HP/PR valve -> precooler stage
//! for each engine, the APU's own bleed stage, FlyByWire's real left/
//! centre/right cross-bleed topology (no shared manifold), and the
//! consumers it feeds: packs, wing anti-ice, engine start, hydraulic
//! reservoir pressurisation. Backlog item 1, extended upstream per the
//! lead's follow-up, with items 2 (leak/rupture + a structural/wiring jet-
//! flux interface) and 3 (ODLS-driven isolation) wired through it.
//!
//! **Scope note (avoiding duplicate work).** `deep::cabin::water.rs`
//! already models potable-water pneumatic pressurisation; `deep::apu::
//! power_section.rs` owns the APU's own gas-generator core (this network
//! takes its bleed port condition as an external input); `deep::fire_ice::
//! fire_loops.rs` already covers engine/APU/gear-bay/cargo/avionics *fire*
//! detection (ATA 26) -- ODLS here is the distinct ATA 36 bleed-duct
//! overheat system. `deep::thermal_zones::topology_a380` is the
//! authoritative A380 zone graph (`ZoneId` is a `Vec` index assigned at
//! that network's own construction, so this self-contained module cannot
//! import it); this module keys its own zone-indexed outputs by the exact
//! same zone *names* that topology builds (`"PylonEngine1"`, `"TailCone"`,
//! `"WingLeLeft"`/`"WingLeRight"`, `"BellyFairingPacks"`, `"WingGearWell"`
//! -- `ZONE_NAMES` below), confirmed by reading `topology_a380.rs`'s own
//! `Zone::new(...)` calls, so a future integration pass can map name ->
//! `ZoneId` directly with no renaming. `deep::thermal_zones::registry.rs`
//! already registers its own **interim placeholder** pylon/APU-duct-leak
//! failures (`Area::ThermalZones` ids, ATA 36/49, `PYLON_BLEED_LEAK_MAX_
//! HEAT_W`/`APU_DUCT_LEAK_MAX_HEAT_W` fixed-reference watts) that inject
//! heat directly with no real pressure/mass-flow model behind them -- this
//! network's own `leak.rs`-driven `NetworkOutputs::zone_heat_w` is the real
//! version of exactly that same physical fault, and a future integration
//! pass should retire that placeholder in favour of this one rather than
//! running both (documented here and in `PROGRESS.md`, not resolved
//! unilaterally since `36_thermal.pylon_*_bleed_duct` is that other
//! workstream's own registered component, out of this directory's scope
//! to edit).
//!
//! **Upstream stage (per engine).** FlyByWire's own `CoreProcessingInput
//! OutputModuleAUnit` (`a380_systems/pneumatic.rs:690-849`) is the real
//! CPIOM *software* that regulates the HP/PR valves (specific PID gains,
//! fire-pushbutton/cross-bleed-selector interlocks, APU-bleed-closes-PR-
//! valve logic) -- that control *software* is FBW's own and stays out of
//! this self-contained module's scope (module docs' own precedent: this
//! workstream models physical plant, not FBW's flight-deck computer logic).
//! What is missing entirely from this crate is the *duct/valve hardware*
//! those ports feed: a real IP8 passive check-valve tap, a real HP6
//! electro-pneumatic valve with its own orifice flow and fault surface,
//! and the transfer-pipe volume between them and the pressure-regulating
//! (PR/shutoff) valve. This module builds that hardware, fed directly by
//! the engine's own published port conditions
//! (`physics::engine::EngineOutputs::ip_port_pressure_pa`/`ip_port_temp_k`/
//! `hp_port_pressure_pa`/`hp_port_temp_k`, read-only reference, not
//! imported), and drives it with this module's *own* simple, physically-
//! motivated regulation law (not a copy of FBW's tuned PID): prefer the
//! passive IP8 tap; open the HP valve, proportionally, only once IP8 can no
//! longer hold the regulation target *or* its own pressure has fallen below
//! EASA TCDS E.012's public switch-over figure; close the PR valve when the
//! engine is isolated, starting, or the transfer pipe itself is too weak.
//!
//! **Cross-bleed topology**, exactly FlyByWire's own (no shared manifold
//! volume, `a380_systems/pneumatic.rs`):
//! - Left cross-bleed valve (valve 9, `PNEU_XBLEED_VALVE_L_OPEN`,
//!   `pneumatic.rs:278`): engine 1 <-> engine 2.
//! - Centre cross-bleed valve (valve 10, `PNEU_XBLEED_VALVE_C_OPEN`,
//!   `pneumatic.rs:279`): engine 1 <-> engine 4 (spans the fuselage
//!   centreline, `pneumatic.rs:452`).
//! - Right cross-bleed valve (valve 11, `PNEU_XBLEED_VALVE_R_OPEN`,
//!   `pneumatic.rs:280`): engine 3 <-> engine 4.
//! - APU bleed valve: APU duct -> engine 1's duct directly
//!   (`pneumatic.rs:410-414`, `apu_bleed_air_valve.update_move_fluid(...,
//!   engine_1_system)`).
//! - Hydraulic reservoir pressurisation: green from engine 1's duct,
//!   yellow from engine 4's duct (`pneumatic.rs:416-446`), exactly FBW's
//!   own assignment.
//! - Packs: pack 1 from engines 1 and 2 (two valves), pack 2 from engines 3
//!   and 4 (`pneumatic.rs:456-468`, matching the real per-pack dual flow-
//!   control-valve architecture `PackFlowValveState`/`fcv_id` implies).
//! - Wing anti-ice (no FBW equivalent exists at all, module docs elsewhere):
//!   left from engine 2's duct, right from engine 3's duct -- the inboard
//!   engine on each side, a reasonable (not publicly sourced) tap point,
//!   documented as such.
//! - Engine start: each engine's own start duct taps its *own* local duct
//!   directly through a real check valve -- when that engine is not yet
//!   running, its own local duct is pressurised only by whatever reaches it
//!   through the cross-bleed chain (from another engine, or the APU via
//!   engine 1), which is exactly why the cross-bleed system exists on the
//!   ground.
//!
//! **Isolation.** An ODLS trip on an engine's pylon (or the APU's tail-cone
//! run) closes that engine's *own* PR/shutoff valve (the real "ENG n BLEED"
//! pushbutton's own valve) **and** every cross-bleed/APU connection
//! touching that engine, so a confirmed fault genuinely isolates that duct
//! from every possible source, not just its own engine -- closing the PR
//! valve alone would leave a neighbour able to keep feeding the same faulty
//! run through cross-bleed.

use super::duct::{
    one_way_transfer_kg, orifice_mass_flow_kg_s, passive_valve_open_fraction, transfer_kg, DuctSection, DuctSectionFaults, DuctVolume,
};
use super::leak;
use super::odls::{OdlsFaults, OverheatDetectionLoop};
use super::precooler::{Precooler, PrecoolerFaults};

/// Zone names, exactly as `deep::thermal_zones::topology_a380::build()`
/// constructs them (`Zone::new("PylonEngine1", ...)` etc., confirmed by
/// reading that file). The first `ODLS_ZONE_COUNT` carry a real overheat
/// detection loop; the last two (pack bay, hydraulic bay) still receive
/// real leak/insulation heat but are not, in this pass, given their own
/// ODLS loop (a documented scope decision -- pack-bay/gear-bay overheat
/// protection is a different real system in its own right, left for a
/// future pass rather than invented here).
pub const ZONE_NAMES: [&str; 9] = [
    "PylonEngine1",
    "PylonEngine2",
    "PylonEngine3",
    "PylonEngine4",
    "TailCone",
    "WingLeLeft",
    "WingLeRight",
    "BellyFairingPacks",
    "WingGearWell",
];
const PYLON: [usize; 4] = [0, 1, 2, 3];
const TAIL_CONE: usize = 4;
const WING_LE: [usize; 2] = [5, 6];
const BELLY_FAIRING_PACKS: usize = 7;
const WING_GEAR_WELL: usize = 8;
pub const ZONE_COUNT: usize = 9;
pub const ODLS_ZONE_COUNT: usize = 7;

/// This zone's own absolute ODLS alarm temperature (`odls.rs`'s own module
/// doc for the derivation): the pylon bays and the APU's tail-cone bay run
/// hot in normal operation from engine/APU proximity, the same
/// "pylon/strut" compartment class `thermal_zones::PROGRESS.md`'s
/// investigation names; the wing leading-edge duct runs are the cooler
/// "wing/fuselage" class.
fn odls_threshold_k(zone: usize) -> f64 {
    if zone == TAIL_CONE || PYLON.contains(&zone) {
        OverheatDetectionLoop::THRESHOLD_PYLON_STRUT_K
    } else {
        OverheatDetectionLoop::THRESHOLD_WING_FUSELAGE_K
    }
}

// ---------------------------------------------------------------------------
// Upstream stage constants (IP tap, HP valve, PR/shutoff valve).
// ---------------------------------------------------------------------------

/// IP8 tap effective area: **GENERIC** (FBW's own IP valve uses an
/// exponential-relaxation model with no explicit orifice area, module
/// docs), sized to the same order of magnitude as the cited HP/PR valve
/// areas below (the same class of large bleed duct).
const IP_TAP_AREA_M2: f64 = 0.008;
/// The IP tap's spring characteristic: matches FBW's own `PurelyPneumatic
/// Valve`/`SPRING_CHARACTERISTIC = 1.` against a pressure difference in
/// psi (`passive_valve_open_fraction`'s own citation, `duct.rs`).
const IP_TAP_SPRING_PA: f64 = 6894.757;
/// HP6 valve orifice area: cited, `a380_systems/pneumatic.rs:908`
/// (`HP_VALVE_ORIFICE_AREA_M2`).
const HP_VALVE_AREA_M2: f64 = 0.006207;
/// PR/shutoff valve orifice area: cited, `a380_systems/pneumatic.rs:910`
/// (`PR_VALVE_ORIFICE_AREA_M2`).
const PR_VALVE_AREA_M2: f64 = 0.008107;
/// Discharge coefficient for both: cited, `a380_systems/pneumatic.rs:909,911`.
const VALVE_CD: f64 = 0.65;
/// EASA TCDS E.012 section 10's public IP8/HP6 switch-over pressure (public
/// aircraft data sheet, cited already by `a380_systems/pneumatic.rs:696`
/// and this crate's own `physics::engine::mod.rs` `SWITCH_OVER_PA` test):
/// bleed comes off IP8 whenever its own port pressure is above this: the HP
/// valve stays shut and the passive IP tap alone regulates.
const IP_SWITCHOVER_PA: f64 = 206_800.0;
/// FBW's own HP valve interlock: it never opens with a weak HP6 source
/// (`a380_systems/pneumatic.rs:794`, `psi(15.)`).
const HP_VALVE_MIN_HP_PORT_PA: f64 = 15.0 * 6894.757;
/// FBW's own PR valve interlock: it never opens against a weak transfer
/// pressure (`a380_systems/pneumatic.rs:807`, `psi(15.)`).
const PR_VALVE_MIN_TRANSFER_PA: f64 = 15.0 * 6894.757;
/// Regulation target both valves aim their downstream pressure at: cited,
/// `a380_systems/pneumatic.rs:692` (`PRESSURE_REGULATING_VALVE_TARGET_PSI
/// = 40.`, "FCOM").
const REGULATION_TARGET_PA: f64 = 40.0 * 6894.757;
/// **GENERIC** proportional gain for this module's own (not FBW's PID)
/// regulation law: full authority ~20 psi below target.
const VALVE_GAIN_PER_PA: f64 = 1.0 / (20.0 * 6894.757);
/// **GENERIC** valve actuator lag, same role as `precooler.rs`'s FAV lag
/// (damps the one-tick-delayed proportional loop).
const VALVE_ACTUATOR_TIME_CONSTANT_S: f64 = 1.0;
/// Transfer pipe volume: cited, `a380_systems/pneumatic.rs:991-995`
/// (`transfer_pressure_pipe`, `Volume::new::<cubic_meter>(1.)`).
const TRANSFER_PIPE_VOLUME_M3: f64 = 1.0;

// ---------------------------------------------------------------------------
// Cross-bleed / consumer valve areas.
// ---------------------------------------------------------------------------

/// Cross-bleed valve area: **GENERIC** (FBW does not cite an explicit
/// orifice area for `CrossBleedValve`), same order of magnitude as the
/// engine's own bleed valves (the same class of large-transport bleed duct).
const CROSSBLEED_AREA_M2: f64 = 0.006;
/// APU bleed valve area: **GENERIC**, comparable to one engine's own HP
/// valve (the APU load compressor's bleed is sized to support a full
/// single-pack/single-engine-start load).
const APU_VALVE_AREA_M2: f64 = 0.005;
/// Pack flow-control-valve area, per valve (two per pack now, module
/// docs): **GENERIC**.
const PACK_VALVE_AREA_M2: f64 = 0.0035;
/// Wing anti-ice valve: cited geometry, "Flow Trimming Restrictor 47 mm
/// diameter", the A320's own public WAI duct spec (`fbw-a32nx/.../
/// pneumatic/wing_anti_ice.rs:333`).
const WAI_VALVE_AREA_M2: f64 = 0.001735;
/// Engine start valve: **GENERIC**, same order of magnitude as the
/// engine's own HP bleed valve.
const START_VALVE_AREA_M2: f64 = 0.006;
/// The start duct's own non-return valve seat area: **GENERIC**.
const START_CHECK_VALVE_SEAT_AREA_M2: f64 = 0.006;
/// Hydraulic reservoir pressurisation orifice: **GENERIC**, a small fixed
/// restrictor.
const HYD_RESERVOIR_ORIFICE_AREA_M2: f64 = 0.0002;

/// Faults on one engine's upstream HP/IP/PR valve stage (0.0 = healthy ..
/// 1.0 = fully failed).
#[derive(Clone, Copy, Debug, Default)]
pub struct UpstreamFaults {
    /// The HP valve is seized at whatever position it last held.
    pub hp_valve_stuck: f64,
    /// The PR/shutoff valve is seized at whatever position it last held.
    pub pr_valve_stuck: f64,
    /// The IP8 tap's passive check valve is stuck toward closed, forcing
    /// reliance on the HP valve (0 healthy .. 1 fully stuck shut).
    pub ip_check_valve_stuck_closed: f64,
}

/// Faults on every duct section, precooler, upstream stage and ODLS zone
/// this network owns (0.0 = healthy .. 1.0 = fully failed everywhere).
#[derive(Clone, Debug, Default)]
pub struct DuctNetworkFaults {
    pub upstream: [UpstreamFaults; 4],
    pub engine_duct: [DuctSectionFaults; 4],
    pub engine_precooler: [PrecoolerFaults; 4],
    pub apu_duct: DuctSectionFaults,
    pub apu_precooler: PrecoolerFaults,
    pub packs: [DuctSectionFaults; 2],
    pub wai: [DuctSectionFaults; 2],
    pub start: [DuctSectionFaults; 4],
    /// The engine start duct's own non-return valve (module docs): 0.0
    /// healthy, 1.0 fully failed open (backflow as free as forward flow).
    pub start_check_valve_failure: [f64; 4],
    pub hyd_reservoir: [DuctSectionFaults; 2],
    pub odls: [OdlsFaults; ODLS_ZONE_COUNT],
}

/// One engine's published bleed port conditions and precooler cooling
/// supply -- external inputs (module docs).
#[derive(Clone, Copy, Debug)]
pub struct EngineBleedInput {
    /// `physics::engine::EngineOutputs::ip_port_pressure_pa`/`_temp_k`
    /// (read-only reference, not imported): the IP compressor's own total
    /// pressure/temperature at IP8, upstream of any valve.
    pub ip_port_pressure_pa: f64,
    pub ip_port_temp_k: f64,
    /// `EngineOutputs::hp_port_pressure_pa`/`_temp_k`: the HP compressor's
    /// own conditions at HP6.
    pub hp_port_pressure_pa: f64,
    pub hp_port_temp_k: f64,
    /// Bypass (fan) air available to this engine's own precooler, kg/s
    /// (conceptually `EngineOutputs::bypass_mdot_kg_s`).
    pub fan_air_available_kg_s: f64,
    pub fan_air_k: f64,
}

/// The APU's own bleed port condition (a single load-compressor discharge,
/// no separate IP/HP split -- module docs).
#[derive(Clone, Copy, Debug)]
pub struct ApuBleedInput {
    pub pressure_pa: f64,
    pub temp_k: f64,
    pub fan_air_available_kg_s: f64,
    pub fan_air_k: f64,
}

/// One tick's inputs.
pub struct NetworkInputs {
    pub dt_s: f64,
    pub ambient_pa: f64,
    pub ambient_k: f64,
    pub engines: [EngineBleedInput; 4],
    pub apu: ApuBleedInput,
    pub apu_bleed_selected: bool,
    /// APU bleed valve command, 0..1, before this network's own ODLS override.
    pub apu_bleed_valve_command: f64,
    /// Left/Centre/Right cross-bleed valve commands, 0..1 (FBW valve
    /// numbers 9/10/11), before this network's own ODLS override.
    pub cross_bleed_valve_command: [f64; 3],
    /// `[pack][side]`: pack 1 from engines 1/2, pack 2 from engines 3/4.
    pub pack_valve_open: [[f64; 2]; 2],
    pub wai_selected: [bool; 2],
    pub starter_engaged: [bool; 4],
    /// The real ENG BLEED pushbutton, per engine: `false` (OFF) shuts that
    /// engine's own PR/shutoff valve -- the literal valve that pushbutton
    /// switches (module docs: "the real 'ENG n BLEED' pushbutton's own
    /// valve") -- and every cross-bleed/pack/wing-anti-ice/hydraulic tap
    /// fed from its duct, the same way a confirmed ODLS trip already does,
    /// without latching: unlike a trip, toggling the pushbutton back on
    /// restores the source immediately.
    pub engine_bleed_pb_auto: [bool; 4],
    /// This tick's zone air temperature, K, indexed by `ZONE_NAMES`.
    pub zone_air_k: [f64; ZONE_COUNT],
}

/// One tick's outputs.
#[derive(Clone, Copy, Debug, Default)]
pub struct NetworkOutputs {
    /// Heat this tick delivered to each zone (leak/rupture enthalpy +
    /// insulation loss), W -- indexed by `ZONE_NAMES`, the exact input a
    /// future integration pass hands to `ThermalNetwork::inject_heat_w`.
    pub zone_heat_w: [f64; ZONE_COUNT],
    /// **Structural/wiring damage interface** (item 2): the worst
    /// (`max`, an intensity, not summed) impinging-jet local heat flux,
    /// W/m^2, any duct section in that zone reported this tick -- see
    /// `leak::LeakResult::jet_impact_flux_w_m2`'s own docs. Zero unless a
    /// rupture past the jet-impingement onset is actually present.
    pub jet_impact_flux_w_m2: [f64; ZONE_COUNT],
    pub odls_trip: [bool; ODLS_ZONE_COUNT],
    pub odls_loop_fault: [bool; ODLS_ZONE_COUNT],
    /// Each loop's own health, independent of the other (module doc on
    /// `odls::OdlsOutputs`'s identically-named fields): a single loop
    /// failing open must be visible on its own even though, correctly, it
    /// cannot move `odls_loop_fault`/`odls_trip` while its twin stays
    /// healthy.
    pub odls_loop_a_fault: [bool; ODLS_ZONE_COUNT],
    pub odls_loop_b_fault: [bool; ODLS_ZONE_COUNT],
    pub engine_isolated: [bool; 4],
    pub apu_isolated: bool,
    pub cross_bleed_valve_open: [f64; 3],
    pub apu_bleed_valve_open: f64,
    pub hp_valve_open: [f64; 4],
    pub pr_valve_open: [f64; 4],
    pub transfer_pipe_pressure_pa: [f64; 4],
    pub engine_duct_pressure_pa: [f64; 4],
    pub pack_supply_pressure_pa: [f64; 2],
    pub wai_duct_pressure_pa: [f64; 2],
    pub wai_valve_open: [f64; 2],
    pub start_duct_pressure_pa: [f64; 4],
    pub hyd_reservoir_pressure_pa: [f64; 2],
    pub engine_precooler_overtemp: [bool; 4],
    pub apu_precooler_overtemp: bool,
    /// Real mass flow actually being drawn from the APU's own load
    /// compressor this tick, kg/s -- the precooler's own hot-side flow
    /// through [`APU_VALVE_AREA_M2`], computed from the real
    /// `ApuBleedInput` port condition, zero whenever `apu_bleed_selected`
    /// is false. `deep::apu` needs this to know its load compressor is
    /// actually loaded (otherwise its own erosion/surge-control-valve/IGV
    /// failures have nothing to act on): published as
    /// `PNEU_APU_BLEED_DEMAND_KG_S` (`live.rs`).
    pub apu_bleed_demand_kg_s: f64,
    /// Each precooler's own delivered outlet temperature, K -- the
    /// quantity its fouling/FAV/sensor faults actually act on, and what a
    /// bleed page or Study panel shows. (`overtemp` above is only the
    /// binary trip that temperature crosses.)
    pub engine_precooler_outlet_k: [f64; 4],
    pub apu_precooler_outlet_k: f64,
    /// Gas temperature in each duct volume, K, alongside the pressures
    /// above: a duct's state is (P, T), and insulation-damage/leak faults
    /// move the temperature as much as the pressure.
    pub engine_duct_temp_k: [f64; 4],
    pub apu_duct_temp_k: f64,
    pub pack_supply_temp_k: [f64; 2],
    pub wai_duct_temp_k: [f64; 2],
}

pub struct DuctNetwork {
    engine_precooler: [Precooler; 4],
    transfer_pipe: [DuctVolume; 4],
    hp_valve_open: [f64; 4],
    pr_valve_open: [f64; 4],
    engine_duct: [DuctSection; 4],
    apu_precooler: Precooler,
    apu_duct: DuctSection,
    packs: [DuctSection; 2],
    wai: [DuctSection; 2],
    start: [DuctSection; 4],
    hyd_reservoir: [DuctSection; 2],
    odls: [OverheatDetectionLoop; ODLS_ZONE_COUNT],
    engine_isolated: [bool; 4],
    apu_isolated: bool,
    wai_valve_open: [f64; 2],
    wai_valve_pid_state: [f64; 2],
}

impl DuctNetwork {
    const START_PA: f64 = 101_325.0;
    const START_K: f64 = 288.15;
    const ENGINE_DUCT_VOLUME_M3: f64 = 2.5; // matches FBW's own `precooler_outlet_pipe`, pneumatic.rs:1001-1005
    const ENGINE_DUCT_DIAMETER_M: f64 = 0.1016; // 4 in, `a380_systems/pneumatic.rs:900-910`'s own cited bleed duct diameter range
    const ENGINE_DUCT_UA_W_K: f64 = 8.0;
    const APU_DUCT_VOLUME_M3: f64 = 0.5;
    const APU_DUCT_DIAMETER_M: f64 = 0.08;
    const APU_DUCT_UA_W_K: f64 = 6.0;
    const PACK_DUCT_VOLUME_M3: f64 = 0.4;
    const PACK_DUCT_DIAMETER_M: f64 = 0.1;
    const PACK_DUCT_UA_W_K: f64 = 5.0;
    const WAI_DUCT_VOLUME_M3: f64 = 2.0; // cited, `wing_anti_ice.rs:349`, `WAI_PIPE_VOLUME`
    const WAI_DUCT_DIAMETER_M: f64 = 0.05;
    const WAI_DUCT_UA_W_K: f64 = 4.0;
    const START_DUCT_VOLUME_M3: f64 = 0.5;
    const START_DUCT_DIAMETER_M: f64 = 0.09;
    const START_DUCT_UA_W_K: f64 = 5.0;
    const HYD_RESERVOIR_DUCT_VOLUME_M3: f64 = 0.05;
    const HYD_RESERVOIR_DUCT_DIAMETER_M: f64 = 0.03;
    const HYD_RESERVOIR_DUCT_UA_W_K: f64 = 3.0;

    pub fn new() -> Self {
        let p = Self::START_PA;
        let k = Self::START_K;
        Self {
            engine_precooler: [Precooler::new(); 4],
            transfer_pipe: [DuctVolume::new(TRANSFER_PIPE_VOLUME_M3, p, k); 4],
            hp_valve_open: [0.0; 4],
            pr_valve_open: [0.0; 4],
            engine_duct: std::array::from_fn(|i| {
                DuctSection::new(ZONE_NAMES[PYLON[i]], Self::ENGINE_DUCT_VOLUME_M3, Self::ENGINE_DUCT_DIAMETER_M, Self::ENGINE_DUCT_UA_W_K, p, k)
            }),
            apu_precooler: Precooler::new(),
            apu_duct: DuctSection::new(ZONE_NAMES[TAIL_CONE], Self::APU_DUCT_VOLUME_M3, Self::APU_DUCT_DIAMETER_M, Self::APU_DUCT_UA_W_K, p, k),
            packs: std::array::from_fn(|_| DuctSection::new(ZONE_NAMES[BELLY_FAIRING_PACKS], Self::PACK_DUCT_VOLUME_M3, Self::PACK_DUCT_DIAMETER_M, Self::PACK_DUCT_UA_W_K, p, k)),
            wai: std::array::from_fn(|i| DuctSection::new(ZONE_NAMES[WING_LE[i]], Self::WAI_DUCT_VOLUME_M3, Self::WAI_DUCT_DIAMETER_M, Self::WAI_DUCT_UA_W_K, p, k)),
            start: std::array::from_fn(|i| DuctSection::new(ZONE_NAMES[PYLON[i]], Self::START_DUCT_VOLUME_M3, Self::START_DUCT_DIAMETER_M, Self::START_DUCT_UA_W_K, p, k)),
            hyd_reservoir: std::array::from_fn(|_| DuctSection::new(ZONE_NAMES[WING_GEAR_WELL], Self::HYD_RESERVOIR_DUCT_VOLUME_M3, Self::HYD_RESERVOIR_DUCT_DIAMETER_M, Self::HYD_RESERVOIR_DUCT_UA_W_K, p, k)),
            odls: std::array::from_fn(|z| OverheatDetectionLoop::new(k, odls_threshold_k(z))),
            engine_isolated: [false; 4],
            apu_isolated: false,
            wai_valve_open: [0.0; 2],
            wai_valve_pid_state: [0.0; 2],
        }
    }

    /// Apply one duct section's leak/rupture and insulation loss. Returns
    /// `(heat_to_zone_w, jet_impact_flux_w_m2)`; the caller sums the first
    /// (extensive) and takes the max of the second (an intensity) across
    /// every section sharing a zone.
    fn apply_faults(section: &mut DuctSection, ambient_pa: f64, zone_air_k: f64, dt_s: f64, faults: &DuctSectionFaults) -> (f64, f64) {
        let leak_result = leak::step(section, ambient_pa, zone_air_k, faults);
        let (t, p) = (section.gas.temp_k(), section.gas.pressure_pa());
        section.gas.add_mass(-leak_result.mass_flow_kg_s * dt_s, t, p);
        let insulation_w = section.step_insulation(zone_air_k, dt_s, faults);
        (leak_result.heat_to_zone_w + insulation_w.max(0.0), leak_result.jet_impact_flux_w_m2)
    }

    fn credit(out: &mut NetworkOutputs, zone: usize, heat_w: f64, flux_w_m2: f64) {
        out.zone_heat_w[zone] += heat_w;
        out.jet_impact_flux_w_m2[zone] = out.jet_impact_flux_w_m2[zone].max(flux_w_m2);
    }

    pub fn step(&mut self, inputs: &NetworkInputs, faults: &DuctNetworkFaults) -> NetworkOutputs {
        let dt = inputs.dt_s.max(0.0);
        let mut out = NetworkOutputs::default();

        // --- ODLS first (confirm-delayed, reflects last tick's zone
        // condition consistently before any flow moves this tick).
        for z in 0..ODLS_ZONE_COUNT {
            let o = self.odls[z].step(inputs.zone_air_k[z], dt, &faults.odls[z]);
            out.odls_trip[z] = o.trip;
            out.odls_loop_fault[z] = o.loop_fault;
            out.odls_loop_a_fault[z] = o.loop_a_fault;
            out.odls_loop_b_fault[z] = o.loop_b_fault;
        }
        for i in 0..4 {
            if out.odls_trip[PYLON[i]] {
                self.engine_isolated[i] = true;
            }
        }
        if out.odls_trip[TAIL_CONE] {
            self.apu_isolated = true;
        }
        out.engine_isolated = self.engine_isolated;
        out.apu_isolated = self.apu_isolated;
        let isolated = self.engine_isolated;
        let apu_isolated = self.apu_isolated;
        // The ENG BLEED pushbutton's own effect on gas flow (module docs on
        // `NetworkInputs::engine_bleed_pb_auto`): folded into every gate
        // below that already checks `isolated[i]`, but kept a *local*,
        // non-latching condition -- it must never feed `self.engine_
        // isolated`/`out.engine_isolated`, which are specifically the ODLS
        // trip's own latch and annunciation.
        let source_shut = |i: usize| isolated[i] || !inputs.engine_bleed_pb_auto[i];

        // --- Upstream: IP tap + HP valve -> transfer pipe -> PR valve ->
        // precooler -> engine local duct, per engine (module docs).
        for i in 0..4 {
            let ports = &inputs.engines[i];
            let uf = &faults.upstream[i];

            let ip_diff = ports.ip_port_pressure_pa - self.transfer_pipe[i].pressure_pa();
            let ip_frac = passive_valve_open_fraction(ip_diff, IP_TAP_SPRING_PA) * (1.0 - uf.ip_check_valve_stuck_closed.clamp(0.0, 1.0));
            let ip_mdot = orifice_mass_flow_kg_s(VALVE_CD, IP_TAP_AREA_M2 * ip_frac, ports.ip_port_pressure_pa, ports.ip_port_temp_k, self.transfer_pipe[i].pressure_pa());
            self.transfer_pipe[i].add_mass(ip_mdot * dt, ports.ip_port_temp_k, ports.ip_port_pressure_pa);

            let hp_commanded = if ports.ip_port_pressure_pa > IP_SWITCHOVER_PA || ports.hp_port_pressure_pa < HP_VALVE_MIN_HP_PORT_PA {
                0.0
            } else {
                ((REGULATION_TARGET_PA - self.transfer_pipe[i].pressure_pa()) * VALVE_GAIN_PER_PA).clamp(0.0, 1.0)
            };
            if uf.hp_valve_stuck < 0.5 {
                let k = 1.0 / VALVE_ACTUATOR_TIME_CONSTANT_S;
                self.hp_valve_open[i] = hp_commanded + (self.hp_valve_open[i] - hp_commanded) * (-k * dt).exp();
            }
            let hp_mdot = orifice_mass_flow_kg_s(VALVE_CD, HP_VALVE_AREA_M2 * self.hp_valve_open[i].clamp(0.0, 1.0), ports.hp_port_pressure_pa, ports.hp_port_temp_k, self.transfer_pipe[i].pressure_pa());
            self.transfer_pipe[i].add_mass(hp_mdot * dt, ports.hp_port_temp_k, ports.hp_port_pressure_pa);

            let pr_commanded = if source_shut(i) || inputs.starter_engaged[i] || self.transfer_pipe[i].pressure_pa() < PR_VALVE_MIN_TRANSFER_PA {
                0.0
            } else {
                ((REGULATION_TARGET_PA - self.engine_duct[i].gas.pressure_pa()) * VALVE_GAIN_PER_PA).clamp(0.0, 1.0)
            };
            if uf.pr_valve_stuck < 0.5 {
                let k = 1.0 / VALVE_ACTUATOR_TIME_CONSTANT_S;
                self.pr_valve_open[i] = pr_commanded + (self.pr_valve_open[i] - pr_commanded) * (-k * dt).exp();
            }
            let pr_mdot = orifice_mass_flow_kg_s(VALVE_CD, PR_VALVE_AREA_M2 * self.pr_valve_open[i].clamp(0.0, 1.0), self.transfer_pipe[i].pressure_pa(), self.transfer_pipe[i].temp_k(), self.engine_duct[i].gas.pressure_pa());
            let pr_source_temp = self.transfer_pipe[i].temp_k();
            let pr_source_pa = self.transfer_pipe[i].pressure_pa();
            self.transfer_pipe[i].add_mass(-pr_mdot * dt, pr_source_temp, pr_source_pa);

            let pc = self.engine_precooler[i].step(dt, pr_source_temp, pr_mdot, ports.fan_air_available_kg_s, ports.fan_air_k, &faults.engine_precooler[i]);
            self.engine_duct[i].gas.add_mass(pr_mdot * dt, pc.outlet_temp_k, pr_source_pa);
            out.engine_precooler_overtemp[i] = pc.overtemp_active;
            out.engine_precooler_outlet_k[i] = pc.outlet_temp_k;
            out.hp_valve_open[i] = self.hp_valve_open[i];
            out.pr_valve_open[i] = self.pr_valve_open[i];
            out.transfer_pipe_pressure_pa[i] = self.transfer_pipe[i].pressure_pa();

            let (relief, backflow) = self.engine_precooler[i].relief_and_backflow_kg_s(self.engine_duct[i].gas.pressure_pa(), inputs.ambient_pa, &faults.engine_precooler[i]);
            let (t, p) = (self.engine_duct[i].gas.temp_k(), self.engine_duct[i].gas.pressure_pa());
            self.engine_duct[i].gas.add_mass(-relief * dt, t, p);
            self.engine_duct[i].gas.add_mass(backflow * dt, inputs.ambient_k, inputs.ambient_pa);

            let (heat, flux) = Self::apply_faults(&mut self.engine_duct[i], inputs.ambient_pa, inputs.zone_air_k[PYLON[i]], dt, &faults.engine_duct[i]);
            Self::credit(&mut out, PYLON[i], heat, flux);
            out.engine_duct_pressure_pa[i] = self.engine_duct[i].gas.pressure_pa();
            out.engine_duct_temp_k[i] = self.engine_duct[i].gas.temp_k();
        }

        // --- APU's own stage: source -> precooler -> APU duct (always
        // flows, same "upstream of isolation" principle as the engines').
        {
            let src = &inputs.apu;
            let mdot_hot = if inputs.apu_bleed_selected {
                orifice_mass_flow_kg_s(VALVE_CD, APU_VALVE_AREA_M2, src.pressure_pa, src.temp_k, self.apu_duct.gas.pressure_pa())
            } else {
                0.0
            };
            let pc = self.apu_precooler.step(dt, src.temp_k, mdot_hot, src.fan_air_available_kg_s, src.fan_air_k, &faults.apu_precooler);
            self.apu_duct.gas.add_mass(mdot_hot * dt, pc.outlet_temp_k, src.pressure_pa);
            out.apu_precooler_overtemp = pc.overtemp_active;
            out.apu_bleed_demand_kg_s = mdot_hot;
            out.apu_precooler_outlet_k = pc.outlet_temp_k;

            let (relief, backflow) = self.apu_precooler.relief_and_backflow_kg_s(self.apu_duct.gas.pressure_pa(), inputs.ambient_pa, &faults.apu_precooler);
            let (t, p) = (self.apu_duct.gas.temp_k(), self.apu_duct.gas.pressure_pa());
            self.apu_duct.gas.add_mass(-relief * dt, t, p);
            self.apu_duct.gas.add_mass(backflow * dt, inputs.ambient_k, inputs.ambient_pa);

            let (heat, flux) = Self::apply_faults(&mut self.apu_duct, inputs.ambient_pa, inputs.zone_air_k[TAIL_CONE], dt, &faults.apu_duct);
            Self::credit(&mut out, TAIL_CONE, heat, flux);
        }

        // --- Wing anti-ice control (computed before the shared mutable
        // borrow below, since it only reads `self.wai`/`self.wai_valve_*`,
        // disjoint fields from `self.engine_duct`).
        for i in 0..2 {
            let zone = WING_LE[i];
            let auto_closed = out.odls_trip[zone];
            self.wai_valve_open[i] = if !inputs.wai_selected[i] || auto_closed {
                0.0
            } else {
                // GENERIC proportional regulator toward a duct target
                // pressure, the same general control concept the A320's
                // own public `WingAntiIceValveController` uses (target
                // `ambient + 22.5 psi`, `wing_anti_ice.rs:79-86`).
                const TARGET_ABOVE_AMBIENT_PA: f64 = 22.5 * 6894.757;
                const GAIN_PER_PA: f64 = 1.0 / (10.0 * 6894.757);
                let error_pa = (inputs.ambient_pa + TARGET_ABOVE_AMBIENT_PA) - self.wai[i].gas.pressure_pa();
                let commanded = (error_pa * GAIN_PER_PA).clamp(0.0, 1.0);
                const TAU_S: f64 = 2.0;
                let a = (-dt / TAU_S).exp();
                self.wai_valve_pid_state[i] = commanded + (self.wai_valve_pid_state[i] - commanded) * a;
                self.wai_valve_pid_state[i].clamp(0.0, 1.0)
            };
        }

        // --- Cross-bleed (L/C/R), APU->engine 1, packs (dual feed), wing
        // anti-ice and hydraulic reservoir taps, all off the 4 engine
        // ducts directly (module docs: FBW's own topology, no manifold).
        // One shared mutable borrow of `engine_duct` for this whole block;
        // every other field touched here is disjoint from it.
        {
            let [d0, d1, d2, d3] = &mut self.engine_duct;

            let l_open = if source_shut(0) || source_shut(1) { 0.0 } else { inputs.cross_bleed_valve_command[0].clamp(0.0, 1.0) };
            let c_open = if source_shut(0) || source_shut(3) { 0.0 } else { inputs.cross_bleed_valve_command[1].clamp(0.0, 1.0) };
            let r_open = if source_shut(2) || source_shut(3) { 0.0 } else { inputs.cross_bleed_valve_command[2].clamp(0.0, 1.0) };
            transfer_kg(dt, VALVE_CD, CROSSBLEED_AREA_M2 * l_open, &mut d0.gas, &mut d1.gas);
            transfer_kg(dt, VALVE_CD, CROSSBLEED_AREA_M2 * c_open, &mut d0.gas, &mut d3.gas);
            transfer_kg(dt, VALVE_CD, CROSSBLEED_AREA_M2 * r_open, &mut d2.gas, &mut d3.gas);
            out.cross_bleed_valve_open = [l_open, c_open, r_open];

            let apu_open = if apu_isolated || source_shut(0) { 0.0 } else { inputs.apu_bleed_valve_command.clamp(0.0, 1.0) };
            out.apu_bleed_valve_open = apu_open;
            transfer_kg(dt, VALVE_CD, APU_VALVE_AREA_M2 * apu_open, &mut self.apu_duct.gas, &mut d0.gas);

            let pack1_from_1 = if source_shut(0) { 0.0 } else { inputs.pack_valve_open[0][0].clamp(0.0, 1.0) };
            let pack1_from_2 = if source_shut(1) { 0.0 } else { inputs.pack_valve_open[0][1].clamp(0.0, 1.0) };
            transfer_kg(dt, VALVE_CD, PACK_VALVE_AREA_M2 * pack1_from_1, &mut d0.gas, &mut self.packs[0].gas);
            transfer_kg(dt, VALVE_CD, PACK_VALVE_AREA_M2 * pack1_from_2, &mut d1.gas, &mut self.packs[0].gas);

            let pack2_from_3 = if source_shut(2) { 0.0 } else { inputs.pack_valve_open[1][0].clamp(0.0, 1.0) };
            let pack2_from_4 = if source_shut(3) { 0.0 } else { inputs.pack_valve_open[1][1].clamp(0.0, 1.0) };
            transfer_kg(dt, VALVE_CD, PACK_VALVE_AREA_M2 * pack2_from_3, &mut d2.gas, &mut self.packs[1].gas);
            transfer_kg(dt, VALVE_CD, PACK_VALVE_AREA_M2 * pack2_from_4, &mut d3.gas, &mut self.packs[1].gas);

            let wai_left_open = if source_shut(1) { 0.0 } else { self.wai_valve_open[0] };
            let wai_right_open = if source_shut(2) { 0.0 } else { self.wai_valve_open[1] };
            transfer_kg(dt, VALVE_CD, WAI_VALVE_AREA_M2 * wai_left_open, &mut d1.gas, &mut self.wai[0].gas);
            transfer_kg(dt, VALVE_CD, WAI_VALVE_AREA_M2 * wai_right_open, &mut d2.gas, &mut self.wai[1].gas);

            let starter_forward = |engaged: bool| if engaged { START_VALVE_AREA_M2 } else { 0.0 };
            one_way_transfer_kg(dt, VALVE_CD, starter_forward(inputs.starter_engaged[0]), START_CHECK_VALVE_SEAT_AREA_M2, &mut d0.gas, &mut self.start[0].gas, faults.start_check_valve_failure[0]);
            one_way_transfer_kg(dt, VALVE_CD, starter_forward(inputs.starter_engaged[1]), START_CHECK_VALVE_SEAT_AREA_M2, &mut d1.gas, &mut self.start[1].gas, faults.start_check_valve_failure[1]);
            one_way_transfer_kg(dt, VALVE_CD, starter_forward(inputs.starter_engaged[2]), START_CHECK_VALVE_SEAT_AREA_M2, &mut d2.gas, &mut self.start[2].gas, faults.start_check_valve_failure[2]);
            one_way_transfer_kg(dt, VALVE_CD, starter_forward(inputs.starter_engaged[3]), START_CHECK_VALVE_SEAT_AREA_M2, &mut d3.gas, &mut self.start[3].gas, faults.start_check_valve_failure[3]);

            transfer_kg(dt, VALVE_CD, HYD_RESERVOIR_ORIFICE_AREA_M2, &mut d0.gas, &mut self.hyd_reservoir[0].gas);
            transfer_kg(dt, VALVE_CD, HYD_RESERVOIR_ORIFICE_AREA_M2, &mut d3.gas, &mut self.hyd_reservoir[1].gas);
        }

        // The cross-bleed/consumer block above moved mass between the four
        // engine ducts, so their published state is refreshed here rather
        // than left at the pre-cross-bleed value the upstream loop set.
        for i in 0..4 {
            out.engine_duct_pressure_pa[i] = self.engine_duct[i].gas.pressure_pa();
            out.engine_duct_temp_k[i] = self.engine_duct[i].gas.temp_k();
        }
        out.apu_duct_temp_k = self.apu_duct.gas.temp_k();

        for i in 0..2 {
            let (heat, flux) = Self::apply_faults(&mut self.packs[i], inputs.ambient_pa, inputs.zone_air_k[BELLY_FAIRING_PACKS], dt, &faults.packs[i]);
            Self::credit(&mut out, BELLY_FAIRING_PACKS, heat, flux);
            out.pack_supply_pressure_pa[i] = self.packs[i].gas.pressure_pa();
            out.pack_supply_temp_k[i] = self.packs[i].gas.temp_k();
        }
        for i in 0..2 {
            let zone = WING_LE[i];
            let (heat, flux) = Self::apply_faults(&mut self.wai[i], inputs.ambient_pa, inputs.zone_air_k[zone], dt, &faults.wai[i]);
            Self::credit(&mut out, zone, heat, flux);
            out.wai_duct_pressure_pa[i] = self.wai[i].gas.pressure_pa();
            out.wai_duct_temp_k[i] = self.wai[i].gas.temp_k();
            out.wai_valve_open[i] = self.wai_valve_open[i];
        }
        for i in 0..4 {
            let (heat, flux) = Self::apply_faults(&mut self.start[i], inputs.ambient_pa, inputs.zone_air_k[PYLON[i]], dt, &faults.start[i]);
            Self::credit(&mut out, PYLON[i], heat, flux);
            out.start_duct_pressure_pa[i] = self.start[i].gas.pressure_pa();
        }
        for i in 0..2 {
            let (heat, flux) = Self::apply_faults(&mut self.hyd_reservoir[i], inputs.ambient_pa, inputs.zone_air_k[WING_GEAR_WELL], dt, &faults.hyd_reservoir[i]);
            Self::credit(&mut out, WING_GEAR_WELL, heat, flux);
            out.hyd_reservoir_pressure_pa[i] = self.hyd_reservoir[i].gas.pressure_pa();
        }

        out
    }
}

impl Default for DuctNetwork {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn healthy_engine() -> EngineBleedInput {
        // IP8 above the switch-over pressure: the HP valve should stay
        // shut and the passive IP tap alone should regulate.
        EngineBleedInput { ip_port_pressure_pa: 260_000.0, ip_port_temp_k: 400.0, hp_port_pressure_pa: 550_000.0, hp_port_temp_k: 600.0, fan_air_available_kg_s: 60.0, fan_air_k: 288.15 }
    }
    fn not_running_engine() -> EngineBleedInput {
        EngineBleedInput { ip_port_pressure_pa: 40_000.0, ip_port_temp_k: 288.15, hp_port_pressure_pa: 40_000.0, hp_port_temp_k: 288.15, fan_air_available_kg_s: 0.0, fan_air_k: 288.15 }
    }

    fn base_inputs() -> NetworkInputs {
        NetworkInputs {
            dt_s: 1.0,
            ambient_pa: 40_000.0,
            ambient_k: 250.0,
            engines: [healthy_engine(); 4],
            apu: ApuBleedInput { pressure_pa: 0.0, temp_k: 288.15, fan_air_available_kg_s: 0.0, fan_air_k: 288.15 },
            apu_bleed_selected: false,
            apu_bleed_valve_command: 0.0,
            cross_bleed_valve_command: [1.0, 1.0, 1.0],
            pack_valve_open: [[1.0, 1.0], [1.0, 1.0]],
            wai_selected: [false, false],
            starter_engaged: [false; 4],
            engine_bleed_pb_auto: [true; 4],
            zone_air_k: [250.0; ZONE_COUNT],
        }
    }

    #[test]
    fn a_healthy_engine_pressurises_its_own_duct_and_the_packs_via_ip8_alone() {
        let mut net = DuctNetwork::new();
        let inputs = base_inputs();
        let mut out = NetworkOutputs::default();
        for _ in 0..300 {
            out = net.step(&inputs, &DuctNetworkFaults::default());
        }
        assert!(out.engine_duct_pressure_pa[0] > 101_325.0, "engine duct must pressurise");
        assert!(out.pack_supply_pressure_pa[0] > 101_325.0 && out.pack_supply_pressure_pa[1] > 101_325.0);
        assert!(out.hp_valve_open[0] < 0.05, "IP8 is well above switch-over, the HP valve should stay essentially shut, got {}", out.hp_valve_open[0]);
        assert!(!out.odls_trip.iter().any(|&t| t));
    }

    #[test]
    fn the_hp_valve_opens_when_ip8_alone_cannot_hold_regulation() {
        let mut net = DuctNetwork::new();
        let mut inputs = base_inputs();
        // Engine 1 on its own: shut every cross-connection off its duct --
        // the three cross-bleed valves *and* both pack feeds, because pack
        // 1's dual feed is itself a path from engine 1's duct to engine
        // 2's. Without this, engine 1's three healthy neighbours simply
        // hold its duct at their own regulated pressure, engine 1's
        // regulators see no error, and the HP valve correctly has nothing
        // to do -- which is what the original version of this test was
        // actually measuring.
        inputs.cross_bleed_valve_command = [0.0, 0.0, 0.0];
        inputs.pack_valve_open = [[0.0, 0.0], [0.0, 0.0]];
        // IP8 below switch-over: the HP valve must take over.
        inputs.engines[0].ip_port_pressure_pa = 150_000.0;
        inputs.engines[0].hp_port_pressure_pa = 500_000.0;
        let mut out = NetworkOutputs::default();
        let mut peak_hp_open = 0.0_f64;
        for _ in 0..300 {
            out = net.step(&inputs, &DuctNetworkFaults::default());
            peak_hp_open = peak_hp_open.max(out.hp_valve_open[0]);
        }
        assert!(peak_hp_open > 0.1, "HP valve must open once IP8 is below the switch-over pressure, peak opening was {}", peak_hp_open);

        // What the valve *ends* at is not the evidence, and the original
        // version of this test asserting a steady-state opening was asking
        // for the one thing this regulator cannot do. `hp_commanded` is a
        // pure proportional term, `(REGULATION_TARGET_PA - transfer_pipe) *
        // VALVE_GAIN_PER_PA`, so its steady-state command against zero
        // error is zero by construction; and with the packs and the
        // cross-bleeds shut there is no consumer anywhere downstream to
        // keep an error alive. So the valve must open, do its work and
        // close again -- which is precisely the assertion pair below.
        assert!(out.hp_valve_open[0] < 0.05, "with the target made and nothing consuming air, the regulator must have closed again, got {}", out.hp_valve_open[0]);

        // The evidence the HP valve did the work is the pressure it left
        // behind, bracketed at both ends:
        //  - the IP tap is a *passive* non-return valve
        //    (`passive_valve_open_fraction`), so it can never lift the pipe
        //    above its own 150 kPa port pressure. Reaching the 40 psi
        //    (275.8 kPa) regulation target at all is only possible through
        //    the HP valve.
        //  - but the pipe must stay well below the 500 kPa HP port, or the
        //    valve would not be regulating at all, just sitting open until
        //    the pipe equalised with its source.
        // The pipe settles a little *above* the target rather than on it:
        // the loop (1 s actuator lag driving a 1 m^3 volume through a
        // 0.006207 m^2 valve at gain 1/20 psi) is underdamped -- linearised
        // about the target it is `u'' + u' + 5.81*u = 0`, i.e. zeta = 0.21
        // and ~50% overshoot of the initial 126 kPa error -- and because
        // the valve is one-way onto a volume with no consumer, the pipe
        // simply latches at that first overshoot peak (~340 kPa) instead of
        // oscillating back down. Hence `>= target`, not `== target`.
        let hp_port_pa = inputs.engines[0].hp_port_pressure_pa;
        assert!(out.transfer_pipe_pressure_pa[0] > 150_000.0, "the passive IP tap alone cannot lift the transfer pipe above its own 150 kPa port, got {} Pa", out.transfer_pipe_pressure_pa[0]);
        assert!(out.transfer_pipe_pressure_pa[0] >= REGULATION_TARGET_PA, "the HP valve must carry the transfer pipe up to the {} Pa regulation target, got {} Pa", REGULATION_TARGET_PA, out.transfer_pipe_pressure_pa[0]);
        assert!(out.transfer_pipe_pressure_pa[0] < 0.5 * (REGULATION_TARGET_PA + hp_port_pa), "the HP valve must regulate, not just equalise the pipe with its {} Pa source, got {} Pa", hp_port_pa, out.transfer_pipe_pressure_pa[0]);
        assert!(out.engine_duct_pressure_pa[0] > 150_000.0, "the engine must still pressurise its own duct through the HP valve, got {} Pa", out.engine_duct_pressure_pa[0]);
    }

    #[test]
    fn a_non_running_engine_is_pressurised_by_its_neighbour_through_the_left_cross_bleed_valve() {
        let mut net = DuctNetwork::new();
        let mut inputs = base_inputs();
        inputs.engines[1] = not_running_engine(); // engine 2 not running
        let mut out = NetworkOutputs::default();
        for _ in 0..300 {
            out = net.step(&inputs, &DuctNetworkFaults::default());
        }
        assert!(out.engine_duct_pressure_pa[1] > 101_325.0 + 1000.0, "engine 2's own duct must pressurise via the left cross-bleed valve from engine 1, got {} Pa", out.engine_duct_pressure_pa[1]);
    }

    #[test]
    fn closing_all_cross_bleed_valves_stops_a_non_running_engine_from_pressurising() {
        let mut net = DuctNetwork::new();
        let mut inputs = base_inputs();
        inputs.engines[1] = not_running_engine();
        inputs.cross_bleed_valve_command = [0.0, 0.0, 0.0];
        // Pack 1 is fed from engine 1 *and* engine 2 (`pack_valve_open[0]
        // = [from engine 1, from engine 2]`), and a pack valve is an
        // ordinary two-way orifice, so with both of its feeds open the
        // pack's own supply duct is a second, parallel bridge between
        // engine 1's duct and engine 2's -- shutting the cross-bleed
        // valves alone leaves engine 1 pressurising engine 2 straight
        // through the pack, which is exactly what the model did (260 kPa,
        // i.e. engine 1's own IP port pressure) and exactly what the real
        // dual-feed pack supply would do too. Isolating engine 2's duct
        // means shutting everything that touches it, so engine 2's own
        // pack valve goes shut as well; engine 1 keeps feeding pack 1
        // through its own valve, so the pack is still live.
        inputs.pack_valve_open[0] = [1.0, 0.0];
        let mut out = NetworkOutputs::default();
        // 2000 s: long enough for the isolated duct's own gas to finish
        // equilibrating thermally with its bay (see the hand solve below,
        // tau = 275 s), so the assertion is on a settled state rather than
        // halfway through a transient.
        for _ in 0..2000 {
            out = net.step(&inputs, &DuctNetworkFaults::default());
        }
        assert!(out.pack_supply_pressure_pa[0] > 150_000.0, "pack 1 must still be fed by engine 1 -- this test isolates engine 2's duct, not the pack, got {} Pa", out.pack_supply_pressure_pa[0]);

        // Isolated means *no mass crosses the duct boundary*, which is not
        // the same as "the pressure does not move": the duct starts at
        // START_K = 288.15 K but its pylon is at `zone_air_k` = 250 K, so
        // its trapped charge cools through the lagging at constant volume
        // and constant mass, and an isochoric cool-down takes the pressure
        // down with the temperature (P = m*R*T/V, so P/P0 = T/T0).
        //   tau = m*Cv/UA, m = P0*V/(R*T0)
        //                    = 101325*2.5/(287.057*288.15) = 3.0625 kg
        //                 Cv = 1005 - 287.057 = 717.94 J/(kg*K)
        //                 UA = ENGINE_DUCT_UA_W_K = 8 W/K
        //       -> tau = 3.0625*717.94/8 = 275 s
        //   settled P = 101325 * 250/288.15 = 87 913 Pa
        // (the old "within 2000 Pa of 101 325" expectation ignored this
        // entirely and was being read at 300 s, one-third of the way down
        // the cool-down.)
        let settled_pa = 101_325.0 * 250.0 / DuctNetwork::START_K;
        assert!((out.engine_duct_pressure_pa[1] - settled_pa).abs() < 100.0, "with every valve onto it shut, a non-running engine's duct keeps its own charge and just cools to its bay: expected {} Pa, got {} Pa", settled_pa, out.engine_duct_pressure_pa[1]);
        assert!(out.engine_duct_pressure_pa[1] < 101_325.0, "nothing may add mass to an isolated duct, so it can never rise above its unpressurised start");
    }

    #[test]
    fn a_confirmed_pylon_overheat_isolates_that_engines_pr_valve_and_its_cross_bleed_connections() {
        let mut net = DuctNetwork::new();
        let mut inputs = base_inputs();
        inputs.zone_air_k[0] = OverheatDetectionLoop::THRESHOLD_PYLON_STRUT_K + 30.0; // engine 1's pylon is hot, past its own absolute threshold
        let mut out = NetworkOutputs::default();
        for _ in 0..30 {
            out = net.step(&inputs, &DuctNetworkFaults::default());
        }
        assert!(out.odls_trip[0], "engine 1's pylon ODLS must trip on a sustained overheat");
        assert!(out.pr_valve_open[0] < 0.01, "a tripped ODLS must force that engine's own PR valve shut");
        assert_eq!(out.cross_bleed_valve_open[0], 0.0, "the left cross-bleed valve touches engine 1 and must be forced shut");
        assert_eq!(out.cross_bleed_valve_open[1], 0.0, "the centre cross-bleed valve touches engine 1 and must be forced shut");
        assert!(out.cross_bleed_valve_open[2] > 0.0, "the right cross-bleed valve (engines 3/4) does not touch engine 1 and is unaffected");

        // Latching: even once the zone cools back down, isolation stays shut.
        inputs.zone_air_k[0] = 250.0;
        let out2 = net.step(&inputs, &DuctNetworkFaults::default());
        assert!(out2.pr_valve_open[0] < 0.01, "a real ODLS trip requires a reset, not self-clearing");
    }

    #[test]
    fn the_eng_bleed_pushbutton_off_shuts_that_engines_own_source_without_latching() {
        let mut net = DuctNetwork::new();
        let mut inputs = base_inputs();
        inputs.engine_bleed_pb_auto[0] = false;
        let mut out = NetworkOutputs::default();
        for _ in 0..300 {
            out = net.step(&inputs, &DuctNetworkFaults::default());
        }
        assert!(out.pr_valve_open[0] < 0.01, "the pushbutton off must shut engine 1's own PR valve, got {}", out.pr_valve_open[0]);
        assert!(!out.engine_isolated[0], "a pushbutton off must not read as an ODLS fault/trip -- it is a normal switch, not a latch");

        // Selecting it back on restores the source immediately, unlike a
        // real ODLS trip, which needs a reset.
        inputs.engine_bleed_pb_auto[0] = true;
        for _ in 0..300 {
            out = net.step(&inputs, &DuctNetworkFaults::default());
        }
        assert!(out.engine_duct_pressure_pa[0] > 101_325.0 + 1000.0, "turning the pushbutton back on must re-pressurise the duct, got {}", out.engine_duct_pressure_pa[0]);
    }

    #[test]
    fn a_ruptured_engine_duct_reports_a_real_jet_impact_flux_only_in_its_own_zone() {
        let mut net = DuctNetwork::new();
        let inputs = base_inputs();
        let mut faults = DuctNetworkFaults::default();
        faults.engine_duct[2].rupture = 1.0;
        let mut out = NetworkOutputs::default();
        for _ in 0..10 {
            out = net.step(&inputs, &faults);
        }
        assert!(out.zone_heat_w[PYLON[2]] > 1000.0, "a full duct rupture must dump substantial heat into its own pylon zone");
        assert!(out.jet_impact_flux_w_m2[PYLON[2]] > 0.0, "a full rupture must report a real structural/wiring jet flux");
        for &other in &[PYLON[0], PYLON[1], PYLON[3], TAIL_CONE, WING_LE[0], WING_LE[1]] {
            assert_eq!(out.jet_impact_flux_w_m2[other], 0.0, "no other zone has an impinging jet");
        }
    }

    #[test]
    fn wing_anti_ice_only_flows_when_selected_and_pressurises_its_own_duct() {
        let mut net = DuctNetwork::new();
        let mut inputs = base_inputs();
        inputs.wai_selected = [true, false];
        let mut out = NetworkOutputs::default();
        for _ in 0..200 {
            out = net.step(&inputs, &DuctNetworkFaults::default());
        }
        assert!(out.wai_duct_pressure_pa[0] > out.wai_duct_pressure_pa[1] + 10_000.0, "the selected side must sit well above the unselected side");
        assert_eq!(out.wai_valve_open[1], 0.0);
    }

    /// No engine bleed pressure at all, so any pressure change in the
    /// check-valve tests below can only come from the start duct's own
    /// (attempted) backflow.
    fn no_bleed_inputs() -> NetworkInputs {
        let mut inputs = base_inputs();
        for e in inputs.engines.iter_mut() {
            *e = not_running_engine();
        }
        inputs.cross_bleed_valve_command = [0.0, 0.0, 0.0];
        inputs.zone_air_k = [DuctNetwork::START_K; ZONE_COUNT];
        inputs
    }

    #[test]
    fn an_engine_start_check_valve_stops_a_lit_engines_pressure_pushing_back_into_its_own_duct() {
        let mut net = DuctNetwork::new();
        net.start[0].gas.add_mass(0.02, 900.0, 900_000.0);
        let inputs = no_bleed_inputs();
        let duct_before = net.engine_duct[0].gas.pressure_pa();
        let _ = net.step(&inputs, &DuctNetworkFaults::default());
        assert!((net.engine_duct[0].gas.pressure_pa() - duct_before).abs() < 1.0, "a healthy check valve must not let the started engine's pressure reach the duct");
    }

    #[test]
    fn a_failed_start_check_valve_lets_pressure_leak_back_into_the_engine_duct() {
        let mut net = DuctNetwork::new();
        net.start[0].gas.add_mass(0.02, 900.0, 900_000.0);
        let inputs = no_bleed_inputs();
        let mut faults = DuctNetworkFaults::default();
        faults.start_check_valve_failure[0] = 1.0;
        let duct_before = net.engine_duct[0].gas.pressure_pa();
        let _ = net.step(&inputs, &faults);
        assert!(net.engine_duct[0].gas.pressure_pa() > duct_before + 1.0, "a fully failed check valve must let pressure leak backward");
    }

    #[test]
    fn no_nan_at_rest_dt_zero() {
        let mut net = DuctNetwork::new();
        let mut inputs = base_inputs();
        inputs.dt_s = 0.0;
        let out = net.step(&inputs, &DuctNetworkFaults::default());
        assert!(out.engine_duct_pressure_pa.iter().all(|p| p.is_finite()));
        assert!(out.pack_supply_pressure_pa.iter().all(|p| p.is_finite()));
        assert!(out.zone_heat_w.iter().all(|w| w.is_finite()));
    }
}
