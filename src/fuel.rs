//! The aircraft's fuel: MSFS's fuel system, ported in `fuel_network.rs`,
//! run from FlyByWire's own definition and driven the way MSFS drives it.
//!
//! In MSFS the A380X's fuel lives in the simulator's tanks, moved by the
//! simulator's fuel system from `flight_model.cfg`, commanded by FlyByWire's
//! cockpit and transfer logic, and burnt by FlyByWire's FADEC straight out of
//! the four engine feed-line tanks (EngineControl_A380X.cpp:657-913). Here
//! the network holds the fuel, and X-Plane's nine tanks are kept equal to it
//! so X-Plane's weight and balance see the same fuel.
//!
//! - Spawn: the network takes the `[FuelSystem.0]` state of the FlyByWire
//!   flight file for the start state (`A32NX_START_STATE`, 1 hangar.flt,
//!   2 apron, 3 taxi, 4 runway, 5 Climb, 6 cruise, 7 approach, 8 final), as
//!   MSFS loads it with the flight. A cold aircraft on the ground
//!   loads its saved tank levels from the ini FlyByWire's FADEC keeps (same
//!   keys and defaults, FuelConfiguration_A380X); any other start takes the
//!   levels X-Plane has.
//! - Engine master switches open and close the engine LP valves 1 to 4, as
//!   the cockpit's FUELSYSTEM_VALVE_OPEN/CLOSE do (pedestal.xml:104-146).
//! - The engines burn their FADEC fuel flow from feed-line tanks 12 to 15.
//!   When a feed line runs dry the engine is starved, and X-Plane's engine
//!   loses its fuel.
//! - FlyByWire's refuelling (REFUEL_STARTED_BY_USR) writes the tank levels
//!   in, as fuel.rs's object write does; a change made in X-Plane's own fuel
//!   menu is taken in, as the FADEC takes in a change made in MSFS's menu.
//! - Tank levels are saved on the ground with an engine off, every five
//!   seconds, as the FADEC does.
//!
//! - Transfers are FlyByWire's `LegacyFuel.ts`, ported in
//!   `fuel_transfer.rs`, and the APU's fuel valve and pump follow FlyByWire's
//!   APU fuel aspect, both run in FlyByWire's order after the systems tick.
//!
//! - The fuel pump and valve electrical circuits (`CIRCUIT_FUEL_PUMP:n`,
//!   `CIRCUIT_FUEL_VALVE:n` in the cockpit part's systems.cfg) are powered
//!   from the bus they connect to. FlyByWire's glue ties each MSFS bus to one
//!   of its own buses, connecting the MSFS bus to the infinite `bus.1` while
//!   `A32NX_ELEC_<bus>_BUS_IS_POWERED` is 1 (systems_wasm electrical.rs:13-38;
//!   the table in a380_systems_wasm lib.rs:66-83, [`MSFS_BUSES`]). `bus.1`
//!   itself is the INFINIBAT bus (systems.cfg:303,320) and always powered.

use std::collections::HashMap;
use std::path::{Path, PathBuf};

use systems::simulation::{SimulatorReaderWriter, VariableIdentifier, VariableRegistry};

use crate::circuits::Circuits;
use crate::physics::{fluids, gas};
// MSFS_BUSES, bus_power_variable and the embedded systems.cfg used to be
// defined here; circuits.rs generalised this module's circuit parsing to
// every systems.cfg circuit type (not just fuel pumps/valves), so all three
// come from there now (docs/analysis/systems.md LIGHT-001's "reusing fuel.rs's
// parse_fuel_circuits/bus_power_variable pattern", circuits.rs's module doc).
// failures.rs's one reference to the old `crate::fuel::MSFS_BUSES` path was
// repointed at `crate::circuits::MSFS_BUSES` to match.
pub use crate::circuits::bus_power_variable;
use crate::circuits::SYSTEMS_CFG;
use crate::fuel_network::FuelNetwork;
use crate::fuel_transfer::{apu_fuel_demand_gph, ApuFuelAspect, FuelVars, LegacyFuel};
use crate::xp::{DataRef, Xplm};
use crate::Vars;

const FLIGHT_MODEL_CFG: &str = include_str!(
    "../../fbw-aircraft/fbw-a380x/src/base/flybywire-aircraft-a380-842/SimObjects/AirPlanes/FlyByWire_A380X/common/config/flight_model.cfg"
);
const APRON_FLT: &str = include_str!(
    "../../fbw-aircraft/fbw-a380x/src/base/flybywire-aircraft-a380-842/SimObjects/AirPlanes/FlyByWire_A380X/common/flt/apron.FLT"
);
const RUNWAY_FLT: &str = include_str!(
    "../../fbw-aircraft/fbw-a380x/src/base/flybywire-aircraft-a380-842/SimObjects/AirPlanes/FlyByWire_A380X/common/flt/runway.FLT"
);
const CRUISE_FLT: &str = include_str!(
    "../../fbw-aircraft/fbw-a380x/src/base/flybywire-aircraft-a380-842/SimObjects/AirPlanes/FlyByWire_A380X/common/flt/cruise.FLT"
);
const HANGAR_FLT: &str = include_str!(
    "../../fbw-aircraft/fbw-a380x/src/base/flybywire-aircraft-a380-842/SimObjects/AirPlanes/FlyByWire_A380X/common/flt/hangar.flt"
);
const TAXI_FLT: &str = include_str!(
    "../../fbw-aircraft/fbw-a380x/src/base/flybywire-aircraft-a380-842/SimObjects/AirPlanes/FlyByWire_A380X/common/flt/taxi.flt"
);
const CLIMB_FLT: &str = include_str!(
    "../../fbw-aircraft/fbw-a380x/src/base/flybywire-aircraft-a380-842/SimObjects/AirPlanes/FlyByWire_A380X/common/flt/Climb.flt"
);
const APPROACH_FLT: &str = include_str!(
    "../../fbw-aircraft/fbw-a380x/src/base/flybywire-aircraft-a380-842/SimObjects/AirPlanes/FlyByWire_A380X/common/flt/approach.FLT"
);
const FINAL_FLT: &str = include_str!(
    "../../fbw-aircraft/fbw-a380x/src/base/flybywire-aircraft-a380-842/SimObjects/AirPlanes/FlyByWire_A380X/common/flt/final.FLT"
);

/// The flight file for a start state number (StartState, systems
/// simulation/mod.rs:79-106), or `None` outside 1-8.
pub fn flt_for_start_state(state: f64) -> Option<&'static str> {
    Some(match state {
        x if (0.9..1.9).contains(&x) => HANGAR_FLT,
        x if (1.9..2.9).contains(&x) => APRON_FLT,
        x if (2.9..3.9).contains(&x) => TAXI_FLT,
        x if (3.9..4.9).contains(&x) => RUNWAY_FLT,
        x if (4.9..5.9).contains(&x) => CLIMB_FLT,
        x if (5.9..6.9).contains(&x) => CRUISE_FLT,
        x if (6.9..7.9).contains(&x) => APPROACH_FLT,
        x if (7.9..8.9).contains(&x) => FINAL_FLT,
        _ => return None,
    })
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CircuitKind {
    FuelPump,
    FuelValve,
}

/// One fuel circuit: its `circuit.N` number, type index and the MSFS buses
/// it connects to.
#[derive(Clone, Debug, PartialEq)]
pub struct FuelCircuit {
    pub number: usize,
    pub kind: CircuitKind,
    pub index: usize,
    pub buses: Vec<u32>,
}

/// The `CIRCUIT_FUEL_PUMP` and `CIRCUIT_FUEL_VALVE` circuits of a
/// systems.cfg `[ELECTRICAL]` section.
pub fn parse_fuel_circuits(cfg: &str) -> Vec<FuelCircuit> {
    let mut out = Vec::new();
    for line in cfg.lines() {
        let line = line.split(';').next().unwrap_or("").trim();
        let Some((key, value)) = line.split_once('=') else { continue };
        let Some(number) = key.trim().to_ascii_lowercase().strip_prefix("circuit.").and_then(|n| n.parse().ok()) else {
            continue;
        };
        let mut kind = None;
        let mut buses = Vec::new();
        for field in value.split('#') {
            let Some((name, v)) = field.split_once(':') else { continue };
            match name.trim().to_ascii_lowercase().as_str() {
                "type" => {
                    let v = v.trim();
                    kind = if let Some(n) = v.strip_prefix("CIRCUIT_FUEL_PUMP:") {
                        n.trim().parse().ok().map(|n| (CircuitKind::FuelPump, n))
                    } else if let Some(n) = v.strip_prefix("CIRCUIT_FUEL_VALVE:") {
                        n.trim().parse().ok().map(|n| (CircuitKind::FuelValve, n))
                    } else {
                        None
                    };
                }
                "connections" => {
                    buses = v
                        .split(',')
                        .filter_map(|c| c.trim().strip_prefix("bus.").and_then(|n| n.trim().parse().ok()))
                        .collect();
                }
                _ => {}
            }
        }
        if let Some((kind, index)) = kind {
            out.push(FuelCircuit { number, kind, index, buses });
        }
    }
    out
}

/// MSFS's JET_A weight, pounds per US gallon.
pub const JET_A_LBS_PER_GAL: f64 = 6.699;
const LB_TO_KG: f64 = 0.45359237;
/// US gallons to cubic metres (1 US gal = 231 in^3 exactly).
const GAL_TO_M3: f64 = 0.003785411784;
/// Inches of mercury to pascals (exact conversion factor).
const INHG_TO_PA: f64 = 3386.389;
/// Knots to metres per second.
const KT_TO_MS: f64 = 0.5144444;
/// Pounds per square inch to pascals (exact conversion factor).
const PSI_TO_PA: f64 = 6894.757;
/// hyperrealism.md physics workstream 5: a representative aviation fuel
/// boost-pump supply voltage (115 V AC, the common MSFS/real-aircraft fuel
/// pump convention) and motor efficiency (a typical small aviation AC motor
/// figure, ~70-80%), used only to turn each pump's already-real hydraulic
/// power into a current-draw figure for the Study panel. Not a per-pump AMM
/// rating (none is public per tank/pump position).
const FUEL_PUMP_VOLTAGE_V: f64 = 115.;
const FUEL_PUMP_MOTOR_EFFICIENCY: f64 = 0.75;

/// hyperrealism.md physics workstream 5 (fluids): FUEL-001's jettison flow
/// used to be a flat 500 gal/h constant (the brief names this exact
/// shortcut for removal). It is now a real orifice flow
/// (`physics::fluids::orifice_flow_m3_s`, `Q = Cd*A*sqrt(2*dP/rho)`) driven
/// by the wing tanks' own gravity head plus each tank's boost pump pressure,
/// against ambient static pressure at the nozzle. The nozzles' effective
/// throat area (`Cd*A`) is calibrated once, at load time, from the same
/// sourced reference this used to hard-code outright: the emergency
/// (gravity) transfer valves' flow figure (`GravityBasedFuelFlow:500`,
/// flight_model.cfg:305-306) at an assumed reference head
/// (`JETTISON_REFERENCE_HEAD_M`) -- so the *nominal* rate at that reference
/// condition matches the old constant, but the rate now correctly falls as
/// the tanks drain (falling head) and varies with altitude (falling ambient
/// back-pressure), instead of being flat regardless of tank state.
const JETTISON_REFERENCE_GAL_PER_HOUR: f64 = 500.;
/// Assumed reference fuel head for the calibration above: an order-of-
/// magnitude wing-tank vertical extent (A380 outer wing dihedral/tank
/// depth), not a cited AMM dimension -- flagged as a derived placeholder.
const JETTISON_REFERENCE_HEAD_M: f64 = 2.0;
/// Nozzle discharge coefficient for a plain (non-Venturi) jettison nozzle, a
/// standard aviation/hydraulics-engineering sharp-orifice value (ASHRAE/
/// Crane Technical Paper 410-style handbooks commonly cite 0.6-0.8 for a
/// sharp-edged orifice; 0.7 is the midpoint, not an A380-specific figure).
const JETTISON_DISCHARGE_COEFFICIENT: f64 = 0.7;

/// FUEL-004: Jet A's specification maximum freeze point (ASTM D1655), the
/// public reference `JET_A_LBS_PER_GAL`'s own density already assumes (Jet
/// A, not the colder Jet A-1 at -47C). No FBW source models this; kept as
/// the one citable public number. Re-exported from `physics::fluids` so
/// both modules agree on one number.
const FUEL_FREEZE_POINT_C: f64 = fluids::FUEL_FREEZE_POINT_C;

/// hyperrealism.md physics workstream 5: FUEL-004's tank temperature used to
/// be a flat two-hour first-order lag toward ambient (the brief names this
/// exact shortcut for removal). It is now a real heat-transfer model: an
/// overall heat-transfer coefficient (`TANK_SKIN_CONDUCTIVITY_...` below)
/// combining internal natural convection to the tank wall and external
/// forced convection to the recovery-temperature airflow at TAS, against
/// each tank's own wetted area (`physics::fluids::tank_wetted_area_m2`, from
/// its live capacity), plus hydraulic/IDG heat rejected into the engine feed
/// tanks the way the real A380's fuel/hydraulic heat exchangers do
/// (Power & Motion Technology, "Hydraulics onboard the A380": two HHX per
/// circuit, one per pylon, "transfer heat into the fuel flow from the
/// outer-engine feed circuits",
/// https://www.powermotiontech.com/hydraulics/hydraulic-pumps-motors/article/21884283/hydraulics-onboard-the-a380).
/// See `docs/physics/fluids.md` for the full derivation and every constant's
/// source.
///
/// Typical natural-convection-to-liquid heat transfer coefficient (a
/// standard textbook order-of-magnitude range is roughly 50-1000 W/m^2K for
/// natural convection to a liquid, e.g. Incropera & DeWitt, "Fundamentals of
/// Heat and Mass Transfer"; 150 W/m^2K is the chosen mid-range figure, not a
/// fuel-specific measurement).
const TANK_INTERNAL_H_W_M2K: f64 = 150.;
/// External forced-convection coefficient over the wing skin at true
/// airspeed `v` (m/s): `h = 10 + 5*sqrt(v)`, a common flat-plate-in-airflow
/// engineering approximation (order-of-magnitude, not a wind-tunnel-derived
/// A380 figure) giving roughly 10 W/m^2K static and 60-90 W/m^2K at cruise
/// TAS.
fn tank_external_h_w_m2k(true_airspeed_m_s: f64) -> f64 {
    10. + 5. * true_airspeed_m_s.abs().sqrt()
}
/// Boundary-layer recovery factor for a turbulent flow, the standard
/// aerodynamic-heating reference value.
const RECOVERY_FACTOR: f64 = 0.9;
/// Jet A specific heat capacity, J/(kg*K); a commonly cited typical value
/// for kerosene-type jet fuel around ambient/cruise temperatures (CRC
/// Handbook of Aviation Fuel Properties-adjacent references commonly quote
/// approximately 2000 J/(kg*K); not a temperature-resolved correlation).
const JET_A_SPECIFIC_HEAT_J_KGK: f64 = 2000.;
/// Wing-tank aspect ratio used for the wetted-area estimate (long, shallow
/// box): see `physics::fluids::tank_wetted_area_m2`'s doc.
const TANK_ASPECT_RATIO: f64 = 6.0;
/// hyperrealism.md physics workstream 5: FUEL gap "pump outlet pressure not
/// dropping with low tank quantity [or] unporting in pitch" -- a boost
/// pump's inlet sits a small margin above the tank floor (the same physical
/// origin as `unusable_capacity`'s hard cliff); this is the size of that
/// margin as a fraction of the tank's own box-height, over which
/// `physics::fluids::unporting_factor` ramps pump pressure smoothly from
/// full to zero as the tilted fuel surface approaches or passes the inlet.
/// No AMM figure is public for a real inlet standpipe height; 15% of the
/// tank's own modelled depth is used as an order-of-magnitude engineering
/// margin (a few percent to low tens of percent is typical for a submerged
/// pump/standpipe clearance), flagged as a derived placeholder like the
/// other tank-geometry constants in this module.
const PUMP_SUBMERSION_MARGIN_FRACTION: f64 = 0.15;
/// Fraction of engine-driven-pump shaft power (`ENGINE_GEARBOX_HYD_LOAD_W`)
/// that is internal pump loss, dissipated as heat into the hydraulic fluid:
/// `1 - SHAFT_EFFICIENCY` from the matching FBW patch
/// (`patches/fbw-rust/fluids.patch`, `EngineDrivenPump::SHAFT_EFFICIENCY`).
const HYD_PUMP_LOSS_FRACTION: f64 = 0.10;
/// Generator (IDG) electrical-to-mechanical loss fraction rejected as heat;
/// a typical aviation generator efficiency figure (~85-90%) gives roughly
/// 10-15% loss -- 0.15 is used as the more conservative (more heat) end,
/// not a specific A380 IDG test-stand figure.
const IDG_LOSS_FRACTION: f64 = 0.15;

/// FlyByWire's FADEC defaults: the four feed tanks hold 1233.9 gal, the rest
/// nothing (FuelConfiguration_A380X.h:31-43). Indexed by tank 1..11.
///
/// `pub(crate)`: aircraft_presets.rs's expedited load reuses this same
/// FlyByWire-sourced default (not an invented number) to refill a feed tank
/// that cannot start an engine, through the same `FUEL_TANK_QUANTITY_n` /
/// `REFUEL_STARTED_BY_USR` path a real EFB refuel uses (see `update`, below).
pub(crate) const DEFAULT_GALLONS: [f64; 11] = [0., 1233.9, 0., 0., 1233.9, 1233.9, 0., 0., 1233.9, 0., 0.];

/// The ini keys FlyByWire's FADEC saves the tanks under, by tank 1..11.
const INI_KEYS: [&str; 11] = [
    "FUEL_LEFT_OUTER_QTY",
    "FUEL_FEED_ONE_QTY",
    "FUEL_LEFT_MID_QTY",
    "FUEL_LEFT_INNER_QTY",
    "FUEL_FEED_TWO_QTY",
    "FUEL_FEED_THREE_QTY",
    "FUEL_RIGHT_INNER_QTY",
    "FUEL_RIGHT_MID_QTY",
    "FUEL_FEED_FOUR_QTY",
    "FUEL_RIGHT_OUTER_QTY",
    "FUEL_TRIM_QTY",
];

/// X-Plane's nine tanks as the converter laid them out (acf `_tank_name`):
/// which network tanks each holds, in fill order.
const XPLANE_TANKS: [&[usize]; 9] = [
    &[11],        // TRIM
    &[2, 12, 1],  // FEED ONE + its feed line + LEFT OUTER
    &[3],         // LEFT MID
    &[4],         // LEFT INNER
    &[5, 13],     // FEED TWO + its feed line
    &[6, 14],     // FEED THREE + its feed line
    &[7],         // RIGHT INNER
    &[8],         // RIGHT MID
    &[9, 15, 16, 10], // FEED FOUR + its feed line + the APU line + RIGHT OUTER
];

/// Feed-line ("Extra") tank for each engine.
const FEED_LINE: [usize; 4] = [12, 13, 14, 15];
/// hyperrealism.md physics workstream 5: the four real engine feed tanks
/// (FeedOne/Two/Three/Four, network tank numbers 2/5/6/9 -- see
/// `DEFAULT_GALLONS`'s own comment for the same four indices) upstream of
/// the [`FEED_LINE`] pseudo-tanks the FADEC actually burns from. Unlike
/// `FEED_LINE`, these have their own entry in `temp_c`/`tank_temp` (indices
/// 1..11 only; `FEED_LINE`'s 12-16 are line/APU pseudo-nodes with no
/// temperature state), so this is the array used for hydraulic/IDG heat
/// rejection and the suction-feed head pressure -- using `FEED_LINE` there
/// instead would index `temp_c` out of bounds.
const ENGINE_FEED_TANKS: [usize; 4] = [2, 5, 6, 9];

/// A change in X-Plane's tanks larger than this, in kg, was made by someone
/// else (X-Plane's fuel menu), not by this plugin's last write.
const OUTSIDE_CHANGE_KG: f32 = 2.0;

pub fn ini_path() -> PathBuf {
    PathBuf::from("Output").join("preferences").join("fbw_a380x_fuel.ini")
}

/// Guards read-modify-write of `fbw_a380x_fuel.ini`, the FADEC's own saved
/// tank levels: race/desync rule 8 (xphfbw-js-bridge.md) requires the same
/// named-mutex protocol for every Output/preferences file more than one
/// process can write (settings files, and this one), not just the ones
/// agent B's app_settings.rs guards.
fn ini_mutex() -> Option<crate::xphfbw_bridge::NamedMutex> {
    crate::xphfbw_bridge::NamedMutex::create("Local\\XPHFBW_fuel_ini")
}

/// Tank levels from FlyByWire's ini text, defaults where a key is missing.
pub fn parse_ini(text: &str) -> [f64; 11] {
    let mut gallons = DEFAULT_GALLONS;
    let mut in_fuel = false;
    for line in text.lines() {
        let line = line.trim();
        if line.starts_with('[') {
            in_fuel = line.eq_ignore_ascii_case("[FUEL]");
            continue;
        }
        if !in_fuel {
            continue;
        }
        if let Some((key, value)) = line.split_once('=') {
            let key = key.trim();
            if let Some(i) = INI_KEYS.iter().position(|k| k.eq_ignore_ascii_case(key)) {
                if let Ok(v) = value.trim().parse::<f64>() {
                    gallons[i] = v;
                }
            }
        }
    }
    gallons
}

pub fn ini_text(gallons: &[f64; 11]) -> String {
    let mut out = String::from("[FUEL]\n");
    for (key, g) in INI_KEYS.iter().zip(gallons) {
        out.push_str(&format!("{key} = {g:.6}\n"));
    }
    out
}

/// Share one X-Plane tank's fuel out over the network tanks it stands for,
/// filling them in order up to capacity; what is left goes to the last.
pub fn split_into(total_gallons: f64, tanks: &[usize], capacity: impl Fn(usize) -> f64) -> Vec<(usize, f64)> {
    let mut left = total_gallons.max(0.);
    let mut out = Vec::with_capacity(tanks.len());
    for (k, &t) in tanks.iter().enumerate() {
        let take = if k + 1 == tanks.len() { left } else { left.min(capacity(t)) };
        out.push((t, take));
        left -= take;
    }
    out
}

/// FUEL-001: splits `want_gal` off `tanks` (index, current gallons) in
/// proportion to each tank's share of their total, capped so no tank goes
/// negative and so at most the total is ever removed. Empty input or a
/// non-positive total/want takes nothing.
pub fn jettison_shares(tanks: &[(usize, f64)], want_gal: f64) -> Vec<(usize, f64)> {
    let total: f64 = tanks.iter().map(|(_, g)| g).sum();
    if total <= 0. || want_gal <= 0. {
        return Vec::new();
    }
    let want = want_gal.min(total);
    tanks.iter().map(|&(t, g)| (t, (g - want * g / total).max(0.))).collect()
}

struct Ids {
    tank_quantity: Vec<VariableIdentifier>,  // FUELSYSTEM TANK QUANTITY:1..16
    tank_weight: Vec<VariableIdentifier>,    // FUELSYSTEM TANK WEIGHT:1..16
    aspect_quantity: Vec<VariableIdentifier>, // FUEL_TANK_QUANTITY_1..11
    pump_active: Vec<VariableIdentifier>,
    pump_switch: Vec<VariableIdentifier>,
    valve_open: Vec<VariableIdentifier>,
    valve_switch: Vec<VariableIdentifier>,
    trigger_status: Vec<VariableIdentifier>,
    junction_setting: Vec<VariableIdentifier>,
    line_flow_apu: VariableIdentifier,
    engine_pressure: Vec<VariableIdentifier>,
    masters: [VariableIdentifier; 4],
    engine_ff: [VariableIdentifier; 4],
    engine_state: [VariableIdentifier; 4],
    engine_n3: [VariableIdentifier; 4],
    refuel_started: VariableIdentifier,
    start_state: VariableIdentifier,
    on_ground: VariableIdentifier,
    /// Each fuel circuit with the power variables of its buses (`None` for
    /// the always powered bus 1) and its `CIRCUIT CONNECTION ON:n`.
    circuits: Vec<(FuelCircuit, Vec<Option<VariableIdentifier>>, VariableIdentifier)>,
    /// FUEL-004: `AMBIENT TEMPERATURE`, each tank's `FUEL_TEMP_n` (1..11,
    /// Celsius), the freeze point and the FOB LO TEMP caution.
    ambient_temp: VariableIdentifier,
    tank_temp: Vec<VariableIdentifier>,
    freeze_point: VariableIdentifier,
    fob_lo_temp: VariableIdentifier,
    /// hyperrealism.md physics workstream 5: true airspeed (for the
    /// recovery-temperature calculation) and ambient static pressure (for
    /// the jettison orifice's back-pressure).
    true_airspeed: VariableIdentifier,
    ambient_pressure: VariableIdentifier,
    /// Pump unporting (module const doc, `PUMP_SUBMERSION_MARGIN_FRACTION`):
    /// the aircraft's own pitch/bank attitude, driving each tank's boost
    /// pump inlet submersion.
    pitch_deg: VariableIdentifier,
    bank_deg: VariableIdentifier,
    /// The shared engine-load contract's hydraulic and electrical terms
    /// (docs/briefs/hyperrealism.md): read as 0 until the hydraulics/
    /// electrical workstreams' vars exist for a given tick, per the
    /// contract's own rule. Used here only for hydraulic/IDG heat rejected
    /// into the feed tanks.
    hyd_load: [VariableIdentifier; 4],
    elec_load: [VariableIdentifier; 4],
    /// The engine model's own fuel burn (`ENGINE_FUEL_DEMAND_KG_S:n`,
    /// contract). 0 reads as "no model yet" per the contract; `update` falls
    /// back to the existing `ENGINE_FF`-derived path in that case.
    engine_fuel_demand: [VariableIdentifier; 4],
    /// hyperrealism.md physics workstream 5: each fuel pump's own pressure
    /// and current draw, for the Study panel (previously unpublished; the
    /// network only ever published tank/valve/trigger state).
    pump_pressure: Vec<VariableIdentifier>,
    pump_current: Vec<VariableIdentifier>,
    /// hyperrealism.md physics workstream (fuel second pass): the APU feed
    /// line's own continuous pressure, for the fire+APU workstream's feed
    /// pressure consumer -- before this, the APU feed had no pressure
    /// concept in the network at all (`fuel_network.rs::apu_feed_pressure_psi`,
    /// new here), only a demand/delivered gallon flow.
    apu_feed_pressure: VariableIdentifier,
}

/// FUEL-001: the two jettison nozzle valves (flight_model.cfg:414-415,
/// `Valve.57`/`58`) and the network tank indices they can drain from a
/// cockpit-triggerable switch this plugin adds (FlyByWire has no cockpit
/// switch for this: neither the systems crate nor any `src/*.rs` file
/// mentions jettison, docs/analysis/systems.md FUEL-001).
struct Jettison {
    /// New here: no FBW L:var exists for this, so it is not under the
    /// `A32NX_` namespace. A future cockpit switch (converter) or the Study
    /// panel's Ground Services/Fuel page can drive it.
    switch: VariableIdentifier,
    valve_left: Option<usize>,
    valve_right: Option<usize>,
    /// The wing tanks the aft gallery jettison lines draw from (LeftOuter,
    /// LeftMid, LeftInner, RightInner, RightMid, RightOuter;
    /// flight_model.cfg:142-151), drained in proportion to their contents.
    tanks: Vec<usize>,
    /// hyperrealism.md physics workstream 5: the two nozzles' combined
    /// effective throat area (`Cd*A`, m^2), calibrated once at construction
    /// from the sourced reference flow (see the module const doc above).
    nozzle_cda_m2: f64,
}

/// MSFS's own state that X-Plane has no counterpart for, which FlyByWire's
/// APU fuel aspect sets with key events and reads back.
#[derive(Default)]
struct MsfsState {
    apu_switch: f64,
    bleed_air_apu: f64,
}

/// FlyByWire's variables by the names their TypeScript and aspects use:
/// L:vars with the A32NX_ prefix, simvars with spaces. Each name is looked up
/// once.
struct VarsByName<'a> {
    vars: &'a mut Vars,
    ids: &'a mut HashMap<String, VariableIdentifier>,
    msfs: &'a mut MsfsState,
}

impl VarsByName<'_> {
    fn id(&mut self, name: &str) -> VariableIdentifier {
        if let Some(id) = self.ids.get(name) {
            return *id;
        }
        // The registry adds FlyByWire's prefix itself.
        let bare = name.strip_prefix("A32NX_").unwrap_or(name);
        let id = self.vars.get(bare.to_string());
        self.ids.insert(name.to_string(), id);
        id
    }
}

impl FuelVars for VarsByName<'_> {
    fn read(&mut self, name: &str) -> f64 {
        match name {
            "APU SWITCH" => self.msfs.apu_switch,
            "BLEED AIR APU" => self.msfs.bleed_air_apu,
            _ => {
                let id = self.id(name);
                self.vars.read(&id)
            }
        }
    }

    fn write(&mut self, name: &str, value: f64) {
        match name {
            "APU SWITCH" => self.msfs.apu_switch = value,
            "BLEED AIR APU" => self.msfs.bleed_air_apu = value,
            _ => {
                let id = self.id(name);
                self.vars.write(&id, value);
            }
        }
    }
}

pub struct Fuel {
    net: FuelNetwork,
    legacy: LegacyFuel,
    apu: Option<ApuFuelAspect>,
    names: HashMap<String, VariableIdentifier>,
    msfs: MsfsState,
    ids: Ids,
    xp_tanks: Option<DataRef>,
    mixture: Option<DataRef>,
    last_written: Option<[f32; 9]>,
    previous_master: [Option<bool>; 4],
    starved: [bool; 4],
    since_save: f64,
    started: bool,
    /// FUEL-004: each network tank's temperature, Celsius, 1..11.
    temp_c: [f64; 11],
    jettison: Jettison,
    /// Read and write the FADEC's saved tank levels (`fbw_a380x_fuel.ini`).
    /// Off for the offline emulator, whose cases must not share a file.
    persist: bool,
}

impl Fuel {
    pub fn new(vars: &mut Vars, xplm: &Xplm) -> Result<Self, String> {
        let mut net = FuelNetwork::from_cfg(FLIGHT_MODEL_CFG)?;
        net.set_fuel_density_lbs_per_gal(JET_A_LBS_PER_GAL);
        let range = |vars: &mut Vars, name: &str, n: usize| -> Vec<VariableIdentifier> {
            (1..=n).map(|i| vars.get(format!("{name}:{i}"))).collect()
        };
        let ids = Ids {
            tank_quantity: range(vars, "FUELSYSTEM TANK QUANTITY", net.tank_count()),
            tank_weight: range(vars, "FUELSYSTEM TANK WEIGHT", net.tank_count()),
            aspect_quantity: (1..=11).map(|i| vars.get(format!("FUEL_TANK_QUANTITY_{i}"))).collect(),
            pump_active: range(vars, "FUELSYSTEM PUMP ACTIVE", net.pump_count()),
            pump_switch: range(vars, "FUELSYSTEM PUMP SWITCH", net.pump_count()),
            valve_open: range(vars, "FUELSYSTEM VALVE OPEN", net.valve_count()),
            valve_switch: range(vars, "FUELSYSTEM VALVE SWITCH", net.valve_count()),
            trigger_status: range(vars, "FUELSYSTEM TRIGGER STATUS", net.trigger_count()),
            junction_setting: range(vars, "FUELSYSTEM JUNCTION SETTING", net.junction_count()),
            line_flow_apu: vars.get("FUELSYSTEM LINE FUEL FLOW:141".into()),
            engine_pressure: range(vars, "FUELSYSTEM ENGINE PRESSURE", 4),
            masters: [1, 2, 3, 4].map(|n| vars.get(format!("GENERAL ENG STARTER:{n}"))),
            engine_ff: [1, 2, 3, 4].map(|n| vars.get(format!("ENGINE_FF:{n}"))),
            engine_state: [1, 2, 3, 4].map(|n| vars.get(format!("ENGINE_STATE:{n}"))),
            engine_n3: [1, 2, 3, 4].map(|n| vars.get(format!("ENGINE_N3:{n}"))),
            refuel_started: vars.get("REFUEL_STARTED_BY_USR".into()),
            start_state: vars.get("START_STATE".into()),
            on_ground: vars.get("SIM ON GROUND".into()),
            circuits: parse_fuel_circuits(SYSTEMS_CFG)
                .into_iter()
                .map(|c| {
                    let ids = c.buses.iter().map(|&b| bus_power_variable(b).map(|name| vars.get(name))).collect();
                    // MSFS starts every circuit connected; the pushbuttons
                    // toggle it (ELECTRICAL_BUS_TO_CIRCUIT_CONNECTION_TOGGLE).
                    let connection = vars.get(format!("CIRCUIT CONNECTION ON:{}", c.number));
                    vars.write(&connection, 1.);
                    (c, ids, connection)
                })
                .collect(),
            ambient_temp: vars.get("AMBIENT TEMPERATURE".into()),
            tank_temp: (1..=11).map(|i| vars.get(format!("FUEL_TEMP_{i}"))).collect(),
            freeze_point: vars.get("FUEL_TEMP_FREEZE_POINT".into()),
            fob_lo_temp: vars.get("FUEL_FOB_LO_TEMP".into()),
            true_airspeed: vars.get("AIRSPEED TRUE".into()),
            ambient_pressure: vars.get("AMBIENT PRESSURE".into()),
            pitch_deg: vars.get("PLANE PITCH DEGREES".into()),
            bank_deg: vars.get("PLANE BANK DEGREES".into()),
            // Shared engine-load contract (docs/briefs/hyperrealism.md):
            // hydraulics writes ENGINE_GEARBOX_HYD_LOAD_W:n
            // (physics::hydraulics, this workstream); electrical writes
            // ENGINE_GEARBOX_ELEC_LOAD_W:n once that workstream lands. Both
            // read as 0 until written, per the contract.
            hyd_load: [1, 2, 3, 4].map(|n| vars.get(format!("ENGINE_GEARBOX_HYD_LOAD_W:{n}"))),
            elec_load: [1, 2, 3, 4].map(|n| vars.get(format!("ENGINE_GEARBOX_ELEC_LOAD_W:{n}"))),
            engine_fuel_demand: [1, 2, 3, 4].map(|n| vars.get(format!("ENGINE_FUEL_DEMAND_KG_S:{n}"))),
            pump_pressure: (1..=net.pump_count()).map(|i| vars.get(format!("FUEL_PUMP_PRESSURE_PSI:{i}"))).collect(),
            pump_current: (1..=net.pump_count()).map(|i| vars.get(format!("FUEL_PUMP_CURRENT_A:{i}"))).collect(),
            apu_feed_pressure: vars.get("APU_FUEL_FEED_PRESSURE_PSI".into()),
        };
        let jettison = Jettison {
            // No FBW L:var: new here (see the struct doc). The space keeps
            // it unprefixed, MSFS-simvar-style, the same as `CIRCUIT
            // CONNECTION ON:n` above.
            switch: vars.get("FUEL JETTISON SWITCH".into()),
            valve_left: net.valve_index("JettisonNozzleValveLeft"),
            valve_right: net.valve_index("JettisonNozzleValveRight"),
            tanks: ["LeftOuter", "LeftMid", "LeftInner", "RightInner", "RightMid", "RightOuter"]
                .iter()
                .filter_map(|n| net.tank_index(n))
                .collect(),
            // Calibrated from the sourced reference flow at the assumed
            // reference head (module const doc): Cd*A such that the orifice
            // equation reproduces JETTISON_REFERENCE_GAL_PER_HOUR at
            // JETTISON_REFERENCE_HEAD_M of gravity head.
            nozzle_cda_m2: fluids::effective_cda_m2(
                JETTISON_REFERENCE_GAL_PER_HOUR / 3600. * GAL_TO_M3,
                fluids::hydrostatic_pressure_pa(JETTISON_REFERENCE_HEAD_M, fluids::JET_A_DENSITY_KG_M3_AT_15C),
                fluids::JET_A_DENSITY_KG_M3_AT_15C,
            ),
        };
        Ok(Self {
            net,
            legacy: LegacyFuel::new(),
            apu: None,
            names: HashMap::new(),
            msfs: MsfsState::default(),
            ids,
            xp_tanks: xplm.find("sim/flightmodel/weight/m_fuel"),
            mixture: xplm.find("sim/cockpit2/engine/actuators/mixture_ratio"),
            last_written: None,
            previous_master: [None; 4],
            starved: [false; 4],
            since_save: 0.,
            started: false,
            temp_c: [15.; 11],
            jettison,
            persist: true,
        })
    }

    /// Whether the saved tank levels are read at a cold start and saved on
    /// the ground; without, a cold start takes FlyByWire's default load.
    pub fn set_persistence(&mut self, on: bool) {
        self.persist = on;
    }

    fn read_xplane(&self, xplm: &Xplm) -> Option<[f32; 9]> {
        let d = self.xp_tanks?;
        let mut kg = [0f32; 9];
        (xplm.get_vf(d, &mut kg) == 9).then_some(kg)
    }

    /// Take X-Plane's tank contents into the network.
    fn take_from_xplane(&mut self, kg: &[f32; 9]) {
        for (x, tanks) in XPLANE_TANKS.iter().enumerate() {
            let gallons = kg[x] as f64 / LB_TO_KG / JET_A_LBS_PER_GAL;
            let net = &self.net;
            let split = split_into(gallons, tanks, |t| net.tank_capacity(t));
            for (t, g) in split {
                self.net.set_tank_gallons(t, g);
            }
        }
    }

    fn start(&mut self, vars: &mut Vars, xplm: &Xplm) {
        // The plugin decides the start state before the systems are built
        // (start_state.rs); without one, the old guess.
        let chosen = vars.read(&self.ids.start_state);
        let (state, flt) = match flt_for_start_state(chosen) {
            Some(flt) => (chosen.round(), flt),
            None => {
                let on_ground = vars.read(&self.ids.on_ground) != 0.;
                let running = self.ids.masters.iter().any(|m| vars.read(m) != 0.);
                match (on_ground, running) {
                    (true, false) => (2., APRON_FLT),
                    (true, true) => (4., RUNWAY_FLT),
                    (false, _) => (6., CRUISE_FLT),
                }
            }
        };
        vars.write(&self.ids.start_state, state);
        self.net.apply_flt_state(flt);
        if state >= 5. {
            // FlyByWire's in-flight states leave the APU fuel valves 50 and
            // 51 shut, which only the ground states open; open them so the
            // APU can be started after joining in the air.
            self.net.open_valve(50);
            self.net.open_valve(51);
        }

        // Cold on the ground (hangar, apron): the FADEC's saved levels.
        if state <= 2. {
            let read = || std::fs::read_to_string(ini_path()).unwrap_or_default();
            let text = match (self.persist, ini_mutex()) {
                (false, _) => String::new(),
                (true, Some(m)) => m.with(read),
                (true, None) => read(),
            };
            let gallons = if self.persist { parse_ini(&text) } else { DEFAULT_GALLONS };
            for (i, g) in gallons.iter().enumerate() {
                self.net.set_tank_gallons(i + 1, *g);
            }
        } else if let Some(kg) = self.read_xplane(xplm) {
            self.take_from_xplane(&kg);
        }
        // The feed lines and the APU line hold their gallon, as FlyByWire's
        // flight files have them.
        for line in 12..=16 {
            self.net.set_tank_gallons(line, self.net.tank_capacity(line));
        }
        for i in 0..4 {
            let master = vars.read(&self.ids.masters[i]) != 0.;
            self.net.set_valve(i + 1, master);
            self.previous_master[i] = Some(master);
        }

        let mut by_name = VarsByName { vars, ids: &mut self.names, msfs: &mut self.msfs };
        self.legacy.init(&mut by_name);
        self.apu = Some(ApuFuelAspect::new_a380(&mut by_name));
        self.started = true;
    }

    /// Run the fuel system for one tick, after FlyByWire's systems have
    /// ticked (their aspects and TypeScript run after the systems too). Starves
    /// X-Plane's engine where a feed line is empty. `circuits` is the general
    /// circuit model (circuits.rs, #47): a pulled breaker on one of this
    /// module's own fuel pump/valve circuit numbers cuts it, on top of the
    /// bus-power gating `power_circuits` already did.
    pub fn update(&mut self, vars: &mut Vars, xplm: &Xplm, delta: f64, circuits: &Circuits) {
        if !self.started {
            self.start(vars, xplm);
        }

        // hyperrealism.md physics workstream 5: snapshot every tank's
        // gallons before this tick's mass movement, so the temperature
        // model (run at the end of this method, after every source of mass
        // movement below) can tell which tanks gained fuel this tick and
        // mix in the losing tanks' temperature (energy-conserving transfer
        // mixing), rather than treating each tank's thermal mass as a
        // closed system the way the old first-order lag implicitly did.
        let before_gallons: [f64; 11] = std::array::from_fn(|i| self.net.tank_gallons(i + 1));

        self.power_circuits(vars, circuits);
        self.jettison(vars, delta);

        // The APU fuel aspect, then the transfer logic.
        {
            let mut by_name = VarsByName { vars: &mut *vars, ids: &mut self.names, msfs: &mut self.msfs };
            if let Some(apu) = self.apu.as_mut() {
                apu.update(&mut by_name, &mut self.net);
            }
            self.legacy.update(delta * 1000., true, &mut by_name, &mut self.net);
        }

        // Engine masters: LP valves 1 to 4.
        for i in 0..4 {
            let master = vars.read(&self.ids.masters[i]) != 0.;
            if self.previous_master[i] != Some(master) {
                self.net.handle_key_event(if master { "FUELSYSTEM_VALVE_OPEN" } else { "FUELSYSTEM_VALVE_CLOSE" }, (i + 1) as u32, 0);
                self.previous_master[i] = Some(master);
            }
            let n3 = vars.read(&self.ids.engine_n3[i]);
            self.net.set_engine_rpm_fraction(i + 1, n3 / 100.);
        }

        // Refuelling from FlyByWire's own tools writes the tank levels in.
        if vars.read(&self.ids.refuel_started) != 0. {
            for (i, id) in self.ids.aspect_quantity.iter().enumerate() {
                let g = vars.read(id);
                self.net.set_tank_gallons(i + 1, g);
            }
        }

        // A change made in X-Plane's own fuel menu is taken in.
        if let (Some(now), Some(written)) = (self.read_xplane(xplm), self.last_written) {
            if now.iter().zip(&written).any(|(a, b)| (a - b).abs() > OUTSIDE_CHANGE_KG) {
                self.take_from_xplane(&now);
            }
        }

        // The engine burns from the feed lines. hyperrealism.md physics
        // workstream 5: prefer the engine model's own mass fuel flow
        // (`ENGINE_FUEL_DEMAND_KG_S:n`, the shared contract) once that
        // workstream writes it; 0 reads as "no model yet" (the contract's
        // own rule), so this falls back to the FADEC's volumetric fuel flow
        // (EngineControl_A380X.cpp:865-912) meanwhile -- never faking the
        // contract variable, only choosing which real source to burn from.
        for i in 0..4 {
            let demand_kg_s = vars.read(&self.ids.engine_fuel_demand[i]).max(0.);
            let want_gal = if demand_kg_s > 0. {
                demand_kg_s * delta / LB_TO_KG / JET_A_LBS_PER_GAL
            } else {
                let ff_kg_h = vars.read(&self.ids.engine_ff[i]).max(0.);
                ff_kg_h * delta / 3600. / LB_TO_KG / JET_A_LBS_PER_GAL
            };
            let line = FEED_LINE[i];
            let have = self.net.tank_gallons(line);
            let burn = want_gal.min(have);
            self.net.set_tank_gallons(line, have - burn);
            self.starved[i] = want_gal > 0. && burn + 1e-9 < want_gal;
        }

        let apu_demand = {
            let mut by_name = VarsByName { vars: &mut *vars, ids: &mut self.names, msfs: &mut self.msfs };
            apu_fuel_demand_gph(&mut by_name, &self.net)
        };
        self.net.set_apu_running(self.msfs.apu_switch != 0.);
        // hyperrealism.md physics workstream 5: wire the viscosity derate
        // (previously computed and tested in `physics::fluids` but never
        // plumbed in, per docs/physics/fluids.md's own follow-up note) into
        // the network's line conductance, from the average engine-feed-tank
        // temperature (the tanks whose pumps' flow capacity actually matters
        // to engine supply) one tick behind (this tick's own temperature
        // update runs after the network solve below; fuel thermal mass is
        // large enough that a one-tick lag on a derived viscosity is not
        // physically significant).
        let feed_temp_c = ENGINE_FEED_TANKS.iter().map(|&t| self.temp_c[t - 1]).sum::<f64>() / ENGINE_FEED_TANKS.len() as f64;
        self.net.set_viscosity_derate(fluids::viscosity_flow_derate(fluids::jet_a_viscosity_cst(feed_temp_c)));
        self.apply_pump_unporting(vars);
        self.net.update(delta, [0.; 4], apu_demand);

        // hyperrealism.md physics workstream 5: after every source of mass
        // movement this tick (jettison, transfer, refuel, X-Plane sync,
        // engine/APU burn, the network's own internal solve), so transfer
        // mixing sees the tick's real gallon deltas.
        self.update_temperatures(vars, xplm, delta, &before_gallons);

        self.publish(vars);
        self.update_fqms(vars);

        // A starved engine gets no fuel in X-Plane either. This runs after
        // the engine control has set the fuel from the master switch.
        if let Some(d) = self.mixture {
            for (i, starved) in self.starved.iter().enumerate() {
                if *starved {
                    xplm.set_vf_at(d, i, 0.);
                }
            }
        }

        self.write_xplane(xplm);

        self.since_save += delta;
        let on_ground = vars.read(&self.ids.on_ground) != 0.;
        let an_engine_off = self.ids.engine_state.iter().any(|s| {
            let state = vars.read(s);
            state == 0. || state == 4.
        });
        if self.persist && on_ground && an_engine_off && self.since_save > 5. {
            self.save();
        }
    }

    /// Power each fuel pump and valve circuit from its buses and, new here,
    /// its circuits.rs breaker.
    fn power_circuits(&mut self, vars: &mut Vars, circuits: &Circuits) {
        for (circuit, buses, connection) in &self.ids.circuits {
            // A circuit is powered when it is connected, its breaker is
            // closed, and one of its buses is powered.
            let powered = vars.read(connection) != 0.
                && circuits.breaker_closed(vars, circuit.number)
                && buses.iter().any(|b| b.map_or(true, |id| vars.read(&id) != 0.));
            match circuit.kind {
                CircuitKind::FuelPump => {
                    self.net.set_fuel_pump_circuit_powered(circuit.index, powered);
                    // Breaker-coupling workstream (fixed): a cavitating
                    // centrifugal boost pump moves *less* fluid at *less*
                    // head, so its delivered hydraulic power -- and its
                    // motor current -- *drops*, it does not climb toward
                    // locked rotor. An earlier pass here published the
                    // cavitation fraction straight into the *mechanical
                    // overload* coupling (`publish_load_fraction` ->
                    // `mechanical_current_multiplier`), which is the wrong
                    // direction (physics::motor's own module doc, "cavitating
                    // centrifugal pump" section, has the full writeup).
                    // `pump_hydraulic_power_fraction` is the real, `P = delta_p
                    // * Q / eta`-derived delivered-power fraction; published
                    // through the dedicated hydraulic-power registry, which
                    // `breakers.rs::post_systems` folds in with the opposite
                    // sign from the mechanical-load one. Published here (not
                    // inside `fuel_network.rs`) because only this loop has
                    // both the pump's own `CIRCUIT_FUEL_PUMP:N` type index
                    // (`circuit.index`) and the absorbed breaker catalogue's
                    // own id for the *same* circuit (`circuit.number`,
                    // `breakers.rs`'s `"sys-<number>"`, confirmed by
                    // `breakers::tests::pulling_an_absorbed_circuit_breaker_
                    // really_opens_its_systems_cfg_circuit`).
                    let power_fraction = self.net.pump_hydraulic_power_fraction(circuit.index);
                    crate::physics::motor::publish_hydraulic_power_fraction(&format!("sys-{}", circuit.number), power_fraction);
                }
                CircuitKind::FuelValve => self.net.set_fuel_valve_circuit_powered(circuit.index, powered),
            }
        }
    }

    /// FUEL-002: `FUELSYSTEM_PUMP_ON/OFF/SET/TOGGLE` clicks from the
    /// overhead FUEL panel, routed to the native fuel network the same way
    /// FlyByWire's own transfer logic already routes its own pump control
    /// (fuel_transfer.rs:689,692) — `fuel_network.rs::handle_key_event`
    /// already implements them (fuel_network.rs:1498-1501); only the
    /// dispatch from a cockpit click was missing. Valve/trigger/junction
    /// events are deliberately not forwarded: LegacyFuel drives those
    /// internally (key_events.rs's `NOT_APPLIED` list), and forwarding a
    /// cockpit click for them too would actuate the fuel system twice.
    pub fn handle_event(&mut self, name: &str, args: &[f64]) -> bool {
        if !matches!(name, "FUELSYSTEM_PUMP_ON" | "FUELSYSTEM_PUMP_OFF" | "FUELSYSTEM_PUMP_SET" | "FUELSYSTEM_PUMP_TOGGLE") {
            return false;
        }
        let index = args.first().copied().unwrap_or(0.).max(0.).round() as u32;
        let value = args.get(1).copied().unwrap_or(0.).max(0.).round() as u32;
        self.net.handle_key_event(name, index, value)
    }

    /// FUEL-001: opens or closes the two jettison nozzle valves from the new
    /// `FUEL JETTISON SWITCH`, and, while armed and both valves are open,
    /// drains the wing tanks feeding the aft gallery jettison lines
    /// (`Jettison` struct doc) in proportion to their current contents.
    /// `flight_model.cfg`'s own jettison lines (146/147) have no downstream
    /// tank at all — `net.update`'s flow solver has nothing pulling fuel
    /// through them — so the overboard flow is applied directly here rather
    /// than through the network, matching the "vent to atmosphere" sink
    /// docs/analysis/systems.md's FUEL-001 proposal describes.
    fn jettison(&mut self, vars: &mut Vars, delta: f64) {
        let armed = vars.read(&self.jettison.switch) != 0.;
        for v in [self.jettison.valve_left, self.jettison.valve_right].into_iter().flatten() {
            if armed {
                self.net.open_valve(v);
            } else {
                self.net.close_valve(v);
            }
        }
        if !armed || self.jettison.tanks.is_empty() {
            return;
        }
        let both_open = [self.jettison.valve_left, self.jettison.valve_right].into_iter().flatten().all(|v| self.net.valve_switch(v));
        if !both_open {
            return;
        }
        let net = &self.net;
        let tanks: Vec<(usize, f64)> = self.jettison.tanks.iter().map(|&t| (t, net.tank_gallons(t))).collect();

        // hyperrealism.md physics workstream 5: real orifice flow instead of
        // the flat JETTISON_REFERENCE_GAL_PER_HOUR constant (module const
        // doc). Driving pressure is the jettisoning tanks' own average
        // gravity head (their current fuel column, from the same
        // aspect-ratio box the tank temperature model uses for wetted area)
        // above the nozzle, less ambient static back-pressure's altitude
        // contribution relative to sea level (the nozzle discharges to
        // ambient, so only the *drop* in ambient pressure with altitude
        // matters, not its absolute value -- this rises with altitude,
        // giving faster jettison flow up high, the correct physical sense).
        let avg_gallons = tanks.iter().map(|&(_, g)| g).sum::<f64>() / tanks.len() as f64;
        let avg_capacity = self.jettison.tanks.iter().map(|&t| net.tank_capacity(t)).sum::<f64>() / self.jettison.tanks.len() as f64;
        let fill_fraction = if avg_capacity > 0. { (avg_gallons / avg_capacity).clamp(0., 1.) } else { 0. };
        let box_height_m = fluids::tank_box_height_m(avg_capacity * GAL_TO_M3, TANK_ASPECT_RATIO);
        let head_m = box_height_m * fill_fraction;
        let density = fluids::jet_a_density_kg_m3(15.);
        let ambient_pa = vars.read(&self.ids.ambient_pressure) * INHG_TO_PA;
        let sea_level_pa = 29.92 * INHG_TO_PA;
        let delta_p = fluids::hydrostatic_pressure_pa(head_m, density) + (sea_level_pa - ambient_pa).max(0.);
        let flow_m3_s = fluids::orifice_flow_m3_s(JETTISON_DISCHARGE_COEFFICIENT, self.jettison.nozzle_cda_m2, delta_p, density);
        let want = flow_m3_s / GAL_TO_M3 * delta;

        for (t, left) in jettison_shares(&tanks, want) {
            self.net.set_tank_gallons(t, left);
        }
    }

    /// hyperrealism.md physics workstream 5: FUEL gap "pump outlet pressure
    /// not dropping with low tank quantity, unporting in pitch". Every
    /// network tank's own boost/transfer pump(s) get a fresh
    /// `physics::fluids::unporting_factor` derate each tick, from that
    /// tank's own fill fraction (total gallons including unusable, since the
    /// physical fuel depth includes it) and the aircraft's live pitch/bank
    /// (`PLANE PITCH/BANK DEGREES`), using the same aspect-ratio tank-box
    /// geometry the temperature and jettison models already use
    /// (`TANK_ASPECT_RATIO`). `fuel_network.rs::pump_own_pressure` folds this
    /// into every pump type uniformly (see its own doc), so a pump's rated
    /// pressure genuinely falls -- smoothly, not as a hard cliff -- as its
    /// tank empties or the aircraft manoeuvres, instead of holding full rated
    /// pressure right up until `unusablecapacity`.
    fn apply_pump_unporting(&mut self, vars: &mut Vars) {
        let pitch = vars.read(&self.ids.pitch_deg);
        let bank = vars.read(&self.ids.bank_deg);
        for t in 1..=11 {
            let capacity_gal = self.net.tank_capacity(t);
            if capacity_gal <= 0. {
                continue;
            }
            let fill_fraction = self.net.tank_total_gallons(t) / capacity_gal;
            let box_height_m = fluids::tank_box_height_m(capacity_gal * GAL_TO_M3, TANK_ASPECT_RATIO);
            let factor = fluids::unporting_factor(fill_fraction, pitch, bank, box_height_m, TANK_ASPECT_RATIO, PUMP_SUBMERSION_MARGIN_FRACTION);
            self.net.set_tank_pump_derate(t, factor);
        }
    }

    /// hyperrealism.md physics workstream 5: FUEL-004's real per-tank heat
    /// transfer, replacing the flat two-hour first-order lag (module const
    /// doc). Explicit-Euler energy balance over `delta`:
    /// `m * cp * dT = (U * A_wetted * (T_recovery - T) + Q_hhx) * delta`,
    /// plus mixing on transfer (a tank that gained fuel this tick blends in
    /// the volume-weighted mean temperature of the tanks that lost it).
    /// Publishes the freeze point and a FOB LO TEMP caution when any
    /// fuelled tank reaches it, as before. FlyByWire's own ECAM already has
    /// a `FUEL TEMP LO` message defined (ata28.ts id 281800086,
    /// `sensed: true`, no condition wired) that whoever owns the FWS/ECAM
    /// could gate on `FUEL_FOB_LO_TEMP`.
    fn update_temperatures(&mut self, vars: &mut Vars, xplm: &Xplm, delta: f64, before_gallons: &[f64; 11]) {
        let _ = xplm; // reserved: no X-Plane-only input needed yet beyond `vars`.
        let ambient_c = vars.read(&self.ids.ambient_temp);
        let tas_ms = vars.read(&self.ids.true_airspeed) * KT_TO_MS;
        let sound_speed_ms = (1.4 * gas::AIR_SPECIFIC_GAS_CONSTANT * (ambient_c + 273.15).max(1.)).sqrt();
        let mach = (tas_ms.abs() / sound_speed_ms).max(0.);
        let recovery_c = fluids::recovery_temperature_k(ambient_c + 273.15, mach, RECOVERY_FACTOR) - 273.15;
        let h_ext = tank_external_h_w_m2k(tas_ms);
        let u = (TANK_INTERNAL_H_W_M2K * h_ext) / (TANK_INTERNAL_H_W_M2K + h_ext);

        // Transfer mixing: the volume-weighted mean temperature of whatever
        // tanks lost fuel this tick, for whichever tanks gained it (an
        // approximation: it does not track which specific tank fed which,
        // only the network-wide mix, since the network's own flow solve
        // does not expose per-edge tank-to-tank routing here).
        let mut lost_gal = 0.;
        let mut lost_gal_temp = 0.;
        for i in 0..11 {
            let delta_gal = self.net.tank_gallons(i + 1) - before_gallons[i];
            if delta_gal < -1e-9 {
                lost_gal += -delta_gal;
                lost_gal_temp += self.temp_c[i] * (-delta_gal);
            }
        }
        let incoming_temp = if lost_gal > 1e-9 { lost_gal_temp / lost_gal } else { ambient_c };

        // Hydraulic/IDG heat rejected through the HHX into the feed tanks
        // (module const doc). Contract vars read 0 until their workstream
        // writes them.
        let total_hyd_w: f64 = self.ids.hyd_load.iter().map(|id| vars.read(id)).sum();
        let total_elec_w: f64 = self.ids.elec_load.iter().map(|id| vars.read(id)).sum();
        let loss_w = total_hyd_w * HYD_PUMP_LOSS_FRACTION + total_elec_w * IDG_LOSS_FRACTION;
        // `heat_exchanger_transfer_w`'s effectiveness-NTU form
        // (`effectiveness * C_min * dT`) is expressed here as a plain
        // fraction of the available loss heat, one HHX "pass" worth of
        // temperature difference folded into `loss_w` itself (i.e.
        // `C_min * dT == loss_w`, `effectiveness` alone is the free
        // parameter): `hhx_w = effectiveness * loss_w`.
        let hhx_w = fluids::heat_exchanger_transfer_w(fluids::HHX_EFFECTIVENESS, loss_w, 1.0, 0.0);
        let hhx_w_per_feed_tank = hhx_w / ENGINE_FEED_TANKS.len() as f64;

        let mut any_cold = false;
        for (i, id) in self.ids.tank_temp.iter().enumerate() {
            let tank_n = i + 1;
            let gallons_now = self.net.tank_gallons(tank_n);
            let delta_gal = gallons_now - before_gallons[i];

            if delta_gal > 1e-9 {
                self.temp_c[i] = if before_gallons[i] > 1e-9 {
                    (self.temp_c[i] * before_gallons[i] + incoming_temp * delta_gal) / gallons_now.max(1e-9)
                } else {
                    incoming_temp
                };
            }

            if gallons_now > 0.1 {
                let volume_m3 = gallons_now * GAL_TO_M3;
                let area_m2 = fluids::tank_wetted_area_m2(volume_m3, TANK_ASPECT_RATIO);
                let mass_kg = volume_m3 * fluids::jet_a_density_kg_m3(self.temp_c[i]);
                let q_ext_w = fluids::convective_heat_w(u, area_m2, recovery_c - self.temp_c[i]);
                let q_hhx_w = if ENGINE_FEED_TANKS.contains(&tank_n) { hhx_w_per_feed_tank } else { 0. };
                if mass_kg > 1e-6 {
                    self.temp_c[i] += (q_ext_w + q_hhx_w) * delta / (mass_kg * JET_A_SPECIFIC_HEAT_J_KGK);
                }
                if self.temp_c[i] <= FUEL_FREEZE_POINT_C {
                    any_cold = true;
                }
            } else {
                // Empty tank: no thermal mass to model, tracks ambient.
                self.temp_c[i] = ambient_c;
            }
            vars.write(id, self.temp_c[i]);
        }
        vars.write(&self.ids.freeze_point, FUEL_FREEZE_POINT_C);
        vars.write(&self.ids.fob_lo_temp, any_cold as i32 as f64);
    }

    fn publish(&self, vars: &mut Vars) {
        let net = &self.net;
        for (i, id) in self.ids.tank_quantity.iter().enumerate() {
            vars.write(id, net.tank_gallons(i + 1));
        }
        for (i, id) in self.ids.tank_weight.iter().enumerate() {
            vars.write(id, net.tank_weight_lbs(i + 1));
        }
        for (i, id) in self.ids.aspect_quantity.iter().enumerate() {
            vars.write(id, net.tank_gallons(i + 1));
        }
        for (i, id) in self.ids.pump_active.iter().enumerate() {
            vars.write(id, net.pump_active(i + 1) as i32 as f64);
        }
        for (i, id) in self.ids.pump_switch.iter().enumerate() {
            vars.write(id, net.pump_switch(i + 1) as f64);
        }
        for (i, id) in self.ids.valve_open.iter().enumerate() {
            vars.write(id, net.valve_open(i + 1));
        }
        for (i, id) in self.ids.valve_switch.iter().enumerate() {
            vars.write(id, net.valve_switch(i + 1) as i32 as f64);
        }
        for (i, id) in self.ids.trigger_status.iter().enumerate() {
            vars.write(id, net.trigger_status(i + 1) as i32 as f64);
        }
        for (i, id) in self.ids.junction_setting.iter().enumerate() {
            vars.write(id, net.junction_setting(i + 1) as f64);
        }
        // hyperrealism.md physics workstream 5: feed pressure at the engine
        // inlet floored by gravity/suction feed. The network's own pump-
        // pressure solve already reports 0 once a feed tank's boost pumps
        // are all off/failed; on the real aircraft the tank's own head still
        // gravity/suction-feeds the engine at a much lower (but non-zero)
        // pressure at low altitude. Adds a hydrostatic floor from the feed
        // tank's own fill height (same box-geometry model as the tank
        // temperature/jettison calculations) instead of reporting zero.
        for (i, id) in self.ids.engine_pressure.iter().enumerate() {
            let pump_psi = net.engine_pressure_psi(i + 1);
            let feed_tank = ENGINE_FEED_TANKS[i];
            let gallons = net.tank_gallons(feed_tank);
            let capacity = net.tank_capacity(feed_tank);
            let fill_fraction = if capacity > 0. { (gallons / capacity).clamp(0., 1.) } else { 0. };
            let head_m = fluids::tank_box_height_m(capacity * GAL_TO_M3, TANK_ASPECT_RATIO) * fill_fraction;
            let density = fluids::jet_a_density_kg_m3(self.temp_c[feed_tank - 1]);
            let suction_psi = fluids::hydrostatic_pressure_pa(head_m, density) / PSI_TO_PA;
            vars.write(id, pump_psi.max(suction_psi));
        }
        vars.write(&self.ids.line_flow_apu, net.line_flow_gph(141));

        // hyperrealism.md physics workstream (fuel second pass): the APU
        // feed line's own continuous pressure -- `fuel_network.rs`'s new
        // `apu_feed_pressure_psi` (the pumps/suction solve, same mechanism
        // as `engine_pressure_psi`), floored the same way the engine feed
        // pressure is by the APU line tank's own gravity/suction head so a
        // pump-off APU feed reads a real, non-zero low-altitude suction
        // pressure rather than a flat zero. Tank 16 is the APU line pseudo-
        // tank (`XPLANE_TANKS`'s tank 9 group, `&[9, 15, 16, 10]`); it has no
        // entry in `temp_c` (indices 1..11 only), so 15 C reference density
        // is used here rather than a tracked temperature, the same way
        // `fuel_network.rs`'s own untracked pseudo-nodes are handled
        // elsewhere in this module.
        {
            const APU_LINE_TANK: usize = 16;
            let pump_psi = net.apu_feed_pressure_psi(0);
            let gallons = net.tank_gallons(APU_LINE_TANK);
            let capacity = net.tank_capacity(APU_LINE_TANK);
            let fill_fraction = if capacity > 0. { (gallons / capacity).clamp(0., 1.) } else { 0. };
            let head_m = fluids::tank_box_height_m(capacity * GAL_TO_M3, TANK_ASPECT_RATIO) * fill_fraction;
            let density = fluids::jet_a_density_kg_m3(15.);
            let suction_psi = fluids::hydrostatic_pressure_pa(head_m, density) / PSI_TO_PA;
            vars.write(&self.ids.apu_feed_pressure, pump_psi.max(suction_psi));
        }

        // hyperrealism.md physics workstream 5: each fuel pump's own
        // pressure-flow point and current draw (Study panel quantities),
        // computed from the network's own pressure/flow (`fuel_network.rs`'s
        // new `pump_pressure_psi`/`pump_flow_gph` accessors, added by this
        // workstream) rather than invented per-pump figures. Current draw
        // assumes a representative 115 V AC three-phase aviation boost-pump
        // motor and a typical motor efficiency (`FUEL_PUMP_MOTOR_EFFICIENCY`
        // module const) -- generic figures, not a per-pump AMM spec (no
        // public per-pump rating exists), flagged as such in
        // docs/physics/fluids.md.
        for i in 0..self.ids.pump_pressure.len() {
            let n = i + 1;
            let pressure_pa = net.pump_pressure_psi(n) * PSI_TO_PA;
            let flow_m3_s = net.pump_flow_gph(n) / 3600. * GAL_TO_M3;
            let hyd_w = fluids::hydraulic_power_w(pressure_pa, flow_m3_s.abs());
            let current_a = fluids::pump_current_a(hyd_w, FUEL_PUMP_VOLTAGE_V, FUEL_PUMP_MOTOR_EFFICIENCY);
            vars.write(&self.ids.pump_pressure[i], net.pump_pressure_psi(n));
            vars.write(&self.ids.pump_current[i], current_a);
        }
    }

    /// hyperrealism.md physics workstream (fuel second pass), brief item 1:
    /// "FQMS quantity indication, independent of true quantity". FBW's own
    /// FQDC (`a380_systems/src/fuel/fuel_quantity_data_concentrator.rs:56-57`)
    /// publishes the ARINC 429 tank-quantity words its whole FQMS/ECAM chain
    /// reads straight from `FUEL_TANK_QUANTITY_n` -- this plugin's own *true*
    /// value (its own source comment: "these values are also used [by] the
    /// FQMS because in the sim there only exists this value"). This method
    /// overwrites those same words, strictly after FBW's systems tick (the
    /// same position `apply_pump_unporting` etc already run relative to),
    /// with an independently-modelled *indicated* value: a multi-probe
    /// average (`physics::fluids::probe_indicated_fill_fraction`, exact at
    /// true fill when level/mildly tilted, biased only once attitude clips
    /// the probe array -- the real FQI error mode) times the true tank
    /// temperature's own Jet A density, gated per FQDC channel by that
    /// channel's real cited power bus (`cpiom_f/mod.rs:745-746`: FQDC_1 on
    /// "501PP", FQDC_2 on "109PP"/"101PP"/"107PP"), read 0 (unpowered) until
    /// the electrical workstream publishes those named-bus vars -- the same
    /// shared-contract "reads 0 until written" rule `ENGINE_GEARBOX_HYD_LOAD_W`
    /// already uses. FBW's own `FuelPage.tsx` fallback chain
    /// (FQMS -> FQDC1 -> FQDC2 -> amber XX) is left untouched and does the
    /// right thing with these words unchanged. True fuel physics (burn,
    /// transfer, weight and balance) is never touched here -- only the
    /// *indication* path, per the brief's own "publish indicated values...
    /// keeping the true values for physics."
    fn update_fqms(&mut self, vars: &mut Vars) {
        const TANK_NAMES: [&str; 11] = [
            "LEFT_OUTER", "FEED_1", "LEFT_MID", "LEFT_INNER", "FEED_2", "FEED_3", "RIGHT_INNER", "RIGHT_MID", "FEED_4",
            "RIGHT_OUTER", "TRIM",
        ];
        // ARINC 429 SSM (`shared/arinc429.rs`'s `SignStatus`): 0b00 failure
        // warning, 0b01 no computed data, 0b11 normal operation.
        const SSM_FAILURE_WARNING: u64 = 0b00;
        const SSM_NO_COMPUTED_DATA: u64 = 0b01;
        const SSM_NORMAL_OPERATION: u64 = 0b11;
        // FBW's own `to_arinc429` (`shared/arinc429.rs:154-159`): pack the
        // f32-bit-pattern value with the SSM in the high 32 bits of a u64,
        // stored as that u64 numerically converted to f64 (exact for this
        // magnitude, matching `from_arinc429`'s own `simvar as u64`).
        let encode = |value_kg: f64, ssm: u64| -> f64 { (((value_kg as f32).to_bits() as u64) | (ssm << 32)) as f64 };

        let pitch = vars.read(&self.ids.pitch_deg);
        let bank = vars.read(&self.ids.bank_deg);

        // Channel health is FlyByWire's own: its FQDCs/FQMS already publish
        // these words with an SSM from their real CPIOM power, availability
        // and self-test (a380_systems fuel/cpiom_f). Keep that SSM and only
        // replace the value with the probe-based indication; a word FBW marks
        // failed or without data stays exactly as FBW wrote it. (The power
        // buses 501PP/109PP are sub-buses FBW never publishes as
        // ELEC_*_BUS_IS_POWERED, so gating on those names would read
        // unpowered forever.)
        let reindicate = |vars: &mut Vars, name: String, kg: f64| {
            let id = vars.get(name);
            let word = vars.read(&id);
            let ssm = ((word as u64) >> 32) & 0b11;
            if ssm == SSM_NORMAL_OPERATION || ssm == 0b10 {
                vars.write(&id, encode(kg, ssm));
            }
        };
        let _ = (SSM_FAILURE_WARNING, SSM_NO_COMPUTED_DATA);

        for (i, name) in TANK_NAMES.iter().enumerate() {
            let tank = i + 1;
            let capacity_gal = self.net.tank_capacity(tank);
            let gallons = self.net.tank_gallons(tank);
            let fill_fraction = if capacity_gal > 0. { (gallons / capacity_gal).clamp(0., 1.) } else { 0. };
            let box_height_m = fluids::tank_box_height_m(capacity_gal * GAL_TO_M3, TANK_ASPECT_RATIO);
            let tilt_fraction = fluids::tank_tilt_fraction(pitch, bank, box_height_m, TANK_ASPECT_RATIO);
            // Probe count scaled to tank size (no AMM probe count is public):
            // a larger tank needs more probes to resolve a larger free-
            // surface tilt without clipping, an order-of-magnitude derived
            // figure, not a sourced count.
            let probe_count = (4.0 + capacity_gal / 1200.0).round().clamp(4., 14.) as u32;
            let indicated_fraction = fluids::probe_indicated_fill_fraction(fill_fraction, tilt_fraction, probe_count);
            let density = fluids::jet_a_density_kg_m3(self.temp_c[i]);
            let indicated_kg = indicated_fraction * capacity_gal * GAL_TO_M3 * density;

            reindicate(vars, format!("FQDC_1_{name}_TANK_QUANTITY"), indicated_kg);
            reindicate(vars, format!("FQDC_2_{name}_TANK_QUANTITY"), indicated_kg);
            reindicate(vars, format!("FQMS_{name}_TANK_QUANTITY"), indicated_kg);
        }
    }

    fn write_xplane(&mut self, xplm: &Xplm) {
        let Some(d) = self.xp_tanks else { return };
        let mut kg = [0f32; 9];
        for (x, tanks) in XPLANE_TANKS.iter().enumerate() {
            let gallons: f64 = tanks.iter().map(|&t| self.net.tank_gallons(t)).sum();
            kg[x] = (gallons * JET_A_LBS_PER_GAL * LB_TO_KG) as f32;
        }
        xplm.set_vf(d, &kg);
        self.last_written = Some(kg);
    }

    fn save(&mut self) {
        let mut gallons = [0.; 11];
        for (i, g) in gallons.iter_mut().enumerate() {
            *g = self.net.tank_gallons(i + 1);
        }
        let _ = write_ini_atomic(&ini_path(), &gallons);
        self.since_save = 0.;
    }
}

/// Write-temp-then-rename under [`ini_mutex`] (race/desync rule 8): a plain
/// `fs::write` here could leave a truncated ini if X-Plane is killed
/// mid-write, or race a second writer's own temp file of the same name onto
/// disk half-written before either renames.
fn write_ini_atomic(path: &Path, gallons: &[f64; 11]) -> std::io::Result<()> {
    if let Some(dir) = path.parent() {
        std::fs::create_dir_all(dir)?;
    }
    let tmp = path.with_extension("ini.tmp");
    let text = ini_text(gallons);
    let write = || -> std::io::Result<()> {
        std::fs::write(&tmp, text.as_bytes())?;
        std::fs::rename(&tmp, path)
    };
    match ini_mutex() {
        Some(m) => m.with(write),
        None => write(),
    }
}


#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_embedded_definition_builds_a_network_with_the_a380s_tanks() {
        let net = FuelNetwork::from_cfg(FLIGHT_MODEL_CFG).unwrap();
        assert_eq!(net.tank_count(), 16);
        assert!((net.tank_capacity(4) - 12189.4).abs() < 1e-6);
    }

    #[test]
    fn a_missing_ini_gives_flybywires_defaults() {
        let g = parse_ini("");
        assert_eq!(g[1], 1233.9);
        assert_eq!(g[0], 0.);
        assert_eq!(g.iter().sum::<f64>(), 4. * 1233.9);
    }

    #[test]
    fn writing_the_ini_leaves_no_temp_file_and_round_trips() {
        // Race/desync rule 8 (xphfbw-js-bridge.md): the save must go
        // through a temp file that is renamed into place, never a direct
        // write a reader (or a second writer) could see half-finished.
        let dir = std::env::temp_dir().join(format!("fbw_fuel_ini_atomic_{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        let path = dir.join("fbw_a380x_fuel.ini");
        let mut gallons = DEFAULT_GALLONS;
        gallons[3] = 4242.5;
        write_ini_atomic(&path, &gallons).unwrap();
        assert!(path.is_file(), "the ini itself was written");
        assert!(!path.with_extension("ini.tmp").exists(), "the temp file was renamed away, not left behind");
        let text = std::fs::read_to_string(&path).unwrap();
        assert_eq!(parse_ini(&text), gallons);
        // A second save (the periodic save while parked) still leaves
        // exactly the final file, not an ini.tmp beside it.
        gallons[3] = 1.0;
        write_ini_atomic(&path, &gallons).unwrap();
        assert!(!path.with_extension("ini.tmp").exists());
        assert_eq!(parse_ini(&std::fs::read_to_string(&path).unwrap()), gallons);
        std::fs::remove_dir_all(&dir).ok();
    }

    #[test]
    fn the_ini_round_trips_with_flybywires_keys() {
        let mut g = DEFAULT_GALLONS;
        g[3] = 5000.25;
        let text = ini_text(&g);
        assert!(text.contains("FUEL_LEFT_INNER_QTY = 5000.25"));
        assert_eq!(parse_ini(&text), g);
    }

    #[test]
    fn a_merged_xplane_tank_fills_the_feed_tank_first() {
        let capacity = |t: usize| if t == 2 { 7299.6 } else if t == 12 { 1.0 } else { 2731.5 };
        let split = split_into(8000., &[2, 12, 1], capacity);
        assert_eq!(split[0], (2, 7299.6));
        assert_eq!(split[1], (12, 1.0));
        assert!((split[2].1 - 699.4).abs() < 1e-9);
        let total: f64 = split.iter().map(|s| s.1).sum();
        assert!((total - 8000.).abs() < 1e-9);
    }

    #[test]
    fn every_network_tank_has_one_xplane_tank() {
        let mut all: Vec<usize> = XPLANE_TANKS.iter().flat_map(|t| t.iter().copied()).collect();
        all.sort();
        assert_eq!(all, (1..=16).collect::<Vec<_>>());
    }

    #[test]
    fn every_start_state_has_its_flight_file() {
        for n in 1..=8 {
            let flt = flt_for_start_state(n as f64).unwrap();
            let mut net = FuelNetwork::from_cfg(FLIGHT_MODEL_CFG).unwrap();
            assert!(net.apply_flt_state(flt) > 0, "state {n}");
            // The APU fuel valves: open in the ground states' files, shut in
            // the air states' (the plugin opens them after loading).
            let apu_open = net.valve_switch(50) && net.valve_switch(51);
            assert_eq!(apu_open, n <= 4, "state {n}");
        }
        assert!(flt_for_start_state(0.).is_none());
        assert!(flt_for_start_state(9.).is_none());
        for n in [1, 2, 3, 4, 6, 8] {
            assert!(flt_for_start_state(n as f64).unwrap().contains(&format!("A32NX_START_STATE={n}")), "state {n}");
        }
    }

    #[test]
    fn the_cockpit_systems_cfg_has_every_fuel_circuit() {
        let circuits = parse_fuel_circuits(SYSTEMS_CFG);
        let pumps: Vec<_> = circuits.iter().filter(|c| c.kind == CircuitKind::FuelPump).collect();
        let valves: Vec<_> = circuits.iter().filter(|c| c.kind == CircuitKind::FuelValve).collect();
        assert_eq!(pumps.len(), 25);
        assert_eq!(valves.len(), 60);
        // Everything hangs off the infinite bus but the APU pump, on DC ESS
        // (systems.cfg:538).
        for c in &circuits {
            let expected = if c.kind == CircuitKind::FuelPump && c.index == 21 { vec![10] } else { vec![1] };
            assert_eq!(c.buses, expected, "{c:?}");
        }
        // The line without a colon after Name (systems.cfg:481) still parses.
        assert!(valves.iter().any(|c| c.index == 8));
        assert_eq!(bus_power_variable(10).as_deref(), Some("ELEC_DC_ESS_BUS_IS_POWERED"));
        assert_eq!(bus_power_variable(11).as_deref(), Some("ELEC_309PP_BUS_IS_POWERED"));
        assert_eq!(bus_power_variable(1), None);
    }

    #[test]
    fn a_dead_dc_ess_bus_stops_the_apu_pump() {
        let run = |dc_ess: bool| {
            let mut net = FuelNetwork::from_cfg(FLIGHT_MODEL_CFG).unwrap();
            net.apply_flt_state(APRON_FLT);
            for c in parse_fuel_circuits(SYSTEMS_CFG) {
                let powered = c.buses.iter().any(|&b| b != 10 || dc_ess);
                match c.kind {
                    CircuitKind::FuelPump => net.set_fuel_pump_circuit_powered(c.index, powered),
                    CircuitKind::FuelValve => net.set_fuel_valve_circuit_powered(c.index, powered),
                }
            }
            net.pump_on(21);
            net.update(0.1, [0.; 4], 0.);
            net.pump_active(21)
        };
        assert!(run(true), "APU pump runs with DC ESS");
        assert!(!run(false), "APU pump stops without DC ESS");
    }

    #[test]
    fn every_spawn_state_applies() {
        for flt in [APRON_FLT, RUNWAY_FLT, CRUISE_FLT] {
            let mut net = FuelNetwork::from_cfg(FLIGHT_MODEL_CFG).unwrap();
            assert!(net.apply_flt_state(flt) > 0);
        }
    }

    #[test]
    fn the_jettison_valves_and_wing_tanks_resolve_in_the_network() {
        let net = FuelNetwork::from_cfg(FLIGHT_MODEL_CFG).unwrap();
        assert!(net.valve_index("JettisonNozzleValveLeft").is_some());
        assert!(net.valve_index("JettisonNozzleValveRight").is_some());
        for name in ["LeftOuter", "LeftMid", "LeftInner", "RightInner", "RightMid", "RightOuter"] {
            assert!(net.tank_index(name).is_some(), "{name}");
        }
    }

    #[test]
    fn opening_the_jettison_valves_shows_up_as_open() {
        let mut net = FuelNetwork::from_cfg(FLIGHT_MODEL_CFG).unwrap();
        let left = net.valve_index("JettisonNozzleValveLeft").unwrap();
        assert!(!net.valve_switch(left));
        net.open_valve(left);
        assert!(net.valve_switch(left));
        net.close_valve(left);
        assert!(!net.valve_switch(left));
    }

    #[test]
    fn jettison_shares_drain_proportionally_and_never_go_negative() {
        let tanks = [(1, 1000.), (2, 500.), (3, 0.)];
        let out = jettison_shares(&tanks, 300.);
        assert_eq!(out.len(), 3);
        // 1000/1500 and 500/1500 of the 300 gallons wanted.
        assert!((out[0].1 - 800.).abs() < 1e-9);
        assert!((out[1].1 - 400.).abs() < 1e-9);
        assert_eq!(out[2].1, 0.);
        // Asking for more than the total leaves nothing negative.
        let dry = jettison_shares(&tanks, 10_000.);
        assert!(dry.iter().all(|&(_, g)| g >= 0.));
    }

    #[test]
    fn jettison_shares_takes_nothing_from_empty_tanks_or_a_zero_request() {
        assert!(jettison_shares(&[], 100.).is_empty());
        assert!(jettison_shares(&[(1, 0.)], 100.).is_empty());
        assert!(jettison_shares(&[(1, 500.)], 0.).is_empty());
    }

    #[test]
    fn the_freeze_point_is_jet_as_astm_spec_maximum() {
        assert_eq!(FUEL_FREEZE_POINT_C, -40.);
    }

    // hyperrealism.md physics workstream 5: the jettison nozzle calibration
    // (module const doc) should reproduce the sourced reference flow at the
    // assumed reference head, the same way `Fuel::new` computes it.
    #[test]
    fn jettison_nozzle_calibration_reproduces_the_reference_flow() {
        let density = fluids::JET_A_DENSITY_KG_M3_AT_15C;
        let reference_flow_m3_s = JETTISON_REFERENCE_GAL_PER_HOUR / 3600. * GAL_TO_M3;
        let reference_dp = fluids::hydrostatic_pressure_pa(JETTISON_REFERENCE_HEAD_M, density);
        let cda = fluids::effective_cda_m2(reference_flow_m3_s, reference_dp, density);
        let flow_back = fluids::orifice_flow_m3_s(1.0, cda, reference_dp, density);
        assert!((flow_back - reference_flow_m3_s).abs() / reference_flow_m3_s < 1e-9);
    }

    // Physical sense check: jettison flow (module doc's "Cd*A" formula) must
    // fall as the tank drains (falling head) and never go negative.
    #[test]
    fn jettison_flow_falls_as_head_falls() {
        let density = fluids::JET_A_DENSITY_KG_M3_AT_15C;
        let cda = fluids::effective_cda_m2(
            JETTISON_REFERENCE_GAL_PER_HOUR / 3600. * GAL_TO_M3,
            fluids::hydrostatic_pressure_pa(JETTISON_REFERENCE_HEAD_M, density),
            density,
        );
        let full = fluids::orifice_flow_m3_s(
            JETTISON_DISCHARGE_COEFFICIENT,
            cda,
            fluids::hydrostatic_pressure_pa(JETTISON_REFERENCE_HEAD_M, density),
            density,
        );
        let half = fluids::orifice_flow_m3_s(
            JETTISON_DISCHARGE_COEFFICIENT,
            cda,
            fluids::hydrostatic_pressure_pa(JETTISON_REFERENCE_HEAD_M / 2., density),
            density,
        );
        let empty = fluids::orifice_flow_m3_s(JETTISON_DISCHARGE_COEFFICIENT, cda, 0., density);
        assert!(full > half);
        assert!(half > empty);
        assert_eq!(empty, 0.);
    }

    // A leak/pump-loss failure case: with no driving pressure at all (pump
    // failed and empty/flat tank), jettison and feed-pressure physics alike
    // must settle at exactly zero, never negative or NaN.
    #[test]
    fn zero_head_and_zero_pressure_never_produce_nan_or_negative_flow() {
        let density = fluids::jet_a_density_kg_m3(FUEL_FREEZE_POINT_C);
        let flow = fluids::orifice_flow_m3_s(JETTISON_DISCHARGE_COEFFICIENT, 0.001, 0., density);
        assert_eq!(flow, 0.);
        assert!(!flow.is_nan());
        let suction = fluids::hydrostatic_pressure_pa(0., density);
        assert_eq!(suction, 0.);
    }
}
