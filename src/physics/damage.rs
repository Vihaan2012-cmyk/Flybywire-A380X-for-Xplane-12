//! Exceedance and damage tracking (hyperrealism physics workstream 6:
//! failures, damage, MEL and persistence; `docs/physics/failures.md` has
//! every threshold's source table).
//!
//! This module reads only variables other workstreams already publish
//! (`ENGINE_EGT_UNTRIMMED:n`/`ENGINE_EGT:n`, the thrust-lever angle, the APU's own EGT/warning
//! variables) and raw X-Plane datarefs (airspeed, gear/flap position,
//! weight, ground contact) it owns outright, so it never needs another
//! workstream's file. Two kinds of output:
//! - failures from `failures::extra_failures()` it can arm directly
//!   (`failures::set_active`), for effects wired now (an X-Plane native
//!   failure dataref, or nothing further to wire); and
//! - hook variables (`ENGINE_CREEP_LIFE_FRACTION:n`,
//!   `ENGINE_COMPRESSOR_EFFICIENCY_LOSS:n`) the engine workstream should
//!   read to turn accumulated creep into higher EGT/lower efficiency, per
//!   the brief. Until it does, these are published but unconsumed — this
//!   module does not claim the thrust/EGT effect itself.

#![allow(dead_code)] // Study-panel API (snapshot/repair), wired up by the lead; TGT_TAKEOFF_C documents the redline for reference.

use crate::failures;
use crate::xp::{DataRef, Xplm};
use systems::simulation::VariableIdentifier;

// ---------------------------------------------------------------------------
// Cited limits (docs/physics/failures.md has the full table).
// ---------------------------------------------------------------------------

/// EASA.E.012 Issue 12 (16 March 2026), Rolls-Royce Trent 900 (Trent 970-84/
/// 972-84/972E-84) type-certificate data sheet, §IV.1.2 "Turbine Gas
/// Temperature (TGT) - Trimmed" and its Notes 5/14. FlyByWire's A380
/// package models a Trent 972B-84 (`physics/engine/params.rs`), so this is
/// the primary reference; the GP7200's EASA.IM.E.026 Issue 03 (4 Jan 2013)
/// figures are noted alongside in the docs table for when an engine choice
/// is added.
mod trent900 {
    /// Maximum continuous TGT, unrestricted duration (°C).
    pub const TGT_MAX_CONTINUOUS_C: f64 = 850.0;
    /// Maximum take-off TGT, 5-minute limit (Note 5) (°C).
    pub const TGT_TAKEOFF_C: f64 = 900.0;
    /// Maximum over-temperature, 20-second limit (Note 14) (°C).
    pub const TGT_OVERTEMP_C: f64 = 920.0;
    pub const TAKEOFF_LIMIT_S: f64 = 5.0 * 60.0;
    /// The same three limits as measured on the engine (Note 16, "TGT
    /// trimming": the EEC shows a trimmed value in the cockpit; Profile 5,
    /// the minimum service standard). The physical engine's TGT
    /// (`physics/engine`, the LP turbine inlet plane of Note 6) is the
    /// measured, untrimmed temperature, so it is compared against these.
    pub const TGT_TAKEOFF_UNTRIMMED_C: f64 = 956.0;
    pub const TGT_MAX_CONTINUOUS_UNTRIMMED_C: f64 = 939.0;
    pub const TGT_OVERTEMP_UNTRIMMED_C: f64 = 957.0;
    /// §IV.1.2: below 50% HP speed, maximum during ground starts; maximum
    /// during in-flight relights (trimmed; no untrimmed figure is given, so
    /// the measured TGT is held to these, the conservative reading).
    pub const TGT_GROUND_START_C: f64 = 700.0;
    pub const GROUND_START_HP_PCT: f64 = 50.0;
    pub const TGT_RELIGHT_C: f64 = 850.0;
    /// §IV.2.2 minimum oil pressure: ground idle to 70% HP, and above 95%
    /// HP (psi). Between 70% and 95% the data sheet states no figure.
    pub const OIL_PRESS_MIN_LOW_PSI: f64 = 25.0;
    pub const OIL_PRESS_LOW_BAND_HP_PCT: f64 = 70.0;
    pub const OIL_PRESS_MIN_HIGH_PSI: f64 = 50.0;
    pub const OIL_PRESS_HIGH_BAND_HP_PCT: f64 = 95.0;
    /// §IV.1.4 combined oil scavenge temperature (°C): minimum for starting
    /// (without / with the Special Starting procedure), minimum for
    /// acceleration to take-off power, maximum for unrestricted use.
    pub const OIL_TEMP_MIN_START_C: f64 = -30.0;
    pub const OIL_TEMP_MIN_START_SPECIAL_C: f64 = -40.0;
    pub const OIL_TEMP_MIN_TAKEOFF_C: f64 = 40.0;
    pub const OIL_TEMP_MAX_C: f64 = 196.0;
    /// §IV.3 maximum rotor speeds for take-off (% of 12,200 / 8,300 / 2,900
    /// rpm) and the IP over-speed allowed for 20 s (Note 14).
    pub const HP_TAKEOFF_MAX_PCT: f64 = 97.8;
    pub const IP_TAKEOFF_MAX_PCT: f64 = 98.7;
    pub const LP_TAKEOFF_MAX_PCT: f64 = 97.2;
    pub const IP_OVERSPEED_PCT: f64 = 99.5;
    pub const IP_OVERSPEED_LIMIT_S: f64 = 20.0;
    /// Note 5: "the take-off rating ... may be used for up to 10 minutes in
    /// the event of an engine failure".
    pub const TAKEOFF_OEI_LIMIT_S: f64 = 10.0 * 60.0;
    pub const OVERTEMP_LIMIT_S: f64 = 20.0;
}

/// FlyByWire's own `A32NX_AUTOTHRUST_TLA:n` scale (`throttle.rs`): the TOGA
/// detent starts at `TLA_TOGA` degrees.
const TLA_TOGA_DEG: f64 = crate::throttle::TLA_TOGA;

/// FlyByWire flight_model.cfg's own reference speeds and per-CONF flap
/// limits (`D:\Microsoft Flight Simulator 2020\...\FlyByWire_A380_842\
/// flight_model.cfg`), read directly rather than re-derived. VFE by
/// `FLAPS_HANDLE_INDEX` (`handling.rs`): 0 = up (no limit), matching
/// `[FLAPS.1]`'s `flaps-position.N` airspeed-limit column (line 887-892).
const VFE_KT: [f64; 5] = [f64::INFINITY, 263.0, 220.0, 196.0, 182.0];
/// `[REFERENCE SPEEDS] max_gear_extended` (flight_model.cfg:801); the cfg
/// gives one gear speed limit, used for both VLO and VLE.
const VLE_VLO_KT: f64 = 250.0;
/// `[REFERENCE SPEEDS] max_indicated_speed` / `max_mach` (flight_model.cfg
/// lines 787-788), FlyByWire's own VMO/MMO redlines.
const VMO_KT: f64 = 390.0;
const MMO: f64 = 0.97;
/// Airbus "A380 Aircraft Characteristics - Airport and Maintenance
/// Planning" (airbus.com/sites/g/files/jlcbta136/files/2021-11/
/// Airbus-Aircraft-AC-A380.pdf), lightest weight variant (WV000, MTOW
/// 510,000 kg = 1,124,355 lb), matching FlyByWire's own
/// `max_gross_weight` (flight_model.cfg:16): MLW 386,000 kg.
const MLW_KG: f64 = 386_000.0;

/// Tailstrike pitch angle, derived from FlyByWire's own contact-point
/// geometry (flight_model.cfg): the aft body gear (`point.1`/`point.2`,
/// z = -5.7 ft, y = -15.4 ft) is the pivot; the tailstrike point
/// (`point.17`, "Body tailstrike location", z = -72.402222 ft,
/// y = 0.002656 ft) is 66.702 ft further aft and 15.397 ft higher.
/// theta = atan(15.397 / 66.702) = 12.99 degrees.
const TAILSTRIKE_PITCH_DEG: f64 = 12.99;

/// Generic transport-category maintenance-manual convention for a
/// hard-landing inspection trigger (no public A380-specific figure found;
/// clearly marked as generic, not cited to Airbus): sink rate at or beyond
/// 600 ft/min, or normal load factor at or beyond 1.6 g, at touchdown.
const HARD_LANDING_VS_FPM: f64 = -600.0;
const HARD_LANDING_G: f64 = 1.6;

/// "Engine running" proxy for hours/cycle bookkeeping: EGT clearly above
/// ambient (a shutdown/cold engine's `ENGINE_EGT` sits at ambient; FlyByWire's
/// idle EGT is several hundred degrees, `fadec.rs` `idle_egt`). A derived
/// heuristic, not a published limit — only used for the wear clock, not for
/// any exceedance threshold.
const ENGINE_RUNNING_EGT_C: f64 = 150.0;

// ---------------------------------------------------------------------------
// Oil leak -> quantity -> pressure -> bearing-wear chain (failures::extra
// 79_000 oil pump fault, 79_004 oil leak). No published Trent 900 oil system
// capacity/leak-rate figure is available, so these are clearly-marked,
// order-of-magnitude derived planning figures (docs/physics/failures.md):
// sized so a leak is a genuine in-flight emergency (empties the sump over
// several minutes, not instantly and not over hours) and so running the
// bearings dry for any real length of time -- not just a momentary blip --
// is what does the damage, the same way real oil-starvation bearing
// failures develop.
// ---------------------------------------------------------------------------

/// A leak drains a generic-sized sump from full (100%) to empty over roughly
/// 6 minutes of running -- fast enough to matter within one flight, slow
/// enough to be a diagnosable trend rather than an instant cliff-edge.
const OIL_LEAK_DRAIN_PCT_PER_S: f64 = 100.0 / 360.0;
/// Below this quantity, the pump can no longer keep the gallery fully
/// pressurised (it starts drawing air): oil pressure falls off linearly from
/// here to zero at 0% quantity, mirroring how a real low-oil-quantity
/// caution precedes a low-oil-pressure one.
const OIL_STARVATION_QTY_PCT: f64 = 50.0;
/// A pump fault (79_000+n) is modelled as an immediate, direct pressure
/// shortfall (not a quantity drain: the oil is still there, the pump just
/// cannot move enough of it), independent of and on top of any quantity
/// effect.
const OIL_PUMP_FAULT_PRESSURE_FRACTION: f64 = 0.3;
/// Seconds of sustained oil starvation (pressure fraction below
/// [`OIL_STARVATION_ARM_FRACTION`]) before the bearings are considered
/// damaged -- a generic "don't run it dry for long" order-of-magnitude
/// figure, not a cited limit.
const OIL_STARVATION_ARM_SECONDS: f64 = 45.0;
const OIL_STARVATION_ARM_FRACTION: f64 = 0.4;

// ---------------------------------------------------------------------------
// Per-engine creep/life tracking.
// ---------------------------------------------------------------------------

/// Serde default for `EngineWear::oil_quantity_pct`: full, until a leak
/// (failures::extra 79_004+n) starts draining it.
fn full_oil_quantity() -> f64 {
    100.0
}

/// No exceedances recorded.
const NO_EXCEEDANCES: Exceedances = Exceedances {
    ground_start_tgt: 0,
    relight_tgt: 0,
    low_oil_pressure: 0,
    cold_start: 0,
    cold_takeoff: 0,
    oil_overtemperature: 0,
    hp_overspeed: 0,
    ip_overspeed: 0,
    lp_overspeed: 0,
    customer_bleed: 0,
};

/// Bleed flows below this are the pneumatic model settling, not a draw.
const BLEED_NOISE_KG_S: f64 = 0.01;

/// EASA.E.012 Note 16's TGT trim: the EEC shows the measured TGT less a
/// pre-determined amount. The data sheet gives that amount only through its
/// paired limits, and they pair by rating: take-off 900 displayed for 956
/// measured (Profile 5), maximum continuous 850 for 939. So the take-off
/// ratings (TOGA, FLEX) take the take-off column's trim and every other
/// rating the maximum-continuous column's. (The over-temperature pair, 920
/// for 957, is a separate limit on each scale, not a third trim: the two
/// are monitored independently.)
pub fn tgt_trim_c(takeoff_rating: bool) -> f64 {
    if takeoff_rating {
        trent900::TGT_TAKEOFF_UNTRIMMED_C - trent900::TGT_TAKEOFF_C
    } else {
        trent900::TGT_MAX_CONTINUOUS_UNTRIMMED_C - trent900::TGT_MAX_CONTINUOUS_C
    }
}

/// Engine limit exceedances of EASA.E.012, each counted once per event
/// (the logbook entries an engine monitoring unit would record).
#[derive(Clone, Copy, Default, Debug, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub struct Exceedances {
    pub ground_start_tgt: u32,
    pub relight_tgt: u32,
    pub low_oil_pressure: u32,
    pub cold_start: u32,
    pub cold_takeoff: u32,
    pub oil_overtemperature: u32,
    pub hp_overspeed: u32,
    pub ip_overspeed: u32,
    pub lp_overspeed: u32,
    /// Customer bleed above section 10's maximum for the port and T41.
    pub customer_bleed: u32,
}

#[derive(Clone, Copy, serde::Serialize, serde::Deserialize)]
pub struct EngineWear {
    /// Life fraction consumed by time-at-temperature creep and by TOGA held
    /// past its time limit (Miner's-rule-style linear accumulation: 1.0
    /// means a full allowed exceedance budget has been used up once over).
    /// Never resets except by a MEL repair action (`mel::Mel::repair`).
    pub creep_life_fraction: f64,
    /// Limit exceedances recorded so far (see [`Exceedances`]).
    #[serde(default)]
    pub exceedances: Exceedances,
    /// Which exceedance conditions held last tick, so each is counted once
    /// per event (same order as `Exceedances`' fields).
    #[serde(skip)]
    in_exceedance: [bool; 10],
    #[serde(skip)]
    seconds_ip_overspeed: f64,
    /// Derived, saturating function of `creep_life_fraction`, republished
    /// as the engine workstream's hook.
    pub compressor_efficiency_loss: f64,
    /// Hours the "running" proxy has been true (engine hours).
    pub hours: f64,
    /// Times the "running" proxy transitioned false -> true (start cycles).
    pub cycles: u32,
    was_running: bool,
    /// Percent of full oil quantity, drained by failures::extra
    /// 79_004+n ("oil leak") in [`Damage::update_engines`] and never
    /// otherwise consumed (there is no oil-consumption physics outside a
    /// leak) -- 100 until a leak starts. Persisted, like `hours`/`cycles`,
    /// so a leak's effect survives a session the way a real leak would.
    #[serde(default = "full_oil_quantity")]
    pub oil_quantity_pct: f64,
    /// Seconds this engine has been starved of oil past
    /// `OIL_STARVATION_QTY_PCT` (leak) or under a pump fault, for the
    /// sustained-starvation bearing-damage timer.
    #[serde(skip)]
    seconds_oil_starved: f64,
    /// Seconds continuously with TGT above `TGT_MAX_CONTINUOUS_C` this
    /// takeoff/go-around (not persisted: reset to zero the moment TGT drops
    /// back, and never survives a session anyway once it's below the
    /// 60-second range that matters).
    #[serde(skip)]
    seconds_above_mct: f64,
    #[serde(skip)]
    seconds_above_overtemp: f64,
    /// Seconds the thrust lever has continuously been in the TOGA detent.
    #[serde(skip)]
    toga_lever_seconds: f64,
}

impl Default for EngineWear {
    fn default() -> Self {
        Self {
            creep_life_fraction: 0.0,
            compressor_efficiency_loss: 0.0,
            hours: 0.0,
            cycles: 0,
            was_running: false,
            oil_quantity_pct: full_oil_quantity(),
            seconds_oil_starved: 0.0,
            seconds_above_mct: 0.0,
            seconds_above_overtemp: 0.0,
            toga_lever_seconds: 0.0,
            exceedances: NO_EXCEEDANCES,
            in_exceedance: [false; 10],
            seconds_ip_overspeed: 0.0,
        }
    }
}

pub struct Damage {
    /// Measured (untrimmed) TGT, `ENGINE_EGT_UNTRIMMED:n`: what the
    /// engine's metal sees, held to the untrimmed limits.
    egt: [VariableIdentifier; 4],
    /// Displayed (trimmed) TGT, `ENGINE_EGT:n`, held to the limits the
    /// data sheet states only on that scale (start, relight).
    egt_displayed: [VariableIdentifier; 4],
    bleed_flow: [VariableIdentifier; 4],
    bleed_limit: [VariableIdentifier; 4],
    tla: [VariableIdentifier; 4],
    n1: [VariableIdentifier; 4],
    n2: [VariableIdentifier; 4],
    n3: [VariableIdentifier; 4],
    oil_press: [VariableIdentifier; 4],
    oil_temp: [VariableIdentifier; 4],
    engine_state: [VariableIdentifier; 4],
    creep_fraction_out: [VariableIdentifier; 4],
    efficiency_loss_out: [VariableIdentifier; 4],
    /// Oil pressure available as a fraction of the normal (leak-free,
    /// pump-healthy) value: 1.0 normally, falling as a leak drains the sump
    /// past `OIL_STARVATION_QTY_PCT` or a pump fault caps it at
    /// `OIL_PUMP_FAULT_PRESSURE_FRACTION`. The engine workstream's own
    /// `oil_press_psi` (`physics/engine/mod.rs`) has no leak/pump-fault
    /// input yet, so this is published as a hook
    /// (`ENGINE_OIL_PRESSURE_FRACTION:n`) for it to multiply in, the same
    /// pattern as `creep_fraction_out`/`efficiency_loss_out`.
    oil_pressure_fraction_out: [VariableIdentifier; 4],
    pub engines: [EngineWear; 4],

    apu_egt: VariableIdentifier,
    apu_egt_warning: VariableIdentifier,
    apu_overtemp_seconds: f64,
    /// Percent of full APU oil quantity, drained by failures::extra 49_005
    /// ("APU oil leak") the same way `EngineWear::oil_quantity_pct` is —
    /// falling past `OIL_STARVATION_QTY_PCT` arms 49_004 ("APU oil low")
    /// directly, the physical consequence rather than a standalone flag.
    apu_oil_pct: f64,
    apu_oil_pressure_fraction_out: VariableIdentifier,

    /// FlyByWire's own real per-wheel brake temperature (`Brake::update`,
    /// `fbw-common/.../hydraulic/brake.rs`), 16 positions: left wing
    /// `[1,2,5,6]`, right wing `[3,4,7,8]`, left body `[9,10,13,14]`, right
    /// body `[11,12,15,16]` (`a380_systems/src/hydraulic/mod.rs:2074-2101`).
    /// Always available regardless of BTMU power (unlike the cockpit-facing
    /// `REPORTED_BRAKE_TEMPERATURE_n`), since a wheel's physical heat does
    /// not care whether its gauge is powered.
    brake_temperature: [VariableIdentifier; 16],

    flaps_handle_index: VariableIdentifier,

    ias: Option<DataRef>,
    mach: Option<DataRef>,
    vs_fpm: Option<DataRef>,
    on_ground: Option<DataRef>,
    gear_handle_down: Option<DataRef>,
    gforce_normal: Option<DataRef>,
    pitch_deg: Option<DataRef>,
    mass_kg: Option<DataRef>,
    groundspeed_ms: Option<DataRef>,
    wheel_brake_ratio: Option<DataRef>,

    was_on_ground: bool,
    /// Seconds spent continuously airborne since the last time on the
    /// ground, so a touchdown can be told apart from start-of-plugin noise
    /// (see `MIN_AIRBORNE_SECONDS_FOR_LANDING`).
    time_airborne_s: f64,
    /// Accumulated, thermally-decaying brake energy (J), a derived scale
    /// (docs/physics/failures.md): not a cited AFM/cert limit.
    brake_energy_j: f64,
    prev_groundspeed_ms: f64,

    /// Log lines for exceedance events this tick (Study/log consumption).
    pub events: Vec<String>,
}

/// Reference mass for the brake-energy scale: MLW at a typical heavy-jet
/// reference approach speed band (140 kt = 72 m/s), giving a first-
/// principles kinetic-energy scale with no published brake-specific number
/// (docs/physics/failures.md marks this as derived, not a cert limit).
const BRAKE_ENERGY_REFERENCE_J: f64 = 0.5 * MLW_KG * (72.0_f64 * 72.0_f64);
/// Generic transport-category brake thermal time constant order of
/// magnitude (cooling over several minutes with no forced fan cooling);
/// marked generic in the docs table.
const BRAKE_COOLING_TAU_S: f64 = 600.0;

/// Wheel/brake position groupings, FlyByWire's own indexing
/// (`a380_systems/src/hydraulic/mod.rs:2074-2101`, `BRAKE_TEMPERATURE_1..16`,
/// 0-based here): left wing, right wing, left body, right body, each 4
/// wheels. Matches `failures::extra::gear`'s tyre-burst ordering
/// (32_101 left wing .. 32_104 right body).
const LEG_WHEEL_INDICES: [[usize; 4]; 4] = [
    [0, 1, 4, 5],   // left wing: BRAKE_TEMPERATURE_1,2,5,6
    [2, 3, 6, 7],   // right wing: 3,4,7,8
    [8, 9, 12, 13], // left body: 9,10,13,14
    [10, 11, 14, 15], // right body: 11,12,15,16
];

/// Thermal (fusible) wheel plug melting point: a widely-cited generic
/// figure for the eutectic-alloy fuse plugs used in transport-category
/// wheels (order of magnitude discussed in FAA AC 20-97B "Aircraft Tire
/// Maintenance and Operational Practices"); no A380-specific wheel data
/// sheet figure was available in this pass, so this is explicitly generic,
/// not cited to Airbus/Safran, matching this file's convention for
/// uncited thresholds (see `HARD_LANDING_VS_FPM` above).
pub(crate) const FUSE_PLUG_MELT_C: f64 = 177.0;

impl Damage {
    /// `xplm` is `None` in unit tests (there is no X-Plane host to bind
    /// to): every raw dataref then reads as `0`/`false`, so only the
    /// `Vars`-driven engine/APU exceedance paths are exercisable outside
    /// X-Plane. In production `Correctness`/`Plugin::new` always pass
    /// `Some`.
    pub fn new<W: systems::simulation::VariableRegistry>(vars: &mut W, xplm: Option<&Xplm>) -> Self {
        Self {
            egt: std::array::from_fn(|i| vars.get(format!("ENGINE_EGT_UNTRIMMED:{}", i + 1))),
            egt_displayed: std::array::from_fn(|i| vars.get(format!("ENGINE_EGT:{}", i + 1))),
            bleed_flow: std::array::from_fn(|i| vars.get(format!("ENGINE_BLEED_EXTRACTION_KG_S:{}", i + 1))),
            bleed_limit: std::array::from_fn(|i| vars.get(format!("ENGINE_BLEED_LIMIT_KG_S:{}", i + 1))),
            tla: std::array::from_fn(|i| vars.get(format!("A32NX_AUTOTHRUST_TLA:{}", i + 1))),
            n1: std::array::from_fn(|i| vars.get(format!("ENGINE_N1:{}", i + 1))),
            n2: std::array::from_fn(|i| vars.get(format!("ENGINE_N2:{}", i + 1))),
            n3: std::array::from_fn(|i| vars.get(format!("ENGINE_N3:{}", i + 1))),
            oil_press: std::array::from_fn(|i| vars.get(format!("GENERAL ENG OIL PRESSURE:{}", i + 1))),
            oil_temp: std::array::from_fn(|i| vars.get(format!("GENERAL ENG OIL TEMPERATURE:{}", i + 1))),
            engine_state: std::array::from_fn(|i| vars.get(format!("ENGINE_STATE:{}", i + 1))),
            creep_fraction_out: std::array::from_fn(|i| vars.get(format!("ENGINE_CREEP_LIFE_FRACTION:{}", i + 1))),
            efficiency_loss_out: std::array::from_fn(|i| vars.get(format!("ENGINE_COMPRESSOR_EFFICIENCY_LOSS:{}", i + 1))),
            oil_pressure_fraction_out: std::array::from_fn(|i| vars.get(format!("ENGINE_OIL_PRESSURE_FRACTION:{}", i + 1))),
            engines: Default::default(),
            apu_egt: vars.get("APU_EGT".to_owned()),
            apu_egt_warning: vars.get("APU_EGT_WARNING".to_owned()),
            apu_overtemp_seconds: 0.0,
            apu_oil_pct: full_oil_quantity(),
            apu_oil_pressure_fraction_out: vars.get("APU_OIL_PRESSURE_FRACTION".to_owned()),
            brake_temperature: std::array::from_fn(|i| vars.get(format!("BRAKE_TEMPERATURE_{}", i + 1))),
            flaps_handle_index: vars.get("FLAPS_HANDLE_INDEX".to_owned()),
            ias: xplm.and_then(|x| x.find("sim/flightmodel/position/indicated_airspeed")),
            mach: xplm.and_then(|x| x.find("sim/flightmodel/misc/machno")),
            vs_fpm: xplm.and_then(|x| x.find("sim/flightmodel/position/vh_ind_fpm")),
            on_ground: xplm.and_then(|x| x.find("sim/flightmodel/failures/onground_any")),
            gear_handle_down: xplm.and_then(|x| x.find("sim/cockpit2/controls/gear_handle_down")),
            gforce_normal: xplm.and_then(|x| x.find("sim/flightmodel2/misc/gforce_normal")),
            pitch_deg: xplm.and_then(|x| x.find("sim/flightmodel/position/theta")),
            mass_kg: xplm.and_then(|x| x.find("sim/flightmodel/weight/m_total")),
            groundspeed_ms: xplm.and_then(|x| x.find("sim/flightmodel/position/groundspeed")),
            wheel_brake_ratio: xplm.and_then(|x| x.find("sim/cockpit2/controls/wheel_brake_ratio")),
            was_on_ground: true,
            time_airborne_s: 0.0,
            brake_energy_j: 0.0,
            prev_groundspeed_ms: 0.0,
            events: Vec::new(),
        }
    }

    fn get_f(xplm: Option<&Xplm>, d: Option<DataRef>) -> f64 {
        match (xplm, d) {
            (Some(xplm), Some(d)) => xplm.get_f(d) as f64,
            _ => 0.,
        }
    }
    fn get_i(xplm: Option<&Xplm>, d: Option<DataRef>) -> i32 {
        match (xplm, d) {
            (Some(xplm), Some(d)) => xplm.get_i(d),
            _ => 0,
        }
    }

    /// Call once per tick, after the systems tick (so this frame's EGT/TLA
    /// are current). `delta` is real seconds (already zero while paused, if
    /// the caller uses the same `paused()` gate as `random_failures`).
    pub fn update<W: systems::simulation::VariableRegistry + systems::simulation::SimulatorReaderWriter>(
        &mut self,
        vars: &mut W,
        xplm: Option<&Xplm>,
        delta: f64,
    ) {
        self.events.clear();
        if delta <= 0.0 {
            return;
        }
        let on_ground = Self::get_i(xplm, self.on_ground) != 0 || xplm.is_none();
        self.update_engines(vars, delta);
        self.update_engine_limits(vars, on_ground, delta);
        self.update_apu(vars, delta);
        self.update_flap_gear_speed_overspeeds(vars, xplm);
        self.update_vmo_mmo(xplm);
        self.update_brake_energy(xplm, delta);
        self.update_brake_temperature_tyre_burst(vars);
        self.update_touchdown_exceedances(xplm, delta);
    }

    fn arm(&mut self, id: u64, description: &str) {
        if !failures::active_ids().contains(&id) {
            failures::set_active(id, true);
            self.events.push(format!("exceedance: {description} (failure {id})"));
        }
    }

    /// EASA.E.012's remaining limits, recorded as exceedances: start and
    /// relight TGT, minimum oil pressure, oil temperature for starting and
    /// take-off and its maximum, and rotor speeds.
    fn update_engine_limits<W: systems::simulation::SimulatorReaderWriter>(&mut self, vars: &mut W, on_ground: bool, delta: f64) {
        use crate::fadec::EngineState as S;
        for n in 0..4 {
            let egt = vars.read(&self.egt_displayed[n]);
            let bleed = vars.read(&self.bleed_flow[n]);
            let bleed_limit = vars.read(&self.bleed_limit[n]);
            let (n1, n2, n3) = (vars.read(&self.n1[n]), vars.read(&self.n2[n]), vars.read(&self.n3[n]));
            let oil_press = vars.read(&self.oil_press[n]);
            let oil_temp = vars.read(&self.oil_temp[n]);
            let state = S::from(vars.read(&self.engine_state[n]));
            let starting = matches!(state, S::Starting | S::Restarting);
            let running = matches!(state, S::On);
            let e = &mut self.engines[n];
            if n2 > trent900::IP_OVERSPEED_PCT {
                e.seconds_ip_overspeed += delta;
            } else {
                e.seconds_ip_overspeed = 0.0;
            }
            let conditions = [
                (on_ground && starting && n3 < trent900::GROUND_START_HP_PCT && egt > trent900::TGT_GROUND_START_C, "ground start TGT above 700 C"),
                (!on_ground && starting && egt > trent900::TGT_RELIGHT_C, "in-flight relight TGT above 850 C"),
                (
                    running
                        && ((n3 < trent900::OIL_PRESS_LOW_BAND_HP_PCT && oil_press < trent900::OIL_PRESS_MIN_LOW_PSI)
                            || (n3 > trent900::OIL_PRESS_HIGH_BAND_HP_PCT && oil_press < trent900::OIL_PRESS_MIN_HIGH_PSI)),
                    "oil pressure below minimum",
                ),
                (starting && oil_temp < trent900::OIL_TEMP_MIN_START_C, "start with oil below -30 C (Special Starting procedure required; -40 C is the absolute minimum)"),
                (running && on_ground && n1 > 80.0 && oil_temp < trent900::OIL_TEMP_MIN_TAKEOFF_C, "take-off power with oil below 40 C"),
                (oil_temp > trent900::OIL_TEMP_MAX_C, "oil temperature above 196 C"),
                (n3 > trent900::HP_TAKEOFF_MAX_PCT, "HP speed above 97.8%"),
                (e.seconds_ip_overspeed > trent900::IP_OVERSPEED_LIMIT_S, "IP over-speed above 99.5% for more than 20 s"),
                (n1 > trent900::LP_TAKEOFF_MAX_PCT, "LP speed above 97.2%"),
                (
                    running && bleed > BLEED_NOISE_KG_S && bleed > bleed_limit,
                    "customer bleed above the data sheet's maximum for its port and turbine entry temperature",
                ),
            ];
            for (k, (holds, what)) in conditions.into_iter().enumerate() {
                if holds && !e.in_exceedance[k] {
                    let x = &mut e.exceedances;
                    let counter = [
                        &mut x.ground_start_tgt,
                        &mut x.relight_tgt,
                        &mut x.low_oil_pressure,
                        &mut x.cold_start,
                        &mut x.cold_takeoff,
                        &mut x.oil_overtemperature,
                        &mut x.hp_overspeed,
                        &mut x.ip_overspeed,
                        &mut x.lp_overspeed,
                        &mut x.customer_bleed,
                    ];
                    *counter.into_iter().nth(k).expect("ten counters") += 1;
                    self.events.push(format!("engine {} exceedance: {what}", n + 1));
                }
                e.in_exceedance[k] = holds;
            }
        }
    }

    fn update_engines<W: systems::simulation::SimulatorReaderWriter>(&mut self, vars: &mut W, delta: f64) {
        // Any engine below the running threshold this tick is a candidate
        // "engine out" for the OEI (10-minute) time limit; a full 4-engine
        // takeoff uses the 5-minute limit.
        let egt_c: [f64; 4] = std::array::from_fn(|i| vars.read(&self.egt[i]));
        let oei = egt_c.iter().filter(|&&t| t < ENGINE_RUNNING_EGT_C).count() >= 1;
        let takeoff_limit_s = if oei { trent900::TAKEOFF_OEI_LIMIT_S } else { trent900::TAKEOFF_LIMIT_S };

        for n in 0..4 {
            let t = egt_c[n];
            // Computed while `self.engines[n]` is borrowed, then acted on
            // (via `self.arm`, which needs its own `&mut self`) only after
            // that borrow ends, since the two can't be live at once.
            let mut overtemp_exceeded = false;
            let (creep_life_fraction, compressor_efficiency_loss) = {
                let e = &mut self.engines[n];

                let running = t >= ENGINE_RUNNING_EGT_C;
                if running {
                    e.hours += delta / 3600.0;
                    if !e.was_running {
                        e.cycles += 1;
                    }
                }
                e.was_running = running;

                if t > trent900::TGT_OVERTEMP_UNTRIMMED_C {
                    e.seconds_above_overtemp += delta;
                    // Miner's-rule linear damage: the 20-second overtemperature
                    // budget consumed at 1x while above it.
                    e.creep_life_fraction += delta / trent900::OVERTEMP_LIMIT_S;
                    if e.seconds_above_overtemp > trent900::OVERTEMP_LIMIT_S {
                        overtemp_exceeded = true;
                    }
                } else {
                    e.seconds_above_overtemp = 0.0;
                }

                if t > trent900::TGT_MAX_CONTINUOUS_UNTRIMMED_C {
                    e.seconds_above_mct += delta;
                    e.creep_life_fraction += delta / takeoff_limit_s;
                } else {
                    e.seconds_above_mct = 0.0;
                }

                let tla = vars.read(&self.tla[n]);
                if tla >= TLA_TOGA_DEG - 0.1 {
                    e.toga_lever_seconds += delta;
                    if e.toga_lever_seconds > takeoff_limit_s {
                        // A smaller, independent contributor: the lever was
                        // held at TOGA past the logbook-exceedance time even if
                        // EGT itself stayed under the redline on a cold day.
                        e.creep_life_fraction += delta / (takeoff_limit_s * 10.0);
                    }
                } else {
                    e.toga_lever_seconds = 0.0;
                }

                // Saturating efficiency loss: up to 15% (an order-of-magnitude
                // ceiling, not a cited figure) as creep life is consumed.
                e.compressor_efficiency_loss = (e.creep_life_fraction * 0.05).min(0.15);
                (e.creep_life_fraction, e.compressor_efficiency_loss)
            };

            if overtemp_exceeded {
                self.arm(72_008 + n as u64, &format!("engine {} turbine overtemperature", n + 1));
            }
            // failures::extra's per-engine block: base 72_000 bearing wear,
            // 72_004 compressor stall, 72_008 turbine blade damage (armed
            // above, from the 20s/920C path). One full exceedance budget
            // used up (>=1.0) risks bearing wear; two (>=2.0) risk a stall.
            if creep_life_fraction >= 1.0 {
                self.arm(72_000 + n as u64, &format!("engine {} bearing wear", n + 1));
            }
            if creep_life_fraction >= 2.0 {
                self.arm(72_004 + n as u64, &format!("engine {} compressor stall risk", n + 1));
            }

            // 79_004+n oil leak -> quantity drops -> pressure fraction drops
            // -> sustained starvation arms 72_000+n bearing wear directly, a
            // second, independent physical route into the same failure
            // creep already arms from overtemperature. 79_000+n oil pump
            // fault caps pressure fraction the same way without touching
            // quantity (the oil is there, the pump just can't move it).
            let leaking = failures::active_ids().contains(&(79_004 + n as u64));
            let pump_fault = failures::active_ids().contains(&(79_000 + n as u64));
            let oil_pressure_fraction = {
                let e = &mut self.engines[n];
                if leaking {
                    e.oil_quantity_pct = (e.oil_quantity_pct - OIL_LEAK_DRAIN_PCT_PER_S * delta).max(0.0);
                }
                let quantity_fraction = if e.oil_quantity_pct >= OIL_STARVATION_QTY_PCT {
                    1.0
                } else {
                    (e.oil_quantity_pct / OIL_STARVATION_QTY_PCT).clamp(0.0, 1.0)
                };
                let fraction = if pump_fault {
                    quantity_fraction.min(OIL_PUMP_FAULT_PRESSURE_FRACTION)
                } else {
                    quantity_fraction
                };

                if fraction < OIL_STARVATION_ARM_FRACTION {
                    e.seconds_oil_starved += delta;
                } else {
                    e.seconds_oil_starved = 0.0;
                }
                fraction
            };
            if self.engines[n].seconds_oil_starved > OIL_STARVATION_ARM_SECONDS {
                self.arm(72_000 + n as u64, &format!("engine {} bearing wear (oil starvation)", n + 1));
            }

            vars.write(&self.creep_fraction_out[n], creep_life_fraction);
            vars.write(&self.efficiency_loss_out[n], compressor_efficiency_loss);
            vars.write(&self.oil_pressure_fraction_out[n], oil_pressure_fraction);
        }
    }

    fn update_apu<W: systems::simulation::SimulatorReaderWriter>(&mut self, vars: &mut W, delta: f64) {
        let egt = vars.read(&self.apu_egt);
        let warning = vars.read(&self.apu_egt_warning);
        if warning > 0.0 && egt > warning {
            self.apu_overtemp_seconds += delta;
            // A generic 60-second sustained-overtemperature trigger (no
            // published APU time limit found): documented as generic.
            if self.apu_overtemp_seconds > 60.0 {
                self.arm(49_000, "APU EGT overtemperature damage");
            }
        } else {
            self.apu_overtemp_seconds = 0.0;
        }

        // 49_004 APU oil leak: drains apu_oil_pct exactly like an engine oil
        // leak drains EngineWear::oil_quantity_pct (79_004), and the same
        // below-threshold falloff into a published pressure-fraction hook
        // for the (not yet built) APU/fluids consumer to read -- the same
        // "Local, but still publishes a real hook downstream" pattern
        // `update_engines` uses for `oil_pressure_fraction_out`.
        if failures::active_ids().contains(&49_004) {
            self.apu_oil_pct = (self.apu_oil_pct - OIL_LEAK_DRAIN_PCT_PER_S * delta).max(0.0);
        }
        let apu_oil_pressure_fraction = if self.apu_oil_pct >= OIL_STARVATION_QTY_PCT {
            1.0
        } else {
            (self.apu_oil_pct / OIL_STARVATION_QTY_PCT).clamp(0.0, 1.0)
        };
        vars.write(&self.apu_oil_pressure_fraction_out, apu_oil_pressure_fraction);
    }

    fn update_flap_gear_speed_overspeeds<W: systems::simulation::SimulatorReaderWriter>(&mut self, vars: &mut W, xplm: Option<&Xplm>) {
        let ias = Self::get_f(xplm, self.ias);
        let flap_index = vars.read(&self.flaps_handle_index).round().clamp(0.0, 4.0) as usize;
        if ias > VFE_KT[flap_index] {
            self.arm(27_100, &format!("flap overspeed at CONF {flap_index} ({ias:.0} kt)"));
        }
        let gear_down = Self::get_i(xplm, self.gear_handle_down) != 0;
        if gear_down && ias > VLE_VLO_KT {
            self.arm(32_120, &format!("gear overspeed ({ias:.0} kt)"));
        }
    }

    fn update_vmo_mmo(&mut self, xplm: Option<&Xplm>) {
        let ias = Self::get_f(xplm, self.ias);
        let mach = Self::get_f(xplm, self.mach);
        if ias > VMO_KT || mach > MMO {
            self.arm(34_120, &format!("VMO/MMO overspeed ({ias:.0} kt / M{mach:.2})"));
        }
    }

    fn update_brake_energy(&mut self, xplm: Option<&Xplm>, delta: f64) {
        let on_ground = Self::get_i(xplm, self.on_ground) != 0;
        let brake = Self::get_f(xplm, self.wheel_brake_ratio);
        let v = Self::get_f(xplm, self.groundspeed_ms);
        let mass = Self::get_f(xplm, self.mass_kg);

        // Thermal decay first (Newtonian cooling toward zero).
        self.brake_energy_j *= (-delta / BRAKE_COOLING_TAU_S).exp();

        if on_ground && brake > 0.05 && v < self.prev_groundspeed_ms {
            // Kinetic energy lost this tick, apportioned to the brakes by
            // the commanded brake ratio (the rest is aerodynamic/rolling
            // drag and reverse thrust) — a simplification, documented.
            let ke_lost = 0.5 * mass * (self.prev_groundspeed_ms.powi(2) - v.powi(2));
            if ke_lost > 0.0 {
                self.brake_energy_j += ke_lost * brake as f64;
            }
        }
        self.prev_groundspeed_ms = v;

        if self.brake_energy_j > BRAKE_ENERGY_REFERENCE_J {
            self.arm(32_110, "left wing gear brake wear-out (brake energy)");
            self.arm(32_111, "right wing gear brake wear-out (brake energy)");
        }
    }

    /// Real per-wheel fuse-plug/tyre-burst arm (docs/physics/
    /// landing-gear-brakes.md gap #3): re-points the tyre-burst failures at
    /// FlyByWire's own `BRAKE_TEMPERATURE_n` (real heat from actuator
    /// pressure and wheel speed, `Brake::update`) instead of the coarse
    /// brake-energy heuristic. Any one wheel's brake past the fuse-plug
    /// melting point is a real physical cause for that leg's tyre to lose
    /// pressure and burst under load, independent of anything else this
    /// module tracks.
    fn update_brake_temperature_tyre_burst<W: systems::simulation::SimulatorReaderWriter>(&mut self, vars: &mut W) {
        let temps: [f64; 16] = std::array::from_fn(|i| vars.read(&self.brake_temperature[i]));
        // 32_101..32_104: left wing, right wing, left body, right body
        // (`failures::extra::gear`'s ordering after the nose leg 32_100,
        // which has no brakes and so no entry here).
        for (leg, indices) in LEG_WHEEL_INDICES.iter().enumerate() {
            let hottest = indices.iter().map(|&i| temps[i]).fold(f64::MIN, f64::max);
            if hottest > FUSE_PLUG_MELT_C {
                let names = ["left wing", "right wing", "left body", "right body"];
                self.arm(32_101 + leg as u64, &format!("{} gear tyre burst (brake fuse-plug overheat, {hottest:.0} C)", names[leg]));
            }
        }
    }

    /// Minimum time actually airborne before an `on_ground` transition
    /// counts as a landing. A cold start already parked on the ramp, or a
    /// one-frame blip of `sim/flightmodel/failures/onground_any` while the
    /// aircraft settles onto its gear at spawn, is not a flight -- and a
    /// long-haul A380 is routinely well above `MLW_KG` while still parked
    /// (that is normal before an actual departure, not an exceedance), so
    /// an ungated transition mislabels the parked aircraft as having just
    /// "landed" overweight. A real circuit -- even a touch-and-go -- is
    /// airborne far longer than this.
    const MIN_AIRBORNE_SECONDS_FOR_LANDING: f64 = 5.0;

    /// Whether an `on_ground` transition is a genuine touchdown rather than
    /// start-of-plugin noise: it must actually be a transition, and it must
    /// have followed a plausible period of real flight.
    fn is_genuine_touchdown(on_ground: bool, was_on_ground: bool, time_airborne_s: f64) -> bool {
        on_ground && !was_on_ground && time_airborne_s >= Self::MIN_AIRBORNE_SECONDS_FOR_LANDING
    }

    fn update_touchdown_exceedances(&mut self, xplm: Option<&Xplm>, delta: f64) {
        let on_ground = Self::get_i(xplm, self.on_ground) != 0;
        if !on_ground {
            self.time_airborne_s += delta;
        }
        if Self::is_genuine_touchdown(on_ground, self.was_on_ground, self.time_airborne_s) {
            // The instant of touchdown.
            let vs = Self::get_f(xplm, self.vs_fpm);
            let g = Self::get_f(xplm, self.gforce_normal);
            let pitch = Self::get_f(xplm, self.pitch_deg);
            let mass = Self::get_f(xplm, self.mass_kg);

            if vs < HARD_LANDING_VS_FPM || g > HARD_LANDING_G {
                self.arm(32_121, &format!("hard landing (VS {vs:.0} fpm, {g:.2} g)"));
            }
            if pitch > TAILSTRIKE_PITCH_DEG {
                self.arm(32_122, &format!("tailstrike ({pitch:.1} deg pitch at touchdown)"));
            }
            if mass > MLW_KG {
                self.arm(32_123, &format!("overweight landing ({mass:.0} kg vs {MLW_KG:.0} kg MLW)"));
            }
        }
        if on_ground {
            self.time_airborne_s = 0.0;
        }
        self.was_on_ground = on_ground;
    }

    /// Apply the Study panel's queued "repair engine n" requests (the
    /// maintenance action that resets an engine's wear, alongside
    /// `mel::Mel::repair` clearing the failure itself). Returns the log
    /// lines. Same request-queue pattern as `mel::request_defer`.
    pub fn apply_requests(&mut self) -> Vec<String> {
        let mut log = Vec::new();
        for n in take_repair_requests() {
            if let Some(e) = self.engines.get_mut(n) {
                *e = EngineWear::default();
                log.push(format!("engine {} wear reset (maintenance repair)", n + 1));
            }
        }
        log
    }
}

/// The Study panel's per-engine wear/damage display and its "repair"
/// button: `publish`/`snapshot` share the latest `[EngineWear; 4]` (updated
/// once per tick, `Plugin::tick`); `request_repair_engine`/
/// `Damage::apply_requests` are the same queued-request pattern
/// `mel::request_defer` uses, since the Study page has no direct access to
/// the running `Damage` (owned by `Plugin`).
static LATEST_WEAR: std::sync::Mutex<[EngineWear; 4]> = std::sync::Mutex::new([EngineWear {
    creep_life_fraction: 0.0,
    compressor_efficiency_loss: 0.0,
    hours: 0.0,
    cycles: 0,
    was_running: false,
    oil_quantity_pct: 100.0,
    seconds_oil_starved: 0.0,
    seconds_above_mct: 0.0,
    seconds_above_overtemp: 0.0,
    toga_lever_seconds: 0.0,
    exceedances: NO_EXCEEDANCES,
    in_exceedance: [false; 10],
    seconds_ip_overspeed: 0.0,
}; 4]);

pub fn publish(engines: [EngineWear; 4]) {
    if let Ok(mut w) = LATEST_WEAR.lock() {
        *w = engines;
    }
}

pub fn snapshot() -> [EngineWear; 4] {
    LATEST_WEAR.lock().map(|w| *w).unwrap_or_default()
}

static REPAIR_REQUESTS: std::sync::Mutex<Vec<usize>> = std::sync::Mutex::new(Vec::new());

/// `n` is 1-based (engine 1..=4), matching the failure catalogue's naming.
pub fn request_repair_engine(n: usize) {
    if n >= 1 && n <= 4 {
        if let Ok(mut r) = REPAIR_REQUESTS.lock() {
            r.push(n - 1);
        }
    }
}

fn take_repair_requests() -> Vec<usize> {
    REPAIR_REQUESTS.lock().map(|mut r| std::mem::take(&mut *r)).unwrap_or_default()
}

/// Test-isolation helper (see `scenarios::reset_global_state`): clears this
/// module's two process-global `static`s -- the Study panel's published
/// wear snapshot and any queued repair request -- so one test's engine wear
/// display or pending repair can never leak into the next test that shares
/// this process. Does not touch a `Damage` instance's own `engines` field,
/// which is per-test/owned, not global.
#[cfg(any(test, feature = "test-support"))]
pub fn reset_for_tests() {
    if let Ok(mut w) = LATEST_WEAR.lock() {
        *w = [EngineWear::default(); 4];
    }
    if let Ok(mut r) = REPAIR_REQUESTS.lock() {
        r.clear();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_tgt_trim_maps_each_rating_limit_onto_its_displayed_limit() {
        // EASA.E.012 Note 16, Profile 5: 956 measured shows 900 at take-off,
        // 939 measured shows 850 at maximum continuous.
        assert_eq!(trent900::TGT_TAKEOFF_UNTRIMMED_C - tgt_trim_c(true), trent900::TGT_TAKEOFF_C);
        assert_eq!(trent900::TGT_MAX_CONTINUOUS_UNTRIMMED_C - tgt_trim_c(false), trent900::TGT_MAX_CONTINUOUS_C);
    }
    use crate::aspects::test_vars::TestVars;
    use systems::simulation::{SimulatorReaderWriter, VariableRegistry};

    /// Sustained EGT above the Trent 900's 850 C maximum-continuous TGT,
    /// with all four engines "running" (so the 5-minute all-engines limit
    /// applies, not the 10-minute OEI one — EASA.E.012 §IV.1.2 Note 5),
    /// accumulates creep life and arms the bearing-wear failure once the
    /// takeoff time limit is exceeded.
    #[test]
    fn sustained_egt_above_mct_arms_bearing_wear_after_the_takeoff_time_limit() {
        let _guard = crate::failures::tests::SERIAL.lock().unwrap_or_else(|e| e.into_inner());
        // `arm` -> `failures::set_active` is a no-op for an id that hasn't
        // been registered yet (failures.rs:849), and nothing in this test's
        // own path (`Damage::new`, `update`) registers 72_000 -- only
        // building a `Failures` does. Without this, the assertion below
        // passed or failed depending on whether some other test happened to
        // have constructed a `Failures` earlier in the same test binary.
        let _f = crate::failures::Failures::new();
        crate::failures::replace([]);
        let mut vars = TestVars::default();
        let mut d = Damage::new(&mut vars, None);
        for n in 0..4 {
            vars.write(&d.egt[n], 945.0); // above the 939 C (untrimmed) max continuous, all engines "running"
        }

        // Just under the 5-minute limit: no failure yet.
        for _ in 0..299 {
            d.update(&mut vars, None, 1.0);
        }
        assert!(!crate::failures::active_ids().contains(&72_000), "not armed yet at {}", d.engines[0].creep_life_fraction);

        // Past the limit.
        for _ in 0..5 {
            d.update(&mut vars, None, 1.0);
        }
        assert!(crate::failures::active_ids().contains(&72_000), "72000 should be armed past the 5-minute limit");
        assert!(d.engines[0].creep_life_fraction >= 1.0);
        assert!(d.engines[0].compressor_efficiency_loss > 0.0);
        crate::failures::replace([]);
    }

    /// A single engine below the running-EGT proxy makes the other three
    /// "engine failed" candidates, switching the takeoff time limit from 5
    /// to 10 minutes (EASA.E.012 Note 5): the same sustained overtemperature
    /// that would have armed the failure at 5 minutes must not yet have
    /// armed it just past 5 minutes when one engine is out.
    #[test]
    fn an_engine_out_extends_the_takeoff_time_limit_to_ten_minutes() {
        let _guard = crate::failures::tests::SERIAL.lock().unwrap_or_else(|e| e.into_inner());
        crate::failures::replace([]);
        let mut vars = TestVars::default();
        let mut d = Damage::new(&mut vars, None);
        vars.write(&d.egt[0], 860.0);
        vars.write(&d.egt[1], 860.0);
        vars.write(&d.egt[2], 860.0);
        vars.write(&d.egt[3], 20.0); // engine 4 shut down/failed

        for _ in 0..(5 * 60 + 30) {
            d.update(&mut vars, None, 1.0);
        }
        assert!(!crate::failures::active_ids().contains(&72_000), "the OEI limit is 10 minutes, not 5");
        crate::failures::replace([]);
    }

    #[test]
    fn egt_at_or_below_max_continuous_never_accumulates_creep() {
        let mut vars = TestVars::default();
        let mut d = Damage::new(&mut vars, None);
        for n in 0..4 {
            vars.write(&d.egt[n], 840.0); // below 850 C
        }
        for _ in 0..3600 {
            d.update(&mut vars, None, 1.0);
        }
        assert_eq!(d.engines[0].creep_life_fraction, 0.0);
    }

    #[test]
    fn a_paused_tick_does_nothing() {
        let mut vars = TestVars::default();
        let mut d = Damage::new(&mut vars, None);
        d.update(&mut vars, None, 0.0);
        assert!(d.events.is_empty());
        assert_eq!(d.engines[0].creep_life_fraction, 0.0);
    }

    /// The "engine running" proxy accumulates hours and counts a start
    /// cycle on each false-to-true transition.
    #[test]
    fn engine_hours_and_cycles_accumulate_while_running() {
        let mut vars = TestVars::default();
        let mut d = Damage::new(&mut vars, None);
        vars.write(&d.egt[0], 20.0); // cold
        for n in 1..4 {
            vars.write(&d.egt[n], 20.0);
        }
        d.update(&mut vars, None, 1.0);
        assert_eq!(d.engines[0].cycles, 0);

        vars.write(&d.egt[0], 400.0); // started
        for _ in 0..3600 {
            d.update(&mut vars, None, 1.0);
        }
        assert_eq!(d.engines[0].cycles, 1);
        assert!((d.engines[0].hours - 1.0).abs() < 0.01, "{}", d.engines[0].hours);
    }

    /// Symptom: failure 32123 (overweight landing) activated on a parked
    /// aircraft at start. A cold apron/hangar start is already `on_ground`,
    /// and a heavy long-haul A380 is routinely well above `MLW_KG` while
    /// still parked (normal before an actual departure) -- so any
    /// `on_ground` transition alone must not be read as a landing.
    #[test]
    fn a_start_already_on_the_ground_is_not_a_touchdown() {
        // `was_on_ground` starts `true` (Damage::new), so there is no
        // false-to-true transition at all on a cold ground start.
        assert!(!Damage::is_genuine_touchdown(true, true, 0.0));
    }

    #[test]
    fn a_one_frame_on_ground_blip_is_not_a_touchdown() {
        // `on_ground` flickered false for under a second (spawn/physics
        // settling noise) before reading true again: nowhere near a real
        // flight, so it must not arm anything.
        assert!(!Damage::is_genuine_touchdown(true, false, 0.2));
    }

    #[test]
    fn landing_after_a_real_period_airborne_is_a_touchdown() {
        assert!(Damage::is_genuine_touchdown(true, false, 120.0));
    }

    #[test]
    fn still_airborne_is_never_a_touchdown() {
        assert!(!Damage::is_genuine_touchdown(false, false, 120.0));
        assert!(!Damage::is_genuine_touchdown(false, true, 120.0));
    }

    /// End-to-end: a cold apron start (already on the ground, well above
    /// MLW with a full load) must not arm 32123 just from ticking, the way
    /// the sim log showed it doing.
    #[test]
    fn a_cold_apron_start_does_not_arm_an_overweight_landing() {
        let _guard = crate::failures::tests::SERIAL.lock().unwrap_or_else(|e| e.into_inner());
        crate::failures::replace([]);
        let mut vars = TestVars::default();
        let mut d = Damage::new(&mut vars, None);
        // `xplm` is `None` in this harness, so `on_ground`/`mass_kg` read as
        // `false`/`0`; what matters here is that ticking alone (with no
        // real flight ever recorded) cannot arm 32123.
        for _ in 0..600 {
            d.update(&mut vars, None, 1.0);
        }
        assert!(!crate::failures::active_ids().contains(&32_123));
        crate::failures::replace([]);
    }

    /// The physical chain failures::extra 79_004 ("engine oil leak") is
    /// meant to drive: quantity drains, pressure fraction falls off past
    /// `OIL_STARVATION_QTY_PCT`, and once the engine has run starved past
    /// `OIL_STARVATION_ARM_SECONDS`, the bearings actually wear (72_000) --
    /// the same failure id `sustained_egt_above_mct_arms_bearing_wear_...`
    /// arms from the unrelated overtemperature route, now reachable from a
    /// leak too, exactly the way a real oil-starved bearing fails either
    /// from heat or from running dry.
    #[test]
    fn an_oil_leak_starves_the_bearings_and_arms_bearing_wear() {
        let _guard = crate::failures::tests::SERIAL.lock().unwrap_or_else(|e| e.into_inner());
        let _f = crate::failures::Failures::new();
        crate::failures::replace([]);
        crate::failures::set_active(79_004, true); // engine 1 oil leak
        let mut vars = TestVars::default();
        let mut d = Damage::new(&mut vars, None);
        // Engines "running" but well below the overtemperature route, so
        // only the leak can be what arms 72_000 here.
        for n in 0..4 {
            vars.write(&d.egt[n], 400.0);
        }

        for _ in 0..200 {
            d.update(&mut vars, None, 1.0);
        }
        assert!(d.engines[0].oil_quantity_pct < 50.0, "oil quantity should be draining, was {}", d.engines[0].oil_quantity_pct);
        assert!(!crate::failures::active_ids().contains(&72_000), "not starved long enough yet");

        for _ in 0..200 {
            d.update(&mut vars, None, 1.0);
        }
        assert_eq!(d.engines[0].oil_quantity_pct, 0.0, "sump should be empty by now");
        assert!(crate::failures::active_ids().contains(&72_000), "sustained oil starvation should arm bearing wear");
        // Engine 2 never leaked: its own oil system is unaffected.
        assert!(!crate::failures::active_ids().contains(&72_001));
        crate::failures::replace([]);
    }

    /// A pump fault (79_000) is a direct pressure shortfall, independent of
    /// quantity: the oil is still there.
    #[test]
    fn a_pump_fault_caps_oil_pressure_without_draining_quantity() {
        let _guard = crate::failures::tests::SERIAL.lock().unwrap_or_else(|e| e.into_inner());
        let _f = crate::failures::Failures::new();
        crate::failures::replace([]);
        crate::failures::set_active(79_000, true); // engine 1 oil pump fault
        let mut vars = TestVars::default();
        let mut d = Damage::new(&mut vars, None);
        for n in 0..4 {
            vars.write(&d.egt[n], 400.0);
        }
        d.update(&mut vars, None, 1.0);
        assert_eq!(d.engines[0].oil_quantity_pct, 100.0, "a pump fault does not drain the sump");
        crate::failures::replace([]);
    }

    /// failures::extra 49_004 ("APU oil leak") drains `apu_oil_pct`, which
    /// falls off `apu_oil_pressure_fraction_out` once starved -- the same
    /// leak-then-low-pressure chain as the per-engine one above (79_004).
    #[test]
    fn an_apu_oil_leak_drains_quantity_and_drops_oil_pressure() {
        let _guard = crate::failures::tests::SERIAL.lock().unwrap_or_else(|e| e.into_inner());
        let _f = crate::failures::Failures::new();
        crate::failures::replace([]);
        crate::failures::set_active(49_004, true);
        let mut vars = TestVars::default();
        let mut d = Damage::new(&mut vars, None);

        d.update(&mut vars, None, 1.0);
        assert_eq!(d.apu_oil_pct, 100.0 - OIL_LEAK_DRAIN_PCT_PER_S, "a leak should drain quantity from the first tick");

        for _ in 0..400 {
            d.update(&mut vars, None, 1.0);
        }
        assert_eq!(d.apu_oil_pct, 0.0, "sump should be empty by now");
        assert_eq!(vars.read(&d.apu_oil_pressure_fraction_out), 0.0, "an empty sump must show zero oil pressure");
        crate::failures::replace([]);
    }
}
