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
pub(crate) mod trent900 {
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
/// A380 MLW, kg, for the weight variant this port flies: **WV003**, the one
/// FlyByWire's own `flight_model.cfg` describes (MTOW 510 t via
/// `max_gross_weight` = 1,124,355 lb, MLW 395 t, MZFW 373 t), and therefore
/// the one the converted `.acf` inherits.
///
/// This previously read 386,000 kg, cited as WV000 with "MTOW 510,000 kg" --
/// but Airbus's own weight variant table ("A380 Aircraft Characteristics -
/// Airport and Maintenance Planning", airbus.com/sites/g/files/jlcbta136/
/// files/2021-11/Airbus-Aircraft-AC-A380.pdf) gives WV000 as 560/386/361 and
/// WV003 as 510/395/373. The old pair mixed the two, and the cited MTOW
/// belonged to neither variant named. It also called `max_gross_weight` the
/// MLW, where that field is the *takeoff* weight.
///
/// See `deep::gear_structure`'s own copy, and
/// `docs/sources/a380-reference-documents.md` for the full table.
const MLW_KG: f64 = 395_000.0;

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

/// CS 25.303: "unless otherwise specified, a factor of safety of 1.5 must
/// be applied to the prescribed limit load", which gives the ultimate load
/// the structure must carry without failure. Flap and gear airloads go
/// with dynamic pressure, so with speed squared: ultimate is reached at the
/// limit speed x sqrt(1.5).
const ULTIMATE_FACTOR_OF_SAFETY: f64 = 1.5;
/// `deep::gear_structure` publishes its legs as nose, left wing, right
/// wing, left body, right body (`GEAR_STRUT_COLLAPSED:1..5`); the .acf's own
/// gear order (`_gear/0..4`, X-Plane's `rel_collapse1..5`) is nose, left
/// body, right body, left wing, right wing.
const DEEP_LEG_TO_XPLANE_GEAR: [u64; 5] = [0, 3, 4, 1, 2];

/// How long a flap (VFE) or gear (VLE) speed limit must be exceeded
/// continuously before it counts as an overspeed. A single sample above the
/// limit used to arm the damage failure, so a one-knot gust, or selecting
/// the next flap position a moment before the speed had bled off, did it
/// (2026-09-28). Generic (no published A380 exceedance-recording delay):
/// long enough that turbulence and selection timing never count, short
/// enough that any real overspeed still does.
const OVERSPEED_HOLD_S: f64 = 3.0;

/// Whether a condition has now held continuously for `hold_s`, keeping its
/// running time in `timer` (reset the moment it stops holding).
fn sustained(timer: &mut f64, exceeded: bool, delta: f64, hold_s: f64) -> bool {
    if exceeded {
        *timer += delta;
    } else {
        *timer = 0.0;
    }
    *timer >= hold_s
}

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
    /// Derived, saturating function of `creep_life_fraction` (1.0 once
    /// [`Self::fod_ingested`]), shown on the Study engine pages. Display
    /// only: the engine model's gas path does not read this number; its
    /// damage comes from failures through `engine_commands.rs`'s `EXOTIC`
    /// table.
    pub compressor_efficiency_loss: f64,
    /// This engine ingested a walkaround inlet cover ([`Damage::arm_fod`]):
    /// its HP compressor is destroyed. Persisted, and re-arms 72_024+n
    /// ("FOD damage", the same destruction `EXOTIC` gives 72_012) on every
    /// tick, so the engine stays unserviceable across sessions until the
    /// Study panel's maintenance repair resets this record.
    #[serde(default)]
    pub fod_ingested: bool,
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
    /// Which quarter of creep life has already been reported, so the log
    /// gets one line per step rather than one per tick.
    #[serde(skip)]
    logged_creep_quarter: f64,
    /// `creep_life_fraction` split by the term that charged it, so the log
    /// names the route instead of leaving it to be inferred from the three
    /// dwell timers -- two of which reset the moment the condition lifts,
    /// and so read zero at exactly the tick worth reporting. Persisted with
    /// the fraction they sum to.
    #[serde(default)]
    pub creep_from_overtemp: f64,
    #[serde(default)]
    pub creep_from_mct: f64,
    #[serde(default)]
    pub creep_from_toga: f64,
}

impl EngineWear {
    /// Reject a persisted `creep_life_fraction` this engine's own recorded
    /// history could not have produced, instead of trusting the save file
    /// blindly (`docs/analysis/bug-hunt-2026-09-21.md`'s follow-up: 1.3-1.8
    /// loaded from `Output/preferences/fbw_a380x_airframe.json` and armed
    /// 72_000+n -- "Engine n bearing wear" -- on the very first tick of a
    /// session that had not yet run a single frame of its own physics, in
    /// turn adding a full HPT-design-torque resistive load in
    /// `physics::engine` that a starter sized for ordinary spin-up drag
    /// cannot overcome: the engine never spools, N1/N2 pinned at exactly
    /// 0.0, for a "failure" nothing this session concluded).
    ///
    /// Two independent, already-recorded signals, both anchored on the same
    /// untrimmed EGT this struct's own `update_engines`/`update_engine_limits`
    /// both read:
    /// - Miner's rule (`update_engines`, `trent900::OVERTEMP_LIMIT_S`): a
    ///   full exceedance budget (1.0 of `creep_life_fraction`) needs at
    ///   least `OVERTEMP_LIMIT_S` seconds cumulatively above the untrimmed
    ///   overtemperature limit, and none of that time can predate this
    ///   engine's own recorded `hours` (creep only accrues while the engine
    ///   reads "running"). `creep_life_fraction * OVERTEMP_LIMIT_S >
    ///   hours * 3600.0` is a plain impossibility, not a threshold call.
    /// - For a short-history engine (under one recorded hour -- so nothing
    ///   it did could have been real cruise, only starts), the same
    ///   untrimmed EGT is independently watched by `update_engine_limits`'s
    ///   `ground_start_tgt`/`relight_tgt`/`cold_start` exceedance counters.
    ///   A full exceedance budget of creep with every one of those still at
    ///   zero means the two watchers of the same signal disagree
    ///   completely: not proof by itself, but real, already-recorded
    ///   evidence the value did not come from an overtemperature this
    ///   engine's own limit-tracking would also have caught.
    /// Either signal rejects. The value is clamped to 0.0 rather than
    /// guessed at some smaller nonzero number -- there is no honest way to
    /// recover what it should have been from what was saved -- and the
    /// rejection is returned for the caller to log with the numbers that
    /// triggered it, so *why* it accumulated stays visible instead of
    /// silently vanishing.
    fn plausibility_checked(mut self, engine: usize) -> (Self, Option<String>) {
        if !(self.creep_life_fraction > 0.0) {
            return (self, None);
        }
        let max_from_hours = self.hours * 3600.0 / trent900::OVERTEMP_LIMIT_S;
        let short_history = self.hours < 1.0;
        let no_start_tgt_exceedance =
            self.exceedances.ground_start_tgt == 0 && self.exceedances.relight_tgt == 0 && self.exceedances.cold_start == 0;
        let exceeds_hours_bound = self.creep_life_fraction > max_from_hours;
        let unbacked_by_any_start_exceedance = self.creep_life_fraction >= 1.0 && short_history && no_start_tgt_exceedance;
        if exceeds_hours_bound || unbacked_by_any_start_exceedance {
            let reason = format!(
                "engine {}: persisted creep_life_fraction {:.4} over {:.4} engine-hours ({} cycle(s), 0 ground-start/relight/cold-start TGT exceedances logged) is not physically reachable -- Miner's rule needs >= {:.1} s cumulatively above the {:.0} C untrimmed overtemperature limit for that much creep, and neither this engine's recorded running time nor its own start-TGT exceedance watch backs that; rejected on load, not carried into this session",
                engine + 1,
                self.creep_life_fraction,
                self.hours,
                self.cycles,
                self.creep_life_fraction * trent900::OVERTEMP_LIMIT_S,
                trent900::TGT_OVERTEMP_UNTRIMMED_C,
            );
            self.creep_life_fraction = 0.0;
            self.compressor_efficiency_loss = 0.0;
            return (self, Some(reason));
        }
        (self, None)
    }
}

/// Validate a full persisted `[EngineWear; 4]` on load
/// (`plausibility_checked`'s per-engine rule), returning the (possibly
/// corrected) array and one log line per engine it had to reject.
pub fn load_engines(saved: [EngineWear; 4]) -> ([EngineWear; 4], Vec<String>) {
    let mut rejections = Vec::new();
    let checked = std::array::from_fn(|i| {
        let (wear, rejection) = saved[i].plausibility_checked(i);
        if let Some(reason) = rejection {
            rejections.push(reason);
        }
        wear
    });
    (checked, rejections)
}

impl Default for EngineWear {
    fn default() -> Self {
        Self {
            creep_life_fraction: 0.0,
            compressor_efficiency_loss: 0.0,
            fod_ingested: false,
            hours: 0.0,
            cycles: 0,
            was_running: false,
            oil_quantity_pct: full_oil_quantity(),
            seconds_oil_starved: 0.0,
            seconds_above_mct: 0.0,
            seconds_above_overtemp: 0.0,
            toga_lever_seconds: 0.0,
            logged_creep_quarter: 0.0,
            creep_from_overtemp: 0.0,
            creep_from_mct: 0.0,
            creep_from_toga: 0.0,
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

    /// (W162 follow-up) Used to be this port's own plain value: `deep/apu/
    /// live.rs` published the bare `APU_EGT` name directly, and since
    /// `deep.tick` runs after `simulation.tick`, that plain publish won the
    /// name every frame -- so reading it here as-is, never decoded, was
    /// correct at the time. W162 renamed deep's own publish to
    /// `DEEP_APU_EGT` (the bare name collided with FlyByWire's own
    /// `ElectronicControlBox`, which publishes `APU_EGT` as a packed ARINC
    /// word, `write_arinc429(&self.apu_egt_id, ...)`). Once that rename
    /// lands, this bare name stops being deep's plain value and becomes
    /// FlyByWire's own packed word -- the same encoding `apu_egt_warning`
    /// below already is -- so it must be decoded the identical SSM-gated
    /// way, not read as a raw f64, or it compares as a huge packed float
    /// against a correctly-decoded few-hundred-value `warning` and arms
    /// almost immediately once the APU is running.
    apu_egt: VariableIdentifier,
    apu_egt_warning: VariableIdentifier,
    apu_overtemp_seconds: f64,
    /// How long IAS has been continuously above the flap handle's VFE /
    /// the gear's VLE (see `OVERSPEED_HOLD_S`).
    flap_overspeed_seconds: f64,
    gear_overspeed_seconds: f64,
    /// The same, past ultimate (`ULTIMATE_FACTOR_OF_SAFETY`).
    flap_ultimate_seconds: f64,
    gear_ultimate_seconds: f64,
    /// FlyByWire's own surface positions (percent): only a surface that was
    /// out and carrying the airload is damaged by it.
    flaps_position_pct: VariableIdentifier,
    slats_position_pct: VariableIdentifier,
    /// `deep::gear_structure`'s per-leg strut failure.
    deep_leg_collapsed: [VariableIdentifier; 5],
    deep_leg_force_n: [VariableIdentifier; 5],
    /// This session's structural damage, id -> magnitude, published whole
    /// every tick through `failures::set_damage_levels`.
    structural: std::collections::BTreeMap<u64, f64>,

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

/// The brake energy past which the wing-gear brakes count as worn out: the
/// kinetic energy of a maximum-energy rejected take-off, which is what
/// transport brakes are designed to absorb (CS 25.735's accelerate-stop
/// requirement) -- MTOW (WV003, 510 t, see `MLW_KG`) at a typical heavy-jet
/// decision-speed band (150 kt = 77 m/s). Derived, not a published A380
/// brake figure (docs/physics/failures.md). It used to be MLW at a 140 kt
/// approach speed, which a normal heavy-weight landing stopped mostly on the
/// brakes could reach, arming wear-out on both wing gears (2026-09-28).
const MTOW_KG: f64 = 510_000.0;
const BRAKE_ENERGY_REFERENCE_J: f64 = 0.5 * MTOW_KG * (77.0_f64 * 77.0_f64);
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
            flap_overspeed_seconds: 0.0,
            gear_overspeed_seconds: 0.0,
            flap_ultimate_seconds: 0.0,
            gear_ultimate_seconds: 0.0,
            flaps_position_pct: vars.get("LEFT_FLAPS_POSITION_PERCENT".to_owned()),
            slats_position_pct: vars.get("LEFT_SLATS_POSITION_PERCENT".to_owned()),
            deep_leg_collapsed: std::array::from_fn(|i| vars.get(format!("GEAR_STRUT_COLLAPSED:{}", i + 1))),
            deep_leg_force_n: std::array::from_fn(|i| vars.get(format!("GEAR_LEG_FORCE_N:{}", i + 1))),
            structural: std::collections::BTreeMap::new(),
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
        self.update_flap_gear_speed_overspeeds(vars, xplm, delta);
        self.update_vmo_mmo(xplm);
        self.update_brake_energy(xplm, delta);
        self.update_brake_temperature_tyre_burst(vars);
        self.update_touchdown_exceedances(xplm, delta);
        let ias = Self::get_f(xplm, self.ias);
        let gear_down = Self::get_i(xplm, self.gear_handle_down) != 0;
        self.update_structural(vars, ias, gear_down, delta);
        failures::set_damage_levels(self.structural.clone());
    }

    /// Structural damage once a load passes ultimate (CS 25.303), each
    /// through the model that owns the part, latched for the session:
    /// - flaps/slats extended past their placard speed x sqrt(1.5), held
    ///   for `OVERSPEED_HOLD_S`: the drive is bent and jams where it is
    ///   (27_101/27_102, FlyByWire's own flap/slat assembly);
    /// - gear down past its placard speed x sqrt(1.5), held: the gear doors'
    ///   actuators jam (FlyByWire's own GearActuatorJammed, 32_023-32_025),
    ///   so the gear cannot be raised;
    /// - a strut `deep::gear_structure` failed past its own ultimate load
    ///   (per leg, from its touchdown sink speed and load, the aircraft's
    ///   weight included): X-Plane's own gear collapse for that leg
    ///   (32_130-32_134).
    /// VMO/MMO and tailstrike stay logged exceedances: there is no sourced
    /// design dive speed to put an ultimate on, and a tailstrike is a
    /// contact, not a load this module measures.
    fn update_structural<W: systems::simulation::SimulatorReaderWriter>(&mut self, vars: &mut W, ias: f64, gear_down: bool, delta: f64) {
        let ultimate_speed_factor = ULTIMATE_FACTOR_OF_SAFETY.sqrt();
        let flap_index = vars.read(&self.flaps_handle_index).round().clamp(0.0, 4.0) as usize;
        let flap_limit = VFE_KT[flap_index] * ultimate_speed_factor;
        if sustained(&mut self.flap_ultimate_seconds, ias > flap_limit, delta, OVERSPEED_HOLD_S) {
            if vars.read(&self.flaps_position_pct) > 1.0 {
                self.structural_damage(27_101, &format!("flaps jammed: {ias:.0} kt at CONF {flap_index}, ultimate {flap_limit:.0} kt"));
            }
            if vars.read(&self.slats_position_pct) > 1.0 {
                self.structural_damage(27_102, &format!("slats jammed: {ias:.0} kt at CONF {flap_index}, ultimate {flap_limit:.0} kt"));
            }
        }
        let gear_limit = VLE_VLO_KT * ultimate_speed_factor;
        if sustained(&mut self.gear_ultimate_seconds, gear_down && ias > gear_limit, delta, OVERSPEED_HOLD_S) {
            for (id, door) in [(32_023, "nose"), (32_024, "left"), (32_025, "right")] {
                self.structural_damage(id, &format!("{door} gear doors jammed: {ias:.0} kt gear down, ultimate {gear_limit:.0} kt"));
            }
        }
        for (deep_leg, xplane_gear) in DEEP_LEG_TO_XPLANE_GEAR.into_iter().enumerate() {
            if vars.read(&self.deep_leg_collapsed[deep_leg]) > 0.5 {
                let force = vars.read(&self.deep_leg_force_n[deep_leg]);
                self.structural_damage(
                    32_130 + xplane_gear,
                    &format!("gear leg collapsed (deep leg {}, X-Plane gear {xplane_gear}, leg force {force:.0} N)", deep_leg + 1),
                );
            }
        }
    }

    fn structural_damage(&mut self, id: u64, description: &str) {
        if !self.structural.contains_key(&id) {
            self.structural.insert(id, 1.0);
            self.events.push(format!("structural damage past ultimate: {description} (failure {id})"));
        }
    }

    fn arm(&mut self, id: u64, description: &str) {
        if !failures::active_ids().contains(&id) {
            failures::set_active(id, true);
            self.events.push(format!("exceedance: {description} (failure {id})"));
        }
    }

    /// Foreign-object damage from a walkaround engine-inlet cover a running
    /// engine has ingested (`walkaround.rs`, called the moment N2 crosses
    /// its own rotating threshold with the cover still installed). A whole
    /// fabric cover goes through the fan and down the core: the HP
    /// compressor is destroyed. Arms 72_024+n, which `engine_commands.rs`'s
    /// `EXOTIC` table gives exactly the perturbation of 72_012 ("HP
    /// compressor destruction": compression and flow gone, imbalance on the
    /// bearings), so the engine model itself loses its flame and runs the
    /// core down -- the start fails and the engine is unserviceable. Marks
    /// the engine's persisted [`EngineWear::fod_ingested`], so it stays that
    /// way across sessions until the Study panel's repair. `n` is 0-based
    /// (engine 1 is `n == 0`); out-of-range `n` is ignored.
    pub fn arm_fod(&mut self, n: usize, description: &str) {
        let Some(engine) = self.engines.get_mut(n) else { return };
        engine.fod_ingested = true;
        engine.compressor_efficiency_loss = 1.0;
        self.arm(72_024 + n as u64, description);
    }

    /// Every failure id this module's own [`Damage::arm`] calls can set,
    /// listed once so persistence can tell "this module derived it from
    /// live physics" apart from "the crew armed it from the Study/EFB
    /// panel" (`docs/analysis/bug-hunt-2026-09-21.md`'s follow-up: a
    /// persisted `creep_life_fraction` armed 72_000..72_003 on the very
    /// first tick of a fresh session, before this session's own physics had
    /// run a single time, and there was nothing to tell that apart from a
    /// deliberate arm once it reached disk). Kept as one literal list next
    /// to the call sites it mirrors, so a new `self.arm(id, ...)` added
    /// above without a matching entry here is a one-line diff to catch in
    /// review, not a silent gap.
    pub fn derived_failure_ids() -> Vec<u64> {
        let mut ids = vec![
            49_000, // APU EGT overtemperature damage
            27_100, // flap overspeed
            32_120, // gear overspeed
            34_120, // VMO/MMO overspeed
            32_110, 32_111, // wing gear brake wear-out
            32_121, // hard landing
            32_122, // tailstrike
            32_123, // overweight landing
        ];
        for n in 0..4u64 {
            ids.push(72_000 + n); // bearing wear (creep or oil starvation)
            ids.push(72_004 + n); // compressor stall risk
            ids.push(72_008 + n); // turbine overtemperature damage
        }
        for leg in 0..4u64 {
            ids.push(32_101 + leg); // gear tyre burst (fuse-plug melt)
        }
        for n in 0..4u64 {
            ids.push(72_024 + n); // FOD from an ingested walkaround inlet cover
        }
        ids
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
            let mut creep_report: Option<String> = None;
            let (creep_life_fraction, compressor_efficiency_loss, fod_ingested) = {
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
                    // The TCDS allows an excursion of up to 20 s above this
                    // limit, so one that ends inside it is permitted, not
                    // damage -- the same rule the MCT and TOGA terms below
                    // follow. Every second above it used to charge 1/20 of
                    // the whole budget, cumulatively and persisted, so start
                    // and thrust-spike transients wore an engine out over a
                    // dozen sessions (2026-09-28). Only a single excursion
                    // held past the allowance is a real exceedance: it arms
                    // the turbine over-temperature failure, and its excess
                    // is charged at the same tenth-rate as the others.
                    if e.seconds_above_overtemp > trent900::OVERTEMP_LIMIT_S {
                        overtemp_exceeded = true;
                        e.creep_life_fraction += delta / (trent900::OVERTEMP_LIMIT_S * 10.0);
                        e.creep_from_overtemp += delta / (trent900::OVERTEMP_LIMIT_S * 10.0);
                    }
                } else {
                    e.seconds_above_overtemp = 0.0;
                }

                if t > trent900::TGT_MAX_CONTINUOUS_UNTRIMMED_C {
                    // Above maximum continuous, but that is not damage by
                    // itself: the TCDS *permits* the take-off rating, up to
                    // TGT_TAKEOFF_UNTRIMMED_C, for `takeoff_limit_s` (Note
                    // 5 -- five minutes, ten with an engine out). A normal
                    // take-off spends its whole roll and initial climb here
                    // by design.
                    //
                    // This used to charge `delta / takeoff_limit_s` for
                    // every second above MCT, so five minutes at take-off
                    // thrust consumed a full creep-life budget and armed
                    // "bearing wear" on all four engines -- on the take-off
                    // the engine is certified for. Every flight ended with
                    // four seized engines, and the spent fraction persisted,
                    // so it happened again on the next load.
                    //
                    // Only the time *beyond* the certified allowance is
                    // damage, and it is charged at the same tenth-rate the
                    // lever-position check below uses for overrunning the
                    // same limit.
                    e.seconds_above_mct += delta;
                    if e.seconds_above_mct > takeoff_limit_s {
                        e.creep_life_fraction += delta / (takeoff_limit_s * 10.0);
                        e.creep_from_mct += delta / (takeoff_limit_s * 10.0);
                    }
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
                        e.creep_from_toga += delta / (takeoff_limit_s * 10.0);
                    }
                } else {
                    e.toga_lever_seconds = 0.0;
                }

                // Saturating efficiency loss: up to 15% (an order-of-magnitude
                // ceiling, not a cited figure) as creep life is consumed.
                e.compressor_efficiency_loss = if e.fod_ingested { 1.0 } else { (e.creep_life_fraction * 0.05).min(0.15) };
                // Creep life is what arms bearing wear, and an engine that
                // reaches 1.0 seizes -- so every quarter of it consumed is
                // worth a line saying which term did it and what the turbine
                // was actually doing at the time: the three accrual terms,
                // each with its own running total, beside the measured TGT
                // and the two limits it is compared with.
                let quarter = (e.creep_life_fraction * 4.0).floor();
                if quarter > e.logged_creep_quarter {
                    e.logged_creep_quarter = quarter;
                    creep_report = Some(format!(
                        "engine {} creep life {:.2} = over-temperature {:.2} + above-MCT {:.2} + TOGA-lever {:.2} (TGT {t:.0} C measured against max-continuous {:.0}, over-temperature {:.0}; dwells now {:.0}/{:.0}/{:.0} s)",
                        n + 1,
                        e.creep_life_fraction,
                        e.creep_from_overtemp,
                        e.creep_from_mct,
                        e.creep_from_toga,
                        trent900::TGT_MAX_CONTINUOUS_UNTRIMMED_C,
                        trent900::TGT_OVERTEMP_UNTRIMMED_C,
                        e.seconds_above_mct,
                        e.seconds_above_overtemp,
                        e.toga_lever_seconds,
                    ));
                }
                (e.creep_life_fraction, e.compressor_efficiency_loss, e.fod_ingested)
            };

            if let Some(line) = creep_report {
                self.events.push(line);
            }
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
            // A persisted ingestion re-arms its failure every tick (the id
            // itself is derived, so a reload drops it until this puts it
            // back): the compressor stays destroyed until repaired.
            if fod_ingested {
                self.arm(72_024 + n as u64, &format!("engine {} HP compressor destroyed by an ingested inlet cover", n + 1));
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
        // `A32NX_APU_EGT` and `A32NX_APU_EGT_WARNING` are both names
        // FlyByWire's own source defines as ARINC 429 words
        // (`fbw-common/.../apu/electronic_control_box.rs`
        // `write_arinc429(&self.apu_egt_id, ...)` /
        // `write_arinc429(&self.apu_egt_warning_id, ...)`): the f32 bit
        // pattern in the low 32 bits, a two-bit SSM above it, the whole
        // word stored as that u64's numeric value in an f64
        // (`shared/arinc429.rs` `to_arinc429`/`from_arinc429`).
        //
        // `A32NX_APU_EGT` used NOT to be decoded here (`deep/apu/live.rs`
        // used to publish the bare `APU_EGT` name plain, and since
        // `deep.tick` runs after `simulation.tick`, that plain publish won
        // the name every frame) -- but W162 renamed deep's own publish away
        // to `DEEP_APU_EGT` (the bare name collided with FlyByWire's own
        // packed word), so this bare name is now genuinely FlyByWire's own
        // ARINC word, the same encoding `apu_egt_warning` already is, and
        // must be decoded the identical SSM-gated way (see `apu_egt`'s own
        // doc comment) -- not read as a raw f64, which would compare a huge
        // packed float against a correctly-decoded few-hundred-value
        // `warning` and arm almost immediately once the APU is running.
        //
        // Decoded by *which variable this is*, not by whether a given
        // reading happens to look packed (`prim.rs`'s own `from_simvar`
        // shows any ordinary float decodes to some SSM/value pair with no
        // error, so a value-shape guess can misread a plain number as
        // packed or a packed one as plain without warning -- unlike
        // `study/canvas.rs`'s display-only `looks_packed`, this decision
        // arms a failure and must not guess). Trust a decoded value only
        // when its SSM is normal operation: a word FlyByWire marks failed,
        // without data, or under test must never arm anything.
        const SSM_NORMAL_OPERATION: u64 = 0b11;
        let egt_word = vars.read(&self.apu_egt) as u64;
        let egt_ssm = (egt_word >> 32) & 0b11;
        let egt = (egt_ssm == SSM_NORMAL_OPERATION).then(|| f32::from_bits(egt_word as u32) as f64);
        let warning_word = vars.read(&self.apu_egt_warning) as u64;
        let warning_ssm = (warning_word >> 32) & 0b11;
        let warning = (warning_ssm == SSM_NORMAL_OPERATION).then(|| f32::from_bits(warning_word as u32) as f64);

        if let (Some(egt), Some(warning)) = (egt, warning) {
            if warning > 0.0 && egt > warning {
                self.apu_overtemp_seconds += delta;
                // A generic 60-second sustained-overtemperature trigger (no
                // published APU time limit found): documented as generic.
                // 49_000 is the deep APU's turbine damage (`deep/apu/live.rs`),
                // so the damaged turbine runs hot and its ECB trips it.
                if self.apu_overtemp_seconds > 60.0 {
                    self.arm(49_000, "APU EGT overtemperature damage");
                }
            } else {
                self.apu_overtemp_seconds = 0.0;
            }
        } else {
            // No trustworthy reading/threshold published yet (unwritten,
            // which decodes to SSM 0, or a word FlyByWire itself marks
            // failed/no-data/test): hold the timer at zero rather than arm
            // a damage failure from data nothing vouches for.
            self.apu_overtemp_seconds = 0.0;
        }

        // 49_004 "APU oil leak" is not modelled here: it is the deep APU's
        // own oil leak (`deep/apu/live.rs`), whose ECB trips on low oil
        // pressure and hands the shutdown to FlyByWire's ECB (`APU_ECB_TRIP`).
        // The separate sump this used to drain published a pressure nothing
        // read.
    }

    fn update_flap_gear_speed_overspeeds<W: systems::simulation::SimulatorReaderWriter>(&mut self, vars: &mut W, xplm: Option<&Xplm>, delta: f64) {
        let ias = Self::get_f(xplm, self.ias);
        let flap_index = vars.read(&self.flaps_handle_index).round().clamp(0.0, 4.0) as usize;
        if sustained(&mut self.flap_overspeed_seconds, ias > VFE_KT[flap_index], delta, OVERSPEED_HOLD_S) {
            self.arm(27_100, &format!("flap overspeed at CONF {flap_index} ({ias:.0} kt)"));
        }
        let gear_down = Self::get_i(xplm, self.gear_handle_down) != 0;
        if sustained(&mut self.gear_overspeed_seconds, gear_down && ias > VLE_VLO_KT, delta, OVERSPEED_HOLD_S) {
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
                // The failures this module derived for this engine go with
                // the wear that armed them: resetting the record alone left
                // them armed, still perturbing the engine's gas path, for
                // the rest of the session.
                for base in [72_000u64, 72_004, 72_008, 72_024] {
                    failures::set_active(base + n as u64, false);
                }
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
    fod_ingested: false,
    hours: 0.0,
    cycles: 0,
    was_running: false,
    oil_quantity_pct: 100.0,
    seconds_oil_starved: 0.0,
    seconds_above_mct: 0.0,
    seconds_above_overtemp: 0.0,
    toga_lever_seconds: 0.0,
    logged_creep_quarter: 0.0,
    creep_from_overtemp: 0.0,
    creep_from_mct: 0.0,
    creep_from_toga: 0.0,
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

/// The engine (1-based) whose own damage record armed failure `id`, for the
/// failures [`Damage::apply_requests`] clears with the record: creep,
/// oil-starvation and overtemperature wear (72_000/72_004/72_008 + n) and
/// an ingested inlet cover (72_024 + n). Repairing one of these as a failure
/// has to repair the engine too, or the record re-arms it the next tick.
pub fn engine_of_damage_failure(id: u64) -> Option<usize> {
    [72_000u64, 72_004, 72_008, 72_024]
        .iter()
        .find(|&&base| (base..base + 4).contains(&id))
        .map(|&base| (id - base) as usize + 1)
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

    /// A gust or an early flap selection -- a couple of seconds over the
    /// limit -- is not an overspeed; three seconds continuously over is.
    #[test]
    fn a_speed_limit_counts_only_once_exceeded_continuously_for_the_hold_time() {
        let mut t = 0.0;
        assert!(!sustained(&mut t, true, 1.0, OVERSPEED_HOLD_S));
        assert!(!sustained(&mut t, true, 1.0, OVERSPEED_HOLD_S), "2 s over is a gust");
        assert!(!sustained(&mut t, false, 1.0, OVERSPEED_HOLD_S), "back under: the timer restarts");
        assert!(!sustained(&mut t, true, 1.0, OVERSPEED_HOLD_S));
        assert!(!sustained(&mut t, true, 1.0, OVERSPEED_HOLD_S));
        assert!(sustained(&mut t, true, 1.0, OVERSPEED_HOLD_S), "3 s continuously over is an overspeed");
    }

    /// The brakes are designed for a maximum-energy rejected take-off, so a
    /// normal heavy landing stopped entirely on the brakes is well inside
    /// that, and an RTO at MTOW from above the reference band is past it.
    #[test]
    fn a_heavy_landing_is_inside_the_brake_energy_reference_and_a_max_rto_is_not() {
        let heavy_landing = 0.5 * MLW_KG * (72.0_f64 * 72.0_f64); // MLW, 140 kt, all on the brakes
        assert!(heavy_landing < BRAKE_ENERGY_REFERENCE_J * 0.75, "a heavy landing must leave the brakes real margin");
        let max_rto = 0.5 * MTOW_KG * (82.0_f64 * 82.0_f64); // MTOW, 160 kt
        assert!(max_rto > BRAKE_ENERGY_REFERENCE_J, "an RTO at MTOW from 160 kt must still count");
    }

    /// A TGT excursion above the over-temperature limit that ends inside
    /// the TCDS's own 20-second allowance is permitted: no creep, no
    /// failure. Every such second used to charge 1/20 of the whole budget,
    /// cumulatively and persisted, so start and thrust-spike transients wore
    /// an engine out over a dozen sessions (2026-09-28: engine 2 at 0.08
    /// after 2.7 hours).
    #[test]
    fn an_overtemp_inside_the_20s_allowance_costs_nothing() {
        let _guard = crate::failures::tests::serial();
        let _f = crate::failures::Failures::new();
        crate::failures::replace([]);
        let mut vars = TestVars::default();
        let mut d = Damage::new(&mut vars, None);
        for n in 0..4 {
            vars.write(&d.egt[n], 970.0);
        }
        for _ in 0..15 {
            d.update(&mut vars, None, 1.0);
        }
        assert_eq!(d.engines[0].creep_life_fraction, 0.0, "15 s inside the 20 s allowance is certified operation");
        assert!(!crate::failures::active_ids().contains(&72_008));
    }

    /// `arm_fod` (`walkaround.rs`'s engine-inlet-cover-ingestion route)
    /// arms the per-engine FOD id and gives the engine a real, causal
    /// efficiency loss through the same channel a creep-derived exceedance
    /// already drives -- not a scripted message with no physical effect.
    #[test]
    fn arm_fod_damages_the_named_engine_and_arms_its_own_id_only() {
        let _guard = crate::failures::tests::serial();
        let _f = crate::failures::Failures::new();
        crate::failures::replace([]);
        let mut vars = TestVars::default();
        let mut d = Damage::new(&mut vars, None);
        assert_eq!(d.engines[1].compressor_efficiency_loss, 0.0);
        d.arm_fod(1, "engine 2 FOD from an ingested walkaround inlet cover");
        assert!(d.engines[1].compressor_efficiency_loss > 0.0, "ingestion must cost real compressor efficiency");
        assert!(crate::failures::active_ids().contains(&72_025), "engine 2's own FOD id (72_024+1) is armed");
        assert!(!crate::failures::active_ids().contains(&72_024), "engine 1's FOD id must not be touched");
        assert_eq!(d.engines[0].compressor_efficiency_loss, 0.0);
        crate::failures::replace([]);
    }

    fn set(vars: &mut TestVars, name: &str, value: f64) {
        let id = systems::simulation::VariableRegistry::get(vars, name.to_owned());
        systems::simulation::SimulatorReaderWriter::write(vars, &id, value);
    }

    /// CONF 3's VFE is 196 kt, so its ultimate is 196 x sqrt(1.5) = 240 kt.
    /// Below it nothing breaks however long it is held, a brief excursion
    /// past it (a gust) is inside the hold, and a held one jams only the
    /// surfaces that were out and carrying the load.
    #[test]
    fn past_ultimate_held_extended_flaps_jam_but_retracted_slats_do_not() {
        let _guard = crate::failures::tests::serial();
        let _f = crate::failures::Failures::new();
        crate::failures::replace([]);
        let mut vars = TestVars::default();
        let mut d = Damage::new(&mut vars, None);
        set(&mut vars, "FLAPS_HANDLE_INDEX", 3.0);
        set(&mut vars, "LEFT_FLAPS_POSITION_PERCENT", 60.0);
        set(&mut vars, "LEFT_SLATS_POSITION_PERCENT", 0.0);
        for _ in 0..60 {
            d.update_structural(&mut vars, 239.0, false, 1.0);
        }
        assert!(d.structural.is_empty(), "under ultimate: {:?}", d.structural);
        d.update_structural(&mut vars, 250.0, false, 1.0);
        d.update_structural(&mut vars, 200.0, false, 1.0);
        assert!(d.structural.is_empty(), "a one-second excursion is inside the hold");
        for _ in 0..5 {
            d.update_structural(&mut vars, 250.0, false, 1.0);
        }
        assert!(d.structural.contains_key(&27_101), "extended flaps past ultimate, held, must jam");
        assert!(!d.structural.contains_key(&27_102), "retracted slats carried no load");
        crate::failures::replace([]);
    }

    /// VLE 250 kt, ultimate 250 x sqrt(1.5) = 306 kt: the doors jam, and only
    /// with the gear down.
    #[test]
    fn a_gear_overspeed_past_ultimate_jams_the_gear_doors() {
        let _guard = crate::failures::tests::serial();
        let _f = crate::failures::Failures::new();
        crate::failures::replace([]);
        let mut vars = TestVars::default();
        let mut d = Damage::new(&mut vars, None);
        for _ in 0..10 {
            d.update_structural(&mut vars, 320.0, false, 1.0);
        }
        assert!(d.structural.is_empty(), "gear up: nothing out in the airflow");
        for _ in 0..10 {
            d.update_structural(&mut vars, 300.0, true, 1.0);
        }
        assert!(d.structural.is_empty(), "past VLE but under ultimate");
        for _ in 0..5 {
            d.update_structural(&mut vars, 320.0, true, 1.0);
        }
        for id in [32_023, 32_024, 32_025] {
            assert!(d.structural.contains_key(&id), "{id}: the door actuators must jam");
        }
        crate::failures::replace([]);
    }

    /// `deep::gear_structure`'s leg 2 is the left wing leg; X-Plane's is gear
    /// 3 (`_gear/3`, `rel_collapse4`): the leg the deep model broke is the
    /// one that collapses, and the damage reaches the failure set.
    #[test]
    fn a_leg_the_deep_gear_model_breaks_collapses_that_same_leg() {
        let _guard = crate::failures::tests::serial();
        let _f = crate::failures::Failures::new();
        crate::failures::replace([]);
        let mut vars = TestVars::default();
        let mut d = Damage::new(&mut vars, None);
        set(&mut vars, "GEAR_STRUT_COLLAPSED:2", 1.0);
        d.update(&mut vars, None, 1.0);
        assert_eq!(d.structural.keys().copied().collect::<Vec<_>>(), vec![32_133]);
        assert!(crate::failures::is_active(32_133), "published through the damage channel");
        assert!(!crate::failures::armed_ids().contains(&32_133), "never saved as crew-armed");
        crate::failures::replace([]);
        crate::failures::set_damage_levels(std::collections::BTreeMap::new());
    }

    /// An ingested inlet cover destroys the compressor for good: its
    /// failure stays armed tick after tick, comes back after a reload (the
    /// id itself is derived and dropped on save; the persisted record
    /// re-arms it), and only the maintenance repair clears it.
    #[test]
    fn an_ingested_cover_destroys_the_compressor_until_repaired() {
        let _guard = crate::failures::tests::serial();
        let _f = crate::failures::Failures::new();
        crate::failures::replace([]);
        let mut vars = TestVars::default();
        let mut d = Damage::new(&mut vars, None);
        d.arm_fod(2, "engine 3 FOD from an ingested walkaround inlet cover");
        for _ in 0..10 {
            d.update(&mut vars, None, 1.0);
        }
        assert!(crate::failures::is_active(72_026), "the destruction must stay armed");
        assert!(d.engines[2].fod_ingested);
        assert_eq!(d.engines[2].compressor_efficiency_loss, 1.0);
        assert!(!d.engines[1].fod_ingested && !crate::failures::is_active(72_025));

        // A new session: the save keeps the record, not the derived id.
        let saved: [EngineWear; 4] = serde_json::from_str(&serde_json::to_string(&d.engines).unwrap()).unwrap();
        crate::failures::replace([]);
        let mut d = Damage::new(&mut vars, None);
        d.engines = load_engines(saved).0;
        d.update(&mut vars, None, 1.0);
        assert!(crate::failures::is_active(72_026), "a reload must not repair the engine");

        request_repair_engine(3); // 1-based: engine 3 is index 2
        let _ = d.apply_requests();
        assert!(!d.engines[2].fod_ingested);
        assert!(!crate::failures::is_active(72_026), "the repair must clear the failure it armed");
        d.update(&mut vars, None, 1.0);
        assert!(!crate::failures::is_active(72_026), "and it must not come back");
        crate::failures::replace([]);
    }

    /// The Study page's "Repair" on an engine's own damage failure (a
    /// destroyed compressor from an ingested cover) goes through
    /// `web::apply_action`'s `repairFailure`, which only knows the failure
    /// id: it must reach the engine's record, or the record re-arms the
    /// failure the next tick and the repair never sticks.
    #[test]
    fn repairing_an_engine_damage_failure_from_the_study_page_repairs_the_engine() {
        assert_eq!(engine_of_damage_failure(72_024), Some(1));
        assert_eq!(engine_of_damage_failure(72_027), Some(4));
        assert_eq!(engine_of_damage_failure(72_005), Some(2));
        assert_eq!(engine_of_damage_failure(72_012), None, "not a record-armed id");
        assert_eq!(engine_of_damage_failure(72_028), None);

        let _guard = crate::failures::tests::serial();
        let _f = crate::failures::Failures::new();
        crate::failures::replace([]);
        let mut vars = TestVars::default();
        let mut d = Damage::new(&mut vars, None);
        d.arm_fod(0, "engine 1 FOD from an ingested walkaround inlet cover");
        d.update(&mut vars, None, 1.0);
        assert!(crate::failures::is_active(72_024));

        crate::study::web::apply_action(r#"{"kind":"repairFailure","id":72024}"#).unwrap();
        let _ = d.apply_requests();
        for _ in 0..5 {
            d.update(&mut vars, None, 1.0);
        }
        assert!(!d.engines[0].fod_ingested, "the engine is repaired, not just the failure");
        assert!(!crate::failures::is_active(72_024), "and the failure stays cleared");
        crate::failures::replace([]);
    }

    /// Hundreds of brief spikes -- many sessions of starts and thrust
    /// transients -- never add up to wear: each one is inside the
    /// allowance on its own.
    #[test]
    fn repeated_brief_overtemp_spikes_never_wear_the_engine() {
        let _guard = crate::failures::tests::serial();
        let _f = crate::failures::Failures::new();
        crate::failures::replace([]);
        let mut vars = TestVars::default();
        let mut d = Damage::new(&mut vars, None);
        for _ in 0..500 {
            vars.write(&d.egt[0], 970.0);
            d.update(&mut vars, None, 2.0);
            vars.write(&d.egt[0], 700.0);
            d.update(&mut vars, None, 10.0);
        }
        assert_eq!(d.engines[0].creep_life_fraction, 0.0);
        assert!(!crate::failures::active_ids().contains(&72_000), "no bearing wear from transients");
    }

    /// Held past the allowance, it is a real exceedance: turbine
    /// over-temperature damage arms, and only the excess costs creep, at
    /// the same tenth-rate the take-off and TOGA limits charge.
    #[test]
    fn an_overtemp_held_past_the_allowance_charges_only_the_excess() {
        let _guard = crate::failures::tests::serial();
        let _f = crate::failures::Failures::new();
        crate::failures::replace([]);
        let mut vars = TestVars::default();
        let mut d = Damage::new(&mut vars, None);
        vars.write(&d.egt[0], 970.0);
        for _ in 0..40 {
            d.update(&mut vars, None, 1.0);
        }
        let expected = 20.0 / (trent900::OVERTEMP_LIMIT_S * 10.0);
        assert!((d.engines[0].creep_life_fraction - expected).abs() < 1e-9, "20 s over the allowance: {}", d.engines[0].creep_life_fraction);
        assert!(crate::failures::active_ids().contains(&72_008), "a 40 s overtemp is a real exceedance");
        crate::failures::replace([]);
    }

    /// Sustained EGT above the Trent 900's 850 C maximum-continuous TGT,
    /// with all four engines "running" (so the 5-minute all-engines limit
    /// applies, not the 10-minute OEI one — EASA.E.012 §IV.1.2 Note 5),
    /// accumulates creep life and arms the bearing-wear failure once the
    /// takeoff time limit is exceeded.
    #[test]
    fn sustained_egt_above_mct_arms_bearing_wear_after_the_takeoff_time_limit() {
        let _guard = crate::failures::tests::serial();
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

        // Inside the certified take-off allowance the engine is doing
        // exactly what the TCDS permits, so it must cost *nothing*. This is
        // the assertion that matters: charging creep here consumed a full
        // engine life every normal take-off and seized all four engines.
        for _ in 0..299 {
            d.update(&mut vars, None, 1.0);
        }
        assert_eq!(
            d.engines[0].creep_life_fraction, 0.0,
            "the certified 5-minute take-off rating must not consume creep life"
        );
        assert!(!crate::failures::active_ids().contains(&72_000));

        // Past the limit it starts to count, but slowly -- a few seconds
        // over is a logbook entry, not a written-off engine.
        for _ in 0..60 {
            d.update(&mut vars, None, 1.0);
        }
        let after_a_minute_over = d.engines[0].creep_life_fraction;
        assert!(after_a_minute_over > 0.0, "past the limit it must start to accrue");
        assert!(after_a_minute_over < 0.05, "a minute over must not be a fifth of the engine: {after_a_minute_over}");
        assert!(!crate::failures::active_ids().contains(&72_000), "still nowhere near a failure");

        // Held far past it, it does eventually arm.
        for _ in 0..3_000 {
            d.update(&mut vars, None, 1.0);
        }
        assert!(crate::failures::active_ids().contains(&72_000), "sustained far past the limit must arm 72000");
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
        let _guard = crate::failures::tests::serial();
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

    /// The live defect this test pins: `Output/preferences/fbw_a380x_
    /// airframe.json` loaded with `creep_life_fraction` 1.3-1.8 across all
    /// four engines, accumulated over just 0.0685 recorded engine-hours and
    /// 3 cycles with zero ground-start/relight/cold-start TGT exceedances
    /// ever logged -- and `damage.rs` armed 72_000..72_003 ("Engine n
    /// bearing wear") from it on the very first tick, before this session
    /// ran a single frame of its own physics. `EngineWear::
    /// plausibility_checked` must reject that value on load rather than
    /// trust it.
    #[test]
    fn load_engines_rejects_creep_the_recorded_hours_cannot_support() {
        let implausible = EngineWear { creep_life_fraction: 1.8147173309326168, hours: 0.068539646572537, cycles: 3, ..EngineWear::default() };
        let saved = [implausible, EngineWear::default(), EngineWear::default(), EngineWear::default()];
        let (checked, rejections) = load_engines(saved);
        assert_eq!(checked[0].creep_life_fraction, 0.0, "the implausible value must not survive load");
        assert_eq!(checked[0].compressor_efficiency_loss, 0.0);
        assert_eq!(checked[0].hours, 0.068539646572537, "only the implausible field is corrected, not the whole record");
        assert_eq!(checked[0].cycles, 3);
        assert_eq!(rejections.len(), 1, "exactly engine 1 must be rejected");
        assert!(rejections[0].contains("engine 1"), "{}", rejections[0]);
        for e in &checked[1..] {
            assert_eq!(e.creep_life_fraction, 0.0);
        }
    }

    /// A real, physically-reachable creep value must survive load
    /// unchanged: a long flight (5 recorded engine-hours) whose creep is
    /// well under what `OVERTEMP_LIMIT_S`-based Miner's rule allows for
    /// that many hours is not rejected just for being nonzero.
    #[test]
    fn load_engines_keeps_creep_the_recorded_hours_can_support() {
        let plausible = EngineWear { creep_life_fraction: 0.3, hours: 5.0, cycles: 4, ..EngineWear::default() };
        let saved = [plausible, EngineWear::default(), EngineWear::default(), EngineWear::default()];
        let (checked, rejections) = load_engines(saved);
        assert_eq!(checked[0].creep_life_fraction, 0.3);
        assert!(rejections.is_empty(), "{rejections:?}");
    }

    /// A short-history engine (under a recorded hour, so nothing it did
    /// could have been real cruise) that logged a real ground-start TGT
    /// exceedance is not rejected just because its history is short: the
    /// second signal (the independent exceedance watch) backs the creep
    /// this time.
    #[test]
    fn load_engines_keeps_short_history_creep_backed_by_a_logged_exceedance() {
        let mut exceedances = NO_EXCEEDANCES;
        exceedances.ground_start_tgt = 1;
        let backed = EngineWear { creep_life_fraction: 1.0, hours: 0.05, cycles: 1, exceedances, ..EngineWear::default() };
        let saved = [backed, EngineWear::default(), EngineWear::default(), EngineWear::default()];
        let (checked, rejections) = load_engines(saved);
        assert_eq!(checked[0].creep_life_fraction, 1.0, "logged exceedances back this a genuine hot start");
        assert!(rejections.is_empty(), "{rejections:?}");
    }

    #[test]
    fn egt_at_or_below_max_continuous_never_accumulates_creep() {
        // `failures::STATE` is process-wide and this test reaches it (directly, or
        // through model code such as `Breakers::pre_systems` / `Damage::arm`).
        let _serial = crate::failures::tests::serial();
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
        // `failures::STATE` is process-wide and this test reaches it (directly, or
        // through model code such as `Breakers::pre_systems` / `Damage::arm`).
        let _serial = crate::failures::tests::serial();
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
        let _guard = crate::failures::tests::serial();
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
        let _guard = crate::failures::tests::serial();
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
        let _guard = crate::failures::tests::serial();
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

    /// (W162 follow-up) A helper packing an ARINC word the same way
    /// FlyByWire's own `to_arinc429` does: f32 bits low, a two-bit SSM
    /// above, stored as that u64's numeric value in an f64 -- used to drive
    /// both `apu_egt` and `apu_egt_warning`, now that both are FlyByWire's
    /// own packed words (see `apu_egt`'s own doc comment).
    fn pack_arinc(ssm: u64, data: f32) -> f64 {
        ((ssm << 32) | data.to_bits() as u64) as f64
    }

    /// `A32NX_APU_EGT`/`A32NX_APU_EGT_WARNING` as FlyByWire's own
    /// `ElectronicControlBox` actually sends both, since W162's rename: an
    /// ARINC 429 word, the f32 bits low, the two-bit SSM above
    /// (`shared/arinc429.rs` `to_arinc429`). A real APU EGT past the packed
    /// threshold, with both words marked normal operation, must arm the
    /// overtemperature failure after the 60 s window -- the case the raw
    /// (undecoded) comparison always missed, since a plain reading of ~900
    /// can never exceed a raw packed word in the billions.
    #[test]
    fn a_packed_apu_egt_warning_word_arms_overtemp_once_the_real_egt_exceeds_it() {
        let _guard = crate::failures::tests::serial();
        let _f = crate::failures::Failures::new();
        crate::failures::replace([]);
        let mut vars = TestVars::default();
        let mut d = Damage::new(&mut vars, None);
        vars.write(&d.apu_egt, pack_arinc(0b11, 950.0)); // 950.0 C packed, SSM 3 (normal operation)
        vars.write(&d.apu_egt_warning, pack_arinc(0b11, 900.0)); // 900.0 C packed, SSM 3 (normal operation)

        for _ in 0..60 {
            d.update(&mut vars, None, 1.0);
        }
        assert!(!crate::failures::active_ids().contains(&49_000), "not yet past the 60 s window");

        d.update(&mut vars, None, 1.0);
        assert!(crate::failures::active_ids().contains(&49_000), "950 C real EGT past a decoded 900 C threshold must arm");
        crate::failures::replace([]);
    }

    /// A word FlyByWire itself marks failed (SSM 0) -- the sender does not
    /// vouch for it (the ECB is off), so it must never arm the
    /// overtemperature failure no matter how hot the packed EGT reads.
    #[test]
    fn a_packed_apu_egt_warning_word_marked_failure_never_arms() {
        let _guard = crate::failures::tests::serial();
        let _f = crate::failures::Failures::new();
        crate::failures::replace([]);
        let mut vars = TestVars::default();
        let mut d = Damage::new(&mut vars, None);
        vars.write(&d.apu_egt, pack_arinc(0b11, 950.0));
        vars.write(&d.apu_egt_warning, pack_arinc(0b00, 900.0)); // 900.0 C packed, SSM 0 (failure warning)

        for _ in 0..120 {
            d.update(&mut vars, None, 1.0);
        }
        assert!(!crate::failures::active_ids().contains(&49_000), "a word the ECB marks failed must never arm the failure");
        crate::failures::replace([]);
    }

    /// The ordinary case today: `APU_EGT_WARNING` unwritten (0.0, nothing
    /// in the live plugin publishes it) decodes to SSM 0 and must not arm
    /// even a high real EGT.
    #[test]
    fn an_unwritten_apu_egt_warning_never_arms() {
        let _guard = crate::failures::tests::serial();
        let _f = crate::failures::Failures::new();
        crate::failures::replace([]);
        let mut vars = TestVars::default();
        let mut d = Damage::new(&mut vars, None);
        vars.write(&d.apu_egt, pack_arinc(0b11, 950.0));

        for _ in 0..120 {
            d.update(&mut vars, None, 1.0);
        }
        assert!(!crate::failures::active_ids().contains(&49_000));
        crate::failures::replace([]);
    }

    /// (W162 follow-up) `A32NX_APU_EGT` unwritten (0.0, decodes to SSM 0)
    /// must not arm even against a trustworthy, high packed warning
    /// threshold -- an untrusted/absent real reading is not "hotter than
    /// the threshold", it is simply not known.
    #[test]
    fn an_unwritten_apu_egt_never_arms_against_a_trustworthy_warning() {
        let _guard = crate::failures::tests::serial();
        let _f = crate::failures::Failures::new();
        crate::failures::replace([]);
        let mut vars = TestVars::default();
        let mut d = Damage::new(&mut vars, None);
        vars.write(&d.apu_egt_warning, pack_arinc(0b11, 900.0));

        for _ in 0..120 {
            d.update(&mut vars, None, 1.0);
        }
        assert!(!crate::failures::active_ids().contains(&49_000));
        crate::failures::replace([]);
    }
}
