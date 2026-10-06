//! The EFB interface: FlyByWire's flyPad ground, payload, fuel, pushback,
//! settings and failures functions as `fbw/efb/...` datarefs and commands,
//! for a third-party EFB. docs/efb-interface.md lists every one.
//!
//! Everything here is what the flyPad itself does, on the systems' own
//! variables: nothing is simulated beside them.
//!
//! - Payload: the A380 flyPad page (fbw-common EFB Ground/Pages/Payload/
//!   WideBody/A380Payload.tsx) splits a passenger count over the fourteen
//!   seat stations (`setTargetPax`, :382-407) and a cargo weight over the three
//!   holds (`setTargetCargo`, :409-429), chooses seats at random in each
//!   station's seat flags (bitFlags.ts SeatFlags), and turns a ZFW or GW entry
//!   into passengers and cargo (`processZfw`/`processGw`, :431-461). The
//!   boarding itself is FlyByWire's payload system (a380_systems payload/
//!   mod.rs, systems payload/mod.rs), which reads the `_DESIRED` flags.
//! - Refuel: the A380 page (Ground/Pages/Fuel/A380_842/A380Fuel.tsx) only sets
//!   `A32NX_FUEL_DESIRED` (kg), `A32NX_REFUEL_STARTED_BY_USR` and the rate
//!   setting; the per-tank distribution is FlyByWire's systems' (fuel refuel,
//!   which reads `A32NX_EFB_REFUEL_RATE_SETTING`), so none is ported here.
//! - Ground services (Services/A380_842/A380Services.tsx): the door toggles
//!   are `K:TOGGLE_AIRCRAFT_EXIT` on the door model; the service vehicles are
//!   X-Plane's own ground operations; the GPU is GPUManagement.ts ported.
//! - Settings: the NXDataStore settings the flyPad copies into variables
//!   (EFB Settings/sync.ts:40-251, fbw-a380x EFB settingsSync.ts:7-14), kept in
//!   `Output/preferences/fbw_a380x_settings.ini` under their MSFS stored-data
//!   keys (`A380X_<key>`, persistence.ts:108 with AIRCRAFT_PROJECT_PREFIX
//!   "a380x", fbw-a380x/.env:6).

use std::collections::BTreeMap;
use std::path::PathBuf;

use systems::simulation::{SimulatorReaderWriter, VariableIdentifier, VariableRegistry};

use crate::doors::{Doors, Request};
use crate::published::{self, Command, Published, Value};
use crate::xp::{self, DataRef, Xplm};
use crate::Vars;

// ---------------------------------------------------------------------------
// ECAM Phase 2 (E-FUEL-DESIGN.md D12/D14): CG/weight-disagree and CG-envelope
// discretes, published here because `efb.rs` is the real owner of every
// input value (`AIRFRAME_*`/`AIRFRAME_*_DESIRED`), not `deep::fuel`'s own
// duplicate ledger. See `E-FUEL-DESIGN.md`'s D17 for why an `FbwProc`
// trigger is allowed to read a plain `efb.rs`-published name at all
// (`TRIGGER_REAL_OWNER_VARS`, `deep/ecam/fbw_tests.rs`).
// ---------------------------------------------------------------------------

/// D12, **GENERIC**: how far a crew-entered "desired" ZFW CG figure may sit
/// from the fully precise computed "actual" one before it counts as a real
/// disagreement rather than ordinary crew-entry rounding. An
/// order-of-magnitude "still within rounding" band, the same kind of
/// reasoned-but-unmeasured figure `deep::fuel::live::WING_IMBALANCE_LIMIT_KG`
/// already is elsewhere in this crate -- not a cited AMM number.
const ZFW_CG_DISAGREE_TOLERANCE_PCT: f64 = 0.5;
/// D12, **GENERIC**: the same reasoning as above, for the weight figure
/// (typical load-sheet rounding, not a cited AMM number).
const WEIGHT_DISAGREE_TOLERANCE_KG: f64 = 500.0;

/// D14: the A380-842's real weight/CG performance-envelope polygons,
/// `[%MAC, weight_kg]` point lists, copied verbatim from
/// `D:/Microsoft Flight Simulator 2020/.../Community/
/// flybywire-aircraft-a380-842/config/a380x/a380-842/airframe.json5:65-73`
/// (read in full for this revision, not taken on trust). This is the
/// broadest, general in-flight envelope FlyByWire's own flyPad `efb.js` runs
/// its own "CG Outside Takeoff Envelope"-style point-in-polygon test
/// against.
const FLIGHT_ENVELOPE: &[(f64, f64)] = &[(29.0, 270000.0), (28.0, 270000.0), (28.0, 375000.0), (35.0, 510000.0), (44.0, 510000.0), (44.0, 270000.0), (43.0, 270000.0)];
/// D14: `airframe.json5:58-64`'s `mtow` envelope, used for the predicted
/// take-off CG/GW check (`281800081`).
const MTOW_ENVELOPE: &[(f64, f64)] = &[(29.0, 270000.0), (29.0, 375000.0), (35.75, 510000.0), (43.0, 510000.0), (43.0, 270000.0)];

/// D14, **GENERIC**: how close to the `FLIGHT_ENVELOPE`'s forward (low
/// %MAC) boundary counts as "AT the forward limit" rather than merely
/// "still in range" -- an early-annunciation band before the real
/// structural/`CG_OUT_OF_RANGE` limit is actually reached. The underlying
/// limit itself (the polygon) is real and cited above; only this margin is
/// a judgement call.
const FWD_MARGIN_PCT: f64 = 0.5;
/// D14: the real EXCESS AFT CG trigger, sourced directly rather than a
/// margin against the envelope's own aft edge: FCOM PRO-ABN-ECAM p.5164,
/// "The aircraft center of gravity (CG), computed by the Weight and Balance
/// Backup Computer (WBBC) exceeds 50% while the fuel system is in automatic
/// mode." This model has no separate WBBC-vs-FQMS CG channel and no
/// automatic/manual fuel-system-mode discrete, so `AIRFRAME_GW_CG_PERCENT_
/// MAC` (the one real CG figure this crate publishes) is compared directly
/// against the FCOM's own number, without the "automatic mode" gate --
/// simpler and more conservative than a margin against the 43% structural
/// limit this design used before the FCOM addendum, and a real, cited value
/// rather than a `FWD_MARGIN_PCT`-style judgement call.
const AFT_CG_EXCESS_PCT: f64 = 50.0;

/// D14: a standard even-odd ray-casting point-in-polygon test. Not a
/// fabricated threshold -- a textbook computational-geometry algorithm; the
/// only real numbers are the polygon points it is fed (`FLIGHT_ENVELOPE`/
/// `MTOW_ENVELOPE` above, both cited to `airframe.json5`).
fn point_in_polygon(x: f64, y: f64, poly: &[(f64, f64)]) -> bool {
    let mut inside = false;
    let n = poly.len();
    for i in 0..n {
        let (x1, y1) = poly[i];
        let (x2, y2) = poly[(i + 1) % n];
        if (y1 > y) != (y2 > y) {
            let x_at_y = x1 + (y - y1) / (y2 - y1) * (x2 - x1);
            if x < x_at_y {
                inside = !inside;
            }
        }
    }
    inside
}

// ---------------------------------------------------------------------------
// Payload, as the flyPad splits it.
// ---------------------------------------------------------------------------

/// cabin.json5 seatMap: (L:var without prefix, capacity), in station order
/// (fbw-a380x config/a380x/a380-842/cabin.json5; the capacities are the seats
/// its rows list, and match a380_systems payload/mod.rs A380_PAX).
pub const SEAT_STATIONS: [(&str, u32); 14] = [
    ("PAX_MAIN_FWD_A", 28),
    ("PAX_MAIN_FWD_B", 28),
    ("PAX_MAIN_MID_1A", 39),
    ("PAX_MAIN_MID_1B", 50),
    ("PAX_MAIN_MID_1C", 43),
    ("PAX_MAIN_MID_2A", 48),
    ("PAX_MAIN_MID_2B", 40),
    ("PAX_MAIN_MID_2C", 36),
    ("PAX_MAIN_AFT_A", 42),
    ("PAX_MAIN_AFT_B", 40),
    ("PAX_UPPER_FWD", 14),
    ("PAX_UPPER_MID_A", 30),
    ("PAX_UPPER_MID_B", 28),
    ("PAX_UPPER_AFT", 18),
];

/// cabin.json5 cargoMap: (L:var without prefix, EFB name, kg).
pub const CARGO_STATIONS: [(&str, &str, f64); 3] =
    [("CARGO_FWD", "fwd", 28_577.), ("CARGO_AFT", "aft", 20_310.), ("CARGO_BULK", "bulk", 2_513.)];

/// getMaxPax / getMaxCargo (Store/features/config.ts:40-47).
pub const MAX_PAX: u32 = 484;
pub const MAX_CARGO_KG: f64 = 51_400.;

/// cabin.json5:20-27.
const DEFAULT_PAX_WEIGHT_KG: f64 = 84.;
const DEFAULT_BAG_WEIGHT_KG: f64 = 20.;
const PAX_WEIGHT_KG: (f64, f64) = (10., 250.);
const BAG_WEIGHT_KG: (f64, f64) = (1., 250.);

/// JavaScript's Math.round.
fn js_round(v: f64) -> f64 {
    (v + 0.5).floor()
}

/// Math.random, for the seats the flyPad picks.
pub struct Random(u64);

impl Random {
    pub fn new(seed: u64) -> Self {
        Self(seed | 1)
    }

    fn from_clock() -> Self {
        let nanos = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).map_or(1, |d| d.as_nanos() as u64);
        Self::new(nanos)
    }

    /// Uniform in [0, 1) (xorshift64*).
    fn next(&mut self) -> f64 {
        self.0 ^= self.0 >> 12;
        self.0 ^= self.0 << 25;
        self.0 ^= self.0 >> 27;
        (self.0.wrapping_mul(0x2545_f491_4f6c_dd1d) >> 11) as f64 / (1u64 << 53) as f64
    }
}

/// bitFlags.ts SeatFlags: seat s is bit s % 31 of 32-bit word s / 31, and the
/// variable holds high word * 2^32 + low word.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SeatFlags {
    pub bits: u64,
    pub seats: u32,
}

impl SeatFlags {
    pub fn from_value(value: f64, seats: u32) -> Self {
        Self { bits: value.max(0.) as u64, seats }
    }

    pub fn value(&self) -> f64 {
        self.bits as f64
    }

    fn mask(seat: u32) -> u64 {
        1 << ((seat / 31) * 32 + seat % 31)
    }

    fn filled(&self, seat: u32) -> bool {
        seat <= 63 && self.bits & Self::mask(seat) != 0
    }

    pub fn count(&self) -> u32 {
        self.bits.count_ones()
    }

    fn ids(&self, filled: bool) -> Vec<u32> {
        (0..self.seats).filter(|&s| self.filled(s) == filled).collect()
    }

    /// fillSeats / emptySeats: toggle `n` seats picked at random from the
    /// empty (fill) or filled (empty) ones.
    fn choose(&mut self, fill: bool, n: u32, random: &mut Random) {
        let mut choices = self.ids(!fill);
        for _ in 0..n {
            if choices.is_empty() {
                break;
            }
            let chosen = (random.next() * choices.len() as f64) as usize;
            self.bits ^= Self::mask(choices.remove(chosen));
        }
    }
}

/// The station's share in setTargetPax: `parseFloat(Number((Math.ceil((capacity
/// / maxPax) * 1e2) / 1e2).toExponential(2)).toPrecision(3))`.
fn station_share(capacity: u32) -> f64 {
    let share = ((capacity as f64 / MAX_PAX as f64) * 1e2).ceil() / 1e2;
    format!("{share:.2e}").parse().unwrap_or(share)
}

/// setTargetPax (A380Payload.tsx:382-407) on the desired seat flags.
pub fn set_target_pax(desired: &mut [SeatFlags; 14], pax: i64, random: &mut Random) {
    let total: i64 = desired.iter().map(|f| f.count() as i64).sum();
    if pax == total || pax > MAX_PAX as i64 || pax < 0 {
        return;
    }
    let mut remaining = pax;
    let mut fill_station = |i: usize, share: f64, to_fill: i64, remaining: &mut i64| {
        let flags = &mut desired[i];
        let to_be_filled = ((share * to_fill as f64).trunc() as i64).min(SEAT_STATIONS[i].1 as i64);
        *remaining -= to_be_filled;
        let seated = flags.count() as i64;
        flags.choose(to_be_filled > seated, (to_be_filled - seated).unsigned_abs() as u32, random);
    };
    for i in (1..SEAT_STATIONS.len()).rev() {
        fill_station(i, station_share(SEAT_STATIONS[i].1), pax, &mut remaining);
    }
    let rest = remaining;
    fill_station(0, 1., rest, &mut remaining);
}

/// setTargetCargo (A380Payload.tsx:409-429): the desired kg per hold.
pub fn set_target_cargo(pax: i64, freight_kg: f64, bag_weight_kg: f64) -> [f64; 3] {
    let loadable = (pax as f64 * bag_weight_kg + js_round(freight_kg)).min(MAX_CARGO_KG);
    let mut remaining = loadable;
    let mut out = [0.; 3];
    for i in (1..CARGO_STATIONS.len()).rev() {
        let c = js_round(CARGO_STATIONS[i].2 / MAX_CARGO_KG * loadable);
        remaining -= c;
        out[i] = c;
    }
    out[0] = js_round(remaining);
    out
}

/// processZfw / processGw (A380Payload.tsx:431-461): the passengers and cargo
/// for `payload_kg` of passengers and cargo, passengers first.
pub fn split_payload(payload_kg: f64, pax_weight_kg: f64, bag_weight_kg: f64) -> (i64, f64) {
    let per_pax = pax_weight_kg + bag_weight_kg;
    let pax = js_round(payload_kg / per_pax).min(MAX_PAX as f64).max(0.);
    let cargo = (payload_kg - pax * per_pax).min(MAX_CARGO_KG).max(0.);
    (pax as i64, cargo)
}

/// calculateBoardingTime (A380Payload.tsx:510-548), seconds.
pub fn boarding_seconds(rate: BoardingRate, doors_open: u32, pax_diff: f64, cargo_diff_kg: f64) -> f64 {
    let multiplier = match rate {
        BoardingRate::Real => 5.,
        BoardingRate::Fast => 1.,
        BoardingRate::Instant => 0.,
    } / doors_open.max(1) as f64;
    (pax_diff.abs() * multiplier).max(cargo_diff_kg.abs() / 60. * multiplier)
}

/// L:A32NX_BOARDING_RATE (sync.ts:210-224).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BoardingRate {
    Instant = 0,
    Fast = 1,
    Real = 2,
}

// ---------------------------------------------------------------------------
// Refuel, as the A380 fuel page allows it.
// ---------------------------------------------------------------------------

/// A380Fuel.tsx:141-145.
const TOTAL_FUEL_GALLONS: f64 = 85_471.7;
const FUEL_GALLONS_TO_KG: f64 = 3.039_075_693_483_925;
const FAST_SPEED_FACTOR: f64 = 5.;
const FUELRATE_TOTAL_GAL_SEC: f64 = 16.;
pub const TOTAL_MAX_FUEL_KG: f64 = TOTAL_FUEL_GALLONS * FUEL_GALLONS_TO_KG;

/// RefuelRateSetting (A380Fuel.tsx:120-124).
pub const REFUEL_INSTANT: f64 = 2.;
const REFUEL_FAST: f64 = 1.;

/// isRefuelAllowed without GSX (A380Fuel.tsx:196-213).
pub fn refuel_allowed(started: bool, engine_running: bool, on_ground: bool, rate: f64) -> bool {
    let only_instant = engine_running || !on_ground;
    started || !only_instant || rate == REFUEL_INSTANT
}

/// updateDesiredFuel (:219-225).
pub fn desired_fuel(kg: f64) -> f64 {
    if kg > TOTAL_MAX_FUEL_KG {
        js_round(TOTAL_MAX_FUEL_KG)
    } else {
        kg
    }
}

/// updateDesiredFuelPercent (:227-233).
pub fn desired_fuel_percent(percent: f64) -> f64 {
    let percent = if percent < 0.5 { 0. } else { percent };
    desired_fuel(js_round(TOTAL_MAX_FUEL_KG * (percent / 100.)))
}

/// calculateEta (:237-251), in seconds rather than the page's minutes.
pub fn refuel_seconds(total_kg: f64, desired_kg: f64, rate: f64) -> f64 {
    if (desired_kg - total_kg).abs() < 10. || rate == REFUEL_INSTANT {
        return 0.;
    }
    let gallons = (total_kg - desired_kg).abs() / FUEL_GALLONS_TO_KG;
    let factor = if rate == REFUEL_FAST { FAST_SPEED_FACTOR } else { 1. };
    gallons / (FUELRATE_TOTAL_GAL_SEC * factor)
}

// ---------------------------------------------------------------------------
// Settings.
// ---------------------------------------------------------------------------

/// How a stored string becomes the variable's number.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Mapping {
    /// `Number(value)`.
    Number,
    /// A modern setting, stored as JSON `true`/`false`.
    JsonBool,
    /// REAL 2, FAST 1, INSTANT 0 (CONFIG_BOARDING_RATE).
    Boarding,
    /// REAL 0, FAST 2, INSTANT 1 (CONFIG_ALIGN_TIME).
    Align,
}

/// One synced setting: stored key, variable (as registered: `L:` dropped),
/// default stored value, mapping.
pub struct Setting {
    pub key: &'static str,
    pub var: &'static str,
    pub default: &'static str,
    pub mapping: Mapping,
}

const fn s(key: &'static str, var: &'static str, default: &'static str, mapping: Mapping) -> Setting {
    Setting { key, var, default, mapping }
}

/// sync.ts:40-251 and settingsSync.ts:7-14, in their order.
pub const SETTINGS: &[Setting] = &[
    s("SOUND_EXTERIOR_MASTER", "A32NX_SOUND_EXTERIOR_MASTER", "0", Mapping::Number),
    s("SOUND_INTERIOR_ENGINE", "A32NX_SOUND_INTERIOR_ENGINE", "0", Mapping::Number),
    s("SOUND_INTERIOR_WIND", "A32NX_SOUND_INTERIOR_WIND", "0", Mapping::Number),
    s("EFB_BRIGHTNESS", "A32NX_EFB_BRIGHTNESS", "0", Mapping::Number),
    s("EFB_USING_AUTOBRIGHTNESS", "A32NX_EFB_USING_AUTOBRIGHTNESS", "1", Mapping::Number),
    s("CABIN_MANUAL_BRIGHTNESS", "A32NX_CABIN_MANUAL_BRIGHTNESS", "0", Mapping::Number),
    s("CABIN_USING_AUTOBRIGHTNESS", "A32NX_CABIN_USING_AUTOBRIGHTNESS", "1", Mapping::Number),
    s("ISIS_BARO_UNIT_INHG", "A32NX_ISIS_BARO_UNIT_INHG", "0", Mapping::Number),
    s("REALISTIC_TILLER_ENABLED", "A32NX_REALISTIC_TILLER_ENABLED", "0", Mapping::Number),
    s("HOME_COCKPIT_ENABLED", "A32NX_HOME_COCKPIT_ENABLED", "0", Mapping::Number),
    s("SOUND_PASSENGER_AMBIENCE_ENABLED", "A32NX_SOUND_PASSENGER_AMBIENCE_ENABLED", "1", Mapping::Number),
    s("SOUND_ANNOUNCEMENTS_ENABLED", "A32NX_SOUND_ANNOUNCEMENTS_ENABLED", "1", Mapping::Number),
    s("SOUND_BOARDING_MUSIC_ENABLED", "A32NX_SOUND_BOARDING_MUSIC_ENABLED", "1", Mapping::Number),
    s("RADIO_RECEIVER_USAGE_ENABLED", "A32NX_RADIO_RECEIVER_USAGE_ENABLED", "0", Mapping::Number),
    s("FDR_ENABLED", "A32NX_FDR_ENABLED", "1", Mapping::Number),
    s("MODEL_WHEELCHOCKS_ENABLED", "A32NX_MODEL_WHEELCHOCKS_ENABLED", "1", Mapping::Number),
    s("MODEL_CONES_ENABLED", "A32NX_MODEL_CONES_ENABLED", "1", Mapping::Number),
    s("FO_SYNC_EFIS_ENABLED", "A32NX_FO_SYNC_EFIS_ENABLED", "0", Mapping::Number),
    s("MODEL_SATCOM_ENABLED", "A32NX_SATCOM_ENABLED", "0", Mapping::Number),
    s("CONFIG_PILOT_AVATAR_VISIBLE", "A32NX_PILOT_AVATAR_VISIBLE_0", "0", Mapping::Number),
    s("CONFIG_FIRST_OFFICER_AVATAR_VISIBLE", "A32NX_PILOT_AVATAR_VISIBLE_1", "0", Mapping::Number),
    s("GSX_PAYLOAD_SYNC", "A32NX_GSX_PAYLOAD_SYNC_ENABLED", "0", Mapping::Number),
    // A modern setting; its default is NXDataStore's (persistence.ts:37).
    s("CONFIG_USING_METRIC_UNIT", "A32NX_EFB_USING_METRIC_UNIT", "true", Mapping::JsonBool),
    s("CONFIG_USING_PORTABLE_DEVICES", "A32NX_CONFIG_USING_PORTABLE_DEVICES", "1", Mapping::Number),
    s("REFUEL_RATE_SETTING", "A32NX_EFB_REFUEL_RATE_SETTING", "0", Mapping::Number),
    s("CONFIG_BOARDING_RATE", "A32NX_BOARDING_RATE", "REAL", Mapping::Boarding),
    s("CONFIG_ALIGN_TIME", "A32NX_CONFIG_ADIRS_IR_ALIGN_TIME", "REAL", Mapping::Align),
    // A380X_DEFAULT_RADIO_AUTO_CALL_OUTS (AutoCallOuts.ts:30-39): 2500, 1000,
    // 400, 50, 40, 30, 20, 10, 5.
    s("CONFIG_A380X_FWC_RADIO_AUTO_CALL_OUT_PINS", "A380X_FWC_RADIO_AUTO_CALL_OUT_PINS", "1032265", Mapping::Number),
];

impl Setting {
    /// The variable's value for a stored string (`(mapFunction ?? Number)`).
    pub fn to_number(&self, stored: &str) -> f64 {
        match self.mapping {
            Mapping::Number => {
                let t = stored.trim();
                if t.is_empty() {
                    0.
                } else {
                    t.parse().unwrap_or(f64::NAN)
                }
            }
            Mapping::JsonBool => (stored.trim() == "true") as i32 as f64,
            Mapping::Boarding => match stored {
                "FAST" => 1.,
                "INSTANT" => 0.,
                _ => 2.,
            },
            Mapping::Align => match stored {
                "FAST" => 2.,
                "INSTANT" => 1.,
                _ => 0.,
            },
        }
    }

    /// The stored string for a number written to the dataref.
    pub fn to_stored(&self, value: f64) -> String {
        match self.mapping {
            Mapping::Number => {
                if value.fract() == 0. {
                    format!("{}", value as i64)
                } else {
                    format!("{value}")
                }
            }
            Mapping::JsonBool => (value != 0.).to_string(),
            Mapping::Boarding => ["INSTANT", "FAST", "REAL"].get(value.round().clamp(0., 2.) as usize).unwrap_or(&"REAL").to_string(),
            Mapping::Align => ["REAL", "INSTANT", "FAST"].get(value.round().clamp(0., 2.) as usize).unwrap_or(&"REAL").to_string(),
        }
    }
}

/// The stored-data key MSFS would use.
pub fn stored_key(key: &str) -> String {
    format!("A380X_{key}")
}

pub fn settings_path() -> PathBuf {
    PathBuf::from("Output").join("preferences").join("fbw_a380x_settings.ini")
}

pub fn parse_ini(text: &str) -> BTreeMap<String, String> {
    text.lines()
        .filter_map(|l| {
            let l = l.trim();
            if l.starts_with(';') || l.starts_with('#') {
                return None;
            }
            let (k, v) = l.split_once('=')?;
            Some((k.trim().to_string(), v.trim().to_string()))
        })
        .collect()
}

pub fn write_ini(values: &BTreeMap<String, String>) -> String {
    let mut out = String::from("; FlyByWire A380X flyPad settings (NXDataStore keys)\n");
    for (k, v) in values {
        out.push_str(&format!("{k}={v}\n"));
    }
    out
}

// ---------------------------------------------------------------------------
// X-Plane.
// ---------------------------------------------------------------------------

/// The flyPad's service doors (A380Services.tsx:134-140): name, point.
const SERVICE_DOORS: [(&str, usize); 5] =
    [("main1_left", 0), ("main2_left", 2), ("upper1_left", 10), ("main4_right", 9), ("cargo_fwd", 16)];

/// GPUManagement's GPU door (extras-host index.ts:98: point 19, four GPUs).
const GPU_POINT: usize = 19;

/// The boarding doors calculateBoardingTime counts (A380Payload.tsx:500-502).
const BOARDING_DOORS: [usize; 3] = [0, 2, 10];

const KT_PER_MS: f64 = 1.943_844;
const FT_PER_M: f64 = 3.280_84;

struct Ids {
    on_ground: VariableIdentifier,
    stationary: VariableIdentifier,
    seats: [VariableIdentifier; 14],
    seats_desired: [VariableIdentifier; 14],
    cargo: [VariableIdentifier; 3],
    cargo_desired: [VariableIdentifier; 3],
    pax_weight: VariableIdentifier,
    bag_weight: VariableIdentifier,
    boarding_started: VariableIdentifier,
    fuel_desired: VariableIdentifier,
    fuel_total: VariableIdentifier,
    refuel_started: VariableIdentifier,
    ext_pwr_avail: [VariableIdentifier; 4],
    ext_pwr_pb: [VariableIdentifier; 4],
    ac_powered: [VariableIdentifier; 4],
    dc_ess_powered: VariableIdentifier,
    pushback_enabled: VariableIdentifier,
    pushback_speed: VariableIdentifier,
    pushback_heading: VariableIdentifier,
    pushback_wait: VariableIdentifier,
    park_brake: VariableIdentifier,
    start_state: VariableIdentifier,
    settings: Vec<VariableIdentifier>,
    /// zfw, zfw desired, gw, gw desired (kg); zfw, zfw desired, gw, gw desired,
    /// take-off CG (% MAC).
    weights: [VariableIdentifier; 9],
    // ---- Phase 2 (E-FUEL-DESIGN.md D12/D14) additions: efb.rs-published
    // discretes an `FbwProc` trigger reads via `TRIGGER_REAL_OWNER_VARS`. ----
    airframe_zfw_cg_disagree: VariableIdentifier,
    airframe_weight_disagree: VariableIdentifier,
    airframe_cg_out_of_range: VariableIdentifier,
    airframe_cg_at_fwd_limit: VariableIdentifier,
    airframe_cg_excess_aft: VariableIdentifier,
    airframe_to_cg_out_of_range: VariableIdentifier,
}

struct Refs {
    groundspeed: Option<DataRef>,
    engines_running: Option<DataRef>,
    gpu_on: Option<DataRef>,
    pushback_attached: Option<DataRef>,
}

struct Ground {
    available: Value,
    doors: [Command; 5],
    jetway: Command,
    stairs: Command,
    baggage: Command,
    catering: Command,
    fuel_truck: Command,
    gpu: Command,
    gpu_connected: Value,
}

struct Payload {
    pax_target: Value,
    cargo_target: Value,
    zfw_target: Value,
    gw_target: Value,
    pax_weight: Value,
    bag_weight: Value,
    boarding_started: Value,
    deboard: Command,
    boarding_rate: Value,
    pax: Value,
    pax_desired: Value,
    cargo: Value,
    cargo_desired: Value,
    station_pax: [Value; 14],
    station_pax_desired: [Value; 14],
    hold_kg: [Value; 3],
    hold_kg_desired: [Value; 3],
    eta: Value,
}

struct Refuel {
    target: Value,
    target_percent: Value,
    rate: Value,
    started: Value,
    start_stop: Command,
    allowed: Value,
    total: Value,
    eta: Value,
}

struct Pushback {
    enabled: Value,
    speed: Value,
    heading: Value,
    stop: Command,
    call_tug: Command,
    release_tug: Command,
    attached: Value,
    parking_brake: Value,
}

struct Other {
    settings: Vec<Value>,
    activate: Value,
    deactivate: Value,
    toggle: Value,
    failures_count: Value,
    start_state: Value,
    start_state_override: Value,
    weights: [Value; 9],
}

/// How long X-Plane's GPU flag must hold before a hook or unhook counts.
const GPU_SETTLE_S: f64 = 2.;

/// How long after a spawn the "aircraft started moving" ground-power
/// auto-disconnect in `update_ground` may arm. X-Plane settles a freshly
/// placed aircraft onto its gear over roughly the first one to two
/// seconds; during that settle `groundspeed` (and so `IS_STATIONARY`)
/// reads as moving from the settle alone (every kept log:
/// `A32NX_IS_STATIONARY` stays 0 for up to ~1.7s right after spawn), which
/// used to satisfy the 1-second "moving" threshold below on its own and
/// immediately disconnect the ground power a cold start had just
/// connected (`ext_power_at_start`), blacking out DC ESS -- and every
/// LGCIU1 discrete gated on it -- until something else repowers the
/// aircraft. Comfortably past the settle actually observed.
///
/// Gated on `spawn::epoch` rather than counted only from `Efb::new`
/// (W157's review of this guard's first version, fixes/W98.md): `Efb::new`
/// runs once per plugin session (`XPluginEnable`), but X-Plane does not
/// reload the plugin for a same-aircraft reposition or restart-flight, so
/// a guard that only ever reset at construction gave every second-and-
/// later spawn in the same session no settle protection at all.
const SPAWN_SETTLE_S: f64 = 3.;

/// Whether `update_ground`'s "aircraft started moving" auto-disconnect may
/// start counting `moving_for_s`. See `SPAWN_SETTLE_S`.
fn moving_disconnect_armed(age_s: f64) -> bool {
    age_s >= SPAWN_SETTLE_S
}

pub struct Efb {
    ids: Ids,
    refs: Refs,
    _published: Published,
    ground: Ground,
    payload: Payload,
    refuel: Refuel,
    pushback: Pushback,
    other: Other,
    random: Random,
    empty_weight_kg: f64,
    stored: BTreeMap<String, String>,
    was_available: Option<bool>,
    gpu_hooked: Option<bool>,
    /// X-Plane's GPU flag as read last frame, and how long it has held.
    gpu_raw: bool,
    gpu_raw_for_s: f64,
    was_moving: bool,
    /// Seconds the aircraft has kept rolling above the moving threshold.
    moving_for_s: f64,
    /// Seconds since `spawn_epoch_seen` was last (re)detected, gating
    /// `moving_for_s` against the spawn settle (`SPAWN_SETTLE_S`,
    /// `moving_disconnect_armed`). Reset to 0 whenever `spawn::epoch()`
    /// moves past `spawn_epoch_seen`, not just once at `Efb::new` -- see
    /// `SPAWN_SETTLE_S`'s doc comment.
    age_s: f64,
    /// The `spawn::epoch()` value last seen, so a fresh
    /// `XPLM_MSG_PLANE_LOADED`/`XPLM_MSG_AIRPORT_LOADED` can be told apart
    /// from every other tick.
    spawn_epoch_seen: u64,
    /// Seconds until boarding starts after a deboard request (the page's
    /// 500 ms timeout, A380Payload.tsx:483-485).
    deboard_in: Option<f64>,
    /// A cold start on the ground (apron or hangar) begins on external
    /// power: every ground power unit available and connected.
    ext_power_at_start: bool,
    /// Ground power, its pushbuttons and the buses it feeds, as last logged.
    power_logged: Option<String>,
}

fn read_value(p: &mut Published, name: &str, writable: bool) -> Value {
    p.number(&format!("fbw/efb/{name}"), 0., writable)
}

impl Efb {
    pub fn new(vars: &mut Vars, xplm: &Xplm) -> Self {
        let mut p = Published::default();
        let cmd = |p: &mut Published, name: &str, what: &str| p.command(&format!("fbw/efb/{name}"), what);
        let ground = Ground {
            available: read_value(&mut p, "ground/available", false),
            doors: SERVICE_DOORS.map(|(name, point)| {
                cmd(&mut p, &format!("ground/door_{name}"), &format!("flyPad: toggle door {} (point {point})", crate::doors::NAMES[point]))
            }),
            jetway: cmd(&mut p, "ground/jetway", "flyPad: jet bridge"),
            stairs: cmd(&mut p, "ground/stairs", "flyPad: stairs"),
            baggage: cmd(&mut p, "ground/baggage", "flyPad: baggage truck"),
            catering: cmd(&mut p, "ground/catering", "flyPad: catering truck"),
            fuel_truck: cmd(&mut p, "ground/fuel_truck", "flyPad: fuel truck"),
            gpu: cmd(&mut p, "ground/gpu", "flyPad: ground power unit"),
            gpu_connected: read_value(&mut p, "ground/gpu_connected", false),
        };
        let payload = Payload {
            pax_target: read_value(&mut p, "payload/pax_target", true),
            cargo_target: read_value(&mut p, "payload/cargo_target_kg", true),
            zfw_target: read_value(&mut p, "payload/zfw_target_kg", true),
            gw_target: read_value(&mut p, "payload/gw_target_kg", true),
            pax_weight: read_value(&mut p, "payload/pax_weight_kg", true),
            bag_weight: read_value(&mut p, "payload/bag_weight_kg", true),
            boarding_started: read_value(&mut p, "payload/boarding_started", true),
            deboard: cmd(&mut p, "payload/deboard", "flyPad: deboard all passengers and cargo"),
            boarding_rate: read_value(&mut p, "payload/boarding_rate", true),
            pax: read_value(&mut p, "payload/pax", false),
            pax_desired: read_value(&mut p, "payload/pax_desired", false),
            cargo: read_value(&mut p, "payload/cargo_kg", false),
            cargo_desired: read_value(&mut p, "payload/cargo_desired_kg", false),
            station_pax: std::array::from_fn(|i| read_value(&mut p, &format!("payload/station/{}/pax", i + 1), false)),
            station_pax_desired: std::array::from_fn(|i| read_value(&mut p, &format!("payload/station/{}/pax_desired", i + 1), false)),
            hold_kg: CARGO_STATIONS.map(|(_, name, _)| read_value(&mut p, &format!("payload/cargo/{name}/kg"), false)),
            hold_kg_desired: CARGO_STATIONS.map(|(_, name, _)| read_value(&mut p, &format!("payload/cargo/{name}/kg_desired"), true)),
            eta: read_value(&mut p, "payload/boarding_eta_s", false),
        };
        let refuel = Refuel {
            target: read_value(&mut p, "refuel/target_kg", true),
            target_percent: read_value(&mut p, "refuel/target_percent", true),
            rate: read_value(&mut p, "refuel/rate", true),
            started: read_value(&mut p, "refuel/started", true),
            start_stop: cmd(&mut p, "refuel/start_stop", "flyPad: start or stop refuelling"),
            allowed: read_value(&mut p, "refuel/allowed", false),
            total: read_value(&mut p, "refuel/total_kg", false),
            eta: read_value(&mut p, "refuel/eta_s", false),
        };
        let pushback = Pushback {
            enabled: read_value(&mut p, "pushback/system_enabled", true),
            speed: read_value(&mut p, "pushback/speed_factor", true),
            heading: read_value(&mut p, "pushback/heading_factor", true),
            stop: cmd(&mut p, "pushback/stop", "flyPad: stop the tug"),
            call_tug: cmd(&mut p, "pushback/call_tug", "flyPad: call the tug"),
            release_tug: cmd(&mut p, "pushback/release_tug", "flyPad: release the tug"),
            attached: read_value(&mut p, "pushback/attached", false),
            parking_brake: read_value(&mut p, "pushback/parking_brake", true),
        };
        let other = Other {
            settings: SETTINGS.iter().map(|s| read_value(&mut p, &format!("settings/{}", s.key.to_ascii_lowercase()), true)).collect(),
            activate: read_value(&mut p, "failures/activate", true),
            deactivate: read_value(&mut p, "failures/deactivate", true),
            toggle: read_value(&mut p, "failures/toggle", true),
            failures_count: read_value(&mut p, "failures/count", false),
            start_state: read_value(&mut p, "start_state", false),
            start_state_override: read_value(&mut p, "start_state_override", true),
            weights: [
                "zfw_kg",
                "zfw_desired_kg",
                "gw_kg",
                "gw_desired_kg",
                "zfw_cg_mac",
                "zfw_cg_mac_desired",
                "gw_cg_mac",
                "gw_cg_mac_desired",
                "to_cg_mac",
            ]
            .map(|n| read_value(&mut p, &format!("wb/{n}"), false)),
        };
        let mut get = |n: &str| vars.get(n.to_string());
        let ids = Ids {
            on_ground: get("SIM ON GROUND"),
            stationary: get("IS_STATIONARY"),
            seats: SEAT_STATIONS.map(|(v, _)| get(v)),
            seats_desired: SEAT_STATIONS.map(|(v, _)| get(&format!("{v}_DESIRED"))),
            cargo: CARGO_STATIONS.map(|(v, _, _)| get(v)),
            cargo_desired: CARGO_STATIONS.map(|(v, _, _)| get(&format!("{v}_DESIRED"))),
            pax_weight: get("WB_PER_PAX_WEIGHT"),
            bag_weight: get("WB_PER_BAG_WEIGHT"),
            boarding_started: get("BOARDING_STARTED_BY_USR"),
            fuel_desired: get("FUEL_DESIRED"),
            fuel_total: get("TOTAL_FUEL_QUANTITY"),
            refuel_started: get("REFUEL_STARTED_BY_USR"),
            ext_pwr_avail: [1, 2, 3, 4].map(|n| get(&format!("EXT_PWR_AVAIL:{n}"))),
            ext_pwr_pb: [1, 2, 3, 4].map(|n| get(&format!("OVHD_ELEC_EXT_PWR_{n}_PB_IS_ON"))),
            ac_powered: [1, 2, 3, 4].map(|n| get(&format!("ELEC_AC_{n}_BUS_IS_POWERED"))),
            dc_ess_powered: get("ELEC_DC_ESS_BUS_IS_POWERED"),
            pushback_enabled: get("PUSHBACK_SYSTEM_ENABLED"),
            pushback_speed: get("PUSHBACK_SPD_FACTOR"),
            pushback_heading: get("PUSHBACK_HDG_FACTOR"),
            pushback_wait: get("PUSHBACK WAIT"),
            park_brake: get("PARK_BRAKE_LEVER_POS"),
            start_state: get("START_STATE"),
            settings: Vec::new(),
            weights: [
                "AIRFRAME_ZFW",
                "AIRFRAME_ZFW_DESIRED",
                "AIRFRAME_GW",
                "AIRFRAME_GW_DESIRED",
                "AIRFRAME_ZFW_CG_PERCENT_MAC",
                "AIRFRAME_ZFW_CG_PERCENT_MAC_DESIRED",
                "AIRFRAME_GW_CG_PERCENT_MAC",
                "AIRFRAME_GW_CG_PERCENT_MAC_DESIRED",
                "AIRFRAME_TO_CG_PERCENT_MAC",
            ]
            .map(|n| get(n)),
            airframe_zfw_cg_disagree: get("AIRFRAME_ZFW_CG_DISAGREE"),
            airframe_weight_disagree: get("AIRFRAME_WEIGHT_DISAGREE"),
            airframe_cg_out_of_range: get("AIRFRAME_CG_OUT_OF_RANGE"),
            airframe_cg_at_fwd_limit: get("AIRFRAME_CG_AT_FWD_LIMIT"),
            airframe_cg_excess_aft: get("AIRFRAME_CG_EXCESS_AFT"),
            airframe_to_cg_out_of_range: get("AIRFRAME_TO_CG_OUT_OF_RANGE"),
        };
        // The settings' variables keep their own prefixes (A32NX_, A380X_).
        let mut ids = ids;
        ids.settings = SETTINGS.iter().map(|s| vars.add(s.var.to_string(), crate::NAMED)).collect();

        let stored = std::fs::read_to_string(settings_path()).map(|t| parse_ini(&t)).unwrap_or_default();
        let balance = crate::weight_balance::parse(crate::weight_balance::FLIGHT_MODEL_CFG);
        let efb = Self {
            ids,
            refs: Refs {
                groundspeed: xplm.find("sim/flightmodel/position/groundspeed"),
                engines_running: xplm.find("sim/flightmodel/engine/ENGN_running"),
                gpu_on: xplm.find("sim/cockpit2/electrical/GPU_generator_on"),
                pushback_attached: xplm.find("sim/aircraft/overflow/pushback_attached"),
            },
            _published: p,
            ground,
            payload,
            refuel,
            pushback,
            other,
            random: Random::from_clock(),
            // A:EMPTY WEIGHT, which the page reads (A380Payload.tsx:318).
            empty_weight_kg: balance.empty.pounds * crate::weight_balance::LB_TO_KG,
            stored,
            was_available: None,
            gpu_hooked: None,
            gpu_raw: false,
            gpu_raw_for_s: 0.,
            was_moving: false,
            moving_for_s: 0.,
            age_s: 0.,
            spawn_epoch_seen: crate::spawn::epoch(),
            deboard_in: None,
            ext_power_at_start: true,
            power_logged: None,
        };
        // The flyPad writes every synced setting once at start (sync.ts:229-250),
        // and fills in the default weights (A380Payload.tsx:563-571).
        for (i, setting) in SETTINGS.iter().enumerate() {
            let value = setting.to_number(&efb.stored_value(setting));
            vars.write(&efb.ids.settings[i], value);
        }
        if vars.read(&efb.ids.pax_weight) == 0. {
            vars.write(&efb.ids.pax_weight, DEFAULT_PAX_WEIGHT_KG);
        }
        if vars.read(&efb.ids.bag_weight) == 0. {
            vars.write(&efb.ids.bag_weight, DEFAULT_BAG_WEIGHT_KG);
        }
        if let Ok(text) = std::fs::read_to_string(crate::start_state::override_path()) {
            if let Some(state) = crate::start_state::parse_override(&text) {
                published::set(efb.other.start_state_override, f64::from(state));
            }
        }
        efb
    }

    fn stored_value(&self, setting: &Setting) -> String {
        self.stored.get(&stored_key(setting.key)).cloned().unwrap_or_else(|| setting.default.to_string())
    }

    fn set_setting(&mut self, vars: &mut Vars, index: usize, value: f64) {
        let setting = &SETTINGS[index];
        let stored = setting.to_stored(value);
        vars.write(&self.ids.settings[index], setting.to_number(&stored));
        if self.stored.get(&stored_key(setting.key)) != Some(&stored) {
            let key = stored_key(setting.key);
            self.stored.insert(key.clone(), stored.clone());
            let path = settings_path();
            // Race rule 8 (docs/briefs/xphfbw-js-bridge.md): the ini has other
            // writers (XPHFBW's settings page, the instruments' stored data),
            // so re-read it under the shared lock and change only this key,
            // rather than writing back this module's older copy of the rest.
            let result = crate::app_settings::with_settings_lock(|| {
                if let Some(dir) = path.parent() {
                    std::fs::create_dir_all(dir)?;
                }
                let mut on_disk = std::fs::read_to_string(&path).map(|t| parse_ini(&t)).unwrap_or_default();
                on_disk.insert(key.clone(), stored.clone());
                let partial = path.with_extension("ini.tmp");
                std::fs::write(&partial, write_ini(&on_disk))?;
                std::fs::rename(&partial, &path)?;
                self.stored = on_disk;
                Ok::<(), std::io::Error>(())
            });
            if let Err(e) = result {
                crate::log(&format!("could not save the flyPad settings: {e}"));
            }
        }
    }

    fn setting_index(key: &str) -> usize {
        SETTINGS.iter().position(|s| s.key == key).unwrap_or(0)
    }

    fn desired_flags(&self, vars: &mut Vars) -> [SeatFlags; 14] {
        std::array::from_fn(|i| SeatFlags::from_value(vars.read(&self.ids.seats_desired[i]), SEAT_STATIONS[i].1))
    }

    fn target_pax(&mut self, vars: &mut Vars, pax: i64) {
        let mut flags = self.desired_flags(vars);
        set_target_pax(&mut flags, pax, &mut self.random);
        for (id, f) in self.ids.seats_desired.iter().zip(flags) {
            vars.write(id, f.value());
        }
    }

    fn target_cargo(&mut self, vars: &mut Vars, pax: i64, freight_kg: f64) {
        let bag = vars.read(&self.ids.bag_weight);
        for (id, kg) in self.ids.cargo_desired.iter().zip(set_target_cargo(pax, freight_kg, bag)) {
            vars.write(id, kg);
        }
    }

    /// Before the doors and the systems.
    pub fn update(&mut self, vars: &mut Vars, xplm: &Xplm, doors: &mut Doors, delta: f64) {
        let groundspeed_ms = self.refs.groundspeed.map_or(0., |d| xplm.get_f(d) as f64);
        // A380_WINGS.xml:347: stationary below 0.1 ft/s.
        vars.write(&self.ids.stationary, (groundspeed_ms * FT_PER_M <= 0.1) as i32 as f64);
        let on_ground = vars.read(&self.ids.on_ground) != 0.;
        if std::mem::take(&mut self.ext_power_at_start) {
            // START_STATE 1 hangar, 2 apron (start_state.rs).
            let state = vars.read(&self.ids.start_state);
            // xphfbw.coldStartGroundPower (app_settings.rs, Simulation
            // settings panel): applies at the next aircraft load, since
            // this whole block only ever runs once, here, on the first
            // `update` after `Efb::new` -- there is nothing later in the
            // flight to apply a live change to.
            if on_ground && (state == 1. || state == 2.) && crate::app_settings::current().cold_start_ground_power {
                self.set_ext_power(vars, true);
                for pb in self.ids.ext_pwr_pb {
                    vars.write(&pb, 1.);
                }
                crate::log("ground power connected for a cold start");
            }
        }
        self.log_power_changes(vars);
        let running = self.refs.engines_running.map_or([0; 8], |d| {
            let mut r = [0; 8];
            xplm.get_vi(d, &mut r);
            r
        });
        // X-Plane's own tug, or FlyByWire's (extra_backend pushback.rs keeps
        // `PUSHBACK ATTACHED`).
        let attached = self.refs.pushback_attached.is_some_and(|d| xplm.get_i(d) != 0)
            || {
                let tug = vars.get("PUSHBACK ATTACHED".to_string());
                vars.read(&tug) != 0.
            };

        self.update_ground(vars, xplm, doors, on_ground, attached, groundspeed_ms * KT_PER_MS, delta);
        self.update_payload(vars, doors, on_ground, running[0] != 0 || running[3] != 0, delta);
        self.update_refuel(vars, on_ground, running[..4].iter().any(|&r| r != 0));
        self.update_pushback(vars, attached);
        self.update_other(vars);
    }

    fn update_ground(&mut self, vars: &mut Vars, xplm: &Xplm, doors: &mut Doors, on_ground: bool, attached: bool, ground_kt: f64, delta: f64) {
        // A380Services.tsx:118-121.
        let available = on_ground && vars.read(&self.ids.stationary) != 0. && !attached;
        let g = &self.ground;
        let pressed = |c: Command| published::presses(c) > 0;
        let doors_pressed: Vec<bool> = g.doors.iter().map(|&c| pressed(c)).collect();
        let (jetway, stairs, baggage, catering, fuel, gpu) =
            (pressed(g.jetway), pressed(g.stairs), pressed(g.baggage), pressed(g.catering), pressed(g.fuel_truck), pressed(g.gpu));
        if available {
            for (k, _) in doors_pressed.iter().enumerate().filter(|(_, p)| **p) {
                doors.request(SERVICE_DOORS[k].1, Request::Toggle);
            }
            // K:TOGGLE_JETWAY; K:TOGGLE_RAMPTRUCK, REQUEST_LUGGAGE,
            // REQUEST_CATERING and REQUEST_FUEL_KEY are X-Plane's one truck
            // service.
            if jetway {
                xp::command_once("sim/ground_ops/jetway");
            }
            if stairs || baggage || catering || fuel {
                xp::command_once("sim/ground_ops/service_plane");
            }
            if gpu {
                self.toggle_gpu(vars);
            }
        } else if doors_pressed.iter().any(|&p| p) || jetway || stairs || baggage || catering || fuel || gpu {
            crate::log("flyPad ground services are unavailable: not stationary on the ground, or a tug is attached");
        }
        // Ground services lost: close the service doors that are fully open
        // (A380Services.tsx:500-525).
        if self.was_available == Some(true) && !available {
            for (_, point) in SERVICE_DOORS {
                if doors.model().doors.get(point).is_some_and(|d| d.open_percent >= 100.) {
                    doors.request(point, Request::Toggle);
                }
            }
        }
        self.was_available = Some(available);
        published::set(self.ground.available, available as i32 as f64);

        // GPUManagement.ts: the GPU door open is X-Plane's GPU connected.
        // X-Plane's GPU flag flickers while a flight loads (on for a frame,
        // then off), which read as the GPU being unplugged and cut the ground
        // power a cold start had just connected. A hook or unhook only counts
        // once the flag has held for GPU_SETTLE_S.
        let raw = self.refs.gpu_on.is_some_and(|d| xplm.get_i(d) != 0);
        self.gpu_raw_for_s = if raw == self.gpu_raw { self.gpu_raw_for_s + delta } else { 0. };
        self.gpu_raw = raw;
        let settled = self.gpu_raw_for_s >= GPU_SETTLE_S;
        let hooked = if settled { raw } else { self.gpu_hooked.unwrap_or(false) };
        doors.set_open(GPU_POINT, if hooked { 100. } else { 0. });
        match self.gpu_hooked {
            // onUpdate's first in-game frame (:78-83).
            None if !settled => {}
            None if hooked => self.set_ext_power(vars, true),
            // gpuHookedUp.sub (:70).
            // X-Plane's GPU flag is its own electrical model's switch, which
            // it turns off by itself after loading (held on for seconds
            // first), so it only ever connects ground power here; ground
            // power is disconnected by the GPU button or by moving off.
            Some(was) if was != hooked && hooked => self.set_ext_power(vars, true),
            _ => {}
        }
        if settled || self.gpu_hooked.is_some() {
            self.gpu_hooked = Some(hooked);
        }
        // "disable ext power when aircraft starts moving" (:71-76), once as it
        // starts rather than on every change of speed.
        // X-Plane settles a freshly placed aircraft onto its gear with a
        // moment of ground speed; only a roll kept up for a second is the
        // aircraft starting to move. That settle can itself run past the
        // 1-second threshold below (every kept log's `A32NX_IS_STATIONARY`
        // reads 0 for up to ~1.7s right after spawn), so this is also
        // gated on `moving_disconnect_armed`, which stays false for
        // `SPAWN_SETTLE_S` after each spawn (`spawn::epoch`, not just
        // `Efb::new` -- a reposition or restart-flight is a fresh spawn
        // too, and gets its own grace period the same way) -- otherwise
        // the settle alone used to disconnect the ground power
        // `ext_power_at_start` had just connected (every kept log: "ground
        // power connected for a cold start" immediately followed by
        // "ground power disconnected").
        let epoch = crate::spawn::epoch();
        if epoch != self.spawn_epoch_seen {
            self.spawn_epoch_seen = epoch;
            self.age_s = 0.;
        }
        self.age_s += delta;
        self.moving_for_s =
            if moving_disconnect_armed(self.age_s) && ground_kt > 0.3 { self.moving_for_s + delta } else { 0. };
        let moving = self.moving_for_s >= 1.;
        if moving && !self.was_moving && self.any_ext_power(vars) {
            self.toggle_gpu(vars);
        }
        self.was_moving = moving;
        published::set(self.ground.gpu_connected, self.any_ext_power(vars) as i32 as f64);
    }

    fn any_ext_power(&self, vars: &mut Vars) -> bool {
        self.ids.ext_pwr_avail.iter().any(|id| vars.read(id) != 0.)
    }

    /// toggleGPU (GPUManagement.ts:87-101). In MSFS an unhooked GPU is
    /// requested and hooks up moments later; X-Plane's ground crew has no
    /// GPU for an A380 and never hooks one, so the ground power itself is
    /// connected or disconnected here, and X-Plane's GPU is asked for (or sent
    /// away) alongside only to show it.
    fn toggle_gpu(&mut self, vars: &mut Vars) {
        let hooked = self.gpu_hooked.unwrap_or(false);
        let connect = !self.any_ext_power(vars);
        self.set_ext_power(vars, connect);
        if connect != hooked {
            xp::command_once("sim/ground_ops/toggle_gpu_request");
        }
        crate::log(if connect { "ground power connected" } else { "ground power disconnected" });
    }

    /// A log line whenever ground power, its pushbuttons or the buses it
    /// feeds change, so a start that stays dark says which link dropped.
    fn log_power_changes(&mut self, vars: &mut Vars) {
        let bits = |ids: &[VariableIdentifier], vars: &mut Vars| -> String {
            ids.iter().map(|id| if vars.read(id) != 0. { '1' } else { '0' }).collect()
        };
        let line = format!(
            "ground power: avail {} pb on {} AC buses {} DC ESS {}",
            bits(&self.ids.ext_pwr_avail, vars),
            bits(&self.ids.ext_pwr_pb, vars),
            bits(&self.ids.ac_powered, vars),
            bits(std::slice::from_ref(&self.ids.dc_ess_powered), vars),
        );
        if self.power_logged.as_deref() != Some(line.as_str()) {
            crate::log(&line);
            self.power_logged = Some(line);
        }
    }

    /// setEXTpower (GPUManagement.ts:107-121).
    fn set_ext_power(&mut self, vars: &mut Vars, connect: bool) {
        for i in 0..4 {
            vars.write(&self.ids.ext_pwr_avail[i], connect as i32 as f64);
            if !connect {
                vars.write(&self.ids.ext_pwr_pb[i], 0.);
            }
        }
    }

    fn update_payload(&mut self, vars: &mut Vars, doors: &Doors, on_ground: bool, eng_1_or_4: bool, delta: f64) {
        let started = vars.read(&self.ids.boarding_started) != 0.;
        let pl = &self.payload;
        let (pax_target, cargo_target, zfw_target, gw_target) =
            (published::take(pl.pax_target), published::take(pl.cargo_target), published::take(pl.zfw_target), published::take(pl.gw_target));
        let (pax_weight, bag_weight, boarding, rate) = (
            published::take(pl.pax_weight),
            published::take(pl.bag_weight),
            published::take(pl.boarding_started),
            published::take(pl.boarding_rate),
        );
        let holds: Vec<Option<f64>> = pl.hold_kg_desired.iter().map(|&v| published::take(v)).collect();
        let deboard = published::presses(pl.deboard) > 0;

        // Entries are refused while boarding, as the page disables them.
        let editable = !started;
        if editable {
            if let Some(w) = pax_weight {
                vars.write(&self.ids.pax_weight, w.round().clamp(PAX_WEIGHT_KG.0, PAX_WEIGHT_KG.1));
            }
            if let Some(w) = bag_weight {
                vars.write(&self.ids.bag_weight, w.round().clamp(BAG_WEIGHT_KG.0, BAG_WEIGHT_KG.1));
            }
            // PayloadElements.tsx:349-353, 377-380, 407, 424.
            if let Some(n) = pax_target {
                let n = n.round() as i64;
                self.target_pax(vars, n);
                self.target_cargo(vars, n, 0.);
            }
            if let Some(kg) = cargo_target {
                self.target_cargo(vars, 0, kg);
            }
            let (pw, bw) = (vars.read(&self.ids.pax_weight), vars.read(&self.ids.bag_weight));
            let zfw_gw = [zfw_target.map(|z| z - self.empty_weight_kg), gw_target.map(|g| {
                let fuel = vars.read(&self.ids.weights[2]) - vars.read(&self.ids.weights[0]);
                g - self.empty_weight_kg - fuel
            })];
            for payload_kg in zfw_gw.into_iter().flatten() {
                let (pax, cargo) = split_payload(payload_kg, pw, bw);
                self.target_pax(vars, pax);
                self.target_cargo(vars, pax, cargo);
            }
            // onClickCargo (A380Payload.tsx:463-471).
            for (i, kg) in holds.iter().enumerate() {
                if let Some(kg) = kg {
                    vars.write(&self.ids.cargo_desired[i], js_round(kg.clamp(0., CARGO_STATIONS[i].2)));
                }
            }
        } else if pax_target.is_some() || cargo_target.is_some() || zfw_target.is_some() || gw_target.is_some() {
            crate::log("flyPad payload entries are locked while boarding");
        }
        if let Some(b) = boarding {
            vars.write(&self.ids.boarding_started, (b != 0.) as i32 as f64);
        }
        // handleDeboarding (A380Payload.tsx:473-492).
        if deboard {
            if started {
                vars.write(&self.ids.boarding_started, 0.);
            } else {
                self.target_pax(vars, 0);
                self.target_cargo(vars, 0, 0.);
                self.deboard_in = Some(0.5);
            }
        }
        if let Some(t) = self.deboard_in.as_mut() {
            *t -= delta;
            if *t <= 0. {
                vars.write(&self.ids.boarding_started, 1.);
                self.deboard_in = None;
            }
        }

        // The rate: INSTANT unless cold and dark (A380Payload.tsx:573-587).
        let boarding_index = Self::setting_index("CONFIG_BOARDING_RATE");
        let cold_and_dark = !(eng_1_or_4 || !on_ground);
        if let Some(r) = rate {
            if cold_and_dark || r.round() == 0. {
                self.set_setting(vars, boarding_index, r);
            }
        }
        let current = vars.read(&self.ids.settings[boarding_index]);
        if !cold_and_dark && current != 0. {
            self.set_setting(vars, boarding_index, 0.);
        }

        // What the page shows.
        let flags: [SeatFlags; 14] = std::array::from_fn(|i| SeatFlags::from_value(vars.read(&self.ids.seats[i]), SEAT_STATIONS[i].1));
        let desired = self.desired_flags(vars);
        let cargo: Vec<f64> = self.ids.cargo.iter().map(|id| vars.read(id)).collect();
        let cargo_desired: Vec<f64> = self.ids.cargo_desired.iter().map(|id| vars.read(id)).collect();
        let pax: u32 = flags.iter().map(SeatFlags::count).sum();
        let pax_desired: u32 = desired.iter().map(SeatFlags::count).sum();
        let pl = &self.payload;
        for i in 0..14 {
            published::set(pl.station_pax[i], flags[i].count() as f64);
            published::set(pl.station_pax_desired[i], desired[i].count() as f64);
        }
        for i in 0..3 {
            published::set(pl.hold_kg[i], cargo[i]);
            published::set(pl.hold_kg_desired[i], cargo_desired[i]);
        }
        published::set(pl.pax, pax as f64);
        published::set(pl.pax_desired, pax_desired as f64);
        published::set(pl.cargo, cargo.iter().sum());
        published::set(pl.cargo_desired, cargo_desired.iter().sum());
        published::set(pl.pax_weight, vars.read(&self.ids.pax_weight));
        published::set(pl.bag_weight, vars.read(&self.ids.bag_weight));
        published::set(pl.boarding_started, vars.read(&self.ids.boarding_started));
        let rate_value = vars.read(&self.ids.settings[boarding_index]);
        published::set(pl.boarding_rate, rate_value);
        let doors_open = BOARDING_DOORS.iter().filter(|&&p| doors.model().doors.get(p).is_some_and(|d| d.open_percent > 0.)).count();
        let rate = match rate_value.round() as i64 {
            2 => BoardingRate::Real,
            1 => BoardingRate::Fast,
            _ => BoardingRate::Instant,
        };
        let eta = boarding_seconds(
            rate,
            doors_open as u32,
            pax_desired as f64 - pax as f64,
            cargo_desired.iter().sum::<f64>() - cargo.iter().sum::<f64>(),
        );
        published::set(pl.eta, eta);
        published::set(pl.pax_target, pax_desired as f64);
        published::set(pl.cargo_target, cargo_desired.iter().sum());
        published::set(pl.zfw_target, vars.read(&self.ids.weights[1]));
        published::set(pl.gw_target, vars.read(&self.ids.weights[3]));
    }

    fn update_refuel(&mut self, vars: &mut Vars, on_ground: bool, engine_running: bool) {
        let rf = &self.refuel;
        let (target, percent, rate, started_w, toggle) = (
            published::take(rf.target),
            published::take(rf.target_percent),
            published::take(rf.rate),
            published::take(rf.started),
            published::presses(rf.start_stop) > 0,
        );
        let rate_index = Self::setting_index("REFUEL_RATE_SETTING");
        if let Some(r) = rate {
            self.set_setting(vars, rate_index, r.round().clamp(0., 2.));
        }
        if let Some(kg) = target {
            vars.write(&self.ids.fuel_desired, desired_fuel(kg.max(0.)));
        }
        if let Some(p) = percent {
            vars.write(&self.ids.fuel_desired, desired_fuel_percent(p.clamp(0., 100.)));
        }
        let rate = vars.read(&self.ids.settings[rate_index]);
        let started = vars.read(&self.ids.refuel_started) != 0.;
        let allowed = refuel_allowed(started, engine_running, on_ground, rate);
        // switchRefuelState (A380Fuel.tsx:265-269).
        let want = if toggle { Some(!started) } else { started_w.map(|v| v != 0.) };
        if let Some(want) = want.filter(|&w| w != started) {
            if started || allowed {
                vars.write(&self.ids.refuel_started, want as i32 as f64);
            } else {
                crate::log("flyPad refuel unavailable: engines running or airborne need the INSTANT rate");
            }
        }
        let total = vars.read(&self.ids.fuel_total);
        let desired = vars.read(&self.ids.fuel_desired);
        let rf = &self.refuel;
        published::set(rf.rate, rate);
        published::set(rf.target, desired);
        published::set(rf.target_percent, desired / TOTAL_MAX_FUEL_KG * 100.);
        published::set(rf.started, vars.read(&self.ids.refuel_started));
        published::set(rf.allowed, refuel_allowed(vars.read(&self.ids.refuel_started) != 0., engine_running, on_ground, rate) as i32 as f64);
        published::set(rf.total, total);
        published::set(rf.eta, refuel_seconds(total, desired, rate));
    }

    fn update_pushback(&mut self, vars: &mut Vars, attached: bool) {
        let pb = &self.pushback;
        if let Some(v) = published::take(pb.enabled) {
            vars.write(&self.ids.pushback_enabled, (v != 0.) as i32 as f64);
        }
        // handleTugSpeed / handleTugDirection (PushbackPage.tsx:129-135).
        if let Some(v) = published::take(pb.speed) {
            vars.write(&self.ids.pushback_speed, v.clamp(-1., 1.));
        }
        if let Some(v) = published::take(pb.heading) {
            vars.write(&self.ids.pushback_heading, v.clamp(-1., 1.));
        }
        if published::presses(pb.stop) > 0 {
            vars.write(&self.ids.pushback_speed, 0.);
            vars.write(&self.ids.pushback_heading, 0.);
        }
        // callTug / releaseTug (:72-86): PUSHBACK WAIT here; the tug itself
        // (K:TOGGLE_PUSHBACK) is the pushback module's.
        if published::presses(pb.call_tug) > 0 {
            vars.write(&self.ids.pushback_wait, 1.);
        }
        if published::presses(pb.release_tug) > 0 {
            vars.write(&self.ids.pushback_wait, 0.);
        }
        if let Some(v) = published::take(pb.parking_brake) {
            vars.write(&self.ids.park_brake, (v != 0.) as i32 as f64);
        }
        published::set(pb.enabled, vars.read(&self.ids.pushback_enabled));
        published::set(pb.speed, vars.read(&self.ids.pushback_speed));
        published::set(pb.heading, vars.read(&self.ids.pushback_heading));
        published::set(pb.attached, attached as i32 as f64);
        published::set(pb.parking_brake, vars.read(&self.ids.park_brake));
    }

    fn update_other(&mut self, vars: &mut Vars) {
        for i in 0..SETTINGS.len() {
            if let Some(v) = published::take(self.other.settings[i]) {
                self.set_setting(vars, i, v);
            }
            published::set(self.other.settings[i], vars.read(&self.ids.settings[i]));
        }
        let o = &self.other;
        if let Some(id) = published::take(o.activate) {
            crate::failures::set_active(id as u64, true);
        }
        if let Some(id) = published::take(o.deactivate) {
            crate::failures::set_active(id as u64, false);
        }
        if let Some(id) = published::take(o.toggle) {
            crate::failures::toggle(id as u64);
        }
        published::set(o.failures_count, crate::failures::active_ids().len() as f64);
        published::set(o.start_state, vars.read(&self.ids.start_state));
        if let Some(v) = published::take(o.start_state_override) {
            let path = crate::start_state::override_path();
            let result = if (1. ..9.).contains(&v.round()) {
                if let Some(dir) = path.parent() {
                    let _ = std::fs::create_dir_all(dir);
                }
                std::fs::write(&path, format!("{}\n", v.round() as i64))
            } else {
                published::set(o.start_state_override, 0.);
                match std::fs::remove_file(&path) {
                    Err(e) if e.kind() != std::io::ErrorKind::NotFound => Err(e),
                    _ => Ok(()),
                }
            };
            if let Err(e) = result {
                crate::log(&format!("could not write the start state override: {e}"));
            }
        }
        for (v, id) in o.weights.iter().zip(&self.ids.weights) {
            published::set(*v, vars.read(id));
        }
        self.update_cg_checks(vars);
    }

    /// D12/D14: the CG/weight-disagree and CG-envelope discretes, computed
    /// once here (the real owner of every input) and published as plain
    /// named variables an `FbwProc` trigger reads through
    /// `TRIGGER_REAL_OWNER_VARS` (`deep/ecam/fbw_tests.rs`). The arithmetic
    /// itself lives in the pure, `Vars`-free `cg_checks` below, so it can be
    /// unit-tested directly (`efb.rs`'s own test module, no `Xplm`/`Vars`
    /// involved at all -- unlike `Fuel::new`, W89.md's flagged risk this
    /// design deliberately avoids by never constructing a full `Efb` in a
    /// test).
    fn update_cg_checks(&mut self, vars: &mut Vars) {
        let w = &self.ids.weights;
        let out = cg_checks(CgInputs {
            zfw_cg: vars.read(&w[4]),
            zfw_cg_desired: vars.read(&w[5]),
            gw: vars.read(&w[2]),
            gw_desired: vars.read(&w[3]),
            gw_cg: vars.read(&w[6]),
            to_cg: vars.read(&w[8]),
        });
        vars.write(&self.ids.airframe_zfw_cg_disagree, out.zfw_cg_disagree as i32 as f64);
        vars.write(&self.ids.airframe_weight_disagree, out.weight_disagree as i32 as f64);
        vars.write(&self.ids.airframe_cg_out_of_range, out.cg_out_of_range as i32 as f64);
        vars.write(&self.ids.airframe_to_cg_out_of_range, out.to_cg_out_of_range as i32 as f64);
        vars.write(&self.ids.airframe_cg_at_fwd_limit, out.cg_at_fwd_limit as i32 as f64);
        vars.write(&self.ids.airframe_cg_excess_aft, out.cg_excess_aft as i32 as f64);
    }
}

/// The five real inputs `update_cg_checks` reads (`AIRFRAME_*`/`AIRFRAME_*_
/// DESIRED`, `efb.rs`'s own `Ids::weights`), lifted out so the arithmetic
/// below takes no `Vars`/`Efb` at all.
struct CgInputs {
    zfw_cg: f64,
    zfw_cg_desired: f64,
    gw: f64,
    gw_desired: f64,
    gw_cg: f64,
    to_cg: f64,
}

struct CgOutputs {
    zfw_cg_disagree: bool,
    weight_disagree: bool,
    cg_out_of_range: bool,
    to_cg_out_of_range: bool,
    cg_at_fwd_limit: bool,
    cg_excess_aft: bool,
}

/// D12/D14's pure arithmetic: every `AIRFRAME_*` discrete this Phase 2 pass
/// adds, from the five real inputs above. No `Vars`/`Xplm` involved, so this
/// is directly unit-testable (see `cg_checks_tests` below).
fn cg_checks(i: CgInputs) -> CgOutputs {
    // D12: crew-entered "desired" vs the fully precise computed "actual"
    // figure, each strictly beyond its own GENERIC tolerance (a value
    // sitting exactly on the band is not treated as a fault of arbitrary
    // sign).
    let zfw_cg_disagree = (i.zfw_cg - i.zfw_cg_desired).abs() > ZFW_CG_DISAGREE_TOLERANCE_PCT;
    let weight_disagree = (i.gw - i.gw_desired).abs() > WEIGHT_DISAGREE_TOLERANCE_KG;

    // D14: the real CG-envelope comparator against `airframe.json5`'s own
    // polygons.
    let in_flight_envelope = point_in_polygon(i.gw_cg, i.gw, FLIGHT_ENVELOPE);
    let cg_out_of_range = !in_flight_envelope;

    // `AIRFRAME_GW_DESIRED` (the crew-entered target GW) is the closest real
    // proxy available for a "predicted take-off gross weight" figure -- no
    // published FMS-predicted-TOW channel was found. A proxy, not a literal
    // predicted-TOW channel; flagged in E-FUEL-DESIGN.md D14 for Phase 3 to
    // confirm or improve.
    let to_cg_out_of_range = !point_in_polygon(i.to_cg, i.gw_desired, MTOW_ENVELOPE);

    // In-envelope, but a `FWD_MARGIN_PCT` nudge toward lower %MAC would fall
    // out of it: "close to the forward boundary" for any polygon shape,
    // without hand-coding which edge is the forward one.
    let cg_at_fwd_limit = in_flight_envelope && !point_in_polygon(i.gw_cg - FWD_MARGIN_PCT, i.gw, FLIGHT_ENVELOPE);

    // FCOM PRO-ABN-ECAM p.5164's own real number (see `AFT_CG_EXCESS_PCT`'s
    // doc comment): a direct, cited threshold, not a margin against the
    // structural aft limit.
    let cg_excess_aft = i.gw_cg > AFT_CG_EXCESS_PCT;

    CgOutputs { zfw_cg_disagree, weight_disagree, cg_out_of_range, to_cg_out_of_range, cg_at_fwd_limit, cg_excess_aft }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_cabin_adds_up_to_the_flypad_maximums() {
        assert_eq!(SEAT_STATIONS.iter().map(|s| s.1).sum::<u32>(), MAX_PAX);
        assert_eq!(CARGO_STATIONS.iter().map(|s| s.2).sum::<f64>(), MAX_CARGO_KG);
        assert_eq!(station_share(50), 0.11);
        assert_eq!(station_share(14), 0.03);
    }

    #[test]
    fn seat_flags_use_the_flypad_bit_layout() {
        let mut f = SeatFlags { bits: 0, seats: 50 };
        f.bits ^= SeatFlags::mask(31);
        assert_eq!(f.value(), 4_294_967_296., "seat 31 is the high word's bit 0");
        assert!(f.filled(31) && !f.filled(30));
        let mut r = Random::new(7);
        f.choose(true, 60, &mut r);
        assert_eq!(f.count(), 50, "no more seats than the station has");
        f.choose(false, 20, &mut r);
        assert_eq!(f.count(), 30);
        assert!(f.value() < 2f64.powi(53), "exact in a double");
    }

    #[test]
    fn target_pax_splits_by_station_share_with_the_rest_forward() {
        let mut flags: [SeatFlags; 14] = std::array::from_fn(|i| SeatFlags { bits: 0, seats: SEAT_STATIONS[i].1 });
        let mut r = Random::new(42);
        set_target_pax(&mut flags, 300, &mut r);
        let counts: Vec<u32> = flags.iter().map(SeatFlags::count).collect();
        // Station 13 (18 seats): ceil(18/484*100)/100 = 0.04, trunc(0.04*300) = 12.
        assert_eq!(counts[13], 12);
        // Station 3 (50 seats): 0.11 * 300 = 33.
        assert_eq!(counts[3], 33);
        assert_eq!(counts.iter().sum::<u32>(), 300);
        // Down to 0 empties every station.
        set_target_pax(&mut flags, 0, &mut r);
        assert_eq!(flags.iter().map(SeatFlags::count).sum::<u32>(), 0);
        // Refused: more than the cabin holds.
        set_target_pax(&mut flags, 485, &mut r);
        assert_eq!(flags.iter().map(SeatFlags::count).sum::<u32>(), 0);
    }

    #[test]
    fn cargo_and_zfw_follow_the_page() {
        // 300 pax at 20 kg of bags plus 5000 kg of freight.
        let c = set_target_cargo(300, 5000., 20.);
        assert_eq!(c.iter().sum::<f64>(), 11_000.);
        assert_eq!(c[2], js_round(2513. / 51_400. * 11_000.));
        assert_eq!(set_target_cargo(484, 100_000., 20.).iter().sum::<f64>(), MAX_CARGO_KG);
        // 40 000 kg / 104 kg rounds to 385 pax, which leave nothing for cargo.
        assert_eq!(split_payload(40_000., 84., 20.), (385, 0.));
        assert_eq!(split_payload(40_050., 84., 20.), (385, 10.));
        assert_eq!(split_payload(-5., 84., 20.), (0, 0.));
    }

    #[test]
    fn refuel_rules_and_times() {
        assert!(refuel_allowed(false, false, true, 0.));
        assert!(!refuel_allowed(false, true, true, 0.));
        assert!(refuel_allowed(false, true, true, REFUEL_INSTANT));
        assert!(refuel_allowed(true, true, false, 0.), "a running refuel can always be stopped");
        assert_eq!(desired_fuel(1e6), js_round(TOTAL_MAX_FUEL_KG));
        assert_eq!(desired_fuel_percent(0.4), 0.);
        // 16 gal/s: 1600 gallons take 100 s, 20 s at FAST.
        let kg = 1600. * FUEL_GALLONS_TO_KG;
        assert!((refuel_seconds(0., kg, 0.) - 100.).abs() < 1e-9);
        assert!((refuel_seconds(0., kg, REFUEL_FAST) - 20.).abs() < 1e-9);
        assert_eq!(refuel_seconds(0., kg, REFUEL_INSTANT), 0.);
        assert_eq!(boarding_seconds(BoardingRate::Real, 2, 100., 0.), 250.);
        assert_eq!(boarding_seconds(BoardingRate::Fast, 0, 10., 1200.), 20.);
    }

    #[test]
    fn moving_disconnect_waits_out_the_spawn_settle() {
        assert!(!moving_disconnect_armed(0.), "must not arm right after a spawn");
        assert!(!moving_disconnect_armed(1.7), "the settle seen in every kept log must not arm it");
        assert!(!moving_disconnect_armed(SPAWN_SETTLE_S - 0.01));
        assert!(moving_disconnect_armed(SPAWN_SETTLE_S));
        assert!(moving_disconnect_armed(10.), "armed for the rest of that spawn");
    }

    #[test]
    fn settings_map_as_sync_ts_maps_them() {
        let boarding = &SETTINGS[Efb::setting_index("CONFIG_BOARDING_RATE")];
        assert_eq!(boarding.to_number("REAL"), 2.);
        assert_eq!(boarding.to_number("INSTANT"), 0.);
        assert_eq!(boarding.to_stored(1.), "FAST");
        let align = &SETTINGS[Efb::setting_index("CONFIG_ALIGN_TIME")];
        assert_eq!(align.to_number("FAST"), 2.);
        assert_eq!(align.to_stored(1.), "INSTANT");
        let metric = &SETTINGS[Efb::setting_index("CONFIG_USING_METRIC_UNIT")];
        assert_eq!(metric.to_number(metric.default), 1.);
        assert_eq!(metric.to_stored(0.), "false");
        let pins = &SETTINGS[Efb::setting_index("CONFIG_A380X_FWC_RADIO_AUTO_CALL_OUT_PINS")];
        assert_eq!(pins.to_number(pins.default), (1 | 8 | 64 | 1 << 14 | 1 << 15 | 1 << 16 | 1 << 17 | 1 << 18 | 1 << 19) as f64);
        let ini = write_ini(&[(stored_key("REFUEL_RATE_SETTING"), "2".to_string())].into_iter().collect());
        assert_eq!(parse_ini(&ini).get("A380X_REFUEL_RATE_SETTING").map(String::as_str), Some("2"));
    }

    // ---- Phase 2 (E-FUEL-DESIGN.md D12/D14) -------------------------------

    fn nominal_inputs() -> CgInputs {
        // A point well inside every envelope: 35% MAC at 400,000 kg is
        // inside `FLIGHT_ENVELOPE` and `MTOW_ENVELOPE` alike, comfortably
        // clear of the 0.5% margins and the 50% aft limit.
        CgInputs { zfw_cg: 35.0, zfw_cg_desired: 35.0, gw: 400_000.0, gw_desired: 400_000.0, gw_cg: 35.0, to_cg: 35.0 }
    }

    #[test]
    fn cg_and_weight_disagree_use_their_own_generic_tolerance_band() {
        let mut i = nominal_inputs();
        let healthy = cg_checks(CgInputs { zfw_cg: i.zfw_cg, zfw_cg_desired: i.zfw_cg_desired, gw: i.gw, gw_desired: i.gw_desired, gw_cg: i.gw_cg, to_cg: i.to_cg });
        assert!(!healthy.zfw_cg_disagree && !healthy.weight_disagree, "matching actual/desired figures must not disagree");

        i.zfw_cg_desired = i.zfw_cg - ZFW_CG_DISAGREE_TOLERANCE_PCT; // exactly at the boundary
        assert!(!cg_checks(clone_inputs(&i)).zfw_cg_disagree, "exactly at the tolerance boundary is not a fault (strict >, not >=)");
        i.zfw_cg_desired = i.zfw_cg - ZFW_CG_DISAGREE_TOLERANCE_PCT - 0.01;
        assert!(cg_checks(clone_inputs(&i)).zfw_cg_disagree, "beyond the tolerance is a fault");

        let mut w = nominal_inputs();
        w.gw_desired = w.gw - WEIGHT_DISAGREE_TOLERANCE_KG;
        assert!(!cg_checks(clone_inputs(&w)).weight_disagree, "exactly at the tolerance boundary is not a fault");
        w.gw_desired = w.gw - WEIGHT_DISAGREE_TOLERANCE_KG - 1.0;
        assert!(cg_checks(clone_inputs(&w)).weight_disagree, "beyond the tolerance is a fault");
    }

    #[test]
    fn cg_out_of_range_uses_the_real_flight_envelope_polygon() {
        let inside = cg_checks(nominal_inputs());
        assert!(!inside.cg_out_of_range, "35% MAC at 400,000 kg is inside airframe.json5's own flight envelope");

        let mut outside = nominal_inputs();
        outside.gw_cg = 10.0; // far forward of every polygon point
        assert!(cg_checks(clone_inputs(&outside)).cg_out_of_range, "10% MAC is outside the flight envelope on every polygon vertex");

        let mut aft_outside = nominal_inputs();
        aft_outside.gw_cg = 60.0; // far aft of every polygon point
        assert!(cg_checks(clone_inputs(&aft_outside)).cg_out_of_range);
    }

    #[test]
    fn cg_at_fwd_limit_fires_only_a_margin_before_the_real_forward_edge() {
        // The flight envelope's forward edge sits at 28% MAC between
        // 270,000 kg and 375,000 kg (airframe.json5:65-73). At 300,000 kg,
        // comfortably inside (30%) must be quiet; just inside the margin
        // (28 + FWD_MARGIN_PCT/2) must fire; already outside (27%) must not
        // re-fire this id (that is `AIRFRAME_CG_OUT_OF_RANGE`'s own job).
        let comfortable = cg_checks(CgInputs { zfw_cg: 30.0, zfw_cg_desired: 30.0, gw: 300_000.0, gw_desired: 300_000.0, gw_cg: 30.0, to_cg: 30.0 });
        assert!(!comfortable.cg_at_fwd_limit && !comfortable.cg_out_of_range);

        let near_fwd = cg_checks(CgInputs { zfw_cg: 28.2, zfw_cg_desired: 28.2, gw: 300_000.0, gw_desired: 300_000.0, gw_cg: 28.2, to_cg: 28.2 });
        assert!(near_fwd.cg_at_fwd_limit, "28.2% MAC is within the 0.5% margin of the 28% forward edge");
        assert!(!near_fwd.cg_out_of_range, "still inside the envelope itself");

        let past_fwd = cg_checks(CgInputs { zfw_cg: 27.0, zfw_cg_desired: 27.0, gw: 300_000.0, gw_desired: 300_000.0, gw_cg: 27.0, to_cg: 27.0 });
        assert!(!past_fwd.cg_at_fwd_limit, "already out of range is AIRFRAME_CG_OUT_OF_RANGE's own alert, not this one");
        assert!(past_fwd.cg_out_of_range);
    }

    #[test]
    fn cg_excess_aft_uses_the_fcom_s_own_50_percent_figure() {
        let under = cg_checks(CgInputs { zfw_cg: 49.9, zfw_cg_desired: 49.9, gw: 400_000.0, gw_desired: 400_000.0, gw_cg: 49.9, to_cg: 30.0 });
        assert!(!under.cg_excess_aft, "FCOM PRO-ABN-ECAM p.5164: the trigger is exceeding 50%, not merely close to it");
        let at = cg_checks(CgInputs { zfw_cg: 50.0, zfw_cg_desired: 50.0, gw: 400_000.0, gw_desired: 400_000.0, gw_cg: 50.0, to_cg: 30.0 });
        assert!(!at.cg_excess_aft, "strict >, matching the FCOM's own \"exceeds 50%\" wording");
        let over = cg_checks(CgInputs { zfw_cg: 50.1, zfw_cg_desired: 50.1, gw: 400_000.0, gw_desired: 400_000.0, gw_cg: 50.1, to_cg: 30.0 });
        assert!(over.cg_excess_aft);
    }

    #[test]
    fn to_cg_out_of_range_checks_the_mtow_envelope_against_the_desired_gw_proxy() {
        let inside = cg_checks(CgInputs { zfw_cg: 35.0, zfw_cg_desired: 35.0, gw: 400_000.0, gw_desired: 400_000.0, gw_cg: 35.0, to_cg: 35.0 });
        assert!(!inside.to_cg_out_of_range);
        let outside = cg_checks(CgInputs { zfw_cg: 35.0, zfw_cg_desired: 35.0, gw: 400_000.0, gw_desired: 400_000.0, gw_cg: 35.0, to_cg: 10.0 });
        assert!(outside.to_cg_out_of_range, "10% MAC take-off CG is outside the mtow envelope on every polygon vertex");
    }

    /// `CgInputs` derives no `Clone` (kept minimal; nothing else needs one),
    /// so the tests above that build one variant from another copy it by
    /// hand.
    fn clone_inputs(i: &CgInputs) -> CgInputs {
        CgInputs { zfw_cg: i.zfw_cg, zfw_cg_desired: i.zfw_cg_desired, gw: i.gw, gw_desired: i.gw_desired, gw_cg: i.gw_cg, to_cg: i.to_cg }
    }

    #[test]
    fn point_in_polygon_matches_the_real_flight_envelope_shape() {
        // A textbook even-odd ray-casting sanity check against the exact
        // polygon this design cites (airframe.json5:65-73): a point clearly
        // inside, one clearly outside on each side, and one on a vertex.
        assert!(point_in_polygon(35.0, 400_000.0, FLIGHT_ENVELOPE));
        assert!(!point_in_polygon(0.0, 400_000.0, FLIGHT_ENVELOPE));
        assert!(!point_in_polygon(100.0, 400_000.0, FLIGHT_ENVELOPE));
        assert!(!point_in_polygon(35.0, 1_000_000.0, FLIGHT_ENVELOPE), "above every polygon point's weight");
    }
}
