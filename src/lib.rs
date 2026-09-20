//! FlyByWire's A380X systems simulation, running in X-Plane 12.
//!
//! FlyByWire's systems never talk to MSFS directly: they read and write named
//! variables through [`SimulatorReaderWriter`] and ask a [`VariableRegistry`]
//! for the identifier of each name. Their MSFS glue (the `a380_systems_wasm`
//! crate) is what binds those names to SimConnect; this plugin is the same
//! layer for X-Plane, so their systems code compiles and runs unchanged.
//!
//! Every variable becomes an X-Plane dataref under `fbw/`, readable and
//! writable by the cockpit, Lua and other plugins. The state the simulation
//! expects each tick (airspeed, attitude, weight and so on) is read from
//! X-Plane's own datarefs and converted to the units FlyByWire reads in:
//! knots, feet, feet per second squared, degrees Celsius, pounds, degrees.
//!
//! The X-Plane SDK is not needed to build this: XPLM's functions are looked
//! up in the already-loaded `XPLM_64.dll` at start-up.

use std::collections::HashMap;
use std::ffi::{c_char, c_int, c_void, CString};
use std::time::Duration;

use a380_systems::A380;
use systems::simulation::{
    Simulation, SimulatorReaderWriter, StartState, VariableIdentifier, VariableRegistry,
};

mod afs_events;
pub mod remote;
pub mod xphfbw_bridge;
pub mod xphfbw_bridge_views;
// [slot modules: xphfbw_datarefs] The xphfbw/ status datarefs, its three
// commands, and the "XPHFBW settings" aircraft menu entry.
mod xphfbw_datarefs;
// The XPHFBW app's own settings (xphfbw.json): pub so app/src/settings.rs
// can use with_settings_lock (rule 8) and so agent A's start_systems can
// read systems_out_of_process().
pub mod app_settings;
mod big_stack;
mod state_dump;
mod engine_commands;
// hyperrealism.md physics workstream 5 (fluids): hydraulics/fuel/oxygen
// physics not already covered by FlyByWire's own ported systems. Shared
// across workstreams; each adds its own `physics::<area>` submodule.
pub mod invariants;
pub mod physics;
pub mod deep;
mod fbw_computers;
mod fbw_types;
mod prim;
mod fadec;
mod fbw_controllers;
mod fuel;
// The network and transfer logic carry MSFS's full API; the plugin uses part.
#[allow(dead_code)]
mod fuel_network;
#[allow(dead_code)]
mod fuel_transfer;
// [slot modules: circuits] The general systems.cfg circuit model (buses,
// connection switch, breaker) fuel.rs and lights.rs are both built on.
pub mod circuits;
pub mod breakers;
// [slot modules: lights] Exterior/cockpit/cabin light and window heat/wiper
// circuits, powered from FlyByWire's buses through circuits.rs.
mod lights;
// [slot modules: oxygen] Crew and passenger oxygen quantity, consumption,
// low-pressure caution and mask deployment.
mod oxygen;
mod panel;
mod study;
mod throttle;
pub mod xp;
// [slot modules: flight_controls] X-Plane's surfaces follow FlyByWire's actuators.
mod flight_controls;
mod handling;
// [slot modules: sensors] X-Plane's state for the simulator variables the systems read.
mod sensors;
pub mod aspects;
mod correctness;
pub mod failures;
// Shared setup for cross-system scenario tests (three-way failure
// intersections): `scenarios::reset_global_state()` clears every module's
// process-global failure/wear/breaker-request state between tests. See the
// module doc for why this is needed and how to extend it.
#[cfg(any(test, feature = "test-support"))]
pub mod scenarios;
// Cross-system emergence tests over this plugin's own physics modules
// (fuel, engine, electrical, hydraulics, breakers), ticked together offline
// against `Xplm::dummy()`/`Vars` -- no live X-Plane process. See the module
// doc for the seam and how to add scenarios.
#[cfg(test)]
mod offline_harness;
// The engine/fuel chain of `Plugin::tick`, runnable against `Xplm::dummy()`
// for the offline emulator (see the module doc).
#[cfg(any(test, feature = "test-support"))]
pub mod offline_chain;
pub mod mel;
// Damageable components and their physical parameters: failures as
// perturbations, combined per parameter (see the module doc).
pub mod components;
// The user's own operator MEL, parsed by tools/parse_mel.py (never bundled).
pub mod mel_catalog;
// Persistent component wear (hot hours, cycles, thermal-stress integral,
// degradation fraction), keyed by component id -- see the module doc.
pub mod wear;
// hyperrealism.md physics workstream 6 (failures, damage, MEL, persistence):
// the MTBF random-failure engine and the on-disk airframe state.
pub mod random_failures;
// The arm-by-time/altitude/speed/flight-phase scripted trigger engine
// (not yet ticked from `Plugin::tick` -- see the module doc for the one
// remaining integration call).
pub mod scripted_failures;
mod persistence;
pub mod start_state;
// [slot modules: mapdata] Terrain, traffic and airport map data for the displays.
#[allow(dead_code)]
mod mapdata;
// [slot modules: radios] NAV, ADF and COM receivers, their key events, the manual LS tuning.
mod published;
mod radios;
// [slot modules: surveillance] The SURV panel: transponder/TCAS system
// select, TCAS TA ONLY/range, WXR/TAWS lane select, G/S mode inhibit.
pub(crate) mod surveillance;
// [slot modules: doors] The interactive points and X-Plane's doors.
mod doors;
// [slot modules: weight_balance] Payload mass and MSFS's centre of gravity on X-Plane.
mod weight_balance;
// [slot modules: efb] The fbw/efb/ interface for a third-party EFB.
mod efb;
// [slot modules: sound] FlyByWire's Wwise sounds played through X-Plane's sound API.
mod sound;
// [slot modules: extra_backend] FlyByWire's extra backend: lighting and aircraft presets, pushback.
mod extra_backend;
// [slot modules: fcdc] FlyByWire's two FCDCs and the spoiler lever LVars (FlyByWireInterface updateFcdc, updateSpoilers).
mod extra_backend_fcdc;
// [slot modules: extra_backend_fbw] The rest of FlyByWireInterface.cpp: sidestick/
// pedal/tracking, the reverser force applied to X-Plane's velocity, LightSync
// inputs, sim rate limiting and the performance warning.
mod extra_backend_fbw;
// [slot modules: key_events] MSFS key events FlyByWire sends, with MSFS's effects.
mod key_events;
// The JavaScript and TypeScript engine, for FlyByWire's TypeScript.
#[cfg(feature = "js")]
mod js;
#[cfg(feature = "js")]
mod js_bridge;
#[cfg(feature = "js")]
mod js_worker;
// The plugin's per-frame driver for XPHFBW's shared-memory bridge
// (docs/briefs/xphfbw-js-bridge.md): resolves and publishes slots the same
// way js_bridge's QuickJS host does, so it needs the same feature.
#[cfg(feature = "js")]
mod xphfbw_host;
mod perf;
#[cfg(all(test, feature = "js"))]
mod xphfbw_harness;
// [slot modules: display] The cockpit screens FlyByWire's instruments draw on.
#[cfg(feature = "js")]
mod display;
// [slot modules: navdata] MSFS's facility database from X-Plane's navigation data.
#[cfg(feature = "js")]
mod navdata;
// [slot modules: oans] Local AMDB airport map data for the OANS (ND/MFD).
#[cfg(feature = "js")]
mod oans;
// [slot modules: wxr] Weather radar on the ND, from X-Plane's real weather.
#[allow(dead_code)]
mod wxr;
// [slot modules: ecam_patches] ECAM/FWS and instrument SourcePatches (D:
// ECAM/instruments) not owned by another module's own source_patches().
#[cfg(feature = "js")]
mod ecam_patches;
mod xplane_mirror; // FBW's own state onto X-Plane's standard datarefs, for third-party add-ons.

use xp::{DataRef, Xplm};

/// Minimal, cfg-gated pub surface for the offline `emulator` crate
/// (D:\fbw-xp-systems\emulator, a separate crate, not a dependent of this
/// one): re-exports of otherwise crate-private modules that were already
/// written to run against `VariableRegistry + SimulatorReaderWriter`
/// generically, or directly against no `Vars`/`Xplm` at all, or (breakers,
/// physics::electrical/air/hydraulics) against the concrete `Vars` above,
/// itself made `pub` for the same reason. No behaviour changes anywhere;
/// only visibility. See the `test-support` feature's doc comment in
/// Cargo.toml.
#[cfg(feature = "test-support")]
pub mod test_support {
    pub use crate::{
        aspects, breakers, circuits, components, failures, invariants, mel, offline_chain, physics, random_failures, scenarios,
        scripted_failures, start_state, wear, xp, Vars,
    };
}

/// What the flight loop leaves behind for the panel to read. The panel runs
/// on its own thread and never touches X-Plane, so this is the only thing
/// crossing between them.
#[derive(Default)]
pub(crate) struct Snapshot {
    pub time: f64,
    pub ticks: u64,
    pub names: Vec<String>,
    pub values: Vec<f64>,
    /// Where each value comes from: 0 nothing yet, 1 an X-Plane dataref read
    /// every tick, 2 the systems themselves. The panels show this rather than
    /// letting an unfed variable pass for a measurement.
    pub sources: Vec<u8>,
    /// The `fbw/` dataref each variable is published as.
    pub datarefs: Vec<String>,
    /// Where each name sits in the lists above, rebuilt only when a variable
    /// is registered, so a panel never searches two thousand names a frame.
    pub index: HashMap<String, usize>,
    /// Counts up whenever the names change, so a panel can keep what it
    /// looked up until then.
    pub generation: u64,
}

impl Snapshot {
    /// Where a variable sits, whichever way its name is written: with or
    /// without FlyByWire's prefix.
    pub fn find(&self, name: &str) -> Option<usize> {
        if let Some(&i) = self.index.get(name) {
            return Some(i);
        }
        match name.strip_prefix(NAME_PREFIX) {
            Some(bare) => self.index.get(bare).copied(),
            None => self.index.get(&format!("{NAME_PREFIX}{name}")).copied(),
        }
    }
}

/// Where a variable's value comes from, as the snapshot records it.
const NO_SOURCE: u8 = 0;
const FROM_XPLANE: u8 = 1;
const FROM_SYSTEMS: u8 = 2;

/// The X-Plane dataref feeding a variable, if one is mapped to it.
pub(crate) fn source_dataref(name: &str) -> Option<&'static str> {
    mapping(name).map(|m| m.0)
}

pub(crate) fn snapshot() -> &'static std::sync::Mutex<Snapshot> {
    static SNAPSHOT: std::sync::OnceLock<std::sync::Mutex<Snapshot>> = std::sync::OnceLock::new();
    SNAPSHOT.get_or_init(Default::default)
}

/// Where the panel listens. X-Plane's own plugins leave this range alone.
// 8380 is SimBridge's (FlyByWire's own local server), which runs alongside.
const PANEL_PORT: u16 = 8390;

/// Identifier types, as FlyByWire's MSFS layer numbers them: variables it
/// asks for unprefixed are the simulator's own (`AIRSPEED INDICATED`),
/// the rest are the aircraft's named variables (`A32NX_...`).
const SIMULATOR: usize = 0;
const NAMED: usize = 1;

/// FlyByWire prefixes the A380X's named variables with this, as its MSFS
/// build does, so a variable is called the same here as in their code.
const NAME_PREFIX: &str = "A32NX_";

/// One variable: its value, its dataref, and where its value comes from.
struct Slot {
    value: f64,
    /// Set from this X-Plane dataref every tick, instead of from the systems.
    input: Option<Input>,
    /// Registered as `fbw/...` so anything in X-Plane can read or write it.
    published: bool,
    /// The `fbw/` dataref it was published as, once it has one.
    dataref: String,
    /// Whether the plugin fills it from X-Plane's own state each tick, either
    /// from a mapped dataref or from a figure worked out from several.
    from_xplane: bool,
    /// Whether the systems have ever written it. A variable nothing writes
    /// and nothing feeds is holding its start value, not a measurement.
    written: bool,
    /// This variable's physical bound (if any), worked out once from its
    /// name in [`Vars::add`] so every `write`/`write_from_xplane` is a
    /// cheap precomputed-bound check, not a string match. See
    /// `invariants::bound_for`.
    bound: invariants::Bound,
}

/// An X-Plane dataref feeding one variable, with the conversion into the unit
/// FlyByWire reads it in.
struct Input {
    dataref: DataRef,
    kind: Kind,
    convert: fn(f64) -> f64,
}

#[derive(Clone, Copy)]
enum Kind {
    Float,
    Double,
    Int,
}

/// The variables, standing in for both of FlyByWire's simulator traits.
///
/// The struct and the handful of inherent methods marked `pub` below are
/// `pub` unconditionally (cheap; no behaviour change), but only reachable
/// from outside this crate through the feature-gated `test_support`
/// re-export module further down, which is the actual gate: lets the
/// out-of-tree `emulator` crate build the same `Vars` the real `Plugin`
/// uses, off `Xplm::dummy()`, with no live X-Plane process.
pub struct Vars {
    xplm: &'static Xplm,
    /// Slots per identifier type, indexed by the identifier's index.
    slots: [Vec<Slot>; 2],
    names: [Vec<String>; 2],
    ids: HashMap<String, VariableIdentifier>,
    /// Dataref names already handed to X-Plane.
    taken: std::collections::HashSet<String>,
    /// Dataref names, kept alive for as long as X-Plane holds them.
    kept: Vec<CString>,
    /// How many of each type are already published, so a tick with nothing
    /// new does not walk every variable.
    published_upto: [usize; 2],
}

impl Vars {
    pub fn new(xplm: &'static Xplm) -> Self {
        Self {
            xplm,
            slots: [Vec::new(), Vec::new()],
            names: [Vec::new(), Vec::new()],
            ids: HashMap::new(),
            taken: std::collections::HashSet::new(),
            kept: Vec::new(),
            published_upto: [0, 0],
        }
    }

    fn add(&mut self, name: String, kind: usize) -> VariableIdentifier {
        if let Some(id) = self.ids.get(&name) {
            return *id;
        }
        // Identifiers count up from the first of their type.
        let mut id = VariableIdentifier::new(kind);
        for _ in 0..self.slots[kind].len() {
            id = id.next();
        }
        let input = (kind == SIMULATOR).then(|| self.input_for(&name)).flatten();
        self.slots[kind].push(Slot {
            value: 0.,
            input,
            published: false,
            dataref: String::new(),
            from_xplane: false,
            written: false,
            bound: invariants::bound_for(&name),
        });
        self.names[kind].push(name.clone());
        self.ids.insert(name, id);
        id
    }

    fn slot(&mut self, id: &VariableIdentifier) -> Option<&mut Slot> {
        self.slots
            .get_mut(id.identifier_type())?
            .get_mut(id.identifier_index())
    }

    /// Whether `id` is a named (`L:`) variable nothing has ever written --
    /// still holding the zero [`Self::add`] gave it, not a measurement any
    /// view has published. `js_bridge.rs`'s `VarsHost::read` logs the first
    /// such read once per name, so a missing provider (a view that never
    /// runs, a wrong var name) shows up in the log instead of silently
    /// reading 0 forever.
    pub(crate) fn is_unwritten_named(&mut self, id: &VariableIdentifier) -> bool {
        id.identifier_type() == NAMED && self.slot(id).is_some_and(|s| !s.written)
    }

    /// Where a simulator variable comes from in X-Plane. Unmapped ones keep
    /// whatever their dataref is set to, which is how the cockpit and Lua
    /// drive the systems.
    fn input_for(&self, name: &str) -> Option<Input> {
        let (dataref, kind, convert) = mapping(name)?;
        let dataref = self.xplm.find(dataref)?;
        Some(Input {
            dataref,
            kind,
            convert,
        })
    }

    /// Register a variable by its own full name (as the cockpit's bindings
    /// name it): a simulator variable if it is one, otherwise an aircraft
    /// variable exactly so named.
    /// `cockpit_variables.txt` (written by the converter next to the
    /// plugin): "name" or "name<TAB>start value" per line. Registers every
    /// name; a start value only applies to a variable nothing has written
    /// yet (the cockpit's switch positions at spawn, e.g. each engine pump
    /// DISC pushbutton in AUTO). Returns how many were registered. Public so
    /// the offline emulator seeds its switches exactly as the plugin does.
    pub fn register_cockpit_variables_text(&mut self, text: &str) -> usize {
        let mut n = 0;
        for line in text.lines().map(str::trim).filter(|l| !l.is_empty()) {
            let (name, start) = match line.split_once('\t') {
                Some((name, v)) => (name, v.trim().parse::<f64>().ok()),
                None => (line, None),
            };
            let id = self.register_named(name);
            if let Some(v) = start {
                if self.slot(&id).is_some_and(|s| !s.written) {
                    SimulatorReaderWriter::write(self, &id, v);
                }
            }
            n += 1;
        }
        n
    }

    pub fn register_named(&mut self, name: &str) -> VariableIdentifier {
        if is_simulator_variable(name) {
            self.add(name.to_string(), SIMULATOR)
        } else {
            self.add(name.to_string(), NAMED)
        }
    }

    /// Every variable registered so far, with its current value: the
    /// `test-support` full-snapshot escape hatch for the out-of-tree
    /// `emulator` crate (its own catalogue functions cover breakers/
    /// failures/circuits/controls; this is for everything else, e.g. an
    /// invariant sweep over every FlyByWire-published variable at once,
    /// the same shape `start_state.rs`'s own cold-and-dark test's
    /// `check_every_variable` uses against `TestVars::index`/`values`).
    /// Every variable's value, in [`Self::snapshot_all`]'s order, without
    /// copying the names: what a per-tick check needs (the offline
    /// battery reads this every tick; names only when something is found).
    pub fn values(&self) -> Vec<f64> {
        [SIMULATOR, NAMED].into_iter().flat_map(|kind| self.slots[kind].iter().map(|s| s.value)).collect()
    }

    /// How many variables exist; [`Self::values`] is this long.
    pub fn len(&self) -> usize {
        self.slots[SIMULATOR].len() + self.slots[NAMED].len()
    }

    pub fn snapshot_all(&self) -> Vec<(String, f64)> {
        [SIMULATOR, NAMED]
            .into_iter()
            .flat_map(|kind| self.names[kind].iter().zip(self.slots[kind].iter()).map(|(n, s)| (n.clone(), s.value)))
            .collect()
    }

    /// Publish every variable X-Plane has not seen yet as `fbw/<name>`.
    fn publish(&mut self) -> usize {
        let mut published = 0;
        for kind in [SIMULATOR, NAMED] {
            let (from, to) = (self.published_upto[kind], self.slots[kind].len());
            if from == to {
                continue;
            }
            self.published_upto[kind] = to;
            for i in from..to {
                if self.slots[kind][i].published {
                    continue;
                }
                // Two variables can sanitise to one name ("WHEEL_RPM:1" and
                // "WHEEL_RPM_1" both lose the colon). X-Plane keeps whichever
                // came first and warns about the rest, so later ones are
                // numbered instead of being lost.
                let mut dataref = format!("fbw/{}", sanitise(&self.names[kind][i]));
                if !self.taken.insert(dataref.clone()) {
                    let mut n = 2;
                    while !self.taken.insert(format!("{dataref}_{n}")) {
                        n += 1;
                    }
                    dataref = format!("{dataref}_{n}");
                }
                let Ok(c) = CString::new(dataref.clone()) else { continue };
                // The refcon carries the slot: type in the high bits, index
                // in the low ones.
                let refcon = ((kind << 32) | i) as *mut c_void;
                self.xplm.publish(&c, refcon);
                self.kept.push(c);
                self.slots[kind][i].published = true;
                self.slots[kind][i].dataref = dataref;
                published += 1;
            }
        }
        published
    }

    /// Write a value the plugin took from X-Plane, and remember that this is
    /// where the variable's value comes from.
    fn write_from_xplane(&mut self, identifier: &VariableIdentifier, value: f64) {
        let kind = identifier.identifier_type();
        let index = identifier.identifier_index();
        let checked = match self.slots.get(kind).and_then(|s| s.get(index)) {
            Some(_) if invariants::is_arinc_sentinel(value) => value,
            Some(slot) => {
                let name = self.names.get(kind).and_then(|n| n.get(index)).map_or("<unknown>", String::as_str);
                invariants::check(name, value, slot.bound, "Vars::write_from_xplane")
            }
            None => return,
        };
        if let Some(slot) = self.slots.get_mut(kind).and_then(|s| s.get_mut(index)) {
            slot.value = checked;
            slot.from_xplane = true;
        }
    }

    /// Simulator variables (MSFS's own, e.g. "G FORCE") with no X-Plane
    /// input that nothing has written: in MSFS the simulator would give them a
    /// value, here they read 0.
    fn unfed_simulator_variables(&self) -> Vec<String> {
        self.slots[SIMULATOR]
            .iter()
            .zip(&self.names[SIMULATOR])
            .filter(|(slot, _)| slot.input.is_none() && !slot.written && !slot.from_xplane)
            .map(|(_, name)| name.clone())
            .collect()
    }

    /// Read the mapped X-Plane datarefs into their variables. These come
    /// from X-Plane itself (and, through it, third-party plugins and Lua)
    /// rather than from another FBW system, but a bad add-on or a corrupt
    /// dataref is just as capable of feeding a NaN or an impossible value
    /// into the substrate, so it gets the same guard.
    fn read_inputs(&mut self) {
        for (slot, name) in self.slots[SIMULATOR].iter_mut().zip(self.names[SIMULATOR].iter()) {
            if let Some(input) = &slot.input {
                let raw = match input.kind {
                    Kind::Float => self.xplm.get_f(input.dataref) as f64,
                    Kind::Double => self.xplm.get_d(input.dataref),
                    Kind::Int => self.xplm.get_i(input.dataref) as f64,
                };
                let converted = (input.convert)(raw);
                slot.value = if invariants::is_arinc_sentinel(converted) {
                    converted
                } else {
                    invariants::check(name, converted, slot.bound, "Vars::read_inputs")
                };
            }
        }
    }
}

impl VariableRegistry for Vars {
    fn get(&mut self, name: String) -> VariableIdentifier {
        // FlyByWire asks for MSFS's own variables ("AIRSPEED INDICATED") and
        // its aircraft variables ("ELEC_AC_1_BUS_IS_POWERED") the same way;
        // their MSFS glue tells them apart. Simulator variables keep their
        // plain name so the X-Plane inputs reach them.
        if is_simulator_variable(&name) {
            self.add(name, SIMULATOR)
        } else {
            // A name already carrying the prefix is the same variable (as in
            // `find`), never "A32NX_A32NX_...".
            let bare = name.strip_prefix(NAME_PREFIX).unwrap_or(&name);
            self.add(format!("{NAME_PREFIX}{bare}"), NAMED)
        }
    }

    fn get_unprefixed(&mut self, name: String) -> VariableIdentifier {
        self.add(name, SIMULATOR)
    }
}

impl SimulatorReaderWriter for Vars {
    fn read(&mut self, identifier: &VariableIdentifier) -> f64 {
        self.slot(identifier).map_or(0., |s| s.value)
    }

    fn write(&mut self, identifier: &VariableIdentifier, value: f64) {
        let kind = identifier.identifier_type();
        let index = identifier.identifier_index();
        let checked = match self.slots.get(kind).and_then(|s| s.get(index)) {
            Some(_) if invariants::is_arinc_sentinel(value) => value,
            Some(slot) => {
                let name = self.names.get(kind).and_then(|n| n.get(index)).map_or("<unknown>", String::as_str);
                invariants::check(name, value, slot.bound, "Vars::write")
            }
            None => return,
        };
        if let Some(slot) = self.slots.get_mut(kind).and_then(|s| s.get_mut(index)) {
            slot.value = checked;
            slot.written = true;
        }
    }
}

/// Whether a name is one of MSFS's simulator variables rather than one of the
/// aircraft's own. MSFS's are written with spaces ("TURB ENG CORRECTED N1:1");
/// the aircraft's never are. A few simulator names without spaces are known by
/// their X-Plane mapping.
fn is_simulator_variable(name: &str) -> bool {
    name.contains(' ') || mapping(name).is_some()
}

/// A dataref name X-Plane accepts: letters, digits, underscore and slash.
fn sanitise(name: &str) -> String {
    name.chars()
        .map(|c| if c.is_ascii_alphanumeric() || c == '_' || c == '/' { c } else { '_' })
        .collect()
}

// Conversions into the units FlyByWire's UpdateContext reads (see its
// `update`): knots for speeds, feet for lengths, feet per second squared for
// accelerations, degrees for angles, pounds for weight.
const MS_TO_KNOT: f64 = 1.943_844;
const M_TO_FT: f64 = 3.280_84;
const G_TO_FT_S2: f64 = 32.174;
const KG_TO_LB: f64 = 2.204_623;
const PA_TO_INHG: f64 = 0.000_295_3;
const PA_TO_MB: f64 = 0.01;
const KGM3_TO_SLUGFT3: f64 = 0.001_940_32;
const RAD_TO_DEG: f64 = 57.295_78;
const DEG_TO_RAD: f64 = 0.017_453_29;
const KGM2_TO_SLUGFT2: f64 = 0.737_562;

fn identity(v: f64) -> f64 {
    v
}

/// Standard pressure altitude in feet for a static pressure in pascals.
/// X-Plane has no pressure altitude dataref; the systems want the altitude
/// an altimeter set to 1013.25 would show.
fn pressure_altitude_ft(pascals: f64) -> f64 {
    (1. - (pascals / 101_325.).powf(0.190_284)) * 145_366.45
}

/// The X-Plane dataref behind each simulator variable FlyByWire reads.
/// Anything not listed keeps whatever its `fbw/` dataref holds.
fn mapping(name: &str) -> Option<(&'static str, Kind, fn(f64) -> f64)> {
    use Kind::{Double, Float, Int};
    Some(match name {
        "AMBIENT TEMPERATURE" => ("sim/weather/aircraft/temperature_ambient_deg_c", Float, identity),
        "AIRSPEED INDICATED" => ("sim/flightmodel/position/indicated_airspeed", Float, identity),
        "AIRSPEED TRUE" => ("sim/flightmodel/position/true_airspeed", Float, |v| v * MS_TO_KNOT),
        "GPS GROUND SPEED" => ("sim/flightmodel/position/groundspeed", Float, |v| v * MS_TO_KNOT),
        "AIRSPEED MACH" => ("sim/flightmodel/misc/machno", Float, identity),
        "PRESSURE ALTITUDE" => (
            "sim/weather/aircraft/barometer_current_pas",
            Float,
            pressure_altitude_ft,
        ),
        "AMBIENT PRESSURE" => (
            "sim/weather/aircraft/barometer_current_pas",
            Float,
            |v| v * PA_TO_INHG,
        ),
        "AMBIENT DENSITY" => ("sim/weather/rho", Float, |v| v * KGM3_TO_SLUGFT3),
        "SIM ON GROUND" => ("sim/flightmodel/failures/onground_any", Int, identity),
        // MSFS's camera enum: 2 cockpit, 3 external (PilotSeat.ts reads it
        // to tell whether the viewer sits in the flight deck).
        "CAMERA STATE" => ("sim/graphics/view/view_is_external", Int, |v| if v != 0. { 3. } else { 2. }),
        "GEAR HANDLE POSITION" => ("sim/cockpit2/controls/gear_handle_down", Int, identity),
        // Held as a ratio: FlyByWire reads it in "percent over 100".
        "STRUCTURAL ICE PCT" => ("sim/flightmodel/failures/frm_ice", Float, identity),
        "PLANE ALT ABOVE GROUND" => ("sim/flightmodel/position/y_agl", Float, |v| v * M_TO_FT),
        // fs-base-ui's SimPlane.js getAltitudeAboveGround (environment.js
        // [1841], the source `Simplane.getIsGrounded` -- and so
        // `FwsFlightPhases.ts`'s `groundImmediate` -- is built on) reads
        // this MSFS-only "CG-adjusted" variant, not the plain one above.
        // Unmapped, it kept its `fbw/`-published default of 0 forever (see
        // `input_for`'s doc comment above), which reads as permanently
        // grounded -- the opposite of the observed "FWC flight phase: 2 =>
        // 7" symptom, but leaves ground detection dead either way. X-Plane
        // has no separate CG-offset AGL dataref, so this aliases to the
        // same y_agl the uncorrected variant above uses.
        "PLANE ALT ABOVE GROUND MINUS CG" => ("sim/flightmodel/position/y_agl", Float, |v| v * M_TO_FT),
        "PLANE LATITUDE" => ("sim/flightmodel/position/latitude", Double, identity),
        "TOTAL WEIGHT" => ("sim/flightmodel/weight/m_total", Float, |v| v * KG_TO_LB),
        // Vertical speed is read in feet per minute.
        "VELOCITY WORLD Y" => ("sim/flightmodel/position/vh_ind", Float, |v| v * M_TO_FT * 60.),
        // Body velocities are read in feet per second. X-Plane's aircraft
        // axes: x right, y up, z back.
        "VELOCITY BODY X" => ("sim/flightmodel/forces/vx_acf_axis", Float, |v| v * M_TO_FT),
        "VELOCITY BODY Y" => ("sim/flightmodel/forces/vy_acf_axis", Float, |v| v * M_TO_FT),
        "VELOCITY BODY Z" => ("sim/flightmodel/forces/vz_acf_axis", Float, |v| -v * M_TO_FT),
        // X-Plane reports accelerations as g; MSFS in feet per second squared.
        "ACCELERATION BODY X" => ("sim/flightmodel/forces/g_side", Float, |v| v * G_TO_FT_S2),
        "ACCELERATION BODY Y" => ("sim/flightmodel/forces/g_nrml", Float, |v| v * G_TO_FT_S2),
        "ACCELERATION_BODY_Z_WITH_REVERSER" => {
            ("sim/flightmodel/forces/g_axil", Float, |v| v * G_TO_FT_S2)
        }
        // Wind in metres per second, in the world frame.
        "AMBIENT WIND X" => ("sim/weather/aircraft/wind_now_x_msc", Float, identity),
        "AMBIENT WIND Y" => ("sim/weather/aircraft/wind_now_y_msc", Float, identity),
        "AMBIENT WIND Z" => ("sim/weather/aircraft/wind_now_z_msc", Float, |v| -v),
        // MSFS counts pitch positive nose down and bank positive to the left,
        // the opposite of X-Plane's.
        "PLANE PITCH DEGREES" => ("sim/flightmodel/position/theta", Float, |v| -v),
        "PLANE BANK DEGREES" => ("sim/flightmodel/position/phi", Float, |v| -v),
        "PLANE HEADING DEGREES TRUE" => ("sim/flightmodel/position/psi", Float, identity),
        // MSFS's body axes for rates: x pitch, y yaw, z roll; X-Plane's P, Q
        // and R are roll, pitch and yaw. Velocities are read in degrees per
        // second, accelerations in radians per second squared. Like pitch and
        // bank above, MSFS's pitch (x) and roll (z) axes run the opposite way
        // to X-Plane's Q and P: FlyByWire's own FlyByWireInterface.cpp negates
        // both (`q_deg_s = -1 * simData.bodyRotationVelocity.x`, `p_deg_s =
        // -1 * simData.bodyRotationVelocity.z`) to recover its nose-up/
        // right-bank-positive convention, so the raw simvar must carry the
        // opposite sign; the yaw (y) axis is not negated there (`r_deg_s = 1
        // * simData.bodyRotationVelocity.y`), matching X-Plane's R directly.
        "ROTATION VELOCITY BODY X" => ("sim/flightmodel/position/Qrad", Float, |v| -v * RAD_TO_DEG),
        "ROTATION VELOCITY BODY Y" => ("sim/flightmodel/position/Rrad", Float, |v| v * RAD_TO_DEG),
        "ROTATION VELOCITY BODY Z" => ("sim/flightmodel/position/Prad", Float, |v| -v * RAD_TO_DEG),
        "ROTATION ACCELERATION BODY X" => ("sim/flightmodel/position/Q_dot", Float, |v| -v * DEG_TO_RAD),
        "ROTATION ACCELERATION BODY Y" => ("sim/flightmodel/position/R_dot", Float, |v| v * DEG_TO_RAD),
        "ROTATION ACCELERATION BODY Z" => ("sim/flightmodel/position/P_dot", Float, |v| -v * DEG_TO_RAD),
        "AMBIENT PRECIP RATE" => ("sim/weather/region/rain_percent", Float, |v| v * 10.),
        // The a380_systems_wasm registry's own polar form of the wind vector
        // AMBIENT WIND X/Y/Z already carry (a380_systems_wasm/src/lib.rs:440-441).
        "AMBIENT WIND DIRECTION" => (
            "sim/weather/aircraft/wind_now_direction_degt",
            Float,
            identity,
        ),
        "AMBIENT WIND VELOCITY" => (
            "sim/weather/aircraft/wind_now_speed_msc",
            Float,
            |v| v * MS_TO_KNOT,
        ),
        // The pilot's own altimeter, uncorrected for the standard-atmosphere
        // figure PRESSURE ALTITUDE already reads (a380_systems_wasm/src/lib.rs:482).
        "INDICATED ALTITUDE" => (
            "sim/cockpit2/gauges/indicators/altitude_ft_pilot",
            Float,
            identity,
        ),
        "SEA LEVEL PRESSURE" => (
            "sim/weather/region/sealevel_pressure_pas",
            Float,
            |v| v * PA_TO_MB,
        ),
        _ => return None,
    })
}

/// The plugin: FlyByWire's aircraft, its variables, and the clock.
struct Plugin {
    /// FlyByWire's systems: in their own process when it is there (remote/).
    simulation: remote::Systems,
    /// When the systems process last reported its timing.
    systems_report_at: f64,
    /// Whether the simulator variables nothing feeds have been logged.
    unfed_logged: bool,
    vars: Vars,
    time: f64,
    ticks: u64,
    /// Every variable written to disk every few hundred frames.
    state_dump: state_dump::StateDump,
    computed: Computed,
    /// FlyByWire's engine control, ported from their C++.
    fadec: fadec::Fadec,
    /// The thrust levers, through FlyByWire's throttle axis mapping.
    throttles: throttle::Throttles,
    /// FlyByWire's FADEC computers, compiled from their C++, driving
    /// X-Plane's throttles.
    engine_commands: engine_commands::EngineCommands,
    /// FlyByWire's FCUs, PRIMs and SECs, compiled from their C++.
    prims: prim::Prims,
    /// The FCU and autothrust key events, as X-Plane commands.
    commands: afs_events::Commands,
    priority_takeover_commands: afs_events::PriorityTakeoverCommands,
    /// X-Plane's state the PRIMs read directly (FlyByWire's SimData).
    prim_refs: PrimRefs,
    /// MSFS's fuel system from FlyByWire's definition, with their transfer
    /// logic; X-Plane's tanks follow it.
    fuel: Option<fuel::Fuel>,
    /// hyperrealism.md physics workstream 5: sums FlyByWire's engine-driven
    /// pumps' shaft power into `ENGINE_GEARBOX_HYD_LOAD_W:n`.
    hydraulics: physics::hydraulics::Hydraulics,
    /// hyperrealism.md physics workstream 4: strapdown IRS, pitot-static ADR
    /// and the radio altimeter's terrain probe, feeding the ADIRUS/RAs
    /// sensor-realistic inputs instead of X-Plane truth (docs/physics/adirs.md).
    adirs_physics: physics::adirs::AdirsPhysics,
    /// hyperrealism.md physics workstream 2: sums FlyByWire's now-Kirchhoff-
    /// solved generators' real shaft-power demand into
    /// `ENGINE_GEARBOX_ELEC_LOAD_W:n`/`:0` (docs/physics/electrical.md).
    electrical_loads: physics::electrical::EngineLoads,
    /// hyperrealism.md physics workstream 2: real breaker/SSPC current and
    /// I^2t/magnetic trip physics for every `circuits.rs` circuit.
    circuit_protection: physics::electrical::CircuitProtection,
    /// Emergence goal (docs/briefs/hyperrealism.md): the five equipment-bay
    /// thermal nodes, publishing `BAY_<NAME>_TEMPERATURE_C` under the exact
    /// names `breakers.rs`'s already-landed `bay_for`/`trip_step_with_
    /// ambient` coupling reads (`AVIONICS`/`CARGO_FWD`/`CARGO_AFT`/
    /// `WING_ROOT`). See `physics::bays`'s module doc for the pneumatic
    /// leak -> bay heat -> breaker-trip chain.
    bays: physics::bays::Bays,
    /// docs/analysis/cockpit-study-cbs.md CB-001/CB-002/STUDY-001: the
    /// wider circuit-breaker catalogue over FlyByWire's own systems
    /// (generators, TRUs, flight-control computers, air-conditioning LRUs,
    /// fire-detection loops, hydraulic pumps, ...), on top of the 154
    /// `systems.cfg` circuits above.
    breakers: breakers::Breakers,
    /// hyperrealism.md physics workstream 3: republishes FlyByWire's now-real
    /// per-engine bleed extraction mass flow (the PRV's own flow,
    /// `EngineBleedAirSystem::bleed_extraction_flow`, pneumatic.rs) as
    /// `ENGINE_BLEED_EXTRACTION_KG_S:n` (docs/physics/air.md).
    bleed_loads: physics::air::EngineBleedLoads,
    /// Physics workstream 6: engine/APU time-at-temperature exceedance
    /// tracking and the flap/gear/VMO/brake-energy/touchdown exceedances
    /// (docs/physics/failures.md).
    damage: physics::damage::Damage,
    /// Physics workstream 6: per-wheel tyre nitrogen pressure/temperature/
    /// leak/wear-pin model and the fuse-plug melt it can arm
    /// (docs/physics/failures.md).
    tyres: physics::tyre::Tyres,
    /// X-Plane visible/physical failure effects workstream: mirrors this
    /// plugin's own causal fire/damage state onto X-Plane's native failure
    /// and effect datarefs (`docs/physics/fire.md`).
    xp_effects: physics::xp_effects::XpEffects,
    /// Physics workstream 6: the MTBF-based random-failure engine.
    random_failures: random_failures::RandomFailures,
    /// Physics workstream 6: the deferred-item (MEL) set.
    mel: mel::Mel,
    /// Physics workstream 6: scripted arm-by-time/altitude/speed/
    /// flight-phase triggers (`scripted_failures.rs`).
    scripted: scripted_failures::Scripted,
    /// `sim/cockpit2/gauges/indicators/altitude_ft_pilot`, the same pilot
    /// altimeter reading `lib.rs`'s "INDICATED ALTITUDE" mapping uses.
    scripted_altitude: Option<crate::xp::DataRef>,
    /// `sim/flightmodel/position/indicated_airspeed`, the same dataref
    /// `physics/damage.rs`'s own `ias` field reads.
    scripted_ias: Option<crate::xp::DataRef>,
    /// FlyByWire's own `A32NX_FMGC_FLIGHT_PHASE` LVar (`prim.rs`/`radios.rs`
    /// already read it); `scripted_failures::Phase::from_fmgc` decodes it.
    scripted_phase_id: systems::simulation::VariableIdentifier,
    /// Physics workstream 6: component wear/damage, failures and engine/
    /// airframe hours, `Output/preferences/fbw_a380x_airframe.json`.
    persistence: persistence::Persistence,
    // [slot fields: circuits] Every systems.cfg circuit's bus/breaker state.
    circuits: circuits::Circuits,
    // [slot fields: lights] Exterior/cockpit light and window heat/wiper circuits.
    lights: lights::Lights,
    // [slot fields: oxygen] Crew/passenger oxygen quantity and pressure.
    oxygen: oxygen::Oxygen,
    // [slot fields: flight_controls] X-Plane's control surfaces, driven by
    // FlyByWire's hydraulic actuators.
    flight_controls: flight_controls::FlightControls,
    /// Gear, brakes, autobrake, flaps and steering: FlyByWire's input aspects
    /// and X-Plane's flight model following the systems.
    handling: handling::Handling,
    // [slot fields: sensors] X-Plane's state for the simulator variables the
    // systems read, and the ILS on nav receiver 3.
    sensors: sensors::Sensors,
    /// FlyByWire's A380 aspects, failures and start state.
    correctness: correctness::Correctness,
    // [slot fields: radios] MSFS's radios on X-Plane's, and the manual LS tuning.
    radios: radios::Radios,
    // [slot fields: surveillance] The SURV panel's AESS controls.
    surveillance: surveillance::Surveillance,
    // [slot fields: doors] The interactive points, opened and closed at their rates.
    doors: doors::Doors,
    // [slot fields: weight_balance] Payload and CG onto X-Plane's flight model.
    weight_balance: weight_balance::WeightBalance,
    // [slot fields: mapdata] The terrain worker's inputs and outputs, and TCAS targets.
    mapdata: mapdata::MapData,
    // [slot fields: efb] flyPad ground services, payload, refuel, pushback, settings.
    efb: efb::Efb,
    // [slot fields: sound] FlyByWire's Wwise sounds, loaded and decoded off
    // the main thread, played through XPLM's PCM bus from this tick.
    sound: sound::Sound,
    // [slot fields: extra_backend] Lighting presets, pushback, aircraft presets.
    extra_backend: extra_backend::ExtraBackend,
    // [slot fields: fcdc] FlyByWire's FCDCs 1 and 2 and the spoiler lever LVars.
    fcdc: extra_backend_fcdc::ExtraBackendFcdc,
    // [slot fields: extra_backend_fbw] Sidestick/pedal/tracking, the reverser
    // force, LightSync inputs, sim rate limiting and the performance warning.
    fbw_extras: extra_backend_fbw::FlyByWireGlue,
    fbw_extras_refs: FbwExtraRefs,
    fbw_extras_xplane: extra_backend::Xplane,
    // [slot fields: key_events] The general key event dispatcher.
    key_events: key_events::KeyEvents,
    /// The script engine, when the plugin has scripts.
    #[cfg(feature = "js")]
    js: Option<js_bridge::JsHost>,
    /// XPHFBW's shared-memory bridge (docs/briefs/xphfbw-js-bridge.md), once
    /// something has created it for XPHFBW.exe's session tag (start_systems,
    /// agent A's fallback chain: `None` here just means the plugin's own
    /// QuickJS engine (`js`, above) is the only one running).
    #[cfg(feature = "js")]
    xphfbw: Option<xphfbw_host::XphfbwHost>,
    /// The `xphfbw/` status datarefs, its three commands, and the "XPHFBW
    /// settings" aircraft menu entry (docs/briefs/xphfbw-js-bridge.md,
    /// "Custom datarefs and commands").
    xphfbw_datarefs: xphfbw_datarefs::XphfbwDatarefs,
    /// The deep-systems areas (`src/deep`), stepped once per frame with
    /// this tick's `Truth` and the armed deep failures, publishing what the
    /// ECAM triggers and the Study pages read. See `deep::plugin`'s module
    /// doc for where every `Truth` field comes from.
    deep: deep::plugin::DeepLayer,
}

/// The datarefs behind [`prim::SimReadings`].
struct PrimRefs {
    yoke_pitch: Option<DataRef>,
    yoke_roll: Option<DataRef>,
    yoke_heading: Option<DataRef>,
    mag_psi: Option<DataRef>,
    altitude_ind: Option<DataRef>,
    radio_height: Option<DataRef>,
    speedbrake: Option<DataRef>,
    // [msfs-coverage.md] SimData fields the C++ FlyByWireInterface actually
    // reads (updateBaseData/handleFcuInitialization/etc) that had no source:
    // G FORCE, STRUCT BODY ROTATION VELOCITY/ACCELERATION, ACCELERATION BODY
    // Z, AUTOPILOT MASTER, KOHLSMAN SETTING STD:4. See sensors.rs's
    // g_force/body_rotation_*/accel_body_z_m_s2/autopilot_master_on/
    // kohlsman_setting_std_4 for the conversions.
    g_nrml: Option<DataRef>,
    p_rad_s: Option<DataRef>,
    q_rad_s: Option<DataRef>,
    r_rad_s: Option<DataRef>,
    p_dot_deg_s2: Option<DataRef>,
    q_dot_deg_s2: Option<DataRef>,
    r_dot_deg_s2: Option<DataRef>,
    g_axil: Option<DataRef>,
    autopilot_on: Option<DataRef>,
    baro_is_std_pilot: Option<DataRef>,
}

impl PrimRefs {
    fn new(xplm: &Xplm) -> Self {
        Self {
            yoke_pitch: xplm.find("sim/joystick/yoke_pitch_ratio"),
            yoke_roll: xplm.find("sim/joystick/yoke_roll_ratio"),
            yoke_heading: xplm.find("sim/joystick/yoke_heading_ratio"),
            mag_psi: xplm.find("sim/flightmodel/position/mag_psi"),
            altitude_ind: xplm.find("sim/cockpit2/gauges/indicators/altitude_ft_pilot"),
            radio_height: xplm.find("sim/cockpit2/gauges/indicators/radio_altimeter_height_ft_pilot"),
            speedbrake: xplm.find("sim/cockpit2/controls/speedbrake_ratio"),
            g_nrml: xplm.find("sim/flightmodel/forces/g_nrml"),
            p_rad_s: xplm.find("sim/flightmodel/position/Prad"),
            q_rad_s: xplm.find("sim/flightmodel/position/Qrad"),
            r_rad_s: xplm.find("sim/flightmodel/position/Rrad"),
            p_dot_deg_s2: xplm.find("sim/flightmodel/position/P_dot"),
            q_dot_deg_s2: xplm.find("sim/flightmodel/position/Q_dot"),
            r_dot_deg_s2: xplm.find("sim/flightmodel/position/R_dot"),
            g_axil: xplm.find("sim/flightmodel/forces/g_axil"),
            autopilot_on: xplm.find("sim/cockpit2/autopilot/autopilot_on"),
            baro_is_std_pilot: xplm.find("sim/cockpit2/gauges/actuators/barometer_setting_is_std_pilot"),
        }
    }

    fn read(&self, xplm: &Xplm) -> prim::SimReadings {
        let get = |d: Option<DataRef>| d.map_or(0., |d| xplm.get_f(d) as f64);
        let get_i = |d: Option<DataRef>| d.is_some_and(|d| xplm.get_i(d) != 0);
        let (spoilers_armed, spoilers_handle_position) = prim::SimReadings::spoilers_from_xplane(get(self.speedbrake));
        prim::SimReadings {
            inputs: prim::SimReadings::from_xplane_axes(get(self.yoke_pitch), get(self.yoke_roll), get(self.yoke_heading)),
            psi_magnetic_deg: get(self.mag_psi),
            h_ind_ft: get(self.altitude_ind),
            h_radio_ft: get(self.radio_height),
            spoilers_armed,
            spoilers_handle_position,
            nz_g: sensors::g_force(get(self.g_nrml)),
            body_rotation_velocity_rad_s: sensors::body_rotation_velocity_rad_s(get(self.p_rad_s), get(self.q_rad_s), get(self.r_rad_s)),
            body_rotation_acceleration_rad_s2: sensors::body_rotation_acceleration_rad_s2(
                get(self.p_dot_deg_s2),
                get(self.q_dot_deg_s2),
                get(self.r_dot_deg_s2),
            ),
            accel_body_z_m_s2: sensors::accel_body_z_m_s2(get(self.g_axil)),
            autopilot_master_on: sensors::autopilot_master_on(get_i(self.autopilot_on)),
            kohlsman_setting_std_4: sensors::kohlsman_setting_std_4(get_i(self.baro_is_std_pilot)),
        }
    }
}

/// The datarefs [`extra_backend_fbw::FlyByWireGlue`] needs beyond
/// [`prim::SimReadings`] and `Vars`: attitude for `handleSimulationRate`,
/// longitude for `A:ON ANY RUNWAY`, and the sun for `E:TIME OF DAY`.
struct FbwExtraRefs {
    theta: Option<DataRef>,
    phi: Option<DataRef>,
    longitude: Option<DataRef>,
    sun_pitch: Option<DataRef>,
    sun_heading: Option<DataRef>,
}

impl FbwExtraRefs {
    fn new(xplm: &Xplm) -> Self {
        Self {
            theta: xplm.find("sim/flightmodel/position/theta"),
            phi: xplm.find("sim/flightmodel/position/phi"),
            longitude: xplm.find("sim/flightmodel/position/longitude"),
            sun_pitch: xplm.find("sim/graphics/scenery/sun_pitch_degrees"),
            sun_heading: xplm.find("sim/graphics/scenery/sun_heading_degrees"),
        }
    }

    fn read(&self, xplm: &Xplm, sim: prim::SimReadings) -> extra_backend_fbw::Readings {
        let get = |d: Option<DataRef>| d.map_or(0., |d| xplm.get_f(d) as f64);
        extra_backend_fbw::Readings {
            sim,
            longitude: self.longitude.map_or(0., |d| xplm.get_d(d)),
            theta_deg: get(self.theta),
            phi_deg: get(self.phi),
            sun_pitch_deg: get(self.sun_pitch),
            sun_heading_deg: get(self.sun_heading),
        }
    }
}

/// The inputs worked out each tick rather than read from one dataref, with
/// everything they need looked up once instead of every frame.
struct Computed {
    yaw_moi: VariableIdentifier,
    pitch_moi: VariableIdentifier,
    is_ready: VariableIdentifier,
    jzz: Option<DataRef>,
    jyy: Option<DataRef>,
    mass: Option<DataRef>,
    paused: Option<DataRef>,
    gear_deploy: Option<DataRef>,
    /// `msfs_derived`'s own identifiers, resolved once here instead of by
    /// name every tick (perf: `Vars::get` hashes a freshly allocated
    /// `String` on every call; `msfs_derived` used to build one with
    /// `format!`/`to_string()` per variable, every single tick, only to
    /// look up an identifier that never changes after the first tick).
    fd_light: VariableIdentifier,
    ap_fd_active: [VariableIdentifier; 2],
    gear_pct_extended: VariableIdentifier,
}

impl Plugin {
    /// Publish the variables the cockpit's bindings use (the converter lists
    /// them in cockpit_variables.txt next to the plugin), so every cockpit
    /// control and legend finds its dataref from the start.
    fn register_cockpit_variables(&mut self) {
        #[cfg(feature = "js")]
        let path = js_bridge::plugin_dir().map(|d| d.join("cockpit_variables.txt"));
        #[cfg(not(feature = "js"))]
        let path: Option<std::path::PathBuf> = None;
        let Some(text) = path.and_then(|p| std::fs::read_to_string(p).ok()) else { return };
        let n = self.vars.register_cockpit_variables_text(&text);
        log(&format!("{n} cockpit variables registered"));
    }
    fn new(xplm: &'static Xplm) -> Self {
        let mut vars = Vars::new(xplm);
        // On the ground and stopped means a cold aircraft on a stand;
        // anything else means we joined it in flight.
        // FlyByWire's start state from X-Plane's situation, written to
        // A32NX_START_STATE as the flight file would (start_state.rs).
        let start_state: StartState = start_state::detect(xplm, &mut vars);
        let simulation = start_systems(start_state, &mut vars);
        let fadec = fadec::Fadec::new(&mut vars, xplm);
        let throttles = throttle::Throttles::new(xplm);
        let mut engine_commands = engine_commands::EngineCommands::new(&mut vars, xplm);
        // A spawn state other than Hangar/Apron begins with the engines
        // already running (start_state.rs's own module docs: "every state
        // but Hangar and Apron starts with the engines running";
        // `fadec::Fadec::new`'s own `initialize` turns the masters on for
        // exactly these states). `EngineCommands::new` always builds each
        // physical engine stone cold (`Engine::new()`,
        // physics/engine/mod.rs) regardless of state, and nothing called
        // the `spawn_at_idle()` this crate already ships for exactly this
        // case (its own doc comment: "the state a spawn with engines
        // running starts each engine in, as a simulator's in-flight or
        // engines-running spawn does") -- it was only ever wired into the
        // offline harness (offline_chain.rs). Left cold, a non-cold spawn
        // had `fadec.rs`'s `next_state` promote every engine straight to
        // `On` (its `Off` arm) while the real physics engine sat at
        // N1=N2=N3=0 with no starter engaged (state is already `On`, not
        // `Starting`, so `engine_commands.rs`'s own `starter_engaged` gate
        // never latches) -- a cold core has no torque source to spin up on
        // its own, so it stayed at 0% forever while EngineState and the
        // cockpit's masters said running throughout. See
        // docs/deep/debug_start_fps.md.
        if !matches!(start_state, StartState::Hangar | StartState::Apron) {
            engine_commands.spawn_at_idle();
        }
        let prims = prim::Prims::new(&mut vars, start_state.into());
        let commands = afs_events::Commands::register(xplm);
        let priority_takeover_commands = afs_events::PriorityTakeoverCommands::register(xplm);
        let prim_refs = PrimRefs::new(xplm);
        let fuel = match fuel::Fuel::new(&mut vars, xplm) {
            Ok(fuel) => Some(fuel),
            Err(e) => {
                log(&format!("no fuel system: {e}"));
                None
            }
        };
        // hyperrealism.md physics workstream 5: the engine-driven hydraulic
        // pump shaft-power contract variable.
        let hydraulics = physics::hydraulics::Hydraulics::new(&mut vars);
        // hyperrealism.md physics workstream 4: navigation sensor physics
        // (strapdown IRS, pitot-static ADR, radio altimeter terrain probe).
        let adirs_physics = physics::adirs::AdirsPhysics::new(&mut vars, xplm);
        // [slot new: circuits] Built before lights, which reads it.
        let circuits = circuits::Circuits::new(&mut vars);
        // hyperrealism.md physics workstream 2: the shared engine-load
        // contract's electrical term, and circuit protection built on this
        // tick's freshly-built `circuits`.
        let electrical_loads = physics::electrical::EngineLoads::new(&mut vars);
        let circuit_protection = physics::electrical::CircuitProtection::new(&mut vars, &circuits);
        // Emergence goal: equipment bay thermal model. Independent of
        // `circuits`/`circuit_protection` (nothing reads its bay
        // temperatures yet -- see physics::bays's module doc), so
        // construction order relative to them doesn't matter.
        let bays = physics::bays::Bays::new(&mut vars);
        // docs/analysis/cockpit-study-cbs.md CB-001/CB-002/STUDY-001: the
        // wider breaker catalogue. Built after `circuits` (whose
        // `PANEL_CB_NODES` datarefs some entries reuse by name).
        let breakers = breakers::Breakers::new(&mut vars);
        // hyperrealism.md physics workstream 3: the shared engine-load
        // contract's bleed term.
        let bleed_loads = physics::air::EngineBleedLoads::new(&mut vars);
        // Physics workstream 6 (failures, damage, MEL, persistence): load
        // the airframe state before anything else touches wear/damage or
        // failures, so a restored deferral/failure is in place from the
        // first tick.
        let persistence = persistence::Persistence::new(&crate::xp::system_path().unwrap_or_else(|| ".".into()));
        let mut damage = physics::damage::Damage::new(&mut vars, Some(xplm));
        damage.engines = persistence.state.engines;
        wear::publish(persistence.state.wear.clone());
        // Physics workstream 6: per-wheel tyre model, built alongside
        // damage (it re-arms damage.rs's own per-leg tyre-burst ids at
        // fuse-plug melt).
        let tyres = physics::tyre::Tyres::new(&mut vars, Some(xplm));
        // X-Plane visible/physical failure effects workstream (fire, cockpit
        // smoke): built alongside damage, ticked after it each frame.
        let mut xp_effects = physics::xp_effects::XpEffects::new(&mut vars, Some(xplm));
        let mut random_failures = random_failures::RandomFailures::new(0x1234_5678_9abc_def0);
        random_failures.restore(persistence.state.random_failures_config, persistence.state.random_failures_seed);
        let mut mel = mel::Mel::new();
        mel.restore(persistence.state.deferred.clone());
        mel.restore_tech_log(persistence.state.tech_log.clone());
        // Direct component settings, now that the models built above have
        // registered their components.
        failures::register_component_catalogue();
        components::restore_direct(persistence.state.components.clone());
        let scripted = scripted_failures::Scripted::new();
        let scripted_altitude = xplm.find("sim/cockpit2/gauges/indicators/altitude_ft_pilot");
        let scripted_ias = xplm.find("sim/flightmodel/position/indicated_airspeed");
        let scripted_phase_id = vars.get("FMGC_FLIGHT_PHASE".to_owned());
        // [slot new: lights]
        let lights = lights::Lights::new(&mut vars, xplm, &circuits);
        // [slot new: oxygen]
        let oxygen = oxygen::Oxygen::new(&mut vars, xplm);
        // [slot new: flight_controls] Takes X-Plane's surfaces over.
        let flight_controls = flight_controls::FlightControls::new(&mut vars, xplm);
        let handling = handling::Handling::new(&mut vars, xplm);
        // [slot new: sensors]
        let sensors = sensors::Sensors::new(&mut vars, xplm);
        let correctness = correctness::Correctness::new(&mut vars, xplm, start_state);
        // Restore the persisted active/deferred failure set now that
        // `Failures::new()` (inside `Correctness::new`) has registered every
        // id, including this workstream's own catalogue.
        failures::replace(persistence.state.active_failure_ids.iter().copied());
        failures::restore_magnitudes(persistence.state.active_failure_magnitudes.iter().map(|(&id, &m)| (id, m)));
        // [slot new: radios]
        let radios = radios::Radios::new(&mut vars, xplm);
        // [slot new: surveillance]
        let surveillance = surveillance::Surveillance::new(&mut vars);
        // [slot new: doors]
        let doors = doors::Doors::new(&mut vars, xplm);
        // [slot new: weight_balance]
        let weight_balance = weight_balance::WeightBalance::new(&mut vars, xplm);
        // [slot new: mapdata]
        let mapdata = mapdata::MapData::new(xplm);
        // [slot new: efb]
        let efb = efb::Efb::new(&mut vars, xplm);
        // [slot new: sound] Starts loading the user's MSFS package in the
        // background; nothing here touches XPLM or Vars.
        let sound = sound::Sound::new();
        // [slot new: extra_backend]
        let extra_backend = extra_backend::ExtraBackend::new(&mut vars, xplm);
        // [slot new: fcdc]
        let fcdc = extra_backend_fcdc::ExtraBackendFcdc::new(xplm);
        // [slot new: extra_backend_fbw] Loads apt.dat's runways off the main
        // thread for `A:ON ANY RUNWAY`.
        let fbw_extras = extra_backend_fbw::FlyByWireGlue::new(&mut vars, true);
        let fbw_extras_refs = FbwExtraRefs::new(xplm);
        let fbw_extras_xplane = extra_backend::Xplane::new(xplm);
        // [slot new: key_events]
        let key_events = key_events::KeyEvents::new(xplm);
        // [slot new: js] FlyByWire's instruments (js_bridge.rs), and their
        // fbw/hevent/ commands.
        #[cfg(feature = "js")]
        let js = js_bridge::JsHost::find(xplm);
        // XPHFBW's bridge (xphfbw_host.rs): only once start_systems actually
        // started XPHFBW.exe (session_tag() is set only then, in
        // start_xphfbw). The pid is not captured there yet, so a dead
        // process is not detected this way for now (xphfbw_host.rs's own
        // `gone()`/`check_gone` degrade to never-gone, not a crash).
        #[cfg(feature = "js")]
        let xphfbw: Option<xphfbw_host::XphfbwHost> = session_tag().and_then(|tag| {
            let panel_cfg = js_bridge::plugin_dir()
                .and_then(|dir| dir.parent().and_then(|p| p.parent()).map(|p| p.to_path_buf()))
                .and_then(|aircraft| std::fs::read_to_string(aircraft.join("panel").join("panel.cfg")).ok());
            // XPHFBW's pid, so the host notices the app going away and hands
            // the screens back to the QuickJS engine (rule 7).
            let pid = match &simulation {
                remote::Systems::Remote(r) => r.process_id(),
                _ => None,
            };
            let host = panel_cfg.and_then(|cfg| xphfbw_host::XphfbwHost::start(&tag, xplm, &cfg, pid));
            if host.is_none() {
                log("XPHFBW: could not start the plugin's side of the js-bridge; the QuickJS engine keeps the displays");
            }
            host
        });
        // [slot new: deep] The deep-systems areas. Resolves every variable
        // id it reads and writes here, once, and builds the deep failure
        // catalogue's id set once, so its per-frame work is reads, writes
        // and the areas' own physics (deep/plugin.rs, "Frame cost").
        let deep = deep::plugin::DeepLayer::new(&mut vars, Some(xplm));
        log(&format!(
            "deep: {} live area(s) wired in: {}",
            deep.area_names().len(),
            if deep.area_names().is_empty() { "none yet".to_owned() } else { deep.area_names().join(", ") }
        ));
        let computed = Computed {
            yaw_moi: vars.get("TOTAL WEIGHT YAW MOI".to_owned()),
            pitch_moi: vars.get("TOTAL WEIGHT PITCH MOI".to_owned()),
            // FlyByWire's systems wait for this before they run. It is one of
            // the aircraft's own variables, so it carries the prefix.
            is_ready: vars.get("IS_READY".to_owned()),
            jzz: xplm.find("sim/aircraft/weight/acf_Jzz_unitmass"),
            jyy: xplm.find("sim/aircraft/weight/acf_Jyy_unitmass"),
            mass: xplm.find("sim/flightmodel/weight/m_total"),
            paused: xplm.find("sim/time/paused"),
            gear_deploy: xplm.find("sim/flightmodel2/gear/deploy_ratio"),
            fd_light: vars.get("FCU_FD_LIGHT_ON".to_owned()),
            ap_fd_active: [
                vars.get("AUTOPILOT FLIGHT DIRECTOR ACTIVE:1".to_owned()),
                vars.get("AUTOPILOT FLIGHT DIRECTOR ACTIVE:2".to_owned()),
            ],
            gear_pct_extended: vars.get("GEAR TOTAL PCT EXTENDED".to_owned()),
        };
        Self {
            simulation,
            systems_report_at: 0.,
            unfed_logged: false,
            vars,
            time: 0.,
            ticks: 0,
            state_dump: state_dump::StateDump::start(),
            computed,
            fadec,
            throttles,
            engine_commands,
            prims,
            commands,
            priority_takeover_commands,
            prim_refs,
            fuel,
            hydraulics,
            adirs_physics,
            electrical_loads,
            circuit_protection,
            bays,
            breakers,
            bleed_loads,
            damage,
            tyres,
            xp_effects,
            random_failures,
            mel,
            scripted,
            scripted_altitude,
            scripted_ias,
            scripted_phase_id,
            persistence,
            // [slot init: circuits]
            circuits,
            // [slot init: lights]
            lights,
            // [slot init: oxygen]
            oxygen,
            // [slot init: flight_controls]
            flight_controls,
            handling,
            // [slot init: sensors]
            sensors,
            correctness,
            // [slot init: radios]
            radios,
            // [slot init: surveillance]
            surveillance,
            // [slot init: doors]
            doors,
            // [slot init: weight_balance]
            weight_balance,
            // [slot init: mapdata]
            mapdata,
            // [slot init: efb]
            efb,
            // [slot init: sound]
            sound,
            // [slot init: extra_backend]
            extra_backend,
            // [slot init: fcdc]
            fcdc,
            // [slot init: extra_backend_fbw]
            fbw_extras,
            fbw_extras_refs,
            fbw_extras_xplane,
            // [slot init: key_events]
            key_events,
            #[cfg(feature = "js")]
            js,
            #[cfg(feature = "js")]
            xphfbw,
            xphfbw_datarefs: xphfbw_datarefs::XphfbwDatarefs::new(),
            // [slot init: deep]
            deep,
        }
    }

    fn paused(&self) -> bool {
        self.computed
            .paused
            .is_some_and(|d| self.vars.xplm.get_i(d) != 0)
    }

    fn tick(&mut self, delta: f64) {
        crate::perf::lap("tick start");
        invariants::advance_tick(delta);
        flush_log_queue();
        xp::run_queued_commands();
        for (name, value) in study::web::take_writes() {
            let id = self.vars.get(name.to_owned());
            self.vars.write(&id, value);
        }
        self.vars.read_inputs();
        // The moments of inertia are not plain datarefs: X-Plane gives them
        // per kilogram, the systems want slug feet squared for the aircraft.
        let xplm = self.vars.xplm;
        let c = &self.computed;
        let mass = c.mass.map_or(0., |d| xplm.get_f(d) as f64);
        let yaw = c.jzz.map_or(0., |d| xplm.get_f(d) as f64) * mass * KGM2_TO_SLUGFT2;
        let pitch = c.jyy.map_or(0., |d| xplm.get_f(d) as f64) * mass * KGM2_TO_SLUGFT2;
        let (yaw_id, pitch_id, ready_id) = (c.yaw_moi, c.pitch_moi, c.is_ready);
        self.vars.write_from_xplane(&yaw_id, yaw);
        self.vars.write_from_xplane(&pitch_id, pitch);
        self.vars.write_from_xplane(&ready_id, 1.);
        crate::perf::lap("tick-inputs: radios");
        // [slot tick-inputs: radios] Before the sensors, which read the ILS
        // receiver the radios tune.
        self.radios.update(&mut self.vars, xplm);
        crate::perf::lap("tick-inputs: surveillance");
        // [slot tick-inputs: surveillance] The SURV panel's AESS controls
        // (transponder/TCAS system select, TCAS TA ONLY/range, WXR/TAWS
        // lane select, G/S mode inhibit).
        self.surveillance.update(&mut self.vars, xplm);
        crate::perf::lap("tick-inputs: efb");
        // [slot tick-inputs: efb] Before the doors, whose services and GPU it moves.
        self.efb.update(&mut self.vars, xplm, &mut self.doors, delta);
        crate::perf::lap("tick-inputs: doors");
        // [slot tick-inputs: doors] Before the systems, which read the doors.
        self.doors.update(&mut self.vars, xplm, delta);
        crate::perf::lap("tick-inputs: sensors");
        // [slot tick-inputs: sensors] Before the PRIMs, which read the ILS.
        // Breakers pulled or reset from the Study panel.
        self.circuits.apply_requests(&mut self.vars);
        self.breakers.apply_requests(&mut self.vars);
        // Bridges every catalogue breaker's closed state onto FlyByWire's
        // own failure ids / this workstream's plugin_var gates / an
        // absorbed systems.cfg circuit's own breaker, before
        // `correctness.before_systems` hands the active failure set to the
        // systems tick below.
        self.breakers.pre_systems(&mut self.vars, &mut self.circuits);
        self.sensors.update_inputs(&mut self.vars, xplm, self.time);
        // hyperrealism.md physics workstream 4: before the systems tick,
        // which reads its ADIRS_SENSED_* outputs in place of raw X-Plane
        // truth (docs/physics/adirs.md; patches/fbw-rust/navigation-sensors.patch).
        self.adirs_physics.update(&mut self.vars, xplm, delta, self.time);
        self.handling.inputs(&mut self.vars, xplm);
        crate::perf::lap("tick-inputs: extra_backend");
        // [slot tick-inputs: extra_backend] After the sensors' PUSHBACK STATE:
        // FlyByWire's own tug.
        self.extra_backend.inputs(&mut self.vars);

        // The engines first, so the systems see this tick's engines.
        let levers = self.throttles.update(xplm, delta);
        self.fadec.update(&mut self.vars, xplm, &levers, delta, self.time);
        // FlyByWire's C++ computers in FlyByWireInterface::update's order
        // (cpp:121-149): FCUs, PRIMs, SECs, then the FADECs with the PRIM
        // buses, then the surface commands. The FADECs come after the engine
        // control, which writes the thrust limits they read.
        let readings = self.prim_refs.read(xplm);
        crate::perf::lap("tick: extra_backend_fbw");
        // [slot tick: extra_backend_fbw] Before the PRIMs: updateFlyByWire's
        // outputs (sidestick, pedals, tracking mode), the reverser force onto
        // X-Plane's velocity, LightSync's inputs, sim rate and the
        // performance warning.
        let fbw_extras_readings = self.fbw_extras_refs.read(xplm, readings);
        self.fbw_extras.update(&mut self.vars, &mut self.fbw_extras_xplane, &fbw_extras_readings, delta);
        for event in self.prims.fcu_initialization(&readings, self.time) {
            afs_events::send(event);
        }
        let events = afs_events::take();
        // Sidestick priority takeover pushbuttons: a held state (see
        // afs_events::PriorityTakeoverCommands), written straight into the
        // LVars the PRIM/SEC discrete inputs read every tick
        // (capt_priority_takeover_pressed / fo_priority_takeover_pressed,
        // prim.rs), matching MSFS's 3D cockpit click-spot
        // (A32NX_Interior_Misc.xml:385-388).
        let (priority_capt, priority_fo) = afs_events::priority_takeover_held();
        let id = self.vars.get("PRIORITY_TAKEOVER:1".to_owned());
        self.vars.write(&id, priority_capt as u8 as f64);
        let id = self.vars.get("PRIORITY_TAKEOVER:2".to_owned());
        self.vars.write(&id, priority_fo as u8 as f64);
        let prim_buses = self.prims.update(&mut self.vars, &readings, &events, delta, self.time);
        let eec = self.engine_commands.update(
            &mut self.vars,
            xplm,
            delta,
            self.time,
            &prim_buses,
            &events.throttles,
        );
        self.prims.update_after_fadecs(&mut self.vars, eec);
        crate::perf::lap("tick: fcdc");
        // [slot tick: fcdc] FlyByWire runs updateFcdc between updateSec and
        // updateFadec, and updateSpoilers after updateServoSolenoidStatus
        // (cpp:139-154). The FADECs run inside engine_commands above, so right
        // after update_after_fadecs is the closest place for both.
        self.fcdc.update(&mut self.vars, xplm, &self.prims, &readings, delta);

        crate::perf::lap("tick-before-systems: flight_controls");
        // [slot tick-before-systems: flight_controls]
        self.handling.before_systems(&mut self.vars, delta);
        self.correctness.before_systems(&mut self.vars, &mut self.simulation, delta);
        crate::perf::lap("tick-before-systems: xphfbw_host");
        // [slot tick-before-systems: xphfbw_host] Rule 3 (docs/briefs/
        // xphfbw-js-bridge.md): a view's slot writes apply before the
        // systems tick reads the variables, so the tick sees this frame's
        // cockpit input, not last frame's.
        #[cfg(feature = "js")]
        if let Some(host) = self.xphfbw.as_mut() {
            host.pre_tick(&mut self.vars);
        }
        self.time += delta;
        self.ticks += 1;
        self.simulation
            .tick(Duration::from_secs_f64(delta), self.time, &mut self.vars);
        if let remote::Systems::Remote(r) = &mut self.simulation {
            if self.time - self.systems_report_at >= 60. {
                self.systems_report_at = self.time;
                if let Some(line) = r.take_report() {
                    log(&line);
                }
            }
        }
        // hyperrealism.md physics workstream 5: after the systems tick, so
        // this frame's engine-driven pump shaft power (written by FlyByWire's
        // own hydraulic/mod.rs, patches/fbw-rust/fluids.patch) is available
        // to sum into ENGINE_GEARBOX_HYD_LOAD_W:n. Before fuel, whose tank
        // heat model reads the same pump loads for hydraulic heat rejection.
        self.hydraulics.update(&mut self.vars);
        // hyperrealism.md physics workstream 2: after the systems tick, so
        // this frame's Kirchhoff-solved generator shaft power and bus
        // voltages are this tick's. Before fuel/lights, so a breaker this
        // tick trips before they read `self.circuits`'s breaker state.
        self.electrical_loads.update(&mut self.vars);
        self.circuit_protection.update(&mut self.vars, &mut self.circuits, delta);
        // Emergence goal: bay temperatures, before `breakers.post_systems`
        // so a future breaker-derating coupling reading
        // `BAY_<NAME>_TEMPERATURE_C` sees this tick's value, not last
        // tick's (physics::bays's module doc).
        self.bays.update(&mut self.vars, delta);
        // docs/analysis/cockpit-study-cbs.md CB-002/STUDY-001: real current
        // and I^2t/magnetic trip for the wider breaker catalogue, same
        // ordering rationale as `circuit_protection` above (a trip this
        // tick is visible to `pre_systems` from next tick).
        self.breakers.post_systems(&mut self.vars, delta);
        // hyperrealism.md physics workstream 3: after the systems tick, so this
        // frame's real bleed extraction flow (the PRV's own flow,
        // patches/fbw-rust/air.patch) is available to republish as
        // ENGINE_BLEED_EXTRACTION_KG_S:n.
        self.bleed_loads.update(&mut self.vars);
        // Physics workstream 6 (failures, damage, MEL, persistence): after
        // the systems tick, so this frame's EGT/TLA are current. `real_delta`
        // is zero while X-Plane is paused, so none of these advance a wear
        // clock, draw a random failure, or accrue airframe hours then.
        let real_delta = if self.paused() { 0.0 } else { delta };
        self.damage.update(&mut self.vars, Some(xplm), real_delta);
        for line in &self.damage.events {
            log(line);
        }
        for line in self.damage.apply_requests() {
            log(&line);
        }
        // Physics workstream 6: per-wheel tyre model, after damage so a
        // fuse-plug melt this tick arms the same leg id damage.rs's own
        // brake-only backstop uses, before xp_effects mirrors it onto
        // X-Plane's native failure datarefs.
        self.tyres.update(&mut self.vars, Some(xplm), real_delta);
        for line in self.tyres.events.drain(..) {
            log(&line);
        }
        self.xp_effects.update(&mut self.vars, Some(xplm));
        physics::damage::publish(self.damage.engines);
        self.persistence.state.engines = self.damage.engines;
        // Damageable components: progress wear-like settings and recombine
        // every source (failure magnitudes, direct settings) per parameter.
        components::tick(real_delta / 3600.0);
        self.random_failures.apply_requests();
        let mut already = std::collections::BTreeSet::from_iter(failures::active_ids());
        already.extend(self.mel.list(self.persistence.state.airframe_hours).into_iter().map(|(d, _)| d.id));
        for id in self.random_failures.update(real_delta / 3600.0, &already) {
            failures::set_active(id, true);
            log(&format!("random failure: {id} ({}) activated", failures::any_failure_name(id)));
        }
        for line in self.mel.apply_requests(self.persistence.state.airframe_hours) {
            log(&line);
        }
        for id in self.mel.expired(self.persistence.state.airframe_hours) {
            log(&format!(
                "MEL deferral for failure {id} ({}) has expired: repair required before further dispatch",
                failures::any_failure_name(id)
            ));
        }
        mel::publish(self.mel.snapshot(), self.persistence.state.airframe_hours, self.mel.tech_log());
        // Physics workstream 6: arm-by-time/altitude/speed/flight-phase
        // scripted triggers (`scripted_failures.rs`), sampled from the same
        // real datarefs/LVar `physics/damage.rs`'s own exceedance model and
        // `prim.rs`/`radios.rs`'s FMGC-phase readers already use.
        for line in self.scripted.apply_requests() {
            log(&line);
        }
        let scripted_sample = scripted_failures::Sample {
            elapsed_hours: self.persistence.state.airframe_hours,
            altitude_ft: self.scripted_altitude.map_or(0.0, |d| xplm.get_f(d) as f64),
            speed_kt: self.scripted_ias.map_or(0.0, |d| xplm.get_f(d) as f64),
            phase: scripted_failures::Phase::from_fmgc(self.vars.read(&self.scripted_phase_id)),
        };
        for id in self.scripted.update(&scripted_sample) {
            failures::set_active(id, true);
            log(&format!("scripted failure: {id} ({}) activated", failures::any_failure_name(id)));
        }
        self.persistence.tick(real_delta);
        self.persistence.state.active_failure_ids = failures::active_ids();
        self.persistence.state.active_failure_magnitudes = failures::active_magnitudes();
        self.persistence.state.wear = wear::snapshot();
        self.persistence.state.components = components::snapshot_direct();
        self.persistence.state.deferred = self.mel.snapshot();
        self.persistence.state.tech_log = self.mel.tech_log();
        let (rf_config, rf_seed) = self.random_failures.snapshot();
        self.persistence.state.random_failures_config = rf_config;
        self.persistence.state.random_failures_seed = rf_seed;
        // The fuel system after the systems, as FlyByWire's aspects and
        // transfer logic run after theirs.
        if let Some(fuel) = self.fuel.as_mut() {
            // Publishes APU_FUEL_FEED_PRESSURE_PSI itself (the APU fuel
            // pressure switch's input, docs/physics/fire.md).
            fuel.update(&mut self.vars, xplm, delta, &self.circuits);
        }
        crate::perf::lap("tick-after-systems: deep");
        // [slot tick-after-systems: deep] The deep-systems areas
        // (src/deep). After the systems tick, so this frame's FlyByWire bus
        // potentials, hydraulic section pressures and APU state are
        // current; after engine_commands, so the bleed ports are this
        // frame's physics::engine output; and after the failure block
        // above, so a random/scripted failure armed this frame reaches the
        // areas on the frame it is armed rather than the next one. The
        // areas' own published values are read back by the ECAM bridge's
        // triggers and the Study pages next frame (deep/live.rs's
        // "Ordering" note).
        self.deep.tick(&mut self.vars, Some(xplm), delta);
        crate::perf::lap("tick-after-systems: lights");
        // [slot tick-after-systems: lights] After the systems, so the buses'
        // ELEC_*_BUS_IS_POWERED are this tick's.
        self.lights.update(&mut self.vars, xplm, &self.circuits, delta);
        crate::perf::lap("tick-after-systems: oxygen");
        // [slot tick-after-systems: oxygen] After the systems, so the cabin
        // altitude/pressure this tick are theirs.
        self.oxygen.update(&mut self.vars, xplm, delta);
        crate::perf::lap("tick-after-systems: flight_controls");
        // [slot tick-after-systems: flight_controls] The actuators have moved
        // this tick; X-Plane's surfaces follow.
        self.flight_controls.update(&mut self.vars, xplm);
        self.handling.after_systems(&mut self.vars, xplm);
        crate::perf::lap("tick-after-systems: correctness");
        // [slot tick-after-systems: correctness] FlyByWire's post-tick aspects.
        self.correctness.after_previous_tick(&mut self.vars);
        crate::perf::lap("tick-after-systems: weight_balance");
        // [slot tick-after-systems: weight_balance] After the payload aspect and
        // the fuel have written this tick's weights.
        self.weight_balance.update(&mut self.vars, xplm);
        crate::perf::lap("tick-after-systems: doors");
        // [slot tick-after-systems: doors] The hydraulic cargo doors' clips.
        self.doors.update_model(&mut self.vars, xplm);
        crate::perf::lap("tick-after-systems: extra_backend");
        // [slot tick-after-systems: extra_backend] After the systems, before the
        // scripts, in panel.cfg's gauge order (VCockpit21-23).
        self.extra_backend.update(&mut self.vars, delta, self.time);
        crate::perf::lap("tick-after-systems: sensors");
        // [slot tick-after-systems: sensors]
        // [slot tick-after-systems: mapdata] The EGPWC's outputs to the terrain
        // worker, its thresholds back; TCAS targets before the scripts ask.
        self.mapdata.update(&mut self.vars, xplm);
        // Scripts after the systems, as MSFS instruments update after them.
        // (docs/briefs/xphfbw-js-bridge.md.) `h_events`/`provider_events` are
        // each taken once so the plugin's own QuickJS cockpit (`js`) and
        // XPHFBW's bridge (`xphfbw`) see the same frame's events without
        // either draining the other's share of the queue (js_bridge.rs's
        // `take_hevents`/`take_provider_events`).
        #[cfg(feature = "js")]
        {
            let h_events = js_bridge::take_hevents();
            let provider_events = js_bridge::take_provider_events();
            let mut events: Vec<(String, f64)> = Vec::new();
            // XPHFBW first, so this tick's switch between the two display
            // engines is settled before either consumes this tick's cockpit
            // events: otherwise, on the tick its views finish loading (or
            // reload after a script error), the same KCCU keystroke reached
            // both the plugin's own QuickJS cockpit and XPHFBW's views — a
            // double press (docs/deep/debug_mcdu.md).
            let was_active = self.xphfbw.as_ref().is_some_and(|h| h.displays_active());
            if let Some(host) = self.xphfbw.as_mut() {
                host.post_tick(&mut self.vars, self.time, &h_events, &provider_events);
                events.extend(host.take_events());
                crate::perf::lap("tick-after-systems: xphfbw_datarefs");
                // [slot tick-after-systems: xphfbw_datarefs] xphfbw/views_loaded.
                xphfbw_datarefs::set_views_loaded(host.views_loaded());
                // Rule 7: "never both engines drawing one screen" — the
                // plugin's own QuickJS cockpit stops once every screened
                // XPHFBW view has loaded, and resumes if XPHFBW goes away.
                let active = host.displays_active();
                if let Some(js) = self.js.as_mut() {
                    if active && js.running() {
                        js.suspend();
                    } else if !active && !js.running() {
                        js.resume();
                    }
                }
            }
            if let Some(js) = self.js.as_mut() {
                // Cockpit events went to XPHFBW above whenever its views were
                // the authoritative displays going into this tick.
                let cockpit_events: &[_] = if was_active { &[] } else { &h_events };
                js.update(&mut self.vars, delta, self.time, cockpit_events, &provider_events);
                events.extend(js.take_events());
            }
            crate::perf::lap("tick-after-systems: radios");
            // [slot tick-after-systems: radios] The scripts' key events: the
            // radios take theirs, the doors theirs.
            for (name, value) in events {
                let name = name.trim_start_matches("K:");
                crate::perf::lap("tick-after-systems: extra_backend #2");
                // [slot tick-after-systems: extra_backend] ... the pushback tug its own,
                // and key_events.rs every other K: event FlyByWire sends.
                let handled = self.radios.handle_event(name, value, xplm)
                    || self.doors.handle_event(name, value)
                    || self.extra_backend.handle_event(name, value)
                    // [slot tick-after-systems: fuel] FUELSYSTEM_PUMP_ON/OFF/
                    // TOGGLE clicks, routed to the native fuel network.
                    || self.fuel.as_mut().is_some_and(|f| f.handle_event(name, &[value]))
                    || self.key_events.handle(&mut self.vars, name, value);
                if !handled {
                    self.key_events.unhandled(name);
                }
            }
        }
        crate::perf::lap("tick-after-systems: sound");
        // [slot tick-after-systems: sound] After the scripts, so a
        // PLAY_INSTRUMENT_SOUND queued this tick plays this tick.
        self.sound.update(&mut self.vars);
        crate::perf::lap("tick-after-systems: key_events");
        // [slot tick-after-systems: key_events] Events with several arguments
        // (TRIGGER_KEY_EVENT, key_events::push).
        for (name, args) in key_events::KeyEvents::take_pushed() {
            let value = args.first().copied().unwrap_or(0.);
            let handled = self.key_events.handle_args(&mut self.vars, &name, &args)
                || self.radios.handle_event(&name, value, xplm)
                || self.doors.handle_event(&name, value)
                || self.extra_backend.handle_event(&name, value)
                // [slot tick-after-systems: fuel]
                || self.fuel.as_mut().is_some_and(|f| f.handle_event(&name, &args));
            if !handled {
                self.key_events.unhandled(&name);
            }
        }
        crate::perf::lap("tick-after-systems: display");
        // [slot tick-after-systems: display] Screen brightness from the
        // potentiometers the instruments' knobs set.
        #[cfg(feature = "js")]
        {
            let vars = &mut self.vars;
            display::update_brightness(|name| vars.ids.get(name).copied().map(|id| vars.read(&id)));
        }
        crate::perf::lap("tick-after-systems: wxr");
        // [slot tick-after-systems: wxr] After the scripts, so this tick's
        // ND mode/range/overlay selector L:vars (just written) are current.
        #[cfg(feature = "js")]
        wxr::tick(&self.vars, xplm, self.time);
        xplane_mirror::update(&mut self.vars, xplm); // [slot tick-after-systems: xplane_mirror]
        self.vars.publish();
        self.msfs_derived(xplm);
        self.take_snapshot();
        if !self.unfed_logged && self.time >= 30. {
            self.unfed_logged = true;
            let unfed = self.vars.unfed_simulator_variables();
            if !unfed.is_empty() {
                log(&format!(
                    "{} simulator variables are read but nothing feeds them (MSFS would): {}",
                    unfed.len(),
                    unfed.join(", ")
                ));
            }
        }
        // `state_dump::EVERY_TICKS` is always 1 (the real interval,
        // `xphfbw.stateDumpFrames`, is checked live inside `StateDump::tick`
        // itself, see its module docs) so this used to lock the shared
        // snapshot `Mutex` -- contended with the panel thread -- every
        // single tick even with dumps off. `enabled()` is checked first
        // instead: cheap, and false in the common (dumps off) case, so the
        // snapshot lock is only taken on a tick that could actually dump.
        if self.ticks % state_dump::EVERY_TICKS == 0 && self.state_dump.enabled() {
            if let Ok(s) = snapshot().lock() {
                self.state_dump.tick(s.time, s.ticks, &s.names, &s.datarefs, &s.values, &s.sources);
            }
        }
        crate::perf::lap("tick-after-systems: xphfbw_datarefs #2");
        // [slot tick-after-systems: xphfbw_datarefs] The xphfbw/ status
        // datarefs and its three commands (docs/briefs/xphfbw-js-bridge.md,
        // "Custom datarefs and commands"). Last: acting on a restart_app
        // press replaces self.simulation, so nothing above should still be
        // holding a borrow of the old one.
        let cmds = self.xphfbw_datarefs.poll_commands();
        if cmds.show_app {
            xphfbw_datarefs::signal_show_app();
        }
        if cmds.restart_displays {
            xphfbw_datarefs::signal_restart_displays();
        }
        if cmds.restart_app {
            log("XPHFBW: restart_app pressed; relaunching the systems backend");
            self.simulation = start_systems(start_state::detect(xplm, &mut self.vars), &mut self.vars);
        }
        let status = xphfbw_status(&self.simulation);
        self.xphfbw_datarefs.update(&status, session_tag().as_deref());
        crate::perf::end_frame();
    }

    /// Simulator variables MSFS derives from several of its own values.
    ///
    /// The three identifiers this needs are resolved once, in `Computed`
    /// (`Plugin::new`), not here: this runs every tick, and
    /// `Vars::get`/`VariableRegistry::get` takes an owned `String` to hash
    /// against `Vars::ids`, so building one fresh with `format!`/
    /// `to_string()` per variable every tick (four allocations a frame,
    /// times the tick rate) was pure per-frame waste once the identifier
    /// is known after the first tick -- see `docs/deep/debug_start_fps.md`.
    fn msfs_derived(&mut self, xplm: &Xplm) {
        // The flight director: MSFS's flag follows the FCU's FD pushbutton.
        let fd = self.vars.read(&self.computed.fd_light);
        for id in self.computed.ap_fd_active {
            self.vars.write(&id, fd);
        }
        // All gear extended, as a ratio (FlyByWire compares it with 0.95 in
        // "percent", which MSFS hands back over 100 for this variable).
        if let Some(d) = self.computed.gear_deploy {
            let mut ratios = [0f32; 10];
            let n = xplm.get_vf(d, &mut ratios).min(5);
            if n > 0 {
                let mean = ratios[..n].iter().map(|&r| r as f64).sum::<f64>() / n as f64;
                self.vars.write_from_xplane(&self.computed.gear_pct_extended, mean);
            }
        }
    }

    /// Hand the panel the state this tick ended with. Names only change when
    /// a variable is registered, so they are copied only then.
    fn take_snapshot(&self) {
        let Ok(mut snapshot) = snapshot().lock() else { return };
        let total: usize = self.vars.slots.iter().map(Vec::len).sum();
        snapshot.time = self.time;
        snapshot.ticks = self.ticks;
        if snapshot.names.len() != total {
            snapshot.names = [SIMULATOR, NAMED]
                .iter()
                .flat_map(|&kind| self.vars.names[kind].iter().cloned())
                .collect();
            snapshot.datarefs = [SIMULATOR, NAMED]
                .iter()
                .flat_map(|&kind| self.vars.slots[kind].iter().map(|s| s.dataref.clone()))
                .collect();
            snapshot.index = snapshot
                .names
                .iter()
                .enumerate()
                .map(|(i, n)| (n.clone(), i))
                .collect();
            snapshot.generation += 1;
        }
        snapshot.values.clear();
        snapshot.sources.clear();
        for kind in [SIMULATOR, NAMED] {
            snapshot.values.extend(self.vars.slots[kind].iter().map(|s| s.value));
            snapshot.sources.extend(self.vars.slots[kind].iter().map(|slot| {
                if slot.input.is_some() || slot.from_xplane {
                    FROM_XPLANE
                } else if slot.written {
                    FROM_SYSTEMS
                } else {
                    NO_SOURCE
                }
            }));
        }
    }

}

/// What the XPHFBW app (app/) needs from the plugin's library: where FlyByWire's
/// settings live and how they are written, so both write them one way.
pub mod settings_files {
    use std::collections::BTreeMap;
    use std::path::{Path, PathBuf};

    /// FlyByWire's stored data (NXDataStore), relative to X-Plane's folder.
    pub fn datastore_path(xplane: &Path) -> PathBuf {
        xplane.join("Output").join("preferences").join("fbw_a380x_datastore.json")
    }

    /// The flyPad settings efb.rs mirrors into simulator variables.
    pub fn flypad_ini_path(xplane: &Path) -> PathBuf {
        xplane.join(crate::efb::settings_path())
    }

    /// The XPHFBW app's own settings.
    pub fn app_settings_path(xplane: &Path) -> PathBuf {
        xplane.join("Output").join("preferences").join("xphfbw.json")
    }

    /// NXDataStore's stored key for a setting.
    pub fn stored_key(key: &str) -> String {
        crate::efb::stored_key(key)
    }

    /// Whether the flyPad ini owns a setting (efb.rs reads it from there).
    pub fn flypad_owns(key: &str) -> bool {
        crate::efb::SETTINGS.iter().any(|s| s.key == key)
    }

    pub fn parse_ini(text: &str) -> BTreeMap<String, String> {
        crate::efb::parse_ini(text)
    }

    pub fn write_ini(values: &BTreeMap<String, String>) -> String {
        crate::efb::write_ini(values)
    }
}

/// The systems process's entry point (src/bin/fbw_a380_systems_server.rs).
/// The Study tab's JSON (`study::web`), so XPHFBW can lay the pages out while
/// the plugin's own port is not up. Breakers read the snapshot, which is
/// empty outside X-Plane: no current, no trip.
pub mod study_json {
    pub fn pages() -> String {
        crate::study::web::pages_json()
    }
    pub fn failures() -> String {
        crate::study::web::failures_json()
    }
    pub fn breakers() -> String {
        crate::study::web::breakers_json()
    }
    /// Every damageable component's parameters, at their healthy values
    /// outside X-Plane.
    pub fn components() -> String {
        static REGISTERED: std::sync::Once = std::sync::Once::new();
        REGISTERED.call_once(|| {
            crate::engine_commands::register_component_catalogue();
            crate::failures::register_component_catalogue();
        });
        crate::study::web::components_json()
    }
    pub fn maintenance() -> String {
        crate::study::web::maintenance_json()
    }
    /// `query` is the request's query string (`q=...`).
    pub fn mel(query: &str) -> String {
        crate::study::web::mel_search_json(query)
    }
    /// Where the operator MEL is looked for, outside X-Plane.
    pub fn set_xplane_root(root: std::path::PathBuf) {
        crate::mel_catalog::set_xplane_root(root);
    }
}

pub fn serve_systems(tag: &str, parent: Option<u32>) -> Result<(), String> {
    remote::server::serve(tag, parent)
}

/// The tag XPHFBW's systems block (and its js-bridge session,
/// `xphfbw_bridge::Session`) were created with, once `start_systems` chose
/// XPHFBW as the backend: "the systems tag is the session tag"
/// (docs/briefs/xphfbw-js-bridge.md, architecture recap). `None` before that,
/// or when the fallback chain picked `fbw_a380_systems_server.exe` or the
/// in-process systems instead, neither of which has a js-bridge session.
/// Whoever creates `xphfbw_bridge::Session` (agent C, xphfbw_host.rs) opens
/// it by this same tag rather than inventing its own.
fn session_tag_cell() -> &'static std::sync::Mutex<Option<String>> {
    static SESSION_TAG: std::sync::OnceLock<std::sync::Mutex<Option<String>>> = std::sync::OnceLock::new();
    SESSION_TAG.get_or_init(|| std::sync::Mutex::new(None))
}

fn set_session_tag(tag: String) {
    if let Ok(mut g) = session_tag_cell().lock() {
        *g = Some(tag);
    }
}

pub(crate) fn session_tag() -> Option<String> {
    session_tag_cell().lock().ok().and_then(|g| g.clone())
}

/// What `Plugin::tick` knows about the chosen backend, for
/// [`xphfbw_datarefs::XphfbwDatarefs::update`].
fn xphfbw_status(sim: &remote::Systems) -> xphfbw_datarefs::Status {
    match sim {
        remote::Systems::Remote(r) => xphfbw_datarefs::Status {
            app_running: session_tag().is_some(),
            systems_remote: true,
            systems_round_trip_ms: if r.stats.ticks > 0 {
                r.stats.round_trip_us as f64 / r.stats.ticks as f64 / 1000.
            } else {
                0.
            },
            systems_late_ticks: r.stats.late.min(u32::MAX as u64) as u32,
        },
        remote::Systems::Local(_) => xphfbw_datarefs::Status::default(),
    }
}

/// FlyByWire's systems: in XPHFBW.exe (with the JS bridge and FlyByWire's
/// own instruments), in `fbw_a380_systems_server.exe` alongside the plugin,
/// or in the plugin itself, in that fallback order
/// (docs/briefs/xphfbw-app.md "Plugin changes"; docs/briefs/xphfbw-js-bridge.md
/// agent A). `xphfbw.json`'s `systemsOutOfProcess` (app_settings.rs, agent B)
/// gates the first two: off skips straight to the plugin, same as
/// `FBW_SYSTEMS_IN_PROCESS=1`.
fn start_systems(start_state: StartState, vars: &mut Vars) -> remote::Systems {
    const SERVER_EXE: &str = "fbw_a380_systems_server.exe";
    let env_in_process = std::env::var("FBW_SYSTEMS_IN_PROCESS").is_ok_and(|v| v == "1");
    let out_of_process = app_settings::systems_out_of_process() && !env_in_process;
    if out_of_process {
        #[cfg(feature = "js")]
        if let Some(dir) = js_bridge::plugin_dir() {
            let xphfbw_exe = dir.join("XPHFBW").join("XPHFBW.exe");
            if xphfbw_exe.is_file() {
                let xp_root = xp::system_path();
                // plugin_dir() is aircraft/plugins/fbw_a380_systems; two
                // parents up is the aircraft folder XPHFBW.exe needs to
                // serve /VFS/ from (agent G).
                let aircraft_dir = dir.parent().and_then(|p| p.parent()).map(|p| p.to_path_buf());
                match start_xphfbw(&xphfbw_exe, xp_root.as_deref(), aircraft_dir.as_deref(), start_state, vars) {
                    Ok(r) => {
                        log(&format!("XPHFBW: running, {} variables shared, in lockstep with the frame", r.variable_count()));
                        return remote::Systems::Remote(r);
                    }
                    Err(e) => log(&format!("XPHFBW: {e}; trying {SERVER_EXE}")),
                }
            }
            let exe = dir.join("64").join(SERVER_EXE);
            if exe.is_file() {
                match remote::client::RemoteSystems::start(&exe, start_state, vars) {
                    Ok(r) => {
                        log(&format!("systems process: running, {} variables shared, in lockstep with the frame", r.variable_count()));
                        return remote::Systems::Remote(r);
                    }
                    Err(e) => log(&format!("systems process: {e}; running the systems in the plugin")),
                }
            }
        }
    }
    remote::Systems::Local(Box::new(Simulation::new(start_state, A380::new, vars)))
}

/// Start XPHFBW.exe (`XPHFBW.exe <tag> <X-Plane pid> --xp-root=<X-Plane
/// folder> --aircraft=<aircraft folder>`, docs/briefs/xphfbw-app.md) and
/// connect to its systems, capturing the session tag `RemoteSystems::connect`
/// generates so [`session_tag`] can hand it to whoever creates the js-bridge
/// session with it.
#[cfg(feature = "js")]
fn start_xphfbw<V: VariableRegistry + SimulatorReaderWriter>(
    exe: &std::path::Path,
    xp_root: Option<&std::path::Path>,
    aircraft_dir: Option<&std::path::Path>,
    state: StartState,
    vars: &mut V,
) -> Result<remote::client::RemoteSystems, String> {
    use std::os::windows::process::CommandExt;
    const CREATE_NO_WINDOW: u32 = 0x0800_0000;
    let exe = exe.to_path_buf();
    let xp_root = xp_root.map(|p| p.to_path_buf());
    let aircraft_dir = aircraft_dir.map(|p| p.to_path_buf());
    let captured_tag: std::rc::Rc<std::cell::RefCell<Option<String>>> = std::rc::Rc::new(std::cell::RefCell::new(None));
    let holder = captured_tag.clone();
    let result = remote::client::RemoteSystems::connect(state, vars, move |tag| {
        *holder.borrow_mut() = Some(tag.to_string());
        let mut cmd = std::process::Command::new(&exe);
        cmd.arg(tag).arg(std::process::id().to_string());
        if let Some(root) = &xp_root {
            cmd.arg(format!("--xp-root={}", root.display()));
        }
        if let Some(dir) = &aircraft_dir {
            cmd.arg(format!("--aircraft={}", dir.display()));
        }
        cmd.creation_flags(CREATE_NO_WINDOW).spawn().map(Some).map_err(|e| format!("cannot start {}: {e}", exe.display()))
    });
    if result.is_ok() {
        if let Some(tag) = captured_tag.borrow_mut().take() {
            set_session_tag(tag);
        }
    }
    result
}

static mut PLUGIN: Option<Box<Plugin>> = None;
static mut XPLM: Option<Xplm> = None;

/// The slot a dataref accessor was registered with.
fn slot_of(refcon: *mut c_void) -> (usize, usize) {
    let packed = refcon as usize;
    (packed >> 32, packed & 0xffff_ffff)
}

fn value_of(refcon: *mut c_void) -> f64 {
    let (kind, index) = slot_of(refcon);
    unsafe {
        let plugin = &raw const PLUGIN;
        (*plugin)
            .as_ref()
            .and_then(|p| p.vars.slots.get(kind)?.get(index))
            .map_or(0., |s| s.value)
    }
}

fn set_value(refcon: *mut c_void, value: f64) {
    let (kind, index) = slot_of(refcon);
    unsafe {
        let plugin = &raw mut PLUGIN;
        if let Some(slot) = (*plugin)
            .as_mut()
            .and_then(|p| p.vars.slots.get_mut(kind)?.get_mut(index))
        {
            slot.value = value;
        }
    }
}

pub unsafe extern "C" fn get_datad(refcon: *mut c_void) -> f64 {
    value_of(refcon)
}
pub unsafe extern "C" fn get_dataf(refcon: *mut c_void) -> f32 {
    value_of(refcon) as f32
}
pub unsafe extern "C" fn get_datai(refcon: *mut c_void) -> c_int {
    value_of(refcon).round() as c_int
}
pub unsafe extern "C" fn set_datad(refcon: *mut c_void, value: f64) {
    set_value(refcon, value);
}
pub unsafe extern "C" fn set_dataf(refcon: *mut c_void, value: f32) {
    set_value(refcon, value as f64);
}
pub unsafe extern "C" fn set_datai(refcon: *mut c_void, value: c_int) {
    set_value(refcon, value as f64);
}

/// Called by X-Plane every frame.
unsafe extern "C" fn flight_loop(
    elapsed: f32,
    _since_loop: f32,
    _counter: c_int,
    _refcon: *mut c_void,
) -> f32 {
    let plugin = &raw mut PLUGIN;
    if let Some(plugin) = (*plugin).as_mut() {
        // A paused or slewing sim can hand us a zero or a very long frame;
        // the systems integrate over the step, so keep it sane.
        // A paused sim does not move FlyByWire's systems either: MSFS skips
        // the tick outright rather than running a sliver of time.
        if plugin.paused() {
            return -1.;
        }
        let delta = (elapsed as f64).clamp(0.001, 0.2);
        let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| plugin.tick(delta)));
        if result.is_err() {
            log("systems panicked; the simulation is stopped");
            let plugin = &raw mut PLUGIN;
            *plugin = None;
        }
    }
    -1.
}

fn log(message: &str) {
    // XPLMDebugString only on X-Plane's thread; others wait for the next tick.
    if !xp::on_main_thread() {
        LOG_QUEUE.lock().unwrap_or_else(std::sync::PoisonError::into_inner).push(message.to_string());
        return;
    }
    unsafe {
        let xplm = &raw const XPLM;
        if let Some(xplm) = (*xplm).as_ref() {
            xplm.log(&format!("FBW A380 systems: {message}\n"));
        }
        #[cfg(test)]
        if (*xplm).is_none() {
            eprintln!("log: {message}");
        }
    }
}

static LOG_QUEUE: std::sync::Mutex<Vec<String>> = std::sync::Mutex::new(Vec::new());

/// Writes what other threads logged; the plugin's tick calls this.
fn flush_log_queue() {
    let lines = std::mem::take(&mut *LOG_QUEUE.lock().unwrap_or_else(std::sync::PoisonError::into_inner));
    for line in lines {
        log(&line);
    }
}

/// # Safety
/// X-Plane calls this once, on the main thread, with three 256 byte buffers.
#[no_mangle]
pub unsafe extern "C" fn XPluginStart(
    name: *mut c_char,
    signature: *mut c_char,
    description: *mut c_char,
) -> c_int {
    write_into(name, "FBW A380X systems");
    write_into(signature, "msfs2xp.fbw.a380.systems");
    write_into(description, "FlyByWire's A380X systems simulation in X-Plane");
    xp::remember_main_thread();

    let Some(xplm) = Xplm::load() else { return 0 };
    let xplm_ref = &raw mut XPLM;
    *xplm_ref = Some(xplm);
    log("starting");
    1
}

/// # Safety
/// Called by X-Plane on the main thread.
#[no_mangle]
pub unsafe extern "C" fn XPluginEnable() -> c_int {
    let xplm = &raw const XPLM;
    let Some(xplm) = (*xplm).as_ref() else { return 0 };
    // On a large stack: the systems no longer fit X-Plane's main-thread
    // stack while they are built (big_stack.rs). Boxed, so the finished
    // aircraft is not copied back across stack frames.
    let result = big_stack::run(|| Box::new(Plugin::new(xplm)));
    match result {
        Ok(mut plugin) => {
            plugin.register_cockpit_variables();
            let published = plugin.vars.publish();
            log(&format!("{published} variables published as fbw/ datarefs"));
            let p = &raw mut PLUGIN;
            *p = Some(plugin);
            xplm.register_loop(flight_loop);
            match panel::start(PANEL_PORT) {
                Ok(port) => log(&format!("systems panel on http://127.0.0.1:{port}")),
                Err(e) => log(&format!("no systems panel: {e}")),
            }
            study::build_menu(xplm);
            xphfbw_datarefs::build_menu(xplm);
            // [slot enable: display] The screens' cockpit devices.
            #[cfg(feature = "js")]
            display::start(xplm);
            1
        }
        Err(_) => {
            log("could not build the aircraft; systems are not running");
            0
        }
    }
}

/// # Safety
/// Called by X-Plane on the main thread.
#[no_mangle]
pub unsafe extern "C" fn XPluginDisable() {
    panel::stop();
    let xplm = &raw const XPLM;
    if let Some(xplm) = (*xplm).as_ref() {
        xplm.unregister_loop(flight_loop);
        study::destroy(xplm);
        xphfbw_datarefs::destroy_menu(xplm);
        // [slot disable: display]
        #[cfg(feature = "js")]
        display::stop(xplm);
        let p = &raw mut PLUGIN;
        if let Some(plugin) = (*p).as_mut() {
            plugin.engine_commands.release(xplm);
            // [slot release: flight_controls] Surfaces back to X-Plane.
            plugin.flight_controls.release(xplm);
            plugin.handling.release(xplm);
            // [slot release: extra_backend]
            plugin.extra_backend.release();
            // [slot release: sound] Stops whatever is still looping.
            plugin.sound.release();
            // [slot release: js] The fbw/hevent/ command handlers.
            #[cfg(feature = "js")]
            if let Some(js) = plugin.js.as_mut() {
                js.release(xplm);
            }
            plugin.commands.release(xplm);
            plugin.priority_takeover_commands.release(xplm);
            // Physics workstream 6: save the airframe state on exit, so
            // wear/damage/MEL/failure history survives to the next start.
            if let Err(e) = plugin.persistence.save() {
                log(&format!("could not save fbw_a380x_airframe.json on exit: {e}"));
            }
        }
    }
    let p = &raw mut PLUGIN;
    *p = None;
}

/// # Safety
/// Called by X-Plane on the main thread.
#[no_mangle]
pub unsafe extern "C" fn XPluginStop() {}

/// # Safety
/// Called by X-Plane on the main thread.
#[no_mangle]
pub unsafe extern "C" fn XPluginReceiveMessage(_from: c_int, _message: c_int, _param: *mut c_void) {}

/// Copy a string into one of X-Plane's 256 byte buffers.
unsafe fn write_into(buffer: *mut c_char, text: &str) {
    let bytes = text.as_bytes();
    let len = bytes.len().min(255);
    std::ptr::copy_nonoverlapping(bytes.as_ptr() as *const c_char, buffer, len);
    *buffer.add(len) = 0;
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn simulator_variables_keep_their_plain_names() {
        // MSFS's own variables, which X-Plane feeds.
        assert!(is_simulator_variable("AIRSPEED INDICATED"));
        assert!(is_simulator_variable("TURB ENG CORRECTED N1:1"));
        assert!(is_simulator_variable("ACCELERATION_BODY_Z_WITH_REVERSER"));
        // The aircraft's own, which carry FlyByWire's prefix.
        assert!(!is_simulator_variable("IS_READY"));
        assert!(!is_simulator_variable("ELEC_AC_1_BUS_IS_POWERED"));
    }

    #[test]
    fn dataref_names_keep_only_what_x_plane_accepts() {
        assert_eq!(sanitise("A32NX_ELEC_AC_1_BUS_IS_POWERED"), "A32NX_ELEC_AC_1_BUS_IS_POWERED");
        assert_eq!(sanitise("AIRSPEED INDICATED"), "AIRSPEED_INDICATED");
        assert_eq!(sanitise("GEAR:1"), "GEAR_1");
    }

    #[test]
    fn pressure_altitude_follows_the_standard_atmosphere() {
        assert!(pressure_altitude_ft(101_325.).abs() < 1.);
        // 500 hPa is about 18 300 feet.
        assert!((pressure_altitude_ft(50_000.) - 18_289.).abs() < 50.);
    }

    /// a380_systems_wasm/src/lib.rs:440-441,482,500: the Rust systems
    /// registry's own wind-vector-as-polar, indicated altitude and sea level
    /// pressure fields, still on X-Plane's dataref/unit at source 0 as of the
    /// 2026-09-17 coverage pass (msfs-coverage.md).
    #[test]
    fn ambient_wind_direction_and_velocity_come_from_the_wind_at_the_aircraft() {
        let (dataref, kind, convert) = mapping("AMBIENT WIND DIRECTION").unwrap();
        assert_eq!(dataref, "sim/weather/aircraft/wind_now_direction_degt");
        assert!(matches!(kind, Kind::Float));
        // X-Plane and MSFS both give wind direction in true degrees the wind
        // blows from: no conversion, just X-Plane's own value passed through.
        assert_eq!(convert(270.), 270.);

        let (dataref, _, convert) = mapping("AMBIENT WIND VELOCITY").unwrap();
        assert_eq!(dataref, "sim/weather/aircraft/wind_now_speed_msc");
        // 10 m/s is about 19.4 knots.
        assert!((convert(10.) - 19.438_44).abs() < 0.001);
    }

    #[test]
    fn indicated_altitude_is_the_pilot_altimeter_not_pressure_altitude() {
        let (dataref, kind, convert) = mapping("INDICATED ALTITUDE").unwrap();
        assert_eq!(dataref, "sim/cockpit2/gauges/indicators/altitude_ft_pilot");
        assert!(matches!(kind, Kind::Float));
        assert_eq!(convert(3_500.), 3_500.);
        // Distinct from PRESSURE ALTITUDE, which reads static pressure instead.
        assert_ne!(mapping("INDICATED ALTITUDE").unwrap().0, mapping("PRESSURE ALTITUDE").unwrap().0);
    }

    #[test]
    fn sea_level_pressure_converts_pascals_to_millibars() {
        let (dataref, _, convert) = mapping("SEA LEVEL PRESSURE").unwrap();
        assert_eq!(dataref, "sim/weather/region/sealevel_pressure_pas");
        // Standard sea level pressure, 101 325 Pa, is 1013.25 mb (QNH 1013).
        assert!((convert(101_325.) - 1_013.25).abs() < 0.001);
    }

    #[test]
    fn simulator_variables_are_mapped_to_x_plane() {
        let (dataref, _, convert) = mapping("AIRSPEED TRUE").unwrap();
        assert_eq!(dataref, "sim/flightmodel/position/true_airspeed");
        // 100 m/s is 194 knots.
        assert!((convert(100.) - 194.38).abs() < 0.01);
        // MSFS counts pitch the other way round.
        let (_, _, pitch) = mapping("PLANE PITCH DEGREES").unwrap();
        assert_eq!(pitch(5.), -5.);
        assert!(mapping("A32NX_SOMETHING").is_none());
    }

    #[test]
    fn rotation_rates_match_the_pitch_and_bank_sign_flip() {
        // The pitch (x) and roll (z) rate axes must flip sign the same way
        // PLANE PITCH/BANK DEGREES do (FlyByWireInterface.cpp negates
        // simData.bodyRotationVelocity.x and .z but not .y when it recovers
        // its own nose-up/right-bank-positive convention), or the ADIRS/
        // aerodynamic model feeding on them drifts. The yaw (y) axis keeps
        // X-Plane's sign.
        let (dataref, _, x) = mapping("ROTATION VELOCITY BODY X").unwrap();
        assert_eq!(dataref, "sim/flightmodel/position/Qrad");
        assert_eq!(x(1.), -RAD_TO_DEG);
        let (dataref, _, y) = mapping("ROTATION VELOCITY BODY Y").unwrap();
        assert_eq!(dataref, "sim/flightmodel/position/Rrad");
        assert_eq!(y(1.), RAD_TO_DEG);
        let (dataref, _, z) = mapping("ROTATION VELOCITY BODY Z").unwrap();
        assert_eq!(dataref, "sim/flightmodel/position/Prad");
        assert_eq!(z(1.), -RAD_TO_DEG);

        let (dataref, _, x) = mapping("ROTATION ACCELERATION BODY X").unwrap();
        assert_eq!(dataref, "sim/flightmodel/position/Q_dot");
        assert_eq!(x(1.), -DEG_TO_RAD);
        let (dataref, _, y) = mapping("ROTATION ACCELERATION BODY Y").unwrap();
        assert_eq!(dataref, "sim/flightmodel/position/R_dot");
        assert_eq!(y(1.), DEG_TO_RAD);
        let (dataref, _, z) = mapping("ROTATION ACCELERATION BODY Z").unwrap();
        assert_eq!(dataref, "sim/flightmodel/position/P_dot");
        assert_eq!(z(1.), -DEG_TO_RAD);
    }

    #[test]
    fn ambient_wind_frame_matches_update_context_rs_rotation() {
        // Worked example: wind from the north at 20 m/s (air moving south),
        // aircraft heading east. A north wind on an eastbound aircraft is a
        // crosswind from the left.
        //
        // X-Plane's `wind_now_*_msc` datarefs are in its OpenGL local frame:
        // +X east, +Y up, +Z south (DataRefs.txt: `psi`, the heading, is
        // measured "from the Z axis", and X-Plane's heading-0-is-north/
        // heading-increases-toward-east convention only works out if north
        // is -Z). Air moving south is then (x=0, y=0, z=+20) before this
        // table's conversion.
        let (_, _, wx) = mapping("AMBIENT WIND X").unwrap();
        let (_, _, wy) = mapping("AMBIENT WIND Y").unwrap();
        let (_, _, wz) = mapping("AMBIENT WIND Z").unwrap();
        let world_wind = [wx(0.), wy(0.), wz(20.)];

        // update_context.rs's own rotation: `heading_rotation.inverse() *
        // world_ambient_wind`, heading_rotation a standard right-handed
        // rotation about +Y by true_heading (nalgebra's convention, which
        // this table also relies on for PLANE HEADING DEGREES TRUE being
        // passed through as X-Plane's own psi unchanged). At 90 degrees
        // (east) that inverse rotation sends (x, y, z) -> (-z, y, x).
        let heading_rad_east = 90f64.to_radians();
        let (s, c) = heading_rad_east.sin_cos();
        // R_y(-heading): (x cos(-h) + z sin(-h), y, -x sin(-h) + z cos(-h))
        let rotated = [
            world_wind[0] * c - world_wind[2] * s,
            world_wind[1],
            world_wind[0] * s + world_wind[2] * c,
        ];

        // Body frame (update_context.rs's own comment): "X axis positive is
        // left to right". A left crosswind is air moving rightward past the
        // aircraft, i.e. a positive X component here.
        assert!(
            rotated[0] > 0.,
            "a north wind heading east should be a left crosswind (positive body X), got {rotated:?}"
        );
    }
}
