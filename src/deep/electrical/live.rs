//! The live electrical system: the one instance of the A380 per-load
//! network that actually runs in the simulation.
//!
//! `network.rs`/`loads.rs`/`sources.rs`/`shedding.rs` are the physics and
//! the catalogue; nothing owned an instance of any of it. This module is
//! that instance, wired to `crate::deep::live`'s [`Area`] contract.
//!
//! ## Who owns the solve
//!
//! The electrical network, the ATA circuit-breaker catalogue
//! (`deep::breakers`) and the wire-bundle model (`deep::wiring`) are one
//! physical system, and it is solved **exactly once per frame, here**:
//!
//! * This area owns the whole resistive solve -- bus voltages, every load's
//!   current, every feeder's current -- because `network::Network::step` is
//!   the only model in the crate that has the topology to do it (buses,
//!   sources, contactors, diodes, loads).
//! * `deep::breakers` owns the *trip decision* for the 399-entry ELMS
//!   catalogue. It cannot compute a current, so it reads the current its
//!   breaker actually carried from this area's own published output, one
//!   frame late, and publishes back `BKR_<id>_OPEN`. This area applies that
//!   command to the matching `network::Breaker`'s contacts on the next
//!   frame.
//! * `deep::wiring` owns the harness: which circuit is chafed, burnt, open
//!   or corroded, and what arc current that produces. It too reads this
//!   area's published bus voltages, and publishes back per-circuit
//!   severities, which this area folds into the matching load's own
//!   `LoadFaults` -- so an arcing wire genuinely adds current to the circuit
//!   it chafes into, and that current is what the breaker then sees.
//!
//! The one-frame lag on both couplings is the lag `crate::deep::live`'s own
//! module doc documents as deliberate; at 30-60 Hz it is far below the time
//! constant of an I^2t element (seconds), a TRU's thermal mass (minutes) or
//! a chafe's own progression.
//!
//! ## The board
//!
//! Areas publish through a `FnMut(&str, f64)` closure and have no way to
//! read a variable back. The three areas therefore exchange their coupling
//! quantities through [`board`]: a thread-local snapshot written at
//! `publish` time and read at the next frame's `tick`. It carries exactly
//! the quantities that are also published as named variables (per-breaker
//! current, per-breaker open command, bus voltage, per-circuit wiring
//! severities) -- typed and indexed rather than string-keyed, because this
//! is the largest network in the crate and 1,300 string lookups a frame is
//! not free. `Deep::tick` ticks every area before publishing any, so the
//! lag is exactly one frame regardless of the order the areas were added
//! in.

use std::cell::RefCell;
use std::collections::HashMap;
use std::sync::OnceLock;

use crate::deep::api::Registry;
use crate::deep::live::{Area, DerivedFailure, Faults, Truth};

use super::loads::{self, Catalog};
use super::network::{BusId, Contactor, ContactorKind, FeedSource, Network, NetworkReport, ALL_BUS_IDS};
use super::shedding::{power_budget, SheddingRelays, ShedInputs};
use super::sources::{
    ApuGeneratorFaults, BatteryFaults, GroundPowerFaults, RatFaults, StaticInverterFaults, TruFaults, VfgFaults, Wiring, WiringInputs,
};

// ---------------------------------------------------------------------
// Ratings/limits this live layer needs in order to judge "is this source
// in trouble", all re-cited from the modules that already derive them.

/// `sources::Vfg::RATED_TRUE_POWER_W` (real, FBW-sourced:
/// `alternating_current.rs:393`).
const GEN_RATED_TRUE_POWER_W: f64 = 150_000.0;
/// `sources::ApuGenerator::RATED_TRUE_POWER_W` (real, FBW-sourced:
/// `Pw980ApuGenerator::MAXIMUM_LOAD_WATT`).
const APU_GEN_RATED_TRUE_POWER_W: f64 = 120_000.0;
/// `sources::GroundPower::RATED_APPARENT_POWER_VA` * its own `POWER_FACTOR`
/// (both real, FBW-sourced: `external_power_source.rs`).
const GPU_RATED_TRUE_POWER_W: f64 = 90_000.0 * 0.8;

/// MIL-STD-704F steady-state 115 V AC utilisation limits (public standard):
/// 108 V to 118 V. A generator control unit trips its generator off line
/// when its own bus leaves that band -- the real under/over-voltage
/// protection, not a scripted "GEN FAULT" flag.
const AC_UNDERVOLTAGE_TRIP_V: f64 = 108.0;
const AC_OVERVOLTAGE_TRIP_V: f64 = 118.0;

/// The same 85%-of-nominal under-voltage margin `loads::min_operating_voltage`
/// already derives and cites from MIL-STD-704F: below this a 28 V DC source
/// is no longer holding its own bus up at all, which is what a TR FAULT or a
/// BAT FAULT annunciates.
fn dc_undervoltage_trip_v() -> f64 {
    BusId::Dc1.nominal_voltage() * 0.85
}

/// NEC Chapter 9 Table 8 DC resistance, uncoated copper at 20 C, converted
/// to ohm/m (the published table's own figures, divided by 304.8 m per
/// 1000 ft -- the same table `deep::wiring::gauge` tabulates, quoted here
/// rather than imported so this module keeps no cross-area dependency for a
/// constant).
const OHM_PER_M_AWG_4_0: f64 = 0.049_01 / 304.8;
const OHM_PER_M_AWG_4: f64 = 0.248_5 / 304.8;

/// A main AC bus tie: AWG 4/0 tie-bar cable between two Primary Power
/// Centres. Run length is GENERIC (no public A380 wiring-diagram length
/// exists) but sized the same way `wiring::routing::hop_length_m` sizes its
/// own hops -- ~15 m between two power centres on an aircraft of this size.
const AC_TIE_OHM: f64 = OHM_PER_M_AWG_4_0 * 15.0;
/// An essential/shed feeder: lighter AWG 4 feeder cable, ~20 m from a main
/// power centre to a secondary one.
const ESS_FEEDER_OHM: f64 = OHM_PER_M_AWG_4 * 20.0;
/// A DC tie/feeder between two DC busbars in the same equipment centre,
/// AWG 4, ~15 m.
const DC_FEEDER_OHM: f64 = OHM_PER_M_AWG_4 * 15.0;
/// A ground-service feeder, AWG 4, ~25 m out to the service panel.
const GND_SVC_FEEDER_OHM: f64 = OHM_PER_M_AWG_4 * 25.0;

/// The catalogue's transit-only actuators: an electro-hydraulic gear or
/// gear-door actuator draws its 2.1 kW only while the gear is actually
/// travelling, and is dead the rest of the flight. `loads.rs` gives every
/// entry `commanded_on: true`, which is right for a continuously-running
/// consumer and wrong for these six -- left on, they sit at 250 A each and
/// collapse the essential DC bus for the whole flight. `ElectricalLive::new`
/// forces them off as the correct cold-start default; from the second frame
/// on, [`ElectricalLive::command_transit_loads`] drives them for real off
/// `Controls::gear_door_commanded_open` (the door and, on this catalogue's
/// simplified model, the leg it travels with are both "in transit" whenever
/// that real, continuous commanded position sits away from either end
/// stop).
/// The continuous rating of whatever source feeds a bus directly, A --
/// every figure re-cited from `sources.rs`'s own breaker ratings, which are
/// themselves FBW-sourced. `0.0` for a bus with no source of its own (its
/// feeder is then sized purely by the load it carries).
fn source_rating_a(bus: BusId) -> f64 {
    match bus {
        // `sources::generator_rated_a`: 150 kW / 0.8 / 115 V.
        BusId::Ac1 | BusId::Ac2 | BusId::Ac3 | BusId::Ac4 => 150_000.0 / 0.8 / 115.0,
        // `sources::TRU_RATED_A`.
        BusId::Dc1 | BusId::Dc2 | BusId::DcEss | BusId::DcApu => 200.0,
        // `sources::Wiring::build`'s own battery breaker rating,
        // `Battery::RATED_CAPACITY_AH * 4.0`.
        BusId::DcBat | BusId::DcHot1 => 23.0 * 4.0,
        // `sources::Rat::MAX_POWER_W` at 115 V -- the largest source that
        // can ever feed the emergency bus.
        BusId::AcEmer => 70_000.0 / 115.0,
        // `sources::GroundPower::RATED_APPARENT_POWER_VA` at 115 V.
        BusId::AcGndFltSvc => 90_000.0 / 115.0,
        _ => 0.0,
    }
}

const TRANSIT_ONLY_ACTUATORS: [&str; 6] =
    ["gear-actuator-nose", "gear-actuator-left", "gear-actuator-right", "gear-door-actuator-nose", "gear-door-actuator-left", "gear-door-actuator-right"];

/// The gap-closing pass's own transit-only/one-shot loads. `Truth` did not
/// carry a real command for any of these when they were first catalogued --
/// an engine ignition exciter only fires during a start sequence, a
/// fire-bottle squib fires once on a real discharge command, an APU start
/// contactor is only energised through a real start sequence, and a cargo
/// door actuator draws only while the door is actually moving -- so
/// `ElectricalLive::new` forced every one of them off, the same honest
/// choice [`TRANSIT_ONLY_ACTUATORS`] above made for the same reason.
///
/// `Truth::controls` now carries `starter_engaged`, the fire and agent
/// pushbuttons and `apu_start_pb_on`, and `deep::cabin` publishes the cargo
/// door's own commanded/actual position -- so
/// [`ElectricalLive::command_transit_loads`] drives all 17 for real from the
/// second frame on. This array still does its original job at construction
/// (a correct cold-start default before that method has run once).
const TRANSIT_ONLY_NO_TRUTH_INPUT: [&str; 17] = [
    "ignition-1a",
    "ignition-1b",
    "ignition-2a",
    "ignition-2b",
    "ignition-3a",
    "ignition-3b",
    "ignition-4a",
    "ignition-4b",
    "eng-fire-bottle-1-squib-1",
    "eng-fire-bottle-1-squib-2",
    "eng-fire-bottle-2-squib-1",
    "eng-fire-bottle-2-squib-2",
    "apu-fire-bottle-squib-1",
    "apu-fire-bottle-squib-2",
    "apu-start-contactor",
    "cargo-door-fwd-actuator-ctl",
    "cargo-door-aft-actuator-ctl",
];

/// Fallback equipment-bay temperature, C, for the one frame before
/// `deep::thermal_zones` has published `THERMAL_ZONE_MAINAVIONICS_
/// TEMPERATURE_C` at all. **GENERIC**: a ventilated avionics bay is
/// designed around cabin-like conditions (the ECS supplies it from the same
/// conditioned air), not ambient static air, so 20 C -- the low end of
/// normal cabin comfort range -- is a far safer stand-in than SAT, which at
/// cruise sits some 70 K colder than a real bay ever does.
const EQUIPMENT_BAY_FALLBACK_C: f64 = 20.0;

/// How far the cargo door's published commanded/actual position may differ,
/// percent, before the actuator-control circuit is considered still
/// driving. **GENERIC**: `deep::cabin::doors_slides`' own actuator settles
/// well inside this at rest; wide enough that normal feedback noise never
/// falsely holds the circuit live.
const CARGO_DOOR_TRAVEL_MARGIN_PERCENT: f64 = 2.0;

/// How far `Controls::gear_door_commanded_open` may sit from a fully
/// closed/open end stop, as a fraction of travel, before the corresponding
/// gear/gear-door actuator is considered in transit. **GENERIC**: small
/// enough that a door parked at either stop reads unambiguously as "not
/// travelling", wide enough to clear the settling noise a real actuator's
/// position feedback carries at the stop.
const GEAR_TRANSIT_MARGIN_FRACTION: f64 = 0.02;

/// Power-budget hysteresis: the galley shed relay is commanded once the
/// budget goes negative and released only once the margin has recovered
/// past 10% of available capacity. A plain Schmitt band -- without it a
/// shed relay chatters at exactly the balance point, the same reason
/// `network::Load` carries `UNDERVOLTAGE_RESTART_MARGIN`. GENERIC width
/// (no published A380 ELMS threshold), stated as a fraction of capacity so
/// it scales with whatever generation is actually on line.
const SHED_RELEASE_MARGIN_FRACTION: f64 = 0.10;

// ---------------------------------------------------------------------
// The board: the typed, one-frame-late channel between the three areas.

pub mod board {
    use super::*;

    /// Everything the three coupled areas hand each other. Indices are into
    /// the canonical network built by [`topology`], which every area
    /// resolves its own ids against once at construction.
    #[derive(Default, Clone, Debug)]
    pub struct Board {
        /// Per network-breaker index: the current it carried, A. Written by
        /// `deep::electrical`, read by `deep::breakers`.
        pub breaker_current_a: Vec<f64>,
        /// Per network-breaker index: 1.0 when `deep::breakers`' trip unit
        /// holds this breaker open. Written by `deep::breakers`, read by
        /// `deep::electrical`.
        pub breaker_open_cmd: Vec<f64>,
        /// Per bus index: last solved bus voltage, V. Written by
        /// `deep::electrical`, read by `deep::wiring` (a wiring fault's arc
        /// current depends on the voltage behind it).
        pub bus_voltage: [f64; 17],
        /// Per load index: severities `deep::wiring` derived for that
        /// load's own feeder from harness damage. Written by
        /// `deep::wiring`, read by `deep::electrical`, which folds them into
        /// the matching `network::LoadFaults`.
        pub load_short: Vec<f64>,
        pub load_open: Vec<f64>,
        pub load_high_resistance: Vec<f64>,
    }

    thread_local! {
        static BOARD: RefCell<Board> = RefCell::new(Board::default());
    }

    pub fn with_board<R>(f: impl FnOnce(&Board) -> R) -> R {
        BOARD.with(|b| f(&b.borrow()))
    }

    pub fn with_board_mut<R>(f: impl FnOnce(&mut Board) -> R) -> R {
        BOARD.with(|b| f(&mut b.borrow_mut()))
    }

    /// Reset the board.
    ///
    /// Nothing outside this module has to remember to call this:
    /// [`super::ElectricalLive::new`] does it itself, and that constructor
    /// is on the only path by which an aircraft's electrical area comes
    /// into existence (`deep::live::all_areas`). It stays public because
    /// it is the honest name for the operation, and because a test driving
    /// `breakers`/`wiring` *without* an electrical area still needs it.
    ///
    /// The board is process state rather than `Deep` state on purpose --
    /// it is a typed channel (`Vec<f64>` per breaker, per load, per bus)
    /// and `deep::live`'s own inter-area channel carries single `f64`s by
    /// name -- but process state that survives an aircraft is a bug, not a
    /// feature: two `all_areas()` built one after another in one thread
    /// used to share it, so a second flight in one session started on the
    /// previous aircraft's breaker currents, bus voltages and harness
    /// damage, and two runs of an identical healthy state came out 135
    /// published variables apart.
    pub fn clear() {
        with_board_mut(|b| *b = Board::default());
    }

    /// The canonical network's id-to-index mapping, built once per process.
    ///
    /// `deep::breakers` and `deep::wiring` have to name an electrical
    /// breaker or load by its id; the solve indexes by position. Building
    /// one throwaway network here (the identical construction order
    /// `ElectricalLive::new` and `registry::register` both use) gives every
    /// area the same mapping without any of them owning a second network.
    pub struct Topology {
        pub breaker_index: HashMap<&'static str, usize>,
        pub load_index: HashMap<&'static str, usize>,
        pub breaker_count: usize,
        pub load_count: usize,
    }

    static TOPOLOGY: OnceLock<Topology> = OnceLock::new();

    pub fn topology() -> &'static Topology {
        TOPOLOGY.get_or_init(|| {
            let mut net = Network::new();
            loads::build(&mut net);
            Wiring::build(&mut net, 15.0);
            Topology {
                breaker_index: net.breakers.iter().enumerate().map(|(i, b)| (b.id, i)).collect(),
                load_index: net.loads.iter().enumerate().map(|(i, l)| (l.spec.id, i)).collect(),
                breaker_count: net.breakers.len(),
                load_count: net.loads.len(),
            }
        })
    }
}

use board::topology;

// ---------------------------------------------------------------------
// Failure routing: every id `registry.rs` registers, resolved to the exact
// model field that entry's `model_field` names.

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum LoadField {
    OpenCircuit,
    ShortToGround,
    HighResistance,
    Intermittent,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Target {
    Load(usize, LoadField),
    /// `(index, fails_to_trip?)` -- the two `network::BreakerFaults` fields.
    BreakerFailsToTrip(usize),
    BreakerNuisance(usize),
    ContactorFailsToClose(usize),
    ContactorWelded(usize),
    Diode(usize),
    Bus(usize),
    VfgWinding(usize),
    VfgRegulator(usize),
    ApuGenWinding(usize),
    ApuGenRegulator(usize),
    Tru(usize),
    BatteryCapacity(usize),
    BatteryResistance(usize),
    StaticInverter,
    Rat,
    Gpu,
}

/// Resolves `registry::register`'s own output into `(failure id, target)`
/// pairs against a concrete network.
///
/// Deliberately derived from the registry itself rather than from a second
/// hand-written numbering: `registry.rs` assigns its ids by walking the very
/// same catalogue in the very same order, so re-deriving that walk here
/// would be a copy that could silently drift. Reading the registry back
/// cannot drift -- a load added to `loads.rs` appears in both at once, and
/// this module's own test asserts every registered failure resolved.
fn route_failures(net: &Network) -> (Vec<(u64, Target)>, Vec<String>) {
    let mut reg = Registry::default();
    super::registry::register(&mut reg);
    let mut out = Vec::with_capacity(reg.failures.len());
    let mut unresolved = Vec::new();

    let source_index = |prefix: &str, comp: &str| -> Option<usize> {
        let rest = comp.strip_prefix(prefix)?;
        rest.parse::<usize>().ok().map(|n| n - 1)
    };

    for f in &reg.failures {
        let Some((type_path, field)) = f.model_field.split_once(".faults.") else {
            unresolved.push(f.model_field.clone());
            continue;
        };
        let comp = f.component.as_str();
        let target = match (type_path, field) {
            ("deep::electrical::network::Load", field) => {
                // Component id is `<ata>_elec.<load id>`.
                let id = comp.split_once('.').map(|(_, rest)| rest).unwrap_or(comp);
                let Some(&idx) = topology().load_index.get(id) else {
                    unresolved.push(f.component.clone());
                    continue;
                };
                let lf = match field {
                    "open_circuit" => LoadField::OpenCircuit,
                    "short_to_ground" => LoadField::ShortToGround,
                    "high_resistance" => LoadField::HighResistance,
                    "intermittent" => LoadField::Intermittent,
                    _ => {
                        unresolved.push(f.model_field.clone());
                        continue;
                    }
                };
                Target::Load(idx, lf)
            }
            ("deep::electrical::network::Breaker", field) => {
                let id = comp.strip_prefix("24_elec.bkr.").unwrap_or(comp);
                let Some(&idx) = topology().breaker_index.get(id) else {
                    unresolved.push(f.component.clone());
                    continue;
                };
                match field {
                    "fails_to_trip" => Target::BreakerFailsToTrip(idx),
                    "nuisance_trip" => Target::BreakerNuisance(idx),
                    _ => {
                        unresolved.push(f.model_field.clone());
                        continue;
                    }
                }
            }
            ("deep::electrical::network::Contactor", field) => {
                let id = comp.strip_prefix("24_elec.contactor.").unwrap_or(comp);
                let Some(idx) = net.contactor_index(id) else {
                    unresolved.push(f.component.clone());
                    continue;
                };
                match field {
                    "fails_to_close" => Target::ContactorFailsToClose(idx),
                    "welded_closed" => Target::ContactorWelded(idx),
                    _ => {
                        unresolved.push(f.model_field.clone());
                        continue;
                    }
                }
            }
            ("deep::electrical::network::Diode", _) => {
                let id = comp.strip_prefix("24_elec.diode.").unwrap_or(comp);
                let Some(idx) = net.diode_index(id) else {
                    unresolved.push(f.component.clone());
                    continue;
                };
                Target::Diode(idx)
            }
            ("deep::electrical::network::Bus", _) => {
                let label = comp.strip_prefix("24_elec.bus.").unwrap_or(comp);
                let Some(bus) = ALL_BUS_IDS.iter().find(|b| b.label() == label) else {
                    unresolved.push(f.component.clone());
                    continue;
                };
                Target::Bus(bus.index())
            }
            ("deep::electrical::sources::Vfg", field) => {
                let Some(n) = source_index("24_elec.vfg-", comp) else {
                    unresolved.push(f.component.clone());
                    continue;
                };
                if field == "winding_degradation" {
                    Target::VfgWinding(n)
                } else {
                    Target::VfgRegulator(n)
                }
            }
            ("deep::electrical::sources::ApuGenerator", field) => {
                let Some(n) = source_index("24_elec.apu-gen-", comp) else {
                    unresolved.push(f.component.clone());
                    continue;
                };
                if field == "winding_degradation" {
                    Target::ApuGenWinding(n)
                } else {
                    Target::ApuGenRegulator(n)
                }
            }
            ("deep::electrical::sources::Tru", _) => {
                // `Wiring::tru`'s own array order (sources.rs `tr_names`).
                let idx = match comp {
                    "24_elec.tr-1" => 0,
                    "24_elec.tr-2" => 1,
                    "24_elec.tr-ess" => 2,
                    "24_elec.tr-apu" => 3,
                    _ => {
                        unresolved.push(f.component.clone());
                        continue;
                    }
                };
                Target::Tru(idx)
            }
            ("deep::electrical::sources::Battery", field) => {
                let Some(n) = source_index("24_elec.bat-", comp) else {
                    unresolved.push(f.component.clone());
                    continue;
                };
                if field == "capacity_fade" {
                    Target::BatteryCapacity(n)
                } else {
                    Target::BatteryResistance(n)
                }
            }
            ("deep::electrical::sources::StaticInverter", _) => Target::StaticInverter,
            ("deep::electrical::sources::Rat", _) => Target::Rat,
            ("deep::electrical::sources::GroundPower", _) => Target::Gpu,
            _ => {
                unresolved.push(f.model_field.clone());
                continue;
            }
        };
        out.push((f.id, target));
    }
    (out, unresolved)
}

/// The `deep::breakers` failure id that describes the *same physical
/// channel* as this area's own `network::Breaker.faults.fails_to_trip`.
///
/// The two areas each catalogued the aircraft's breakers, so one physical
/// device carries two ids for "the contacts have welded and it can no
/// longer open": `24_elec.bkr.<id>`'s `fails_to_trip` here and
/// `17_breakers.<id>`'s `contact_resistance` there. The live layer takes the
/// larger of the two so the duplication cannot mask itself -- arming either
/// id welds the one real breaker, rather than arming the ELMS one leaving
/// this area's own (lower-rated, so always first to act) element free to
/// isolate the fault anyway.
fn welded_channel_pairs() -> Vec<(usize, u64)> {
    let mut reg = Registry::default();
    crate::deep::breakers::registry::register(&mut reg);
    let mut out = Vec::new();
    for f in &reg.failures {
        if !f.model_field.contains("contact_resistance") {
            continue;
        }
        let Some(id) = f.component.strip_prefix("17_breakers.") else { continue };
        if let Some(&idx) = topology().breaker_index.get(id) {
            out.push((idx, f.id));
        }
    }
    out
}

// ---------------------------------------------------------------------

/// Var-name tag for one bus, matching the names this area's `registry.rs`
/// already cites in its ECAM triggers (`ELEC_AC_1_BUS_POTENTIAL`, ...) and
/// the plugin's own existing `A32NX_ELEC_*` spelling (`src/study/elec.rs`).
fn bus_tag(bus: BusId) -> &'static str {
    match bus {
        BusId::Ac1 => "AC_1",
        BusId::Ac2 => "AC_2",
        BusId::Ac3 => "AC_3",
        BusId::Ac4 => "AC_4",
        BusId::AcEss => "AC_ESS",
        BusId::AcEssShed => "AC_ESS_SHED",
        BusId::AcEmer => "AC_EMER",
        BusId::AcGndFltSvc => "AC_GND_FLT_SVC",
        BusId::Dc1 => "DC_1",
        BusId::Dc2 => "DC_2",
        BusId::DcEss => "DC_ESS",
        BusId::DcEssShed => "DC_ESS_SHED",
        BusId::DcBat => "DC_BAT",
        BusId::DcHot1 => "DC_HOT_1",
        BusId::DcHot2 => "DC_HOT_2",
        BusId::DcApu => "DC_APU",
        BusId::DcGndFltSvc => "DC_GND_FLT_SVC",
    }
}

// ---------------------------------------------------------------------
// Authority: this area's level-2 couplings into FlyByWire's own failures.
//
// `docs/deep/authority.md`: the deep model is authoritative, and expresses
// its authority through the coarsest FlyByWire input that can carry the
// verdict. For everything below FlyByWire's resolution -- every load's
// current, every breaker's I^2t state, every contactor, every battery,
// the RAT, ground power -- there is nothing to couple: `a380_systems` has
// no concept of them, so the published variables above stand alone. What
// follows is the rest: the components FlyByWire models too, and the exact
// condition in this model that says one of them has failed.
//
// Ids are `crate::failures::a380_failures()`'s own, in that function's
// order; the FlyByWire type each one activates is named beside it.

/// `FailureType::Generator(1..4)` -- `engine_generator.rs`'s own failure,
/// which stops the machine providing any potential at all.
const FBW_GENERATOR: [u64; 4] = [24_020, 24_021, 24_022, 24_023];
/// `FailureType::ApuGenerator(1..2)` (`pw980.rs`).
const FBW_APU_GENERATOR: [u64; 2] = [24_030, 24_031];
/// `FailureType::TransformerRectifier(1..4)`. FlyByWire's A380 numbers
/// them TR1, TR2, TR ESS, TR APU (`a380_systems/src/electrical/
/// direct_current.rs:96-110` builds BCRUs 1/2/3 as tr_1/tr_2/tr_ess and
/// `alternating_current.rs:67` builds number 4 as tr_apu) -- exactly this
/// area's own `src_tr` order.
const FBW_TR: [u64; 4] = [24_000, 24_001, 24_002, 24_003];
/// `FailureType::StaticInverter` (`static_inverter.rs`).
const FBW_STATIC_INVERTER: u64 = 24_004;

/// The registry component id behind each of those, so a derived failure
/// can always say which deep component concluded it.
const VFG_COMPONENT: [&str; 4] = ["24_elec.vfg-1", "24_elec.vfg-2", "24_elec.vfg-3", "24_elec.vfg-4"];
const APU_GEN_COMPONENT: [&str; 2] = ["24_elec.apu-gen-1", "24_elec.apu-gen-2"];
const TR_COMPONENT: [&str; 4] = ["24_elec.tr-1", "24_elec.tr-2", "24_elec.tr-ess", "24_elec.tr-apu"];
const STATIC_INVERTER_COMPONENT: &str = "24_elec.static-inv";
/// `registry.rs`'s own bus component ids, in [`ALL_BUS_IDS`] order.
const BUS_COMPONENT: [&str; 17] = [
    "24_elec.bus.AC1",
    "24_elec.bus.AC2",
    "24_elec.bus.AC3",
    "24_elec.bus.AC4",
    "24_elec.bus.AC_ESS",
    "24_elec.bus.AC_ESS_SHED",
    "24_elec.bus.AC_EMER",
    "24_elec.bus.AC_GND_FLT_SVC",
    "24_elec.bus.DC1",
    "24_elec.bus.DC2",
    "24_elec.bus.DC_ESS",
    "24_elec.bus.DC_ESS_SHED",
    "24_elec.bus.DC_BAT",
    "24_elec.bus.DC_HOT1",
    "24_elec.bus.DC_HOT2",
    "24_elec.bus.DC_APU",
    "24_elec.bus.DC_GND_FLT_SVC",
];

/// The `FailureType::ElectricalBus` id for a deep bus, where FlyByWire
/// models the same busbar. A failed `ElectricalBus` is non-conductive
/// (`electrical/mod.rs:145-157`'s `is_conductive`), which is exactly what
/// a bus isolated behind its own tripped feeder breaker is.
///
/// Four of the seventeen have no FlyByWire counterpart and stay level 1:
///
/// * `AcEssShed`/`DcEssShed` -- FlyByWire's A380 has no separate shed
///   busbar. It reuses `AlternatingCurrentEssentialShed` for the *AC ESS*
///   bus itself (400XP) and `AlternatingCurrentEssential` for AC EMER
///   (491XP), both with a `// TODO` saying so at
///   `alternating_current.rs:46-56`; its DC ESS sub-bus (`108PH`) is not
///   in the registered failure catalogue at all. Mapping a shed bus onto
///   either of those would fail the bus it sheds *from*.
/// * `DcBat` -- the A380 has no DC BAT busbar in FlyByWire's model; each
///   battery sits on its own hot bus, which is coupled below.
/// * The `AcGndFltSvc`/`DcGndFltSvc` pair *is* coupled; it is the ESS
///   shed pair and DC BAT that are not.
fn fbw_bus_failure(bus: BusId) -> Option<u64> {
    Some(match bus {
        BusId::Ac1 => 24_100,
        BusId::Ac2 => 24_101,
        BusId::Ac3 => 24_102,
        BusId::Ac4 => 24_103,
        // FlyByWire's `AlternatingCurrentEssentialShed` *is* its AC ESS
        // bus, and `AlternatingCurrentEssential` its AC EMER bus -- see
        // this function's own doc, and `crate::failures::failure_name`,
        // which names 24_104 "AC EMER" and 24_105 "AC ESS" for exactly
        // that reason.
        BusId::AcEss => 24_105,
        BusId::AcEmer => 24_104,
        BusId::AcGndFltSvc => 24_107,
        BusId::Dc1 => 24_108,
        BusId::Dc2 => 24_109,
        BusId::DcEss => 24_110,
        BusId::DcHot1 => 24_113,
        BusId::DcHot2 => 24_114,
        // `DirectCurrentNamed("309PP")`, the APU battery bus, is what
        // FlyByWire's TR APU feeds (`direct_current.rs:204-207`) -- the
        // same busbar this model calls `DcApu`.
        BusId::DcApu => 24_112,
        BusId::DcGndFltSvc => 24_117,
        BusId::AcEssShed | BusId::DcEssShed | BusId::DcBat => return None,
    })
}

/// How far a continuously degraded machine has to be gone before the
/// verdict "this machine has failed" is worth sending to FlyByWire, whose
/// own generator/TR/inverter failures are binary.
///
/// **GENERIC**, and unavoidably so: `authority.md`'s stated granularity
/// limit is that a partly degraded component either rounds to a trip or
/// stays invisible at level 2. Half is the rounding point at which the
/// machine has lost more of its health parameter's range than it has left;
/// below it the deep model keeps the real degradation (a stator that sags
/// harder under load is still a real, published, ELEC-page-visible thing)
/// and FlyByWire is told nothing.
const DEGRADED_BEYOND_HALF: f64 = 0.5;

/// Why a machine-level verdict was reached, as the Study page shows it.
/// One of a fixed set, chosen in `tick`, so that a derived failure always
/// names its own cause rather than a disjunction of everything it could
/// have been.
const REASON_OVERLOAD_TRIPPED: &str = "its own I^2t overload element ran to the trip and took it off line";
const REASON_REGULATOR_OUT_OF_BAND: &str = "driven and excited, but regulating outside MIL-STD-704F's 108-118 V band";
const REASON_WINDING_DEGRADED: &str = "stator/winding degraded past half its range: it can no longer hold rated voltage under load";
const REASON_TRU_DEGRADED: &str = "rectifier winding degraded past half its range toward its own degraded internal resistance";
const REASON_INVERTER_DEGRADED: &str = "conversion efficiency degraded past half its range toward its own floor";
const REASON_FEEDER_OPEN: &str = "the feeder breaker protecting this bus has tripped open";
/// The healthy case still has to carry a string; nothing reads it, because
/// a zero-magnitude coupling never leaves `Deep::tick`.
const REASON_HEALTHY: &str = "healthy";

/// Every level-2 coupling this area owns, as `(FlyByWire failure id, deep
/// component)`, in the order [`ElectricalLive::each_coupling`] emits them.
/// `new` turns it into the published variable names once, and a unit test
/// holds the two orders together.
fn coupling_table() -> Vec<(u64, &'static str)> {
    let mut v: Vec<(u64, &'static str)> = Vec::new();
    for i in 0..4 {
        v.push((FBW_GENERATOR[i], VFG_COMPONENT[i]));
    }
    for i in 0..2 {
        v.push((FBW_APU_GENERATOR[i], APU_GEN_COMPONENT[i]));
    }
    for i in 0..4 {
        v.push((FBW_TR[i], TR_COMPONENT[i]));
    }
    v.push((FBW_STATIC_INVERTER, STATIC_INVERTER_COMPONENT));
    for (bi, &bus) in ALL_BUS_IDS.iter().enumerate() {
        if let Some(id) = fbw_bus_failure(bus) {
            v.push((id, BUS_COMPONENT[bi]));
        }
    }
    v
}

/// Pre-built variable names: `publish` runs every frame and must never
/// format a string.
struct Names {
    bus_potential: Vec<String>,
    bus_powered: Vec<String>,
    bus_frequency: Vec<String>,
    gen_fault: Vec<String>,
    apu_gen_fault: Vec<String>,
    /// Real delivered power out of each engine/APU generator, W: the
    /// per-source figure `measured_gen_load_w`/`measured_apu_gen_load_w`
    /// already compute from each source's own Thevenin branch. Published
    /// because `deep::apu` reads `ELEC_APU_GEN_{1,2}_LOAD_W` for its
    /// generator wear and overload protection, and the only other thing
    /// this area published for those sources is a *breaker* current, which
    /// is the whole bus's, not the individual generator's.
    gen_load_w: Vec<String>,
    apu_gen_load_w: Vec<String>,
    tr_fault: Vec<String>,
    bat_fault: Vec<String>,
    bat_charge: Vec<String>,
    breaker_current: Vec<String>,
    breaker_closed: Vec<String>,
    load_powered: Vec<String>,
    /// One per entry of [`coupling_table`], in that order: the magnitude
    /// this area is presently asserting on FlyByWire's own failure of that
    /// id. Published so that a derived failure is never invisible -- the
    /// Study page reads these beside the crew's armed failures, which is
    /// `authority.md`'s "every level-2 coupling must be visible".
    derived: Vec<String>,
}

pub struct ElectricalLive {
    net: Network,
    catalog: Catalog,
    wiring: Wiring,
    shedding: SheddingRelays,

    routed: Vec<(u64, Target)>,
    welded_pairs: Vec<(usize, u64)>,
    faults_were_armed: bool,

    names: Names,

    // Cached indices.
    contactor: ContactorIndices,
    src_gen: [usize; 4],
    src_apu_gen: [usize; 2],
    src_tr: [usize; 4],
    src_bat: [usize; 2],
    src_gpu: usize,
    src_rat: usize,

    /// The 23 transit-only/one-shot loads this pass gives a real command to
    /// (see [`Self::command_transit_loads`]): per-engine ignition exciter
    /// lanes, the two engine fire bottles' squibs, the APU bottle's squibs,
    /// the APU start contactor, the two cargo door actuator-control
    /// circuits, and the six gear/gear-door actuators.
    ignition_load: [[usize; 2]; 4],
    eng_fire_squib: [[usize; 2]; 2],
    apu_fire_squib: [usize; 2],
    apu_start_contactor: usize,
    cargo_door_ctl: [usize; 2],
    /// `[nose, left, right]`, matching `Controls::gear_door_commanded_open`.
    gear_actuator: [usize; 3],
    gear_door_actuator: [usize; 3],

    /// The heavy AC motor loads that are simply not running when the
    /// aircraft has no generation on line at all -- see
    /// [`Self::command_emergency_shed`].
    emergency_shed_load: Vec<usize>,

    /// Breakers this area opened because `deep::breakers` commanded it, so
    /// the command releasing can close them again (and a breaker this
    /// area's own element tripped is left alone).
    externally_opened: Vec<bool>,

    /// One feeder breaker per bus, by bus index. `network::Network` measures
    /// their current for us but has no concept of what opening one *does*;
    /// in a real aircraft a feeder breaker sits in series with the bus's own
    /// supply, so when one opens this layer opens every contactor feeding
    /// that bus.
    feeder_breaker: [usize; 17],

    // Per-frame state for `publish`.
    report: NetworkReport,
    gen_fault: [bool; 4],
    apu_gen_fault: [bool; 2],
    tr_fault: [bool; 4],
    /// This frame's level-2 verdicts and the reason behind each
    /// (`docs/deep/authority.md`, [`ElectricalLive::each_coupling`]).
    /// Deliberately *not* the `*_fault` annunciations above: those are
    /// what the crew is shown and include bus-caused undervoltage, which
    /// is a symptom the machine may have no part in. A verdict handed to
    /// FlyByWire has to be about the machine itself, or a bus fault would
    /// leave a healthy generator failed on FlyByWire's side long after the
    /// bus recovered.
    gen_verdict: [Option<&'static str>; 4],
    apu_gen_verdict: [Option<&'static str>; 2],
    tr_verdict: [Option<&'static str>; 4],
    static_inv_verdict: Option<&'static str>,
    bat_fault: [bool; 2],
    bat_charge: [f64; 2],
    galley_shed: bool,
    commercial_shed: bool,
    galley_shed_commanded: bool,
    emergency_config: bool,
    total_demand_w: f64,
    capacity_w: f64,
    rat_deployed: bool,
    /// Whether the RAT deploy solenoid is presently commanded -- see
    /// `command_contactors`'s own note: true for exactly the tick the RAT
    /// transitions from stowed to deployed, never held on afterward.
    rat_solenoid_on: bool,

    // Measured feedback for the next frame's source models.
    measured_gen_load_w: [f64; 4],
    measured_apu_gen_load_w: [f64; 2],
    measured_tr_load_w: [f64; 4],
    measured_battery_current_a: [f64; 2],
}

struct ContactorIndices {
    gen_line: [usize; 4],
    apu_gen_line: [usize; 2],
    tr_line: [usize; 4],
    static_inv_line: usize,
    bat_direct: [usize; 2],
    gpu_line: usize,
    rat_line: usize,
    ac_tie: [usize; 3],
    ac_ess_feed_1: usize,
    ac_ess_feed_4: usize,
    ac_ess_shed: usize,
    ac_emer_to_ess: usize,
    dc_tie_1_2: usize,
    dc_ess_feed_1: usize,
    dc_ess_shed: usize,
    dc_bat_tie: usize,
    ac_gnd_svc_feed: usize,
    dc_gnd_svc_feed: usize,
}

/// This area's live system, constructed cold.
pub fn live_system() -> Box<dyn Area> {
    Box::new(ElectricalLive::new())
}

impl Default for ElectricalLive {
    fn default() -> Self {
        Self::new()
    }
}

impl ElectricalLive {
    pub fn new() -> Self {
        // A new electrical area is a new aircraft, and the board is the
        // typed channel this area shares with `deep::breakers` and
        // `deep::wiring`. Clearing it here rather than asking every caller
        // to remember is what makes it impossible to start a second flight
        // in one process on the previous aircraft's breaker currents, bus
        // voltages and harness damage -- see `board::clear`.
        board::clear();

        // Built in exactly the order `registry::register` builds it, so the
        // failure ids it assigns line up with this network's own indices.
        let mut net = Network::new();
        let catalog = loads::build(&mut net);
        let wiring = Wiring::build(&mut net, 15.0);

        // The bus ties `sources::Wiring::build` does not lay down: it wires
        // each source onto its own bus, but the A380's AC and DC
        // distribution also has the tie/feed contactors that let a bus that
        // has lost its own source be picked up from another, and the shed
        // contactors that drop the ESS SHED buses in emergency
        // configuration. Without them AC_ESS, AC_ESS_SHED, DC_ESS_SHED and
        // the ground-service buses could never be energised at all.
        // Bus-to-bus contactors are bidirectional in `network::relax`, which
        // is what a real tie bar is.
        let mut tie = |id: &'static str, kind: ContactorKind, from: BusId, to: BusId, ohm: f64| -> usize {
            net.add_contactor(Contactor::new(id, kind, FeedSource::Bus(from), to, ohm))
        };
        let ac_tie = [
            tie("ac-tie-1-2", ContactorKind::BusTie, BusId::Ac1, BusId::Ac2, AC_TIE_OHM),
            tie("ac-tie-2-3", ContactorKind::BusTie, BusId::Ac2, BusId::Ac3, AC_TIE_OHM),
            tie("ac-tie-3-4", ContactorKind::BusTie, BusId::Ac3, BusId::Ac4, AC_TIE_OHM),
        ];
        let ac_ess_feed_1 = tie("ac-ess-feed-1", ContactorKind::Feeder, BusId::Ac1, BusId::AcEss, ESS_FEEDER_OHM);
        let ac_ess_feed_4 = tie("ac-ess-feed-4", ContactorKind::Feeder, BusId::Ac4, BusId::AcEss, ESS_FEEDER_OHM);
        let ac_ess_shed = tie("ac-ess-shed", ContactorKind::Feeder, BusId::AcEss, BusId::AcEssShed, ESS_FEEDER_OHM);
        let ac_emer_to_ess = tie("ac-emer-to-ess", ContactorKind::Feeder, BusId::AcEmer, BusId::AcEss, ESS_FEEDER_OHM);
        let dc_tie_1_2 = tie("dc-tie-1-2", ContactorKind::BusTie, BusId::Dc1, BusId::Dc2, DC_FEEDER_OHM);
        let dc_ess_feed_1 = tie("dc-ess-feed-1", ContactorKind::Feeder, BusId::Dc1, BusId::DcEss, DC_FEEDER_OHM);
        let dc_ess_shed = tie("dc-ess-shed", ContactorKind::Feeder, BusId::DcEss, BusId::DcEssShed, DC_FEEDER_OHM);
        let dc_bat_tie = tie("dc-bat-tie", ContactorKind::Feeder, BusId::DcEss, BusId::DcBat, DC_FEEDER_OHM);
        let ac_gnd_svc_feed = tie("ac-gnd-svc-feed", ContactorKind::Feeder, BusId::Ac1, BusId::AcGndFltSvc, GND_SVC_FEEDER_OHM);
        let dc_gnd_svc_feed = tie("dc-gnd-svc-feed", ContactorKind::Feeder, BusId::Dc2, BusId::DcGndFltSvc, GND_SVC_FEEDER_OHM);

        // Transit-only actuators are dead unless something is driving them
        // (the gear/gear-door set, plus the gap-closing pass's own
        // ignition/squib/APU-start-contactor/cargo-door set -- see
        // `TRANSIT_ONLY_NO_TRUTH_INPUT`'s own doc). The RAT deploy solenoid
        // is also transit-only but is handled per-tick in `tick` instead,
        // against the real emergency/`rat_deployed` state this layer
        // already computes.
        for id in TRANSIT_ONLY_ACTUATORS.iter().copied().chain(TRANSIT_ONLY_NO_TRUTH_INPUT.iter().copied()) {
            if let Some(i) = net.load_index(id) {
                net.loads[i].commanded_on = false;
            }
        }

        // One feeder breaker per bus. `network::Network::add_feeder_breaker`
        // exists for exactly this and nothing used it: unlike a load's own
        // breaker, a feeder breaker carries the *whole* bus -- every load on
        // it plus that bus's own `short_to_ground` fault current -- which is
        // what makes a busbar-to-structure short a clearable fault instead
        // of something the generator simply feeds forever. (`registry.rs`'s
        // own effect text for that failure already says "can overload
        // whatever feeder/tie breaker protects it".)
        //
        // Rated the way any feeder is: above the connected load it has to
        // carry, with the same 25% margin `loads::rated_current` applies to
        // a load's own breaker, but never below the rating of the source
        // behind it -- a feeder cannot be the weak point of its own
        // generator's output.
        let mut feeder_breaker = [0usize; 17];
        for &bus in ALL_BUS_IDS.iter() {
            let connected_a: f64 = net
                .loads
                .iter()
                .filter(|l| l.spec.bus == bus)
                .map(|l| l.spec.rated_power_w / (bus.nominal_voltage() * l.spec.power_factor.max(0.1)))
                .sum();
            let rated = (connected_a * 1.25).max(source_rating_a(bus));
            let id: &'static str = Box::leak(format!("feeder-{}", bus.label()).into_boxed_str());
            feeder_breaker[bus.index()] = net.add_feeder_breaker(super::network::Breaker::new(id, rated.max(5.0), bus), bus);
        }

        let find_c = |net: &Network, id: &str| net.contactor_index(id).unwrap_or_else(|| panic!("sources::Wiring::build no longer builds contactor {id}"));
        let find_s = |net: &Network, id: &str| net.sources.iter().position(|s| s.id == id).unwrap_or_else(|| panic!("sources::Wiring::build no longer builds source {id}"));

        let contactor = ContactorIndices {
            gen_line: [find_c(&net, "gen-1-line"), find_c(&net, "gen-2-line"), find_c(&net, "gen-3-line"), find_c(&net, "gen-4-line")],
            apu_gen_line: [find_c(&net, "apu-gen-1-line"), find_c(&net, "apu-gen-2-line")],
            tr_line: [find_c(&net, "tr-1-line"), find_c(&net, "tr-2-line"), find_c(&net, "tr-ess-line"), find_c(&net, "tr-apu-line")],
            static_inv_line: find_c(&net, "static-inv-line"),
            bat_direct: [find_c(&net, "bat-1-direct"), find_c(&net, "bat-2-direct")],
            gpu_line: find_c(&net, "gpu-line"),
            rat_line: find_c(&net, "rat-line"),
            ac_tie,
            ac_ess_feed_1,
            ac_ess_feed_4,
            ac_ess_shed,
            ac_emer_to_ess,
            dc_tie_1_2,
            dc_ess_feed_1,
            dc_ess_shed,
            dc_bat_tie,
            ac_gnd_svc_feed,
            dc_gnd_svc_feed,
        };
        let src_gen = [find_s(&net, "gen-1"), find_s(&net, "gen-2"), find_s(&net, "gen-3"), find_s(&net, "gen-4")];
        let src_apu_gen = [find_s(&net, "apu-gen-1"), find_s(&net, "apu-gen-2")];
        let src_tr = [find_s(&net, "tr-1"), find_s(&net, "tr-2"), find_s(&net, "tr-ess"), find_s(&net, "tr-apu")];
        let src_bat = [find_s(&net, "bat-1"), find_s(&net, "bat-2")];
        let src_gpu = find_s(&net, "gpu");
        let src_rat = find_s(&net, "rat");

        let find_l = |net: &Network, id: &str| net.load_index(id).unwrap_or_else(|| panic!("loads::build no longer builds load {id}"));
        let ignition_load: [[usize; 2]; 4] = std::array::from_fn(|e| {
            let n = e + 1;
            [find_l(&net, &format!("ignition-{n}a")), find_l(&net, &format!("ignition-{n}b"))]
        });
        let eng_fire_squib: [[usize; 2]; 2] = std::array::from_fn(|bottle| {
            let b = bottle + 1;
            [find_l(&net, &format!("eng-fire-bottle-{b}-squib-1")), find_l(&net, &format!("eng-fire-bottle-{b}-squib-2"))]
        });
        let apu_fire_squib = [find_l(&net, "apu-fire-bottle-squib-1"), find_l(&net, "apu-fire-bottle-squib-2")];
        let apu_start_contactor = find_l(&net, "apu-start-contactor");
        let cargo_door_ctl = [find_l(&net, "cargo-door-fwd-actuator-ctl"), find_l(&net, "cargo-door-aft-actuator-ctl")];
        let gear_actuator = [find_l(&net, "gear-actuator-nose"), find_l(&net, "gear-actuator-left"), find_l(&net, "gear-actuator-right")];
        let gear_door_actuator = [find_l(&net, "gear-door-actuator-nose"), find_l(&net, "gear-door-actuator-left"), find_l(&net, "gear-door-actuator-right")];

        // The two families of heavy AC motor load that a real A380 does not
        // run on emergency electrical power (see
        // `command_emergency_shed`). Resolved once here rather than by
        // name every frame; every id is one `loads.rs` builds, so a
        // renaming there fails this lookup loudly instead of silently
        // shedding nothing.
        let emergency_shed_load: Vec<usize> = ["hyd-epump-ga", "hyd-epump-gb", "hyd-epump-ya", "hyd-epump-yb"]
            .iter()
            .map(|id| find_l(&net, id))
            .chain((0..25).map(|i| find_l(&net, &format!("fuel-pump-{i}"))))
            .collect();

        let (routed, unresolved) = route_failures(&net);
        debug_assert!(unresolved.is_empty(), "unrouted registered failures: {unresolved:?}");

        let names = Names {
            bus_potential: ALL_BUS_IDS.iter().map(|&b| format!("ELEC_{}_BUS_POTENTIAL", bus_tag(b))).collect(),
            bus_powered: ALL_BUS_IDS.iter().map(|&b| format!("ELEC_{}_BUS_IS_POWERED", bus_tag(b))).collect(),
            bus_frequency: ALL_BUS_IDS.iter().map(|&b| format!("ELEC_{}_BUS_FREQUENCY", bus_tag(b))).collect(),
            gen_fault: (1..=4).map(|n| format!("ELEC_GEN_{n}_FAULT")).collect(),
            apu_gen_fault: (1..=2).map(|n| format!("ELEC_APU_GEN_{n}_FAULT")).collect(),
            gen_load_w: (1..=4).map(|n| format!("ELEC_ENG_GEN_{n}_LOAD_W")).collect(),
            apu_gen_load_w: (1..=2).map(|n| format!("ELEC_APU_GEN_{n}_LOAD_W")).collect(),
            tr_fault: ["1", "2", "ESS", "APU"].iter().map(|s| format!("ELEC_TR_{s}_FAULT")).collect(),
            bat_fault: (1..=2).map(|n| format!("ELEC_BAT_{n}_FAULT")).collect(),
            bat_charge: (1..=2).map(|n| format!("ELEC_BAT_{n}_CHARGE_FRACTION")).collect(),
            breaker_current: net.breakers.iter().map(|b| format!("ELEC_BKR_{}_CURRENT_A", b.id)).collect(),
            breaker_closed: net.breakers.iter().map(|b| format!("ELEC_BKR_{}_CLOSED", b.id)).collect(),
            load_powered: net.loads.iter().map(|l| format!("ELEC_LOAD_{}_POWERED", l.spec.id)).collect(),
            derived: coupling_table().into_iter().map(|(id, _)| format!("DEEP_DERIVED_FBW_FAILURE_{id}")).collect(),
        };

        let n_breakers = net.breakers.len();
        Self {
            net,
            catalog,
            wiring,
            shedding: SheddingRelays::new(),
            routed,
            welded_pairs: welded_channel_pairs(),
            faults_were_armed: false,
            names,
            contactor,
            src_gen,
            src_apu_gen,
            src_tr,
            src_bat,
            src_gpu,
            src_rat,
            ignition_load,
            eng_fire_squib,
            apu_fire_squib,
            apu_start_contactor,
            cargo_door_ctl,
            gear_actuator,
            gear_door_actuator,
            emergency_shed_load,
            externally_opened: vec![false; n_breakers],
            feeder_breaker,
            report: NetworkReport::default(),
            gen_fault: [false; 4],
            apu_gen_fault: [false; 2],
            tr_fault: [false; 4],
            gen_verdict: [None; 4],
            apu_gen_verdict: [None; 2],
            tr_verdict: [None; 4],
            static_inv_verdict: None,
            bat_fault: [false; 2],
            bat_charge: [1.0; 2],
            galley_shed: false,
            commercial_shed: false,
            galley_shed_commanded: false,
            emergency_config: false,
            total_demand_w: 0.0,
            capacity_w: 0.0,
            rat_deployed: false,
            rat_solenoid_on: false,
            measured_gen_load_w: [0.0; 4],
            measured_apu_gen_load_w: [0.0; 2],
            measured_tr_load_w: [0.0; 4],
            measured_battery_current_a: [0.0; 2],
        }
    }

    /// Read-only access for tests and for whatever Study page wants the
    /// whole solved network rather than the published summary.
    pub fn network(&self) -> &Network {
        &self.net
    }

    fn clear_model_faults(&mut self) {
        for l in &mut self.net.loads {
            l.faults = Default::default();
        }
        for b in &mut self.net.breakers {
            b.faults = Default::default();
        }
        for c in &mut self.net.contactors {
            c.faults = Default::default();
        }
        for d in &mut self.net.diodes {
            d.faults = Default::default();
        }
        for b in &mut self.net.buses {
            b.faults = Default::default();
        }
    }

    /// Feeds every armed magnitude into the exact model field its
    /// `FailureDef::model_field` names, and returns the source-side ones,
    /// which `sources::Wiring` takes as arguments rather than owning.
    fn apply_faults(&mut self, faults: &Faults) -> SourceFaults {
        let mut sf = SourceFaults::default();
        for i in 0..self.routed.len() {
            let (id, target) = self.routed[i];
            let m = faults.get(id);
            if m <= 0.0 {
                continue;
            }
            match target {
                Target::Load(idx, field) => {
                    let f = &mut self.net.loads[idx].faults;
                    let slot = match field {
                        LoadField::OpenCircuit => &mut f.open_circuit,
                        LoadField::ShortToGround => &mut f.short_to_ground,
                        LoadField::HighResistance => &mut f.high_resistance,
                        LoadField::Intermittent => &mut f.intermittent,
                    };
                    *slot = slot.max(m);
                }
                Target::BreakerFailsToTrip(idx) => {
                    let s = &mut self.net.breakers[idx].faults.fails_to_trip;
                    *s = s.max(m);
                }
                Target::BreakerNuisance(idx) => {
                    let s = &mut self.net.breakers[idx].faults.nuisance_trip;
                    *s = s.max(m);
                }
                Target::ContactorFailsToClose(idx) => {
                    let s = &mut self.net.contactors[idx].faults.fails_to_close;
                    *s = s.max(m);
                }
                Target::ContactorWelded(idx) => {
                    let s = &mut self.net.contactors[idx].faults.welded_closed;
                    *s = s.max(m);
                }
                Target::Diode(idx) => {
                    let s = &mut self.net.diodes[idx].faults.open_circuit;
                    *s = s.max(m);
                }
                Target::Bus(idx) => {
                    let s = &mut self.net.buses[idx].faults.short_to_ground;
                    *s = s.max(m);
                }
                Target::VfgWinding(n) => sf.vfg[n].winding_degradation = sf.vfg[n].winding_degradation.max(m),
                Target::VfgRegulator(n) => sf.vfg[n].regulator_drift = sf.vfg[n].regulator_drift.max(m),
                Target::ApuGenWinding(n) => sf.apu_gen[n].winding_degradation = sf.apu_gen[n].winding_degradation.max(m),
                Target::ApuGenRegulator(n) => sf.apu_gen[n].regulator_drift = sf.apu_gen[n].regulator_drift.max(m),
                Target::Tru(n) => sf.tru[n].winding_degradation = sf.tru[n].winding_degradation.max(m),
                Target::BatteryCapacity(n) => sf.battery[n].capacity_fade = sf.battery[n].capacity_fade.max(m),
                Target::BatteryResistance(n) => sf.battery[n].resistance_growth = sf.battery[n].resistance_growth.max(m),
                Target::StaticInverter => sf.static_inverter.efficiency_loss = sf.static_inverter.efficiency_loss.max(m),
                Target::Rat => sf.rat.jammed = sf.rat.jammed.max(m),
                Target::Gpu => sf.ground_power.weak_cart = sf.ground_power.weak_cart.max(m),
            }
        }
        // One physical breaker, two catalogued ids: take the worse.
        for i in 0..self.welded_pairs.len() {
            let (idx, id) = self.welded_pairs[i];
            let m = faults.get(id);
            if m > 0.0 {
                let s = &mut self.net.breakers[idx].faults.fails_to_trip;
                *s = s.max(m);
            }
        }
        sf
    }

    /// Folds `deep::wiring`'s per-circuit harness damage into the matching
    /// load's own `LoadFaults` (see this module's doc: the wiring area owns
    /// the harness, this one owns the solve).
    fn apply_wiring_damage(&mut self) {
        board::with_board(|b| {
            let n = self.net.loads.len();
            for i in 0..n.min(b.load_short.len()) {
                let f = &mut self.net.loads[i].faults;
                f.short_to_ground = f.short_to_ground.max(b.load_short[i]);
            }
            for i in 0..n.min(b.load_open.len()) {
                let f = &mut self.net.loads[i].faults;
                f.open_circuit = f.open_circuit.max(b.load_open[i]);
            }
            for i in 0..n.min(b.load_high_resistance.len()) {
                let f = &mut self.net.loads[i].faults;
                f.high_resistance = f.high_resistance.max(b.load_high_resistance[i]);
            }
        });
    }

    /// Applies `deep::breakers`' trip-unit decision to the contacts.
    fn apply_breaker_commands(&mut self) {
        board::with_board(|b| {
            let n = self.net.breakers.len().min(b.breaker_open_cmd.len());
            for i in 0..n {
                let commanded_open = b.breaker_open_cmd[i] > 0.5;
                if commanded_open {
                    if !self.externally_opened[i] {
                        self.net.breakers[i].pull();
                        self.externally_opened[i] = true;
                    }
                } else if self.externally_opened[i] {
                    // The trip unit has been reset; the contacts close again
                    // (and their own thermal element cools, as a real bimetal
                    // strip does once the current is gone).
                    self.net.breakers[i].reset();
                    self.externally_opened[i] = false;
                }
            }
        });
    }

    /// One frame of generator/bus-tie control, run *after*
    /// `Wiring::pre_step` has written this frame's source terminals, so
    /// every close decision can be taken against the voltage the source
    /// actually has rather than against last frame's guess.
    ///
    /// The one rule underneath all of it: **a source contactor never closes
    /// onto a source that is not producing.** That is what a real GCU/BCL
    /// does, and it matters here because `sources.rs` reduces a dead source
    /// (an unexcited generator, a TRU with no AC input, an undeployed RAT)
    /// to a 0 V Thevenin branch with near-zero internal resistance -- an
    /// ideal short, not an open circuit. Closing onto one would clamp its
    /// bus, and anything tied to that bus, to zero.
    ///
    /// Bus voltages, on the other hand, are necessarily last frame's: a
    /// voltage-sensing relay cannot act on a voltage it has not yet
    /// measured either.
    fn command_contactors(&mut self, truth: &Truth, gpu_plugged_in: bool) {
        let producing = |net: &Network, src: usize| net.sources[src].open_circuit_v > 0.0;

        // A VFG is on line when its own GCU sees a healthy machine: excited
        // (`sources::Vfg::CUT_IN_SPEED_FRACTION`) and not tripped off by its
        // own overload element -- and the crew has not pulled its own GEN
        // pushbutton, which is the GCU's own line contactor command, not a
        // switch this model gets to assume is always in ENGAGED.
        let mut gen_on_line = [false; 4];
        for i in 0..4 {
            let tripped = self.wiring.vfg[i].overload_heat() >= 1.0;
            gen_on_line[i] = truth.controls.eng_gen_pb_on[i] && truth.engine_running[i] && producing(&self.net, self.src_gen[i]) && !tripped;
            let idx = self.contactor.gen_line[i];
            self.net.contactors[idx].commanded_closed = gen_on_line[i];
        }

        let mut apu_on_line = false;
        for i in 0..2 {
            let on = truth.controls.apu_gen_pb_on[i] && truth.apu_running && producing(&self.net, self.src_apu_gen[i]) && self.wiring.apu_gen[i].overload_heat() < 1.0;
            apu_on_line |= on;
            self.net.contactors[self.contactor.apu_gen_line[i]].commanded_closed = on;
        }

        // The four VFGs are not paralleled on the A380: each feeds its own
        // bus. The ties close only when some AC bus has lost its own source
        // *and* there is something left on the tie bar to feed it from --
        // tying dead buses to each other would leave the tie bar with no
        // reference at all.
        let anchored = gen_on_line.iter().any(|&g| g) || apu_on_line || gpu_plugged_in;
        let need_tie = anchored && (gen_on_line.iter().any(|&g| !g) || apu_on_line || gpu_plugged_in);
        for &idx in &self.contactor.ac_tie {
            self.net.contactors[idx].commanded_closed = need_tie;
        }

        // AC ESS: normally from AC1, alternate from AC4.
        //
        // "Live" here means *fed by a real source*, not merely "at
        // voltage". The distinction matters because every bus-to-bus
        // contactor in this network is bidirectional (`network::relax`
        // conducts through a tie both ways, which is what a real tie bar
        // is), so the emergency AC bus's own inverter back-feeds AC ESS and
        // -- through a closed `ac-ess-feed-1` -- AC1 itself. Deciding
        // "AC1 is live" from the voltage alone therefore reads the
        // emergency configuration's own output as evidence that the
        // emergency configuration is not needed: the transfer logic
        // latches on to its own back-feed and the whole network
        // limit-cycles at the frame rate, one frame in emergency and the
        // next out of it, forever. (That is what it did on a cold and dark
        // aircraft: every AC and DC bus flapping between dead and alive on
        // alternate frames, which restarted a dozen motor loads' switch-on
        // inrush every single frame and, entirely correctly, cooked their
        // breakers open on a genuinely sustained overcurrent.)
        //
        // `anchored` above is the honest statement of the same thing: at
        // least one generator, APU generator or ground-power unit is
        // actually on line onto the main AC network. A main AC bus is live
        // when that is true *and* the bus is at voltage -- the second term
        // still catches a bus isolated by its own feeder breaker with a
        // generator running behind it.
        let main_ac_anchored = anchored;
        let main_ac_at_voltage = [BusId::Ac1, BusId::Ac2, BusId::Ac3, BusId::Ac4].map(|b| self.net.bus(b).voltage >= AC_UNDERVOLTAGE_TRIP_V);
        let ac1_live = main_ac_anchored && main_ac_at_voltage[0];
        let ac4_live = main_ac_anchored && main_ac_at_voltage[3];
        self.net.contactors[self.contactor.ac_ess_feed_1].commanded_closed = ac1_live;
        self.net.contactors[self.contactor.ac_ess_feed_4].commanded_closed = !ac1_live && ac4_live;

        // Emergency configuration: no main AC bus is alive at all.
        let emergency = !main_ac_anchored || !main_ac_at_voltage.iter().any(|&live| live);
        self.emergency_config = emergency;

        // The RAT deploys on a total loss of main AC generation in flight.
        // `Rat::deploy` is instantaneous in this model (a stowed/deployed
        // bool, not a multi-second extension), so the one tick it flips
        // `false` to `true` is the *only* tick a real deploy solenoid would
        // be doing anything -- captured here, before the call, as the one
        // real signal `rat-deploy-solenoid`'s `commanded_on` is gated on
        // below, instead of the permanent "no Truth input" off every other
        // transit-only load in this pass gets.
        let was_rat_deployed = self.wiring.rat.deployed();
        if emergency && !truth.on_ground {
            self.wiring.rat.deploy();
        }
        self.rat_deployed = self.wiring.rat.deployed();
        self.rat_solenoid_on = emergency && !was_rat_deployed;
        if let Some(i) = self.net.load_index("rat-deploy-solenoid") {
            self.net.loads[i].commanded_on = self.rat_solenoid_on;
        }

        // The static inverter and the RAT feed the emergency AC bus, and
        // that bus backs the ESS bus up, only once each is actually
        // producing.
        let inv_live = producing(&self.net, self.contactor_source(self.contactor.static_inv_line));
        self.net.contactors[self.contactor.static_inv_line].commanded_closed = emergency && inv_live;
        let rat_live = self.rat_deployed && producing(&self.net, self.src_rat);
        self.net.contactors[self.contactor.rat_line].commanded_closed = rat_live;
        self.net.contactors[self.contactor.ac_emer_to_ess].commanded_closed = emergency && (inv_live || rat_live);

        // The shed buses drop in emergency configuration; that is what makes
        // them shed buses.
        self.net.contactors[self.contactor.ac_ess_shed].commanded_closed = !emergency;
        self.net.contactors[self.contactor.dc_ess_shed].commanded_closed = !emergency;

        // A TRU with no AC input rectifies nothing: its output is an open
        // circuit, so its own feeder contactor opens rather than clamping
        // its DC bus to zero.
        let mut tr_live = [false; 4];
        for i in 0..4 {
            tr_live[i] = producing(&self.net, self.src_tr[i]);
            self.net.contactors[self.contactor.tr_line[i]].commanded_closed = tr_live[i];
        }
        // The essential DC bus is normally carried by DC1 as well as its own
        // TR -- the real Airbus arrangement, and the reason one TR's rating
        // does not have to cover the whole essential load on its own.
        self.net.contactors[self.contactor.dc_ess_feed_1].commanded_closed = tr_live[0];
        // The DC tie picks a dead DC bus up from the live one; the battery
        // bus is tied to DC ESS only while DC ESS is itself being held up,
        // so a dead essential bus can never drain the batteries through it.
        self.net.contactors[self.contactor.dc_tie_1_2].commanded_closed = tr_live[0] != tr_live[1];
        self.net.contactors[self.contactor.dc_bat_tie].commanded_closed = tr_live[2] || tr_live[0];
        // The battery-direct contactor is the real BAT pushbutton's own
        // contactor (`ContactorKind::BatteryDirect`) -- AUTO closes it onto
        // the battery/hot bus, OFF opens it, exactly the real Airbus
        // behaviour that a battery pushbutton switched off also drops its
        // own hot bus (not merely the battery's contribution to the main DC
        // network). `Controls::default`'s normal position is AUTO, so a
        // cold aircraft with nobody touching the pushbuttons still has both
        // hot buses alive.
        for i in 0..2 {
            self.net.contactors[self.contactor.bat_direct[i]].commanded_closed = truth.controls.bat_pb_auto[i];
        }

        // The ground/flight service buses carry cargo handling, service
        // lighting and the cabin-servicing outlets; they are energised on
        // the ground (from the cart, or from AC1 once a generator is
        // running) and dead in flight, which is what makes them *ground*
        // service buses.
        self.net.contactors[self.contactor.gpu_line].commanded_closed = gpu_plugged_in && producing(&self.net, self.src_gpu);
        self.net.contactors[self.contactor.ac_gnd_svc_feed].commanded_closed = truth.on_ground && ac1_live;
        self.net.contactors[self.contactor.dc_gnd_svc_feed].commanded_closed = truth.on_ground && tr_live[1];

        // A feeder breaker that has opened isolates its bus: it is in series
        // with that bus's supply, so nothing may feed the bus through it
        // until it is reset. This is the step `network::Network` cannot take
        // on its own -- it measures a feeder breaker's current but has no
        // model of what one being open means.
        for bi in 0..17 {
            if !self.net.breakers[self.feeder_breaker[bi]].closed {
                let bus = ALL_BUS_IDS[bi];
                for c in &mut self.net.contactors {
                    // Either end: a bus-to-bus tie is one device, and
                    // `network::relax` conducts through it both ways.
                    if c.to == bus || c.from == FeedSource::Bus(bus) {
                        c.commanded_closed = false;
                    }
                }
            }
        }
    }

    /// The 17 avionics/pyro transit-only loads this pass gives a real
    /// command, plus the 6 gear/gear-door actuators -- see this struct's
    /// own field doc. Every one of the 23 now has a real signal behind it;
    /// none remain permanently off.
    fn command_transit_loads(&mut self, truth: &Truth) {
        let c = &truth.controls;

        // Ignition exciters: a real igniter fires high-tension sparks only
        // while its own engine is actually being cranked -- the same
        // `starter_engaged` condition `physics::engine`'s own starter model
        // gates on internally (see `Controls::starter_engaged`'s own doc).
        for eng in 0..4 {
            let on = c.starter_engaged[eng];
            self.net.loads[self.ignition_load[eng][0]].commanded_on = on;
            self.net.loads[self.ignition_load[eng][1]].commanded_on = on;
        }

        // Engine fire bottle squibs: `ata26_extinguishing`'s own doc cites
        // "a real wide-body cross-feed fire-extinguishing architecture" --
        // bottle 1 is the first shot, bottle 2 the second, cross-fed to
        // whichever engine's fire handle is pulled and whose crew is
        // pressing that shot's AGENT pushbutton. A squib only has a path to
        // fire once the handle has rotated the transfer valve open, so both
        // are required, not the agent pushbutton alone.
        for bottle in 0..2 {
            let fired = (0..4).any(|eng| c.fire_pb_released[eng] && c.fire_agent_pb_pressed[eng][bottle]);
            self.net.loads[self.eng_fire_squib[bottle][0]].commanded_on = fired;
            self.net.loads[self.eng_fire_squib[bottle][1]].commanded_on = fired;
        }

        // APU fire bottle: one agent pushbutton, its two squibs redundant
        // initiators on the same one-shot bottle, so both fire together.
        let apu_fired = c.fire_pb_apu_released && c.fire_agent_pb_apu_pressed;
        self.net.loads[self.apu_fire_squib[0]].commanded_on = apu_fired;
        self.net.loads[self.apu_fire_squib[1]].commanded_on = apu_fired;

        // APU start contactor: energised for exactly as long as the crew is
        // holding a real start command.
        self.net.loads[self.apu_start_contactor].commanded_on = c.apu_start_pb_on;

        // Cargo door actuator control: this catalogue models one cargo door
        // class (`registry.rs`'s own "registers one of each as the class"),
        // and `deep::cabin` likewise publishes one channel for it
        // (`CABIN_CARGO_DOOR_PERCENT:1`/`CABIN_CARGO_DOOR_CMD:1`, one frame
        // behind through `Truth::published`), so both the forward and aft
        // control circuits share it. The circuit draws while the door has
        // not yet reached the target the cabin crew's own switch gave it --
        // i.e. while it is actually travelling -- not merely while
        // commanded, which is what a control circuit (as opposed to the
        // door's own drive motor) genuinely does.
        let cargo_pos = truth.published.get_or("CABIN_CARGO_DOOR_PERCENT:1", 0.0);
        let cargo_cmd = truth.published.get_or("CABIN_CARGO_DOOR_CMD:1", 0.0);
        let cargo_moving = (cargo_cmd - cargo_pos).abs() > CARGO_DOOR_TRAVEL_MARGIN_PERCENT;
        self.net.loads[self.cargo_door_ctl[0]].commanded_on = cargo_moving;
        self.net.loads[self.cargo_door_ctl[1]].commanded_on = cargo_moving;

        // Gear and gear-door actuators: FlyByWire's own commanded door
        // position (`Controls::gear_door_commanded_open`) is real and
        // continuous; an intermediate value is the door -- and, on this
        // catalogue's simplified one-actuator-per-leg model, the gear it
        // travels with -- actually driving rather than parked at an end
        // stop.
        for i in 0..3 {
            let pos = c.gear_door_commanded_open[i];
            let in_transit = pos > GEAR_TRANSIT_MARGIN_FRACTION && pos < 1.0 - GEAR_TRANSIT_MARGIN_FRACTION;
            self.net.loads[self.gear_door_actuator[i]].commanded_on = in_transit;
            self.net.loads[self.gear_actuator[i]].commanded_on = in_transit;
        }
    }

    /// The heavy AC motor loads that stop when the aircraft loses all
    /// generation and is left on its batteries, static inverter and (in
    /// flight) the RAT.
    ///
    /// The four electric hydraulic pumps (2.1 kW each) and the 25 AC fuel
    /// boost pumps (600 W each) are, between them, about 23 kW of motor
    /// load sitting on the AC buses. A static inverter is a few hundred
    /// VA. On a real A380 in emergency electrical configuration neither
    /// family is running at all -- the hydraulic systems are on their
    /// engine-driven pumps (and, if it is out, the RAT), and the fuel
    /// system is on suction feed; "fuel pumps lost" is part of what an
    /// ELEC EMER CONFIG actually means to a crew.
    ///
    /// Without this the model asked the inverter for all 23 kW: every one
    /// of those motors would drop out on under-voltage, wait out its own
    /// lockout, restart together, collapse the essential buses again, and
    /// repeat -- a real cyclic overload, which correctly (and, on a cold
    /// and dark aircraft, repeatedly) cooked the breaker of whatever else
    /// shared their bus. The shed is the load-side fix that condition
    /// asks for; nothing about any breaker's rating or curve changes.
    ///
    /// `capacity_w` is this area's own already-computed real generation on
    /// line, so the test is "is anything actually generating", not a
    /// voltage that the emergency sources themselves produce.
    fn command_emergency_shed(&mut self, generation_on_line_w: f64) {
        let shed = generation_on_line_w <= 0.0;
        for &i in &self.emergency_shed_load {
            self.net.loads[i].commanded_on = !shed;
        }
    }

    /// The source index a source-fed contactor draws from.
    fn contactor_source(&self, contactor: usize) -> usize {
        match self.net.contactors[contactor].from {
            FeedSource::Source(i) => i,
            FeedSource::Bus(_) => unreachable!("contactor {contactor} is bus-fed, not source-fed"),
        }
    }

    /// Available real generation capacity, W: what is actually on line.
    fn capacity_w(&self, truth: &Truth, gpu_plugged_in: bool) -> f64 {
        let mut cap = 0.0;
        for i in 0..4 {
            if self.net.contactors[self.contactor.gen_line[i]].commanded_closed && truth.engine_running[i] {
                cap += GEN_RATED_TRUE_POWER_W;
            }
        }
        for i in 0..2 {
            if self.net.contactors[self.contactor.apu_gen_line[i]].commanded_closed && truth.apu_running {
                cap += APU_GEN_RATED_TRUE_POWER_W;
            }
        }
        if gpu_plugged_in {
            cap += GPU_RATED_TRUE_POWER_W;
        }
        cap
    }

    /// The real branch current out of one source, A: its own Thevenin
    /// branch, `(V_oc - V_bus) / (R_source + R_contactor)`, zero when its
    /// contactor is open. Positive out of the source.
    fn source_branch_current_a(&self, src: usize, contactor: usize) -> f64 {
        let c = &self.net.contactors[contactor];
        if !c.closed {
            return 0.0;
        }
        let s = &self.net.sources[src];
        let r = (c.resistance_ohm + s.resistance_ohm).max(1.0e-6);
        (s.open_circuit_v - self.net.bus(c.to).voltage) / r
    }

    fn source_delivered_w(&self, src: usize, contactor: usize) -> f64 {
        let i = self.source_branch_current_a(src, contactor);
        if i <= 0.0 {
            return 0.0;
        }
        i * self.net.bus(self.net.contactors[contactor].to).voltage
    }

    /// This area's whole level-2 coupling table, with this frame's verdict
    /// on each entry (`docs/deep/authority.md`). Emitted in
    /// [`coupling_table`]'s order, healthy entries included at magnitude
    /// `0.0`, so that both `publish` and `derived_failures` walk one list
    /// and the set is a level rather than an event.
    ///
    /// Every verdict here is a *machine*-level one -- the machine's own
    /// overload element, its own regulated terminal, its own health
    /// parameter, or, for a bus, its own feeder breaker. None of them is a
    /// bus voltage: this area's published `ELEC_*_FAULT` annunciations do
    /// include bus-caused undervoltage, because that is what the crew is
    /// shown, but a verdict handed to FlyByWire must not, or a bus fault
    /// would leave a healthy generator failed on FlyByWire's side after
    /// the bus recovered.
    fn each_coupling(&self, out: &mut dyn FnMut(DerivedFailure)) {
        let verdict = |v: Option<&'static str>| (if v.is_some() { 1.0 } else { 0.0 }, v.unwrap_or(REASON_HEALTHY));
        for i in 0..4 {
            let (magnitude, reason) = verdict(self.gen_verdict[i]);
            out(DerivedFailure { fbw_id: FBW_GENERATOR[i], magnitude, deep_component: VFG_COMPONENT[i], reason });
        }
        for i in 0..2 {
            let (magnitude, reason) = verdict(self.apu_gen_verdict[i]);
            out(DerivedFailure { fbw_id: FBW_APU_GENERATOR[i], magnitude, deep_component: APU_GEN_COMPONENT[i], reason });
        }
        for i in 0..4 {
            let (magnitude, reason) = verdict(self.tr_verdict[i]);
            out(DerivedFailure { fbw_id: FBW_TR[i], magnitude, deep_component: TR_COMPONENT[i], reason });
        }
        let (magnitude, reason) = verdict(self.static_inv_verdict);
        out(DerivedFailure { fbw_id: FBW_STATIC_INVERTER, magnitude, deep_component: STATIC_INVERTER_COMPONENT, reason });
        for (bi, &bus) in ALL_BUS_IDS.iter().enumerate() {
            let Some(id) = fbw_bus_failure(bus) else { continue };
            // A feeder breaker is in series with everything that can feed
            // its bus, so a bus behind an open one is isolated no matter
            // what is running -- which is precisely what a failed
            // `ElectricalBus` is on FlyByWire's side. A bus that is merely
            // *unpowered* is not derived: FlyByWire works that out for
            // itself, and claiming it here would keep the bus dead after
            // its own source came back.
            let isolated = !self.net.breakers[self.feeder_breaker[bi]].closed;
            out(DerivedFailure {
                fbw_id: id,
                magnitude: if isolated { 1.0 } else { 0.0 },
                deep_component: BUS_COMPONENT[bi],
                reason: if isolated { REASON_FEEDER_OPEN } else { REASON_HEALTHY },
            });
        }
    }
}

/// The source-side fault magnitudes, which `sources::Wiring` takes as
/// arguments each tick rather than owning as state.
#[derive(Default)]
struct SourceFaults {
    vfg: [VfgFaults; 4],
    apu_gen: [ApuGeneratorFaults; 2],
    tru: [TruFaults; 4],
    battery: [BatteryFaults; 2],
    static_inverter: StaticInverterFaults,
    rat: RatFaults,
    ground_power: GroundPowerFaults,
}

impl Area for ElectricalLive {
    fn name(&self) -> &'static str {
        "electrical"
    }

    fn tick(&mut self, truth: &Truth, faults: &Faults) {
        let dt = truth.dt_s.max(0.0);

        // 1. Faults: clear, then re-apply every armed magnitude into the
        //    field its registry entry names. Skipped entirely on the common
        //    frame where nothing at all is armed.
        let armed = faults.any();
        let sf = if armed || self.faults_were_armed {
            self.clear_model_faults();
            self.apply_faults(faults)
        } else {
            SourceFaults::default()
        };
        self.faults_were_armed = armed;

        // 2. The other two areas' last frame.
        self.apply_wiring_damage();
        self.apply_breaker_commands();

        // 3. Sources first: every source's terminal for this frame, from
        //    `Truth` and last frame's measured feedback.
        //
        //    `engine_speed_fraction` is the VFG's own *core* speed, not the
        //    fan: `Truth::engine_n2_frac` is what the generator's own
        //    accessory-gearbox drive actually tracks, and it is what sets
        //    the generator's output frequency (never its regulated
        //    voltage) -- and, downstream of that, the direct-drive cabin/
        //    avionics-bay fans' own affinity-law power
        //    (`network::Load::frequency_multiplier`). Feeding fan speed
        //    here (as this used to) fed the wrong shaft into both.
        let gpu_plugged_in = truth.gpu_plugged_in;
        let inputs = WiringInputs {
            engine_speed_fraction: truth.engine_n2_frac,
            measured_gen_load_w: self.measured_gen_load_w,
            vfg_faults: sf.vfg,
            apu_speed_fraction: if truth.apu_running { 1.0 } else { 0.0 },
            measured_apu_gen_load_w: self.measured_apu_gen_load_w,
            apu_gen_faults: sf.apu_gen,
            tru_faults: sf.tru,
            measured_tr_load_w: self.measured_tr_load_w,
            battery_faults: sf.battery,
            measured_battery_current_a: self.measured_battery_current_a,
            static_inverter_faults: sf.static_inverter,
            gpu_plugged_in,
            ground_power_faults: sf.ground_power,
            airspeed_kt: truth.environment.tas_ms / 0.514_444,
            rat_faults: sf.rat,
            // The main avionics/equipment bay's own published temperature
            // (`deep::thermal_zones`, one frame behind through
            // `Truth::published`), not static air: at cruise SAT is around
            // -56 C, roughly 70 K colder than a ventilated avionics bay
            // actually runs, which would every tick have made every TRU/
            // VFG/battery thermal model think it was sitting in a walk-in
            // freezer. Falls back to a GENERIC 20 C (a ventilated bay's own
            // target range, not ambient) only until `thermal_zones` has
            // published its first frame.
            ambient_c: truth.published.get_or("THERMAL_ZONE_MAINAVIONICS_TEMPERATURE_C", EQUIPMENT_BAY_FALLBACK_C),
        };
        self.wiring.pre_step(&mut self.net, &inputs, dt);

        // 4. Contactor control against those fresh terminals, then the
        //    load-management shed decision, then the one solve.
        self.command_contactors(truth, gpu_plugged_in);
        self.command_transit_loads(truth);

        let capacity_w = self.capacity_w(truth, gpu_plugged_in);
        self.command_emergency_shed(capacity_w);
        let budget = power_budget(&self.net, capacity_w);
        // Schmitt band, so the relay cannot chatter at the balance point.
        self.galley_shed_commanded = if self.galley_shed_commanded {
            budget.margin_w < capacity_w * SHED_RELEASE_MARGIN_FRACTION
        } else {
            budget.overloaded
        };
        let shed_inputs = ShedInputs { galley_shed_commanded: self.galley_shed_commanded, emergency_config_commanded: self.emergency_config };
        let shed = self.shedding.step(&mut self.net, &self.catalog, &shed_inputs);
        self.galley_shed = shed.galley_shed;
        self.commercial_shed = shed.commercial_shed;
        self.capacity_w = capacity_w;
        self.total_demand_w = budget.total_demand_w;

        self.report = self.net.step(dt);
        for i in 0..4 {
            self.measured_gen_load_w[i] = self.source_delivered_w(self.src_gen[i], self.contactor.gen_line[i]);
        }
        for i in 0..2 {
            self.measured_apu_gen_load_w[i] = self.source_delivered_w(self.src_apu_gen[i], self.contactor.apu_gen_line[i]);
            self.measured_battery_current_a[i] = self.source_branch_current_a(self.src_bat[i], self.contactor.bat_direct[i]);
        }
        for i in 0..4 {
            self.measured_tr_load_w[i] = self.source_delivered_w(self.src_tr[i], self.contactor.tr_line[i]);
        }
        self.wiring.post_step(&self.net, &inputs, dt);

        // 5. Protective annunciations, all read off the solved network.
        let dc_trip = dc_undervoltage_trip_v();
        for i in 0..4 {
            let on_line = self.net.contactors[self.contactor.gen_line[i]].closed;
            let bus_v = self.net.bus(self.net.contactors[self.contactor.gen_line[i]].to).voltage;
            self.gen_fault[i] = self.wiring.vfg[i].overload_heat() >= 1.0 || (on_line && (bus_v < AC_UNDERVOLTAGE_TRIP_V || bus_v > AC_OVERVOLTAGE_TRIP_V));
        }
        for i in 0..2 {
            let on_line = self.net.contactors[self.contactor.apu_gen_line[i]].closed;
            let bus_v = self.net.bus(self.net.contactors[self.contactor.apu_gen_line[i]].to).voltage;
            self.apu_gen_fault[i] = self.wiring.apu_gen[i].overload_heat() >= 1.0 || (on_line && (bus_v < AC_UNDERVOLTAGE_TRIP_V || bus_v > AC_OVERVOLTAGE_TRIP_V));
        }
        // A TR faults when it has an AC input but is no longer holding its
        // own DC bus up -- the real TR FAULT condition.
        let tr_ac_in = [BusId::Ac1, BusId::Ac2, BusId::AcEss, BusId::AcEss];
        let tr_dc_out = [BusId::Dc1, BusId::Dc2, BusId::DcEss, BusId::DcApu];
        for i in 0..4 {
            let ac_powered = self.net.bus(tr_ac_in[i]).voltage > 90.0;
            self.tr_fault[i] = ac_powered && self.net.bus(tr_dc_out[i]).voltage < dc_trip;
        }
        for i in 0..2 {
            let (v, _r) = self.wiring.battery[i].terminal(sf.battery[i]);
            self.bat_fault[i] = self.net.contactors[self.contactor.bat_direct[i]].closed && v < dc_trip;
            self.bat_charge[i] = self.wiring.battery[i].charge_fraction(sf.battery[i]);
        }
        // 6. The level-2 verdicts (`docs/deep/authority.md`): which of the
        //    machines FlyByWire *also* models this model says have failed.
        //    Each is read off the machine alone -- its own overload
        //    element, its own regulated terminal, its own health parameter
        //    -- never off a bus voltage, which another component's fault
        //    can drag down just as easily.
        for i in 0..4 {
            let driven = truth.engine_running[i] && self.net.sources[self.src_gen[i]].open_circuit_v > 0.0;
            let terminal = self.net.sources[self.src_gen[i]].open_circuit_v;
            self.gen_verdict[i] = if self.wiring.vfg[i].overload_heat() >= 1.0 {
                Some(REASON_OVERLOAD_TRIPPED)
            } else if driven && (terminal < AC_UNDERVOLTAGE_TRIP_V || terminal > AC_OVERVOLTAGE_TRIP_V) {
                Some(REASON_REGULATOR_OUT_OF_BAND)
            } else if sf.vfg[i].winding_degradation >= DEGRADED_BEYOND_HALF {
                Some(REASON_WINDING_DEGRADED)
            } else {
                None
            };
        }
        for i in 0..2 {
            let driven = truth.apu_running && self.net.sources[self.src_apu_gen[i]].open_circuit_v > 0.0;
            let terminal = self.net.sources[self.src_apu_gen[i]].open_circuit_v;
            self.apu_gen_verdict[i] = if self.wiring.apu_gen[i].overload_heat() >= 1.0 {
                Some(REASON_OVERLOAD_TRIPPED)
            } else if driven && (terminal < AC_UNDERVOLTAGE_TRIP_V || terminal > AC_OVERVOLTAGE_TRIP_V) {
                Some(REASON_REGULATOR_OUT_OF_BAND)
            } else if sf.apu_gen[i].winding_degradation >= DEGRADED_BEYOND_HALF {
                Some(REASON_WINDING_DEGRADED)
            } else {
                None
            };
        }
        for i in 0..4 {
            self.tr_verdict[i] = (sf.tru[i].winding_degradation >= DEGRADED_BEYOND_HALF).then_some(REASON_TRU_DEGRADED);
        }
        self.static_inv_verdict = (sf.static_inverter.efficiency_loss >= DEGRADED_BEYOND_HALF).then_some(REASON_INVERTER_DEGRADED);
    }

    fn publish(&self, out: &mut dyn FnMut(&str, f64)) {
        for (i, bus) in self.net.buses.iter().enumerate() {
            out(&self.names.bus_potential[i], bus.voltage);
            out(&self.names.bus_powered[i], if self.report.bus_powered[i] { 1.0 } else { 0.0 });
            out(&self.names.bus_frequency[i], bus.frequency_hz);
        }
        for i in 0..4 {
            out(&self.names.gen_fault[i], if self.gen_fault[i] { 1.0 } else { 0.0 });
            out(&self.names.gen_load_w[i], self.measured_gen_load_w[i]);
            out(&self.names.tr_fault[i], if self.tr_fault[i] { 1.0 } else { 0.0 });
        }
        for i in 0..2 {
            out(&self.names.apu_gen_fault[i], if self.apu_gen_fault[i] { 1.0 } else { 0.0 });
            out(&self.names.apu_gen_load_w[i], self.measured_apu_gen_load_w[i]);
            out(&self.names.bat_fault[i], if self.bat_fault[i] { 1.0 } else { 0.0 });
            out(&self.names.bat_charge[i], self.bat_charge[i]);
        }
        // The registry's own ECAM trigger names one APU GEN fault, not two.
        out("ELEC_APU_GEN_FAULT", if self.apu_gen_fault[0] || self.apu_gen_fault[1] { 1.0 } else { 0.0 });
        out("ELEC_GALLEY_SHED_ACTIVE", if self.galley_shed { 1.0 } else { 0.0 });
        out("ELEC_COMMERCIAL_SHED_ACTIVE", if self.commercial_shed { 1.0 } else { 0.0 });
        out("ELEC_RAT_DEPLOYED", if self.rat_deployed { 1.0 } else { 0.0 });
        out("ELEC_EMER_CONFIG_ACTIVE", if self.emergency_config { 1.0 } else { 0.0 });
        out("ELEC_TOTAL_DEMAND_W", self.total_demand_w);
        out("ELEC_AVAILABLE_CAPACITY_W", self.capacity_w);
        out("ELEC_TOTAL_DELIVERED_W", self.report.total_power_w);

        for (i, b) in self.net.breakers.iter().enumerate() {
            out(&self.names.breaker_current[i], b.current_a);
            out(&self.names.breaker_closed[i], if b.closed { 1.0 } else { 0.0 });
        }
        for (i, l) in self.net.loads.iter().enumerate() {
            out(&self.names.load_powered[i], if l.powered { 1.0 } else { 0.0 });
        }

        // The level-2 couplings, so a derived failure is always visible
        // beside the failure it derived from (`docs/deep/authority.md`).
        let mut k = 0usize;
        self.each_coupling(&mut |d| {
            if let Some(name) = self.names.derived.get(k) {
                out(name, d.magnitude);
            }
            k += 1;
        });

        // The typed side of the same data, for the two areas that read it
        // back next frame (see this module's doc comment).
        board::with_board_mut(|board| {
            let n = self.net.breakers.len();
            if board.breaker_current_a.len() != n {
                board.breaker_current_a = vec![0.0; n];
            }
            for (i, b) in self.net.breakers.iter().enumerate() {
                board.breaker_current_a[i] = b.current_a;
            }
            for (i, bus) in self.net.buses.iter().enumerate() {
                board.bus_voltage[i] = bus.voltage;
            }
        });
    }

    fn derived_failures(&self, out: &mut dyn FnMut(DerivedFailure)) {
        self.each_coupling(out);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::deep::live::Deep;
    use std::collections::BTreeMap;

    fn flying_truth() -> Truth {
        Truth {
            dt_s: 1.0 / 30.0,
            on_ground: false,
            altitude_ft: 35_000.0,
            engine_n1_frac: [0.9; 4],
            // The VFGs are now driven off core (N2) speed, not fan speed
            // (see `ElectricalLive::tick`'s own doc) -- a flying engine's
            // core sits well above idle too.
            engine_n2_frac: [0.9; 4],
            engine_running: [true; 4],
            ..Truth::default()
        }
    }

    fn run(live: &mut ElectricalLive, truth: &Truth, faults: &Faults, frames: usize) -> BTreeMap<String, f64> {
        let mut published = BTreeMap::new();
        for _ in 0..frames {
            live.tick(truth, faults);
            published.clear();
            live.publish(&mut |n, v| {
                published.insert(n.to_string(), v);
            });
        }
        published
    }

    #[test]
    fn every_registered_failure_resolves_to_a_real_model_field() {
        let live = ElectricalLive::new();
        let mut reg = Registry::default();
        super::super::registry::register(&mut reg);
        let (routed, unresolved) = route_failures(&live.net);
        assert!(unresolved.is_empty(), "{} registered failures do not resolve: {:?}", unresolved.len(), &unresolved[..unresolved.len().min(10)]);
        assert_eq!(routed.len(), reg.failures.len(), "every registered failure must be routed exactly once");
        let mut ids: Vec<u64> = routed.iter().map(|(id, _)| *id).collect();
        let before = ids.len();
        ids.sort_unstable();
        ids.dedup();
        assert_eq!(ids.len(), before, "a failure id was routed twice");
        assert!(routed.len() > 1_000, "expected the real catalogue, got {}", routed.len());
    }

    /// `sources::Wiring::build`'s own generator/TR/battery/static-inverter
    /// protection breakers now register with `Network::add_feeder_breaker`
    /// instead of a plain `add_breaker`, so each carries its bus's real
    /// demand instead of a permanent 0 A -- the second gap this pass closed
    /// alongside the 125 catalogue loads.
    #[test]
    fn source_protection_breakers_now_carry_their_buss_real_current() {
        board::clear();
        let mut live = ElectricalLive::new();
        let published = run(&mut live, &flying_truth(), &Faults::default(), 20);
        for id in ["gen-1-bkr", "gen-2-bkr", "tr-1-bkr", "tr-ess-bkr"] {
            let v = published[&format!("ELEC_BKR_{id}_CURRENT_A")];
            assert!(v > 0.0, "{id} should now carry its bus's real demand current, got {v} A");
        }
        board::clear();
    }

    #[test]
    fn four_running_engines_energise_every_main_ac_bus_and_the_dc_buses_behind_them() {
        board::clear();
        let mut live = ElectricalLive::new();
        let published = run(&mut live, &flying_truth(), &Faults::default(), 20);
        for tag in ["AC_1", "AC_2", "AC_3", "AC_4", "AC_ESS", "DC_1", "DC_2", "DC_ESS"] {
            let v = published[&format!("ELEC_{tag}_BUS_POTENTIAL")];
            assert!(published[&format!("ELEC_{tag}_BUS_IS_POWERED")] == 1.0, "{tag} should be powered, it sits at {v} V");
        }
        assert!(published["ELEC_AC_1_BUS_POTENTIAL"] > 108.0);
        assert!(published["ELEC_AC_1_BUS_FREQUENCY"] > 360.0, "a VFG's bus runs at its own variable frequency");
        assert_eq!(published["ELEC_GEN_1_FAULT"], 0.0);
    }

    #[test]
    fn a_cold_dark_aircraft_has_no_ac_at_all_but_its_hot_buses_stay_alive() {
        board::clear();
        let mut live = ElectricalLive::new();
        let published = run(&mut live, &Truth::default(), &Faults::default(), 10);
        assert_eq!(published["ELEC_AC_1_BUS_IS_POWERED"], 0.0);
        assert_eq!(published["ELEC_AC_2_BUS_IS_POWERED"], 0.0);
        assert!(published["ELEC_DC_HOT_1_BUS_POTENTIAL"] > 20.0, "a battery-direct hot bus is live on a cold aircraft");
    }

    /// The failure this exercises is registered by `registry.rs` on the
    /// `gen-1` VFG with the effect "terminal voltage sags harder under
    /// load; a fully degraded machine may never reach rated voltage".
    #[test]
    fn arming_gen_1_winding_degradation_sags_its_own_bus_the_way_its_effect_says() {
        board::clear();
        let truth = flying_truth();

        let mut healthy = ElectricalLive::new();
        let before = run(&mut healthy, &truth, &Faults::default(), 30);

        let id = {
            let mut reg = Registry::default();
            super::super::registry::register(&mut reg);
            reg.failures
                .iter()
                .find(|f| f.component == "24_elec.vfg-1" && f.model_field.ends_with("winding_degradation"))
                .expect("GEN 1 winding degradation must be registered")
                .id
        };

        board::clear();
        let mut degraded = ElectricalLive::new();
        let after = run(&mut degraded, &truth, &Faults::from_pairs([(id, 1.0)]), 30);

        assert!(
            after["ELEC_AC_1_BUS_POTENTIAL"] < before["ELEC_AC_1_BUS_POTENTIAL"] - 0.05,
            "a fully degraded stator must sag its own bus: {} V healthy vs {} V degraded",
            before["ELEC_AC_1_BUS_POTENTIAL"],
            after["ELEC_AC_1_BUS_POTENTIAL"]
        );
    }

    /// `registry.rs`'s own effect for a bus `short_to_ground`: "collapses
    /// the bus's own voltage and can overload whatever feeder/tie breaker
    /// protects it".
    #[test]
    fn arming_an_ac_1_bus_short_collapses_that_bus_and_kills_the_loads_on_it() {
        board::clear();
        let truth = flying_truth();
        let id = {
            let mut reg = Registry::default();
            super::super::registry::register(&mut reg);
            reg.failures.iter().find(|f| f.component == "24_elec.bus.AC1").expect("the AC1 bus short must be registered").id
        };
        let mut live = ElectricalLive::new();
        let healthy = run(&mut live, &truth, &Faults::default(), 20);
        assert_eq!(healthy["ELEC_LOAD_cab-fan-1_POWERED"], 1.0, "CAB FAN 1 sits on AC1 and should be running");

        board::clear();
        let mut shorted = ElectricalLive::new();
        // A 0.03 ohm busbar-to-structure short against a VFG's own ~0.002
        // ohm source impedance is ~3.8 kA -- more than twice the feeder's
        // rating, so its I^2t element takes several seconds to clear it.
        let slow = Truth { dt_s: 0.05, ..truth };
        let after = run(&mut shorted, &slow, &Faults::from_pairs([(id, 1.0)]), 400);
        assert!(
            after["ELEC_AC_1_BUS_POTENTIAL"] < healthy["ELEC_AC_1_BUS_POTENTIAL"] * 0.5,
            "a bus short must end with the bus collapsed, not dented: {} V",
            after["ELEC_AC_1_BUS_POTENTIAL"]
        );
        assert_eq!(after["ELEC_BKR_feeder-AC1_CLOSED"], 0.0, "the feeder breaker protecting AC1 is what clears it");
        assert_eq!(after["ELEC_LOAD_cab-fan-1_POWERED"], 0.0, "a collapsed bus cannot run its own loads");
    }

    /// A breaker pulled (here through the board, exactly as
    /// `deep::breakers`' trip unit does it) must take its load dead.
    #[test]
    fn a_pulled_breaker_takes_its_own_load_dead_and_releasing_it_brings_it_back() {
        board::clear();
        let truth = flying_truth();
        let mut live = ElectricalLive::new();
        run(&mut live, &truth, &Faults::default(), 10);
        let idx = topology().breaker_index["cab-fan-1"];

        board::with_board_mut(|b| {
            b.breaker_open_cmd = vec![0.0; topology().breaker_count];
            b.breaker_open_cmd[idx] = 1.0;
        });
        let opened = run(&mut live, &truth, &Faults::default(), 5);
        assert_eq!(opened["ELEC_BKR_cab-fan-1_CLOSED"], 0.0);
        assert_eq!(opened["ELEC_LOAD_cab-fan-1_POWERED"], 0.0, "a load behind an open breaker draws nothing");
        assert_eq!(opened["ELEC_BKR_cab-fan-1_CURRENT_A"], 0.0);
        assert_eq!(opened["ELEC_LOAD_cab-fan-2_POWERED"], 1.0, "its neighbour on another bus is untouched");

        board::with_board_mut(|b| b.breaker_open_cmd[idx] = 0.0);
        let closed = run(&mut live, &truth, &Faults::default(), 5);
        assert_eq!(closed["ELEC_BKR_cab-fan-1_CLOSED"], 1.0);
        assert_eq!(closed["ELEC_LOAD_cab-fan-1_POWERED"], 1.0);
        board::clear();
    }

    /// Representative sample of the 125 loads this pass added to close
    /// `deep::breakers::catalog`'s "128 breakers protect no modelled load"
    /// gap (one continuous avionics LRU, one continuous small excitation
    /// circuit, and one valve actuator, spread across three different
    /// buses) -- each must actually be drawing (so its own breaker carries
    /// real current, the whole point of closing the gap) and must actually
    /// go dead when that breaker is pulled, exactly like every pre-existing
    /// catalogue entry.
    #[test]
    fn a_representative_sample_of_the_new_gap_closing_loads_goes_dead_when_its_own_breaker_is_pulled() {
        board::clear();
        let truth = flying_truth();
        for id in ["satcom", "dfdr", "fadec-1a", "crew-o2-shutoff", "fuel-valve-0-pos-ind"] {
            let mut live = ElectricalLive::new();
            let healthy = run(&mut live, &truth, &Faults::default(), 20);
            let power_var = format!("ELEC_LOAD_{id}_POWERED");
            let current_var = format!("ELEC_BKR_{id}_CURRENT_A");
            assert_eq!(healthy[&power_var], 1.0, "{id} should be powered and drawing on a healthy aircraft");
            assert!(healthy[&current_var] > 0.0, "{id}'s own breaker should carry real current now it protects a modelled load, got {}", healthy[&current_var]);

            let idx = topology().breaker_index[id];
            board::with_board_mut(|b| {
                b.breaker_open_cmd = vec![0.0; topology().breaker_count];
                b.breaker_open_cmd[idx] = 1.0;
            });
            let opened = run(&mut live, &truth, &Faults::default(), 5);
            assert_eq!(opened[&power_var], 0.0, "{id} must go dead once its own breaker is pulled");
            assert_eq!(opened[&current_var], 0.0);
            board::clear();
        }
    }

    /// The transit-only/one-shot loads this pass added (ignition exciters,
    /// fire-bottle squibs, the APU start contactor, cargo door actuator
    /// control) must be dead on an otherwise healthy, steady-state aircraft
    /// with nobody starting an engine, discharging a bottle, pressing START
    /// or moving a cargo door -- `loads.rs` defaults every entry
    /// `commanded_on: true`, so this is what proves `command_transit_loads`
    /// genuinely reads their real commands rather than leaving them
    /// (silently) on.
    #[test]
    fn transit_only_loads_stay_dead_on_a_steady_state_aircraft_with_nothing_commanding_them() {
        board::clear();
        let mut live = ElectricalLive::new();
        let published = run(&mut live, &flying_truth(), &Faults::default(), 20);
        for id in ["ignition-1a", "ignition-4b", "eng-fire-bottle-1-squib-1", "apu-fire-bottle-squib-2", "apu-start-contactor", "cargo-door-fwd-actuator-ctl", "cargo-door-aft-actuator-ctl"] {
            assert_eq!(published[&format!("ELEC_LOAD_{id}_POWERED")], 0.0, "{id} is not presently commanded and must stay dead, not silently on");
        }
        board::clear();
    }

    /// The rule's own required test: an ignition exciter draws current only
    /// while its engine is actually being started, and goes dead again the
    /// instant the starter disengages -- not before the start, not for the
    /// rest of the flight once the engine is running.
    #[test]
    fn an_ignition_exciter_draws_only_while_its_engine_is_being_started() {
        board::clear();
        let mut live = ElectricalLive::new();

        // A ground start: no engine generator is up yet, so the DC network
        // this exciter sits on (`ata7x_engine`'s own `Dc1`/`Dc2`) needs the
        // APU's generator, tied across the AC buses, to be live at all.
        let mut cranking = flying_truth();
        cranking.engine_running = [false; 4];
        cranking.engine_n1_frac = [0.0; 4];
        cranking.apu_running = true;
        cranking.controls.starter_engaged[0] = true;
        let during_start = run(&mut live, &cranking, &Faults::default(), 5);
        assert_eq!(during_start["ELEC_LOAD_ignition-1a_POWERED"], 1.0, "exciter A must draw while engine 1 is being cranked");
        assert_eq!(during_start["ELEC_LOAD_ignition-1b_POWERED"], 1.0, "exciter B must draw while engine 1 is being cranked");
        assert_eq!(during_start["ELEC_LOAD_ignition-2a_POWERED"], 0.0, "engine 2's own exciter must be untouched by engine 1's start");

        let mut running = flying_truth(); // starter_engaged defaults to false once started.
        running.apu_running = true;
        let after_start = run(&mut live, &running, &Faults::default(), 5);
        assert_eq!(after_start["ELEC_LOAD_ignition-1a_POWERED"], 0.0, "a running engine's exciter must go dead once the starter disengages");
        board::clear();
    }

    /// The rule's own required test: plugging in external power actually
    /// energises the ground-service buses, which is exactly what
    /// `command_contactors`'s own `gpu_line`/`ac_gnd_svc_feed` gating
    /// promises and what `command_contactors`'s `tick` used to be unable to
    /// do at all with `gpu_plugged_in` hardcoded `false`.
    #[test]
    fn ground_power_actually_energises_the_ground_service_buses() {
        board::clear();
        let cold_dark = Truth { on_ground: true, ..Truth::default() };
        let mut unplugged = ElectricalLive::new();
        let dark = run(&mut unplugged, &cold_dark, &Faults::default(), 20);
        assert_eq!(dark["ELEC_AC_GND_FLT_SVC_BUS_IS_POWERED"], 0.0, "a cold, dark, unplugged aircraft must have no ground-service power");

        board::clear();
        let plugged_in = Truth { on_ground: true, gpu_plugged_in: true, ..Truth::default() };
        let mut plugged = ElectricalLive::new();
        let lit = run(&mut plugged, &plugged_in, &Faults::default(), 20);
        assert_eq!(lit["ELEC_AC_GND_FLT_SVC_BUS_IS_POWERED"], 1.0, "a plugged-in GPU must actually energise the AC ground-service bus");
        assert!(lit["ELEC_AC_GND_FLT_SVC_BUS_POTENTIAL"] > 100.0, "{}", lit["ELEC_AC_GND_FLT_SVC_BUS_POTENTIAL"]);
        board::clear();
    }

    /// The RAT deploy solenoid is the one transit-only load this layer
    /// *does* have a real signal for: dead in normal flight, and drawing
    /// for exactly the tick an all-generation-lost emergency configuration
    /// commands the RAT out (`sources::Rat::deploy` is instantaneous in
    /// this model, not a multi-second extension, so that real signal is
    /// itself only ever true for one tick -- see this layer's own gating
    /// of the load next to where `deploy()` is called).
    #[test]
    fn the_rat_deploy_solenoid_only_draws_while_an_emergency_is_commanding_the_rat_out() {
        board::clear();
        let mut live = ElectricalLive::new();
        let normal = run(&mut live, &flying_truth(), &Faults::default(), 20);
        assert_eq!(normal["ELEC_LOAD_rat-deploy-solenoid_POWERED"], 0.0, "no emergency: the solenoid must not be drawing");

        let dead_truth = Truth {
            dt_s: 1.0 / 30.0,
            on_ground: false,
            engine_n1_frac: [0.0; 4],
            engine_running: [false; 4],
            environment: crate::deep::integration::weather_truth::EnvironmentTruth { tas_ms: 150.0, ..Truth::default().environment },
            ..Truth::default()
        };

        // The bus voltages take a few frames to actually collapse from
        // their initial nominal value, so find the tick emergency
        // configuration is first detected rather than assuming it is the
        // very first one.
        let mut saw_solenoid_on = false;
        let mut published = BTreeMap::new();
        for _ in 0..60 {
            live.tick(&dead_truth, &Faults::default());
            published.clear();
            live.publish(&mut |n, v| {
                published.insert(n.to_string(), v);
            });
            if published["ELEC_LOAD_rat-deploy-solenoid_POWERED"] == 1.0 {
                saw_solenoid_on = true;
                assert_eq!(published["ELEC_EMER_CONFIG_ACTIVE"], 1.0, "the solenoid must only draw while an emergency is actually commanding the RAT out");
                break;
            }
        }
        assert!(saw_solenoid_on, "the RAT deploy solenoid never drew any current across a full emergency-configuration transition");

        let after = run(&mut live, &dead_truth, &Faults::default(), 10);
        assert_eq!(after["ELEC_LOAD_rat-deploy-solenoid_POWERED"], 0.0, "once the RAT is deployed the solenoid has nothing left to do and must go dead again");
        board::clear();
    }

    #[test]
    fn wiring_damage_published_onto_the_board_opens_the_load_it_names() {
        board::clear();
        let truth = flying_truth();
        let mut live = ElectricalLive::new();
        run(&mut live, &truth, &Faults::default(), 10);
        let idx = topology().load_index["cab-fan-1"];
        board::with_board_mut(|b| {
            b.load_open = vec![0.0; topology().load_count];
            b.load_open[idx] = 1.0;
        });
        let after = run(&mut live, &truth, &Faults::default(), 5);
        assert_eq!(after["ELEC_LOAD_cab-fan-1_POWERED"], 0.0, "a burnt-through conductor carries nothing");
        board::clear();
    }

    #[test]
    fn losing_every_generator_in_flight_deploys_the_rat_and_raises_emergency_config() {
        board::clear();
        let mut live = ElectricalLive::new();
        let truth = Truth { dt_s: 1.0 / 30.0, on_ground: false, engine_n1_frac: [0.0; 4], engine_running: [false; 4], environment: crate::deep::integration::weather_truth::EnvironmentTruth { tas_ms: 150.0, ..Truth::default().environment }, ..Truth::default() };
        let published = run(&mut live, &truth, &Faults::default(), 30);
        assert_eq!(published["ELEC_EMER_CONFIG_ACTIVE"], 1.0);
        assert_eq!(published["ELEC_RAT_DEPLOYED"], 1.0);
        assert_eq!(published["ELEC_COMMERCIAL_SHED_ACTIVE"], 1.0, "emergency configuration sheds the commercial load");
        assert_eq!(published["ELEC_AC_ESS_SHED_BUS_IS_POWERED"], 0.0, "the shed bus is what gets shed");
    }

    #[test]
    fn it_plugs_into_deep_and_publishes_every_variable_its_ecam_triggers_read() {
        board::clear();
        let mut deep = Deep::new().with_area(live_system());
        let mut published = BTreeMap::new();
        deep.tick(flying_truth(), &Faults::default(), &mut |n, v| {
            published.insert(n.to_string(), v);
        });
        for name in [
            "ELEC_GEN_1_FAULT",
            "ELEC_APU_GEN_FAULT",
            "ELEC_TR_1_FAULT",
            "ELEC_BAT_1_FAULT",
            "ELEC_AC_1_BUS_POTENTIAL",
            "ELEC_AC_1_BUS_IS_POWERED",
            "ELEC_AC_2_BUS_IS_POWERED",
            "ELEC_AC_3_BUS_IS_POWERED",
            "ELEC_AC_4_BUS_IS_POWERED",
            "ELEC_GALLEY_SHED_ACTIVE",
            "ELEC_COMMERCIAL_SHED_ACTIVE",
        ] {
            assert!(published.contains_key(name), "{name} is read by an ECAM trigger but nobody publishes it");
        }
        assert_eq!(deep.area_names(), vec!["electrical"]);
        board::clear();
    }

    #[test]
    #[ignore = "diagnostic"]
    fn diagnose() {
        board::clear();
        let mut live = ElectricalLive::new();
        let p = run(&mut live, &flying_truth(), &Faults::default(), 30);
        println!("AC1 {} Hz {} demand {} cap {}", p["ELEC_AC_1_BUS_POTENTIAL"], p["ELEC_AC_1_BUS_FREQUENCY"], p["ELEC_TOTAL_DEMAND_W"], p["ELEC_AVAILABLE_CAPACITY_W"]);
        println!("galley shed {} commercial {}", p["ELEC_GALLEY_SHED_ACTIVE"], p["ELEC_COMMERCIAL_SHED_ACTIVE"]);
        let idx = topology().breaker_index["cab-fan-1"];
        let b = &live.net.breakers[idx];
        println!("cab-fan-1 bkr closed {} rated {} I {} heat {} cause {:?}", b.closed, b.rated_a, b.current_a, b.heat_fraction(), b.trip_cause);
        let li = topology().load_index["cab-fan-1"];
        println!("cab-fan-1 load powered {} I {} feed {:?}", live.net.loads[li].powered, live.net.loads[li].current_a, live.net.loads[li].active_feed);
        let mut open: Vec<&str> = live.net.breakers.iter().filter(|b| !b.closed).map(|b| b.id).collect();
        open.sort_unstable();
        println!("open breakers ({}): {:?}", open.len(), &open[..open.len().min(30)]);
        let dead: Vec<(&str, &str)> = live.net.loads.iter().filter(|l| !l.powered).map(|l| (l.spec.id, l.spec.bus.label())).collect();
        println!("unpowered loads {} of {}: {:?}", dead.len(), live.net.loads.len(), dead);
        for b in super::ALL_BUS_IDS {
            let p: f64 = live.net.loads.iter().filter(|l| l.spec.bus == b).map(|l| l.spec.rated_power_w).sum();
            let i: f64 = live.net.loads.iter().filter(|l| l.spec.bus == b).map(|l| l.current_a).sum();
            println!("{:>16} = {:8.2} V  rated {:9.0} W  I {:8.1} A", b.label(), live.net.bus(b).voltage, p, i);
        }
        for c in &live.net.contactors {
            println!("contactor {:>18} cmd {} closed {} -> {}", c.id, c.commanded_closed, c.closed, c.to.label());
        }
        let mut top: Vec<(&str, f64, f64)> = live.net.loads.iter().filter(|l| l.spec.bus == BusId::DcEss).map(|l| (l.spec.id, l.current_a, l.spec.rated_power_w)).collect();
        top.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap());
        println!("DC_ESS top loads {:?}", &top[..top.len().min(8)]);
    }

    /// Frame-cost measurement for this pass's own report: electrical is the
    /// largest network in the crate (`route_failures`'s own test already
    /// asserts over 1,000 routed failures), and this pass added a 23-load
    /// `command_transit_loads` pass and a `truth.published` lookup every
    /// tick, both new per-frame costs on top of the existing solve.
    /// `#[ignore]`d like this file's other diagnostics -- run explicitly
    /// with `cargo test -- --ignored` to see the number, never as part of
    /// the normal suite (wall-clock timing is not a correctness assertion).
    #[test]
    #[ignore = "diagnostic"]
    fn frame_cost() {
        board::clear();
        let mut live = ElectricalLive::new();
        let truth = flying_truth();
        let faults = Faults::default();
        // Warm up (first tick pays for lazily-built indices/allocations
        // that a real running aircraft only pays once).
        for _ in 0..30 {
            live.tick(&truth, &faults);
            let mut out = |_: &str, _: f64| {};
            live.publish(&mut out);
        }
        let n = 2000;
        let start = std::time::Instant::now();
        for _ in 0..n {
            live.tick(&truth, &faults);
            let mut out = |_: &str, _: f64| {};
            live.publish(&mut out);
        }
        let elapsed = start.elapsed();
        let per_tick_us = elapsed.as_secs_f64() * 1_000_000.0 / n as f64;
        println!("electrical: {per_tick_us:.1} us/tick over {n} ticks ({:.3} ms total)", elapsed.as_secs_f64() * 1000.0);
        println!("budget at 60 Hz: 16667 us/frame for the whole plugin; at 30 Hz: 33333 us/frame");
        board::clear();
    }

    #[test]
    fn nothing_produces_a_nan_at_rest_or_at_zero_dt() {
        board::clear();
        let mut live = ElectricalLive::new();
        let truth = Truth { dt_s: 0.0, ..Truth::default() };
        let published = run(&mut live, &truth, &Faults::default(), 3);
        for (name, v) in &published {
            assert!(v.is_finite(), "{name} is {v}");
        }
    }

    // -----------------------------------------------------------------
    // Authority (docs/deep/authority.md): the level-2 couplings.

    fn derived(live: &ElectricalLive) -> BTreeMap<u64, f64> {
        let mut out = BTreeMap::new();
        live.derived_failures(&mut |d| {
            out.insert(d.fbw_id, d.magnitude);
        });
        out
    }

    #[test]
    fn the_coupling_table_matches_what_the_area_actually_emits() {
        // `publish` pairs each coupling with a pre-built name by position,
        // so the table and the emission order are one thing and must not
        // drift. Every id must also be one FlyByWire really registers,
        // since `Failures::apply` silently ignores anything else.
        board::clear();
        let live = ElectricalLive::new();
        let table = coupling_table();
        let mut emitted: Vec<(u64, &'static str)> = Vec::new();
        live.each_coupling(&mut |d| emitted.push((d.fbw_id, d.deep_component)));
        assert_eq!(emitted, table, "coupling_table() and each_coupling() must walk the same list in the same order");
        assert_eq!(live.names.derived.len(), table.len());

        let catalogue: std::collections::BTreeSet<u64> = crate::failures::a380_failures().into_iter().map(|(id, _)| id).collect();
        for (id, component) in &table {
            assert!(catalogue.contains(id), "{component} derives {id}, which FlyByWire does not register");
        }
        let mut ids: Vec<u64> = table.iter().map(|(id, _)| *id).collect();
        let before = ids.len();
        ids.sort_unstable();
        ids.dedup();
        assert_eq!(ids.len(), before, "two deep components must not claim the same FlyByWire failure");
        // 4 generators + 2 APU generators + 4 TRs + the static inverter +
        // the 14 buses FlyByWire also models.
        assert_eq!(table.len(), 25);
    }

    #[test]
    fn a_healthy_aircraft_tells_flybywire_nothing_at_all() {
        // The derived set is a level, and on a working aircraft it is
        // empty: no coupling may fire on a healthy cold start or in the
        // cruise, or the crew would be handed failures nobody caused.
        board::clear();
        let mut live = ElectricalLive::new();
        run(&mut live, &flying_truth(), &Faults::default(), 60);
        assert!(derived(&live).values().all(|&m| m == 0.0), "cruise: {:?}", derived(&live));

        board::clear();
        let mut cold = ElectricalLive::new();
        run(&mut cold, &Truth::default(), &Faults::default(), 60);
        assert!(
            derived(&cold).values().all(|&m| m == 0.0),
            "a cold dark aircraft has dead buses, but a dead bus is not a failed one: {:?}",
            derived(&cold)
        );
    }

    /// The bus short from `arming_an_ac_1_bus_short_...` above, followed
    /// through to FlyByWire: once the feeder breaker has cleared it, AC1 is
    /// isolated, and that is a verdict `a380_systems` can act on.
    #[test]
    fn a_cleared_bus_short_tells_flybywire_that_bus_is_failed() {
        board::clear();
        let truth = flying_truth();
        let id = {
            let mut reg = Registry::default();
            super::super::registry::register(&mut reg);
            reg.failures.iter().find(|f| f.component == "24_elec.bus.AC1").expect("the AC1 bus short must be registered").id
        };
        let mut live = ElectricalLive::new();
        let slow = Truth { dt_s: 0.05, ..truth };
        let published = run(&mut live, &slow, &Faults::from_pairs([(id, 1.0)]), 400);

        assert_eq!(published["ELEC_BKR_feeder-AC1_CLOSED"], 0.0, "setup: the feeder must have tripped");
        // 24_100 is `FailureType::ElectricalBus(AlternatingCurrent(1))`.
        assert_eq!(derived(&live).get(&24_100), Some(&1.0), "AC1 behind an open feeder must reach FlyByWire as a failed bus");
        assert_eq!(derived(&live).get(&24_101), Some(&0.0), "and no other bus may be blamed for it");
        assert_eq!(published["DEEP_DERIVED_FBW_FAILURE_24100"], 1.0, "and it must be visible, not silent");
    }

    /// A bus fault is the bus's, not the machine's. FlyByWire keeps a
    /// failed generator failed until it is told otherwise, so blaming
    /// generator 2 for a short on its bus would leave a perfectly healthy
    /// machine dead on FlyByWire's side long after the feeder cleared it.
    #[test]
    fn a_bus_short_is_never_blamed_on_the_generator_feeding_that_bus() {
        board::clear();
        let truth = flying_truth();
        let id = {
            let mut reg = Registry::default();
            super::super::registry::register(&mut reg);
            reg.failures.iter().find(|f| f.component == "24_elec.bus.AC2").expect("the AC2 bus short must be registered").id
        };
        let mut live = ElectricalLive::new();
        let slow = Truth { dt_s: 0.05, ..truth };
        run(&mut live, &slow, &Faults::from_pairs([(id, 1.0)]), 400);
        let d = derived(&live);
        assert_eq!(d.get(&24_101), Some(&1.0), "AC2 is isolated behind its own feeder, and that is the verdict");
        for gen in FBW_GENERATOR {
            assert_eq!(d.get(&gen), Some(&0.0), "no generator may be blamed for a busbar fault ({gen})");
        }
    }

    /// A machine degraded past half its own range: FlyByWire's generator
    /// failure is binary, so `DEGRADED_BEYOND_HALF` is where the deep
    /// model's continuous stator degradation rounds to a trip.
    #[test]
    fn a_stator_degraded_past_half_reaches_flybywire_as_a_failed_generator() {
        board::clear();
        let truth = flying_truth();
        let id = {
            let mut reg = Registry::default();
            super::super::registry::register(&mut reg);
            reg.failures
                .iter()
                .find(|f| f.component == "24_elec.vfg-1" && f.model_field.ends_with("winding_degradation"))
                .expect("GEN 1 winding degradation must be registered")
                .id
        };

        let mut mild = ElectricalLive::new();
        run(&mut mild, &truth, &Faults::from_pairs([(id, 0.3)]), 30);
        assert_eq!(derived(&mild).get(&24_020), Some(&0.0), "a third-degraded stator still holds its bus; FlyByWire is told nothing");

        board::clear();
        let mut gone = ElectricalLive::new();
        run(&mut gone, &truth, &Faults::from_pairs([(id, 1.0)]), 30);
        assert_eq!(derived(&gone).get(&24_020), Some(&1.0), "a fully degraded stator is a failed generator");
        assert_eq!(derived(&gone).get(&24_021), Some(&0.0), "and only that one");
    }

    #[test]
    fn a_degraded_transformer_rectifier_reaches_flybywires_own_tr() {
        board::clear();
        let truth = flying_truth();
        // `route_failures` maps `24_elec.tr-ess` to index 2, which is
        // `FailureType::TransformerRectifier(3)` -- FlyByWire's TR ESS.
        let id = {
            let mut reg = Registry::default();
            super::super::registry::register(&mut reg);
            reg.failures.iter().find(|f| f.component == "24_elec.tr-ess").expect("the TR ESS failure must be registered").id
        };
        let mut live = ElectricalLive::new();
        run(&mut live, &truth, &Faults::from_pairs([(id, 1.0)]), 30);
        let d = derived(&live);
        assert_eq!(d.get(&24_002), Some(&1.0), "TR ESS is FlyByWire's TransformerRectifier(3)");
        assert_eq!(d.get(&24_000), Some(&0.0));
        assert_eq!(d.get(&24_001), Some(&0.0));
        assert_eq!(d.get(&24_003), Some(&0.0));
    }

    /// End to end, with FlyByWire's real A380 systems on the other side:
    /// the deep model's verdict, through the same `FailureType` the crew's
    /// own armed failure would take, changes the coarse solve.
    ///
    /// The global `crate::failures` registry is deliberately not touched
    /// here (it is process-wide and shared with every other test); the id
    /// is mapped to its `FailureType` through `a380_failures()`, which is
    /// exactly what `Failures::apply` does with it.
    #[test]
    fn a_derived_bus_failure_changes_flybywires_own_solve() {
        use crate::aspects::test_vars::TestVars;
        use std::time::Duration;
        use systems::simulation::{Simulation, StartState};

        // The DC hot bus, because a bare test bed can genuinely power it:
        // its battery needs nothing but its own pushbutton, where every AC
        // bus would need a running engine's generator, and this test is
        // about the coupling rather than about FlyByWire's start state.
        board::clear();
        let id = {
            let mut reg = Registry::default();
            super::super::registry::register(&mut reg);
            reg.failures.iter().find(|f| f.component == "24_elec.bus.DC_HOT1").expect("the DC HOT 1 bus short must be registered").id
        };
        let mut live = ElectricalLive::new();
        // A busbar-to-structure short on a 28 V bus behind a 92 A feeder
        // is several hundred amps; its I^2t element takes seconds.
        let slow = Truth { dt_s: 0.05, ..Truth::default() };
        run(&mut live, &slow, &Faults::from_pairs([(id, 1.0)]), 600);
        let verdict = derived(&live);
        assert_eq!(verdict.get(&24_113), Some(&1.0), "setup: the deep model must have concluded DC HOT 1 is isolated");

        let mut vars = TestVars::default();
        let mut sim = Simulation::new(StartState::Apron, a380_systems::A380::new, &mut vars);
        for bat in ["BAT_1", "BAT_2", "BAT_ESS", "BAT_APU"] {
            vars.set(&format!("A32NX_OVHD_ELEC_{bat}_PB_IS_AUTO"), 1.0);
        }
        let tick = |sim: &mut Simulation<a380_systems::A380>, vars: &mut TestVars, n: usize| {
            for i in 0..n {
                sim.tick(Duration::from_millis(50), 100.0 + i as f64 * 0.05, vars);
            }
        };
        tick(&mut sim, &mut vars, 20);
        assert_eq!(vars.value("A32NX_ELEC_DC_HOT_1_BUS_IS_POWERED"), 1.0, "setup: FlyByWire's own hot bus is alive on its battery");

        // Exactly what the plugin patch in `docs/deep/authority.md` does
        // with `Deep::derived_magnitudes()`.
        let types: Vec<systems::failures::FailureType> =
            crate::failures::a380_failures().into_iter().filter(|(fid, _)| verdict.get(fid).copied().unwrap_or(0.0) > 0.0).map(|(_, t)| t).collect();
        assert!(!types.is_empty());
        sim.update_active_failures(types.into_iter().collect());
        tick(&mut sim, &mut vars, 20);
        assert_eq!(
            vars.value("A32NX_ELEC_DC_HOT_1_BUS_IS_POWERED"),
            0.0,
            "the deep model's verdict must change FlyByWire's own solve, not just a display"
        );
    }

    /// A second aircraft built in the same process must start clean.
    ///
    /// The board is a `thread_local!`, so it used to survive an aircraft:
    /// two `all_areas()` built one after another shared the first one's
    /// last frame of breaker currents, bus voltages and harness damage,
    /// and two runs of the *identical* healthy state came out 135
    /// published variables apart. Nothing here calls `board::clear()` --
    /// that is the point: `ElectricalLive::new` does it, so a caller
    /// cannot forget.
    #[test]
    fn a_second_aircraft_in_the_same_process_does_not_inherit_the_first_ones_board() {
        let truth = flying_truth();
        let faults = Faults::default();

        let mut first = ElectricalLive::new();
        let a = run(&mut first, &truth, &faults, 30);
        assert!(board::with_board(|b| b.breaker_current_a.iter().any(|&i| i > 0.0)), "the first aircraft must leave real current on the board, or this test proves nothing");

        let mut second = ElectricalLive::new();
        let b = run(&mut second, &truth, &faults, 30);

        let differing: Vec<&String> = a.iter().filter(|(k, v)| b.get(*k).map_or(true, |w| w != *v)).map(|(k, _)| k).collect();
        assert!(differing.is_empty(), "{} published variables differ between two identical healthy aircraft, e.g. {:?}", differing.len(), differing.iter().take(6).collect::<Vec<_>>());
    }

    /// The real delivered power of each engine and APU generator, which
    /// `deep::apu` reads back for its own generator wear and overload
    /// protection. The only thing this area published for those sources
    /// before was a breaker current, which is the whole bus's rather than
    /// the individual machine's.
    #[test]
    fn each_generator_publishes_its_own_delivered_power() {
        let truth = flying_truth();
        let published = run(&mut ElectricalLive::new(), &truth, &Faults::default(), 30);
        for n in 1..=4 {
            let w = published[&format!("ELEC_ENG_GEN_{n}_LOAD_W")];
            assert!(w > 0.0 && w.is_finite(), "engine generator {n} is on line and carrying the aircraft, but publishes {w} W");
        }
        for n in 1..=2 {
            let name = format!("ELEC_APU_GEN_{n}_LOAD_W");
            let w = published[&name];
            assert!(w.is_finite() && w >= 0.0, "{name} published {w}");
        }

        // ... and with the APU actually running and its generators on
        // line, the APU figure is real rather than zero.
        let apu = Truth { apu_running: true, on_ground: true, engine_running: [false; 4], engine_n2_frac: [0.0; 4], ..flying_truth() };
        let published = run(&mut ElectricalLive::new(), &apu, &Faults::default(), 30);
        let total: f64 = (1..=2).map(|n| published[&format!("ELEC_APU_GEN_{n}_LOAD_W")]).sum();
        assert!(total > 0.0, "both APU generators on line carrying the whole aircraft publish {total} W between them");
    }
}
