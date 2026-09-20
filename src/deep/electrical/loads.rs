//! The A380 load catalogue: every consumer named in
//! `D:\fbw-xp-systems\src\breakers.rs`'s ~130 ATA-grouped entries becomes its
//! own [`network::Load`] here (same id, name, ATA chapter, bus and rating
//! basis, cited back to the exact `breakers.rs` function/line it came from),
//! plus the major consumers that catalogue does not enumerate at all: fuel
//! pumps/valves, lighting feeders (lumped per circuit type), galleys, IFE
//! seat boxes per cabin zone, window/probe heat, and the avionics computers/
//! radios `breakers.rs` itself does not reach.
//!
//! `breakers.rs`'s own generator/TRU/APU-generator/static-inverter/bus-feeder
//! entries (its `ata24`) are deliberately **not** duplicated as loads here:
//! those are sources and their own protection interface, built in
//! `sources.rs` (a TR/GEN/bus-feeder "breaker" there gates a
//! [`network::Contactor`], not a [`network::Load`]).
//!
//! Every rating not already a real, cited FBW figure is marked `GENERIC` in
//! its own `basis` string, same convention `breakers.rs` uses throughout.

use super::network::{Breaker, BusId, LoadFeed, LoadSpec, Network};

/// A load's category, for `shedding.rs`'s galley/commercial-shed relays and
/// emergency-configuration logic (kept out of `network.rs`, which has no
/// aircraft-specific taxonomy of its own).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LoadCategory {
    /// Flight-safety-critical: never shed by anything except its own bus
    /// dying outright (flight computers, LGCIU, EGPWC, radio altimeters,
    /// fire detection, fuel shutoff valves).
    Essential,
    /// Galley power (ovens, chillers, water heaters): the first thing shed
    /// on a generator loss / single-engine-inoperative power budget.
    Galley,
    /// Cabin commercial load (IFE seat boxes, cabin lighting beyond the
    /// minimum): shed after galleys if the budget still does not balance.
    Commercial,
    /// Everything else: avionics, motors, valves, sensors, lighting feeders
    /// not in the commercial group -- normally kept, but still subject to
    /// its own bus dying.
    Other,
}

/// One catalogue entry's resolved network index, for `shedding.rs` (and any
/// future consumer) to look up by category without re-deriving it from
/// `LoadSpec::id` string prefixes.
pub struct Catalog {
    pub galley: Vec<usize>,
    pub commercial: Vec<usize>,
    pub essential: Vec<usize>,
    pub other: Vec<usize>,
}

impl Catalog {
    fn new() -> Self {
        Self { galley: Vec::new(), commercial: Vec::new(), essential: Vec::new(), other: Vec::new() }
    }
    fn push(&mut self, category: LoadCategory, index: usize) {
        match category {
            LoadCategory::Essential => self.essential.push(index),
            LoadCategory::Galley => self.galley.push(index),
            LoadCategory::Commercial => self.commercial.push(index),
            LoadCategory::Other => self.other.push(index),
        }
    }
}

// ---------------------------------------------------------------------
// Generic per-class defaults, so each catalogue entry only states what is
// actually distinctive about it.

/// Under-voltage dropout point: MIL-STD-704F's steady-state AC/DC limits sit
/// around 90-107% nominal; 85% is a slightly looser, GENERIC "the LRU's own
/// internal regulation gives up" margin below that (no A380-specific
/// per-LRU spec is public).
fn min_operating_voltage(bus: BusId) -> f64 {
    bus.nominal_voltage() * 0.85
}

/// Feeder wiring resistance a load's own `short_to_ground` fault current is
/// limited by (GENERIC, same order of magnitude as
/// `Battery::WIRING_RESISTANCE_OHM` = 0.02 ohm,
/// `fbw-common/.../electrical/battery.rs:78`): a longer/thinner-gauge 115 V
/// AC feeder run is given a somewhat higher figure than a short, heavier-
/// gauge 28 V DC one.
fn wiring_resistance_ohm(bus: BusId) -> f64 {
    if bus.is_ac() {
        0.08
    } else {
        0.03
    }
}

fn avionics_spec(id: &'static str, name: &'static str, ata: u16, bus: BusId, watts: f64, basis: &'static str) -> LoadSpec {
    LoadSpec {
        id,
        name,
        ata,
        bus,
        rated_power_w: watts,
        power_factor: if bus.is_ac() { 0.95 } else { 1.0 },
        min_operating_voltage: min_operating_voltage(bus),
        inrush_multiple: 1.3,
        inrush_duration_s: 0.05,
        wiring_resistance_ohm: wiring_resistance_ohm(bus),
        // Every avionics LRU has its own internal switching power supply,
        // insensitive to line frequency within its own input tolerance --
        // not a simple line-frequency-synchronous motor.
        rated_frequency_hz: 0.0,
        basis,
    }
}

fn motor_spec(id: &'static str, name: &'static str, ata: u16, bus: BusId, watts: f64, power_factor: f64, inrush_multiple: f64, inrush_duration_s: f64, basis: &'static str) -> LoadSpec {
    LoadSpec { id, name, ata, bus, rated_power_w: watts, power_factor, min_operating_voltage: min_operating_voltage(bus), inrush_multiple, inrush_duration_s, wiring_resistance_ohm: wiring_resistance_ohm(bus), rated_frequency_hz: 0.0, basis }
}

/// A simple line-frequency induction motor with no speed control of its own
/// (`network::LoadSpec::rated_frequency_hz`'s own doc: the real, cited
/// exception to `motor_spec`'s assumption that a motor load regulates its
/// own speed) -- on the A380 this is genuinely only the direct-drive
/// recirculation/extraction/cooling fans, not any inverter/controller-fed
/// pump or valve actuator.
/// The line frequency a direct-drive motor fed from the A380's own
/// variable-frequency AC system is rated at, Hz.
///
/// Derived, not chosen: `sources::Vfg` produces 360-800 Hz across its
/// engine's own speed range, and for a centrifugal fan the affinity law
/// makes shaft power scale with the cube of that frequency. A motor on such
/// a bus therefore has to carry its nameplate at the **top** of the band --
/// the point at which it draws the most -- or it would be in a permanent
/// 6-7x overload at cruise and burn out (or trip its own breaker) on every
/// flight. The nameplate point is thus `Vfg::FREQ_MAX_HZ`, 800 Hz, and the
/// same machine draws proportionally less at every lower engine speed.
/// (400 Hz, the figure this entry carried before, is the *constant*-
/// frequency aircraft standard, which is exactly what a variable-frequency
/// aircraft does not have.)
const VF_MOTOR_RATED_FREQUENCY_HZ: f64 = 800.0;

fn frequency_sensitive_motor_spec(id: &'static str, name: &'static str, ata: u16, bus: BusId, watts: f64, power_factor: f64, inrush_multiple: f64, inrush_duration_s: f64, rated_frequency_hz: f64, basis: &'static str) -> LoadSpec {
    let mut spec = motor_spec(id, name, ata, bus, watts, power_factor, inrush_multiple, inrush_duration_s, basis);
    spec.rated_frequency_hz = rated_frequency_hz;
    spec
}

fn resistive_spec(id: &'static str, name: &'static str, ata: u16, bus: BusId, watts: f64, basis: &'static str) -> LoadSpec {
    LoadSpec {
        id,
        name,
        ata,
        bus,
        rated_power_w: watts,
        power_factor: 1.0,
        min_operating_voltage: min_operating_voltage(bus),
        // A cold resistive heating element's resistance is lower than hot
        // (positive temperature coefficient), a real, if modest, inrush.
        inrush_multiple: 1.5,
        inrush_duration_s: 2.0,
        rated_frequency_hz: 0.0,
        wiring_resistance_ohm: wiring_resistance_ohm(bus),
        basis,
    }
}

fn own_breaker(net: &mut Network, id: &'static str, rated_a: f64, bus: BusId) -> usize {
    net.add_breaker(Breaker::new(id, rated_a, bus))
}

fn add(net: &mut Network, cat: &mut Catalog, category: LoadCategory, spec: LoadSpec, rated_a: f64) {
    let bus = spec.bus;
    let bkr = own_breaker(net, spec.id, rated_a, bus);
    let idx = net.add_load(spec, bkr);
    cat.push(category, idx);
}

/// A real dual-fed A380 LRU: `spec.bus` is its normal feed, `second_bus` its
/// second (typically ESS/backup) feed, each on its own breaker, OR-ed
/// together inside the box's own power supply (`network::LoadFeed`'s own
/// doc) -- losing either single feed alone does not lose the unit. This is
/// the real architecture for the aircraft's flight-control/nav computers,
/// ADIRUs, CPIOMs, the FWS and its display units: each takes power from two
/// or three separate buses through its own breaker/SSPC per feed.
fn add_dual(net: &mut Network, cat: &mut Catalog, category: LoadCategory, spec: LoadSpec, rated_a: f64, second_bus: BusId) {
    let normal_bus = spec.bus;
    let id = spec.id;
    let normal_bkr_id: &'static str = Box::leak(format!("{id}-normal-bkr").into_boxed_str());
    let second_bkr_id: &'static str = Box::leak(format!("{id}-2nd-bkr").into_boxed_str());
    let normal_bkr = own_breaker(net, normal_bkr_id, rated_a, normal_bus);
    let second_bkr = own_breaker(net, second_bkr_id, rated_a, second_bus);
    let feeds = vec![LoadFeed { bus: normal_bus, breaker: normal_bkr, priority: 0 }, LoadFeed { bus: second_bus, breaker: second_bkr, priority: 1 }];
    let idx = net.add_load_multi_feed(spec, feeds);
    cat.push(category, idx);
}

/// A real triple-fed A380 LRU (the FWS/CPIOM class of critical computer,
/// with two normal-side feeds plus an ESS backup) -- same OR-ing principle
/// as [`add_dual`], one more feed/breaker. Not yet called (no catalogue
/// entry needs a third feed yet); kept ready for the FWS/CPIOM expansion
/// noted in PROGRESS.md as next up.
#[allow(dead_code)]
fn add_triple(net: &mut Network, cat: &mut Catalog, category: LoadCategory, spec: LoadSpec, rated_a: f64, second_bus: BusId, third_bus: BusId) {
    let normal_bus = spec.bus;
    let id = spec.id;
    let bkr_id = |suffix: &str| -> &'static str { Box::leak(format!("{id}-{suffix}-bkr").into_boxed_str()) };
    let normal_bkr = own_breaker(net, bkr_id("normal"), rated_a, normal_bus);
    let second_bkr = own_breaker(net, bkr_id("2nd"), rated_a, second_bus);
    let third_bkr = own_breaker(net, bkr_id("3rd"), rated_a, third_bus);
    let feeds = vec![
        LoadFeed { bus: normal_bus, breaker: normal_bkr, priority: 0 },
        LoadFeed { bus: second_bus, breaker: second_bkr, priority: 1 },
        LoadFeed { bus: third_bus, breaker: third_bkr, priority: 2 },
    ];
    let idx = net.add_load_multi_feed(spec, feeds);
    cat.push(category, idx);
}

/// A breaker rated with the usual small margin above a load's own rated
/// current (real aircraft breakers are sized above steady load, not exactly
/// at it, so a healthy load never nuisance-trips its own protection) --
/// GENERIC 25% margin, typical thermal-breaker sizing practice.
fn rated_current(spec: &LoadSpec) -> f64 {
    let base = spec.rated_power_w / (spec.bus.nominal_voltage() * spec.power_factor.max(0.1));
    base * 1.25
}

// ---------------------------------------------------------------------
// ATA21 -- air conditioning consumers (`breakers.rs::ata21`).

fn ata21(net: &mut Network, cat: &mut Catalog) {
    const FANS: [(&str, &str, BusId); 4] = [
        ("cab-fan-1", "CAB FAN 1", BusId::Ac1),
        ("cab-fan-2", "CAB FAN 2", BusId::Ac2),
        ("cab-fan-3", "CAB FAN 3", BusId::Ac3),
        ("cab-fan-4", "CAB FAN 4", BusId::Ac4),
    ];
    for (id, name, bus) in FANS {
        // A real, cited engineering complexity of a variable-frequency
        // system: these are simple direct-drive AC induction motors with no
        // speed control of their own, so their own shaft speed -- and by
        // the fan affinity laws, their own power draw -- genuinely tracks
        // the VFG's own variable (360-800 Hz) output frequency, rated here
        // at a nominal 400 Hz reference point (GENERIC; no A380-specific
        // fan-motor nameplate is public).
        let spec = frequency_sensitive_motor_spec(id, name, 21, bus, 500.0, 0.85, 3.0, 1.0, VF_MOTOR_RATED_FREQUENCY_HZ, "breakers.rs::ata21 CAB FAN 1-4 (500 W typical large-transport recirculation fan motor, typical/derived); real bus from a380_systems/air_conditioning/mod.rs CabinFan::new; frequency-sensitive direct-drive induction motor, fan affinity laws vs the VFG's own variable output frequency, nameplate at VF_MOTOR_RATED_FREQUENCY_HZ");
        let a = rated_current(&spec);
        add(net, cat, LoadCategory::Other, spec, a);
    }
    let spec = motor_spec("hotair-1", "HOT AIR VALVE 1", 21, BusId::AcEss, 50.0, 0.8, 2.0, 0.3, "breakers.rs::ata21 HOT AIR VALVE 1 (50 W typical motor-operated valve actuator, typical/derived)");
    add(net, cat, LoadCategory::Other, spec.clone(), rated_current(&spec));
    let spec = motor_spec("hotair-2", "HOT AIR VALVE 2", 21, BusId::AcEss, 50.0, 0.8, 2.0, 0.3, "breakers.rs::ata21 HOT AIR VALVE 2");
    add(net, cat, LoadCategory::Other, spec.clone(), rated_current(&spec));
    let spec = motor_spec("fwd-isol-valve", "FWD CARGO ISOL VALVE", 21, BusId::Dc2, 50.0, 0.8, 2.0, 0.3, "breakers.rs::ata21 FWD CARGO ISOL VALVE (VCM Fwd DC2 channel)");
    add(net, cat, LoadCategory::Other, spec.clone(), rated_current(&spec));
    let spec = motor_spec("fwd-extract-fan", "FWD CARGO EXTRACT FAN", 21, BusId::Dc2, 150.0, 0.85, 3.0, 1.0, "breakers.rs::ata21 FWD CARGO EXTRACT FAN");
    add(net, cat, LoadCategory::Other, spec.clone(), rated_current(&spec));
    let spec = motor_spec("bulk-isol-valve", "BULK CARGO ISOL VALVE", 21, BusId::DcEss, 50.0, 0.8, 2.0, 0.3, "breakers.rs::ata21 BULK CARGO ISOL VALVE (VCM Aft DC_ESS channel)");
    add(net, cat, LoadCategory::Other, spec.clone(), rated_current(&spec));
    let spec = motor_spec("bulk-extract-fan", "BULK CARGO EXTRACT FAN", 21, BusId::DcEss, 150.0, 0.85, 3.0, 1.0, "breakers.rs::ata21 BULK CARGO EXTRACT FAN");
    add(net, cat, LoadCategory::Other, spec.clone(), rated_current(&spec));
    let spec = resistive_spec("cargo-heater", "BULK CARGO HEATER", 21, BusId::Ac2, 1000.0, "breakers.rs::ata21 BULK CARGO HEATER (1000 W typical cargo-bay heater element, typical/derived; mod.rs AirHeater::new(AC2))");
    add(net, cat, LoadCategory::Other, spec.clone(), rated_current(&spec));

    const FDAC: [(&str, &str, BusId); 4] = [("fdac-1a", "FDAC 1 CHANNEL 1", BusId::AcEss), ("fdac-1b", "FDAC 1 CHANNEL 2", BusId::Ac2), ("fdac-2a", "FDAC 2 CHANNEL 1", BusId::AcEss), ("fdac-2b", "FDAC 2 CHANNEL 2", BusId::Ac4)];
    for (id, name, bus) in FDAC {
        let spec = avionics_spec(id, name, 21, bus, 50.0, "breakers.rs::ata21 FDAC (50 W generic avionics LRU, typical/derived; mod.rs FullDigitalAGUController::new)");
        add(net, cat, LoadCategory::Essential, spec.clone(), rated_current(&spec));
    }
    const TADD: [(&str, &str, BusId); 2] = [("tadd-1", "TADD CHANNEL 1", BusId::Ac2), ("tadd-2", "TADD CHANNEL 2", BusId::Ac4)];
    for (id, name, bus) in TADD {
        let spec = avionics_spec(id, name, 21, bus, 50.0, "breakers.rs::ata21 TADD (mod.rs TrimAirDriveDevice::new)");
        add(net, cat, LoadCategory::Essential, spec.clone(), rated_current(&spec));
    }
    const VCM: [(&str, &str, BusId); 4] = [("vcm-fwd-1", "VCM FWD CHANNEL 1", BusId::Dc2), ("vcm-fwd-2", "VCM FWD CHANNEL 2", BusId::DcEss), ("vcm-aft-1", "VCM AFT CHANNEL 1", BusId::Dc2), ("vcm-aft-2", "VCM AFT CHANNEL 2", BusId::DcEss)];
    for (id, name, bus) in VCM {
        let spec = avionics_spec(id, name, 21, bus, 50.0, "breakers.rs::ata21 VCM (mod.rs VentilationControlModule::new)");
        add(net, cat, LoadCategory::Other, spec.clone(), rated_current(&spec));
    }
    const OCSM_AP: [(&str, &str, BusId); 4] = [("ocsm-1-ap", "OCSM 1 AUTO PARTITION", BusId::Dc1), ("ocsm-2-ap", "OCSM 2 AUTO PARTITION", BusId::Dc1), ("ocsm-3-ap", "OCSM 3 AUTO PARTITION", BusId::Dc2), ("ocsm-4-ap", "OCSM 4 AUTO PARTITION", BusId::Dc2)];
    for (id, name, bus) in OCSM_AP {
        let spec = avionics_spec(id, name, 21, bus, 50.0, "breakers.rs::ata21 OCSM auto-partition (mod.rs OutflowValveControlModule::new)");
        add(net, cat, LoadCategory::Essential, spec.clone(), rated_current(&spec));
    }
    const OCSM_CH: [(&str, &str, BusId); 8] = [
        ("ocsm-1a", "OCSM 1 CHANNEL 1", BusId::Dc1),
        ("ocsm-1b", "OCSM 1 CHANNEL 2", BusId::DcEss),
        ("ocsm-2a", "OCSM 2 CHANNEL 1", BusId::Dc1),
        ("ocsm-2b", "OCSM 2 CHANNEL 2", BusId::DcEss),
        ("ocsm-3a", "OCSM 3 CHANNEL 1", BusId::Dc2),
        ("ocsm-3b", "OCSM 3 CHANNEL 2", BusId::DcEss),
        ("ocsm-4a", "OCSM 4 CHANNEL 1", BusId::Dc2),
        ("ocsm-4b", "OCSM 4 CHANNEL 2", BusId::DcEss),
    ];
    for (id, name, bus) in OCSM_CH {
        let spec = avionics_spec(id, name, 21, bus, 50.0, "breakers.rs::ata21 OCSM channel");
        add(net, cat, LoadCategory::Essential, spec.clone(), rated_current(&spec));
    }
    let cpiom_bus = [BusId::Dc1, BusId::DcEss, BusId::DcEss, BusId::Dc2];
    for (app_idx, app) in ["AGS", "TCS", "VCS", "CPCS"].iter().enumerate() {
        for k in 0..4usize {
            let id: &'static str = Box::leak(format!("cpiom-b{}-{}", k + 1, app.to_lowercase()).into_boxed_str());
            let name: &'static str = Box::leak(format!("CPIOM B{} {} APP", k + 1, app).into_boxed_str());
            let basis: &'static str = Box::leak(format!("breakers.rs::ata21 CPIOM B{} {} APP (mod.rs CPIOM B bus map, ~line 1334; app group {})", k + 1, app, app_idx).into_boxed_str());
            let spec = avionics_spec(id, name, 21, cpiom_bus[k], 50.0, basis);
            add(net, cat, LoadCategory::Other, spec.clone(), rated_current(&spec));
        }
    }
    for pack in 1..=2u32 {
        for side in 1..=2u32 {
            let id: &'static str = Box::leak(format!("pack-{pack}-flow-valve-{side}").into_boxed_str());
            let name: &'static str = Box::leak(format!("PACK {pack} FLOW VALVE {side}").into_boxed_str());
            let basis: &'static str = Box::leak(format!("breakers.rs::ata21 PACK {pack} FLOW VALVE {side} (pneumatic.rs PackComplex ElectroPneumaticValve, DC_ESS)").into_boxed_str());
            let spec = motor_spec(id, name, 21, BusId::DcEss, 50.0, 0.8, 2.0, 0.3, basis);
            add(net, cat, LoadCategory::Other, spec.clone(), rated_current(&spec));
        }
    }
    // Avionics-bay cooling fans (not individually named in breakers.rs, but
    // a real, necessary ATA21 consumer for any avionics-bay LRU set this
    // size): two redundant fans, GENERIC typical avionics-bay blower rating.
    for (id, name, bus) in [("avionics-fan-1", "AVIONICS BAY FAN 1", BusId::AcEss), ("avionics-fan-2", "AVIONICS BAY FAN 2", BusId::AcEssShed), ("avionics-fan-3", "AVIONICS BAY FAN 3", BusId::Ac1), ("avionics-fan-4", "AVIONICS BAY FAN 4", BusId::Ac2)] {
        let spec = frequency_sensitive_motor_spec(id, name, 21, bus, 300.0, 0.85, 3.0, 1.0, VF_MOTOR_RATED_FREQUENCY_HZ, "GENERIC: typical avionics-bay cooling blower motor (300 W), not individually named in breakers.rs but required to ventilate the LRU set it protects; direct-drive induction motor, frequency-sensitive, nameplate at VF_MOTOR_RATED_FREQUENCY_HZ");
        add(net, cat, LoadCategory::Other, spec.clone(), rated_current(&spec));
    }
}

// ---------------------------------------------------------------------
// ATA26 -- fire detection loops (`breakers.rs::ata26`).

fn ata26(net: &mut Network, cat: &mut Catalog) {
    let zones = ["ENG1", "ENG2", "ENG3", "ENG4", "APU", "MLGBAY"];
    for zone in zones {
        for loop_name in ["A", "B"] {
            let id: &'static str = Box::leak(format!("fire-loop-{}-{loop_name}", zone.to_lowercase()).into_boxed_str());
            let name: &'static str = Box::leak(format!("FIRE DET {zone} LOOP {loop_name}").into_boxed_str());
            let basis: &'static str = Box::leak(format!("breakers.rs::ata26 FIRE DET {zone} LOOP {loop_name} (20 W typical detection-loop controller electronics, typical/derived; fire_and_smoke_protection.rs DC_ESS/DC_HOT1)").into_boxed_str());
            let spec = avionics_spec(id, name, 26, BusId::DcEss, 20.0, basis);
            add(net, cat, LoadCategory::Essential, spec.clone(), rated_current(&spec));
        }
    }
}

// ---------------------------------------------------------------------
// ATA22/27 -- flight control / autoflight computers (`breakers.rs::ata27`).

fn ata27(net: &mut Network, cat: &mut Catalog) {
    let entries: [(&str, &str, u16); 11] = [
        ("rollout", "ROLLOUT", 22),
        ("fcu-1", "FCU 1", 22),
        ("fcu-2", "FCU 2", 22),
        ("prim-1", "PRIM 1", 27),
        ("prim-2", "PRIM 2", 27),
        ("prim-3", "PRIM 3", 27),
        ("sec-1", "SEC 1", 27),
        ("sec-2", "SEC 2", 27),
        ("sec-3", "SEC 3", 27),
        ("fcdc-1", "FCDC 1", 27),
        ("fcdc-2", "FCDC 2", 27),
    ];
    for (k, (id, name, ata)) in entries.into_iter().enumerate() {
        // Real Airbus flight-control/autoflight computers are dual-fed: a
        // normal DC bus (alternating DC1/DC2 across the lane set, the
        // redundant-bus pattern breakers.rs's own doc already cites) plus a
        // DC ESS backup feed, OR-ed internally -- losing either single feed
        // does not lose the computer. Two breakers per computer.
        let normal_bus = if k % 2 == 0 { BusId::Dc1 } else { BusId::Dc2 };
        let spec = avionics_spec(id, name, ata, normal_bus, 100.0, "breakers.rs::ata27 flight-control/autoflight computer (100 W typical FCC-class LRU, typical/derived: FBW's C++ FCCs carry no Rust-side bus figure); real Airbus-style dual feed, normal DC bus + DC ESS backup, each on its own breaker, OR-ed internally");
        let a = rated_current(&spec);
        add_dual(net, cat, LoadCategory::Essential, spec, a, BusId::DcEss);
    }
}

// ---------------------------------------------------------------------
// ATA32 -- landing gear (`breakers.rs::ata32`, `ata32_gear_and_door_sensors`).

fn ata32(net: &mut Network, cat: &mut Catalog) {
    // Real Airbus LGCIUs are dual-lane, each lane itself fed from two
    // separate buses (its own normal DC feed plus the opposite lane's own
    // backup) so a single bus loss never blinds gear/door sensing entirely.
    let spec = avionics_spec("lgciu-1", "LGCIU 1", 32, BusId::DcEss, 50.0, "breakers.rs::ata32 LGCIU 1; real dual feed, DC ESS normal + DC2 backup, each on its own breaker");
    let a = rated_current(&spec);
    add_dual(net, cat, LoadCategory::Essential, spec, a, BusId::Dc2);
    let spec = avionics_spec("lgciu-2", "LGCIU 2", 32, BusId::Dc2, 50.0, "breakers.rs::ata32 LGCIU 2; real dual feed, DC2 normal + DC ESS backup, each on its own breaker");
    let a = rated_current(&spec);
    add_dual(net, cat, LoadCategory::Essential, spec, a, BusId::DcEss);

    // Electric hydraulic pumps: FBW's own ELECTRIC_PUMP_MAX_CURRENT_AMPERE
    // (75 A, hydraulic/mod.rs:1750, real/FBW-sourced) at nominal 28 V DC
    // gives the rated power; a locked/dragging rotor (bearing wear) is
    // exactly what `LoadFaults::high_resistance` represents here. Each pump
    // also has its own small, separate contactor-coil supply -- the low-
    // power DC circuit that energises the motor's own line contactor,
    // physically and electrically distinct from the pump motor's own high-
    // current feed (a real coil failing does not draw the motor's own 75 A,
    // but does stop the pump from ever engaging).
    const PUMPS: [(&str, &str, BusId); 4] = [("hyd-epump-ga", "HYD GREEN ELEC PUMP A", BusId::Ac3), ("hyd-epump-gb", "HYD GREEN ELEC PUMP B", BusId::Ac4), ("hyd-epump-ya", "HYD YELLOW ELEC PUMP A", BusId::AcEss), ("hyd-epump-yb", "HYD YELLOW ELEC PUMP B", BusId::Ac2)];
    for (id, name, bus) in PUMPS {
        // 75 A at 28 V DC-equivalent motor-controller input power (real
        // rated current from FBW; the pump motor itself runs off a variable-
        // frequency inverter fed from the named AC bus, breakers.rs's own
        // bus assignment) -- modelled at unity-ish pf on the AC bus feeding
        // its controller, real inrush of a starting hydraulic pump motor.
        let spec = motor_spec(id, name, 29, bus, 75.0 * 28.0, 0.85, 4.0, 0.5, "breakers.rs::ata32 electric hydraulic pump (FBW ELECTRIC_PUMP_MAX_CURRENT_AMPERE = 75 A, hydraulic/mod.rs:1750, real/FBW-sourced; 28 V-equivalent power rating, real/FBW current x reference DC voltage)");
        add(net, cat, LoadCategory::Other, spec.clone(), rated_current(&spec));

        let coil_id: &'static str = Box::leak(format!("{id}-coil").into_boxed_str());
        let coil_name: &'static str = Box::leak(format!("{name} CONTACTOR COIL").into_boxed_str());
        let coil_spec = avionics_spec(coil_id, coil_name, 29, BusId::DcEss, 20.0, "GENERIC: typical DC line-contactor holding-coil supply (~20 W), the low-power control circuit that energises the pump motor's own contactor, distinct from the motor's own high-current feed");
        add(net, cat, LoadCategory::Other, coil_spec.clone(), rated_current(&coil_spec));
    }
    let spec = motor_spec("autobrake-disarm-sol", "AUTOBRAKE DISARM SOLENOID", 32, BusId::Dc2, 56.0, 1.0, 3.0, 0.1, "breakers.rs::ata32 AUTOBRAKE DISARM SOLENOID (56 W typical small solenoid valve, typical/derived; autobrakes.rs DC2)");
    add(net, cat, LoadCategory::Other, spec.clone(), rated_current(&spec));

    const SENSORS: [&str; 12] = [
        "prox-uplock-gear-nose-1",
        "prox-downlock-gear-nose-2",
        "prox-uplock-gear-right-1",
        "prox-downlock-gear-right-2",
        "prox-uplock-gear-left-2",
        "prox-downlock-gear-left-1",
        "prox-uplock-door-nose-1",
        "prox-downlock-door-nose-2",
        "prox-uplock-door-right-2",
        "prox-downlock-door-right-1",
        "prox-uplock-door-left-2",
        "prox-downlock-door-left-1",
    ];
    for id in SENSORS {
        let name: &'static str = Box::leak(id.replace('-', " ").to_uppercase().into_boxed_str());
        let spec = avionics_spec(id, name, 32, BusId::DcEss, 5.0, "breakers.rs::ata32_gear_and_door_sensors proximity sensor (5 W typical target/pickup, typical/derived; LGCIU's own DC_ESS supply)");
        add(net, cat, LoadCategory::Essential, spec.clone(), rated_current(&spec));
    }
    const ACTUATORS: [&str; 6] = ["gear-actuator-nose", "gear-actuator-left", "gear-actuator-right", "gear-door-actuator-nose", "gear-door-actuator-left", "gear-door-actuator-right"];
    for id in ACTUATORS {
        let name: &'static str = Box::leak(id.replace('-', " ").to_uppercase().into_boxed_str());
        let spec = motor_spec(id, name, 32, BusId::DcEss, 75.0 * 28.0, 0.85, 3.0, 0.5, "breakers.rs::ata32_gear_and_door_sensors gear/door actuator control (same order of magnitude as the electric hydraulic pumps, typical/derived)");
        add(net, cat, LoadCategory::Other, spec.clone(), rated_current(&spec));
    }
}

// ---------------------------------------------------------------------
// ATA34 -- radio altimeters, antennas, EGPWC (`breakers.rs::ata34`).

fn ata34(net: &mut Network, cat: &mut Catalog) {
    const RAS: [(&str, &str, BusId); 3] = [("ra-sys-a", "RA SYS A", BusId::Ac1), ("ra-sys-b", "RA SYS B", BusId::Ac2), ("ra-sys-c", "RA SYS C", BusId::AcEss)];
    for (id, name, bus) in RAS {
        let spec = avionics_spec(id, name, 34, bus, 50.0, "breakers.rs::ata34 radio altimeter transceiver (navigation.rs A380RadioAltimeters)");
        add(net, cat, LoadCategory::Essential, spec.clone(), rated_current(&spec));
    }
    for (n, bus) in [(1, BusId::Ac1), (2, BusId::Ac2), (3, BusId::AcEss)] {
        let id: &'static str = Box::leak(format!("ra-ant-interrupt-{n}").into_boxed_str());
        let name: &'static str = Box::leak(format!("RA {n} ANTENNA INTERRUPT").into_boxed_str());
        let spec = avionics_spec(id, name, 34, bus, 10.0, "breakers.rs::ata34_ra_antennas antenna-coupling network (10 W class, typical/derived)");
        add(net, cat, LoadCategory::Essential, spec.clone(), rated_current(&spec));
        let id2: &'static str = Box::leak(format!("ra-ant-coupling-{n}").into_boxed_str());
        let name2: &'static str = Box::leak(format!("RA {n} ANTENNA DIRECT COUPLING").into_boxed_str());
        let spec2 = avionics_spec(id2, name2, 34, bus, 10.0, "breakers.rs::ata34_ra_antennas antenna-coupling network");
        add(net, cat, LoadCategory::Essential, spec2.clone(), rated_current(&spec2));
    }
    let spec = avionics_spec("egpwc", "EGPWC (TAWS)", 34, BusId::AcEss, 100.0, "breakers.rs::ata34_ra_antennas EGPWC (100 W typical flight-warning-class computer LRU, typical/derived; real AC_ESS bus, enhanced_gpwc/mod.rs)");
    add(net, cat, LoadCategory::Essential, spec.clone(), rated_current(&spec));
}

// ---------------------------------------------------------------------
// ATA28 -- fuel pumps/valves, absorbed from the embedded systems.cfg
// (`breakers.rs::absorbed_systems_cfg`, `circuits.rs`'s own 25 pumps/60
// valves). Every consumer here becomes its own load (not lumped), matching
// the brief's "every consumer named in breakers.rs becomes a load" -- the
// per-circuit wattage is the same generic figure
// `physics::electrical.rs::rated_watts` already uses for these two circuit
// types (real precedent already established in this codebase, not invented
// here): 600 W/pump, 50 W/valve.
fn ata28_fuel(net: &mut Network, cat: &mut Catalog) {
    // circuits.rs's own MSFS bus numbers are not imported (no crate-internal
    // dependency); fuel pumps/valves are distributed across the DC/AC buses
    // that feed fuel system LRUs on a real A380 (tank-by-tank, alternating
    // sides), a documented approximation in the absence of importing the
    // exact per-circuit bus assignment.
    let pump_buses = [BusId::Ac1, BusId::Ac2, BusId::Ac3, BusId::Ac4, BusId::AcEss];
    for i in 0..25usize {
        let id: &'static str = Box::leak(format!("fuel-pump-{i}").into_boxed_str());
        let name: &'static str = Box::leak(format!("FUEL PUMP {i}").into_boxed_str());
        let bus = pump_buses[i % pump_buses.len()];
        let spec = motor_spec(id, name, 28, bus, 600.0, 0.85, 3.0, 1.0, "circuits.rs CIRCUIT_FUEL_PUMP (25 real pumps); wattage = physics::electrical.rs::rated_watts(\"CIRCUIT_FUEL_PUMP\") = 600 W, the same generic figure already established as precedent in this codebase for this exact circuit type");
        add(net, cat, LoadCategory::Essential, spec.clone(), rated_current(&spec));
    }
    let valve_buses = [BusId::Dc1, BusId::Dc2, BusId::DcEss, BusId::DcBat];
    for i in 0..60usize {
        let id: &'static str = Box::leak(format!("fuel-valve-{i}").into_boxed_str());
        let name: &'static str = Box::leak(format!("FUEL VALVE {i}").into_boxed_str());
        let bus = valve_buses[i % valve_buses.len()];
        let spec = motor_spec(id, name, 28, bus, 50.0, 0.8, 2.0, 0.3, "circuits.rs CIRCUIT_FUEL_VALVE (60 real valves); wattage = physics::electrical.rs::rated_watts(\"CIRCUIT_FUEL_VALVE\") = 50 W, same precedent figure");
        add(net, cat, LoadCategory::Essential, spec.clone(), rated_current(&spec));
    }
}

// ---------------------------------------------------------------------
// ATA33 -- lighting feeders, lumped per circuit type (the brief's own
// instruction: "lighting feeders as lumped loads only"). Wattages are the
// same generic per-type figures `physics::electrical.rs::rated_watts`
// already established for these exact `CIRCUIT_LIGHT_*` types.
fn ata33_lighting(net: &mut Network, cat: &mut Catalog) {
    const LIGHTS: [(&str, &str, BusId, f64); 12] = [
        ("light-landing", "LANDING LIGHTS", BusId::Ac1, 600.0),
        ("light-taxi", "TAXI LIGHTS", BusId::Ac2, 250.0),
        ("light-nav", "NAV LIGHTS", BusId::AcEssShed, 40.0),
        ("light-beacon", "BEACON LIGHTS", BusId::AcEssShed, 100.0),
        ("light-strobe", "STROBE LIGHTS", BusId::Ac3, 300.0),
        ("light-logo", "LOGO LIGHTS", BusId::Ac4, 150.0),
        ("light-wing", "WING LIGHTS", BusId::Ac1, 150.0),
        ("light-recognition", "RECOGNITION LIGHTS", BusId::DcBat, 40.0),
        ("light-cabin", "CABIN LIGHTS", BusId::AcGndFltSvc, 200.0),
        ("light-panel", "PANEL LIGHTS", BusId::DcEss, 30.0),
        ("light-pedestal", "PEDESTAL LIGHTS", BusId::DcEss, 20.0),
        ("light-glareshield", "GLARESHIELD LIGHTS", BusId::DcEss, 20.0),
    ];
    for (id, name, bus, watts) in LIGHTS {
        let spec = resistive_spec(id, name, 33, bus, watts, "circuits.rs CIRCUIT_LIGHT_* lumped per type (brief instruction); wattage = physics::electrical.rs::rated_watts, same precedent figure for this exact circuit type");
        add(net, cat, LoadCategory::Other, spec.clone(), rated_current(&spec));
    }
}

// ---------------------------------------------------------------------
// ATA30 -- window/probe heat (not modelled anywhere else in this codebase).
fn ata30_ice_protection(net: &mut Network, cat: &mut Catalog) {
    for (n, bus) in [(1, BusId::Ac1), (2, BusId::Ac2)] {
        let id: &'static str = Box::leak(format!("windshield-heat-{n}").into_boxed_str());
        let name: &'static str = Box::leak(format!("WINDSHIELD HEAT {n}").into_boxed_str());
        let spec = resistive_spec(id, name, 30, bus, 2000.0, "GENERIC: typical wide-body windshield electric anti-ice heating element (2000 W/side), no A380-specific public figure");
        add(net, cat, LoadCategory::Other, spec.clone(), rated_current(&spec));
    }
    // Pitot heat (3 pitots) and AOA/TAT probe heat (3 probes), the same
    // generic per-probe figure `physics::electrical.rs::rated_watts`
    // already established for `CIRCUIT_PITOT_HEAT`.
    for (n, bus) in [(1, BusId::Ac1), (2, BusId::Ac2), (3, BusId::AcEss)] {
        let id: &'static str = Box::leak(format!("pitot-heat-{n}").into_boxed_str());
        let name: &'static str = Box::leak(format!("PITOT HEAT {n}").into_boxed_str());
        let spec = resistive_spec(id, name, 30, bus, 600.0, "GENERIC precedent: physics::electrical.rs::rated_watts(\"CIRCUIT_PITOT_HEAT\") = 600 W, same figure reused here for a probe not otherwise modelled");
        add(net, cat, LoadCategory::Essential, spec.clone(), rated_current(&spec));
    }
    for (name_txt, id, bus) in [("AOA HEAT 1", "aoa-heat-1", BusId::Dc1), ("AOA HEAT 2", "aoa-heat-2", BusId::Dc2), ("TAT PROBE HEAT", "tat-heat", BusId::DcEss)] {
        let name: &'static str = Box::leak(name_txt.to_string().into_boxed_str());
        let spec = resistive_spec(id, name, 30, bus, 150.0, "GENERIC: typical small-probe (AOA vane / TAT) heating element, an order of magnitude below a pitot tube's own heater");
        add(net, cat, LoadCategory::Essential, spec.clone(), rated_current(&spec));
    }
}

// ---------------------------------------------------------------------
// ATA25/38 -- galleys, lumped per zone (GENERIC: no public A380 per-galley
// wattage; large-transport galley complexes of this class typically run a
// few kW of ovens/water heaters/chillers per zone).
fn ata25_galleys(net: &mut Network, cat: &mut Catalog) {
    const GALLEYS: [(&str, &str, BusId, f64); 6] = [
        ("galley-fwd-upper", "FWD UPPER GALLEY", BusId::Ac1, 8000.0),
        ("galley-aft-upper", "AFT UPPER GALLEY", BusId::Ac2, 8000.0),
        ("galley-fwd-main", "FWD MAIN DECK GALLEY", BusId::Ac3, 10000.0),
        ("galley-mid-main", "MID MAIN DECK GALLEY", BusId::Ac4, 10000.0),
        ("galley-aft-main", "AFT MAIN DECK GALLEY", BusId::Ac1, 10000.0),
        ("galley-lower", "LOWER DECK GALLEY LIFT", BusId::Ac2, 3000.0),
    ];
    for (id, name, bus, watts) in GALLEYS {
        let spec = resistive_spec(id, name, 25, bus, watts, "GENERIC: typical wide-body galley complex load (ovens, water heaters, chillers combined), no public per-zone A380 figure; the first load shed on a generator loss (`shedding.rs`)");
        add(net, cat, LoadCategory::Galley, spec.clone(), rated_current(&spec));
    }
}

// ---------------------------------------------------------------------
// ATA44 -- IFE seat boxes, lumped per cabin zone (GENERIC: no public
// A380/FlyByWire per-seat wattage; ~30 W per seat box is a typical figure
// for a wide-body IFE smart-monitor seat unit, x an approximate zone seat
// count).
fn ata44_ife(net: &mut Network, cat: &mut Catalog) {
    const ZONES: [(&str, &str, BusId, u32); 5] = [
        ("ife-upper-deck", "IFE UPPER DECK ZONE", BusId::AcGndFltSvc, 90),
        ("ife-main-fwd", "IFE MAIN DECK FWD ZONE", BusId::AcGndFltSvc, 120),
        ("ife-main-mid", "IFE MAIN DECK MID ZONE", BusId::AcGndFltSvc, 150),
        ("ife-main-aft", "IFE MAIN DECK AFT ZONE", BusId::AcGndFltSvc, 130),
        ("ife-server", "IFE SERVER RACK", BusId::Ac2, 1),
    ];
    for (id, name, bus, seats) in ZONES {
        let per_seat_w = if seats > 1 { 30.0 } else { 4000.0 }; // the server rack itself, GENERIC
        let watts = per_seat_w * seats as f64;
        let spec = avionics_spec(id, name, 44, bus, watts, "GENERIC: typical wide-body IFE seat-box power (~30 W/seat) x an approximate zone seat count, or a server-rack figure for the head-end; no public A380/FlyByWire figure");
        add(net, cat, LoadCategory::Commercial, spec.clone(), rated_current(&spec));
    }
}

// ---------------------------------------------------------------------
// Avionics computers/radios not reached by any `breakers.rs` group above
// (ATA22/23/34): FMS, transponders, ADIRS, VHF/HF radios, weather radar,
// TCAS -- real LRU classes on a real A380, generic per-box wattage
// (`AVIONICS_LRU_W`-class figure, same order `breakers.rs` itself uses).
fn avionics_misc(net: &mut Network, cat: &mut Catalog) {
    // Flight-critical boxes (FMS, ADIRU, TCAS) are real dual-fed LRUs on the
    // A380 (a normal bus plus an ESS/backup bus, OR-ed internally); radios/
    // transponders/weather radar are conventionally single-fed with a
    // manual transfer switch, not an automatic OR, so stay single-feed here.
    const DUAL_BOXES: [(&str, &str, u16, BusId, BusId, f64); 7] = [
        ("fms-1", "FMS 1", 34, BusId::Dc1, BusId::DcEss, 60.0),
        ("fms-2", "FMS 2", 34, BusId::Dc2, BusId::DcEss, 60.0),
        ("fms-3", "FMS 3", 34, BusId::Dc1, BusId::DcEss, 60.0),
        ("adirs-1", "ADIRU 1", 34, BusId::AcEss, BusId::DcEss, 80.0),
        ("adirs-2", "ADIRU 2", 34, BusId::Ac2, BusId::DcEss, 80.0),
        ("adirs-3", "ADIRU 3", 34, BusId::AcEssShed, BusId::DcEss, 80.0),
        ("tcas", "TCAS COMPUTER", 34, BusId::DcEss, BusId::Dc2, 70.0),
    ];
    for (id, name, ata, normal_bus, second_bus, watts) in DUAL_BOXES {
        let spec = avionics_spec(id, name, ata, normal_bus, watts, "GENERIC: typical avionics LRU class figure (same order of magnitude breakers.rs's own AVIONICS_LRU_W uses), no A380-specific public per-box wattage; real dual feed (normal + ESS/backup bus), each on its own breaker, OR-ed internally");
        let a = rated_current(&spec);
        add_dual(net, cat, LoadCategory::Essential, spec, a, second_bus);
    }
    const SINGLE_BOXES: [(&str, &str, u16, BusId, f64); 5] = [
        ("xpdr-1", "TRANSPONDER 1", 34, BusId::Dc1, 50.0),
        ("xpdr-2", "TRANSPONDER 2", 34, BusId::Dc2, 50.0),
        ("vhf-1", "VHF 1", 23, BusId::Dc1, 40.0),
        ("vhf-2", "VHF 2", 23, BusId::Dc2, 40.0),
        ("wxr", "WEATHER RADAR", 34, BusId::Ac1, 150.0),
    ];
    for (id, name, ata, bus, watts) in SINGLE_BOXES {
        let spec = avionics_spec(id, name, ata, bus, watts, "GENERIC: typical avionics LRU class figure (same order of magnitude breakers.rs's own AVIONICS_LRU_W uses), no A380-specific public per-box wattage; conventionally single-fed with a manual transfer switch, not an automatic OR");
        add(net, cat, LoadCategory::Essential, spec.clone(), rated_current(&spec));
    }
}

// ---------------------------------------------------------------------
// ATA36 -- bleed-air valve sets, one shared breaker per engine
// (`breakers.rs::ata36`: HP/PR/fan-air valves share one real feed).
fn ata36_bleed(net: &mut Network, cat: &mut Catalog) {
    for n in 1..=4u32 {
        let bus = if n <= 2 { BusId::Dc1 } else { BusId::Dc2 };
        let id: &'static str = Box::leak(format!("bleed-eng-{n}").into_boxed_str());
        let name: &'static str = Box::leak(format!("BLEED ENG {n} VALVES").into_boxed_str());
        let basis: &'static str = Box::leak(format!("breakers.rs::ata36 BLEED ENG {n} (HP + pressure-regulating + fan-air valve, one shared feed, real Airbus-style single bleed CB; 3x50 W typical valve actuators, typical/derived)").into_boxed_str());
        let spec = motor_spec(id, name, 36, bus, 150.0, 0.8, 2.0, 0.3, basis);
        add(net, cat, LoadCategory::Other, spec.clone(), rated_current(&spec));
    }
}

// ---------------------------------------------------------------------
// Closing `deep::breakers::catalog`'s own "128 breakers protect no
// modelled load" gap
// (`breakers_protecting_no_modelled_load_are_a_known_named_gap`). Every
// function below gives a real `Load` to a consumer that catalogue's own
// group-2 ("other real A380 equipment, no load model yet") or group-3
// ("control/excitation supplies paired with an existing actuator breaker")
// entries already named and rated -- using the **exact same id, bus,
// wattage and power factor** that catalogue entry already derived and
// cited, not a second, independently guessed figure. Reusing an
// already-justified number for both the breaker's rating and the load's
// demand is what "derived, not guessed" means here: the two describe the
// same physical circuit, so a second, independent derivation could only
// create a place for them to drift apart, never a more real number. Every
// basis string below names the exact `breakers.rs` function its own figure
// came from; where that catalogue's own figure was itself `GENERIC`
// (typical-class, no public A380 part number), this one says so too --
// see this pass's report for the full unsourced list.
//
// Three of `ata24_power_sources`'s six catalogued breakers -- BATTERY 1,
// BATTERY 2, APU BATTERY -- deliberately stay unmodelled here and
// `protected_load: None` in `catalog.rs`: a battery's own output/current-
// limiter breaker protects a *source's* output current, not a consumer's
// demand, and `network::Load` (this file's whole vocabulary) models demand
// only. `sources::Wiring::build` already gives each battery its own
// `network::Source` (`bat-1`/`bat-2`, a different id family and a
// different `Network` list -- `net.sources`, not `net.loads`) whose
// branch current is exactly what that breaker would be measuring. Adding a
// `Load` under the same id would double-count that current against the
// same battery's own `Source` branch, not model a second, real consumer --
// there is no consumer here for this pass to close.

/// ATA24 -- the two source-protection *control* circuits `ata24_power_
/// sources` catalogues a real load for (the battery output breakers
/// themselves stay unmodelled; see this section's own header comment).
fn ata24_power_sources_extra(net: &mut Network, cat: &mut Catalog) {
    let spec = avionics_spec(
        "ext-pwr-contactor",
        "EXTERNAL POWER CONTACTOR CONTROL",
        24,
        BusId::DcHot1,
        20.0,
        "breakers.rs::ata24_power_sources EXTERNAL POWER CONTACTOR CONTROL (GENERIC: typical small contactor-coil control circuit, real A380 external power system, no public per-part figure; same figure that catalogue's own breaker already cites)",
    );
    add(net, cat, LoadCategory::Other, spec.clone(), rated_current(&spec));
    for n in 1..=2u32 {
        let id: &'static str = Box::leak(format!("bat-charge-limiter-{n}").into_boxed_str());
        let name: &'static str = Box::leak(format!("BATTERY {n} CHARGE LIMITER").into_boxed_str());
        let basis: &'static str = Box::leak(format!("breakers.rs::ata24_power_sources BATTERY {n} CHARGE LIMITER (GENERIC: typical small charge-controller LRU control circuit; same figure that catalogue's own breaker already cites)").into_boxed_str());
        let spec = avionics_spec(id, name, 24, BusId::DcEss, 20.0, basis);
        add(net, cat, LoadCategory::Other, spec.clone(), rated_current(&spec));
    }
}

/// ATA23 -- communications LRUs `ata23_comms` catalogues (VHF 1/2 already
/// have their own load in [`avionics_misc`]; this is the rest of that
/// breaker function's group).
fn ata23_comms(net: &mut Network, cat: &mut Catalog) {
    let spec = avionics_spec("satcom", "SATCOM", 23, BusId::DcEss, 100.0, "breakers.rs::ata23_comms SATCOM (GENERIC: typical wide-body SATCOM transceiver LRU, real A380 equipment class, no public per-box figure; same figure that catalogue's own breaker already cites)");
    add(net, cat, LoadCategory::Other, spec.clone(), rated_current(&spec));
    let spec = avionics_spec("hf-1", "HF 1", 23, BusId::Dc1, 100.0, "breakers.rs::ata23_comms HF 1 (GENERIC: typical HF transceiver LRU; same figure that catalogue's own breaker already cites)");
    add(net, cat, LoadCategory::Other, spec.clone(), rated_current(&spec));
    let spec = avionics_spec("hf-2", "HF 2", 23, BusId::Dc2, 100.0, "breakers.rs::ata23_comms HF 2 (same class as HF 1)");
    add(net, cat, LoadCategory::Other, spec.clone(), rated_current(&spec));
    let spec = avionics_spec("acars-mu", "ACARS MU", 23, BusId::DcEss, 50.0, "breakers.rs::ata23_comms ACARS MU (GENERIC: typical avionics LRU class figure; same figure that catalogue's own breaker already cites)");
    add(net, cat, LoadCategory::Other, spec.clone(), rated_current(&spec));
    // The one AC-bus, non-avionics-pf entry in this group: a PA amplifier's
    // output stage is a real audio power amplifier, not a switching supply,
    // so it keeps the catalogue's own 0.9 power factor rather than
    // `avionics_spec`'s DC-LRU default.
    let spec = motor_spec("pa-amplifier", "PA AMPLIFIER", 23, BusId::Ac1, 200.0, 0.9, 1.3, 0.2, "breakers.rs::ata23_comms PA AMPLIFIER (GENERIC: typical wide-body PA amplifier power stage; same figure/power factor that catalogue's own breaker already cites)");
    add(net, cat, LoadCategory::Other, spec.clone(), rated_current(&spec));
    let spec = avionics_spec("interphone", "INTERPHONE", 23, BusId::DcEss, 50.0, "breakers.rs::ata23_comms INTERPHONE (GENERIC: typical avionics LRU class figure; same figure that catalogue's own breaker already cites)");
    add(net, cat, LoadCategory::Other, spec.clone(), rated_current(&spec));
}

/// ATA31 -- mandatory flight recorders (`ata31_recorders`).
fn ata31_recorders(net: &mut Network, cat: &mut Catalog) {
    let spec = avionics_spec("dfdr", "DFDR", 31, BusId::DcEss, 50.0, "breakers.rs::ata31_recorders DFDR (GENERIC: typical avionics LRU class figure, real mandatory A380 equipment; same figure that catalogue's own breaker already cites)");
    add(net, cat, LoadCategory::Essential, spec.clone(), rated_current(&spec));
    let spec = avionics_spec("cvr", "CVR", 31, BusId::DcEss, 50.0, "breakers.rs::ata31_recorders CVR (same class as DFDR, real mandatory A380 equipment)");
    add(net, cat, LoadCategory::Essential, spec.clone(), rated_current(&spec));
    // The QAR is a maintenance-data recorder, not flight-safety mandatory
    // equipment like the DFDR/CVR pair above, so it is kept `Other` rather
    // than `Essential`.
    let spec = avionics_spec("qar", "QAR", 31, BusId::Dc2, 30.0, "breakers.rs::ata31_recorders QAR (GENERIC: typical small avionics LRU class figure; same figure that catalogue's own breaker already cites)");
    add(net, cat, LoadCategory::Other, spec.clone(), rated_current(&spec));
}

/// ATA35 -- crew/passenger oxygen system consumers `ata35_oxygen`
/// catalogues (the shutoff valve's own position-indication circuit is
/// handled with the rest of [`position_indication_supplies`]).
fn ata35_oxygen_extra(net: &mut Network, cat: &mut Catalog) {
    let spec = motor_spec("crew-o2-shutoff", "CREW OXYGEN SHUTOFF VALVE", 35, BusId::Dc1, 50.0, 0.8, 2.0, 0.3, "breakers.rs::ata35_oxygen CREW OXYGEN SHUTOFF VALVE (GENERIC: typical motor/solenoid-operated shutoff valve actuator, real A380 crew oxygen system; same figure that catalogue's own breaker already cites)");
    add(net, cat, LoadCategory::Essential, spec.clone(), rated_current(&spec));
    let spec = avionics_spec("pax-o2-gen-ctl", "PAX OXYGEN GENERATOR CONTROL", 35, BusId::DcEss, 30.0, "breakers.rs::ata35_oxygen PAX OXYGEN GENERATOR CONTROL (GENERIC: typical small control-circuit LRU figure; same figure that catalogue's own breaker already cites)");
    add(net, cat, LoadCategory::Essential, spec.clone(), rated_current(&spec));
    let spec = avionics_spec("o2-pressure-xducer", "OXYGEN PRESSURE TRANSDUCER", 35, BusId::DcEss, 5.0, "breakers.rs::ata35_oxygen OXYGEN PRESSURE TRANSDUCER (GENERIC: typical small pressure-transducer power draw; same figure that catalogue's own breaker already cites)");
    add(net, cat, LoadCategory::Essential, spec.clone(), rated_current(&spec));
}

/// ATA49 -- APU controller/start consumers `ata49_apu` catalogues (the fuel
/// shutoff valve's own position-indication circuit is handled with the
/// rest of [`position_indication_supplies`]).
fn ata49_apu_extra(net: &mut Network, cat: &mut Catalog) {
    let spec = avionics_spec("apu-ecu-a", "APU ECU CHANNEL A", 49, BusId::DcApu, 60.0, "breakers.rs::ata49_apu APU ECU CHANNEL A (GENERIC: typical dual-channel engine/APU controller LRU class figure, real PW980 APU has its own FADEC-class controller; same figure that catalogue's own breaker already cites)");
    add(net, cat, LoadCategory::Essential, spec.clone(), rated_current(&spec));
    let spec = avionics_spec("apu-ecu-b", "APU ECU CHANNEL B", 49, BusId::DcEss, 60.0, "breakers.rs::ata49_apu APU ECU CHANNEL B (same class as channel A, redundant bus feed)");
    add(net, cat, LoadCategory::Essential, spec.clone(), rated_current(&spec));
    let spec = motor_spec("apu-fuel-shutoff-valve", "APU FUEL SHUTOFF VALVE", 49, BusId::Dc1, 50.0, 0.8, 2.0, 0.3, "breakers.rs::ata49_apu APU FUEL SHUTOFF VALVE (GENERIC: typical motor/solenoid-operated shutoff valve actuator; same figure that catalogue's own breaker already cites)");
    add(net, cat, LoadCategory::Essential, spec.clone(), rated_current(&spec));
    // Transit-only: a start contactor is only energised through the APU's
    // own start sequence (motoring the starter-generator), not once it is
    // self-sustaining -- `electrical::live`'s `TRANSIT_ONLY_NO_TRUTH_INPUT`
    // holds it off by default, the same honest "no Truth field for this
    // yet" choice already made for the gear actuators.
    let spec = avionics_spec("apu-start-contactor", "APU START CONTACTOR", 49, BusId::Dc1, 20.0, "breakers.rs::ata49_apu APU START CONTACTOR (GENERIC: typical contactor-coil control circuit; same figure that catalogue's own breaker already cites; transit-only, see electrical::live::TRANSIT_ONLY_NO_TRUTH_INPUT)");
    add(net, cat, LoadCategory::Essential, spec.clone(), rated_current(&spec));
}

/// ATA73/74 -- engine FADEC channels and ignition exciters `ata7x_engine`
/// catalogues. FADEC channels are continuous avionics LRUs (a real FADEC is
/// powered from the aircraft DC bus for monitoring even engine-off); the
/// ignition exciters are not -- a real exciter only fires during an engine
/// start or while continuous ignition is selected, so
/// `electrical::live::TRANSIT_ONLY_NO_TRUTH_INPUT` holds them off by
/// default absent a `Truth` field for either condition.
fn ata73_74_engine(net: &mut Network, cat: &mut Catalog) {
    for n in 1..=4u32 {
        let bus_a = if n <= 2 { BusId::DcEss } else { BusId::Dc1 };
        let bus_b = if n <= 2 { BusId::Dc2 } else { BusId::DcEss };
        let id_a: &'static str = Box::leak(format!("fadec-{n}a").into_boxed_str());
        let name_a: &'static str = Box::leak(format!("FADEC {n} CHANNEL A").into_boxed_str());
        let basis_a: &'static str = Box::leak(format!("breakers.rs::ata7x_engine FADEC {n} CHANNEL A (GENERIC: typical dual-lane FADEC-class controller channel, real Trent 972B-84 architecture, no public per-channel electrical figure; same figure that catalogue's own breaker already cites)").into_boxed_str());
        let spec = avionics_spec(id_a, name_a, 73, bus_a, 80.0, basis_a);
        add(net, cat, LoadCategory::Essential, spec.clone(), rated_current(&spec));
        let id_b: &'static str = Box::leak(format!("fadec-{n}b").into_boxed_str());
        let name_b: &'static str = Box::leak(format!("FADEC {n} CHANNEL B").into_boxed_str());
        let basis_b: &'static str = Box::leak(format!("breakers.rs::ata7x_engine FADEC {n} CHANNEL B (same class as channel A, redundant bus feed)").into_boxed_str());
        let spec = avionics_spec(id_b, name_b, 73, bus_b, 80.0, basis_b);
        add(net, cat, LoadCategory::Essential, spec.clone(), rated_current(&spec));
    }
    for n in 1..=4u32 {
        let bus_a = if n % 2 == 1 { BusId::Dc1 } else { BusId::Dc2 };
        let bus_b = if n % 2 == 1 { BusId::Dc2 } else { BusId::Dc1 };
        let id_a: &'static str = Box::leak(format!("ignition-{n}a").into_boxed_str());
        let name_a: &'static str = Box::leak(format!("IGNITION {n} EXCITER A").into_boxed_str());
        let basis_a: &'static str = Box::leak(format!("breakers.rs::ata7x_engine IGNITION {n} EXCITER A (GENERIC: typical high-energy ignition exciter unit pulsed power class (~250 W), no public per-part figure; same figure/power factor that catalogue's own breaker already cites; transit-only, see electrical::live::TRANSIT_ONLY_NO_TRUTH_INPUT)").into_boxed_str());
        let spec = motor_spec(id_a, name_a, 74, bus_a, 250.0, 0.9, 2.0, 0.5, basis_a);
        add(net, cat, LoadCategory::Essential, spec.clone(), rated_current(&spec));
        let id_b: &'static str = Box::leak(format!("ignition-{n}b").into_boxed_str());
        let name_b: &'static str = Box::leak(format!("IGNITION {n} EXCITER B").into_boxed_str());
        let basis_b: &'static str = Box::leak(format!("breakers.rs::ata7x_engine IGNITION {n} EXCITER B (same class as exciter A, redundant lane on the opposite DC bus)").into_boxed_str());
        let spec = motor_spec(id_b, name_b, 74, bus_b, 250.0, 0.9, 2.0, 0.5, basis_b);
        add(net, cat, LoadCategory::Essential, spec.clone(), rated_current(&spec));
    }
}

/// ATA26 -- engine/APU fire-extinguisher bottle squibs `ata26_extinguishing`
/// catalogues. A pyrotechnic squib is a one-shot device: it fires once on
/// a real discharge command and is otherwise dead, so (like the ignition
/// exciters above) it is held off by
/// `electrical::live::TRANSIT_ONLY_NO_TRUTH_INPUT` absent a `Truth` field
/// for "bottle discharge commanded".
fn ata26_extinguishing(net: &mut Network, cat: &mut Catalog) {
    for bottle in 1..=2u32 {
        for squib in 1..=2u32 {
            let bus = if bottle == 1 { BusId::Dc1 } else { BusId::Dc2 };
            let id: &'static str = Box::leak(format!("eng-fire-bottle-{bottle}-squib-{squib}").into_boxed_str());
            let name: &'static str = Box::leak(format!("ENG FIRE BOTTLE {bottle} SQUIB {squib}").into_boxed_str());
            let basis: &'static str = Box::leak(format!("breakers.rs::ata26_extinguishing ENG FIRE BOTTLE {bottle} SQUIB {squib} (GENERIC: typical one-shot pyrotechnic squib firing circuit, real wide-body cross-feed fire-extinguishing architecture; same figure that catalogue's own breaker already cites; transit-only/one-shot, see electrical::live::TRANSIT_ONLY_NO_TRUTH_INPUT)").into_boxed_str());
            let spec = avionics_spec(id, name, 26, bus, 20.0, basis);
            add(net, cat, LoadCategory::Essential, spec.clone(), rated_current(&spec));
        }
    }
    for squib in 1..=2u32 {
        let id: &'static str = Box::leak(format!("apu-fire-bottle-squib-{squib}").into_boxed_str());
        let name: &'static str = Box::leak(format!("APU FIRE BOTTLE SQUIB {squib}").into_boxed_str());
        let basis: &'static str = Box::leak(format!("breakers.rs::ata26_extinguishing APU FIRE BOTTLE SQUIB {squib} (same class as the engine bottle squibs; transit-only/one-shot)").into_boxed_str());
        let spec = avionics_spec(id, name, 26, BusId::DcApu, 20.0, basis);
        add(net, cat, LoadCategory::Essential, spec.clone(), rated_current(&spec));
    }
}

/// ATA29 -- the RAT deploy solenoid and PTU control valve
/// `ata29_hydraulics_extra` catalogues. The PTU valve is modelled
/// continuous, the same convention every other valve actuator in this
/// catalogue uses; the RAT solenoid is genuinely transit-only, and unlike
/// the ignition/squib/cargo-door set above this layer *does* have a real
/// signal for it (`ElectricalLive`'s own `emergency_config`/`rat_deployed`
/// state, the same state that already drives `sources::Rat::deploy`) --
/// see `electrical::live::ElectricalLive::tick`'s own gating of this load,
/// not a permanent off.
fn ata29_hydraulics_extra(net: &mut Network, cat: &mut Catalog) {
    let spec = avionics_spec("rat-deploy-solenoid", "RAT DEPLOY SOLENOID", 29, BusId::DcHot2, 100.0, "breakers.rs::ata29_hydraulics_extra RAT DEPLOY SOLENOID (GENERIC: typical deployment solenoid, hot-bus fed so it works with both engines/APU/main batteries down; same figure that catalogue's own breaker already cites; transit-only, gated in electrical::live against the real emergency/rat_deployed state)");
    add(net, cat, LoadCategory::Essential, spec.clone(), rated_current(&spec));
    let spec = motor_spec("ptu-control-valve", "PTU CONTROL VALVE", 29, BusId::DcEss, 50.0, 0.8, 2.0, 0.3, "breakers.rs::ata29_hydraulics_extra PTU CONTROL VALVE (GENERIC: typical motor/solenoid-operated valve actuator, real green/yellow hydraulic power-transfer-unit architecture; same figure that catalogue's own breaker already cites)");
    add(net, cat, LoadCategory::Essential, spec.clone(), rated_current(&spec));
}

/// ATA52 -- cargo door actuator control circuits `ata52_doors` catalogues.
/// Transit-only like the landing-gear actuators (a cargo door actuator
/// draws only while the door is actually moving), held off by
/// `electrical::live::TRANSIT_ONLY_NO_TRUTH_INPUT` absent a `Truth` field
/// for "door commanded".
fn ata52_doors(net: &mut Network, cat: &mut Catalog) {
    let spec = motor_spec("cargo-door-fwd-actuator-ctl", "FWD CARGO DOOR ACTUATOR CONTROL", 52, BusId::Dc1, 100.0, 0.8, 2.0, 0.3, "breakers.rs::ata52_doors FWD CARGO DOOR ACTUATOR CONTROL (GENERIC: typical powered cargo door actuator control circuit; same figure that catalogue's own breaker already cites; transit-only, see electrical::live::TRANSIT_ONLY_NO_TRUTH_INPUT)");
    add(net, cat, LoadCategory::Other, spec.clone(), rated_current(&spec));
    let spec = motor_spec("cargo-door-aft-actuator-ctl", "AFT CARGO DOOR ACTUATOR CONTROL", 52, BusId::Dc2, 100.0, 0.8, 2.0, 0.3, "breakers.rs::ata52_doors AFT CARGO DOOR ACTUATOR CONTROL (same class as the forward cargo door; transit-only)");
    add(net, cat, LoadCategory::Other, spec.clone(), rated_current(&spec));
}

/// ATA33 -- emergency-lighting battery chargers and exterior service
/// lighting `ata33_emergency_lighting` catalogues.
fn ata33_emergency_lighting(net: &mut Network, cat: &mut Catalog) {
    let spec = avionics_spec("emer-lighting-charger-1", "EMER LIGHTING BATTERY CHARGER 1", 33, BusId::DcHot1, 100.0, "breakers.rs::ata33_emergency_lighting EMER LIGHTING BATTERY CHARGER 1 (GENERIC: typical NiCd/Li-ion emergency-lighting pack charger circuit; same figure that catalogue's own breaker already cites)");
    add(net, cat, LoadCategory::Other, spec.clone(), rated_current(&spec));
    let spec = avionics_spec("emer-lighting-charger-2", "EMER LIGHTING BATTERY CHARGER 2", 33, BusId::DcHot2, 100.0, "breakers.rs::ata33_emergency_lighting EMER LIGHTING BATTERY CHARGER 2 (same class as charger 1)");
    add(net, cat, LoadCategory::Other, spec.clone(), rated_current(&spec));
    let spec = resistive_spec("ext-service-lighting", "EXTERIOR SERVICE LIGHTING", 33, BusId::AcGndFltSvc, 100.0, "breakers.rs::ata33_emergency_lighting EXTERIOR SERVICE LIGHTING (GENERIC: typical ground-service floodlight circuit; same figure that catalogue's own breaker already cites)");
    add(net, cat, LoadCategory::Other, spec.clone(), rated_current(&spec));
}

/// A position-indication microswitch/LVDT excitation supply: 5 W, unity
/// power factor regardless of an AC or DC parent bus (a small rectified
/// excitation circuit, not a line-frequency load) and negligible inrush (a
/// sensor excitation supply, not a motor) -- matching
/// `breakers.rs::push_position_excitation` exactly, id for id.
fn position_indication_spec(id: &'static str, name: &'static str, ata: u16, bus: BusId, basis: &'static str) -> LoadSpec {
    LoadSpec { id, name, ata, bus, rated_power_w: 5.0, power_factor: 1.0, min_operating_voltage: min_operating_voltage(bus), inrush_multiple: 1.0, inrush_duration_s: 0.0, wiring_resistance_ohm: wiring_resistance_ohm(bus), rated_frequency_hz: 0.0, basis }
}

fn position_indication_load(net: &mut Network, cat: &mut Catalog, parent_id: &'static str, parent_name: &'static str, ata: u16, bus: BusId, basis_suffix: &'static str) {
    let id: &'static str = Box::leak(format!("{parent_id}-pos-ind").into_boxed_str());
    let name: &'static str = Box::leak(format!("{parent_name} POSITION IND").into_boxed_str());
    let basis: &'static str = Box::leak(format!("breakers.rs::ata_control_excitation_supplies position-indication microswitch/LVDT excitation circuit (GENERIC 5 W, same figure that catalogue's own breaker already cites); {basis_suffix}").into_boxed_str());
    let spec = position_indication_spec(id, name, ata, bus, basis);
    add(net, cat, LoadCategory::Essential, spec.clone(), rated_current(&spec));
}

/// The 77 position-indication/excitation circuits `breakers.rs::ata_
/// control_excitation_supplies` pairs with an existing actuator breaker
/// above, one for one, same ATA/bus as their own parent actuator.
fn position_indication_supplies(net: &mut Network, cat: &mut Catalog) {
    let valve_buses = [BusId::Dc1, BusId::Dc2, BusId::DcEss, BusId::DcBat];
    for i in 0..60usize {
        let parent_id: &'static str = Box::leak(format!("fuel-valve-{i}").into_boxed_str());
        let parent_name: &'static str = Box::leak(format!("FUEL VALVE {i}").into_boxed_str());
        position_indication_load(net, cat, parent_id, parent_name, 28, valve_buses[i % valve_buses.len()], "pairs with this catalogue's own FUEL VALVE actuator breaker");
    }
    position_indication_load(net, cat, "hotair-1", "HOT AIR VALVE 1", 21, BusId::AcEss, "pairs with HOT AIR VALVE 1's own actuator breaker");
    position_indication_load(net, cat, "hotair-2", "HOT AIR VALVE 2", 21, BusId::AcEss, "pairs with HOT AIR VALVE 2's own actuator breaker");
    position_indication_load(net, cat, "fwd-isol-valve", "FWD CARGO ISOL VALVE", 21, BusId::Dc2, "pairs with FWD CARGO ISOL VALVE's own actuator breaker");
    position_indication_load(net, cat, "bulk-isol-valve", "BULK CARGO ISOL VALVE", 21, BusId::DcEss, "pairs with BULK CARGO ISOL VALVE's own actuator breaker");
    for pack in 1..=2u32 {
        for side in 1..=2u32 {
            let parent_id: &'static str = Box::leak(format!("pack-{pack}-flow-valve-{side}").into_boxed_str());
            let parent_name: &'static str = Box::leak(format!("PACK {pack} FLOW VALVE {side}").into_boxed_str());
            position_indication_load(net, cat, parent_id, parent_name, 21, BusId::DcEss, "pairs with its own PACK FLOW VALVE actuator breaker");
        }
    }
    for n in 1..=4u32 {
        let bus = if n <= 2 { BusId::Dc1 } else { BusId::Dc2 };
        let parent_id: &'static str = Box::leak(format!("bleed-eng-{n}").into_boxed_str());
        let parent_name: &'static str = Box::leak(format!("BLEED ENG {n} VALVES").into_boxed_str());
        position_indication_load(net, cat, parent_id, parent_name, 36, bus, "pairs with the shared BLEED ENG valve-set actuator breaker");
    }
    position_indication_load(net, cat, "ptu-control-valve", "PTU CONTROL VALVE", 29, BusId::DcEss, "pairs with PTU CONTROL VALVE's own actuator breaker");
    position_indication_load(net, cat, "apu-fuel-shutoff-valve", "APU FUEL SHUTOFF VALVE", 49, BusId::Dc1, "pairs with APU FUEL SHUTOFF VALVE's own actuator breaker");
    position_indication_load(net, cat, "crew-o2-shutoff", "CREW OXYGEN SHUTOFF VALVE", 35, BusId::Dc1, "pairs with CREW OXYGEN SHUTOFF VALVE's own actuator breaker");
    position_indication_load(net, cat, "cargo-door-fwd-actuator-ctl", "FWD CARGO DOOR", 52, BusId::Dc1, "pairs with the forward cargo door's own actuator-control breaker");
    position_indication_load(net, cat, "cargo-door-aft-actuator-ctl", "AFT CARGO DOOR", 52, BusId::Dc2, "pairs with the aft cargo door's own actuator-control breaker");
}

/// Build the whole catalogue onto `net`, returning the category index for
/// `shedding.rs`.
pub fn build(net: &mut Network) -> Catalog {
    let mut cat = Catalog::new();
    ata21(net, &mut cat);
    ata26(net, &mut cat);
    ata27(net, &mut cat);
    ata32(net, &mut cat);
    ata34(net, &mut cat);
    ata28_fuel(net, &mut cat);
    ata33_lighting(net, &mut cat);
    ata30_ice_protection(net, &mut cat);
    ata25_galleys(net, &mut cat);
    ata44_ife(net, &mut cat);
    avionics_misc(net, &mut cat);
    ata36_bleed(net, &mut cat);
    // Closing `deep::breakers::catalog`'s own named gap (see this file's own
    // section above): each function below adds exactly the load a
    // group-2/group-3 breaker in that catalogue already names and rates.
    ata24_power_sources_extra(net, &mut cat);
    ata23_comms(net, &mut cat);
    ata31_recorders(net, &mut cat);
    ata35_oxygen_extra(net, &mut cat);
    ata49_apu_extra(net, &mut cat);
    ata73_74_engine(net, &mut cat);
    ata26_extinguishing(net, &mut cat);
    ata29_hydraulics_extra(net, &mut cat);
    ata52_doors(net, &mut cat);
    ata33_emergency_lighting(net, &mut cat);
    position_indication_supplies(net, &mut cat);
    cat
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_full_catalogue_builds_without_duplicate_ids_and_every_load_has_its_own_breaker() {
        let mut net = Network::new();
        let cat = build(&mut net);
        let mut ids: Vec<&str> = net.loads.iter().map(|l| l.spec.id).collect();
        let before = ids.len();
        ids.sort_unstable();
        ids.dedup();
        assert_eq!(ids.len(), before, "duplicate load id in the catalogue");
        // At least one breaker per load, but a real dual/triple-fed LRU
        // (`add_dual`/`add_triple`) now has more breakers than loads -- one
        // per feed, not one per load.
        assert!(net.breakers.len() >= net.loads.len(), "every load needs at least one breaker: {} breakers, {} loads", net.breakers.len(), net.loads.len());
        for load in &net.loads {
            assert!(!load.feeds.is_empty(), "{} has no feeds at all", load.spec.id);
        }
        assert!(net.loads.len() > 200, "expected a substantial catalogue, got {}", net.loads.len());
        assert_eq!(cat.galley.len() + cat.commercial.len() + cat.essential.len() + cat.other.len(), net.loads.len());
    }

    #[test]
    fn every_load_has_a_positive_rating_and_a_cited_basis() {
        let mut net = Network::new();
        build(&mut net);
        for load in &net.loads {
            assert!(load.spec.rated_power_w > 0.0, "{} has no rated power", load.spec.id);
            assert!(!load.spec.basis.is_empty(), "{} has no basis citation", load.spec.id);
        }
    }

    #[test]
    fn galleys_and_ife_are_flagged_commercial_shed_candidates_not_essential() {
        let mut net = Network::new();
        let cat = build(&mut net);
        assert!(!cat.galley.is_empty());
        assert!(!cat.commercial.is_empty());
        for &i in &cat.galley {
            assert_eq!(net.loads[i].spec.ata, 25);
        }
        for &i in &cat.essential {
            assert_ne!(net.loads[i].spec.ata, 25, "a galley should never be marked essential");
        }
    }

    #[test]
    fn fuel_pumps_and_valves_are_each_their_own_load() {
        let mut net = Network::new();
        build(&mut net);
        let pumps = net.loads.iter().filter(|l| l.spec.id.starts_with("fuel-pump-")).count();
        // `starts_with("fuel-valve-")` also matches the 60
        // `fuel-valve-N-pos-ind` position-indication loads this pass added
        // (a real, separate circuit from the valve's own actuator, see
        // `position_indication_supplies`), so this counts the actuator
        // loads specifically by excluding that suffix.
        let valves = net.loads.iter().filter(|l| l.spec.id.starts_with("fuel-valve-") && !l.spec.id.ends_with("-pos-ind")).count();
        let valve_pos_ind = net.loads.iter().filter(|l| l.spec.id.starts_with("fuel-valve-") && l.spec.id.ends_with("-pos-ind")).count();
        assert_eq!(pumps, 25);
        assert_eq!(valves, 60);
        assert_eq!(valve_pos_ind, 60);
    }

    /// The 125 ids `deep::breakers::catalog`'s group-2/group-3 entries name
    /// (128 minus the three battery-output breakers this file's own header
    /// comment explains cannot be a `Load`) must all resolve to a real load
    /// here, one for one -- this is what actually closes the gap, not just
    /// what the breaker catalogue's own text claims.
    #[test]
    fn every_closed_gap_id_is_a_real_load() {
        let mut net = Network::new();
        build(&mut net);
        let ids: std::collections::HashSet<&str> = net.loads.iter().map(|l| l.spec.id).collect();
        let mut expected: Vec<String> = vec![
            "ext-pwr-contactor".into(),
            "bat-charge-limiter-1".into(),
            "bat-charge-limiter-2".into(),
            "satcom".into(),
            "hf-1".into(),
            "hf-2".into(),
            "acars-mu".into(),
            "pa-amplifier".into(),
            "interphone".into(),
            "dfdr".into(),
            "cvr".into(),
            "qar".into(),
            "crew-o2-shutoff".into(),
            "pax-o2-gen-ctl".into(),
            "o2-pressure-xducer".into(),
            "apu-ecu-a".into(),
            "apu-ecu-b".into(),
            "apu-fuel-shutoff-valve".into(),
            "apu-start-contactor".into(),
            "rat-deploy-solenoid".into(),
            "ptu-control-valve".into(),
            "cargo-door-fwd-actuator-ctl".into(),
            "cargo-door-aft-actuator-ctl".into(),
            "emer-lighting-charger-1".into(),
            "emer-lighting-charger-2".into(),
            "ext-service-lighting".into(),
        ];
        for n in 1..=4 {
            expected.push(format!("fadec-{n}a"));
            expected.push(format!("fadec-{n}b"));
            expected.push(format!("ignition-{n}a"));
            expected.push(format!("ignition-{n}b"));
        }
        for bottle in 1..=2 {
            for squib in 1..=2 {
                expected.push(format!("eng-fire-bottle-{bottle}-squib-{squib}"));
            }
        }
        expected.push("apu-fire-bottle-squib-1".into());
        expected.push("apu-fire-bottle-squib-2".into());
        for i in 0..60 {
            expected.push(format!("fuel-valve-{i}-pos-ind"));
        }
        for parent in ["hotair-1", "hotair-2", "fwd-isol-valve", "bulk-isol-valve", "ptu-control-valve", "apu-fuel-shutoff-valve", "crew-o2-shutoff", "cargo-door-fwd-actuator-ctl", "cargo-door-aft-actuator-ctl"] {
            expected.push(format!("{parent}-pos-ind"));
        }
        for pack in 1..=2 {
            for side in 1..=2 {
                expected.push(format!("pack-{pack}-flow-valve-{side}-pos-ind"));
            }
        }
        for n in 1..=4 {
            expected.push(format!("bleed-eng-{n}-pos-ind"));
        }
        assert_eq!(expected.len(), 125, "the expected list itself must total 125 (128 minus the 3 battery-output breakers)");
        let mut missing: Vec<&String> = expected.iter().filter(|id| !ids.contains(id.as_str())).collect();
        missing.sort();
        assert!(missing.is_empty(), "gap-closing ids with no real load: {missing:?}");

        // And the three battery-output breakers must NOT have grown a
        // fabricated load -- see this file's own header comment for why.
        for id in ["bat-1", "bat-2", "bat-apu"] {
            assert!(!ids.contains(id), "{id} is a source's own output breaker, not a consumer -- it must not have a Load");
        }
    }

    /// The new transit-only/one-shot loads must actually be dead by
    /// default (`commanded_on: true` is `Load::new`'s own default) --
    /// `electrical::live::ElectricalLive::new` is what turns that off, so
    /// this only checks the raw catalogue is honestly "on" here; the live
    /// behaviour is asserted in `electrical::live`'s own tests.
    #[test]
    fn every_gap_closing_load_defaults_on_like_every_other_catalogue_entry() {
        let mut net = Network::new();
        build(&mut net);
        for id in ["ignition-1a", "eng-fire-bottle-1-squib-1", "apu-start-contactor", "cargo-door-fwd-actuator-ctl", "rat-deploy-solenoid"] {
            let idx = net.load_index(id).unwrap_or_else(|| panic!("{id} missing"));
            assert!(net.loads[idx].commanded_on, "{id} should default on in the raw catalogue; live.rs is what gates it");
        }
    }
}
