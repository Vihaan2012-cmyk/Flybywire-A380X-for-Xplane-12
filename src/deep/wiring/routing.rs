//! A380 routing: plausible GENERIC routes for real A380 circuits from their
//! bus/panel to their consumer's zone, respecting segregation (redundant
//! systems routed apart so a single event cannot take out both sides unless
//! their bundles genuinely share a zone).
//!
//! The circuit ids, real bus feeds and consumer descriptions below are
//! reproduced from `D:\fbw-xp-systems\src\breakers.rs`'s own catalogue
//! (read in full for this task's context, per `docs/deep/BRIEF.md`'s own
//! instruction to read it) as **literal data** -- this module does not
//! `use` or otherwise depend on `crate::breakers` (this push's
//! self-contained-directory rule): the ids are copied so a future
//! integration layer can join on them by string, and the bus/consumer
//! facts are copied so this catalogue's routing decisions are grounded in
//! the plugin's own real breaker set rather than invented from nothing.
//! Segment lengths, exact panel locations within a zone and the specific
//! gauge/insulation chosen for a given circuit's class are GENERIC (no
//! public A380 wiring diagram exists); the *topology* -- which zones a
//! circuit's power must physically cross, and which side/essential channel
//! it belongs to -- follows the real bus assignments below and the
//! standard two-generation-channel-plus-essential-channel segregation
//! principle common to Airbus fly-by-wire types (public, general
//! knowledge; not a verified A380-specific pairing beyond what
//! `breakers.rs`'s own catalogue already fixes for AC1-4/DC1-2/ESS).

use super::bundle::{CircuitWire, Segment, WireBundleNetwork};
use super::gauge::{Awg, Insulation};
use super::zones::Zone;

/// Which independent electrical channel a circuit belongs to, for the
/// segregation check: two circuits on the same [`Side`] are allowed to
/// share a segment; two on *different* [`Side`]s that both matter for the
/// same redundant function should not (`routing::segregation_report`
/// flags it when a generated network does).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Side {
    /// Engines 1/2 (left wing), AC1/AC2/DC1.
    Side1,
    /// Engines 3/4 (right wing), AC3/AC4/DC2.
    Side2,
    /// Battery/essential channel: AC_ESS(_SHED), DC_ESS, DC_HOT*, the
    /// static inverter -- stays live when both normal generation channels
    /// are lost.
    Ess,
    /// APU generation/DC APU bus.
    Apu,
    Ground,
}

/// Real A380 bus label (matching `circuits::MSFS_BUSES`'s own FBW names,
/// read as context, reproduced independently as a plain string key) ->
/// [`Side`]. Unknown labels default to [`Side::Ess`] -- the safe choice for
/// a segregation check, since assuming redundancy that was never verified
/// is the wrong direction to guess in.
pub fn side_of_bus(bus: &str) -> Side {
    match bus {
        "AC1" | "AC2" | "DC1" => Side::Side1,
        "AC3" | "AC4" | "DC2" => Side::Side2,
        "AC_ESS" | "AC_ESS_SHED" | "AC_247XP" | "DC_ESS" | "DC_HOT1" | "DC_HOT2" | "DC_HOT3" | "DC_HOT4" | "AC_STAT_INV" | "DC_247PP" => Side::Ess,
        "APU_GEN" | "309PP" | "DC_309PP" => Side::Apu,
        "AC_GND_FLT_SVC" | "DC_GND_FLT_SVC" => Side::Ground,
        _ => Side::Ess,
    }
}

/// Engine 1/2 -> Side1 (left wing), 3/4 -> Side2 (right wing) -- standard
/// four-engine transport wing layout, public/general.
pub fn side_of_engine(n: u8) -> Side {
    if n <= 2 {
        Side::Side1
    } else {
        Side::Side2
    }
}

const LRU_120C_ETFE: WireSpec2 = WireSpec2 { awg: Awg::Size(20), insulation: Insulation::Etfe150 };
const FEEDER_150C: WireSpec2 = WireSpec2 { awg: Awg::Size(4), insulation: Insulation::Ptfe200 };
const HEAVY_FEEDER: WireSpec2 = WireSpec2 { awg: Awg::Aught(1), insulation: Insulation::Ptfe200 };
const SENSOR_260C: WireSpec2 = WireSpec2 { awg: Awg::Size(22), insulation: Insulation::Ptfe260 };
const VALVE_ACTUATOR_WIRE: WireSpec2 = WireSpec2 { awg: Awg::Size(16), insulation: Insulation::Etfe150 };

/// Local alias so this file's table below reads as plain data (`gauge::WireSpec`
/// is a tuple-free struct already, but naming it here keeps the table compact).
type WireSpec2 = super::gauge::WireSpec;

/// One catalogue entry: a real breaker id (`breakers.rs`'s own `id`), its
/// real bus label, and its consumer's zone, class and per-segment wire.
struct RouteEntry {
    circuit: &'static str,
    bus: &'static str,
    /// Zones crossed, source (bus/panel end) to consumer end, inclusive.
    /// Most avionics LRUs live in the same bay as their bus bar, so this is
    /// often a single zone; a generator or an APU feeder genuinely crosses
    /// several.
    path: &'static [Zone],
    wire: WireSpec2,
}

/// The real breaker catalogue's routing (`breakers.rs` citations inline).
/// Not exhaustive of all ~265 breakers -- a representative, real-grounded
/// slice across every ATA chapter `breakers.rs` covers, enough for the
/// segregation/query/fault backlog items to exercise real topology. Extend
/// this table (not the network-building code) to cover more.
fn catalogue() -> Vec<RouteEntry> {
    use Zone::*;
    vec![
        // ATA24: TRUs and the static inverter physically rack-mounted in
        // the main equipment centre alongside their own bus bar
        // (breakers.rs ata24: tr-1 DC1/Msfs(8), tr-2 DC2/Msfs(9), tr-ess
        // DC_ESS/Msfs(10), tr-apu DC_APU/Msfs(11), static-inv AC_STAT_INV).
        RouteEntry { circuit: "tr-1", bus: "AC1", path: &[MainAvionics], wire: FEEDER_150C },
        RouteEntry { circuit: "tr-2", bus: "AC2", path: &[MainAvionics], wire: FEEDER_150C },
        RouteEntry { circuit: "tr-ess", bus: "AC_ESS", path: &[MainAvionics], wire: FEEDER_150C },
        RouteEntry { circuit: "tr-apu", bus: "309PP", path: &[MainAvionics], wire: FEEDER_150C },
        RouteEntry { circuit: "static-inv", bus: "AC_STAT_INV", path: &[MainAvionics], wire: LRU_120C_ETFE },
        // Generators: the VFG itself is on the engine accessory gearbox: a
        // heavy feeder runs engine -> wing root -> main avionics bus bar
        // (breakers.rs ata24 gens 1-4, buses AC1-4).
        RouteEntry { circuit: "gen-1", bus: "AC1", path: &[Engine(1), WingRoot, MainAvionics], wire: HEAVY_FEEDER },
        RouteEntry { circuit: "gen-2", bus: "AC2", path: &[Engine(2), WingRoot, MainAvionics], wire: HEAVY_FEEDER },
        RouteEntry { circuit: "gen-3", bus: "AC3", path: &[Engine(3), WingRoot, MainAvionics], wire: HEAVY_FEEDER },
        RouteEntry { circuit: "gen-4", bus: "AC4", path: &[Engine(4), WingRoot, MainAvionics], wire: HEAVY_FEEDER },
        RouteEntry { circuit: "apu-gen-1", bus: "APU_GEN", path: &[Apu, TailCone, WingRoot, MainAvionics], wire: HEAVY_FEEDER },
        RouteEntry { circuit: "apu-gen-2", bus: "APU_GEN", path: &[Apu, TailCone, WingRoot, MainAvionics], wire: HEAVY_FEEDER },
        // ATA32: LGCIUs are avionics-bay LRUs (breakers.rs ata32: lgciu-1
        // DC_ESS/Msfs(10), lgciu-2 DC2/Msfs(9) -- a real asymmetric-bus
        // redundancy pattern, Ess vs Side2, not a matched pair).
        RouteEntry { circuit: "lgciu-1", bus: "DC_ESS", path: &[MainAvionics], wire: LRU_120C_ETFE },
        RouteEntry { circuit: "lgciu-2", bus: "DC2", path: &[MainAvionics], wire: LRU_120C_ETFE },
        // Gear/door proximity sensors and actuators: LGCIU-driven, DC_ESS
        // (breakers.rs ata32_gear_and_door_sensors), but physically the
        // sensor/actuator itself sits in the gear bay its name says, so the
        // wire genuinely runs avionics -> wing root -> that bay.
        RouteEntry { circuit: "prox-uplock-gear-nose-1", bus: "DC_ESS", path: &[MainAvionics, WingRoot, NoseGearBay], wire: SENSOR_260C },
        RouteEntry { circuit: "prox-downlock-gear-nose-2", bus: "DC_ESS", path: &[MainAvionics, WingRoot, NoseGearBay], wire: SENSOR_260C },
        RouteEntry { circuit: "prox-uplock-gear-left-1", bus: "DC_ESS", path: &[MainAvionics, WingRoot, MainGearBay], wire: SENSOR_260C },
        RouteEntry { circuit: "prox-downlock-gear-left-2", bus: "DC_ESS", path: &[MainAvionics, WingRoot, MainGearBay], wire: SENSOR_260C },
        RouteEntry { circuit: "prox-uplock-gear-right-1", bus: "DC_ESS", path: &[MainAvionics, WingRoot, MainGearBay], wire: SENSOR_260C },
        RouteEntry { circuit: "prox-downlock-gear-right-2", bus: "DC_ESS", path: &[MainAvionics, WingRoot, MainGearBay], wire: SENSOR_260C },
        RouteEntry { circuit: "gear-actuator-nose", bus: "DC_ESS", path: &[MainAvionics, WingRoot, NoseGearBay], wire: FEEDER_150C },
        RouteEntry { circuit: "gear-actuator-left", bus: "DC_ESS", path: &[MainAvionics, WingRoot, MainGearBay], wire: FEEDER_150C },
        RouteEntry { circuit: "gear-actuator-right", bus: "DC_ESS", path: &[MainAvionics, WingRoot, MainGearBay], wire: FEEDER_150C },
        // ATA29: the 4 electric hydraulic pumps live at the wing root
        // (breakers.rs ata32 hyd-epump-*, buses green A/B/yellow A/B).
        RouteEntry { circuit: "hyd-epump-ga", bus: "AC2", path: &[MainAvionics, WingRoot], wire: FEEDER_150C },
        RouteEntry { circuit: "hyd-epump-gb", bus: "AC3", path: &[MainAvionics, WingRoot], wire: FEEDER_150C },
        RouteEntry { circuit: "hyd-epump-ya", bus: "AC4", path: &[MainAvionics, WingRoot], wire: FEEDER_150C },
        RouteEntry { circuit: "hyd-epump-yb", bus: "AC1", path: &[MainAvionics, WingRoot], wire: FEEDER_150C },
        // ATA26: fire detection loops, per zone per A/B loop (breakers.rs
        // ata26 -- each pair on DC_ESS/Msfs(10), one breaker per zone per
        // loop). Real A/B loops are run physically separated where the
        // airframe allows it; the routing table gives loop A and loop B
        // *different* segment ids in the same zone (see `build_generic_
        // a380_network`) so a segment-scoped fault cannot silently take
        // both, matching that real segregation intent.
        RouteEntry { circuit: "fire-loop-eng-1-a", bus: "DC_ESS", path: &[MainAvionics, WingRoot, Engine(1)], wire: SENSOR_260C },
        RouteEntry { circuit: "fire-loop-eng-1-b", bus: "DC_ESS", path: &[MainAvionics, WingRoot, Engine(1)], wire: SENSOR_260C },
        RouteEntry { circuit: "fire-loop-eng-2-a", bus: "DC_ESS", path: &[MainAvionics, WingRoot, Engine(2)], wire: SENSOR_260C },
        RouteEntry { circuit: "fire-loop-eng-2-b", bus: "DC_ESS", path: &[MainAvionics, WingRoot, Engine(2)], wire: SENSOR_260C },
        RouteEntry { circuit: "fire-loop-apu-a", bus: "DC_ESS", path: &[MainAvionics, TailCone, Apu], wire: SENSOR_260C },
        RouteEntry { circuit: "fire-loop-apu-b", bus: "DC_ESS", path: &[MainAvionics, TailCone, Apu], wire: SENSOR_260C },
        RouteEntry { circuit: "fire-loop-mlg-bay-a", bus: "DC_ESS", path: &[MainAvionics, WingRoot, MainGearBay], wire: SENSOR_260C },
        RouteEntry { circuit: "fire-loop-mlg-bay-b", bus: "DC_ESS", path: &[MainAvionics, WingRoot, MainGearBay], wire: SENSOR_260C },
        // ATA34: radio altimeters, main-avionics LRUs (breakers.rs ata34,
        // AC1/AC2/AC_ESS -- "RA SYS A/B/C").
        RouteEntry { circuit: "ra-sys-a", bus: "AC1", path: &[MainAvionics], wire: LRU_120C_ETFE },
        RouteEntry { circuit: "ra-sys-b", bus: "AC2", path: &[MainAvionics], wire: LRU_120C_ETFE },
        RouteEntry { circuit: "ra-sys-c", bus: "AC_ESS", path: &[MainAvionics], wire: LRU_120C_ETFE },
        RouteEntry { circuit: "egpwc", bus: "AC_ESS", path: &[MainAvionics], wire: LRU_120C_ETFE },
        // ATA27: flight-control computers (breakers.rs ata27 alternates
        // DC_ESS/DC2), main-avionics LRUs.
        RouteEntry { circuit: "prim-1", bus: "DC_ESS", path: &[MainAvionics], wire: LRU_120C_ETFE },
        RouteEntry { circuit: "prim-2", bus: "DC2", path: &[MainAvionics], wire: LRU_120C_ETFE },
        RouteEntry { circuit: "prim-3", bus: "DC_ESS", path: &[MainAvionics], wire: LRU_120C_ETFE },
        RouteEntry { circuit: "sec-1", bus: "DC2", path: &[MainAvionics], wire: LRU_120C_ETFE },
        RouteEntry { circuit: "sec-2", bus: "DC_ESS", path: &[MainAvionics], wire: LRU_120C_ETFE },
        RouteEntry { circuit: "sec-3", bus: "DC2", path: &[MainAvionics], wire: LRU_120C_ETFE },
        RouteEntry { circuit: "fcdc-1", bus: "DC_ESS", path: &[MainAvionics], wire: LRU_120C_ETFE },
        RouteEntry { circuit: "fcdc-2", bus: "DC2", path: &[MainAvionics], wire: LRU_120C_ETFE },
        // ATA36: engine bleed valves, one feeder per engine from its DC bus
        // out to the pylon/nacelle (breakers.rs ata36: eng1/2 DC1, eng3/4
        // DC2).
        RouteEntry { circuit: "bleed-eng-1", bus: "DC1", path: &[MainAvionics, WingRoot, Engine(1)], wire: VALVE_ACTUATOR_WIRE },
        RouteEntry { circuit: "bleed-eng-2", bus: "DC1", path: &[MainAvionics, WingRoot, Engine(2)], wire: VALVE_ACTUATOR_WIRE },
        RouteEntry { circuit: "bleed-eng-3", bus: "DC2", path: &[MainAvionics, WingRoot, Engine(3)], wire: VALVE_ACTUATOR_WIRE },
        RouteEntry { circuit: "bleed-eng-4", bus: "DC2", path: &[MainAvionics, WingRoot, Engine(4)], wire: VALVE_ACTUATOR_WIRE },
        // ATA21: cabin recirculation fans, one per numbered AC bus
        // (breakers.rs ata21 cab-fan-1..4), and the two cargo isolation
        // valve/extract fan pairs, local to their own cargo compartment.
        RouteEntry { circuit: "cab-fan-1", bus: "AC1", path: &[MainAvionics, Cockpit], wire: FEEDER_150C },
        RouteEntry { circuit: "cab-fan-2", bus: "AC2", path: &[MainAvionics, Cockpit], wire: FEEDER_150C },
        RouteEntry { circuit: "fwd-isol-valve", bus: "DC2", path: &[MainAvionics, CargoFwd], wire: VALVE_ACTUATOR_WIRE },
        RouteEntry { circuit: "fwd-extract-fan", bus: "DC2", path: &[MainAvionics, CargoFwd], wire: FEEDER_150C },
        RouteEntry { circuit: "bulk-isol-valve", bus: "DC_ESS", path: &[MainAvionics, CargoAft], wire: VALVE_ACTUATOR_WIRE },
        RouteEntry { circuit: "bulk-extract-fan", bus: "DC_ESS", path: &[MainAvionics, CargoAft], wire: FEEDER_150C },
        RouteEntry { circuit: "cargo-heater", bus: "AC2", path: &[MainAvionics, CargoAft], wire: FEEDER_150C },
    ]
}

/// Deterministic segment length for a `(from, to)` zone hop, m -- GENERIC,
/// sized to a plausible A380-scale run for that pair (a same-bay run is a
/// few metres of local looming; an avionics-bay-to-wing-root-to-nacelle run
/// is tens of metres, consistent with the aircraft's ~80 m length and ~80 m
/// wingspan). No public A380 wiring-diagram length exists for any specific
/// run.
fn hop_length_m(from: Zone, to: Zone) -> f64 {
    use Zone::*;
    match (from, to) {
        (a, b) if a == b => 3.0, // within one zone, e.g. bus bar to a nearby LRU
        (MainAvionics, WingRoot) | (WingRoot, MainAvionics) => 25.0,
        (WingRoot, Engine(_)) | (Engine(_), WingRoot) => 20.0,
        (WingRoot, MainGearBay) | (MainGearBay, WingRoot) => 10.0,
        (MainAvionics, NoseGearBay) | (NoseGearBay, MainAvionics) => 15.0,
        (WingRoot, NoseGearBay) | (NoseGearBay, WingRoot) => 12.0,
        (MainAvionics, TailCone) | (TailCone, MainAvionics) => 45.0,
        (TailCone, Apu) | (Apu, TailCone) => 5.0,
        (MainAvionics, CargoFwd) | (CargoFwd, MainAvionics) => 8.0,
        (MainAvionics, CargoAft) | (CargoAft, MainAvionics) => 30.0,
        (MainAvionics, Cockpit) | (Cockpit, MainAvionics) => 6.0,
        _ => 15.0, // any other adjacency: a plausible mid-length run, GENERIC
    }
}

/// Builds the generic A380 wire-bundle network from [`catalogue`]: one
/// segment per `(zone, side)` pair actually used by a circuit crossing that
/// zone on that hop (so Side1/Side2/Ess/Apu circuits crossing the *same*
/// zone still land in *different* segments -- the segregation rule), except
/// fire loops A/B, which are further split per loop id even though both are
/// nominally the same [`Side`], matching their own real physical
/// separation.
pub fn build_generic_a380_network() -> WireBundleNetwork {
    let mut net = WireBundleNetwork::new();
    let entries = catalogue();

    // Collect (segment_id, zone, length) -> members, keyed so every
    // (from,to,side_or_loop) hop becomes exactly one segment shared by
    // every circuit taking that same hop on that same side.
    use std::collections::BTreeMap;
    let mut segments: BTreeMap<&'static str, (Zone, f64, Vec<CircuitWire>)> = BTreeMap::new();
    let mut routes: Vec<(&'static str, Vec<&'static str>)> = Vec::new();

    for e in &entries {
        let side = side_of_bus(e.bus);
        let mut seg_ids = Vec::new();
        for &zone in e.path {
            let seg_id = segment_id(zone, side, e.circuit);
            let len = hop_length_m(zone, zone_before(e.path, zone).unwrap_or(zone));
            let entry = segments.entry(seg_id).or_insert_with(|| (zone, len, Vec::new()));
            if !entry.2.iter().any(|c: &CircuitWire| c.circuit == e.circuit) {
                entry.2.push(CircuitWire { circuit: e.circuit, wire: e.wire });
            }
            seg_ids.push(seg_id);
        }
        routes.push((e.circuit, seg_ids));
    }

    for (id, (zone, length_m, circuits)) in segments {
        net.add_segment(Segment { id, zone, length_m, circuits });
    }
    for (circuit, seg_ids) in routes {
        net.set_route(circuit, seg_ids);
    }
    net
}

/// The zone immediately before `zone` in `path` (for the hop length), or
/// `None` if `zone` is the first stop (uses the same length as a same-zone
/// hop -- the local run from the bus bar to this zone's own panel).
fn zone_before(path: &[Zone], zone: Zone) -> Option<Zone> {
    let i = path.iter().position(|&z| z == zone)?;
    if i == 0 {
        Some(zone)
    } else {
        Some(path[i - 1])
    }
}

/// A stable segment id for one `(zone, side)` hop -- shared by every
/// circuit on that side crossing that zone -- except fire-loop circuits,
/// which each get their own id (`"loop-<a|b>-<zone>"`) regardless of side,
/// since loop A and loop B must never collapse onto the same physical
/// bundle even though both are nominally [`Side::Ess`].
fn segment_id(zone: Zone, side: Side, circuit: &'static str) -> &'static str {
    if let Some(loop_id) = fire_loop_segment_override(zone, circuit) {
        return loop_id;
    }
    let side_tag = match side {
        Side::Side1 => "side1",
        Side::Side2 => "side2",
        Side::Ess => "ess",
        Side::Apu => "apu",
        Side::Ground => "gnd",
    };
    Box::leak(format!("seg-{}-{}", zone.name().to_lowercase(), side_tag).into_boxed_str())
}

fn fire_loop_segment_override(zone: Zone, circuit: &'static str) -> Option<&'static str> {
    if !circuit.starts_with("fire-loop-") {
        return None;
    }
    let loop_tag = if circuit.ends_with("-a") { "a" } else { "b" };
    Some(Box::leak(format!("seg-fireloop-{}-{}", loop_tag, zone.name().to_lowercase()).into_boxed_str()))
}

/// Segregation report: every pair of segments that occupy the *same* zone
/// but were built for *different, both-matter* sides (Side1 vs Side2, or
/// two different fire-loop ids) is fine on its own -- that is the point of
/// splitting by side/loop. This instead flags the one thing that must never
/// happen: a single segment id itself listing circuits from two different
/// non-`Ess` sides (Side1 *and* Side2 sharing one physical bundle), which
/// would mean a single chafe/fire event could take out both a system and
/// its own redundant partner without a zone-wide fault.
pub fn segregation_violations(net: &WireBundleNetwork) -> Vec<String> {
    let mut out = Vec::new();
    for seg in net.segments() {
        let sides: Vec<Side> = seg.circuits.iter().map(|c| circuit_side(c.circuit)).collect();
        let has_side1 = sides.contains(&Side::Side1);
        let has_side2 = sides.contains(&Side::Side2);
        if has_side1 && has_side2 {
            out.push(format!("segment {} mixes Side1 and Side2 circuits: {:?}", seg.id, seg.circuits.iter().map(|c| c.circuit).collect::<Vec<_>>()));
        }
    }
    out
}

/// A circuit's side, from the catalogue's own bus assignment (used only by
/// [`segregation_violations`]'s self-check; the network itself never needs
/// to look this back up once built).
fn circuit_side(circuit: &str) -> Side {
    catalogue().into_iter().find(|e| e.circuit == circuit).map(|e| side_of_bus(e.bus)).unwrap_or(Side::Ess)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn side_of_bus_splits_the_two_normal_generation_channels_and_keeps_ess_apart() {
        assert_eq!(side_of_bus("AC1"), Side::Side1);
        assert_eq!(side_of_bus("AC3"), Side::Side2);
        assert_eq!(side_of_bus("AC_ESS"), Side::Ess);
        assert_eq!(side_of_bus("APU_GEN"), Side::Apu);
        assert_eq!(side_of_bus("nonsense"), Side::Ess, "unknown bus defaults to the conservative Ess assumption");
    }

    #[test]
    fn the_generic_network_builds_with_no_segregation_violation() {
        let net = build_generic_a380_network();
        let violations = segregation_violations(&net);
        assert!(violations.is_empty(), "{violations:?}");
    }

    #[test]
    fn gen_1_and_gen_3_cross_wing_root_in_different_segments() {
        let net = build_generic_a380_network();
        let r1 = net.route_of("gen-1");
        let r3 = net.route_of("gen-3");
        let wr1 = r1.iter().find(|s| s.contains("wing_root")).copied();
        let wr3 = r3.iter().find(|s| s.contains("wing_root")).copied();
        assert!(wr1.is_some() && wr3.is_some());
        assert_ne!(wr1, wr3, "side1 and side2 generator feeders must not share a wing-root segment");
    }

    #[test]
    fn fire_loop_a_and_b_share_a_zone_but_never_a_segment() {
        let net = build_generic_a380_network();
        let a = net.route_of("fire-loop-eng-1-a");
        let b = net.route_of("fire-loop-eng-1-b");
        for (sa, sb) in a.iter().zip(b.iter()) {
            assert_ne!(sa, sb, "loop A and loop B must never land on the same physical bundle");
        }
        // But both really do cross Engine(1): burning the whole zone (not
        // just one segment) must be able to reach both -- proven in
        // faults.rs's own zone-overheat test.
        assert!(net.segments_for_circuit("fire-loop-eng-1-a").iter().any(|s| s.zone == Zone::Engine(1)));
        assert!(net.segments_for_circuit("fire-loop-eng-1-b").iter().any(|s| s.zone == Zone::Engine(1)));
    }

    #[test]
    fn every_catalogued_circuit_gets_a_nonempty_route() {
        let net = build_generic_a380_network();
        for e in catalogue() {
            assert!(!net.route_of(e.circuit).is_empty(), "{} has no route", e.circuit);
        }
    }
}
