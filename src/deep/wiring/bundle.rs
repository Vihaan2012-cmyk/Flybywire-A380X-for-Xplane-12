//! Wire bundle model: segments routed through airframe zones, each carrying
//! a set of circuits (by breaker/load id), with wire gauge, length,
//! resistance per metre vs temperature (`gauge::Awg`) and insulation
//! temperature rating (`gauge::Insulation`).
//!
//! A [`Segment`] is one physical loom run confined to a single [`Zone`]: a
//! tie-wrapped bundle of individual conductors that share a routing path
//! for that stretch (the thing that "burns" as one event, or that a single
//! chafe point damages). A [`Route`] is one circuit's ordered path across
//! one or more segments, source (bus/panel) to consumer -- `routing.rs`
//! builds the A380 catalogue of these; this module only holds the generic
//! data structure and the resistance/query arithmetic over it.
//!
//! Deliberately **not** one segment per zone: two circuits that must stay
//! segregated (redundant side 1/side 2, or a fire loop's own A/B pair) can
//! occupy the *same* zone in two *different* segments, so a fault that hits
//! one segment (a chafe point, a localized burn) does not automatically
//! reach the other -- only a zone-wide fault (`faults::zone_overheat_effects`)
//! reaches every segment in a zone, matching the backlog's own distinction
//! between a chafe (segment/circuit-specific) and a bundle fire (zone-wide).

use super::gauge::WireSpec;
use super::zones::Zone;
use std::collections::HashMap;

/// One circuit's own wire within a segment -- a segment bundles several
/// circuits together, and real looms mix gauges (a heavy feeder next to
/// several signal wires), so each circuit keeps its own [`WireSpec`] rather
/// than the segment sharing one.
#[derive(Clone, Debug, PartialEq)]
pub struct CircuitWire {
    pub circuit: &'static str,
    pub wire: WireSpec,
}

/// One physical loom run, confined to one zone.
#[derive(Clone, Debug, PartialEq)]
pub struct Segment {
    pub id: &'static str,
    pub zone: Zone,
    pub length_m: f64,
    pub circuits: Vec<CircuitWire>,
}
impl Segment {
    pub fn wire_of(&self, circuit: &str) -> Option<&WireSpec> {
        self.circuits.iter().find(|c| c.circuit == circuit).map(|c| &c.wire)
    }
}

/// The whole wire-bundle network: every segment, plus every circuit's
/// ordered route across them.
#[derive(Default)]
pub struct WireBundleNetwork {
    segments: Vec<Segment>,
    by_id: HashMap<&'static str, usize>,
    /// circuit id -> ordered segment ids, source to consumer.
    routes: HashMap<&'static str, Vec<&'static str>>,
}

impl WireBundleNetwork {
    pub fn new() -> Self {
        Self::default()
    }

    /// Add one segment. Panics on a duplicate id (a build-time programming
    /// error in the routing catalogue, not a runtime condition).
    pub fn add_segment(&mut self, seg: Segment) {
        assert!(!self.by_id.contains_key(seg.id), "duplicate segment id {}", seg.id);
        self.by_id.insert(seg.id, self.segments.len());
        self.segments.push(seg);
    }

    /// Record one circuit's ordered path across already-added segments.
    /// Every listed segment must exist and must actually carry that
    /// circuit's wire, and the route replaces this circuit's previous
    /// segment_ids for readability.
    pub fn set_route(&mut self, circuit: &'static str, segment_ids: Vec<&'static str>) {
        for &sid in &segment_ids {
            let seg = self.segment(sid).unwrap_or_else(|| panic!("route for {circuit} names unknown segment {sid}"));
            assert!(seg.wire_of(circuit).is_some(), "route for {circuit} names segment {sid} that does not carry its wire");
        }
        self.routes.insert(circuit, segment_ids);
    }

    pub fn segment(&self, id: &str) -> Option<&Segment> {
        self.by_id.get(id).map(|&i| &self.segments[i])
    }

    pub fn segments(&self) -> &[Segment] {
        &self.segments
    }

    pub fn route_of(&self, circuit: &str) -> &[&'static str] {
        self.routes.get(circuit).map(Vec::as_slice).unwrap_or(&[])
    }

    /// Every segment a circuit's route passes through, in order.
    pub fn segments_for_circuit(&self, circuit: &str) -> Vec<&Segment> {
        self.route_of(circuit).iter().filter_map(|&sid| self.segment(sid)).collect()
    }

    /// A circuit's total series wiring resistance along its whole route:
    /// each segment's `length_m * resistance_per_m(ambient of that
    /// segment's zone)`, summed. `ambient_c` is supplied by the caller (the
    /// thermal-zone model, or a fixed value in a test) keyed by [`Zone`] --
    /// this module never assumes a temperature of its own, the plain-data
    /// interface the brief asks for.
    pub fn circuit_resistance_ohm(&self, circuit: &str, ambient_c: &dyn Fn(Zone) -> f64) -> f64 {
        self.segments_for_circuit(circuit)
            .iter()
            .filter_map(|seg| seg.wire_of(circuit).map(|w| (seg, w)))
            .map(|(seg, w)| seg.length_m * w.awg.resistance_per_m(ambient_c(seg.zone)))
            .sum()
    }

    /// Every distinct circuit id with at least one segment in `zone`
    /// (query 1: "what circuits pass through zone Z").
    pub fn circuits_in_zone(&self, zone: Zone) -> Vec<&'static str> {
        let mut out: Vec<&'static str> = self.segments.iter().filter(|s| s.zone == zone).flat_map(|s| s.circuits.iter().map(|c| c.circuit)).collect();
        out.sort_unstable();
        out.dedup();
        out
    }

    pub fn segments_in_zone(&self, zone: Zone) -> Vec<&Segment> {
        self.segments.iter().filter(|s| s.zone == zone).collect()
    }

    /// Every circuit whose route includes `segment_id` -- the members of
    /// one physical bundle, for "what fails if bundle B burns".
    pub fn circuits_in_segment(&self, segment_id: &str) -> Vec<&'static str> {
        self.segment(segment_id).map(|s| s.circuits.iter().map(|c| c.circuit).collect()).unwrap_or_default()
    }
}

#[cfg(test)]
mod tests {
    use super::super::gauge::{Awg, Insulation};
    use super::*;

    fn wire(circuit: &'static str) -> CircuitWire {
        CircuitWire { circuit, wire: WireSpec { awg: Awg::Size(20), insulation: Insulation::Etfe150 } }
    }

    #[test]
    fn resistance_is_length_times_gauge_summed_across_the_route() {
        let mut net = WireBundleNetwork::new();
        net.add_segment(Segment { id: "s1", zone: Zone::MainAvionics, length_m: 5.0, circuits: vec![wire("c1")] });
        net.add_segment(Segment { id: "s2", zone: Zone::WingRoot, length_m: 10.0, circuits: vec![wire("c1")] });
        net.set_route("c1", vec!["s1", "s2"]);
        let r20 = Awg::Size(20).resistance_per_m_at_20c();
        let got = net.circuit_resistance_ohm("c1", &|_| 20.0);
        assert!((got - r20 * 15.0).abs() < 1e-9, "expected {} got {}", r20 * 15.0, got);
    }

    #[test]
    fn hotter_zone_on_the_route_raises_resistance() {
        let mut net = WireBundleNetwork::new();
        net.add_segment(Segment { id: "s1", zone: Zone::Engine(1), length_m: 8.0, circuits: vec![wire("c1")] });
        net.set_route("c1", vec!["s1"]);
        let cold = net.circuit_resistance_ohm("c1", &|_| 20.0);
        let hot = net.circuit_resistance_ohm("c1", &|_| 200.0);
        assert!(hot > cold);
    }

    #[test]
    fn circuits_in_zone_lists_every_circuit_with_a_segment_there_deduplicated() {
        let mut net = WireBundleNetwork::new();
        net.add_segment(Segment { id: "s1", zone: Zone::CargoFwd, length_m: 3.0, circuits: vec![wire("a"), wire("b")] });
        net.add_segment(Segment { id: "s2", zone: Zone::CargoFwd, length_m: 2.0, circuits: vec![wire("a")] });
        net.add_segment(Segment { id: "s3", zone: Zone::CargoAft, length_m: 2.0, circuits: vec![wire("c")] });
        let mut here = net.circuits_in_zone(Zone::CargoFwd);
        here.sort_unstable();
        assert_eq!(here, vec!["a", "b"]);
        assert_eq!(net.circuits_in_zone(Zone::CargoAft), vec!["c"]);
        assert!(net.circuits_in_zone(Zone::Cockpit).is_empty());
    }

    #[test]
    fn a_segment_confines_a_fault_to_its_own_members_not_the_whole_zone() {
        let mut net = WireBundleNetwork::new();
        net.add_segment(Segment { id: "loop-a", zone: Zone::Engine(1), length_m: 4.0, circuits: vec![wire("fire-a")] });
        net.add_segment(Segment { id: "loop-b", zone: Zone::Engine(1), length_m: 4.0, circuits: vec![wire("fire-b")] });
        assert_eq!(net.circuits_in_segment("loop-a"), vec!["fire-a"]);
        assert_eq!(net.circuits_in_segment("loop-b"), vec!["fire-b"]);
        // Both share a zone, but a segment-scoped query keeps them apart.
        let mut zoned = net.circuits_in_zone(Zone::Engine(1));
        zoned.sort_unstable();
        assert_eq!(zoned, vec!["fire-a", "fire-b"]);
    }

    #[test]
    #[should_panic(expected = "unknown segment")]
    fn a_route_naming_an_unregistered_segment_panics_at_build_time() {
        let mut net = WireBundleNetwork::new();
        net.add_segment(Segment { id: "s1", zone: Zone::MainAvionics, length_m: 1.0, circuits: vec![wire("c1")] });
        net.set_route("c1", vec!["s1", "does-not-exist"]);
    }
}
