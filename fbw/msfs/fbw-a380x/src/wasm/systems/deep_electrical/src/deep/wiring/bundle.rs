use super::gauge::WireSpec;
use super::zones::Zone;
use std::collections::HashMap;

#[derive(Clone, Debug, PartialEq)]
pub struct CircuitWire {
    pub circuit: &'static str,
    pub wire: WireSpec,
}

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

#[derive(Default)]
pub struct WireBundleNetwork {
    segments: Vec<Segment>,
    by_id: HashMap<&'static str, usize>,
    routes: HashMap<&'static str, Vec<&'static str>>,
}

impl WireBundleNetwork {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn add_segment(&mut self, seg: Segment) {
        assert!(!self.by_id.contains_key(seg.id), "duplicate segment id {}", seg.id);
        self.by_id.insert(seg.id, self.segments.len());
        self.segments.push(seg);
    }

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

    pub fn segments_for_circuit(&self, circuit: &str) -> Vec<&Segment> {
        self.route_of(circuit).iter().filter_map(|&sid| self.segment(sid)).collect()
    }

    pub fn circuit_resistance_ohm(&self, circuit: &str, ambient_c: &dyn Fn(Zone) -> f64) -> f64 {
        self.segments_for_circuit(circuit)
            .iter()
            .filter_map(|seg| seg.wire_of(circuit).map(|w| (seg, w)))
            .map(|(seg, w)| seg.length_m * w.awg.resistance_per_m(ambient_c(seg.zone)))
            .sum()
    }

    pub fn circuits_in_zone(&self, zone: Zone) -> Vec<&'static str> {
        let mut out: Vec<&'static str> = self.segments.iter().filter(|s| s.zone == zone).flat_map(|s| s.circuits.iter().map(|c| c.circuit)).collect();
        out.sort_unstable();
        out.dedup();
        out
    }

    pub fn segments_in_zone(&self, zone: Zone) -> Vec<&Segment> {
        self.segments.iter().filter(|s| s.zone == zone).collect()
    }

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
