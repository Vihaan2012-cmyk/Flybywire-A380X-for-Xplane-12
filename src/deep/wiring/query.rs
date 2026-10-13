//! Queries: "what circuits pass through zone Z", and "what fails if bundle
//! B burns" (a specific physical loom, [`bundle::Segment`], not a whole
//! zone -- see `bundle.rs`'s own doc for why a segment is the "bundle" a
//! chafe or localized burn actually damages, as distinct from
//! `faults::zone_overheat_effects`'s zone-wide fire).

use super::bundle::WireBundleNetwork;
use super::faults::CircuitEffect;
use super::zones::Zone;

/// Every circuit with at least one segment routed through `zone`.
pub fn circuits_through_zone(net: &WireBundleNetwork, zone: Zone) -> Vec<&'static str> {
    net.circuits_in_zone(zone)
}

/// "What fails if bundle `segment_id` burns": every circuit sharing that
/// physical loom loses its conductor entirely (a burned-through bundle is
/// destroyed, not left at some intermediate resistance -- the zone-wide
/// overheat model's own continuous ramp is for a *developing* fire;
/// this query answers the end state maintenance actually cares about, "if
/// this loom is gone, what goes with it").
pub fn bundle_burn_effects(net: &WireBundleNetwork, segment_id: &str) -> Vec<(&'static str, CircuitEffect)> {
    net.circuits_in_segment(segment_id).into_iter().map(|c| (c, CircuitEffect::Open)).collect()
}

/// Every *other* circuit sharing at least one segment with `circuit` --
/// its bundle-mates, the set a chafe or short at any of their shared
/// segments could crosstalk into.
pub fn bundle_mates(net: &WireBundleNetwork, circuit: &str) -> Vec<&'static str> {
    let mut out: Vec<&'static str> = net.segments_for_circuit(circuit).iter().flat_map(|s| s.circuits.iter().map(|c| c.circuit)).filter(|&c| c != circuit).collect();
    out.sort_unstable();
    out.dedup();
    out
}

#[cfg(test)]
mod tests {
    use super::super::routing::build_generic_a380_network;
    use super::*;

    #[test]
    fn circuits_through_the_main_avionics_zone_include_the_trus() {
        let net = build_generic_a380_network();
        let here = circuits_through_zone(&net, Zone::MainAvionics);
        assert!(here.contains(&"tr-1"));
        assert!(here.contains(&"tr-ess"));
    }

    #[test]
    fn burning_one_fire_loops_own_segment_only_opens_that_loop_not_its_partner() {
        let net = build_generic_a380_network();
        let seg_a = net.route_of("fire-loop-eng-1-a")[net.route_of("fire-loop-eng-1-a").len() - 1];
        let effects = bundle_burn_effects(&net, seg_a);
        let ids: Vec<&str> = effects.iter().map(|(id, _)| *id).collect();
        assert!(ids.contains(&"fire-loop-eng-1-a"));
        assert!(!ids.contains(&"fire-loop-eng-1-b"), "loop B is on a different physical bundle");
        assert!(effects.iter().all(|(_, e)| *e == CircuitEffect::Open));
    }

    #[test]
    fn an_unknown_segment_burns_nothing() {
        let net = build_generic_a380_network();
        assert!(bundle_burn_effects(&net, "not-a-real-segment").is_empty());
    }

    #[test]
    fn bundle_mates_excludes_the_circuit_itself() {
        let net = build_generic_a380_network();
        let mates = bundle_mates(&net, "gen-1");
        assert!(!mates.contains(&"gen-1"));
    }
}
