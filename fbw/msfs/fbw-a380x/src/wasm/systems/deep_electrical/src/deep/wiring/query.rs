use super::bundle::WireBundleNetwork;
use super::faults::CircuitEffect;
use super::zones::Zone;

pub fn circuits_through_zone(net: &WireBundleNetwork, zone: Zone) -> Vec<&'static str> {
    net.circuits_in_zone(zone)
}

pub fn bundle_burn_effects(net: &WireBundleNetwork, segment_id: &str) -> Vec<(&'static str, CircuitEffect)> {
    net.circuits_in_segment(segment_id).into_iter().map(|c| (c, CircuitEffect::Open)).collect()
}

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
