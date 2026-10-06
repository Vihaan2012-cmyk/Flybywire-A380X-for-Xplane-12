use super::faults::{combine_pass_fraction, EndSystemFaults, LinkFaults, ModuleFaults, SwitchFaults};
use super::topology::{EndSystemIdx, LINK_RATE_BPS, NetworkSide, NetworkTopology, NodeId, SwitchIdx};
use std::cell::RefCell;
use std::collections::{HashMap, VecDeque};

#[derive(Default)]
pub struct NetworkFaults {
    pub switches: [HashMap<SwitchIdx, SwitchFaults>; 2],
    pub segments: [HashMap<(NodeId, NodeId), LinkFaults>; 2],
    pub end_systems: HashMap<EndSystemIdx, EndSystemFaults>,
    pub modules: HashMap<EndSystemIdx, ModuleFaults>,
}
impl NetworkFaults {
    fn edge_key(a: NodeId, b: NodeId) -> (NodeId, NodeId) {
        if a <= b { (a, b) } else { (b, a) }
    }

    pub fn switch(&self, side: NetworkSide, idx: SwitchIdx) -> SwitchFaults {
        self.switches[side.index()].get(&idx).cloned().unwrap_or_default()
    }

    pub fn segment(&self, side: NetworkSide, a: NodeId, b: NodeId) -> LinkFaults {
        self.segments[side.index()].get(&Self::edge_key(a, b)).copied().unwrap_or_default()
    }

    pub fn set_segment(&mut self, side: NetworkSide, a: NodeId, b: NodeId, f: LinkFaults) {
        self.segments[side.index()].insert(Self::edge_key(a, b), f);
    }

    pub fn end_system(&self, idx: EndSystemIdx) -> EndSystemFaults {
        self.end_systems.get(&idx).copied().unwrap_or_default()
    }

    pub fn module(&self, idx: EndSystemIdx) -> ModuleFaults {
        self.modules.get(&idx).cloned().unwrap_or_else(|| ModuleFaults { powered: true, ..Default::default() })
    }
}

#[derive(Clone, Copy, Debug)]
pub struct PortLoad {
    pub offered_bps: f64,
    pub capacity_bps: f64,
}
impl PortLoad {
    pub fn oversubscribed(&self) -> bool {
        self.offered_bps > self.capacity_bps
    }

    pub fn overflow_loss_fraction(&self) -> f64 {
        if self.offered_bps <= self.capacity_bps || self.offered_bps <= 0.0 {
            0.0
        } else {
            ((self.offered_bps - self.capacity_bps) / self.offered_bps).clamp(0.0, 1.0)
        }
    }
}

pub struct NetworkGraph<'t> {
    topology: &'t NetworkTopology,
    adjacency: [HashMap<NodeId, Vec<NodeId>>; 2],
    memo: Option<Memo>,
}

#[derive(Default)]
struct Memo {
    paths: RefCell<HashMap<(usize, NodeId, NodeId), Option<Vec<NodeId>>>>,
    loads: RefCell<HashMap<(usize, NodeId, NodeId), PortLoad>>,
}

impl<'t> NetworkGraph<'t> {
    pub fn memoized(topology: &'t NetworkTopology) -> Self {
        Self { memo: Some(Memo::default()), ..Self::new(topology) }
    }

    pub fn new(topology: &'t NetworkTopology) -> Self {
        let adjacency = NetworkSide::BOTH.map(|side| {
            let mut adj: HashMap<NodeId, Vec<NodeId>> = HashMap::new();
            for (a, b) in topology.edges(side) {
                adj.entry(a).or_default().push(b);
                adj.entry(b).or_default().push(a);
            }
            adj
        });
        Self { topology, adjacency, memo: None }
    }

    fn adjacency(&self, side: NetworkSide) -> &HashMap<NodeId, Vec<NodeId>> {
        &self.adjacency[side.index()]
    }

    fn node_up(&self, side: NetworkSide, node: NodeId, faults: &NetworkFaults) -> bool {
        match node {
            NodeId::Switch(i) => faults.switch(side, i).is_available(),
            NodeId::End(_) => true,
        }
    }

    pub fn edge_pass_fraction(&self, side: NetworkSide, a: NodeId, b: NodeId, faults: &NetworkFaults) -> f64 {
        let seg = faults.segment(side, a, b);
        let port_a = if let NodeId::Switch(i) = a { faults.switch(side, i).port_towards(b) } else { 0.0 };
        let port_b = if let NodeId::Switch(i) = b { faults.switch(side, i).port_towards(a) } else { 0.0 };
        combine_pass_fraction(&[seg.open, port_a, port_b])
    }

    fn edge_usable(&self, side: NetworkSide, a: NodeId, b: NodeId, faults: &NetworkFaults) -> bool {
        faults.segment(side, a, b).is_available() && self.edge_pass_fraction(side, a, b, faults) > 0.0
    }

    pub fn reachable_ends(&self, side: NetworkSide, from: NodeId, faults: &NetworkFaults) -> Vec<bool> {
        let mut ends = vec![false; self.topology.end_systems.len()];
        if !self.node_up(side, from, faults) {
            return ends;
        }
        let adj = self.adjacency(side);
        let mut visited: HashMap<NodeId, bool> = HashMap::new();
        let mut frontier = VecDeque::new();
        frontier.push_back(from);
        visited.insert(from, true);
        while let Some(node) = frontier.pop_front() {
            if let NodeId::End(i) = node {
                if let Some(slot) = ends.get_mut(i) {
                    *slot = true;
                }
            }
            for &next in adj.get(&node).into_iter().flatten() {
                if visited.contains_key(&next) {
                    continue;
                }
                if !self.node_up(side, next, faults) || !self.edge_usable(side, node, next, faults) {
                    continue;
                }
                visited.insert(next, true);
                frontier.push_back(next);
            }
        }
        ends
    }

    pub fn reachable(&self, side: NetworkSide, from: NodeId, to: NodeId, faults: &NetworkFaults) -> bool {
        self.shortest_path(side, from, to, faults).is_some()
    }

    pub fn shortest_path(&self, side: NetworkSide, from: NodeId, to: NodeId, faults: &NetworkFaults) -> Option<Vec<NodeId>> {
        let Some(memo) = &self.memo else { return self.search(side, from, to, faults) };
        let key = (side.index(), from, to);
        if let Some(path) = memo.paths.borrow().get(&key) {
            return path.clone();
        }
        let path = self.search(side, from, to, faults);
        memo.paths.borrow_mut().insert(key, path.clone());
        path
    }

    fn search(&self, side: NetworkSide, from: NodeId, to: NodeId, faults: &NetworkFaults) -> Option<Vec<NodeId>> {
        if !self.node_up(side, from, faults) || !self.node_up(side, to, faults) {
            return None;
        }
        if from == to {
            return Some(vec![from]);
        }
        let adj = self.adjacency(side);
        let mut came_from: HashMap<NodeId, NodeId> = HashMap::new();
        let mut visited: HashMap<NodeId, bool> = HashMap::new();
        let mut frontier = VecDeque::new();
        frontier.push_back(from);
        visited.insert(from, true);
        while let Some(node) = frontier.pop_front() {
            if node == to {
                let mut path = vec![to];
                let mut cur = to;
                while let Some(&prev) = came_from.get(&cur) {
                    path.push(prev);
                    cur = prev;
                }
                path.reverse();
                return Some(path);
            }
            for &next in adj.get(&node).into_iter().flatten() {
                if visited.contains_key(&next) {
                    continue;
                }
                if !self.node_up(side, next, faults) || !self.edge_usable(side, node, next, faults) {
                    continue;
                }
                visited.insert(next, true);
                came_from.insert(next, node);
                frontier.push_back(next);
            }
        }
        None
    }

    pub fn path_pass_fraction(&self, side: NetworkSide, path: &[NodeId], faults: &NetworkFaults) -> f64 {
        if path.len() < 2 {
            return 1.0;
        }
        let mut pass = 1.0;
        for w in path.windows(2) {
            pass *= self.edge_pass_fraction(side, w[0], w[1], faults);
        }
        for &node in &path[1..path.len() - 1] {
            if let NodeId::Switch(i) = node {
                pass *= 1.0 - faults.switch(side, i).failure.clamp(0.0, 1.0);
            }
        }
        pass
    }

    pub fn hop_count(path: &[NodeId]) -> usize {
        path.len().saturating_sub(1)
    }

    pub fn port_load(&self, side: NetworkSide, a: NodeId, b: NodeId, faults: &NetworkFaults) -> PortLoad {
        let Some(memo) = &self.memo else { return self.load(side, a, b, faults) };
        let key = (side.index(), a, b);
        if let Some(&load) = memo.loads.borrow().get(&key) {
            return load;
        }
        let load = self.load(side, a, b, faults);
        memo.loads.borrow_mut().insert(key, load);
        load
    }

    fn load(&self, side: NetworkSide, a: NodeId, b: NodeId, faults: &NetworkFaults) -> PortLoad {
        let mut offered = 0.0;
        for vl in &self.topology.virtual_links {
            let source = NodeId::End(vl.source);
            let crosses = vl.destinations.iter().any(|&d| {
                self.shortest_path(side, source, NodeId::End(d), faults)
                    .is_some_and(|p| p.windows(2).any(|w| (w[0] == a && w[1] == b) || (w[0] == b && w[1] == a)))
            });
            if crosses {
                offered += vl.allocated_bps();
            }
        }
        for node in [a, b] {
            if let NodeId::End(i) = node {
                let babble = faults.end_system(i).babbling.clamp(0.0, 1.0);
                if babble > 0.0 {
                    let regulated: f64 = self.topology.virtual_links.iter().filter(|vl| vl.source == i).map(|vl| vl.allocated_bps()).sum();
                    offered += (LINK_RATE_BPS - regulated).max(0.0) * babble;
                }
            }
        }
        PortLoad { offered_bps: offered, capacity_bps: LINK_RATE_BPS }
    }
}

#[cfg(test)]
mod memo_tests {
    use super::super::topology::a380_reference_topology;
    use super::*;

    #[test]
    fn a_memoized_graph_answers_exactly_as_a_plain_one() {
        let t = a380_reference_topology();
        let plain = NetworkGraph::new(&t);
        let memo = NetworkGraph::memoized(&t);
        let faults = NetworkFaults::default();
        let n = t.end_systems.len();
        for side in NetworkSide::BOTH {
            for a in 0..n {
                let ends = plain.reachable_ends(side, NodeId::End(a), &faults);
                for b in 0..n {
                    let from = NodeId::End(a);
                    let to = NodeId::End(b);
                    let path = plain.shortest_path(side, from, to, &faults);
                    assert_eq!(memo.shortest_path(side, from, to, &faults), path);
                    assert_eq!(memo.shortest_path(side, from, to, &faults), path, "and again, from the memo");
                    assert_eq!(ends[b], path.is_some(), "reachable_ends agrees with a search per pair");
                    if let Some(p) = path {
                        for w in p.windows(2) {
                            let x = plain.port_load(side, w[0], w[1], &faults);
                            let y = memo.port_load(side, w[0], w[1], &faults);
                            assert_eq!((x.offered_bps, x.capacity_bps), (y.offered_bps, y.capacity_bps));
                        }
                    }
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use super::super::faults::PartitionFaults;
    use super::super::topology::a380_reference_topology;

    #[test]
    fn healthy_network_reaches_every_end_system_from_every_other() {
        let t = a380_reference_topology();
        let g = NetworkGraph::new(&t);
        let faults = NetworkFaults::default();
        for side in NetworkSide::BOTH {
            for i in 0..t.end_systems.len() {
                for j in 0..t.end_systems.len() {
                    if i != j {
                        assert!(g.reachable(side, NodeId::End(i), NodeId::End(j), &faults));
                    }
                }
            }
        }
    }

    #[test]
    fn a_fully_failed_switch_removes_it_from_reachability() {
        let t = a380_reference_topology();
        let g = NetworkGraph::new(&t);
        let mut faults = NetworkFaults::default();
        let cpiom_a1_switch = t.end_systems[1].attach[NetworkSide::A.index()];
        faults.switches[0].insert(cpiom_a1_switch, SwitchFaults { failure: 1.0, ..Default::default() });
        assert!(!g.reachable(NetworkSide::A, NodeId::End(1), NodeId::End(0), &faults));
        assert!(g.reachable(NetworkSide::B, NodeId::End(1), NodeId::End(0), &faults));
    }

    #[test]
    fn a_severed_segment_blocks_only_that_edge() {
        let t = a380_reference_topology();
        let g = NetworkGraph::new(&t);
        let mut faults = NetworkFaults::default();
        faults.set_segment(NetworkSide::A, NodeId::Switch(0), NodeId::Switch(1), LinkFaults { open: 1.0 });
        assert!(g.reachable(NetworkSide::A, NodeId::Switch(0), NodeId::Switch(1), &faults));
    }

    #[test]
    fn cutting_every_alternate_route_isolates_the_pair() {
        let t = a380_reference_topology();
        let g = NetworkGraph::new(&t);
        let mut faults = NetworkFaults::default();
        for n in [1usize, 2, 7] {
            faults.set_segment(NetworkSide::A, NodeId::Switch(0), NodeId::Switch(n), LinkFaults { open: 1.0 });
        }
        assert!(!g.reachable(NetworkSide::A, NodeId::Switch(0), NodeId::Switch(3), &faults));
        assert!(g.reachable(NetworkSide::A, NodeId::Switch(0), NodeId::Switch(0), &faults));
    }

    #[test]
    fn partial_switch_failure_costs_pass_fraction_without_losing_reachability() {
        let t = a380_reference_topology();
        let g = NetworkGraph::new(&t);
        let mut faults = NetworkFaults::default();
        let path = g.shortest_path(NetworkSide::A, NodeId::End(1), NodeId::End(0), &faults).unwrap();
        let healthy_pass = g.path_pass_fraction(NetworkSide::A, &path, &faults);
        assert_eq!(healthy_pass, 1.0);
        if let Some(&NodeId::Switch(mid)) = path.get(1) {
            faults.switches[0].insert(mid, SwitchFaults { failure: 0.5, ..Default::default() });
        }
        assert!(g.reachable(NetworkSide::A, NodeId::End(1), NodeId::End(0), &faults));
        let degraded_pass = g.path_pass_fraction(NetworkSide::A, &path, &faults);
        assert!(degraded_pass < healthy_pass);
    }

    #[test]
    fn a_babbling_end_system_oversubscribes_its_own_port() {
        let t = a380_reference_topology();
        let g = NetworkGraph::new(&t);
        let mut faults = NetworkFaults::default();
        let es = 0usize;
        let switch = NodeId::Switch(t.end_systems[es].attach[0]);
        let node = NodeId::End(es);
        let quiet = g.port_load(NetworkSide::A, node, switch, &faults);
        assert!(!quiet.oversubscribed());
        faults.end_systems.insert(es, EndSystemFaults { babbling: 1.0 });
        let flooded = g.port_load(NetworkSide::A, node, switch, &faults);
        assert!(flooded.oversubscribed());
        assert!(flooded.overflow_loss_fraction() > 0.0);
    }

    #[test]
    fn module_faults_default_to_available_when_absent() {
        let faults = NetworkFaults::default();
        assert!(faults.module(0).is_available());
        let _ = PartitionFaults::default();
    }
}
