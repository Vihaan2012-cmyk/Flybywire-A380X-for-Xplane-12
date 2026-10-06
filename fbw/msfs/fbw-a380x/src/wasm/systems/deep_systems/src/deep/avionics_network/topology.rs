use std::collections::HashMap;

pub type SwitchIdx = usize;
pub type EndSystemIdx = usize;
pub type VirtualLinkIdx = usize;

pub const LINK_RATE_BPS: f64 = 100_000_000.0;

pub const MIN_FRAME_BYTES: u32 = 64;
pub const MAX_FRAME_BYTES: u32 = 1518;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum NetworkSide {
    A,
    B,
}
impl NetworkSide {
    pub const BOTH: [NetworkSide; 2] = [NetworkSide::A, NetworkSide::B];

    pub fn index(self) -> usize {
        match self {
            NetworkSide::A => 0,
            NetworkSide::B => 1,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum NodeId {
    Switch(SwitchIdx),
    End(EndSystemIdx),
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CpiomType {
    A,
    B,
    C,
    D,
    E,
    F,
    G,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ModuleKind {
    Cpiom(CpiomType),
    Iom,
}

#[derive(Clone, Debug)]
pub struct EndSystemSpec {
    pub name: &'static str,
    pub kind: ModuleKind,
    pub bay: &'static str,
    pub attach: [SwitchIdx; 2],
    pub partitions: Vec<&'static str>,
}

#[derive(Clone, Debug)]
pub struct SwitchSpec {
    pub name: String,
    pub switch_neighbours: Vec<SwitchIdx>,
}

#[derive(Clone, Debug)]
pub struct VirtualLinkSpec {
    pub id: u16,
    pub name: &'static str,
    pub source: EndSystemIdx,
    pub destinations: Vec<EndSystemIdx>,
    pub bag_ms: f64,
    pub max_frame_bytes: u32,
}
impl VirtualLinkSpec {
    pub fn new(
        id: u16,
        name: &'static str,
        source: EndSystemIdx,
        destinations: Vec<EndSystemIdx>,
        bag_ms: f64,
        max_frame_bytes: u32,
    ) -> Self {
        assert!(
            [1.0, 2.0, 4.0, 8.0, 16.0, 32.0, 64.0, 128.0].contains(&bag_ms),
            "ARINC 664 Part 7 BAG must be a power of two ms from 1 to 128, got {bag_ms}"
        );
        assert!(
            (MIN_FRAME_BYTES..=MAX_FRAME_BYTES).contains(&max_frame_bytes),
            "frame size {max_frame_bytes} outside the Ethernet 64..1518 byte range"
        );
        Self { id, name, source, destinations, bag_ms, max_frame_bytes }
    }

    pub fn allocated_bps(&self) -> f64 {
        self.max_frame_bytes as f64 * 8.0 / (self.bag_ms / 1000.0)
    }

    pub fn frame_time_s(&self) -> f64 {
        self.max_frame_bytes as f64 * 8.0 / LINK_RATE_BPS
    }
}

pub struct NetworkTopology {
    pub switches: [Vec<SwitchSpec>; 2],
    pub end_systems: Vec<EndSystemSpec>,
    pub virtual_links: Vec<VirtualLinkSpec>,
}
impl NetworkTopology {
    pub fn switch_ports(&self, side: NetworkSide, switch: SwitchIdx) -> Vec<NodeId> {
        let spec = &self.switches[side.index()][switch];
        let mut ports: Vec<NodeId> = spec.switch_neighbours.iter().map(|&s| NodeId::Switch(s)).collect();
        for (i, es) in self.end_systems.iter().enumerate() {
            if es.attach[side.index()] == switch {
                ports.push(NodeId::End(i));
            }
        }
        ports
    }

    pub fn edges(&self, side: NetworkSide) -> Vec<(NodeId, NodeId)> {
        let mut edges = Vec::new();
        for (s, spec) in self.switches[side.index()].iter().enumerate() {
            for &n in &spec.switch_neighbours {
                if n > s {
                    edges.push((NodeId::Switch(s), NodeId::Switch(n)));
                }
            }
        }
        for (i, es) in self.end_systems.iter().enumerate() {
            edges.push((NodeId::End(i), NodeId::Switch(es.attach[side.index()])));
        }
        edges
    }

    pub fn end_system_index(&self, name: &str) -> Option<EndSystemIdx> {
        self.end_systems.iter().position(|e| e.name == name)
    }
}

pub fn fbw_switch_id(side: NetworkSide, idx: SwitchIdx) -> u8 {
    const NETWORK_A_IDS: [u8; 8] = [1, 2, 3, 4, 5, 6, 7, 9];
    const NETWORK_B_IDS: [u8; 8] = [11, 12, 13, 14, 15, 16, 17, 19];
    match side {
        NetworkSide::A => NETWORK_A_IDS[idx],
        NetworkSide::B => NETWORK_B_IDS[idx],
    }
}

fn fbw_switch_neighbours() -> Vec<Vec<SwitchIdx>> {
    vec![
        vec![1, 2, 7],
        vec![0, 3, 7],
        vec![0, 3, 4, 6, 7],
        vec![1, 2, 5, 6, 7],
        vec![2, 5, 6],
        vec![3, 4, 6],
        vec![2, 3, 4, 5],
        vec![0, 1, 2, 3],
    ]
}

pub fn a380_reference_topology() -> NetworkTopology {
    let neighbours = fbw_switch_neighbours();
    let names = ["1", "2", "3", "4", "5", "6", "7", "9"];
    let make_switches = |prefix: &'static str| -> Vec<SwitchSpec> {
        neighbours
            .iter()
            .enumerate()
            .map(|(i, n)| SwitchSpec {
                name: format!("{prefix}{}", names[i]),
                switch_neighbours: n.clone(),
            })
            .collect()
    };
    let switches_a = make_switches("AFDX-A-");
    let switches_b = make_switches("AFDX-B-");

    let map_switch = |id: u8| -> SwitchIdx {
        match id {
            1..=7 => (id - 1) as usize,
            9 => 7,
            _ => panic!("switch id {id} not in this reference topology"),
        }
    };
    let end_systems = vec![
        EndSystemSpec {
            name: "CPIOM-C1",
            kind: ModuleKind::Cpiom(CpiomType::C),
            bay: "AVIONICS_BAY_FWD",
            attach: [map_switch(3), map_switch(3)],
            partitions: vec!["FWS", "ECAM"],
        },
        EndSystemSpec {
            name: "CPIOM-A1",
            kind: ModuleKind::Cpiom(CpiomType::A),
            bay: "AVIONICS_BAY_FWD",
            attach: [map_switch(7), map_switch(7)],
            partitions: vec!["PRIM", "FG"],
        },
        EndSystemSpec {
            name: "CPIOM-F1",
            kind: ModuleKind::Cpiom(CpiomType::F),
            bay: "AVIONICS_BAY_AFT",
            attach: [map_switch(5), map_switch(5)],
            partitions: vec!["FUEL"],
        },
        EndSystemSpec {
            name: "IOM-A1",
            kind: ModuleKind::Iom,
            bay: "AVIONICS_BAY_FWD",
            attach: [map_switch(1), map_switch(1)],
            partitions: vec!["IO_DRIVER"],
        },
        EndSystemSpec {
            name: "IOM-A5",
            kind: ModuleKind::Iom,
            bay: "AVIONICS_BAY_AFT",
            attach: [map_switch(3), map_switch(3)],
            partitions: vec!["IO_DRIVER"],
        },
    ];

    let vl = |id, name, source, destinations: &[EndSystemIdx], bag_ms, bytes| {
        VirtualLinkSpec::new(id, name, source, destinations.to_vec(), bag_ms, bytes)
    };
    let virtual_links = vec![
        vl(1, "FWS_WARNINGS", 0, &[1, 2], 8.0, 256),
        vl(2, "PRIM_STATUS", 1, &[2, 3], 16.0, 512),
        vl(3, "FUEL_STATE", 2, &[3, 4, 0], 4.0, 128),
        vl(4, "BULK_TABLE", 4, &[0], 128.0, 1400),
    ];

    NetworkTopology { switches: [switches_a, switches_b], end_systems, virtual_links }
}

pub fn module_kind_by_index(topology: &NetworkTopology) -> HashMap<EndSystemIdx, ModuleKind> {
    topology.end_systems.iter().enumerate().map(|(i, e)| (i, e.kind)).collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_power_of_two_bag_is_accepted() {
        let _ = VirtualLinkSpec::new(1, "x", 0, vec![1], 8.0, 256);
    }

    #[test]
    #[should_panic(expected = "power of two")]
    fn a_non_power_of_two_bag_is_rejected() {
        VirtualLinkSpec::new(1, "x", 0, vec![1], 3.0, 256);
    }

    #[test]
    fn allocated_bandwidth_matches_bag_and_frame_size() {
        let v = VirtualLinkSpec::new(1, "x", 0, vec![1], 8.0, 256);
        assert!((v.allocated_bps() - 256_000.0).abs() < 1.0);
    }

    #[test]
    fn fbw_switch_id_matches_avionics_data_communication_network_rs() {
        assert_eq!(fbw_switch_id(NetworkSide::A, 0), 1);
        assert_eq!(fbw_switch_id(NetworkSide::A, 3), 4);
        assert_eq!(fbw_switch_id(NetworkSide::A, 7), 9);
        assert_eq!(fbw_switch_id(NetworkSide::B, 0), 11);
        assert_eq!(fbw_switch_id(NetworkSide::B, 3), 14);
        assert_eq!(fbw_switch_id(NetworkSide::B, 7), 19);
    }

    #[test]
    fn reference_topology_has_symmetric_switch_adjacency() {
        let t = a380_reference_topology();
        for side in NetworkSide::BOTH {
            let switches = &t.switches[side.index()];
            for (i, s) in switches.iter().enumerate() {
                for &n in &s.switch_neighbours {
                    assert!(
                        switches[n].switch_neighbours.contains(&i),
                        "switch {i} lists {n} as a neighbour but not vice versa"
                    );
                }
            }
        }
    }

    #[test]
    fn every_end_system_attaches_to_a_real_switch_on_both_sides() {
        let t = a380_reference_topology();
        for es in &t.end_systems {
            for side in NetworkSide::BOTH {
                assert!(es.attach[side.index()] < t.switches[side.index()].len());
            }
        }
    }

    #[test]
    fn switch_ports_lists_neighbours_then_attached_end_systems() {
        let t = a380_reference_topology();
        let ports = t.switch_ports(NetworkSide::A, 2);
        assert_eq!(ports.len(), 5 + 2);
        assert!(ports.contains(&NodeId::End(0)));
        assert!(ports.contains(&NodeId::End(4)));
    }

    #[test]
    fn edges_cover_every_switch_link_once_and_every_end_system_once() {
        let t = a380_reference_topology();
        let edges = t.edges(NetworkSide::A);
        let switch_switch = edges.iter().filter(|(a, b)| matches!((a, b), (NodeId::Switch(_), NodeId::Switch(_)))).count();
        let total_neighbour_slots: usize = t.switches[0].iter().map(|s| s.switch_neighbours.len()).sum();
        assert_eq!(switch_switch * 2, total_neighbour_slots);
        let end_switch = edges.iter().filter(|(a, _)| matches!(a, NodeId::End(_))).count();
        assert_eq!(end_switch, t.end_systems.len());
    }
}
