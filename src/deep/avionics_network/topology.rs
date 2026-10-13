//! Static description of an AFDX (ARINC 664 Part 7) network: which end
//! systems, switches and virtual links exist and how they are wired.
//! Nothing here carries state — faults live in `faults`, traffic and
//! delivery in `graph`/`message`.
//!
//! The A380 has two physically separate but topologically congruent
//! networks, A and B (ARINC 664 Part 7's dual-redundant AFDX network
//! model): every end system is dual-attached, one NIC per network, and
//! every virtual link exists, independently, on both. Losing all of one
//! network degrades a function to single-network operation; it does not by
//! itself lose the function (see `consequences`).
//!
//! The reference topology built by [`a380_reference_topology`] mirrors
//! FlyByWire's own public network description in
//! `a380_systems/src/avionics_data_communication_network.rs` (switch
//! adjacency for network A, lines 145-154 of that file as read for this
//! model; network B is the same graph, mirrored onto its own switches).
//! CPIOM/IOM names, types and their switch attachment follow the same
//! file's `cpio_modules`/`io_modules` tables (lines 227-291). Virtual link
//! ids, names, BAG and frame sizes are not public for the real aircraft
//! and are GENERIC, chosen within ARINC 664 Part 7's legal ranges.

use std::collections::HashMap;

/// Index into a per-network `Vec<SwitchSpec>`.
pub type SwitchIdx = usize;
/// Index into `NetworkTopology::end_systems`.
pub type EndSystemIdx = usize;
/// Index into `NetworkTopology::virtual_links`.
pub type VirtualLinkIdx = usize;

/// Ethernet full-duplex line rate of an AFDX end system/switch port.
/// 100BASE-TX (100 Mbit/s) is the rate general AFDX descriptions (e.g.
/// ARINC 664 Part 7 tutorials from Condor Engineering/Aeroflex, and
/// Airbus's own public AFDX overviews) use throughout; the real A380's
/// exact port speed per link is not public. GENERIC.
pub const LINK_RATE_BPS: f64 = 100_000_000.0;

/// Ethernet frame size bounds AFDX inherits unchanged: 64 bytes minimum
/// (preamble/IFG excluded), 1518 bytes maximum (IEEE 802.3, public).
pub const MIN_FRAME_BYTES: u32 = 64;
pub const MAX_FRAME_BYTES: u32 = 1518;

/// Which of the two redundant networks.
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

/// A node in one network's graph: either a switch or an end system's NIC
/// on that network.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum NodeId {
    Switch(SwitchIdx),
    End(EndSystemIdx),
}

/// Airbus's own lettering for what a core processing I/O module does
/// (public via A380 ATA 42/type-training overviews of the AFDX/CPIOM
/// architecture): flight controls (A), some cabin/utilities, fuel, etc.
/// This is a label only — it does not change the physics — used so
/// `consequences` and `FAILURES.md` can say which class of module a fault
/// hit.
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

/// One network-attached module: a CPIOM or IOM, dual-attached to both AFDX
/// networks the way a real A380 CPIOM/IOM is (its own two independent
/// network interface cards, one per network, so one network's total loss
/// costs it half its bandwidth and redundancy, not the module).
#[derive(Clone, Debug)]
pub struct EndSystemSpec {
    pub name: &'static str,
    pub kind: ModuleKind,
    /// Bay this module is mounted in, for `ventilation`'s thermal grouping.
    pub bay: &'static str,
    /// Which switch this module's network A / network B NIC lands on
    /// (indexed by `NetworkSide::index()`).
    pub attach: [SwitchIdx; 2],
    /// ARINC 653 partitions (applications) hosted on this module. A pure
    /// IOM with no application layer still gets one driver partition.
    pub partitions: Vec<&'static str>,
}

#[derive(Clone, Debug)]
pub struct SwitchSpec {
    pub name: String,
    /// Other switches on the *same* network this one is cabled to
    /// directly. Port numbers for `SwitchFaults::port_failure` are derived
    /// (see `NetworkTopology::switch_ports`) as this list's order followed
    /// by the end systems attached here, in `end_systems` order — so it
    /// only has to be stable within one `NetworkTopology`.
    pub switch_neighbours: Vec<SwitchIdx>,
}

/// One ARINC 664 Part 7 virtual link: a fixed, one-to-many virtual
/// point-to-multipoint channel with a policed bandwidth. The *id* is
/// shared by both networks — it is the same logical channel, sent
/// independently over A and B; that duplication is exactly the redundancy
/// "dual redundant AFDX networks" refers to, not two different VLs.
#[derive(Clone, Debug)]
pub struct VirtualLinkSpec {
    pub id: u16,
    pub name: &'static str,
    pub source: EndSystemIdx,
    pub destinations: Vec<EndSystemIdx>,
    /// Bandwidth Allocation Gap: minimum time between two frames of this
    /// VL, ms. ARINC 664 Part 7 restricts BAG to a power of two from 1 to
    /// 128 ms (so every end system's and switch's traffic shapers/policers
    /// stay on a common clock); enforced in `new`.
    pub bag_ms: f64,
    /// Max frame size on the wire, bytes (Ethernet header + payload + FCS).
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

    /// Bandwidth this VL is allocated on the wire, bit/s: one max-size
    /// frame every BAG, the definition of AFDX bandwidth allocation.
    pub fn allocated_bps(&self) -> f64 {
        self.max_frame_bytes as f64 * 8.0 / (self.bag_ms / 1000.0)
    }

    /// Store-and-forward transmission time for one max-size frame at the
    /// link rate, s.
    pub fn frame_time_s(&self) -> f64 {
        self.max_frame_bytes as f64 * 8.0 / LINK_RATE_BPS
    }
}

pub struct NetworkTopology {
    /// Switches, indexed `[NetworkSide::index()][SwitchIdx]`.
    pub switches: [Vec<SwitchSpec>; 2],
    pub end_systems: Vec<EndSystemSpec>,
    pub virtual_links: Vec<VirtualLinkSpec>,
}
impl NetworkTopology {
    /// The ordered list of nodes switch `switch` (on `side`) reaches
    /// directly: its `switch_neighbours`, in order, then every end system
    /// attached to it here, in `end_systems` order. Port `i` for
    /// `SwitchFaults::port_failure` purposes is this list's `i`-th entry.
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

    /// Every undirected edge of one network's graph: switch-switch (each
    /// counted once) and end system-switch.
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

/// FlyByWire's public network-A adjacency
/// (`avionics_data_communication_network.rs` lines 145-154), switch ids
/// 1,2,3,4,5,6,7,9 relabelled 0..7 in that file's own order. Network B
/// mirrors this graph on switches 11,12,13,14,15,16,17,19, relabelled the
/// same way here as 0..7 of the second `Vec`.
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

/// A reference topology built from the sourcing above: 8 switches per
/// network, a handful of CPIOMs/IOMs (a representative subset of FlyByWire's
/// public table, not all 22+8), and a small set of GENERIC virtual links
/// exercising every backlog item (multicast, tight and loose BAG, small
/// and large frames).
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

    // CPIOM/IOM attachment mirrors the FBW file's own switch numbers
    // (e.g. CPIOM C1 -> switch 3/13, `avionics_data_communication_network.rs`
    // line 263), relabelled through the same 1,2,3,4,5,6,7,9 -> 0..7 map.
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
        // FWS ECAM warnings broadcast to the flight guidance and fuel
        // CPIOMs: tight BAG (frequent), small frame.
        vl(1, "FWS_WARNINGS", 0, &[1, 2], 8.0, 256),
        // PRIM flight-control law outputs to the fuel CPIOM (e.g. for CG
        // management) and to IOM-A1: medium BAG, medium frame.
        vl(2, "PRIM_STATUS", 1, &[2, 3], 16.0, 512),
        // Fuel system state, high rate, small frame, multicast to both
        // IOMs and the FWS.
        vl(3, "FUEL_STATE", 2, &[3, 4, 0], 4.0, 128),
        // A large, loosely-timed data set (e.g. a loadable table) from
        // IOM-A5 to CPIOM-C1: loose BAG, near max frame.
        vl(4, "BULK_TABLE", 4, &[0], 128.0, 1400),
    ];

    NetworkTopology { switches: [switches_a, switches_b], end_systems, virtual_links }
}

/// Every module's static class label, keyed by its `end_systems` index —
/// convenience used by `consequences`.
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
        // 256 bytes = 2048 bits every 8 ms = 256 kbit/s.
        let v = VirtualLinkSpec::new(1, "x", 0, vec![1], 8.0, 256);
        assert!((v.allocated_bps() - 256_000.0).abs() < 1.0);
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
        let ports = t.switch_ports(NetworkSide::A, 2); // switch "3"
        // switch 2 (id 3) neighbours [0,3,4,6,7] plus CPIOM-C1 and IOM-A5
        // (both attach[0] == 2).
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
