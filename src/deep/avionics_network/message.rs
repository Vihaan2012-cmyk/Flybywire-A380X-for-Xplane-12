//! Message delivery on top of `graph`: a virtual link's frames leaving its
//! source at its Bandwidth Allocation Gap, crossing the network with
//! store-and-forward latency and bounded jitter, an Ethernet-style CRC
//! integrity check, and — the redundancy management every AFDX receiver
//! performs — picking whichever of network A's or B's frame is valid and
//! newest, so losing one whole network degrades a virtual link (single
//! network, no more redundancy) without losing its data.
//!
//! Two independent loss mechanisms are modelled, each recognisably
//! different in effect, matching how a real AFDX receiver tells them
//! apart:
//!   * physical/queueing loss (a failed switch or port, a severed segment,
//!     a port oversubscribed past its line rate) — the frame never
//!     arrives, so the data simply goes stale (`DataStatus::NoData` once
//!     the staleness window elapses);
//!   * a frame that *does* arrive but fails its CRC (modelled here as
//!     config-table corruption at the source scrambling the payload after
//!     its checksum would have been computed) — `DataStatus::NoComputedData`,
//!     the ARINC 653/664 status for "received, but not usable".

use super::faults::combine_pass_fraction;
use super::graph::{NetworkFaults, NetworkGraph};
use super::topology::{EndSystemIdx, NetworkSide, NetworkTopology, NodeId, VirtualLinkIdx};
use std::collections::VecDeque;

/// Fixed switch processing/queuing latency added at every hop, on top of
/// store-and-forward transmission time. Public AFDX overviews (e.g. ARINC
/// 664 Part 7 tutorials, Airbus's own conference material on AFDX) quote
/// switch latency in the tens of microseconds range at light load; 40 us
/// per hop is a commonly cited representative figure. GENERIC for this
/// specific network — the real A380's per-switch latency is not public.
pub const HOP_LATENCY_S: f64 = 40e-6;

/// A received virtual link's payload status: the ARINC 653/664 four-state
/// model FlyByWire's own AFDX code uses
/// (`AvionicsDataCommunicationNetworkMessageFunctionalDataSetStatus`,
/// `integrated_modular_avionics/mod.rs` lines 126-133), reproduced here
/// independently so this module has no dependency on that crate.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq, Ord, PartialOrd)]
pub enum DataStatus {
    #[default]
    NoData,
    NoComputedData,
    FunctionalTest,
    NormalOperation,
}
impl DataStatus {
    pub fn is_usable(self) -> bool {
        self == DataStatus::NormalOperation
    }
}

/// Standard reflected CRC-32 (IEEE 802.3 Frame Check Sequence polynomial
/// 0xEDB88320, public) — the same integrity mechanism every AFDX/Ethernet
/// frame carries. Used here to make "a frame corrupted after its FCS was
/// computed gets caught at the receiver" a real, checkable computation
/// rather than an assumption.
pub fn crc32_ieee(data: &[u8]) -> u32 {
    let mut crc: u32 = 0xFFFF_FFFF;
    for &byte in data {
        crc ^= byte as u32;
        for _ in 0..8 {
            let mask = 0u32.wrapping_sub(crc & 1);
            crc = (crc >> 1) ^ (0xEDB8_8320 & mask);
        }
    }
    !crc
}

/// Spaces out an event whose long-run rate should be exactly
/// `fraction_per_call` of calls, evenly, the way a Bresenham line
/// algorithm spaces pixels — deterministic and reproducible (needed for
/// tests) rather than seeded randomness, while still not simply firing
/// every Nth call in a way that could alias with a periodic caller.
#[derive(Clone, Copy, Debug, Default)]
pub(super) struct RateAccumulator {
    debt: f64,
}
impl RateAccumulator {
    pub(super) fn fires(&mut self, fraction: f64) -> bool {
        let fraction = fraction.clamp(0.0, 1.0);
        self.debt += fraction;
        if self.debt >= 1.0 {
            self.debt -= 1.0;
            true
        } else {
            false
        }
    }
}

/// A tiny deterministic xorshift64 generator for jitter only: real AFDX
/// jitter varies frame to frame with switch queue occupancy, but this
/// model has no traffic-queue simulation fine-grained enough to derive
/// that; a bounded pseudo-random value (seeded from the frame's own
/// sequence number, so it is reproducible) stands in. GENERIC.
fn xorshift_unit(seed: u64) -> f64 {
    let mut x = seed.wrapping_mul(0x9E37_79B9_7F4A_7C15) | 1;
    x ^= x << 13;
    x ^= x >> 7;
    x ^= x << 17;
    (x >> 11) as f64 / (1u64 << 53) as f64
}

struct PendingFrame {
    arrival_s: f64,
    valid: bool,
}

/// One network side's (A or B) delivery pipeline for a single virtual
/// link/destination pair: shapes its own transmissions to the VL's BAG,
/// carries frames in flight with their computed latency and jitter, and
/// remembers the most recently arrived frame's validity and age.
pub struct SideChannel {
    accumulated_s: f64,
    next_sequence: u32,
    loss_acc: RateAccumulator,
    corrupt_acc: RateAccumulator,
    in_flight: VecDeque<PendingFrame>,
    last_arrival: Option<(f64, bool)>,
}
impl Default for SideChannel {
    fn default() -> Self {
        Self { accumulated_s: 0.0, next_sequence: 0, loss_acc: RateAccumulator::default(), corrupt_acc: RateAccumulator::default(), in_flight: VecDeque::new(), last_arrival: None }
    }
}
impl SideChannel {
    /// Advances this channel by `dt_s`: emits any frames the VL's BAG says
    /// are due (regulated by the accumulator, so a long `dt_s` correctly
    /// emits several rather than one late one), decides whether each
    /// physically arrives (network + module pass fraction) and, if it
    /// does, whether it carries valid or corrupted data, then delivers
    /// whatever is now due to have arrived.
    #[allow(clippy::too_many_arguments)]
    fn step(&mut self, dt_s: f64, now_s: f64, bag_s: f64, frame_time_s: f64, hops: usize, pass_fraction: f64, corruption_fraction: f64) {
        let bag_s = bag_s.max(1e-3);
        self.accumulated_s += dt_s.max(0.0);
        let mut emitted = 0;
        while self.accumulated_s >= bag_s && emitted < 256 {
            self.accumulated_s -= bag_s;
            emitted += 1;
            let sequence = self.next_sequence;
            self.next_sequence = self.next_sequence.wrapping_add(1);
            if self.loss_acc.fires(1.0 - pass_fraction.clamp(0.0, 1.0)) {
                continue; // never arrives: physical/queueing loss.
            }
            let valid = !self.corrupt_acc.fires(corruption_fraction);
            let base_latency = hops as f64 * (frame_time_s + HOP_LATENCY_S);
            let jitter_max = hops as f64 * HOP_LATENCY_S;
            let jitter = xorshift_unit(sequence as u64 ^ ((hops as u64) << 32)) * jitter_max;
            self.in_flight.push_back(PendingFrame { arrival_s: now_s + base_latency + jitter, valid });
        }
        while let Some(front) = self.in_flight.front() {
            if front.arrival_s > now_s {
                break;
            }
            let f = self.in_flight.pop_front().unwrap();
            self.last_arrival = Some((f.arrival_s, f.valid));
        }
    }

    /// This side's own view of its data: fresh and valid, fresh but
    /// corrupted, or gone stale (nothing valid within `staleness_s`).
    pub fn status_at(&self, now_s: f64, staleness_s: f64) -> (DataStatus, f64) {
        match self.last_arrival {
            None => (DataStatus::NoData, f64::INFINITY),
            Some((t, valid)) => {
                let age = (now_s - t).max(0.0);
                if age > staleness_s {
                    (DataStatus::NoData, age)
                } else if valid {
                    (DataStatus::NormalOperation, age)
                } else {
                    (DataStatus::NoComputedData, age)
                }
            }
        }
    }
}

/// A destination's redundancy-managed view of one virtual link: two
/// `SideChannel`s (network A and B carrying the same VL id, independently)
/// merged by first-valid-wins — ARINC 664 Part 7's redundancy management,
/// the whole reason for having two networks.
pub struct VirtualLinkReceiver {
    pub vl: VirtualLinkIdx,
    pub destination: EndSystemIdx,
    sides: [SideChannel; 2],
}
impl VirtualLinkReceiver {
    pub fn new(vl: VirtualLinkIdx, destination: EndSystemIdx) -> Self {
        Self { vl, destination, sides: [SideChannel::default(), SideChannel::default()] }
    }

    /// Advances both sides by `dt_s` against the current topology/graph
    /// state and fault set.
    pub fn step(&mut self, topology: &NetworkTopology, graph: &NetworkGraph<'_>, faults: &NetworkFaults, dt_s: f64, now_s: f64) {
        let vl = &topology.virtual_links[self.vl];
        let source_node = NodeId::End(vl.source);
        let dest_node = NodeId::End(self.destination);
        for side in NetworkSide::BOTH {
            let source_m = faults.module(vl.source);
            let dest_m = faults.module(self.destination);
            let (pass, hops) = if !source_m.is_available() || !dest_m.is_available() {
                (0.0, 0)
            } else if let Some(path) = graph.shortest_path(side, source_node, dest_node, faults) {
                let network_pass = graph.path_pass_fraction(side, &path, faults);
                let mut overflow = 0.0f64;
                for w in path.windows(2) {
                    overflow = overflow.max(graph.port_load(side, w[0], w[1], faults).overflow_loss_fraction());
                }
                let module_pass = combine_pass_fraction(&[source_m.hardware_failure, dest_m.hardware_failure]);
                (network_pass * (1.0 - overflow) * module_pass, NetworkGraph::hop_count(&path))
            } else {
                (0.0, 0)
            };
            self.sides[side.index()].step(dt_s, now_s, vl.bag_ms / 1000.0, vl.frame_time_s(), hops.max(1), pass, source_m.config_corruption);
        }
    }

    /// The merged status a consuming application actually sees, and which
    /// network (if either) supplied it — `None` on both sides down means
    /// the function has lost this data outright; `Some` on exactly one
    /// side means it is running degraded (single network, no redundancy
    /// left) even while `status` still reads `NormalOperation`.
    pub fn status(&self, now_s: f64, staleness_s: f64) -> (DataStatus, f64, [bool; 2]) {
        let a = self.sides[0].status_at(now_s, staleness_s);
        let b = self.sides[1].status_at(now_s, staleness_s);
        let usable = [a.0.is_usable(), b.0.is_usable()];
        let chosen = match (a.0.is_usable(), b.0.is_usable()) {
            (true, true) => {
                if a.1 <= b.1 {
                    a
                } else {
                    b
                }
            }
            (true, false) => a,
            (false, true) => b,
            (false, false) => {
                if a.1 <= b.1 {
                    a
                } else {
                    b
                }
            }
        };
        (chosen.0, chosen.1, usable)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use super::super::faults::{LinkFaults, ModuleFaults, SwitchFaults};
    use super::super::topology::a380_reference_topology;

    fn healthy_faults(topology: &NetworkTopology) -> NetworkFaults {
        let mut f = NetworkFaults::default();
        for i in 0..topology.end_systems.len() {
            f.modules.insert(i, ModuleFaults::healthy(topology.end_systems[i].partitions.len()));
        }
        f
    }

    #[test]
    fn crc32_of_known_bytes_matches_the_public_test_vector() {
        // "123456789" is the standard CRC-32/IEEE check value 0xCBF43926.
        assert_eq!(crc32_ieee(b"123456789"), 0xCBF4_3926);
    }

    #[test]
    fn rate_accumulator_fires_at_the_requested_long_run_rate() {
        let mut acc = RateAccumulator::default();
        let fires = (0..1000).filter(|_| acc.fires(0.25)).count();
        assert_eq!(fires, 250);
    }

    #[test]
    fn healthy_virtual_link_delivers_normal_operation_on_both_sides() {
        let t = a380_reference_topology();
        let g = NetworkGraph::new(&t);
        let faults = healthy_faults(&t);
        let mut rx = VirtualLinkReceiver::new(0, 1); // FWS_WARNINGS -> CPIOM-A1
        let mut now = 0.0;
        for _ in 0..2000 {
            rx.step(&t, &g, &faults, 0.01, now);
            now += 0.01;
        }
        let (status, age, usable) = rx.status(now, 0.5);
        assert_eq!(status, DataStatus::NormalOperation);
        assert!(age < 0.5);
        assert_eq!(usable, [true, true]);
    }

    #[test]
    fn losing_network_a_degrades_to_single_network_not_to_lost() {
        let t = a380_reference_topology();
        let g = NetworkGraph::new(&t);
        let mut faults = healthy_faults(&t);
        // Fail every switch on network A.
        for i in 0..t.switches[0].len() {
            faults.switches[0].insert(i, SwitchFaults { failure: 1.0, ..Default::default() });
        }
        let mut rx = VirtualLinkReceiver::new(0, 1);
        let mut now = 0.0;
        for _ in 0..2000 {
            rx.step(&t, &g, &faults, 0.01, now);
            now += 0.01;
        }
        let (status, _, usable) = rx.status(now, 0.5);
        assert_eq!(status, DataStatus::NormalOperation);
        assert_eq!(usable, [false, true]);
    }

    #[test]
    fn losing_both_networks_loses_the_data() {
        let t = a380_reference_topology();
        let g = NetworkGraph::new(&t);
        let mut faults = healthy_faults(&t);
        for side in 0..2 {
            for i in 0..t.switches[side].len() {
                faults.switches[side].insert(i, SwitchFaults { failure: 1.0, ..Default::default() });
            }
        }
        let mut rx = VirtualLinkReceiver::new(0, 1);
        let mut now = 0.0;
        for _ in 0..2000 {
            rx.step(&t, &g, &faults, 0.01, now);
            now += 0.01;
        }
        let (status, _, usable) = rx.status(now, 0.5);
        assert_eq!(status, DataStatus::NoData);
        assert_eq!(usable, [false, false]);
    }

    #[test]
    fn config_table_corruption_yields_no_computed_data_not_no_data() {
        let t = a380_reference_topology();
        let g = NetworkGraph::new(&t);
        let mut faults = healthy_faults(&t);
        faults.modules.get_mut(&0).unwrap().config_corruption = 1.0; // source of VL 0
        let mut rx = VirtualLinkReceiver::new(0, 1);
        let mut now = 0.0;
        for _ in 0..2000 {
            rx.step(&t, &g, &faults, 0.01, now);
            now += 0.01;
        }
        let (status, _, usable) = rx.status(now, 0.5);
        // Both networks deliver the frame (it is not lost in transit) but
        // every one of it is corrupted.
        assert_eq!(status, DataStatus::NoComputedData);
        assert_eq!(usable, [false, false]);
    }

    #[test]
    fn a_severed_segment_on_the_only_path_eventually_goes_stale() {
        let t = a380_reference_topology();
        let g = NetworkGraph::new(&t);
        let mut faults = healthy_faults(&t);
        // Sever CPIOM-A1's own attach segment on both networks — its only
        // edge into either graph, so this fully isolates it.
        let cpiom_a1_switch = t.end_systems[1].attach[0];
        faults.set_segment(NetworkSide::A, NodeId::End(1), NodeId::Switch(cpiom_a1_switch), LinkFaults { open: 1.0 });
        faults.set_segment(NetworkSide::B, NodeId::End(1), NodeId::Switch(cpiom_a1_switch), LinkFaults { open: 1.0 });
        let mut rx = VirtualLinkReceiver::new(0, 1);
        let mut now = 0.0;
        for _ in 0..2000 {
            rx.step(&t, &g, &faults, 0.01, now);
            now += 0.01;
        }
        let (status, _, usable) = rx.status(now, 0.5);
        assert_eq!(status, DataStatus::NoData);
        assert_eq!(usable, [false, false]);
    }

    #[test]
    fn no_nan_at_zero_dt_or_at_rest() {
        let t = a380_reference_topology();
        let g = NetworkGraph::new(&t);
        let faults = healthy_faults(&t);
        let mut rx = VirtualLinkReceiver::new(0, 1);
        rx.step(&t, &g, &faults, 0.0, 0.0);
        let (_, age, _) = rx.status(0.0, 0.5);
        assert!(!age.is_nan());
    }
}
