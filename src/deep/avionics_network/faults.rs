//! Faults this network model accepts, each a fraction `0.0` (healthy) ..
//! `1.0` (fully failed), following this crate's convention (see
//! `physics::engine::oil::OilFaults`). Every fault acts on a named element
//! of the graph built in `topology`/`graph`, never on a symptom directly.

use super::topology::NodeId;
use std::collections::HashMap;

/// Combines independent failure causes acting at the same point the way
/// independent probabilities combine in series: the chance a frame gets
/// through is the product of each cause's own chance of letting it through.
/// A single fraction of `1.0` anywhere in the product always yields `0.0`
/// (nothing gets through), matching a real total failure regardless of how
/// healthy everything else at that point is.
pub fn combine_pass_fraction(faults: &[f64]) -> f64 {
    faults.iter().fold(1.0, |pass, &f| pass * (1.0 - f.clamp(0.0, 1.0)))
}

/// A whole AFDX switch: total failure (drops every frame it would forward;
/// at `1.0` it also drops out of BFS reachability, exactly like FlyByWire's
/// `AvionicsFullDuplexSwitch::is_available`) and, independently, per-port
/// failure — the connector, its transceiver, or that one line card, which
/// only takes down the one segment on that port.
#[derive(Clone, Debug, Default)]
pub struct SwitchFaults {
    pub failure: f64,
    /// Keyed by the neighbour node this switch's port faces (see
    /// `NetworkTopology::switch_ports`); a neighbour with no entry is
    /// healthy.
    pub port_failure: HashMap<NodeId, f64>,
}
impl SwitchFaults {
    pub fn port_towards(&self, neighbour: NodeId) -> f64 {
        self.port_failure.get(&neighbour).copied().unwrap_or(0.0)
    }

    pub fn is_available(&self) -> bool {
        self.failure < 1.0
    }
}

/// One physical cable/segment between two ports (switch-switch or
/// switch-end system): open fraction, `0.0` healthy .. `1.0` fully severed
/// (connector pulled, cable cut).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct LinkFaults {
    pub open: f64,
}
impl LinkFaults {
    pub fn is_available(&self) -> bool {
        self.open < 1.0
    }
}

/// An end system (a CPIOM/IOM's network interface) gone rogue: `0.0` keeps
/// to its virtual links' regulated Bandwidth Allocation Gap (BAG); `1.0`
/// floods its egress port at the raw link rate regardless of any VL's
/// schedule — the AFDX "babbling idiot" case ARINC 664 Part 7's traffic
/// policing exists to catch, modelled in `graph::PortLoad`.
#[derive(Clone, Copy, Debug, Default)]
pub struct EndSystemFaults {
    pub babbling: f64,
}

/// One ARINC 653 partition (application) hosted on a CPIOM/IOM: crashed or
/// hung, `0.0` healthy .. `1.0` fully down. Independent of the module's
/// hardware and AFDX stack — a healthy CPIOM can lose one partition while
/// its others, and the network interface itself, keep running (ARINC 653's
/// whole point is partition-level fault containment).
#[derive(Clone, Copy, Debug, Default)]
pub struct PartitionFaults {
    pub failure: f64,
    /// `E-IND-DESIGN.md` 314800001: this partition's own FWS function
    /// rejected the loaded airline-customization database at startup (a
    /// checksum/version validity check) -- an inherently boolean software
    /// validation outcome, not a physical fraction, so this is read as a
    /// discrete flag (>= 0.5, this crate's own BITE-flag convention) rather
    /// than a continuous fault like `failure` above. Only meaningful on the
    /// CPIOM-C1 FWS partition; harmless (never armed) everywhere else.
    pub customization_db_rejected: f64,
    /// `E-IND-DESIGN.md` 314800005: as `customization_db_rejected`, for the
    /// ATQC (Airline Total Quality Control?) database validity check.
    pub atqc_db_rejected: f64,
}

/// A CPIOM or IOM module: its own hardware and loaded configuration, plus
/// the two conditions it cannot fix itself — bus power and bay cooling —
/// which are computed elsewhere (the crate's electrical model; this
/// crate's own `ventilation`) and handed in here each step as interface
/// inputs, not generated internally.
#[derive(Clone, Debug, Default)]
pub struct ModuleFaults {
    pub hardware_failure: f64,
    /// Loaded routing/VL configuration table corrupted (e.g. a checksum
    /// mismatch on last load): the module cannot resolve this fraction of
    /// its virtual link assignments, so traffic on the affected VLs is not
    /// sent/received at all rather than degraded gracefully — a corrupted
    /// table either matches an entry or it does not.
    pub config_corruption: f64,
    pub partitions: Vec<PartitionFaults>,
    /// Interface: is this module's bus energised right now.
    pub powered: bool,
    /// Interface: bay overheat trip from `ventilation` (`0.0` within
    /// limits, `1.0` tripped hot enough that the module's own supervisor
    /// has shut it down).
    pub overheat_trip_frac: f64,
}
impl ModuleFaults {
    pub fn healthy(n_partitions: usize) -> Self {
        Self { powered: true, partitions: vec![PartitionFaults::default(); n_partitions], ..Default::default() }
    }

    /// Whether the module is up enough to source or sink any AFDX traffic
    /// at all: unpowered, fully failed hardware or a completed overheat
    /// trip all take the whole box off the network, not just one
    /// partition.
    pub fn is_available(&self) -> bool {
        self.powered && self.hardware_failure < 1.0 && self.overheat_trip_frac < 1.0
    }

    /// Fraction of frames this module's own hardware degradation and
    /// config-table corruption combine to lose, for traffic that does get
    /// as far as being offered to a healthy, powered, cool module.
    pub fn pass_fraction(&self) -> f64 {
        if !self.is_available() {
            0.0
        } else {
            combine_pass_fraction(&[self.hardware_failure, self.config_corruption])
        }
    }

    pub fn partition_available(&self, index: usize) -> bool {
        self.is_available() && self.partitions.get(index).is_some_and(|p| p.failure < 1.0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn combining_healthy_causes_passes_everything() {
        assert_eq!(combine_pass_fraction(&[0.0, 0.0, 0.0]), 1.0);
    }

    #[test]
    fn one_full_failure_anywhere_blocks_everything() {
        assert_eq!(combine_pass_fraction(&[0.0, 1.0, 0.0]), 0.0);
    }

    #[test]
    fn two_partial_causes_are_worse_than_either_alone() {
        let one = combine_pass_fraction(&[0.5]);
        let two = combine_pass_fraction(&[0.5, 0.5]);
        assert!(two < one);
        assert!((two - 0.25).abs() < 1e-12);
    }

    #[test]
    fn switch_port_failure_is_zero_for_an_unlisted_neighbour() {
        let s = SwitchFaults::default();
        assert_eq!(s.port_towards(NodeId::Switch(3)), 0.0);
    }

    #[test]
    fn module_unpowered_is_unavailable_regardless_of_hardware_health() {
        let mut m = ModuleFaults::healthy(2);
        m.powered = false;
        assert!(!m.is_available());
        assert_eq!(m.pass_fraction(), 0.0);
    }

    #[test]
    fn module_overheat_trip_takes_it_off_the_network() {
        let mut m = ModuleFaults::healthy(1);
        m.overheat_trip_frac = 1.0;
        assert!(!m.is_available());
    }

    #[test]
    fn partition_failure_is_independent_of_its_siblings() {
        let mut m = ModuleFaults::healthy(2);
        m.partitions[0].failure = 1.0;
        assert!(!m.partition_available(0));
        assert!(m.partition_available(1));
    }
}
