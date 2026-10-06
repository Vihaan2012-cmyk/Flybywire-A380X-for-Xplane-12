use super::topology::NodeId;
use std::collections::HashMap;

pub fn combine_pass_fraction(faults: &[f64]) -> f64 {
    faults.iter().fold(1.0, |pass, &f| pass * (1.0 - f.clamp(0.0, 1.0)))
}

#[derive(Clone, Debug, Default)]
pub struct SwitchFaults {
    pub failure: f64,
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

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct LinkFaults {
    pub open: f64,
}
impl LinkFaults {
    pub fn is_available(&self) -> bool {
        self.open < 1.0
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub struct EndSystemFaults {
    pub babbling: f64,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct PartitionFaults {
    pub failure: f64,
    pub customization_db_rejected: f64,
    pub atqc_db_rejected: f64,
}

#[derive(Clone, Debug, Default)]
pub struct ModuleFaults {
    pub hardware_failure: f64,
    pub config_corruption: f64,
    pub partitions: Vec<PartitionFaults>,
    pub powered: bool,
    pub overheat_trip_frac: f64,
}
impl ModuleFaults {
    pub fn healthy(n_partitions: usize) -> Self {
        Self { powered: true, partitions: vec![PartitionFaults::default(); n_partitions], ..Default::default() }
    }

    pub fn is_available(&self) -> bool {
        self.powered && self.hardware_failure < 1.0 && self.overheat_trip_frac < 1.0
    }

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
