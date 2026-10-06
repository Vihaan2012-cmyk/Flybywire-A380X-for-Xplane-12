use super::graph::{NetworkFaults, NetworkGraph};
use super::message::{DataStatus, VirtualLinkReceiver};
use super::topology::{a380_reference_topology, EndSystemIdx, NetworkTopology, VirtualLinkIdx};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Availability {
    Normal,
    Degraded,
    Lost,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FunctionStatus {
    pub availability: Availability,
    pub age_s: f64,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Combine {
    All,
    Any,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum SourceStatus {
    Afdx { status: DataStatus, age_s: f64, usable: [bool; 2] },
    Arinc429 { status: DataStatus, age_s: f64 },
}
impl SourceStatus {
    pub fn data_status(&self) -> DataStatus {
        match *self {
            SourceStatus::Afdx { status, .. } | SourceStatus::Arinc429 { status, .. } => status,
        }
    }

    pub fn age_s(&self) -> f64 {
        match *self {
            SourceStatus::Afdx { age_s, .. } | SourceStatus::Arinc429 { age_s, .. } => age_s,
        }
    }

    pub fn fully_redundant(&self) -> bool {
        match *self {
            SourceStatus::Afdx { usable, .. } => usable[0] && usable[1],
            SourceStatus::Arinc429 { status, .. } => status.is_usable(),
        }
    }
}

#[derive(Clone, Debug)]
pub struct FunctionSpec {
    pub name: &'static str,
    pub combine: Combine,
}

pub fn evaluate_function(spec: &FunctionSpec, sources: &[SourceStatus]) -> FunctionStatus {
    if sources.is_empty() {
        return FunctionStatus { availability: Availability::Lost, age_s: f64::INFINITY };
    }
    let usable: Vec<&SourceStatus> = sources.iter().filter(|s| s.data_status().is_usable()).collect();
    let met = match spec.combine {
        Combine::All => usable.len() == sources.len(),
        Combine::Any => !usable.is_empty(),
    };
    if !met {
        let age = sources.iter().map(|s| s.age_s()).fold(f64::INFINITY, f64::min);
        return FunctionStatus { availability: Availability::Lost, age_s: age };
    }
    let fully_redundant = match spec.combine {
        Combine::All => sources.iter().all(|s| s.fully_redundant()),
        Combine::Any => usable.iter().any(|s| s.fully_redundant()),
    };
    let age = match spec.combine {
        Combine::All => sources.iter().map(|s| s.age_s()).fold(0.0, f64::max),
        Combine::Any => usable.iter().map(|s| s.age_s()).fold(f64::INFINITY, f64::min),
    };
    FunctionStatus { availability: if fully_redundant { Availability::Normal } else { Availability::Degraded }, age_s: age }
}

pub struct FunctionMonitor {
    pub spec: FunctionSpec,
    receivers: Vec<VirtualLinkReceiver>,
}
impl FunctionMonitor {
    pub fn new(spec: FunctionSpec, links: Vec<(VirtualLinkIdx, EndSystemIdx)>) -> Self {
        let receivers = links.into_iter().map(|(vl, dest)| VirtualLinkReceiver::new(vl, dest)).collect();
        Self { spec, receivers }
    }

    pub fn step(&mut self, topology: &NetworkTopology, graph: &NetworkGraph<'_>, faults: &NetworkFaults, dt_s: f64, now_s: f64) {
        for rx in &mut self.receivers {
            rx.step(topology, graph, faults, dt_s, now_s);
        }
    }

    pub fn status(&self, now_s: f64, staleness_s: f64) -> FunctionStatus {
        let sources: Vec<SourceStatus> = self
            .receivers
            .iter()
            .map(|rx| {
                let (status, age_s, usable) = rx.status(now_s, staleness_s);
                SourceStatus::Afdx { status, age_s, usable }
            })
            .collect();
        evaluate_function(&self.spec, &sources)
    }
}

pub fn reference_function_monitors() -> Vec<FunctionMonitor> {
    let t = a380_reference_topology();
    debug_assert_eq!(t.virtual_links[0].name, "FWS_WARNINGS");
    debug_assert_eq!(t.virtual_links[2].name, "FUEL_STATE");
    let fws_warnings: VirtualLinkIdx = 0;
    let fuel_state: VirtualLinkIdx = 2;
    vec![
        FunctionMonitor::new(FunctionSpec { name: "ECAM warnings at CPIOM-A1", combine: Combine::All }, vec![(fws_warnings, 1)]),
        FunctionMonitor::new(
            FunctionSpec { name: "Fuel state observed by either IOM", combine: Combine::Any },
            vec![(fuel_state, 3), (fuel_state, 4)],
        ),
    ]
}

#[cfg(test)]
mod tests {
    use super::*;
    use super::super::faults::{ModuleFaults, SwitchFaults};

    fn afdx(status: DataStatus, usable: [bool; 2]) -> SourceStatus {
        SourceStatus::Afdx { status, age_s: 0.01, usable }
    }

    #[test]
    fn all_combine_is_lost_if_any_required_source_is_lost() {
        let spec = FunctionSpec { name: "x", combine: Combine::All };
        let sources = [afdx(DataStatus::NormalOperation, [true, true]), afdx(DataStatus::NoData, [false, false])];
        assert_eq!(evaluate_function(&spec, &sources).availability, Availability::Lost);
    }

    #[test]
    fn all_combine_is_degraded_when_one_source_lost_a_network_but_still_delivers() {
        let spec = FunctionSpec { name: "x", combine: Combine::All };
        let sources = [afdx(DataStatus::NormalOperation, [true, true]), afdx(DataStatus::NormalOperation, [true, false])];
        assert_eq!(evaluate_function(&spec, &sources).availability, Availability::Degraded);
    }

    #[test]
    fn all_combine_is_normal_when_every_source_has_full_redundancy() {
        let spec = FunctionSpec { name: "x", combine: Combine::All };
        let sources = [afdx(DataStatus::NormalOperation, [true, true]), afdx(DataStatus::NormalOperation, [true, true])];
        assert_eq!(evaluate_function(&spec, &sources).availability, Availability::Normal);
    }

    #[test]
    fn any_combine_survives_losing_one_of_two_independent_sources() {
        let spec = FunctionSpec { name: "x", combine: Combine::Any };
        let sources = [afdx(DataStatus::NoData, [false, false]), afdx(DataStatus::NormalOperation, [true, true])];
        assert_eq!(evaluate_function(&spec, &sources).availability, Availability::Normal);
    }

    #[test]
    fn any_combine_is_lost_only_once_every_source_is_gone() {
        let spec = FunctionSpec { name: "x", combine: Combine::Any };
        let sources = [afdx(DataStatus::NoData, [false, false]), afdx(DataStatus::NoData, [false, false])];
        assert_eq!(evaluate_function(&spec, &sources).availability, Availability::Lost);
    }

    #[test]
    fn a_function_with_no_sources_is_lost_not_a_panic() {
        let spec = FunctionSpec { name: "x", combine: Combine::All };
        let status = evaluate_function(&spec, &[]);
        assert_eq!(status.availability, Availability::Lost);
        assert!(status.age_s.is_infinite());
    }

    #[test]
    fn reference_function_monitor_degrades_then_is_lost_as_networks_fail() {
        let t = a380_reference_topology();
        let g = NetworkGraph::new(&t);
        let mut faults = NetworkFaults::default();
        for i in 0..t.end_systems.len() {
            faults.modules.insert(i, ModuleFaults::healthy(t.end_systems[i].partitions.len()));
        }
        let mut monitors = reference_function_monitors();
        let ecam = &mut monitors[0];
        let mut now = 0.0;
        for _ in 0..2000 {
            ecam.step(&t, &g, &faults, 0.01, now);
            now += 0.01;
        }
        assert_eq!(ecam.status(now, 0.5).availability, Availability::Normal);

        for i in 0..t.switches[0].len() {
            faults.switches[0].insert(i, SwitchFaults { failure: 1.0, ..Default::default() });
        }
        for _ in 0..2000 {
            ecam.step(&t, &g, &faults, 0.01, now);
            now += 0.01;
        }
        assert_eq!(ecam.status(now, 0.5).availability, Availability::Degraded);

        for i in 0..t.switches[1].len() {
            faults.switches[1].insert(i, SwitchFaults { failure: 1.0, ..Default::default() });
        }
        for _ in 0..2000 {
            ecam.step(&t, &g, &faults, 0.01, now);
            now += 0.01;
        }
        assert_eq!(ecam.status(now, 0.5).availability, Availability::Lost);
    }
}
