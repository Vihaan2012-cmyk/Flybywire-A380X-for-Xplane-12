use super::consequences::{Availability, FunctionMonitor};
use super::faults::{EndSystemFaults, LinkFaults, ModuleFaults, PartitionFaults, SwitchFaults};
use super::graph::{NetworkFaults, NetworkGraph};
use super::registry::key_safe;
use super::topology::{
    a380_reference_topology, fbw_switch_id, ModuleKind, NetworkSide, NetworkTopology, NodeId,
};
use super::ventilation::{
    Bay, ExtractValveFaults, Fan, FanFaults, OverheatTrip, CPIOM_HEAT_W, IOM_HEAT_W, SUPPLY_AIR_K,
};
use super::{consequences, registry};
use crate::deep::api::Registry;
use crate::deep::live::{Area, Faults, Truth};
use std::collections::BTreeMap;

pub fn live_system() -> Box<dyn Area> {
    Box::new(LiveAvionicsNetwork::new())
}

const AC_BUS_ALIVE_V: f64 = 90.0;
const DC_BUS_ALIVE_V: f64 = 18.0;

const AC_BUS_POTENTIAL_VAR: [&str; 4] =
    ["ELEC_AC_1_BUS_POTENTIAL", "ELEC_AC_2_BUS_POTENTIAL", "ELEC_AC_3_BUS_POTENTIAL", "ELEC_AC_4_BUS_POTENTIAL"];
const DC_BUS_POTENTIAL_VAR: [&str; 2] = ["ELEC_DC_1_BUS_POTENTIAL", "ELEC_DC_2_BUS_POTENTIAL"];

const STALENESS_S: f64 = 0.5;

const FULLY_FAILED: f64 = 0.5;

pub struct FaultIndex(BTreeMap<String, u64>);

impl FaultIndex {
    pub fn build() -> Self {
        let mut r = Registry::default();
        registry::register(&mut r);
        Self(r.failures.iter().map(|f| (f.name.clone(), f.id)).collect())
    }

    pub fn id(&self, name: &str) -> u64 {
        self.0.get(name).copied().unwrap_or(0)
    }
}

struct SwitchIds {
    failure: u64,
    ports: Vec<(NodeId, u64)>,
}

struct ModuleIds {
    hardware: u64,
    config: u64,
    babbling: u64,
    power: u64,
    partitions: Vec<u64>,
}

struct BayIds {
    fans: [u64; 2],
    valve: u64,
}

struct Ids {
    switches: [Vec<SwitchIds>; 2],
    segments: [Vec<((NodeId, NodeId), u64)>; 2],
    modules: Vec<ModuleIds>,
    bays: Vec<BayIds>,
}

struct LiveBay {
    key: String,
    bay: Bay,
    modules: Vec<usize>,
    fan_bus: [usize; 2],
}

#[derive(Clone, Debug, Default)]
struct Snapshot {
    network_available: [bool; 2],
    module_available: Vec<bool>,
    module_powered: Vec<bool>,
    module_overheat_trip: Vec<f64>,
    module_pass_fraction: Vec<f64>,
    bay_airflow: Vec<f64>,
    bay_temp_c: Vec<f64>,
    function_availability: Vec<f64>,
    function_age_s: Vec<f64>,

    switch_available: [Vec<bool>; 2],
    switch_health_frac: [Vec<f64>; 2],
    switch_port_health_frac: [Vec<Vec<f64>>; 2],
    segment_health_frac: [Vec<f64>; 2],
    module_network_reachable: Vec<[bool; 2]>,
    module_networks_up: Vec<f64>,
    module_partition_available: Vec<Vec<bool>>,
    module_port_load_frac: Vec<[f64; 2]>,
    bay_fan_health_frac: Vec<[f64; 2]>,
    vl_paths_up: Vec<f64>,
    fws_customization_db_rejected: bool,
    fws_atqc_db_rejected: bool,
    cds_fcu_switch_off: bool,
}

fn availability_code(a: Availability) -> f64 {
    match a {
        Availability::Normal => 2.0,
        Availability::Degraded => 1.0,
        Availability::Lost => 0.0,
    }
}

fn node_key(topology: &NetworkTopology, side: NetworkSide, node: NodeId) -> String {
    match node {
        NodeId::Switch(i) => key_safe(&topology.switches[side.index()][i].name),
        NodeId::End(i) => key_safe(topology.end_systems[i].name),
    }
}

fn side_letter(side: NetworkSide) -> &'static str {
    match side {
        NetworkSide::A => "A",
        NetworkSide::B => "B",
    }
}

pub struct LiveAvionicsNetwork {
    topology: NetworkTopology,
    ids: Ids,
    monitors: Vec<FunctionMonitor>,
    bays: Vec<LiveBay>,
    trips: Vec<OverheatTrip>,
    module_bus: Vec<usize>,
    now_s: f64,
    snapshot: Snapshot,
    coolg_overheat_id: [u64; 2],
    coolg_prot_fault_id: [u64; 2],
    coolg_overheat: [f64; 2],
    coolg_prot_fault: [f64; 2],
    fws_partition: (usize, usize),
    fws_customization_db_rejected_id: u64,
    fws_atqc_db_rejected_id: u64,
}

impl Default for LiveAvionicsNetwork {
    fn default() -> Self {
        Self::new()
    }
}

impl LiveAvionicsNetwork {
    pub fn new() -> Self {
        let topology = a380_reference_topology();
        let index = FaultIndex::build();

        let switches = NetworkSide::BOTH.map(|side| {
            topology.switches[side.index()]
                .iter()
                .enumerate()
                .map(|(i, spec)| SwitchIds {
                    failure: index.id(&format!("AFDX switch {} failure", spec.name)),
                    ports: topology
                        .switch_ports(side, i)
                        .into_iter()
                        .map(|neighbour| {
                            let id = index.id(&format!(
                                "AFDX switch {} port to {:?} failure",
                                spec.name, neighbour
                            ));
                            (neighbour, id)
                        })
                        .collect(),
                })
                .collect::<Vec<_>>()
        });

        let segments = NetworkSide::BOTH.map(|side| {
            topology
                .edges(side)
                .into_iter()
                .map(|(a, b)| {
                    let id =
                        index.id(&format!("AFDX cable {:?}-{:?} ({:?}) failure", a, b, side));
                    ((a, b), id)
                })
                .collect::<Vec<_>>()
        });

        let modules: Vec<ModuleIds> = topology
            .end_systems
            .iter()
            .map(|es| ModuleIds {
                hardware: index.id(&format!("{} hardware failure", es.name)),
                config: index.id(&format!("{} configuration table corruption", es.name)),
                babbling: index
                    .id(&format!("{} babbling (unregulated transmission)", es.name)),
                power: index.id(&format!("{} loss of bus power", es.name)),
                partitions: es
                    .partitions
                    .iter()
                    .map(|p| index.id(&format!("{} partition {} failure", es.name, p)))
                    .collect(),
            })
            .collect();

        let mut bay_names: Vec<&'static str> =
            topology.end_systems.iter().map(|e| e.bay).collect();
        bay_names.sort_unstable();
        bay_names.dedup();

        let bay_ids: Vec<BayIds> = bay_names
            .iter()
            .map(|bay| BayIds {
                fans: ["PRIMARY", "STANDBY"]
                    .map(|role| index.id(&format!("{bay} {role} extraction fan failure"))),
                valve: index.id(&format!("{bay} extract valve stuck closed")),
            })
            .collect();

        let bays: Vec<LiveBay> = bay_names
            .iter()
            .enumerate()
            .map(|(b, &name)| LiveBay {
                key: key_safe(name),
                bay: Bay::new(SUPPLY_AIR_K),
                modules: topology
                    .end_systems
                    .iter()
                    .enumerate()
                    .filter(|(_, es)| es.bay == name)
                    .map(|(i, _)| i)
                    .collect(),
                fan_bus: [(2 * b) % 4, (2 * b + 1) % 4],
            })
            .collect();

        let module_bus = (0..topology.end_systems.len()).map(|i| i % 2).collect();

        let trips = vec![OverheatTrip::default(); topology.end_systems.len()];
        let monitors = consequences::reference_function_monitors();

        let coolg_overheat_id = [1, 2].map(|n| index.id(&format!("Avionics cooling system {n} overheat")));
        let coolg_prot_fault_id = [1, 2].map(|n| index.id(&format!("Avionics cooling system {n} protection fault")));
        let fws_module = topology.end_systems.iter().position(|es| es.name == "CPIOM-C1").expect("CPIOM-C1 module exists");
        let fws_partition_idx = topology.end_systems[fws_module].partitions.iter().position(|&p| p == "FWS").expect("CPIOM-C1 hosts an FWS partition");
        let fws_customization_db_rejected_id = index.id("CPIOM-C1 partition FWS customization database rejected");
        let fws_atqc_db_rejected_id = index.id("CPIOM-C1 partition FWS ATQC database rejected");

        Self {
            topology,
            ids: Ids { switches, segments, modules, bays: bay_ids },
            monitors,
            bays,
            trips,
            module_bus,
            now_s: 0.0,
            snapshot: Snapshot::default(),
            coolg_overheat_id,
            coolg_prot_fault_id,
            coolg_overheat: [0.0; 2],
            coolg_prot_fault: [0.0; 2],
            fws_partition: (fws_module, fws_partition_idx),
            fws_customization_db_rejected_id,
            fws_atqc_db_rejected_id,
        }
    }

    fn bus_volts(truth: &Truth, name: Option<&&'static str>, raw: f64) -> f64 {
        name.and_then(|n| truth.published.get(n)).unwrap_or(raw)
    }

    fn ac_bus_alive(truth: &Truth, bus: usize) -> bool {
        let raw = truth.ac_bus_volts.get(bus).copied().unwrap_or(0.0);
        Self::bus_volts(truth, AC_BUS_POTENTIAL_VAR.get(bus), raw) > AC_BUS_ALIVE_V
    }

    fn dc_bus_alive(truth: &Truth, bus: usize) -> bool {
        let raw = truth.dc_bus_volts.get(bus).copied().unwrap_or(0.0);
        Self::bus_volts(truth, DC_BUS_POTENTIAL_VAR.get(bus), raw) > DC_BUS_ALIVE_V
    }

    fn module_heat_w(kind: ModuleKind, available: bool) -> f64 {
        if !available {
            return 0.0;
        }
        match kind {
            ModuleKind::Cpiom(_) => CPIOM_HEAT_W,
            ModuleKind::Iom => IOM_HEAT_W,
        }
    }

    fn network_faults(&self, truth: &Truth, faults: &Faults) -> NetworkFaults {
        let mut nf = NetworkFaults::default();

        for side in NetworkSide::BOTH {
            for (i, s) in self.ids.switches[side.index()].iter().enumerate() {
                let mut sf = SwitchFaults { failure: faults.get(s.failure), ..Default::default() };
                for &(neighbour, id) in &s.ports {
                    let m = faults.get(id);
                    if m > 0.0 {
                        sf.port_failure.insert(neighbour, m);
                    }
                }
                if sf.failure > 0.0 || !sf.port_failure.is_empty() {
                    nf.switches[side.index()].insert(i, sf);
                }
            }
            for &((a, b), id) in &self.ids.segments[side.index()] {
                let open = faults.get(id);
                if open > 0.0 {
                    nf.set_segment(side, a, b, LinkFaults { open });
                }
            }
        }

        for (i, m) in self.ids.modules.iter().enumerate() {
            let babbling = faults.get(m.babbling);
            if babbling > 0.0 {
                nf.end_systems.insert(i, EndSystemFaults { babbling });
            }
            nf.modules.insert(
                i,
                ModuleFaults {
                    hardware_failure: faults.get(m.hardware),
                    config_corruption: faults.get(m.config),
                    partitions: m
                        .partitions
                        .iter()
                        .map(|&id| PartitionFaults { failure: faults.get(id), ..PartitionFaults::default() })
                        .collect(),
                    powered: Self::dc_bus_alive(truth, self.module_bus[i])
                        && faults.get(m.power) < FULLY_FAILED,
                    overheat_trip_frac: self.trips[i].frac(),
                },
            );
        }

        nf
    }
}

impl Area for LiveAvionicsNetwork {
    fn name(&self) -> &'static str {
        "avionics_network"
    }

    fn tick(&mut self, truth: &Truth, faults: &Faults) {
        let dt = truth.dt_s;
        self.now_s += dt;

        let previously_available: Vec<bool> = (0..self.topology.end_systems.len())
            .map(|i| {
                Self::dc_bus_alive(truth, self.module_bus[i])
                    && faults.get(self.ids.modules[i].power) < FULLY_FAILED
                    && faults.get(self.ids.modules[i].hardware) < 1.0
                    && self.trips[i].frac() < 1.0
            })
            .collect();

        let mut bay_airflow = vec![0.0; self.bays.len()];
        let mut bay_temp_c = vec![0.0; self.bays.len()];
        let mut bay_fan_health_frac = vec![[0.0_f64; 2]; self.bays.len()];
        for (b, live) in self.bays.iter_mut().enumerate() {
            let ids = &self.ids.bays[b];
            let fans: Vec<Fan> = [0usize, 1]
                .map(|f| Fan {
                    powered: Self::ac_bus_alive(truth, live.fan_bus[f]),
                    faults: FanFaults { failure: faults.get(ids.fans[f]) },
                })
                .to_vec();
            bay_fan_health_frac[b] = [fans[0].output_frac(), fans[1].output_frac()];
            let heat_w: f64 = live
                .modules
                .iter()
                .map(|&i| {
                    Self::module_heat_w(
                        self.topology.end_systems[i].kind,
                        previously_available[i],
                    )
                })
                .sum();
            let state = live.bay.step(
                heat_w,
                &fans,
                1.0,
                &ExtractValveFaults { stuck_closed: faults.get(ids.valve) },
                dt,
            );
            bay_airflow[b] = state.airflow_frac;
            bay_temp_c[b] = state.temp_k - 273.15;
            for &i in &live.modules {
                self.trips[i].step(state.temp_k, dt);
            }
        }

        let nf = self.network_faults(truth, faults);
        let graph = NetworkGraph::memoized(&self.topology);

        let module_available: Vec<bool> =
            (0..self.topology.end_systems.len()).map(|i| nf.module(i).is_available()).collect();
        let module_powered: Vec<bool> = (0..self.topology.end_systems.len()).map(|i| nf.module(i).powered).collect();
        let module_pass_fraction: Vec<f64> =
            (0..self.topology.end_systems.len()).map(|i| nf.module(i).pass_fraction()).collect();
        let module_overheat_trip: Vec<f64> = self.trips.iter().map(|t| t.frac()).collect();

        let end_count = self.topology.end_systems.len();
        let mut reach: [Vec<Option<Vec<bool>>>; 2] = [vec![None; end_count], vec![None; end_count]];
        let mut reach_from = |side: NetworkSide, a: usize| -> Vec<bool> {
            reach[side.index()][a].get_or_insert_with(|| graph.reachable_ends(side, NodeId::End(a), &nf)).clone()
        };
        let mut network_available = [false; 2];
        let mut module_network_reachable = vec![[false; 2]; end_count];
        for side in NetworkSide::BOTH {
            let s = side.index();
            let up: Vec<usize> = (0..end_count).filter(|&i| module_available[i]).collect();
            for &a in &up {
                let from_a = reach_from(side, a);
                for &b in &up {
                    if a != b && from_a[b] {
                        network_available[s] = true;
                        module_network_reachable[a][s] = true;
                    }
                }
            }
        }
        let module_networks_up: Vec<f64> =
            module_network_reachable.iter().map(|r| r[0] as u8 as f64 + r[1] as u8 as f64).collect();

        let mut switch_available: [Vec<bool>; 2] = Default::default();
        let mut switch_health_frac: [Vec<f64>; 2] = Default::default();
        let mut switch_port_health_frac: [Vec<Vec<f64>>; 2] = Default::default();
        let mut segment_health_frac: [Vec<f64>; 2] = Default::default();
        for side in NetworkSide::BOTH {
            let s = side.index();
            for sw in &self.ids.switches[s] {
                let failure = faults.get(sw.failure).clamp(0.0, 1.0);
                switch_available[s].push(failure < 1.0);
                switch_health_frac[s].push(1.0 - failure);
                switch_port_health_frac[s]
                    .push(sw.ports.iter().map(|&(_, id)| 1.0 - faults.get(id).clamp(0.0, 1.0)).collect());
            }
            for &(_, id) in &self.ids.segments[s] {
                segment_health_frac[s].push(1.0 - faults.get(id).clamp(0.0, 1.0));
            }
        }

        let module_partition_available: Vec<Vec<bool>> = self
            .ids
            .modules
            .iter()
            .enumerate()
            .map(|(i, m)| m.partitions.iter().map(|&pid| module_available[i] && faults.get(pid) < 1.0).collect())
            .collect();

        let (fws_module, fws_part) = self.fws_partition;
        let fws_partition_available = module_partition_available.get(fws_module).and_then(|v| v.get(fws_part)).copied().unwrap_or(false);
        let fws_customization_db_rejected = fws_partition_available && faults.get(self.fws_customization_db_rejected_id) >= FULLY_FAILED;
        let fws_atqc_db_rejected = fws_partition_available && faults.get(self.fws_atqc_db_rejected_id) >= FULLY_FAILED;

        let module_port_load_frac: Vec<[f64; 2]> = (0..self.topology.end_systems.len())
            .map(|i| {
                NetworkSide::BOTH.map(|side| {
                    let switch = NodeId::Switch(self.topology.end_systems[i].attach[side.index()]);
                    let load = graph.port_load(side, NodeId::End(i), switch, &nf);
                    if load.capacity_bps > 0.0 {
                        (load.offered_bps / load.capacity_bps).max(0.0)
                    } else {
                        0.0
                    }
                })
            })
            .collect();

        let vl_paths_up: Vec<f64> = self
            .topology
            .virtual_links
            .iter()
            .map(|vl| {
                NetworkSide::BOTH
                    .iter()
                    .filter(|&&side| {
                        let from_source = reach_from(side, vl.source);
                        vl.destinations.iter().all(|&d| from_source.get(d).copied().unwrap_or(false))
                    })
                    .count() as f64
            })
            .collect();

        for monitor in &mut self.monitors {
            monitor.step(&self.topology, &graph, &nf, dt, self.now_s);
        }
        let (function_availability, function_age_s) = self
            .monitors
            .iter()
            .map(|m| {
                let status = m.status(self.now_s, STALENESS_S);
                (availability_code(status.availability), status.age_s.min(1e6))
            })
            .unzip();

        self.snapshot = Snapshot {
            network_available,
            module_available,
            module_powered,
            module_overheat_trip,
            module_pass_fraction,
            bay_airflow,
            bay_temp_c,
            function_availability,
            function_age_s,
            switch_available,
            switch_health_frac,
            switch_port_health_frac,
            segment_health_frac,
            module_network_reachable,
            module_networks_up,
            module_partition_available,
            module_port_load_frac,
            bay_fan_health_frac,
            vl_paths_up,
            fws_customization_db_rejected,
            fws_atqc_db_rejected,
            cds_fcu_switch_off: truth.controls.fcu_switch_off,
        };

        for i in 0..2 {
            self.coolg_overheat[i] = faults.get(self.coolg_overheat_id[i]);
            self.coolg_prot_fault[i] = faults.get(self.coolg_prot_fault_id[i]);
        }
    }

    fn publish(&self, out: &mut dyn FnMut(&str, f64)) {
        let s = &self.snapshot;
        for i in 0..2 {
            out(&format!("DEEP_AVNCS_COOLG_{}_OVHT", i + 1), self.coolg_overheat[i]);
        }
        out("DEEP_AVNCS_COOLG_PROT_FAULT", if self.coolg_prot_fault.iter().any(|&m| m > 0.0) { 1.0 } else { 0.0 });

        out("AFDX_NETWORK_A_AVAILABLE", f64::from(s.network_available[0]));
        out("AFDX_NETWORK_B_AVAILABLE", f64::from(s.network_available[1]));
        out(
            "AFDX_FAILED_CABLE_COUNT",
            s.segment_health_frac.iter().flatten().filter(|&&h| 1.0 - h >= FULLY_FAILED).count() as f64,
        );
        for (i, es) in self.topology.end_systems.iter().enumerate() {
            let key = key_safe(es.name);
            out(
                &format!("AVNCS_MODULE_{key}_AVAILABLE"),
                f64::from(s.module_available.get(i).copied().unwrap_or(false)),
            );
            out(
                &format!("AVNCS_MODULE_{key}_POWERED"),
                f64::from(s.module_powered.get(i).copied().unwrap_or(false)),
            );
        }
        out(
            "AVNCS_ANY_MODULE_POWERED",
            if s.module_powered.iter().any(|&p| p) { 1.0 } else { 0.0 },
        );
        for (b, bay) in self.bays.iter().enumerate() {
            out(
                &format!("AVNCS_{}_AIRFLOW_FRAC", bay.key),
                s.bay_airflow.get(b).copied().unwrap_or(0.0),
            );
        }

        for (b, bay) in self.bays.iter().enumerate() {
            out(&format!("AVNCS_{}_TEMP_C", bay.key), s.bay_temp_c.get(b).copied().unwrap_or(0.0));
        }
        for (i, es) in self.topology.end_systems.iter().enumerate() {
            let key = key_safe(es.name);
            out(
                &format!("AVNCS_MODULE_{key}_OVERHEAT_TRIP"),
                s.module_overheat_trip.get(i).copied().unwrap_or(0.0),
            );
            out(
                &format!("AVNCS_MODULE_{key}_PASS_FRACTION"),
                s.module_pass_fraction.get(i).copied().unwrap_or(0.0),
            );
        }
        for (i, monitor) in self.monitors.iter().enumerate() {
            let key = key_safe(monitor.spec.name);
            out(
                &format!("AVNCS_FUNCTION_{key}_AVAILABILITY"),
                s.function_availability.get(i).copied().unwrap_or(0.0),
            );
            out(
                &format!("AVNCS_FUNCTION_{key}_AGE_S"),
                s.function_age_s.get(i).copied().unwrap_or(0.0),
            );
        }

        for side in NetworkSide::BOTH {
            let sidx = side.index();
            for (i, spec) in self.topology.switches[sidx].iter().enumerate() {
                let key = key_safe(&spec.name);
                out(
                    &format!("AVNCS_SWITCH_{key}_AVAILABLE"),
                    f64::from(s.switch_available[sidx].get(i).copied().unwrap_or(true)),
                );
                out(
                    &format!("AVNCS_SWITCH_{key}_HEALTH_FRAC"),
                    s.switch_health_frac[sidx].get(i).copied().unwrap_or(1.0),
                );
                out(
                    &format!("AFDX_SWITCH_{}_FAILURE", fbw_switch_id(side, i)),
                    f64::from(!s.switch_available[sidx].get(i).copied().unwrap_or(true)),
                );
                for (p, neighbour) in self.topology.switch_ports(side, i).into_iter().enumerate() {
                    let nkey = node_key(&self.topology, side, neighbour);
                    out(
                        &format!("AVNCS_SWITCH_{key}_PORT_{nkey}_HEALTH_FRAC"),
                        s.switch_port_health_frac[sidx].get(i).and_then(|v| v.get(p)).copied().unwrap_or(1.0),
                    );
                }
            }
            for (k, (a, b)) in self.topology.edges(side).into_iter().enumerate() {
                let akey = node_key(&self.topology, side, a);
                let bkey = node_key(&self.topology, side, b);
                out(
                    &format!("AVNCS_CABLE_{akey}_{bkey}_{}_HEALTH_FRAC", side_letter(side)),
                    s.segment_health_frac[sidx].get(k).copied().unwrap_or(1.0),
                );
            }
        }

        for (i, es) in self.topology.end_systems.iter().enumerate() {
            let key = key_safe(es.name);
            for (p, part) in es.partitions.iter().enumerate() {
                let pkey = key_safe(part);
                out(
                    &format!("AVNCS_MODULE_{key}_PARTITION_{pkey}_AVAILABLE"),
                    f64::from(s.module_partition_available.get(i).and_then(|v| v.get(p)).copied().unwrap_or(false)),
                );
            }
            let load = s.module_port_load_frac.get(i).copied().unwrap_or([0.0; 2]);
            out(&format!("AVNCS_MODULE_{key}_PORT_LOAD_FRAC_A"), load[0]);
            out(&format!("AVNCS_MODULE_{key}_PORT_LOAD_FRAC_B"), load[1]);
            let reach = s.module_network_reachable.get(i).copied().unwrap_or([false; 2]);
            out(&format!("AVNCS_MODULE_{key}_NETWORK_A_REACHABLE"), f64::from(reach[0]));
            out(&format!("AVNCS_MODULE_{key}_NETWORK_B_REACHABLE"), f64::from(reach[1]));
            out(
                &format!("AVNCS_MODULE_{key}_NETWORKS_UP"),
                s.module_networks_up.get(i).copied().unwrap_or(0.0),
            );
        }

        for (b, bay) in self.bays.iter().enumerate() {
            let health = s.bay_fan_health_frac.get(b).copied().unwrap_or([0.0; 2]);
            out(&format!("AVNCS_{}_FAN_PRIMARY_HEALTH_FRAC", bay.key), health[0]);
            out(&format!("AVNCS_{}_FAN_STANDBY_HEALTH_FRAC", bay.key), health[1]);
        }

        for (i, vl) in self.topology.virtual_links.iter().enumerate() {
            let key = key_safe(vl.name);
            out(&format!("AVNCS_VL_{key}_PATHS_UP"), s.vl_paths_up.get(i).copied().unwrap_or(0.0));
            out(&format!("AVNCS_VL_{key}_PATHS_DESIGNED"), NetworkSide::BOTH.len() as f64);
        }

        out("AVNCS_MODULE_CPIOM_C1_CUSTOMIZATION_DB_REJECTED", f64::from(s.fws_customization_db_rejected));
        out("AVNCS_MODULE_CPIOM_C1_ATQC_DB_REJECTED", f64::from(s.fws_atqc_db_rejected));
        out("CDS_FCU_SWITCH_OFF", f64::from(s.cds_fcu_switch_off));
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::deep::live::PublishedFrame;

    fn powered() -> Truth {
        Truth {
            dt_s: 1.0 / 30.0,
            ac_bus_volts: [115.0; 4],
            dc_bus_volts: [28.0; 2],
            ..Truth::default()
        }
    }

    fn vars(area: &LiveAvionicsNetwork) -> BTreeMap<String, f64> {
        let mut map = BTreeMap::new();
        area.publish(&mut |name, value| {
            map.insert(name.to_string(), value);
        });
        map
    }

    fn run(
        area: &mut LiveAvionicsNetwork,
        truth: &Truth,
        faults: &Faults,
        seconds: f64,
    ) -> BTreeMap<String, f64> {
        let ticks = (seconds / truth.dt_s).round().max(1.0) as u32;
        for _ in 0..ticks {
            area.tick(truth, faults);
        }
        vars(area)
    }

    #[test]
    fn every_live_element_resolved_a_real_failure_id() {
        let area = LiveAvionicsNetwork::new();
        let mut ids = Vec::new();
        for side in 0..2 {
            for s in &area.ids.switches[side] {
                ids.push(s.failure);
                ids.extend(s.ports.iter().map(|&(_, id)| id));
            }
            ids.extend(area.ids.segments[side].iter().map(|&(_, id)| id));
        }
        for m in &area.ids.modules {
            ids.extend([m.hardware, m.config, m.babbling, m.power]);
            ids.extend(m.partitions.iter().copied());
        }
        for b in &area.ids.bays {
            ids.extend(b.fans);
            ids.push(b.valve);
        }
        assert!(ids.iter().all(|&id| id != 0), "a live network element has no catalogue failure behind it");
        let mut sorted = ids.clone();
        sorted.sort_unstable();
        sorted.dedup();
        assert_eq!(sorted.len(), ids.len(), "two live elements resolved to the same failure id");
    }

    #[test]
    fn a_healthy_powered_aircraft_has_both_networks_every_module_and_every_function() {
        let mut area = LiveAvionicsNetwork::new();
        let published = run(&mut area, &powered(), &Faults::default(), 5.0);

        assert_eq!(published["AFDX_NETWORK_A_AVAILABLE"], 1.0);
        assert_eq!(published["AFDX_NETWORK_B_AVAILABLE"], 1.0);
        assert_eq!(published["AVNCS_MODULE_CPIOM_C1_AVAILABLE"], 1.0);
        assert_eq!(published["AVNCS_MODULE_IOM_A5_AVAILABLE"], 1.0);
        assert_eq!(published["AVNCS_AVIONICS_BAY_FWD_AIRFLOW_FRAC"], 1.0);
        assert!(published["AVNCS_AVIONICS_BAY_FWD_TEMP_C"] < 60.0);
        for (name, value) in &published {
            assert!(value.is_finite(), "{name} = {value}");
            if name.ends_with("_AVAILABILITY") {
                assert_eq!(*value, 2.0, "{name} is not fully redundant on a healthy aircraft");
            }
        }
    }

    #[test]
    fn with_no_dc_power_no_module_is_on_either_network() {
        let mut area = LiveAvionicsNetwork::new();
        let truth = Truth { ac_bus_volts: [115.0; 4], ..Truth::default() };
        let published = run(&mut area, &truth, &Faults::default(), 5.0);
        assert_eq!(published["AVNCS_MODULE_CPIOM_C1_AVAILABLE"], 0.0);
        assert_eq!(published["AFDX_NETWORK_A_AVAILABLE"], 0.0);
        assert_eq!(published["AFDX_NETWORK_B_AVAILABLE"], 0.0);
    }

    #[test]
    fn an_armed_module_hardware_failure_takes_that_module_and_its_function_down() {
        let index = FaultIndex::build();
        let id = index.id("CPIOM-C1 hardware failure");
        assert_ne!(id, 0);

        let mut area = LiveAvionicsNetwork::new();
        let published = run(&mut area, &powered(), &Faults::from_pairs([(id, 1.0)]), 5.0);

        assert_eq!(published["AVNCS_MODULE_CPIOM_C1_AVAILABLE"], 0.0);
        assert_eq!(published["AVNCS_MODULE_CPIOM_A1_AVAILABLE"], 1.0, "it took out the wrong module");
        assert_eq!(
            published["AVNCS_FUNCTION_ECAM_WARNINGS_AT_CPIOM_A1_AVAILABILITY"], 0.0,
            "the function fed by the failed module still reports available"
        );
        assert_eq!(published["AFDX_NETWORK_A_AVAILABLE"], 1.0);
    }

    #[test]
    fn cutting_one_networks_attachment_cable_costs_redundancy_not_the_function() {
        let area = LiveAvionicsNetwork::new();
        let attach = area.topology.end_systems[0].attach[0];
        let index = FaultIndex::build();
        let id = index.id(&format!(
            "AFDX cable {:?}-{:?} ({:?}) failure",
            NodeId::End(0),
            NodeId::Switch(attach),
            NetworkSide::A
        ));
        assert_ne!(id, 0, "the cable's registered name changed");

        let mut area = LiveAvionicsNetwork::new();
        let published = run(&mut area, &powered(), &Faults::from_pairs([(id, 1.0)]), 5.0);

        assert_eq!(
            published["AVNCS_FUNCTION_ECAM_WARNINGS_AT_CPIOM_A1_AVAILABILITY"], 1.0,
            "a single cable cut must leave the function degraded, not normal and not lost"
        );
        assert_eq!(published["AVNCS_MODULE_CPIOM_C1_AVAILABLE"], 1.0);
    }

    #[test]
    fn the_failed_cable_count_counts_only_fully_severed_cables() {
        let area = LiveAvionicsNetwork::new();
        let cables: Vec<u64> = area.ids.segments[0].iter().take(3).map(|&(_, id)| id).collect();

        let mut area = LiveAvionicsNetwork::new();
        let healthy = run(&mut area, &powered(), &Faults::default(), 5.0);
        assert_eq!(healthy["AFDX_FAILED_CABLE_COUNT"], 0.0);

        let mut area = LiveAvionicsNetwork::new();
        let one = run(&mut area, &powered(), &Faults::from_pairs([(cables[0], 1.0), (cables[1], 0.2)]), 5.0);
        assert_eq!(one["AFDX_FAILED_CABLE_COUNT"], 1.0);

        let mut area = LiveAvionicsNetwork::new();
        let two = run(&mut area, &powered(), &Faults::from_pairs([(cables[0], 1.0), (cables[2], 1.0)]), 5.0);
        assert_eq!(two["AFDX_FAILED_CABLE_COUNT"], 2.0);
    }

    #[test]
    fn losing_one_bay_fan_keeps_the_draught_and_losing_both_does_not() {
        let index = FaultIndex::build();
        let primary = index.id("AVIONICS_BAY_FWD PRIMARY extraction fan failure");
        let standby = index.id("AVIONICS_BAY_FWD STANDBY extraction fan failure");
        assert!(primary != 0 && standby != 0);

        let mut one = LiveAvionicsNetwork::new();
        let published = run(&mut one, &powered(), &Faults::from_pairs([(primary, 1.0)]), 5.0);
        assert_eq!(published["AVNCS_AVIONICS_BAY_FWD_AIRFLOW_FRAC"], 1.0, "one fan still ventilates the bay");

        let mut both = LiveAvionicsNetwork::new();
        let published = run(
            &mut both,
            &powered(),
            &Faults::from_pairs([(primary, 1.0), (standby, 1.0)]),
            5.0,
        );
        assert_eq!(published["AVNCS_AVIONICS_BAY_FWD_AIRFLOW_FRAC"], 0.0);
        assert_eq!(
            published["AVNCS_AVIONICS_BAY_AFT_AIRFLOW_FRAC"], 1.0,
            "it took the cooling out of the wrong bay"
        );
    }

    #[test]
    fn a_bay_that_loses_all_cooling_eventually_trips_its_modules_off_the_network() {
        let index = FaultIndex::build();
        let valve = index.id("AVIONICS_BAY_FWD extract valve stuck closed");
        let mut area = LiveAvionicsNetwork::new();
        let truth = Truth { dt_s: 2.0, ..powered() };
        let faults = Faults::from_pairs([(valve, 1.0)]);

        let early = run(&mut area, &truth, &faults, 60.0);
        assert_eq!(early["AVNCS_AVIONICS_BAY_FWD_AIRFLOW_FRAC"], 0.0);
        assert_eq!(early["AVNCS_MODULE_CPIOM_C1_AVAILABLE"], 1.0, "it tripped far too fast");

        let late = run(&mut area, &truth, &faults, 7_200.0);
        assert!(
            late["AVNCS_AVIONICS_BAY_FWD_TEMP_C"] > 70.0,
            "the bay only reached {} C",
            late["AVNCS_AVIONICS_BAY_FWD_TEMP_C"]
        );
        assert_eq!(late["AVNCS_MODULE_CPIOM_C1_OVERHEAT_TRIP"], 1.0);
        assert_eq!(late["AVNCS_MODULE_CPIOM_C1_AVAILABLE"], 0.0);
        assert_eq!(
            late["AVNCS_MODULE_CPIOM_F1_AVAILABLE"], 1.0,
            "the aft bay's modules must be untouched"
        );
    }

    #[test]
    fn every_variable_an_ecam_trigger_names_is_published() {
        let mut area = LiveAvionicsNetwork::new();
        area.tick(&powered(), &Faults::default());
        let published = vars(&area);
        assert!(published.contains_key("AFDX_NETWORK_A_AVAILABLE"));
        assert!(published.contains_key("AFDX_NETWORK_B_AVAILABLE"));
        for es in &area.topology.end_systems {
            let name = format!("AVNCS_MODULE_{}_AVAILABLE", key_safe(es.name));
            assert!(published.contains_key(&name), "{name} is never published");
        }
        for bay in &area.bays {
            let name = format!("AVNCS_{}_AIRFLOW_FRAC", bay.key);
            assert!(published.contains_key(&name), "{name} is never published");
        }
        assert_eq!(area.name(), "avionics_network");
    }

    #[test]
    fn cutting_both_sides_of_a_modules_attachment_shows_redundancy_loss_then_function_loss() {
        let area = LiveAvionicsNetwork::new();
        let attach = area.topology.end_systems[0].attach;
        let index = FaultIndex::build();
        let id_a = index.id(&format!(
            "AFDX cable {:?}-{:?} ({:?}) failure",
            NodeId::End(0),
            NodeId::Switch(attach[0]),
            NetworkSide::A
        ));
        let id_b = index.id(&format!(
            "AFDX cable {:?}-{:?} ({:?}) failure",
            NodeId::End(0),
            NodeId::Switch(attach[1]),
            NetworkSide::B
        ));
        assert!(id_a != 0 && id_b != 0, "the cables' registered names changed");

        let mut one = LiveAvionicsNetwork::new();
        let published = run(&mut one, &powered(), &Faults::from_pairs([(id_a, 1.0)]), 5.0);
        assert_eq!(published["AVNCS_MODULE_CPIOM_C1_AVAILABLE"], 1.0, "the module itself is untouched");
        assert_eq!(published["AVNCS_MODULE_CPIOM_C1_NETWORK_A_REACHABLE"], 0.0, "the cut side must read unreachable");
        assert_eq!(published["AVNCS_MODULE_CPIOM_C1_NETWORK_B_REACHABLE"], 1.0, "the other side is untouched");
        assert_eq!(published["AVNCS_MODULE_CPIOM_C1_NETWORKS_UP"], 1.0, "exactly one fault from losing the function");
        assert_eq!(
            published["AVNCS_FUNCTION_ECAM_WARNINGS_AT_CPIOM_A1_AVAILABILITY"], 1.0,
            "degraded, not lost, on the first cut alone"
        );

        let mut both = LiveAvionicsNetwork::new();
        let published = run(&mut both, &powered(), &Faults::from_pairs([(id_a, 1.0), (id_b, 1.0)]), 5.0);
        assert_eq!(published["AVNCS_MODULE_CPIOM_C1_NETWORK_A_REACHABLE"], 0.0);
        assert_eq!(published["AVNCS_MODULE_CPIOM_C1_NETWORK_B_REACHABLE"], 0.0);
        assert_eq!(published["AVNCS_MODULE_CPIOM_C1_NETWORKS_UP"], 0.0);
        assert_eq!(
            published["AVNCS_FUNCTION_ECAM_WARNINGS_AT_CPIOM_A1_AVAILABILITY"], 0.0,
            "both sides cut: the function must actually be lost now"
        );
    }

    #[test]
    fn a_single_port_failure_moves_that_ports_own_health_reading() {
        let area = LiveAvionicsNetwork::new();
        let switch_name = area.topology.switches[0][0].name.clone();
        let neighbour = area.topology.switch_ports(NetworkSide::A, 0)[0];
        let index = FaultIndex::build();
        let id = index.id(&format!("AFDX switch {switch_name} port to {neighbour:?} failure"));
        assert_ne!(id, 0, "the port's registered name changed");

        let mut area = LiveAvionicsNetwork::new();
        let published = run(&mut area, &powered(), &Faults::from_pairs([(id, 1.0)]), 1.0);

        let switch_key = key_safe(&switch_name);
        let neighbour_key = node_key(&area.topology, NetworkSide::A, neighbour);
        let var_name = format!("AVNCS_SWITCH_{switch_key}_PORT_{neighbour_key}_HEALTH_FRAC");
        assert_eq!(published[&var_name], 0.0, "a fully failed port must read zero on its own variable");
        assert_eq!(
            published[&format!("AVNCS_SWITCH_{switch_key}_HEALTH_FRAC")], 1.0,
            "the switch's own health is independent of one port"
        );
    }

    #[test]
    fn a_cable_and_a_switch_failure_each_move_their_own_component_reading() {
        let area = LiveAvionicsNetwork::new();
        let (a, b) = area.topology.edges(NetworkSide::A)[0];
        let cable_index = FaultIndex::build();
        let cable_id = cable_index.id(&format!("AFDX cable {a:?}-{b:?} ({:?}) failure", NetworkSide::A));
        assert_ne!(cable_id, 0);

        let mut cable_area = LiveAvionicsNetwork::new();
        let published = run(&mut cable_area, &powered(), &Faults::from_pairs([(cable_id, 1.0)]), 1.0);
        let akey = node_key(&cable_area.topology, NetworkSide::A, a);
        let bkey = node_key(&cable_area.topology, NetworkSide::A, b);
        assert_eq!(published[&format!("AVNCS_CABLE_{akey}_{bkey}_A_HEALTH_FRAC")], 0.0);

        let switch_name = area.topology.switches[0][1].name.clone();
        let switch_index = FaultIndex::build();
        let switch_id = switch_index.id(&format!("AFDX switch {switch_name} failure"));
        assert_ne!(switch_id, 0);

        let mut switch_area = LiveAvionicsNetwork::new();
        let published = run(&mut switch_area, &powered(), &Faults::from_pairs([(switch_id, 1.0)]), 1.0);
        let switch_key = key_safe(&switch_name);
        assert_eq!(published[&format!("AVNCS_SWITCH_{switch_key}_AVAILABLE")], 0.0);
        assert_eq!(published[&format!("AVNCS_SWITCH_{switch_key}_HEALTH_FRAC")], 0.0);
    }

    #[test]
    fn a_partition_failure_moves_only_its_own_partition_variable() {
        let index = FaultIndex::build();
        let id = index.id("CPIOM-C1 partition ECAM failure");
        assert_ne!(id, 0);

        let mut area = LiveAvionicsNetwork::new();
        let published = run(&mut area, &powered(), &Faults::from_pairs([(id, 1.0)]), 1.0);

        assert_eq!(published["AVNCS_MODULE_CPIOM_C1_PARTITION_ECAM_AVAILABLE"], 0.0);
        assert_eq!(
            published["AVNCS_MODULE_CPIOM_C1_PARTITION_FWS_AVAILABLE"], 1.0,
            "its sibling partition is untouched"
        );
        assert_eq!(
            published["AVNCS_MODULE_CPIOM_C1_AVAILABLE"], 1.0,
            "the module's own AFDX interface is unaffected by one partition"
        );
    }

    #[test]
    fn a_babbling_end_system_moves_its_own_egress_port_load_fraction() {
        let index = FaultIndex::build();
        let id = index.id("CPIOM-C1 babbling (unregulated transmission)");
        assert_ne!(id, 0);

        let mut healthy = LiveAvionicsNetwork::new();
        let base = run(&mut healthy, &powered(), &Faults::default(), 1.0);
        assert!(base["AVNCS_MODULE_CPIOM_C1_PORT_LOAD_FRAC_A"] < 1.0, "a healthy port is not oversubscribed");

        let mut area = LiveAvionicsNetwork::new();
        let published = run(&mut area, &powered(), &Faults::from_pairs([(id, 1.0)]), 1.0);
        assert!(
            published["AVNCS_MODULE_CPIOM_C1_PORT_LOAD_FRAC_A"] > 1.0,
            "a babbling transmitter must oversubscribe its own port"
        );
    }

    #[test]
    fn a_single_fan_failure_moves_its_own_health_reading_even_though_the_bays_airflow_does_not() {
        let index = FaultIndex::build();
        let primary = index.id("AVIONICS_BAY_FWD PRIMARY extraction fan failure");
        assert_ne!(primary, 0);

        let mut area = LiveAvionicsNetwork::new();
        let published = run(&mut area, &powered(), &Faults::from_pairs([(primary, 1.0)]), 1.0);

        assert_eq!(
            published["AVNCS_AVIONICS_BAY_FWD_FAN_PRIMARY_HEALTH_FRAC"], 0.0,
            "the failed fan's own health must read zero"
        );
        assert_eq!(
            published["AVNCS_AVIONICS_BAY_FWD_FAN_STANDBY_HEALTH_FRAC"], 1.0,
            "its healthy twin is untouched"
        );
        assert_eq!(
            published["AVNCS_AVIONICS_BAY_FWD_AIRFLOW_FRAC"], 1.0,
            "the bay draught is still fully established behind the healthy fan"
        );
    }

    #[test]
    fn a_virtual_links_path_count_reflects_the_network_not_just_endpoint_health() {
        let mut healthy = LiveAvionicsNetwork::new();
        let published = run(&mut healthy, &powered(), &Faults::default(), 1.0);
        assert_eq!(
            published["AVNCS_VL_FWS_WARNINGS_PATHS_UP"], 2.0,
            "a healthy dual network carries every VL on both sides"
        );
        assert_eq!(published["AVNCS_VL_FWS_WARNINGS_PATHS_DESIGNED"], 2.0);

        let mut one_side_down = LiveAvionicsNetwork::new();
        let ids: Vec<(u64, f64)> = one_side_down.ids.switches[0].iter().map(|s| (s.failure, 1.0)).collect();
        let published = run(&mut one_side_down, &powered(), &Faults::from_pairs(ids), 1.0);
        assert_eq!(
            published["AVNCS_VL_FWS_WARNINGS_PATHS_UP"], 1.0,
            "losing every switch on one side must cost that VL exactly one path"
        );
        assert_eq!(published["AVNCS_VL_FWS_WARNINGS_PATHS_DESIGNED"], 2.0, "the design target does not change");
    }

    #[test]
    fn module_power_follows_the_electrical_areas_solved_bus_not_the_raw_truth_field() {
        let mut truth = powered();
        truth.published = PublishedFrame::from(BTreeMap::from([
            ("ELEC_DC_1_BUS_POTENTIAL".to_string(), 0.0),
            ("ELEC_DC_2_BUS_POTENTIAL".to_string(), 0.0),
        ]));
        assert_eq!(truth.dc_bus_volts, [28.0; 2], "setup: the raw field still claims both DC mains are at nominal");

        let mut area = LiveAvionicsNetwork::new();
        let published = run(&mut area, &truth, &Faults::default(), 5.0);
        assert_eq!(
            published["AVNCS_MODULE_CPIOM_C1_AVAILABLE"], 0.0,
            "the electrical model says its bus is dead; the module cannot still be on the network"
        );
        assert_eq!(published["AVNCS_MODULE_CPIOM_A1_AVAILABLE"], 0.0);
        assert_eq!(published["AFDX_NETWORK_A_AVAILABLE"], 0.0);
        assert_eq!(published["AFDX_NETWORK_B_AVAILABLE"], 0.0);
    }

    #[test]
    fn the_electrical_areas_solved_bus_also_wins_when_the_raw_field_is_the_pessimistic_one() {
        let mut truth = Truth { dt_s: 1.0 / 30.0, ..Truth::default() };
        assert_eq!(truth.dc_bus_volts, [0.0; 2], "setup: the raw field claims a dark aircraft");
        truth.published = PublishedFrame::from(BTreeMap::from([
            ("ELEC_AC_1_BUS_POTENTIAL".to_string(), 115.0),
            ("ELEC_AC_2_BUS_POTENTIAL".to_string(), 115.0),
            ("ELEC_AC_3_BUS_POTENTIAL".to_string(), 115.0),
            ("ELEC_AC_4_BUS_POTENTIAL".to_string(), 115.0),
            ("ELEC_DC_1_BUS_POTENTIAL".to_string(), 28.0),
            ("ELEC_DC_2_BUS_POTENTIAL".to_string(), 28.0),
        ]));

        let mut area = LiveAvionicsNetwork::new();
        let published = run(&mut area, &truth, &Faults::default(), 5.0);
        assert_eq!(published["AVNCS_MODULE_CPIOM_C1_AVAILABLE"], 1.0);
        assert_eq!(published["AFDX_NETWORK_A_AVAILABLE"], 1.0);
        assert_eq!(
            published["AVNCS_AVIONICS_BAY_FWD_AIRFLOW_FRAC"], 1.0,
            "the bay extraction fans hang off the solved AC buses too"
        );
    }

    #[test]
    fn losing_one_solved_dc_main_takes_only_the_modules_on_that_bus() {
        let mut truth = powered();
        truth.published = PublishedFrame::from(BTreeMap::from([
            ("ELEC_DC_1_BUS_POTENTIAL".to_string(), 0.0),
            ("ELEC_DC_2_BUS_POTENTIAL".to_string(), 28.0),
        ]));
        let mut area = LiveAvionicsNetwork::new();
        let published = run(&mut area, &truth, &Faults::default(), 5.0);
        assert_eq!(published["AVNCS_MODULE_CPIOM_C1_AVAILABLE"], 0.0, "CPIOM-C1 is end system 0, so it is on DC 1");
        assert_eq!(
            published["AVNCS_MODULE_CPIOM_A1_AVAILABLE"], 1.0,
            "CPIOM-A1 is end system 1, so it is on DC 2 and must survive"
        );
    }

    #[test]
    fn a_real_electrical_bus_failure_takes_a_real_avionics_module_off_the_network() {
        use crate::deep::api::Registry;
        use crate::deep::electrical::live::board;
        use crate::deep::live::Deep;

        let short_dc1 = {
            let mut reg = Registry::default();
            crate::deep::electrical::registry::register(&mut reg);
            reg.failures
                .iter()
                .find(|f| f.component == "24_elec.bus.DC1" && f.model_field.contains("short_to_ground"))
                .expect("deep::electrical registers a short-to-ground on the DC 1 busbar")
                .id
        };

        let truth = || Truth {
            dt_s: 1.0 / 30.0,
            on_ground: false,
            engine_running: [true; 4],
            engine_n1_frac: [0.9; 4],
            engine_n2_frac: [0.9; 4],
            engine_n3_frac: [0.9; 4],
            ac_bus_volts: [115.0; 4],
            dc_bus_volts: [28.0; 2],
            ..Truth::default()
        };

        let run_for = |faults: &Faults| -> BTreeMap<String, f64> {
            board::clear();
            let mut deep = Deep::new()
                .with_area(crate::deep::electrical::live::live_system())
                .with_area(live_system());
            let mut published = BTreeMap::new();
            for _ in 0..300 {
                published.clear();
                deep.tick(truth(), faults, &mut |n, v| {
                    published.insert(n.to_string(), v);
                });
            }
            board::clear();
            published
        };

        let healthy = run_for(&Faults::default());
        assert!(
            healthy["ELEC_DC_1_BUS_POTENTIAL"] > DC_BUS_ALIVE_V,
            "setup: the electrical area must actually solve DC 1 alive"
        );
        assert_eq!(healthy["AVNCS_MODULE_CPIOM_C1_AVAILABLE"], 1.0, "setup: the module runs on a healthy aircraft");

        let shorted = run_for(&Faults::from_pairs([(short_dc1, 1.0)]));
        assert!(
            shorted["ELEC_DC_1_BUS_POTENTIAL"] <= DC_BUS_ALIVE_V,
            "a dead short on the busbar must collapse it, got {} V",
            shorted["ELEC_DC_1_BUS_POTENTIAL"]
        );
        assert_eq!(
            shorted["AVNCS_MODULE_CPIOM_C1_AVAILABLE"], 0.0,
            "an armed electrical failure must reach the avionics module it really feeds"
        );
    }

    #[test]
    fn fws_database_rejected_flags_fire_independently_and_need_the_partition_powered() {
        let index = FaultIndex::build();
        let custom_id = index.id("CPIOM-C1 partition FWS customization database rejected");
        let atqc_id = index.id("CPIOM-C1 partition FWS ATQC database rejected");
        assert_ne!(custom_id, 0);
        assert_ne!(atqc_id, 0);

        let mut healthy_area = LiveAvionicsNetwork::new();
        let healthy = run(&mut healthy_area, &powered(), &Faults::default(), 2.0);
        assert_eq!(healthy["AVNCS_MODULE_CPIOM_C1_CUSTOMIZATION_DB_REJECTED"], 0.0);
        assert_eq!(healthy["AVNCS_MODULE_CPIOM_C1_ATQC_DB_REJECTED"], 0.0);

        let mut custom_area = LiveAvionicsNetwork::new();
        let custom = run(&mut custom_area, &powered(), &Faults::from_pairs([(custom_id, 1.0)]), 2.0);
        assert_eq!(custom["AVNCS_MODULE_CPIOM_C1_CUSTOMIZATION_DB_REJECTED"], 1.0, "the armed check must fire");
        assert_eq!(custom["AVNCS_MODULE_CPIOM_C1_ATQC_DB_REJECTED"], 0.0, "the other check must stay quiet");

        let mut atqc_area = LiveAvionicsNetwork::new();
        let atqc = run(&mut atqc_area, &powered(), &Faults::from_pairs([(atqc_id, 1.0)]), 2.0);
        assert_eq!(atqc["AVNCS_MODULE_CPIOM_C1_ATQC_DB_REJECTED"], 1.0);
        assert_eq!(atqc["AVNCS_MODULE_CPIOM_C1_CUSTOMIZATION_DB_REJECTED"], 0.0);

        let mut cold_area = LiveAvionicsNetwork::new();
        let cold = run(&mut cold_area, &Truth::default(), &Faults::from_pairs([(custom_id, 1.0), (atqc_id, 1.0)]), 2.0);
        assert_eq!(cold["AVNCS_MODULE_CPIOM_C1_AVAILABLE"], 0.0, "setup: the module must actually be unpowered");
        assert_eq!(cold["AVNCS_MODULE_CPIOM_C1_CUSTOMIZATION_DB_REJECTED"], 0.0);
        assert_eq!(cold["AVNCS_MODULE_CPIOM_C1_ATQC_DB_REJECTED"], 0.0);
    }

    #[test]
    fn cds_fcu_switch_off_mirrors_truths_own_control_field() {
        let mut off_area = LiveAvionicsNetwork::new();
        let mut truth = powered();
        truth.controls.fcu_switch_off = true;
        let off = run(&mut off_area, &truth, &Faults::default(), 1.0);
        assert_eq!(off["CDS_FCU_SWITCH_OFF"], 1.0);

        let mut on_area = LiveAvionicsNetwork::new();
        let on = run(&mut on_area, &powered(), &Faults::default(), 1.0);
        assert_eq!(on["CDS_FCU_SWITCH_OFF"], 0.0);
    }
}
