//! The live avionics data network: both AFDX networks, every switch, every
//! cable, every CPIOM/IOM and every avionics bay's cooling, owned and
//! stepped every frame.
//!
//! `topology.rs` describes the A380's dual-redundant ARINC 664 Part 7
//! network; `graph.rs` routes over it; `message.rs` delivers virtual-link
//! frames through it with real latency, jitter, CRC integrity and
//! redundancy management; `ventilation.rs` keeps the modules cool enough to
//! stay on it. None of it had an owner: nothing in the plugin constructed
//! a `NetworkTopology`, so no frame was ever routed. This module builds the
//! whole thing once and steps it.
//!
//! What it owns:
//!
//! * **Both networks, A and B**, each with the 8 switches and the
//!   inter-switch cabling of `topology::a380_reference_topology` -- the
//!   real dual-redundant arrangement, in which every module has its own
//!   network interface on each side and the same virtual link is sent
//!   independently over both.
//! * **Every switch port and every cable** as separately failable
//!   elements, keyed by the node each port faces, exactly as
//!   `registry.rs` registers them.
//! * **Every end system** (CPIOM-C1, CPIOM-A1, CPIOM-F1, IOM-A1, IOM-A5),
//!   with its hardware, its loaded configuration table, its egress traffic
//!   shaping, its bus power and each of its ARINC 653 partitions.
//! * **Every avionics bay**: its two extraction fans, its extract valve,
//!   its thermal node, and one overheat supervisor per module in it -- the
//!   causal chain by which losing bay cooling eventually takes modules off
//!   the network rather than being announced as a separate scripted
//!   effect.
//! * **The reference function monitors** (`consequences.rs`), so the
//!   consequence of a network fault is reported as a function's
//!   availability, not just as a link statistic.
//!
//! ## Ordering within a tick
//!
//! Bay temperature is stepped first, from the heat the modules that were
//! up at the end of the previous tick were dissipating; the overheat
//! supervisors then see this tick's temperature; and the network is routed
//! with the trip fractions those supervisors just produced. A module
//! cannot therefore trip and be routed around in the same frame it
//! overheats, which is right: the supervisor's own time constant is
//! seconds.
//!
//! ## What `Truth` cannot supply yet
//!
//! * **Which bus feeds which module and which fan.** `Truth` publishes two
//!   DC and four AC bus voltages but not the avionics load allocation, so
//!   modules are spread across the two DC buses and each bay's two fans
//!   across two AC buses (see [`LiveAvionicsNetwork::new`]). That spread is
//!   a real design property -- no bus loss may take out a whole bay -- but
//!   the actual A380 allocation is not public and is not in `Truth`;
//!   `Truth::avionics_module_bus` (or the electrical area publishing each
//!   module's supply) would replace it.
//! * **ARINC 429 bus faults.** `arinc429.rs` models the legacy
//!   point-to-point links, but `registry.rs` deliberately does not
//!   enumerate them per instance (which LRU on which wire is a
//!   per-installation fact), so there is no failure id to drive a channel
//!   with and none is instantiated here.

use super::consequences::{Availability, FunctionMonitor};
use super::faults::{EndSystemFaults, LinkFaults, ModuleFaults, PartitionFaults, SwitchFaults};
use super::graph::{NetworkFaults, NetworkGraph};
use super::registry::key_safe;
use super::topology::{
    a380_reference_topology, ModuleKind, NetworkSide, NetworkTopology, NodeId,
};
use super::ventilation::{
    Bay, ExtractValveFaults, Fan, FanFaults, OverheatTrip, CPIOM_HEAT_W, IOM_HEAT_W, SUPPLY_AIR_K,
};
use super::{consequences, registry};
use crate::deep::api::Registry;
use crate::deep::live::{Area, Faults, Truth};
use std::collections::BTreeMap;

/// The live system for this area.
pub fn live_system() -> Box<dyn Area> {
    Box::new(LiveAvionicsNetwork::new())
}

/// A 115 V AC bus (avionics extraction fans) and a 28 V DC bus (the
/// modules themselves) are alive above these.
const AC_BUS_ALIVE_V: f64 = 90.0;
const DC_BUS_ALIVE_V: f64 = 18.0;

/// How long a virtual link's data stays usable with nothing new arriving.
/// GENERIC: half a second is several times the loosest BAG in the
/// reference topology (128 ms) and far below any timescale a consuming
/// function acts on, so it distinguishes "the link is down" from "the next
/// frame has not arrived yet" without being sensitive to either.
const STALENESS_S: f64 = 0.5;

/// A failure magnitude at or above this is a fully-failed part, for the
/// entries `registry.rs` describes as discrete (bus power, partitions).
const FULLY_FAILED: f64 = 0.5;

// ---------------------------------------------------------------------
// Failure id resolution.
// ---------------------------------------------------------------------

/// Maps a registered failure's name to its id.
///
/// This area's ids come from a sequential counter over the topology, so
/// they are not constants that can be named. The failure *names* are
/// unique and are built from the topology (`"AFDX switch AFDX-A-1 port to
/// Switch(1) failure"`, `"CPIOM-C1 partition FWS failure"`, ...), so the
/// index is built by running the real registration and reading back what it
/// produced, and looked up by rebuilding the same name from the same
/// topology. If `registry.rs` renumbers, nothing here changes; if it
/// renames, `every_live_element_resolved_a_real_failure_id` fails loudly.
pub struct FaultIndex(BTreeMap<String, u64>);

impl FaultIndex {
    pub fn build() -> Self {
        let mut r = Registry::default();
        registry::register(&mut r);
        Self(r.failures.iter().map(|f| (f.name.clone(), f.id)).collect())
    }

    /// The id registered under this name, or 0 -- which `Faults::get`
    /// reads as healthy -- if there is none.
    pub fn id(&self, name: &str) -> u64 {
        self.0.get(name).copied().unwrap_or(0)
    }
}

// ---------------------------------------------------------------------
// Resolved ids, laid out the way `tick` walks them.
// ---------------------------------------------------------------------

struct SwitchIds {
    failure: u64,
    /// One entry per port, in `NetworkTopology::switch_ports` order.
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
    /// Primary and standby extraction fans.
    fans: [u64; 2],
    valve: u64,
}

struct Ids {
    switches: [Vec<SwitchIds>; 2],
    segments: [Vec<((NodeId, NodeId), u64)>; 2],
    modules: Vec<ModuleIds>,
    bays: Vec<BayIds>,
}

/// One avionics bay's live cooling.
struct LiveBay {
    /// `key_safe` form of the bay name, which is how `registry.rs` builds
    /// the Var name its VENT FAULT alert triggers on.
    key: String,
    bay: Bay,
    /// Indices into `topology.end_systems` of the modules in this bay.
    modules: Vec<usize>,
    /// Which AC bus each of the two fans is fed from.
    fan_bus: [usize; 2],
}

#[derive(Clone, Debug, Default)]
struct Snapshot {
    network_available: [bool; 2],
    module_available: Vec<bool>,
    module_overheat_trip: Vec<f64>,
    module_pass_fraction: Vec<f64>,
    bay_airflow: Vec<f64>,
    bay_temp_c: Vec<f64>,
    /// Per reference function: availability as a number (see
    /// [`availability_code`]) and the age of the data behind it.
    function_availability: Vec<f64>,
    function_age_s: Vec<f64>,
}

/// `Availability` as a published number: 2 normal (fully redundant),
/// 1 degraded (running on one network), 0 lost.
fn availability_code(a: Availability) -> f64 {
    match a {
        Availability::Normal => 2.0,
        Availability::Degraded => 1.0,
        Availability::Lost => 0.0,
    }
}

pub struct LiveAvionicsNetwork {
    topology: NetworkTopology,
    ids: Ids,
    monitors: Vec<FunctionMonitor>,
    bays: Vec<LiveBay>,
    /// One thermal supervisor per module, indexed like
    /// `topology.end_systems`.
    trips: Vec<OverheatTrip>,
    /// Which DC bus each module is fed from.
    module_bus: Vec<usize>,
    /// The network's own clock: `message.rs` schedules frame arrivals
    /// against an absolute time, so the area has to keep one.
    now_s: f64,
    snapshot: Snapshot,
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

        // The bay list is built exactly as `registry.rs` builds it
        // (sorted, deduplicated), so bay `n` here is bay `n` there.
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
                // **GENERIC allocation of a real design property.** That a
                // bay's two fans come off two different AC buses -- so no
                // single bus loss can take a bay's cooling out -- is the
                // segregation principle CS 25.1309 and CS 25.1360's
                // separation requirements force on any such installation,
                // and it is what this model has to reproduce. *Which* bus
                // feeds which fan is not public. Searched: A380 ATA 21/24
                // training material, FlyByWire's own
                // `avionics_data_communication_network.rs` (it carries the
                // switch/CPIOM/IOM topology this model already takes from
                // it, but models no electrical supply for them at all) and
                // their `electrical` module (no CPIOM or fan loads). Nobody
                // publishes the allocation, so a deterministic round-robin
                // that *guarantees* the segregation stands in for it.
                fan_bus: [(2 * b) % 4, (2 * b + 1) % 4],
            })
            .collect();

        // Same status, same search: modules alternate between the two DC
        // buses so neither bus carries a whole bay. The property (no single
        // bus loss empties a bay) is real; this particular assignment is
        // GENERIC.
        let module_bus = (0..topology.end_systems.len()).map(|i| i % 2).collect();

        let trips = vec![OverheatTrip::default(); topology.end_systems.len()];
        let monitors = consequences::reference_function_monitors();

        Self {
            topology,
            ids: Ids { switches, segments, modules, bays: bay_ids },
            monitors,
            bays,
            trips,
            module_bus,
            now_s: 0.0,
            snapshot: Snapshot::default(),
        }
    }

    fn ac_bus_alive(truth: &Truth, bus: usize) -> bool {
        truth.ac_bus_volts.get(bus).copied().unwrap_or(0.0) > AC_BUS_ALIVE_V
    }

    fn dc_bus_alive(truth: &Truth, bus: usize) -> bool {
        truth.dc_bus_volts.get(bus).copied().unwrap_or(0.0) > DC_BUS_ALIVE_V
    }

    /// What one module dissipates as heat right now: its rated
    /// dissipation if it is up, nothing at all if it is not (an unpowered
    /// or tripped box is not warming its bay).
    fn module_heat_w(kind: ModuleKind, available: bool) -> f64 {
        if !available {
            return 0.0;
        }
        match kind {
            ModuleKind::Cpiom(_) => CPIOM_HEAT_W,
            ModuleKind::Iom => IOM_HEAT_W,
        }
    }

    /// Assembles every failure this area's `registry.rs` registers into the
    /// fault set `graph`/`message`/`consequences` consume, with bus power
    /// from `Truth` and the bay overheat trips from this tick's
    /// ventilation.
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
                        .map(|&id| PartitionFaults { failure: faults.get(id) })
                        .collect(),
                    // The catalogue entry's magnitude text describes the
                    // *model field*'s convention (0 unpowered, 1 powered);
                    // the failure itself is the usual 0 healthy .. 1
                    // failed, so arming it de-energises the module just as
                    // losing its bus does.
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

        // ---- Bay cooling, then the supervisors watching it -------------
        // Heat comes from the modules that were up at the end of the last
        // tick (see the module docs on ordering).
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
        for (b, live) in self.bays.iter_mut().enumerate() {
            let ids = &self.ids.bays[b];
            let fans: Vec<Fan> = [0usize, 1]
                .map(|f| Fan {
                    powered: Self::ac_bus_alive(truth, live.fan_bus[f]),
                    faults: FanFaults { failure: faults.get(ids.fans[f]) },
                })
                .to_vec();
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
            // The extract valve is commanded fully open whenever the bay
            // is being ventilated, which is always: there is no cockpit
            // control that shuts it in normal operation.
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

        // ---- Route both networks with this tick's fault set ------------
        let nf = self.network_faults(truth, faults);
        let graph = NetworkGraph::new(&self.topology);

        let module_available: Vec<bool> =
            (0..self.topology.end_systems.len()).map(|i| nf.module(i).is_available()).collect();
        let module_pass_fraction: Vec<f64> =
            (0..self.topology.end_systems.len()).map(|i| nf.module(i).pass_fraction()).collect();
        let module_overheat_trip: Vec<f64> = self.trips.iter().map(|t| t.frac()).collect();

        // `registry.rs`: the network side is available if any end system
        // can still reach any other one on it. A side with only one module
        // left up is not a network.
        let mut network_available = [false; 2];
        for side in NetworkSide::BOTH {
            let up: Vec<usize> =
                (0..self.topology.end_systems.len()).filter(|&i| module_available[i]).collect();
            'pairs: for (n, &a) in up.iter().enumerate() {
                for &b in &up[n + 1..] {
                    if graph.reachable(side, NodeId::End(a), NodeId::End(b), &nf) {
                        network_available[side.index()] = true;
                        break 'pairs;
                    }
                }
            }
        }

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
            module_overheat_trip,
            module_pass_fraction,
            bay_airflow,
            bay_temp_c,
            function_availability,
            function_age_s,
        };
    }

    fn publish(&self, out: &mut dyn FnMut(&str, f64)) {
        let s = &self.snapshot;

        // --- the variables `registry.rs` triggers ECAM alerts on ---------
        out("AFDX_NETWORK_A_AVAILABLE", f64::from(s.network_available[0]));
        out("AFDX_NETWORK_B_AVAILABLE", f64::from(s.network_available[1]));
        for (i, es) in self.topology.end_systems.iter().enumerate() {
            let key = key_safe(es.name);
            out(
                &format!("AVNCS_MODULE_{key}_AVAILABLE"),
                f64::from(s.module_available.get(i).copied().unwrap_or(false)),
            );
        }
        for (b, bay) in self.bays.iter().enumerate() {
            out(
                &format!("AVNCS_{}_AIRFLOW_FRAC", bay.key),
                s.bay_airflow.get(b).copied().unwrap_or(0.0),
            );
        }

        // --- the rest of the network, for the EFB Study pages ------------
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
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Every bus alive: the aircraft powered and the avionics running.
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
        // Both fans running: the forced draught is fully established and
        // the bay sits close to its supply air.
        assert_eq!(published["AVNCS_AVIONICS_BAY_FWD_AIRFLOW_FRAC"], 1.0);
        assert!(published["AVNCS_AVIONICS_BAY_FWD_TEMP_C"] < 60.0);
        // Every function fully redundant: delivered on both networks.
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

    /// `registry.rs`'s module hardware failure: "at 1.0 the module drops
    /// off both AFDX networks entirely; full failure loses every function
    /// hosted on it." NETWORK CPIOM-C1 FAULT reads
    /// `AVNCS_MODULE_CPIOM_C1_AVAILABLE == 0`, and CPIOM-C1 sources the
    /// FWS warnings virtual link the first reference function consumes.
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
        // The rest of the network is still a network.
        assert_eq!(published["AFDX_NETWORK_A_AVAILABLE"], 1.0);
    }

    /// `registry.rs`'s AFDX cable failure: "a full break can partition the
    /// graph if this is the only route between the two nodes." A module's
    /// own attachment cable is exactly that, and cutting it on network A
    /// alone must leave the function running degraded on network B -- the
    /// whole point of the dual network.
    #[test]
    fn cutting_one_networks_attachment_cable_costs_redundancy_not_the_function() {
        let area = LiveAvionicsNetwork::new();
        // CPIOM-C1 is end system 0; find its network A attachment cable.
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

    /// `registry.rs`'s extraction fan and extract valve failures: "only
    /// losing every fan in the bay collapses it to natural convection",
    /// and the stuck valve does the same on its own. The VENT FAULT alert
    /// triggers on `AVNCS_<bay>_AIRFLOW_FRAC == 0`.
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

    /// The full causal chain the ventilation model exists for: lose a
    /// bay's cooling, the bay heats on its modules' own dissipation, their
    /// thermal supervisors trip, and the modules drop off the network --
    /// nothing scripted, just the heat balance running long enough.
    #[test]
    fn a_bay_that_loses_all_cooling_eventually_trips_its_modules_off_the_network() {
        let index = FaultIndex::build();
        let valve = index.id("AVIONICS_BAY_FWD extract valve stuck closed");
        let mut area = LiveAvionicsNetwork::new();
        // A bay with no forced draught at all loses its heat only to
        // natural convection, which this model puts at 1.5 W/K against a
        // 25 kJ/K bay: a genuinely slow thermal node whose time constant
        // is hours, so the trip is an hour away, not a minute. The tick is
        // coarse here for the same reason -- nothing in this chain has a
        // time constant anywhere near a frame.
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
}

