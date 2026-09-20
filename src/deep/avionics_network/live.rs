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
//! ## Where the power comes from
//!
//! Every module's supply and every extraction fan's supply is read from
//! `deep::electrical`'s own solved network, through the published-frame
//! seam `deep::live` provides: that area publishes
//! `ELEC_AC_{1..4}_BUS_POTENTIAL` and `ELEC_DC_{1,2}_BUS_POTENTIAL` from
//! its per-bus solve -- generators, TRs, batteries, contactors, ties,
//! per-load and per-breaker state -- and [`LiveAvionicsNetwork`] reads
//! them back one frame later (see [`LiveAvionicsNetwork::bus_volts`] for
//! the indexing, the lag and what happens when the name is absent).
//!
//! This is what makes an electrical failure reach the avionics: arming a
//! bus short, a generator fault, a contactor or a feeder breaker in
//! `deep::electrical` collapses the bus it really feeds, and the modules
//! and bay fans on that bus go with it, rather than the two models
//! disagreeing about who has power.
//!
//! ## What `Truth` cannot supply yet
//!
//! * **Which bus feeds which module and which fan.** Neither `Truth` nor
//!   `deep::electrical` carries the avionics *load allocation* -- the
//!   published buses say what each bus is doing, not which CPIOM hangs off
//!   which one -- so modules are spread across the two DC buses and each
//!   bay's two fans across two AC buses (see
//!   [`LiveAvionicsNetwork::new`]). That spread is a real design property
//!   -- no bus loss may take out a whole bay -- but the actual A380
//!   allocation is not public; `deep::electrical` modelling each module as
//!   a named load on a named bus would replace it.
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
///
/// These stay this area's own numbers rather than deferring to
/// `deep::electrical`'s `ELEC_<bus>_BUS_IS_POWERED`: that flag is
/// deliberately loose (its own `POWERED_VOLTAGE_FRACTION` is 50 % of
/// nominal -- "is there any real power here at all"), and a CPIOM or a
/// 115 V extraction fan motor is not running on a bus at 60 V. Reading
/// the solved *potential* and applying this area's own equipment
/// threshold to it is strictly finer-grained than reading the flag.
const AC_BUS_ALIVE_V: f64 = 90.0;
const DC_BUS_ALIVE_V: f64 = 18.0;

/// `deep::electrical`'s own published potential for each of the four main
/// AC buses and the two main DC buses, in the order this area (and
/// `Truth::ac_bus_volts` / `Truth::dc_bus_volts`) indexes them.
///
/// The mapping is checked, not assumed. `deep::electrical`'s
/// `network::ALL_BUS_IDS` is ordered `Ac1, Ac2, Ac3, Ac4, AcEss,
/// AcEssShed, AcEmer, AcGndFltSvc, Dc1, Dc2, ...`; its `live::bus_tag`
/// spells those first four `AC_1..AC_4` and the two DC mains `DC_1`,
/// `DC_2`; and `live::publish` emits `ELEC_<tag>_BUS_POTENTIAL` for each.
/// `deep::plugin` fills `Truth::ac_bus_volts[i]` from
/// `ELEC_AC_{i+1}_BUS_POTENTIAL` and `Truth::dc_bus_volts[i]` from
/// `ELEC_DC_{i+1}_BUS_POTENTIAL` (`plugin.rs`'s own id table), so index
/// `i` means the same physical bus on both sides of this seam and the
/// fallback below is genuinely the same bus, not a different one.
const AC_BUS_POTENTIAL_VAR: [&str; 4] =
    ["ELEC_AC_1_BUS_POTENTIAL", "ELEC_AC_2_BUS_POTENTIAL", "ELEC_AC_3_BUS_POTENTIAL", "ELEC_AC_4_BUS_POTENTIAL"];
const DC_BUS_POTENTIAL_VAR: [&str; 2] = ["ELEC_DC_1_BUS_POTENTIAL", "ELEC_DC_2_BUS_POTENTIAL"];

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

    // --- Everything below makes a *single* fault visible on its own,
    // rather than only through a whole network side or a monitored
    // function's rolled-up status (see this module's own doc comment and
    // `docs/deep/BRIEF.md`'s audit finding on this area: 93 of 159
    // registered failures moved nothing published). Each is a direct
    // read of the exact model field the matching failure in `registry.rs`
    // drives, so arming that failure at any magnitude above zero moves
    // its own variable, in every aircraft state, independent of whether
    // it currently lies on a monitored function's path. ---
    /// Per side, per switch (`NetworkTopology::switches[side]` order):
    /// `SwitchFaults::is_available()` and `1 - failure`.
    switch_available: [Vec<bool>; 2],
    switch_health_frac: [Vec<f64>; 2],
    /// Per side, per switch, per port (`NetworkTopology::switch_ports`
    /// order): `1 - port_failure` facing that one neighbour.
    switch_port_health_frac: [Vec<Vec<f64>>; 2],
    /// Per side, per segment (`NetworkTopology::edges` order): `1 - open`.
    segment_health_frac: [Vec<f64>; 2],
    /// Per end system: whether it can currently reach at least one other
    /// up end system on that side -- finer than `network_available`
    /// (which is the *aircraft's* network, not this one module's own
    /// attachment), and what actually distinguishes "this module lost one
    /// side" from "the whole side is down" or "everything is fine".
    module_network_reachable: Vec<[bool; 2]>,
    /// `module_network_reachable` collapsed to a count, 0..2: the number
    /// this module is "one fault from losing the function" language in
    /// the audit means directly -- `1.0` is exactly the state redundancy
    /// monitoring exists to catch.
    module_networks_up: Vec<f64>,
    /// Per end system, per partition (`EndSystemSpec::partitions` order):
    /// `ModuleFaults::partition_available`.
    module_partition_available: Vec<Vec<bool>>,
    /// Per end system, per side: this module's own egress port load as a
    /// fraction of line rate (`graph::PortLoad::offered_bps /
    /// capacity_bps`) -- what a babbling transmitter actually does to its
    /// own port, visible before it ever costs another virtual link a
    /// frame.
    module_port_load_frac: Vec<[f64; 2]>,
    /// Per bay, `[primary, standby]`: each fan's own contribution to the
    /// draught (`ventilation::Fan::output_frac`), before `Bay::step`
    /// takes the *best* of the two -- the number a single fan failure
    /// moves even though the bay's own airflow fraction does not.
    bay_fan_health_frac: Vec<[f64; 2]>,
    /// Per virtual link (`NetworkTopology::virtual_links` order): how many
    /// of the two networks currently carry a real path from its source to
    /// every one of its destinations -- "how many paths does this virtual
    /// link actually have", against the two it is always designed for.
    vl_paths_up: Vec<f64>,
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

/// A key-safe name for whichever kind of node this is, for the per-port and
/// per-cable Var names below (`AVNCS_SWITCH_<sw>_PORT_<neighbour>_...`,
/// `AVNCS_CABLE_<a>_<b>_<side>_...`).
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

    /// Volts on one bus, taken from `deep::electrical`'s own solve.
    ///
    /// `Truth::published` carries what every other area published on the
    /// previous frame (`deep::live`'s designed one-frame lag, which on a
    /// bus whose contactors move in tens of milliseconds and whose
    /// consumers here have seconds-long time constants is well inside the
    /// noise). A name nobody published reads as `None`, never as zero.
    ///
    /// **When the name is absent we fall back to the matching raw `Truth`
    /// field, deliberately.** That field is FlyByWire's own bus potential
    /// for the same bus, and it is a real voltage: 0 V on a dead bus. So
    /// the fallback is *not* "default to powered" -- a genuinely dark
    /// aircraft still reads dark through it, and every failure this change
    /// exists to expose still fires. It covers exactly two cases: the
    /// first frame, before any area has published anything, and a build in
    /// which `deep::electrical` is not among the ticked areas. In both,
    /// the alternative (assume the bus is dead) would take every avionics
    /// module off both networks for a reason that has nothing to do with
    /// the electrical system, which would be a fabricated failure rather
    /// than a conservative default.
    ///
    /// Where both exist the published value wins outright: per
    /// `docs/deep/authority.md` the deep model is authoritative, and the
    /// raw field is FlyByWire's coarse answer *after* it has been told
    /// what this model concluded.
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
    /// from `deep::electrical`'s solved buses and the bay overheat trips
    /// from this tick's ventilation.
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
        let mut bay_fan_health_frac = vec![[0.0_f64; 2]; self.bays.len()];
        for (b, live) in self.bays.iter_mut().enumerate() {
            let ids = &self.ids.bays[b];
            let fans: Vec<Fan> = [0usize, 1]
                .map(|f| Fan {
                    powered: Self::ac_bus_alive(truth, live.fan_bus[f]),
                    faults: FanFaults { failure: faults.get(ids.fans[f]) },
                })
                .to_vec();
            // Each fan's own contribution, before `Bay::step` collapses the
            // two to whichever is better -- see `Snapshot::bay_fan_health_frac`.
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
        //
        // The same walk also records, *per end system*, whether it is one
        // of the pair that made that side available -- `network_available`
        // alone cannot tell "every module still reaches every other one"
        // from "exactly one pair does and everything else is isolated", and
        // it is that per-module answer a single cable or port fault
        // actually moves (see `Snapshot::module_network_reachable`).
        let mut network_available = [false; 2];
        let mut module_network_reachable = vec![[false; 2]; self.topology.end_systems.len()];
        for side in NetworkSide::BOTH {
            let s = side.index();
            let up: Vec<usize> =
                (0..self.topology.end_systems.len()).filter(|&i| module_available[i]).collect();
            for &a in &up {
                for &b in &up {
                    if a != b && graph.reachable(side, NodeId::End(a), NodeId::End(b), &nf) {
                        network_available[s] = true;
                        module_network_reachable[a][s] = true;
                    }
                }
            }
        }
        let module_networks_up: Vec<f64> =
            module_network_reachable.iter().map(|r| r[0] as u8 as f64 + r[1] as u8 as f64).collect();

        // ---- Per-switch, per-port and per-cable health: the exact model
        // field every switch/port/cable failure in `registry.rs` drives,
        // published directly rather than only through whatever function
        // happens to route over it this tick. ----------------------------
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

        // ---- Per-partition availability: independent of the module's own
        // AFDX interface (ARINC 653 fault containment), so a partition
        // failure has to be read here, not off the module's own
        // `_AVAILABLE`. -----------------------------------------------
        let module_partition_available: Vec<Vec<bool>> = self
            .ids
            .modules
            .iter()
            .enumerate()
            .map(|(i, m)| m.partitions.iter().map(|&pid| module_available[i] && faults.get(pid) < 1.0).collect())
            .collect();

        // ---- Per-module egress port load: what a babbling transmitter
        // does to its own attachment port, visible even in a state where it
        // has not yet cost another virtual link a frame (`graph::PortLoad`
        // is not otherwise published anywhere). ---------------------------
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

        // ---- Per virtual link: how many of the two networks actually
        // carry a path from its source to every one of its destinations
        // right now, against the two it is always designed for. ----------
        let vl_paths_up: Vec<f64> = self
            .topology
            .virtual_links
            .iter()
            .map(|vl| {
                NetworkSide::BOTH
                    .iter()
                    .filter(|&&side| {
                        vl.destinations
                            .iter()
                            .all(|&d| graph.reachable(side, NodeId::End(vl.source), NodeId::End(d), &nf))
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

        // --- redundancy made visible: per-switch, per-port, per-cable,
        // per-partition and per-module-attachment state, so a single fault
        // moves something even when it never changes a whole network side
        // or a monitored function's rolled-up status. ---------------------
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
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::deep::live::PublishedFrame;

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

    // -----------------------------------------------------------------
    // Redundancy made visible: the audit found 93 of this area's 159
    // failures moved nothing published, because only a whole network side
    // or a monitored function's rolled-up status was ever read back. The
    // tests below arm exactly those failure classes and check the direct
    // per-component reading each one now drives, plus the pair the brief
    // asks for explicitly: cutting one side of a dual-redundant link must
    // move a published variable while the function stays available, and a
    // second cut must then take it down.
    // -----------------------------------------------------------------

    /// The central pair: a single cable cut costs redundancy, visibly, and
    /// only the second cut (the other side of the same attachment) costs
    /// the function. `cutting_one_networks_attachment_cable_costs_
    /// redundancy_not_the_function` above already proves the function's
    /// own status; this proves the *redundancy* is separately visible even
    /// though that function's own status does not distinguish "one fault
    /// from losing this" from "fully healthy" on its own two-valued read.
    #[test]
    fn cutting_both_sides_of_a_modules_attachment_shows_redundancy_loss_then_function_loss() {
        let area = LiveAvionicsNetwork::new();
        let attach = area.topology.end_systems[0].attach; // CPIOM-C1
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

        // First cut: network A only. The module and the function it feeds
        // both stay up, but the aircraft is now one fault from losing the
        // function, and that has to be a published fact, not something
        // only visible once it is too late.
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

        // Second cut: network B too. Now the module really is off both
        // networks and the function it feeds is genuinely lost.
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

    /// One of the audit's 48 dead port failures: a switch port failure that
    /// sits nowhere near either reference function's path used to move
    /// nothing published at all. It now reads directly off its own port.
    #[test]
    fn a_single_port_failure_moves_that_ports_own_health_reading() {
        let area = LiveAvionicsNetwork::new();
        let switch_name = area.topology.switches[0][0].name.clone(); // "AFDX-A-1"
        let neighbour = area.topology.switch_ports(NetworkSide::A, 0)[0]; // a switch-switch port
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

    /// One of the audit's 24 dead cable failures, and one of its 8 dead
    /// switch failures: both now read directly off their own component
    /// regardless of whether they happen to sit on a monitored path.
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

        // A switch with no end system of its own attached ("AFDX-A-2"),
        // so failing it wholesale cannot be confused with a module fault.
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

    /// One of the audit's 7 dead partition failures: ARINC 653 fault
    /// containment means a crashed partition never touches the module's own
    /// AFDX interface or its sibling partitions, so it has to be read off
    /// its own variable, not inferred from `..._AVAILABLE`.
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

    /// One of the audit's 3 dead babbling-node failures: oversubscription
    /// is a property of the babbling module's own egress port, not of any
    /// one virtual link sharing it, and used to be visible only once it
    /// cost some other traffic a frame.
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

    /// One of the audit's 4 dead bay-fan failures: `Bay::step` takes the
    /// *best* of a bay's two fans, so a single fan failure never moves the
    /// bay's own airflow fraction -- it has to be read off the fan itself.
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

    /// "How many paths a virtual link actually has versus how many it
    /// should have": a healthy dual network carries every VL on both
    /// sides; losing every switch on one side costs it exactly one, and
    /// the designed count never moves.
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

    // -----------------------------------------------------------------
    // The seam into `deep::electrical`.
    // -----------------------------------------------------------------

    /// A module's supply is the electrical model's solved bus, not the raw
    /// `Truth` field. Raw `Truth` says both DC mains sit at nominal; the
    /// electrical area published them dead last frame. The modules must
    /// follow the electrical area.
    #[test]
    fn module_power_follows_the_electrical_areas_solved_bus_not_the_raw_truth_field() {
        let mut truth = powered();
        truth.published = PublishedFrame(BTreeMap::from([
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

    /// The same seam the other way round, which is the half that proves
    /// the raw field really is only a fallback: raw `Truth` claims every
    /// bus is dark, the electrical area published them alive, and the
    /// avionics run.
    #[test]
    fn the_electrical_areas_solved_bus_also_wins_when_the_raw_field_is_the_pessimistic_one() {
        let mut truth = Truth { dt_s: 1.0 / 30.0, ..Truth::default() };
        assert_eq!(truth.dc_bus_volts, [0.0; 2], "setup: the raw field claims a dark aircraft");
        truth.published = PublishedFrame(BTreeMap::from([
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

    /// Losing one solved DC main must take out only the modules on it --
    /// the spread across the two DC buses is the design property
    /// [`LiveAvionicsNetwork::new`] documents, and it only means anything
    /// if the two buses can be lost independently.
    #[test]
    fn losing_one_solved_dc_main_takes_only_the_modules_on_that_bus() {
        let mut truth = powered();
        truth.published = PublishedFrame(BTreeMap::from([
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

    /// The whole point of closing the seam, end to end and with nothing
    /// hand-fed: two real areas in a real `Deep`, and a real registered
    /// `deep::electrical` failure -- a short from the DC 1 busbar to
    /// structure -- taking a CPIOM off both AFDX networks. Before this
    /// change no electrical failure could reach this area at all, because
    /// the raw `Truth` field it used to read is held at nominal in both
    /// runs below and never moves.
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
}
