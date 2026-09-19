//! One general model of every `circuit.N` line in the A380X's embedded
//! systems.cfg `[ELECTRICAL]` section: its buses, its `CIRCUIT CONNECTION
//! ON:n` pushbutton, and (new here) a breaker-closed state, default closed.
//!
//! `fuel.rs` already had its own narrow version of this (`parse_fuel_circuits`,
//! restricted to `CIRCUIT_FUEL_PUMP`/`CIRCUIT_FUEL_VALVE`, `bus_power_variable`)
//! for the fuel pump/valve circuits (LIGHT-001 analysis, docs/analysis/
//! systems.md "Electrical (ATA 24)"). This module generalises that parser to
//! every circuit type the file has — lights (`CIRCUIT_LIGHT_*`, lights.rs),
//! fuel (`CIRCUIT_FUEL_PUMP`/`_VALVE`, fuel.rs), radios/avionics/gear/wipers
//! and everything else (151 circuits total) — and adds the one thing neither
//! of them had: a per-circuit breaker a caller can pull, independent of the
//! circuit's own connection pushbutton and bus power.
//!
//! `fuel.rs` re-exports [`MSFS_BUSES`] and [`bus_power_variable`] from here
//! instead of keeping its own copy, and consults [`Circuits::breaker_closed`]
//! so a pulled breaker also cuts fuel pumps and valves. `lights.rs` is built
//! on this module directly.
//!
//! Stage 3 (docs/analysis/cockpit-study-cbs.md CB-002, "no physical
//! overhead/avionics-bay circuit-breaker panel is modeled at all") will need
//! 50+ working circuit breakers; [`Circuits::list`] is the candidate
//! inventory that makes that tractable without new cockpit geometry (a
//! Study-panel-only CB page can call [`Circuits::set_breaker`] by
//! `circuit.N` number for any of the 151 entries), and [`Circuits::of_type`]
//! groups them by the same `CIRCUIT_*` type names the file already uses.
//!
//! Where an FBW Rust consumer's power is decided inside the systems crate
//! itself rather than by one of these MSFS circuits (for example the C++
//! computers' `COMPUTER_FAILURES` in failures.rs, or a PRIM/SEC/FCDC bus),
//! a breaker here has nothing to gate yet; stage 3 would need a small
//! `Circuits`-to-`failures::set_active`-style bridge for those, the same way
//! `fuel.rs`/`lights.rs` bridge a breaker to their own consumer instead.
//!
//! The methods that touch variables are generic over
//! `VariableRegistry + SimulatorReaderWriter` (the plugin's `Vars` and the
//! aspects' `TestVars` both implement it), so this module is unit-testable
//! without X-Plane's own bindings.

use std::collections::HashMap;

use systems::simulation::{SimulatorReaderWriter, VariableIdentifier, VariableRegistry};

/// The same embedded file `fuel.rs`'s fuel circuits come from; the
/// `[ELECTRICAL]` section covers every circuit, not just fuel's own.
pub const SYSTEMS_CFG: &str = include_str!(
    "../../fbw-aircraft/fbw-a380x/src/base/flybywire-aircraft-a380-842/SimObjects/AirPlanes/FlyByWire_A380X/attachments/flybywire/Part_Interior_Cockpit/config/systems.cfg"
);

/// FlyByWire's MSFS bus numbers and the buses they follow
/// (a380_systems_wasm lib.rs:66-83), with the systems.cfg names
/// (systems.cfg:304-319). Bus 1 (INFINIBAT) is always powered.
pub const MSFS_BUSES: [(u32, &str, &str); 16] = [
    (2, "AC_BUS_1", "AC_1"),
    (3, "AC_BUS_2", "AC_2"),
    (4, "AC_BUS_3", "AC_3"),
    (5, "AC_BUS_4", "AC_4"),
    (6, "AC_ESS_BUS", "AC_ESS"),
    (7, "AC_ESS_SHED_BUS", "AC_ESS_SHED"),
    (16, "AC_GND_FLT_SVC_BUS", "AC_GND_FLT_SVC"),
    (8, "DC_BUS_1", "DC_1"),
    (9, "DC_BUS_2", "DC_2"),
    (10, "DC_ESS_BUS", "DC_ESS"),
    (11, "DC_APU_BUS", "309PP"),
    (12, "DC_HOT_BUS_1", "DC_HOT_1"),
    (13, "DC_HOT_BUS_2", "DC_HOT_2"),
    (14, "DC_HOT_BUS_ESS", "DC_HOT_3"),
    (15, "DC_HOT_BUS_APU", "DC_HOT_4"),
    (17, "DC_GND_FLT_SVC_BUS", "DC_GND_FLT_SVC"),
];

/// The FlyByWire bus power variable an MSFS bus follows (without the
/// registry's prefix); `None` for bus 1, which is always powered, and for
/// any bus FlyByWire does not tie to one of its own.
pub fn bus_power_variable(msfs_bus: u32) -> Option<String> {
    MSFS_BUSES.iter().find(|(n, _, _)| *n == msfs_bus).map(|(_, _, fbw)| format!("ELEC_{fbw}_BUS_IS_POWERED"))
}

/// One `circuit.N` line of the systems.cfg `[ELECTRICAL]` section, whatever
/// its type: the type name (`CIRCUIT_FUEL_PUMP`, `CIRCUIT_LIGHT_BEACON`,
/// `CIRCUIT_NAV`, ...), the index after its colon (0 when the line has none,
/// e.g. `CIRCUIT_LIGHT_RECOGNITION`, systems.cfg:390 — no circuit in the file
/// uses index 0 for real, so it is a safe "no index" sentinel), the MSFS
/// buses it connects to, and its `Name:` field where it has one.
#[derive(Clone, Debug, PartialEq)]
pub struct CircuitDef {
    pub number: usize,
    pub type_name: String,
    pub index: usize,
    pub buses: Vec<u32>,
    pub name: Option<String>,
    /// The circuit's own real rated wattage (the second `Power:idle,
    /// rated,tripAmps` value, e.g. `Power:3, 5, 20.0` -> `5.0`) -- FBW's own
    /// literal figure for this exact consumer, not a typical/derived
    /// fallback. `None` for a line with no `Power:` field.
    pub rated_w: Option<f64>,
}

/// Every `circuit.N` line of a systems.cfg `[ELECTRICAL]` section, whatever
/// its type. `fuel.rs`'s old `parse_fuel_circuits` was the same parser
/// narrowed to two type names; this is the general form.
pub fn parse_circuits(cfg: &str) -> Vec<CircuitDef> {
    let mut out = Vec::new();
    for line in cfg.lines() {
        let line = line.split(';').next().unwrap_or("").trim();
        let Some((key, value)) = line.split_once('=') else { continue };
        let Some(number) = key.trim().to_ascii_lowercase().strip_prefix("circuit.").and_then(|n| n.parse().ok()) else {
            continue;
        };
        let mut type_name = None;
        let mut index = 0usize;
        let mut buses = Vec::new();
        let mut name = None;
        let mut rated_w = None;
        for field in value.split('#') {
            let Some((k, v)) = field.split_once(':') else { continue };
            match k.trim().to_ascii_lowercase().as_str() {
                "type" => {
                    let v = v.trim();
                    // The type carries its own optional ":index" suffix
                    // (`CIRCUIT_LIGHT_BEACON:1`); split on the first colon
                    // in the value, which is the one right after the type.
                    match v.split_once(':') {
                        Some((n, i)) => {
                            type_name = Some(n.to_string());
                            index = i.trim().parse().unwrap_or(0);
                        }
                        None => type_name = Some(v.to_string()),
                    }
                }
                "connections" => {
                    buses = v.split(',').filter_map(|c| c.trim().strip_prefix("bus.").and_then(|n| n.trim().parse().ok())).collect();
                }
                "name" => name = Some(v.trim().to_string()),
                // `Power:idle, rated, tripAmps` (e.g. "Power:3, 5, 20.0") --
                // the second value is this circuit's own real rated watts.
                "power" => {
                    rated_w = v.split(',').nth(1).and_then(|s| s.trim().parse::<f64>().ok());
                }
                _ => {}
            }
        }
        if let Some(type_name) = type_name {
            out.push(CircuitDef { number, type_name, index, buses, name, rated_w });
        }
    }
    out
}

/// One circuit's live state.
struct CircuitState {
    def: CircuitDef,
    /// One per bus in `def.buses`; `None` stands for bus.1 (INFINIBAT),
    /// which has no power variable and is always powered.
    bus_ids: Vec<Option<VariableIdentifier>>,
    /// `CIRCUIT CONNECTION ON:n`, the cockpit's own pushbutton, MSFS-style
    /// (fuel.rs's fuel circuits use the same variable).
    connection: VariableIdentifier,
    /// `CIRCUIT BREAKER CLOSED:n`, new here: default closed (1), independent
    /// of the connection pushbutton.
    breaker: VariableIdentifier,
}

/// Everything [`Circuits::list`] reports about one circuit, without needing
/// a variable-registry borrow: enough for a Study-panel CB page or the
/// candidate CB inventory (docs/analysis/cockpit-study-cbs.md section 4).
#[derive(Clone, Debug, PartialEq)]
pub struct CircuitInfo {
    pub number: usize,
    pub type_name: String,
    pub index: usize,
    pub name: Option<String>,
    pub buses: Vec<u32>,
    /// The circuit's own real rated wattage from systems.cfg's `Power:`
    /// field (`CircuitDef::rated_w`), when the line has one -- FBW's own
    /// literal figure for this exact consumer, not `physics::electrical::
    /// rated_watts`'s typical/derived fallback table.
    pub rated_w: Option<f64>,
}

/// Breakers the Study panel asked to pull or reset, applied on the next tick.
static TOGGLE_REQUESTS: std::sync::Mutex<Vec<usize>> = std::sync::Mutex::new(Vec::new());

/// Ask for a circuit's breaker to be pulled if closed, reset if pulled.
pub fn request_toggle(number: usize) {
    if let Ok(mut r) = TOGGLE_REQUESTS.lock() {
        r.push(number);
    }
}

/// Test-isolation helper (see `scenarios::reset_global_state`): drops any
/// queued breaker-toggle request left over from a previous test.
#[cfg(any(test, feature = "test-support"))]
pub fn reset_for_tests() {
    if let Ok(mut r) = TOGGLE_REQUESTS.lock() {
        r.clear();
    }
}

/// Every circuit of the embedded systems.cfg, with its breaker state.
pub struct Circuits {
    states: Vec<CircuitState>,
    by_number: HashMap<usize, usize>,
}

impl Circuits {
    /// The embedded `[ELECTRICAL]` section (the same file `fuel.rs` reads),
    /// plus the cockpit's own decorative CB panel breakers ([`PANEL_CB_NODES`],
    /// registered here rather than in `from_cfg` since they are not part of
    /// any `[ELECTRICAL]` section, real or test).
    pub fn new<V: VariableRegistry + SimulatorReaderWriter>(vars: &mut V) -> Self {
        let circuits = Self::from_cfg(vars, SYSTEMS_CFG);
        for node in PANEL_CB_NODES {
            register_panel_breaker(vars, node);
        }
        circuits
    }

    /// For tests, or a different `[ELECTRICAL]` section.
    pub fn from_cfg<V: VariableRegistry + SimulatorReaderWriter>(vars: &mut V, cfg: &str) -> Self {
        let mut states = Vec::new();
        let mut by_number = HashMap::new();
        for def in parse_circuits(cfg) {
            let bus_ids = def.buses.iter().map(|&b| bus_power_variable(b).map(|name| vars.get(name))).collect();
            // MSFS starts every circuit connected; the pushbuttons toggle it
            // (ELECTRICAL_BUS_TO_CIRCUIT_CONNECTION_TOGGLE, key_events.rs).
            let connection = vars.get(format!("CIRCUIT CONNECTION ON:{}", def.number));
            vars.write(&connection, 1.);
            // New: every breaker starts closed.
            let breaker = vars.get(format!("CIRCUIT BREAKER CLOSED:{}", def.number));
            vars.write(&breaker, 1.);
            by_number.insert(def.number, states.len());
            states.push(CircuitState { def, bus_ids, connection, breaker });
        }
        Self { states, by_number }
    }

    /// Every circuit's static definition, for a CB page or the Study panel.
    pub fn list(&self) -> Vec<CircuitInfo> {
        self.states
            .iter()
            .map(|s| CircuitInfo {
                number: s.def.number,
                type_name: s.def.type_name.clone(),
                index: s.def.index,
                name: s.def.name.clone(),
                buses: s.def.buses.clone(),
                rated_w: s.def.rated_w,
            })
            .collect()
    }

    /// Every circuit of one type (`CIRCUIT_LIGHT_BEACON`, `CIRCUIT_FUEL_PUMP`,
    /// ...), in file order.
    pub fn of_type(&self, type_name: &str) -> Vec<CircuitInfo> {
        self.list().into_iter().filter(|c| c.type_name == type_name).collect()
    }

    /// Close (`true`) or open (`false`) one circuit's breaker by its
    /// `circuit.N` number. An open breaker powers nothing downstream of it,
    /// the same as a real one — stage 3's CB page, or a reset-panel-style
    /// control, calls this. A number with no circuit does nothing.
    ///
    /// Not called anywhere yet: no cockpit CB geometry exists to drive it
    /// from (CB-002), and stage 3 owns building the Study CB page that will.
    #[allow(dead_code)]
    /// Apply the breaker toggles asked for since the last tick.
    pub fn apply_requests<V: VariableRegistry + SimulatorReaderWriter>(&mut self, vars: &mut V) {
        let requests = TOGGLE_REQUESTS.lock().map(|mut r| std::mem::take(&mut *r)).unwrap_or_default();
        for number in requests {
            let closed = self.breaker_closed(vars, number);
            self.set_breaker(vars, number, !closed);
        }
    }

    pub fn set_breaker<V: VariableRegistry + SimulatorReaderWriter>(&mut self, vars: &mut V, number: usize, closed: bool) {
        if let Some(&i) = self.by_number.get(&number) {
            vars.write(&self.states[i].breaker, if closed { 1. } else { 0. });
        }
    }

    /// Whether a circuit's breaker is closed; `true` for a number with no
    /// circuit (nothing to gate, so it should not block a caller).
    pub fn breaker_closed<V: VariableRegistry + SimulatorReaderWriter>(&self, vars: &mut V, number: usize) -> bool {
        self.by_number.get(&number).map_or(true, |&i| vars.read(&self.states[i].breaker) != 0.)
    }

    /// Whether a circuit is live right now: its connection pushbutton made,
    /// its breaker closed, and at least one of its buses powered (bus.1,
    /// with no power variable, always counts).
    pub fn powered<V: VariableRegistry + SimulatorReaderWriter>(&self, vars: &mut V, number: usize) -> bool {
        let Some(&i) = self.by_number.get(&number) else { return false };
        let s = &self.states[i];
        vars.read(&s.connection) != 0. && vars.read(&s.breaker) != 0. && s.bus_ids.iter().any(|b| b.map_or(true, |id| vars.read(&id) != 0.))
    }

    /// Whether any of several circuits is live — for a light or other
    /// consumer fed redundantly from more than one circuit/bus, where losing
    /// one still leaves it lit (the model has one visual for the group; see
    /// lights.rs).
    pub fn any_powered<V: VariableRegistry + SimulatorReaderWriter>(&self, vars: &mut V, numbers: &[usize]) -> bool {
        numbers.iter().any(|&n| self.powered(vars, n))
    }
}

/// Synthetic circuit numbers for [`PANEL_CB_NODES`] start here, well above
/// every real `circuit.N` in [`SYSTEMS_CFG`] (max 154 as of this writing), so
/// a panel-only breaker's number can never collide with — or be mistaken
/// for — a real circuit's.
pub const PANEL_ONLY_BASE: usize = 10_000;

/// The A380X cockpit model's own overhead/avionics-bay circuit-breaker panel
/// (`a380_cockpit.gltf`, node names `CB_*`), checked against every
/// [`parse_circuits`] type and `Name:` field in [`SYSTEMS_CFG`]: none match.
/// FlyByWire's simplified MSFS electrical model has no `CIRCUIT_*` for ATC,
/// FMC, CIDS, GCU or any other of these labels — real A380 avionics-bay
/// breakers systems.cfg never represents as their own circuit (CB-002).
///
/// Each still gets its own writable, default-closed breaker dataref here
/// (`CIRCUIT BREAKER CLOSED:<PANEL_ONLY_BASE + index>`) so the modelled
/// geometry (60 `CB_*` nodes minus the 8 unlabelled `CB_EMPTYn` spare/blank
/// positions below) can be made clickable in X-Plane; pulling one currently
/// gates nothing further downstream (nothing in [`Circuits`] knows about a
/// number outside [`SYSTEMS_CFG`], and `powered`/`breaker_closed` already
/// handle an unknown number safely). Wiring a real consumer to one of these
/// is stage 3 work (see the module doc and CB-002), not this list.
///
/// Keep this exact order and spelling in sync with the converter's own copy
/// (`msfs2xp-aircraft/src/main.rs` `PANEL_CB_NODES`) — the two crates don't
/// share code, and the synthetic number only matches if the index does.
pub const PANEL_CB_NODES: &[&str] = &[
    "CB_AESU1",
    "CB_AESU2",
    "CB_AICU1",
    "CB_AICU2",
    "CB_ARPT_NAV",
    "CB_ATC",
    "CB_AVS1",
    "CB_AVS2",
    "CB_BSCS1",
    "CB_BSCS2",
    "CB_CIDS1",
    "CB_CIDS2",
    "CB_CIDS3",
    "CB_CPCS1",
    "CB_CPCS2",
    "CB_DSMS",
    "CB_DTLNKROUTER",
    "CB_ENG1_EIPM2",
    "CB_ENG2_EIPM1",
    "CB_ENG3_EIPM2",
    "CB_ENG4_EIPM1",
    "CB_ESS_TR",
    "CB_FLAPS1",
    "CB_FLAPS2",
    "CB_FMC_A",
    "CB_FMC_B",
    "CB_FMC_C",
    "CB_FQMS1",
    "CB_FQMS2",
    "CB_FWS1",
    "CB_FWS2",
    "CB_GCU",
    "CB_LGCIS1",
    "CB_LGCIS2",
    "CB_NSS_AVNCS",
    "CB_NSS_FLT_OPS",
    "CB_PACK1_CTL",
    "CB_PACK2_CTL",
    "CB_PAX_BBAND",
    "CB_SCS1",
    "CB_SCS2",
    "CB_SDF1",
    "CB_SDF2",
    "CB_SDF3",
    "CB_SLAT2",
    "CB_SLATS1",
    "CB_TCS1",
    "CB_TCS2",
    "CB_TR1",
    "CB_TR_2A",
    "CB_VCS1",
    "CB_VCS2",
];

/// `PANEL_CB_NODES`'s synthetic circuit number for one of its node names, or
/// `None` for a name not in the list (including the `CB_EMPTYn` spares,
/// which are left out on purpose — see the list's own doc comment).
pub fn panel_cb_number(node_name: &str) -> Option<usize> {
    PANEL_CB_NODES.iter().position(|&n| n == node_name).map(|i| PANEL_ONLY_BASE + i)
}

/// Register one [`PANEL_CB_NODES`] breaker's dataref, default closed, the
/// same way a real circuit's breaker starts (see [`Circuits::from_cfg`]).
/// Returns `None` for a name [`panel_cb_number`] does not know.
pub fn register_panel_breaker<V: VariableRegistry + SimulatorReaderWriter>(vars: &mut V, node_name: &str) -> Option<VariableIdentifier> {
    let number = panel_cb_number(node_name)?;
    let id = vars.get(format!("CIRCUIT BREAKER CLOSED:{number}"));
    vars.write(&id, 1.);
    Some(id)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::aspects::test_vars::TestVars;

    #[test]
    fn every_circuit_line_parses() {
        let circuits = parse_circuits(SYSTEMS_CFG);
        // fuel.rs's own test already pins pumps (25) and valves (60); this
        // pins the total so the generic parser and the narrow one agree.
        let pumps = circuits.iter().filter(|c| c.type_name == "CIRCUIT_FUEL_PUMP").count();
        let valves = circuits.iter().filter(|c| c.type_name == "CIRCUIT_FUEL_VALVE").count();
        assert_eq!(pumps, 25);
        assert_eq!(valves, 60);
        // The exterior/interior light circuits systems.md's LIGHT-001 cites.
        let beacons = circuits.iter().filter(|c| c.type_name == "CIRCUIT_LIGHT_BEACON").count();
        assert_eq!(beacons, 2);
        // A line whose Type has no ":index" suffix still parses, with 0.
        let recognition = circuits.iter().find(|c| c.type_name == "CIRCUIT_LIGHT_RECOGNITION").unwrap();
        assert_eq!(recognition.index, 0);
        assert_eq!(recognition.buses, vec![1]);
        assert_eq!(recognition.name.as_deref(), Some("Recognition_Light"));
    }

    #[test]
    fn a_circuit_on_bus_1_is_always_powered() {
        let mut vars = TestVars::default();
        let circuits = Circuits::new(&mut vars);
        // Recognition light: bus.1 (INFINIBAT) only.
        let recognition = circuits.list().into_iter().find(|c| c.type_name == "CIRCUIT_LIGHT_RECOGNITION").unwrap();
        assert!(circuits.powered(&mut vars, recognition.number));
    }

    #[test]
    fn a_circuit_on_a_real_bus_needs_that_bus_powered() {
        let mut vars = TestVars::default();
        let circuits = Circuits::new(&mut vars);
        let beacon = circuits.list().into_iter().find(|c| c.type_name == "CIRCUIT_LIGHT_BEACON" && c.index == 1).unwrap();
        assert_eq!(beacon.buses, vec![2]);
        assert!(!circuits.powered(&mut vars, beacon.number), "unpowered before the bus is fed");
        // Same lookup `Circuits` itself uses, so it lands on the identifier
        // `vars.set`'s unprefixed write would miss (`vars.get` on a
        // space-free name prefixes it with A32NX_, the way the plugin's own
        // `Vars` does for the aircraft's own variables).
        let id = vars.get("ELEC_AC_1_BUS_IS_POWERED".to_string());
        vars.write(&id, 1.);
        assert!(circuits.powered(&mut vars, beacon.number));
    }

    #[test]
    fn a_pulled_breaker_cuts_power_even_with_the_bus_alive() {
        let mut vars = TestVars::default();
        let mut circuits = Circuits::new(&mut vars);
        let recognition = circuits.list().into_iter().find(|c| c.type_name == "CIRCUIT_LIGHT_RECOGNITION").unwrap();
        assert!(circuits.powered(&mut vars, recognition.number));
        circuits.set_breaker(&mut vars, recognition.number, false);
        assert!(!circuits.breaker_closed(&mut vars, recognition.number));
        assert!(!circuits.powered(&mut vars, recognition.number));
        circuits.set_breaker(&mut vars, recognition.number, true);
        assert!(circuits.powered(&mut vars, recognition.number));
    }

    #[test]
    fn disconnecting_a_circuit_also_cuts_it() {
        let mut vars = TestVars::default();
        let circuits = Circuits::new(&mut vars);
        let recognition = circuits.list().into_iter().find(|c| c.type_name == "CIRCUIT_LIGHT_RECOGNITION").unwrap();
        vars.set("CIRCUIT CONNECTION ON:26", 0.);
        assert!(!circuits.powered(&mut vars, recognition.number));
    }

    #[test]
    fn a_breaker_for_an_unknown_number_does_not_panic_and_reads_closed() {
        let mut vars = TestVars::default();
        let mut circuits = Circuits::new(&mut vars);
        circuits.set_breaker(&mut vars, 999_999, false);
        assert!(circuits.breaker_closed(&mut vars, 999_999));
        assert!(!circuits.powered(&mut vars, 999_999));
    }

    #[test]
    fn of_type_groups_by_the_files_own_type_names() {
        let mut vars = TestVars::default();
        let circuits = Circuits::new(&mut vars);
        let taxi = circuits.of_type("CIRCUIT_LIGHT_TAXI");
        assert_eq!(taxi.len(), 3);
        assert!(taxi.iter().map(|c| c.index).collect::<Vec<_>>().contains(&1));
    }

    #[test]
    fn bus_power_variable_matches_fuel_rs() {
        assert_eq!(bus_power_variable(10).as_deref(), Some("ELEC_DC_ESS_BUS_IS_POWERED"));
        assert_eq!(bus_power_variable(11).as_deref(), Some("ELEC_309PP_BUS_IS_POWERED"));
        assert_eq!(bus_power_variable(1), None);
    }

    #[test]
    fn panel_cb_nodes_has_no_duplicates_and_no_empties() {
        let mut sorted = PANEL_CB_NODES.to_vec();
        sorted.sort_unstable();
        sorted.dedup();
        assert_eq!(sorted.len(), PANEL_CB_NODES.len(), "duplicate CB node name");
        assert_eq!(PANEL_CB_NODES.len(), 52, "60 CB_* nodes in a380_cockpit.gltf minus the 8 CB_EMPTYn spares");
        assert!(!PANEL_CB_NODES.iter().any(|n| n.contains("EMPTY")), "spare positions are not real breakers");
    }

    #[test]
    fn panel_cb_labels_match_no_real_systems_cfg_circuit() {
        // The finding this module's doc comment cites: none of the cockpit's
        // own CB panel labels name a real MSFS circuit, by type or by Name:.
        let circuits = parse_circuits(SYSTEMS_CFG);
        for &node in PANEL_CB_NODES {
            let label = node.trim_start_matches("CB_");
            assert!(
                !circuits.iter().any(|c| c.type_name.eq_ignore_ascii_case(label) || c.name.as_deref().is_some_and(|n| n.eq_ignore_ascii_case(label))),
                "{node} unexpectedly matches a real circuit"
            );
        }
    }

    #[test]
    fn panel_cb_number_is_stable_and_out_of_the_real_range() {
        assert_eq!(panel_cb_number("CB_ATC"), Some(PANEL_ONLY_BASE + 5));
        assert_eq!(panel_cb_number("CB_EMPTY1"), None, "spare position, not in the list");
        assert_eq!(panel_cb_number("CB_NOT_A_REAL_NODE"), None);
        let max_real = parse_circuits(SYSTEMS_CFG).iter().map(|c| c.number).max().unwrap_or(0);
        assert!(PANEL_ONLY_BASE > max_real);
    }

    #[test]
    fn a_panel_breaker_gets_a_real_writable_dataref_default_closed() {
        let mut vars = TestVars::default();
        let circuits = Circuits::new(&mut vars);
        let id = register_panel_breaker(&mut vars, "CB_FMC_A").expect("CB_FMC_A is in PANEL_CB_NODES");
        assert_eq!(vars.read(&id), 1., "starts closed like a real breaker");
        // Registering twice returns the same identifier (VariableRegistry::get
        // is idempotent by name) and does not reset a pulled breaker.
        vars.write(&id, 0.);
        let again = register_panel_breaker(&mut vars, "CB_FMC_A").unwrap();
        assert_eq!(again, id);
        // circuits.rs's own generic lookups treat the panel-only number the
        // same as any number outside SYSTEMS_CFG: nothing to gate, so
        // breaker_closed reports "closed" (no circuit to hold open) and
        // powered stays false (no circuit to power) regardless of the pull.
        let number = panel_cb_number("CB_FMC_A").unwrap();
        assert!(circuits.breaker_closed(&mut vars, number));
        assert!(!circuits.powered(&mut vars, number));
    }
}
