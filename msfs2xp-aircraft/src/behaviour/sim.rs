//! What the simulator holds before anything is clicked: the aircraft's own
//! flight file and electrical definition, for variables nothing in X-Plane
//! publishes.

use std::collections::HashMap;

use super::bind::dataref;

/// Simulator state from the package's files.
#[derive(Default, Debug, Clone)]
pub struct SimState {
    /// Variables that never change (keyed as `rpn::key`): read as constants.
    pub constants: HashMap<String, f64>,
    /// Start values of datarefs the systems plugin may not publish, by
    /// dataref name.
    pub defaults: HashMap<String, f64>,
    /// Start values of XML-only variables (`L:XMLVAR_*`), kept in Lua.
    pub locals: HashMap<String, f64>,
}

/// `[Section]` -> key -> value of an MSFS .flt or .cfg.
fn sections(text: &str) -> HashMap<String, Vec<(String, String)>> {
    let mut out: HashMap<String, Vec<(String, String)>> = HashMap::new();
    let mut sect = String::new();
    for l in text.lines() {
        let l = l.split(';').next().unwrap_or("").trim();
        if l.starts_with('[') {
            sect = l.trim_matches(['[', ']']).to_ascii_uppercase();
        } else if let Some((k, v)) = l.split_once('=') {
            out.entry(sect.clone()).or_default().push((k.trim().to_string(), v.trim().to_string()));
        }
    }
    out
}

fn value(v: &str) -> Option<f64> {
    match v.trim() {
        t if t.eq_ignore_ascii_case("true") => Some(1.0),
        t if t.eq_ignore_ascii_case("false") => Some(0.0),
        t => t.parse().ok(),
    }
}

/// From the flight the aircraft starts with (`apron.FLT`) and its
/// `systems.cfg`:
/// - `[LocalVars.0]`: L: variables' start values (A32NX_OVHD_INTLT_ANN=1,
///   the annunciator light switch at BRT rather than TEST);
/// - `[Switches.0] Potentiometer.N` (a ratio): LIGHT POTENTIOMETER:N as that
///   ratio, as its dataref holds it;
/// - `[Engine Parameters.N.0] GeneratorSwitch`: GENERAL ENG MASTER
///   ALTERNATOR:N;
/// - `circuit.N = ...#Connections:bus.M`: CIRCUIT CONNECTION ON:N starts 1;
/// - the CIRCUIT_GENERAL_PANEL circuit on a bus fed by a battery of
///   practically infinite capacity (FlyByWire's INFINIBAT, capacity
///   99999999) never loses power: CIRCUIT GENERAL PANEL ON is 1, the
///   `FAILURE` factor MSFS's ASOBO_GT_Emissive_Gauge multiplies every
///   emissive by.
///
/// The APU generator switches start on (APU GENERATOR SWITCH:1 and :2 = 1,
/// the systems plugin's own default for them).
pub fn sim_state(flt: &str, systems_cfg: &str) -> SimState {
    let mut st = SimState::default();
    let f = sections(flt);
    for (k, v) in f.get("LOCALVARS.0").into_iter().flatten() {
        if let Some(x) = value(v) {
            if k.to_ascii_uppercase().starts_with("XMLVAR_") {
                st.locals.insert(format!("L:{k}"), x);
            } else {
                st.defaults.insert(dataref(k), x);
            }
        }
    }
    for (k, v) in f.get("SWITCHES.0").into_iter().flatten() {
        if let (Some(n), Some(x)) = (k.strip_prefix("Potentiometer.").and_then(|n| n.parse::<u32>().ok()), value(v)) {
            st.defaults.insert(dataref(&format!("LIGHT POTENTIOMETER:{n}")), x);
        }
    }
    for e in 1..=8 {
        if let Some(x) = f.get(&format!("ENGINE PARAMETERS.{e}.0")).and_then(|s| s.iter().find(|(k, _)| k == "GeneratorSwitch")).and_then(|(_, v)| value(v)) {
            st.defaults.insert(dataref(&format!("GENERAL ENG MASTER ALTERNATOR:{e}")), x);
        }
    }
    for n in 1..=2 {
        st.defaults.insert(dataref(&format!("APU GENERATOR SWITCH:{n}")), 1.0);
    }

    let f_switches = f.get("SWITCHES.0").cloned().unwrap_or_default();
    let cfg = sections(systems_cfg);
    let elec = cfg.get("ELECTRICAL").cloned().unwrap_or_default();
    let fields = |v: &str| -> HashMap<String, String> {
        v.split('#').filter_map(|p| p.split_once(':').map(|(a, b)| (a.trim().to_ascii_lowercase(), b.trim().to_string()))).collect()
    };
    // Buses fed by a battery that never runs down.
    let infinite: Vec<String> = elec
        .iter()
        .filter(|(k, _)| k.starts_with("battery."))
        .map(|(_, v)| fields(v))
        .filter(|f| f.get("capacity").and_then(|c| c.parse::<f64>().ok()).is_some_and(|c| c >= 1e6))
        .filter_map(|f| f.get("connections").cloned())
        .flat_map(|c| c.split(',').map(|b| b.trim().to_string()).collect::<Vec<_>>())
        .collect();
    for (k, v) in &elec {
        let Some(n) = k.strip_prefix("circuit.").and_then(|n| n.parse::<u32>().ok()) else { continue };
        let f = fields(v);
        let Some(conn) = f.get("connections") else { continue };
        st.defaults.insert(dataref(&format!("CIRCUIT CONNECTION ON:{n}")), 1.0);
        let ty = f.get("type").map(String::as_str).unwrap_or("");
        // Panel, pedestal, glareshield and cabin lights: on as the flight's
        // [Switches.0] sets them, and powered (every bus is fed by the
        // infinite battery), so LIGHT <X> ON follows the switch.
        for (circuit, var, switch) in [
            ("CIRCUIT_LIGHT_PANEL:", "LIGHT PANEL", "PanelLights"),
            ("CIRCUIT_LIGHT_PEDESTAL:", "LIGHT PEDESTRAL", "PedestalLights"),
            ("CIRCUIT_LIGHT_GLARESHIELD:", "LIGHT GLARESHIELD", "GlareshieldLights"),
            ("CIRCUIT_LIGHT_CABIN:", "LIGHT CABIN", "CabinLights"),
        ] {
            let Some(i) = ty.strip_prefix(circuit).and_then(|i| i.trim().parse::<u32>().ok()) else { continue };
            let on = f_switches.iter().find(|(k, _)| k == switch).and_then(|(_, v)| value(v)).unwrap_or(0.0);
            st.defaults.insert(dataref(&format!("{var}:{i}")), on);
            st.defaults.insert(dataref(&format!("{var} ON:{i}")), on);
        }
        if ty.eq_ignore_ascii_case("CIRCUIT_GENERAL_PANEL") && conn.split(',').any(|b| infinite.contains(&b.trim().to_string())) {
            st.constants.insert("A:CIRCUIT GENERAL PANEL ON".into(), 1.0);
        }
    }
    st
}
