use std::collections::BTreeMap;

use systems::simulation::test::{SimulationTestBed, TestBed};

use crate::A380;

const AIRCRAFT_DIR: &str = "fbw-a380x/src/base/flybywire-aircraft-a380-842/SimObjects/AirPlanes/FlyByWire_A380_842";

fn aircraft_file(name: &str) -> String {
    let root = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../../../..");
    let path = root.join(AIRCRAFT_DIR).join(name);
    std::fs::read_to_string(&path).unwrap_or_else(|e| panic!("read {}: {e}", path.display()))
}

fn sections(text: &str) -> BTreeMap<String, Vec<(String, String)>> {
    let mut out: BTreeMap<String, Vec<(String, String)>> = BTreeMap::new();
    let mut current = String::new();
    for line in text.lines() {
        let line = line.split(';').next().unwrap_or("").trim();
        if line.is_empty() {
            continue;
        }
        if line.starts_with('[') && line.ends_with(']') {
            current = line[1..line.len() - 1].to_string();
            continue;
        }
        if let Some((k, v)) = line.split_once('=') {
            out.entry(current.clone()).or_default().push((k.trim().to_string(), v.trim().to_string()));
        }
    }
    out
}

fn number(v: &str) -> Option<f64> {
    match v.to_ascii_lowercase().as_str() {
        "true" => Some(1.),
        "false" => Some(0.),
        s => s.split(',').next().and_then(|n| n.trim().parse().ok()),
    }
}

fn tank_capacities_gal() -> BTreeMap<u32, f64> {
    let cfg = sections(&aircraft_file("flight_model.cfg"));
    cfg.get("FUEL")
        .into_iter()
        .flatten()
        .filter_map(|(k, v)| {
            let n: u32 = k.strip_prefix("Tank.")?.parse().ok()?;
            let cap = v.split('#').find_map(|f| f.strip_prefix("Capacity:"))?.parse().ok()?;
            Some((n, cap))
        })
        .collect()
}

pub(super) fn write_named(bench: &mut SimulationTestBed<A380>, name: &str, value: f64) {
    let id = bench
        .known_variable_identifier(name)
        .or_else(|| name.strip_prefix("A32NX_").and_then(|s| bench.known_variable_identifier(s)))
        .unwrap_or_else(|| bench.variable_identifier(name));
    bench.write_identifier(&id, value);
}

fn flight_state_writes(flt: &str) -> (Vec<(String, f64)>, bool) {
    let s = sections(&aircraft_file(flt));
    let mut writes: Vec<(String, f64)> = Vec::new();
    for (k, v) in s.get("LocalVars.0").into_iter().flatten() {
        if let Some(x) = number(v) {
            writes.push((k.clone(), x));
        }
    }
    let capacities = tank_capacities_gal();
    for (k, v) in s.get("FuelSystem.0").into_iter().flatten() {
        let Some(x) = number(v) else { continue };
        if let Some(n) = k.strip_prefix("Tank.") {
            let Ok(n) = n.parse::<u32>() else { continue };
            writes.push((format!("FUELSYSTEM TANK LEVEL:{n}"), x));
            if let Some(cap) = capacities.get(&n) {
                writes.push((format!("FUELSYSTEM TANK QUANTITY:{n}"), x * cap));
            }
        } else if let Some(n) = k.strip_prefix("Valve.") {
            writes.push((format!("FUELSYSTEM VALVE SWITCH:{n}"), x));
            writes.push((format!("FUELSYSTEM VALVE OPEN:{n}"), x));
        } else if let Some(n) = k.strip_prefix("Pump.") {
            writes.push((format!("FUELSYSTEM PUMP SWITCH:{n}"), x));
        }
    }
    for n in 1..=4 {
        for (k, v) in s.get(&format!("Engine Parameters.{n}.0")).into_iter().flatten() {
            let Some(x) = number(v) else { continue };
            match k.as_str() {
                "EngineMasterSwitch" => writes.push((format!("GENERAL ENG STARTER:{n}"), x)),
                "Pct N1" => writes.push((format!("TURB ENG CORRECTED N1:{n}"), x)),
                "Pct N2" => writes.push((format!("TURB ENG CORRECTED N2:{n}"), x)),
                "IgnitionSwitch" => writes.push((format!("TURB ENG IGNITION SWITCH EX1:{n}"), x)),
                "GeneratorSwitch" => writes.push((format!("GENERAL ENG MASTER ALTERNATOR:{n}"), x)),
                _ => {}
            }
        }
    }
    for (k, v) in s.get("BleedAir.0").into_iter().flatten() {
        let Some(x) = number(v) else { continue };
        if let Some(n) = k.strip_prefix("EngineAirBleed.") {
            writes.push((format!("BLEED AIR ENGINE:{n}"), x));
        } else if k == "APUAirBleed" {
            writes.push(("BLEED AIR APU".to_string(), x));
        }
    }
    for (k, v) in s.get("Switches.0").into_iter().flatten() {
        let Some(x) = number(v) else { continue };
        if let Some(n) = k.strip_prefix("Potentiometer.") {
            writes.push((format!("LIGHT POTENTIOMETER:{n}"), x));
        }
    }
    for (k, v) in s.get("Controls.0").into_iter().flatten() {
        if let (Some(x), "AntiSkidActive") = (number(v), k.as_str()) {
            writes.push(("ANTISKID BRAKES ACTIVE".to_string(), x));
        }
    }
    let on_ground = s.get("SimVars.0").into_iter().flatten().find(|(k, _)| k == "SimOnGround").and_then(|(_, v)| number(v)).unwrap_or(1.);
    (writes, on_ground > 0.5)
}

pub(super) fn apply_flight_state(bench: &mut SimulationTestBed<A380>, flt: &str) -> usize {
    static CACHE: std::sync::OnceLock<std::sync::Mutex<BTreeMap<String, std::sync::Arc<(Vec<(String, f64)>, bool)>>>> = std::sync::OnceLock::new();
    let cached = {
        let mut cache = CACHE.get_or_init(Default::default).lock().expect("flight state cache");
        cache.entry(flt.to_string()).or_insert_with(|| std::sync::Arc::new(flight_state_writes(flt))).clone()
    };
    let (writes, on_ground) = &*cached;
    bench.set_on_ground(*on_ground);
    if *on_ground {
        use uom::si::{f64::{Length, Velocity}, length::foot, velocity::knot};
        bench.set_indicated_airspeed(Velocity::new::<knot>(0.));
        bench.set_true_airspeed(Velocity::new::<knot>(0.));
        bench.set_pressure_altitude(Length::new::<foot>(0.));
    }
    for (name, value) in writes {
        write_named(bench, name, *value);
    }
    writes.len()
}
