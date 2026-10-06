use std::collections::BTreeMap;

use crate::deep::api::Registry;
use crate::{BreakerCommand, DeepElectrical, DerivedFailure, ElectricalInputs, Faults};

pub const FRAMES: usize = 600;

pub const CREW_UNIT: &str = "fms-1-normal-bkr";

pub fn inputs(frame: usize) -> ElectricalInputs {
    let mut t = ElectricalInputs { dt_s: 0.05, ..Default::default() };
    let gpu = frame >= 100;
    let engines = frame >= 200;
    let airborne = frame >= 260;
    t.gpu_plugged_in = gpu;
    t.on_ground = !airborne;
    t.environment.tas_ms = if frame >= 250 { 100.0 } else { 0.0 };
    t.environment.sat_c = if airborne { 5.0 } else { 15.0 };
    for i in 0..4 {
        t.engine_running[i] = engines;
        t.engine_n2_frac[i] = if engines { 0.65 } else { 0.0 };
        t.engine_oil_temp_c[i] = if engines { 80.0 } else { 15.0 };
        t.ac_bus_powered[i] = gpu || engines;
        t.ac_bus_volts[i] = if gpu || engines { 115.0 } else { 0.0 };
        t.controls.starter_engaged[i] = (180..200).contains(&frame);
    }
    for i in 0..2 {
        t.dc_bus_powered[i] = gpu || engines;
        t.dc_bus_volts[i] = if gpu || engines { 28.0 } else { 0.0 };
    }
    t.controls.apu_start_pb_on = (120..140).contains(&frame);
    t.apu_running = (140..220).contains(&frame);
    t.controls.gear_door_commanded_open = if (262..280).contains(&frame) { [0.5; 3] } else { [0.0; 3] };
    t.controls.fire_pb_released[3] = frame >= 560;
    t.controls.fire_agent_pb_pressed[3][0] = (570..575).contains(&frame);
    t
}

pub fn commands(frame: usize) -> Option<(&'static str, BreakerCommand)> {
    match frame {
        300 => Some((CREW_UNIT, BreakerCommand::Open)),
        350 => Some((CREW_UNIT, BreakerCommand::Close)),
        _ => None,
    }
}

pub struct FaultIds {
    pub drift: Vec<u64>,
    pub short: Option<u64>,
    pub wiring: Option<u64>,
}

pub fn fault_ids() -> FaultIds {
    let mut breakers = Registry::default();
    crate::deep::breakers::registry::register(&mut breakers);
    let drift = breakers.failures.iter().filter(|f| f.model_field.contains("trip_calibration_drift")).map(|f| f.id).collect();
    let mut electrical = Registry::default();
    crate::deep::electrical::registry::register(&mut electrical);
    let short = electrical.failures.iter().find(|f| f.model_field.contains("short")).map(|f| f.id);
    let mut wiring = Registry::default();
    crate::deep::wiring::registry::register(&mut wiring);
    let wiring = wiring.failures.first().map(|f| f.id);
    FaultIds { drift, short, wiring }
}

pub fn armed(frame: usize, ids: &FaultIds) -> Vec<(u64, f64)> {
    let mut armed = Vec::new();
    if frame >= 400 {
        armed.extend(ids.drift.iter().map(|&id| (id, 1.0)));
    }
    if frame >= 450 {
        armed.extend(ids.short.map(|id| (id, 1.0)));
    }
    if frame >= 480 {
        armed.extend(ids.wiring.map(|id| (id, 0.8)));
    }
    armed
}

pub fn faults(frame: usize, ids: &FaultIds) -> Faults {
    Faults::from_pairs(armed(frame, ids))
}

pub fn digest<'a>(published: &BTreeMap<String, f64>, derived: impl IntoIterator<Item = (u64, f64, &'a str)>) -> u64 {
    let mut h: u64 = 0xcbf2_9ce4_8422_2325;
    let mut eat = |bytes: &[u8]| {
        for &b in bytes {
            h ^= b as u64;
            h = h.wrapping_mul(0x0100_0000_01b3);
        }
    };
    for (name, value) in published {
        eat(name.as_bytes());
        eat(&value.to_bits().to_le_bytes());
    }
    for (id, magnitude, component) in derived {
        eat(&id.to_le_bytes());
        eat(&magnitude.to_bits().to_le_bytes());
        eat(component.as_bytes());
    }
    h
}

pub fn run() -> Vec<u64> {
    let ids = fault_ids();
    let mut deep = DeepElectrical::new();
    let mut out = Vec::with_capacity(FRAMES);
    for frame in 0..FRAMES {
        if let Some((id, command)) = commands(frame) {
            assert!(deep.command(id, command), "{id} refused {command:?}");
        }
        let mut published = BTreeMap::new();
        deep.tick(&inputs(frame), &faults(frame, &ids), &mut |k, v| {
            published.insert(k.to_owned(), v);
        });
        out.push(digest(&published, deep.derived_failures().iter().map(|d| (d.fbw_id, d.magnitude, d.deep_component))));
    }
    out
}
