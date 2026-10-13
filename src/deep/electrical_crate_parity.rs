//! The `deep_electrical` crate, which MSFS runs, against this plugin's own
//! electrical, breaker and wiring areas.
//!
//! The crate compiles these areas' own source files against a cut-down
//! `Truth`, so the physics is shared by construction; what this checks is
//! everything around it: the cut-down input carries every field the areas
//! read, the crate steps and publishes in `Deep::tick`'s order, and the
//! crew commands act on the same trip units. The golden script
//! (`deep_electrical::golden`) runs through both, and every frame's digest
//! of every published value and derived failure must match to the bit.
//!
//! `GOLDEN_RECORD=1` also writes the digests to the crate's
//! `tests/golden/digests.txt`, which the crate's own test (and so a copy
//! vendored into FlyByWire) replays.

use std::collections::{BTreeMap, BTreeSet};

use deep_electrical::golden;
use deep_electrical::BreakerCommand;

use super::breakers::live::BreakersLive;
use super::electrical::live::ElectricalLive;
use super::live::{Area, Faults, PublishedFrame, Truth};
use super::wiring::live::WiringLive;

const DIGESTS: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/crates/deep_electrical/tests/golden/digests.txt");

/// The crate's input, as the plugin's full `Truth` (everything else at
/// its default).
fn truth_from(i: &deep_electrical::ElectricalInputs, published: PublishedFrame) -> Truth {
    let mut t = Truth::default();
    t.dt_s = i.dt_s;
    t.environment.sat_c = i.environment.sat_c;
    t.environment.tas_ms = i.environment.tas_ms;
    t.on_ground = i.on_ground;
    t.engine_running = i.engine_running;
    t.engine_n2_frac = i.engine_n2_frac;
    t.engine_oil_temp_c = i.engine_oil_temp_c;
    t.apu_running = i.apu_running;
    t.ac_bus_volts = i.ac_bus_volts;
    t.dc_bus_volts = i.dc_bus_volts;
    t.ac_bus_powered = i.ac_bus_powered;
    t.dc_bus_powered = i.dc_bus_powered;
    t.gpu_plugged_in = i.gpu_plugged_in;
    let c = &i.controls;
    t.controls.fire_pb_released = c.fire_pb_released;
    t.controls.fire_pb_apu_released = c.fire_pb_apu_released;
    t.controls.fire_agent_pb_pressed = c.fire_agent_pb_pressed;
    t.controls.fire_agent_pb_apu_pressed = c.fire_agent_pb_apu_pressed;
    t.controls.starter_engaged = c.starter_engaged;
    t.controls.gear_door_commanded_open = c.gear_door_commanded_open;
    t.controls.eng_gen_pb_on = c.eng_gen_pb_on;
    t.controls.apu_gen_pb_on = c.apu_gen_pb_on;
    t.controls.bat_pb_auto = c.bat_pb_auto;
    t.controls.apu_start_pb_on = c.apu_start_pb_on;
    t.published = published;
    t
}

/// The golden script through this plugin's own areas, stepped exactly as
/// `Deep::tick` steps them (it is spelled out rather than run through
/// `Deep` only because the crew command needs the breaker area by type).
fn plugin_run() -> Vec<u64> {
    let ids = golden::fault_ids();
    // `all_areas()` order: the electrical constructor clears the board.
    let mut breakers = BreakersLive::new();
    let mut electrical = ElectricalLive::new();
    let mut wiring = WiringLive::new();
    let mut last = PublishedFrame::default();
    let mut digests = Vec::with_capacity(golden::FRAMES);
    for frame in 0..golden::FRAMES {
        if let Some((id, command)) = golden::commands(frame) {
            let unit = breakers.breaker_mut(id).expect("a catalogue id");
            match command {
                BreakerCommand::Open => unit.pull(),
                BreakerCommand::Close => unit.reset().expect("not locked out"),
            }
        }
        let faults = Faults::from_pairs(golden::armed(frame, &ids));
        let truth = truth_from(&golden::inputs(frame), std::mem::take(&mut last));
        breakers.tick(&truth, &faults);
        electrical.tick(&truth, &faults);
        wiring.tick(&truth, &faults);
        let areas: [&dyn Area; 3] = [&breakers, &electrical, &wiring];
        let mut derived = Vec::new();
        for area in areas {
            area.derived_failures(&mut |d| {
                if d.magnitude > 0.0 {
                    derived.push(d);
                }
            });
        }
        let mut published = BTreeMap::new();
        for area in areas {
            area.publish(&mut |k, v| {
                published.insert(k.to_owned(), v);
            });
        }
        digests.push(golden::digest(&published, derived.iter().map(|d| (d.fbw_id, d.magnitude, d.deep_component))));
        last = PublishedFrame::from(published);
    }
    digests
}

#[test]
fn the_crate_reproduces_the_plugin_areas_frame_for_frame() {
    let plugin = plugin_run();
    let crate_run = golden::run();
    if let Some(frame) = (0..golden::FRAMES).find(|&f| plugin[f] != crate_run[f]) {
        panic!("deep_electrical diverges from the plugin's own areas at frame {frame} of the golden script");
    }
    if std::env::var_os("GOLDEN_RECORD").is_some() {
        let text: String = plugin.iter().map(|d| format!("{d:016x}\n")).collect();
        std::fs::write(DIGESTS, text).expect("write the golden digests");
    }
}

#[test]
fn the_recorded_golden_digests_are_this_plugins() {
    let recorded = std::fs::read_to_string(DIGESTS).expect("record them: GOLDEN_RECORD=1 cargo test the_crate_reproduces");
    let recorded: Vec<u64> = recorded.lines().map(|l| u64::from_str_radix(l, 16).expect("hex digest")).collect();
    assert_eq!(recorded, plugin_run(), "the electrical areas changed: re-record the golden digests (GOLDEN_RECORD=1) and re-sync the crate into FlyByWire");
}

/// Which units feed each FlyByWire consumer `crate::breakers` gates through
/// a `plugin_var`: the deep unit with the same id; for a dual-fed LRU, its
/// `-normal-bkr` and `-2nd-bkr` pair (`deep::breakers::catalog`'s
/// `push_electrical_dual`); or the one deep unit whose id is the same once
/// hyphens and case are ignored (the fire loops: `fire-loop-eng-1-A` here,
/// `fire-loop-eng1-A` in the deep catalogue, both "FIRE DET ENG 1 LOOP A"
/// on the same detection loop). A gate with no such unit is not driven.
fn gates_by_rule() -> (Vec<(&'static str, Vec<&'static str>)>, Vec<(&'static str, &'static str)>) {
    let deep_ids: BTreeSet<&'static str> = super::breakers::catalog::all().iter().map(|d| d.id).collect();
    let mut gates = Vec::new();
    let mut unmapped = Vec::new();
    for def in crate::breakers::catalog() {
        let Some(var) = def.plugin_var else { continue };
        let units: Vec<&'static str> = if let Some(&id) = deep_ids.get(def.id) {
            vec![id]
        } else {
            let pair: Vec<&'static str> = ["-normal-bkr", "-2nd-bkr"].iter().filter_map(|s| deep_ids.get(format!("{}{s}", def.id).as_str()).copied()).collect();
            if pair.is_empty() {
                let squash = |id: &str| id.replace('-', "").to_ascii_lowercase();
                let same: Vec<&'static str> = deep_ids.iter().copied().filter(|d| squash(d) == squash(def.id)).collect();
                assert!(same.len() <= 1, "{} matches several deep units: {same:?}", def.id);
                same
            } else {
                pair
            }
        };
        if units.is_empty() {
            unmapped.push((def.id, var));
        } else {
            gates.push((var, units));
        }
    }
    (gates, unmapped)
}

#[test]
fn the_crate_gates_are_this_plugins_gated_consumers() {
    let (expected, unmapped) = gates_by_rule();
    if std::env::var_os("PRINT_GATES").is_some() {
        for (var, units) in &expected {
            println!("GATE {var} {}", units.join(" "));
        }
        for (id, var) in &unmapped {
            println!("UNMAPPED {id} {var}");
        }
    }
    let actual: Vec<(&str, Vec<&str>)> = deep_electrical::GATES.iter().map(|g| (g.variable, g.units.to_vec())).collect();
    assert_eq!(actual, expected);
}
