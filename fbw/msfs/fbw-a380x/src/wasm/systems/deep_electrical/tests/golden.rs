use std::collections::BTreeMap;

use deep_electrical::{golden, BreakerCommand, DeepElectrical};

#[test]
fn the_golden_script_reproduces_the_x_plane_plugins_digests() {
    let recorded: Vec<u64> = include_str!("golden/digests.txt").lines().map(|l| u64::from_str_radix(l, 16).expect("hex digest")).collect();
    assert_eq!(recorded.len(), golden::FRAMES);
    let run = golden::run();
    if let Some(frame) = (0..golden::FRAMES).find(|&f| recorded[f] != run[f]) {
        panic!("diverges from the X-Plane plugin's digests at frame {frame}: this copy is stale or was edited; re-sync it");
    }
}

#[test]
fn the_golden_script_exercises_the_physics() {
    let ids = golden::fault_ids();
    assert!(!ids.drift.is_empty() && ids.short.is_some() && ids.wiring.is_some());
    let mut deep = DeepElectrical::new();
    let crew = deep.unit_index(golden::CREW_UNIT).expect("the crew unit is catalogued");
    let mut last = BTreeMap::new();
    let mut max_current = 0.0f64;
    for frame in 0..golden::FRAMES {
        if let Some((id, command)) = golden::commands(frame) {
            assert!(deep.command(id, command));
        }
        last.clear();
        deep.tick(&golden::inputs(frame), &golden::faults(frame, &ids), &mut |k, v| {
            last.insert(k.to_owned(), v);
        });
        max_current = max_current.max(deep.current_a_at(crew));
        match frame {
            320 => assert!(deep.is_open_at(crew), "the crew opened it at 300"),
            360 => assert!(!deep.is_open_at(crew), "the crew closed it at 350"),
            399 => assert_eq!(last["BREAKERS_TRIPPED_NOT_COMMANDED_COUNT"], 0.0, "nothing trips on a healthy aircraft"),
            _ => {}
        }
    }
    assert!(max_current > 0.0, "the crew unit's load drew current");
    assert!(last["BREAKERS_TRIPPED_NOT_COMMANDED_COUNT"] > 0.0, "the armed faults tripped units");
    let _ = BreakerCommand::Open;
}
