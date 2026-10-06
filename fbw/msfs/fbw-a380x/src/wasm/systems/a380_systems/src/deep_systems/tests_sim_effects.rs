use systems::simulation::test::{ReadByName, SimulationTestBed, TestBed, WriteByName};
use systems::simulation::Aircraft;
use uom::si::f64::Velocity;
use uom::si::velocity::knot;

use super::tests::{aircraft, run};
use crate::A380;

fn hold_gear_and_ias(test_bed: &mut SimulationTestBed<A380>, gear_down: bool, ias_kt: f64, frames: usize) {
    for _ in 0..frames {
        test_bed.write_by_name("GEAR_HANDLE_POSITION", if gear_down { 1.0 } else { 0.0 });
        test_bed.set_indicated_airspeed(Velocity::new::<knot>(ias_kt));
        run(test_bed, 1);
    }
}

#[test]
fn a_healthy_gear_extension_arms_no_gear_door_jam() {
    let mut test_bed = aircraft();
    hold_gear_and_ias(&mut test_bed, true, 180.0, 50);
    let ids = test_bed.query(|a| a.derived_failure_ids());
    for id in super::sim_effects::GEAR_DOOR_JAM_IDS {
        assert!(!ids.contains(&id), "a healthy hold must not jam the gear doors: {ids:?}");
    }
}

#[test]
fn a_sustained_gear_overspeed_past_ultimate_jams_the_gear_doors() {
    let mut test_bed = aircraft();
    hold_gear_and_ias(&mut test_bed, true, 320.0, 40);
    let ids = test_bed.query(|a| a.derived_failure_ids());
    for id in super::sim_effects::GEAR_DOOR_JAM_IDS {
        assert!(ids.contains(&id), "gear door jam id {id} must be concluded past ultimate: {ids:?}");
    }
}

#[test]
fn a_brief_gear_overspeed_excursion_arms_nothing() {
    let mut test_bed = aircraft();
    hold_gear_and_ias(&mut test_bed, true, 320.0, 15);
    let ids = test_bed.query(|a| a.derived_failure_ids());
    for id in super::sim_effects::GEAR_DOOR_JAM_IDS {
        assert!(!ids.contains(&id), "a brief excursion must not jam the gear doors yet: {ids:?}");
    }
}
