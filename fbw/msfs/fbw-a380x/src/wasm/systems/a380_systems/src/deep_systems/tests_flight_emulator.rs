use std::collections::BTreeMap;
use std::time::Duration;

use systems::shared::InternationalStandardAtmosphere;
use systems::simulation::test::{ReadByName, SimulationTestBed, TestBed, WriteByName};
use uom::si::{f64::Length, f64::Velocity, length::foot, velocity::knot};

use super::tests_ecam::{engines_running, eval_cond, fws_powered, run_holding_gear_and_door};
use crate::A380;

const DIR: &str = "D:/A380/msfs-a380/wave5";
const ID: &str = "725896";

const STATE_VARS: &[&str] = &[
    "ELEC_AC_1_BUS_IS_POWERED",
    "ELEC_AC_2_BUS_IS_POWERED",
    "ELEC_AC_3_BUS_IS_POWERED",
    "ELEC_AC_4_BUS_IS_POWERED",
    "ELEC_AC_ESS_BUS_IS_POWERED",
    "ELEC_DC_1_BUS_IS_POWERED",
    "ELEC_DC_2_BUS_IS_POWERED",
    "ELEC_DC_ESS_BUS_IS_POWERED",
    "ELEC_ENG_GEN_1_POTENTIAL",
    "ELEC_ENG_GEN_4_POTENTIAL",
    "HYD_GREEN_SYSTEM_1_SECTION_PRESSURE",
    "HYD_YELLOW_SYSTEM_1_SECTION_PRESSURE",
    "HYD_GREEN_RESERVOIR_LEVEL",
    "HYD_YELLOW_RESERVOIR_LEVEL",
    "PNEU_ENG_1_PRECOOLER_OUTLET_PRESSURE",
    "PNEU_ENG_4_PRECOOLER_OUTLET_PRESSURE",
    "PNEU_ENG_1_PR_VALVE_OPEN",
    "PNEU_XBLEED_VALVE_L_OPEN_AMOUNT",
    "PRESS_CABIN_ALTITUDE_B1",
    "FUEL_TANK_QUANTITY_2",
    "BRAKE_TEMPERATURE_1",
    "APU_N",
    "OXYGEN_CREW_PRESSURE_PSI",
    "AUTOPILOT_1_ACTIVE",
    "BREAKERS_TRIPPED_NOT_COMMANDED_COUNT",
    "DEEP_FAILURES_ARMED_COUNT",
    "HYD_GREEN_FLUID_TEMP_C",
    "HYD_YELLOW_FLUID_TEMP_C",
];

fn read_if_present(test_bed: &mut SimulationTestBed<A380>, name: &str) -> Option<f64> {
    if test_bed.contains_variable_with_name(name) {
        Some(ReadByName::<_, f64>::read_by_name(test_bed, name))
    } else {
        test_bed.query(|a| a.deep_systems.snapshot()).get(&format!("A32NX_{name}")).copied()
    }
}

fn set_air_data(test_bed: &mut SimulationTestBed<A380>, altitude_ft: f64, tas_kt: f64, cas_kt: f64) {
    let alt = Length::new::<foot>(altitude_ft.max(0.));
    test_bed.set_pressure_altitude(alt);
    test_bed.set_ambient_pressure(InternationalStandardAtmosphere::pressure_at_altitude(alt));
    test_bed.set_ambient_temperature(InternationalStandardAtmosphere::temperature_at_altitude(alt));
    test_bed.set_true_airspeed(Velocity::new::<knot>(tas_kt.max(0.)));
    test_bed.set_indicated_airspeed(Velocity::new::<knot>(cas_kt.max(0.)));
}

fn engines_for(phase: &str) -> (f64, f64, f64) {
    match phase {
        "takeoff" => (95., 97., 98.),
        "climb" => (90., 95., 96.),
        "cruise" => (85., 92., 94.),
        "descent" | "approach" => (30., 70., 75.),
        _ => (20., 65., 70.),
    }
}

struct Triggers {
    list: Vec<(String, deep_systems::deep::api::Cond, f64, Vec<u32>)>,
    held: Vec<f64>,
}

impl Triggers {
    fn new() -> Self {
        use deep_systems::deep::api::Phase::*;
        let deep_phase = |p: &deep_systems::deep::api::Phase| -> u32 {
            match p {
                ElecPower => 1,
                FirstEngineStarted => 2,
                FirstEngineTakeoffPower => 3,
                Above80Kt => 4,
                LiftOff => 6,
                Above1500Ft => 8,
                Below800Ft => 9,
                Touchdown => 10,
                Below80Kt => 11,
                SecondEngineShutdown => 12,
            }
        };
        let registry = deep_systems::deep::registry();
        let mut list: Vec<_> = registry
            .alerts
            .iter()
            .map(|a| (format!("{} {}", a.key, a.title), a.trigger.clone(), a.confirm_s, a.inhibited_in.iter().map(deep_phase).collect()))
            .collect();
        list.extend(
            deep_systems::deep::ecam::fbw::wirings()
                .into_iter()
                .map(|p| (format!("{} {}", p.id, p.title), p.trigger, p.confirm_s, p.inhibit.to_vec())),
        );
        let held = vec![0.0; list.len()];
        Self { list, held }
    }

    fn step(&mut self, test_bed: &mut SimulationTestBed<A380>, fws_phase: u32) -> Vec<String> {
        let powered = fws_powered(test_bed);
        let mut showing = Vec::new();
        for (k, (key, trigger, confirm, inhibit)) in self.list.iter().enumerate() {
            if powered && !inhibit.contains(&fws_phase) && eval_cond(test_bed, trigger) {
                self.held[k] += 1.0;
                if self.held[k] >= confirm.max(1.0) {
                    showing.push(key.clone());
                }
            } else {
                self.held[k] = 0.0;
            }
        }
        showing
    }
}

fn fws_phase(phase: &str, on_ground: bool, altitude_ft: f64) -> u32 {
    match phase {
        "takeoff" if on_ground => 3,
        "takeoff" => 6,
        "climb" | "cruise" | "descent" => 8,
        "approach" if altitude_ft < 800. => 9,
        "approach" => 8,
        "rollout" => 10,
        _ => 11,
    }
}

#[test]
#[ignore]
fn flight_emulator_flies_the_fms_trajectory_and_dumps_the_systems() {
    let fms_path = format!("{DIR}/emulator-fms-{ID}.json");
    let Ok(text) = std::fs::read_to_string(&fms_path) else {
        println!("{fms_path} not found: run the FMS half (FlightEmulator.test.ts) first");
        return;
    };
    let fms: serde_json::Value = serde_json::from_str(&text).unwrap();
    let trajectory = fms["trajectory"].as_array().cloned().unwrap_or_default();
    assert!(!trajectory.is_empty(), "the FMS half wrote no trajectory");

    let mut test_bed = SimulationTestBed::new(A380::new);
    super::tests::computers_healthy(&mut test_bed);
    let mut triggers = Triggers::new();
    let mut dumps: Vec<serde_json::Value> = Vec::new();
    let mut first_seen: BTreeMap<String, String> = BTreeMap::new();
    let mut t_s: u64 = 0;
    let mut segment = String::from("cold and dark");

    let mut dump = |test_bed: &mut SimulationTestBed<A380>, t_s: u64, segment: &str, showing: &[String], extra: serde_json::Value, first_seen: &mut BTreeMap<String, String>| {
        for s in showing {
            first_seen.entry(s.clone()).or_insert_with(|| format!("T+{}m {segment}", t_s / 60));
        }
        let mut state = serde_json::Map::new();
        for name in STATE_VARS {
            if let Some(v) = read_if_present(test_bed, name) {
                state.insert((*name).to_owned(), serde_json::json!((v * 1000.).round() / 1000.));
            }
        }
        dumps.push(serde_json::json!({ "tS": t_s, "segment": segment, "fms": extra, "systems": state, "ecam": showing }));
    };

    test_bed.set_on_ground(true);
    set_air_data(&mut test_bed, fms["ofp"]["originElevationFt"].as_f64().unwrap_or(0.), 0., 0.);
    for id in 1..=4 {
        test_bed.write_by_name(&format!("OVHD_ELEC_BAT_{id}_PB_IS_AUTO"), true);
    }
    let ground = |test_bed: &mut SimulationTestBed<A380>, seconds: u64, name: &str, fws_phase: u32, t_s: &mut u64, triggers: &mut Triggers, dump: &mut dyn FnMut(&mut SimulationTestBed<A380>, u64, &str, &[String], serde_json::Value)| {
        for _ in 0..seconds {
            run_holding_gear_and_door(test_bed, Duration::from_secs(1));
            *t_s += 1;
            let showing = triggers.step(test_bed, fws_phase);
            if *t_s % 60 == 0 {
                dump(test_bed, *t_s, name, &showing, serde_json::Value::Null);
            }
        }
    };
    {
        let mut d = |tb: &mut SimulationTestBed<A380>, t: u64, s: &str, sh: &[String], x: serde_json::Value| dump(tb, t, s, sh, x, &mut first_seen);
        ground(&mut test_bed, 60, "cold and dark", 1, &mut t_s, &mut triggers, &mut d);
        for i in 1..=4 {
            test_bed.write_by_name(&format!("EXT_PWR_AVAIL:{i}"), true);
            test_bed.write_by_name(&format!("OVHD_ELEC_EXT_PWR_{i}_PB_IS_ON"), true);
        }
        test_bed.write_by_name("CONFIG_ADIRS_IR_ALIGN_TIME", 1.);
        for n in 1..=3 {
            test_bed.write_by_name(&format!("OVHD_ADIRS_IR_{n}_MODE_SELECTOR_KNOB"), 1.);
        }
        ground(&mut test_bed, 300, "external power", 1, &mut t_s, &mut triggers, &mut d);
        engines_running(&mut test_bed, 20., 65., 70.);
        for i in 1..=4 {
            test_bed.write_by_name(&format!("OVHD_ELEC_EXT_PWR_{i}_PB_IS_ON"), false);
            test_bed.write_by_name(&format!("EXT_PWR_AVAIL:{i}"), false);
        }
        ground(&mut test_bed, 300, "engines idle, taxi out", 2, &mut t_s, &mut triggers, &mut d);
    }

    let mut last_phase = String::new();
    for sample in &trajectory {
        let phase = sample["phase"].as_str().unwrap_or("cruise").to_owned();
        let on_ground = sample["onGround"].as_bool().unwrap_or(false);
        let altitude_ft = sample["altitudeFt"].as_f64().unwrap_or(0.);
        test_bed.set_on_ground(on_ground);
        set_air_data(&mut test_bed, altitude_ft, sample["tasKt"].as_f64().unwrap_or(0.), sample["casKt"].as_f64().unwrap_or(0.));
        if phase != last_phase {
            let (n1, n2, n3) = engines_for(&phase);
            engines_running(&mut test_bed, n1, n2, n3);
            last_phase = phase.clone();
        }
        segment = phase.clone();
        for _ in 0..10 {
            run_holding_gear_and_door(&mut test_bed, Duration::from_secs(1));
            t_s += 1;
            let showing = triggers.step(&mut test_bed, fws_phase(&phase, on_ground, altitude_ft));
            if t_s % 60 == 0 {
                dump(&mut test_bed, t_s, &segment, &showing, sample.clone(), &mut first_seen);
            }
        }
    }

    test_bed.set_on_ground(true);
    set_air_data(&mut test_bed, fms["ofp"]["destinationElevationFt"].as_f64().unwrap_or(0.), 0., 0.);
    engines_running(&mut test_bed, 20., 65., 70.);
    {
        let mut d = |tb: &mut SimulationTestBed<A380>, t: u64, s: &str, sh: &[String], x: serde_json::Value| dump(tb, t, s, sh, x, &mut first_seen);
        ground(&mut test_bed, 300, "taxi in", 11, &mut t_s, &mut triggers, &mut d);
    }

    std::fs::write(
        format!("{DIR}/emulator-systems-{ID}.json"),
        serde_json::to_string_pretty(&serde_json::json!({ "firstSeen": first_seen, "dumps": dumps })).unwrap(),
    )
    .unwrap();
    let mut md = format!("# Full-flight emulator, systems half (OFP {ID})\n\n{} minutes dumped.\n\n## ECAM triggers that showed (first time)\n\n", dumps.len());
    if first_seen.is_empty() {
        md.push_str("None.\n");
    }
    for (k, when) in &first_seen {
        md.push_str(&format!("- {k}: {when}\n"));
    }
    md.push_str("\n## State every 10 minutes\n\n");
    for d in dumps.iter().step_by(10) {
        md.push_str(&format!("- T+{}m {}: {}\n", d["tS"].as_u64().unwrap_or(0) / 60, d["segment"].as_str().unwrap_or(""), d["systems"]));
    }
    std::fs::write(format!("{DIR}/emulator-systems-{ID}.md"), md).unwrap();
    println!("flight emulator: {} dumps, {} distinct ECAM triggers showed", dumps.len(), first_seen.len());
}
