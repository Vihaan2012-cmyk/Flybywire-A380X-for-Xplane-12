#[allow(unused_imports)]
use std::time::Duration;

#[allow(unused_imports)]
use systems::simulation::test::{ReadByName, SimulationTestBed, TestBed, WriteByName};

#[allow(unused_imports)]
use super::tests::{aircraft, arm, computers_healthy, failure_id, run};
#[allow(unused_imports)]
use crate::A380;

const FBW_OWNED_TRIGGER_VARS: &[&str] = deep_systems::deep::integration::failure_audit::PLUGIN_OWNED_TRIGGER_VARS;

const KNOWN_DEAD_OR_BRANCH_VARS: &[&str] = &["CARGO_BULK_SMOKE_DETECTED"];

fn resolved_lvar_name(name: &str) -> String {
    let key = deep_systems::lvar_key(name);
    key.strip_prefix("A32NX_").unwrap_or(&key).to_owned()
}

fn published_lvar_names() -> std::collections::BTreeSet<String> {
    deep_systems::DeepSystems::new().published_names().iter().map(|n| resolved_lvar_name(n)).collect()
}

fn dangling_trigger_vars() -> Vec<(String, u16, String, Vec<String>)> {
    let published = published_lvar_names();
    let mut owned: std::collections::BTreeSet<String> = FBW_OWNED_TRIGGER_VARS.iter().map(|n| resolved_lvar_name(n)).collect();
    owned.extend(KNOWN_DEAD_OR_BRANCH_VARS.iter().map(|n| resolved_lvar_name(n)));
    let registry = deep_systems::deep::registry();
    let mut out = Vec::new();
    for alert in &registry.alerts {
        let vars = deep_systems::deep::integration::failure_audit::trigger_vars(alert);
        let missing: Vec<String> = vars
            .iter()
            .map(|v| resolved_lvar_name(v))
            .filter(|resolved| !published.contains(resolved) && !owned.contains(resolved))
            .collect();
        if !missing.is_empty() {
            out.push((alert.key.clone(), alert.ata, alert.title.clone(), missing));
        }
    }
    out
}

#[test]
fn every_alert_trigger_variable_resolves_to_a_name_the_host_publishes_or_flybywire_owns() {
    let dangling = dangling_trigger_vars();
    if !dangling.is_empty() {
        let mut msg = format!("{} alert(s) read a trigger variable nothing publishes:\n", dangling.len());
        for (key, ata, title, missing) in &dangling {
            msg.push_str(&format!("  ATA{ata} {key} ({title}): missing {}\n", missing.join(", ")));
        }
        let path = std::env::temp_dir().join("ecam_dangling_trigger_vars.txt");
        let _ = std::fs::write(&path, &msg);
        panic!("{msg}\n(also written to {})", path.display());
    }
}

#[test]
fn resolved_lvar_name_strips_a_pre_existing_a32nx_prefix_instead_of_doubling_it() {
    assert_eq!(resolved_lvar_name("A32NX_ENG_1_ANTI_ICE_BLEED_KG_S"), "ENG_1_ANTI_ICE_BLEED_KG_S");
    assert_eq!(resolved_lvar_name("DEEP_APU_EGT"), "DEEP_APU_EGT");
    assert_eq!(resolved_lvar_name("a32nx_mixed_case"), "MIXED_CASE");
}

fn read_var(test_bed: &mut SimulationTestBed<A380>, name: &str) -> f64 {
    ReadByName::<_, f64>::read_by_name(test_bed, &resolved_lvar_name(name))
}

fn cmp_f64(x: f64, cmp: deep_systems::deep::api::Cmp, y: f64) -> bool {
    use deep_systems::deep::api::Cmp::*;
    match cmp {
        Lt => x < y,
        Le => x <= y,
        Gt => x > y,
        Ge => x >= y,
        Eq => (x - y).abs() < 1e-9,
        Ne => (x - y).abs() >= 1e-9,
    }
}

pub(super) fn eval_cond(test_bed: &mut SimulationTestBed<A380>, c: &deep_systems::deep::api::Cond) -> bool {
    use deep_systems::deep::api::Cond;
    match c {
        Cond::Always => true,
        Cond::Var { name, cmp, value } => cmp_f64(read_var(test_bed, name), *cmp, *value),
        Cond::VarVar { a, cmp, b } => {
            let (av, bv) = (read_var(test_bed, a), read_var(test_bed, b));
            cmp_f64(av, *cmp, bv)
        }
        Cond::And(v) => v.iter().all(|c| eval_cond(test_bed, c)),
        Cond::Or(v) => v.iter().any(|c| eval_cond(test_bed, c)),
        Cond::Not(inner) => !eval_cond(test_bed, inner),
    }
}

pub(super) fn fws_powered(test_bed: &mut SimulationTestBed<A380>) -> bool {
    (1..=4).any(|n| ReadByName::<_, bool>::read_by_name(test_bed, &format!("ELEC_AC_{n}_BUS_IS_POWERED")))
}

fn alerts_triggered(test_bed: &mut SimulationTestBed<A380>, phase: Option<deep_systems::deep::api::Phase>) -> Vec<(String, u16, String)> {
    let registry = deep_systems::deep::registry();
    let powered = fws_powered(test_bed);
    let mut out = Vec::new();
    for alert in &registry.alerts {
        if !powered {
            continue;
        }
        if let Some(p) = phase {
            if alert.inhibited_in.contains(&p) {
                continue;
            }
        }
        if eval_cond(test_bed, &alert.trigger) {
            out.push((alert.key.clone(), alert.ata, alert.title.clone()));
        }
    }
    out
}

fn phase_has_no_deep_alert(test_bed: &mut SimulationTestBed<A380>, name: &str, phase: Option<deep_systems::deep::api::Phase>) {
    let fired = alerts_triggered(test_bed, phase);
    assert!(fired.is_empty(), "{name}: a healthy aircraft raised deep ECAM alert(s) a powered, phase-correct FWS would show: {fired:?}");
}

pub(super) fn run_holding_gear_and_door(test_bed: &mut SimulationTestBed<A380>, total: Duration) {
    let frames = (total.as_millis() / 100).max(1) as usize;
    for _ in 0..frames {
        test_bed.write_by_name("GEAR_HANDLE_POSITION", 1.0);
        test_bed.write_by_name("INTERACTIVE POINT OPEN:0", 0.0);
        run(test_bed, 1);
    }
}

const BUSES: [&str; 6] = ["AC_1", "AC_2", "AC_3", "AC_4", "DC_1", "DC_ESS"];

pub(super) fn engines_running(test_bed: &mut SimulationTestBed<A380>, n1: f64, n2: f64, n3: f64) {
    test_bed.write_by_name("AIRCRAFT_PRESET_QUICK_MODE", 1.);
    for n in 1..=4 {
        test_bed.write_by_name(&format!("AUTOTHRUST_N1_COMMANDED:{n}"), n1);
        test_bed.write_by_name(&format!("GENERAL ENG OIL PRESSURE:{n}"), 60.);
        test_bed.write_by_name(&format!("GENERAL ENG OIL TEMPERATURE:{n}"), 80.);
        test_bed.write_by_name(&format!("ENGINE_STATE:{n}"), 1.);
        test_bed.write_by_name(&format!("GENERAL ENG STARTER:{n}"), 1.);
        test_bed.write_by_name(&format!("TURB ENG CORRECTED N1:{n}"), n1);
        test_bed.write_by_name(&format!("ENGINE_N1:{n}"), n1);
        let ff_kg_h = 650. + ((n1 - 20.) / 65.).clamp(0., 1.2) * 7350.;
        test_bed.write_by_name(&format!("ENGINE_FF:{n}"), ff_kg_h);
        test_bed.write_by_name(&format!("TURB ENG CORRECTED N2:{n}"), n2);
        test_bed.write_by_name(&format!("ENGINE_N2:{n}"), n2);
        test_bed.write_by_name(&format!("ENGINE_N2_HEALTHY:{n}"), n2);
        test_bed.write_by_name(&format!("ENGINE_N3:{n}"), n3);
        test_bed.write_by_name(&format!("ENGINE_N3_HEALTHY:{n}"), n3);
    }
}

#[test]
fn a_whole_healthy_flight_raises_no_deep_ecam_alert() {
    use systems::shared::InternationalStandardAtmosphere;
    use uom::si::{f64::Length, length::foot, velocity::knot};

    use deep_systems::deep::api::Phase;

    let mut test_bed = SimulationTestBed::new(A380::new);
    test_bed.set_on_ground(true);
    for id in 1..=4 {
        test_bed.write_by_name(&format!("OVHD_ELEC_BAT_{id}_PB_IS_AUTO"), true);
    }
    test_bed.write_by_name("GEAR_HANDLE_POSITION", 1.0);
    test_bed.write_by_name("INTERACTIVE POINT OPEN:0", 0.0);
    run_holding_gear_and_door(&mut test_bed, Duration::from_secs(60));
    phase_has_no_deep_alert(&mut test_bed, "cold and dark", Some(Phase::ElecPower));

    for i in 1..=4 {
        test_bed.write_by_name(&format!("EXT_PWR_AVAIL:{i}"), true);
        test_bed.write_by_name(&format!("OVHD_ELEC_EXT_PWR_{i}_PB_IS_ON"), true);
    }
    test_bed.write_by_name("CONFIG_ADIRS_IR_ALIGN_TIME", 1.);
    for n in 1..=3 {
        test_bed.write_by_name(&format!("OVHD_ADIRS_IR_{n}_MODE_SELECTOR_KNOB"), 1.);
    }
    run_holding_gear_and_door(&mut test_bed, Duration::from_secs(600));
    phase_has_no_deep_alert(&mut test_bed, "external power, ADIRS aligning", Some(Phase::ElecPower));

    engines_running(&mut test_bed, 20., 65., 70.);
    for i in 1..=4 {
        test_bed.write_by_name(&format!("OVHD_ELEC_EXT_PWR_{i}_PB_IS_ON"), false);
        test_bed.write_by_name(&format!("EXT_PWR_AVAIL:{i}"), false);
    }
    run_holding_gear_and_door(&mut test_bed, Duration::from_secs(600));
    phase_has_no_deep_alert(&mut test_bed, "engines at idle, taxi", Some(Phase::FirstEngineStarted));

    test_bed.set_on_ground(false);
    engines_running(&mut test_bed, 85., 95., 97.);
    for minute in 1..=20 {
        test_bed.set_pressure_altitude(Length::new::<foot>(1750. * minute as f64));
        test_bed.set_ambient_pressure(InternationalStandardAtmosphere::pressure_at_altitude(Length::new::<foot>(1750. * minute as f64)));
        test_bed.set_ambient_temperature(InternationalStandardAtmosphere::temperature_at_altitude(Length::new::<foot>(1750. * minute as f64)));
        test_bed.set_true_airspeed(uom::si::f64::Velocity::new::<knot>(250. + 10. * minute as f64));
        test_bed.set_indicated_airspeed(uom::si::f64::Velocity::new::<knot>((250. + 10. * minute as f64).min(300.)));
        run_holding_gear_and_door(&mut test_bed, Duration::from_secs(60));
    }
    phase_has_no_deep_alert(&mut test_bed, "climb", Some(Phase::Above1500Ft));

    test_bed.set_pressure_altitude(Length::new::<foot>(35000.));
    test_bed.set_ambient_pressure(InternationalStandardAtmosphere::pressure_at_altitude(Length::new::<foot>(35000.)));
    test_bed.set_ambient_temperature(InternationalStandardAtmosphere::temperature_at_altitude(Length::new::<foot>(35000.)));
    test_bed.set_true_airspeed(uom::si::f64::Velocity::new::<knot>(490.));
    test_bed.set_indicated_airspeed(uom::si::f64::Velocity::new::<knot>(300.));
    run_holding_gear_and_door(&mut test_bed, Duration::from_secs(1200));
    phase_has_no_deep_alert(&mut test_bed, "cruise FL350", Some(Phase::Above1500Ft));

    for minute in 1..=20 {
        let alt_ft = 35000. - 1650. * minute as f64;
        test_bed.set_pressure_altitude(Length::new::<foot>(alt_ft.max(0.)));
        test_bed.set_ambient_pressure(InternationalStandardAtmosphere::pressure_at_altitude(Length::new::<foot>(alt_ft.max(0.))));
        test_bed.set_ambient_temperature(InternationalStandardAtmosphere::temperature_at_altitude(Length::new::<foot>(alt_ft.max(0.))));
        test_bed.set_true_airspeed(uom::si::f64::Velocity::new::<knot>((490. - 10. * minute as f64).max(140.)));
        test_bed.set_indicated_airspeed(uom::si::f64::Velocity::new::<knot>((300. - 5. * minute as f64).max(140.)));
        run_holding_gear_and_door(&mut test_bed, Duration::from_secs(60));
    }
    test_bed.set_on_ground(true);
    engines_running(&mut test_bed, 20., 65., 70.);
    test_bed.set_true_airspeed(uom::si::f64::Velocity::new::<knot>(0.));
    test_bed.set_indicated_airspeed(uom::si::f64::Velocity::new::<knot>(0.));
    test_bed.set_pressure_altitude(Length::new::<foot>(0.));
    run_holding_gear_and_door(&mut test_bed, Duration::from_secs(120));
    phase_has_no_deep_alert(&mut test_bed, "landed, taxi in", Some(Phase::Below80Kt));

    let unpowered: Vec<&str> =
        BUSES.into_iter().filter(|bus| !ReadByName::<_, bool>::read_by_name(&mut test_bed, &format!("ELEC_{bus}_BUS_IS_POWERED"))).collect();
    assert!(unpowered.is_empty(), "buses unpowered at end of flight: {unpowered:?}");
}

#[test]
#[ignore]
fn report_every_trigger_over_a_whole_healthy_flight() {
    use std::collections::BTreeMap;
    use systems::shared::InternationalStandardAtmosphere;
    use uom::si::{f64::Length, length::foot, velocity::knot};

    let registry = deep_systems::deep::registry();
    let deep_phase = |p: &deep_systems::deep::api::Phase| -> u32 {
        use deep_systems::deep::api::Phase::*;
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
    let mut triggers: Vec<(String, deep_systems::deep::api::Cond, f64, Vec<u32>)> = registry
        .alerts
        .iter()
        .map(|a| (format!("{} {}", a.key, a.title), a.trigger.clone(), a.confirm_s, a.inhibited_in.iter().map(deep_phase).collect()))
        .collect();
    triggers.extend(
        deep_systems::deep::ecam::fbw::wirings()
            .into_iter()
            .map(|p| (format!("{} {}", p.id, p.title), p.trigger, p.confirm_s, p.inhibit.to_vec())),
    );
    let mut held = vec![0.0f64; triggers.len()];
    let mut fired: BTreeMap<String, Vec<String>> = BTreeMap::new();

    let mut test_bed = SimulationTestBed::new(A380::new);
    super::tests::computers_healthy(&mut test_bed);
    test_bed.set_on_ground(true);
    for id in 1..=4 {
        test_bed.write_by_name(&format!("OVHD_ELEC_BAT_{id}_PB_IS_AUTO"), true);
    }
    test_bed.write_by_name("GEAR_HANDLE_POSITION", 1.0);
    test_bed.write_by_name("INTERACTIVE POINT OPEN:0", 0.0);

    let second = |test_bed: &mut SimulationTestBed<A380>, segment: &str, phase: u32, held: &mut Vec<f64>, fired: &mut BTreeMap<String, Vec<String>>| {
        run_holding_gear_and_door(test_bed, Duration::from_secs(1));
        let powered = fws_powered(test_bed);
        for (k, (key, trigger, confirm, inhibit)) in triggers.iter().enumerate() {
            if powered && !inhibit.contains(&phase) && eval_cond(test_bed, trigger) {
                held[k] += 1.0;
                if held[k] >= confirm.max(1.0) {
                    let list = fired.entry(key.clone()).or_default();
                    if !list.iter().any(|s| s.starts_with(segment)) {
                        list.push(segment.to_owned());
                    }
                }
            } else {
                held[k] = 0.0;
            }
        }
    };

    for _ in 0..60 {
        second(&mut test_bed, "cold and dark", 1, &mut held, &mut fired);
    }
    for i in 1..=4 {
        test_bed.write_by_name(&format!("EXT_PWR_AVAIL:{i}"), true);
        test_bed.write_by_name(&format!("OVHD_ELEC_EXT_PWR_{i}_PB_IS_ON"), true);
    }
    test_bed.write_by_name("CONFIG_ADIRS_IR_ALIGN_TIME", 1.);
    for n in 1..=3 {
        test_bed.write_by_name(&format!("OVHD_ADIRS_IR_{n}_MODE_SELECTOR_KNOB"), 1.);
    }
    for _ in 0..300 {
        second(&mut test_bed, "external power", 1, &mut held, &mut fired);
    }
    engines_running(&mut test_bed, 20., 65., 70.);
    for i in 1..=4 {
        test_bed.write_by_name(&format!("OVHD_ELEC_EXT_PWR_{i}_PB_IS_ON"), false);
        test_bed.write_by_name(&format!("EXT_PWR_AVAIL:{i}"), false);
    }
    for _ in 0..300 {
        second(&mut test_bed, "engines idle, taxi", 2, &mut held, &mut fired);
    }
    test_bed.set_on_ground(false);
    engines_running(&mut test_bed, 85., 95., 97.);
    for s in 1..=1200 {
        let m = s as f64 / 60.;
        let ft = 1750. * m;
        test_bed.set_pressure_altitude(Length::new::<foot>(ft));
        test_bed.set_ambient_pressure(InternationalStandardAtmosphere::pressure_at_altitude(Length::new::<foot>(ft)));
        test_bed.set_ambient_temperature(InternationalStandardAtmosphere::temperature_at_altitude(Length::new::<foot>(ft)));
        test_bed.set_true_airspeed(uom::si::f64::Velocity::new::<knot>(250. + 10. * m));
        test_bed.set_indicated_airspeed(uom::si::f64::Velocity::new::<knot>((250. + 10. * m).min(300.)));
        second(&mut test_bed, "climb", 8, &mut held, &mut fired);
    }
    test_bed.set_true_airspeed(uom::si::f64::Velocity::new::<knot>(490.));
    test_bed.set_indicated_airspeed(uom::si::f64::Velocity::new::<knot>(300.));
    for _ in 0..600 {
        second(&mut test_bed, "cruise FL350", 8, &mut held, &mut fired);
    }
    for s in 1..=1200 {
        let m = s as f64 / 60.;
        let ft = (35000. - 1650. * m).max(0.);
        test_bed.set_pressure_altitude(Length::new::<foot>(ft));
        test_bed.set_ambient_pressure(InternationalStandardAtmosphere::pressure_at_altitude(Length::new::<foot>(ft)));
        test_bed.set_ambient_temperature(InternationalStandardAtmosphere::temperature_at_altitude(Length::new::<foot>(ft)));
        test_bed.set_true_airspeed(uom::si::f64::Velocity::new::<knot>((490. - 10. * m).max(140.)));
        test_bed.set_indicated_airspeed(uom::si::f64::Velocity::new::<knot>((300. - 5. * m).max(140.)));
        second(&mut test_bed, "descent", 8, &mut held, &mut fired);
    }
    test_bed.set_on_ground(true);
    engines_running(&mut test_bed, 20., 65., 70.);
    test_bed.set_true_airspeed(uom::si::f64::Velocity::new::<knot>(0.));
    test_bed.set_indicated_airspeed(uom::si::f64::Velocity::new::<knot>(0.));
    test_bed.set_pressure_altitude(Length::new::<foot>(0.));
    for _ in 0..120 {
        second(&mut test_bed, "landed, taxi in", 11, &mut held, &mut fired);
    }

    std::fs::write("D:/A380/msfs-a380/wiring/flight-alarms.json", serde_json::to_string_pretty(&fired).unwrap()).unwrap();
    println!("whole healthy flight: {} of {} triggers a crew would see", fired.len(), triggers.len());
}

#[test]
fn a_spoofed_gps_walks_off_into_fm_gps_pos_disagree() {
    let disagree = deep_systems::deep::ecam::fbw::wirings().into_iter().find(|p| p.id == 340_800_025).expect("340800025 is wired").trigger;
    let mut test_bed = aircraft();
    run(&mut test_bed, 100);
    assert!(!eval_cond(&mut test_bed, &disagree), "healthy receivers must not disagree");

    assert_eq!(arm(&mut test_bed, failure_id("34_nav.gps_receiver_1", "spoofing") as f64, 1.0), 1.0);
    run(&mut test_bed, 600);
    let early = read_var(&mut test_bed, "DEEP_GPS_1_OFFSET_N_M");
    assert!(!eval_cond(&mut test_bed, &disagree), "a minute of walk-off is far short of 0.5': {early} m");

    run(&mut test_bed, 10_000);
    let late = read_var(&mut test_bed, "DEEP_GPS_1_OFFSET_N_M");
    assert!(read_var(&mut test_bed, "DEEP_GPS_1_VALID") == 1.0, "a spoofed receiver still claims a valid fix");
    assert!(eval_cond(&mut test_bed, &disagree), "1,000 s of walk-off must pass 0.5': {late} m");
    for n in 2..=3 {
        let off = read_var(&mut test_bed, &format!("DEEP_GPS_{n}_OFFSET_N_M"));
        assert!(off.abs() < 100.0, "GPS {n} was not spoofed: {off} m");
    }
}
