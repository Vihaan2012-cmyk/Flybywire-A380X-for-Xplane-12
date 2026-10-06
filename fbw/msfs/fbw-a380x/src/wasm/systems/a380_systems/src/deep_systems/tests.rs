use std::time::Duration;

use systems::simulation::test::{ReadByName, SimulationTestBed, TestBed, WriteByName};
use systems::simulation::Aircraft;

use super::MAX_ARMED;
use crate::A380;

pub(super) fn computers_healthy(test_bed: &mut SimulationTestBed<A380>) {
    for n in 1..=3 {
        test_bed.write_by_name(&format!("PRIM_{n}_HEALTHY"), true);
        test_bed.write_by_name(&format!("SEC_{n}_HEALTHY"), true);
    }
}

pub(super) fn aircraft() -> SimulationTestBed<A380> {
    let mut test_bed = SimulationTestBed::new(A380::new);
    test_bed.write_by_name("IS_READY", 1.);
    computers_healthy(&mut test_bed);
    test_bed.write_by_name("EXT_PWR_AVAIL:1", true);
    test_bed.write_by_name("OVHD_ELEC_EXT_PWR_1_PB_IS_ON", true);
    run(&mut test_bed, 20);
    test_bed
}

pub(super) fn run(test_bed: &mut SimulationTestBed<A380>, frames: usize) {
    for _ in 0..frames {
        test_bed.run_with_delta(Duration::from_millis(100));
    }
}

pub(super) fn derived_failure_type(id: u64) -> Option<systems::failures::FailureType> {
    use systems::failures::FailureType;
    use systems::shared::ElectricalBusType;
    Some(match id {
        24_000 => FailureType::TransformerRectifier(1),
        24_001 => FailureType::TransformerRectifier(2),
        24_002 => FailureType::TransformerRectifier(3),
        24_003 => FailureType::TransformerRectifier(4),
        24_004 => FailureType::StaticInverter,
        24_020 => FailureType::Generator(1),
        24_021 => FailureType::Generator(2),
        24_022 => FailureType::Generator(3),
        24_023 => FailureType::Generator(4),
        24_030 => FailureType::ApuGenerator(1),
        24_031 => FailureType::ApuGenerator(2),
        24_100 => FailureType::ElectricalBus(ElectricalBusType::AlternatingCurrent(1)),
        24_101 => FailureType::ElectricalBus(ElectricalBusType::AlternatingCurrent(2)),
        24_102 => FailureType::ElectricalBus(ElectricalBusType::AlternatingCurrent(3)),
        24_103 => FailureType::ElectricalBus(ElectricalBusType::AlternatingCurrent(4)),
        24_104 => FailureType::ElectricalBus(ElectricalBusType::AlternatingCurrentEssential),
        24_105 => FailureType::ElectricalBus(ElectricalBusType::AlternatingCurrentEssentialShed),
        24_106 => FailureType::ElectricalBus(ElectricalBusType::AlternatingCurrentNamed("247XP")),
        24_107 => FailureType::ElectricalBus(ElectricalBusType::AlternatingCurrentGndFltService),
        24_108 => FailureType::ElectricalBus(ElectricalBusType::DirectCurrent(1)),
        24_109 => FailureType::ElectricalBus(ElectricalBusType::DirectCurrent(2)),
        24_110 => FailureType::ElectricalBus(ElectricalBusType::DirectCurrentEssential),
        24_111 => FailureType::ElectricalBus(ElectricalBusType::DirectCurrentNamed("247PP")),
        24_112 => FailureType::ElectricalBus(ElectricalBusType::DirectCurrentNamed("309PP")),
        24_113 => FailureType::ElectricalBus(ElectricalBusType::DirectCurrentHot(1)),
        24_114 => FailureType::ElectricalBus(ElectricalBusType::DirectCurrentHot(2)),
        24_115 => FailureType::ElectricalBus(ElectricalBusType::DirectCurrentHot(3)),
        24_116 => FailureType::ElectricalBus(ElectricalBusType::DirectCurrentHot(4)),
        24_117 => FailureType::ElectricalBus(ElectricalBusType::DirectCurrentGndFltService),
        34_020 => FailureType::RadioAntennaDirectCoupling(1),
        34_021 => FailureType::RadioAntennaDirectCoupling(2),
        34_022 => FailureType::RadioAntennaDirectCoupling(3),
        21_054 => FailureType::HotAirPositionIndication(1),
        21_055 => FailureType::HotAirPositionIndication(2),
        _ => return None,
    })
}

pub(super) fn run_deriving_failures(test_bed: &mut SimulationTestBed<A380>, frames: usize) {
    for _ in 0..frames {
        for id in test_bed.query(|a| a.derived_failure_ids()) {
            if let Some(failure_type) = derived_failure_type(id) {
                test_bed.fail(failure_type);
            }
        }
        run(test_bed, 1);
    }
}

pub(super) fn failure_id(component: &str, name: &str) -> u64 {
    deep_systems::deep::registry()
        .failures
        .iter()
        .find(|f| f.component == component && f.name.contains(name))
        .unwrap_or_else(|| panic!("no failure {name} on {component}"))
        .id
}

pub(super) fn arm(test_bed: &mut SimulationTestBed<A380>, id: f64, magnitude: f64) -> f64 {
    test_bed.write_by_name("DEEP_FAILURE_CMD_MAGNITUDE", magnitude);
    test_bed.write_by_name("DEEP_FAILURE_CMD_ID", id);
    run(test_bed, 1);
    let consumed: f64 = test_bed.read_by_name("DEEP_FAILURE_CMD_ID");
    assert_eq!(consumed, 0., "the command is consumed");
    test_bed.read_by_name("DEEP_FAILURE_CMD_RESULT")
}

#[test]
fn nothing_trips_on_a_healthy_aircraft_on_ground_power() {
    let mut test_bed = aircraft();
    run(&mut test_bed, 100);
    let tripped: f64 = test_bed.read_by_name("BREAKERS_TRIPPED_NOT_COMMANDED_COUNT");
    assert_eq!(tripped, 0.);
    let open: f64 = test_bed.read_by_name("BREAKERS_OPEN_COUNT");
    assert_eq!(open, 0.);
}

#[test]
fn the_efb_opens_and_closes_a_unit_and_its_load_follows() {
    let mut test_bed = aircraft();
    let current: f64 = test_bed.read_by_name("BKR_FMS_1_NORMAL_BKR_CURRENT_A");
    assert!(current > 0., "the FMS draws current on ground power");

    test_bed.write_by_name("BKR_FMS_1_NORMAL_BKR_CMD", 1.);
    run(&mut test_bed, 4);
    let open: f64 = test_bed.read_by_name("BKR_FMS_1_NORMAL_BKR_OPEN");
    let status: f64 = test_bed.read_by_name("BKR_FMS_1_NORMAL_BKR_STATUS");
    let current: f64 = test_bed.read_by_name("BKR_FMS_1_NORMAL_BKR_CURRENT_A");
    let cmd: f64 = test_bed.read_by_name("BKR_FMS_1_NORMAL_BKR_CMD");
    assert_eq!(open, 1.);
    assert_eq!(status, 1., "open by command, not tripped");
    assert_eq!(current, 0.);
    assert_eq!(cmd, 0., "the command is consumed");

    test_bed.write_by_name("BKR_FMS_1_NORMAL_BKR_CMD", 2.);
    run(&mut test_bed, 4);
    let open: f64 = test_bed.read_by_name("BKR_FMS_1_NORMAL_BKR_OPEN");
    let current: f64 = test_bed.read_by_name("BKR_FMS_1_NORMAL_BKR_CURRENT_A");
    assert_eq!(open, 0.);
    assert!(current > 0.);
}

#[test]
fn an_open_unit_cuts_its_flybywire_consumer_and_a_dual_fed_one_needs_both() {
    let mut test_bed = aircraft();
    let gate: f64 = test_bed.read_by_name("ELEC_PUMP_GA_BREAKER_OPEN");
    assert_eq!(gate, 0.);
    test_bed.write_by_name("BKR_HYD_EPUMP_GA_CMD", 1.);
    run(&mut test_bed, 2);
    let gate: f64 = test_bed.read_by_name("ELEC_PUMP_GA_BREAKER_OPEN");
    assert_eq!(gate, 1.);

    test_bed.write_by_name("BKR_LGCIU_1_NORMAL_BKR_CMD", 1.);
    run(&mut test_bed, 2);
    let gate: f64 = test_bed.read_by_name("ELEC_LGCIU_1_BREAKER_OPEN");
    assert_eq!(gate, 0., "LGCIU 1 still has its second feed");
    test_bed.write_by_name("BKR_LGCIU_1_2ND_BKR_CMD", 1.);
    run(&mut test_bed, 2);
    let gate: f64 = test_bed.read_by_name("ELEC_LGCIU_1_BREAKER_OPEN");
    assert_eq!(gate, 1.);
}

#[test]
fn a_pulled_reset_button_holds_its_units_open_and_pushing_it_in_closes_them() {
    let mut test_bed = aircraft();
    test_bed.write_by_name("RESET_PANEL_FMC_A", true);
    run(&mut test_bed, 2);
    for key in ["FMS_1_NORMAL_BKR", "FMS_1_2ND_BKR"] {
        let open: f64 = test_bed.read_by_name(&format!("BKR_{key}_OPEN"));
        assert_eq!(open, 1., "{key}");
    }
    test_bed.write_by_name("BKR_FMS_1_NORMAL_BKR_CMD", 2.);
    run(&mut test_bed, 2);
    let open: f64 = test_bed.read_by_name("BKR_FMS_1_NORMAL_BKR_OPEN");
    assert_eq!(open, 1.);

    test_bed.write_by_name("RESET_PANEL_FMC_A", false);
    run(&mut test_bed, 2);
    for key in ["FMS_1_NORMAL_BKR", "FMS_1_2ND_BKR"] {
        let open: f64 = test_bed.read_by_name(&format!("BKR_{key}_OPEN"));
        assert_eq!(open, 0., "{key}");
    }
}

#[test]
fn a_pulled_tr_reset_fails_that_tr_while_it_is_out() {
    let mut test_bed = aircraft();
    assert!(!test_bed.query(|a| a.derived_failure_ids()).contains(&24_000));
    test_bed.write_by_name("RESET_PANEL_TR1", true);
    run(&mut test_bed, 1);
    assert!(test_bed.query(|a| a.derived_failure_ids()).contains(&24_000));
    test_bed.write_by_name("RESET_PANEL_TR1", false);
    run(&mut test_bed, 1);
    assert!(!test_bed.query(|a| a.derived_failure_ids()).contains(&24_000));
}

#[test]
fn the_host_publishes_its_tick_time() {
    let mut test_bed = aircraft();
    run(&mut test_bed, 1);
    let us: f64 = test_bed.read_by_name("DEEP_SYSTEMS_TICK_US");
    assert!(us > 0.);
}

#[test]
fn every_deep_area_runs_in_the_host() {
    let mut test_bed = aircraft();
    run(&mut test_bed, 1);
    let total: f64 = test_bed.read_by_name("BREAKERS_TOTAL");
    assert_eq!(total, 335.);
    let failures = deep_systems::failure_ids();
    assert!(failures.len() > 5000, "{} failures registered", failures.len());
}

#[test]
fn the_efb_arms_a_failure_reads_it_back_and_clears_it() {
    let mut test_bed = aircraft();
    let id = failure_id("22_elec.fcu-1", "high resistance");
    assert_eq!(arm(&mut test_bed, id as f64, 0.6), 1.);
    let count: f64 = test_bed.read_by_name("DEEP_FAILURES_ARMED_COUNT");
    let slot_id: f64 = test_bed.read_by_name("DEEP_FAILURE_ARMED_0_ID");
    let slot_magnitude: f64 = test_bed.read_by_name("DEEP_FAILURE_ARMED_0_MAGNITUDE");
    assert_eq!(count, 1.);
    assert_eq!(slot_id, id as f64);
    assert!((slot_magnitude - 0.6).abs() < 1e-9);

    assert_eq!(arm(&mut test_bed, id as f64, 0.), 1., "magnitude 0 clears");
    let count: f64 = test_bed.read_by_name("DEEP_FAILURES_ARMED_COUNT");
    assert_eq!(count, 0.);
}

#[test]
fn the_efb_is_told_when_it_names_no_failure_or_arms_too_many() {
    let mut test_bed = aircraft();
    assert_eq!(arm(&mut test_bed, 999_999_999., 1.), 3., "no such failure");
    let count: f64 = test_bed.read_by_name("DEEP_FAILURES_ARMED_COUNT");
    assert_eq!(count, 0.);

    let ids: Vec<u64> = deep_systems::failure_ids().into_iter().take(MAX_ARMED + 1).collect();
    for &id in &ids[..MAX_ARMED] {
        assert_eq!(arm(&mut test_bed, id as f64, 0.1), 1.);
    }
    assert_eq!(arm(&mut test_bed, ids[MAX_ARMED] as f64, 0.1), 2., "one too many");
    assert_eq!(arm(&mut test_bed, -1., 0.), 1., "clear all");
    let count: f64 = test_bed.read_by_name("DEEP_FAILURES_ARMED_COUNT");
    assert_eq!(count, 0.);
}

#[test]
fn a_short_armed_from_the_efb_trips_its_breaker() {
    let mut test_bed = aircraft();
    run(&mut test_bed, 20);
    let tripped: f64 = test_bed.read_by_name("BREAKERS_TRIPPED_NOT_COMMANDED_COUNT");
    assert_eq!(tripped, 0.);

    let id = failure_id("22_elec.fcu-1", "short to ground");
    assert_eq!(arm(&mut test_bed, id as f64, 1.), 1.);
    run(&mut test_bed, 50);
    let tripped: f64 = test_bed.read_by_name("BREAKERS_TRIPPED_NOT_COMMANDED_COUNT");
    assert!(tripped >= 1., "a dead short trips a protection unit on its own");
}

#[test]
fn a_slow_leak_armed_from_the_efb_deflates_only_that_legs_tyres() {
    let mut test_bed = aircraft();
    let healthy: f64 = test_bed.read_by_name("TYRE_PRESSURE_PA:1");
    assert!(healthy > 1_000_000., "a tyre starts inflated: {healthy} Pa");

    assert_eq!(arm(&mut test_bed, 32_101., 0.5), 1.);
    run(&mut test_bed, 600);
    let left_wing: Vec<f64> = (1..=22).map(|n| test_bed.read_by_name(&format!("TYRE_LEAKED_FRACTION:{n}"))).collect();
    let leaking = left_wing.iter().filter(|&&f| f > 0.).count();
    assert_eq!(leaking, 4, "the four left wing gear wheels leak, nothing else: {left_wing:?}");
}

#[test]
#[ignore]
fn every_failure_does_something_in_msfs() {
    use deep_systems::deep::integration::failure_audit::{armed_with, baseline, diff_against, profiles, MAGNITUDES};
    use deep_systems::Truth;
    use systems::shared::InternationalStandardAtmosphere;
    use uom::si::{f64::*, length::foot, velocity::knot};

    deep_systems::set_log_quiet(true);

    fn truth_now(test_bed: &mut SimulationTestBed<A380>) -> Truth {
        test_bed.query(|a| a.deep_systems.last_truth.clone()).expect("the host has ticked")
    }
    fn fly_at(test_bed: &mut SimulationTestBed<A380>, altitude: Length, tas_kt: f64) {
        test_bed.set_pressure_altitude(altitude);
        test_bed.set_ambient_pressure(InternationalStandardAtmosphere::pressure_at_altitude(altitude));
        test_bed.set_ambient_temperature(InternationalStandardAtmosphere::temperature_at_altitude(altitude));
        test_bed.set_true_airspeed(Velocity::new::<knot>(tas_kt));
        test_bed.set_indicated_airspeed(Velocity::new::<knot>(tas_kt.min(300.)));
    }
    fn engines(test_bed: &mut SimulationTestBed<A380>, n1: f64, n2: f64, n3: f64) {
        for n in 1..=4 {
            test_bed.write_by_name(&format!("ENGINE_STATE:{n}"), 1.);
            test_bed.write_by_name(&format!("ENGINE_N1:{n}"), n1);
            test_bed.write_by_name(&format!("TURB ENG CORRECTED N1:{n}"), n1);
            test_bed.write_by_name(&format!("TURB ENG CORRECTED N2:{n}"), n2);
            test_bed.write_by_name(&format!("ENGINE_N2:{n}"), n2);
            test_bed.write_by_name(&format!("ENGINE_N2_HEALTHY:{n}"), n2);
            test_bed.write_by_name(&format!("ENGINE_N3:{n}"), n3);
            test_bed.write_by_name(&format!("ENGINE_N3_HEALTHY:{n}"), n3);
        }
    }

    let mut captured: Vec<(&'static str, Truth)> = Vec::new();
    let mut test_bed = SimulationTestBed::new(A380::new);
    computers_healthy(&mut test_bed);
    test_bed.set_on_ground(true);
    for id in 1..=4 {
        test_bed.write_by_name(&format!("OVHD_ELEC_BAT_{id}_PB_IS_AUTO"), true);
        test_bed.write_by_name(&format!("EXT_PWR_AVAIL:{id}"), true);
        test_bed.write_by_name(&format!("OVHD_ELEC_EXT_PWR_{id}_PB_IS_ON"), true);
    }
    test_bed.write_by_name("CONFIG_ADIRS_IR_ALIGN_TIME", 1.);
    for n in 1..=3 {
        test_bed.write_by_name(&format!("OVHD_ADIRS_IR_{n}_MODE_SELECTOR_KNOB"), 1.);
    }
    test_bed.run_multiple_frames(Duration::from_secs(120));
    captured.push(("msfs ground power", truth_now(&mut test_bed)));
    engines(&mut test_bed, 20., 65., 70.);
    for i in 1..=4 {
        test_bed.write_by_name(&format!("OVHD_ELEC_EXT_PWR_{i}_PB_IS_ON"), false);
        test_bed.write_by_name(&format!("EXT_PWR_AVAIL:{i}"), false);
    }
    test_bed.run_multiple_frames(Duration::from_secs(120));
    captured.push(("msfs engines idle", truth_now(&mut test_bed)));
    test_bed.set_on_ground(false);
    engines(&mut test_bed, 85., 95., 97.);
    for minute in 1..=20 {
        fly_at(&mut test_bed, Length::new::<foot>(1750. * minute as f64), 250. + 10. * minute as f64);
        test_bed.run_multiple_frames(Duration::from_secs(60));
    }
    fly_at(&mut test_bed, Length::new::<foot>(35000.), 490.);
    test_bed.run_multiple_frames(Duration::from_secs(120));
    captured.push(("msfs cruise FL350", truth_now(&mut test_bed)));

    let synthetic: Vec<(&'static str, Truth, usize)> = profiles().into_iter().map(|p| (p.name, (p.truth)(), p.frames)).collect();
    let states: Vec<(&'static str, Truth, usize, bool)> = captured
        .into_iter()
        .map(|(n, t)| (n, t, 16, true))
        .chain(synthetic.into_iter().map(|(n, t, f)| (n, t, f, false)))
        .collect();
    let baselines: Vec<_> = states.iter().map(|(_, truth, frames, _)| baseline(truth, &armed_with(0, 0.), *frames)).collect();

    let failures: Vec<_> = deep_systems::deep::registry().failures.into_iter().filter(|f| !deep_systems::msfs_excluded::is_msfs_excluded(f.id)).collect();
    let threads = std::thread::available_parallelism().map(|n| n.get().min(8)).unwrap_or(4);
    type Verdict = (u64, String, String, String, String, Option<(&'static str, f64, bool)>);

    fn field_name(model_field: &str) -> &str {
        model_field.rsplit('.').next().unwrap_or(model_field)
    }
    fn diagnose(v: &Verdict, src_minus_registries: &str) -> String {
        let field = field_name(&v.4);
        if field.len() < 4 {
            return format!("not auto-diagnosed: model field `{field}` is too short to search reliably; needs a human read of `{}`", v.4);
        }
        let occurrences = src_minus_registries.matches(field).count();
        if occurrences == 0 {
            format!("model field `{field}` (from `{}`) is never referenced anywhere in deep_systems outside its own registry -- nothing reads it back", v.4)
        } else {
            format!(
                "model field `{field}` is referenced {occurrences} time(s) elsewhere in deep_systems, but arming this failure moved no published value in any audited state -- likely gated by a Truth input, cockpit command or Cond this harness held constant, or reachable only in a flight phase/profile it doesn't cover"
            )
        }
    }
    struct AreaGroup {
        file: &'static str,
        areas: &'static [&'static str],
    }
    const GROUPS: &[AreaGroup] = &[
        AreaGroup { file: "electrical", areas: &["Electrical"] },
        AreaGroup { file: "hydraulics", areas: &["Hydraulics"] },
        AreaGroup { file: "pneumatic", areas: &["PneumaticDucts", "ThermalZones"] },
        AreaGroup { file: "fire-gear-apu", areas: &["FireIce", "GearStructure", "Apu", "Oxygen"] },
        AreaGroup { file: "engines", areas: &["EngineAccessories", "EngineCore"] },
        AreaGroup { file: "flight-controls", areas: &["FlightControls", "AutoFlight"] },
        AreaGroup { file: "fuel", areas: &["Fuel"] },
        AreaGroup { file: "avionics", areas: &["Sensors", "AvionicsNetwork", "Communications", "Environment", "Cabin"] },
        AreaGroup { file: "breakers", areas: &["Breakers", "Wiring"] },
    ];

    let deep_src_root = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../deep_systems/src");
    fn concat_rs(dir: &std::path::Path, out: &mut String, skip: &[&str]) {
        let Ok(entries) = std::fs::read_dir(dir) else { return };
        for entry in entries.flatten() {
            let path = entry.path();
            if path.is_dir() {
                concat_rs(&path, out, skip);
            } else if path.extension().is_some_and(|e| e == "rs") && !path.file_name().is_some_and(|n| skip.iter().any(|s| n == std::ffi::OsStr::new(s))) {
                if let Ok(s) = std::fs::read_to_string(&path) {
                    out.push_str(&s);
                    out.push('\n');
                }
            }
        }
    }
    let mut src_minus_registries = String::new();
    concat_rs(&deep_src_root, &mut src_minus_registries, &["registry.rs", "all_registry.rs"]);
    let dead_dir = std::path::Path::new("D:/A380/msfs-a380/reports/dead-failures");
    std::fs::create_dir_all(dead_dir).ok();

    let write_reports = |verdicts: &[Verdict], total: usize| {
        let live_msfs = verdicts.iter().filter(|v| matches!(v.5, Some((_, _, true)))).count();
        let live_synthetic_only = verdicts.iter().filter(|v| matches!(v.5, Some((_, _, false)))).count();
        let dead: Vec<&Verdict> = verdicts.iter().filter(|v| v.5.is_none()).collect();
        let mut report = format!(
            "# Deep failure audit, MSFS host (PARTIAL: {}/{total} failures checked so far)\n\n{} failures checked: {} move something in a state captured from the MSFS host, {} only in the harness's synthetic states, {} in none.\n\n",
            verdicts.len(),
            verdicts.len(),
            live_msfs,
            live_synthetic_only,
            dead.len()
        );
        let mut by_area: std::collections::BTreeMap<&str, Vec<&Verdict>> = Default::default();
        for v in &dead {
            by_area.entry(v.1.as_str()).or_default().push(v);
        }
        report.push_str("## Dead, by area (so far)\n\n");
        for (area, vs) in &by_area {
            report.push_str(&format!("### {area} ({})\n\n", vs.len()));
            for v in vs {
                report.push_str(&format!("- {} {} [{}] `{}`\n", v.0, v.2, v.3, v.4));
            }
            report.push('\n');
        }
        let dir = std::path::Path::new("D:/A380/fbw-build/wasm-fs2020/audit");
        std::fs::create_dir_all(dir).ok();
        std::fs::write(dir.join("msfs-audit.md"), &report).ok();

        let mut covered: std::collections::BTreeSet<&str> = Default::default();
        for g in GROUPS {
            let mut vs: Vec<&Verdict> = dead.iter().filter(|v| g.areas.contains(&v.1.as_str())).copied().collect();
            vs.sort_by_key(|v| (v.1.clone(), v.0));
            covered.extend(g.areas.iter());
            let mut out = format!(
                "# Dead failures: {} ({} of {} checked so far; {total} total in the catalogue -- PARTIAL, sweep still running)\n\n{} states (3 captured from this MSFS host, {} synthetic from the X-Plane audit harness), magnitudes {:?}.\n\nEach diagnosis is a heuristic textual search for the model field's name elsewhere in `deep_systems` -- it tells you whether *anything at all* reads the field back, not why the effect doesn't reach a published value. Confirm by reading the file cited in `model_field` before fixing.\n\n",
                g.file,
                vs.len(),
                dead.len(),
                states.len(),
                states.len() - 3,
                MAGNITUDES
            );
            for v in &vs {
                out.push_str(&format!("## {} -- {}\n\n- area: {}\n- component: `{}`\n- model_field: `{}`\n- diagnosis: {}\n\n", v.0, v.2, v.1, v.3, v.4, diagnose(v, &src_minus_registries)));
            }
            std::fs::write(dead_dir.join(format!("{}.md", g.file)), &out).ok();
        }
        let uncovered: Vec<&Verdict> = dead.iter().copied().filter(|v| !covered.contains(v.1.as_str())).collect();
        if !uncovered.is_empty() {
            let mut out = String::from("# Dead failures in areas not named by the coordination brief (PARTIAL)\n\n");
            for v in &uncovered {
                out.push_str(&format!("- {} {} [{}] `{}` -- {}\n", v.0, v.2, v.1, v.4, diagnose(v, &src_minus_registries)));
            }
            std::fs::write(dead_dir.join("uncovered-areas.md"), &out).ok();
        }
    };

    {
        use deep_systems::deep::integration::failure_audit::{alerts_reading_unpublished, trigger_verdicts, Tri};
        let published: std::collections::BTreeSet<String> = deep_systems::deep::live::all_areas().published_names().into_iter().collect();
        let alerts = deep_systems::deep::registry().alerts;
        let unpublished = alerts_reading_unpublished(&alerts, &published);
        let verdicts_ecam = trigger_verdicts(&alerts, &published);
        let mut out = format!("# Dead ECAM alerts\n\n{} alerts read a variable no deep area publishes; of those, the ones below are proven Never (can't fire) or Always (stuck on) once unpublished variables are read as 0.\n\n", unpublished.len());
        for (key, tri, missing) in &verdicts_ecam {
            if matches!(tri, Tri::Never | Tri::Always) {
                out.push_str(&format!("- `{key}` -- {:?}, missing: {}\n", tri, missing.join(", ")));
            }
        }
        out.push_str("\n## All triggers reading an unpublished variable (not all are provably dead)\n\n");
        for u in &unpublished {
            out.push_str(&format!("- `{}` ({}) ATA {} -- missing: {}\n", u.key, u.title, u.ata, u.missing.join(", ")));
        }
        std::fs::write(dead_dir.join("ecam.md"), &out).ok();
    }

    const BATCH: usize = 300;
    let mut verdicts: Vec<Verdict> = Vec::with_capacity(failures.len());
    for batch in failures.chunks(BATCH) {
        let chunk = batch.len().div_ceil(threads).max(1);
        let batch_verdicts: Vec<Verdict> = std::thread::scope(|s| {
            let handles: Vec<_> = batch
                .chunks(chunk)
                .map(|part| {
                    let states = &states;
                    let baselines = &baselines;
                    s.spawn(move || {
                        part.iter()
                            .map(|f| {
                                let mut alive = None;
                                'search: for (i, (name, truth, _, msfs)) in states.iter().enumerate() {
                                    for m in MAGNITUDES {
                                        if !diff_against(&baselines[i], truth, &armed_with(f.id, m)).is_empty() {
                                            alive = Some((*name, m, *msfs));
                                            break 'search;
                                        }
                                    }
                                }
                                (f.id, format!("{:?}", f.area), f.name.clone(), f.component.clone(), f.model_field.clone(), alive)
                            })
                            .collect::<Vec<_>>()
                    })
                })
                .collect();
            handles.into_iter().flat_map(|h| h.join().expect("audit thread")).collect()
        });
        verdicts.extend(batch_verdicts);
        write_reports(&verdicts, failures.len());
        println!("audit progress: {}/{} failures checked, {} dead so far", verdicts.len(), failures.len(), verdicts.iter().filter(|v| v.5.is_none()).count());
    }
    println!("audit complete: {} failures checked", verdicts.len());
}

#[test]
#[ignore]
fn every_breaker_does_something_in_msfs() {
    let mut test_bed = aircraft();
    for id in 1..=4 {
        test_bed.write_by_name(&format!("EXT_PWR_AVAIL:{id}"), true);
        test_bed.write_by_name(&format!("OVHD_ELEC_EXT_PWR_{id}_PB_IS_ON"), true);
        test_bed.write_by_name(&format!("OVHD_ELEC_BAT_{id}_PB_IS_AUTO"), true);
    }
    test_bed.write_by_name("CONFIG_ADIRS_IR_ALIGN_TIME", 1.);
    for n in 1..=3 {
        test_bed.write_by_name(&format!("OVHD_ADIRS_IR_{n}_MODE_SELECTOR_KNOB"), 1.);
    }
    test_bed.run_multiple_frames(Duration::from_secs(120));

    let units: Vec<&'static str> = test_bed.query(|a| a.deep_systems.deep.unit_ids());
    let electrical = |name: &str| {
        name.starts_with("ELEC_")
            || name.starts_with("BKR_")
            || name.starts_with("BREAKERS_")
            || name.starts_with("WIRING_")
            || name.starts_with("ELMS_")
    };
    let mut report = String::new();
    let mut tiers: std::collections::BTreeMap<&str, Vec<String>> = Default::default();
    for unit in &units {
        let key = deep_systems::lvar_key(unit);
        let before = test_bed.query(|a| a.deep_systems.snapshot());
        test_bed.write_by_name(&format!("BKR_{key}_CMD"), 1.);
        run(&mut test_bed, 30);
        let after = test_bed.query(|a| a.deep_systems.snapshot());
        let changed: Vec<&String> = after
            .iter()
            .filter(|(name, value)| before.get(*name).map_or(true, |b| b != *value && !(b.is_nan() && value.is_nan())))
            .map(|(name, _)| name)
            .chain(before.keys().filter(|name| !after.contains_key(*name)))
            .collect();
        let fbw: Vec<&&String> = changed.iter().filter(|n| n.starts_with("GATE ") || n.starts_with("FBW FAILURE ")).collect();
        let deep: Vec<&&String> = changed.iter().filter(|n| !n.starts_with("GATE ") && !n.starts_with("FBW FAILURE ") && !electrical(n)).collect();
        let tier = if !fbw.is_empty() {
            "flybywire"
        } else if !deep.is_empty() {
            "deep"
        } else if !changed.is_empty() {
            "electrical"
        } else {
            "nothing"
        };
        let examples: Vec<&str> = fbw.iter().chain(deep.iter()).take(4).map(|s| s.as_str()).collect();
        tiers.entry(tier).or_default().push(format!("{unit}: {} outputs changed {examples:?}", changed.len()));
        test_bed.write_by_name(&format!("BKR_{key}_CMD"), 2.);
        run(&mut test_bed, 30);
    }
    report.push_str(&format!("# What each protection unit does in MSFS\n\n{} units\n\n", units.len()));
    for (tier, list) in &tiers {
        report.push_str(&format!("- {tier}: {}\n", list.len()));
    }
    for (tier, list) in &tiers {
        report.push_str(&format!("\n## {tier} ({})\n\n", list.len()));
        for line in list {
            report.push_str(&format!("- {line}\n"));
        }
    }
    let dir = std::path::Path::new("D:/A380/fbw-build/wasm-fs2020/audit");
    std::fs::create_dir_all(dir).ok();
    std::fs::write(dir.join("breaker-audit.md"), &report).ok();
    println!("{}", report.lines().take(8).collect::<Vec<_>>().join("\n"));
}

#[test]
fn tyres_do_not_roll_or_heat_once_the_wheels_leave_the_ground() {
    let mut test_bed = aircraft();
    test_bed.set_on_ground(false);
    test_bed.write_by_name("GPS GROUND SPEED", 300.);
    for _ in 0..600 {
        test_bed.run_with_delta(Duration::from_secs(1));
    }
    let armed: f64 = test_bed.read_by_name("DEEP_FAILURES_ARMED_COUNT");
    assert_eq!(armed, 0., "ten minutes of cruise must not burst a tyre");
    let hottest = (1..=22).map(|n| { let t: f64 = test_bed.read_by_name(&format!("TYRE_TEMPERATURE_C:{n}")); t }).fold(f64::MIN, f64::max);
    assert!(hottest < 60., "airborne tyres only soak and cool, hottest {hottest} C");

    let mut rolling = aircraft();
    rolling.set_on_ground(true);
    rolling.write_by_name("GPS GROUND SPEED", 30.);
    for _ in 0..120 {
        rolling.run_with_delta(Duration::from_secs(1));
    }
    let warm = (1..=22).map(|n| { let t: f64 = rolling.read_by_name(&format!("TYRE_TEMPERATURE_C:{n}")); t }).fold(f64::MIN, f64::max);
    assert!(warm > hottest, "taxiing still warms the tyres: {warm} C");
}

#[test]
fn a_full_tyre_burst_armed_from_the_efb_flattens_one_tyre_at_once() {
    let mut test_bed = aircraft();
    run(&mut test_bed, 10);
    assert_eq!(arm(&mut test_bed, 32_101., 1.), 1.);
    run(&mut test_bed, 5);
    let leaked: Vec<f64> = (1..=22).map(|n| test_bed.read_by_name(&format!("TYRE_LEAKED_FRACTION:{n}"))).collect();
    assert_eq!(leaked.iter().filter(|&&f| f >= 1.).count(), 1, "exactly one tyre is flat: {leaked:?}");
    assert_eq!(leaked.iter().filter(|&&f| f > 0.).count(), 1, "the rest of the aircraft is untouched: {leaked:?}");
    let flat = leaked.iter().position(|&f| f >= 1.).unwrap() + 1;
    let pressure: f64 = test_bed.read_by_name(&format!("TYRE_PRESSURE_PA:{flat}"));
    assert!(pressure < 1., "a burst tyre holds no pressure: {pressure} Pa");
}
