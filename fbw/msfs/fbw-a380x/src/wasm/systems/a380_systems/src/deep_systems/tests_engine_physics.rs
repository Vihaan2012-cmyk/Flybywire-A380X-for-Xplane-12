use deep_systems::physics::engine::free_engine::{FreeEngine, Inputs, Outputs};
use deep_systems::physics::engine::params::{P_REF_PA, T_REF_K};

const DT: f64 = 1.0 / 30.0;
const TAKEOFF_N1: f64 = 83.8;

fn ground() -> Inputs {
    Inputs::at(P_REF_PA, T_REF_K, 0.0, 0.0, DT)
}

fn running(n1_target: f64) -> Inputs {
    Inputs { n1_target_pct: n1_target, fuel_available: true, ignition: false, ..ground() }
}

fn run_for(e: &mut FreeEngine, i: &Inputs, seconds: f64) -> Vec<Outputs> {
    (0..(seconds / DT).round() as usize).map(|_| e.step(i)).collect()
}

fn isa(alt_ft: f64) -> (f64, f64) {
    let h = alt_ft * 0.3048;
    if h <= 11_000.0 {
        let t = 288.15 - 0.0065 * h;
        (101_325.0 * (t / 288.15).powf(5.2559), t)
    } else {
        (22_632.0 * (-9.80665 * (h - 11_000.0) / (287.05 * 216.65)).exp(), 216.65)
    }
}

pub(super) fn steady(alt_ft: f64, mach: f64, n1: f64) -> Outputs {
    let (p, t) = isa(alt_ft);
    let tas = mach * (1.4 * 287.05 * t).sqrt();
    let i = Inputs { n1_target_pct: n1, fuel_available: true, ..Inputs::at(p, t, mach, tas, DT) };
    let mut e = FreeEngine::new();
    e.settle_running(&i, n1, n1 + 5.0, n1 * 0.4 + 60.0);
    run_for(&mut e, &i, 60.0).last().copied().unwrap()
}

fn at_altitude(alt_ft: f64, mach: f64, n1_target: f64) -> Inputs {
    let (p, t) = isa(alt_ft);
    let tas = mach * (1.4 * 287.05 * t).sqrt();
    Inputs { n1_target_pct: n1_target, fuel_available: true, ..Inputs::at(p, t, mach, tas, DT) }
}

fn spread(outs: &[Outputs], f: impl Fn(&Outputs) -> f64) -> f64 {
    let (lo, hi) = outs.iter().map(&f).fold((f64::MAX, f64::MIN), |(lo, hi), v| (lo.min(v), hi.max(v)));
    hi - lo
}

fn idling() -> FreeEngine {
    let mut e = FreeEngine::new();
    e.settle_running(&running(18.5), 18.5, 48.0, 64.5);
    run_for(&mut e, &running(18.5), 30.0);
    e
}

#[test]
fn a_healthy_engine_holds_steady_at_idle_and_at_takeoff() {
    let mut e = idling();
    let idle = run_for(&mut e, &running(18.5), 10.0);
    assert!(spread(&idle, |o| o.n1_pct) < 0.2, "idle N1 hunts: {}", spread(&idle, |o| o.n1_pct));
    assert!(spread(&idle, |o| o.egt_c) < 3.0, "idle EGT wanders: {}", spread(&idle, |o| o.egt_c));
    run_for(&mut e, &running(TAKEOFF_N1), 30.0);
    let takeoff = run_for(&mut e, &running(TAKEOFF_N1), 10.0);
    assert!(spread(&takeoff, |o| o.n1_pct) < 0.5, "takeoff N1 hunts: {}", spread(&takeoff, |o| o.n1_pct));
    assert!(spread(&takeoff, |o| o.egt_c) < 5.0, "takeoff EGT wanders: {}", spread(&takeoff, |o| o.egt_c));
}

#[test]
fn idle_to_takeoff_spools_up_without_surge_or_overtemperature() {
    let mut e = idling();
    let accel = run_for(&mut e, &running(TAKEOFF_N1), 30.0);
    let final_thrust = accel.last().unwrap().net_thrust_n;
    let reached = accel.iter().position(|o| o.net_thrust_n >= 0.95 * final_thrust).unwrap() as f64 * DT;
    assert!(reached < 12.0, "95% takeoff thrust took {reached:.1} s");
    assert!(accel.iter().all(|o| o.lit && o.hpc_stall_margin >= 1.0), "the spool-up surged or flamed out");
    let peak_egt = accel.iter().map(|o| o.egt_c).fold(f64::MIN, f64::max);
    assert!(peak_egt < 920.0, "spool-up EGT peaked at {peak_egt:.0} C");
}

#[test]
fn takeoff_to_idle_spools_down_without_flaming_out() {
    let mut e = idling();
    run_for(&mut e, &running(TAKEOFF_N1), 30.0);
    let decel = run_for(&mut e, &running(18.5), 20.0);
    assert!(decel.iter().all(|o| o.lit), "the engine flamed out decelerating");
    let back = decel.iter().position(|o| o.n1_pct < 25.0).map(|k| k as f64 * DT);
    assert!(back.is_some_and(|s| s < 15.0), "N1 back under 25% after {back:?} s");
}

#[test]
fn a_start_from_rest_lights_off_and_reaches_idle() {
    let mut e = FreeEngine::new();
    let crank = Inputs { starter_supply_fraction: 1.0, ignition: true, ..running(18.5) };
    let start = run_for(&mut e, &crank, 80.0);
    let light_off = start.iter().position(|o| o.lit).map(|k| k as f64 * DT);
    assert!(light_off.is_some_and(|s| s < 20.0), "light-off at {light_off:?} s");
    let idle = start.iter().position(|o| o.n3_pct >= 60.0).map(|k| k as f64 * DT);
    assert!(idle.is_some_and(|s| s < 70.0), "N3 60% reached at {idle:?} s");
    let peak = start.iter().map(|o| o.egt_c).fold(f64::MIN, f64::max);
    assert!(peak < 750.0, "start EGT peaked at {peak:.0} C");
}

#[test]
fn cutting_the_fuel_flames_out_and_the_core_runs_down() {
    let mut e = idling();
    let cut = run_for(&mut e, &Inputs { fuel_available: false, ..running(18.5) }, 60.0);
    assert!(!cut[30].lit, "still burning after the fuel was cut");
    assert!(cut.last().unwrap().n3_pct < 30.0, "N3 {} a minute after the cut", cut.last().unwrap().n3_pct);
    let stopped = run_for(&mut e, &Inputs { fuel_available: false, ..running(18.5) }, 120.0);
    assert!(stopped.last().unwrap().n3_pct < 1.0, "the core must come to rest, N3 {} three minutes after the cut", stopped.last().unwrap().n3_pct);
    assert!(cut.last().unwrap().egt_c < 150.0, "EGT {} a minute after the cut", cut.last().unwrap().egt_c);
}

#[test]
fn a_destroyed_hp_compressor_cannot_keep_the_core_running() {
    let mut e = idling();
    let destroyed = Inputs { hpc_efficiency: 0.05, hpc_flow_capacity: 0.05, ..running(18.5) };
    let after = run_for(&mut e, &destroyed, 40.0);
    let last = after.last().unwrap();
    assert!(!last.lit || last.n3_pct < 50.0, "a destroyed compressor still holds N3 {} lit {}", last.n3_pct, last.lit);
    assert!(last.n1_pct < 15.0, "N1 {} with the core destroyed", last.n1_pct);
}

#[test]
fn a_flamed_out_engine_windmills_in_flight() {
    let mut e = FreeEngine::new();
    e.settle_running(&at_altitude(35_000.0, 0.8, 84.0), 84.0, 86.0, 88.0);
    let out = run_for(&mut e, &Inputs { fuel_available: false, ..at_altitude(35_000.0, 0.8, 84.0) }, 90.0);
    let last = out.last().unwrap();
    assert!(!last.lit);
    assert!(last.n1_pct > 15.0 && last.n1_pct < 50.0, "windmilling N1 {}", last.n1_pct);
    assert!(last.n3_pct > 2.0, "the core must keep turning in the airstream: N3 {}", last.n3_pct);
}

#[test]
fn healthy_operating_points_match_the_engine() {
    let idle = steady(0.0, 0.0, 18.5);
    assert!((0.17..0.23).contains(&idle.wf_kg_s) && (340.0..420.0).contains(&idle.egt_c), "idle {:.3} kg/s {:.0} C", idle.wf_kg_s, idle.egt_c);
    let takeoff = steady(0.0, 0.0, TAKEOFF_N1);
    assert!((2.0..2.7).contains(&takeoff.wf_kg_s) && (820.0..900.0).contains(&takeoff.egt_c), "takeoff {:.3} kg/s {:.0} C", takeoff.wf_kg_s, takeoff.egt_c);
    assert!((280_000.0..330_000.0).contains(&takeoff.net_thrust_n), "takeoff thrust {:.0} kN", takeoff.net_thrust_n / 1000.0);
    let cruise = steady(37_000.0, 0.85, 86.0);
    assert!((0.7..0.9).contains(&cruise.wf_kg_s) && (650.0..750.0).contains(&cruise.egt_c), "cruise {:.3} kg/s {:.0} C", cruise.wf_kg_s, cruise.egt_c);
    let flat_rated_limit = steady(0.0, 0.0, 95.0);
    assert!(flat_rated_limit.n1_pct > 94.0, "the engine must reach the highest takeoff N1 FBW can command: {:.1}", flat_rated_limit.n1_pct);
}


#[test]
fn pack_bleed_costs_egt_and_fuel_at_cruise() {
    let settle = |bleed: f64| {
        let i = Inputs { ip_bleed_kg_s: bleed, ..at_altitude(37_000.0, 0.85, 86.0) };
        let mut e = FreeEngine::new();
        e.settle_running(&i, 86.0, 87.0, 90.0);
        run_for(&mut e, &i, 60.0).last().copied().unwrap()
    };
    let clean = settle(0.0);
    let bled = settle(1.2);
    assert!(bled.egt_c > clean.egt_c + 3.0 && bled.egt_c < clean.egt_c + 60.0, "bleed EGT {:.0} vs {:.0}", bled.egt_c, clean.egt_c);
    assert!(bled.wf_kg_s > clean.wf_kg_s, "bleed fuel {:.3} vs {:.3}", bled.wf_kg_s, clean.wf_kg_s);
}

mod in_the_aircraft {
    use super::super::tests::{aircraft, arm, run};
    use crate::A380;
    use systems::simulation::test::{SimulationTestBed, TestBed, WriteByName};

    fn engines_running_at_idle() -> SimulationTestBed<A380> {
        let mut test_bed = aircraft();
        for n in 1..=4 {
            test_bed.write_by_name(&format!("GENERAL ENG STARTER:{n}"), 1.0);
            test_bed.write_by_name(&format!("ENGINE_STATE:{n}"), 1.0);
            test_bed.write_by_name(&format!("ENGINE_N1:{n}"), 18.5);
            test_bed.write_by_name(&format!("ENGINE_N2:{n}"), 48.0);
            test_bed.write_by_name(&format!("ENGINE_N3:{n}"), 64.5);
            test_bed.write_by_name(&format!("ENGINE_N2_HEALTHY:{n}"), 48.0);
            test_bed.write_by_name(&format!("ENGINE_N3_HEALTHY:{n}"), 64.5);
            test_bed.write_by_name(&format!("AUTOTHRUST_N1_COMMANDED:{n}"), 18.5);
        }
        test_bed
    }

    fn phys(test_bed: &mut SimulationTestBed<A380>, eng: usize, what: &str) -> f64 {
        test_bed.query(|a| a.deep_systems.snapshot())[&format!("A32NX_ENG_{eng}_PHYS_{what}")]
    }

    fn run_with_fadec(test_bed: &mut SimulationTestBed<A380>, frames: usize) {
        for _ in 0..frames {
            run(test_bed, 1);
            let p = test_bed.query(|a| a.deep_systems.snapshot());
            if p.get("A32NX_ENG_1_PHYS_VALID").copied().unwrap_or(0.0) < 0.5 {
                continue;
            }
            for n in 1..=4 {
                let g = |s: &str| p[&format!("A32NX_ENG_{n}_PHYS_{s}")];
                let (n1, n2, n3) = (g("N1"), g("N2"), g("N3"));
                test_bed.write_by_name(&format!("ENGINE_N1:{n}"), n1);
                test_bed.write_by_name(&format!("ENGINE_N2:{n}"), n2);
                test_bed.write_by_name(&format!("ENGINE_N3:{n}"), n3);
                test_bed.write_by_name(&format!("ENGINE_N2_HEALTHY:{n}"), n2);
                test_bed.write_by_name(&format!("ENGINE_N3_HEALTHY:{n}"), n3);
            }
        }
    }

    #[test]
    fn the_engine_model_runs_the_engine_on_the_real_fuel_chain() {
        let mut test_bed = engines_running_at_idle();
        run_with_fadec(&mut test_bed, 30 * 30);
        assert_eq!(phys(&mut test_bed, 1, "VALID"), 1.0);
        assert_eq!(phys(&mut test_bed, 1, "LIT"), 1.0, "a running engine with its master on must stay lit");
        let n1 = phys(&mut test_bed, 1, "N1");
        let egt = phys(&mut test_bed, 1, "EGT_C");
        let ff = phys(&mut test_bed, 1, "FF_KG_S");
        assert!((17.0..20.0).contains(&n1), "idle N1 {n1}");
        assert!((330.0..450.0).contains(&egt), "idle EGT {egt}");
        assert!((0.15..0.25).contains(&ff), "idle fuel flow {ff}");

        for n in 1..=4 {
            test_bed.write_by_name(&format!("AUTOTHRUST_N1_COMMANDED:{n}"), super::TAKEOFF_N1);
        }
        run_with_fadec(&mut test_bed, 20 * 30);
        let n1 = phys(&mut test_bed, 1, "N1");
        assert!((super::TAKEOFF_N1 - 1.0..super::TAKEOFF_N1 + 1.0).contains(&n1), "takeoff N1 {n1}");
        assert!(phys(&mut test_bed, 1, "EGT_C") < 900.0);
    }

    #[test]
    fn engines_already_turning_in_the_sim_at_spawn_sync_before_fbw_reports_them() {
        let mut test_bed = aircraft();
        for n in 1..=4 {
            test_bed.write_by_name(&format!("TURB ENG CORRECTED N1:{n}"), 18.5);
            test_bed.write_by_name(&format!("TURB ENG CORRECTED N2:{n}"), 64.5);
            test_bed.write_by_name(&format!("GENERAL ENG STARTER:{n}"), 1.0);
            test_bed.write_by_name(&format!("AUTOTHRUST_N1_COMMANDED:{n}"), 18.5);
        }
        run_with_fadec(&mut test_bed, 60);
        for n in 1..=4 {
            test_bed.write_by_name(&format!("ENGINE_STATE:{n}"), 1.0);
        }
        run_with_fadec(&mut test_bed, 30 * 10);
        for n in 1..=4 {
            assert_eq!(phys(&mut test_bed, n, "VALID"), 1.0);
            assert_eq!(phys(&mut test_bed, n, "LIT"), 1.0, "engine {n} was turning at spawn and must be running in the model");
            let n1 = phys(&mut test_bed, n, "N1");
            assert!((17.0..20.5).contains(&n1), "engine {n} N1 {n1}");
        }
    }

    #[test]
    fn a_cold_and_dark_spawn_goes_valid_with_the_engines_at_rest() {
        let mut test_bed = aircraft();
        run_with_fadec(&mut test_bed, 40);
        assert_eq!(phys(&mut test_bed, 1, "VALID"), 1.0);
        assert_eq!(phys(&mut test_bed, 1, "LIT"), 0.0);
        assert!(phys(&mut test_bed, 1, "N3") < 1.0);
    }

    #[test]
    fn standing_still_at_toga_the_eec_holds_seventy_eight_percent_until_the_aircraft_rolls() {
        let mut test_bed = engines_running_at_idle();
        test_bed.set_on_ground(true);
        test_bed.write_by_name("GPS GROUND SPEED", 0.);
        run_with_fadec(&mut test_bed, 10 * 30);
        for n in 1..=4 {
            test_bed.write_by_name(&format!("AUTOTHRUST_N1_COMMANDED:{n}"), super::TAKEOFF_N1);
        }
        run_with_fadec(&mut test_bed, 30 * 10);
        let held = phys(&mut test_bed, 1, "N1");
        assert!((77.0..78.6).contains(&held), "static TOGA N1 {held}");

        test_bed.write_by_name("GPS GROUND SPEED", 40.);
        run_with_fadec(&mut test_bed, 20 * 10);
        let rolling = phys(&mut test_bed, 1, "N1");
        assert!((super::TAKEOFF_N1 - 1.0..super::TAKEOFF_N1 + 1.0).contains(&rolling), "rolling TOGA N1 {rolling}");
    }

    #[test]
    fn three_minutes_at_takeoff_thrust_cost_the_hot_section_no_life() {
        let mut test_bed = engines_running_at_idle();
        run_with_fadec(&mut test_bed, 10 * 30);
        for n in 1..=4 {
            test_bed.write_by_name(&format!("AUTOTHRUST_N1_COMMANDED:{n}"), super::TAKEOFF_N1);
        }
        run_with_fadec(&mut test_bed, 3 * 60 * 10);
        for n in 1..=4 {
            let tgt = test_bed.query(|a| a.deep_systems.published_value(&format!("A32NX_ENG_{n}_PHYS_EGT_C")));
            assert!(tgt > 600., "the hot-section monitor must see engine {n}'s real TGT, read {tgt}");
            let used: f64 = systems::simulation::test::ReadByName::read_by_name(&mut test_bed, &format!("DEEP_ENG_{n}_HOT_SECTION_LIFE_USED"));
            assert_eq!(used, 0., "engine {n} lost hot-section life at a normal take-off ({tgt:.0} C untrimmed)");
        }
    }

    #[test]
    fn a_destroyed_hp_compressor_flames_the_engine_out_and_the_core_runs_down() {
        let mut test_bed = engines_running_at_idle();
        run_with_fadec(&mut test_bed, 10 * 30);
        assert_eq!(arm(&mut test_bed, 72012.0, 1.0), 1.);
        run_with_fadec(&mut test_bed, 40 * 30);
        let n3 = phys(&mut test_bed, 1, "N3");
        assert!(n3 < 50.0, "N3 {n3} with the HP compressor destroyed");
        assert_eq!(phys(&mut test_bed, 1, "LIT"), 0.0, "a destroyed core cannot keep burning");
        assert_eq!(phys(&mut test_bed, 2, "LIT"), 1.0, "engine 2 is untouched");
        let n1_2 = phys(&mut test_bed, 2, "N1");
        assert!((17.0..20.0).contains(&n1_2), "engine 2 N1 {n1_2}");
    }

    fn snapshot_value(test_bed: &mut SimulationTestBed<A380>, name: &str) -> f64 {
        test_bed.query(|a| a.deep_systems.snapshot()).get(name).copied().unwrap_or(f64::NAN)
    }

    #[test]
    fn an_hp_compressor_destroyed_at_takeoff_power_breaks_out_of_the_case_and_sets_the_nacelle_alight() {
        let mut test_bed = engines_running_at_idle();
        run_with_fadec(&mut test_bed, 100);
        for n in 1..=4 {
            test_bed.write_by_name(&format!("AUTOTHRUST_N1_COMMANDED:{n}"), super::TAKEOFF_N1);
        }
        run_with_fadec(&mut test_bed, 200);
        assert!(phys(&mut test_bed, 1, "N3") > 90.0, "takeoff N3 {}", phys(&mut test_bed, 1, "N3"));
        assert_eq!(arm(&mut test_bed, 72012.0, 1.0), 1.);
        run_with_fadec(&mut test_bed, 150);

        let breach = snapshot_value(&mut test_bed, "A32NX_ENG_1_CASE_BREACH_FRAC");
        let leak = snapshot_value(&mut test_bed, "A32NX_ENG_1_NACELLE_FUEL_LEAK_KG_S");
        assert!(breach > 0.5, "debris released at takeoff N3 must get out of the case: {breach}");
        assert!(leak > 0.02, "the breached fuel line must leak into the nacelle: {leak}");
        assert_eq!(snapshot_value(&mut test_bed, "FIRE_ZONE_ENG1_BURNING"), 1.0, "a hot core and a fuel leak in its nacelle must burn");
        assert_eq!(snapshot_value(&mut test_bed, "A32NX_ENG_1_CORE_FIRE_CONFIRMED"), 1.0, "both loops must see the nacelle fire");
        assert_eq!(snapshot_value(&mut test_bed, "FIRE_ZONE_ENG2_BURNING"), 0.0, "engine 2 is untouched");
        assert!(snapshot_value(&mut test_bed, "FUEL_TOTAL_LEAK_KG_S") >= leak * 0.99, "the nacelle leak comes out of the tanks");

        test_bed.write_by_name("FIRE_BUTTON_ENG1", 1.0);
        run_with_fadec(&mut test_bed, 10);
        assert_eq!(snapshot_value(&mut test_bed, "A32NX_ENG_1_NACELLE_FUEL_LEAK_KG_S"), 0.0, "the fire pb shuts the LP valve and isolates the leak");
    }

    #[test]
    fn an_hp_compressor_destroyed_at_idle_stays_inside_the_case() {
        let mut test_bed = engines_running_at_idle();
        run_with_fadec(&mut test_bed, 100);
        assert_eq!(arm(&mut test_bed, 72012.0, 1.0), 1.);
        run_with_fadec(&mut test_bed, 600);
        assert_eq!(snapshot_value(&mut test_bed, "A32NX_ENG_1_CASE_BREACH_FRAC"), 0.0);
        assert_eq!(snapshot_value(&mut test_bed, "A32NX_ENG_1_NACELLE_FUEL_LEAK_KG_S"), 0.0);
        assert_eq!(snapshot_value(&mut test_bed, "FIRE_ZONE_ENG1_BURNING"), 0.0, "contained debris starts no fire");
    }
}


#[test]
#[ignore]
fn engine_model_costs() {
    use deep_systems::physics::engine::{ShadowEngine, ShadowInputs};
    let i = Inputs { ip_bleed_kg_s: 0.0, ..running(18.5) };
    let mut e = FreeEngine::new();
    e.settle_running(&i, 18.5, 47.0, 62.5);
    let n = 3000;
    let t = std::time::Instant::now();
    for _ in 0..n {
        std::hint::black_box(e.step(&i));
    }
    let free_us = t.elapsed().as_secs_f64() * 1e6 / n as f64;
    let si = ShadowInputs {
        ambient_pressure_pa: P_REF_PA,
        ambient_temp_k: T_REF_K,
        mach: 0.0,
        n1_pct: 18.5,
        n2_pct: 47.0,
        n3_pct: 62.5,
        wf_kg_s: 0.21,
        bleed_extraction_kg_s: 0.0,
        bleed_from_ip_port: false,
        compressor_efficiency_loss_fraction: 0.0,
        compressor_flow_capacity_loss_fraction: 0.0,
        turbine_efficiency_loss_fraction: 0.0,
        oil_pressure_fraction: 1.0,
        oil_faults: Default::default(),
        fuel_temp_k: 288.0,
        dt_s: 0.1,
    };
    let mut s = ShadowEngine::new();
    for _ in 0..50 {
        s.step(&si);
    }
    let t = std::time::Instant::now();
    for _ in 0..n {
        std::hint::black_box(s.step(&si));
    }
    let shadow_us = t.elapsed().as_secs_f64() * 1e6 / n as f64;
    println!("per engine per frame: free engine {free_us:.1} us, gas-path shadow {shadow_us:.1} us; x4 engines = {:.2} ms", 4. * (free_us + shadow_us) / 1e3);
}
