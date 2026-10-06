use std::collections::BTreeMap;
use std::time::{Duration, Instant};

use deep_systems::{DeepSystems, Faults};
use systems::shared::InternationalStandardAtmosphere;
use systems::simulation::test::{ReadByName, SimulationTestBed, TestBed, WriteByName};
use uom::si::{f64::*, length::foot, velocity::knot};

use super::tests::{aircraft, run};
use crate::A380;

fn mean(v: &[f64]) -> f64 {
    if v.is_empty() {
        0.
    } else {
        v.iter().sum::<f64>() / v.len() as f64
    }
}

fn percentile(sorted: &[f64], p: f64) -> f64 {
    if sorted.is_empty() {
        return 0.;
    }
    let idx = (((sorted.len() - 1) as f64) * p).round() as usize;
    sorted[idx]
}

fn engines(test_bed: &mut SimulationTestBed<A380>, n1: f64, n2: f64, n3: f64) {
    for n in 1..=4 {
        test_bed.write_by_name(&format!("ENGINE_STATE:{n}"), 1.);
        test_bed.write_by_name(&format!("TURB ENG CORRECTED N1:{n}"), n1);
        test_bed.write_by_name(&format!("TURB ENG CORRECTED N2:{n}"), n2);
        test_bed.write_by_name(&format!("ENGINE_N2:{n}"), n2);
        test_bed.write_by_name(&format!("ENGINE_N2_HEALTHY:{n}"), n2);
        test_bed.write_by_name(&format!("ENGINE_N3:{n}"), n3);
        test_bed.write_by_name(&format!("ENGINE_N3_HEALTHY:{n}"), n3);
    }
}

fn fly_at(test_bed: &mut SimulationTestBed<A380>, altitude_ft: f64, tas_kt: f64) {
    let altitude = Length::new::<foot>(altitude_ft);
    test_bed.set_pressure_altitude(altitude);
    test_bed.set_ambient_pressure(InternationalStandardAtmosphere::pressure_at_altitude(altitude));
    test_bed.set_ambient_temperature(InternationalStandardAtmosphere::temperature_at_altitude(altitude));
    test_bed.set_true_airspeed(Velocity::new::<knot>(tas_kt));
    test_bed.set_indicated_airspeed(Velocity::new::<knot>(tas_kt.min(300.)));
}

fn run_timed(test_bed: &mut SimulationTestBed<A380>, frames: usize, deep_us: &mut Vec<f64>, frame_us: &mut Vec<f64>) {
    for _ in 0..frames {
        let started = Instant::now();
        let frame_ms = std::env::var("DEEP_PERF_FRAME_MS").ok().and_then(|v| v.parse().ok()).unwrap_or(100);
        test_bed.run_with_delta(Duration::from_millis(frame_ms));
        frame_us.push(started.elapsed().as_secs_f64() * 1e6);
        let us: f64 = test_bed.read_by_name("DEEP_SYSTEMS_TICK_US");
        deep_us.push(us);
    }
}

#[test]
fn deep_tick_time_budget_over_a_whole_flight() {
    let mut test_bed = aircraft();

    let mut deep_us: Vec<f64> = Vec::new();
    let mut frame_us: Vec<f64> = Vec::new();

    run_timed(&mut test_bed, 200, &mut deep_us, &mut frame_us);

    engines(&mut test_bed, 20., 65., 70.);
    for i in 1..=4 {
        test_bed.write_by_name(&format!("OVHD_ELEC_EXT_PWR_{i}_PB_IS_ON"), false);
        test_bed.write_by_name(&format!("EXT_PWR_AVAIL:{i}"), false);
    }
    run_timed(&mut test_bed, 200, &mut deep_us, &mut frame_us);

    test_bed.set_on_ground(false);
    engines(&mut test_bed, 85., 95., 97.);
    for minute in 1..=10 {
        fly_at(&mut test_bed, 1750. * minute as f64, 250. + 10. * minute as f64);
        run_timed(&mut test_bed, 10, &mut deep_us, &mut frame_us);
    }

    fly_at(&mut test_bed, 35000., 490.);
    run_timed(&mut test_bed, 300, &mut deep_us, &mut frame_us);

    deep_us.sort_by(|a, b| a.partial_cmp(b).unwrap());
    frame_us.sort_by(|a, b| a.partial_cmp(b).unwrap());

    let deep_mean = mean(&deep_us);
    let deep_p99 = percentile(&deep_us, 0.99);
    let deep_max = *deep_us.last().unwrap();
    let frame_mean = mean(&frame_us);
    let frame_p99 = percentile(&frame_us, 0.99);
    let frame_max = *frame_us.last().unwrap();

    println!(
        "DEEP_SYSTEMS_TICK_US over {} frames (ground power, engine start, climb, cruise): mean {:.1} us, p99 {:.1} us, max {:.1} us",
        deep_us.len(),
        deep_mean,
        deep_p99,
        deep_max
    );
    println!(
        "whole A380 SimulationElement update() (wall clock, includes deep + every FlyByWire system + test-bed overhead) over {} frames: mean {:.1} us, p99 {:.1} us, max {:.1} us",
        frame_us.len(),
        frame_mean,
        frame_p99,
        frame_max
    );

    assert!(deep_mean < 8000., "deep host mean tick time regressed: {deep_mean:.1} us (budget 8000 us)");
    assert!(deep_p99 < 15000., "deep host p99 tick time regressed: {deep_p99:.1} us (budget 15000 us)");
}

#[test]
fn deep_area_time_breakdown() {
    let mut test_bed = aircraft();
    engines(&mut test_bed, 85., 95., 97.);
    for i in 1..=4 {
        test_bed.write_by_name(&format!("OVHD_ELEC_EXT_PWR_{i}_PB_IS_ON"), false);
        test_bed.write_by_name(&format!("EXT_PWR_AVAIL:{i}"), false);
    }
    test_bed.set_on_ground(false);
    fly_at(&mut test_bed, 35000., 490.);
    run(&mut test_bed, 300);

    let (truth, armed) = test_bed.query(|a| (a.deep_systems.last_truth.clone(), a.deep_systems.armed_failures().clone()));
    let truth = truth.expect("the host has ticked");
    let faults = Faults::from_pairs(armed.iter().map(|(&id, &m)| (id, m)));

    let mut deep = DeepSystems::new();
    for _ in 0..5 {
        deep.tick(truth.clone(), &faults, &mut |_, _| {});
    }

    const ITERS: u32 = 30;
    let mut totals: BTreeMap<&'static str, Duration> = BTreeMap::new();
    for _ in 0..ITERS {
        let timings = deep.tick_timed(truth.clone(), &faults, &mut |_, _| {});
        for (name, tick_time, publish_time) in timings {
            *totals.entry(name).or_default() += tick_time + publish_time;
        }
    }

    let mut ranked: Vec<(&'static str, Duration)> = totals.into_iter().map(|(name, total)| (name, total / ITERS)).collect();
    ranked.sort_by(|a, b| b.1.cmp(&a.1));

    let total_us: f64 = ranked.iter().map(|(_, dt)| dt.as_secs_f64() * 1e6).sum();
    println!("per-area time, mean of {ITERS} frames at cruise (tick + publish), total {total_us:.1} us:");
    for (name, dt) in &ranked {
        let us = dt.as_secs_f64() * 1e6;
        println!("  {name:<32} {us:>8.1} us  ({:>4.1}%)", 100. * us / total_us);
    }
    println!("top five hot spots:");
    for (name, dt) in ranked.iter().take(5) {
        println!("  {name:<32} {:>8.1} us", dt.as_secs_f64() * 1e6);
    }
}

#[test]
fn registered_variable_counts() {
    let published_names = DeepSystems::new().published_names().len();

    let mut test_bed = aircraft();
    engines(&mut test_bed, 85., 95., 97.);
    for i in 1..=4 {
        test_bed.write_by_name(&format!("OVHD_ELEC_EXT_PWR_{i}_PB_IS_ON"), false);
        test_bed.write_by_name(&format!("EXT_PWR_AVAIL:{i}"), false);
    }
    test_bed.set_on_ground(false);
    fly_at(&mut test_bed, 35000., 490.);
    run(&mut test_bed, 300);

    let (registered, units, gates, reset_buttons) = test_bed.query(|a| {
        (
            a.deep_systems.published.len(),
            a.deep_systems.units.len(),
            a.deep_systems.gates.len(),
            a.deep_systems.reset_buttons.len(),
        )
    });
    let arming_vars = 4 + super::MAX_ARMED * 2;
    let breaker_vars = units * 2;
    let total = registered + breaker_vars + gates + reset_buttons + arming_vars + 1;

    run(&mut test_bed, 1);
    let changed_next_frame = test_bed.query(|a| a.deep_systems.published.iter().filter(|p| p.changed).count());

    println!(
        "registered L:vars: {registered} area-published + {breaker_vars} breaker ({units} units x current/cmd) + {gates} gates + {reset_buttons} reset buttons + {arming_vars} EFB arming + 1 tick time = {total} total"
    );
    println!(
        "published names from a fresh DeepSystems (published_names(), pre-dedup source for the {registered} registered): {published_names}"
    );
    println!(
        "of the {registered} area-published values, {changed_next_frame} changed on the next cruise frame (write() sends only these, `mod.rs`'s `for p in self.published.iter().filter(|p| p.changed)`)"
    );
    assert!(changed_next_frame < registered, "expected only a remainder to change frame to frame at a settled cruise");
}
