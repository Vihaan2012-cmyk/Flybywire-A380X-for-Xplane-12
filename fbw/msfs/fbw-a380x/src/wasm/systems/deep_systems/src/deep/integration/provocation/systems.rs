#[allow(unused_imports)]
use super::{failure_on, Provocation, Provoked};
use crate::deep::api::{FailureDef, Registry};
use crate::deep::live::Truth;

pub(super) fn provocations() -> Vec<Provocation> {
    vec![
        Provocation {
            applies: is_nacelle_fire_loop_fails_to_detect,
            provoke: provoke_nacelle_fire_loop,
            why: "a real fire in that engine's nacelle: a fuel/oil leak (`fire_ice`'s `ENG n` leak feeding a fire) lit by the running engine's hot case, whose flame crosses the loops' 450 C trip point -- which a loop that fails to detect must then fail to confirm.",
        },
        Provocation { applies: is_apu_precooler_fault, provoke: provoke_apu_precooler_fault, why: "APU bleed in use: the precooler only does anything with hot bleed air actually flowing through it." },
        Provocation { applies: is_apu_feed_valve, provoke: provoke_apu_feed_valve, why: "the APU running and its feed valve commanded open, drawing real fuel flow a stuck valve can restrict." },
    ]
}

fn is_nacelle_fire_loop_fails_to_detect(f: &FailureDef) -> bool {
    matches!(
        f.id,
        2_026_001 | 2_026_003 | 2_026_005 | 2_026_007 | 2_026_009 | 2_026_011 | 2_026_013 | 2_026_015 | 2_026_017 | 2_026_019 | 2_026_021 | 2_026_023 | 2_026_025 | 2_026_027 | 2_026_029 | 2_026_031
    )
}

fn engine_of(f: &FailureDef) -> u16 {
    f.component.chars().last().and_then(|c| c.to_digit(10)).unwrap_or(1) as u16
}

fn provoke_nacelle_fire_loop(f: &FailureDef, _r: &Registry) -> Provoked {
    let engine = engine_of(f);
    let leak = crate::deep::api::failure_id(crate::deep::api::Area::FireIce, 26, 99 + engine);
    Provoked { later: None,
        profile: "cold_dark",
        truth: Some(Box::new(move |t: &mut Truth| {
            t.dt_s = 1.0;
            t.engine_running[(engine - 1) as usize] = true;
        })),
        companions: vec![(leak, 1.0)],
        frames: 300,
    }
}

fn is_apu_precooler_fault(f: &FailureDef) -> bool {
    matches!(f.id, 15_036_011 | 15_036_012 | 15_036_013)
}

fn provoke_apu_precooler_fault(_f: &FailureDef, _r: &Registry) -> Provoked {
    Provoked { later: None,
        profile: "ground_apu",
        truth: Some(Box::new(|t: &mut Truth| {
            t.controls.apu_bleed_pb_on = true;
            t.controls.pack_pb_on = [true, true];
            t.controls.cross_bleed_selector = 2.0;
            t.apu_bleed_pressure_pa = 310_000.0;
        })),
        companions: vec![],
        frames: 600,
    }
}

fn is_apu_feed_valve(f: &FailureDef) -> bool {
    f.id == 19_028_089
}

fn provoke_apu_feed_valve(_f: &FailureDef, _r: &Registry) -> Provoked {
    Provoked { later: None,
        profile: "ground_apu",
        truth: Some(Box::new(|t: &mut Truth| {
            t.apu_running = true;
            t.controls.apu_master_sw_on = true;
            t.controls.apu_bleed_pb_on = true;
        })),
        companions: vec![],
        frames: 600,
    }
}
