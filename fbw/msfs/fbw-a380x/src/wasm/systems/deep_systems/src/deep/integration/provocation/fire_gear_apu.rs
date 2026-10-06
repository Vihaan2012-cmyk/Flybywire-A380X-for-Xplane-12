#[allow(unused_imports)]
use super::{failure_on, Provocation, Provoked};
use crate::deep::api::{failure_id, Area as RegArea, FailureDef, Registry};
use crate::deep::live::Truth;

pub(super) fn provocations() -> Vec<Provocation> {
    vec![
        Provocation { applies: is_bottle_leak, provoke: provoke_bottle_leak, why: "a bottle's own continuous leak; the low-pressure switch must trip once enough simulated time passes (a few hours at full severity -- see the area's own `a_leaking_fire_bottle_falls_below_its_pressure_switch` test)." },
        Provocation { applies: is_cargo_suppression_leak, provoke: provoke_cargo_suppression_leak, why: "a cargo suppression bottle's own continuous leak; the 30 kg charge takes longer to show low pressure than an engine bottle's 5 kg." },
        Provocation { applies: is_bay_leak_feeding_fire, provoke: provoke_bay_leak_feeding_fire, why: "the bay's own ignition source, with the leak supplying it fuel: in the MLG bay a dragging brake heated on a long taxi past the fluid's autoignition point; in a hold or the main avionics bay the content fire `deep::thermal_zones` burns there (`fire_ice::live`'s `ignition_source()`)." },
        Provocation { applies: is_cargo_smoke_lens, provoke: provoke_cargo_smoke_lens, why: "a content fire burning in that hold: its smoke, carried by `deep::thermal_zones`' own smoke transport, is what the detector's chamber samples through the lens." },
        Provocation { applies: is_lavatory_link, provoke: provoke_lavatory_link, why: "a lavatory bin fire, modelled directly as `Truth::cabin_temp_k` reaching the fusible link's melt point -- the one cabin heat input this port has for a lavatory fire." },
        Provocation { applies: is_rain_removal, provoke: provoke_rain_removal, why: "rain on the aircraft plus the rain-removal command selected." },
        Provocation { applies: is_gear_uplock, provoke: provoke_gear_uplock, why: "the gear cycling: lever selected up, airborne, run long enough for the leg to travel to the up end-stop and attempt to lock." },
        Provocation { applies: is_gear_downlock, provoke: provoke_gear_downlock, why: "the gear cycling into the down position. NOTE: `Retraction::new` always starts a leg already down-and-locked, and the provoked harness holds one static Truth for the whole run, so this harness cannot express a retract-then-extend sequence; the downlock-engage code path is never reached. This is a harness limitation, not confirmation the failure is unwired -- see the report." },
        Provocation { applies: is_nosewheel_steering, provoke: provoke_nosewheel_steering, why: "the tiller and steering used on the ground: weight on the nose wheel, a real steering command, groundspeed." },
        Provocation { applies: is_pax_oxygen, provoke: provoke_pax_oxygen, why: "a cabin depressurisation: cabin altitude past the 14,000 ft automatic-deployment trigger, with the deployment circuit powered." },
        Provocation { applies: is_apu_start_fault, provoke: provoke_apu_start_fault, why: "an APU start attempt: master on, start pushbutton on, run long enough for the start sequence to try to light." },
        Provocation { applies: is_apu_oil_leak, provoke: provoke_apu_oil_leak, why: "the APU running; a continuous oil leak needs simulated time to show on quantity/pressure." },
        Provocation { applies: is_apu_gen_overload, provoke: provoke_apu_gen_overload, why: "the APU generator carrying an overload: APU running, its generator on line, heavy bus load." },
        Provocation { applies: is_apu_fire_protection, provoke: provoke_apu_fire_protection, why: "an APU compartment fire (the companion registry failure `APU compartment fire`), which the fire loop must detect and the squib must discharge against." },
    ]
}

fn is_bottle_leak(f: &FailureDef) -> bool {
    matches!(f.id, 8_026_201 | 8_026_203 | 8_026_205 | 8_026_207 | 8_026_209 | 8_026_211 | 8_026_213 | 8_026_215 | 8_026_217)
}

fn provoke_bottle_leak(_f: &FailureDef, _r: &Registry) -> Provoked {
    Provoked { later: None,
        profile: "ground_apu",
        truth: Some(Box::new(|t: &mut Truth| {
            t.engine_running = [false; 4];
            t.apu_running = false;
            t.dt_s = 10.0;
        })),
        companions: vec![],
        frames: 4000,
    }
}

fn is_cargo_suppression_leak(f: &FailureDef) -> bool {
    matches!(f.id, 8_026_219 | 8_026_223)
}

fn provoke_cargo_suppression_leak(_f: &FailureDef, _r: &Registry) -> Provoked {
    Provoked { later: None,
        profile: "ground_apu",
        truth: Some(Box::new(|t: &mut Truth| {
            t.engine_running = [false; 4];
            t.apu_running = false;
            t.dt_s = 60.0;
        })),
        companions: vec![],
        frames: 5000,
    }
}

fn is_bay_leak_feeding_fire(f: &FailureDef) -> bool {
    matches!(f.id, 8_026_105 | 8_026_106 | 8_026_107 | 8_026_108)
}

fn content_fire(zone: u64) -> u64 {
    let n = match zone {
        6 => 1,
        7 => 2,
        _ => 12,
    };
    failure_id(RegArea::ThermalZones, 26, n)
}

fn provoke_bay_leak_feeding_fire(f: &FailureDef, r: &Registry) -> Provoked {
    if f.id == 8_026_105 {
        return Provoked { later: None,
            profile: "ground_apu",
            truth: Some(Box::new(|t: &mut Truth| {
                t.dt_s = 0.5;
                t.groundspeed_m_s = 15.0;
                t.controls.parking_brake_on = false;
            })),
            companions: failure_on(r, "32_gear.wheel_1_brake", "dragging brake").map(|id| vec![(id, 1.0)]).unwrap_or_default(),
            frames: 3600,
        };
    }
    Provoked { later: None,
        profile: "ground_apu",
        truth: Some(Box::new(|t: &mut Truth| {
            t.dt_s = 1.0;
        })),
        companions: vec![(content_fire(f.id - 8_026_100), 1.0)],
        frames: 120,
    }
}

fn is_cargo_smoke_lens(f: &FailureDef) -> bool {
    matches!(f.id, 8_026_227 | 8_026_228)
}

fn provoke_cargo_smoke_lens(f: &FailureDef, _r: &Registry) -> Provoked {
    Provoked { later: None,
        profile: "ground_apu",
        truth: Some(Box::new(|t: &mut Truth| {
            t.dt_s = 1.0;
        })),
        companions: vec![(content_fire(6 + (f.id - 8_026_227)), 1.0)],
        frames: 120,
    }
}

fn is_lavatory_link(f: &FailureDef) -> bool {
    f.id == 8_026_229
}

fn provoke_lavatory_link(_f: &FailureDef, _r: &Registry) -> Provoked {
    Provoked { later: None,
        profile: "cold_dark",
        truth: Some(Box::new(|t: &mut Truth| {
            t.cabin_temp_k = 420.0;
        })),
        companions: vec![],
        frames: 5,
    }
}

fn is_rain_removal(f: &FailureDef) -> bool {
    matches!(f.id, 8_030_049 | 8_030_050)
}

fn provoke_rain_removal(_f: &FailureDef, _r: &Registry) -> Provoked {
    Provoked { later: None,
        profile: "cruise",
        truth: Some(Box::new(|t: &mut Truth| {
            t.controls.rain_removal_selected = [true, true];
            t.environment.precipitation_on_aircraft_ratio = 1.0;
        })),
        companions: vec![],
        frames: 50,
    }
}

fn is_gear_uplock(f: &FailureDef) -> bool {
    matches!(f.id, 5_032_022 | 5_032_027 | 5_032_032 | 5_032_037 | 5_032_043)
}

fn provoke_gear_uplock(_f: &FailureDef, _r: &Registry) -> Provoked {
    Provoked { later: None, profile: "gear_cycle", truth: None, companions: vec![], frames: 1500 }
}

fn is_gear_downlock(f: &FailureDef) -> bool {
    matches!(f.id, 5_032_023 | 5_032_028 | 5_032_033 | 5_032_038 | 5_032_044)
}

fn provoke_gear_downlock(_f: &FailureDef, _r: &Registry) -> Provoked {
    Provoked { later: None,
        profile: "touchdown",
        truth: Some(Box::new(|t: &mut Truth| {
            t.controls.gear_lever_down = true;
        })),
        companions: vec![],
        frames: 400,
    }
}

fn is_nosewheel_steering(f: &FailureDef) -> bool {
    matches!(f.id, 5_032_082 | 5_032_083 | 5_032_084)
}

fn provoke_nosewheel_steering(_f: &FailureDef, _r: &Registry) -> Provoked {
    Provoked { later: None,
        profile: "ground_apu",
        truth: Some(Box::new(|t: &mut Truth| {
            t.leg_on_ground = [true; 5];
            t.groundspeed_m_s = 10.0;
            t.hydraulic_pressure_pa = [34_474_000.0; 2];
            t.controls.parking_brake_on = false;
            t.controls.steering_command_deg = [30.0, 0.0, 0.0];
        })),
        companions: vec![],
        frames: 100,
    }
}

fn is_pax_oxygen(f: &FailureDef) -> bool {
    matches!(f.id, 20_035_012 | 20_035_013 | 20_035_015 | 20_035_016)
}

fn provoke_pax_oxygen(_f: &FailureDef, _r: &Registry) -> Provoked {
    Provoked { later: None,
        profile: "cruise",
        truth: Some(Box::new(|t: &mut Truth| {
            t.cabin_pressure_pa = 50_000.0;
            t.ac_bus_volts = [115.0; 4];
            t.dc_bus_volts = [28.0; 2];
        })),
        companions: vec![],
        frames: 10,
    }
}

fn is_apu_start_fault(f: &FailureDef) -> bool {
    matches!(f.id, 7_049_007 | 7_049_008)
}

fn provoke_apu_start_fault(_f: &FailureDef, _r: &Registry) -> Provoked {
    Provoked { later: None,
        profile: "apu_start_soak",
        truth: Some(Box::new(|t: &mut Truth| {
            t.apu_running = false;
            t.apu_bleed_pressure_pa = 0.0;
        })),
        companions: vec![],
        frames: 600,
    }
}

fn is_apu_oil_leak(f: &FailureDef) -> bool {
    f.id == 7_049_010
}

fn provoke_apu_oil_leak(_f: &FailureDef, _r: &Registry) -> Provoked {
    Provoked { later: None, profile: "apu_start_soak", truth: Some(Box::new(|t: &mut Truth| t.dt_s = 10.0)), companions: vec![], frames: 2000 }
}

fn is_apu_gen_overload(f: &FailureDef) -> bool {
    matches!(f.id, 7_049_013 | 7_049_015)
}

fn provoke_apu_gen_overload(_f: &FailureDef, _r: &Registry) -> Provoked {
    Provoked { later: None,
        profile: "apu_start_soak",
        truth: Some(Box::new(|t: &mut Truth| {
            t.controls.apu_gen_pb_on = [true, true];
        })),
        companions: vec![],
        frames: 300,
    }
}

fn is_apu_fire_protection(f: &FailureDef) -> bool {
    matches!(f.id, 7_049_017 | 7_049_018)
}

fn provoke_apu_fire_protection(f: &FailureDef, r: &Registry) -> Provoked {
    let mut companions = vec![];
    if let Some(id) = failure_on(r, "26_thermal.apu_compartment_fire_load", "fire") {
        companions.push((id, 1.0));
    }
    let apu_squib = f.id == 7_049_018;
    Provoked { later: None,
        profile: "ground_apu",
        truth: Some(Box::new(move |t: &mut Truth| {
            t.dt_s = 1.0;
            if apu_squib {
                t.controls.fire_pb_apu_released = true;
                t.controls.fire_agent_pb_apu_pressed = true;
            }
        })),
        companions,
        frames: 400,
    }
}
