use super::{failure_on, Provocation, Provoked};
use crate::deep::api::{FailureDef, Registry};
use crate::deep::live::Truth;

pub(super) fn provocations() -> Vec<Provocation> {
    vec![
        smoke_detector(),
        door_proximity(),
        vibration_pickup(),
        pitot_drain_blocked(),
        static_port_leak_to_cabin(),
        radio_altimeter_degradation(),
        ice_detector_heater(),
        tat_probe_3_recovery(),
        tgt_junction_open(),
    ]
}

fn tgt_junction_open() -> Provocation {
    fn applies(f: &FailureDef) -> bool {
        f.component.starts_with("77_eng.tgt_harness_") && f.model_field.contains("open_circuit")
    }
    fn provoke(f: &FailureDef, registry: &Registry) -> Provoked {
        let engine = f.component.trim_start_matches("77_eng.tgt_harness_");
        let companions = failure_on(registry, &format!("73_fuel.manifold_{engine}"), "nozzle group 0 coking")
            .map(|id| vec![(id, 1.0)])
            .unwrap_or_default();
        Provoked { later: None, profile: "cruise", truth: None, companions, frames: 16 }
    }
    Provocation {
        applies,
        provoke,
        why: "a coked nozzle group in the junction's own sector makes the turbine face non-uniform, so losing that junction shifts the average",
    }
}

const SMOKE_ZONE_SOURCE: &[(&str, &str, &str)] = &[
    ("fwd_cargo_a", "26_thermal.cargo_fwd_fire_load", "fire"),
    ("fwd_cargo_b", "26_thermal.cargo_fwd_fire_load", "fire"),
    ("aft_cargo_a", "26_thermal.cargo_aft_fire_load", "fire"),
    ("aft_cargo_b", "26_thermal.cargo_aft_fire_load", "fire"),
    ("lav_1", "26_thermal.cabin_main_deck_lavatory_fire_load", "lavatory"),
    ("lav_2", "26_thermal.cabin_main_deck_lavatory_fire_load", "lavatory"),
    ("lav_3", "26_thermal.cabin_main_deck_lavatory_fire_load", "lavatory"),
    ("lav_4", "26_thermal.cabin_main_deck_lavatory_fire_load", "lavatory"),
    ("lav_5", "26_thermal.cabin_upper_deck_lavatory_fire_load", "lavatory"),
    ("lav_6", "26_thermal.cabin_upper_deck_lavatory_fire_load", "lavatory"),
    ("lav_7", "26_thermal.cabin_upper_deck_lavatory_fire_load", "lavatory"),
    ("lav_8", "26_thermal.cabin_upper_deck_lavatory_fire_load", "lavatory"),
    ("bulk_cargo", "26_thermal.cargo_bulk_fire_load", "cargo compartment fire"),
    ("avncs_main_l", "26_thermal.main_avionics_equipment_fire_load", "equipment fire"),
    ("avncs_main_r", "26_thermal.main_avionics_equipment_fire_load", "equipment fire"),
    ("avncs_upper_l", "26_thermal.upper_avionics_equipment_fire_load", "equipment fire"),
    ("avncs_upper_r", "26_thermal.upper_avionics_equipment_fire_load", "equipment fire"),
    ("avncs_aft", "26_thermal.aft_avionics_equipment_fire_load", "equipment fire"),
    ("main5l_fltrest", "26_thermal.cabin_main_deck_lavatory_fire_load", "lavatory"),
    ("main5l_cabrest", "26_thermal.cabin_main_deck_lavatory_fire_load", "lavatory"),
    ("main_1l_cws", "26_thermal.cabin_main_deck_lavatory_fire_load", "lavatory"),
    ("main_1l_rcc", "26_thermal.cabin_main_deck_lavatory_fire_load", "lavatory"),
    ("upper_1l_cws", "26_thermal.cabin_upper_deck_lavatory_fire_load", "lavatory"),
    ("upper_1l_rcc", "26_thermal.cabin_upper_deck_lavatory_fire_load", "lavatory"),
    ("main_2l_cws", "26_thermal.cabin_main_deck_lavatory_fire_load", "lavatory"),
    ("main_2l_rcc", "26_thermal.cabin_main_deck_lavatory_fire_load", "lavatory"),
    ("upper_2l_cws", "26_thermal.cabin_upper_deck_lavatory_fire_load", "lavatory"),
    ("upper_2l_rcc", "26_thermal.cabin_upper_deck_lavatory_fire_load", "lavatory"),
    ("main_3r_cws", "26_thermal.cabin_main_deck_lavatory_fire_load", "lavatory"),
    ("main_3r_rcc", "26_thermal.cabin_main_deck_lavatory_fire_load", "lavatory"),
    ("upper_3r_cws", "26_thermal.cabin_upper_deck_lavatory_fire_load", "lavatory"),
    ("upper_3r_rcc", "26_thermal.cabin_upper_deck_lavatory_fire_load", "lavatory"),
    ("upper_1l_shower", "26_thermal.cabin_upper_deck_lavatory_fire_load", "lavatory"),
    ("upper_1r_shower", "26_thermal.cabin_upper_deck_lavatory_fire_load", "lavatory"),
    ("fwdlowercrewrest", "26_thermal.fwd_lower_crew_rest_fire_load", "fire"),
];

fn smoke_detector() -> Provocation {
    Provocation {
        applies: |f| {
            f.component.starts_with("26_fire.smoke_")
                && (f.model_field.contains("faults.stuck") || f.model_field.contains("faults.sensitivity_loss"))
        },
        provoke: |f, registry| {
            let slug = f.component.strip_prefix("26_fire.smoke_").unwrap_or("");
            let mut companions = Vec::new();
            if let Some(&(_, comp, frag)) = SMOKE_ZONE_SOURCE.iter().find(|(s, _, _)| *s == slug) {
                if let Some(id) = failure_on(registry, comp, frag) {
                    companions.push((id, 1.0));
                }
            }
            Provoked { later: None, profile: "cruise", truth: None, companions, frames: 30 }
        },
        why: "a smoke detector's own sensing faults (desensitised optics, a frozen reading) only change anything once there is real smoke for it to mis-sense; a registered fire or leak failure in the same zone is the real way smoke gets there, not a contrived input",
    }
}

const DOOR_SLUG_INDEX: &[(&str, usize)] =
    &[("m1l", 0), ("m2l", 1), ("m2r", 2), ("m4l", 3), ("m5l", 4), ("u1l", 5), ("cargo_16", 11), ("cargo_17", 12)];

fn door_proximity() -> Provocation {
    Provocation {
        applies: |f| f.component.starts_with("52_doors.prox_"),
        provoke: |f, _registry| {
            let rest = f.component.strip_prefix("52_doors.prox_").unwrap_or("");
            let slug = rest.strip_suffix("_open").or_else(|| rest.strip_suffix("_closed")).unwrap_or(rest);
            let index = DOOR_SLUG_INDEX.iter().find(|(s, _)| *s == slug).map(|&(_, i)| i);
            let truth: Option<Box<dyn Fn(&mut Truth)>> = index.map(|i| -> Box<dyn Fn(&mut Truth)> {
                Box::new(move |t: &mut Truth| t.door_open_fraction[i] = 1.0)
            });
            Provoked { later: None, profile: "cruise", truth, companions: Vec::new(), frames: 10 }
        },
        why: "a door proximity sensor's rigging/stuck faults only show once the door is actually away from where it started; Truth::door_open_fraction moving is the real input a flight produces, the same travel src/doors.rs already animates",
    }
}

fn vibration_pickup() -> Provocation {
    Provocation {
        applies: |f| f.component.starts_with("77_eng.vibration_") && f.model_field.contains("faults.stuck"),
        provoke: |f, registry| {
            let rest = f.component.trim_start_matches("77_eng.vibration_");
            let (engine, location) = rest.split_once('_').unwrap_or((rest, "fan"));
            let rotor = if location == "core" { "hp" } else { "fan" };
            let companions = failure_on(registry, &format!("77_vib.{rotor}_rotor_{engine}"), "blade loss").map(|id| vec![(id, 1.0)]).unwrap_or_default();
            Provoked { later: None, profile: "cruise", truth: None, companions, frames: 10 }
        },
        why: "a stuck vibration pickup hides a real vibration rise: a blade-loss unbalance on the rotor it sits over, at cruise speed",
    }
}

fn pitot_drain_blocked() -> Provocation {
    Provocation {
        applies: |f| f.component.starts_with("34_nav.pitot_") && f.model_field.contains("faults.drain_blocked"),
        provoke: |f, registry| {
            let companions =
                failure_on(registry, &f.component, "insect/tape").map(|id| vec![(id, 1.0)]).unwrap_or_default();
            Provoked { later: None, profile: "cruise", truth: None, companions, frames: 10 }
        },
        why: "the drain hole's own fault changes the probe's behaviour only once the tube itself is blocked too (registry.rs's own registered effect); a companion tube blockage on the same probe is the real co-occurring condition",
    }
}

fn static_port_leak_to_cabin() -> Provocation {
    Provocation {
        applies: |f| f.component.starts_with("34_nav.static_") && f.model_field.contains("faults.leak_to_cabin"),
        provoke: |_f, _registry| {
            let truth: Option<Box<dyn Fn(&mut Truth)>> =
                Some(Box::new(|t: &mut Truth| t.cabin_pressure_pa = 85_000.0));
            Provoked { later: None, profile: "cruise", truth, companions: Vec::new(), frames: 5 }
        },
        why: "a leak toward the cabin only biases the reading once cabin pressure really differs from ambient static pressure, which is what being pressurised at altitude really produces",
    }
}

fn radio_altimeter_degradation() -> Provocation {
    Provocation {
        applies: |f| {
            f.component.starts_with("34_nav.ra_")
                && (f.model_field.contains("faults.tracking_loop_degradation")
                    || f.model_field.contains("faults.rx_antenna_degradation"))
        },
        provoke: |_f, _registry| {
            let truth: Option<Box<dyn Fn(&mut Truth)>> = Some(Box::new(|t: &mut Truth| {
                t.on_ground = false;
                t.altitude_ft = 800.0;
            }));
            Provoked { later: None, profile: "cruise", truth, companions: Vec::new(), frames: 20 }
        },
        why: "tracking-loop filter and receive-antenna degradation only add extra multipath jitter while the radio altimeter is actually in range and tracking the surface; a real low altitude is that condition",
    }
}

fn ice_detector_heater() -> Provocation {
    Provocation {
        applies: |f| f.component.starts_with("30_ice.detector_") && f.model_field.contains("faults.heater_failure"),
        provoke: |_f, _registry| {
            let in_cloud = |t: &mut Truth| {
                let mut sample = crate::deep::weather::WeatherSample::default();
                sample.clouds[0] = crate::deep::weather::WeatherCloudLayer { cloud_type: 1.0, coverage: 1.0, alt_base_m: 0.0, alt_top_m: 0.0 };
                t.environment.weather = Some(sample);
            };
            Provoked { later: None, profile: "icing_climb", truth: Some(Box::new(in_cloud)), companions: Vec::new(), frames: 24 }
        },
        why: "a deice heater failure only shows in real icing conditions, where the healthy detector cycles detect/deice and the faulted one latches and keeps accumulating instead: icing_climb flown inside a freezing cloud",
    }
}

fn tat_probe_3_recovery() -> Provocation {
    Provocation {
        applies: |f| f.component == "34_nav.tat_3" && f.model_field.contains("faults.recovery_degradation"),
        provoke: |_f, _registry| {
            Provoked { later: None, profile: "cruise", truth: None, companions: Vec::new(), frames: 3 }
        },
        why: "TAT probe 3's recovery-factor degradation is published as a plain boolean discrete (the same convention its own heater_failure already uses), so arming it is itself the whole condition",
    }
}
