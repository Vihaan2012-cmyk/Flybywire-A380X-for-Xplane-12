use crate::deep::api::*;

const ATA_FIRE_PROTECTION: u16 = 26;
const ATA_ICE_RAIN: u16 = 30;

const ZONES: [&str; 9] = ["eng1", "eng2", "eng3", "eng4", "apu", "mlg", "cargo_fwd", "cargo_aft", "avionics"];
const ZONE_TITLES: [&str; 9] = ["ENG 1", "ENG 2", "ENG 3", "ENG 4", "APU", "MLG BAY", "CARGO FWD", "CARGO AFT", "AVIONICS"];
const ZONE_MAX_LEAK_KG_S: [f64; 9] = [0.05, 0.05, 0.05, 0.05, 0.03, 0.01, 0.02, 0.02, 0.005];

pub fn register(r: &mut Registry) {
    register_fire_detection_loops(r);
    register_combustion_zones(r);
    register_extinguishing(r);
    register_anti_ice(r);
    register_ecam(r);
}

fn register_fire_detection_loops(r: &mut Registry) {
    let mut n = 0u16;
    for (zi, zone) in ZONES.iter().enumerate() {
        for loop_letter in ["a", "b"] {
            let component_id = format!("26_fire.{zone}_loop_{loop_letter}");
            n += 1;
            let open_id = failure_id(Area::FireIce, ATA_FIRE_PROTECTION, n);
            n += 1;
            let short_id = failure_id(Area::FireIce, ATA_FIRE_PROTECTION, n);

            r.component(ComponentDef {
                id: component_id.clone(),
                area: Area::FireIce,
                ata: ATA_FIRE_PROTECTION,
                name: format!("{} fire detection loop {}", ZONE_TITLES[zi], loop_letter.to_uppercase()),
                params: vec![
                    ParamDef { name: "open_circuit".into(), meaning: "loop reading driven out of physical range (broken conductor/ruptured pneumatic tube): 0 healthy .. 1 fully open".into(), healthy: 0.0 },
                    ParamDef { name: "short_circuit".into(), meaning: "loop shorted/internal leak: reads indistinguishably from real heat (false fire): 0 healthy .. 1 dead short".into(), healthy: 0.0 },
                ],
                failures: vec![open_id, short_id],
            });

            r.failure(FailureDef {
                id: open_id,
                area: Area::FireIce,
                ata: ATA_FIRE_PROTECTION,
                name: format!("{} fire loop {} open circuit", ZONE_TITLES[zi], loop_letter.to_uppercase()),
                component: component_id.clone(),
                model_field: "deep::fire_ice::fire_loops::LoopFaults.open_circuit".into(),
                magnitude: "0..1, blends the loop's resistance/pressure reading toward the out-of-physical-range fault value".into(),
                effect: "ZoneDetector reports loop_x_fault=true; FDU falls back to trusting the other loop alone (or fails toward presumed fire if both loops fault together)".into(),
            });
            r.failure(FailureDef {
                id: short_id,
                area: Area::FireIce,
                ata: ATA_FIRE_PROTECTION,
                name: format!("{} fire loop {} short circuit", ZONE_TITLES[zi], loop_letter.to_uppercase()),
                component: component_id,
                model_field: "deep::fire_ice::fire_loops::LoopFaults.short_circuit".into(),
                magnitude: "0..1, blends the loop's resistance/pressure reading toward the fire-mimicking short value".into(),
                effect: "loop reports fire_signal=true indistinguishably from a real fire; under OR logic (or once the other loop is also faulted) the zone is falsely declared on fire".into(),
            });
        }
    }
}

fn register_combustion_zones(r: &mut Registry) {
    for (zi, zone) in ZONES.iter().enumerate() {
        let component_id = format!("26_fire.{zone}_leak_source");
        let fid = failure_id(Area::FireIce, ATA_FIRE_PROTECTION, 100 + zi as u16);

        r.component(ComponentDef {
            id: component_id.clone(),
            area: Area::FireIce,
            ata: ATA_FIRE_PROTECTION,
            name: format!("{} flammable-fluid leak (fire source)", ZONE_TITLES[zi]),
            params: vec![ParamDef {
                name: "leak_severity".into(),
                meaning: format!("fraction of the zone's maximum modelled leak rate ({} kg/s at 1.0): 0 none .. 1 max leak", ZONE_MAX_LEAK_KG_S[zi]),
                healthy: 0.0,
            }],
            failures: vec![fid],
        });

        r.failure(FailureDef {
            id: fid,
            area: Area::FireIce,
            ata: ATA_FIRE_PROTECTION,
            name: format!("{} fuel/oil/hydraulic leak feeding a fire", ZONE_TITLES[zi]),
            component: component_id,
            model_field: "deep::fire_ice::combustion::ZoneSupply.fuel_available_kg_s".into(),
            magnitude: format!("0..1, scales the leak rate 0..{} kg/s feeding ZoneCombustion", ZONE_MAX_LEAK_KG_S[zi]),
            effect: "with an ignition source (or the zone already above its fluid's autoignition point from a neighbour's fire) this fuel/air-limited leak sustains combustion and raises the zone's own temperature, which can in turn ignite a neighbouring zone through its conductive link".into(),
        });
    }
}

fn register_extinguishing(r: &mut Registry) {
    let mut n = 200u16;

    for eng in 1..=4u16 {
        for bottle in 1..=2u16 {
            n += 1;
            register_bottle(r, &mut n, &format!("26_fire.eng{eng}_bottle{bottle}"), &format!("ENG {eng} fire bottle {bottle}"), None);
        }
    }
    n += 1;
    register_bottle(r, &mut n, "26_fire.apu_bottle1", "APU fire bottle", None);
    for (field, title) in [("cargo_fwd_bottle", "Cargo FWD"), ("cargo_aft_bottle", "Cargo AFT")] {
        let component_id = format!("26_fire.{field}");
        n += 1;
        let leak_id = failure_id(Area::FireIce, ATA_FIRE_PROTECTION, n);
        n += 1;
        let knockdown_id = failure_id(Area::FireIce, ATA_FIRE_PROTECTION, n);
        n += 1;
        let extended_id = failure_id(Area::FireIce, ATA_FIRE_PROTECTION, n);
        n += 1;
        let distribution_id = failure_id(Area::FireIce, ATA_FIRE_PROTECTION, n);

        r.component(ComponentDef {
            id: component_id.clone(),
            area: Area::FireIce,
            ata: ATA_FIRE_PROTECTION,
            name: format!("{title} suppression bottle"),
            params: vec![
                ParamDef { name: "leak".into(), meaning: "slow continuous agent leak from the valve seat/body: 0 healthy .. 1 max modelled leak orifice".into(), healthy: 0.0 },
                ParamDef { name: "knockdown_squib_fault".into(), meaning: "the knockdown (bottle 1) squib fails to fully rupture its disc: 0 healthy .. 1 no knockdown discharge at all".into(), healthy: 0.0 },
                ParamDef { name: "extended_squib_fault".into(), meaning: "the extended (bottle 2) squib fails to fully rupture its disc: 0 healthy .. 1 no metered/extended discharge at all".into(), healthy: 0.0 },
                ParamDef { name: "distribution_fault".into(), meaning: "the agent-distribution line/valve from the bottle manifold to this hold: 0 healthy .. 1 no agent reaches the hold regardless of either bottle".into(), healthy: 0.0 },
            ],
            failures: vec![leak_id, knockdown_id, extended_id, distribution_id],
        });
        r.failure(FailureDef {
            id: leak_id,
            area: Area::FireIce,
            ata: ATA_FIRE_PROTECTION,
            name: format!("{title} suppression bottle leak"),
            component: component_id.clone(),
            model_field: "deep::fire_ice::extinguishing::CargoSuppressionFaults.leak".into(),
            magnitude: "0..1, scales the leak orifice area; a full-severity leak empties the bottle over roughly 9-10 hours".into(),
            effect: "bottle mass/pressure fall over time; if not caught before use, delivers less agent (or none) when actually fired".into(),
        });
        r.failure(FailureDef {
            id: knockdown_id,
            area: Area::FireIce,
            ata: ATA_FIRE_PROTECTION,
            name: format!("{title} knockdown (bottle 1) squib failure"),
            component: component_id.clone(),
            model_field: "deep::fire_ice::extinguishing::CargoSuppressionFaults.knockdown_squib_fault".into(),
            magnitude: "0..1, reduces the achieved knockdown-stage discharge orifice area; at 1.0 the knockdown stage never discharges at all".into(),
            effect: "reduced or (at 1.0) zero agent delivered during the initial high-rate knockdown discharge, PUSH_OVHD_CARGOSMOKE_FWD/_AFT (W194), regardless of the extended stage's own health".into(),
        });
        r.failure(FailureDef {
            id: extended_id,
            area: Area::FireIce,
            ata: ATA_FIRE_PROTECTION,
            name: format!("{title} extended (bottle 2) squib failure"),
            component: component_id.clone(),
            model_field: "deep::fire_ice::extinguishing::CargoSuppressionFaults.extended_squib_fault".into(),
            magnitude: "0..1, reduces the achieved metered/extended-stage discharge orifice area; at 1.0 the extended stage never discharges at all".into(),
            effect: "reduced or (at 1.0) zero agent delivered once the system has switched to the metered/extended stage, regardless of the knockdown stage's own health".into(),
        });
        r.failure(FailureDef {
            id: distribution_id,
            area: Area::FireIce,
            ata: ATA_FIRE_PROTECTION,
            name: format!("{title} agent distribution path fault"),
            component: component_id,
            model_field: "deep::fire_ice::extinguishing::CargoSuppressionFaults.distribution_fault".into(),
            magnitude: "0..1, gates the effective discharge orifice area the same way either squib fault does".into(),
            effect: "reduced or (at 1.0) zero agent reaches the hold even with both bottles/squibs fully healthy".into(),
        });
    }

    for bay in ["fwd", "aft"] {
        n += 1;
        let fid = failure_id(Area::FireIce, ATA_FIRE_PROTECTION, n);
        let component_id = format!("26_fire.cargo_{bay}_smoke_detector");
        r.component(ComponentDef {
            id: component_id.clone(),
            area: Area::FireIce,
            ata: ATA_FIRE_PROTECTION,
            name: format!("Cargo {} optical smoke detector", bay.to_uppercase()),
            params: vec![ParamDef { name: "lens_obscured".into(), meaning: "optical chamber dirty/obscured: 0 healthy .. 1 fully blind".into(), healthy: 0.0 }],
            failures: vec![fid],
        });
        r.failure(FailureDef {
            id: fid,
            area: Area::FireIce,
            ata: ATA_FIRE_PROTECTION,
            name: format!("Cargo {} smoke detector lens obscured", bay.to_uppercase()),
            component: component_id,
            model_field: "deep::fire_ice::extinguishing::SmokeDetectorFaults.lens_obscured".into(),
            magnitude: "0..1, derates the detector's effective smoke density used for the alarm obscuration threshold".into(),
            effect: "delays, and at 1.0 fully prevents, a real smoke alarm despite genuine smoke accumulating".into(),
        });
    }

    n += 1;
    let fid = failure_id(Area::FireIce, ATA_FIRE_PROTECTION, n);
    let component_id = "26_fire.lavatory_extinguisher".to_string();
    r.component(ComponentDef {
        id: component_id.clone(),
        area: Area::FireIce,
        ata: ATA_FIRE_PROTECTION,
        name: "Lavatory trash-bin fusible-link extinguisher (xN lavatories, GENERIC count)".into(),
        params: vec![ParamDef { name: "link_degraded".into(), meaning: "aged/corroded fusible link needs a higher temperature to melt: 0 healthy (77 C) .. 1 fully degraded (+50 C)".into(), healthy: 0.0 }],
        failures: vec![fid],
    });
    r.failure(FailureDef {
        id: fid,
        area: Area::FireIce,
        ata: ATA_FIRE_PROTECTION,
        name: "Lavatory fusible link degraded".into(),
        component: component_id,
        model_field: "deep::fire_ice::extinguishing::LavatoryFaults.link_degraded".into(),
        magnitude: "0..1, raises the link's effective melt temperature up to 50 C above its 77 C design rating".into(),
        effect: "delays the automatic extinguisher's discharge past the design trigger temperature".into(),
    });

    for bottle in ["1", "2"] {
        n += 1;
        let fid = failure_id(Area::FireIce, ATA_FIRE_PROTECTION, n);
        let component_id = format!("26_fire.ldcr_bottle_{bottle}");
        r.component(ComponentDef {
            id: component_id.clone(),
            area: Area::FireIce,
            ata: ATA_FIRE_PROTECTION,
            name: format!("FWD Lower Crew Rest (LDCR) suppression bottle {bottle}"),
            params: vec![ParamDef { name: "squib_fault".into(), meaning: "0 healthy .. 1 (any nonzero) this bottle's own squib circuit reports faulted".into(), healthy: 0.0 }],
            failures: vec![fid],
        });
        r.failure(FailureDef {
            id: fid,
            area: Area::FireIce,
            ata: ATA_FIRE_PROTECTION,
            name: format!("LDCR bottle {bottle} squib circuit fault"),
            component: component_id,
            model_field: "deep::fire_ice::live::FireIceLive.ldcr_bottle_squib_fault".into(),
            magnitude: "0..1, any nonzero arms the discrete".into(),
            effect: format!("FIRE_LDCR_BTL_{bottle}_SQUIB_FAULT goes true, independent of whether the module has ever been commanded to discharge (FCOM PRO-ABN-ECAM p.4985)"),
        });
    }

    n += 1;
    let fid = failure_id(Area::FireIce, ATA_FIRE_PROTECTION, n);
    let component_id = "26_fire.lavatory_bin".to_string();
    r.component(ComponentDef {
        id: component_id.clone(),
        area: Area::FireIce,
        ata: ATA_FIRE_PROTECTION,
        name: "Lavatory 1 waste bin".into(),
        params: vec![ParamDef { name: "bin_fire".into(), meaning: "0 none .. 1 a fully developed waste-bin fire".into(), healthy: 0.0 }],
        failures: vec![fid],
    });
    r.failure(FailureDef {
        id: fid,
        area: Area::FireIce,
        ata: ATA_FIRE_PROTECTION,
        name: "Lavatory 1 waste bin fire".into(),
        component: component_id,
        model_field: "deep::fire_ice::live::FireIceLive.lav_bin_fire".into(),
        magnitude: "0..1, the fire's burn rate".into(),
        effect: "heats the bin until the fusible link melts and the extinguisher puts it out; its smoke reaches lavatory 1's smoke detector (LAV SMOKE) until then".into(),
    });
}

fn register_bottle(r: &mut Registry, n: &mut u16, component_id: &str, title: &str, squib_effect_override: Option<&str>) {
    let leak_id = failure_id(Area::FireIce, ATA_FIRE_PROTECTION, *n);
    *n += 1;
    let squib_id = failure_id(Area::FireIce, ATA_FIRE_PROTECTION, *n);

    r.component(ComponentDef {
        id: component_id.to_string(),
        area: Area::FireIce,
        ata: ATA_FIRE_PROTECTION,
        name: title.to_string(),
        params: vec![
            ParamDef { name: "leak".into(), meaning: "slow continuous agent leak from the valve seat/body: 0 healthy .. 1 max modelled leak orifice".into(), healthy: 0.0 },
            ParamDef { name: "squib_failure".into(), meaning: "pyrotechnic squib fails to fully rupture its disc: 0 healthy .. 1 no discharge at all when fired".into(), healthy: 0.0 },
        ],
        failures: vec![leak_id, squib_id],
    });
    r.failure(FailureDef {
        id: leak_id,
        area: Area::FireIce,
        ata: ATA_FIRE_PROTECTION,
        name: format!("{title} leak"),
        component: component_id.to_string(),
        model_field: "deep::fire_ice::extinguishing::BottleFaults.leak".into(),
        magnitude: "0..1, scales the leak orifice area; a full-severity leak empties the bottle over roughly 9-10 hours".into(),
        effect: "bottle mass/pressure fall over time; if not caught before use, delivers less agent (or none) when actually fired".into(),
    });
    r.failure(FailureDef {
        id: squib_id,
        area: Area::FireIce,
        ata: ATA_FIRE_PROTECTION,
        name: format!("{title} squib failure"),
        component: component_id.to_string(),
        model_field: "deep::fire_ice::extinguishing::BottleFaults.squib_failure".into(),
        magnitude: "0..1, reduces the achieved discharge orifice area; at 1.0 the disc never ruptures at all".into(),
        effect: squib_effect_override.unwrap_or("reduced or (at 1.0) zero agent delivered into the zone when fired, regardless of a correct fire pushbutton/agent pushbutton sequence").into(),
    });
}

fn register_anti_ice(r: &mut Registry) {
    let mut n = 0u16;

    for (zone, title) in [("wing_l", "L WING"), ("wing_r", "R WING")] {
        register_bleed_anti_ice(r, &mut n, zone, title);
    }
    for eng in 1..=4u16 {
        register_bleed_anti_ice(r, &mut n, &format!("nacelle{eng}"), &format!("ENG {eng} NACELLE"));
    }

    for probe in ["pitot1", "pitot2", "pitot3", "aoa1", "aoa2", "aoa3", "tat1", "tat2"] {
        register_probe_heater(r, &mut n, probe);
    }

    register_window_heat(r, &mut n, "l_windshield", "L WINDSHIELD");
    register_window_heat(r, &mut n, "r_windshield", "R WINDSHIELD");

    register_rain_removal(r, &mut n, "l_windshield", "L WINDSHIELD");
    register_rain_removal(r, &mut n, "r_windshield", "R WINDSHIELD");
}

fn register_bleed_anti_ice(r: &mut Registry, n: &mut u16, zone: &str, title: &str) {
    let component_id = format!("30_ice.{zone}_anti_ice_valve");
    *n += 1;
    let closed_id = failure_id(Area::FireIce, ATA_ICE_RAIN, *n);
    *n += 1;
    let open_id = failure_id(Area::FireIce, ATA_ICE_RAIN, *n);
    *n += 1;
    let leak_id = failure_id(Area::FireIce, ATA_ICE_RAIN, *n);

    r.component(ComponentDef {
        id: component_id.clone(),
        area: Area::FireIce,
        ata: ATA_ICE_RAIN,
        name: format!("{title} anti-ice bleed valve/duct"),
        params: vec![
            ParamDef { name: "valve_stuck_closed".into(), meaning: "valve fails toward closed: 0 healthy .. 1 no flow regardless of command".into(), healthy: 0.0 },
            ParamDef { name: "valve_stuck_open".into(), meaning: "valve fails toward open: 0 healthy .. 1 full flow regardless of command".into(), healthy: 0.0 },
            ParamDef { name: "duct_leak".into(), meaning: "upstream duct leak: 0 healthy .. 1 all commanded flow lost before the piccolo tube".into(), healthy: 0.0 },
        ],
        failures: vec![closed_id, open_id, leak_id],
    });
    r.failure(FailureDef {
        id: closed_id,
        area: Area::FireIce,
        ata: ATA_ICE_RAIN,
        name: format!("{title} anti-ice valve stuck closed"),
        component: component_id.clone(),
        model_field: "deep::fire_ice::anti_ice::BleedAntiIceFaults.valve_stuck_closed".into(),
        magnitude: "0..1, reduces commanded bleed flow toward zero".into(),
        effect: "the leading edge gets no anti-ice heat and ices exactly as an unheated surface would (icing.rs's own Messinger balance)".into(),
    });
    r.failure(FailureDef {
        id: open_id,
        area: Area::FireIce,
        ata: ATA_ICE_RAIN,
        name: format!("{title} anti-ice valve stuck open"),
        component: component_id.clone(),
        model_field: "deep::fire_ice::anti_ice::BleedAntiIceFaults.valve_stuck_open".into(),
        magnitude: "0..1, floors delivered bleed flow at full regardless of command".into(),
        effect: "continues delivering full bleed heat once icing conditions/demand end, driving the skin/duct temperature into an overheat trip (OVERHEAT_TRIP_C)".into(),
    });
    r.failure(FailureDef {
        id: leak_id,
        area: Area::FireIce,
        ata: ATA_ICE_RAIN,
        name: format!("{title} anti-ice duct leak"),
        component: component_id,
        model_field: "deep::fire_ice::anti_ice::BleedAntiIceFaults.duct_leak".into(),
        magnitude: "0..1, fraction of commanded bleed flow lost before reaching the piccolo tube/heated skin".into(),
        effect: "reduced bleed heat delivery weakens anti-ice protection, allowing partial ice accretion under otherwise-protected conditions".into(),
    });
}

fn register_probe_heater(r: &mut Registry, n: &mut u16, probe: &str) {
    let component_id = format!("30_ice.{probe}_heater");
    *n += 1;
    let open_id = failure_id(Area::FireIce, ATA_ICE_RAIN, *n);
    *n += 1;
    let controller_id = failure_id(Area::FireIce, ATA_ICE_RAIN, *n);
    *n += 1;
    let sensor_id = failure_id(Area::FireIce, ATA_ICE_RAIN, *n);

    r.component(ComponentDef {
        id: component_id.clone(),
        area: Area::FireIce,
        ata: ATA_ICE_RAIN,
        name: format!("{} probe heater", probe.to_uppercase()),
        params: vec![
            ParamDef { name: "heater_open_circuit".into(), meaning: "heater element degraded/open winding: 0 healthy .. 1 no power deliverable".into(), healthy: 0.0 },
            ParamDef { name: "controller_fault".into(), meaning: "heater controller fails to command power at all: 0 healthy .. 1 (>=1.0) fully faulted".into(), healthy: 0.0 },
            ParamDef { name: "sensor_fault".into(), meaning: "temperature feedback sticks at a fixed warm reading: 0 healthy .. 1 fully stuck".into(), healthy: 0.0 },
        ],
        failures: vec![open_id, controller_id, sensor_id],
    });
    r.failure(FailureDef {
        id: open_id,
        area: Area::FireIce,
        ata: ATA_ICE_RAIN,
        name: format!("{} probe heater open circuit", probe.to_uppercase()),
        component: component_id.clone(),
        model_field: "deep::fire_ice::anti_ice::ProbeHeaterFaults.heater_open_circuit".into(),
        magnitude: "0..1, fraction reduction of rated power actually deliverable".into(),
        effect: "probe cannot hold above freezing in icing conditions and ices, risking blockage".into(),
    });
    r.failure(FailureDef {
        id: controller_id,
        area: Area::FireIce,
        ata: ATA_ICE_RAIN,
        name: format!("{} probe heater controller fault", probe.to_uppercase()),
        component: component_id.clone(),
        model_field: "deep::fire_ice::anti_ice::ProbeHeaterFaults.controller_fault".into(),
        magnitude: "0..1, at 1.0 the controller never commands power".into(),
        effect: "heater never energises even in icing conditions; probe ices".into(),
    });
    r.failure(FailureDef {
        id: sensor_id,
        area: Area::FireIce,
        ata: ATA_ICE_RAIN,
        name: format!("{} probe heater sensor fault", probe.to_uppercase()),
        component: component_id,
        model_field: "deep::fire_ice::anti_ice::ProbeHeaterFaults.sensor_fault".into(),
        magnitude: "0..1, blends the sensed temperature toward a fixed stuck-warm reading".into(),
        effect: "silent failure: the controller believes the probe is warm enough and withholds heat while the probe genuinely ices".into(),
    });
}

fn register_window_heat(r: &mut Registry, n: &mut u16, zone: &str, title: &str) {
    let component_id = format!("30_ice.{zone}_heat_film");
    *n += 1;
    let defect_id = failure_id(Area::FireIce, ATA_ICE_RAIN, *n);
    *n += 1;
    let controller_id = failure_id(Area::FireIce, ATA_ICE_RAIN, *n);
    *n += 1;
    let sensor_id = failure_id(Area::FireIce, ATA_ICE_RAIN, *n);

    r.component(ComponentDef {
        id: component_id.clone(),
        area: Area::FireIce,
        ata: ATA_ICE_RAIN,
        name: format!("{title} heated-film windshield"),
        params: vec![
            ParamDef { name: "film_defect".into(), meaning: "conductive-film resistance defect (nick/corrosion): 0 healthy .. 1 (clamped 0.995) severe local concentration".into(), healthy: 0.0 },
            ParamDef { name: "controller_fault".into(), meaning: "heat controller fault: 0 healthy .. 1 stuck full-on with no working overheat cutout".into(), healthy: 0.0 },
            ParamDef { name: "sensor_fault".into(), meaning: "temperature feedback sticks at a fixed warm reading: 0 healthy .. 1 fully stuck".into(), healthy: 0.0 },
        ],
        failures: vec![defect_id, controller_id, sensor_id],
    });
    r.failure(FailureDef {
        id: defect_id,
        area: Area::FireIce,
        ata: ATA_ICE_RAIN,
        name: format!("{title} film defect"),
        component: component_id.clone(),
        model_field: "deep::fire_ice::anti_ice::WindowHeatFaults.film_defect".into(),
        magnitude: "0..1 (clamped 0.995), concentrates nameplate power by 1/(1-defect)^2 into a local hot spot".into(),
        effect: "a severe defect drives the local hot spot past the delamination and crack damage thresholds".into(),
    });
    r.failure(FailureDef {
        id: controller_id,
        area: Area::FireIce,
        ata: ATA_ICE_RAIN,
        name: format!("{title} heat controller fault"),
        component: component_id.clone(),
        model_field: "deep::fire_ice::anti_ice::WindowHeatFaults.controller_fault".into(),
        magnitude: "0..1, at 1.0 the heater is commanded full-on unconditionally with no working overheat cutout".into(),
        effect: "the film runs away to an overheat condition (bulk surface exceeds WINDOW_OVERHEAT_PROTECT_C)".into(),
    });
    r.failure(FailureDef {
        id: sensor_id,
        area: Area::FireIce,
        ata: ATA_ICE_RAIN,
        name: format!("{title} heat sensor fault"),
        component: component_id,
        model_field: "deep::fire_ice::anti_ice::WindowHeatFaults.sensor_fault".into(),
        magnitude: "0..1, blends the sensed temperature toward a fixed stuck-warm reading".into(),
        effect: "silent under-heating: the controller withholds heat believing the window already warm, leaving it unprotected against icing/fogging".into(),
    });
}

fn register_rain_removal(r: &mut Registry, n: &mut u16, zone: &str, title: &str) {
    *n += 1;
    let fid = failure_id(Area::FireIce, ATA_ICE_RAIN, *n);
    let component_id = format!("30_ice.{zone}_rain_removal");
    r.component(ComponentDef {
        id: component_id.clone(),
        area: Area::FireIce,
        ata: ATA_ICE_RAIN,
        name: format!("{title} rain removal jet"),
        params: vec![ParamDef { name: "system_fault".into(), meaning: "valve/duct/blower fault: 0 healthy .. 1 no jet velocity delivered".into(), healthy: 0.0 }],
        failures: vec![fid],
    });
    r.failure(FailureDef {
        id: fid,
        area: Area::FireIce,
        ata: ATA_ICE_RAIN,
        name: format!("{title} rain removal system fault"),
        component: component_id,
        model_field: "deep::fire_ice::anti_ice::RainRemovalFaults.system_fault".into(),
        magnitude: "0..1, reduces the jet's effective dynamic pressure/shear-removal rate".into(),
        effect: "the water film on the windshield exterior clears more slowly (or not at all), degrading visibility in rain".into(),
    });
}

fn register_ecam(r: &mut Registry) {
    r.alert(
        EcamAlert::new(
            "WING_L_A_ICE_VLV_PRESS_HI",
            ATA_ICE_RAIN,
            "A-ICE L WING VLV PRESS HI",
            Level::Caution,
            var("ANTI_ICE_WING_L_OVERHEAT").on(),
        )
        .confirm(2.0)
        .step(line("WING A-ICE", "OFF").done(var("ANTI_ICE_WING_L_VALVE_OPEN").off()))
        .status_line("INOP: WING A ICE")
        .raised_by(&[failure_id(Area::FireIce, ATA_ICE_RAIN, 2)]),
    );
    r.alert(
        EcamAlert::new(
            "WING_R_A_ICE_VLV_PRESS_HI",
            ATA_ICE_RAIN,
            "A-ICE R WING VLV PRESS HI",
            Level::Caution,
            var("ANTI_ICE_WING_R_OVERHEAT").on(),
        )
        .confirm(2.0)
        .step(line("WING A-ICE", "OFF").done(var("ANTI_ICE_WING_R_VALVE_OPEN").off()))
        .status_line("INOP: WING A ICE")
        .raised_by(&[failure_id(Area::FireIce, ATA_ICE_RAIN, 5)]),
    );
}
