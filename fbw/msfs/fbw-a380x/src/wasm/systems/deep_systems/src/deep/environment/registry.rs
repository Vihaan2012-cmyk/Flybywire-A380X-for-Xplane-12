use crate::deep::api::*;

pub fn register(r: &mut Registry) {
    register_bird_strike(r);
    register_lightning(r);
    register_hail(r);
    register_volcanic_ash(r);
    register_ice_crystal_icing(r);
    register_runway_contamination(r);
}

fn register_bird_strike(r: &mut Registry) {
    let mut fan_ids = [0u64; 4];
    let mut core_ids = [0u64; 4];
    const FAN_SEQ: [u16; 4] = [1, 8, 9, 10];
    const CORE_SEQ: [u16; 4] = [2, 11, 12, 13];
    for eng in 1..=4u16 {
        let fan_component = format!("72_env.fan_bird_damage_{eng}");
        r.component(ComponentDef {
            id: fan_component.clone(),
            area: Area::Environment,
            ata: 72,
            name: format!("Engine {eng} fan blade set (bird-strike damage state)"),
            params: vec![ParamDef { name: "damage_frac".into(), meaning: "0 undamaged .. 1 destructive (fan-blade fracture/imbalance)".into(), healthy: 0.0 }],
            failures: vec![],
        });
        fan_ids[(eng - 1) as usize] = r.failure(FailureDef {
            id: failure_id(Area::Environment, 72, FAN_SEQ[(eng - 1) as usize]),
            area: Area::Environment,
            ata: 72,
            name: if eng == 1 { "Bird strike fan blade damage".into() } else { format!("Engine {eng} bird strike fan blade damage") },
            component: fan_component,
            model_field: "environment::bird_strike::StrikeOutcome.fan_damage_frac".into(),
            magnitude: "0 no damage .. 1 destructive blade fracture/imbalance, from impact energy vs. the CS-E 800 large-bird reference energy".into(),
            effect: "engine vibration, thrust loss, possible surge/flameout (consumed by the engine model)".into(),
        });

        let core_component = format!("72_env.core_fod_{eng}");
        r.component(ComponentDef {
            id: core_component.clone(),
            area: Area::Environment,
            ata: 72,
            name: format!("Engine {eng} IP compressor front stage (foreign-object ingestion state)"),
            params: vec![ParamDef { name: "ingested_mass_frac".into(), meaning: "fraction of the strike's mass that entered the core flow path vs. the bypass duct".into(), healthy: 0.0 }],
            failures: vec![],
        });
        core_ids[(eng - 1) as usize] = r.failure(FailureDef {
            id: failure_id(Area::Environment, 72, CORE_SEQ[(eng - 1) as usize]),
            area: Area::Environment,
            ata: 72,
            name: if eng == 1 { "Bird strike core ingestion FOD".into() } else { format!("Engine {eng} bird strike core ingestion FOD") },
            component: core_component,
            model_field: "environment::bird_strike::StrikeOutcome.core_ingestion_frac".into(),
            magnitude: "fraction of ingested bird mass reaching the IP compressor front stage (bypass-ratio flow split)".into(),
            effect: "IP compressor blade damage, compressor efficiency loss".into(),
        });
    }

    let mut windshield_ids = [0u64; 6];
    const WINDSHIELD_SEQ: [u16; 6] = [1, 4, 5, 6, 7, 8];
    for p in 1..=6u16 {
        let windshield_component = format!("56_env.windshield_bird_{p}");
        r.component(ComponentDef {
            id: windshield_component.clone(),
            area: Area::Environment,
            ata: 56,
            name: format!("Flight-deck windshield panel {p} (bird-strike damage state)"),
            params: vec![ParamDef { name: "damage_frac".into(), meaning: "0 undamaged, >0.6 cracked, >=1.0 penetrated".into(), healthy: 0.0 }],
            failures: vec![],
        });
        windshield_ids[(p - 1) as usize] = r.failure(FailureDef {
            id: failure_id(Area::Environment, 56, WINDSHIELD_SEQ[(p - 1) as usize]),
            area: Area::Environment,
            ata: 56,
            name: if p == 1 { "Bird strike windshield damage".into() } else { format!("Bird strike windshield damage (panel {p})") },
            component: windshield_component,
            model_field: "environment::bird_strike::StrikeOutcome.windshield_crack / .windshield_penetrated".into(),
            magnitude: "impact energy / CS-25.775(b) 4 lb-at-Vc reference energy, clamped 0..1; >0.6 cracks, >=1.0 penetrates".into(),
            effect: "loss of visibility, possible depressurisation if penetrated".into(),
        });
    }

    r.component(ComponentDef {
        id: "53_env.radome".into(),
        area: Area::Environment,
        ata: 53,
        name: "Nose radome".into(),
        params: vec![ParamDef { name: "damage_frac".into(), meaning: "0 undamaged .. 1 destroyed/departed".into(), healthy: 0.0 }],
        failures: vec![],
    });

    let mut wing_le_ids = [0u64; 12];
    const WING_LE_SEQ: [u16; 12] = [1, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13];
    for s in 1..=12u16 {
        let wing_le_component = format!("57_env.wing_leading_edge_bird_{s}");
        r.component(ComponentDef {
            id: wing_le_component.clone(),
            area: Area::Environment,
            ata: 57,
            name: format!("Wing leading edge segment {s} (6 per side, root to tip), L/R (bird-strike damage state)"),
            params: vec![ParamDef { name: "dent_drag_delta_cd".into(), meaning: "local drag-coefficient increment from a dent's separated flow".into(), healthy: 0.0 }],
            failures: vec![],
        });
        wing_le_ids[(s - 1) as usize] = r.failure(FailureDef {
            id: failure_id(Area::Environment, 57, WING_LE_SEQ[(s - 1) as usize]),
            area: Area::Environment,
            ata: 57,
            name: if s == 1 { "Bird strike leading edge dent".into() } else { format!("Bird strike leading edge dent (segment {s})") },
            component: wing_le_component,
            model_field: "environment::bird_strike::StrikeOutcome.leading_edge_dent_drag_delta_cd".into(),
            magnitude: "impact energy / CS-25.631 8 lb-at-Vc reference energy, clamped 0..1".into(),
            effect: "local drag increment; at 1.0 implies a structural inspection item".into(),
        });
    }

    r.component(ComponentDef {
        id: "32_env.nose_gear".into(),
        area: Area::Environment,
        ata: 32,
        name: "Nose landing gear (bird-strike damage state)".into(),
        params: vec![ParamDef { name: "damage_frac".into(), meaning: "0 undamaged .. 1 unsafe to extend/retract or steer".into(), healthy: 0.0 }],
        failures: vec![],
    });

    let mut probe_ids = [0u64; 6];
    const PROBE_SEQ: [u16; 6] = [1, 5, 6, 7, 8, 9];
    for p in 1..=6u16 {
        let probe_component = format!("34_env.air_data_probe_bird_{p}");
        r.component(ComponentDef {
            id: probe_component.clone(),
            area: Area::Environment,
            ata: 34,
            name: format!("Pitot/AoA/TAT probe {p}, forward fuselage (bird-strike damage state)"),
            params: vec![ParamDef { name: "blocked".into(), meaning: "0 clear, 1 blocked (bird strike deforms or plugs the orifice outright)".into(), healthy: 0.0 }],
            failures: vec![],
        });
        probe_ids[(p - 1) as usize] = r.failure(FailureDef {
            id: failure_id(Area::Environment, 34, PROBE_SEQ[(p - 1) as usize]),
            area: Area::Environment,
            ata: 34,
            name: if p == 1 { "Bird strike air data probe blockage".into() } else { format!("Bird strike air data probe blockage (probe {p})") },
            component: probe_component,
            model_field: "environment::bird_strike::StrikeOutcome.probe_blocked".into(),
            magnitude: "0 clear, 1 blocked (binary: any strike on a probe is assumed to disable it)".into(),
            effect: "unreliable airspeed/AoA on the affected probe".into(),
        });
    }

    let radome = r.failure(FailureDef {
        id: failure_id(Area::Environment, 53, 1),
        area: Area::Environment,
        ata: 53,
        name: "Bird strike radome damage".into(),
        component: "53_env.radome".into(),
        model_field: "environment::bird_strike::StrikeOutcome.radome_damage_frac".into(),
        magnitude: "impact energy / GENERIC radome reference energy (half the CS-25.775(b) windshield energy)".into(),
        effect: "weather radar loss, drag increase, possible radome departure".into(),
    });
    let gear = r.failure(FailureDef {
        id: failure_id(Area::Environment, 32, 1),
        area: Area::Environment,
        ata: 32,
        name: "Bird strike nose gear damage".into(),
        component: "32_env.nose_gear".into(),
        model_field: "environment::bird_strike::StrikeOutcome.nose_gear_damage_frac".into(),
        magnitude: "impact energy / CS-25.631 8 lb-at-Vc reference energy, clamped 0..1".into(),
        effect: "gear retraction/extension or steering fault at high magnitude".into(),
    });

    let _ = (&windshield_ids, radome, gear);
}

fn register_lightning(r: &mut Registry) {
    r.component(ComponentDef {
        id: "53_env.radome_lightning".into(),
        area: Area::Environment,
        ata: 53,
        name: "Nose radome (lightning damage state)".into(),
        params: vec![ParamDef { name: "damage_frac".into(), meaning: "0 undamaged .. 1 destroyed; far worse if the strike misses the diverter-strip network".into(), healthy: 0.0 }],
        failures: vec![],
    });
    r.component(ComponentDef {
        id: "53_env.composite_extremity".into(),
        area: Area::Environment,
        ata: 53,
        name: "Composite extremity skin (wingtips, tail), lightning attach/exit points".into(),
        params: vec![ParamDef { name: "damage_frac".into(), meaning: "0 undamaged .. 1 mesh/paint burn-through requiring inspection".into(), healthy: 0.0 }],
        failures: vec![],
    });
    r.component(ComponentDef {
        id: "34_env.standby_compass".into(),
        area: Area::Environment,
        ata: 34,
        name: "Standby magnetic compass".into(),
        params: vec![ParamDef { name: "deviation_deg".into(), meaning: "persistent post-strike heading error until the next compass swing".into(), healthy: 0.0 }],
        failures: vec![],
    });
    let mut bus_ids = [0u64; 7];
    const BUS_SEQ: [u16; 7] = [1, 2, 3, 4, 5, 6, 7];
    const BUS_NAMES: [&str; 7] = ["PRIM", "SEC", "FMGC", "ADIRS", "standby instruments", "FADEC", "IFE"];
    for (k, bus_name) in BUS_NAMES.iter().enumerate() {
        let bus_component = format!("24_env.bus_transient_{}", k + 1);
        r.component(ComponentDef {
            id: bus_component.clone(),
            area: Area::Environment,
            ata: 24,
            name: format!("Bus/computer conducted-transient exposure, {bus_name}"),
            params: vec![ParamDef { name: "peak_volts".into(), meaning: "induced transient this strike put on the bus; upset likely above the GENERIC 50 V threshold".into(), healthy: 0.0 }],
            failures: vec![],
        });
        bus_ids[k] = r.failure(FailureDef {
            id: failure_id(Area::Environment, 24, BUS_SEQ[k]),
            area: Area::Environment,
            ata: 24,
            name: if k == 0 { "Lightning induced bus/computer transient".into() } else { format!("Lightning induced bus/computer transient ({bus_name})") },
            component: bus_component,
            model_field: "environment::lightning::LightningEvent.transients[].peak_volts".into(),
            magnitude: "peak current x GENERIC per-bus exposure factor (0.06 shielded flight-control computers .. 0.35 nacelle-mounted FADEC)".into(),
            effect: "possible reset/data corruption on buses above the GENERIC 50 V upset threshold".into(),
        });
    }

    let radome = r.failure(FailureDef {
        id: failure_id(Area::Environment, 53, 2),
        area: Area::Environment,
        ata: 53,
        name: "Lightning radome damage".into(),
        component: "53_env.radome_lightning".into(),
        model_field: "environment::lightning::LightningEvent.radome_damage_frac".into(),
        magnitude: "peak current / ARP5412 200 kA reference, x20 worse if the diverter strip is missed".into(),
        effect: "weather radar loss, possible radome departure".into(),
    });
    let _structure = r.failure(FailureDef {
        id: failure_id(Area::Environment, 53, 3),
        area: Area::Environment,
        ata: 53,
        name: "Lightning composite extremity burn".into(),
        component: "53_env.composite_extremity".into(),
        model_field: "environment::lightning::LightningEvent.structure_damage_frac".into(),
        magnitude: "GENERIC 0.05 x (peak current / 200 kA) at any composite entry/exit point".into(),
        effect: "mesh/paint damage, inspection item; negligible for the well-bonded metal fuselage".into(),
    });
    let compass = r.failure(FailureDef {
        id: failure_id(Area::Environment, 34, 3),
        area: Area::Environment,
        ata: 34,
        name: "Lightning standby compass deviation".into(),
        component: "34_env.standby_compass".into(),
        model_field: "environment::lightning::LightningEvent.compass_error_deg".into(),
        magnitude: "GENERIC 0..10 deg, scaling with peak current and proximity of the strike path to the nose".into(),
        effect: "standby compass reads with a fixed offset until a compass swing".into(),
    });
    let _ = (radome, compass);
}

fn register_hail(r: &mut Registry) {
    r.component(ComponentDef {
        id: "53_env.radome_hail".into(),
        area: Area::Environment,
        ata: 53,
        name: "Nose radome (hail damage state)".into(),
        params: vec![
            ParamDef { name: "damage_frac".into(), meaning: "0 undamaged .. 1 significant (TORRO H2/20mm-at-VMO reference), persistent/cumulative".into(), healthy: 0.0 },
            ParamDef { name: "wxr_attenuation_frac".into(), meaning: "GENERIC weather-radar beam attenuation/distortion, tracks damage_frac".into(), healthy: 0.0 },
        ],
        failures: vec![],
    });
    let mut hail_windshield_ids = [0u64; 6];
    const HAIL_WINDSHIELD_SEQ: [u16; 6] = [2, 9, 10, 11, 12, 13];
    for p in 1..=6u16 {
        let hail_windshield_component = format!("56_env.windshield_hail_{p}");
        r.component(ComponentDef {
            id: hail_windshield_component.clone(),
            area: Area::Environment,
            ata: 56,
            name: format!("Flight-deck windshield panel {p} (hail damage state)"),
            params: vec![
                ParamDef { name: "damage_frac".into(), meaning: "0 undamaged .. 1 significant (GENERIC 30mm-at-VMO reference), persistent/cumulative".into(), healthy: 0.0 },
                ParamDef { name: "window_heat_fault".into(), meaning: "0/1: the conductive heating film has faulted above a GENERIC 0.3 severity".into(), healthy: 0.0 },
                ParamDef { name: "leak_area_m2".into(), meaning: "GENERIC pressurisation leak area, only above 0.9 severity".into(), healthy: 0.0 },
            ],
            failures: vec![],
        });
        hail_windshield_ids[(p - 1) as usize] = r.failure(FailureDef {
            id: failure_id(Area::Environment, 56, HAIL_WINDSHIELD_SEQ[(p - 1) as usize]),
            area: Area::Environment,
            ata: 56,
            name: if p == 1 { "Hail windshield damage".into() } else { format!("Hail windshield damage (panel {p})") },
            component: hail_windshield_component,
            model_field: "environment::hail::HailOutcome.damage_frac / .window_heat_fault / .visibility_loss_frac / .leak_area_m2 (target = Windshield)".into(),
            magnitude: "cumulative impact-energy density / a GENERIC 30mm-hailstone-at-VMO reference x 15 hits".into(),
            effect: "visibility loss; window-heat film fault above 0.3; a pressurisation leak above 0.9".into(),
        });
    }

    let mut hail_wing_le_ids = [0u64; 12];
    const HAIL_WING_LE_SEQ: [u16; 12] = [2, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24];
    for s in 1..=12u16 {
        let hail_wing_le_component = format!("57_env.wing_leading_edge_hail_{s}");
        r.component(ComponentDef {
            id: hail_wing_le_component.clone(),
            area: Area::Environment,
            ata: 57,
            name: format!("Wing leading edge / slat segment {s} (6 per side, root to tip), L/R (hail damage state)"),
            params: vec![
                ParamDef { name: "damage_frac".into(), meaning: "0 undamaged .. 1 significant (GENERIC 30mm-at-VMO reference), persistent/cumulative".into(), healthy: 0.0 },
                ParamDef { name: "clmax_delta".into(), meaning: "GENERIC max-lift-coefficient penalty, 0 .. -0.05".into(), healthy: 0.0 },
                ParamDef { name: "slat_jam_risk_frac".into(), meaning: "GENERIC slat extend/retract mechanism jam risk, ramps above 0.5 damage".into(), healthy: 0.0 },
            ],
            failures: vec![],
        });
        hail_wing_le_ids[(s - 1) as usize] = r.failure(FailureDef {
            id: failure_id(Area::Environment, 57, HAIL_WING_LE_SEQ[(s - 1) as usize]),
            area: Area::Environment,
            ata: 57,
            name: if s == 1 { "Hail leading edge / slat damage".into() } else { format!("Hail leading edge / slat damage (segment {s})") },
            component: hail_wing_le_component,
            model_field: "environment::hail::HailOutcome.damage_frac / .clmax_delta / .slat_jam_risk_frac (target = WingLeadingEdge)".into(),
            magnitude: "cumulative impact-energy density / a GENERIC 30mm-hailstone-at-VMO reference x 15 hits".into(),
            effect: "drag increase, max-lift penalty, and above 0.5 a slat mechanism jam risk".into(),
        });
    }

    let mut hail_engine_ids = [0u64; 4];
    const HAIL_ENGINE_SEQ: [u16; 4] = [7, 14, 15, 16];
    for eng in 1..=4u16 {
        let hail_engine_component = format!("72_env.engine_hail_ingestion_{eng}");
        r.component(ComponentDef {
            id: hail_engine_component.clone(),
            area: Area::Environment,
            ata: 72,
            name: format!("Engine {eng} fan/compressor (hail ingestion state)"),
            params: vec![
                ParamDef { name: "fan_damage_frac".into(), meaning: "0 none .. 1 destructive, persistent/cumulative".into(), healthy: 0.0 },
                ParamDef { name: "compressor_efficiency_loss_frac".into(), meaning: "0 none .. 0.25 ceiling (GENERIC), irreversible".into(), healthy: 0.0 },
                ParamDef { name: "flameout_risk_frac".into(), meaning: "instantaneous, worse at low N1 per CS-E 790's low-power ingestion concern".into(), healthy: 0.0 },
            ],
            failures: vec![],
        });
        hail_engine_ids[(eng - 1) as usize] = r.failure(FailureDef {
            id: failure_id(Area::Environment, 72, HAIL_ENGINE_SEQ[(eng - 1) as usize]),
            area: Area::Environment,
            ata: 72,
            name: if eng == 1 { "Hail/ice engine ingestion".into() } else { format!("Engine {eng} hail/ice engine ingestion") },
            component: hail_engine_component,
            model_field: "environment::hail::HailOutcome.fan_damage_frac / .compressor_efficiency_loss_frac / .flameout_risk_frac (target = EngineInlet)".into(),
            magnitude: "fan: cumulative energy density / reference; compressor: GENERIC erosion per kg ingested, capped 0.25; flameout: ingested mass / a GENERIC tolerance that shrinks with N1 (CS-E 790 / 14 CFR 33.68)".into(),
            effect: "fan damage, permanent compressor efficiency loss, elevated flameout/roll-back risk especially at low power".into(),
        });
    }

    let mut _hail_nacelle_ids = [0u64; 4];
    const HAIL_NACELLE_SEQ: [u16; 4] = [1, 2, 3, 4];
    for n in 1..=4u16 {
        let hail_nacelle_component = format!("71_env.nacelle_hail_{n}");
        r.component(ComponentDef {
            id: hail_nacelle_component.clone(),
            area: Area::Environment,
            ata: 71,
            name: format!("Engine {n} nacelle inlet lip/cowl (hail damage state)"),
            params: vec![ParamDef { name: "damage_frac".into(), meaning: "0 undamaged .. 1 significant (GENERIC 30mm-at-VMO reference), persistent/cumulative".into(), healthy: 0.0 }],
            failures: vec![],
        });
        _hail_nacelle_ids[(n - 1) as usize] = r.failure(FailureDef {
            id: failure_id(Area::Environment, 71, HAIL_NACELLE_SEQ[(n - 1) as usize]),
            area: Area::Environment,
            ata: 71,
            name: if n == 1 { "Hail nacelle/cowl damage".into() } else { format!("Hail nacelle/cowl damage (nacelle {n})") },
            component: hail_nacelle_component,
            model_field: "environment::hail::HailOutcome.damage_frac / .drag_delta_cd (target = Nacelle)".into(),
            magnitude: "cumulative impact-energy density / a GENERIC 30mm-hailstone-at-VMO reference x 15 hits".into(),
            effect: "drag increase from cowl/inlet-lip denting".into(),
        });
    }

    let mut _hail_probe_ids = [0u64; 6];
    const HAIL_PROBE_SEQ: [u16; 6] = [4, 10, 11, 12, 13, 14];
    for p in 1..=6u16 {
        let hail_probe_component = format!("34_env.air_data_probe_hail_{p}");
        r.component(ComponentDef {
            id: hail_probe_component.clone(),
            area: Area::Environment,
            ata: 34,
            name: format!("Pitot/AoA/TAT probe or external antenna {p} (hail damage state)"),
            params: vec![ParamDef { name: "damage_frac".into(), meaning: "0 undamaged .. 1 significant (GENERIC 15mm-at-VMO reference), persistent/cumulative".into(), healthy: 0.0 }],
            failures: vec![],
        });
        _hail_probe_ids[(p - 1) as usize] = r.failure(FailureDef {
            id: failure_id(Area::Environment, 34, HAIL_PROBE_SEQ[(p - 1) as usize]),
            area: Area::Environment,
            ata: 34,
            name: if p == 1 { "Hail probe/antenna damage".into() } else { format!("Hail probe/antenna damage (probe {p})") },
            component: hail_probe_component,
            model_field: "environment::hail::HailOutcome.damage_frac (target = Probe)".into(),
            magnitude: "cumulative impact-energy density / a GENERIC 15mm-at-VMO reference x 15 hits".into(),
            effect: "unreliable airspeed/AoA or lost antenna function".into(),
        });
    }

    let radome = r.failure(FailureDef {
        id: failure_id(Area::Environment, 53, 4),
        area: Area::Environment,
        ata: 53,
        name: "Hail radome damage".into(),
        component: "53_env.radome_hail".into(),
        model_field: "environment::hail::HailOutcome.damage_frac / .wxr_attenuation_frac / .drag_delta_cd (target = Radome)".into(),
        magnitude: "cumulative impact-energy density / a 20mm-hailstone-at-VMO reference x 15 hits (C-grade radome resistance limit)".into(),
        effect: "weather-radar attenuation/beam distortion, drag increase, persists after the storm".into(),
    });

    let _ = radome;
    const HAIL_PANEL_NAMES: [&str; 6] = ["L WINDSHIELD", "R WINDSHIELD", "L SLIDING WINDOW", "R SLIDING WINDOW", "L FIXED WINDOW", "R FIXED WINDOW"];
    const HAIL_PANEL_FCOM_TITLES: [&str; 6] = [
        "A-ICE L WINDSHIELD HEATG FAULT",
        "A-ICE R WINDSHIELD HEATG FAULT",
        "A-ICE L SLIDING WINDOW HEATG FAULT",
        "A-ICE R SLIDING WINDOW HEATG FAULT",
        "A-ICE L FIXED WINDOW HEATG FAULT",
        "A-ICE R FIXED WINDOW HEATG FAULT",
    ];
    for p in 1..=6u16 {
        let panel_name = HAIL_PANEL_NAMES[(p - 1) as usize];
        r.alert(
            EcamAlert::new(
                &format!("ENV_HAIL_WINDOW_HEAT_{p}_FAULT"),
                56,
                HAIL_PANEL_FCOM_TITLES[(p - 1) as usize],
                Level::Caution,
                var(&format!("ENV_HAIL_WINDOW_HEAT_FAULT:{p}")).on(),
            )
            .confirm(1.0)
            .step(line(&format!("WINDOW HEAT {panel_name}"), "MONITOR").colour("green"))
            .status_line(&format!("{panel_name} WINDOW HEAT ......... FAULT"))
            .inop_sys(&format!("{panel_name} WINDOW HEAT"))
            .raised_by(&[hail_windshield_ids[(p - 1) as usize]]),
        );
    }
}

fn register_volcanic_ash(r: &mut Registry) {
    r.component(ComponentDef {
        id: "72_env.ngv_glassing".into(),
        area: Area::Environment,
        ata: 72,
        name: "HPT nozzle guide vanes (ash glassing state), x4 engines".into(),
        params: vec![ParamDef { name: "flow_capacity_loss_frac".into(), meaning: "0 clear .. 1 throat fully blocked by deposited molten ash".into(), healthy: 0.0 }],
        failures: vec![],
    });
    r.component(ComponentDef {
        id: "72_env.compressor_erosion".into(),
        area: Area::Environment,
        ata: 72,
        name: "Compressor blading (ash erosion state), x4 engines".into(),
        params: vec![ParamDef { name: "efficiency_loss_frac".into(), meaning: "0 none .. 0.25 ceiling (GENERIC), irreversible".into(), healthy: 0.0 }],
        failures: vec![],
    });
    r.component(ComponentDef {
        id: "56_env.windshield_ash".into(),
        area: Area::Environment,
        ata: 56,
        name: "Flight-deck windshield (ash abrasion state)".into(),
        params: vec![ParamDef { name: "visibility_loss_frac".into(), meaning: "0 clear .. 1 opaque, cumulative sandblasting".into(), healthy: 0.0 }],
        failures: vec![],
    });
    r.component(ComponentDef {
        id: "34_env.air_data_probe_ash".into(),
        area: Area::Environment,
        ata: 34,
        name: "Pitot/AoA probe (ash blockage state), x6".into(),
        params: vec![ParamDef { name: "blockage_frac".into(), meaning: "0 clear .. 1 fully blocked".into(), healthy: 0.0 }],
        failures: vec![],
    });

    let glassing = r.failure(FailureDef {
        id: failure_id(Area::Environment, 72, 5),
        area: Area::Environment,
        ata: 72,
        name: "Volcanic ash NGV glassing".into(),
        component: "72_env.ngv_glassing".into(),
        model_field: "environment::volcanic_ash::AshOutputs.flow_capacity_loss_frac".into(),
        magnitude: "deposited molten-ash mass / GENERIC 2 kg full-blockage reference".into(),
        effect: "reduced core flow capacity, raised backpressure, surge-margin loss".into(),
    });
    let erosion = r.failure(FailureDef {
        id: failure_id(Area::Environment, 72, 6),
        area: Area::Environment,
        ata: 72,
        name: "Volcanic ash compressor erosion".into(),
        component: "72_env.compressor_erosion".into(),
        model_field: "environment::volcanic_ash::AshOutputs.compressor_efficiency_loss_frac".into(),
        magnitude: "GENERIC erosion-rate integral of unmelted ash flux x velocity^2.5, capped 0.25".into(),
        effect: "permanent compressor efficiency loss".into(),
    });
    let _windshield = r.failure(FailureDef {
        id: failure_id(Area::Environment, 56, 3),
        area: Area::Environment,
        ata: 56,
        name: "Volcanic ash windshield abrasion".into(),
        component: "56_env.windshield_ash".into(),
        model_field: "environment::volcanic_ash::AshOutputs.windshield_visibility_loss_frac".into(),
        magnitude: "GENERIC cumulative sandblasting rate x ash flux x TAS".into(),
        effect: "progressive, permanent loss of forward visibility".into(),
    });
    let pitot = r.failure(FailureDef {
        id: failure_id(Area::Environment, 34, 2),
        area: Area::Environment,
        ata: 34,
        name: "Volcanic ash pitot blockage".into(),
        component: "34_env.air_data_probe_ash".into(),
        model_field: "environment::volcanic_ash::AshOutputs.pitot_blockage_frac".into(),
        magnitude: "GENERIC blockage growth rate x ash concentration".into(),
        effect: "unreliable airspeed, matching documented ash-encounter ADR anomalies".into(),
    });

    let _ = pitot;
}

fn register_ice_crystal_icing(r: &mut Registry) {
    r.component(ComponentDef {
        id: "72_env.ice_crystal_accretion".into(),
        area: Area::Environment,
        ata: 72,
        name: "IP compressor front stage / splitter (ice crystal accretion state), x4 engines".into(),
        params: vec![ParamDef { name: "flow_capacity_loss_frac".into(), meaning: "0 clear .. 1 fully blocked; sheds and resets near 0.6 (GENERIC threshold)".into(), healthy: 0.0 }],
        failures: vec![],
    });

    let accretion = r.failure(FailureDef {
        id: failure_id(Area::Environment, 72, 3),
        area: Area::Environment,
        ata: 72,
        name: "Ice crystal core accretion".into(),
        component: "72_env.ice_crystal_accretion".into(),
        model_field: "environment::ice_crystal_icing::IceCrystalOutputs.flow_capacity_loss_frac".into(),
        magnitude: "accreted ice mass / GENERIC 0.5 kg full-blockage reference, only near a 0 C surface (Mason et al. adherence window)".into(),
        effect: "reduced core flow capacity, periodic shedding events".into(),
    });
    let rollback = r.failure(FailureDef {
        id: failure_id(Area::Environment, 72, 4),
        area: Area::Environment,
        ata: 72,
        name: "Ice crystal engine roll-back/flameout risk".into(),
        component: "72_env.ice_crystal_accretion".into(),
        model_field: "environment::ice_crystal_icing::IceCrystalOutputs.rollback_risk_frac / .flameout_risk_frac".into(),
        magnitude: "flow_capacity_loss_frac^2 (GENERIC), +0.4 transient spike during a shedding event".into(),
        effect: "core speed roll-back, possible flameout during/after a shedding event".into(),
    });

    let _ = (accretion, rollback);
}

fn register_runway_contamination(r: &mut Registry) {
    r.component(ComponentDef {
        id: "32_env.runway_friction".into(),
        area: Area::Environment,
        ata: 32,
        name: "Runway surface friction state (not an aircraft part, but feeds every gear's braking model)".into(),
        params: vec![
            ParamDef { name: "mu_effective".into(), meaning: "GENERIC effective braking friction coefficient, 0.05 (nil/hydroplaning) .. 0.40 (dry)".into(), healthy: 0.40 },
            ParamDef { name: "hydroplaning".into(), meaning: "0 tyre in contact, 1 riding on a fluid film".into(), healthy: 0.0 },
        ],
        failures: vec![],
    });

    r.failure(FailureDef {
        id: failure_id(Area::Environment, 32, 2),
        area: Area::Environment,
        ata: 32,
        name: "Runway contamination friction/hydroplaning loss".into(),
        component: "32_env.runway_friction".into(),
        model_field: "environment::runway_contamination::RunwayFrictionOutput.mu_effective".into(),
        magnitude: "1 - mu_effective/0.40 (GENERIC dry reference), reaching ~0.9 in full dynamic hydroplaning".into(),
        effect: "longer stopping distance, reduced directional control on the affected gear -- felt through the aircraft's actual braking/deceleration performance; the real aircraft has no automatic ECAM hydroplaning advisory".into(),
    });
}
