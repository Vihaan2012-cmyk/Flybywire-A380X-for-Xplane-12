use crate::deep::api::*;
use crate::deep::thermal_zones::topology_a380;

fn frac(name: &str, meaning: &str, healthy: f64) -> ParamDef {
    ParamDef { name: name.to_string(), meaning: meaning.to_string(), healthy }
}

pub fn register(r: &mut Registry) {
    register_ventilation_failures(r);
    register_fire_failures(r);
    register_ice_and_duct_failures(r);
    register_gear_door_failures(r);
    register_pneumatic_and_apu_failures(r);
    register_insulation_failures(r);
    register_ecam_alerts(r);
    register_ecam_completeness_additions(r);
}

fn register_ecam_completeness_additions(r: &mut Registry) {
    const ATA: u16 = 21;
    {
        let comp = "21_therm.fwd_cargo_trim_air_valve".to_string();
        let fid = failure_id(Area::ThermalZones, ATA, 8);
        r.component(ComponentDef {
            id: comp.clone(),
            area: Area::ThermalZones,
            ata: ATA,
            name: "FWD cargo zone trim-air valve".into(),
            params: vec![frac("fault", "the FWD cargo zone's own trim-air valve broken", 0.0)],
            failures: vec![fid],
        });
        r.failure(FailureDef {
            id: fid,
            area: Area::ThermalZones,
            ata: ATA,
            name: "FWD cargo zone trim-air valve fault".into(),
            component: comp,
            model_field: "thermal_zones::live::ThermalZonesLive.fwd_cargo_trv_fault".into(),
            magnitude: "0 healthy .. 1 fully faulted".into(),
            effect: "FlyByWire's own 211800030 COND FWD CARGO TEMP REGUL FAULT; distinct from the aircraft-wide HOT AIR valves 1/2 (already-wired 211800032/033) and from the automatic zone controller".into(),
        });
    }
    {
        let comp = "21_therm.ths_bay_ventilation_fan".to_string();
        let fid = failure_id(Area::ThermalZones, ATA, 9);
        r.component(ComponentDef {
            id: comp.clone(),
            area: Area::ThermalZones,
            ata: ATA,
            name: "THS (trimmable horizontal stabiliser) bay ventilation fan".into(),
            params: vec![frac("fault", "the bay's own ventilation fan broken", 0.0)],
            failures: vec![fid],
        });
        r.failure(FailureDef {
            id: fid,
            area: Area::ThermalZones,
            ata: ATA,
            name: "THS bay ventilation fan fault".into(),
            component: comp,
            model_field: "thermal_zones::live::ThermalZonesLive.ths_bay_vent_fault".into(),
            magnitude: "0 healthy .. 1 fully faulted -- a component-broken flag; the bay has no thermal-network zone of its own in this port or in a380_systems, so the trigger is this flag, not a temperature this pass would otherwise have to invent (E-AIR-DESIGN.md 212800028)".into(),
            effect: "FlyByWire's own 212800028 VENT THS BAY VENT FAULT".into(),
        });
    }
    {
        let comp = "21_therm.bulk_cargo_duct_heater".to_string();
        let fid = failure_id(Area::ThermalZones, ATA, 10);
        r.component(ComponentDef {
            id: comp.clone(),
            area: Area::ThermalZones,
            ata: ATA,
            name: "Bulk cargo compartment heater duct".into(),
            params: vec![frac("overheat", "the duct's own outlet overheats, 0 healthy .. 1 clearly over the FCOM's 70 C trip", 0.0)],
            failures: vec![fid],
        });
        r.failure(FailureDef {
            id: fid,
            area: Area::ThermalZones,
            ata: ATA,
            name: "Bulk cargo duct heater overtemperature".into(),
            component: comp,
            model_field: "thermal_zones::live::ThermalZonesLive.bulk_cargo_duct_temp_c".into(),
            magnitude: "0 healthy (duct tracks the real, already-modelled CargoBulk zone air temperature) .. 1 fully faulted (+120 C GENERIC excess over it)".into(),
            effect: "FlyByWire's own 211800024 COND BULK CARGO DUCT OVHT; the real 70 C trip (FCOM PRO-ABN-ECAM p.4669, E-AIR-FCOM.json) is applied in fbw/ata21_22_23.rs, not baked in here".into(),
        });
    }
    {
        let comp = "21_therm.trim_air_duct".to_string();
        let fid = failure_id(Area::ThermalZones, ATA, 11);
        r.component(ComponentDef {
            id: comp.clone(),
            area: Area::ThermalZones,
            ata: ATA,
            name: "Cockpit/cabin trim-air duct".into(),
            params: vec![frac("overheat", "the duct's own outlet overheats, 0 healthy .. 1 clearly over the FCOM's 70 C trip", 0.0)],
            failures: vec![fid],
        });
        r.failure(FailureDef {
            id: fid,
            area: Area::ThermalZones,
            ata: ATA,
            name: "Trim-air duct overtemperature".into(),
            component: comp,
            model_field: "thermal_zones::live::ThermalZonesLive.trim_air_duct_temp_c".into(),
            magnitude: "0 healthy (duct tracks the real, already-modelled CabinMainDeck zone air temperature) .. 1 fully faulted (+120 C GENERIC excess over it)".into(),
            effect: "FlyByWire's own 211800028 COND DUCT OVHT; the real 70 C trip (FCOM PRO-ABN-ECAM p.4675, E-AIR-FCOM.json) is applied in fbw/ata21_22_23.rs, not baked in here".into(),
        });
    }
}

struct VentZone {
    n: u16,
    component_id: &'static str,
    name: &'static str,
    field: &'static str,
    zone: &'static str,
}

fn ventilation_zones() -> [VentZone; 7] {
    [
        VentZone { n: 1, component_id: "21_thermal.main_avionics_fan", name: "Main avionics bay ventilation fan failure", field: "vents.main_avionics_fan", zone: "MainAvionics" },
        VentZone { n: 2, component_id: "21_thermal.upper_avionics_fan", name: "Upper avionics bay ventilation fan failure", field: "vents.upper_avionics_fan", zone: "UpperAvionics" },
        VentZone { n: 3, component_id: "21_thermal.cargo_fwd_fan", name: "Forward cargo extract fan failure", field: "vents.cargo_fwd_fan", zone: "CargoFwd" },
        VentZone { n: 4, component_id: "21_thermal.cargo_aft_fan", name: "Aft cargo extract fan failure", field: "vents.cargo_aft_fan", zone: "CargoAft" },
        VentZone { n: 5, component_id: "21_thermal.cargo_bulk_fan", name: "Bulk cargo extract fan failure", field: "vents.cargo_bulk_fan", zone: "CargoBulk" },
        VentZone { n: 6, component_id: "21_thermal.belly_pack_bay_vent", name: "Belly fairing pack bay ram-air scoop/drain blockage", field: "vents.belly_pack_bay_vent", zone: "BellyFairingPacks" },
        VentZone { n: 7, component_id: "21_thermal.apu_compartment_vent", name: "APU compartment ventilation blockage", field: "vents.apu_compartment_vent", zone: "ApuCompartment" },
    ]
}

fn register_ventilation_failures(r: &mut Registry) {
    for v in ventilation_zones() {
        r.component(ComponentDef {
            id: v.component_id.to_string(),
            area: Area::ThermalZones,
            ata: 21,
            name: v.name.to_string(),
            params: vec![frac("flow_loss", "fraction of nameplate ventilation flow lost (0 = full flow, 1 = fully blocked/failed)", 0.0)],
            failures: vec![failure_id(Area::ThermalZones, 21, v.n)],
        });
        r.failure(FailureDef {
            id: failure_id(Area::ThermalZones, 21, v.n),
            area: Area::ThermalZones,
            ata: 21,
            name: v.name.to_string(),
            component: v.component_id.to_string(),
            model_field: format!("thermal_zones::network::VentilationLink.health (via ThermalNetwork::set_ventilation_health(topology_a380::A380VentLinks.{}, 1.0 - magnitude))", v.field),
            magnitude: "0..1, fraction of nameplate ventilation flow lost".to_string(),
            effect: format!("{} loses cooling/purge airflow; its steady-state air temperature rises toward its baseline-heat/skin-conduction balance, and any registered thermal component there accrues damage faster.", v.zone),
        });
    }
}

fn register_fire_failures(r: &mut Registry) {
    const CARGO_FIRE_MAX_HEAT_W: f64 = 200_000.0;
    const CARGO_FIRE_MAX_SMOKE_KG_S: f64 = 0.01;
    const NACELLE_FIRE_MAX_HEAT_W: f64 = 500_000.0;
    const NACELLE_FIRE_MAX_SMOKE_KG_S: f64 = 0.005;
    const APU_FIRE_MAX_HEAT_W: f64 = 300_000.0;
    const APU_FIRE_MAX_SMOKE_KG_S: f64 = 0.008;
    const LAVATORY_FIRE_MAX_HEAT_W: f64 = 20_000.0;
    const LAVATORY_FIRE_MAX_SMOKE_KG_S: f64 = 0.002;

    let cargo = [("CargoFwd", "cargo_fwd", 1u16), ("CargoAft", "cargo_aft", 2u16), ("CargoBulk", "cargo_bulk", 3u16)];
    for (zone_name, field, n) in cargo {
        let component_id = format!("26_thermal.{field}_fire_load");
        r.component(ComponentDef {
            id: component_id.clone(),
            area: Area::ThermalZones,
            ata: 26,
            name: format!("{zone_name} cargo fire load"),
            params: vec![frac("severity", "fire severity, 0 = no fire, 1 = full-severity fire (reference heat/smoke release rate)", 0.0)],
            failures: vec![failure_id(Area::ThermalZones, 26, n)],
        });
        r.failure(FailureDef {
            id: failure_id(Area::ThermalZones, 26, n),
            area: Area::ThermalZones,
            ata: 26,
            name: format!("{zone_name} cargo compartment fire"),
            component: component_id,
            model_field: format!("thermal_zones::network::Zone.injected_heat_w / .injected_smoke_kg_s (via ThermalNetwork::inject_heat_w(zones.{field}, magnitude*{CARGO_FIRE_MAX_HEAT_W}) and inject_smoke_kg_s(zones.{field}, magnitude*{CARGO_FIRE_MAX_SMOKE_KG_S}))"),
            magnitude: format!("0..1, fraction of the reference {CARGO_FIRE_MAX_HEAT_W:.0} W / {CARGO_FIRE_MAX_SMOKE_KG_S} kg/s full-severity fire"),
            effect: format!("{zone_name} air temperature and smoke concentration rise; heat conducts to neighbouring zones through the real structure links (e.g. CargoFwd<->MainAvionics), and smoke is readable by a detector via ThermalNetwork::smoke_concentration."),
        });
    }

    for engine in 1..=4u16 {
        let field = format!("nacelle_cowl[{}]", engine - 1);
        let component_id = format!("26_thermal.nacelle_{engine}_fire_load");
        r.component(ComponentDef {
            id: component_id.clone(),
            area: Area::ThermalZones,
            ata: 26,
            name: format!("Engine {engine} nacelle fire load"),
            params: vec![frac("severity", "fire severity, 0 = no fire, 1 = full-severity fire", 0.0)],
            failures: vec![failure_id(Area::ThermalZones, 26, 3 + engine)],
        });
        r.failure(FailureDef {
            id: failure_id(Area::ThermalZones, 26, 3 + engine),
            area: Area::ThermalZones,
            ata: 26,
            name: format!("Engine {engine} nacelle fire"),
            component: component_id,
            model_field: format!("thermal_zones::network::Zone.injected_heat_w / .injected_smoke_kg_s (via ThermalNetwork::inject_heat_w(zones.{field}, magnitude*{NACELLE_FIRE_MAX_HEAT_W}) and inject_smoke_kg_s(zones.{field}, magnitude*{NACELLE_FIRE_MAX_SMOKE_KG_S}))"),
            magnitude: format!("0..1, fraction of the reference {NACELLE_FIRE_MAX_HEAT_W:.0} W / {NACELLE_FIRE_MAX_SMOKE_KG_S} kg/s full-severity fire"),
            effect: format!("NacelleCowl{engine} air temperature and smoke concentration rise rapidly (small zone, large ram-air ventilation already present); heat conducts into PylonEngine{engine} through the real structure link, threatening its wiring."),
        });
    }

    r.component(ComponentDef {
        id: "26_thermal.apu_compartment_fire_load".to_string(),
        area: Area::ThermalZones,
        ata: 26,
        name: "APU compartment fire load".to_string(),
        params: vec![frac("severity", "fire severity, 0 = no fire, 1 = full-severity fire", 0.0)],
        failures: vec![failure_id(Area::ThermalZones, 26, 8)],
    });
    r.failure(FailureDef {
        id: failure_id(Area::ThermalZones, 26, 8),
        area: Area::ThermalZones,
        ata: 26,
        name: "APU compartment fire".to_string(),
        component: "26_thermal.apu_compartment_fire_load".to_string(),
        model_field: format!("thermal_zones::network::Zone.injected_heat_w / .injected_smoke_kg_s (via ThermalNetwork::inject_heat_w(zones.apu_compartment, magnitude*{APU_FIRE_MAX_HEAT_W}) and inject_smoke_kg_s(zones.apu_compartment, magnitude*{APU_FIRE_MAX_SMOKE_KG_S}))"),
        magnitude: format!("0..1, fraction of the reference {APU_FIRE_MAX_HEAT_W:.0} W / {APU_FIRE_MAX_SMOKE_KG_S} kg/s full-severity fire"),
        effect: "ApuCompartment air temperature and smoke concentration rise; heat conducts into TailCone through the real structure link.".to_string(),
    });

    let decks = [("CabinMainDeck", "cabin_main_deck", "main-deck", 9u16), ("CabinUpperDeck", "cabin_upper_deck", "upper-deck", 10u16)];
    for (zone_name, field, deck, n) in decks {
        let component_id = format!("26_thermal.{field}_lavatory_fire_load");
        r.component(ComponentDef {
            id: component_id.clone(),
            area: Area::ThermalZones,
            ata: 26,
            name: format!("{zone_name} lavatory waste-bin fire load"),
            params: vec![frac("severity", "fire severity, 0 = no fire, 1 = full-severity waste-bin fire (reference heat/smoke release rate)", 0.0)],
            failures: vec![failure_id(Area::ThermalZones, 26, n)],
        });
        r.failure(FailureDef {
            id: failure_id(Area::ThermalZones, 26, n),
            area: Area::ThermalZones,
            ata: 26,
            name: format!("{deck} lavatory waste-bin fire"),
            component: component_id,
            model_field: format!("thermal_zones::network::Zone.injected_heat_w / .injected_smoke_kg_s (via ThermalNetwork::inject_heat_w(zones.{field}, magnitude*{LAVATORY_FIRE_MAX_HEAT_W}) and inject_smoke_kg_s(zones.{field}, magnitude*{LAVATORY_FIRE_MAX_SMOKE_KG_S}))"),
            magnitude: format!("0..1, fraction of the reference {LAVATORY_FIRE_MAX_HEAT_W:.0} W / {LAVATORY_FIRE_MAX_SMOKE_KG_S} kg/s full-severity waste-bin fire"),
            effect: format!(
                "{zone_name} smoke concentration rises, which is what the four {deck} lavatory smoke detectors in deep::sensors sample (DEEP_SMOKE_LAV_n_ALARM); the deck's air also warms slowly and the heat conducts into the other deck and the cargo bays through the real structure links."
            ),
        });
    }

    const AVNCS_FIRE_MAX_HEAT_W: f64 = 20_000.0;
    const AVNCS_FIRE_MAX_SMOKE_KG_S: f64 = 0.002;

    let avionics_equipment = [("AftAvionics", "aft_avionics", "Aft avionics equipment fire", 11u16), ("MainAvionics", "main_avionics", "Main avionics equipment fire", 12u16), ("UpperAvionics", "upper_avionics", "Upper avionics equipment fire", 13u16)];
    for (zone_name, field, title, n) in avionics_equipment {
        let component_id = format!("26_thermal.{field}_equipment_fire_load");
        r.component(ComponentDef {
            id: component_id.clone(),
            area: Area::ThermalZones,
            ata: 26,
            name: format!("{zone_name} equipment fire load"),
            params: vec![frac("severity", "fire severity, 0 = no fire, 1 = full-severity equipment fire (reference heat/smoke release rate)", 0.0)],
            failures: vec![failure_id(Area::ThermalZones, 26, n)],
        });
        r.failure(FailureDef {
            id: failure_id(Area::ThermalZones, 26, n),
            area: Area::ThermalZones,
            ata: 26,
            name: title.to_string(),
            component: component_id,
            model_field: format!("thermal_zones::network::Zone.injected_heat_w / .injected_smoke_kg_s (via ThermalNetwork::inject_heat_w(zones.{field}, magnitude*{AVNCS_FIRE_MAX_HEAT_W}) and inject_smoke_kg_s(zones.{field}, magnitude*{AVNCS_FIRE_MAX_SMOKE_KG_S}))"),
            magnitude: format!("0..1, fraction of the reference {AVNCS_FIRE_MAX_HEAT_W:.0} W / {AVNCS_FIRE_MAX_SMOKE_KG_S} kg/s full-severity equipment fire"),
            effect: format!("{zone_name} smoke concentration rises, which is what deep::sensors' new avionics-bay smoke detectors (E-FIRE §B) sample; heat conducts to neighbouring zones through the real structure links."),
        });
    }

    const LDCR_FIRE_MAX_HEAT_W: f64 = 20_000.0;
    const LDCR_FIRE_MAX_SMOKE_KG_S: f64 = 0.002;
    r.component(ComponentDef {
        id: "26_thermal.fwd_lower_crew_rest_fire_load".to_string(),
        area: Area::ThermalZones,
        ata: 26,
        name: "FWD Lower Crew Rest (LDCR) module fire load".to_string(),
        params: vec![frac("severity", "fire severity, 0 = no fire, 1 = full-severity furnishings/waste-bin fire (reference heat/smoke release rate)", 0.0)],
        failures: vec![failure_id(Area::ThermalZones, 26, 14)],
    });
    r.failure(FailureDef {
        id: failure_id(Area::ThermalZones, 26, 14),
        area: Area::ThermalZones,
        ata: 26,
        name: "FWD Lower Crew Rest module furnishings/waste-bin fire".to_string(),
        component: "26_thermal.fwd_lower_crew_rest_fire_load".to_string(),
        model_field: format!("thermal_zones::network::Zone.injected_heat_w / .injected_smoke_kg_s (via ThermalNetwork::inject_heat_w(zones.fwd_lower_crew_rest, magnitude*{LDCR_FIRE_MAX_HEAT_W}) and inject_smoke_kg_s(zones.fwd_lower_crew_rest, magnitude*{LDCR_FIRE_MAX_SMOKE_KG_S}))"),
        magnitude: format!("0..1, fraction of the reference {LDCR_FIRE_MAX_HEAT_W:.0} W / {LDCR_FIRE_MAX_SMOKE_KG_S} kg/s full-severity fire"),
        effect: "FwdLowerCrewRest smoke concentration rises, sampled by deep::sensors' new FWD Lower Crew Rest detector (E-FIRE §B), which serves both the module's own smoke alert and the LOWER DECK LAVATORY alert (same module, same real lavatory); heat conducts into CargoFwd through the real structure link.".to_string(),
    });
}

fn register_ice_and_duct_failures(r: &mut Registry) {
    let wings = [("WingLeLeft", "wing_le_left", 1u16), ("WingLeRight", "wing_le_right", 2u16)];
    for (zone_name, field, n) in wings {
        let component_id = format!("30_thermal.{field}_antiice_duct");
        r.component(ComponentDef {
            id: component_id.clone(),
            area: Area::ThermalZones,
            ata: 30,
            name: format!("{zone_name} anti-ice duct"),
            params: vec![frac("leak_fraction", "duct wall/joint leak, fraction of full-severity leak orifice", 0.0)],
            failures: vec![failure_id(Area::ThermalZones, 30, n)],
        });
        r.failure(FailureDef {
            id: failure_id(Area::ThermalZones, 30, n),
            area: Area::ThermalZones,
            ata: 30,
            name: format!("{zone_name} anti-ice duct leak"),
            component: component_id,
            model_field: format!("thermal_zones::network::Zone.injected_heat_w (via ThermalNetwork::inject_heat_w(zones.{field}, live::anti_ice_duct_leak_heat_w(magnitude, ..)))"),
            magnitude: "0..1, fraction of a full-severity crack (2% of the 50 mm WAI duct bore) at the wing's own side engine bleed port condition (Truth::engine_bleed_pressure_pa/_temp_k, higher-pressure of the pair) -- no fixed reference wattage: the leak's heat is the crack's own choked-orifice enthalpy flow above the bay, so it is zero with the source engines shut down and bounded by the duct's own temperature".to_string(),
            effect: format!("{zone_name}'s structure/air temperature rises well past its normal transient anti-ice cycle, threatening its registered insulation/wiring components (e.g. WingLeLeftAntiIceDuctInsulation)."),
        });
    }

    for engine in 1..=4u16 {
        let field = format!("nacelle_cowl[{}]", engine - 1);
        let component_id = format!("30_thermal.nacelle_{engine}_vent_duct");
        r.component(ComponentDef {
            id: component_id.clone(),
            area: Area::ThermalZones,
            ata: 30,
            name: format!("Engine {engine} nacelle anti-ice/ventilation duct assembly"),
            params: vec![
                frac("leak_fraction", "anti-ice/bleed duct leak, fraction of full-severity leak orifice", 0.0),
                frac("scoop_blockage_fraction", "ram-air vent scoop ice blockage, fraction of nameplate flow lost", 0.0),
            ],
            failures: vec![failure_id(Area::ThermalZones, 30, 2 + engine), failure_id(Area::ThermalZones, 30, 6 + engine)],
        });
        r.failure(FailureDef {
            id: failure_id(Area::ThermalZones, 30, 2 + engine),
            area: Area::ThermalZones,
            ata: 30,
            name: format!("Engine {engine} nacelle anti-ice duct leak"),
            component: component_id.clone(),
            model_field: format!("thermal_zones::network::Zone.injected_heat_w (via ThermalNetwork::inject_heat_w(zones.{field}, live::anti_ice_duct_leak_heat_w(magnitude, ..)))"),
            magnitude: "0..1, fraction of a full-severity crack (2% of the 50 mm WAI-class duct bore) at this engine's own bleed port condition (Truth::engine_bleed_pressure_pa/_temp_k) -- no fixed reference wattage: the leak's heat is the crack's own choked-orifice enthalpy flow above the bay, zero with the engine shut down".to_string(),
            effect: format!("NacelleCowl{engine} runs hot, threatening NacelleCowl{engine}Wiring."),
        });
        r.failure(FailureDef {
            id: failure_id(Area::ThermalZones, 30, 6 + engine),
            area: Area::ThermalZones,
            ata: 30,
            name: format!("Engine {engine} nacelle vent scoop ice blockage"),
            component: component_id,
            model_field: format!("thermal_zones::network::VentilationLink.health (via ThermalNetwork::set_ventilation_health(vents.nacelle_vent[{}], 1.0 - magnitude))", engine - 1),
            magnitude: "0..1, fraction of the nacelle's ram-air ventilation flow blocked by ice".to_string(),
            effect: format!("NacelleCowl{engine} loses its large ram-air ventilation term, so any heat present (engine proximity, a duct leak) accumulates faster than normal."),
        });
    }
}

fn register_gear_door_failures(r: &mut Registry) {
    let doors = [
        ("NoseGearWell", "nose_gear_door", "nose_gear_well", 1u16),
        ("WingGearWell", "wing_gear_door", "wing_gear_well", 2u16),
        ("BodyGearWell", "body_gear_door", "body_gear_well", 3u16),
    ];
    for (zone_name, field, _zone_field, n) in doors {
        let component_id = format!("32_thermal.{field}");
        r.component(ComponentDef {
            id: component_id.clone(),
            area: Area::ThermalZones,
            ata: 32,
            name: format!("{zone_name} bay door"),
            params: vec![frac("jam_fraction", "door mechanism jam: 0 = follows commanded position freely, 1 = fully stuck at whatever position it jammed in", 0.0)],
            failures: vec![failure_id(Area::ThermalZones, 32, n)],
        });
        r.failure(FailureDef {
            id: failure_id(Area::ThermalZones, 32, n),
            area: Area::ThermalZones,
            ata: 32,
            name: format!("{zone_name} bay door jam"),
            component: component_id,
            model_field: format!("thermal_zones::network::VentilationLink.health (via ThermalNetwork::set_ventilation_health(vents.{field}, stuck_value) instead of tracking the landing-gear system's commanded open fraction)"),
            magnitude: "0..1, probability/degree the door is stuck away from its commanded position at the moment of the fault".to_string(),
            effect: format!("{zone_name} either loses its normal closed-door thermal insulation (stuck open: bay runs cold at altitude, warm on a hot ramp) or loses its normal in-flight purge ventilation (stuck closed: bay retains brake/hydraulic heat)."),
        });
    }
}

fn register_pneumatic_and_apu_failures(r: &mut Registry) {
    const PYLON_BLEED_LEAK_AREA_FRACTION_OF_BORE: f64 = 0.02;
    const PYLON_BLEED_DUCT_BORE_M: f64 = 0.1016;
    for engine in 1..=4u16 {
        let field = format!("pylon[{}]", engine - 1);
        let component_id = format!("36_thermal.pylon_{engine}_bleed_duct");
        r.component(ComponentDef {
            id: component_id.clone(),
            area: Area::ThermalZones,
            ata: 36,
            name: format!("Pylon {engine} bleed duct run"),
            params: vec![frac("leak_fraction", "duct wall/joint leak, fraction of full-severity leak orifice", 0.0)],
            failures: vec![failure_id(Area::ThermalZones, 36, engine)],
        });
        r.failure(FailureDef {
            id: failure_id(Area::ThermalZones, 36, engine),
            area: Area::ThermalZones,
            ata: 36,
            name: format!("Pylon {engine} bleed duct leak"),
            component: component_id,
            model_field: format!(
                "thermal_zones::network::Zone.injected_heat_w (via ThermalNetwork::inject_heat_w(zones.{field}, live::pylon_bleed_leak_heat_w(magnitude, truth.engine_bleed_pressure_pa[{}], truth.engine_bleed_temp_k[{}], bay air, ambient)))",
                engine - 1,
                engine - 1
            ),
            magnitude: format!(
                "0..1, fraction of the full-severity crack area {:.0}% of the duct's {:.4} m bore; the heat delivered is that crack's own choked mass flow at the engine's real bleed port condition times its enthalpy above the bay, so it is zero with the engine shut down and ~52 kW at take-off port conditions",
                PYLON_BLEED_LEAK_AREA_FRACTION_OF_BORE * 100.0,
                PYLON_BLEED_DUCT_BORE_M
            ),
            effect: format!("PylonEngine{engine} runs hot, conducting into NacelleCowl{engine} and WingTe{} through the real structure links.", if engine <= 2 { "Left" } else { "Right" }),
        });
    }

    r.component(ComponentDef {
        id: "49_thermal.apu_bleed_duct_tailcone".to_string(),
        area: Area::ThermalZones,
        ata: 49,
        name: "APU bleed duct run (tail cone)".to_string(),
        params: vec![frac("leak_fraction", "duct wall/joint leak, fraction of full-severity leak orifice", 0.0)],
        failures: vec![failure_id(Area::ThermalZones, 49, 1)],
    });
    r.failure(FailureDef {
        id: failure_id(Area::ThermalZones, 49, 1),
        area: Area::ThermalZones,
        ata: 49,
        name: "APU bleed duct leak (tail cone run)".to_string(),
        component: "49_thermal.apu_bleed_duct_tailcone".to_string(),
        model_field: "thermal_zones::network::Zone.injected_heat_w (via ThermalNetwork::inject_heat_w(zones.tail_cone, live::apu_bleed_leak_heat_w(magnitude, truth.apu_bleed_pressure_pa, published DEEP_PNEU_APU_DUCT_TEMPERATURE_C, bay air, ambient)))".to_string(),
        magnitude: "0..1, fraction of the full-severity crack area; the heat delivered is that crack's own choked mass flow at the APU's real bleed port condition times its enthalpy above the bay, so it is zero with the APU shut down".to_string(),
        effect: "TailCone runs hot, conducting into ApuCompartment and BodyGearWell through the real structure links.".to_string(),
    });
}

fn register_insulation_failures(r: &mut Registry) {
    r.component(ComponentDef {
        id: "53_thermal.crown_insulation_blanket".to_string(),
        area: Area::ThermalZones,
        ata: 53,
        name: "Crown area thermal/acoustic insulation blanket".to_string(),
        params: vec![frac("condition", "blanket condition: 1 = intact (attenuates exterior heat transfer), 0 = damaged/missing/soaked (bare-metal exterior heat transfer)", 1.0)],
        failures: vec![failure_id(Area::ThermalZones, 53, 1)],
    });
    r.failure(FailureDef {
        id: failure_id(Area::ThermalZones, 53, 1),
        area: Area::ThermalZones,
        ata: 53,
        name: "Crown area insulation blanket damage".to_string(),
        component: "53_thermal.crown_insulation_blanket".to_string(),
        model_field: "thermal_zones::network::Zone.insulation_effectiveness (set directly: network.zones[zones.crown_area].insulation_effectiveness = 1.0 - magnitude)".to_string(),
        magnitude: "0..1, fraction of the blanket's insulating effectiveness lost (0 = intact, 1 = fully damaged/missing)".to_string(),
        effect: "CrownArea's structure tracks the outside recovery temperature much more closely (colder at altitude, hotter on a sunny ramp), and via its CabinUpperDeck conduction link, chills or heats the cabin ceiling above normal.".to_string(),
    });
}

fn register_ecam_alerts(r: &mut Registry) {
    for engine in 1..=4u64 {
        let _ = (
            failure_id(Area::ThermalZones, 30, 2 + engine as u16),
            failure_id(Area::ThermalZones, 30, 6 + engine as u16),
            failure_id(Area::ThermalZones, 26, 3 + engine as u16),
        );
    }



}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn registers_without_validation_errors() {
        let mut r = Registry::default();
        register(&mut r);
        let errors = r.validate_area();
        assert!(errors.is_empty(), "registry validation errors: {errors:?}");
    }

    #[test]
    fn every_component_and_failure_carries_the_thermal_zones_area() {
        let mut r = Registry::default();
        register(&mut r);
        assert!(!r.failures.is_empty());
        assert!(!r.components.is_empty());
        for f in &r.failures {
            assert_eq!(f.area, Area::ThermalZones);
            assert!(f.id / 1_000_000 == Area::ThermalZones as u64);
        }
        for c in &r.components {
            assert_eq!(c.area, Area::ThermalZones);
        }
    }

    #[test]
    fn failure_ids_are_unique() {
        let mut r = Registry::default();
        register(&mut r);
        let mut ids: Vec<u64> = r.failures.iter().map(|f| f.id).collect();
        let before = ids.len();
        ids.sort_unstable();
        ids.dedup();
        assert_eq!(ids.len(), before, "duplicate failure ids registered");
    }

    #[test]
    fn topology_zone_names_used_in_effect_text_exist_in_the_built_network() {
        let a380 = topology_a380::build();
        let names: Vec<&str> = a380.network.zones.iter().map(|z| z.name).collect();
        for expected in ["MainAvionics", "UpperAvionics", "CargoFwd", "CargoAft", "CargoBulk", "WingLeLeft", "WingLeRight", "TailCone", "ApuCompartment", "BellyFairingPacks", "CrownArea"] {
            assert!(names.contains(&expected), "expected zone {expected} to exist in topology_a380::build()");
        }
    }
}
