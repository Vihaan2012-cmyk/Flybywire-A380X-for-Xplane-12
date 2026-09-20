//! Registers `thermal_zones`'s failures, components and ECAM alerts into
//! `crate::deep::api::Registry` (`Area::ThermalZones`, area code 11).
//!
//! **Model-field/Var note**: this module's `network`/`damage` code is
//! deliberately self-contained (no `crate::Vars`/X-Plane dependency, this
//! push's hard rule 2), so none of the `THERMAL_ZONE_<NAME>_TEMPERATURE_C`
//! / `THERMAL_ZONE_<NAME>_SMOKE_CONCENTRATION` / `THERMAL_COMPONENT_
//! <NAME>_DAMAGE` variable names an `EcamAlert::trigger` below reads are
//! published yet -- whoever wires `topology_a380::build()`'s
//! `ThermalNetwork`/`ThermalDamageRegistry` into the running simulation
//! needs to publish them each tick (the same convention
//! `physics::bays.rs` already uses for `BAY_<NAME>_TEMPERATURE_C`, listed
//! here so the naming lines up with that precedent). Recorded in
//! `PROGRESS.md`.

use crate::deep::api::*;
use crate::deep::thermal_zones::topology_a380;

/// One 0..1 health parameter, matching every failure/component's fraction
/// convention (`0.0 = healthy .. 1.0 = fully failed`), except where noted.
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
}

// ---------------------------------------------------------------------------
// ATA 21: zone ventilation (fans, ram-air scoops).
// ---------------------------------------------------------------------------

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

// ---------------------------------------------------------------------------
// ATA 26: fire/smoke sources (cargo, nacelle, APU compartment).
// ---------------------------------------------------------------------------

fn register_fire_failures(r: &mut Registry) {
    // Reference fire heat/smoke release rates at magnitude 1.0. GENERIC:
    // no public A380 fire-test heat-release figure exists for any of
    // these compartments; sized so a full-severity fire dominates the
    // zone's own baseline heat balance within tens of seconds to a few
    // minutes (the physically meaningful order of magnitude for a
    // cargo/engine/APU fire scenario), not a specific certification test
    // value.
    const CARGO_FIRE_MAX_HEAT_W: f64 = 200_000.0;
    const CARGO_FIRE_MAX_SMOKE_KG_S: f64 = 0.01;
    const NACELLE_FIRE_MAX_HEAT_W: f64 = 500_000.0; // fuel/oil fire in a ventilated nacelle: larger and faster
    const NACELLE_FIRE_MAX_SMOKE_KG_S: f64 = 0.005;
    const APU_FIRE_MAX_HEAT_W: f64 = 300_000.0;
    const APU_FIRE_MAX_SMOKE_KG_S: f64 = 0.008;
    // A lavatory waste-bin fire: the one cabin fire the aircraft is
    // certified to detect by itself. CS/FAR 25.854 requires a smoke
    // detector in every lavatory and a built-in extinguisher in every
    // waste receptacle, which is why FlyByWire's catalogue carries a MAIN
    // DECK and an UPPER DECK LAVATORY SMOKE procedure at all. Before this
    // there was no cabin-deck smoke source of any kind in the crate --
    // `apply_fire_failures` injected only into the cargo bays, the nacelle
    // cowls and the APU compartment -- so the eight lavatory detectors
    // `deep::sensors` models could physically never alarm.
    //
    // GENERIC magnitudes, like every other figure in this function: two
    // orders below a cargo-compartment fire, because a bin fire is small
    // and contained, and sized so a full-severity one takes the deck past
    // the detectors' own obscuration threshold inside a minute rather than
    // instantly -- which is the order of magnitude the detection
    // requirement is written in. Not a certification test value.
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

    // The two cabin decks. `thermal_zones` has no lavatory zone of its own
    // and this does not invent one: the lavatory extract draws the deck's
    // own air past the detector (`sensors::live_discrete`'s own note on
    // these eight detectors says exactly that), so the deck concentration
    // is what a lavatory detector sees, and the deck is where the smoke is
    // injected.
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
}

// ---------------------------------------------------------------------------
// ATA 30: ice & rain protection (anti-ice duct leaks, vent-scoop icing).
// ---------------------------------------------------------------------------

fn register_ice_and_duct_failures(r: &mut Registry) {
    const WING_DUCT_LEAK_MAX_HEAT_W: f64 = 30_000.0; // GENERIC: order of magnitude of a hot bleed anti-ice duct's own leak enthalpy flow, same class of figure as physics::bays.rs's own bleed-leak sizing
    const NACELLE_DUCT_LEAK_MAX_HEAT_W: f64 = 20_000.0;

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
            model_field: format!("thermal_zones::network::Zone.injected_heat_w (via ThermalNetwork::inject_heat_w(zones.{field}, magnitude*{WING_DUCT_LEAK_MAX_HEAT_W}))"),
            magnitude: format!("0..1, fraction of the reference {WING_DUCT_LEAK_MAX_HEAT_W:.0} W full-severity duct leak"),
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
            model_field: format!("thermal_zones::network::Zone.injected_heat_w (via ThermalNetwork::inject_heat_w(zones.{field}, magnitude*{NACELLE_DUCT_LEAK_MAX_HEAT_W}))"),
            magnitude: format!("0..1, fraction of the reference {NACELLE_DUCT_LEAK_MAX_HEAT_W:.0} W full-severity duct leak"),
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

// ---------------------------------------------------------------------------
// ATA 32: landing gear bay doors.
// ---------------------------------------------------------------------------

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

// ---------------------------------------------------------------------------
// ATA 36 / 49: pneumatic and APU bleed duct leaks outside bays.rs's own
// WingRootBleed zone (pylon runs, tail cone APU duct run).
// ---------------------------------------------------------------------------

fn register_pneumatic_and_apu_failures(r: &mut Registry) {
    // The pylon leak is no longer a reference wattage at all: it is a crack
    // area (2% of the duct's own 4 in bore), whose choked mass flow and
    // enthalpy above the bay are computed every tick from the engine's real
    // bleed port condition. `live::pylon_bleed_leak_heat_w` carries the full
    // derivation. The old `40_000.0 W` fixed figure was both 2.7x larger
    // than the 200 C/44 psi source it claimed to come from and unbounded by
    // that source, so it could drive a bay hotter than the air leaking into
    // it.
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

    const APU_DUCT_LEAK_MAX_HEAT_W: f64 = 25_000.0;
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
        model_field: format!("thermal_zones::network::Zone.injected_heat_w (via ThermalNetwork::inject_heat_w(zones.tail_cone, magnitude*{APU_DUCT_LEAK_MAX_HEAT_W}))"),
        magnitude: format!("0..1, fraction of the reference {APU_DUCT_LEAK_MAX_HEAT_W:.0} W full-severity leak"),
        effect: "TailCone runs hot, conducting into ApuCompartment and BodyGearWell through the real structure links.".to_string(),
    });
}

// ---------------------------------------------------------------------------
// ATA 53: fuselage thermal/acoustic insulation.
// ---------------------------------------------------------------------------

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

// ---------------------------------------------------------------------------
// ECAM alerts.
// ---------------------------------------------------------------------------

fn register_ecam_alerts(r: &mut Registry) {
    // Cargo smoke is announced by the fire protection system, which owns
    // the three alerts (`fire_ice::registry`, ATA 26, carrying the
    // detection loops, the confirmation time, the take-off inhibit and the
    // suppression procedure). What this area adds is the physical side: a
    // bay's smoke concentration rising from a fire or an overheat modelled
    // here, at the same 2e-4 kg/m3 the detectors see.
    let cargo = [
        ("CARGO_SMOKE_FWD", "THERMAL_ZONE_CARGOFWD_SMOKE_CONCENTRATION", 1u16),
        ("CARGO_SMOKE_AFT", "THERMAL_ZONE_CARGOAFT_SMOKE_CONCENTRATION", 2u16),
        ("CARGO_SMOKE_BULK", "THERMAL_ZONE_CARGOBULK_SMOKE_CONCENTRATION", 3u16),
    ];
    for (key, smoke_var, fnum) in cargo {
        r.contribute(key).when(var(smoke_var).gt(0.0002)).raised_by(&[failure_id(Area::ThermalZones, 26, fnum)]);
    }

    for engine in 1..=4u64 {
        let key = format!("ENG_{engine}_NAC_OVHT");
        let title = format!("ENG {engine} NAC OVHT");
        let temp_var = format!("THERMAL_ZONE_NACELLECOWL{engine}_TEMPERATURE_C");
        let master_var = format!("ENGINE_MASTER:{engine}");
        r.alert(
            EcamAlert::new(&key, 71, &title, Level::Caution, var(&temp_var).gt(150.0))
                .confirm(5.0)
                .step(line(&format!("ENG {engine} MASTER"), "OFF").done(var(&master_var).off()).only_if(var(&temp_var).gt(250.0)))
                .status_line(&format!("INOP: ENG {engine} A ICE"))
                .raised_by(&[
                    failure_id(Area::ThermalZones, 30, 2 + engine as u16),
                    failure_id(Area::ThermalZones, 30, 6 + engine as u16),
                    failure_id(Area::ThermalZones, 26, 3 + engine as u16),
                ]),
        );
    }

    // The APU compartment running away past 250 C is a third way to reach
    // the one APU FIRE warning `fire_ice::registry` owns (the APU's own
    // detection loop is the second, contributed from `apu::registry`).
    r.contribute("APU_FIRE").when(var("THERMAL_ZONE_APUCOMPARTMENT_TEMPERATURE_C").gt(250.0)).raised_by(&[failure_id(Area::ThermalZones, 26, 8)]);

    r.alert(
        EcamAlert::new("APU_COMPT_OVHT", 49, "APU COMPT OVHT", Level::Caution, var("THERMAL_ZONE_APUCOMPARTMENT_TEMPERATURE_C").gt(120.0))
            .confirm(10.0)
            .step(line("APU MASTER SW", "OFF").done(var("APU_MASTER_SW").off()))
            .status_line("APU INOP")
            .raised_by(&[failure_id(Area::ThermalZones, 21, 7), failure_id(Area::ThermalZones, 49, 1)]),
    );

    let wings = [("L_WING_A_ICE_DUCT_LEAK", "L WING A ICE DUCT LEAK", "THERMAL_ZONE_WINGLELEFT_TEMPERATURE_C", "WING_ANTI_ICE_SW:L", 1u16), ("R_WING_A_ICE_DUCT_LEAK", "R WING A ICE DUCT LEAK", "THERMAL_ZONE_WINGLERIGHT_TEMPERATURE_C", "WING_ANTI_ICE_SW:R", 2u16)];
    for (key, title, temp_var, sw_var, fnum) in wings {
        r.alert(
            EcamAlert::new(key, 30, title, Level::Caution, var(temp_var).gt(120.0))
                .confirm(10.0)
                .step(line("WING A ICE", "OFF").done(var(sw_var).off()))
                .status_line("INOP: WING A ICE")
                .raised_by(&[failure_id(Area::ThermalZones, 30, fnum)]),
        );
    }

    r.alert(
        EcamAlert::new("ECS_PACK_BAY_OVHT", 21, "ECS PACK BAY OVHT", Level::Caution, var("THERMAL_ZONE_BELLYFAIRINGPACKS_TEMPERATURE_C").gt(90.0))
            .confirm(10.0)
            .step(line("PACK 1", "OFF").done(var("PACK_1_SW").off()))
            .step(line("PACK 2", "OFF").done(var("PACK_2_SW").off()).after(5.0))
            .status_line("INOP: PACK 1+2")
            .raised_by(&[failure_id(Area::ThermalZones, 21, 6)]),
    );

    r.alert(
        EcamAlert::new("AVIONICS_VENT_FAULT", 21, "AVIONICS VENT FAULT", Level::Caution, any(vec![var("THERMAL_COMPONENT_MAINAVIONICSWIRINGBUNDLE_DAMAGE").gt(0.0), var("THERMAL_ZONE_MAINAVIONICS_TEMPERATURE_C").gt(70.0)]))
            .confirm(10.0)
            .step(line("AVNCS VENT OVRD", "ON").done(var("AVIONICS_VENT_OVRD_SW").on()))
            .status_line("INOP: AVNCS VENT SYS 1")
            .raised_by(&[failure_id(Area::ThermalZones, 21, 1), failure_id(Area::ThermalZones, 21, 2)]),
    );
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
        assert!(!r.alerts.is_empty());
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
        // Sanity cross-check: every zone name this file's failure `effect`
        // strings assume (e.g. "CargoFwd") is a real zone `topology_a380`
        // actually builds, so the documentation cannot silently drift from
        // the model.
        let a380 = topology_a380::build();
        let names: Vec<&str> = a380.network.zones.iter().map(|z| z.name).collect();
        for expected in ["MainAvionics", "UpperAvionics", "CargoFwd", "CargoAft", "CargoBulk", "WingLeLeft", "WingLeRight", "TailCone", "ApuCompartment", "BellyFairingPacks", "CrownArea"] {
            assert!(names.contains(&expected), "expected zone {expected} to exist in topology_a380::build()");
        }
    }
}
