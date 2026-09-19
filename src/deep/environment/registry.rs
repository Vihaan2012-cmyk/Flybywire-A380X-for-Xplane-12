//! Registers this directory's failures, components and ECAM alerts with the
//! crate-wide `Registry` (`docs/deep/BRIEF.md`, "Registering failures,
//! components and ECAM alerts"). Area is `Area::Environment` (14) for
//! every id here.
//!
//! ## ATA allocation ledger (keep in sync — `n` is sequential per ATA)
//! - 21 (air conditioning / cabin): 1 = ash cabin odour
//! - 24 (electrical power): 1 = lightning bus/computer transient
//! - 32 (landing gear): 1 = bird strike nose gear, 2 = runway contamination
//!   friction/hydroplaning loss
//! - 34 (navigation / air data): 1 = bird strike probe blockage, 2 = ash
//!   pitot blockage, 3 = lightning magnetic compass error, 4 = hail
//!   probe/antenna damage
//! - 53 (fuselage): 1 = bird strike radome, 2 = lightning radome, 3 =
//!   lightning composite extremity, 4 = hail radome
//! - 56 (windows): 1 = bird strike windshield, 2 = hail windshield, 3 = ash
//!   windshield abrasion
//! - 57 (wings): 1 = bird strike leading edge, 2 = hail leading edge/slat
//! - 71 (power plant, general): 1 = hail nacelle/cowl damage
//! - 72 (engine): 1 = bird strike fan, 2 = bird strike core ingestion FOD,
//!   3 = ice crystal core accretion, 4 = ice crystal rollback/flameout
//!   risk, 5 = ash vane glassing, 6 = ash compressor erosion, 7 =
//!   hail/ice engine ingestion
//!
//! ## Vars this directory's outputs still need publishing under
//! Nothing here touches `VariableRegistry` (self-contained, per the brief);
//! the ECAM triggers below are written against the Var names each output
//! struct's field should be published as once the datarefs layer wires
//! them up. Until then `Registry::validate()` still passes (it only checks
//! failure/component/alert cross-references, not that a Var exists), but
//! the alerts will simply never trigger at runtime. New Vars needed:
//! `ENV_BIRD_FAN_DAMAGE:<1-4>`, `ENV_BIRD_CORE_FOD:<1-4>`,
//! `ENV_BIRD_WINDSHIELD_DAMAGE:<1-6>`, `ENV_BIRD_RADOME_DAMAGE`,
//! `ENV_BIRD_WING_LE_DRAG:<1-12>`, `ENV_BIRD_NOSE_GEAR_DAMAGE`,
//! `ENV_BIRD_PROBE_BLOCKED:<1-6>` — one Var per `bird_strike::StrikeOutcome`
//! field, indexed the same way as the corresponding `ImpactTarget`.
//! Lightning adds: `ENV_LTG_RADOME_DAMAGE`, `ENV_LTG_STRUCTURE_DAMAGE`,
//! `ENV_LTG_COMPASS_ERROR_DEG`, `ENV_LTG_BUS_UPSET:<name>` (one per
//! `lightning::BusId` variant) from `lightning::LightningEvent`. Hail adds:
//! `ENV_HAIL_RADOME_DAMAGE`, `ENV_HAIL_WINDSHIELD_DAMAGE:<1-6>`,
//! `ENV_HAIL_WING_LE_DAMAGE:<1-12>` from `hail::HailOutcome`. Volcanic ash
//! adds: `ENV_ASH_FLOW_CAPACITY_LOSS:<1-4>`, `ENV_ASH_COMPRESSOR_EFF_LOSS:<1-4>`,
//! `ENV_ASH_WINDSHIELD_LOSS`, `ENV_ASH_PITOT_BLOCKED:<1-6>`,
//! `ENV_ASH_CABIN_ODOR`, `ENV_ASH_FLAMEOUT_RISK:<1-4>` from
//! `volcanic_ash::AshOutputs`, one set per engine where noted. Ice crystal
//! icing adds: `ENV_ICE_CORE_FLOW_LOSS:<1-4>`, `ENV_ICE_ROLLBACK_RISK:<1-4>`,
//! `ENV_ICE_FLAMEOUT_RISK:<1-4>`, `ENV_ICE_SHEDDING_EVENT:<1-4>` from
//! `ice_crystal_icing::IceCrystalOutputs`. Runway contamination adds:
//! `ENV_RWY_MU_EFFECTIVE`, `ENV_RWY_HYDROPLANING` from
//! `runway_contamination::RunwayFrictionOutput` (the braking/gear system's
//! own per-gear friction demand reads these). Wind shear adds:
//! `ENV_F_FACTOR` from `wind_shear::f_factor`. Turbulence
//! (`wind_shear::TurbulenceGusts`) has no ECAM entry -- it feeds the
//! flight model directly, as on the real aircraft.

use crate::deep::api::*;

pub fn register(r: &mut Registry) {
    register_bird_strike(r);
    register_lightning(r);
    register_hail(r);
    register_volcanic_ash(r);
    register_ice_crystal_icing(r);
    register_runway_contamination(r);
    register_wind_shear(r);
}

fn register_bird_strike(r: &mut Registry) {
    // ---- Components -------------------------------------------------
    r.component(ComponentDef {
        id: "72_env.fan_bird_damage".into(),
        area: Area::Environment,
        ata: 72,
        name: "Fan blade set (bird-strike damage state)".into(),
        params: vec![ParamDef { name: "damage_frac".into(), meaning: "0 undamaged .. 1 destructive (fan-blade fracture/imbalance)".into(), healthy: 0.0 }],
        failures: vec![],
    });
    r.component(ComponentDef {
        id: "72_env.core_fod".into(),
        area: Area::Environment,
        ata: 72,
        name: "IP compressor front stage (foreign-object ingestion state), x4 engines".into(),
        params: vec![ParamDef { name: "ingested_mass_frac".into(), meaning: "fraction of the strike's mass that entered the core flow path vs. the bypass duct".into(), healthy: 0.0 }],
        failures: vec![],
    });
    r.component(ComponentDef {
        id: "56_env.windshield".into(),
        area: Area::Environment,
        ata: 56,
        name: "Flight-deck windshield panel, x6 panels".into(),
        params: vec![ParamDef { name: "damage_frac".into(), meaning: "0 undamaged, >0.6 cracked, >=1.0 penetrated".into(), healthy: 0.0 }],
        failures: vec![],
    });
    r.component(ComponentDef {
        id: "53_env.radome".into(),
        area: Area::Environment,
        ata: 53,
        name: "Nose radome".into(),
        params: vec![ParamDef { name: "damage_frac".into(), meaning: "0 undamaged .. 1 destroyed/departed".into(), healthy: 0.0 }],
        failures: vec![],
    });
    r.component(ComponentDef {
        id: "57_env.wing_leading_edge".into(),
        area: Area::Environment,
        ata: 57,
        name: "Wing leading edge segment, x12 (6 per side, root to tip), L/R".into(),
        params: vec![ParamDef { name: "dent_drag_delta_cd".into(), meaning: "local drag-coefficient increment from a dent's separated flow".into(), healthy: 0.0 }],
        failures: vec![],
    });
    r.component(ComponentDef {
        id: "32_env.nose_gear".into(),
        area: Area::Environment,
        ata: 32,
        name: "Nose landing gear (bird-strike damage state)".into(),
        params: vec![ParamDef { name: "damage_frac".into(), meaning: "0 undamaged .. 1 unsafe to extend/retract or steer".into(), healthy: 0.0 }],
        failures: vec![],
    });
    r.component(ComponentDef {
        id: "34_env.air_data_probe".into(),
        area: Area::Environment,
        ata: 34,
        name: "Pitot/AoA/TAT probe, x6 forward fuselage".into(),
        params: vec![ParamDef { name: "blocked".into(), meaning: "0 clear, 1 blocked (bird strike deforms or plugs the orifice outright)".into(), healthy: 0.0 }],
        failures: vec![],
    });

    // ---- Failures -----------------------------------------------------
    let fan = r.failure(FailureDef {
        id: failure_id(Area::Environment, 72, 1),
        area: Area::Environment,
        ata: 72,
        name: "Bird strike fan blade damage".into(),
        component: "72_env.fan_bird_damage".into(),
        model_field: "environment::bird_strike::StrikeOutcome.fan_damage_frac".into(),
        magnitude: "0 no damage .. 1 destructive blade fracture/imbalance, from impact energy vs. the CS-E 800 large-bird reference energy".into(),
        effect: "engine vibration, thrust loss, possible surge/flameout (consumed by the engine model)".into(),
    });
    let core = r.failure(FailureDef {
        id: failure_id(Area::Environment, 72, 2),
        area: Area::Environment,
        ata: 72,
        name: "Bird strike core ingestion FOD".into(),
        component: "72_env.core_fod".into(),
        model_field: "environment::bird_strike::StrikeOutcome.core_ingestion_frac".into(),
        magnitude: "fraction of ingested bird mass reaching the IP compressor front stage (bypass-ratio flow split)".into(),
        effect: "IP compressor blade damage, compressor efficiency loss".into(),
    });
    let wind = r.failure(FailureDef {
        id: failure_id(Area::Environment, 56, 1),
        area: Area::Environment,
        ata: 56,
        name: "Bird strike windshield damage".into(),
        component: "56_env.windshield".into(),
        model_field: "environment::bird_strike::StrikeOutcome.windshield_crack / .windshield_penetrated".into(),
        magnitude: "impact energy / CS-25.775(b) 4 lb-at-Vc reference energy, clamped 0..1; >0.6 cracks, >=1.0 penetrates".into(),
        effect: "loss of visibility, possible depressurisation if penetrated".into(),
    });
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
    // No dedicated ECAM entry on the real aircraft for a leading-edge
    // dent; registered for the Components/Failures pages only (see the
    // note after this function's alerts).
    let _wing_le = r.failure(FailureDef {
        id: failure_id(Area::Environment, 57, 1),
        area: Area::Environment,
        ata: 57,
        name: "Bird strike leading edge dent".into(),
        component: "57_env.wing_leading_edge".into(),
        model_field: "environment::bird_strike::StrikeOutcome.leading_edge_dent_drag_delta_cd".into(),
        magnitude: "impact energy / CS-25.631 8 lb-at-Vc reference energy, clamped 0..1".into(),
        effect: "local drag increment; at 1.0 implies a structural inspection item".into(),
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
    let probe = r.failure(FailureDef {
        id: failure_id(Area::Environment, 34, 1),
        area: Area::Environment,
        ata: 34,
        name: "Bird strike air data probe blockage".into(),
        component: "34_env.air_data_probe".into(),
        model_field: "environment::bird_strike::StrikeOutcome.probe_blocked".into(),
        magnitude: "0 clear, 1 blocked (binary: any strike on a probe is assumed to disable it)".into(),
        effect: "unreliable airspeed/AoA on the affected probe".into(),
    });

    // ---- ECAM alerts ----------------------------------------------------
    // Written against the Var names documented in this file's header,
    // which the datarefs layer still needs to publish from
    // `bird_strike::StrikeOutcome` (this module is self-contained and
    // does not touch `VariableRegistry` itself, per the brief).
    r.alert(
        EcamAlert::new("ENV_ENG_1_BIRD_FAN_DAMAGE", 72, "ENG 1 FAN DAMAGE", Level::Warning, var("ENV_BIRD_FAN_DAMAGE:1").gt(0.3))
            .confirm(1.0)
            .inhibit(&[Phase::LiftOff, Phase::Below80Kt])
            .step(line("THR LEVER 1", "IDLE").done(var("AUTOTHRUST_TLA:1").le(0.0)))
            .step(line("ENG 1 MASTER", "OFF").only_if(var("ENV_BIRD_FAN_DAMAGE:1").ge(0.7)).done(var("ENGINE_MASTER:1").off()).after(5.0))
            .status_line("ENG 1 ................ FAULT")
            .inop_sys("ENG 1 REVERSER")
            .raised_by(&[fan, core]),
    );
    r.alert(
        EcamAlert::new("ENV_WINDSHIELD_1_DAMAGE", 56, "L WINDSHIELD DAMAGE", Level::Warning, var("ENV_BIRD_WINDSHIELD_DAMAGE:1").ge(1.0))
            .confirm(0.0)
            .step(line("CABIN ALTITUDE", "MONITOR").colour("green"))
            .step(line("SPEED", "REDUCE").done(var("AIRSPEED_KT").le(280.0)))
            .status_line("L WINDSHIELD ......... PENETRATED")
            .raised_by(&[wind]),
    );
    r.alert(
        EcamAlert::new("ENV_RADOME_DAMAGE", 53, "RADOME DAMAGE", Level::Caution, var("ENV_BIRD_RADOME_DAMAGE").gt(0.5))
            .confirm(1.0)
            .step(line("WEATHER RADAR", "OFF").done(var("WXR_POWER").off()))
            .status_line("WXR ................... FAULT")
            .inop_sys("WEATHER RADAR")
            .raised_by(&[radome]),
    );
    r.alert(
        EcamAlert::new("ENV_NOSE_GEAR_BIRD_DAMAGE", 32, "NOSE L/G BIRD DAMAGE", Level::Caution, var("ENV_BIRD_NOSE_GEAR_DAMAGE").gt(0.4))
            .confirm(0.5)
            .step(line("L/G", "DO NOT RETRACT").colour("amber"))
            .status_line("L/G ................... FAULT")
            .raised_by(&[gear]),
    );
    r.alert(
        EcamAlert::new("ENV_AIR_DATA_PROBE_BLOCKED", 34, "AIR DATA DISAGREE", Level::Caution, any(vec![var("ENV_BIRD_PROBE_BLOCKED:1").on(), var("ENV_BIRD_PROBE_BLOCKED:2").on(), var("ENV_BIRD_PROBE_BLOCKED:3").on()]))
            .confirm(3.0)
            .step(line("ADR", "CHECK").colour("green"))
            .status_line("AIR DATA .............. CHECK")
            .raised_by(&[probe]),
    );
    // Wing leading edge and general core-FOD downstream effects (vibration,
    // compressor efficiency) are left for the engine/structures owners'
    // own ECAM alerts to raise from `ENV_BIRD_CORE_FOD`/`ENV_BIRD_WING_LE_DRAG`
    // once wired, rather than duplicated here.
}

fn register_lightning(r: &mut Registry) {
    // ---- Components -------------------------------------------------
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
    r.component(ComponentDef {
        id: "24_env.bus_transient".into(),
        area: Area::Environment,
        ata: 24,
        name: "Bus/computer conducted-transient exposure, one per BusId (PRIM/SEC/FMGC/ADIRS/standby instruments/FADEC/IFE)".into(),
        params: vec![ParamDef { name: "peak_volts".into(), meaning: "induced transient this strike put on the bus; upset likely above the GENERIC 50 V threshold".into(), healthy: 0.0 }],
        failures: vec![],
    });

    // ---- Failures -----------------------------------------------------
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
    // Composite extremity burn is a maintenance/inspection item, not a
    // flight-deck alert on the real aircraft, so its id is not raised by
    // any alert below (still registered for the Components/Failures pages).
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
    let transient = r.failure(FailureDef {
        id: failure_id(Area::Environment, 24, 1),
        area: Area::Environment,
        ata: 24,
        name: "Lightning induced bus/computer transient".into(),
        component: "24_env.bus_transient".into(),
        model_field: "environment::lightning::LightningEvent.transients[].peak_volts".into(),
        magnitude: "peak current x GENERIC per-bus exposure factor (0.06 shielded flight-control computers .. 0.35 nacelle-mounted FADEC)".into(),
        effect: "possible reset/data corruption on buses above the GENERIC 50 V upset threshold".into(),
    });

    // ---- ECAM alerts ----------------------------------------------------
    r.alert(
        EcamAlert::new("ENV_LTG_RADOME_DAMAGE", 53, "RADOME DAMAGE", Level::Caution, var("ENV_LTG_RADOME_DAMAGE").gt(0.5))
            .confirm(1.0)
            .step(line("WEATHER RADAR", "OFF").done(var("WXR_POWER").off()))
            .status_line("WXR ................... FAULT")
            .inop_sys("WEATHER RADAR")
            .raised_by(&[radome]),
    );
    r.alert(
        EcamAlert::new("ENV_LTG_COMPASS_FAULT", 34, "STBY COMPASS FAULT", Level::Advisory, var("ENV_LTG_COMPASS_ERROR_DEG").gt(2.0))
            .confirm(2.0)
            .step(line("STBY COMPASS", "DISREGARD").colour("green"))
            .status_line("STBY COMPASS SWING ..... REQUIRED")
            .raised_by(&[compass]),
    );
    r.alert(
        EcamAlert::new("ENV_LTG_FADEC_TRANSIENT", 24, "ENG FADEC TRANSIENT", Level::Caution, var("ENV_LTG_BUS_UPSET:EngineFadec").on())
            .confirm(0.5)
            .step(line("ENG PARAMETERS", "MONITOR").colour("green"))
            .status_line("ENG FADEC ............. CHECK")
            .raised_by(&[transient]),
    );
}

fn register_hail(r: &mut Registry) {
    // ---- Components -------------------------------------------------
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
    r.component(ComponentDef {
        id: "56_env.windshield_hail".into(),
        area: Area::Environment,
        ata: 56,
        name: "Flight-deck windshield panel (hail damage state), x6 panels".into(),
        params: vec![
            ParamDef { name: "damage_frac".into(), meaning: "0 undamaged .. 1 significant (GENERIC 30mm-at-VMO reference), persistent/cumulative".into(), healthy: 0.0 },
            ParamDef { name: "window_heat_fault".into(), meaning: "0/1: the conductive heating film has faulted above a GENERIC 0.3 severity".into(), healthy: 0.0 },
            ParamDef { name: "leak_area_m2".into(), meaning: "GENERIC pressurisation leak area, only above 0.9 severity".into(), healthy: 0.0 },
        ],
        failures: vec![],
    });
    r.component(ComponentDef {
        id: "57_env.wing_leading_edge_hail".into(),
        area: Area::Environment,
        ata: 57,
        name: "Wing leading edge / slat segment (hail damage state), x12, L/R".into(),
        params: vec![
            ParamDef { name: "damage_frac".into(), meaning: "0 undamaged .. 1 significant (GENERIC 30mm-at-VMO reference), persistent/cumulative".into(), healthy: 0.0 },
            ParamDef { name: "clmax_delta".into(), meaning: "GENERIC max-lift-coefficient penalty, 0 .. -0.05".into(), healthy: 0.0 },
            ParamDef { name: "slat_jam_risk_frac".into(), meaning: "GENERIC slat extend/retract mechanism jam risk, ramps above 0.5 damage".into(), healthy: 0.0 },
        ],
        failures: vec![],
    });
    r.component(ComponentDef {
        id: "72_env.engine_hail_ingestion".into(),
        area: Area::Environment,
        ata: 72,
        name: "Fan/compressor (hail ingestion state), x4 engines".into(),
        params: vec![
            ParamDef { name: "fan_damage_frac".into(), meaning: "0 none .. 1 destructive, persistent/cumulative".into(), healthy: 0.0 },
            ParamDef { name: "compressor_efficiency_loss_frac".into(), meaning: "0 none .. 0.25 ceiling (GENERIC), irreversible".into(), healthy: 0.0 },
            ParamDef { name: "flameout_risk_frac".into(), meaning: "instantaneous, worse at low N1 per CS-E 790's low-power ingestion concern".into(), healthy: 0.0 },
        ],
        failures: vec![],
    });
    r.component(ComponentDef {
        id: "71_env.nacelle_hail".into(),
        area: Area::Environment,
        ata: 71,
        name: "Engine nacelle inlet lip/cowl (hail damage state), x4 engines".into(),
        params: vec![ParamDef { name: "damage_frac".into(), meaning: "0 undamaged .. 1 significant (GENERIC 30mm-at-VMO reference), persistent/cumulative".into(), healthy: 0.0 }],
        failures: vec![],
    });
    r.component(ComponentDef {
        id: "34_env.air_data_probe_hail".into(),
        area: Area::Environment,
        ata: 34,
        name: "Pitot/AoA/TAT probe or external antenna (hail damage state), x6".into(),
        params: vec![ParamDef { name: "damage_frac".into(), meaning: "0 undamaged .. 1 significant (GENERIC 15mm-at-VMO reference), persistent/cumulative".into(), healthy: 0.0 }],
        failures: vec![],
    });

    // ---- Failures -----------------------------------------------------
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
    let windshield = r.failure(FailureDef {
        id: failure_id(Area::Environment, 56, 2),
        area: Area::Environment,
        ata: 56,
        name: "Hail windshield damage".into(),
        component: "56_env.windshield_hail".into(),
        model_field: "environment::hail::HailOutcome.damage_frac / .window_heat_fault / .visibility_loss_frac / .leak_area_m2 (target = Windshield)".into(),
        magnitude: "cumulative impact-energy density / a GENERIC 30mm-hailstone-at-VMO reference x 15 hits".into(),
        effect: "visibility loss; window-heat film fault above 0.3; a pressurisation leak above 0.9".into(),
    });
    let wing_le = r.failure(FailureDef {
        id: failure_id(Area::Environment, 57, 2),
        area: Area::Environment,
        ata: 57,
        name: "Hail leading edge / slat damage".into(),
        component: "57_env.wing_leading_edge_hail".into(),
        model_field: "environment::hail::HailOutcome.damage_frac / .clmax_delta / .slat_jam_risk_frac (target = WingLeadingEdge)".into(),
        magnitude: "cumulative impact-energy density / a GENERIC 30mm-hailstone-at-VMO reference x 15 hits".into(),
        effect: "drag increase, max-lift penalty, and above 0.5 a slat mechanism jam risk".into(),
    });
    let engine = r.failure(FailureDef {
        id: failure_id(Area::Environment, 72, 7),
        area: Area::Environment,
        ata: 72,
        name: "Hail/ice engine ingestion".into(),
        component: "72_env.engine_hail_ingestion".into(),
        model_field: "environment::hail::HailOutcome.fan_damage_frac / .compressor_efficiency_loss_frac / .flameout_risk_frac (target = EngineInlet)".into(),
        magnitude: "fan: cumulative energy density / reference; compressor: GENERIC erosion per kg ingested, capped 0.25; flameout: ingested mass / a GENERIC tolerance that shrinks with N1 (CS-E 790 / 14 CFR 33.68)".into(),
        effect: "fan damage, permanent compressor efficiency loss, elevated flameout/roll-back risk especially at low power".into(),
    });
    // No dedicated ECAM entry here; a real ADR/antenna-fault message would
    // be raised by the sensors/avionics owner off the same magnitude once
    // wired, the same convention used for bird-strike/ash probe damage.
    let _probe = r.failure(FailureDef {
        id: failure_id(Area::Environment, 34, 4),
        area: Area::Environment,
        ata: 34,
        name: "Hail probe/antenna damage".into(),
        component: "34_env.air_data_probe_hail".into(),
        model_field: "environment::hail::HailOutcome.damage_frac (target = Probe)".into(),
        magnitude: "cumulative impact-energy density / a GENERIC 15mm-at-VMO reference x 15 hits".into(),
        effect: "unreliable airspeed/AoA or lost antenna function".into(),
    });
    // Nacelle denting is a maintenance/inspection item on the real
    // aircraft, not its own flight-deck alert; registered for the
    // Components/Failures pages only.
    let _nacelle = r.failure(FailureDef {
        id: failure_id(Area::Environment, 71, 1),
        area: Area::Environment,
        ata: 71,
        name: "Hail nacelle/cowl damage".into(),
        component: "71_env.nacelle_hail".into(),
        model_field: "environment::hail::HailOutcome.damage_frac / .drag_delta_cd (target = Nacelle)".into(),
        magnitude: "cumulative impact-energy density / a GENERIC 30mm-hailstone-at-VMO reference x 15 hits".into(),
        effect: "drag increase from cowl/inlet-lip denting".into(),
    });

    // ---- ECAM alerts ----------------------------------------------------
    r.alert(
        EcamAlert::new("ENV_HAIL_RADOME_DAMAGE", 53, "RADOME DAMAGE", Level::Caution, var("ENV_HAIL_RADOME_DAMAGE").gt(0.5))
            .confirm(1.0)
            .step(line("WEATHER RADAR", "OFF").done(var("WXR_POWER").off()))
            .status_line("WXR ................... FAULT")
            .inop_sys("WEATHER RADAR")
            .raised_by(&[radome]),
    );
    r.alert(
        EcamAlert::new("ENV_HAIL_WINDOW_HEAT_FAULT", 56, "WINDOW HEAT L FAULT", Level::Caution, var("ENV_HAIL_WINDOW_HEAT_FAULT:1").on())
            .confirm(1.0)
            .step(line("WINDOW HEAT L", "MONITOR").colour("green"))
            .status_line("L WINDOW HEAT ......... FAULT")
            .inop_sys("L WINDOW HEAT")
            .raised_by(&[windshield]),
    );
    r.alert(
        EcamAlert::new("ENV_HAIL_WINDSHIELD_DAMAGE", 56, "WINDSHIELD DAMAGE", Level::Advisory, var("ENV_HAIL_WINDSHIELD_DAMAGE:1").gt(0.5))
            .confirm(2.0)
            .step(line("SPEED", "REDUCE").done(var("AIRSPEED_KT").le(280.0)))
            .status_line("L WINDSHIELD .......... CHECK")
            .raised_by(&[windshield]),
    );
    r.alert(
        EcamAlert::new("ENV_HAIL_SLAT_JAM_RISK", 57, "SLAT FAULT", Level::Caution, var("ENV_HAIL_SLAT_JAM_RISK:1").gt(0.3))
            .confirm(2.0)
            .step(line("SLAT", "DO NOT OPERATE").colour("amber"))
            .status_line("SLAT ................... FAULT")
            .raised_by(&[wing_le]),
    );
    r.alert(
        EcamAlert::new("ENV_HAIL_ENG_1_FLAMEOUT_RISK", 72, "ENG 1 STALL", Level::Warning, var("ENV_HAIL_FLAMEOUT_RISK:1").gt(0.6))
            .confirm(1.0)
            .step(line("THR LEVER 1", "IDLE").done(var("AUTOTHRUST_TLA:1").le(0.0)))
            .step(line("IF N2 < IDLE: ENG 1 RELIGHT", "WHEN OUT OF HAIL").only_if(var("ENV_HAIL_FLAMEOUT_RISK:1").ge(1.0)).colour("amber"))
            .status_line("ENG 1 ................. DEGRADED")
            .raised_by(&[engine]),
    );
}

fn register_volcanic_ash(r: &mut Registry) {
    // ---- Components -------------------------------------------------
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

    // ---- Failures -----------------------------------------------------
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
    // No dedicated ECAM entry on the real aircraft for progressive
    // windshield haze; registered for the Components/Failures pages only.
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

    // ---- ECAM alerts ----------------------------------------------------
    r.alert(
        EcamAlert::new("ENV_ASH_ENG_1_DEGRADED", 72, "ENG 1 ASH DEGRADED", Level::Caution, any(vec![var("ENV_ASH_FLOW_CAPACITY_LOSS:1").gt(0.2), var("ENV_ASH_COMPRESSOR_EFF_LOSS:1").gt(0.05)]))
            .confirm(5.0)
            .step(line("ENG 1 PARAMETERS", "MONITOR").colour("green"))
            .step(line("IF FLAMEOUT: ENG 1 RELIGHT", "WHEN CLEAR OF ASH").only_if(var("ENV_ASH_FLAMEOUT_RISK:1").ge(1.0)).colour("amber"))
            .status_line("ENG 1 ................. DEGRADED")
            .raised_by(&[glassing, erosion]),
    );
    r.alert(
        EcamAlert::new("ENV_ASH_ENCOUNTER", 21, "VOLCANIC ASH", Level::Warning, var("ENV_ASH_CABIN_ODOR").gt(0.1))
            .confirm(1.0)
            .step(line("ALL ENG START SWITCHES", "IGN/START").colour("cyan"))
            .step(line("EXIT ASH", "TURN OR DESCEND").colour("amber"))
            .status_line("AIR DATA .............. CHECK")
            .raised_by(&[pitot]),
    );
}

fn register_ice_crystal_icing(r: &mut Registry) {
    // ---- Components -------------------------------------------------
    r.component(ComponentDef {
        id: "72_env.ice_crystal_accretion".into(),
        area: Area::Environment,
        ata: 72,
        name: "IP compressor front stage / splitter (ice crystal accretion state), x4 engines".into(),
        params: vec![ParamDef { name: "flow_capacity_loss_frac".into(), meaning: "0 clear .. 1 fully blocked; sheds and resets near 0.6 (GENERIC threshold)".into(), healthy: 0.0 }],
        failures: vec![],
    });

    // ---- Failures -----------------------------------------------------
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

    // ---- ECAM alerts ----------------------------------------------------
    r.alert(
        EcamAlert::new("ENV_ICE_ENG_1_ROLLBACK", 72, "ENG 1 ICE ROLLBACK", Level::Caution, var("ENV_ICE_ROLLBACK_RISK:1").gt(0.15))
            .confirm(3.0)
            .step(line("ENG ANTI ICE", "ON").done(var("ENGINE_ANTI_ICE:1").on()))
            .step(line("DESCEND OR EXIT CLOUD", "IF POSSIBLE").colour("amber"))
            .status_line("ENG 1 ................. DEGRADED")
            .raised_by(&[accretion, rollback]),
    );
}

fn register_runway_contamination(r: &mut Registry) {
    // ---- Components -------------------------------------------------
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

    // ---- Failures -----------------------------------------------------
    // Runway contamination is an environmental condition, not a component
    // fault, but the resulting friction loss/hydroplaning is registered as
    // a "failure" in the sense the brief uses it (a magnitude 0..1 that
    // drives a model field with a real effect) so the braking system's
    // stopping-distance model has a catalogued cause to point to.
    let hydroplaning = r.failure(FailureDef {
        id: failure_id(Area::Environment, 32, 2),
        area: Area::Environment,
        ata: 32,
        name: "Runway contamination friction/hydroplaning loss".into(),
        component: "32_env.runway_friction".into(),
        model_field: "environment::runway_contamination::RunwayFrictionOutput.mu_effective".into(),
        magnitude: "1 - mu_effective/0.40 (GENERIC dry reference), reaching ~0.9 in full dynamic hydroplaning".into(),
        effect: "longer stopping distance, reduced directional control on the affected gear".into(),
    });

    // ---- ECAM alerts ----------------------------------------------------
    r.alert(
        EcamAlert::new("ENV_RWY_HYDROPLANING", 32, "RISK OF HYDROPLANING", Level::Advisory, var("ENV_RWY_HYDROPLANING").on())
            .confirm(0.0)
            .step(line("AUTOBRAKE MAX", "SET").colour("cyan"))
            .status_line("BRAKING ACTION ........ REDUCED")
            .raised_by(&[hydroplaning]),
    );
}

fn register_wind_shear(r: &mut Registry) {
    // Wind shear is a direct real-time hazard detection, the same way the
    // real aircraft's Predictive/Reactive Windshear System works, not a
    // component fault -- so this alert has no backing FailureDef/
    // ComponentDef and `raised_by` is left empty. Threshold matches
    // `wind_shear::F_FACTOR_HAZARD_THRESHOLD` (0.105, GENERIC).
    r.alert(
        EcamAlert::new("ENV_WINDSHEAR_WARNING", 34, "WINDSHEAR", Level::Warning, var("ENV_F_FACTOR").gt(0.105))
            .confirm(0.0)
            .aural(Aural::Named("windshear_windshear_windshear"))
            .step(line("TOGA THRUST", "APPLY").colour("cyan"))
            .step(line("PITCH", "FOLLOW FD ORDERS").colour("cyan"))
            .step(line("WINGS", "LEVEL").colour("cyan"))
            .status_line("WINDSHEAR DETECTED"),
    );
    // Turbulence (`wind_shear::TurbulenceGusts`) has no ECAM entry, see
    // this file's header.
}
