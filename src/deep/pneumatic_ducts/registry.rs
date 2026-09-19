//! Registers this directory's components, failures and ECAM alerts with the
//! shared deep-systems catalogue (`crate::deep::api`). See
//! `docs/deep/BRIEF.md`, "Registering failures, components and ECAM alerts".
//!
//! ATA assignment: 36 (Pneumatic -- bleed ducts, cross-bleed manifold,
//! precoolers, engine start, hydraulic reservoir pressurisation, the
//! overheat detection loop system) and 30 (Ice and Rain Protection -- wing
//! anti-ice duct/valve, the same chapter `docs/physics/ice-protection.md`
//! already files wing anti-ice under).
//!
//! Following the precedent `deep::sensors::registry` already set for this
//! shared catalogue: one failure id per **genuinely distinct fault
//! mechanism** on a component class, with instance count noted in the
//! component's own name/multiplicity (e.g. "x4 engines") rather than one id
//! per numbered instance -- the model itself (`network.rs`'s per-engine
//! `DuctNetworkFaults` arrays) already carries the actual per-instance
//! state; the catalogue lists *what kinds* of things can go wrong, not a
//! separate id per copy of the same wrong thing.
//!
//! ## New Vars this directory's model would need to publish
//! Nothing here is wired into the crate yet (`docs/deep/BRIEF.md` hard rule
//! 2), so, like `deep::sensors::registry`, the ECAM triggers below name the
//! Vars a future wiring pass must publish from `network.rs`'s
//! `NetworkOutputs`/`DuctNetworkFaults`: `DEEP_PNEU_ODLS_<ZONE>_TRIP`,
//! `DEEP_PNEU_ODLS_<ZONE>_FAULT` (one per `network::ZONE_NAMES` entry),
//! `DEEP_PNEU_ENG_<n>_PRECOOLER_OVHT`, `DEEP_PNEU_APU_PRECOOLER_OVHT`,
//! `DEEP_PNEU_ENG_<n>_ISOLATION_OPEN`, `DEEP_PNEU_APU_ISOLATION_OPEN`,
//! `DEEP_PNEU_MANIFOLD_PRESSURE_PA`. Also noted in `PROGRESS.md`.
//!
//! Existing cockpit controls this file's ECAM procedures *should* complete
//! from once wired (not yet confirmed by grep against a published Var
//! name -- `a380_systems/pneumatic.rs`'s `A380PneumaticOverheadPanel` does
//! have real `AutoOffFaultPushButton::new_auto(context, "PNEU_ENG_<n>_
//! BLEED")`/`OnOffFaultPushButton::new_on(context, "PNEU_APU_BLEED")`
//! push buttons, but this workstream did not verify their exact published
//! `_PB_IS_*` Var suffix under BRIEF rule 2's self-contained/read-only
//! scope): until confirmed, procedure lines below complete from this
//! module's own proposed `DEEP_PNEU_*_PB_ON`/`DEEP_PNEU_WAI_*_SELECTED`
//! Vars, which a future integration pass should point at the real button
//! state instead of duplicating it.

use crate::deep::api::*;

/// Zone names in the exact order `network::ZONE_NAMES`/`network::
/// ODLS_ZONE_COUNT` uses (module docs there): 4 pylons, wing root, APU
/// bay, 2 wing leading edges. Kept as plain strings here too (this
/// registry must not depend on `network.rs`'s internals per its own
/// self-contained-module rule; duplicated intentionally, same as every
/// other cited-but-not-imported figure in this push).
// Updated to match `network::ZONE_NAMES` exactly (the real zone names
// `deep::thermal_zones::topology_a380::build()` constructs, confirmed by
// reading that file): the shared cross-bleed manifold volume was removed
// (`network.rs` now uses FBW's own left/centre/right direct-valve topology,
// no manifold component/failures below any more -- see `register_upstream_
// stage` for what replaced ids 15-17), and "APU_BAY"/"WING_ROOT" are now
// "TailCone" (where the APU duct run actually is) and each pylon's own zone.
const ODLS_ZONES: [(&str, &str); 7] = [
    ("PylonEngine1", "PYLON 1"),
    ("PylonEngine2", "PYLON 2"),
    ("PylonEngine3", "PYLON 3"),
    ("PylonEngine4", "PYLON 4"),
    ("TailCone", "TAIL CONE"),
    ("WingLeLeft", "L WING LEADING EDGE"),
    ("WingLeRight", "R WING LEADING EDGE"),
];

pub fn register(r: &mut Registry) {
    register_engine_bleed_duct(r);
    register_engine_precooler(r);
    register_upstream_stage(r);
    register_apu_bleed_duct(r);
    register_apu_precooler(r);
    register_pack_supply_duct(r);
    register_wai_duct(r);
    register_engine_start_duct(r);
    register_hyd_reservoir_duct(r);
    register_odls(r);
    register_ecam(r);
}

/// The IP tap / HP valve / PR (shutoff) valve stage added upstream of the
/// precooler, fed directly by the engine's own published IP8/HP6 port
/// pressures (`network.rs` module docs). Reuses failure ids 15-17
/// (previously the now-removed shared cross-bleed manifold's leak/rupture/
/// insulation faults, superseded by the real left/centre/right topology).
fn register_upstream_stage(r: &mut Registry) {
    let component = "36_pneu.engine_upstream_valve_stage".to_string();
    r.component(ComponentDef {
        id: component.clone(),
        area: Area::PneumaticDucts,
        ata: 36,
        name: "Engine IP tap / HP valve / PR (shutoff) valve stage (x4 engines)".into(),
        params: vec![
            ParamDef { name: "hp_valve_stuck".into(), meaning: "HP6 valve seized at its position when the fault engaged, 0 healthy .. 1 seized".into(), healthy: 0.0 },
            ParamDef { name: "pr_valve_stuck".into(), meaning: "PR/shutoff valve (the real 'ENG n BLEED' pushbutton's own valve) seized, 0 healthy .. 1 seized".into(), healthy: 0.0 },
            ParamDef { name: "ip_check_valve_stuck_closed".into(), meaning: "IP8 passive tap stuck toward closed, forcing reliance on the HP valve, 0 healthy .. 1 fully shut".into(), healthy: 0.0 },
        ],
        failures: vec![failure_id(Area::PneumaticDucts, 36, 15), failure_id(Area::PneumaticDucts, 36, 16), failure_id(Area::PneumaticDucts, 36, 17)],
    });
    r.failure(FailureDef {
        id: failure_id(Area::PneumaticDucts, 36, 15),
        area: Area::PneumaticDucts,
        ata: 36,
        name: "Engine HP valve stuck".into(),
        component: component.clone(),
        model_field: "network::UpstreamFaults.hp_valve_stuck (network::DuctNetworkFaults.upstream[n])".into(),
        magnitude: "0 healthy .. 1 seized at whatever position it last held".into(),
        effect: "Stuck shut: no HP6 backup once IP8 falls below the EASA switch-over pressure, engine duct pressure sags. Stuck open: continues drawing hot HP6 air even once IP8 alone would suffice, an avoidable EGT/performance penalty upstream at the compressor.".into(),
    });
    r.failure(FailureDef {
        id: failure_id(Area::PneumaticDucts, 36, 16),
        area: Area::PneumaticDucts,
        ata: 36,
        name: "Engine PR (shutoff) valve stuck".into(),
        component: component.clone(),
        model_field: "network::UpstreamFaults.pr_valve_stuck (network::DuctNetworkFaults.upstream[n])".into(),
        magnitude: "0 healthy .. 1 seized at whatever position it last held".into(),
        effect: "Stuck shut: that engine can no longer supply its own duct at all (a real 'ENG n BLEED FAULT'), leaving it dependent on cross-bleed. Stuck open: an ODLS trip's own isolation command can no longer close it, defeating the real isolation this system exists for.".into(),
    });
    r.failure(FailureDef {
        id: failure_id(Area::PneumaticDucts, 36, 17),
        area: Area::PneumaticDucts,
        ata: 36,
        name: "Engine IP8 check valve stuck closed".into(),
        component,
        model_field: "network::UpstreamFaults.ip_check_valve_stuck_closed (network::DuctNetworkFaults.upstream[n])".into(),
        magnitude: "0 healthy .. 1 fully stuck shut".into(),
        effect: "The passive IP8 tap can no longer pass its normal share of flow, forcing the HP valve to open further/more often than normal to hold regulation -- a real, measurable shift from the efficient IP8 source to the costlier HP6 one.".into(),
    });
}

fn register_engine_bleed_duct(r: &mut Registry) {
    let component = "36_pneu.engine_bleed_duct".to_string();
    r.component(ComponentDef {
        id: component.clone(),
        area: Area::PneumaticDucts,
        ata: 36,
        name: "Engine bleed duct, pylon run (x4 engines)".into(),
        params: vec![
            ParamDef { name: "leak".into(), meaning: "Crack/seal failure, orifice area 0..2% of the duct's own bore".into(), healthy: 0.0 },
            ParamDef { name: "rupture".into(), meaning: "Duct severance, orifice area 0..100% of full bore".into(), healthy: 0.0 },
            ParamDef { name: "insulation_damage".into(), meaning: "Lagging blanket torn/missing, conductance to the pylon zone rises up to 10x".into(), healthy: 0.0 },
        ],
        failures: vec![failure_id(Area::PneumaticDucts, 36, 1), failure_id(Area::PneumaticDucts, 36, 2), failure_id(Area::PneumaticDucts, 36, 3)],
    });
    r.failure(FailureDef {
        id: failure_id(Area::PneumaticDucts, 36, 1),
        area: Area::PneumaticDucts,
        ata: 36,
        name: "Engine bleed duct leak".into(),
        component: component.clone(),
        model_field: "duct::DuctSectionFaults.leak (network::DuctNetworkFaults.engine_duct[n])".into(),
        magnitude: "Crack area, 0 healthy .. 1 = 2% of the duct's bore area (leak.rs::LEAK_AREA_FRACTION_OF_BORE)".into(),
        effect: "Mass flow escapes to the pylon zone (leak.rs::step); manifold/downstream pressure sags proportionally, and the pylon ODLS eventually trips on the resulting heat if severe enough".into(),
    });
    r.failure(FailureDef {
        id: failure_id(Area::PneumaticDucts, 36, 2),
        area: Area::PneumaticDucts,
        ata: 36,
        name: "Engine bleed duct rupture".into(),
        component: component.clone(),
        model_field: "duct::DuctSectionFaults.rupture (network::DuctNetworkFaults.engine_duct[n])".into(),
        magnitude: "Severance area, 0 healthy .. 1 = full duct bore area".into(),
        effect: "Large mass flow escapes as a near-sonic jet; above leak.rs::RUPTURE_JET_ONSET the jet is treated as impinging (higher heat-transfer effectiveness into the pylon zone), reliably tripping that pylon's ODLS and, once confirmed, latching that engine's isolation valve shut (network.rs)".into(),
    });
    r.failure(FailureDef {
        id: failure_id(Area::PneumaticDucts, 36, 3),
        area: Area::PneumaticDucts,
        ata: 36,
        name: "Engine bleed duct insulation damage".into(),
        component,
        model_field: "duct::DuctSectionFaults.insulation_damage (network::DuctNetworkFaults.engine_duct[n])".into(),
        magnitude: "Lagging condition, 0 intact .. 1 fully bare pipe (duct.rs::DuctSection::BARE_PIPE_MULTIPLIER = 10x conductance)".into(),
        effect: "The pylon zone receives ordinary (non-fault) duct heat loss at up to 10x the healthy rate -- no ODLS trip on its own at normal bleed temperatures, but raises the pylon's baseline temperature so a subsequent leak trips ODLS sooner".into(),
    });
}

fn register_engine_precooler(r: &mut Registry) {
    let component = "36_pneu.engine_precooler".to_string();
    r.component(ComponentDef {
        id: component.clone(),
        area: Area::PneumaticDucts,
        ata: 36,
        name: "Engine bleed precooler (x4 engines)".into(),
        params: vec![
            ParamDef { name: "fouling".into(), meaning: "Core scaling/debris, conductance loss 0 clean .. 1 = 80% conductance lost".into(), healthy: 0.0 },
            ParamDef { name: "fan_air_valve_stuck".into(), meaning: "FAV seized at whatever position it last held, 0 healthy .. 1 seized".into(), healthy: 0.0 },
            ParamDef { name: "temp_sensor_fault".into(), meaning: "Modulating outlet-temperature sensor frozen at its last good reading, 0 healthy .. 1 frozen".into(), healthy: 0.0 },
            ParamDef { name: "check_valve_failure".into(), meaning: "Non-return valve leaks ambient air backward into the duct, 0 healthy .. 1 fully open in reverse".into(), healthy: 0.0 },
        ],
        failures: vec![
            failure_id(Area::PneumaticDucts, 36, 4),
            failure_id(Area::PneumaticDucts, 36, 5),
            failure_id(Area::PneumaticDucts, 36, 6),
            failure_id(Area::PneumaticDucts, 36, 7),
        ],
    });
    r.failure(FailureDef {
        id: failure_id(Area::PneumaticDucts, 36, 4),
        area: Area::PneumaticDucts,
        ata: 36,
        name: "Engine precooler fouling".into(),
        component: component.clone(),
        model_field: "precooler::PrecoolerFaults.fouling (network::DuctNetworkFaults.engine_precooler[n])".into(),
        magnitude: "Conductance loss fraction, 0 clean .. 1 (precooler.rs::FOULING_MAX_REDUCTION = 80% loss at 1.0)".into(),
        effect: "For the same cooling flow the outlet runs hotter (precooler.rs's NTU-effectiveness heat exchange); the FAV opens further to compensate, and at full fouling may not hold the outlet near target at all, raising overtemperature-trip risk".into(),
    });
    r.failure(FailureDef {
        id: failure_id(Area::PneumaticDucts, 36, 5),
        area: Area::PneumaticDucts,
        ata: 36,
        name: "Engine precooler fan air valve stuck".into(),
        component: component.clone(),
        model_field: "precooler::PrecoolerFaults.fan_air_valve_stuck (network::DuctNetworkFaults.engine_precooler[n])".into(),
        magnitude: "0 healthy .. 1 seized at its position when the fault engaged".into(),
        effect: "No further cooling-flow modulation: stuck closed (or partly open) leaves the bleed hot regardless of demand; the hard overtemperature protection (true-temperature-based, module docs) is unaffected by this fault and can still force full flow via a separate path -- but a valve seized fully open cannot heed it either".into(),
    });
    r.failure(FailureDef {
        id: failure_id(Area::PneumaticDucts, 36, 6),
        area: Area::PneumaticDucts,
        ata: 36,
        name: "Engine precooler outlet temperature sensor fault".into(),
        component: component.clone(),
        model_field: "precooler::PrecoolerFaults.temp_sensor_fault (network::DuctNetworkFaults.engine_precooler[n])".into(),
        magnitude: "0 healthy .. 1 frozen at the last good reading".into(),
        effect: "The modulating FAV loop under/over-corrects against a stale reading; the independent hard overtemperature trip still sees the true temperature (precooler.rs module docs: a biased sensor cannot defeat it), so this fault degrades regulation quality without defeating the safety protection".into(),
    });
    r.failure(FailureDef {
        id: failure_id(Area::PneumaticDucts, 36, 7),
        area: Area::PneumaticDucts,
        ata: 36,
        name: "Engine precooler check valve failure".into(),
        component,
        model_field: "precooler::PrecoolerFaults.check_valve_failure (network::DuctNetworkFaults.engine_precooler[n])".into(),
        magnitude: "0 healthy .. 1 fully open in reverse".into(),
        effect: "When duct pressure sags below ambient (e.g. low power, other faults), outside air is drawn back into the duct uncontrolled instead of being blocked, cooling/diluting it unpredictably".into(),
    });
}

fn register_apu_bleed_duct(r: &mut Registry) {
    let component = "36_pneu.apu_bleed_duct".to_string();
    r.component(ComponentDef {
        id: component.clone(),
        area: Area::PneumaticDucts,
        ata: 36,
        name: "APU bleed duct (x1)".into(),
        params: vec![
            ParamDef { name: "leak".into(), meaning: "Same mechanism as the engine bleed duct".into(), healthy: 0.0 },
            ParamDef { name: "rupture".into(), meaning: "Same mechanism as the engine bleed duct".into(), healthy: 0.0 },
            ParamDef { name: "insulation_damage".into(), meaning: "Same mechanism as the engine bleed duct".into(), healthy: 0.0 },
        ],
        failures: vec![failure_id(Area::PneumaticDucts, 36, 8), failure_id(Area::PneumaticDucts, 36, 9), failure_id(Area::PneumaticDucts, 36, 10)],
    });
    r.failure(FailureDef { id: failure_id(Area::PneumaticDucts, 36, 8), area: Area::PneumaticDucts, ata: 36, name: "APU bleed duct leak".into(), component: component.clone(), model_field: "network::DuctNetworkFaults.apu_duct.leak".into(), magnitude: "Same as fault 1, applied to the APU bay zone".into(), effect: "Same as fault 1; heats the APU bay instead of a pylon".into() });
    r.failure(FailureDef { id: failure_id(Area::PneumaticDucts, 36, 9), area: Area::PneumaticDucts, ata: 36, name: "APU bleed duct rupture".into(), component: component.clone(), model_field: "network::DuctNetworkFaults.apu_duct.rupture".into(), magnitude: "Same as fault 2, applied to the APU bay zone".into(), effect: "Same as fault 2; can trip the APU bay ODLS and latch the APU isolation valve shut".into() });
    r.failure(FailureDef { id: failure_id(Area::PneumaticDucts, 36, 10), area: Area::PneumaticDucts, ata: 36, name: "APU bleed duct insulation damage".into(), component, model_field: "network::DuctNetworkFaults.apu_duct.insulation_damage".into(), magnitude: "Same as fault 3".into(), effect: "Same as fault 3, applied to the APU bay zone".into() });
}

fn register_apu_precooler(r: &mut Registry) {
    let component = "36_pneu.apu_precooler".to_string();
    r.component(ComponentDef {
        id: component.clone(),
        area: Area::PneumaticDucts,
        ata: 36,
        name: "APU load-compressor precooler (x1)".into(),
        params: vec![
            ParamDef { name: "fouling".into(), meaning: "Same mechanism as the engine precooler".into(), healthy: 0.0 },
            ParamDef { name: "fan_air_valve_stuck".into(), meaning: "Same mechanism as the engine precooler".into(), healthy: 0.0 },
            ParamDef { name: "temp_sensor_fault".into(), meaning: "Same mechanism as the engine precooler".into(), healthy: 0.0 },
            ParamDef { name: "check_valve_failure".into(), meaning: "Same mechanism as the engine precooler".into(), healthy: 0.0 },
        ],
        failures: vec![
            failure_id(Area::PneumaticDucts, 36, 11),
            failure_id(Area::PneumaticDucts, 36, 12),
            failure_id(Area::PneumaticDucts, 36, 13),
            failure_id(Area::PneumaticDucts, 36, 14),
        ],
    });
    r.failure(FailureDef { id: failure_id(Area::PneumaticDucts, 36, 11), area: Area::PneumaticDucts, ata: 36, name: "APU precooler fouling".into(), component: component.clone(), model_field: "network::DuctNetworkFaults.apu_precooler.fouling".into(), magnitude: "Same as fault 4".into(), effect: "Same as fault 4".into() });
    r.failure(FailureDef { id: failure_id(Area::PneumaticDucts, 36, 12), area: Area::PneumaticDucts, ata: 36, name: "APU precooler fan air valve stuck".into(), component: component.clone(), model_field: "network::DuctNetworkFaults.apu_precooler.fan_air_valve_stuck".into(), magnitude: "Same as fault 5".into(), effect: "Same as fault 5".into() });
    r.failure(FailureDef { id: failure_id(Area::PneumaticDucts, 36, 13), area: Area::PneumaticDucts, ata: 36, name: "APU precooler outlet temperature sensor fault".into(), component: component.clone(), model_field: "network::DuctNetworkFaults.apu_precooler.temp_sensor_fault".into(), magnitude: "Same as fault 6".into(), effect: "Same as fault 6".into() });
    r.failure(FailureDef { id: failure_id(Area::PneumaticDucts, 36, 14), area: Area::PneumaticDucts, ata: 36, name: "APU precooler check valve failure".into(), component, model_field: "network::DuctNetworkFaults.apu_precooler.check_valve_failure".into(), magnitude: "Same as fault 7".into(), effect: "Same as fault 7".into() });
}

// (The shared cross-bleed manifold component/failures that used to live
// here were removed: `network.rs` now uses FBW's own real left/centre/
// right direct-valve topology with no manifold volume at all. Failure ids
// 15-17 were reused for `register_upstream_stage`'s HP/PR/IP-check-valve
// faults above, a genuinely new and distinct physical fault set, not a
// renaming.)

fn register_pack_supply_duct(r: &mut Registry) {
    let component = "36_pneu.pack_supply_duct".to_string();
    r.component(ComponentDef {
        id: component.clone(),
        area: Area::PneumaticDucts,
        ata: 36,
        name: "Pack supply duct (x2 packs)".into(),
        params: vec![
            ParamDef { name: "leak".into(), meaning: "Same mechanism as the engine bleed duct".into(), healthy: 0.0 },
            ParamDef { name: "rupture".into(), meaning: "Same mechanism as the engine bleed duct".into(), healthy: 0.0 },
            ParamDef { name: "insulation_damage".into(), meaning: "Same mechanism as the engine bleed duct".into(), healthy: 0.0 },
        ],
        failures: vec![failure_id(Area::PneumaticDucts, 36, 18), failure_id(Area::PneumaticDucts, 36, 19), failure_id(Area::PneumaticDucts, 36, 20)],
    });
    r.failure(FailureDef { id: failure_id(Area::PneumaticDucts, 36, 18), area: Area::PneumaticDucts, ata: 36, name: "Pack supply duct leak".into(), component: component.clone(), model_field: "network::DuctNetworkFaults.packs[n].leak".into(), magnitude: "Same as fault 1".into(), effect: "Starves that pack of supply pressure; the manifold and other consumers are largely unaffected (downstream of the pack valve)".into() });
    r.failure(FailureDef { id: failure_id(Area::PneumaticDucts, 36, 19), area: Area::PneumaticDucts, ata: 36, name: "Pack supply duct rupture".into(), component: component.clone(), model_field: "network::DuctNetworkFaults.packs[n].rupture".into(), magnitude: "Same as fault 2".into(), effect: "Same as fault 18, far more severely".into() });
    r.failure(FailureDef { id: failure_id(Area::PneumaticDucts, 36, 20), area: Area::PneumaticDucts, ata: 36, name: "Pack supply duct insulation damage".into(), component, model_field: "network::DuctNetworkFaults.packs[n].insulation_damage".into(), magnitude: "Same as fault 3".into(), effect: "Same as fault 3, applied to the wing-root zone".into() });
}

fn register_wai_duct(r: &mut Registry) {
    let component = "30_pneu.wing_anti_ice_duct".to_string();
    r.component(ComponentDef {
        id: component.clone(),
        area: Area::PneumaticDucts,
        ata: 30,
        name: "Wing anti-ice duct (x2, L/R leading edge)".into(),
        params: vec![
            ParamDef { name: "leak".into(), meaning: "Same mechanism as the engine bleed duct".into(), healthy: 0.0 },
            ParamDef { name: "rupture".into(), meaning: "Same mechanism as the engine bleed duct".into(), healthy: 0.0 },
            ParamDef { name: "insulation_damage".into(), meaning: "Same mechanism as the engine bleed duct".into(), healthy: 0.0 },
        ],
        failures: vec![failure_id(Area::PneumaticDucts, 30, 1), failure_id(Area::PneumaticDucts, 30, 2), failure_id(Area::PneumaticDucts, 30, 3)],
    });
    r.failure(FailureDef { id: failure_id(Area::PneumaticDucts, 30, 1), area: Area::PneumaticDucts, ata: 30, name: "Wing anti-ice duct leak".into(), component: component.clone(), model_field: "network::DuctNetworkFaults.wai[n].leak".into(), magnitude: "Same as fault 1".into(), effect: "Heats that wing's leading-edge zone directly (real risk: an undetected WAI duct leak burning through the leading-edge structure, the well-publicised reason this class of system carries ODLS at all)".into() });
    r.failure(FailureDef { id: failure_id(Area::PneumaticDucts, 30, 2), area: Area::PneumaticDucts, ata: 30, name: "Wing anti-ice duct rupture".into(), component: component.clone(), model_field: "network::DuctNetworkFaults.wai[n].rupture".into(), magnitude: "Same as fault 2".into(), effect: "Same as fault 1, far more severely; reliably trips that side's leading-edge ODLS".into() });
    r.failure(FailureDef { id: failure_id(Area::PneumaticDucts, 30, 3), area: Area::PneumaticDucts, ata: 30, name: "Wing anti-ice duct insulation damage".into(), component, model_field: "network::DuctNetworkFaults.wai[n].insulation_damage".into(), magnitude: "Same as fault 3".into(), effect: "Same as fault 3".into() });
}

fn register_engine_start_duct(r: &mut Registry) {
    let component = "36_pneu.engine_start_duct".to_string();
    r.component(ComponentDef {
        id: component.clone(),
        area: Area::PneumaticDucts,
        ata: 36,
        name: "Engine pneumatic starter duct (x4 engines)".into(),
        params: vec![
            ParamDef { name: "leak".into(), meaning: "Same mechanism as the engine bleed duct".into(), healthy: 0.0 },
            ParamDef { name: "rupture".into(), meaning: "Same mechanism as the engine bleed duct".into(), healthy: 0.0 },
            ParamDef { name: "insulation_damage".into(), meaning: "Same mechanism as the engine bleed duct".into(), healthy: 0.0 },
            ParamDef { name: "check_valve_failure".into(), meaning: "Non-return valve leaks the lit engine's own rising pressure backward into the manifold, 0 healthy .. 1 fully open in reverse".into(), healthy: 0.0 },
        ],
        failures: vec![
            failure_id(Area::PneumaticDucts, 36, 21),
            failure_id(Area::PneumaticDucts, 36, 22),
            failure_id(Area::PneumaticDucts, 36, 23),
            failure_id(Area::PneumaticDucts, 36, 24),
        ],
    });
    r.failure(FailureDef { id: failure_id(Area::PneumaticDucts, 36, 21), area: Area::PneumaticDucts, ata: 36, name: "Engine start duct leak".into(), component: component.clone(), model_field: "network::DuctNetworkFaults.start[n].leak".into(), magnitude: "Same as fault 1".into(), effect: "Weakens starter torque available at the engine during a pneumatic start; heats that engine's own pylon zone".into() });
    r.failure(FailureDef { id: failure_id(Area::PneumaticDucts, 36, 22), area: Area::PneumaticDucts, ata: 36, name: "Engine start duct rupture".into(), component: component.clone(), model_field: "network::DuctNetworkFaults.start[n].rupture".into(), magnitude: "Same as fault 2".into(), effect: "Same as fault 21, far more severely".into() });
    r.failure(FailureDef { id: failure_id(Area::PneumaticDucts, 36, 23), area: Area::PneumaticDucts, ata: 36, name: "Engine start duct insulation damage".into(), component: component.clone(), model_field: "network::DuctNetworkFaults.start[n].insulation_damage".into(), magnitude: "Same as fault 3".into(), effect: "Same as fault 3".into() });
    r.failure(FailureDef {
        id: failure_id(Area::PneumaticDucts, 36, 24),
        area: Area::PneumaticDucts,
        ata: 36,
        name: "Engine start duct check valve failure".into(),
        component,
        model_field: "network::DuctNetworkFaults.start_check_valve_failure[n] (duct::one_way_transfer_kg's backflow_leak_fraction)".into(),
        magnitude: "0 healthy (perfect non-return) .. 1 fully open in reverse".into(),
        effect: "Once that engine lights and its own pressure exceeds the manifold's, a failed check valve lets it push pressure/heat backward into the manifold instead of being blocked -- disturbs every other consumer sharing the manifold during that engine's start".into(),
    });
}

fn register_hyd_reservoir_duct(r: &mut Registry) {
    let component = "36_pneu.hydraulic_reservoir_pressurisation_duct".to_string();
    r.component(ComponentDef {
        id: component.clone(),
        area: Area::PneumaticDucts,
        ata: 36,
        name: "Hydraulic reservoir pressurisation duct (x2, green/yellow)".into(),
        params: vec![
            ParamDef { name: "leak".into(), meaning: "Same mechanism as the engine bleed duct".into(), healthy: 0.0 },
            ParamDef { name: "rupture".into(), meaning: "Same mechanism as the engine bleed duct".into(), healthy: 0.0 },
            ParamDef { name: "insulation_damage".into(), meaning: "Same mechanism as the engine bleed duct".into(), healthy: 0.0 },
        ],
        failures: vec![failure_id(Area::PneumaticDucts, 36, 25), failure_id(Area::PneumaticDucts, 36, 26), failure_id(Area::PneumaticDucts, 36, 27)],
    });
    r.failure(FailureDef { id: failure_id(Area::PneumaticDucts, 36, 25), area: Area::PneumaticDucts, ata: 36, name: "Hydraulic reservoir pressurisation duct leak".into(), component: component.clone(), model_field: "network::DuctNetworkFaults.hyd_reservoir[n].leak".into(), magnitude: "Same as fault 1".into(), effect: "Reduces reservoir air pressurisation, raising cavitation risk for that hydraulic system's pumps at high demand".into() });
    r.failure(FailureDef { id: failure_id(Area::PneumaticDucts, 36, 26), area: Area::PneumaticDucts, ata: 36, name: "Hydraulic reservoir pressurisation duct rupture".into(), component: component.clone(), model_field: "network::DuctNetworkFaults.hyd_reservoir[n].rupture".into(), magnitude: "Same as fault 2".into(), effect: "Same as fault 25, far more severely".into() });
    r.failure(FailureDef { id: failure_id(Area::PneumaticDucts, 36, 27), area: Area::PneumaticDucts, ata: 36, name: "Hydraulic reservoir pressurisation duct insulation damage".into(), component, model_field: "network::DuctNetworkFaults.hyd_reservoir[n].insulation_damage".into(), magnitude: "Same as fault 3".into(), effect: "Same as fault 3".into() });
}

fn register_odls(r: &mut Registry) {
    let component = "36_pneu.overheat_detection_loop".to_string();
    r.component(ComponentDef {
        id: component.clone(),
        area: Area::PneumaticDucts,
        ata: 36,
        name: "Overheat Detection Loop System, dual-loop (x8 zones: 4 pylons, wing root, APU bay, 2 wing leading edges)".into(),
        params: vec![
            ParamDef { name: "loop_a_open".into(), meaning: "Loop A element circuit broken: no valid reading from A, 0 healthy .. 1 fully open".into(), healthy: 0.0 },
            ParamDef { name: "loop_a_short".into(), meaning: "Loop A element shorted: A pegs at a fixed high (false-hot) reading, 0 healthy .. 1 fully shorted".into(), healthy: 0.0 },
            ParamDef { name: "loop_b_open".into(), meaning: "Same as loop_a_open, loop B".into(), healthy: 0.0 },
            ParamDef { name: "loop_b_short".into(), meaning: "Same as loop_a_short, loop B".into(), healthy: 0.0 },
            ParamDef { name: "false_detection".into(), meaning: "System-level spurious trip not tied to either loop's own wiring, 0 healthy .. 1 trips a cold zone on its own".into(), healthy: 0.0 },
        ],
        failures: vec![
            failure_id(Area::PneumaticDucts, 36, 28),
            failure_id(Area::PneumaticDucts, 36, 29),
            failure_id(Area::PneumaticDucts, 36, 30),
            failure_id(Area::PneumaticDucts, 36, 31),
            failure_id(Area::PneumaticDucts, 36, 32),
        ],
    });
    r.failure(FailureDef { id: failure_id(Area::PneumaticDucts, 36, 28), area: Area::PneumaticDucts, ata: 36, name: "ODLS loop A open circuit".into(), component: component.clone(), model_field: "odls::OdlsFaults.loop_a_open (network::DuctNetworkFaults.odls[zone])".into(), magnitude: "0 healthy .. 1 open (>= 0.5 treated as fully open, odls.rs::interpret)".into(), effect: "Loop A stops providing a valid reading; loop B alone still governs trip/no-trip (fail-safe voting, odls.rs). Both loops open at once reports a FAULT with no valid detection at all".into() });
    r.failure(FailureDef { id: failure_id(Area::PneumaticDucts, 36, 29), area: Area::PneumaticDucts, ata: 36, name: "ODLS loop A short circuit".into(), component: component.clone(), model_field: "odls::OdlsFaults.loop_a_short (network::DuctNetworkFaults.odls[zone])".into(), magnitude: "0 healthy .. 1 shorted (>= 0.5 treated as fully shorted)".into(), effect: "Loop A pegs at a fixed reading well past the trip threshold, tripping that zone's isolation on its own (a real false alarm from a genuine wiring fault, distinct from fault 32)".into() });
    r.failure(FailureDef { id: failure_id(Area::PneumaticDucts, 36, 30), area: Area::PneumaticDucts, ata: 36, name: "ODLS loop B open circuit".into(), component: component.clone(), model_field: "odls::OdlsFaults.loop_b_open (network::DuctNetworkFaults.odls[zone])".into(), magnitude: "Same as fault 28, loop B".into(), effect: "Same as fault 28, loop B".into() });
    r.failure(FailureDef { id: failure_id(Area::PneumaticDucts, 36, 31), area: Area::PneumaticDucts, ata: 36, name: "ODLS loop B short circuit".into(), component: component.clone(), model_field: "odls::OdlsFaults.loop_b_short (network::DuctNetworkFaults.odls[zone])".into(), magnitude: "Same as fault 29, loop B".into(), effect: "Same as fault 29, loop B".into() });
    r.failure(FailureDef {
        id: failure_id(Area::PneumaticDucts, 36, 32),
        area: Area::PneumaticDucts,
        ata: 36,
        name: "ODLS false detection".into(),
        component,
        model_field: "odls::OdlsFaults.false_detection (network::DuctNetworkFaults.odls[zone])".into(),
        magnitude: "0 healthy .. 1 = a cold zone trips on its own regardless of real temperature (odls.rs::FALSE_DETECTION_FULL_K)".into(),
        effect: "Trips and latches that zone's isolation valve shut with no real overheat present -- a nuisance trip, not a wiring fault on either specific loop (distinct from faults 29/31)".into(),
    });
}

fn register_ecam(r: &mut Registry) {
    // One bleed-leak alert per pylon, triggered by the real ODLS trip
    // output (`network::NetworkOutputs::odls_trip[PYLON[n]]`, module docs:
    // publish as `DEEP_PNEU_ODLS_PYLON_<n>_TRIP`).
    for (n, zone) in [1, 2, 3, 4].into_iter().zip(["PylonEngine1", "PylonEngine2", "PylonEngine3", "PylonEngine4"]) {
        r.alert(
            EcamAlert::new(&format!("DEEP_PNEU_ENG_{n}_BLEED_LEAK"), 36, &format!("AIR ENG {n} BLEED LEAK"), Level::Warning, var(&format!("DEEP_PNEU_ODLS_{zone}_TRIP")).on())
                .confirm(1.0) // odls.rs's own CONFIRM_TIME_S already debounces; this is the ECAM's own display-side confirm
                .inhibit(&[Phase::LiftOff, Phase::Below800Ft])
                .step(line(&format!("ENG {n} BLEED"), "OFF").done(var(&format!("DEEP_PNEU_ENG_{n}_BLEED_PB_ON")).off()))
                .step(line("APU BLEED", "AS RQRD").colour("white").only_if(Cond::Always))
                .status_line(&format!("AIR ENG {n} BLEED LEAK"))
                .inop_sys(&format!("PACK {} DEGRADED", if n <= 2 { 1 } else { 2 }))
                .raised_by(&[failure_id(Area::PneumaticDucts, 36, 1), failure_id(Area::PneumaticDucts, 36, 2)]),
        );
    }

    r.alert(
        EcamAlert::new("DEEP_PNEU_APU_BLEED_LEAK", 36, "AIR APU BLEED LEAK", Level::Warning, var("DEEP_PNEU_ODLS_TailCone_TRIP").on())
            .confirm(1.0)
            .step(line("APU BLEED", "OFF").done(var("DEEP_PNEU_APU_BLEED_PB_ON").off()))
            .status_line("AIR APU BLEED LEAK")
            .raised_by(&[failure_id(Area::PneumaticDucts, 36, 8), failure_id(Area::PneumaticDucts, 36, 9)]),
    );

    for (side, zone) in [("L", "WingLeLeft"), ("R", "WingLeRight")] {
        r.alert(
            EcamAlert::new(&format!("DEEP_PNEU_WING_{side}_LEAK"), 30, &format!("AIR WING {side} DUCT LEAK"), Level::Warning, var(&format!("DEEP_PNEU_ODLS_{zone}_TRIP")).on())
                .confirm(1.0)
                .inhibit(&[Phase::LiftOff, Phase::Below800Ft])
                .step(line(&format!("WING A ICE {side}"), "OFF").done(var(&format!("DEEP_PNEU_WAI_{side}_SELECTED")).off()))
                .status_line(&format!("AIR WING {side} DUCT LEAK"))
                .inop_sys(&format!("WING A ICE {side} SYS"))
                .raised_by(&[failure_id(Area::PneumaticDucts, 30, 1), failure_id(Area::PneumaticDucts, 30, 2)]),
        );
    }

    for n in 1..=4u16 {
        r.alert(
            EcamAlert::new(&format!("DEEP_PNEU_ENG_{n}_PRECOOLER_OVHT"), 36, &format!("AIR ENG {n} PRECOOLER OVHT"), Level::Caution, var(&format!("DEEP_PNEU_ENG_{n}_PRECOOLER_OVHT")).on())
                .confirm(2.0)
                .step(line(&format!("ENG {n} BLEED"), "OFF").done(var(&format!("DEEP_PNEU_ENG_{n}_BLEED_PB_ON")).off()))
                .status_line(&format!("AIR ENG {n} PRECOOLER"))
                .raised_by(&[failure_id(Area::PneumaticDucts, 36, 4), failure_id(Area::PneumaticDucts, 36, 5), failure_id(Area::PneumaticDucts, 36, 6)]),
        );
    }

    // Aggregate ODLS-fault (no valid detection anywhere in a zone) advisory
    // -- a maintenance-relevant condition, not itself a leak.
    r.alert(
        EcamAlert::new(
            "DEEP_PNEU_ODLS_FAULT",
            36,
            "AIR BLEED LEAK DET FAULT",
            Level::Advisory,
            any(ODLS_ZONES.iter().map(|&(zone, _)| var(&format!("DEEP_PNEU_ODLS_{zone}_FAULT")).on()).collect()),
        )
        .confirm(5.0)
        .status_line("AIR BLEED LEAK DET FAULT")
        .raised_by(&[
            failure_id(Area::PneumaticDucts, 36, 28),
            failure_id(Area::PneumaticDucts, 36, 30),
        ]),
    );
}
