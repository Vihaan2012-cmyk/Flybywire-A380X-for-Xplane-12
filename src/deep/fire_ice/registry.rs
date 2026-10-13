//! Registers every failure, component and ECAM alert this directory's
//! models support, through the shared `crate::deep::api::Registry`
//! (`docs/deep/BRIEF.md` "Registering failures, components and ECAM
//! alerts"). This is the one file in `fire_ice` allowed to depend on
//! crate internals (the registration API itself); every other module
//! here stays self-contained (BRIEF rule 2).
//!
//! ## New Vars this registry's ECAM triggers assume
//! `fire_ice` is wired into the plugin's main update loop now (this doc
//! predates that). It once named its variables following FlyByWire's own
//! style as "naming precedent, not a dependency" -- true only while
//! nothing published under those names yet. Two of them (`FIRE_DETECTED_
//! APU`/`FIRE_DETECTED_MLG`) turned out to be identical to `a380_systems`'s
//! own live variables of the same name, not just similarly styled, and
//! since `deep.tick` runs after FlyByWire's own `simulation.tick`, this
//! crate was silently overwriting FlyByWire's own fire detection every
//! frame (W162). Every `FIRE_DETECTED_*`/`FIRE_SQUIB_*_IS_DISCHARGED` name
//! below is now `DEEP_`-prefixed for exactly this reason; the rest
//! (`ANTI_ICE_WING_VALVE_OPEN:n` etc.) were checked and do not collide
//! with any FlyByWire variable, so they keep their original names.
//! Listed in full in `PROGRESS.md`.

use crate::deep::api::*;

const ATA_FIRE_PROTECTION: u16 = 26;
const ATA_ICE_RAIN: u16 = 30;

/// Zone short names matching `fire_loops::ZONES`'s order, for building
/// component/failure/Var ids consistently across this file.
const ZONES: [&str; 9] = ["eng1", "eng2", "eng3", "eng4", "apu", "mlg", "cargo_fwd", "cargo_aft", "avionics"];
/// The same zones' display names, matching how the real ECAM would title
/// them.
const ZONE_TITLES: [&str; 9] = ["ENG 1", "ENG 2", "ENG 3", "ENG 4", "APU", "MLG BAY", "CARGO FWD", "CARGO AFT", "AVIONICS"];
/// Per-zone maximum modelled flammable-fluid leak rate feeding
/// `combustion::ZoneSupply.fuel_available_kg_s` at failure magnitude 1.0,
/// kg/s. **GENERIC**: no published per-zone leak-rate figure exists;
/// engine/APU zones (fuel/oil manifolds nearby) are sized an order of
/// magnitude above the smaller bays, consistent with `physics::bays.rs`'s
/// own `LEAK_AREA_MAX_M2` sizing philosophy (small enough not to
/// instantly saturate, large enough to matter).
const ZONE_MAX_LEAK_KG_S: [f64; 9] = [0.05, 0.05, 0.05, 0.05, 0.03, 0.01, 0.02, 0.02, 0.005];

pub fn register(r: &mut Registry) {
    register_fire_detection_loops(r);
    register_combustion_zones(r);
    register_extinguishing(r);
    register_anti_ice(r);
    register_ecam(r);
}

// ---------------------------------------------------------------------------
// Backlog item 1: dual-loop fire/overheat detection (fire_loops.rs)
// ---------------------------------------------------------------------------

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

// ---------------------------------------------------------------------------
// Backlog item 2: per-zone combustion fire sources (combustion.rs)
// ---------------------------------------------------------------------------

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

// ---------------------------------------------------------------------------
// Backlog item 3: extinguishing (bottles, squibs, smoke detection)
// ---------------------------------------------------------------------------

fn register_extinguishing(r: &mut Registry) {
    let mut n = 200u16;

    // Two Halon bottles per engine (FBW's own fire_and_smoke_protection.rs
    // uses the same "two bottles per engine" split, read as precedent).
    for eng in 1..=4u16 {
        for bottle in 1..=2u16 {
            n += 1;
            register_bottle(r, &mut n, &format!("26_fire.eng{eng}_bottle{bottle}"), &format!("ENG {eng} fire bottle {bottle}"), None);
        }
    }
    n += 1;
    register_bottle(r, &mut n, "26_fire.apu_bottle1", "APU fire bottle", None);
    // (W194) The cargo bottles now have a real command: `live.rs`'s
    // `FireIceLive::step_bottles` passes `truth.controls.
    // cargo_agent_pb_pressed[b]`, sourced from the real PUSH_OVHD_
    // CARGOSMOKE_FWD/_AFT overhead pushbuttons (FlyByWire's own tooltip for
    // them says "(Inop.)" -- no FBW system consumes the name, but this
    // port's own bottle model already existed and only lacked the
    // command).
    //
    // (E-FIRE §F, ECAM completeness pass) Each hold's single `squib_
    // failure` is split into `knockdown_squib_fault`/`extended_squib_
    // fault` -- the general two-bottle cargo fire-suppression
    // architecture (US Patent 9,248,326/8,925,642) has bottle 1 discharge
    // rapidly for initial knockdown and bottle 2 be timed to discharge
    // later for extended suppression, exactly the two-stage shape
    // `CargoSuppressionSystem::step` already implements internally --
    // matching real hardware (two bottles, two squibs) rather than one. A
    // new `distribution_fault` (the line/valve between the bottle
    // manifold and the hold, independent of either bottle) is added
    // alongside it.
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

    // Cargo optical smoke detectors.
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

    // Lavatory fusible-link extinguisher (multiplicity: xN lavatories,
    // GENERIC count -- this crate does not yet fix the A380's actual
    // lavatory count/layout).
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

    // ECAM completeness pass (E-FIRE §E, `260800054`/`055` SMOKE FWD LWR
    // CAB REST BTL 1/2 FAULT): the FWD Lower Crew Rest (LDCR) module's own
    // two-bottle suppression circuit. Sourced as a real 2-bottle
    // architecture for this exact class of compartment: FCOM PRO-ABN-ECAM
    // p.4985 registers the identical procedure for the *aft* LDCR module
    // ("The fire extinguishing bottle 1(2) in the aft lower deck cabin
    // crew rest compartment is failed"), confirming Airbus fits this class
    // of lower-deck crew-rest module with two numbered bottles, not the
    // single lavatory-style extinguisher `registry.rs:100-103`'s own
    // waste-bin unit uses. Each bottle's own squib-circuit fault is a pass-
    // through discrete (same class as `smoke_detector::circuit_fault`), not
    // the cargo bays' two-stage knockdown/metered physics -- no FCOM or
    // FBW text distinguishes a knockdown/extended split for this smaller
    // module, so none is invented. Wired directly off `FIRE_LDCR_BTL_
    // {1,2}_SQUIB_FAULT` in `deep::ecam::fbw::ata26` (module doc:
    // `live::FireIceLive`'s own doc block on `ldcr_bottle_squib_fault`).
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

// ---------------------------------------------------------------------------
// Backlog item 5: anti-ice (icing.rs, backlog item 4, is environmental --
// no failure of its own; its consequence is what anti-ice below prevents)
// ---------------------------------------------------------------------------

fn register_anti_ice(r: &mut Registry) {
    let mut n = 0u16;

    for (zone, title) in [("wing_l", "L WING"), ("wing_r", "R WING")] {
        register_bleed_anti_ice(r, &mut n, zone, title);
    }
    for eng in 1..=4u16 {
        register_bleed_anti_ice(r, &mut n, &format!("nacelle{eng}"), &format!("ENG {eng} NACELLE"));
    }

    // Probes: pitot 1-3, AOA 1-3, TAT 1-2 (a representative, GENERIC set --
    // no published exact A380 probe count/heater layout is assumed beyond
    // "several redundant probes", the standard transport-aircraft
    // practice).
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

// ---------------------------------------------------------------------------
// ECAM alerts
// ---------------------------------------------------------------------------

fn register_ecam(r: &mut Registry) {
    // -- Fire warnings (level 3, red, CRC) --
    for eng in 1..=4u16 {
        r.alert(
            EcamAlert::new(&format!("ENG_{eng}_FIRE"), ATA_FIRE_PROTECTION, &format!("ENG {eng} FIRE"), Level::Warning, var(&format!("DEEP_FIRE_DETECTED_ENG:{eng}")).on())
                .confirm(0.0)
                .step(line(&format!("THR LEVER {eng}"), "IDLE").done(var(&format!("AUTOTHRUST_TLA:{eng}")).le(0.0)))
                .step(line(&format!("ENG {eng} MASTER"), "OFF").done(var(&format!("ENGINE_MASTER:{eng}")).off()))
                .step(line(&format!("ENG {eng} FIRE PB"), "PUSH").done(var(&format!("FIRE_BUTTON_ENG:{eng}")).on()))
                .step(line("IF FIRE ON THE FIRE PB", "").colour("white"))
                .step(line("AGENT 1", "DISCH").only_if(var(&format!("FIRE_BUTTON_ENG:{eng}")).on()).done(var(&format!("DEEP_FIRE_SQUIB_1_ENG_{eng}_IS_DISCHARGED")).on()).after(10.0))
                .step(line("AFTER 30 S IF FIRE PERSISTS: AGENT 2", "DISCH").only_if(var(&format!("FIRE_BUTTON_ENG:{eng}")).on()).done(var(&format!("DEEP_FIRE_SQUIB_2_ENG_{eng}_IS_DISCHARGED")).on()).after(30.0))
                .inop_sys(&format!("ENG {eng}"))
                .raised_by(&[failure_id(Area::FireIce, ATA_FIRE_PROTECTION, 100 + (eng as u16 - 1))]),
        );
    }
    r.alert(
        // This area owns the one APU FIRE warning. Two further causes are
        // contributed to it: the APU's own detection loop (`apu::registry`)
        // and the APU compartment running away thermally
        // (`thermal_zones::registry`). The APU is shut down before the
        // bottle is fired, so the agent meets a stopped, unfuelled APU.
        // W162: `DEEP_FIRE_DETECTED_APU`, not FlyByWire's own
        // `FIRE_DETECTED_APU` -- this trigger evaluates this area's own
        // model output, and the two used to collide (this area silently
        // overwrote FlyByWire's own APU fire detection every frame, since
        // `deep.tick` runs after `simulation.tick`).
        EcamAlert::new("APU_FIRE", ATA_FIRE_PROTECTION, "APU FIRE", Level::Warning, var("DEEP_FIRE_DETECTED_APU").on())
            .step(line("APU MASTER SW", "OFF").done(var("OVHD_APU_MASTER_SW_PB_IS_ON").off()))
            .step(line("APU FIRE PB", "PUSH").done(var("FIRE_BUTTON_APU").on()))
            .step(line("AGENT", "DISCH").only_if(var("FIRE_BUTTON_APU").on()).done(var("DEEP_FIRE_SQUIB_1_APU_1_IS_DISCHARGED").on()).after(1.0))
            .inop_sys("APU")
            .raised_by(&[failure_id(Area::FireIce, ATA_FIRE_PROTECTION, 104)]),
    );
    r.alert(
        // W162: `DEEP_FIRE_DETECTED_MLG`, not FlyByWire's own
        // `FIRE_DETECTED_MLG` -- same reason as APU FIRE above.
        EcamAlert::new("MLG_BAY_FIRE", ATA_FIRE_PROTECTION, "L/R WHEEL WELL FIRE", Level::Warning, var("DEEP_FIRE_DETECTED_MLG").on())
            .status_line("MLG bay fire detected, no dedicated extinguishing system fitted (matches FBW's own precedent)")
            .raised_by(&[failure_id(Area::FireIce, ATA_FIRE_PROTECTION, 105)]),
    );

    // -- Cargo smoke (level 3, red on the A380) --
    //
    // This area owns the three cargo smoke warnings; `thermal_zones`
    // contributes the physical trigger (a bay's modelled smoke
    // concentration passing what the detectors see) and its own failures,
    // through `Registry::contribute`. All three of the A380's holds are
    // here: forward, aft and bulk.
    //
    // The two-second confirmation and the take-off inhibit are the
    // detectors' own: a smoke warning is not annunciated on a single
    // sample, and CS-25 inhibits level-3 warnings through lift-off and
    // above 80 kt so nothing draws the crew off the roll.
    for (bay, title) in [("fwd", "CARGO SMOKE FWD"), ("aft", "CARGO SMOKE AFT"), ("bulk", "CARGO SMOKE BULK")] {
        let up = bay.to_uppercase();
        r.alert(
            EcamAlert::new(&format!("CARGO_SMOKE_{up}"), ATA_FIRE_PROTECTION, title, Level::Warning, var(&format!("CARGO_{up}_SMOKE_DETECTED")).on())
                .confirm(2.0)
                .inhibit(&[Phase::LiftOff, Phase::Above80Kt])
                .step(line("CARGO HEAT", "OFF").done(var(&format!("CARGO_HEAT_SW:{up}")).off()))
                .step(line("CARGO VENT SYS", "OFF").done(var("CARGO_VENT_SYS_SW").off()).after(5.0))
                .step(line(&format!("CARGO {up} FIRE AGENT"), "PUSH").done(var(&format!("CARGO_{up}_SUPPRESSION_ARMED")).on()))
                .status_line("LAND ASAP")
                .status_line("Suppression: high-rate knockdown then metered discharge for the extended diversion time (14 CFR/EASA CS-25.858)")
                .inop_sys("CARGO COMPT"),
        );
    }

    // -- Fire loop faults (level 2, amber, single chime) --
    for (zi, zone) in ZONES.iter().enumerate() {
        let open_a = failure_id(Area::FireIce, ATA_FIRE_PROTECTION, (zi as u16) * 4 + 1);
        let short_a = failure_id(Area::FireIce, ATA_FIRE_PROTECTION, (zi as u16) * 4 + 2);
        let open_b = failure_id(Area::FireIce, ATA_FIRE_PROTECTION, (zi as u16) * 4 + 3);
        let short_b = failure_id(Area::FireIce, ATA_FIRE_PROTECTION, (zi as u16) * 4 + 4);
        r.alert(
            EcamAlert::new(&format!("{}_FIRE_DET_FAULT", zone.to_uppercase()), ATA_FIRE_PROTECTION, &format!("{} FIRE DET FAULT", ZONE_TITLES[zi]), Level::Caution, any(vec![
                var(&format!("FIRE_LOOP_A_{}_FAULT", zone.to_uppercase())).on(),
                var(&format!("FIRE_LOOP_B_{}_FAULT", zone.to_uppercase())).on(),
                // A lone shorted loop is not a `loop_x_fault` (it reads
                // indistinguishably from real heat), but it *is* a real
                // disagreement between the two loops -- exactly what the
                // dual-loop architecture exists to let the crew see. Without
                // this arm, `open_a`/`open_b` were the only reachable causes
                // and `short_a`/`short_b` (named in `raised_by` below) could
                // never actually move this alert's trigger.
                var(&format!("FIRE_LOOP_{}_DISAGREE", zone.to_uppercase())).on(),
            ]))
                .confirm(5.0)
                .status_line(&format!("{} fire detection degraded to single loop", ZONE_TITLES[zi]))
                .raised_by(&[open_a, short_a, open_b, short_b]),
        );
    }

    // -- Fire bottle status (memo/advisory) --
    for eng in 1..=4u16 {
        r.alert(
            EcamAlert::new(&format!("ENG_{eng}_FIRE_AGENT_LO"), ATA_FIRE_PROTECTION, &format!("ENG {eng} FIRE AGENT LO PR"), Level::Advisory, any(vec![var(&format!("FIRE_BOTTLE_ENG{eng}_1_LOW_PRESSURE")).on(), var(&format!("FIRE_BOTTLE_ENG{eng}_2_LOW_PRESSURE")).on()]))
                .status_line(&format!("ENG {eng} fire bottle(s) below nominal pressure -- reduced/no agent available if fired")),
        );
    }

    // -- Anti-ice (memo when selected, caution on fault) --
    for (zone, title) in [("wing_l", "L WING"), ("wing_r", "R WING")] {
        r.alert(EcamAlert::new(&format!("{}_A_ICE_ON", zone.to_uppercase()), ATA_ICE_RAIN, &format!("{title} A-ICE"), Level::Memo, var(&format!("ANTI_ICE_{}_VALVE_OPEN", zone.to_uppercase())).on()));
    }
    for eng in 1..=4u16 {
        r.alert(
            EcamAlert::new(&format!("ENG_{eng}_NAI_FAULT"), ATA_ICE_RAIN, &format!("ENG {eng} NAI FAULT"), Level::Caution, var(&format!("ANTI_ICE_NACELLE{eng}_OVERHEAT")).on())
                .confirm(2.0)
                .step(line(&format!("ENG {eng} NAI"), "OFF").done(var(&format!("ANTI_ICE_NACELLE{eng}_VALVE_OPEN")).off())),
        );
    }
    r.alert(
        EcamAlert::new("WING_A_ICE_OVHT", ATA_ICE_RAIN, "WING A-ICE OVHT", Level::Caution, any(vec![var("ANTI_ICE_WING_L_OVERHEAT").on(), var("ANTI_ICE_WING_R_OVERHEAT").on()]))
            .confirm(2.0)
            .step(line("WING A-ICE", "OFF").done(all(vec![var("ANTI_ICE_WING_L_VALVE_OPEN").off(), var("ANTI_ICE_WING_R_VALVE_OPEN").off()]))),
    );
    r.alert(EcamAlert::new("WINDSHIELD_HEAT_FAULT", ATA_ICE_RAIN, "WINDSHIELD HEAT FAULT", Level::Caution, any(vec![var("WINDOW_HEAT_L_FAULT").on(), var("WINDOW_HEAT_R_FAULT").on()])).confirm(2.0));
    r.alert(EcamAlert::new("PROBE_HEAT_FAULT", ATA_ICE_RAIN, "PROBE/WINDOW HEAT", Level::Caution, any(vec![var("PROBE_HEAT_PITOT1_FAULT").on(), var("PROBE_HEAT_PITOT2_FAULT").on(), var("PROBE_HEAT_PITOT3_FAULT").on()])).confirm(10.0));
}
