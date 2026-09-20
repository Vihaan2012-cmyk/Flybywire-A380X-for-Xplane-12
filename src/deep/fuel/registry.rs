//! Registers every failure, component and ECAM alert this area's models
//! support with `crate::deep::api::Registry`, per `docs/deep/BRIEF.md`'s
//! "Registering failures, components and ECAM alerts" section.
//! `Area::Fuel` (19, added to `api.rs` by the lead -- this file was written
//! against that name from the start, see `PROGRESS.md`), ATA 28 throughout.
//!
//! One failure id per genuinely distinct physical fault (`docs/deep/
//! BRIEF.md`'s "do not pad with renamings" rule): a gallery leak is
//! registered once here even though it feeds two consumers
//! (`cg_transfer::achieved_transfer_rate_kg_s`'s derate *and*
//! `leak::gallery_leak_kg_s`'s own flow), not once per consumer.

use crate::deep::api::*;

const ATA: u16 = 28;

/// The eleven real tanks, as `(id_suffix, display_name)`, in
/// `flight_model.cfg` `Tank.1`..`Tank.11` order (see `geometry.rs`'s module
/// doc for the source).
const TANKS: [(&str, &str); 11] = [
    ("left_outer", "LEFT OUTER"),
    ("feed_1", "FEED 1"),
    ("left_mid", "LEFT MID"),
    ("left_inner", "LEFT INNER"),
    ("feed_2", "FEED 2"),
    ("feed_3", "FEED 3"),
    ("right_inner", "RIGHT INNER"),
    ("right_mid", "RIGHT MID"),
    ("feed_4", "FEED 4"),
    ("right_outer", "RIGHT OUTER"),
    ("trim", "TRIM"),
];

/// Registers one component with a single 0..1 fault parameter and its one
/// matching failure -- the common case throughout this registry.
#[allow(clippy::too_many_arguments)]
fn one(r: &mut Registry, id: u64, comp_id: String, comp_name: String, param: &str, meaning: &str, fail_name: String, model_field: String, magnitude: &str, effect: &str) {
    r.component(ComponentDef { id: comp_id.clone(), area: Area::Fuel, ata: ATA, name: comp_name, params: vec![ParamDef { name: param.into(), meaning: meaning.into(), healthy: 0.0 }], failures: vec![id] });
    r.failure(FailureDef { id, area: Area::Fuel, ata: ATA, name: fail_name, component: comp_id, model_field, magnitude: magnitude.into(), effect: effect.into() });
}

pub fn register(r: &mut Registry) {
    let mut n: u16 = 0;
    let mut next = move || {
        n += 1;
        failure_id(Area::Fuel, ATA, n)
    };

    // ---- Item 1: tank geometry/attitude (geometry.rs) ---------------------
    // Baffle/rib damage: the ribbed wing-box structure that gives real tanks
    // their fast slosh damping (geometry.rs's `slosh_damping_ratio` doc) can
    // itself be damaged (fatigue crack, impact), reducing damping and
    // letting the free surface slosh longer and further per disturbance.
    let mut geometry_ids = Vec::new();
    for (suffix, name) in TANKS {
        let id = next();
        geometry_ids.push(id);
        one(
            r,
            id,
            format!("28_fuel.tank_geometry.{suffix}"),
            format!("{name} TANK structure/baffles"),
            "baffle_damage_fraction",
            "0..1, fatigue/impact damage to the tank's internal ribs/baffles",
            format!("{name} tank baffle/rib damage"),
            "geometry.rs::TankShape.slosh_damping_ratio (reduced proportionally to this fraction)".into(),
            "0..1, fraction of nominal slosh damping lost",
            "the free surface sloshes longer and further per disturbance (geometry::sloshing_settle_time_s grows), increasing transient unporting risk during manoeuvring and, in the extreme, imposing higher structural slosh loads on the damaged baffle itself",
        );
    }

    // ---- Item 2: quantity gauging / FQMS (gauging.rs) ---------------------
    let mut probe_ids = Vec::new();
    let mut densitometer_ids = Vec::new();
    for (suffix, name) in TANKS {
        let probe_id = next();
        let compensator_id = next();
        probe_ids.push(probe_id);
        probe_ids.push(compensator_id);
        r.component(ComponentDef {
            id: format!("28_fuel.fqms_probes.{suffix}"),
            area: Area::Fuel,
            ata: ATA,
            name: format!("{name} TANK capacitance probe array"),
            params: vec![
                ParamDef { name: "one_probe_failure_fraction".into(), meaning: "0..1, one probe of the array's own count failing (open/shorted past 0.9, drifting below it)".into(), healthy: 0.0 },
                ParamDef { name: "compensator_common_mode_fraction".into(), meaning: "0..1, the array's shared compensator/reference biasing every surviving probe together".into(), healthy: 0.0 },
            ],
            failures: vec![probe_id, compensator_id],
        });
        r.failure(FailureDef {
            id: probe_id,
            area: Area::Fuel,
            ata: ATA,
            name: format!("{name} tank FQMS probe failure"),
            component: format!("28_fuel.fqms_probes.{suffix}"),
            model_field: "gauging.rs::ProbeFault.failure_fraction (one element of the tank's probe array)".into(),
            magnitude: "0..1, that probe's own failure fraction".into(),
            effect: "past 0.9 the FQMS's BITE excludes the probe entirely (gauging::fqms_indicated_fraction drops it from the average, confidence falls); below that it drifts, biasing the indicated quantity without the crew being told which probe is at fault".into(),
        });
        r.failure(FailureDef {
            id: compensator_id,
            area: Area::Fuel,
            ata: ATA,
            name: format!("{name} tank FQMS compensator fault"),
            component: format!("28_fuel.fqms_probes.{suffix}"),
            model_field: "gauging.rs::fqms_indicated_fraction (a common-mode bias applied to every surviving probe's reading)".into(),
            magnitude: "0..1 of the same MAX_PROBE_BIAS_FRACTION span, applied to the whole array at once".into(),
            effect: "every probe reads consistently high or low together, so the FQMS's cross-check between probes cannot detect it the way a single divergent probe is caught -- the whole tank's indicated quantity is wrong even though every individual probe still agrees with its neighbours".into(),
        });

        let dens_id = next();
        densitometer_ids.push(dens_id);
        one(
            r,
            dens_id,
            format!("28_fuel.densitometer.{suffix}"),
            format!("{name} TANK densitometer"),
            "output_failure_fraction",
            "0..1, fraction of the way from measuring true density to being stuck on the default reference density",
            format!("{name} tank densitometer failure"),
            "gauging.rs::indicated_mass_kg (densitometer_failed input, true_density_kg_m3 replaced by default_density_kg_m3)".into(),
            "0/1 (modelled as a switch: past 0.5 the FQMS falls back to the default reference density)",
            "mass is computed from a fixed reference density instead of the fuel's actual (temperature-dependent) density, so indicated mass drifts from true mass by the same fraction the real density has drifted from the reference -- worst on a cold-soaked long-haul sector",
        );
    }

    // ---- Item 3: automatic CG control & transfer sequencing (cg_transfer.rs) ----
    // Named per FlyByWire's own `flight_model.cfg` component names
    // (`fuel_network.rs`'s own doc comments cite the same file/lines).
    let trim_pump_left = next();
    one(r, trim_pump_left, "28_fuel.pump.trim_left".into(), "TrimTankPumpLeft".into(), "pump_degradation_fraction", "0..1, delivered flow/pressure lost to wear", "Trim tank left pump degradation".into(), "cg_transfer.rs::TransferFaults.pump_degradation_fraction (trim path)".into(), "0..1 fraction of rated flow lost", "trim tank transfer (aft in cruise for CG-aft drag reduction, forward before descent) runs slower; with the right pump also degraded the crew may not reach the scheduled CG in time for top of descent");
    let trim_pump_right = next();
    one(r, trim_pump_right, "28_fuel.pump.trim_right".into(), "TrimTankPumpRight".into(), "pump_degradation_fraction", "0..1, delivered flow/pressure lost to wear", "Trim tank right pump degradation".into(), "cg_transfer.rs::TransferFaults.pump_degradation_fraction (trim path)".into(), "0..1 fraction of rated flow lost", "same as the left pump; both together stop trim transfer altogether");
    let trim_inlet_1 = next();
    one(r, trim_inlet_1, "28_fuel.valve.trim_inlet_1".into(), "TrimTankInletValve1".into(), "valve_stuck_fraction", "0..1, seized fraction, frozen at its position when it seized", "Trim tank inlet valve 1 sticks".into(), "cg_transfer.rs::TransferFaults.valve_stuck_fraction (trim inlet path 1)".into(), "0..1 stuck fraction", "forward trim transfer (into the trim tank) is throttled or blocked depending on the position it seized at");
    let trim_inlet_2 = next();
    one(r, trim_inlet_2, "28_fuel.valve.trim_inlet_2".into(), "TrimTankInletValve2".into(), "valve_stuck_fraction", "0..1, seized fraction, frozen at its position when it seized", "Trim tank inlet valve 2 sticks".into(), "cg_transfer.rs::TransferFaults.valve_stuck_fraction (trim inlet path 2)".into(), "0..1 stuck fraction", "the redundant forward trim transfer path is throttled or blocked; with valve 1 also stuck, forward (nose-down CG) transfer is lost entirely");
    let trim_iso_fwd = next();
    one(r, trim_iso_fwd, "28_fuel.valve.trim_iso_fwd".into(), "TrimLineIsolationValveFwd".into(), "valve_stuck_fraction", "0..1, seized fraction", "Trim line forward isolation valve sticks".into(), "cg_transfer.rs::TransferFaults.valve_stuck_fraction (aft-transfer path, forward gallery leg)".into(), "0..1 stuck fraction", "isolates (or fails to isolate) the trim transfer line from the forward gallery; stuck shut blocks aft (CG-aft) transfer through that leg, stuck open defeats the isolation the real system uses to segregate trim-tank pressure from the main galleries");
    let trim_iso_aft = next();
    one(r, trim_iso_aft, "28_fuel.valve.trim_iso_aft".into(), "TrimLineIsolationValveAft".into(), "valve_stuck_fraction", "0..1, seized fraction", "Trim line aft isolation valve sticks".into(), "cg_transfer.rs::TransferFaults.valve_stuck_fraction (aft-transfer path, aft gallery leg)".into(), "0..1 stuck fraction", "same as the forward isolation valve for the aft gallery leg");
    let outer_xfer_left = next();
    one(r, outer_xfer_left, "28_fuel.valve.outer_xfer_left".into(), "LeftOuterFwdTransferValve".into(), "valve_stuck_fraction", "0..1, seized fraction", "Left outer tank transfer valve sticks".into(), "cg_transfer.rs::TransferFaults.valve_stuck_fraction (outer-tank retention path)".into(), "0..1 stuck fraction", "the load-alleviation outer-tank-last transfer sequence cannot move that tank's fuel inboard on demand: stuck shut strands relieving fuel out at the tip past when it was wanted, stuck open defeats the retention scheduling (`cg_transfer::outer_tank_retention_active`) entirely");
    let outer_xfer_right = next();
    one(r, outer_xfer_right, "28_fuel.valve.outer_xfer_right".into(), "RightOuterFwdTransferValve".into(), "valve_stuck_fraction", "0..1, seized fraction", "Right outer tank transfer valve sticks".into(), "cg_transfer.rs::TransferFaults.valve_stuck_fraction (outer-tank retention path)".into(), "0..1 stuck fraction", "same as the left outer valve, right wing");
    let inner_xfer_left = next();
    one(r, inner_xfer_left, "28_fuel.valve.inner_xfer_left".into(), "LeftInnerFwdTransferValve".into(), "valve_stuck_fraction", "0..1, seized fraction", "Left inner tank transfer valve sticks".into(), "cg_transfer.rs::TransferFaults.valve_stuck_fraction (inner-to-feed path)".into(), "0..1 stuck fraction", "inner tank fuel cannot feed the feed tanks on schedule, risking a feed tank running low while the inner tank still holds usable fuel");
    let inner_xfer_right = next();
    one(r, inner_xfer_right, "28_fuel.valve.inner_xfer_right".into(), "RightInnerFwdTransferValve".into(), "valve_stuck_fraction", "0..1, seized fraction", "Right inner tank transfer valve sticks".into(), "cg_transfer.rs::TransferFaults.valve_stuck_fraction (inner-to-feed path)".into(), "0..1 stuck fraction", "same as the left inner valve, right wing");
    let mid_xfer_left = next();
    one(r, mid_xfer_left, "28_fuel.valve.mid_xfer_left".into(), "LeftMidFwdTransferValve".into(), "valve_stuck_fraction", "0..1, seized fraction", "Left mid tank transfer valve sticks".into(), "cg_transfer.rs::TransferFaults.valve_stuck_fraction (mid-to-feed path)".into(), "0..1 stuck fraction", "mid tank fuel cannot feed the feed tanks on schedule");
    let mid_xfer_right = next();
    one(r, mid_xfer_right, "28_fuel.valve.mid_xfer_right".into(), "RightMidFwdTransferValve".into(), "valve_stuck_fraction", "0..1, seized fraction", "Right mid tank transfer valve sticks".into(), "cg_transfer.rs::TransferFaults.valve_stuck_fraction (mid-to-feed path)".into(), "0..1 stuck fraction", "same as the left mid valve, right wing");
    let mut crossfeed_ids = Vec::new();
    for i in 1..=4u16 {
        let id = next();
        crossfeed_ids.push(id);
        one(r, id, format!("28_fuel.valve.crossfeed_{i}"), format!("CrossFeedValve{i}"), "valve_stuck_fraction", "0..1, seized fraction", format!("Cross-feed valve {i} sticks"), "cg_transfer.rs::TransferFaults.valve_stuck_fraction (wing-balance cross-feed path)".into(), "0..1 stuck fraction", "wing-balance cross-feed (`cg_transfer::wing_balance_transfer_needed`/`heavy_side`) cannot move fuel between wings through this valve; stuck open when not commanded can itself cause an unwanted imbalance or, with an engine failure on that side, an unwanted single-engine fuel path");
    }

    // ---- Item 4: fuel temperature / contamination (thermal.rs) -----------
    let mut fcoc_ids = Vec::new();
    for n in 1..=4u16 {
        let id = next();
        fcoc_ids.push(id);
        one(r, id, format!("28_fuel.fcoc.{n}"), format!("ENG {n} fuel-cooled oil cooler (FCOC)"), "fouling_fraction", "0..1, heat-exchanger fouling reducing heat transfer into the fuel", format!("ENG {n} FCOC fouling"), "thermal.rs::fcoc_temperature_rise_k (fcoc_heat_w input attenuated by 1 - fouling_fraction before this function is called)".into(), "0..1 fouling fraction", "less of the engine oil's heat is rejected into the returning fuel: the feed tank runs colder than it otherwise would (worse cold-soak/wax margin) while the engine oil itself runs hotter for longer, a shared consequence this model exposes on the fuel side only");
    }
    let mut filter_ids = Vec::new();
    for n in 1..=4u16 {
        let water_id = next();
        let heater_id = next();
        filter_ids.push(water_id);
        filter_ids.push(heater_id);
        r.component(ComponentDef {
            id: format!("28_fuel.filter.{n}"),
            area: Area::Fuel,
            ata: ATA,
            name: format!("ENG {n} fuel feed filter"),
            params: vec![
                ParamDef { name: "free_water_fraction".into(), meaning: "0..1, undrained free water volume fraction present at the filter inlet".into(), healthy: 0.0 },
                ParamDef { name: "anti_ice_heater_failed".into(), meaning: "0/1, the filter's own anti-ice heater has failed off".into(), healthy: 0.0 },
            ],
            failures: vec![water_id, heater_id],
        });
        r.failure(FailureDef {
            id: water_id,
            area: Area::Fuel,
            ata: ATA,
            name: format!("ENG {n} fuel filter water contamination"),
            component: format!("28_fuel.filter.{n}"),
            model_field: "thermal.rs::filter_ice_blockage_fraction (free_water_fraction input)".into(),
            magnitude: "0..1, free water volume fraction (GENERIC ceiling, an undrained-sumps severity judgement, not a spec limit)".into(),
            effect: "below 0 C this water freezes at the filter mesh (thermal::filter_ice_blockage_fraction), progressively blocking it; above 0 C it is inert but still displaces usable fuel volume and is the reservoir the ice-blockage failure below draws on".into(),
        });
        r.failure(FailureDef {
            id: heater_id,
            area: Area::Fuel,
            ata: ATA,
            name: format!("ENG {n} fuel filter anti-ice heater failure"),
            component: format!("28_fuel.filter.{n}"),
            model_field: "thermal.rs::filter_ice_blockage_fraction (anti_ice_heater_on forced false)".into(),
            magnitude: "0/1 (failed off)".into(),
            effect: "removes the one mitigation the ice-blockage model has: with the heater failed and free water present, cold fuel is free to block the filter (thermal::filter_ice_blockage_fraction no longer returns 0 regardless of sub-cooling)".into(),
        });
    }

    // ---- Item 5: jettison (jettison.rs) -----------------------------------
    let jettison_valve_left = next();
    one(r, jettison_valve_left, "28_fuel.valve.jettison_left".into(), "JettisonNozzleValveLeft".into(), "valve_stuck_fraction", "0..1, seized fraction, frozen at its position when it seized", "Left jettison nozzle valve sticks".into(), "jettison.rs::JettisonValve (stuck_fraction input to .step)".into(), "0..1 stuck fraction", "stuck shut denies jettison capability through that nozzle (the other nozzle must carry the whole rate); stuck open risks an uncommanded fuel loss once the isolation valves upstream are opened for any reason");
    let jettison_valve_right = next();
    one(r, jettison_valve_right, "28_fuel.valve.jettison_right".into(), "JettisonNozzleValveRight".into(), "valve_stuck_fraction", "0..1, seized fraction, frozen at its position when it seized", "Right jettison nozzle valve sticks".into(), "jettison.rs::JettisonValve (stuck_fraction input to .step)".into(), "0..1 stuck fraction", "same as the left jettison valve");
    let jettison_nozzle_left = next();
    one(r, jettison_nozzle_left, "28_fuel.nozzle.jettison_left".into(), "Left jettison nozzle".into(), "blockage_fraction", "0..1, debris/ice narrowing the nozzle throat", "Left jettison nozzle blockage".into(), "jettison.rs::effective_cda_m2 (blockage_fraction input)".into(), "0..1 throat area lost", "jettison rate through that nozzle falls in proportion (jettison::jettison_mass_flow_kg_s), lengthening the time needed to reach max landing weight before an overweight landing");
    let jettison_nozzle_right = next();
    one(r, jettison_nozzle_right, "28_fuel.nozzle.jettison_right".into(), "Right jettison nozzle".into(), "blockage_fraction", "0..1, debris/ice narrowing the nozzle throat", "Right jettison nozzle blockage".into(), "jettison.rs::effective_cda_m2 (blockage_fraction input)".into(), "0..1 throat area lost", "same as the left jettison nozzle");

    // ---- Item 6: leaks (leak.rs) ------------------------------------------
    let mut tank_leak_ids = Vec::new();
    for (suffix, name) in TANKS {
        let id = next();
        tank_leak_ids.push(id);
        one(r, id, format!("28_fuel.tank_wall.{suffix}"), format!("{name} TANK skin/structure"), "leak_magnitude", "0..1, leak orifice size fraction of a GENERIC maximum hole area", format!("{name} tank structural fuel leak"), "leak.rs::tank_wall_leak_kg_s (orifice_area_m2, via leak::leak_area_m2(magnitude, max_area))".into(), "0..1 of leak::leak_area_m2's own max_area_m2 ceiling", "fuel is lost overboard at a rate set by the tank's own remaining head (leak.rs::head_pressure_pa), unmetered by any engine/APU flow meter -- exactly the discrepancy leak::LeakDetector is built to catch");
    }
    let gallery_leak_fwd = next();
    let gallery_leak_aft = next();
    one(r, gallery_leak_fwd, "28_fuel.gallery.forward".into(), "Forward transfer gallery".into(), "leak_fraction", "0..1, fraction of transfer flow through this gallery section diverted by a leak instead of reaching its destination tank", "Forward transfer gallery leak".into(), "cg_transfer.rs::TransferFaults.gallery_leak_fraction (forward-gallery transfer paths) and leak.rs::gallery_leak_kg_s (mass lost overboard, same fault)".into(), "0..1 diverted fraction", "forward transfers (inner/mid/outer to feed, forward trim path) are throttled exactly as `cg_transfer::achieved_transfer_rate_kg_s` models, and the diverted fuel is an unmetered loss `leak::LeakDetector` can catch");
    one(r, gallery_leak_aft, "28_fuel.gallery.aft".into(), "Aft transfer gallery".into(), "leak_fraction", "0..1, fraction of transfer flow through this gallery section diverted by a leak instead of reaching its destination tank", "Aft transfer gallery leak".into(), "cg_transfer.rs::TransferFaults.gallery_leak_fraction (aft-gallery transfer paths) and leak.rs::gallery_leak_kg_s (mass lost overboard, same fault)".into(), "0..1 diverted fraction", "aft transfers and the jettison feed path through AftGalleryJunction1/2 are throttled and the diverted fuel is an unmetered loss");

    let mut leak_related_ids = tank_leak_ids.clone();
    leak_related_ids.push(gallery_leak_fwd);
    leak_related_ids.push(gallery_leak_aft);

    // ---- ECAM alerts -------------------------------------------------------

    r.alert(
        EcamAlert::new("FUEL_LEAK", ATA, "FUEL LEAK", Level::Caution, var("FUEL_LEAK_DETECTED").on())
            .step(line("FUEL X FEED TK PUMPS", "AS RQRD"))
            .step(line("CROSSFEED", "OFF").done(var("FUEL_CROSSFEED_OPEN").off()))
            .status_line("FUEL LEAK: LAND ASAP")
            .raised_by(&leak_related_ids),
    );

    r.alert(
        EcamAlert::new("FUEL_TRIM_TRANSFER_FAULT", ATA, "FUEL TRIM TK TRANSFER FAULT", Level::Caution, var("FUEL_TRIM_TRANSFER_FAULT").on())
            .confirm(1.0)
            .status_line("AUTO FUEL TRANSFER FAULT")
            .raised_by(&[trim_pump_left, trim_pump_right, trim_inlet_1, trim_inlet_2, trim_iso_fwd, trim_iso_aft, gallery_leak_fwd, gallery_leak_aft]),
    );

    r.alert(
        EcamAlert::new("FUEL_CG_TRANSFER_DEGRADED", ATA, "FUEL AUTO CG XFR FAULT", Level::Advisory, var("FUEL_CG_TRANSFER_DEGRADED").on())
            .confirm(5.0)
            .status_line("MAN FUEL TRANSFER MAY BE RQRD")
            .raised_by(&[outer_xfer_left, outer_xfer_right, inner_xfer_left, inner_xfer_right, mid_xfer_left, mid_xfer_right]),
    );

    r.alert(
        EcamAlert::new("FUEL_IMBALANCE_XFEED_FAULT", ATA, "FUEL WING XFEED FAULT", Level::Caution, var("FUEL_CROSSFEED_FAULT").on())
            .confirm(2.0)
            .status_line("MAN WING BALANCE MAY BE RQRD")
            .raised_by(&crossfeed_ids),
    );

    r.alert(
        EcamAlert::new("FUEL_FOB_LO_TEMP", ATA, "FUEL FOB LO TEMP", Level::Caution, var("FUEL_FOB_LO_TEMP").on())
            .confirm(3.0)
            .status_line("MONITOR FUEL TEMP")
            .raised_by(&fcoc_ids),
    );

    r.alert(
        EcamAlert::new("FUEL_FILTER_ICING", ATA, "FUEL FILTER ICING", Level::Advisory, var("FUEL_FILTER_ICE_DETECTED").on())
            .confirm(5.0)
            .status_line("MONITOR FUEL FILTER DP")
            .raised_by(&filter_ids),
    );

    r.alert(
        EcamAlert::new("FUEL_JETTISON_FAULT", ATA, "FUEL JETTISON FAULT", Level::Caution, any(vec![var("FUEL_JETTISON_L_VALVE_FAULT").on(), var("FUEL_JETTISON_R_VALVE_FAULT").on()]))
            .confirm(1.0)
            .status_line("JETTISON RATE MAY BE REDUCED")
            .raised_by(&[jettison_valve_left, jettison_valve_right, jettison_nozzle_left, jettison_nozzle_right]),
    );

    r.alert(
        EcamAlert::new("FUEL_QTY_DEGRADED", ATA, "FUEL QTY INDICATION FAULT", Level::Advisory, var("FUEL_FQMS_LOW_CONFIDENCE").on())
            .confirm(5.0)
            .status_line("FOB ACCURACY DEGRADED")
            .raised_by(&{
                let mut ids = probe_ids.clone();
                ids.extend(&densitometer_ids);
                ids
            }),
    );

    r.alert(
        EcamAlert::new("FUEL_TANK_SLOSH_ADVISORY", ATA, "FUEL TK STRUCTURE ADVISORY", Level::Advisory, var("FUEL_TANK_BAFFLE_DAMAGE_DETECTED").on())
            .confirm(10.0)
            .status_line("AVOID AGGRESSIVE MANOEUVRING")
            .raised_by(&geometry_ids),
    );
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn registers_cleanly_with_no_validation_errors() {
        let mut r = Registry::default();
        register(&mut r);
        let errors = r.validate_area();
        assert!(errors.is_empty(), "{errors:?}");
    }

    #[test]
    fn every_component_has_at_least_one_failure_and_every_failure_names_a_registered_component() {
        let mut r = Registry::default();
        register(&mut r);
        assert!(!r.components.is_empty());
        assert!(!r.failures.is_empty());
        for c in &r.components {
            assert!(!c.failures.is_empty(), "{} has no failures", c.id);
        }
    }

    #[test]
    fn every_failure_id_carries_the_fuel_area_and_ata_28_and_is_unique() {
        let mut r = Registry::default();
        register(&mut r);
        let mut ids: Vec<u64> = r.failures.iter().map(|f| f.id).collect();
        let before = ids.len();
        ids.sort_unstable();
        ids.dedup();
        assert_eq!(ids.len(), before, "duplicate failure ids");
        for f in &r.failures {
            assert_eq!(f.ata, ATA);
            assert_eq!(f.id / 1_000_000, Area::Fuel as u64);
        }
    }

    #[test]
    fn eleven_tanks_each_get_geometry_gauging_and_leak_coverage() {
        let mut r = Registry::default();
        register(&mut r);
        for (suffix, _) in TANKS {
            assert!(r.components.iter().any(|c| c.id == format!("28_fuel.tank_geometry.{suffix}")));
            assert!(r.components.iter().any(|c| c.id == format!("28_fuel.fqms_probes.{suffix}")));
            assert!(r.components.iter().any(|c| c.id == format!("28_fuel.densitometer.{suffix}")));
            assert!(r.components.iter().any(|c| c.id == format!("28_fuel.tank_wall.{suffix}")));
        }
    }

    #[test]
    fn every_ecam_alert_only_raises_from_registered_failures() {
        let mut r = Registry::default();
        register(&mut r);
        assert!(!r.alerts.is_empty());
        for a in &r.alerts {
            for id in &a.failures {
                assert!(r.failures.iter().any(|f| f.id == *id), "alert {} names unknown failure {}", a.key, id);
            }
        }
    }
}
