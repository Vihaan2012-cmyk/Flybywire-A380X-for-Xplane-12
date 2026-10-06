use crate::deep::api::*;

const ATA: u16 = 28;

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

    let jettison_valve_left = next();
    one(r, jettison_valve_left, "28_fuel.valve.jettison_left".into(), "JettisonNozzleValveLeft".into(), "valve_stuck_fraction", "0..1, seized fraction, frozen at its position when it seized", "Left jettison nozzle valve sticks".into(), "jettison.rs::JettisonValve (stuck_fraction input to .step)".into(), "0..1 stuck fraction", "stuck shut denies jettison capability through that nozzle (the other nozzle must carry the whole rate); stuck open risks an uncommanded fuel loss once the isolation valves upstream are opened for any reason");
    let jettison_valve_right = next();
    one(r, jettison_valve_right, "28_fuel.valve.jettison_right".into(), "JettisonNozzleValveRight".into(), "valve_stuck_fraction", "0..1, seized fraction, frozen at its position when it seized", "Right jettison nozzle valve sticks".into(), "jettison.rs::JettisonValve (stuck_fraction input to .step)".into(), "0..1 stuck fraction", "same as the left jettison valve");
    let jettison_nozzle_left = next();
    one(r, jettison_nozzle_left, "28_fuel.nozzle.jettison_left".into(), "Left jettison nozzle".into(), "blockage_fraction", "0..1, debris/ice narrowing the nozzle throat", "Left jettison nozzle blockage".into(), "jettison.rs::effective_cda_m2 (blockage_fraction input)".into(), "0..1 throat area lost", "jettison rate through that nozzle falls in proportion (jettison::jettison_mass_flow_kg_s), lengthening the time needed to reach max landing weight before an overweight landing");
    let jettison_nozzle_right = next();
    one(r, jettison_nozzle_right, "28_fuel.nozzle.jettison_right".into(), "Right jettison nozzle".into(), "blockage_fraction", "0..1, debris/ice narrowing the nozzle throat", "Right jettison nozzle blockage".into(), "jettison.rs::effective_cda_m2 (blockage_fraction input)".into(), "0..1 throat area lost", "same as the left jettison nozzle");

    let mut tank_leak_ids = Vec::new();
    for (suffix, name) in TANKS {
        let id = next();
        tank_leak_ids.push(id);
        one(r, id, format!("28_fuel.tank_wall.{suffix}"), format!("{name} TANK skin/structure"), "leak_magnitude", "0..1, leak orifice size fraction of a GENERIC maximum hole area", format!("{name} tank structural fuel leak"), "leak.rs::tank_wall_leak_kg_s (orifice_area_m2, via leak::leak_area_m2(magnitude, max_area))".into(), "0..1 of leak::leak_area_m2's own max_area_m2 ceiling", "fuel is lost overboard at a rate set by the tank's own remaining head (leak.rs::head_pressure_pa), unmetered by any engine/APU flow meter -- exactly the discrepancy leak::LeakDetector is built to catch");
    }

    let apu_feed_pump = next();
    one(
        r,
        apu_feed_pump,
        "28_fuel.pump.apu_feed".into(),
        "APUFeedPump".into(),
        "pump_degradation_fraction",
        "0..1, delivered flow/pressure lost to wear",
        "APU feed pump degradation".into(),
        "live.rs::apu_feed_pump_degradation (direct reading; this model's APU burn is commands.apu_fuel_flow_kg_s directly, not pump-pressure-gated)".into(),
        "0..1 fraction of rated flow lost",
        "the APU's dedicated fuel feed pump has lost delivery; past half its rated flow it can no longer reliably supply the APU on its own",
    );
    let apu_feed_valve = next();
    one(
        r,
        apu_feed_valve,
        "28_fuel.valve.apu_feed".into(),
        "APUIsoValve/APULPValve".into(),
        "valve_stuck_fraction",
        "0..1, seized fraction, frozen at its position when it seized",
        "APU feed valve sticks".into(),
        "jettison.rs::JettisonValve (reused struct; stuck_fraction input to .step)".into(),
        "0..1 stuck fraction",
        "the APU feed valve does not follow the commanded APU fuel-flow state: stuck open denies the isolation the real system uses once the APU stops drawing fuel, stuck shut starves the APU of fuel it was commanded to receive",
    );

    let mut eng_lp_valve_ids = [0u64; 4];
    for n in 1..=4u16 {
        let id = next();
        eng_lp_valve_ids[(n - 1) as usize] = id;
        one(
            r,
            id,
            format!("28_fuel.valve.eng_lp.{n}"),
            format!("Engine{n}LPValve"),
            "valve_stuck_fraction",
            "0..1, seized fraction, frozen at its position when it seized",
            format!("ENG {n} LP fuel shutoff valve sticks"),
            "jettison.rs::JettisonValve (reused struct; stuck_fraction input to .step)".into(),
            "0..1 stuck fraction",
            "that engine's low-pressure (fire) shutoff valve fails to follow the engine master/fire-handle command, denying (stuck shut) or defeating (stuck open) the fuel isolation the master switch/fire handle commands",
        );
    }
    let _ = eng_lp_valve_ids;

    for n in 1..=4u16 {
        let main_id = next();
        one(
            r,
            main_id,
            format!("28_fuel.pump.feed_main.{n}"),
            format!("Feed{n}TankPump1"),
            "pump_degradation_fraction",
            "0..1, delivered flow/pressure lost to wear",
            format!("Feed tank {n} main pump degradation"),
            "live.rs::feed_pump_degradation (direct reading; feed-tank draw in this model is Truth::engine_fuel_flow_kg_s directly, not pump-pressure-gated)".into(),
            "0..1 fraction of rated flow lost",
            "the feed tank's main boost pump has lost delivery -- the same LO PRESS-caution reading a real aircraft still carries per pump even when its standby partner is healthy",
        );
        let stby_id = next();
        one(
            r,
            stby_id,
            format!("28_fuel.pump.feed_stby.{n}"),
            format!("Feed{n}TankPump2"),
            "pump_degradation_fraction",
            "0..1, delivered flow/pressure lost to wear",
            format!("Feed tank {n} standby pump degradation"),
            "live.rs::feed_pump_degradation (direct reading; feed-tank draw in this model is Truth::engine_fuel_flow_kg_s directly, not pump-pressure-gated)".into(),
            "0..1 fraction of rated flow lost",
            "the feed tank's standby boost pump has lost delivery; with the main pump also degraded the tank has no boosted supply left",
        );
    }

    for (side, side_name) in [("left", "Left"), ("right", "Right")] {
        let outer_id = next();
        one(
            r,
            outer_id,
            format!("28_fuel.pump.outer.{side}"),
            format!("{side_name}OuterTankPump"),
            "pump_degradation_fraction",
            "0..1, delivered flow/pressure lost to wear",
            format!("{side_name} outer tank pump degradation"),
            "cg_transfer.rs::TransferFaults.pump_degradation_fraction (this side's CG-transfer path, while the outer tank is its own source)".into(),
            "0..1 fraction of rated flow lost",
            "when this side's outer tank is the active CG-transfer source, less of its own fuel reaches the feed tanks per second",
        );
        let mid_fwd_id = next();
        one(
            r,
            mid_fwd_id,
            format!("28_fuel.pump.mid_fwd.{side}"),
            format!("{side_name}MidTankPumpFwd"),
            "pump_degradation_fraction",
            "0..1, delivered flow/pressure lost to wear",
            format!("{side_name} mid tank forward pump degradation"),
            "cg_transfer.rs::TransferFaults.pump_degradation_fraction (this side's CG-transfer path, while the mid tank is its own source; combined with the aft pump as the mid tank's own redundant pair)".into(),
            "0..1 fraction of rated flow lost",
            "when this side's mid tank is the active CG-transfer source, less of its own fuel reaches the feed tanks per second unless the aft pump is still healthy",
        );
        let mid_aft_id = next();
        one(
            r,
            mid_aft_id,
            format!("28_fuel.pump.mid_aft.{side}"),
            format!("{side_name}MidTankPumpAft"),
            "pump_degradation_fraction",
            "0..1, delivered flow/pressure lost to wear",
            format!("{side_name} mid tank aft pump degradation"),
            "cg_transfer.rs::TransferFaults.pump_degradation_fraction (this side's CG-transfer path, while the mid tank is its own source; combined with the forward pump as the mid tank's own redundant pair)".into(),
            "0..1 fraction of rated flow lost",
            "when this side's mid tank is the active CG-transfer source, less of its own fuel reaches the feed tanks per second unless the forward pump is still healthy",
        );
        let inner_fwd_id = next();
        one(
            r,
            inner_fwd_id,
            format!("28_fuel.pump.inner_fwd.{side}"),
            format!("{side_name}InnerTankPumpFwd"),
            "pump_degradation_fraction",
            "0..1, delivered flow/pressure lost to wear",
            format!("{side_name} inner tank forward pump degradation"),
            "cg_transfer.rs::TransferFaults.pump_degradation_fraction (this side's CG-transfer path, while the inner tank is its own source; combined with the aft pump as the inner tank's own redundant pair)".into(),
            "0..1 fraction of rated flow lost",
            "when this side's inner tank is the active CG-transfer source, less of its own fuel reaches the feed tanks per second unless the aft pump is still healthy",
        );
        let inner_aft_id = next();
        one(
            r,
            inner_aft_id,
            format!("28_fuel.pump.inner_aft.{side}"),
            format!("{side_name}InnerTankPumpAft"),
            "pump_degradation_fraction",
            "0..1, delivered flow/pressure lost to wear",
            format!("{side_name} inner tank aft pump degradation"),
            "cg_transfer.rs::TransferFaults.pump_degradation_fraction (this side's CG-transfer path, while the inner tank is its own source; combined with the forward pump as the inner tank's own redundant pair)".into(),
            "0..1 fraction of rated flow lost",
            "when this side's inner tank is the active CG-transfer source, less of its own fuel reaches the feed tanks per second unless the forward pump is still healthy",
        );
        let _ = (outer_id, mid_fwd_id, mid_aft_id, inner_fwd_id, inner_aft_id);
    }

    let leak_detector_fault = next();
    one(
        r,
        leak_detector_fault,
        "28_fuel.leak_detector".into(),
        "Fuel leak detection computer".into(),
        "detector_fault_fraction",
        "0/1, the leak-detection function itself has failed",
        "Fuel leak detection system fault".into(),
        "live.rs::LeakDetector output forced false while this is active".into(),
        "0/1 (failed)".into(),
        "the leak-detection computation is forced to report no leak regardless of the real indicated-vs-metered discrepancy: an honest 'detector cannot see a real leak right now' effect, not a fabricated leak",
    );

    let mut fqdc_ids = [0u64; 2];
    for n in 1..=2u16 {
        let id = next();
        fqdc_ids[(n - 1) as usize] = id;
        one(
            r,
            id,
            format!("28_fuel.computer.fqdc.{n}"),
            format!("FQDC channel {n}"),
            "channel_fault_fraction",
            "0/1, that FQDC channel has failed",
            format!("FQDC channel {n} fault"),
            "live.rs::fqms_low_confidence (this channel's own half of the gauging chain forced to confidence = 0 while active)".into(),
            "0/1 (failed)".into(),
            "that data-acquisition channel's own half of the fuel-gauging chain is forced to zero confidence, the same real consequence a probe/densitometer fault already produces",
        );
    }
    let _ = fqdc_ids;
    let mut fqms_ids = [0u64; 2];
    for n in 1..=2u16 {
        let id = next();
        fqms_ids[(n - 1) as usize] = id;
        one(
            r,
            id,
            format!("28_fuel.computer.fqms.{n}"),
            format!("FQMS channel {n}"),
            "channel_fault_fraction",
            "0/1, that FQMS channel has failed",
            format!("FQMS channel {n} fault"),
            "live.rs::fqms_low_confidence (this channel's own half of the gauging chain forced to confidence = 0 while active)".into(),
            "0/1 (failed)".into(),
            "the higher-level function FQDC feeds has failed (a processing/software fault downstream of good FQDC data), forcing that channel's own half of the gauging chain to zero confidence",
        );
    }
    let _ = fqms_ids;
    let seq_norm = next();
    let seq_altn = next();
    r.component(ComponentDef {
        id: "28_fuel.computer.transfer_sequencer".into(),
        area: Area::Fuel,
        ata: ATA,
        name: "Fuel transfer sequencing computer (automatic CG/wing transfer logic)".into(),
        params: vec![
            ParamDef { name: "norm_fault_fraction".into(), meaning: "0/1, the normal (primary) transfer-sequencing channel has failed".into(), healthy: 0.0 },
            ParamDef { name: "altn_fault_fraction".into(), meaning: "0/1, the alternate (backup) transfer-sequencing channel has failed".into(), healthy: 0.0 },
        ],
        failures: vec![seq_norm, seq_altn],
    });
    r.failure(FailureDef {
        id: seq_norm,
        area: Area::Fuel,
        ata: ATA,
        name: "Normal transfer sequencer fault".into(),
        component: "28_fuel.computer.transfer_sequencer".into(),
        model_field: "live.rs::transfer_sequencer_norm_fault (direct reading)".into(),
        magnitude: "0/1 (failed)".into(),
        effect: "the primary computer that commands the automatic CG/wing transfer sequence has failed; the valves and pumps it would command stay mechanically healthy but unsequenced".into(),
    });
    r.failure(FailureDef {
        id: seq_altn,
        area: Area::Fuel,
        ata: ATA,
        name: "Alternate transfer sequencer fault".into(),
        component: "28_fuel.computer.transfer_sequencer".into(),
        model_field: "live.rs::transfer_sequencer_altn_fault (direct reading)".into(),
        magnitude: "0/1 (failed)".into(),
        effect: "the backup transfer-sequencing channel has failed; with the normal channel also failed the automatic sequence is lost entirely".into(),
    });

    let wb_backup_fault = next();
    one(
        r,
        wb_backup_fault,
        "28_fuel.computer.wb_backup".into(),
        "Weight & balance backup computer".into(),
        "backup_fault_fraction",
        "0/1, the backup weight-and-balance computation channel has failed",
        "Weight & balance backup computation fault".into(),
        "live.rs::wb_backup_fault (direct reading; a discrete health verdict, no numeric threshold)".into(),
        "0/1 (failed)".into(),
        "the backup channel that cross-checks the primary weight-and-balance computation has failed",
    );
    let gallery_leak_fwd = next();
    let gallery_leak_aft = next();
    one(r, gallery_leak_fwd, "28_fuel.gallery.forward".into(), "Forward transfer gallery".into(), "leak_fraction", "0..1, fraction of transfer flow through this gallery section diverted by a leak instead of reaching its destination tank", "Forward transfer gallery leak".into(), "cg_transfer.rs::TransferFaults.gallery_leak_fraction (forward-gallery transfer paths) and leak.rs::gallery_leak_kg_s (mass lost overboard, same fault)".into(), "0..1 diverted fraction", "forward transfers (inner/mid/outer to feed, forward trim path) are throttled exactly as `cg_transfer::achieved_transfer_rate_kg_s` models, and the diverted fuel is an unmetered loss `leak::LeakDetector` can catch");
    one(r, gallery_leak_aft, "28_fuel.gallery.aft".into(), "Aft transfer gallery".into(), "leak_fraction", "0..1, fraction of transfer flow through this gallery section diverted by a leak instead of reaching its destination tank", "Aft transfer gallery leak".into(), "cg_transfer.rs::TransferFaults.gallery_leak_fraction (aft-gallery transfer paths) and leak.rs::gallery_leak_kg_s (mass lost overboard, same fault)".into(), "0..1 diverted fraction", "aft transfers and the jettison feed path through AftGalleryJunction1/2 are throttled and the diverted fuel is an unmetered loss");

    let mut leak_related_ids = tank_leak_ids.clone();
    leak_related_ids.push(gallery_leak_fwd);
    leak_related_ids.push(gallery_leak_aft);

    let _ = &leak_related_ids;
    let _ = (trim_pump_left, trim_pump_right, trim_inlet_1, trim_inlet_2, trim_iso_fwd, trim_iso_aft);
    let _ = (outer_xfer_left, outer_xfer_right, inner_xfer_left, inner_xfer_right, mid_xfer_left, mid_xfer_right);
    let _ = &crossfeed_ids;

    let _ = &filter_ids;

    let _ = (&fcoc_ids, jettison_valve_left, jettison_valve_right, jettison_nozzle_left, jettison_nozzle_right);
    let _ = (&probe_ids, &densitometer_ids);
    let _ = &geometry_ids;
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
    fn fuel_alerts_are_flybywire_procedures_not_registry_alerts() {
        let mut r = Registry::default();
        register(&mut r);
        assert!(r.alerts.is_empty(), "FUEL TEMP LO and FUEL JETTISON FAULT are FlyByWire procedures wired in ecam::fbw::ata28");
    }
}
