use crate::deep::api::*;

const AREA: Area = Area::EngineAccessories;

fn param(name: &str, meaning: &str, healthy: f64) -> ParamDef {
    ParamDef { name: name.to_string(), meaning: meaning.to_string(), healthy }
}

fn reg_component(r: &mut Registry, ata: u16, id: String, name: String, params: Vec<ParamDef>) {
    r.component(ComponentDef { id, area: AREA, ata, name, params, failures: Vec::new() });
}

fn reg_failure(r: &mut Registry, ata: u16, n: &mut u16, name: String, component: &str, model_field: &str, magnitude: &str, effect: &str) -> u64 {
    let id = failure_id(AREA, ata, *n);
    *n += 1;
    r.failure(FailureDef {
        id,
        area: AREA,
        ata,
        name,
        component: component.to_string(),
        model_field: model_field.to_string(),
        magnitude: magnitude.to_string(),
        effect: effect.to_string(),
    });
    id
}

pub fn register(r: &mut Registry) {
    register_fuel_system(r);
    register_ignition(r);
    register_starting(r);
    register_airflow_control(r);
    register_rotor_dynamics(r);
    register_thrust_reverser(r);
    register_eec(r);
    register_thrust_lever_channel_b(r);
    register_nacelle(r);
    register_oil_system(r);
    register_turbine_blade_damage(r);
    register_bearing_seizure(r);
}

fn register_fuel_system(r: &mut Registry) {
    const ATA: u16 = 73;
    let mut n: u16 = 1;

    for eng in 1..=4u16 {
        let lp_id = format!("73_fuel.lp_pump_{eng}");
        reg_component(
            r,
            ATA,
            lp_id.clone(),
            format!("Engine {eng} LP (boost) fuel pump"),
            vec![
                param("wear", "impeller/bearing wear, 0 healthy .. 1 seized; derates the centrifugal pressure-rise coefficient", 0.0),
                param("inlet_restriction", "inlet strainer icing/debris, 0 clear .. 1 blocked; throttles suction pressure feeding the NPSH check", 0.0),
            ],
        );
        reg_failure(
            r, ATA, &mut n,
            format!("Engine {eng} LP fuel pump wear"),
            &lp_id, "fuel::lp_pump::LpPumpFaults.wear", "0 healthy .. 1 seized",
            "Derates the pump's pressure-rise coefficient; less boost pressure reaches the HP pump inlet.",
        );
        reg_failure(
            r, ATA, &mut n,
            format!("Engine {eng} LP fuel pump inlet restriction"),
            &lp_id, "fuel::lp_pump::LpPumpFaults.inlet_restriction", "0 clear .. 1 blocked",
            "Throttles suction pressure; with hot fuel, drives NPSH below required and cavitates (flow and pressure both fall).",
        );

        let strainer_id = format!("73_fuel.strainer_{eng}");
        reg_component(
            r, ATA, strainer_id.clone(), format!("Engine {eng} fuel strainer"),
            vec![param("clog", "element blocked with debris, 0 clean .. 1 blocked; resistance grows as 1/(1-clog)^2", 0.0)],
        );
        reg_failure(
            r, ATA, &mut n,
            format!("Engine {eng} fuel strainer clog"),
            &strainer_id, "fuel::strainer::StrainerFaults.clog", "0 clean .. 1 blocked",
            "Differential pressure rises until the 12 psi bypass valve cracks (FCOM PRO-ABN-ECAM p.5787); beyond that, undelivered debris passes on to the fine filter downstream.",
        );

        let filter_id = format!("73_fuel.filter_{eng}");
        reg_component(
            r, ATA, filter_id.clone(), format!("Engine {eng} fuel filter"),
            vec![
                param("clog", "element blocked with debris/wax, 0 clean .. 1 blocked; resistance grows as 1/(1-clog)^2", 0.0),
                param("monitor_fault", "the differential-pressure monitor's own electronics/switch, 0 healthy .. 1 dead, independent of clog (`E-ENG-DESIGN.md` Pattern 16)", 0.0),
            ],
        );
        reg_failure(
            r, ATA, &mut n,
            format!("Engine {eng} fuel filter clog"),
            &filter_id, "fuel::filter::FilterFaults.clog", "0 clean .. 1 blocked",
            "Differential pressure rises until the 35 psi bypass valve cracks (reaches the cockpit as the real `ENG n FUEL FILTER CLOGGED` FbwProc, deep/ecam/fbw/generated.rs); past 60 psi the bypass valve opens and reaches the cockpit as the real `ENG n FUEL SYS CONTAMINATION` FbwProc, deep/ecam/fbw/ata70.rs.",
        );
        reg_failure(
            r, ATA, &mut n,
            format!("Engine {eng} fuel filter monitor fault"),
            &filter_id, "fuel::filter::FilterFaults.monitor_fault", "0 healthy .. 1 dead",
            "The bypass-warning monitor itself reads faulted independent of the element's real clog state (`ENG n FUEL FILTER MONITORING FAULT`, FCOM PRO-ABN-ECAM p.5784: \"The fuel filter is no longer monitored\").",
        );

        let hp_id = format!("73_fuel.hp_pump_{eng}");
        reg_component(
            r, ATA, hp_id.clone(), format!("Engine {eng} HP (gear) fuel pump"),
            vec![
                param("wear", "gear/bearing wear, 0 healthy .. 1 worn; internal slip flow grows as 1/(1-wear)^2 at a given discharge pressure", 0.0),
                param("inlet_starvation", "fuel not reaching the gear pockets (fed forward from LP pump cavitation), 0 none .. 1 total", 0.0),
            ],
        );
        reg_failure(
            r, ATA, &mut n,
            format!("Engine {eng} HP fuel pump wear"),
            &hp_id, "fuel::hp_pump::HpPumpFaults.wear", "0 healthy .. 1 worn",
            "Internal slip flow grows as 1/(1-wear)^2; delivered flow falls short of the theoretical displacement flow, starving the FMU and reaching the cockpit through the real `ENG n THRUST LOSS` FbwProc (deep/ecam/fbw/ata70.rs) once the shortfall is severe enough to matter.",
        );
        reg_failure(
            r, ATA, &mut n,
            format!("Engine {eng} HP fuel pump inlet starvation"),
            &hp_id, "fuel::hp_pump::HpPumpFaults.inlet_starvation", "0 none .. 1 total",
            "Caps delivered flow directly, regardless of slip -- there is simply not enough fuel arriving to fill the gear pockets; same real `ENG n THRUST LOSS` path as the wear fault above.",
        );

        let fmu_id = format!("73_fuel.fmu_{eng}");
        reg_component(
            r, ATA, fmu_id.clone(), format!("Engine {eng} fuel metering unit"),
            vec![
                param("valve_sticking", "metering valve seizure/fouling, 0 free .. 1 seized; scales actuator slew rate to zero", 0.0),
                param("spill_stuck_open", "spill valve stuck open, 0 healthy .. 1 fully open; collapses the regulated differential toward zero", 0.0),
                param("spill_stuck_closed", "spill valve stuck closed, 0 healthy .. 1 fully closed; differential rises toward raw HP pump pressure", 0.0),
            ],
        );
        reg_failure(
            r, ATA, &mut n,
            format!("Engine {eng} FMU metering valve sticking"),
            &fmu_id, "fuel::fmu::FmuFaults.valve_sticking", "0 free .. 1 seized",
            "Actuator slew rate falls to zero; the valve tracks the commanded flow ever more slowly and, fully stuck, freezes in place; a large enough error reaches the cockpit through the real `ENG n THRUST LOSS` FbwProc (deep/ecam/fbw/ata70.rs, on A32NX_ENG_n_THRUST_ABNORMAL/A32NX_ENG_n_FMU_FAULT).",
        );
        reg_failure(
            r, ATA, &mut n,
            format!("Engine {eng} FMU spill valve stuck open"),
            &fmu_id, "fuel::fmu::FmuFaults.spill_stuck_open", "0 healthy .. 1 fully open",
            "The regulated differential collapses toward zero; metered flow starves even at full valve area (flameout risk); same real `ENG n THRUST LOSS` path as the metering valve fault above.",
        );
        reg_failure(
            r, ATA, &mut n,
            format!("Engine {eng} FMU spill valve stuck closed"),
            &fmu_id, "fuel::fmu::FmuFaults.spill_stuck_closed", "0 healthy .. 1 fully closed",
            "The differential rises toward the raw HP pump discharge pressure; over-meters flow for a given commanded area; same real `ENG n THRUST LOSS` path as the metering valve fault above.",
        );

        let sov_id = format!("73_fuel.hp_sov_{eng}");
        reg_component(
            r, ATA, sov_id.clone(), format!("Engine {eng} HP fuel shut-off valve"),
            vec![param("stuck", "mechanically stuck, 0 free .. 1 seized; travel rate scaled to zero", 0.0)],
        );
        reg_failure(
            r, ATA, &mut n,
            format!("Engine {eng} HP fuel shut-off valve stuck"),
            &sov_id, "fuel::shutoff_valve::ShutoffValveFaults.stuck", "0 free .. 1 seized",
            "Travel rate scaled to zero; stuck open defeats a fire-handle shutdown, stuck closed flames the engine out and blocks a restart; reaches the cockpit through the real `ENG n HP FUEL VLV FAULT` FbwProc (deep/ecam/fbw/ata70.rs, FCOM PRO-ABN-ECAM p.5789).",
        );

        let ft_id = format!("73_fuel.flow_transmitter_{eng}");
        reg_component(
            r, ATA, ft_id.clone(), format!("Engine {eng} fuel flow transmitter"),
            vec![
                param("channel_a_bias", "pick-off A miscalibration, 0 accurate .. 1 max bias (up to 5 kg/s)", 0.0),
                param("channel_a_frozen", "pick-off A frozen, 0 live .. 1 fully frozen", 0.0),
                param("channel_b_bias", "pick-off B miscalibration, 0 accurate .. 1 max bias (up to 5 kg/s)", 0.0),
                param("channel_b_frozen", "pick-off B frozen, 0 live .. 1 fully frozen", 0.0),
            ],
        );
        reg_failure(r, ATA, &mut n, format!("Engine {eng} fuel flow transmitter channel A bias"), &ft_id, "fuel::flow_transmitter::PickoffFaults.bias_frac_of_design (channel A)", "0 accurate .. 1 max bias", "Offsets channel A's reading up to 5 kg/s independent of channel B; no FCOM alert exists for a channel disagree, but the biased reading reaches the cockpit directly through the published fuel-flow gauge value it corrupts.");
        reg_failure(r, ATA, &mut n, format!("Engine {eng} fuel flow transmitter channel A frozen"), &ft_id, "fuel::flow_transmitter::PickoffFaults.frozen (channel A)", "0 live .. 1 frozen", "Channel A stops updating and holds its last reading while channel B keeps tracking; same real gauge-reading path as the bias fault above.");
        reg_failure(r, ATA, &mut n, format!("Engine {eng} fuel flow transmitter channel B bias"), &ft_id, "fuel::flow_transmitter::PickoffFaults.bias_frac_of_design (channel B)", "0 accurate .. 1 max bias", "Offsets channel B's reading up to 5 kg/s independent of channel A; same real gauge-reading path as the channel A bias fault above.");
        reg_failure(r, ATA, &mut n, format!("Engine {eng} fuel flow transmitter channel B frozen"), &ft_id, "fuel::flow_transmitter::PickoffFaults.frozen (channel B)", "0 live .. 1 frozen", "Channel B stops updating and holds its last reading while channel A keeps tracking; same real gauge-reading path as the channel A bias fault above.");

        let manifold_id = format!("73_fuel.manifold_{eng}");
        let mut manifold_params = Vec::new();
        for g in 0..super::fuel::manifold::NUM_GROUPS {
            manifold_params.push(param(&format!("group_{g}_blockage"), "nozzle group coking/blockage, 0 clean .. 1 blocked", 0.0));
        }
        reg_component(r, ATA, manifold_id.clone(), format!("Engine {eng} burner manifold and nozzles"), manifold_params);
        for g in 0..super::fuel::manifold::NUM_GROUPS {
            reg_failure(
                r, ATA, &mut n,
                format!("Engine {eng} burner nozzle group {g} coking"),
                &manifold_id, &format!("fuel::manifold::ManifoldFaults.group_blockage[{g}]"), "0 clean .. 1 blocked",
                "Shrinks that group's orifice area; the shared manifold pressure rises until the other groups pass the difference, raising hot_streak_severity for the combustor to see as a local hot streak; no FCOM alert exists for a nozzle imbalance, but a severe enough streak reaches the cockpit through the real `ENG n EGT OVER LIMIT` FbwProc (deep/ecam/fbw/ata70.rs) via the TGT rise it causes.",
            );
        }
    }
}

fn register_ignition(r: &mut Registry) {
    const ATA: u16 = 74;
    let mut n: u16 = 1;

    for eng in 1..=4u16 {
        let mut chain_failures = Vec::new();
        for chain in ["a", "b"] {
            let exciter_id = format!("74_ignition.exciter_{chain}_{eng}");
            reg_component(
                r, ATA, exciter_id.clone(), format!("Engine {eng} ignition exciter {}", chain.to_uppercase()),
                vec![param("exciter_failure", "capacitor charging circuit failure, 0 healthy .. 1 dead; scales RC charge rate to zero", 0.0)],
            );
            let exciter_fail = reg_failure(
                r, ATA, &mut n,
                format!("Engine {eng} ignition exciter {} failure", chain.to_uppercase()),
                &exciter_id, &format!("ignition::IgnitionFaults.exciter_{chain}_failure"), "0 healthy .. 1 dead",
                "Charges more slowly (spark rate falls); fully dead, that chain never reaches trigger voltage and never sparks.",
            );

            let igniter_id = format!("74_ignition.igniter_{chain}_{eng}");
            reg_component(
                r, ATA, igniter_id.clone(), format!("Engine {eng} igniter plug {}", chain.to_uppercase()),
                vec![param("erosion", "spark-gap erosion, 0 new .. 1 fully eroded; raises the breakdown voltage the gap needs", 0.0)],
            );
            let igniter_erosion = reg_failure(
                r, ATA, &mut n,
                format!("Engine {eng} igniter plug {} erosion", chain.to_uppercase()),
                &igniter_id, &format!("ignition::IgnitionFaults.igniter_{chain}_erosion"), "0 new .. 1 fully eroded",
                "Raises the gap's breakdown voltage; once it exceeds the exciter's fixed peak output, that chain stops firing outright.",
            );
            chain_failures.push(exciter_fail);
            chain_failures.push(igniter_erosion);
        }
        let _ = &chain_failures;
    }
}

fn register_starting(r: &mut Registry) {
    const ATA: u16 = 80;
    let mut n: u16 = 1;

    for eng in 1..=4u16 {
        let sav_id = format!("80_start.air_valve_{eng}");
        reg_component(
            r, ATA, sav_id.clone(), format!("Engine {eng} starter air valve"),
            vec![param("stuck", "mechanically stuck, 0 free .. 1 seized; travel rate scaled to zero, reads as stuck-open or stuck-closed depending on the commanded direction at the time", 0.0)],
        );
        let sav_stuck = reg_failure(
            r, ATA, &mut n,
            format!("Engine {eng} starter air valve stuck"),
            &sav_id, "starting::air_valve::AirValveFaults.stuck", "0 free .. 1 seized",
            "Travel rate scaled to zero; stuck open keeps driving the starter after the normal handoff speed (see the ATS disintegration path), stuck closed gives no start air at all.",
        );
        let _ = sav_stuck;

        let ats_id = format!("80_start.ats_{eng}");
        reg_component(
            r, ATA, ats_id.clone(), format!("Engine {eng} air turbine starter"),
            vec![
                param("clutch_fails_to_engage", "sprag fails to engage, 0 full transmission .. 1 none; a hung start with a spinning but useless turbine", 0.0),
                param("clutch_fails_to_disengage", "sprag fails to freewheel once overtaken, 0 healthy .. 1 fully coupled; drags the spool and, combined with a sustained high spool speed, can disintegrate the starter", 0.0),
            ],
        );
        reg_failure(
            r, ATA, &mut n,
            format!("Engine {eng} starter clutch fails to engage"),
            &ats_id, "starting::turbine::AtsFaults.clutch_fails_to_engage", "0 full .. 1 none",
            "Reduces torque transmitted to the spool during cranking; a hung start with the turbine itself spinning normally.",
        );
        let clutch_disengage = reg_failure(
            r, ATA, &mut n,
            format!("Engine {eng} starter clutch fails to disengage"),
            &ats_id, "starting::turbine::AtsFaults.clutch_fails_to_disengage", "0 healthy .. 1 fully coupled",
            "Couples spool speed back into the starter rotor past the turbine's own free speed; drags the spool, and if the spool later reaches the FADEC's own N3 overspeed setpoint with the clutch still coupled, disintegrates the starter (starting::turbine::AirTurbineStarter.disintegrated).",
        );
        let _ = clutch_disengage;
    }
}

fn register_airflow_control(r: &mut Registry) {
    register_vsv(r);
    register_handling_bleeds(r);
}

fn register_vsv(r: &mut Registry) {
    const ATA: u16 = 72;
    let mut n: u16 = 1;

    for eng in 1..=4u16 {
        let vsv_id = format!("72_air.vsv_{eng}");
        reg_component(
            r, ATA, vsv_id.clone(), format!("Engine {eng} IP compressor variable stator vanes"),
            vec![
                param("jam", "actuator jam, 0 free .. 1 seized; rising stiction leaves a permanent schedule error that grows with the fraction, pinning at full seizure (never moves) at 1", 0.0),
                param("rigging_bias_deg", "feedback/rigging offset, signed degrees (not a 0..1 fraction); a persistent angle error even with a healthy actuator", 0.0),
            ],
        );
        let jam = reg_failure(
            r, ATA, &mut n,
            format!("Engine {eng} IP VSV actuator jam"),
            &vsv_id, "airflow_control::vsv::VsvFaults.jam", "0 free .. 1 seized",
            "Rising stiction leaves the ring unable to close the full schedule error; the resulting permanent, magnitude-scaled schedule error costs stall margin (airflow_control::vsv::VsvState.stall_margin_delta_pct) for the gas path's compressor model to apply.",
        );
        let rigging = reg_failure(
            r, ATA, &mut n,
            format!("Engine {eng} IP VSV rigging error"),
            &vsv_id, "airflow_control::vsv::VsvFaults.rigging_bias_deg", "signed degrees, 0 = none",
            "A healthy, fully responsive actuator still settles off the true schedule by the bias amount, costing stall margin exactly as a jam does.",
        );
        let _ = (jam, rigging);
    }
}

fn register_handling_bleeds(r: &mut Registry) {
    const ATA: u16 = 75;
    let mut n: u16 = 1;

    for eng in 1..=4u16 {
        for (spool, key) in [("IP", "ip"), ("HP", "hp")] {
            let id = format!("75_air.{key}_handling_bleed_{eng}");
            reg_component(
                r, ATA, id.clone(), format!("Engine {eng} {spool} handling bleed valve"),
                vec![param("jam", "mechanically jammed, 0 free .. 1 seized; reads as jammed-open or jammed-closed depending on the commanded direction at the time", 0.0)],
            );
            let jam = reg_failure(
                r, ATA, &mut n,
                format!("Engine {eng} {spool} handling bleed valve jam"),
                &id, &format!("airflow_control::bleed_valve::BleedValveFaults.jam ({key})"), "0 free .. 1 seized",
                "Jammed open at high power bleeds core air the engine needs (thrust/fuel-air-ratio penalty); jammed closed at low power loses the stall margin the valve exists to provide (airflow_control::bleed_valve::BleedValveState.stall_margin_delta_pct).",
            );
            let _ = jam;
        }
    }
}

fn register_rotor_dynamics(r: &mut Registry) {
    const ATA: u16 = 77;
    let mut n: u16 = 1;

    for eng in 1..=4u16 {
        for (spool, key, var_suffix) in [("N1", "fan", "N1"), ("N2", "ip", "N2"), ("N3", "hp", "N3")] {
            let id = format!("77_vib.{key}_rotor_{eng}");
            reg_component(
                r, ATA, id.clone(), format!("Engine {eng} {spool} rotor imbalance"),
                vec![
                    param("blade_loss_frac", "eccentric mass from a lost blade, 0 none .. 1 a full single-blade-loss-equivalent unbalance", 0.0),
                    param("ice_frac", "eccentric mass from asymmetric ice accretion, same 0..1 scale", 0.0),
                    param("bird_strike_frac", "eccentric mass from bird-strike impact damage, same 0..1 scale", 0.0),
                ],
            );
            reg_failure(r, ATA, &mut n, format!("Engine {eng} {spool} blade loss"), &id, &format!("rotor_dynamics::imbalance::ImbalanceFaults.blade_loss_frac ({key})"), "0 none .. 1 full blade loss", "Adds eccentric mass; synchronous vibration rises with the square of shaft speed (rotor_dynamics::imbalance::VibrationState.index), read live on the SD ENG page's VIB N1/N2/N3 (no automatic ECAM alert: FCOM `ENG HI VIBRATIONS` is a crew-selected ABN PROC menu item, phases None).");
            reg_failure(r, ATA, &mut n, format!("Engine {eng} {spool} asymmetric ice accretion"), &id, &format!("rotor_dynamics::imbalance::ImbalanceFaults.ice_frac ({key})"), "0 none .. 1 full", "Adds eccentric mass the same way a blade loss does, at a smaller typical magnitude.");
            reg_failure(r, ATA, &mut n, format!("Engine {eng} {spool} bird-strike imbalance"), &id, &format!("rotor_dynamics::imbalance::ImbalanceFaults.bird_strike_frac ({key})"), "0 none .. 1 full", "Adds eccentric mass from impact damage, same mechanism as blade loss/ice.");
            let _ = var_suffix;
        }

        for (key, display, chamber, var_suffix) in [
            ("fan_front", "FAN FRONT BRG", "Front", "N1"),
            ("ip_front", "IP FRONT BRG", "Front", "N2"),
            ("hp_turbine", "HP TURBINE BRG", "HP/IP", "N3"),
            ("ip_turbine", "IP TURBINE BRG", "HP/IP", "N2"),
            ("lp_turbine_rear", "LP TURBINE REAR BRG", "Tail", "N1"),
        ] {
            let bearing_id = format!("77_vib.bearing_{key}_{eng}");
            reg_component(
                r, ATA, bearing_id.clone(), format!("Engine {eng} {display} ({chamber} oil chamber)"),
                vec![
                    param("outer_race_spall", "outer race spall, 0 none .. 1 severe; rings at BPFO", 0.0),
                    param("inner_race_spall", "inner race spall, 0 none .. 1 severe; rings at BPFI", 0.0),
                    param("rolling_element_spall", "rolling-element spall, 0 none .. 1 severe; rings at BSF", 0.0),
                    param("cage_wear", "cage/separator wear, 0 none .. 1 severe; rings at FTF", 0.0),
                ],
            );
            reg_failure(r, ATA, &mut n, format!("Engine {eng} {display} outer race spall"), &bearing_id, &format!("rotor_dynamics::bearings::EngineBearingFaults.faults[{key}].outer_race_spall"), "0 none .. 1 severe", "Vibration at this bearing's outer-race ball-pass frequency (BPFO); accumulates debris that feeds the oil system's chip detector (rotor_dynamics::bearings::BearingState.chip_detected), reaching the cockpit through FCOM `ENG n OIL CHIP DETECTED` (FwsAbnormalSensed.ts oilChipDetected, FlyByWire id 701800077-080) once a bearing's own A32NX_ENG_n_<BEARING>_DEBRIS_G crosses the FwsCore.ts threshold.");
            reg_failure(r, ATA, &mut n, format!("Engine {eng} {display} inner race spall"), &bearing_id, &format!("rotor_dynamics::bearings::EngineBearingFaults.faults[{key}].inner_race_spall"), "0 none .. 1 severe", "Vibration at BPFI, the highest of the four defect frequencies for this geometry; also feeds the chip detector, same cockpit path as outer_race_spall.");
            reg_failure(r, ATA, &mut n, format!("Engine {eng} {display} rolling-element spall"), &bearing_id, &format!("rotor_dynamics::bearings::EngineBearingFaults.faults[{key}].rolling_element_spall"), "0 none .. 1 severe", "Vibration at the ball-spin frequency (BSF); also feeds the chip detector, same cockpit path as outer_race_spall.");
            reg_failure(r, ATA, &mut n, format!("Engine {eng} {display} cage wear"), &bearing_id, &format!("rotor_dynamics::bearings::EngineBearingFaults.faults[{key}].cage_wear"), "0 none .. 1 severe", "Vibration at the fundamental train frequency (FTF), the lowest of the four; also feeds the chip detector, same cockpit path as outer_race_spall.");
            let _ = var_suffix;
        }
    }
}

fn register_thrust_reverser(r: &mut Registry) {
    const ATA: u16 = 78;
    let mut n: u16 = 1;

    for eng in [2u16, 3u16] {
        let id = format!("78_rev.reverser_{eng}");
        reg_component(
            r, ATA, id.clone(), format!("Engine {eng} thrust reverser"),
            vec![
                param("lock_a_fails_to_hold", "primary lock cannot restrain even when it should, 0 healthy .. 1 total", 0.0),
                param("lock_a_jam", "primary lock jammed engaged, cannot release when commanded, 0 free .. 1 seized", 0.0),
                param("lock_b_fails_to_hold", "secondary lock (hydraulic isolation valve) cannot restrain, 0 healthy .. 1 total", 0.0),
                param("lock_b_jam", "secondary lock jammed engaged, 0 free .. 1 seized", 0.0),
                param("lock_c_fails_to_hold", "tertiary (sleeve mechanical) lock cannot restrain, 0 healthy .. 1 total", 0.0),
                param("lock_c_jam", "tertiary lock jammed engaged, 0 free .. 1 seized", 0.0),
                param("actuator_jam", "actuator ram seized, 0 free .. 1 solid; blocks both deploy and stow", 0.0),
                param("control_fault", "the reverser's own EEC-side control loop, 0 healthy .. 1 dead; leaves the actuator itself free but uncommandable, so the sleeve holds its last position (`E-ENG-DESIGN.md` Pattern 24)", 0.0),
            ],
        );
        let a_hold = reg_failure(r, ATA, &mut n, format!("Engine {eng} reverser primary lock fails to hold"), &id, "thrust_reverser::LockFaults.fails_to_hold (lock_a)", "0 healthy .. 1 total", "One of three independent restraints; alone, the other two still prevent deployment (an OR across all three).");
        let a_jam = reg_failure(r, ATA, &mut n, format!("Engine {eng} reverser primary lock jam"), &id, "thrust_reverser::LockFaults.jam (lock_a)", "0 free .. 1 seized", "Blocks legitimate deployment outright (every lock must release, an AND across all three) -- the safe-direction failure.");
        let b_hold = reg_failure(r, ATA, &mut n, format!("Engine {eng} reverser secondary lock fails to hold"), &id, "thrust_reverser::LockFaults.fails_to_hold (lock_b)", "0 healthy .. 1 total", "Same redundancy logic as the primary lock.");
        let b_jam = reg_failure(r, ATA, &mut n, format!("Engine {eng} reverser secondary lock jam"), &id, "thrust_reverser::LockFaults.jam (lock_b)", "0 free .. 1 seized", "Blocks legitimate deployment, same as the primary lock's jam.");
        let c_hold = reg_failure(r, ATA, &mut n, format!("Engine {eng} reverser tertiary lock fails to hold"), &id, "thrust_reverser::LockFaults.fails_to_hold (lock_c)", "0 healthy .. 1 total", "Same redundancy logic; all three failing this way together is what an uncommanded deployment actually requires (thrust_reverser::ReverserState.uncommanded_deployment).");
        let c_jam = reg_failure(r, ATA, &mut n, format!("Engine {eng} reverser tertiary lock jam"), &id, "thrust_reverser::LockFaults.jam (lock_c)", "0 free .. 1 seized", "Blocks legitimate deployment, same as the other two locks' jams.");
        let act_jam = reg_failure(r, ATA, &mut n, format!("Engine {eng} reverser actuator jam"), &id, "thrust_reverser::ReverserFaults.actuator_jam", "0 free .. 1 solid", "Blocks both deploy and stow motion regardless of lock state -- the direct fails-to-deploy/fails-to-stow fault.");
        reg_failure(r, ATA, &mut n, format!("Engine {eng} reverser control fault"), &id, "thrust_reverser::ReverserFaults.control_fault", "0 healthy .. 1 dead", "Freezes the sleeve at its last position, the same functional effect as `actuator_jam` from a control-loop (not physical) cause -- `ENG n REVERSER CTL FAULT` (`E-ENG-DESIGN.md` Pattern 24).");

        let _ = (a_hold, b_hold, c_hold, a_jam, b_jam, c_jam, act_jam);
    }
}

fn register_eec(r: &mut Registry) {
    const ATA: u16 = 73;
    let mut n: u16 = 500;

    for eng in 1..=4u16 {
        let id = format!("73_eec.channels_{eng}");
        let mut params = vec![
            param("channel_a_fault", "channel A electronics failure, 0 healthy .. 1 dead; a dead channel is ignored entirely by selection, not merely biased", 0.0),
            param("channel_b_fault", "channel B electronics failure, 0 healthy .. 1 dead", 0.0),
        ];
        for p in ["N1", "N2", "N3", "TGT", "P30"] {
            params.push(param(&format!("{p}_sensor_a"), "channel A sensor bias/frozen fault for this parameter, 0 healthy", 0.0));
            params.push(param(&format!("{p}_sensor_b"), "channel B sensor bias/frozen fault for this parameter, 0 healthy", 0.0));
        }
        params.push(param("backup_oil_temp_probe_bias", "the EEC's own backup oil-temperature probe (trend/logging channel), 0 healthy .. 1 max bias, independent of the primary reading (`E-ENG-DESIGN.md` Pattern 23)", 0.0));
        reg_component(r, ATA, id.clone(), format!("Engine {eng} EEC (dual channel)"), params);

        reg_failure(r, ATA, &mut n, format!("Engine {eng} EEC channel A fault"), &id, "eec::EecFaults.channel_a_fault", "0 healthy .. 1 dead", "Hands control to channel B if it is healthy; both dead leaves no valid EEC channel (eec::ActiveChannel::None); reaches the cockpit through the real `ENG n FADEC SYS FAULT` (single channel dead) and `ENG n FADEC FAULT` (both channels dead) FbwProcs, deep/ecam/fbw/ata70.rs.");
        reg_failure(r, ATA, &mut n, format!("Engine {eng} EEC channel B fault"), &id, "eec::EecFaults.channel_b_fault", "0 healthy .. 1 dead", "Symmetric with channel A's fault; same real FADEC SYS FAULT/FADEC FAULT path.");

        for p in ["N1", "N2", "N3", "TGT", "P30"] {
            reg_failure(r, ATA, &mut n, format!("Engine {eng} EEC {p} sensor fault, channel A"), &id, &format!("eec::EecFaults.sensor_a[{p}] (bias/frozen)"), "0 healthy .. 1 max bias or fully frozen", "Biases or freezes channel A's reading for this parameter; flagged against channel B once the disagreement exceeds this parameter's threshold (eec::EecState.disagree); reaches the cockpit through the real `ENG n SENSOR FAULT` FbwProc (deep/ecam/fbw/generated.rs for engines 1-2, FwsCore.ts for engines 3-4).");
            reg_failure(r, ATA, &mut n, format!("Engine {eng} EEC {p} sensor fault, channel B"), &id, &format!("eec::EecFaults.sensor_b[{p}] (bias/frozen)"), "0 healthy .. 1 max bias or fully frozen", "Symmetric with channel A's sensor fault for the same parameter; same real SENSOR FAULT path.");
        }

        reg_failure(r, ATA, &mut n, format!("Engine {eng} EEC backup oil-temperature probe bias"), &id, "eec::EecFaults.backup_oil_temp_probe_bias", "0 healthy .. 1 max bias", "Biases the EEC's own backup (trend/logging) oil-temperature reading against the primary; a real, non-controlling internal-sensor fault, never fed back into thrust control.");
    }
}

fn register_thrust_lever_channel_b(r: &mut Registry) {
    const ATA: u16 = 76;
    let mut n: u16 = 1;
    for eng in 1..=4u16 {
        let id = format!("76_ctl.tla_channel_b_{eng}");
        reg_component(
            r, ATA, id.clone(), format!("Engine {eng} thrust lever position transducer, channel B"),
            vec![param("bias", "channel B position bias against FlyByWire's own single TLA channel, 0 healthy .. 1 max bias", 0.0)],
        );
        reg_failure(r, ATA, &mut n, format!("Engine {eng} thrust lever channel B bias"), &id, "TlaChannelBFaults.bias", "0 healthy .. 1 max bias", "Biases the second (channel B) thrust-lever position reading against FlyByWire's own single channel; flagged once the disagreement exceeds THR_LEVER_DISAGREE_TOLERANCE_DEG (`ENG n THR LEVER FAULT`, `E-ENG-DESIGN.md` Pattern 31).");
    }
}

fn register_nacelle(r: &mut Registry) {
    register_nacelle_anti_ice(r);
    register_nacelle_ventilation(r);
    register_nacelle_fire_detection(r);
}

fn register_nacelle_anti_ice(r: &mut Registry) {
    const ATA: u16 = 30;
    let mut n: u16 = 1;
    for eng in 1..=4u16 {
        let id = format!("30_ice.nacelle_valve_{eng}");
        reg_component(
            r, ATA, id.clone(), format!("Engine {eng} nacelle anti-ice valve"),
            vec![param("stuck", "mechanically stuck, 0 free .. 1 seized; reads as stuck-open (bleed air waste) or stuck-closed (no ice protection) depending on the commanded direction", 0.0)],
        );
        let stuck = reg_failure(r, ATA, &mut n, format!("Engine {eng} nacelle anti-ice valve stuck"), &id, "nacelle::anti_ice::AntiIceValveFaults.stuck", "0 free .. 1 seized", "Stuck closed leaves the cowl lip at ambient with no ice protection; stuck open wastes bleed air after icing conditions clear.");
        r.alert(
            EcamAlert::new(&format!("ENG_{eng}_ANTI_ICE_VLV_CLOSED"), ATA, &format!("A-ICE ENG {eng} VLV CLOSED"), Level::Caution, all(vec![var(&format!("A32NX_ENG_{eng}_ANTI_ICE_DISAGREE")).on(), var(&format!("A32NX_ENG_{eng}_ANTI_ICE_POSITION")).le(0.5)]))
                .confirm(5.0)
                .status_line(&format!("ENG {eng} ANTI ICE"))
                .inop_sys(&format!("ENG {eng} ANTI ICE"))
                .raised_by(&[stuck]),
        );
        r.alert(
            EcamAlert::new(&format!("ENG_{eng}_ANTI_ICE_VLV_OPEN"), ATA, &format!("A-ICE ENG {eng} VLV OPEN"), Level::Caution, all(vec![var(&format!("A32NX_ENG_{eng}_ANTI_ICE_DISAGREE")).on(), var(&format!("A32NX_ENG_{eng}_ANTI_ICE_POSITION")).gt(0.5)]))
                .confirm(5.0)
                .status_line(&format!("ENG {eng} ANTI ICE"))
                .inop_sys(&format!("ENG {eng} ANTI ICE"))
                .raised_by(&[stuck]),
        );
    }
}

fn register_nacelle_ventilation(r: &mut Registry) {
    const ATA: u16 = 71;
    let mut n: u16 = 1;
    for eng in 1..=4u16 {
        let id = format!("71_pwr.nacelle_ventilation_{eng}");
        reg_component(
            r, ATA, id.clone(), format!("Engine {eng} nacelle ventilation"),
            vec![
                param("scoop_blockage", "inlet/outlet scoop blocked (ice, debris), 0 clear .. 1 fully blocked; derates the ram-air term", 0.0),
                param("eductor_blockage", "eductor line blocked or leaking, 0 healthy .. 1 fully lost; derates the bleed-driven term", 0.0),
            ],
        );
        let scoop = reg_failure(r, ATA, &mut n, format!("Engine {eng} nacelle scoop blockage"), &id, "nacelle::ventilation::VentilationFaults.scoop_blockage", "0 clear .. 1 blocked", "Derates the ram-air ventilation term; matters most in cruise where ram dominates.");
        let eductor = reg_failure(r, ATA, &mut n, format!("Engine {eng} nacelle eductor blockage"), &id, "nacelle::ventilation::VentilationFaults.eductor_blockage", "0 healthy .. 1 lost", "Derates the bleed-driven ventilation term; matters most on the ground where ram gives nothing, and can reintroduce a vapour-accumulation risk.");
        let _ = (scoop, eductor);
    }
}

fn register_nacelle_fire_detection(r: &mut Registry) {
    const ATA: u16 = 26;
    let mut n: u16 = 1;
    for eng in 1..=4u16 {
        for (zone_name, key) in [("CORE", "core"), ("FAN", "fan")] {
            let id = format!("26_fire.zone_{key}_{eng}");
            reg_component(
                r, ATA, id.clone(), format!("Engine {eng} {zone_name} zone fire detection loops"),
                vec![
                    param("loop_a_fails_to_detect", "loop A depressurised/broken, 0 healthy .. 1 never trips regardless of temperature", 0.0),
                    param("loop_a_false_trip", "loop A chafed/shorted, 0 healthy .. 1 always trips regardless of temperature", 0.0),
                    param("loop_b_fails_to_detect", "loop B depressurised/broken, same scale", 0.0),
                    param("loop_b_false_trip", "loop B chafed/shorted, same scale", 0.0),
                ],
            );
            let a_fail = reg_failure(r, ATA, &mut n, format!("Engine {eng} {zone_name} zone fire loop A fails to detect"), &id, &format!("nacelle::fire_detection::LoopFaults.fails_to_detect (loop A, {key})"), "0 healthy .. 1 never trips", "The other loop alone cannot confirm a fire (both must agree); flagged as a loop disagree if the zone is genuinely hot.");
            let a_false = reg_failure(r, ATA, &mut n, format!("Engine {eng} {zone_name} zone fire loop A false trip"), &id, &format!("nacelle::fire_detection::LoopFaults.false_trip (loop A, {key})"), "0 healthy .. 1 always trips", "A lone false trip cannot confirm a fire in a cool zone; flagged as a loop disagree.");
            let b_fail = reg_failure(r, ATA, &mut n, format!("Engine {eng} {zone_name} zone fire loop B fails to detect"), &id, &format!("nacelle::fire_detection::LoopFaults.fails_to_detect (loop B, {key})"), "0 healthy .. 1 never trips", "Symmetric with loop A's failure to detect.");
            let b_false = reg_failure(r, ATA, &mut n, format!("Engine {eng} {zone_name} zone fire loop B false trip"), &id, &format!("nacelle::fire_detection::LoopFaults.false_trip (loop B, {key})"), "0 healthy .. 1 always trips", "Symmetric with loop A's false trip.");
            let _ = (a_fail, a_false, b_fail, b_false);
        }
    }
}

fn register_oil_system(r: &mut Registry) {
    const ATA: u16 = 79;
    let mut n: u16 = 1;
    let mut leak_ids = [0u64; 4];

    for eng in 1..=4u16 {
        let id = format!("79_oil.leak_{eng}");
        reg_component(
            r, ATA, id.clone(), format!("Engine {eng} oil feed gallery"),
            vec![param("leak_frac", "hole in the pressurised feed gallery, 0 sound .. 1 a leak that drains the tank in ~6 minutes of running at the reference gallery pressure (physics::engine::oil's own LEAK_FULL_DRAIN_S)", 0.0)],
        );
        let leak = reg_failure(
            r, ATA, &mut n,
            format!("Engine {eng} oil leak"),
            &id, "physics::engine::oil::OilFaults.leak", "0 sound .. 1 full leak",
            "An orifice in the pressurised gallery: flow out goes as the square root of gallery pressure, so quantity falls fastest at high power and stops once the engine is shut down. Pressure itself barely moves until the falling level uncovers the pump's inlet, then follows the quantity down (physics::engine::oil::OilSystem) -- published as A32NX_ENG_n_GASPATH_OIL_QUANTITY_FRAC/_GASPATH_OIL_PRESS_PSI; no FCOM alert exists for a low-quantity reading by itself, but a severe enough leak starves pressure past 25 psi and reaches the cockpit through the real `ENG n OIL PRESS LO` FbwProc (deep/ecam/fbw/ata70.rs).",
        );
        leak_ids[(eng - 1) as usize] = leak;
    }

    for eng in 1..=4u16 {
        let pump_id = format!("79_oil.pump_{eng}");
        reg_component(
            r, ATA, pump_id.clone(), format!("Engine {eng} oil pressure pump"),
            vec![param("wear", "gear-pump wear/internal damage, 0 healthy .. 1 failed; derates delivery as Surroundings.pump_fraction", 0.0)],
        );
        reg_failure(
            r, ATA, &mut n,
            format!("Engine {eng} oil pump fault"),
            &pump_id, "physics::engine::oil::Surroundings.pump_fraction (pump wear)", "0 healthy .. 1 no delivery",
            "A worn or internally damaged gear pump delivers a falling fraction of its design flow at a given N3, so jet flow and gallery pressure fall together with it -- unlike the leak above, indicated quantity barely moves, because nothing is pouring overboard; no FCOM alert exists for an oil-pump fault specifically, but the falling pressure is the published A32NX_ENG_n_GASPATH_OIL_PRESS_PSI gauge value and, past 25 psi, reaches the cockpit through the real `ENG n OIL PRESS LO` FbwProc (deep/ecam/fbw/ata70.rs).",
        );
    }
    let _ = &leak_ids;
}

fn register_bearing_seizure(r: &mut Registry) {
    const ATA: u16 = 72;
    let mut n: u16 = 901;

    for eng in 1..=4u16 {
        let id = format!("72_eng.bearing_seizure_{eng}");
        reg_component(
            r, ATA, id.clone(), format!("Engine {eng} main bearing seizure"),
            vec![param("seizure_frac", "a main bearing locking solid, 0 free .. 1 fully seized; drives turbine/compressor efficiency to the gas path's own floor", 0.0)],
        );
        reg_failure(
            r, ATA, &mut n,
            format!("Engine {eng} main bearing seizure"),
            &id, "engine_accessories::live.bearing_seizure_frac", "0 free .. 1 fully seized",
            "A locked main bearing stops that shaft turning freely: the turbine and compressor stages it carries lose nearly all their efficiency (fed into physics::engine::ShadowInputs.turbine_efficiency_loss_fraction and added to the compressor loss fraction) and drag the free-engine HP shaft (hp_drag_torque_n_m), collapsing real N3 -- no invented `MAIN BRG SEIZURE` title exists in the FCOM; a severe-enough seizure stops the engine and reaches the cockpit through the real, already-implemented FCOM `ENG n FAIL` alert (FwsAbnormalSensed.ts engFail, FlyByWire id 701800029-032), which reads the engine's actual physical N3 (HPNEng<n>) collapsing below 50%.",
        );
    }
}

fn register_turbine_blade_damage(r: &mut Registry) {
    const ATA: u16 = 72;
    let mut n: u16 = 700;

    for eng in 1..=4u16 {
        let id = format!("72_turb.blade_damage_{eng}");
        reg_component(
            r, ATA, id.clone(), format!("Engine {eng} turbine blades"),
            vec![
                param("damage_frac", "surface damage/cracking/nicked tips, 0 none .. 1 at this fault's own 15%-efficiency-loss ceiling; costs turbine aerodynamic efficiency only, no imbalance", 0.0),
                param("release_frac", "a blade has actually departed, 0 none .. 1 at this fault's own 35%-efficiency-loss ceiling; costs efficiency *and* feeds the N3 rotor imbalance model (ATA 77, 77_vib.hp_rotor_n) with the departed blade's eccentric mass", 0.0),
            ],
        );
        let damage = reg_failure(
            r, ATA, &mut n,
            format!("Engine {eng} turbine blade damage"),
            &id, "engine_accessories::TurbineBladeDamageFaults.damage_frac", "0 none .. 1 full (15% efficiency loss)",
            "Derates `physics::engine::mod.rs`'s turbine_efficiency_loss_fraction directly: more pressure drop needed for the same shaft work, so EGT and fuel flow both rise for a given thrust -- the same mechanism as a compressor efficiency loss, just on the expansion side.",
        );
        let release = reg_failure(
            r, ATA, &mut n,
            format!("Engine {eng} HP turbine blade release"),
            &id, "engine_accessories::TurbineBladeDamageFaults.release_frac", "0 none .. 1 full (35% efficiency loss)",
            "The same efficiency-loss mechanism as this component's own `damage_frac`, at a bigger ceiling, plus the departed blade's eccentric mass added into this chain's own N3 rotor imbalance (ATA 77, `register_rotor_dynamics`'s `blade_loss_frac`) by `engine_accessories::live`'s `step_rotors` -- a real release both derates the engine and shakes it, which plain blade damage does not.",
        );
        let _ = (damage, release);
    }
}
