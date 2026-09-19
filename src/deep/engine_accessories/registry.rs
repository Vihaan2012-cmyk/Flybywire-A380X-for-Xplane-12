//! Registers this directory's failures, components and ECAM alerts with the
//! shared `deep::api` registry. Every physical part in this area exists
//! once per engine (four identical Trent 972B-84 installations), so this
//! file loops over engines 1..=4 itself: a component id, a failure id and
//! an ECAM alert's variable names (e.g. `GENERAL ENG STARTER:2`) are all
//! necessarily per-engine already, so nothing outside this file needs to
//! "multiply by 4".
//!
//! ATA chapter numbers used here follow standard iSpec 2200 engine
//! chapters: 73 Engine Fuel and Control (including the EEC), 74 Ignition,
//! 80 Starting, 72 Engine (compressor variable geometry), 75 Air (engine
//! bleed/handling valves), 77 Engine Indicating (vibration), 78 Engine
//! Exhaust (thrust reverser), 30 Ice and Rain Protection (nacelle
//! anti-ice), 71 Power Plant (nacelle ventilation), 26 Fire Protection
//! (fire-loop sensing only — see `nacelle::fire_detection`'s module docs for
//! the documented hand-off to the dedicated fire-system agent).
//!
//! New Vars this area's models would need published (none exist yet; see
//! `PROGRESS.md`): every `A32NX_ENG_n_*` name referenced by the ECAM alerts
//! below. Vars already published by this plugin are used as-is
//! (`GENERAL ENG STARTER:n`, `A32NX_AUTOTHRUST_TLA:n`, `FIRE_BUTTON_ENGn`).

use crate::deep::api::*;

const AREA: Area = Area::EngineAccessories;

fn param(name: &str, meaning: &str, healthy: f64) -> ParamDef {
    ParamDef { name: name.to_string(), meaning: meaning.to_string(), healthy }
}

/// `ComponentDef.failures` is left empty here: each `FailureDef` below
/// already names its `component` (the direction `Registry::validate()`
/// cross-checks), and every failure in this file is registered immediately
/// after its component, in the same place, so nothing here is actually
/// undeclared -- it is just not duplicated in the reverse direction too.
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
    register_nacelle(r);
    // Further backlog items append their own register_x(r) calls here as
    // they are built.
}

// ---------------------------------------------------------------------------
// Fuel system (ATA 73): LP pump -> filter -> HP pump -> FMU -> HP SOV ->
// flow transmitter -> manifold/nozzles.
// ---------------------------------------------------------------------------

fn register_fuel_system(r: &mut Registry) {
    const ATA: u16 = 73;
    let mut n: u16 = 1;

    for eng in 1..=4u16 {
        // ---- LP (boost) fuel pump.
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

        // ---- Fuel filter + bypass valve.
        let filter_id = format!("73_fuel.filter_{eng}");
        reg_component(
            r, ATA, filter_id.clone(), format!("Engine {eng} fuel filter"),
            vec![param("clog", "element blocked with debris/wax, 0 clean .. 1 blocked; resistance grows as 1/(1-clog)^2", 0.0)],
        );
        let filter_clog = reg_failure(
            r, ATA, &mut n,
            format!("Engine {eng} fuel filter clog"),
            &filter_id, "fuel::filter::FilterFaults.clog", "0 clean .. 1 blocked",
            "Differential pressure rises until the 35 psi bypass valve cracks; beyond that, unfiltered fuel reaches the HP pump.",
        );
        r.alert(
            EcamAlert::new(&format!("ENG_{eng}_FUEL_FILTER_CLOG"), ATA, &format!("ENG {eng} FUEL FILTER CLOG"), Level::Advisory, var(&format!("A32NX_ENG_{eng}_FUEL_FILTER_IMPENDING_BYPASS")).on())
                .confirm(10.0)
                .status_line(&format!("ENG {eng} FUEL FILTER"))
                .raised_by(&[filter_clog]),
        );
        r.alert(
            EcamAlert::new(&format!("ENG_{eng}_FUEL_FILTER_BYPASS"), ATA, &format!("ENG {eng} FUEL FILTER BYPASS"), Level::Caution, var(&format!("A32NX_ENG_{eng}_FUEL_FILTER_BYPASSED")).on())
                .confirm(3.0)
                .inhibit(&[Phase::LiftOff, Phase::Above1500Ft, Phase::Below800Ft, Phase::Touchdown])
                .step(line(&format!("ENG {eng} FUEL FILTER"), "MONITOR"))
                .status_line(&format!("ENG {eng} FUEL FILTER BYPASS"))
                .raised_by(&[filter_clog]),
        );

        // ---- HP (gear) fuel pump.
        let hp_id = format!("73_fuel.hp_pump_{eng}");
        reg_component(
            r, ATA, hp_id.clone(), format!("Engine {eng} HP (gear) fuel pump"),
            vec![
                param("wear", "gear/bearing wear, 0 healthy .. 1 worn; internal slip flow grows as 1/(1-wear)^2 at a given discharge pressure", 0.0),
                param("inlet_starvation", "fuel not reaching the gear pockets (fed forward from LP pump cavitation), 0 none .. 1 total", 0.0),
            ],
        );
        let hp_wear = reg_failure(
            r, ATA, &mut n,
            format!("Engine {eng} HP fuel pump wear"),
            &hp_id, "fuel::hp_pump::HpPumpFaults.wear", "0 healthy .. 1 worn",
            "Internal slip flow grows as 1/(1-wear)^2; delivered flow falls short of the theoretical displacement flow.",
        );
        let hp_starve = reg_failure(
            r, ATA, &mut n,
            format!("Engine {eng} HP fuel pump inlet starvation"),
            &hp_id, "fuel::hp_pump::HpPumpFaults.inlet_starvation", "0 none .. 1 total",
            "Caps delivered flow directly, regardless of slip -- there is simply not enough fuel arriving to fill the gear pockets.",
        );
        r.alert(
            EcamAlert::new(&format!("ENG_{eng}_HP_FUEL_PUMP_FAULT"), ATA, &format!("ENG {eng} HP FUEL PUMP FAULT"), Level::Caution, var(&format!("A32NX_ENG_{eng}_HP_PUMP_LOW_FLOW")).on())
                .confirm(5.0)
                .inhibit(&[Phase::LiftOff, Phase::Above1500Ft, Phase::Below800Ft, Phase::Touchdown])
                .step(line(&format!("THR LEVER {eng}"), "CONFIRM"))
                .step(
                    line(&format!("ENG {eng} MASTER"), "OFF")
                        .only_if(var(&format!("A32NX_ENG_{eng}_FF_DISAGREE")).on())
                        .done(var(&format!("GENERAL ENG STARTER:{eng}")).off()),
                )
                .inop_sys(&format!("ENG {eng} HP FUEL PUMP"))
                .raised_by(&[hp_wear, hp_starve]),
        );

        // ---- Fuel metering unit.
        let fmu_id = format!("73_fuel.fmu_{eng}");
        reg_component(
            r, ATA, fmu_id.clone(), format!("Engine {eng} fuel metering unit"),
            vec![
                param("valve_sticking", "metering valve seizure/fouling, 0 free .. 1 seized; scales actuator slew rate to zero", 0.0),
                param("spill_stuck_open", "spill valve stuck open, 0 healthy .. 1 fully open; collapses the regulated differential toward zero", 0.0),
                param("spill_stuck_closed", "spill valve stuck closed, 0 healthy .. 1 fully closed; differential rises toward raw HP pump pressure", 0.0),
            ],
        );
        let fmu_stick = reg_failure(
            r, ATA, &mut n,
            format!("Engine {eng} FMU metering valve sticking"),
            &fmu_id, "fuel::fmu::FmuFaults.valve_sticking", "0 free .. 1 seized",
            "Actuator slew rate falls to zero; the valve tracks the commanded flow ever more slowly and, fully stuck, freezes in place.",
        );
        let fmu_open = reg_failure(
            r, ATA, &mut n,
            format!("Engine {eng} FMU spill valve stuck open"),
            &fmu_id, "fuel::fmu::FmuFaults.spill_stuck_open", "0 healthy .. 1 fully open",
            "The regulated differential collapses toward zero; metered flow starves even at full valve area (flameout risk).",
        );
        let fmu_closed = reg_failure(
            r, ATA, &mut n,
            format!("Engine {eng} FMU spill valve stuck closed"),
            &fmu_id, "fuel::fmu::FmuFaults.spill_stuck_closed", "0 healthy .. 1 fully closed",
            "The differential rises toward the raw HP pump discharge pressure; over-meters flow for a given commanded area.",
        );
        r.alert(
            EcamAlert::new(&format!("ENG_{eng}_FADEC_FUEL_METERING_FAULT"), ATA, &format!("ENG {eng} FADEC FUEL METERING FAULT"), Level::Caution, var(&format!("A32NX_ENG_{eng}_FMU_FAULT")).on())
                .confirm(5.0)
                .inhibit(&[Phase::LiftOff, Phase::Above1500Ft, Phase::Below800Ft, Phase::Touchdown])
                .step(line(&format!("ENG {eng} FADEC"), "MONITOR THRUST"))
                .step(
                    line(&format!("ENG {eng} MASTER"), "OFF")
                        .only_if(var(&format!("A32NX_ENG_{eng}_THRUST_ABNORMAL")).on())
                        .done(var(&format!("GENERAL ENG STARTER:{eng}")).off())
                        .after(30.0),
                )
                .inop_sys(&format!("ENG {eng} FADEC METERING"))
                .raised_by(&[fmu_stick, fmu_open, fmu_closed]),
        );

        // ---- HP fuel shut-off valve.
        let sov_id = format!("73_fuel.hp_sov_{eng}");
        reg_component(
            r, ATA, sov_id.clone(), format!("Engine {eng} HP fuel shut-off valve"),
            vec![param("stuck", "mechanically stuck, 0 free .. 1 seized; travel rate scaled to zero", 0.0)],
        );
        let sov_stuck = reg_failure(
            r, ATA, &mut n,
            format!("Engine {eng} HP fuel shut-off valve stuck"),
            &sov_id, "fuel::shutoff_valve::ShutoffValveFaults.stuck", "0 free .. 1 seized",
            "Travel rate scaled to zero; stuck open defeats a fire-handle shutdown, stuck closed flames the engine out and blocks a restart.",
        );
        r.alert(
            EcamAlert::new(&format!("ENG_{eng}_HP_SOV_FAULT"), ATA, &format!("ENG {eng} HP SOV FAULT"), Level::Warning, var(&format!("A32NX_ENG_{eng}_HP_SOV_DISAGREE")).on())
                .confirm(3.0)
                .step(line(&format!("ENG {eng} FIRE PUSH BUTTON"), "CONFIRM").done(var(&format!("FIRE_BUTTON_ENG{eng}")).on()))
                .inop_sys(&format!("ENG {eng} HP SOV"))
                .raised_by(&[sov_stuck]),
        );

        // ---- Fuel flow transmitter (dual pick-off).
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
        let ft_a_bias = reg_failure(r, ATA, &mut n, format!("Engine {eng} fuel flow transmitter channel A bias"), &ft_id, "fuel::flow_transmitter::PickoffFaults.bias_frac_of_design (channel A)", "0 accurate .. 1 max bias", "Offsets channel A's reading up to 5 kg/s independent of channel B.");
        let ft_a_frozen = reg_failure(r, ATA, &mut n, format!("Engine {eng} fuel flow transmitter channel A frozen"), &ft_id, "fuel::flow_transmitter::PickoffFaults.frozen (channel A)", "0 live .. 1 frozen", "Channel A stops updating and holds its last reading while channel B keeps tracking.");
        let ft_b_bias = reg_failure(r, ATA, &mut n, format!("Engine {eng} fuel flow transmitter channel B bias"), &ft_id, "fuel::flow_transmitter::PickoffFaults.bias_frac_of_design (channel B)", "0 accurate .. 1 max bias", "Offsets channel B's reading up to 5 kg/s independent of channel A.");
        let ft_b_frozen = reg_failure(r, ATA, &mut n, format!("Engine {eng} fuel flow transmitter channel B frozen"), &ft_id, "fuel::flow_transmitter::PickoffFaults.frozen (channel B)", "0 live .. 1 frozen", "Channel B stops updating and holds its last reading while channel A keeps tracking.");
        r.alert(
            EcamAlert::new(&format!("ENG_{eng}_FUEL_FLOW_DISAGREE"), ATA, &format!("ENG {eng} FUEL FLOW DISAGREE"), Level::Advisory, var(&format!("A32NX_ENG_{eng}_FF_CHANNEL_DISAGREE")).on())
                .confirm(5.0)
                .status_line(&format!("ENG {eng} FF CHANNEL FAULT"))
                .raised_by(&[ft_a_bias, ft_a_frozen, ft_b_bias, ft_b_frozen]),
        );

        // ---- Burner manifold and nozzle groups.
        let manifold_id = format!("73_fuel.manifold_{eng}");
        let mut manifold_params = Vec::new();
        for g in 0..super::fuel::manifold::NUM_GROUPS {
            manifold_params.push(param(&format!("group_{g}_blockage"), "nozzle group coking/blockage, 0 clean .. 1 blocked", 0.0));
        }
        reg_component(r, ATA, manifold_id.clone(), format!("Engine {eng} burner manifold and nozzles"), manifold_params);
        let mut nozzle_failures = Vec::new();
        for g in 0..super::fuel::manifold::NUM_GROUPS {
            nozzle_failures.push(reg_failure(
                r, ATA, &mut n,
                format!("Engine {eng} burner nozzle group {g} coking"),
                &manifold_id, &format!("fuel::manifold::ManifoldFaults.group_blockage[{g}]"), "0 clean .. 1 blocked",
                "Shrinks that group's orifice area; the shared manifold pressure rises until the other groups pass the difference, raising hot_streak_severity for the combustor to see as a local hot streak.",
            ));
        }
        r.alert(
            EcamAlert::new(&format!("ENG_{eng}_FUEL_NOZZLE_IMBALANCE"), ATA, &format!("ENG {eng} FUEL NOZZLE"), Level::Advisory, var(&format!("A32NX_ENG_{eng}_NOZZLE_IMBALANCE")).on())
                .confirm(30.0)
                .status_line(&format!("ENG {eng} FUEL NOZZLE"))
                .raised_by(&nozzle_failures),
        );
    }
}

// ---------------------------------------------------------------------------
// Ignition (ATA 74): two exciter/igniter chains per engine.
// ---------------------------------------------------------------------------

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
        r.alert(
            EcamAlert::new(&format!("ENG_{eng}_IGNITION_FAULT"), ATA, &format!("ENG {eng} IGNITION FAULT"), Level::Caution, var(&format!("A32NX_ENG_{eng}_NO_IGNITION_AVAILABLE")).on())
                .confirm(1.0)
                .inhibit(&[Phase::LiftOff, Phase::Above1500Ft])
                .step(line(&format!("ENG {eng} IGN"), "CHECK"))
                .inop_sys(&format!("ENG {eng} IGNITION"))
                .raised_by(&chain_failures),
        );
    }
}

// ---------------------------------------------------------------------------
// Starting (ATA 80): starter air valve, air turbine starter + sprag clutch,
// duty-cycle heating.
// ---------------------------------------------------------------------------

fn register_starting(r: &mut Registry) {
    const ATA: u16 = 80;
    let mut n: u16 = 1;

    for eng in 1..=4u16 {
        // ---- Starter air valve.
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
        r.alert(
            EcamAlert::new(&format!("ENG_{eng}_START_VALVE_FAULT"), ATA, &format!("ENG {eng} START VALVE FAULT"), Level::Caution, var(&format!("A32NX_ENG_{eng}_START_VALVE_DISAGREE")).on())
                .confirm(3.0)
                .inhibit(&[Phase::LiftOff, Phase::Above1500Ft])
                .step(line(&format!("ENG {eng} START SEL"), "NORM").done(var(&format!("GENERAL ENG STARTER ACTIVE:{eng}")).off()))
                .inop_sys(&format!("ENG {eng} START VALVE"))
                .raised_by(&[sav_stuck]),
        );

        // ---- Air turbine starter + sprag clutch.
        let ats_id = format!("80_start.ats_{eng}");
        reg_component(
            r, ATA, ats_id.clone(), format!("Engine {eng} air turbine starter"),
            vec![
                param("clutch_fails_to_engage", "sprag fails to engage, 0 full transmission .. 1 none; a hung start with a spinning but useless turbine", 0.0),
                param("clutch_fails_to_disengage", "sprag fails to freewheel once overtaken, 0 healthy .. 1 fully coupled; drags the spool and, combined with a sustained high spool speed, can disintegrate the starter", 0.0),
            ],
        );
        // Registered (feeds the Components/Failures pages) but not itself
        // singled out by any ECAM alert below: a hung start already shows
        // up through the ordinary "engine will not light" symptoms.
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
        r.alert(
            EcamAlert::new(&format!("ENG_{eng}_STARTER_CUTOUT_FAULT"), ATA, &format!("ENG {eng} STARTER FAULT"), Level::Caution, var(&format!("A32NX_ENG_{eng}_STARTER_DISENGAGE_FAULT")).on())
                .confirm(3.0)
                .inhibit(&[Phase::LiftOff, Phase::Above1500Ft])
                .status_line(&format!("ENG {eng} STARTER"))
                .inop_sys(&format!("ENG {eng} STARTER"))
                .raised_by(&[clutch_disengage]),
        );
        r.alert(
            EcamAlert::new(&format!("ENG_{eng}_STARTER_DISINTEGRATED"), ATA, &format!("ENG {eng} STARTER FAULT"), Level::Warning, var(&format!("A32NX_ENG_{eng}_STARTER_DISINTEGRATED")).on())
                .step(line(&format!("ENG {eng} START SEL"), "NORM"))
                .inop_sys(&format!("ENG {eng} STARTER"))
                .raised_by(&[clutch_disengage]),
        );

        // ---- Starter duty-cycle heating: an operating-limit condition, not
        // an injectable fault, so it raises no FailureDef of its own.
        r.alert(
            EcamAlert::new(&format!("ENG_{eng}_STARTER_OVERHEAT"), ATA, &format!("ENG {eng} STARTER OVERHEAT"), Level::Advisory, var(&format!("A32NX_ENG_{eng}_STARTER_OVERHEAT")).on())
                .confirm(1.0)
                .status_line(&format!("ENG {eng} STARTER"))
                .step(line(&format!("ENG {eng} START"), "DO NOT SELECT").done(var(&format!("GENERAL ENG STARTER ACTIVE:{eng}")).off())),
        );
    }
}

// ---------------------------------------------------------------------------
// Compressor airflow control: IP VSV (ATA 72, engine-internal compressor
// geometry) and IP/HP handling bleed valves (ATA 75, engine air).
// ---------------------------------------------------------------------------

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
                param("jam", "actuator jam, 0 free .. 1 seized; freezes the vane ring regardless of what the schedule now calls for", 0.0),
                param("rigging_bias_deg", "feedback/rigging offset, signed degrees (not a 0..1 fraction); a persistent angle error even with a healthy actuator", 0.0),
            ],
        );
        let jam = reg_failure(
            r, ATA, &mut n,
            format!("Engine {eng} IP VSV actuator jam"),
            &vsv_id, "airflow_control::vsv::VsvFaults.jam", "0 free .. 1 seized",
            "Freezes the vane ring at whatever angle it was at; the growing schedule error costs stall margin (airflow_control::vsv::VsvState.stall_margin_delta_pct) for the gas path's compressor model to apply.",
        );
        let rigging = reg_failure(
            r, ATA, &mut n,
            format!("Engine {eng} IP VSV rigging error"),
            &vsv_id, "airflow_control::vsv::VsvFaults.rigging_bias_deg", "signed degrees, 0 = none",
            "A healthy, fully responsive actuator still settles off the true schedule by the bias amount, costing stall margin exactly as a jam does.",
        );
        r.alert(
            EcamAlert::new(&format!("ENG_{eng}_VSV_FAULT"), ATA, &format!("ENG {eng} VSV FAULT"), Level::Caution, var(&format!("A32NX_ENG_{eng}_VSV_SCHEDULE_ERROR_DEG")).gt(10.0))
                .confirm(5.0)
                .inhibit(&[Phase::LiftOff, Phase::Above1500Ft, Phase::Below800Ft, Phase::Touchdown])
                .status_line(&format!("ENG {eng} VSV"))
                .inop_sys(&format!("ENG {eng} VSV"))
                .raised_by(&[jam, rigging]),
        );
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
            r.alert(
                EcamAlert::new(&format!("ENG_{eng}_{spool}_BLEED_VALVE_FAULT"), ATA, &format!("ENG {eng} {spool} BLEED VALVE FAULT"), Level::Caution, var(&format!("A32NX_ENG_{eng}_{spool}_HANDLING_BLEED_DISAGREE")).on())
                    .confirm(5.0)
                    .inhibit(&[Phase::LiftOff, Phase::Above1500Ft, Phase::Below800Ft, Phase::Touchdown])
                    .status_line(&format!("ENG {eng} {spool} BLEED VALVE"))
                    .inop_sys(&format!("ENG {eng} {spool} BLEED VALVE"))
                    .raised_by(&[jam]),
            );
        }
    }
}

// ---------------------------------------------------------------------------
// Rotor dynamics (ATA 77, Engine Indicating): per-spool imbalance vibration
// and bearing defect signature.
// ---------------------------------------------------------------------------

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
            let blade = reg_failure(r, ATA, &mut n, format!("Engine {eng} {spool} blade loss"), &id, &format!("rotor_dynamics::imbalance::ImbalanceFaults.blade_loss_frac ({key})"), "0 none .. 1 full blade loss", "Adds eccentric mass; synchronous vibration rises with the square of shaft speed (rotor_dynamics::imbalance::VibrationState.index).");
            let ice = reg_failure(r, ATA, &mut n, format!("Engine {eng} {spool} asymmetric ice accretion"), &id, &format!("rotor_dynamics::imbalance::ImbalanceFaults.ice_frac ({key})"), "0 none .. 1 full", "Adds eccentric mass the same way a blade loss does, at a smaller typical magnitude.");
            let bird = reg_failure(r, ATA, &mut n, format!("Engine {eng} {spool} bird-strike imbalance"), &id, &format!("rotor_dynamics::imbalance::ImbalanceFaults.bird_strike_frac ({key})"), "0 none .. 1 full", "Adds eccentric mass from impact damage, same mechanism as blade loss/ice.");
            r.alert(
                EcamAlert::new(&format!("ENG_{eng}_{spool}_VIB_HI"), ATA, &format!("ENG {eng} {spool} VIB HI"), Level::Caution, var(&format!("A32NX_ENG_{eng}_{var_suffix}_VIB_INDEX")).gt(4.0))
                    .confirm(5.0)
                    .inhibit(&[Phase::LiftOff, Phase::Above1500Ft])
                    .step(line(&format!("THR LEVER {eng}"), "REDUCE"))
                    .status_line(&format!("ENG {eng} {spool} VIB"))
                    .raised_by(&[blade, ice, bird]),
            );
        }

        // The Trent's real 5-bearing arrangement, matching
        // `physics::engine::oil`'s own three chambers exactly (see
        // `rotor_dynamics::bearings`' module docs for the citation): Front
        // = fan + IP front bearings, HpIp = HP + IP turbine bearings, Tail
        // = the single LP turbine rear bearing. Each gets its own
        // component, its own 4 spall failures, and its own vibration *and*
        // oil-debris (chip detector) alerts, rather than one representative
        // set standing in for the whole engine.
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
            let outer = reg_failure(r, ATA, &mut n, format!("Engine {eng} {display} outer race spall"), &bearing_id, &format!("rotor_dynamics::bearings::EngineBearingFaults.faults[{key}].outer_race_spall"), "0 none .. 1 severe", "Vibration at this bearing's outer-race ball-pass frequency (BPFO); also feeds the oil system's chip detector (rotor_dynamics::bearings::BearingState.chip_detected).");
            let inner = reg_failure(r, ATA, &mut n, format!("Engine {eng} {display} inner race spall"), &bearing_id, &format!("rotor_dynamics::bearings::EngineBearingFaults.faults[{key}].inner_race_spall"), "0 none .. 1 severe", "Vibration at BPFI, the highest of the four defect frequencies for this geometry; also feeds the chip detector.");
            let ball = reg_failure(r, ATA, &mut n, format!("Engine {eng} {display} rolling-element spall"), &bearing_id, &format!("rotor_dynamics::bearings::EngineBearingFaults.faults[{key}].rolling_element_spall"), "0 none .. 1 severe", "Vibration at the ball-spin frequency (BSF); also feeds the chip detector.");
            let cage = reg_failure(r, ATA, &mut n, format!("Engine {eng} {display} cage wear"), &bearing_id, &format!("rotor_dynamics::bearings::EngineBearingFaults.faults[{key}].cage_wear"), "0 none .. 1 severe", "Vibration at the fundamental train frequency (FTF), the lowest of the four; also feeds the chip detector.");
            let key_upper = key.to_uppercase();
            r.alert(
                EcamAlert::new(&format!("ENG_{eng}_{key_upper}_VIB"), ATA, &format!("ENG {eng} {display} VIB"), Level::Advisory, var(&format!("A32NX_ENG_{eng}_{var_suffix}_{key_upper}_DEFECT_AMPLITUDE_MM_S")).gt(2.0))
                    .confirm(10.0)
                    .status_line(&format!("ENG {eng} {display}"))
                    .raised_by(&[outer, inner, ball, cage]),
            );
            r.alert(
                EcamAlert::new(&format!("ENG_{eng}_{key_upper}_CHIP"), ATA, &format!("ENG {eng} {display} CHIP DET"), Level::Advisory, var(&format!("A32NX_ENG_{eng}_{key_upper}_CHIP_DETECTED")).on())
                    .confirm(1.0)
                    .status_line(&format!("ENG {eng} {display} CHIP DETECTOR"))
                    .raised_by(&[outer, inner, ball, cage]),
            );
        }
    }
}

// ---------------------------------------------------------------------------
// Thrust reverser (ATA 78, Engine Exhaust): inboard engines only (2 and 3
// -- the real A380 carries no reverser on 1/4).
// ---------------------------------------------------------------------------

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
            ],
        );
        let a_hold = reg_failure(r, ATA, &mut n, format!("Engine {eng} reverser primary lock fails to hold"), &id, "thrust_reverser::LockFaults.fails_to_hold (lock_a)", "0 healthy .. 1 total", "One of three independent restraints; alone, the other two still prevent deployment (an OR across all three).");
        let a_jam = reg_failure(r, ATA, &mut n, format!("Engine {eng} reverser primary lock jam"), &id, "thrust_reverser::LockFaults.jam (lock_a)", "0 free .. 1 seized", "Blocks legitimate deployment outright (every lock must release, an AND across all three) -- the safe-direction failure.");
        let b_hold = reg_failure(r, ATA, &mut n, format!("Engine {eng} reverser secondary lock fails to hold"), &id, "thrust_reverser::LockFaults.fails_to_hold (lock_b)", "0 healthy .. 1 total", "Same redundancy logic as the primary lock.");
        let b_jam = reg_failure(r, ATA, &mut n, format!("Engine {eng} reverser secondary lock jam"), &id, "thrust_reverser::LockFaults.jam (lock_b)", "0 free .. 1 seized", "Blocks legitimate deployment, same as the primary lock's jam.");
        let c_hold = reg_failure(r, ATA, &mut n, format!("Engine {eng} reverser tertiary lock fails to hold"), &id, "thrust_reverser::LockFaults.fails_to_hold (lock_c)", "0 healthy .. 1 total", "Same redundancy logic; all three failing this way together is what an uncommanded deployment actually requires (thrust_reverser::ReverserState.uncommanded_deployment).");
        let c_jam = reg_failure(r, ATA, &mut n, format!("Engine {eng} reverser tertiary lock jam"), &id, "thrust_reverser::LockFaults.jam (lock_c)", "0 free .. 1 seized", "Blocks legitimate deployment, same as the other two locks' jams.");
        let act_jam = reg_failure(r, ATA, &mut n, format!("Engine {eng} reverser actuator jam"), &id, "thrust_reverser::ReverserFaults.actuator_jam", "0 free .. 1 solid", "Blocks both deploy and stow motion regardless of lock state -- the direct fails-to-deploy/fails-to-stow fault.");

        r.alert(
            EcamAlert::new(&format!("ENG_{eng}_REVERSER_UNLOCKED"), ATA, &format!("ENG {eng} REVERSER UNLOCKED"), Level::Warning, var(&format!("A32NX_ENG_{eng}_REV_UNCOMMANDED")).on())
                .step(line(&format!("ENG {eng} THRUST LEVER"), "IDLE"))
                .step(line(&format!("ENG {eng} MASTER"), "OFF").done(var(&format!("GENERAL ENG STARTER:{eng}")).off()))
                .inop_sys(&format!("ENG {eng} REVERSER"))
                .raised_by(&[a_hold, b_hold, c_hold]),
        );
        r.alert(
            EcamAlert::new(&format!("ENG_{eng}_REVERSER_FAULT"), ATA, &format!("ENG {eng} REVERSER FAULT"), Level::Caution, var(&format!("A32NX_ENG_{eng}_REV_POSITION_DISAGREE")).on())
                .confirm(3.0)
                .inhibit(&[Phase::LiftOff, Phase::Above1500Ft])
                .status_line(&format!("ENG {eng} REVERSER"))
                .inop_sys(&format!("ENG {eng} REVERSER"))
                .raised_by(&[a_jam, b_jam, c_jam, act_jam]),
        );
    }
}

// ---------------------------------------------------------------------------
// EEC (ATA 73, Engine Fuel and Control -- same chapter the existing
// catalogue's "FADEC channel fault" already uses, `failures.rs`). Starts its
// `n` counter at 500 to stay clear of `register_fuel_system`'s own ATA-73
// range (21 failures/engine x 4 = 84, i.e. 1..84), since ids only need to
// be unique per (area, ata), not per function.
// ---------------------------------------------------------------------------

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
        reg_component(r, ATA, id.clone(), format!("Engine {eng} EEC (dual channel)"), params);

        let chan_a = reg_failure(r, ATA, &mut n, format!("Engine {eng} EEC channel A fault"), &id, "eec::EecFaults.channel_a_fault", "0 healthy .. 1 dead", "Hands control to channel B if it is healthy; both dead leaves no valid EEC channel (eec::ActiveChannel::None).");
        let chan_b = reg_failure(r, ATA, &mut n, format!("Engine {eng} EEC channel B fault"), &id, "eec::EecFaults.channel_b_fault", "0 healthy .. 1 dead", "Symmetric with channel A's fault.");

        let mut sensor_failures = Vec::new();
        for p in ["N1", "N2", "N3", "TGT", "P30"] {
            let fa = reg_failure(r, ATA, &mut n, format!("Engine {eng} EEC {p} sensor fault, channel A"), &id, &format!("eec::EecFaults.sensor_a[{p}] (bias/frozen)"), "0 healthy .. 1 max bias or fully frozen", "Biases or freezes channel A's reading for this parameter; flagged against channel B once the disagreement exceeds this parameter's threshold (eec::EecState.disagree).");
            let fb = reg_failure(r, ATA, &mut n, format!("Engine {eng} EEC {p} sensor fault, channel B"), &id, &format!("eec::EecFaults.sensor_b[{p}] (bias/frozen)"), "0 healthy .. 1 max bias or fully frozen", "Symmetric with channel A's sensor fault for the same parameter.");
            sensor_failures.push(fa);
            sensor_failures.push(fb);
        }

        r.alert(
            EcamAlert::new(&format!("ENG_{eng}_EEC_CHANNEL_FAULT"), ATA, &format!("ENG {eng} EEC CHANNEL FAULT"), Level::Caution, var(&format!("A32NX_ENG_{eng}_EEC_CHANNEL_FAULT")).on())
                .confirm(1.0)
                .status_line(&format!("ENG {eng} EEC"))
                .raised_by(&[chan_a, chan_b]),
        );
        r.alert(
            EcamAlert::new(&format!("ENG_{eng}_EEC_DUAL_FAULT"), ATA, &format!("ENG {eng} EEC FAULT"), Level::Warning, var(&format!("A32NX_ENG_{eng}_EEC_NO_VALID_CHANNEL")).on())
                .step(line(&format!("ENG {eng} THR"), "MANUAL BACKUP"))
                .inop_sys(&format!("ENG {eng} EEC"))
                .raised_by(&[chan_a, chan_b]),
        );
        r.alert(
            EcamAlert::new(&format!("ENG_{eng}_EEC_SENSOR_DISAGREE"), ATA, &format!("ENG {eng} EEC SENSOR DISAGREE"), Level::Advisory, var(&format!("A32NX_ENG_{eng}_EEC_SENSOR_DISAGREE")).on())
                .confirm(5.0)
                .status_line(&format!("ENG {eng} EEC SENSORS"))
                .raised_by(&sensor_failures),
        );
    }
}

// ---------------------------------------------------------------------------
// Nacelle: anti-ice (ATA 30), ventilation (ATA 71, Power Plant), fire/
// overheat detection sensing (ATA 26, sense-only -- see fire_detection.rs).
// ---------------------------------------------------------------------------

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
            EcamAlert::new(&format!("ENG_{eng}_ANTI_ICE_FAULT"), ATA, &format!("ENG {eng} ANTI ICE FAULT"), Level::Caution, var(&format!("A32NX_ENG_{eng}_ANTI_ICE_DISAGREE")).on())
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
        r.alert(
            EcamAlert::new(&format!("ENG_{eng}_NACELLE_VENT_LO"), ATA, &format!("ENG {eng} NACELLE VENT LO"), Level::Caution, var(&format!("A32NX_ENG_{eng}_NACELLE_VAPOUR_RISK")).on())
                .confirm(10.0)
                .status_line(&format!("ENG {eng} NACELLE VENT"))
                .raised_by(&[scoop, eductor]),
        );
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
            r.alert(
                EcamAlert::new(&format!("ENG_{eng}_{zone_name}_FIRE_LOOP_FAULT"), ATA, &format!("ENG {eng} {zone_name} FIRE DET FAULT"), Level::Advisory, var(&format!("A32NX_ENG_{eng}_{zone_name}_FIRE_LOOP_DISAGREE")).on())
                    .confirm(5.0)
                    .status_line(&format!("ENG {eng} {zone_name} FIRE DET"))
                    .raised_by(&[a_fail, a_false, b_fail, b_false]),
            );
        }
    }
}
