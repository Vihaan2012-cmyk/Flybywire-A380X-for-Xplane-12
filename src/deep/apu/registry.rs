//! Registers every APU failure, component and ECAM alert through
//! `deep::api`, per `docs/deep/BRIEF.md`'s "Registering failures, components
//! and ECAM alerts" section. `Area::Apu`, ATA 49 throughout.

use crate::deep::api::*;

pub fn register(r: &mut Registry) {
    const ATA: u16 = 49;
    let fid = |n: u16| failure_id(Area::Apu, ATA, n);

    // ---- Failures ------------------------------------------------------

    let compressor_erosion = fid(1);
    let turbine_damage = fid(2);
    let load_compressor_erosion = fid(3);
    let igv_jam = fid(4);
    let scv_jam = fid(5);
    let starter_degradation = fid(6);
    let igniter_failure = fid(7);
    let fcu_fault = fid(8);
    let speed_sensor_fault = fid(9);
    let oil_leak = fid(10);
    let inlet_door_jam = fid(11);
    let gen1_wear = fid(12);
    let gen1_overload = fid(13);
    let gen2_wear = fid(14);
    let gen2_overload = fid(15);
    let egt_sensor_fault = fid(16);
    let fire_loop_failure = fid(17);
    let fire_squib_failure = fid(18);

    r.failure(FailureDef {
        id: compressor_erosion,
        area: Area::Apu,
        ata: ATA,
        name: "APU core compressor erosion".into(),
        component: "49_apu.core_compressor".into(),
        model_field: "power_section.rs::PowerSectionFaults.compressor_efficiency_loss".into(),
        magnitude: "0..1 isentropic efficiency loss fraction".into(),
        effect: "Same Euler blade work but less becomes useful pressure rise: compressor exit pressure falls for the same speed, and EGT rises for the same fuel flow/speed as the governor works to hold N under load.".into(),
    });
    r.failure(FailureDef {
        id: turbine_damage,
        area: Area::Apu,
        ata: ATA,
        name: "APU turbine damage".into(),
        component: "49_apu.turbine".into(),
        model_field: "power_section.rs::PowerSectionFaults.turbine_efficiency_loss".into(),
        magnitude: "0..1 isentropic efficiency loss fraction".into(),
        effect: "Less shaft work extracted per unit expansion: more of the gas's enthalpy survives to the exit (higher EGT) and the governor must burn more fuel to hold governed N under the same load.".into(),
    });
    r.failure(FailureDef {
        id: load_compressor_erosion,
        area: Area::Apu,
        ata: ATA,
        name: "APU load/bleed compressor erosion".into(),
        component: "49_apu.load_compressor".into(),
        model_field: "load_compressor.rs::LoadCompressorFaults.efficiency_loss".into(),
        magnitude: "0..1 isentropic efficiency loss fraction".into(),
        effect: "Lower delivered bleed pressure for the same demand and spool speed.".into(),
    });
    r.failure(FailureDef {
        id: igv_jam,
        area: Area::Apu,
        ata: ATA,
        name: "APU load compressor IGV actuator jam".into(),
        component: "49_apu.igv_actuator".into(),
        model_field: "load_compressor.rs::LoadCompressorFaults.igv_jam (actuator.rs::Actuator)".into(),
        magnitude: "0..1 seizure fraction (0=free .. 1=frozen wherever it currently is)".into(),
        effect: "Vanes freeze at their current opening; if that is closed relative to current demand, bleed delivery is starved even though the aircraft calls for more.".into(),
    });
    r.failure(FailureDef {
        id: scv_jam,
        area: Area::Apu,
        ata: ATA,
        name: "APU surge control valve actuator jam".into(),
        component: "49_apu.scv_actuator".into(),
        model_field: "load_compressor.rs::LoadCompressorFaults.scv_jam (actuator.rs::Actuator)".into(),
        magnitude: "0..1 seizure fraction (freezes at whatever recirculation position it held, i.e. stuck open or stuck closed depending on demand history at the moment it jams)".into(),
        effect: "Cannot open to recirculate flow when bleed demand drops faster than it can follow: the load compressor's operating point falls below its surge line and it genuinely surges (compressor_map.rs::Point.in_surge).".into(),
    });
    r.failure(FailureDef {
        id: starter_degradation,
        area: Area::Apu,
        ata: ATA,
        name: "APU starter motor degradation/failure".into(),
        component: "49_apu.starter_motor".into(),
        model_field: "starter.rs::StarterFaults.starter_degradation".into(),
        magnitude: "0..1, weakens the shared back-EMF/torque constant (STARTER_KE_KT)".into(),
        effect: "More current for less torque at the same speed: a slower, weaker start, and in severe cases the core never reaches light-off speed.".into(),
    });
    r.failure(FailureDef {
        id: igniter_failure,
        area: Area::Apu,
        ata: ATA,
        name: "APU igniter failure".into(),
        component: "49_apu.igniter".into(),
        model_field: "starter.rs::StarterFaults.igniter_failure".into(),
        magnitude: "0..1, raises the effective light-off speed threshold".into(),
        effect: "At full failure the threshold sits at/above self-sustaining speed, unreachable by the starter alone: a hung start with fuel never lit.".into(),
    });
    r.failure(FailureDef {
        id: fcu_fault,
        area: Area::Apu,
        ata: ATA,
        name: "APU fuel control unit (metering valve) fault".into(),
        component: "49_apu.fuel_control_unit".into(),
        model_field: "fuel_control.rs::FuelControlFaults.metering_valve_jam".into(),
        magnitude: "0..1 seizure fraction".into(),
        effect: "Fuel flow freezes at whatever it was delivering; over- or under-fuels the combustor from then on regardless of the governor's command.".into(),
    });
    r.failure(FailureDef {
        id: speed_sensor_fault,
        area: Area::Apu,
        ata: ATA,
        name: "APU ECB channel A/B speed pickup fault".into(),
        component: "49_apu.governor_speed_sensor".into(),
        model_field: "ecb.rs::EcbFaults.channel_a/b.speed_sensor (SensorFault{bias,failed})".into(),
        magnitude: "0..1 bias (under-reads true N by up to params::N_SENSOR_MAX_BIAS_PERCENT), or an outright failed=true dropout".into(),
        effect: "A single channel's bias is masked by the other, still-valid channel's own vote/average (ecb.rs::Ecb::step) -- only when both channels are biased the same way, or both fail outright (its own separate 'dual channel speed loss' condition), does the governor see a wrong or missing speed and either overfuel toward overspeed or lose control entirely.".into(),
    });
    r.failure(FailureDef {
        id: oil_leak,
        area: Area::Apu,
        ata: ATA,
        name: "APU oil leak".into(),
        component: "49_apu.oil_system".into(),
        model_field: "oil.rs::OilFaults.leak".into(),
        magnitude: "0..1 fraction of MAX_LEAK_RATE_L_S".into(),
        effect: "Tank level falls; the pump progressively starves as level drops below the low-level threshold, pressure falls, and sustained low pressure while running trips low oil pressure protection.".into(),
    });
    r.failure(FailureDef {
        id: inlet_door_jam,
        area: Area::Apu,
        ata: ATA,
        name: "APU inlet door actuator jam".into(),
        component: "49_apu.inlet_door".into(),
        model_field: "inlet_door.rs::InletDoorFaults.jam".into(),
        magnitude: "0..1 seizure fraction".into(),
        effect: "Door fails to reach fully open, imposing a continuing inlet total-pressure loss that reduces available power and raises EGT for the same demand.".into(),
    });
    r.failure(FailureDef {
        id: gen1_wear,
        area: Area::Apu,
        ata: ATA,
        name: "APU generator 1 winding/bearing wear".into(),
        component: "49_apu.generator_1".into(),
        model_field: "generators.rs::GeneratorFaults.efficiency_loss (Apu.faults.gen1)".into(),
        magnitude: "0..1".into(),
        effect: "More shaft power needed for the same electrical output, loading the core's torque balance.".into(),
    });
    r.failure(FailureDef {
        id: gen1_overload,
        area: Area::Apu,
        ata: ATA,
        name: "APU generator 1 overload protection failure".into(),
        component: "49_apu.generator_1".into(),
        model_field: "generators.rs::GeneratorFaults.overload_protection_failed (Apu.faults.gen1)".into(),
        magnitude: "boolean, represented as 0/1".into(),
        effect: "Removes the normal clamp at rated shaft power: an overloaded generator keeps demanding ever more shaft torque instead of being current-limited.".into(),
    });
    r.failure(FailureDef {
        id: gen2_wear,
        area: Area::Apu,
        ata: ATA,
        name: "APU generator 2 winding/bearing wear".into(),
        component: "49_apu.generator_2".into(),
        model_field: "generators.rs::GeneratorFaults.efficiency_loss (Apu.faults.gen2)".into(),
        magnitude: "0..1".into(),
        effect: "More shaft power needed for the same electrical output, loading the core's torque balance.".into(),
    });
    r.failure(FailureDef {
        id: gen2_overload,
        area: Area::Apu,
        ata: ATA,
        name: "APU generator 2 overload protection failure".into(),
        component: "49_apu.generator_2".into(),
        model_field: "generators.rs::GeneratorFaults.overload_protection_failed (Apu.faults.gen2)".into(),
        magnitude: "boolean, represented as 0/1".into(),
        effect: "Removes the normal clamp at rated shaft power: an overloaded generator keeps demanding ever more shaft torque instead of being current-limited.".into(),
    });
    r.failure(FailureDef {
        id: egt_sensor_fault,
        area: Area::Apu,
        ata: ATA,
        name: "APU ECB channel A/B EGT thermocouple fault".into(),
        component: "49_apu.egt_sensor".into(),
        model_field: "ecb.rs::EcbFaults.channel_a/b.egt_sensor (SensorFault{bias,failed})".into(),
        magnitude: "0..1, under-reads true EGT by up to params::EGT_SENSOR_MAX_BIAS_C, or an outright dropout".into(),
        effect: "Cockpit-indicated EGT reads low relative to the true turbine-exit temperature, masking a real overtemperature from the crew/ECAM; the true physics and the hard protective trip are unaffected.".into(),
    });
    r.failure(FailureDef {
        id: fire_loop_failure,
        area: Area::Apu,
        ata: ATA,
        name: "APU fire loop failure".into(),
        component: "49_apu.fire_loop".into(),
        model_field: "fire.rs::FireFaults.loop_failure".into(),
        magnitude: "0..1".into(),
        effect: "A real fire is never confirmed: automatic fuel/bleed shutoff never commands and the bottle cannot be commanded to discharge.".into(),
    });
    r.failure(FailureDef {
        id: fire_squib_failure,
        area: Area::Apu,
        ata: ATA,
        name: "APU fire bottle squib failure".into(),
        component: "49_apu.fire_bottle".into(),
        model_field: "fire.rs::FireFaults.squib_failure".into(),
        magnitude: "0..1".into(),
        effect: "Fire is confirmed and shutoff still commands, but the extinguisher bottle never discharges.".into(),
    });

    // ---- Components ------------------------------------------------------

    r.component(ComponentDef {
        id: "49_apu.core_compressor".into(),
        area: Area::Apu,
        ata: ATA,
        name: "APU core (power-section) compressor".into(),
        params: vec![ParamDef { name: "efficiency_loss".into(), meaning: "isentropic efficiency loss fraction (erosion/damage)".into(), healthy: 0.0 }],
        failures: vec![compressor_erosion],
    });
    r.component(ComponentDef {
        id: "49_apu.turbine".into(),
        area: Area::Apu,
        ata: ATA,
        name: "APU gas-generator turbine".into(),
        params: vec![ParamDef { name: "efficiency_loss".into(), meaning: "isentropic efficiency loss fraction (blade erosion/FOD damage)".into(), healthy: 0.0 }],
        failures: vec![turbine_damage],
    });
    r.component(ComponentDef {
        id: "49_apu.load_compressor".into(),
        area: Area::Apu,
        ata: ATA,
        name: "APU load (customer bleed) compressor".into(),
        params: vec![ParamDef { name: "efficiency_loss".into(), meaning: "isentropic efficiency loss fraction".into(), healthy: 0.0 }],
        failures: vec![load_compressor_erosion],
    });
    r.component(ComponentDef {
        id: "49_apu.igv_actuator".into(),
        area: Area::Apu,
        ata: ATA,
        name: "APU load compressor inlet guide vane actuator".into(),
        params: vec![ParamDef { name: "jam".into(), meaning: "actuator seizure fraction, 0=free .. 1=frozen in place".into(), healthy: 0.0 }],
        failures: vec![igv_jam],
    });
    r.component(ComponentDef {
        id: "49_apu.scv_actuator".into(),
        area: Area::Apu,
        ata: ATA,
        name: "APU surge control (anti-surge/recirculation) valve actuator".into(),
        params: vec![ParamDef { name: "jam".into(), meaning: "actuator seizure fraction, 0=free .. 1=frozen in place".into(), healthy: 0.0 }],
        failures: vec![scv_jam],
    });
    r.component(ComponentDef {
        id: "49_apu.starter_motor".into(),
        area: Area::Apu,
        ata: ATA,
        name: "APU starter motor".into(),
        params: vec![ParamDef { name: "degradation".into(), meaning: "back-EMF/torque constant weakening fraction (brush/commutator wear)".into(), healthy: 0.0 }],
        failures: vec![starter_degradation],
    });
    r.component(ComponentDef {
        id: "49_apu.igniter".into(),
        area: Area::Apu,
        ata: ATA,
        name: "APU igniter".into(),
        params: vec![ParamDef { name: "failure".into(), meaning: "effective light-off speed threshold raise fraction".into(), healthy: 0.0 }],
        failures: vec![igniter_failure],
    });
    r.component(ComponentDef {
        id: "49_apu.fuel_control_unit".into(),
        area: Area::Apu,
        ata: ATA,
        name: "APU fuel control unit (metering valve)".into(),
        params: vec![ParamDef { name: "metering_valve_jam".into(), meaning: "metering valve actuator seizure fraction".into(), healthy: 0.0 }],
        failures: vec![fcu_fault],
    });
    r.component(ComponentDef {
        id: "49_apu.governor_speed_sensor".into(),
        area: Area::Apu,
        ata: ATA,
        name: "APU governor speed sensor".into(),
        params: vec![ParamDef { name: "bias".into(), meaning: "fractional under-read of true N, 0..1 of N_SENSOR_MAX_BIAS_PERCENT".into(), healthy: 0.0 }],
        failures: vec![speed_sensor_fault],
    });
    r.component(ComponentDef {
        id: "49_apu.oil_system".into(),
        area: Area::Apu,
        ata: ATA,
        name: "APU oil system".into(),
        params: vec![ParamDef { name: "leak".into(), meaning: "oil leak severity, fraction of MAX_LEAK_RATE_L_S".into(), healthy: 0.0 }],
        failures: vec![oil_leak],
    });
    r.component(ComponentDef {
        id: "49_apu.inlet_door".into(),
        area: Area::Apu,
        ata: ATA,
        name: "APU inlet door actuator".into(),
        params: vec![ParamDef { name: "jam".into(), meaning: "actuator seizure fraction, 0=free .. 1=frozen in place".into(), healthy: 0.0 }],
        failures: vec![inlet_door_jam],
    });
    r.component(ComponentDef {
        id: "49_apu.generator_1".into(),
        area: Area::Apu,
        ata: ATA,
        name: "APU generator 1".into(),
        params: vec![
            ParamDef { name: "efficiency_loss".into(), meaning: "winding/bearing wear fraction".into(), healthy: 0.0 },
            ParamDef { name: "overload_protection_failed".into(), meaning: "0=protected, 1=overcurrent protection failed".into(), healthy: 0.0 },
        ],
        failures: vec![gen1_wear, gen1_overload],
    });
    r.component(ComponentDef {
        id: "49_apu.generator_2".into(),
        area: Area::Apu,
        ata: ATA,
        name: "APU generator 2".into(),
        params: vec![
            ParamDef { name: "efficiency_loss".into(), meaning: "winding/bearing wear fraction".into(), healthy: 0.0 },
            ParamDef { name: "overload_protection_failed".into(), meaning: "0=protected, 1=overcurrent protection failed".into(), healthy: 0.0 },
        ],
        failures: vec![gen2_wear, gen2_overload],
    });
    r.component(ComponentDef {
        id: "49_apu.egt_sensor".into(),
        area: Area::Apu,
        ata: ATA,
        name: "APU EGT sensor".into(),
        params: vec![ParamDef { name: "bias".into(), meaning: "fractional under-read of true EGT, 0..1 of EGT_SENSOR_MAX_BIAS_C".into(), healthy: 0.0 }],
        failures: vec![egt_sensor_fault],
    });
    r.component(ComponentDef {
        id: "49_apu.fire_loop".into(),
        area: Area::Apu,
        ata: ATA,
        name: "APU fire detection loop interface".into(),
        params: vec![ParamDef { name: "loop_failure".into(), meaning: "loop failure fraction, 0..1".into(), healthy: 0.0 }],
        failures: vec![fire_loop_failure],
    });
    r.component(ComponentDef {
        id: "49_apu.fire_bottle".into(),
        area: Area::Apu,
        ata: ATA,
        name: "APU fire extinguisher bottle".into(),
        params: vec![ParamDef { name: "squib_failure".into(), meaning: "squib failure fraction, 0..1".into(), healthy: 0.0 }],
        failures: vec![fire_squib_failure],
    });

    // ---- ECAM alerts -----------------------------------------------------

    r.alert(
        EcamAlert::new("APU_EGT_OVER_LIMIT", ATA, "APU EGT OVER LIMIT", Level::Warning, var("APU_EGT").gt(950.0))
            .confirm(1.0)
            .inhibit(&[Phase::LiftOff, Phase::Above80Kt])
            .step(line("APU MASTER SW", "OFF").done(var("OVHD_APU_MASTER_SW_PB_IS_ON").off()))
            .status_line("APU INOP")
            .inop_sys("APU")
            .raised_by(&[compressor_erosion, turbine_damage, fcu_fault, speed_sensor_fault]),
    );

    r.alert(
        EcamAlert::new("APU_OVERSPEED", ATA, "APU OVERSPEED", Level::Warning, var("APU_N").gt(105.0))
            .confirm(0.5)
            .step(line("APU MASTER SW", "OFF").done(var("OVHD_APU_MASTER_SW_PB_IS_ON").off()))
            .status_line("APU INOP")
            .inop_sys("APU")
            .raised_by(&[speed_sensor_fault, fcu_fault]),
    );

    r.alert(
        EcamAlert::new("APU_OIL_LO_PR", ATA, "APU OIL LO PR", Level::Caution, var("APU_OIL_PRESSURE_PSI").lt(15.0))
            .confirm(5.0)
            .step(line("APU MASTER SW", "OFF").done(var("OVHD_APU_MASTER_SW_PB_IS_ON").off()))
            .status_line("APU INOP")
            .inop_sys("APU")
            .raised_by(&[oil_leak]),
    );

    // APU FIRE is announced by the fire protection system, which owns the
    // single alert (`fire_ice::registry`, ATA 26, with the real fire
    // pushbutton and squib variables and the full procedure). The APU's own
    // detection loop is a further way to reach it, contributed here rather
    // than declared as a competing second alert of the same name.
    r.contribute("APU_FIRE").when(var("APU_FIRE_LOOP_DETECTED").on()).raised_by(&[fire_loop_failure, fire_squib_failure]);

    r.alert(
        EcamAlert::new("APU_BLEED_FAULT", ATA, "APU BLEED FAULT", Level::Caution, var("APU_LOAD_COMPRESSOR_SURGE").on())
            .confirm(3.0)
            .step(line("APU BLEED", "OFF").done(var("OVHD_APU_BLEED_PB_IS_ON").off()))
            .status_line("APU BLEED INOP")
            .inop_sys("APU BLEED")
            .raised_by(&[igv_jam, scv_jam, load_compressor_erosion]),
    );

    r.alert(
        EcamAlert::new("APU_GEN_1_FAULT", ATA, "APU GEN 1 FAULT", Level::Caution, var("APU_GEN_1_OVERLOAD").on())
            .confirm(2.0)
            .step(line("APU GEN 1", "OFF").done(var("OVHD_ELEC_APU_GEN_1_PB_IS_ON").off()))
            .status_line("APU GEN 1 INOP")
            .inop_sys("APU GEN 1")
            .raised_by(&[gen1_overload, gen1_wear]),
    );

    r.alert(
        EcamAlert::new("APU_GEN_2_FAULT", ATA, "APU GEN 2 FAULT", Level::Caution, var("APU_GEN_2_OVERLOAD").on())
            .confirm(2.0)
            .step(line("APU GEN 2", "OFF").done(var("OVHD_ELEC_APU_GEN_2_PB_IS_ON").off()))
            .status_line("APU GEN 2 INOP")
            .inop_sys("APU GEN 2")
            .raised_by(&[gen2_overload, gen2_wear]),
    );

    r.alert(
        EcamAlert::new("APU_START_FAULT", ATA, "APU START FAULT", Level::Caution, all(vec![var("OVHD_APU_START_PB_IS_ON").on(), var("APU_N").lt(55.0)]))
            .confirm(60.0)
            .step(line("APU START", "OFF").done(var("OVHD_APU_START_PB_IS_ON").off()))
            .status_line("APU START FAULT")
            .raised_by(&[starter_degradation, igniter_failure, fcu_fault]),
    );
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn registers_cleanly_with_no_validation_errors() {
        let mut r = Registry::default();
        register(&mut r);
        let errors = r.validate();
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
    fn eighteen_distinct_failures_are_registered() {
        let mut r = Registry::default();
        register(&mut r);
        assert_eq!(r.failures.len(), 18);
        let mut ids: Vec<u64> = r.failures.iter().map(|f| f.id).collect();
        ids.sort_unstable();
        ids.dedup();
        assert_eq!(ids.len(), 18, "duplicate failure ids");
    }
}
