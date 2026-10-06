use crate::deep::api::{all, any, var, Cond, EcamAlert, Level, Phase, Registry};

fn network_alive() -> Cond {
    any(vec![
        var("ELEC_AC_1_BUS_IS_POWERED").on(),
        var("ELEC_AC_2_BUS_IS_POWERED").on(),
        var("ELEC_AC_3_BUS_IS_POWERED").on(),
        var("ELEC_AC_4_BUS_IS_POWERED").on(),
    ])
}

pub fn register(r: &mut Registry) {
    let mut a = EcamAlert::new("WIRED_ANTI_ICE_PROBE_1_FAULT", 34, "A-ICE PITOT TAT AOA PROBE 1 HEATG FAULT", Level::Caution, all(vec![any(vec![var("DEEP_PITOT_1_HEATER_FAILED").eq(1.0), var("DEEP_TAT_1_HEATER_FAILED").eq(1.0)]), network_alive()]))
        .confirm(3.0)
        .inhibit(&[Phase::FirstEngineTakeoffPower, Phase::Above80Kt, Phase::LiftOff, Phase::Below800Ft, Phase::Touchdown]);
    a.failures = vec![6034001];
    r.alert(a);
    let mut a = EcamAlert::new("WIRED_ANTI_ICE_PROBE_2_FAULT", 34, "A-ICE PITOT TAT AOA PROBE 2 HEATG FAULT", Level::Caution, all(vec![any(vec![var("DEEP_PITOT_2_HEATER_FAILED").eq(1.0), var("DEEP_TAT_2_HEATER_FAILED").eq(1.0)]), network_alive()]))
        .confirm(3.0)
        .inhibit(&[Phase::FirstEngineTakeoffPower, Phase::Above80Kt, Phase::LiftOff, Phase::Below800Ft, Phase::Touchdown]);
    a.failures = vec![6034005];
    r.alert(a);
    let mut a = EcamAlert::new("WIRED_ANTI_ICE_PROBE_3_FAULT", 34, "A-ICE PITOT TAT AOA PROBE 3 HEATG FAULT", Level::Caution, all(vec![any(vec![var("DEEP_PITOT_3_HEATER_FAILED").eq(1.0), var("DEEP_TAT_3_HEATER_FAILED").eq(1.0)]), network_alive()]))
        .confirm(3.0)
        .inhibit(&[Phase::FirstEngineTakeoffPower, Phase::Above80Kt, Phase::LiftOff, Phase::Below800Ft, Phase::Touchdown]);
    a.failures = vec![6034009];
    r.alert(a);
    let mut a = EcamAlert::new("WIRED_ANTI_ICE_PROBE_STBY_FAULT", 34, "A-ICE STBY PITOT PROBE HEATG FAULT", Level::Caution, all(vec![var("DEEP_PITOT_4_HEATER_FAILED").eq(1.0), network_alive()]))
        .confirm(3.0)
        .inhibit(&[Phase::FirstEngineTakeoffPower, Phase::Above80Kt, Phase::LiftOff, Phase::Below800Ft, Phase::Touchdown]);
    a.failures = vec![6034013];
    r.alert(a);
    r.contribute("WIRED_ANTI_ICE_PROBE_1_FAULT").when(all(vec![any(vec![var("PROBE_HEAT_PITOT1_FAULT").eq(1.0), var("PROBE_HEAT_AOA1_FAULT").eq(1.0), var("PROBE_HEAT_TAT1_FAULT").eq(1.0)]), network_alive()])).raised_by(&vec![8030019, 8030020, 8030021, 8030028, 8030029, 8030030, 8030037, 8030038, 8030039]);
    r.contribute("WIRED_ANTI_ICE_PROBE_2_FAULT").when(all(vec![any(vec![var("PROBE_HEAT_PITOT2_FAULT").eq(1.0), var("PROBE_HEAT_AOA2_FAULT").eq(1.0), var("PROBE_HEAT_TAT2_FAULT").eq(1.0)]), network_alive()])).raised_by(&vec![8030022, 8030023, 8030024, 8030031, 8030032, 8030033, 8030040, 8030041, 8030042]);
    r.contribute("WIRED_ANTI_ICE_PROBE_3_FAULT").when(all(vec![any(vec![var("PROBE_HEAT_PITOT3_FAULT").eq(1.0), var("PROBE_HEAT_AOA3_FAULT").eq(1.0)]), network_alive()])).raised_by(&vec![8030025, 8030026, 8030027, 8030034, 8030035, 8030036]);
    let mut a = EcamAlert::new("WIRED_APU_FAULT", 49, "APU FAULT", Level::Caution, all(vec![any(vec![var("APU_ECB_TRIP").eq(1.0), all(vec![var("APU_START_PHASE").ge(1.0), var("APU_START_PHASE").le(2.0), var("DEEP_APU_N").lt(50.0)])]), network_alive()]))
        .confirm(60.0)
        .inhibit(&[Phase::FirstEngineTakeoffPower, Phase::Above80Kt, Phase::LiftOff, Phase::Below800Ft, Phase::Touchdown]);
    a.failures = vec![7049001, 7049002, 7049006, 7049007, 7049008, 7049009, 7049010, 7049011];
    r.alert(a);
    let mut a = EcamAlert::new("WIRED_DOOR_CARGO_16_NOT_CLOSED", 52, "DOOR FWD CARGO NOT CLOSED", Level::Caution, all(vec![any(vec![var("DEEP_PROX_DOOR_CARGO_16_CLOSED_NEAR").eq(0.0), var("DEEP_PROX_DOOR_CARGO_16_OPEN_NEAR").eq(1.0)]), network_alive()]))
        .confirm(3.0)
        .inhibit(&[Phase::FirstEngineTakeoffPower, Phase::Above80Kt, Phase::LiftOff, Phase::Below800Ft, Phase::Touchdown]);
    a.failures = vec![6052038, 6052040, 6052042];
    r.alert(a);
    let mut a = EcamAlert::new("WIRED_DOOR_CARGO_17_NOT_CLOSED", 52, "DOOR AFT CARGO NOT CLOSED", Level::Caution, all(vec![any(vec![var("DEEP_PROX_DOOR_CARGO_AFT_CLOSED_NEAR").eq(0.0), var("DEEP_PROX_DOOR_CARGO_AFT_OPEN_NEAR").eq(1.0)]), network_alive()]))
        .confirm(3.0)
        .inhibit(&[Phase::FirstEngineTakeoffPower, Phase::Above80Kt, Phase::LiftOff, Phase::Below800Ft, Phase::Touchdown]);
    a.failures = vec![6052044, 6052046, 6052048];
    r.alert(a);
    let mut a = EcamAlert::new("WIRED_ICE_DET_FAULT", 30, "A-ICE ICE DET FAULT", Level::Caution, all(vec![var("DEEP_ICE_DETECTOR_1").eq(1.0), var("DEEP_ICE_DETECTOR_2").eq(1.0), network_alive()]))
        .confirm(5.0)
        .inhibit(&[Phase::FirstEngineTakeoffPower, Phase::Above80Kt, Phase::LiftOff, Phase::Below800Ft, Phase::Touchdown]);
    a.failures = vec![6030001, 6030002, 6030003, 6030004, 6030005, 6030006];
    r.alert(a);
    r.contribute("ENV_HAIL_WINDOW_HEAT_1_FAULT").when(all(vec![var("WINDOW_HEAT_L_FAULT").eq(1.0), network_alive()])).raised_by(&[8030045]);
    r.contribute("ENV_HAIL_WINDOW_HEAT_2_FAULT").when(all(vec![var("WINDOW_HEAT_R_FAULT").eq(1.0), network_alive()])).raised_by(&[8030047, 8030048]);
    for eng in [1u16, 3, 4] {
        r.contribute(&format!("ENG_{eng}_ANTI_ICE_VLV_OPEN")).when(var(&format!("ANTI_ICE_NACELLE{eng}_OVERHEAT")).on());
    }
}
