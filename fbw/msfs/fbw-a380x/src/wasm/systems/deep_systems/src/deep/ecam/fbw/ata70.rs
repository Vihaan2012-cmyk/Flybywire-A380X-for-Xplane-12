use super::{phase, proc, sd_page, FbwProc};
use crate::deep::api::{all, any, not, var, Cond, Level};

fn network_alive() -> Cond {
    any(vec![
        var("ELEC_AC_1_BUS_IS_POWERED").on(),
        var("ELEC_AC_2_BUS_IS_POWERED").on(),
        var("ELEC_AC_3_BUS_IS_POWERED").on(),
        var("ELEC_AC_4_BUS_IS_POWERED").on(),
    ])
}

fn at_or_above_mct(eng: u32) -> Cond {
    var(&format!("A32NX_ENG_{eng}_TLA_DEG")).ge(36.7)
}

fn eng_to_power_signal(eng: u32) -> Cond {
    let tla = format!("A32NX_ENG_{eng}_TLA_DEG");
    let mct_band = all(vec![var(&tla).gt(33.3), var(&tla).lt(36.7)]);
    any(vec![at_or_above_mct(eng), all(vec![var("A32NX_TO_FLEX_TEMP_SET").on(), mct_band])])
}

const OIL_TEMP_MIN_TO_C: f64 = 50.0;

const N1_RED_LIMIT_PCT: f64 = 96.1;
const N2_RED_LIMIT_PCT: f64 = 97.8;

#[allow(dead_code)]
const TGT_TO_LIMIT_UNTRIMMED_C: f64 = 983.0;
const TGT_MCT_LIMIT_UNTRIMMED_C: f64 = 963.0;
const TGT_OVERTEMP_LIMIT_UNTRIMMED_C: f64 = 991.0;

const OIL_PRESS_MIN_PA: f64 = 25.0 * crate::physics::engine::oil::PSI_PA;

const OIL_TEMP_MAX_C: f64 = 177.0;

const CORE_RUNNING_FRACTION: f64 = 0.5;

fn core_running(eng: u32) -> Cond {
    any(vec![
        var(&format!("DEEP_ENG_{eng}_N3_PICKUP_A_FRAC")).gt(CORE_RUNNING_FRACTION),
        var(&format!("DEEP_ENG_{eng}_N3_PICKUP_B_FRAC")).gt(CORE_RUNNING_FRACTION),
    ])
}

pub fn wire(v: &mut Vec<FbwProc>) {
    for eng in 1..=4u32 {
        let i = u64::from(eng) - 1;

        for (chain, base) in [("A", 701_800_057u64), ("B", 701_800_061u64)] {
            v.push(
                proc(
                    base + i,
                    if chain == "A" { "ENG n IGN A FAULT" } else { "ENG n IGN B FAULT" },
                    Level::Advisory,
                    sd_page::ENG,
                    all(vec![
                        var(&format!("A32NX_ENG_{eng}_IGN_POWERED")).on(),
                        var(&format!("A32NX_ENG_{eng}_IGN_{chain}_SPARK_RATE_HZ")).le(0.0),
                    ]),
                    "the igniters are energised and this chain is producing no spark, while the other chain may still be firing",
                )
                .confirm(2.0)
                .inhibit(&[3, 4, 5, 6, 7, 8, 9, 10]),
            );
        }

        v.push(
            proc(
                701_800_065 + i,
                "ENG n IGN A+B FAULT",
                Level::Caution,
                sd_page::ENG,
                var(&format!("A32NX_ENG_{eng}_NO_IGNITION_AVAILABLE")).on(),
                "both ignition chains are energised and producing no spark at all, FCOM PRO-ABN-ECAM p.5792",
            )
            .confirm(1.0)
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]),
        );

        v.push(
            proc(
                701_800_085 + i,
                "ENG n OIL PRESS LO",
                Level::Warning,
                sd_page::ENG,
                all(vec![core_running(eng), var(&format!("DEEP_ENG_{eng}_OIL_PRESSURE_SENSED_PA")).lt(OIL_PRESS_MIN_PA)]),
                "the engine's own oil pressure transducer reads below the 25 psi EASA TCDS E.012 minimum while the HP spool is turning at running speed",
            )
            .confirm(2.0)
            .inhibit(&[1, 5, 6, 12]),
        );

        v.push(
            proc(
                701_800_093 + i,
                "ENG n OIL TEMP HI",
                Level::Caution,
                sd_page::ENG,
                all(vec![core_running(eng), var(&format!("DEEP_ENG_{eng}_OIL_TEMP_SENSED_C")).gt(OIL_TEMP_MAX_C)]),
                "the engine's own oil temperature transducer reads above the 177 C that FlyByWire's own A380X ENGINE SD page draws amber, with the HP spool turning",
            )
            .confirm(15.0)
            .inhibit(&[1, 4, 5, 6, 7, 9, 10, 12]),
        );

        v.push(
            proc(
                701_800_037 + i,
                "ENG n FUEL FILTER MONITORING FAULT",
                Level::Advisory,
                sd_page::ENG,
                var(&format!("A32NX_ENG_{eng}_FUEL_FILTER_MONITOR_FAULT")).on(),
                "the fuel filter's own bypass-warning monitor reads faulted, independent of the element's real clog state",
            )
            .confirm(5.0)
            .inhibit(&[phase::ELEC_PWR, phase::FIRST_ENG_STARTED, phase::SECOND_ENG_TO_POWER, phase::AT_OR_ABOVE_80_KT, phase::AT_OR_ABOVE_V1, phase::LIFT_OFF, phase::AT_OR_ABOVE_400_FT, phase::AT_OR_ABOVE_1500_FT, phase::AT_OR_BELOW_800_FT, phase::TOUCH_DOWN]),
        );

        v.push(
            proc(
                701_800_041 + i,
                "ENG n FUEL LEAK",
                Level::Caution,
                sd_page::ENG,
                all(vec![var(&format!("FUEL_ENG_LEAK_DETECTED:{eng}")).on(), network_alive()]),
                "deep::fuel's per-engine feed-line leak discrete, resolved to this specific engine rather than a wing side",
            )
            .confirm(0.0)
            .inhibit(&[phase::ELEC_PWR, phase::FIRST_ENG_STARTED, phase::SECOND_ENG_TO_POWER, phase::AT_OR_ABOVE_80_KT, phase::AT_OR_ABOVE_V1, phase::LIFT_OFF, phase::AT_OR_ABOVE_400_FT, phase::AT_OR_BELOW_800_FT, phase::TOUCH_DOWN, phase::AT_OR_BELOW_80_KT, phase::ENGINES_SHUTDOWN]),
        );

        v.push(
            proc(
                701_800_045 + i,
                "ENG n FUEL STRAINER CLOGGED",
                Level::Advisory,
                sd_page::ENG,
                var(&format!("A32NX_ENG_{eng}_FUEL_STRAINER_CLOGGED")).on(),
                "the fuel strainer's own bypass valve has cracked open at the FCOM's 12 psi differential (PRO-ABN-ECAM p.5787), upstream of the fine filter",
            )
            .confirm(5.0)
            .inhibit(&[phase::SECOND_ENG_TO_POWER, phase::AT_OR_ABOVE_80_KT, phase::AT_OR_ABOVE_V1, phase::LIFT_OFF, phase::AT_OR_ABOVE_400_FT, phase::AT_OR_ABOVE_1500_FT, phase::AT_OR_BELOW_800_FT, phase::TOUCH_DOWN]),
        );

        v.push(
            proc(
                701_800_049 + i,
                "ENG n FUEL SYS CONTAMINATION",
                Level::Caution,
                sd_page::ENG,
                all(vec![
                    any(vec![var(&format!("FUEL_ENG_CONTAMINATION_DETECTED:{eng}")).on(), var(&format!("A32NX_ENG_{eng}_FUEL_FILTER_BYPASSED")).on()]),
                    network_alive(),
                ]),
                "deep::fuel's per-engine filter free-water fraction above zero, resolved to this specific engine's own feed line; OR'd with the engine_accessories fuel filter's own bypass-valve-open discrete, the FCOM's other trigger for this same alert (PRO-ABN-ECAM p.5788, bypass valve of the fuel filter open)",
            )
            .confirm(0.0)
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]),
        );

        v.push(
            proc(
                701_800_089 + i,
                "ENG n OIL SYS CONTAMINATION",
                Level::Caution,
                sd_page::ENG,
                var(&format!("A32NX_ENG_{eng}_OIL_FILTER_BYPASSED")).on(),
                "FCOM p.5799: the return-line oil filter is clogged and its bypass valve is open; physics::engine::oil cracks the bypass once filter_clog drives the element's drop past FILTER_BYPASS_PSI",
            )
            .confirm(5.0)
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]),
        );

        v.push(
            proc(
                701_800_069 + i,
                "ENG n MINOR FAULT",
                Level::Advisory,
                sd_page::ENG,
                var(&format!("A32NX_ENG_{eng}_EEC_MAINTENANCE_FAULT")).on(),
                "the EEC's own backup (non-controlling) oil-temperature probe disagrees with the primary reading -- a real, deferred internal-sensor fault, standing in for the FCOM's own unspecified 'some internal engine sensor'",
            )
            .confirm(10.0)
            .inhibit(&[phase::FIRST_ENG_STARTED, phase::SECOND_ENG_TO_POWER, phase::AT_OR_ABOVE_80_KT, phase::AT_OR_ABOVE_V1, phase::LIFT_OFF, phase::AT_OR_ABOVE_400_FT, phase::AT_OR_ABOVE_1500_FT, phase::AT_OR_BELOW_800_FT, phase::TOUCH_DOWN]),
        );

        v.push(
            proc(
                701_800_101 + i,
                "ENG n OVTHR PROT LOST",
                Level::Advisory,
                sd_page::ENG,
                all(vec![
                    var(&format!("DEEP_ENG_{eng}_N3_PICKUP_A_VALID")).eq(0.0),
                    var(&format!("DEEP_ENG_{eng}_N3_PICKUP_B_VALID")).eq(0.0),
                    var(&format!("DEEP_ENG_{eng}_OIL_PRESSURE_SENSED_PA")).gt(OIL_PRESS_MIN_PA),
                ]),
                "both independent N3 (HP) speed-pickup channels read invalid while oil pressure proves the engine is actually turning -- the overthrust-protection function's own monitoring input lost",
            )
            .confirm(5.0)
            .inhibit(&[phase::FIRST_ENG_STARTED, phase::SECOND_ENG_TO_POWER, phase::AT_OR_ABOVE_80_KT, phase::AT_OR_ABOVE_V1, phase::LIFT_OFF, phase::AT_OR_ABOVE_400_FT, phase::AT_OR_ABOVE_1500_FT, phase::AT_OR_BELOW_800_FT, phase::TOUCH_DOWN]),
        );

        v.push(
            proc(
                701_800_129 + i,
                "ENG n THR LEVER FAULT",
                Level::Caution,
                sd_page::ENG,
                var(&format!("A32NX_ENG_{eng}_THR_LEVER_DISAGREE")).on(),
                "the thrust lever's second (channel B) position transducer disagrees with FlyByWire's own single TLA channel by more than the tolerance",
            )
            .confirm(2.0)
            .inhibit(phase::ENG_56),
        );

        v.push(
            proc(
                701_800_009 + i,
                "ENG n EGT OVER LIMIT",
                Level::Caution,
                sd_page::ENG,
                any(vec![
                    all(vec![core_running(eng), at_or_above_mct(eng), var(&format!("DEEP_ENG_{eng}_TGT_SENSED_C")).gt(TGT_OVERTEMP_LIMIT_UNTRIMMED_C)]),
                    all(vec![core_running(eng), not(at_or_above_mct(eng)), var(&format!("DEEP_ENG_{eng}_TGT_SENSED_C")).gt(TGT_MCT_LIMIT_UNTRIMMED_C)]),
                ]),
                "at or above MCT, the engine's own untrimmed TGT is past the EASA TCDS E.012 20 s over-temperature limit (991 C, FCOM's own T/O-GA/reverser/alpha-floor red-line regime); at or below MCT, it is past the TCDS's own MCT continuous limit (963 C, the FCOM's own at-or-below-MCT amber-line regime)",
            )
            .confirm(20.0)
            .inhibit(&[phase::AT_OR_ABOVE_80_KT, phase::AT_OR_ABOVE_V1, phase::LIFT_OFF]),
        );

        v.push(
            proc(
                701_800_073 + i,
                "ENG n N1/N2 OVER LIMIT",
                Level::Caution,
                sd_page::ENG,
                any(vec![
                    var(&format!("A32NX_ENG_{eng}_EEC_N1_SELECTED")).gt(N1_RED_LIMIT_PCT),
                    var(&format!("A32NX_ENG_{eng}_EEC_N2_SELECTED")).gt(N2_RED_LIMIT_PCT),
                ]),
                "the EEC's own selected N1 or N2 reading is above the FCOM's red-line limit (96.1%/97.8%, PRO-ABN-ECAM p.5794)",
            )
            .confirm(0.0)
            .inhibit(phase::ENG_56),
        );

        v.push(
            proc(
                701_800_097 + i,
                "ENG n OIL TEMP LO",
                Level::Caution,
                sd_page::ENG,
                all(vec![eng_to_power_signal(eng), var(&format!("DEEP_ENG_{eng}_OIL_TEMP_SENSED_C")).lt(OIL_TEMP_MIN_TO_C)]),
                "this engine's thrust lever is at or above take-off power while its own oil temperature reads below the FCOM's own 50 C threshold (PRO-ABN-ECAM p.5802)",
            )
            .confirm(0.0)
            .inhibit(&[phase::ELEC_PWR, phase::AT_OR_ABOVE_80_KT, phase::AT_OR_ABOVE_V1, phase::LIFT_OFF, phase::AT_OR_ABOVE_400_FT, phase::AT_OR_ABOVE_1500_FT, phase::AT_OR_BELOW_800_FT, phase::TOUCH_DOWN, phase::ENGINES_SHUTDOWN]),
        );

        v.push(
            proc(
                701_800_117 + i,
                "ENG n START FAULT",
                Level::Advisory,
                sd_page::ENG,
                any(vec![
                    all(vec![
                        var(&format!("A32NX_ENG_{eng}_START_VALVE_POSITION")).gt(0.5),
                        not(core_running(eng)),
                        var(&format!("A32NX_ENG_{eng}_TLA_DEG")).gt(2.0),
                    ]),
                    all(vec![
                        var(&format!("A32NX_ENG_{eng}_START_VALVE_POSITION")).gt(0.5),
                        var(&format!("A32NX_ENG_{eng}_STARTER_TORQUE_NM")).lt(50.0),
                    ]),
                    var(&format!("A32NX_ENG_{eng}_STARTER_DISINTEGRATED")).on(),
                ]),
                "a start is commanded (the start valve is open) and the core is not yet running while this engine's own thrust lever reads away from idle -- the FCOM's 'THR LEVERS NOT AT IDLE' start-fault cause, or the start valve open with the starter delivering no torque (FCOM p.5810 'no starter air pressure'; failures 2080002/3, 2080005/6, 2080008/9, 2080011/12), or the air turbine starter has disintegrated",
            )
            .confirm(2.0)
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]),
        );

        v.push(
            proc(
                701_800_125 + i,
                "ENG n START VLV FAULT (NOT OPEN)",
                Level::Caution,
                sd_page::ENG,
                all(vec![
                    var(&format!("A32NX_ENG_{eng}_START_VALVE_DISAGREE")).on(),
                    var(&format!("A32NX_ENG_{eng}_START_VALVE_POSITION")).le(0.5),
                ]),
                "the starter air valve is stuck and reads commanded open but the valve itself reads closed, FCOM PRO-ABN-ECAM p.5817",
            )
            .confirm(3.0)
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]),
        );
    }

    let airborne = all(vec![
        var("GEAR_LEG_COMPRESSION:1").lt(0.05),
        var("GEAR_LEG_COMPRESSION:2").lt(0.05),
        var("GEAR_LEG_COMPRESSION:3").lt(0.05),
        var("GEAR_LEG_COMPRESSION:4").lt(0.05),
        var("GEAR_LEG_COMPRESSION:5").lt(0.05),
    ]);
    v.push(
        proc(
            701_800_153,
            "ENG RELIGHT IN FLIGHT",
            Level::Caution,
            sd_page::ENG,
            any((1..=4u32)
                .map(|eng| {
                    all(vec![
                        var(&format!("A32NX_ENG_{eng}_IGN_POWERED")).on(),
                        not(core_running(eng)),
                        airborne.clone(),
                        var(&format!("DEEP_ENG_{eng}_TGT_SENSED_C")).gt(TGT_MCT_LIMIT_UNTRIMMED_C),
                    ])
                })
                .collect()),
            "ignition is energised on at least one engine, its core is not running and the aircraft is genuinely airborne (all five gear legs unloaded) -- a relight attempt, with the TGT above MCT's own untrimmed 963 C trim offset reused as the best-available analogue for the FCOM's 850 C trimmed relight limit",
        )
        .confirm(2.0)
        .inhibit(phase::NONE),
    );

    v.push(
        proc(
            701_800_156,
            "ENG TAIL PIPE FIRE",
            Level::Caution,
            sd_page::ENG,
            any((1..=4u32)
                .map(|eng| all(vec![not(core_running(eng)), var(&format!("A32NX_ENG_{eng}_HP_SOV_POSITION")).gt(0.05)]))
                .collect()),
            "the core has stopped turning (shutdown) while this engine's own HP fuel shut-off valve still reads substantially open -- unmetered fuel continuing to reach the hot section after the master lever was turned off, the FCOM's own recognition criterion (PRO-ABN-ECAM p.5853, 'the EGT may fail to decrease after the MASTER LEVER is turned off')",
        )
        .confirm(10.0)
        .inhibit(phase::NONE),
    );


    let lever_between_cl_and_mct = |eng: u32| {
        let tla = format!("A32NX_ENG_{eng}_TLA_DEG");
        all(vec![var(&tla).gt(22.9), var(&tla).le(33.3)])
    };
    v.push(
        proc(
            701_800_157,
            "ENG THR LEVERS NOT SET",
            Level::Caution,
            sd_page::ENG,
            any(vec![lever_between_cl_and_mct(1), lever_between_cl_and_mct(2), lever_between_cl_and_mct(3), lever_between_cl_and_mct(4)]),
            "at least one thrust lever is set between the CL/climb detent (22.9 deg) and the FLEX/MCT band (33.3 deg), FlyByWire's own detents (FwsFlightPhases.ts) and the FCOM's own first triggering clause (PRO-ABN-ECAM p.5860); the FCOM's second clause (a FADEC-selected take-off mode discrete) has no equivalent in this model and is not wired",
        )
        .confirm(0.0)
        .inhibit(&[phase::ELEC_PWR, phase::AT_OR_ABOVE_80_KT, phase::AT_OR_ABOVE_V1, phase::LIFT_OFF, phase::AT_OR_ABOVE_400_FT, phase::AT_OR_ABOVE_1500_FT, phase::AT_OR_BELOW_800_FT, phase::TOUCH_DOWN, phase::AT_OR_BELOW_80_KT, phase::ENGINES_SHUTDOWN]),
    );

    let mut energized_conds = Vec::new();
    for eng in [2u32, 3u32] {
        energized_conds.push(var(&format!("A32NX_ENG_{eng}_REV_ENERGIZED")).on());

        v.push(
            proc(
                if eng == 2 { 701_800_137 } else { 701_800_138 },
                "ENG n REVERSER CTL FAULT",
                Level::Caution,
                sd_page::ENG,
                var(&format!("A32NX_ENG_{eng}_REV_CTL_FAULT")).on(),
                "the reverser's own EEC-side control loop reads faulted, holding the sleeve at its last position",
            )
            .confirm(2.0)
            .inhibit(&[phase::SECOND_ENG_TO_POWER, phase::AT_OR_ABOVE_80_KT, phase::AT_OR_ABOVE_V1, phase::LIFT_OFF, phase::AT_OR_ABOVE_400_FT]),
        );

        v.push(
            proc(
                if eng == 2 { 701_800_139 } else { 701_800_140 },
                "ENG n REVERSER ENERGIZED",
                Level::Caution,
                sd_page::ENG,
                var(&format!("A32NX_ENG_{eng}_REV_ENERGIZED")).on(),
                "the reverse-thrust lever is selected for this engine, independent of the sleeve's actual position",
            )
            .confirm(0.0)
            .inhibit(&[4, 5, 6, 7, 9, 10]),
        );

        v.push(
            proc(
                if eng == 2 { 701_800_143 } else { 701_800_144 },
                "ENG n REVERSER INHIBITED",
                Level::Advisory,
                sd_page::ENG,
                var(&format!("A32NX_ENG_{eng}_REV_MEL_INOP")).on(),
                "this reverser's thrust-reverser-lock-fault item is currently deferred under the MEL -- a maintenance action, not an in-flight condition",
            )
            .confirm(0.0)
            .inhibit(&[phase::SECOND_ENG_TO_POWER, phase::AT_OR_ABOVE_80_KT, phase::AT_OR_ABOVE_V1, phase::LIFT_OFF, phase::AT_OR_ABOVE_400_FT, phase::AT_OR_ABOVE_1500_FT, phase::AT_OR_BELOW_800_FT, phase::TOUCH_DOWN]),
        );

        v.push(
            proc(
                if eng == 2 { 701_800_145 } else { 701_800_146 },
                "ENG n REV LOCKED",
                Level::Advisory,
                sd_page::ENG,
                all(vec![
                    var(&format!("A32NX_ENG_{eng}_REV_POSITION")).le(0.05),
                    var(&format!("A32NX_ENG_{eng}_REV_LOCK_DEGRADED_COUNT")).eq(0.0),
                    var(&format!("A32NX_ENG_{eng}_REV_CTL_FAULT")).on(),
                ]),
                "the reverser's control has failed and its sleeve is held stowed and locked",
            )
            .confirm(1.0)
            .inhibit(&[phase::SECOND_ENG_TO_POWER, phase::AT_OR_ABOVE_80_KT, phase::AT_OR_ABOVE_V1, phase::LIFT_OFF, phase::AT_OR_ABOVE_400_FT, phase::AT_OR_ABOVE_1500_FT, phase::AT_OR_BELOW_800_FT]),
        );

        v.push(
            proc(
                if eng == 2 { 701_800_147 } else { 701_800_148 },
                "ENG n REVERSER MINOR FAULT",
                Level::Advisory,
                sd_page::ENG,
                all(vec![
                    var(&format!("A32NX_ENG_{eng}_REV_LOCK_DEGRADED_COUNT")).eq(1.0),
                    var(&format!("A32NX_ENG_{eng}_REV_POSITION_DISAGREE")).off(),
                ]),
                "exactly one of this reverser's three independent locks is degraded while the sleeve position still agrees with its command",
            )
            .confirm(5.0)
            .inhibit(&[phase::ELEC_PWR, phase::FIRST_ENG_STARTED, phase::SECOND_ENG_TO_POWER, phase::AT_OR_ABOVE_80_KT, phase::AT_OR_ABOVE_V1, phase::LIFT_OFF, phase::AT_OR_ABOVE_400_FT, phase::AT_OR_ABOVE_1500_FT, phase::AT_OR_BELOW_800_FT, phase::TOUCH_DOWN]),
        );

        v.push(
            proc(
                if eng == 2 { 701_800_141 } else { 701_800_142 },
                "ENG n REVERSER FAULT",
                Level::Caution,
                sd_page::ENG,
                var(&format!("A32NX_ENG_{eng}_REV_POSITION_DISAGREE")).on(),
                "the reverser sleeve's sensed position disagrees with its command -- a jammed actuator or control-loop lock, replacing the deep registry's own synthetic-id copy of this same condition",
            )
            .confirm(3.0)
            .inhibit(&[phase::SECOND_ENG_TO_POWER, phase::AT_OR_ABOVE_80_KT, phase::AT_OR_ABOVE_V1, phase::LIFT_OFF, phase::AT_OR_ABOVE_400_FT]),
        );

        v.push(
            proc(
                if eng == 2 { 701_800_149 } else { 701_800_150 },
                "ENG n REVERSER UNLOCKED",
                Level::Warning,
                sd_page::ENG,
                var(&format!("A32NX_ENG_{eng}_REV_UNCOMMANDED")).on(),
                "all three independent reverser locks have failed to hold together, an uncommanded deployment risk; replacing the deep registry's own synthetic-id copy of this same condition",
            )
            .inhibit(&[phase::AT_OR_ABOVE_V1, phase::LIFT_OFF]),
        );
    }

    v.push(
        proc(
            701_800_154,
            "ENG REVERSER SELECTED",
            Level::Advisory,
            sd_page::ENG,
            all(vec![any(energized_conds), airborne]),
            "either reverser-carrying engine's lever is selected to reverse thrust while the aircraft is genuinely airborne (all five gear legs unloaded), matching the FCOM's own 'in flight' wording",
        )
        .confirm(0.0)
        .inhibit(&[phase::ELEC_PWR, phase::FIRST_ENG_STARTED, phase::SECOND_ENG_TO_POWER, phase::AT_OR_ABOVE_80_KT, phase::AT_OR_ABOVE_V1, phase::LIFT_OFF, phase::TOUCH_DOWN, phase::AT_OR_BELOW_80_KT, phase::ENGINES_SHUTDOWN]),
    );

    for eng in 1..=4u32 {
        let i = u64::from(eng) - 1;

        v.push(
            proc(
                701_800_021 + i,
                "ENG n FADEC SYS FAULT",
                Level::Caution,
                sd_page::ENG,
                all(vec![var(&format!("A32NX_ENG_{eng}_EEC_CHANNEL_FAULT")).eq(1.0), network_alive()]),
                "FCOM ENG 1(2)(3)(4) FADEC SYS FAULT, PRO-ABN-ECAM p.5774: a single EEC channel dead still leaves the other in control but is the FADEC failure the FCOM describes; retitled from the invented 'ENG n EEC CHANNEL FAULT' and level corrected from Advisory to Caution to match the FCOM master-caution image",
            )
            .confirm(3.0)
            .inhibit(&[3, 4, 5, 6, 7, 8, 9, 10]),
        );

        v.push(
            proc(
                701_800_053 + i,
                "ENG n HP FUEL VLV FAULT",
                Level::Caution,
                sd_page::ENG,
                all(vec![var(&format!("A32NX_ENG_{eng}_HP_SOV_DISAGREE")).on(), network_alive()]),
                "FCOM ENG 1(2)(3)(4) HP FUEL VLV FAULT, PRO-ABN-ECAM p.5789: the HP shut-off valve's sensed position disagrees with its command, failed either open or closed; retitled from the invented 'ENG n HP SOV FAULT' and level corrected from Warning to Caution to match the FCOM master-caution image",
            )
            .confirm(3.0)
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]),
        );

        v.push(
            proc(
                701_800_133 + i,
                "ENG n THRUST LOSS",
                Level::Caution,
                sd_page::ENG,
                all(vec![
                    any(vec![var(&format!("A32NX_ENG_{eng}_THRUST_ABNORMAL")).on(), var(&format!("A32NX_ENG_{eng}_HP_PUMP_LOW_FLOW")).on()]),
                    network_alive(),
                ]),
                "FCOM ENG 1(2)(3)(4) THRUST LOSS, PRO-ABN-ECAM p.5822: the FMU's own commanded-vs-metered flow error past the thrust-affecting tolerance, OR'd with the HP pump's own low-flow discrete -- both are the real engine-accessories causes of a genuine thrust shortfall, folding in the invented 'ENG n FADEC FUEL METERING FAULT' and 'ENG n HP FUEL PUMP FAULT' registry alerts",
            )
            .confirm(5.0)
            .inhibit(&[1, 2, 4, 5, 6, 7, 8, 9, 10, 11, 12]),
        );
    }
}
