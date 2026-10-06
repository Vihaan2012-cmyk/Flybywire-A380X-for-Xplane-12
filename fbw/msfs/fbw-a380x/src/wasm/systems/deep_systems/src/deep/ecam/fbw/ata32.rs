use super::{item, phase, proc, sd_page, FbwProc};
use crate::deep::api::{all, any, not, var, Cmp, Cond, Level};

fn network_alive() -> Cond {
    any(vec![
        var("ELEC_AC_1_BUS_IS_POWERED").on(),
        var("ELEC_AC_2_BUS_IS_POWERED").on(),
        var("ELEC_AC_3_BUS_IS_POWERED").on(),
        var("ELEC_AC_4_BUS_IS_POWERED").on(),
    ])
}

const HYD_SOURCE_AVAILABLE_PSI: f64 = 0.3 * 5000.0;
const MIN_HOLDING_PA: f64 = 4.0e6;
const MAX_NOSE_ANGLE_DEG: f64 = 70.0;
use crate::deep::gear_structure::live::ALTN_STEER_SYS_HOT_C;

fn norm_brk_available() -> Cond {
    var("HYD_GREEN_MANIFOLD_PRESSURE_PSI").gt(HYD_SOURCE_AVAILABLE_PSI)
}
fn altn_brk_available() -> Cond {
    var("HYD_YELLOW_MANIFOLD_PRESSURE_PSI").gt(HYD_SOURCE_AVAILABLE_PSI)
}
fn emer_brk_available() -> Cond {
    var("PARK_BRAKE_PRESS_PA").gt(MIN_HOLDING_PA)
}
fn norm_steer_available() -> Cond {
    var("HYD_YELLOW_MANIFOLD_PRESSURE_PSI").gt(HYD_SOURCE_AVAILABLE_PSI)
}
fn altn_steer_available() -> Cond {
    var("HYD_GREEN_MANIFOLD_PRESSURE_PSI").gt(HYD_SOURCE_AVAILABLE_PSI)
}

fn any_leg_on_ground() -> Cond {
    any(LEGS.iter().map(|n| var(&format!("SENSED_ON_GROUND:{n}")).on()).collect())
}

const LEGS: [u32; 5] = [1, 2, 3, 4, 5];

fn lever_up() -> Cond {
    var("GEAR_LEVER_SELECTED_DOWN").off()
}

fn lever_down() -> Cond {
    var("GEAR_LEVER_SELECTED_DOWN").on()
}

const TYRE_LOW_PA: f64 = 0.9 * crate::physics::tyre::COLD_PRESSURE_PA;

fn tyre_pressure_vars() -> Vec<String> {
    let mut v: Vec<String> = crate::deep::live::all_areas().published_names().into_iter().filter(|n| n.contains("DEEP_TYRE_PRESSURE_COLD_EQUIV_PA:")).collect();
    v.sort();
    v.dedup();
    v
}

const ANTISKID_GEAR_WHEELS: [[u32; 4]; 4] = [[1, 2, 5, 6], [3, 4, 7, 8], [9, 10, 13, 14], [11, 12, 15, 16]];

fn antiskid_lost(gear: usize) -> Cond {
    all(ANTISKID_GEAR_WHEELS[gear].iter().map(|n| var(&format!("ANTISKID_CHANNEL_FAULT:{n}")).on()).collect())
}

fn antiskid_lost_on_all() -> Cond {
    any(vec![
        all(vec![var("BSCU_CHANNEL_FAULT:1").on(), var("BSCU_CHANNEL_FAULT:2").on()]),
        all((0..4).map(antiskid_lost).collect()),
    ])
}

fn antiskid_combination(wings: bool, left_body: bool, right_body: bool) -> Cond {
    let wing_cond = all(vec![antiskid_lost(0), antiskid_lost(1)]);
    all(vec![
        if wings { wing_cond } else { not(wing_cond) },
        if left_body { antiskid_lost(2) } else { not(antiskid_lost(2)) },
        if right_body { antiskid_lost(3) } else { not(antiskid_lost(3)) },
        not(antiskid_lost_on_all()),
        network_alive(),
    ])
}

fn wire_antiskid(v: &mut Vec<FbwProc>) {
    v.push(
        proc(320_800_001, "BRAKES A-SKID FAULT ON ALL L/G", Level::Caution, sd_page::WHEEL, all(vec![antiskid_lost_on_all(), network_alive()]), "FCOM PRO-ABN-ECAM p.5423: BCS 1 and BCS 2 failed, or no gear keeps its antiskid")
            .confirm(1.0)
            .inhibit(&[4, 5, 6, 7])
            .items(2, Vec::new()),
    );
    for (id, title, wings, left_body, right_body, page) in [
        (320_800_002u64, "BRAKES A-SKID FAULT ON L + R BODY L/G", false, true, true, "FCOM PRO-ABN-ECAM p.5425"),
        (320_800_003, "BRAKES A-SKID FAULT ON LEFT BODY L/G", false, true, false, "FCOM PRO-ABN-ECAM p.5428"),
        (320_800_004, "BRAKES A-SKID FAULT ON RIGHT BODY L/G", false, false, true, "FCOM PRO-ABN-ECAM p.5428"),
        (320_800_005, "BRAKES A-SKID FAULT ON WING + L BODY L/GS", true, true, false, "FCOM PRO-ABN-ECAM p.5431"),
        (320_800_006, "BRAKES A-SKID FAULT ON WING + R BODY L/GS", true, false, true, "FCOM PRO-ABN-ECAM p.5431"),
        (320_800_007, "BRAKES A-SKID FAULT ON WING L/GS", true, false, false, "FCOM PRO-ABN-ECAM p.5434"),
    ] {
        v.push(
            proc(id, title, Level::Caution, sd_page::WHEEL, antiskid_combination(wings, left_body, right_body), page)
                .confirm(1.0)
                .inhibit(&[4, 5, 6, 7])
                .items(2, Vec::new()),
        );
    }
}

pub fn wire(v: &mut Vec<FbwProc>) {
    wire_antiskid(v);
    v.push(
        proc(
            320_800_040,
            "L/G GEAR NOT LOCKED UP",
            Level::Caution,
            sd_page::WHEEL,
            all(vec![lever_up(), any(LEGS.iter().map(|n| var(&format!("GEAR_UPLOCKED:{n}")).off()).collect())]),
            "the lever is up and at least one of the five legs has not up-locked well after the modelled retraction time",
        )
        .confirm(30.0)
        .inhibit(&[2, 3, 4, 5, 6, 10, 11])
        .items(
            7,
            vec![item(2).checked(lever_down())],
        ),
    );




    v.push(
        proc(
            320_800_023,
            "BRAKES PARK BRK PRESS LO",
            Level::Caution,
            sd_page::WHEEL,
            all(vec![var("PARK_BRAKE_SET").on(), var("PARK_BRAKE_HOLDING").off()]),
            "the park brake is set but not holding pressure, moved from the registry's own L_G_PARK_BRK_LO_PR",
        )
        .confirm(3.0)
        .inhibit(&[3, 4, 5, 6, 7, 8, 9, 10])
        .items(4, Vec::new()),
    );

    v.push(
        proc(
            320_800_036,
            "L/G DOORS NOT CLOSED",
            Level::Caution,
            sd_page::WHEEL,
            all(vec![
                lever_up(),
                all(LEGS.iter().map(|n| var(&format!("GEAR_UPLOCKED:{n}")).on()).collect()),
                any(LEGS.iter().map(|n| var(&format!("GEAR_DOOR_POSITION:{n}")).gt(0.05)).collect()),
            ]),
            "retraction is complete on all five legs but a gear door is still open by more than 5 % of its travel",
        )
        .confirm(20.0)
        .inhibit(&[4, 5, 6, 10, 11])
        .items(12, vec![item(7).checked(lever_down()), item(9).checked(lever_up())]),
    );

    v.push(
        proc(
            320_800_063,
            "WHEEL TIRE PRESS LO",
            Level::Caution,
            sd_page::WHEEL,
            any(tyre_pressure_vars().iter().map(|n| var(n).lt(TYRE_LOW_PA)).collect()),
            "any wheel's own pressure transducer reading, taken back to 15 C (the TPIS's cold-equivalent pressure), more than 10 % below the cold service pressure physics::tyre models",
        )
        .confirm(10.0)
        .inhibit(&[3, 4, 5, 6, 7, 10]),
    );

    v.push(
        proc(
            320_800_009,
            "BRAKES ACCU PRESS LO",
            Level::Caution,
            sd_page::WHEEL,
            all(vec![var("PARK_BRAKE_PRESS_PA").lt(MIN_HOLDING_PA), network_alive()]),
            "the parking-brake accumulator's own pressure below gear_structure::brakes::MIN_HOLDING_PA, ungated on PARK_BRAKE_SET (this id's own FCOM text is unconditional); FCOM PRO-ABN-ECAM p.5440",
        )
        .confirm(3.0)
        .inhibit(&[3, 4, 5, 6, 7, 9, 10])
        .items(4, Vec::new()),
    );

    let lgciu1_dead = || {
        any(vec![
            var("ELEC_LOAD_lgciu-1_POWERED").off(),
            var("A32NX_LGCIU_1_INTERNAL_FAULT").on(),
        ])
    };
    let lgciu2_dead = || {
        any(vec![
            var("ELEC_LOAD_lgciu-2_POWERED").off(),
            var("A32NX_LGCIU_2_INTERNAL_FAULT").on(),
        ])
    };
    v.push(
        proc(320_800_033, "L/G CTL 1 FAULT", Level::Advisory, sd_page::WHEEL, all(vec![lgciu1_dead(), network_alive()]), "electrical::loads.rs's own lgciu-1 load unpowered, or the LGCIU's own internal_error_failure (F32002) flipping LGCIU_1_INTERNAL_FAULT; FCOM PRO-ABN-ECAM p.5475")
            .confirm(1.0)
            .inhibit(&[3, 4, 5, 6, 7, 9, 10, 11])
            .suppressed_by(&[320_800_035]),
    );
    v.push(
        proc(320_800_034, "L/G CTL 2 FAULT", Level::Advisory, sd_page::WHEEL, all(vec![lgciu2_dead(), network_alive()]), "as 320800033, lgciu-2 / F32003")
            .confirm(1.0)
            .inhibit(&[3, 4, 5, 6, 7, 9, 10, 11])
            .suppressed_by(&[320_800_035]),
    );
    v.push(
        proc(320_800_035, "L/G CTL 1+2 FAULT", Level::Caution, sd_page::WHEEL, all(vec![lgciu1_dead(), lgciu2_dead(), network_alive()]), "both LGCIUs unpowered or internally faulted together; FCOM PRO-ABN-ECAM p.5476")
            .confirm(1.0)
            .inhibit(&[4, 5, 6, 7, 9, 10])
            .items(2, Vec::new()),
    );

    v.push(
        proc(
            320_800_041,
            "L/G GEAR UPLOCK FAULT",
            Level::Advisory,
            sd_page::WHEEL,
            all(vec![lever_down(), any(LEGS.iter().map(|n| var(&format!("GEAR_STUCK_LOCKED:{n}")).on()).collect())]),
            "the lever is down and a leg's uplock hook resists release (retraction::Retraction's own stuck_locked semantics); FCOM PRO-ABN-ECAM p.5490",
        )
        .inhibit(&[3, 4, 5, 6, 7, 8, 9, 10, 11])
        .items(1, Vec::new()),
    );

    v.push(
        proc(
            320_800_010,
            "BRAKES ALTN + EMER BRK FAULT",
            Level::Advisory,
            sd_page::WHEEL,
            all(vec![not(altn_brk_available()), not(emer_brk_available()), network_alive()]),
            "both the yellow (ALTN) manifold and the parking-brake accumulator (EMER) below their own availability thresholds; FCOM PRO-ABN-ECAM p.5442",
        )
        .confirm(1.0)
        .inhibit(&[3, 4, 5, 6, 7, 9, 10])
        .items(1, Vec::new()),
    );
    v.push(
        proc(
            320_800_011,
            "BRAKES ALTN BRK FAULT",
            Level::Advisory,
            sd_page::WHEEL,
            all(vec![any(vec![not(altn_brk_available()), var("BSCU_CHANNEL_FAULT:2").on()]), network_alive()]),
            "the yellow (ALTN) manifold below its availability threshold, or the BSCU's own channel-2 BITE; FCOM PRO-ABN-ECAM p.5444",
        )
        .confirm(1.0)
        .inhibit(&[3, 4, 5, 6, 7, 9, 10])
        .items(1, Vec::new()),
    );
    v.push(
        proc(320_800_012, "BRAKES ALTN BRK PRESS MONITORING FAULT", Level::Advisory, sd_page::WHEEL, all(vec![var("ALTN_BRK_PRESS_SENSOR_FAULT").on(), network_alive()]), "the BSCU's own alternate brake pressure sensor BITE; FCOM PRO-ABN-ECAM p.5446")
            .confirm(1.0)
            .inhibit(&[3, 4, 5, 6, 7, 8, 9, 10, 11]),
    );
    v.push(
        proc(320_800_013, "BRAKES AUTO BRK FAULT", Level::Caution, sd_page::WHEEL, all(vec![var("AUTO_BRK_FAULT").on(), network_alive()]), "the BSCU's own autobrake function BITE; FCOM PRO-ABN-ECAM p.5447")
            .confirm(1.0)
            .inhibit(&[3, 4, 5, 6, 7]),
    );
    v.push(
        proc(320_800_015, "BRAKES CTL 1 FAULT", Level::Advisory, sd_page::WHEEL, all(vec![var("BSCU_CHANNEL_FAULT:1").on(), network_alive()]), "the BSCU's own control-channel-1 BITE; FCOM PRO-ABN-ECAM p.5452")
            .confirm(1.0)
            .inhibit(&[3, 4, 5, 6, 7, 9, 10, 11]),
    );
    v.push(
        proc(320_800_016, "BRAKES CTL 2 FAULT", Level::Advisory, sd_page::WHEEL, all(vec![var("BSCU_CHANNEL_FAULT:2").on(), network_alive()]), "as 320800015, control channel 2")
            .confirm(1.0)
            .inhibit(&[3, 4, 5, 6, 7, 9, 10, 11]),
    );
    v.push(
        proc(
            320_800_017,
            "BRAKES EMER BRK FAULT",
            Level::Advisory,
            sd_page::WHEEL,
            all(vec![not(emer_brk_available()), network_alive()]),
            "the parking-brake accumulator below MIN_HOLDING_PA -- shares its root cause honestly with 320800009/320800010's own accumulator term; FCOM PRO-ABN-ECAM p.5453",
        )
        .confirm(1.0)
        .inhibit(&[3, 4, 5, 6, 7, 9, 10])
        .items(1, Vec::new()),
    );
    v.push(
        proc(320_800_019, "BRAKES MINOR FAULT", Level::Advisory, sd_page::WHEEL, all(vec![var("BRAKES_MINOR_FAULT").on(), network_alive()]), "a wheel's own antiskid_inop fault armed but below the channel's own BITE threshold; FCOM PRO-ABN-ECAM p.5456")
            .confirm(1.0)
            .inhibit(&[3, 4, 5, 6, 7, 8, 9, 10, 11]),
    );
    v.push(
        proc(
            320_800_020,
            "BRAKES NORM BRK FAULT",
            Level::Advisory,
            sd_page::WHEEL,
            all(vec![any(vec![not(norm_brk_available()), var("BSCU_CHANNEL_FAULT:1").on()]), network_alive()]),
            "the green (NORM) manifold below its availability threshold, or the BSCU's own channel-1 BITE; FCOM PRO-ABN-ECAM p.5457",
        )
        .confirm(1.0)
        .inhibit(&[3, 4, 5, 6, 7, 10, 11])
        .items(1, Vec::new()),
    );
    v.push(
        proc(320_800_021, "BRAKES NORM BRK PRESS MONITORING FAULT", Level::Advisory, sd_page::WHEEL, all(vec![var("NORM_BRK_PRESS_SENSOR_FAULT").on(), network_alive()]), "the BSCU's own normal brake pressure sensor BITE; FCOM PRO-ABN-ECAM p.5459")
            .confirm(1.0)
            .inhibit(&[3, 4, 5, 6, 7, 8, 9, 10, 11]),
    );
    v.push(
        proc(
            320_800_024,
            "BRAKES PEDAL BRAKING FAULT",
            Level::Caution,
            sd_page::WHEEL,
            all(vec![any(vec![var("BRAKE_PEDAL_SENSOR_FAULT:1").on(), var("BRAKE_PEDAL_SENSOR_FAULT:2").on()]), network_alive()]),
            "either brake pedal position transducer's own BITE; FCOM PRO-ABN-ECAM p.5463",
        )
        .confirm(1.0)
        .inhibit(&[4, 5, 6, 7])
        .items(2, Vec::new()),
    );
    v.push(
        proc(
            320_800_025,
            "BRAKES RELEASED",
            Level::Caution,
            sd_page::WHEEL,
            all(vec![
                var("BRAKE_PEDAL_COMMANDED_FRACTION").gt(0.1),
                all((1..=16).map(|n| var(&format!("BRAKE_APPLIED_FRACTION:{n}")).lt(0.05)).collect()),
                any_leg_on_ground(),
                network_alive(),
            ]),
            "commanded braking (BRAKE_PEDAL_COMMANDED_FRACTION, a sound if not exhaustive subset of the FCOM's own pedal-or-autobrake demand) that produced no applied force on any of the sixteen braked wheels; FCOM PRO-ABN-ECAM p.5465",
        )
        .confirm(1.0)
        .inhibit(&[4, 5, 6, 7])
        .items(1, Vec::new()),
    );
    v.push(
        proc(
            320_800_026,
            "BRAKES RESIDUAL BRAKING",
            Level::Caution,
            sd_page::WHEEL,
            all(vec![
                var("BRAKE_PEDAL_COMMANDED_FRACTION").lt(0.05),
                any((1..=16).map(|n| var(&format!("BRAKE_APPLIED_FRACTION:{n}")).gt(0.3)).collect()),
                any_leg_on_ground(),
                network_alive(),
            ]),
            "a wheel still applying real braking force with the pedals released -- the already-modelled dragging fault's real effect, now visible through BRAKE_APPLIED_FRACTION; FCOM PRO-ABN-ECAM p.5467",
        )
        .confirm(1.0)
        .inhibit(&[4, 5, 6, 7, 9, 10])
        .items(8, Vec::new()),
    );
    v.push(
        proc(320_800_027, "BRAKES SEL VLV JAMMED OPEN", Level::Advisory, sd_page::WHEEL, all(vec![var("BRAKE_SEL_VLV_JAMMED").on(), network_alive()]), "the BSCU's own brake selector valve BITE; FCOM PRO-ABN-ECAM p.5468")
            .confirm(1.0)
            .inhibit(&[3, 4, 5, 6, 7, 8, 9, 10, 11]),
    );
    v.push(
        proc(
            320_800_028,
            "BRAKES SYS REDUNDANCY LOST",
            Level::Advisory,
            sd_page::WHEEL,
            all(vec![
                any(vec![not(norm_brk_available()), var("BSCU_CHANNEL_FAULT:1").on()]),
                any(vec![not(altn_brk_available()), var("BSCU_CHANNEL_FAULT:2").on()]),
                emer_brk_available(),
                network_alive(),
            ]),
            "both NORM and ALTN degraded, down to EMER as the last available system; FCOM PRO-ABN-ECAM p.5469",
        )
        .confirm(1.0)
        .inhibit(&[3, 4, 5, 6, 7, 8, 9, 10, 11])
        .items(5, Vec::new()),
    );

    v.push(
        proc(
            320_800_031,
            "L/G ABNORM OLEO PRESS",
            Level::Advisory,
            sd_page::WHEEL,
            all(vec![any(LEGS.iter().map(|n| var(&format!("GEAR_STRUT_GAS_CHARGE_FRACTION:{n}")).lt(0.9)).collect()), network_alive()]),
            "a leg's own strut gas-charge fraction more than 10% under its reference charge, the same GENERIC industry-servicing-band reasoning TYRE_LOW_PA above already documents; FCOM PRO-ABN-ECAM p.5472",
        )
        .confirm(10.0)
        .inhibit(&[1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11]),
    );
    v.push(
        proc(
            320_800_032,
            "L/G BOGIE POSITION FAULT",
            Level::Advisory,
            sd_page::WHEEL,
            all(vec![lever_up(), any(vec![var("BOGIE_TRIMMED:4").off(), var("BOGIE_TRIMMED:5").off()])]),
            "a body leg's bogie-beam trim/levelling actuator failing to trim before retraction, a second gate of the same kind retraction.rs's own door-open interlock already is; FCOM PRO-ABN-ECAM p.5473",
        )
        .confirm(10.0)
        .inhibit(&[2, 3, 4, 5, 6, 7, 8, 9, 10, 11])
        .items(1, Vec::new()),
    );
    v.push(
        proc(
            320_800_042,
            "L/G GRVTY EXTN FAULT",
            Level::Advisory,
            sd_page::WHEEL,
            all(vec![var("GRAVITY_EXTEND_SELECTED").on(), any(LEGS.iter().map(|n| var(&format!("GEAR_STUCK_LOCKED:{n}")).on()).collect())]),
            "gravity extension selected but a leg is still stuck locked well past the FCOM's own ~70s figure (retraction.rs's GRAVITY_EXTEND_TOTAL_S, already cited there); FCOM PRO-ABN-ECAM p.5492",
        )
        .confirm(70.0)
        .inhibit(&[3, 4, 5, 6, 7, 8, 9, 10, 11]),
    );
    v.push(
        proc(
            320_800_043,
            "L/G OLEO PRESS MONITORING FAULT",
            Level::Advisory,
            sd_page::WHEEL,
            all(vec![any(LEGS.iter().map(|n| var(&format!("GEAR_STRUT_PRESS_SENSOR_FAULT:{n}")).on()).collect()), network_alive()]),
            "a leg's own strut pressure-sensing/monitoring BITE, independent of the real charge fraction; no FCOM title match, level/inhibit taken from 320800031's own FCOM-confirmed sibling (same physical family)",
        )
        .confirm(1.0)
        .inhibit(&[1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11]),
    );
    v.push(
        proc(
            320_800_044,
            "L/G RETRACTION FAULT",
            Level::Caution,
            sd_page::WHEEL,
            all(vec![lever_up(), any(vec![var("BODY_STEER_ANGLE_DEG:1").gt(2.0), var("BODY_STEER_ANGLE_DEG:1").lt(-2.0), var("BODY_STEER_ANGLE_DEG:2").gt(2.0), var("BODY_STEER_ANGLE_DEG:2").lt(-2.0)])]),
            "retraction commanded while a body leg's rear-axle steering has not centred (the catalogue's own procedure text names 'the body wheel steering is not locked' as a precondition), a same-area cross-read of steering.rs's own published angle; FCOM PRO-ABN-ECAM p.5512",
        )
        .confirm(10.0)
        .inhibit(&[4, 5, 8, 9, 10, 11])
        .items(13, Vec::new()),
    );
    v.push(
        proc(
            320_800_046,
            "L/G WEIGHT ON WHEELS FAULT",
            Level::Advisory,
            sd_page::WHEEL,
            all(vec![any(LEGS.iter().map(|n| Cond::VarVar { a: format!("SENSED_ON_GROUND:{n}"), cmp: Cmp::Ne, b: format!("TRUE_ON_GROUND:{n}") }).collect()), network_alive()]),
            "a leg's sensed ground-contact state disagreeing with its true state (retraction.rs's own sensor_lies pattern, applied to weight-on-wheels sensing); FCOM PRO-ABN-ECAM p.5516",
        )
        .confirm(2.0)
        .inhibit(&[2, 3, 4, 5, 6, 7, 8, 9, 10, 11]),
    );

    v.push(
        proc(
            320_800_047,
            "STEER ALTN N/W STEER FAULT",
            Level::Advisory,
            sd_page::WHEEL,
            all(vec![any(vec![not(altn_steer_available()), var("STEER_CTL_FAULT:2").on()]), network_alive()]),
            "the green (ALTN) nosewheel-steering source below its availability threshold, or the SSC's own channel-2 BITE; FCOM PRO-ABN-ECAM p.5517",
        )
        .confirm(1.0)
        .inhibit(&[3, 4, 5, 6, 7, 9, 10, 11]),
    );
    v.push(
        proc(320_800_048, "STEER ALTN STEER SYS HOT", Level::Caution, sd_page::WHEEL, var("ALTN_STEER_SYS_TEMP_C").gt(ALTN_STEER_SYS_HOT_C), "the ALTN nosewheel-steering circuit's own modelled thermal law past its GENERIC hot threshold; FCOM PRO-ABN-ECAM p.5518")
            .confirm(1.0)
            .inhibit(&[3, 4, 5, 6, 7, 8, 9, 10])
            .items(1, Vec::new()),
    );
    v.push(
        proc(320_800_049, "STEER B/W STEER FAULT", Level::Advisory, sd_page::WHEEL, all(vec![var("BODY_STEER_FAULT:1").on(), network_alive()]), "the left body position's own actuator_leak reaching the SSC's BITE threshold; FCOM PRO-ABN-ECAM p.5519")
            .confirm(1.0)
            .inhibit(&[3, 4, 5, 6, 7, 9, 10])
            .items(1, Vec::new()),
    );
    v.push(
        proc(320_800_050, "STEER B/W STEER FAULT", Level::Advisory, sd_page::WHEEL, all(vec![var("BODY_STEER_FAULT:2").on(), network_alive()]), "as 320800049, right body position")
            .confirm(1.0)
            .inhibit(&[3, 4, 5, 6, 7, 9, 10])
            .items(2, Vec::new()),
    );
    v.push(
        proc(320_800_051, "STEER CAPT STEER TILLER FAULT", Level::Advisory, sd_page::WHEEL, all(vec![var("STEER_TILLER_FAULT:capt").on(), network_alive()]), "the captain's tiller transducer's own BITE; FCOM PRO-ABN-ECAM p.5523")
            .confirm(1.0)
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]),
    );
    v.push(
        proc(320_800_052, "STEER FO STEER TILLER FAULT", Level::Advisory, sd_page::WHEEL, all(vec![var("STEER_TILLER_FAULT:fo").on(), network_alive()]), "as 320800051, F/O side")
            .confirm(1.0)
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]),
    );
    v.push(
        proc(
            320_800_053,
            "STEER CTL 1 FAULT",
            Level::Advisory,
            sd_page::WHEEL,
            all(vec![var("STEER_CTL_FAULT:1").on(), network_alive()]),
            "the SSC's own control-channel-1 BITE; no FCOM title match, level/inhibit taken from the BRAKES CTL 1(2) FAULT family (320800015/016, same single-control-channel class)",
        )
        .confirm(1.0)
        .inhibit(&[3, 4, 5, 6, 7, 9, 10, 11]),
    );
    v.push(
        proc(320_800_054, "STEER CTL 2 FAULT", Level::Advisory, sd_page::WHEEL, all(vec![var("STEER_CTL_FAULT:2").on(), network_alive()]), "as 320800053, control channel 2")
            .confirm(1.0)
            .inhibit(&[3, 4, 5, 6, 7, 9, 10, 11]),
    );
    v.push(
        proc(
            320_800_055,
            "STEER N/W + B/W STEER FAULT",
            Level::Caution,
            sd_page::WHEEL,
            all(vec![
                any(vec![not(norm_steer_available()), not(altn_steer_available()), var("STEER_CTL_FAULT:1").on(), var("STEER_CTL_FAULT:2").on()]),
                any(vec![var("BODY_STEER_FAULT:1").on(), var("BODY_STEER_FAULT:2").on()]),
                network_alive(),
            ]),
            "a nose-steering condition and a body-steering condition together; FCOM PRO-ABN-ECAM p.5526",
        )
        .confirm(1.0)
        .inhibit(&[4, 5, 6, 7]),
    );
    v.push(
        proc(
            320_800_056,
            "STEER N/W STEER ANGLE LIMIT EXCEEDED",
            Level::Caution,
            sd_page::WHEEL,
            any(vec![var("NW_STEER_ANGLE_DEG").gt(MAX_NOSE_ANGLE_DEG), var("NW_STEER_ANGLE_DEG").lt(-MAX_NOSE_ANGLE_DEG)]),
            "the true nosewheel angle past steering.rs's own MAX_NOSE_ANGLE_DEG, only possible once steer_overtravel_fail defeats the healthy clamp; FCOM PRO-ABN-ECAM p.5528",
        )
        .confirm(1.0)
        .inhibit(&[3, 4, 5, 6, 7, 8, 9, 10, 11])
        .items(1, Vec::new()),
    );
    v.push(
        proc(
            320_800_057,
            "STEER N/W STEER DISC FAULT",
            Level::Caution,
            sd_page::WHEEL,
            all(vec![var("NW_STEER_DISC_SELECTED").on(), var("NW_STEER_DISCONNECTED").off(), network_alive()]),
            "the towing lever selected disconnect but the mechanism has not responded, past a GENERIC settle time for a mechanical disconnect solenoid; FCOM PRO-ABN-ECAM p.5529",
        )
        .confirm(5.0)
        .inhibit(&[3, 4, 5, 6, 7, 9, 10])
        .items(2, Vec::new()),
    );
    v.push(
        proc(
            320_800_058,
            "STEER N/W STEER FAULT",
            Level::Caution,
            sd_page::WHEEL,
            all(vec![any(vec![not(norm_steer_available()), not(altn_steer_available()), var("STEER_CTL_FAULT:1").on(), var("STEER_CTL_FAULT:2").on()]), network_alive()]),
            "the general nose-steering-degraded catch-all, matching this file's own single/general convention; FCOM PRO-ABN-ECAM p.5531",
        )
        .confirm(1.0)
        .inhibit(&[4, 5, 6, 7]),
    );
    v.push(
        proc(
            320_800_059,
            "STEER N/W STEER NOT DISC",
            Level::Caution,
            sd_page::WHEEL,
            all(vec![var("NW_STEER_DISC_SELECTED").off(), var("NW_STEER_DISCONNECTED").on(), network_alive()]),
            "the complementary stuck-disconnected case: not selected, but a stale disconnect state persists; FCOM PRO-ABN-ECAM p.5533",
        )
        .confirm(5.0)
        .inhibit(&[3, 4, 5, 6, 7, 8, 9, 10, 11, 12])
        .items(2, Vec::new()),
    );
    v.push(
        proc(
            320_800_060,
            "STEER NORM N/W STEER FAULT",
            Level::Caution,
            sd_page::WHEEL,
            all(vec![any(vec![not(norm_steer_available()), var("STEER_CTL_FAULT:1").on()]), network_alive()]),
            "the yellow (NORM) nosewheel-steering source below its availability threshold, or the SSC's own channel-1 BITE; FCOM PRO-ABN-ECAM p.5534",
        )
        .confirm(1.0)
        .inhibit(&[3, 4, 5, 6, 7, 10])
        .items(1, Vec::new()),
    );
    v.push(
        proc(320_800_061, "STEER PEDAL STEER CTL FAULT", Level::Caution, sd_page::WHEEL, all(vec![var("STEER_PEDAL_FAULT").on(), network_alive()]), "the pedal-steering transducer's own BITE; FCOM PRO-ABN-ECAM p.5536")
            .confirm(1.0)
            .inhibit(&[4, 5, 6, 7, 9, 10]),
    );
    v.push(
        proc(
            320_800_062,
            "STEER SEL VLV JAMMED OPEN",
            Level::Advisory,
            sd_page::WHEEL,
            all(vec![var("STEER_SEL_VLV_JAMMED").on(), network_alive()]),
            "the SSC's own selector-valve BITE; no FCOM title match, level/inhibit taken from the BRAKES SEL VLV JAMMED OPEN family (320800027, same selector-valve class)",
        )
        .confirm(1.0)
        .inhibit(&[3, 4, 5, 6, 7, 8, 9, 10, 11])
        .items(3, Vec::new()),
    );
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::deep::ecam::fbw;
    use crate::deep::integration::failure_audit::bare;
    use crate::deep::live::{Faults, Truth};

    fn run(truth: Truth, faults: &Faults, frames: usize) -> std::collections::BTreeMap<String, f64> {
        let mut deep = crate::deep::integration::failure_audit::fresh_areas();
        let mut out = std::collections::BTreeMap::new();
        for _ in 0..frames {
            out.clear();
            deep.tick(truth.clone(), faults, &mut |n, v| {
                out.insert(bare(n).to_owned(), v);
            });
        }
        out
    }

    fn holds(c: &Cond, published: &std::collections::BTreeMap<String, f64>) -> bool {
        c.eval(&|n: &str| *published.get(bare(n)).unwrap_or(&0.0))
    }

    fn wiring(id: u64) -> FbwProc {
        fbw::wirings().into_iter().find(|p| p.id == id).unwrap_or_else(|| panic!("{id} is not wired"))
    }

    fn failure_id(pred: impl Fn(&crate::deep::api::FailureDef) -> bool, what: &str) -> u64 {
        let r = crate::deep::registry();
        let hits: Vec<u64> = r.failures.iter().filter(|f| pred(f)).map(|f| f.id).collect();
        assert!(!hits.is_empty(), "no registered failure matches {what}");
        hits[0]
    }

    fn gear_fail(model_field_suffix: &str) -> u64 {
        failure_id(|f| f.area == crate::deep::api::Area::GearStructure && f.model_field.ends_with(model_field_suffix), model_field_suffix)
    }
    fn gear_fail_on(component: &str, model_field_suffix: &str) -> u64 {
        failure_id(|f| f.component == component && f.model_field.ends_with(model_field_suffix), model_field_suffix)
    }

    fn gear_direct(t: &Truth, faults: &Faults, ticks: usize) -> std::collections::BTreeMap<String, f64> {
        use crate::deep::gear_structure::live::GearStructureLive;
        use crate::deep::live::Area as _;
        let mut live = GearStructureLive::new();
        for _ in 0..ticks {
            live.tick(t, faults);
        }
        let mut out = std::collections::BTreeMap::new();
        live.publish(&mut |n, v| {
            out.insert(bare(n).to_owned(), v);
        });
        for bus in ["ELEC_AC_1_BUS_IS_POWERED", "ELEC_AC_2_BUS_IS_POWERED", "ELEC_AC_3_BUS_IS_POWERED", "ELEC_AC_4_BUS_IS_POWERED"] {
            out.insert(bus.to_string(), 1.0);
        }
        out
    }

    fn flying() -> Truth {
        Truth {
            dt_s: 0.05,
            on_ground: true,
            engine_running: [true; 4],
            engine_n1_frac: [0.9; 4],
            engine_n2_frac: [0.9; 4],
            engine_n3_frac: [0.9; 4],
            ac_bus_volts: [115.0; 4],
            dc_bus_volts: [28.0; 2],
            hydraulic_pressure_pa: [5000.0 * 6894.757; 2],
            ..Truth::default()
        }
    }

    fn cold() -> Truth {
        Truth { dt_s: 0.1, ..Truth::default() }
    }

    #[test]
    fn every_new_id_is_silent_cold_and_dark() {
        let out = run(cold(), &Faults::default(), 30);
        for id in [
            311_800_001, 311_800_002, 311_800_003, 311_800_004, 311_800_005, 311_800_006, 311_800_007, 311_800_008, 311_800_009, 311_800_010, 311_800_011, 313_800_003, 313_800_004, 313_800_007,
            314_800_001, 314_800_005, 316_800_001, 318_800_001, 319_800_001, 319_800_004, 320_800_009, 320_800_010, 320_800_011, 320_800_012, 320_800_013, 320_800_015, 320_800_016, 320_800_017,
            320_800_019, 320_800_020, 320_800_021, 320_800_024, 320_800_025, 320_800_026, 320_800_027, 320_800_028, 320_800_031, 320_800_032, 320_800_033, 320_800_034, 320_800_035, 320_800_041,
            320_800_042, 320_800_043, 320_800_044, 320_800_046, 320_800_047, 320_800_048, 320_800_049, 320_800_050, 320_800_051, 320_800_052, 320_800_053, 320_800_054, 320_800_055, 320_800_056,
            320_800_057, 320_800_058, 320_800_059, 320_800_060, 320_800_061, 320_800_062, 320_800_001, 320_800_002, 320_800_003, 320_800_004,
            320_800_005, 320_800_006, 320_800_007,
        ] {
            assert!(!holds(&wiring(id).trigger, &out), "{id} ({}) fired on a cold and dark aircraft", wiring(id).title);
        }
    }

    #[test]
    fn brakes_accu_press_lo_fires_purely_on_pressure_with_no_holding_gate() {
        let leak = gear_fail("ParkingBrakeFaults.leak");
        let mut t = flying();
        t.dt_s = 1.0;
        t.controls.parking_brake_on = true;
        let healthy = gear_direct(&t, &Faults::default(), 20);
        let leaking = gear_direct(&t, &Faults::from_pairs([(leak, 1.0)]), 40_000);
        assert!(!holds(&wiring(320_800_009).trigger, &healthy));
        assert!(holds(&wiring(320_800_009).trigger, &leaking), "BRAKES ACCU PRESS LO must fire once the accumulator pressure itself falls");
    }

    #[test]
    fn each_antiskid_gear_combination_raises_exactly_its_own_title() {
        let wheels = |gears: &[usize]| {
            let mut m: std::collections::BTreeMap<String, f64> = std::collections::BTreeMap::new();
            m.insert("ELEC_AC_1_BUS_IS_POWERED".into(), 1.0);
            for &g in gears {
                for n in ANTISKID_GEAR_WHEELS[g] {
                    m.insert(format!("ANTISKID_CHANNEL_FAULT:{n}"), 1.0);
                }
            }
            m
        };
        let ids = [320_800_001u64, 320_800_002, 320_800_003, 320_800_004, 320_800_005, 320_800_006, 320_800_007];
        for (gears, expected) in [
            (vec![], None),
            (vec![2], Some(320_800_003)),
            (vec![3], Some(320_800_004)),
            (vec![2, 3], Some(320_800_002)),
            (vec![0, 1], Some(320_800_007)),
            (vec![0, 1, 2], Some(320_800_005)),
            (vec![0, 1, 3], Some(320_800_006)),
            (vec![0, 1, 2, 3], Some(320_800_001)),
        ] {
            let published = wheels(&gears);
            let fired: Vec<u64> = ids.iter().copied().filter(|&id| holds(&wiring(id).trigger, &published)).collect();
            assert_eq!(fired, expected.into_iter().collect::<Vec<_>>(), "gears {gears:?}");
        }
        let mut one_wheel = wheels(&[]);
        one_wheel.insert("ANTISKID_CHANNEL_FAULT:9".into(), 1.0);
        assert!(ids.iter().all(|&id| !holds(&wiring(id).trigger, &one_wheel)), "one wheel's channel is not a gear's antiskid");
        let mut bcs = wheels(&[]);
        bcs.insert("BSCU_CHANNEL_FAULT:1".into(), 1.0);
        bcs.insert("BSCU_CHANNEL_FAULT:2".into(), 1.0);
        assert!(holds(&wiring(320_800_001).trigger, &bcs));
    }

    #[test]
    fn lgciu_faults_fire_singly_and_suppress_into_the_combined_procedure() {
        let l1 = failure_id(|f| f.component == "32_elec.lgciu-1" && f.model_field.contains("open_circuit"), "lgciu-1 open circuit");
        let l2 = failure_id(|f| f.component == "32_elec.lgciu-2" && f.model_field.contains("open_circuit"), "lgciu-2 open circuit");
        let one = run(flying(), &Faults::from_pairs([(l1, 1.0)]), 20);
        let both = run(flying(), &Faults::from_pairs([(l1, 1.0), (l2, 1.0)]), 20);
        assert!(holds(&wiring(320_800_033).trigger, &one));
        assert!(!holds(&wiring(320_800_035).trigger, &one));
        assert!(holds(&wiring(320_800_035).trigger, &both));
    }

    #[test]
    fn gear_uplock_fault_fires_on_a_jammed_uplock_with_the_lever_down() {
        use crate::deep::gear_structure::live::GearStructureLive;
        use crate::deep::live::Area as _;
        let jam = gear_fail("RetractionFaults.uplock_jam");
        let mut live = GearStructureLive::new();
        let mut t = flying();
        t.dt_s = 0.1;
        t.on_ground = false;
        t.controls.gear_lever_down = false;
        for _ in 0..300 {
            live.tick(&t, &Faults::default());
        }
        let mut published = std::collections::BTreeMap::new();
        live.publish(&mut |n, v| {
            published.insert(bare(n).to_owned(), v);
        });
        assert_eq!(published.get("GEAR_UPLOCKED:1"), Some(&1.0), "setup: nose leg must be up-locked");

        t.controls.gear_lever_down = true;
        let jam_faults = Faults::from_pairs([(jam, 1.0)]);
        for _ in 0..300 {
            live.tick(&t, &jam_faults);
        }
        let mut jammed = std::collections::BTreeMap::new();
        live.publish(&mut |n, v| {
            jammed.insert(bare(n).to_owned(), v);
        });
        assert!(holds(&wiring(320_800_041).trigger, &jammed));
    }

    #[test]
    fn bscu_channel_and_pressure_sensor_faults_fire_their_own_alerts() {
        let ctl1 = gear_fail_on("32_gear.bscu", "ctl_1_fail");
        let ctl2 = gear_fail_on("32_gear.bscu", "ctl_2_fail");
        let norm_sensor = gear_fail_on("32_gear.bscu", "norm_press_sensor_fail");
        let alt_sensor = gear_fail_on("32_gear.bscu", "alt_press_sensor_fail");
        let autobrake = gear_fail_on("32_gear.bscu", "autobrake_fail");
        let sel_valve = gear_fail_on("32_gear.bscu", "sel_valve_jam");

        let healthy = run(flying(), &Faults::default(), 5);
        for id in [320_800_015, 320_800_016, 320_800_021, 320_800_012, 320_800_013, 320_800_027] {
            assert!(!holds(&wiring(id).trigger, &healthy), "{id} must be quiet when healthy");
        }
        assert!(holds(&wiring(320_800_015).trigger, &run(flying(), &Faults::from_pairs([(ctl1, 1.0)]), 5)));
        assert!(holds(&wiring(320_800_016).trigger, &run(flying(), &Faults::from_pairs([(ctl2, 1.0)]), 5)));
        assert!(holds(&wiring(320_800_021).trigger, &run(flying(), &Faults::from_pairs([(norm_sensor, 1.0)]), 5)));
        assert!(holds(&wiring(320_800_012).trigger, &run(flying(), &Faults::from_pairs([(alt_sensor, 1.0)]), 5)));
        assert!(holds(&wiring(320_800_013).trigger, &run(flying(), &Faults::from_pairs([(autobrake, 1.0)]), 5)));
        assert!(holds(&wiring(320_800_027).trigger, &run(flying(), &Faults::from_pairs([(sel_valve, 1.0)]), 5)));
    }

    #[test]
    fn brake_source_faults_fire_norm_altn_emer_and_the_redundancy_lost_combination() {
        let ctl1 = gear_fail_on("32_gear.bscu", "ctl_1_fail");
        let ctl2 = gear_fail_on("32_gear.bscu", "ctl_2_fail");
        let leak = gear_fail("ParkingBrakeFaults.leak");

        let ctl1_only = run(flying(), &Faults::from_pairs([(ctl1, 1.0)]), 5);
        assert!(holds(&wiring(320_800_020).trigger, &ctl1_only), "BRAKES NORM BRK FAULT must fire on channel-1 BITE alone");
        assert!(!holds(&wiring(320_800_028).trigger, &ctl1_only), "one degraded system alone must not lose redundancy");

        let ctl2_only = run(flying(), &Faults::from_pairs([(ctl2, 1.0)]), 5);
        assert!(holds(&wiring(320_800_011).trigger, &ctl2_only), "BRAKES ALTN BRK FAULT must fire on channel-2 BITE alone");

        let both = run(flying(), &Faults::from_pairs([(ctl1, 1.0), (ctl2, 1.0)]), 5);
        assert!(holds(&wiring(320_800_028).trigger, &both), "NORM and ALTN both degraded, EMER still healthy -> BRAKES SYS REDUNDANCY LOST");

        let mut leak_truth = flying();
        leak_truth.dt_s = 1.0;
        leak_truth.controls.parking_brake_on = true;
        let mut leaking = gear_direct(&leak_truth, &Faults::from_pairs([(leak, 1.0)]), 40_000);
        leaking.insert("HYD_YELLOW_MANIFOLD_PRESSURE_PSI".to_string(), 5000.0);
        leaking.insert("HYD_GREEN_MANIFOLD_PRESSURE_PSI".to_string(), 5000.0);
        assert!(holds(&wiring(320_800_017).trigger, &leaking));
        assert!(!holds(&wiring(320_800_010).trigger, &leaking), "ALTN is healthy, so the combined ALTN+EMER procedure must stay quiet");
    }

    #[test]
    fn brake_pedal_transducer_faults_fire_pedal_braking_fault() {
        let left = gear_fail_on("32_gear.brake_pedal_transducers", "left");
        let out = run(flying(), &Faults::from_pairs([(left, 1.0)]), 5);
        assert!(holds(&wiring(320_800_024).trigger, &out));
    }

    #[test]
    fn brakes_released_and_residual_braking_read_the_real_applied_fraction() {
        let t = flying();
        let mut starved = t.clone();
        starved.controls.brake_pedal_pos = [1.0, 1.0];
        starved.hydraulic_pressure_pa = [0.0, 0.0];
        let released = run(starved, &Faults::default(), 5);
        assert!(holds(&wiring(320_800_025).trigger, &released), "commanded braking with both hydraulic sources dead must show BRAKES RELEASED");

        let dragging = gear_fail_on("32_gear.wheel_1_brake", "dragging");
        let mut released_pedals = flying();
        released_pedals.controls.brake_pedal_pos = [0.0, 0.0];
        released_pedals.dt_s = 0.1;
        let dragging_out = run(released_pedals, &Faults::from_pairs([(dragging, 1.0)]), 600);
        assert!(holds(&wiring(320_800_026).trigger, &dragging_out), "a dragging wheel with the pedals released must show BRAKES RESIDUAL BRAKING");
    }

    #[test]
    fn oleo_pressure_sensing_and_charge_band_alerts_fire_independently() {
        let sensor = gear_fail_on("32_gear.l_wing_strut", "gas_charge_sensor_fail");
        let out = run(flying(), &Faults::from_pairs([(sensor, 1.0)]), 5);
        assert!(holds(&wiring(320_800_043).trigger, &out));
        assert!(!holds(&wiring(320_800_031).trigger, &out), "a monitoring-BITE fault must not itself move the real charge fraction");

        let gas_leak = gear_fail_on("32_gear.l_wing_strut", "gas_leak");
        let mut slow_truth = flying();
        slow_truth.dt_s = 1.0;
        let leaked = gear_direct(&slow_truth, &Faults::from_pairs([(gas_leak, 1.0)]), 20_000);
        assert!(holds(&wiring(320_800_031).trigger, &leaked));
    }

    #[test]
    fn weight_on_wheels_fault_fires_on_a_lying_sensor_on_the_ground() {
        let wow = gear_fail_on("32_gear.l_wing_strut", "wow_sensing_fail");
        let out = run(flying(), &Faults::from_pairs([(wow, 1.0)]), 5);
        assert!(holds(&wiring(320_800_046).trigger, &out));
    }

    #[test]
    fn bogie_position_fault_fires_when_a_body_leg_fails_to_trim_while_retracting() {
        let bogie = gear_fail_on("32_gear.l_body_retraction", "bogie_trim_fail");
        let mut t = flying();
        t.controls.gear_lever_down = false;
        t.dt_s = 0.1;
        let out = gear_direct(&t, &Faults::from_pairs([(bogie, 1.0)]), 2_000);
        assert!(holds(&wiring(320_800_032).trigger, &out));
    }

    #[test]
    fn gravity_extension_fault_fires_when_a_severe_uplock_jam_defeats_it() {
        use crate::deep::gear_structure::live::GearStructureLive;
        use crate::deep::live::Area as _;
        let jam = gear_fail("RetractionFaults.uplock_jam");
        let mut live = GearStructureLive::new();
        let mut t = flying();
        t.dt_s = 0.1;
        t.on_ground = false;
        t.controls.gear_lever_down = false;
        for _ in 0..300 {
            live.tick(&t, &Faults::default());
        }
        let mut up = std::collections::BTreeMap::new();
        live.publish(&mut |n, v| {
            up.insert(bare(n).to_owned(), v);
        });
        assert_eq!(up.get("GEAR_UPLOCKED:1"), Some(&1.0));

        t.controls.gear_lever_down = true;
        t.controls.gravity_extend_selected = true;
        let jam_faults = Faults::from_pairs([(jam, 0.99)]);
        for _ in 0..5_000 {
            live.tick(&t, &jam_faults);
        }
        let mut out = std::collections::BTreeMap::new();
        live.publish(&mut |n, v| {
            out.insert(bare(n).to_owned(), v);
        });
        assert!(holds(&wiring(320_800_042).trigger, &out));
    }

    #[test]
    fn steering_control_and_input_transducer_faults_fire_their_own_alerts() {
        let capt_tiller = gear_fail_on("32_gear.steer_input_transducers", "capt_tiller_fail");
        let pedal_steer = gear_fail_on("32_gear.steer_input_transducers", "pedal_steer_fail");
        let sel_valve = gear_fail_on("32_gear.steer_ctl", "sel_valve_jam");

        assert!(holds(&wiring(320_800_051).trigger, &run(flying(), &Faults::from_pairs([(capt_tiller, 1.0)]), 5)));
        assert!(holds(&wiring(320_800_061).trigger, &run(flying(), &Faults::from_pairs([(pedal_steer, 1.0)]), 5)));
        assert!(holds(&wiring(320_800_062).trigger, &run(flying(), &Faults::from_pairs([(sel_valve, 1.0)]), 5)));
    }

    #[test]
    fn body_steer_fault_and_combined_nw_plus_bw_fault() {
        let body_leak = gear_fail_on("32_gear.l_body_steering", "actuator_leak");
        let ctl1 = gear_fail_on("32_gear.steer_ctl", "ctl_1_fail");
        let out = run(flying(), &Faults::from_pairs([(body_leak, 1.0)]), 5);
        assert!(holds(&wiring(320_800_049).trigger, &out));
        assert!(!holds(&wiring(320_800_055).trigger, &out), "a body-only condition alone must not raise the combined procedure");

        let both = run(flying(), &Faults::from_pairs([(body_leak, 1.0), (ctl1, 1.0)]), 5);
        assert!(holds(&wiring(320_800_055).trigger, &both));
    }

    #[test]
    fn nw_steer_disconnect_and_overtravel_and_thermal_alerts_fire() {
        let disc = gear_fail_on("32_gear.nose_steering", "disc_mechanism_fail");
        let mut t = flying();
        t.controls.nw_steer_disc_selected = true;
        let out = run(t, &Faults::from_pairs([(disc, 1.0)]), 100);
        assert!(holds(&wiring(320_800_057).trigger, &out), "a jammed disconnect mechanism must stop it responding to the selection, past the settle time");

        let overtravel = gear_fail_on("32_gear.nose_steering", "steer_overtravel_fail");
        let mut big = flying();
        big.controls.steering_command_deg[0] = MAX_NOSE_ANGLE_DEG + 40.0;
        let angle_out = run(big, &Faults::from_pairs([(overtravel, 1.0)]), 100);
        assert!(holds(&wiring(320_800_056).trigger, &angle_out));

        let mut hot = flying();
        hot.controls.steering_command_deg[0] = 45.0;
        hot.dt_s = 1.0;
        let hot_out = gear_direct(&hot, &Faults::default(), 3_000);
        assert!(holds(&wiring(320_800_048).trigger, &hot_out));
    }
}
