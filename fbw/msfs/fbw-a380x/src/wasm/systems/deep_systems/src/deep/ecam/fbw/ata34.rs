use super::{proc, sd_page, FbwProc};
use crate::deep::api::{all, any, var};
use crate::deep::api::Level;
use crate::deep::api::Cond;

fn network_alive() -> Cond {
    any(vec![
        var("ELEC_AC_1_BUS_IS_POWERED").on(),
        var("ELEC_AC_2_BUS_IS_POWERED").on(),
        var("ELEC_AC_3_BUS_IS_POWERED").on(),
        var("ELEC_AC_4_BUS_IS_POWERED").on(),
    ])
}

fn aoa_fault(n: u32) -> crate::deep::api::Cond {
    any(vec![
        var(&format!("DEEP_AOA_{n}_JAMMED")).on(),
        var(&format!("DEEP_AOA_{n}_HEATER_FAILED")).on(),
        var(&format!("DEEP_AOA_{n}_DEG")).gt(5.0),
        var(&format!("DEEP_AOA_{n}_DEG")).lt(-5.0),
    ])
}

fn gps_invalid(n: u32) -> crate::deep::api::Cond {
    var(&format!("DEEP_GPS_{n}_VALID")).eq(0.0)
}

fn gps_position_off(n: u32) -> crate::deep::api::Cond {
    all(vec![
        var(&format!("DEEP_GPS_{n}_VALID")).on(),
        any(vec![
            var(&format!("DEEP_GPS_{n}_OFFSET_N_M")).gt(926.0),
            var(&format!("DEEP_GPS_{n}_OFFSET_N_M")).lt(-926.0),
            var(&format!("DEEP_GPS_{n}_OFFSET_E_M")).gt(926.0),
            var(&format!("DEEP_GPS_{n}_OFFSET_E_M")).lt(-926.0),
        ]),
    ])
}

pub fn wire(v: &mut Vec<FbwProc>) {
    v.push(
        proc(340_800_025, "NAV FM/GPS POS DISAGREE", Level::Caution, sd_page::STATUS, any((1..=3).map(gps_position_off).collect()), "a valid GPS fix more than 0.5' from the true position (deep::sensors::gps spoofing walk-off, DEEP_GPS_n_OFFSET_N/E_M)")
            .confirm(10.0)
            .inhibit(&[1, 3, 4, 5, 6, 12])
            .items(0, Vec::new()),
    );

    v.push(proc(340_800_011, "NAV AOA 1 FAULT", Level::Caution, sd_page::STATUS, aoa_fault(1), "AoA vane 1 jammed or its heater failed (deep::sensors DEEP_AOA_1_JAMMED / DEEP_AOA_1_HEATER_FAILED)").confirm(2.0).items(0, Vec::new()));
    v.push(proc(340_800_012, "NAV AOA 2 FAULT", Level::Caution, sd_page::STATUS, aoa_fault(2), "AoA vane 2 jammed or its heater failed").confirm(2.0).items(0, Vec::new()));
    v.push(proc(340_800_013, "NAV AOA 3 FAULT", Level::Caution, sd_page::STATUS, aoa_fault(3), "AoA vane 3 jammed or its heater failed").confirm(2.0).items(0, Vec::new()));
    v.push(
        proc(
            340_800_014,
            "NAV AOA DISAGREE",
            Level::Caution,
            sd_page::STATUS,
            any(vec![var("DEEP_AOA_1_JAMMED").on(), var("DEEP_AOA_2_JAMMED").on(), var("DEEP_AOA_3_JAMMED").on()]),
            "deep::sensors DEEP_AOA_n_JAMMED, moved from the registry's own DEEP_NAV_AOA_DISAGREE",
        )
        .confirm(2.0)
        .inhibit(&[4, 5, 6, 7, 9, 10])
        .items(0, Vec::new()),
    );


    v.push(
        proc(340_800_035, "NAV GPS 2 FAULT", Level::Caution, sd_page::STATUS, gps_invalid(2), "GPS/MMR receiver 2's own validity flag gone false (deep::sensors DEEP_GPS_2_VALID)")
            .confirm(5.0)
            .suppressed_by(&[340_800_036])
            .inhibit(&[4, 5, 6, 7, 10])
            .items(0, Vec::new()),
    );
    v.push(
        proc(340_800_036, "NAV GPS 1+2 FAULT", Level::Caution, sd_page::STATUS, all(vec![gps_invalid(1), gps_invalid(2)]), "both GPS/MMR receivers' validity flags false together")
            .confirm(5.0)
            .inhibit(&[4, 5, 6, 7, 10])
            .items(0, Vec::new()),
    );

    let adr_outlier = |n: u32| var(&format!("DEEP_ADR_{n}_OUTLIER")).on();
    v.push(
        proc(
            340_800_009,
            "NAV AIR DATA DISAGREE",
            Level::Caution,
            sd_page::STATUS,
            any(vec![adr_outlier(1), adr_outlier(2), adr_outlier(3), var("DEEP_ADR_VOTE_DISAGREE").on()]),
            "FCOM p.5556: at least one ADR the voter already flags as an outlier, or the registry's own DEEP_ADR_VOTE_DISAGREE (moved from the invented NAV ADR DISAGREE registry alert)",
        )
        .confirm(2.0)
        .inhibit(&[4, 5, 6, 7, 9, 10])
        .items(0, Vec::new()),
    );
    v.push(
        proc(
            340_800_010,
            "NAV ALL AIR DATA DISAGREE",
            Level::Caution,
            sd_page::STATUS,
            all(vec![adr_outlier(1), adr_outlier(2), adr_outlier(3)]),
            "FCOM p.5556: the same NAV AIR DATA DISAGREE procedure's three-source subtitle -- the voter cannot reconcile any of the three ADRs",
        )
        .confirm(2.0)
        .inhibit(&[4, 5, 6, 7, 9, 10])
        .items(0, Vec::new()),
    );
    v.push(
        proc(
            340_800_007,
            "NAV ADR 1+2+3 DATA DEGRADED",
            Level::Caution,
            sd_page::STATUS,
            all(vec![adr_outlier(1), adr_outlier(2), adr_outlier(3)]),
            "FCOM p.5549: FlyByWire's own softer wording for the same three-way voter disagreement 340800010 already reads",
        )
        .confirm(2.0)
        .suppressed_by(&[340_800_010])
        .inhibit(&[4, 5, 6, 7, 10])
        .items(2, Vec::new()),
    );

    v.push(
        proc(
            340_800_071,
            "NAV UNRELIABLE AIR SPEED INDICATION",
            Level::Warning,
            sd_page::STATUS,
            all(vec![adr_outlier(1), adr_outlier(2), adr_outlier(3)]),
            "the ADR voter cannot reconcile airspeed across any of the three ADRs (same composition as 340800010)",
        )
        .confirm(2.0)
        .items(0, Vec::new()),
    );

    for (id, title) in [
        (340_800_022, "NAV FLS 1 CAPABILITY LOST"),
        (340_800_027, "NAV GLS 1 CAPABILITY LOST"),
        (340_800_030, "NAV GLS 1 FAULT"),
        (340_800_037, "NAV ILS 1 FAULT"),
        (340_800_046, "NAV LS 1 FAULT"),
    ] {
        v.push(proc(id, title, Level::Caution, sd_page::STATUS, gps_invalid(1), "FCOM: the FLS/GLS/ILS/LS function within MMR 1 is failed (one receiver, several reception modes, DEEP_GPS_1_VALID)").confirm(5.0).inhibit(&[3, 4, 5, 6, 7]).items(0, Vec::new()));
    }
    for (id, title) in [
        (340_800_023, "NAV FLS 2 CAPABILITY LOST"),
        (340_800_028, "NAV GLS 2 CAPABILITY LOST"),
        (340_800_031, "NAV GLS 2 FAULT"),
        (340_800_038, "NAV ILS 2 FAULT"),
        (340_800_047, "NAV LS 2 FAULT"),
    ] {
        v.push(proc(id, title, Level::Caution, sd_page::STATUS, gps_invalid(2), "FCOM: the FLS/GLS/ILS/LS function within MMR 2 is failed (DEEP_GPS_2_VALID)").confirm(5.0).inhibit(&[3, 4, 5, 6, 7]).items(0, Vec::new()));
    }
    for (id, title) in [
        (340_800_024, "NAV FLS 1+2 CAPABILITY LOST"),
        (340_800_029, "NAV GLS 1+2 CAPABILITY LOST"),
        (340_800_032, "NAV GLS 1+2 FAULT"),
        (340_800_039, "NAV ILS 1+2 FAULT"),
        (340_800_048, "NAV LS 1+2 FAULT"),
    ] {
        v.push(proc(id, title, Level::Caution, sd_page::STATUS, all(vec![gps_invalid(1), gps_invalid(2)]), "FCOM: the FLS/GLS/ILS/LS function within both MMRs is failed").confirm(5.0).inhibit(&[3, 4, 5, 6, 7]).items(0, Vec::new()));
    }

    v.push(
        proc(341_800_026, "SURV GPWS 1 FAULT", Level::Caution, sd_page::STATUS, var("ELEC_AC_ESS_BUS_IS_POWERED").off(), "AESS lane 1 (GPWS) loses its AC ESS bus power source, the same gate FlyByWire's own EfisTawsBridge.ts uses")
            .confirm(2.0)
            .suppressed_by(&[341_800_028])
            .inhibit(&[3, 4, 5, 6, 7, 10, 11])
            .items(3, Vec::new()),
    );
    v.push(
        proc(341_800_027, "SURV GPWS 2 FAULT", Level::Caution, sd_page::STATUS, var("ELEC_AC_4_BUS_IS_POWERED").off(), "AESS lane 2 (GPWS) loses its AC 4 bus power source")
            .confirm(2.0)
            .suppressed_by(&[341_800_028])
            .inhibit(&[3, 4, 5, 6, 7, 10, 11])
            .items(3, Vec::new()),
    );
    v.push(
        proc(
            341_800_028,
            "SURV GPWS 1+2 FAULT",
            Level::Caution,
            sd_page::STATUS,
            all(vec![var("ELEC_AC_ESS_BUS_IS_POWERED").off(), var("ELEC_AC_4_BUS_IS_POWERED").off()]),
            "both AESS lanes down together -- no GPWS capability at all",
        )
        .confirm(2.0)
        .inhibit(&[3, 4, 5, 6, 7, 10, 11])
        .items(1, Vec::new()),
    );

    v.push(
        proc(
            340_800_067,
            "NAV STATIC PROBE FAULT",
            Level::Caution,
            sd_page::STATUS,
            any(vec![
                var("DEEP_STATIC_1_DEGRADED").on(),
                var("DEEP_STATIC_2_DEGRADED").on(),
                var("DEEP_STATIC_3_DEGRADED").on(),
                var("DEEP_STATIC_4_DEGRADED").on(),
            ]),
            "at least one system's static-port pair reads degraded (deep::sensors DEEP_STATIC_n_DEGRADED, average_pair's own verdict when one port of a pair is blocked)",
        )
        .confirm(3.0)
        .inhibit(&[3, 4, 5, 6, 7, 8, 9, 10])
        .items(0, Vec::new()),
    );

    v.push(
        proc(340_800_016, "NAV CAPT AND F/O ALT DISAGREE", Level::Caution, sd_page::STATUS, var("DEEP_ADR_CAPT_FO_ALT_DIFF_FT").gt(250.0), "FCOM p.5569: CAPT/F.O displayed altitude differs by more than 250 ft (QNH) / 500 ft (STD) -- the tighter QNH figure is applied unconditionally")
            .confirm(2.0)
            .inhibit(&[4, 5, 6, 10])
            .items(0, Vec::new()),
    );

    v.push(
        proc(
            340_800_017,
            "NAV CAPT AND F/O ATT DISAGREE",
            Level::Caution,
            sd_page::STATUS,
            any(vec![var("DEEP_IR_CAPT_FO_PITCH_DIFF_DEG").gt(5.0), var("DEEP_IR_CAPT_FO_ROLL_DIFF_DEG").gt(5.0)]),
            "FCOM p.5570: more than 5 deg pitch or roll discrepancy between the CAPT and F.O side IRs",
        )
        .confirm(2.0)
        .inhibit(&[4, 5, 6, 10])
        .items(0, Vec::new()),
    );
    v.push(
        proc(340_800_020, "NAV CAPT AND F/O HDG DISAGREE", Level::Caution, sd_page::STATUS, var("DEEP_IR_CAPT_FO_HDG_DIFF_DEG").gt(5.0), "FCOM p.5573: more than 5 deg TRUE-reference heading discrepancy between the CAPT and F.O side IRs; the 7 deg MAGNETIC-reference allowance is not applied (every IR shares one magnetic variation, so this port's magnetic headings disagree exactly as the true ones do)")
            .confirm(2.0)
            .inhibit(&[4, 5, 6, 10])
            .items(0, Vec::new()),
    );

    v.push(
        proc(340_800_018, "NAV CAPT AND F/O BARO REF DISAGREE", Level::Caution, sd_page::STATUS, var("DEEP_BARO_REF_DISAGREE").on(), "FCOM p.5572: the Captain's barometric reference is QNH(STD) while the First Officer's is STD(QNH) -- FlyByWire's own A32NX_FCU_EFIS_{L,R}_DISPLAY_BARO_MODE, compared side to side")
            .confirm(10.0)
            .inhibit(&[3, 4, 5, 6, 10])
            .items(0, Vec::new()),
    );

    v.push(
        proc(340_800_033, "NAV GNSS SIGNAL DEGRADED", Level::Advisory, sd_page::STATUS, any(vec![var("DEEP_GPS_1_DEGRADED").on(), var("DEEP_GPS_2_DEGRADED").on()]), "a GPS/MMR receiver's own jamming magnitude sits in the tested mild band (fix still good, DEEP_GPS_n_VALID stays 1) rather than the heavy band that fails it outright")
            .confirm(5.0)
            .items(0, Vec::new()),
    );

    v.push(proc(340_800_050, "NAV OAT PROBE 1 FAULT", Level::Caution, sd_page::STATUS, var("DEEP_OAT_1_HEATER_FAILED").on(), "OAT probe 1's own heater has failed (deep::sensors, a new instance of the same probe class TAT/AoA already use)").confirm(2.0).items(0, Vec::new()));
    v.push(proc(340_800_051, "NAV OAT PROBE 2 FAULT", Level::Caution, sd_page::STATUS, var("DEEP_OAT_2_HEATER_FAILED").on(), "OAT probe 2's own heater has failed").confirm(2.0).items(0, Vec::new()));


    v.push(proc(340_800_056, "NAV RA SYS A LOST BY PRIM", Level::Caution, sd_page::STATUS, var("DEEP_PRIM_1_USING_RA_A").off(), "PRIM 1's own RA-A link has faulted (deep::flight_controls, a minimal PRIM RA-health flag, no control-law model)").confirm(2.0).items(0, Vec::new()));
    v.push(proc(340_800_057, "NAV RA SYS B LOST BY PRIM", Level::Caution, sd_page::STATUS, var("DEEP_PRIM_2_USING_RA_B").off(), "PRIM 2's own RA-B link has faulted").confirm(2.0).items(0, Vec::new()));
    v.push(proc(340_800_058, "NAV RA SYS C LOST BY PRIM", Level::Caution, sd_page::STATUS, var("DEEP_PRIM_3_USING_RA_C").off(), "PRIM 3's own RA-C link has faulted").confirm(2.0).items(0, Vec::new()));

    for (id, title, n) in [(340_800_064, "NAV SIDESLIP PROBE 1 FAULT", 1), (340_800_065, "NAV SIDESLIP PROBE 2 FAULT", 2), (340_800_066, "NAV SIDESLIP PROBE 3 FAULT", 3)] {
        v.push(
            proc(
                id,
                title,
                Level::Advisory,
                sd_page::STATUS,
                any(vec![var(&format!("DEEP_SIDESLIP_{n}_JAMMED")).on(), var(&format!("DEEP_SIDESLIP_{n}_HEATER_FAILED")).on()]),
                "FCOM p.5611: one sideslip probe is failed (jammed or its heater failed) -- the same aoa_fault pattern, a third vane instance",
            )
            .confirm(2.0)
            .inhibit(&[3, 4, 5, 6, 7, 8, 9, 10, 11])
            .items(0, Vec::new()),
        );
    }

    v.push(
        proc(340_800_070, "NAV TAT PROBE 3 FAULT", Level::Caution, sd_page::STATUS, var("DEEP_TAT_3_HEATER_FAILED").on(), "TAT probe 3's own heater has failed (deep::sensors, a third instance of the existing two-probe component)")
            .confirm(2.0)
            .items(0, Vec::new()),
    );
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::deep::api::Cond;
    use crate::deep::integration::failure_audit::fresh_areas;
    use crate::deep::live::{Faults, Truth};
    use crate::deep::sensors::live::FaultIndex;
    use std::collections::BTreeMap;

    fn flying() -> Truth {
        Truth { dt_s: 0.1, on_ground: false, altitude_ft: 20_000.0, ac_bus_volts: [115.0; 4], dc_bus_volts: [28.0; 2], engine_running: [true; 4], engine_n1_frac: [0.85; 4], ..Truth::default() }
    }

    fn run(truth: Truth, faults: &Faults, frames: usize) -> BTreeMap<String, f64> {
        let mut deep = fresh_areas();
        let mut out = BTreeMap::new();
        for _ in 0..frames {
            out.clear();
            deep.tick(truth.clone(), faults, &mut |n, v| {
                out.insert(n.to_string(), v);
            });
        }
        out
    }

    fn holds(c: &Cond, published: &BTreeMap<String, f64>) -> bool {
        c.eval(&|n: &str| *published.get(n).unwrap_or(&0.0))
    }

    fn wiring(id: u64) -> FbwProc {
        let mut v = Vec::new();
        wire(&mut v);
        v.into_iter().find(|p| p.id == id).unwrap_or_else(|| panic!("{id} is not wired by ata34::wire"))
    }

    #[test]
    fn every_trigger_reads_a_published_variable() {
        use crate::deep::integration::failure_audit::{bare, cond_vars};
        use crate::deep::live::all_areas;
        let published: std::collections::BTreeSet<String> = all_areas().published_names().iter().map(|n| bare(n).to_owned()).collect();
        let mut v = Vec::new();
        wire(&mut v);
        for p in &v {
            let mut names = Vec::new();
            cond_vars(&p.trigger, &mut names);
            for n in names {
                let n = bare(&n);
                assert!(published.contains(n), "{} ({}) reads {n}, which nothing publishes", p.id, p.title);
            }
        }
    }

    #[test]
    fn aoa_2_fault_fires_only_on_the_seized_vane() {
        let index = FaultIndex::build();
        let id = index.id("34_nav.aoa_2", "mechanically_stuck");
        assert_ne!(id, 0, "the AoA seizure fault id could not be resolved");

        let healthy = run(flying(), &Faults::default(), 10);
        let armed = run(flying(), &Faults::from_pairs([(id, 1.0)]), 30);

        let aoa1 = wiring(340_800_011);
        let aoa2 = wiring(340_800_012);
        assert!(!holds(&aoa2.trigger, &healthy), "NAV AOA 2 FAULT must be quiet with every vane free");
        assert!(holds(&aoa2.trigger, &armed), "NAV AOA 2 FAULT must fire once vane 2 seizes");
        assert!(!holds(&aoa1.trigger, &armed), "vane 1 was untouched, so its own procedure must stay quiet");
    }

    #[test]
    fn gps_2_and_the_1_plus_2_combo_need_the_right_receivers_down() {
        let index = FaultIndex::build();
        let id2 = index.id("34_nav.gps_receiver_2", "receiver_fault");
        assert_ne!(id2, 0, "the GPS 2 receiver fault id could not be resolved");

        let healthy = run(flying(), &Faults::default(), 10);
        let one_down = run(flying(), &Faults::from_pairs([(id2, 1.0)]), 10);

        let gps2 = wiring(340_800_035);
        let combo = wiring(340_800_036);
        assert!(!holds(&gps2.trigger, &healthy), "NAV GPS 2 FAULT must be quiet with both receivers healthy");
        assert!(holds(&gps2.trigger, &one_down), "NAV GPS 2 FAULT must fire once receiver 2 fails outright");
        assert!(!holds(&combo.trigger, &one_down), "receiver 1 is still healthy, so the 1+2 combo must not fire on receiver 2 alone");
    }

    #[test]
    fn fm_gps_disagree_fires_once_a_spoofed_receiver_has_walked_off() {
        let index = FaultIndex::build();
        let id = index.id("34_nav.gps_receiver_1", "spoof_target_offset_m");
        assert_ne!(id, 0, "the GPS 1 spoofing fault id could not be resolved");
        let disagree = wiring(340_800_025);

        let healthy = run(Truth { dt_s: 1.0, ..flying() }, &Faults::default(), 1200);
        let early = run(Truth { dt_s: 1.0, ..flying() }, &Faults::from_pairs([(id, 1.0)]), 60);
        let late = run(Truth { dt_s: 1.0, ..flying() }, &Faults::from_pairs([(id, 1.0)]), 1200);
        assert!(!holds(&disagree.trigger, &healthy), "a healthy receiver must not disagree: {:?}", healthy.get("DEEP_GPS_1_OFFSET_N_M"));
        assert!(!holds(&disagree.trigger, &early), "one minute of walk-off is ~60 m, far short of 0.5'");
        assert!(holds(&disagree.trigger, &late), "20 minutes of walk-off must pass 0.5': {:?}", late.get("DEEP_GPS_1_OFFSET_N_M"));
    }

    #[test]
    fn static_probe_fault_fires_when_one_port_of_a_pair_is_blocked() {
        let index = FaultIndex::build();
        let id = index.id("34_nav.static_1_1", "blocked");
        assert_ne!(id, 0, "the static port fault id could not be resolved");

        let healthy = run(flying(), &Faults::default(), 20);
        let armed = run(flying(), &Faults::from_pairs([(id, 1.0)]), 60);

        let p = wiring(340_800_067);
        assert!(!holds(&p.trigger, &healthy), "NAV STATIC PROBE FAULT must be quiet with every port clear");
        assert!(holds(&p.trigger, &armed), "NAV STATIC PROBE FAULT must fire once one port of a pair is blocked");
    }
}
