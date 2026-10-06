use super::{proc, sd_page, FbwProc};
use crate::deep::api::{all, any, var, Cond, Level};

fn network_alive() -> Cond {
    any(vec![
        var("ELEC_AC_1_BUS_IS_POWERED").on(),
        var("ELEC_AC_2_BUS_IS_POWERED").on(),
        var("ELEC_AC_3_BUS_IS_POWERED").on(),
        var("ELEC_AC_4_BUS_IS_POWERED").on(),
    ])
}

pub fn procs() -> Vec<FbwProc> {
    vec![
        proc(211800024, "COND BULK CARGO DUCT OVHT", Level::Caution, sd_page::COND, all(vec![any(vec![var("WIRING_ZONE_CARGO_AFT_OVERHEAT_SEVERITY").ne(0.0), var("WIRING_ZONE_CARGO_FWD_OVERHEAT_SEVERITY").ne(0.0)]), network_alive()]), "C47, C48: An overheating bundle in the aft cargo harness raises the zone overheat severity flag and drops the isol valve, extract fan and heater breakers, matching a real duct overheat. Failures: [12091009, 12091016, 12091023]")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(3.0).items(1, Vec::new()),
        proc(211800056, "AIR ABNORM BLEED CONFIG", Level::Caution, sd_page::COND, all(vec![all(vec![any(vec![all(vec![var("DEEP_ENG_1_N3_PICKUP_A_FRAC").gt(0.5), var("DEEP_PNEU_ENG_1_ISOLATION_OPEN").eq(0.0)]), all(vec![var("DEEP_ENG_2_N3_PICKUP_A_FRAC").gt(0.5), var("DEEP_PNEU_ENG_2_ISOLATION_OPEN").eq(0.0)]), all(vec![var("DEEP_ENG_3_N3_PICKUP_A_FRAC").gt(0.5), var("DEEP_PNEU_ENG_3_ISOLATION_OPEN").eq(0.0)]), all(vec![var("DEEP_ENG_4_N3_PICKUP_A_FRAC").gt(0.5), var("DEEP_PNEU_ENG_4_ISOLATION_OPEN").eq(0.0)])]), var("DEEP_PNEU_XBLEED_L_OPEN").eq(0.0), var("DEEP_PNEU_XBLEED_C_OPEN").eq(0.0), var("DEEP_PNEU_XBLEED_R_OPEN").eq(0.0)]), network_alive()]), "M00: corrected trigger (2 Oct): the alert as the aircraft raises it Failures: [15036029, 15036031, 15036032]")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(10.0).items(15, Vec::new()),
        proc(212800015, "VENT AVNCS EXTRACT FAULT", Level::Caution, sd_page::COND, all(vec![any(vec![var("AVNCS_AVIONICS_BAY_AFT_AIRFLOW_FRAC").lt(0.5), var("AVNCS_AVIONICS_BAY_FWD_AIRFLOW_FRAC").lt(0.5)]), network_alive()]), "B01, B01: extract valve stuck closed drops aft avionics bay airflow from fully open to none, a real extraction fault. Failures: [9021003, 9021006]")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10, 11]).confirm(5.0).items(1, Vec::new()),
        proc(220800007, "AUTO FLT AFS CTL PNL+CAPT BKUP CTL FAULT", Level::Caution, -1, all(vec![var("DEEP_AUTOFLT_CAPT_FCU_BKUP_FAULT").eq(1.0), network_alive()]), "B01: Captain's MFD FCU backup control path fails, the real AFS CTL PNL+CAPT BKUP CTL FAULT condition. Failures: [22022003]")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(2.0).items(1, Vec::new()),
        proc(220800008, "AUTO FLT AFS CTL PNL+F/O BKUP CTL FAULT", Level::Caution, -1, all(vec![var("DEEP_AUTOFLT_FO_FCU_BKUP_FAULT").eq(1.0), network_alive()]), "B01: F/O's MFD FCU backup control path fails, the real AFS CTL PNL+F/O BKUP CTL FAULT condition. Failures: [22022004]")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(2.0).items(1, Vec::new()),
        proc(220800014, "AUTO FLT TCAS MODE FAULT", Level::Caution, -1, all(vec![var("ELEC_LOAD_tcas_CUT").ne(0.0), network_alive()]), "D01: tcas loses its supply through its own breaker or wiring: the FWS monitors that LRU and raises its fault Failures: [1024534, 1024536, 1034065, 1034066, 1034068, 17034045]")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(2.0).items(3, Vec::new()),
        proc(230800003, "CAB COM COM DEGRADED", Level::Advisory, -1, all(vec![any(vec![var("DEEP_COM_CIDS_1_FAULT").eq(1.0), var("DEEP_COM_CIDS_2_FAULT").eq(1.0), var("DEEP_COM_CIDS_3_FAULT").eq(1.0)]), network_alive()]), "B02: Any one of the three CIDS directors faulting degrades cabin communications redundancy, matching the real COM DEGRADED advisory. Failures: [21023001, 21023002, 21023003]")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(5.0).items(2, Vec::new()),
        proc(230800004, "COM CAPT PTT STUCK", Level::Caution, -1, all(vec![var("DEEP_COM_CAPT_PTT_STUCK").eq(1.0), network_alive()]), "B02: A stuck captain PTT blocks the frequency, exactly the COM CAPT PTT STUCK caution. Failures: [21023008]")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(3.0).items(0, Vec::new()),
        proc(230800005, "COM F/O PTT STUCK", Level::Caution, -1, all(vec![var("DEEP_COM_FO_PTT_STUCK").eq(1.0), network_alive()]), "B02: A stuck F/O PTT blocks the frequency, exactly the COM F/O PTT STUCK caution. Failures: [21023009]")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(3.0).items(0, Vec::new()),
        proc(230800006, "COM THIRD OCCUPANT PTT STUCK", Level::Caution, -1, all(vec![var("DEEP_COM_THIRD_PTT_STUCK").eq(1.0), network_alive()]), "B02: A stuck third-occupant PTT blocks the frequency, exactly the COM THIRD OCCUPANT PTT STUCK caution. Failures: [21023010]")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(3.0).items(0, Vec::new()),
        proc(230800022, "COM VHF 1 EMITTING", Level::Caution, -1, all(vec![var("DEEP_COM_VHF1_EMITTING").eq(1.0), network_alive()]), "B02: VHF1 stuck transmitting is exactly the COM VHF1 EMITTING caution. Failures: [21023019]")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(3.0).items(1, Vec::new()),
        proc(230800023, "COM VHF 2 EMITTING", Level::Caution, -1, all(vec![var("DEEP_COM_VHF2_EMITTING").eq(1.0), network_alive()]), "B02: VHF2 stuck transmitting is exactly the COM VHF2 EMITTING caution. Failures: [21023020]")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(3.0).items(1, Vec::new()),
        proc(230800024, "COM VHF 3 EMITTING", Level::Caution, -1, all(vec![var("DEEP_COM_VHF3_EMITTING").eq(1.0), network_alive()]), "B02: VHF3 stuck transmitting is exactly the COM VHF3 EMITTING caution. Failures: [21023021]")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(3.0).items(1, Vec::new()),
        proc(240800002, "ELEC AC BUS 1 FAULT", Level::Caution, sd_page::ELEC_AC, all(vec![var("ELEC_AC_1_BUS_POTENTIAL").lt(90.0), var("ELEC_AC_1_BUS_IS_POWERED").off(), network_alive()]), "C02: AC1 bus short to ground drives the generator's own fault discrete from 0 to 1 and sags AC1 bus potential, matching an AC BUS 1 FAULT caution; also raised directly when AC1 bus potential drops below 90 V and the bus reports unpowered, covering bus-loss causes other than a generator fault (folded in from the deleted ELEC_AC_BUS_1_FAULT registry alert). Failures: [1024904]")
            .inhibit(&[4, 5, 10]).confirm(1.0).items(1, Vec::new()),
        proc(240800013, "ELEC APU BAT FAULT", Level::Caution, sd_page::ELEC_AC, all(vec![var("ELEC_APU_BAT_FAULT").eq(1.0), network_alive()]), "C06: Direct discrete match to the real APU battery fault caution. Failures: [1024953]")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(2.0).items(1, Vec::new()),
        proc(240800016, "ELEC APU TR FAULT", Level::Caution, sd_page::ELEC_AC, all(vec![var("ELEC_TR_APU_FAULT").eq(1.0), network_alive()]), "C06: TR-APU-LINE contactor failing to close drops the APU TR's own fault discrete, same as a real TR fault. Failures: [1024885]")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(3.0).items(2, Vec::new()),
        proc(240800019, "ELEC BUS TIE OFF", Level::Caution, sd_page::ELEC_AC, all(vec![var("ELEC_BUS_TIE_OFF").eq(1.0), network_alive()]), "C06: Direct discrete match to the real bus tie off caution. Failures: [1024962]")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(3.0).items(0, Vec::new()),
        proc(240800022, "ELEC CABIN L SUPPLY CENTER OVHT", Level::Caution, sd_page::ELEC_AC, all(vec![var("ELEC_CABIN_L_SUPPLY_CENTER_OVHT").eq(1.0), network_alive()]), "C06: Direct discrete match to the real cabin L supply centre overheat caution. Failures: [1024963]")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(1.0).items(1, Vec::new()),
        proc(240800023, "ELEC CABIN R SUPPLY CENTER OVHT", Level::Caution, sd_page::ELEC_AC, all(vec![var("ELEC_CABIN_R_SUPPLY_CENTER_OVHT").eq(1.0), network_alive()]), "C06: Direct discrete match to the real cabin R supply centre overheat caution. Failures: [1024964]")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(1.0).items(1, Vec::new()),
        proc(240800024, "ELEC CABIN L SUPPLY CENTER OVHT DET FAULT", Level::Caution, sd_page::ELEC_AC, all(vec![var("ELEC_CABIN_L_SUPPLY_CENTER_OVHT_DET_FAULT").eq(1.0), network_alive()]), "C06: Direct discrete match; the overheat detector itself has failed, not the overheat condition. Failures: [1024965]")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(2.0).items(0, Vec::new()),
        proc(240800025, "ELEC CABIN R SUPPLY CENTER OVHT DET FAULT", Level::Caution, sd_page::ELEC_AC, all(vec![var("ELEC_CABIN_R_SUPPLY_CENTER_OVHT_DET_FAULT").eq(1.0), network_alive()]), "C06: Direct discrete match; the overheat detector itself has failed, not the overheat condition. Failures: [1024966]")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(2.0).items(0, Vec::new()),
        proc(240800026, "ELEC \\x1b'mDC BUS 1 FAULT", Level::Caution, sd_page::ELEC_AC, all(vec![var("ELEC_TR_1_FAULT").eq(1.0), network_alive()]), "C03: TR1 fault flag sets and valves/VCMs on DC1 drop, matching ELEC DC BUS 1 FAULT. Failures: [1024912]")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(2.0).items(10, Vec::new()),
        proc(240800029, "ELEC \\x1b'mDC BUS 2 FAULT", Level::Caution, sd_page::ELEC_AC, all(vec![var("ELEC_TR_2_FAULT").eq(1.0), network_alive()]), "C03: TR2 fault flag sets and valves/VCMs on DC2 drop, matching ELEC DC BUS 2 FAULT. Failures: [1024913]")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(2.0).items(11, Vec::new()),
        proc(240800030, "ELEC \\x1b'mDC ESS BUS FAULT", Level::Caution, sd_page::ELEC_AC, all(vec![var("ELEC_TR_ESS_FAULT").eq(1.0), network_alive()]), "C03: TR ESS fault flag sets and DC ESS/ESS SHED potential sags, matching ELEC DC ESS BUS FAULT. Failures: [1024914]")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(2.0).items(20, Vec::new()),
        proc(240800032, "ELEC DRIVE 1 DISC FAULT", Level::Caution, sd_page::ELEC_AC, all(vec![var("ELEC_DRIVE_1_DISC_FAULT").eq(1.0), network_alive()]), "C06: Direct discrete match to the real IDG 1 disconnect-fault caution. Failures: [1024954]")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(2.0).items(0, Vec::new()),
        proc(240800033, "ELEC DRIVE 2 DISC FAULT", Level::Caution, sd_page::ELEC_AC, all(vec![var("ELEC_DRIVE_2_DISC_FAULT").eq(1.0), network_alive()]), "C06: Direct discrete match to the real IDG 2 disconnect-fault caution. Failures: [1024955]")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(2.0).items(0, Vec::new()),
        proc(240800034, "ELEC DRIVE 3 DISC FAULT", Level::Caution, sd_page::ELEC_AC, all(vec![var("ELEC_DRIVE_3_DISC_FAULT").eq(1.0), network_alive()]), "C06: Direct discrete match to the real IDG 3 disconnect-fault caution. Failures: [1024956]")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(2.0).items(0, Vec::new()),
        proc(240800035, "ELEC DRIVE 4 DISC FAULT", Level::Caution, sd_page::ELEC_AC, all(vec![var("ELEC_DRIVE_4_DISC_FAULT").eq(1.0), network_alive()]), "C06: Direct discrete match to the real IDG 4 disconnect-fault caution. Failures: [1024957]")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(2.0).items(0, Vec::new()),
        proc(240800056, "ELEC EXT PWR 1 FAULT", Level::Caution, sd_page::ELEC_AC, all(vec![var("ELEC_EXT_PWR_1_FAULT").eq(1.0), network_alive()]), "C06: Direct discrete match to the real EXT PWR 1 fault caution. Failures: [1024973]")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(2.0).items(1, Vec::new()),
        proc(240800057, "ELEC EXT PWR 2 FAULT", Level::Caution, sd_page::ELEC_AC, all(vec![var("ELEC_EXT_PWR_2_FAULT").eq(1.0), network_alive()]), "C06: Direct discrete match to the real EXT PWR 2 fault caution. Failures: [1024974]")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(2.0).items(1, Vec::new()),
        proc(240800058, "ELEC EXT PWR 3 FAULT", Level::Caution, sd_page::ELEC_AC, all(vec![var("ELEC_EXT_PWR_3_FAULT").eq(1.0), network_alive()]), "C06: Direct discrete match to the real EXT PWR 3 fault caution. Failures: [1024975]")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(2.0).items(1, Vec::new()),
        proc(240800059, "ELEC EXT PWR 4 FAULT", Level::Caution, sd_page::ELEC_AC, all(vec![var("ELEC_EXT_PWR_4_FAULT").eq(1.0), network_alive()]), "C06: Direct discrete match to the real EXT PWR 4 fault caution. Failures: [1024976]")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(2.0).items(1, Vec::new()),
        proc(240800060, "ELEC F/CTL ACTUATOR PWR SUPPLY FAULT", Level::Caution, sd_page::ELEC_AC, all(vec![var("ELEC_FCTL_ACTUATOR_PWR_FAULT").eq(1.0), network_alive()]), "C06: Direct discrete match to the real flight control actuator power supply fault caution. Failures: [1024950]")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(2.0).items(0, Vec::new()),
        proc(240800070, "ELEC PRIMARY SUPPLY CENTER 1 FAULT", Level::Caution, sd_page::ELEC_AC, all(vec![var("ELEC_PSC_1_FAULT").eq(1.0), network_alive()]), "C06: Direct discrete match to the real Primary Supply Centre 1 fault caution. Failures: [1024951]")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(2.0).items(0, Vec::new()),
        proc(240800071, "ELEC PRIMARY SUPPLY CENTER 2 FAULT", Level::Caution, sd_page::ELEC_AC, all(vec![var("ELEC_PSC_2_FAULT").eq(1.0), network_alive()]), "C06: Direct discrete match to the real Primary Supply Centre 2 fault caution. Failures: [1024952]")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(2.0).items(0, Vec::new()),
        proc(240800072, "ELEC RAT FAULT", Level::Caution, sd_page::ELEC_AC, all(vec![any(vec![var("ELEC_RAT_FAULT").ne(0.0), var("ELEC_EMER_GEN_OUTPUT_CAPABILITY_LOSS").ne(0.0)]), network_alive()]), "C07: A jammed RAT sets its own fault flag and the emergency-gen capability-loss flag together; FBW's RAT FAULT procedure is the real cockpit message for both. Failures: [1024942]")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(3.0).items(0, Vec::new()),
        proc(240800074, "ELEC SECONDARY SUPPLY CENTER 1 DEGRADED", Level::Caution, sd_page::ELEC_AC, all(vec![var("ELEC_SSC_1_DEGRADED").ne(0.0), network_alive()]), "C07: SSC1 comm degraded is exactly the modelled SSC1 DEGRADED flag. Failures: [1024967]")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(3.0).items(0, Vec::new()),
        proc(240800075, "ELEC SECONDARY SUPPLY CENTER 2 DEGRADED", Level::Caution, sd_page::ELEC_AC, all(vec![var("ELEC_SSC_2_DEGRADED").ne(0.0), network_alive()]), "C07: SSC2 comm degraded mirrors SSC1 case on the other supply center. Failures: [1024968]")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(3.0).items(0, Vec::new()),
        proc(240800076, "ELEC SECONDARY SUPPLY CENTER 1 FAULT", Level::Caution, sd_page::ELEC_AC, all(vec![var("ELEC_SSC_1_FAULT").ne(0.0), network_alive()]), "C07: Supply fault flag is the modelled equivalent of SSC1 FAULT. Failures: [1024969]")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(3.0).items(0, Vec::new()),
        proc(240800077, "ELEC SECONDARY SUPPLY CENTER 2 FAULT", Level::Caution, sd_page::ELEC_AC, all(vec![var("ELEC_SSC_2_FAULT").ne(0.0), network_alive()]), "C07: Supply fault flag is the modelled equivalent of SSC2 FAULT. Failures: [1024970]")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(3.0).items(0, Vec::new()),
        proc(240800078, "ELEC SECONDARY SUPPLY CENTER 1 REDUND LOST", Level::Caution, sd_page::ELEC_AC, all(vec![var("ELEC_SSC_1_REDUND_LOST").ne(0.0), network_alive()]), "C07: Redundancy-lost flag maps directly to the matching FBW procedure. Failures: [1024971]")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(3.0).items(0, Vec::new()),
        proc(240800079, "ELEC SECONDARY SUPPLY CENTER 2 REDUND LOST", Level::Caution, sd_page::ELEC_AC, all(vec![var("ELEC_SSC_2_REDUND_LOST").ne(0.0), network_alive()]), "C07: Redundancy-lost flag maps directly to the matching FBW procedure. Failures: [1024972]")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(3.0).items(0, Vec::new()),
        proc(240800080, "ELEC STATIC INV FAULT", Level::Caution, sd_page::ELEC_AC, all(vec![var("ELEC_STATIC_INV_FAULT").eq(1.0), network_alive()]), "M00: corrected trigger (2 Oct): the alert as the aircraft raises it Failures: [1024941]")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(2.0).items(0, Vec::new()),
        proc(240800084, "ELEC TR 1 MONITORING FAULT", Level::Caution, sd_page::ELEC_AC, all(vec![var("ELEC_TR_1_MONITORING_FAULT").ne(0.0), network_alive()]), "C07: TR1 monitoring fault flag is a direct match for the FBW procedure of the same name. Failures: [1024947]")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(3.0).items(0, Vec::new()),
        proc(240800085, "ELEC TR 2 MONITORING FAULT", Level::Caution, sd_page::ELEC_AC, all(vec![var("ELEC_TR_2_MONITORING_FAULT").ne(0.0), network_alive()]), "C07: TR2 monitoring fault flag is a direct match for the FBW procedure of the same name. Failures: [1024948]")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(3.0).items(0, Vec::new()),
        proc(240800086, "ELEC TR ESS MONITORING FAULT", Level::Caution, sd_page::ELEC_AC, all(vec![var("ELEC_TR_ESS_MONITORING_FAULT").ne(0.0), network_alive()]), "C07: TR ESS monitoring fault flag is a direct match for the FBW procedure of the same name. Failures: [1024949]")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(3.0).items(0, Vec::new()),
        proc(260800029, "SMOKE AFT AVNCS DET FAULT", Level::Caution, -1, all(vec![var("DEEP_SMOKE_AVNCS_AFT_FAULT").eq(1.0), network_alive()]), "M00: corrected trigger (2 Oct): the alert as the aircraft raises it Failures: [6026072]")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(2.0).items(0, Vec::new()),
        proc(260800030, "SMOKE AFT AVNCS SMOKE", Level::Warning, -1, all(vec![any(vec![var("FIRE_ZONE_AVIONICS_BURNING").eq(1.0), any(vec![var("FIRE_LOOP_A_AVIONICS_FIRE").eq(1.0), var("FIRE_LOOP_B_AVIONICS_FIRE").eq(1.0), var("FIRE_LOOP_AVIONICS_DISAGREE").eq(1.0)]), var("DEEP_SMOKE_AVNCS_AFT_ALARM").eq(1.0)]), network_alive()]), "B05, B05, B06: A leak feeding an avionics bay fire is a real burn, the same alert the bay's smoke detection gives for any avionics fire. Failures: [6026070, 8026034, 8026036, 8026108]")
            .inhibit(&[]).confirm(1.0).items(5, Vec::new()),
        proc(260800031, "SMOKE DET FAULT", Level::Caution, -1, all(vec![any(vec![var("DEEP_SMOKE_LAV_1_FAULT").eq(1.0), var("DEEP_SMOKE_LAV_2_FAULT").eq(1.0), var("DEEP_SMOKE_LAV_3_FAULT").eq(1.0), var("DEEP_SDF_CONFIGURATION_FAULT").eq(1.0)]), network_alive()]), "B06: Any lavatory detector circuit fault maps to the generic SMOKE DET FAULT caution; deck can't be resolved for a more specific one. Failures: [6026020, 6026024, 6026028]; titles-systems: DEEP_SDF_CONFIGURATION_FAULT (6026141) OR'd in, replacing the invented WIRED_SMOKE_DET_SYS_FAULT registry alert")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(2.0).items(6, Vec::new()),
        proc(260800034, "SMOKE L MAIN AVNCS DET FAULT", Level::Caution, -1, all(vec![var("DEEP_SMOKE_AVNCS_MAIN_L_FAULT").eq(1.0), network_alive()]), "M00: corrected trigger (2 Oct): the alert as the aircraft raises it Failures: [6026056]")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(2.0).items(0, Vec::new()),
        proc(260800035, "SMOKE R MAIN AVNCS DET FAULT", Level::Caution, -1, all(vec![var("DEEP_SMOKE_AVNCS_MAIN_R_FAULT").eq(1.0), network_alive()]), "M00: corrected trigger (2 Oct): the alert as the aircraft raises it Failures: [6026060]")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(2.0).items(0, Vec::new()),
        proc(260800036, "SMOKE L UPPER AVNCS DET FAULT", Level::Caution, -1, all(vec![var("DEEP_SMOKE_AVNCS_UPPER_L_FAULT").eq(1.0), network_alive()]), "M00: corrected trigger (2 Oct): the alert as the aircraft raises it Failures: [6026064]")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(2.0).items(0, Vec::new()),
        proc(260800037, "SMOKE R UPPER AVNCS DET FAULT", Level::Caution, -1, all(vec![var("DEEP_SMOKE_AVNCS_UPPER_R_FAULT").eq(1.0), network_alive()]), "M00: corrected trigger (2 Oct): the alert as the aircraft raises it Failures: [6026068]")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(2.0).items(0, Vec::new()),
        proc(260800038, "SMOKE L MAIN AVNCS SMOKE", Level::Warning, -1, all(vec![var("DEEP_SMOKE_AVNCS_MAIN_L_ALARM").eq(1.0), network_alive()]), "B06: Spurious alarm reads as real smoke: L MAIN AVNCS SMOKE. Failures: [6026054]")
            .inhibit(&[]).confirm(1.0).items(6, Vec::new()),
        proc(260800039, "SMOKE R MAIN AVNCS SMOKE", Level::Warning, -1, all(vec![var("DEEP_SMOKE_AVNCS_MAIN_R_ALARM").eq(1.0), network_alive()]), "B06: Spurious alarm reads as real smoke: R MAIN AVNCS SMOKE. Failures: [6026058]")
            .inhibit(&[]).confirm(1.0).items(6, Vec::new()),
        proc(260800040, "SMOKE L UPPER AVNCS SMOKE", Level::Warning, -1, all(vec![var("DEEP_SMOKE_AVNCS_UPPER_L_ALARM").eq(1.0), network_alive()]), "B06: Spurious alarm reads as real smoke: L UPPER AVNCS SMOKE. Failures: [6026062]")
            .inhibit(&[]).confirm(1.0).items(8, Vec::new()),
        proc(260800041, "SMOKE R UPPER AVNCS SMOKE", Level::Warning, -1, all(vec![var("DEEP_SMOKE_AVNCS_UPPER_R_ALARM").eq(1.0), network_alive()]), "B06: Spurious alarm reads as real smoke: R UPPER AVNCS SMOKE. Failures: [6026066]")
            .inhibit(&[]).confirm(1.0).items(8, Vec::new()),
        proc(260800043, "SMOKE FWD CARGO BOTTLES FAULT", Level::Caution, -1, all(vec![any(vec![var("FIRE_CARGO_FWD_KNOCKDOWN_SQUIB_FAULT").eq(1.0), var("FIRE_CARGO_FWD_EXTENDED_SQUIB_FAULT").eq(1.0), var("FIRE_CARGO_FWD_DISTRIBUTION_FAULT").eq(1.0)]), network_alive()]), "B05: Any Cargo FWD bottle squib or distribution fault degrades the suppression system. Failures: [8026220, 8026221, 8026222]")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(2.0).items(0, Vec::new()),
        proc(260800044, "SMOKE AFT CARGO BOTTLES FAULT", Level::Caution, -1, all(vec![any(vec![var("FIRE_CARGO_AFT_KNOCKDOWN_SQUIB_FAULT").eq(1.0), var("FIRE_CARGO_AFT_EXTENDED_SQUIB_FAULT").eq(1.0), var("FIRE_CARGO_AFT_DISTRIBUTION_FAULT").eq(1.0)]), network_alive()]), "B05: Any Cargo AFT bottle squib or distribution fault degrades the suppression system. Failures: [8026224, 8026225, 8026226]")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(2.0).items(0, Vec::new()),
        proc(260800045, "SMOKE FWD CARGO SMOKE", Level::Warning, -1, all(vec![any(vec![any(vec![var("FIRE_LOOP_A_CARGO_FWD_FIRE").eq(1.0), var("FIRE_LOOP_B_CARGO_FWD_FIRE").eq(1.0), var("FIRE_LOOP_CARGO_FWD_DISAGREE").eq(1.0)]), var("DEEP_SMOKE_FWD_CARGO_B_ALARM").eq(1.0), var("CARGO_FWD_SMOKE_DETECTED").eq(1.0)]), network_alive()]), "B05, B06: A shorted Cargo FWD loop reads as fire or disagreement, the real SMOKE warning; also raised directly by the deep optical smoke detector. Failures: [6026006, 8026026, 8026028]")
            .inhibit(&[4, 5, 6, 9, 10]).confirm(1.0).items(8, Vec::new()),
        proc(260800046, "SMOKE AFT CARGO SMOKE", Level::Warning, -1, all(vec![any(vec![any(vec![var("FIRE_LOOP_A_CARGO_AFT_FIRE").eq(1.0), var("FIRE_LOOP_B_CARGO_AFT_FIRE").eq(1.0), var("FIRE_LOOP_CARGO_AFT_DISAGREE").eq(1.0)]), var("DEEP_SMOKE_AFT_CARGO_A_ALARM").eq(1.0), var("DEEP_SMOKE_AFT_CARGO_B_ALARM").eq(1.0), var("CARGO_AFT_SMOKE_DETECTED").eq(1.0)]), network_alive()]), "B05, B05, B06: A shorted Cargo AFT loop reads as fire or disagreement, the real SMOKE warning; also raised directly by the deep optical smoke detector. Failures: [6026010, 6026014, 8026030, 8026032]")
            .inhibit(&[4, 5, 6, 9, 10]).confirm(1.0).items(8, Vec::new()),
        proc(260800047, "SMOKE BULK CARGO SMOKE", Level::Warning, -1, all(vec![var("THERMAL_ZONE_CARGOBULK_SMOKE_CONCENTRATION").gt(0.0002), network_alive()]), "No optical bulk smoke detector is modelled; the real CargoBulk content-fire smoke concentration crossing the detection threshold is the only real signal available for this title. Failures: [11026003]")
            .inhibit(&[4, 5, 6, 9, 10]).confirm(1.0).items(8, Vec::new()),
        proc(260800048, "SMOKE FWD CARGO DET FAULT", Level::Caution, -1, all(vec![any(vec![var("DEEP_SMOKE_FWD_CARGO_A_FAULT").eq(1.0), var("DEEP_SMOKE_FWD_CARGO_B_FAULT").eq(1.0)]), network_alive()]), "M00: corrected trigger (2 Oct): the alert as the aircraft raises it Failures: [6026004, 6026008]")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(2.0).items(2, Vec::new()),
        proc(260800049, "SMOKE AFT CARGO DET FAULT", Level::Caution, -1, all(vec![any(vec![var("DEEP_SMOKE_AFT_CARGO_A_FAULT").eq(1.0), var("DEEP_SMOKE_AFT_CARGO_B_FAULT").eq(1.0)]), network_alive()]), "M00: corrected trigger (2 Oct): the alert as the aircraft raises it Failures: [6026012, 6026016]")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(2.0).items(3, Vec::new()),
        proc(260800050, "SMOKE BULK CARGO DET FAULT", Level::Caution, -1, all(vec![var("DEEP_SMOKE_CARGO_BULK_FAULT").eq(1.0), network_alive()]), "B06: Bulk cargo detector circuit fault maps to BULK CARGO DET FAULT. Failures: [6026052]")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(2.0).items(3, Vec::new()),
        proc(260800054, "SMOKE FWD LWR CAB REST BTL 1 FAULT", Level::Caution, -1, all(vec![var("FIRE_LDCR_BTL_1_SQUIB_FAULT").eq(1.0), network_alive()]), "B05: LDCR bottle 1 squib circuit fault is exactly the FWD LWR CAB REST BTL 1 fault message. Failures: [8026230]")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(2.0).items(0, Vec::new()),
        proc(260800055, "SMOKE FWD LWR CAB REST BTL 2 FAULT", Level::Caution, -1, all(vec![var("FIRE_LDCR_BTL_2_SQUIB_FAULT").eq(1.0), network_alive()]), "B05: LDCR bottle 2 squib circuit fault is exactly the FWD LWR CAB REST BTL 2 fault message. Failures: [8026231]")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(2.0).items(0, Vec::new()),
        proc(260800056, "SMOKE FWD LWR CAB REST DET FAULT", Level::Caution, -1, all(vec![var("DEEP_SMOKE_FWDLOWERCREWREST_FAULT").eq(1.0), network_alive()]), "M00: corrected trigger (2 Oct): the alert as the aircraft raises it Failures: [6026140]")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(2.0).items(0, Vec::new()),
        proc(260800057, "SMOKE FWD LWR CAB REST SMOKE", Level::Warning, -1, all(vec![var("DEEP_SMOKE_FWDLOWERCREWREST_ALARM").eq(1.0), network_alive()]), "B06: Spurious alarm reads as real smoke: FWD LWR CAB REST SMOKE. Failures: [6026138]")
            .inhibit(&[]).confirm(1.0).items(4, Vec::new()),
        proc(260800058, "SMOKE MAIN 5L FLT REST DET FAULT", Level::Caution, -1, all(vec![var("DEEP_SMOKE_MAIN5L_FLTREST_FAULT").eq(1.0), network_alive()]), "M00: corrected trigger (2 Oct): the alert as the aircraft raises it Failures: [6026076]")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(2.0).items(0, Vec::new()),
        proc(260800059, "SMOKE MAIN 5L CAB REST DET FAULT", Level::Caution, -1, all(vec![var("DEEP_SMOKE_MAIN5L_CABREST_FAULT").eq(1.0), network_alive()]), "M00: corrected trigger (2 Oct): the alert as the aircraft raises it Failures: [6026080]")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(2.0).items(0, Vec::new()),
        proc(260800060, "SMOKE MAIN 5L FLT REST SMOKE", Level::Warning, -1, all(vec![var("DEEP_SMOKE_MAIN5L_FLTREST_ALARM").ge(1.0), network_alive()]), "B07: spurious signal raises the real smoke-alarm output for MAIN 5L Flight Rest. Failures: [6026074]")
            .inhibit(&[]).confirm(3.0).items(2, Vec::new()),
        proc(260800061, "SMOKE MAIN 5L CAB REST SMOKE", Level::Warning, -1, all(vec![var("DEEP_SMOKE_MAIN5L_CABREST_ALARM").ge(1.0), network_alive()]), "B07: spurious signal raises the real smoke-alarm output for MAIN 5L Cabin Rest. Failures: [6026078]")
            .inhibit(&[]).confirm(3.0).items(2, Vec::new()),
        proc(260800062, "SMOKE MAIN DECK LAVATORY DET FAULT", Level::Caution, -1, all(vec![any(vec![var("DEEP_SMOKE_LAV_1_FAULT").eq(1.0), var("DEEP_SMOKE_LAV_2_FAULT").eq(1.0), var("DEEP_SMOKE_LAV_3_FAULT").eq(1.0), var("DEEP_SMOKE_LAV_4_FAULT").eq(1.0)]), network_alive()]), "M00: corrected trigger (2 Oct): the alert as the aircraft raises it Failures: [6026020, 6026024, 6026028, 6026032]")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(2.0).items(0, Vec::new()),
        proc(260800063, "SMOKE UPPER DECK LAVATORY DET FAULT", Level::Caution, -1, all(vec![any(vec![var("DEEP_SMOKE_LAV_5_FAULT").eq(1.0), var("DEEP_SMOKE_LAV_6_FAULT").eq(1.0), var("DEEP_SMOKE_LAV_7_FAULT").eq(1.0), var("DEEP_SMOKE_LAV_8_FAULT").eq(1.0)]), network_alive()]), "M00: corrected trigger (2 Oct): the alert as the aircraft raises it Failures: [6026036, 6026040, 6026044, 6026048]")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(2.0).items(0, Vec::new()),
        proc(260800064, "SMOKE LOWER DECK LAVATORY DET FAULT", Level::Caution, -1, all(vec![var("DEEP_SMOKE_FWDLOWERCREWREST_FAULT").eq(1.0), network_alive()]), "M00: corrected trigger (2 Oct): the alert as the aircraft raises it Failures: [6026140]")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(2.0).items(0, Vec::new()),
        proc(260800067, "SMOKE LOWER DECK LAVATORY SMOKE", Level::Warning, -1, all(vec![var("DEEP_SMOKE_LAV_3_ALARM").eq(1.0), network_alive()]), "C08: Spurious signal sets the lav 3 detector's alarm flag, producing a real smoke warning. Failures: [6026026]")
            .inhibit(&[]).confirm(1.0).items(2, Vec::new()),
        proc(260800068, "SMOKE MAIN 1L CWS DET FAULT", Level::Caution, -1, all(vec![var("DEEP_SMOKE_MAIN_1L_CWS_FAULT").eq(1.0), network_alive()]), "M00: corrected trigger (2 Oct): the alert as the aircraft raises it Failures: [6026084]")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(2.0).items(2, Vec::new()),
        proc(260800069, "SMOKE MAIN 1L RCC DET FAULT", Level::Advisory, -1, all(vec![var("DEEP_SMOKE_MAIN_1L_RCC_FAULT").eq(1.0), network_alive()]), "M00: corrected trigger (2 Oct): the alert as the aircraft raises it Failures: [6026088]")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(10.0).items(2, Vec::new()),
        proc(260800072, "SMOKE MAIN 2L CWS DET FAULT", Level::Advisory, -1, all(vec![var("DEEP_SMOKE_MAIN_2L_CWS_FAULT").eq(1.0), network_alive()]), "M00: corrected trigger (2 Oct): the alert as the aircraft raises it Failures: [6026100]")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(10.0).items(2, Vec::new()),
        proc(260800073, "SMOKE MAIN 2L RCC DET FAULT", Level::Advisory, -1, all(vec![var("DEEP_SMOKE_MAIN_2L_RCC_FAULT").eq(1.0), network_alive()]), "M00: corrected trigger (2 Oct): the alert as the aircraft raises it Failures: [6026104]")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(10.0).items(2, Vec::new()),
        proc(260800075, "SMOKE UPPER 2L RCC DET FAULT", Level::Caution, -1, all(vec![var("DEEP_SMOKE_UPPER_2L_RCC_FAULT").eq(1.0), network_alive()]), "M00: corrected trigger (2 Oct): the alert as the aircraft raises it Failures: [6026112]")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(3.0).items(2, Vec::new()),
        proc(260800076, "SMOKE MAIN 3R CWS DET FAULT", Level::Advisory, -1, all(vec![var("DEEP_SMOKE_MAIN_3R_CWS_FAULT").eq(1.0), network_alive()]), "M00: corrected trigger (2 Oct): the alert as the aircraft raises it Failures: [6026116]")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(10.0).items(2, Vec::new()),
        proc(260800077, "SMOKE MAIN 3R RCC DET FAULT", Level::Advisory, -1, all(vec![var("DEEP_SMOKE_MAIN_3R_RCC_FAULT").eq(1.0), network_alive()]), "M00: corrected trigger (2 Oct): the alert as the aircraft raises it Failures: [6026120]")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(10.0).items(2, Vec::new()),
        proc(260800078, "SMOKE UPPER 3R CWS DET FAULT", Level::Caution, -1, all(vec![var("DEEP_SMOKE_UPPER_3R_CWS_FAULT").eq(1.0), network_alive()]), "C10: Circuit/self-test fault is a discrete BITE flag the detector reports on itself, independent of any smoke event. Failures: [6026124]")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(3.0).items(2, Vec::new()),
        proc(260800079, "SMOKE UPPER 3R RCC DET FAULT", Level::Caution, -1, all(vec![var("DEEP_SMOKE_UPPER_3R_RCC_FAULT").eq(1.0), network_alive()]), "C10: Circuit/self-test fault is a discrete BITE flag the detector reports on itself, independent of any smoke event. Failures: [6026128]")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(3.0).items(2, Vec::new()),
        proc(260800080, "SMOKE MAIN 1L CWS SMOKE", Level::Warning, -1, all(vec![var("DEEP_SMOKE_MAIN_1L_CWS_ALARM").ge(1.0), network_alive()]), "B07: spurious signal raises the real smoke-alarm output for MAIN 1L CWS. Failures: [6026082]")
            .inhibit(&[]).confirm(3.0).items(3, Vec::new()),
        proc(260800081, "SMOKE MAIN 1L RCC SMOKE", Level::Warning, -1, all(vec![var("DEEP_SMOKE_MAIN_1L_RCC_ALARM").ge(1.0), network_alive()]), "B07: spurious signal raises the real smoke-alarm output for MAIN 1L RCC. Failures: [6026086]")
            .inhibit(&[]).confirm(3.0).items(3, Vec::new()),
        proc(260800082, "SMOKE UPPER 1L CWS SMOKE", Level::Warning, -1, all(vec![var("DEEP_SMOKE_UPPER_1L_CWS_ALARM").eq(1.0), network_alive()]), "C09: Spurious signal sets the detector's alarm bit with no real smoke present, exactly as a false SMOKE warning would. Failures: [6026090]")
            .inhibit(&[]).confirm(1.0).items(3, Vec::new()),
        proc(260800083, "SMOKE UPPER 1L RCC SMOKE", Level::Warning, -1, all(vec![var("DEEP_SMOKE_UPPER_1L_RCC_ALARM").eq(1.0), network_alive()]), "C09: Spurious signal sets the detector's alarm bit with no real smoke present, exactly as a false SMOKE warning would. Failures: [6026094]")
            .inhibit(&[]).confirm(1.0).items(3, Vec::new()),
        proc(260800084, "SMOKE MAIN 2L CWS SMOKE", Level::Warning, -1, all(vec![var("DEEP_SMOKE_MAIN_2L_CWS_ALARM").ge(1.0), network_alive()]), "B07: spurious signal raises the real smoke-alarm output for MAIN 2L CWS. Failures: [6026098]")
            .inhibit(&[]).confirm(3.0).items(3, Vec::new()),
        proc(260800085, "SMOKE MAIN 2L RCC SMOKE", Level::Warning, -1, all(vec![var("DEEP_SMOKE_MAIN_2L_RCC_ALARM").ge(1.0), network_alive()]), "B07: spurious signal raises the real smoke-alarm output for MAIN 2L RCC. Failures: [6026102]")
            .inhibit(&[]).confirm(3.0).items(3, Vec::new()),
        proc(260800086, "SMOKE UPPER 2L CWS SMOKE", Level::Warning, -1, all(vec![var("DEEP_SMOKE_UPPER_2L_CWS_ALARM").eq(1.0), network_alive()]), "C09: Spurious signal sets the detector's alarm bit with no real smoke present, exactly as a false SMOKE warning would. Failures: [6026106]")
            .inhibit(&[]).confirm(1.0).items(3, Vec::new()),
        proc(260800087, "SMOKE UPPER 2L RCC SMOKE", Level::Warning, -1, all(vec![var("DEEP_SMOKE_UPPER_2L_RCC_ALARM").eq(1.0), network_alive()]), "C09: Spurious signal sets the detector's alarm bit with no real smoke present, exactly as a false SMOKE warning would. Failures: [6026110]")
            .inhibit(&[]).confirm(1.0).items(3, Vec::new()),
        proc(260800088, "SMOKE MAIN 3R CWS SMOKE", Level::Warning, -1, all(vec![var("DEEP_SMOKE_MAIN_3R_CWS_ALARM").ge(1.0), network_alive()]), "B07: spurious signal raises the real smoke-alarm output for MAIN 3R CWS. Failures: [6026114]")
            .inhibit(&[]).confirm(3.0).items(3, Vec::new()),
        proc(260800089, "SMOKE MAIN 3R RCC SMOKE", Level::Warning, -1, all(vec![var("DEEP_SMOKE_MAIN_3R_RCC_ALARM").ge(1.0), network_alive()]), "B07: spurious signal raises the real smoke-alarm output for MAIN 3R RCC. Failures: [6026118]")
            .inhibit(&[]).confirm(3.0).items(3, Vec::new()),
        proc(260800090, "SMOKE UPPER 3R CWS SMOKE", Level::Warning, -1, all(vec![var("DEEP_SMOKE_UPPER_3R_CWS_ALARM").eq(1.0), network_alive()]), "C10: Spurious signal makes the detector report smoke exactly as a real fire would, so the genuine SMOKE warning fires. Failures: [6026122]")
            .inhibit(&[]).confirm(1.0).items(3, Vec::new()),
        proc(260800091, "SMOKE UPPER 3R RCC SMOKE", Level::Warning, -1, all(vec![var("DEEP_SMOKE_UPPER_3R_RCC_ALARM").eq(1.0), network_alive()]), "C10: Spurious signal makes the detector report smoke exactly as a real fire would, so the genuine SMOKE warning fires. Failures: [6026126]")
            .inhibit(&[]).confirm(1.0).items(3, Vec::new()),
        proc(260800092, "SMOKE SAFETY TEST REQUIRED", Level::Advisory, -1, all(vec![var("DEEP_SDF_SAFETY_TEST_OVERDUE").eq(1.0), network_alive()]), "B06: Overdue automatic safety test maps directly to the SMOKE SAFETY TEST REQUIRED advisory. Failures: [6026142]")
            .inhibit(&[]).confirm(1.0).items(0, Vec::new()),
        proc(260800095, "SMOKE UPPER 1L SHOWER SMOKE", Level::Warning, -1, all(vec![var("DEEP_SMOKE_UPPER_1L_SHOWER_ALARM").eq(1.0), network_alive()]), "C09: Spurious signal sets the detector's alarm bit with no real smoke present, exactly as a false SMOKE warning would. Failures: [6026130]")
            .inhibit(&[]).confirm(1.0).items(2, Vec::new()),
        proc(260800096, "SMOKE UPPER 1R SHOWER SMOKE", Level::Warning, -1, all(vec![var("DEEP_SMOKE_UPPER_1R_SHOWER_ALARM").eq(1.0), network_alive()]), "C09: Spurious signal sets the detector's alarm bit with no real smoke present, exactly as a false SMOKE warning would. Failures: [6026134]")
            .inhibit(&[]).confirm(1.0).items(2, Vec::new()),
        proc(271800016, "F/CTL GND SPLRs FAULT", Level::Caution, sd_page::FCTL, all(vec![var("FCTL_GND_SPLR_LOGIC_FAULT").eq(1.0), network_alive()]), "B09: Ground spoiler deploy/retract logic failure is exactly the condition behind F/CTL GND SPLRs FAULT. Failures: [4027359, 4027360]")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(2.0).items(0, Vec::new()),
        proc(271800027, "F/CTL L SIDESTICK SENSOR FAULT", Level::Caution, sd_page::FCTL, all(vec![any(vec![var("FCTL_L_SIDESTICK_PITCH_FAULT").eq(1.0), var("FCTL_L_SIDESTICK_ROLL_FAULT").eq(1.0)]), network_alive()]), "B09: Captain's sidestick pitch/roll transducer open circuit or drift is a sensor fault, shown as F/CTL L SIDESTICK SENSOR FAULT. Failures: [4027364, 4027365, 4027366, 4027367, 4027368, 4027369, 4027370, 4027371]")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(1.0).items(0, Vec::new()),
        proc(271800028, "F/CTL R SIDESTICK SENSOR FAULT", Level::Caution, sd_page::FCTL, all(vec![any(vec![var("FCTL_R_SIDESTICK_PITCH_FAULT").eq(1.0), var("FCTL_R_SIDESTICK_ROLL_FAULT").eq(1.0)]), network_alive()]), "B09: F.O.'s sidestick pitch/roll transducer open circuit or drift is a sensor fault, shown as F/CTL R SIDESTICK SENSOR FAULT. Failures: [4027390, 4027391, 4027392, 4027393, 4027394, 4027395, 4027396, 4027397]")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(1.0).items(0, Vec::new()),
        proc(271800029, "F/CTL LOAD ALLEVIATION FAULT", Level::Caution, sd_page::FCTL, all(vec![var("FCTL_LOAD_ALLEVIATION_FAULT").eq(1.0), network_alive()]), "B09: A wing accelerometer failure for the load alleviation function is exactly F/CTL LOAD ALLEVIATION FAULT. Failures: [4027407, 4027408, 4027409, 4027410, 4027411, 4027412]")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(2.0).items(0, Vec::new()),
        proc(271800050, "F/CTL RUDDER PEDAL FAULT", Level::Caution, sd_page::FCTL, all(vec![var("FCTL_RUDDER_PEDAL_FAULT").eq(1.0), network_alive()]), "B10: Either rudder pedal transducer channel failing raises the FCTL_RUDDER_PEDAL_FAULT flag, which is exactly the F/CTL RUDDER PEDAL FAULT caution. Failures: [4027372, 4027373, 4027374, 4027375]")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(2.0).items(0, Vec::new()),
        proc(271800056, "F/CTL RUDDER TRIM FAULT", Level::Caution, sd_page::FCTL, all(vec![var("FCTL_RUDDER_TRIM_FAULT").eq(1.0), network_alive()]), "B10: Rudder trim motor failure or jam raises FCTL_RUDDER_TRIM_FAULT, matching F/CTL RUDDER TRIM FAULT. Failures: [4027315, 4027316]")
            .inhibit(&[4, 5, 6, 7, 9, 10]).confirm(2.0).items(0, Vec::new()),
        proc(272800016, "F/CTL FLAPS LOCKED", Level::Caution, sd_page::FCTL, all(vec![any(vec![var("FCTL_FLAP_L_FAULT").eq(1.0), var("FCTL_FLAP_R_FAULT").eq(1.0)]), network_alive()]), "B09: A flap drive-line fault (jam, hardover, shaft break, etc.) is sensed by the SFCC and the flap system is locked, shown as F/CTL FLAPS LOCKED. Failures: [4027317, 4027318, 4027319, 4027320, 4027321, 4027322, 4027323, 4027324]")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(2.0).items(0, Vec::new()),
        proc(272800024, "F/CTL SLATS LOCKED", Level::Caution, sd_page::FCTL, all(vec![any(vec![var("FCTL_DROOP_L_FAULT").eq(1.0), var("FCTL_DROOP_R_FAULT").eq(1.0)]), network_alive()]), "B09: A droop-nose drive-line fault is a leading-edge device fault the SFCC detects and locks out, shown as F/CTL SLATS LOCKED. Failures: [4027345, 4027346, 4027347, 4027348, 4027349, 4027350, 4027351, 4027352]")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(2.0).items(0, Vec::new()),
        proc(281800003, "FUEL APU FEED FAULT", Level::Caution, sd_page::FUEL, all(vec![var("FUEL_APU_FEED_PUMP_FAULT").eq(1.0), network_alive()]), "B11: APU feed pump degrades -> FUEL APU FEED FAULT. Failures: [19028088]")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(3.0).items(0, Vec::new()),
        proc(281800014, "FUEL CROSSFEED VLV 1 FAULT", Level::Caution, sd_page::FUEL, all(vec![var("FUEL_CROSSFEED_VALVE_FAULT:1").eq(1.0), network_alive()]), "B12: Cross-feed valve 1 stuck is the real FUEL CROSSFEED VLV 1 FAULT caution. Failures: [19028057]")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(3.0).items(0, Vec::new()),
        proc(281800015, "FUEL CROSSFEED VLV 2 FAULT", Level::Caution, sd_page::FUEL, all(vec![var("FUEL_CROSSFEED_VALVE_FAULT:2").eq(1.0), network_alive()]), "B12: Cross-feed valve 2 stuck is the real FUEL CROSSFEED VLV 2 FAULT caution. Failures: [19028058]")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(3.0).items(0, Vec::new()),
        proc(281800016, "FUEL CROSSFEED VLV 3 FAULT", Level::Caution, sd_page::FUEL, all(vec![var("FUEL_CROSSFEED_VALVE_FAULT:3").eq(1.0), network_alive()]), "B12: Cross-feed valve 3 stuck is the real FUEL CROSSFEED VLV 3 FAULT caution. Failures: [19028059]")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(3.0).items(0, Vec::new()),
        proc(281800017, "FUEL CROSSFEED VLV 4 FAULT", Level::Caution, sd_page::FUEL, all(vec![var("FUEL_CROSSFEED_VALVE_FAULT:4").eq(1.0), network_alive()]), "B12: Cross-feed valve 4 stuck is the real FUEL CROSSFEED VLV 4 FAULT caution. Failures: [19028060]")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(3.0).items(0, Vec::new()),
        proc(281800018, "FUEL ENG 1 LP VLV FAULT", Level::Caution, sd_page::FUEL, all(vec![var("FUEL_ENG_LP_VALVE_FAULT:1").eq(1.0), network_alive()]), "B12: Engine 1 LP fuel shutoff valve stuck is the real FUEL ENG 1 LP VLV FAULT caution. Failures: [19028090]")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(3.0).items(0, Vec::new()),
        proc(281800019, "FUEL ENG 2 LP VLV FAULT", Level::Caution, sd_page::FUEL, all(vec![var("FUEL_ENG_LP_VALVE_FAULT:2").eq(1.0), network_alive()]), "B12: Engine 2 LP fuel shutoff valve stuck is the real FUEL ENG 2 LP VLV FAULT caution. Failures: [19028091]")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(3.0).items(0, Vec::new()),
        proc(281800020, "FUEL ENG 3 LP VLV FAULT", Level::Caution, sd_page::FUEL, all(vec![var("FUEL_ENG_LP_VALVE_FAULT:3").eq(1.0), network_alive()]), "B12: Engine 3 LP fuel shutoff valve stuck is the real FUEL ENG 3 LP VLV FAULT caution. Failures: [19028092]")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(3.0).items(0, Vec::new()),
        proc(281800021, "FUEL ENG 4 LP VLV FAULT", Level::Caution, sd_page::FUEL, all(vec![var("FUEL_ENG_LP_VALVE_FAULT:4").eq(1.0), network_alive()]), "B12: Engine 4 LP fuel shutoff valve stuck is the real FUEL ENG 4 LP VLV FAULT caution. Failures: [19028093]")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(3.0).items(0, Vec::new()),
        proc(281800031, "FUEL FEED TK 1 MAIN PMP FAULT", Level::Caution, sd_page::FUEL, all(vec![var("FUEL_FEED_PUMP_FAULT:main_1").eq(1.0), network_alive()]), "B11: Feed tank 1 main pump degrades -> FUEL FEED TK 1 MAIN PMP FAULT. Failures: [19028094]")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(3.0).items(0, Vec::new()),
        proc(281800032, "FUEL FEED TK 2 MAIN PMP FAULT", Level::Caution, sd_page::FUEL, all(vec![var("FUEL_FEED_PUMP_FAULT:main_2").eq(1.0), network_alive()]), "B11: Feed tank 2 main pump degrades -> FUEL FEED TK 2 MAIN PMP FAULT. Failures: [19028096]")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(3.0).items(0, Vec::new()),
        proc(281800033, "FUEL FEED TK 3 MAIN PMP FAULT", Level::Caution, sd_page::FUEL, all(vec![var("FUEL_FEED_PUMP_FAULT:main_3").eq(1.0), network_alive()]), "B11: Feed tank 3 main pump degrades -> FUEL FEED TK 3 MAIN PMP FAULT. Failures: [19028098]")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(3.0).items(0, Vec::new()),
        proc(281800034, "FUEL FEED TK 4 MAIN PMP FAULT", Level::Caution, sd_page::FUEL, all(vec![var("FUEL_FEED_PUMP_FAULT:main_4").eq(1.0), network_alive()]), "B11: Feed tank 4 main pump degrades -> FUEL FEED TK 4 MAIN PMP FAULT. Failures: [19028100]")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(3.0).items(0, Vec::new()),
        proc(281800035, "FUEL FEED TK 1 STBY PMP FAULT", Level::Caution, sd_page::FUEL, all(vec![var("FUEL_FEED_PUMP_FAULT:stby_1").eq(1.0), network_alive()]), "B11: Feed tank 1 standby pump degrades -> FUEL FEED TK 1 STBY PMP FAULT. Failures: [19028095]")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(3.0).items(0, Vec::new()),
        proc(281800036, "FUEL FEED TK 2 STBY PMP FAULT", Level::Caution, sd_page::FUEL, all(vec![var("FUEL_FEED_PUMP_FAULT:stby_2").eq(1.0), network_alive()]), "B11: Feed tank 2 standby pump degrades -> FUEL FEED TK 2 STBY PMP FAULT. Failures: [19028097]")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(3.0).items(0, Vec::new()),
        proc(281800037, "FUEL FEED TK 3 STBY PMP FAULT", Level::Caution, sd_page::FUEL, all(vec![var("FUEL_FEED_PUMP_FAULT:stby_3").eq(1.0), network_alive()]), "B11: Feed tank 3 standby pump degrades -> FUEL FEED TK 3 STBY PMP FAULT. Failures: [19028099]")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(3.0).items(0, Vec::new()),
        proc(281800038, "FUEL FEED TK 4 STBY PMP FAULT", Level::Caution, sd_page::FUEL, all(vec![var("FUEL_FEED_PUMP_FAULT:stby_4").eq(1.0), network_alive()]), "B11: Feed tank 4 standby pump degrades -> FUEL FEED TK 4 STBY PMP FAULT. Failures: [19028101]")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(3.0).items(0, Vec::new()),
        proc(281800043, "FUEL FQDC 1 FAULT", Level::Caution, sd_page::FUEL, all(vec![var("FUEL_FQDC_FAULT:1").eq(1.0), network_alive()]), "B10: FQDC channel 1 fault sets FUEL_FQDC_FAULT:1, matching FUEL FQDC 1 FAULT. Failures: [19028113]")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(2.0).items(0, Vec::new()),
        proc(281800044, "FUEL FQDC 2 FAULT", Level::Caution, sd_page::FUEL, all(vec![var("FUEL_FQDC_FAULT:2").eq(1.0), network_alive()]), "B10: FQDC channel 2 fault sets FUEL_FQDC_FAULT:2, matching FUEL FQDC 2 FAULT. Failures: [19028114]")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(2.0).items(0, Vec::new()),
        proc(281800046, "FUEL FQMS 1 FAULT", Level::Caution, sd_page::FUEL, all(vec![any(vec![var("FUEL_FQMS_FAULT:1").eq(1.0), all(vec![var("AVNCS_MODULE_CPIOM_F1_POWERED").eq(1.0), var("AVNCS_MODULE_CPIOM_F1_PARTITION_FUEL_AVAILABLE").eq(0.0)])]), network_alive()]), "B10: FQMS channel 1 fault sets FUEL_FQMS_FAULT:1, matching FUEL FQMS 1 FAULT. Failures: [19028115]")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(2.0).items(0, Vec::new()),
        proc(281800047, "FUEL FQMS 2 FAULT", Level::Caution, sd_page::FUEL, all(vec![var("FUEL_FQMS_FAULT:2").eq(1.0), network_alive()]), "B10: FQMS channel 2 fault sets FUEL_FQMS_FAULT:2, matching FUEL FQMS 2 FAULT. Failures: [19028116]")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(2.0).items(0, Vec::new()),
        proc(281800049, "FUEL GAUGING FAULT", Level::Caution, sd_page::FUEL, all(vec![var("FUEL_FQMS_LOW_CONFIDENCE").eq(1.0), network_alive()]), "B11: A densitometer or probe failure drops FQMS gauging confidence below valid -> FUEL GAUGING FAULT. Failures: [19028012, 19028014, 19028015, 19028017, 19028018, 19028020, 19028021, 19028023]")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(3.0).items(0, Vec::new()),
        proc(281800058, "FUEL L INR TK AFT PMP FAULT", Level::Caution, sd_page::FUEL, all(vec![var("FUEL_WING_PUMP_FAULT:inner_aft_left").eq(1.0), network_alive()]), "B11: Left inner tank aft pump degrades -> FUEL L INR TK AFT PMP FAULT. Failures: [19028106]")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(3.0).items(0, Vec::new()),
        proc(281800059, "FUEL L MID TK AFT PMP FAULT", Level::Caution, sd_page::FUEL, all(vec![var("FUEL_WING_PUMP_FAULT:mid_aft_left").eq(1.0), network_alive()]), "B12: Left mid tank aft wing pump fault is the real FUEL L MID TK AFT PMP FAULT caution. Failures: [19028104]")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(3.0).items(0, Vec::new()),
        proc(281800060, "FUEL R INR TK AFT PMP FAULT", Level::Caution, sd_page::FUEL, all(vec![var("FUEL_WING_PUMP_FAULT:inner_aft_right").eq(1.0), network_alive()]), "B12: Right inner tank aft wing pump fault is the real FUEL R INR TK AFT PMP FAULT caution. Failures: [19028111]")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(3.0).items(0, Vec::new()),
        proc(281800061, "FUEL R MID TK AFT PMP FAULT", Level::Caution, sd_page::FUEL, all(vec![var("FUEL_WING_PUMP_FAULT:mid_aft_right").eq(1.0), network_alive()]), "B12: Right mid tank aft wing pump fault is the real FUEL R MID TK AFT PMP FAULT caution. Failures: [19028109]")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(3.0).items(0, Vec::new()),
        proc(281800062, "FUEL L MID TK FWD PMP FAULT", Level::Caution, sd_page::FUEL, all(vec![var("FUEL_WING_PUMP_FAULT:mid_fwd_left").eq(1.0), network_alive()]), "B12: Left mid tank forward wing pump fault is the real FUEL L MID TK FWD PMP FAULT caution. Failures: [19028103]")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(3.0).items(0, Vec::new()),
        proc(281800063, "FUEL L INR TK FWD PMP FAULT", Level::Caution, sd_page::FUEL, all(vec![var("FUEL_WING_PUMP_FAULT:inner_fwd_left").eq(1.0), network_alive()]), "B12: Left inner tank forward wing pump fault is the real FUEL L INR TK FWD PMP FAULT caution. Failures: [19028105]")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(3.0).items(0, Vec::new()),
        proc(281800064, "FUEL R MID TK FWD PMP FAULT", Level::Caution, sd_page::FUEL, all(vec![var("FUEL_WING_PUMP_FAULT:mid_fwd_right").eq(1.0), network_alive()]), "B12: Right mid tank forward wing pump fault is the real FUEL R MID TK FWD PMP FAULT caution. Failures: [19028108]")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(3.0).items(0, Vec::new()),
        proc(281800065, "FUEL R INR TK FWD PMP FAULT", Level::Caution, sd_page::FUEL, all(vec![var("FUEL_WING_PUMP_FAULT:inner_fwd_right").eq(1.0), network_alive()]), "B12: Right inner tank forward wing pump fault is the real FUEL R INR TK FWD PMP FAULT caution. Failures: [19028110]")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(3.0).items(0, Vec::new()),
        proc(281800068, "FUEL L OUTR TK PMP FAULT", Level::Caution, sd_page::FUEL, all(vec![var("FUEL_WING_PUMP_FAULT:outer_left").eq(1.0), network_alive()]), "B12: Left outer tank pump fault is the real FUEL L OUTR TK PMP FAULT caution. Failures: [19028102]")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(3.0).items(0, Vec::new()),
        proc(281800069, "FUEL R OUTR TK PMP FAULT", Level::Caution, sd_page::FUEL, all(vec![var("FUEL_WING_PUMP_FAULT:outer_right").eq(1.0), network_alive()]), "B12: Right outer tank pump fault is the real FUEL R OUTR TK PMP FAULT caution. Failures: [19028107]")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(3.0).items(0, Vec::new()),
        proc(281800072, "FUEL LEAK DET FAULT", Level::Caution, sd_page::FUEL, all(vec![var("FUEL_LEAK_DETECTOR_FAULT").eq(1.0), network_alive()]), "B11: Leak detection system BITE fault -> FUEL LEAK DET FAULT. Failures: [19028112]")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(3.0).items(0, Vec::new()),
        proc(281800073, "FUEL LEAK DETECTED", Level::Caution, sd_page::FUEL, all(vec![any(vec![var("FUEL_TANK_LEAK_KG_S:4").gt(0.05), var("FUEL_TANK_LEAK_KG_S:3").gt(0.05), var("FUEL_TANK_LEAK_KG_S:1").gt(0.05), var("FUEL_TANK_LEAK_KG_S:7").gt(0.05), var("FUEL_TANK_LEAK_KG_S:8").gt(0.05), var("FUEL_TANK_LEAK_KG_S:10").gt(0.05), var("FUEL_TANK_LEAK_KG_S:11").gt(0.05), var("FUEL_TANK_LEAK_KG_S:2").gt(0.05), var("FUEL_TANK_LEAK_KG_S:5").gt(0.05), var("FUEL_TANK_LEAK_KG_S:6").gt(0.05), var("FUEL_TANK_LEAK_KG_S:9").gt(0.05)]), network_alive()]), "B12, B12, B12, B12, B12, B12, B12: Left inner tank structural leak is a sustained fuel-quantity loss that trips FUEL LEAK DETECTED. Extended coverage-c: the four feed tanks (2, 5, 6, 9) use the same leak-rate threshold as the other seven tanks -- a feed tank's structural leak is the same sustained fuel-quantity loss FUEL LEAK DETECTED already catches. Failures: [19028077, 19028079, 19028080, 19028083, 19028084, 19028086, 19028087, 19028078, 19028081, 19028082, 19028085]")
            .inhibit(&[1, 3, 4, 5, 6, 7, 9, 10, 11, 12]).confirm(5.0).items(0, Vec::new()),
        proc(281800078, "FUEL NORM XFR FAULT", Level::Caution, sd_page::FUEL, all(vec![var("FUEL_TRANSFER_SEQUENCER_FAULT:norm").eq(1.0), network_alive()]), "B10: Normal transfer sequencer fault sets the norm flag, matching FUEL NORM XFR FAULT. Failures: [19028117]")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(2.0).items(0, Vec::new()),
        proc(281800085, "FUEL SYS COMPONENT FAULT", Level::Advisory, sd_page::FUEL, all(vec![any(vec![var("FUEL_FILTER_HEATER_FAULT:1").eq(1.0), var("FUEL_FILTER_HEATER_FAULT:2").eq(1.0), var("FUEL_FILTER_HEATER_FAULT:3").eq(1.0), var("FUEL_FILTER_HEATER_FAULT:4").eq(1.0)]), network_alive()]), "B11: Fuel filter anti-ice heater element fails -> generic BITE catch-all FUEL SYS COMPONENT FAULT. Failures: [19028066, 19028068, 19028070, 19028072]")
            .inhibit(&[3, 4, 5, 6, 7, 8, 9, 10, 11]).confirm(5.0).items(0, Vec::new()),
        proc(281800089, "FUEL TRIM TK L PMP FAULT", Level::Caution, sd_page::FUEL, all(vec![var("FUEL_TRIM_PUMP_DEGRADATION:1").eq(1.0), network_alive()]), "B12: Trim tank left pump degradation is the real FUEL TRIM TK L PMP FAULT caution. Failures: [19028045]")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(3.0).items(0, Vec::new()),
        proc(281800090, "FUEL TRIM TK R PMP FAULT", Level::Caution, sd_page::FUEL, all(vec![var("FUEL_TRIM_PUMP_DEGRADATION:2").eq(1.0), network_alive()]), "B12: Trim tank right pump degradation is the real FUEL TRIM TK R PMP FAULT caution. Failures: [19028046]")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(3.0).items(0, Vec::new()),
        proc(281800095, "FUEL WEIGHT & BALANCE BKUP FAULT", Level::Advisory, sd_page::FUEL, all(vec![var("FUEL_WB_BACKUP_FAULT").eq(1.0), network_alive()]), "B11: Backup W&B computation path fails -> FQMS BITE raises FUEL WEIGHT & BALANCE BKUP FAULT. Failures: [19028119]")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(3.0).items(0, Vec::new()),
        proc(290800013, "HYD G FUEL HEAT EXCHANGER VLV FAULT", Level::Caution, sd_page::HYD, all(vec![var("HYD_GREEN_FUEL_HX_VALVE_FAULT").eq(1.0), network_alive()]), "C17: Exact candidate match: HYD G FUEL HEAT EXCHANGER VLV FAULT is precisely the green fuel/hydraulic HX valve stuck failure. Failures: [3029033]")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(2.0).items(0, Vec::new()),
        proc(290800014, "HYD Y FUEL HEAT EXCHANGER VLV FAULT", Level::Caution, sd_page::HYD, all(vec![var("HYD_YELLOW_FUEL_HX_VALVE_FAULT").ne(0.0), network_alive()]), "C18: fuel/hydraulic heat exchanger valve stuck flag matches the FBW HYD Y FUEL HEAT EXCHANGER VLV FAULT procedure exactly Failures: [3029070]")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(2.0).items(0, Vec::new()),
        proc(290800015, "HYD G HEAT EXCHANGER AIR LEAK", Level::Caution, sd_page::HYD, all(vec![var("HYD_GREEN_FUEL_HX_AIR_LEAK").eq(1.0), network_alive()]), "C17: Exact candidate match: HYD G HEAT EXCHANGER AIR LEAK is precisely the green fuel/hydraulic heat-exchanger air-side leak. Failures: [3029034]")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(2.0).items(0, Vec::new()),
        proc(290800016, "HYD Y HEAT EXCHANGER AIR LEAK", Level::Caution, sd_page::HYD, all(vec![var("HYD_YELLOW_FUEL_HX_AIR_LEAK").ne(0.0), network_alive()]), "C18: air-side leak discrete flag matches the FBW HYD Y HEAT EXCHANGER AIR LEAK procedure exactly Failures: [3029071]")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(2.0).items(0, Vec::new()),
        proc(290800017, "HYD G HEAT EXCHANGER AIR LEAK DET FAULT", Level::Caution, sd_page::HYD, all(vec![var("HYD_GREEN_FUEL_HX_AIR_LEAK_DET_FAULT").eq(1.0), network_alive()]), "C17: Exact candidate match: HYD G HEAT EXCHANGER AIR LEAK DET FAULT is precisely the leak-switch circuit fault. Failures: [3029035]")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(2.0).items(0, Vec::new()),
        proc(290800018, "HYD Y HEAT EXCHANGER AIR LEAK DET FAULT", Level::Caution, sd_page::HYD, all(vec![var("HYD_YELLOW_FUEL_HX_AIR_LEAK_DET_FAULT").ne(0.0), network_alive()]), "C18: air-leak detector circuit fault flag matches the FBW HYD Y HEAT EXCHANGER AIR LEAK DET FAULT procedure exactly Failures: [3029072]")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(2.0).items(0, Vec::new()),
        proc(290800023, "HYD G SYS CHAN A OVHT DET FAULT", Level::Caution, sd_page::HYD, all(vec![var("HYD_GREEN_SYS_CHAN_A_OVHT_DET_FAULT").ne(0.0), network_alive()]), "C18: overheat-detection channel A fault flag matches the FBW HYD G SYS CHAN A OVHT DET FAULT procedure exactly Failures: [3029036]")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(2.0).items(0, Vec::new()),
        proc(290800024, "HYD G SYS CHAN B OVHT DET FAULT", Level::Caution, sd_page::HYD, all(vec![var("HYD_GREEN_SYS_CHAN_B_OVHT_DET_FAULT").ne(0.0), network_alive()]), "C18: overheat-detection channel B fault flag matches the FBW HYD G SYS CHAN B OVHT DET FAULT procedure exactly Failures: [3029037]")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(2.0).items(0, Vec::new()),
        proc(290800037, "HYD G SYS TEMP HI", Level::Caution, sd_page::HYD, all(vec![var("DEEP_HYD_GREEN_RESERVOIR_TEMP_SENSED_C").gt(150.0), network_alive()]), "C18: open-circuit sender pegs at 200C, above the real hydraulic-fluid high-temp limit (~135C) that HYD G SYS TEMP HI monitors Failures: [6029003]")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(5.0).items(5, Vec::new()),
        proc(290800038, "HYD Y SYS TEMP HI", Level::Caution, sd_page::HYD, all(vec![var("DEEP_HYD_YELLOW_RESERVOIR_TEMP_SENSED_C").gt(150.0), network_alive()]), "C18: open-circuit sender pegs at 200C, above the real hydraulic-fluid high-temp limit (~135C) that HYD Y SYS TEMP HI monitors Failures: [6029010]")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(5.0).items(5, Vec::new()),
        proc(311800003, "CDS F/O EFIS BKUP CTL FAULT", Level::Caution, -1, all(vec![var("ELEC_LOAD_fo-efis-bkup-ctl_CUT").ne(0.0), network_alive()]), "D01: fo-efis-bkup-ctl loses its supply through its own breaker or wiring: the FWS monitors that LRU and raises its fault Failures: [1024582, 1031017, 1031018, 1031020]")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(2.0).items(0, Vec::new()),
        proc(311800005, "CDS F/O EFIS CTL PNL FAULT", Level::Caution, -1, all(vec![var("ELEC_LOAD_fo-efis-ctl-panel_CUT").ne(0.0), network_alive()]), "D01: fo-efis-ctl-panel loses its supply through its own breaker or wiring: the FWS monitors that LRU and raises its fault Failures: [1024586, 1031025, 1031026, 1031028]")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(2.0).items(0, Vec::new()),
        proc(313800007, "CDS CAPT MAILBOX ACCESS FAULT", Level::Caution, -1, all(vec![var("ELEC_LOAD_cds-mailbox-capt_CUT").ne(0.0), network_alive()]), "D01: cds-mailbox-capt loses its supply through its own breaker or wiring: the FWS monitors that LRU and raises its fault Failures: [1024606, 1031057, 1031058, 1031060, 17031033]")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(2.0).items(0, Vec::new()),
        proc(316800001, "NAV HUD FAULT", Level::Caution, -1, all(vec![var("ELEC_LOAD_hud_CUT").ne(0.0), network_alive()]), "D01: hud loses its supply through its own breaker or wiring: the FWS monitors that LRU and raises its fault Failures: [1024608, 1031061, 1031062, 1031064]")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(2.0).items(0, Vec::new()),
        proc(319800001, "RECORDER ACCELMTR FAULT", Level::Caution, -1, all(vec![var("ELEC_LOAD_recorder-accelerometer_CUT").ne(0.0), network_alive()]), "D01: recorder-accelerometer loses its supply through its own breaker or wiring: the FWS monitors that LRU and raises its fault Failures: [1024612, 1031069, 1031070, 1031072, 17031039]")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(2.0).items(0, Vec::new()),
        proc(319800002, "RECORDER CVR FAULT", Level::Caution, -1, all(vec![var("ELEC_LOAD_cvr_CUT").ne(0.0), network_alive()]), "D01: cvr loses its supply through its own breaker or wiring: the FWS monitors that LRU and raises its fault Failures: [1024576, 1031005, 1031006, 1031008]")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(2.0).items(0, Vec::new()),
        proc(319800003, "RECORDER DFDR FAULT", Level::Caution, -1, all(vec![var("ELEC_LOAD_dfdr_CUT").ne(0.0), network_alive()]), "D01: dfdr loses its supply through its own breaker or wiring: the FWS monitors that LRU and raises its fault Failures: [1024574, 1031001, 1031002, 1031004]")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(2.0).items(0, Vec::new()),
        proc(319800004, "RECORDER SYS FAULT", Level::Caution, -1, all(vec![var("ELEC_LOAD_dfdau_CUT").ne(0.0), network_alive()]), "D01: dfdau loses its supply through its own breaker or wiring: the FWS monitors that LRU and raises its fault Failures: [1024614, 1031073, 1031074, 1031076]")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(2.0).items(0, Vec::new()),
        proc(320800012, "BRAKES ALTN BRK PRESS MONITORING FAULT", Level::Caution, sd_page::WHEEL, all(vec![var("ALTN_BRK_PRESS_SENSOR_FAULT").eq(1.0), network_alive()]), "B28: Alternate brake pressure sensor fault matches ALTN BRK PRESS MONITORING FAULT. Failures: [5032092]")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(2.0).items(0, Vec::new()),
        proc(320800013, "BRAKES AUTO BRK FAULT", Level::Caution, sd_page::WHEEL, all(vec![var("AUTO_BRK_FAULT").eq(1.0), network_alive()]), "B28: Autobrake function failure is exactly BRAKES AUTO BRK FAULT. Failures: [5032093]")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(2.0).items(0, Vec::new()),
        proc(320800015, "BRAKES CTL 1 FAULT", Level::Caution, sd_page::WHEEL, all(vec![var("BSCU_CHANNEL_FAULT:1").eq(1.0), network_alive()]), "B28: BSCU channel 1 fault is exactly BRAKES CTL 1 FAULT. Failures: [5032089]")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(2.0).items(0, Vec::new()),
        proc(320800016, "BRAKES CTL 2 FAULT", Level::Caution, sd_page::WHEEL, all(vec![var("BSCU_CHANNEL_FAULT:2").eq(1.0), network_alive()]), "B28: BSCU channel 2 fault is exactly BRAKES CTL 2 FAULT. Failures: [5032090]")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(2.0).items(0, Vec::new()),
        proc(320800021, "BRAKES NORM BRK PRESS MONITORING FAULT", Level::Caution, sd_page::WHEEL, all(vec![var("NORM_BRK_PRESS_SENSOR_FAULT").eq(1.0), network_alive()]), "B28: Normal brake pressure sensor fault matches NORM BRK PRESS MONITORING FAULT. Failures: [5032091]")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(2.0).items(0, Vec::new()),
        proc(320800024, "BRAKES PEDAL BRAKING FAULT", Level::Caution, sd_page::WHEEL, all(vec![any(vec![var("BRAKE_PEDAL_SENSOR_FAULT:1").ne(0.0), var("BRAKE_PEDAL_SENSOR_FAULT:2").ne(0.0)]), network_alive()]), "C21: A failed pedal transducer is a lost pedal-braking input channel, which is exactly what FBW's pedal braking fault procedure covers. Failures: [5032095, 5032096]")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(2.0).items(2, Vec::new()),
        proc(320800027, "BRAKES SEL VLV JAMMED OPEN", Level::Caution, sd_page::WHEEL, all(vec![var("BRAKE_SEL_VLV_JAMMED").eq(1.0), network_alive()]), "B28: Brake selector valve jammed open matches BRAKES SEL VLV JAMMED OPEN. Failures: [5032094]")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(2.0).items(0, Vec::new()),
        proc(320800043, "L/G OLEO PRESS MONITORING FAULT", Level::Caution, sd_page::WHEEL, all(vec![any(vec![var("GEAR_STRUT_PRESS_SENSOR_FAULT:4").eq(1.0), var("GEAR_STRUT_PRESS_SENSOR_FAULT:2").eq(1.0), var("GEAR_STRUT_PRESS_SENSOR_FAULT:1").eq(1.0), var("GEAR_STRUT_PRESS_SENSOR_FAULT:5").eq(1.0), var("GEAR_STRUT_PRESS_SENSOR_FAULT:3").eq(1.0)]), network_alive()]), "B28, B28, B28, B28, B28: Strut pressure sensor failure matches L/G OLEO PRESS MONITORING FAULT. Failures: [5032003, 5032007, 5032011, 5032015, 5032019]")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(2.0).items(0, Vec::new()),
        proc(320800046, "L/G WEIGHT ON WHEELS FAULT", Level::Caution, sd_page::WHEEL, all(vec![any(vec![all(vec![var("GEAR_LEG_COMPRESSION:1").gt(0.3), var("DEEP_PROX_GEAR_NOSE_WOW_NEAR").eq(0.0)]), all(vec![var("GEAR_LEG_COMPRESSION:1").lt(0.05), var("DEEP_PROX_GEAR_NOSE_WOW_NEAR").eq(1.0)]), all(vec![var("GEAR_LEG_COMPRESSION:2").gt(0.3), var("DEEP_PROX_GEAR_LEFT_WING_WOW_NEAR").eq(0.0)]), all(vec![var("GEAR_LEG_COMPRESSION:2").lt(0.05), var("DEEP_PROX_GEAR_LEFT_WING_WOW_NEAR").eq(1.0)]), all(vec![var("GEAR_LEG_COMPRESSION:3").gt(0.3), var("DEEP_PROX_GEAR_RIGHT_WING_WOW_NEAR").eq(0.0)]), all(vec![var("GEAR_LEG_COMPRESSION:3").lt(0.05), var("DEEP_PROX_GEAR_RIGHT_WING_WOW_NEAR").eq(1.0)]), all(vec![var("GEAR_LEG_COMPRESSION:4").gt(0.3), var("DEEP_PROX_GEAR_LEFT_BODY_WOW_NEAR").eq(0.0)]), all(vec![var("GEAR_LEG_COMPRESSION:4").lt(0.05), var("DEEP_PROX_GEAR_LEFT_BODY_WOW_NEAR").eq(1.0)]), all(vec![var("GEAR_LEG_COMPRESSION:5").gt(0.3), var("DEEP_PROX_GEAR_RIGHT_BODY_WOW_NEAR").eq(0.0)]), all(vec![var("GEAR_LEG_COMPRESSION:5").lt(0.05), var("DEEP_PROX_GEAR_RIGHT_BODY_WOW_NEAR").eq(1.0)])]), network_alive()]), "M00: corrected trigger (2 Oct): the alert as the aircraft raises it Failures: [6032007, 6032009, 6032016, 6032018, 6032025, 6032027, 6032034, 6032036]")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(3.0).items(0, Vec::new()),
        proc(320800049, "STEER B/W STEER FAULT", Level::Caution, sd_page::WHEEL, all(vec![any(vec![var("BODY_STEER_FAULT:1").eq(1.0), var("BODY_STEER_FAULT:2").eq(1.0)]), network_alive()]), "B28, B28: Left body rear-axle steering leak sets BODY_STEER_FAULT, matching STEER B/W STEER FAULT. Failures: [5032086, 5032088]")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(2.0).items(1, Vec::new()),
        proc(320800051, "STEER CAPT STEER TILLER FAULT", Level::Caution, sd_page::WHEEL, all(vec![var("STEER_TILLER_FAULT:capt").eq(1.0), network_alive()]), "B28: Captain's tiller transducer failure matches STEER CAPT STEER TILLER FAULT. Failures: [5032100]")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(2.0).items(0, Vec::new()),
        proc(320800052, "STEER FO STEER TILLER FAULT", Level::Caution, sd_page::WHEEL, all(vec![var("STEER_TILLER_FAULT:fo").eq(1.0), network_alive()]), "B28: F/O's tiller transducer failure matches STEER FO STEER TILLER FAULT. Failures: [5032101]")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(2.0).items(0, Vec::new()),
        proc(320800053, "STEER CTL 1 FAULT", Level::Caution, sd_page::WHEEL, all(vec![var("STEER_CTL_FAULT:1").eq(1.0), network_alive()]), "B28: Steering control channel 1 failure matches STEER CTL 1 FAULT. Failures: [5032097]")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(2.0).items(0, Vec::new()),
        proc(320800054, "STEER CTL 2 FAULT", Level::Caution, sd_page::WHEEL, all(vec![var("STEER_CTL_FAULT:2").eq(1.0), network_alive()]), "B28: Steering control channel 2 failure matches STEER CTL 2 FAULT. Failures: [5032098]")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(2.0).items(0, Vec::new()),
        proc(320800061, "STEER PEDAL STEER CTL FAULT", Level::Caution, sd_page::WHEEL, all(vec![var("STEER_PEDAL_FAULT").eq(1.0), network_alive()]), "B28: Pedal steering transducer failure matches STEER PEDAL STEER CTL FAULT. Failures: [5032102]")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(2.0).items(0, Vec::new()),
        proc(320800062, "STEER SEL VLV JAMMED OPEN", Level::Caution, sd_page::WHEEL, all(vec![var("STEER_SEL_VLV_JAMMED").eq(1.0), network_alive()]), "B28: Steering selector valve jammed open matches STEER SEL VLV JAMMED OPEN. Failures: [5032099]")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(2.0).items(3, Vec::new()),
        proc(340800007, "NAV ADR 1+2+3 DATA DEGRADED", Level::Caution, -1, all(vec![var("DEEP_PITOT_1_BLOCKED").eq(1.0), network_alive()]), "C26: a blocked or damaged pitot 1 puts ADR 1 out of agreement with the other two, the same degraded-data case the real aircraft flags. Failures: [6034002, 6034003]")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(3.0).items(2, Vec::new()),
        proc(340800009, "NAV AIR DATA DISAGREE", Level::Caution, -1, all(vec![var("ENV_ASH_PITOT_BLOCKED").eq(1.0), network_alive()]), "C26: volcanic ash blocks one pitot line, giving the single-source air data disagree the real aircraft flags. Failures: [14034002]")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(3.0).items(0, Vec::new()),
        proc(340800010, "NAV ALL AIR DATA DISAGREE", Level::Caution, -1, all(vec![any(vec![any(vec![var("ENV_BIRD_PROBE_BLOCKED:1").eq(1.0), var("ENV_BIRD_PROBE_BLOCKED:2").eq(1.0), var("ENV_BIRD_PROBE_BLOCKED:3").eq(1.0), var("ENV_BIRD_PROBE_BLOCKED:4").eq(1.0), var("ENV_BIRD_PROBE_BLOCKED:5").eq(1.0), var("ENV_BIRD_PROBE_BLOCKED:6").eq(1.0)]), any(vec![var("ENV_HAIL_PROBE_DAMAGE:1").eq(1.0), var("ENV_HAIL_PROBE_DAMAGE:2").eq(1.0), var("ENV_HAIL_PROBE_DAMAGE:3").eq(1.0), var("ENV_HAIL_PROBE_DAMAGE:4").eq(1.0), var("ENV_HAIL_PROBE_DAMAGE:5").eq(1.0), var("ENV_HAIL_PROBE_DAMAGE:6").eq(1.0)])]), network_alive()]), "C26, C26: a bird strike blocks every air data probe at once, the same loss of agreement the real aircraft flags across all three ADRs. Failures: [14034001, 14034004]")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(3.0).items(0, Vec::new()),
        proc(340800026, "NAV FM / IR POS DISAGREE", Level::Caution, -1, all(vec![any(vec![var("DEEP_IR_1_GYRO_DRIFT_DEG_HR").gt(50.0), var("DEEP_IR_2_GYRO_DRIFT_DEG_HR").gt(50.0), var("DEEP_IR_3_GYRO_DRIFT_DEG_HR").gt(50.0)]), network_alive()]), "B31, B31, B31: IR1 gyro drift accumulates into an inertial position error the FM/IR comparator detects. Failures: [6034098, 6034099, 6034100]")
            .inhibit(&[1, 2, 3, 4, 5, 6, 7, 9, 10, 11, 12]).confirm(30.0).items(0, Vec::new()),
        proc(340800033, "NAV GNSS SIGNAL DEGRADED", Level::Advisory, -1, all(vec![var("DEEP_GPS_3_VALID").eq(0.0), network_alive()]), "B31: GPS3 has no dedicated per-channel fault alert in this section; loss of its valid flag is a genuine GNSS capability degradation. Failures: [6034077, 6034078, 6034080, 6034081]")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(5.0).items(0, Vec::new()),
        proc(340800034, "NAV GPS 1 FAULT", Level::Caution, -1, all(vec![var("DEEP_GPS_1_VALID").eq(0.0), network_alive()]), "B31: GPS1 receiver/antenna fault or jamming drops the channel valid flag, same as a GPS1 receiver failure. Failures: [6034067, 6034068, 6034070, 6034071]")
            .inhibit(&[4, 5, 6, 7, 10]).confirm(5.0).items(0, Vec::new()),
        proc(340800050, "NAV OAT PROBE 1 FAULT", Level::Advisory, -1, all(vec![var("DEEP_OAT_1_HEATER_FAILED").eq(1.0), network_alive()]), "B31: OAT probe 1 heater failure is exactly the monitored OAT PROBE 1 FAULT condition. Failures: [6034088]")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(5.0).items(0, Vec::new()),
        proc(340800051, "NAV OAT PROBE 2 FAULT", Level::Advisory, -1, all(vec![var("DEEP_OAT_2_HEATER_FAILED").eq(1.0), network_alive()]), "B31: OAT probe 2 heater failure is exactly the monitored OAT PROBE 2 FAULT condition. Failures: [6034089]")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(5.0).items(0, Vec::new()),
        proc(340800064, "NAV SIDESLIP PROBE 1 FAULT", Level::Caution, -1, all(vec![any(vec![var("DEEP_SIDESLIP_1_HEATER_FAILED").eq(1.0), var("DEEP_SIDESLIP_1_JAMMED").eq(1.0)]), network_alive()]), "B32: sideslip vane 1 heater failure or jam is exactly NAV SIDESLIP PROBE 1 FAULT Failures: [6034090, 6034091]")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(2.0).items(0, Vec::new()),
        proc(340800065, "NAV SIDESLIP PROBE 2 FAULT", Level::Caution, -1, all(vec![any(vec![var("DEEP_SIDESLIP_2_HEATER_FAILED").eq(1.0), var("DEEP_SIDESLIP_2_JAMMED").eq(1.0)]), network_alive()]), "B32: sideslip vane 2 heater failure or jam is exactly NAV SIDESLIP PROBE 2 FAULT Failures: [6034092, 6034093]")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(2.0).items(0, Vec::new()),
        proc(340800066, "NAV SIDESLIP PROBE 3 FAULT", Level::Caution, -1, all(vec![any(vec![var("DEEP_SIDESLIP_3_HEATER_FAILED").eq(1.0), var("DEEP_SIDESLIP_3_JAMMED").eq(1.0)]), network_alive()]), "B32: sideslip vane 3 heater failure or jam is exactly NAV SIDESLIP PROBE 3 FAULT Failures: [6034094, 6034095]")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(2.0).items(0, Vec::new()),
        proc(340800068, "NAV TAT PROBE 1 FAULT", Level::Caution, -1, all(vec![any(vec![var("DEEP_TAT_1_HEATER_FAILED").eq(1.0), var("DEEP_ADR_1_SAT_ERROR_C").lt(-5.0)]), network_alive()]), "B32: TAT 1 heater loss or recovery-factor degradation both corrupt ADR 1's SAT enough to flag NAV TAT PROBE 1 FAULT Failures: [6034045, 6034046]")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(3.0).items(0, Vec::new()),
        proc(340800069, "NAV TAT PROBE 2 FAULT", Level::Caution, -1, all(vec![any(vec![var("DEEP_TAT_2_HEATER_FAILED").eq(1.0), var("DEEP_ADR_2_SAT_ERROR_C").lt(-5.0)]), network_alive()]), "B32: TAT 2 heater loss or recovery-factor degradation both corrupt ADR 2's SAT enough to flag NAV TAT PROBE 2 FAULT Failures: [6034047, 6034048]")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(3.0).items(0, Vec::new()),
        proc(340800070, "NAV TAT PROBE 3 FAULT", Level::Caution, -1, all(vec![any(vec![var("DEEP_TAT_3_HEATER_FAILED").eq(1.0), var("DEEP_TAT_3_RECOVERY_DEGRADED").eq(1.0)]), network_alive()]), "B32: TAT 3 heater loss or recovery-factor degradation both flag NAV TAT PROBE 3 FAULT Failures: [6034096, 6034097]")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(3.0).items(0, Vec::new()),
        proc(340800071, "NAV UNRELIABLE AIR SPEED INDICATION", Level::Caution, -1, all(vec![any(vec![var("DEEP_ADR_1_CAS_ERROR_MS").gt(160.0), var("DEEP_ADR_2_CAS_ERROR_MS").gt(160.0), var("DEEP_ADR_3_CAS_ERROR_MS").gt(160.0)]), network_alive()]), "B31, B31, B31: ADR1 drain-hole blockage biases speed without tripping the vote outlier flag, matching an unreliable-airspeed condition. Failures: [6034004, 6034008, 6034012]")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(5.0).items(0, Vec::new()),
        proc(520800027, "DOOR UPPER 1L NOT CLOSED", Level::Warning, sd_page::DOOR, all(vec![any(vec![var("CABIN_DOOR_UPPER_1L_OPEN_PERCENT").gt(0.0), var("DEEP_PROX_DOOR_U1L_CLOSED_NEAR").eq(0.0), var("CABIN_DOOR_UPPER_1L_LATCH_SENSOR_FAULT").eq(1.0)]), network_alive()]), "M00: corrected trigger (2 Oct): the alert as the aircraft raises it Failures: [6052034, 6052036, 10052006]")
            .inhibit(&[]).confirm(2.0).items(6, Vec::new()),
        proc(520800028, "DOOR UPPER 1R NOT CLOSED", Level::Warning, sd_page::DOOR, all(vec![var("CABIN_DOOR_UPPER_1R_LATCH_SENSOR_FAULT").eq(1.0), network_alive()]), "B37: Latch sensor fault is exactly what the real DOOR UPPER 1R NOT CLOSED check monitors. Failures: [10052007]")
            .inhibit(&[]).confirm(2.0).items(6, Vec::new()),
        proc(520800029, "DOOR UPPER 2L NOT CLOSED", Level::Warning, sd_page::DOOR, all(vec![var("CABIN_DOOR_UPPER_2L_LATCH_SENSOR_FAULT").eq(1.0), network_alive()]), "B37: Latch sensor fault is exactly what the real DOOR UPPER 2L NOT CLOSED check monitors. Failures: [10052008]")
            .inhibit(&[]).confirm(2.0).items(6, Vec::new()),
        proc(520800030, "DOOR UPPER 2R NOT CLOSED", Level::Warning, sd_page::DOOR, all(vec![var("CABIN_DOOR_UPPER_2R_LATCH_SENSOR_FAULT").eq(1.0), network_alive()]), "B37: Latch sensor fault is exactly what the real DOOR UPPER 2R NOT CLOSED check monitors. Failures: [10052009]")
            .inhibit(&[]).confirm(2.0).items(6, Vec::new()),
        proc(520800031, "DOOR UPPER 3L NOT CLOSED", Level::Warning, sd_page::DOOR, all(vec![var("CABIN_DOOR_UPPER_3L_LATCH_SENSOR_FAULT").eq(1.0), network_alive()]), "B37: Latch sensor fault is exactly what the real DOOR UPPER 3L NOT CLOSED check monitors. Failures: [10052010]")
            .inhibit(&[]).confirm(2.0).items(6, Vec::new()),
        proc(520800032, "DOOR UPPER 3R NOT CLOSED", Level::Warning, sd_page::DOOR, all(vec![var("CABIN_DOOR_UPPER_3R_LATCH_SENSOR_FAULT").eq(1.0), network_alive()]), "B37: Latch sensor fault is exactly what the real DOOR UPPER 3R NOT CLOSED check monitors. Failures: [10052011]")
            .inhibit(&[]).confirm(2.0).items(6, Vec::new()),
        proc(701800001, "ENG 1 CTL SYS FAULT", Level::Caution, sd_page::ENG, all(vec![all(vec![var("DEEP_ENG_1_N3_PICKUP_A_FRAC").gt(0.5), any(vec![var("DEEP_ENG_1_T25_SENSED_C").ge(499.5), var("DEEP_ENG_1_T25_SENSED_C").le(-69.5)])]), network_alive()]), "M00: corrected trigger (2 Oct): the alert as the aircraft raises it Failures: [6077083, 6077084]")
            .inhibit(&[5, 6]).confirm(3.0).items(0, Vec::new()),
        proc(701800002, "ENG 2 CTL SYS FAULT", Level::Caution, sd_page::ENG, all(vec![all(vec![var("DEEP_ENG_2_N3_PICKUP_A_FRAC").gt(0.5), any(vec![var("DEEP_ENG_2_T25_SENSED_C").ge(499.5), var("DEEP_ENG_2_T25_SENSED_C").le(-69.5)])]), network_alive()]), "M00: corrected trigger (2 Oct): the alert as the aircraft raises it Failures: [6077087, 6077088]")
            .inhibit(&[5, 6]).confirm(3.0).items(0, Vec::new()),
        proc(701800003, "ENG 3 CTL SYS FAULT", Level::Caution, sd_page::ENG, all(vec![all(vec![var("DEEP_ENG_3_N3_PICKUP_A_FRAC").gt(0.5), any(vec![var("DEEP_ENG_3_T25_SENSED_C").ge(499.5), var("DEEP_ENG_3_T25_SENSED_C").le(-69.5)])]), network_alive()]), "M00: corrected trigger (2 Oct): the alert as the aircraft raises it Failures: [6077091, 6077092]")
            .inhibit(&[5, 6]).confirm(3.0).items(0, Vec::new()),
        proc(701800004, "ENG 4 CTL SYS FAULT", Level::Caution, sd_page::ENG, all(vec![all(vec![var("DEEP_ENG_4_N3_PICKUP_A_FRAC").gt(0.5), any(vec![var("DEEP_ENG_4_T25_SENSED_C").ge(499.5), var("DEEP_ENG_4_T25_SENSED_C").le(-69.5)])]), network_alive()]), "M00: corrected trigger (2 Oct): the alert as the aircraft raises it Failures: [6077095, 6077096]")
            .inhibit(&[5, 6]).confirm(3.0).items(0, Vec::new()),
        proc(701800013, "ENG 1 FADEC FAULT", Level::Caution, sd_page::ENG, all(vec![any(vec![var("DEEP_ENG_1_VIB_FAN_VALID").eq(0.0), var("A32NX_ENG_1_EEC_NO_VALID_CHANNEL").eq(1.0), var("ENV_LTG_BUS_UPSET:EngineFadec").on()]), network_alive()]), "C42: Fan vibration pickup dropout is a FADEC-detected sensor validity loss on a discrete valid/invalid flag, not a continuous reading that varies in normal flight; OR'd with the deep registry's own dual-EEC-channel-dead discrete and a lightning-induced FADEC bus upset, both of which are the same real loss of FADEC control the FCOM procedure describes. Failures: [6077059, 2073500, 2073501]")
            .inhibit(&[4, 5, 6, 7, 9, 10]).confirm(2.0).items(0, Vec::new()),
        proc(701800014, "ENG 2 FADEC FAULT", Level::Caution, sd_page::ENG, all(vec![any(vec![var("DEEP_ENG_2_VIB_FAN_VALID").eq(0.0), var("A32NX_ENG_2_EEC_NO_VALID_CHANNEL").eq(1.0), var("ENV_LTG_BUS_UPSET:EngineFadec").on()]), network_alive()]), "C42: Fan vibration pickup dropout is a FADEC-detected sensor validity loss on a discrete valid/invalid flag; OR'd with the deep registry's own dual-EEC-channel-dead discrete and a lightning-induced FADEC bus upset. Failures: [6077065, 2073513, 2073514]")
            .inhibit(&[4, 5, 6, 7, 9, 10]).confirm(2.0).items(0, Vec::new()),
        proc(701800015, "ENG 3 FADEC FAULT", Level::Caution, sd_page::ENG, all(vec![any(vec![var("DEEP_ENG_3_VIB_FAN_VALID").eq(0.0), var("A32NX_ENG_3_EEC_NO_VALID_CHANNEL").eq(1.0), var("ENV_LTG_BUS_UPSET:EngineFadec").on()]), network_alive()]), "C42: Fan vibration pickup dropout is a FADEC-detected sensor validity loss on a discrete valid/invalid flag; OR'd with the deep registry's own dual-EEC-channel-dead discrete and a lightning-induced FADEC bus upset. Failures: [6077071, 2073526, 2073527]")
            .inhibit(&[4, 5, 6, 7, 9, 10]).confirm(2.0).items(0, Vec::new()),
        proc(701800033, "ENG 1 FUEL FILTER CLOGGED", Level::Advisory, sd_page::ENG, all(vec![var("A32NX_ENG_1_FUEL_FILTER_IMPENDING_BYPASS").eq(1.0), network_alive()]), "C35: Impending-bypass flag is the filter's own clog-detection discrete, exact match to the real FUEL FILTER CLOGGED ECAM procedure. Failures: [2073004]")
            .inhibit(&[3, 4, 5, 6, 7, 8, 9, 10]).confirm(3.0).items(0, Vec::new()),
        proc(701800034, "ENG 2 FUEL FILTER CLOGGED", Level::Advisory, sd_page::ENG, all(vec![var("A32NX_ENG_2_FUEL_FILTER_IMPENDING_BYPASS").eq(1.0), network_alive()]), "C35: Same clog-detection discrete as engine 1, engine 2 filter. Failures: [2073027]")
            .inhibit(&[3, 4, 5, 6, 7, 8, 9, 10]).confirm(3.0).items(0, Vec::new()),
        proc(701800035, "ENG 3 FUEL FILTER CLOGGED", Level::Advisory, sd_page::ENG, all(vec![var("A32NX_ENG_3_FUEL_FILTER_IMPENDING_BYPASS").eq(1.0), network_alive()]), "C35: Same clog-detection discrete, engine 3 filter. Failures: [2073050]")
            .inhibit(&[3, 4, 5, 6, 7, 8, 9, 10]).confirm(3.0).items(0, Vec::new()),
        proc(701800036, "ENG 4 FUEL FILTER CLOGGED", Level::Advisory, sd_page::ENG, all(vec![var("A32NX_ENG_4_FUEL_FILTER_IMPENDING_BYPASS").eq(1.0), network_alive()]), "C35: Same clog-detection discrete, engine 4 filter. Failures: [2073073]")
            .inhibit(&[3, 4, 5, 6, 7, 8, 9, 10]).confirm(3.0).items(0, Vec::new()),
        proc(701800105, "ENG 1 SENSOR FAULT", Level::Advisory, sd_page::ENG, all(vec![var("A32NX_ENG_1_EEC_SENSOR_DISAGREE").eq(1.0), network_alive()]), "C34: Every N1/N2/N3/TGT/P30 channel A or B sensor fault on engine 1 raises the same EEC sensor-disagree flag, which is the real aircraft's single generic sensor-fault message. Failures: [2073502, 2073503, 2073504, 2073505, 2073506, 2073507,")
            .inhibit(&[2, 3, 4, 5, 6, 7, 8, 9, 10]).confirm(3.0).items(0, Vec::new()),
        proc(701800106, "ENG 2 SENSOR FAULT", Level::Advisory, sd_page::ENG, all(vec![any(vec![var("A32NX_ENG_2_EEC_SENSOR_DISAGREE").eq(1.0), var("DEEP_FF_XMTR_2_FAULT").eq(1.0)]), network_alive()]), "C34: Same generic sensor-disagree message as engine 1, mirrored for engine 2's channel A/B sensor faults; OR'd with the deep sensors module's own fuel-flow transmitter fault discrete, the same generic probe-fault message the FCOM describes. Failures: [2073515, 2073516, 2073517, 2073518, 2073519, 2073520, 2073521, 2073522]")
            .inhibit(&[2, 3, 4, 5, 6, 7, 8, 9, 10]).confirm(3.0).items(0, Vec::new()),
        proc(701800121, "ENG 1 START VLV FAULT (NOT CLOSED)", Level::Caution, sd_page::ENG, any(vec![all(vec![var("A32NX_ENG_1_STARTER_DISENGAGE_FAULT").ne(0.0), network_alive()]), all(vec![var("A32NX_ENG_1_START_VALVE_DISAGREE").on(), var("A32NX_ENG_1_START_VALVE_POSITION").gt(0.5)])]), "C46: Clutch fails to disengage, so the starter keeps being driven (torque reverses, rotor overspeeds) exactly as when the start valve fails to close. Failures: [2080003]; or the start valve itself is stuck open")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(2.0).items(0, Vec::new()),
        proc(701800122, "ENG 2 START VLV FAULT (NOT CLOSED)", Level::Caution, sd_page::ENG, any(vec![all(vec![var("A32NX_ENG_2_STARTER_DISENGAGE_FAULT").ne(0.0), network_alive()]), all(vec![var("A32NX_ENG_2_START_VALVE_DISAGREE").on(), var("A32NX_ENG_2_START_VALVE_POSITION").gt(0.5)])]), "C46: Clutch fails to disengage on engine 2, same symptom (starter continues to motor the shaft) as the start valve not closing. Failures: [2080006]; or the start valve itself is stuck open")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(2.0).items(0, Vec::new()),
        proc(701800123, "ENG 3 START VLV FAULT (NOT CLOSED)", Level::Caution, sd_page::ENG, any(vec![all(vec![var("A32NX_ENG_3_STARTER_DISENGAGE_FAULT").ne(0.0), network_alive()]), all(vec![var("A32NX_ENG_3_START_VALVE_DISAGREE").on(), var("A32NX_ENG_3_START_VALVE_POSITION").gt(0.5)])]), "C46: Clutch fails to disengage on engine 3, same symptom as the start valve not closing. Failures: [2080009]; or the start valve itself is stuck open")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(2.0).items(0, Vec::new()),
        proc(701800124, "ENG 4 START VLV FAULT (NOT CLOSED)", Level::Caution, sd_page::ENG, any(vec![all(vec![var("A32NX_ENG_4_STARTER_DISENGAGE_FAULT").ne(0.0), network_alive()]), all(vec![var("A32NX_ENG_4_START_VALVE_DISAGREE").on(), var("A32NX_ENG_4_START_VALVE_POSITION").gt(0.5)])]), "C46: Clutch fails to disengage on engine 4, same symptom as the start valve not closing. Failures: [2080012]; or the start valve itself is stuck open")
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]).confirm(2.0).items(0, Vec::new()),
    ]
}
