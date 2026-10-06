//! ATA 24 -- electrical. FlyByWire defines 86 abnormal-sensed procedures in
//! `AbnormalSensed/ata24.ts` and wires **none** of them: `grep -c '^    24'
//! FwsAbnormalSensed.ts` returns 0. Every ELEC procedure on this aircraft,
//! from a single bus fault to the thirty-line EMER CONFIG checklist, is text
//! nothing can raise.
//!
//! What this port can raise them from is `deep::electrical`'s own published
//! bus state: `ELEC_<bus>_BUS_IS_POWERED` and `ELEC_<bus>_BUS_POTENTIAL` for
//! every AC and DC bus, plus per-unit `ELEC_GEN_n_FAULT`,
//! `ELEC_TR_{1,2,ESS,APU}_FAULT` and `ELEC_BAT_n_FAULT`.
//!
//! # The "network alive" gate
//!
//! An unpowered bus is only a *fault* when the aircraft has power to give
//! it. On the real aircraft that is implicit -- an FWS with no electrical
//! power annunciates nothing -- but this port's JS runs whatever the
//! aircraft's state, so a bare `ELEC_AC_2_BUS_IS_POWERED == 0` would light
//! every bus-fault procedure on a cold and dark aeroplane. [`network_alive`]
//! makes that condition explicit from the same published variables: at least
//! one of the four main AC buses is live, so AC generation exists and a
//! dead bus is the bus's own problem. This is not a gate invented to make an
//! alert behave -- it is the condition the real FWS's own power supply
//! imposes, written down.
//!
//! # What is deliberately left unwired
//!
//! * `240800002` AC BUS 1 FAULT, `240800017` BAT 1 (ESS) FAULT, `240800061`
//!   GEN 1 FAULT, `240800081` TR 1 FAULT and `240800055` EMER CONFIG:
//!   `deep::electrical`'s own `registry.rs` already registers an alert with
//!   the same title and the same trigger variable (`ELEC AC BUS 1 FAULT`,
//!   `ELEC BAT 1 FAULT`, `ELEC GEN 1 FAULT`, `ELEC TR 1 FAULT`, `ELEC EMER
//!   CONFIG`). Wiring FlyByWire's id as well would put the same warning on
//!   the EWD twice.
//! * `240800003` AC BUS 1+2 & DC BUS 1 FAULT: a combined procedure that
//!   includes AC BUS 1, which our own single-bus alert also announces and
//!   which nothing here can suppress (`notActiveWhenItemActive` only reaches
//!   FlyByWire's own dict, not `deep::api`'s alerts).
//!
//! # E-ELEC Phase 2 (2026-09-27)
//!
//! Everything else `E:/fbw-debug/ecam/UNWIRED.md` listed for this chapter is
//! now wired below, against `E:/fbw-debug/ecam/E-ELEC-FCOM.json`'s FCOM
//! mapping and `E-ELEC-DESIGN.md`'s design: the per-channel APU generator
//! faults (`240800014`/`015`, already published), the generator pushbuttons
//! (`240800065`-`068`, already published), and new `deep::electrical`/
//! `deep::breakers` components for the generator-drive oil/disconnect
//! system, the ENMU/ELMU/PSC/SSC computer-health verdicts, the TR/C/B
//! monitoring channels, the external-power and F/CTL-actuator supplies, the
//! cabin supply-centre overheat detectors, the RAT's own standing health
//! verdict, and the bus-tie/remote-C/B-control switch positions.

use super::{item, phase, proc, sd_page, FbwProc};
use crate::deep::api::{all, any, var, Cond, Level};

/// At least one main AC bus is live, so the electrical network is running
/// and an unpowered bus is that bus's own fault rather than a cold
/// aeroplane. See this module's doc comment.
fn network_alive() -> Cond {
    any(vec![
        var("ELEC_AC_1_BUS_IS_POWERED").on(),
        var("ELEC_AC_2_BUS_IS_POWERED").on(),
        var("ELEC_AC_3_BUS_IS_POWERED").on(),
        var("ELEC_AC_4_BUS_IS_POWERED").on(),
    ])
}

/// One AC bus dead: below the same 90 V of 115 V nominal that
/// `deep::electrical::registry`'s own `ELEC AC BUS 1 FAULT` uses, and the
/// electrical model's own verdict that it is unpowered. Both, so a bus held
/// up at a degraded voltage by a failing source is not called dead, and a
/// model that reports "powered" at a voltage no equipment could use is not
/// believed.
fn ac_dead(tag: &str) -> Cond {
    all(vec![var(&format!("ELEC_{tag}_BUS_POTENTIAL")).lt(90.0), var(&format!("ELEC_{tag}_BUS_IS_POWERED")).off()])
}

/// One DC bus dead. No voltage threshold: `deep::electrical` publishes
/// `IS_POWERED` for a DC bus from its own network solve, and unlike the AC
/// side there is no second, independent quantity here to cross-check it
/// against without inventing a DC undervoltage limit this port has no source
/// for.
fn dc_dead(tag: &str) -> Cond {
    var(&format!("ELEC_{tag}_BUS_IS_POWERED")).off()
}

fn bus_fault(id: u64, title: &'static str, parts: Vec<Cond>, note: &'static str) -> FbwProc {
    let mut conds = parts;
    conds.push(network_alive());
    // 0.5 s, the same confirmation `deep::electrical::registry`'s own
    // `ELEC AC BUS 1 FAULT` uses, so a contactor transfer is not a fault.
    proc(id, title, Level::Caution, sd_page::ELEC_AC, all(conds), note).confirm(0.5).inhibit(phase::TAKEOFF_AND_LANDING_ROLL)
}

pub fn wire(v: &mut Vec<FbwProc>) {
    // ---- Single AC buses. Each is suppressed while the combined procedure
    // that covers it is active, exactly as FlyByWire's own entries do it
    // (`211800009`'s `notActiveWhenItemActive: ['211800021']`).
    v.push(
        bus_fault(240_800_004, "ELEC AC BUS 2 FAULT", vec![ac_dead("AC_2")], "AC bus 2 unpowered and below 90 V while the AC network is live")
            .suppressed_by(&[240_800_005, 240_800_006])
            // Every one of the eight lines stays crew-actioned. The two
            // "GEN 1+2 ... OFF THEN ON" resets are a *cycle*, and the
            // generator pushbutton this port publishes
            // (`A32NX_OVHD_ELEC_ENG_GEN_n_PB_IS_ON`, `src/aspects.rs:612`)
            // shows only a state, never a cycle; "COMMERCIAL 1 ... OFF" is
            // the overhead commercial pushbutton, which nothing here
            // publishes (`ELEC_COMMERCIAL_SHED_ACTIVE` is the *automatic*
            // load shed, a different thing, and ticking the line from it
            // would claim the crew had acted when the aircraft had);
            // "EMER OUTR TK XFR ... ON" is a fuel overhead pushbutton
            // `deep::fuel` does not publish either.
            .items(8, Vec::new()),
    );
    v.push(
        bus_fault(240_800_007, "ELEC AC BUS 3 FAULT", vec![ac_dead("AC_3")], "AC bus 3 unpowered and below 90 V while the AC network is live")
            .suppressed_by(&[240_800_005, 240_800_008])
            .items(12, Vec::new()),
    );
    v.push(
        bus_fault(240_800_009, "ELEC AC BUS 4 FAULT", vec![ac_dead("AC_4")], "AC bus 4 unpowered and below 90 V while the AC network is live")
            .suppressed_by(&[240_800_006, 240_800_008])
            .items(9, Vec::new()),
    );
    v.push(
        bus_fault(240_800_010, "ELEC AC EMER BUS FAULT", vec![ac_dead("AC_EMER")], "the AC emergency bus is unpowered and below 90 V while the AC network is live").items(4, Vec::new()),
    );
    v.push(
        bus_fault(240_800_012, "ELEC AC ESS BUS FAULT", vec![ac_dead("AC_ESS")], "the AC essential bus is unpowered and below 90 V while the AC network is live").items(11, Vec::new()),
    );

    // ---- Combined AC bus losses. The trigger is the conjunction of the
    // same single-bus conditions, so a combined procedure can only be active
    // when every bus it names really is dead.
    v.push(
        bus_fault(240_800_006, "ELEC AC BUS 2+4 FAULT", vec![ac_dead("AC_2"), ac_dead("AC_4")], "AC buses 2 and 4 both dead while 1 or 3 still carries the network")
            .items(28, Vec::new()),
    );
    v.push(
        bus_fault(
            240_800_005,
            "ELEC AC BUS 2+3 & DC BUS 1+2 FAULT",
            vec![ac_dead("AC_2"), ac_dead("AC_3"), dc_dead("DC_1"), dc_dead("DC_2")],
            "AC buses 2 and 3 and both main DC buses dead together -- the two TRs they feed lose their source with them",
        )
        .items(41, Vec::new()),
    );
    v.push(
        bus_fault(
            240_800_008,
            "ELEC AC BUS 3+4 & DC BUS 2 FAULT",
            vec![ac_dead("AC_3"), ac_dead("AC_4"), dc_dead("DC_2")],
            "AC buses 3 and 4 dead together with the DC bus 2 they feed through TR 2",
        )
        .items(15, Vec::new()),
    );

    // ---- DC buses.
    v.push(
        bus_fault(240_800_026, "ELEC DC BUS 1 FAULT", vec![dc_dead("DC_1")], "DC bus 1 unpowered while the AC network is live")
            .suppressed_by(&[240_800_005, 240_800_027, 240_800_028])
            .items(11, Vec::new()),
    );
    v.push(
        bus_fault(240_800_029, "ELEC DC BUS 2 FAULT", vec![dc_dead("DC_2")], "DC bus 2 unpowered while the AC network is live")
            .suppressed_by(&[240_800_005, 240_800_008, 240_800_027])
            .items(12, Vec::new()),
    );
    v.push(
        bus_fault(240_800_027, "ELEC DC BUS 1+2 FAULT", vec![dc_dead("DC_1"), dc_dead("DC_2")], "both main DC buses unpowered while the AC network is live")
            .items(22, Vec::new()),
    );
    v.push(
        bus_fault(240_800_030, "ELEC DC ESS BUS FAULT", vec![dc_dead("DC_ESS")], "the DC essential bus is unpowered while the AC network is live")
            .suppressed_by(&[240_800_028])
            .items(20, Vec::new()),
    );
    v.push(
        bus_fault(240_800_028, "ELEC DC BUS 1+ESS FAULT", vec![dc_dead("DC_1"), dc_dead("DC_ESS")], "DC bus 1 and the DC essential bus unpowered together")
            .items(21, Vec::new()),
    );
    v.push(
        bus_fault(
            240_800_031,
            "ELEC DC ESS BUS PART FAULT",
            vec![var("ELEC_DC_ESS_BUS_IS_POWERED").on(), var("ELEC_DC_ESS_SHED_BUS_IS_POWERED").off()],
            "the DC essential bus is still live but its shed section is not -- part of the bus, which is what PART FAULT names",
        )
        .suppressed_by(&[240_800_030])
        .items(16, Vec::new()),
    );

    // ---- Individual units. Each of these reads one boolean
    // `deep::electrical` publishes from its own model of that unit.
    v.push(
        proc(240_800_016, "ELEC APU TR FAULT", Level::Caution, sd_page::ELEC_DC, var("ELEC_TR_APU_FAULT").on(), "deep::electrical's own verdict on the APU transformer-rectifier")
            .confirm(1.0)
            .items(2, Vec::new()),
    );
    v.push(
        proc(240_800_018, "ELEC BAT 2 (ESS) FAULT", Level::Caution, sd_page::ELEC_DC, var("ELEC_BAT_2_FAULT").on(), "deep::electrical's own verdict on battery 2; battery 1's own procedure is already raised by our ELEC BAT 1 FAULT")
            .confirm(1.0)
            .items(1, Vec::new()),
    );
    for n in 2..=4u64 {
        // GEN 1's procedure is left to our own `ELEC GEN 1 FAULT`; see the
        // module doc.
        let (id, title) = match n {
            2 => (240_800_062, "ELEC GEN 2 FAULT"),
            3 => (240_800_063, "ELEC GEN 3 FAULT"),
            _ => (240_800_064, "ELEC GEN 4 FAULT"),
        };
        v.push(
            proc(id, title, Level::Caution, sd_page::ELEC_AC, var(&format!("ELEC_GEN_{n}_FAULT")).on(), "deep::electrical's own verdict on that engine generator")
                .confirm(1.0)
                .items(
                    1,
                    // "GEN n ... OFF" -- the overhead generator pushbutton,
                    // which `src/aspects.rs:612-618` copies from MSFS's
                    // `GENERAL ENG MASTER ALTERNATOR:n` into
                    // `A32NX_OVHD_ELEC_ENG_GEN_n_PB_IS_ON` every frame, so
                    // this line ticks when the crew actually selects the
                    // generator off and at no other time.
                    vec![item(0).checked(var(&format!("A32NX_OVHD_ELEC_ENG_GEN_{n}_PB_IS_ON")).off())],
                ),
        );
    }
    // ---- C/B TRIPPED. `deep::breakers` runs all 399 trip units of the
    // ELMS and, since this pass, publishes the one aggregate this procedure
    // needs: how many of them a *protection element* opened. The count it
    // already had, `BREAKERS_OPEN_COUNT`, is `!closed` and therefore rises
    // the moment the crew pulls a breaker on purpose -- an alert on that
    // would call a deliberate isolation a fault. The new count is built
    // from `trip::Breaker::status()`, which already separates
    // `OpenCommanded` (a CDS/OIT `remote_open` or a pull at the panel) from
    // `Tripped(Thermal|Magnetic|ArcFault)` and the repeated-trip
    // `LockedOut` latch. `>= 1` rather than `> 0` because the count is a
    // whole number of breakers.
    //
    // No network-alive gate: unlike a bus procedure this is not a statement
    // about a bus being dark. A cold and dark aeroplane has every breaker
    // closed (`BreakersLive::new`), so the count is 0 and the procedure is
    // silent without one.
    v.push(
        proc(
            240_800_021,
            "ELEC C/B TRIPPED",
            Level::Caution,
            sd_page::CB,
            var("BREAKERS_TRIPPED_NOT_COMMANDED_COUNT").ge(1.0),
            "at least one of deep::breakers' 399 trip units was opened by its own thermal, magnetic, arc-fault or lockout element rather than by a crew command",
        )
        // 1 s, so a breaker that opens and is immediately reset by the
        // model's own coupling with `deep::electrical` does not flash the
        // procedure up for a single frame.
        .confirm(1.0)
        .items(0, Vec::new()),
    );

    v.push(proc(240_800_082, "ELEC TR 2 FAULT", Level::Caution, sd_page::ELEC_DC, var("ELEC_TR_2_FAULT").on(), "deep::electrical's own verdict on TR 2; TR 1's procedure is already raised by our ELEC TR 1 FAULT").confirm(1.0));
    v.push(proc(240_800_083, "ELEC TR ESS FAULT", Level::Caution, sd_page::ELEC_DC, var("ELEC_TR_ESS_FAULT").on(), "deep::electrical's own verdict on the essential TR").confirm(1.0));

    // =========================================================================
    // E-ELEC Phase 2 (2026-09-27), against E:/fbw-debug/ecam/E-ELEC-FCOM.json
    // (Task A) and E-ELEC-DESIGN.md. Every level/inhibit below is the FCOM's
    // own decoded value (BRIEF-phase2-FCOM.md: the FCOM is now the source for
    // inhibition, levels and triggers where it gives them), cited by its
    // PRO-ABN-ECAM page; where the FCOM's own triggering text gave a real
    // number or a materially different meaning than E-ELEC-DESIGN.md guessed
    // (240800001, 240800044, 240800048, 240800072, 240800074/076/078 and
    // siblings), this pass follows the FCOM and says so. `240800003`,
    // `240800017` and `240800055` stay unwired, duplicate-of deep::
    // electrical's own registry alerts, per the module doc comment above and
    // E-ELEC-DESIGN.md's own re-verification.
    // =========================================================================

    // ---- `240800001 ELEC ABNORMAL FLIGHT OPS SUPPLY` -- FCOM p.4831:
    // corrects E-ELEC-DESIGN.md's own guess (a generic OR of this chapter's
    // faults). The real trigger is narrow and specific: "The OITs have been
    // abnormally supplied by the DC ESS busbar in flight" -- the Onboard
    // Information Terminals' normal source (DC bus 2) is dead while DC ESS,
    // which they fail over to, is still live. Both are already-published
    // booleans; no new component. The FCOM's own bar inhibits phases 2-11
    // (visible only at ELEC PWR ON and after the last engine shuts down),
    // which is why no separate on-ground/in-flight gate is added here.
    v.push(
        proc(
            240_800_001,
            "ELEC ABNORMAL FLIGHT OPS SUPPLY",
            Level::Advisory,
            sd_page::ELEC_AC,
            all(vec![dc_dead("DC_2"), var("ELEC_DC_ESS_BUS_IS_POWERED").on()]),
            "FCOM PRO-ABN-ECAM p.4831: the OITs' normal DC 2 source is dead while they are still fed via DC ESS",
        )
        .confirm(1.0)
        .inhibit(&[2, 3, 4, 5, 6, 7, 8, 9, 10, 11])
        .items(0, Vec::new()),
    );

    // ---- `240800011 ELEC AC ESS BUS ALTN` -- publish-only readout of the
    // network's own NORM(AC1)/ALTN(AC4) feeder logic (`ElectricalLive::
    // publish`'s `ELEC_AC_ESS_FED_BY_ALTN`). FCOM p.4866.
    v.push(
        proc(240_800_011, "ELEC AC ESS BUS ALTN", Level::Caution, sd_page::ELEC_AC, var("ELEC_AC_ESS_FED_BY_ALTN").on(), "FCOM p.4866; deep::electrical's own NORM/ALTN feeder-contactor readout")
            .confirm(1.0)
            .inhibit(&[4, 5, 6, 7, 8, 9, 10])
            .items(0, Vec::new()),
    );

    // ---- `240800013 ELEC APU BAT FAULT` -- a third `Battery`/`BatteryFaults`
    // instance, same model as BAT 1/2. FCOM p.4870.
    v.push(
        proc(240_800_013, "ELEC APU BAT FAULT", Level::Caution, sd_page::ELEC_DC, var("ELEC_APU_BAT_FAULT").on(), "FCOM p.4870; a third instance of the existing Battery/BatteryFaults model, dedicated to the APU start/standby battery")
            .confirm(1.0)
            .inhibit(&[3, 4, 5, 6, 7, 9, 10])
            .items(1, Vec::new()),
    );

    // ---- `240800014`/`015 ELEC APU GEN A(B) FAULT` -- EXISTS: per-channel
    // APU generator fault, already published (`live.rs:1850`), not a
    // duplicate of the OR'd `ELEC_APU_GEN_FAULT` (see E-ELEC-DESIGN.md's
    // finding 1). FCOM p.4872.
    v.push(
        proc(240_800_014, "ELEC APU GEN A FAULT", Level::Caution, sd_page::ELEC_AC, var("ELEC_APU_GEN_1_FAULT").on(), "FCOM p.4872; deep::electrical's own per-channel APU generator verdict, channel A")
            .confirm(1.0)
            .inhibit(&[3, 4, 5, 6, 7, 9, 10])
            .items(1, Vec::new()),
    );
    v.push(
        proc(240_800_015, "ELEC APU GEN B FAULT", Level::Caution, sd_page::ELEC_AC, var("ELEC_APU_GEN_2_FAULT").on(), "FCOM p.4872; deep::electrical's own per-channel APU generator verdict, channel B")
            .confirm(1.0)
            .inhibit(&[3, 4, 5, 6, 7, 9, 10])
            .items(1, Vec::new()),
    );

    // ---- `240800019 ELEC BUS TIE OFF` -- FCOM p.4879: "The BUS TIE pb-sw is
    // abnormally set to OFF." No MSFS aspect exists for this switch, so it is
    // armed from the Study panel (`MiscFault::BusTieOff`) like a failure.
    v.push(
        proc(240_800_019, "ELEC BUS TIE OFF", Level::Caution, sd_page::ELEC_AC, var("ELEC_BUS_TIE_OFF").on(), "FCOM p.4879: the BUS TIE pb-sw is abnormally set to OFF")
            .confirm(1.0)
            .inhibit(&[3, 4, 5, 6, 7, 9, 10])
            .items(0, Vec::new()),
    );

    // ---- `240800020`/`054 ELEC (EMER) C/B MONITORING FAULT` -- boolean
    // monitoring-path health, `deep::breakers`. FCOM p.4880/4928: both are
    // STATUS/INOP-SYS-only (Advisory, no aural, no master light), not the
    // Caution/Warning E-ELEC-DESIGN.md guessed before this pass had the FCOM.
    v.push(
        proc(240_800_020, "ELEC C/B MONITORING FAULT", Level::Advisory, sd_page::CB, var("BREAKERS_CB_MONITORING_FAULT").on(), "FCOM p.4880: the C/B monitoring function is failed; STATUS-only (INOP SYS), no aural or master light")
            .confirm(1.0)
            .inhibit(&[2, 3, 4, 5, 6, 7, 8, 9, 10, 11])
            .items(0, Vec::new()),
    );
    v.push(
        proc(
            240_800_054,
            "ELEC EMER C/B MONITORING FAULT",
            Level::Advisory,
            sd_page::CB,
            all(vec![var("BREAKERS_EMER_CB_MONITORING_FAULT").on(), var("ELEC_EMER_CONFIG_ACTIVE").on()]),
            "FCOM p.4928: the emergency part of the C/B monitoring function is failed, gated by ELEC_EMER_CONFIG_ACTIVE since the emergency path is only in circuit then",
        )
        .confirm(1.0)
        .inhibit(&[2, 3, 4, 5, 6, 7, 8, 9, 10, 11])
        .items(0, Vec::new()),
    );

    // ---- `240800022`/`023 ELEC CABIN L(R) SUPPLY CENTER OVHT` and
    // `240800024`/`025 ... OVHT DET FAULT` -- FCOM p.4882/4883: the
    // detector's own trip, and the detector's own health, as two independent
    // booleans (`deep::electrical`'s `cabin-ovht-{l,r}`/`cabin-ovht-det-
    // {l,r}` MISC_FAULTS entries) rather than a new thermal-zone build.
    v.push(
        proc(240_800_022, "ELEC CABIN L SUPPLY CENTER OVHT", Level::Caution, sd_page::ELEC_AC, var("ELEC_CABIN_L_SUPPLY_CENTER_OVHT").on(), "FCOM p.4882: the overheat detectors have detected an overheat in the left cabin supply center")
            .confirm(3.0)
            .inhibit(&[4, 5, 6, 7, 9, 10])
            .items(1, Vec::new()),
    );
    v.push(
        proc(240_800_023, "ELEC CABIN R SUPPLY CENTER OVHT", Level::Caution, sd_page::ELEC_AC, var("ELEC_CABIN_R_SUPPLY_CENTER_OVHT").on(), "FCOM p.4882: same as 240800022, right cabin supply center")
            .confirm(3.0)
            .inhibit(&[4, 5, 6, 7, 9, 10])
            .items(1, Vec::new()),
    );
    v.push(
        proc(240_800_024, "ELEC CABIN L SUPPLY CENTER OVHT DET FAULT", Level::Advisory, sd_page::ELEC_AC, var("ELEC_CABIN_L_SUPPLY_CENTER_OVHT_DET_FAULT").on(), "FCOM p.4883: the left cabin supply center's own overheat detector has failed; STATUS-only")
            .confirm(1.0)
            .inhibit(&[2, 3, 4, 5, 6, 7, 8, 9, 10, 11])
            .items(0, Vec::new()),
    );
    v.push(
        proc(240_800_025, "ELEC CABIN R SUPPLY CENTER OVHT DET FAULT", Level::Advisory, sd_page::ELEC_AC, var("ELEC_CABIN_R_SUPPLY_CENTER_OVHT_DET_FAULT").on(), "FCOM p.4883: same as 240800024, right cabin supply center")
            .confirm(1.0)
            .inhibit(&[2, 3, 4, 5, 6, 7, 8, 9, 10, 11])
            .items(0, Vec::new()),
    );

    // ---- `240800032`-`035 ELEC DRIVE n DISC FAULT` -- FCOM p.4917: "the
    // disconnection function of the engine generator from its assigned
    // engine failed" -- the disconnect mechanism's own fault-detection
    // circuit, boolean (`GeneratorDrive`'s per-generator `drive-disc-n`
    // MISC_FAULTS entry).
    for n in 1..=4u64 {
        let (id, title) = match n {
            1 => (240_800_032, "ELEC DRIVE 1 DISC FAULT"),
            2 => (240_800_033, "ELEC DRIVE 2 DISC FAULT"),
            3 => (240_800_034, "ELEC DRIVE 3 DISC FAULT"),
            _ => (240_800_035, "ELEC DRIVE 4 DISC FAULT"),
        };
        v.push(
            proc(id, title, Level::Caution, sd_page::ELEC_AC, var(&format!("ELEC_DRIVE_{n}_DISC_FAULT")).on(), "FCOM p.4917: the disconnection function of that engine generator's drive failed")
                .confirm(1.0)
                .inhibit(&[2, 3, 4, 5, 6, 7, 8, 9, 10, 11])
                .items(0, Vec::new()),
        );
    }

    // ---- `240800036`-`039 ELEC DRIVE n DISCONNECTED` -- FCOM p.4918: the
    // generator has actually disconnected from its engine while operating.
    // No separate "disconnect event" state exists; this reuses the same
    // already-sourced GEN n FAULT overload/over-under-voltage verdict
    // `ata24.rs`'s own GEN n FAULT procedures already raise, at a longer
    // confirm (5 s vs 1 s) representing a sufficiently sustained fault
    // escalating to a physical drive disconnect, matching real GCU behaviour
    // (the control breaker and the drive disconnect together).
    for n in 1..=4u64 {
        let (id, title) = match n {
            1 => (240_800_036, "ELEC DRIVE 1 DISCONNECTED"),
            2 => (240_800_037, "ELEC DRIVE 2 DISCONNECTED"),
            3 => (240_800_038, "ELEC DRIVE 3 DISCONNECTED"),
            _ => (240_800_039, "ELEC DRIVE 4 DISCONNECTED"),
        };
        v.push(
            proc(
                id,
                title,
                Level::Caution,
                sd_page::ELEC_AC,
                var(&format!("ELEC_GEN_{n}_FAULT")).on(),
                "FCOM p.4918: the generator disconnects from its engine while operating -- modelled as that generator's own already-sourced FAULT verdict sustained long enough (5 s) to represent escalation to a physical disconnect",
            )
            .confirm(5.0)
            .inhibit(&[3, 4, 5, 6, 7, 9, 10])
            .items(1, Vec::new()),
        );
    }

    // ---- `240800040`-`043 ELEC DRIVE n OIL LEVEL LO` -- FCOM p.4920: "the
    // oil level of the engine driven generator is low." Reuses
    // `apu::oil.rs`'s own low-level trip fraction
    // (`OIL_LOW_LEVEL_TRIP_L / OIL_TANK_CAPACITY_L` = 2.0/8.0 = 0.25), the
    // same GENERIC lubrication-system convention this codebase already
    // ships, since the FCOM gives no percentage of its own for this line.
    const OIL_LEVEL_LO_FRAC: f64 = 0.25;
    for n in 1..=4u64 {
        let (id, title) = match n {
            1 => (240_800_040, "ELEC DRIVE 1 OIL LEVEL LO"),
            2 => (240_800_041, "ELEC DRIVE 2 OIL LEVEL LO"),
            3 => (240_800_042, "ELEC DRIVE 3 OIL LEVEL LO"),
            _ => (240_800_043, "ELEC DRIVE 4 OIL LEVEL LO"),
        };
        v.push(
            proc(
                id,
                title,
                Level::Caution,
                sd_page::ELEC_AC,
                var(&format!("ELEC_DRIVE_{n}_OIL_LEVEL_FRAC")).lt(OIL_LEVEL_LO_FRAC),
                "FCOM p.4920: the oil level of the engine driven generator is low; threshold reused from apu::oil.rs's own OIL_LOW_LEVEL_TRIP_L/OIL_TANK_CAPACITY_L",
            )
            .confirm(5.0)
            .inhibit(&[2, 3, 4, 5, 6, 7, 8, 9, 10, 11])
            .items(1, Vec::new()),
        );
    }

    // ---- `240800044`-`047 ELEC DRIVE n OIL OVHT` -- FCOM p.4920 gives its
    // own real number: "higher than 200 degC."
    for n in 1..=4u64 {
        let (id, title) = match n {
            1 => (240_800_044, "ELEC DRIVE 1 OIL OVHT"),
            2 => (240_800_045, "ELEC DRIVE 2 OIL OVHT"),
            3 => (240_800_046, "ELEC DRIVE 3 OIL OVHT"),
            _ => (240_800_047, "ELEC DRIVE 4 OIL OVHT"),
        };
        v.push(
            proc(id, title, Level::Caution, sd_page::ELEC_AC, var(&format!("ELEC_DRIVE_{n}_OIL_TEMP_C")).gt(200.0), "FCOM p.4920: the oil temperature is higher than 200 degC")
                .confirm(5.0)
                .inhibit(&[1, 3, 4, 5, 6, 7, 9, 10, 12])
                .items(2, Vec::new()),
        );
    }

    // ---- `240800048`-`051 ELEC DRIVE n OIL PRESS LO` -- FCOM p.4948 gives
    // its own real number: "lower than 35 PSI," a different physical system
    // from the APU's own 15 PSI oil trip, so the raw pressure is compared
    // directly rather than reading the internally-15-PSI-calibrated
    // `low_pressure_tripped` boolean.
    for n in 1..=4u64 {
        let (id, title) = match n {
            1 => (240_800_048, "ELEC DRIVE 1 OIL PRESS LO"),
            2 => (240_800_049, "ELEC DRIVE 2 OIL PRESS LO"),
            3 => (240_800_050, "ELEC DRIVE 3 OIL PRESS LO"),
            _ => (240_800_051, "ELEC DRIVE 4 OIL PRESS LO"),
        };
        v.push(
            proc(id, title, Level::Caution, sd_page::ELEC_AC, var(&format!("ELEC_DRIVE_{n}_OIL_PRESSURE_PSI")).lt(35.0), "FCOM p.4948: the oil pressure is abnormally low (lower than 35 PSI)")
                .confirm(1.0)
                .inhibit(&[1, 4, 5, 6, 7, 9, 10, 12])
                .items(2, Vec::new()),
        );
    }

    // ---- `240800052`/`053 ELEC ELEC NETWORK MANAGEMENT n FAULT` -- FCOM
    // p.4926: "the ENMU 1(2) is failed." Boolean computer-health fault.
    v.push(
        proc(240_800_052, "ELEC ELEC NETWORK MANAGEMENT 1 FAULT", Level::Caution, sd_page::ELEC_AC, var("ELEC_ENMU_1_FAULT").on(), "FCOM p.4926: ENMU 1 is failed")
            .confirm(1.0)
            .inhibit(&[3, 4, 5, 6, 7, 9, 10])
            .items(1, Vec::new()),
    );
    v.push(
        proc(240_800_053, "ELEC ELEC NETWORK MANAGEMENT 2 FAULT", Level::Caution, sd_page::ELEC_AC, var("ELEC_ENMU_2_FAULT").on(), "FCOM p.4926: ENMU 2 is failed")
            .confirm(1.0)
            .inhibit(&[3, 4, 5, 6, 7, 9, 10])
            .items(1, Vec::new()),
    );

    // ---- `240800056`-`059 ELEC EXT PWR n FAULT` -- FCOM p.4943: "the
    // external power unit, or its associated GGPCU, is failed." A boolean
    // unit/GGPCU health fault (`ext-pwr-n-fault` MISC_FAULTS entry), gated by
    // that receptacle actually being on line (`ELEC_EXT_PWR_n_ON_LINE`,
    // deep::electrical's own already-published contactor readout) since no
    // deep area publishes the raw "plugged in" aspect (`EXT_PWR_AVAIL:n`
    // lives only in `src/aspects.rs`, not a deep `publish()`).
    // STATUS/INOP-SYS only (Advisory), all phases but 1/2/12 inhibited.
    for n in 1..=4u64 {
        let (id, title) = match n {
            1 => (240_800_056, "ELEC EXT PWR 1 FAULT"),
            2 => (240_800_057, "ELEC EXT PWR 2 FAULT"),
            3 => (240_800_058, "ELEC EXT PWR 3 FAULT"),
            _ => (240_800_059, "ELEC EXT PWR 4 FAULT"),
        };
        v.push(
            proc(
                id,
                title,
                Level::Advisory,
                sd_page::ELEC_AC,
                all(vec![var(&format!("ELEC_EXT_PWR_{n}_ON_LINE")).on(), var(&format!("ELEC_EXT_PWR_{n}_FAULT")).on()]),
                "FCOM p.4943: the external power unit, or its associated GGPCU, is failed; gated by that receptacle being on line",
            )
            .confirm(0.5)
            .inhibit(&[3, 4, 5, 6, 7, 8, 9, 10])
            .items(1, Vec::new()),
        );
    }

    // ---- `240800060 ELEC F/CTL ACTUATOR PWR SUPPLY FAULT` -- boolean
    // power-conditioning-unit health, STATUS-only (Advisory).
    v.push(
        proc(240_800_060, "ELEC  F/CTL ACTUATOR PWR SUPPLY FAULT", Level::Advisory, sd_page::FCTL, var("ELEC_FCTL_ACTUATOR_PWR_FAULT").on(), "the F/CTL EHA/EBHA power-conditioning unit's own health, independent of the AC/DC bus it draws from")
            .confirm(1.0)
            .inhibit(&[3, 4, 5, 6, 7, 8, 9, 10, 11])
            .items(0, Vec::new()),
    );

    // ---- `240800065`-`068 ELEC GEN n OFF` -- EXISTS: the overhead generator
    // pushbutton, already published every frame (`src/aspects.rs:612-618`).
    // FCOM p.4947: "the GEN n pb-sw is abnormally set to OFF."
    for n in 1..=4u64 {
        let (id, title) = match n {
            1 => (240_800_065, "ELEC GEN 1 OFF"),
            2 => (240_800_066, "ELEC GEN 2 OFF"),
            3 => (240_800_067, "ELEC GEN 3 OFF"),
            _ => (240_800_068, "ELEC GEN 4 OFF"),
        };
        v.push(
            proc(id, title, Level::Caution, sd_page::ELEC_AC, var(&format!("ELEC_GEN_{n}_PB_ON")).off(), "FCOM p.4947: the GEN n pb-sw is abnormally set to OFF (deep::electrical's own publish of the same pushbutton state command_contactors reads)")
                .confirm(1.0)
                .inhibit(&[1, 3, 4, 5, 6, 7, 9, 10, 12])
                .items(0, Vec::new()),
        );
    }

    // ---- `240800069 ELEC LOAD MANAGEMENT FAULT` -- FCOM p.4949: "the ELMU
    // is failed." Boolean computer-health fault.
    v.push(
        proc(240_800_069, "ELEC LOAD MANAGEMENT FAULT", Level::Caution, sd_page::ELEC_AC, var("ELEC_ELMU_FAULT").on(), "FCOM p.4949: the ELMU is failed")
            .confirm(1.0)
            .inhibit(&[3, 4, 5, 6, 7, 9, 10, 11])
            .items(1, Vec::new()),
    );

    // ---- `240800070`/`071 ELEC PRIMARY SUPPLY CENTER n FAULT` -- FCOM
    // p.4950: "some loads are abnormally disconnected from the primary
    // supply center." Boolean centre health, STATUS-only (Advisory).
    v.push(
        proc(240_800_070, "ELEC PRIMARY SUPPLY CENTER 1 FAULT", Level::Advisory, sd_page::ELEC_AC, var("ELEC_PSC_1_FAULT").on(), "FCOM p.4950: some loads are abnormally disconnected from PSC1")
            .confirm(1.0)
            .inhibit(&[2, 3, 4, 5, 6, 7, 8, 9, 10, 11])
            .items(0, Vec::new()),
    );
    v.push(
        proc(240_800_071, "ELEC PRIMARY SUPPLY CENTER 2 FAULT", Level::Advisory, sd_page::ELEC_AC, var("ELEC_PSC_2_FAULT").on(), "FCOM p.4950: same as 240800070, PSC2")
            .confirm(1.0)
            .inhibit(&[2, 3, 4, 5, 6, 7, 8, 9, 10, 11])
            .items(0, Vec::new()),
    );

    // ---- `240800072 ELEC RAT FAULT` -- FCOM p.4952: the RAT's own standing
    // health verdict (failed, stowed-not-locked, heater failed, overload,
    // under/over-voltage, short-circuit, contactor failure), reusing
    // `sf.rat.jammed >= DEGRADED_BEYOND_HALF`, the same threshold and
    // convention already applied to the TRUs and the static inverter --
    // corrects E-ELEC-DESIGN.md's own "commanded but not producing" guess.
    // STATUS-only (Advisory).
    v.push(
        proc(240_800_072, "ELEC RAT FAULT", Level::Advisory, sd_page::ELEC_AC, var("ELEC_RAT_FAULT").on(), "FCOM p.4952: the RAT's own standing health verdict (failed/stowed-not-locked/heater failed/electrical fault)")
            .confirm(2.0)
            .inhibit(&[3, 4, 5, 6, 7, 8, 9, 10, 11])
            .items(0, Vec::new()),
    );

    // ---- `240800073 ELEC REMOTE C/B CTL ACTIVE` -- FCOM p.4954:
    // "maintenance personnel left the REMOTE C/B CTL pb (maintenance panel)
    // set to ON." No MSFS aspect for this maintenance-panel switch, so
    // (like 240800019) armed from the Study panel.
    v.push(
        proc(240_800_073, "ELEC REMOTE C/B CTL ACTIVE", Level::Caution, sd_page::CB, var("BREAKERS_REMOTE_CTL_ACTIVE").on(), "FCOM p.4954: maintenance personnel left the REMOTE C/B CTL pb set to ON")
            .confirm(1.0)
            .inhibit(&[1, 3, 4, 5, 6, 7, 9, 10, 12])
            .items(1, Vec::new()),
    );

    // ---- `240800074`-`079 ELEC SECONDARY SUPPLY CENTER n DEGRADED/FAULT/
    // REDUND LOST` -- re-read against the FCOM (p.4955/4956/4958) rather than
    // E-ELEC-DESIGN.md's own contactor-pair guess: three independent real
    // conditions per centre (communication degraded; some systems no longer
    // supplied; backup-path redundancy lost), each its own boolean. All
    // STATUS-only (Advisory).
    v.push(
        proc(240_800_074, "ELEC SECONDARY SUPPLY CENTER 1 DEGRADED", Level::Advisory, sd_page::ELEC_AC, var("ELEC_SSC_1_DEGRADED").on(), "FCOM p.4955: communication is degraded between SSC1 and CPIOM E / other aircraft systems")
            .confirm(1.0)
            .inhibit(&[2, 3, 4, 5, 6, 7, 8, 9, 10, 11])
            .items(0, Vec::new()),
    );
    v.push(
        proc(240_800_075, "ELEC SECONDARY SUPPLY CENTER 2 DEGRADED", Level::Advisory, sd_page::ELEC_AC, var("ELEC_SSC_2_DEGRADED").on(), "FCOM p.4955: same as 240800074, SSC2")
            .confirm(1.0)
            .inhibit(&[2, 3, 4, 5, 6, 7, 8, 9, 10, 11])
            .items(0, Vec::new()),
    );
    v.push(
        proc(240_800_076, "ELEC SECONDARY SUPPLY CENTER 1 FAULT", Level::Advisory, sd_page::ELEC_AC, var("ELEC_SSC_1_FAULT").on(), "FCOM p.4956: some systems are no longer supplied by SSC1")
            .confirm(1.0)
            .inhibit(&[2, 3, 4, 5, 6, 7, 8, 9, 10, 11])
            .items(0, Vec::new()),
    );
    v.push(
        proc(240_800_077, "ELEC SECONDARY SUPPLY CENTER 2 FAULT", Level::Advisory, sd_page::ELEC_AC, var("ELEC_SSC_2_FAULT").on(), "FCOM p.4956: same as 240800076, SSC2")
            .confirm(1.0)
            .inhibit(&[2, 3, 4, 5, 6, 7, 8, 9, 10, 11])
            .items(0, Vec::new()),
    );
    v.push(
        proc(240_800_078, "ELEC SECONDARY SUPPLY CENTER 1 REDUND LOST", Level::Advisory, sd_page::ELEC_AC, var("ELEC_SSC_1_REDUND_LOST").on(), "FCOM p.4958: the redundancy of some system electrical supply from SSC1 is lost, no operational impact")
            .confirm(1.0)
            .inhibit(&[2, 3, 4, 5, 6, 7, 8, 9, 10, 11])
            .items(0, Vec::new()),
    );
    v.push(
        proc(240_800_079, "ELEC SECONDARY SUPPLY CENTER 2 REDUND LOST", Level::Advisory, sd_page::ELEC_AC, var("ELEC_SSC_2_REDUND_LOST").on(), "FCOM p.4958: same as 240800078, SSC2")
            .confirm(1.0)
            .inhibit(&[2, 3, 4, 5, 6, 7, 8, 9, 10, 11])
            .items(0, Vec::new()),
    );

    // ---- `240800080 ELEC STATIC INV FAULT` -- publish-only readout of the
    // already-computed, already-sourced static-inverter degradation verdict.
    // FCOM p.4959. STATUS-only (Advisory).
    v.push(
        proc(240_800_080, "ELEC STATIC INV FAULT", Level::Advisory, sd_page::ELEC_AC, var("ELEC_STATIC_INV_FAULT").on(), "FCOM p.4959; deep::electrical's own static-inverter degradation verdict (FBW_STATIC_INVERTER/24_004)")
            .confirm(1.0)
            .inhibit(&[3, 4, 5, 6, 7, 8, 9, 10])
            .items(0, Vec::new()),
    );

    // ---- `240800084`-`086 ELEC TR n MONITORING FAULT` -- FCOM p.4963: "the
    // monitoring function of TR n is failed." Boolean, independent of that
    // TR's own electrical verdict. STATUS-only (Advisory).
    v.push(
        proc(240_800_084, "ELEC TR 1 MONITORING FAULT", Level::Advisory, sd_page::ELEC_DC, var("ELEC_TR_1_MONITORING_FAULT").on(), "FCOM p.4963: the monitoring function of TR 1 is failed")
            .confirm(1.0)
            .inhibit(&[2, 3, 4, 5, 6, 7, 8, 9, 10, 11])
            .items(0, Vec::new()),
    );
    v.push(
        proc(240_800_085, "ELEC TR 2 MONITORING FAULT", Level::Advisory, sd_page::ELEC_DC, var("ELEC_TR_2_MONITORING_FAULT").on(), "FCOM p.4963: same as 240800084, TR 2")
            .confirm(1.0)
            .inhibit(&[2, 3, 4, 5, 6, 7, 8, 9, 10, 11])
            .items(0, Vec::new()),
    );
    v.push(
        proc(240_800_086, "ELEC TR ESS MONITORING FAULT", Level::Advisory, sd_page::ELEC_DC, var("ELEC_TR_ESS_MONITORING_FAULT").on(), "FCOM p.4963: same as 240800084, TR ESS")
            .confirm(1.0)
            .inhibit(&[2, 3, 4, 5, 6, 7, 8, 9, 10, 11])
            .items(0, Vec::new()),
    );
}
