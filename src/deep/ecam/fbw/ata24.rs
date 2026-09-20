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
//!   GEN 1 FAULT, `240800081` TR 1 FAULT, `240800055` EMER CONFIG and
//!   `240800014`/`240800015` APU GEN A/B FAULT: `deep::electrical`'s own
//!   `registry.rs` already registers an alert with the same title and the
//!   same trigger variable (`ELEC AC BUS 1 FAULT`, `ELEC BAT 1 FAULT`,
//!   `ELEC GEN 1 FAULT`, `ELEC TR 1 FAULT`, `ELEC EMER CONFIG`, `ELEC APU
//!   GEN FAULT`, the last of which is published as the OR of both APU
//!   generators). Wiring FlyByWire's id as well would put the same warning
//!   on the EWD twice.
//! * `240800003` AC BUS 1+2 & DC BUS 1 FAULT: a combined procedure that
//!   includes AC BUS 1, which our own single-bus alert also announces and
//!   which nothing here can suppress (`notActiveWhenItemActive` only reaches
//!   FlyByWire's own dict, not `deep::api`'s alerts).
//! * `240800065`..`240800068` GEN n OFF, `240800036`..`240800051` DRIVE n
//!   (IDG) faults, `240800056`..`240800059` EXT PWR n FAULT,
//!   `240800070`..`240800079` supply-centre faults, `240800080` STATIC INV,
//!   `240800084`..`240800086` TR MONITORING: no area publishes the
//!   generator-control-unit state, the integrated drive's oil, the external
//!   power receptacles, the primary/secondary supply centres, the static
//!   inverter or the TR monitors. Not modelled; left unwired.

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
}
