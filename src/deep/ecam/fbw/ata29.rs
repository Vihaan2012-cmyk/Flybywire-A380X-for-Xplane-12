//! ATA 29 -- hydraulics. All 18 unwired procedures in `ata29-30.ts` belong
//! to this chapter (every ATA 30 ice-and-rain procedure FlyByWire defines is
//! already wired in `FwsAbnormalSensed.ts`).
//!
//! # ECAM completeness pass (E-FIRE, FCOM addendum)
//!
//! The A380 FCOM (`E:/fbw-debug/ecam/refs/A380-FCOM.txt`, PRO-ABN-ECAM
//! chapter) sources every trigger, level and flight-phase inhibit below --
//! see `E:/fbw-debug/ecam/E-FIRE-FCOM.json` for the full per-id mapping.
//! Three new components close what the previous pass left unmodelled:
//!
//! * **The fuel/hydraulic heat exchanger's own valve**
//!   (`hydraulics::thermal::HxValveFaults`) and **air-leak-detection
//!   switch** (`AirLeakSwitchFaults`), both pure monitored-circuit
//!   discretes -- `HYD_{GREEN,YELLOW}_FUEL_HX_VALVE_FAULT`, `_FUEL_HX_AIR_
//!   LEAK`, `_FUEL_HX_AIR_LEAK_DET_FAULT` (`hydraulics::live::publish_hx`).
//!   `013`/`014`, `015`/`016`, `017`/`018`.
//! * **Dual overheat-detection channels A/B**
//!   (`OverheatChannelFaults`), one pair per circuit --
//!   `HYD_{GREEN,YELLOW}_SYS_CHAN_{A,B}_OVHT_DET_FAULT`. `023`-`026`
//!   directly; `033`/`034` SYS OVHT DET FAULT is their OR (no A/B split in
//!   FlyByWire's own title); `027`/`028` SYS COOLING FAULT and `029`/`030`
//!   SYS MONITORING FAULT are broader ORs over the same discretes (module
//!   doc on each proc below).
//! * **A genuine second, manifold thermal state** distinct from the
//!   reservoir's own (`topology::Circuit`'s `manifold_thermal`/`manifold_
//!   sizing`, `ThermalSizing::a380_manifold`) -- `HYD_{GREEN,YELLOW}_
//!   SYS_TEMP_HI` (`037`/`038`) no longer duplicates `HYD_{G,Y}_RESERVOIR_
//!   OVHT`'s own lumped-state comparison.
//!
//! # What is deliberately left unwired
//!
//! `001`-`012`, `019`-`022`, `031`/`032`, `035`/`036`, `039`/`040`:
//! `deep::hydraulics`'s own `registry.rs` already raises an identically
//! sourced alert for each (electric pump fault, engine pump pressure low,
//! reservoir air/level low, `SYS OVHT` as `RSVR OVHT`, `SYS PRESS LO`,
//! `G+Y SYS PRESS LO`, yellow elec pump A+B off) -- wiring FlyByWire's own
//! id too would put the same warning on the EWD twice. See `E-FIRE-FCOM.
//! json` (`note: "already wired"`) for the full list.

use super::{phase, proc, sd_page, FbwProc};
use crate::deep::api::{any, var, Cond, Level};

pub fn wire(v: &mut Vec<FbwProc>) {
    v.push(
        proc(290_800_013, "HYD G FUEL HEAT EXCHANGER VLV FAULT", Level::Caution, sd_page::HYD, var("HYD_GREEN_FUEL_HX_VALVE_FAULT").on(), "FCOM PRO-ABN-ECAM p.5302")
            .confirm(1.0)
            .inhibit(&[3, 4, 5, 6, 7, 8, 9, 10])
            .items(0, Vec::new()),
    );

    v.push(
        proc(290_800_014, "HYD Y FUEL HEAT EXCHANGER VLV FAULT", Level::Caution, sd_page::HYD, var("HYD_YELLOW_FUEL_HX_VALVE_FAULT").on(), "FCOM PRO-ABN-ECAM p.5302")
            .confirm(1.0)
            .inhibit(&[3, 4, 5, 6, 7, 8, 9, 10])
            .items(0, Vec::new()),
    );

    v.push(
        proc(290_800_015, "HYD G HEAT EXCHANGER AIR LEAK", Level::Advisory, sd_page::HYD, var("HYD_GREEN_FUEL_HX_AIR_LEAK").on(), "FCOM PRO-ABN-ECAM p.5303")
            .confirm(1.0)
            .inhibit(&[2, 3, 4, 5, 6, 7, 8, 9, 10, 11])
            .items(0, Vec::new()),
    );

    v.push(
        proc(290_800_016, "HYD Y HEAT EXCHANGER AIR LEAK", Level::Advisory, sd_page::HYD, var("HYD_YELLOW_FUEL_HX_AIR_LEAK").on(), "FCOM PRO-ABN-ECAM p.5303")
            .confirm(1.0)
            .inhibit(&[2, 3, 4, 5, 6, 7, 8, 9, 10, 11])
            .items(0, Vec::new()),
    );

    v.push(
        proc(290_800_017, "HYD G HEAT EXCHANGER AIR LEAK DET FAULT", Level::Advisory, sd_page::HYD, var("HYD_GREEN_FUEL_HX_AIR_LEAK_DET_FAULT").on(), "FCOM PRO-ABN-ECAM p.5304")
            .confirm(1.0)
            .inhibit(&[2, 3, 4, 5, 6, 7, 8, 9, 10, 11])
            .items(0, Vec::new()),
    );

    v.push(
        proc(290_800_018, "HYD Y HEAT EXCHANGER AIR LEAK DET FAULT", Level::Advisory, sd_page::HYD, var("HYD_YELLOW_FUEL_HX_AIR_LEAK_DET_FAULT").on(), "FCOM PRO-ABN-ECAM p.5304")
            .confirm(1.0)
            .inhibit(&[2, 3, 4, 5, 6, 7, 8, 9, 10, 11])
            .items(0, Vec::new()),
    );

    v.push(
        proc(290_800_023, "HYD G SYS CHAN A OVHT DET FAULT", Level::Advisory, sd_page::HYD, var("HYD_GREEN_SYS_CHAN_A_OVHT_DET_FAULT").on(), "FCOM PRO-ABN-ECAM p.5309")
            .confirm(1.0)
            .inhibit(&[2, 3, 4, 5, 6, 7, 8, 9, 10, 11])
            .items(0, Vec::new()),
    );

    v.push(
        proc(290_800_024, "HYD G SYS CHAN B OVHT DET FAULT", Level::Advisory, sd_page::HYD, var("HYD_GREEN_SYS_CHAN_B_OVHT_DET_FAULT").on(), "FCOM PRO-ABN-ECAM p.5309")
            .confirm(1.0)
            .inhibit(&[2, 3, 4, 5, 6, 7, 8, 9, 10, 11])
            .items(0, Vec::new()),
    );

    v.push(
        proc(290_800_025, "HYD Y SYS CHAN A OVHT DET FAULT", Level::Advisory, sd_page::HYD, var("HYD_YELLOW_SYS_CHAN_A_OVHT_DET_FAULT").on(), "FCOM PRO-ABN-ECAM p.5309")
            .confirm(1.0)
            .inhibit(&[2, 3, 4, 5, 6, 7, 8, 9, 10, 11])
            .items(0, Vec::new()),
    );

    v.push(
        proc(290_800_026, "HYD Y SYS CHAN B OVHT DET FAULT", Level::Advisory, sd_page::HYD, var("HYD_YELLOW_SYS_CHAN_B_OVHT_DET_FAULT").on(), "FCOM PRO-ABN-ECAM p.5309")
            .confirm(1.0)
            .inhibit(&[2, 3, 4, 5, 6, 7, 8, 9, 10, 11])
            .items(0, Vec::new()),
    );

    v.push(
        proc(290_800_027, "HYD G SYS COOLING FAULT", Level::Advisory, sd_page::HYD, any(vec![var("HYD_GREEN_FUEL_HX_VALVE_FAULT").on(), var("HYD_GREEN_FUEL_HX_AIR_LEAK").on()]), "FCOM PRO-ABN-ECAM p.5310")
            .confirm(1.0)
            .inhibit(&[3, 4, 5, 6, 7, 8, 9, 10, 11])
            .items(0, Vec::new()),
    );

    v.push(
        proc(290_800_028, "HYD Y SYS COOLING FAULT", Level::Advisory, sd_page::HYD, any(vec![var("HYD_YELLOW_FUEL_HX_VALVE_FAULT").on(), var("HYD_YELLOW_FUEL_HX_AIR_LEAK").on()]), "FCOM PRO-ABN-ECAM p.5310")
            .confirm(1.0)
            .inhibit(&[3, 4, 5, 6, 7, 8, 9, 10, 11])
            .items(0, Vec::new()),
    );

    v.push(
        proc(290_800_029, "HYD G SYS MONITORING FAULT", Level::Advisory, sd_page::HYD, any(vec![var("HYD_GREEN_SYS_CHAN_A_OVHT_DET_FAULT").on(), var("HYD_GREEN_SYS_CHAN_B_OVHT_DET_FAULT").on(), var("HYD_GREEN_FUEL_HX_VALVE_FAULT").on(), var("HYD_GREEN_FUEL_HX_AIR_LEAK").on()]), "FCOM PRO-ABN-ECAM p.5311")
            .confirm(1.0)
            .inhibit(&[4, 5, 6, 7, 9, 10, 11])
            .items(0, Vec::new()),
    );

    v.push(
        proc(290_800_030, "HYD Y SYS MONITORING FAULT", Level::Advisory, sd_page::HYD, any(vec![var("HYD_YELLOW_SYS_CHAN_A_OVHT_DET_FAULT").on(), var("HYD_YELLOW_SYS_CHAN_B_OVHT_DET_FAULT").on(), var("HYD_YELLOW_FUEL_HX_VALVE_FAULT").on(), var("HYD_YELLOW_FUEL_HX_AIR_LEAK").on()]), "FCOM PRO-ABN-ECAM p.5311")
            .confirm(1.0)
            .inhibit(&[4, 5, 6, 7, 9, 10, 11])
            .items(0, Vec::new()),
    );

    v.push(
        proc(290_800_033, "HYD G SYS OVHT DET FAULT", Level::Advisory, sd_page::HYD, any(vec![var("HYD_GREEN_SYS_CHAN_A_OVHT_DET_FAULT").on(), var("HYD_GREEN_SYS_CHAN_B_OVHT_DET_FAULT").on()]), "FCOM PRO-ABN-ECAM p.5314")
            .confirm(1.0)
            .inhibit(&[3, 4, 5, 6, 7, 8, 9, 10, 11])
            .items(0, Vec::new()),
    );

    v.push(
        proc(290_800_034, "HYD Y SYS OVHT DET FAULT", Level::Advisory, sd_page::HYD, any(vec![var("HYD_YELLOW_SYS_CHAN_A_OVHT_DET_FAULT").on(), var("HYD_YELLOW_SYS_CHAN_B_OVHT_DET_FAULT").on()]), "FCOM PRO-ABN-ECAM p.5314")
            .confirm(1.0)
            .inhibit(&[3, 4, 5, 6, 7, 8, 9, 10, 11])
            .items(0, Vec::new()),
    );

    v.push(
        proc(290_800_037, "HYD G SYS TEMP HI", Level::Caution, sd_page::HYD, var("HYD_GREEN_SYS_TEMP_HI").on(), "FCOM PRO-ABN-ECAM p.5322 -- FCOM title says TEMP EXCESS HI; FlyByWire is SYS TEMP HI -- same procedure, confirmed by page render (identical images/family to HYD SYS OVHT)")
            .confirm(2.0)
            .inhibit(&[4, 5, 6, 7, 9, 10])
            .items(5, Vec::new()),
    );

    v.push(
        proc(290_800_038, "HYD Y SYS TEMP HI", Level::Caution, sd_page::HYD, var("HYD_YELLOW_SYS_TEMP_HI").on(), "FCOM PRO-ABN-ECAM p.5322 -- FCOM title says TEMP EXCESS HI; FlyByWire is SYS TEMP HI -- same procedure, confirmed by page render")
            .confirm(2.0)
            .inhibit(&[4, 5, 6, 7, 9, 10])
            .items(5, Vec::new()),
    );
}
