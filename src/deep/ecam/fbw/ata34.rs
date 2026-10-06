//! ATA 34 -- navigation. FlyByWire defines 58 unwired abnormal-sensed
//! procedures in `AbnormalSensed/ata34.ts`; the first pass wrote off the
//! whole chapter as "navigation receivers we don't model" without opening
//! them id by id. That call was too broad in one direction and (on a
//! second look) too narrow in another: `deep::sensors` publishes per-ADIRU
//! AoA vane jam/heater state, per-unit GPS validity and per-system
//! static-port degradation that nothing else announces, so those are real
//! wins. It also turned out that `FwsAbnormalSensed.ts` already wires the
//! whole ADR (`340800001`-`006`) and most of the radio-altimeter
//! (`340800053`-`062`) groups itself -- checked directly against its
//! source rather than against the stale "0 wired for ATA 34" total the
//! first pass's chapter count implied -- so those ids are correctly left
//! alone: FlyByWire already gives them a trigger, and adding a second one
//! here would be exactly the double-trigger `no_entry_takes_an_id_
//! flybywire_already_triggers` exists to catch.
//!
//! # What this module wires
//!
//! * **AOA n FAULT** (`340800011`/`012`/`013`) from
//!   `DEEP_AOA_n_{JAMMED,HEATER_FAILED}` per unit -- `deep::sensors::
//!   registry`'s own `NAV AOA DISAGREE` only carries the OR of all three,
//!   so it cannot distinguish which vane, and these per-unit ids are new
//!   information, not a duplicate. `FwsAbnormalSensed.ts` does not wire
//!   any of the three (verified directly against its source).
//! * **GPS 2 FAULT** (`340800035`) and **GPS 1+2 FAULT** (`340800036`) from
//!   `DEEP_GPS_2_VALID`/`DEEP_GPS_1_VALID`. `340800034` GPS 1 FAULT is
//!   already `deep::sensors::registry`'s own `NAV GPS 1 FAULT` on the same
//!   `DEEP_GPS_1_VALID == 0` and stays unwired; `035`/`036` are not in
//!   `FwsAbnormalSensed.ts` either.
//! * **STATIC PROBE FAULT** (`340800067`) from
//!   `DEEP_STATIC_{1,2,3,4}_DEGRADED` -- not announced anywhere else and
//!   not in `FwsAbnormalSensed.ts`.
//!
//! # E-ELEC Phase 2 (2026-09-27)
//!
//! Against `E:/fbw-debug/ecam/E-ELEC-FCOM.json` (Task A) and
//! `E-ELEC-DESIGN.md`'s Groups A-G, this pass wired the ADR voter's own
//! outlier composition (`340800007`/`009`/`010`), the unreliable-airspeed
//! reuse of the same composition (`340800071`), the re-opened GPWS group
//! (`341800026`-`028`), the MMR/GPS receiver reuse across FLS/GLS/ILS/LS
//! (`340800022`-`024`/`027`-`032`/`037`-`039`/`046`-`048`), a minimal
//! `InertialReference` (`340800016`/`017`/`020`, `deep::sensors`), FlyByWire's
//! own raw EFIS baro-mode enum compared side to side (`340800018`), the
//! GNSS-degraded publish (`340800033`), two new OAT probe instances
//! (`340800050`/`051`), a minimal PRIM RA-link health flag
//! (`340800056`-`058`, `deep::flight_controls`), three new sideslip vane
//! instances (`340800064`-`066`) and a third TAT probe instance
//! (`340800070`).
//!
//! # What stays unwired, and why
//!
//! * `340800001`-`340800006` every ADR n FAULT and pairwise combo:
//!   `FwsAbnormalSensed.ts` already keys these into its own
//!   `ewdAbnormalSensed` map (confirmed by reading the file directly).
//! * `340800014` AOA DISAGREE: already `deep::sensors::registry`'s own
//!   `NAV AOA DISAGREE`, the OR of all three vanes' jam/heater flags.
//! * `340800053`-`340800055`/`059`-`340800062` RA SYS A/B/C FAULT and every
//!   pair/triple combo: already keyed in `FwsAbnormalSensed.ts` (confirmed
//!   directly).
//! * `340800019` CAPT AND F/O BARO VALUE DISAGREE: no separate FCOM
//!   procedure exists for a numeric baro *value* disagree (only the REF
//!   *mode* procedure `340800018` reads); UNSOURCED.
//! * `340800025`/`026` FM/GPS and FM/IR POS DISAGREE: no latitude/longitude
//!   or FMS position solution is published by any deep area or carried in
//!   `Truth` (confirmed: `deep::sensors`'s GPS model is validity-only, and
//!   building a real dead-reckoning position/geodesic-disagree calculator
//!   from scratch is a new navigation subsystem, not a single alert's
//!   cause); UNSOURCED.
//! * `340800049` LS TUNING DISAGREE: no per-MMR tuned frequency/course/mode
//!   state is published (the GPS/MMR model carries validity only); the
//!   FCOM's own trigger is a receiver-internal auto-tuning comparison, not
//!   a pilot-selectable input this port could compare instead; UNSOURCED.
//! * `340800052` RA DEGRADED: FlyByWire's own source marks this "error
//!   model not implemented", and `deep::sensors` has no accuracy-degraded
//!   state for the radio altimeter distinct from `IN_RANGE`/`VALID`;
//!   UNSOURCED.
//! * `340800063` RESIDUAL AIR SPEED: no FCOM entry and no published
//!   speed/time pair found anywhere in this pass; UNSOURCED.
//! * `340800068`/`069` TAT PROBE 1/2 FAULT: already inside `deep::sensors::
//!   registry`'s own combined `NAV TAT PROBE FAULT`; wiring either
//!   individually would put a second, unit-specific title on the EWD for
//!   the same heater failure.
//! * `340900001`/`340900002` IR ALIGNMENT IN ATT MODE / FLUCTUATING
//!   VERTICAL SPEED: marked `(WIP)` in FlyByWire's own catalogue; UNSOURCED.
//! * `340900003` UNRELIABLE AIRSPEED INDICATION: FlyByWire's own duplicate
//!   title for `340800071`, wired above; not double-triggered.
//! * `341800015`-`341800028` ROW/ROP, TCAS, XPDR, TERR SYS, TAWS and
//!   `340700001`-`340700003` LDG ELEVN/AIR DATA SELECTION: surveillance and
//!   MEMO/INFO entries outside the abnormal-sensed id range this pass
//!   covers (`3418xxxxx`/`3407xxxxx`, not `3408xxxxx`), and not modelled.

use super::{proc, sd_page, FbwProc};
use crate::deep::api::{all, any, var};
use crate::deep::api::Level;

/// One AOA vane's own fault: jammed (mechanically stuck, or heater failure
/// leading to icing and jam per `deep::sensors::registry`'s own
/// `register_aoa_vane` note) or its heater failed outright. Either is a
/// fault of that specific vane.
fn aoa_fault(n: u32) -> crate::deep::api::Cond {
    any(vec![var(&format!("DEEP_AOA_{n}_JAMMED")).on(), var(&format!("DEEP_AOA_{n}_HEATER_FAILED")).on()])
}

/// One GPS/MMR receiver's own validity flag gone false.
fn gps_invalid(n: u32) -> crate::deep::api::Cond {
    var(&format!("DEEP_GPS_{n}_VALID")).eq(0.0)
}

pub fn wire(v: &mut Vec<FbwProc>) {
    // ---- AOA. Per-vane fault; `deep::sensors::registry`'s own
    // `NAV AOA DISAGREE` only carries the OR of all three, so these are new
    // information, not a duplicate.
    v.push(proc(340_800_011, "NAV AOA 1 FAULT", Level::Caution, sd_page::STATUS, aoa_fault(1), "AoA vane 1 jammed or its heater failed (deep::sensors DEEP_AOA_1_JAMMED / DEEP_AOA_1_HEATER_FAILED)").confirm(2.0).items(0, Vec::new()));
    v.push(proc(340_800_012, "NAV AOA 2 FAULT", Level::Caution, sd_page::STATUS, aoa_fault(2), "AoA vane 2 jammed or its heater failed").confirm(2.0).items(0, Vec::new()));
    v.push(proc(340_800_013, "NAV AOA 3 FAULT", Level::Caution, sd_page::STATUS, aoa_fault(3), "AoA vane 3 jammed or its heater failed").confirm(2.0).items(0, Vec::new()));

    // ---- GPS. GPS 1 FAULT is already deep::sensors::registry's own
    // `NAV GPS 1 FAULT` on the same `DEEP_GPS_1_VALID == 0` and is left
    // unwired; the combo below is new.
    v.push(
        proc(340_800_035, "NAV GPS 2 FAULT", Level::Caution, sd_page::STATUS, gps_invalid(2), "GPS/MMR receiver 2's own validity flag gone false (deep::sensors DEEP_GPS_2_VALID)")
            .confirm(5.0)
            .suppressed_by(&[340_800_036])
            .items(0, Vec::new()),
    );
    v.push(
        proc(340_800_036, "NAV GPS 1+2 FAULT", Level::Caution, sd_page::STATUS, all(vec![gps_invalid(1), gps_invalid(2)]), "both GPS/MMR receivers' validity flags false together")
            .confirm(5.0)
            .items(0, Vec::new()),
    );

    // =========================================================================
    // E-ELEC Phase 2 (2026-09-27), against E:/fbw-debug/ecam/E-ELEC-FCOM.json
    // (Task A) and E-ELEC-DESIGN.md. Composition/reuse-only additions: every
    // signal below is already published by an existing component, so these
    // are pure `Cond` compositions in this file, not new deep-area state.
    // =========================================================================

    // ---- Group A: the ADR voter's own already-published per-channel
    // outlier flags (`DEEP_ADR_n_OUTLIER`, `sensors/live.rs`), composed into
    // the disagree/degraded family FlyByWire's own logic does not cover
    // (verified: `340800001`-`006` are FlyByWire's, `007`/`009`/`010` are
    // not). FCOM p.5549/5556 (both Caution; `010`'s combined-disagree
    // subtitle is the same procedure as `009`, not a separate page).
    let adr_outlier = |n: u32| var(&format!("DEEP_ADR_{n}_OUTLIER")).on();
    v.push(
        proc(
            340_800_009,
            "NAV AIR DATA DISAGREE",
            Level::Caution,
            sd_page::STATUS,
            any(vec![adr_outlier(1), adr_outlier(2), adr_outlier(3)]),
            "FCOM p.5556: at least one ADR the voter already flags as an outlier",
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

    // ---- Group F: the same ADR-voter composition, read as "unreliable
    // airspeed" (the voter cannot agree on airspeed across the three ADRs).
    // FlyByWire's own title colour is red for both ids (any level allowed);
    // this pass follows E-ELEC-DESIGN.md's Warning choice -- loss of a
    // primary flight parameter is more severe than the plain disagree. The
    // FCOM's own abnormal-sensed chapter has no content for this title (its
    // real text lives in a *non-sensed*, crew-initiated procedure it
    // references instead, "NAV UNRELIABLE AIRSPEED PROCEDURE"), so the
    // trigger is sourced from the sibling reuse, not from an FCOM page here.
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
    // `340900003` is FlyByWire's own duplicate title under the `9`-kind
    // range for the same alert; per the no-duplicate-id rule only
    // `340800071` is wired, and this entry is a placeholder note rather than
    // a second trigger -- deliberately left unwired (see module doc).

    // ---- Group B: one physical MMR per side serving GPS/ILS/GLS/FLS/LS
    // reception together -- a receiver fault removes every mode it serves.
    // `gps_invalid(n)` is the same validity flag `340800035`/`036` already
    // read. FCOM: FLS p.5576/5577, GLS p.5581/5582 (CAPABILITY LOST and
    // FAULT are FlyByWire's two names for the same receiver-down state, per
    // the FCOM's own identical triggering text under both titles), ILS
    // p.5593 area (LS/ILS share the localizer receiver), all Caution.
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

    // ---- Re-opened GPWS group (`341800026`-`028`): both AESS lanes are
    // gated by real bus power exactly as FlyByWire's own `EfisTawsBridge.ts`
    // gates its terrain/GPWS fail flags -- lane 1 from AC ESS, lane 2 from
    // AC 4, both already published by `deep::electrical`. FCOM p.5601-area
    // (radio altimeter neighbourhood; GPWS itself is not in this FCOM
    // revision's ECAM ABN chapter under its own title, so the level/inhibit
    // below follow the sibling AOA-family convention `ata34.rs` already uses
    // for a single-lane surveillance-computer loss, per E-ELEC-DESIGN.md).
    v.push(
        proc(341_800_026, "SURV GPWS 1 FAULT", Level::Caution, sd_page::STATUS, var("ELEC_AC_ESS_BUS_IS_POWERED").off(), "AESS lane 1 (GPWS) loses its AC ESS bus power source, the same gate FlyByWire's own EfisTawsBridge.ts uses")
            .confirm(2.0)
            .suppressed_by(&[341_800_028])
            .items(3, Vec::new()),
    );
    v.push(
        proc(341_800_027, "SURV GPWS 2 FAULT", Level::Caution, sd_page::STATUS, var("ELEC_AC_4_BUS_IS_POWERED").off(), "AESS lane 2 (GPWS) loses its AC 4 bus power source")
            .confirm(2.0)
            .suppressed_by(&[341_800_028])
            .items(3, Vec::new()),
    );
    v.push(
        proc(
            341_800_028,
            "SURV GPWS 1+2 FAULT",
            Level::Warning,
            sd_page::STATUS,
            all(vec![var("ELEC_AC_ESS_BUS_IS_POWERED").off(), var("ELEC_AC_4_BUS_IS_POWERED").off()]),
            "both AESS lanes down together -- no GPWS capability at all",
        )
        .confirm(2.0)
        .items(1, Vec::new()),
    );

    // ---- Static ports. Not announced anywhere else.
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
        .items(0, Vec::new()),
    );

    // =========================================================================
    // E-ELEC Phase 2 continued (2026-09-27), completing the remaining 19
    // MODEL ids against E-ELEC-FCOM.json and E-ELEC-DESIGN.md's Groups C, D,
    // E and G. `340800052` (RA DEGRADED) and `340900001`/`340900002` (WIP)
    // stay UNSOURCED, unchanged. `340800019` (BARO VALUE DISAGREE),
    // `340800025`/`026` (FM/GPS, FM/IR POS DISAGREE), `340800049` (LS TUNING
    // DISAGREE) and `340800063` (RESIDUAL AIR SPEED) are newly moved to
    // UNSOURCED by this pass -- see this file's own module doc addendum
    // below for why each one genuinely has no real source in this port
    // rather than an invented one.
    // =========================================================================

    // ---- Group G (partial): `340800016 NAV CAPT AND F/O ALT DISAGREE`.
    // FCOM p.5569 gives 500 ft (STD) or 250 ft (QNH); `deep::sensors`
    // applies the tighter 250 ft threshold unconditionally (no baro-mode
    // gate here -- see `DEEP_ADR_CAPT_FO_ALT_DIFF_FT`'s own doc) rather than
    // inventing which of the two applies without one.
    v.push(
        proc(340_800_016, "NAV CAPT AND F/O ALT DISAGREE", Level::Caution, sd_page::STATUS, var("DEEP_ADR_CAPT_FO_ALT_DIFF_FT").gt(250.0), "FCOM p.5569: CAPT/F.O displayed altitude differs by more than 250 ft (QNH) / 500 ft (STD) -- the tighter QNH figure is applied unconditionally")
            .confirm(2.0)
            .inhibit(&[4, 5, 6, 10])
            .items(0, Vec::new()),
    );

    // ---- Group G: `340800017`/`020` -- the IRs the ATT HDG knob selects
    // for each side, compared on their own published outputs
    // (`deep::sensors`, from `physics::adirs`'s strapdown solutions).
    // FCOM p.5570/5573 thresholds (5 deg pitch or roll, 5 deg heading true).
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

    // ---- Group D (partial): `340800018 NAV CAPT AND F/O BARO REF
    // DISAGREE` -- FlyByWire's own raw EFIS baro-mode enum, compared side to
    // side (`deep::sensors::DEEP_BARO_REF_DISAGREE`).
    v.push(
        proc(340_800_018, "NAV CAPT AND F/O BARO REF DISAGREE", Level::Caution, sd_page::STATUS, var("DEEP_BARO_REF_DISAGREE").on(), "FCOM p.5572: the Captain's barometric reference is QNH(STD) while the First Officer's is STD(QNH) -- FlyByWire's own A32NX_FCU_EFIS_{L,R}_DISPLAY_BARO_MODE, compared side to side")
            .confirm(10.0)
            .inhibit(&[3, 4, 5, 6, 10])
            .items(0, Vec::new()),
    );

    // ---- Group B: `340800033 NAV GNSS SIGNAL DEGRADED` -- publish-only
    // exposure of the already-tested mild-jamming magnitude band
    // (`sensors/live.rs`'s own `armed_gps_jamming_costs_that_receiver_its_
    // fix` test proves a mild magnitude leaves `DEEP_GPS_n_VALID` at 1.0
    // while a heavy one drops it to 0.0). No FCOM entry for this title
    // (absent, per E-ELEC-FCOM.json); level/inhibit follow
    // E-ELEC-DESIGN.md's own Advisory choice for "not yet a fault."
    v.push(
        proc(340_800_033, "NAV GNSS SIGNAL DEGRADED", Level::Advisory, sd_page::STATUS, any(vec![var("DEEP_GPS_1_DEGRADED").on(), var("DEEP_GPS_2_DEGRADED").on()]), "a GPS/MMR receiver's own jamming magnitude sits in the tested mild band (fix still good, DEEP_GPS_n_VALID stays 1) rather than the heavy band that fails it outright")
            .confirm(5.0)
            .items(0, Vec::new()),
    );

    // ---- Group C: `340800050`/`051 NAV OAT PROBE 1(2) FAULT` -- new probe
    // instances of the boolean heater-failure convention (see
    // `registry::register_oat_probe_1_2`'s own doc for why no reading is
    // simulated).
    v.push(proc(340_800_050, "NAV OAT PROBE 1 FAULT", Level::Caution, sd_page::STATUS, var("DEEP_OAT_1_HEATER_FAILED").on(), "OAT probe 1's own heater has failed (deep::sensors, a new instance of the same probe class TAT/AoA already use)").confirm(2.0).items(0, Vec::new()));
    v.push(proc(340_800_051, "NAV OAT PROBE 2 FAULT", Level::Caution, sd_page::STATUS, var("DEEP_OAT_2_HEATER_FAILED").on(), "OAT probe 2's own heater has failed").confirm(2.0).items(0, Vec::new()));

    // ---- Group E: `340800056`-`058 NAV RA SYS A(B)(C) LOST BY PRIM` -- a
    // minimal PRIM computer-health flag (`deep::flight_controls`, no
    // control-law model). No FCOM entry for this granularity (absent);
    // level/inhibit follow the sibling RA-fault family's own Caution
    // convention (E-ELEC-DESIGN.md).
    v.push(proc(340_800_056, "NAV RA SYS A LOST BY PRIM", Level::Caution, sd_page::STATUS, var("DEEP_PRIM_1_USING_RA_A").off(), "PRIM 1's own RA-A link has faulted (deep::flight_controls, a minimal PRIM RA-health flag, no control-law model)").confirm(2.0).items(0, Vec::new()));
    v.push(proc(340_800_057, "NAV RA SYS B LOST BY PRIM", Level::Caution, sd_page::STATUS, var("DEEP_PRIM_2_USING_RA_B").off(), "PRIM 2's own RA-B link has faulted").confirm(2.0).items(0, Vec::new()));
    v.push(proc(340_800_058, "NAV RA SYS C LOST BY PRIM", Level::Caution, sd_page::STATUS, var("DEEP_PRIM_3_USING_RA_C").off(), "PRIM 3's own RA-C link has faulted").confirm(2.0).items(0, Vec::new()));

    // ---- `340800064`-`066 NAV SIDESLIP PROBE 1(2)(3) FAULT` -- FCOM
    // p.5611: "one sideslip probe is failed," STATUS-only (Advisory, no
    // aural or master light -- confirmed by rendering the page, blank
    // Indications block, INOP SYS ALL PHASES).
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

    // ---- `340800070 NAV TAT PROBE 3 FAULT` -- a third instance of the same
    // probe class as `068`/`069` (already duplicate-of the combined alert);
    // see `registry::register_tat_probe_3`'s own doc for why ADR 3's own
    // physics still shares probe 1 rather than this new instance.
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

    /// A powered aircraft in the air, so the air-data and nav sensors this
    /// chapter reads are not simply unpowered (which a cold-and-dark
    /// default would report as invalid for reasons that have nothing to do
    /// with the faults under test).
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

    /// Every trigger this module writes must read only names
    /// `deep::live::all_areas()` actually publishes -- the same standing
    /// guard `fbw_tests.rs` runs over the whole aggregation, checked here
    /// too so this file fails on its own if it ever drifts.
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

    /// AOA family: seize vane 2's hinge outright and check the fault lands
    /// on unit 2 alone.
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

    /// GPS family: fail receiver 2's electronics outright and check the
    /// single procedure and the 1+2 combo behave -- the combo needs both.
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

    /// Static-probe family: block one port of one system's pair -- the
    /// averaging line falls back to the healthy port, which is exactly
    /// what its own `degraded` flag exists to catch -- and check the
    /// procedure fires on that alone.
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
