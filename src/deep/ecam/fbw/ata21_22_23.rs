//! ATA 21/22/23 -- air conditioning/pressurisation/ventilation, auto flight,
//! communications. See `E:/fbw-debug/ecam/E-AIR-DESIGN.md` (Phase 1) and
//! `E-AIR-FCOM.json` (the FCOM mapping, Task A of the FCOM addendum) for the
//! full per-id sourcing this module implements.
//!
//! # What changed since the prior (0-of-73) pass
//!
//! That pass found "no area models the pack/valve/fan controllers at this
//! resolution" and wired nothing. This pass adds the missing components
//! (`deep::pneumatic_ducts`, `deep::thermal_zones`, `deep::cabin`,
//! `deep::avionics_network`, `deep::sensors`, and two new areas,
//! `deep::autoflight` and `deep::communications`) and wires **63 of the 73**
//! unwired ids here. The FCOM (2011 revision) closed several numeric
//! thresholds the design sheet had left open (95 C pack outlet, 70 C duct,
//! -0.72/1.45/8.92-9.2 psi cabin differential, 40 s PTT, the real approach-
//! capability OR-condition) -- every one cited below as `FCOM p.<page>`,
//! `E-AIR-FCOM.json`.
//!
//! # Second pass (coordinator request): closing the Truth-bridge gaps
//!
//! A first cut of this module left 18 ids unwired. Re-reviewed per the
//! coordinator's three-part instruction (build a plugin-side Truth bridge
//! where the signal is real and reachable; write an exact `fbw-aircraft`
//! spec only where the signal truly lives inside FlyByWire's own systems;
//! source `T.O ACCELERATION DEGRADED` from FlyByWire's own FMS/FWS code if
//! it exists there). Eight of those 18 are now wired:
//!
//! * `213800008` CAB PRESS DIFF PRESS LO -- all three AND terms are real and
//!   now bridged: `landing_elevation_ft` (the ARINC 429 `A32NX_FM1_LANDING_
//!   ELEVATION` word, already published for the SD PRESS page) and
//!   `vertical_speed_fpm` (X-Plane's own native `VERTICAL SPEED` dataref,
//!   already aliased in `lib.rs`, no FlyByWire dependency at all).
//! * `211800056` AIR ABNORM BLEED CONFIG -- re-read the FCOM text: the "not
//!   displayed on the EWD" line describes the INOP SYS/STATUS half of the
//!   procedure, not the alert itself, and `pneumatic_ducts` already
//!   publishes both halves of the real condition (per-engine isolation,
//!   crossbleed valve state) -- no bridge needed, just a combination of
//!   already-published names.
//! * `220800002` AUTOLAND -- dual AP engagement (`A32NX_AUTOPILOT_{1,2}_
//!   ACTIVE`, real, bridged) and the AUTOLAND-light's own real 200 ft RA
//!   gate (found in the FCOM's AFS abnormal-operations text, DSC-22-FG-90-
//!   10) combine with 220800006's own approach-capability verdict.
//! * `211800045` AIR PACK REGUL DEGRADED, `220800009`-`220800012` AUTO FLT
//!   ENG n A/THR OFF -- the aircraft-wide A/THR status (`A32NX_AUTOTHRUST_
//!   STATUS`, real, bridged) closes half of the per-engine ids; the other
//!   half of these, and all of 211800045, are **pending FlyByWire writes**
//!   (no per-engine A/THR authority signal and no FWD-cargo-flow-demand
//!   verdict exist anywhere in FlyByWire's current source -- confirmed by
//!   reading both the TS `FwsCore`/`AutoThrust` call sites and the WASM
//!   `a380_systems` tree). Specified in `E:/fbw-debug/ecam/
//!   E-AIR-FBW-WRITES.md` and wired now against the pending variable names,
//!   so each lights up the moment the write lands; each area's own test
//!   module proves the downstream logic is correct today.
//!
//! `T.O ACCELERATION DEGRADED` was re-checked against FlyByWire's own FMS/
//! FWS source (`FwsCore.ts`, the whole `systems-host` tree, and the WASM
//! `a380_systems` engine/FADEC code): no acceleration-monitor or expected-
//! acceleration computation exists anywhere in FlyByWire's current
//! codebase, so it stays a documented gap, not a bridge or a write spec.
//!
//! # The 10 still left unwired, and why
//!
//! * `211800045`, `220800009`-`220800012` -- see above: the per-engine/FWD-
//!   cargo half of each is a pending FlyByWire write, but the ids
//!   themselves ARE wired (against the pending variable), so they do not
//!   appear in this list; listed here only for clarity that their full
//!   real behaviour is not live until that write lands.
//! * `221800010` T.O ACCELERATION DEGRADED -- confirmed genuinely absent
//!   from FlyByWire's own code and from this 2011 FCOM revision (checked
//!   the whole text, not only PRO-ABN-ECAM); no source exists to cite.
//! * The 9 DUPLICATE ids (`212800006,014,015,016,017,018,025,026,027`) --
//!   unchanged from Phase 1: already raised under this port's own
//!   `AVNCS_AVIONICS_BAY_*_VENT_FAULT` / `ECS_PACK_BAY_OVHT` ids; wiring
//!   FlyByWire's id too would double-annunciate.
//!
//! Every one of the above stays a documented gap, not a guessed trigger.

use super::{proc, sd_page, FbwProc};
use crate::deep::api::{all, any, var, Cond, Level};

/// At least one main AC bus is live: the same gate `fbw/ata24.rs`'s own
/// `network_alive` uses, for the same reason (`docs/deep/BRIEF.md`'s cold-
/// and-dark rule) -- a real FWS with no electrical power annunciates
/// nothing, and every alert in this module needs the aircraft to have at
/// least house power to have anything to say.
fn network_alive() -> Cond {
    any(vec![
        var("ELEC_AC_1_BUS_IS_POWERED").on(),
        var("ELEC_AC_2_BUS_IS_POWERED").on(),
        var("ELEC_AC_3_BUS_IS_POWERED").on(),
        var("ELEC_AC_4_BUS_IS_POWERED").on(),
    ])
}

fn gated(c: Cond) -> Cond {
    all(vec![c, network_alive()])
}

/// A component-broken flag (0 healthy .. 1 fully faulted) read straight off
/// a published Var, the shape most of this module's entries take. Default
/// `phase::TAKEOFF_AND_LANDING` inhibit unless overridden by the caller.
fn binary(id: u64, title: &'static str, level: Level, sd: i32, var_name: &'static str, inhibit: &'static [u32], item_count: usize, note: &'static str) -> FbwProc {
    proc(id, title, level, sd, gated(var(var_name).gt(0.0)), note).inhibit(inhibit).items(item_count, Vec::new())
}

pub fn wire(v: &mut Vec<FbwProc>) {
    // =======================================================================
    // ATA 21 -- AIR / PACKS
    // =======================================================================

    // 211800013/014: pack n ACM overheat. FCOM's own real 95 C trip (p.4653,
    // E-AIR-FCOM.json) applied here against the real modelled outlet
    // temperature `pneumatic_ducts::live` publishes (DEEP_PNEU_PACK_n_ACM_
    // OUTLET_TEMPERATURE_C) -- see that area's own doc on how the outlet is
    // modelled (a real inlet temperature, degraded cooling effectiveness).
    v.push(
        proc(
            211_800_013,
            "AIR PACK 1 OVHT",
            Level::Caution,
            sd_page::BLEED,
            gated(var("DEEP_PNEU_PACK_1_ACM_OUTLET_TEMPERATURE_C").gt(95.0)),
            "FCOM PRO-ABN-ECAM p.4653: \"the pack outlet temperature is above 95 C\" -- the real, cited trip, applied to pneumatic_ducts' own modelled ACM outlet temperature",
        )
        .inhibit(&[3, 4, 5, 6, 7, 9, 10])
        .items(1, Vec::new()),
    );
    v.push(
        proc(
            211_800_014,
            "AIR PACK 2 OVHT",
            Level::Caution,
            sd_page::BLEED,
            gated(var("DEEP_PNEU_PACK_2_ACM_OUTLET_TEMPERATURE_C").gt(95.0)),
            "mirror of 211800013 for pack 2",
        )
        .inhibit(&[3, 4, 5, 6, 7, 9, 10])
        .items(1, Vec::new()),
    );

    // 211800015/016: pack n regulation train fault.
    v.push(binary(
        211_800_015,
        "AIR PACK 1 REGUL FAULT",
        Level::Caution,
        sd_page::BLEED,
        "DEEP_PNEU_PACK_1_REGUL_FAULT",
        &[3, 4, 5, 6, 7, 9, 10],
        5,
        "pneumatic_ducts's own pack 1 regulation-train component (bypass valve/water extractor/ram-air-door interlock); distinct from the FDAC BothChannelsFault already claimed by the wired 211800009",
    ));
    v.push(binary(
        211_800_016,
        "AIR PACK 2 REGUL FAULT",
        Level::Caution,
        sd_page::BLEED,
        "DEEP_PNEU_PACK_2_REGUL_FAULT",
        &[3, 4, 5, 6, 7, 9, 10],
        5,
        "mirror of 211800015 for pack 2",
    ));

    // 211800017-020: pack n FCV m fault. pneumatic_ducts's own FCV
    // component (see that area's doc on why this is a real component here
    // rather than a bridge to FlyByWire's FcvFault).
    for (id, title, var_name) in [
        (211_800_017u64, "AIR PACK 1 VLV 1 FAULT", "DEEP_PNEU_PACK_1_FCV_1_FAULT"),
        (211_800_018, "AIR PACK 1 VLV 2 FAULT", "DEEP_PNEU_PACK_1_FCV_2_FAULT"),
        (211_800_019, "AIR PACK 2 VLV 1 FAULT", "DEEP_PNEU_PACK_2_FCV_1_FAULT"),
        (211_800_020, "AIR PACK 2 VLV 2 FAULT", "DEEP_PNEU_PACK_2_FCV_2_FAULT"),
    ] {
        v.push(binary(id, title, Level::Advisory, sd_page::BLEED, var_name, &[3, 4, 5, 6, 7, 9, 10, 11], 0, "pneumatic_ducts's own real FCV component; see that area's doc on why this is not the FDAC-bridge the design sheet first planned"));
    }

    // 211800022: pack 1+2 regulation redundancy fault, corrected against the
    // real FCOM text (p.4662) -- see `pneumatic_ducts::live`'s own comment
    // on `pack_regul_redundancy_fault` for the AND-across-packs/OR-across-
    // components formula this trigger reads.
    v.push(binary(
        211_800_022,
        "AIR PACK 1+2 REGUL REDUNDANCY FAULT",
        Level::Caution,
        sd_page::BLEED,
        "DEEP_PNEU_PACK_REGUL_REDUNDANCY_FAULT",
        &[3, 4, 5, 6, 7, 9, 10],
        0,
        "FCOM PRO-ABN-ECAM p.4662 (\"AIR PACK 1+2 REGUL REDUNDANCY LOST\"): at least one of several redundant components failed, on each pack together",
    ));

    // 211800024/028: bulk cargo duct / cockpit-cabin trim-air duct
    // overheat. FCOM's own real 70 C trips (p.4669/p.4675) applied against
    // thermal_zones' own modelled duct-proxy temperature.
    v.push(
        proc(
            211_800_024,
            "COND BULK CARGO DUCT OVHT",
            Level::Caution,
            sd_page::COND,
            gated(var("DEEP_THERM_BULK_CARGO_DUCT_TEMPERATURE_C").gt(70.0)),
            "FCOM PRO-ABN-ECAM p.4669: \"there is a bulk cargo duct overheat when the air temperature inside the duct exceeds 70 C\"",
        )
        .inhibit(&[3, 4, 5, 6, 7, 9, 10])
        .items(1, Vec::new()),
    );
    v.push(
        proc(
            211_800_028,
            "COND DUCT OVHT",
            Level::Caution,
            sd_page::COND,
            gated(var("DEEP_THERM_TRIM_AIR_DUCT_TEMPERATURE_C").gt(70.0)),
            "FCOM PRO-ABN-ECAM p.4675: \"there is a duct overheat when the air temperature inside the applicable duct exceeds 70 C\"",
        )
        .inhibit(&[3, 4, 5, 6, 7, 9, 10])
        .items(0, Vec::new()),
    );

    // 211800030: FWD cargo zone trim-air valve fault.
    v.push(binary(
        211_800_030,
        "COND FWD CARGO TEMP REGUL FAULT",
        Level::Caution,
        sd_page::COND,
        "DEEP_THERM_FWD_CARGO_TRV_FAULT",
        &[3, 4, 5, 6, 7, 9, 10],
        2,
        "thermal_zones's own FWD cargo zone trim-air valve component; distinct from the already-wired aircraft-wide HOT AIR valves 1/2 (211800032/033)",
    ));

    // 211800034: mixer unit pressure regulator fault.
    v.push(binary(
        211_800_034,
        "COND MIXER PRESS REGUL FAULT",
        Level::Advisory,
        sd_page::BLEED,
        "DEEP_PNEU_MIXER_PRESS_REGUL_FAULT",
        &[3, 4, 5, 6, 7, 9, 10, 11],
        0,
        "a380_systems acknowledges this exact real failure mode as an unimplemented TODO (full_digital_agu_controller.rs:287); pneumatic_ducts's own mixer-unit component fills it",
    ));

    // 211800036: purser temperature selector panel fault.
    v.push(binary(
        211_800_036,
        "COND PURSER TEMP SEL FAULT",
        Level::Caution,
        sd_page::COND,
        "DEEP_CABIN_PURSER_TEMP_SEL_FAULT",
        &[3, 4, 5, 6, 7, 9, 10, 11],
        0,
        "cabin's own purser temperature selector panel component (a real crew control FlyByWire already reads, cpiom_b.rs:861, purs_sel_temp_id)",
    ));

    // 211800037/038: ram-air door n position-disagree.
    v.push(binary(
        211_800_037,
        "COND RAM AIR 1 FAULT",
        Level::Advisory,
        sd_page::COND,
        "DEEP_PNEU_RAM_AIR_1_FAULT",
        &[2, 3, 4, 5, 6, 7, 8, 9, 10, 11],
        0,
        "pneumatic_ducts's own ram-air door 1 stuck failure, on the real, already-published door position (COND_PACK_1_RAM_AIR_DOOR_POSITION)",
    ));
    v.push(binary(
        211_800_038,
        "COND RAM AIR 2 FAULT",
        Level::Advisory,
        sd_page::COND,
        "DEEP_PNEU_RAM_AIR_2_FAULT",
        &[2, 3, 4, 5, 6, 7, 8, 9, 10, 11],
        0,
        "mirror of 211800037 for pack 2",
    ));

    // 211800045: pack flow insufficient for the FWD cargo compartment's own
    // demand. **Pending FlyByWire write**: the comparison (`PackFlow::
    // pack_flow_demand` vs `FWD_CARGO_ZONE_VOLUME_CUBIC_METER`) is entirely
    // internal to `cpiom_b.rs`, with no published SimVar either half could
    // be bridged from without a `write()` this worktree may not add --
    // see `E:/fbw-debug/ecam/E-AIR-FBW-WRITES.md`. Wired now against the
    // pending boolean so it lights up the moment that write lands.
    v.push(
        proc(
            211_800_045,
            "AIR PACK REGUL DEGRADED",
            Level::Caution,
            sd_page::BLEED,
            gated(var("DEEP_PNEU_PACK_FLOW_INSUFFICIENT_FWD_CRG").on()),
            "pending FlyByWire write (E-AIR-FBW-WRITES.md): the real PackFlow::pack_flow_demand vs FWD_CARGO_ZONE_VOLUME_CUBIC_METER comparison already computed inside cpiom_b.rs, not yet published",
        )
        .inhibit(&[3, 4, 5, 6, 7, 9, 10])
        .items(2, Vec::new()),
    );

    // =======================================================================
    // ATA 21 -- VENT
    // =======================================================================
    //
    // 212800006/014-018/025-027 stay unwired: DUPLICATE of this port's own
    // AVNCS_AVIONICS_BAY_{AFT,FWD}_VENT_FAULT and ECS_PACK_BAY_OVHT, per
    // E-AIR-DESIGN.md (confirmed again against E-AIR-FCOM.json's own
    // matches for these ids: same real condition, different id, would
    // double-annunciate).

    // 212800012/013: secondary (recirculation) cabin fan count.
    v.push(
        proc(
            212_800_012,
            "COND PART SECONDARY CABIN FANS FAULT",
            Level::Advisory,
            sd_page::COND,
            gated(all(vec![var("DEEP_CABIN_SECONDARY_FANS_FAILED_COUNT").ge(1.0), var("DEEP_CABIN_SECONDARY_FANS_FAILED_COUNT").lt(4.0)])),
            "cabin's own model of the 4 real secondary cabin fans a380_systems already computes per fan (cabin_fan_has_failed, mod.rs:732-734) but does not publish per-fan; 1..3 of 4 failed",
        )
        .inhibit(&[2, 3, 4, 5, 6, 7, 9, 10, 11])
        .suppressed_by(&[212_800_013])
        .items(0, Vec::new()),
    );
    v.push(
        proc(
            212_800_013,
            "COND SECONDARY CABIN FANS FAULT",
            Level::Advisory,
            sd_page::COND,
            gated(var("DEEP_CABIN_SECONDARY_FANS_FAILED_COUNT").ge(4.0)),
            "all 4 secondary cabin fans failed",
        )
        .inhibit(&[3, 4, 5, 6, 7, 9, 10, 11])
        .items(0, Vec::new()),
    );

    // 212800019/020: avionics cooling system n overheat.
    v.push(binary(
        212_800_019,
        "VENT COOLG SYS 1 OVHT",
        Level::Caution,
        sd_page::COND,
        "DEEP_AVNCS_COOLG_1_OVHT",
        &[3, 4, 5, 6, 7, 9, 10],
        3,
        "avionics_network's own supplemental cooling system 1 component; FCOM p.4731 confirms the real LRU (\"there is an overheat on the system 1(2) of the supplemental cooling system\")",
    ));
    v.push(binary(
        212_800_020,
        "VENT COOLG SYS 2 OVHT",
        Level::Caution,
        sd_page::COND,
        "DEEP_AVNCS_COOLG_2_OVHT",
        &[3, 4, 5, 6, 7, 9, 10],
        3,
        "mirror of 212800019 for system 2",
    ));

    // 212800021: cooling protection fault (either system, OR'd).
    v.push(binary(
        212_800_021,
        "VENT COOLG SYS PROT FAULT",
        Level::Advisory,
        sd_page::COND,
        "DEEP_AVNCS_COOLG_PROT_FAULT",
        &[3, 4, 5, 6, 7, 8, 9, 10, 11],
        0,
        "FCOM p.4733: \"the overheat detection system of the supplemental cooling system 1(2) is lost\" -- distinct from the overheat itself",
    ));

    // 212800022/023: IFE bay isolation valve / extraction fan.
    v.push(binary(
        212_800_022,
        "VENT IFE BAY ISOL FAULT",
        Level::Advisory,
        sd_page::COND,
        "DEEP_CABIN_IFE_BAY_ISOL_FAULT",
        &[3, 4, 5, 6, 7, 8, 9, 10, 11],
        0,
        "cabin's own IFE bay isolation valve component",
    ));
    v.push(binary(
        212_800_023,
        "VENT IFE BAY VENT FAULT",
        Level::Caution,
        sd_page::COND,
        "DEEP_CABIN_IFE_BAY_VENT_FAULT",
        &[3, 4, 5, 6, 7, 9, 10],
        2,
        "cabin's own IFE bay extraction fan component",
    ));

    // 212800024: lav & galley extraction fan.
    v.push(binary(
        212_800_024,
        "VENT LAV & GALLEYS EXTRACT FAULT",
        Level::Advisory,
        sd_page::COND,
        "DEEP_CABIN_LAV_GALLEY_EXTRACT_FAULT",
        &[3, 4, 5, 6, 7, 9, 10, 11],
        1,
        "cabin's own lav & galley extraction fan component, distinct from the FWD/BULK cargo extraction fans a380_systems' VCM models",
    ));

    // 212800028: THS bay ventilation fan.
    v.push(binary(
        212_800_028,
        "VENT THS BAY VENT FAULT",
        Level::Advisory,
        sd_page::COND,
        "DEEP_THERM_THS_BAY_VENT_FAULT",
        &[3, 4, 5, 6, 7, 8, 9, 10, 11],
        0,
        "thermal_zones's own THS bay ventilation fan component -- a flag, not a temperature, since no THS bay thermal zone exists anywhere to invent a threshold from",
    ));

    // =======================================================================
    // ATA 21 -- PRESS
    // =======================================================================

    // 213800003: cabin excess negative differential pressure. FCOM (p.4757)
    // is a real Level-3 warning, but FlyByWire's own title colour for this
    // id is `\x1b<4m` (amber) -- `no_entry_is_louder_than_flybywires_own_
    // title_colour` requires the quieter of the two, so this is wired
    // Caution, not Warning, and the mismatch is noted in E-AIR-FCOM.json.
    v.push(
        proc(
            213_800_003,
            "CAB PRESS EXCESS NEGATIVE DIFF PRESS",
            Level::Caution,
            sd_page::PRESS,
            gated(var("DEEP_CPC_1_DIFF_PRESSURE_SENSED_PA").lt(-4964.0)),
            "FCOM PRO-ABN-ECAM p.4757: cabin differential pressure lower than -0.72 PSI (-4964 Pa, 0.72*6894.76); FlyByWire's own real, already-computed FWC signal (cpiom_b.rs:862, FWC_EXCESSIVE_NEGATIVE_DIFF_PRESSURE) via DEEP_CPC_1_DIFF_PRESSURE_SENSED_PA. Wired Caution, not Warning: FlyByWire's own title colour for this id is amber, and no_entry_is_louder_than_flybywires_own_title_colour keeps the quieter of the two (E-AIR-FCOM.json)",
        )
        .inhibit(&[1, 2, 3, 4, 5, 6, 7, 10, 11, 12])
        .items(3, Vec::new()),
    );

    // 213800007: cabin differential pressure high (short of the already-
    // wired 213800002 EXCESS DIFF PRESS).
    v.push(
        proc(
            213_800_007,
            "CAB PRESS DIFF PRESS HI",
            Level::Caution,
            sd_page::PRESS,
            gated(all(vec![var("DEEP_CPC_1_DIFF_PRESSURE_SENSED_PA").gt(61_494.0), var("DEEP_CPC_1_DIFF_PRESSURE_SENSED_PA").lt(63_432.0)])),
            "FlyByWire's own cpiom_b.rs:859-860, FWC_DIFF_PRESS_HI_LOWER_LIMIT=8.92 PSI (61 494 Pa) / _UPPER_LIMIT=9.2 PSI (63 432 Pa); bounded below the already-wired 9.65 PSI EXCESS DIFF PRESS limit",
        )
        .inhibit(&[1, 2, 3, 4, 5, 6, 7, 9, 10, 11, 12])
        .items(3, Vec::new()),
    );

    // 213800010: manual pressurisation control path fault.
    v.push(binary(
        213_800_010,
        "CAB PRESS MAN CTL FAULT",
        Level::Caution,
        sd_page::PRESS,
        "DEEP_PNEU_PRESS_MAN_CTL_FAULT",
        &[3, 4, 5, 6, 7, 9, 10, 11],
        1,
        "pneumatic_ducts's own manual pressurisation control path component, distinct from the already-wired 213800005 AUTO CTL FAULT",
    ));

    // 213800015: all-4-OCSM outflow valve control fault. FCOM has no
    // procedure for the all-4 case (only single-valve and 2-or-3-of-4,
    // E-AIR-FCOM.json), so the inhibit/level follow the numbered siblings'
    // own family per E-AIR-DESIGN.md rather than an FCOM citation.
    v.push(binary(
        213_800_015,
        "CAB PRESS OUTFLW VLV CTL FAULT",
        Level::Caution,
        sd_page::PRESS,
        "DEEP_PNEU_OUTFLW_VLV_CTL_FAULT_ALL",
        &[3, 4, 5, 6, 7, 9, 10, 11],
        0,
        "all 4 OCSMs' own BothChannelsFault together (real FlyByWire per-channel discretes); no FCOM procedure exists for this exact combination, so phase/level follow 213800019-028's own family",
    ));

    // 213800016: CPC sensor validity fault (either CPC, OR'd).
    v.push(
        proc(
            213_800_016,
            "CAB PRESS SENSORS FAULT",
            Level::Advisory,
            sd_page::PRESS,
            gated(any(vec![var("DEEP_CPC_1_SENSOR_FAULT").on(), var("DEEP_CPC_2_SENSOR_FAULT").on()])),
            "sensors's own CPC transducer stuck verdict (already-registered `stuck` field reaching its own documented 1.0 \"fully frozen\" ceiling); distinct from cpcs_has_fault (213800005/029-042) and from adirs_data_is_valid",
        )
        .inhibit(&[3, 4, 5, 6, 7, 9, 10, 11])
        .items(0, Vec::new()),
    );

    // 213800018: cabin air extract valve fault.
    v.push(binary(
        213_800_018,
        "COND CABIN AIR EXTRACT VLV FAULT",
        Level::Advisory,
        sd_page::PRESS,
        "DEEP_PNEU_CABIN_AIR_EXTRACT_VLV_FAULT",
        &[2, 3, 4, 5, 6, 7, 8, 9, 10, 11],
        0,
        "pneumatic_ducts's own cabin air extract valve component (the overhead CABIN AIR EXTRACT pushbutton's own valve)",
    ));

    // 213800008: cabin differential pressure low while descending near the
    // landing field. All three AND terms are now real and bridged: the
    // 1.45 psi limit (9997 Pa, FlyByWire's own cpiom_b.rs LOW_DIFFERENTIAL_
    // PRESSURE_WARNING, FCOM PRO-ABN-ECAM p.4751), FlyByWire's own real
    // FMS-computed landing elevation (`Truth::landing_elevation_ft`, ARINC
    // A32NX_FM1_LANDING_ELEVATION) via sensors' own `DEEP_CPC_1_CABIN_ALT_
    // ABOVE_LANDING_ELEV_FT`, and X-Plane's own real exterior vertical
    // speed (`DEEP_ADIRS_VERTICAL_SPEED_FPM`).
    v.push(
        proc(
            213_800_008,
            "CAB PRESS DIFF PRESS LO",
            Level::Caution,
            sd_page::PRESS,
            gated(all(vec![
                var("DEEP_CPC_1_DIFF_PRESSURE_SENSED_PA").lt(9_997.0),
                var("DEEP_CPC_1_CABIN_ALT_ABOVE_LANDING_ELEV_FT").gt(1_500.0),
                var("DEEP_ADIRS_VERTICAL_SPEED_FPM").lt(-500.0),
            ])),
            "FCOM PRO-ABN-ECAM p.4751: \"during descent, and if the aircraft altitude is at least 1500 ft above the landing field elevation, the cabin differential pressure is almost at 0 PSI\" -- cpiom_b.rs's own 1.45 PSI (9997 Pa) LOW_DIFFERENTIAL_PRESSURE_WARNING, FlyByWire's own real FMS landing elevation, and X-Plane's own real vertical speed",
        )
        .inhibit(&[1, 2, 3, 4, 5, 6, 7, 9, 10, 11, 12])
        .items(2, Vec::new()),
    );

    // 211800056: abnormal bleed configuration -- at least one engine's
    // bleed is isolated while no crossbleed path compensates. Built purely
    // from pneumatic_ducts' own already-published isolation/crossbleed
    // state, no new bridge needed. FCOM PRO-ABN-ECAM p.5661's own real
    // condition ("displayed after AIR ENG n BLEED FAULT/BLEED TEMP LO/ENG n
    // SHUT DOWN, if the appropriate XBLEED valves did not open
    // automatically") is a *commanded-vs-actual* comparison this port has
    // no distinct signal for; this reads the real *resulting* configuration
    // instead (an engine isolated, no crossbleed path open), which is the
    // same physical situation the alert exists to flag.
    v.push(
        proc(
            211_800_056,
            "AIR ABNORM BLEED CONFIG",
            Level::Advisory,
            sd_page::BLEED,
            gated(all(vec![
                any(vec![
                    var("DEEP_PNEU_ENG_1_ISOLATION_OPEN").off(),
                    var("DEEP_PNEU_ENG_2_ISOLATION_OPEN").off(),
                    var("DEEP_PNEU_ENG_3_ISOLATION_OPEN").off(),
                    var("DEEP_PNEU_ENG_4_ISOLATION_OPEN").off(),
                ]),
                var("DEEP_PNEU_XBLEED_L_OPEN").off(),
                var("DEEP_PNEU_XBLEED_C_OPEN").off(),
                var("DEEP_PNEU_XBLEED_R_OPEN").off(),
            ])),
            "FCOM PRO-ABN-ECAM p.5661: an engine bleed isolated with no crossbleed path open, pneumatic_ducts' own already-published DEEP_PNEU_ENG_n_ISOLATION_OPEN / DEEP_PNEU_XBLEED_{L,C,R}_OPEN",
        )
        .inhibit(&[3, 4, 5, 6, 7, 9, 10])
        .items(15, Vec::new()),
    );

    // =======================================================================
    // ATA 22 -- AUTO FLIGHT
    // =======================================================================

    // 220800002: AUTOLAND status. Not in the FCOM's PRO-ABN-ECAM chapter
    // (E-AIR-FCOM.json: absent), so no phase bar to cite -- inhibited on the
    // same narrow late-approach/landing family as its sibling 220800005
    // (FlyByWire's own AFS group). All three AND terms are real: dual AP
    // (`Truth::ap1_active`/`ap2_active`, real `L:A32NX_AUTOPILOT_{1,2}_
    // ACTIVE`), approach capability not downgraded (220800006's own real
    // verdict), and below 200 ft RA -- the AUTOLAND light's own real gate
    // found in the FCOM's AFS abnormal-operations text (DSC-22-FG-90-10,
    // "WARNINGS DURING APPROACH": "when the aircraft is in autoland below
    // 200 ft RA, the AUTOLAND light flashes if..."), the flashing/
    // discontinue case, not this steady status memo, but the same real 200
    // ft gate for the light being lit at all.
    v.push(
        proc(
            220_800_002,
            "AUTOLAND",
            Level::Caution,
            sd_page::STATUS,
            gated(all(vec![
                var("DEEP_AUTOFLT_DUAL_AP_ENGAGED").on(),
                var("DEEP_AUTOFLT_APPROACH_CAPABILITY_DOWNGRADED").off(),
                var("DEEP_AUTOFLT_RADIO_HEIGHT_FT").lt(200.0),
            ])),
            "dual AP engaged (real L:A32NX_AUTOPILOT_{1,2}_ACTIVE) with approach capability not downgraded, below 200 ft RA -- FCOM DSC-22-FG-90-10's own real 200 ft AUTOLAND-light gate",
        )
        .inhibit(&[4, 5, 6, 10])
        .items(0, Vec::new()),
    );

    // 220800005: FCU (AFS control panel) fault.
    v.push(binary(
        220_800_005,
        "AUTO FLT AFS CTL PNL FAULT",
        Level::Caution,
        sd_page::STATUS,
        "DEEP_AUTOFLT_FCU_FAULT",
        &[4, 5, 6, 10],
        3,
        "FCOM PRO-ABN-ECAM p.4785: \"the AFS control panel is failed\"; autoflight's own FCU component",
    ));

    // 220800006: approach capability downgraded -- see autoflight's own doc
    // on why `prim_healthy` alone stands in for the FCOM's own OR of
    // PRIM-internal gyrometer/accelerometer/AOA-sideslip-IRS-vote losses
    // (FCOM p.4789, E-AIR-FCOM.json).
    v.push(binary(
        220_800_006,
        "AUTO FLT APPROACH CAPABILITY DOWNGRADED",
        Level::Caution,
        sd_page::STATUS,
        "DEEP_AUTOFLT_APPROACH_CAPABILITY_DOWNGRADED",
        &[1, 3, 4, 5, 6, 7, 10, 11, 12],
        0,
        "FCOM PRO-ABN-ECAM p.4789's own OR-condition, approximated here by any PRIM unhealthy (Truth::prim_healthy, real and already published) -- see autoflight::registry's own doc for why this port cannot see which PRIM-internal lane failed",
    ));

    // 220800007/008: FCU + capt/F-O MFD backup, compound.
    v.push(
        proc(
            220_800_007,
            "AUTO FLT AFS CTL PNL+CAPT BKUP CTL FAULT",
            Level::Caution,
            sd_page::STATUS,
            gated(all(vec![var("DEEP_AUTOFLT_FCU_FAULT").gt(0.0), var("DEEP_AUTOFLT_CAPT_FCU_BKUP_FAULT").gt(0.0)])),
            "FCOM PRO-ABN-ECAM p.4791: \"the AFS control panel is failed, and the CAPT (F/O) AFS page on the FCU backup is failed\" -- the exact compound condition",
        )
        .inhibit(&[4, 5, 6, 10])
        .items(1, Vec::new()),
    );
    v.push(
        proc(
            220_800_008,
            "AUTO FLT AFS CTL PNL+F/O BKUP CTL FAULT",
            Level::Caution,
            sd_page::STATUS,
            gated(all(vec![var("DEEP_AUTOFLT_FCU_FAULT").gt(0.0), var("DEEP_AUTOFLT_FO_FCU_BKUP_FAULT").gt(0.0)])),
            "mirror of 220800007 for the F/O backup page, same FCOM procedure",
        )
        .inhibit(&[4, 5, 6, 10])
        .items(1, Vec::new()),
    );

    // 220800009-012: per-engine autothrust OFF. The aircraft-wide "armed or
    // active" half is real and bridged (`L:A32NX_AUTOTHRUST_STATUS`,
    // `FwsCore.ts:3121`). **Pending FlyByWire write** for the per-engine
    // half: no per-engine A/THR authority/health signal exists anywhere in
    // FlyByWire's AutoThrust logic today (checked both the TS `FwsCore`/
    // `FlightManagementComputer` and the WASM `a380_systems` -- there is no
    // `autothrust` module there at all, the whole system lives in
    // TypeScript) -- see `E:/fbw-debug/ecam/E-AIR-FBW-WRITES.md`. Wired now
    // against the pending per-engine boolean so it lights up the moment
    // that write lands.
    for (id, title, eng) in [
        (220_800_009u64, "AUTO FLT ENG 1 A/THR OFF", 1u16),
        (220_800_010, "AUTO FLT ENG 2 A/THR OFF", 2),
        (220_800_011, "AUTO FLT ENG 3 A/THR OFF", 3),
        (220_800_012, "AUTO FLT ENG 4 A/THR OFF", 4),
    ] {
        v.push(
            proc(
                id,
                title,
                Level::Caution,
                sd_page::STATUS,
                gated(all(vec![var("DEEP_AUTOFLT_ATHR_ARMED_OR_ACTIVE").on(), var(&format!("DEEP_AUTOFLT_ENG_{eng}_ATHR_FAULT")).on()])),
                "FCOM PRO-ABN-ECAM p.4792: \"the A/THR is armed or active, but failed on the indicated engine\" -- the aircraft-wide status is real (L:A32NX_AUTOTHRUST_STATUS); the per-engine fault is a pending FlyByWire write (E-AIR-FBW-WRITES.md)",
            )
            .inhibit(&[2, 3, 4, 5, 6, 10, 11])
            .items(1, Vec::new()),
        );
    }

    // 220800014: TCAS/AP mode arbitration fault.
    v.push(binary(
        220_800_014,
        "AUTO FLT TCAS MODE FAULT",
        Level::Caution,
        sd_page::STATUS,
        "DEEP_AUTOFLT_TCAS_MODE_FAULT",
        &[3, 4, 5, 6, 9, 10, 11],
        3,
        "FCOM PRO-ABN-ECAM p.4796: \"the AP/FD TCAS mode is failed\"; autoflight's own arbitration-logic component",
    ));

    // 220800015: FCU switched off (a discrete state, not a fault).
    v.push(binary(
        220_800_015,
        "CDS & AUTO FLT FCU SWITCHED OFF",
        Level::Caution,
        sd_page::STATUS,
        "DEEP_AUTOFLT_FCU_SWITCHED_OFF",
        &[2, 3, 4, 5, 6, 7, 9, 10, 11],
        0,
        "FCOM PRO-ABN-ECAM p.4797: \"the FCU is switched off: the EFIS CPs and the AFS CP are electrically shutoff\"",
    ));

    // =======================================================================
    // ATA 23 -- COMMUNICATIONS
    // =======================================================================

    // 230800001-003: CIDS 1+2+3 / cabin com channel fault / degraded.
    v.push(
        proc(
            230_800_001,
            "CAB COM CIDS 1+2+3 FAULT",
            Level::Caution,
            sd_page::STATUS,
            gated(all(vec![var("DEEP_COM_CIDS_1_FAULT").gt(0.0), var("DEEP_COM_CIDS_2_FAULT").gt(0.0), var("DEEP_COM_CIDS_3_FAULT").gt(0.0)])),
            "communications's own 3 CIDS computer components, all 3 ANDed",
        )
        .inhibit(&[3, 4, 5, 6, 7, 9, 10])
        .items(0, Vec::new()),
    );
    let cids_channel_names = ["DEEP_COM_CIDS_PA_UPPER_MAGNITUDE", "DEEP_COM_CIDS_PA_MAIN_MAGNITUDE", "DEEP_COM_CIDS_PA_LOWER_MAGNITUDE", "DEEP_COM_CIDS_INTERPHONE_MAGNITUDE"];
    v.push(
        proc(
            230_800_002,
            "CAB COM CIDS CABIN COM FAULT",
            Level::Caution,
            sd_page::STATUS,
            gated(any(cids_channel_names.iter().map(|n| var(n).ge(1.0)).collect())),
            "communications's own 4 CIDS channel components, any one fully faulted",
        )
        .inhibit(&[3, 4, 5, 6, 7, 9, 10])
        .items(4, Vec::new()),
    );
    v.push(
        proc(
            230_800_003,
            "CAB COM COM DEGRADED",
            Level::Advisory,
            sd_page::STATUS,
            gated(any(cids_channel_names.iter().map(|n| all(vec![var(n).gt(0.0), var(n).lt(1.0)])).collect())),
            "communications's own 4 CIDS channel components, any one degraded (not fully faulted) -- a lesser severity tier of 230800002 on the same components",
        )
        .inhibit(&[3, 4, 5, 6, 7, 8, 9, 10, 11])
        .items(2, Vec::new()),
    );

    // 230800004-006: cockpit PTT switches. Confirm 40 s is the FCOM's own
    // real figure (p.4816, E-AIR-FCOM.json), not the 60 s guessed by
    // analogy the design sheet first floated.
    for (id, title, var_name) in [
        (230_800_004u64, "COM CAPT PTT STUCK", "DEEP_COM_CAPT_PTT_STUCK"),
        (230_800_005, "COM F/O PTT STUCK", "DEEP_COM_FO_PTT_STUCK"),
        (230_800_006, "COM THIRD OCCUPANT PTT STUCK", "DEEP_COM_THIRD_PTT_STUCK"),
    ] {
        v.push(
            proc(id, title, Level::Caution, sd_page::STATUS, gated(var(var_name).gt(0.0)), "FCOM PRO-ABN-ECAM p.4816: \"stuck in the transmit position for more than 40 s, and no transmission key is selected\"")
                .confirm(40.0)
                .inhibit(&[3, 4, 5, 6, 7, 9, 10])
                .items(0, Vec::new()),
        );
    }

    // 230800007: ATSU/datalink router fault.
    v.push(binary(
        230_800_007,
        "COM DATALINK FAULT",
        Level::Caution,
        sd_page::STATUS,
        "DEEP_COM_DATALINK_FAULT",
        &[3, 4, 5, 6, 7, 9, 10],
        2,
        "communications's own ATSU/datalink router component",
    ));

    // 230800008-011: HF 1/2 datalink / stuck-emitting.
    v.push(binary(230_800_008, "COM HF 1 DATALINK FAULT", Level::Advisory, sd_page::STATUS, "DEEP_COM_HF1_DATALINK_FAULT", &[3, 4, 5, 6, 7, 9, 10], 0, "communications's own HF 1 transceiver component"));
    v.push(binary(230_800_009, "COM HF 2 DATALINK FAULT", Level::Advisory, sd_page::STATUS, "DEEP_COM_HF2_DATALINK_FAULT", &[3, 4, 5, 6, 7, 9, 10], 0, "communications's own HF 2 transceiver component"));
    v.push(binary(230_800_010, "COM HF 1 EMITTING", Level::Caution, sd_page::STATUS, "DEEP_COM_HF1_EMITTING", &[3, 4, 5, 6, 7, 9, 10], 0, "communications's own HF 1 transceiver, stuck-emitting sub-fault"));
    v.push(binary(230_800_011, "COM HF 2 EMITTING", Level::Caution, sd_page::STATUS, "DEEP_COM_HF2_EMITTING", &[3, 4, 5, 6, 7, 9, 10], 0, "communications's own HF 2 transceiver, stuck-emitting sub-fault"));

    // 230800019-021: SATCOM datalink / main / voice.
    v.push(binary(230_800_019, "COM SATCOM DATALINK FAULT", Level::Advisory, sd_page::STATUS, "DEEP_COM_SATCOM_DATALINK_FAULT", &[3, 4, 5, 6, 7, 9, 10], 0, "communications's own SATCOM transceiver, datalink sub-fault"));
    v.push(binary(230_800_020, "COM SATCOM FAULT", Level::Advisory, sd_page::STATUS, "DEEP_COM_SATCOM_FAULT", &[3, 4, 5, 6, 7, 9, 10, 11], 0, "communications's own SATCOM transceiver, main-unit sub-fault"));
    v.push(binary(230_800_021, "COM SATCOM VOICE FAULT", Level::Advisory, sd_page::STATUS, "DEEP_COM_SATCOM_VOICE_FAULT", &[3, 4, 5, 6, 7, 9, 10, 11], 0, "communications's own SATCOM transceiver, voice-channel sub-fault"));

    // 230800022-025: VHF 1/2/3 stuck-emitting, VHF 3 datalink. Confirm 60 s
    // is the catalogue-sourced RMP TX KEY DESELECT figure the design sheet
    // cites for this family.
    for (id, title, var_name) in [
        (230_800_022u64, "COM VHF 1 EMITTING", "DEEP_COM_VHF1_EMITTING"),
        (230_800_023, "COM VHF 2 EMITTING", "DEEP_COM_VHF2_EMITTING"),
        (230_800_024, "COM VHF 3 EMITTING", "DEEP_COM_VHF3_EMITTING"),
    ] {
        v.push(
            proc(id, title, Level::Caution, sd_page::STATUS, gated(var(var_name).gt(0.0)), "communications's own VHF stuck-emitting component, extending (in doc comment only) radios.rs's real VHF tuning model; 60 s RMP TX KEY DESELECT is the catalogue's own figure")
                .confirm(60.0)
                .inhibit(&[3, 4, 5, 6, 7, 9, 10])
                .items(1, Vec::new()),
        );
    }
    v.push(binary(230_800_025, "COM VHF 3 DATALINK FAULT", Level::Advisory, sd_page::STATUS, "DEEP_COM_VHF3_DATALINK_FAULT", &[3, 4, 5, 6, 7, 9, 10], 0, "communications's own VHF 3 component, datalink sub-fault (VHF 3 is the dedicated ACARS-over-VHF datalink radio on the real aircraft)"));
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn wires_sixty_three_of_the_seventy_three() {
        let mut v = Vec::new();
        wire(&mut v);
        assert_eq!(v.len(), 63, "see this module's own doc comment for the 10 deliberately left unwired");
    }

    /// A hand-built variable map, not the plugin's real `Vars`: the four
    /// pending-write triggers below are wired against a variable name this
    /// area already publishes as a real passthrough of a `Truth` field that
    /// itself reads a SimVar FlyByWire has not written yet (see each
    /// trigger's own doc comment and `E:/fbw-debug/ecam/
    /// E-AIR-FBW-WRITES.md`). This proves the *trigger logic* is correct
    /// today, independently of whether the underlying write exists: once it
    /// does, the real plugin's own `Truth`-population reads it exactly the
    /// same way `prim_healthy` or `ap1_active` are already read, and the
    /// alert lights up with no further change here.
    #[test]
    fn the_pending_write_triggers_light_up_once_their_variable_carries_a_real_value() {
        let mut v = Vec::new();
        wire(&mut v);
        let find = |id: u64| v.iter().find(|p| p.id == id).unwrap().clone();

        let network_alive_map = |extra: &[(&str, f64)]| -> std::collections::BTreeMap<String, f64> {
            let mut m = std::collections::BTreeMap::new();
            m.insert("ELEC_AC_1_BUS_IS_POWERED".to_owned(), 1.0);
            for (k, val) in extra {
                m.insert((*k).to_owned(), *val);
            }
            m
        };

        // 211800045: healthy (no map entry, reads 0/false) is quiet; the
        // pending variable carrying 1.0 (as it will once FlyByWire writes
        // it) fires.
        let p = find(211_800_045);
        assert!(!p.trigger.eval(&|n| *network_alive_map(&[]).get(n).unwrap_or(&0.0)));
        assert!(p.trigger.eval(&|n| *network_alive_map(&[("DEEP_PNEU_PACK_FLOW_INSUFFICIENT_FWD_CRG", 1.0)]).get(n).unwrap_or(&0.0)));

        // 220800009 (ENG 1 A/THR OFF): needs both the real, already-bridged
        // aircraft-wide status AND the pending per-engine variable.
        let p = find(220_800_009);
        let armed_only = network_alive_map(&[("DEEP_AUTOFLT_ATHR_ARMED_OR_ACTIVE", 1.0)]);
        assert!(!p.trigger.eval(&|n| *armed_only.get(n).unwrap_or(&0.0)), "armed alone, no per-engine fault yet, must stay quiet");
        let armed_and_faulted = network_alive_map(&[("DEEP_AUTOFLT_ATHR_ARMED_OR_ACTIVE", 1.0), ("DEEP_AUTOFLT_ENG_1_ATHR_FAULT", 1.0)]);
        assert!(p.trigger.eval(&|n| *armed_and_faulted.get(n).unwrap_or(&0.0)), "once FlyByWire writes the per-engine fault, this must fire");
        // Engine 2's own procedure must not fire from engine 1's pending
        // variable.
        let p2 = find(220_800_010);
        assert!(!p2.trigger.eval(&|n| *armed_and_faulted.get(n).unwrap_or(&0.0)));
    }
}
