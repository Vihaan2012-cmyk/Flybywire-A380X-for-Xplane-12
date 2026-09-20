//! ATA 21/22/23 -- air conditioning, autoflight, communications. FlyByWire's
//! `AbnormalSensed/ata21-22-23.ts` defines the procedures below as unwired
//! (not present in `FwsAbnormalSensed.ts`'s `ewdAbnormalSensed` map); this
//! module's job was to find which of them this port's own areas can raise
//! without inventing a trigger. It wires **none**, and this doc comment is
//! the record of why each of the 73 was refused rather than a silent gap.
//!
//! # ATA 22 (autoflight) and 23 (comms) -- structurally unwirable, all 30
//!
//! `220800002/005-012/014/015`, `221800010` (12 ids) and `230800001-011,
//! 019-025` (18 ids) each name a specific autoflight or communications
//! *computer* -- AFS CTL PNL, FCU, TCAS mode logic, A/THR per engine, FMC/FMS
//! switching, CIDS, RMP 1/2/3, HF/VHF/SATCOM radios and datalink. No `deep::`
//! area models the AFS computers or the comms radios (`deep::published_names`
//! carries no `AFS_*`, `FMGC_*`, `RMP_*`, `VHF_*`, `HF_*`, `SATCOM_*` or
//! `CIDS_*` names), so every trigger here would have to be invented. Left
//! unwired, exactly as `docs/deep/fbw_unwired.md` predicted for this
//! grouping.
//!
//! # ATA 21 (air conditioning) -- 43 ids, none wirable without breaking a
//! rule
//!
//! The other 43 unwired ids are all ATA 21 (`211800013-020,022,024,028,030,
//! 034,036-038,045,056`; `212800006,012-028`; `213800003,007,008,010,015,
//! 016,018`). `deep::thermal_zones`, `deep::pneumatic_ducts` and the sensors
//! area (`DEEP_CPC_*`) do publish real numbers here, so each was checked
//! individually against the two rules that make a wiring worth anything --
//! and every one fails at least one of them:
//!
//! * **Would double-annunciate.** `212800006` AVNCS VENT CTL FAULT,
//!   `212800014` VENT AVNCS BLOWING FAULT, `212800015` VENT AVNCS EXTRACT
//!   FAULT and `212800016/017` VENT AVNCS L/R BLOWING FAULT all sit on the
//!   same avionics-bay airflow condition `deep::registry()` already
//!   announces as `AVNCS_AVIONICS_BAY_AFT_VENT_FAULT` /
//!   `AVNCS_AVIONICS_BAY_FWD_VENT_FAULT` (`AVNCS_AVIONICS_BAY_{AFT,FWD}_
//!   AIRFLOW_FRAC`). `212800025/026/027` PACK BAY 1/2/1+2 VENT FAULT sit on
//!   the same zone temperature our own `ECS_PACK_BAY_OVHT`
//!   (`THERMAL_ZONE_BELLYFAIRINGPACKS_TEMPERATURE_C`) already fires from --
//!   in this model a lost pack-bay vent *is* the pack-bay overheat, so a
//!   second id on the same reading is the double warning this whole effort
//!   exists to avoid.
//! * **No sourced threshold exists to reuse, and inventing one is exactly
//!   what the brief forbids.** `deep::pneumatic_ducts` publishes
//!   `DEEP_DUCT_TEMP_PACK_{1,2}_SUPPLY_DUCT_C` -- a real number -- for
//!   `211800028` COND DUCT OVHT and `211800024` BULK CARGO DUCT OVHT, but
//!   nowhere in this port is there a sourced pneumatic duct overheat limit
//!   (unlike the engine/APU precoolers, which already publish their own
//!   verdict as `DEEP_PNEU_*_PRECOOLER_OVHT`, or the AC bus voltage floor
//!   ATA 24 reused from `deep::electrical::registry`). Likewise
//!   `deep::sensors` publishes `DEEP_CPC_{1,2}_DIFF_PRESSURE_SENSED_PA` as a
//!   raw reading with no verdict anywhere in `deep::registry()` built on it,
//!   so `213800007` DIFF PRESS HI and `213800008` DIFF PRESS LO have no
//!   limit to borrow -- FlyByWire's own `213800002` EXCESS DIFF PRESS is
//!   sensed entirely on the JS side and this port cannot see what number it
//!   uses. Picking a number ourselves would be the invented threshold this
//!   module and ATA 24's own doc comment both refuse. `213800003` EXCESS
//!   NEGATIVE DIFF PRESS is the same problem in the other direction.
//! * **Names a controller/regulation state nothing models.** `211800013/014`
//!   PACK 1/2 OVHT, `211800015/016` PACK 1/2 REGUL FAULT, `211800017-020`
//!   PACK VLV FAULTs, `211800022` PACK 1+2 REGUL REDUNDANCY FAULT,
//!   `211800030` FWD CARGO TEMP REGUL FAULT, `211800034` MIXER PRESS REGUL
//!   FAULT, `211800036` PURSER TEMP SEL FAULT, `211800037/038` RAM AIR 1/2
//!   FAULT, `211800045` PACK REGUL DEGRADED, `211800056` ABNORM BLEED CONFIG
//!   (a crew-actioned procedure with no clean single condition of its own),
//!   `212800012/013` (PARTIAL/)SECONDARY CABIN FANS FAULT, `212800018` VENT
//!   AVNCS OVBD VLV FAULT, `212800019/020` VENT COOLG SYS 1/2 OVHT (no
//!   `COOLG` name published at all), `212800021` COOLG SYS PROT FAULT,
//!   `212800022/023` IFE BAY ISOL/VENT FAULT, `212800024` LAV & GALLEYS
//!   EXTRACT FAULT, `212800028` THS BAY VENT FAULT (no THS thermal zone
//!   published), `213800010` MAN CTL FAULT, `213800015` OUTFLW VLV CTL
//!   FAULT, `213800016` SENSORS FAULT and `213800018` CABIN AIR EXTRACT VLV
//!   FAULT: none of these names a quantity `deep::pneumatic_ducts`,
//!   `deep::thermal_zones` or `deep::sensors` computes. The pack, valve,
//!   fan, cooling-system and outflow-valve *controllers* are not modelled at
//!   this resolution -- only the ducts, zones and (for pressurization) the
//!   raw CPC readings are.
//!
//! Net: **0 of 73 wired.** `docs/deep/fbw_unwired.md`'s own prediction for
//! this pass -- "ATA 21/22/23 (73): ... No area models the AFS or the comms
//! radios" for 22/23, and "the tighter window" of ATA 21 controller/limit
//! gaps -- held up under an id-by-id check. Wiring any of the near-misses
//! above on a guessed threshold or a shared reading would have produced a
//! second alert that either never suppresses correctly against our own
//! registry or fires on a number nobody sourced; per this module's own
//! rule and `ata24.rs`'s, an alert that fires on an approximation is worse
//! than one that does not fire.

use super::FbwProc;

/// Wires nothing; see this module's doc comment for the id-by-id reasons.
pub fn wire(_v: &mut Vec<FbwProc>) {}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn wires_nothing_because_every_candidate_fails_a_rule() {
        let mut v = Vec::new();
        wire(&mut v);
        assert!(
            v.is_empty(),
            "ATA 21/22/23 has no id that clears both the no-double-annunciation and \
             no-invented-trigger rules; see this module's doc comment for the id-by-id survey"
        );
    }
}
