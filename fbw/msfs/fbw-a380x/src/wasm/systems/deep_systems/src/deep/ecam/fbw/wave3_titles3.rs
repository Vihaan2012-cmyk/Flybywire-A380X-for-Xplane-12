use super::{phase, proc, sd_page, FbwProc};
use crate::deep::api::{all, any, var, Level};

pub fn procs() -> Vec<FbwProc> {
    vec![
        proc(
            701_800_141,
            "ENG 2 REVERSER FAULT",
            Level::Caution,
            sd_page::ENG,
            var("A32NX_ENG_2_REV_POSITION_DISAGREE").on(),
            "FCOM PRO-ABN-ECAM p.5826 ENG 2(3) REVERSER FAULT",
        )
        .inhibit(&[phase::SECOND_ENG_TO_POWER, phase::AT_OR_ABOVE_80_KT, phase::AT_OR_ABOVE_V1, phase::LIFT_OFF, phase::AT_OR_ABOVE_400_FT])
        .items(0, Vec::new()),
        proc(
            701_800_149,
            "ENG 2 REVERSER UNLOCKED",
            Level::Caution,
            sd_page::ENG,
            var("A32NX_ENG_2_REV_UNCOMMANDED").on(),
            "FCOM PRO-ABN-ECAM p.5831 ENG 2(3) REVERSER UNLOCKED",
        )
        .inhibit(phase::ENG_56)
        .items(0, Vec::new()),
        proc(
            701_800_114,
            "ENG 2 STALL",
            Level::Caution,
            sd_page::ENG,
            var("ENV_HAIL_FLAMEOUT_RISK:2").gt(0.6),
            "FCOM PRO-ABN-ECAM p.5808 ENG 1(2)(3)(4) STALL",
        )
        .inhibit(phase::ENG_56)
        .items(0, Vec::new()),
        proc(
            701_800_034,
            "ENG 2 FUEL FILTER CLOGGED",
            Level::Advisory,
            sd_page::ENG,
            var("A32NX_ENG_2_FUEL_FILTER_IMPENDING_BYPASS").on(),
            "FCOM PRO-ABN-ECAM p.5784 ENG 1(2)(3)(4) FUEL FILTER CLOGGED",
        )
        .inhibit(&[
            phase::SECOND_ENG_TO_POWER,
            phase::AT_OR_ABOVE_80_KT,
            phase::AT_OR_ABOVE_V1,
            phase::LIFT_OFF,
            phase::AT_OR_ABOVE_400_FT,
            phase::AT_OR_ABOVE_1500_FT,
            phase::AT_OR_BELOW_800_FT,
            phase::TOUCH_DOWN,
        ])
        .items(0, Vec::new()),
        proc(
            701_800_035,
            "ENG 3 FUEL FILTER CLOGGED",
            Level::Advisory,
            sd_page::ENG,
            var("A32NX_ENG_3_FUEL_FILTER_IMPENDING_BYPASS").on(),
            "FCOM PRO-ABN-ECAM p.5784 ENG 1(2)(3)(4) FUEL FILTER CLOGGED",
        )
        .inhibit(&[
            phase::SECOND_ENG_TO_POWER,
            phase::AT_OR_ABOVE_80_KT,
            phase::AT_OR_ABOVE_V1,
            phase::LIFT_OFF,
            phase::AT_OR_ABOVE_400_FT,
            phase::AT_OR_ABOVE_1500_FT,
            phase::AT_OR_BELOW_800_FT,
            phase::TOUCH_DOWN,
        ])
        .items(0, Vec::new()),
    ]
}
