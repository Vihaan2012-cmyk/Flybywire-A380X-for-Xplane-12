use super::{proc, sd_page, FbwProc};
use crate::deep::api::{any, var, Level};

pub fn procs() -> Vec<FbwProc> {
    vec![
        proc(
            701_800_113,
            "ENG 1 STALL",
            Level::Caution,
            sd_page::ENG,
            any(vec![var("A32NX_ENG_1_GASPATH_SURGE").on(), var("ENV_HAIL_FLAMEOUT_RISK:1").gt(0.6)]),
            "72_eng.compressor_stall_1: engine_accessories::live::step_gas_path_shadow feeds this failure's armed severity into the HP compressor's own efficiency/flow-capacity loss input; GASPATH_SURGE is the gas-path shadow's real stall-margin verdict, not a scripted trigger. Failures: [72004]; FCOM PRO-ABN-ECAM p.5808",
        )
        .inhibit(&[5, 6])
        .items(0, Vec::new()),
        proc(
            701_800_114,
            "ENG 2 STALL",
            Level::Caution,
            sd_page::ENG,
            var("A32NX_ENG_2_GASPATH_SURGE").on(),
            "72_eng.compressor_stall_2: see ENG 1 STALL (701800113). Failures: [72005]; FCOM PRO-ABN-ECAM p.5808",
        )
        .inhibit(&[5, 6])
        .items(0, Vec::new()),
        proc(
            701_800_115,
            "ENG 3 STALL",
            Level::Caution,
            sd_page::ENG,
            any(vec![var("A32NX_ENG_3_GASPATH_SURGE").on(), var("ENV_HAIL_FLAMEOUT_RISK:3").gt(0.6)]),
            "72_eng.compressor_stall_3: see ENG 1 STALL (701800113). Failures: [72006]; FCOM PRO-ABN-ECAM p.5808",
        )
        .inhibit(&[5, 6])
        .items(0, Vec::new()),
        proc(
            701_800_116,
            "ENG 4 STALL",
            Level::Caution,
            sd_page::ENG,
            any(vec![var("A32NX_ENG_4_GASPATH_SURGE").on(), var("ENV_HAIL_FLAMEOUT_RISK:4").gt(0.6)]),
            "72_eng.compressor_stall_4: see ENG 1 STALL (701800113). Failures: [72007]; FCOM PRO-ABN-ECAM p.5808",
        )
        .inhibit(&[5, 6])
        .items(0, Vec::new()),
    ]
}
