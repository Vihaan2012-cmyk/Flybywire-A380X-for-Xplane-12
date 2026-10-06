use crate::gates::Gate;

pub const GATES_CPP: &[Gate] = &[
    Gate { variable: "ROLLOUT_BREAKER_OPEN", units: &["rollout-normal-bkr", "rollout-2nd-bkr"] },
    Gate { variable: "PRIM_1_BREAKER_OPEN", units: &["prim-1-normal-bkr", "prim-1-2nd-bkr"] },
    Gate { variable: "PRIM_2_BREAKER_OPEN", units: &["prim-2-normal-bkr", "prim-2-2nd-bkr"] },
    Gate { variable: "PRIM_3_BREAKER_OPEN", units: &["prim-3-normal-bkr", "prim-3-2nd-bkr"] },
    Gate { variable: "SEC_1_BREAKER_OPEN", units: &["sec-1-normal-bkr", "sec-1-2nd-bkr"] },
    Gate { variable: "SEC_2_BREAKER_OPEN", units: &["sec-2-normal-bkr", "sec-2-2nd-bkr"] },
    Gate { variable: "SEC_3_BREAKER_OPEN", units: &["sec-3-normal-bkr", "sec-3-2nd-bkr"] },
    Gate { variable: "FCDC_1_BREAKER_OPEN", units: &["fcdc-1-normal-bkr", "fcdc-1-2nd-bkr"] },
    Gate { variable: "FCDC_2_BREAKER_OPEN", units: &["fcdc-2-normal-bkr", "fcdc-2-2nd-bkr"] },
];
