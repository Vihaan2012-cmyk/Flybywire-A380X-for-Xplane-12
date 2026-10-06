use crate::gates::Gate;

pub const GATES_JS: &[Gate] = &[
    Gate { variable: "ELEC_CAPT_EWD_DU_BREAKER_OPEN", units: &["capt-ewd-du"] },
    Gate { variable: "ELEC_CAPT_ND_DU_BREAKER_OPEN", units: &["capt-nd-du-normal-bkr", "capt-nd-du-2nd-bkr"] },
    Gate { variable: "ELEC_CAPT_PFD_DU_BREAKER_OPEN", units: &["capt-pfd-du"] },
    Gate { variable: "ELEC_FO_ND_DU_BREAKER_OPEN", units: &["fo-nd-du-normal-bkr", "fo-nd-du-2nd-bkr"] },
    Gate { variable: "ELEC_FO_PFD_DU_BREAKER_OPEN", units: &["fo-pfd-du"] },
];
