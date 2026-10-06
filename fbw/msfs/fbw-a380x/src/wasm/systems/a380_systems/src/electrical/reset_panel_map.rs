pub(crate) const RESET_PANEL_UNITS: &[(&str, &[&str])] = &[
    ("FMC_A", &["fms-1-normal-bkr", "fms-1-2nd-bkr"]),
    ("FMC_B", &["fms-2-normal-bkr", "fms-2-2nd-bkr"]),
    ("FMC_C", &["fms-3-normal-bkr", "fms-3-2nd-bkr"]),
    ("ATC", &["xpdr-1", "xpdr-2"]),
    ("LGCIS1", &["lgciu-1-normal-bkr", "lgciu-1-2nd-bkr"]),
    ("LGCIS2", &["lgciu-2-normal-bkr", "lgciu-2-2nd-bkr"]),
    ("PACK1_CTL", &["cpiom-b1-ags", "cpiom-b3-ags"]),
    ("PACK2_CTL", &["cpiom-b2-ags", "cpiom-b4-ags"]),
    ("TCS1", &["cpiom-b1-tcs"]),
    ("TCS2", &["cpiom-b2-tcs"]),
    ("VCS1", &["cpiom-b1-vcs"]),
    ("VCS2", &["cpiom-b2-vcs"]),
    ("CPCS1", &["cpiom-b1-cpcs"]),
    ("CPCS2", &["cpiom-b2-cpcs"]),
];

pub(crate) const RESET_PANEL_FAILURES: &[(&str, &[u64])] = &[
    ("TR1", &[24_000]),
    ("TR_2A", &[24_001]),
    ("ESS_TR", &[24_002]),
];

pub(super) const UNMAPPED: &[&str] = &[
    "AESU1", "AESU2", "AICU1", "AICU2", "ARPT_NAV", "AVS1", "AVS2", "BSCS1", "BSCS2", "CIDS1",
    "CIDS2", "CIDS3", "DSMS", "DTLNKROUTER", "ENG1_EIPM2", "ENG2_EIPM1", "ENG3_EIPM2",
    "ENG4_EIPM1", "FLAPS1", "FLAPS2", "FQMS1", "FQMS2", "FWS1", "FWS2", "GCU", "NSS_AVNCS",
    "NSS_FLT_OPS", "PAX_BBAND", "SCS1", "SCS2", "SDF1", "SDF2", "SDF3", "SLAT2", "SLATS1",
];

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::BTreeSet;

    const PANEL: [&str; 52] = [
        "AESU1", "AESU2", "AICU1", "AICU2", "ARPT_NAV", "ATC", "AVS1", "AVS2", "BSCS1", "BSCS2",
        "CIDS1", "CIDS2", "CIDS3", "CPCS1", "CPCS2", "DSMS", "DTLNKROUTER", "ENG1_EIPM2",
        "ENG2_EIPM1", "ENG3_EIPM2", "ENG4_EIPM1", "ESS_TR", "FLAPS1", "FLAPS2", "FMC_A", "FMC_B",
        "FMC_C", "FQMS1", "FQMS2", "FWS1", "FWS2", "GCU", "LGCIS1", "LGCIS2", "NSS_AVNCS",
        "NSS_FLT_OPS", "PACK1_CTL", "PACK2_CTL", "PAX_BBAND", "SCS1", "SCS2", "SDF1", "SDF2",
        "SDF3", "SLAT2", "SLATS1", "TCS1", "TCS2", "TR1", "TR_2A", "VCS1", "VCS2",
    ];

    #[test]
    fn every_button_is_accounted_for_exactly_once() {
        let mut seen = BTreeSet::new();
        for name in RESET_PANEL_UNITS
            .iter()
            .map(|(n, _)| *n)
            .chain(RESET_PANEL_FAILURES.iter().map(|(n, _)| *n))
            .chain(UNMAPPED.iter().copied())
        {
            assert!(seen.insert(name), "{name} twice");
        }
        assert_eq!(seen, PANEL.iter().copied().collect::<BTreeSet<_>>());
    }

    #[test]
    fn every_mapped_unit_exists() {
        let ids: BTreeSet<&str> = deep_systems::deep::breakers::catalog::all().iter().map(|b| b.id).collect();
        for (name, units) in RESET_PANEL_UNITS {
            for u in *units {
                assert!(ids.contains(u), "{name} maps to {u}, not a catalogue unit");
            }
        }
    }
}
