#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum MelCategory {
    A,
    B,
    C,
    D,
}

impl MelCategory {
    pub fn interval_hours(self) -> f64 {
        match self {
            MelCategory::A => 24.0,
            MelCategory::B => 3.0 * 24.0,
            MelCategory::C => 10.0 * 24.0,
            MelCategory::D => 120.0 * 24.0,
        }
    }

    pub fn label(self) -> &'static str {
        match self {
            MelCategory::A => "A",
            MelCategory::B => "B",
            MelCategory::C => "C",
            MelCategory::D => "D",
        }
    }
}

pub const GENERIC_CATEGORY: MelCategory = MelCategory::C;

pub const MEL_FAILURES: &[(&str, &[u64])] = &[
    ("21-21-01", &[21_001, 21_002, 21_003, 21_004]),
    ("21-28-02", &[21_009]),
    ("21-28-06", &[21_007]),
    ("21-60-04", &[21_005, 21_006]),
    ("21-60-11", &[21_011]),
    ("22-80-02", &[22_002]),
    ("22-80-03", &[22_003]),
    ("24-21-01", &[24_020, 24_021, 24_022, 24_023]),
    ("24-23-01", &[24_030, 24_031]),
    ("24-32-02", &[24_003]),
    ("26-10-01", &[26_017, 26_018]),
    ("26-10-02", &[26_007, 26_008, 26_009, 26_010, 26_011, 26_012, 26_013, 26_014]),
    ("26-10-03", &[26_015, 26_016]),
    ("27-93-01", &[27_000]),
    ("27-93-02", &[27_001]),
    ("27-93-03", &[27_002]),
    ("27-94-01", &[27_003]),
    ("27-94-02", &[27_004]),
    ("27-94-03", &[27_005]),
    ("27-96-01", &[27_007]),
    ("28-26-06", &[28_000]),
    ("28-26-07", &[28_001]),
    ("28-26-10", &[28_002]),
    ("28-26-13", &[28_003]),
    ("28-26-12", &[28_004]),
    ("28-26-11", &[28_005]),
    ("28-26-14", &[28_006]),
    ("28-26-15", &[28_007]),
    ("28-25-03", &[28_011]),
    ("28-27-01", &[28_008]),
    ("28-25-01", &[28_120, 28_121]),
    ("28-25-02", &[28_122]),
    ("28-25-04", &[28_123, 28_126]),
    ("28-25-05", &[28_127, 28_130]),
    ("28-25-06", &[28_124, 28_125]),
    ("28-25-07", &[28_128, 28_129]),
    ("28-25-08", &[28_131, 28_132]),
    ("28-25-09", &[28_133, 28_134]),
    ("28-25-10", &[28_135, 28_136]),
    ("28-25-11", &[28_137, 28_138]),
    ("28-25-12", &[28_139, 28_140]),
    ("28-25-13", &[28_141, 28_142]),
    ("28-25-14", &[28_143]),
    ("28-25-19", &[28_144, 28_145]),
    ("28-26-01", &[28_100, 28_101]),
    ("29-21-01", &[29_006, 29_007]),
    ("29-21-02", &[29_008, 29_009]),
    ("34-11-05", &[30_000, 30_001, 30_002]),
    ("34-11-01", &[34_100, 34_101, 34_102]),
    ("34-11-02", &[34_103, 34_104, 34_105]),
    ("34-42-01", &[34_000, 34_001, 34_002]),
    ("36-11-05", &[36_008, 36_009, 36_010, 36_011]),
    ("36-11-01", &[36_012, 36_013, 36_014, 36_015]),
    ("36-13-01", &[36_017]),
    ("36-13-02", &[36_016, 36_018]),
    ("21-50-02", &[21_050, 21_051]),
    ("21-50-03", &[21_052, 21_053]),
    ("49-10-01", &[49_002]),
    ("49-20-01", &[28_110]),
    ("74-31-01", &[74_000, 74_001, 74_002, 74_003]),
    ("78-30-04", &[78_000, 78_001, 78_002, 78_003]),
    ("80-11-01", &[80_000, 80_001, 80_002, 80_003]),
];

pub fn failures_for(mel_ref: &str) -> &'static [u64] {
    let item = mel_ref.get(..8).unwrap_or(mel_ref);
    MEL_FAILURES.iter().find(|(r, _)| *r == item).map_or(&[], |(_, f)| *f)
}

pub fn mel_ref_for(failure_id: u64) -> Option<&'static str> {
    MEL_FAILURES.iter().find(|(_, ids)| ids.contains(&failure_id)).map(|(r, _)| *r)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn category_intervals_match_the_mmel_policy_bands() {
        assert_eq!(MelCategory::B.interval_hours(), 72.0);
        assert_eq!(MelCategory::C.interval_hours(), 240.0);
        assert_eq!(MelCategory::D.interval_hours(), 2880.0);
    }

    #[test]
    fn failures_for_matches_a_full_sub_item_id_or_the_bare_item() {
        assert_eq!(failures_for("49-10-01"), &[49_002]);
        assert_eq!(failures_for("49-10-01A"), &[49_002]);
        assert!(failures_for("00-00-00").is_empty());
    }

    #[test]
    fn mel_ref_for_is_the_reverse_of_failures_for() {
        assert_eq!(mel_ref_for(49_002), Some("49-10-01"));
        assert_eq!(mel_ref_for(24_021), Some("24-21-01"));
        assert_eq!(mel_ref_for(u64::MAX), None);
    }

    #[test]
    fn no_failure_id_is_listed_under_two_mel_items() {
        let mut seen = std::collections::BTreeMap::new();
        for &(item, ids) in MEL_FAILURES {
            for &id in ids {
                if let Some(&first) = seen.get(&id) {
                    panic!("{id} is listed under both {first} and {item}");
                }
                seen.insert(id, item);
            }
        }
    }
}
