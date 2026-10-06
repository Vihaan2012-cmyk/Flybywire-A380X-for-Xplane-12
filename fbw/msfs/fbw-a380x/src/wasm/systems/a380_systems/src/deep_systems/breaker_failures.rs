pub(super) const BREAKER_FAILURES: &[(&[&str], &[u64])] = &[
    (&["hotair-1"], &[21_005]),
    (&["hotair-2"], &[21_006]),
    (&["fwd-isol-valve"], &[21_007]),
    (&["fwd-extract-fan"], &[21_008]),
    (&["bulk-isol-valve"], &[21_009]),
    (&["bulk-extract-fan"], &[21_010]),
    (&["cargo-heater"], &[21_011]),
    (&["ocsm-1-ap"], &[21_022]),
    (&["ocsm-2-ap"], &[21_023]),
    (&["ocsm-3-ap"], &[21_024]),
    (&["ocsm-4-ap"], &[21_025]),
    (&["cpiom-b1-ags"], &[21_034]),
    (&["cpiom-b2-ags"], &[21_035]),
    (&["cpiom-b3-ags"], &[21_036]),
    (&["cpiom-b4-ags"], &[21_037]),
    (&["cpiom-b1-tcs"], &[21_038]),
    (&["cpiom-b2-tcs"], &[21_039]),
    (&["cpiom-b3-tcs"], &[21_040]),
    (&["cpiom-b4-tcs"], &[21_041]),
    (&["cpiom-b1-vcs"], &[21_042]),
    (&["cpiom-b2-vcs"], &[21_043]),
    (&["cpiom-b3-vcs"], &[21_044]),
    (&["cpiom-b4-vcs"], &[21_045]),
    (&["cpiom-b1-cpcs"], &[21_046]),
    (&["cpiom-b2-cpcs"], &[21_047]),
    (&["cpiom-b3-cpcs"], &[21_048]),
    (&["cpiom-b4-cpcs"], &[21_049]),

    (&["gear-actuator-nose"], &[32_020]),
    (&["gear-actuator-left"], &[32_021]),
    (&["gear-actuator-right"], &[32_022]),
    (&["gear-door-actuator-nose"], &[32_023]),
    (&["gear-door-actuator-left"], &[32_024]),
    (&["gear-door-actuator-right"], &[32_025]),
    (&["prox-uplock-gear-nose-1"], &[32_004]),
    (&["prox-downlock-gear-nose-2"], &[32_005]),
    (&["prox-uplock-gear-right-1"], &[32_006]),
    (&["prox-downlock-gear-right-2"], &[32_007]),
    (&["prox-uplock-gear-left-2"], &[32_008]),
    (&["prox-downlock-gear-left-1"], &[32_009]),
    (&["prox-uplock-door-nose-1"], &[32_010]),
    (&["prox-downlock-door-nose-2"], &[32_011]),
    (&["prox-uplock-door-right-2"], &[32_012]),
    (&["prox-downlock-door-right-1"], &[32_013]),
    (&["prox-uplock-door-left-2"], &[32_014]),
    (&["prox-downlock-door-left-1"], &[32_015]),

    (&["ra-ant-interrupt-1"], &[34_010]),
    (&["ra-ant-interrupt-2"], &[34_011]),
    (&["ra-ant-interrupt-3"], &[34_012]),
    (&["ra-ant-coupling-1"], &[34_020]),
    (&["ra-ant-coupling-2"], &[34_021]),
    (&["ra-ant-coupling-3"], &[34_022]),
];

pub(super) fn active<'a>(units: impl Iterator<Item = (&'a str, bool)>) -> Vec<u64> {
    let open: std::collections::HashSet<&str> = units.filter(|(_, open)| *open).map(|(id, _)| id).collect();
    BREAKER_FAILURES
        .iter()
        .filter(|(feeds, _)| !feeds.is_empty() && feeds.iter().all(|u| open.contains(u)))
        .flat_map(|(_, ids)| ids.iter().copied())
        .collect()
}
