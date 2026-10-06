use crate::deep::api::*;

const FAN_BIRD_DAMAGE_FAILURE_IDS: [u64; 4] = [14_072_001, 14_072_008, 14_072_009, 14_072_010];
const CORE_FOD_FAILURE_IDS: [u64; 4] = [14_072_002, 14_072_011, 14_072_012, 14_072_013];

pub fn register(_r: &mut Registry) {
    let _ = (FAN_BIRD_DAMAGE_FAILURE_IDS, CORE_FOD_FAILURE_IDS);
}
