//! Numeric ids for injected alerts, in a range FlyByWire's own procedure
//! ids can never reach.
//!
//! FlyByWire's own convention (fbw-a380x AbnormalSensed/ata*.ts doc
//! comments, e.g. `ata29-30.ts:6-17`): a 9-digit id, `ATA(2 digits) +
//! sub-chapter(1) + kind(1) + sequence(3)`. The largest possible value of
//! that shape is 999_999_999 (ATA "99", sub 9, kind 9, seq 999) --
//! confirmed against the actual compiled catalogue, whose own largest ids
//! are the `9998NNNNN` "secondary failure" entries
//! (`SystemsHost.js:172521`, `999800005`..`999800007`). Any id at or above
//! 1_000_000_000 (ten digits) is therefore outside that whole id space by
//! construction, not merely by observation of what happens to be used
//! today.

use crate::deep::api::EcamAlert;

/// The first id this bridge ever hands out.
pub const ID_BASE: u64 = 1_000_000_000;

/// One alert with the numeric id FlyByWire's tables key it by. `EcamAlert`
/// derives only `Clone, Debug` (`api.rs:263`), not `PartialEq`, so this
/// does not derive it either.
#[derive(Clone, Debug)]
pub struct Assigned<'a> {
    pub id: u64,
    pub alert: &'a EcamAlert,
}

/// Assigns every alert a ten-digit id, `ID_BASE + i`, `i` its position after
/// sorting by `key` -- deterministic regardless of registration order (areas
/// register in whatever order the lead calls them in), and stable across a
/// run as long as the set of keys does not change. Two areas registering
/// the same `key` is already rejected by `Registry::alert` (a duplicate is
/// recorded in `Registry::errors`); this function does not re-check it.
pub fn assign(alerts: &[EcamAlert]) -> Vec<Assigned<'_>> {
    let mut sorted: Vec<&EcamAlert> = alerts.iter().collect();
    sorted.sort_by(|a, b| a.key.cmp(&b.key));
    sorted.into_iter().enumerate().map(|(i, alert)| Assigned { id: ID_BASE + i as u64, alert }).collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::deep::api::{var, EcamAlert, Level};

    fn alert(key: &str) -> EcamAlert {
        EcamAlert::new(key, 21, key, Level::Caution, var("X").on())
    }

    #[test]
    fn every_id_is_outside_flybywires_nine_digit_space_and_unique() {
        let alerts = vec![alert("C"), alert("A"), alert("B")];
        let assigned = assign(&alerts);
        assert_eq!(assigned.len(), 3);
        for a in &assigned {
            assert!(a.id >= ID_BASE, "id {} is not >= ID_BASE", a.id);
            assert!(a.id > 999_999_999, "id {} could collide with a 9-digit FlyByWire id", a.id);
        }
        let mut ids: Vec<u64> = assigned.iter().map(|a| a.id).collect();
        ids.sort_unstable();
        ids.dedup();
        assert_eq!(ids.len(), 3, "ids must be unique");
    }

    #[test]
    fn assignment_is_deterministic_regardless_of_registration_order() {
        let in_order = vec![alert("A"), alert("B"), alert("C")];
        let reversed = vec![alert("C"), alert("B"), alert("A")];
        let by_key = |v: &[Assigned<'_>]| -> Vec<(String, u64)> { v.iter().map(|a| (a.alert.key.clone(), a.id)).collect() };
        assert_eq!(by_key(&assign(&in_order)), by_key(&assign(&reversed)));
    }
}
