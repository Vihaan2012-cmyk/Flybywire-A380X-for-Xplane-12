use crate::deep::api::EcamAlert;

pub const ID_BASE: u64 = 1_000_000_000;

#[derive(Clone, Debug)]
pub struct Assigned<'a> {
    pub id: u64,
    pub alert: &'a EcamAlert,
}

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
