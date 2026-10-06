use std::collections::BTreeMap;

use deep_systems::mel_catalog::{self, MEL_FAILURES};
use systems::simulation::{InitContext, Reader, SimulatorReader, SimulatorWriter, VariableIdentifier, Write};

const ITEM_COUNT: usize = MEL_FAILURES.len();

const RESULT_NONE: f64 = 0.;
const RESULT_APPLIED: f64 = 1.;
const RESULT_UNKNOWN: f64 = 3.;

struct Deferral {
    expires_at_hours: f64,
}

pub struct Mel {
    elapsed_hours: f64,
    deferred: BTreeMap<usize, Deferral>,

    cmd_item_id: VariableIdentifier,
    cmd_action_id: VariableIdentifier,
    cmd_result_id: VariableIdentifier,
    item_deferred_ids: [VariableIdentifier; ITEM_COUNT],
    item_remaining_hours_ids: [VariableIdentifier; ITEM_COUNT],

    command: Option<(i64, i64)>,
    consumed: bool,
    result: f64,
}

impl Mel {
    pub fn new(context: &mut InitContext) -> Self {
        Self {
            elapsed_hours: 0.,
            deferred: BTreeMap::new(),
            cmd_item_id: context.get_identifier("DEEP_MEL_CMD_ITEM".to_owned()),
            cmd_action_id: context.get_identifier("DEEP_MEL_CMD".to_owned()),
            cmd_result_id: context.get_identifier("DEEP_MEL_CMD_RESULT".to_owned()),
            item_deferred_ids: std::array::from_fn(|k| context.get_identifier(format!("DEEP_MEL_ITEM_{k}_DEFERRED"))),
            item_remaining_hours_ids: std::array::from_fn(|k| context.get_identifier(format!("DEEP_MEL_ITEM_{k}_REMAINING_HOURS"))),
            command: None,
            consumed: false,
            result: RESULT_NONE,
        }
    }

    #[cfg(test)]
    fn new_for_test() -> Self {
        Self {
            elapsed_hours: 0.,
            deferred: BTreeMap::new(),
            cmd_item_id: VariableIdentifier::new(0usize),
            cmd_action_id: VariableIdentifier::new(0usize),
            cmd_result_id: VariableIdentifier::new(0usize),
            item_deferred_ids: std::array::from_fn(|_| VariableIdentifier::new(0usize)),
            item_remaining_hours_ids: std::array::from_fn(|_| VariableIdentifier::new(0usize)),
            command: None,
            consumed: false,
            result: RESULT_NONE,
        }
    }

    pub fn read(&mut self, reader: &mut SimulatorReader) {
        let action = reader.read_f64(&self.cmd_action_id).round() as i64;
        self.command = if action != 0 { Some((reader.read_f64(&self.cmd_item_id).round() as i64, action)) } else { None };
        self.consumed = self.command.is_some();
    }

    pub fn restore_deferred(&mut self, item: usize) {
        if item < ITEM_COUNT {
            self.deferred.entry(item).or_insert_with(|| Deferral { expires_at_hours: mel_catalog::GENERIC_CATEGORY.interval_hours() });
        }
    }

    pub fn update(&mut self, delta_hours: f64) {
        self.elapsed_hours += delta_hours;

        if let Some((item, action)) = self.command.take() {
            let valid_item = usize::try_from(item).ok().filter(|&i| i < ITEM_COUNT);
            self.result = match (valid_item, action) {
                (Some(item), 1) => {
                    self.deferred.entry(item).or_insert_with(|| Deferral { expires_at_hours: self.elapsed_hours + mel_catalog::GENERIC_CATEGORY.interval_hours() });
                    RESULT_APPLIED
                }
                (Some(item), 2) => {
                    self.deferred.remove(&item);
                    RESULT_APPLIED
                }
                _ => RESULT_UNKNOWN,
            };
        }

        let elapsed_hours = self.elapsed_hours;
        deep_systems::mel::set_deferred(
            self.deferred
                .iter()
                .flat_map(|(&item, d)| MEL_FAILURES[item].1.iter().map(move |&id| (id, d.expires_at_hours - elapsed_hours)))
                .collect(),
        );
    }

    pub fn write(&self, writer: &mut SimulatorWriter) {
        if self.consumed {
            writer.write(&self.cmd_item_id, 0.);
            writer.write(&self.cmd_action_id, 0.);
            writer.write(&self.cmd_result_id, self.result);
        }
        for k in 0..ITEM_COUNT {
            let remaining = self.deferred.get(&k).map(|d| d.expires_at_hours - self.elapsed_hours);
            writer.write(&self.item_deferred_ids[k], if remaining.is_some() { 1. } else { 0. });
            writer.write(&self.item_remaining_hours_ids[k], remaining.unwrap_or(0.));
        }
    }

    pub fn derived_failure_ids(&self) -> Vec<u64> {
        self.deferred.keys().flat_map(|&item| MEL_FAILURES[item].1.iter().copied()).collect()
    }

    pub fn is_deferred(&self, fbw_id: u64) -> bool {
        self.deferred.keys().any(|&item| MEL_FAILURES[item].1.contains(&fbw_id))
    }

    pub fn deferred_items(&self) -> impl Iterator<Item = usize> + '_ {
        self.deferred.keys().copied()
    }

    #[cfg(test)]
    pub(super) fn deferred_fbw_ids(&self) -> BTreeMap<u64, f64> {
        let elapsed_hours = self.elapsed_hours;
        self.deferred.iter().flat_map(|(&item, d)| MEL_FAILURES[item].1.iter().map(move |&id| (id, d.expires_at_hours - elapsed_hours))).collect()
    }
}

#[cfg(test)]
pub(super) fn serial_for_tests() -> std::sync::MutexGuard<'static, ()> {
    static LOCK: std::sync::Mutex<()> = std::sync::Mutex::new(());
    LOCK.lock().unwrap_or_else(|e| e.into_inner())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn item_for(fbw_id: u64) -> usize {
        MEL_FAILURES.iter().position(|(_, ids)| ids.contains(&fbw_id)).expect("fbw_id must be MEL-catalogued for this test")
    }

    #[test]
    fn deferring_an_item_activates_its_fbw_failure_ids_and_releasing_clears_them() {
        let _serial = serial_for_tests();
        let item = item_for(78_001);
        let mut mel = Mel::new_for_test();

        assert!(!mel.is_deferred(78_001));
        assert!(mel.derived_failure_ids().is_empty());

        mel.command = Some((item as i64, 1));
        mel.update(1.0);
        assert!(mel.is_deferred(78_001), "deferring the item activates every fbw id it covers");
        assert!(mel.derived_failure_ids().contains(&78_001));
        assert_eq!(mel.deferred_fbw_ids()[&78_001], mel_catalog::GENERIC_CATEGORY.interval_hours());
        assert_eq!(deep_systems::mel::deferred_state(78_001), Some(mel_catalog::GENERIC_CATEGORY.interval_hours()));

        mel.command = Some((item as i64, 2));
        mel.update(1.0);
        assert!(!mel.is_deferred(78_001), "releasing it clears the fbw id again");
        assert!(mel.derived_failure_ids().is_empty());
        assert_eq!(deep_systems::mel::deferred_state(78_001), None);
    }

    #[test]
    fn an_out_of_range_item_is_rejected() {
        let _serial = serial_for_tests();
        let mut mel = Mel::new_for_test();
        mel.command = Some((i64::try_from(ITEM_COUNT).unwrap(), 1));
        mel.update(1.0);
        assert_eq!(mel.result, RESULT_UNKNOWN);
        assert!(mel.derived_failure_ids().is_empty());
    }

    #[test]
    fn the_interval_counts_down_from_when_the_item_was_first_deferred() {
        let _serial = serial_for_tests();
        let item = item_for(78_001);
        let mut mel = Mel::new_for_test();

        mel.command = Some((item as i64, 1));
        mel.update(10.0);
        mel.update(200.0);
        assert!(mel.is_deferred(78_001));
        assert_eq!(mel.deferred_fbw_ids()[&78_001], 40.0);
        mel.update(45.0);
        assert!(mel.deferred_fbw_ids()[&78_001] < 0.0, "expired but still shown as such, not silently dropped");
    }

    #[test]
    fn restoring_a_persisted_item_defers_it_with_a_fresh_interval() {
        let _serial = serial_for_tests();
        let item = item_for(78_001);
        let mut mel = Mel::new_for_test();
        mel.restore_deferred(item);
        assert!(mel.is_deferred(78_001));
        assert_eq!(mel.deferred_fbw_ids()[&78_001], mel_catalog::GENERIC_CATEGORY.interval_hours());
    }
}
