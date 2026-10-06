use std::cell::RefCell;
use std::collections::BTreeMap;
#[cfg(test)]
use std::sync::Mutex;

thread_local! {
    static DEFERRED: RefCell<BTreeMap<u64, f64>> = const { RefCell::new(BTreeMap::new()) };
}

pub fn set_deferred(remaining_hours: BTreeMap<u64, f64>) {
    DEFERRED.with(|d| *d.borrow_mut() = remaining_hours);
}

pub fn deferred_state(id: u64) -> Option<f64> {
    DEFERRED.with(|d| d.borrow().get(&id).copied())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn serial() -> std::sync::MutexGuard<'static, ()> {
        static LOCK: Mutex<()> = Mutex::new(());
        LOCK.lock().unwrap_or_else(|e| e.into_inner())
    }

    #[test]
    fn nothing_is_deferred_until_the_host_says_so() {
        let _s = serial();
        set_deferred(BTreeMap::new());
        assert_eq!(deferred_state(78_001), None);
    }

    #[test]
    fn a_deferred_id_reads_back_its_remaining_hours_and_clears_when_dropped() {
        let _s = serial();
        let mut m = BTreeMap::new();
        m.insert(78_001, 239.5);
        set_deferred(m);
        assert_eq!(deferred_state(78_001), Some(239.5));
        assert_eq!(deferred_state(78_002), None, "only what the host set is deferred");

        set_deferred(BTreeMap::new());
        assert_eq!(deferred_state(78_001), None, "repairing it (host stops sending it) clears it");
    }
}
