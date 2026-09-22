//! Minimum Equipment List (hyperrealism physics workstream 6: failures,
//! damage, MEL and persistence).
//!
//! The A/B/C/D repair-interval category scheme is the standard MMEL
//! structure used across FAA and EASA master minimum equipment lists (FAA
//! AC 25-1591 / EASA MMEL Policy: category A has no fixed interval and uses
//! the interval stated by the item itself, B = 3 calendar days, C = 10
//! calendar days, D = 120 calendar days, all excluding the day of
//! discovery). This module tracks that structure generically; individual
//! item text (the dispatch condition column) is only ever taken from
//! publicly available Airbus A380 MMEL excerpts when cited, and is
//! otherwise kept generic and marked as such below and in
//! `docs/physics/failures.md`, per the hyperrealism brief's sourcing rule.
//!
//! An item becomes deferrable the moment its failure id appears in
//! [`items()`]. Deferring keeps the failure active (the systems still see
//! it: nothing here suppresses `failures::active_ids()`) but marks it
//! dispatch-legal until the interval expires; expiry is tracked in flight
//! hours through `persistence.rs`'s round trip, converted from the
//! category's calendar-day interval using a fixed day length (24h, the
//! conservative reading: a "calendar day" MMEL interval keeps counting
//! whether or not the aircraft flies).


use std::collections::BTreeMap;

/// MMEL repair-interval category (FAA AC 25-1591 / EASA MMEL Policy).
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum MelCategory {
    /// Repair interval as stated by the specific item; this module uses the
    /// shortest standard bound (see `docs/physics/failures.md`) since no
    /// item here specifies its own.
    A,
    /// 3 calendar days.
    B,
    /// 10 calendar days.
    C,
    /// 120 calendar days.
    D,
}

impl MelCategory {
    /// The repair interval in hours, day-based per the MMEL policy
    /// (calendar days x 24h). Category A has no universal bound; this
    /// model uses 24h (the tightest of the standard bands) as the
    /// conservative default, clearly marked here rather than left unbound.
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

/// One deferred item, persisted (`persistence.rs`).
#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct Deferral {
    pub id: u64,
    /// Airframe hours (persistence.rs's running total) when deferred.
    pub deferred_at_hours: f64,
    /// Airframe hours at which the repair interval expires.
    pub expires_at_hours: f64,
    /// The operator MEL sub-item (`mel_catalog.rs`, e.g. "49-10-01A") the
    /// defect was deferred under, when one was chosen.
    #[serde(default)]
    pub mel_ref: Option<String>,
}

/// One technical log entry, persisted: what was done to the aircraft.
#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct TechLogEntry {
    pub airframe_hours: f64,
    /// Seconds since the Unix epoch, wall clock.
    pub unix_time: u64,
    pub text: String,
}

/// Entries kept; older ones drop off.
const TECH_LOG_MAX: usize = 1000;



/// The live set of deferred items, plus repairs applied this tick (for the
/// log and for `damage.rs` to reset the matching wear).
#[derive(Default)]
pub struct Mel {
    deferred: BTreeMap<u64, Deferral>,
    tech_log: Vec<TechLogEntry>,
}

impl Mel {
    pub fn new() -> Self {
        Self::default()
    }

    /// Restore a persisted set (persistence.rs, load at start).
    pub fn restore(&mut self, deferred: Vec<Deferral>) {
        self.deferred = deferred.into_iter().map(|d| (d.id, d)).collect();
    }

    /// A snapshot for persistence (save).
    pub fn snapshot(&self) -> Vec<Deferral> {
        self.deferred.values().cloned().collect()
    }

    pub fn restore_tech_log(&mut self, log: Vec<TechLogEntry>) {
        self.tech_log = log;
    }

    pub fn tech_log(&self) -> Vec<TechLogEntry> {
        self.tech_log.clone()
    }

    fn log(&mut self, now_hours: f64, text: String) {
        let unix_time = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).map_or(0, |d| d.as_secs());
        self.tech_log.push(TechLogEntry { airframe_hours: now_hours, unix_time, text });
        if self.tech_log.len() > TECH_LOG_MAX {
            let excess = self.tech_log.len() - TECH_LOG_MAX;
            self.tech_log.drain(..excess);
        }
    }

    /// Defer `id` under the MEL, if it is a deferrable item and not already
    /// deferred. `now_hours` is the current airframe-hours total
    /// (`persistence::Persistence::airframe_hours`).
    ///
    /// With `mel_ref` (an operator MEL sub-item, `mel_catalog.rs`) the
    /// interval is that item's own (B/C/D in calendar days); a category A
    /// item states its interval in its conditions, which the page shows, and
    /// takes `MelCategory::A`'s conservative 24 h here. Without one, the
    /// failure's generic category (`failures::mel_item`) applies.
    pub fn defer(&mut self, id: u64, now_hours: f64, mel_ref: Option<&str>) -> Result<(), String> {
        if !crate::failures::active_ids().contains(&id) {
            return Err("not an active failure".into());
        }
        let interval_hours = match mel_ref {
            Some(r) => {
                let (_, sub) = crate::mel_catalog::sub_item(r).ok_or_else(|| format!("no MEL item {r}"))?;
                if !crate::mel_catalog::failures_for(r).contains(&id) {
                    return Err(format!("MEL {r} does not cover this failure"));
                }
                crate::mel_catalog::interval_hours(sub).unwrap_or(MelCategory::A.interval_hours())
            }
            None => crate::failures::mel_item(id).ok_or("not a deferrable item")?.interval_hours(),
        };
        self.insert_deferral(id, now_hours, interval_hours, mel_ref.map(str::to_owned));
        Ok(())
    }

    /// Placard unit `failure_id` of operator MEL sub-item `mel_ref` INOP:
    /// the unit is really lost (its catalogued failure armed), and deferred
    /// under this item with its repair interval. Only items the simulation
    /// models (`mel_catalog::MEL_FAILURES`) can be placarded. Returns the log
    /// line.
    pub fn placard(&mut self, mel_ref: &str, failure_id: u64, now_hours: f64) -> Result<String, String> {
        let (item, sub) = crate::mel_catalog::sub_item(mel_ref).ok_or_else(|| format!("no MEL item {mel_ref}"))?;
        if !crate::mel_catalog::failures_for(mel_ref).contains(&failure_id) {
            return Err(format!("MEL {mel_ref} does not cover failure {failure_id}"));
        }
        if self.deferred.contains_key(&failure_id) {
            return Err(format!("{} is already deferred", crate::failures::any_failure_name(failure_id)));
        }
        let interval = crate::mel_catalog::interval_hours(sub).unwrap_or(MelCategory::A.interval_hours());
        crate::failures::set_active(failure_id, true);
        self.insert_deferral(failure_id, now_hours, interval, Some(mel_ref.to_owned()));
        Ok(format!(
            "MEL {mel_ref} ({}) placarded INOP: {} ({failure_id}) armed and deferred",
            item.title,
            crate::failures::any_failure_name(failure_id)
        ))
    }

    fn insert_deferral(&mut self, id: u64, now_hours: f64, interval_hours: f64, mel_ref: Option<String>) {
        self.deferred.entry(id).or_insert(Deferral {
            id,
            deferred_at_hours: now_hours,
            expires_at_hours: now_hours + interval_hours,
            mel_ref,
        });
    }

    /// Clear a deferral and the failure itself: the maintenance action that
    /// repairs an item (also used for random-failure repairs). Returns
    /// whether anything changed.
    pub fn repair(&mut self, id: u64) -> bool {
        let had_deferral = self.deferred.remove(&id).is_some();
        let was_active = crate::failures::active_ids().contains(&id);
        if was_active {
            crate::failures::set_active(id, false);
        }
        had_deferral || was_active
    }

    pub fn is_deferred(&self, id: u64) -> bool {
        self.deferred.contains_key(&id)
    }

    /// Whether dispatch is legal for every currently-active failure: every
    /// active failure is either not on the MEL at all (must already be
    /// cleared for dispatch by the operator's own procedure, outside this
    /// model's scope) or deferred with time remaining.
    pub fn expired(&self, now_hours: f64) -> Vec<u64> {
        self.deferred
            .values()
            .filter(|d| now_hours >= d.expires_at_hours)
            .map(|d| d.id)
            .collect()
    }

    /// The Study/MEL page: every deferred item with its remaining hours.
    pub fn list(&self, now_hours: f64) -> Vec<(Deferral, f64)> {
        self.deferred
            .values()
            .map(|d| (d.clone(), (d.expires_at_hours - now_hours).max(0.)))
            .collect()
    }

    /// Drain the Study panel's queued defer/repair requests
    /// (`request_defer`/`request_repair`), applying each and returning the
    /// log lines. Call once per tick, after `damage.rs` has armed this
    /// frame's exceedances and before `persistence.rs` snapshots the set,
    /// the same "static request queue, drained by the owning module's own
    /// tick" pattern `circuits.rs`'s breaker requests use.
    pub fn apply_requests(&mut self, now_hours: f64) -> Vec<String> {
        let mut log = Vec::new();
        for req in take_requests() {
            let name = crate::failures::any_failure_name;
            match req {
                Request::Defer(id, mel_ref) => match self.defer(id, now_hours, mel_ref.as_deref()) {
                    Ok(()) => {
                        let text = match &mel_ref {
                            Some(r) => format!("{} ({id}) deferred under MEL {r}", name(id)),
                            None => format!("{} ({id}) deferred under the MEL", name(id)),
                        };
                        self.log(now_hours, text.clone());
                        log.push(text);
                    }
                    Err(e) => log.push(format!("failure {id} could not be deferred: {e}")),
                },
                Request::Repair(id) => {
                    if self.repair(id) {
                        let text = format!("{} ({id}) repaired", name(id));
                        self.log(now_hours, text.clone());
                        log.push(text);
                    }
                }
                Request::Placard(r, failure_id) => match self.placard(&r, failure_id, now_hours) {
                    Ok(text) => {
                        self.log(now_hours, text.clone());
                        log.push(text);
                    }
                    Err(e) => log.push(format!("MEL {r} could not be placarded: {e}")),
                },
                Request::Replace(component) => {
                    let mut store = crate::wear::snapshot();
                    store.reset_component(&component);
                    crate::wear::publish(store);
                    let text = format!("{component} replaced (wear reset)");
                    self.log(now_hours, text.clone());
                    log.push(text);
                }
            }
        }
        log
    }
}

/// The Study/Failures tab's per-id MEL state (`study/web.rs`'s
/// `failures_json`): `publish`/`snapshot` share the latest deferred set and
/// the airframe hours it was computed against, the same pattern
/// `physics/damage.rs`'s `LATEST_WEAR` uses, since the page has no direct
/// access to the running `Mel` (owned by `Plugin`). Call `publish` once per
/// tick, after `apply_requests`.
type Latest = (Vec<Deferral>, f64, Vec<TechLogEntry>);

static LATEST: std::sync::Mutex<Latest> = std::sync::Mutex::new((Vec::new(), 0.0, Vec::new()));

pub fn publish(deferred: Vec<Deferral>, now_hours: f64, tech_log: Vec<TechLogEntry>) {
    if let Ok(mut l) = LATEST.lock() {
        *l = (deferred, now_hours, tech_log);
    }
}

/// The latest published deferrals, airframe hours and tech log, for the
/// MEL & Maintenance page.
pub fn latest() -> Latest {
    LATEST.lock().map(|l| l.clone()).unwrap_or_default()
}

/// Whether `id` is currently deferred under the MEL, and its remaining
/// hours if so (negative once expired, matching `Mel::expired`'s
/// `now_hours >= expires_at_hours` test rather than `list`'s clamp, so a
/// caller can tell "about to expire" from "expired").
pub fn deferred_state(id: u64) -> Option<f64> {
    let (deferred, now_hours, _) = LATEST.lock().ok()?.clone();
    deferred.iter().find(|d| d.id == id).map(|d| d.expires_at_hours - now_hours)
}

/// The Study panel's MEL page: defer or repair a failure. Queued rather
/// than applied immediately since the page has no direct access to the
/// running `Mel`/`Persistence` (owned by `Plugin`); `Mel::apply_requests`
/// drains this once per tick.
enum Request {
    Defer(u64, Option<String>),
    Repair(u64),
    /// Placard one unit (its failure id) of an operator MEL sub-item INOP
    /// (`Mel::placard`).
    Placard(String, u64),
    /// Replace a worn component (`wear.rs` id): its wear back to new.
    Replace(String),
}

static REQUESTS: std::sync::Mutex<Vec<Request>> = std::sync::Mutex::new(Vec::new());

fn take_requests() -> Vec<Request> {
    REQUESTS.lock().map(|mut r| std::mem::take(&mut *r)).unwrap_or_default()
}

pub fn request_defer(id: u64, mel_ref: Option<String>) {
    if let Ok(mut r) = REQUESTS.lock() {
        r.push(Request::Defer(id, mel_ref));
    }
}

pub fn request_replace(component: String) {
    if let Ok(mut r) = REQUESTS.lock() {
        r.push(Request::Replace(component));
    }
}

pub fn request_placard(mel_ref: String, failure_id: u64) {
    if let Ok(mut r) = REQUESTS.lock() {
        r.push(Request::Placard(mel_ref, failure_id));
    }
}

pub fn request_repair(id: u64) {
    if let Ok(mut r) = REQUESTS.lock() {
        r.push(Request::Repair(id));
    }
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
    fn a_deferral_expires_after_its_category_interval() {
        let mut mel = Mel::new();
        mel.deferred.insert(
            1,
            Deferral { id: 1, deferred_at_hours: 10.0, expires_at_hours: 10.0 + MelCategory::C.interval_hours(), mel_ref: None },
        );
        assert!(mel.expired(10.0 + 240.0 - 1.0).is_empty());
        assert_eq!(mel.expired(10.0 + 240.0 + 1.0), vec![1]);
    }

    #[test]
    fn repair_clears_the_deferral() {
        // `failures::STATE` is process-wide and this test reaches it (directly, or
        // through model code such as `Breakers::pre_systems` / `Damage::arm`).
        let _serial = crate::failures::tests::serial();
        let mut mel = Mel::new();
        mel.deferred.insert(5, Deferral { id: 5, deferred_at_hours: 0.0, expires_at_hours: 100.0, mel_ref: None });
        assert!(mel.is_deferred(5));
        mel.repair(5);
        assert!(!mel.is_deferred(5));
    }

    #[test]
    fn a_deferral_under_a_mel_item_uses_that_items_own_interval() {
        // 49-10-01A is category C in the A380 MEL: 10 calendar days.
        let items = crate::mel_catalog::parse(
            r#"{"items":[{"ata":"49-10-01","title":"APU","subitems":[{"id":"49-10-01A","category":"C","installed":"1",
            "required":"0","placard":"Yes","repair_interval_days":10,"conditions":"","references":[],"ops_procedure":null}]}]}"#,
        )
        .unwrap();
        let (_, sub) = crate::mel_catalog::find_in(&items, "49-10-01A").unwrap();
        let mut mel = Mel::new();
        mel.insert_deferral(7, 50.0, crate::mel_catalog::interval_hours(sub).unwrap(), Some("49-10-01A".into()));
        assert!(mel.expired(50.0 + 239.0).is_empty());
        assert_eq!(mel.expired(50.0 + 241.0), vec![7]);
        assert_eq!(mel.snapshot()[0].mel_ref.as_deref(), Some("49-10-01A"));
    }

    #[test]
    fn the_tech_log_keeps_the_newest_entries() {
        let mut mel = Mel::new();
        for i in 0..(TECH_LOG_MAX + 5) {
            mel.log(i as f64, format!("entry {i}"));
        }
        let log = mel.tech_log();
        assert_eq!(log.len(), TECH_LOG_MAX);
        assert_eq!(log.last().unwrap().text, format!("entry {}", TECH_LOG_MAX + 4));
    }
}
