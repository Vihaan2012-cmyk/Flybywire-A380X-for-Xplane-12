//! Persistent wear store (hyperrealism physics workstream 6: failures,
//! damage, MEL and persistence): accumulated hot hours, engine/gear-style
//! cycles, a thermal-stress integral, and a degradation fraction, kept
//! per component id and surviving across flights the same way
//! `physics/damage.rs`'s `EngineWear` and `mel.rs`'s deferral list do --
//! a process-global "latest" published each tick (read by the Study panel
//! and any consumer that doesn't own the tracker), with the authoritative
//! copy loaded from and saved back to `persistence::AirframeState::wear`
//! (`src/persistence.rs`) by the plugin's own `Persistence` struct, exactly
//! like `damage.engines`/`persistence.state.engines`.
//!
//! This module intentionally does *not* clamp `degradation_fraction` (or
//! anything else) to a physical range itself: a substrate agent is adding
//! logged physical clamps at the shared accessors this workstream and
//! others read wear back through, and a second, silent clamp here would
//! only hide what that logging is meant to catch. Callers accumulate
//! whatever their own physical model computes; only [`WearStore::get`]'s
//! documented range is a *contract* for callers, not an enforced one.
//!
//! Component ids are caller-chosen strings (e.g. `"engine-1"`, `"apu"`,
//! `"tr-1"`, a `breakers.rs` catalogue id): this module owns no id
//! catalogue of its own, it only accumulates whatever a physics/electrical/
//! etc. module reports against a stable id, the same way `mel.rs` accepts
//! any `failures.rs` id without knowing the failure catalogue itself.

use std::collections::BTreeMap;
use std::sync::Mutex;

/// One component's accumulated wear.
#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Wear {
    /// Hours spent above the component's own "hot" threshold (caller-
    /// defined -- e.g. `physics/damage.rs`'s TGT redlines for an engine),
    /// accumulated real time only (never sim-paused time), same convention
    /// `EngineWear::hours` uses.
    pub hot_hours: f64,
    /// Load/thermal cycles (e.g. start-stop, pressurization), counted by
    /// the caller crossing whatever edge it defines a cycle as.
    pub cycles: u32,
    /// Running integral of a caller-supplied thermal-stress rate (e.g.
    /// `(T / T_rated)^n * dt`), unitless accumulation -- the caller's
    /// model defines what a given magnitude means.
    pub thermal_stress_integral: f64,
    /// Fractional degradation the caller's model has accumulated.
    /// Documented range is `0.0..=1.0` ("undamaged" to "fully degraded"),
    /// but this module does not enforce it (see the module doc): a shared
    /// accessor elsewhere is responsible for the logged physical clamp.
    pub degradation_fraction: f64,
}

impl Default for Wear {
    fn default() -> Self {
        Self { hot_hours: 0.0, cycles: 0, thermal_stress_integral: 0.0, degradation_fraction: 0.0 }
    }
}

/// Every component's [`Wear`], keyed by its caller-chosen id. Serializes
/// straight into `persistence::AirframeState::wear`.
#[derive(Clone, Debug, Default, serde::Serialize, serde::Deserialize)]
pub struct WearStore(BTreeMap<String, Wear>);

impl WearStore {
    /// `id`'s wear, or [`Wear::default`] (all-zero) if nothing has ever
    /// been recorded against it -- a component with no history reads as
    /// brand new, not as an error.
    pub fn get(&self, id: &str) -> Wear {
        self.0.get(id).copied().unwrap_or_default()
    }

    /// Every id this store has ever recorded wear for.
    pub fn ids(&self) -> impl Iterator<Item = &str> {
        self.0.keys().map(String::as_str)
    }

    /// Add this tick's contribution to `id`'s wear: `hot_hours` and
    /// `thermal_stress` add (a running accumulation), `cycles` adds
    /// (a running count), `degradation_delta` adds to the existing
    /// fraction (the caller's model decides how much a tick degrades the
    /// component by; this store just remembers the running total -- see
    /// the module doc on why it isn't clamped here).
    pub fn accumulate(&mut self, id: &str, hot_hours: f64, cycles: u32, thermal_stress: f64, degradation_delta: f64) {
        let w = self.0.entry(id.to_owned()).or_default();
        w.hot_hours += hot_hours;
        w.cycles += cycles;
        w.thermal_stress_integral += thermal_stress;
        w.degradation_fraction += degradation_delta;
    }

    /// Replace `id`'s wear outright (Ground Services "replace component"/
    /// reset-to-new maintenance action, or a persistence load).
    pub fn set(&mut self, id: &str, wear: Wear) {
        self.0.insert(id.to_owned(), wear);
    }

    /// Reset `id` back to brand-new (a maintenance "replace component"
    /// action), dropping its entry entirely so [`ids`](Self::ids) no
    /// longer lists it until it accumulates wear again.
    pub fn reset_component(&mut self, id: &str) {
        self.0.remove(id);
    }
}

/// Validate a persisted `WearStore` on load, once, the same way
/// `physics::damage::load_engines` already guards `EngineWear`'s
/// `creep_life_fraction` against the identical bug class (a transient
/// spike from a single bad tick, saved, then carried forward forever
/// because nothing on load ever looks at it again -- the W217 persisted-
/// creep incident this module's own doc alludes to as "a substrate agent
/// is adding logged physical clamps at the shared accessors"). This is
/// deliberately narrower than that accessor is expected to be: it only
/// rejects a `degradation_fraction` that is negative or non-finite --
/// unambiguous corruption, not a value merely above the documented
/// `0.0..=1.0` ceiling, which the module doc (see above) leaves for the
/// shared accessor to interpret, not this load path, to decide. It does
/// NOT change [`WearStore::accumulate`]/[`WearStore::set`]'s own no-clamp
/// behaviour during a live session -- only what a *previous* session's
/// file is trusted to restore.
pub fn load_checked(saved: WearStore) -> (WearStore, Vec<String>) {
    let mut rejections = Vec::new();
    let mut checked = WearStore::default();
    for id in saved.ids() {
        let mut w = saved.get(id);
        let bad_degradation = !w.degradation_fraction.is_finite() || w.degradation_fraction < 0.0;
        let bad_hot_hours = !w.hot_hours.is_finite() || w.hot_hours < 0.0;
        let bad_stress = !w.thermal_stress_integral.is_finite() || w.thermal_stress_integral < 0.0;
        if bad_degradation || bad_hot_hours || bad_stress {
            rejections.push(format!(
                "component {id}: persisted wear (degradation_fraction={:.4}, hot_hours={:.4}, thermal_stress_integral={:.4}) has a negative or non-finite field, not physically reachable through accumulate(); reset to new on load",
                w.degradation_fraction, w.hot_hours, w.thermal_stress_integral
            ));
            w = Wear::default();
        }
        checked.set(id, w);
    }
    (checked, rejections)
}

/// The process-global "current" wear, published each tick the same way
/// `physics/damage.rs`'s `LATEST_WEAR` is: the owning tracker (wherever it
/// ends up living -- `Plugin`, alongside `damage`) calls [`publish`] after
/// updating; the Study panel and any other reader calls [`snapshot`].
static LATEST: Mutex<Option<WearStore>> = Mutex::new(None);

pub fn publish(store: WearStore) {
    if let Ok(mut w) = LATEST.lock() {
        *w = Some(store);
    }
}

pub fn snapshot() -> WearStore {
    LATEST.lock().ok().and_then(|w| w.clone()).unwrap_or_default()
}

/// Test-only global reset: clears the published snapshot, mirroring
/// `failures::reset_all`. Call at test SETUP -- this state is process-wide.
pub fn reset_all() {
    if let Ok(mut w) = LATEST.lock() {
        *w = None;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A component with no recorded wear reads as brand new: every field
    /// zero. Expected value is the type's own documented "new" state, not
    /// anything the sim computes.
    #[test]
    fn unknown_component_reads_as_brand_new() {
        let store = WearStore::default();
        assert_eq!(store.get("engine-1"), Wear::default());
        assert_eq!(store.ids().count(), 0);
    }

    /// Two accumulate() calls of 1.0h and 1.5h of hot time, 1 cycle then 2
    /// cycles, and stress contributions of 0.2 then 0.3, must sum exactly
    /// -- plain arithmetic, independent of any sim output: 1.0+1.5=2.5h,
    /// 1+2=3 cycles, 0.2+0.3=0.5 stress integral.
    #[test]
    fn accumulate_sums_across_calls() {
        let mut store = WearStore::default();
        store.accumulate("engine-1", 1.0, 1, 0.2, 0.05);
        store.accumulate("engine-1", 1.5, 2, 0.3, 0.05);
        let w = store.get("engine-1");
        assert_eq!(w.hot_hours, 2.5);
        assert_eq!(w.cycles, 3);
        assert!((w.thermal_stress_integral - 0.5).abs() < 1e-12);
        assert!((w.degradation_fraction - 0.1).abs() < 1e-12);
    }

    /// Components are independent: wear recorded against "engine-1" must
    /// never bleed into "engine-2", and both ids must be listed once each.
    #[test]
    fn components_accumulate_independently() {
        let mut store = WearStore::default();
        store.accumulate("engine-1", 5.0, 10, 1.0, 0.1);
        store.accumulate("engine-2", 2.0, 4, 0.4, 0.02);
        assert_eq!(store.get("engine-1").hot_hours, 5.0);
        assert_eq!(store.get("engine-2").hot_hours, 2.0);
        let ids: Vec<&str> = store.ids().collect();
        assert_eq!(ids, vec!["engine-1", "engine-2"]);
    }

    /// This module deliberately does not clamp `degradation_fraction`: a
    /// caller accumulating past 1.0 (e.g. 0.6 + 0.7) must see the raw
    /// 1.3 unmolested, since a shared accessor elsewhere owns the logged
    /// physical clamp (module doc). A silent clamp here would defeat that.
    #[test]
    fn degradation_fraction_is_not_silently_clamped() {
        let mut store = WearStore::default();
        store.accumulate("gear-actuator", 0.0, 0, 0.0, 0.6);
        store.accumulate("gear-actuator", 0.0, 0, 0.0, 0.7);
        assert!((store.get("gear-actuator").degradation_fraction - 1.3).abs() < 1e-12);
    }

    /// A maintenance "replace component" action drops the id back to
    /// unrecorded (brand new), not merely zeroed fields still present.
    #[test]
    fn reset_component_removes_its_history() {
        let mut store = WearStore::default();
        store.accumulate("brake-1", 3.0, 6, 0.9, 0.3);
        store.reset_component("brake-1");
        assert_eq!(store.get("brake-1"), Wear::default());
        assert_eq!(store.ids().count(), 0);
    }

    /// set() replaces a component's wear outright (a persistence load
    /// restoring a saved value), overwriting rather than accumulating.
    #[test]
    fn set_replaces_rather_than_accumulates() {
        let mut store = WearStore::default();
        store.accumulate("apu", 1.0, 1, 0.1, 0.05);
        store.set("apu", Wear { hot_hours: 42.0, cycles: 7, thermal_stress_integral: 3.0, degradation_fraction: 0.5 });
        let w = store.get("apu");
        assert_eq!(w.hot_hours, 42.0);
        assert_eq!(w.cycles, 7);
    }

    /// Round-trips through the same JSON serialisation persistence.rs uses
    /// for the rest of `AirframeState`, so a saved store survives a
    /// save/load cycle byte-for-byte in value.
    #[test]
    fn serializes_and_round_trips_through_json() {
        let mut store = WearStore::default();
        store.accumulate("engine-3", 12.5, 30, 4.4, 0.22);
        let json = serde_json::to_string(&store).unwrap();
        let back: WearStore = serde_json::from_str(&json).unwrap();
        assert_eq!(back.get("engine-3"), store.get("engine-3"));
    }

    /// publish()/snapshot() is the process-global channel the Study panel
    /// and any non-owning reader use, mirroring physics/damage.rs's
    /// LATEST_WEAR. Before publish(), snapshot() reads as empty (default),
    /// not a stale value from an unrelated test -- guarded by reset_all().
    #[test]
    fn publish_and_snapshot_round_trip_the_process_global() {
        reset_all();
        assert_eq!(snapshot().ids().count(), 0);
        let mut store = WearStore::default();
        store.accumulate("tr-1", 8.0, 2, 1.1, 0.15);
        publish(store);
        assert_eq!(snapshot().get("tr-1").hot_hours, 8.0);
        reset_all();
        assert_eq!(snapshot().ids().count(), 0, "reset_all must clear the published snapshot for the next test");
    }
}
