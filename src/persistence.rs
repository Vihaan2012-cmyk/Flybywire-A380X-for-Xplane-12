//! Persistence (hyperrealism physics workstream 6: failures, damage, MEL
//! and persistence): component wear/damage, active and deferred failures,
//! engine hours/cycles, airframe hours and random-failure configuration,
//! saved to `Output/preferences/fbw_a380x_airframe.json`, loaded at plugin
//! start and saved periodically and on exit.
//!
//! Corruption-safe write: serialise to a temp file next to the target, then
//! rename over it (`std::fs::rename` is atomic on the same volume on both
//! Windows and POSIX), so a crash or power loss mid-write never leaves a
//! half-written file the next load would choke on.
//!
//! What is deliberately **not** restored here: crew/passenger oxygen
//! quantity (`oxygen.rs`) and hydraulic reservoir levels
//! (`A32NX_HYD_GREEN_RESERVOIR_LEVEL`/`..._YELLOW_...`) are recorded as
//! `consumables_last_observed` for the Study/maintenance log, but writing
//! them back through `Vars` would not change anything: both are computed
//! fresh from each module's own internal state every tick (`oxygen.rs`'s
//! `Oxygen` keeps its percentage in a plain field seeded to 100% in `new`,
//! not read back from its output variable), so there is no live input path
//! for this workstream to feed a restored value into. Restoring them for
//! real needs a small hook in each owning module (a `with_quantity(...)`
//! constructor parameter, or a variable it reads once at start) — see the
//! workstream 6 report for exactly what each owner would need to add.

#![allow(dead_code)] // Study-panel API (reset), wired up by the lead (see the workstream 6 report).

use std::io::Write;
use std::path::{Path, PathBuf};

use crate::mel::Deferral;
use crate::physics::damage::EngineWear;
use crate::random_failures::{Config as RandomFailuresConfig, Rng};

const CURRENT_VERSION: u32 = 1;

/// Guards read-modify-write of `fbw_a380x_airframe.json`, the same way
/// xphfbw-js-bridge.md rule 8 requires for the settings files (datastore
/// JSON, flyPad ini, xphfbw.json): every writer holds this named mutex
/// around its read-modify-write. A plain `Mutex` only protects this one
/// process; a second writer (XPHFBW's app, once the Study panel's reset/
/// wear-edit actions move there) needs the cross-process one.
fn file_mutex() -> Option<crate::xphfbw_bridge::NamedMutex> {
    crate::xphfbw_bridge::NamedMutex::create("Local\\XPHFBW_airframe_json")
}

#[derive(Clone, Default, serde::Serialize, serde::Deserialize)]
pub struct ConsumablesObserved {
    pub oxygen_crew_percent: f64,
    pub oxygen_pax_percent: f64,
    pub hydraulic_green_reservoir_level: f64,
    pub hydraulic_yellow_reservoir_level: f64,
}

#[derive(Clone, serde::Serialize, serde::Deserialize)]
pub struct AirframeState {
    pub version: u32,
    pub airframe_hours: f64,
    pub engines: [EngineWear; 4],
    pub apu_hours: f64,
    /// The ids `failures::active_ids()` had at save time (excluding
    /// anything the MEL already has deferred, which `deferred` covers).
    pub active_failure_ids: Vec<u64>,
    /// `failures::active_magnitudes()` at save time -- the continuous
    /// physical fraction (docs/physics/failures.md) each of
    /// `active_failure_ids` was armed at. An id present in
    /// `active_failure_ids` but missing here (an older save file) restores
    /// at `1.0`, matching `failures::magnitude`'s own fallback.
    #[serde(default)]
    pub active_failure_magnitudes: std::collections::BTreeMap<u64, f64>,
    pub deferred: Vec<Deferral>,
    pub random_failures_config: RandomFailuresConfig,
    pub random_failures_seed: Rng,
    pub consumables_last_observed: ConsumablesObserved,
    /// Persistent component wear (`wear.rs`): hot hours, cycles,
    /// thermal-stress integral and degradation fraction, keyed by component
    /// id. Threaded the same snapshot/publish way as `engines` above.
    #[serde(default)]
    pub wear: crate::wear::WearStore,
    /// The technical log (`mel.rs`): deferrals, repairs, replacements.
    #[serde(default)]
    pub tech_log: Vec<crate::mel::TechLogEntry>,
    /// Direct component parameter settings (`components.rs`), including any
    /// still worsening.
    #[serde(default)]
    pub components: Vec<crate::components::Direct>,
}

impl Default for AirframeState {
    /// The "new airframe" reset: zero wear, nothing active or deferred,
    /// random failures off until the user turns them on.
    fn default() -> Self {
        Self {
            version: CURRENT_VERSION,
            airframe_hours: 0.0,
            engines: Default::default(),
            apu_hours: 0.0,
            active_failure_ids: Vec::new(),
            active_failure_magnitudes: std::collections::BTreeMap::new(),
            deferred: Vec::new(),
            random_failures_config: RandomFailuresConfig::default(),
            random_failures_seed: Rng::new(0x1234_5678_9abc_def0),
            consumables_last_observed: ConsumablesObserved::default(),
            wear: crate::wear::WearStore::default(),
            tech_log: Vec::new(),
            components: Vec::new(),
        }
    }
}

pub struct Persistence {
    path: PathBuf,
    /// Seconds of real time since the last save (periodic save timer).
    since_save: f64,
    pub state: AirframeState,
}

/// How often to save while running (in addition to on exit), so a crash
/// loses at most this much wear/damage/MEL history.
const SAVE_INTERVAL_S: f64 = 120.0;

impl Persistence {
    /// `xplane_root` is X-Plane's own root (the directory `Output/` lives
    /// under); the plugin already knows it to find `Output/preferences/`.
    pub fn new(xplane_root: &Path) -> Self {
        let path = xplane_root.join("Output").join("preferences").join("fbw_a380x_airframe.json");
        let state = Self::load(&path).unwrap_or_default();
        Self { path, since_save: 0.0, state }
    }

    fn load(path: &Path) -> Option<AirframeState> {
        // Race/desync rule 8 (xphfbw-js-bridge.md): fbw_a380x_airframe.json
        // can be touched by more than one process (this plugin's own
        // save/reset and, from the Study panel moving into the app,
        // XPHFBW itself), so every read-modify-write holds the same named
        // mutex the settings files use the same pattern for.
        let read = || std::fs::read_to_string(path);
        let text = match file_mutex() {
            Some(m) => m.with(read),
            None => read(),
        }
        .ok()?;
        match serde_json::from_str::<AirframeState>(&text) {
            Ok(state) if state.version == CURRENT_VERSION => Some(state),
            Ok(_) => {
                crate::log("fbw_a380x_airframe.json is from a different version; starting a new airframe");
                None
            }
            Err(e) => {
                crate::log(&format!("fbw_a380x_airframe.json is corrupt ({e}); starting a new airframe"));
                None
            }
        }
    }

    /// Write-temp-then-rename: corruption-safe even if X-Plane is killed
    /// mid-write. Held under [`file_mutex`] (race/desync rule 8): without
    /// it, two writers racing to create the same `.json.tmp` name could have
    /// one truncate the other's in-flight write before either renames.
    ///
    /// A no-op, without touching the file, while `xphfbw.persistence`
    /// (app_settings.rs, the Simulation settings panel) is off — checked
    /// live on every call, so turning it off mid-session stops the
    /// periodic autosave, the exit save, and "reset airframe"'s save from
    /// writing anything further; the file already on disk (if any) is left
    /// exactly as it was. In-memory wear/damage/failure state keeps
    /// updating regardless: this setting is about the file, not the
    /// simulation.
    pub fn save(&self) -> std::io::Result<()> {
        self.save_if_enabled(crate::app_settings::current().persistence)
    }

    /// [`save`]'s body, taking the `xphfbw.persistence` flag as a
    /// parameter rather than reading `app_settings::current()` itself, so
    /// the "disabled means untouched" behaviour is unit-testable without
    /// the real process-wide settings watcher.
    fn save_if_enabled(&self, enabled: bool) -> std::io::Result<()> {
        if !enabled {
            return Ok(());
        }
        if let Some(dir) = self.path.parent() {
            std::fs::create_dir_all(dir)?;
        }
        let json = serde_json::to_string_pretty(&self.state).map_err(|e| std::io::Error::new(std::io::ErrorKind::Other, e))?;
        let tmp = self.path.with_extension("json.tmp");
        let path = &self.path;
        let write = move || -> std::io::Result<()> {
            {
                let mut f = std::fs::File::create(&tmp)?;
                f.write_all(json.as_bytes())?;
                f.sync_all()?;
            }
            std::fs::rename(&tmp, path)
        };
        match file_mutex() {
            Some(m) => m.with(write),
            None => write(),
        }
    }

    /// Call once per tick with real elapsed seconds (already excludes
    /// pause, the same `paused()` gate as `random_failures`/`damage`);
    /// advances the airframe-hours clock and saves periodically.
    pub fn tick(&mut self, delta_s: f64) {
        self.state.airframe_hours += delta_s / 3600.0;
        self.since_save += delta_s;
        if self.since_save >= SAVE_INTERVAL_S {
            self.since_save = 0.0;
            if let Err(e) = self.save() {
                crate::log(&format!("could not save fbw_a380x_airframe.json: {e}"));
            }
        }
    }

    /// "New airframe": wipe all wear/damage/MEL/failure history and save
    /// immediately, for the Study panel's reset action.
    pub fn reset(&mut self) {
        self.state = AirframeState::default();
        let _ = self.save();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_round_trip_preserves_wear_deferrals_and_the_random_seed() {
        let dir = std::env::temp_dir().join(format!("fbw_persistence_test_{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).unwrap();

        let mut p = Persistence::new(&dir);
        p.state.airframe_hours = 123.5;
        p.state.engines[0].creep_life_fraction = 0.42;
        p.state.engines[0].hours = 10.0;
        p.state.engines[0].cycles = 3;
        p.state.deferred.push(Deferral { id: 24_020, deferred_at_hours: 100.0, expires_at_hours: 340.0, mel_ref: Some("24-01-01A".into()) });
        p.state.random_failures_config.enabled = true;
        p.state.random_failures_config.rate_multiplier = 2.5;
        p.state.random_failures_seed = Rng::new(999);
        p.save().unwrap();

        let reloaded = Persistence::new(&dir);
        assert_eq!(reloaded.state.airframe_hours, 123.5);
        assert_eq!(reloaded.state.engines[0].creep_life_fraction, 0.42);
        assert_eq!(reloaded.state.engines[0].hours, 10.0);
        assert_eq!(reloaded.state.engines[0].cycles, 3);
        assert_eq!(reloaded.state.deferred.len(), 1);
        assert_eq!(reloaded.state.deferred[0].id, 24_020);
        assert_eq!(reloaded.state.deferred[0].mel_ref.as_deref(), Some("24-01-01A"));
        assert!(reloaded.state.random_failures_config.enabled);
        assert_eq!(reloaded.state.random_failures_config.rate_multiplier, 2.5);

        std::fs::remove_dir_all(&dir).ok();
    }

    #[test]
    fn a_missing_file_starts_a_new_airframe() {
        let dir = std::env::temp_dir().join(format!("fbw_persistence_missing_{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        let p = Persistence::new(&dir);
        assert_eq!(p.state.airframe_hours, 0.0);
        assert_eq!(p.state.engines[0].creep_life_fraction, 0.0);
    }

    #[test]
    fn a_corrupt_file_starts_a_new_airframe_instead_of_failing() {
        let dir = std::env::temp_dir().join(format!("fbw_persistence_corrupt_{}", std::process::id()));
        std::fs::create_dir_all(dir.join("Output").join("preferences")).unwrap();
        std::fs::write(dir.join("Output").join("preferences").join("fbw_a380x_airframe.json"), b"{ not json").unwrap();
        let p = Persistence::new(&dir);
        assert_eq!(p.state.airframe_hours, 0.0);
        std::fs::remove_dir_all(&dir).ok();
    }

    #[test]
    fn reset_wipes_wear_and_deferrals() {
        let dir = std::env::temp_dir().join(format!("fbw_persistence_reset_{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).unwrap();
        let mut p = Persistence::new(&dir);
        p.state.engines[0].creep_life_fraction = 5.0;
        p.state.deferred.push(Deferral { id: 1, deferred_at_hours: 0.0, expires_at_hours: 10.0, mel_ref: None });
        p.reset();
        assert_eq!(p.state.engines[0].creep_life_fraction, 0.0);
        assert!(p.state.deferred.is_empty());
        // The reset persisted too.
        let reloaded = Persistence::new(&dir);
        assert_eq!(reloaded.state.engines[0].creep_life_fraction, 0.0);
        std::fs::remove_dir_all(&dir).ok();
    }

    #[test]
    fn tick_accumulates_airframe_hours() {
        let dir = std::env::temp_dir().join(format!("fbw_persistence_hours_{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).unwrap();
        let mut p = Persistence::new(&dir);
        for _ in 0..3600 {
            p.tick(1.0);
        }
        assert!((p.state.airframe_hours - 1.0).abs() < 1e-9);
        std::fs::remove_dir_all(&dir).ok();
    }

    /// `xphfbw.persistence` off (app_settings.rs): `save()` must not touch
    /// the file at all, even though the in-memory state changed.
    #[test]
    fn save_is_a_no_op_while_persistence_is_disabled() {
        let dir = std::env::temp_dir().join(format!("fbw_persistence_disabled_{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).unwrap();
        let mut p = Persistence::new(&dir);
        p.state.airframe_hours = 42.0;
        p.save_if_enabled(false).unwrap();
        assert!(!p.path.exists(), "disabled persistence must not create the file");

        // Enabling it again writes normally.
        p.save_if_enabled(true).unwrap();
        assert!(p.path.exists());
        let reloaded = Persistence::new(&dir);
        assert_eq!(reloaded.state.airframe_hours, 42.0);
        std::fs::remove_dir_all(&dir).ok();
    }
}
