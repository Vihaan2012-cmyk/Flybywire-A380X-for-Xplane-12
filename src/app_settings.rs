//! The XPHFBW app's own settings ("Simulation (XPHFBW)" in
//! `docs/briefs/xphfbw-app.md`), read from `Output/preferences/xphfbw.json`
//! (`settings_files::app_settings_path`) and applied to the modules whose
//! behaviour they gate. See `docs/briefs/xphfbw-js-bridge.md`, "B — App
//! settings in the plugin".
//!
//! xphfbw.json is written by the app's settings window
//! (`app/src/settings.rs`) and read here: a background thread polls its
//! mtime every [`POLL_EVERY`] and reloads only when it changed, so a
//! setting the pilot changes in the app's window is picked up without
//! restarting X-Plane. [`current`] lazily starts that thread on first use
//! and returns the latest snapshot; it never blocks on disk I/O itself.
//!
//! What "live" means per key (the rest apply at the next aircraft load,
//! i.e. the next time the plugin's `Plugin::new` builds the owning struct
//! — this module cannot rebuild another module's state from inside a poll
//! thread):
//! - `randomFailures` / `failureRate`: live. `random_failures.rs`'s
//!   `RandomFailures::apply_requests` (called every tick already) resyncs
//!   from here whenever [`generation`] has moved since it last looked,
//!   unless the Study panel queued its own change that tick (that wins).
//! - `persistence`: live. `persistence.rs`'s `Persistence::save` checks it
//!   on every call (periodic autosave, exit save, and "reset airframe"),
//!   so turning it off stops further writes to
//!   `fbw_a380x_airframe.json` from that point on; the file already on
//!   disk is left untouched either way, and in-memory wear/damage/failure
//!   state keeps updating regardless (this setting is about the file, not
//!   the simulation).
//! - `stateDumps` / `stateDumpFrames` / `stateDumpKeep`: live.
//!   `state_dump.rs`'s `StateDump::tick`/`write` check every call.
//! - `coldStartGroundPower`: next aircraft load. `efb.rs`'s cold-start
//!   block runs exactly once, on the first `Efb::update` after
//!   `Efb::new` (i.e. once per aircraft load).
//! - `cabinVisible`: live. `cabin_option.rs`'s `CabinOption::update`
//!   pushes it into `fbw/options/cabin_visible` (the dataref the converted
//!   cabin objects' `ANIM_show` reads) every tick it changes.
//! - `systemsOutOfProcess`: next aircraft load. [`systems_out_of_process`]
//!   is read by `start_systems` (lib.rs, agent A), which only runs once,
//!   at `Plugin::new`.
//!
//! ## Rule 8 (the settings-files mutex)
//!
//! Every writer of xphfbw.json, the flyPad ini, or the NXDataStore JSON
//! holds the named mutex `Local\XPHFBW_settings_files` around its whole
//! read-modify-write, and writes to a temp file and renames
//! (`docs/briefs/xphfbw-js-bridge.md` race rule 8). [`with_settings_lock`]
//! is that shared entry point: `app/src/settings.rs::save` and this
//! plugin's flyPad-settings writer (`js_bridge.rs`'s `FlyPadSettings::set`)
//! both wrap their read-modify-write in it. This module only *reads*
//! xphfbw.json, and every writer renames into place atomically, so a
//! reader never needs the lock itself.

use std::path::Path;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{OnceLock, RwLock};
use std::time::Duration;

use serde_json::{Map, Value};

/// How often the background thread checks xphfbw.json's mtime.
const POLL_EVERY: Duration = Duration::from_secs(2);

/// The "Simulation (XPHFBW)" panel's settings, with FlyByWire/XPHFBW's
/// shipped defaults (must match `app/ui/index.html`'s `DEFAULTS`).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AppSettings {
    pub systems_out_of_process: bool,
    pub cold_start_ground_power: bool,
    pub random_failures: bool,
    pub failure_rate: f64,
    pub persistence: bool,
    pub state_dumps: bool,
    pub state_dump_frames: u64,
    pub state_dump_keep: usize,
    /// The EFB's "Passenger cabin" switch (Settings > Aircraft).
    pub cabin_visible: bool,
}

impl Default for AppSettings {
    fn default() -> Self {
        Self {
            systems_out_of_process: true,
            cold_start_ground_power: true,
            random_failures: false,
            failure_rate: 1.0,
            persistence: true,
            state_dumps: false,
            state_dump_frames: crate::state_dump::DEFAULT_EVERY_TICKS,
            state_dump_keep: crate::state_dump::DEFAULT_KEEP_PER_SESSION,
            cabin_visible: true,
        }
    }
}

fn as_bool(map: &Map<String, Value>, key: &str, default: bool) -> bool {
    match map.get(key) {
        Some(Value::Bool(b)) => *b,
        Some(Value::String(s)) => s == "true",
        _ => default,
    }
}

fn as_f64(map: &Map<String, Value>, key: &str, default: f64) -> f64 {
    match map.get(key) {
        Some(Value::Number(n)) => n.as_f64().unwrap_or(default),
        Some(Value::String(s)) => s.trim().parse().unwrap_or(default),
        _ => default,
    }
}

/// Parses the keys this module owns out of xphfbw.json's map (unprefixed:
/// `app/src/settings.rs::load` already strips `xphfbw.` before writing the
/// file). Anything missing, mistyped, or out of the UI's own range
/// (`app/ui/index.html`) falls back to the default rather than producing a
/// setting that could panic a caller (a zero frame interval, for example).
fn parse(map: &Map<String, Value>) -> AppSettings {
    let defaults = AppSettings::default();
    AppSettings {
        systems_out_of_process: as_bool(map, "systemsOutOfProcess", defaults.systems_out_of_process),
        cold_start_ground_power: as_bool(map, "coldStartGroundPower", defaults.cold_start_ground_power),
        random_failures: as_bool(map, "randomFailures", defaults.random_failures),
        failure_rate: as_f64(map, "failureRate", defaults.failure_rate).clamp(0.0, 1_000_000.0),
        persistence: as_bool(map, "persistence", defaults.persistence),
        state_dumps: as_bool(map, "stateDumps", defaults.state_dumps),
        state_dump_frames: (as_f64(map, "stateDumpFrames", defaults.state_dump_frames as f64) as u64).clamp(1, 1_000_000),
        state_dump_keep: (as_f64(map, "stateDumpKeep", defaults.state_dump_keep as f64) as u64).clamp(1, 1_000_000) as usize,
        cabin_visible: as_bool(map, "cabinVisible", defaults.cabin_visible),
    }
}

/// Reads and parses xphfbw.json at `path`; the shipped defaults if it does
/// not exist or does not parse (a fresh install, or a write caught
/// mid-rename — the writer renames into place, so a reader only ever sees
/// either the old file or the new one, never a partial one, but a reader
/// that races the very first `create_dir_all` can still see "missing").
fn load_from_path(path: &Path) -> AppSettings {
    let map = std::fs::read_to_string(path)
        .ok()
        .and_then(|t| serde_json::from_str::<Value>(&t).ok())
        .and_then(|v| v.as_object().cloned())
        .unwrap_or_default();
    parse(&map)
}

/// The live settings plus a generation counter, reusable in isolation
/// (unit-tested on its own instance, never the process-wide [`GLOBAL`],
/// so a test here can never race another module's test over shared
/// mutable state).
struct Store {
    settings: RwLock<AppSettings>,
    generation: AtomicU64,
}

impl Store {
    fn new(initial: AppSettings) -> Self {
        Self { settings: RwLock::new(initial), generation: AtomicU64::new(0) }
    }

    fn get(&self) -> AppSettings {
        *self.settings.read().unwrap_or_else(std::sync::PoisonError::into_inner)
    }

    /// Replaces the settings and bumps the generation, even if `settings`
    /// is unchanged (called only when xphfbw.json's mtime moved, so every
    /// call here is a real reload worth telling pollers about).
    fn set(&self, settings: AppSettings) {
        *self.settings.write().unwrap_or_else(std::sync::PoisonError::into_inner) = settings;
        self.generation.fetch_add(1, Ordering::SeqCst);
    }

    fn generation(&self) -> u64 {
        self.generation.load(Ordering::SeqCst)
    }
}

static GLOBAL: OnceLock<Store> = OnceLock::new();
static WATCHER_STARTED: OnceLock<()> = OnceLock::new();

fn global() -> &'static Store {
    GLOBAL.get_or_init(|| Store::new(AppSettings::default()))
}

/// Starts the background poller the first time anything asks for the
/// current settings; a no-op every time after. Outside X-Plane (unit
/// tests, `cargo test`), `crate::xp::system_path` returns `None` forever,
/// so the thread just idles and every caller keeps the shipped defaults —
/// nothing here touches the filesystem in that case.
fn ensure_watcher_started() {
    WATCHER_STARTED.get_or_init(|| {
        // Do the first load synchronously so the very first tick already
        // has whatever xphfbw.json says, rather than one poll interval of
        // defaults.
        reload_if_available();
        let spawned = std::thread::Builder::new().name("fbw app settings".into()).spawn(|| loop {
            std::thread::sleep(POLL_EVERY);
            reload_if_available();
        });
        if let Err(e) = spawned {
            crate::log(&format!("app settings: could not start the watcher thread ({e}); xphfbw.json changes need an aircraft reload"));
        }
    });
}

/// Reloads from xphfbw.json if its mtime moved since the last successful
/// reload (or this is the first check); a no-op otherwise, and a no-op if
/// X-Plane's folder is not known yet.
fn reload_if_available() {
    static LAST_MTIME: OnceLock<std::sync::Mutex<Option<std::time::SystemTime>>> = OnceLock::new();
    let Some(root) = crate::xp::system_path() else { return };
    let path = crate::settings_files::app_settings_path(&root);
    let modified = std::fs::metadata(&path).and_then(|m| m.modified()).ok();
    let cell = LAST_MTIME.get_or_init(|| std::sync::Mutex::new(None));
    let mut last = cell.lock().unwrap_or_else(std::sync::PoisonError::into_inner);
    if *last == modified && last.is_some() {
        return;
    }
    *last = modified;
    drop(last);
    global().set(load_from_path(&path));
}

/// The current settings; starts the watcher on first call (see
/// [`ensure_watcher_started`]). Cheap — a clone of a small `Copy` struct
/// under a read lock — so every tick can call this.
pub fn current() -> AppSettings {
    ensure_watcher_started();
    global().get()
}

/// Bumps every time xphfbw.json is (re)loaded, including the first load.
/// `random_failures.rs` uses this to notice a change without re-reading
/// the file itself every tick.
pub fn generation() -> u64 {
    ensure_watcher_started();
    global().generation()
}

/// `xphfbw.systemsOutOfProcess`, for `start_systems` (lib.rs, agent A) to
/// decide whether to launch `fbw_a380_systems_server.exe`/XPHFBW.exe
/// separately or run the systems in-process.
pub fn systems_out_of_process() -> bool {
    current().systems_out_of_process
}

// ---------------------------------------------------------------------
// Rule 8: the cross-process settings-files mutex.
// ---------------------------------------------------------------------

/// The name rule 8 (`docs/briefs/xphfbw-js-bridge.md`) specifies for the
/// datastore JSON/flyPad ini/xphfbw.json read-modify-write lock.
/// `persistence.rs`'s own `file_mutex()` follows the same pattern for
/// `fbw_a380x_airframe.json`, under its own name.
const SETTINGS_FILES_MUTEX: &str = r"Local\XPHFBW_settings_files";

/// Holds the rule-8 settings-files mutex for the duration of `f`, released
/// even if `f` panics (`xphfbw_bridge::NamedMutex::with` itself does not
/// catch a panic in its closure, so this wraps `f` in `catch_unwind`
/// first: that inner closure never panics, so `NamedMutex::with`'s own
/// `ReleaseMutex` always runs before this function re-raises the panic).
/// If the OS mutex cannot even be created (should not happen on Windows),
/// runs `f` unguarded rather than losing the write entirely — no worse
/// than before this rule existed.
pub fn with_settings_lock<R>(f: impl FnOnce() -> R) -> R {
    let Some(mutex) = crate::xphfbw_bridge::NamedMutex::create(SETTINGS_FILES_MUTEX) else {
        return f();
    };
    match mutex.with(|| std::panic::catch_unwind(std::panic::AssertUnwindSafe(f))) {
        Ok(r) => r,
        Err(payload) => std::panic::resume_unwind(payload),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn map(pairs: &[(&str, Value)]) -> Map<String, Value> {
        pairs.iter().map(|(k, v)| (k.to_string(), v.clone())).collect()
    }

    #[test]
    fn defaults_when_the_key_is_missing() {
        let s = parse(&Map::new());
        assert_eq!(s, AppSettings::default());
    }

    #[test]
    fn parses_every_key_the_app_writes() {
        let m = map(&[
            ("systemsOutOfProcess", Value::Bool(false)),
            ("coldStartGroundPower", Value::Bool(false)),
            ("randomFailures", Value::Bool(true)),
            ("failureRate", Value::String("2.5".into())),
            ("persistence", Value::Bool(false)),
            ("stateDumps", Value::Bool(false)),
            ("stateDumpFrames", Value::String("50".into())),
            ("stateDumpKeep", Value::String("10".into())),
        ]);
        let s = parse(&m);
        assert!(!s.systems_out_of_process);
        assert!(!s.cold_start_ground_power);
        assert!(s.random_failures);
        assert_eq!(s.failure_rate, 2.5);
        assert!(!s.persistence);
        assert!(!s.state_dumps);
        assert_eq!(s.state_dump_frames, 50);
        assert_eq!(s.state_dump_keep, 10);
    }

    /// The EFB's "Passenger cabin" switch: shown unless the pilot turns it
    /// off, and a string from an older writer reads the same as a bool.
    #[test]
    fn the_cabin_is_shown_unless_switched_off() {
        assert!(parse(&Map::new()).cabin_visible);
        assert!(!parse(&map(&[("cabinVisible", Value::Bool(false))])).cabin_visible);
        assert!(!parse(&map(&[("cabinVisible", Value::String("false".into()))])).cabin_visible);
        assert!(parse(&map(&[("cabinVisible", Value::Bool(true))])).cabin_visible);
    }

    #[test]
    fn a_zero_frame_interval_falls_back_to_one_rather_than_panicking_a_modulo() {
        let m = map(&[("stateDumpFrames", Value::String("0".into()))]);
        assert_eq!(parse(&m).state_dump_frames, 1);
    }

    #[test]
    fn an_unparsable_number_falls_back_to_the_default() {
        let m = map(&[("failureRate", Value::String("not a number".into()))]);
        assert_eq!(parse(&m).failure_rate, AppSettings::default().failure_rate);
    }

    #[test]
    fn a_missing_file_reads_as_the_shipped_defaults() {
        let path = std::env::temp_dir().join(format!("xphfbw-app-settings-missing-{}.json", std::process::id()));
        let _ = std::fs::remove_file(&path);
        assert_eq!(load_from_path(&path), AppSettings::default());
    }

    #[test]
    fn round_trips_through_a_real_file() {
        let path = std::env::temp_dir().join(format!("xphfbw-app-settings-{}.json", std::process::id()));
        std::fs::write(&path, r#"{"randomFailures": true, "failureRate": "3", "stateDumpFrames": "75"}"#).unwrap();
        let s = load_from_path(&path);
        assert!(s.random_failures);
        assert_eq!(s.failure_rate, 3.0);
        assert_eq!(s.state_dump_frames, 75);
        // Untouched keys keep their default.
        assert!(s.persistence);
        std::fs::remove_file(&path).ok();
    }

    #[test]
    fn a_corrupt_file_reads_as_the_shipped_defaults_instead_of_failing() {
        let path = std::env::temp_dir().join(format!("xphfbw-app-settings-corrupt-{}.json", std::process::id()));
        std::fs::write(&path, b"{ not json").unwrap();
        assert_eq!(load_from_path(&path), AppSettings::default());
        std::fs::remove_file(&path).ok();
    }

    #[test]
    fn a_local_store_tracks_generation_and_the_latest_value() {
        let store = Store::new(AppSettings::default());
        assert_eq!(store.generation(), 0);
        let mut changed = AppSettings::default();
        changed.random_failures = true;
        store.set(changed);
        assert_eq!(store.generation(), 1);
        assert!(store.get().random_failures);
        store.set(changed); // Same value, still a real reload: still bumps.
        assert_eq!(store.generation(), 2);
    }

    #[test]
    fn the_settings_lock_runs_the_closure_and_returns_its_value() {
        assert_eq!(with_settings_lock(|| 42), 42);
    }

    #[test]
    fn the_settings_lock_can_be_taken_again_after_a_previous_call_released_it() {
        with_settings_lock(|| {});
        with_settings_lock(|| {});
    }

    /// A real mutual-exclusion test: many threads each do a
    /// load-yield-store (not an atomic RMW) on a shared counter inside the
    /// lock. If `with_settings_lock` failed to exclude, interleaved
    /// load/store pairs would lose increments and the final count would
    /// come out low; with correct exclusion it always matches exactly.
    #[test]
    fn the_settings_lock_serialises_concurrent_writers() {
        use std::sync::atomic::AtomicI64;
        use std::sync::Arc;
        let counter = Arc::new(AtomicI64::new(0));
        let handles: Vec<_> = (0..8)
            .map(|_| {
                let counter = counter.clone();
                std::thread::spawn(move || {
                    for _ in 0..500 {
                        with_settings_lock(|| {
                            let v = counter.load(Ordering::Relaxed);
                            std::thread::yield_now();
                            counter.store(v + 1, Ordering::Relaxed);
                        });
                    }
                })
            })
            .collect();
        for h in handles {
            h.join().unwrap();
        }
        assert_eq!(counter.load(Ordering::Relaxed), 8 * 500);
    }

    #[test]
    fn a_panic_inside_the_lock_still_releases_it() {
        let result = std::panic::catch_unwind(|| {
            with_settings_lock(|| panic!("boom"));
        });
        assert!(result.is_err());
        // If the mutex were left held, this would deadlock the test run
        // (WaitForSingleObject with INFINITE); reaching this line proves
        // it was released.
        with_settings_lock(|| {});
    }
}
