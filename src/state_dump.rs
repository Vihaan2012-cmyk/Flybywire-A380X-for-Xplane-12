//! A dump of the aircraft's whole state, every variable, every
//! `xphfbw.stateDumpFrames` frames (`app_settings.rs`, default
//! [`DEFAULT_EVERY_TICKS`]), to `D:\fbw-build\state-dumps`, for reading
//! what the systems did after a flight.
//!
//! `xphfbw.stateDumps`/`stateDumpFrames`/`stateDumpKeep` (the Simulation
//! settings panel) apply live: [`StateDump::tick`] and [`write`] read
//! `app_settings::current()` on every call, so toggling dumps off, or
//! changing the interval or the keep count, takes effect on the very next
//! tick/dump without an aircraft reload.
//!
//! `FBW_DUMP` turns the same dumps on from the environment, for a session
//! started from a script or a shell rather than through the settings panel
//! -- including one already in the air, since nothing here is read once at
//! start-up. `FBW_DUMP=1` uses the shipped interval; `FBW_DUMP=<frames>`
//! sets it. It overrides the `xphfbw.stateDumps` switch rather than
//! consulting it, because the point of it is to dump on a run where nobody
//! got to the panel first. The other two settings still apply, so the
//! interval can be adjusted mid-session from the panel even on an
//! environment-started dump.
//!
//! The flight loop only copies the values; a thread of its own formats and
//! writes them, so a dump never costs a frame. Each X-Plane session gets a
//! folder; a session keeps its latest `xphfbw.stateDumpKeep` dumps
//! (default [`DEFAULT_KEEP_PER_SESSION`]) and only the latest
//! [`KEEP_SESSIONS`] sessions are kept, so the dumps stay a few tens of
//! megabytes however long the simulator runs.

use std::io::Write;
use std::path::{Path, PathBuf};
use std::sync::mpsc::{sync_channel, SyncSender, TrySendError};
use std::sync::Arc;

/// `Plugin::tick` (lib.rs) offers this module every tick (`% 1` is always
/// `0`): the real, user-configurable interval (`xphfbw.stateDumpFrames`)
/// is checked live inside [`StateDump::tick`] itself, so a settings change
/// never needs that call site to change too.
pub const EVERY_TICKS: u64 = 1;
/// `xphfbw.stateDumpFrames`'s shipped default (`app_settings.rs`), and the
/// interval used while the app's settings cannot be read yet.
pub const DEFAULT_EVERY_TICKS: u64 = 200;
const ROOT: &str = r"D:\fbw-build\state-dumps";
/// `xphfbw.stateDumpKeep`'s shipped default.
pub const DEFAULT_KEEP_PER_SESSION: usize = 60;
const KEEP_SESSIONS: usize = 3;

struct Dump {
    time: f64,
    ticks: u64,
    names: Arc<Vec<String>>,
    datarefs: Arc<Vec<String>>,
    values: Vec<f64>,
    sources: Vec<u8>,
}

pub struct StateDump {
    sender: Option<SyncSender<Dump>>,
    names: Arc<Vec<String>>,
    datarefs: Arc<Vec<String>>,
}

impl StateDump {
    /// Starts the writer thread; without a folder to write to, dumping is off.
    pub fn start() -> Self {
        let off = Self { sender: None, names: Arc::default(), datarefs: Arc::default() };
        let Ok(session) = session_folder(Path::new(ROOT)) else { return off };
        // One dump waiting at most: a slow disk skips dumps, never frames.
        let (sender, receiver) = sync_channel::<Dump>(1);
        let spawned = std::thread::Builder::new().name("fbw state dump".into()).spawn(move || {
            for dump in receiver {
                if let Err(e) = write(&session, &dump) {
                    crate::log(&format!("state dump: {e}"));
                }
            }
        });
        if spawned.is_err() {
            return off;
        }
        crate::log(&format!(
            "state dump: every variable every N frames (xphfbw.stateDumpFrames, default {DEFAULT_EVERY_TICKS}) under {ROOT}"
        ));
        Self { sender: Some(sender), ..off }
    }

    /// Whether a dump could possibly happen this tick: the writer thread is
    /// still alive and `xphfbw.stateDumps` is (live) on. Cheap (an
    /// `Option::is_some` plus one small `RwLock` read, see
    /// `app_settings::current`'s own doc comment) — cheap enough that
    /// `lib.rs`'s tick loop calls it *before* paying for the shared
    /// snapshot `Mutex` lock, instead of locking that Mutex (which the
    /// panel thread also reads) on every single tick only to hand
    /// [`Self::tick`] a reference it was always going to throw away
    /// whenever dumps are off — the common case once `stateDumps` defaults
    /// to off (see `docs/deep/debug_start_fps.md`).
    pub fn enabled(&self) -> bool {
        self.sender.is_some() && (env_frames().is_some() || crate::app_settings::current().state_dumps)
    }

    /// Called once a tick with the tick's state; dumps every
    /// `xphfbw.stateDumpFrames`th tick, or never while `xphfbw.stateDumps`
    /// is off — both checked live, every call.
    pub fn tick(&mut self, time: f64, ticks: u64, names: &[String], datarefs: &[String], values: &[f64], sources: &[u8]) {
        let Some(sender) = &self.sender else { return };
        let settings = crate::app_settings::current();
        let forced = env_frames();
        // `FBW_DUMP=<frames>` sets the interval; `FBW_DUMP=1` means "on",
        // which is one frame -- an interval nobody wants and a disk nobody
        // has, so it reads as "the shipped interval" instead. Anything
        // larger is taken literally.
        let every = match forced {
            Some(1) | None => settings.state_dump_frames.max(1),
            Some(frames) => frames,
        };
        if (forced.is_none() && !settings.state_dumps) || ticks == 0 || ticks % every != 0 {
            return;
        }
        // Names change only when a variable is registered.
        if self.names.len() != names.len() {
            self.names = Arc::new(names.to_vec());
            self.datarefs = Arc::new(datarefs.to_vec());
        }
        let dump = Dump {
            time,
            ticks,
            names: self.names.clone(),
            datarefs: self.datarefs.clone(),
            values: values.to_vec(),
            sources: sources.to_vec(),
        };
        if let Err(TrySendError::Disconnected(_)) = sender.try_send(dump) {
            self.sender = None;
        }
    }
}

/// A new folder for this session under `root`, the oldest sessions beyond
/// [`KEEP_SESSIONS`] removed.
fn session_folder(root: &Path) -> std::io::Result<PathBuf> {
    std::fs::create_dir_all(root)?;
    let secs = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map_or(0, |d| d.as_secs());
    let session = root.join(format!("session-{secs}"));
    std::fs::create_dir_all(&session)?;
    prune(root, "session-", KEEP_SESSIONS);
    Ok(session)
}

/// Keep the newest `keep` entries of `dir` whose names start with `prefix`
/// (names sort by their number, which only grows).
fn prune(dir: &Path, prefix: &str, keep: usize) {
    let Ok(entries) = std::fs::read_dir(dir) else { return };
    let mut found: Vec<(u64, PathBuf)> = entries
        .flatten()
        .filter_map(|e| {
            let name = e.file_name().to_string_lossy().into_owned();
            let number = name.strip_prefix(prefix)?.split('.').next()?.parse::<u64>().ok()?;
            Some((number, e.path()))
        })
        .collect();
    found.sort_unstable_by_key(|(n, _)| std::cmp::Reverse(*n));
    for (_, path) in found.into_iter().skip(keep) {
        let _ = if path.is_dir() { std::fs::remove_dir_all(&path) } else { std::fs::remove_file(&path) };
    }
}

/// `FBW_DUMP`'s frame interval, or `None` when it is unset or off.
///
/// Read once: an environment variable cannot change inside a running
/// process, and this is called on every tick that could dump.
fn env_frames() -> Option<u64> {
    use std::sync::OnceLock;
    static FRAMES: OnceLock<Option<u64>> = OnceLock::new();
    *FRAMES.get_or_init(|| {
        let raw = std::env::var("FBW_DUMP").ok()?;
        let raw = raw.trim();
        if raw.is_empty() || raw == "0" || raw.eq_ignore_ascii_case("off") {
            return None;
        }
        Some(raw.parse::<u64>().ok().filter(|f| *f > 0).unwrap_or(1))
    })
}

fn write(session: &Path, dump: &Dump) -> std::io::Result<()> {
    let path = session.join(format!("dump-{:010}.tsv", dump.ticks));
    let partial = path.with_extension("tsv.part");
    {
        let mut out = std::io::BufWriter::new(std::fs::File::create(&partial)?);
        format(&mut out, dump)?;
        out.flush()?;
    }
    std::fs::rename(&partial, &path)?;
    let keep = crate::app_settings::current().state_dump_keep.max(1);
    prune(session, "dump-", keep);
    Ok(())
}

fn format(out: &mut impl Write, dump: &Dump) -> std::io::Result<()> {
    writeln!(out, "# sim time {:.3} s, frame {}, {} variables", dump.time, dump.ticks, dump.values.len())?;
    writeln!(out, "# source: 0 nothing yet, 1 X-Plane, 2 the systems")?;
    writeln!(out, "name\tvalue\tsource\tdataref")?;
    for (i, value) in dump.values.iter().enumerate() {
        let name = dump.names.get(i).map_or("", String::as_str);
        let dataref = dump.datarefs.get(i).map_or("", String::as_str);
        let source = dump.sources.get(i).copied().unwrap_or(0);
        writeln!(out, "{name}\t{value}\t{source}\t{dataref}")?;
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_dump_lists_every_variable_with_its_value_and_source() {
        let dump = Dump {
            time: 12.5,
            ticks: 400,
            names: Arc::new(vec!["A32NX_EXT_PWR_AVAIL:1".into(), "AMBIENT PRESSURE".into()]),
            datarefs: Arc::new(vec!["fbw/A32NX_EXT_PWR_AVAIL_1".into(), "fbw/AMBIENT_PRESSURE".into()]),
            values: vec![1., 29.92],
            sources: vec![2, 1],
        };
        let mut text = Vec::new();
        format(&mut text, &dump).unwrap();
        let text = String::from_utf8(text).unwrap();
        assert!(text.starts_with("# sim time 12.500 s, frame 400, 2 variables\n"), "{text}");
        assert!(text.contains("A32NX_EXT_PWR_AVAIL:1\t1\t2\tfbw/A32NX_EXT_PWR_AVAIL_1\n"), "{text}");
        assert!(text.contains("AMBIENT PRESSURE\t29.92\t1\tfbw/AMBIENT_PRESSURE\n"), "{text}");
    }

    #[test]
    fn only_the_newest_dumps_are_kept() {
        let dir = std::env::temp_dir().join(format!("fbw-dump-prune-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        for n in [5, 400, 200, 600, 1000] {
            std::fs::write(dir.join(format!("dump-{n:010}.tsv")), "x").unwrap();
        }
        std::fs::write(dir.join("other.txt"), "x").unwrap();
        prune(&dir, "dump-", 2);
        let mut left: Vec<String> = std::fs::read_dir(&dir).unwrap().flatten().map(|e| e.file_name().to_string_lossy().into_owned()).collect();
        left.sort();
        assert_eq!(left, ["dump-0000000600.tsv", "dump-0000001000.tsv", "other.txt"]);
        std::fs::remove_dir_all(&dir).unwrap();
    }
}

