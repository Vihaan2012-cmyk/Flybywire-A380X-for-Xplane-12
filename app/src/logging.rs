//! The app's own log file: `%LOCALAPPDATA%\XPHFBW\logs\xphfbw-<date>.log`,
//! rotated to the newest [`KEEP`] files; a background line every
//! [`STATUS_POLL`] recording the systems thread's status
//! (docs/briefs/xphfbw-app.md, agent L scope 2: "capturing app events and
//! the systems thread status").
//!
//! CEF has its own `debug.log`; [`cef_log_path`] returns the path in this
//! same folder that whoever builds CEF's `Settings` should set as
//! `log_file` (told to agent G/lead per the brief).

use std::fs::{File, OpenOptions};
use std::io::{Read, Write};
use std::path::{Path, PathBuf};
use std::sync::{Mutex, OnceLock};
use std::time::Duration;

use windows_sys::Win32::Foundation::SYSTEMTIME;
use windows_sys::Win32::System::SystemInformation::GetLocalTime;

/// How many `xphfbw-<date>.log` files to keep (oldest deleted on [`init`]).
const KEEP: usize = 10;
/// How often the background thread logs the systems thread's status.
const STATUS_POLL: Duration = Duration::from_secs(30);

static LOG_FILE: OnceLock<Mutex<File>> = OnceLock::new();
static LOG_PATH: OnceLock<PathBuf> = OnceLock::new();
static LOG_DIR: OnceLock<PathBuf> = OnceLock::new();

/// The app's log folder, creating it if it does not exist yet.
pub fn dir() -> PathBuf {
    if let Some(d) = LOG_DIR.get() {
        return d.clone();
    }
    let base = std::env::var("LOCALAPPDATA").map(PathBuf::from).unwrap_or_else(|_| std::env::temp_dir());
    let d = base.join("XPHFBW").join("logs");
    let _ = std::fs::create_dir_all(&d);
    // Another thread may have raced us to set it; either value is the same path.
    let _ = LOG_DIR.set(d.clone());
    d
}

/// The path CEF's own debug log should be told to use, in the same folder
/// as the app's log. Agent G/lead: set `Settings.log_file` to this
/// (`CefString::from(logging::cef_log_path().to_string_lossy().as_ref())`).
///
/// `#[allow(dead_code)]`: unused until whoever builds `Settings` calls it;
/// remove once they do.
#[allow(dead_code)]
pub fn cef_log_path() -> PathBuf {
    dir().join("cef-debug.log")
}

/// Opens (rotating first) today's log file and starts the periodic status
/// line. Call once, as early as possible in `main`.
pub fn init() {
    let d = dir();
    rotate(&d);
    let path = d.join(format!("xphfbw-{}.log", today()));
    if let Ok(file) = OpenOptions::new().create(true).append(true).open(&path) {
        let _ = LOG_FILE.set(Mutex::new(file));
        let _ = LOG_PATH.set(path);
    }
    log("app: started, log file open");
    let _ = std::thread::Builder::new().name("app log status".into()).spawn(poll_status);
}

/// Every [`STATUS_POLL`], a line with the systems thread's status, once
/// `main` has published `Shared` to `window::SHARED`.
fn poll_status() {
    // `XPHFBW_STATUS_SECS` shortens the interval for a diagnostic run (a
    // throwaway instance with no plugin behind it only lives a few seconds).
    let every = std::env::var("XPHFBW_STATUS_SECS").ok().and_then(|v| v.trim().parse::<u64>().ok()).filter(|&s| s > 0).map_or(STATUS_POLL, Duration::from_secs);
    loop {
        std::thread::sleep(every);
        let Some(shared) = crate::window::SHARED.get() else { continue };
        let Ok(st) = shared.status.lock() else { continue };
        let error = st.systems_error.as_deref().map(|e| format!(" error=\"{e}\"")).unwrap_or_default();
        log(&format!("status: xplane_connected={} systems_running={}{error}; {}", st.xplane_connected, st.systems_running, crate::views::paint_summary()));
    }
}

/// Appends one timestamped line to the app log; also to stderr in debug
/// builds. Safe from any thread, and safe to call before [`init`] runs (the
/// line is simply dropped, matching how `crate::` logging elsewhere in this
/// codebase already tolerates a not-yet-ready sink).
pub fn log(message: &str) {
    #[cfg(debug_assertions)]
    eprintln!("[xphfbw] {message}");
    let Some(file) = LOG_FILE.get() else { return };
    if let Ok(mut f) = file.lock() {
        let _ = writeln!(f, "{} {message}", now_string());
        let _ = f.flush();
    }
}

/// The last `lines` lines of the current log file, e.g. for a crash report.
pub fn tail(lines: usize) -> String {
    let Some(path) = LOG_PATH.get() else { return String::new() };
    let Ok(mut file) = File::open(path) else { return String::new() };
    let mut text = String::new();
    if file.read_to_string(&mut text).is_err() {
        return String::new();
    }
    let all: Vec<&str> = text.lines().collect();
    let start = all.len().saturating_sub(lines);
    all[start..].join("\n")
}

/// `YYYYMMDD-HHMMSS`, safe as part of a path (no `:`), for a crash report
/// folder's name.
pub fn filename_timestamp() -> String {
    let st = local_now();
    format!("{:04}{:02}{:02}-{:02}{:02}{:02}", st.wYear, st.wMonth, st.wDay, st.wHour, st.wMinute, st.wSecond)
}

fn rotate(dir: &Path) {
    let Ok(entries) = std::fs::read_dir(dir) else { return };
    let mut files: Vec<PathBuf> = entries
        .flatten()
        .map(|e| e.path())
        .filter(|p| p.file_name().and_then(|n| n.to_str()).is_some_and(|n| n.starts_with("xphfbw-") && n.ends_with(".log")))
        .collect();
    files.sort(); // the date in the name sorts oldest-first lexically
    while files.len() >= KEEP {
        let oldest = files.remove(0);
        let _ = std::fs::remove_file(oldest);
    }
}

fn local_now() -> SYSTEMTIME {
    unsafe {
        let mut st: SYSTEMTIME = std::mem::zeroed();
        GetLocalTime(&mut st);
        st
    }
}

fn today() -> String {
    let st = local_now();
    format!("{:04}-{:02}-{:02}", st.wYear, st.wMonth, st.wDay)
}

fn now_string() -> String {
    let st = local_now();
    format!("{:04}-{:02}-{:02} {:02}:{:02}:{:02}", st.wYear, st.wMonth, st.wDay, st.wHour, st.wMinute, st.wSecond)
}
