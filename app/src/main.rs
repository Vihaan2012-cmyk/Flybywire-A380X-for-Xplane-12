//! XPHFBW: the FlyByWire A380X's companion app for X-Plane 12
//! (docs/briefs/xphfbw-app.md).
//!
//! One executable in several roles:
//! - CEF's own subprocesses (renderer, GPU, utility) when CEF starts it with
//!   `--type=...`;
//! - the app itself: FlyByWire's systems on a thread of their own, serving
//!   the plugin over shared memory; the settings window; a small local web
//!   server the settings page talks to;
//! - `XPHFBW.exe --show` asks a running app to show its window.
//!
//! Started by the plugin as `XPHFBW.exe <connection tag> <X-Plane pid>
//! --xp-root=<X-Plane folder>`; it ends when X-Plane does.

#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")]

mod crash;
mod logging;
mod renderer;
mod scheme;
mod settings;
mod tray;
mod views;
mod web;
mod window;

use std::path::{Path, PathBuf};
use std::sync::atomic::AtomicBool;
use std::sync::{Arc, Mutex};

use cef::*;

/// What the settings page shows about the running app.
#[derive(Default)]
pub struct Status {
    pub systems_running: bool,
    pub systems_error: Option<String>,
    pub xplane_connected: bool,
}

pub struct Shared {
    /// The session tag the plugin started us with (`None` for a bare
    /// `--show` relaunch with no session). The browser process needs this
    /// to tag CEF's child processes (`renderer::append_child_switches`).
    pub tag: Option<String>,
    pub xplane_root: PathBuf,
    /// The converted aircraft's folder (`--aircraft=`), for gauge views'
    /// `readFile`/`aircraftDir` (`renderer.rs`).
    pub aircraft_dir: PathBuf,
    /// `app/js` next to the exe (installed as `XPHFBW/js`): agent F's MSFS
    /// runtime, served by `scheme.rs`'s `xphfbw://runtime/...`.
    pub runtime_dir: PathBuf,
    pub ui_dir: PathBuf,
    pub port: std::sync::atomic::AtomicU16,
    pub status: Mutex<Status>,
    pub show_requested: AtomicBool,
}

const SHOW_EVENT: &str = "Local\\XPHFBW_show_window";

fn main() {
    let _ = api_hash(sys::CEF_API_VERSION_LAST, 0);
    let args = cef::args::Args::new();
    let Some(cmd_line) = args.as_cmd_line() else { return };
    let is_browser_process = cmd_line.has_switch(Some(&CefString::from("type"))) != 1;

    let mut app = window::XphfbwApp::new();
    let code = execute_process(Some(args.as_main_args()), Some(&mut app), std::ptr::null_mut());
    if !is_browser_process {
        std::process::exit(code.max(0));
    }
    logging::init();

    let argv: Vec<String> = std::env::args().skip(1).collect();
    let wants_show = argv.iter().any(|a| a == "--show");
    let positional: Vec<&String> = argv.iter().filter(|a| !a.starts_with("--")).collect();
    let tag = positional.first().map(|s| s.to_string());
    let parent_pid = positional.get(1).and_then(|p| p.parse::<u32>().ok());

    // A second launch only asks the running app to show its window.
    if tag.is_none() && window::signal_existing(SHOW_EVENT) {
        return;
    }

    let exe_dir = std::env::current_exe().ok().and_then(|p| p.parent().map(PathBuf::from)).unwrap_or_default();
    let xplane_root = argv
        .iter()
        .find_map(|a| a.strip_prefix("--xp-root=").map(PathBuf::from))
        .or_else(settings::find_xplane)
        .unwrap_or_default();
    fbw_a380_systems::study_json::set_xplane_root(xplane_root.clone());
    let aircraft_dir = argv.iter().find_map(|a| a.strip_prefix("--aircraft=").map(PathBuf::from)).unwrap_or_default();
    // Every cockpit screen's gauge HTML is served from here
    // (views.rs::gauge_url() -> coui://html_ui/Pages/VCockpit/Instruments/...,
    // resolved straight to <aircraft>/html_ui/... by scheme.rs::resolve_coui).
    // It is NOT written by the converter (msfs2xp-aircraft's html_ui output
    // is Fonts/Images only); tools/install.sh has to merge it in from
    // FlyByWire's own build. An install that skips that merge (2026-09-25's
    // manual swap) leaves this file absent and every screen just stays
    // black, with nothing in the log to say why until the first gauge 404s
    // (scheme.rs's not_found, views.rs's ViewLoad) -- check for it here,
    // once, as loudly as possible, before any of that.
    if let Some(pfd) = missing_gauge_payload(&aircraft_dir) {
        logging::log(&format!(
            "app: FATAL: {} is missing -- this install's html_ui/ was never merged with FlyByWire's built instruments (tools/install.sh or docs/js-build.md). Every cockpit screen will be BLACK.",
            pfd.display()
        ));
    }
    let shared = Arc::new(Shared {
        tag: tag.clone(),
        xplane_root,
        aircraft_dir,
        runtime_dir: exe_dir.join("js"),
        ui_dir: exe_dir.join("ui"),
        port: std::sync::atomic::AtomicU16::new(0),
        status: Mutex::new(Status::default()),
        // The window opens with the app, also when X-Plane starts it.
        show_requested: AtomicBool::new(true),
    });
    crash::install(shared.clone());

    // Outlive nobody. The plugin passes its own process id as the second
    // positional argument, and until now the only thing that watched it was
    // the systems thread below -- which exists only when the plugin also
    // chose this app as its *systems* backend. Started for the instrument
    // browsers alone, or with the plugin running the systems in-process or
    // in fbw_a380_systems_server.exe, nothing watched anything: X-Plane
    // exited and this app and its CEF children stayed up indefinitely,
    // still holding their GPU surfaces (five of them were found alive an
    // hour and a half after X-Plane had gone, the parent still spawning
    // renderers).
    //
    // A parent id given at all now means "die with that process", whatever
    // else this instance is doing. A standalone launch passes none and is
    // unaffected.
    if let Some(pid) = parent_pid {
        std::thread::Builder::new()
            .name("parent watch".into())
            .spawn(move || {
                use fbw_a380_systems::remote::win;
                let Some(parent) = win::Process::open(pid) else {
                    // Already gone, or not ours to open: either way there is
                    // nothing to serve.
                    logging::log(&format!("app: parent {pid} could not be opened; exiting"));
                    window::quit_from_any_thread();
                    std::thread::sleep(std::time::Duration::from_secs(5));
                    std::process::exit(0);
                };
                while win::wait_any(&[parent.handle()], 1000).is_none() {}
                logging::log(&format!("app: parent {pid} has exited; shutting down"));
                window::quit_from_any_thread();
                // Same reasoning as the systems thread: CEF's orderly
                // shutdown waits on every instrument browser and can hang
                // with nobody left to close them.
                std::thread::sleep(std::time::Duration::from_secs(5));
                logging::log("app: shutdown did not finish after the parent went; exiting");
                std::process::exit(0);
            })
            .ok();
    }

    // FlyByWire's systems, for the plugin that started us.
    if let Some(tag) = tag.clone() {
        let s = shared.clone();
        let _ = std::thread::Builder::new().name("fbw systems".into()).stack_size(256 << 20).spawn(move || {
            {
                let mut st = s.status.lock().unwrap();
                st.systems_running = true;
                st.xplane_connected = true;
            }
            let result = fbw_a380_systems::serve_systems(&tag, parent_pid);
            let mut st = s.status.lock().unwrap();
            st.systems_running = false;
            st.xplane_connected = false;
            if let Err(e) = result {
                st.systems_error = Some(e);
            }
            drop(st);
            // X-Plane has gone (or the plugin quit): the app goes with it.
            window::quit_from_any_thread();
            // Chromium's orderly shutdown waits on every instrument browser
            // and can hang with nobody left to close them; never outlive
            // X-Plane by more than a few seconds (CEF's own child processes
            // end with this one).
            std::thread::sleep(std::time::Duration::from_secs(5));
            logging::log("app: X-Plane has gone and shutdown did not finish; exiting");
            std::process::exit(0);
        });
    }

    web::start(shared.clone());
    window::watch_show_requests(SHOW_EVENT, shared.clone());
    window::watch_restart_displays();
    tray::start(shared.clone());

    // Chromium locks its cache folder to one process: the copy X-Plane starts
    // and one opened by hand (the settings window alone) each keep their own,
    // so either can run while the other is up.
    let cache = std::env::var("LOCALAPPDATA")
        .map(PathBuf::from)
        .unwrap_or_else(|_| exe_dir.clone())
        .join("XPHFBW")
        .join(if tag.is_some() { "cache-session" } else { "cache" });
    let settings = Settings {
        no_sandbox: 1,
        windowless_rendering_enabled: 1,
        root_cache_path: CefString::from(cache.to_string_lossy().as_ref()),
        log_severity: LogSeverity::WARNING,
        // CEF's own debug.log, next to the app's own log file (logging.rs).
        log_file: CefString::from(logging::cef_log_path().to_string_lossy().as_ref()),
        ..Default::default()
    };
    let _ = window::SHARED.set(shared.clone());
    if initialize(Some(args.as_main_args()), Some(&settings), Some(&mut app), std::ptr::null_mut()) != 1 {
        std::process::exit(1);
    }
    run_message_loop();
    shutdown();
}

/// The cockpit's gauge HTML this app needs at start: any one file proves
/// `<aircraft>/html_ui/` actually has FlyByWire's built instruments merged
/// in (`tools/install.sh`), not just the converter's own Fonts/Images. Picks
/// the PFD specifically since that is what `docs/briefs/debug.md`'s
/// coordinator called out by name and what `P10-build-install.ps1`'s
/// `Test-GaugesInstalled` already checks explicitly, so this mirrors an
/// already-agreed single point of failure rather than inventing a new one.
/// Returns `None` when `aircraft_dir` is empty (no `--aircraft=` given, e.g.
/// a bare `--show` relaunch) since there is nothing installed to check yet,
/// or when the file is present.
fn missing_gauge_payload(aircraft_dir: &Path) -> Option<PathBuf> {
    if aircraft_dir.as_os_str().is_empty() {
        return None;
    }
    let pfd = aircraft_dir.join("html_ui/Pages/VCockpit/Instruments/A380X/PFD/pfd.html");
    if pfd.exists() { None } else { Some(pfd) }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn missing_gauge_payload_is_none_with_no_aircraft_dir() {
        assert_eq!(missing_gauge_payload(Path::new("")), None);
    }

    #[test]
    fn missing_gauge_payload_names_the_missing_pfd_html() {
        let dir = std::env::temp_dir().join("xphfbw-test-missing-gauge-payload");
        let _ = std::fs::remove_dir_all(&dir);
        assert_eq!(
            missing_gauge_payload(&dir),
            Some(dir.join("html_ui/Pages/VCockpit/Instruments/A380X/PFD/pfd.html"))
        );
    }

    #[test]
    fn missing_gauge_payload_is_none_once_the_file_exists() {
        let dir = std::env::temp_dir().join("xphfbw-test-missing-gauge-payload-present");
        let pfd_dir = dir.join("html_ui/Pages/VCockpit/Instruments/A380X/PFD");
        std::fs::create_dir_all(&pfd_dir).unwrap();
        std::fs::write(pfd_dir.join("pfd.html"), "x").unwrap();
        assert_eq!(missing_gauge_payload(&dir), None);
        let _ = std::fs::remove_dir_all(&dir);
    }
}
