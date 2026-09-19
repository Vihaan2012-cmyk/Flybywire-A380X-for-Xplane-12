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

use std::path::PathBuf;
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
