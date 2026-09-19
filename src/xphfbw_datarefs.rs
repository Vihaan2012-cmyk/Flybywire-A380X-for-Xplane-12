//! Custom datarefs and commands for XPHFBW
//! (docs/briefs/xphfbw-js-bridge.md, "Custom datarefs and commands"): status
//! any dataref reader (cockpit, Lua, other plugins) can watch, three
//! commands, and the aircraft menu's "XPHFBW settings" entry.
//!
//! Status comes from what `start_systems`/`Plugin::tick` (lib.rs) know about
//! the chosen systems backend, and — once the session's `SlotTable` exists
//! (agent C, xphfbw_host.rs) — from its `displays_active` header field, read
//! here by opening the same session tag (`crate::session_tag`: "the systems
//! tag is the session tag"). `xphfbw_bridge.rs`'s layout is frozen, so this
//! only reads a field already there; `views_loaded` has no such field yet,
//! so `set_views_loaded` is a hook for whoever tracks `Uplink::Loaded` acks
//! (agent C) to call, rather than a new wire field.

use std::sync::atomic::{AtomicU32, Ordering};

use crate::published::{self, Command, Published, Value};
use crate::remote::win::Event;
use crate::xp::{MenuId, Xplm};
use crate::xphfbw_bridge;

/// The named event XPHFBW's app already opens at start-up
/// (app/src/window.rs `SHOW_EVENT`/`watch_show_requests`): setting it raises
/// the settings window. A second `XPHFBW.exe --show` launch uses the same
/// event (app/src/main.rs `signal_existing`); this is that path without
/// spawning a process.
const SHOW_EVENT: &str = "Local\\XPHFBW_show_window";
/// Not consumed by the app yet: reserved for a `watch_show_requests`-style
/// listener next to it in app/src/window.rs (agent E/G), which would restart
/// every instrument view's off-screen browser.
const RESTART_DISPLAYS_EVENT: &str = "Local\\XPHFBW_restart_displays";

fn signal(name: &str) {
    if let Some(event) = Event::open(name) {
        event.set();
    }
}

/// Ask a running XPHFBW.exe to raise its settings window.
pub(crate) fn signal_show_app() {
    signal(SHOW_EVENT);
}

/// Ask a running XPHFBW.exe to restart its instrument views.
pub(crate) fn signal_restart_displays() {
    signal(RESTART_DISPLAYS_EVENT);
}

static VIEWS_LOADED: AtomicU32 = AtomicU32::new(0);

/// Hook for whoever tracks per-view `Uplink::Loaded { ok: true, .. }` acks
/// (agent C, xphfbw_host.rs) to publish through `xphfbw/views_loaded`.
#[allow(dead_code)]
pub(crate) fn set_views_loaded(n: u32) {
    VIEWS_LOADED.store(n, Ordering::Relaxed);
}

/// What `start_systems`/`Plugin::tick` know about the chosen backend, for
/// [`XphfbwDatarefs::update`].
#[derive(Default)]
pub(crate) struct Status {
    /// XPHFBW.exe is the chosen backend (not the `fbw_a380_systems_server.exe`
    /// fallback, and not running in the plugin itself).
    pub app_running: bool,
    /// FlyByWire's systems are running in their own process, XPHFBW.exe or
    /// `fbw_a380_systems_server.exe`.
    pub systems_remote: bool,
    pub systems_round_trip_ms: f64,
    pub systems_late_ticks: u32,
}

/// Commands pressed since the last [`XphfbwDatarefs::poll_commands`] call.
pub(crate) struct Commands {
    pub show_app: bool,
    pub restart_displays: bool,
    pub restart_app: bool,
}

pub(crate) struct XphfbwDatarefs {
    _published: Published,
    app_running: Value,
    displays_active: Value,
    systems_remote: Value,
    systems_round_trip_ms: Value,
    systems_late_ticks: Value,
    views_loaded: Value,
    show_app: Command,
    restart_displays: Command,
    restart_app: Command,
    /// The session's slots, opened once a tag exists, to read
    /// `displays_active` (rule 7, docs/briefs/xphfbw-js-bridge.md). Re-opened
    /// whenever the tag changes (a new plugin/systems session).
    slots: Option<(String, xphfbw_bridge::SlotTable)>,
}

impl XphfbwDatarefs {
    pub fn new() -> Self {
        let mut p = Published::default();
        Self {
            app_running: p.number("xphfbw/app_running", 0., false),
            displays_active: p.number("xphfbw/displays_active", 0., false),
            systems_remote: p.number("xphfbw/systems_remote", 0., false),
            systems_round_trip_ms: p.number("xphfbw/systems_round_trip_ms", 0., false),
            systems_late_ticks: p.number("xphfbw/systems_late_ticks", 0., false),
            views_loaded: p.number("xphfbw/views_loaded", 0., false),
            show_app: p.command("xphfbw/show_app", "XPHFBW: show the settings window"),
            restart_displays: p.command("xphfbw/restart_displays", "XPHFBW: restart the instrument displays"),
            restart_app: p.command("xphfbw/restart_app", "XPHFBW: restart the systems and displays"),
            _published: p,
            slots: None,
        }
    }

    /// Commands pressed since the last call; the caller (`Plugin::tick`) acts
    /// on them, since restarting the systems needs `&mut Vars` this module
    /// does not have.
    pub fn poll_commands(&mut self) -> Commands {
        Commands {
            show_app: published::presses(self.show_app) > 0,
            restart_displays: published::presses(self.restart_displays) > 0,
            restart_app: published::presses(self.restart_app) > 0,
        }
    }

    /// Publish this tick's status.
    pub fn update(&mut self, status: &Status, session_tag: Option<&str>) {
        published::set(self.app_running, status.app_running as u8 as f64);
        published::set(self.systems_remote, status.systems_remote as u8 as f64);
        published::set(self.systems_round_trip_ms, status.systems_round_trip_ms);
        published::set(self.systems_late_ticks, status.systems_late_ticks as f64);
        published::set(self.views_loaded, VIEWS_LOADED.load(Ordering::Relaxed) as f64);
        published::set(self.displays_active, self.read_displays_active(session_tag) as f64);
    }

    fn read_displays_active(&mut self, session_tag: Option<&str>) -> u32 {
        let Some(tag) = session_tag else {
            self.slots = None;
            return 0;
        };
        if self.slots.as_ref().map_or(true, |(t, _)| t != tag) {
            self.slots = xphfbw_bridge::SlotTable::open(tag).map(|s| (tag.to_string(), s));
        }
        self.slots.as_ref().map_or(0, |(_, s)| s.header().displays_active.load(Ordering::Relaxed))
    }
}

static mut MENU: MenuId = std::ptr::null_mut();

/// Build the aircraft menu's "XPHFBW settings" entry.
pub fn build_menu(xplm: &Xplm) {
    unsafe {
        let menu = xplm.menu("XPHFBW settings", menu_handler);
        if menu.is_null() {
            crate::log("no XPHFBW settings menu: X-Plane gave the aircraft no menu to hang it on");
            return;
        }
        xplm.menu_item(menu, "Show XPHFBW settings...", std::ptr::null_mut());
        let m = &raw mut MENU;
        *m = menu;
    }
}

pub fn destroy_menu(xplm: &Xplm) {
    unsafe {
        let m = &raw mut MENU;
        xplm.destroy_menu(*m);
        *m = std::ptr::null_mut();
    }
}

unsafe extern "C" fn menu_handler(_menu: *mut std::ffi::c_void, _item: *mut std::ffi::c_void) {
    signal_show_app();
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn commands_are_cleared_after_polling() {
        let mut d = XphfbwDatarefs::new();
        assert!(!d.poll_commands().show_app);
        published::press(d.show_app);
        published::press(d.restart_app);
        let cmds = d.poll_commands();
        assert!(cmds.show_app);
        assert!(cmds.restart_app);
        assert!(!cmds.restart_displays);
        // A press is taken exactly once: the next poll sees nothing new.
        let cmds = d.poll_commands();
        assert!(!cmds.show_app && !cmds.restart_app && !cmds.restart_displays);
    }

    #[test]
    fn status_is_published_with_no_session_open() {
        let mut d = XphfbwDatarefs::new();
        let status = Status { app_running: true, systems_remote: true, systems_round_trip_ms: 12.5, systems_late_ticks: 3 };
        d.update(&status, None);
        assert_eq!(published::get(d.app_running), 1.);
        assert_eq!(published::get(d.systems_remote), 1.);
        assert_eq!(published::get(d.systems_round_trip_ms), 12.5);
        assert_eq!(published::get(d.systems_late_ticks), 3.);
        // No session tag: displays_active reads 0 rather than a stale value.
        assert_eq!(published::get(d.displays_active), 0.);
    }

    #[test]
    fn displays_active_follows_the_sessions_slot_table() {
        let tag = format!("test_xphfbw_datarefs_{}_{}", std::process::id(), std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).unwrap().subsec_nanos());
        let plugin_slots = xphfbw_bridge::SlotTable::create(&tag).expect("the slot table is created");
        plugin_slots.header().displays_active.store(1, Ordering::Relaxed);
        let mut d = XphfbwDatarefs::new();
        d.update(&Status::default(), Some(&tag));
        assert_eq!(published::get(d.displays_active), 1.);
        plugin_slots.header().displays_active.store(0, Ordering::Relaxed);
        d.update(&Status::default(), Some(&tag));
        assert_eq!(published::get(d.displays_active), 0.);
    }

    #[test]
    fn views_loaded_reads_back_what_was_set() {
        set_views_loaded(7);
        let mut d = XphfbwDatarefs::new();
        d.update(&Status::default(), None);
        assert_eq!(published::get(d.views_loaded), 7.);
        set_views_loaded(0);
    }

    #[test]
    fn signalling_with_no_app_running_does_not_panic() {
        // No process has "Local\XPHFBW_show_window" open in the test binary,
        // so this is a no-op rather than a panic or a hang.
        signal_show_app();
        signal_restart_displays();
    }
}
