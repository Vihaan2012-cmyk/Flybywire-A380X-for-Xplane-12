//! CEF: the app object, the settings window, and running work on CEF's UI
//! thread from other threads.

use std::cell::RefCell;
use std::sync::atomic::Ordering;
use std::sync::{Arc, OnceLock};

use cef::*;

use crate::Shared;

pub static SHARED: OnceLock<Arc<Shared>> = OnceLock::new();

thread_local! {
    /// The settings window, while it is open (CEF's UI thread only).
    static WINDOW: RefCell<Option<Window>> = const { RefCell::new(None) };
}

/// Whether to rasterise every instrument on the CPU (`disable-gpu`), from
/// the app's own **"Force safe mode CPU rendering"** setting.
///
/// This used to be unconditional, and the settings checkbox that claims to
/// control it did nothing: there was no way to turn it off. That matters
/// more than it sounds. Chromium then software-rasterises eighteen browser
/// views, several of them 1646x1024 or larger, which is what holds the
/// instruments down to a handful of frames a second no matter how fast the
/// aircraft's own state is changing -- measured with `FBW_SCREEN_STATS=1`,
/// which showed every screen publishing at 1-5 Hz, the simplest page (the
/// SD) reaching 17 and the busiest (the MFD) near zero, with the plugin
/// uploading every frame it was handed. The upload path was never the
/// limit; the rasteriser was.
///
/// The original reason for forcing it -- "no video memory taken from
/// X-Plane" -- was sound when the converted aircraft was over its VRAM
/// budget. It is much less pressing now that the texture work has brought
/// that down, so this is a setting worth being able to change.
///
/// Defaults to **off**, i.e. the GPU draws the instruments.
///
/// It defaulted to on, and was unconditional before that. The measurement
/// is what changed the default: with CPU rasterisation every screen
/// published at 1-5 Hz, the simplest page reached 17 and the busiest sat
/// near zero, and the rate did not rise when the aircraft was accelerating
/// -- so it was not waiting on content, it was waiting on the rasteriser.
/// Eighteen views, several 1646x1024 or larger, is simply more than
/// software rendering can carry, and the instruments are unusable that way.
///
/// The reason it was forced -- "no video memory taken from X-Plane" -- was
/// sound when the converted aircraft was over its VRAM budget. The texture
/// work has since taken that from 2797 MB to 1085 MB, leaving room.
///
/// A machine that cannot spare the video memory turns it back on in the
/// settings, which now works. Read from the process arguments rather than
/// any shared state because CEF asks this before anything else is built --
/// `window::SHARED` is not populated until after `execute_process()` (which
/// is what fires this) returns in `main.rs` -- and defaults to *on* if the
/// X-Plane root is not known, since that is the safe direction when nothing
/// can be read.
///
/// The root has to be `--xp-root=`, not `--aircraft=`: the settings page
/// (`web.rs`) and the crash reporter (`crash.rs`) both read/write this same
/// setting via `shared.xplane_root`, and `settings_files::app_settings_path`
/// resolves `xphfbw.json` under *that* root's `Output/preferences`, not the
/// converted aircraft's own folder.
fn force_cpu_rendering() -> bool {
    let xp_root = xp_root_arg(std::env::args());
    let Some(xp_root) = xp_root else { return true };
    let settings = crate::settings::load(&xp_root);
    match settings.get("xphfbw.forceCpuRendering") {
        Some(v) => {
            let text = v.as_str().map(str::trim).unwrap_or("");
            text.eq_ignore_ascii_case("true") || text == "1" || v.as_bool() == Some(true)
        }
        None => false,
    }
}

/// The `--xp-root=` value from a process's own arguments, exactly how
/// `main.rs` parses it for `Shared::xplane_root` -- split out so
/// `force_cpu_rendering` can be tested without a real process command line.
fn xp_root_arg(mut args: impl Iterator<Item = String>) -> Option<std::path::PathBuf> {
    args.find_map(|a| a.strip_prefix("--xp-root=").map(std::path::PathBuf::from))
}

/// The value-less Chromium switches for the chosen rendering mode.
///
/// On the GPU, DirectComposition is switched off. Every instrument is an
/// offscreen (windowless) view that CEF hands back as a pixel buffer, so
/// nothing here ever presents through Windows' compositor -- but Chromium
/// still sets DirectComposition up for its output surfaces, and on this
/// machine that killed the GPU process the moment the first view asked it
/// for a context (exit 0x80000003, three times in a row, every launch;
/// cef-debug.log 2026-09-26). After the third crash Chromium gives up on the
/// GPU for the rest of the session (`--use-gl=disabled`,
/// `--disable-gpu-compositing` on every renderer), which is the same
/// software rasterising that holds every screen to a few frames a second.
/// A/B on the same machine, same profile: with this switch the GPU process
/// stays up on the hardware; without it, it crashes three times and falls
/// back to software.
fn gpu_mode_switches(force_cpu: bool) -> &'static [&'static str] {
    if force_cpu {
        &["disable-gpu", "disable-gpu-compositing"]
    } else {
        &["disable-direct-composition"]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_gpu_path_turns_direct_composition_off_and_never_disables_the_gpu() {
        let gpu = gpu_mode_switches(false);
        assert!(gpu.contains(&"disable-direct-composition"), "offscreen views never present through DirectComposition, and setting it up crashed the GPU process");
        assert!(!gpu.iter().any(|s| s.starts_with("disable-gpu")), "the GPU path must not switch the GPU off");
        assert_eq!(gpu_mode_switches(true), &["disable-gpu", "disable-gpu-compositing"], "the safe-mode setting still forces software rendering");
    }

    #[test]
    fn force_cpu_rendering_keys_off_the_xp_root_flag_not_the_aircraft_flag() {
        let argv = [
            "XPHFBW.exe".to_string(),
            "sometag".to_string(),
            "--aircraft=D:/Steam Games/steamapps/common/X-Plane 12/Aircraft/FlyByWire A380X".to_string(),
            "--xp-root=D:/Steam Games/steamapps/common/X-Plane 12".to_string(),
        ];
        let xp_root = xp_root_arg(argv.into_iter()).expect("--xp-root= is parsed");
        assert_eq!(xp_root, std::path::PathBuf::from("D:/Steam Games/steamapps/common/X-Plane 12"));

        // The whole point of the fix: this is the exact file the settings
        // page (web.rs) and the crash reporter (crash.rs) read and write,
        // because both key off `shared.xplane_root`, which is this same
        // `--xp-root=` value -- not `--aircraft=`.
        assert_eq!(
            fbw_a380_systems::settings_files::app_settings_path(&xp_root),
            std::path::PathBuf::from("D:/Steam Games/steamapps/common/X-Plane 12")
                .join("Output")
                .join("preferences")
                .join("xphfbw.json"),
        );
    }
}

wrap_app! {
    pub struct XphfbwApp;

    impl App {
        fn on_before_command_line_processing(&self, process_type: Option<&CefString>, command_line: Option<&mut CommandLine>) {
            // The browser process decides for all.
            let is_browser = process_type.map_or(true, |t| t.to_string().is_empty());
            if let (true, Some(cl)) = (is_browser, command_line) {
                for switch in gpu_mode_switches(force_cpu_rendering()) {
                    cl.append_switch(Some(&CefString::from(*switch)));
                }
                if !force_cpu_rendering() {
                    // cef-debug.log (2026-09-25 12:40 run) caught the GPU
                    // process crashing three times at startup, exit
                    // 0x80000003 (a Chromium CHECK failure), the third one
                    // logging "Failed to create shared context for
                    // virtualization" (gpu_channel_manager.cc) -- a failure
                    // inside ANGLE's own backend selection / context-sharing
                    // setup, not a sandbox violation: `--disable-gpu-sandbox`
                    // would be a no-op here, since `main.rs`'s CEF `Settings`
                    // already sets `no_sandbox: 1` for every subprocess,
                    // GPU included. The same exe worked eighty minutes
                    // earlier with nothing else different, which is what a
                    // flaky backend probe at startup looks like. Pin the
                    // ANGLE backend so Chromium does not re-probe/re-select
                    // its GL/D3D implementation on every launch; d3d11 is
                    // already what it settles on on this hardware (it has
                    // been Chromium's preferred ANGLE backend on Windows for
                    // years), so this removes a source of run-to-run
                    // variance in startup without changing what actually
                    // renders once it is up. Only meaningful when the GPU is
                    // drawing at all, hence the `else`.
                    cl.append_switch_with_value(Some(&CefString::from("use-angle")), Some(&CefString::from("d3d11")));
                }
                // Diagnostics only, never set by the plugin: Chromium's
                // DevTools protocol on this local port, to inspect a view.
                if let Ok(port) = std::env::var("XPHFBW_DEVTOOLS_PORT") {
                    cl.append_switch_with_value(Some(&CefString::from("remote-debugging-port")), Some(&CefString::from(port.as_str())));
                }
            }
        }

        fn browser_process_handler(&self) -> Option<BrowserProcessHandler> {
            Some(XphfbwBrowserProcessHandler::new())
        }

        fn render_process_handler(&self) -> Option<RenderProcessHandler> {
            Some(crate::renderer::new_render_process_handler())
        }

        /// Runs in every process (browser and every renderer/subprocess):
        /// required for `coui://`/`xphfbw://` URLs to parse as standard
        /// URLs (`location.host`/`location.pathname`) everywhere, not just
        /// where `scheme::install` registers their handlers.
        fn on_register_custom_schemes(&self, registrar: Option<&mut SchemeRegistrar>) {
            if let Some(registrar) = registrar {
                crate::scheme::register_custom_schemes(registrar);
            }
        }
    }
}

wrap_browser_process_handler! {
    struct XphfbwBrowserProcessHandler;

    impl BrowserProcessHandler {
        fn on_context_initialized(&self) {
            poll_show_requests();
            if let Some(shared) = SHARED.get() {
                crate::scheme::install(shared.aircraft_dir.clone(), shared.runtime_dir.clone());
                if let Some(tag) = shared.tag.clone() {
                    crate::views::start(tag, shared.aircraft_dir.clone());
                }
            }
        }

        /// Every CEF child process (renderers included) needs to know which
        /// bridge session, X-Plane install and aircraft it belongs to.
        fn on_before_child_process_launch(&self, command_line: Option<&mut CommandLine>) {
            crate::renderer::append_child_switches(command_line);
        }
    }
}

/// Every quarter second on the UI thread: open (or raise) the window when
/// something asked for it.
fn poll_show_requests() {
    if let Some(shared) = SHARED.get() {
        if shared.show_requested.swap(false, Ordering::Relaxed) {
            show_window(shared);
        }
    }
    let mut task = PollTask::new();
    post_delayed_task(ThreadId::UI, Some(&mut task), 250);
}

wrap_task! {
    struct PollTask;

    impl Task {
        fn execute(&self) {
            poll_show_requests();
        }
    }
}

wrap_task! {
    struct QuitTask;

    impl Task {
        fn execute(&self) {
            quit_message_loop();
        }
    }
}

/// End the app from any thread.
pub fn quit_from_any_thread() {
    let mut task = QuitTask::new();
    post_task(ThreadId::UI, Some(&mut task));
}

fn show_window(shared: &Shared) {
    let existing = WINDOW.with(|w| w.borrow().clone());
    if let Some(window) = existing {
        window.show();
        window.activate();
        return;
    }
    let port = shared.port.load(Ordering::Relaxed);
    if port == 0 {
        // The page's server is not up yet; ask again next poll.
        shared.show_requested.store(true, Ordering::Relaxed);
        return;
    }
    let url = CefString::from(format!("http://127.0.0.1:{port}/").as_str());
    let browser_settings = BrowserSettings::default();
    let browser_view = browser_view_create(None, Some(&url), Some(&browser_settings), None, None, None);
    let mut delegate = XphfbwWindowDelegate::new(RefCell::new(browser_view));
    window_create_top_level(Some(&mut delegate));
}

wrap_window_delegate! {
    struct XphfbwWindowDelegate {
        browser_view: RefCell<Option<BrowserView>>,
    }

    impl ViewDelegate {
        fn preferred_size(&self, _view: Option<&mut View>) -> Size {
            Size { width: 1280, height: 860 }
        }
    }

    impl PanelDelegate {}

    impl WindowDelegate {
        fn on_window_created(&self, window: Option<&mut Window>) {
            let browser_view = self.browser_view.borrow();
            let (Some(window), Some(browser_view)) = (window, browser_view.as_ref()) else { return };
            let mut view = View::from(browser_view);
            window.add_child_view(Some(&mut view));
            window.set_title(Some(&CefString::from("XPHFBW")));
            if let Some(mut icon) = crate::tray::app_icon_image() {
                window.set_window_icon(Some(&mut icon));
                window.set_window_app_icon(Some(&mut icon));
            }
            window.show();
            WINDOW.with(|w| *w.borrow_mut() = Some(window.clone()));
        }

        fn on_window_destroyed(&self, _window: Option<&mut Window>) {
            *self.browser_view.borrow_mut() = None;
            WINDOW.with(|w| *w.borrow_mut() = None);
        }

        fn can_close(&self, _window: Option<&mut Window>) -> i32 {
            let browser_view = self.browser_view.borrow();
            match browser_view.as_ref().and_then(|v| v.browser()) {
                Some(browser) => browser.host().map_or(1, |h| h.try_close_browser()),
                None => 1,
            }
        }
    }
}

// ---------------------------------------------------------------------------
// "Show the window" between launches: a named event.
// ---------------------------------------------------------------------------

#[link(name = "kernel32")]
extern "system" {
    fn CreateEventW(attributes: *mut std::ffi::c_void, manual_reset: i32, initial: i32, name: *const u16) -> *mut std::ffi::c_void;
    fn OpenEventW(access: u32, inherit: i32, name: *const u16) -> *mut std::ffi::c_void;
    fn SetEvent(event: *mut std::ffi::c_void) -> i32;
    fn WaitForSingleObject(object: *mut std::ffi::c_void, milliseconds: u32) -> u32;
    fn CloseHandle(object: *mut std::ffi::c_void) -> i32;
}

fn wide(s: &str) -> Vec<u16> {
    s.encode_utf16().chain(std::iter::once(0)).collect()
}

/// Ask an app already running to show its window. False when none runs.
pub fn signal_existing(name: &str) -> bool {
    const EVENT_MODIFY_STATE: u32 = 0x0002;
    let n = wide(name);
    let h = unsafe { OpenEventW(EVENT_MODIFY_STATE, 0, n.as_ptr()) };
    if h.is_null() {
        return false;
    }
    unsafe {
        SetEvent(h);
        CloseHandle(h);
    }
    true
}

/// Turn the event into a show request for the UI thread's poll.
pub fn watch_show_requests(name: &str, shared: Arc<Shared>) {
    let n = wide(name);
    let h = unsafe { CreateEventW(std::ptr::null_mut(), 0, 0, n.as_ptr()) } as usize;
    if h == 0 {
        return;
    }
    let _ = std::thread::Builder::new().name("show requests".into()).spawn(move || loop {
        if unsafe { WaitForSingleObject(h as *mut _, 0xFFFF_FFFF) } == 0 {
            shared.show_requested.store(true, Ordering::Relaxed);
        }
    });
}

/// The plugin's `xphfbw/restart_displays` command (src/xphfbw_datarefs.rs
/// signals this event): reload every instrument view.
pub fn watch_restart_displays() {
    let n = wide(r"Local\XPHFBW_restart_displays");
    let h = unsafe { CreateEventW(std::ptr::null_mut(), 0, 0, n.as_ptr()) } as usize;
    if h == 0 {
        return;
    }
    let _ = std::thread::Builder::new().name("restart displays requests".into()).spawn(move || loop {
        if unsafe { WaitForSingleObject(h as *mut _, 0xFFFF_FFFF) } == 0 {
            crate::views::restart_displays();
        }
    });
}
