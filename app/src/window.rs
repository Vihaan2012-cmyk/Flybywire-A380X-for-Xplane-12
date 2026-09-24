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
/// any shared state because CEF asks this before anything else is built,
/// and defaults to *on* if the aircraft path is not known, since that is
/// the safe direction when nothing can be read.
fn force_cpu_rendering() -> bool {
    let aircraft = std::env::args().find_map(|a| a.strip_prefix("--aircraft=").map(std::path::PathBuf::from));
    let Some(aircraft) = aircraft else { return true };
    let settings = crate::settings::load(&aircraft);
    match settings.get("xphfbw.forceCpuRendering") {
        Some(v) => {
            let text = v.as_str().map(str::trim).unwrap_or("");
            text.eq_ignore_ascii_case("true") || text == "1" || v.as_bool() == Some(true)
        }
        None => false,
    }
}

wrap_app! {
    pub struct XphfbwApp;

    impl App {
        fn on_before_command_line_processing(&self, process_type: Option<&CefString>, command_line: Option<&mut CommandLine>) {
            // The browser process decides for all.
            let is_browser = process_type.map_or(true, |t| t.to_string().is_empty());
            if let (true, Some(cl)) = (is_browser, command_line) {
                if force_cpu_rendering() {
                    cl.append_switch(Some(&CefString::from("disable-gpu")));
                    cl.append_switch(Some(&CefString::from("disable-gpu-compositing")));
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
