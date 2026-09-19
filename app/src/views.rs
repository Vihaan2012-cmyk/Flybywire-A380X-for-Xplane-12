//! One windowless CEF browser per FlyByWire instrument view
//! (docs/briefs/xphfbw-js-bridge.md, agent G): `xphfbw_bridge_views::view_list`
//! (agent C, shared with the plugin so both sides number views the same way)
//! finds the views in the aircraft's panel.cfg; each gets its own off-screen
//! browser navigated to its gauge's `coui://` URL, sized to the gauge (a
//! screenless host — SystemsHost, ExtrasHost — gets 1x1); `on_paint` copies
//! straight into that screen's `ScreenBlock` (the protocol's rule 6); a
//! UI-thread timer drains the `input` ring and sends mouse events to the
//! right browser.
//!
//! Runs entirely on CEF's UI thread (this app does not set
//! `multi_threaded_message_loop`, so that is the same OS thread as
//! `run_message_loop()` in main.rs): the view state below is a
//! `thread_local`, not behind a lock.

use std::cell::RefCell;
use std::path::{Path, PathBuf};
use std::rc::Rc;
use std::sync::atomic::Ordering;

use cef::*;
use fbw_a380_systems::xphfbw_bridge::{Input, InputKind, ScreenBlock, Session, MAX_DIRTY_RECTS};
use fbw_a380_systems::xphfbw_bridge_views::{view_list, ViewDef};

/// Shared with the plugin (xphfbw_bridge_views), so both number screens alike.
use fbw_a380_systems::xphfbw_bridge_views::{EFB_HEIGHT, EFB_SCREEN, EFB_WIDTH, SCREEN_ORDER};

const OPEN_RETRY_MS: i64 = 250;
/// ~75s of retries: the plugin creates the session only once the systems
/// this app builds are ready (remote::client's START_TIMEOUT is 60s), then
/// its screens.
const OPEN_MAX_TRIES: u32 = 300;
const INPUT_POLL_MS: i64 = 8;

thread_local! {
    static STATE: RefCell<Option<State>> = const { RefCell::new(None) };
}

struct State {
    aircraft_root: PathBuf,
    session: Session,
    /// Every view's browser (restart_displays, shutdown).
    browsers: Vec<Browser>,
    /// Indexed like `SCREEN_ORDER`: the browser that screen's view owns, for
    /// routing `Input` records (rule 6's "`Input.screen`" is that same
    /// index, `display/mod.rs`'s `Bridge::screens` doc comment).
    screen_browsers: Vec<Option<Browser>>,
}

/// Starts opening the plugin's session for `tag` (with retries) and, once it
/// opens, creates every view's browser. Call once, from CEF's
/// `on_context_initialized` (window.rs), after `scheme::install`.
pub fn start(tag: String, aircraft_root: PathBuf) {
    try_open(tag, aircraft_root, OPEN_MAX_TRIES);
}

fn try_open(tag: String, aircraft_root: PathBuf, tries_left: u32) {
    // The plugin makes the session and then every screen (xphfbw_host.rs,
    // display::set_bridge): wait for both, so no view starts without the
    // screen it paints into.
    if let Some(session) = Session::open(&tag) {
        if screens_ready(&tag, &aircraft_root) {
            create_views(&tag, session, aircraft_root);
            return;
        }
    }
    if tries_left == 0 {
        crate::logging::log(&format!("views: gave up opening the session {tag} (the plugin never created it?)"));
        return;
    }
    let mut task = OpenSessionTask::new(tag, aircraft_root, tries_left - 1);
    post_delayed_task(ThreadId::UI, Some(&mut task), OPEN_RETRY_MS);
}

wrap_task! {
    struct OpenSessionTask {
        tag: String,
        aircraft_root: PathBuf,
        tries_left: u32,
    }

    impl Task {
        fn execute(&self) {
            try_open(self.tag.clone(), self.aircraft_root.clone(), self.tries_left);
        }
    }
}

/// Whether every screen panel.cfg's views paint into already exists.
fn screens_ready(tag: &str, aircraft_root: &Path) -> bool {
    let Ok(panel_cfg) = std::fs::read_to_string(aircraft_root.join("panel").join("panel.cfg")) else {
        // create_views reports the missing panel.cfg itself.
        return true;
    };
    view_list(&panel_cfg).iter().filter(|d| !d.screen.is_empty()).all(|d| ScreenBlock::open(tag, &d.screen, d.width, d.height).is_some())
}

fn create_views(tag: &str, session: Session, aircraft_root: PathBuf) {
    let panel_cfg = match std::fs::read_to_string(aircraft_root.join("panel").join("panel.cfg")) {
        Ok(text) => text,
        Err(e) => {
            crate::logging::log(&format!("views: could not read {}: {e}", aircraft_root.join("panel").join("panel.cfg").display()));
            return;
        }
    };
    let fps = display_fps(&aircraft_root);
    let defs = view_list(&panel_cfg);
    crate::logging::log(&format!("views: {} views from panel.cfg, {fps} fps", defs.len()));

    let mut browsers = Vec::with_capacity(defs.len());
    let mut screen_browsers: Vec<Option<Browser>> = SCREEN_ORDER.iter().map(|_| None).collect();
    for def in &defs {
        let Some(browser) = spawn_view(tag, def, fps) else {
            crate::logging::log(&format!("views: could not create the browser for {} ({})", def.section, def.gauge_url));
            continue;
        };
        if !def.screen.is_empty() {
            match screen_slot(&def.screen) {
                Some(i) => screen_browsers[i] = Some(browser.clone()),
                None => crate::logging::log(&format!("views: {} names an unknown screen {}", def.section, def.screen)),
            }
        }
        browsers.push(browser);
    }
    if let Some(browser) = spawn_efb_view(tag, fps) {
        if let Some(i) = screen_slot(EFB_SCREEN) {
            screen_browsers[i] = Some(browser.clone());
        }
        browsers.push(browser);
    } else {
        crate::logging::log("views: could not create the EFB browser (the app's own web server has no port yet?)");
    }

    let count = browsers.len();
    STATE.with(|s| *s.borrow_mut() = Some(State { aircraft_root, session, browsers, screen_browsers }));
    crate::logging::log(&format!("views: {count} browsers created"));
    start_input_timer();
}

/// The EFB screen's browser: not one of panel.cfg's views (it is filtered
/// out by `EXCLUDED_GAUGES`, since this port does not render FlyByWire's own
/// EFB gauge at all). Instead it loads the app's own settings/study page
/// (`app/ui/index.html`, served by `web.rs` on `shared.port`) at the EFB
/// mesh's own size (`EFB_WIDTH`/`EFB_HEIGHT`, matching panel.cfg's
/// `$SCREEN_EFB` gauge so the converted mesh's UVs still line up), so the
/// same app the user's desktop window shows becomes the in-cockpit EFB too
/// (the task's "the app also becomes the EFB").
fn spawn_efb_view(tag: &str, fps: i32) -> Option<Browser> {
    // `web::start` (main.rs) binds its listener and stores `shared.port`
    // before `initialize()`/`run_message_loop()` run, and this only runs
    // from `on_context_initialized` (window.rs), well after that: the port
    // is already known, no retry loop needed here (unlike `try_open`'s wait
    // for the plugin's session).
    let port = crate::window::SHARED.get()?.port.load(Ordering::Relaxed);
    if port == 0 {
        crate::logging::log("views: the EFB's web server has no port yet");
        return None;
    }
    let block = ScreenBlock::open(tag, EFB_SCREEN, EFB_WIDTH, EFB_HEIGHT)?;
    let screen = Some(Rc::new(block));
    let render_handler = ViewPaint::new(EFB_WIDTH as i32, EFB_HEIGHT as i32, screen);
    let mut client = ViewClient::new(render_handler);
    let window_info = WindowInfo::default().set_as_windowless(unsafe { std::mem::zeroed() });
    let browser_settings = BrowserSettings { windowless_frame_rate: fps.max(1), background_color: 0, ..Default::default() };
    let url = format!("http://127.0.0.1:{port}/");
    let browser = browser_host_create_browser_sync(Some(&window_info), Some(&mut client), Some(&CefString::from(url.as_str())), Some(&browser_settings), None, None)?;
    focus(&browser);
    Some(browser)
}

fn spawn_view(tag: &str, def: &ViewDef, fps: i32) -> Option<Browser> {
    let (width, height) = view_size(def);
    let screen = if def.screen.is_empty() {
        None
    } else {
        match ScreenBlock::open(tag, &def.screen, def.width, def.height) {
            Some(b) => Some(Rc::new(b)),
            None => {
                crate::logging::log(&format!("views: no ScreenBlock {} for {} (the plugin has not made it yet?)", def.screen, def.section));
                None
            }
        }
    };
    let render_handler = ViewPaint::new(width, height, screen);
    let mut client = ViewClient::new(render_handler);
    let window_info = WindowInfo::default().set_as_windowless(unsafe { std::mem::zeroed() });
    // Transparent: an ND page's empty areas must show the terrain or weather
    // radar layer the plugin draws under it (MSFS stacks the terrain gauge
    // below nd.html); the plugin draws opaque black under every screen.
    let browser_settings = BrowserSettings { windowless_frame_rate: fps.max(1), background_color: 0, ..Default::default() };
    let mut extra = dictionary_value_create()?;
    extra.set_int(Some(&CefString::from("view")), def.index as i32);
    extra.set_string(Some(&CefString::from("screen")), Some(&CefString::from(def.screen.as_str())));
    let url = gauge_url(def);
    let browser = browser_host_create_browser_sync(
        Some(&window_info),
        Some(&mut client),
        Some(&CefString::from(url.as_str())),
        Some(&browser_settings),
        Some(&mut extra),
        None,
    )?;
    focus(&browser);
    Some(browser)
}

/// An off-screen browser has no OS window to carry focus, so CEF never
/// infers it: without this, `document.hasFocus()` stays false and focus-
/// dependent controls (the MFD's text fields) ignore clicks
/// (docs/deep/debug_screen_clicks.md). Each view is its own browser, so this
/// takes focus from no other screen.
fn focus(browser: &Browser) {
    if let Some(host) = browser.host() {
        host.set_focus(1);
    }
}

fn display_fps(aircraft_root: &Path) -> i32 {
    parse_fps(crate::settings::load(aircraft_root).get("xphfbw.displayFps"))
}

fn parse_fps(value: Option<&serde_json::Value>) -> i32 {
    let n = value.and_then(|v| v.as_i64().or_else(|| v.as_str().and_then(|s| s.trim().parse::<i64>().ok())));
    match n {
        Some(v) if v > 0 => v as i32,
        _ => 60,
    }
}

fn gauge_url(def: &ViewDef) -> String {
    format!("coui://html_ui/Pages/VCockpit/Instruments/{}", def.gauge_url)
}

/// The page's pixel size; a screenless view (SystemsHost, ExtrasHost) is
/// headless at 1x1 regardless of what panel.cfg's `pixel_size` says.
fn view_size(def: &ViewDef) -> (i32, i32) {
    if def.screen.is_empty() {
        (1, 1)
    } else {
        (def.width.max(1) as i32, def.height.max(1) as i32)
    }
}

fn screen_slot(id: &str) -> Option<usize> {
    SCREEN_ORDER.iter().position(|s| s.eq_ignore_ascii_case(id))
}

// ---------------------------------------------------------------------------
// Painting: on_paint -> ScreenBlock (rule 6).
// ---------------------------------------------------------------------------

wrap_render_handler! {
    struct ViewPaint {
        width: i32,
        height: i32,
        screen: Option<Rc<ScreenBlock>>,
    }

    impl RenderHandler {
        fn view_rect(&self, _browser: Option<&mut Browser>, rect: Option<&mut Rect>) {
            if let Some(rect) = rect {
                rect.x = 0;
                rect.y = 0;
                rect.width = self.width;
                rect.height = self.height;
            }
        }

        fn screen_info(&self, _browser: Option<&mut Browser>, screen_info: Option<&mut ScreenInfo>) -> ::std::os::raw::c_int {
            let Some(info) = screen_info else { return 0 };
            info.device_scale_factor = 1.0;
            1
        }

        fn on_paint(
            &self,
            _browser: Option<&mut Browser>,
            type_: PaintElementType,
            dirty_rects: Option<&[Rect]>,
            buffer: *const u8,
            width: ::std::os::raw::c_int,
            height: ::std::os::raw::c_int,
        ) {
            if type_ != PaintElementType::VIEW || buffer.is_null() || width <= 0 || height <= 0 {
                return;
            }
            let Some(block) = self.screen.as_deref() else { return };
            paint(block, dirty_rects, buffer, width, height);
        }
    }
}

fn paint(block: &ScreenBlock, dirty_rects: Option<&[Rect]>, buffer: *const u8, width: i32, height: i32) {
    let h = block.header();
    if h.width.load(Ordering::Relaxed) as i32 != width || h.height.load(Ordering::Relaxed) as i32 != height {
        // A resize XPHFBW does not support (panel.cfg's gauge size should
        // always match): drop the frame rather than write out of bounds.
        return;
    }
    let src = unsafe { std::slice::from_raw_parts(buffer, width as usize * height as usize * 4) };
    h.writing.store(1, Ordering::Relaxed);
    let dst = block.pixels();
    let whole = [Rect { x: 0, y: 0, width, height }];
    // More changed areas than the header holds: the whole frame, so none is lost.
    let rects: &[Rect] = dirty_rects.filter(|r| !r.is_empty() && r.len() <= MAX_DIRTY_RECTS).unwrap_or(&whole[..]);
    let n = rects.len().min(MAX_DIRTY_RECTS);
    for (i, r) in rects.iter().take(n).enumerate() {
        copy_rect(src, dst, width as usize, r);
        let base = i * 4;
        h.dirty[base].store(r.x.max(0) as u32, Ordering::Relaxed);
        h.dirty[base + 1].store(r.y.max(0) as u32, Ordering::Relaxed);
        h.dirty[base + 2].store(r.width.max(0) as u32, Ordering::Relaxed);
        h.dirty[base + 3].store(r.height.max(0) as u32, Ordering::Relaxed);
    }
    h.dirty_count.store(n as u32, Ordering::Relaxed);
    h.frame.fetch_add(1, Ordering::Relaxed);
    h.writing.store(0, Ordering::Relaxed);
}

/// A straight BGRA copy of one dirty rect, row by row (rule 6): `src` and
/// `dst` are both `stride_px * height` pixels wide (CEF's paint buffer and
/// the `ScreenBlock`'s pixels are checked to be the same size in `paint`),
/// so a rect's rows sit at the same byte offsets in both.
fn copy_rect(src: &[u8], dst: &mut [u8], stride_px: usize, r: &Rect) {
    let (x, y, w, h) = (r.x.max(0) as usize, r.y.max(0) as usize, r.width.max(0) as usize, r.height.max(0) as usize);
    let row_bytes = w * 4;
    for row in 0..h {
        let start = ((y + row) * stride_px + x) * 4;
        let end = start + row_bytes;
        if end > dst.len() || end > src.len() {
            break;
        }
        dst[start..end].copy_from_slice(&src[start..end]);
    }
}

wrap_client! {
    struct ViewClient {
        render_handler: RenderHandler,
    }

    impl Client {
        fn render_handler(&self) -> Option<RenderHandler> {
            Some(self.render_handler.clone())
        }
    }
}

// ---------------------------------------------------------------------------
// Input: the `input` ring (plugin -> app) -> mouse events on the right
// browser, drained on a UI-thread timer.
// ---------------------------------------------------------------------------

fn start_input_timer() {
    let mut task = InputTimerTask::new();
    post_delayed_task(ThreadId::UI, Some(&mut task), INPUT_POLL_MS);
}

wrap_task! {
    struct InputTimerTask;

    impl Task {
        fn execute(&self) {
            drain_input();
            start_input_timer();
        }
    }
}

fn drain_input() {
    STATE.with(|s| {
        let borrow = s.borrow();
        let Some(state) = borrow.as_ref() else { return };
        for record in state.session.input.drain() {
            let Some(input) = Input::decode(&record) else { continue };
            if input.kind != InputKind::Move {
                static LOGGED: std::sync::atomic::AtomicU32 = std::sync::atomic::AtomicU32::new(0);
                if LOGGED.fetch_add(1, std::sync::atomic::Ordering::Relaxed) < 20 {
                    let has_browser = matches!(state.screen_browsers.get(input.screen as usize), Some(Some(_)));
                    crate::logging::log(&format!("input: screen {} {:?} at ({:.0},{:.0}), browser {}", input.screen, input.kind, input.x, input.y, has_browser));
                }
            }
            let Some(Some(browser)) = state.screen_browsers.get(input.screen as usize) else { continue };
            let Some(host) = browser.host() else { continue };
            send_input_event(&host, input);
        }
    });
}

fn send_input_event(host: &BrowserHost, input: Input) {
    if input.kind == InputKind::Key {
        send_key_event(host, input);
        return;
    }
    let event = MouseEvent { x: input.x.round() as i32, y: input.y.round() as i32, modifiers: 0 };
    match input.kind {
        InputKind::Down => host.send_mouse_click_event(Some(&event), MouseButtonType::LEFT, 0, 1),
        InputKind::Up => host.send_mouse_click_event(Some(&event), MouseButtonType::LEFT, 1, 1),
        InputKind::Move => host.send_mouse_move_event(Some(&event), 0),
        // CEF's wheel delta is in the same "notch" units browsers expect
        // from a real wheel (~120 per click); the bridge's `delta` is
        // "wheel clicks, up positive" (xphfbw_bridge.rs), so scale by 120.
        InputKind::Wheel => host.send_mouse_wheel_event(Some(&event), 0, (input.delta * 120.) as i32),
        InputKind::Key => unreachable!("handled above"),
    }
}

/// The KCCU typing into the MFD (`InputKind::Key`'s doc comment): one
/// `KEYEVENT_CHAR` carries the typed character straight to whatever text
/// field has focus in the page, the same way a real keystroke would; no
/// separate down/up pair, since the SDK callback this comes from fires once
/// per keystroke, not a press-and-release pair.
fn send_key_event(host: &BrowserHost, input: Input) {
    let ch = char::from_u32(input.x as u32).unwrap_or('\0');
    if ch == '\0' {
        return;
    }
    let event = KeyEvent {
        type_: KeyEventType::CHAR,
        modifiers: input.delta as u32,
        windows_key_code: input.y as i32,
        character: ch as u16,
        unmodified_character: ch as u16,
        ..Default::default()
    };
    host.send_key_event(Some(&event));
}

// ---------------------------------------------------------------------------
// Agent A's "restart displays" web action (web.rs owns the wiring; this is
// the `pub fn` it calls).
// ---------------------------------------------------------------------------

/// Reloads every view's browser and refreshes its windowless frame rate
/// from `xphfbw.json` (agent B's `displayFps`). Safe to call from any
/// thread (it posts to CEF's UI thread) and safe to call before views have
/// started (it just logs and does nothing).
pub fn restart_displays() {
    let mut task = RestartTask::new();
    post_task(ThreadId::UI, Some(&mut task));
}

wrap_task! {
    struct RestartTask;

    impl Task {
        fn execute(&self) {
            STATE.with(|s| {
                let borrow = s.borrow();
                let Some(state) = borrow.as_ref() else {
                    crate::logging::log("views: restart-displays requested before views started");
                    return;
                };
                let fps = display_fps(&state.aircraft_root);
                for browser in &state.browsers {
                    if let Some(host) = browser.host() {
                        host.set_windowless_frame_rate(fps.max(1));
                    }
                    browser.reload();
                }
                crate::logging::log(&format!("views: restarted {} browsers at {fps} fps", state.browsers.len()));
            });
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;

    fn def(section: &str, gauge_url: &str, width: u32, height: u32, screen: &str) -> ViewDef {
        ViewDef { index: 0, section: section.into(), gauge_url: gauge_url.into(), width, height, screen: screen.into() }
    }

    #[test]
    fn screen_order_has_sixteen_distinct_non_empty_ids() {
        assert_eq!(SCREEN_ORDER.len(), 16);
        let mut sorted = SCREEN_ORDER.to_vec();
        sorted.sort();
        sorted.dedup();
        assert_eq!(sorted.len(), SCREEN_ORDER.len(), "SCREEN_ORDER has a duplicate");
        assert!(SCREEN_ORDER.iter().all(|s| !s.is_empty()));
    }

    #[test]
    fn screen_slot_matches_case_insensitively_and_rejects_unknown_ids() {
        assert_eq!(screen_slot("SCREEN_DU_PFDL"), Some(0));
        assert_eq!(screen_slot("screen_du_pfdl"), Some(0));
        assert_eq!(screen_slot("SCREEN_DU_RMP_3"), Some(14));
        assert_eq!(screen_slot("SCREEN_EFB"), Some(15), "the EFB gets its own hand-made view, not one from panel.cfg's view_list");
    }

    #[test]
    fn gauge_url_is_rooted_under_the_instruments_folder() {
        let v = def("VCockpit01", "A380X/PFD/pfd.html", 768, 1024, "SCREEN_DU_PFDL");
        assert_eq!(gauge_url(&v), "coui://html_ui/Pages/VCockpit/Instruments/A380X/PFD/pfd.html");
    }

    #[test]
    fn view_size_is_the_panel_cfg_size_except_headless_hosts_are_1x1() {
        let screened = def("VCockpit01", "A380X/PFD/pfd.html", 768, 1024, "SCREEN_DU_PFDL");
        assert_eq!(view_size(&screened), (768, 1024));
        let headless = def("VCockpit02", "A380X/SystemsHost/index.html", 768, 1024, "");
        assert_eq!(view_size(&headless), (1, 1));
        let zero_sized = def("VCockpit03", "A380X/X/x.html", 0, 0, "SCREEN_ISIS_1");
        assert_eq!(view_size(&zero_sized), (1, 1), "never a 0x0 browser even if panel.cfg somehow said so");
    }

    #[test]
    fn parse_fps_falls_back_to_60_for_anything_not_a_positive_number() {
        assert_eq!(parse_fps(None), 60);
        assert_eq!(parse_fps(Some(&json!(30))), 30);
        assert_eq!(parse_fps(Some(&json!("20"))), 20);
        assert_eq!(parse_fps(Some(&json!(-5))), 60);
        assert_eq!(parse_fps(Some(&json!(0))), 60);
        assert_eq!(parse_fps(Some(&json!("not a number"))), 60);
    }
}
