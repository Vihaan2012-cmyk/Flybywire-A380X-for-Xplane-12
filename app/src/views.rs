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
use fbw_a380_systems::xphfbw_bridge_views::{device_size, EFB_HEIGHT, EFB_SCREEN, EFB_SUPERSAMPLE, EFB_WIDTH, SCREEN_ORDER};

const OPEN_RETRY_MS: i64 = 250;
/// ~75s of retries: the plugin creates the session only once the systems
/// this app builds are ready (remote::client's START_TIMEOUT is 60s), then
/// its screens.
const OPEN_MAX_TRIES: u32 = 300;
const INPUT_POLL_MS: i64 = 8;
/// How long a screen's browser is given to paint its first frame before it
/// is taken as stillborn and made again, and how many times that is tried.
const FIRST_PAINT_MS: i64 = 4000;
const FIRST_PAINT_TRIES: u32 = 3;
/// How long a screen's browser is given to report `Loaded` (`window
/// .__xphfbw.loaded()`, renderer.rs's `dispatch`) once it HAS painted before
/// it is taken as stuck and made again, and how many times that is tried.
/// A page can paint blank or loading-spinner frames without FlyByWire's own
/// bootstrap ever finishing -- the symptom this watches for is distinct from
/// `FIRST_PAINT_MS`'s "never painted at all" (HEADER: the broken sessions'
/// gauges painted a handful of frames while zero `js: xphfbw ... loaded`
/// lines ever showed up) -- so this runs as its own independent watchdog,
/// on its own clock, and can remake a screen `watch_first_paint` never would
/// (and vice versa).
const LOADED_MS: i64 = 6000;
const LOADED_TRIES: u32 = 3;

thread_local! {
    static STATE: RefCell<Option<State>> = const { RefCell::new(None) };
}

struct State {
    aircraft_root: PathBuf,
    /// The bridge tag, kept so a screen's `ScreenBlock` can be reopened to
    /// read its frame counter (`has_painted`) and so a browser can be made
    /// again (`remake`).
    tag: String,
    /// The frame rate and view list these browsers were made with, for the
    /// same reason.
    fps: i32,
    defs: Vec<ViewDef>,
    session: Session,
    /// Every view's browser (restart_displays, shutdown).
    browsers: Vec<Browser>,
    /// Indexed like `SCREEN_ORDER`: the browser that screen's view owns, for
    /// routing `Input` records (rule 6's "`Input.screen`" is that same
    /// index, `display/mod.rs`'s `Bridge::screens` doc comment).
    screen_browsers: Vec<Option<Browser>>,
    /// The frame rates the governor last set (flight instruments, every
    /// other view); `(0, 0)` makes its next run set them whatever they are.
    governed: (i32, i32),
    /// X-Plane's frame counter and when the governor last read it.
    governor_last: Option<(u64, std::time::Instant)>,
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
    STATE.with(|s| {
        *s.borrow_mut() = Some(State {
            aircraft_root,
            tag: tag.to_owned(),
            fps,
            defs,
            session,
            browsers,
            screen_browsers,
            governed: (0, 0),
            governor_last: None,
        })
    });
    crate::logging::log(&format!("views: {count} browsers created"));
    start_input_timer();
    start_governor_timer();
    watch_first_paint(FIRST_PAINT_TRIES);
    watch_loaded(LOADED_TRIES);
}

// ---------------------------------------------------------------------------
// Screens that never paint.
// ---------------------------------------------------------------------------

/// A screen's browser can be created and then never start. CEF's GPU process
/// crashed three times while these browsers were being made, and two frames
/// never answered the browser-info handshake ("Timeout of new browser info
/// response", cef-debug.log); the browsers waiting on them stayed blank
/// forever. Nothing reports it -- `browser_host_create_browser_sync` has
/// already returned a browser by then -- so the screen simply stays black,
/// which is what the EFB did on every run.
///
/// Every painting browser bumps its `ScreenBlock`'s frame counter once per
/// `on_paint` (see `paint`), so a screen that is still on frame 0 well after
/// its browser was made has never painted. Make it again.
fn watch_first_paint(tries_left: u32) {
    let mut task = FirstPaintTask::new(tries_left);
    post_delayed_task(ThreadId::UI, Some(&mut task), FIRST_PAINT_MS);
}

wrap_task! {
    struct FirstPaintTask {
        tries_left: u32,
    }

    impl Task {
        fn execute(&self) {
            check_first_paint(self.tries_left);
        }
    }
}

fn check_first_paint(tries_left: u32) {
    let silent: Vec<usize> = STATE.with(|s| {
        let borrow = s.borrow();
        let Some(state) = borrow.as_ref() else { return Vec::new() };
        (0..SCREEN_ORDER.len()).filter(|&i| state.screen_browsers[i].is_some() && !has_painted(state, i)).collect()
    });
    if silent.is_empty() {
        return;
    }
    let names: Vec<&str> = silent.iter().map(|&i| SCREEN_ORDER[i]).collect();
    if tries_left == 0 {
        crate::logging::log(&format!("views: {} never painted and will stay black: {}", silent.len(), names.join(", ")));
        // Losing half the screens or more for the whole retry budget matches
        // the GPU-process crash loop in cef-debug.log (three crashes fighting
        // "Failed to create shared context for virtualization" during
        // startup), not one browser that happened to be slow -- displays_active
        // is all-or-nothing (xphfbw_host.rs), so a real bridge failure takes
        // every screen with it, while an isolated bad browser (already fixed
        // above by remaking it) only ever takes one or two. There is no live
        // switch to flip this session: this process's CEF browser process was
        // already started with its GPU command-line switches by the time any
        // of this runs (window.rs's on_before_command_line_processing fires
        // once, before create_views), and relaunching the whole exe now would
        // tear down the FlyByWire systems thread serving the plugin mid-flight
        // -- worse than one flight with black screens. So this only makes the
        // *next* launch safe, by turning on "Force safe mode CPU rendering"
        // for it.
        if looks_like_a_crash_loop(silent.len(), SCREEN_ORDER.len()) {
            fall_back_to_cpu_rendering_next_time();
        }
        return;
    }
    crate::logging::log(&format!("views: {} never painted, making their browsers again: {}", silent.len(), names.join(", ")));
    for i in silent {
        remake(i);
    }
    watch_first_paint(tries_left - 1);
}

/// Whether `silent` screens out of `total` staying black for the whole
/// first-paint retry budget looks like the GPU-process crash loop (every
/// screen goes with it, `displays_active` is all-or-nothing) rather than one
/// unlucky browser. Half is the line: one or two silent screens out of
/// eighteen is well explained by an individual browser being slow or unlucky
/// (that is what the retries above already fix); losing half or more is not.
fn looks_like_a_crash_loop(silent: usize, total: usize) -> bool {
    total > 0 && silent * 2 >= total
}

/// After [`looks_like_a_crash_loop`] fires: turn on "Force safe mode CPU
/// rendering" (`window.rs`'s `force_cpu_rendering`) for the *next* launch, so
/// the next flight starts safe instead of repeating the same crash loop,
/// without the user having to find the settings checkbox themselves. Written
/// through `settings::save` under `shared.xplane_root` -- the root every
/// other reader/writer of `xphfbw.*` settings uses (`web.rs`'s
/// `/api/settings`, `crash.rs`'s `autoCrashReport`); `force_cpu_rendering`
/// itself currently reads from `--aircraft` instead, a separate bug (see
/// W03's fix) -- once that is corrected this write reaches it, and until
/// then it is at least consistent with everywhere else settings live.
fn fall_back_to_cpu_rendering_next_time() {
    let Some(shared) = crate::window::SHARED.get() else { return };
    let mut values = serde_json::Map::new();
    values.insert("xphfbw.forceCpuRendering".to_string(), serde_json::Value::Bool(true));
    match crate::settings::save(&shared.xplane_root, &values) {
        Ok(()) => crate::logging::log("views: half or more of the screens never recovered from the GPU crash loop; forcing CPU rendering for the next launch"),
        Err(e) => crate::logging::log(&format!("views: could not persist forceCpuRendering after the GPU crash loop: {e}")),
    }
}

/// Whether this screen's block has ever been painted into.
fn has_painted(state: &State, screen: usize) -> bool {
    let Some((w, h)) = screen_size(state, screen) else { return true };
    let Some(block) = ScreenBlock::open(&state.tag, SCREEN_ORDER[screen], w, h) else { return true };
    block.header().frame.load(Ordering::Relaxed) != 0
}

/// The pixel size this screen's browser paints at: panel.cfg's for a gauge
/// view, the EFB's supersampled device size for the EFB, which has no
/// panel.cfg entry of its own (`spawn_efb_view`, `device_size`).
fn screen_size(state: &State, screen: usize) -> Option<(u32, u32)> {
    let id = SCREEN_ORDER[screen];
    if id.eq_ignore_ascii_case(EFB_SCREEN) {
        return Some(device_size(EFB_SCREEN, EFB_WIDTH, EFB_HEIGHT));
    }
    state.defs.iter().find(|d| d.screen.eq_ignore_ascii_case(id)).map(|d| (d.width, d.height))
}

// ---------------------------------------------------------------------------
// Screens that paint but never report Loaded.
// ---------------------------------------------------------------------------

/// A screen's page can paint (CEF's blank first frame, a loading spinner,
/// the gauge's own chrome) without FlyByWire's runtime ever finishing its
/// bootstrap and calling `window.__xphfbw.loaded()` (`renderer.rs`'s
/// `dispatch("loaded", ...)`, which pushes `Uplink::Loaded` for the plugin
/// to mirror into `SlotHeader::view_loaded`) -- exactly what the broken
/// sessions in the HEADER showed: gauges painted a handful of frames while
/// zero `js: xphfbw ... loaded` lines ever appeared. `watch_first_paint`
/// alone would never catch that, since these screens DID paint. Make them
/// again instead.
fn watch_loaded(tries_left: u32) {
    let mut task = LoadedTask::new(tries_left);
    post_delayed_task(ThreadId::UI, Some(&mut task), LOADED_MS);
}

wrap_task! {
    struct LoadedTask {
        tries_left: u32,
    }

    impl Task {
        fn execute(&self) {
            check_loaded(self.tries_left);
        }
    }
}

fn check_loaded(tries_left: u32) {
    let stuck: Vec<usize> = STATE.with(|s| {
        let borrow = s.borrow();
        let Some(state) = borrow.as_ref() else { return Vec::new() };
        (0..SCREEN_ORDER.len())
            .filter(|&i| state.screen_browsers[i].is_some() && has_painted(state, i) && loaded_state(state, i).is_some_and(loaded_is_stuck))
            .collect()
    });
    if stuck.is_empty() {
        return;
    }
    let names: Vec<&str> = stuck.iter().map(|&i| SCREEN_ORDER[i]).collect();
    if tries_left == 0 {
        crate::logging::log(&format!("views: {} painted but never reported loaded, giving up: {}", stuck.len(), names.join(", ")));
        return;
    }
    crate::logging::log(&format!("views: {} painted but never reported loaded, making their browsers again: {}", stuck.len(), names.join(", ")));
    for i in stuck {
        remake(i);
    }
    watch_loaded(tries_left - 1);
}

/// The view index (`ViewDef::index`, the same index `Uplink::Loaded.view`
/// and `SlotHeader::view_loaded` use) this screen's `Loaded` state should be
/// read from, or `None` for a screen with no such state to watch: the EFB's
/// browser loads the app's own settings/study page, not one of FlyByWire's
/// gauges (`spawn_efb_view`'s doc comment), so it never calls
/// `window.__xphfbw.loaded()` at all and would otherwise look permanently
/// stuck.
fn view_index_for_screen(defs: &[ViewDef], screen_id: &str) -> Option<u32> {
    if screen_id.eq_ignore_ascii_case(EFB_SCREEN) {
        return None;
    }
    defs.iter().find(|d| d.screen.eq_ignore_ascii_case(screen_id)).map(|d| d.index)
}

/// This screen's last known `Loaded` report, straight out of shared memory
/// (`SlotHeader::view_loaded`, `xphfbw_host.rs`'s `apply_deferred` mirrors
/// it there): `None` when there is nothing to watch (`view_index_for_screen`)
/// or the index is somehow out of `MAX_VIEWS` range; otherwise 0 not yet
/// reported, 1 reported ok, 2 reported failed (`loaded_is_stuck`).
fn loaded_state(state: &State, screen: usize) -> Option<u32> {
    let view = view_index_for_screen(&state.defs, SCREEN_ORDER[screen])?;
    state.session.slots.header().view_loaded.get(view as usize).map(|c| c.load(Ordering::Relaxed))
}

/// Whether a `view_loaded` reading means "still not confirmed loaded":
/// true for 0 (never reported) and 2 (reported failed) alike, false only
/// for 1 (reported `ok: true`).
fn loaded_is_stuck(raw: u32) -> bool {
    raw != 1
}

/// Close this screen's browser and make another in its place.
fn remake(screen: usize) {
    STATE.with(|s| {
        let mut borrow = s.borrow_mut();
        let Some(state) = borrow.as_mut() else { return };
        let id = SCREEN_ORDER[screen];
        if let Some(old) = state.screen_browsers[screen].take() {
            let gone = old.identifier();
            state.browsers.retain(|b| b.identifier() != gone);
            if let Some(host) = old.host() {
                // Force: a browser that never started has no page to ask.
                host.close_browser(1);
            }
        }
        let made = if id.eq_ignore_ascii_case(EFB_SCREEN) {
            spawn_efb_view(&state.tag, state.fps)
        } else {
            let def = state.defs.iter().find(|d| d.screen.eq_ignore_ascii_case(id)).cloned();
            def.and_then(|d| spawn_view(&state.tag, &d, state.fps))
        };
        match made {
            Some(browser) => {
                state.screen_browsers[screen] = Some(browser.clone());
                state.browsers.push(browser);
                state.governed = (0, 0);
            }
            None => crate::logging::log(&format!("views: could not make {id}'s browser again")),
        }
    });
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
    // Laid out at EFB_WIDTH x EFB_HEIGHT CSS pixels, painted at
    // EFB_SUPERSAMPLE x into a buffer that size (xphfbw_bridge_views).
    let (dw, dh) = device_size(EFB_SCREEN, EFB_WIDTH, EFB_HEIGHT);
    let block = ScreenBlock::open(tag, EFB_SCREEN, dw, dh)?;
    let screen = Some(Rc::new(block));
    let render_handler = ViewPaint::new(EFB_WIDTH as i32, EFB_HEIGHT as i32, EFB_SUPERSAMPLE as f32, screen);
    let url = format!("http://127.0.0.1:{port}/");
    let load_handler = ViewLoad::new(EFB_SCREEN.to_string(), "EFB".to_string(), url.clone());
    let mut client = ViewClient::new(render_handler, load_handler);
    let window_info = WindowInfo::default().set_as_windowless(unsafe { std::mem::zeroed() });
    let browser_settings = BrowserSettings { windowless_frame_rate: fps.max(1), background_color: 0, ..Default::default() };
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
    let render_handler = ViewPaint::new(width, height, 1.0, screen);
    let url = gauge_url(def);
    let load_screen = if def.screen.is_empty() { "headless".to_string() } else { def.screen.clone() };
    let load_handler = ViewLoad::new(load_screen, def.section.clone(), url.clone());
    let mut client = ViewClient::new(render_handler, load_handler);
    let window_info = WindowInfo::default().set_as_windowless(unsafe { std::mem::zeroed() });
    // Transparent: an ND page's empty areas must show the terrain or weather
    // radar layer the plugin draws under it (MSFS stacks the terrain gauge
    // below nd.html); the plugin draws opaque black under every screen.
    let browser_settings = BrowserSettings { windowless_frame_rate: fps.max(1), background_color: 0, ..Default::default() };
    let mut extra = dictionary_value_create()?;
    extra.set_int(Some(&CefString::from("view")), def.index as i32);
    extra.set_string(Some(&CefString::from("screen")), Some(&CefString::from(def.screen.as_str())));
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

/// Every `on_paint` across all views, and where each one went, so a screen
/// that never updates says why in the app log (`paint_summary`, logged with
/// the periodic status line) instead of looking exactly like one that is
/// merely slow: the plugin only ever sees the frame counter not move.
pub struct PaintStats {
    pub calls: std::sync::atomic::AtomicU64,
    pub not_view: std::sync::atomic::AtomicU64,
    pub empty: std::sync::atomic::AtomicU64,
    pub no_block: std::sync::atomic::AtomicU64,
    pub size_mismatch: std::sync::atomic::AtomicU64,
    /// The last dropped frame's size and the block's, packed 16 bits each:
    /// painted width, painted height, block width, block height.
    pub last_mismatch: std::sync::atomic::AtomicU64,
    pub written: std::sync::atomic::AtomicU64,
}

pub static PAINT_STATS: PaintStats = PaintStats {
    calls: std::sync::atomic::AtomicU64::new(0),
    not_view: std::sync::atomic::AtomicU64::new(0),
    empty: std::sync::atomic::AtomicU64::new(0),
    no_block: std::sync::atomic::AtomicU64::new(0),
    size_mismatch: std::sync::atomic::AtomicU64::new(0),
    last_mismatch: std::sync::atomic::AtomicU64::new(0),
    written: std::sync::atomic::AtomicU64::new(0),
};

/// The paint counters since the last call, for the periodic status line.
pub fn paint_summary() -> String {
    let take = |a: &std::sync::atomic::AtomicU64| a.swap(0, Ordering::Relaxed);
    let (calls, not_view, empty, no_block, mismatch, written) = (
        take(&PAINT_STATS.calls),
        take(&PAINT_STATS.not_view),
        take(&PAINT_STATS.empty),
        take(&PAINT_STATS.no_block),
        take(&PAINT_STATS.size_mismatch),
        take(&PAINT_STATS.written),
    );
    let m = PAINT_STATS.last_mismatch.load(Ordering::Relaxed);
    let sizes = if mismatch > 0 { format!(" (last: painted {}x{}, block {}x{})", m >> 48, (m >> 32) & 0xFFFF, (m >> 16) & 0xFFFF, m & 0xFFFF) } else { String::new() };
    format!("paint: {calls} on_paint, {written} written, {mismatch} size mismatch{sizes}, {no_block} without a screen block, {not_view} popup, {empty} empty")
}

wrap_render_handler! {
    struct ViewPaint {
        width: i32,
        height: i32,
        // Device pixels per CSS pixel: `on_paint` delivers
        // `width * scale` x `height * scale`.
        scale: f32,
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
            info.device_scale_factor = self.scale;
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
            PAINT_STATS.calls.fetch_add(1, Ordering::Relaxed);
            if type_ != PaintElementType::VIEW {
                PAINT_STATS.not_view.fetch_add(1, Ordering::Relaxed);
                return;
            }
            if buffer.is_null() || width <= 0 || height <= 0 {
                PAINT_STATS.empty.fetch_add(1, Ordering::Relaxed);
                return;
            }
            let Some(block) = self.screen.as_deref() else {
                PAINT_STATS.no_block.fetch_add(1, Ordering::Relaxed);
                return;
            };
            paint(block, dirty_rects, buffer, width, height);
        }
    }
}

fn paint(block: &ScreenBlock, dirty_rects: Option<&[Rect]>, buffer: *const u8, width: i32, height: i32) {
    let h = block.header();
    if h.width.load(Ordering::Relaxed) as i32 != width || h.height.load(Ordering::Relaxed) as i32 != height {
        // A resize XPHFBW does not support (panel.cfg's gauge size should
        // always match): drop the frame rather than write out of bounds.
        PAINT_STATS.size_mismatch.fetch_add(1, Ordering::Relaxed);
        PAINT_STATS.last_mismatch.store(
            (width as u64 & 0xFFFF) << 48 | (height as u64 & 0xFFFF) << 32 | (h.width.load(Ordering::Relaxed) as u64 & 0xFFFF) << 16 | (h.height.load(Ordering::Relaxed) as u64 & 0xFFFF),
            Ordering::Relaxed,
        );
        return;
    }
    PAINT_STATS.written.fetch_add(1, Ordering::Relaxed);
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
        load_handler: LoadHandler,
    }

    impl Client {
        fn render_handler(&self) -> Option<RenderHandler> {
            Some(self.render_handler.clone())
        }

        fn load_handler(&self) -> Option<LoadHandler> {
            Some(self.load_handler.clone())
        }
    }
}

// ---------------------------------------------------------------------------
// Load failures: today's whole-`html_ui`-folder loss (S06) made every gauge
// 404 with nothing in the log to say so -- a browser that fails to load just
// paints nothing, indistinguishable from one that is merely slow (its
// `ScreenBlock` frame counter never moves either way until something is
// actually painted). Report the two ways a load can fail: a network/scheme
// error (`on_load_error`) and a page that loaded but got a non-2xx status
// (`on_load_end`; `scheme.rs`'s `not_found` sets exactly this on a missing
// `coui://`/`xphfbw://` file). Only the main frame is reported: a
// sub-resource's own 404 is `scheme.rs`'s job (every request a page makes,
// not just its own top-level load), and a gauge's iframes, if any, would
// otherwise double every failure.
// ---------------------------------------------------------------------------

/// Whether `error_code` is worth logging: `ABORTED` is what a cancelled
/// navigation reports too (`restart_displays`'s `browser.reload()`, or
/// simply navigating away before a page finished), not a failure.
fn is_reportable_error(error_code: Errorcode) -> bool {
    error_code != Errorcode::ABORTED
}

/// Whether an `on_load_end` status is a failure worth logging. CEF gives 0
/// for schemes that never set an explicit code; `coui`/`xphfbw` always
/// answer 200 (`serve`) or 404 (`not_found`, both in `scheme.rs`), so 0 is
/// left alone here rather than treated as a failure.
fn is_load_failure_status(status: ::std::os::raw::c_int) -> bool {
    status != 0 && !(200..300).contains(&status)
}

wrap_load_handler! {
    struct ViewLoad {
        screen: String,
        section: String,
        url: String,
    }

    impl LoadHandler {
        fn on_load_error(
            &self,
            _browser: Option<&mut Browser>,
            frame: Option<&mut Frame>,
            error_code: Errorcode,
            error_text: Option<&CefString>,
            failed_url: Option<&CefString>,
        ) {
            let Some(frame) = frame else { return };
            if frame.is_main() == 0 || !is_reportable_error(error_code) {
                return;
            }
            let text = error_text.map(|t| t.to_string()).unwrap_or_default();
            let url = failed_url.map(|u| u.to_string()).unwrap_or_else(|| self.url.clone());
            crate::logging::log(&format!("views: {} ({}) failed to load {url}: {error_code:?} {text}", self.screen, self.section));
        }

        fn on_load_end(
            &self,
            _browser: Option<&mut Browser>,
            frame: Option<&mut Frame>,
            http_status_code: ::std::os::raw::c_int,
        ) {
            let Some(frame) = frame else { return };
            if frame.is_main() == 0 || !is_load_failure_status(http_status_code) {
                return;
            }
            crate::logging::log(&format!("views: {} ({}) loaded {} with HTTP {http_status_code}", self.screen, self.section, self.url));
        }
    }
}

// ---------------------------------------------------------------------------
// Frame-rate governor: the views render no faster than X-Plane shows them.
// ---------------------------------------------------------------------------

/// How often the governor measures X-Plane's frame rate, ms.
const GOVERNOR_PERIOD_MS: i64 = 1000;
/// The screens whose picture moves every frame in flight (attitude, speed
/// and altitude tapes, a rotating map). They get the full governed rate;
/// every other view (engine and system pages, MFD, OIT, EFB, RMPs, clock)
/// gets half of it.
const SMOOTH_SCREENS: &[&str] = &["SCREEN_DU_PFDL", "SCREEN_DU_PFDR", "SCREEN_DU_NDL", "SCREEN_DU_NDR", "SCREEN_ISIS_1"];
/// Below this a flight instrument's tapes visibly step, whatever X-Plane's
/// own frame rate.
const SMOOTH_FLOOR_FPS: i32 = 15;
const SLOW_FLOOR_FPS: i32 = 10;

/// The render rates for X-Plane's measured frame rate: `displayFps` (`cap`)
/// at most, and never more than X-Plane can put on screen (plus a fifth, so
/// a fresh picture is ready for nearly every X-Plane frame even though the
/// two clocks drift). CEF at 60 fps under an X-Plane at 17 renders
/// two-thirds of its frames for nobody, on the same GPU X-Plane needs.
fn governed_rates(sim_fps: f64, cap: i32) -> (i32, i32) {
    let cap = cap.max(1);
    let wanted = if sim_fps.is_finite() && sim_fps > 0. { (sim_fps * 1.2).ceil() as i32 } else { 0 };
    let smooth = wanted.clamp(SMOOTH_FLOOR_FPS.min(cap), cap);
    let slow = (smooth / 2).clamp(SLOW_FLOOR_FPS.min(cap), cap);
    (smooth, slow)
}

/// Whether a new rate is worth setting: a 1-2 fps wobble in X-Plane's frame
/// rate would otherwise reschedule every view's frame clock every second.
fn rate_moved(current: i32, target: i32) -> bool {
    current <= 0 || (target - current).abs() >= 3
}

fn start_governor_timer() {
    let mut task = GovernorTask::new();
    post_delayed_task(ThreadId::UI, Some(&mut task), GOVERNOR_PERIOD_MS);
}

wrap_task! {
    struct GovernorTask;

    impl Task {
        fn execute(&self) {
            govern();
            start_governor_timer();
        }
    }
}

fn govern() {
    STATE.with(|s| {
        let mut borrow = s.borrow_mut();
        let Some(state) = borrow.as_mut() else { return };
        // The plugin adds 2 to `frame` per X-Plane frame (odd while it
        // writes, `xphfbw_host.rs`'s `publish`).
        let frame = state.session.slots.header().frame.load(Ordering::Acquire);
        let now = std::time::Instant::now();
        let last = state.governor_last.replace((frame, now));
        let Some((last_frame, at)) = last else { return };
        let seconds = (now - at).as_secs_f64();
        if seconds <= 0. {
            return;
        }
        let sim_fps = frame.saturating_sub(last_frame) as f64 / 2. / seconds;
        let (smooth, slow) = governed_rates(sim_fps, state.fps);
        if !rate_moved(state.governed.0, smooth) && !rate_moved(state.governed.1, slow) {
            return;
        }
        let mut smooth_ids = Vec::new();
        for (i, browser) in state.screen_browsers.iter().enumerate() {
            let Some(browser) = browser else { continue };
            if SMOOTH_SCREENS.iter().any(|id| SCREEN_ORDER.get(i).is_some_and(|s| s.eq_ignore_ascii_case(id))) {
                smooth_ids.push(browser.identifier());
            }
        }
        for browser in &state.browsers {
            let rate = if smooth_ids.contains(&browser.identifier()) { smooth } else { slow };
            if let Some(host) = browser.host() {
                host.set_windowless_frame_rate(rate);
            }
        }
        state.governed = (smooth, slow);
        crate::logging::log(&format!(
            "views: X-Plane at {sim_fps:.0} fps: flight instruments render at {smooth}, the other views at {slow} (cap {})",
            state.fps
        ));
    });
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
                let mut borrow = s.borrow_mut();
                let Some(state) = borrow.as_mut() else {
                    crate::logging::log("views: restart-displays requested before views started");
                    return;
                };
                let fps = display_fps(&state.aircraft_root);
                state.fps = fps;
                state.governed = (0, 0);
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
mod governor_tests {
    use super::*;

    #[test]
    fn views_render_no_faster_than_x_plane_shows_them() {
        // X-Plane at 17 fps under a 60 fps cap: about 21, not 60.
        assert_eq!(governed_rates(17., 60), (21, 10));
        // A fast X-Plane is held to the cap.
        assert_eq!(governed_rates(90., 60), (60, 30));
        // Paused, loading, or not measured yet: the floors.
        assert_eq!(governed_rates(0., 60), (15, 10));
        assert_eq!(governed_rates(f64::NAN, 60), (15, 10));
        // A cap below the floors wins.
        assert_eq!(governed_rates(40., 12), (12, 10));
        assert_eq!(governed_rates(40., 8), (8, 8));
    }

    #[test]
    fn a_small_wobble_does_not_reset_every_view() {
        assert!(rate_moved(0, 21), "the first run always sets the rates");
        assert!(!rate_moved(21, 22));
        assert!(!rate_moved(21, 19));
        assert!(rate_moved(21, 24));
        assert!(rate_moved(21, 15));
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
    fn screen_order_has_eighteen_distinct_non_empty_ids() {
        assert_eq!(SCREEN_ORDER.len(), 18);
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
        assert_eq!(screen_slot("SCREEN_OIT_LEFT"), Some(16));
        assert_eq!(screen_slot("SCREEN_OIT_RIGHT"), Some(17));
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
    fn view_index_for_screen_skips_the_efb_and_finds_the_matching_view() {
        let defs = vec![
            ViewDef { index: 0, section: "VCockpit01".into(), gauge_url: "A380X/PFD/pfd.html".into(), width: 768, height: 1024, screen: "SCREEN_DU_PFDL".into() },
            ViewDef { index: 4, section: "VCockpit05".into(), gauge_url: "A380X/OIT/oit.html".into(), width: 1024, height: 768, screen: "SCREEN_OIT_LEFT".into() },
        ];
        assert_eq!(view_index_for_screen(&defs, "SCREEN_DU_PFDL"), Some(0));
        assert_eq!(view_index_for_screen(&defs, "screen_oit_left"), Some(4), "case-insensitive, like screen_slot");
        assert_eq!(view_index_for_screen(&defs, EFB_SCREEN), None, "the EFB is the app's own page, not a FlyByWire gauge, and never calls loaded()");
        assert_eq!(view_index_for_screen(&defs, "SCREEN_UNKNOWN"), None, "no matching view");
    }

    #[test]
    fn loaded_is_stuck_is_true_for_anything_but_a_confirmed_ok() {
        assert!(loaded_is_stuck(0), "never reported");
        assert!(loaded_is_stuck(2), "reported failed");
        assert!(!loaded_is_stuck(1), "reported ok");
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

    #[test]
    fn is_reportable_error_filters_aborted_navigations_only() {
        assert!(!is_reportable_error(Errorcode::ABORTED));
        assert!(is_reportable_error(Errorcode::FAILED));
        assert!(is_reportable_error(Errorcode::FILE_NOT_FOUND));
    }

    #[test]
    fn is_load_failure_status_flags_non_2xx_but_leaves_zero_alone() {
        assert!(!is_load_failure_status(0), "coui/xphfbw always set an explicit code; 0 means some other scheme");
        assert!(!is_load_failure_status(200));
        assert!(!is_load_failure_status(204));
        assert!(!is_load_failure_status(299));
        assert!(is_load_failure_status(404), "scheme.rs's not_found()");
        assert!(is_load_failure_status(500));
        assert!(is_load_failure_status(101));
    }

    #[test]
    fn half_or_more_silent_screens_looks_like_a_crash_loop() {
        assert!(!looks_like_a_crash_loop(0, 18), "nothing silent is not a crash loop");
        assert!(!looks_like_a_crash_loop(1, 18), "one unlucky browser is not a crash loop");
        assert!(!looks_like_a_crash_loop(8, 18), "under half stays the isolated-browser explanation");
        assert!(looks_like_a_crash_loop(9, 18), "exactly half is the line, inclusive");
        assert!(looks_like_a_crash_loop(18, 18), "every screen black is the textbook case");
        assert!(!looks_like_a_crash_loop(0, 0), "no screens at all is never a crash loop");
    }
}
