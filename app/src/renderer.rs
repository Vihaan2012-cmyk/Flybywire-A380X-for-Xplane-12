//! CEF renderer process: installs `window.__xphfbw` in every FBW gauge
//! view's main frame (docs/briefs/xphfbw-js-bridge.md, "The renderer-side
//! API contract", agent E), and the browser-process hook that tells each
//! child process which session, X-Plane install and aircraft it belongs to.
//!
//! `src/xphfbw_bridge.rs` (the plugin crate, opened here read-only) is the
//! frozen wire protocol; this file does not change its layouts. Per-view
//! runtime state (the seqlock snapshot, the read-your-writes overlay, call
//! ids, caches) lives in thread-local storage keyed by CEF `Browser`
//! identifier, because every V8 value this file touches belongs to the
//! renderer's single JS/blink thread and must never cross threads — the
//! shared-memory session objects (`Shared`, `NamedMutex`-backed rings) are
//! `Send` but not `Sync`, which a `thread_local!` sidesteps entirely rather
//! than fighting.

mod snapshot;
mod stored_data;
mod wmm;

use std::cell::{Cell, OnceCell, RefCell};
use std::collections::{HashMap, HashSet};
use std::path::{Path, PathBuf};
use std::rc::Rc;
use std::sync::atomic::Ordering;

use cef::*;
use fbw_a380_systems::xphfbw_bridge::{Downlink, Session, Uplink};

/// Native functions installed on `window.__xphfbw` (contract rule list
/// item 81-97 in the brief). Order does not matter; every one is bound to
/// the same [`XphfbwV8Handler`], dispatched on `name`.
const FUNCTIONS: &[&str] =
    &["snapshot", "getVar", "setVar", "getString", "setString", "sendEvent", "call", "poll", "gameString", "storedData", "readFile", "log", "magVar", "loaded"];

fn cs(s: &str) -> CefString {
    CefString::from(s)
}

// ---------------------------------------------------------------------------
// Process-wide and per-view state (thread-local: renderer-thread only).
// ---------------------------------------------------------------------------

/// What every view in this renderer process shares: the bridge session
/// (opened once, by the `--xphfbw-tag` the browser process appended in
/// `on_before_child_process_launch`) and the X-Plane/aircraft folders.
struct ProcessInfo {
    xp_root: PathBuf,
    aircraft_dir: PathBuf,
    session: Rc<Session>,
}

/// One gauge view's state: which slot values it last saw (rule 1), writes
/// it is still shadowing (rule 3), in-flight `call()` promises (rule 5),
/// and the small string/GAME caches `getString`/`gameString` serve from.
struct ViewState {
    view: u32,
    screen: String,
    aircraft_dir: PathBuf,
    xp_root: PathBuf,
    session: Rc<Session>,
    snapshot: RefCell<Option<snapshot::Snapshot>>,
    overlay: RefCell<snapshot::Overlay>,
    call_seq: Cell<u64>,
    pending_calls: RefCell<HashMap<u64, V8Value>>,
    game_strings: RefCell<HashMap<String, String>>,
    game_requested: RefCell<HashSet<String>>,
    strings: RefCell<HashMap<String, String>>,
    /// Downlink records drained while `gameString` waited for its answer,
    /// handed out by the next `poll()` in their original order (rule 4).
    stash: RefCell<Vec<Downlink>>,
}

impl ViewState {
    fn new(view: u32, screen: String, process: &Rc<ProcessInfo>) -> Self {
        Self {
            view,
            screen,
            aircraft_dir: process.aircraft_dir.clone(),
            xp_root: process.xp_root.clone(),
            session: process.session.clone(),
            snapshot: RefCell::new(None),
            overlay: RefCell::new(snapshot::Overlay::new()),
            call_seq: Cell::new(0),
            pending_calls: RefCell::new(HashMap::new()),
            game_strings: RefCell::new(HashMap::new()),
            game_requested: RefCell::new(HashSet::new()),
            strings: RefCell::new(HashMap::new()),
            stash: RefCell::new(Vec::new()),
        }
    }
}

thread_local! {
    static PROCESS: OnceCell<Option<Rc<ProcessInfo>>> = const { OnceCell::new() };
    static VIEWS: RefCell<HashMap<i32, Rc<ViewState>>> = RefCell::new(HashMap::new());
}

/// Reads `--xphfbw-tag`/`--xp-root`/`--aircraft` from this process's own
/// command line (set by the browser process before spawning it) and opens
/// the bridge session, once per process; every view in this process shares
/// the result.
fn process_info() -> Option<Rc<ProcessInfo>> {
    PROCESS.with(|cell| cell.get_or_init(load_process_info).clone())
}

fn load_process_info() -> Option<Rc<ProcessInfo>> {
    let cl = command_line_get_global()?;
    let tag = switch_value(&cl, "xphfbw-tag")?;
    let xp_root = PathBuf::from(switch_value(&cl, "xp-root")?);
    let aircraft_dir = PathBuf::from(switch_value(&cl, "aircraft")?);
    let session = Session::open(&tag)?;
    Some(Rc::new(ProcessInfo { xp_root, aircraft_dir, session: Rc::new(session) }))
}

fn switch_value(cl: &CommandLine, name: &str) -> Option<String> {
    let key = cs(name);
    if cl.has_switch(Some(&key)) == 0 {
        return None;
    }
    let raw = cl.switch_value(Some(&key));
    let s = CefString::from(&raw).to_string();
    (!s.is_empty()).then_some(s)
}

fn current_view() -> Option<Rc<ViewState>> {
    let ctx = v8_context_get_current_context()?;
    let browser = ctx.browser()?;
    VIEWS.with(|v| v.borrow().get(&browser.identifier()).cloned())
}

// ---------------------------------------------------------------------------
// snapshot() / getVar / setVar — rules 1-3.
// ---------------------------------------------------------------------------

/// Copies every slot's value under the plugin's seqlock (rule 1) and
/// records how many were resolved as of that frame (rule 2), then drops any
/// overlay entries the plugin has since republished past (rule 3).
fn take_snapshot(view: &ViewState) {
    let table = &view.session.slots;
    let header = table.header();
    let (frame, values) = snapshot::seqlock_read(|| header.frame.load(Ordering::Acquire), || table.values().to_vec());
    let resolved = header.resolved.load(Ordering::Acquire);
    view.overlay.borrow_mut().prune(frame);
    *view.snapshot.borrow_mut() = Some(snapshot::Snapshot { frame, resolved, values });
}

/// `getVar` takes a snapshot itself only if this view has not taken one yet
/// (the runtime is expected to call `snapshot()` once per rAF tick; this is
/// the fallback for a `getVar` before that has happened).
fn ensure_snapshot(view: &ViewState) {
    if view.snapshot.borrow().is_none() {
        take_snapshot(view);
    }
}

fn get_var(view: &ViewState, name: &str, unit: &str) -> f64 {
    let Some(slot) = view.session.slots.register(name, unit) else {
        return 0.0; // The slot table is full.
    };
    ensure_snapshot(view);
    let (raw, frame) = {
        let snap = view.snapshot.borrow();
        let snap = snap.as_ref().expect("just ensured");
        (snap.raw(slot), snap.frame)
    };
    view.overlay.borrow_mut().resolve(slot, raw, frame)
}

fn set_var(view: &ViewState, name: &str, unit: &str, value: f64) {
    let Some(slot) = view.session.slots.register(name, unit) else { return };
    view.session.uplink.push(&Uplink::Write { slot, value }.encode());
    let frame_at_write = view.snapshot.borrow().as_ref().map(|s| s.frame).unwrap_or_else(|| view.session.slots.header().frame.load(Ordering::Acquire));
    view.overlay.borrow_mut().set(slot, value, frame_at_write);
}

// ---------------------------------------------------------------------------
// call() / poll() — rules 4-5.
// ---------------------------------------------------------------------------

fn call(view: &ViewState, name: String, args_json: String) -> Option<V8Value> {
    let counter = view.call_seq.get() + 1;
    view.call_seq.set(counter);
    let id = ((view.view as u64) << 48) | counter;
    view.session.uplink.push(&Uplink::Call { view: view.view, id, name, args_json }.encode());
    let promise = v8_value_create_promise()?;
    view.pending_calls.borrow_mut().insert(id, promise.clone());
    Some(promise)
}

fn resolve_call(view: &ViewState, id: u64, ok: bool, text: &str) {
    let Some(promise) = view.pending_calls.borrow_mut().remove(&id) else { return };
    if ok {
        if let Some(mut v) = v8_value_create_string(Some(&cs(text))) {
            promise.resolve_promise(Some(&mut v));
        }
    } else {
        promise.reject_promise(Some(&cs(text)));
    }
}

fn make_triplet(kind: &str, name: &str, payload: &str) -> Option<V8Value> {
    let arr = v8_value_create_array(3)?;
    let mut k = v8_value_create_string(Some(&cs(kind)))?;
    let mut n = v8_value_create_string(Some(&cs(name)))?;
    let mut p = v8_value_create_string(Some(&cs(payload)))?;
    arr.set_value_byindex(0, Some(&mut k));
    arr.set_value_byindex(1, Some(&mut n));
    arr.set_value_byindex(2, Some(&mut p));
    Some(arr)
}

/// Drains this view's downlink: `Reply` records resolve/reject the matching
/// `call()` promise (never surfaced to JS); everything else becomes one
/// `[kind, name, payload]` entry, kinds `"h"`, `"provider"`, `"game"`,
/// `"string"` per the contract.
fn poll(view: &ViewState) -> Option<V8Value> {
    let ring = view.session.downlinks.get(view.view as usize)?;
    let mut out: Vec<(&'static str, String, String)> = Vec::new();
    let stashed = std::mem::take(&mut *view.stash.borrow_mut());
    let fresh = ring.drain().into_iter().filter_map(|bytes| Downlink::decode(&bytes));
    for record in stashed.into_iter().chain(fresh) {
        match record {
            Downlink::Reply { id, ok, text } => resolve_call(view, id, ok, &text),
            Downlink::HEvent { name } => out.push(("h", name, String::new())),
            Downlink::ProviderEvent { name, json } => out.push(("provider", name, json)),
            Downlink::GameString { name, value } => {
                view.game_strings.borrow_mut().insert(name.clone(), value.clone());
                out.push(("game", name, value));
            }
            Downlink::StringValue { name, value } => {
                view.strings.borrow_mut().insert(name.clone(), value.clone());
                out.push(("string", name, value));
            }
        }
    }
    let arr = v8_value_create_array(out.len() as i32)?;
    for (i, (kind, name, payload)) in out.into_iter().enumerate() {
        if let Some(mut triple) = make_triplet(kind, &name, &payload) {
            arr.set_value_byindex(i as i32, Some(&mut triple));
        }
    }
    Some(arr)
}

fn game_string(view: &ViewState, name: &str) -> String {
    if let Some(value) = view.game_strings.borrow().get(name) {
        return value.clone();
    }
    if !view.game_requested.borrow_mut().insert(name.to_string()) {
        return String::new();
    }
    view.session.uplink.push(&Uplink::GameString { view: view.view, name: name.to_string() }.encode());
    // MSFS answers GAME strings synchronously, and FlyByWire reads some once
    // at start-up (MsfsBackend's FLIGHT NAVDATA DATE RANGE): wait for the
    // plugin's answer, a frame or two, keeping whatever else arrives
    // meanwhile for the next poll().
    let Some(ring) = view.session.downlinks.get(view.view as usize) else { return String::new() };
    let until = std::time::Instant::now() + std::time::Duration::from_millis(GAME_STRING_WAIT_MS);
    while std::time::Instant::now() < until {
        for record in ring.drain().into_iter().filter_map(|bytes| Downlink::decode(&bytes)) {
            if let Downlink::GameString { name: n, value } = &record {
                view.game_strings.borrow_mut().insert(n.clone(), value.clone());
            }
            view.stash.borrow_mut().push(record);
        }
        if let Some(value) = view.game_strings.borrow().get(name) {
            return value.clone();
        }
        std::thread::sleep(std::time::Duration::from_millis(2));
    }
    String::new()
}

/// How long `gameString` waits for a value it has never seen.
const GAME_STRING_WAIT_MS: u64 = 250;

// ---------------------------------------------------------------------------
// readFile — html_ui-relative or /VFS/ paths, from the aircraft dir.
// ---------------------------------------------------------------------------

fn read_file(aircraft_dir: &Path, requested: &str) -> Option<String> {
    // The same mapping as scheme.rs's coui://: `/VFS/...` is the aircraft
    // package root, anything else is under its `html_ui` folder.
    let (base, rel) = match requested.strip_prefix("/VFS/") {
        Some(rest) => (aircraft_dir.to_path_buf(), rest),
        None => (aircraft_dir.join("html_ui"), requested),
    };
    let rel = rel.trim_start_matches(['/', '\\']);
    // No parent-directory traversal, and no escaping the aircraft folder via
    // an absolute path or a drive letter/UNC prefix (`Path::join` would
    // otherwise happily replace the base with an absolute joined path).
    if rel.split(['/', '\\']).any(|part| part == "..") || Path::new(rel).is_absolute() || rel.contains(':') {
        return None;
    }
    std::fs::read_to_string(base.join(rel)).ok()
}

// ---------------------------------------------------------------------------
// V8 argument helpers.
// ---------------------------------------------------------------------------

fn arg(args: &[Option<V8Value>], i: usize) -> Option<&V8Value> {
    args.get(i).and_then(|a| a.as_ref())
}

fn arg_str(args: &[Option<V8Value>], i: usize) -> String {
    arg(args, i).map(|v| CefString::from(&v.string_value()).to_string()).unwrap_or_default()
}

fn arg_f64(args: &[Option<V8Value>], i: usize) -> f64 {
    arg(args, i).map(|v| v.double_value()).unwrap_or(0.0)
}

fn arg_f64_array(args: &[Option<V8Value>], i: usize) -> Vec<f64> {
    let Some(v) = arg(args, i) else { return Vec::new() };
    let len = v.array_length().max(0);
    (0..len).filter_map(|idx| v.value_byindex(idx)).map(|e| e.double_value()).collect()
}

// ---------------------------------------------------------------------------
// Dispatch: one V8Handler for every `__xphfbw` function, matched by name.
// ---------------------------------------------------------------------------

fn dispatch(name: &str, args: &[Option<V8Value>]) -> Result<Option<V8Value>, String> {
    // magVar is a pure function: no session, no view needed.
    if name == "magVar" {
        let lat = arg_f64(args, 0);
        let lon = arg_f64(args, 1);
        return Ok(v8_value_create_double(wmm::declination(lat, lon)));
    }

    let view = current_view().ok_or_else(|| "xphfbw: this view's session is gone".to_string())?;
    match name {
        "snapshot" => {
            take_snapshot(&view);
            Ok(None)
        }
        "getVar" => {
            let value = get_var(&view, &arg_str(args, 0), &arg_str(args, 1));
            Ok(v8_value_create_double(value))
        }
        "setVar" => {
            set_var(&view, &arg_str(args, 0), &arg_str(args, 1), arg_f64(args, 2));
            Ok(None)
        }
        "getString" => {
            let name = arg_str(args, 0);
            let value = view.strings.borrow().get(&name).cloned().unwrap_or_default();
            Ok(v8_value_create_string(Some(&cs(&value))))
        }
        "setString" => {
            let name = arg_str(args, 0);
            let value = arg_str(args, 1);
            view.strings.borrow_mut().insert(name.clone(), value.clone());
            view.session.uplink.push(&Uplink::SetString { name, value }.encode());
            Ok(None)
        }
        "sendEvent" => {
            let name = arg_str(args, 0);
            let values = arg_f64_array(args, 1);
            view.session.uplink.push(&Uplink::Event { name, values }.encode());
            Ok(None)
        }
        "call" => Ok(call(&view, arg_str(args, 0), arg_str(args, 1))),
        "poll" => Ok(poll(&view)),
        "gameString" => {
            let value = game_string(&view, &arg_str(args, 0));
            Ok(v8_value_create_string(Some(&cs(&value))))
        }
        "storedData" => {
            let op = arg_str(args, 0);
            let key = arg_str(args, 1);
            let value = arg_str(args, 2);
            let result = stored_data::run(&view.xp_root, &op, &key, &value);
            Ok(v8_value_create_string(Some(&cs(&result))))
        }
        "readFile" => match read_file(&view.aircraft_dir, &arg_str(args, 0)) {
            Some(text) => Ok(v8_value_create_string(Some(&cs(&text)))),
            None => Ok(v8_value_create_null()),
        },
        "log" => {
            let level = arg(args, 0).map(|v| v.int_value()).unwrap_or(0).clamp(0, 2) as u8;
            let text = arg_str(args, 1);
            view.session.uplink.push(&Uplink::Log { view: view.view, level, text }.encode());
            Ok(None)
        }
        "loaded" => {
            let ok = arg(args, 0).map(|v| v.bool_value() != 0).unwrap_or(false);
            let text = arg_str(args, 1);
            view.session.uplink.push(&Uplink::Loaded { view: view.view, ok, text }.encode());
            Ok(None)
        }
        other => Err(format!("xphfbw: unknown native \"{other}\"")),
    }
}

wrap_v8_handler! {
    struct XphfbwV8Handler;

    impl V8Handler {
        fn execute(
            &self,
            name: Option<&CefString>,
            _object: Option<&mut V8Value>,
            arguments: Option<&[Option<V8Value>]>,
            retval: Option<&mut Option<V8Value>>,
            exception: Option<&mut CefString>,
        ) -> ::std::os::raw::c_int {
            let name = name.map(|n| n.to_string()).unwrap_or_default();
            let args: &[Option<V8Value>] = arguments.unwrap_or(&[]);
            match dispatch(&name, args) {
                Ok(value) => {
                    if let Some(r) = retval {
                        *r = value;
                    }
                }
                Err(message) => {
                    if let Some(e) = exception {
                        *e = CefString::from(message.as_str());
                    }
                }
            }
            1
        }
    }
}

// ---------------------------------------------------------------------------
// Installing window.__xphfbw.
// ---------------------------------------------------------------------------

fn build_object(view: &ViewState) -> Option<V8Value> {
    let obj = v8_value_create_object(None, None)?;
    let attr = V8Propertyattribute::default();

    if let Some(mut v) = v8_value_create_uint(view.view) {
        obj.set_value_bykey(Some(&cs("view")), Some(&mut v), attr);
    }
    if let Some(mut v) = v8_value_create_string(Some(&cs(&view.screen))) {
        obj.set_value_bykey(Some(&cs("screen")), Some(&mut v), attr);
    }
    if let Some(mut v) = v8_value_create_string(Some(&cs(&view.aircraft_dir.to_string_lossy()))) {
        obj.set_value_bykey(Some(&cs("aircraftDir")), Some(&mut v), attr);
    }
    for name in FUNCTIONS {
        let mut handler = XphfbwV8Handler::new();
        if let Some(mut f) = v8_value_create_function(Some(&cs(name)), Some(&mut handler)) {
            obj.set_value_bykey(Some(&cs(name)), Some(&mut f), attr);
        }
    }
    Some(obj)
}

fn install(view: &ViewState, context: &mut V8Context) {
    context.enter();
    if let Some(global) = context.global() {
        if let Some(mut obj) = build_object(view) {
            global.set_value_bykey(Some(&cs("__xphfbw")), Some(&mut obj), V8Propertyattribute::default());
        }
    }
    context.exit();
}

wrap_render_process_handler! {
    struct XphfbwRenderProcessHandler;

    impl RenderProcessHandler {
        /// The browser process created a new view; if agent G's
        /// `extra_info` carries `view`/`screen` (the agreed dictionary
        /// keys), this is one of FBW's gauge views, not the app's own
        /// settings window (which has no `extra_info` at all).
        fn on_browser_created(&self, browser: Option<&mut Browser>, extra_info: Option<&mut DictionaryValue>) {
            let (Some(browser), Some(extra_info)) = (browser, extra_info) else { return };
            if extra_info.has_key(Some(&cs("view"))) == 0 {
                return;
            }
            let Some(process) = process_info() else { return };
            let view_index = extra_info.int(Some(&cs("view"))).max(0) as u32;
            let screen = CefString::from(&extra_info.string(Some(&cs("screen")))).to_string();
            let state = Rc::new(ViewState::new(view_index, screen, &process));
            VIEWS.with(|v| v.borrow_mut().insert(browser.identifier(), state));
        }

        fn on_browser_destroyed(&self, browser: Option<&mut Browser>) {
            if let Some(browser) = browser {
                VIEWS.with(|v| {
                    v.borrow_mut().remove(&browser.identifier());
                });
            }
        }

        fn on_context_created(
            &self,
            browser: Option<&mut Browser>,
            frame: Option<&mut Frame>,
            context: Option<&mut V8Context>,
        ) {
            let (Some(browser), Some(frame), Some(context)) = (browser, frame, context) else { return };
            if frame.is_main() == 0 {
                return; // Only the view's main frame gets window.__xphfbw.
            }
            let Some(view) = VIEWS.with(|v| v.borrow().get(&browser.identifier()).cloned()) else { return };
            install(&view, context);
        }
    }
}

/// `app/src/window.rs`'s hook: `XphfbwApp::render_process_handler`.
pub fn new_render_process_handler() -> RenderProcessHandler {
    XphfbwRenderProcessHandler::new()
}

// ---------------------------------------------------------------------------
// Browser process: tag every child process with the session it belongs to.
// ---------------------------------------------------------------------------

/// `app/src/window.rs`'s hook:
/// `XphfbwBrowserProcessHandler::on_before_child_process_launch`. Every
/// renderer (and other CEF child process) needs to know which bridge
/// session, X-Plane install and aircraft folder it belongs to, since that
/// cannot be discovered any other way once it is off in its own process.
pub fn append_child_switches(command_line: Option<&mut CommandLine>) {
    let (Some(cl), Some(shared)) = (command_line, crate::window::SHARED.get()) else { return };
    let Some(tag) = shared.tag.as_deref() else { return };
    cl.append_switch_with_value(Some(&cs("xphfbw-tag")), Some(&cs(tag)));
    cl.append_switch_with_value(Some(&cs("xp-root")), Some(&cs(&shared.xplane_root.to_string_lossy())));
    cl.append_switch_with_value(Some(&cs("aircraft")), Some(&cs(&shared.aircraft_dir.to_string_lossy())));
}

#[cfg(test)]
mod tests {
    use super::read_file;

    #[test]
    fn read_file_maps_paths_like_coui() {
        let dir = std::env::temp_dir().join(format!("xphfbw-readfile-{}", std::process::id()));
        std::fs::create_dir_all(dir.join("html_ui/Pages")).unwrap();
        std::fs::create_dir_all(dir.join("panel")).unwrap();
        std::fs::write(dir.join("html_ui/Pages/a.js"), "page").unwrap();
        std::fs::write(dir.join("panel/panel.cfg"), "cfg").unwrap();
        assert_eq!(read_file(&dir, "/Pages/a.js").as_deref(), Some("page"));
        assert_eq!(read_file(&dir, "/VFS/panel/panel.cfg").as_deref(), Some("cfg"));
        assert_eq!(read_file(&dir, "/Pages/../../panel/panel.cfg"), None);
        let _ = std::fs::remove_dir_all(&dir);
    }
}
