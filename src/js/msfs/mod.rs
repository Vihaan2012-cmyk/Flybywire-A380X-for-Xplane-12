//! MSFS's instrument runtime: FlyByWire's instruments and hosts, as MSFS
//! builds and ships them, run unmodified on the engine.
//!
//! MSFS runs each `[VCockpitNN]` section of panel.cfg as its own Coherent
//! view: a VCockpit page (asobo-vcockpits-core `VCockpit.html`) with its
//! own document, globals and `requestAnimationFrame`, into which the
//! section's `htmlgaugeNN` pages are imported one after the other. Here
//! each view is its own [`Engine`], set up with the scripts in this folder
//! (the MSFS side of Coherent, `SimVar`, the view's environment and the
//! VCockpit panel and `BaseInstrument`), then a DOM (`src/js/dom`, or the
//! stand-in until that is there), then the panel's gauges.
//!
//! The simulator side MSFS gives every view is [`Cockpit`]: it carries
//! events between views (generic data listeners, flow events, H: events),
//! key events and their intercepts, the stored data, Coherent calls it
//! passes to the plugin's providers (answered now or on a later tick), and
//! the files under `html_ui`.

use std::collections::{HashMap, HashSet};
use std::path::PathBuf;
use std::time::{Duration, Instant};

use super::{CallReply, Engine, EngineOptions, Host, LogLevel};

/// The runtime's scripts, in the order a view runs them. The DOM goes in
/// after `window.js`.
const BEFORE_DOM: &[(&str, &str)] = &[("msfs/window.js", include_str!("window.js"))];
const AFTER_DOM: &[(&str, &str)] = &[
    ("msfs/coherent.js", include_str!("coherent.js")),
    ("msfs/simvar.js", include_str!("simvar.js")),
    ("msfs/environment.js", include_str!("environment.js")),
    ("msfs/instrument.js", include_str!("instrument.js")),
];
const DOM_STANDIN: (&str, &str) = ("msfs/dom_standin.js", include_str!("dom_standin.js"));

/// Gauges not run: the EFB (out of scope — the app draws its own on that
/// mesh), the OITs' superseded *legacy* page, the popup, and WASM gauges,
/// which are native modules rather than pages. The current OIT page
/// (`A380X/OIT/oit.html`) does run (docs/oit.md).
pub const EXCLUDED_GAUGES: &[&str] = &["A380X/EFB/", "A380X/OITlegacy/", "A380X/popup/", "WasmInstrument/"];

/// One `htmlgaugeNN` line.
#[derive(Clone, Debug, PartialEq)]
pub struct PanelGauge {
    /// The page, relative to `/Pages/VCockpit/Instruments/`, with its query.
    pub url: String,
    /// x, y, width, height in the section's logical size.
    pub rect: [i32; 4],
    /// What follows the rectangle (`L` for the captain's terronnd).
    pub params: Vec<String>,
}

/// The id the renderer knows a native gauge's image by
/// (docs/display-stream.md, 60 NATIVE_IMAGE): FlyByWire's terronnd.wasm
/// gauge is `TERRONND_` and its side parameter.
pub fn native_image_id(gauge: &PanelGauge) -> Option<String> {
    let url = gauge.url.to_ascii_lowercase();
    if url.starts_with("wasminstrument/") && url.contains("wasm_gauge=terronnd") {
        let side = gauge.params.first()?.to_ascii_uppercase();
        return Some(format!("TERRONND_{side}"));
    }
    None
}

/// One `[VCockpitNN]` section.
#[derive(Clone, Debug, PartialEq)]
pub struct PanelView {
    pub name: String,
    /// The texture it draws into, without `$`; `None` for `NO_TEXTURE`.
    pub texture: Option<String>,
    /// `size_mm`: the size gauge rectangles are given in.
    pub logical: (u32, u32),
    /// `pixel_size`: the page's size.
    pub display: (u32, u32),
    pub gauges: Vec<PanelGauge>,
}

/// The views panel.cfg describes, in its order. Gauge lines keep their
/// order in the section; a commented-out line (`;htmlgauge00=`) is not one.
pub fn parse_panel_cfg(text: &str) -> Vec<PanelView> {
    let mut views: Vec<PanelView> = Vec::new();
    for raw in text.lines() {
        let line = raw.split("//").next().unwrap_or("").trim();
        if line.starts_with(';') || line.is_empty() {
            continue;
        }
        if let Some(section) = line.strip_prefix('[').and_then(|l| l.strip_suffix(']')) {
            if section.to_ascii_lowercase().starts_with("vcockpit") {
                views.push(PanelView { name: section.to_string(), texture: None, logical: (0, 0), display: (0, 0), gauges: Vec::new() });
            } else {
                // A section that is not a view ends the one before it.
                views.push(PanelView { name: String::new(), texture: None, logical: (0, 0), display: (0, 0), gauges: Vec::new() });
            }
            continue;
        }
        let Some(view) = views.last_mut() else { continue };
        if view.name.is_empty() {
            continue;
        }
        let Some((key, value)) = line.split_once('=') else { continue };
        let key = key.trim().to_ascii_lowercase();
        let value = value.split(';').next().unwrap_or("").trim();
        let pair = |v: &str| -> (u32, u32) {
            let mut it = v.split(',').map(|n| n.trim().parse::<f64>().unwrap_or(0.) as u32);
            (it.next().unwrap_or(0), it.next().unwrap_or(0))
        };
        match key.as_str() {
            "size_mm" => view.logical = pair(value),
            "pixel_size" => view.display = pair(value),
            "texture" => {
                let t = value.trim_start_matches('$');
                view.texture = (!t.eq_ignore_ascii_case("NO_TEXTURE") && !t.is_empty()).then(|| t.to_string());
            }
            k if k.starts_with("htmlgauge") => {
                let mut parts = value.split(',');
                let url = parts.next().unwrap_or("").trim().to_string();
                let mut rect = [0i32; 4];
                for r in rect.iter_mut() {
                    *r = parts.next().and_then(|p| p.trim().parse::<f64>().ok()).unwrap_or(0.) as i32;
                }
                let params = parts.map(|p| p.trim().to_string()).filter(|p| !p.is_empty()).collect();
                if !url.is_empty() {
                    view.gauges.push(PanelGauge { url, rect, params });
                }
            }
            _ => {}
        }
    }
    views.retain(|v| !v.name.is_empty());
    views
}

/// How the cockpit is set up.
pub struct CockpitOptions {
    /// The `html_ui` folder: MSFS's `coui://html_ui/`.
    pub html_ui: PathBuf,
    pub panel_cfg: String,
    /// panel.xml, which instruments read their `<Instrument>` config from.
    pub panel_xml: String,
    /// Where stored data (`SetStoredData`) is kept between flights.
    pub datastore: Option<PathBuf>,
    /// Where some stored-data keys are kept instead (the flyPad settings).
    pub settings: Option<Box<dyn StoredDataBackend>>,
    /// Use the node-tree-only stand-in DOM instead of `src/js/dom`, which
    /// paints (for headless runs of logic alone).
    pub standin_dom: bool,
    /// Changes to the scripts as they load, for parts of FlyByWire's code
    /// the plugin runs natively instead.
    pub patches: Vec<SourcePatch>,
    /// Gauge URLs starting with these are left out.
    pub excluded: Vec<String>,
    /// Longest one call into a view's scripts may run before it is
    /// interrupted. This is a watchdog, not a frame budget: an interrupted
    /// instrument is left half way through its update.
    pub budget: Duration,
    pub memory_limit: usize,
}

impl CockpitOptions {
    pub fn new(html_ui: PathBuf, panel_cfg: String) -> Self {
        Self {
            html_ui,
            panel_cfg,
            panel_xml: String::new(),
            datastore: None,
            settings: None,
            standin_dom: false,
            patches: Vec::new(),
            excluded: EXCLUDED_GAUGES.iter().map(|s| s.to_string()).collect(),
            budget: Duration::from_millis(500),
            memory_limit: 1024 * 1024 * 1024,
        }
    }
}

/// A change to a script as it loads: in the file at `path` (under html_ui),
/// the text `find`, which must occur exactly once, becomes `replace`. A
/// patch that does not match is reported and the file runs unchanged.
#[derive(Clone, Debug)]
pub struct SourcePatch {
    pub path: String,
    pub find: String,
    pub replace: String,
    /// Why, for the log.
    pub reason: String,
}

/// What one view costs.
#[derive(Clone, Debug, Default)]
pub struct ViewStats {
    pub name: String,
    pub gauges: Vec<String>,
    pub load_ms: f64,
    pub last_tick_ms: f64,
    pub max_tick_ms: f64,
    pub total_tick_ms: f64,
    pub ticks: u64,
    pub memory: usize,
}

/// A native gauge in a view: its image id, its rectangle in the view's
/// logical size, and whether an HTML gauge comes before it in panel.cfg.
struct NativeGauge {
    id: String,
    rect: [i32; 4],
    over: bool,
}

struct View {
    panel: PanelView,
    natives: Vec<NativeGauge>,
    engine: Engine,
    loaded: bool,
    failed: bool,
    /// Items for `__msfsDeliver`, as JSON.
    inbox: Vec<String>,
    /// Consecutive ticks the current `inbox` batch has failed to deliver
    /// because the engine's own watchdog interrupted it (`Cockpit::tick`'s
    /// doc comment on the click/event delivery retry, debug_screen_clicks.md):
    /// a script that is merely slow this one frame gets a fresh budget next
    /// tick instead of silently losing whatever was in the batch (a cockpit
    /// screen click, most consequentially), but a batch that keeps timing
    /// out is dropped after [`INBOX_RETRY_LIMIT`] tries rather than growing
    /// forever as later frames' events keep appending to it.
    inbox_stalls: u32,
    stats: ViewStats,
}

/// See [`View::inbox_stalls`].
const INBOX_RETRY_LIMIT: u32 = 3;

/// Whether an `__msfsDeliver` batch that just failed with `error` (the
/// engine's own error text, [`Engine::invoke`]'s `Err`) should be kept
/// queued for a retry on the next tick, `stalls` being [`View::inbox_stalls`]
/// after counting this failure: only for the interrupt the engine's own
/// watchdog raises (`js/mod.rs`'s `a_runaway_script_is_interrupted` test:
/// its message always contains "interrupt"), never for an ordinary script
/// error (that would just repeat forever, an infinite loop of its own), and
/// never past [`INBOX_RETRY_LIMIT`] consecutive tries (a script that is
/// truly hung, not merely slow this one frame, must not grow the batch
/// forever as later frames' own events keep appending to it).
fn should_retry_delivery(error: &str, stalls: u32) -> bool {
    stalls <= INBOX_RETRY_LIMIT && error.to_ascii_lowercase().contains("interrupt")
}

/// Where a delivery goes.
#[derive(Clone, Copy)]
enum To {
    All,
    Others(usize),
    View(usize),
}

struct PendingCall {
    id: u64,
    view: usize,
    name: String,
    args: String,
}

/// Everything the views share: the simulator side.
struct Shared {
    root: PathBuf,
    outbox: Vec<(To, String)>,
    /// Key name (upper case, no `K:`) -> the views intercepting it, and
    /// whether each keeps the event from the simulator.
    intercepts: HashMap<String, Vec<(usize, bool)>>,
    pending: Vec<PendingCall>,
    next_call: u64,
    store: DataStore,
    /// Instrument identifiers (`A380X_PFD_1`) by the view they are in.
    instruments: Vec<(usize, String, String)>,
    patches: Vec<SourcePatch>,
}

/// The views and the simulator side they share.
pub struct Cockpit {
    views: Vec<View>,
    shared: Shared,
    panel_xml: String,
}

impl Cockpit {
    /// Make a view for each panel.cfg section with a gauge to run, and run
    /// the runtime's scripts in it. The gauges load on [`Cockpit::tick`].
    /// `setup` runs first in each view's engine, to add the plugin's host
    /// functions (the display's).
    pub fn new(options: CockpitOptions, setup: &dyn Fn(&Engine) -> Result<(), String>) -> Result<Self, String> {
        let mut views = Vec::new();
        for mut panel in parse_panel_cfg(&options.panel_cfg) {
            // Native gauges draw into the view's screen too: their images go
            // under the HTML gauges listed after them and over those before.
            let mut natives = Vec::new();
            let mut html_seen = false;
            for g in &panel.gauges {
                match native_image_id(g) {
                    Some(id) => natives.push(NativeGauge { id, rect: g.rect, over: html_seen }),
                    None => html_seen |= !g.url.to_ascii_lowercase().starts_with("wasminstrument/"),
                }
            }
            // [wxr] FlyByWire ships no WXR gauge (docs/wxr.md), so there is
            // nothing in panel.cfg for native_image_id to recognise; compose
            // WXR_L/WXR_R in the same slot as the terronnd gauge it is
            // mutually exclusive with (same rect, same side of the HTML
            // gauge), wherever that gauge's id shows one is present.
            let wxr_gauges: Vec<NativeGauge> = natives
                .iter()
                .filter_map(|n| n.id.strip_prefix("TERRONND_").map(|side| NativeGauge { id: format!("WXR_{side}"), rect: n.rect, over: n.over }))
                .collect();
            natives.extend(wxr_gauges);
            panel.gauges.retain(|g| !options.excluded.iter().any(|e| g.url.to_ascii_lowercase().starts_with(&e.to_ascii_lowercase())));
            if panel.gauges.is_empty() {
                continue;
            }
            let engine = Engine::new(EngineOptions {
                budget: options.budget,
                memory_limit: options.memory_limit,
                root: options.html_ui.clone(),
                ..Default::default()
            })
            .map_err(|e| format!("{}: {e}", panel.name))?;
            setup(&engine).map_err(|e| format!("{}: {e}", panel.name))?;
            let mut host = NullHost;
            let (w, h) = panel.display;
            let screen = panel.texture.clone().unwrap_or_default();
            for (name, source) in BEFORE_DOM {
                engine.run_script(&mut host, name, source).map_err(|e| format!("{}: {e}", panel.name))?;
            }
            let dom: &[(&str, &str)] = if options.standin_dom { &[DOM_STANDIN] } else { &super::dom::SCRIPTS };
            for (name, source) in dom {
                engine.run_script(&mut host, name, source).map_err(|e| format!("{}: {e}", panel.name))?;
            }
            // The view's document: the DOM makes one per screen, painting to
            // the screen this view's texture names.
            let create = format!(
                "if (typeof globalThis.__createDocument === 'function') {{ globalThis.document = globalThis.__createDocument({}, {w}, {h}); }}",
                json_string(&screen)
            );
            engine.run_script(&mut host, "msfs/document", &create).map_err(|e| format!("{}: {e}", panel.name))?;
            for (name, source) in AFTER_DOM {
                engine.run_script(&mut host, name, source).map_err(|e| format!("{}: {e}", panel.name))?;
            }
            let stats = ViewStats {
                name: panel.name.clone(),
                gauges: panel.gauges.iter().map(|g| g.url.clone()).collect(),
                ..Default::default()
            };
            views.push(View { panel, natives, engine, loaded: false, failed: false, inbox: Vec::new(), inbox_stalls: 0, stats });
        }
        let store = DataStore::open(options.datastore).with_backend(options.settings);
        Ok(Self {
            views,
            shared: Shared {
                root: options.html_ui,
                outbox: Vec::new(),
                intercepts: HashMap::new(),
                pending: Vec::new(),
                next_call: 1,
                store,
                instruments: Vec::new(),
                patches: options.patches,
            },
            panel_xml: options.panel_xml,
        })
    }

    /// The views, by panel.cfg section name.
    /// Evaluate `source` in the view named `view` (for tests and
    /// diagnostics), returning its value as a string.
    #[allow(dead_code)]
    pub fn eval_in(&self, view: &str, source: &str) -> Result<String, String> {
        let v = self.views.iter().find(|v| v.panel.name == view).ok_or_else(|| format!("no view {view}"))?;
        v.engine.eval("diagnostics", source)
    }

    pub fn view_names(&self) -> Vec<String> {
        self.views.iter().map(|v| v.panel.name.clone()).collect()
    }

    pub fn stats(&self) -> Vec<ViewStats> {
        self.views.iter().map(|v| ViewStats { memory: v.engine.memory_used(), ..v.stats.clone() }).collect()
    }

    /// Whether every view has been loaded (or failed to).
    pub fn all_loaded(&self) -> bool {
        self.views.iter().all(|v| v.loaded || v.failed)
    }

    /// Instruments that have identified themselves: (view, Guid, identifier).
    pub fn instruments(&self) -> Vec<(String, String, String)> {
        self.shared.instruments.iter().map(|(v, g, i)| (self.views[*v].panel.name.clone(), g.clone(), i.clone())).collect()
    }

    /// An H: event from the cockpit (a click, an X-Plane command): every
    /// instrument's `onInteractionEvent` gets it, as in MSFS.
    pub fn h_event(&mut self, name: &str) {
        let name = name.strip_prefix("H:").unwrap_or(name);
        self.shared.outbox.push((To::All, event_item("OnInteractionEvent", &format!("[\"\",[{}]]", json_string(name)))));
    }

    /// A key event from outside the scripts (the plugin, X-Plane). Views
    /// intercepting it hear it; `true` if none keeps it from the simulator.
    pub fn key_event(&mut self, name: &str, values: [f64; 3]) -> bool {
        self.shared.intercept(name, values)
    }

    /// A Coherent event every view receives (a provider's `SendAirport`,
    /// say): `args_json` is the handlers' arguments as a JSON array.
    pub fn broadcast(&mut self, name: &str, args_json: &str) {
        self.shared.outbox.push((To::All, event_item(name, args_json)));
    }

    /// Mouse input on a screen, for the view drawing into it
    /// (`__screenEvent`, docs/display-stream.md). The DOM finds the gauge
    /// under the point.
    pub fn screen_event(&mut self, screen: &str, kind: &str, x: f64, y: f64, button: i32, delta: f64) {
        let key = screen.trim_start_matches('$');
        if let Some(view) = self.views.iter().position(|v| v.panel.texture.as_deref().is_some_and(|t| t.eq_ignore_ascii_case(key))) {
            let item = format!("[\"screen\",{},{},{},{},{},{}]", json_string(screen), json_string(kind), num(x), num(y), button, num(delta));
            self.shared.outbox.push((To::View(view), item));
        }
    }

    /// Load the next view not yet loaded (one per tick, so starting does
    /// not stall the simulator for all of them at once), then run every
    /// loaded view: calls still waiting on the host are asked again,
    /// events are delivered, and timers and animation frames run.
    pub fn tick(&mut self, host: &mut dyn Host, now_ms: f64) {
        self.retry_calls(host);
        if let Some(index) = self.views.iter().position(|v| !v.loaded && !v.failed) {
            self.load(host, index);
        }
        for index in 0..self.views.len() {
            if !self.views[index].loaded {
                continue;
            }
            self.route();
            let started = Instant::now();
            let view = &mut self.views[index];
            let mut vh = ViewHost { outer: &mut *host, shared: &mut self.shared, view: index };
            if !view.inbox.is_empty() {
                // The batch (cockpit screen clicks/drags among the items,
                // `Displays::dispatch_pointer`'s doc comment) is only
                // cleared once it is actually delivered. `Engine::invoke`'s
                // own watchdog (`js/mod.rs`'s `budgeted`) can interrupt a
                // view mid-batch on a tick where that view's scripts happen
                // to be slow (a busy dropdown re-layout, say) rather than
                // genuinely hung; clearing the inbox unconditionally before
                // the call (as this used to) permanently drops whatever was
                // in it, which — for a batch carrying a screen click — reads
                // exactly as "clicking the cockpit screen did nothing"
                // (debug_screen_clicks.md). Kept for one more tick's fresh
                // budget instead, up to `INBOX_RETRY_LIMIT` tries, so a
                // batch that is slow once still lands; a batch that keeps
                // timing out (a genuinely runaway script) is still dropped,
                // rather than growing forever as later frames' own events
                // keep appending to the same undelivered batch.
                let batch = format!("[{}]", view.inbox.join(","));
                match view.engine.invoke(&mut vh, "__msfsDeliver", &[&batch]) {
                    Ok(()) => {
                        view.inbox.clear();
                        view.inbox_stalls = 0;
                    }
                    Err(e) => {
                        vh.log(LogLevel::Error, &format!("{}: {e}", view.panel.name));
                        view.inbox_stalls += 1;
                        if !should_retry_delivery(&e, view.inbox_stalls) {
                            if view.inbox_stalls > INBOX_RETRY_LIMIT {
                                vh.log(
                                    LogLevel::Error,
                                    &format!("{}: dropping an undelivered batch after {INBOX_RETRY_LIMIT} interrupted retries", view.panel.name),
                                );
                            }
                            view.inbox.clear();
                            view.inbox_stalls = 0;
                        }
                    }
                }
            }
            if let Err(e) = view.engine.tick(&mut vh, now_ms) {
                vh.log(LogLevel::Error, &format!("{}: {e}", view.panel.name));
            }
            let ms = started.elapsed().as_secs_f64() * 1000.;
            let s = &mut view.stats;
            s.last_tick_ms = ms;
            s.max_tick_ms = s.max_tick_ms.max(ms);
            s.total_tick_ms += ms;
            s.ticks += 1;
        }
        self.route();
        self.shared.store.save_if_changed();
    }

    fn load(&mut self, host: &mut dyn Host, index: usize) {
        let started = Instant::now();
        let guid_base = self.views[..index].iter().map(|v| v.panel.gauges.len()).sum::<usize>();
        let view = &mut self.views[index];
        let p = &view.panel;
        let instruments: Vec<String> = p
            .gauges
            .iter()
            .enumerate()
            .map(|(i, g)| {
                format!(
                    "{{\"iGUId\":{},\"sUrl\":{},\"vPosAndSize\":{{\"x\":{},\"y\":{},\"z\":{},\"w\":{}}}}}",
                    guid_base + i + 1,
                    json_string(&g.url),
                    g.rect[0],
                    g.rect[1],
                    g.rect[2],
                    g.rect[3]
                )
            })
            .collect();
        // What the simulator sends a VCockpit page (VCockpit.js,
        // `ShowVCockpitPanel`): the section, its sizes, the page attributes
        // BaseInstrument reads (`quality`, `gamestate`), and panel.xml.
        let data = format!(
            "{{\"sName\":{},\"vLogicalSize\":{{\"x\":{},\"y\":{}}},\"vDisplaySize\":{{\"x\":{},\"y\":{}}},\"daAttributes\":[{{\"name\":\"quality\",\"value\":\"high\"}},{{\"name\":\"gamestate\",\"value\":\"ingame\"}}],\"daInstruments\":[{}],\"sConfigFile\":{}}}",
            json_string(&p.name),
            p.logical.0,
            p.logical.1,
            p.display.0,
            p.display.1,
            instruments.join(","),
            json_string(&self.panel_xml)
        );
        let natives: Vec<String> = view
            .natives
            .iter()
            .map(|n| format!("[{},{},{},{},{},{}]", json_string(&n.id), n.over, n.rect[0], n.rect[1], n.rect[2], n.rect[3]))
            .collect();
        let mut vh = ViewHost { outer: host, shared: &mut self.shared, view: index };
        // Loading runs gauge scripts of megabytes: the load budget applies.
        let r = view.engine.run_script(&mut vh, "msfs/load", &format!("__vcockpit.natives([{}]); __vcockpit.load({});", natives.join(","), data));
        view.loaded = true;
        if let Err(e) = r {
            vh.log(LogLevel::Error, &format!("{} did not load: {e}", view.panel.name));
            view.failed = true;
            view.loaded = false;
        }
        view.stats.load_ms = started.elapsed().as_secs_f64() * 1000.;
    }

    /// Move the simulator side's deliveries into the views' inboxes.
    fn route(&mut self) {
        for (to, item) in self.shared.outbox.drain(..) {
            for (i, view) in self.views.iter_mut().enumerate() {
                let hit = match to {
                    To::All => true,
                    To::Others(from) => i != from,
                    To::View(v) => i == v,
                };
                if hit && (view.loaded || matches!(to, To::View(_))) {
                    view.inbox.push(item.clone());
                }
            }
        }
    }

    fn retry_calls(&mut self, host: &mut dyn Host) {
        let pending = std::mem::take(&mut self.shared.pending);
        for call in pending {
            match host.call(&call.name, &call.args) {
                CallReply::Pending(_) => self.shared.pending.push(call),
                CallReply::Resolved(json) => {
                    let value = if json.trim().is_empty() { "null".to_string() } else { json };
                    self.shared.outbox.push((To::View(call.view), format!("[\"resolve\",{},true,{}]", call.id, value)));
                }
                CallReply::Rejected(message) => {
                    self.shared.outbox.push((To::View(call.view), format!("[\"resolve\",{},false,{}]", call.id, json_string(&message))));
                }
            }
        }
    }
}

fn event_item(name: &str, args_json: &str) -> String {
    format!("[\"event\",{},{}]", json_string(name), if args_json.trim().is_empty() { "[]" } else { args_json })
}

impl Shared {
    /// A key event passing the intercepts: every view intercepting it hears
    /// it as `keyIntercepted` (msfs-sdk KeyEventManager: key, value1,
    /// value0, value2); `true` if no view keeps it from the simulator.
    fn intercept(&mut self, name: &str, values: [f64; 3]) -> bool {
        let key = name.strip_prefix("K:").unwrap_or(name).trim().to_ascii_uppercase();
        let Some(views) = self.intercepts.get(&key) else { return true };
        let mut pass = true;
        let args = format!("[{},{},{},{}]", json_string(&key), num(values[1]), num(values[0]), num(values[2]));
        for &(view, block) in views {
            self.outbox.push((To::View(view), event_item("keyIntercepted", &args)));
            pass &= !block;
        }
        pass
    }
}

fn num(v: f64) -> String {
    if v.is_finite() {
        format!("{v}")
    } else {
        "0".into()
    }
}

/// A JSON string literal.
pub fn json_string(s: &str) -> String {
    serde_json::Value::String(s.to_string()).to_string()
}

/// The host one view's scripts see: the plugin's, with the simulator side
/// MSFS adds between views.
struct ViewHost<'a> {
    outer: &'a mut dyn Host,
    shared: &'a mut Shared,
    view: usize,
}

impl ViewHost<'_> {
    /// A key event a script sent, as written (`K:NAME`, or `K:2:NAME` with
    /// its argument count), past the other views' intercepts unless it
    /// bypasses them, then to the simulator.
    fn send_key(&mut self, name: &str, values: &[f64], bypass: bool) {
        let written = name.strip_prefix("K:").unwrap_or(name);
        // The key itself, without an argument count.
        let key = match written.split_once(':') {
            Some((count, rest)) if count.chars().all(|c| c.is_ascii_digit()) => rest,
            _ => written,
        };
        let v = |i: usize| values.get(i).copied().unwrap_or(0.);
        if bypass || self.shared.intercept(key, [v(0), v(1), v(2)]) {
            self.outer.send_event(&format!("K:{written}"), values);
        }
    }
}

impl Host for ViewHost<'_> {
    fn get_var(&mut self, name: &str, unit: &str) -> f64 {
        self.outer.get_var(name, unit)
    }

    fn set_var(&mut self, name: &str, unit: &str, value: f64) {
        if let Some(h) = name.strip_prefix("H:") {
            // Every instrument hears an H: event, the sender's own included.
            self.shared.outbox.push((To::All, event_item("OnInteractionEvent", &format!("[\"\",[{}]]", json_string(h)))));
            self.outer.send_event(name, &[value]);
        } else if name.starts_with("K:") {
            self.send_key(name, &[value], false);
        } else {
            self.outer.set_var(name, unit, value);
        }
    }

    fn log(&mut self, level: LogLevel, message: &str) {
        self.outer.log(level, message)
    }

    /// Registered ids are the view's engine's; the plugin sees them made
    /// unique across views.
    fn get_var_reg(&mut self, id: usize, name: &str, unit: &str) -> f64 {
        self.outer.get_var_reg((self.view << 24) | id, name, unit)
    }

    fn set_var_reg(&mut self, id: usize, name: &str, unit: &str, value: f64) {
        if name.starts_with("H:") || name.starts_with("K:") {
            self.set_var(name, unit, value)
        } else {
            self.outer.set_var_reg((self.view << 24) | id, name, unit, value)
        }
    }

    fn send_event(&mut self, name: &str, values: &[f64]) {
        if name.starts_with("K:") {
            self.send_key(name, values, false);
        } else {
            self.set_var(name, "number", values.first().copied().unwrap_or(0.));
        }
    }

    fn get_string(&mut self, name: &str) -> String {
        self.outer.get_string(name)
    }

    fn set_string(&mut self, name: &str, value: &str) {
        self.outer.set_string(name, value)
    }

    fn call(&mut self, name: &str, args_json: &str) -> CallReply {
        let args: Vec<serde_json::Value> = serde_json::from_str(args_json).unwrap_or_default();
        let n = |i: usize| args.get(i).and_then(|v| v.as_f64().or_else(|| v.as_bool().map(|b| b as i32 as f64))).unwrap_or(0.);
        let s = |i: usize| args.get(i).and_then(|v| v.as_str()).unwrap_or("").to_string();
        match name {
            // msfs-sdk KeyEventManager.triggerKey(key, bypass, value0, value1, value2).
            "TRIGGER_KEY_EVENT" => {
                let bypass = args.get(1).is_some_and(|v| v.as_bool().unwrap_or(v.as_f64().unwrap_or(0.) != 0.));
                self.send_key(&s(0), &[n(2), n(3), n(4)], bypass);
                CallReply::Resolved(String::new())
            }
            // KeyEventManager.interceptKey(key, passThrough): 0 passes the
            // event on, 1 keeps it.
            "INTERCEPT_KEY_EVENT" => {
                let key = s(0).trim().to_ascii_uppercase();
                let block = n(1) != 0.;
                let list = self.shared.intercepts.entry(key).or_default();
                match list.iter_mut().find(|(v, _)| *v == self.view) {
                    Some(entry) => entry.1 = block,
                    None => list.push((self.view, block)),
                }
                CallReply::Resolved(String::new())
            }
            _ => match self.outer.call(name, args_json) {
                CallReply::Pending(_) => {
                    let id = self.shared.next_call;
                    self.shared.next_call += 1;
                    self.shared.pending.push(PendingCall { id, view: self.view, name: name.to_string(), args: args_json.to_string() });
                    CallReply::Pending(id)
                }
                reply => reply,
            },
        }
    }

    fn trigger(&mut self, name: &str, args_json: &str) {
        let args: Vec<serde_json::Value> = serde_json::from_str(args_json).unwrap_or_default();
        let s = |i: usize| args.get(i).and_then(|v| v.as_str()).unwrap_or("").to_string();
        let view = self.view;
        match name {
            // JS_LISTENER_GENERICDATA SEND(key, json): the other views'
            // generic data listeners receive it.
            "GENERIC_DATA" => {
                let item = event_item(
                    "EVENT_FROM_VIEW_LISTENER",
                    &format!("[\"JS_LISTENER_GENERICDATA\",{},{}]", json_string(&s(0)), json_string(&s(1))),
                );
                self.shared.outbox.push((To::Others(view), item));
            }
            // A flow event raising an HTML event in every view's instruments
            // (ON_MOUSERECT_HTMLEVENT, ON_HTMLEVENT_TO_ALL_VIEWS).
            "HTML_EVENT" => {
                let list = args.get(0).cloned().unwrap_or(serde_json::Value::Array(Vec::new()));
                self.shared.outbox.push((To::All, event_item("OnInteractionEvent", &format!("[\"\",{list}]"))));
            }
            // ON_HTMLEVENT_TO_SPECIFIC_VIEW / MULTIPLE_VIEWS: the instruments
            // named, by their identifiers.
            "HTML_EVENT_TO" => {
                let targets: HashSet<String> = s(0).split(',').map(|t| t.trim().to_string()).collect();
                let list = args.get(1).cloned().unwrap_or(serde_json::Value::Array(Vec::new()));
                for (v, guid, id) in &self.shared.instruments {
                    if targets.contains(id) {
                        let item = event_item("OnInteractionEvent", &format!("[{},{list}]", json_string(guid)));
                        self.shared.outbox.push((To::View(*v), item));
                    }
                }
            }
            // TRIGGER_EVENT_TO_ALL_SUBSCRIBERS(listener, event, ...json).
            "TO_ALL_SUBSCRIBERS" => {
                let rest: Vec<String> = args.iter().skip(2).map(|a| a.to_string()).collect();
                let mut list = vec![json_string(&s(0)), "\"ON_EVENT_TO_ALL_SUBSCRIBERS\"".to_string(), json_string(&s(1))];
                list.extend(rest);
                self.shared.outbox.push((To::All, event_item("EVENT_FROM_VIEW_LISTENER", &format!("[{}]", list.join(",")))));
            }
            "INSTRUMENT_INITIALIZED" => {
                let (guid, id) = (s(0), s(1));
                self.shared.instruments.retain(|(v, g, _)| !(*v == view && *g == guid));
                self.shared.instruments.push((view, guid, id));
            }
            "ALL_INSTRUMENTS_LOADED" => {
                self.shared.outbox.push((To::View(view), event_item("OnAllInstrumentsLoaded", "[]")));
            }
            // The simulator connects a view listener and says so on a later
            // frame.
            "ADD_VIEW_LISTENER" => {
                self.shared.outbox.push((To::View(view), event_item("VIEW_LISTENER_REGISTERED", &format!("[{}]", json_string(&s(0))))));
            }
            // Anything else a script triggers only reaches the handlers in its
            // own view (Coherent.trigger emits there first), as in MSFS, and
            // the plugin, should it want it.
            other => self.outer.trigger(other, args_json),
        }
    }

    fn read_file(&mut self, path: &str) -> Result<String, String> {
        let clean = path.split(['?', '#']).next().unwrap_or("");
        let clean = clean.strip_prefix("coui://html_ui").unwrap_or(clean);
        // /VFS/ is the package's root, which holds html_ui.
        let (mut full, clean) = match clean.strip_prefix("/VFS/") {
            Some(rest) => (self.shared.root.parent().map(PathBuf::from).unwrap_or_default(), rest),
            None => (self.shared.root.clone(), clean),
        };
        for part in clean.split('/') {
            match part {
                "" | "." => {}
                ".." => return Err(format!("{path}: outside html_ui")),
                p => full.push(p),
            }
        }
        let mut text = std::fs::read_to_string(&full).map_err(|e| format!("{path} ({}): {e}", full.display()))?;
        for patch in self.shared.patches.iter().filter(|p| p.path.eq_ignore_ascii_case(clean)) {
            let found = text.matches(patch.find.as_str()).count();
            if found == 1 {
                text = text.replacen(patch.find.as_str(), &patch.replace, 1);
                self.outer.log(LogLevel::Info, &format!("MSFS runtime: {}: {}", patch.path, patch.reason));
            } else {
                self.outer.log(
                    LogLevel::Error,
                    &format!("MSFS runtime: {}: the patch ({}) matches {found} times, not once; the file runs unchanged", patch.path, patch.reason),
                );
            }
        }
        Ok(text)
    }

    fn stored_data(&mut self, op: &str, key: &str, value: &str) -> String {
        self.shared.store.op(op, key, value)
    }

    fn get_magvar(&mut self, lat: f64, lon: f64) -> f64 {
        self.outer.get_magvar(lat, lon)
    }
}

/// A host with nothing behind it, for setting a view up.
struct NullHost;

impl Host for NullHost {
    fn get_var(&mut self, _name: &str, _unit: &str) -> f64 {
        0.
    }
    fn set_var(&mut self, _name: &str, _unit: &str, _value: f64) {}
    fn log(&mut self, _level: LogLevel, _message: &str) {}
}

/// Stored-data keys kept somewhere else than the data store's own file (the
/// flyPad settings the plugin's EFB interface also reads and writes).
pub trait StoredDataBackend {
    fn owns(&self, key: &str) -> bool;
    /// The value, or `None` if none is stored.
    fn get(&mut self, key: &str) -> Option<String>;
    fn set(&mut self, key: &str, value: Option<&str>);
    /// Every key and value it holds, for searches.
    fn all(&mut self) -> Vec<(String, String)>;
}

/// MSFS's stored data (`GetStoredData` and friends), kept in a JSON file
/// of key to string, except for the keys a backend owns.
pub struct DataStore {
    path: Option<PathBuf>,
    data: std::collections::BTreeMap<String, String>,
    backend: Option<Box<dyn StoredDataBackend>>,
    changed: bool,
    saved_at: Option<Instant>,
}

impl DataStore {
    pub fn open(path: Option<PathBuf>) -> Self {
        let data = path
            .as_deref()
            .and_then(|p| std::fs::read_to_string(p).ok())
            .and_then(|t| serde_json::from_str::<std::collections::BTreeMap<String, String>>(&t).ok())
            .unwrap_or_default();
        Self { path, data, backend: None, changed: false, saved_at: None }
    }

    pub fn with_backend(mut self, backend: Option<Box<dyn StoredDataBackend>>) -> Self {
        self.backend = backend;
        self
    }

    /// `get` (the empty string if there is no such key, as in MSFS 2020),
    /// `set`, `delete`, or `search` (a JSON array of `{key, data}` for keys
    /// starting with `key`).
    pub fn op(&mut self, op: &str, key: &str, value: &str) -> String {
        if let Some(backend) = self.backend.as_mut().filter(|b| b.owns(key)) {
            match op {
                "get" => return backend.get(key).unwrap_or_default(),
                "set" => {
                    backend.set(key, Some(value));
                    return value.to_string();
                }
                "delete" => {
                    backend.set(key, None);
                    return String::new();
                }
                _ => {}
            }
        }
        match op {
            "get" => self.data.get(key).cloned().unwrap_or_default(),
            "search" => {
                let mut found: std::collections::BTreeMap<String, String> =
                    self.data.range(key.to_string()..).take_while(|(k, _)| k.starts_with(key)).map(|(k, v)| (k.clone(), v.clone())).collect();
                if let Some(backend) = self.backend.as_mut() {
                    found.extend(backend.all().into_iter().filter(|(k, _)| k.starts_with(key)));
                }
                let list: Vec<serde_json::Value> = found.into_iter().map(|(k, v)| serde_json::json!({ "key": k, "data": v })).collect();
                serde_json::Value::Array(list).to_string()
            }
            "set" => {
                if self.data.get(key).map(String::as_str) != Some(value) {
                    self.data.insert(key.to_string(), value.to_string());
                    self.changed = true;
                }
                value.to_string()
            }
            "delete" => {
                self.changed |= self.data.remove(key).is_some();
                String::new()
            }
            _ => String::new(),
        }
    }

    /// Write the file if something changed, at most once a second.
    pub fn save_if_changed(&mut self) {
        if !self.changed || self.saved_at.is_some_and(|t| t.elapsed() < Duration::from_secs(1)) {
            return;
        }
        self.save();
    }

    pub fn save(&mut self) {
        let Some(path) = self.path.as_deref() else { return };
        if let Some(dir) = path.parent() {
            let _ = std::fs::create_dir_all(dir);
        }
        if let Ok(text) = serde_json::to_string_pretty(&self.data) {
            if std::fs::write(path, text).is_ok() {
                self.changed = false;
            }
        }
        self.saved_at = Some(Instant::now());
    }
}

impl Drop for Cockpit {
    fn drop(&mut self) {
        if self.shared.store.changed {
            self.shared.store.save();
        }
    }
}

#[cfg(test)]
mod tests;
