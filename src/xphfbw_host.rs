//! The plugin's per-frame driver for XPHFBW's shared-memory bridge
//! (docs/briefs/xphfbw-js-bridge.md). One `XphfbwHost` owns the `Session`
//! the plugin creates for XPHFBW's session tag (the same tag
//! `remote::client::RemoteSystems` used for the systems block) and, each
//! frame:
//!
//! - **`pre_tick`** (called before the systems tick): drains the uplink once
//!   (rule 4, ordering), resolving any slot a view registered since the last
//!   frame the same way js_bridge.rs's QuickJS host does
//!   ([`js_bridge::VarsHost::resolve`]), and applies every `Uplink::Write`
//!   straight away (rule 3: writes apply before the systems tick reads the
//!   variables). Everything else the uplink carried (events, calls, string
//!   writes, GAME: string requests, logs, `Loaded` reports) is kept for
//!   `post_tick`.
//! - **`post_tick`** (called after the systems tick, alongside
//!   `js_bridge::JsHost::update`): publishes every slot's value under the
//!   seqlock (rule 1), answers the deferred uplink records (replies, `GAME:`
//!   strings, string broadcasts), delivers this frame's H: events and
//!   provider events to every view's downlink, retries pending calls, and
//!   updates `displays_active` (rule 7) once every screened view has
//!   reported `Loaded { ok: true }`.
//!
//! `take_events()` hands lib.rs's tick the same `(K:/H: name, value)` pairs
//! `js_bridge::JsHost::take_events` does, for the same radios/doors/
//! extra_backend/fuel/key_events handling.

use std::collections::HashMap;
use std::sync::atomic::Ordering;

use crate::js::{CallReply, Host};
use crate::js_bridge::{self, EnvRefs, HostState, Resolved, VarsHost};
use crate::remote::win;
use crate::xp::Xplm;
use crate::xphfbw_bridge::{Downlink, Session, Uplink};
use crate::xphfbw_bridge_views::{view_list, ViewDef};
use crate::Vars;

struct PendingCall {
    view: u32,
    id: u64,
    name: String,
    args_json: String,
}

/// Distinct view messages written to the log before it goes quiet.
const MAX_LOGGED: usize = 2000;
/// How long a GAME string request waits for its provider: about a minute.
const GAME_STRING_FRAMES: u32 = 3600;

pub struct XphfbwHost {
    /// Leaked to `'static` at `start` (Session's own API is all `&self`, so
    /// a shared reference is enough): agent D's src/display/mod.rs
    /// `set_bridge` needs one to read `displays_active` and the screens'
    /// `ScreenBlock`s for the same tag, straight from the SlotTable header
    /// this module publishes under the seqlock.
    session: &'static Session,
    env: EnvRefs,
    state: HostState,
    /// Every slot the plugin has resolved so far, by slot index (mirrors
    /// js_bridge.rs's `JsHost::slots`, one per XPHFBW variable instead of
    /// one per QuickJS worker slot).
    resolved: Vec<(String, Resolved)>,
    views: Vec<ViewDef>,
    /// Whether each view (by its `ViewDef::index`) has reported `Loaded`.
    loaded: Vec<Option<bool>>,
    /// Messages already written to X-Plane's log, per view: FlyByWire's
    /// instruments repeat some every frame, and writing Log.txt that often
    /// costs X-Plane most of its frame rate.
    logged: std::collections::HashSet<(u32, String)>,
    /// GAME strings a view asked for that no provider could answer yet (the
    /// nav database still indexing), with the frames left to keep asking.
    game_waiting: Vec<(u32, String, u32)>,
    displays_active: bool,
    /// Uplink records `pre_tick` drained but did not apply straight away
    /// (everything but `Write`), for `post_tick` to answer.
    deferred: Vec<Uplink>,
    calls: Vec<PendingCall>,
    /// String variables views have set (`SimVar.SetSimVarValue` with a
    /// string), broadcast to every view on a change.
    strings: HashMap<String, String>,
    /// `(view, guid, instrument id)`, as `INSTRUMENT_INITIALIZED` reports
    /// them (src/js/msfs/mod.rs `Shared.instruments`, the same tracking):
    /// `HTML_EVENT_TO`'s targeted delivery looks views up by instrument id.
    instruments: Vec<(u32, String, String)>,
    /// Key name (upper case, no `K:`) -> the views intercepting it and
    /// whether each keeps the event from the simulator
    /// (`INTERCEPT_KEY_EVENT`; src/js/msfs/mod.rs `Shared.intercepts`).
    intercepts: HashMap<String, Vec<(u32, bool)>>,
    /// K:/H: writes and single-value events from views, for lib.rs's tick to
    /// route through the same handlers js_bridge.rs's events do.
    events: Vec<(String, f64)>,
    /// XPHFBW.exe's process, if its pid is known, so a dead process is
    /// noticed without needing to own its `Child` (whatever launched it
    /// does).
    process: Option<win::Process>,
    gone: bool,
}

impl XphfbwHost {
    /// Creates the session's shared objects for `tag` (the same tag XPHFBW's
    /// own process was started with) and works out the view numbering from
    /// `panel_cfg` ([`view_list`]). `None` if the shared objects could not
    /// be created (Windows refused, or a session with this tag already
    /// exists). Attaches `tag`/the session to src/display/mod.rs
    /// (`display::set_bridge`), which reads `displays_active` from the
    /// SlotTable header this module publishes; `Drop` detaches it again.
    pub fn start(tag: &str, xplm: &Xplm, panel_cfg: &str, xphfbw_pid: Option<u32>) -> Option<Self> {
        // Session's whole API takes `&self` (interior mutability behind the
        // named mutexes/atomics its shared memory already needs across
        // processes), so a leaked, genuinely `'static` reference is sound
        // and needs no unsafe lifetime extension from a `static mut`.
        let session: &'static Session = Box::leak(Box::new(Session::create(tag)?));
        let tag: &'static str = Box::leak(tag.to_string().into_boxed_str());
        let views = view_list(panel_cfg);
        let loaded = vec![None; views.len()];
        crate::display::set_bridge(Some(tag), Some(session));
        Some(Self {
            session,
            env: EnvRefs::new(xplm),
            state: HostState::default(),
            resolved: Vec::new(),
            views,
            loaded,
            logged: std::collections::HashSet::new(),
            game_waiting: Vec::new(),
            displays_active: false,
            deferred: Vec::new(),
            calls: Vec::new(),
            strings: HashMap::new(),
            instruments: Vec::new(),
            intercepts: HashMap::new(),
            events: Vec::new(),
            process: xphfbw_pid.and_then(win::Process::open),
            gone: false,
        })
    }

    pub fn view_count(&self) -> usize {
        self.views.len()
    }

    /// Views that have reported `Loaded { ok: true }` so far, for
    /// `xphfbw/views_loaded` (src/xphfbw_datarefs.rs).
    pub fn views_loaded(&self) -> u32 {
        self.loaded.iter().filter(|l| **l == Some(true)).count() as u32
    }

    /// Whether XPHFBW's process has exited (its pid was known and its
    /// handle is now signalled). Once this is `true`, `displays_active`
    /// drops on the next `post_tick` (rule 7: "if XPHFBW goes away (process
    /// exits) the plugin sets it 0 and restarts its own engine") and stays
    /// so, so a caller can drop this host and go back to the QuickJS-only
    /// path.
    pub fn gone(&self) -> bool {
        self.gone
    }

    pub fn displays_active(&self) -> bool {
        self.displays_active
    }

    /// K:/H: writes and single-value events views sent, oldest first.
    pub fn take_events(&mut self) -> Vec<(String, f64)> {
        std::mem::take(&mut self.events)
    }

    /// Rule 3/4: resolve any slot registered since the last frame, drain the
    /// uplink exactly once, and apply every write immediately so the systems
    /// tick right after this sees this frame's cockpit input. Everything
    /// else the uplink carried is kept for `post_tick`.
    pub fn pre_tick(&mut self, vars: &mut Vars) {
        if self.check_gone() {
            return;
        }
        self.resolve_new_slots(vars);
        let records = self.session.uplink.drain();
        let mut host = VarsHost::new(vars, &mut self.state, &self.env, 0.);
        for bytes in records {
            let Some(u) = Uplink::decode(&bytes) else { continue };
            match u {
                Uplink::Write { slot, value } => {
                    if let Some((name, r)) = self.resolved.get(slot as usize) {
                        let (name, r) = (name.clone(), *r);
                        host.write(&name, r, value);
                    }
                }
                other => self.deferred.push(other),
            }
        }
    }

    /// Rule 1/6/7: publishes every slot's value under the seqlock, answers
    /// the uplink records `pre_tick` deferred, delivers this frame's H:
    /// events and provider events to every view, retries pending calls, and
    /// updates `displays_active`.
    pub fn post_tick(&mut self, vars: &mut Vars, time: f64, h_events: &[String], provider_events: &[(String, String)]) {
        if self.gone {
            return;
        }
        self.apply_deferred();
        self.publish(vars, time);
        for name in h_events {
            self.broadcast(Downlink::HEvent { name: name.clone() });
        }
        for (name, json) in provider_events {
            self.broadcast(Downlink::ProviderEvent { name: name.clone(), json: json.clone() });
        }
        self.retry_calls(vars, time);
        self.retry_game_strings();
        self.update_displays_active();
    }

    /// Answers the GAME strings that were not ready, once they are; after
    /// [`GAME_STRING_FRAMES`] an empty answer, as MSFS gives for an unknown one.
    fn retry_game_strings(&mut self) {
        let waiting = std::mem::take(&mut self.game_waiting);
        for (view, name, frames_left) in waiting {
            match js_bridge::provider_game_string(&name) {
                Some(value) => self.send(view, Downlink::GameString { name, value }),
                None if frames_left == 0 => self.send(view, Downlink::GameString { name, value: String::new() }),
                None => self.game_waiting.push((view, name, frames_left - 1)),
            }
        }
    }

    /// Any slot a view registered ([`crate::xphfbw_bridge::SlotTable::register`])
    /// since the last frame: resolved the same way js_bridge.rs's QuickJS
    /// host resolves any name, then marked resolved (rule 2: a slot at or
    /// past `resolved` reads 0).
    fn resolve_new_slots(&mut self, vars: &mut Vars) {
        let count = self.session.slots.header().count.load(Ordering::Relaxed) as usize;
        if count <= self.resolved.len() {
            return;
        }
        let mut host = VarsHost::new(vars, &mut self.state, &self.env, 0.);
        for i in self.resolved.len()..count {
            let (name, unit) = self.session.slots.entry(i);
            let r = host.resolve(&name, &unit);
            self.resolved.push((name, r));
        }
        self.session.slots.header().resolved.store(count as u32, Ordering::Relaxed);
    }

    /// Rule 1: `frame` odd while writing, every slot's value, `time_ms`,
    /// `frame` even again.
    fn publish(&mut self, vars: &mut Vars, time: f64) {
        let header = self.session.slots.header();
        header.frame.fetch_add(1, Ordering::Relaxed);
        header.time_ms.store((time * 1000.).to_bits(), Ordering::Relaxed);
        {
            let mut host = VarsHost::new(vars, &mut self.state, &self.env, time);
            let values = self.session.slots.values();
            for (i, (name, r)) in self.resolved.iter().enumerate() {
                values[i] = host.read(name, *r);
            }
        }
        header.frame.fetch_add(1, Ordering::Relaxed);
    }

    /// Everything `pre_tick`'s uplink drain kept besides writes: events go
    /// to `self.events` for lib.rs's tick, calls go to the pending list
    /// `post_tick` retries, everything else is answered straight away.
    fn apply_deferred(&mut self) {
        let deferred = std::mem::take(&mut self.deferred);
        let mut broadcasts: Vec<Downlink> = Vec::new();
        let mut sends: Vec<(u32, Downlink)> = Vec::new();
        for u in deferred {
            match u {
                Uplink::Write { .. } => unreachable!("applied in pre_tick"),
                Uplink::Event { name, values } => {
                    if name.starts_with("K:") && values.len() > 1 {
                        crate::key_events::push(&name, &values);
                    } else {
                        self.events.push((name, values.first().copied().unwrap_or(0.)));
                    }
                }
                Uplink::SetString { name, value } => {
                    self.strings.insert(name.clone(), value.clone());
                    broadcasts.push(Downlink::StringValue { name, value });
                }
                Uplink::Call { view, id, name, args_json } => {
                    // Calls that used to go straight through the in-process
                    // `Coherent.trigger` router (src/js/msfs/mod.rs
                    // `ViewHost::trigger`/`call`'s TRIGGER_KEY_EVENT/
                    // INTERCEPT_KEY_EVENT): F's runtime sends them as an
                    // ordinary call instead (there is no bridge "trigger"
                    // kind, and a call gets the promise resolution these
                    // need). Answered here, straight away, so they do not
                    // wait on `retry_calls`'s provider chain, which does not
                    // know them.
                    if !self.route_view_trigger(view, id, &name, &args_json) {
                        self.calls.push(PendingCall { view, id, name, args_json });
                    }
                }
                Uplink::GameString { view, name } => {
                    match js_bridge::provider_game_string(&name) {
                        Some(value) => sends.push((view, Downlink::GameString { name, value })),
                        None => self.game_waiting.push((view, name, GAME_STRING_FRAMES)),
                    }
                }
                Uplink::Log { view, level, text } => {
                    let tag = match level {
                        0 => "",
                        1 => "warning: ",
                        _ => "error: ",
                    };
                    if self.logged.len() < MAX_LOGGED && self.logged.insert((view, text.clone())) {
                        crate::log(&format!("js: xphfbw view {view}: {tag}{text}"));
                    }
                }
                Uplink::Loaded { view, ok, text } => {
                    if let Some(slot) = self.loaded.get_mut(view as usize) {
                        *slot = Some(ok);
                    }
                    if ok {
                        crate::log(&format!("js: xphfbw view {view} loaded: {text}"));
                    } else {
                        crate::log(&format!("js: xphfbw view {view} failed to load: {text}"));
                    }
                }
            }
        }
        for d in broadcasts {
            self.broadcast(d);
        }
        for (view, d) in sends {
            self.send(view, d);
        }
    }

    /// New and previously-pending calls ([`crate::js_bridge::direct_call`],
    /// then the providers'): a reply goes to the calling view's downlink
    /// only (rule 5); a call still pending is asked again next frame.
    fn retry_calls(&mut self, vars: &mut Vars, time: f64) {
        if self.calls.is_empty() {
            return;
        }
        let pending = std::mem::take(&mut self.calls);
        let mut host = VarsHost::new(vars, &mut self.state, &self.env, time);
        let mut replies: Vec<(u32, Downlink)> = Vec::new();
        for call in pending {
            match host.call(&call.name, &call.args_json) {
                CallReply::Pending(_) => self.calls.push(call),
                CallReply::Resolved(json) => {
                    let text = if json.trim().is_empty() { "null".to_string() } else { json };
                    replies.push((call.view, Downlink::Reply { id: call.id, ok: true, text }));
                }
                CallReply::Rejected(message) => {
                    replies.push((call.view, Downlink::Reply { id: call.id, ok: false, text: message }));
                }
            }
        }
        for (view, d) in replies {
            self.send(view, d);
        }
    }

    /// The view-to-view routing calls `Coherent.trigger`/some `.call`s used
    /// to reach in-process (src/js/msfs/mod.rs `ViewHost::trigger`/`call`,
    /// the reference this mirrors): `GENERIC_DATA`, `HTML_EVENT`,
    /// `HTML_EVENT_TO`, `TO_ALL_SUBSCRIBERS`/`TRIGGER_EVENT_TO_ALL_SUBSCRIBERS`
    /// fan out as `Downlink::ProviderEvent`s (poll() kind "provider", a
    /// Coherent event on the renderer); `INSTRUMENT_INITIALIZED` just tracks
    /// the caller; `TRIGGER_KEY_EVENT`/`INTERCEPT_KEY_EVENT` go through
    /// `intercept`/`crate::key_events`. `ADD_VIEW_LISTENER` and
    /// `ALL_INSTRUMENTS_LOADED` are answered locally in the page (agent F),
    /// so they never reach here. Sends the caller's reply and returns `true`
    /// when `name` is one of these; `false` (nothing sent yet) for a call
    /// the provider chain (`retry_calls`) should try instead.
    fn route_view_trigger(&mut self, view: u32, id: u64, name: &str, args_json: &str) -> bool {
        let args: Vec<serde_json::Value> = serde_json::from_str(args_json).unwrap_or_default();
        let s = |i: usize| args.get(i).and_then(|v| v.as_str()).unwrap_or("").to_string();
        match name {
            "GENERIC_DATA" => {
                let json = serde_json::json!(["JS_LISTENER_GENERICDATA", s(0), s(1)]).to_string();
                self.broadcast_except(view, Downlink::ProviderEvent { name: "EVENT_FROM_VIEW_LISTENER".into(), json });
            }
            "HTML_EVENT" => {
                let list = args.first().cloned().unwrap_or(serde_json::Value::Array(Vec::new()));
                let json = serde_json::json!(["", list]).to_string();
                self.broadcast(Downlink::ProviderEvent { name: "OnInteractionEvent".into(), json });
            }
            "HTML_EVENT_TO" => {
                let targets: std::collections::HashSet<String> = s(0).split(',').map(|t| t.trim().to_string()).collect();
                let list = args.get(1).cloned().unwrap_or(serde_json::Value::Array(Vec::new()));
                let matches: Vec<(u32, String)> =
                    self.instruments.iter().filter(|(_, _, iid)| targets.contains(iid)).map(|(v, guid, _)| (*v, guid.clone())).collect();
                for (v, guid) in matches {
                    let json = serde_json::json!([guid, list]).to_string();
                    self.send(v, Downlink::ProviderEvent { name: "OnInteractionEvent".into(), json });
                }
            }
            "TO_ALL_SUBSCRIBERS" | "TRIGGER_EVENT_TO_ALL_SUBSCRIBERS" => {
                let rest: Vec<serde_json::Value> = args.iter().skip(2).cloned().collect();
                let mut list = vec![serde_json::Value::String(s(0)), serde_json::Value::String("ON_EVENT_TO_ALL_SUBSCRIBERS".into()), serde_json::Value::String(s(1))];
                list.extend(rest);
                let json = serde_json::Value::Array(list).to_string();
                self.broadcast(Downlink::ProviderEvent { name: "EVENT_FROM_VIEW_LISTENER".into(), json });
            }
            "INSTRUMENT_INITIALIZED" => {
                let (guid, iid) = (s(0), s(1));
                self.instruments.retain(|(v, g, _)| !(*v == view && *g == guid));
                self.instruments.push((view, guid, iid));
            }
            "TRIGGER_KEY_EVENT" => {
                // msfs-sdk KeyEventManager.triggerKey(key, bypass, v0, v1, v2).
                let n = |i: usize| args.get(i).and_then(|v| v.as_f64().or_else(|| v.as_bool().map(|b| b as i32 as f64))).unwrap_or(0.);
                let bypass = args.get(1).is_some_and(|v| v.as_bool().unwrap_or(v.as_f64().unwrap_or(0.) != 0.));
                self.trigger_key(&s(0), [n(2), n(3), n(4)], bypass);
            }
            "INTERCEPT_KEY_EVENT" => {
                // KeyEventManager.interceptKey(key, passThrough): 0 passes
                // the event on, 1 keeps it.
                let key = s(0).trim().to_ascii_uppercase();
                let block = args.get(1).and_then(|v| v.as_f64()).unwrap_or(0.) != 0.;
                let list = self.intercepts.entry(key).or_default();
                match list.iter_mut().find(|(v, _)| *v == view) {
                    Some(entry) => entry.1 = block,
                    None => list.push((view, block)),
                }
            }
            _ => return false,
        }
        self.send(view, Downlink::Reply { id, ok: true, text: "null".to_string() });
        true
    }

    /// A key event a view's `triggerKey` sent, as `written` (`NAME`, or
    /// `n:NAME` with its argument count — msfs-sdk strips that before
    /// calling here, so `written` is already the bare key). Past the other
    /// views' intercepts unless it bypasses them, then to the simulator, the
    /// same as a cockpit click's `K:` write (js_bridge.rs's `VarsHost`,
    /// which queues it for `take_events`/`crate::key_events`).
    fn trigger_key(&mut self, written: &str, values: [f64; 3], bypass: bool) {
        let key = match written.split_once(':') {
            Some((count, rest)) if count.chars().all(|c| c.is_ascii_digit()) => rest,
            _ => written,
        }
        .trim()
        .to_ascii_uppercase();
        if bypass || self.intercept(&key, values) {
            crate::key_events::push(&format!("K:{written}"), &values);
        }
    }

    /// A key event past the other views' intercepts: every view
    /// intercepting it hears it as `keyIntercepted` (msfs-sdk
    /// KeyEventManager: key, value1, value0, value2, matching
    /// src/js/msfs/mod.rs `Shared::intercept`'s argument order); `true` if
    /// no view keeps it from the simulator.
    fn intercept(&mut self, key: &str, values: [f64; 3]) -> bool {
        let Some(views) = self.intercepts.get(key) else { return true };
        let views = views.clone();
        let json = serde_json::json!([key, values[1], values[0], values[2]]).to_string();
        let mut pass = true;
        for (view, block) in views {
            self.send(view, Downlink::ProviderEvent { name: "keyIntercepted".to_string(), json: json.clone() });
            pass &= !block;
        }
        pass
    }

    /// Rule 7: `displays_active` goes to 1 only once every screened view
    /// (`ViewDef::screen` not empty) has reported `Loaded { ok: true }`,
    /// and back to 0 if any of them later fails or XPHFBW's process exits.
    fn update_displays_active(&mut self) {
        let screened: Vec<usize> = self.views.iter().enumerate().filter(|(_, v)| !v.screen.is_empty()).map(|(i, _)| i).collect();
        let all_ok = !screened.is_empty() && screened.iter().all(|&i| self.loaded.get(i).copied().flatten() == Some(true));
        if all_ok != self.displays_active {
            self.displays_active = all_ok;
            self.session.slots.header().displays_active.store(all_ok as u32, Ordering::Relaxed);
        }
    }

    /// Whether XPHFBW's process (if its pid was known) has exited since the
    /// last check; latches `gone`/`displays_active` off once it has.
    fn check_gone(&mut self) -> bool {
        if self.gone {
            return true;
        }
        let Some(process) = self.process.as_ref() else { return false };
        if win::wait_any(&[process.handle()], 0) == Some(0) {
            self.gone = true;
            self.displays_active = false;
            self.session.slots.header().displays_active.store(0, Ordering::Relaxed);
            crate::log("js: xphfbw: the XPHFBW process is gone; the plugin's own instruments resume");
        }
        self.gone
    }

    fn send(&self, view: u32, downlink: Downlink) {
        if let Some(ring) = self.session.downlinks.get(view as usize) {
            ring.push(&downlink.encode());
        }
    }

    fn broadcast(&self, downlink: Downlink) {
        let bytes = downlink.encode();
        for ring in &self.session.downlinks {
            ring.push(&bytes);
        }
    }

    /// Rule 4's ordering aside, `GENERIC_DATA` is the one view-to-view
    /// broadcast the reference (`ViewHost::trigger`) sends `To::Others`
    /// rather than `To::All`: the caller's own generic data listener does
    /// not hear its own send.
    fn broadcast_except(&self, exclude: u32, downlink: Downlink) {
        let bytes = downlink.encode();
        for (i, ring) in self.session.downlinks.iter().enumerate() {
            if i as u32 != exclude {
                ring.push(&bytes);
            }
        }
    }
}

impl Drop for XphfbwHost {
    /// Detaches from src/display/mod.rs so it goes back to the QuickJS
    /// drawing path rather than reading a session this host no longer
    /// drives (the leaked `Session`/tag memory itself outlives this, which
    /// is fine: it is a fixed, small, one-time-per-session leak).
    fn drop(&mut self) {
        crate::display::set_bridge(None, None);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use systems::simulation::SimulatorReaderWriter;

    fn tag() -> String {
        format!("test_{}_{}", std::process::id(), std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).unwrap().subsec_nanos())
    }

    const PANEL_CFG: &str = r#"
[VCockpit01]
pixel_size=768,1024
texture=$SCREEN_DU_PFDL
htmlgauge00=A380X/PFD/pfd.html, 0,0,768,1024

[VCockpit02]
pixel_size=1,1
texture=NO_TEXTURE
htmlgauge00=A380X/SystemsHost/index.html, 0,0,1,1
"#;

    /// A fake uplink: a `Session::open`ed by the test, standing in for a
    /// view's renderer, driving a real `XphfbwHost` in-process.
    struct FakeView {
        session: Session,
        view: u32,
    }

    impl FakeView {
        fn open(tag: &str, view: u32) -> Self {
            Self { session: Session::open(tag).expect("the session opens"), view }
        }

        fn register(&self, name: &str, unit: &str) -> u32 {
            self.session.slots.register(name, unit).expect("a free slot")
        }

        fn write(&self, slot: u32, value: f64) {
            assert!(self.session.uplink.push(&Uplink::Write { slot, value }.encode()));
        }

        fn event(&self, name: &str, values: Vec<f64>) {
            assert!(self.session.uplink.push(&Uplink::Event { name: name.to_string(), values }.encode()));
        }

        fn call(&self, id: u64, name: &str, args_json: &str) {
            assert!(self.session.uplink.push(&Uplink::Call { view: self.view, id, name: name.to_string(), args_json: args_json.to_string() }.encode()));
        }

        fn loaded(&self, ok: bool) {
            assert!(self.session.uplink.push(&Uplink::Loaded { view: self.view, ok, text: String::new() }.encode()));
        }

        /// One seqlock-consistent snapshot of a slot's value (rule 1).
        fn read(&self, slot: u32) -> f64 {
            let header = self.session.slots.header();
            loop {
                let f1 = header.frame.load(Ordering::Relaxed);
                if f1 % 2 != 0 {
                    continue;
                }
                let value = self.session.slots.read(slot);
                let f2 = header.frame.load(Ordering::Relaxed);
                if f1 == f2 {
                    return value;
                }
            }
        }

        fn downlink(&self) -> Vec<Downlink> {
            self.session.downlinks[self.view as usize].drain().iter().filter_map(|b| Downlink::decode(b)).collect()
        }
    }

    /// X-Plane's bindings, stood in for: these tests do not run inside
    /// X-Plane, so there is no `XPLM_64.dll` to bind to (`Xplm::load`
    /// requires that). `Xplm::dummy` (xp.rs, `#[cfg(test)]`) always answers
    /// "no such dataref" from `find`, which is all `crate::Vars` (an `L:`
    /// name never has an X-Plane input mapping) and `js_bridge::EnvRefs`
    /// need to stay well-defined without one.
    fn xplm() -> &'static Xplm {
        static XPLM: std::sync::OnceLock<Xplm> = std::sync::OnceLock::new();
        XPLM.get_or_init(Xplm::dummy)
    }

    fn vars() -> crate::Vars {
        crate::Vars::new(xplm())
    }

    #[test]
    fn rule_1_slots_publish_under_the_seqlock_with_a_consistent_time() {
        let t = tag();
        let mut host = XphfbwHost::start(&t, xplm(), PANEL_CFG, None).unwrap();
        let view = FakeView::open(&t, 0);
        let slot = view.register("L:A32NX_TEST", "number");
        let mut vars = vars();
        let id = vars_id(&mut vars, "A32NX_TEST");
        vars.write(&id, 42.);
        host.pre_tick(&mut vars);
        host.post_tick(&mut vars, 1.5, &[], &[]);
        assert_eq!(view.read(slot), 42.);
        // Published under the lock: the header's frame is even (published),
        // never left odd (mid-write) after post_tick returns.
        assert_eq!(host.session.slots.header().frame.load(Ordering::Relaxed) % 2, 0);
    }

    #[test]
    fn rule_2_a_freshly_registered_slot_reads_0_before_the_plugin_resolves_it() {
        let t = tag();
        let _host = XphfbwHost::start(&t, xplm(), PANEL_CFG, None).unwrap();
        let view = FakeView::open(&t, 0);
        let slot = view.register("L:A32NX_FRESH", "number");
        // Nothing has resolved yet (no pre_tick/post_tick ran since
        // registering): the slot is past `resolved`.
        assert!(slot >= view.session.slots.header().resolved.load(Ordering::Relaxed));
        assert_eq!(view.read(slot), 0.);
    }

    #[test]
    fn rule_3_a_write_applies_before_the_systems_tick_reads_it() {
        let t = tag();
        let mut host = XphfbwHost::start(&t, xplm(), PANEL_CFG, None).unwrap();
        let view = FakeView::open(&t, 0);
        let mut vars = vars();
        let id = vars_id(&mut vars, "A32NX_KNOB");
        vars.write(&id, 0.);
        let slot = view.register("L:A32NX_KNOB", "number");
        view.write(slot, 7.);
        // pre_tick applies the write; a "systems tick" standing in as a
        // plain read right after must already see it.
        host.pre_tick(&mut vars);
        assert_eq!(vars.read(&id), 7.);
        host.post_tick(&mut vars, 0., &[], &[]);
        assert_eq!(view.read(slot), 7.);
    }

    #[test]
    fn rule_4_uplink_records_from_one_batch_apply_in_order() {
        let t = tag();
        let mut host = XphfbwHost::start(&t, xplm(), PANEL_CFG, None).unwrap();
        let view = FakeView::open(&t, 0);
        let mut vars = vars();
        let id = vars_id(&mut vars, "A32NX_SEQ");
        vars.write(&id, 0.);
        let slot = view.register("L:A32NX_SEQ", "number");
        // Several writes to the same slot in one batch: the last one queued
        // must win, as a FIFO drain applying records in order guarantees.
        for v in [1., 2., 3.] {
            view.write(slot, v);
        }
        host.pre_tick(&mut vars);
        assert_eq!(vars.read(&id), 3.);
    }

    #[test]
    fn rule_5_a_call_reply_goes_only_to_the_calling_views_downlink() {
        let t = tag();
        let mut host = XphfbwHost::start(&t, xplm(), PANEL_CFG, None).unwrap();
        let caller = FakeView::open(&t, 0);
        let other = FakeView::open(&t, 1);
        caller.call(99, "PLAY_INSTRUMENT_SOUND", "[\"test\"]");
        let mut vars = vars();
        host.pre_tick(&mut vars);
        host.post_tick(&mut vars, 0., &[], &[]);
        let replies = caller.downlink();
        assert_eq!(replies, vec![Downlink::Reply { id: 99, ok: true, text: "null".into() }]);
        assert!(other.downlink().is_empty());
    }

    /// Agent F: `GENERIC_DATA` (a Coherent view-listener send) used to reach
    /// every *other* view in-process (`ViewHost::trigger`'s `To::Others`);
    /// over the bridge it arrives as an `Uplink::Call` and must fan out the
    /// same way, replying to the caller without echoing it to itself.
    #[test]
    fn generic_data_reaches_every_other_view_but_not_the_sender() {
        let t = tag();
        let mut host = XphfbwHost::start(&t, xplm(), PANEL_CFG, None).unwrap();
        let sender = FakeView::open(&t, 0);
        let other = FakeView::open(&t, 1);
        sender.call(1, "GENERIC_DATA", "[\"KEY\",\"{\\\"a\\\":1}\"]");
        let mut vars = vars();
        host.pre_tick(&mut vars);
        host.post_tick(&mut vars, 0., &[], &[]);
        assert_eq!(sender.downlink(), vec![Downlink::Reply { id: 1, ok: true, text: "null".into() }]);
        let got = other.downlink();
        assert_eq!(got.len(), 1);
        assert!(matches!(&got[0], Downlink::ProviderEvent { name, .. } if name == "EVENT_FROM_VIEW_LISTENER"));
    }

    /// Agent F: `HTML_EVENT_TO` targets the views whose `INSTRUMENT_INITIALIZED`
    /// reported the given instrument id (src/js/msfs/mod.rs `Shared.instruments`).
    #[test]
    fn html_event_to_reaches_only_the_named_instruments_view() {
        let t = tag();
        let mut host = XphfbwHost::start(&t, xplm(), PANEL_CFG, None).unwrap();
        let pfd = FakeView::open(&t, 0);
        let systems_host = FakeView::open(&t, 1);
        pfd.call(1, "INSTRUMENT_INITIALIZED", "[\"guid-1\",\"A380X_PFD_1\"]");
        systems_host.call(2, "INSTRUMENT_INITIALIZED", "[\"guid-2\",\"A380X_SystemsHost\"]");
        let mut vars = vars();
        host.pre_tick(&mut vars);
        host.post_tick(&mut vars, 0., &[], &[]);
        pfd.downlink();
        systems_host.downlink();
        pfd.call(3, "HTML_EVENT_TO", "[\"A380X_PFD_1\",[{\"x\":1}]]");
        host.pre_tick(&mut vars);
        host.post_tick(&mut vars, 0., &[], &[]);
        assert!(systems_host.downlink().is_empty());
        let got = pfd.downlink();
        assert_eq!(got.len(), 2, "the reply and the targeted event");
        assert!(got.iter().any(|d| matches!(d, Downlink::ProviderEvent { name, .. } if name == "OnInteractionEvent")));
    }

    /// A view's `INTERCEPT_KEY_EVENT(key, block=true)` keeps a later
    /// `TRIGGER_KEY_EVENT` for that key from reaching the simulator, and the
    /// intercepting view hears it as `keyIntercepted`
    /// (src/js/msfs/mod.rs `Shared::intercept`).
    #[test]
    fn an_intercepted_key_event_reaches_the_intercepting_view_and_not_the_simulator() {
        let t = tag();
        let mut host = XphfbwHost::start(&t, xplm(), PANEL_CFG, None).unwrap();
        let mfd = FakeView::open(&t, 0);
        mfd.call(1, "INTERCEPT_KEY_EVENT", "[\"A32NX.FCU_AP_1_PUSH\",1]");
        mfd.call(2, "TRIGGER_KEY_EVENT", "[\"A32NX.FCU_AP_1_PUSH\",false,1,0,0]");
        let mut vars = vars();
        host.pre_tick(&mut vars);
        host.post_tick(&mut vars, 0., &[], &[]);
        let got = mfd.downlink();
        assert!(got.contains(&Downlink::Reply { id: 1, ok: true, text: "null".into() }));
        assert!(got.contains(&Downlink::Reply { id: 2, ok: true, text: "null".into() }));
        assert!(got.iter().any(|d| matches!(d, Downlink::ProviderEvent { name, .. } if name == "keyIntercepted")));
        // Blocked: nothing reached crate::key_events's queue for X-Plane.
        assert!(crate::key_events::KeyEvents::take_pushed().iter().all(|(n, _)| n != "A32NX.FCU_AP_1_PUSH"));
    }

    #[test]
    fn rule_7_displays_active_only_after_every_screened_view_has_loaded() {
        let t = tag();
        let mut host = XphfbwHost::start(&t, xplm(), PANEL_CFG, None).unwrap();
        assert_eq!(host.view_count(), 2);
        let pfd = FakeView::open(&t, 0);
        let systems_host = FakeView::open(&t, 1);
        let mut vars = vars();
        // Only the screenless host (view 1) has loaded: the screened PFD
        // (view 0) has not, so displays stay with the plugin's own engine.
        systems_host.loaded(true);
        host.pre_tick(&mut vars);
        host.post_tick(&mut vars, 0., &[], &[]);
        assert!(!host.displays_active());
        assert_eq!(host.session.slots.header().displays_active.load(Ordering::Relaxed), 0);
        // Now the PFD reports loaded too: every screened view has, so XPHFBW
        // takes over the displays.
        pfd.loaded(true);
        host.pre_tick(&mut vars);
        host.post_tick(&mut vars, 0., &[], &[]);
        assert!(host.displays_active());
        assert_eq!(host.session.slots.header().displays_active.load(Ordering::Relaxed), 1);
    }

    #[test]
    fn h_events_and_provider_events_reach_every_views_downlink() {
        let t = tag();
        let mut host = XphfbwHost::start(&t, xplm(), PANEL_CFG, None).unwrap();
        let a = FakeView::open(&t, 0);
        let b = FakeView::open(&t, 1);
        let mut vars = vars();
        host.pre_tick(&mut vars);
        host.post_tick(&mut vars, 0., &["A32NX_CHRONO_TOGGLE".to_string()], &[("FACILITY_LOADED".to_string(), "{}".to_string())]);
        for view in [&a, &b] {
            let got = view.downlink();
            assert!(got.contains(&Downlink::HEvent { name: "A32NX_CHRONO_TOGGLE".into() }));
            assert!(got.contains(&Downlink::ProviderEvent { name: "FACILITY_LOADED".into(), json: "{}".into() }));
        }
    }

    #[test]
    fn k_and_h_events_are_handed_to_take_events_for_lib_rs_to_route() {
        let t = tag();
        let mut host = XphfbwHost::start(&t, xplm(), PANEL_CFG, None).unwrap();
        let view = FakeView::open(&t, 0);
        view.event("H:A32NX_CHRONO_TOGGLE", vec![1.]);
        let mut vars = vars();
        host.pre_tick(&mut vars);
        host.post_tick(&mut vars, 0., &[], &[]);
        assert_eq!(host.take_events(), vec![("H:A32NX_CHRONO_TOGGLE".to_string(), 1.)]);
    }

    /// A helper making a `VariableIdentifier` the same way `VarsHost` does
    /// for an `L:` name, so a test can seed/read the variable a fake view's
    /// slot resolves to.
    fn vars_id(vars: &mut crate::Vars, named: &str) -> systems::simulation::VariableIdentifier {
        vars.add(named.to_string(), crate::NAMED)
    }
}
