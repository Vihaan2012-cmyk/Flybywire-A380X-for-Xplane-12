//! FlyByWire's instruments on their own thread.
//!
//! The cockpit's views (`js::msfs::Cockpit`) are dozens of milliseconds of
//! script work a frame; run on X-Plane's thread, that is the frame rate. So
//! they run here, on a worker, and X-Plane's flight loop only trades with
//! them once a frame ([`Worker::exchange`]):
//!
//! - **Variables** are slots. The first time the scripts read a name in a
//!   unit, the worker gives it the next slot and asks for it; X-Plane's side
//!   resolves it as the host always did (`js_bridge`'s `VarsHost`) and from
//!   then on sends every slot's value each frame. A read of a slot not yet
//!   filled is 0, once, at the instrument's first read of that variable.
//! - **Writes** to a slot go back and are applied, converted, next frame;
//!   the worker's own copy has them straight away, so a script reads back
//!   what it wrote.
//! - **Events** (`K:`, `H:`), **log lines** and **Coherent calls** the
//!   plugin's providers answer go back the same way; a call is pending until
//!   its answer comes (the runtime asks again each tick, as it does for any
//!   pending call).
//! - **To the instruments** go the cockpit's H: events and the providers'
//!   events. Screen events and streams go straight between the worker and
//!   `display`, which is thread-safe.
//!
//! The worker runs one tick per frame X-Plane sends; if it falls behind it
//! takes the newest frame (events of skipped frames are kept). When X-Plane
//! is paused no frames come and the instruments wait.

use std::collections::{HashMap, HashSet};
use std::sync::{Arc, Condvar, Mutex};
use std::thread::JoinHandle;

use crate::js::msfs::Cockpit;
use crate::js::{CallReply, Host, LogLevel};

/// The script engine's stack: QuickJS recurses deeply in FlyByWire's bundles.
const STACK_BYTES: usize = 64 * 1024 * 1024;

/// What X-Plane's side sends the worker each frame.
#[derive(Default)]
pub struct Frame {
    pub time_ms: f64,
    /// Every slot's value, by slot.
    pub values: Vec<f64>,
    pub h_events: Vec<String>,
    pub provider_events: Vec<(String, String)>,
    /// Answers to calls, by name and arguments.
    pub replies: Vec<((String, String), CallReply)>,
    /// `GAME:` strings asked for, by name.
    pub game_strings: Vec<(String, String)>,
}

/// What the worker sends back.
#[derive(Default)]
pub struct Outbox {
    /// New slots, in slot order from `first_slot`: name and unit.
    pub first_slot: usize,
    pub slots: Vec<(String, String)>,
    /// Slot writes, in order.
    pub writes: Vec<(usize, f64)>,
    /// `K:`/`H:` writes and single-value events, in order.
    pub events: Vec<(String, f64)>,
    pub logs: Vec<String>,
    /// Calls for the plugin's providers, by name and arguments.
    pub calls: Vec<(String, String)>,
    pub game_strings: Vec<String>,
    /// Set once every view has loaded: what each cost.
    pub loaded: Option<Vec<String>>,
    /// Longest and total tick time since the last exchange, ms.
    pub tick_max_ms: f64,
    pub ticks: u32,
}

#[derive(Default)]
struct Exchange {
    frame: Option<Frame>,
    out: Outbox,
    stop: bool,
}

/// Starts the instruments' views: made on the worker, where they live.
pub type MakeCockpit = Box<dyn FnOnce() -> Result<Cockpit, String> + Send>;

pub struct Worker {
    shared: Arc<(Mutex<Exchange>, Condvar)>,
    thread: Option<JoinHandle<()>>,
}

impl Worker {
    pub fn start(make: MakeCockpit) -> Result<Self, String> {
        let shared = Arc::new((Mutex::new(Exchange::default()), Condvar::new()));
        let theirs = shared.clone();
        let thread = std::thread::Builder::new()
            .name("fbw-instruments".into())
            .stack_size(STACK_BYTES)
            .spawn(move || {
                let crashed = theirs.clone();
                if let Err(panic) = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| run(theirs, make))) {
                    let why = panic
                        .downcast_ref::<&str>()
                        .map(|s| s.to_string())
                        .or_else(|| panic.downcast_ref::<String>().cloned())
                        .unwrap_or_default();
                    if let Ok(mut ex) = crashed.0.lock() {
                        ex.out.logs.push(format!("js: error: the instruments' thread stopped: {why}"));
                    }
                }
            })
            .map_err(|e| format!("the instruments' thread did not start: {e}"))?;
        Ok(Self { shared, thread: Some(thread) })
    }

    /// Send this frame and take what the worker sent since the last one.
    /// `fill` gets the outbox first (so new slots can be resolved and writes
    /// applied), then returns the frame to send.
    pub fn exchange(&mut self, fill: impl FnOnce(Outbox) -> Frame) {
        let (lock, wake) = &*self.shared;
        let out = match lock.lock() {
            Ok(mut ex) => std::mem::take(&mut ex.out),
            Err(_) => return,
        };
        let frame = fill(out);
        if let Ok(mut ex) = lock.lock() {
            match ex.frame.as_mut() {
                // Not taken yet: newer values, and the events of both.
                Some(old) => {
                    old.time_ms = frame.time_ms;
                    old.values = frame.values;
                    old.h_events.extend(frame.h_events);
                    old.provider_events.extend(frame.provider_events);
                    old.replies.extend(frame.replies);
                    old.game_strings.extend(frame.game_strings);
                }
                None => ex.frame = Some(frame),
            }
            wake.notify_one();
        }
    }

    pub fn stop(&mut self) {
        let (lock, wake) = &*self.shared;
        if let Ok(mut ex) = lock.lock() {
            ex.stop = true;
            wake.notify_one();
        }
        if let Some(t) = self.thread.take() {
            let _ = t.join();
        }
    }
}

impl Drop for Worker {
    fn drop(&mut self) {
        self.stop();
    }
}

/// The instruments' host on the worker: slots, and what goes back.
struct WorkerHost {
    slots: HashMap<(String, String), usize>,
    reg: HashMap<usize, usize>,
    values: Vec<f64>,
    out: Outbox,
    strings: HashMap<String, String>,
    game_strings: HashMap<String, String>,
    asked_strings: HashSet<String>,
    replies: HashMap<(String, String), CallReply>,
    asked: HashSet<(String, String)>,
    logged: HashSet<String>,
}

impl WorkerHost {
    fn slot(&mut self, name: &str, unit: &str) -> usize {
        let key = (name.to_string(), unit.to_string());
        if let Some(&s) = self.slots.get(&key) {
            return s;
        }
        let s = self.slots.len();
        if self.out.slots.is_empty() {
            self.out.first_slot = s;
        }
        self.out.slots.push(key.clone());
        self.slots.insert(key, s);
        s
    }

    fn read(&self, slot: usize) -> f64 {
        self.values.get(slot).copied().unwrap_or(0.)
    }

    fn write(&mut self, slot: usize, value: f64) {
        if self.values.len() <= slot {
            self.values.resize(slot + 1, 0.);
        }
        self.values[slot] = value;
        self.out.writes.push((slot, value));
    }

    fn event(&mut self, name: &str, value: f64) {
        self.out.events.push((name.to_string(), value));
    }
}

impl Host for WorkerHost {
    fn get_var(&mut self, name: &str, unit: &str) -> f64 {
        let s = self.slot(name, unit);
        self.read(s)
    }

    fn set_var(&mut self, name: &str, unit: &str, value: f64) {
        if name.starts_with("K:") || name.starts_with("H:") {
            return self.event(name, value);
        }
        let s = self.slot(name, unit);
        self.write(s, value);
    }

    fn get_var_reg(&mut self, id: usize, name: &str, unit: &str) -> f64 {
        let s = match self.reg.get(&id) {
            Some(&s) => s,
            None => {
                let s = self.slot(name, unit);
                self.reg.insert(id, s);
                s
            }
        };
        self.read(s)
    }

    fn set_var_reg(&mut self, id: usize, name: &str, unit: &str, value: f64) {
        if name.starts_with("K:") || name.starts_with("H:") {
            return self.event(name, value);
        }
        let s = match self.reg.get(&id) {
            Some(&s) => s,
            None => {
                let s = self.slot(name, unit);
                self.reg.insert(id, s);
                s
            }
        };
        self.write(s, value);
    }

    /// `triggerKey`'s events, with all their values, go to key_events.rs
    /// (a queue behind a lock); the rest go back as events.
    fn send_event(&mut self, name: &str, values: &[f64]) {
        if name.starts_with("K:") && values.len() > 1 {
            crate::key_events::push(name, values);
        } else {
            self.event(name, values.first().copied().unwrap_or(0.));
        }
    }

    fn get_string(&mut self, name: &str) -> String {
        if let Some(game) = name.strip_prefix("GAME:") {
            if let Some(v) = self.game_strings.get(game) {
                return v.clone();
            }
            if self.asked_strings.insert(game.to_string()) {
                self.out.game_strings.push(game.to_string());
            }
            return String::new();
        }
        self.strings.get(name).cloned().unwrap_or_default()
    }

    fn set_string(&mut self, name: &str, value: &str) {
        self.strings.insert(name.to_string(), value.to_string());
    }

    fn log(&mut self, level: LogLevel, message: &str) {
        // An error thrown every frame is logged once.
        if level == LogLevel::Error && !self.logged.insert(message.to_string()) {
            return;
        }
        let tag = match level {
            LogLevel::Info => "",
            LogLevel::Warn => "warning: ",
            LogLevel::Error => "error: ",
        };
        self.out.logs.push(format!("js: {tag}{message}"));
    }

    /// SimBridge's address and map data answer here (both thread-safe); the
    /// plugin's providers answer on X-Plane's side, a frame or more later.
    fn call(&mut self, name: &str, args_json: &str) -> CallReply {
        if let Some(reply) = crate::js_bridge::direct_call(name, args_json) {
            return reply;
        }
        let key = (name.to_string(), args_json.to_string());
        if let Some(reply) = self.replies.remove(&key) {
            self.asked.remove(&key);
            return reply;
        }
        if self.asked.insert(key.clone()) {
            self.out.calls.push(key);
        }
        CallReply::Pending(0)
    }
}

fn run(shared: Arc<(Mutex<Exchange>, Condvar)>, make: MakeCockpit) {
    let (lock, wake) = &*shared;
    let mut cockpit = match make() {
        Ok(c) => c,
        Err(e) => {
            if let Ok(mut ex) = lock.lock() {
                ex.out.logs.push(format!("js: the instruments could not start: {e}"));
            }
            return;
        }
    };
    let mut host = WorkerHost {
        slots: HashMap::new(),
        reg: HashMap::new(),
        values: Vec::new(),
        out: Outbox::default(),
        strings: HashMap::new(),
        game_strings: HashMap::new(),
        asked_strings: HashSet::new(),
        replies: HashMap::new(),
        asked: HashSet::new(),
        logged: HashSet::new(),
    };
    host.out.logs.push(format!("js: FlyByWire's instruments: views {}", cockpit.view_names().join(", ")));
    let mut reported = false;
    loop {
        let frame = {
            let Ok(mut ex) = lock.lock() else { return };
            loop {
                if ex.stop {
                    return;
                }
                if let Some(f) = ex.frame.take() {
                    break f;
                }
                ex = match wake.wait(ex) {
                    Ok(g) => g,
                    Err(_) => return,
                };
            }
        };
        // This frame's values, keeping writes to slots X-Plane's side has
        // not resolved yet.
        let known = frame.values.len();
        if host.values.len() < known {
            host.values.resize(known, 0.);
        }
        host.values[..known].copy_from_slice(&frame.values);
        host.replies.extend(frame.replies);
        host.game_strings.extend(frame.game_strings);
        for (name, args) in &frame.provider_events {
            cockpit.broadcast(name, args);
        }
        for name in &frame.h_events {
            cockpit.h_event(name);
        }
        for e in crate::display::take_events() {
            cockpit.screen_event(e.screen, e.kind, e.x, e.y, e.button, e.delta);
        }
        let started = std::time::Instant::now();
        cockpit.tick(&mut host, frame.time_ms);
        crate::display::service();
        let ms = started.elapsed().as_secs_f64() * 1000.;
        host.out.tick_max_ms = host.out.tick_max_ms.max(ms);
        host.out.ticks += 1;
        if !reported && cockpit.all_loaded() {
            reported = true;
            let mut lines: Vec<String> = cockpit
                .stats()
                .iter()
                .map(|s| format!("js: {} {}: loaded in {:.0} ms, {:.1} MB", s.name, s.gauges.join(" "), s.load_ms, s.memory as f64 / 1e6))
                .collect();
            let ids: Vec<String> = cockpit.instruments().into_iter().map(|(_, _, id)| id).collect();
            lines.push(format!("js: instruments {}", ids.join(", ")));
            host.out.loaded = Some(lines);
        }
        let Ok(mut ex) = lock.lock() else { return };
        let out = std::mem::take(&mut host.out);
        merge(&mut ex.out, out);
    }
}

/// Add a tick's outbox to what X-Plane's side has not taken yet.
fn merge(into: &mut Outbox, out: Outbox) {
    if into.slots.is_empty() {
        into.first_slot = out.first_slot;
    }
    into.slots.extend(out.slots);
    into.writes.extend(out.writes);
    into.events.extend(out.events);
    into.logs.extend(out.logs);
    into.calls.extend(out.calls);
    into.game_strings.extend(out.game_strings);
    if out.loaded.is_some() {
        into.loaded = out.loaded;
    }
    into.tick_max_ms = into.tick_max_ms.max(out.tick_max_ms);
    into.ticks += out.ticks;
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::js::msfs::CockpitOptions;
    use std::path::PathBuf;
    use std::time::{Duration, Instant};

    /// FlyByWire's instruments on the worker while a stand-in for X-Plane's
    /// frame trades with it at 60 Hz: what that frame pays, and that the
    /// instruments keep up. `cargo test --release --features js --
    /// --ignored the_frame_only_trades`.
    #[test]
    #[ignore]
    fn the_frame_only_trades_with_the_instruments() {
        let html_ui = PathBuf::from(r"D:\fbw-aircraft\fbw-a380x\out\flybywire-aircraft-a380-842\html_ui");
        let panel = PathBuf::from(r"D:\Microsoft Flight Simulator 2020\Microsoft Flight Simulator 2020 Packages\Community\flybywire-aircraft-a380-842\SimObjects\AirPlanes\FlyByWire_A380_842\panel");
        let panel_cfg = std::fs::read_to_string(panel.join("panel.cfg")).unwrap();
        let panel_xml = std::fs::read_to_string(panel.join("panel.xml")).unwrap_or_default();
        let mut worker = Worker::start(Box::new(move || {
            let mut options = CockpitOptions::new(html_ui, panel_cfg);
            options.panel_xml = panel_xml;
            Cockpit::new(options, &|_| Ok(()))
        }))
        .unwrap();
        let mut store: HashMap<String, f64> = HashMap::new();
        for bus in ["AC_1", "AC_2", "AC_3", "AC_4", "AC_ESS", "DC_1", "DC_2", "DC_ESS", "DC_HOT_1", "DC_HOT_2"] {
            store.insert(format!("L:A32NX_ELEC_{bus}_BUS_IS_POWERED"), 1.);
        }
        let mut slots: Vec<String> = Vec::new();
        let (mut frame_max, mut frame_total, mut ticks, mut tick_max) = (0f64, 0f64, 0u32, 0f64);
        let seconds = 20.;
        let frames = (seconds * 60.) as usize;
        let mut writes = 0usize;
        for i in 0..frames {
            let started = Instant::now();
            worker.exchange(|out| {
                for line in &out.logs {
                    if line.contains("error") {
                        println!("{line}");
                    }
                }
                for (name, _) in out.slots {
                    slots.push(name);
                }
                writes += out.writes.len();
                for (slot, value) in out.writes {
                    store.insert(slots[slot].clone(), value);
                }
                ticks += out.ticks;
                tick_max = tick_max.max(out.tick_max_ms);
                let values = slots.iter().map(|n| *store.get(n).unwrap_or(&0.)).collect();
                Frame { time_ms: i as f64 * 1000. / 60., values, ..Default::default() }
            });
            let ms = started.elapsed().as_secs_f64() * 1000.;
            frame_max = frame_max.max(ms);
            frame_total += ms;
            std::thread::sleep(Duration::from_micros(16_667));
        }
        worker.stop();
        println!(
            "frames {frames}: trade mean {:.3} ms, max {:.2} ms; slots {}; writes {writes}; instrument ticks {ticks} (max {tick_max:.1} ms)",
            frame_total / frames as f64,
            frame_max,
            slots.len()
        );
        assert!(ticks > 0);
    }
}
