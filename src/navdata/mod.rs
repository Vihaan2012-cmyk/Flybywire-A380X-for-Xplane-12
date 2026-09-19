//! MSFS's facility database, answered from X-Plane's navigation data.
//!
//! FlyByWire's FMS and the msfs-sdk `FacilityLoader` read navigation data
//! through Coherent calls on MSFS's `JS_LISTENER_FACILITY`. [`NavData`]
//! answers those calls by name, with JSON arguments and results, from
//! X-Plane's earth_nav.dat, earth_fix.dat, earth_awy.dat, earth_hold.dat,
//! CIFP and apt.dat (with `Custom Data` and scenery pack precedence), in the
//! facility object shapes msfs-sdk 2.3 declares.
//!
//! As in MSFS, loading a facility takes two steps: a `LOAD_*` call answers
//! whether the ICAO names a facility, and the facility arrives later as an
//! event (`SendAirport`, `SendVor`, `SendNdb`, `SendIntersection`). Nearest
//! searches answer a search id, and the results arrive as
//! `NearestSearchCompleted` (or `NearestSearchCompletedWithStruct`).
//! [`NavData::take_events`] hands the events over, each with its handlers'
//! arguments as a JSON array; deliver them after the call has resolved (a
//! later tick), as MSFS does.
//!
//! The database is indexed on a background thread at start; until it is
//! ready every call fails with [`NOT_READY`], which a caller should treat as
//! "ask again next tick" rather than as an error. Airport facilities with
//! runways or procedures (a CIFP file read and parsed, and a few hundred
//! kilobytes of JSON) are built on a second background thread and cached;
//! an airport an ident search names exactly is built ahead, since it is
//! usually loaded next.

mod apt;
mod cifp;
mod dat;
mod db;
mod facility;
mod icao;
mod json;
mod nearest;
#[cfg(test)]
mod tests;

use std::collections::{HashMap, VecDeque};
use std::path::Path;
use std::sync::mpsc::{channel, Receiver, Sender};
use std::sync::Arc;
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

pub use db::LoadStats;
use db::{Db, FacRef, Sources};
use icao::{Code, Icao};
use json::Value;

/// The error every call returns while the database is still loading.
pub const NOT_READY: &str = "navdata: the database is still loading";

/// Every Coherent call name [`NavData::call`] answers.
pub const CALLS: &[&str] = &[
    "LOAD_AIRPORT",
    "LOAD_AIRPORTS",
    "LOAD_AIRPORT_FROM_STRUCT",
    "LOAD_VOR",
    "LOAD_VORS",
    "LOAD_VOR_FROM_STRUCT",
    "LOAD_NDB",
    "LOAD_NDBS",
    "LOAD_NDB_FROM_STRUCT",
    "LOAD_INTERSECTION",
    "LOAD_INTERSECTIONS",
    "LOAD_INTERSECTION_FROM_STRUCT",
    "SEARCH_BY_IDENT",
    "SEARCH_BY_IDENT_WITH_STRUCT",
    "START_NEAREST_SEARCH_SESSION",
    "START_NEAREST_SEARCH_SESSION_WITH_STRUCT",
    "SEARCH_NEAREST",
    "SET_NEAREST_AIRPORT_FILTER",
    "SET_NEAREST_EXTENDED_AIRPORT_FILTERS",
    "SET_NEAREST_INTERSECTION_FILTER",
    "SET_NEAREST_VOR_FILTER",
    "SET_NEAREST_BOUNDARY_FILTER",
    // The legacy GPS module's nearest airport list (C:fs9gps:NearestAirport*),
    // read with SimVar.GetSimVarArrayValues (app/js/msfs/simvar.js).
    "FS9GPS_NEAREST_AIRPORTS",
];

/// Built airport facilities kept.
const AIRPORT_CACHE: usize = 24;
/// Parsed CIFP files kept by the worker.
const CIFP_CACHE: usize = 16;

/// The airport data that reads CIFP: procedures, and runway threshold
/// elevations.
const CIFP_FLAGS: u32 = facility::flags::APPROACHES | facility::flags::DEPARTURES | facility::flags::ARRIVALS | facility::flags::RUNWAYS;

/// An airport facility to build on the worker.
struct Job {
    index: u32,
    flags: u32,
    magvar: f64,
}

struct Worker {
    jobs: Sender<Job>,
    done: Receiver<(u32, u32, Arc<str>)>,
    _thread: JoinHandle<()>,
}

pub struct NavData {
    loader: Option<JoinHandle<Result<Db, String>>>,
    db: Option<Arc<Db>>,
    /// The AIRAC cycle read straight from earth_nav.dat's header, before the
    /// full (multi-second) index finishes: `cycle()`/`date_range()` need
    /// this ready almost at once, since a `game_string` answered even once
    /// while not ready (js_bridge.rs's provider glue has no "ask again"
    /// status for these, unlike Coherent calls' `Pending`) is cached empty
    /// forever on the instruments' worker (js_worker.rs's `WorkerHost`), and
    /// MSFS's `FLIGHT NAVDATA DATE RANGE` was read within the first tick or
    /// two, well before the background parse of ~39k airports could answer.
    header_cycle: Option<u32>,
    error: Option<String>,
    worker: Option<Worker>,
    /// Built airports by (index, data flags), most recently used last.
    airports: VecDeque<((u32, u32), Arc<str>)>,
    /// Airports on the worker, and whether a script is waiting for each.
    queued: HashMap<(u32, u32), bool>,
    events: VecDeque<(String, String)>,
    sessions: HashMap<i32, nearest::Session>,
    next_session: i32,
    next_search: i32,
    approach_types: HashMap<u32, u64>,
    magvar: Option<Box<dyn Fn(f64, f64) -> f64 + Send>>,
    /// +1 or -1 once the source's sign is checked against the data.
    magvar_sign: Option<f64>,
}

impl NavData {
    /// Starts indexing X-Plane's navigation data under `xplane_root` on a
    /// background thread and returns at once.
    pub fn load(xplane_root: &Path) -> Result<Self, String> {
        let default_data = xplane_root.join("Resources").join("default data");
        if !default_data.is_dir() {
            return Err(format!("{} is not an X-Plane folder (no Resources/default data)", xplane_root.display()));
        }
        // Same source rules the full load uses (db::Sources::find), computed
        // here so the header can be read at once: cheap (a directory scan
        // and a few bytes), unlike the parse handed to the background
        // thread below.
        let sources = Sources::find(xplane_root);
        let header_cycle = header_cycle(&sources.nav);
        let loader = std::thread::Builder::new()
            .name("navdata-index".into())
            .spawn(move || Db::load_from(sources))
            .map_err(|e| e.to_string())?;
        Ok(NavData {
            loader: Some(loader),
            db: None,
            header_cycle,
            error: None,
            worker: None,
            airports: VecDeque::new(),
            queued: HashMap::new(),
            events: VecDeque::new(),
            sessions: HashMap::new(),
            next_session: 1,
            next_search: 1,
            approach_types: HashMap::new(),
            magvar: None,
            magvar_sign: None,
        })
    }

    /// Where magnetic variation comes from (degrees, east positive), for
    /// airports, NDBs, and localizers whose magnetic course the data does not
    /// give: the plugin passes XPLMGetMagneticVariation, X-Plane's own model,
    /// as MSFS fills these from its own. It is only called on the thread that
    /// makes calls. Without one those values are 0.
    ///
    /// The SDK does not say which way the variation it returns is signed, so
    /// the source is checked against the localizers whose magnetic course
    /// earth_nav.dat publishes (true bearing less magnetic course), where the
    /// variation is large enough for the sign to be unmistakable.
    pub fn set_magvar_source(&mut self, source: Box<dyn Fn(f64, f64) -> f64 + Send>) {
        self.magvar = Some(source);
        self.magvar_sign = None;
    }

    fn check_magvar_sign(&mut self, db: &Db) {
        let Some(source) = self.magvar.as_ref() else { return };
        if self.magvar_sign.is_some() {
            return;
        }
        let mut agree = 0i32;
        let published = db.vhf.iter().filter_map(|v| {
            let loc = v.loc.as_ref()?;
            let decl = facility::normalise180(loc.true_bearing - loc.mag_course?);
            (decl.abs() >= 5.).then_some((v.lat, v.lon, decl))
        });
        for (lat, lon, decl) in published.take(64) {
            let model = source(lat, lon);
            if model.abs() >= 1. {
                agree += if (model > 0.) == (decl > 0.) { 1 } else { -1 };
            }
        }
        self.magvar_sign = Some(if agree < 0 { -1. } else { 1. });
    }

    fn poll_loader(&mut self) {
        if self.loader.as_ref().is_some_and(|l| l.is_finished()) {
            self.finish_loading();
        }
    }

    fn finish_loading(&mut self) {
        if let Some(loader) = self.loader.take() {
            match loader.join() {
                Ok(Ok(db)) => self.db = Some(Arc::new(db)),
                Ok(Err(e)) => self.error = Some(e),
                Err(_) => self.error = Some("navdata: indexing panicked".into()),
            }
        }
    }

    /// Whether the database is loaded (checking without waiting).
    #[allow(dead_code)] // For the runtime and tools.
    pub fn is_ready(&mut self) -> bool {
        self.poll_loader();
        self.db.is_some()
    }

    /// Blocks until the database has loaded or failed.
    #[allow(dead_code)] // For the runtime and tools.
    pub fn wait_ready(&mut self) -> Result<(), String> {
        self.finish_loading();
        match (&self.db, &self.error) {
            (Some(_), _) => Ok(()),
            (None, Some(e)) => Err(e.clone()),
            (None, None) => Err("navdata: not loaded".into()),
        }
    }

    /// The load failure, if indexing failed.
    #[allow(dead_code)] // For the runtime and tools.
    pub fn load_error(&mut self) -> Option<&str> {
        self.poll_loader();
        self.error.as_deref()
    }

    #[allow(dead_code)] // For the runtime and tools.
    pub fn stats(&self) -> Option<&LoadStats> {
        self.db.as_ref().map(|d| &d.stats)
    }

    /// The AIRAC cycle of earth_nav.dat: from the full database once it is
    /// loaded, otherwise the header read eagerly at [`NavData::load`] (the
    /// two agree; `Db::load_from` reads the same file the same way).
    #[allow(dead_code)] // For the runtime and tools.
    pub fn cycle(&self) -> Option<u32> {
        self.db.as_ref().and_then(|db| db.cycle).or(self.header_cycle)
    }

    /// The navigation data's validity as MSFS's `FLIGHT NAVDATA DATE RANGE`
    /// game variable gives it (msfs-sdk `AiracUtils.parseFacilitiesCycle`
    /// reads it): effective and expiry dates and the expiry year,
    /// "MMMddMMMdd/yy".
    #[allow(dead_code)] // For the runtime and tools.
    pub fn date_range(&self) -> Option<String> {
        airac_date_range(self.cycle()?)
    }

    /// Whether `name` is one of the calls this answers.
    pub fn handles(name: &str) -> bool {
        CALLS.contains(&name)
    }

    fn magvar_at(&self, lat: f64, lon: f64) -> f64 {
        self.magvar.as_ref().map_or(0., |f| f(lat, lon) * self.magvar_sign.unwrap_or(1.))
    }

    /// Answers a Coherent call: `args_json` is the call's arguments as a
    /// JSON array, the result is the JSON value the call's promise resolves
    /// to.
    pub fn call(&mut self, name: &str, args_json: &str) -> Result<String, String> {
        if !Self::handles(name) {
            return Err(format!("navdata: {name} is not a facility call"));
        }
        self.poll_loader();
        if let Some(e) = &self.error {
            return Err(e.clone());
        }
        let Some(db) = self.db.clone() else { return Err(NOT_READY.into()) };
        self.check_magvar_sign(&db);
        let args = match json::parse(if args_json.trim().is_empty() { "[]" } else { args_json })? {
            Value::Arr(a) => a,
            other => vec![other],
        };
        let arg = |i: usize| args.get(i).unwrap_or(&Value::Null);
        let num = |i: usize| arg(i).as_f64().unwrap_or(0.);
        let icao = |i: usize| Icao::from_value(arg(i)).unwrap_or_default();
        let bool_json = |b: bool| if b { "true".to_string() } else { "false".to_string() };
        let kind = match name.split('_').nth(1) {
            Some("AIRPORT" | "AIRPORTS") => b'A',
            Some("VOR" | "VORS") => b'V',
            Some("NDB" | "NDBS") => b'N',
            _ => b'W',
        };
        Ok(match name {
            "LOAD_AIRPORT_FROM_STRUCT" => {
                let flags = if matches!(arg(1), Value::Null) { facility::flags::ALL } else { (num(1) as u32) & facility::flags::ALL };
                bool_json(self.load_facility(&db, &icao(0), kind, flags))
            }
            "LOAD_AIRPORT" | "LOAD_VOR" | "LOAD_VOR_FROM_STRUCT" | "LOAD_NDB" | "LOAD_NDB_FROM_STRUCT" | "LOAD_INTERSECTION" | "LOAD_INTERSECTION_FROM_STRUCT" => {
                bool_json(self.load_facility(&db, &icao(0), kind, facility::flags::ALL))
            }
            "LOAD_AIRPORTS" | "LOAD_VORS" | "LOAD_NDBS" | "LOAD_INTERSECTIONS" => {
                let icaos: Vec<Icao> = match arg(0) {
                    Value::Arr(items) => items.iter().filter_map(Icao::from_value).collect(),
                    _ => Vec::new(),
                };
                let found: Vec<bool> = icaos.iter().map(|i| self.load_facility(&db, i, kind, facility::flags::ALL)).collect();
                let mut out = String::new();
                json::array(&mut out, found, |out, b| out.push_str(if b { "true" } else { "false" }));
                out
            }
            "SEARCH_BY_IDENT" | "SEARCH_BY_IDENT_WITH_STRUCT" => {
                let ident = arg(0).as_str().unwrap_or("");
                let max = if num(2) > 0. { num(2) as usize } else { 40 };
                let found = search_by_ident(&db, ident, num(1) as i32, max);
                let wanted = ident.trim().to_ascii_uppercase();
                for r in &found {
                    if let FacRef::Airport(i) = *r {
                        if db.airports[i as usize].ident.as_str() == wanted {
                            self.build(&db, i, facility::flags::ALL, false);
                        }
                    }
                }
                let mut out = String::new();
                let structs = name.ends_with("STRUCT");
                json::array(&mut out, found, |out, r| {
                    let icao = db.icao(r);
                    if structs {
                        icao.write_struct(out);
                    } else {
                        json::string(out, &icao.v1());
                    }
                });
                out
            }
            "START_NEAREST_SEARCH_SESSION" | "START_NEAREST_SEARCH_SESSION_WITH_STRUCT" => {
                let id = self.next_session;
                self.next_session += 1;
                self.sessions.insert(id, nearest::Session::new(num(0) as i32, name.ends_with("STRUCT")));
                id.to_string()
            }
            "SEARCH_NEAREST" => {
                let id = num(0) as i32;
                let Some(mut session) = self.sessions.remove(&id) else { return Err(format!("navdata: no nearest search session {id}")) };
                let search_id = self.next_search;
                self.next_search += 1;
                let (lat, lon, radius, max) = (num(1), num(2), num(3), num(4).max(0.) as usize);
                let cache = &mut self.approach_types;
                let mut approach_types = |i: u32| *cache.entry(i).or_insert_with(|| airport_approach_types(&db, i));
                let (added, removed) = session.search(&db, lat, lon, radius, max, &mut approach_types);
                let event = if session.structs { "NearestSearchCompletedWithStruct" } else { "NearestSearchCompleted" };
                self.events.push_back((event.into(), format!("[{}]", session.results_json(&db, id, search_id, &added, &removed))));
                self.sessions.insert(id, session);
                search_id.to_string()
            }
            "SET_NEAREST_AIRPORT_FILTER" => {
                if let Some(s) = self.sessions.get_mut(&(num(0) as i32)) {
                    s.show_closed = num(1) != 0.;
                    s.airport_class_mask = nearest::Session::mask(num(2));
                }
                "null".into()
            }
            "SET_NEAREST_EXTENDED_AIRPORT_FILTERS" => {
                if let Some(s) = self.sessions.get_mut(&(num(0) as i32)) {
                    s.surface_mask = nearest::Session::mask(num(1));
                    s.approach_mask = nearest::Session::mask(num(2));
                    s.towered_mask = nearest::Session::mask(num(3));
                    s.min_runway_length_m = num(4);
                }
                "null".into()
            }
            "SET_NEAREST_INTERSECTION_FILTER" => {
                if let Some(s) = self.sessions.get_mut(&(num(0) as i32)) {
                    s.intersection_type_mask = nearest::Session::mask(num(1));
                    s.show_terminal = matches!(arg(2), Value::Null) || num(2) != 0.;
                }
                "null".into()
            }
            "SET_NEAREST_VOR_FILTER" => {
                if let Some(s) = self.sessions.get_mut(&(num(0) as i32)) {
                    s.vor_class_mask = nearest::Session::mask(num(1));
                    s.vor_type_mask = nearest::Session::mask(num(2));
                }
                "null".into()
            }
            // X-Plane's navigation data has no airspace boundaries for
            // boundary sessions to find; their searches come back empty.
            "SET_NEAREST_BOUNDARY_FILTER" => "null".into(),
            // [lat, lon, max items, max distance m] -> the nearest open land
            // airports, closest first: [[legacy ICAO, lat, lon, elevation ft,
            // distance m], ...].
            "FS9GPS_NEAREST_AIRPORTS" => {
                let (lat, lon, max, radius) = (num(0), num(1), num(2).max(0.) as usize, num(3));
                let mut found: Vec<(u32, f64)> = Vec::new();
                db.within(lat, lon, radius, |r, d| {
                    if let db::FacRef::Airport(i) = r {
                        let a = &db.airports[i as usize];
                        if a.kind == 1 && !a.closed {
                            found.push((i, d));
                        }
                    }
                });
                found.sort_by(|a, b| a.1.total_cmp(&b.1));
                found.truncate(max);
                let mut out = String::from("[");
                for (k, (i, d)) in found.iter().enumerate() {
                    let a = &db.airports[*i as usize];
                    if k > 0 {
                        out.push(',');
                    }
                    out.push('[');
                    json::string(&mut out, &format!("A      {}", a.ident.as_str().trim()));
                    for v in [a.lat, a.lon, a.elev_ft as f64, *d] {
                        out.push(',');
                        json::number(&mut out, v);
                    }
                    out.push(']');
                }
                out.push(']');
                out
            }
            _ => unreachable!("every name in CALLS is handled"),
        })
    }

    /// Queues the facility a `LOAD_*` call of `kind` asks for; returns
    /// whether the ICAO names one.
    fn load_facility(&mut self, db: &Arc<Db>, icao: &Icao, kind: u8, flags: u32) -> bool {
        let mut out = String::from("[");
        let event = match (kind, db.find(icao)) {
            (b'A', Some(FacRef::Airport(i))) if icao.kind == b'A' => {
                if flags & CIFP_FLAGS == 0 {
                    let a = &db.airports[i as usize];
                    out.push_str(&build_airport(db, i, flags, self.magvar_at(a.lat, a.lon), None));
                    "SendAirport"
                } else {
                    self.build(db, i, flags, true);
                    return true;
                }
            }
            (b'V', Some(FacRef::Vhf(i))) if icao.kind == b'V' => {
                let v = &db.vhf[i as usize];
                facility::vor(db, i, self.magvar_at(v.lat, v.lon), &mut out);
                "SendVor"
            }
            (b'N', Some(FacRef::Ndb(i))) if icao.kind == b'N' => {
                let n = &db.ndbs[i as usize];
                facility::ndb(db, i, self.magvar_at(n.lat, n.lon), &mut out);
                "SendNdb"
            }
            // VORs and NDBs have intersection records too (msfs-sdk
            // `FacilityLoader.onFacilityReceived`'s type mismatch case).
            (b'W', Some(r @ (FacRef::Waypoint(_) | FacRef::Ndb(_)))) => {
                facility::intersection(db, r, &mut out);
                "SendIntersection"
            }
            (b'W', Some(r @ FacRef::Vhf(i))) if db.vhf[i as usize].loc.is_none() => {
                facility::intersection(db, r, &mut out);
                "SendIntersection"
            }
            _ => return false,
        };
        out.push(']');
        self.events.push_back((event.into(), out));
        true
    }

    /// Sends an airport from the cache, or queues it on the worker; with
    /// `send`, its `SendAirport` event follows when it is built.
    fn build(&mut self, db: &Arc<Db>, index: u32, flags: u32, send: bool) {
        let key = (index, flags);
        if let Some(p) = self.airports.iter().position(|e| e.0 == key) {
            let entry = self.airports.remove(p).expect("found");
            if send {
                self.events.push_back(("SendAirport".into(), format!("[{}]", entry.1)));
            }
            self.airports.push_back(entry);
            return;
        }
        if let Some(waiting) = self.queued.get_mut(&key) {
            *waiting |= send;
            return;
        }
        let a = &db.airports[index as usize];
        let magvar = self.magvar_at(a.lat, a.lon);
        let worker = self.worker.get_or_insert_with(|| {
            let (jobs, job_rx) = channel::<Job>();
            let (done_tx, done) = channel();
            let db = db.clone();
            let thread = std::thread::Builder::new()
                .name("navdata-airports".into())
                .spawn(move || airport_worker(db, job_rx, done_tx))
                .expect("navdata: could not start the airport worker");
            Worker { jobs, done, _thread: thread }
        });
        if worker.jobs.send(Job { index, flags, magvar }).is_ok() {
            self.queued.insert(key, send);
        }
    }

    fn built(&mut self, index: u32, flags: u32, json: Arc<str>) {
        if self.queued.remove(&(index, flags)) == Some(true) {
            self.events.push_back(("SendAirport".into(), format!("[{json}]")));
        }
        self.airports.retain(|e| e.0 != (index, flags));
        self.airports.push_back(((index, flags), json));
        while self.airports.len() > AIRPORT_CACHE {
            self.airports.pop_front();
        }
    }

    /// Events raised since the last call: (Coherent event name, the
    /// handlers' arguments as a JSON array).
    pub fn take_events(&mut self) -> Vec<(String, String)> {
        let built: Vec<_> = self.worker.as_ref().map(|w| w.done.try_iter().collect()).unwrap_or_default();
        for (index, flags, json) in built {
            self.built(index, flags, json);
        }
        self.events.drain(..).collect()
    }

    /// Like `take_events`, but first waits (up to `timeout`) for airports
    /// still being built.
    #[allow(dead_code)] // For the runtime and tools.
    pub fn take_events_blocking(&mut self, timeout: Duration) -> Vec<(String, String)> {
        let deadline = Instant::now() + timeout;
        while !self.queued.is_empty() {
            let Some(worker) = self.worker.as_ref() else { break };
            match worker.done.recv_timeout(deadline.saturating_duration_since(Instant::now())) {
                Ok((index, flags, json)) => self.built(index, flags, json),
                Err(_) => break,
            }
        }
        self.take_events()
    }
}

fn cached_cifp(db: &Db, cache: &mut VecDeque<(u32, Arc<cifp::Cifp>)>, index: u32) -> Option<Arc<cifp::Cifp>> {
    if let Some(p) = cache.iter().position(|c| c.0 == index) {
        let entry = cache.remove(p).expect("found");
        let c = entry.1.clone();
        cache.push_front(entry);
        return Some(c);
    }
    let a = &db.airports[index as usize];
    let bytes = std::fs::read(db.cifp_path(a.ident.as_str())?).ok()?;
    let c = Arc::new(cifp::parse(&String::from_utf8_lossy(&bytes)));
    cache.push_front((index, c.clone()));
    cache.truncate(CIFP_CACHE);
    Some(c)
}

fn build_airport(db: &Db, index: u32, flags: u32, magvar: f64, parsed: Option<&cifp::Cifp>) -> String {
    let a = &db.airports[index as usize];
    let gates = if flags & facility::flags::GATES != 0 {
        db.sources.apt_files.get(a.file as usize).and_then(|p| apt::gates(p, a.offset).ok()).unwrap_or_default()
    } else {
        Vec::new()
    };
    let mut out = String::with_capacity(if flags & CIFP_FLAGS != 0 { 256 * 1024 } else { 4096 });
    facility::airport(db, &facility::AirportRequest { index, flags, magvar, cifp: parsed, gates: &gates }, &mut out);
    out
}

/// Builds airport facilities off the calling thread.
fn airport_worker(db: Arc<Db>, jobs: Receiver<Job>, done: Sender<(u32, u32, Arc<str>)>) {
    let mut cache = VecDeque::new();
    for job in jobs {
        let parsed = cached_cifp(&db, &mut cache, job.index);
        let json = build_airport(&db, job.index, job.flags, job.magvar, parsed.as_deref());
        if done.send((job.index, job.flags, json.into())).is_err() {
            return;
        }
    }
}

/// Bits of MSFS `ApproachType` for the approaches an airport's CIFP lists.
fn airport_approach_types(db: &Db, index: u32) -> u64 {
    let a = &db.airports[index as usize];
    let Some(text) = db.cifp_path(a.ident.as_str()).and_then(|p| std::fs::read(p).ok()) else { return 0 };
    let text = String::from_utf8_lossy(&text);
    text.lines()
        .filter_map(|l| l.strip_prefix("APPCH:"))
        .filter_map(|p| p.split(',').nth(1).and_then(|t| t.bytes().next()))
        .filter(|t| *t != b'A')
        .fold(0, |acc, t| acc | 1 << facility::approach_type(t))
}

/// `SEARCH_BY_IDENT`: facilities whose ident starts with `ident`, exact
/// matches first, filtered by `FacilitySearchType`.
fn search_by_ident(db: &Db, ident: &str, filter: i32, max: usize) -> Vec<FacRef> {
    let ident = ident.trim().to_ascii_uppercase();
    let Some(prefix) = Code::new(&ident) else { return Vec::new() };
    if ident.is_empty() {
        return Vec::new();
    }
    let wanted = |r: &FacRef| match (filter, r) {
        (0 | 8, _) => true,
        (1, FacRef::Airport(_)) | (2, FacRef::Waypoint(_)) | (3, FacRef::Vhf(_)) | (4, FacRef::Ndb(_)) => true,
        _ => false,
    };
    let (mut exact, mut partial): (Vec<(Code, FacRef)>, Vec<(Code, FacRef)>) = db.with_ident_prefix(&prefix).filter(|(_, r)| wanted(r)).partition(|(c, _)| *c == prefix);
    exact.sort_by_key(|(_, r)| *r);
    partial.sort();
    exact.into_iter().chain(partial).map(|(_, r)| r).take(max).collect()
}

/// Days since 1970-01-01 for a civil date.
fn days_from_civil(y: i64, m: i64, d: i64) -> i64 {
    let y = if m <= 2 { y - 1 } else { y };
    let era = y.div_euclid(400);
    let yoe = y - era * 400;
    let doy = (153 * (m + if m > 2 { -3 } else { 9 }) + 2) / 5 + d - 1;
    let doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    era * 146_097 + doe - 719_468
}

fn civil_from_days(z: i64) -> (i64, i64, i64) {
    let z = z + 719_468;
    let era = z.div_euclid(146_097);
    let doe = z - era * 146_097;
    let yoe = (doe - doe / 1460 + doe / 36524 - doe / 146_096) / 365;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let d = doy - (153 * mp + 2) / 5 + 1;
    let m = if mp < 10 { mp + 3 } else { mp - 9 };
    (yoe + era * 400 + if m <= 2 { 1 } else { 0 }, m, d)
}

/// The AIRAC cycle from a nav data file's header, reading only the first few
/// lines rather than the file `Db::load_from` parses in full.
fn header_cycle(path: &Path) -> Option<u32> {
    use std::io::BufRead;
    let file = std::fs::File::open(path).ok()?;
    let mut reader = std::io::BufReader::new(file);
    let mut line = String::new();
    for _ in 0..3 {
        line.clear();
        if reader.read_line(&mut line).ok()? == 0 {
            return None;
        }
        if let Some(cycle) = dat::cycle(&line) {
            return Some(cycle);
        }
    }
    None
}

/// "MMMddMMMdd/yy" for an AIRAC cycle "YYNN": cycles are 28 days, and
/// 2001 took effect on 2 January 2020.
fn airac_date_range(cycle: u32) -> Option<String> {
    let (yy, nn) = ((cycle / 100) as i64, (cycle % 100) as i64);
    if nn == 0 || nn > 14 {
        return None;
    }
    let epoch = days_from_civil(2020, 1, 2);
    let year = 2000 + yy;
    let jan1 = days_from_civil(year, 1, 1);
    let first = epoch + (jan1 - epoch).div_euclid(28) * 28 + if (jan1 - epoch).rem_euclid(28) == 0 { 0 } else { 28 };
    let effective = first + (nn - 1) * 28;
    let expires = effective + 27;
    const MONTHS: [&str; 12] = ["JAN", "FEB", "MAR", "APR", "MAY", "JUN", "JUL", "AUG", "SEP", "OCT", "NOV", "DEC"];
    let (_, m1, d1) = civil_from_days(effective);
    let (y2, m2, d2) = civil_from_days(expires);
    Some(format!("{}{:02}{}{:02}/{:02}", MONTHS[(m1 - 1) as usize], d1, MONTHS[(m2 - 1) as usize], d2, y2 % 100))
}
