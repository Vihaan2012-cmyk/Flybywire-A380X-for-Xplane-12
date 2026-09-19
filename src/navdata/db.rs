//! The navigation database built from X-Plane's files: every airport,
//! waypoint, VHF navaid and NDB, with lookups by MSFS ICAO, by ident prefix
//! and by position, and the airways through each fix.

use std::collections::HashMap;
use std::path::{Path, PathBuf};
use std::time::Instant;

use super::apt::{self, Airport};
use super::dat::{self, NavRecord};
use super::icao::{Code, Icao};

pub const EARTH_RADIUS_M: f64 = 6_371_000.;
pub const FT_TO_M: f64 = 0.3048;
pub const NM_TO_M: f64 = 1852.;

/// A facility in the database.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum FacRef {
    Airport(u32),
    Waypoint(u32),
    Vhf(u32),
    Ndb(u32),
}

#[derive(Clone, Debug)]
pub struct Waypoint {
    pub lat: f64,
    pub lon: f64,
    pub ident: Code,
    pub airport: Code,
    pub region: [u8; 2],
    pub wtype: [u8; 3],
}

#[derive(Clone, Debug)]
pub struct Dme {
    pub lat: f64,
    pub lon: f64,
    pub elev_ft: f32,
    pub bias_nm: f32,
}

#[derive(Clone, Debug)]
pub struct Glideslope {
    pub lat: f64,
    pub lon: f64,
    pub elev_ft: f32,
    pub angle: f64,
}

#[derive(Clone, Debug)]
pub struct Localizer {
    /// 4 ILS, 5 LOC/LDA/SDF.
    pub row: u8,
    pub runway: Code,
    pub true_bearing: f64,
    pub mag_course: Option<f64>,
    pub gs: Option<Glideslope>,
}

/// MSFS `VorType`.
pub mod vor_type {
    pub const VOR: u8 = 1;
    pub const VORDME: u8 = 2;
    pub const DME: u8 = 3;
    pub const TACAN: u8 = 4;
    pub const VORTAC: u8 = 5;
    pub const ILS: u8 = 6;
}

/// A VHF navaid: VOR, VOR/DME, VORTAC, TACAN, standalone DME, or a localizer.
#[derive(Clone, Debug)]
pub struct Vhf {
    pub lat: f64,
    pub lon: f64,
    pub elev_ft: f32,
    /// Hundredths of a MHz.
    pub freq: u32,
    /// Service volume class from earth_nav.dat (25, 40, 125, 130, 150; 18
    /// and so on for localizers).
    pub class: u16,
    /// East positive; the direction of the 0 radial in true degrees.
    pub slaved_var: f32,
    pub ident: Code,
    pub region: [u8; 2],
    /// Set for localizers (and nothing else).
    pub airport: Code,
    pub name: String,
    pub kind: u8,
    pub dme: Option<Dme>,
    pub loc: Option<Box<Localizer>>,
}

#[derive(Clone, Debug)]
pub struct Ndb {
    pub lat: f64,
    pub lon: f64,
    pub elev_ft: f32,
    pub freq_khz: u32,
    pub class: u16,
    pub bfo: bool,
    pub ident: Code,
    pub airport: Code,
    pub region: [u8; 2],
    pub name: String,
}

/// One airway through a fix.
#[derive(Clone, Debug)]
pub struct Route {
    pub name: u32,
    /// Bit 0 low, bit 1 high (MSFS `RouteType`: 1, 2 or 3).
    pub levels: u8,
    pub prev: Option<FacRef>,
    pub next: Option<FacRef>,
    pub prev_min_alt_ft: i32,
    pub next_min_alt_ft: i32,
}

/// Where each file comes from, after X-Plane's precedence rules.
#[derive(Clone, Debug, Default)]
pub struct Sources {
    pub nav: PathBuf,
    pub fix: PathBuf,
    pub awy: PathBuf,
    pub hold: PathBuf,
    pub apt_meta: PathBuf,
    pub user_nav: Option<PathBuf>,
    pub user_fix: Option<PathBuf>,
    /// Scenery packs' earth_nav.dat (Global Airports' holds Laminar's hand
    /// placed localizer corrections), lowest priority first.
    pub pack_navs: Vec<PathBuf>,
    /// Searched in order for `<ICAO>.dat`.
    pub cifp_dirs: Vec<PathBuf>,
    /// apt.dat files, highest priority first.
    pub apt_files: Vec<PathBuf>,
}

impl Sources {
    /// X-Plane's rules: a file in `Custom Data` replaces the one in
    /// `Resources/default data`; user_nav.dat and user_fix.dat in `Custom
    /// Data` add to the database; airports come from the scenery packs in
    /// scenery_packs.ini order, with Global Airports where the ini places
    /// it (last if it does not).
    pub fn find(root: &Path) -> Sources {
        let custom = root.join("Custom Data");
        let default = root.join("Resources").join("default data");
        let pick = |name: &str| {
            let c = custom.join(name);
            if c.is_file() {
                c
            } else {
                default.join(name)
            }
        };
        let optional = |name: &str| Some(custom.join(name)).filter(|p| p.is_file());
        let global = root.join("Global Scenery").join("Global Airports").join("Earth nav data").join("apt.dat");
        let global_nav = root.join("Global Scenery").join("Global Airports").join("Earth nav data").join("earth_nav.dat");
        let mut apt_files = Vec::new();
        let mut pack_navs = Vec::new();
        let mut global_listed = false;
        if let Ok(ini) = std::fs::read_to_string(root.join("Custom Scenery").join("scenery_packs.ini")) {
            for line in ini.lines() {
                let Some(pack) = line.strip_prefix("SCENERY_PACK ") else { continue };
                let pack = pack.trim();
                if pack == "*GLOBAL_AIRPORTS*" {
                    global_listed = true;
                    apt_files.push(global.clone());
                    pack_navs.push(global_nav.clone());
                    continue;
                }
                let dir = if Path::new(pack).is_absolute() { PathBuf::from(pack) } else { root.join(pack) };
                apt_files.push(dir.join("Earth nav data").join("apt.dat"));
                pack_navs.push(dir.join("Earth nav data").join("earth_nav.dat"));
            }
        }
        if !global_listed {
            apt_files.push(global);
            pack_navs.push(global_nav);
        }
        apt_files.retain(|p| p.is_file());
        pack_navs.retain(|p| p.is_file());
        pack_navs.reverse();
        Sources {
            nav: pick("earth_nav.dat"),
            fix: pick("earth_fix.dat"),
            awy: pick("earth_awy.dat"),
            hold: pick("earth_hold.dat"),
            apt_meta: pick("earth_aptmeta.dat"),
            user_nav: optional("user_nav.dat"),
            user_fix: optional("user_fix.dat"),
            pack_navs,
            cifp_dirs: vec![custom.join("CIFP"), default.join("CIFP")],
            apt_files,
        }
    }
}

/// How long each part of the load took, and how much it found.
#[derive(Clone, Debug, Default)]
pub struct LoadStats {
    pub navaids_ms: u128,
    pub fixes_ms: u128,
    pub airways_ms: u128,
    pub airports_ms: u128,
    pub index_ms: u128,
    pub total_ms: u128,
    pub airports: usize,
    pub waypoints: usize,
    pub vhf: usize,
    pub ndbs: usize,
    pub airway_segments: usize,
    pub holds: usize,
}

pub struct Db {
    pub sources: Sources,
    pub cycle: Option<u32>,
    pub airports: Vec<Airport>,
    pub waypoints: Vec<Waypoint>,
    pub vhf: Vec<Vhf>,
    pub ndbs: Vec<Ndb>,
    pub route_names: Vec<String>,
    pub routes: HashMap<FacRef, Vec<Route>>,
    /// Terminal holds by airport ident.
    pub holds: HashMap<Code, Vec<dat::HoldRecord>>,
    /// Localizers by airport.
    pub locs: HashMap<Code, Vec<u32>>,
    by_icao: HashMap<Icao, FacRef>,
    /// Every facility by ident, sorted.
    by_ident: Vec<(Code, FacRef)>,
    grid: HashMap<(i16, i16), Vec<FacRef>>,
    pub stats: LoadStats,
}

fn lines(path: &Path, mut f: impl FnMut(&str)) -> Result<Option<u32>, String> {
    let bytes = std::fs::read(path).map_err(|e| format!("{}: {e}", path.display()))?;
    let text = String::from_utf8_lossy(&bytes);
    let mut cycle = None;
    for (i, line) in text.lines().enumerate() {
        if i < 3 {
            cycle = cycle.or_else(|| dat::cycle(line));
        }
        f(line);
    }
    Ok(cycle)
}

/// Great circle distance in metres.
pub fn distance_m(lat1: f64, lon1: f64, lat2: f64, lon2: f64) -> f64 {
    let (p1, p2) = (lat1.to_radians(), lat2.to_radians());
    let dp = p2 - p1;
    let dl = (lon2 - lon1).to_radians();
    let a = (dp / 2.).sin().powi(2) + p1.cos() * p2.cos() * (dl / 2.).sin().powi(2);
    2. * EARTH_RADIUS_M * a.sqrt().min(1.).asin()
}

/// Initial true bearing from the first point to the second, 0-360.
pub fn bearing_deg(lat1: f64, lon1: f64, lat2: f64, lon2: f64) -> f64 {
    let (p1, p2) = (lat1.to_radians(), lat2.to_radians());
    let dl = (lon2 - lon1).to_radians();
    let y = dl.sin() * p2.cos();
    let x = p1.cos() * p2.sin() - p1.sin() * p2.cos() * dl.cos();
    y.atan2(x).to_degrees().rem_euclid(360.)
}

fn cell(lat: f64, lon: f64) -> (i16, i16) {
    (lat.floor().clamp(-90., 89.) as i16, ((lon.floor() as i32 + 180).rem_euclid(360) - 180) as i16)
}

impl Db {
    #[allow(dead_code)] // Convenience wrapper for tools; NavData::load computes Sources itself.
    pub fn load(root: &Path) -> Result<Db, String> {
        Self::load_from(Sources::find(root))
    }

    pub fn load_from(sources: Sources) -> Result<Db, String> {
        let started = Instant::now();
        let mut stats = LoadStats::default();
        let mut db = Db {
            sources,
            cycle: None,
            airports: Vec::new(),
            waypoints: Vec::new(),
            vhf: Vec::new(),
            ndbs: Vec::new(),
            route_names: Vec::new(),
            routes: HashMap::new(),
            holds: HashMap::new(),
            locs: HashMap::new(),
            by_icao: HashMap::new(),
            by_ident: Vec::new(),
            grid: HashMap::new(),
            stats: LoadStats::default(),
        };

        // Navaids, then fixes, from separate threads' worth of work but in
        // order here: airways need both.
        let t = Instant::now();
        // The global file, then scenery packs' and the user's, each replacing
        // the navaids it names again.
        let navs: Vec<PathBuf> = std::iter::once(db.sources.nav.clone()).chain(db.sources.pack_navs.clone()).chain(db.sources.user_nav.clone()).collect();
        for (i, path) in navs.iter().enumerate() {
            let mut records = Vec::new();
            let cycle = lines(path, |l| records.extend(dat::nav(l)))?;
            if i == 0 {
                db.cycle = cycle;
            }
            db.add_navaids(records, i > 0);
        }
        stats.navaids_ms = t.elapsed().as_millis();

        let t = Instant::now();
        let fixes: Vec<PathBuf> = std::iter::once(db.sources.fix.clone()).chain(db.sources.user_fix.clone()).collect();
        for (n, path) in fixes.iter().enumerate() {
            let (wps, by_icao) = (&mut db.waypoints, &mut db.by_icao);
            lines(path, |l| {
                let Some(r) = dat::fix(l) else { return };
                let key = Icao::new(b'W', r.region, r.airport, r.ident);
                let w = Waypoint { lat: r.lat, lon: r.lon, ident: r.ident, airport: r.airport, region: r.region, wtype: r.wtype };
                match by_icao.get(&key) {
                    // user_fix.dat replaces the waypoints it names again.
                    Some(&FacRef::Waypoint(i)) if n > 0 => wps[i as usize] = w,
                    // Otherwise the first of a duplicated ICAO is the one it
                    // finds; the others are found by position and ident.
                    Some(_) => wps.push(w),
                    None => {
                        by_icao.insert(key, FacRef::Waypoint(wps.len() as u32));
                        wps.push(w);
                    }
                }
            })?;
        }
        stats.fixes_ms = t.elapsed().as_millis();

        let t = Instant::now();
        if db.sources.awy.is_file() {
            let mut records = Vec::new();
            lines(&db.sources.awy.clone(), |l| records.extend(dat::awy(l)))?;
            stats.airway_segments = records.len();
            db.build_airways(records);
        }
        if db.sources.hold.is_file() {
            let holds = &mut db.holds;
            lines(&db.sources.hold.clone(), |l| {
                if let Some(h) = dat::hold(l) {
                    if !h.airport.is_empty() {
                        holds.entry(h.airport).or_default().push(h);
                    }
                }
            })?;
        }
        stats.airways_ms = t.elapsed().as_millis();

        let t = Instant::now();
        db.load_airports()?;
        stats.airports_ms = t.elapsed().as_millis();

        let t = Instant::now();
        db.build_indices();
        stats.index_ms = t.elapsed().as_millis();
        stats.total_ms = started.elapsed().as_millis();
        stats.airports = db.airports.len();
        stats.waypoints = db.waypoints.len();
        stats.vhf = db.vhf.len();
        stats.ndbs = db.ndbs.len();
        stats.holds = db.holds.values().map(Vec::len).sum();
        db.stats = stats;
        Ok(db)
    }

    /// Adds navaids; with `replace`, one whose ICAO is already present
    /// replaces that one.
    fn add_navaids(&mut self, records: Vec<NavRecord>, replace: bool) {
        for r in records {
            match r {
                NavRecord::Ndb { lat, lon, elev_ft, freq_khz, class, bfo, ident, airport, region, name } => {
                    let key = Icao::new(b'N', region, airport, ident);
                    let ndb = Ndb { lat, lon, elev_ft, freq_khz, class, bfo, ident, airport, region, name };
                    match self.by_icao.get(&key).copied() {
                        Some(FacRef::Ndb(i)) if replace => self.ndbs[i as usize] = ndb,
                        Some(_) => self.ndbs.push(ndb),
                        None => {
                            self.by_icao.insert(key, FacRef::Ndb(self.ndbs.len() as u32));
                            self.ndbs.push(ndb);
                        }
                    }
                }
                NavRecord::Vor { lat, lon, elev_ft, freq, class, slaved_var, ident, region, name } => {
                    let kind = if name.ends_with("VORTAC") {
                        vor_type::VORTAC
                    } else if name.ends_with("TACAN") {
                        vor_type::TACAN
                    } else if name.ends_with("VOR/DME") {
                        vor_type::VORDME
                    } else {
                        vor_type::VOR
                    };
                    // A TACAN is its own DME.
                    let dme = matches!(kind, vor_type::VORTAC | vor_type::TACAN).then_some(Dme { lat, lon, elev_ft, bias_nm: 0. });
                    self.push_vhf(Vhf { lat, lon, elev_ft, freq, class, slaved_var, ident, region, airport: Code::EMPTY, name, kind, dme, loc: None }, replace);
                }
                NavRecord::Loc { row, lat, lon, elev_ft, freq, true_bearing, mag_course, ident, airport, region, runway, name } => {
                    let loc = Localizer { row, runway, true_bearing, mag_course, gs: None };
                    self.push_vhf(Vhf {
                        lat,
                        lon,
                        elev_ft,
                        freq,
                        class: 0,
                        slaved_var: 0.,
                        ident,
                        region,
                        airport,
                        name,
                        kind: vor_type::ILS,
                        dme: None,
                        loc: Some(Box::new(loc)),
                    }, replace);
                }
                NavRecord::Gs { lat, lon, elev_ft, angle, ident, airport } => {
                    if let Some(FacRef::Vhf(i)) = self.by_icao.get(&Icao::new(b'V', [0; 2], airport, ident)).copied() {
                        if let Some(loc) = self.vhf[i as usize].loc.as_mut() {
                            loc.gs = Some(Glideslope { lat, lon, elev_ft, angle });
                        }
                    }
                }
                NavRecord::Dme { lat, lon, elev_ft, freq, class, bias_nm, ident, airport, region, name, .. } => {
                    let dme = Dme { lat, lon, elev_ft, bias_nm };
                    if !airport.is_empty() {
                        // An ILS DME serves every localizer of that ident.
                        let mut paired = false;
                        if let Some(FacRef::Vhf(i)) = self.by_icao.get(&Icao::new(b'V', [0; 2], airport, ident)).copied() {
                            self.vhf[i as usize].dme = Some(dme.clone());
                            paired = true;
                        }
                        if paired {
                            continue;
                        }
                    } else if let Some(FacRef::Vhf(i)) = self.by_icao.get(&Icao::new(b'V', region, Code::EMPTY, ident)).copied() {
                        let v = &mut self.vhf[i as usize];
                        if v.freq == freq && v.dme.is_none() {
                            if v.kind == vor_type::VOR {
                                v.kind = vor_type::VORDME;
                            }
                            v.dme = Some(dme);
                            continue;
                        }
                    }
                    self.push_vhf(Vhf {
                        lat,
                        lon,
                        elev_ft,
                        freq,
                        class,
                        slaved_var: 0.,
                        ident,
                        region,
                        airport: Code::EMPTY,
                        name,
                        kind: vor_type::DME,
                        dme: Some(dme),
                        loc: None,
                    }, replace);
                }
            }
        }
    }

    fn push_vhf(&mut self, v: Vhf, replace: bool) {
        // Localizer ICAOs carry the airport and no region.
        let key = if v.loc.is_some() { Icao::new(b'V', [0; 2], v.airport, v.ident) } else { Icao::new(b'V', v.region, Code::EMPTY, v.ident) };
        match self.by_icao.get(&key).copied() {
            Some(FacRef::Vhf(i)) if replace => self.vhf[i as usize] = v,
            Some(_) => self.vhf.push(v),
            None => {
                self.by_icao.insert(key, FacRef::Vhf(self.vhf.len() as u32));
                self.vhf.push(v);
            }
        }
    }

    fn enroute_ref(&self, ident: Code, region: [u8; 2], kind: u8) -> Option<FacRef> {
        let key = match kind {
            11 => Icao::new(b'W', region, Code::EMPTY, ident),
            2 => Icao::new(b'N', region, Code::EMPTY, ident),
            3 => Icao::new(b'V', region, Code::EMPTY, ident),
            _ => return None,
        };
        self.by_icao.get(&key).copied()
    }

    /// Orders each airway's segments into chains and records, at each fix,
    /// the fixes before and after it.
    fn build_airways(&mut self, records: Vec<dat::AwyRecord>) {
        struct Seg {
            a: FacRef,
            b: FacRef,
            direction: u8,
            level: u8,
            base_ft: i32,
        }
        let mut names: HashMap<String, u32> = HashMap::new();
        let mut by_name: Vec<Vec<Seg>> = Vec::new();
        for r in records {
            let (Some(a), Some(b)) = (self.enroute_ref(r.from.0, r.from.1, r.from.2), self.enroute_ref(r.to.0, r.to.1, r.to.2)) else { continue };
            for name in r.names {
                let id = *names.entry(name.clone()).or_insert_with(|| {
                    self.route_names.push(name);
                    by_name.push(Vec::new());
                    (self.route_names.len() - 1) as u32
                });
                by_name[id as usize].push(Seg { a, b, direction: r.direction, level: r.level, base_ft: r.base_ft });
            }
        }
        for (name, segs) in by_name.into_iter().enumerate() {
            // One entry per pair of fixes, whichever way round it was listed;
            // a segment listed for both levels gets both.
            let mut edges: HashMap<(FacRef, FacRef), (i32, u8, i32)> = HashMap::new();
            for s in &segs {
                let key = if s.a < s.b { (s.a, s.b) } else { (s.b, s.a) };
                let vote = match s.direction {
                    b'F' => 100,
                    b'B' => -100,
                    _ => 1,
                } * if s.a < s.b { 1 } else { -1 };
                let e = edges.entry(key).or_insert((0, 0, i32::MAX));
                e.0 += vote;
                e.1 |= 1 << (s.level.clamp(1, 2) - 1);
                e.2 = e.2.min(s.base_ft);
            }
            let mut adjacent: HashMap<FacRef, Vec<FacRef>> = HashMap::new();
            for &(a, b) in edges.keys() {
                adjacent.entry(a).or_default().push(b);
                adjacent.entry(b).or_default().push(a);
            }
            for list in adjacent.values_mut() {
                list.sort();
            }
            let mut used: std::collections::HashSet<(FacRef, FacRef)> = std::collections::HashSet::new();
            // Walk from chain ends first, then whatever is left (loops).
            let mut starts: Vec<FacRef> = adjacent.iter().filter(|(_, n)| n.len() == 1).map(|(k, _)| *k).collect();
            starts.sort();
            let mut rest: Vec<FacRef> = adjacent.keys().copied().collect();
            rest.sort();
            starts.extend(rest);
            for start in starts {
                loop {
                    let mut chain = vec![start];
                    let mut at = start;
                    while let Some(&next) = adjacent[&at].iter().find(|&&n| !used.contains(&if at < n { (at, n) } else { (n, at) })) {
                        used.insert(if at < next { (at, next) } else { (next, at) });
                        chain.push(next);
                        at = next;
                    }
                    if chain.len() < 2 {
                        break;
                    }
                    // Keep the orientation the file lists most (and any
                    // one-way restriction).
                    let votes: i32 = chain
                        .windows(2)
                        .map(|w| {
                            let (a, b) = (w[0], w[1]);
                            let key = if a < b { (a, b) } else { (b, a) };
                            edges[&key].0 * if a < b { 1 } else { -1 }
                        })
                        .sum();
                    if votes < 0 {
                        chain.reverse();
                    }
                    for w in chain.windows(2) {
                        let (a, b) = (w[0], w[1]);
                        let key = if a < b { (a, b) } else { (b, a) };
                        let (_, levels, base) = edges[&key];
                        self.add_route(a, name as u32, levels, None, Some((b, base)));
                        self.add_route(b, name as u32, levels, Some((a, base)), None);
                    }
                }
            }
        }
    }

    fn add_route(&mut self, at: FacRef, name: u32, levels: u8, prev: Option<(FacRef, i32)>, next: Option<(FacRef, i32)>) {
        let routes = self.routes.entry(at).or_default();
        // Fill the open side of an existing entry for this airway before
        // starting another (a fix in the middle of a chain gets one entry).
        if let Some(r) = routes.iter_mut().find(|r| r.name == name && ((prev.is_some() && r.prev.is_none()) || (next.is_some() && r.next.is_none()))) {
            r.levels |= levels;
            if let Some((p, alt)) = prev {
                r.prev = Some(p);
                r.prev_min_alt_ft = alt;
            }
            if let Some((n, alt)) = next {
                r.next = Some(n);
                r.next_min_alt_ft = alt;
            }
            return;
        }
        routes.push(Route {
            name,
            levels,
            prev: prev.map(|p| p.0),
            next: next.map(|n| n.0),
            prev_min_alt_ft: prev.map_or(0, |p| p.1),
            next_min_alt_ft: next.map_or(0, |n| n.1),
        });
    }

    fn load_airports(&mut self) -> Result<(), String> {
        let mut by_header: HashMap<Code, usize> = HashMap::new();
        let files = self.sources.apt_files.clone();
        for (i, path) in files.iter().enumerate() {
            let airports = &mut self.airports;
            apt::scan(path, i as u16, |a| match by_header.get(&a.header_id) {
                // A higher priority pack already defines this airport; its
                // layout wins, but it may lack metadata this one has.
                Some(&j) => {
                    let w = &mut airports[j];
                    if w.region == [0; 2] {
                        w.region = a.region;
                    }
                    if w.city.is_empty() {
                        w.city = a.city;
                    }
                    if w.iata.is_empty() {
                        w.iata = a.iata;
                    }
                    if w.ta_ft.is_none() {
                        w.ta_ft = a.ta_ft;
                    }
                    if w.tl_ft.is_none() {
                        w.tl_ft = a.tl_ft;
                    }
                    if w.freqs.is_empty() {
                        w.freqs = a.freqs;
                    }
                    if w.ident == w.header_id && a.ident != a.header_id {
                        w.ident = a.ident;
                    }
                }
                None => {
                    by_header.insert(a.header_id, airports.len());
                    airports.push(a);
                }
            })
            .map_err(|e| format!("{}: {e}", path.display()))?;
        }
        // Fill gaps from the navigation data's airport table.
        if self.sources.apt_meta.is_file() {
            let mut meta = HashMap::new();
            lines(&self.sources.apt_meta.clone(), |l| {
                if let Some(m) = dat::apt_meta(l) {
                    meta.insert(m.ident, m);
                }
            })?;
            for a in &mut self.airports {
                let Some(m) = meta.get(&a.ident) else { continue };
                if a.region == [0; 2] {
                    a.region = m.region;
                }
                if a.datum.is_none() {
                    a.datum = Some((m.lat, m.lon));
                    a.lat = m.lat;
                    a.lon = m.lon;
                }
                if a.ta_ft.is_none() && m.ta_ft > 0 {
                    a.ta_ft = Some(m.ta_ft);
                }
                if a.tl_ft.is_none() && m.tl_ft > 0 {
                    a.tl_ft = Some(m.tl_ft);
                }
            }
        }
        Ok(())
    }

    fn build_indices(&mut self) {
        for (i, a) in self.airports.iter().enumerate() {
            self.by_icao.entry(Icao::new(b'A', [0; 2], Code::EMPTY, a.ident)).or_insert(FacRef::Airport(i as u32));
        }
        for (i, v) in self.vhf.iter().enumerate() {
            if v.loc.is_some() {
                self.locs.entry(v.airport).or_default().push(i as u32);
            }
        }
        let mut by_ident = Vec::with_capacity(self.airports.len() + self.waypoints.len() + self.vhf.len() + self.ndbs.len());
        let mut grid: HashMap<(i16, i16), Vec<FacRef>> = HashMap::new();
        let mut add = |ident: Code, r: FacRef, lat: f64, lon: f64| {
            by_ident.push((ident, r));
            grid.entry(cell(lat, lon)).or_default().push(r);
        };
        for (i, a) in self.airports.iter().enumerate() {
            add(a.ident, FacRef::Airport(i as u32), a.lat, a.lon);
        }
        for (i, w) in self.waypoints.iter().enumerate() {
            add(w.ident, FacRef::Waypoint(i as u32), w.lat, w.lon);
        }
        for (i, v) in self.vhf.iter().enumerate() {
            add(v.ident, FacRef::Vhf(i as u32), v.lat, v.lon);
        }
        for (i, n) in self.ndbs.iter().enumerate() {
            add(n.ident, FacRef::Ndb(i as u32), n.lat, n.lon);
        }
        by_ident.sort_unstable();
        self.by_ident = by_ident;
        self.grid = grid;
    }

    /// The facility an ICAO names. Airports match whatever their region.
    pub fn find(&self, icao: &Icao) -> Option<FacRef> {
        let key = match icao.kind {
            b'A' => Icao::new(b'A', [0; 2], Code::EMPTY, icao.ident),
            _ => *icao,
        };
        self.by_icao.get(&key).copied()
    }

    /// Facilities whose ident starts with `prefix`, in ident order.
    pub fn with_ident_prefix(&self, prefix: &Code) -> impl Iterator<Item = (Code, FacRef)> + '_ {
        let prefix = *prefix;
        let start = self.by_ident.partition_point(|(c, _)| *c < prefix);
        self.by_ident[start..].iter().take_while(move |(c, _)| c.starts_with(&prefix)).copied()
    }

    pub fn position(&self, r: FacRef) -> (f64, f64) {
        match r {
            FacRef::Airport(i) => (self.airports[i as usize].lat, self.airports[i as usize].lon),
            FacRef::Waypoint(i) => (self.waypoints[i as usize].lat, self.waypoints[i as usize].lon),
            FacRef::Vhf(i) => (self.vhf[i as usize].lat, self.vhf[i as usize].lon),
            FacRef::Ndb(i) => (self.ndbs[i as usize].lat, self.ndbs[i as usize].lon),
        }
    }

    /// The ICAO MSFS would give a facility.
    pub fn icao(&self, r: FacRef) -> Icao {
        match r {
            FacRef::Airport(i) => {
                let a = &self.airports[i as usize];
                Icao::new(b'A', a.region, Code::EMPTY, a.ident)
            }
            FacRef::Waypoint(i) => {
                let w = &self.waypoints[i as usize];
                Icao::new(b'W', w.region, w.airport, w.ident)
            }
            FacRef::Vhf(i) => {
                let v = &self.vhf[i as usize];
                if v.loc.is_some() {
                    Icao::new(b'V', [0; 2], v.airport, v.ident)
                } else {
                    Icao::new(b'V', v.region, Code::EMPTY, v.ident)
                }
            }
            FacRef::Ndb(i) => {
                let n = &self.ndbs[i as usize];
                Icao::new(b'N', n.region, n.airport, n.ident)
            }
        }
    }

    /// Calls `f` with every facility within `radius_m` of a point, and its
    /// distance. Order is unspecified.
    pub fn within(&self, lat: f64, lon: f64, radius_m: f64, mut f: impl FnMut(FacRef, f64)) {
        if radius_m <= 0. {
            return;
        }
        let span = (radius_m / EARTH_RADIUS_M).to_degrees();
        let lat0 = (lat - span).floor().max(-90.) as i32;
        let lat1 = (lat + span).floor().min(89.) as i32;
        for row in lat0..=lat1 {
            let edge = (row as f64).abs().max((row as f64 + 1.).abs()).min(89.9);
            let lon_span = if lat1 == 89 || lat0 == -90 { 180. } else { span / edge.to_radians().cos() };
            let (c0, c1) = if lon_span >= 180. { (-180, 179) } else { ((lon - lon_span).floor() as i32, (lon + lon_span).floor() as i32) };
            for c in c0..=c1 {
                let col = ((c + 180).rem_euclid(360) - 180) as i16;
                let Some(list) = self.grid.get(&(row as i16, col)) else { continue };
                for &r in list {
                    let (flat, flon) = self.position(r);
                    let d = distance_m(lat, lon, flat, flon);
                    if d <= radius_m {
                        f(r, d);
                    }
                }
            }
        }
    }

    /// The CIFP file for an airport, if there is one.
    pub fn cifp_path(&self, ident: &str) -> Option<PathBuf> {
        self.sources.cifp_dirs.iter().map(|d| d.join(format!("{ident}.dat"))).find(|p| p.is_file())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn geometry() {
        // EGLL 09L to 27R threshold ends, from apt.dat: about 3.9 km, heading east.
        let d = distance_m(51.4774961, -0.4894104, 51.4776934, -0.4332280);
        assert!((d - 3_900.).abs() < 20., "{d}");
        let b = bearing_deg(51.4774961, -0.4894104, 51.4776934, -0.4332280);
        assert!((b - 89.7).abs() < 0.2, "{b}");
        assert_eq!(cell(51.5, -0.46), (51, -1));
        assert_eq!(cell(-33.9, 151.2), (-34, 151));
        assert_eq!(cell(10., 180.), (10, -180));
    }
}
