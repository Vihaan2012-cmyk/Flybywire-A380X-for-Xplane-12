//! MSFS nearest search sessions: each session keeps its filters and the
//! results of its last search, and every search reports what was added and
//! removed since then (msfs-sdk `NearestSearchResults`).

use std::collections::HashSet;

use super::db::{vor_type, Db, FacRef};
use super::facility::{airport_class, intersection_type, runway_surface};
use super::icao::Icao;
use super::json::{array, Obj};

/// `FacilitySearchType`.
pub mod search_type {
    pub const ALL: i32 = 0;
    pub const AIRPORT: i32 = 1;
    pub const INTERSECTION: i32 = 2;
    pub const VOR: i32 = 3;
    pub const NDB: i32 = 4;
    pub const BOUNDARY: i32 = 5;
}

const ALL_BITS: u64 = u64::MAX;

#[derive(Clone, Debug)]
pub struct Session {
    pub search_type: i32,
    /// Whether results are `JS_ICAO` objects (the `_WITH_STRUCT` sessions)
    /// or V1 strings.
    pub structs: bool,
    pub current: Vec<FacRef>,
    pub show_closed: bool,
    pub airport_class_mask: u64,
    pub surface_mask: u64,
    pub approach_mask: u64,
    pub towered_mask: u64,
    pub min_runway_length_m: f64,
    pub intersection_type_mask: u64,
    pub show_terminal: bool,
    pub vor_class_mask: u64,
    pub vor_type_mask: u64,
}

impl Session {
    pub fn new(search_type: i32, structs: bool) -> Session {
        Session {
            search_type,
            structs,
            current: Vec::new(),
            show_closed: false,
            airport_class_mask: ALL_BITS,
            surface_mask: ALL_BITS,
            approach_mask: ALL_BITS,
            towered_mask: 3,
            min_runway_length_m: 0.,
            intersection_type_mask: ALL_BITS,
            show_terminal: true,
            vor_class_mask: ALL_BITS,
            vor_type_mask: ALL_BITS,
        }
    }

    /// A mask argument: scripts pass 2147483647 (or -1) to mean "no filter".
    pub fn mask(v: f64) -> u64 {
        let v = v as i64;
        if v < 0 || v >= 0x7fff_ffff {
            ALL_BITS
        } else {
            v as u64
        }
    }

    fn accepts(&self, db: &Db, r: FacRef, approach_types: &mut dyn FnMut(u32) -> u64) -> bool {
        let bit = |mask: u64, n: u32| n >= 64 || mask & (1 << n) != 0;
        match r {
            FacRef::Airport(i) => {
                let a = &db.airports[i as usize];
                if a.closed && !self.show_closed {
                    return false;
                }
                if !bit(self.airport_class_mask, airport_class(a)) {
                    return false;
                }
                if self.surface_mask != ALL_BITS && !a.runways.iter().any(|rw| bit(self.surface_mask, runway_surface(rw.surface))) {
                    return false;
                }
                if self.approach_mask != ALL_BITS && approach_types(i) & self.approach_mask == 0 {
                    return false;
                }
                if !bit(self.towered_mask, if a.towered() { 1 } else { 0 }) {
                    return false;
                }
                if self.min_runway_length_m > 0. {
                    let longest = a.runways.iter().map(|rw| super::db::distance_m(rw.ends[0].lat, rw.ends[0].lon, rw.ends[1].lat, rw.ends[1].lon)).fold(0., f64::max);
                    if longest < self.min_runway_length_m {
                        return false;
                    }
                }
                true
            }
            FacRef::Waypoint(i) => {
                let w = &db.waypoints[i as usize];
                (self.show_terminal || w.airport.is_empty()) && bit(self.intersection_type_mask, intersection_type(w))
            }
            FacRef::Vhf(i) => {
                let v = &db.vhf[i as usize];
                let class = if v.loc.is_some() {
                    4
                } else {
                    match v.class {
                        25 => 1,
                        40 => 2,
                        125 | 130 | 150 => 3,
                        _ => 0,
                    }
                };
                bit(self.vor_class_mask, class) && bit(self.vor_type_mask, v.kind as u32)
            }
            FacRef::Ndb(_) => true,
        }
    }

    fn wants(&self, r: FacRef) -> bool {
        use search_type::*;
        match (self.search_type, r) {
            (ALL, _) => true,
            (AIRPORT, FacRef::Airport(_)) | (VOR, FacRef::Vhf(_)) | (NDB, FacRef::Ndb(_)) | (INTERSECTION, FacRef::Waypoint(_)) => true,
            // Navaids are intersections too, of types VOR (3) and NDB (4).
            (INTERSECTION, FacRef::Vhf(_)) => self.intersection_type_mask != ALL_BITS && self.intersection_type_mask & (1 << 3) != 0,
            (INTERSECTION, FacRef::Ndb(_)) => self.intersection_type_mask != ALL_BITS && self.intersection_type_mask & (1 << 4) != 0,
            _ => false,
        }
    }

    /// Runs a search and returns (added, removed) relative to the last one.
    pub fn search(&mut self, db: &Db, lat: f64, lon: f64, radius_m: f64, max_items: usize, approach_types: &mut dyn FnMut(u32) -> u64) -> (Vec<FacRef>, Vec<FacRef>) {
        let mut found: Vec<(f64, FacRef)> = Vec::new();
        if max_items > 0 && self.search_type != search_type::BOUNDARY && radius_m > 0. {
            // Widen the search until it holds enough, so that a large radius
            // over dense data costs no more than it has to.
            let mut reach = radius_m.min(30_000.);
            loop {
                found.clear();
                db.within(lat, lon, reach, |r, d| {
                    if self.wants(r) {
                        if let FacRef::Vhf(i) = r {
                            // Localizers are not intersections.
                            if self.search_type == search_type::INTERSECTION && db.vhf[i as usize].kind == vor_type::ILS {
                                return;
                            }
                        }
                        found.push((d, r));
                    }
                });
                found.sort_by(|a, b| a.0.total_cmp(&b.0).then(a.1.cmp(&b.1)));
                let mut kept = Vec::with_capacity(max_items.min(found.len()));
                for &(d, r) in &found {
                    if kept.len() == max_items {
                        break;
                    }
                    if self.accepts(db, r, approach_types) {
                        kept.push((d, r));
                    }
                }
                if kept.len() == max_items || reach >= radius_m {
                    found = kept;
                    break;
                }
                reach = (reach * 3.).min(radius_m);
            }
        }
        let new: Vec<FacRef> = found.into_iter().map(|f| f.1).collect();
        let new_set: HashSet<FacRef> = new.iter().copied().collect();
        let old_set: HashSet<FacRef> = self.current.iter().copied().collect();
        let added = new.iter().copied().filter(|r| !old_set.contains(r)).collect();
        let removed = self.current.iter().copied().filter(|r| !new_set.contains(r)).collect();
        self.current = new;
        (added, removed)
    }

    /// The `NearestSearchCompleted` payload.
    pub fn results_json(&self, db: &Db, session_id: i32, search_id: i32, added: &[FacRef], removed: &[FacRef]) -> String {
        let mut out = String::new();
        let mut o = Obj::new(&mut out);
        o.num("sessionId", session_id as f64).num("searchId", search_id as f64);
        let structs = self.structs;
        let write = |out: &mut String, r: &FacRef| {
            let icao: Icao = db.icao(*r);
            if structs {
                icao.write_struct(out);
            } else {
                super::json::string(out, &icao.v1());
            }
        };
        array(o.key("added"), added, write);
        array(o.key("removed"), removed, write);
        o.end();
        out
    }
}
