//! MSFS facility objects, written as JSON, field for field as msfs-sdk 2.3
//! declares them (`AirportFacility`, `VorFacility`, `NdbFacility`,
//! `IntersectionFacility`, `AirportRunway`, `FacilityILSFrequency`,
//! `ApproachProcedure`, `Procedure`, `FlightPlanLeg`, `AirwaySegment`, ...)
//! with the Coherent `__Type` names, and the legacy V1 ICAO strings
//! alongside the `JS_ICAO` objects.

use super::apt::{self, Airport};
use super::cifp::{self, Cifp, FixRef, Kind, Leg, Procedure};
use super::db::{bearing_deg, distance_m, vor_type, Db, FacRef, FT_TO_M, NM_TO_M};
use super::icao::{region_str, Code, Icao};
use super::json::{self, array, Obj};

/// `AirportFacilityDataFlags`.
pub mod flags {
    pub const APPROACHES: u32 = 1;
    pub const DEPARTURES: u32 = 2;
    pub const ARRIVALS: u32 = 4;
    pub const FREQUENCIES: u32 = 8;
    pub const GATES: u32 = 16;
    pub const HOLDING_PATTERNS: u32 = 32;
    pub const RUNWAYS: u32 = 64;
    pub const ALL: u32 = 127;
}

pub fn normalise180(v: f64) -> f64 {
    let v = (v + 180.).rem_euclid(360.) - 180.;
    if v == -180. {
        180.
    } else {
        v
    }
}

/// Binary coded decimal of the four digits after the leading hundreds
/// digit: 118.50 MHz is 0x1850.
fn bcd16(value: u32) -> u32 {
    let v = value % 10000;
    ((v / 1000) << 12) | ((v / 100 % 10) << 8) | ((v / 10 % 10) << 4) | (v % 10)
}

fn empty_icao(o: &mut Obj<'_>, name: &str) {
    Icao::EMPTY.write_fields(o, name);
}

// ---------------------------------------------------------------------------
// Navaids and waypoints.

/// The station declination of a localizer, east positive: its true bearing
/// less its published magnetic course, or `fallback` when the file gives no
/// magnetic course.
pub fn loc_declination(db: &Db, i: u32, fallback: f64) -> f64 {
    let v = &db.vhf[i as usize];
    match v.loc.as_ref() {
        Some(l) => l.mag_course.map_or(fallback, |m| normalise180(l.true_bearing - m)),
        None => normalise180(v.slaved_var as f64),
    }
}

/// `LandingSystemCategory` from the localizer's name and glideslope.
fn ls_category(v: &super::db::Vhf) -> u32 {
    let Some(loc) = v.loc.as_ref() else { return 0 };
    let has_gs = loc.gs.is_some();
    match v.name.as_str() {
        "ILS-cat-I" => 1,
        "ILS-cat-II" => 2,
        "ILS-cat-III" => 3,
        "LOC" => 4,
        "LDA" if has_gs => 7,
        "LDA" => 6,
        "SDF" if has_gs => 9,
        "SDF" => 8,
        _ if loc.row == 4 => 1,
        _ => 4,
    }
}

/// A `JS_ILSFrequency`; all zero and empty when there is no localizer.
pub fn ils_frequency(db: &Db, loc: Option<u32>, fallback_magvar: f64, out: &mut String) {
    let mut o = Obj::new(out);
    o.str("__Type", "JS_ILSFrequency");
    let Some(i) = loc else {
        empty_icao(&mut o, "icao");
        o.str("name", "").num("freqMHz", 0.).num("freqBCD16", 0.).num("type", 0.).bool("hasBackcourse", false).bool("hasGlideslope", false);
        o.num("glideslopeAlt", 0.).num("glideslopeAngle", 0.).num("glideslopeLat", 0.).num("glideslopeLon", 0.);
        o.num("localizerCourse", 0.).num("localizerWidth", 0.).num("magvar", 0.).num("lsCategory", 0.);
        o.end();
        return;
    };
    let v = &db.vhf[i as usize];
    let loc = v.loc.as_ref().expect("a localizer");
    let decl = loc_declination(db, i, fallback_magvar);
    db.icao(FacRef::Vhf(i)).write_fields(&mut o, "icao");
    o.str("name", &v.name)
        .num("freqMHz", v.freq as f64 / 100.)
        .num("freqBCD16", bcd16(v.freq) as f64)
        .num("type", 0.)
        .bool("hasBackcourse", false)
        .bool("hasGlideslope", loc.gs.is_some());
    match loc.gs.as_ref() {
        Some(gs) => o.num("glideslopeAlt", gs.elev_ft as f64 * FT_TO_M).num("glideslopeAngle", gs.angle).num("glideslopeLat", gs.lat).num("glideslopeLon", gs.lon),
        None => o.num("glideslopeAlt", 0.).num("glideslopeAngle", 0.).num("glideslopeLat", 0.).num("glideslopeLon", 0.),
    };
    let course = loc.mag_course.unwrap_or_else(|| (loc.true_bearing - decl).rem_euclid(360.));
    o.num("localizerCourse", course).num("localizerWidth", 0.).num("magvar", decl).num("lsCategory", ls_category(v) as f64);
    o.end();
}

/// TACAN channel and mode for a paired VHF frequency (ICAO Annex 10 pairing).
fn tacan_channel(freq: u32) -> Option<(u32, u32)> {
    let (base, first) = match freq {
        10800..=11225 => (10800, 17),
        11230..=11795 => (11230, 70),
        _ => return None,
    };
    let steps = (freq - base) / 5;
    Some((first + steps / 2, if steps % 2 == 0 { 88 } else { 89 }))
}

/// A `JS_FacilityVOR`, for VORs, DMEs, TACANs and localizers.
pub fn vor(db: &Db, i: u32, fallback_magvar: f64, out: &mut String) {
    let v = &db.vhf[i as usize];
    let is_loc = v.loc.is_some();
    let decl = if is_loc { loc_declination(db, i, fallback_magvar) } else { normalise180(v.slaved_var as f64) };
    let mut o = Obj::new(out);
    o.str("__Type", "JS_FacilityVOR");
    db.icao(FacRef::Vhf(i)).write_fields(&mut o, "icao");
    o.str("name", &v.name).num("lat", v.lat).num("lon", v.lon).str("region", region_str(&v.region)).str("city", "");
    o.num("freqMHz", v.freq as f64 / 100.).num("freqBCD16", bcd16(v.freq) as f64);
    // Positive west, 0-360, as the sim codes station declination.
    o.num("magneticVariation", (-decl).rem_euclid(360.)).num("magvar", decl);
    o.num("navRange", if is_loc { 0. } else { v.class as f64 * NM_TO_M });
    o.num("type", v.kind as f64);
    let class = if is_loc {
        4
    } else {
        match v.class {
            25 => 1,
            40 => 2,
            125 | 130 | 150 => 3,
            _ => 0,
        }
    };
    o.num("vorClass", class as f64);
    match v.dme.as_ref() {
        Some(d) => {
            let mut dme = Obj::new(o.key("dme"));
            dme.str("__Type", "JS_DME")
                .num("alt", d.elev_ft as f64 * FT_TO_M)
                .bool("atGlideslope", v.loc.as_ref().and_then(|l| l.gs.as_ref()).is_some_and(|g| distance_m(g.lat, g.lon, d.lat, d.lon) < 30.))
                .bool("atNav", distance_m(v.lat, v.lon, d.lat, d.lon) < 30.)
                .num("lat", d.lat)
                .num("lon", d.lon)
                .num("dmeBias", d.bias_nm as f64 * NM_TO_M);
            dme.end();
        }
        None => {
            o.null("dme");
        }
    }
    if is_loc {
        ils_frequency(db, Some(i), fallback_magvar, o.key("ils"));
    } else {
        o.null("ils");
    }
    match (matches!(v.kind, vor_type::TACAN | vor_type::VORTAC), tacan_channel(v.freq)) {
        (true, Some((channel, mode))) => {
            let mut t = Obj::new(o.key("tacan"));
            t.str("__Type", "JS_TACAN").num("alt", v.elev_ft as f64 * FT_TO_M).num("channel", channel as f64).num("lat", v.lat).num("lon", v.lon).num("mode", mode as f64);
            t.end();
        }
        _ => {
            o.null("tacan");
        }
    }
    o.bool("trueReferenced", false).num("alt", v.elev_ft as f64 * FT_TO_M).num("weatherBroadcast", 0.);
    o.end();
}

/// A `JS_FacilityNDB`.
pub fn ndb(db: &Db, i: u32, magvar: f64, out: &mut String) {
    let n = &db.ndbs[i as usize];
    let mut o = Obj::new(out);
    o.str("__Type", "JS_FacilityNDB");
    db.icao(FacRef::Ndb(i)).write_fields(&mut o, "icao");
    o.str("name", &n.name).num("lat", n.lat).num("lon", n.lon).str("region", region_str(&n.region)).str("city", "");
    o.num("freqMHz", n.freq_khz as f64).num("freqBCD16", bcd16(n.freq_khz) as f64).num("magvar", magvar);
    o.num("range", n.class as f64 * NM_TO_M);
    // Locator, low power, normal and high power.
    let kind = match n.class {
        0..=15 => 0,
        16..=25 => 1,
        26..=50 => 2,
        _ => 3,
    };
    o.num("type", kind as f64).bool("bfoRequired", n.bfo).num("alt", n.elev_ft as f64 * FT_TO_M).num("weatherBroadcast", 0.);
    o.end();
}

/// MSFS `IntersectionType` from the ARINC 424 waypoint type (field 5.42).
pub fn intersection_type(w: &super::db::Waypoint) -> u32 {
    match (w.wtype[0], w.wtype[1]) {
        (b'V', _) => 9,
        (_, b'A' | b'B') => 7,
        (_, b'I' | b'K' | b'N') => 6,
        (_, b'F') if w.airport.is_empty() => 5,
        (b'R' | b'C', _) => 1,
        (b'W', _) => 8,
        (b'N', _) => 4,
        (b'I' | b'U', _) => 2,
        _ => 0,
    }
}

/// The nearest VOR-bearing station to a point, for intersections.
fn nearest_vor(db: &Db, lat: f64, lon: f64) -> Option<(u32, f64)> {
    let mut best: Option<(u32, f64)> = None;
    for radius in [100_000., 400_000., 1_500_000.] {
        db.within(lat, lon, radius, |r, d| {
            if let FacRef::Vhf(i) = r {
                let v = &db.vhf[i as usize];
                if matches!(v.kind, vor_type::VOR | vor_type::VORDME | vor_type::VORTAC | vor_type::TACAN) && best.is_none_or(|b| d < b.1) {
                    best = Some((i, d));
                }
            }
        });
        if best.is_some() {
            break;
        }
    }
    best
}

/// A `JS_FacilityIntersection` for a waypoint, VOR or NDB.
pub fn intersection(db: &Db, r: FacRef, out: &mut String) {
    let (lat, lon) = db.position(r);
    let (name, region, kind) = match r {
        FacRef::Waypoint(i) => {
            let w = &db.waypoints[i as usize];
            ("", w.region, intersection_type(w))
        }
        FacRef::Vhf(i) => ("", db.vhf[i as usize].region, 3),
        FacRef::Ndb(i) => ("", db.ndbs[i as usize].region, 4),
        FacRef::Airport(_) => return,
    };
    let mut o = Obj::new(out);
    o.str("__Type", "JS_FacilityIntersection");
    db.icao(r).write_fields(&mut o, "icao");
    o.str("name", name).num("lat", lat).num("lon", lon).str("region", region_str(&region)).str("city", "");
    let routes = db.routes.get(&r).map(Vec::as_slice).unwrap_or(&[]);
    array(o.key("routes"), routes, |out, route| {
        let mut ro = Obj::new(out);
        ro.str("__Type", "JS_Route").str("name", &db.route_names[route.name as usize]).num("type", route.levels as f64);
        match route.prev {
            Some(p) => db.icao(p).write_fields(&mut ro, "prevIcao"),
            None => empty_icao(&mut ro, "prevIcao"),
        }
        ro.num("prevMinAlt", route.prev_min_alt_ft as f64 * FT_TO_M);
        match route.next {
            Some(n) => db.icao(n).write_fields(&mut ro, "nextIcao"),
            None => empty_icao(&mut ro, "nextIcao"),
        }
        ro.num("nextMinAlt", route.next_min_alt_ft as f64 * FT_TO_M);
        ro.end();
    });
    match nearest_vor(db, lat, lon) {
        Some((i, d)) => {
            let v = &db.vhf[i as usize];
            let radial = bearing_deg(v.lat, v.lon, lat, lon);
            db.icao(FacRef::Vhf(i)).write_fields(&mut o, "nearestVorICAO");
            o.num("nearestVorType", v.kind as f64)
                .num("nearestVorFrequencyBCD16", bcd16(v.freq) as f64)
                .num("nearestVorFrequencyMHz", v.freq as f64 / 100.)
                .num("nearestVorTrueRadial", radial)
                .num("nearestVorMagneticRadial", (radial - v.slaved_var as f64).rem_euclid(360.))
                .num("nearestVorDistance", d);
        }
        None => {
            empty_icao(&mut o, "nearestVorICAO");
            o.num("nearestVorType", 0.).num("nearestVorFrequencyBCD16", 0.).num("nearestVorFrequencyMHz", 0.);
            o.num("nearestVorTrueRadial", 0.).num("nearestVorMagneticRadial", 0.).num("nearestVorDistance", 0.);
        }
    }
    o.num("type", kind as f64);
    o.end();
}

// ---------------------------------------------------------------------------
// Airports.

/// MSFS `RunwaySurfaceType` for an apt.dat surface code.
pub fn runway_surface(code: u16) -> u32 {
    match code {
        1 | 20..=38 => 4,
        2 | 50..=57 => 0,
        3 => 1,
        4 | 12 => 12,
        5 => 14,
        13 => 27,
        14 => 8,
        _ => 0,
    }
}

fn is_hard(code: u16) -> bool {
    matches!(code, 1 | 2 | 15 | 20..=38 | 50..=57)
}

/// MSFS `AirportClass`.
pub fn airport_class(a: &Airport) -> u32 {
    let land: Vec<_> = a.runways.iter().filter(|r| !r.water).collect();
    // apt.dat row 17 is a heliport, whatever else it lists.
    if a.kind == 17 {
        4
    } else if land.iter().any(|r| is_hard(r.surface)) {
        1
    } else if !land.is_empty() {
        2
    } else if !a.runways.is_empty() {
        3
    } else if a.helipads > 0 {
        4
    } else {
        0
    }
}

/// Runway number and MSFS `RunwayDesignator` from "09L".
pub fn runway_id(id: &str) -> Option<(u32, u32)> {
    let digits: String = id.chars().take_while(char::is_ascii_digit).collect();
    let number: u32 = digits.parse().ok()?;
    let designator = match id[digits.len()..].chars().next() {
        Some('L') => 1,
        Some('R') => 2,
        Some('C') => 3,
        Some('W') => 4,
        _ => 0,
    };
    Some((number, designator))
}

fn designator_letter(d: u32) -> &'static str {
    match d {
        1 => "L",
        2 => "R",
        3 => "C",
        4 => "W",
        5 => "A",
        6 => "B",
        _ => "",
    }
}

/// The inputs to an airport facility, gathered on the main thread.
pub struct AirportRequest<'a> {
    pub index: u32,
    pub flags: u32,
    /// Magnetic variation at the airport, east positive.
    pub magvar: f64,
    pub cifp: Option<&'a Cifp>,
    pub gates: &'a [apt::Gate],
}

pub fn airport(db: &Db, req: &AirportRequest<'_>, out: &mut String) {
    let a = &db.airports[req.index as usize];
    let f = req.flags;
    let mut o = Obj::new(out);
    o.str("__Type", "JS_FacilityAirport");
    db.icao(FacRef::Airport(req.index)).write_fields(&mut o, "icao");
    let city = a.city.clone();
    o.str("name", &a.name).num("lat", a.lat).num("lon", a.lon).str("region", region_str(&a.region)).str("city", &city);
    o.num("loadedDataFlags", f as f64).num("airportPrivateType", 0.).str("fuel1", "").str("fuel2", "").str("bestApproach", "");
    o.num("radarCoverage", 0.).num("airspaceType", 0.).num("airportClass", airport_class(a) as f64).bool("towered", a.towered());

    // Frequencies.
    let freqs = if f & flags::FREQUENCIES != 0 { a.freqs.as_slice() } else { &[] };
    array(o.key("frequencies"), freqs, |out, q| {
        let upper = q.name.to_ascii_uppercase();
        let kind = match q.kind {
            0 if upper.contains("ASOS") => 13,
            0 if upper.contains("AWOS") => 12,
            0 => 1,
            1 if upper.contains("CTAF") => 4,
            1 => 3,
            2 => 7,
            3 => 5,
            4 => 6,
            5 => 8,
            6 => 9,
            _ => 0,
        };
        let mut fo = Obj::new(out);
        fo.str("__Type", "JS_Frequency");
        empty_icao(&mut fo, "icao");
        fo.str("name", &q.name).num("freqMHz", q.khz as f64 / 1000.).num("freqBCD16", bcd16(q.khz / 10) as f64).num("type", kind as f64);
        fo.end();
    });

    // Runways.
    // apt.dat may write "4L" where CIFP and earth_nav.dat write "04L".
    let same_runway = |a: &str, b: &str| runway_id(a).is_some() && runway_id(a) == runway_id(b);
    let runway_elev = |id: &str| -> f64 {
        req.cifp
            .and_then(|c| c.runways.iter().find(|r| r.id.as_str().strip_prefix("RW").is_some_and(|r| same_runway(r, id))))
            .and_then(|r| r.threshold_elev_ft)
            .map_or(a.elev_ft as f64, |e| e as f64)
            * FT_TO_M
    };
    let locs = db.locs.get(&a.header_id).or_else(|| db.locs.get(&a.ident));
    // An ILS before a localizer-only record for the same runway.
    let loc_for = |id: &str| -> Option<u32> {
        let mut found: Vec<u32> = locs.map(|l| l.iter().copied().filter(|&i| db.vhf[i as usize].loc.as_ref().is_some_and(|x| same_runway(x.runway.as_str(), id))).collect()).unwrap_or_default();
        found.sort_by_key(|&i| db.vhf[i as usize].loc.as_ref().map_or(9, |x| x.row));
        found.first().copied()
    };
    let runways: Vec<&apt::Runway> = if f & flags::RUNWAYS != 0 {
        a.runways.iter().filter(|r| runway_id(r.ends[0].id.as_str()).is_some() && runway_id(r.ends[1].id.as_str()).is_some()).collect()
    } else {
        Vec::new()
    };
    array(o.key("runways"), runways, |out, r| {
        let (e1, e2) = (&r.ends[0], &r.ends[1]);
        let (n1, d1) = runway_id(e1.id.as_str()).unwrap_or_default();
        let (n2, d2) = runway_id(e2.id.as_str()).unwrap_or_default();
        let (el1, el2) = (runway_elev(e1.id.as_str()), runway_elev(e2.id.as_str()));
        let mut ro = Obj::new(out);
        ro.str("__Type", "JS_Runway")
            .num("latitude", (e1.lat + e2.lat) / 2.)
            .num("longitude", (e1.lon + e2.lon) / 2.)
            .num("elevation", (el1 + el2) / 2.)
            .num("direction", bearing_deg(e1.lat, e1.lon, e2.lat, e2.lon))
            .str("designation", &format!("{n1}-{n2}"))
            .num("length", distance_m(e1.lat, e1.lon, e2.lat, e2.lon))
            .num("width", r.width_m as f64)
            .num("surface", runway_surface(r.surface) as f64)
            // X-Plane knows whether a runway has edge lights, not their schedule.
            .num("lighting", if r.water || r.edge_lights == 0 { 1. } else { 0. })
            .num("designatorCharPrimary", d1 as f64)
            .num("designatorCharSecondary", d2 as f64);
        ils_frequency(db, loc_for(e1.id.as_str()), req.magvar, ro.key("primaryILSFrequency"));
        ils_frequency(db, loc_for(e2.id.as_str()), req.magvar, ro.key("secondaryILSFrequency"));
        ro.num("primaryBlastpadLength", 0.)
            .num("primaryElevation", el1)
            .num("primaryOverrunLength", e1.overrun_m as f64)
            .num("primaryThresholdLength", e1.displaced_m as f64)
            .num("secondaryBlastpadLength", 0.)
            .num("secondaryElevation", el2)
            .num("secondaryOverrunLength", e2.overrun_m as f64)
            .num("secondaryThresholdLength", e2.displaced_m as f64);
        ro.end();
    });

    // Procedures.
    let procs = |kind: Kind, flag: u32| -> Vec<&Procedure> {
        match req.cifp {
            Some(c) if f & flag != 0 => c.procedures.iter().filter(|p| p.kind == kind).collect(),
            _ => Vec::new(),
        }
    };
    let ctx = ProcContext { db, airport: a, cifp: req.cifp };
    array(o.key("departures"), procs(Kind::Sid, flags::DEPARTURES), |out, p| ctx.departure_or_arrival(p, out));
    array(o.key("approaches"), procs(Kind::Approach, flags::APPROACHES), |out, p| ctx.approach(p, out));
    array(o.key("arrivals"), procs(Kind::Star, flags::ARRIVALS), |out, p| ctx.departure_or_arrival(p, out));

    // Gates.
    let gates = if f & flags::GATES != 0 { req.gates } else { &[] };
    array(o.key("gates"), gates.iter().filter_map(|g| gate_parts(g).map(|p| (g, p))), |out, (g, (name, number, suffix))| {
        let circumference = 2. * std::f64::consts::PI * super::db::EARTH_RADIUS_M;
        let mut go = Obj::new(out);
        go.str("__Type", "JS_Gate")
            .num("latitude", (g.lat - a.lat) * circumference / 360.)
            .num("longitude", (g.lon - a.lon) * circumference * a.lat.to_radians().cos() / 360.)
            .num("name", name as f64)
            .num("number", number as f64)
            .num("suffix", suffix as f64);
        go.end();
    });

    // Terminal holding patterns.
    let holds = if f & flags::HOLDING_PATTERNS != 0 { db.holds.get(&a.ident).map(Vec::as_slice).unwrap_or(&[]) } else { &[] };
    array(o.key("holdingPatterns"), holds, |out, h| {
        let kind = match h.fix_type {
            2 => b'N',
            3 => b'V',
            _ => b'W',
        };
        let fix = ctx.resolve_icao(kind, h.ident, h.region, h.airport);
        let mut ho = Obj::new(out);
        ho.str("__Type", "JS_HoldingPattern");
        fix.write_struct(ho.key("icaoStruct"));
        ho.num("inboundCourse", h.inbound_course as f64)
            .num("legLength", h.leg_length_nm as f64 * NM_TO_M)
            .num("legTime", h.leg_time_min as f64)
            .num("maxAltitude", h.max_alt_ft as f64 * FT_TO_M)
            .num("minAltitude", h.min_alt_ft as f64 * FT_TO_M)
            .str("name", "")
            .num("radius", 0.)
            .num("rnp", 0.)
            .num("speed", h.speed_kt as f64)
            .bool("turnRight", h.turn_right);
        ho.end();
    });

    o.num("magvar", req.magvar)
        .num("transitionAlt", a.ta_ft.map_or(0., |v| v as f64 * FT_TO_M))
        .num("transitionLevel", a.tl_ft.map_or(0., |v| v as f64 * FT_TO_M))
        .str("iata", a.iata.as_str())
        .num("altitude", a.elev_ft as f64 * FT_TO_M);
    o.end();
}

/// MSFS gate name, number and suffix codes from an apt.dat startup location:
/// letters A-Z are codes 12-37, a gate without a letter is `GATE` (10) and
/// any other stand is `PARKING` (1).
fn gate_parts(g: &apt::Gate) -> Option<(u32, u32, u32)> {
    let token = g.name.split_whitespace().rev().find(|t| t.chars().any(|c| c.is_ascii_digit()))?;
    let bytes = token.as_bytes();
    let start = bytes.iter().position(u8::is_ascii_digit)?;
    let end = start + bytes[start..].iter().take_while(|b| b.is_ascii_digit()).count();
    let number: u32 = token[start..end].parse().ok()?;
    let letter = |b: Option<&u8>| b.filter(|b| b.is_ascii_uppercase()).map(|b| 12 + (b - b'A') as u32);
    let prefix = if start > 0 { letter(bytes.get(start - 1)) } else { None };
    let name = prefix.unwrap_or(if g.kind == "gate" { 10 } else { 1 });
    Some((name, number, letter(bytes.get(end)).unwrap_or(0)))
}

// ---------------------------------------------------------------------------
// Procedures.

pub struct ProcContext<'a> {
    pub db: &'a Db,
    pub airport: &'a Airport,
    pub cifp: Option<&'a Cifp>,
}

const SID_RUNWAY: &[u8] = b"14FT";
const SID_COMMON: &[u8] = b"25M";
const SID_ENROUTE: &[u8] = b"36SV";
const STAR_ENROUTE: &[u8] = b"147F";
const STAR_COMMON: &[u8] = b"258M";
const STAR_RUNWAY: &[u8] = b"369S";

impl ProcContext<'_> {
    /// The ICAO of a facility a procedure names, checked against the
    /// database: the terminal area it names first, then elsewhere.
    fn resolve_icao(&self, kind: u8, ident: Code, region: [u8; 2], airport: Code) -> Icao {
        let db = self.db;
        let want = Icao::new(kind, region, airport, ident);
        if db.find(&want).is_some() {
            return want;
        }
        let alt = Icao::new(kind, region, if airport.is_empty() { self.airport.ident } else { Code::EMPTY }, ident);
        if db.find(&alt).is_some() {
            return alt;
        }
        // Same ident and region anywhere: the one nearest the airport.
        let mut best: Option<(Icao, f64)> = None;
        for (_, r) in db.with_ident_prefix(&ident).filter(|(c, _)| *c == ident) {
            let icao = db.icao(r);
            if icao.kind != kind || (region != [0; 2] && icao.region != region && kind != b'V') {
                continue;
            }
            let (lat, lon) = db.position(r);
            let d = distance_m(lat, lon, self.airport.lat, self.airport.lon);
            if best.is_none_or(|b| d < b.1) {
                best = Some((icao, d));
            }
        }
        best.map_or(want, |b| b.0)
    }

    fn fix_icao(&self, f: &FixRef) -> Icao {
        if f.is_empty() {
            return Icao::EMPTY;
        }
        let apt = self.airport.ident;
        match (f.section, f.subsection) {
            (b'D', b'B') => self.resolve_icao(b'N', f.ident, f.region, Code::EMPTY),
            (b'D', _) => self.resolve_icao(b'V', f.ident, f.region, Code::EMPTY),
            (b'E', _) => self.resolve_icao(b'W', f.ident, f.region, Code::EMPTY),
            (b'P', b'C') => self.resolve_icao(b'W', f.ident, f.region, apt),
            (b'P', b'N') => self.resolve_icao(b'N', f.ident, f.region, apt),
            (b'P', b'G') => Icao::new(b'R', f.region, apt, f.ident),
            (b'P', b'A') => Icao::new(b'A', f.region, Code::EMPTY, f.ident),
            (b'P', b'I') => self.resolve_icao(b'V', f.ident, [0; 2], apt),
            _ => Icao::EMPTY,
        }
    }

    fn leg(&self, leg: &Leg, missed: bool, out: &mut String) {
        let mut o = Obj::new(out);
        o.str("__Type", "JS_Leg").num("type", cifp::leg_type(leg.path) as f64);
        self.fix_icao(&leg.fix).write_fields(&mut o, "fixIcao");
        o.bool("flyOver", matches!(leg.desc[1], b'Y' | b'B'))
            .bool("distanceMinutes", leg.minutes)
            .bool("trueDegrees", leg.course_true)
            .num(
                "turnDirection",
                match leg.turn {
                    b'L' => 1.,
                    b'R' => 2.,
                    b'E' => 3.,
                    _ => 0.,
                },
            );
        self.fix_icao(&leg.navaid).write_fields(&mut o, "originIcao");
        let center = if &leg.path == b"RF" { self.fix_icao(&leg.center) } else { Icao::EMPTY };
        center.write_fields(&mut o, "arcCenterFixIcao");
        o.num("theta", leg.theta.unwrap_or(0.))
            .num("rho", leg.rho_nm.unwrap_or(0.) * NM_TO_M)
            .num("course", leg.course.unwrap_or(0.))
            .num("distance", leg.distance.map_or(0., |d| if leg.minutes { d } else { d * NM_TO_M }));
        let (speed_desc, speed) = match leg.speed_kt {
            Some(s) => (
                match leg.speed_desc {
                    b'+' => 2,
                    b'-' => 3,
                    _ => 1,
                },
                s,
            ),
            None => (0, 0),
        };
        o.num("speedRestriction", speed as f64).num("speedRestrictionDesc", speed_desc as f64);
        let (alt_desc, alt1, alt2) = altitude_restriction(leg);
        o.num("altDesc", alt_desc as f64).num("altitude1", alt1 as f64 * FT_TO_M).num("altitude2", alt2 as f64 * FT_TO_M);
        let mut fix_flags = match leg.desc[3] {
            b'A' | b'C' | b'D' => 1,
            b'B' => 2,
            b'M' => 4,
            b'F' => 8,
            _ => 0,
        };
        if missed && leg.desc[3] == b'H' {
            fix_flags |= 16;
        }
        o.num("fixTypeFlags", fix_flags as f64)
            .num("verticalAngle", leg.vertical_angle.map_or(0., |a| 360. + a))
            .num("rnp", leg.rnp_nm.map_or(0., |r| r * NM_TO_M));
        o.end();
    }

    fn legs<'l>(&self, legs: impl IntoIterator<Item = &'l Leg>, missed: bool, out: &mut String) {
        array(out, legs, |out, l| self.leg(l, missed, out));
    }

    /// Runway number and designator pairs a transition ident names: "RW09L"
    /// is one runway, "RW12B" every runway numbered 12, "ALL" none.
    fn runways_named(&self, trans: &str) -> Vec<(u32, u32)> {
        let Some(id) = trans.strip_prefix("RW") else { return Vec::new() };
        let Some((number, _)) = runway_id(id) else { return Vec::new() };
        if !id.ends_with('B') {
            return runway_id(id).into_iter().collect();
        }
        let mut ids: Vec<String> = self.cifp.map(|c| c.runways.iter().filter_map(|r| r.id.as_str().strip_prefix("RW").map(str::to_string)).collect()).unwrap_or_default();
        if ids.is_empty() {
            ids = self.airport.runways.iter().flat_map(|r| r.ends.iter().map(|e| e.id.as_str().to_string())).collect();
        }
        let mut out: Vec<(u32, u32)> = ids.iter().filter_map(|i| runway_id(i)).filter(|(n, _)| *n == number).collect();
        out.sort();
        out.dedup();
        out
    }

    fn runway_transition(&self, out: &mut String, runway: (u32, u32), legs: &[&Leg]) {
        let mut o = Obj::new(out);
        o.str("__Type", "JS_RunwayTransition").num("runwayNumber", runway.0 as f64).num("runwayDesignation", runway.1 as f64);
        self.legs(legs.iter().copied(), false, o.key("legs"));
        o.end();
    }

    /// A `JS_Departure` or `JS_Arrival`.
    fn departure_or_arrival(&self, p: &Procedure, out: &mut String) {
        let (runway_types, common_types, enroute_types, type_name) = match p.kind {
            Kind::Sid => (SID_RUNWAY, SID_COMMON, SID_ENROUTE, "JS_Departure"),
            _ => (STAR_RUNWAY, STAR_COMMON, STAR_ENROUTE, "JS_Arrival"),
        };
        let groups = |types: &[u8]| -> Vec<(String, Vec<&Leg>)> {
            let mut groups: Vec<(String, Vec<&Leg>)> = Vec::new();
            for l in p.legs.iter().filter(|l| types.contains(&l.route_type)) {
                match groups.iter_mut().find(|g| g.0 == l.transition) {
                    Some(g) => g.1.push(l),
                    None => groups.push((l.transition.clone(), vec![l])),
                }
            }
            groups
        };
        let common = groups(common_types);
        let mut runway = groups(runway_types);
        let enroute = groups(enroute_types);
        // MSFS has one common route. Several (one per runway) are runway
        // transitions in all but name; a single one for named runways still
        // tells which runways the procedure serves.
        let mut common_legs: Vec<&Leg> = Vec::new();
        if common.len() == 1 {
            common_legs = common[0].1.clone();
            if runway.is_empty() && common[0].0.starts_with("RW") {
                runway.push((common[0].0.clone(), Vec::new()));
            }
        } else {
            for (name, legs) in common {
                if name.starts_with("RW") {
                    runway.push((name, legs));
                } else {
                    common_legs.extend(legs);
                }
            }
        }
        let mut o = Obj::new(out);
        o.str("__Type", type_name).str("name", &p.ident);
        self.legs(common_legs.iter().copied(), false, o.key("commonLegs"));
        array(o.key("enRouteTransitions"), &enroute, |out, (name, legs)| {
            let mut t = Obj::new(out);
            t.str("__Type", "JS_EnRouteTransition").str("name", name);
            self.legs(legs.iter().copied(), false, t.key("legs"));
            t.end();
        });
        let expanded: Vec<((u32, u32), &Vec<&Leg>)> = runway.iter().flat_map(|(name, legs)| self.runways_named(name).into_iter().map(move |r| (r, legs))).collect();
        array(o.key("runwayTransitions"), expanded, |out, (r, legs)| self.runway_transition(out, r, legs));
        o.bool("rnpAr", p.legs.iter().any(|l| l.qualifier1 == b'P'));
        o.end();
    }

    /// A `JS_Approach`.
    fn approach(&self, p: &Procedure, out: &mut String) {
        let (kind_char, number, designator, suffix) = approach_ident(&p.ident);
        let finals: Vec<&Leg> = p.legs.iter().filter(|l| l.route_type != b'A').collect();
        let route_type = finals.first().map_or(kind_char, |l| l.route_type);
        let missed_at = finals.iter().position(|l| l.desc[2] == b'M').unwrap_or(finals.len());
        let mut transitions: Vec<(String, Vec<&Leg>)> = Vec::new();
        for l in p.legs.iter().filter(|l| l.route_type == b'A') {
            match transitions.iter_mut().find(|t| t.0 == l.transition) {
                Some(t) => t.1.push(l),
                None => transitions.push((l.transition.clone(), vec![l])),
            }
        }
        let rnav_flags = p.service.iter().filter(|s| s.0).fold(0u32, |acc, (_, name)| {
            acc | match name.as_str() {
                "LNAV" => 1,
                "LNAV/VNAV" => 2,
                "LP" => 4,
                "LPV" => 8,
                _ => 0,
            }
        });
        let rnp_ar = route_type == b'H' || finals.iter().any(|l| l.qualifier1 == b'P');
        let runway = if number > 0 { format!("{number:02}{}", designator_letter(designator)) } else { String::new() };
        let mut o = Obj::new(out);
        o.str("__Type", "JS_Approach").str("name", &approach_name(route_type, &runway, &suffix)).str("runway", &runway);
        array(o.key("icaos"), std::iter::empty::<&str>(), |out, s| json::string(out, s));
        array(o.key("transitions"), &transitions, |out, (name, legs)| {
            let mut t = Obj::new(out);
            t.str("__Type", "JS_ApproachTransition").str("name", name);
            self.legs(legs.iter().copied(), false, t.key("legs"));
            t.end();
        });
        self.legs(finals[..missed_at].iter().copied(), false, o.key("finalLegs"));
        self.legs(finals[missed_at..].iter().copied(), true, o.key("missedLegs"));
        o.num("approachType", approach_type(route_type) as f64)
            .str("approachSuffix", &suffix)
            .num("runwayDesignator", designator as f64)
            .num("runwayNumber", number as f64)
            .num("rnavTypeFlags", rnav_flags as f64)
            .bool("rnpAr", rnp_ar)
            .bool("missedApproachRnpAr", rnp_ar);
        o.end();
    }
}

/// MSFS `AltitudeRestrictionType` and the two altitudes (feet) for a leg.
/// ARINC descriptors beyond the four MSFS types keep their first altitude's
/// meaning: G and I are at, H and J at or above (with the glideslope
/// altitude in the second field), C is at or above the second altitude
/// (moved to the first), V at or above, X at, Y at or below.
pub fn altitude_restriction(leg: &Leg) -> (u32, i32, i32) {
    let a2 = leg.alt2_ft.unwrap_or(0);
    if leg.alt_desc == b'C' {
        return if a2 > 0 { (2, a2, 0) } else { (0, 0, 0) };
    }
    let Some(a1) = leg.alt1_ft else { return (0, 0, 0) };
    match leg.alt_desc {
        b'+' | b'H' | b'J' | b'V' => (2, a1, a2),
        b'-' | b'Y' => (3, a1, a2),
        b'B' => (4, a1, a2),
        _ => (1, a1, a2),
    }
}

/// Type letter, runway number, designator and multiple indicator of an
/// ARINC approach ident ("I27L", "R04LY", "R27-Y", "VDMA").
pub fn approach_ident(ident: &str) -> (u8, u32, u32, String) {
    let kind = ident.bytes().next().unwrap_or(b' ');
    let rest = ident.get(1..).unwrap_or("");
    let digits: String = rest.chars().take_while(char::is_ascii_digit).collect();
    if digits.len() != 2 {
        let suffix = rest.rsplit_once('-').map(|(_, s)| s.to_string()).unwrap_or_default();
        return (kind, 0, 0, suffix);
    }
    let number = digits.parse().unwrap_or(0);
    let mut tail = &rest[2..];
    let designator = match tail.chars().next() {
        Some('L') => 1,
        Some('R') => 2,
        Some('C') => 3,
        _ => 0,
    };
    if designator > 0 {
        tail = &tail[1..];
    }
    (kind, number, designator, tail.trim_start_matches('-').trim().to_string())
}

/// MSFS `ApproachType` for an ARINC approach route type.
pub fn approach_type(route_type: u8) -> u32 {
    match route_type {
        b'P' => 1,
        b'V' => 2,
        b'N' => 3,
        b'I' | b'G' => 4,
        b'L' => 5,
        b'U' => 6,
        b'X' => 7,
        b'D' | b'S' => 8,
        b'Q' => 9,
        b'R' | b'H' | b'F' => 10,
        b'B' => 11,
        _ => 0,
    }
}

fn approach_name(route_type: u8, runway: &str, suffix: &str) -> String {
    let kind = match route_type {
        b'P' => "GPS",
        b'V' => "VOR",
        b'N' => "NDB",
        b'I' | b'G' => "ILS",
        b'L' => "LOC",
        b'U' => "SDF",
        b'X' => "LDA",
        b'D' | b'S' => "VORDME",
        b'Q' => "NDBDME",
        b'R' | b'H' | b'F' => "RNAV",
        b'B' => "LOC BC",
        b'J' => "GLS",
        b'T' => "TACAN",
        _ => "",
    };
    [kind, runway, suffix].iter().filter(|s| !s.is_empty()).copied().collect::<Vec<_>>().join(" ")
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn codes() {
        assert_eq!(bcd16(11850), 0x1850);
        assert_eq!(bcd16(10950), 0x0950);
        assert_eq!(tacan_channel(11460), Some((93, 88)));
        assert_eq!(tacan_channel(10800), Some((17, 88)));
        assert_eq!(tacan_channel(11795), Some((126, 89)));
        assert_eq!(tacan_channel(11200), Some((57, 88)));
        assert_eq!(runway_id("09L"), Some((9, 1)));
        assert_eq!(runway_id("36"), Some((36, 0)));
        assert_eq!(runway_id("N"), None);
        assert_eq!(approach_ident("I27L"), (b'I', 27, 1, String::new()));
        assert_eq!(approach_ident("R04LY"), (b'R', 4, 1, "Y".into()));
        assert_eq!(approach_ident("R27-Y"), (b'R', 27, 0, "Y".into()));
        assert_eq!(approach_ident("VDM-A"), (b'V', 0, 0, "A".into()));
        assert_eq!(approach_name(b'R', "04L", "Y"), "RNAV 04L Y");
        assert!((normalise180(359.291) + 0.709).abs() < 1e-9);
        let gate = |kind: &str, name: &str| gate_parts(&apt::Gate { lat: 0., lon: 0., kind: kind.into(), name: name.into() });
        assert_eq!(gate("gate", "Gate A12"), Some((12, 12, 0)));
        assert_eq!(gate("gate", "505"), Some((10, 505, 0)));
        assert_eq!(gate("tie_down", "Stand 21R"), Some((1, 21, 29)));
        assert_eq!(gate("misc", "Maintenance"), None);
    }
}
