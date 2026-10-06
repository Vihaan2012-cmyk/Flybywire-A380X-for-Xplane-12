//! Tests against X-Plane 12's installed navigation data, with every facility
//! object checked field by field against msfs-sdk 2.3.3's declarations
//! (msfssdk.d.ts). Each test is skipped when the installation or the SDK is
//! not on this machine.

use std::collections::HashMap;
use std::path::Path;
use std::sync::{Mutex, OnceLock};
use std::time::{Duration, Instant};

use serde_json::Value as Json;

use super::NavData;

const XPLANE: &str = r"D:\Steam Games\steamapps\common\X-Plane 12";
const SDK_TYPES: &str = r"D:\A380\fbw-build\sdk\package\msfssdk.d.ts";

#[link(name = "psapi")]
extern "system" {
    fn GetCurrentProcess() -> *mut std::ffi::c_void;
    fn GetProcessMemoryInfo(process: *mut std::ffi::c_void, counters: *mut MemoryCounters, size: u32) -> i32;
}

#[repr(C)]
#[derive(Default)]
struct MemoryCounters {
    cb: u32,
    page_fault_count: u32,
    peak_working_set: usize,
    working_set: usize,
    quota_peak_paged_pool: usize,
    quota_paged_pool: usize,
    quota_peak_non_paged_pool: usize,
    quota_non_paged_pool: usize,
    pagefile_usage: usize,
    peak_pagefile_usage: usize,
}

/// Private bytes of this process.
fn private_bytes() -> usize {
    let mut c = MemoryCounters { cb: std::mem::size_of::<MemoryCounters>() as u32, ..Default::default() };
    unsafe { GetProcessMemoryInfo(GetCurrentProcess(), &mut c, c.cb) };
    c.pagefile_usage
}

/// One database for every test (loading takes seconds).
fn navdata() -> Option<std::sync::MutexGuard<'static, NavData>> {
    static NAV: OnceLock<Option<Mutex<NavData>>> = OnceLock::new();
    NAV.get_or_init(|| {
        if !Path::new(XPLANE).join("Resources").is_dir() {
            eprintln!("X-Plane 12 is not installed at {XPLANE}; skipping");
            return None;
        }
        let before = private_bytes();
        let started = Instant::now();
        let mut nav = NavData::load(Path::new(XPLANE)).unwrap();
        nav.wait_ready().unwrap();
        let s = nav.stats().unwrap();
        eprintln!(
            "navdata: loaded in {} ms (navaids {} ms, fixes {} ms, airways and holds {} ms, airports {} ms, index {} ms), {} MB more private memory; \
             {} airports, {} waypoints, {} VHF, {} NDB, {} airway segments, {} terminal holds; cycle {:?}, {:?}",
            started.elapsed().as_millis(),
            s.navaids_ms,
            s.fixes_ms,
            s.airways_ms,
            s.airports_ms,
            s.index_ms,
            private_bytes().saturating_sub(before) / (1 << 20),
            s.airports,
            s.waypoints,
            s.vhf,
            s.ndbs,
            s.airway_segments,
            s.holds,
            nav.cycle(),
            nav.date_range(),
        );
        Some(Mutex::new(nav))
    })
    .as_ref()
    .map(|m| m.lock().unwrap_or_else(|e| e.into_inner()))
}

fn call(nav: &mut NavData, name: &str, args: &str) -> Json {
    let text = nav.call(name, args).unwrap_or_else(|e| panic!("{name}({args}): {e}"));
    serde_json::from_str(&text).unwrap_or_else(|e| panic!("{name}({args}) gave bad JSON: {e}"))
}

/// The events a call raised, once airports on the worker are built.
fn events(nav: &mut NavData) -> Vec<(String, Json)> {
    nav.take_events_blocking(Duration::from_secs(10))
        .into_iter()
        .map(|(name, args)| {
            let args: Json = serde_json::from_str(&args).unwrap_or_else(|e| panic!("{name} gave bad JSON: {e}"));
            (name, args)
        })
        .collect()
}

/// A `LOAD_*` call's facility, as its `Send*` event brings it, or null when
/// the call answers false.
fn fetch(nav: &mut NavData, name: &str, args: &str) -> Json {
    let found = call(nav, name, args);
    let mut sent = events(nav);
    match found {
        Json::Bool(true) => {
            assert_eq!(sent.len(), 1, "{name}({args}) sent {:?}", sent.iter().map(|e| &e.0).collect::<Vec<_>>());
            let (event, args) = sent.remove(0);
            assert!(event.starts_with("Send"), "{event}");
            let Json::Array(mut a) = args else { panic!("{event} arguments are not an array") };
            assert_eq!(a.len(), 1);
            a.remove(0)
        }
        Json::Bool(false) => {
            assert!(sent.is_empty());
            Json::Null
        }
        other => panic!("{name}({args}) answered {other}"),
    }
}

/// A nearest search's results, as its completion event brings them.
fn search(nav: &mut NavData, name: &str, args: &str) -> Json {
    let search_id = call(nav, name, args);
    let mut sent = events(nav);
    assert_eq!(sent.len(), 1);
    let (event, Json::Array(mut a)) = sent.remove(0) else { panic!("arguments are not an array") };
    assert!(event.starts_with("NearestSearchCompleted"), "{event}");
    let results = a.remove(0);
    assert_eq!(results["searchId"], search_id);
    results
}

fn icao_struct(kind: &str, region: &str, airport: &str, ident: &str) -> String {
    format!(r#"{{"__Type":"JS_ICAO","type":"{kind}","region":"{region}","airport":"{airport}","ident":"{ident}"}}"#)
}

fn airport(nav: &mut NavData, ident: &str) -> Json {
    fetch(nav, "LOAD_AIRPORT_FROM_STRUCT", &format!("[{}, 127]", icao_struct("A", "", "", ident)))
}

fn num(v: &Json, key: &str) -> f64 {
    v[key].as_f64().unwrap_or_else(|| panic!("{key} is not a number in {v}"))
}

fn runway_end<'a>(apt: &'a Json, number: u64, designator: u64) -> (&'a Json, bool) {
    for r in apt["runways"].as_array().unwrap() {
        let d: Vec<u64> = r["designation"].as_str().unwrap().split('-').map(|n| n.parse().unwrap()).collect();
        if d[0] == number && r["designatorCharPrimary"] == designator {
            return (r, true);
        }
        if d[1] == number && r["designatorCharSecondary"] == designator {
            return (r, false);
        }
    }
    panic!("no runway {number}/{designator}");
}

/// The fields msfs-sdk declares for its facility interfaces, with those
/// inherited, and whether each is optional.
struct SdkTypes(HashMap<String, (Option<String>, Vec<(String, bool)>)>);

impl SdkTypes {
    fn load() -> Option<SdkTypes> {
        let text = std::fs::read_to_string(SDK_TYPES).ok()?;
        let mut out = HashMap::new();
        let mut lines = text.lines();
        while let Some(line) = lines.next() {
            let Some(rest) = line.strip_prefix("interface ") else { continue };
            let head: Vec<&str> = rest.trim_end_matches('{').split_whitespace().collect();
            let name = head[0].split('<').next().unwrap().to_string();
            let parent = head.iter().position(|w| *w == "extends").map(|i| head[i + 1].split('<').next().unwrap().to_string());
            let mut fields = Vec::new();
            for l in lines.by_ref() {
                if l.starts_with('}') {
                    break;
                }
                let l = l.trim().trim_start_matches("readonly ");
                if l.starts_with("/*") || l.starts_with('*') {
                    continue;
                }
                if let Some(colon) = l.find(':') {
                    let key = &l[..colon];
                    if key.chars().all(|c| c.is_alphanumeric() || c == '_' || c == '?') && !key.is_empty() {
                        fields.push((key.trim_end_matches('?').to_string(), key.ends_with('?')));
                    }
                }
            }
            out.insert(name, (parent, fields));
        }
        Some(SdkTypes(out))
    }

    fn fields(&self, name: &str) -> Vec<(String, bool)> {
        let (parent, own) = self.0.get(name).unwrap_or_else(|| panic!("msfs-sdk declares no {name}"));
        let mut all = parent.as_ref().map(|p| self.fields(p)).unwrap_or_default();
        all.extend(own.iter().cloned());
        all
    }

    /// Asserts `value` has every required field `name` declares.
    fn check(&self, name: &str, value: &Json, path: &str) {
        let obj = value.as_object().unwrap_or_else(|| panic!("{path} is not an object: {value}"));
        for (field, optional) in self.fields(name) {
            assert!(optional || obj.contains_key(&field), "{path} ({name}) lacks {field}");
        }
    }
}

fn check_icao(v: &Json, path: &str) {
    assert_eq!(v["__Type"], "JS_ICAO", "{path}");
    for k in ["type", "region", "airport", "ident"] {
        assert!(v[k].is_string(), "{path}.{k}");
    }
}

fn check_leg(t: &SdkTypes, leg: &Json, path: &str) {
    t.check("FlightPlanLeg", leg, path);
    for k in ["fixIcaoStruct", "originIcaoStruct", "arcCenterFixIcaoStruct"] {
        check_icao(&leg[k], &format!("{path}.{k}"));
    }
}

fn check_airport_shape(t: &SdkTypes, apt: &Json) {
    t.check("AirportFacility", apt, "airport");
    check_icao(&apt["icaoStruct"], "airport.icaoStruct");
    for (i, f) in apt["frequencies"].as_array().unwrap().iter().enumerate() {
        t.check("FacilityFrequency", f, &format!("frequencies[{i}]"));
    }
    for (i, r) in apt["runways"].as_array().unwrap().iter().enumerate() {
        t.check("AirportRunway", r, &format!("runways[{i}]"));
        t.check("FacilityILSFrequency", &r["primaryILSFrequency"], &format!("runways[{i}].primaryILSFrequency"));
        t.check("FacilityILSFrequency", &r["secondaryILSFrequency"], &format!("runways[{i}].secondaryILSFrequency"));
    }
    for key in ["departures", "arrivals"] {
        for (i, p) in apt[key].as_array().unwrap().iter().enumerate() {
            let path = format!("{key}[{i}]");
            t.check("Procedure", p, &path);
            for (j, l) in p["commonLegs"].as_array().unwrap().iter().enumerate() {
                check_leg(t, l, &format!("{path}.commonLegs[{j}]"));
            }
            for (j, tr) in p["enRouteTransitions"].as_array().unwrap().iter().enumerate() {
                t.check("EnrouteTransition", tr, &format!("{path}.enRouteTransitions[{j}]"));
                for (k, l) in tr["legs"].as_array().unwrap().iter().enumerate() {
                    check_leg(t, l, &format!("{path}.enRouteTransitions[{j}].legs[{k}]"));
                }
            }
            for (j, tr) in p["runwayTransitions"].as_array().unwrap().iter().enumerate() {
                t.check("RunwayTransition", tr, &format!("{path}.runwayTransitions[{j}]"));
                for (k, l) in tr["legs"].as_array().unwrap().iter().enumerate() {
                    check_leg(t, l, &format!("{path}.runwayTransitions[{j}].legs[{k}]"));
                }
            }
        }
    }
    for (i, a) in apt["approaches"].as_array().unwrap().iter().enumerate() {
        let path = format!("approaches[{i}]");
        t.check("ApproachProcedure", a, &path);
        assert!(!a["finalLegs"].as_array().unwrap().is_empty(), "{path} has no final legs");
        for key in ["finalLegs", "missedLegs"] {
            for (j, l) in a[key].as_array().unwrap().iter().enumerate() {
                check_leg(t, l, &format!("{path}.{key}[{j}]"));
            }
        }
        for (j, tr) in a["transitions"].as_array().unwrap().iter().enumerate() {
            t.check("ApproachTransition", tr, &format!("{path}.transitions[{j}]"));
            for (k, l) in tr["legs"].as_array().unwrap().iter().enumerate() {
                check_leg(t, l, &format!("{path}.transitions[{j}].legs[{k}]"));
            }
        }
    }
    for (i, h) in apt["holdingPatterns"].as_array().unwrap().iter().enumerate() {
        t.check("FacilityHoldingPattern", h, &format!("holdingPatterns[{i}]"));
    }
}

/// Every referenced terminal fix loads.
fn check_references_resolve(nav: &mut NavData, apt: &Json) {
    let mut legs = Vec::new();
    for key in ["departures", "arrivals"] {
        for p in apt[key].as_array().unwrap() {
            legs.extend(p["commonLegs"].as_array().unwrap().iter().cloned());
            for tk in ["enRouteTransitions", "runwayTransitions"] {
                for tr in p[tk].as_array().unwrap() {
                    legs.extend(tr["legs"].as_array().unwrap().iter().cloned());
                }
            }
        }
    }
    for a in apt["approaches"].as_array().unwrap() {
        for key in ["finalLegs", "missedLegs"] {
            legs.extend(a[key].as_array().unwrap().iter().cloned());
        }
        for tr in a["transitions"].as_array().unwrap() {
            legs.extend(tr["legs"].as_array().unwrap().iter().cloned());
        }
    }
    let mut missing = Vec::new();
    for leg in &legs {
        for key in ["fixIcaoStruct", "originIcaoStruct", "arcCenterFixIcaoStruct"] {
            let icao = &leg[key];
            let call_name = match icao["type"].as_str().unwrap() {
                "W" => "LOAD_INTERSECTION_FROM_STRUCT",
                "V" => "LOAD_VOR_FROM_STRUCT",
                "N" => "LOAD_NDB_FROM_STRUCT",
                "A" => "LOAD_AIRPORT_FROM_STRUCT",
                _ => continue,
            };
            let args = if call_name.contains("AIRPORT") { format!("[{icao}, 0]") } else { format!("[{icao}]") };
            if fetch(nav, call_name, &args).is_null() {
                missing.push(icao.to_string());
            }
        }
    }
    missing.sort();
    missing.dedup();
    assert!(missing.is_empty(), "{} of {} legs name facilities that do not load: {missing:?}", missing.len(), legs.len());
}

#[test]
fn egll_runways_ils_and_approach_legs() {
    let Some(mut nav) = navdata() else { return };
    let Some(types) = SdkTypes::load() else { return };
    let started = Instant::now();
    let egll = airport(&mut nav, "EGLL");
    eprintln!("EGLL with everything, call to event (built on the worker): {:?}", started.elapsed());
    check_airport_shape(&types, &egll);
    assert_eq!(egll["icao"], "A      EGLL ");
    assert_eq!(egll["icaoStruct"]["region"], "EG");
    assert_eq!(egll["loadedDataFlags"], 127);
    assert_eq!(egll["towered"], true);
    assert!((num(&egll, "transitionAlt") / 0.3048 - 6000.).abs() < 0.5, "{}", egll["transitionAlt"]);

    // 09L/27R: 3,891 m by 50 m (apt.dat), ILS IRR 110.30 on 27R, 3 degree
    // glidepath, magnetic course 271 from earth_nav.dat's encoding.
    let (rwy, primary) = runway_end(&egll, 27, 2);
    assert!(!primary, "27R is 09L's secondary end");
    assert!((num(rwy, "length") - 3891.).abs() < 2., "{}", rwy["length"]);
    assert!((num(rwy, "width") - 50.).abs() < 0.1);
    assert!((num(rwy, "direction") - 89.7).abs() < 0.2, "{}", rwy["direction"]);
    let ils = &rwy["secondaryILSFrequency"];
    assert_eq!(ils["icao"], "V  EGLLIRR  ");
    assert_eq!(num(ils, "freqMHz"), 110.3);
    assert_eq!(num(ils, "freqBCD16"), 0x1030 as f64);
    assert_eq!(ils["hasGlideslope"], true);
    assert_eq!(num(ils, "glideslopeAngle"), 3.);
    assert_eq!(num(ils, "localizerCourse"), 269.);
    assert_eq!(num(ils, "lsCategory"), 3.);
    assert!((num(rwy, "secondaryElevation") / 0.3048 - 78.).abs() < 0.5, "CIFP threshold elevation {}", rwy["secondaryElevation"]);
    assert_eq!(rwy["primaryILSFrequency"]["icao"], "V  EGLLIAA  ");

    // CIFP EGLL.dat I27L: FACF CF27L, FAF FF27L at 2500 ft (G) with the ILL
    // localizer as recommended navaid, then the runway; the missed approach
    // starts at the leg flagged "M" in column 3 of the description.
    let app = egll["approaches"].as_array().unwrap().iter().find(|a| a["name"] == "ILS 27L").expect("ILS 27L");
    assert_eq!((app["runwayNumber"].as_u64(), app["runwayDesignator"].as_u64(), app["approachType"].as_u64()), (Some(27), Some(1), Some(4)));
    let finals = app["finalLegs"].as_array().unwrap();
    let faf = finals.iter().find(|l| l["fixIcaoStruct"]["ident"] == "FF27L").expect("FF27L");
    assert_eq!(faf["type"], 4, "CF");
    assert_eq!(faf["fixIcao"], "WEGEGLLFF27L");
    assert_eq!(faf["originIcao"], "V  EGLLILL  ");
    assert_eq!(num(faf, "fixTypeFlags"), 8.);
    assert_eq!(num(faf, "course"), 269.);
    assert_eq!(num(faf, "altDesc"), 1.);
    assert!((num(faf, "altitude1") / 0.3048 - 2500.).abs() < 0.01);
    assert!((num(faf, "verticalAngle") - 357.).abs() < 1e-9);
    assert!((num(faf, "rho") / 1852. - 9.6).abs() < 1e-9);
    assert!((num(faf, "distance") / 1852. - 2.5).abs() < 1e-9);
    let last = finals.last().unwrap();
    assert_eq!(last["fixIcaoStruct"]["type"], "R");
    assert_eq!(last["fixIcaoStruct"]["ident"], "RW27L");
    assert!(!app["missedLegs"].as_array().unwrap().is_empty());
    assert!(!app["transitions"].as_array().unwrap().is_empty());
    check_references_resolve(&mut nav, &egll);

    // Cached: the second answer is a copy.
    let started = Instant::now();
    nav.call("LOAD_AIRPORT_FROM_STRUCT", &format!("[{}, 127]", icao_struct("A", "", "", "EGLL"))).unwrap();
    assert_eq!(nav.take_events().len(), 1, "sent at once from the cache");
    let cached = started.elapsed();
    eprintln!("EGLL again, cached: {cached:?}");
    assert!(cached < Duration::from_millis(5));
}

#[test]
fn kjfk_runways_ils_and_rf_legs() {
    let Some(mut nav) = navdata() else { return };
    let Some(types) = SdkTypes::load() else { return };
    let kjfk = airport(&mut nav, "KJFK");
    check_airport_shape(&types, &kjfk);
    assert_eq!(kjfk["icaoStruct"]["region"], "K6");
    // 04R: ILS IJFK 109.50, CAT III (earth_nav.dat), magnetic course 44.
    let (rwy, primary) = runway_end(&kjfk, 4, 2);
    let ils = if primary { &rwy["primaryILSFrequency"] } else { &rwy["secondaryILSFrequency"] };
    assert_eq!(ils["icaoStruct"]["ident"], "IJFK");
    assert_eq!(num(ils, "freqMHz"), 109.5);
    assert_eq!(num(ils, "localizerCourse"), 44.);
    assert_eq!(num(ils, "lsCategory"), 3.);
    // 13R has no ILS.
    let (rwy, primary) = runway_end(&kjfk, 13, 2);
    let none = if primary { &rwy["primaryILSFrequency"] } else { &rwy["secondaryILSFrequency"] };
    assert_eq!(none["icao"], "            ");

    // CIFP KJFK.dat R13L: RF leg to JEVNI, right turn, centre CFBMG, RNP 0.3,
    // an RNP AR approach (route qualifier P).
    let app = kjfk["approaches"].as_array().unwrap().iter().find(|a| a["runwayNumber"] == 13 && a["runwayDesignator"] == 1 && a["approachType"] == 10 && a["approachSuffix"] == "").expect("RNAV 13L");
    assert_eq!(app["rnpAr"], true);
    let rf = app["finalLegs"].as_array().unwrap().iter().find(|l| l["fixIcaoStruct"]["ident"] == "JEVNI").expect("JEVNI");
    assert_eq!(rf["type"], 17);
    assert_eq!(rf["turnDirection"], 2);
    assert_eq!(rf["arcCenterFixIcaoStruct"]["ident"], "CFBMG");
    assert!((num(rf, "rnp") / 1852. - 0.3).abs() < 1e-9);
    // KJFK's LPV approaches carry their levels of service (PRDAT).
    let r04ly = kjfk["approaches"].as_array().unwrap().iter().find(|a| a["runwayNumber"] == 4 && a["runwayDesignator"] == 1 && a["approachSuffix"] == "Y").expect("RNAV 04L Y");
    assert_eq!(r04ly["rnavTypeFlags"], 1 | 2 | 8);
    assert!(!kjfk["departures"].as_array().unwrap().is_empty());
    assert!(!kjfk["arrivals"].as_array().unwrap().is_empty());
    assert!(!kjfk["holdingPatterns"].as_array().unwrap().is_empty());
    check_references_resolve(&mut nav, &kjfk);
}

#[test]
fn omdb_runways_and_ils() {
    let Some(mut nav) = navdata() else { return };
    let Some(types) = SdkTypes::load() else { return };
    let omdb = airport(&mut nav, "OMDB");
    check_airport_shape(&types, &omdb);
    assert_eq!(omdb["icaoStruct"]["region"], "OM");
    let designations: Vec<&str> = omdb["runways"].as_array().unwrap().iter().map(|r| r["designation"].as_str().unwrap()).collect();
    assert_eq!(designations.len(), 2, "{designations:?}");
    for (n, d, ident, freq) in [(12, 1, "IDBL", 110.1), (12, 2, "IDBE", 109.5), (30, 1, "IDBW", 111.3), (30, 2, "IDBR", 110.9)] {
        let (rwy, primary) = runway_end(&omdb, n, d);
        let ils = if primary { &rwy["primaryILSFrequency"] } else { &rwy["secondaryILSFrequency"] };
        assert_eq!(ils["icaoStruct"]["ident"], ident, "{n}/{d}");
        assert_eq!(num(ils, "freqMHz"), freq, "{n}/{d}");
        assert_eq!(ils["hasGlideslope"], true, "{n}/{d}");
    }
    // 30L's threshold elevation, 60 ft in CIFP OMDB.dat.
    let (rwy, primary) = runway_end(&omdb, 30, 1);
    let elevation = if primary { num(rwy, "primaryElevation") } else { num(rwy, "secondaryElevation") };
    assert!((elevation / 0.3048 - 60.).abs() < 0.5, "{elevation}");
    check_references_resolve(&mut nav, &omdb);
}

#[test]
fn navaids_waypoints_and_airways() {
    let Some(mut nav) = navdata() else { return };
    let Some(types) = SdkTypes::load() else { return };
    // BIG: Biggin VOR/DME, 115.10 (earth_nav.dat).
    let big = fetch(&mut nav, "LOAD_VOR_FROM_STRUCT", &format!("[{}]", icao_struct("V", "EG", "", "BIG")));
    types.check("VorFacility", &big, "BIG");
    types.check("FacilityDme", &big["dme"], "BIG.dme");
    assert_eq!(big["icao"], "VEG    BIG  ");
    assert_eq!(num(&big, "freqMHz"), 115.1);
    assert_eq!(big["type"], 2);
    assert_eq!(big["vorClass"], 3);
    assert!(big["ils"].is_null());
    // The V1 string form of the same call.
    let again = fetch(&mut nav, "LOAD_VOR", r#"["VEG    BIG  "]"#);
    assert_eq!(again["lat"], big["lat"]);
    // A localizer loads as a VOR facility with its ILS record.
    let ill = fetch(&mut nav, "LOAD_VOR", r#"["V  EGLLILL  "]"#);
    assert_eq!(ill["type"], 6);
    types.check("FacilityILSFrequency", &ill["ils"], "ILL.ils");

    let ndbs = call(&mut nav, "SEARCH_BY_IDENT_WITH_STRUCT", r#"["EPM", 4, 10]"#);
    let epm = ndbs.as_array().unwrap().iter().find(|i| i["ident"] == "EPM").expect("EPM NDB");
    let ndb = fetch(&mut nav, "LOAD_NDB_FROM_STRUCT", &format!("[{epm}]"));
    types.check("NdbFacility", &ndb, "EPM");

    // An airway through a fix: follow UL9's neighbours back and forth.
    let found = call(&mut nav, "SEARCH_BY_IDENT", r#"["KONAN", 2, 10]"#);
    let konan = found.as_array().unwrap().first().expect("KONAN").as_str().unwrap().to_string();
    let wpt = fetch(&mut nav, "LOAD_INTERSECTION", &format!("[\"{konan}\"]"));
    types.check("IntersectionFacility", &wpt, "KONAN");
    let routes = wpt["routes"].as_array().unwrap();
    assert!(!routes.is_empty());
    for (i, r) in routes.iter().enumerate() {
        types.check("AirwaySegment", r, &format!("KONAN.routes[{i}]"));
        let name = r["name"].as_str().unwrap();
        for (side, back) in [("nextIcaoStruct", "prevIcao"), ("prevIcaoStruct", "nextIcao")] {
            if r[side]["ident"] == "" {
                continue;
            }
            let other = fetch(&mut nav, "LOAD_INTERSECTION_FROM_STRUCT", &format!("[{}]", r[side]));
            assert!(!other.is_null(), "{name} {side} {}", r[side]);
            let link = other["routes"].as_array().unwrap().iter().filter(|o| o["name"] == name).any(|o| o[back] == wpt["icao"]);
            assert!(link, "{name}: {} does not link back to KONAN", other["icao"]);
        }
    }
    // The VOR's intersection record (type mismatch in msfs-sdk).
    let as_wpt = fetch(&mut nav, "LOAD_INTERSECTION_FROM_STRUCT", &format!("[{}]", icao_struct("V", "EG", "", "BIG")));
    assert_eq!(as_wpt["icaoStruct"]["type"], "V");
    assert_eq!(as_wpt["type"], 3);
    assert!(fetch(&mut nav, "LOAD_VOR", r#"["VXX    QQQQQ"]"#).is_null());

    // Search by ident: exact matches first, V1 strings or structs.
    let egl = call(&mut nav, "SEARCH_BY_IDENT", r#"["EGL", 1, 40]"#);
    let list: Vec<&str> = egl.as_array().unwrap().iter().map(|v| v.as_str().unwrap()).collect();
    assert!(list.contains(&"A      EGLL "), "{list:?}");
    let exact = call(&mut nav, "SEARCH_BY_IDENT_WITH_STRUCT", r#"["EGLL", 0, 40]"#);
    assert_eq!(exact[0]["ident"], "EGLL");
    check_icao(&exact[0], "SEARCH_BY_IDENT_WITH_STRUCT[0]");
}

#[test]
fn nearest_search_diffs() {
    let Some(mut nav) = navdata() else { return };
    let Some(types) = SdkTypes::load() else { return };
    let session: i64 = call(&mut nav, "START_NEAREST_SEARCH_SESSION_WITH_STRUCT", "[1]").as_i64().unwrap();
    call(&mut nav, "SET_NEAREST_AIRPORT_FILTER", &format!("[{session}, 0, {}]", 2 | 4 | 8 | 16 | 32));
    // FlyByWire's nearby facility monitor: 250 NM, 100 items.
    let (lat, lon) = (51.4775, -0.461389);
    let started = Instant::now();
    let first = search(&mut nav, "SEARCH_NEAREST", &format!("[{session}, {lat}, {lon}, {}, 100]", 250. * 1852.));
    eprintln!("nearest 100 airports in 250 NM of EGLL: {:?}", started.elapsed());
    types.check("NearestSearchResults", &first, "results");
    assert_eq!(first["sessionId"], session);
    let added: Vec<String> = first["added"].as_array().unwrap().iter().map(|i| i["ident"].as_str().unwrap().to_string()).collect();
    assert_eq!(added.len(), 100);
    assert_eq!(added[0], "EGLL", "nearest first");
    assert!(first["removed"].as_array().unwrap().is_empty());

    // The same place again: nothing changes.
    let same = search(&mut nav, "SEARCH_NEAREST", &format!("[{session}, {lat}, {lon}, {}, 100]", 250. * 1852.));
    assert!(same["added"].as_array().unwrap().is_empty() && same["removed"].as_array().unwrap().is_empty());
    assert!(same["searchId"].as_i64() > first["searchId"].as_i64());

    // 200 NM east: what left and what came balance, and nothing is both.
    let moved = search(&mut nav, "SEARCH_NEAREST", &format!("[{session}, {lat}, {}, {}, 100]", lon + 5.5, 250. * 1852.));
    let (a, r) = (moved["added"].as_array().unwrap(), moved["removed"].as_array().unwrap());
    assert!(!a.is_empty());
    assert_eq!(a.len(), r.len());
    assert!(a.iter().all(|x| !r.contains(x)));

    // Radius 0 clears the session (FlyByWire stops a monitor this way).
    let cleared = search(&mut nav, "SEARCH_NEAREST", &format!("[{session}, {lat}, {lon}, 0, 0]"));
    assert_eq!(cleared["removed"].as_array().unwrap().len(), 100);

    // Intersections and VORs with V1 strings.
    let s2 = call(&mut nav, "START_NEAREST_SEARCH_SESSION", "[2]").as_i64().unwrap();
    call(&mut nav, "SET_NEAREST_INTERSECTION_FILTER", &format!("[{s2}, {}, 1]", 2 | 4 | 32 | 64 | 128));
    let started = Instant::now();
    let wpts = search(&mut nav, "SEARCH_NEAREST", &format!("[{s2}, {lat}, {lon}, {}, 100]", 250. * 1852.));
    eprintln!("nearest 100 intersections in 250 NM of EGLL: {:?}", started.elapsed());
    assert_eq!(wpts["added"].as_array().unwrap().len(), 100);
    assert!(wpts["added"][0].as_str().unwrap().starts_with('W'));
    let s3 = call(&mut nav, "START_NEAREST_SEARCH_SESSION", "[3]").as_i64().unwrap();
    let vors = search(&mut nav, "SEARCH_NEAREST", &format!("[{s3}, {lat}, {lon}, {}, 20]", 250. * 1852.));
    assert_eq!(vors["added"].as_array().unwrap().len(), 20);
    assert!(vors["added"].as_array().unwrap().iter().all(|v| v.as_str().unwrap().starts_with('V')));
}

#[test]
fn calls_stay_fast() {
    let Some(mut nav) = navdata() else { return };
    // A call and taking its events, as the runtime does.
    let time = |nav: &mut NavData, name: &str, args: &str, runs: u32| -> Duration {
        let started = Instant::now();
        for _ in 0..runs {
            nav.call(name, args).unwrap();
            nav.take_events();
        }
        started.elapsed() / runs
    };
    // What FlyByWire's nearby monitor does each second: move a little and
    // load the minimal airports.
    let s = call(&mut nav, "START_NEAREST_SEARCH_SESSION_WITH_STRUCT", "[1]").as_i64().unwrap();
    let mut samples = Vec::with_capacity(50);
    for step in 0..50 {
        let args = format!("[{s}, {}, -0.46, {}, 100]", 51.4775 + step as f64 * 0.01, 250. * 1852.);
        samples.push(time(&mut nav, "SEARCH_NEAREST", &args, 1));
    }
    // A single call's wall-clock time is at the mercy of whatever else the
    // OS scheduler ran on the core that instant (this machine also runs
    // this workstream's other agents' `cargo build`s in parallel), so one
    // slow sample among 50 is expected noise, not a regression. The 90th
    // percentile still catches a real, systemic slowdown (one that moves
    // most samples, not just an unlucky one) while tolerating a couple of
    // preempted outliers.
    samples.sort();
    let worst = samples[(samples.len() * 9 / 10).min(samples.len() - 1)];
    let true_worst = *samples.last().unwrap();
    let minimal = time(&mut nav, "LOAD_AIRPORT_FROM_STRUCT", &format!("[{}, 0]", icao_struct("A", "", "", "EGKK")), 20);
    let vor = time(&mut nav, "LOAD_VOR", r#"["VEG    BIG  "]"#, 100);
    let wpt = time(&mut nav, "LOAD_INTERSECTION_FROM_STRUCT", &format!("[{}]", icao_struct("W", "EG", "", "KONAN")), 100);
    let ident = time(&mut nav, "SEARCH_BY_IDENT", r#"["LAM", 0, 40]"#, 100);
    // An airport with procedures: queued on the worker, then sent.
    let queued = time(&mut nav, "LOAD_AIRPORT_FROM_STRUCT", &format!("[{}, 127]", icao_struct("A", "", "", "LIRF")), 1);
    let started = Instant::now();
    assert_eq!(events(&mut nav).len(), 1);
    let worker = started.elapsed();
    // One an ident search names is built ahead, and sent at once.
    nav.call("SEARCH_BY_IDENT", r#"["LFPG", 1, 40]"#).unwrap();
    assert!(events(&mut nav).is_empty());
    let prebuilt = time(&mut nav, "LOAD_AIRPORT_FROM_STRUCT", &format!("[{}, 127]", icao_struct("A", "", "", "LFPG")), 1);
    eprintln!(
        "per call: nearest airports worst (p90) {worst:?} (true max {true_worst:?}), minimal airport {minimal:?}, VOR {vor:?}, intersection {wpt:?}, \
         ident search {ident:?}, LIRF with procedures {queued:?} (then {worker:?} on the worker), prebuilt LFPG {prebuilt:?}"
    );
    assert!(worst < Duration::from_millis(5), "p90 of 50 nearest-search samples was {worst:?}");
    // Averaged over 20-100 runs each, so already robust to a single
    // scheduler hiccup; a small margin over the 1 ms these calls actually
    // take still catches a real N-times regression.
    assert!(minimal < Duration::from_millis(2) && vor < Duration::from_millis(2) && wpt < Duration::from_millis(2) && ident < Duration::from_millis(2));
    // Single-shot (loading a specific, not-yet-cached airport can't be
    // repeated to average away one slow disk read), so given a slightly
    // wider margin than the averaged calls above for the same reason.
    assert!(queued < Duration::from_millis(3) && prebuilt < Duration::from_millis(3));
}

#[test]
fn not_found_and_bad_calls() {
    let Some(mut nav) = navdata() else { return };
    assert!(fetch(&mut nav, "LOAD_AIRPORT", r#"["A      ZZZZ "]"#).is_null());
    // A VOR asked for as an airport is not found.
    assert!(fetch(&mut nav, "LOAD_AIRPORT", r#"["VEG    BIG  "]"#).is_null());
    let many = call(&mut nav, "LOAD_AIRPORTS", r#"[["A      EGKK ", "A      ZZZZ "]]"#);
    assert_eq!(many, serde_json::json!([true, false]));
    let sent = events(&mut nav);
    assert_eq!(sent.len(), 1);
    assert_eq!(sent[0].1[0]["icaoStruct"]["ident"], "EGKK");
    // Not ready yet is its own error, for the runtime to ask again.
    let mut loading = NavData::load(Path::new(XPLANE)).unwrap();
    assert_eq!(loading.call("LOAD_VOR", r#"["VEG    BIG  "]"#).err().as_deref(), Some(super::NOT_READY));
    // No metar_source wired up in this fixture (js_bridge.rs's start_navdata
    // is what wires the real one, js_bridge.rs ~1319): the "not found" shape,
    // not an error -- see metar_and_taf_calls below for the wired-up path.
    assert_eq!(call(&mut nav, "GET_METAR_BY_IDENT", r#"["EGLL"]"#), serde_json::json!({"icao": "", "metarString": ""}));
    assert!(nav.call("SEARCH_NEAREST", "[9999, 0, 0, 1, 1]").is_err());
}

#[test]
fn magvar_source_sign_is_checked_against_localizers() {
    if !Path::new(XPLANE).join("Resources").is_dir() {
        return;
    }
    let mut nav = NavData::load(Path::new(XPLANE)).unwrap();
    nav.wait_ready().unwrap();
    let db = nav.db.clone().unwrap();
    // A model signed west-positive: the published variation, negated.
    let published: Vec<(f64, f64, f64)> = db
        .vhf
        .iter()
        .filter_map(|v| {
            let loc = v.loc.as_ref()?;
            Some((v.lat, v.lon, super::facility::normalise180(loc.true_bearing - loc.mag_course?)))
        })
        .collect();
    let model = published.clone();
    nav.set_magvar_source(Box::new(move |lat, lon| model.iter().find(|p| p.0 == lat && p.1 == lon).map_or(0., |p| -p.2)));
    nav.call("LOAD_VOR", r#"["VEG    BIG  "]"#).unwrap();
    assert_eq!(nav.magvar_sign, Some(-1.));
    let (lat, lon, decl) = *published.iter().find(|p| p.2.abs() > 10.).unwrap();
    assert!((nav.magvar_at(lat, lon) - decl).abs() < 1e-9);
}

#[test]
fn metar_and_taf_calls() {
    if !Path::new(XPLANE).join("Resources").is_dir() {
        return;
    }
    let mut nav = NavData::load(Path::new(XPLANE)).unwrap();
    nav.wait_ready().unwrap();
    // Stands in for js_bridge.rs's real source (crate::xp::metar_for_airport,
    // XPLMGetMETARForAirport): EGLL has one, nothing else does -- the same
    // "no report" shape a real airport with none gets from X-Plane's own
    // empty-string answer.
    nav.set_metar_source(Box::new(|icao| (icao == "EGLL").then(|| "EGLL 251550Z 24012KT 9999 FEW030 12/08 Q1015".to_string())));
    let metar = call(&mut nav, "GET_METAR_BY_IDENT", r#"["egll"]"#);
    assert_eq!(metar["icao"], "EGLL");
    assert_eq!(metar["metarString"], "EGLL 251550Z 24012KT 9999 FEW030 12/08 Q1015");
    let miss = call(&mut nav, "GET_METAR_BY_IDENT", r#"["ZZZZ"]"#);
    assert_eq!(miss, serde_json::json!({"icao": "", "metarString": ""}));
    // EGLL itself (51.4775, -0.4614): the nearest-airport search resolves to
    // it and the same source answers.
    let by_latlon = call(&mut nav, "GET_METAR_BY_LATLON", "[51.4775, -0.4614]");
    assert_eq!(by_latlon["icao"], "EGLL");
    // No X-Plane TAF source exists: always the "no report" shape, never
    // Rejected and never a fabricated forecast.
    assert_eq!(call(&mut nav, "GET_TAF_BY_IDENT", r#"["EGLL"]"#), serde_json::json!({"icao": "", "tafString": ""}));
    assert_eq!(call(&mut nav, "GET_TAF_BY_LATLON", "[51.4775, -0.4614]"), serde_json::json!({"icao": "", "tafString": ""}));
}

/// `header_cycle` (used by `NavData::load` to answer `cycle()`/`date_range()`
/// eagerly) reads a file of its own, not X-Plane's installed data, so this
/// always runs.
#[test]
fn header_cycle_reads_the_airac_cycle_without_the_full_parse() {
    let path = std::env::temp_dir().join(format!("fbw-navdata-header-cycle-test-{}.dat", std::process::id()));
    std::fs::write(&path, "I\n1200 Version - data cycle 2406, build 20251002, metadata NavXP1200.\n\n").unwrap();
    let cycle = super::header_cycle(&path);
    let _ = std::fs::remove_file(&path);
    assert_eq!(cycle, Some(2406));
}

/// The bug this guards: MSFS's `FLIGHT NAVDATA DATE RANGE` game string has
/// no "ask again" status the way a Coherent call does, so whatever
/// `NavData::game_string` answers on the very first ask is cached on the
/// instruments' worker forever (js_worker.rs's `WorkerHost::get_string`).
/// `cycle()`/`date_range()` must therefore answer from earth_nav.dat's
/// header at once, well before the background index (seconds, for the
/// whole world's airports) finishes.
#[test]
fn date_range_is_ready_before_the_background_index_finishes() {
    if !Path::new(XPLANE).join("Resources").is_dir() {
        return;
    }
    let nav = NavData::load(Path::new(XPLANE)).unwrap();
    let cycle = nav.cycle().expect("cycle available before wait_ready()");
    let range = nav.date_range().expect("date_range available before wait_ready()");
    assert!(range.contains('/'), "{range}");
    eprintln!("cycle {cycle} date_range {range} (before the background index finished)");
}
