//! Which of FlyByWire's eight start states X-Plane's situation is.
//!
//! In MSFS the flight file MSFS loads for the spawn sets `A32NX_START_STATE`
//! (apron.FLT:398 `=2`, hangar.flt:375 `=1`, taxi.flt:435 `=3`,
//! runway.FLT:463 `=4`, cruise.FLT:447 `=6`, final.FLT:448 `=8`), the systems
//! glue reads it before building the aircraft (systems_wasm lib.rs:58-66) and
//! passes it to `Simulation::new`. X-Plane has no flight files, so the state
//! is worked out from X-Plane's situation when the plugin starts.
//!
//! What the systems do with it (systems simulation/mod.rs:137-167): the
//! ground states (Hangar, Apron, Taxi, Runway) start on the ground, the air
//! states (Climb, Cruise, Approach, Final) in flight; every state but Hangar
//! and Apron starts with the engines running; only Final (in the air) starts
//! with the gear down. The PRIM's FCU initialisation keys off >= 5 and == 4
//! (FlyByWireInterface.cpp handleFcuInitialization, prim.rs:298-320).
//!
//! The rule, first match wins:
//!
//! On the ground (`onground_any`):
//! 1. no engine running -> Apron. X-Plane has no hangar spawn, and the
//!    systems treat Hangar and Apron alike (both on the ground with engines
//!    off); FlyByWire's hangar.flt differs from apron.FLT in the battery
//!    switch (hangar.flt:212 `BatterySwitch=False`, apron.FLT:218 `True`)
//!    and X-Plane's cold starts always have the battery off, so the battery
//!    cannot tell a hangar from a gate. Hangar is chosen by the override.
//! 2. engines running and lined up on a runway (inside an ILS/localizer
//!    runway's centreline strip, heading within 30 degrees of it) or rolling
//!    faster than 40 kt (only a take-off or landing roll is that fast on the
//!    ground) -> Runway.
//! 3. engines running -> Taxi.
//!
//! In the air:
//! 4. gear down -> Final: only Final starts the gear down
//!    (`start_gear_down`, simulation/mod.rs:141-143; final.FLT:578-581 gear
//!    100, flaps full), so a down gear must be Final for FlyByWire's gear to
//!    agree with X-Plane's.
//! 5. climbing faster than 250 ft/min -> Climb.
//! 6. flaps extended, or descending faster than 250 ft/min below 10 000 ft
//!    above ground or within 50 nm of the FMS destination -> Approach.
//! 7. otherwise -> Cruise (also FlyByWire's default, simulation/mod.rs:66-78).
//!
//! An override: `Output/preferences/fbw_a380x_start_state.txt` holding a
//! number 1-8 or a state name forces that state (and is the only way to
//! choose Hangar).
//!
//! Note: FlyByWire's own Climb.flt and approach.FLT do not set
//! A32NX_START_STATE, so in MSFS those spawns read 0, which the systems take
//! as Cruise; here they get their own number, which only differs from
//! Cruise in `StartState` itself (nothing in the systems or the PRIM port
//! tells 5 or 7 from 6).

use std::ffi::{c_char, c_int, c_void, CString};
use std::path::PathBuf;

use systems::simulation::{SimulatorReaderWriter, StartState, VariableRegistry};

use crate::xp::Xplm;

/// What the rule looks at.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Situation {
    pub on_ground: bool,
    pub engines_running: bool,
    pub battery_on: bool,
    pub ground_speed_kt: f64,
    pub agl_ft: f64,
    pub vertical_speed_fpm: f64,
    pub gear_down: bool,
    pub flaps_extended: bool,
    pub on_runway: bool,
    pub destination_nm: Option<f64>,
}

pub fn classify(s: &Situation, override_state: Option<StartState>) -> StartState {
    if let Some(state) = override_state {
        return state;
    }
    if s.on_ground {
        if !s.engines_running {
            StartState::Apron
        } else if s.on_runway || s.ground_speed_kt > 40. {
            StartState::Runway
        } else {
            StartState::Taxi
        }
    } else if s.gear_down {
        StartState::Final
    } else if s.vertical_speed_fpm > 250. {
        StartState::Climb
    } else if s.flaps_extended
        || (s.vertical_speed_fpm < -250. && (s.agl_ft < 10_000. || s.destination_nm.is_some_and(|d| d < 50.)))
    {
        StartState::Approach
    } else {
        StartState::Cruise
    }
}

pub fn parse_override(text: &str) -> Option<StartState> {
    let t = text.trim().to_ascii_lowercase();
    if let Ok(n) = t.parse::<f64>() {
        return (1. ..9.).contains(&n).then(|| StartState::from(n));
    }
    Some(match t.as_str() {
        "hangar" => StartState::Hangar,
        "apron" => StartState::Apron,
        "taxi" => StartState::Taxi,
        "runway" => StartState::Runway,
        "climb" => StartState::Climb,
        "cruise" => StartState::Cruise,
        "approach" => StartState::Approach,
        "final" => StartState::Final,
        _ => return None,
    })
}

pub fn override_path() -> PathBuf {
    PathBuf::from("Output").join("preferences").join("fbw_a380x_start_state.txt")
}

/// Great-circle distance in metres.
pub fn distance_m(lat1: f64, lon1: f64, lat2: f64, lon2: f64) -> f64 {
    let (p1, p2) = (lat1.to_radians(), lat2.to_radians());
    let dp = p2 - p1;
    let dl = (lon2 - lon1).to_radians();
    let a = (dp / 2.).sin().powi(2) + p1.cos() * p2.cos() * (dl / 2.).sin().powi(2);
    2. * 6_371_000. * a.sqrt().asin()
}

/// Initial bearing from 1 to 2, degrees true.
pub fn bearing_deg(lat1: f64, lon1: f64, lat2: f64, lon2: f64) -> f64 {
    let (p1, p2) = (lat1.to_radians(), lat2.to_radians());
    let dl = (lon2 - lon1).to_radians();
    let y = dl.sin() * p2.cos();
    let x = p1.cos() * p2.sin() - p1.sin() * p2.cos() * dl.cos();
    (y.atan2(x).to_degrees() + 360.) % 360.
}

fn angle_diff(a: f64, b: f64) -> f64 {
    let d = (a - b).rem_euclid(360.);
    d.min(360. - d)
}

/// Whether the aircraft stands on the runway a localizer serves. The
/// localizer antenna sits beyond the far end of its runway on the extended
/// centreline, its course the runway heading: an aircraft on that runway is
/// behind the antenna within a few kilometres, within half a runway width
/// plus margin of the centreline.
pub fn on_localizer_runway(ac_lat: f64, ac_lon: f64, ac_heading: f64, loc_lat: f64, loc_lon: f64, loc_course: f64) -> bool {
    let d = distance_m(loc_lat, loc_lon, ac_lat, ac_lon);
    if !(50. ..=6_000.).contains(&d) {
        return false;
    }
    // Bearing from the antenna back to the aircraft, against the reciprocal
    // of the course.
    let off = angle_diff(bearing_deg(loc_lat, loc_lon, ac_lat, ac_lon), loc_course + 180.);
    let cross_track = d * off.to_radians().sin();
    off < 90. && cross_track.abs() < 45. && angle_diff(ac_heading, loc_course) < 30.
}

extern "system" {
    fn LoadLibraryA(name: *const c_char) -> *mut c_void;
    fn GetProcAddress(module: *mut c_void, name: *const c_char) -> *mut c_void;
}

fn xplm_symbol(name: &str) -> Option<*mut c_void> {
    unsafe {
        let module_name = CString::new("XPLM_64.dll").ok()?;
        let module = LoadLibraryA(module_name.as_ptr());
        if module.is_null() {
            return None;
        }
        let name = CString::new(name).ok()?;
        let p = GetProcAddress(module, name.as_ptr());
        (!p.is_null()).then_some(p)
    }
}

type NavByType = unsafe extern "C" fn(c_int) -> c_int;
type NavInfo = unsafe extern "C" fn(
    c_int,
    *mut c_int,
    *mut f32,
    *mut f32,
    *mut f32,
    *mut c_int,
    *mut f32,
    *mut c_char,
    *mut c_char,
    *mut c_char,
);
type NoArgInt = unsafe extern "C" fn() -> c_int;
type FmsEntryInfo = unsafe extern "C" fn(c_int, *mut c_int, *mut c_char, *mut c_int, *mut c_int, *mut f32, *mut f32);

const NAV_ILS: c_int = 8;
const NAV_LOCALIZER: c_int = 16;

/// Whether X-Plane's navigation data has a localizer whose runway the
/// aircraft is on.
fn on_runway_by_navdata(lat: f64, lon: f64, heading: f64) -> bool {
    let (Some(first), Some(last), Some(info)) = (
        xplm_symbol("XPLMFindFirstNavAidOfType"),
        xplm_symbol("XPLMFindLastNavAidOfType"),
        xplm_symbol("XPLMGetNavAidInfo"),
    ) else {
        return false;
    };
    let (first, last, info) = unsafe {
        (
            std::mem::transmute::<*mut c_void, NavByType>(first),
            std::mem::transmute::<*mut c_void, NavByType>(last),
            std::mem::transmute::<*mut c_void, NavInfo>(info),
        )
    };
    for kind in [NAV_ILS, NAV_LOCALIZER] {
        let (a, b) = unsafe { (first(kind), last(kind)) };
        if a < 0 || b < a {
            continue;
        }
        for r in a..=b {
            let (mut t, mut la, mut lo, mut hd) = (0, 0f32, 0f32, 0f32);
            unsafe { info(r, &mut t, &mut la, &mut lo, std::ptr::null_mut(), std::ptr::null_mut(), &mut hd, std::ptr::null_mut(), std::ptr::null_mut(), std::ptr::null_mut()) };
            if t != kind {
                continue;
            }
            // X-Plane stores an ILS course as true heading; some data packs
            // add the glide slope angle times 1000 (e.g. 300254.2).
            let course = (hd as f64).rem_euclid(1000.);
            if (la as f64 - lat).abs() < 0.1
                && (lo as f64 - lon).abs() < 0.1 / lat.to_radians().cos().max(0.1)
                && on_localizer_runway(lat, lon, heading, la as f64, lo as f64, course)
            {
                return true;
            }
        }
    }
    false
}

fn destination_nm(lat: f64, lon: f64) -> Option<f64> {
    let count = unsafe { std::mem::transmute::<*mut c_void, NoArgInt>(xplm_symbol("XPLMCountFMSEntries")?)() };
    if count <= 0 {
        return None;
    }
    let dest = unsafe { std::mem::transmute::<*mut c_void, NoArgInt>(xplm_symbol("XPLMGetDestinationFMSEntry")?)() };
    let info = unsafe { std::mem::transmute::<*mut c_void, FmsEntryInfo>(xplm_symbol("XPLMGetFMSEntryInfo")?) };
    let (mut t, mut la, mut lo) = (0, 0f32, 0f32);
    unsafe { info(dest.clamp(0, count - 1), &mut t, std::ptr::null_mut(), std::ptr::null_mut(), std::ptr::null_mut(), &mut la, &mut lo) };
    // Type 0 is xplm_FMS_Unknown: an empty entry.
    (t != 0 && (la != 0. || lo != 0.)).then(|| distance_m(lat, lon, la as f64, lo as f64) / 1852.)
}

/// Read X-Plane's situation.
pub fn read_situation(xplm: &Xplm) -> Situation {
    let f = |name: &str| xplm.find(name).map_or(0., |d| xplm.get_f(d) as f64);
    let i = |name: &str| xplm.find(name).map_or(0, |d| xplm.get_i(d));
    let d = |name: &str| xplm.find(name).map_or(0., |d| xplm.get_d(d));
    let any_i = |name: &str| {
        xplm.find(name).is_some_and(|d| {
            let mut v = [0; 16];
            let n = xplm.get_vi(d, &mut v);
            v[..n].iter().any(|x| *x != 0)
        })
    };
    let any_f = |name: &str, above: f32| {
        xplm.find(name).is_some_and(|d| {
            let mut v = [0f32; 16];
            let n = xplm.get_vf(d, &mut v);
            v[..n].iter().any(|x| *x > above)
        })
    };
    let (lat, lon, heading) = (
        d("sim/flightmodel/position/latitude"),
        d("sim/flightmodel/position/longitude"),
        f("sim/flightmodel/position/psi"),
    );
    let agl_ft = f("sim/flightmodel/position/y_agl") * 3.280_84;
    // Before X-Plane has placed the aircraft its position reads garbage
    // (millions of feet below ground); treat that as parked.
    let position_valid = agl_ft.is_finite() && (-1_000.0..=100_000.0).contains(&agl_ft);
    let on_ground = !position_valid || i("sim/flightmodel/failures/onground_any") != 0;
    let engines_running = any_i("sim/flightmodel/engine/ENGN_running");
    Situation {
        on_ground,
        engines_running,
        battery_on: any_i("sim/cockpit2/electrical/battery_on"),
        ground_speed_kt: f("sim/flightmodel/position/groundspeed") * 1.943_844,
        agl_ft: if position_valid { agl_ft } else { 0.0 },
        vertical_speed_fpm: f("sim/flightmodel/position/vh_ind_fpm"),
        gear_down: i("sim/cockpit2/controls/gear_handle_down") != 0
            || any_f("sim/flightmodel2/gear/deploy_ratio", 0.5),
        flaps_extended: f("sim/cockpit2/controls/flap_handle_request_ratio") > 0.01,
        on_runway: on_ground && engines_running && on_runway_by_navdata(lat, lon, heading),
        destination_nm: if on_ground { None } else { destination_nm(lat, lon) },
    }
}

pub fn name(state: StartState) -> &'static str {
    match state {
        StartState::Hangar => "hangar",
        StartState::Apron => "apron",
        StartState::Taxi => "taxi",
        StartState::Runway => "runway",
        StartState::Climb => "climb",
        StartState::Cruise => "cruise",
        StartState::Approach => "approach",
        StartState::Final => "final",
    }
}

/// Decide the start state from X-Plane, write `A32NX_START_STATE` as the
/// flight file would, and log why.
pub fn detect<V: VariableRegistry + SimulatorReaderWriter>(xplm: &Xplm, vars: &mut V) -> StartState {
    let situation = read_situation(xplm);
    let forced = std::fs::read_to_string(override_path()).ok().and_then(|t| parse_override(&t));
    let state = classify(&situation, forced);
    let id = vars.get("START_STATE".to_owned());
    vars.write(&id, state.into());
    crate::log(&format!(
        "start state {} ({}){}: {:?}",
        f64::from(state),
        name(state),
        if forced.is_some() { " from the override file" } else { "" },
        situation
    ));
    state
}

#[cfg(test)]
mod tests {
    use super::*;

    fn ground(engines: bool) -> Situation {
        Situation { on_ground: true, engines_running: engines, ..Default::default() }
    }

    fn air(vs: f64, agl: f64) -> Situation {
        Situation { vertical_speed_fpm: vs, agl_ft: agl, ..Default::default() }
    }

    #[test]
    fn ground_states() {
        assert_eq!(classify(&ground(false), None), StartState::Apron);
        assert_eq!(classify(&ground(true), None), StartState::Taxi);
        assert_eq!(classify(&Situation { on_runway: true, ..ground(true) }, None), StartState::Runway);
        assert_eq!(classify(&Situation { ground_speed_kt: 80., ..ground(true) }, None), StartState::Runway);
        // Cold on a runway is still a cold aircraft.
        assert_eq!(classify(&Situation { on_runway: true, ..ground(false) }, None), StartState::Apron);
        assert_eq!(classify(&ground(false), Some(StartState::Hangar)), StartState::Hangar);
    }

    #[test]
    fn air_states() {
        assert_eq!(classify(&air(0., 35_000.), None), StartState::Cruise);
        assert_eq!(classify(&air(2_000., 3_000.), None), StartState::Climb);
        assert_eq!(classify(&air(-1_500., 35_000.), None), StartState::Cruise);
        assert_eq!(classify(&air(-1_500., 8_000.), None), StartState::Approach);
        assert_eq!(
            classify(&Situation { destination_nm: Some(30.), ..air(-1_500., 20_000.) }, None),
            StartState::Approach
        );
        assert_eq!(classify(&Situation { flaps_extended: true, ..air(0., 3_000.) }, None), StartState::Approach);
        // Flaps out after take-off, still climbing.
        assert_eq!(classify(&Situation { flaps_extended: true, ..air(1_500., 1_500.) }, None), StartState::Climb);
        assert_eq!(classify(&Situation { gear_down: true, ..air(-700., 1_500.) }, None), StartState::Final);
        assert_eq!(classify(&Situation { gear_down: true, ..air(1_500., 500.) }, None), StartState::Final);
    }

    #[test]
    fn the_numbers_are_flybywires() {
        let states = [
            StartState::Hangar,
            StartState::Apron,
            StartState::Taxi,
            StartState::Runway,
            StartState::Climb,
            StartState::Cruise,
            StartState::Approach,
            StartState::Final,
        ];
        for (i, s) in states.into_iter().enumerate() {
            assert_eq!(f64::from(s), (i + 1) as f64);
            assert_eq!(parse_override(&format!("{}\n", i + 1)), Some(s));
            assert_eq!(parse_override(name(s)), Some(s));
        }
        assert_eq!(parse_override("0"), None);
        assert_eq!(parse_override("nonsense"), None);
    }

    #[test]
    fn lined_up_on_a_localizer_runway() {
        // KSEA 16L-ish: a localizer at the south end, course 180 true; the
        // aircraft 3 km north of it, on the centreline, heading south.
        let (loc_lat, loc_lon) = (47.4300, -122.3080);
        let north = 3_000. / 111_195.;
        assert!(on_localizer_runway(loc_lat + north, loc_lon, 180., loc_lat, loc_lon, 180.));
        // Facing the other way.
        assert!(!on_localizer_runway(loc_lat + north, loc_lon, 0., loc_lat, loc_lon, 180.));
        // 100 m off the centreline: a parallel taxiway.
        let east = 100. / (111_195. * loc_lat.to_radians().cos());
        assert!(!on_localizer_runway(loc_lat + north, loc_lon + east, 180., loc_lat, loc_lon, 180.));
        // Past the antenna, on the approach side of nothing.
        assert!(!on_localizer_runway(loc_lat - north, loc_lon, 180., loc_lat, loc_lon, 180.));
        // Too far.
        assert!(!on_localizer_runway(loc_lat + 10. * north, loc_lon, 180., loc_lat, loc_lon, 180.));
    }

    /// Systems-runtime sanity harness (debug.md's whole-aircraft pass): runs
    /// FlyByWire's real A380 simulation, through this plugin's own
    /// `aspects.rs` glue exactly as the plugin ticks it, for two minutes of
    /// a cold-and-dark apron start with no input at all, then with the
    /// battery/APU pushbuttons and an APU start commanded. After every tick,
    /// every published variable is checked for NaN/infinity and for a small
    /// set of generic "this cannot be right" bounds (temperatures,
    /// pressures, N%/percentage-shaped variables), matched by name since
    /// there is no single catalogue of every variable's unit -- and a few
    /// specific variables are checked against what a cold, unpowered
    /// aircraft must show (ADIRS not aligned, APU not running, no bleed
    /// air), so a state that contradicts a cold aircraft fails loudly
    /// instead of silently.
    ///
    /// This exercises FlyByWire's own ported systems (electrical, APU,
    /// pneumatics, ADIRS state machine) plus this crate's `aspects.rs`
    /// bridging layer; it does not exercise this crate's own gas-turbine
    /// engine replacement (`physics::engine`, no equivalent inside
    /// `a380_systems` to drive here) or any module that only runs against
    /// the concrete XPLM-backed `Vars`/`Xplm` (fuel.rs, physics::air,
    /// physics::hydraulics, physics::adirs's own glue) -- those have no
    /// offline construction path without a live X-Plane process (see the
    /// workstream report). `physics::engine::tests` has the equivalent
    /// long-run sanity harness for the engine model itself, including a
    /// cold-and-dark-then-one-engine-start scenario.
    #[test]
    fn a_cold_and_dark_apron_start_then_apu_start_never_produce_nan_runaway_or_a_state_a_cold_aircraft_could_not_be_in() {
        use crate::aspects::test_vars::TestVars;
        use a380_systems::A380;
        use systems::simulation::Simulation;

        fn check_every_variable(vars: &TestVars, phase: &str) {
            for (name, &i) in &vars.index {
                let v = vars.values[i];
                assert!(v.is_finite(), "{phase}: {name} is {v} (not finite)");
                let upper = name.to_ascii_uppercase();
                // ARINC429-encoded words (raw SSM+data bit patterns, e.g.
                // `APU_EGT_CAUTION`/`APU_EGT_WARNING`'s threshold words) are
                // not engineering-unit values at all, so the shape-by-name
                // heuristics below would misread their encoded bit pattern
                // as, say, a temperature in the billions; skip them.
                let is_ratio_or_ssm = upper.contains("_SSM")
                    || upper.contains("RATIO")
                    || upper.contains("NORMAL")
                    || upper.contains("CAUTION")
                    || upper.contains("WARNING");
                if (upper.contains("TEMP") || upper.contains("EGT")) && !is_ratio_or_ssm {
                    // Celsius, generous either side of anything an engine,
                    // APU, brake or cabin could show cold or lit.
                    assert!(v > -90. && v < 1_200., "{phase}: {name} = {v} is a runaway temperature");
                }
                if upper.contains("PRESSURE") && !is_ratio_or_ssm {
                    // A transducer with no signal (unpowered, e.g. on a cold
                    // aircraft with no bleed source) writes exactly -1 as
                    // FlyByWire's own "no data" sentinel, not a physical
                    // reading: `EngineBleedAirSystem::write`
                    // (a380_systems/src/pneumatic.rs) does
                    // `.map_or(-1., |p| p.get::<psi>())` for every
                    // `*_TRANSDUCER_PRESSURE`. Any other negative pressure
                    // is still a runaway.
                    let is_no_data_sentinel = upper.contains("TRANSDUCER") && v == -1.0;
                    // Pascals or psi, whichever the variable uses: never
                    // negative, never past a hydraulic relief valve's own
                    // ceiling by orders of magnitude.
                    assert!(
                        is_no_data_sentinel || (v > -1.0 && v < 1.0e7),
                        "{phase}: {name} = {v} is a runaway pressure"
                    );
                }
                let looks_like_a_speed_or_percentage = (upper.ends_with("_N1") || upper.ends_with("_N2") || upper.ends_with("_N3")
                    || upper.contains("_N_RAW") || upper.ends_with("_PCT") || upper.ends_with("_PERCENT")
                    || upper.ends_with("_PERCENTAGE"))
                    && !is_ratio_or_ssm;
                if looks_like_a_speed_or_percentage {
                    assert!(v > -10. && v < 130., "{phase}: {name} = {v} is a runaway speed/percentage");
                }
            }
        }

        let mut vars = TestVars::default();
        // A parked aircraft's weight is never actually zero (X-Plane always
        // reports at least the empty weight); this stands in for what
        // `weight_balance.rs`/`sensors.rs` always keep populated in the
        // running plugin, so `UpdateContext::total_weight()` (`TOTAL
        // WEIGHT`) isn't a bare 0 -- FlyByWire's own wing-flex ground-weight
        // ratio divides by it (`wing_flex.rs`, `total_weight_on_wheels /
        // context.total_weight()`), and a genuinely zero weight there is a
        // 0/0 = NaN this harness would otherwise wrongly blame on this
        // plugin's own glue for.
        vars.set("TOTAL WEIGHT", 600_000.0);
        // FlyByWire's own `SimulationTestBed::new_with_start_state` (the
        // harness every one of their own systems tests runs through,
        // fbw-common systems/src/simulation/test.rs:263-269) always seeds
        // exactly these two before ticking anything: ambient_pressure
        // 29.92 inHg, ambient_temperature 0 C. This harness builds
        // `Simulation` directly instead of going through their `TestBed`
        // (module docs above), so nothing here ever set them, and
        // `TestVars` (aspects.rs) defaults every unset variable to a bare
        // 0 -- AMBIENT PRESSURE reading 0 is a vacuum, a condition the
        // ported electrical/air-conditioning/pneumatic code was never
        // written to divide by (see e.g. `air_cycle_machine.rs`'s
        // `rho_ambient = ambient_pressure / (R * ambient_temperature)`,
        // and the pressure-ratio terms throughout `pneumatic.rs`), which is
        // the NaN this test's own name is written to catch. Seeding the
        // same baseline FlyByWire's own tests always assume is not a
        // scripted value; it is the "sitting in the open air" condition no
        // real aircraft, cold or running, is ever without.
        vars.set("AMBIENT PRESSURE", 29.92);
        vars.set("AMBIENT TEMPERATURE", 0.0);
        let mut sim = Simulation::new(StartState::Apron, A380::new, &mut vars);
        let mut aspects = crate::aspects::a380(&mut vars);
        let dt_s = 0.05;

        // Phase 1: cold and dark, no sim input at all, for a minute.
        for i in 0..1_200u32 {
            aspects.pre_tick(&mut vars, dt_s);
            sim.tick(std::time::Duration::from_millis(50), i as f64 * dt_s, &mut vars);
            aspects.post_tick(&mut vars);
            check_every_variable(&vars, "cold and dark");
        }
        assert_eq!(vars.value("A32NX_ADIRS_ADIRU_1_STATE"), 0., "an unpowered cold aircraft must not be aligning or aligned");
        assert_eq!(vars.value("A32NX_APU_N_RAW"), 0., "the APU must not be spinning with nothing switched on");
        assert_eq!(vars.value("A32NX_APU_BLEED_AIR_VALVE_OPEN"), 0., "no bleed source exists on a cold aircraft");

        // Phase 2: batteries and the APU brought on, for another minute.
        for bat in ["BAT_1", "BAT_2", "BAT_ESS", "BAT_APU"] {
            vars.set(&format!("A32NX_OVHD_ELEC_{bat}_PB_IS_AUTO"), 1.);
        }
        vars.set("A32NX_OVHD_APU_MASTER_SW_PB_IS_ON", 1.);
        vars.set("A32NX_OVHD_APU_START_PB_IS_ON", 1.);
        for i in 1_200..2_400u32 {
            aspects.pre_tick(&mut vars, dt_s);
            sim.tick(std::time::Duration::from_millis(50), i as f64 * dt_s, &mut vars);
            aspects.post_tick(&mut vars);
            check_every_variable(&vars, "APU start");
        }
    }
}
