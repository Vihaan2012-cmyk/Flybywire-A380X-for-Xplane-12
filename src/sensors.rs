//! X-Plane's state for the MSFS simulator variables FlyByWire's A380X systems
//! read that the plugin's `mapping()` table does not cover.
//!
//! Doors: the systems read INTERACTIVE POINT OPEN:0 (M1L), :2 (M2L), :3 (M2R),
//! :6 (M4L), :8 (M5L), :10 (U1L) (payload/mod.rs:239-255,
//! air_conditioning/mod.rs:257-258; points in flight_model.cfg:826-837), and
//! the cargo door aspect reads :16 and :17 (cargo_doors.rs, ported in
//! aspects.rs). Nothing in X-Plane opens them: the converted model's
//! `fbw/anim/ANIM_DOOR_*` are outputs "at rest, for the systems to drive"
//! (the aircraft's README), so they stay writable `fbw/INTERACTIVE_POINT_OPEN_n`
//! datarefs (percent). Payload: X-Plane has no stations, only `m_fixed`; the
//! station weights are the payload aspect's (aspects.rs) and are not fed.
//!
//! The set of variables is the one the systems register (enumerated by
//! building `A380` against a recording registry; see the test
//! `every_simulator_variable_the_systems_read_is_accounted_for`), not the
//! longer list the MSFS glue declares in a380_systems_wasm/src/lib.rs:429-564,
//! some of which only its aspects or nothing at all read.
//!
//! Where X-Plane has no equivalent the variable is left alone: it keeps the
//! value its `fbw/` dataref holds, which is what the cockpit, Lua or another
//! plugin can drive, and starts at FlyByWire's no-data value.
//!
//! What is fed, from what, and why:
//!
//! | simulator variable | X-Plane | conversion |
//! |---|---|---|
//! | CONTACT POINT COMPRESSION, :1..:4 | flightmodel2/gear/tire_vertical_deflection_mtr[0..4] | m -> ft, / max compression (flight_model.cfg col 9), percent |
//! | GEAR ANIMATION POSITION:1..4 | flightmodel2/gear/deploy_ratio[1..4] | x100 |
//! | WHEEL RPM:1, :2 | flightmodel2/gear/tire_rotation_speed_rad_sec[1], [2] | rad/s -> rpm |
//! | GPS GROUND TRUE TRACK | flightmodel/position/hpath | [0, 360) |
//! | GPS GROUND MAGNETIC TRACK | hpath + (mag_psi - psi) | [0, 360) |
//! | INCIDENCE ALPHA | flightmodel/position/alpha | degrees, same sign |
//! | PLANE HEADING DEGREES MAGNETIC | flightmodel/position/mag_psi | degrees |
//! | PLANE LONGITUDE | flightmodel/position/longitude | degrees |
//! | LIGHT BEACON | cockpit2/switches/beacon_on | bool |
//! | PUSHBACK STATE | aircraft/overflow/pushback_attached | 0 straight / 3 none |
//! | SURFACE TYPE | flightmodel2/gear/on_ground, on_grass, on_noisy | MSFS enum |
//! | ENG ON FIRE:1..4 | cockpit2/annunciators/engine_fires[0..3] | bool |
//! | NAV ...:3 (the PRIMs' ILS) | cockpit2/radios/.../nav_*[2] | see [`Ils`] |
//! | AMBIENT IN CLOUD (ICE-001) | weather/aircraft/cloud_base_msl_m[0..3], cloud_tops_msl_m, cloud_coverage_percent, flightmodel/position/elevation | see [`is_in_cloud`] |

use std::ffi::{c_char, c_int, c_void, CString};

use systems::simulation::{SimulatorReaderWriter, VariableIdentifier, VariableRegistry};

use crate::prim::NavSimData;
use crate::xp::{DataRef, Xplm};
use crate::Vars;

/// One foot is 0.3048 m exactly.
const M_TO_FT: f64 = 1. / 0.3048;

/// The A380X's contact points 0..4 (nose, left body, right body, left wing,
/// right wing) have a maximum compression of these many feet: column 9 of
/// point.0..4, which `set_max_compression = 1` makes the maximum compression in
/// feet (flight_model.cfg:78, 90, 100-104). The converted .acf numbers its gear
/// the same way (_gear/0 nose at x 0; _gear/1,2 at x -11.5/+11.5 ft, z +5.7;
/// _gear/3,4 at x -23/+23 ft, z -5.7), as FlyByWire's
/// structural_flex/wing_flex.rs:51-55 and landing_gear/mod.rs:259-263 name them.
pub const MAX_COMPRESSION_FT: [f64; 5] = [1.204_819_277_108_43, 2.4, 2.4, 2.4, 2.4];

/// CONTACT POINT COMPRESSION, read by FlyByWire as a Ratio in percent
/// (simulation/mod.rs:791): the deflection over the contact point's maximum.
pub fn contact_point_compression_percent(deflection_m: f64, max_compression_ft: f64) -> f64 {
    if max_compression_ft <= 0. {
        return 0.;
    }
    (deflection_m * M_TO_FT / max_compression_ft * 100.).clamp(0., 100.)
}

/// WHEEL RPM, read as an AngularVelocity in revolutions per minute
/// (simulation/mod.rs:801). X-Plane's speed is the absolute value in rad/s.
pub fn wheel_rpm(rad_per_s: f64) -> f64 {
    rad_per_s.abs() * 60. / std::f64::consts::TAU
}

pub fn normalise_360(deg: f64) -> f64 {
    let a = deg.rem_euclid(360.);
    if a >= 360. {
        0.
    } else {
        a
    }
}

pub fn normalise_180(deg: f64) -> f64 {
    let a = normalise_360(deg);
    if a >= 180. {
        a - 360.
    } else {
        a
    }
}

/// X-Plane's own magnetic minus true heading at the aircraft. Taken from its
/// two headings rather than `magnetic_variation`, whose sign DataRefs.txt does
/// not state.
pub fn magnetic_minus_true(psi_true: f64, psi_magnetic: f64) -> f64 {
    normalise_180(psi_magnetic - psi_true)
}

pub fn magnetic_track(hpath_true: f64, psi_true: f64, psi_magnetic: f64) -> f64 {
    normalise_360(hpath_true + magnetic_minus_true(psi_true, psi_magnetic))
}

/// MSFS's pushback states (hydraulic/pushback.rs:20-25): 0 straight, 1 left,
/// 2 right, 3 none. X-Plane says only whether its tug is attached, not which
/// way it turns; FlyByWire only asks whether the state is 3 (pushback.rs:68-70)
/// and works the steering angle out from the yaw rate (pushback.rs:51-58).
pub fn pushback_state(tug_attached: bool) -> f64 {
    if tug_attached {
        0.
    } else {
        3.
    }
}

/// SurfaceTypeMsfs (update_context.rs:56-90) from the wheels on the ground:
/// grass 1 if any is on grass, gravel 14 on X-Plane's "noisy surface like
/// gravel", otherwise concrete 0, the value the variable holds with no data.
pub fn surface_type(on_ground: &[c_int], on_grass: &[c_int], on_noisy: &[c_int]) -> f64 {
    let any = |flags: &[c_int]| on_ground.iter().zip(flags).any(|(&g, &f)| g != 0 && f != 0);
    if any(on_grass) {
        1.
    } else if any(on_noisy) {
        14.
    } else {
        0.
    }
}

/// Localizer channels are 108.10 to 111.95 MHz with an odd tenths digit
/// (ICAO Annex 10 ILS channel pairing); VORs in that band use even tenths.
/// `freq_10khz` is X-Plane's 10 kHz unit (11030 is 110.30 MHz).
pub fn is_localizer_frequency(freq_10khz: c_int) -> bool {
    (10_810..=11_195).contains(&freq_10khz) && (freq_10khz / 10) % 2 == 1
}

/// FlyByWire's PFD scale: one localizer dot is 0.8 degrees of NAV RADIAL
/// ERROR, one glide slope dot 0.4 degrees of NAV GLIDE SLOPE ERROR
/// (PFD/LandingSystemIndicator.tsx:253, 353). The signs agree with X-Plane's:
/// FBW's dots move the diamond right / down for positive values (:267, :367)
/// and X-Plane's needles move right / down for positive hdef / vdef (Laminar
/// 737 738cockpit_optional.obj:13452-13457, 13477-13481); and FBW's calculated
/// receiver makes a positive glide slope error the aircraft above the beam
/// (CalculatedRadioReceiver.cpp:108).
pub const LOC_DEG_PER_DOT: f64 = 0.8;
pub const GS_DEG_PER_DOT: f64 = 0.4;

/// The glide slope angle a nav.dat glide slope carries in its bearing field:
/// the angle times 100 000 plus the bearing (300090.57 is 3.00 degrees on
/// 090.57). None when the field holds a plain bearing.
pub fn glide_slope_angle_from_bearing_field(field: f32) -> Option<f64> {
    let field = field as f64;
    (field >= 1000.).then(|| (field / 1000.).floor() / 100.)
}

/// X-Plane's nav receiver behind MSFS's NAV:3, zero-based.
pub const ILS_RECEIVER: usize = 2;

/// XPLMNavType's glide slope (XPLMNavigation.h:61).
const XPLM_NAV_GLIDESLOPE: c_int = 32;

#[link(name = "kernel32")]
extern "system" {
    fn LoadLibraryA(name: *const c_char) -> *mut c_void;
    fn GetProcAddress(module: *mut c_void, name: *const c_char) -> *mut c_void;
}

type FindNavAidFn =
    unsafe extern "C" fn(*const c_char, *const c_char, *mut f32, *mut f32, *mut c_int, c_int) -> c_int;
type GetNavAidInfoFn = unsafe extern "C" fn(
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

fn xplm_function(name: &str) -> Option<*mut c_void> {
    unsafe {
        let module = CString::new("XPLM_64.dll").ok()?;
        let module = LoadLibraryA(module.as_ptr());
        if module.is_null() {
            return None;
        }
        let name = CString::new(name).ok()?;
        let p = GetProcAddress(module, name.as_ptr());
        (!p.is_null()).then_some(p)
    }
}

/// The ILS on X-Plane's nav receiver 3, as MSFS's NAV simvars for receiver 3
/// (SimConnectInterface.cpp:214-219, 255; SimConnectData.h:77-82, 119):
///
/// - NAV HAS LOCALIZER: nav_display_horizontal and a localizer frequency;
/// - NAV HAS GLIDE SLOPE: nav_display_vertical and a localizer frequency;
/// - NAV RADIAL ERROR / GLIDE SLOPE ERROR: hdef / vdef dots at FBW's scale;
/// - NAV HAS DME / DME: nav_has_dme, nav_dme_distance_nm;
/// - NAV MAGVAR: magnetic minus true, so that updateIls's course - magvar
///   (cpp:1350) is the true runway heading, as CalculatedRadioReceiver.cpp:44-45
///   uses it;
/// - NAV RAW GLIDE SLOPE: nav1/nav2_slope_degt when that receiver is on the
///   same frequency, else the nearest glide slope on the frequency in X-Plane's
///   nav database; positive, as CalculatedRadioReceiver.cpp:108 subtracts it
///   from the elevation angle.
struct Ils {
    frequency: Option<DataRef>,
    horizontal: Option<DataRef>,
    vertical: Option<DataRef>,
    hdef: Option<DataRef>,
    vdef: Option<DataRef>,
    has_dme: Option<DataRef>,
    dme: Option<DataRef>,
    slope: [Option<DataRef>; 2],
    latitude: Option<DataRef>,
    ids: [VariableIdentifier; 8],
    find_navaid: Option<FindNavAidFn>,
    navaid_info: Option<GetNavAidInfoFn>,
    /// The frequency last looked up, when, and the angle found.
    looked_up: Option<(c_int, f64, Option<f64>)>,
}

/// How often the nav database is searched again for the same frequency, as
/// the nearest glide slope on it changes only between airports.
const GS_LOOKUP_SECONDS: f64 = 30.;

impl Ils {
    fn new(vars: &mut Vars, xplm: &Xplm) -> Self {
        let names = [
            NavSimData::LOC_VALID,
            NavSimData::LOC_ERROR,
            NavSimData::GS_VALID,
            NavSimData::GS_ERROR,
            NavSimData::DME_VALID,
            NavSimData::DME,
            NavSimData::MAGVAR,
            NavSimData::GS_ANGLE,
        ];
        Self {
            frequency: xplm.find("sim/cockpit2/radios/actuators/nav_frequency_hz"),
            horizontal: xplm.find("sim/cockpit2/radios/indicators/nav_display_horizontal"),
            vertical: xplm.find("sim/cockpit2/radios/indicators/nav_display_vertical"),
            hdef: xplm.find("sim/cockpit2/radios/indicators/nav_hdef_dots_pilot"),
            vdef: xplm.find("sim/cockpit2/radios/indicators/nav_vdef_dots_pilot"),
            has_dme: xplm.find("sim/cockpit2/radios/indicators/nav_has_dme"),
            dme: xplm.find("sim/cockpit2/radios/indicators/nav_dme_distance_nm"),
            slope: [
                xplm.find("sim/cockpit/radios/nav1_slope_degt"),
                xplm.find("sim/cockpit/radios/nav2_slope_degt"),
            ],
            latitude: xplm.find("sim/flightmodel/position/latitude"),
            ids: names.map(|n| vars.get(n.to_owned())),
            find_navaid: xplm_function("XPLMFindNavAid")
                .map(|p| unsafe { std::mem::transmute::<*mut c_void, FindNavAidFn>(p) }),
            navaid_info: xplm_function("XPLMGetNavAidInfo")
                .map(|p| unsafe { std::mem::transmute::<*mut c_void, GetNavAidInfoFn>(p) }),
            looked_up: None,
        }
    }

    fn update(&mut self, vars: &mut Vars, xplm: &Xplm, psi: f64, mag_psi: f64, longitude: f64, time: f64) {
        let n = ILS_RECEIVER + 1;
        let ints = |d: Option<DataRef>| {
            let mut v = [0 as c_int; ILS_RECEIVER + 1];
            d.map(|d| xplm.get_vi(d, &mut v));
            v
        };
        let floats = |d: Option<DataRef>| {
            let mut v = [0f32; ILS_RECEIVER + 1];
            d.map(|d| xplm.get_vf(d, &mut v));
            v
        };
        let Some(_) = self.frequency else { return };
        let frequency = ints(self.frequency);
        let freq = frequency[ILS_RECEIVER];
        let localizer = is_localizer_frequency(freq);
        let loc_valid = localizer && ints(self.horizontal)[ILS_RECEIVER] != 0;
        let gs_valid = localizer && ints(self.vertical)[ILS_RECEIVER] != 0;
        let gs_deg = if localizer {
            self.glide_slope_angle(xplm, &frequency[..n], freq, longitude, time)
        } else {
            None
        };
        let values = [
            loc_valid as i32 as f64,
            floats(self.hdef)[ILS_RECEIVER] as f64 * LOC_DEG_PER_DOT,
            gs_valid as i32 as f64,
            floats(self.vdef)[ILS_RECEIVER] as f64 * GS_DEG_PER_DOT,
            (ints(self.has_dme)[ILS_RECEIVER] != 0) as i32 as f64,
            floats(self.dme)[ILS_RECEIVER] as f64,
            magnetic_minus_true(psi, mag_psi),
            gs_deg.unwrap_or(0.),
        ];
        for (id, v) in self.ids.iter().zip(values) {
            vars.write_from_xplane(id, v);
        }
    }

    fn glide_slope_angle(
        &mut self,
        xplm: &Xplm,
        frequencies: &[c_int],
        freq: c_int,
        longitude: f64,
        time: f64,
    ) -> Option<f64> {
        for (i, slope) in self.slope.iter().enumerate() {
            if let Some(d) = slope {
                if frequencies[i] == freq {
                    let v = xplm.get_f(*d) as f64;
                    if v > 0. {
                        return Some(v);
                    }
                }
            }
        }
        if let Some((f, when, angle)) = self.looked_up {
            if f == freq && time - when < GS_LOOKUP_SECONDS {
                return angle;
            }
        }
        let (find, info) = (self.find_navaid?, self.navaid_info?);
        let mut lat = self.latitude.map_or(0., |d| xplm.get_d(d)) as f32;
        let mut lon = longitude as f32;
        let mut frequency = freq;
        let angle = unsafe {
            let navaid = find(
                std::ptr::null(),
                std::ptr::null(),
                &mut lat,
                &mut lon,
                &mut frequency,
                XPLM_NAV_GLIDESLOPE,
            );
            if navaid < 0 {
                None
            } else {
                let mut field = 0f32;
                info(
                    navaid,
                    std::ptr::null_mut(),
                    std::ptr::null_mut(),
                    std::ptr::null_mut(),
                    std::ptr::null_mut(),
                    std::ptr::null_mut(),
                    &mut field,
                    std::ptr::null_mut(),
                    std::ptr::null_mut(),
                    std::ptr::null_mut(),
                );
                glide_slope_angle_from_bearing_field(field)
            }
        };
        self.looked_up = Some((freq, time, angle));
        angle
    }
}

fn find_all(xplm: &Xplm, names: &[&str]) -> Vec<Option<DataRef>> {
    names.iter().map(|n| xplm.find(n)).collect()
}

/// The sensors: what they read from X-Plane and the variables they feed.
pub struct Sensors {
    // X-Plane.
    deflection: Option<DataRef>,
    deploy: Option<DataRef>,
    tire_speed: Option<DataRef>,
    tire_angle: Option<DataRef>,
    on_ground: Option<DataRef>,
    on_grass: Option<DataRef>,
    on_noisy: Option<DataRef>,
    fires: Option<DataRef>,
    baro_stby: Option<DataRef>,
    // hpath, psi, mag_psi, alpha, longitude, beacon, pushback
    scalars: Vec<Option<DataRef>>,
    // Variables.
    compression: [VariableIdentifier; 5],
    gear_animation: [VariableIdentifier; 4],
    wheel_rpm: [VariableIdentifier; 2],
    center_wheel_rotation_angle: VariableIdentifier,
    kohlsman_mb_3: VariableIdentifier,
    true_track: VariableIdentifier,
    magnetic_track: VariableIdentifier,
    alpha: VariableIdentifier,
    heading_magnetic: VariableIdentifier,
    longitude: VariableIdentifier,
    beacon: VariableIdentifier,
    pushback: VariableIdentifier,
    surface: VariableIdentifier,
    engine_fire: [VariableIdentifier; 4],
    ils: Ils,
    // ICE-001: per-layer cloud base/tops/coverage and the aircraft's own MSL
    // elevation.
    cloud_base: Option<DataRef>,
    cloud_tops: Option<DataRef>,
    cloud_coverage: Option<DataRef>,
    elevation: Option<DataRef>,
    in_cloud: VariableIdentifier,
    // Second pass (docs/physics/landing-gear-brakes.md gap #2): the real
    // cockpit A-SKID switch, not a forced-on constant.
    antiskid_switch: Option<DataRef>,
    antiskid: VariableIdentifier,
}

impl Sensors {
    pub fn new(vars: &mut Vars, xplm: &Xplm) -> Self {
        // Not a closure over `vars` (a mutable borrow spanning the whole
        // function would block the plain `vars.write`/`Ils::new` calls
        // below): each call below borrows `vars` only for that one read.
        let compression = [
            vars.get("CONTACT POINT COMPRESSION".into()),
            vars.get("CONTACT POINT COMPRESSION:1".into()),
            vars.get("CONTACT POINT COMPRESSION:2".into()),
            vars.get("CONTACT POINT COMPRESSION:3".into()),
            vars.get("CONTACT POINT COMPRESSION:4".into()),
        ];
        let gear_animation = [1, 2, 3, 4].map(|n| vars.get(format!("GEAR ANIMATION POSITION:{n}")));
        // hydraulic/mod.rs:2074-2100: WHEEL RPM:1 and :2 are the left and right
        // body gear (FBW notes the wing gear should be :3 and :4).
        let wheel_rpm = [1, 2].map(|n| vars.get(format!("WHEEL RPM:{n}")));
        // nose_wheel_steering.rs:165,176 reads this to lay the steering
        // offset on top of the tire's own spin for the animated wheel angle.
        let center_wheel_rotation_angle = vars.get("CENTER WHEEL ROTATION ANGLE".into());
        // ISISlegacy/PressureIndicator.tsx:16-17: the standby altimeter's own
        // baro setting, read directly by the JS instrument.
        let kohlsman_mb_3 = vars.get("KOHLSMAN SETTING MB:3".into());
        let true_track = vars.get("GPS GROUND TRUE TRACK".into());
        let magnetic_track = vars.get("GPS GROUND MAGNETIC TRACK".into());
        let alpha = vars.get("INCIDENCE ALPHA".into());
        let heading_magnetic = vars.get("PLANE HEADING DEGREES MAGNETIC".into());
        let longitude = vars.get("PLANE LONGITUDE".into());
        let beacon = vars.get("LIGHT BEACON".into());
        let pushback = vars.get("PUSHBACK STATE".into());
        let surface = vars.get("SURFACE TYPE".into());
        let engine_fire = [1, 2, 3, 4].map(|n| vars.get(format!("ENG ON FIRE:{n}")));
        let in_cloud = vars.get("AMBIENT IN CLOUD".into());
        let antiskid = vars.get("ANTISKID BRAKES ACTIVE".into());
        // The real cockpit A-SKID switch: the converted aircraft's
        // SWITCH_AUTOBKR_ASKID control (centre pedestal) toggles this
        // `fbw/ANTISKID_BRAKES_ACTIVE` dataref between 1 and 0
        // (cockpit_bindings.txt:1431; main.lua's SWITCH_AUTOBKR_ASKID reads
        // it back with `rd("fbw/ANTISKID_BRAKES_ACTIVE") ~= 0` to drive the
        // switch's own visual position). The converter's own default value
        // for it is 0 (main.lua's dataref defaults table), but FlyByWire's
        // BSCU starts with antiskid on (hydraulic/mod.rs:4370) and X-Plane's
        // stock aircraft default (K:ANTISKID_BRAKES_TOGGLE) is likewise on
        // unless switched off — so this is corrected to 1 once at spawn,
        // the one place this plugin still writes it; every tick after that
        // only copies the switch's own current position through to
        // `ANTISKID BRAKES ACTIVE`, the simulator variable
        // `A380HydraulicBrakeSteerComputerUnit::read` consumes
        // (hydraulic/mod.rs) and which gates autobrake arming, the
        // alternate-brake pressure limit and nosewheel steering
        // availability (hydraulic/mod.rs:4399-4403, :4607-4609,
        // `update_brake_pressure_limitation`).
        let antiskid_switch = xplm.find("fbw/ANTISKID_BRAKES_ACTIVE");
        if let Some(d) = antiskid_switch {
            if xplm.get_f(d) == 0. {
                xplm.set_f(d, 1.);
            }
        }
        vars.write(&antiskid, if antiskid_switch.map(|d| xplm.get_f(d) != 0.).unwrap_or(true) { 1. } else { 0. });
        let ils = Ils::new(vars, xplm);
        let find = |n: &str| xplm.find(n);
        Self {
            deflection: find("sim/flightmodel2/gear/tire_vertical_deflection_mtr"),
            deploy: find("sim/flightmodel2/gear/deploy_ratio"),
            tire_speed: find("sim/flightmodel2/gear/tire_rotation_speed_rad_sec"),
            tire_angle: find("sim/flightmodel2/gear/tire_rotation_angle_deg"),
            on_ground: find("sim/flightmodel2/gear/on_ground"),
            on_grass: find("sim/flightmodel2/gear/on_grass"),
            on_noisy: find("sim/flightmodel2/gear/on_noisy"),
            fires: find("sim/cockpit2/annunciators/engine_fires"),
            baro_stby: find("sim/cockpit2/gauges/actuators/barometer_setting_in_hg_stby"),
            scalars: find_all(
                xplm,
                &[
                    "sim/flightmodel/position/hpath",
                    "sim/flightmodel/position/psi",
                    "sim/flightmodel/position/mag_psi",
                    "sim/flightmodel/position/alpha",
                    "sim/flightmodel/position/longitude",
                    "sim/cockpit2/switches/beacon_on",
                    "sim/aircraft/overflow/pushback_attached",
                ],
            ),
            compression,
            gear_animation,
            wheel_rpm,
            center_wheel_rotation_angle,
            kohlsman_mb_3,
            true_track,
            magnetic_track,
            alpha,
            heading_magnetic,
            longitude,
            beacon,
            pushback,
            surface,
            engine_fire,
            ils,
            cloud_base: find("sim/weather/aircraft/cloud_base_msl_m"),
            cloud_tops: find("sim/weather/aircraft/cloud_tops_msl_m"),
            cloud_coverage: find("sim/weather/aircraft/cloud_coverage_percent"),
            elevation: xplm.find("sim/flightmodel/position/elevation"),
            in_cloud,
            antiskid_switch,
            antiskid,
        }
    }

    /// Before the PRIMs and the systems.
    pub fn update_inputs(&mut self, vars: &mut Vars, xplm: &Xplm, time: f64) {
        let floats = |d: Option<DataRef>| {
            d.map(|d| {
                let mut v = [0f32; 5];
                xplm.get_vf(d, &mut v);
                v
            })
        };
        let ints = |d: Option<DataRef>| {
            d.map(|d| {
                let mut v = [0 as c_int; 5];
                xplm.get_vi(d, &mut v);
                v
            })
        };

        if let Some(deflection) = floats(self.deflection) {
            for i in 0..5 {
                let v = contact_point_compression_percent(deflection[i] as f64, MAX_COMPRESSION_FT[i]);
                vars.write_from_xplane(&self.compression[i], v);
            }
        }
        if let Some(deploy) = floats(self.deploy) {
            for i in 0..4 {
                vars.write_from_xplane(&self.gear_animation[i], deploy[i + 1] as f64 * 100.);
            }
        }
        if let Some(speed) = floats(self.tire_speed) {
            for i in 0..2 {
                vars.write_from_xplane(&self.wheel_rpm[i], wheel_rpm(speed[i + 1] as f64));
            }
        }
        // CENTER WHEEL ROTATION ANGLE: the nose gear (index 0, this module's
        // own convention for GEAR ANIMATION POSITION/CONTACT POINT
        // COMPRESSION) already turns 0..360 the way MSFS's variable does.
        if let Some(angle) = floats(self.tire_angle) {
            vars.write_from_xplane(&self.center_wheel_rotation_angle, angle[0] as f64);
        }
        // KOHLSMAN SETTING MB:3: the standby altimeter's own baro setting.
        if let Some(d) = self.baro_stby {
            vars.write_from_xplane(&self.kohlsman_mb_3, kohlsman_mb_from_inhg(xplm.get_f(d) as f64));
        }
        if let (Some(g), Some(grass), Some(noisy)) = (ints(self.on_ground), ints(self.on_grass), ints(self.on_noisy)) {
            vars.write_from_xplane(&self.surface, surface_type(&g, &grass, &noisy));
        }
        if let Some(d) = self.fires {
            let mut fires = [0 as c_int; 4];
            xplm.get_vi(d, &mut fires);
            for i in 0..4 {
                vars.write_from_xplane(&self.engine_fire[i], (fires[i] != 0) as i32 as f64);
            }
        }

        let s = &self.scalars;
        let f = |i: usize| s[i].map(|d| xplm.get_f(d) as f64);
        let (hpath, psi, mag_psi, alpha) = (f(0), f(1), f(2), f(3));
        let longitude = s[4].map(|d| xplm.get_d(d));
        if let Some(hpath) = hpath {
            vars.write_from_xplane(&self.true_track, normalise_360(hpath));
            if let (Some(psi), Some(mag)) = (psi, mag_psi) {
                vars.write_from_xplane(&self.magnetic_track, magnetic_track(hpath, psi, mag));
            }
        }
        if let Some(alpha) = alpha {
            vars.write_from_xplane(&self.alpha, alpha);
        }
        if let Some(mag) = mag_psi {
            vars.write_from_xplane(&self.heading_magnetic, mag);
        }
        if let Some(lon) = longitude {
            vars.write_from_xplane(&self.longitude, lon);
        }
        if let Some(d) = s[5] {
            vars.write_from_xplane(&self.beacon, (xplm.get_i(d) != 0) as i32 as f64);
        }
        if let Some(d) = s[6] {
            vars.write_from_xplane(&self.pushback, pushback_state(xplm.get_i(d) != 0));
        }
        if let (Some(psi), Some(mag)) = (psi, mag_psi) {
            self.ils.update(vars, xplm, psi, mag, longitude.unwrap_or(0.), time);
        }

        // A-SKID switch: copy the cockpit's own current position through
        // every tick (not just at spawn), so toggling it off in flight
        // really disables antiskid/autobrake/nosewheel-steering via the
        // BSCU's existing `anti_skid_activated`-gated logic.
        if let Some(d) = self.antiskid_switch {
            vars.write_from_xplane(&self.antiskid, if xplm.get_f(d) != 0. { 1. } else { 0. });
        }

        // ICE-001: `AMBIENT IN CLOUD` from X-Plane's per-layer cloud state.
        if let (Some(base_d), Some(tops_d), Some(cov_d), Some(elev_d)) =
            (self.cloud_base, self.cloud_tops, self.cloud_coverage, self.elevation)
        {
            let mut base = [0f32; 3];
            let mut tops = [0f32; 3];
            let mut coverage = [0f32; 3];
            xplm.get_vf(base_d, &mut base);
            xplm.get_vf(tops_d, &mut tops);
            xplm.get_vf(cov_d, &mut coverage);
            let elevation_m = xplm.get_d(elev_d);
            let in_cloud = is_in_cloud(elevation_m, &base, &tops, &coverage);
            vars.write_from_xplane(&self.in_cloud, in_cloud as i32 as f64);
        }
    }
}

/// FlyByWire's own `icing_state/mod.rs::is_in_cloud` (systems.md ICE-001)
/// needs a bool from X-Plane's own weather; here the aircraft is "in cloud"
/// when its MSL elevation sits inside any one of the three layers' base/tops
/// and that layer's coverage is above a quarter (systems.md's own "scattered
/// or denser" proposal, DataRefs.txt `cloud_coverage_percent` is 0..1).
pub fn is_in_cloud(elevation_m: f64, base_msl_m: &[f32; 3], tops_msl_m: &[f32; 3], coverage: &[f32; 3]) -> bool {
    const MIN_COVERAGE: f32 = 0.25;
    (0..3).any(|i| {
        coverage[i] > MIN_COVERAGE
            && elevation_m >= base_msl_m[i] as f64
            && elevation_m <= tops_msl_m[i] as f64
            && tops_msl_m[i] > base_msl_m[i]
    })
}

/// Degrees to radians, for the SimData struct's body rotation fields below
/// (X-Plane's `*_dot` rate datarefs are in degrees per second squared).
const DEG_TO_RAD: f64 = std::f64::consts::PI / 180.;

/// One g is this many metres per second squared (standard gravity), the unit
/// FlyByWire's SimData.bz_m_s2 (ACCELERATION BODY Z) is named for
/// (SimConnectData.h:15; SimConnectInterface.cpp:148 requests it in that
/// unit — unlike lib.rs's `mapping()` ACCELERATION BODY X/Y, which convert
/// the same g_side/g_nrml datarefs to *feet* per second squared for the
/// generic named simvar. That is a separate table entry this module does not
/// own; noted here so the units mismatch is not repeated for BODY Z).
const G_TO_M_S2: f64 = 9.806_65;

/// G FORCE (SimConnectData.h:8 `nz_g`), read at FlyByWireInterface.cpp:1214
/// (`updateBaseData`, the flight data recorder's normal load factor) and at
/// cpp:1186-1188 as the `/ g` divisor is applied to `bodyRotationAcceleration`,
/// not to this field itself. MSFS's G FORCE is the normal load factor in g's,
/// ~1 in level flight; X-Plane's `sim/flightmodel/forces/g_nrml` ("total
/// g-forces on the plane as a multiple, downward") is the same quantity, so
/// this is a straight pass-through.
pub fn g_force(g_nrml: f64) -> f64 {
    g_nrml
}

/// STRUCT BODY ROTATION VELOCITY (SimConnectData.h:11), a `SIMCONNECT_DATA_XYZ`
/// in radians per second, read at FlyByWireInterface.cpp:1178-1180. MSFS's own
/// body axes for this struct: x pitch, y yaw, z roll (matching the mapping()
/// table's "ROTATION VELOCITY BODY X/Y/Z", lib.rs:496-501, which serve the
/// same three rates from the same X-Plane datarefs as separate named
/// variables) — so the struct's members are read the same way, straight from
/// X-Plane's Qrad/Rrad/Prad (already radians per second, X-Plane's own
/// pitch/yaw/roll rates).
pub fn body_rotation_velocity_rad_s(p_rad_s: f64, q_rad_s: f64, r_rad_s: f64) -> (f64, f64, f64) {
    (q_rad_s, r_rad_s, p_rad_s)
}

/// STRUCT BODY ROTATION ACCELERATION (SimConnectData.h:12), the same x
/// pitch/y yaw/z roll axes as [`body_rotation_velocity_rad_s`], in radians
/// per second squared, read at FlyByWireInterface.cpp:1186-1188. X-Plane
/// gives the angular accelerations directly as `Q_dot`/`R_dot`/`P_dot`
/// (degrees per second squared — the same datarefs lib.rs's mapping() uses
/// for "ROTATION ACCELERATION BODY X/Y/Z", lib.rs:502-504), so this converts
/// them rather than differencing successive rates across the frame delta
/// (equivalent, but X-Plane's own derivative avoids a frame of lag and
/// division-by-small-dt noise).
pub fn body_rotation_acceleration_rad_s2(p_dot_deg_s2: f64, q_dot_deg_s2: f64, r_dot_deg_s2: f64) -> (f64, f64, f64) {
    (q_dot_deg_s2 * DEG_TO_RAD, r_dot_deg_s2 * DEG_TO_RAD, p_dot_deg_s2 * DEG_TO_RAD)
}

/// ACCELERATION BODY Z (SimConnectData.h:15 `bz_m_s2`), read at
/// FlyByWireInterface.cpp:1220 into the flight data recorder. FBW's field
/// name and SimConnectInterface.cpp:148's data definition both call for
/// metres per second squared. X-Plane's `sim/flightmodel/forces/g_axil`
/// ("total g-forces on the plane as a multiple, along the plane") is the
/// aircraft's longitudinal body axis, the same body axis MSFS calls Z here
/// (compare `ACCELERATION_BODY_Z_WITH_REVERSER`, lib.rs:484-486, which reads
/// the same g_axil dataref for the generic named variable).
pub fn accel_body_z_m_s2(g_axil: f64) -> f64 {
    g_axil * G_TO_M_S2
}

/// AUTOPILOT MASTER (SimConnectData.h:44 `autopilot_master_on`), read at
/// FlyByWireInterface.cpp:165: if MSFS's own default autopilot is on, FBW
/// sends `AUTOPILOT_OFF` to disconnect it, since the A380X flies through its
/// own FCU/PRIMs and never wants MSFS's autopilot engaged. This port has
/// nothing else to source that flag from but X-Plane's own native autopilot
/// state, `sim/cockpit2/autopilot/autopilot_on` ("Is the autopilot really
/// on? Takes into account electrical system, failures, etc.").
pub fn autopilot_master_on(xp_autopilot_on: bool) -> bool {
    xp_autopilot_on
}

/// One inch of mercury is this many millibars/hectopascals.
const INHG_TO_MBAR: f64 = 33.863_886_666_67;

/// KOHLSMAN SETTING MB:3 (a380_systems_wasm/src/lib.rs's own index 1 is a
/// different consumer; index 3 is read directly by the JS ISIS instrument,
/// ISISlegacy/PressureIndicator.tsx:16-17, the standby altimeter). X-Plane's
/// `barometer_setting_in_hg_stby` is the standby instrument's own baro
/// setting, in inches of mercury; MSFS wants millibars.
pub fn kohlsman_mb_from_inhg(in_hg: f64) -> f64 {
    in_hg * INHG_TO_MBAR
}

/// KOHLSMAN SETTING STD:4 (SimConnectData.h:116 `kohlsmanSettingStd_4`),
/// read at FlyByWireInterface.cpp:3070 to decide whether the pilot's
/// altimeter follows the FCU's own STD logic. X-Plane's
/// `sim/cockpit2/gauges/actuators/barometer_setting_is_std_pilot` is the
/// matching pilot-side STD flag.
pub fn kohlsman_setting_std_4(xp_is_std: bool) -> bool {
    xp_is_std
}

#[cfg(test)]
mod tests {
    use super::*;
    use a380_systems::A380;
    use systems::simulation::{Simulation, StartState};

    #[derive(Default)]
    struct Names(Vec<String>);
    impl VariableRegistry for Names {
        fn get(&mut self, name: String) -> VariableIdentifier {
            self.0.push(name);
            VariableIdentifier::new(0usize)
        }
        fn get_unprefixed(&mut self, name: String) -> VariableIdentifier {
            self.0.push(name);
            VariableIdentifier::new(0usize)
        }
    }

    /// Every MSFS simulator variable the A380 registers, and where it comes
    /// from in this plugin. A variable added upstream fails this test until
    /// it is placed.
    #[test]
    fn every_simulator_variable_the_systems_read_is_accounted_for() {
        let mut names = Names::default();
        let _ = Simulation::new(StartState::Apron, A380::new, &mut names);
        let fed_here = [
            "CONTACT POINT COMPRESSION", "GEAR ANIMATION POSITION:", "WHEEL RPM:", "GPS GROUND TRUE TRACK",
            "GPS GROUND MAGNETIC TRACK", "INCIDENCE ALPHA", "PLANE HEADING DEGREES MAGNETIC", "PLANE LONGITUDE",
            "LIGHT BEACON", "PUSHBACK STATE", "SURFACE TYPE", "ENG ON FIRE:", "ANTISKID BRAKES ACTIVE",
            // ICE-001.
            "AMBIENT IN CLOUD",
        ];
        // Fed by other modules: lib.rs mapping()/Computed, fadec.rs,
        // engine_commands.rs (TAT), fuel.rs; or written by the systems.
        let elsewhere = [
            "TOTAL AIR TEMPERATURE", "TOTAL WEIGHT", "TURB ENG ", "GENERAL ENG STARTER ACTIVE:", "FUELSYSTEM ",
            "BRAKE LEFT FORCE FACTOR", "BRAKE RIGHT FORCE FACTOR",
        ];
        // No X-Plane source: left at the dataref's value.
        let no_source = ["UNLIMITED FUEL", "INTERACTIVE POINT OPEN:"];
        let mut unplaced = Vec::new();
        for n in names.0.iter().filter(|n| n.contains(' ')) {
            let placed = crate::source_dataref(n).is_some()
                || fed_here.iter().chain(&elsewhere).chain(&no_source).any(|p| n.starts_with(p));
            if !placed {
                unplaced.push(n.clone());
            }
        }
        unplaced.sort();
        unplaced.dedup();
        assert!(unplaced.is_empty(), "{unplaced:?}");
    }

    #[test]
    fn compression_is_deflection_over_msfs_max_compression() {
        // Main gear: 2.4 ft is 0.73152 m. Half of it is 50 %.
        assert!((contact_point_compression_percent(0.73152 / 2., MAX_COMPRESSION_FT[1]) - 50.).abs() < 1e-9);
        // Nose: 1.2048 ft is 0.367229 m.
        assert!((contact_point_compression_percent(0.367_228_915, MAX_COMPRESSION_FT[0]) - 100.).abs() < 1e-4);
        // Gear in the air, and past full travel.
        assert_eq!(contact_point_compression_percent(0., 2.4), 0.);
        assert_eq!(contact_point_compression_percent(5., 2.4), 100.);
        // FBW's weight on wheels threshold is 1 % (landing_gear/mod.rs:266):
        // 7.3 mm on a main gear.
        assert!(contact_point_compression_percent(0.0074, 2.4) > 1.);
        assert!(contact_point_compression_percent(0.0072, 2.4) < 1.);
    }

    #[test]
    fn wheel_speed_in_rpm() {
        assert!((wheel_rpm(std::f64::consts::TAU) - 60.).abs() < 1e-9);
        assert!((wheel_rpm(-std::f64::consts::PI) - 30.).abs() < 1e-9);
    }

    #[test]
    fn tracks_and_magnetic_variation_follow_x_planes_own_headings() {
        // 10 degrees west: magnetic heading is true + 10.
        assert!((magnetic_minus_true(90., 100.) - 10.).abs() < 1e-9);
        assert!((magnetic_track(355., 90., 100.) - 5.).abs() < 1e-9);
        assert!((magnetic_minus_true(355., 5.) - 10.).abs() < 1e-9);
        assert_eq!(normalise_360(-90.), 270.);
        assert_eq!(normalise_180(180.), -180.);
        // updateIls's runway heading: magnetic course - (mag - true) is true.
        let nav = NavSimData { loc_valid: true, loc_magvar_deg: magnetic_minus_true(90., 100.), ..Default::default() };
        let bus = crate::prim::ils_bus(&nav, 100.);
        assert!((bus.runway_heading_deg.Data - 90.).abs() < 1e-4);
    }

    #[test]
    fn pushback_state_is_msfs_enum() {
        assert_eq!(pushback_state(false), 3.);
        assert_eq!(pushback_state(true), 0.);
    }

    #[test]
    fn surface_type_from_wheels_on_the_ground() {
        assert_eq!(surface_type(&[1, 1, 1, 1, 1], &[0; 5], &[0; 5]), 0.);
        assert_eq!(surface_type(&[1, 1, 1, 1, 1], &[0, 1, 0, 0, 0], &[0; 5]), 1.);
        assert_eq!(surface_type(&[1, 1, 1, 1, 1], &[0; 5], &[0, 0, 1, 0, 0]), 14.);
        // A wheel off the ground does not count.
        assert_eq!(surface_type(&[1, 0, 1, 1, 1], &[0, 1, 0, 0, 0], &[0; 5]), 0.);
    }

    #[test]
    fn localizer_frequencies_have_odd_tenths() {
        assert!(is_localizer_frequency(11030)); // 110.30
        assert!(is_localizer_frequency(10815)); // 108.15
        assert!(is_localizer_frequency(11195)); // 111.95
        assert!(!is_localizer_frequency(11020)); // 110.20 VOR
        assert!(!is_localizer_frequency(11390)); // 113.90 VOR
        assert!(!is_localizer_frequency(0));
    }

    #[test]
    fn dots_become_degrees_at_fbws_pfd_scale() {
        // Full PFD scale, 2 dots: 1.6 degrees of localizer, 0.8 of glide slope.
        assert!((2. * LOC_DEG_PER_DOT - 1.6).abs() < 1e-12);
        assert!((2. * GS_DEG_PER_DOT - 0.8).abs() < 1e-12);
    }

    #[test]
    fn glide_slope_angle_from_nav_dat_bearing_field() {
        assert_eq!(glide_slope_angle_from_bearing_field(300_090.57), Some(3.));
        assert_eq!(glide_slope_angle_from_bearing_field(325_270.0), Some(3.25));
        assert_eq!(glide_slope_angle_from_bearing_field(270.0), None);
    }

    #[test]
    fn ils_bus_ssm_and_localiser_correction() {
        use crate::prim::{autoland_warning_condition, correct_msfs_localiser_error};
        // No signal: NCD (1) on heading and deviations, frequency word NO (3)
        // with 0 (FlyByWireInterface.cpp:1349-1358).
        let bus = crate::prim::ils_bus(&NavSimData::default(), 0.);
        assert_eq!(bus.runway_heading_deg.SSM, 1);
        assert_eq!(bus.localizer_deviation_deg.SSM, 1);
        assert_eq!(bus.glideslope_deviation_deg.SSM, 1);
        assert_eq!((bus.ils_frequency_mhz.SSM, bus.ils_frequency_mhz.Data), (3, 0.));
        let nav = NavSimData {
            loc_valid: true,
            loc_error_deg: 0.8,
            gs_valid: true,
            gs_error_deg: -0.2,
            ..Default::default()
        };
        let bus = crate::prim::ils_bus(&nav, 0.);
        assert_eq!((bus.localizer_deviation_deg.SSM, bus.glideslope_deviation_deg.SSM), (3, 3));
        assert!((bus.localizer_deviation_deg.Data - 0.8).abs() < 1e-6);
        assert!((bus.glideslope_deviation_deg.Data + 0.2).abs() < 1e-6);
        // MathUtils.h:80-89: back-course readings fold into +-90.
        assert!((correct_msfs_localiser_error(178.) - 2.).abs() < 1e-9);
        assert!((correct_msfs_localiser_error(-178.) + 2.).abs() < 1e-9);
        assert!((correct_msfs_localiser_error(-1.) + 1.).abs() < 1e-9);
        // cpp:1951-1954.
        let on_beam = NavSimData { loc_valid: true, gs_valid: true, ..Default::default() };
        assert!(!autoland_warning_condition(true, 150., &on_beam));
        assert!(autoland_warning_condition(false, 150., &on_beam));
        let off_loc = NavSimData { loc_error_deg: 0.3, ..on_beam };
        assert!(autoland_warning_condition(true, 20., &off_loc));
        assert!(!autoland_warning_condition(true, 10., &off_loc));
        let off_gs = NavSimData { gs_error_deg: 0.5, ..on_beam };
        assert!(!autoland_warning_condition(true, 90., &off_gs));
        assert!(autoland_warning_condition(true, 110., &off_gs));
    }

    #[test]
    fn in_cloud_needs_altitude_inside_the_layer_and_enough_coverage() {
        let base = [1000., 0., 0.];
        let tops = [3000., 0., 0.];
        // Below a quarter coverage: not "in cloud" even inside the layer.
        assert!(!is_in_cloud(2000., &base, &tops, &[0.1, 0., 0.]));
        // Enough coverage, but below the layer.
        assert!(!is_in_cloud(500., &base, &tops, &[0.5, 0., 0.]));
        // Enough coverage, above the layer.
        assert!(!is_in_cloud(4000., &base, &tops, &[0.5, 0., 0.]));
        // Inside the layer with real coverage.
        assert!(is_in_cloud(2000., &base, &tops, &[0.5, 0., 0.]));
        // A second/third layer each count too.
        assert!(is_in_cloud(2000., &[0., 1000., 0.], &[0., 3000., 0.], &[0., 0.9, 0.]));
    }

    #[test]
    fn an_empty_layer_bottom_equal_to_top_is_never_in_cloud() {
        assert!(!is_in_cloud(1000., &[1000., 0., 0.], &[1000., 0., 0.], &[1., 0., 0.]));
    }

    #[test]
    fn g_force_is_g_nrml_unconverted() {
        assert!((g_force(1.02) - 1.02).abs() < 1e-9);
        assert!((g_force(2.5) - 2.5).abs() < 1e-9);
    }

    #[test]
    fn body_rotation_velocity_reorders_prq_into_msfs_xyz() {
        // x = pitch (q), y = yaw (r), z = roll (p).
        let (x, y, z) = body_rotation_velocity_rad_s(0.1, 0.2, 0.3);
        assert_eq!((x, y, z), (0.2, 0.3, 0.1));
    }

    #[test]
    fn body_rotation_acceleration_converts_degrees_to_radians_and_reorders() {
        let (x, y, z) = body_rotation_acceleration_rad_s2(180., 90., 360.);
        assert!((x - std::f64::consts::FRAC_PI_2).abs() < 1e-9); // q_dot 90 deg/s2
        assert!((y - std::f64::consts::TAU).abs() < 1e-9); // r_dot 360 deg/s2
        assert!((z - std::f64::consts::PI).abs() < 1e-9); // p_dot 180 deg/s2
    }

    #[test]
    fn accel_body_z_converts_g_to_metres_per_second_squared() {
        assert!((accel_body_z_m_s2(1.) - G_TO_M_S2).abs() < 1e-9);
        assert!((accel_body_z_m_s2(0.) - 0.).abs() < 1e-9);
        assert!((accel_body_z_m_s2(-0.5) - (-0.5 * G_TO_M_S2)).abs() < 1e-9);
    }

    #[test]
    fn kohlsman_mb_from_inhg_converts_standby_baro_to_millibars() {
        // 29.92 inHg standard is ~1013.25 mbar.
        assert!((kohlsman_mb_from_inhg(29.92) - 1013.21).abs() < 0.1);
        assert!((kohlsman_mb_from_inhg(0.) - 0.).abs() < 1e-9);
    }

    #[test]
    fn autopilot_master_on_and_kohlsman_setting_std_4_pass_through_xplane() {
        assert!(autopilot_master_on(true));
        assert!(!autopilot_master_on(false));
        assert!(kohlsman_setting_std_4(true));
        assert!(!kohlsman_setting_std_4(false));
    }
}
