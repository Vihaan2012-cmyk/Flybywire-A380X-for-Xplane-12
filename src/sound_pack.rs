use std::path::Path;
use std::sync::{Mutex, OnceLock};

use systems::simulation::{SimulatorReaderWriter, VariableIdentifier, VariableRegistry};

use crate::published::{self, Published, Value};
use crate::throttle::{TLA_REVERSE, TLA_TOGA};
use crate::xp::{DataRef, Xplm};
use crate::Vars;

const REPLACED: &[&str] = &[
    "apuwhine",
    "apuwhine2",
    "apubass",
    "apubass2",
    "apurun",
    "apushut",
    "apustartinit",
    "apubassout",
    "apuwhineext",
    "apuwhineext2",
    "apurun380ext",
    "apustartinitext_01",
    "apushutdownext",
    "AEVC380",
    "cockpitpacks380",
    "noseroll380",
    "cabingroll380",
    "NWheelSpinAboveGround380",
    "Centerthump380",
    "smoothtouch380",
    "medtouch380",
    "hardtouch380",
    "cabtouchsmooth1",
    "cabtouchmed1",
    "a380cabtouchhard",
    "380Geardownwind",
    "380gup",
    "380gmain",
    "noselock380",
    "nosedownlock380",
    "cockpitwind380",
    "380cabinwind",
    "gearleverclick",
    "flapnotch",
    "Spoiler0",
    "Spoiler50",
    "Spoiler100",
    "SpoilerARM",
    "SpoilerDISARM",
    "detent",
    "IdleDetent",
    "parkbrk",
    "CRC_380",
    "mastercaution",
    "cavcharge",
    "cchordloop",
    "cchordshort",
    "aural_stall_new",
    "aural_pullup_new",
    "aural_sink_rate_new",
    "aural_dontsink_new",
    "aural_too_low_gear",
    "aural_too_low_flaps",
    "aural_too_low_terrain",
    "new_2500",
    "new_2_500",
    "new_1000",
    "new_500",
    "new_400",
    "new_300",
    "new_200",
    "new_100",
    "50_380",
    "40_380",
    "30_380",
    "20_380",
    "10_380",
    "new_retard",
    "aural_minimumnew",
    "light_switch_seatbelt",
    "flaps380",
    "flaps380stop",
    "slats380stop",
];

const CALLOUTS: &[(&str, u32)] = &[
    ("new_2500", 2500),
    ("new_2_500", 2500),
    ("new_1000", 1000),
    ("new_500", 500),
    ("new_400", 400),
    ("new_300", 300),
    ("new_200", 200),
    ("new_100", 100),
    ("50_380", 50),
    ("40_380", 40),
    ("30_380", 30),
    ("20_380", 20),
    ("10_380", 10),
];

const CALLOUT_HEIGHTS: [u32; 12] = [2500, 1000, 500, 400, 300, 200, 100, 50, 40, 30, 20, 10];

const CLICKS: &[(&str, &str)] = &[
    ("pb1on", "p1"),
    ("mpb1on", "p1"),
    ("fcubutton", "p1"),
    ("roundbutton", "p1"),
    ("CHRon", "p1"),
    ("SETon", "p1"),
    ("RSTon", "p1"),
    ("firetestbutton", "p1"),
    ("pb1off", "p2"),
    ("mpb1off", "p2"),
    ("CHRoff", "p2"),
    ("SEToff", "p2"),
    ("RSToff", "p2"),
    ("mcdubuttons", "p3"),
    ("firepb", "p4"),
    ("engmaster", "s1"),
    ("lswitch", "s2"),
    ("flapcover", "s4"),
    ("firepbguard", "s4"),
    ("revlever", "sel0"),
    ("turnknob", "sel1"),
    ("smallknob", "sel1"),
    ("SETknob", "sel1"),
    ("RMPknob", "sel1"),
    ("QNHknob", "sel1"),
    ("pulloutknob", "sel2"),
    ("clds", "sel2"),
    ("rudtrim", "sel3"),
];

const CLICK_SOUNDS: [&str; 12] = ["p1", "p2", "p3", "p4", "p5", "s1", "s2", "s3", "s4", "sel0", "sel1", "sel2"];

const REPEATED_AURAL_HOLD_S: f64 = 1.5;
const CAVALRY_MIN_S: f64 = 1.1;
const CCHORD_REPEAT_S: f64 = 1.7;
const STALL_REPEAT_S: f64 = 2.6;
const CALLOUT_HOLD_S: f64 = 0.5;
const FLAPS_STILL_S: f64 = 0.3;

static REQUESTS: Mutex<Vec<String>> = Mutex::new(Vec::new());

pub fn request(name: &str) {
    if let Ok(mut queue) = REQUESTS.lock() {
        queue.push(name.to_owned());
    }
}

pub fn click_sound(event: &str) -> Option<&'static str> {
    CLICKS.iter().find(|(e, _)| e.eq_ignore_ascii_case(event)).map(|(_, c)| *c)
}

#[cfg(feature = "js")]
pub fn plugin_dir() -> Option<std::path::PathBuf> {
    crate::js_bridge::plugin_dir()
}

#[cfg(not(feature = "js"))]
pub fn plugin_dir() -> Option<std::path::PathBuf> {
    None
}

pub fn installed() -> bool {
    static INSTALLED: OnceLock<bool> = OnceLock::new();
    *INSTALLED.get_or_init(|| {
        plugin_dir().and_then(|d| Some(d.parent()?.parent()?.join("fmod"))).is_some_and(|d| has_sound_file(&d))
    })
}

fn has_sound_file(dir: &Path) -> bool {
    std::fs::read_dir(dir)
        .map(|entries| entries.flatten().any(|e| e.path().extension().is_some_and(|x| x.eq_ignore_ascii_case("snd"))))
        .unwrap_or(false)
}

pub fn replaces(event: &str) -> bool {
    event.starts_with("Engine_") || REPLACED.contains(&event)
}

const KT_PER_M_S: f64 = 1.943_844;
const FT_PER_M: f64 = 3.280_84;
const TLA_AT_DETENT: f64 = 1.;
const ANNOUNCEMENT_GAP_S: f64 = 60.;

const PHASE_TAKEOFF: i64 = 1;
const PHASE_CLIMB: i64 = 2;
const PHASE_CRUISE: i64 = 3;
const PHASE_DESCENT: i64 = 4;
const PHASE_APPROACH: i64 = 5;

pub const BANK_PARAMETERS: &[(&str, &str)] = &[
    ("sim/cockpit2/engine/indicators/N1_percent[", "fbw/snd/engine/indicators/N1_sound_scaled["),
    ("sim/flightmodel/engine/ENGN_N1_[", "fbw/snd/engine/ENGN_N1_scaled__["),
    ("sim/cockpit2/engine/indicators/N2_percent[", "fbw/snd/engine/indicators/N2_sound_scaled["),
    ("sim/flightmodel2/engines/thrust_reverser_deploy_ratio[", "fbw/snd/reverser_deploy_ratio["),
    ("sim/cockpit2/electrical/APU_N1_percent", "fbw/snd/apu_n1_percent"),
    ("sim/cockpit2/electrical/APU_EGT_c", "fbw/snd/apu_egt_c"),
    ("sim/cockpit2/electrical/APU_running", "fbw/snd/apu_running_now"),
    ("sim/sounds/flightmodel2/Airborne", "fbw/snd/airborne"),
    ("sim/flightmodel2/controls/speedbrake_ratio", "fbw/snd/speedbrake_deployed"),
    ("sim/cockpit2/controls/flap_handle_deploy_ratio", "fbw/snd/flap_deployed"),
    ("sim/cockpit2/electrical/Avionics_Power", "fbw/snd/avionics_powered"),
    ("sim/cockpit2/electrical/battery_on[", "fbw/snd/battery_power["),
    ("sim/cockpit2/Cooling/Packs_L", "fbw/snd/packs_flow"),
    ("sim/cockpit2/ATC/ATCGRD", "fbw/snd/atc_ground"),
    ("sim/cockpit2/ATC/ATCTWR", "fbw/snd/atc_tower"),
    ("sim/cockpit2/ATC/ATCAPP", "fbw/snd/atc_appr"),
    ("sim/cockpit2/ATC/ATCCTR", "fbw/snd/atc_cntr"),
    ("sim/cockpit2/controls/parking_brake_ratio", "fbw/snd/park_brake_set"),
    ("sim/cockpit2/gauges/indicators/radio_altimeter_height_ft_pilot ", "fbw/snd/radio_altitude_ft"),
    ("sim/weather/thunderstorm_percent", "fbw/snd/thunderstorm"),
    ("1-sim/flaphandle", "fbw/snd/flap_hdl"),
    ("1-sim/gearhandle", "fbw/snd/gear_hdl"),
    ("1-sim/speedbrakehandle", "fbw/snd/speedbrake_hdl"),
    ("AirbusFBW/throttle_input[", "fbw/snd/throttle_in["),
    ("sim/sounds/PA/PBoarding", "fbw/snd/pa_pboarding"),
    ("sim/sounds/PA/Boarding", "fbw/snd/pa_boarding"),
    ("sim/sounds/PA/Taxi", "fbw/snd/pa_taxi"),
    ("sim/sounds/PA/Takeoff", "fbw/snd/pa_takeoff"),
    ("sim/sounds/PA/After_Takeoff", "fbw/snd/pa_after_takeoff"),
    ("sim/sounds/PA/in_Cruise", "fbw/snd/pa_cruise"),
    ("sim/sounds/PA/Descent", "fbw/snd/pa_descent"),
    ("sim/sounds/PA/Approach", "fbw/snd/pa_approach"),
    ("sim/sounds/PA/After_Landing", "fbw/snd/pa_after_landing"),
    ("sim/sounds/PA/Gate_Arv", "fbw/snd/pa_gate_arv"),
    ("1-sim/lamp/223", "fbw/snd/crc_lt"),
    ("1-sim/lamp/224", "fbw/snd/sc_lmp"),
    ("AirbusFBW/RetardWarning", "fbw/snd/retard_on"),
    ("AirbusFBW/APWarning", "fbw/snd/ap_warning"),
    ("1-sim/elec/busStatus/DCecc", "fbw/snd/dc_ess_on"),
    ("1-sim/L/throttle", "fbw/snd/retd_thr"),
    ("sim/cockpit2/annunciators/stall_warning", "fbw/snd/stall_on"),
    ("sim/cockpit2/gauges/indicators/radio_altimeter_dh_lit_pilot", "fbw/snd/minimum_lit"),
    ("sim/cockpit/warnings/annunciators/GPWS", "fbw/snd/gpws_on"),
    ("sim/flightmodel/engine/ENGN_thro_use[0]", "fbw/snd/dsink_thr"),
    ("sim/cockpit/Warning/Low_Gear", "fbw/snd/low_gear"),
    ("sim/cockpit/Warning/Low_Flaps", "fbw/snd/low_flaps"),
    ("sim/cockpit/Warning/Low_Terrain", "fbw/snd/low_terrain"),
    ("sim/cockpit2/gauges/indicators/radio_altimeter_height_ft_pilot", "fbw/snd/radio_alt_callout"),
];

const PA_FLAGS: &[&str] = &[
    "sim/sounds/PA/PBoarding",
    "sim/sounds/PA/Boarding",
    "sim/sounds/PA/Taxi",
    "sim/sounds/PA/Takeoff",
    "sim/sounds/PA/After_Takeoff",
    "sim/sounds/PA/in_Cruise",
    "sim/sounds/PA/Descent",
    "sim/sounds/PA/Approach",
    "sim/sounds/PA/After_Landing",
    "sim/sounds/PA/Gate_Arv",
];

const FLAP_HANDLE_STEPS: f64 = 5.;

fn thunderstorm(weather_preset: i32) -> f64 {
    match weather_preset {
        8 => 1.,
        7 => 0.5,
        _ => 0.,
    }
}

pub fn bank_name(original: &str, base: &str) -> String {
    let (base, bracket) = match base.strip_suffix('[') {
        Some(b) => (b, "["),
        None => (base, ""),
    };
    let pad = original.len().saturating_sub(base.len() + bracket.len());
    format!("{base}{}{bracket}", "_".repeat(pad))
}

fn dataref_of(original: &str) -> String {
    let (o, b) = BANK_PARAMETERS.iter().find(|(o, _)| *o == original).expect("a bank parameter");
    bank_name(o, b).trim_end_matches('[').to_string()
}

const PACK_IDLE_N1: f64 = 17.;
const PACK_IDLE_N2: f64 = 76.;

fn through(x: f64, points: &[(f64, f64)]) -> f64 {
    let last = points.len() - 1;
    let i = points.windows(2).position(|w| x <= w[1].0).unwrap_or(last - 1);
    let ((x0, y0), (x1, y1)) = (points[i], points[i + 1]);
    y0 + (x - x0) * (y1 - y0) / (x1 - x0)
}

fn pack_n1(n1: f64, idle_n1: f64, toga_n1: f64) -> f64 {
    let n1 = if idle_n1 > 5. && toga_n1 > idle_n1 + 5. {
        through(n1, &[(0., 0.), (idle_n1, PACK_IDLE_N1), (toga_n1, 100.)])
    } else if toga_n1 > 50. {
        n1 * 100. / toga_n1
    } else {
        n1
    };
    n1.clamp(0., 110.)
}

fn pack_n2(n3: f64, idle_n3: f64) -> f64 {
    let n2 = if idle_n3 > 20. && idle_n3 < 95. { through(n3, &[(0., 0.), (idle_n3, PACK_IDLE_N2), (100., 100.)]) } else { n3 };
    n2.clamp(0., 110.)
}

const TOUCHDOWN_HOLD_S: f64 = 3.;

fn airborne(on_ground: bool, since_touchdown_s: f64) -> f64 {
    if !on_ground || since_touchdown_s < TOUCHDOWN_HOLD_S {
        1.
    } else {
        0.
    }
}

fn settled(ratio: f64, below: f64) -> f64 {
    if ratio < below {
        0.
    } else {
        ratio
    }
}

fn throttle_ratio(tla: f64) -> f64 {
    if tla.abs() < TLA_AT_DETENT {
        0.
    } else if (tla - TLA_TOGA).abs() < TLA_AT_DETENT {
        1.
    } else {
        (tla / TLA_TOGA).clamp(0., 1.)
    }
}

fn throttle_reverse(tla: f64) -> f64 {
    if tla > -TLA_AT_DETENT {
        0.
    } else {
        (tla / -TLA_REVERSE).clamp(-1., 0.)
    }
}

fn apu_running(was: bool, n: f64, previous_n: f64) -> bool {
    if was {
        !(n < 95. && n < previous_n)
    } else {
        n >= 99. || (n > 2. && n > previous_n)
    }
}

fn speedbrake_detent(previous: f64, armed: bool, handle: f64) -> f64 {
    if armed {
        return -1.;
    }
    [0., 0.5, 1.].iter().position(|d| (handle - d).abs() < 0.04).map_or(previous, |i| i as f64)
}

#[derive(Clone, Copy, Debug, PartialEq)]
enum Atc {
    Ground,
    Tower,
    Approach,
    Centre,
}

fn atc(powered: bool, on_ground: bool, ground_speed_kt: f64, agl_ft: f64, altitude_ft: f64) -> Option<Atc> {
    if !powered {
        None
    } else if on_ground {
        Some(if ground_speed_kt > 40. { Atc::Tower } else { Atc::Ground })
    } else if agl_ft < 2500. {
        Some(Atc::Tower)
    } else if altitude_ft < 18000. {
        Some(Atc::Approach)
    } else {
        Some(Atc::Centre)
    }
}

#[derive(Clone, Copy, Debug, Default)]
struct Flight {
    powered: bool,
    on_ground: bool,
    engines_running: bool,
    ground_speed_kt: f64,
    altitude_ft: f64,
    door_open: bool,
    boarding: bool,
    landing_lights: bool,
    phase: i64,
}

#[derive(Default)]
struct Aurals {
    retard_s: f64,
    pull_s: f64,
    sink_s: f64,
    dont_sink_s: f64,
    low_gear_s: f64,
    low_flaps_s: f64,
    low_terrain_s: f64,
    cavalry_s: f64,
    cchord_s: f64,
    stall_s: f64,
    callout_s: f64,
    callout_ra: f64,
    flaps_still_s: f64,
    flaps_moving: bool,
    last_flaps: Option<(f64, f64)>,
    last_sc: bool,
    last_cchord: bool,
    last_cchord_short: bool,
    last_stall: bool,
    last_cabin: Option<(f64, f64)>,
    single_chime: f64,
    cchord: f64,
    stall: f64,
    minimum: f64,
    seatbelt: f64,
    flaps_stop: f64,
    callouts: [f64; 12],
    clicks: [f64; 12],
}

struct AuralInputs {
    crc: bool,
    single_chime: bool,
    cavalry: bool,
    cchord: bool,
    cchord_short: bool,
    stall: bool,
    on_ground: bool,
    cabin: (f64, f64),
    flaps: (f64, f64),
}

impl Aurals {
    fn request(&mut self, name: &str) {
        if let Some((_, height)) = CALLOUTS.iter().find(|(n, _)| *n == name) {
            if let Some(i) = CALLOUT_HEIGHTS.iter().position(|h| h == height) {
                self.callouts[i] += 1.;
                self.callout_ra = *height as f64;
                self.callout_s = CALLOUT_HOLD_S;
            }
            return;
        }
        if let Some(i) = click_sound(name).and_then(|c| CLICK_SOUNDS.iter().position(|k| *k == c)) {
            self.clicks[i] += 1.;
            return;
        }
        match name {
            "new_retard" => self.retard_s = REPEATED_AURAL_HOLD_S,
            "aural_pullup_new" => self.pull_s = REPEATED_AURAL_HOLD_S,
            "aural_sink_rate_new" => self.sink_s = REPEATED_AURAL_HOLD_S,
            "aural_dontsink_new" => self.dont_sink_s = REPEATED_AURAL_HOLD_S,
            "aural_too_low_gear" => self.low_gear_s = REPEATED_AURAL_HOLD_S,
            "aural_too_low_flaps" => self.low_flaps_s = REPEATED_AURAL_HOLD_S,
            "aural_too_low_terrain" => self.low_terrain_s = REPEATED_AURAL_HOLD_S,
            "aural_minimumnew" => self.minimum += 1.,
            _ => {}
        }
    }

    fn step(&mut self, i: &AuralInputs, dt: f64) {
        for t in [&mut self.retard_s, &mut self.pull_s, &mut self.sink_s, &mut self.dont_sink_s, &mut self.low_gear_s, &mut self.low_flaps_s, &mut self.low_terrain_s, &mut self.callout_s] {
            *t = (*t - dt).max(0.);
        }
        if i.single_chime && !self.last_sc {
            self.single_chime += 1.;
        }
        self.last_sc = i.single_chime;
        self.cavalry_s = if i.cavalry { CAVALRY_MIN_S } else { (self.cavalry_s - dt).max(0.) };
        if i.cchord && (!self.last_cchord || self.cchord_s <= 0.) {
            self.cchord += 1.;
            self.cchord_s = CCHORD_REPEAT_S;
        }
        if i.cchord_short && !self.last_cchord_short {
            self.cchord += 1.;
        }
        self.cchord_s = (self.cchord_s - dt).max(0.);
        self.last_cchord = i.cchord;
        self.last_cchord_short = i.cchord_short;
        let stall = i.stall && !i.on_ground;
        if stall && (!self.last_stall || self.stall_s <= 0.) {
            self.stall += 1.;
            self.stall_s = STALL_REPEAT_S;
        }
        self.stall_s = (self.stall_s - dt).max(0.);
        self.last_stall = stall;
        if self.last_cabin.is_some_and(|c| c != i.cabin) {
            self.seatbelt += 1.;
        }
        self.last_cabin = Some(i.cabin);
        let moved = self.last_flaps.is_some_and(|(f, s)| (f - i.flaps.0).abs() > 1e-3 || (s - i.flaps.1).abs() > 1e-3);
        self.last_flaps = Some(i.flaps);
        if moved {
            self.flaps_moving = true;
            self.flaps_still_s = 0.;
        } else if self.flaps_moving {
            self.flaps_still_s += dt;
            if self.flaps_still_s >= FLAPS_STILL_S {
                self.flaps_moving = false;
                self.flaps_stop += 1.;
            }
        }
        let _ = i.crc;
    }

    fn callout_ra(&self, radio_alt: f64) -> f64 {
        if self.callout_s > 0. {
            self.callout_ra
        } else {
            radio_alt
        }
    }
}

#[derive(Default)]
struct Announcements {
    done: u8,
    heard: bool,
    airborne_seen: bool,
    since_last_s: f64,
    was_boarding: bool,
    was_running: bool,
}

impl Announcements {
    fn due(&self, step: u8, f: &Flight) -> bool {
        let after_landing = f.on_ground && self.airborne_seen;
        match step {
            1 => f.on_ground && f.powered && f.door_open && !f.engines_running && !f.boarding,
            2 => f.on_ground && f.boarding && !f.engines_running,
            3 => f.on_ground && f.engines_running && !f.door_open && f.ground_speed_kt > 3.,
            4 => f.on_ground && f.engines_running && f.ground_speed_kt > 3. && (f.landing_lights || f.phase == PHASE_TAKEOFF),
            5 => !f.on_ground && f.phase == PHASE_CLIMB && f.altitude_ft > 10000.,
            6 => !f.on_ground && f.phase == PHASE_CRUISE,
            7 => !f.on_ground && f.phase == PHASE_DESCENT,
            8 => !f.on_ground && f.phase == PHASE_APPROACH,
            9 => after_landing && f.ground_speed_kt < 40.,
            10 => after_landing && !f.engines_running,
            11 => after_landing && f.door_open,
            _ => false,
        }
    }

    fn reset(&mut self) {
        *self = Self { since_last_s: ANNOUNCEMENT_GAP_S, ..Self::default() };
    }

    fn step(&mut self, f: &Flight, enabled: bool, dt: f64) -> u8 {
        let boarding_started = f.boarding && !self.was_boarding;
        let engines_started = f.engines_running && !self.was_running;
        self.was_boarding = f.boarding;
        self.was_running = f.engines_running;
        if self.done >= 10 && f.on_ground && (boarding_started || engines_started) {
            self.reset();
            self.was_boarding = f.boarding;
            self.was_running = f.engines_running;
        }
        self.airborne_seen |= !f.on_ground;
        self.since_last_s += dt;
        if self.since_last_s >= ANNOUNCEMENT_GAP_S {
            if let Some(step) = (self.done + 1..=11).find(|&s| self.due(s, f)) {
                self.done = step;
                self.heard = enabled;
                self.since_last_s = 0.;
            }
        }
        if !enabled {
            self.heard = false;
        }
        if self.heard {
            self.done
        } else {
            0
        }
    }
}

struct Inputs {
    flaps_percent: VariableIdentifier,
    flaps_handle: VariableIdentifier,
    gear_handle: VariableIdentifier,
    park_brake: VariableIdentifier,
    spoilers_handle: VariableIdentifier,
    spoilers_armed: VariableIdentifier,
    spoilers: Vec<VariableIdentifier>,
    apu_n: VariableIdentifier,
    ac_1: VariableIdentifier,
    dc_ess: VariableIdentifier,
    pack_flows: [VariableIdentifier; 2],
    tla: [VariableIdentifier; 4],
    toga_n1: VariableIdentifier,
    idle_n1: VariableIdentifier,
    idle_n3: VariableIdentifier,
    apu_egt: VariableIdentifier,
    fbw_n1: [VariableIdentifier; 4],
    fbw_n3: [VariableIdentifier; 4],
    radio_alt: Option<DataRef>,
    weather_preset: Option<DataRef>,
    reversers: [VariableIdentifier; 2],
    engine_state: [VariableIdentifier; 4],
    phase: VariableIdentifier,
    announcements: VariableIdentifier,
    atc_level: VariableIdentifier,
    boarding: VariableIdentifier,
    doors: Vec<VariableIdentifier>,
    on_ground: Option<DataRef>,
    ground_speed: Option<DataRef>,
    agl: Option<DataRef>,
    elevation: Option<DataRef>,
    burning: Option<DataRef>,
    landing_lights: Option<DataRef>,
    flight_time: Option<DataRef>,
    slats_percent: VariableIdentifier,
    fwc_crc: VariableIdentifier,
    fwc_sc: VariableIdentifier,
    fwc_cavalry: VariableIdentifier,
    alt_deviation: VariableIdentifier,
    alt_deviation_short: VariableIdentifier,
    stall_aural: VariableIdentifier,
    seatbelt_switch: VariableIdentifier,
    no_smoking_memo: VariableIdentifier,
}

struct AuralOutputs {
    crc: Value,
    single_chime: Value,
    cavalry: Value,
    cchord: Value,
    stall: Value,
    retard: Value,
    minimum: Value,
    gpws_pull: Value,
    gpws_sink: Value,
    gpws_dont_sink: Value,
    gpws_low: Value,
    seatbelt: Value,
    flaps_moving: Value,
    flaps_stop: Value,
    callouts: Vec<Value>,
    clicks: Vec<Value>,
    crc_lamp: Value,
    sc_lamp: Value,
    retard_on: Value,
    ap_warning: Value,
    dc_ess_on: Value,
    retard_thr: Value,
    stall_on: Value,
    minimum_lit: Value,
    gpws_on: Value,
    dsink_thr: Value,
    low_gear: Value,
    low_flaps: Value,
    low_terrain: Value,
    radio_alt_callout: Value,
}

struct Outputs {
    flap_surface: Value,
    flap_lever: Value,
    gear_lever: Value,
    pbrake_lever: Value,
    speedbrake_lever: Value,
    speedbrake_surface: Value,
    apu_running: Value,
    avionics_power: Value,
    dc_power: Value,
    packs_on: Value,
    throttle_ratio: Value,
    throttle_reverse: Value,
    reverser_2: Value,
    reverser_3: Value,
    engine_starting: Value,
    engine_shutting: Value,
    pa: Value,
    atc: [Value; 4],
    n1_sound: Value,
    n1_engine: Value,
    n2_sound: Value,
    reverser_deploy: Value,
    apu_n1: Value,
    apu_egt: Value,
    apu_running_now: Value,
    airborne: Value,
    speedbrake_deployed: Value,
    flap_deployed: Value,
    avionics_powered: Value,
    battery_power: Value,
    packs_flow: Value,
    atc_bank: [Value; 4],
    park_brake_set: Value,
    radio_altitude: Value,
    thunderstorm: Value,
    flap_handle: Value,
    gear_handle: Value,
    speedbrake_handle: Value,
    throttle_input: Value,
    _pa_flags: Vec<Value>,
}

pub struct SoundPack {
    inputs: Inputs,
    outputs: Outputs,
    aural_outputs: AuralOutputs,
    aurals: Aurals,
    _published: Published,
    apu_on: bool,
    apu_n: f64,
    speedbrake: f64,
    announcements: Announcements,
    flight_time_s: f64,
    since_touchdown_s: f64,
}

impl SoundPack {
    pub fn new(vars: &mut Vars, xplm: &Xplm) -> Option<Self> {
        if !installed() {
            return None;
        }
        let mut get = |name: String| vars.get(name);
        let inputs = Inputs {
            flaps_percent: get("LEFT_FLAPS_POSITION_PERCENT".into()),
            flaps_handle: get("FLAPS_HANDLE_INDEX".into()),
            gear_handle: get("GEAR_HANDLE_POSITION".into()),
            park_brake: get("PARK_BRAKE_LEVER_POS".into()),
            spoilers_handle: get("SPOILERS_HANDLE_POSITION".into()),
            spoilers_armed: get("SPOILERS_ARMED".into()),
            spoilers: ["LEFT", "RIGHT"].iter().flat_map(|side| (1..=8).map(move |k| format!("HYD_SPOILER_{k}_{side}_DEFLECTION"))).map(&mut get).collect(),
            apu_n: get("APU_N_RAW".into()),
            ac_1: get("ELEC_AC_1_BUS_IS_POWERED".into()),
            dc_ess: get("ELEC_DC_ESS_BUS_IS_POWERED".into()),
            pack_flows: [get("PNEU_PACK_1_FLOW_VALVE_1_FLOW_RATE".into()), get("PNEU_PACK_2_FLOW_VALVE_1_FLOW_RATE".into())],
            tla: [1, 2, 3, 4].map(|n| get(format!("AUTOTHRUST_TLA:{n}"))),
            toga_n1: get("AUTOTHRUST_THRUST_LIMIT_TOGA".into()),
            idle_n1: get("ENGINE_IDLE_N1".into()),
            idle_n3: get("ENGINE_IDLE_N3".into()),
            apu_egt: get("APU_EGT".into()),
            fbw_n1: [1, 2, 3, 4].map(|n| get(format!("ENGINE_N1:{n}"))),
            fbw_n3: [1, 2, 3, 4].map(|n| get(format!("ENGINE_N3:{n}"))),
            reversers: [get("REVERSER_2_POSITION".into()), get("REVERSER_3_POSITION".into())],
            engine_state: [1, 2, 3, 4].map(|n| get(format!("ENGINE_STATE:{n}"))),
            phase: get("FMGC_FLIGHT_PHASE".into()),
            announcements: get("SOUND_ANNOUNCEMENTS_ENABLED".into()),
            atc_level: get("SOUND_LEVEL_ATC".into()),
            boarding: get("BOARDING_STARTED_BY_USR".into()),
            doors: (0..16).map(|p| get(format!("INTERACTIVE POINT OPEN:{p}"))).collect(),
            radio_alt: xplm.find("sim/cockpit2/gauges/indicators/radio_altimeter_height_ft_pilot"),
            weather_preset: xplm.find("sim/weather/region/weather_preset"),
            on_ground: xplm.find("sim/flightmodel/failures/onground_any"),
            ground_speed: xplm.find("sim/flightmodel/position/groundspeed"),
            agl: xplm.find("sim/flightmodel/position/y_agl"),
            elevation: xplm.find("sim/flightmodel/position/elevation"),
            burning: xplm.find("sim/flightmodel2/engines/engine_is_burning_fuel"),
            landing_lights: xplm.find("sim/cockpit2/switches/landing_lights_on"),
            flight_time: xplm.find("sim/time/total_flight_time_sec"),
            slats_percent: get("LEFT_SLATS_POSITION_PERCENT".into()),
            fwc_crc: get("FWC_CRC".into()),
            fwc_sc: get("FWC_SC".into()),
            fwc_cavalry: get("FWC_CAVALRY_CHARGE".into()),
            alt_deviation: get("ALT_DEVIATION".into()),
            alt_deviation_short: get("ALT_DEVIATION_SHORT".into()),
            stall_aural: get("AUDIO_ROP_MAX_BRAKING".into()),
            seatbelt_switch: get("CABIN SEATBELTS ALERT SWITCH".into()),
            no_smoking_memo: get("NO_SMOKING_MEMO".into()),
        };
        let mut p = Published::default();
        let mut n = |name: &str| p.number(&format!("fbw/snd/{name}"), 0., false);
        let outputs = Outputs {
            flap_surface: n("flap_surface"),
            flap_lever: n("flap_lever"),
            gear_lever: n("gear_lever"),
            pbrake_lever: n("pbrake_lever"),
            speedbrake_lever: n("speedbrake_lever"),
            speedbrake_surface: n("speedbrake_surface"),
            apu_running: n("apu_running"),
            avionics_power: n("avionics_power"),
            dc_power: n("dc_power"),
            packs_on: n("packs_on"),
            throttle_ratio: n("throttle_ratio"),
            throttle_reverse: n("throttle_reverse"),
            reverser_2: n("reverser_2"),
            reverser_3: n("reverser_3"),
            pa: n("pa"),
            atc: [n("atc_grd"), n("atc_twr"), n("atc_app"), n("atc_ctr")],
            engine_starting: p.numbers("fbw/snd/engine_starting", 4),
            engine_shutting: p.numbers("fbw/snd/engine_shutting", 4),
            n1_sound: p.numbers(&dataref_of("sim/cockpit2/engine/indicators/N1_percent["), 4),
            n1_engine: p.numbers(&dataref_of("sim/flightmodel/engine/ENGN_N1_["), 4),
            n2_sound: p.numbers(&dataref_of("sim/cockpit2/engine/indicators/N2_percent["), 4),
            reverser_deploy: p.numbers(&dataref_of("sim/flightmodel2/engines/thrust_reverser_deploy_ratio["), 4),
            apu_n1: p.number(&dataref_of("sim/cockpit2/electrical/APU_N1_percent"), 0., false),
            apu_egt: p.number(&dataref_of("sim/cockpit2/electrical/APU_EGT_c"), 0., false),
            apu_running_now: p.number(&dataref_of("sim/cockpit2/electrical/APU_running"), 0., false),
            airborne: p.number(&dataref_of("sim/sounds/flightmodel2/Airborne"), 0., false),
            speedbrake_deployed: p.number(&dataref_of("sim/flightmodel2/controls/speedbrake_ratio"), 0., false),
            flap_deployed: p.number(&dataref_of("sim/cockpit2/controls/flap_handle_deploy_ratio"), 0., false),
            avionics_powered: p.number(&dataref_of("sim/cockpit2/electrical/Avionics_Power"), 0., false),
            battery_power: p.numbers(&dataref_of("sim/cockpit2/electrical/battery_on["), 1),
            packs_flow: p.number(&dataref_of("sim/cockpit2/Cooling/Packs_L"), 0., false),
            atc_bank: ["ATCGRD", "ATCTWR", "ATCAPP", "ATCCTR"].map(|a| p.number(&dataref_of(&format!("sim/cockpit2/ATC/{a}")), 0., false)),
            park_brake_set: p.number(&dataref_of("sim/cockpit2/controls/parking_brake_ratio"), 0., false),
            radio_altitude: p.number(&dataref_of("sim/cockpit2/gauges/indicators/radio_altimeter_height_ft_pilot "), 0., false),
            thunderstorm: p.number(&dataref_of("sim/weather/thunderstorm_percent"), 0., false),
            flap_handle: p.number(&dataref_of("1-sim/flaphandle"), 0., false),
            gear_handle: p.number(&dataref_of("1-sim/gearhandle"), 0., false),
            speedbrake_handle: p.number(&dataref_of("1-sim/speedbrakehandle"), 0., false),
            throttle_input: p.numbers(&dataref_of("AirbusFBW/throttle_input["), 4),
            _pa_flags: PA_FLAGS.iter().map(|f| p.number(&dataref_of(f), 0., false)).collect(),
        };
        let mut n = |name: &str| p.number(&format!("fbw/snd/{name}"), 0., false);
        let (crc, single_chime, cavalry, cchord, stall, retard, minimum) =
            (n("aural_crc"), n("aural_single_chime"), n("aural_cavalry"), n("aural_cchord"), n("aural_stall"), n("aural_retard"), n("aural_minimum"));
        let (gpws_pull, gpws_sink, gpws_dont_sink, gpws_low) = (n("gpws_pull"), n("gpws_sink"), n("gpws_dont_sink"), n("gpws_low"));
        let (seatbelt, flaps_moving, flaps_stop) = (n("seatbelt_chime"), n("flaps_moving"), n("flaps_stop"));
        let callouts: Vec<Value> = CALLOUT_HEIGHTS.iter().map(|h| n(&format!("callout_{h}"))).collect();
        let clicks: Vec<Value> = CLICK_SOUNDS.iter().map(|c| n(&format!("click_{c}"))).collect();
        let mut bank = |original: &str, value: f64| p.number(&dataref_of(original), value, false);
        let aural_outputs = AuralOutputs {
            crc,
            single_chime,
            cavalry,
            cchord,
            stall,
            retard,
            minimum,
            gpws_pull,
            gpws_sink,
            gpws_dont_sink,
            gpws_low,
            seatbelt,
            flaps_moving,
            flaps_stop,
            callouts,
            clicks,
            crc_lamp: bank("1-sim/lamp/223", 1.),
            sc_lamp: bank("1-sim/lamp/224", 1.),
            retard_on: bank("AirbusFBW/RetardWarning", 1.),
            ap_warning: bank("AirbusFBW/APWarning", 1.),
            dc_ess_on: bank("1-sim/elec/busStatus/DCecc", 1.),
            retard_thr: bank("1-sim/L/throttle", 0.5),
            stall_on: bank("sim/cockpit2/annunciators/stall_warning", 1.),
            minimum_lit: bank("sim/cockpit2/gauges/indicators/radio_altimeter_dh_lit_pilot", 1.),
            gpws_on: bank("sim/cockpit/warnings/annunciators/GPWS", 1.),
            dsink_thr: bank("sim/flightmodel/engine/ENGN_thro_use[0]", 1.),
            low_gear: bank("sim/cockpit/Warning/Low_Gear", 0.),
            low_flaps: bank("sim/cockpit/Warning/Low_Flaps", 0.),
            low_terrain: bank("sim/cockpit/Warning/Low_Terrain", 0.),
            radio_alt_callout: bank("sim/cockpit2/gauges/indicators/radio_altimeter_height_ft_pilot", 0.),
        };
        crate::log("sound pack: fmod/ found; FlyByWire's engine, APU, gear, wind, lever, warning and callout sounds give way to it, fbw/snd/* published");
        for path in ["sim/operation/sound/engine_volume_ratio", "sim/operation/sound/fan_volume_ratio"] {
            if let Some(d) = xplm.find(path).filter(|&d| xplm.get_f(d) <= 0.) {
                xplm.set_f(d, 1.);
                crate::log(&format!("sound pack: {path} was 0, left there by an earlier version's mute; now 1"));
            }
        }
        let mut announcements = Announcements::default();
        announcements.reset();
        Some(Self { inputs, outputs, aural_outputs, aurals: Aurals::default(), _published: p, apu_on: false, apu_n: 0., speedbrake: 0., announcements, flight_time_s: 0., since_touchdown_s: TOUCHDOWN_HOLD_S })
    }

    pub fn update(&mut self, vars: &mut Vars, xplm: &Xplm, dt: f64) {
        let i = &self.inputs;
        let o = &self.outputs;
        let mut read = |id: &VariableIdentifier| vars.read(id);

        let flaps = (read(&i.flaps_percent) / 100.).clamp(0., 1.);
        published::set(o.flap_surface, flaps);
        published::set(o.flap_deployed, flaps);
        let flaps_handle = read(&i.flaps_handle).round();
        published::set(o.flap_lever, flaps_handle);
        published::set(o.flap_handle, (flaps_handle / FLAP_HANDLE_STEPS).clamp(0., 1.));
        let gear_handle = read(&i.gear_handle).round();
        published::set(o.gear_lever, gear_handle);
        published::set(o.gear_handle, gear_handle);
        let park_brake = read(&i.park_brake).round();
        published::set(o.pbrake_lever, park_brake);
        published::set(o.park_brake_set, park_brake);
        self.speedbrake = speedbrake_detent(self.speedbrake, read(&i.spoilers_armed) > 0.5, read(&i.spoilers_handle));
        published::set(o.speedbrake_lever, self.speedbrake);
        published::set(o.speedbrake_handle, read(&i.spoilers_handle).clamp(0., 1.));
        let spoilers = i.spoilers.iter().map(&mut read).sum::<f64>() / i.spoilers.len().max(1) as f64;
        published::set(o.speedbrake_surface, settled(spoilers, 0.02));
        published::set(o.speedbrake_deployed, settled(spoilers, 0.02));

        let apu_n = read(&i.apu_n);
        self.apu_on = apu_running(self.apu_on, apu_n, self.apu_n);
        self.apu_n = apu_n;
        published::set(o.apu_running, self.apu_on as u8 as f64);
        published::set(o.apu_running_now, self.apu_on as u8 as f64);
        published::set(o.apu_n1, apu_n.clamp(0., 110.));
        published::set(o.apu_egt, read(&i.apu_egt));

        let ac_1 = read(&i.ac_1) > 0.5;
        published::set(o.avionics_power, ac_1 as u8 as f64);
        published::set(o.avionics_powered, ac_1 as u8 as f64);
        let dc = (read(&i.dc_ess) > 0.5) as u8 as f64;
        published::set(o.dc_power, dc);
        published::set_at(o.battery_power, 0, dc);
        let pack_flow: f64 = i.pack_flows.iter().map(&mut read).sum();
        published::set(o.packs_on, (pack_flow > 0.05) as u8 as f64);
        published::set(o.packs_flow, if pack_flow > 0.05 { 2. } else { 0. });

        let tla: Vec<f64> = i.tla.iter().map(&mut read).collect();
        published::set(o.throttle_ratio, throttle_ratio(tla.iter().copied().fold(f64::MIN, f64::max)));
        published::set(o.throttle_reverse, throttle_reverse(tla[1].min(tla[2])));
        for (k, t) in tla.iter().enumerate() {
            published::set_at(o.throttle_input, k, throttle_ratio(*t));
        }
        let reversers = [settled(read(&i.reversers[0]), 0.01), settled(read(&i.reversers[1]), 0.01)];
        published::set(o.reverser_2, reversers[0]);
        published::set(o.reverser_3, reversers[1]);
        published::set_at(o.reverser_deploy, 1, reversers[0]);
        published::set_at(o.reverser_deploy, 2, reversers[1]);
        for (k, id) in i.engine_state.iter().enumerate() {
            let state = read(id).round() as i64;
            published::set_at(o.engine_starting, k, matches!(state, 2 | 3) as u8 as f64);
            published::set_at(o.engine_shutting, k, (state == 4) as u8 as f64);
        }

        let (toga_n1, idle_n1, idle_n3) = (read(&i.toga_n1), read(&i.idle_n1), read(&i.idle_n3));
        for k in 0..4 {
            let s = pack_n1(read(&i.fbw_n1[k]), idle_n1, toga_n1);
            published::set_at(o.n1_sound, k, s);
            published::set_at(o.n1_engine, k, s);
            published::set_at(o.n2_sound, k, pack_n2(read(&i.fbw_n3[k]), idle_n3));
        }
        published::set(o.radio_altitude, i.radio_alt.map_or(0., |d| xplm.get_f(d) as f64));
        published::set(o.thunderstorm, thunderstorm(i.weather_preset.map_or(0, |d| xplm.get_i(d))));

        let mut burning = [0; 4];
        if let Some(d) = i.burning {
            xplm.get_vi(d, &mut burning);
        }
        let flight = Flight {
            powered: ac_1,
            on_ground: i.on_ground.is_none_or(|d| xplm.get_i(d) != 0),
            engines_running: burning.iter().any(|b| *b != 0),
            ground_speed_kt: i.ground_speed.map_or(0., |d| xplm.get_f(d) as f64 * KT_PER_M_S),
            altitude_ft: i.elevation.map_or(0., |d| xplm.get_d(d) * FT_PER_M),
            door_open: i.doors.iter().any(|d| read(d) > 10.),
            boarding: read(&i.boarding) > 0.5,
            landing_lights: i.landing_lights.is_some_and(|d| xplm.get_i(d) != 0),
            phase: read(&i.phase).round() as i64,
        };
        self.since_touchdown_s = if flight.on_ground { self.since_touchdown_s + dt } else { 0. };
        published::set(o.airborne, airborne(flight.on_ground, self.since_touchdown_s));
        let enabled = read(&i.announcements) > 0.5;
        let flight_time_s = i.flight_time.map_or(0., |d| xplm.get_f(d) as f64);
        if flight_time_s + 1. < self.flight_time_s {
            self.announcements.reset();
        }
        self.flight_time_s = flight_time_s;
        published::set(o.pa, self.announcements.step(&flight, enabled, dt) as f64);

        let requests = REQUESTS.lock().map(|mut q| std::mem::take(&mut *q)).unwrap_or_default();
        for name in &requests {
            self.aurals.request(name);
        }
        let aural_inputs = AuralInputs {
            crc: read(&i.fwc_crc) > 0.5,
            single_chime: read(&i.fwc_sc) > 0.5,
            cavalry: read(&i.fwc_cavalry) > 0.5,
            cchord: read(&i.alt_deviation) > 0.5,
            cchord_short: read(&i.alt_deviation_short) > 0.5,
            stall: read(&i.stall_aural) > 0.5,
            on_ground: flight.on_ground,
            cabin: (read(&i.seatbelt_switch).round(), read(&i.no_smoking_memo).round()),
            flaps: (read(&i.flaps_percent), read(&i.slats_percent)),
        };
        self.aurals.step(&aural_inputs, dt);
        let au = &self.aurals;
        let ao = &self.aural_outputs;
        published::set(ao.crc, aural_inputs.crc as u8 as f64);
        published::set(ao.single_chime, au.single_chime);
        published::set(ao.cavalry, (au.cavalry_s > 0.) as u8 as f64);
        published::set(ao.cchord, au.cchord);
        published::set(ao.stall, au.stall);
        published::set(ao.retard, (au.retard_s > 0.) as u8 as f64);
        published::set(ao.minimum, au.minimum);
        published::set(ao.gpws_pull, (au.pull_s > 0.) as u8 as f64);
        published::set(ao.gpws_sink, (au.sink_s > 0.) as u8 as f64);
        published::set(ao.gpws_dont_sink, (au.dont_sink_s > 0.) as u8 as f64);
        published::set(ao.low_gear, (au.low_gear_s > 0.) as u8 as f64);
        published::set(ao.low_flaps, (au.low_flaps_s > 0.) as u8 as f64);
        published::set(ao.low_terrain, (au.low_terrain_s > 0.) as u8 as f64);
        published::set(ao.gpws_low, (au.low_gear_s > 0. || au.low_flaps_s > 0. || au.low_terrain_s > 0.) as u8 as f64);
        published::set(ao.seatbelt, au.seatbelt);
        published::set(ao.flaps_moving, au.flaps_moving as u8 as f64);
        published::set(ao.flaps_stop, au.flaps_stop);
        for (v, c) in ao.callouts.iter().zip(au.callouts) {
            published::set(*v, c);
        }
        for (v, c) in ao.clicks.iter().zip(au.clicks) {
            published::set(*v, c);
        }
        published::set(ao.radio_alt_callout, au.callout_ra(i.radio_alt.map_or(0., |d| xplm.get_f(d) as f64)));

        let agl_ft = i.agl.map_or(0., |d| xplm.get_f(d) as f64 * FT_PER_M);
        let chatter = read(&i.atc_level) > 0.;
        let now = atc(flight.powered, flight.on_ground, flight.ground_speed_kt, agl_ft, flight.altitude_ft).filter(|_| chatter);
        for ((value, bank), which) in o.atc.iter().zip(&o.atc_bank).zip([Atc::Ground, Atc::Tower, Atc::Approach, Atc::Centre]) {
            let on = (now == Some(which)) as u8 as f64;
            published::set(*value, on);
            published::set(*bank, on);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn idle_and_take_off_thrust_land_on_the_packs_own_idle_and_full_n1() {
        assert!((pack_n1(20., 20., 83.8) - PACK_IDLE_N1).abs() < 1e-9, "the idle recording, not the layer above it");
        assert!((pack_n1(83.8, 20., 83.8) - 100.).abs() < 1e-9);
        assert!((pack_n1(10., 20., 83.8) - 8.5).abs() < 1e-9, "spool-up runs from 0 to the pack's idle");
        assert!(pack_n1(51.9, 20., 83.8) > 55. && pack_n1(51.9, 20., 83.8) < 60.);
        assert!((pack_n1(41.9, 0., 83.8) - 50.).abs() < 1e-9, "no idle figure yet: straight scale to take-off thrust");
        assert_eq!(pack_n1(40., 0., 0.), 40.);
        assert_eq!(pack_n1(-1., 20., 83.8), 0.);
    }

    #[test]
    fn take_off_thrust_on_flybywires_own_scale_is_the_top_of_the_pack_and_climb_sits_below_it() {
        assert!((pack_n1(83.8, 19.8, 83.8) - 100.).abs() < 1e-9);
        let climb = pack_n1(80.5, 19.8, 83.8);
        assert!(climb > 90. && climb < 96., "{climb}");
        assert!(pack_n1(88., 19.8, 83.8) > 100., "an N1 off FlyByWire's scale is what pinned the sound at full power");
    }

    #[test]
    fn thunder_follows_x_planes_weather_preset() {
        assert_eq!(thunderstorm(8), 1.);
        assert_eq!(thunderstorm(7), 0.5);
        assert_eq!(thunderstorm(3), 0.);
    }

    #[test]
    fn the_start_layers_are_silent_at_our_idle_n3() {
        assert!((pack_n2(66., 66.) - PACK_IDLE_N2).abs() < 1e-9);
        assert!(pack_n2(66., 66.) > 74.788, "the pack's start layers fade out by 74.8");
        assert!((pack_n2(33., 66.) - 38.).abs() < 1e-9);
        assert!((pack_n2(100., 66.) - 100.).abs() < 1e-9);
        assert_eq!(pack_n2(50., 0.), 50.);
    }

    #[test]
    fn a_touchdown_is_heard_for_a_moment_after_the_wheels_meet_the_ground() {
        assert_eq!(airborne(false, 0.), 1.);
        assert_eq!(airborne(true, 1.), 1.);
        assert_eq!(airborne(true, 3.5), 0.);
    }

    #[test]
    fn every_bank_parameter_keeps_its_length_so_the_bank_stays_valid() {
        for (original, base) in BANK_PARAMETERS {
            let name = bank_name(original, base);
            assert_eq!(name.len(), original.len(), "{original} -> {name}");
            assert_eq!(name.ends_with('['), original.ends_with('['), "{name}");
            assert!(name.starts_with("fbw/snd/"), "{name}");
        }
        assert_eq!(bank_name("sim/flightmodel/engine/ENGN_N1_[", "fbw/snd/engine/ENGN_N1_scaled__["), "fbw/snd/engine/ENGN_N1_scaled__[");
        assert_eq!(bank_name("sim/cockpit2/ATC/ATCGRD", "fbw/snd/atc_ground"), "fbw/snd/atc_ground_____");
    }

    #[test]
    fn the_levers_land_exactly_on_the_detents_the_pack_tests_for() {
        assert_eq!(throttle_ratio(0.6), 0.);
        assert_eq!(throttle_ratio(44.5), 1.);
        assert!(throttle_ratio(25.) > 0.55 && throttle_ratio(24.) < 0.55, "CLB");
        assert!(throttle_ratio(35.) > 0.76 && throttle_ratio(34.) < 0.76, "FLX/MCT");
        assert_eq!(throttle_reverse(-0.5), 0.);
        assert_eq!(throttle_reverse(-20.), -1.);
        assert!(throttle_reverse(-12.) < -0.5);
    }

    #[test]
    fn the_apu_runs_from_the_start_of_its_spool_up_to_the_start_of_its_spool_down() {
        let mut on = false;
        let mut n = 0.;
        let mut trace = Vec::new();
        for next in [0., 0., 3., 20., 60., 100., 100., 100., 100., 99.9, 97., 90., 40., 0.] {
            on = apu_running(on, next, n);
            n = next;
            trace.push(on);
        }
        assert_eq!(trace, [false, false, true, true, true, true, true, true, true, true, true, false, false, false]);
        assert!(apu_running(false, 100., 100.), "loaded with the APU already running");
    }

    #[test]
    fn the_speedbrake_lever_clicks_only_into_its_detents() {
        assert_eq!(speedbrake_detent(0., true, 0.), -1.);
        assert_eq!(speedbrake_detent(-1., false, 0.), 0.);
        assert_eq!(speedbrake_detent(0., false, 0.3), 0.);
        assert_eq!(speedbrake_detent(0., false, 0.52), 1.);
        assert_eq!(speedbrake_detent(1., false, 0.8), 1.);
        assert_eq!(speedbrake_detent(1., false, 0.99), 2.);
    }

    #[test]
    fn atc_chatter_follows_the_flight() {
        assert_eq!(atc(false, true, 0., 0., 0.), None);
        assert_eq!(atc(true, true, 10., 0., 0.), Some(Atc::Ground));
        assert_eq!(atc(true, true, 120., 0., 0.), Some(Atc::Tower));
        assert_eq!(atc(true, false, 160., 1500., 1600.), Some(Atc::Tower));
        assert_eq!(atc(true, false, 250., 8000., 9000.), Some(Atc::Approach));
        assert_eq!(atc(true, false, 480., 35000., 36000.), Some(Atc::Centre));
    }

    fn run(a: &mut Announcements, f: Flight, seconds: u32) -> Vec<u8> {
        let mut heard = Vec::new();
        for _ in 0..seconds {
            let pa = a.step(&f, true, 1.);
            if heard.last() != Some(&pa) {
                heard.push(pa);
            }
        }
        heard
    }

    #[test]
    fn a_whole_flight_hears_every_announcement_once_and_in_order() {
        let mut a = Announcements::default();
        a.reset();
        let gate = Flight { powered: true, on_ground: true, door_open: true, ..Flight::default() };
        let mut heard = run(&mut a, gate, 100);
        heard.extend(run(&mut a, Flight { boarding: true, ..gate }, 100));
        let taxi = Flight { powered: true, on_ground: true, engines_running: true, ground_speed_kt: 12., boarding: true, ..Flight::default() };
        heard.extend(run(&mut a, taxi, 100));
        heard.extend(run(&mut a, Flight { landing_lights: true, ..taxi }, 100));
        let air = Flight { powered: true, engines_running: true, ground_speed_kt: 300., altitude_ft: 15000., boarding: true, ..Flight::default() };
        for phase in [PHASE_CLIMB, PHASE_CRUISE, PHASE_DESCENT, PHASE_APPROACH] {
            heard.extend(run(&mut a, Flight { phase, ..air }, 100));
        }
        heard.extend(run(&mut a, Flight { on_ground: true, ground_speed_kt: 20., ..air }, 100));
        heard.extend(run(&mut a, Flight { on_ground: true, engines_running: false, ground_speed_kt: 0., ..air }, 100));
        heard.extend(run(&mut a, Flight { on_ground: true, engines_running: false, ground_speed_kt: 0., door_open: true, ..air }, 100));
        heard.dedup();
        assert_eq!(heard, [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11]);
    }

    #[test]
    fn a_flight_loaded_in_the_air_starts_at_its_own_phase_and_a_new_one_starts_over() {
        let mut a = Announcements::default();
        a.reset();
        let cruise = Flight { powered: true, engines_running: true, ground_speed_kt: 480., altitude_ft: 36000., phase: PHASE_CRUISE, ..Flight::default() };
        assert_eq!(run(&mut a, cruise, 10), [6]);
        a.done = 11;
        a.was_running = false;
        let pushback = Flight { powered: true, on_ground: true, engines_running: true, ground_speed_kt: 4., ..Flight::default() };
        assert_eq!(run(&mut a, pushback, 10), [3], "the next flight's engine start begins a new sequence");
    }

    #[test]
    fn announcements_switched_off_stop_and_stay_silent_until_the_next_one() {
        let mut a = Announcements::default();
        a.reset();
        let cruise = Flight { powered: true, engines_running: true, altitude_ft: 36000., phase: PHASE_CRUISE, ..Flight::default() };
        assert_eq!(a.step(&cruise, true, 1.), 6);
        assert_eq!(a.step(&cruise, false, 1.), 0);
        assert_eq!(a.step(&cruise, true, 1.), 0, "switching back on does not replay the cruise one");
        assert_eq!(run(&mut a, Flight { phase: PHASE_DESCENT, ..cruise }, 100), [0, 7]);
    }

    #[test]
    fn the_pack_replaces_what_it_has_a_recording_for_and_nothing_else() {
        for e in ["Engine_1_Front", "Engine_4_Rear", "apurun", "380gmain", "cockpitwind380", "detent", "parkbrk", "CRC_380", "mastercaution", "cavcharge", "new_retard", "50_380", "flaps380"] {
            assert!(replaces(e), "{e}");
        }
        for e in ["V1", "3click", "aural_100above", "new_2000", "5_380", "pb1on", "apuflapopen", "TR380"] {
            assert!(!replaces(e), "{e}");
        }
    }

    fn quiet() -> AuralInputs {
        AuralInputs { crc: false, single_chime: false, cavalry: false, cchord: false, cchord_short: false, stall: false, on_ground: false, cabin: (0., 0.), flaps: (0., 0.) }
    }

    #[test]
    fn a_callout_plays_its_own_recording_with_the_radio_altitude_inside_its_band() {
        let mut a = Aurals::default();
        a.request("50_380");
        assert_eq!(a.callouts[CALLOUT_HEIGHTS.iter().position(|h| *h == 50).unwrap()], 1.);
        assert_eq!(a.callout_ra(48.7), 50.);
        a.step(&quiet(), 0.6);
        assert_eq!(a.callout_ra(41.2), 41.2, "back to the real height after the callout");
        a.request("new_2_500");
        a.request("new_2000");
        assert_eq!(a.callouts[0], 1., "2500 twice-named, 2000 has no recording");
    }

    #[test]
    fn repeated_aurals_hold_between_flybywires_requests_and_stop_after_the_last() {
        let mut a = Aurals::default();
        a.request("new_retard");
        for _ in 0..10 {
            a.step(&quiet(), 0.1);
        }
        assert!(a.retard_s > 0.);
        a.request("new_retard");
        for _ in 0..16 {
            a.step(&quiet(), 0.1);
        }
        assert_eq!(a.retard_s, 0.);
    }

    #[test]
    fn stall_repeats_in_flight_only_and_the_c_chord_repeats_while_it_lasts() {
        let mut a = Aurals::default();
        let stall = AuralInputs { stall: true, ..quiet() };
        for _ in 0..30 {
            a.step(&stall, 0.1);
        }
        assert_eq!(a.stall, 2., "at once and again after 2.6 s");
        let mut ground = Aurals::default();
        ground.step(&AuralInputs { stall: true, on_ground: true, ..quiet() }, 0.1);
        assert_eq!(ground.stall, 0., "the same variable is max braking on the ground");
        let mut c = Aurals::default();
        for _ in 0..20 {
            c.step(&AuralInputs { cchord: true, ..quiet() }, 0.1);
        }
        assert_eq!(c.cchord, 2.);
    }

    #[test]
    fn flaps_play_while_moving_and_a_stop_after() {
        let mut a = Aurals::default();
        for k in 0..5 {
            a.step(&AuralInputs { flaps: (k as f64, 0.), ..quiet() }, 0.1);
        }
        assert!(a.flaps_moving);
        for _ in 0..5 {
            a.step(&AuralInputs { flaps: (4., 0.), ..quiet() }, 0.1);
        }
        assert!(!a.flaps_moving);
        assert_eq!(a.flaps_stop, 1.);
    }

    #[test]
    fn cockpit_clicks_map_onto_the_packs_switch_sounds() {
        assert_eq!(click_sound("pb1on"), Some("p1"));
        assert_eq!(click_sound("lswitch"), Some("s2"));
        assert_eq!(click_sound("papertear"), None);
        let mut a = Aurals::default();
        a.request("turnknob");
        assert_eq!(a.clicks[CLICK_SOUNDS.iter().position(|c| *c == "sel1").unwrap()], 1.);
    }
}
