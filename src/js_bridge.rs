//! The JavaScript engine, connected to the plugin.
//!
//! FlyByWire's instruments and hosts run as MSFS runs them (`js::msfs`): each
//! panel.cfg `[VCockpitNN]` section with an HTML gauge is a view, loading
//! FlyByWire's built bundles from the aircraft's folder:
//!
//! ```text
//! <aircraft>/
//!   html_ui/        the package's html_ui, as FlyByWire's build makes it
//!                   (Pages/VCockpit/Instruments/A380X/PFD/pfd.html, ...)
//!   panel/panel.cfg the package's panel.cfg and panel.xml
//!   plugins/fbw_a380_systems/64/win.xpl
//! ```
//!
//! (docs/js-build.md has the build; tools/install.sh the copy.) Without them, a plain
//! entry module `js/main.ts` (or .tsx, .js, .mjs) runs instead, with an
//! optional `js/importmap.json` (`{"imports": {"name": "path"}}`).
//!
//! Variables reach the plugin's as MSFS instruments name them: `L:NAME` is the
//! aircraft variable named exactly so, read and written as its number (a
//! `bool` unit reads 0 or 1); `A:NAME:index` or a bare name is a simulator
//! variable, converted from the unit FlyByWire's systems keep it in
//! (`provides_aircraft_variable` in a380_systems_wasm) into the unit asked
//! for; `E:` names are the simulator's clock; `GAME:` names are MSFS game
//! variables. `K:` writes are key events for the plugin: `take_events` has
//! them as `("K:NAME", value)` (or `K:n:NAME`), and msfs-sdk's
//! `triggerKey(key, bypass, v0, v1, v2)` goes to `key_events::push` with its
//! three values. `H:` events reach every instrument, and `take_events` too.
//!
//! H: events for the instruments from the cockpit are X-Plane commands
//! `fbw/hevent/<NAME>` (tools/js-build/hevents.txt lists the names).

use std::cell::RefCell;
use std::collections::{BTreeMap, HashMap, HashSet, VecDeque};
use std::ffi::c_void;
use std::path::PathBuf;
use std::time::{SystemTime, UNIX_EPOCH};

use systems::simulation::{SimulatorReaderWriter, VariableIdentifier};

use crate::js::msfs::{Cockpit, CockpitOptions, StoredDataBackend};
use crate::js_worker::{Frame, MakeCockpit, Worker};
use crate::js::units::{self, Kind};
use crate::js::{CallReply, Engine, EngineOptions, Host, ImportMap, LogLevel};
use crate::xp::{CommandRef, DataRef, Xplm};
use crate::{Vars, NAMED, SIMULATOR};

/// How many script events are kept for the plugin to read.
const EVENT_LIMIT: usize = 1024;

/// Where MSFS's stored data is kept between flights, beside the flyPad
/// settings (efb.rs), relative to X-Plane's folder as X-Plane runs.
const DATASTORE: &str = "Output/preferences/fbw_a380x_datastore.json";

/// The H: event names the converted cockpit fires, one per line.
const HEVENTS: &str = include_str!("../tools/js-build/hevents.txt");

/// FlyByWire's systems' simulator variables and the unit each is kept in
/// here, from their MSFS glue (the units the plugin's inputs convert into).
const SYSTEMS_WASM: &str = include_str!("../../fbw-aircraft/fbw-a380x/src/wasm/systems/a380_systems_wasm/src/lib.rs");

fn stored_units() -> &'static HashMap<String, String> {
    static UNITS: std::sync::OnceLock<HashMap<String, String>> = std::sync::OnceLock::new();
    UNITS.get_or_init(|| {
        let mut out = HashMap::new();
        for part in SYSTEMS_WASM.split("provides_aircraft_variable(").skip(1) {
            let mut strings = part.split('"');
            let (Some(_), Some(name), Some(_), Some(unit)) = (strings.next(), strings.next(), strings.next(), strings.next()) else {
                continue;
            };
            out.entry(name.to_string()).or_insert_with(|| unit.to_string());
        }
        out
    })
}

/// A variable name without its `:index`.
fn base_name(name: &str) -> &str {
    match name.rsplit_once(':') {
        Some((base, index)) if index.chars().all(|c| c.is_ascii_digit()) => base,
        _ => name,
    }
}

/// What a name and unit were resolved to.
#[derive(Clone, Copy)]
enum Target {
    Var(VariableIdentifier),
    Env(Env),
    /// `GAME:CAMERA POS IN PLANE:X/Y/Z` (PilotSeat.ts's `CAMERA_POS_IN_PLANE`,
    /// used to detect which seat/whether the view is in the flight deck).
    CameraAxis(CameraAxis),
    Game,
}

#[derive(Clone, Copy)]
enum Env {
    SimulationTime,
    /// `E:ABSOLUTE TIME`: real wall-clock seconds since 0000-01-01 (.NET
    /// `DateTime` ticks), the clock msfs-sdk's `ClockPublisher` reads for
    /// `simTimeHiFreq`/`realTimeHiFreq` (msfssdk.js `TimeUtils.
    /// simAbsoluteTimeToJSTimestamp`). It used to fall through to
    /// `Env::Unknown` and read as NaN, which poisoned every `dt` computed
    /// from `simTimeHiFreq` (`now - lastUpdateTime` is NaN forever once
    /// `now` is NaN), so throttled `update(dt)` cycles across every host
    /// (`FwsCore.update`'s `fwsUpdateThrottler.canUpdate(NaN)` always
    /// returns -1) never ran again after their first tick.
    AbsoluteTime,
    /// `E:SIMULATION RATE`: X-Plane's time acceleration.
    SimulationRate,
    /// `E:ZULU DAY OF MONTH` / `MONTH OF YEAR` / `YEAR`: X-Plane's date
    /// (its clock has a day and month but no year, so the year is today's).
    ZuluDay,
    ZuluMonth,
    ZuluYear,
    ZuluTime,
    LocalTime,
    /// MSFS's `E:TIME OF DAY` enum (extra_backend_fbw.rs's `time_of_day`,
    /// resolved from the sun each tick on the main thread).
    TimeOfDay,
    /// `E:ZULU SUNRISE TIME` / `E:ZULU SUNSET TIME`: msfs-sdk's
    /// `ClockPublisher` (bundled into every instrument, e.g. `EWD/bundle.js`,
    /// `ExtrasHost/index.js`: `zulu_sunrise`/`zulu_sunset`) reads these as
    /// seconds since UTC midnight at the aircraft's position, today. Computed
    /// from the NOAA solar-position equations (`solar_event_seconds`) at
    /// X-Plane's own latitude/longitude/date; not a simulator variable MSFS
    /// or X-Plane exposes directly.
    ZuluSunrise,
    ZuluSunset,
    /// `E:IS AIRCRAFT` (`GPUManagement.ts`'s `isInAircraft`, gating ground
    /// power/GPU logic): whether the current view belongs to *this*
    /// aircraft, as opposed to MSFS's drone camera/another AI aircraft/a
    /// multiplayer peer. X-Plane has no such "whose aircraft is the camera
    /// on" concept — its camera is always this plugin's one aircraft, views
    /// and all — so this is always true here, not a stand-in default.
    IsAircraft,
    Unknown,
}

/// The axis of `GAME:CAMERA POS IN PLANE`.
#[derive(Clone, Copy)]
enum CameraAxis {
    X,
    Y,
    Z,
}

/// From the stored number to the unit asked for.
#[derive(Clone, Copy)]
enum Conversion {
    Raw,
    Bool,
    Linear(f64, f64),
    Encoded(units::Unit, units::Unit),
}

impl Conversion {
    fn between(stored: Option<&str>, wanted: &str) -> Self {
        match units::kind(wanted) {
            Some(Kind::Bool) => Conversion::Bool,
            Some(Kind::Measure(to)) => match stored.and_then(units::kind) {
                Some(Kind::Measure(from)) if from.dimension == to.dimension => match units::linear(&from, &to) {
                    Some((scale, offset)) => Conversion::Linear(scale, offset),
                    None => Conversion::Encoded(from, to),
                },
                _ => Conversion::Raw,
            },
            _ => Conversion::Raw,
        }
    }

    fn read(self, value: f64) -> f64 {
        match self {
            Conversion::Raw => value,
            Conversion::Bool => (value != 0.) as i32 as f64,
            Conversion::Linear(scale, offset) => value * scale + offset,
            Conversion::Encoded(from, to) => units::convert(value, &from, &to).unwrap_or(value),
        }
    }

    fn write(self, value: f64) -> f64 {
        match self {
            Conversion::Raw => value,
            Conversion::Bool => (value != 0.) as i32 as f64,
            Conversion::Linear(scale, offset) => (value - offset) / scale,
            Conversion::Encoded(from, to) => units::convert(value, &to, &from).unwrap_or(value),
        }
    }
}

/// An opaque handle to what a name and unit were resolved to (a `VarsHost`
/// keeps the ability to read/write it; xphfbw_host.rs stores these by slot,
/// same as js_worker.rs's `JsHost::slots`, and never looks inside one).
#[derive(Clone, Copy)]
pub(crate) struct Resolved {
    target: Target,
    conversion: Conversion,
}

/// X-Plane's clock, for `E:` variables (DataRefs.txt: seconds), and the
/// pilot's head/aircraft position `E:ZULU SUNRISE/SUNSET TIME` and
/// `GAME:CAMERA POS IN PLANE` are computed from.
pub(crate) struct EnvRefs {
    zulu: Option<DataRef>,
    local: Option<DataRef>,
    rate: Option<DataRef>,
    day: Option<DataRef>,
    month: Option<DataRef>,
    /// `sim/flightmodel/position/latitude`/`longitude` (degrees): the
    /// aircraft's position, for the solar sunrise/sunset computation.
    latitude: Option<DataRef>,
    longitude: Option<DataRef>,
    /// `sim/graphics/view/pilots_head_x/y/z` (DataRefs.txt: meters, "relative
    /// to CG" — actually the aircraft's own local-coordinate origin, which
    /// msfs2xp-aircraft's converter places at the MSFS package's model
    /// origin, `acf.rs`: ".acf lengths are feet with x right, y up and z aft
    /// of the reference point, which here is the MSFS model origin, so the
    /// objects sit at 0,0,0"). `GAME:CAMERA POS IN PLANE` is in MSFS's own
    /// body axes (x lateral, y vertical, z longitudinal-forward, all in
    /// meters here since that is the unit every JS caller asks for through
    /// `gamevar.getValue_XYZ`); X-Plane's z is aft-positive (negate) and its
    /// x is right-positive, while FlyByWire's own `PilotSeat.ts` treats
    /// negative `GAME:CAMERA POS IN PLANE:X` as the right seat, so that axis
    /// is negated too (see `read`'s `CameraAxis` arm).
    head_x: Option<DataRef>,
    head_y: Option<DataRef>,
    head_z: Option<DataRef>,
}

impl EnvRefs {
    /// Looked up once (find(), and xphfbw_host.rs's `XphfbwHost::start`):
    /// both the QuickJS path and the bridge path resolve `E:`/camera-axis
    /// names against the same datarefs.
    pub(crate) fn new(xplm: &Xplm) -> Self {
        Self {
            zulu: xplm.find("sim/time/zulu_time_sec"),
            local: xplm.find("sim/time/local_time_sec"),
            rate: xplm.find("sim/time/sim_speed"),
            day: xplm.find("sim/cockpit2/clock_timer/current_day"),
            month: xplm.find("sim/cockpit2/clock_timer/current_month"),
            latitude: xplm.find("sim/flightmodel/position/latitude"),
            longitude: xplm.find("sim/flightmodel/position/longitude"),
            head_x: xplm.find("sim/graphics/view/pilots_head_x"),
            head_y: xplm.find("sim/graphics/view/pilots_head_y"),
            head_z: xplm.find("sim/graphics/view/pilots_head_z"),
        }
    }
}

/// The state the host keeps between ticks. `pub(crate)`: xphfbw_host.rs
/// keeps its own (the bridge path resolves variables independently of the
/// QuickJS path's), built with `HostState::default()`.
#[derive(Default)]
pub(crate) struct HostState {
    ids: HashMap<String, VariableIdentifier>,
    reg: HashMap<usize, Resolved>,
    events: VecDeque<(String, f64)>,
    strings: HashMap<String, String>,
    logged: HashSet<String>,
}

/// Resolves a name/unit to a variable (or `E:`/`GAME:` source) and applies
/// its conversion, reading and writing FlyByWire's variables. The bridge
/// path (xphfbw_host.rs) uses the exact same type (via `VarsHost::new`) so
/// XPHFBW's views are resolved and read/written with the same logic the
/// QuickJS path uses, per docs/briefs/xphfbw-js-bridge.md: "The plugin
/// resolves new slots each frame with the same logic the QuickJS host uses
/// (js_bridge.rs VarsHost::resolve/read)".
pub(crate) struct VarsHost<'a> {
    vars: &'a mut Vars,
    state: &'a mut HostState,
    env: &'a EnvRefs,
    time: f64,
}

impl<'a> VarsHost<'a> {
    pub(crate) fn new(vars: &'a mut Vars, state: &'a mut HostState, env: &'a EnvRefs, time: f64) -> Self {
        Self { vars, state, env, time }
    }
}

impl VarsHost<'_> {
    fn id(&mut self, name: &str) -> VariableIdentifier {
        if let Some(id) = self.state.ids.get(name) {
            return *id;
        }
        let id = match name.strip_prefix("L:") {
            Some(named) => self.vars.add(named.trim().to_string(), NAMED),
            None => self.vars.add(name.strip_prefix("A:").unwrap_or(name).trim().to_string(), SIMULATOR),
        };
        self.state.ids.insert(name.to_string(), id);
        id
    }

    pub(crate) fn resolve(&mut self, name: &str, unit: &str) -> Resolved {
        if let Some(env) = name.strip_prefix("E:") {
            let env = match env.trim().to_ascii_uppercase().as_str() {
                "SIMULATION TIME" => Env::SimulationTime,
                "ABSOLUTE TIME" => Env::AbsoluteTime,
                "SIMULATION RATE" => Env::SimulationRate,
                "ZULU DAY OF MONTH" => Env::ZuluDay,
                "ZULU MONTH OF YEAR" => Env::ZuluMonth,
                "ZULU YEAR" => Env::ZuluYear,
                "ZULU TIME" => Env::ZuluTime,
                "LOCAL TIME" => Env::LocalTime,
                "TIME OF DAY" => Env::TimeOfDay,
                "ZULU SUNRISE TIME" => Env::ZuluSunrise,
                "ZULU SUNSET TIME" => Env::ZuluSunset,
                "IS AIRCRAFT" => Env::IsAircraft,
                _ => Env::Unknown,
            };
            // TIME OF DAY is MSFS's dawn/day/dusk/night enum (0-3), IS
            // AIRCRAFT a bool, neither seconds.
            let stored = if matches!(
                env,
                Env::TimeOfDay | Env::SimulationRate | Env::ZuluDay | Env::ZuluMonth | Env::ZuluYear | Env::IsAircraft
            ) {
                None
            } else {
                Some("seconds")
            };
            return Resolved { target: Target::Env(env), conversion: Conversion::between(stored, unit) };
        }
        if let Some(camera) = name.strip_prefix("GAME:CAMERA POS IN PLANE:") {
            let axis = match camera.trim().to_ascii_uppercase().as_str() {
                "X" => Some(CameraAxis::X),
                "Y" => Some(CameraAxis::Y),
                "Z" => Some(CameraAxis::Z),
                _ => None,
            };
            if let Some(axis) = axis {
                return Resolved { target: Target::CameraAxis(axis), conversion: Conversion::Raw };
            }
        }
        if name.starts_with("GAME:") {
            return Resolved { target: Target::Game, conversion: Conversion::Raw };
        }
        let id = self.id(name);
        let conversion = if name.starts_with("L:") {
            // Aircraft variables are numbers, read as they were written.
            if matches!(units::kind(unit), Some(Kind::Bool)) {
                Conversion::Bool
            } else {
                Conversion::Raw
            }
        } else {
            let plain = name.strip_prefix("A:").unwrap_or(name).trim();
            Conversion::between(stored_units().get(base_name(plain)).map(String::as_str), unit)
        };
        Resolved { target: Target::Var(id), conversion }
    }

    pub(crate) fn read(&mut self, name: &str, r: Resolved) -> f64 {
        let raw = match r.target {
            Target::Var(id) => {
                if name.starts_with("L:") && self.vars.is_unwritten_named(&id) {
                    self.log_once(&format!("first read of unwritten L: var: {name}"));
                }
                self.vars.read(&id)
            }
            Target::Env(Env::SimulationTime) => self.time,
            // .NET DateTime ticks (100ns units) expressed in seconds since
            // 0000-01-01; 62_135_596_800 is the Unix epoch in that scale
            // (msfs-sdk's own `simAbsoluteTimeToJSTimestamp` constant).
            // Simulator time, as in MSFS: it stops when paused and runs faster
            // under time acceleration. Anchored to the real date once, then
            // advanced by the simulation's own clock.
            Target::Env(Env::AbsoluteTime) => {
                static ANCHOR: std::sync::OnceLock<f64> = std::sync::OnceLock::new();
                let now = self.time;
                let anchor = *ANCHOR.get_or_init(|| {
                    SystemTime::now().duration_since(UNIX_EPOCH).map(|d| d.as_secs_f64()).unwrap_or(0.) + 62_135_596_800.0 - now
                });
                anchor + now
            }
            Target::Env(Env::ZuluDay) => self.env.day.map_or(f64::NAN, |d| self.vars.xplm.get_i(d) as f64),
            Target::Env(Env::ZuluMonth) => self.env.month.map_or(f64::NAN, |d| self.vars.xplm.get_i(d) as f64),
            Target::Env(Env::ZuluYear) => {
                // Days since 1970 to a civil year (Howard Hinnant's days_from_civil, inverted).
                let days = (SystemTime::now().duration_since(UNIX_EPOCH).map(|d| d.as_secs()).unwrap_or(0) / 86_400) as i64 + 719_468;
                let era = days.div_euclid(146_097);
                let doe = days - era * 146_097;
                let yoe = (doe - doe / 1460 + doe / 36_524 - doe / 146_096) / 365;
                let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
                let mp = (5 * doy + 2) / 153;
                let month = if mp < 10 { mp + 3 } else { mp - 9 };
                (yoe + era * 400 + i64::from(month <= 2)) as f64
            }
            Target::Env(Env::SimulationRate) => self.env.rate.map_or(1., |d| self.vars.xplm.get_i(d) as f64).max(0.),
            Target::Env(Env::IsAircraft) => 1.,
            Target::Env(Env::ZuluTime) => self.env.zulu.map_or(f64::NAN, |d| self.vars.xplm.get_f(d) as f64),
            Target::Env(Env::LocalTime) => self.env.local.map_or(f64::NAN, |d| self.vars.xplm.get_f(d) as f64),
            // Resolved on this (the main) thread from an atomic the flight
            // loop updates each frame; never blocks.
            Target::Env(Env::TimeOfDay) => crate::extra_backend_fbw::time_of_day(),
            Target::Env(sunrise @ (Env::ZuluSunrise | Env::ZuluSunset)) => {
                match (self.env.latitude, self.env.longitude, self.env.day, self.env.month) {
                    (Some(lat), Some(lon), Some(day), Some(month)) => solar_event_seconds(
                        self.vars.xplm.get_d(lat),
                        self.vars.xplm.get_d(lon),
                        self.vars.xplm.get_i(day),
                        self.vars.xplm.get_i(month),
                        matches!(sunrise, Env::ZuluSunrise),
                    ),
                    _ => f64::NAN,
                }
            }
            Target::CameraAxis(axis) => match (self.env.head_x, self.env.head_y, self.env.head_z) {
                (Some(x), Some(y), Some(z)) => match axis {
                    // X-Plane's x is right-positive, but FlyByWire's
                    // `PilotSeat.ts` treats negative `X` as the right seat,
                    // so `X` is negated. `Z` is X-Plane's aft-positive vs.
                    // MSFS's forward-positive, so it is negated too. `Y`
                    // (vertical, up-positive in both) is not.
                    CameraAxis::X => -(self.vars.xplm.get_f(x) as f64),
                    CameraAxis::Y => self.vars.xplm.get_f(y) as f64,
                    CameraAxis::Z => -(self.vars.xplm.get_f(z) as f64),
                },
                _ => f64::NAN,
            },
            Target::Env(Env::Unknown) | Target::Game => {
                self.log_once(&format!("{name} has no source here and reads as NaN"));
                f64::NAN
            }
        };
        r.conversion.read(raw)
    }

    pub(crate) fn write(&mut self, name: &str, r: Resolved, value: f64) {
        match r.target {
            Target::Var(id) => self.vars.write(&id, r.conversion.write(value)),
            _ => self.log_once(&format!("{name} cannot be written here")),
        }
    }

    fn log_once(&mut self, message: &str) {
        if self.state.logged.insert(message.to_string()) {
            crate::log(&format!("js: {message}"));
        }
    }

    fn queue(&mut self, name: &str, value: f64) {
        if self.state.events.len() == EVENT_LIMIT {
            self.state.events.pop_front();
        }
        self.state.events.push_back((name.to_string(), value));
    }
}

/// Seconds since UTC midnight of sunrise (or sunset) at `lat`/`lon` degrees
/// on `day`/`month` (X-Plane's own clock has no year; the sun's position
/// depends only on day-of-year, so this does not need one). NOAA's solar
/// position equations (the Spencer 1971 Fourier approximation NOAA's solar
/// calculator uses), evaluated at solar noon: fractional year `gamma`, the
/// equation of time and the solar declination from it, then the hour angle
/// of the 90.833° (sunrise/sunset, refraction-corrected) zenith. At latitudes
/// where the sun does not rise or set that day the `acos` argument is
/// clamped, giving a continuous (not NaN) edge value, same as every other
/// solar calculator does for polar day/night.
fn solar_event_seconds(lat_deg: f64, lon_deg: f64, day: i32, month: i32, sunrise: bool) -> f64 {
    const CUMULATIVE_DAYS: [i32; 12] = [0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334];
    let month0 = (month.clamp(1, 12) - 1) as usize;
    let day_of_year = (CUMULATIVE_DAYS[month0] + day.clamp(1, 31)).clamp(1, 366) as f64;

    let gamma = 2. * std::f64::consts::PI / 365. * (day_of_year - 1.);
    let eqtime_min = 229.18
        * (0.000_075 + 0.001_868 * gamma.cos()
            - 0.032_077 * gamma.sin()
            - 0.014_615 * (2. * gamma).cos()
            - 0.040_849 * (2. * gamma).sin());
    let decl = 0.006_918 - 0.399_912 * gamma.cos() + 0.070_257 * gamma.sin() - 0.006_758 * (2. * gamma).cos()
        + 0.000_907 * (2. * gamma).sin()
        - 0.002_697 * (3. * gamma).cos()
        + 0.001_48 * (3. * gamma).sin();

    let lat = lat_deg.to_radians();
    let zenith = 90.833_f64.to_radians();
    let cos_ha = zenith.cos() / (lat.cos() * decl.cos()) - lat.tan() * decl.tan();
    let ha_deg = cos_ha.clamp(-1., 1.).acos().to_degrees();

    let solar_noon_min = 720. - 4. * lon_deg - eqtime_min;
    let event_min = if sunrise { solar_noon_min - 4. * ha_deg } else { solar_noon_min + 4. * ha_deg };
    (event_min * 60.).rem_euclid(86_400.)
}

impl Host for VarsHost<'_> {
    fn get_var(&mut self, name: &str, unit: &str) -> f64 {
        let r = self.resolve(name, unit);
        self.read(name, r)
    }

    fn set_var(&mut self, name: &str, unit: &str, value: f64) {
        if name.starts_with("K:") || name.starts_with("H:") {
            self.queue(name, value);
            return;
        }
        let r = self.resolve(name, unit);
        self.write(name, r, value);
    }

    fn get_var_reg(&mut self, id: usize, name: &str, unit: &str) -> f64 {
        let r = match self.state.reg.get(&id) {
            Some(r) => *r,
            None => {
                let r = self.resolve(name, unit);
                self.state.reg.insert(id, r);
                r
            }
        };
        self.read(name, r)
    }

    fn set_var_reg(&mut self, id: usize, name: &str, unit: &str, value: f64) {
        if name.starts_with("K:") || name.starts_with("H:") {
            self.queue(name, value);
            return;
        }
        let r = match self.state.reg.get(&id) {
            Some(r) => *r,
            None => {
                let r = self.resolve(name, unit);
                self.state.reg.insert(id, r);
                r
            }
        };
        self.write(name, r, value);
    }

    /// `triggerKey`'s events, with all their values, go to key_events.rs;
    /// the rest are queued for `take_events`.
    fn send_event(&mut self, name: &str, values: &[f64]) {
        if name.starts_with("K:") && values.len() > 1 {
            crate::key_events::push(name, values);
        } else {
            self.queue(name, values.first().copied().unwrap_or(0.));
        }
    }

    fn get_string(&mut self, name: &str) -> String {
        if let Some(game) = name.strip_prefix("GAME:") {
            if let Some(value) = provider_game_string(game) {
                return value;
            }
            self.log_once(&format!("{name} has no source here and reads as the empty string"));
        }
        self.state.strings.get(name).cloned().unwrap_or_default()
    }

    fn set_string(&mut self, name: &str, value: &str) {
        self.state.strings.insert(name.to_string(), value.to_string());
    }

    /// `Facilities.getMagVar`: X-Plane's own magnetic variation model, the
    /// same source navdata's facility magvar uses (js_bridge.rs's
    /// `start_navdata`), queried directly for an arbitrary point rather than
    /// only the aircraft's own position.
    fn get_magvar(&mut self, lat: f64, lon: f64) -> f64 {
        crate::xp::magnetic_variation(lat, lon).map_or(0., f64::from)
    }

    fn log(&mut self, level: LogLevel, message: &str) {
        // An error thrown every frame is logged once.
        if level == LogLevel::Error && !self.state.logged.insert(message.to_string()) {
            return;
        }
        let tag = match level {
            LogLevel::Info => "",
            LogLevel::Warn => "warning: ",
            LogLevel::Error => "error: ",
        };
        crate::log(&format!("js: {tag}{message}"));
    }

    /// The calls answered anywhere ([`direct_call`]), then the providers'.
    fn call(&mut self, name: &str, args_json: &str) -> CallReply {
        if let Some(reply) = direct_call(name, args_json) {
            return reply;
        }
        provider_call(name, args_json).unwrap_or_else(|| CallReply::Rejected(format!("Coherent.call('{name}'): nothing here answers this call")))
    }
}

/// Calls answered on any thread: the runtime's `fetch` for SimBridge's
/// address (map data answers its terrain API), and map data's calls.
pub(crate) fn direct_call(name: &str, args_json: &str) -> Option<CallReply> {
    if name == "fetch" {
        return Some(simbridge_fetch(args_json));
    }
    // [sound] LegacySoundManager/FwsSoundManager aurals: Coherent.call('PLAY_INSTRUMENT_SOUND', wwiseEventName).
    if name == "PLAY_INSTRUMENT_SOUND" {
        if let Some(sound) = serde_json::from_str::<Vec<serde_json::Value>>(args_json).ok().and_then(|a| a.first().and_then(|v| v.as_str()).map(str::to_owned)) {
            crate::sound::play_instrument_sound(&sound);
        }
        return Some(CallReply::Resolved("null".to_string()));
    }
    // Authentication.tsx's Navigraph login link: opens the URL in the user's
    // real default browser, as MSFS's own OPEN_WEB_BROWSER does.
    if name == "OPEN_WEB_BROWSER" {
        if let Some(url) = serde_json::from_str::<Vec<serde_json::Value>>(args_json).ok().and_then(|a| a.first().and_then(|v| v.as_str()).map(str::to_owned)) {
            let _ = std::process::Command::new("cmd").args(["/C", "start", "", &url]).spawn();
        }
        return Some(CallReply::Resolved("null".to_string()));
    }
    // TodPauseManager.ts's top-of-descent popup: pauses/unpauses the sim
    // through X-Plane's own pause commands (Commands.txt), the same effect
    // MSFS's toolbar "active pause" has.
    if name == "TOOLBAR_SET_ACTIVE_PAUSE" {
        let pause = serde_json::from_str::<Vec<serde_json::Value>>(args_json).ok().and_then(|a| a.first().and_then(|v| v.as_bool())).unwrap_or(false);
        crate::xp::command_once(if pause { "sim/operation/pause_on" } else { "sim/operation/pause_off" });
        return Some(CallReply::Resolved("null".to_string()));
    }
    // failures-orchestrator.ts's FBW_FAILURE_UPDATE: in real MSFS this tells
    // the C++ WASM's failure state over the comm bus. Here PRIM/SEC/FADEC run
    // as directly-ported computer objects (prim.rs) driven from the Rust
    // systems' own failure variables (failures.rs), not from this bus, so
    // there is nothing further to feed; only resolving the call (instead of
    // leaving it unanswered/rejected) matters, so the EFB's failure list
    // does not treat every toggle as a Coherent error.
    if name == "COMM_BUS_WASM_CALLBACK" {
        return Some(CallReply::Resolved("null".to_string()));
    }
    crate::mapdata::plugin::coherent_call(name, args_json).map(|reply| match reply {
        Ok(json) => CallReply::Resolved(json),
        Err(e) => CallReply::Rejected(e),
    })
}

/// `fetch(url, {method, body})` to SimBridge (`http://localhost:8380`, fbw-common
/// simbridge/common.ts): the terrain API map data implements, and `/health`
/// listing that service (SimBridge's health.controller.ts checks `api` and
/// `mcdu`; only the API is here, so only it is listed, as up). Anything else
/// is not found. The reply is `{status, body}`.
fn simbridge_fetch(args_json: &str) -> CallReply {
    let args: Vec<serde_json::Value> = serde_json::from_str(args_json).unwrap_or_default();
    let s = |i: usize| args.get(i).and_then(|v| v.as_str()).unwrap_or("").to_string();
    let (method, url, body) = (s(0), s(1), s(2));
    // [oans] Navigraph's AMDB API (amdb.ts, patched to use fetch(): see
    // src/oans/plugin.rs), answered from the user's local data instead.
    if let Some((status, text)) = crate::oans::plugin::oans_request(&method, &url, &body) {
        return CallReply::Resolved(serde_json::json!({ "status": status, "body": text }).to_string());
    }
    let path = url.splitn(4, '/').nth(3).map(|p| format!("/{p}")).unwrap_or_default();
    let (status, text) = if path.split('?').next() == Some("/health") {
        (200, r#"{"status":"ok","info":{"api":{"status":"up"}},"error":{},"details":{"api":{"status":"up"}}}"#.to_string())
    } else {
        crate::mapdata::plugin::simbridge_request(&method, &path, &body).unwrap_or((404, String::new()))
    };
    CallReply::Resolved(serde_json::json!({ "status": status, "body": text }).to_string())
}

#[link(name = "kernel32")]
extern "system" {
    fn GetModuleHandleExW(flags: u32, name: *const u16, module: *mut *mut c_void) -> i32;
    fn GetModuleFileNameW(module: *mut c_void, buffer: *mut u16, size: u32) -> u32;
}

/// This plugin's folder (the one holding `64/win.xpl`).
pub(crate) fn plugin_dir() -> Option<PathBuf> {
    const FROM_ADDRESS: u32 = 0x4;
    const UNCHANGED_REFCOUNT: u32 = 0x2;
    unsafe {
        let mut module: *mut c_void = std::ptr::null_mut();
        let address = plugin_dir as *const () as *const u16;
        if GetModuleHandleExW(FROM_ADDRESS | UNCHANGED_REFCOUNT, address, &mut module) == 0 {
            return None;
        }
        let mut buffer = vec![0u16; 1024];
        let len = GetModuleFileNameW(module, buffer.as_mut_ptr(), buffer.len() as u32) as usize;
        if len == 0 {
            return None;
        }
        let path = PathBuf::from(String::from_utf16_lossy(&buffer[..len]));
        Some(path.parent()?.parent()?.to_path_buf())
    }
}

enum Scripts {
    /// FlyByWire's instruments, as MSFS runs them, on their own thread.
    /// `worker` is `None` while suspended (xphfbw_host.rs's `displays_active`:
    /// XPHFBW's own browsers run the same views, so this plugin's copy stops
    /// rather than doing the same work and never drawing it, rule 7).
    /// `recipe` stays so `resume` can start a fresh worker.
    Cockpit { worker: Option<Worker>, recipe: CockpitRecipe },
    /// One entry module.
    Main { engine: Engine, main: PathBuf, loaded: bool, failed: bool },
}

/// What `find` used to build the cockpit's views, kept so `suspend`/`resume`
/// can make a fresh one without re-deriving the aircraft's paths.
struct CockpitRecipe {
    html_ui: PathBuf,
    panel_cfg: String,
    panel_xml: String,
}

impl CockpitRecipe {
    fn make(&self) -> MakeCockpit {
        let (html_ui, panel_cfg, panel_xml) = (self.html_ui.clone(), self.panel_cfg.clone(), self.panel_xml.clone());
        Box::new(move || {
            let mut options = CockpitOptions::new(html_ui, panel_cfg);
            options.panel_xml = panel_xml;
            options.datastore = Some(PathBuf::from(DATASTORE));
            options.settings = Some(Box::new(FlyPadSettings::default()));
            options.patches = native_ports();
            Cockpit::new(options, &register_display)
        })
    }
}

/// The engine and its scripts.
pub struct JsHost {
    scripts: Scripts,
    state: HostState,
    env: EnvRefs,
    time_ms: f64,
    commands: Vec<(CommandRef, usize)>,
    /// The instruments' variable slots, by slot: the name as they read it,
    /// and what it resolved to.
    slots: Vec<(String, Resolved)>,
    /// Values sent last frame, reused.
    values: Vec<f64>,
}

thread_local! {
    /// H: events X-Plane commands fired since the last tick, by index into
    /// the names.
    static HEVENT_QUEUE: RefCell<Vec<usize>> = const { RefCell::new(Vec::new()) };
}

fn hevent_names() -> &'static Vec<String> {
    static NAMES: std::sync::OnceLock<Vec<String>> = std::sync::OnceLock::new();
    NAMES.get_or_init(|| HEVENTS.lines().map(str::trim).filter(|l| !l.is_empty() && !l.starts_with('#')).map(String::from).collect())
}

unsafe extern "C" fn on_hevent(_command: CommandRef, phase: std::ffi::c_int, refcon: *mut c_void) -> std::ffi::c_int {
    // One event per press: the command's begin phase.
    if phase == 0 {
        HEVENT_QUEUE.with(|q| q.borrow_mut().push(refcon as usize));
    }
    1
}

/// H: events the cockpit's `fbw/hevent/` X-Plane commands fired since the
/// last call, by name. Taken once a frame by lib.rs's tick and handed to
/// both `JsHost::update` and xphfbw_host.rs's `XphfbwHost::post_tick`, so
/// XPHFBW's views see the same events the plugin's own QuickJS cockpit does
/// without either draining the other's share of the queue.
pub(crate) fn take_hevents() -> Vec<String> {
    HEVENT_QUEUE.with(|q| std::mem::take(&mut *q.borrow_mut())).into_iter().filter_map(|i| hevent_names().get(i).cloned()).collect()
}

impl JsHost {
    /// The scripts, if the plugin has FlyByWire's instruments or an entry
    /// module.
    pub fn find(xplm: &'static Xplm) -> Option<Self> {
        let dir = plugin_dir()?;
        // FlyByWire's built html_ui and the package's panel files sit in the
        // aircraft folder, two above the plugin's, where the renderer finds
        // the fonts and images (tools/install.sh).
        let aircraft = dir.parent().and_then(|p| p.parent()).map(PathBuf::from).unwrap_or_else(|| dir.clone());
        // [navdata] The facility database, indexed in the background.
        start_navdata();
        let env = EnvRefs::new(xplm);
        let scripts = match (std::fs::read_to_string(aircraft.join("panel").join("panel.cfg")), aircraft.join("html_ui")) {
            (Ok(panel_cfg), html_ui) if html_ui.is_dir() => {
                let panel_xml = std::fs::read_to_string(aircraft.join("panel").join("panel.xml")).unwrap_or_default();
                let recipe = CockpitRecipe { html_ui, panel_cfg, panel_xml };
                // Made on the instruments' thread, where the engines live.
                match Worker::start(recipe.make()) {
                    Ok(worker) => Scripts::Cockpit { worker: Some(worker), recipe },
                    Err(e) => {
                        crate::log(&format!("js: {e}"));
                        return None;
                    }
                }
            }
            _ => {
                let js = dir.join("js");
                let main = ["main.ts", "main.tsx", "main.js", "main.mjs"].iter().map(|n| js.join(n)).find(|p| p.is_file())?;
                let import_map = match std::fs::read_to_string(js.join("importmap.json")) {
                    Ok(text) => ImportMap::from_json(&text, &js).unwrap_or_else(|e| {
                        crate::log(&format!("js: importmap.json: {e}"));
                        ImportMap::new()
                    }),
                    Err(_) => ImportMap::new(),
                };
                let engine = match Engine::new(EngineOptions { root: js.clone(), import_map, ..Default::default() }) {
                    Ok(engine) => engine,
                    Err(e) => {
                        crate::log(&format!("js: engine could not start: {e}"));
                        return None;
                    }
                };
                // [display] The screens' host functions, and their mouse input.
                if let Err(e) = register_display(&engine).and_then(|_| deliver_screen_events_on_tick(&engine)) {
                    crate::log(&format!("js: the display host functions did not register: {e}"));
                }
                crate::log(&format!("js: engine ready, entry {}", main.display()));
                Scripts::Main { engine, main, loaded: false, failed: false }
            }
        };
        let mut commands = Vec::new();
        if matches!(scripts, Scripts::Cockpit { .. }) {
            for (i, name) in hevent_names().iter().enumerate() {
                if let Some(command) = xplm.create_command(&format!("fbw/hevent/{name}"), &format!("FlyByWire H:{name}")) {
                    xplm.register_command_handler(command, on_hevent, i as *mut c_void);
                    commands.push((command, i));
                }
            }
        }
        Some(Self { scripts, state: HostState::default(), env, time_ms: 0., commands, slots: Vec::new(), values: Vec::new() })
    }

    /// Advance the scripts' clock. FlyByWire's instruments run on their
    /// thread: this frame takes what they sent (new variables to resolve,
    /// writes, events, calls, log lines) and sends them the variables' values
    /// and this frame's events. `h_events`/`provider_events` are this
    /// frame's, already taken once by the caller ([`take_hevents`],
    /// [`take_provider_events`]) so xphfbw_host.rs's bridge path can see the
    /// same ones without a second, competing drain.
    pub fn update(&mut self, vars: &mut Vars, delta: f64, simulation_time: f64, h_events: &[String], provider_events: &[(String, String)]) {
        self.time_ms += delta * 1000.;
        let mut host = VarsHost::new(vars, &mut self.state, &self.env, simulation_time);
        match &mut self.scripts {
            // Suspended (XPHFBW's displays are active): nothing to trade.
            Scripts::Cockpit { worker: None, .. } => {}
            Scripts::Cockpit { worker: Some(worker), .. } => {
                let (slots, values, time_ms) = (&mut self.slots, &mut self.values, self.time_ms);
                worker.exchange(|out| {
                    for line in &out.logs {
                        crate::log(line);
                    }
                    if let Some(lines) = &out.loaded {
                        for line in lines {
                            crate::log(line);
                        }
                    }
                    // New slots, resolved as the host resolves any name.
                    debug_assert_eq!(out.first_slot, slots.len());
                    for (name, unit) in out.slots {
                        let r = host.resolve(&name, &unit);
                        slots.push((name, r));
                    }
                    for (slot, value) in out.writes {
                        if let Some((name, r)) = slots.get(slot) {
                            let (name, r) = (name.clone(), *r);
                            host.write(&name, r, value);
                        }
                    }
                    for (name, value) in out.events {
                        host.queue(&name, value);
                    }
                    let replies = out
                        .calls
                        .into_iter()
                        .filter_map(|(name, args)| {
                            let reply = provider_call(&name, &args)
                                .unwrap_or_else(|| CallReply::Rejected(format!("Coherent.call('{name}'): nothing here answers this call")));
                            // Still pending here too: asked again next frame.
                            (!matches!(reply, CallReply::Pending(_))).then_some(((name, args), reply))
                        })
                        .collect();
                    let game_strings = out
                        .game_strings
                        .into_iter()
                        .map(|name| {
                            let value = provider_game_string(&name).unwrap_or_else(|| {
                                host.log_once(&format!("GAME:{name} has no source here and reads as the empty string"));
                                String::new()
                            });
                            (name, value)
                        })
                        .collect();
                    values.clear();
                    for i in 0..slots.len() {
                        let (name, r) = (slots[i].0.clone(), slots[i].1);
                        values.push(host.read(&name, r));
                    }
                    Frame { time_ms, values: values.clone(), h_events: h_events.to_vec(), provider_events: provider_events.to_vec(), replies, game_strings }
                });
            }
            Scripts::Main { engine, main, loaded, failed } => {
                if *failed {
                    return;
                }
                if !*loaded {
                    match engine.load_module(&mut host, main) {
                        Ok(()) => *loaded = true,
                        Err(e) => {
                            crate::log(&format!("js: {} did not load: {e}", main.display()));
                            *failed = true;
                            return;
                        }
                    }
                }
                if let Err(e) = engine.tick(&mut host, self.time_ms) {
                    host.log(LogLevel::Error, &e);
                }
            }
        }
    }

    /// Events scripts sent (`K:` and `H:` writes), oldest first.
    pub fn take_events(&mut self) -> Vec<(String, f64)> {
        self.state.events.drain(..).collect()
    }

    /// An H: event for the instruments, as a cockpit click sends it.
    #[allow(dead_code)]
    pub fn h_event(&mut self, name: &str) {
        if let Some(i) = hevent_names().iter().position(|n| n == name) {
            HEVENT_QUEUE.with(|q| q.borrow_mut().push(i));
        }
    }

    /// The `fbw/hevent/` commands' handlers off.
    pub fn release(&mut self, xplm: &Xplm) {
        for (command, i) in self.commands.drain(..) {
            xplm.unregister_command_handler(command, on_hevent, i as *mut c_void);
        }
        if let Scripts::Cockpit { worker: Some(worker), .. } = &mut self.scripts {
            worker.stop();
        }
    }

    /// Whether the QuickJS instruments' worker is running (always `true` for
    /// a lone entry module).
    pub fn running(&self) -> bool {
        !matches!(self.scripts, Scripts::Cockpit { worker: None, .. })
    }

    /// Stops FlyByWire's instruments worker: xphfbw_host.rs calls this once
    /// XPHFBW's displays go active (rule 7, "never both engines drawing one
    /// screen"). A no-op for a lone entry module or one already stopped.
    pub fn suspend(&mut self) {
        if let Scripts::Cockpit { worker: worker @ Some(_), .. } = &mut self.scripts {
            if let Some(mut w) = worker.take() {
                w.stop();
            }
        }
    }

    /// Starts a fresh instruments worker: xphfbw_host.rs calls this when
    /// XPHFBW's displays go inactive again (its process went away, rule 7)
    /// so the plugin's own engine resumes drawing. Slots and the resolver's
    /// cache are reset to match the new worker's, which numbers its own
    /// slots from zero. A no-op for a lone entry module or one already
    /// running.
    pub fn resume(&mut self) {
        if let Scripts::Cockpit { worker: worker @ None, recipe } = &mut self.scripts {
            match Worker::start(recipe.make()) {
                Ok(w) => {
                    *worker = Some(w);
                    self.state = HostState::default();
                    self.slots.clear();
                    self.values.clear();
                }
                Err(e) => crate::log(&format!("js: the instruments could not restart: {e}")),
            }
        }
    }
}

/// Parts of FlyByWire's hosts the plugin runs natively, which must not run
/// twice: their instrument is left out of the host's backplane, so it is
/// constructed (subscriptions only) but never initialised or updated. The
/// text is esbuild's output of FlyByWire's development build
/// (docs/js-build.md); a production (minified) build does not match, and
/// the runtime says so.
fn native_ports() -> Vec<crate::js::msfs::SourcePatch> {
    let patch = |path: &str, instrument: &str, field: &str, native: &str| crate::js::msfs::SourcePatch {
        path: path.to_string(),
        find: format!("this.backplane.addInstrument(\"{instrument}\", this.{field});"),
        replace: format!("/* {instrument}: {native} */"),
        reason: format!("{instrument} is left out: {native} does its work"),
    };
    let mut patches = vec![
        // SystemsHost.ts:191,213.
        patch("/Pages/VCockpit/Instruments/A380X/SystemsHost/SystemsHost.js", "LegacyFuel", "legacyFuel", "fuel_transfer.rs"),
        // extras-host index.ts:98,151.
        patch("/Pages/VCockpit/Instruments/A380X/ExtrasHost/index.js", "GPUManagement", "gpuManagement", "efb.rs"),
    ];
    // [oans] amdb.ts's two Navigraph calls, sent through fetch() instead of
    // axios (which cannot run here): src/oans/plugin.rs.
    patches.extend(crate::oans::plugin::source_patches());
    // [wxr] EfisTawsBridge.ts's permanently-failed wxr1Failed/wxr2Failed,
    // tied to real AESU bus power instead: src/wxr/mod.rs.
    patches.extend(crate::wxr::source_patches());
    // [ecam_patches] ECAM/FWS and instrument gaps (top50 #9, 16, 18-27, 36-40):
    // src/ecam_patches.rs.
    patches.extend(crate::ecam_patches::source_patches());
    patches
}

/// The flyPad settings efb.rs keeps in `Output/preferences/fbw_a380x_settings.ini`
/// under their stored-data keys (`A380X_<KEY>`): instruments reading or
/// writing those keys read and write that file, so both agree.
#[derive(Default)]
struct FlyPadSettings {
    cache: Option<(std::time::SystemTime, BTreeMap<String, String>)>,
}

impl FlyPadSettings {
    fn load(&mut self) -> &mut BTreeMap<String, String> {
        let path = crate::efb::settings_path();
        let modified = std::fs::metadata(&path).and_then(|m| m.modified()).ok();
        let fresh = matches!((&self.cache, modified), (Some((at, _)), Some(m)) if *at == m);
        if !fresh {
            let values = std::fs::read_to_string(&path).map(|t| crate::efb::parse_ini(&t)).unwrap_or_default();
            self.cache = Some((modified.unwrap_or(std::time::UNIX_EPOCH), values));
        }
        &mut self.cache.as_mut().expect("loaded").1
    }
}

impl StoredDataBackend for FlyPadSettings {
    fn owns(&self, key: &str) -> bool {
        crate::efb::SETTINGS.iter().any(|s| crate::efb::stored_key(s.key) == key)
    }

    fn get(&mut self, key: &str) -> Option<String> {
        self.load().get(key).cloned()
    }

    // Race/desync rule 8 (docs/briefs/xphfbw-js-bridge.md): the flyPad ini
    // is written by more than one process (this worker, and the app's own
    // save, app/src/settings.rs), so the whole read-modify-write holds the
    // shared settings-files mutex and writes to a temp file and renames.
    fn set(&mut self, key: &str, value: Option<&str>) {
        crate::app_settings::with_settings_lock(|| {
            let values = self.load();
            match value {
                Some(v) => values.insert(key.to_string(), v.to_string()),
                None => values.remove(key),
            };
            let text = crate::efb::write_ini(values);
            let path = crate::efb::settings_path();
            if let Some(dir) = path.parent() {
                let _ = std::fs::create_dir_all(dir);
            }
            let tmp = path.with_extension("ini.tmp");
            if let Err(e) = std::fs::write(&tmp, text).and_then(|()| std::fs::rename(&tmp, &path)) {
                crate::log(&format!("js: could not save the flyPad settings: {e}"));
            }
            self.cache = None;
        });
    }

    fn all(&mut self) -> Vec<(String, String)> {
        self.load().iter().map(|(k, v)| (k.clone(), v.clone())).collect()
    }
}

// [display] The screens' host functions (docs/display-stream.md), backed by
// src/display. `measureText` and `fontMetrics` take the screen as an extra,
// optional last argument, because FlyByWire's screens give one family name
// different fonts.
fn register_display(engine: &Engine) -> Result<(), String> {
    use rquickjs::function::Opt;
    use rquickjs::{Ctx, Exception, Function, Value};

    engine.with_host_object(|ctx, host| {
        fn submit<'js>(ctx: Ctx<'js>, screen: String, ops: Value<'js>, strings: Vec<String>) -> rquickjs::Result<()> {
            let result = match ops.as_object().and_then(|o| o.as_typed_array::<f64>()) {
                // Safety: no script runs while the slice is read.
                Some(array) => crate::display::submit(&screen, unsafe { array.as_slice() }, strings),
                None => {
                    let ops: Vec<f64> = rquickjs::FromJs::from_js(&ctx, ops)?;
                    crate::display::submit(&screen, &ops, strings)
                }
            };
            result.map_err(|e| Exception::throw_message(&ctx, &e))
        }
        host.set("submitDisplay", Function::new(ctx.clone(), submit)?)?;
        let measure = |family: String, size: f64, text: String, screen: Opt<String>| -> f64 {
            crate::display::measure_text(screen.0.as_deref(), &family, size, &text)
        };
        host.set("measureText", Function::new(ctx.clone(), measure)?)?;
        let metrics = |family: String, size: f64, screen: Opt<String>| -> Vec<f64> {
            let (ascent, descent) = crate::display::font_metrics(screen.0.as_deref(), &family, size);
            vec![ascent, descent]
        };
        host.set("fontMetrics", Function::new(ctx.clone(), metrics)?)?;
        fn size<'js>(ctx: Ctx<'js>, screen: String) -> rquickjs::Result<Vec<f64>> {
            match crate::display::screen_size(&screen) {
                Some((w, h)) => Ok(vec![w as f64, h as f64]),
                None => Err(Exception::throw_message(&ctx, &format!("there is no screen {screen}"))),
            }
        }
        host.set("screenSize", Function::new(ctx.clone(), size)?)?;
        fn events<'js>(ctx: Ctx<'js>) -> rquickjs::Result<rquickjs::Array<'js>> {
            let list = rquickjs::Array::new(ctx.clone())?;
            for (i, e) in crate::display::take_events().into_iter().enumerate() {
                let item = rquickjs::Array::new(ctx.clone())?;
                item.set(0, e.screen)?;
                item.set(1, e.kind)?;
                item.set(2, e.x)?;
                item.set(3, e.y)?;
                item.set(4, e.button)?;
                item.set(5, e.delta)?;
                list.set(i, item)?;
            }
            Ok(list)
        }
        host.set("takeScreenEvents", Function::new(ctx.clone(), events)?)?;
        Ok(())
    })
}

/// For a lone entry module: mouse events reach `__screenEvent` at the start
/// of each tick, where the scripts' SimVar writes land. (The instruments'
/// views get theirs from the cockpit, each view its own screen's.)
fn deliver_screen_events_on_tick(engine: &Engine) -> Result<(), String> {
    engine
        .eval(
            "display-events",
            "(() => {
               const tick = globalThis.__tick;
               globalThis.__tick = (ms) => {
                 const events = __host.takeScreenEvents();
                 if (typeof globalThis.__screenEvent === 'function') {
                   for (const e of events) {
                     try { globalThis.__screenEvent(e[0], e[1], e[2], e[3], e[4], e[5]); } catch (err) { console.error(err); }
                   }
                 }
                 return tick(ms);
               };
             })()",
        )
        .map(|_| ())
}

// [navdata] Coherent call providers: the parts of the plugin that answer
// calls scripts make of MSFS's simulator side, starting with the facility
// database (src/navdata). A provider answers `None` for calls that are not
// its own; `CallReply::Pending` means "not yet", and the runtime asks again
// on later ticks. The events providers raise (`SendAirport`,
// `NearestSearchCompleted`, ...) are taken with `take_provider_events`, each
// with its handlers' arguments as a JSON array, for every view
// (`Cockpit::broadcast`).
pub(crate) trait CoherentProvider {
    fn call(&mut self, name: &str, args_json: &str) -> Option<CallReply>;
    fn take_events(&mut self) -> Vec<(String, String)>;
    /// A string game variable this provides (`GAME:` names, spaced).
    fn game_string(&mut self, _name: &str) -> Option<String> {
        None
    }
}

thread_local! {
    static PROVIDERS: RefCell<Vec<Box<dyn CoherentProvider>>> = const { RefCell::new(Vec::new()) };
}

pub(crate) fn add_provider(provider: Box<dyn CoherentProvider>) {
    // [diagnostic] One-time: proves/disproves the theory that a provider
    // registered on one thread is invisible to `provider_game_string`/
    // `provider_call` callers on another, since PROVIDERS is thread_local.
    // Remove once the navdata GameString bug (MFD/ND "FLIGHT NAVDATA DATE
    // RANGE" never answering) is confirmed root-caused.
    crate::log(&format!("js: provider registered on thread {:?}", std::thread::current().id()));
    PROVIDERS.with(|p| p.borrow_mut().push(provider));
}

/// The first provider's answer to a call, if one answers it.
pub(crate) fn provider_call(name: &str, args_json: &str) -> Option<CallReply> {
    PROVIDERS.with(|p| p.borrow_mut().iter_mut().find_map(|provider| provider.call(name, args_json)))
}

pub(crate) fn take_provider_events() -> Vec<(String, String)> {
    PROVIDERS.with(|p| p.borrow_mut().iter_mut().flat_map(|provider| provider.take_events()).collect())
}

pub(crate) fn provider_game_string(name: &str) -> Option<String> {
    // [diagnostic] One-time, only on a miss with an empty provider list:
    // if this ever logs a *different* thread id than "provider registered
    // on thread ..." above for the same run, PROVIDERS' thread_local is the
    // root cause of the navdata GameString bug (the caller's thread never
    // sees the NavData provider `start_navdata` registered elsewhere).
    // Remove once that bug is confirmed root-caused.
    static LOGGED_EMPTY: std::sync::atomic::AtomicBool = std::sync::atomic::AtomicBool::new(false);
    let empty = PROVIDERS.with(|p| p.borrow().is_empty());
    if empty && !LOGGED_EMPTY.swap(true, std::sync::atomic::Ordering::Relaxed) {
        crate::log(&format!(
            "js: provider_game_string({name:?}) found no providers at all on thread {:?} (thread_local mismatch?)",
            std::thread::current().id()
        ));
    }
    PROVIDERS.with(|p| p.borrow_mut().iter_mut().find_map(|provider| provider.game_string(name)))
}

impl CoherentProvider for crate::navdata::NavData {
    fn call(&mut self, name: &str, args_json: &str) -> Option<CallReply> {
        if !crate::navdata::NavData::handles(name) {
            return None;
        }
        Some(match crate::navdata::NavData::call(self, name, args_json) {
            Ok(value) => CallReply::Resolved(value),
            // Still indexing: the runtime asks again on a later tick.
            Err(e) if e == crate::navdata::NOT_READY => CallReply::Pending(0),
            Err(e) => CallReply::Rejected(e),
        })
    }

    fn take_events(&mut self) -> Vec<(String, String)> {
        crate::navdata::NavData::take_events(self)
    }

    /// MSFS's `FLIGHT NAVDATA DATE RANGE`, from X-Plane's navigation data.
    ///
    /// This must answer correctly (or not at all) on the very first ask: a
    /// `game_string` has no "not ready yet" status the way a Coherent call
    /// has `CallReply::Pending` (see `game_strings` above and
    /// `js_worker.rs`'s `WorkerHost::get_string`), so whatever this returns
    /// the first time a view asks is cached on the instruments' worker
    /// forever, with no retry. Answering `None`/empty here while the
    /// multi-second background index was still running used to get cached
    /// that way (Msfs.ts then logs "Failed to parse facilitiesDateRange" on
    /// every later read, for the life of that view). `NavData::cycle` now
    /// reads the AIRAC cycle straight from earth_nav.dat's header at
    /// `NavData::load` (a handful of bytes), so it and `date_range()` are
    /// ready long before the first tick, independent of `is_ready()`.
    fn game_string(&mut self, name: &str) -> Option<String> {
        if name != "FLIGHT NAVDATA DATE RANGE" {
            return None;
        }
        self.date_range()
    }
}

/// Starts indexing X-Plane's navigation data for the facility calls, with
/// X-Plane's magnetic variation model.
fn start_navdata() {
    let Some(root) = crate::xp::system_path() else {
        crate::log("navdata: X-Plane's folder is unknown; the facility database is not available");
        return;
    };
    match crate::navdata::NavData::load(&root) {
        Ok(mut nav) => {
            nav.set_magvar_source(Box::new(|lat, lon| crate::xp::magnetic_variation(lat, lon).map_or(0., f64::from)));
            add_provider(Box::new(nav));
        }
        Err(e) => crate::log(&format!("navdata: {e}")),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn systems_units_come_from_flybywires_msfs_glue() {
        let units = stored_units();
        assert_eq!(units.get("AIRSPEED INDICATED").map(String::as_str), Some("Knots"));
        assert_eq!(units.get("AMBIENT TEMPERATURE").map(String::as_str), Some("celsius"));
        assert_eq!(base_name("CONTACT POINT COMPRESSION:1"), "CONTACT POINT COMPRESSION");
    }

    #[test]
    fn conversions_read_and_write_back() {
        let c = Conversion::between(Some("Knots"), "meters per second");
        assert!((c.read(100.) - 51.444_444).abs() < 1e-3);
        assert!((c.write(c.read(123.)) - 123.).abs() < 1e-9);
        assert!(matches!(Conversion::between(None, "feet"), Conversion::Raw));
        assert_eq!(Conversion::between(Some("Feet"), "Bool").read(3.), 1.);
    }

    #[test]
    fn hevent_names_are_the_cockpits() {
        let names = hevent_names();
        assert!(names.len() > 400);
        assert!(names.iter().any(|n| n == "A32NX_CHRONO_TOGGLE"));
    }

    #[test]
    fn sunrise_and_sunset_bracket_solar_noon_on_the_equator() {
        // Equinox (day 80, ~21 March), longitude 0: solar noon is ~12:00
        // UTC, so sunrise/sunset should fall close to 06:00/18:00 UTC
        // (within the Fourier approximation's few-minute error).
        let sunrise = solar_event_seconds(0., 0., 21, 3, true);
        let sunset = solar_event_seconds(0., 0., 21, 3, false);
        assert!((sunrise - 6. * 3600.).abs() < 20. * 60., "sunrise was {sunrise}s");
        assert!((sunset - 18. * 3600.).abs() < 20. * 60., "sunset was {sunset}s");
        assert!(sunrise < sunset);
    }

    #[test]
    fn sunrise_moves_earlier_going_east() {
        // Each 15 degrees east moves the event about an hour (4 min/degree)
        // earlier in UTC.
        let prime_meridian = solar_event_seconds(0., 0., 21, 3, true);
        let fifteen_east = solar_event_seconds(0., 15., 21, 3, true);
        assert!((prime_meridian - fifteen_east - 3600.).abs() < 120.);
    }

    #[test]
    fn polar_day_and_night_stay_finite() {
        // High-latitude midsummer (day 172, ~21 June): the far north never
        // sets, the far south never rises. Neither should read as NaN/inf;
        // the hour-angle argument is clamped instead of panicking.
        for lat in [89., -89.] {
            let sunrise = solar_event_seconds(lat, 0., 21, 6, true);
            let sunset = solar_event_seconds(lat, 0., 21, 6, false);
            assert!(sunrise.is_finite() && (0. ..86_400.).contains(&sunrise));
            assert!(sunset.is_finite() && (0. ..86_400.).contains(&sunset));
        }
    }
}
