//! The rest of FlyByWire's fly-by-wire module glue (fbw_a380
//! FlyByWireInterface.cpp) that prim.rs does not carry, and simulator
//! variables FlyByWire's extras-host reads that the plugin did not feed:
//!
//! - updateFlyByWire (cpp:2886-2905): `A32NX_SIDESTICK_POSITION_X/Y`, the
//!   rudder pedal animation position and `A32NX_FLIGHT_CONTROLS_TRACKING_MODE`
//!   (the rudder pedal position itself is handling.rs's);
//! - updatePerformanceMonitoring (cpp:1046-1071) and handleSimulationRate
//!   (cpp:1073-1132), with SimConnectInterface's SIM_RATE_INCR/DECR limits
//!   (SimConnectInterface.cpp:3692-3722) applied to X-Plane's sim speed;
//! - `ON ANY RUNWAY` from apt.dat's runways, and `E:TIME OF DAY` from the sun
//!   (LightSync.ts:106-135 reads both);
//! - the reverser force `a380_systems_wasm/reversers.rs` applies to MSFS's
//!   object: `VELOCITY BODY Z` += `REVERSER_DELTA_SPEED`, `ROTATION
//!   ACCELERATION BODY Y` += 10x `REVERSER_ANGULAR_ACCELERATION`
//!   (reversers.rs:34, 74-79). Both are already computed every tick by the
//!   unchanged `ReverserForce` (engine/reverser_thrust.rs) inside
//!   `Simulation<A380>`; only the MSFS object write is X-Plane-specific, so
//!   it is ported here the way pushback.rs nudges the aircraft: a delta added
//!   to X-Plane's velocity and yaw rate, gated on either value being nonzero
//!   exactly as reversers.rs's `ObjectWrite::on` gates the write. FlyByWire's
//!   low-speed boost (reversers.rs:29-31, "to overcome magical MSFS static
//!   ground friction in reverse") is an MSFS-only workaround with no
//!   equivalent bug documented for X-Plane's own ground friction, so it is
//!   left out. `engine_commands.rs` zeroes X-Plane's own reverse throttle so
//!   this is the reverse thrust's only source.
//!
//! What X-Plane cannot give FlyByWire here:
//! - Slew and pause for the tracking mode: X-Plane has no slew, and the
//!   plugin's flight loop does not tick while paused, so the tracking mode is
//!   the external override alone.
//! - `GLASSCOCKPIT AUTOMATIC BRIGHTNESS` (LightSync.ts:181): MSFS computes it
//!   inside the simulator and does not document how; nothing is fed, so
//!   LightSync's clamp gives its 15 % floor.
//! - The ModelConfiguration.ini FlyByWire reads the rate limits from
//!   (cpp:179-220): its defaults are used (minimum 1, maximum 4, limiting by
//!   performance and reduction both on), and the `A32NX_SIMULATION_RATE_LIMIT_*`
//!   variables can change the limits as in MSFS (cpp:1018).

use std::io::{BufRead, BufReader};
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicU8, Ordering};
use std::sync::mpsc::{channel, Receiver};

use systems::simulation::{SimulatorReaderWriter, VariableIdentifier, VariableRegistry};

use crate::extra_backend::XplaneIo;
use crate::prim::SimReadings;

/// FlyByWireInterface.h:37-38.
const MAX_ACCEPTABLE_SAMPLE_TIME: f64 = 1. / 6.;
const LOW_PERFORMANCE_TIMER_THRESHOLD: u32 = 3 * 6;

const SIM_SPEED: &str = "sim/time/sim_speed";
const TRUE_HEADING: &str = "sim/flightmodel/position/psi";
const LOCAL_VX: &str = "sim/flightmodel/position/local_vx";
const LOCAL_VZ: &str = "sim/flightmodel/position/local_vz";
const YAW_RATE: &str = "sim/flightmodel/position/Rrad";
const FT_TO_M: f64 = 0.3048;
/// reversers.rs:34.
const ASYMMETRY_EFFECT_MAGIC_MULTIPLIER: f64 = 10.;

/// MSFS's `E:TIME OF DAY` enum: 0 dawn, 1 day, 2 dusk, 3 night (SDK,
/// Environment Variables). Kept here for the script runtime to answer with.
static TIME_OF_DAY: AtomicU8 = AtomicU8::new(1);

/// The latest `E:TIME OF DAY`.
#[allow(dead_code)]
pub fn time_of_day() -> f64 {
    TIME_OF_DAY.load(Ordering::Relaxed) as f64
}

/// `E:TIME OF DAY` from the sun. MSFS does not document its thresholds; the
/// civil twilight definition is used: day with the sun above the horizon,
/// night below -6 degrees, dawn or dusk between, dawn while the sun is in the
/// eastern half of the sky (heading below 180 degrees true), dusk otherwise.
pub fn time_of_day_from_sun(sun_pitch_deg: f64, sun_heading_deg: f64) -> u8 {
    if sun_pitch_deg > 0. {
        1
    } else if sun_pitch_deg < -6. {
        3
    } else if sun_heading_deg.rem_euclid(360.) < 180. {
        0
    } else {
        2
    }
}

/// One runway: its ends and width.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Runway {
    pub ends: [(f64, f64); 2],
    pub width_m: f64,
}

impl Runway {
    /// Whether a point is on the paved rectangle between the thresholds
    /// (apt.dat row 100's end coordinates, which include displaced
    /// thresholds but not blast pads).
    pub fn contains(&self, lat: f64, lon: f64) -> bool {
        let (lat0, lon0) = self.ends[0];
        let k = lat0.to_radians().cos();
        let m = 111_319.49;
        let (x1, y1) = ((self.ends[1].1 - lon0) * m * k, (self.ends[1].0 - lat0) * m);
        let (px, py) = ((lon - lon0) * m * k, (lat - lat0) * m);
        let len2 = x1 * x1 + y1 * y1;
        if len2 <= 0. {
            return false;
        }
        let t = (px * x1 + py * y1) / len2;
        let along = (0. ..=1.).contains(&t);
        let cross = (px * y1 - py * x1).abs() / len2.sqrt();
        along && cross <= self.width_m / 2.
    }
}

/// apt.dat 1200 row 100 (land runway): width at field 1, the ends'
/// latitude and longitude at fields 9-10 and 18-19.
pub fn parse_runway(line: &str) -> Option<Runway> {
    let f: Vec<&str> = line.split_whitespace().collect();
    if f.first() != Some(&"100") || f.len() < 20 {
        return None;
    }
    let n = |i: usize| f[i].parse::<f64>().ok();
    Some(Runway { width_m: n(1)?, ends: [(n(9)?, n(10)?), (n(18)?, n(19)?)] })
}

/// Runways by whole-degree cell of their first end.
#[derive(Default)]
pub struct Runways {
    cells: std::collections::HashMap<(i32, i32), Vec<Runway>>,
}

impl Runways {
    pub fn add(&mut self, r: Runway) {
        let cell = (r.ends[0].0.floor() as i32, r.ends[0].1.floor() as i32);
        self.cells.entry(cell).or_default().push(r);
    }

    pub fn on_any(&self, lat: f64, lon: f64) -> bool {
        let (la, lo) = (lat.floor() as i32, lon.floor() as i32);
        (-1..=1).any(|dy| (-1..=1).any(|dx| self.cells.get(&(la + dy, lo + dx)).is_some_and(|v| v.iter().any(|r| r.contains(lat, lon)))))
    }

    pub fn read(&mut self, path: &Path) -> std::io::Result<()> {
        let reader = BufReader::with_capacity(1 << 20, std::fs::File::open(path)?);
        for line in reader.split(b'\n') {
            let line = line?;
            if line.starts_with(b"100 ") {
                if let Some(r) = parse_runway(&String::from_utf8_lossy(&line)) {
                    self.add(r);
                }
            }
        }
        Ok(())
    }
}

/// X-Plane's apt.dat files, relative to X-Plane's folder (the plugin's
/// working directory, as fuel.rs's paths assume): Global Airports and every
/// custom scenery pack that has one.
fn apt_dat_files() -> Vec<PathBuf> {
    let mut files = vec![PathBuf::from("Global Scenery/Global Airports/Earth nav data/apt.dat")];
    if let Ok(dirs) = std::fs::read_dir("Custom Scenery") {
        files.extend(dirs.flatten().map(|d| d.path().join("Earth nav data").join("apt.dat")).filter(|p| p.is_file()));
    }
    files
}

pub struct FlyByWireGlue {
    ids: Ids,
    runways: Option<Runways>,
    loading: Option<Receiver<Runways>>,
    low_performance_timer: u32,
    target_rate: Option<f64>,
    last_rate: f64,
}

struct Ids {
    sidestick_x: VariableIdentifier,
    sidestick_y: VariableIdentifier,
    pedal_animation: VariableIdentifier,
    rudder_trim_actual: VariableIdentifier,
    tracking_mode: VariableIdentifier,
    external_override: VariableIdentifier,
    performance_warning: VariableIdentifier,
    min_rate: VariableIdentifier,
    max_rate: VariableIdentifier,
    on_any_runway: VariableIdentifier,
    on_ground: VariableIdentifier,
    latitude: VariableIdentifier,
    reverser_delta_speed: VariableIdentifier,
    reverser_angular_accel: VariableIdentifier,
}

/// What the glue reads from X-Plane each tick.
#[derive(Clone, Copy, Debug, Default)]
pub struct Readings {
    pub sim: SimReadings,
    pub longitude: f64,
    /// X-Plane's pitch and bank, positive nose up and right wing down.
    pub theta_deg: f64,
    pub phi_deg: f64,
    pub sun_pitch_deg: f64,
    pub sun_heading_deg: f64,
}

impl FlyByWireGlue {
    pub fn new<V: VariableRegistry + SimulatorReaderWriter>(vars: &mut V, load_runways: bool) -> Self {
        let g = |vars: &mut V, n: &str| vars.get(n.to_string());
        let ids = Ids {
            sidestick_x: g(vars, "SIDESTICK_POSITION_X"),
            sidestick_y: g(vars, "SIDESTICK_POSITION_Y"),
            pedal_animation: g(vars, "RUDDER_PEDAL_ANIMATION_POSITION"),
            rudder_trim_actual: g(vars, "RUDDER_TRIM_ACTUAL_POSITION"),
            tracking_mode: g(vars, "FLIGHT_CONTROLS_TRACKING_MODE"),
            external_override: g(vars, "EXTERNAL_OVERRIDE"),
            performance_warning: g(vars, "PERFORMANCE_WARNING_ACTIVE"),
            min_rate: g(vars, "SIMULATION_RATE_LIMIT_MINIMUM"),
            max_rate: g(vars, "SIMULATION_RATE_LIMIT_MAXIMUM"),
            on_any_runway: vars.get_unprefixed("ON ANY RUNWAY".to_string()),
            on_ground: vars.get_unprefixed("SIM ON GROUND".to_string()),
            latitude: vars.get_unprefixed("PLANE LATITUDE".to_string()),
            // The same identifiers `ReverserForce` (engine/reverser_thrust.rs,
            // already running inside `Simulation<A380>`) registers, so these
            // resolve to what it wrote this tick.
            reverser_delta_speed: g(vars, "REVERSER_DELTA_SPEED"),
            reverser_angular_accel: g(vars, "REVERSER_ANGULAR_ACCELERATION"),
        };
        // ModelConfiguration.ini defaults (cpp:211-212).
        vars.write(&ids.min_rate, 1.);
        vars.write(&ids.max_rate, 4.);
        let loading = load_runways.then(|| {
            let (tx, rx) = channel();
            let _ = std::thread::Builder::new().name("fbw-runways".into()).spawn(move || {
                let mut runways = Runways::default();
                for f in apt_dat_files() {
                    if let Err(e) = runways.read(&f) {
                        crate::log(&format!("runways: {}: {e}", f.display()));
                    }
                }
                let _ = tx.send(runways);
            });
            rx
        });
        Self { ids, runways: None, loading, low_performance_timer: 0, target_rate: None, last_rate: 1. }
    }

    /// Before the PRIMs: updateFlyByWire's outputs and the extras' inputs.
    pub fn update<V: VariableRegistry + SimulatorReaderWriter, X: XplaneIo>(&mut self, vars: &mut V, xplane: &mut X, r: &Readings, delta: f64) {
        let i = &self.ids;
        // cpp:2892-2901.
        vars.write(&i.sidestick_x, -r.sim.inputs[1]);
        vars.write(&i.sidestick_y, -r.sim.inputs[0]);
        let trim = vars.read(&i.rudder_trim_actual);
        vars.write(&i.pedal_animation, (-100. * (r.sim.inputs[2] + trim / 30.)).clamp(-100., 100.));
        let tracking = vars.read(&i.external_override) != 0.;
        vars.write(&i.tracking_mode, tracking as i32 as f64);

        if let Some(runways) = self.loading.as_ref().and_then(|rx| rx.try_recv().ok()) {
            self.runways = Some(runways);
            self.loading = None;
        }
        let on_ground = vars.read(&i.on_ground) != 0.;
        let lat = vars.read(&i.latitude);
        let on_runway = on_ground && self.runways.as_ref().is_some_and(|rw| rw.on_any(lat, r.longitude));
        vars.write(&i.on_any_runway, on_runway as i32 as f64);
        TIME_OF_DAY.store(time_of_day_from_sun(r.sun_pitch_deg, r.sun_heading_deg), Ordering::Relaxed);

        self.apply_reverser_thrust(vars, xplane, delta);
        self.performance_and_rate(vars, xplane, r, delta);
    }

    /// The MSFS object write `a380_systems_wasm/reversers.rs` makes each
    /// frame (`ReverserThrust::write`, reversers.rs:66-79), ported to
    /// X-Plane's velocity and yaw rate the way pushback.rs nudges the
    /// aircraft. `REVERSER_DELTA_SPEED` and `REVERSER_ANGULAR_ACCELERATION`
    /// are this tick's values from `ReverserForce`, already run inside
    /// `Simulation<A380>` before this glue's caller reads them (the same one
    /// tick of lag FlyByWire's own `ExecuteOn::PreTick` read has in MSFS).
    fn apply_reverser_thrust<V: VariableRegistry + SimulatorReaderWriter, X: XplaneIo>(&mut self, vars: &mut V, xplane: &mut X, delta: f64) {
        let delta_speed = vars.read(&self.ids.reverser_delta_speed);
        let dissymetry = vars.read(&self.ids.reverser_angular_accel);
        // reversers.rs:78: `ObjectWrite::on(values[1].abs() > 0. || values[3].abs() > 0.)`.
        if delta_speed == 0. && dissymetry == 0. {
            return;
        }
        let heading = xplane.get(TRUE_HEADING, None).unwrap_or(0.).to_radians();
        let dv = delta_speed * FT_TO_M;
        let vx = xplane.get(LOCAL_VX, None).unwrap_or(0.) + dv * heading.sin();
        let vz = xplane.get(LOCAL_VZ, None).unwrap_or(0.) - dv * heading.cos();
        xplane.set(LOCAL_VX, None, vx);
        xplane.set(LOCAL_VZ, None, vz);
        // ROTATION ACCELERATION BODY Y, integrated over this tick as X-Plane
        // has no writable rotation acceleration to hand it to directly.
        let r = xplane.get(YAW_RATE, None).unwrap_or(0.) + ASYMMETRY_EFFECT_MAGIC_MULTIPLIER * dissymetry * delta;
        xplane.set(YAW_RATE, None, r);
    }

    /// updatePerformanceMonitoring, the SIM_RATE_INCR limit and
    /// handleSimulationRate. X-Plane changes its sim speed itself, so the
    /// limits are applied to the change once it has happened.
    fn performance_and_rate<V: VariableRegistry + SimulatorReaderWriter, X: XplaneIo>(&mut self, vars: &mut V, xplane: &mut X, r: &Readings, delta: f64) {
        let Some(rate) = xplane.get(SIM_SPEED, None) else { return };
        // X-Plane's frame time is real time; FlyByWire's sample time is
        // simulation time (cpp:1035).
        let sample_time = (delta * rate.max(1.)).max(0.002);
        // cpp:1048-1067.
        if sample_time > MAX_ACCEPTABLE_SAMPLE_TIME && self.low_performance_timer < LOW_PERFORMANCE_TIMER_THRESHOLD {
            self.low_performance_timer += 1;
        } else if sample_time < MAX_ACCEPTABLE_SAMPLE_TIME {
            self.low_performance_timer = 0;
        }
        let warning = self.low_performance_timer >= LOW_PERFORMANCE_TIMER_THRESHOLD;
        vars.write(&self.ids.performance_warning, warning as i32 as f64);

        let (min_rate, max_rate) = (vars.read(&self.ids.min_rate), vars.read(&self.ids.max_rate));
        let set = |xplane: &mut X, to: f64, why: &str| {
            crate::log(&format!("simulation rate {rate} -> {to} ({why})"));
            xplane.set(SIM_SPEED, None, to);
        };
        // SIM_RATE_INCR (SimConnectInterface.cpp:3692-3708), with the
        // theoretical frame rate at the previous rate.
        if rate > self.last_rate && self.last_rate >= 1. {
            let theoretical_fps = (1. / (delta * self.last_rate).max(0.002)) / (self.last_rate * 2.);
            if !(self.last_rate < max_rate && theoretical_fps >= 6.) {
                set(xplane, self.last_rate, "limited by max sim rate or theoretical fps");
                self.last_rate = self.last_rate.max(1.);
                return;
            }
        }
        // SIM_RATE_DECR (cpp:3710-3721).
        if rate < self.last_rate && rate < min_rate && self.last_rate >= min_rate {
            set(xplane, self.last_rate, "limited by min sim rate");
            return;
        }
        self.last_rate = rate;

        // handleSimulationRate (cpp:1077-1128).
        if let Some(target) = self.target_rate {
            if rate != target {
                return;
            }
        }
        self.target_rate = None;
        if rate <= 1. {
            return;
        }
        let halved = (rate / 2.).max(1.).floor();
        if rate > max_rate {
            self.target_rate = Some(halved);
            set(xplane, halved, "maximum allowed exceeded");
            return;
        }
        // MSFS's Theta is positive nose down (cpp:1176 negates it for use).
        let msfs_theta = -r.theta_deg;
        if warning || r.phi_deg.abs() > 33. || msfs_theta < -20. || msfs_theta > 10. {
            self.target_rate = Some(halved);
            self.low_performance_timer = 0;
            set(xplane, halved, "performance issues or abnormal situation");
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::aspects::test_vars::TestVars;
    use crate::extra_backend::sim::test_xplane::FakeXplane;

    #[test]
    fn apt_dat_runways_contain_the_centreline_and_not_beside_it() {
        // EDDF 07C/25C (apt.dat 1200 row 100).
        let line = "100 45.11 1 0 0.25 1 3 0 07C 50.03253400 008.53454300 0.00 0.00 3 7 0 1 25C 50.04503300 008.58693500 0.00 0.00 3 11 0 1";
        let r = parse_runway(line).unwrap();
        assert_eq!(r.width_m, 45.11);
        let mid = ((r.ends[0].0 + r.ends[1].0) / 2., (r.ends[0].1 + r.ends[1].1) / 2.);
        assert!(r.contains(mid.0, mid.1));
        // 100 m north of the centreline.
        assert!(!r.contains(mid.0 + 100. / 111_319.49, mid.1));
        let mut all = Runways::default();
        all.add(r);
        assert!(all.on_any(mid.0, mid.1));
        assert!(!parse_runway("101 49 1 06 47.1 -122.5 24 47.2 -122.4").is_some());
    }

    #[test]
    fn time_of_day_follows_civil_twilight() {
        assert_eq!(time_of_day_from_sun(20., 150.), 1);
        assert_eq!(time_of_day_from_sun(-3., 90.), 0);
        assert_eq!(time_of_day_from_sun(-3., 270.), 2);
        assert_eq!(time_of_day_from_sun(-12., 0.), 3);
    }

    #[test]
    fn fly_by_wire_outputs_follow_the_sidestick_and_pedals() {
        let mut vars = TestVars::default();
        let mut xp = FakeXplane::default();
        let mut g = FlyByWireGlue::new(&mut vars, false);
        let mut r = Readings::default();
        r.sim.inputs = [0.5, -0.25, 0.1];
        xp.values.insert(SIM_SPEED.into(), 1.);
        vars.set("A32NX_RUDDER_TRIM_ACTUAL_POSITION", 3.);
        g.update(&mut vars, &mut xp, &r, 0.02);
        assert_eq!(vars.value("A32NX_SIDESTICK_POSITION_X"), 0.25);
        assert_eq!(vars.value("A32NX_SIDESTICK_POSITION_Y"), -0.5);
        assert!((vars.value("A32NX_RUDDER_PEDAL_ANIMATION_POSITION") + 20.).abs() < 1e-9);
        assert_eq!(vars.value("A32NX_FLIGHT_CONTROLS_TRACKING_MODE"), 0.);
        vars.set("A32NX_EXTERNAL_OVERRIDE", 1.);
        g.update(&mut vars, &mut xp, &r, 0.02);
        assert_eq!(vars.value("A32NX_FLIGHT_CONTROLS_TRACKING_MODE"), 1.);
    }

    #[test]
    fn simulation_rate_is_limited_as_flybywire_limits_it() {
        let mut vars = TestVars::default();
        let mut xp = FakeXplane::default();
        let mut g = FlyByWireGlue::new(&mut vars, false);
        let r = Readings::default();
        xp.values.insert(SIM_SPEED.into(), 1.);
        g.update(&mut vars, &mut xp, &r, 0.02);
        // 1x -> 2x at 50 fps: allowed (theoretical 25 fps).
        xp.values.insert(SIM_SPEED.into(), 2.);
        g.update(&mut vars, &mut xp, &r, 0.02);
        assert_eq!(xp.values[SIM_SPEED], 2.);
        // 2x -> 4x at 5 fps: refused.
        xp.values.insert(SIM_SPEED.into(), 4.);
        g.update(&mut vars, &mut xp, &r, 0.2);
        assert_eq!(xp.values[SIM_SPEED], 2.);
        // Bank beyond 33 degrees at 2x: halved.
        let steep = Readings { phi_deg: 40., ..Default::default() };
        g.update(&mut vars, &mut xp, &steep, 0.02);
        assert_eq!(xp.values[SIM_SPEED], 1.);
    }

    #[test]
    fn reverser_thrust_nudges_velocity_and_yaw_only_when_active() {
        let mut vars = TestVars::default();
        let mut xp = FakeXplane::default();
        let mut g = FlyByWireGlue::new(&mut vars, false);
        let r = Readings::default();
        xp.values.insert(SIM_SPEED.into(), 1.);
        xp.values.insert(TRUE_HEADING.into(), 0.);
        // No reverser force: X-Plane's velocity and yaw rate are untouched.
        g.update(&mut vars, &mut xp, &r, 0.02);
        assert!(!xp.values.contains_key(LOCAL_VZ));
        assert!(!xp.values.contains_key(YAW_RATE));
        // Reversing decelerates (negative REVERSER_DELTA_SPEED) and yaws with
        // an asymmetric reverser (REVERSER_ANGULAR_ACCELERATION).
        vars.set("A32NX_REVERSER_DELTA_SPEED", -10.);
        vars.set("A32NX_REVERSER_ANGULAR_ACCELERATION", 0.5);
        g.update(&mut vars, &mut xp, &r, 0.02);
        assert!((xp.values[LOCAL_VZ] - 3.048).abs() < 1e-6, "{}", xp.values[LOCAL_VZ]);
        assert_eq!(xp.values[LOCAL_VX], 0.);
        assert!((xp.values[YAW_RATE] - 0.1).abs() < 1e-9, "{}", xp.values[YAW_RATE]);
    }
}
