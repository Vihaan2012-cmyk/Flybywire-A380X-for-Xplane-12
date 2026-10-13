//! The doors: MSFS's interactive points, opening and closing at the rate
//! flight_model.cfg gives each, as FlyByWire's systems and model read them.
//!
//! flight_model.cfg's `[INTERACTIVE POINTS]` (lines 803-846) defines twenty
//! points: main deck 0-9, upper deck 10-15, cargo 16-17, the fuel hose 18 and
//! ground power 19, each with its "Open Close Rate" (column 0) and type
//! (column 4: 0 main exit, 1 cargo, 2 emergency, 3 fuel hose, 4 ground power).
//! The rate is one figure for opening and closing alike. Its unit is written as
//! "percent per second of animation"; the A380's 0.4 is taken as the fraction
//! of the travel per second, 2.5 s end to end, as `INTERACTIVE POINT OPEN` is
//! read both in percent and in "percent over 100" (Anims_Door.xml:10,
//! OitSimvarPublisher.tsx:54).
//!
//! - `INTERACTIVE POINT OPEN:n`, percent, follows each point. FlyByWire reads
//!   :0, :2, :6, :8 and :10 for boarding (payload/mod.rs:237-257), :0 and :3
//!   for air conditioning (air_conditioning/mod.rs:257-258), :16 and :17 for
//!   the cargo doors (the cargo door aspect, aspects.rs), and the ECAM and
//!   FWS read :0-:9 (SD CabinDoor.tsx:15, FwsCore.ts:554-573). A value written
//!   to `fbw/INTERACTIVE_POINT_OPEN_n` from outside is taken as where the door
//!   is.
//! - `K:TOGGLE_AIRCRAFT_EXIT`, value point + 1, toggles a point, as the EFB's
//!   ground services (A380Services.tsx:140-146) and the cockpit's door handles
//!   send it. The handles open a passenger door only below 0.8 psi of cabin
//!   differential pressure, or close an open one (Anims_Door.xml:14); that
//!   interlock applies to the handle commands, not to the event.
//! - Commands `fbw/door/<name>/toggle`, `open` and `close`, and X-Plane's own
//!   `sim/flight_controls/door_toggle_N`, `door_open_N` and `door_close_N`,
//!   N = point + 1, which the converted cockpit's door handles and X-Plane's
//!   door keys use (the converter maps `#TOGGLE_ID# (>K:TOGGLE_AIRCRAFT_EXIT)`
//!   to `door_toggle_<TOGGLE_ID>`). X-Plane's own door handling does not see
//!   them: this model owns the doors. Both are handles, with the interlock.
//! - The model: `fbw/anim/ANIM_DOOR_<name>` is the point's percent over 100,
//!   the door template's `(A:INTERACTIVE POINT OPEN:#ID#, Percent)` with the
//!   default ANIM_LENGTH of 100 (Anims_Door.xml:6-11). The converted exterior
//!   animates the main deck doors M1L-M5R only; the upper deck doors have no
//!   clip in it. The cargo doors' `fbw/anim/fwd_door_cargo` and
//!   `aft_door_cargo` follow FlyByWire's hydraulic cargo doors,
//!   `A32NX_FWD_DOOR_CARGO_POSITION` and `A32NX_AFT_DOOR_CARGO_POSITION` over
//!   100 (A380_Fuselage_Behavior.xml:65-76). These datarefs belong to the
//!   aircraft's SASL script, which X-Plane loads after this plugin, so they
//!   are looked up again until found.
//! - N in X-Plane's door commands: the converted .acf defines no doors of its
//!   own (no `acf/_door` entries), and the converter's cockpit bindings fire
//!   `door_toggle_<TOGGLE_ID>` with the door template's TOGGLE_ID, which is
//!   point + 1 (cockpit_bindings.txt: ANIM_DOOR_M1L door_toggle_1 ...
//!   ANIM_DOOR_M5R door_toggle_10), so N = point + 1 throughout.
//! - X-Plane's doors 1-20 follow the points: `sim/cockpit2/switches/door_open`
//!   and `door_open_ratio` (both writable), and
//!   `sim/flightmodel2/misc/door_cycle_time` (writable) the cfg's rate.

use systems::simulation::{SimulatorReaderWriter, VariableIdentifier, VariableRegistry};

use crate::published::{self, Command, Published, Value};
use crate::xp::{DataRef, Xplm};
use crate::Vars;

const FLIGHT_MODEL_CFG: &str = include_str!(
    "../../fbw-aircraft/fbw-a380x/src/base/flybywire-aircraft-a380-842/SimObjects/AirPlanes/FlyByWire_A380X/common/config/flight_model.cfg"
);

/// The points' names: the door template's DOOR_ID for 0-15
/// (Anims_Door.xml:37-148), the cfg's comments for the rest (flight_model.cfg
/// "; Cargo" fwd at z 76.5, aft at z -43; "; Fuel"; "; Ground Power").
pub const NAMES: [&str; 20] = [
    "M1L", "M1R", "M2L", "M2R", "M3L", "M3R", "M4L", "M4R", "M5L", "M5R", "U1L", "U1R", "U2L", "U2R", "U3L", "U3R", "CARGO_FWD",
    "CARGO_AFT", "FUEL_HOSE", "GROUND_POWER",
];

/// The passenger doors' handle interlock (Anims_Door.xml:14), psi.
const HANDLE_MAX_DELTA_PRESSURE_PSI: f64 = 0.8;

/// One `interactive_point.n` line.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Point {
    /// Fraction of the travel per second.
    pub rate: f64,
    /// Feet from the datum: z (back to front), x, y.
    pub position: [f64; 3],
    pub kind: i64,
}

/// The cfg's interactive points, in order.
pub fn parse_points(cfg: &str) -> Vec<Point> {
    let mut points: Vec<(usize, Point)> = cfg
        .lines()
        .filter_map(|line| {
            let line = line.split(';').next()?.trim();
            let (key, value) = line.split_once('=')?;
            let n: usize = key.trim().strip_prefix("interactive_point.")?.parse().ok()?;
            let v: Vec<f64> = value.split(',').filter_map(|s| s.trim().parse().ok()).collect();
            (v.len() >= 5).then(|| (n, Point { rate: v[0], position: [v[1], v[2], v[3]], kind: v[4] as i64 }))
        })
        .collect();
    points.sort_by_key(|p| p.0);
    points.into_iter().map(|p| p.1).collect()
}

/// One door: where it is, in percent, and where it is going.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Door {
    pub open_percent: f64,
    pub opening: bool,
}

impl Door {
    /// Move towards open or closed at `rate` of the travel per second.
    pub fn step(&mut self, rate: f64, delta: f64) {
        let target = if self.opening { 100. } else { 0. };
        let travel = (rate * 100. * delta).max(0.);
        self.open_percent = if self.open_percent < target {
            (self.open_percent + travel).min(target)
        } else {
            (self.open_percent - travel).max(target)
        };
    }

}

/// What a handle asks of a door.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Request {
    Toggle,
    Open,
    Close,
}

/// The doors, without X-Plane.
#[derive(Clone, Debug)]
pub struct DoorModel {
    pub points: Vec<Point>,
    pub doors: Vec<Door>,
}

impl DoorModel {
    pub fn new() -> Self {
        let points = parse_points(FLIGHT_MODEL_CFG);
        let doors = vec![Door::default(); points.len()];
        Self { points, doors }
    }

    /// A request from the event: no interlock.
    pub fn request(&mut self, point: usize, request: Request) {
        if let Some(door) = self.doors.get_mut(point) {
            door.opening = match request {
                Request::Toggle => !door.opening,
                Request::Open => true,
                Request::Close => false,
            };
        }
    }

    /// A request from a door handle: a passenger door (types 0 and 2) opens
    /// only below the interlock's pressure, but always closes.
    pub fn handle(&mut self, point: usize, request: Request, cabin_delta_psi: f64) {
        let Some((door, p)) = self.doors.get(point).zip(self.points.get(point)) else { return };
        let passenger = matches!(p.kind, 0 | 2);
        // The template toggles when the pressure is low or the door is open
        // at all (Anims_Door.xml:14).
        let allowed = !passenger || cabin_delta_psi < HANDLE_MAX_DELTA_PRESSURE_PSI || door.open_percent > 0.;
        let opens = match request {
            Request::Toggle => !door.opening,
            Request::Open => true,
            Request::Close => false,
        };
        if allowed || !opens {
            self.request(point, request);
        }
    }

    pub fn step(&mut self, delta: f64) {
        for (door, point) in self.doors.iter_mut().zip(&self.points) {
            door.step(point.rate, delta);
        }
    }
}

impl Default for DoorModel {
    fn default() -> Self {
        Self::new()
    }
}

struct Commands {
    fbw: Vec<[Command; 3]>,
    xplane: Vec<[Command; 3]>,
}

/// The doors in X-Plane.
pub struct Doors {
    model: DoorModel,
    open_ids: Vec<VariableIdentifier>,
    /// What each `INTERACTIVE POINT OPEN` held after the last tick.
    written: Vec<f64>,
    delta_pressure: VariableIdentifier,
    cargo_positions: [VariableIdentifier; 2],
    anims: Vec<Option<DataRef>>,
    cargo_anims: [Option<DataRef>; 2],
    /// Ticks until the missing model datarefs are looked for again.
    lookup_in: u32,
    door_open: Option<DataRef>,
    door_ratio: Option<DataRef>,
    cycle_time: Option<DataRef>,
    commands: Commands,
    /// Unregistered with the plugin, when it is dropped.
    _published: Published,
    open_values: Vec<Value>,
}

impl Doors {
    pub fn new(vars: &mut Vars, xplm: &Xplm) -> Self {
        let model = DoorModel::new();
        let n = model.points.len();
        let mut p = Published::default();
        let mut commands = Commands { fbw: Vec::new(), xplane: Vec::new() };
        let mut open_values = Vec::new();
        for (i, name) in NAMES.iter().enumerate().take(n) {
            commands.fbw.push(["toggle", "open", "close"].map(|what| {
                p.command(&format!("fbw/door/{name}/{what}"), &format!("Door {name} (interactive point {i}): {what}"))
            }));
            commands.xplane.push(
                ["toggle", "open", "close"].map(|what| p.intercept(&format!("sim/flight_controls/door_{what}_{}", i + 1))),
            );
            open_values.push(p.number(&format!("fbw/door/{name}/open_ratio"), 0., false));
        }
        let open_ids = (0..n).map(|i| vars.get(format!("INTERACTIVE POINT OPEN:{i}"))).collect();
        // Door clips exist for the main deck only (the converted objects).
        let anims = NAMES.iter().take(n).map(|name| xplm.find(&format!("fbw/anim/ANIM_DOOR_{name}"))).collect();
        let doors = Self {
            open_ids,
            written: vec![0.; n],
            delta_pressure: vars.get("PRESS_MAN_CABIN_DELTA_PRESSURE".into()),
            cargo_positions: [vars.get("FWD_DOOR_CARGO_POSITION".into()), vars.get("AFT_DOOR_CARGO_POSITION".into())],
            anims,
            cargo_anims: [xplm.find("fbw/anim/fwd_door_cargo"), xplm.find("fbw/anim/aft_door_cargo")],
            lookup_in: 0,
            door_open: xplm.find("sim/cockpit2/switches/door_open"),
            door_ratio: xplm.find("sim/cockpit2/switches/door_open_ratio"),
            cycle_time: xplm.find("sim/flightmodel2/misc/door_cycle_time"),
            model,
            commands,
            _published: p,
            open_values,
        };
        if let Some(d) = doors.cycle_time {
            for (i, point) in doors.model.points.iter().enumerate().filter(|(_, p)| p.rate > 0.) {
                xplm.set_vf_at(d, i, (1. / point.rate) as f32);
            }
        }
        doors
    }

    /// `K:TOGGLE_AIRCRAFT_EXIT`. Returns whether the event was a door's.
    #[cfg_attr(not(feature = "js"), allow(dead_code))]
    pub fn handle_event(&mut self, name: &str, value: f64) -> bool {
        if name.trim().trim_start_matches("K:") != "TOGGLE_AIRCRAFT_EXIT" {
            return false;
        }
        let exit = value.round() as i64;
        if exit >= 1 {
            self.model.request(exit as usize - 1, Request::Toggle);
        }
        true
    }

    /// The model, for the EFB interface.
    pub fn model(&self) -> &DoorModel {
        &self.model
    }

    pub fn request(&mut self, point: usize, request: Request) {
        self.model.request(point, request);
    }

    /// Put a point where something outside the door model has it (the ground
    /// power cable, which has no rate of its own), before [`Doors::update`].
    pub fn set_open(&mut self, point: usize, percent: f64) {
        if let Some(door) = self.model.doors.get_mut(point) {
            door.open_percent = percent.clamp(0., 100.);
            door.opening = percent > 0.;
        }
    }

    /// Look the SASL-owned model datarefs up again, once a second at most.
    fn find_model_datarefs(&mut self, xplm: &Xplm) {
        if self.anims.iter().chain(&self.cargo_anims).all(Option::is_some) {
            return;
        }
        if self.lookup_in > 0 {
            self.lookup_in -= 1;
            return;
        }
        self.lookup_in = 60;
        for (anim, name) in self.anims.iter_mut().zip(NAMES) {
            if anim.is_none() {
                *anim = xplm.find(&format!("fbw/anim/ANIM_DOOR_{name}"));
            }
        }
        for (anim, name) in self.cargo_anims.iter_mut().zip(["fbw/anim/fwd_door_cargo", "fbw/anim/aft_door_cargo"]) {
            if anim.is_none() {
                *anim = xplm.find(name);
            }
        }
    }

    /// Before the systems, which read the doors.
    pub fn update(&mut self, vars: &mut Vars, xplm: &Xplm, delta: f64) {
        self.find_model_datarefs(xplm);
        // A value written from outside is where the door is now.
        for (i, id) in self.open_ids.iter().enumerate() {
            let value = vars.read(id);
            if (value - self.written[i]).abs() > 1e-6 {
                let door = &mut self.model.doors[i];
                door.open_percent = value.clamp(0., 100.);
                door.opening = value > 0.;
            }
        }
        let delta_psi = vars.read(&self.delta_pressure);
        let requests = [Request::Toggle, Request::Open, Request::Close];
        for i in 0..self.commands.fbw.len() {
            for (k, request) in requests.iter().enumerate() {
                let presses = published::presses(self.commands.fbw[i][k]) + published::presses(self.commands.xplane[i][k]);
                for _ in 0..presses {
                    self.model.handle(i, *request, delta_psi);
                }
            }
        }
        self.model.step(delta);

        for (i, door) in self.model.doors.iter().enumerate() {
            vars.write_from_xplane(&self.open_ids[i], door.open_percent);
            self.written[i] = door.open_percent;
            let ratio = door.open_percent / 100.;
            if let Some(d) = self.anims[i] {
                xplm.set_f(d, ratio as f32);
            }
            if let Some(d) = self.door_open {
                xplm.set_vi_at(d, i, door.opening as i32);
            }
            if let Some(d) = self.door_ratio {
                xplm.set_vf_at(d, i, ratio as f32);
            }
            published::set(self.open_values[i], ratio);
        }
    }

    /// After the systems: the hydraulic cargo doors' clips.
    pub fn update_model(&self, vars: &mut Vars, xplm: &Xplm) {
        for (id, anim) in self.cargo_positions.iter().zip(self.cargo_anims) {
            if let Some(d) = anim {
                xplm.set_f(d, (vars.read(id) / 100.).clamp(0., 1.) as f32);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_cfg_defines_twenty_points_named_by_the_door_template() {
        let m = DoorModel::new();
        assert_eq!(m.points.len(), 20);
        assert_eq!(NAMES.len(), 20);
        // flight_model.cfg interactive_point.0 and .16-.19.
        assert_eq!(m.points[0], Point { rate: 0.4, position: [96.4, -8.65, 2.32], kind: 0 });
        assert_eq!(m.points[3].kind, 2);
        assert_eq!((m.points[16].kind, m.points[16].position[0]), (1, 76.5));
        assert_eq!((m.points[17].kind, m.points[17].position[0]), (1, -43.));
        assert_eq!((m.points[18].kind, m.points[18].rate), (3, 0.));
        assert_eq!(m.points[19].kind, 4);
    }

    #[test]
    fn a_door_travels_at_its_rate_and_reverses_when_toggled() {
        let mut m = DoorModel::new();
        m.request(0, Request::Toggle);
        m.step(1.);
        assert!((m.doors[0].open_percent - 40.).abs() < 1e-9, "0.4 of the travel a second");
        m.request(0, Request::Toggle);
        m.step(0.5);
        assert!((m.doors[0].open_percent - 20.).abs() < 1e-9);
        m.request(0, Request::Open);
        m.step(10.);
        assert_eq!(m.doors[0].open_percent, 100.);
        // A rate of 0 (the fuel hose) never moves.
        m.request(18, Request::Open);
        m.step(10.);
        assert_eq!(m.doors[18].open_percent, 0.);
    }

    #[test]
    fn a_handle_will_not_open_a_pressurised_passenger_door() {
        let mut m = DoorModel::new();
        m.handle(2, Request::Toggle, 1.5);
        assert!(!m.doors[2].opening);
        // Cargo doors have no such interlock.
        m.handle(16, Request::Open, 1.5);
        assert!(m.doors[16].opening);
        m.handle(2, Request::Open, 0.2);
        m.step(0.1);
        // Once open at all it can be operated whatever the pressure.
        m.handle(2, Request::Toggle, 1.5);
        assert!(!m.doors[2].opening);
    }
}
