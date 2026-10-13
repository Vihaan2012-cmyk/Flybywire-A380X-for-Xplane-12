//! FlyByWire's pushback (fbw-common/src/wasm/extra-backend/Pushback/
//! Pushback.cpp and .h, with Pushback_A380X.h's factors): the flyPad's
//! pushback tool, which moves the aircraft from `A32NX_PUSHBACK_SPD_FACTOR`
//! and `A32NX_PUSHBACK_HDG_FACTOR` (-1..1) while `A32NX_PUSHBACK_SYSTEM_ENABLED`
//! is set and the tug is attached.
//!
//! How FlyByWire moves the aircraft in MSFS (Pushback.cpp:121-213): every
//! visual frame it writes the sim object's body velocities through a
//! SimConnect data definition: `VELOCITY BODY Z` = the damped speed (ft/s),
//! `ROTATION VELOCITY BODY Y` = the damped turn rate, every other body
//! velocity zero, plus a pitch `ROTATION ACCELERATION BODY X` of -1 or +2
//! against a pitching MSFS adds on its own, and `PUSHBACK WAIT`. It also
//! sends `KEY_TUG_HEADING` to point MSFS's tug.
//!
//! X-Plane: the same velocities, written each flight loop to the writable
//! velocity datarefs the flight model integrates (DataRefs.txt):
//! `sim/flightmodel/position/local_vx` and `local_vz` (m/s, local OpenGL
//! axes: x east, z south) along the true heading `psi`, and `Rrad` (rad/s,
//! yaw rate, positive nose right as MSFS's body Y rotation). Nothing is
//! overridden: X-Plane's gear, tyres and collisions keep working, the aircraft
//! is not teleported, and the vertical velocity `local_vy` is left to
//! X-Plane. Not carried over:
//! - the pitch counter-acceleration, which compensates an MSFS artefact
//!   (Pushback.cpp:171-178, "The sim seems to add this rotation"); X-Plane's
//!   `Q_dot` is not writable and X-Plane adds no such rotation;
//! - `PUSHBACK WAIT` and `KEY_TUG_HEADING`, which drive MSFS's own tug; X-Plane
//!   has no tug here to point. The tug heading FlyByWire computes is still
//!   published in the debug variable.
//!
//! The tug: MSFS's `PUSHBACK ATTACHED` comes from MSFS's tug, which the flyPad
//! attaches with `K:TOGGLE_PUSHBACK` (PushbackPage.tsx callTug). X-Plane's own
//! tug (`sim/aircraft/overflow/pushback_attached`, read only) only runs canned
//! manoeuvres (`sim/ground_ops/pushback_left`, `_straight`, `_right`,
//! Commands.txt) and moves the aircraft itself, so it cannot take FlyByWire's
//! speed and heading factors. The plugin therefore keeps `PUSHBACK ATTACHED`
//! as a variable (`fbw/PUSHBACK_ATTACHED`), toggled by `K:TOGGLE_PUSHBACK` and
//! the command `fbw/pushback/toggle`; the pushback variables themselves are the
//! ones the EFB interface (efb.rs) writes. While X-Plane's tug is attached this
//! module moves nothing. `PUSHBACK STATE` stays sensors.rs's (0 with X-Plane's
//! tug, else 3) and is set to 0 here, after sensors.rs, while FlyByWire's tug
//! is attached, as MSFS reports it with its tug on, so FlyByWire's nose wheel
//! steering (hydraulic/pushback.rs) sees either tug.

use systems::simulation::{SimulatorReaderWriter, VariableIdentifier, VariableRegistry};

use super::{named, XplaneIo};
use crate::published::{self, Published};

/// Pushback_A380X.h:13-15.
const PARKING_BRAKE_FACTOR: f64 = 100.;
const SPEED_FACTOR: f64 = 15.;
const TURN_SPEED_FACTOR: f64 = 0.25;

const FT_TO_M: f64 = 0.3048;

const LOCAL_VX: &str = "sim/flightmodel/position/local_vx";
const LOCAL_VZ: &str = "sim/flightmodel/position/local_vz";
const YAW_RATE: &str = "sim/flightmodel/position/Rrad";
const TRUE_HEADING: &str = "sim/flightmodel/position/psi";
const XPLANE_TUG: &str = "sim/aircraft/overflow/pushback_attached";

/// DampingController.hpp.
#[derive(Clone, Copy, Debug)]
pub struct DampingController {
    last_value: f64,
    accel_step_size: f64,
    epsilon: f64,
}

impl DampingController {
    pub const fn new(start: f64, accel_step_size: f64, epsilon: f64) -> Self {
        Self { last_value: start, accel_step_size, epsilon }
    }

    pub fn update_target_value(&mut self, target: f64) -> f64 {
        if (self.last_value - target).abs() <= self.epsilon {
            return target;
        }
        self.last_value += if target > self.last_value { self.accel_step_size } else { -self.accel_step_size };
        self.last_value
    }
}

/// helper::Math::angleAdd (math_utils.hpp).
fn angle_add(a: f64, b: f64) -> f64 {
    ((a + b) % 360. + 360.) % 360.
}

/// What one frame of FlyByWire's pushback commands, in MSFS's terms.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Command {
    /// VELOCITY BODY Z, ft/s, positive forward.
    pub velocity_body_z: f64,
    /// ROTATION VELOCITY BODY Y, rad/s, positive nose right.
    pub rotation_velocity_body_y: f64,
    /// The tug heading, degrees true.
    pub tug_heading: f64,
    pub tug_commanded_speed: f64,
}

pub struct Pushback {
    speed_dampener: DampingController,
    turn_dampener: DampingController,
    ids: Ids,
    published: Published,
    toggle: published::Command,
    /// Whether the last frame moved the aircraft.
    moving: bool,
}

struct Ids {
    system_enabled: VariableIdentifier,
    park_brake: VariableIdentifier,
    attached: VariableIdentifier,
    pushback_state: VariableIdentifier,
    speed_factor: VariableIdentifier,
    heading_factor: VariableIdentifier,
    aircraft_park_brake_factor: VariableIdentifier,
    aircraft_speed_factor: VariableIdentifier,
    aircraft_turn_speed_factor: VariableIdentifier,
    is_ready: VariableIdentifier,
    on_ground: VariableIdentifier,
    debug: VariableIdentifier,
    debug_delta: VariableIdentifier,
    debug_speed: VariableIdentifier,
    debug_heading: VariableIdentifier,
    debug_inertia_speed: VariableIdentifier,
    debug_rot_x: VariableIdentifier,
}

impl Pushback {
    /// Pushback::initialize (cpp:34-114).
    pub fn new<V: VariableRegistry + SimulatorReaderWriter, X: XplaneIo>(vars: &mut V, _xplane: &mut X) -> Self {
        let ids = Ids {
            system_enabled: named(vars, "PUSHBACK_SYSTEM_ENABLED"),
            park_brake: named(vars, "PARK_BRAKE_LEVER_POS"),
            attached: vars.get("PUSHBACK ATTACHED".to_string()),
            pushback_state: vars.get("PUSHBACK STATE".to_string()),
            speed_factor: named(vars, "PUSHBACK_SPD_FACTOR"),
            heading_factor: named(vars, "PUSHBACK_HDG_FACTOR"),
            aircraft_park_brake_factor: named(vars, "PUSHBACK_AIRCRAFT_PARKBRAKE_FACTOR"),
            aircraft_speed_factor: named(vars, "PUSHBACK_AIRCRAFT_SPEED_FACTOR"),
            aircraft_turn_speed_factor: named(vars, "PUSHBACK_AIRCRAFT_TURN_SPEED_FACTOR"),
            is_ready: named(vars, "IS_READY"),
            on_ground: vars.get("SIM ON GROUND".to_string()),
            debug: named(vars, "PUSHBACK_DEBUG"),
            debug_delta: named(vars, "PUSHBACK_UPDT_DELTA"),
            debug_speed: named(vars, "PUSHBACK_SPD"),
            debug_heading: named(vars, "PUSHBACK_HDG"),
            debug_inertia_speed: named(vars, "PUSHBACK_INERTIA_SPD"),
            debug_rot_x: named(vars, "PUSHBACK_R_X_OUT"),
        };
        vars.write(&ids.aircraft_park_brake_factor, PARKING_BRAKE_FACTOR);
        vars.write(&ids.aircraft_speed_factor, SPEED_FACTOR);
        vars.write(&ids.aircraft_turn_speed_factor, TURN_SPEED_FACTOR);
        let mut published = Published::default();
        let toggle = published.command("fbw/pushback/toggle", "Attach or detach FlyByWire's pushback tug (K:TOGGLE_PUSHBACK)");
        Self {
            // Pushback.h:43-44.
            speed_dampener: DampingController::new(0., 0.15, 0.1),
            turn_dampener: DampingController::new(0., 0.01, 0.001),
            ids,
            published,
            toggle,
            moving: false,
        }
    }

    /// `K:TOGGLE_PUSHBACK`.
    pub fn handle_event(&mut self, name: &str) -> bool {
        if name == "TOGGLE_PUSHBACK" {
            published::press(self.toggle);
            return true;
        }
        false
    }

    /// Before the systems: the tug toggles, and MSFS's pushback state while
    /// the tug is on.
    pub fn write_pushback_state<V: SimulatorReaderWriter>(&mut self, vars: &mut V) {
        for _ in 0..published::presses(self.toggle) {
            let attached = vars.read(&self.ids.attached) != 0.;
            vars.write(&self.ids.attached, if attached { 0. } else { 1. });
        }
        if vars.read(&self.ids.attached) != 0. {
            vars.write(&self.ids.pushback_state, 0.);
        }
    }

    /// Pushback::update (cpp:121-213) up to the velocities it writes.
    pub fn command<V: SimulatorReaderWriter>(&mut self, vars: &mut V, aircraft_heading: f64) -> Option<Command> {
        let r = |vars: &mut V, id: &VariableIdentifier| vars.read(id);
        if r(vars, &self.ids.is_ready) == 0.
            || r(vars, &self.ids.system_enabled) == 0.
            || r(vars, &self.ids.attached) == 0.
            || r(vars, &self.ids.on_ground) == 0.
        {
            return None;
        }
        let parking_brake = r(vars, &self.ids.park_brake) != 0.;
        let speed_factor = r(vars, &self.ids.aircraft_speed_factor);
        let turn_factor = r(vars, &self.ids.aircraft_turn_speed_factor);
        let brake_factor = r(vars, &self.ids.aircraft_park_brake_factor);
        let commanded_heading_factor = r(vars, &self.ids.heading_factor);

        let speed = if parking_brake { speed_factor / brake_factor } else { speed_factor };
        let tug_commanded_speed = r(vars, &self.ids.speed_factor) * speed;
        let inertia_speed = self.speed_dampener.update_target_value(tug_commanded_speed);

        let turn = if parking_brake { turn_factor / brake_factor } else { turn_factor };
        let rotation = self.turn_dampener.update_target_value((inertia_speed / speed_factor) * commanded_heading_factor * turn);
        let tug_heading = angle_add(aircraft_heading, commanded_heading_factor * -90.);
        Some(Command { velocity_body_z: inertia_speed, rotation_velocity_body_y: rotation, tug_heading, tug_commanded_speed })
    }

    pub fn update<V: SimulatorReaderWriter, X: XplaneIo>(&mut self, vars: &mut V, xplane: &mut X, delta: f64) {
        // X-Plane's own tug drives the aircraft itself; never both.
        if xplane.get(XPLANE_TUG, None).unwrap_or(0.) != 0. {
            self.moving = false;
            return;
        }
        let heading = xplane.get(TRUE_HEADING, None).unwrap_or(0.);
        let Some(c) = self.command(vars, heading) else {
            self.moving = false;
            return;
        };
        let v = c.velocity_body_z * FT_TO_M;
        let psi = heading.to_radians();
        xplane.set(LOCAL_VX, None, v * psi.sin());
        xplane.set(LOCAL_VZ, None, -v * psi.cos());
        xplane.set(YAW_RATE, None, c.rotation_velocity_body_y);
        self.moving = true;
        if vars.read(&self.ids.debug) != 0. {
            vars.write(&self.ids.debug_delta, delta);
            vars.write(&self.ids.debug_speed, c.tug_commanded_speed);
            vars.write(&self.ids.debug_heading, c.tug_heading);
            vars.write(&self.ids.debug_inertia_speed, c.velocity_body_z);
            // The counter-rotation is not applied here (see above).
            vars.write(&self.ids.debug_rot_x, 0.);
        }
    }

    /// Nothing of X-Plane's is overridden; the commands go with `published`.
    pub fn release<X: XplaneIo>(&mut self, _xplane: &mut X) {
        self.moving = false;
        let _ = &self.published;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::aspects::test_vars::TestVars;
    use crate::extra_backend::sim::test_xplane::FakeXplane;

    fn ready(vars: &mut TestVars) {
        vars.set("A32NX_IS_READY", 1.);
        vars.set("SIM ON GROUND", 1.);
        vars.set("A32NX_PUSHBACK_SYSTEM_ENABLED", 1.);
        vars.set("PUSHBACK ATTACHED", 1.);
    }

    #[test]
    fn full_reverse_ramps_to_fifteen_feet_per_second_backwards_along_the_heading() {
        let mut vars = TestVars::default();
        let mut xp = FakeXplane::default();
        let mut p = Pushback::new(&mut vars, &mut xp);
        ready(&mut vars);
        vars.set("A32NX_PUSHBACK_SPD_FACTOR", -1.);
        xp.values.insert(TRUE_HEADING.into(), 90.);
        p.update(&mut vars, &mut xp, 1. / 30.);
        // One 0.15 ft/s step per frame (Pushback.h:43).
        assert!((xp.values[LOCAL_VX] + 0.15 * FT_TO_M).abs() < 1e-9);
        for _ in 0..200 {
            p.update(&mut vars, &mut xp, 1. / 30.);
        }
        // Heading east, backwards: westward, 15 ft/s.
        assert!((xp.values[LOCAL_VX] + 15. * FT_TO_M).abs() < 1e-6, "{}", xp.values[LOCAL_VX]);
        assert!(xp.values[LOCAL_VZ].abs() < 1e-6);
        assert_eq!(xp.values[YAW_RATE], 0.);
    }

    #[test]
    fn steering_turns_in_proportion_to_speed_and_the_park_brake_slows_everything() {
        let mut vars = TestVars::default();
        let mut xp = FakeXplane::default();
        let mut p = Pushback::new(&mut vars, &mut xp);
        ready(&mut vars);
        vars.set("A32NX_PUSHBACK_SPD_FACTOR", -1.);
        vars.set("A32NX_PUSHBACK_HDG_FACTOR", 1.);
        let mut c = None;
        for _ in 0..300 {
            c = p.command(&mut vars, 0.);
        }
        let c = c.unwrap();
        // (inertia / 15) * heading factor * 0.25 = -0.25 rad/s, reached in 0.01 steps.
        assert!((c.rotation_velocity_body_y + 0.25).abs() <= 0.001, "{c:?}");
        assert_eq!(c.tug_heading, 270.);
        vars.set("A32NX_PARK_BRAKE_LEVER_POS", 1.);
        let mut c = None;
        for _ in 0..300 {
            c = p.command(&mut vars, 0.);
        }
        assert!((c.unwrap().velocity_body_z + 0.15).abs() <= 0.1);
    }

    #[test]
    fn nothing_moves_without_the_tug_or_in_the_air() {
        let mut vars = TestVars::default();
        let mut xp = FakeXplane::default();
        let mut p = Pushback::new(&mut vars, &mut xp);
        ready(&mut vars);
        vars.set("PUSHBACK ATTACHED", 0.);
        vars.set("A32NX_PUSHBACK_SPD_FACTOR", 1.);
        p.update(&mut vars, &mut xp, 0.1);
        assert!(!xp.values.contains_key(LOCAL_VX));
        vars.set("PUSHBACK ATTACHED", 1.);
        vars.set("SIM ON GROUND", 0.);
        p.update(&mut vars, &mut xp, 0.1);
        assert!(!xp.values.contains_key(LOCAL_VX));
    }

    #[test]
    fn toggle_pushback_attaches_the_tug_and_msfs_state_follows() {
        let mut vars = TestVars::default();
        let mut xp = FakeXplane::default();
        let mut p = Pushback::new(&mut vars, &mut xp);
        vars.set("PUSHBACK STATE", 3.);
        assert!(p.handle_event("TOGGLE_PUSHBACK"));
        p.write_pushback_state(&mut vars);
        assert_eq!(vars.value("PUSHBACK ATTACHED"), 1.);
        assert_eq!(vars.value("PUSHBACK STATE"), 0.);
        p.handle_event("TOGGLE_PUSHBACK");
        p.write_pushback_state(&mut vars);
        assert_eq!(vars.value("PUSHBACK ATTACHED"), 0.);
    }
}
