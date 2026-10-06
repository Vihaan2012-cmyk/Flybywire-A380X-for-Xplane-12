//! Gear, brakes, autobrake, flaps, slats and wheel steering: FlyByWire's
//! inputs taken from X-Plane's commands and axes, and FlyByWire's results
//! given to X-Plane's flight model.
//!
//! In MSFS, FlyByWire masks the sim's own key events for these controls and
//! turns them into its variables (`a380_systems_wasm`: gear.rs, brakes.rs,
//! autobrakes.rs, flaps.rs, nose_wheel_steering.rs, body_wheel_steering.rs);
//! after its systems run it writes gear, flap and slat positions back to the
//! sim and sends the brake force and steering angle as axis events. Here:
//!
//! * X-Plane's stock commands for these controls are taken before X-Plane sees
//!   them and consumed (the `mask()` of every one of FlyByWire's events), and
//!   become the MSFS event FlyByWire handles; every event without data is also
//!   an `fbw/event/<name>` command. Hardware axes (toe brakes, flaps, tiller)
//!   are read from X-Plane's processed joystick axes and sent as the matching
//!   axis event when they move, as MSFS sends axis events.
//! * FlyByWire's aspects are ported in [`aspects`]: same variables, mappings,
//!   smooth press, debounce and reset.
//! * The flight model gets gear deployment, toe brake ratios, flap and slat
//!   ratios and tyre steering angles from FlyByWire each tick ([`physics`]),
//!   with X-Plane's own gear, toe brake and steering logic overridden.

mod aspects;
mod physics;

use std::ffi::{c_int, c_void};
use std::sync::{Mutex, OnceLock};
use std::time::Instant;

use systems::simulation::{SimulatorReaderWriter, VariableIdentifier, VariableRegistry};

use crate::xp::{CommandRef, DataRef, Xplm};
use crate::Vars;

use physics::{LeverAction, LeverBridge};

/// How an X-Plane command becomes FlyByWire's event.
#[derive(Clone, Copy, PartialEq)]
enum Kind {
    /// One event when pressed.
    Press,
    /// An event every tick while held, as a held MSFS key repeats.
    Hold,
    /// Each press latches or unlatches a held event.
    Toggle,
    /// Consumed with nothing sent: FlyByWire has no such control, and
    /// X-Plane must not act on it either.
    Swallow,
}

/// A manual trim switch's one-tick pulse variable (`prim.rs`'s
/// `TrimPulses::NAMES`), written directly rather than as an event.
fn is_trim_pulse(event: &str) -> bool {
    event.starts_with("XP_")
        && ["_TRIM_UP_PULSE", "_TRIM_DOWN_PULSE", "_TRIM_LEFT_PULSE", "_TRIM_RIGHT_PULSE", "_TRIM_RESET_PULSE"].iter().any(|s| event.ends_with(s))
}

/// X-Plane's stock commands and the MSFS event each stands for.
const STOCK: &[(&str, &str, Kind)] = &[
    ("sim/flight_controls/landing_gear_up", "GEAR_UP", Kind::Press),
    ("sim/flight_controls/landing_gear_down", "GEAR_DOWN", Kind::Press),
    ("sim/flight_controls/landing_gear_toggle", "GEAR_TOGGLE", Kind::Press),
    ("sim/flight_controls/landing_gear_off", "", Kind::Swallow),
    // The manual trim switches. FlyByWire takes ELEV_TRIM_UP/DN and the
    // rudder trim events in its C++ interface, not as mapped events, as the
    // PRIM/SEC discretes `prim.rs`'s `TrimPulses` reads; these X-Plane
    // commands used to move only X-Plane's own trim, which the plugin
    // overwrites from the THS every frame, so a pilot's trim input did
    // nothing at all. Held, one pulse a tick, as a held switch.
    ("sim/flight_controls/pitch_trim_up", "XP_PITCH_TRIM_UP_PULSE", Kind::Hold),
    ("sim/flight_controls/pitch_trim_down", "XP_PITCH_TRIM_DOWN_PULSE", Kind::Hold),
    ("sim/flight_controls/rudder_trim_left", "XP_RUDDER_TRIM_LEFT_PULSE", Kind::Hold),
    ("sim/flight_controls/rudder_trim_right", "XP_RUDDER_TRIM_RIGHT_PULSE", Kind::Hold),
    ("sim/flight_controls/rudder_trim_center", "XP_RUDDER_TRIM_RESET_PULSE", Kind::Press),
    ("sim/flight_controls/flaps_up", "FLAPS_DECR", Kind::Press),
    ("sim/flight_controls/flaps_down", "FLAPS_INCR", Kind::Press),
    ("sim/flight_controls/flaps_up_full", "FLAPS_UP", Kind::Press),
    ("sim/flight_controls/flaps_down_full", "FLAPS_DOWN", Kind::Press),
    ("sim/flight_controls/flaps_detent_1", "FLAPS_1", Kind::Press),
    ("sim/flight_controls/flaps_detent_2", "FLAPS_2", Kind::Press),
    ("sim/flight_controls/flaps_detent_3", "FLAPS_3", Kind::Press),
    ("sim/flight_controls/flaps_detent_4", "FLAPS_DOWN", Kind::Press),
    ("sim/flight_controls/flaps_detent_5", "", Kind::Swallow),
    ("sim/flight_controls/flaps_detent_6", "", Kind::Swallow),
    ("sim/flight_controls/flaps_detent_7", "", Kind::Swallow),
    ("sim/flight_controls/flaps_detent_8", "", Kind::Swallow),
    ("sim/flight_controls/brakes_regular", "BRAKES", Kind::Hold),
    ("sim/flight_controls/brakes_max", "BRAKES", Kind::Hold),
    ("sim/flight_controls/brakes_toggle_regular", "BRAKES", Kind::Toggle),
    ("sim/flight_controls/brakes_toggle_max", "BRAKES", Kind::Toggle),
    ("sim/flight_controls/left_brake", "BRAKES_LEFT", Kind::Hold),
    ("sim/flight_controls/right_brake", "BRAKES_RIGHT", Kind::Hold),
    ("sim/flight_controls/park_brake_toggle", "PARKING_BRAKES", Kind::Press),
    ("sim/flight_controls/park_brake_set", "PARKING_BRAKES_ON", Kind::Press),
    ("sim/flight_controls/park_brake_release", "PARKING_BRAKES_OFF", Kind::Press),
    ("sim/flight_controls/brakes_off_auto", "AUTOBRAKE_DISARM", Kind::Press),
    ("sim/flight_controls/brakes_1_auto", "AUTOBRAKE_LO_SET", Kind::Press),
    ("sim/flight_controls/brakes_2_auto", "A32NX.AUTOBRAKE_SET_L2", Kind::Press),
    ("sim/flight_controls/brakes_3_auto", "AUTOBRAKE_MED_SET", Kind::Press),
    ("sim/flight_controls/brakes_max_auto", "A32NX.AUTOBRAKE_SET_HI", Kind::Press),
    ("sim/flight_controls/brakes_rto_auto", "AUTOBRAKE_HI_SET", Kind::Press),
    ("sim/flight_controls/brakes_toggle_auto", "", Kind::Swallow),
    ("sim/flight_controls/brakes_dn_auto", "", Kind::Swallow),
    ("sim/flight_controls/brakes_up_auto", "", Kind::Swallow),
    // FlyByWire's PEDALS DISC binding (nose_wheel_steering.rs:81-88).
    ("sim/flight_controls/water_rudder_toggle", "TOGGLE_WATER_RUDDER", Kind::Hold),
];

/// FlyByWire's events without data, each also an `fbw/event/` command.
const FBW_EVENTS: &[(&str, Kind)] = &[
    ("GEAR_UP", Kind::Press),
    ("GEAR_DOWN", Kind::Press),
    ("GEAR_TOGGLE", Kind::Press),
    ("PARKING_BRAKES", Kind::Press),
    ("PARKING_BRAKES_ON", Kind::Press),
    ("PARKING_BRAKES_OFF", Kind::Press),
    ("BRAKES", Kind::Hold),
    ("BRAKES_LEFT", Kind::Hold),
    ("BRAKES_RIGHT", Kind::Hold),
    ("AUTOBRAKE_LO_SET", Kind::Press),
    ("AUTOBRAKE_MED_SET", Kind::Press),
    ("AUTOBRAKE_HI_SET", Kind::Press),
    ("A32NX.AUTO_THROTTLE_DISCONNECT", Kind::Press),
    ("A32NX.AUTOBRAKE_SET_DISARM", Kind::Press),
    ("AUTOBRAKE_DISARM", Kind::Press),
    ("A32NX.AUTOBRAKE_SET_BTV", Kind::Press),
    ("A32NX.AUTOBRAKE_SET_LO", Kind::Press),
    ("A32NX.AUTOBRAKE_SET_L2", Kind::Press),
    ("A32NX.AUTOBRAKE_SET_L3", Kind::Press),
    ("A32NX.AUTOBRAKE_SET_HI", Kind::Press),
    ("FLAPS_INCR", Kind::Press),
    ("FLAPS_DECR", Kind::Press),
    ("FLAPS_UP", Kind::Press),
    ("FLAPS_1", Kind::Press),
    ("FLAPS_2", Kind::Press),
    ("FLAPS_3", Kind::Press),
    ("FLAPS_DOWN", Kind::Press),
    ("STEERING_INC", Kind::Press),
    ("STEERING_DEC", Kind::Press),
    ("TOGGLE_WATER_RUDDER", Kind::Hold),
];

/// One registered command handler.
struct Registered {
    command: CommandRef,
}

/// What the command handlers leave for the next tick.
#[derive(Default)]
struct Pending {
    events: Vec<(&'static str, f64)>,
    held: Vec<bool>,
    latched: Vec<bool>,
    table: Vec<(&'static str, Kind, bool)>,
}

static PENDING: Mutex<Option<Pending>> = Mutex::new(None);

/// Seconds on a monotonic clock, where FlyByWire's debounce uses `Instant`.
fn now() -> f64 {
    static START: OnceLock<Instant> = OnceLock::new();
    START.get_or_init(Instant::now).elapsed().as_secs_f64()
}

unsafe extern "C" fn on_command(_command: CommandRef, phase: c_int, refcon: *mut c_void) -> c_int {
    let i = refcon as usize;
    let Ok(mut guard) = PENDING.lock() else { return 1 };
    let Some(p) = guard.as_mut() else { return 1 };
    let Some(&(event, kind, consume)) = p.table.get(i) else { return 1 };
    match (kind, phase) {
        (Kind::Press, 0) => p.events.push((event, now())),
        (Kind::Hold, 0) => {
            p.held[i] = true;
            p.events.push((event, now()));
        }
        (Kind::Hold, 2) => p.held[i] = false,
        (Kind::Toggle, 0) => p.latched[i] = !p.latched[i],
        _ => {}
    }
    // 0 stops X-Plane (and later handlers) acting on it: FlyByWire's mask().
    if consume {
        0
    } else {
        1
    }
}

/// X-Plane's joystick axis functions, as numbered by
/// `sim/joystick/joystick_axis_assignments` and indexing
/// `sim/joystick/joy_mapped_axis_value` (FlyWithLua's get_axis_assignments.lua
/// names them in this order from 1; this X-Plane's own joystick preferences
/// agree: axes set to Left toe brake, Right toe brake, Flaps, Throttle 1 hold
/// 6, 7, 11 and 20).
/// Yaw (3): read here rather than via `sim/joystick/yoke_heading_ratio`
/// (which `inputs()` and `lib.rs`'s `PrimRefs::read` used to read instead).
/// X-Plane's own DataRefs.txt documents `override_joystick_heading` as
/// uniquely "disabl[ing] auto-coordination" (`override_joystick_pitch`/
/// `_roll` say nothing of the kind), meaning that without real rudder-pedal
/// hardware `yoke_heading_ratio` silently carries X-Plane's own
/// turn-coordination synthesis, not the pilot's pedal -- and FlyByWire's own
/// yaw damper (prim.rs) then doubles that on top (W84/W99). This constant
/// indexes `joy_mapped_axis_avail`/`_value` instead, the same raw-hardware
/// arrays the other axes below already use.
const AXIS_YAW: usize = 3;
const AXIS_LEFT_TOE: usize = 6;
const AXIS_RIGHT_TOE: usize = 7;
const AXIS_FLAPS: usize = 11;
const AXIS_MIXTURE_4: usize = 31;
const AXIS_TILLER: usize = 37;
const AXIS_COPILOT_LEFT_TOE: usize = 70;
const AXIS_COPILOT_RIGHT_TOE: usize = 71;
const AXES: usize = 81;

struct Refs {
    joy_avail: Option<DataRef>,
    joy_value: Option<DataRef>,

    override_gearbrake: Option<DataRef>,
    override_toe_brakes: Option<DataRef>,
    override_wheel_steer: Option<DataRef>,
    override_control_surfaces: Option<DataRef>,

    gear_deploy: Option<DataRef>,
    gear_handle_down: Option<DataRef>,
    left_brake: Option<DataRef>,
    right_brake: Option<DataRef>,
    parkbrake: Option<DataRef>,
    auto_brake_level: Option<DataRef>,
    flaprqst: Option<DataRef>,
    flap_handle_request: Option<DataRef>,
    flaprat: Option<DataRef>,
    flap2rat: Option<DataRef>,
    flap1_deploy: Option<DataRef>,
    flap2_deploy: Option<DataRef>,
    slatrat: Option<DataRef>,
    slat1_deploy: Option<DataRef>,
    slat2_deploy: Option<DataRef>,
    flap1_deg: Option<DataRef>,
    flap2_deg: Option<DataRef>,
    steer_command: Option<DataRef>,
    steer_actual: Option<DataRef>,
    flap_detents: Option<DataRef>,
}

impl Refs {
    fn new(xplm: &Xplm) -> Self {
        let f = |n: &str| xplm.find(n);
        Self {
            joy_avail: f("sim/joystick/joy_mapped_axis_avail"),
            joy_value: f("sim/joystick/joy_mapped_axis_value"),
            override_gearbrake: f("sim/operation/override/override_gearbrake"),
            override_toe_brakes: f("sim/operation/override/override_toe_brakes"),
            override_wheel_steer: f("sim/operation/override/override_wheel_steer"),
            override_control_surfaces: f("sim/operation/override/override_control_surfaces"),
            gear_deploy: f("sim/aircraft/parts/acf_gear_deploy"),
            gear_handle_down: f("sim/cockpit2/controls/gear_handle_down"),
            left_brake: f("sim/cockpit2/controls/left_brake_ratio"),
            right_brake: f("sim/cockpit2/controls/right_brake_ratio"),
            parkbrake: f("sim/flightmodel/controls/parkbrake"),
            auto_brake_level: f("sim/cockpit2/switches/auto_brake_level"),
            flaprqst: f("sim/flightmodel/controls/flaprqst"),
            flap_handle_request: f("sim/cockpit2/controls/flap_handle_request_ratio"),
            flaprat: f("sim/flightmodel/controls/flaprat"),
            flap2rat: f("sim/flightmodel/controls/flap2rat"),
            flap1_deploy: f("sim/flightmodel2/controls/flap1_deploy_ratio"),
            flap2_deploy: f("sim/flightmodel2/controls/flap2_deploy_ratio"),
            slatrat: f("sim/flightmodel/controls/slatrat"),
            slat1_deploy: f("sim/flightmodel2/controls/slat1_deploy_ratio"),
            slat2_deploy: f("sim/flightmodel2/controls/slat2_deploy_ratio"),
            flap1_deg: f("sim/flightmodel2/wing/flap1_deg"),
            flap2_deg: f("sim/flightmodel2/wing/flap2_deg"),
            steer_command: f("sim/flightmodel2/gear/tire_steer_command_deg"),
            steer_actual: f("sim/flightmodel2/gear/tire_steer_actual_deg"),
            flap_detents: f("sim/aircraft/controls/acf_flap_detents"),
        }
    }

    fn overrides(&self) -> [Option<DataRef>; 3] {
        [self.override_gearbrake, self.override_toe_brakes, self.override_wheel_steer]
    }
}

/// The converter's cockpit levers, published by SASL (possibly after this
/// plugin starts, so they are looked for again until found).
struct Lever {
    name: &'static str,
    dataref: Option<DataRef>,
    bridge: LeverBridge,
}

impl Lever {
    fn new(name: &'static str) -> Self {
        Self { name, dataref: None, bridge: LeverBridge::default() }
    }
}

/// The FlyByWire outputs the flight model follows.
struct OutIds {
    gear_center: VariableIdentifier,
    gear_left: VariableIdentifier,
    gear_right: VariableIdentifier,
    door_center: VariableIdentifier,
    door_left: VariableIdentifier,
    door_right: VariableIdentifier,
    gear_handle_position: VariableIdentifier,
    brake_left_force: VariableIdentifier,
    brake_right_force: VariableIdentifier,
    left_flaps_angle: VariableIdentifier,
    right_flaps_angle: VariableIdentifier,
    left_slats_angle: VariableIdentifier,
    right_slats_angle: VariableIdentifier,
    nose_ratio: VariableIdentifier,
    left_body_ratio: VariableIdentifier,
    right_body_ratio: VariableIdentifier,
    flaps_handle_index: VariableIdentifier,
    park_brake_lever_pos: VariableIdentifier,
    rudder_pedal_position: VariableIdentifier,
}

pub struct Handling {
    aspects: aspects::Aspects,
    ids: OutIds,
    refs: Refs,
    registered: Vec<Registered>,
    /// Last reading of each hardware axis, once it is available: left toe
    /// brake, right toe brake, flaps, tiller, yaw (rudder pedals, W99).
    axes: [Option<f64>; 5],
    flaps_lever: Lever,
    gear_lever: Lever,
    park_lever: Lever,
    ticks: u64,
    /// Lowest gear deployment this run has handed X-Plane, so the log says
    /// it once per new low rather than every tick.
    gear_deploy_floor: f64,
    /// Whether FlyByWire's landing gear has ever published a position, so
    /// its silent initial zero is never mistaken for "gear up".
    gear_reported: bool,
}

unsafe impl Send for Handling {}

impl Handling {
    pub fn new(vars: &mut Vars, xplm: &Xplm) -> Self {
        let aspects = aspects::Aspects::new(vars);
        let mut get = |n: &str| vars.get(n.to_owned());
        let ids = OutIds {
            // hydraulic/landing_gear.rs:67-76 (systems)
            gear_center: get("GEAR_CENTER_POSITION"),
            gear_left: get("GEAR_LEFT_POSITION"),
            gear_right: get("GEAR_RIGHT_POSITION"),
            door_center: get("GEAR_DOOR_CENTER_POSITION"),
            door_left: get("GEAR_DOOR_LEFT_POSITION"),
            door_right: get("GEAR_DOOR_RIGHT_POSITION"),
            // landing_gear/mod.rs:772
            gear_handle_position: get("GEAR_HANDLE_POSITION"),
            // a380 hydraulic/mod.rs:4700-4704, written at 4797-4801
            brake_left_force: get("BRAKE LEFT FORCE FACTOR"),
            brake_right_force: get("BRAKE RIGHT FORCE FACTOR"),
            // flap_slat.rs:96-97, 134
            left_flaps_angle: get("LEFT_FLAPS_ANGLE"),
            right_flaps_angle: get("RIGHT_FLAPS_ANGLE"),
            left_slats_angle: get("LEFT_SLATS_ANGLE"),
            right_slats_angle: get("RIGHT_SLATS_ANGLE"),
            // nose_steering.rs:140, 298
            nose_ratio: get("NOSE_WHEEL_POSITION_RATIO"),
            left_body_ratio: get("LEFT_BODY_STEERING_POSITION_RATIO"),
            right_body_ratio: get("RIGHT_BODY_STEERING_POSITION_RATIO"),
            flaps_handle_index: get("FLAPS_HANDLE_INDEX"),
            park_brake_lever_pos: get("PARK_BRAKE_LEVER_POS"),
            rudder_pedal_position: get("RUDDER_PEDAL_POSITION"),
        };
        let refs = Refs::new(xplm);
        if let Some(d) = refs.flap_detents {
            let n = xplm.get_i(d);
            if n as usize != physics::ACF_FLAP_DETENT_DEG.len() - 1 {
                crate::log(&format!(
                    "handling: the aircraft has {n} flap detents; the flap mapping expects the converted A380X's 5"
                ));
            }
        }
        let mut handling = Self {
            aspects,
            ids,
            refs,
            registered: Vec::new(),
            axes: [None; 5],
            flaps_lever: Lever::new("fbw/cockpit/flaps_lever"),
            gear_lever: Lever::new("fbw/cockpit/lever_landing_gear"),
            park_lever: Lever::new("fbw/cockpit/lever_parking_brake"),
            ticks: 0,
            gear_deploy_floor: f64::INFINITY,
            gear_reported: false,
        };
        handling.register(xplm);
        handling
    }

    fn register(&mut self, xplm: &Xplm) {
        let mut table: Vec<(&'static str, Kind, bool)> = Vec::new();
        for &(name, event, kind) in STOCK {
            debug_assert!(kind == Kind::Swallow || is_trim_pulse(event) || self.aspects.handles(event));
            if let Some(command) = xplm.create_command(name, name) {
                table.push((event, kind, true));
                self.registered.push(Registered { command });
            }
        }
        for &(event, kind) in FBW_EVENTS {
            let name = format!("fbw/event/{}", event.replace('.', "_"));
            if let Some(command) = xplm.create_command(&name, &format!("FlyByWire event {event}")) {
                table.push((event, kind, false));
                self.registered.push(Registered { command });
            }
        }
        if let Ok(mut p) = PENDING.lock() {
            *p = Some(Pending {
                events: Vec::new(),
                held: vec![false; table.len()],
                latched: vec![false; table.len()],
                table,
            });
        }
        for (i, r) in self.registered.iter().enumerate() {
            xplm.register_command_handler(r.command, on_command, i as *mut c_void);
        }
    }

    /// Commands, axes and cockpit levers since the last tick: FlyByWire's
    /// event handling, which in MSFS happens between gauge updates.
    pub fn inputs(&mut self, vars: &mut Vars, xplm: &Xplm) {
        self.ticks += 1;
        let events = match PENDING.lock() {
            Ok(mut guard) => match guard.as_mut() {
                Some(p) => {
                    let mut events = std::mem::take(&mut p.events);
                    let t = now();
                    for (i, &(event, _, _)) in p.table.iter().enumerate() {
                        if p.held[i] || p.latched[i] {
                            events.push((event, t));
                        }
                    }
                    events
                }
                None => Vec::new(),
            },
            Err(_) => Vec::new(),
        };
        for (event, at) in events {
            if is_trim_pulse(event) {
                let id = vars.get(event.to_owned());
                vars.write(&id, 1.);
            } else {
                self.aspects.handle(vars, event, 0, at);
            }
        }

        self.read_axes(vars, xplm);

        // FlyByWireInterface.cpp:2896: the rudder pedals in percent, from the
        // sim's rudder input (inputs[2], which prim.rs takes as the negated
        // yoke heading ratio) -- here, `self.axes[4]` (AXIS_YAW), the raw
        // pedal axis `read_axes()` just refreshed above, NOT
        // `sim/joystick/yoke_heading_ratio` (see AXIS_YAW's doc comment for
        // why). With no rudder-pedal hardware `self.axes[4]` is `None`, read
        // as centred, matching MSFS never sending an AXIS_RUDDER_SET without
        // a real rudder-pedal peripheral.
        let input_2 = -self.axes[4].unwrap_or(0.);
        vars.write(&self.ids.rudder_pedal_position, (-100. * input_2).clamp(-100., 100.));

        self.cockpit_levers(vars, xplm);
    }

    fn read_axes(&mut self, vars: &mut Vars, xplm: &Xplm) {
        let (Some(avail_ref), Some(value_ref)) = (self.refs.joy_avail, self.refs.joy_value) else { return };
        let mut avail = [0 as c_int; AXES];
        let mut value = [0f32; AXES];
        xplm.get_vi(avail_ref, &mut avail);
        xplm.get_vf(value_ref, &mut value);
        let axis = |i: usize| crate::hardware_axis(avail[i], value[i]);
        let either = |a: Option<f64>, b: Option<f64>| match (a, b) {
            (Some(a), Some(b)) => Some(a.max(b)),
            (a, b) => a.or(b),
        };
        let readings = [
            either(axis(AXIS_LEFT_TOE), axis(AXIS_COPILOT_LEFT_TOE)),
            either(axis(AXIS_RIGHT_TOE), axis(AXIS_COPILOT_RIGHT_TOE)),
            axis(AXIS_FLAPS),
            axis(AXIS_TILLER),
            axis(AXIS_YAW),
        ];
        let t = now();
        for (i, reading) in readings.into_iter().enumerate() {
            let Some(v) = reading else {
                self.axes[i] = None;
                continue;
            };
            // Only movement sends an axis event, as in MSFS; the first
            // reading is where the axis rests.
            let moved = self.axes[i].is_some_and(|last| last != v);
            self.axes[i] = Some(v);
            if !moved {
                continue;
            }
            let (event, data) = match i {
                0 => ("AXIS_LEFT_BRAKE_SET", aspects::f64_to_pos_32k(v)),
                1 => ("AXIS_RIGHT_BRAKE_SET", aspects::f64_to_pos_32k(v)),
                2 => ("AXIS_FLAPS_SET", aspects::f64_to_pos_32k(v)),
                // X-Plane's tiller is -1 (left) to 1; MSFS's steering axis
                // is inverted (nose_wheel_steering.rs:50-55).
                3 => ("AXIS_STEERING_SET", aspects::f64_to_pos_32k((1. - v) / 2.)),
                // Yaw (4): no MSFS axis event exists for this one; `inputs()`
                // reads `self.axes[4]` directly, every tick, below (W99).
                _ => continue,
            };
            self.aspects.handle(vars, event, data, t);
        }
        // AXIS_MIXTURE4_SET (nose_wheel_steering.rs:42-48) is FlyByWire's
        // legacy tiller binding; X-Plane's Mixture 4 axis is left to X-Plane.
        let _ = AXIS_MIXTURE_4;
    }

    fn cockpit_levers(&mut self, vars: &mut Vars, xplm: &Xplm) {
        if self.ticks % 100 == 1 {
            for lever in [&mut self.flaps_lever, &mut self.gear_lever, &mut self.park_lever] {
                if lever.dataref.is_none() {
                    lever.dataref = xplm.find(lever.name);
                }
            }
        }
        let t = now();
        // The flap lever's detents are at 0, .25, .5, .75 and 1 of its travel
        // for handle index 0..4 (pedestal.xml:200-208).
        if let Some(d) = self.flaps_lever.dataref {
            let index = vars.read(&self.ids.flaps_handle_index);
            match self.flaps_lever.bridge.update(xplm.get_f(d) as f64, index / 4.) {
                LeverAction::Moved(v) => vars.write(&self.ids.flaps_handle_index, (v * 4.).round().clamp(0., 4.)),
                LeverAction::Show(v) => xplm.set_f(d, v as f32),
                LeverAction::None => {}
            }
        }
        // The gear lever animates with GEAR_HANDLE_POSITION and sends
        // GEAR_UP / GEAR_DOWN (gear.xml:17-21, 36-40).
        if let Some(d) = self.gear_lever.dataref {
            let handle = vars.read(&self.ids.gear_handle_position);
            match self.gear_lever.bridge.update(xplm.get_f(d) as f64, handle) {
                LeverAction::Moved(v) => {
                    self.aspects.handle(vars, if v >= 0.5 { "GEAR_DOWN" } else { "GEAR_UP" }, 0, t);
                }
                LeverAction::Show(v) => xplm.set_f(d, v as f32),
                LeverAction::None => {}
            }
        }
        // The parking brake lever animates with PARK_BRAKE_LEVER_POS
        // (A32NX_Interior_Handling.xml:185-188).
        if let Some(d) = self.park_lever.dataref {
            let pos = vars.read(&self.ids.park_brake_lever_pos);
            match self.park_lever.bridge.update(xplm.get_f(d) as f64, pos) {
                LeverAction::Moved(v) => {
                    let event = if v >= 0.5 { "PARKING_BRAKES_ON" } else { "PARKING_BRAKES_OFF" };
                    self.aspects.handle(vars, event, 0, t);
                }
                LeverAction::Show(v) => xplm.set_f(d, v as f32),
                LeverAction::None => {}
            }
        }
    }

    /// The aspects' pre-tick work, just before the systems run.
    pub fn before_systems(&mut self, vars: &mut Vars, delta: f64) {
        self.aspects.pre_tick(vars, delta);
    }

    /// The aspects' post-tick work and FlyByWire's results to the flight model.
    pub fn after_systems(&mut self, vars: &mut Vars, xplm: &Xplm) {
        self.aspects.post_tick(vars, now());
        let r = &self.refs;
        let ids = &self.ids;
        let mut read = |id: &VariableIdentifier| vars.read(id);

        for d in r.overrides().into_iter().flatten() {
            xplm.set_i(d, 1);
        }

        // Gear (gear.rs:49-55, 91-111). A walkaround downlock pin acts in
        // FlyByWire's own gear model (`walkaround.rs`: GearActuatorJammed),
        // so its published positions already hold a pinned leg down.
        let (deploy, handle_down) = physics::gear_deploy(
            read(&ids.gear_center),
            read(&ids.gear_left),
            read(&ids.gear_right),
            read(&ids.door_center),
            read(&ids.door_left),
            read(&ids.door_right),
        );
        if let Some(d) = r.gear_deploy {
            // Telling X-Plane's flight model the gear is anything less than
            // down, while the aircraft is sitting on it, retracts the legs
            // under its own weight and X-Plane calls that a crash. The three
            // positions come from FlyByWire's hydraulic landing gear
            // (`GEAR_{CENTER,LEFT,RIGHT}_POSITION`, percent), so a tick
            // where they read low for any reason is worth seeing: the
            // converted A380 crashes 35-60 s after every load with this
            // plugin and never without it, and every other path has been
            // ruled out (the failure mirroring, the deep gear collapse, plug
            // forces, weight and balance).
            // FlyByWire's gear positions are percentages its hydraulic
            // landing gear publishes. Before that system has run they are
            // all still zero, and zero here does not mean "gear up" -- it
            // means "nobody has said yet". Handing that to X-Plane retracts
            // the legs in the flight model while the aircraft's whole weight
            // is standing on them, which X-Plane rightly calls a crash.
            //
            // So the gear is only ever driven once the systems have reported
            // a real position at least once. Until then the .acf's own value
            // stands, which is gear down -- the state an aircraft sitting on
            // its wheels is actually in. The latch is set for the rest of
            // the session by the first plausible reading, so a genuine
            // retraction or a real gear failure afterwards still passes
            // through untouched.
            let (c, l, rr) = (read(&ids.gear_center), read(&ids.gear_left), read(&ids.gear_right));
            if !self.gear_reported && (c > 0. || l > 0. || rr > 0.) {
                self.gear_reported = true;
            }
            let lowest = deploy.iter().cloned().fold(f64::INFINITY, f64::min);
            if lowest < 0.99 && self.gear_deploy_floor > lowest {
                self.gear_deploy_floor = lowest;
                crate::log(&format!(
                    "gear deploy -> X-Plane: {deploy:?} (GEAR_CENTER/LEFT/RIGHT_POSITION {c:.1}/{l:.1}/{rr:.1}%, handle_down={handle_down}, systems have reported: {})",
                    self.gear_reported
                ));
            }
            if self.gear_reported {
                xplm.set_vf(d, &deploy.map(|v| v as f32));
            }
        }
        if let Some(d) = r.gear_handle_down {
            xplm.set_i(d, handle_down as c_int);
        }

        // Brakes: the force factor MSFS gets as AXIS_*_BRAKE_SET every tick
        // (brakes.rs:47-52, 59-64). MSFS's own parking brake is never set,
        // its events being masked; FlyByWire's parking brake is brake pressure.
        if let Some(d) = r.left_brake {
            xplm.set_f(d, read(&ids.brake_left_force).clamp(0., 1.) as f32);
        }
        if let Some(d) = r.right_brake {
            xplm.set_f(d, read(&ids.brake_right_force).clamp(0., 1.) as f32);
        }
        if let Some(d) = r.parkbrake {
            xplm.set_f(d, 0.);
        }
        // X-Plane's autobrake off (1), FlyByWire's autobrake braking through
        // the force factors.
        if let Some(d) = r.auto_brake_level {
            xplm.set_i(d, 1);
        }

        // Flaps and slats (flaps.rs:53-60, 97-153).
        let flap_deg = (read(&ids.left_flaps_angle) + read(&ids.right_flaps_angle)) / 2.;
        let slat_deg = (read(&ids.left_slats_angle) + read(&ids.right_slats_angle)) / 2.;
        let flap = physics::flap_ratio_for_angle(flap_deg, &physics::ACF_FLAP_DETENT_DEG) as f32;
        let slat = physics::slat_ratio_for_angle(slat_deg) as f32;
        for d in [r.flaprqst, r.flap_handle_request, r.flaprat, r.flap2rat, r.flap1_deploy, r.flap2_deploy]
            .into_iter()
            .flatten()
        {
            xplm.set_f(d, flap);
        }
        for d in [r.slatrat, r.slat1_deploy, r.slat2_deploy].into_iter().flatten() {
            xplm.set_f(d, slat);
        }
        // With the control surfaces overridden (by the flight controls), the
        // flap surfaces are only moved by writing their deflection.
        if r.override_control_surfaces.is_some_and(|d| xplm.get_i(d) != 0) {
            let degrees = [flap_deg.min(physics::ACF_FLAP_DETENT_DEG[5]) as f32; 48];
            for d in [r.flap1_deg, r.flap2_deg].into_iter().flatten() {
                xplm.set_vf(d, &degrees);
            }
        }

        // Steering: MSFS gets STEERING_SET every tick
        // (nose_wheel_steering.rs:123-157); X-Plane gets the tyre angles.
        let steer = physics::tyre_steer_deg(read(&ids.nose_ratio), read(&ids.left_body_ratio), read(&ids.right_body_ratio))
            .map(|v| v as f32);
        for d in [r.steer_command, r.steer_actual].into_iter().flatten() {
            xplm.set_vf(d, &steer);
        }
    }

    /// Hand gear, brakes and steering back to X-Plane.
    pub fn release(&mut self, xplm: &Xplm) {
        for d in self.refs.overrides().into_iter().flatten() {
            xplm.set_i(d, 0);
        }
        for (i, r) in self.registered.drain(..).enumerate() {
            xplm.unregister_command_handler(r.command, on_command, i as *mut c_void);
        }
        if let Ok(mut p) = PENDING.lock() {
            *p = None;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn every_stock_command_goes_to_an_event_fbw_handles() {
        let mut vars = aspects::tests::TestVars::default();
        let a = aspects::Aspects::new(&mut vars);
        for &(name, event, kind) in STOCK {
            assert!(kind == Kind::Swallow || is_trim_pulse(event) || a.handles(event), "{name} -> {event}");
        }
        for &(event, _) in FBW_EVENTS {
            assert!(a.handles(event), "{event}");
        }
    }

    #[test]
    fn a_held_brake_command_ramps_the_pedal_input_like_a_held_key() {
        // brakes.rs:66-75 with a held X-Plane command: one event per tick.
        let mut vars = aspects::tests::TestVars::default();
        let mut a = aspects::Aspects::new(&mut vars);
        for _ in 0..5 {
            a.handle(&mut vars, "BRAKES", 0, 0.);
            a.pre_tick(&mut vars, 0.2);
            a.post_tick(&mut vars, 0.);
        }
        assert!((vars.value("LEFT_BRAKE_PEDAL_INPUT") - 60.).abs() < 1e-9);
    }

    #[test]
    fn x_planes_trim_commands_reach_the_prims_trim_switches() {
        for (command, pulse) in [
            ("sim/flight_controls/pitch_trim_up", "XP_PITCH_TRIM_UP_PULSE"),
            ("sim/flight_controls/pitch_trim_down", "XP_PITCH_TRIM_DOWN_PULSE"),
            ("sim/flight_controls/rudder_trim_left", "XP_RUDDER_TRIM_LEFT_PULSE"),
            ("sim/flight_controls/rudder_trim_right", "XP_RUDDER_TRIM_RIGHT_PULSE"),
            ("sim/flight_controls/rudder_trim_center", "XP_RUDDER_TRIM_RESET_PULSE"),
        ] {
            let entry = STOCK.iter().find(|(name, _, _)| *name == command);
            assert_eq!(entry.map(|(_, event, _)| *event), Some(pulse), "{command} must drive {pulse}");
            assert!(is_trim_pulse(pulse));
        }
        assert!(!is_trim_pulse("GEAR_UP") && !is_trim_pulse("BRAKES"));
    }
}
