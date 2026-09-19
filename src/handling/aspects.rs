//! FlyByWire's MSFS input aspects for gear, brakes, autobrake, flaps and
//! steering, ported from `a380_systems_wasm` and the aspect machinery in
//! `systems_wasm/src/aspects.rs`.
//!
//! Each MSFS key event FlyByWire intercepts becomes one [`EventToVariable`]
//! carrying the same mapping, target, leading debounce and reset value as the
//! `builder.event_to_variable(...)` call it comes from. Pre-tick and post-tick
//! actions (`map`, `map_many`, `reduce`) follow in the order the aspects are
//! declared (a380_systems_wasm/src/lib.rs:602-613: brakes, autobrakes,
//! nose_wheel_steering, body_wheel_steering, flaps, ..., gear).
//!
//! Variables FlyByWire keeps inside its WASM module (`Variable::aspect`, such as
//! `BRAKES_LEFT_EVENT` or `RAW_TILLER_HANDLE_POSITION`) are kept here; named
//! variables live in the plugin's variables and are the same `fbw/` datarefs
//! the systems read.

use systems::shared::{from_bool, normalise_angle, to_bool};
use systems::simulation::{SimulatorReaderWriter, VariableIdentifier, VariableRegistry};

/// SimConnect axis positions (systems_wasm/src/lib.rs:653-657).
const OFFSET_32KPOS: f64 = 16384.;
const RANGE_32KPOS: f64 = 32768.;

/// `sim_connect_32k_pos_to_f64` (systems_wasm/src/lib.rs:659-664).
pub fn pos_32k_to_f64(data: u32) -> f64 {
    (((data as i32) as f64 + OFFSET_32KPOS) / RANGE_32KPOS).clamp(0., 1.)
}

/// `sim_connect_32k_pos_inv_to_f64` (systems_wasm/src/lib.rs:666-671).
pub fn pos_32k_inv_to_f64(data: u32) -> f64 {
    ((-((data as i32) as f64) + OFFSET_32KPOS) / RANGE_32KPOS).clamp(0., 1.)
}

/// `f64_to_sim_connect_32k_pos` (systems_wasm/src/lib.rs:673-679): what an
/// axis at `ratio` (0..1) sends as event data.
pub fn f64_to_pos_32k(ratio: f64) -> u32 {
    ((ratio * RANGE_32KPOS) - OFFSET_32KPOS) as i32 as u32
}

/// The variables FlyByWire keeps inside its aspects.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AspectVar {
    Brakes,
    BrakesLeft,
    BrakesRight,
    BrakesLeftEvent,
    BrakesRightEvent,
    RawTillerHandlePosition,
    RawRudderPedalPosition,
}

const ASPECT_VARS: usize = 7;

#[derive(Clone, Copy, Debug)]
enum Target {
    Named(VariableIdentifier),
    Aspect(AspectVar),
}

/// `EventToVariableMapping` (aspects.rs:703-732).
#[derive(Clone, Copy)]
enum Mapping {
    Value(f64),
    EventDataRaw,
    EventData32kPosition,
    EventData32kPositionInverted,
    EventDataToValue(fn(u32) -> f64),
    CurrentValueToValue(fn(f64) -> f64),
    EventDataAndCurrentValueToValue(fn(u32, f64) -> f64),
    SmoothPress(f64, f64),
}

/// One `EventToVariable` (aspects.rs:746-839) with its `Debounce`
/// (aspects.rs:581-668).
struct EventToVariable {
    event: &'static str,
    target: Target,
    mapping: Mapping,
    /// Leading debounce in seconds; `None` is `NoDebounce`.
    debounce: Option<f64>,
    reset_to: Option<f64>,
    handled_at: Option<f64>,
    event_handled_before_tick: bool,
}

/// `options.mask()` alone.
#[derive(Clone, Copy)]
struct Options {
    debounce: Option<f64>,
    reset_to: Option<f64>,
}

const MASK: Options = Options { debounce: None, reset_to: None };

/// gear.rs:22-30.
fn gear_toggle(current: f64) -> f64 {
    if current > 0.5 {
        0.
    } else {
        1.
    }
}

/// flaps.rs:81-95.
pub fn get_handle_pos_from_0_1(input: f64, current_value: f64) -> f64 {
    if input < -0.8 {
        0.
    } else if input > -0.7 && input < -0.3 {
        1.
    } else if input > -0.2 && input < 0.2 {
        2.
    } else if input > 0.3 && input < 0.7 {
        3.
    } else if input > 0.8 {
        4.
    } else {
        current_value
    }
}

/// nose_wheel_steering.rs:187-193.
fn recenter_when_close_to_center(value: f64, increment: f64) -> f64 {
    if value < 0.5 + increment && value > 0.5 - increment {
        0.5
    } else {
        value
    }
}

/// nose_wheel_steering.rs:57.
const TILLER_KEYBOARD_INCREMENTS: f64 = 0.05;
/// brakes.rs:68-69.
const KEYBOARD_PRESS_SPEED: f64 = 0.6;
const KEYBOARD_RELEASE_SPEED: f64 = 0.3;

/// nose_wheel_steering.rs:195-204.
const MAX_CONTROLLABLE_STEERING_ANGLE_DEGREES: f64 = 70.;
pub fn steering_animation_to_msfs_from_steering_angle(nose_wheel_position: f64) -> f64 {
    const STEERING_ANIMATION_TOTAL_RANGE_DEGREES: f64 = 140.;
    ((nose_wheel_position * MAX_CONTROLLABLE_STEERING_ANGLE_DEGREES
        / (STEERING_ANIMATION_TOTAL_RANGE_DEGREES / 2.))
        / 2.)
        + 0.5
}

/// The named variables the aspects read and write.
struct Ids {
    gear_lever_position_request: VariableIdentifier,
    park_brake_lever_pos: VariableIdentifier,
    left_brake_pedal_input: VariableIdentifier,
    right_brake_pedal_input: VariableIdentifier,
    autobrakes_selected_mode: VariableIdentifier,
    rto_arm_is_pressed: VariableIdentifier,
    autobrake_instinctive_disconnect: VariableIdentifier,
    flaps_handle_index: VariableIdentifier,
    flaps_handle_percent: VariableIdentifier,
    rudder_pedal_position: VariableIdentifier,
    realistic_tiller_enabled: VariableIdentifier,
    rudder_pedal_position_ratio: VariableIdentifier,
    tiller_handle_position: VariableIdentifier,
    tiller_pedal_disconnect: VariableIdentifier,
    nose_wheel_position_ratio: VariableIdentifier,
    nose_wheel_position: VariableIdentifier,
    center_wheel_rotation_angle: VariableIdentifier,
    nose_wheel_left_anim_angle: VariableIdentifier,
    nose_wheel_right_anim_angle: VariableIdentifier,
    left_body_steering_position_ratio: VariableIdentifier,
    right_body_steering_position_ratio: VariableIdentifier,
    left_body_wheel_steering_position: VariableIdentifier,
    right_body_wheel_steering_position: VariableIdentifier,
    left_wheel_rotation_angle: VariableIdentifier,
    right_wheel_rotation_angle: VariableIdentifier,
    left_body_right_anim: VariableIdentifier,
    left_body_left_anim: VariableIdentifier,
    right_body_right_anim: VariableIdentifier,
    right_body_left_anim: VariableIdentifier,
}

/// FlyByWire's handling aspects.
pub struct Aspects {
    handlers: Vec<EventToVariable>,
    aspect: [f64; ASPECT_VARS],
    ids: Ids,
}

impl Aspects {
    pub fn new(registry: &mut impl VariableRegistry) -> Self {
        let mut get = |name: &str| registry.get(name.to_owned());
        let ids = Ids {
            gear_lever_position_request: get("GEAR_LEVER_POSITION_REQUEST"),
            park_brake_lever_pos: get("PARK_BRAKE_LEVER_POS"),
            left_brake_pedal_input: get("LEFT_BRAKE_PEDAL_INPUT"),
            right_brake_pedal_input: get("RIGHT_BRAKE_PEDAL_INPUT"),
            autobrakes_selected_mode: get("AUTOBRAKES_SELECTED_MODE"),
            rto_arm_is_pressed: get("OVHD_AUTOBRK_RTO_ARM_IS_PRESSED"),
            autobrake_instinctive_disconnect: get("AUTOBRAKE_INSTINCTIVE_DISCONNECT"),
            flaps_handle_index: get("FLAPS_HANDLE_INDEX"),
            flaps_handle_percent: get("FLAPS_HANDLE_PERCENT"),
            rudder_pedal_position: get("RUDDER_PEDAL_POSITION"),
            realistic_tiller_enabled: get("REALISTIC_TILLER_ENABLED"),
            rudder_pedal_position_ratio: get("RUDDER_PEDAL_POSITION_RATIO"),
            tiller_handle_position: get("TILLER_HANDLE_POSITION"),
            tiller_pedal_disconnect: get("TILLER_PEDAL_DISCONNECT"),
            nose_wheel_position_ratio: get("NOSE_WHEEL_POSITION_RATIO"),
            nose_wheel_position: get("NOSE_WHEEL_POSITION"),
            center_wheel_rotation_angle: get("CENTER WHEEL ROTATION ANGLE"),
            nose_wheel_left_anim_angle: get("NOSE_WHEEL_LEFT_ANIM_ANGLE"),
            nose_wheel_right_anim_angle: get("NOSE_WHEEL_RIGHT_ANIM_ANGLE"),
            left_body_steering_position_ratio: get("LEFT_BODY_STEERING_POSITION_RATIO"),
            right_body_steering_position_ratio: get("RIGHT_BODY_STEERING_POSITION_RATIO"),
            left_body_wheel_steering_position: get("LEFT_BODY_WHEEL_STEERING_POSITION"),
            right_body_wheel_steering_position: get("RIGHT_BODY_WHEEL_STEERING_POSITION"),
            left_wheel_rotation_angle: get("LEFT WHEEL ROTATION ANGLE"),
            right_wheel_rotation_angle: get("RIGHT WHEEL ROTATION ANGLE"),
            left_body_right_anim: get("LEFT_BODY_WHEEL_STEERING_RIGHT_ANIM_ANGLE"),
            left_body_left_anim: get("LEFT_BODY_WHEEL_STEERING_LEFT_ANIM_ANGLE"),
            right_body_right_anim: get("RIGHT_BODY_WHEEL_STEERING_RIGHT_ANIM_ANGLE"),
            right_body_left_anim: get("RIGHT_BODY_WHEEL_STEERING_LEFT_ANIM_ANGLE"),
        };

        let mut handlers = Vec::new();
        let mut add = |event: &'static str, mapping: Mapping, target: Target, options: Options| {
            handlers.push(EventToVariable {
                event,
                target,
                mapping,
                debounce: options.debounce,
                reset_to: options.reset_to,
                handled_at: None,
                event_handled_before_tick: false,
            });
        };
        use Mapping::*;
        use Target::{Aspect, Named};

        // brakes.rs:10-35
        let park = Named(ids.park_brake_lever_pos);
        add("PARKING_BRAKES", CurrentValueToValue(|v| from_bool(!to_bool(v))), park, MASK);
        add("PARKING_BRAKE_SET", EventDataToValue(|d| from_bool(d == 1)), park, MASK);
        add("PARKING_BRAKES_ON", Value(1.), park, MASK);
        add("PARKING_BRAKES_OFF", Value(0.), park, MASK);
        // brakes.rs:41-58
        add("AXIS_LEFT_BRAKE_SET", EventData32kPosition, Aspect(AspectVar::BrakesLeftEvent), MASK);
        add("AXIS_RIGHT_BRAKE_SET", EventData32kPosition, Aspect(AspectVar::BrakesRightEvent), MASK);
        // brakes.rs:66-87
        let smooth = SmoothPress(KEYBOARD_PRESS_SPEED, KEYBOARD_RELEASE_SPEED);
        add("BRAKES", smooth, Aspect(AspectVar::Brakes), MASK);
        add("BRAKES_LEFT", smooth, Aspect(AspectVar::BrakesLeft), MASK);
        add("BRAKES_RIGHT", smooth, Aspect(AspectVar::BrakesRight), MASK);

        // autobrakes.rs:7-12, 41-42
        let button_press = Options { debounce: Some(1.5), reset_to: Some(0.) };
        let knob_event = Options { debounce: Some(1.5), reset_to: None };
        let options_set = Options { debounce: Some(0.125), reset_to: None };
        let mode = Named(ids.autobrakes_selected_mode);
        // autobrakes.rs:16-39
        add("AUTOBRAKE_LO_SET", Value(2.), mode, knob_event);
        add("AUTOBRAKE_MED_SET", Value(4.), mode, knob_event);
        add("AUTOBRAKE_HI_SET", Value(1.), Named(ids.rto_arm_is_pressed), button_press);
        add(
            "A32NX.AUTO_THROTTLE_DISCONNECT",
            Value(1.),
            Named(ids.autobrake_instinctive_disconnect),
            button_press,
        );
        // autobrakes.rs:44-91
        add("A32NX.AUTOBRAKE_SET", EventDataToValue(|d| d as f64), mode, options_set);
        add("A32NX.AUTOBRAKE_SET_DISARM", Value(0.), mode, options_set);
        add("AUTOBRAKE_DISARM", Value(0.), mode, options_set);
        add("A32NX.AUTOBRAKE_SET_BTV", Value(1.), mode, options_set);
        add("A32NX.AUTOBRAKE_SET_LO", Value(2.), mode, options_set);
        add("A32NX.AUTOBRAKE_SET_L2", Value(3.), mode, options_set);
        add("A32NX.AUTOBRAKE_SET_L3", Value(4.), mode, options_set);
        add("A32NX.AUTOBRAKE_SET_HI", Value(5.), mode, options_set);

        // nose_wheel_steering.rs:43-88
        let tiller = Aspect(AspectVar::RawTillerHandlePosition);
        add("AXIS_MIXTURE4_SET", EventData32kPosition, tiller, MASK);
        add("AXIS_STEERING_SET", EventData32kPositionInverted, tiller, MASK);
        add(
            "STEERING_INC",
            CurrentValueToValue(|v| {
                recenter_when_close_to_center((v + TILLER_KEYBOARD_INCREMENTS).min(1.), TILLER_KEYBOARD_INCREMENTS)
            }),
            tiller,
            MASK,
        );
        add(
            "STEERING_DEC",
            CurrentValueToValue(|v| {
                recenter_when_close_to_center((v - TILLER_KEYBOARD_INCREMENTS).max(0.), TILLER_KEYBOARD_INCREMENTS)
            }),
            tiller,
            MASK,
        );
        add(
            "TOGGLE_WATER_RUDDER",
            Value(1.),
            Named(ids.tiller_pedal_disconnect),
            Options { debounce: None, reset_to: Some(0.) },
        );

        // flaps.rs:10-44
        let flaps = Named(ids.flaps_handle_index);
        add("FLAPS_INCR", CurrentValueToValue(|v| (v + 1.).min(4.)), flaps, MASK);
        add("FLAPS_DECR", CurrentValueToValue(|v| (v - 1.).max(0.)), flaps, MASK);
        add("FLAPS_UP", Value(0.), flaps, MASK);
        add("FLAPS_1", Value(1.), flaps, MASK);
        add("FLAPS_2", Value(2.), flaps, MASK);
        add("FLAPS_3", Value(3.), flaps, MASK);
        add("FLAPS_DOWN", Value(4.), flaps, MASK);
        add(
            "FLAPS_SET",
            EventDataAndCurrentValueToValue(|d, v| get_handle_pos_from_0_1((d as i32 as f64) / 8192. - 1., v)),
            flaps,
            MASK,
        );
        add(
            "AXIS_FLAPS_SET",
            EventDataAndCurrentValueToValue(|d, v| get_handle_pos_from_0_1((d as i32 as f64) / 16384., v)),
            flaps,
            MASK,
        );

        // gear.rs:13-47
        let gear = Named(ids.gear_lever_position_request);
        add("GEAR_SET", EventDataRaw, gear, MASK);
        add("GEAR_TOGGLE", CurrentValueToValue(gear_toggle), gear, MASK);
        add("GEAR_UP", Value(0.), gear, MASK);
        add("GEAR_DOWN", Value(1.), gear, MASK);

        let mut aspect = [0.; ASPECT_VARS];
        // nose_wheel_steering.rs:11, 40
        aspect[AspectVar::RawRudderPedalPosition as usize] = 0.5;
        aspect[AspectVar::RawTillerHandlePosition as usize] = 0.5;

        Self { handlers, aspect, ids }
    }

    /// Whether FlyByWire intercepts (and masks) an event of this name.
    pub fn handles(&self, event: &str) -> bool {
        self.handlers.iter().any(|h| h.event == event)
    }

    pub fn aspect(&self, var: AspectVar) -> f64 {
        self.aspect[var as usize]
    }

    fn read(&self, vars: &mut impl SimulatorReaderWriter, target: Target) -> f64 {
        match target {
            Target::Named(id) => vars.read(&id),
            Target::Aspect(a) => self.aspect[a as usize],
        }
    }

    fn write(&mut self, vars: &mut impl SimulatorReaderWriter, target: Target, value: f64) {
        match target {
            Target::Named(id) => vars.write(&id, value),
            Target::Aspect(a) => self.aspect[a as usize] = value,
        }
    }

    /// An event arriving, as `EventToVariable::handle` (aspects.rs:817-834).
    /// `now` is seconds on a monotonic clock, standing in for `Instant::now()`.
    /// Returns whether a handler took it.
    pub fn handle(&mut self, vars: &mut impl SimulatorReaderWriter, event: &str, data: u32, now: f64) -> bool {
        let Some(i) = self.handlers.iter().position(|h| h.event == event) else {
            return false;
        };
        let (target, mapping, debounce, handled_at) = {
            let h = &self.handlers[i];
            (h.target, h.mapping, h.debounce, h.handled_at)
        };
        // LeadingDebounce::should_handle (aspects.rs:640-651).
        let should_handle = match (debounce, handled_at) {
            (Some(duration), Some(at)) => now - at > duration,
            _ => true,
        };
        if should_handle {
            let value = match mapping {
                Mapping::Value(v) => v,
                Mapping::EventDataRaw => data as f64,
                Mapping::EventData32kPosition => pos_32k_to_f64(data),
                Mapping::EventData32kPositionInverted => pos_32k_inv_to_f64(data),
                Mapping::EventDataToValue(f) => f(data),
                Mapping::CurrentValueToValue(f) => f(self.read(vars, target)),
                Mapping::EventDataAndCurrentValueToValue(f) => f(data, self.read(vars, target)),
                Mapping::SmoothPress(..) => self.read(vars, target),
            };
            self.write(vars, target, value);
            let h = &mut self.handlers[i];
            if debounce.is_some() {
                h.handled_at = Some(now);
            }
            h.event_handled_before_tick = true;
        }
        true
    }

    /// `MsfsAspect::pre_tick` (aspects.rs:320-333) for the handling aspects.
    pub fn pre_tick(&mut self, vars: &mut impl SimulatorReaderWriter, delta: f64) {
        // EventToVariable::adjust_smooth_pressed_value (aspects.rs:798-815).
        for i in 0..self.handlers.len() {
            let (target, mapping, pressed) = {
                let h = &self.handlers[i];
                (h.target, h.mapping, h.event_handled_before_tick)
            };
            if let Mapping::SmoothPress(press, release) = mapping {
                let mut value = self.read(vars, target);
                if pressed {
                    value += delta * press;
                } else {
                    value -= delta * release;
                }
                self.write(vars, target, value.clamp(0., 1.));
            }
        }

        // brakes.rs:91-112 with to_percent_max (brakes.rs:117-119).
        let a = |v: AspectVar| self.aspect[v as usize] * 100.;
        let left = [a(AspectVar::Brakes), a(AspectVar::BrakesLeft), a(AspectVar::BrakesLeftEvent)]
            .into_iter()
            .fold(0., f64::max);
        let right = [a(AspectVar::Brakes), a(AspectVar::BrakesRight), a(AspectVar::BrakesRightEvent)]
            .into_iter()
            .fold(0., f64::max);
        vars.write(&self.ids.left_brake_pedal_input, left);
        vars.write(&self.ids.right_brake_pedal_input, right);

        // nose_wheel_steering.rs:13-19
        let pedal = vars.read(&self.ids.rudder_pedal_position);
        self.aspect[AspectVar::RawRudderPedalPosition as usize] = ((pedal + 100.) / 200.) * 2. - 1.;

        // flaps.rs:46-51
        let index = vars.read(&self.ids.flaps_handle_index);
        vars.write(&self.ids.flaps_handle_percent, index / 4.);
    }

    /// `MsfsAspect::post_tick` (aspects.rs:335-347): the post-tick actions,
    /// then each handler's debounce reset.
    pub fn post_tick(&mut self, vars: &mut impl SimulatorReaderWriter, now: f64) {
        let ids = &self.ids;
        let realistic_tiller = to_bool(vars.read(&ids.realistic_tiller_enabled));
        let raw_pedal = self.aspect[AspectVar::RawRudderPedalPosition as usize];
        let raw_tiller = self.aspect[AspectVar::RawTillerHandlePosition as usize];

        // nose_wheel_steering.rs:21-37
        vars.write(&ids.rudder_pedal_position_ratio, if realistic_tiller { raw_pedal } else { 0. });

        // nose_wheel_steering.rs:90-114
        let disconnect = to_bool(vars.read(&ids.tiller_pedal_disconnect));
        let tiller = if realistic_tiller {
            raw_tiller * 2. - 1.
        } else if !disconnect {
            raw_pedal
        } else {
            0.
        };
        vars.write(&ids.tiller_handle_position, tiller);

        // nose_wheel_steering.rs:116-121
        let nose_ratio = vars.read(&ids.nose_wheel_position_ratio);
        let nose_position = steering_animation_to_msfs_from_steering_angle(nose_ratio);
        vars.write(&ids.nose_wheel_position, nose_position);

        // nose_wheel_steering.rs:159-182 (STEERING_SET and
        // NOSE_WHEEL_STEERING_LIMIT_SET, cpp:123-157, are MSFS's steering and
        // have no place here: X-Plane is given the angle directly).
        const STEERING_RATIO_TO_WHEEL_ANGLE_GAIN: f64 = 80.;
        let center = vars.read(&ids.center_wheel_rotation_angle);
        vars.write(
            &ids.nose_wheel_left_anim_angle,
            normalise_angle(center + (nose_position - 0.5) * STEERING_RATIO_TO_WHEEL_ANGLE_GAIN),
        );
        vars.write(
            &ids.nose_wheel_right_anim_angle,
            normalise_angle(center - (nose_position - 0.5) * STEERING_RATIO_TO_WHEEL_ANGLE_GAIN),
        );

        // body_wheel_steering.rs:8-69
        const REAR_STEERING_RATIO_TO_WHEEL_ANGLE_GAIN: f64 = 40.;
        let left_body = -vars.read(&ids.left_body_steering_position_ratio) / 2. + 0.5;
        let right_body = -vars.read(&ids.right_body_steering_position_ratio) / 2. + 0.5;
        vars.write(&ids.left_body_wheel_steering_position, left_body);
        vars.write(&ids.right_body_wheel_steering_position, right_body);
        let left_wheel = vars.read(&ids.left_wheel_rotation_angle);
        let right_wheel = vars.read(&ids.right_wheel_rotation_angle);
        let gain = REAR_STEERING_RATIO_TO_WHEEL_ANGLE_GAIN;
        vars.write(&ids.left_body_right_anim, normalise_angle(left_wheel + (left_body - 0.5) * gain));
        vars.write(&ids.left_body_left_anim, normalise_angle(left_wheel - (left_body - 0.5) * gain));
        vars.write(&ids.right_body_right_anim, normalise_angle(right_wheel + (right_body - 0.5) * gain));
        vars.write(&ids.right_body_left_anim, normalise_angle(right_wheel - (right_body - 0.5) * gain));

        // EventToVariable::post_tick with NoDebounce / LeadingDebounce
        // (aspects.rs:608-615, 653-667, 836-839).
        for i in 0..self.handlers.len() {
            let (target, debounce, reset_to, handled_at) = {
                let h = &self.handlers[i];
                (h.target, h.debounce, h.reset_to, h.handled_at)
            };
            match debounce {
                None => {
                    if let Some(value) = reset_to {
                        self.write(vars, target, value);
                    }
                }
                Some(duration) => {
                    if let Some(at) = handled_at {
                        if now - at > duration {
                            if let Some(value) = reset_to {
                                self.write(vars, target, value);
                            }
                            self.handlers[i].handled_at = None;
                        }
                    }
                }
            }
            self.handlers[i].event_handled_before_tick = false;
        }
    }
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;
    use std::collections::HashMap;

    /// A registry and store like the plugin's, for tests.
    #[derive(Default)]
    pub struct TestVars {
        pub ids: HashMap<String, VariableIdentifier>,
        pub values: Vec<f64>,
    }

    impl TestVars {
        pub fn value(&self, name: &str) -> f64 {
            self.ids.get(name).map_or(f64::NAN, |id| self.values[id.identifier_index()])
        }
        pub fn set(&mut self, name: &str, value: f64) {
            let id = self.get(name.to_owned());
            self.values[id.identifier_index()] = value;
        }
    }

    impl VariableRegistry for TestVars {
        fn get(&mut self, name: String) -> VariableIdentifier {
            if let Some(id) = self.ids.get(&name) {
                return *id;
            }
            let mut id = VariableIdentifier::new(0usize);
            for _ in 0..self.values.len() {
                id = id.next();
            }
            self.values.push(0.);
            self.ids.insert(name, id);
            id
        }
        fn get_unprefixed(&mut self, name: String) -> VariableIdentifier {
            self.get(name)
        }
    }

    impl SimulatorReaderWriter for TestVars {
        fn read(&mut self, id: &VariableIdentifier) -> f64 {
            self.values[id.identifier_index()]
        }
        fn write(&mut self, id: &VariableIdentifier, value: f64) {
            self.values[id.identifier_index()] = value;
        }
    }

    fn setup() -> (Aspects, TestVars) {
        let mut vars = TestVars::default();
        let aspects = Aspects::new(&mut vars);
        (aspects, vars)
    }

    #[test]
    fn axis_positions_convert_as_simconnect_does() {
        assert_eq!(pos_32k_to_f64((-16384i32) as u32), 0.);
        assert_eq!(pos_32k_to_f64(0), 0.5);
        assert_eq!(pos_32k_to_f64(16384), 1.);
        assert_eq!(pos_32k_inv_to_f64(16384), 0.);
        for r in [0., 0.25, 0.5, 1.] {
            assert!((pos_32k_to_f64(f64_to_pos_32k(r)) - r).abs() < 1e-4);
        }
    }

    #[test]
    fn gear_events_drive_the_lever_request() {
        let (mut a, mut v) = setup();
        a.handle(&mut v, "GEAR_DOWN", 0, 0.);
        assert_eq!(v.value("GEAR_LEVER_POSITION_REQUEST"), 1.);
        a.handle(&mut v, "GEAR_TOGGLE", 0, 0.);
        assert_eq!(v.value("GEAR_LEVER_POSITION_REQUEST"), 0.);
        a.handle(&mut v, "GEAR_TOGGLE", 0, 0.);
        assert_eq!(v.value("GEAR_LEVER_POSITION_REQUEST"), 1.);
        a.handle(&mut v, "GEAR_UP", 0, 0.);
        assert_eq!(v.value("GEAR_LEVER_POSITION_REQUEST"), 0.);
        a.handle(&mut v, "GEAR_SET", 1, 0.);
        assert_eq!(v.value("GEAR_LEVER_POSITION_REQUEST"), 1.);
    }

    #[test]
    fn parking_brake_events() {
        let (mut a, mut v) = setup();
        a.handle(&mut v, "PARKING_BRAKES", 0, 0.);
        assert_eq!(v.value("PARK_BRAKE_LEVER_POS"), 1.);
        a.handle(&mut v, "PARKING_BRAKES", 0, 0.);
        assert_eq!(v.value("PARK_BRAKE_LEVER_POS"), 0.);
        a.handle(&mut v, "PARKING_BRAKE_SET", 1, 0.);
        assert_eq!(v.value("PARK_BRAKE_LEVER_POS"), 1.);
        a.handle(&mut v, "PARKING_BRAKE_SET", 2, 0.);
        assert_eq!(v.value("PARK_BRAKE_LEVER_POS"), 0.);
        a.handle(&mut v, "PARKING_BRAKES_ON", 0, 0.);
        assert_eq!(v.value("PARK_BRAKE_LEVER_POS"), 1.);
        a.handle(&mut v, "PARKING_BRAKES_OFF", 0, 0.);
        assert_eq!(v.value("PARK_BRAKE_LEVER_POS"), 0.);
    }

    #[test]
    fn keyboard_brakes_ramp_up_at_0_6_and_down_at_0_3_per_second() {
        let (mut a, mut v) = setup();
        // Held for one second at 10 ticks a second.
        for _ in 0..10 {
            a.handle(&mut v, "BRAKES", 0, 0.);
            a.pre_tick(&mut v, 0.1);
            a.post_tick(&mut v, 0.);
        }
        assert!((v.value("LEFT_BRAKE_PEDAL_INPUT") - 60.).abs() < 1e-9);
        assert!((v.value("RIGHT_BRAKE_PEDAL_INPUT") - 60.).abs() < 1e-9);
        // Released for one second.
        for _ in 0..10 {
            a.pre_tick(&mut v, 0.1);
            a.post_tick(&mut v, 0.);
        }
        assert!((v.value("LEFT_BRAKE_PEDAL_INPUT") - 30.).abs() < 1e-9);
        // Never past full.
        for _ in 0..40 {
            a.handle(&mut v, "BRAKES_LEFT", 0, 0.);
            a.pre_tick(&mut v, 0.1);
            a.post_tick(&mut v, 0.);
        }
        assert_eq!(v.value("LEFT_BRAKE_PEDAL_INPUT"), 100.);
        assert!(v.value("RIGHT_BRAKE_PEDAL_INPUT") == 0.);
    }

    #[test]
    fn toe_brake_axes_give_the_largest_demand_in_percent() {
        let (mut a, mut v) = setup();
        a.handle(&mut v, "AXIS_LEFT_BRAKE_SET", f64_to_pos_32k(0.75), 0.);
        a.handle(&mut v, "AXIS_RIGHT_BRAKE_SET", f64_to_pos_32k(0.25), 0.);
        a.pre_tick(&mut v, 0.1);
        assert!((v.value("LEFT_BRAKE_PEDAL_INPUT") - 75.).abs() < 0.01);
        assert!((v.value("RIGHT_BRAKE_PEDAL_INPUT") - 25.).abs() < 0.01);
    }

    #[test]
    fn autobrake_knob_events_are_debounced_for_1_5_seconds() {
        let (mut a, mut v) = setup();
        a.handle(&mut v, "AUTOBRAKE_LO_SET", 0, 10.);
        assert_eq!(v.value("AUTOBRAKES_SELECTED_MODE"), 2.);
        a.handle(&mut v, "AUTOBRAKE_DISARM", 0, 10.);
        assert_eq!(v.value("AUTOBRAKES_SELECTED_MODE"), 0.);
        // MED within 1.5 s of LO is ignored; its own debounce is separate
        // from DISARM's.
        a.handle(&mut v, "AUTOBRAKE_LO_SET", 0, 11.);
        assert_eq!(v.value("AUTOBRAKES_SELECTED_MODE"), 0.);
        a.post_tick(&mut v, 11.);
        a.handle(&mut v, "AUTOBRAKE_LO_SET", 0, 11.6);
        assert_eq!(v.value("AUTOBRAKES_SELECTED_MODE"), 2.);
        a.handle(&mut v, "AUTOBRAKE_MED_SET", 0, 11.6);
        assert_eq!(v.value("AUTOBRAKES_SELECTED_MODE"), 4.);
        a.handle(&mut v, "A32NX.AUTOBRAKE_SET", 5, 20.);
        assert_eq!(v.value("AUTOBRAKES_SELECTED_MODE"), 5.);
        a.handle(&mut v, "A32NX.AUTOBRAKE_SET_BTV", 0, 21.);
        assert_eq!(v.value("AUTOBRAKES_SELECTED_MODE"), 1.);
    }

    #[test]
    fn rto_button_resets_once_its_debounce_has_passed() {
        let (mut a, mut v) = setup();
        a.handle(&mut v, "AUTOBRAKE_HI_SET", 0, 0.);
        assert_eq!(v.value("OVHD_AUTOBRK_RTO_ARM_IS_PRESSED"), 1.);
        a.post_tick(&mut v, 1.0);
        assert_eq!(v.value("OVHD_AUTOBRK_RTO_ARM_IS_PRESSED"), 1.);
        a.post_tick(&mut v, 1.6);
        assert_eq!(v.value("OVHD_AUTOBRK_RTO_ARM_IS_PRESSED"), 0.);
        a.handle(&mut v, "A32NX.AUTO_THROTTLE_DISCONNECT", 0, 2.);
        assert_eq!(v.value("AUTOBRAKE_INSTINCTIVE_DISCONNECT"), 1.);
    }

    #[test]
    fn flap_events_step_and_clamp_the_handle() {
        let (mut a, mut v) = setup();
        a.handle(&mut v, "FLAPS_DECR", 0, 0.);
        assert_eq!(v.value("FLAPS_HANDLE_INDEX"), 0.);
        for expected in [1., 2., 3., 4., 4.] {
            a.handle(&mut v, "FLAPS_INCR", 0, 0.);
            assert_eq!(v.value("FLAPS_HANDLE_INDEX"), expected);
        }
        a.pre_tick(&mut v, 0.1);
        assert_eq!(v.value("FLAPS_HANDLE_PERCENT"), 1.);
        a.handle(&mut v, "FLAPS_2", 0, 0.);
        assert_eq!(v.value("FLAPS_HANDLE_INDEX"), 2.);
        a.handle(&mut v, "FLAPS_UP", 0, 0.);
        assert_eq!(v.value("FLAPS_HANDLE_INDEX"), 0.);
    }

    #[test]
    fn flap_axis_uses_fbw_detent_windows_and_holds_between_them() {
        let (mut a, mut v) = setup();
        // An axis at 0..1 sends -16384..16384.
        a.handle(&mut v, "AXIS_FLAPS_SET", f64_to_pos_32k(1.), 0.);
        assert_eq!(v.value("FLAPS_HANDLE_INDEX"), 4.);
        a.handle(&mut v, "AXIS_FLAPS_SET", f64_to_pos_32k(0.5), 0.);
        assert_eq!(v.value("FLAPS_HANDLE_INDEX"), 2.);
        // 0.375 is -0.25 normalised: between windows, so unchanged.
        a.handle(&mut v, "AXIS_FLAPS_SET", f64_to_pos_32k(0.375), 0.);
        assert_eq!(v.value("FLAPS_HANDLE_INDEX"), 2.);
        a.handle(&mut v, "AXIS_FLAPS_SET", f64_to_pos_32k(0.25), 0.);
        assert_eq!(v.value("FLAPS_HANDLE_INDEX"), 1.);
        // FLAPS_SET takes 0..16383 (flaps.rs:30).
        a.handle(&mut v, "FLAPS_SET", 0, 0.);
        assert_eq!(v.value("FLAPS_HANDLE_INDEX"), 0.);
        a.handle(&mut v, "FLAPS_SET", 16383, 0.);
        assert_eq!(v.value("FLAPS_HANDLE_INDEX"), 4.);
    }

    #[test]
    fn pedals_steer_through_the_tiller_unless_realistic_tiller_or_disconnected() {
        let (mut a, mut v) = setup();
        v.set("RUDDER_PEDAL_POSITION", 50.);
        a.pre_tick(&mut v, 0.1);
        a.post_tick(&mut v, 0.);
        assert_eq!(v.value("TILLER_HANDLE_POSITION"), 0.5);
        assert_eq!(v.value("RUDDER_PEDAL_POSITION_RATIO"), 0.);

        a.handle(&mut v, "TOGGLE_WATER_RUDDER", 0, 0.);
        a.pre_tick(&mut v, 0.1);
        a.post_tick(&mut v, 0.);
        assert_eq!(v.value("TILLER_HANDLE_POSITION"), 0.);
        // The disconnect resets after the tick.
        assert_eq!(v.value("TILLER_PEDAL_DISCONNECT"), 0.);

        v.set("REALISTIC_TILLER_ENABLED", 1.);
        a.handle(&mut v, "AXIS_STEERING_SET", f64_to_pos_32k(0.), 0.); // inverted: full right
        a.pre_tick(&mut v, 0.1);
        a.post_tick(&mut v, 0.);
        assert_eq!(v.value("TILLER_HANDLE_POSITION"), 1.);
        assert_eq!(v.value("RUDDER_PEDAL_POSITION_RATIO"), 0.5);
    }

    #[test]
    fn tiller_keys_step_by_five_percent_and_recentre() {
        let (mut a, mut v) = setup();
        a.handle(&mut v, "STEERING_INC", 0, 0.);
        assert!((a.aspect(AspectVar::RawTillerHandlePosition) - 0.55).abs() < 1e-12);
        a.handle(&mut v, "STEERING_DEC", 0, 0.);
        assert_eq!(a.aspect(AspectVar::RawTillerHandlePosition), 0.5);
        for _ in 0..30 {
            a.handle(&mut v, "STEERING_DEC", 0, 0.);
        }
        assert_eq!(a.aspect(AspectVar::RawTillerHandlePosition), 0.);
    }

    #[test]
    fn steering_animations_follow_the_actuators() {
        let (mut a, mut v) = setup();
        v.set("NOSE_WHEEL_POSITION_RATIO", 1.);
        v.set("LEFT_BODY_STEERING_POSITION_RATIO", -1.);
        a.post_tick(&mut v, 0.);
        assert_eq!(v.value("NOSE_WHEEL_POSITION"), 1.);
        assert_eq!(v.value("NOSE_WHEEL_LEFT_ANIM_ANGLE"), 40.);
        assert_eq!(v.value("NOSE_WHEEL_RIGHT_ANIM_ANGLE"), 320.);
        assert_eq!(v.value("LEFT_BODY_WHEEL_STEERING_POSITION"), 1.);
        assert_eq!(v.value("RIGHT_BODY_WHEEL_STEERING_POSITION"), 0.5);
    }

    #[test]
    fn unknown_events_are_not_taken() {
        let (mut a, mut v) = setup();
        assert!(!a.handle(&mut v, "TOGGLE_BEACON_LIGHTS", 0, 0.));
        assert!(a.handles("GEAR_UP"));
    }
}
