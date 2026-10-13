//! What the aircraft presets' calculator code reads and sends, in X-Plane.
//!
//! `(L:...)` variables are the plugin's variables under the same name.
//! `(A:...)` simulator variables are the plugin's simulator variables too,
//! except where X-Plane owns the thing. The key events do what MSFS does, on
//! the same variables and X-Plane state the converted cockpit uses for the
//! same switches, so a preset and a click agree:
//!
//! | event (args) | here | source |
//! |---|---|---|
//! | ELECTRICAL_BUS_TO_CIRCUIT_CONNECTION_TOGGLE (bus, circuit) | toggles `CIRCUIT CONNECTION ON:circuit` | msfs2xp-aircraft behaviour/events.rs `k_event` |
//! | FUELSYSTEM_VALVE_OPEN / _CLOSE (n) | engine LP valves 1-4: the engine master, `GENERAL ENG STARTER:n`; others `FUELSYSTEM VALVE SWITCH:n` | fuel.rs:18-19 and fadec.rs:19-24 (this plugin's master switch opens the LP valve) |
//! | TURBINE_IGNITION_SWITCH_SETn (v) | `TURB ENG IGNITION SWITCH EX1:n` = v | events.rs `k_event` |
//! | CABIN_SEATBELTS_ALERT_SWITCH_TOGGLE | toggles `CABIN SEATBELTS ALERT SWITCH` | events.rs `k_event` |
//! | SPOILERS_ARM_SET (v) | X-Plane `speedbrake_ratio` -0.5 (armed) or 0 | prim.rs `spoilers_from_xplane` reads below -0.25 as armed |
//! | RUDDER_TRIM_SET (v) | X-Plane `rudder_trim` = v / 16383 | MSFS event range -16383..16383 (SDK Event IDs) |
//! | BEACON_LIGHTS_ON / _OFF | `beacon_on` | msfs2xp-aircraft lights.rs `lights_obj` (X-Plane's own light switches) |
//! | NAV_LIGHTS_SET (v) | `navigation_lights_on` | lights.rs |
//! | LOGO_LIGHTS_SET (v) | `generic_lights_switch[2]` | lights.rs `lights_obj` |
//! | TAXI_LIGHTS_ON / _OFF (1) | `taxi_light_on` (the nose TAKEOFF_1 light) | lights.rs `lights_obj` |
//! | TAXI_LIGHTS_ON / _OFF (2, 3) | `generic_lights_switch[0]` (runway turn-off) | lights.rs `lights_obj` |
//! | LANDING_LIGHTS_ON / _OFF (n) | `landing_lights_switch` of that index's lights | see [`landing_lights`] |
//!
//! The reads of those lights, the rudder trim and the spoiler lever come from
//! the same X-Plane state. X-Plane shows the wing taxi lights (MSFS taxi index
//! 1) on generic light 0 with the turn-off lights (lights.rs `lights_obj`), so
//! index 1 here switches only the nose light; the turn-off lights keep their
//! own switch.

use systems::simulation::{SimulatorReaderWriter, VariableRegistry};

use super::rpn::RpnHost;
use super::{calculator_variable, XplaneIo};

use crate::key_events::{landing_lights, RUDDER_TRIM, SWITCHES};

/// The index after a simulator variable's name, 0 without one.
fn split_index(name: &str) -> (&str, usize) {
    match name.rsplit_once(':') {
        Some((base, index)) => index.trim().parse().map_or((name, 0), |i| (base.trim(), i)),
        None => (name, 0),
    }
}

pub struct PresetHost<'a, V, X> {
    pub vars: &'a mut V,
    pub xplane: &'a mut X,
}

impl<V: VariableRegistry + SimulatorReaderWriter, X: XplaneIo> PresetHost<'_, V, X> {
    fn read_var(&mut self, kind: &str, name: &str) -> f64 {
        let id = calculator_variable(self.vars, kind, name);
        self.vars.read(&id)
    }

    fn switch(&mut self, name: &str, index: Option<usize>) -> f64 {
        self.xplane.get(&format!("{SWITCHES}{name}"), index).unwrap_or(0.)
    }

    /// The X-Plane state behind a simulator variable, if X-Plane owns it.
    fn xplane_read(&mut self, name: &str) -> Option<f64> {
        let (base, index) = split_index(name);
        let on = |v: f64| if v > 0. { 1. } else { 0. };
        Some(match base {
            "LIGHT BEACON" => on(self.switch("beacon_on", None)),
            "LIGHT NAV" => on(self.switch("navigation_lights_on", None)),
            "LIGHT LOGO" => on(self.switch("generic_lights_switch", Some(2))),
            "LIGHT TAXI" if index <= 1 => on(self.switch("taxi_light_on", None)),
            "LIGHT TAXI" => on(self.switch("generic_lights_switch", Some(0))),
            "LIGHT LANDING" => {
                let lights = landing_lights(index);
                let lit = lights.iter().any(|&i| self.switch("landing_lights_switch", Some(i)) > 0.);
                if lit {
                    1.
                } else {
                    0.
                }
            }
            // Radians in MSFS; only its sign and zero are compared.
            "RUDDER TRIM" => self.xplane.get(RUDDER_TRIM, None).unwrap_or(0.),
            _ => return None,
        })
    }
}

impl<V: VariableRegistry + SimulatorReaderWriter, X: XplaneIo> RpnHost for PresetHost<'_, V, X> {
    fn get(&mut self, kind: &str, name: &str, _unit: &str) -> f64 {
        if kind == "A" {
            if let Some(v) = self.xplane_read(name) {
                return v;
            }
            // The converter's cockpit writes the seat belt switch without
            // an index (events.rs `k_event`).
            if split_index(name).0 == "CABIN SEATBELTS ALERT SWITCH" {
                return self.read_var("A", "CABIN SEATBELTS ALERT SWITCH");
            }
        }
        self.read_var(kind, name)
    }

    fn set(&mut self, kind: &str, name: &str, _unit: &str, value: f64) {
        let id = calculator_variable(self.vars, kind, name);
        self.vars.write(&id, value);
    }

    fn key_event(&mut self, name: &str, args: &[f64]) {
        if !crate::key_events::apply(self.vars, self.xplane, name, args) {
            crate::log(&format!("extra backend: aircraft presets: key event {name} has no X-Plane effect here"));
        }
    }
}

#[cfg(test)]
pub mod test_xplane {
    use std::collections::HashMap;

    use super::XplaneIo;

    /// X-Plane's datarefs as a map, `name` or `name[i]`.
    #[derive(Default)]
    pub struct FakeXplane {
        pub values: HashMap<String, f64>,
        pub commands: Vec<String>,
    }

    fn key(dataref: &str, index: Option<usize>) -> String {
        match index {
            Some(i) => format!("{dataref}[{i}]"),
            None => dataref.to_string(),
        }
    }

    impl XplaneIo for FakeXplane {
        fn get(&mut self, dataref: &str, index: Option<usize>) -> Option<f64> {
            Some(self.values.get(&key(dataref, index)).copied().unwrap_or(0.))
        }
        fn set(&mut self, dataref: &str, index: Option<usize>, value: f64) {
            self.values.insert(key(dataref, index), value);
        }
        fn command_once(&mut self, command: &str) {
            self.commands.push(command.to_string());
        }
    }
}

#[cfg(test)]
mod tests {
    use super::test_xplane::FakeXplane;
    use super::*;
    use crate::aspects::test_vars::TestVars;
    use crate::extra_backend::rpn::execute;

    #[test]
    fn preset_actions_reach_the_cockpit_state_and_read_back() {
        let mut vars = TestVars::default();
        let mut xplane = FakeXplane::default();
        let mut host = PresetHost { vars: &mut vars, xplane: &mut xplane };
        // aircraft_preset_procedures.xml:153-155: the step's condition turns true.
        let condition = "(A:CIRCUIT CONNECTION ON:2, Bool)";
        assert_eq!(execute(condition, &mut host), 0.);
        execute("2 1 (>K:2:ELECTRICAL_BUS_TO_CIRCUIT_CONNECTION_TOGGLE)", &mut host);
        assert_eq!(execute(condition, &mut host), 1.);
        // Engine 1 master (xml:567-570).
        execute("1 (>K:FUELSYSTEM_VALVE_OPEN)", &mut host);
        assert_eq!(host.vars.value("GENERAL ENG STARTER:1"), 1.);
        // L:A32NX_ names are the plugin's prefixed variables.
        execute("1 (>L:A32NX_OVHD_ELEC_BAT_1_PB_IS_AUTO)", &mut host);
        assert_eq!(host.vars.value("A32NX_OVHD_ELEC_BAT_1_PB_IS_AUTO"), 1.);
        // Lights (xml:134-137, 782-789).
        execute("1 (>K:2:LOGO_LIGHTS_SET) 1 (>K:2:NAV_LIGHTS_SET)", &mut host);
        assert_eq!(execute("(A:LIGHT LOGO, Bool) (A:LIGHT NAV, Bool) &&", &mut host), 1.);
        execute("2 (>K:LANDING_LIGHTS_ON)", &mut host);
        assert_eq!(execute("(A:LIGHT LANDING:2, Number) 1 ==", &mut host), 1.);
        assert_eq!(execute("(A:LIGHT LANDING:1, Number) 1 ==", &mut host), 0.);
        // Seat belts (xml:138-143) toggle the switch the cockpit uses.
        let seat_belts = "(A:CABIN SEATBELTS ALERT SWITCH:1, BOOL) ! if{ 1 (>K:CABIN_SEATBELTS_ALERT_SWITCH_TOGGLE) }";
        execute(seat_belts, &mut host);
        execute(seat_belts, &mut host);
        assert_eq!(host.vars.value("CABIN SEATBELTS ALERT SWITCH"), 1.);
        // Spoiler arm and rudder trim reset (xml:620-627).
        execute("1 (>K:SPOILERS_ARM_SET)", &mut host);
        assert_eq!(host.xplane.values["sim/cockpit2/controls/speedbrake_ratio"], -0.5);
        host.xplane.values.insert(RUDDER_TRIM.into(), 0.2);
        execute("0 (>K:RUDDER_TRIM_SET)", &mut host);
        assert_eq!(execute("(A:RUDDER TRIM, Radians) 0 ==", &mut host), 1.);
    }
}
