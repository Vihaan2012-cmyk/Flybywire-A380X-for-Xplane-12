//! MSFS key events (`K:`) that FlyByWire's A380X code sends, with MSFS's
//! effect, for every event that is not a radio or a door event (radios.rs and
//! doors.rs keep theirs) or the pushback tug's (extra_backend/pushback.rs).
//!
//! Where events come from:
//! - scripts: `SimVar.SetSimVarValue('K:NAME', unit, value)`, queued by
//!   js_bridge.rs as `("K:NAME", value)` and handed to [`KeyEvents::handle`]
//!   through lib.rs; the `K:1:` / `K:2:` forms carry their one value as the
//!   first argument;
//! - `KeyEventManager.triggerKey(key, bypass, v0, v1, v2)` (msfs-sdk), which
//!   in MSFS is `Coherent.call('TRIGGER_KEY_EVENT', key, bypass, v0, v1, v2)`
//!   and carries up to three arguments: [`push`] takes them in that order, for
//!   the script runtime to call;
//! - calculator code (the aircraft presets): `v1 v0 (>K:2:NAME)`, the top of
//!   the stack first (extra_backend/rpn.rs).
//!
//! The senders, found by searching fbw-a380x, fbw-common and the fmgc for
//! `triggerKey(`, `'K:` and `SimVar.SetSimVarValue('K:`, and the aircraft
//! preset procedures:
//!
//! | event (arguments) | effect here | sender / source of the semantics |
//! |---|---|---|
//! | LIGHT_POTENTIOMETER_SET (index, percent) | `LIGHT POTENTIOMETER:index` = percent / 100 (MSFS keeps it in percent over 100) | RmpStateController.ts:102, LightSync.ts:171; MSFS SDK Event IDs |
//! | LIGHT_POTENTIOMETER_n_SET (percent) | the same for n | the cockpit's console light switches |
//! | ELECTRICAL_CIRCUIT_TOGGLE (circuit) | toggles `CIRCUIT SWITCH ON:circuit` | LightSync.ts:68, Transponder.ts:84, VhfRadio.ts:71 |
//! | ELECTRICAL_BUS_TO_CIRCUIT_CONNECTION_TOGGLE (bus, circuit) | toggles `CIRCUIT CONNECTION ON:circuit` | aircraft_preset_procedures.xml; msfs2xp-aircraft events.rs `k_event` |
//! | FUELSYSTEM_VALVE_OPEN / _CLOSE (n) | engine LP valves 1-4 are the engine masters `GENERAL ENG STARTER:n` (fuel.rs:18-19, fadec.rs:19-24); others `FUELSYSTEM VALVE SWITCH:n` | preset procedures |
//! | TURBINE_IGNITION_SWITCH_SETn (v) | `TURB ENG IGNITION SWITCH EX1:n` = v | preset procedures |
//! | CABIN_SEATBELTS_ALERT_SWITCH_TOGGLE | toggles `CABIN SEATBELTS ALERT SWITCH` | preset procedures; events.rs |
//! | SPOILERS_ARM_SET (v) | X-Plane `speedbrake_ratio` -0.5 (armed) or 0 | prim.rs `spoilers_from_xplane` reads below -0.25 as armed |
//! | RUDDER_TRIM_SET (v) | X-Plane `rudder_trim` = v / 16383 | preset procedures; MSFS axis range |
//! | BEACON_LIGHTS_ON / _OFF, NAV_LIGHTS_SET, LOGO_LIGHTS_SET, TAXI_LIGHTS_ON / _OFF, LANDING_LIGHTS_ON / _OFF | X-Plane's light switches the converted model's lights use | preset procedures; msfs2xp-aircraft lights.rs `lights_obj` |
//! | XPNDR_SET (BCD16 code) | `sim/cockpit2/radios/actuators/transponder_code` | TransponderController.ts:147, Transponder.ts:70, MfdSurvControls.tsx:284 |
//! | XPNDR_IDENT_ON | `sim/transponder/transponder_ident` | TransponderController.ts:88 |
//! | SIM_RATE_INCR / _DECR | `sim/time/sim_speed` doubled / halved, not below 1 (X-Plane's is an integer) | QuickControls.tsx:193-194 |
//! | TOGGLE_JETWAY | `sim/ground_ops/jetway` | A380Services.tsx (as efb.rs) |
//! | TOGGLE_RAMPTRUCK, REQUEST_LUGGAGE, REQUEST_CATERING, REQUEST_FUEL_KEY | `sim/ground_ops/service_plane` (X-Plane's one truck service) | A380Services.tsx (as efb.rs) |
//! | REQUEST_POWER_SUPPLY | `sim/ground_ops/toggle_gpu_request` | GPUManagement.ts:105 (as efb.rs) |
//! | A32NX.FCU_*, AP_*, AUTO_THROTTLE_*, A32NX.ATHR_RESET_DISABLE | FlyByWire's FCU and autothrust inputs (afs_events.rs) | FmcAircraftInterface.ts:1628-1632, FlightManagementComputer.ts:788 |
//! | A32NX.FMGC_DIR_TO_TRIGGER | `SimInputAutopilot::dir_to_trigger` (afs_events.rs), read by prim.rs as `direct_to_nav_engage` | MfdFmsFplnDirectTo.tsx:260 (DIR TO INSERT) |
//! | AP_MANAGED_SPEED_IN_MACH_ON | `SimInputAutopilot::mach_mode_activate` - MSFS maps this stock event to its own `A32NX_FMGC_MACH_MODE_ACTIVATE`, not a same-named custom event (SimConnectInterface.cpp:759) | FmcAircraftInterface.ts:1071 |
//! | AP_MANAGED_SPEED_IN_MACH_OFF | `SimInputAutopilot::spd_mode_activate`, mapped the same way to `A32NX_FMGC_SPD_MODE_ACTIVATE` (SimConnectInterface.cpp:760) | FmcAircraftInterface.ts:1073 |
//! | A32NX.FMS_PRESET_SPD_ACTIVATE | `SimInputAutopilot::preset_spd_activate` | SimConnectInterface.cpp:2908 (A32NX_FMGC_PRESET_SPD_ACTIVATE) |
//!
//! Known and deliberately not applied (logged once each):
//! - FUELSYSTEM_VALVE_SET, FUELSYSTEM_TRIGGER_TOGGLE / _OFF, FUELSYSTEM_JUNCTION_SET:
//!   LegacyFuel.ts sends them, but LegacyFuel is already ported natively
//!   (fuel_transfer.rs, run by fuel.rs); applying the script's copy as well
//!   would actuate the fuel system twice.
//! - TUG_DISABLE (Efb.tsx:411): releases MSFS's own tug steering lock, which
//!   X-Plane does not have.
//! - A32NX.THROTTLE_MAPPING_*: the flyPad's throttle calibration; throttle.rs
//!   uses FlyByWire's default detents and keeps no calibration file.
//!
//! Checked against the A380X behaviour XML: most of the
//! A32NX_FCU_EFIS_{L,R}_* family, and A32NX_EFIS_{L,R}_CHRONO_PUSHED, are in
//! SimConnectInterface's Events enum (inherited from the A320) but the A380's
//! own EFIS control panel (efis-cp.xml) writes `L:A32NX_FCU_EFIS_*` local vars
//! directly through RPN rather than sending the event, so there is nothing
//! for this module to serve for those.
//!
//! **Four of them are not like that**, and reading the panel as uniformly
//! RPN-driven is what left the ND dead: efis-cp.xml's own
//! `FBW_AIRLINER_Knob_ND_Template` rigs the ND mode and range knobs as
//! `ASOBO_GT_Knob_Infinite` whose turn code is
//! `'A32NX.FCU_EFIS_#SIDE#_#TYPE#_INC' (>F:KeyEvent)` -- real key events, the
//! only ones that panel sends. Without them `BaseFcuEfisPanelInputs`'
//! `efis_mode_knob_turns`/`efis_range_knob_turns` stayed 0 for the whole
//! flight, so the FCU never moved the EFIS discrete word and `prim.rs` kept
//! publishing one unchanging `A32NX_EFIS_{side}_ND_{MODE,RANGE}`: the knobs
//! turned in the cockpit and the picture never followed. They are served in
//! `afs_events.rs` as `fbw/event/A32NX_FCU_EFIS_{L,R}_{MODE,RANGE}_{INC,DEC}`.

use std::collections::HashSet;
use std::sync::Mutex;

use systems::simulation::{SimulatorReaderWriter, VariableRegistry};

use crate::afs_events::{self, Event};
use crate::extra_backend::XplaneIo;

pub const SWITCHES: &str = "sim/cockpit2/switches/";
const SPEEDBRAKE: &str = "sim/cockpit2/controls/speedbrake_ratio";
pub const RUDDER_TRIM: &str = "sim/cockpit2/controls/rudder_trim";
const TRANSPONDER_CODE: &str = "sim/cockpit2/radios/actuators/transponder_code";
const SIM_SPEED: &str = "sim/time/sim_speed";
/// MSFS's axis range for RUDDER_TRIM_SET.
const EVENT_RANGE: f64 = 16383.;
/// The armed position X-Plane's own speedbrake arm command uses.
const ARMED: f64 = -0.5;

/// MSFS's `LIGHT LANDING` index to X-Plane's landing lights. The converter
/// numbers X-Plane's landing lights in systems.cfg order, one per light node
/// of type 5 that is not an ambient effect (lights.rs `lightdefs`,
/// `lights_obj`); the package's systems.cfg (lines 147-199) gives
/// LIGHT_ASOBO_TAKEOFF_3 and _TAKEOFF_2 index 1, then LIGHT_ASOBO_LAND_1_LH,
/// _1_RH, _2_LH, _2_RH index 2.
pub fn landing_lights(index: usize) -> &'static [usize] {
    match index {
        1 => &[0, 1],
        2 => &[2, 3, 4, 5],
        _ => &[],
    }
}

/// Events with their arguments, from `TRIGGER_KEY_EVENT`.
static PENDING: Mutex<Vec<(String, Vec<f64>)>> = Mutex::new(Vec::new());

/// Queue `KeyEventManager.triggerKey(name, bypass, v0, v1, v2)`'s event with
/// `[v0, v1, v2]` (trailing unused values may be left out). For the script
/// runtime's `TRIGGER_KEY_EVENT`, which does not call it yet.
#[allow(dead_code)]
pub fn push(name: &str, args: &[f64]) {
    if let Ok(mut p) = PENDING.lock() {
        p.push((name.trim_start_matches("K:").to_string(), args.to_vec()));
    }
}

/// A script's `K:` write as event name and arguments: `K:NAME` or
/// `K:2:NAME` with its one value.
pub fn parse(name: &str, value: f64) -> (String, Vec<f64>) {
    let name = name.trim_start_matches("K:");
    let name = match name.split_once(':') {
        Some((count, rest)) if count.parse::<usize>().is_ok() => rest,
        _ => name,
    };
    (name.to_string(), vec![value])
}

fn toggle<V: VariableRegistry + SimulatorReaderWriter>(vars: &mut V, name: &str) {
    let id = vars.get_unprefixed(name.to_string());
    let v = vars.read(&id);
    vars.write(&id, if v != 0. { 0. } else { 1. });
}

fn set<V: VariableRegistry + SimulatorReaderWriter>(vars: &mut V, name: &str, value: f64) {
    let id = vars.get_unprefixed(name.to_string());
    vars.write(&id, value);
}

fn switch<X: XplaneIo>(xplane: &mut X, name: &str, index: Option<usize>, on: bool) {
    xplane.set(&format!("{SWITCHES}{name}"), index, if on { 1. } else { 0. });
}

/// BCD16 (one hex digit per decimal digit, as TransponderController.ts
/// packs it) to the four-digit code.
pub fn bcd16_to_code(bcd: u32) -> u32 {
    (0..4).rev().fold(0, |code, i| code * 10 + ((bcd >> (4 * i)) & 0xf))
}

/// The FCU and autothrust events a script can send by name.
fn afs_event(name: &str, value: f64) -> Option<Event> {
    Some(match name {
        "AUTOPILOT_OFF" => Event::AutopilotOff,
        "AP_MASTER" => Event::ApMaster,
        "AUTOPILOT_DISENGAGE_TOGGLE" => Event::AutopilotDisengageToggle,
        "A32NX.FCU_AP_1_PUSH" => Event::FcuAp1Push,
        "A32NX.FCU_AP_2_PUSH" => Event::FcuAp2Push,
        "A32NX.FCU_AP_DISCONNECT_PUSH" => Event::FcuApDisconnectPush,
        "A32NX.FCU_ATHR_PUSH" => Event::FcuAthrPush,
        "A32NX.FCU_ATHR_DISCONNECT_PUSH" => Event::FcuAthrDisconnectPush,
        "A32NX.FCU_FD_PUSH" => Event::FcuFdPush,
        "TOGGLE_FLIGHT_DIRECTOR" => Event::ToggleFlightDirector,
        "A32NX.FCU_SPD_INC" => Event::FcuSpdInc,
        "A32NX.FCU_SPD_DEC" => Event::FcuSpdDec,
        "A32NX.FCU_SPD_SET" => Event::FcuSpdSet(value),
        "A32NX.FCU_SPD_PUSH" => Event::FcuSpdPush,
        "A32NX.FCU_SPD_PULL" => Event::FcuSpdPull,
        "A32NX.FCU_SPD_MACH_TOGGLE_PUSH" => Event::FcuSpdMachTogglePush,
        "A32NX.FCU_HDG_INC" => Event::FcuHdgInc,
        "A32NX.FCU_HDG_DEC" => Event::FcuHdgDec,
        "A32NX.FCU_HDG_SET" => Event::FcuHdgSet(value),
        "A32NX.FCU_HDG_PUSH" => Event::FcuHdgPush,
        "A32NX.FCU_HDG_PULL" => Event::FcuHdgPull,
        "A32NX.FCU_TRK_FPA_TOGGLE_PUSH" => Event::FcuTrkFpaTogglePush,
        "A32NX.FCU_TRUE_TOGGLE_PUSH" => Event::FcuTrueTogglePush,
        "A32NX.FCU_ALT_INC" => Event::FcuAltInc,
        "A32NX.FCU_ALT_DEC" => Event::FcuAltDec,
        "A32NX.FCU_ALT_SET" => Event::FcuAltSet(value),
        "A32NX.FCU_ALT_PUSH" => Event::FcuAltPush,
        "A32NX.FCU_ALT_PULL" => Event::FcuAltPull,
        "A32NX.FCU_METRIC_ALT_TOGGLE_PUSH" => Event::FcuMetricAltTogglePush,
        "A32NX.FCU_VS_INC" => Event::FcuVsInc,
        "A32NX.FCU_VS_DEC" => Event::FcuVsDec,
        "A32NX.FCU_VS_SET" => Event::FcuVsSet(value),
        "A32NX.FCU_VS_PUSH" => Event::FcuVsPush,
        "A32NX.FCU_VS_PULL" => Event::FcuVsPull,
        "A32NX.FCU_LOC_PUSH" => Event::FcuLocPush,
        "A32NX.FCU_APPR_PUSH" => Event::FcuApprPush,
        "A32NX.FCU_ALT_BUTTON_PUSH" => Event::FcuAltButtonPush,
        "A32NX.FCU_EFIS_L_MODE_INC" => Event::FcuEfisModeTurn(0, 1),
        "A32NX.FCU_EFIS_L_MODE_DEC" => Event::FcuEfisModeTurn(0, -1),
        "A32NX.FCU_EFIS_L_RANGE_INC" => Event::FcuEfisRangeTurn(0, 1),
        "A32NX.FCU_EFIS_L_RANGE_DEC" => Event::FcuEfisRangeTurn(0, -1),
        "A32NX.FCU_EFIS_R_MODE_INC" => Event::FcuEfisModeTurn(1, 1),
        "A32NX.FCU_EFIS_R_MODE_DEC" => Event::FcuEfisModeTurn(1, -1),
        "A32NX.FCU_EFIS_R_RANGE_INC" => Event::FcuEfisRangeTurn(1, 1),
        "A32NX.FCU_EFIS_R_RANGE_DEC" => Event::FcuEfisRangeTurn(1, -1),
        "AUTO_THROTTLE_ARM" => Event::AutoThrottleArm,
        "AUTO_THROTTLE_DISCONNECT" => Event::AutoThrottleDisconnect,
        "A32NX.ATHR_RESET_DISABLE" => Event::AthrResetDisable,
        "A32NX.FMGC_DIR_TO_TRIGGER" => Event::FmgcDirToTrigger,
        // MSFS maps this stock event to its own A32NX_FMGC_MACH_MODE_ACTIVATE,
        // not the mach-mode-labelled name it looks like (SimConnectInterface.cpp:759-760).
        "AP_MANAGED_SPEED_IN_MACH_ON" => Event::ApManagedSpeedInMachOn,
        "AP_MANAGED_SPEED_IN_MACH_OFF" => Event::ApManagedSpeedInMachOff,
        "A32NX.FMS_PRESET_SPD_ACTIVATE" => Event::FmgcPresetSpdActivate,
        _ => return None,
    })
}

/// Events FlyByWire sends that are left alone on purpose (see the module doc).
const NOT_APPLIED: &[&str] = &[
    "FUELSYSTEM_VALVE_SET",
    "FUELSYSTEM_TRIGGER_TOGGLE",
    "FUELSYSTEM_TRIGGER_OFF",
    "FUELSYSTEM_JUNCTION_SET",
    "TUG_DISABLE",
    "A32NX.THROTTLE_MAPPING_SET_DEFAULTS",
    "A32NX.THROTTLE_MAPPING_SAVE_TO_FILE",
    "A32NX.THROTTLE_MAPPING_LOAD_FROM_FILE",
    "A32NX.THROTTLE_MAPPING_LOAD_FROM_LOCAL_VARIABLES",
];

/// Apply one event with MSFS's effect. False when this module does not know
/// the event (it may be a radio's, a door's or the tug's).
pub fn apply<V: VariableRegistry + SimulatorReaderWriter, X: XplaneIo>(vars: &mut V, xplane: &mut X, name: &str, args: &[f64]) -> bool {
    let arg = |i: usize| args.get(i).copied().unwrap_or(0.);
    let index = arg(0).round().max(0.) as usize;
    if let Some(n) = name.strip_prefix("TURBINE_IGNITION_SWITCH_SET").and_then(|n| n.parse::<usize>().ok()) {
        set(vars, &format!("TURB ENG IGNITION SWITCH EX1:{n}"), arg(0));
        return true;
    }
    if let Some(n) = name.strip_prefix("LIGHT_POTENTIOMETER_").and_then(|r| r.strip_suffix("_SET")).and_then(|n| n.parse::<usize>().ok()) {
        set(vars, &format!("LIGHT POTENTIOMETER:{n}"), arg(0) / 100.);
        return true;
    }
    if let Some(event) = afs_event(name, arg(0)) {
        afs_events::send(event);
        return true;
    }
    match name {
        "LIGHT_POTENTIOMETER_SET" => set(vars, &format!("LIGHT POTENTIOMETER:{index}"), arg(1) / 100.),
        "ELECTRICAL_CIRCUIT_TOGGLE" => toggle(vars, &format!("CIRCUIT SWITCH ON:{index}")),
        "ELECTRICAL_BUS_TO_CIRCUIT_CONNECTION_TOGGLE" => {
            toggle(vars, &format!("CIRCUIT CONNECTION ON:{}", arg(1).round().max(0.) as usize))
        }
        "FUELSYSTEM_VALVE_OPEN" | "FUELSYSTEM_VALVE_CLOSE" => {
            let open = if name.ends_with("OPEN") { 1. } else { 0. };
            if (1..=4).contains(&index) {
                set(vars, &format!("GENERAL ENG STARTER:{index}"), open);
            } else {
                set(vars, &format!("FUELSYSTEM VALVE SWITCH:{index}"), open);
            }
        }
        "CABIN_SEATBELTS_ALERT_SWITCH_TOGGLE" => toggle(vars, "CABIN SEATBELTS ALERT SWITCH"),
        "SPOILERS_ARM_SET" => {
            let current = xplane.get(SPEEDBRAKE, None).unwrap_or(0.);
            if arg(0) != 0. {
                if current >= 0. {
                    xplane.set(SPEEDBRAKE, None, ARMED);
                }
            } else if current < 0. {
                xplane.set(SPEEDBRAKE, None, 0.);
            }
        }
        "RUDDER_TRIM_SET" => xplane.set(RUDDER_TRIM, None, (arg(0) / EVENT_RANGE).clamp(-1., 1.)),
        "BEACON_LIGHTS_ON" => switch(xplane, "beacon_on", None, true),
        "BEACON_LIGHTS_OFF" => switch(xplane, "beacon_on", None, false),
        "NAV_LIGHTS_SET" => switch(xplane, "navigation_lights_on", None, arg(0) != 0.),
        "LOGO_LIGHTS_SET" => switch(xplane, "generic_lights_switch", Some(2), arg(0) != 0.),
        "TAXI_LIGHTS_ON" | "TAXI_LIGHTS_OFF" => {
            let on = name.ends_with("_ON");
            if index <= 1 {
                switch(xplane, "taxi_light_on", None, on);
            } else {
                switch(xplane, "generic_lights_switch", Some(0), on);
            }
        }
        "LANDING_LIGHTS_ON" | "LANDING_LIGHTS_OFF" => {
            let on = name.ends_with("_ON");
            for &i in landing_lights(index) {
                switch(xplane, "landing_lights_switch", Some(i), on);
            }
        }
        "XPNDR_SET" => xplane.set(TRANSPONDER_CODE, None, bcd16_to_code(arg(0).max(0.) as u32) as f64),
        "XPNDR_IDENT_ON" => xplane.command_once("sim/transponder/transponder_ident"),
        "SIM_RATE_INCR" | "SIM_RATE_DECR" => {
            let now = xplane.get(SIM_SPEED, None).unwrap_or(1.).max(1.);
            let next = if name.ends_with("INCR") { now * 2. } else { (now / 2.).max(1.) };
            xplane.set(SIM_SPEED, None, next);
        }
        "TOGGLE_JETWAY" => xplane.command_once("sim/ground_ops/jetway"),
        "TOGGLE_RAMPTRUCK" | "REQUEST_LUGGAGE" | "REQUEST_CATERING" | "REQUEST_FUEL_KEY" => {
            xplane.command_once("sim/ground_ops/service_plane")
        }
        "REQUEST_POWER_SUPPLY" => xplane.command_once("sim/ground_ops/toggle_gpu_request"),
        _ => return false,
    }
    true
}

/// The plugin's dispatcher: X-Plane access and what has been logged.
pub struct KeyEvents {
    xplane: crate::extra_backend::Xplane,
    logged: HashSet<String>,
}

impl KeyEvents {
    pub fn new(xplm: &'static crate::xp::Xplm) -> Self {
        Self { xplane: crate::extra_backend::Xplane::new(xplm), logged: HashSet::new() }
    }

    /// One script event, `K:NAME` or `K:n:NAME` with its value. True when it
    /// was this module's (applied, or known and deliberately left alone).
    #[cfg_attr(not(feature = "js"), allow(dead_code))]
    pub fn handle<V: VariableRegistry + SimulatorReaderWriter>(&mut self, vars: &mut V, name: &str, value: f64) -> bool {
        let (name, args) = parse(name, value);
        self.handle_args(vars, &name, &args)
    }

    pub fn handle_args<V: VariableRegistry + SimulatorReaderWriter>(&mut self, vars: &mut V, name: &str, args: &[f64]) -> bool {
        if apply(vars, &mut self.xplane, name, args) {
            return true;
        }
        if NOT_APPLIED.contains(&name) {
            if self.logged.insert(name.to_string()) {
                crate::log(&format!("key event {name} is not applied here (see key_events.rs)"));
            }
            return true;
        }
        false
    }

    /// The events queued with [`push`], for the plugin to offer to the other
    /// handlers when this module does not know one.
    pub fn take_pushed() -> Vec<(String, Vec<f64>)> {
        PENDING.lock().map(|mut p| std::mem::take(&mut *p)).unwrap_or_default()
    }

    /// Log an event nothing handled, once.
    pub fn unhandled(&mut self, name: &str) {
        if self.logged.insert(name.to_string()) {
            crate::log(&format!("key event {name} has no handler"));
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::aspects::test_vars::TestVars;
    use crate::extra_backend::sim::test_xplane::FakeXplane;

    #[test]
    fn script_forms_parse_into_name_and_arguments() {
        assert_eq!(parse("K:XPNDR_SET", 5.), ("XPNDR_SET".to_string(), vec![5.]));
        assert_eq!(parse("K:2:LOGO_LIGHTS_SET", 1.), ("LOGO_LIGHTS_SET".to_string(), vec![1.]));
        assert_eq!(parse("K:A32NX.FCU_ALT_SET", 36000.), ("A32NX.FCU_ALT_SET".to_string(), vec![36000.]));
    }

    #[test]
    fn events_have_msfs_effects() {
        let mut vars = TestVars::default();
        let mut xp = FakeXplane::default();
        // RmpStateController.ts:102: triggerKey(name, true, potentiometer, brightness).
        assert!(apply(&mut vars, &mut xp, "LIGHT_POTENTIOMETER_SET", &[80., 65.]));
        assert!((vars.value("LIGHT POTENTIOMETER:80") - 0.65).abs() < 1e-12);
        assert!(apply(&mut vars, &mut xp, "ELECTRICAL_CIRCUIT_TOGGLE", &[151.]));
        assert_eq!(vars.value("CIRCUIT SWITCH ON:151"), 1.);
        // 7700 in BCD16.
        assert!(apply(&mut vars, &mut xp, "XPNDR_SET", &[0x7700 as f64]));
        assert_eq!(xp.values[TRANSPONDER_CODE], 7700.);
        assert!(apply(&mut vars, &mut xp, "XPNDR_IDENT_ON", &[1.]));
        assert_eq!(xp.commands, vec!["sim/transponder/transponder_ident"]);
        xp.values.insert(SIM_SPEED.into(), 1.);
        apply(&mut vars, &mut xp, "SIM_RATE_INCR", &[1.]);
        apply(&mut vars, &mut xp, "SIM_RATE_INCR", &[1.]);
        assert_eq!(xp.values[SIM_SPEED], 4.);
        apply(&mut vars, &mut xp, "SIM_RATE_DECR", &[1.]);
        assert_eq!(xp.values[SIM_SPEED], 2.);
        assert!(!apply(&mut vars, &mut xp, "COM_RADIO_SET_HZ", &[118_000_000.]));
    }

    #[test]
    fn fcu_value_events_reach_the_fly_by_wire_inputs() {
        let mut vars = TestVars::default();
        let mut xp = FakeXplane::default();
        let _ = afs_events::take();
        assert!(apply(&mut vars, &mut xp, "A32NX.FCU_ALT_SET", &[36000.]));
        let inputs = afs_events::take();
        assert_eq!(inputs.autopilot.alt_set, 36000.);
    }

    /// MfdFmsFplnDirectTo.tsx's DIR TO INSERT button and FmcAircraftInterface.ts's
    /// managed-speed Mach/speed toggle and preset-speed activation
    /// (SimConnectInterface.cpp:2890-2911) must reach the exact
    /// SimInputAutopilot fields prim.rs reads as direct_to_nav_engage,
    /// fms_mach_mode_activate, fms_spd_mode_activate and preset_spd_mach_activate.
    #[test]
    fn fmgc_trigger_events_reach_the_fly_by_wire_inputs() {
        let mut vars = TestVars::default();
        let mut xp = FakeXplane::default();

        let _ = afs_events::take();
        assert!(apply(&mut vars, &mut xp, "A32NX.FMGC_DIR_TO_TRIGGER", &[0.]));
        assert!(afs_events::take().autopilot.dir_to_trigger);

        // MSFS maps this stock event to A32NX_FMGC_MACH_MODE_ACTIVATE, not a
        // same-named custom event - confirm we follow that, not the label.
        let _ = afs_events::take();
        assert!(apply(&mut vars, &mut xp, "AP_MANAGED_SPEED_IN_MACH_ON", &[1.]));
        let inputs = afs_events::take();
        assert!(inputs.autopilot.mach_mode_activate);
        assert!(!inputs.autopilot.spd_mode_activate);

        let _ = afs_events::take();
        assert!(apply(&mut vars, &mut xp, "AP_MANAGED_SPEED_IN_MACH_OFF", &[1.]));
        let inputs = afs_events::take();
        assert!(inputs.autopilot.spd_mode_activate);
        assert!(!inputs.autopilot.mach_mode_activate);

        let _ = afs_events::take();
        assert!(apply(&mut vars, &mut xp, "A32NX.FMS_PRESET_SPD_ACTIVATE", &[0.]));
        assert!(afs_events::take().autopilot.preset_spd_activate);
    }
}

/// [`afs_event`] for `afs_events`' own tests, which check that every knob
/// event is reachable by the MSFS name a script sends as well as by the
/// X-Plane command the cockpit binds.
#[cfg(test)]
pub(crate) fn afs_event_for_test(name: &str) -> Option<Event> {
    afs_event(name, 0.)
}
