//! What the MSFS events the cockpit fires become in X-Plane: a change to a
//! simulator variable (as MSFS itself or FlyByWire's MSFS glue makes it), a
//! command of FlyByWire's systems plugin (`fbw/event/*`), an X-Plane command
//! or dataref where X-Plane's own systems own the thing, or nothing where
//! the event has no effect FlyByWire's systems see. Each entry says where
//! the equivalence comes from.

/// The commands the systems plugin registers, `fbw/event/<suffix>`
/// (D:\A380\fbw-xp-systems\src\afs_events.rs, `COMMANDS`): the FCU, autopilot and
/// autothrust key events of FlyByWire's fly-by-wire module.
pub const PLUGIN_COMMANDS: &[&str] = &[
    "AUTOPILOT_OFF",
    "AP_MASTER",
    "AUTOPILOT_DISENGAGE_TOGGLE",
    "A32NX_FCU_AP_1_PUSH",
    "A32NX_FCU_AP_2_PUSH",
    "A32NX_FCU_AP_DISCONNECT_PUSH",
    "A32NX_FCU_ATHR_PUSH",
    "A32NX_FCU_ATHR_DISCONNECT_PUSH",
    "A32NX_FCU_FD_PUSH",
    "TOGGLE_FLIGHT_DIRECTOR",
    "A32NX_FCU_SPD_INC",
    "A32NX_FCU_SPD_DEC",
    "A32NX_FCU_SPD_PUSH",
    "A32NX_FCU_SPD_PULL",
    "A32NX_FCU_SPD_MACH_TOGGLE_PUSH",
    "A32NX_FCU_HDG_INC",
    "A32NX_FCU_HDG_DEC",
    "A32NX_FCU_HDG_PUSH",
    "A32NX_FCU_HDG_PULL",
    "A32NX_FCU_TRK_FPA_TOGGLE_PUSH",
    "A32NX_FCU_TRUE_TOGGLE_PUSH",
    "A32NX_FCU_ALT_INC",
    "A32NX_FCU_ALT_DEC",
    "A32NX_FCU_ALT_PUSH",
    "A32NX_FCU_ALT_PULL",
    "A32NX_FCU_METRIC_ALT_TOGGLE_PUSH",
    "A32NX_FCU_VS_INC",
    "A32NX_FCU_VS_DEC",
    "A32NX_FCU_VS_PUSH",
    "A32NX_FCU_VS_PULL",
    "A32NX_FCU_LOC_PUSH",
    "A32NX_FCU_APPR_PUSH",
    "A32NX_FCU_ALT_BUTTON_PUSH",
    "AUTO_THROTTLE_ARM",
    "AUTO_THROTTLE_DISCONNECT",
    "A32NX_ATHR_RESET_DISABLE",
];

/// The plugin command for a key event name (`A32NX.FCU_AP_1_PUSH`), if the
/// plugin registers one.
pub fn plugin_command(event: &str) -> Option<String> {
    let suffix = event.trim().replace('.', "_");
    PLUGIN_COMMANDS.contains(&suffix.as_str()).then(|| format!("fbw/event/{suffix}"))
}

/// A variable an event step sets: its name (`{}` takes an argument).
#[derive(Clone, Debug)]
pub struct Step {
    /// `A` or `L`.
    pub kind: &'static str,
    /// Variable name; `{}` is replaced by the `index` argument.
    pub var: String,
    pub index: Option<usize>,
    pub val: Val,
}

#[derive(Clone, Debug)]
pub enum Val {
    Const(f64),
    Arg(usize),
    /// An argument in percent, stored over 100 (MSFS keeps `LIGHT
    /// POTENTIOMETER:n` 0..1 while its SET events take percent).
    Percent(usize),
    /// 1 when the variable was 0, else 0.
    Not,
}

/// An X-Plane command, chosen by the event's arguments.
#[derive(Clone, Debug)]
pub enum Cmd {
    Fixed(String),
    /// `prefix` followed by argument `arg` as an integer.
    Numbered { prefix: String, arg: usize },
    /// One of these by argument `arg`.
    ByArg { arg: usize, names: Vec<(i64, String)> },
}

impl Cmd {
    pub fn name(&self, args: &[f64]) -> Option<String> {
        let a = |i: usize| args.get(i).copied().unwrap_or(0.0).round() as i64;
        match self {
            Cmd::Fixed(n) => Some(n.clone()),
            Cmd::Numbered { prefix, arg } => Some(format!("{prefix}{}", a(*arg))),
            Cmd::ByArg { arg, names } => names.iter().find(|(k, _)| *k == a(*arg)).map(|(_, n)| n.clone()),
        }
    }

    /// Lua expression for the name, with the arguments in `a0`, `a1`...
    pub fn lua(&self) -> String {
        match self {
            Cmd::Fixed(n) => format!("{n:?}"),
            Cmd::Numbered { prefix, arg } => format!("{prefix:?} .. string.format(\"%d\", a{arg})"),
            Cmd::ByArg { arg, names } => {
                let t: Vec<String> = names.iter().map(|(k, n)| format!("[{k}] = {n:?}")).collect();
                format!("({{{}}})[math.floor(a{arg} + 0.5)]", t.join(", "))
            }
        }
    }

    pub fn args(&self) -> usize {
        match self {
            Cmd::Fixed(_) => 0,
            Cmd::Numbered { arg, .. } | Cmd::ByArg { arg, .. } => arg + 1,
        }
    }
}

#[derive(Clone, Debug)]
pub enum KAction {
    /// Simulator or FlyByWire variables it changes.
    Vars(Vec<Step>),
    /// An X-Plane command (the plugin's or X-Plane's own), fired once.
    Command(Cmd),
    /// An X-Plane dataref it sets: dataref chosen by argument `index`,
    /// value argument `value` times `scale`.
    Dataref { index: usize, drefs: Vec<(i64, String)>, value: usize, scale: f64 },
    /// No effect FlyByWire's systems or X-Plane see (why).
    Nothing(&'static str),
}

/// Millibars per inch of mercury.
const MB_PER_INHG: f64 = 33.863_886;

/// The MSFS key events the A380X's cockpit fires, as what they do.
pub fn k_event(name: &str) -> Option<KAction> {
    let step = |kind: &'static str, var: &str, index: Option<usize>, val: Val| Step { kind, var: var.to_string(), index, val };
    let a = |var: &str, index: Option<usize>, val: Val| step("A", var, index, val);
    let trailing = |prefix: &str| -> Option<usize> { name.strip_prefix(prefix).and_then(|n| n.parse().ok()) };
    if let Some(c) = plugin_command(name) {
        return Some(KAction::Command(Cmd::Fixed(c)));
    }
    // The stock Asobo altitude knob/selector events FBW's own compiled
    // interface also subscribes to, for compatibility with the default
    // Asobo altitude-knob mechanism the A380X's FCU_Altitude_Knob templates
    // still use for their plain rotate (SimConnectInterface.cpp: AP_ALT_VAR_INC
    // / AP_ALT_VAR_DEC set fcuAfsPanelInputs.alt_knob.turns exactly like
    // A32NX.FCU_ALT_INC/_DEC; AP_ALT_HOLD_ON/_OFF set alt_knob.pushed/pulled
    // exactly like A32NX.FCU_ALT_PUSH/_PULL).
    let alias = match name {
        "AP_ALT_VAR_INC" => Some("A32NX.FCU_ALT_INC"),
        "AP_ALT_VAR_DEC" => Some("A32NX.FCU_ALT_DEC"),
        "AP_ALT_HOLD_ON" => Some("A32NX.FCU_ALT_PUSH"),
        "AP_ALT_HOLD_OFF" => Some("A32NX.FCU_ALT_PULL"),
        _ => None,
    };
    if let Some(a) = alias {
        return plugin_command(a).map(|c| KAction::Command(Cmd::Fixed(c)));
    }
    if let Some(n) = trailing("TOGGLE_STARTER") {
        return Some(KAction::Vars(vec![a(&format!("GENERAL ENG STARTER:{n}"), None, Val::Not)]));
    }
    if name == "TURBINE_IGNITION_SWITCH_SET" {
        return Some(KAction::Vars((1..=super::bind::ENGINES).map(|e| a(&format!("TURB ENG IGNITION SWITCH EX1:{e}"), None, Val::Arg(0))).collect()));
    }
    if let Some(n) = trailing("TURBINE_IGNITION_SWITCH_SET") {
        return Some(KAction::Vars(vec![a(&format!("TURB ENG IGNITION SWITCH EX1:{n}"), None, Val::Arg(0))]));
    }
    if let Some(n) = trailing("TOGGLE_ALTERNATOR") {
        // The simulator's switch; FlyByWire's glue (and the plugin's aspects)
        // copy it into OVHD_ELEC_ENG_GEN_<n>_PB_IS_ON every tick
        // (a380_systems_wasm lib.rs:590-593), so that is not written here.
        return Some(KAction::Vars(vec![a(&format!("GENERAL ENG MASTER ALTERNATOR:{n}"), None, Val::Not)]));
    }
    if let Some(n) = trailing("ANTI_ICE_TOGGLE_ENG") {
        return Some(KAction::Vars(vec![a(&format!("ENG ANTI ICE:{n}"), None, Val::Not)]));
    }
    // LIGHT_POTENTIOMETER_<n>_SET: LIGHT_POTENTIOMETER_SET with the index in
    // its name (the console light switches set 8 and 9 this way).
    if let Some(n) = name.strip_prefix("LIGHT_POTENTIOMETER_").and_then(|r| r.strip_suffix("_SET")).and_then(|n| n.parse::<usize>().ok()) {
        return Some(KAction::Vars(vec![a(&format!("LIGHT POTENTIOMETER:{n}"), None, Val::Percent(0))]));
    }
    Some(match name {
        // The A380X's FBW_Airbus_FCU_Altitude_Knob turn computes the new
        // altitude itself (reading X-Plane's own AUTOPILOT ALTITUDE LOCK
        // VAR:3, mapped below) and writes it with this stock MSFS event,
        // exactly as FlyByWire's SimConnectInterface.cpp:3007
        // (`simInputAutopilot.ALT_set = data0`) reads it: fed to the PRIMs'
        // sim_input.alt the same way A32NX.FCU_ALT_SET is (prim.rs
        // update_with, port-specific pending var since X-Plane commands
        // carry no argument).
        "AP_ALT_VAR_SET_ENGLISH" => KAction::Vars(vec![a("XP_FCU_ALT_SET_PENDING", None, Val::Arg(0))]),
        // The stock Asobo altitude-increment selector (100/1000 ft):
        // SimConnectInterface.cpp:2402-2406 treats AP_ALT_HOLD identically to
        // A32NX.FCU_ALT_INCREMENT_TOGGLE, flipping A32NX_FCU_ALT_INCREMENT_1000.
        "AP_ALT_HOLD" => KAction::Vars(vec![a("A32NX_FCU_ALT_INCREMENT_1000", None, Val::Not)]),
        "FUELSYSTEM_VALVE_OPEN" => KAction::Vars(vec![a("FUELSYSTEM VALVE SWITCH:{}", Some(0), Val::Const(1.0))]),
        "FUELSYSTEM_VALVE_CLOSE" => KAction::Vars(vec![a("FUELSYSTEM VALVE SWITCH:{}", Some(0), Val::Const(0.0))]),
        "FUELSYSTEM_VALVE_TOGGLE" => KAction::Vars(vec![a("FUELSYSTEM VALVE SWITCH:{}", Some(0), Val::Not)]),
        "FUELSYSTEM_PUMP_TOGGLE" => KAction::Vars(vec![a("FUELSYSTEM PUMP SWITCH:{}", Some(0), Val::Not)]),
        "LIGHT_POTENTIOMETER_SET" => KAction::Vars(vec![a("LIGHT POTENTIOMETER:{}", Some(0), Val::Percent(1))]),
        "CABIN_SEATBELTS_ALERT_SWITCH_TOGGLE" => KAction::Vars(vec![a("CABIN SEATBELTS ALERT SWITCH", None, Val::Not)]),
        "PITOT_HEAT_ON" => KAction::Vars(vec![a("PITOT HEAT", None, Val::Const(1.0))]),
        "PITOT_HEAT_OFF" => KAction::Vars(vec![a("PITOT HEAT", None, Val::Const(0.0))]),
        "WINDSHIELD_DEICE_ON" => KAction::Vars(vec![a("WINDSHIELD DEICE SWITCH", None, Val::Const(1.0))]),
        "WINDSHIELD_DEICE_OFF" => KAction::Vars(vec![a("WINDSHIELD DEICE SWITCH", None, Val::Const(0.0))]),
        "TOGGLE_STRUCTURAL_DEICE" => KAction::Vars(vec![a("STRUCTURAL DEICE SWITCH", None, Val::Not)]),
        "SPOILERS_ARM_ON" => KAction::Vars(vec![a("SPOILERS ARMED", None, Val::Const(1.0))]),
        "SPOILERS_ARM_OFF" => KAction::Vars(vec![a("SPOILERS ARMED", None, Val::Const(0.0))]),
        "GEAR_UP" => KAction::Vars(vec![a("GEAR HANDLE POSITION", None, Val::Const(0.0))]),
        "GEAR_DOWN" => KAction::Vars(vec![a("GEAR HANDLE POSITION", None, Val::Const(1.0))]),
        // CTRL-007: the real, read variable is A32NX_PARK_BRAKE_LEVER_POS
        // (FBW_LANDING_GEAR_Switch_ParkingBrake_SubTemplate,
        // A32NX_Interior_Handling.xml; fbw-xp-systems handling.rs:335 reads
        // it as PARK_BRAKE_LEVER_POS), not MSFS's own BRAKE PARKING POSITION,
        // which nothing downstream reads.
        "PARKING_BRAKES" => KAction::Vars(vec![a("A32NX_PARK_BRAKE_LEVER_POS", None, Val::Not)]),
        "ANTISKID_BRAKES_TOGGLE" => KAction::Vars(vec![a("ANTISKID BRAKES ACTIVE", None, Val::Not)]),
        // State, then the cabin light index (`1 0 (>K:2:CABIN_LIGHTS_SET)`
        // in A380_COCKPIT.xml's VARIABLE_MAPPING update); its circuits are on
        // buses the infinite battery feeds, so the light is on when set.
        "CABIN_LIGHTS_SET" => KAction::Vars(vec![a("LIGHT CABIN:{}", Some(1), Val::Arg(0)), a("LIGHT CABIN ON:{}", Some(1), Val::Arg(0))]),
        // The APU generator pushbuttons: the simulator's switch, which
        // FlyByWire's glue (and the plugin's aspects) copy into
        // OVHD_ELEC_APU_GEN_<n>_PB_IS_ON every tick (a380_systems_wasm
        // lib.rs:580-584).
        "APU_GENERATOR_SWITCH_TOGGLE" => KAction::Vars(vec![a("APU GENERATOR SWITCH:{}", Some(0), Val::Not)]),
        // The bus-to-circuit connection (fuel pump pushbuttons): arguments
        // bus, circuit; the pushbuttons read it back as `1 (>A:BUS LOOKUP
        // INDEX) (A:CIRCUIT CONNECTION ON:<circuit>)` (A32NX_Interior_Misc.xml
        // FBW_Airbus_Fuel_Pump). Every cockpit use is bus 1.
        "ELECTRICAL_BUS_TO_CIRCUIT_CONNECTION_TOGGLE" => KAction::Vars(vec![a("CIRCUIT CONNECTION ON:{}", Some(1), Val::Not)]),
        // Circuit switch and power setting (the wiper switches, circuits
        // 141 and 143 in systems.cfg); read back as CIRCUIT SWITCH ON and
        // CIRCUIT POWER SETTING.
        "ELECTRICAL_CIRCUIT_TOGGLE" => KAction::Vars(vec![a("CIRCUIT SWITCH ON:{}", Some(0), Val::Not)]),
        "ELECTRICAL_CIRCUIT_POWER_SETTING_SET" => KAction::Vars(vec![a("CIRCUIT POWER SETTING:{}", Some(0), Val::Arg(1))]),
        // FlyByWire's glue sets the simulator's APU bleed itself every tick
        // from APU_BLEED_AIR_VALVE_OPEN (systems_wasm electrical.rs:58-80);
        // the pushbutton's own L:A32NX_OVHD_PNEU_APU_BLEED_PB_IS_ON is what
        // the systems read.
        "APU_BLEED_AIR_SOURCE_TOGGLE" => KAction::Nothing("the MSFS APU bleed, which FlyByWire's glue drives from APU_BLEED_AIR_VALVE_OPEN"),
        // X-Plane's own systems: doors, altimeter settings.
        "TOGGLE_AIRCRAFT_EXIT" => KAction::Command(Cmd::Numbered { prefix: "sim/flight_controls/door_toggle_".into(), arg: 0 }),
        // Manual pitch/rudder trim switches: FBW's own PRIM/SEC read these as
        // raw discretes (SimInputPitchTrim/SimInputRudderTrim,
        // SimConnectData.h:147-155), which decide manual trim feel in
        // alternate/direct law (a380_systems fire_and_smoke_protection.rs
        // neighbour trimmable_horizontal_stabilizer.rs, rudder.rs). Pulsed
        // through port-specific one-tick datarefs prim.rs's TrimPulses reads
        // (not X-Plane's own pitch_trim_up/down commands: the THS/rudder
        // surfaces already come from FBW's own hydraulics simulation with
        // sim/operation/override/override_control_surfaces set, so X-Plane's
        // native trim commands would move a dataref nothing downstream uses).
        "ELEV_TRIM_DN" => KAction::Vars(vec![a("XP_PITCH_TRIM_DOWN_PULSE", None, Val::Const(1.0))]),
        "ELEV_TRIM_UP" => KAction::Vars(vec![a("XP_PITCH_TRIM_UP_PULSE", None, Val::Const(1.0))]),
        "RUDDER_TRIM_LEFT" => KAction::Vars(vec![a("XP_RUDDER_TRIM_LEFT_PULSE", None, Val::Const(1.0))]),
        "RUDDER_TRIM_RIGHT" => KAction::Vars(vec![a("XP_RUDDER_TRIM_RIGHT_PULSE", None, Val::Const(1.0))]),
        "RUDDER_TRIM_RESET" => KAction::Vars(vec![a("XP_RUDDER_TRIM_RESET_PULSE", None, Val::Const(1.0))]),
        // Altimeter 1 is the captain's, 3 the standby (the ISIS knob's).
        "KOHLSMAN_INC" => KAction::Command(Cmd::ByArg {
            arg: 0,
            names: vec![(1, "sim/instruments/barometer_up".into()), (3, "sim/instruments/barometer_stby_up".into())],
        }),
        "KOHLSMAN_DEC" => KAction::Command(Cmd::ByArg {
            arg: 0,
            names: vec![(1, "sim/instruments/barometer_down".into()), (3, "sim/instruments/barometer_stby_down".into())],
        }),
        // Millibars times 16, then the altimeter index.
        "KOHLSMAN_SET" => KAction::Dataref {
            index: 1,
            drefs: vec![
                (1, "sim/cockpit2/gauges/actuators/barometer_setting_in_hg_pilot".into()),
                (3, "sim/cockpit2/gauges/actuators/barometer_setting_in_hg_stby".into()),
            ],
            value: 0,
            scale: 1.0 / 16.0 / MB_PER_INHG,
        },
        _ => return None,
    })
}

/// X-Plane datarefs standing for simulator variables X-Plane owns: (dataref,
/// factor from the dataref's unit to the variable's).
pub fn xplane_var(key: &str) -> Option<(&'static str, f64)> {
    match key {
        "A:KOHLSMAN SETTING MB:3" => Some(("sim/cockpit2/gauges/actuators/barometer_setting_in_hg_stby", MB_PER_INHG)),
        "P:Absolute time" | "E:SIMULATION TIME" | "E:ABSOLUTE TIME" => Some(("sim/time/total_running_time_sec", 1.0)),
        // FBW_Airbus_FCU_Altitude_Knob_SubTemplate's turn reads this stock
        // MSFS variable to compute its new value (A32NX_Interior_Autopilot.xml
        // legacy generated fcu.xml source: `(A:AUTOPILOT ALTITUDE LOCK VAR:3,
        // feet)`); FBW's FCU publishes its selected altitude as
        // A32NX_FCU_AFS_DISPLAY_ALT_VALUE (prim.rs update_fcu_afs_lvars,
        // cpp:2481-2511), which the knob reads as its current value.
        "A:AUTOPILOT ALTITUDE LOCK VAR:3" => Some(("fbw/A32NX_FCU_AFS_DISPLAY_ALT_VALUE", 1.0)),
        // Overhead EXT LT switches (bind.rs's ASOBO_LIGHTING_Switch_* arm):
        // these ASOBO base-game templates have no RPN of their own to read
        // (they are hard-coded MSFS interaction logic), so bind.rs writes
        // straight to X-Plane's own switch datarefs under invented but
        // MSFS-simvar-shaped keys, the same real names the stock A380X's own
        // FBW_Light_Sync condition already reads for the beacon
        // (`(A:LIGHT BEACON, bool)`, A380_Cockpit_Behavior.xml). The indexed
        // ones (:1/:2) match both that XML's own SIMVAR_INDEX values and
        // fbw-xp-systems lights.rs's circuit table (CIRCUIT_LIGHT_TAXI:1 is
        // the nose light, :2/:3 the runway-turnoff pair; CIRCUIT_LIGHT_
        // LANDING:1 is the nose landing light). `[n]` addresses one element
        // of an array dataref (X-Plane's own 0-based element index); rig.rs's
        // xref() routes any bracketed name through SASL's auto-typed
        // globalProperty(), which parses "[n]" itself (see W130) -- no plugin
        // change is needed, fbw-xp-systems lights.rs already reads/writes
        // these exact elements every tick (GatedSwitch::step, apply_group).
        "A:LIGHT STROBE" => Some(("sim/cockpit2/switches/strobe_lights_on", 1.0)),
        "A:LIGHT NAV" => Some(("sim/cockpit2/switches/navigation_lights_on", 1.0)),
        "A:LIGHT LOGO" => Some(("sim/cockpit2/switches/generic_lights_switch[2]", 1.0)),
        "A:LIGHT TAXI:1" => Some(("sim/cockpit2/switches/taxi_light_on", 1.0)),
        "A:LIGHT TAXI:2" => Some(("sim/cockpit2/switches/generic_lights_switch[0]", 1.0)),
        "A:LIGHT LANDING:1" => Some(("sim/cockpit2/switches/landing_lights_switch[0]", 1.0)),
        // FBW_Airbus_Wiper (A32NX_Exterior.xml, the WipersLeft/WipersRight
        // Components in A380_Cockpit_Behavior.xml) times its wiper sweep
        // against `(A:ANIMATION DELTA TIME, seconds)` every frame; nothing
        // publishes a "fbw/ANIMATION_DELTA_TIME" systems dataref (it is not a
        // click-settable variable), so without this the generic `A:` mapping
        // (bind::home) would route it at a dataref that always reads 0 and
        // the sweep would never advance. X-Plane's own frame period is the
        // same quantity (rig.rs's exterior Lua preamble already reads it this
        // way as `x_dt`, sim/operation/misc/frame_rate_period, rig.rs:973).
        "A:ANIMATION DELTA TIME" => Some(("sim/operation/misc/frame_rate_period", 1.0)),
        _ => None,
    }
}

/// `H:` events of the package's FCU instrument that it forwards unchanged as
/// a key event the plugin takes (html_ui A380X/FCU/fcu.js, AutopilotManager
/// onEvent 47649-47662 and AltitudeManager onHEvent 47557-47564). The rest
/// (speed, heading, V/S knobs, SPD/MACH, TRUE/MAG) keep their selection in
/// that JavaScript, which X-Plane does not run.
pub fn h_event(name: &str) -> Option<Cmd> {
    let key = match name.trim() {
        "A320_Neo_FCU_AP_1_PUSH" => "A32NX.FCU_AP_1_PUSH",
        "A320_Neo_FCU_AP_2_PUSH" => "A32NX.FCU_AP_2_PUSH",
        "A320_Neo_FCU_LOC_PUSH" => "A32NX.FCU_LOC_PUSH",
        "A320_Neo_FCU_APPR_PUSH" => "A32NX.FCU_APPR_PUSH",
        "A320_Neo_FCU_ALT_PUSH" => "A32NX.FCU_ALT_PUSH",
        "A320_Neo_FCU_ALT_PULL" => "A32NX.FCU_ALT_PULL",
        // The SPD/MACH, HDG/TRK and V/S/FPA knobs (FBW_AUTOPILOT_Knob_SpeedMach_Template,
        // FBW_Airbus_Autopilot_Knob_Heading_SubTemplate,
        // FBW_Airbus_Autopilot_Knob_VerticalSpeed_Template: A32NX_Interior_FCU.xml,
        // A32NX_Interior_Autopilot.xml) fire these directly, meant for the
        // package's own html_ui FCU/fcu.js (AutopilotManager/AltitudeManager),
        // which X-Plane does not run; routed straight to the same
        // fbw/event/A32NX_FCU_* commands that JS instrument would have ended
        // up sending, so the physical knob's turn/push/pull work without it.
        "A320_Neo_FCU_SPEED_INC" => "A32NX.FCU_SPD_INC",
        "A320_Neo_FCU_SPEED_DEC" => "A32NX.FCU_SPD_DEC",
        "A320_Neo_FCU_SPEED_PUSH" => "A32NX.FCU_SPD_PUSH",
        "A320_Neo_FCU_SPEED_PULL" => "A32NX.FCU_SPD_PULL",
        "A320_Neo_FCU_HDG_INC_HEADING" | "A320_Neo_FCU_HDG_INC_TRACK" => "A32NX.FCU_HDG_INC",
        "A320_Neo_FCU_HDG_DEC_HEADING" | "A320_Neo_FCU_HDG_DEC_TRACK" => "A32NX.FCU_HDG_DEC",
        "A320_Neo_FCU_HDG_PUSH" => "A32NX.FCU_HDG_PUSH",
        "A320_Neo_FCU_HDG_PULL" => "A32NX.FCU_HDG_PULL",
        "A320_Neo_FCU_VS_INC_VS" | "A320_Neo_FCU_VS_INC_FPA" => "A32NX.FCU_VS_INC",
        "A320_Neo_FCU_VS_DEC_VS" | "A320_Neo_FCU_VS_DEC_FPA" => "A32NX.FCU_VS_DEC",
        "A320_Neo_FCU_VS_PULL" => "A32NX.FCU_VS_PULL",
        // Every other H: event is for FlyByWire's instruments, which the
        // systems plugin runs: its `fbw/hevent/<name>` command delivers it
        // (fbw-xp-systems js_bridge.rs, tools/js-build/hevents.txt).
        other if !other.is_empty() => return Some(Cmd::Fixed(format!("fbw/hevent/{other}"))),
        _ => return None,
    };
    plugin_command(key).map(Cmd::Fixed)
}
