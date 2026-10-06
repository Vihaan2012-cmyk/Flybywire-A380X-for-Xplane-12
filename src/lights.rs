//! #5 (LIGHT-001, XP-005, LGT-001): exterior, cockpit and cabin lights
//! powered from FlyByWire's electrical buses via the embedded systems.cfg
//! light circuits, on [`circuits::Circuits`]'s general model.
//! #34 (ICE-002): window heat and wipers, as an electrical load and X-Plane's
//! own rain-on-glass datarefs.
//!
//! **The problem this module solves.** key_events.rs's exterior light events
//! (`BEACON_LIGHTS_ON`, `NAV_LIGHTS_SET`, ...) and the cockpit's own click
//! code write straight into X-Plane's own switch datarefs
//! (`sim/cockpit2/switches/beacon_on`, `navigation_lights_on`,
//! `generic_lights_switch`, `taxi_light_on`, `landing_lights_switch`,
//! `strobe_lights_on`). The converted model's exterior lights are X-Plane's
//! own named OBJ8 light types (`airplane_beacon_*`, `airplane_nav_*`, ...,
//! msfs2xp-aircraft lights.rs `lights_obj`), which X-Plane's own engine
//! visually drives straight off those same switch datarefs — so the switch
//! *is* the light's visibility, with nothing in between for this module to
//! gate. `handling/aspects.rs:783` documents the same gap on the handling
//! side (`TOGGLE_BEACON_LIGHTS` is explicitly not claimed there either).
//!
//! Writing `0` into the dataref whenever the circuit is unpowered would put
//! the light out, but it would also overwrite the pilot's switch position:
//! flipping a light on with the bus dead, then restoring the bus, would
//! leave the light dark until the switch was re-clicked, because the dataref
//! itself carries no separate memory of "commanded" versus "actually lit".
//! [`GatedSwitch`] fixes this the way fuel.rs's `OUTSIDE_CHANGE_KG` already
//! detects an outside change to X-Plane's fuel tanks: it remembers the value
//! *this module itself* last wrote, and treats any different value found next
//! tick as a new command from the cockpit (a click, a script, a preset),
//! keeping that commanded position in memory even while the circuit is dead,
//! and reapplying it the instant power returns — so a click still toggles
//! the switch, and the light still goes dark on bus/breaker loss, both at
//! once.
//!
//! One gap this leaves: turning the switch *off* while already dark writes
//! the same `0` this module itself is already forcing, so that particular
//! click cannot be told apart from the forced-dark state; the light then
//! relights on power return needing one more click to turn off. Documented
//! at [`GatedSwitch`]'s own test for it rather than silently assumed away.
//!
//! **Circuit groups.** Multiple systems.cfg circuits (buses.2/.3 redundancy)
//! drive one X-Plane switch, so a light stays lit as long as any one of its
//! circuits has power (`Circuits::any_powered`) — real dual-bus redundancy,
//! simplified by the model having only one visual per group:
//!
//! | Group | X-Plane switch | systems.cfg circuits |
//! |---|---|---|
//! | Beacon | `beacon_on` | `CIRCUIT_LIGHT_BEACON` (2) |
//! | Nav | `navigation_lights_on` | `CIRCUIT_LIGHT_NAV` (4) |
//! | Strobe | `strobe_lights_on` | `CIRCUIT_LIGHT_STROBE` (3) |
//! | Logo | `generic_lights_switch:2` | `CIRCUIT_LIGHT_LOGO` (2) |
//! | Wing | `generic_lights_switch:1` | `CIRCUIT_LIGHT_WING` (2) |
//! | Taxi (nose) | `taxi_light_on` | `CIRCUIT_LIGHT_TAXI:1` |
//! | Taxi (turn-off) | `generic_lights_switch:0` | `CIRCUIT_LIGHT_TAXI:2/:3` |
//! | Landing (nose) | `landing_lights_switch:0,1` | `CIRCUIT_LIGHT_LANDING:1` |
//! | Landing (main) | `landing_lights_switch:2..5` | `CIRCUIT_LIGHT_LANDING:2/:3` |
//!
//! (msfs2xp-aircraft lights.rs's `airplane_generic_*` indices for turn-off/
//! wing/logo; key_events.rs's `landing_lights` for the landing groups;
//! systems.cfg:375-394 for the circuits themselves.)
//!
//! Cockpit/cabin light circuits (`CIRCUIT_LIGHT_PANEL`/`_PEDESTAL`/
//! `_GLARESHIELD`/`_CABIN`) have no X-Plane switch dataref of their own —
//! their brightness is the `LIGHT POTENTIOMETER` array, read directly by the
//! model's own emissive RPN in at least one place already (`SCREEN_BACKLIGHT_
//! AUTOPILOT`'s emissive multiplies `LIGHT POTENTIOMETER:87` by `A32NX_ELEC_
//! DC_ESS_BUS_IS_POWERED`/`DC_2_BUS_IS_POWERED` directly, docs/analysis/
//! cockpit-study-cbs.md CTRL-003/LGT-002). Rather than duplicate that
//! per-panel in Rust, this module publishes one `LIGHT CIRCUIT POWERED:n`
//! per light circuit (exterior and interior alike) so any emissive/backlight
//! RPN, or a future converter change, can read a single already-computed
//! power state instead of re-deriving it from the buses each time; see the
//! report for the exact converter hook for #31's interior dimming knobs.
//!
//! **Window heat and wipers (#34).** No FBW source and no systems.cfg
//! circuit exists for window heat (grep across `fbw-common`/`a380_systems`
//! and this file's own systems.cfg both come up empty), so it is modelled
//! as a new plugin switch, gated by the same two buses the *wiper* circuits
//! use (`CIRCUIT_XML:17`/`:18`, "WipersLeft"/"WipersRight",
//! systems.cfg:533,535) — the nearest real, cited circuits in the same
//! cfg region, not a fabricated one. Wipers themselves are real circuits and
//! drive X-Plane's own rain-on-glass system (`sim/cockpit2/switches/
//! wiper_speed_switch`, `sim/flightmodel2/misc/wiper_angle_deg`,
//! DataRefs.txt); window heat drives X-Plane's own windshield de-ice switch
//! (`sim/cockpit2/ice/ice_window_heat_on`, DataRefs.txt).
//!
//! **Ice protection output wiring (#30/#34).** FlyByWire's own wing/engine
//! anti-ice and the ADIRS probe-heat computer (physics/adirs.rs) already
//! compute the right causal state (PNEU_WING_ANTI_ICE_SYSTEM_ON,
//! ENG ANTI ICE:1..4, the per-ADIRU PROBE_HEAT_LOAD_W:1..3 shared contract),
//! but nothing then carried that state to X-Plane's own icing model, so the
//! overhead switches had no effect on frm_ice/inlet_ice/aoa_ice/pitot_ice at
//! all: pressing ENG/WING ANTI ICE only moved an fbw/-namespaced dataref
//! nobody reads (lib.rs's mapping() has no entry for "ENG ANTI ICE" or
//! "STRUCTURAL DEICE SWITCH", so aspects.rs's mirroring of those MSFS-style
//! names lands on an orphaned slot; DataRefs.txt has the real X-Plane ones).
//! This module now forwards that already-computed state onto X-Plane's real
//! per-engine/per-wing/per-probe switches every tick (DataRefs.txt:
//! cowling_thermal_anti_ice_per_engine, ice_surface_hot_bleed_air_left_on/
//! _right_on -- the A380's wing anti-ice is hot bleed air, not electric
//! boots, so the bleed-air switches are the physically correct ones, not
//! ice_surfce_heat_* -- ice_AOA_heat_on[_copilot|_stby],
//! ice_TAT_heat_on[_copilot|_stby], ice_pitot_heat_on_pilot/copilot/standby,
//! ice_static_heat_on_pilot/copilot/standby). ADIRU 1/2/3 are pilot/copilot/
//! standby, matching adirs.rs's own numbering. This is a one-way read of an
//! already-published value (the "clean interface" adirs.rs's own doc
//! comment invites: "the electrical workstream reads this as a bus
//! consumer" -- ice protection is just another reader), not an edit to
//! physics/adirs.rs.
//!
//! **Rain repellent (#30).** FBW's own A380_Cockpit_Behavior.xml
//! (PUSH_OVHD_RAINRPLNTL/RAINRPNLTR) sets L:A32NX_RAIN_REPELLENT_LEFT_ON/
//! _RIGHT_ON while held, with a tooltip that says "(Inop.)" in FBW's own
//! source -- no system in FlyByWire ever reacts to it, and no SEQ_POWERED/
//! circuit is given for those buttons either (unlike the probes/window heat
//! button's cited A32NX_ELEC_AC_2_BUS_IS_POWERED), so there is no real
//! circuit to gate this on without inventing one. This module forwards the
//! two hold-simvars straight to X-Plane's own sim/cockpit2/switches/
//! rain_repellent_switch[0|1] (DataRefs.txt), the real, native "rain
//! repellent other than wipers" system -- turning FBW's inert placeholder
//! into a working one using X-Plane's own physics, ungated because FBW
//! gives nothing to gate it on.

use systems::simulation::{SimulatorReaderWriter, VariableIdentifier, VariableRegistry};

use crate::circuits::Circuits;
use crate::xp::{DataRef, Xplm};
use crate::Vars;

const SWITCHES: &str = "sim/cockpit2/switches/";

/// Tracks a commanded switch position across the power gating that
/// overwrites the same X-Plane dataref the model's light geometry reads (see
/// the module doc). `step` is pure so it is unit-tested without X-Plane.
#[derive(Default, Clone, Copy, Debug, PartialEq)]
pub struct GatedSwitch {
    /// The value this module itself last wrote, if any.
    last_applied: Option<f64>,
    /// The switch's last known commanded position.
    commanded: f64,
}

impl GatedSwitch {
    /// `raw` is the dataref's current value; `powered` is whether the
    /// circuit feeding it is live right now. Returns the value to write back.
    pub fn step(&mut self, raw: f64, powered: bool) -> f64 {
        if self.last_applied != Some(raw) {
            // Nobody but this module wrote the value we expected: the
            // cockpit (or a script, or a preset) changed it since last tick.
            self.commanded = raw;
        }
        let out = if powered { self.commanded } else { 0. };
        self.last_applied = Some(out);
        out
    }
}

fn apply_scalar(xplm: &Xplm, dataref: Option<DataRef>, gate: &mut GatedSwitch, powered: bool) {
    let Some(d) = dataref else { return };
    let raw = xplm.get_f(d) as f64;
    let out = gate.step(raw, powered);
    xplm.set_f(d, out as f32);
}

/// Applies one gate to a group of indices in an array dataref that always
/// move together (one cockpit switch feeding several X-Plane light slots,
/// e.g. the four `landing_lights_switch` indices behind "LIGHT LANDING:2",
/// key_events.rs's `landing_lights`); the first index is read as the
/// commanded position and the result is written to all of them.
fn apply_group(xplm: &Xplm, dataref: Option<DataRef>, indices: &[usize], gate: &mut GatedSwitch, powered: bool) {
    let (Some(d), Some(&first)) = (dataref, indices.first()) else { return };
    let mut buf = [0f32; 8];
    xplm.get_vf(d, &mut buf);
    let raw = buf.get(first).copied().unwrap_or(0.) as f64;
    let out = gate.step(raw, powered);
    for &i in indices {
        xplm.set_vf_at(d, i, out as f32);
    }
}

/// One exterior light group's X-Plane target and the circuit numbers that
/// power it (any one of them being live is enough; see the module doc).
struct Group {
    circuits: Vec<usize>,
    gate: GatedSwitch,
}

impl Group {
    fn new(circuits: Vec<usize>) -> Self {
        Self { circuits, gate: GatedSwitch::default() }
    }
}

/// Every light circuit's cached power variable, refreshed once a tick and
/// republished as `LIGHT CIRCUIT POWERED:n` for the Study panel or a future
/// converter/model emissive.
struct CircuitPower {
    number: usize,
    id: systems::simulation::VariableIdentifier,
}

/// One RMP's DC-bus-gated outputs (see [`rmp_green_led_on`] and
/// [`rmp_screen_brightness`]): the green STANDBY LED, and the CDS screen's
/// own backlight potentiometer.
struct RmpGreenLed {
    /// The RMP's own DC bus (`A32NX_ELEC_DC_ESS_BUS_IS_POWERED` for RMP 1/2,
    /// `A32NX_ELEC_DC_1_BUS_IS_POWERED` for RMP 3 -- the same buses
    /// `display/screens.rs`'s `RMPS` dimming already gates its screen on).
    dc_power_src: VariableIdentifier,
    /// `A380X_RMP_<n>_BRIGHTNESS_KNOB`: the RMP's own screen brightness
    /// knob (cockpit_variables.txt; the cockpit's physical knob writes it).
    brightness_src: VariableIdentifier,
    /// `A380X_RMP_<n>_GREEN_LED`: what this module now writes.
    out: VariableIdentifier,
    /// `LIGHT POTENTIOMETER:<80|81|82>` (`display/screens.rs`'s
    /// `SCREEN_DU_RMP_1/2/3` dimming; `RmpStateController.ts`'s own
    /// `screenPotentiometer` 80/81/82): the RMP's CDS screen backlight.
    /// Nothing else writes it in this port (W143 finding 10) other than
    /// `extra_backend/lighting_presets.rs` while a lighting preset is
    /// actively loading -- see `update`'s `preset_load_active` guard.
    screen_pot: VariableIdentifier,
}

/// One of MSFS's own indexed light-circuit booleans (`LIGHT PANEL:n`/`LIGHT
/// PANEL ON:n`, `LIGHT CABIN:n`/`LIGHT CABIN ON:n`, `LIGHT PEDESTRAL`/`LIGHT
/// PEDESTRAL ON`) that the converted cockpit's own RPN reads directly
/// (main.lua's pedestal/cabin/panel-knob emissives), rather than through
/// `LIGHT CIRCUIT POWERED:n` (W131: this plugin had no writer for any of
/// them, so they sat at whatever `cockpit_variables.txt`'s start value left
/// them). `circuit_numbers` is systems.cfg's `Type:CIRCUIT_LIGHT_<TYPE>:n`
/// index resolved to that type's actual circuit number(s) -- every circuit
/// of the type for the bare (unindexed) MSFS name, MSFS's own "any circuit
/// of this type" convention, or just the one circuit for an indexed name.
/// Both `plain` and `on` get the same value (see the edit's WHY): the
/// circuit's power state is the only real signal upstream of the
/// potentiometer this port has for either one.
struct MsfsLight {
    plain: VariableIdentifier,
    on: VariableIdentifier,
    circuit_numbers: Vec<usize>,
}

pub struct Lights {
    // X-Plane exterior light datarefs.
    beacon_on: Option<DataRef>,
    navigation_lights_on: Option<DataRef>,
    strobe_lights_on: Option<DataRef>,
    generic_lights_switch: Option<DataRef>,
    taxi_light_on: Option<DataRef>,
    landing_lights_switch: Option<DataRef>,
    // Window heat / wipers.
    window_heat_on: Option<DataRef>,
    wiper_speed_switch: Option<DataRef>,

    // Ice protection output wiring (see module doc): X-Plane's real icing
    // switches, and the already-computed FlyByWire state read into them.
    eng_cowl_anti_ice: Option<DataRef>,
    wing_hot_bleed_left: Option<DataRef>,
    wing_hot_bleed_right: Option<DataRef>,
    aoa_heat: [Option<DataRef>; 3],
    tat_heat: [Option<DataRef>; 3],
    pitot_heat: [Option<DataRef>; 3],
    static_heat: [Option<DataRef>; 3],
    rain_repellent: Option<DataRef>,

    eng_anti_ice_src: [VariableIdentifier; 4],
    wing_anti_ice_src: VariableIdentifier,
    probe_heat_src: [VariableIdentifier; 3],
    rain_repellent_src: [VariableIdentifier; 2],

    beacon: Group,
    nav: Group,
    strobe: Group,
    logo: Group,
    wing: Group,
    taxi_nose: Group,
    taxi_turnoff: Group,
    landing_nose: Group,
    landing_main: Group,

    window_heat: Group,
    wiper_left: Group,
    wiper_right: Group,

    circuit_power: Vec<CircuitPower>,
    msfs_lights: Vec<MsfsLight>,

    rmp_green_led: [RmpGreenLed; 3],

    // Storm light (W143 finding 10, W197): A380_Cockpit_Behavior.xml:107-
    // 130's Component "VARIABLE_MAPPING" has no NODE_ID, so the converter
    // never binds its Update RPN to anything -- the switch's own L:var
    // moves only its own geometry. The MIP flood/pedestal knobs the XML
    // block would otherwise gate are unaffected: bind.rs's
    // FBW_Stepless_Potentiometer handling already drives
    // `LIGHT POTENTIOMETER:83`/`:7` straight from those two knobs' own
    // manipulator commands (installed main.lua:5268-5312, 5358-5392),
    // bypassing the XML's A380X_PED_LIGHTING_*_KNOB/_LEVEL indirection
    // entirely -- only the storm override itself (XML:121-127's `if`
    // branch, "flood lights to max") is missing.
    storm_lt_src: VariableIdentifier,
    mip_flood_pot: VariableIdentifier,
    ambient_pot: VariableIdentifier,

    // `A32NX_LIGHTING_PRESET_LOAD` (the exact variable
    // extra_backend/lighting_presets.rs's own `load_request` field already
    // registers by name -- Vars::add is idempotent by name, so this is the
    // same slot, not a duplicate). extra_backend/lighting_presets.rs's
    // `LIGHTS` table (lines 55-63) writes these SAME five potentiometers
    // (80/81/82/83/7) while a preset is actively loading; without this
    // guard this module's per-tick write would reset `current` back to the
    // knob/storm value on every tick that isn't itself a load step, which
    // stops `load_lighting_preset`'s `converge_value` from ever getting
    // closer than one step and hangs the load's `finished` check forever
    // for these five lights specifically (see the report's "INTERACTION
    // RESOLVED" section). `self.extra_backend.update` runs after
    // `self.lights.update` in the tick (lib.rs:1714 vs :1750), so it always
    // has the last word for the indices it actually steps; this module
    // simply stays out of its way for the whole load instead of relying on
    // write order.
    preset_load_active: VariableIdentifier,
}

/// The circuit numbers of every circuit of one type, from the general model.
fn circuit_numbers(circuits: &Circuits, type_name: &str) -> Vec<usize> {
    circuits.of_type(type_name).into_iter().map(|c| c.number).collect()
}

/// The circuit numbers of every circuit of one type whose index matches (or,
/// with `exclude`, does not match) `index`.
fn circuit_numbers_where(circuits: &Circuits, type_name: &str, index: usize, exclude: bool) -> Vec<usize> {
    circuits
        .of_type(type_name)
        .into_iter()
        .filter(|c| (c.index == index) != exclude)
        .map(|c| c.number)
        .collect()
}

/// The circuit number of the one circuit whose `Name:` field matches.
fn circuit_number_named(circuits: &Circuits, name: &str) -> Vec<usize> {
    circuits.list().into_iter().filter(|c| c.name.as_deref() == Some(name)).map(|c| c.number).collect()
}

/// Whether any of a group's circuits is live right now.
fn group_powered(vars: &mut Vars, circuits: &Circuits, g: &Group) -> bool {
    circuits.any_powered(vars, &g.circuits)
}

impl Lights {
    pub fn new(vars: &mut Vars, xplm: &Xplm, circuits: &Circuits) -> Self {
        let find = |n: &str| xplm.find(&format!("{SWITCHES}{n}"));
        let circuit_power = circuits
            .list()
            .into_iter()
            .filter(|c| c.type_name.starts_with("CIRCUIT_LIGHT_"))
            .map(|c| CircuitPower { number: c.number, id: vars.get(format!("LIGHT CIRCUIT POWERED:{}", c.number)) })
            .collect();
        // W131: MSFS's own LIGHT PANEL/CABIN/PEDESTRAL booleans the cockpit
        // reads directly. `msfs_index` is the type-relative SIMVAR_INDEX
        // (systems.cfg); `None` is the bare/unindexed name, MSFS's own "any
        // circuit of this type" aggregate.
        let msfs_light = |vars: &mut Vars, msfs_type: &str, cfg_type: &str, msfs_index: Option<usize>| {
            let suffix = msfs_index.map(|n| format!(":{n}")).unwrap_or_default();
            let numbers = match msfs_index {
                Some(n) => circuit_numbers_where(circuits, cfg_type, n, false),
                None => circuit_numbers(circuits, cfg_type),
            };
            MsfsLight {
                plain: vars.get(format!("{msfs_type}{suffix}")),
                on: vars.get(format!("{msfs_type} ON{suffix}")),
                circuit_numbers: numbers,
            }
        };
        let msfs_lights = vec![
            msfs_light(vars, "LIGHT PANEL", "CIRCUIT_LIGHT_PANEL", None),
            msfs_light(vars, "LIGHT PANEL", "CIRCUIT_LIGHT_PANEL", Some(2)),
            msfs_light(vars, "LIGHT PANEL", "CIRCUIT_LIGHT_PANEL", Some(4)),
            msfs_light(vars, "LIGHT CABIN", "CIRCUIT_LIGHT_CABIN", Some(1)),
            msfs_light(vars, "LIGHT PEDESTRAL", "CIRCUIT_LIGHT_PEDESTAL", None),
        ];
        Self {
            beacon_on: find("beacon_on"),
            navigation_lights_on: find("navigation_lights_on"),
            strobe_lights_on: find("strobe_lights_on"),
            generic_lights_switch: find("generic_lights_switch"),
            taxi_light_on: find("taxi_light_on"),
            landing_lights_switch: find("landing_lights_switch"),
            window_heat_on: xplm.find("sim/cockpit2/ice/ice_window_heat_on"),
            wiper_speed_switch: xplm.find("sim/cockpit2/switches/wiper_speed_switch"),

            eng_cowl_anti_ice: xplm.find("sim/cockpit2/ice/cowling_thermal_anti_ice_per_engine"),
            wing_hot_bleed_left: xplm.find("sim/cockpit2/ice/ice_surface_hot_bleed_air_left_on"),
            wing_hot_bleed_right: xplm.find("sim/cockpit2/ice/ice_surface_hot_bleed_air_right_on"),
            aoa_heat: [
                xplm.find("sim/cockpit2/ice/ice_AOA_heat_on"),
                xplm.find("sim/cockpit2/ice/ice_AOA_heat_on_copilot"),
                xplm.find("sim/cockpit2/ice/ice_AOA_heat_on_stby"),
            ],
            tat_heat: [
                xplm.find("sim/cockpit2/ice/ice_TAT_heat_on"),
                xplm.find("sim/cockpit2/ice/ice_TAT_heat_on_copilot"),
                xplm.find("sim/cockpit2/ice/ice_TAT_heat_on_stby"),
            ],
            pitot_heat: [
                xplm.find("sim/cockpit2/ice/ice_pitot_heat_on_pilot"),
                xplm.find("sim/cockpit2/ice/ice_pitot_heat_on_copilot"),
                xplm.find("sim/cockpit2/ice/ice_pitot_heat_on_standby"),
            ],
            static_heat: [
                xplm.find("sim/cockpit2/ice/ice_static_heat_on_pilot"),
                xplm.find("sim/cockpit2/ice/ice_static_heat_on_copilot"),
                xplm.find("sim/cockpit2/ice/ice_static_heat_on_standby"),
            ],
            rain_repellent: xplm.find("sim/cockpit2/switches/rain_repellent_switch"),

            eng_anti_ice_src: [
                vars.get("ENG ANTI ICE:1".to_string()),
                vars.get("ENG ANTI ICE:2".to_string()),
                vars.get("ENG ANTI ICE:3".to_string()),
                vars.get("ENG ANTI ICE:4".to_string()),
            ],
            wing_anti_ice_src: vars.get("PNEU_WING_ANTI_ICE_SYSTEM_ON".to_string()),
            probe_heat_src: [
                vars.get("PROBE_HEAT_LOAD_W:1".to_string()),
                vars.get("PROBE_HEAT_LOAD_W:2".to_string()),
                vars.get("PROBE_HEAT_LOAD_W:3".to_string()),
            ],
            rain_repellent_src: [
                vars.get("RAIN_REPELLENT_LEFT_ON".to_string()),
                vars.get("RAIN_REPELLENT_RIGHT_ON".to_string()),
            ],

            beacon: Group::new(circuit_numbers(circuits, "CIRCUIT_LIGHT_BEACON")),
            nav: Group::new(circuit_numbers(circuits, "CIRCUIT_LIGHT_NAV")),
            strobe: Group::new(circuit_numbers(circuits, "CIRCUIT_LIGHT_STROBE")),
            logo: Group::new(circuit_numbers(circuits, "CIRCUIT_LIGHT_LOGO")),
            wing: Group::new(circuit_numbers(circuits, "CIRCUIT_LIGHT_WING")),
            taxi_nose: Group::new(circuit_numbers_where(circuits, "CIRCUIT_LIGHT_TAXI", 1, false)),
            taxi_turnoff: Group::new(circuit_numbers_where(circuits, "CIRCUIT_LIGHT_TAXI", 1, true)),
            landing_nose: Group::new(circuit_numbers_where(circuits, "CIRCUIT_LIGHT_LANDING", 1, false)),
            landing_main: Group::new(circuit_numbers_where(circuits, "CIRCUIT_LIGHT_LANDING", 1, true)),

            window_heat: Group::new({
                let mut c = circuit_number_named(circuits, "WipersLeft");
                c.extend(circuit_number_named(circuits, "WipersRIght"));
                c
            }),
            wiper_left: Group::new(circuit_number_named(circuits, "WipersLeft")),
            wiper_right: Group::new(circuit_number_named(circuits, "WipersRIght")),

            circuit_power,
            msfs_lights,

            // RMP 1/2 share the DC ESS bus (RmpStateController.ts's own
            // `dcPowerVar` for `rmpIndex` 1 and 2); RMP 3 is on DC 1.
            // Potentiometer indices match `display/screens.rs`'s
            // `SCREEN_DU_RMP_1/2/3` dimming (80/81/82), the same ones
            // `RmpStateController.ts`'s own `screenPotentiometer` uses.
            rmp_green_led: [1, 2, 3].map(|n| RmpGreenLed {
                dc_power_src: vars.get(if n == 3 { "ELEC_DC_1_BUS_IS_POWERED" } else { "ELEC_DC_ESS_BUS_IS_POWERED" }.to_string()),
                brightness_src: vars.register_named(&format!("A380X_RMP_{n}_BRIGHTNESS_KNOB")),
                out: vars.register_named(&format!("A380X_RMP_{n}_GREEN_LED")),
                screen_pot: vars.get(format!("LIGHT POTENTIOMETER:{}", match n { 1 => 80, 2 => 81, _ => 82 })),
            }),

            // Storm light: A380X_OVHD_STORM_LT is not a simulator variable
            // (register_named, not vars.get -- see fixes/W119.md's report
            // for why vars.get would double-prefix a non-simulator name).
            // The two potentiometers use vars.get, matching
            // extra_backend/lighting_presets.rs:155's own established use
            // of the same "LIGHT POTENTIOMETER:n" simulator-variable name.
            storm_lt_src: vars.register_named("A380X_OVHD_STORM_LT"),
            mip_flood_pot: vars.get("LIGHT POTENTIOMETER:83".to_string()),
            ambient_pot: vars.get("LIGHT POTENTIOMETER:7".to_string()),

            // Same slot extra_backend/lighting_presets.rs's own
            // `load_request` registers (extra_backend/mod.rs's `named()` is
            // `vars.get(name.to_string())`, identical to this call) --
            // Vars::add is idempotent by name, so this returns the existing
            // identifier whichever module constructs first.
            preset_load_active: vars.get("LIGHTING_PRESET_LOAD".to_string()),
        }
    }

    /// After the systems tick, so `ELEC_*_BUS_IS_POWERED` is this tick's.
    pub fn update(&mut self, vars: &mut Vars, xplm: &Xplm, circuits: &Circuits, delta: f64) {
        let _ = delta; // No dynamics yet: gating is instantaneous.

        let powered = group_powered(vars, circuits, &self.beacon);
        apply_scalar(xplm, self.beacon_on, &mut self.beacon.gate, powered);
        let powered = group_powered(vars, circuits, &self.nav);
        apply_scalar(xplm, self.navigation_lights_on, &mut self.nav.gate, powered);
        let powered = group_powered(vars, circuits, &self.strobe);
        apply_scalar(xplm, self.strobe_lights_on, &mut self.strobe.gate, powered);
        let powered = group_powered(vars, circuits, &self.taxi_nose);
        apply_scalar(xplm, self.taxi_light_on, &mut self.taxi_nose.gate, powered);

        let powered = group_powered(vars, circuits, &self.logo);
        apply_group(xplm, self.generic_lights_switch, &[2], &mut self.logo.gate, powered);
        let powered = group_powered(vars, circuits, &self.wing);
        apply_group(xplm, self.generic_lights_switch, &[1], &mut self.wing.gate, powered);
        let powered = group_powered(vars, circuits, &self.taxi_turnoff);
        apply_group(xplm, self.generic_lights_switch, &[0], &mut self.taxi_turnoff.gate, powered);

        let powered = group_powered(vars, circuits, &self.landing_nose);
        apply_group(xplm, self.landing_lights_switch, &[0, 1], &mut self.landing_nose.gate, powered);
        let powered = group_powered(vars, circuits, &self.landing_main);
        apply_group(xplm, self.landing_lights_switch, &[2, 3, 4, 5], &mut self.landing_main.gate, powered);

        let powered = group_powered(vars, circuits, &self.window_heat);
        apply_scalar(xplm, self.window_heat_on, &mut self.window_heat.gate, powered);
        let powered = group_powered(vars, circuits, &self.wiper_left);
        apply_group(xplm, self.wiper_speed_switch, &[0], &mut self.wiper_left.gate, powered);
        let powered = group_powered(vars, circuits, &self.wiper_right);
        apply_group(xplm, self.wiper_speed_switch, &[1], &mut self.wiper_right.gate, powered);

        // Ice protection output wiring (see module doc): forward FlyByWire's
        // already-computed anti-ice/probe-heat state onto X-Plane's own
        // native icing switches, which is what actually keeps frm_ice/
        // inlet_ice/aoa_ice/pitot_ice from accreting.
        for (n, &id) in self.eng_anti_ice_src.iter().enumerate() {
            if let Some(d) = self.eng_cowl_anti_ice {
                xplm.set_vi_at(d, n, on_state(vars.read(&id)) as i32);
            }
        }
        let wing_on = on_state(vars.read(&self.wing_anti_ice_src));
        apply_bool(xplm, self.wing_hot_bleed_left, wing_on);
        apply_bool(xplm, self.wing_hot_bleed_right, wing_on);
        for i in 0..3 {
            let heat_on = probe_heat_on(vars.read(&self.probe_heat_src[i]));
            apply_bool(xplm, self.aoa_heat[i], heat_on);
            apply_bool(xplm, self.tat_heat[i], heat_on);
            apply_bool(xplm, self.pitot_heat[i], heat_on);
            apply_bool(xplm, self.static_heat[i], heat_on);
        }
        for (n, &id) in self.rain_repellent_src.iter().enumerate() {
            if let Some(d) = self.rain_repellent {
                xplm.set_vi_at(d, n, on_state(vars.read(&id)) as i32);
            }
        }

        for c in &self.circuit_power {
            let on = circuits.powered(vars, c.number) as i32 as f64;
            vars.write(&c.id, on);
        }
        for l in &self.msfs_lights {
            let on = circuits.any_powered(vars, &l.circuit_numbers) as i32 as f64;
            vars.write(&l.plain, on);
            vars.write(&l.on, on);
        }

        // RMP green STANDBY LEDs (see `rmp_green_led_on`'s doc comment):
        // this port's RMP screens are their own CEF/JS instrument views
        // (`display/screens.rs`'s `SCREEN_DU_RMP_1/2/3`), and FBW's own
        // `A380xRmpStateController.ts` (the real, only, writer of
        // `L:A380X_RMP_<n>_GREEN_LED`) runs inside that view, not in this
        // plugin -- so the dataref SASL's lamp-test RPN reads
        // (`fbw/A380X_RMP_<n>_GREEN_LED`) never moves whether or not that
        // view happens to be running. Publishing the same formula here from
        // data this module already has makes the LED work independent of
        // that view. The same gap left the screen's own backlight
        // (`LIGHT POTENTIOMETER:<80|81|82>`, `screen_pot`) permanently at
        // its unwritten default (W143 finding 10): RmpStateController.ts's
        // `screenBrightness` -> `LIGHT_POTENTIOMETER_SET` key event lives in
        // the same never-confirmed-booting view, so it is republished here
        // too, from the formula `rmp_screen_brightness` documents --
        // skipped only while a lighting preset is actively loading (see the
        // `preset_load_active` field doc), so this module never fights
        // `extra_backend/lighting_presets.rs` for the same potentiometer.
        let preset_loading = vars.read(&self.preset_load_active) != 0.;
        for r in &self.rmp_green_led {
            let dc_powered = vars.read(&r.dc_power_src) != 0.;
            let brightness = vars.read(&r.brightness_src);
            vars.write(&r.out, rmp_green_led_on(dc_powered, brightness) as i32 as f64);
            if !preset_loading {
                vars.write(&r.screen_pot, rmp_screen_brightness(dc_powered, brightness));
            }
        }

        // Storm light (W143 finding 10, W197): forces the MIP flood and
        // pedestal/ambient lights to full bright while the switch is on
        // (XML:121-127), defeating their own knobs -- the knobs' own up/
        // down commands already own these two potentiometers the rest of
        // the time (main.lua:5268-5312, 5358-5392), so nothing is written
        // when the switch is off; there is no state to hand back. Also
        // skipped during an active lighting-preset load, same reason as
        // `screen_pot` above.
        if !preset_loading {
            if let Some(v) = storm_light_potentiometer(on_state(vars.read(&self.storm_lt_src))) {
                vars.write(&self.mip_flood_pot, v);
                vars.write(&self.ambient_pot, v);
            }
        }
    }
}

/// A switch/button variable's boolean state: FlyByWire writes 1./0. for its
/// pushbuttons and hold-simvars, but treats "not exactly zero" as on
/// generally (systems_wasm's own `f64 != 0.` convention), so this matches
/// that rather than an exact-equality check.
fn on_state(value: f64) -> bool {
    value != 0.
}

/// Whether an ADIRU's probe-heat channel is drawing power right now, from
/// the watts `physics/adirs.rs` already published on `PROBE_HEAT_LOAD_W:n`
/// (docs/physics/adirs.md): 0 W means the pitot/static/AOA/TAT heaters for
/// that channel are unpowered (bus dead, or FlyByWire's AUTO logic has them
/// off), anything else means real heater load is flowing.
fn probe_heat_on(load_w: f64) -> bool {
    load_w > 0.
}

/// Whether an RMP's green STANDBY LED should be lit, from FBW's own state
/// machine (`instruments/src/RMP/Systems/RmpStateController.ts`'s
/// `onUpdate`): powered, and the brightness knob at 0 (the screen dark but
/// the panel alive) -- `RmpState.OffStandby`, the only state that writes
/// `L:A380X_RMP_<n>_GREEN_LED` true. `failed` (the other input to FBW's own
/// state machine) is left out: this port has no `A380Failure::
/// RadioManagementPanel1/2/3` modelled anywhere (grep of failures.rs and
/// breakers.rs for "RadioManagementPanel"/"RMP" finds nothing), so there is
/// no signal in this plugin to gate it on -- equivalent to FBW's formula
/// with `failed` always false, same simplification the LED already lived
/// with when the RMP JS view itself was the only writer.
fn rmp_green_led_on(dc_powered: bool, brightness_knob: f64) -> bool {
    dc_powered && brightness_knob <= 0.
}

/// An RMP's own CDS screen backlight, from FlyByWire's own state machine
/// (`RmpStateController.ts`'s `onUpdate`/`screenBrightness`): lit only when
/// powered and the brightness knob is above 0 (the complementary condition
/// to `rmp_green_led_on`'s `OffStandby`), and once lit, floored at 5 % so
/// the screen stays legible even with the knob turned almost all the way
/// down -- `failed` omitted, same simplification as `rmp_green_led_on`.
/// `display/mod.rs`'s `screens::brightness` already multiplies this
/// potentiometer by the RMP's DC bus power a second time for its one
/// consumer (`SCREEN_DU_RMP_1/2/3`), so gating on `dc_powered` here too is
/// redundant for that consumer specifically, but matches FBW's own formula
/// exactly in case anything else (a bezel legend backlight) ever reads this
/// potentiometer without separately checking the bus.
fn rmp_screen_brightness(dc_powered: bool, brightness_knob: f64) -> f64 {
    if dc_powered && brightness_knob > 0. {
        brightness_knob.max(5.) / 100.
    } else {
        0.
    }
}

/// The storm-light override (A380_Cockpit_Behavior.xml:121-127): `Some(1.)`
/// (100 %, as the 0..1 fraction `LIGHT POTENTIOMETER:n` is kept in) while
/// the switch is on, `None` while it is off -- deliberately not `Some(0.)`
/// for "off", because off means "leave the knob-driven potentiometer alone"
/// (the physical MIP flood/pedestal knobs already own `LIGHT
/// POTENTIOMETER:83`/`:7` directly, see the struct field doc), not "force
/// it dark".
fn storm_light_potentiometer(storm_on: bool) -> Option<f64> {
    storm_on.then_some(1.)
}

fn apply_bool(xplm: &Xplm, dataref: Option<DataRef>, value: bool) {
    let Some(d) = dataref else { return };
    xplm.set_i(d, value as i32);
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_click_while_unpowered_is_remembered_and_relights_on_power_return() {
        let mut gate = GatedSwitch::default();
        // Boot: off, unpowered.
        assert_eq!(gate.step(0., false), 0.);
        // The cockpit clicks the switch on while the bus is dead.
        assert_eq!(gate.step(1., false), 0., "stays dark with no power");
        // No further click; power returns.
        assert_eq!(gate.step(0., true), 1., "relights without a new click");
    }

    #[test]
    fn power_loss_dims_the_light_without_losing_the_switch_position() {
        let mut gate = GatedSwitch::default();
        assert_eq!(gate.step(1., true), 1.);
        assert_eq!(gate.step(1., true), 1.);
        // The bus dies; the switch itself has not moved.
        assert_eq!(gate.step(1., false), 0.);
        // It comes back once the bus does, still without a new click.
        assert_eq!(gate.step(1., true), 1.);
    }

    #[test]
    fn a_click_while_powered_is_applied_immediately() {
        let mut gate = GatedSwitch::default();
        assert_eq!(gate.step(0., true), 0.);
        assert_eq!(gate.step(1., true), 1.);
        assert_eq!(gate.step(0., true), 0.);
    }

    #[test]
    fn clicking_off_while_unpowered_cannot_be_told_from_the_forced_dark_state() {
        // A known limitation, not a bug: this module can only recognise a
        // new command by comparing against the value it itself last wrote.
        // While unpowered it always forces the dataref to 0 (dark), which
        // reads back identically to the pilot separately clicking the
        // switch off — both are 0. So a switch clicked off while dark keeps
        // its last-known-on commanded position, and relights on power
        // return needing a fresh click, rather than staying off. Documented
        // here rather than silently assumed away.
        let mut gate = GatedSwitch::default();
        assert_eq!(gate.step(1., true), 1.);
        assert_eq!(gate.step(1., false), 0.);
        assert_eq!(gate.step(0., false), 0.); // clicked off while dark; indistinguishable from forced-dark
        assert_eq!(gate.step(0., true), 1., "known limitation: relights, needing a fresh click to turn off");
    }

    #[test]
    fn circuit_numbers_where_splits_landing_index_one_from_the_rest() {
        let mut vars = crate::aspects::test_vars::TestVars::default();
        let circuits = Circuits::new(&mut vars);
        let nose = circuit_numbers_where(&circuits, "CIRCUIT_LIGHT_LANDING", 1, false);
        let main = circuit_numbers_where(&circuits, "CIRCUIT_LIGHT_LANDING", 1, true);
        assert_eq!(nose.len(), 1);
        assert_eq!(main.len(), 2);
        assert!(nose.iter().all(|n| !main.contains(n)));
    }

    #[test]
    fn wiper_circuits_resolve_by_name() {
        let mut vars = crate::aspects::test_vars::TestVars::default();
        let circuits = Circuits::new(&mut vars);
        assert_eq!(circuit_number_named(&circuits, "WipersLeft").len(), 1);
        assert_eq!(circuit_number_named(&circuits, "WipersRIght").len(), 1);
    }

    #[test]
    fn probe_heat_on_reads_any_nonzero_watt_load_as_on() {
        assert!(!probe_heat_on(0.));
        assert!(probe_heat_on(150.)); // 2*PITOT_HEATER_W + TAT_HEATER_W's ballpark
        assert!(!probe_heat_on(-0.)); // still exactly zero
    }

    #[test]
    fn rmp_green_led_matches_flybywires_offstandby_state() {
        // Unpowered: never green (FBW's `state.set` never reaches
        // OffStandby without `powerOn`).
        assert!(!rmp_green_led_on(false, 0.));
        assert!(!rmp_green_led_on(false, 80.));
        // Powered, brightness knob up (screen lit, `RmpState::On`): not
        // OffStandby, LED off.
        assert!(!rmp_green_led_on(true, 80.));
        // Powered, knob at 0 (screen dark but panel alive): OffStandby.
        assert!(rmp_green_led_on(true, 0.));
        assert!(rmp_green_led_on(true, -1.), "a knob reading that has drifted slightly negative is still \"at or below 0\"");
    }

    #[test]
    fn rmp_screen_brightness_matches_flybywires_onstandby_floor() {
        assert_eq!(rmp_screen_brightness(false, 80.), 0., "unpowered: never lit");
        assert_eq!(rmp_screen_brightness(true, 0.), 0., "knob at 0: OffStandby, screen dark (the green LED covers this state instead)");
        assert_eq!(rmp_screen_brightness(true, 80.), 0.8, "knob well above the 5% floor: passes straight through");
        assert_eq!(rmp_screen_brightness(true, 2.), 0.05, "knob just above 0: floored at 5% so the screen stays legible");
    }

    #[test]
    fn storm_light_forces_full_potentiometer_only_when_on() {
        assert_eq!(storm_light_potentiometer(false), None, "off: leave the knob-driven potentiometer alone, don't force it dark");
        assert_eq!(storm_light_potentiometer(true), Some(1.), "on: XML:122-123's 100 percent, as a 0..1 fraction");
    }

    #[test]
    fn on_state_matches_flybywires_nonzero_convention() {
        assert!(!on_state(0.));
        assert!(on_state(1.));
        assert!(on_state(-1.), "systems_wasm treats any nonzero as on, not just 1.0");
    }

    #[test]
    fn there_are_more_than_twenty_light_circuits_to_publish() {
        // Lights::new itself needs a real Xplm (X-Plane's own bindings), so
        // it is not unit-testable here; this pins the count its
        // circuit_power list is built from: nav 4, beacon 2, landing 3,
        // taxi 3, strobe 3, recognition 1, wing 2, logo 2, panel 7 (circuits
        // 31, 61-63, 152-154), cabin 3 (32, 33, 151), pedestal 1,
        // glareshield 3 = 34.
        let mut vars = crate::aspects::test_vars::TestVars::default();
        let circuits = Circuits::new(&mut vars);
        let n = circuits.list().iter().filter(|c| c.type_name.starts_with("CIRCUIT_LIGHT_")).count();
        assert_eq!(n, 34);
    }
}
