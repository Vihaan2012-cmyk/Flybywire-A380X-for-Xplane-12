//! Mirrors FlyByWire's own computed state onto X-Plane's *standard*
//! datarefs (`sim/cockpit2/...`, `sim/cockpit/...`) so that X-Plane's own
//! features and any third-party add-on that only knows X-Plane's SDK — ATC,
//! weather radar plugins, Ground Equipment scripts, VR, sound mods, SimBrief
//! loaders, streaming overlays, AviTab, flight trackers — see the FlyByWire
//! A380X the way they would see any other X-Plane aircraft.
//!
//! **Scope.** This module only *adds* coverage: every dataref below was
//! confirmed absent from the rest of `src` (`grep` across the tree) before
//! being added here, and every one is confirmed writable in `DataRefs.txt`
//! (`D:/Steam Games/steamapps/common/X-Plane 12/Resources/plugins/
//! DataRefs.txt`, the `y`/`n` "Writable" column). A large part of the brief
//! ([`sim/cockpit2/switches`] lights, gear/flap/speedbrake/parking-brake
//! handles, door ratios, com/nav/transponder actuators) is already owned by
//! `lights.rs`, `handling.rs`, `flight_controls.rs`, `doors.rs`, `radios.rs`
//! and `key_events.rs`; this module does not touch any dataref one of those
//! already reads or writes, to avoid a feedback loop. See "Skipped" below.
//!
//! **No persistent module state.** Unlike most of this plugin's modules,
//! this one is not a `Plugin` field: [`update`] is a free function called
//! once per tick from `lib.rs` (its whole footprint there is the `mod`
//! declaration and one call in `tick`). The X-Plane dataref handles are
//! looked up once into a process-wide [`std::sync::OnceLock`] the same way
//! `xp.rs`'s own `probe_terrain_y`/`magnetic_variation` cache their XPLM
//! symbols — raw pointers are kept as `usize` so the `OnceLock` itself stays
//! `Sync`. FlyByWire's own variable identifiers are *not* cached: `Vars::get`
//! is a cheap hash-map lookup (`lib.rs`'s `Vars::add`), so this module just
//! asks for each name fresh every tick rather than adding another persistent
//! cache to keep in sync.
//!
//! ## Dataref table
//!
//! | X-Plane dataref | FlyByWire variable | Notes |
//! |---|---|---|
//! | `sim/cockpit2/engine/indicators/N1_percent:n` | `ENGINE_N1:n` | direct, percent |
//! | `sim/cockpit2/engine/indicators/N2_percent:n` | `ENGINE_N3:n` | A380's HP spool (N3) feeds X-Plane's generic "N2" slot, the same convention `engine_commands.rs` already uses for `ENGN_N2_` |
//! | `sim/cockpit2/engine/indicators/EGT_deg_cel:n` | `ENGINE_EGT:n` | direct, deg C |
//! | `sim/cockpit2/engine/indicators/fuel_flow_kg_sec:n` | `ENGINE_FUEL_DEMAND_KG_S:n` | already kg/s (unlike `ENGINE_FF:n`, which is kg/h) |
//! | `sim/cockpit2/electrical/battery_on:n` | `ELEC_BAT_n_POTENTIAL` (n=1..4) | `on_from_potential`: >1 V |
//! | `sim/cockpit2/electrical/battery_voltage_actual_volts:n` | `ELEC_BAT_n_POTENTIAL` | direct, the physics workstream's Kirchhoff-solved terminal voltage (`docs/physics/electrical.md`) |
//! | `sim/cockpit2/electrical/generator_on:n` | `ELEC_ENG_GEN_n_POTENTIAL` (n=1..4) | `on_from_potential` |
//! | `sim/cockpit2/electrical/APU_generator_on` | `ELEC_APU_GEN_1_POTENTIAL` or `_2_POTENTIAL` | either APU generator on |
//! | `sim/cockpit2/electrical/bus_volts:0..5` | `ELEC_<bus>_BUS_POTENTIAL` | curated 6-of-16 buses, see [`BUS_SLOTS`] |
//! | `sim/cockpit2/autopilot/heading_dial_deg_mag_pilot` | `A32NX_AUTOPILOT_HEADING_SELECTED` | skipped while dashed (-1 sentinel) |
//! | `sim/cockpit2/autopilot/altitude_dial_ft` | `A32NX_FCU_AFS_DISPLAY_ALT_VALUE` | direct, feet |
//! | `sim/cockpit2/autopilot/airspeed_dial_kts_mach` | `A32NX_AUTOPILOT_SPEED_SELECTED` | skipped while dashed (-1 sentinel) |
//! | `sim/cockpit2/autopilot/airspeed_is_mach` | `A32NX_FCU_AFS_DISPLAY_MACH_MODE` | direct, bool |
//! | `sim/cockpit2/autopilot/vvi_dial_fpm` | `A32NX_AUTOPILOT_VS_SELECTED` | 0 while in FPA mode (FlyByWire's own convention) |
//! | `sim/cockpit2/autopilot/fpa` | `A32NX_AUTOPILOT_FPA_SELECTED` | 0 while in VS mode |
//! | `sim/cockpit2/autopilot/trk_fpa` | `A32NX_TRK_FPA_MODE_ACTIVE` | direct, 0=HDG/VS 1=TRK/FPA (X-Plane's own enum matches FlyByWire's) |
//! | `sim/cockpit2/autopilot/flight_director_mode` | `A32NX_AUTOPILOT_1/2_ACTIVE`, `A32NX_FCU_FD_LIGHT_ON` | [`flight_director_mode`]: 2 with AP, 1 with FD only, 0 otherwise |
//! | `sim/cockpit2/autopilot/autothrottle_enabled` | `A32NX_AUTOTHRUST_STATUS` | [`autothrottle_enum`]: -1 off, 0 armed, 1 engaged (X-Plane's enum is coarser than the A380's own ATHR status) |
//! | `sim/cockpit2/annunciators/plugin_master_warning` | `A32NX_MASTER_WARNING` | the SDK's plugin-owned trigger path, not the read-only `master_warning` itself |
//! | `sim/cockpit2/annunciators/plugin_master_caution` | `A32NX_MASTER_CAUTION` | ditto |
//!
//! ## Skipped
//! - `sim/cockpit2/switches/*` (beacon/strobe/nav/landing/taxi lights, wiper
//!   switch), `sim/cockpit2/controls/gear_handle_down`,
//!   `flap_handle_request_ratio`, `speedbrake_ratio`,
//!   `sim/flight_controls/park_brake_*`, `sim/cockpit2/switches/door_open*`,
//!   `sim/cockpit2/radios/actuators/*` (com/nav frequencies, and the
//!   transponder code via `key_events.rs`'s `XPNDR_SET`): each already owned
//!   and actively read/written by `lights.rs`, `handling.rs`,
//!   `flight_controls.rs`, `key_events.rs`, `doors.rs` or `radios.rs`.
//!   Touching any of these here would race that module's own gating (e.g.
//!   `lights.rs`'s `GatedSwitch`) for no benefit — the state is already
//!   correct on those datarefs.
//! - `sim/cockpit2/fuel/fuel_quantity` and the other `fuel_*` indicators are
//!   marked **not writable** in `DataRefs.txt`; X-Plane derives them itself
//!   each frame from `sim/flightmodel/weight/m_fuel`, which `fuel.rs`
//!   already drives every tick. Writing here would be a no-op at best.
//! - `sim/cockpit2/autopilot/autopilot_on`, `autothrottle_on`,
//!   `autothrottle_arm`: all marked **not writable** — X-Plane computes them
//!   from its own (unused, since FlyByWire replaces it) autopilot state
//!   machine. Making X-Plane's own autopilot ever report "on" would need
//!   `sim/operation/override/override_autopilot`, a much bigger claim over
//!   X-Plane's own AP than a mirror module should take; flagged for the
//!   Study/AP-owning module to pick up if a specific add-on needs it.
//! - `sim/flightmodel2/lights/*`: all but two entries are **not writable**
//!   (X-Plane derives them from the exterior light switches, which
//!   `lights.rs` already drives correctly per its own module doc). The two
//!   writable ones (`beacon_brightness_ratio`, `strobe_brightness_ratio`)
//!   need `override_beacons_and_strobes`, which would hand this module (not
//!   `lights.rs`) the strobe flash timing X-Plane otherwise animates itself
//!   — a bigger claim than a mirror should take; left to `lights.rs` if it
//!   ever wants custom flash timing.
//! - `sim/cockpit2/electrical/GPU_generator_on/_amps/_volts`: already an
//!   input `efb.rs` reads (`efb.rs`'s `gpu_on`) as part of its own ground
//!   power start-state logic; writing it here would race that read.
//! - `sim/cockpit2/electrical/APU_generator_amps`: no FlyByWire simulator
//!   variable for APU generator current was found (only
//!   `ELEC_APU_GEN_n_SHAFT_POWER_DEMAND`, a power, and `_POTENTIAL`, a
//!   voltage); writing a value derived from those without a real load model
//!   would be exactly the faked value the debug brief forbids, so it is left
//!   unwritten.
//! - Legacy `sim/cockpit/warnings/annunciators/master_warning`/
//!   `master_caution` and `sim/cockpit/electrical/*`: X-Plane 12 keeps these
//!   pre-`cockpit2` aliases writable independently of `cockpit2`'s
//!   equivalents, but `plugin_master_warning`/`plugin_master_caution` above
//!   are the SDK-documented "writeable without override" path for exactly
//!   this purpose (`DataRefs.txt`'s own wording); driving the legacy family
//!   too would need `sim/operation/override/override_annunciators`, which
//!   claims *every* annunciator on the aircraft, not just these two.
//!
//! `on_ground`-only third-party tools (SimBrief loaders, flight trackers)
//! already work off `lib.rs`'s `mapping()` table (`TOTAL WEIGHT`,
//! `AIRSPEED INDICATED`, `SIM ON GROUND`, ...), which is why they are not
//! repeated here.

use std::sync::OnceLock;

use systems::simulation::{SimulatorReaderWriter, VariableRegistry};

use crate::xp::{DataRef, Xplm};
use crate::Vars;

/// The curated 6-of-16 systems.cfg buses shown on X-Plane's generic
/// `bus_volts[6]` array (`circuits.rs`'s `MSFS_BUSES` has all 16; X-Plane's
/// array has no slots of its own to spare for the rest). Index into
/// `bus_volts`, and the `circuits.rs`/`ELEC_<name>_BUS_POTENTIAL` name.
const BUS_SLOTS: [(usize, &str); 6] =
    [(0, "AC_ESS"), (1, "DC_ESS"), (2, "AC_1"), (3, "AC_2"), (4, "DC_1"), (5, "DC_2")];

/// X-Plane's dataref handles, looked up once. Kept as `usize` (not
/// [`DataRef`]/`*mut c_void`) purely so `OnceLock<Refs>` is `Sync`, the same
/// trick `xp.rs`'s `probe_terrain_y` uses for its probe handle.
struct Refs {
    n1_percent: Option<usize>,
    n2_percent: Option<usize>,
    egt_deg_cel: Option<usize>,
    fuel_flow_kg_sec: Option<usize>,
    battery_on: Option<usize>,
    battery_voltage_actual_volts: Option<usize>,
    generator_on: Option<usize>,
    apu_generator_on: Option<usize>,
    bus_volts: Option<usize>,
    heading_dial: Option<usize>,
    altitude_dial: Option<usize>,
    airspeed_dial_mach: Option<usize>,
    airspeed_is_mach: Option<usize>,
    vvi_dial: Option<usize>,
    fpa: Option<usize>,
    trk_fpa: Option<usize>,
    flight_director_mode: Option<usize>,
    autothrottle_enabled: Option<usize>,
    plugin_master_warning: Option<usize>,
    plugin_master_caution: Option<usize>,
}

impl Refs {
    fn new(xplm: &Xplm) -> Self {
        let f = |n: &str| xplm.find(n).map(|d| d as usize);
        Self {
            n1_percent: f("sim/cockpit2/engine/indicators/N1_percent"),
            n2_percent: f("sim/cockpit2/engine/indicators/N2_percent"),
            egt_deg_cel: f("sim/cockpit2/engine/indicators/EGT_deg_cel"),
            fuel_flow_kg_sec: f("sim/cockpit2/engine/indicators/fuel_flow_kg_sec"),
            battery_on: f("sim/cockpit2/electrical/battery_on"),
            battery_voltage_actual_volts: f("sim/cockpit2/electrical/battery_voltage_actual_volts"),
            generator_on: f("sim/cockpit2/electrical/generator_on"),
            apu_generator_on: f("sim/cockpit2/electrical/APU_generator_on"),
            bus_volts: f("sim/cockpit2/electrical/bus_volts"),
            heading_dial: f("sim/cockpit2/autopilot/heading_dial_deg_mag_pilot"),
            altitude_dial: f("sim/cockpit2/autopilot/altitude_dial_ft"),
            airspeed_dial_mach: f("sim/cockpit2/autopilot/airspeed_dial_kts_mach"),
            airspeed_is_mach: f("sim/cockpit2/autopilot/airspeed_is_mach"),
            vvi_dial: f("sim/cockpit2/autopilot/vvi_dial_fpm"),
            fpa: f("sim/cockpit2/autopilot/fpa"),
            trk_fpa: f("sim/cockpit2/autopilot/trk_fpa"),
            flight_director_mode: f("sim/cockpit2/autopilot/flight_director_mode"),
            autothrottle_enabled: f("sim/cockpit2/autopilot/autothrottle_enabled"),
            plugin_master_warning: f("sim/cockpit2/annunciators/plugin_master_warning"),
            plugin_master_caution: f("sim/cockpit2/annunciators/plugin_master_caution"),
        }
    }
}

/// `Option<usize>` back to the raw dataref handle X-Plane's API wants.
fn dr(v: Option<usize>) -> Option<DataRef> {
    v.map(|p| p as DataRef)
}

/// A FlyByWire simulator variable by its full name, registering it if this
/// is the first read this tick (`Vars::get`/`Vars::add` are hash-map
/// lookups, so nothing here is cached across ticks).
fn v(vars: &mut Vars, name: impl Into<String>) -> f64 {
    let id = vars.get(name.into());
    vars.read(&id)
}

fn set_scalar(xplm: &Xplm, d: Option<DataRef>, value: f64) {
    if let Some(d) = d {
        xplm.set_f(d, value as f32);
    }
}

fn set_scalar_i(xplm: &Xplm, d: Option<DataRef>, value: i32) {
    if let Some(d) = d {
        xplm.set_i(d, value);
    }
}

fn set_at(xplm: &Xplm, d: Option<DataRef>, i: usize, value: f64) {
    if let Some(d) = d {
        xplm.set_vf_at(d, i, value as f32);
    }
}

fn set_at_i(xplm: &Xplm, d: Option<DataRef>, i: usize, value: i32) {
    if let Some(d) = d {
        xplm.set_vi_at(d, i, value);
    }
}

/// Whether a battery/generator terminal voltage means "on": the physics
/// workstream's Kirchhoff-solved potential (`docs/physics/electrical.md`)
/// sags under load but never legitimately reads within a volt of zero while
/// live, so 1 V is a safe, unfussy threshold (matching the kind of
/// on/off-from-a-measurement thresholds `sensors.rs` already uses elsewhere).
fn on_from_potential(volts: f64) -> bool {
    volts > 1.0
}

/// FlyByWire's own "-1 means dashed" sentinel (`prim.rs`'s
/// `A32NX_AUTOPILOT_HEADING_SELECTED`/`_SPEED_SELECTED`) turned into an
/// `Option`, so a dashed FCU value is left unwritten instead of putting a
/// bogus -1 knots/degrees onto the dataref.
fn from_dash_sentinel(value: f64) -> Option<f64> {
    (value >= 0.).then_some(value)
}

/// `sim/cockpit2/autopilot/flight_director_mode`'s own enum (0 off, 1 on,
/// 2 on with autopilot servos) from FlyByWire's AP/FD engagement.
fn flight_director_mode(ap_active: bool, fd_light_on: bool) -> i32 {
    if ap_active {
        2
    } else if fd_light_on {
        1
    } else {
        0
    }
}

/// `sim/cockpit2/autopilot/autothrottle_enabled`'s enum, collapsed from
/// FlyByWire's own three-valued `A32NX_AUTOTHRUST_STATUS` (0 off, 1 armed,
/// 2 engaged+active — `prim.rs`'s own test comments) into X-Plane's coarser
/// -1/0/1 (hard off / armed / a generic "engaged" value): X-Plane's enum has
/// no equivalent of the A380's own THR/SPEED/MCT submodes to select among,
/// so `1` (airspeed hold) stands in for "engaged" generically.
fn autothrottle_enum(status: f64) -> i32 {
    if status >= 2.0 {
        1
    } else if status >= 1.0 {
        0
    } else {
        -1
    }
}

fn bool_to_i(value: f64) -> i32 {
    (value != 0.) as i32
}

/// Runs once per tick, after the systems tick, so every FlyByWire value read
/// here is this tick's. See the module doc's table for the full mapping.
pub fn update(vars: &mut Vars, xplm: &Xplm) {
    static REFS: OnceLock<Refs> = OnceLock::new();
    let r = REFS.get_or_init(|| Refs::new(xplm));

    // -- Engine indicators (N1/N2/EGT/FF), per engine 1..4 --------------
    for (i, n) in (1..=4usize).enumerate() {
        let n1 = v(vars, format!("ENGINE_N1:{n}"));
        // A380's HP spool (N3) is what `engine_commands.rs` already feeds
        // into X-Plane's generic "N2" physics slot (`ENGN_N2_`); this
        // module follows the same convention for the indicator dataref.
        let n3 = v(vars, format!("ENGINE_N3:{n}"));
        let egt = v(vars, format!("ENGINE_EGT:{n}"));
        let ff = v(vars, format!("ENGINE_FUEL_DEMAND_KG_S:{n}")); // already kg/s
        set_at(xplm, dr(r.n1_percent), i, n1);
        set_at(xplm, dr(r.n2_percent), i, n3);
        set_at(xplm, dr(r.egt_deg_cel), i, egt);
        set_at(xplm, dr(r.fuel_flow_kg_sec), i, ff);
    }

    // -- Electrical: batteries, generators, curated bus voltages --------
    for (i, n) in (1..=4usize).enumerate() {
        let potential = v(vars, format!("ELEC_BAT_{n}_POTENTIAL"));
        set_at_i(xplm, dr(r.battery_on), i, on_from_potential(potential) as i32);
        set_at(xplm, dr(r.battery_voltage_actual_volts), i, potential);
    }
    for (i, n) in (1..=4usize).enumerate() {
        let potential = v(vars, format!("ELEC_ENG_GEN_{n}_POTENTIAL"));
        set_at_i(xplm, dr(r.generator_on), i, on_from_potential(potential) as i32);
    }
    let apu1 = v(vars, "ELEC_APU_GEN_1_POTENTIAL");
    let apu2 = v(vars, "ELEC_APU_GEN_2_POTENTIAL");
    set_scalar_i(xplm, dr(r.apu_generator_on), (on_from_potential(apu1) || on_from_potential(apu2)) as i32);
    for &(i, bus) in &BUS_SLOTS {
        let potential = v(vars, format!("ELEC_{bus}_BUS_POTENTIAL"));
        set_at(xplm, dr(r.bus_volts), i, potential);
    }

    // -- Autopilot / FCU -------------------------------------------------
    let ap1 = v(vars, "A32NX_AUTOPILOT_1_ACTIVE") != 0.;
    let ap2 = v(vars, "A32NX_AUTOPILOT_2_ACTIVE") != 0.;
    let fd_light = v(vars, "A32NX_FCU_FD_LIGHT_ON") != 0.;
    set_scalar_i(xplm, dr(r.flight_director_mode), flight_director_mode(ap1 || ap2, fd_light));

    if let Some(hdg) = from_dash_sentinel(v(vars, "A32NX_AUTOPILOT_HEADING_SELECTED")) {
        set_scalar(xplm, dr(r.heading_dial), hdg);
    }
    set_scalar(xplm, dr(r.altitude_dial), v(vars, "A32NX_FCU_AFS_DISPLAY_ALT_VALUE"));
    if let Some(spd) = from_dash_sentinel(v(vars, "A32NX_AUTOPILOT_SPEED_SELECTED")) {
        set_scalar(xplm, dr(r.airspeed_dial_mach), spd);
    }
    set_scalar_i(xplm, dr(r.airspeed_is_mach), bool_to_i(v(vars, "A32NX_FCU_AFS_DISPLAY_MACH_MODE")));
    set_scalar(xplm, dr(r.vvi_dial), v(vars, "A32NX_AUTOPILOT_VS_SELECTED"));
    set_scalar(xplm, dr(r.fpa), v(vars, "A32NX_AUTOPILOT_FPA_SELECTED"));
    set_scalar_i(xplm, dr(r.trk_fpa), bool_to_i(v(vars, "A32NX_TRK_FPA_MODE_ACTIVE")));
    set_scalar_i(xplm, dr(r.autothrottle_enabled), autothrottle_enum(v(vars, "A32NX_AUTOTHRUST_STATUS")));

    // -- Annunciators ------------------------------------------------------
    set_scalar_i(xplm, dr(r.plugin_master_warning), bool_to_i(v(vars, "A32NX_MASTER_WARNING")));
    set_scalar_i(xplm, dr(r.plugin_master_caution), bool_to_i(v(vars, "A32NX_MASTER_CAUTION")));
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn on_from_potential_thresholds_at_one_volt() {
        assert!(!on_from_potential(0.));
        assert!(!on_from_potential(0.9));
        assert!(on_from_potential(1.1));
        assert!(on_from_potential(115.));
    }

    #[test]
    fn dash_sentinel_hides_negative_values_only() {
        assert_eq!(from_dash_sentinel(-1.), None);
        assert_eq!(from_dash_sentinel(0.), Some(0.));
        assert_eq!(from_dash_sentinel(250.), Some(250.));
    }

    #[test]
    fn flight_director_mode_prefers_autopilot_over_fd_light() {
        assert_eq!(flight_director_mode(false, false), 0);
        assert_eq!(flight_director_mode(false, true), 1);
        assert_eq!(flight_director_mode(true, false), 2);
        assert_eq!(flight_director_mode(true, true), 2);
    }

    #[test]
    fn autothrottle_enum_matches_xplanes_coarser_scale() {
        assert_eq!(autothrottle_enum(0.), -1);
        assert_eq!(autothrottle_enum(1.), 0);
        assert_eq!(autothrottle_enum(2.), 1);
        assert_eq!(autothrottle_enum(3.), 1, "anything at or above engaged reads as engaged");
    }

    #[test]
    fn bool_to_i_is_a_strict_nonzero_test() {
        assert_eq!(bool_to_i(0.), 0);
        assert_eq!(bool_to_i(1.), 1);
        assert_eq!(bool_to_i(-1.), 1, "FlyByWire's own bool simvars are only ever 0/1, but a stray nonzero should still read as on");
    }

    #[test]
    fn bus_slots_cover_six_distinct_indices_into_the_curated_buses() {
        let mut indices: Vec<usize> = BUS_SLOTS.iter().map(|&(i, _)| i).collect();
        indices.sort_unstable();
        assert_eq!(indices, vec![0, 1, 2, 3, 4, 5]);
        let names: std::collections::HashSet<&str> = BUS_SLOTS.iter().map(|&(_, n)| n).collect();
        assert_eq!(names.len(), BUS_SLOTS.len(), "no bus repeated across slots");
    }

    #[test]
    fn dr_round_trips_a_pointer_sized_handle() {
        let p: DataRef = 0x1234 as DataRef;
        assert_eq!(dr(Some(p as usize)), Some(p));
        assert_eq!(dr(None), None);
    }
}
