//! The plugin's side of the live layer: what fills [`Truth`] every frame,
//! what takes the [`Faults`] snapshot, and what a published name becomes.
//!
//! `deep::live` defines the contract; this file is the only place that
//! knows both that contract and `crate::Vars`/X-Plane, which is what keeps
//! `docs/deep/BRIEF.md` hard rule 2 (no area depends on `Vars` or X-Plane)
//! true even once every area is running. `Plugin` owns one [`DeepLayer`]
//! and calls [`DeepLayer::tick`] once per frame.
//!
//! ## Where every `Truth` field comes from
//!
//! The standing rule is that no field may hold an invented number: each is
//! either a real reading, or left at the value `Truth::default()`
//! documents. Per field:
//!
//! | `Truth` field | Source |
//! |---|---|
//! | `dt_s` | the frame time `Plugin::tick` is given, clamped to [`MIN_DT_S`]..[`MAX_DT_S`] |
//! | `environment` | `deep::integration::weather_truth::WeatherTruthReader` (real X-Plane weather; see its own module doc for each of its fields) |
//! | `altitude_ft` | `sim/flightmodel/position/elevation` (MSL metres), X-Plane's own position |
//! | `on_ground` | `sim/flightmodel/failures/onground_any`, the same dataref `physics/adirs.rs` and `physics/damage.rs` already read |
//! | `engine_n1_frac[i]` | `ENGINE_N1:n` / 100, this crate's own `physics::engine` output (`engine_commands.rs:494`) |
//! | `engine_running[i]` | `ENGINE_STATE:n` == `EngineState::On`, FlyByWire's own start-state machine (`fadec.rs`) |
//! | `engine_bleed_pressure_pa[i]`, `engine_bleed_temp_k[i]` | `ENGINE_{IP,HP}_PORT_{PRESSURE_PA,TEMP_K}:n`, `physics::engine`'s own customer-bleed port outputs, picked by which port the engine is actually bled from (`PNEU_ENG_n_HP_VALVE_OPEN`, exactly as `engine_commands.rs:466` decides it) |
//! | `apu_running` | `A32NX_OVHD_APU_START_PB_IS_AVAILABLE`, FlyByWire's own APU ECB `is_available()` |
//! | `apu_bleed_pressure_pa` | `A32NX_APU_BLEED_AIR_PRESSURE`, FlyByWire's own ARINC 429 word (psi absolute) |
//! | `ac_bus_volts[i]` | `A32NX_ELEC_AC_{1..4}_BUS_POTENTIAL`, FlyByWire's own electrical system |
//! | `dc_bus_volts[i]` | `A32NX_ELEC_DC_{1,2}_BUS_POTENTIAL`, ditto |
//! | `hydraulic_pressure_pa[i]` | `A32NX_HYD_{GREEN,YELLOW}_SYSTEM_1_SECTION_PRESSURE` (psi), FlyByWire's own hydraulic system |
//! | `engine_n2_frac[i]`, `engine_n3_frac[i]` | `ENGINE_N2:n` / `ENGINE_N3:n`, divided by 100 -- the same `physics::engine` output as N1, written by `engine_commands.rs` right after it |
//! | `engine_hp_port_pressure_pa[i]`, `engine_hp_port_temp_k[i]` | `ENGINE_HP_PORT_{PRESSURE_PA,TEMP_K}:n`, unconditionally (unlike `engine_bleed_*` above, which already picks IP8 or HP6 by which port is bled) |
//! | `engine_fuel_flow_kg_s[i]` | `ENGINE_FUEL_DEMAND_KG_S:n`, `physics::engine`'s own `fuel_flow_kg_s` output in SI (the same number `ENGINE_FF:n` publishes ×3600 for the cockpit) |
//! | `gpu_plugged_in` | any of `A32NX_EXT_PWR_AVAIL:{1..4}` != 0, the same real, plugin-managed Var `efb.rs`'s ground-power control and cold-start setting already write |
//! | `controls.*` | see [`Controls`]'s own doc for each field; sources are in this file's `Ids`/`Refs` construction below, grep for the field name |
//! | `commanded_surfaces.*` | the 29 `HYD_*_DEFLECTION` Vars `flight_controls.rs::FlightControls::new` also resolves (`flight_controls.rs:260-269`), converted to degrees with that file's own public `aileron_or_elevator_down_deg`/`rudder_right_deg`/`spoiler_up_deg` -- read here *before* `flight_controls.rs`/`deep::flight_controls`'s own `SurfaceOverrideWriter` run this tick (`deep.tick` is called before `self.flight_controls.update` in `lib.rs`), so this is FlyByWire's own commanded position, not last tick's physical output |
//! | `aircraft_mass_kg` | `sim/flightmodel/weight/m_total` (kg), X-Plane's own total mass |
//! | `pitch_deg` | `sim/flightmodel/position/theta`, X-Plane's own pitch (positive nose up) |
//! | `groundspeed_m_s` | `sim/flightmodel/position/groundspeed` (m/s) |
//! | `angle_of_attack_deg` | `sim/flightmodel/position/alpha`, the X-Plane SDK's own AoA dataref |
//! | `radio_height_ft` | `sim/cockpit2/gauges/indicators/radio_altimeter_height_ft_pilot`, the same dataref `prim.rs`'s own `h_radio_ft` reads |
//! | `leg_on_ground[i]` | `A32NX_LGCIU_1_{NOSE,LEFT,RIGHT}_GEAR_COMPRESSED` (FlyByWire's own primary LGCIU): nose to the nose leg, left to both `l_wing`/`l_body` and right to both `r_wing`/`r_body` -- this port's sensors do not separate wing from body gear on the same side, so both legs on a side read the one real sensor together, ANDed with `on_ground` |
//! | `leg_touchdown_sink_speed_ms[i]` | held from `sim/flightmodel/position/local_vy` (m/s, X-Plane's OpenGL-frame vertical speed, negated and floored at 0) at the frame `leg_on_ground[i]` last went false -> true; 0.0 while airborne or already settled, computed in [`DeepLayer`] across frames since it is an edge, not a reading |
//! | `cabin_pressure_pa` | `environment.ambient_pressure_pa` + the ARINC 429 `A32NX_PRESS_CPC_1_CABIN_DELTA_PRESSURE` word (psi), FlyByWire's own primary cabin pressure controller |
//! | `cabin_temp_k` | `A32NX_COND_MAIN_DECK_1_TEMP` (C) + 273.15: one representative cabin zone of the fifteen (`COND_{CKPT,MAIN_DECK_1..8,UPPER_DECK_1..7,CARGO_FWD,CARGO_BULK}_TEMP` all exist and are real; `Truth` carries one rather than fifteen, see `docs/deep/truth-requests.md` |
//! | `sun_elevation_deg` | `sim/graphics/scenery/sun_pitch_degrees`, X-Plane's own sun position |
//!
//! ### `Truth::controls`, field by field
//!
//! | `Controls` field | Source |
//! |---|---|
//! | `fire_pb_released[i]` | `A32NX_FIRE_BUTTON_ENG{n}`, the engine fire pushbutton `FirePushButton` publishes (`fire_and_smoke_protection.rs`) |
//! | `fire_pb_apu_released` | `A32NX_FIRE_BUTTON_APU`, ditto for the APU |
//! | `fire_agent_pb_pressed[i][b]` | `A32NX_OVHD_FIRE_AGENT_{1,2}_ENG_{n}_IS_PRESSED`, each bottle's own `MomentaryPushButton` |
//! | `fire_agent_pb_apu_pressed` | `A32NX_OVHD_FIRE_AGENT_1_APU_1_IS_PRESSED` |
//! | *(cargo-bay fire/agent pushbuttons)* | **unsourced** -- `fire_and_smoke_protection.rs` models 8 engine bottles and 1 APU bottle only; no field added rather than one with nothing behind it |
//! | `wing_anti_ice_selected` | `A32NX_BUTTON_OVHD_ANTI_ICE_WING_POSITION` != 0, the wing anti-ice pushbutton's own raw position (`pneumatic.rs`'s `WingAntiIcePushButton`) |
//! | `nacelle_anti_ice_selected[i]` | `A32NX_BUTTON_OVHD_ANTI_ICE_ENG_{n}_POSITION` != 0 |
//! | `engine_bleed_pb_auto[i]` | `A32NX_OVHD_PNEU_ENG_{n}_BLEED_PB_IS_AUTO` != 0 |
//! | `apu_bleed_pb_on` | `A32NX_OVHD_APU_BLEED_PB_IS_ON` != 0 |
//! | `cross_bleed_selector` | `A32NX_KNOB_OVHD_AIRCOND_XBLEED_Position`, raw (0 SHUT / 1 AUTO / 2 OPEN, `CrossBleedValveSelectorMode`'s own discriminants) |
//! | `pack_pb_on[i]` | `A32NX_OVHD_COND_PACK_{1,2}_PB_IS_ON` != 0 |
//! | `starter_engaged[i]` | recomputed from the same real reads `physics::engine`'s own `phys_inputs.starter_engaged` uses: `GENERAL ENG STARTER:n` (master), `TURB ENG IGNITION SWITCH EX1:n` (igniter == 2), `ENGINE_STATE:n` (Starting/Restarting) and `ENGINE_TIMER:n` (>= 1.7 s) -- `engine_commands.rs:445-448`'s own formula, not a new source |
//! | *(rain removal selection)* | **unsourced** -- no rain-removal pushbutton exists in this port; `controls.rain_removal_selected` stays at its `Controls::default()` value (off) every tick |
//! | `gear_door_commanded_open` | `[A32NX_GEAR_DOOR_CENTER_POSITION, ..._LEFT_POSITION, ..._RIGHT_POSITION]`, FlyByWire's own (undamaged) door actuator output `handling.rs` already mirrors onto X-Plane's gear animation |
//! | `gear_lever_down` | `A32NX_GEAR_HANDLE_POSITION` >= 0.5 |
//! | `parking_brake_on` | `A32NX_PARK_BRAKE_LEVER_POS` >= 0.5 |
//! | `brake_pedal_pos` | `sim/cockpit2/controls/{left,right}_brake_ratio`, X-Plane's own raw pedal-input datarefs (before antiskid/autobrake modify them -- `BRAKE {LEFT,RIGHT} FORCE FACTOR` is the commanded force *after* that, which is what X-Plane's brakes actually receive, not what the crew's feet are doing) |
//! | `engine_master_on[i]` | `GENERAL ENG STARTER:n` != 0, the same reading `engine_commands.rs`'s own `master`/`fuel_valve_open` uses |
//! | `eng_gen_pb_on[i]` | `A32NX_OVHD_ELEC_ENG_GEN_{n}_PB_IS_ON` != 0 |
//! | `apu_gen_pb_on[i]` | `A32NX_OVHD_ELEC_APU_GEN_{1,2}_PB_IS_ON` != 0 |
//! | `bat_pb_auto[i]` | `A32NX_OVHD_ELEC_BAT_{1,2}_PB_IS_AUTO` != 0 |
//! | `ground_spoiler_lever_armed` | `sim/cockpit2/controls/speedbrake_ratio`, through `prim::SimReadings::spoilers_from_xplane` (< -0.25 is armed) -- the exact same function and dataref `Prims::read` already uses, so this can never disagree with what the FCU/PRIMs themselves see |
//! | *(manual galley-shed pushbutton)* | **not added** -- `deep::electrical`'s own load-management already computes an automatic `galley_shed_commanded` from the power budget; no real *manual* shed switch was found in this port, and duplicating the automatic one under a different name would invite the two to drift |
//! | `apu_master_sw_on` | `A32NX_OVHD_APU_MASTER_SW_PB_IS_ON` != 0 (named for `deep::apu::live`'s own doc comment asking for exactly this) |
//! | `apu_start_pb_on` | `A32NX_OVHD_APU_START_PB_IS_ON` != 0 |
//!
//! Nothing here derives a value it cannot read. Where a dataref is missing
//! (an older SDK target, or the offline harness) the reading degrades to
//! the field's documented `Truth::default()` value rather than to zero --
//! `ambient_pressure_pa` in particular, since several areas divide by it
//! and zero is a vacuum, not a missing reading.
//!
//! The plain `!= 0.0`/`>= 0.5` boolean reads in `Controls` cannot do that
//! same degradation -- a boolean has no spare sentinel the way a pressure
//! or temperature does, so "never written" and "explicitly commanded off"
//! read identically, as 0.0. This is already true of `apu_running`/
//! `engine_running` above and was never a problem, because FlyByWire's own
//! compiled systems always write every LVar named here during the same
//! tick's earlier "systems" phase (`lib.rs`'s tick order), before `deep`
//! ever runs -- the gap is only real in a synthetic harness (like this
//! file's own `rig()`) that never runs that phase at all, where a handful
//! of these (`engine_bleed_pb_auto`, `pack_pb_on`, `eng_gen_pb_on`,
//! `apu_gen_pb_on`, `bat_pb_auto`, `gear_lever_down`, `parking_brake_on`)
//! would then read `false` rather than `Controls::default()`'s documented
//! *on* position. Recorded here rather than worked around, since inventing
//! a graceful default for a boolean with no unwritten-sentinel would be
//! its own small fabrication.
//!
//! ## Frame cost
//!
//! Everything that can be done once is done once:
//!
//! * every variable read is through a `VariableIdentifier` resolved in
//!   [`DeepLayer::new`], never by name;
//! * every variable *written* goes through the `Publisher` cache built in
//!   [`DeepLayer::new`] from [`Deep::published_names`], so the frame loop
//!   resolves a published name in one string comparison and never
//!   allocates;
//! * `XPLMGetWeatherAtLocation`, which X-Plane's own header says is not
//!   for per-frame use, is called at [`WEATHER_INTERVAL_S`] rather than
//!   every frame;
//! * the deep failure id set is built once, from `deep::registry()`, into
//!   a `BTreeSet`;
//! * the per-frame [`Faults`] snapshot takes `crate::failures`' lock
//!   **once** (`failures::active_magnitudes`) and keeps the armed ids that
//!   are deep ones, instead of asking `failures::armed_magnitude(id)` for
//!   each of the several thousand registered deep ids in turn. The result
//!   is identical -- `armed_magnitude` returns `0.0` for an id that is not
//!   in the active set, and `Faults::get` returns `0.0` for an id that is
//!   not in the snapshot -- but it is one lock per frame instead of
//!   thousands.
//!
//! This pass adds roughly 90 more `VariableIdentifier` reads per frame (29
//! surface deflections, ~40 `Controls` fields across 4 engines plus the
//! non-per-engine ones, a handful of engine/cabin/airframe scalars) and 8
//! more `Option<DataRef>` reads through `Xplm::get_f`/`get_i` -- all
//! resolved once in `DeepLayer::new`, same as every existing field, so the
//! added per-frame cost is that many more `Vars::read`/`Xplm::get_f` calls
//! (each an array index plus a float read, no allocation, no lock) and a
//! five-element loop for the touchdown-edge capture. Measured against the
//! ~180 reads `truth()` already made before this pass (four engines' worth
//! of `EngineIds` plus the bus/hydraulic scalars), this is roughly a 50%
//! increase in `truth()`'s own read count, not a new order of magnitude;
//! `Deep::tick`'s own area-stepping cost, not `truth()`, is what dominates
//! a 30-60 Hz frame budget.

use std::collections::{BTreeSet, HashMap};

use systems::simulation::{SimulatorReaderWriter, VariableIdentifier, VariableRegistry};

use crate::deep::integration::weather_truth::{EnvironmentTruth, WeatherTruthReader};
use crate::deep::live::{CommandedSurfaces, Deep, Faults, Truth};
use crate::fadec::EngineState;
use crate::flight_controls::{aileron_or_elevator_down_deg, rudder_right_deg, spoiler_up_deg};
use crate::prim::SimReadings;
use crate::xp::{DataRef, Xplm};
use crate::Vars;

/// psi -> Pa, the same constant `fuel.rs` and the deep areas already use.
const PSI_TO_PA: f64 = 6894.757;

/// The shortest frame the models are stepped with. X-Plane hands out a
/// zero `dt` on the frame a flight loads; several areas integrate against
/// `dt_s` and a few divide by it, so it is floored here at the same 1 ms
/// `lib.rs`'s own flight-loop clamp uses.
pub const MIN_DT_S: f64 = 0.001;
/// The longest. After a pause, a scenery load or a long frame X-Plane
/// reports the whole wall-clock gap; integrating a first-order lag across
/// several seconds in one step is where an exact-exponential model stays
/// stable but an explicit one does not, and no model here is validated
/// beyond a 5 Hz step. Same value as `lib.rs`'s own flight-loop clamp.
pub const MAX_DT_S: f64 = 0.2;

/// ARINC 429 sign/status "normal operation": the sender vouches for the
/// value. FlyByWire packs a 32 bit float's bits in the low half of the
/// `f64` and the status in the two bits above
/// (`fbw-common/.../shared/arinc429.rs:154`, `to_arinc429`).
const SSM_NORMAL_OPERATION: u32 = 3;

/// One ARINC 429 word as FlyByWire packs it: the value and its status.
fn unpack_arinc(packed: f64) -> (f64, u32) {
    let bits = packed as u64;
    (f32::from_bits(bits as u32) as f64, ((bits >> 32) & 0b11) as u32)
}

/// `fadec.rs`'s `EngineState::On`: the core is turning and lit.
const ENGINE_STATE_ON: f64 = 1.0;

/// The variables one engine contributes to [`Truth`].
struct EngineIds {
    n1_pct: VariableIdentifier,
    n2_pct: VariableIdentifier,
    n3_pct: VariableIdentifier,
    state: VariableIdentifier,
    ip_port_pressure_pa: VariableIdentifier,
    ip_port_temp_k: VariableIdentifier,
    hp_port_pressure_pa: VariableIdentifier,
    hp_port_temp_k: VariableIdentifier,
    /// Which customer port this engine is bled from this tick:
    /// `engine_commands.rs` sets `bleed_from_ip_port` to `hp_valve_open ==
    /// 0`, so the same test picks the same port's pressure/temperature
    /// here and the two can never disagree.
    hp_valve_open: VariableIdentifier,
    fuel_flow_demand_kg_s: VariableIdentifier,
    /// `Controls`' per-engine fields: fire pushbutton, both agent
    /// pushbuttons, nacelle anti-ice selection, engine bleed pushbutton,
    /// the engine generator pushbutton, and the four raw reads
    /// `starter_engaged` is recomputed from (see `plugin.rs`'s own
    /// sourcing table).
    fire_pb_released: VariableIdentifier,
    fire_agent_pb_pressed: [VariableIdentifier; 2],
    nacelle_anti_ice_position: VariableIdentifier,
    bleed_pb_auto: VariableIdentifier,
    eng_gen_pb_on: VariableIdentifier,
    master: VariableIdentifier,
    igniter: VariableIdentifier,
    timer: VariableIdentifier,
}

impl EngineIds {
    fn new(vars: &mut Vars, n: usize) -> Self {
        Self {
            n1_pct: vars.get(format!("ENGINE_N1:{n}")),
            n2_pct: vars.get(format!("ENGINE_N2:{n}")),
            n3_pct: vars.get(format!("ENGINE_N3:{n}")),
            state: vars.get(format!("ENGINE_STATE:{n}")),
            ip_port_pressure_pa: vars.get(format!("ENGINE_IP_PORT_PRESSURE_PA:{n}")),
            ip_port_temp_k: vars.get(format!("ENGINE_IP_PORT_TEMP_K:{n}")),
            hp_port_pressure_pa: vars.get(format!("ENGINE_HP_PORT_PRESSURE_PA:{n}")),
            hp_port_temp_k: vars.get(format!("ENGINE_HP_PORT_TEMP_K:{n}")),
            hp_valve_open: vars.get(format!("PNEU_ENG_{n}_HP_VALVE_OPEN")),
            fuel_flow_demand_kg_s: vars.get(format!("ENGINE_FUEL_DEMAND_KG_S:{n}")),
            fire_pb_released: vars.get(format!("FIRE_BUTTON_ENG{n}")),
            fire_agent_pb_pressed: [1, 2].map(|b| vars.get(format!("OVHD_FIRE_AGENT_{b}_ENG_{n}_IS_PRESSED"))),
            nacelle_anti_ice_position: vars.get(format!("BUTTON_OVHD_ANTI_ICE_ENG_{n}_POSITION")),
            bleed_pb_auto: vars.get(format!("OVHD_PNEU_ENG_{n}_BLEED_PB_IS_AUTO")),
            eng_gen_pb_on: vars.get(format!("OVHD_ELEC_ENG_GEN_{n}_PB_IS_ON")),
            master: vars.get(format!("GENERAL ENG STARTER:{n}")),
            igniter: vars.get(format!("TURB ENG IGNITION SWITCH EX1:{n}")),
            timer: vars.get(format!("ENGINE_TIMER:{n}")),
        }
    }
}

/// The 29 `HYD_*_DEFLECTION` Vars `flight_controls.rs::FlightControls::new`
/// also resolves (`flight_controls.rs:258-269`), read independently here so
/// `deep::live` never depends on `flight_controls.rs`'s own private `Ids`.
/// `vars.get` on an already-resolved name returns the same
/// `VariableIdentifier`, so this costs nothing extra at runtime, just a
/// second, harmless resolution at startup.
struct SurfaceIds {
    ailerons: [[VariableIdentifier; 3]; 2],
    elevators: [[VariableIdentifier; 2]; 2],
    rudders: [VariableIdentifier; 2],
    spoilers: [[VariableIdentifier; 8]; 2],
    ths: VariableIdentifier,
}

impl SurfaceIds {
    fn new(vars: &mut Vars) -> Self {
        const SIDES: [&str; 2] = ["LEFT", "RIGHT"];
        Self {
            ailerons: SIDES.map(|side| ["INWARD", "MIDDLE", "OUTWARD"].map(|part| vars.get(format!("HYD_AIL_{side}_{part}_DEFLECTION")))),
            elevators: SIDES.map(|side| ["INWARD", "OUTWARD"].map(|part| vars.get(format!("HYD_ELEV_{side}_{part}_DEFLECTION")))),
            rudders: ["UPPER", "LOWER"].map(|which| vars.get(format!("HYD_{which}_RUD_DEFLECTION"))),
            spoilers: SIDES.map(|side| std::array::from_fn(|i| vars.get(format!("HYD_SPOILER_{}_{side}_DEFLECTION", i + 1)))),
            ths: vars.get("HYD_FINAL_THS_DEFLECTION".to_owned()),
        }
    }
}

/// The non-per-engine [`Controls`] fields' variables, resolved once.
struct ControlIds {
    fire_pb_apu_released: VariableIdentifier,
    fire_agent_pb_apu_pressed: VariableIdentifier,
    wing_anti_ice_position: VariableIdentifier,
    apu_bleed_pb_on: VariableIdentifier,
    cross_bleed_selector: VariableIdentifier,
    pack_pb_on: [VariableIdentifier; 2],
    /// `[nose, left, right]`.
    gear_door_position: [VariableIdentifier; 3],
    gear_handle_position: VariableIdentifier,
    park_brake_lever_pos: VariableIdentifier,
    apu_gen_pb_on: [VariableIdentifier; 2],
    bat_pb_auto: [VariableIdentifier; 2],
    apu_master_sw_on: VariableIdentifier,
    apu_start_pb_on: VariableIdentifier,
    /// `A32NX_EXT_PWR_AVAIL:{1..4}`, for `Truth::gpu_plugged_in`.
    ext_pwr_avail: [VariableIdentifier; 4],
    /// `A32NX_LGCIU_1_{NOSE,LEFT,RIGHT}_GEAR_COMPRESSED`, for
    /// `Truth::leg_on_ground`.
    lgciu_gear_compressed: [VariableIdentifier; 3],
    /// ARINC 429 cabin delta pressure and one representative cabin zone
    /// temperature, for `Truth::cabin_pressure_pa`/`cabin_temp_k`.
    cabin_delta_pressure: VariableIdentifier,
    cabin_temp_c: VariableIdentifier,
}

impl ControlIds {
    fn new(vars: &mut Vars) -> Self {
        Self {
            fire_pb_apu_released: vars.get("FIRE_BUTTON_APU".to_owned()),
            fire_agent_pb_apu_pressed: vars.get("OVHD_FIRE_AGENT_1_APU_1_IS_PRESSED".to_owned()),
            wing_anti_ice_position: vars.get("BUTTON_OVHD_ANTI_ICE_WING_POSITION".to_owned()),
            apu_bleed_pb_on: vars.get("OVHD_APU_BLEED_PB_IS_ON".to_owned()),
            cross_bleed_selector: vars.get("KNOB_OVHD_AIRCOND_XBLEED_Position".to_owned()),
            pack_pb_on: [1, 2].map(|n| vars.get(format!("OVHD_COND_PACK_{n}_PB_IS_ON"))),
            gear_door_position: ["CENTER", "LEFT", "RIGHT"].map(|s| vars.get(format!("GEAR_DOOR_{s}_POSITION"))),
            gear_handle_position: vars.get("GEAR_HANDLE_POSITION".to_owned()),
            park_brake_lever_pos: vars.get("PARK_BRAKE_LEVER_POS".to_owned()),
            apu_gen_pb_on: [1, 2].map(|n| vars.get(format!("OVHD_ELEC_APU_GEN_{n}_PB_IS_ON"))),
            bat_pb_auto: [1, 2].map(|n| vars.get(format!("OVHD_ELEC_BAT_{n}_PB_IS_AUTO"))),
            apu_master_sw_on: vars.get("OVHD_APU_MASTER_SW_PB_IS_ON".to_owned()),
            apu_start_pb_on: vars.get("OVHD_APU_START_PB_IS_ON".to_owned()),
            ext_pwr_avail: [1, 2, 3, 4].map(|n| vars.get(format!("EXT_PWR_AVAIL:{n}"))),
            lgciu_gear_compressed: ["NOSE", "LEFT", "RIGHT"].map(|s| vars.get(format!("LGCIU_1_{s}_GEAR_COMPRESSED"))),
            cabin_delta_pressure: vars.get("PRESS_CPC_1_CABIN_DELTA_PRESSURE".to_owned()),
            cabin_temp_c: vars.get("COND_MAIN_DECK_1_TEMP".to_owned()),
        }
    }
}

/// Every variable [`Truth`] is filled from, resolved once.
struct Ids {
    engines: [EngineIds; 4],
    apu_available: VariableIdentifier,
    apu_bleed_air_pressure: VariableIdentifier,
    ac_bus_potential: [VariableIdentifier; 4],
    dc_bus_potential: [VariableIdentifier; 2],
    /// Green and yellow, in `Truth::hydraulic_pressure_pa`'s order.
    hydraulic_pressure_psi: [VariableIdentifier; 2],
    surfaces: SurfaceIds,
    controls: ControlIds,
}

/// The X-Plane datarefs [`Truth`] is filled from, found once.
struct Refs {
    /// `sim/flightmodel/position/elevation`, MSL metres: X-Plane's own
    /// fundamental position dataref (`weather_truth.rs` reads the same one
    /// for `XPLMGetWeatherAtLocation`'s altitude argument).
    elevation_m: Option<DataRef>,
    /// `sim/flightmodel/failures/onground_any`, the same dataref
    /// `physics/adirs.rs:1522` and `physics/damage.rs:432` read and
    /// `lib.rs`'s `"SIM ON GROUND"` mapping is built on.
    on_ground: Option<DataRef>,
    /// `sim/flightmodel/weight/m_total`, kg.
    mass_kg: Option<DataRef>,
    /// `sim/flightmodel/position/theta`, degrees, positive nose up.
    pitch_deg: Option<DataRef>,
    /// `sim/flightmodel/position/groundspeed`, m/s.
    groundspeed_m_s: Option<DataRef>,
    /// `sim/flightmodel/position/alpha`, degrees.
    alpha_deg: Option<DataRef>,
    /// `sim/cockpit2/gauges/indicators/radio_altimeter_height_ft_pilot`,
    /// the same dataref `prim.rs`'s own `h_radio_ft` reads.
    radio_height_ft: Option<DataRef>,
    /// `sim/flightmodel/position/local_vy`, m/s, X-Plane's OpenGL-frame
    /// vertical speed (positive up) -- for touchdown sink-speed capture.
    local_vy_m_s: Option<DataRef>,
    /// `sim/graphics/scenery/sun_pitch_degrees`.
    sun_pitch_deg: Option<DataRef>,
    /// `sim/cockpit2/controls/speedbrake_ratio`, the same dataref
    /// `Prims::read` uses for `SimReadings::spoilers_armed` -- reused here,
    /// through the same `prim::SimReadings::spoilers_from_xplane`, so the
    /// two can never disagree about whether the lever is armed.
    speedbrake_ratio: Option<DataRef>,
    /// `sim/cockpit2/controls/{left,right}_brake_ratio`, X-Plane's own raw
    /// pedal-input datarefs.
    brake_pedal: [Option<DataRef>; 2],
}

/// A published name resolved to the variable it writes.
///
/// Primed in [`DeepLayer::new`] from [`Deep::published_names`], so the
/// frame loop never allocates and never calls `Vars::get` (which would
/// allocate the name and its `A32NX_` prefix again, every frame, for every
/// published value).
///
/// Two levels. `order` is the sequence of names the priming pass saw:
/// areas publish the same names in the same order every frame, so the
/// n-th call of a frame is the n-th entry, and confirming that is one
/// length-then-bytes string comparison -- measured at 2.4 ns per published
/// value against 12.6 ns for hashing the name, over the 2445 values ten
/// areas publish. `by_name` is the fallback for anything that does not
/// line up (an area that publishes conditionally, or a name the priming
/// pass never saw), and is what keeps the fast path safe to take: a
/// mismatch costs a hash lookup, never a wrong variable.
#[derive(Default)]
struct Publisher {
    order: Vec<(String, VariableIdentifier)>,
    by_name: HashMap<String, VariableIdentifier>,
}

impl Publisher {
    /// The variable `name` writes, and whether the positional cache is
    /// still in step (so the caller can advance it).
    fn resolve(&mut self, vars: &mut Vars, at: usize, name: &str) -> (VariableIdentifier, bool) {
        if let Some((cached, id)) = self.order.get(at) {
            if cached == name {
                return (*id, true);
            }
        }
        if let Some(id) = self.by_name.get(name) {
            return (*id, false);
        }
        // A name neither cache has seen: resolve it once, then never
        // again. Only reachable on the first frame an area publishes it.
        let id = vars.get(name.to_owned());
        self.by_name.insert(name.to_owned(), id);
        (id, false)
    }
}

/// The deep layer as the plugin owns it.
pub struct DeepLayer {
    deep: Deep,
    weather: WeatherTruthReader,
    ids: Ids,
    refs: Refs,
    /// Every failure id `deep::registry()` assigned, built once. Sorted,
    /// so intersecting the (usually tiny) armed set with it is a handful
    /// of binary searches.
    failure_ids: BTreeSet<u64>,
    publisher: Publisher,
    /// The last full weather read, and the time since it was taken.
    ///
    /// `WeatherTruthReader::read` calls `XPLMGetWeatherAtLocation`, which
    /// `XPLMWeather.h` itself says is "not intended to be used per-frame"
    /// (`wxr/sampler.rs` quotes the same line and budgets its own calls for
    /// the same reason). It is taken at [`WEATHER_INTERVAL_S`] and held in
    /// between, so a 30-60 Hz frame loop makes that call ten times a second
    /// rather than sixty. Everything it reads -- air temperature, ambient
    /// pressure, true airspeed, precipitation, cloud layers -- changes far
    /// more slowly than 0.1 s: the fastest of them, TAS in a take-off
    /// acceleration, moves under a tenth of a knot in that time.
    environment: EnvironmentTruth,
    since_weather_s: f64,
    /// Per leg (`nose, l_wing, r_wing, l_body, r_body`), last tick's
    /// `Truth::leg_on_ground`, so `truth()` can see the false -> true edge
    /// that means "just touched down" rather than a level that is already
    /// true every subsequent frame on the ground.
    prev_leg_on_ground: [bool; 5],
    /// The sink speed captured at each leg's last such edge, held until it
    /// next lifts off. See `Truth::leg_touchdown_sink_speed_ms`.
    held_sink_speed_ms: [f64; 5],
}

/// How often the weather/atmosphere read above is taken. See
/// `DeepLayer::environment`.
pub const WEATHER_INTERVAL_S: f64 = 0.1;

impl DeepLayer {
    pub fn new(vars: &mut Vars, xplm: Option<&Xplm>) -> Self {
        let deep = crate::deep::live::all_areas();
        let mut publisher = Publisher::default();
        for name in deep.published_names() {
            let id = vars.get(name.clone());
            publisher.by_name.insert(name.clone(), id);
            publisher.order.push((name, id));
        }
        let failure_ids = crate::deep::registry().failures.iter().map(|f| f.id).collect();
        let ids = Ids {
            engines: [EngineIds::new(vars, 1), EngineIds::new(vars, 2), EngineIds::new(vars, 3), EngineIds::new(vars, 4)],
            apu_available: vars.get("OVHD_APU_START_PB_IS_AVAILABLE".to_owned()),
            apu_bleed_air_pressure: vars.get("APU_BLEED_AIR_PRESSURE".to_owned()),
            ac_bus_potential: [1, 2, 3, 4].map(|n| vars.get(format!("ELEC_AC_{n}_BUS_POTENTIAL"))),
            dc_bus_potential: [1, 2].map(|n| vars.get(format!("ELEC_DC_{n}_BUS_POTENTIAL"))),
            hydraulic_pressure_psi: ["GREEN", "YELLOW"].map(|c| vars.get(format!("HYD_{c}_SYSTEM_1_SECTION_PRESSURE"))),
            surfaces: SurfaceIds::new(vars),
            controls: ControlIds::new(vars),
        };
        let refs = Refs {
            elevation_m: xplm.and_then(|x| x.find("sim/flightmodel/position/elevation")),
            on_ground: xplm.and_then(|x| x.find("sim/flightmodel/failures/onground_any")),
            mass_kg: xplm.and_then(|x| x.find("sim/flightmodel/weight/m_total")),
            pitch_deg: xplm.and_then(|x| x.find("sim/flightmodel/position/theta")),
            groundspeed_m_s: xplm.and_then(|x| x.find("sim/flightmodel/position/groundspeed")),
            alpha_deg: xplm.and_then(|x| x.find("sim/flightmodel/position/alpha")),
            radio_height_ft: xplm.and_then(|x| x.find("sim/cockpit2/gauges/indicators/radio_altimeter_height_ft_pilot")),
            local_vy_m_s: xplm.and_then(|x| x.find("sim/flightmodel/position/local_vy")),
            sun_pitch_deg: xplm.and_then(|x| x.find("sim/graphics/scenery/sun_pitch_degrees")),
            speedbrake_ratio: xplm.and_then(|x| x.find("sim/cockpit2/controls/speedbrake_ratio")),
            brake_pedal: ["left", "right"].map(|s| xplm.and_then(|x| x.find(&format!("sim/cockpit2/controls/{s}_brake_ratio")))),
        };
        Self {
            deep,
            weather: WeatherTruthReader::new(vars, xplm),
            ids,
            refs,
            failure_ids,
            publisher,
            environment: Truth::default().environment,
            // Due immediately, so the first frame reads for real rather
            // than handing the areas the default atmosphere.
            since_weather_s: f64::MAX,
            // A cold aircraft is parked, so every leg starts down; see
            // `Truth::default`'s own `leg_on_ground: [true; 5]`.
            prev_leg_on_ground: [true; 5],
            held_sink_speed_ms: [0.0; 5],
        }
    }

    /// The areas this layer was built with, for the startup log.
    pub fn area_names(&self) -> Vec<&'static str> {
        self.deep.area_names()
    }

    /// This frame's armed deep failures.
    ///
    /// One `crate::failures` lock, then the armed ids that belong to the
    /// deep catalogue. See this module's "Frame cost" note for why this is
    /// exactly `armed_magnitude(id)` over every deep id, without being
    /// thousands of locks.
    fn faults(&self) -> Faults {
        Faults::from_pairs(
            crate::failures::active_magnitudes().into_iter().filter(|(id, _)| self.failure_ids.contains(id)),
        )
    }

    /// Everything the areas read about the rest of the simulation, this
    /// frame. See this module's table for each field's source.
    fn truth(&mut self, vars: &mut Vars, xplm: Option<&Xplm>, delta: f64) -> Truth {
        let default = Truth::default();
        let dt_s = clamp_dt(delta);
        let f = |d: Option<DataRef>| d.and_then(|d| xplm.map(|x| x.get_f(d) as f64));
        self.since_weather_s = (self.since_weather_s + dt_s).min(f64::MAX);
        if self.since_weather_s >= WEATHER_INTERVAL_S {
            self.since_weather_s = 0.0;
            self.environment = self.weather.read(vars, xplm);
            // A missing barometer dataref reads 0.0, which is a vacuum,
            // not a measurement: several areas divide by ambient pressure,
            // so fall back to what `Truth::default()` documents instead.
            if !(self.environment.ambient_pressure_pa > 0.0) {
                self.environment.ambient_pressure_pa = default.environment.ambient_pressure_pa;
            }
        }
        let environment = self.environment;

        let mut engine_n1_frac = [0.0; 4];
        let mut engine_n2_frac = [0.0; 4];
        let mut engine_n3_frac = [0.0; 4];
        let mut engine_running = [false; 4];
        let mut engine_bleed_pressure_pa = default.engine_bleed_pressure_pa;
        let mut engine_bleed_temp_k = default.engine_bleed_temp_k;
        let mut engine_hp_port_pressure_pa = default.engine_hp_port_pressure_pa;
        let mut engine_hp_port_temp_k = default.engine_hp_port_temp_k;
        let mut engine_fuel_flow_kg_s = [0.0; 4];
        let mut controls = default.controls;
        for (i, e) in self.ids.engines.iter().enumerate() {
            engine_n1_frac[i] = vars.read(&e.n1_pct) / 100.0;
            engine_n2_frac[i] = vars.read(&e.n2_pct) / 100.0;
            engine_n3_frac[i] = vars.read(&e.n3_pct) / 100.0;
            engine_running[i] = vars.read(&e.state) == ENGINE_STATE_ON;
            // `engine_commands.rs:466`: the IP port feeds the customer
            // bleed unless the HP valve is open.
            let from_ip = vars.read(&e.hp_valve_open) == 0.0;
            let (pressure, temp) = if from_ip {
                (vars.read(&e.ip_port_pressure_pa), vars.read(&e.ip_port_temp_k))
            } else {
                (vars.read(&e.hp_port_pressure_pa), vars.read(&e.hp_port_temp_k))
            };
            // Before `engine_commands` has written a frame these read 0:
            // an absolute pressure of zero and a temperature of absolute
            // zero are both impossible, so keep the documented cold-engine
            // default rather than hand an area a number no gas can have.
            if pressure > 0.0 {
                engine_bleed_pressure_pa[i] = pressure;
            }
            if temp > 0.0 {
                engine_bleed_temp_k[i] = temp;
            }
            // The HP6 port unconditionally, unlike the pair above (see
            // `Truth::engine_hp_port_pressure_pa`'s own doc).
            let hp_pressure = vars.read(&e.hp_port_pressure_pa);
            let hp_temp = vars.read(&e.hp_port_temp_k);
            if hp_pressure > 0.0 {
                engine_hp_port_pressure_pa[i] = hp_pressure;
            }
            if hp_temp > 0.0 {
                engine_hp_port_temp_k[i] = hp_temp;
            }
            engine_fuel_flow_kg_s[i] = vars.read(&e.fuel_flow_demand_kg_s);

            controls.fire_pb_released[i] = vars.read(&e.fire_pb_released) != 0.0;
            controls.fire_agent_pb_pressed[i] = e.fire_agent_pb_pressed.map(|id| vars.read(&id) != 0.0);
            controls.nacelle_anti_ice_selected[i] = vars.read(&e.nacelle_anti_ice_position) != 0.0;
            controls.engine_bleed_pb_auto[i] = vars.read(&e.bleed_pb_auto) != 0.0;
            controls.eng_gen_pb_on[i] = vars.read(&e.eng_gen_pb_on) != 0.0;
            let master = vars.read(&e.master) != 0.0;
            controls.engine_master_on[i] = master;
            // `physics::engine::mod.rs`'s own `phys_inputs.starter_engaged`
            // formula (`engine_commands.rs:445-448`), recomputed from the
            // same real reads rather than duplicated as a second Var: the
            // igniter selector at IGN START/CRANK (2), the FADEC's own
            // state machine in Starting or Restarting, and past the
            // start-selector dead time.
            let igniter = vars.read(&e.igniter).round() as i32;
            let state = EngineState::from(vars.read(&e.state));
            let timer = vars.read(&e.timer);
            controls.starter_engaged[i] =
                master && igniter == 2 && matches!(state, EngineState::Starting | EngineState::Restarting) && timer >= 1.7;
        }

        let (apu_bleed_psi, apu_bleed_ssm) = unpack_arinc(vars.read(&self.ids.apu_bleed_air_pressure));
        // The ECB only vouches for its word while it is powered; with the
        // APU cold there is no bleed and the port sits at ambient, which
        // is a reading this frame already has.
        let apu_bleed_pressure_pa = if apu_bleed_ssm == SSM_NORMAL_OPERATION && apu_bleed_psi > 0.0 {
            apu_bleed_psi * PSI_TO_PA
        } else {
            environment.ambient_pressure_pa
        };

        {
            let c = &self.ids.controls;
            controls.fire_pb_apu_released = vars.read(&c.fire_pb_apu_released) != 0.0;
            controls.fire_agent_pb_apu_pressed = vars.read(&c.fire_agent_pb_apu_pressed) != 0.0;
            controls.wing_anti_ice_selected = vars.read(&c.wing_anti_ice_position) != 0.0;
            controls.apu_bleed_pb_on = vars.read(&c.apu_bleed_pb_on) != 0.0;
            controls.cross_bleed_selector = vars.read(&c.cross_bleed_selector);
            controls.pack_pb_on = c.pack_pb_on.map(|id| vars.read(&id) != 0.0);
            controls.gear_door_commanded_open = c.gear_door_position.map(|id| vars.read(&id));
            controls.gear_lever_down = vars.read(&c.gear_handle_position) >= 0.5;
            controls.parking_brake_on = vars.read(&c.park_brake_lever_pos) >= 0.5;
            controls.brake_pedal_pos = std::array::from_fn(|i| f(self.refs.brake_pedal[i]).unwrap_or(0.0).clamp(0.0, 1.0));
            controls.apu_gen_pb_on = c.apu_gen_pb_on.map(|id| vars.read(&id) != 0.0);
            controls.bat_pb_auto = c.bat_pb_auto.map(|id| vars.read(&id) != 0.0);
            controls.apu_master_sw_on = vars.read(&c.apu_master_sw_on) != 0.0;
            controls.apu_start_pb_on = vars.read(&c.apu_start_pb_on) != 0.0;
            // `rain_removal_selected` has no real source in this port (see
            // this module's `Controls` sourcing table) and is left at
            // `default.controls`'s value, already copied in above.
        }
        // `sim/cockpit2/controls/speedbrake_ratio`, through the exact
        // function `Prims::read` uses for `SimReadings::spoilers_armed`, so
        // the two can never disagree about whether the lever is armed.
        controls.ground_spoiler_lever_armed =
            f(self.refs.speedbrake_ratio).map_or(default.controls.ground_spoiler_lever_armed, |ratio| SimReadings::spoilers_from_xplane(ratio).0);

        let gpu_plugged_in = self.ids.controls.ext_pwr_avail.iter().any(|id| vars.read(id) != 0.0);

        let s = &self.ids.surfaces;
        let commanded_surfaces = CommandedSurfaces {
            ailerons_deg: s.ailerons.map(|side| side.map(|id| aileron_or_elevator_down_deg(vars.read(&id)))),
            elevators_deg: s.elevators.map(|side| side.map(|id| aileron_or_elevator_down_deg(vars.read(&id)))),
            rudders_deg: s.rudders.map(|id| rudder_right_deg(vars.read(&id))),
            spoilers_deg: s.spoilers.map(|side| side.map(|id| spoiler_up_deg(vars.read(&id)))),
            ths_deg: vars.read(&s.ths),
        };

        let aircraft_mass_kg = f(self.refs.mass_kg).filter(|m| *m > 0.0).unwrap_or(default.aircraft_mass_kg);
        let pitch_deg = f(self.refs.pitch_deg).unwrap_or(default.pitch_deg);
        let groundspeed_m_s = f(self.refs.groundspeed_m_s).unwrap_or(default.groundspeed_m_s);
        let angle_of_attack_deg = f(self.refs.alpha_deg).unwrap_or(default.angle_of_attack_deg);
        let radio_height_ft = f(self.refs.radio_height_ft).unwrap_or(default.radio_height_ft);

        // Per-leg ground contact: the primary LGCIU's real sensors, ANDed
        // with the aircraft-wide flag the same way `Truth::on_ground`'s own
        // consumers already do ("no leg can be on the ground while the
        // aircraft is not"). Wing and body share the one real sensor on
        // their side (see this module's sourcing table).
        let on_ground_now = self
            .refs
            .on_ground
            .and_then(|d| xplm.map(|x| x.get_i(d) != 0))
            .unwrap_or(default.on_ground);
        let lc = &self.ids.controls.lgciu_gear_compressed;
        let (nose, left, right) = (vars.read(&lc[0]) != 0.0, vars.read(&lc[1]) != 0.0, vars.read(&lc[2]) != 0.0);
        let leg_on_ground = [nose, left, right, left, right].map(|compressed| compressed && on_ground_now);

        // Touchdown sink speed: capture the aircraft's own vertical speed
        // on the false -> true edge of each leg, hold it until that leg
        // next lifts off. `local_vy` is positive up; a touchdown is a
        // descent, so this is its magnitude, floored at 0 for a leg that
        // never actually had a downward speed (e.g. it was already on the
        // ground at the previous frame's edge, which cannot happen here
        // since this only fires on the edge itself, but the floor keeps
        // the field from ever reading a spurious negative).
        let local_vy = f(self.refs.local_vy_m_s).unwrap_or(0.0);
        let descent_speed_m_s = (-local_vy).max(0.0);
        let mut leg_touchdown_sink_speed_ms = self.held_sink_speed_ms;
        for i in 0..5 {
            let just_touched_down = leg_on_ground[i] && !self.prev_leg_on_ground[i];
            if just_touched_down {
                leg_touchdown_sink_speed_ms[i] = descent_speed_m_s;
            } else if !leg_on_ground[i] {
                leg_touchdown_sink_speed_ms[i] = 0.0;
            }
        }
        self.prev_leg_on_ground = leg_on_ground;
        self.held_sink_speed_ms = leg_touchdown_sink_speed_ms;

        // Cabin pressure: ambient plus FlyByWire's own primary cabin
        // pressure controller's ARINC 429 delta-pressure word (psi).
        let (cabin_delta_psi, cabin_delta_ssm) = unpack_arinc(vars.read(&self.ids.controls.cabin_delta_pressure));
        let cabin_pressure_pa = if cabin_delta_ssm == SSM_NORMAL_OPERATION {
            environment.ambient_pressure_pa + cabin_delta_psi * PSI_TO_PA
        } else {
            default.cabin_pressure_pa
        };
        let cabin_temp_c = vars.read(&self.ids.controls.cabin_temp_c);
        // 0 K would be a reading nobody has published yet, not a real cabin
        // temperature (see this file's own convention for `engine_bleed_
        // temp_k` above).
        let cabin_temp_k = if cabin_temp_c > -273.15 { cabin_temp_c + 273.15 } else { default.cabin_temp_k };

        let sun_elevation_deg = f(self.refs.sun_pitch_deg).unwrap_or(default.sun_elevation_deg);

        Truth {
            dt_s,
            // `Deep::tick` replaces this with the previous frame's values
            // before it steps anything; the plugin never fills it.
            published: Default::default(),
            altitude_ft: f(self.refs.elevation_m).map_or(default.altitude_ft, |m| m * crate::M_TO_FT),
            on_ground: on_ground_now,
            environment,
            engine_n1_frac,
            engine_running,
            engine_bleed_pressure_pa,
            engine_bleed_temp_k,
            apu_running: vars.read(&self.ids.apu_available) != 0.0,
            apu_bleed_pressure_pa,
            ac_bus_volts: std::array::from_fn(|i| vars.read(&self.ids.ac_bus_potential[i])),
            dc_bus_volts: std::array::from_fn(|i| vars.read(&self.ids.dc_bus_potential[i])),
            hydraulic_pressure_pa: std::array::from_fn(|i| vars.read(&self.ids.hydraulic_pressure_psi[i]) * PSI_TO_PA),
            engine_n2_frac,
            engine_n3_frac,
            engine_hp_port_pressure_pa,
            engine_hp_port_temp_k,
            engine_fuel_flow_kg_s,
            gpu_plugged_in,
            controls,
            commanded_surfaces,
            aircraft_mass_kg,
            pitch_deg,
            groundspeed_m_s,
            angle_of_attack_deg,
            radio_height_ft,
            leg_on_ground,
            leg_touchdown_sink_speed_ms,
            cabin_pressure_pa,
            cabin_temp_k,
            sun_elevation_deg,
        }
    }

    /// Step every area and publish what they expose, once per frame.
    pub fn tick(&mut self, vars: &mut Vars, xplm: Option<&Xplm>, delta: f64) {
        let truth = self.truth(vars, xplm, delta);
        let faults = self.faults();
        let Self { deep, publisher, .. } = self;
        let mut at = 0usize;
        deep.tick(truth, &faults, &mut |name, value| {
            let (id, in_step) = publisher.resolve(vars, at, name);
            at += in_step as usize;
            vars.write(&id, value);
        });

        // Level 2 of `docs/deep/authority.md`: where a deep area has
        // concluded that a component FlyByWire *also* models has failed,
        // tell FlyByWire's own failure system so its coarse solve follows.
        // That is what makes the deep models the aircraft rather than a
        // commentary on it -- nothing inside `a380_systems` reads the
        // variables we publish, but it does read its own failures.
        //
        // A level, re-stated every frame, never an event: an area emits
        // its whole coupling table with the healthy entries at zero, so a
        // component that recovers clears itself. Kept apart from the
        // crew's own arming, so a derived failure is never saved to the
        // airframe file as though a pilot had armed it.
        crate::failures::set_derived_levels(deep.derived_magnitudes());
    }
}

/// This frame's length as the models may be stepped with it. A `dt` that
/// is not a finite positive number at all (X-Plane's zero on the frame a
/// flight loads) becomes [`MIN_DT_S`], not zero: an area that divides by
/// `dt_s` must never see one it cannot divide by.
pub fn clamp_dt(delta: f64) -> f64 {
    if delta.is_finite() {
        delta.clamp(MIN_DT_S, MAX_DT_S)
    } else {
        MIN_DT_S
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_frame_time_is_never_zero_never_huge_and_never_nan() {
        // The two real cases: X-Plane's zero dt on the frame a flight
        // loads, and the whole wall-clock gap after a pause.
        assert_eq!(clamp_dt(0.0), MIN_DT_S);
        assert_eq!(clamp_dt(12.0), MAX_DT_S);
        assert_eq!(clamp_dt(-1.0), MIN_DT_S);
        assert_eq!(clamp_dt(f64::NAN), MIN_DT_S);
        assert_eq!(clamp_dt(f64::INFINITY), MIN_DT_S);
        assert_eq!(clamp_dt(1.0 / 60.0), 1.0 / 60.0);
    }

    #[test]
    fn an_arinc_word_unpacks_the_way_flybywire_packs_it() {
        // FlyByWire's `to_arinc429`: the f32's bits, the status two bits
        // above. A running APU's 50 psi with the ECB vouching for it.
        let packed = (((SSM_NORMAL_OPERATION as u64) << 32) | 50.0f32.to_bits() as u64) as f64;
        let (value, ssm) = unpack_arinc(packed);
        assert_eq!(ssm, SSM_NORMAL_OPERATION);
        assert!((value - 50.0).abs() < 1e-6, "{value}");
        // A word nobody wrote is not a pressure of zero psi with a good
        // status; it is failure-warning, which `truth` falls back from.
        assert_eq!(unpack_arinc(0.0), (0.0, 0));
    }

    /// A `Vars` off a do-nothing X-Plane binding, the same way
    /// `fadec.rs`'s own tests build one: `find` answers "no such dataref",
    /// so every `Refs` entry is `None` and every `Truth` field that has no
    /// variable written under it falls back to its documented default --
    /// which is exactly what these tests are checking.
    fn rig() -> (&'static Xplm, Vars) {
        let xplm: &'static Xplm = Box::leak(Box::new(Xplm::dummy()));
        (xplm, Vars::new(xplm))
    }

    #[test]
    fn a_truth_with_nothing_written_is_the_documented_default_not_zero() {
        let (xplm, mut vars) = rig();
        let mut layer = DeepLayer::new(&mut vars, Some(xplm));
        let t = layer.truth(&mut vars, Some(xplm), 0.0);
        let d = Truth::default();
        // A vacuum, absolute zero and a dead engine's bleed port at 0 Pa
        // are all numbers no area may be handed just because a dataref or
        // a variable has not been written yet.
        assert_eq!(t.environment.ambient_pressure_pa, d.environment.ambient_pressure_pa);
        assert_eq!(t.engine_bleed_pressure_pa, d.engine_bleed_pressure_pa);
        assert_eq!(t.engine_bleed_temp_k, d.engine_bleed_temp_k);
        assert_eq!(t.apu_bleed_pressure_pa, d.environment.ambient_pressure_pa);
        assert_eq!(t.altitude_ft, d.altitude_ft);
        assert_eq!(t.on_ground, d.on_ground);
        assert_eq!(t.dt_s, MIN_DT_S, "X-Plane's zero dt on the frame a flight loads");
        // These genuinely are zero on a cold aircraft, and FlyByWire
        // publishes them as zero, so zero is the reading, not a gap.
        assert_eq!(t.engine_n1_frac, [0.0; 4]);
        assert_eq!(t.engine_running, [false; 4]);
        assert!(!t.apu_running);
        assert_eq!(t.ac_bus_volts, [0.0; 4]);
        assert_eq!(t.hydraulic_pressure_pa, [0.0; 2]);
    }

    #[test]
    fn the_weather_read_is_taken_on_the_first_frame_and_then_at_its_own_interval() {
        // X-Plane's own header: XPLMGetWeatherAtLocation is "not intended
        // to be used per-frame". The first frame must still read for real
        // rather than hand the areas the default atmosphere.
        let (xplm, mut vars) = rig();
        let mut layer = DeepLayer::new(&mut vars, Some(xplm));
        let sat = vars.get("AMBIENT TEMPERATURE".to_owned());
        vars.write(&sat, -40.0);
        assert_eq!(layer.truth(&mut vars, Some(xplm), 1.0 / 60.0).environment.sat_c, -40.0);
        // Inside the interval the held reading stands, unchanged.
        vars.write(&sat, 20.0);
        let held = ((WEATHER_INTERVAL_S * 60.0) as usize).saturating_sub(1);
        for _ in 0..held {
            assert_eq!(layer.truth(&mut vars, Some(xplm), 1.0 / 60.0).environment.sat_c, -40.0, "a held reading must not change inside the interval");
        }
        // And past it the next read lands, within a frame or two of the
        // interval (exactly which frame is float accumulation, and not
        // something worth pinning a test to).
        let mut fresh = None;
        for _ in 0..3 {
            fresh = Some(layer.truth(&mut vars, Some(xplm), 1.0 / 60.0).environment.sat_c);
            if fresh == Some(20.0) {
                break;
            }
        }
        assert_eq!(fresh, Some(20.0), "the reading must be taken again once the interval has passed");
    }

    #[test]
    fn truth_reads_the_port_the_engine_is_actually_bled_from() {
        // `engine_commands.rs` bleeds the IP port unless the HP valve is
        // open; reading the other one would hand the pneumatic areas a
        // pressure the engine is not actually delivering.
        let (xplm, mut vars) = rig();
        let mut layer = DeepLayer::new(&mut vars, Some(xplm));
        for n in 1..=4 {
            let ip_p = vars.get(format!("ENGINE_IP_PORT_PRESSURE_PA:{n}"));
            let ip_t = vars.get(format!("ENGINE_IP_PORT_TEMP_K:{n}"));
            let hp_p = vars.get(format!("ENGINE_HP_PORT_PRESSURE_PA:{n}"));
            let hp_t = vars.get(format!("ENGINE_HP_PORT_TEMP_K:{n}"));
            vars.write(&ip_p, 300_000.0);
            vars.write(&ip_t, 500.0);
            vars.write(&hp_p, 900_000.0);
            vars.write(&hp_t, 700.0);
        }
        let hp_valve_2 = vars.get("PNEU_ENG_2_HP_VALVE_OPEN".to_owned());
        vars.write(&hp_valve_2, 1.0);
        let t = layer.truth(&mut vars, Some(xplm), 1.0 / 60.0);
        assert_eq!(t.engine_bleed_pressure_pa, [300_000.0, 900_000.0, 300_000.0, 300_000.0]);
        assert_eq!(t.engine_bleed_temp_k, [500.0, 700.0, 500.0, 500.0]);
    }

    #[test]
    fn flybywires_own_psi_and_arinc_readings_arrive_in_si() {
        let (xplm, mut vars) = rig();
        let mut layer = DeepLayer::new(&mut vars, Some(xplm));
        let green = vars.get("HYD_GREEN_SYSTEM_1_SECTION_PRESSURE".to_owned());
        let yellow = vars.get("HYD_YELLOW_SYSTEM_1_SECTION_PRESSURE".to_owned());
        vars.write(&green, 5000.0);
        vars.write(&yellow, 0.0);
        let apu_p = vars.get("APU_BLEED_AIR_PRESSURE".to_owned());
        vars.write(&apu_p, (((SSM_NORMAL_OPERATION as u64) << 32) | 50.0f32.to_bits() as u64) as f64);
        let apu_avail = vars.get("OVHD_APU_START_PB_IS_AVAILABLE".to_owned());
        vars.write(&apu_avail, 1.0);
        let t = layer.truth(&mut vars, Some(xplm), 1.0 / 60.0);
        // The A380's 5000 psi systems, as FlyByWire publishes them.
        assert!((t.hydraulic_pressure_pa[0] - 5000.0 * PSI_TO_PA).abs() < 1.0, "{:?}", t.hydraulic_pressure_pa);
        assert_eq!(t.hydraulic_pressure_pa[1], 0.0);
        assert!(t.apu_running);
        assert!((t.apu_bleed_pressure_pa - 50.0 * PSI_TO_PA).abs() < 1.0, "{}", t.apu_bleed_pressure_pa);
    }

    #[test]
    fn a_published_name_resolves_to_the_same_variable_in_order_or_out_of_it() {
        // The positional cache is only an optimisation: whichever path a
        // name takes, it must reach the same variable, or an area's
        // published value would land on the wrong one.
        let (_xplm, mut vars) = rig();
        let mut p = Publisher::default();
        for name in ["DEEP_TEST_A", "DEEP_TEST_B", "DEEP_TEST_C"] {
            let id = vars.get(name.to_owned());
            p.by_name.insert(name.to_owned(), id);
            p.order.push((name.to_owned(), id));
        }
        let expected: Vec<_> = p.order.iter().map(|(_, id)| *id).collect();
        // In order: every call takes the fast path and advances.
        let mut at = 0;
        for (i, name) in ["DEEP_TEST_A", "DEEP_TEST_B", "DEEP_TEST_C"].iter().enumerate() {
            let (id, in_step) = p.resolve(&mut vars, at, name);
            assert!(in_step);
            assert_eq!(id, expected[i]);
            at += 1;
        }
        // Out of order, and a name no priming pass saw: still correct.
        let (id, in_step) = p.resolve(&mut vars, 0, "DEEP_TEST_C");
        assert!(!in_step);
        assert_eq!(id, expected[2]);
        let (new_id, in_step) = p.resolve(&mut vars, 0, "DEEP_TEST_NEW");
        assert!(!in_step);
        assert_eq!(new_id, vars.get("DEEP_TEST_NEW".to_owned()), "a name seen late must resolve to the same variable");
        assert!(!expected.contains(&new_id));
    }

    #[test]
    fn the_deep_failure_catalogue_is_a_set_of_unique_ids() {
        // `faults()` looks an armed id up in this set, so a duplicate id
        // would silently give one failure two meanings.
        let failures = crate::deep::registry().failures;
        let unique: BTreeSet<u64> = failures.iter().map(|f| f.id).collect();
        assert_eq!(unique.len(), failures.len(), "{} of {} ids are duplicates", failures.len() - unique.len(), failures.len());
    }
}
