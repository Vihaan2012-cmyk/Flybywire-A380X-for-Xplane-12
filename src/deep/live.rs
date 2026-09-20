//! The live layer: what turns the models in this directory from code that
//! compiles into systems that run.
//!
//! Every area under `deep/` models its physics as free functions and small
//! structs with no dependency on `crate::Vars` or X-Plane (`docs/deep/
//! BRIEF.md` hard rule 2), which is what let eighteen areas be written
//! independently. The cost is that nothing owns an instance of any of it:
//! there is no `HydraulicNetwork` anywhere in the running plugin, only the
//! type. This module is the seam that fixes that, without giving the areas
//! the dependency the rule exists to prevent.
//!
//! Three pieces:
//!
//! * [`Truth`] -- everything an area may read about the rest of the
//!   simulation this tick. The plugin fills it once per frame from
//!   X-Plane, from its own engine model and from FlyByWire's published
//!   variables; areas read it and never reach for a variable themselves.
//! * [`Faults`] -- the armed magnitude of every failure, by the id
//!   `deep::api` assigned it. A snapshot, taken by the plugin from
//!   `crate::failures`, so an area never depends on the failure system
//!   either. Every failure in the catalogue is a *continuous* magnitude in
//!   0..1, so an area asks for a number, not for whether something is
//!   "broken".
//! * [`Area`] -- what an area's live system implements: step yourself
//!   forward, then publish what the rest of the aircraft can see, and say
//!   which of FlyByWire's own failures the area has concluded are real
//!   ([`Area::derived_failures`]).
//!
//! [`Deep`] owns one live system per area and is what `Plugin::tick`
//! actually calls.
//!
//! ## Authority
//!
//! Three systems -- electrical, hydraulic, pneumatic -- are modelled twice:
//! once coarsely and completely by FlyByWire's ported `a380_systems`, once
//! finely and partially under `deep/`. `docs/deep/authority.md` is the
//! design for which of the two is the aircraft; the short version is that
//! **the deep model is authoritative, and expresses its authority through
//! the coarsest FlyByWire input that can carry the verdict**.
//!
//! That input is FlyByWire's own failure system, which `crate::failures`
//! already drives. When a deep area concludes that a component FlyByWire
//! *also* models has failed -- a generator, a TR, a bus, an engine-driven
//! pump, a bleed valve -- it does not argue with FlyByWire about voltages
//! or pressures: it reports a [`DerivedFailure`], [`Deep::tick`] collects
//! it, and the plugin hands it to `crate::failures` next to the failures
//! the crew armed from the EFB. FlyByWire then re-solves, its pages show
//! it, and every consumer inside `a380_systems` sees it -- one aircraft,
//! one answer.
//!
//! Everything below FlyByWire's resolution (per-load current, per-feeder
//! breaker state, I^2t heating, arc energy) needs none of this: nothing
//! competes with it, so the area simply publishes it. Everything above the
//! deep model stays FlyByWire's, and the areas keep reading it from
//! [`Truth`].
//!
//! ## Publishing
//!
//! An area publishes through a closure (`&mut dyn FnMut(&str, f64)`)
//! rather than by being handed the variable registry. That keeps the rule
//! intact -- the area names a variable and a value, and the plugin decides
//! what a variable *is* -- and it means the same area can publish into a
//! test harness, into a recording, or into nothing at all, without a
//! second code path. The names are the ones each area's `registry.rs`
//! already cites in its ECAM triggers, so an alert's trigger and the value
//! it reads cannot drift apart.
//!
//! ## Ordering
//!
//! Areas are stepped in the order they appear in [`Deep::tick`], and an
//! area reads the previous frame's values of anything another area
//! publishes. That one-frame lag is deliberate and is the same lag
//! `extra_backend_fbw.rs` already accepts on the reverser force path: it
//! makes the areas independent of each other's order, which is what keeps
//! them separately testable. At 30-60 Hz it is below the time constant of
//! every process modelled here.

use std::collections::BTreeMap;

use super::integration::weather_truth::EnvironmentTruth;

/// What every area published on the previous frame, by variable name.
///
/// This is how one area reads another's output. The areas are deliberately
/// independent -- none may call into another, so that each stays separately
/// testable -- but the aircraft is not: a pneumatic duct's overheat loop
/// watches the temperature of the bay it runs through, and that bay is the
/// thermal area's to compute. Passing the previous frame's published values
/// back in keeps the areas decoupled while letting the physics join up.
///
/// A name nobody published reads as `None`, never as zero: an area must be
/// able to tell "the bay is at 0 C" from "nothing models that bay".
#[derive(Clone, Debug, Default)]
pub struct PublishedFrame(pub BTreeMap<String, f64>);

impl PublishedFrame {
    /// What another area published for `name` last frame, if anything did.
    pub fn get(&self, name: &str) -> Option<f64> {
        self.0.get(name).copied()
    }

    /// `get`, with a caller-chosen stand-in for "nobody models this yet".
    /// The fallback belongs to the caller because only it knows what a
    /// physically sane substitute is -- ambient for a duct pressure, say,
    /// never zero.
    pub fn get_or(&self, name: &str, fallback: f64) -> f64 {
        self.get(name).unwrap_or(fallback)
    }

    pub fn is_empty(&self) -> bool {
        self.0.is_empty()
    }
}

/// The doors [`Truth::door_open_fraction`] carries, in its own order.
///
/// The six passenger doors and both cargo doors `deep::sensors`'
/// `registry.rs` registers proximity sensors for, under the same names it
/// uses (`["M1L", "M2L", "M2R", "M4L", "M5L", "U1L", "Cargo :16",
/// "Cargo :17"]`) -- the two cargo doors spelled here as `src/doors.rs`'s
/// own `NAMES` spells interactive points 16 and 17, which is what they are
/// called everywhere else in this crate.
///
/// These are the doors the plugin has a real mechanical position for; the
/// fuel hose and the ground power connection are interactive points too
/// but are not doors and are not carried.
pub const DOOR_NAMES: [&str; 8] = ["M1L", "M2L", "M2R", "M4L", "M5L", "U1L", "CARGO_FWD", "CARGO_AFT"];

/// Everything the deep areas read about the rest of the simulation.
///
/// Filled once per frame by the plugin. Anything an area needs that is not
/// here has to be added here first -- that is the point, so there is one
/// list of what the models depend on rather than eighteen.
#[derive(Clone, Debug)]
pub struct Truth {
    /// This frame's length. Never zero, never negative; the plugin clamps
    /// it, because X-Plane hands out a zero `dt` on the frame a flight
    /// loads and a very large one after a pause.
    pub dt_s: f64,
    /// Real weather and atmosphere, as `integration::weather_truth` reads
    /// it from X-Plane.
    pub environment: EnvironmentTruth,
    pub altitude_ft: f64,
    pub on_ground: bool,
    /// Per engine, 1-4: fan speed as a fraction of take-off N1, and
    /// whether the core is turning and lit.
    pub engine_n1_frac: [f64; 4],
    pub engine_running: [bool; 4],
    /// Per engine: bleed air available at the pylon, from this crate's own
    /// engine model (`physics::engine`'s IP8/HP6 port outputs).
    pub engine_bleed_pressure_pa: [f64; 4],
    pub engine_bleed_temp_k: [f64; 4],
    /// Per engine: indicated oil pressure at the bearing feed manifold
    /// (Pa above the chamber vents) and indicated tank oil temperature,
    /// from this crate's own `physics::engine::oil` model.
    pub engine_oil_pressure_pa: [f64; 4],
    pub engine_oil_temp_c: [f64; 4],
    /// Per engine: oil left in the tank as a fraction of a full servicing,
    /// 1.0 full .. 0.0 dry -- the true level, not a probe's reading of it.
    ///
    /// `physics::engine::oil` carries a real tank volume with the two
    /// paths that empty it: consumption past the bearing chambers' carbon
    /// seals (a share of the oil being jetted, so nothing is consumed with
    /// the engine stopped), and a leak in the pressurised feed gallery,
    /// which is an orifice and so runs at the gallery pressure behind it.
    /// Once the level uncovers the pump's inlet, `engine_oil_pressure_pa`
    /// follows it down -- the order the real fault develops in.
    pub engine_oil_quantity_fraction: [f64; 4],
    /// Per engine: turbine gas temperature, C -- the IP-LP interstage
    /// plane, which is the Trent's TGT station.
    ///
    /// This is what a *thermocouple harness* senses, not the bare gas
    /// station. `physics::engine::gas_path` computes `tt45_k` at that
    /// plane, but `hot_section.rs` sits between the gas and the probe: a
    /// thermocouple in a fast gas stream reads the gas, in stagnant gas it
    /// settles to the metal around it, so the probe sees the two blended
    /// by flow -- plus the thermocouple's own first-order response. A
    /// harness averages probes, so the probe temperature is the honest
    /// input to one; `tt45_k` is a station no object in the engine is at.
    ///
    /// Deliberately **not** `A32NX_ENG_n_EEC_TGT_SELECTED`, the EEC's
    /// *voted sensor output*: feeding that into the harness that produces
    /// it is a loop, not a measurement. Not the cockpit's `ENGINE_EGT:n`
    /// either, which carries the EEC's TGT trim (EASA.E.012 Note 16) on
    /// top of the measurement.
    pub engine_tgt_c: [f64; 4],
    /// Per engine: station 2.5, the HP compressor inlet, C -- the IP
    /// compressor's own exit total temperature out of the gas path
    /// (`gas_path`'s `tt25_k`), read unconditionally.
    ///
    /// Not `engine_bleed_temp_k`, which carries whichever of IP8/HP6 is
    /// feeding the customer bleed this tick: calling that station 2.5
    /// would be relabelling a signal that changes port under the reader.
    pub engine_t25_c: [f64; 4],
    /// Tyre inflation pressure per wheel, Pa absolute, from this crate's
    /// own `physics::tyre` model (nitrogen, Gay-Lussac with carcass
    /// temperature).
    ///
    /// All 22 of the aircraft's tyres, in `physics::tyre`'s own wheel
    /// order ([`crate::physics::tyre::WHEEL_NAMES`]): 0..16 are the four
    /// main legs' braked wheels -- unchanged, so every existing wheel map
    /// still indexes the wheel it always did -- then 16..18 the nose pair
    /// and 18..22 the two body legs' unbraked rear axles.
    pub tyre_pressure_pa: [f64; crate::physics::tyre::WHEELS],
    /// True mechanical open fraction per door, 0 shut .. 1 fully open.
    /// Not a latch indication -- a proximity sensor driven off a latch
    /// would be sensing another sensor.
    ///
    /// In [`DOOR_NAMES`]' order. The travel is `src/doors.rs`'s own door
    /// model: each interactive point moving at the rate
    /// `flight_model.cfg` gives it, with the handle's cabin-differential
    /// interlock, and taking a position written from outside as where the
    /// door is.
    pub door_open_fraction: [f64; DOOR_NAMES.len()],
    /// APU: running, and its bleed available at the valve.
    pub apu_running: bool,
    pub apu_bleed_pressure_pa: f64,
    /// Bus voltages FlyByWire's own electrical system published last
    /// frame, so the areas that consume power agree with what the crew
    /// sees on the ELEC page rather than running a second, disagreeing
    /// electrical model.
    ///
    /// These are **not** the truth the deep areas defer to. Since
    /// `docs/deep/authority.md` they are FlyByWire's answer *after* it has
    /// been told what the deep electrical model concluded (see
    /// [`Area::derived_failures`]); the areas read them to stay consistent
    /// with the coarse solve, not because the coarse solve outranks them.
    pub ac_bus_volts: [f64; 4],
    pub dc_bus_volts: [f64; 2],
    /// Hydraulic system pressures, green and yellow, Pa -- FlyByWire's own,
    /// with the same standing as `ac_bus_volts` above.
    pub hydraulic_pressure_pa: [f64; 2],
    /// Per engine, 1-4: intermediate-pressure (N2) and high-pressure (N3)
    /// spool speed, each as a fraction of that spool's own design speed --
    /// the same convention `engine_n1_frac` already uses. Hydraulic
    /// engine-driven pumps and engine fuel pumps are geared to the HP
    /// spool, not the fan, and a VFG's output frequency tracks core (N3)
    /// speed; `engine_n1_frac` alone cannot stand in for either. This
    /// crate's own `physics::engine` already computes both every tick,
    /// immediately alongside N1 (`engine_commands.rs`'s `vars.write(&e.n2,
    /// ...)`/`(&e.n3, ...)`, right after the N1 write `engine_n1_frac`
    /// already cites).
    pub engine_n2_frac: [f64; 4],
    pub engine_n3_frac: [f64; 4],
    /// Per engine: the HP compressor exit (HP6) port, *always* -- unlike
    /// `engine_bleed_pressure_pa`/`_temp_k` above, which already carry
    /// whichever of IP8/HP6 is actually feeding the customer bleed this
    /// tick and so read as IP8 (a much cooler, lower-pressure tap) whenever
    /// the HP valve is shut. The HP6 stuck-valve failure and the
    /// precooler's own cooling duty both need the HP6 reading *even when*
    /// the engine is being bled from IP8, which is why this is a separate
    /// pair of fields rather than a flag on the existing one. Same source
    /// as the port-selection logic `engine_bleed_pressure_pa` already
    /// documents (`physics::engine`'s own HP6 output, `engine_commands.rs`'s
    /// `e.hp_port_pressure`/`e.hp_port_temp`), just read unconditionally.
    pub engine_hp_port_pressure_pa: [f64; 4],
    pub engine_hp_port_temp_k: [f64; 4],
    /// Per engine: real fuel flow into the combustor, kg/s --
    /// `physics::engine`'s own `EngineOutputs::fuel_flow_kg_s`, the same
    /// number `ENGINE_FF:n` (kg/h) and `ENGINE_FUEL_DEMAND_KG_S:n` (kg/s)
    /// already publish every tick. There is no separate *commanded* Wf,
    /// N2, TGT or P30 in this port to carry alongside it: the compiled
    /// FADEC bus (`fbw_controllers::BaseEec`) only exposes a commanded
    /// *N1* (`AUTOTHRUST_N1_COMMANDED:n`, already real and readable by any
    /// area that wants a target to compare N1 against) -- N2/N3, TGT
    /// (EGT) and fuel flow are this model's *response* to that command,
    /// not independently commanded setpoints, so a "commanded" version of
    /// them would be invented. See `docs/deep/truth-requests.md` for this
    /// noted as unsourced rather than guessed.
    pub engine_fuel_flow_kg_s: [f64; 4],
    /// External (ground) power plugged in and available at the aircraft's
    /// receptacle -- any of its four connections. `deep::electrical` has
    /// wanted this since `sources.rs`/`live.rs` were written (its own
    /// `command_contactors` carries a `let gpu_plugged_in = false;` with a
    /// comment asking for exactly this field). Sourced from `EXT_PWR_AVAIL:
    /// {1..4}`, the same real, plugin-managed Var the EFB's own ground-power
    /// control (`efb.rs::any_ext_pwr_available`) and the cold-start setting
    /// already read and write -- not an X-Plane-native dataref, but a real
    /// state this plugin is the sole authority over, same tier as
    /// `on_ground`.
    pub gpu_plugged_in: bool,
    /// What the crew has selected on the overhead, pedestal and centre
    /// panels. See [`Controls`] for each field's real source.
    pub controls: Controls,
    /// What PRIM/SEC commanded each flight-control surface to, this tick,
    /// before any physical fault: FlyByWire's own (undamaged) actuator
    /// model's output, read back from the same `HYD_*_DEFLECTION` Vars
    /// `deep::flight_controls`'s `SurfaceOverrideWriter` will later override
    /// (`deep::live`'s own tick order runs before that override, so this
    /// reads FlyByWire's command, never the deep model's own physical
    /// output from a moment ago). See [`CommandedSurfaces`].
    pub commanded_surfaces: CommandedSurfaces,
    /// The aircraft's own mass and motion, real X-Plane readings. Mass in
    /// particular cannot honestly default to zero -- see [`Truth::default`].
    pub aircraft_mass_kg: f64,
    /// Pitch attitude, degrees, positive nose up (`sim/flightmodel/
    /// position/theta`'s own native sign; `lib.rs`'s MSFS-compatibility
    /// `"PLANE PITCH DEGREES"` mapping negates the same dataref to match
    /// MSFS's opposite convention, which is why that sign looks flipped
    /// there and not here).
    pub pitch_deg: f64,
    pub groundspeed_m_s: f64,
    /// Angle of attack, degrees (`sim/flightmodel/position/alpha`, the
    /// X-Plane SDK's own AoA dataref).
    pub angle_of_attack_deg: f64,
    pub radio_height_ft: f64,
    /// Per leg, in `deep::gear_structure`'s own `nose, l_wing, r_wing,
    /// l_body, r_body` order: whether that leg's wheels are on the ground
    /// this tick, and the aircraft's own vertical speed (m/s, positive
    /// down) at the instant that leg last transitioned from airborne to on
    /// the ground -- held at that value until the leg next lifts off, 0.0
    /// while it has never yet touched down this flight. `on_ground` above
    /// is one aircraft-wide flag with no sink speed at all, which is the
    /// single number a hard-landing model is most sensitive to; see
    /// `gear_structure::live`'s own module doc for exactly this gap.
    pub leg_on_ground: [bool; 5],
    pub leg_touchdown_sink_speed_ms: [f64; 5],
    /// Cabin pressure, Pa, and one representative cabin zone's temperature,
    /// K. See `plugin.rs`'s sourcing table for why these are a single
    /// number each rather than per-zone.
    pub cabin_pressure_pa: f64,
    pub cabin_temp_k: f64,
    /// The sun's elevation above the horizon, degrees (negative below it).
    /// `ThermalNetwork::step` takes a solar flux and every zone carries a
    /// sun-exposure fraction; `fire_ice`'s live system passes 0 today
    /// rather than inventing a flux, per its own module doc. Elevation
    /// (not irradiance itself) is what is real and X-Plane-native; an area
    /// wanting a flux still has to turn this into one itself (clear-sky
    /// irradiance is a function of elevation and the atmosphere this Var
    /// does not carry), which is why this is elevation, not an invented
    /// W/m^2 number.
    pub sun_elevation_deg: f64,
    /// What every area published last frame. Empty on the first frame and
    /// whenever an area has not published a name yet, so read it through
    /// `get`/`get_or` and never assume a zero means anything.
    pub published: PublishedFrame,
}

/// Cockpit control state: what the crew has selected, not what the systems
/// are doing about it. Grouped separately from `Truth`'s other fields
/// because this is the largest single block of them (docs/deep/
/// truth-requests.md's "Cockpit control state" section) and because every
/// one of them is a switch or lever position rather than a physical
/// quantity -- keeping them together makes that distinction visible at the
/// call site (`truth.controls.parking_brake_on`, not thirty more fields
/// flattened onto `Truth` itself).
///
/// Every field's real source (or, where none exists in this port, its
/// documented default and why) is in `plugin.rs`'s own sourcing table.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Controls {
    /// Engine fire pushbutton, per engine: `true` once pulled ("released").
    pub fire_pb_released: [bool; 4],
    pub fire_pb_apu_released: bool,
    /// Fire agent (extinguisher bottle) pushbutton, per engine and bottle
    /// (1st/2nd shot): `true` while pressed.
    pub fire_agent_pb_pressed: [[bool; 2]; 4],
    pub fire_agent_pb_apu_pressed: bool,
    /// No real cargo-bay fire pushbutton or agent pushbutton exists in this
    /// port (`fire_and_smoke_protection.rs` models 8 engine bottles and 1
    /// APU bottle, no cargo ones) -- left out entirely rather than adding a
    /// field with nothing real behind it. See `docs/deep/truth-requests.md`.
    pub wing_anti_ice_selected: bool,
    pub nacelle_anti_ice_selected: [bool; 4],
    pub engine_bleed_pb_auto: [bool; 4],
    pub apu_bleed_pb_on: bool,
    /// The cross-bleed selector knob's raw position: 0 = SHUT, 1 = AUTO,
    /// 2 = OPEN (`CrossBleedValveSelectorMode`'s own discriminants). A
    /// single knob controls all three cross-bleed valves.
    pub cross_bleed_selector: f64,
    pub pack_pb_on: [bool; 2],
    /// Whether each engine's pneumatic starter is actually engaged this
    /// tick: master on, igniter at IGN START/CRANK, and the FADEC's own
    /// state machine past the start-selector dead time -- the same
    /// condition `physics::engine::mod.rs`'s own `phys_inputs.starter_
    /// engaged` computes internally for the physical engine model, just
    /// exposed here for `pneumatic_ducts`' start-duct failures too.
    pub starter_engaged: [bool; 4],
    /// No real rain-removal pushbutton exists in this port either; held at
    /// its normal (off) position. `[left, right]` windshield jets.
    pub rain_removal_selected: [bool; 2],
    /// Commanded gear door position, `[nose, left, right]`, FlyByWire's own
    /// actuator output: 0.0 closed .. 1.0 fully open.
    pub gear_door_commanded_open: [f64; 3],
    /// `true` when the gear lever is selected down.
    pub gear_lever_down: bool,
    pub parking_brake_on: bool,
    /// Nose-wheel tiller and body-gear steering command, degrees, `[nose,
    /// body left, body right]`. Positive right.
    pub steering_command_deg: [f64; 3],
    /// Fuel jettison armed, and the two jettison nozzle valves selected.
    pub jettison_armed: bool,
    pub jettison_valve_selected: [bool; 2],
    /// Engine feed cross-feed valves selected open, one per engine.
    pub crossfeed_valve_selected: [bool; 4],
    /// Cargo door commanded open fraction, `[fwd, aft, bulk]`.
    pub cargo_door_commanded_open: [f64; 3],
    /// Galley and lavatory draw, litres per second, as the cabin service
    /// actually consumes it: `[galley, lavatory]`.
    pub water_demand_l_s: [f64; 2],
    /// `[left, right]` brake pedal deflection, 0.0 released .. 1.0 full.
    pub brake_pedal_pos: [f64; 2],
    pub engine_master_on: [bool; 4],
    pub eng_gen_pb_on: [bool; 4],
    /// `[1, 2]`: the A380's two APU generator pushbuttons.
    pub apu_gen_pb_on: [bool; 2],
    /// `[1, 2]`: the two battery pushbuttons' AUTO/OFF position.
    pub bat_pb_auto: [bool; 2],
    /// `true` when the ground-spoiler/speedbrake lever is in the ARMED
    /// detent (X-Plane's own handle convention: pulled past the aft stop).
    /// No real manual galley-shed pushbutton exists in this port either;
    /// `deep::electrical`'s own load-management already computes an
    /// automatic `galley_shed_commanded` from the power budget, which is
    /// not this field's job to duplicate (see `docs/deep/
    /// truth-requests.md`).
    pub ground_spoiler_lever_armed: bool,
    pub apu_master_sw_on: bool,
    pub apu_start_pb_on: bool,
    /// Reverse thrust commanded, **engines 2 and 3 in that order** --
    /// `[bool; 2]`, not `[bool; 4]`, because engines 1 and 4 carry no
    /// reverser at all (`throttle::HAS_REVERSER`), and the same shape
    /// `deep::engine_accessories`' own `EngineAccessoryCommands::
    /// reverser_deploy_commanded` already takes, so wiring it there is one
    /// line with no index arithmetic to get wrong.
    ///
    /// The reverse lever itself: the thrust lever angle, below the
    /// A380's own opening-authorisation angle. This is the *selection*, not
    /// the deployment -- the reverser's locks, its hydraulic actuation and
    /// the N3 and weight-on-wheels interlocks are all in that area's own
    /// model and are not pre-empted here.
    pub reverser_deploy_commanded: [bool; 2],
}

impl Default for Controls {
    /// A cold aircraft, parked: masters and starters off, guards down,
    /// selections off, the gear down with its doors closed, the parking
    /// brake set -- the same resting state `Truth::default` documents for
    /// everything else. Generator, battery and pack pushbuttons default to
    /// their one normal *on/auto* position (matching `aspects.rs`'s own
    /// "the pushbuttons are built on... so the switches start on here too"
    /// for the generators), since that is a real switch position, not the
    /// absence of one.
    fn default() -> Self {
        Self {
            fire_pb_released: [false; 4],
            fire_pb_apu_released: false,
            fire_agent_pb_pressed: [[false; 2]; 4],
            fire_agent_pb_apu_pressed: false,
            wing_anti_ice_selected: false,
            nacelle_anti_ice_selected: [false; 4],
            engine_bleed_pb_auto: [true; 4],
            apu_bleed_pb_on: false,
            cross_bleed_selector: 1.0, // AUTO
            pack_pb_on: [true; 2],
            starter_engaged: [false; 4],
            rain_removal_selected: [false; 2],
            steering_command_deg: [0.0; 3],
            jettison_armed: false,
            jettison_valve_selected: [false; 2],
            crossfeed_valve_selected: [false; 4],
            cargo_door_commanded_open: [0.0; 3],
            water_demand_l_s: [0.0; 2],
            gear_door_commanded_open: [0.0; 3],
            gear_lever_down: true,
            parking_brake_on: true,
            brake_pedal_pos: [0.0; 2],
            engine_master_on: [false; 4],
            eng_gen_pb_on: [true; 4],
            apu_gen_pb_on: [true; 2],
            bat_pb_auto: [true; 2],
            ground_spoiler_lever_armed: false,
            apu_master_sw_on: false,
            apu_start_pb_on: false,
            reverser_deploy_commanded: [false; 2],
        }
    }
}

/// One flight-control surface set's commanded position, degrees, in the
/// same per-panel layout `flight_controls::Actuators`/`SurfaceOverrideWriter
/// ::PhysicalSurfaces` already use -- so `deep::flight_controls` can diff
/// this against its own physical output panel-for-panel, with no
/// re-blending. Sign and travel conventions match `flight_controls.rs`'s
/// own documented ones exactly (trailing edge up/down, rudder right,
/// spoiler up): see `plugin.rs` for the conversion.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct CommandedSurfaces {
    /// `[side][inward, middle, outward]`, degrees, positive trailing edge up.
    pub ailerons_deg: [[f64; 3]; 2],
    /// `[side][inward, outward]`, degrees, positive trailing edge up.
    pub elevators_deg: [[f64; 2]; 2],
    /// `[upper, lower]`, degrees, FlyByWire's own rudder body sign (not
    /// X-Plane's positive-right convention -- see `flight_controls.rs`).
    pub rudders_deg: [f64; 2],
    /// `[side][spoiler 1..=8]`, degrees up, 0..50.
    pub spoilers_deg: [[f64; 8]; 2],
    /// Degrees, positive nose up.
    pub ths_deg: f64,
}

impl Default for Truth {
    /// A cold aircraft on the ground at ISA sea level: what every area
    /// sees before the plugin has filled a single frame. Not all-zero --
    /// zero ambient pressure is a vacuum, and several areas divide by it.
    fn default() -> Self {
        Self {
            dt_s: 1.0 / 30.0,
            environment: EnvironmentTruth {
                sat_c: 15.0,
                leading_edge_c: 15.0,
                ambient_pressure_pa: 101_325.0,
                tas_ms: 0.0,
                precipitation_on_aircraft_ratio: 0.0,
                weather: None,
            },
            altitude_ft: 0.0,
            on_ground: true,
            engine_n1_frac: [0.0; 4],
            engine_running: [false; 4],
            engine_bleed_pressure_pa: [101_325.0; 4],
            engine_bleed_temp_k: [288.15; 4],
            // A cold aircraft's tyres sit at their service pressure; zero
            // would read as twenty-two flat tyres before the first frame.
            tyre_pressure_pa: [crate::physics::tyre::COLD_PRESSURE_PA; crate::physics::tyre::WHEELS],
            // A cold engine's oil sits at ambient with the pump stopped,
            // in a tank that was serviced full before the aircraft was
            // handed over -- an empty one would read as four engines that
            // have already lost their oil.
            engine_oil_pressure_pa: [0.0; 4],
            engine_oil_temp_c: [15.0; 4],
            engine_oil_quantity_fraction: [1.0; 4],
            // A cold engine's gas path is full of the air around it, so
            // both stations sit at the same 15 C ISA sea-level ambient the
            // rest of this state is quoted at.
            engine_tgt_c: [15.0; 4],
            engine_t25_c: [15.0; 4],
            // A parked aircraft is shut up: every door closed.
            door_open_fraction: [0.0; DOOR_NAMES.len()],
            apu_running: false,
            apu_bleed_pressure_pa: 101_325.0,
            ac_bus_volts: [0.0; 4],
            dc_bus_volts: [0.0; 2],
            hydraulic_pressure_pa: [0.0; 2],
            engine_n2_frac: [0.0; 4],
            engine_n3_frac: [0.0; 4],
            engine_hp_port_pressure_pa: [101_325.0; 4],
            engine_hp_port_temp_k: [288.15; 4],
            engine_fuel_flow_kg_s: [0.0; 4],
            gpu_plugged_in: false,
            controls: Controls::default(),
            commanded_surfaces: CommandedSurfaces::default(),
            // A380-800 operating empty weight, kg (Airbus's own published
            // Aircraft Characteristics figure is about 277 t for the -800:
            // the airframe with neither fuel nor payload). Mass is the one
            // field here that cannot honestly default to zero -- an
            // aircraft always weighs something -- and this is the same
            // resting state `deep::fuel::live`'s empty tanks and
            // `deep::gear_structure::live`'s own identically-cited constant
            // already describe.
            aircraft_mass_kg: 277_000.0,
            pitch_deg: 0.0,
            groundspeed_m_s: 0.0,
            angle_of_attack_deg: 0.0,
            radio_height_ft: 0.0,
            leg_on_ground: [true; 5],
            leg_touchdown_sink_speed_ms: [0.0; 5],
            cabin_pressure_pa: 101_325.0,
            cabin_temp_k: 288.15,
            sun_elevation_deg: 0.0,
            published: PublishedFrame::default(),
        }
    }
}

/// How badly each failure is armed, by `deep::api` failure id.
///
/// Absent means healthy. Every magnitude is clamped to 0..1 on the way in,
/// so an area can use it as a fraction without checking.
#[derive(Clone, Debug, Default)]
pub struct Faults(BTreeMap<u64, f64>);

impl Faults {
    pub fn from_pairs(pairs: impl IntoIterator<Item = (u64, f64)>) -> Self {
        Self(pairs.into_iter().map(|(id, m)| (id, m.clamp(0.0, 1.0))).collect())
    }

    /// This failure's magnitude, 0 if it is not armed at all.
    pub fn get(&self, id: u64) -> f64 {
        self.0.get(&id).copied().unwrap_or(0.0)
    }

    /// Whether anything at all is armed -- areas with an expensive
    /// healthy-case shortcut can check this first.
    pub fn any(&self) -> bool {
        self.0.values().any(|&m| m > 0.0)
    }
}

/// One of FlyByWire's own failures, as a deep area's verdict on a
/// component the two models share.
///
/// This is level 2 of `docs/deep/authority.md`: the deep area has
/// concluded that a physical component `a380_systems` also models is no
/// longer working, and says so in the only vocabulary `a380_systems`
/// accepts -- its own failure ids, the ones `crate::failures`'
/// `a380_failures()`/`extra::extra_failures()` catalogue. The plugin feeds
/// these into `crate::failures` beside the ones the crew armed, so a
/// derived failure and an armed one reach `a380_systems` by the same path.
///
/// Every field but `magnitude` is fixed at build time, because a derived
/// failure the crew cannot clear is only tolerable if the crew can see
/// *why*: [`deep_component`](Self::deep_component) and
/// [`reason`](Self::reason) are what the Study page shows next to it.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DerivedFailure {
    /// The id in `crate::failures`' catalogue -- FlyByWire's own
    /// (`a380_failures()`) or this port's extra one, both of which
    /// `failures::set_magnitude` accepts.
    pub fbw_id: u64,
    /// How far gone, 0 healthy .. 1 fully failed. An area emits every
    /// coupling it owns every frame, healthy ones at `0.0`; [`Deep::tick`]
    /// keeps only those above zero. Ids whose FlyByWire side is binary
    /// (most of them) get `1.0` or `0.0` and nothing in between -- the
    /// granularity limit `authority.md` names.
    pub magnitude: f64,
    /// The deep component that concluded it, by its registry id, e.g.
    /// `"24_elec.vfg-1"`.
    pub deep_component: &'static str,
    /// Why, in one phrase a pilot can read: "own overload element tripped
    /// it off line", "feeder breaker open".
    pub reason: &'static str,
}

/// One area's live system.
pub trait Area {
    /// A short name for diagnostics and the frame-time breakdown.
    fn name(&self) -> &'static str;

    /// Advance this area by `truth.dt_s`, with the failures armed as
    /// given.
    fn tick(&mut self, truth: &Truth, faults: &Faults);

    /// Publish what the rest of the aircraft can see: the variables this
    /// area's `registry.rs` names in its ECAM triggers, plus anything the
    /// EFB's Study pages read. Called after every area has ticked.
    fn publish(&self, out: &mut dyn FnMut(&str, f64));

    /// Every FlyByWire failure this area is authoritative over, with this
    /// frame's verdict on it (see [`DerivedFailure`] and
    /// `docs/deep/authority.md`).
    ///
    /// An area emits its whole coupling table every frame, healthy
    /// couplings at magnitude `0.0`, so that the set is a *level* and not
    /// an event: nothing has to remember to clear a derived failure when
    /// the component recovers.
    ///
    /// The default is empty, which is the right answer for every area that
    /// models something FlyByWire does not model at all -- level 1 of
    /// `authority.md`, where publishing is the whole of the job.
    fn derived_failures(&self, _out: &mut dyn FnMut(DerivedFailure)) {}
}

/// Every area's live system, owned in one place.
///
/// Areas are added here as each grows a live system; an area with no
/// entry yet is simply not stepped, which is why this is a list rather
/// than eighteen named fields.
#[derive(Default)]
pub struct Deep {
    areas: Vec<Box<dyn Area>>,
    truth: Truth,
    /// What the areas published last frame, handed back to them as
    /// `Truth::published` on the next one.
    last_published: PublishedFrame,
    /// Every FlyByWire failure the areas concluded was real on the last
    /// tick, in area order. Only magnitudes above zero are kept, so this is
    /// empty on a healthy aircraft and the plugin pays nothing for it.
    derived: Vec<DerivedFailure>,
}

impl Deep {
    /// Every area that has a live system, constructed cold.
    pub fn new() -> Self {
        Self { areas: Vec::new(), truth: Truth::default(), last_published: PublishedFrame::default(), derived: Vec::new() }
    }

    pub fn with_area(mut self, area: Box<dyn Area>) -> Self {
        self.areas.push(area);
        self
    }

    /// The truth as of the last tick, for anything that needs to read back
    /// what the areas were given.
    pub fn truth(&self) -> &Truth {
        &self.truth
    }

    /// Step every area, then publish. `publish` runs after every area has
    /// ticked so that no area can see half a frame.
    ///
    /// Everything published is also kept, and handed back to the areas on
    /// the next tick as `Truth::published`, which is how one area reads
    /// another's output -- a duct's overheat loop watching the bay
    /// temperature the thermal area computes, say. The one-frame lag is the
    /// deliberate one this module's header describes.
    pub fn tick(&mut self, truth: Truth, faults: &Faults, out: &mut dyn FnMut(&str, f64)) {
        self.truth = truth;
        self.truth.published = std::mem::take(&mut self.last_published);
        for area in &mut self.areas {
            area.tick(&self.truth, faults);
        }
        // The areas' verdicts on FlyByWire's own components, collected
        // after every area has ticked and before any has published, so a
        // verdict is this frame's and not half of one. See
        // `docs/deep/authority.md`.
        let mut derived = std::mem::take(&mut self.derived);
        derived.clear();
        for area in &self.areas {
            area.derived_failures(&mut |d| {
                if d.magnitude > 0.0 {
                    derived.push(d);
                }
            });
        }
        self.derived = derived;
        let mut published = std::mem::take(&mut self.truth.published);
        published.0.clear();
        for area in &self.areas {
            area.publish(&mut |name, value| {
                published.0.insert(name.to_string(), value);
                out(name, value);
            });
        }
        self.last_published = published;
    }

    pub fn area_names(&self) -> Vec<&'static str> {
        self.areas.iter().map(|a| a.name()).collect()
    }

    /// Every FlyByWire failure the areas concluded was real on the last
    /// [`tick`](Self::tick), with the deep component and reason behind each
    /// one. The plugin hands these to `crate::failures` beside the crew's
    /// own armed failures; see `docs/deep/authority.md` for the exact
    /// plugin-side patch.
    pub fn derived_failures(&self) -> &[DerivedFailure] {
        &self.derived
    }

    /// The same set as a magnitude per FlyByWire failure id, worst verdict
    /// winning where two areas name the same id (none do today; taking the
    /// max rather than the last means the order areas were added in can
    /// never change the answer).
    pub fn derived_magnitudes(&self) -> BTreeMap<u64, f64> {
        let mut out: BTreeMap<u64, f64> = BTreeMap::new();
        for d in &self.derived {
            let slot = out.entry(d.fbw_id).or_insert(0.0);
            *slot = slot.max(d.magnitude.clamp(0.0, 1.0));
        }
        out
    }

    /// Every variable name the areas publish, without stepping anything.
    ///
    /// `Plugin` calls this once at startup so it can resolve each name to a
    /// `VariableIdentifier` there rather than in the frame loop: at 30-60 Hz
    /// a per-name `Vars::get` would allocate the name and its `A32NX_`
    /// prefix every frame, for every published value, forever.
    ///
    /// Publishing is a pure read of an area's own state (the trait takes
    /// `&self`), so calling it on cold areas has no effect on them beyond
    /// the values it reports, which are discarded here.
    pub fn published_names(&self) -> Vec<String> {
        let mut names = Vec::new();
        for area in &self.areas {
            area.publish(&mut |name, _| names.push(name.to_owned()));
        }
        names
    }
}

/// Every area that has grown a live system, in the order they are stepped.
///
/// Each area publishes `pub fn live_system() -> Box<dyn Area>` from its own
/// `src/deep/<area>/live.rs`; an area that has not grown one yet is simply
/// absent from this list and is not stepped (see [`Deep`]'s own doc). This
/// is the one place the list lives, so adding an area is one line here and
/// nothing in `lib.rs` changes.
///
/// Ordering is deliberate only in that it is fixed: an area reads the
/// previous frame's value of anything another area publishes, so no order
/// here can be wrong (see this module's "Ordering" note). The list is
/// alphabetical so that a new area has an obvious place to go.
pub fn all_areas() -> Deep {
    Deep::new()
        .with_area(crate::deep::apu::live::live_system())
        .with_area(crate::deep::avionics_network::live::live_system())
        .with_area(crate::deep::breakers::live::live_system())
        .with_area(crate::deep::cabin::live::live_system())
        .with_area(crate::deep::electrical::live::live_system())
        .with_area(crate::deep::engine_accessories::live::live_system())
        .with_area(crate::deep::environment::live::live_system())
        .with_area(crate::deep::fire_ice::live::live_system())
        .with_area(crate::deep::flight_controls::live::live_system())
        .with_area(crate::deep::fuel::live::live_system())
        .with_area(crate::deep::gear_structure::live::live_system())
        .with_area(crate::deep::hydraulics::live::live_system())
        .with_area(crate::deep::oxygen::live::live_system())
        .with_area(crate::deep::pneumatic_ducts::live::live_system())
        .with_area(crate::deep::sensors::live::live_system())
        .with_area(crate::deep::thermal_zones::live::live_system())
        .with_area(crate::deep::wiring::live::live_system())
    // Every area under `deep/` has one now. A new area adds its own line
    // here, alphabetically, and nothing else in the plugin changes.
}

#[cfg(test)]
mod tests {
    use super::*;

    #[derive(Default)]
    struct Counter {
        ticks: usize,
        last_dt: f64,
        leak: f64,
    }

    impl Area for Counter {
        fn name(&self) -> &'static str {
            "counter"
        }
        fn tick(&mut self, truth: &Truth, faults: &Faults) {
            self.ticks += 1;
            self.last_dt = truth.dt_s;
            self.leak = faults.get(11_021_001);
        }
        fn publish(&self, out: &mut dyn FnMut(&str, f64)) {
            out("TEST_COUNTER_TICKS", self.ticks as f64);
            out("TEST_COUNTER_LEAK", self.leak);
        }
    }

    #[test]
    fn a_default_truth_is_a_cold_aircraft_in_real_air_not_a_vacuum() {
        // Several areas divide by ambient pressure or density; the
        // before-the-first-frame state has to be somewhere an aircraft
        // could actually be.
        let t = Truth::default();
        assert!(t.environment.ambient_pressure_pa > 90_000.0);
        assert!(t.dt_s > 0.0);
        assert!(t.on_ground && !t.apu_running);
        assert_eq!(t.engine_running, [false; 4]);
    }

    #[test]
    fn an_area_is_ticked_with_the_truth_and_faults_and_then_publishes() {
        let mut deep = Deep::new().with_area(Box::new(Counter::default()));
        let faults = Faults::from_pairs([(11_021_001, 0.4)]);
        let mut published = BTreeMap::new();
        let truth = Truth { dt_s: 0.05, ..Truth::default() };
        deep.tick(truth, &faults, &mut |name, value| {
            published.insert(name.to_string(), value);
        });
        assert_eq!(published.get("TEST_COUNTER_TICKS"), Some(&1.0));
        assert_eq!(published.get("TEST_COUNTER_LEAK"), Some(&0.4));
        assert_eq!(deep.truth().dt_s, 0.05);
        assert_eq!(deep.area_names(), vec!["counter"]);
    }

    #[test]
    fn every_assembled_area_has_its_own_name_and_can_be_stepped_cold() {
        // The assembly is a hand-kept list, so the two things that can go
        // wrong with it are an area added twice and an area that cannot
        // survive its first frame (dt clamped to a real value, a cold
        // aircraft in real air, nothing armed).
        let mut deep = all_areas();
        let names = deep.area_names();
        let unique: std::collections::BTreeSet<_> = names.iter().collect();
        assert_eq!(unique.len(), names.len(), "an area is listed twice in all_areas(): {names:?}");
        let mut published = BTreeMap::new();
        deep.tick(Truth::default(), &Faults::default(), &mut |name, value| {
            assert!(value.is_finite(), "{name} published {value} on the first frame");
            published.insert(name.to_owned(), value);
        });
    }

    #[test]
    fn published_names_are_what_the_areas_actually_publish() {
        // The plugin resolves these to variable ids once at startup, so a
        // name reported here that the areas never publish would be a dead
        // variable, and one they publish but do not report would be
        // resolved in the frame loop instead.
        let deep = all_areas();
        let reported: std::collections::BTreeSet<String> = deep.published_names().into_iter().collect();
        let mut actual = std::collections::BTreeSet::new();
        for area in &deep.areas {
            area.publish(&mut |name, _| {
                actual.insert(name.to_owned());
            });
        }
        assert_eq!(reported, actual);
    }

    /// One area reading another's output, which is what the pneumatic
    /// overheat loops need from the thermal zones.
    #[derive(Default)]
    struct Downstream {
        saw: Option<f64>,
    }

    impl Area for Downstream {
        fn name(&self) -> &'static str {
            "downstream"
        }
        fn tick(&mut self, truth: &Truth, _faults: &Faults) {
            self.saw = truth.published.get("TEST_COUNTER_TICKS");
        }
        fn publish(&self, out: &mut dyn FnMut(&str, f64)) {
            out("TEST_DOWNSTREAM_SAW", self.saw.unwrap_or(-1.0));
        }
    }

    #[test]
    fn an_area_reads_what_another_published_on_the_previous_frame() {
        // Registration order must not matter: `Downstream` is stepped
        // *before* `Counter` here, and still sees its value, because every
        // area ticks before any area publishes.
        let mut deep = Deep::new().with_area(Box::new(Downstream::default())).with_area(Box::new(Counter::default()));
        let faults = Faults::default();
        let mut published = BTreeMap::new();
        let mut run = |deep: &mut Deep, published: &mut BTreeMap<String, f64>| {
            deep.tick(Truth::default(), &faults, &mut |name, value| {
                published.insert(name.to_string(), value);
            });
        };

        run(&mut deep, &mut published);
        assert_eq!(published.get("TEST_DOWNSTREAM_SAW"), Some(&-1.0), "nothing has been published yet, so the read must come back absent rather than zero");

        run(&mut deep, &mut published);
        assert_eq!(published.get("TEST_DOWNSTREAM_SAW"), Some(&1.0), "the second frame sees the first frame's value");

        run(&mut deep, &mut published);
        assert_eq!(published.get("TEST_DOWNSTREAM_SAW"), Some(&2.0), "and it keeps up, exactly one frame behind");
    }

    #[test]
    fn a_name_nobody_publishes_is_absent_not_zero() {
        // An area has to be able to tell "the bay is at 0 C" from "nothing
        // models that bay", which is why this is an Option.
        let frame = PublishedFrame::default();
        assert_eq!(frame.get("NOBODY_PUBLISHES_THIS"), None);
        assert_eq!(frame.get_or("NOBODY_PUBLISHES_THIS", 288.15), 288.15);
        assert!(frame.is_empty());
    }

    /// An area with a level-2 coupling: it emits its whole table every
    /// frame, healthy entries at zero, and the verdict follows a fault the
    /// crew armed against its own deep id.
    #[derive(Default)]
    struct Coupled {
        verdict: f64,
    }

    impl Area for Coupled {
        fn name(&self) -> &'static str {
            "coupled"
        }
        fn tick(&mut self, _truth: &Truth, faults: &Faults) {
            self.verdict = faults.get(11_021_001);
        }
        fn publish(&self, _out: &mut dyn FnMut(&str, f64)) {}
        fn derived_failures(&self, out: &mut dyn FnMut(DerivedFailure)) {
            out(DerivedFailure { fbw_id: 24_020, magnitude: if self.verdict > 0.0 { 1.0 } else { 0.0 }, deep_component: "test.gen-1", reason: "test" });
            // A healthy coupling, emitted every frame at zero so that the
            // set is a level and not an event.
            out(DerivedFailure { fbw_id: 24_021, magnitude: 0.0, deep_component: "test.gen-2", reason: "test" });
        }
    }

    #[test]
    fn a_healthy_aircraft_derives_no_flybywire_failures_at_all() {
        let mut deep = Deep::new().with_area(Box::new(Coupled::default())).with_area(Box::new(Counter::default()));
        deep.tick(Truth::default(), &Faults::default(), &mut |_, _| {});
        assert!(deep.derived_failures().is_empty(), "a zero-magnitude coupling must not reach FlyByWire: {:?}", deep.derived_failures());
        assert!(deep.derived_magnitudes().is_empty());
    }

    #[test]
    fn an_areas_verdict_on_a_flybywire_component_reaches_the_plugin_and_clears_again() {
        let mut deep = Deep::new().with_area(Box::new(Coupled::default()));
        deep.tick(Truth::default(), &Faults::from_pairs([(11_021_001, 1.0)]), &mut |_, _| {});
        assert_eq!(deep.derived_failures().len(), 1);
        let d = deep.derived_failures()[0];
        assert_eq!(d.fbw_id, 24_020);
        assert_eq!(d.magnitude, 1.0);
        assert_eq!(d.deep_component, "test.gen-1");
        assert_eq!(deep.derived_magnitudes().get(&24_020), Some(&1.0));

        // The set is a level: the verdict going away takes the derived
        // failure with it, with nothing to remember to clear.
        deep.tick(Truth::default(), &Faults::default(), &mut |_, _| {});
        assert!(deep.derived_failures().is_empty());
    }

    #[test]
    fn an_area_with_nothing_flybywire_models_derives_nothing() {
        // The trait's default: level 1 of `authority.md`, where an area
        // publishes and nothing competes with it.
        let mut deep = Deep::new().with_area(Box::new(Counter::default()));
        deep.tick(Truth::default(), &Faults::from_pairs([(11_021_001, 1.0)]), &mut |_, _| {});
        assert!(deep.derived_failures().is_empty());
    }

    #[test]
    fn every_derived_failure_names_a_real_flybywire_failure_id_and_says_why() {
        // The whole point of level 2 is that the id means something to
        // `a380_systems`: an id outside `crate::failures`' catalogue would
        // be silently ignored by `Failures::apply`, and a coupling with no
        // reason would be a derived failure the crew cannot explain.
        let known: std::collections::BTreeSet<u64> = crate::failures::all_ids().into_iter().collect();
        let mut deep = all_areas();
        deep.tick(Truth::default(), &Faults::default(), &mut |_, _| {});
        let mut seen = 0usize;
        for area in &deep.areas {
            area.derived_failures(&mut |d| {
                seen += 1;
                assert!(known.contains(&d.fbw_id), "{} derives failure {}, which is in no catalogue", area.name(), d.fbw_id);
                assert!(!d.deep_component.is_empty(), "{} derives {} with no component", area.name(), d.fbw_id);
                assert!(!d.reason.is_empty(), "{} derives {} with no reason", area.name(), d.fbw_id);
                assert!((0.0..=1.0).contains(&d.magnitude), "{} derives {} at magnitude {}", area.name(), d.fbw_id, d.magnitude);
            });
        }
        assert!(seen >= 40, "the three coupled areas should carry their whole coupling table, got {seen}");
    }

    #[test]
    fn an_unarmed_failure_reads_healthy_and_magnitudes_are_bounded() {
        let f = Faults::from_pairs([(1, 2.5), (2, -1.0)]);
        assert_eq!(f.get(999), 0.0, "a failure nobody armed must read healthy, not absent");
        assert_eq!(f.get(1), 1.0, "magnitudes are fractions and cannot exceed fully failed");
        assert_eq!(f.get(2), 0.0);
        assert!(f.any());
        assert!(!Faults::default().any());
    }
}


