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
//!   forward, then publish what the rest of the aircraft can see.
//!
//! [`Deep`] owns one live system per area and is what `Plugin::tick`
//! actually calls.
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
    /// APU: running, and its bleed available at the valve.
    pub apu_running: bool,
    pub apu_bleed_pressure_pa: f64,
    /// Bus voltages FlyByWire's own electrical system publishes, so the
    /// areas that consume power agree with what the crew sees on the ELEC
    /// page rather than running a second, disagreeing electrical model.
    pub ac_bus_volts: [f64; 4],
    pub dc_bus_volts: [f64; 2],
    /// Hydraulic system pressures, green and yellow, Pa.
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
}

impl Deep {
    /// Every area that has a live system, constructed cold.
    pub fn new() -> Self {
        Self { areas: Vec::new(), truth: Truth::default(), last_published: PublishedFrame::default() }
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


