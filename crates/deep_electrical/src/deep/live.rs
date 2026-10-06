//! What the three electrical areas read from the rest of the aircraft each
//! frame, and the trait they implement.
//!
//! The area files are the X-Plane plugin's own and are compiled here
//! unchanged, so they still say `crate::deep::live::{Area, Truth, ...}`.
//! In the plugin, `Truth` is everything every deep area reads. Here it is
//! [`ElectricalInputs`]: exactly the fields these three read, with the same
//! names, types and defaults, so the same source compiles against both.

pub use super::frame::{DerivedFailure, Faults, PublishedFrame};

/// The name the area files use for their per-frame input.
pub type Truth = ElectricalInputs;
/// The name the area files use for the crew's switches.
pub type Controls = ElectricalControls;

/// The outside air, as far as these areas read it.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Environment {
    /// Static air temperature, deg C.
    pub sat_c: f64,
    /// True airspeed, m/s.
    pub tas_ms: f64,
}

/// Everything the electrical network, its 417 protection units and the
/// wiring read each frame. Field for field the plugin's `deep::live::Truth`
/// fields of the same names; the defaults are that struct's too.
#[derive(Clone, Debug)]
pub struct ElectricalInputs {
    /// Frame time, s.
    pub dt_s: f64,
    pub environment: Environment,
    pub on_ground: bool,
    pub engine_running: [bool; 4],
    /// N2 as a fraction of rated, 0..1+.
    pub engine_n2_frac: [f64; 4],
    /// Each engine's oil temperature, deg C: the ambient its generator
    /// drive's own oil system sits in.
    pub engine_oil_temp_c: [f64; 4],
    pub apu_running: bool,
    /// FlyByWire's own AC 1-4 and DC 1-2 bus potentials, V.
    pub ac_bus_volts: [f64; 4],
    pub dc_bus_volts: [f64; 2],
    /// FlyByWire's own AC 1-4 and DC 1-2 bus "is powered" flags.
    pub ac_bus_powered: [bool; 4],
    pub dc_bus_powered: [bool; 2],
    pub gpu_plugged_in: bool,
    pub controls: ElectricalControls,
    /// Values other deep areas published last frame. The electrical area
    /// reads three: `CABIN_CARGO_DOOR_CMD:1`, `CABIN_CARGO_DOOR_PERCENT:1`
    /// and `THERMAL_ZONE_MAINAVIONICS_TEMPERATURE_C`. A host that models
    /// none of them leaves this empty, and each falls back to the default
    /// the area already uses for "nobody models this".
    pub published: PublishedFrame,
}

impl Default for ElectricalInputs {
    fn default() -> Self {
        Self {
            dt_s: 1.0 / 30.0,
            environment: Environment { sat_c: 15.0, tas_ms: 0.0 },
            on_ground: true,
            engine_running: [false; 4],
            engine_n2_frac: [0.0; 4],
            engine_oil_temp_c: [15.0; 4],
            apu_running: false,
            ac_bus_volts: [0.0; 4],
            dc_bus_volts: [0.0; 2],
            ac_bus_powered: [false; 4],
            dc_bus_powered: [false; 2],
            gpu_plugged_in: false,
            controls: ElectricalControls::default(),
            published: PublishedFrame::default(),
        }
    }
}

/// The crew's switches these areas read: the plugin's `deep::live::Controls`
/// fields of the same names.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ElectricalControls {
    /// ENG FIRE pushbutton released (handle pulled), per engine.
    pub fire_pb_released: [bool; 4],
    pub fire_pb_apu_released: bool,
    /// AGENT 1/2 pushbutton pressed, per engine.
    pub fire_agent_pb_pressed: [[bool; 2]; 4],
    pub fire_agent_pb_apu_pressed: bool,
    /// The engine is being cranked (the starter's own engage condition).
    pub starter_engaged: [bool; 4],
    /// FlyByWire's commanded gear door position, 0 closed .. 1 open:
    /// centre, left, right.
    pub gear_door_commanded_open: [f64; 3],
    pub eng_gen_pb_on: [bool; 4],
    pub apu_gen_pb_on: [bool; 2],
    pub bat_pb_auto: [bool; 2],
    pub apu_start_pb_on: bool,
}

impl Default for ElectricalControls {
    fn default() -> Self {
        Self {
            fire_pb_released: [false; 4],
            fire_pb_apu_released: false,
            fire_agent_pb_pressed: [[false; 2]; 4],
            fire_agent_pb_apu_pressed: false,
            starter_engaged: [false; 4],
            gear_door_commanded_open: [0.0; 3],
            eng_gen_pb_on: [true; 4],
            apu_gen_pb_on: [true; 2],
            bat_pb_auto: [true; 2],
            apu_start_pb_on: false,
        }
    }
}

/// One area's live system: the plugin's `deep::live::Area` without the
/// flight-control hook no electrical area implements.
pub trait Area {
    fn name(&self) -> &'static str;
    fn tick(&mut self, truth: &Truth, faults: &Faults);
    fn publish(&self, out: &mut dyn FnMut(&str, f64));
    fn derived_failures(&self, _out: &mut dyn FnMut(DerivedFailure)) {}
    fn as_breakers(&self) -> Option<&crate::deep::breakers::live::BreakersLive> {
        None
    }
    fn as_breakers_mut(&mut self) -> Option<&mut crate::deep::breakers::live::BreakersLive> {
        None
    }
}
