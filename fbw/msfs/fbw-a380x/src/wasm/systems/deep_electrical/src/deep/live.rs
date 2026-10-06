pub use super::frame::{DerivedFailure, Faults, PublishedFrame};

pub type Truth = ElectricalInputs;
pub type Controls = ElectricalControls;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Environment {
    pub sat_c: f64,
    pub tas_ms: f64,
}

#[derive(Clone, Debug)]
pub struct ElectricalInputs {
    pub dt_s: f64,
    pub environment: Environment,
    pub on_ground: bool,
    pub engine_running: [bool; 4],
    pub engine_n2_frac: [f64; 4],
    pub engine_oil_temp_c: [f64; 4],
    pub apu_running: bool,
    pub ac_bus_volts: [f64; 4],
    pub dc_bus_volts: [f64; 2],
    pub ac_bus_powered: [bool; 4],
    pub dc_bus_powered: [bool; 2],
    pub gpu_plugged_in: bool,
    pub controls: ElectricalControls,
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

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ElectricalControls {
    pub fire_pb_released: [bool; 4],
    pub fire_pb_apu_released: bool,
    pub fire_agent_pb_pressed: [[bool; 2]; 4],
    pub fire_agent_pb_apu_pressed: bool,
    pub starter_engaged: [bool; 4],
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

pub trait Area {
    fn name(&self) -> &'static str;
    fn tick(&mut self, truth: &Truth, faults: &Faults);
    fn publish(&self, out: &mut dyn FnMut(&str, f64));
    fn derived_failures(&self, _out: &mut dyn FnMut(DerivedFailure)) {}
}
