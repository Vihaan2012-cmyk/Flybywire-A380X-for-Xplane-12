//! FlyByWire's A380X extra backend (`extra-backend-a380x.wasm`, panel.cfg
//! VCockpit21 htmlgauge03), ported from its C++:
//!
//! - fbw-a380x/src/wasm/extra-backend-a380x/src/Gauge_Extra_Backend.cpp: the
//!   modules and their order (LightingPresets_A380X, Pushback_A380X,
//!   AircraftPresets);
//! - fbw-common/src/wasm/extra-backend/: LightingPresets, Pushback,
//!   AircraftPresets with PresetProcedures and ProcedureStep.
//!
//! The C++ talks to MSFS through FlyByWire's cpp-msfs-framework (DataManager
//! variables, SimConnect data definitions, `execute_calculator_code`); here
//! the same variables are the plugin's [`Vars`](crate::Vars), and what MSFS
//! itself owns (lights, trims, the aircraft's velocities) is X-Plane's, through
//! the [`XplaneIo`] the plugin hands in. Each module says where it differs.
//!
//! MsfsHandler (cpp-msfs-framework/MsfsHandler/MsfsHandler.cpp:102-170) runs
//! the modules every visual frame unless the simulator is paused, with the
//! simulation time as its time stamp; the plugin's flight loop is the same.

pub mod aircraft_presets;
pub mod lighting_presets;
pub mod procedures;
pub mod pushback;
pub mod rpn;
pub mod sim;

use std::collections::HashMap;

use systems::simulation::{SimulatorReaderWriter, VariableIdentifier, VariableRegistry};

use crate::xp::{DataRef, Xplm};

/// X-Plane's datarefs and commands, by name, as the extra backend uses them.
pub trait XplaneIo {
    /// A float, int or float-array element dataref's value, if X-Plane has it.
    fn get(&mut self, dataref: &str, index: Option<usize>) -> Option<f64>;
    fn set(&mut self, dataref: &str, index: Option<usize>, value: f64);
    fn command_once(&mut self, command: &str);
}

/// [`XplaneIo`] on X-Plane itself, with each dataref looked up once.
pub struct Xplane {
    xplm: &'static Xplm,
    found: HashMap<String, Option<(DataRef, i32)>>,
}

/// XPLMDataTypeID bits (XPLMDataAccess.h).
const TYPE_INT: i32 = 1;
const TYPE_FLOAT: i32 = 2;
const TYPE_DOUBLE: i32 = 4;

impl Xplane {
    pub fn new(xplm: &'static Xplm) -> Self {
        Self { xplm, found: HashMap::new() }
    }

    fn find(&mut self, name: &str) -> Option<(DataRef, i32)> {
        let xplm = self.xplm;
        *self.found.entry(name.to_string()).or_insert_with(|| {
            let d = xplm.find(name)?;
            Some((d, crate::xp::data_ref_types(d)))
        })
    }
}

impl XplaneIo for Xplane {
    fn get(&mut self, dataref: &str, index: Option<usize>) -> Option<f64> {
        let (d, types) = self.find(dataref)?;
        Some(match index {
            Some(i) => {
                let mut v = vec![0f32; i + 1];
                if self.xplm.get_vf(d, &mut v) <= i {
                    return None;
                }
                v[i] as f64
            }
            None if types & TYPE_DOUBLE != 0 => self.xplm.get_d(d),
            None if types & TYPE_FLOAT != 0 => self.xplm.get_f(d) as f64,
            None => self.xplm.get_i(d) as f64,
        })
    }

    fn set(&mut self, dataref: &str, index: Option<usize>, value: f64) {
        let Some((d, types)) = self.find(dataref) else { return };
        match index {
            Some(i) => self.xplm.set_vf_at(d, i, value as f32),
            None if types & TYPE_FLOAT != 0 => self.xplm.set_f(d, value as f32),
            None if types & TYPE_INT != 0 => self.xplm.set_i(d, value.round() as i32),
            None => self.xplm.set_f(d, value as f32),
        }
    }

    fn command_once(&mut self, command: &str) {
        crate::xp::command_once(command);
    }
}

/// One of the aircraft's named variables (FlyByWire's `make_named_var`,
/// which adds the `A32NX_` prefix; Gauge_Extra_Backend.cpp:28).
pub fn named<V: VariableRegistry>(vars: &mut V, name: &str) -> VariableIdentifier {
    vars.get(name.to_string())
}

/// A variable as MSFS calculator code names it: `L:NAME` exactly as written,
/// `A:NAME:index` a simulator variable.
pub fn calculator_variable<V: VariableRegistry>(vars: &mut V, kind: &str, name: &str) -> VariableIdentifier {
    match (kind, name.strip_prefix(crate::NAME_PREFIX)) {
        ("L", Some(bare)) => vars.get(bare.to_string()),
        _ => vars.get_unprefixed(name.to_string()),
    }
}

/// The extra backend's three modules, in Gauge_Extra_Backend.cpp's order.
pub struct ExtraBackend {
    xplane: Xplane,
    lighting: lighting_presets::LightingPresets,
    pushback: pushback::Pushback,
    presets: aircraft_presets::AircraftPresets,
}

impl ExtraBackend {
    pub fn new(vars: &mut crate::Vars, xplm: &'static Xplm) -> Self {
        let mut xplane = Xplane::new(xplm);
        let lighting = lighting_presets::LightingPresets::new(vars, lighting_presets::ini_path());
        let pushback = pushback::Pushback::new(vars, &mut xplane);
        let presets = aircraft_presets::AircraftPresets::new(vars);
        crate::log("extra backend: lighting presets, pushback and aircraft presets running");
        Self { xplane, lighting, pushback, presets }
    }

    /// Before the systems: the tug's state as MSFS's PUSHBACK STATE, which
    /// FlyByWire's nose wheel steering reads.
    pub fn inputs<V: VariableRegistry + SimulatorReaderWriter>(&mut self, vars: &mut V) {
        self.pushback.write_pushback_state(vars);
    }

    /// One MsfsHandler update (`PANEL_SERVICE_PRE_DRAW`).
    pub fn update<V: VariableRegistry + SimulatorReaderWriter>(&mut self, vars: &mut V, delta: f64, time: f64) {
        self.lighting.update(vars, time);
        self.pushback.update(vars, &mut self.xplane, delta);
        self.presets.update(vars, &mut self.xplane, delta);
    }

    /// Key events from the scripts (the EFB): true when one was the tug's.
    pub fn handle_event(&mut self, name: &str, _value: f64) -> bool {
        self.pushback.handle_event(name)
    }

    /// X-Plane's aircraft back to X-Plane.
    pub fn release(&mut self) {
        self.pushback.release(&mut self.xplane);
    }
}
