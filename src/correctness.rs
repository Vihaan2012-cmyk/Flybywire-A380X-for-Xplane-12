//! The MSFS glue the other modules don't cover: FlyByWire's A380 aspects
//! (aspects.rs), failures (failures.rs), and the start state (start_state.rs,
//! decided in `Plugin::new`).
//!
//! Per tick, in FlyByWire's order (systems_wasm lib.rs:250-262):
//! - before the systems: the aspects' pre_tick, then any change to the active
//!   failures handed to the simulation;
//! - after the systems: the aspects' post_tick. lib.rs has no after-systems
//!   slot for this module, so it runs at the start of the next tick, before
//!   anything else reads or writes the variables involved (the next pre_tick
//!   and systems tick see exactly what they would in MSFS).

#[cfg(test)]
use a380_systems::A380;
#[cfg(test)]
use systems::simulation::Simulation;
use systems::simulation::StartState;

use crate::aspects::{self, Aspects};
use crate::failures::{Failures, XplaneFailures};
use crate::xp::{DataRef, Xplm};
use crate::Vars;

pub struct Correctness {
    xplm: &'static Xplm,
    #[allow(dead_code)]
    pub start_state: StartState,
    aspects: Aspects,
    failures: Failures,
    _xplane_failures: XplaneFailures,
    ticked: bool,
    inlet_heat: Option<DataRef>,
    wing_heat: Vec<DataRef>,
}

impl Correctness {
    pub fn new(vars: &mut Vars, xplm: &'static Xplm, start_state: StartState) -> Self {
        let aspects = aspects::a380(vars);
        let failures = Failures::new();
        let ids: Vec<u64> = failures.ids().collect();
        let xplane_failures = XplaneFailures::register(xplm, &ids);
        crate::log(&format!(
            "{} FlyByWire failures as fbw/failure/<id> and fbw/failure/<id>/toggle",
            ids.len()
        ));
        Self {
            xplm,
            start_state,
            aspects,
            failures,
            _xplane_failures: xplane_failures,
            ticked: false,
            inlet_heat: xplm.find("sim/cockpit2/ice/ice_inlet_heat_on_per_engine"),
            wing_heat: ["sim/cockpit2/ice/ice_surface_hot_bleed_air_on", "sim/cockpit2/ice/ice_surfce_heat_on"]
                .iter()
                .filter_map(|n| xplm.find(n))
                .collect(),
        }
    }

    /// The previous tick's post_tick, and what its key events do.
    pub fn after_previous_tick(&mut self, vars: &mut Vars) {
        if !self.ticked {
            return;
        }
        self.aspects.post_tick(vars);
        for (event, data) in self.aspects.take_events() {
            aspects::apply_msfs_key_event(vars, &event, data);
            self.mirror_to_xplane(vars, &event, data);
        }
    }

    /// The aspects' pre_tick and the failures, just before the systems tick.
    pub fn before_systems(&mut self, vars: &mut Vars, simulation: &mut impl crate::remote::FailureSink, delta: f64) {
        self.aspects.pre_tick(vars, delta);
        for line in self.failures.apply(simulation, vars, Some(self.xplm)) {
            crate::log(&line);
        }
        self.ticked = true;
    }

    /// X-Plane's counterpart of the MSFS effect of a key event.
    fn mirror_to_xplane(&self, vars: &mut Vars, event: &str, data: u32) {
        use systems::simulation::{SimulatorReaderWriter, VariableRegistry};
        if let Some(n) = event.strip_prefix("ANTI_ICE_SET_ENG").and_then(|n| n.parse::<usize>().ok()) {
            if let (Some(d), true) = (self.inlet_heat, n >= 1) {
                self.xplm.set_vi_at(d, n - 1, (data != 0) as i32);
            }
        } else if event == "TOGGLE_STRUCTURAL_DEICE" {
            let id = vars.get("STRUCTURAL DEICE SWITCH".to_owned());
            let on = vars.read(&id) != 0.;
            for d in &self.wing_heat {
                self.xplm.set_i(*d, on as i32);
            }
        }
    }
}
