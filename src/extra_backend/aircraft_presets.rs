//! FlyByWire's AircraftPresets (fbw-common/src/wasm/extra-backend/
//! AircraftPresets/AircraftPresets.cpp), line for line: the flyPad's
//! "Aircraft Presets" (cold and dark, powered, ready for pushback, taxi,
//! takeoff) run as FlyByWire's own procedure steps.
//!
//! Control variables (AircraftPresets.h:27-35), all `A32NX_` prefixed:
//! `AIRCRAFT_PRESET_LOAD` (1-5 to load, 0 to cancel), `_LOAD_PROGRESS`,
//! `_VERBOSE`, `_LOAD_EXPEDITE`, `_LOAD_EXPEDITE_DELAY` (ms) and
//! `_QUICK_MODE`, which the FADEC reads.
//!
//! The one MSFS-only call: updateProgress also sends the progress and step
//! name to the flyPad over the CommBus (`AIRCRAFT_PRESET_WASM_CALLBACK`,
//! AircraftPresets.cpp:179-186). There is no CommBus here; the same
//! `"<progress>;<step name>"` text is kept in [`AircraftPresets::progress_text`]
//! and logged.

use systems::simulation::{SimulatorReaderWriter, VariableIdentifier, VariableRegistry};

use super::procedures::{self, ProcedureStep, CONDITION, ECON, EXON, EXPEDITED_DELAY, NCON, PROC};
use super::rpn;
use super::sim::PresetHost;
use super::{named, XplaneIo};

/// A feed tank below this many gallons cannot reliably keep an engine
/// running through an expedited start -- well under FlyByWire's own full
/// feed-tank default of 1233.9 gal (`crate::fuel::DEFAULT_GALLONS`), and
/// far above what a merely-thirsty-but-workable tank would read.
const MIN_FEED_TANK_GALLONS_TO_START: f64 = 100.;

/// The four engine feed tanks' network tank numbers (`crate::fuel`'s own
/// `ENGINE_FEED_TANKS`, duplicated here rather than exposed to avoid
/// widening fuel.rs's own surface for one constant array).
const FEED_TANK_NETWORK_INDEXES: [usize; 4] = [2, 5, 6, 9];

/// How many procedure steps an expedited load may run within one call to
/// `update`. Every step either advances `current_step` (bounded by
/// `current_procedure.len()`, at most a few hundred) or stops the loop, so
/// this is a generous safety cap against a runaway loop, not a limit meant
/// to ever bind in practice.
const MAX_EXPEDITED_STEPS_PER_TICK: usize = 2000;

pub struct AircraftPresets {
    load_request: VariableIdentifier,
    progress: VariableIdentifier,
    verbose: VariableIdentifier,
    expedite: VariableIdentifier,
    expedite_delay: VariableIdentifier,
    quick_mode: VariableIdentifier,
    is_ready: VariableIdentifier,
    on_ground: VariableIdentifier,

    current_procedure_id: i64,
    current_procedure: Vec<ProcedureStep>,
    loading_is_active: bool,
    /// Milliseconds.
    current_loading_time: f64,
    current_delay: f64,
    current_step: usize,
    /// The last progress message the flyPad would have been sent.
    pub progress_text: String,
    /// The procedure XML: FlyByWire's, or a copy beside the plugin.
    xml: Option<String>,
    /// Whether this load has already checked/topped up the feed tanks
    /// (once per load, not every tick).
    fuel_checked_this_load: bool,
    /// A fuel top-up wrote `REFUEL_STARTED_BY_USR` = 1 last tick; clear it
    /// back to 0 the next time `update` runs so fuel.rs sees a real,
    /// one-tick refuel pulse (its own "refuel end detected" path) rather
    /// than a refuel left stuck open.
    clear_refuel_started: bool,
}

fn logged(message: &str) {
    crate::log(&format!("extra backend: {message}"));
}

impl AircraftPresets {
    /// AircraftPresets::initialize (cpp:43-61).
    pub fn new<V: VariableRegistry + SimulatorReaderWriter>(vars: &mut V) -> Self {
        let load_request = named(vars, "AIRCRAFT_PRESET_LOAD");
        let progress = named(vars, "AIRCRAFT_PRESET_LOAD_PROGRESS");
        vars.write(&load_request, 0.);
        let quick_mode = named(vars, "AIRCRAFT_PRESET_QUICK_MODE");
        vars.write(&quick_mode, 0.);
        Self {
            load_request,
            progress,
            verbose: named(vars, "AIRCRAFT_PRESET_VERBOSE"),
            expedite: named(vars, "AIRCRAFT_PRESET_LOAD_EXPEDITE"),
            expedite_delay: named(vars, "AIRCRAFT_PRESET_LOAD_EXPEDITE_DELAY"),
            quick_mode,
            is_ready: named(vars, "IS_READY"),
            on_ground: vars.get("SIM ON GROUND".to_string()),
            current_procedure_id: 0,
            current_procedure: Vec::new(),
            loading_is_active: false,
            current_loading_time: 0.,
            current_delay: 0.,
            current_step: 0,
            progress_text: String::new(),
            xml: None,
            fuel_checked_this_load: false,
            clear_refuel_started: false,
        }
    }

    /// Use this procedure text instead of FlyByWire's file (tests, or an
    /// edited copy).
    pub fn with_xml(mut self, xml: String) -> Self {
        self.xml = Some(xml);
        self
    }

    fn procedure_xml(&self) -> String {
        if let Some(xml) = &self.xml {
            return xml.clone();
        }
        // FlyByWire re-reads its file on each request so it can be edited
        // live (PresetProcedures.hpp:71-95); a copy beside the plugin plays
        // that part.
        #[cfg(feature = "js")]
        if let Some(text) = crate::js_bridge::plugin_dir()
            .and_then(|d| std::fs::read_to_string(d.join("aircraft_preset_procedures.xml")).ok())
        {
            return text;
        }
        procedures::PROCEDURES_XML.to_string()
    }

    /// AircraftPresets::update (cpp:63-167). `delta` is the frame's seconds
    /// (`pData->dt`).
    ///
    /// Normal mode keeps FlyByWire's own timed pacing exactly: at most one
    /// step per frame, gated by `current_delay`. Expedited mode instead
    /// runs every step back to back within this one call (`run_step` in a
    /// bounded loop, below) -- no artificial per-step pacing -- stopping
    /// only when the load finishes/is cancelled or a COND step's condition
    /// genuinely is not yet true, which the very next tick simply checks
    /// again rather than waiting out an additional delay on top.
    pub fn update<V: VariableRegistry + SimulatorReaderWriter, X: XplaneIo>(&mut self, vars: &mut V, xplane: &mut X, delta: f64) {
        if self.clear_refuel_started {
            // The one-tick refuel pulse `ensure_fuel_for_engine_start` sent
            // last tick: end it now so fuel.rs sees a real, momentary
            // refuel (its own "refuel end detected" path), not one left
            // stuck open.
            let refuel = named(vars, "REFUEL_STARTED_BY_USR");
            vars.write(&refuel, 0.);
            self.clear_refuel_started = false;
        }
        if vars.read(&self.is_ready) == 0. {
            return;
        }
        let request = vars.read(&self.load_request) as i64;
        if request <= 0 {
            if self.loading_is_active {
                self.finish_loading(vars);
            }
            return;
        }
        if vars.read(&self.on_ground) == 0. {
            logged("AircraftPresets: Aircraft must be on the ground to load a preset!");
            self.finish_loading(vars);
            return;
        }
        if !self.loading_is_active {
            match procedures::preset(&self.procedure_xml(), request) {
                Ok(steps) => self.initialize_new_loading_process(vars, request, steps),
                Err(e) => {
                    logged(&e);
                    logged(&format!("AircraftPresets: Preset {request} not found!"));
                    self.finish_loading(vars);
                }
            }
            return;
        }
        // Only 0 may interrupt a running procedure.
        vars.write(&self.load_request, self.current_procedure_id as f64);
        let expedited = vars.read(&self.expedite) != 0.;
        vars.write(&self.quick_mode, if expedited { 1. } else { 0. });

        if !expedited {
            self.current_loading_time += delta * 1000.;
            if self.current_loading_time <= self.current_delay {
                return;
            }
            self.run_step(vars, xplane, false);
            return;
        }

        if !self.fuel_checked_this_load {
            self.ensure_fuel_for_engine_start(vars);
            self.fuel_checked_this_load = true;
        }
        self.current_loading_time += delta * 1000.;
        for _ in 0..MAX_EXPEDITED_STEPS_PER_TICK {
            if vars.read(&self.load_request) as i64 <= 0 {
                return;
            }
            if !self.run_step(vars, xplane, true) {
                return;
            }
        }
    }

    /// Runs one step of the active procedure (the body of the old `update`,
    /// cpp:139-167, past its pacing gate). Returns whether the caller may
    /// immediately try another step in the same call: false once the load
    /// has finished (`check_completion` already called `finish_loading`) or
    /// a COND step's condition read false and must wait for a later tick.
    fn run_step<V: VariableRegistry + SimulatorReaderWriter, X: XplaneIo>(&mut self, vars: &mut V, xplane: &mut X, expedited: bool) -> bool {
        if self.check_completion(vars) {
            return false;
        }
        let step = self.current_procedure[self.current_step].clone();
        if self.check_step_type_skipping(expedited, &step) {
            return true;
        }
        self.current_delay = self.current_loading_time + step.delay_after;
        let mut host = PresetHost { vars, xplane };
        if step.step_type & CONDITION != 0 {
            let before = self.current_step;
            self.handle_condition_step(&mut host, &step);
            return self.current_step != before;
        }
        if expedited && step.step_type & EXPEDITED_DELAY == 0 {
            self.current_delay = self.current_loading_time + host.vars.read(&self.expedite_delay);
        }
        if self.check_expected_state(&mut host, &step) {
            return true;
        }
        self.update_progress(host.vars, &step);
        self.execute_action(&mut host, &step);
        true
    }

    /// Before an expedited load's engine-start steps run: if a feed tank is
    /// too low to actually keep an engine running (feed tanks can end up
    /// drained by repeated on-ground engine runs with nothing refilling
    /// them, since FlyByWire's own transfer logic only runs in flight --
    /// `fuel_transfer.rs`'s `!on_ground` gate, ported from LegacyFuel.ts),
    /// fill every tank to FlyByWire's own FADEC default (`crate::fuel::
    /// DEFAULT_GALLONS`: the four feed tanks at 1233.9 gal, the rest empty --
    /// not an invented number). Written through `FUEL_TANK_QUANTITY_n` and a
    /// one-tick `REFUEL_STARTED_BY_USR` pulse, the same path a real EFB
    /// refuel uses (`fuel.rs::update`), so fuel.rs's network -- and through
    /// it X-Plane's own tanks -- follow normally; nothing here pokes the
    /// network or the ini directly.
    fn ensure_fuel_for_engine_start<V: VariableRegistry + SimulatorReaderWriter>(&mut self, vars: &mut V) {
        let feed_low = FEED_TANK_NETWORK_INDEXES.iter().any(|&i| {
            let id = named(vars, &format!("FUEL_TANK_QUANTITY_{i}"));
            vars.read(&id) < MIN_FEED_TANK_GALLONS_TO_START
        });
        if !feed_low {
            return;
        }
        logged(
            "AircraftPresets: expedited load found a feed tank too low to start the engines from; \
             filling to FlyByWire's own default fuel load before the engine steps run.",
        );
        for (i, gallons) in crate::fuel::DEFAULT_GALLONS.iter().enumerate() {
            let id = named(vars, &format!("FUEL_TANK_QUANTITY_{}", i + 1));
            vars.write(&id, *gallons);
        }
        let refuel = named(vars, "REFUEL_STARTED_BY_USR");
        vars.write(&refuel, 1.);
        self.clear_refuel_started = true;
    }

    /// cpp:179-186.
    fn update_progress<V: SimulatorReaderWriter>(&mut self, vars: &mut V, step: &ProcedureStep) {
        let fraction = self.current_step as f64 / self.current_procedure.len() as f64;
        vars.write(&self.progress, fraction);
        self.progress_text = format!("{fraction};{}", step.description);
    }

    /// cpp:188-195.
    fn check_completion<V: SimulatorReaderWriter>(&mut self, vars: &mut V) -> bool {
        if self.current_step >= self.current_procedure.len() {
            logged(&format!("AircraftPresets: Aircraft Preset {} done!", self.current_procedure_id));
            self.finish_loading(vars);
            return true;
        }
        false
    }

    /// cpp:197-206.
    fn initialize_new_loading_process<V: SimulatorReaderWriter>(&mut self, vars: &mut V, id: i64, steps: Vec<ProcedureStep>) {
        logged(&format!("AircraftPresets: Aircraft Preset {id} starting procedure!"));
        self.current_procedure_id = id;
        self.current_procedure = steps;
        self.current_loading_time = 0.;
        self.current_delay = 0.;
        self.current_step = 0;
        self.loading_is_active = true;
        self.fuel_checked_this_load = false;
        vars.write(&self.progress, 0.);
    }

    /// cpp:208-218.
    fn check_step_type_skipping(&mut self, expedited: bool, step: &ProcedureStep) -> bool {
        let t = step.step_type;
        if (expedited && t == PROC) || (!expedited && t == EXON) || (expedited && t == NCON) || (!expedited && t == ECON) {
            self.current_step += 1;
            return true;
        }
        false
    }

    /// cpp:220-234.
    fn handle_condition_step<V: VariableRegistry + SimulatorReaderWriter, X: XplaneIo>(
        &mut self,
        host: &mut PresetHost<'_, V, X>,
        step: &ProcedureStep,
    ) {
        self.update_progress(host.vars, step);
        if host.vars.read(&self.verbose) != 0. {
            logged(&format!("AircraftPresets: Aircraft Preset Step Condition: [{}]", step.expected_state_check_code));
        }
        let value = rpn::execute(&step.expected_state_check_code, host);
        if value != 0. {
            self.current_delay = 0.;
            self.current_step += 1;
        }
    }

    /// cpp:236-261.
    fn check_expected_state<V: VariableRegistry + SimulatorReaderWriter, X: XplaneIo>(
        &mut self,
        host: &mut PresetHost<'_, V, X>,
        step: &ProcedureStep,
    ) -> bool {
        if step.expected_state_check_code.is_empty() {
            return false;
        }
        let value = rpn::execute(&step.expected_state_check_code, host);
        if value != 0. {
            if host.vars.read(&self.verbose) != 0. {
                logged(&format!("AircraftPresets: Aircraft Preset Step {} Skipping: {}", self.current_step, step.description));
            }
            self.current_delay = 0.;
            self.current_step += 1;
            return true;
        }
        false
    }

    /// cpp:263-271.
    fn execute_action<V: VariableRegistry + SimulatorReaderWriter, X: XplaneIo>(
        &mut self,
        host: &mut PresetHost<'_, V, X>,
        step: &ProcedureStep,
    ) {
        logged(&format!(
            "AircraftPresets: Aircraft Preset Step {} Execute: {} (delay after: {})",
            self.current_step,
            step.description,
            (self.current_delay - self.current_loading_time) as i64
        ));
        rpn::execute(&step.action_code, host);
        self.current_step += 1;
    }

    /// cpp:273-280.
    fn finish_loading<V: SimulatorReaderWriter>(&mut self, vars: &mut V) {
        logged(&format!("AircraftPresets:update() Aircraft Preset {} loading finished or cancelled!", self.current_procedure_id));
        vars.write(&self.load_request, 0.);
        vars.write(&self.progress, 0.);
        vars.write(&self.quick_mode, 0.);
        self.loading_is_active = false;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::aspects::test_vars::TestVars;
    use crate::extra_backend::sim::test_xplane::FakeXplane;

    const XML: &str = r#"<AircraftPresetProcedures>
        <Procedure Name="POWERUP_CONFIG_ON">
            <Step Name="BAT1 On" Type="STEP" Delay="1000">
                <Condition>(L:A32NX_OVHD_ELEC_BAT_1_PB_IS_AUTO)</Condition>
                <Action>1 (&gt;L:A32NX_OVHD_ELEC_BAT_1_PB_IS_AUTO)</Action>
            </Step>
            <Step Name="Await AC BUS ON" Type="COND" Delay="2000">
                <Condition>(L:A32NX_ELEC_AC_1_BUS_IS_POWERED)</Condition>
            </Step>
            <Step Name="Only expedited" Type="EXON" Delay="0">
                <Action>1 (&gt;L:A32NX_EXPEDITED_ONLY)</Action>
            </Step>
            <Step Name="Beacon On" Type="STEP" Delay="1000">
                <Condition>(A:LIGHT BEACON, Bool)</Condition>
                <Action>0 (&gt;K:BEACON_LIGHTS_ON)</Action>
            </Step>
        </Procedure>
    </AircraftPresetProcedures>"#;

    fn run(p: &mut AircraftPresets, vars: &mut TestVars, xp: &mut FakeXplane, seconds: f64) {
        let dt = 1. / 30.;
        for _ in 0..(seconds / dt) as usize {
            p.update(vars, xp, dt);
        }
    }

    #[test]
    fn powered_preset_runs_its_steps_with_their_delays_and_waits_on_conditions() {
        let mut vars = TestVars::default();
        let mut xp = FakeXplane::default();
        let mut p = AircraftPresets::new(&mut vars).with_xml(XML.to_string());
        vars.set("A32NX_IS_READY", 1.);
        vars.set("SIM ON GROUND", 1.);
        vars.set("A32NX_AIRCRAFT_PRESET_LOAD", 2.);
        run(&mut p, &mut vars, &mut xp, 0.5);
        // Step 1 ran on the second frame; its 1 s delay holds step 2.
        assert_eq!(vars.value("A32NX_OVHD_ELEC_BAT_1_PB_IS_AUTO"), 1.);
        assert_eq!(p.current_step, 1);
        // The condition step waits for the bus.
        run(&mut p, &mut vars, &mut xp, 5.);
        assert_eq!(p.current_step, 1);
        assert_eq!(vars.value("A32NX_AIRCRAFT_PRESET_LOAD"), 2.);
        assert!(p.progress_text.ends_with("Await AC BUS ON"));
        vars.set("A32NX_ELEC_AC_1_BUS_IS_POWERED", 1.);
        run(&mut p, &mut vars, &mut xp, 5.);
        // The EXON step is skipped in normal mode; the beacon comes on; done.
        assert_eq!(vars.value("A32NX_EXPEDITED_ONLY"), 0.);
        assert_eq!(xp.values["sim/cockpit2/switches/beacon_on"], 1.);
        assert_eq!(vars.value("A32NX_AIRCRAFT_PRESET_LOAD"), 0.);
        assert!(!p.loading_is_active);
    }

    #[test]
    fn expedited_load_runs_every_step_back_to_back_within_one_call() {
        let mut vars = TestVars::default();
        let mut xp = FakeXplane::default();
        let mut p = AircraftPresets::new(&mut vars).with_xml(XML.to_string());
        vars.set("A32NX_IS_READY", 1.);
        vars.set("SIM ON GROUND", 1.);
        vars.set("A32NX_AIRCRAFT_PRESET_LOAD_EXPEDITE", 1.);
        vars.set("A32NX_AIRCRAFT_PRESET_LOAD", 2.);
        // One call only starts the load (initialize_new_loading_process
        // still returns immediately); the next call is what should run
        // every ready step back to back, not the one-per-frame pacing
        // normal mode uses.
        p.update(&mut vars, &mut xp, 1. / 30.);
        p.update(&mut vars, &mut xp, 1. / 30.);
        assert_eq!(vars.value("A32NX_OVHD_ELEC_BAT_1_PB_IS_AUTO"), 1., "the first step's action already ran, no 1 s delay");
        assert_eq!(p.current_step, 1, "stopped at the COND step, whose condition genuinely is not yet true");
        assert!(p.loading_is_active);

        // Satisfying the condition lets the rest complete within one more
        // call too, not several more seconds of per-step pacing.
        vars.set("A32NX_ELEC_AC_1_BUS_IS_POWERED", 1.);
        p.update(&mut vars, &mut xp, 1. / 30.);
        assert_eq!(vars.value("A32NX_EXPEDITED_ONLY"), 1., "EXON runs (not skipped) when expedited");
        assert_eq!(xp.values["sim/cockpit2/switches/beacon_on"], 1.);
        assert_eq!(vars.value("A32NX_AIRCRAFT_PRESET_LOAD"), 0.);
        assert!(!p.loading_is_active);
    }

    #[test]
    fn expedited_load_fills_feed_tanks_from_flybywires_default_when_too_low_to_start() {
        let mut vars = TestVars::default();
        let mut xp = FakeXplane::default();
        let mut p = AircraftPresets::new(&mut vars).with_xml(XML.to_string());
        vars.set("A32NX_IS_READY", 1.);
        vars.set("SIM ON GROUND", 1.);
        vars.set("A32NX_AIRCRAFT_PRESET_LOAD_EXPEDITE", 1.);
        // A near-empty feed tank, as the live report's ini/X-Plane tanks
        // found (FEED ONE/TWO/THREE at ~0.39 gal).
        vars.set("A32NX_FUEL_TANK_QUANTITY_2", 0.39);
        vars.set("A32NX_FUEL_TANK_QUANTITY_5", 0.38);
        vars.set("A32NX_FUEL_TANK_QUANTITY_6", 0.39);
        vars.set("A32NX_FUEL_TANK_QUANTITY_9", 829.);
        vars.set("A32NX_AIRCRAFT_PRESET_LOAD", 2.);
        p.update(&mut vars, &mut xp, 1. / 30.); // starts the load
        p.update(&mut vars, &mut xp, 1. / 30.); // first real tick: tops up fuel once

        for i in [2, 5, 6, 9] {
            assert_eq!(
                vars.value(&format!("A32NX_FUEL_TANK_QUANTITY_{i}")),
                1233.9,
                "tank {i} should hold FlyByWire's own FADEC default, not an invented number"
            );
        }
        assert_eq!(
            vars.value("A32NX_REFUEL_STARTED_BY_USR"),
            1.,
            "the fill goes through the same REFUEL_STARTED_BY_USR path a real EFB refuel uses"
        );
        p.update(&mut vars, &mut xp, 1. / 30.); // next tick clears the one-tick pulse
        assert_eq!(vars.value("A32NX_REFUEL_STARTED_BY_USR"), 0.);
    }

    #[test]
    fn expedited_load_leaves_already_adequate_feed_tanks_alone() {
        let mut vars = TestVars::default();
        let mut xp = FakeXplane::default();
        let mut p = AircraftPresets::new(&mut vars).with_xml(XML.to_string());
        vars.set("A32NX_IS_READY", 1.);
        vars.set("SIM ON GROUND", 1.);
        vars.set("A32NX_AIRCRAFT_PRESET_LOAD_EXPEDITE", 1.);
        for i in [2, 5, 6, 9] {
            vars.set(&format!("A32NX_FUEL_TANK_QUANTITY_{i}"), 1233.9);
        }
        vars.set("A32NX_AIRCRAFT_PRESET_LOAD", 2.);
        p.update(&mut vars, &mut xp, 1. / 30.);
        p.update(&mut vars, &mut xp, 1. / 30.);
        assert_eq!(vars.value("A32NX_REFUEL_STARTED_BY_USR"), 0., "already enough fuel to start: no refuel pulse needed");
    }

    #[test]
    fn presets_do_not_load_in_the_air_and_zero_cancels() {
        let mut vars = TestVars::default();
        let mut xp = FakeXplane::default();
        let mut p = AircraftPresets::new(&mut vars).with_xml(XML.to_string());
        vars.set("A32NX_IS_READY", 1.);
        vars.set("A32NX_AIRCRAFT_PRESET_LOAD", 2.);
        run(&mut p, &mut vars, &mut xp, 0.2);
        assert_eq!(vars.value("A32NX_AIRCRAFT_PRESET_LOAD"), 0.);
        assert_eq!(vars.value("A32NX_OVHD_ELEC_BAT_1_PB_IS_AUTO"), 0.);

        vars.set("SIM ON GROUND", 1.);
        vars.set("A32NX_AIRCRAFT_PRESET_LOAD", 2.);
        run(&mut p, &mut vars, &mut xp, 0.2);
        assert!(p.loading_is_active);
        vars.set("A32NX_AIRCRAFT_PRESET_LOAD", 0.);
        run(&mut p, &mut vars, &mut xp, 0.1);
        assert!(!p.loading_is_active);
    }

    #[test]
    fn flybywire_ready_for_takeoff_preset_steps_all_parse_and_evaluate() {
        // Every condition and action in FlyByWire's file runs through the
        // calculator without leaving anything unknown behind.
        let steps = procedures::preset(procedures::PROCEDURES_XML, 5).unwrap();
        let mut vars = TestVars::default();
        let mut xp = FakeXplane::default();
        let mut host = PresetHost { vars: &mut vars, xplane: &mut xp };
        for s in &steps {
            rpn::execute(&s.expected_state_check_code, &mut host);
            rpn::execute(&s.action_code, &mut host);
        }
        assert!(steps.len() > 50);
        assert_eq!(host.vars.value("GENERAL ENG STARTER:4"), 1.);
    }
}
