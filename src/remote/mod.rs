//! FlyByWire's systems in a process of their own.
//!
//! The plugin keeps X-Plane's side (datarefs, the cockpit, the instruments);
//! `fbw_a380_systems_server.exe` runs FlyByWire's `Simulation<A380>`. They
//! share one block of memory holding every variable the systems registered
//! and hand it back and forth once a frame (wire.rs), so the systems tick on
//! the same frame's inputs and their outputs reach the same frame's later
//! steps: lockstep, no frame of lag on the controls. A crash in the systems
//! stops the systems, not X-Plane.
//!
//! Without the server executable, or with `FBW_SYSTEMS_IN_PROCESS=1`, the
//! systems run inside the plugin as before.

pub mod client;
pub mod server;
pub mod win;
pub mod wire;

use std::time::Duration;

use a380_systems::A380;
use rustc_hash::FxHashSet;
use systems::failures::FailureType;
use systems::simulation::{Simulation, SimulatorReaderWriter};

/// Where the systems run.
pub enum Systems {
    Local(Box<Simulation<A380>>),
    Remote(client::RemoteSystems),
}

impl Systems {
    pub fn tick<V: SimulatorReaderWriter>(&mut self, delta: Duration, time: f64, vars: &mut V) {
        match self {
            Systems::Local(s) => s.tick(delta, time, vars),
            Systems::Remote(r) => {
                if let Some(message) = r.tick(delta, time, vars) {
                    crate::log(&format!("systems process: {message}"));
                }
            }
        }
    }

    pub fn is_remote(&self) -> bool {
        matches!(self, Systems::Remote(_))
    }
}

/// Whatever takes the set of active failures before a tick.
pub trait FailureSink {
    fn update_active_failures(&mut self, set: FxHashSet<FailureType>);
}

impl FailureSink for Simulation<A380> {
    fn update_active_failures(&mut self, set: FxHashSet<FailureType>) {
        Simulation::update_active_failures(self, set)
    }
}

impl FailureSink for Systems {
    fn update_active_failures(&mut self, set: FxHashSet<FailureType>) {
        match self {
            Systems::Local(s) => s.update_active_failures(set),
            Systems::Remote(r) => r.update_active_failures(set),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::aspects::test_vars::TestVars;
    use systems::simulation::StartState;

    /// The systems served from a thread of this process over the same shared
    /// memory and events a separate process would use.
    fn remote(state: StartState, vars: &mut TestVars) -> client::RemoteSystems {
        client::RemoteSystems::connect(state, vars, |tag| {
            let tag = tag.to_string();
            std::thread::Builder::new()
                .stack_size(256 << 20)
                .spawn(move || {
                    let _ = server::serve(&tag, None);
                })
                .map(|_| None)
                .map_err(|e| e.to_string())
        })
        .expect("the systems connect")
    }

    fn inputs(vars: &mut TestVars, tick: usize) {
        vars.set("AMBIENT PRESSURE", 29.92);
        vars.set("AMBIENT TEMPERATURE", 15.);
        vars.set("AMBIENT DENSITY", 0.002377);
        vars.set("SEA LEVEL PRESSURE", 1013.25);
        for n in 1..=4 {
            // Ground power on after a second, off for a while later: inputs
            // that change what the systems do mid-run.
            let on = (20..4000).contains(&tick);
            vars.set(&format!("A32NX_EXT_PWR_AVAIL:{n}"), on as i32 as f64);
            vars.set(&format!("A32NX_OVHD_ELEC_EXT_PWR_{n}_PB_IS_ON"), on as i32 as f64);
        }
    }

    #[test]
    fn the_systems_in_their_own_process_switch_on_the_same_frame_as_in_the_plugin() {
        // Two copies of FlyByWire's systems already differ in their last
        // digits run for run (the electrical solver's sums follow hash map
        // order), so outputs are not compared bit for bit. Lag is what the
        // bridge could add, and a frame of lag moves a switch-over: every
        // bus's powered state must change on the very frame it does in the
        // plugin, as ground power comes and goes.
        let _g = crate::failures::tests::SERIAL.lock().unwrap_or_else(|e| e.into_inner());
        let state = StartState::Apron;
        let mut local_vars = TestVars::default();
        let mut local = Simulation::new(state, A380::new, &mut local_vars);
        let mut remote_vars = TestVars::default();
        let mut remote = remote(state, &mut remote_vars);
        assert!(remote.variable_count() > 1000, "{} variables mirrored", remote.variable_count());

        let flags: Vec<String> = local_vars
            .index
            .keys()
            .filter(|n| n.starts_with("A32NX_ELEC_") && n.ends_with("_BUS_IS_POWERED"))
            .cloned()
            .collect();
        assert!(flags.len() >= 10, "{flags:?}");
        let delta = Duration::from_millis(50);
        let mut switches = 0;
        let mut last: Vec<f64> = vec![0.; flags.len()];
        for tick in 0..6000 {
            let time = tick as f64 * 0.05;
            inputs(&mut local_vars, tick);
            inputs(&mut remote_vars, tick);
            local.tick(delta, time, &mut local_vars);
            assert!(remote.tick(delta, time, &mut remote_vars).is_none(), "tick {tick} reported a failure");
            for (k, name) in flags.iter().enumerate() {
                let (a, b) = (local_vars.value(name), remote_vars.value(name));
                assert_eq!(a, b, "tick {tick}: {name} is {a} in the plugin and {b} out of process");
                if a != last[k] {
                    switches += 1;
                    last[k] = a;
                }
            }
        }
        assert!(switches >= 20, "only {switches} bus switch-overs exercised");
        assert_eq!(remote.stats.late, 0);
    }

    /// Regression test for the "Ready for Takeoff preset leaves the aircraft
    /// cold and dark" report: loads FlyByWire's real "Powered" preset (2)
    /// through `AircraftPresets` against the real, local
    /// `a380_systems::Simulation` (not a stub) and checks that AC BUS 1
    /// energises.
    ///
    /// What it was. FlyByWire's pushbuttons keep their state in a simulation
    /// variable, and `SimulationElement` runs read before update before
    /// write -- so whatever `OnOffFaultPushButton::new_on` constructed was
    /// overwritten by the first read of a variable nobody had set. Both APU
    /// generator pushbuttons read off on tick one and latched there. The APU
    /// started, reached AVAILABLE, and its generator sat at a perfectly
    /// healthy 115 V / 400 Hz reporting `output_within_normal_parameters`
    /// with nothing connected to it, so FlyByWire's own ECAM correctly
    /// reported ELEC EMER CONFIG forever.
    ///
    /// FlyByWire's preset procedure never turns those buttons on, and should
    /// not: there is no APU GEN step anywhere in
    /// `aircraft_preset_procedures.xml`, because on the real aircraft they
    /// are already in. In MSFS the aircraft's own panel state sets them
    /// before the systems run; here nothing did, until
    /// `crate::seed_overhead_defaults`.
    ///
    /// Seeding those defaults energises AC BUS 1 within 20 simulated
    /// seconds, and the bus voltage then sags to 113.4 V under real load,
    /// which is the electrical model working rather than idling.
    #[test]
    fn powered_preset_against_real_systems_energises_ac_bus_1() {
        use crate::extra_backend::aircraft_presets::AircraftPresets;
        use crate::extra_backend::sim::test_xplane::FakeXplane;
        let _g = crate::failures::tests::SERIAL.lock().unwrap_or_else(|e| e.into_inner());
        let mut vars = TestVars::default();
        let mut systems = Simulation::new(StartState::Apron, A380::new, &mut vars);
        let mut presets = AircraftPresets::new(&mut vars);
        let mut xp = FakeXplane::default();
        vars.set("A32NX_IS_READY", 1.);
        vars.set("SIM ON GROUND", 1.);
        vars.set("AMBIENT PRESSURE", 29.92);
        vars.set("AMBIENT TEMPERATURE", 15.);
        vars.set("AMBIENT DENSITY", 0.002377);
        vars.set("SEA LEVEL PRESSURE", 1013.25);
        // The factory positions the plugin seeds at startup
        // (crate::seed_overhead_defaults). The plugin queues them
        // through the Study write queue, which this test does not
        // drain, so apply the same list directly.
        for name in crate::OVERHEAD_DEFAULTS_ON {
            vars.set(&format!("A32NX_{name}"), 1.);
        }
        vars.set("A32NX_AIRCRAFT_PRESET_LOAD_EXPEDITE", 1.);
        vars.set("A32NX_AIRCRAFT_PRESET_LOAD", 2.);
        let delta = std::time::Duration::from_millis(50);
        // 4000 * 50 ms = 200 simulated seconds: the APU reaches AVAILABLE
        // well inside the first minute, so this leaves a wide margin.
        let mut ac1_ever_powered = false;
        for tick in 0..4000 {
            let time = tick as f64 * 0.05;
            presets.update(&mut vars, &mut xp, delta.as_secs_f64());
            systems.tick(delta, time, &mut vars);
            ac1_ever_powered |= vars.value("A32NX_ELEC_AC_1_BUS_IS_POWERED") != 0.;
        }
        assert!(
            vars.value("A32NX_OVHD_APU_START_PB_IS_AVAILABLE") != 0.,
            "test setup regressed: the APU itself no longer reaches AVAILABLE in 200 s"
        );
        assert!(
            ac1_ever_powered,
            "AC BUS 1 never powered up from the running, available APU -- see this test's doc comment"
        );
    }

    /// Extends the "Powered" regression above to the panels this task added
    /// to `crate::OVERHEAD_DEFAULTS_ON`: hydraulics and bleed/pneumatic.
    /// Loads the same real "Powered" preset (2) the sibling test above does
    /// -- reaching a running, AVAILABLE APU and energised AC/DC buses --
    /// then feeds engine 1's `TrentEngine` a normal running N2/N3 the way
    /// MSFS's own turbine model otherwise would (`TrentEngine::update` is a
    /// no-op; it only reads `ENGINE_N2`/`ENGINE_N3`, trent_engine.rs
    /// line 73/122-ish), and checks two things against the real, local
    /// `a380_systems::Simulation` (not a stub):
    ///
    /// - the APU bleed valve actually opens (`APU_BLEED_AIR_VALVE_OPEN`,
    ///   pneumatic.rs's `apu_bleed_air_valve_open_id`), proving the APU
    ///   bleed pushbutton's seeded ON default
    ///   (`OVHD_PNEU_APU_BLEED_PB_IS_ON`) reached the pneumatic system and
    ///   not just the variable;
    /// - EDP 1A actually pressurises (`HYD_EDPUMP_1A_LOW_PRESS` clears once
    ///   engine 1 is "running"), proving the EDP pushbutton's seeded AUTO
    ///   default (`OVHD_HYD_ENG_1A_PUMP_PB_IS_AUTO`) let
    ///   `A380EngineDrivenPumpController::update` take the "should
    ///   pressurise" branch instead of the explicit "button reads OFF, stay
    ///   depressurised" branch this bug used to force once DC power came
    ///   up.
    ///
    /// Driving the engine directly through its own external N2/N3 inputs,
    /// rather than through FlyByWire's "Ready for Takeoff" preset and this
    /// plugin's own fadec/key-event glue (`src/fadec.rs`, `src/fuel.rs`,
    /// `src/extra_backend/sim.rs`), keeps this pinned to `a380_systems`
    /// itself: `ENGINE_STATE` -- what that preset's engine-start steps wait
    /// on -- is written by this plugin's own port of FlyByWire's separate
    /// MSFS-side FADEC gauge, not by anything `Simulation<A380>::tick`
    /// drives (confirmed by grepping FlyByWire's own systems source: nothing
    /// under `fbw-common`/`fbw-a380x`'s `systems` crates ever writes
    /// `ENGINE_STATE`, only reads it), so a preset-driven version of this
    /// test would really be pinning that unrelated glue rather than the
    /// overhead-pushbutton defaults this task is about.
    #[test]
    fn powered_preset_against_real_systems_pressurises_hydraulics_and_bleed_air() {
        use crate::extra_backend::aircraft_presets::AircraftPresets;
        use crate::extra_backend::sim::test_xplane::FakeXplane;
        let _g = crate::failures::tests::SERIAL.lock().unwrap_or_else(|e| e.into_inner());
        let mut vars = TestVars::default();
        let mut systems = Simulation::new(StartState::Apron, A380::new, &mut vars);
        let mut presets = AircraftPresets::new(&mut vars);
        let mut xp = FakeXplane::default();
        vars.set("A32NX_IS_READY", 1.);
        vars.set("SIM ON GROUND", 1.);
        vars.set("AMBIENT PRESSURE", 29.92);
        vars.set("AMBIENT TEMPERATURE", 15.);
        vars.set("AMBIENT DENSITY", 0.002377);
        vars.set("SEA LEVEL PRESSURE", 1013.25);
        // The factory positions the plugin seeds at startup
        // (crate::seed_overhead_defaults); see the sibling test above for
        // why this test applies the same list directly instead of going
        // through the Study write queue.
        for name in crate::OVERHEAD_DEFAULTS_ON {
            vars.set(&format!("A32NX_{name}"), 1.);
        }
        vars.set("A32NX_AIRCRAFT_PRESET_LOAD_EXPEDITE", 1.);
        vars.set("A32NX_AIRCRAFT_PRESET_LOAD", 2.);
        let delta = std::time::Duration::from_millis(50);
        // 6000 * 50 ms = 300 simulated seconds: the sibling test above
        // reaches AC BUS 1 powered within 20 s, so this leaves a wide
        // margin for the APU/electrical chain plus however long the
        // hydraulic circuit's own fluid dynamics take to build pressure
        // once EDP 1A starts trying.
        let mut apu_bleed_valve_ever_open = false;
        let mut edp_1a_ever_pressurised = false;
        for tick in 0..6000 {
            let time = tick as f64 * 0.05;
            presets.update(&mut vars, &mut xp, delta.as_secs_f64());
            // A normal running speed for engine 1's Trent, fed the same way
            // MSFS's own turbine model otherwise would.
            vars.set("A32NX_ENGINE_N2:1", 70.);
            vars.set("A32NX_ENGINE_N3:1", 70.);
            systems.tick(delta, time, &mut vars);
            apu_bleed_valve_ever_open |= vars.value("A32NX_APU_BLEED_AIR_VALVE_OPEN") != 0.;
            edp_1a_ever_pressurised |= vars.value("A32NX_HYD_EDPUMP_1A_LOW_PRESS") == 0.;
        }
        eprintln!(
            "DIAG dc_ess={} hyd_1a_auto={} pneu_apu_bleed_on={} low_press={}",
            vars.value("A32NX_ELEC_DC_ESS_BUS_IS_POWERED"),
            vars.value("A32NX_OVHD_HYD_ENG_1A_PUMP_PB_IS_AUTO"),
            vars.value("A32NX_OVHD_PNEU_APU_BLEED_PB_IS_ON"),
            vars.value("A32NX_HYD_EDPUMP_1A_LOW_PRESS"),
        );
        assert!(
            vars.value("A32NX_OVHD_APU_START_PB_IS_AVAILABLE") != 0.,
            "test setup regressed: the APU itself no longer reaches AVAILABLE in 300 s"
        );
        assert!(
            apu_bleed_valve_ever_open,
            "the APU bleed valve never opened -- see this test's doc comment"
        );
        assert!(
            edp_1a_ever_pressurised,
            "EDP 1A never pressurised with engine 1 \"running\" -- see this test's doc comment"
        );
    }

    /// The real executable as its own process: builds, mirrors, and ticks a
    /// few thousand frames in lockstep, reporting the round-trip cost.
    #[test]
    #[ignore = "needs target/release/fbw_a380_systems_server.exe built"]
    fn the_systems_process_runs_in_lockstep() {
        let exe = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../fbw-build/target-main/release/fbw_a380_systems_server.exe");
        let exe = std::env::var("FBW_SERVER_EXE").map(std::path::PathBuf::from).unwrap_or(exe);
        let mut vars = TestVars::default();
        let started = std::time::Instant::now();
        let mut remote = client::RemoteSystems::start(&exe, StartState::Apron, &mut vars).expect("the process starts");
        eprintln!("PROCESS started and mirrored {} variables in {:?}", remote.variable_count(), started.elapsed());
        let delta = Duration::from_millis(16);
        for tick in 0..3000 {
            inputs(&mut vars, tick);
            assert!(remote.tick(delta, tick as f64 * 0.016, &mut vars).is_none());
        }
        assert_eq!(vars.value("A32NX_ELEC_AC_1_BUS_IS_POWERED"), 1.);
        eprintln!("PROCESS {}", remote.take_report().unwrap());
    }
}
