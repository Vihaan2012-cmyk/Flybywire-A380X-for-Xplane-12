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
