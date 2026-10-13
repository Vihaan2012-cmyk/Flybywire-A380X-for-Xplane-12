//! The plugin's own engine and fuel chain, for the offline emulator.
//!
//! `Plugin::tick` (lib.rs) runs, before FlyByWire's systems tick: the
//! throttles, the FADEC, the PRIMs, the engine commands (the engine physics
//! and EECs) and the FCDCs; and after it (once hydraulics, electrical loads,
//! bays, breakers and bleed have run): the fuel system. Those modules only
//! reach X-Plane through datarefs that `Xplm::dummy()` answers "not found"
//! (reads zero, writes dropped), so they run offline unchanged. This module
//! holds exactly those pieces and calls them in `Plugin::tick`'s order, so
//! the emulator's engines start, burn fuel and drive the generators, bleed
//! and pumps through the same code the live aircraft runs.
//!
//! Left out, as in the emulator generally: `extra_backend_fbw` (sidestick,
//! reverser force onto X-Plane's velocity, sim rate), which needs X-Plane's
//! own velocity and time datarefs to mean anything.
//!
//! The fuel system's saved tank levels (`fbw_a380x_fuel.ini`) are neither
//! read nor written here: each case starts from FlyByWire's default load
//! and never touches a file another case (or X-Plane) shares.

use systems::simulation::{SimulatorReaderWriter, VariableRegistry};

use crate::xp::Xplm;
use crate::{afs_events, circuits::Circuits, engine_commands, extra_backend_fcdc, fadec, fuel, physics, prim, throttle, PrimRefs, Vars};

pub struct EngineChain {
    xplm: &'static Xplm,
    throttles: throttle::Throttles,
    fadec: fadec::Fadec,
    prims: prim::Prims,
    prim_refs: PrimRefs,
    engine_commands: engine_commands::EngineCommands,
    fcdc: extra_backend_fcdc::ExtraBackendFcdc,
    bays: physics::bays::Bays,
    fuel: Option<fuel::Fuel>,
    damage: physics::damage::Damage,
    tyres: physics::tyre::Tyres,
}

impl EngineChain {
    /// Built in `Plugin::new`'s order, after FlyByWire's simulation.
    pub fn new(vars: &mut Vars, xplm: &'static Xplm, start_state: f64) -> Self {
        let fadec = fadec::Fadec::new(vars, xplm);
        let throttles = throttle::Throttles::new(xplm);
        let engine_commands = engine_commands::EngineCommands::new(vars, xplm);
        let prims = prim::Prims::new(vars, start_state);
        let prim_refs = PrimRefs::new(xplm);
        let fuel = fuel::Fuel::new(vars, xplm).ok().map(|mut f| {
            f.set_persistence(false);
            f
        });
        let bays = physics::bays::Bays::new(vars);
        let fcdc = extra_backend_fcdc::ExtraBackendFcdc::new(xplm);
        let damage = physics::damage::Damage::new(vars, Some(xplm));
        let tyres = physics::tyre::Tyres::new(vars, Some(xplm));
        Self { xplm, throttles, fadec, prims, prim_refs, engine_commands, fcdc, bays, fuel, damage, tyres }
    }

    /// `Plugin::tick` from "The engines first" to the FCDCs. `time` is the
    /// simulation time before this tick's advance, as there.
    pub fn before_systems(&mut self, vars: &mut Vars, delta: f64, time: f64) {
        let xplm = self.xplm;
        let levers = self.throttles.update(xplm, delta);
        self.fadec.update(vars, xplm, &levers, delta, time);
        let readings = self.prim_refs.read(xplm);
        for event in self.prims.fcu_initialization(&readings, time) {
            afs_events::send(event);
        }
        let events = afs_events::take();
        let (priority_capt, priority_fo) = afs_events::priority_takeover_held();
        let id = vars.get("PRIORITY_TAKEOVER:1".to_owned());
        vars.write(&id, priority_capt as u8 as f64);
        let id = vars.get("PRIORITY_TAKEOVER:2".to_owned());
        vars.write(&id, priority_fo as u8 as f64);
        let prim_buses = self.prims.update(vars, &readings, &events, delta, time);
        let eec = self.engine_commands.update(vars, xplm, delta, time, &prim_buses, &events.throttles);
        self.prims.update_after_fadecs(vars, eec);
        self.fcdc.update(vars, xplm, &self.prims, &readings, delta);
    }

    /// An engines-running spawn: every engine's physics at its own settled
    /// ground idle (the caller sets the masters on and ignition to NORM, and
    /// FlyByWire's FADEC then sees a running engine and declares it On, as
    /// at an in-flight spawn).
    pub fn spawn_engines_at_idle(&mut self) {
        self.engine_commands.spawn_at_idle();
    }

    /// `Plugin::tick`'s bay thermal step (after electrical loads and circuit
    /// protection, before `breakers.post_systems`).
    pub fn bays(&mut self, vars: &mut Vars, delta: f64) {
        self.bays.update(vars, delta);
    }

    /// `Plugin::tick`'s damage (exceedances, engine life) and tyre steps,
    /// after bleed, before fuel.
    pub fn damage_and_tyres(&mut self, vars: &mut Vars, delta: f64) {
        self.damage.update(vars, Some(self.xplm), delta);
        self.tyres.update(vars, Some(self.xplm), delta);
        physics::damage::publish(self.damage.engines);
    }

    /// The engines' accumulated life (hours, cycles, creep, exceedances).
    pub fn engine_wear(&self) -> [physics::damage::EngineWear; 4] {
        self.damage.engines
    }

    /// `Plugin::tick`'s fuel step, after the systems and their loads.
    pub fn fuel(&mut self, vars: &mut Vars, delta: f64, circuits: &Circuits) {
        if let Some(fuel) = self.fuel.as_mut() {
            fuel.update(vars, self.xplm, delta, circuits);
        }
    }
}
