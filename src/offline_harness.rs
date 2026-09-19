//! Offline cross-system harness: runs this plugin's own physics modules
//! (`fuel.rs`/`fuel_network.rs`, `physics::engine`, `physics::electrical`,
//! `physics::hydraulics`, `physics::air`, `breakers.rs`/`circuits.rs`)
//! against each other, in `Plugin::tick`'s own order (`lib.rs` `fn tick`),
//! with no live X-Plane process.
//!
//! ## Why this exists
//!
//! `start_state.rs`'s own doc comment on its cold-and-dark harness says it
//! plainly: that harness exercises FlyByWire's ported `a380_systems` plus
//! this crate's `aspects.rs` bridge, but *not* "any module that only runs
//! against the concrete XPLM-backed `Vars`/`Xplm` (fuel.rs, physics::air,
//! physics::hydraulics, physics::adirs's own glue) -- those have no offline
//! construction path without a live X-Plane process". That was true only
//! because nothing had built the construction path -- not because one is
//! structurally impossible. The seam already exists, unused:
//!
//! ## The seam
//!
//! * `Xplm::dummy()` (`src/xp.rs`, `#[cfg(test)]`) is a do-nothing binding:
//!   `find` always answers "no such dataref", every `get_*` returns a
//!   zeroed value, every `set_*`/`register_*` is a no-op. It already exists
//!   for exactly this reason (see its own doc comment) but until now was
//!   only used to build `Vars` in isolation, never to run the physics
//!   modules that take a `&Xplm` alongside it.
//! * `Vars` (`src/lib.rs`) is `Vars::new(xplm: &'static Xplm)` -- a private
//!   fn, but *crate*-private (it is defined at the crate root, so every
//!   descendant module, including this one, can call it), and it implements
//!   FlyByWire's own `VariableRegistry`/`SimulatorReaderWriter` traits, so
//!   it is usable both by modules that take the concrete `Vars` directly
//!   (`physics::electrical::EngineLoads::update(&self, vars: &mut Vars)`,
//!   `physics::hydraulics::Hydraulics::update`, `physics::air::
//!   EngineBleedLoads::update`) and by modules already generic over
//!   `V: VariableRegistry + SimulatorReaderWriter` (`breakers.rs`,
//!   `circuits.rs`, `CircuitProtection`) -- those last three already run
//!   offline against FlyByWire's own `TestVars` (see `aspects::test_vars`)
//!   with zero changes needed.
//! * `physics::engine::Engine::step` takes a plain `EngineInputs` struct
//!   with no `Vars`/`Xplm` at all -- `gearbox_elec_load_w`/
//!   `gearbox_hyd_load_w`/`bleed_extraction_kg_s` are already just `f64`
//!   fields on it. The only reason the whole-plugin `tick` looks
//!   X-Plane-bound is that `engine_commands.rs` reads those fields off the
//!   `ENGINE_GEARBOX_*_LOAD_W:n`/`ENGINE_BLEED_EXTRACTION_KG_S:n` contract
//!   variables in `Vars` before calling `step` -- and `Vars` itself needs
//!   nothing but a dummy `Xplm` to exist.
//!
//! No production code changes were needed to open this up; this module is
//! the first thing to actually exercise the combination. The one seam this
//! module *adds* is [`offline_vars`], a small helper that leaks a
//! `Xplm::dummy()` to `'static` (test-only leak; nothing outside `#[cfg(test)]`
//! ever calls it) so `Vars::new` can be built the same way the real
//! `Plugin::new` builds it, just off a dummy binding instead of
//! `Xplm::load()`.
//!
//! ## Adding a scenario
//!
//! Call [`offline_vars`] for a fresh `(&'static Xplm, Vars)` pair, call
//! `scenarios::reset_global_state()` and `failures::reset_all()` first (both
//! clear process-global state left behind by other tests in the same binary
//! -- see `scenarios`'s own module doc), then construct only the subsystems
//! your scenario needs and drive them in the same relative order
//! `Plugin::tick` uses: engines before electrical/hydraulics/bleed
//! (`self.fadec`/`self.engine_commands` in `lib.rs` `tick` run before
//! `self.simulation.tick`), then the systems tick (or a stand-in for it, if
//! your scenario does not need FlyByWire's C++/Rust computers), then
//! `self.hydraulics.update` / `self.electrical_loads.update` /
//! `self.breakers.post_systems` / `self.bleed_loads.update` in that order
//! (`lib.rs` `fn tick`, the block after `self.simulation.tick`).

use systems::simulation::{SimulatorReaderWriter, VariableRegistry};

use crate::physics::electrical::EngineLoads;
use crate::physics::engine::{params, Engine, EngineInputs, EngineOutputs};
use crate::xp::Xplm;
use crate::Vars;

/// A fresh, offline `Vars` bound to a dummy (do-nothing) `Xplm`: no live
/// X-Plane process is touched. The `Xplm::dummy()` is leaked to `'static`
/// (test-only; this function is only reachable from `#[cfg(test)]` code)
/// because `Vars::new` takes `&'static Xplm`, the same signature
/// `Plugin::new` uses with the real `Xplm::load()`.
fn offline_vars() -> Vars {
    let xplm: &'static Xplm = Box::leak(Box::new(Xplm::dummy()));
    Vars::new(xplm)
}

/// `isa_sea_level`-equivalent baseline inputs for `Engine::step`, mirroring
/// `physics::engine::tests::isa_sea_level` (that helper is private to its
/// own module, so this is a local copy, not a dependency on it).
fn isa_sea_level() -> EngineInputs {
    EngineInputs {
        ambient_pressure_pa: 101_325.0,
        ambient_temp_k: 288.15,
        mach: 0.0,
        true_airspeed_m_s: 0.0,
        target_n1_corrected_pct: 0.0,
        fuel_valve_open: true,
        starter_engaged: false,
        starter_supply_fraction: 1.0,
        bleed_extraction_kg_s: 0.0,
        bleed_from_ip_port: false,
        gearbox_elec_load_w: 0.0,
        gearbox_hyd_load_w: 0.0,
        compressor_efficiency_loss_fraction: 0.0,
        compressor_flow_capacity_loss_fraction: 0.0,
        turbine_efficiency_loss_fraction: 0.0,
        bearing_friction_extra_fraction: 0.0,
        oil_pressure_fraction: 1.0,
        fuel_temp_k: 288.15,
        oil_faults: Default::default(),
        dt_s: 0.02,
    }
}

/// Runs an engine from rest to a steady state at a fixed corrected-N1
/// target, disengaging the starter once the core has lit off -- the same
/// start-sequence shape `physics::engine::tests::run_to_steady_state` uses
/// (duplicated locally for the same reason as `isa_sea_level` above).
fn run_to_steady_state(engine: &mut Engine, mut inputs: EngineInputs, seconds: f64) -> EngineOutputs {
    let steps = (seconds / inputs.dt_s).round() as u32;
    let mut out = EngineOutputs::default();
    for _ in 0..steps {
        inputs.starter_engaged = out.n3_pct < params::MIN_N3_FOR_COMBUSTION_PCT + 5.0;
        out = engine.step(&inputs);
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Cross-system emergence: engine 1's generator draws a known extra
    /// electrical load; that load should flow through the real contract
    /// chain -- `physics::electrical::EngineLoads::update` republishing
    /// `ELEC_ENG_GEN_1_SHAFT_POWER_DEMAND` onto `ENGINE_GEARBOX_ELEC_LOAD_W:1`
    /// (`physics/electrical.rs:101`), the same variable
    /// `engine_commands.rs:206` reads into `EngineInputs::gearbox_elec_load_w`
    /// -- and out the other side as *more engine fuel flow*, because
    /// `Engine::step` (`physics/engine/mod.rs:557-572`) subtracts the extra
    /// gearbox accessory torque straight from the HP turbine's torque
    /// balance, and the governor (closed-loop on corrected N1) must burn
    /// more fuel to hold the same N1 against that extra drag.
    ///
    /// Independent prediction (not read from the engine model's own
    /// internals): at a fixed spool speed the extra shaft power the
    /// accessory draws must come from extra combustion power, so
    /// `delta_fuel_flow_kg_s ~= delta_watts / (LHV_JET_A1_J_KG *
    /// COMBUSTOR_EFFICIENCY)` -- both constants (`physics/engine/
    /// params.rs:147,164`, LHV_JET_A1_J_KG = 43.1 MJ/kg, ASTM D1655 typical
    /// spec value; COMBUSTOR_EFFICIENCY = 0.999) are the engine model's
    /// documented, cited combustor constants, not this test's own
    /// derivation of the governor/spool-torque code path it is checking.
    /// This first-order balance ignores the engine's own second-order
    /// coupling (a lower net HP torque also perturbs N1/N2 tracking and
    /// mass flow slightly before the governor re-settles), so the
    /// tolerance below is generous (35%), but the direction and rough
    /// magnitude are asserted independently of the code under test.
    ///
    /// Decouple check: repeat the same electrical-load change but do *not*
    /// let it reach `ENGINE_GEARBOX_ELEC_LOAD_W:1` (skip `EngineLoads::
    /// update`, i.e. cut the contract) -- the predicted fuel-flow increase
    /// must vanish, proving the effect above came from the contract, not
    /// from some other path (e.g. the two `Engine::step` runs simply
    /// differing by simulation noise).
    #[test]
    fn raising_generator_1_electrical_load_raises_engine_1_fuel_flow_through_the_gearbox_contract() {
        crate::scenarios::reset_global_state();
        crate::failures::reset_all();

        let target_n1_corrected_pct = 85.0;
        let baseline_w = 50_000.0;
        let delta_w = 150_000.0; // a known, large step: e.g. an extra pack/galley load coming on line.

        // ---- Baseline: engine 1's generator at `baseline_w`. ----
        let mut vars = offline_vars();
        let engine_loads = EngineLoads::new(&mut vars);
        let gen_1_demand = vars.get("ELEC_ENG_GEN_1_SHAFT_POWER_DEMAND".to_owned());
        vars.write(&gen_1_demand, baseline_w);
        engine_loads.update(&mut vars); // the real contract: writes ENGINE_GEARBOX_ELEC_LOAD_W:1
        let gearbox_load_1 = vars.get("ENGINE_GEARBOX_ELEC_LOAD_W:1".to_owned());
        let baseline_gearbox_w = vars.read(&gearbox_load_1);
        assert_eq!(baseline_gearbox_w, baseline_w, "the contract must republish the demand unchanged");

        let mut engine = Engine::new();
        let baseline_inputs = EngineInputs {
            target_n1_corrected_pct,
            gearbox_elec_load_w: baseline_gearbox_w,
            ..isa_sea_level()
        };
        let baseline_out = run_to_steady_state(&mut engine, baseline_inputs, 60.0);

        // ---- Raised: the same generator now demands `baseline_w + delta_w`, through the real contract. ----
        let mut vars2 = offline_vars();
        let engine_loads_2 = EngineLoads::new(&mut vars2);
        let gen_1_demand_2 = vars2.get("ELEC_ENG_GEN_1_SHAFT_POWER_DEMAND".to_owned());
        vars2.write(&gen_1_demand_2, baseline_w + delta_w);
        engine_loads_2.update(&mut vars2);
        let gearbox_load_1_raised = vars2.get("ENGINE_GEARBOX_ELEC_LOAD_W:1".to_owned());
        let raised_gearbox_w = vars2.read(&gearbox_load_1_raised);
        assert_eq!(raised_gearbox_w, baseline_w + delta_w);

        let mut engine_raised = Engine::new();
        let raised_inputs = EngineInputs {
            target_n1_corrected_pct,
            gearbox_elec_load_w: raised_gearbox_w,
            ..isa_sea_level()
        };
        let raised_out = run_to_steady_state(&mut engine_raised, raised_inputs, 60.0);

        let measured_delta_fuel_kg_s = raised_out.fuel_flow_kg_s - baseline_out.fuel_flow_kg_s;
        let predicted_delta_fuel_kg_s = delta_w / (params::LHV_JET_A1_J_KG * params::COMBUSTOR_EFFICIENCY);

        assert!(
            measured_delta_fuel_kg_s > 0.0,
            "raising generator load must raise fuel flow at a fixed N1 target; baseline {} raised {}",
            baseline_out.fuel_flow_kg_s,
            raised_out.fuel_flow_kg_s
        );
        let relative_error = (measured_delta_fuel_kg_s - predicted_delta_fuel_kg_s).abs() / predicted_delta_fuel_kg_s;
        assert!(
            relative_error < 0.35,
            "measured delta fuel flow {measured_delta_fuel_kg_s} kg/s vs independent LHV-based prediction {predicted_delta_fuel_kg_s} kg/s (error {relative_error})"
        );

        // ---- Decouple: same demand change, contract cut (EngineLoads::update never called on vars3). ----
        let mut vars3 = offline_vars();
        let _engine_loads_3 = EngineLoads::new(&mut vars3); // constructed (registers the ids) but never `update`d
        let gen_1_demand_3 = vars3.get("ELEC_ENG_GEN_1_SHAFT_POWER_DEMAND".to_owned());
        vars3.write(&gen_1_demand_3, baseline_w + delta_w); // the electrical side still "sees" the raised load...
        let gearbox_load_1_cut = vars3.get("ENGINE_GEARBOX_ELEC_LOAD_W:1".to_owned());
        let cut_gearbox_w = vars3.read(&gearbox_load_1_cut); // ...but the contract variable was never republished.
        assert_eq!(cut_gearbox_w, 0.0, "an unwritten contract variable must read the shared-contract zero default");

        let mut engine_cut = Engine::new();
        let cut_inputs = EngineInputs {
            target_n1_corrected_pct,
            gearbox_elec_load_w: cut_gearbox_w,
            ..isa_sea_level()
        };
        let cut_out = run_to_steady_state(&mut engine_cut, cut_inputs, 60.0);

        let cut_delta_fuel_kg_s = (cut_out.fuel_flow_kg_s - baseline_out.fuel_flow_kg_s).abs();
        assert!(
            cut_delta_fuel_kg_s < measured_delta_fuel_kg_s * 0.05,
            "with the gearbox-load contract cut, fuel flow must not move with the (unpropagated) electrical load: \
             baseline {} cut {} (would-be connected delta {})",
            baseline_out.fuel_flow_kg_s,
            cut_out.fuel_flow_kg_s,
            measured_delta_fuel_kg_s
        );
    }
}
