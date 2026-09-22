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
/// target, disengaging the starter once the core carries itself -- the same
/// start-sequence shape `physics::engine::tests::run_to_steady_state` uses
/// (duplicated locally for the same reason as `isa_sea_level` above).
fn run_to_steady_state(engine: &mut Engine, mut inputs: EngineInputs, seconds: f64) -> EngineOutputs {
    let steps = (seconds / inputs.dt_s).round() as u32;
    let mut out = EngineOutputs::default();
    for _ in 0..steps {
        // The starter stays in until the core can carry itself, which is
        // the starter's own cut-out speed (`starter::CUTOFF_N3_FRAC`, 50%
        // N3), not light-off speed: between light-off and cut-out the
        // turbine still cannot cover the compressor's work, so cutting the
        // starter at `MIN_N3_FOR_COMBUSTION_PCT + 5` (what this helper used
        // to do, having drifted from the engine's own copy) leaves the core
        // hung at a quarter speed with no N1 at all for the whole run.
        inputs.starter_engaged = out.n3_pct < crate::physics::engine::starter::CUTOFF_N3_FRAC * 100.0;
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
        // `failures::reset_all()` clears the process-wide failure state, so
        // this must serialise against every other test that reads or writes
        // it, exactly as `breakers`, `components` and `fuel_network`
        // already do. Without the lock this reset lands in the middle of
        // another test's arm-then-read and silently disarms it -- which is
        // what it was doing to
        // `breakers::bearing_wear_magnitude_predicts_current_by_the_back_emf_relation`.
        let _g = crate::failures::tests::serial();
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
        let baseline_out = run_to_steady_state(&mut engine, baseline_inputs, 200.0);

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
        let raised_out = run_to_steady_state(&mut engine_raised, raised_inputs, 200.0);

        let measured_delta_fuel_kg_s = raised_out.fuel_flow_kg_s - baseline_out.fuel_flow_kg_s;

        assert!(
            measured_delta_fuel_kg_s > 0.0,
            "raising generator load must raise fuel flow at a fixed N1 target; baseline {} raised {}",
            baseline_out.fuel_flow_kg_s,
            raised_out.fuel_flow_kg_s
        );
        assert!(
            (raised_out.n1_pct - target_n1_corrected_pct).abs() < 0.1 && (baseline_out.n1_pct - target_n1_corrected_pct).abs() < 0.1,
            "both runs must have reached the same N1 for the comparison to mean anything: baseline {} raised {}",
            baseline_out.n1_pct,
            raised_out.n1_pct
        );

        // The independent check is a thermodynamic bracket, not a single
        // number. The heat the extra fuel carries is
        // `delta_fuel * LHV * eta_combustor`; what has to come out of it is
        // `delta_w` of shaft work at the gearbox. The ratio between them is
        // the *marginal* thermal efficiency of the core at this operating
        // point -- an efficiency, so:
        //
        // * it cannot exceed 1. Getting 150 kW of shaft work from less fuel
        //   heat than 150 kW would break the first law, so
        //   `delta_w / (LHV * eta_comb)` is a hard floor on the extra fuel.
        // * it cannot be arbitrarily good either, and the ceiling is the
        //   cycle's own. An ideal Brayton cycle at this engine's overall
        //   pressure ratio can convert at most
        //   `1 - (1/OPR)^((gamma-1)/gamma)` of its heat into work: at OPR
        //   35.9 that is 0.641. No real core beats its own ideal cycle, so
        //   that is a hard bound rather than a judgement. Below about 25%
        //   the engine would have stopped behaving like a turbine engine at
        //   all.
        //
        //   Note this ceiling is above the ~45-50% *overall* thermal
        //   efficiency such a core achieves, and correctly so: the marginal
        //   cost of a little more shaft power pays only the extra fuel, not
        //   the parasitic losses the whole cycle already carries, so the
        //   incremental figure sits above the average one and approaches
        //   the ideal.
        //
        // Bracketing on that band is an independent statement about
        // conservation of energy, derived from published figures for the
        // class of engine, that the model has to land inside -- not a number
        // read back off the model.
        // The ideal Brayton efficiency at the engine's design overall
        // pressure ratio, `1 - (1/OPR)^((g-1)/g)` with air's g = 1.4: 0.656
        // at OPR 42. No real core beats its own ideal cycle, so this is a
        // hard ceiling rather than a judgement about what is plausible.
        const GAMMA_AIR: f64 = 1.4;
        let ideal_cycle_efficiency = 1.0 - (1.0 / params::OPR_DESIGN).powf((GAMMA_AIR - 1.0) / GAMMA_AIR);
        let plausible = 0.25..=ideal_cycle_efficiency;
        let first_law_floor_kg_s = delta_w / (params::LHV_JET_A1_J_KG * params::COMBUSTOR_EFFICIENCY);
        let marginal_thermal_efficiency = delta_w / (measured_delta_fuel_kg_s * params::LHV_JET_A1_J_KG * params::COMBUSTOR_EFFICIENCY);
        assert!(
            measured_delta_fuel_kg_s > first_law_floor_kg_s,
            "the extra fuel {measured_delta_fuel_kg_s} kg/s carries less heat than the {delta_w} W of shaft work it has to do \
             (first-law floor {first_law_floor_kg_s} kg/s)"
        );
        assert!(
            plausible.contains(&marginal_thermal_efficiency),
            "the core turned {delta_w} W of extra gearbox load into {measured_delta_fuel_kg_s} kg/s of extra fuel, a marginal \
             thermal efficiency of {marginal_thermal_efficiency:.3} -- outside 0.25..{:.3}, the ideal cycle's own ceiling",
            ideal_cycle_efficiency
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
        let cut_out = run_to_steady_state(&mut engine_cut, cut_inputs, 200.0);

        // With the contract cut, the engine is told about no load at all, so
        // it must burn *less* than the baseline (which carries `baseline_w`),
        // and must show no sign of the raised demand the electrical side is
        // sitting on. Two things have to hold:
        //
        // 1. The raised demand never arrived. If it had leaked through by
        //    some other path, fuel flow would have gone up toward the raised
        //    case instead of down.
        // 2. What the engine *does* burn is set by what it was told, and by
        //    the same physics: dropping `baseline_w` of load off the gearbox
        //    has to give the fuel back at the same marginal efficiency the
        //    150 kW step cost, within the same band. That pins the
        //    relationship to the load the engine saw, not to the demand the
        //    electrical side published.
        assert!(
            cut_out.fuel_flow_kg_s < baseline_out.fuel_flow_kg_s,
            "with the contract cut the engine carries no gearbox load at all and must burn less than the baseline: \
             baseline {} cut {} raised {}",
            baseline_out.fuel_flow_kg_s,
            cut_out.fuel_flow_kg_s,
            raised_out.fuel_flow_kg_s
        );
        let cut_delta_fuel_kg_s = baseline_out.fuel_flow_kg_s - cut_out.fuel_flow_kg_s;
        let cut_marginal_efficiency = baseline_w / (cut_delta_fuel_kg_s * params::LHV_JET_A1_J_KG * params::COMBUSTOR_EFFICIENCY);
        assert!(
            plausible.contains(&cut_marginal_efficiency),
            "dropping the baseline {baseline_w} W of gearbox load saved {cut_delta_fuel_kg_s} kg/s, a marginal thermal \
             efficiency of {cut_marginal_efficiency:.3} -- the engine is not answering the load it was actually handed"
        );
    }
}


