//! X-Plane visible/physical failure effects (hyperrealism.md workstream:
//! "X-Plane visible failure effects"). This module never invents a cause: it
//! reads state other workstreams already derive causally, and turns it into
//! X-Plane's own native failure/effect datarefs so a real fire, leak or
//! structural loss is seen and felt in X-Plane's own flight model and
//! particle effects, not just toggled as an internal flag.
//!
//! ## Nacelle/APU/MLG fire (`docs/physics/fire.md`)
//!
//! FlyByWire's own `SetOnFireModule` (`a380_systems::fire_and_smoke_
//! protection`) is the sole cause of `ENG_{1..4}_ON_FIRE`/`APU_ON_FIRE`/
//! `MLG_ON_FIRE`: it is driven by the `26_00x` `SetOnFire` failure and
//! cleared only once `zone_extinguishing_determination` (fixed by
//! `patches/fbw-rust/fire.patch`, see `docs/physics/fire.md`) says the
//! bottle discharge actually put it out. This module reads only that
//! already-causal state and mirrors it onto X-Plane's own
//! `sim/operation/failures/rel_engfir{0..3}` (X-Plane's SDK convention:
//! `0` = working, `1` = failed/active, matching `failures.rs::extra::drive`'s
//! existing `NativeXplane` convention) -- so the fire stops the same tick
//! the plugin-side state says it is out, never independently.
//!
//! ### Shared "fire intensity" interface
//!
//! The brief asks for a Vars state carrying "fire burning in nacelle n
//! (intensity)" for other systems (ECAM/FWS heat effects, structural
//! damage) to read. `XP_ENGINE_FIRE_INTENSITY:n` (n = 1..=4, Vars float,
//! 0.0..1.0) is that interface, defined here since no other workstream had
//! added it yet at the time of writing (grepped for `FIRE_INTENSITY`/
//! `NACELLE_FIRE`/`fire_intensity`, nothing found). It is sourced from
//! X-Plane's own `sim/flightmodel2/engines/is_on_fire` (float[16], 0..1,
//! read-only): X-Plane's core flight model already ramps this continuously
//! from the `rel_engfir` failure_enum this module sets, so the intensity
//! comes from X-Plane's own physics rather than this plugin fabricating a
//! ramp rate with no cited time constant (the project rule against faked
//! numbers). If a fire-workstream heat model lands later with a different
//! name, retarget this `Var` to read it instead -- the name is the contract,
//! not this specific source.
//!
//! ## Cockpit smoke from an uncontained aft/gear-bay fire
//!
//! X-Plane's SDK has no APU-bay or MLG-bay fire *visual* dataref (only
//! `rel_engfir*` for the engine nacelles) -- confirmed by grepping
//! `DataRefs.txt` for `fire`/`smoke`/`nacelle`. The nearest real physical
//! consequence the SDK exposes is `sim/operation/failures/rel_smoke_cpit`
//! ("Smoke in cockpit"): a real uncontained fire aft of the pressure bulkhead
//! or in the MLG bay can vent combustion products into the ECS ducting that
//! also feeds the cabin/cockpit, so this module raises cockpit smoke while
//! `APU_ON_FIRE` or `MLG_ON_FIRE` is active and clears it once both are out,
//! again only ever following the plugin's own causal fire state.
//!
//! ## Tyre burst / brake wear-out
//!
//! Already wired by `failures.rs::extra::gear` + `physics/damage.rs`'s
//! brake-energy model (`arm(32_10x, ...)` -> `extra::drive` ->
//! `sim/operation/failures/rel_tireN`/`rel_lbrakes`/`rel_rbrakes`) before
//! this workstream started; verified, not duplicated here.
//!
//! ## Hydraulic reservoir leak
//!
//! FlyByWire's own ported hydraulic model already consumes
//! `FailureType::ReservoirLeak(Green|Yellow)` (`26_000`/`29_000`/`29_001` in
//! `failures.rs::a380_failures`) and drains `A32NX_HYD_{GREEN,YELLOW}_
//! RESERVOIR_LEVEL` for real (`study/hyd.rs:111` reads the same variable for
//! its own leak-plumbing diagram) -- the leak's physical cause and effect on
//! quantity is already causal, not reimplemented here. This module only
//! mirrors "that failure is active" onto X-Plane's own
//! `sim/operation/failures/rel_hydleak`/`rel_hydleak2` (green/yellow), the
//! closest visible/physical consequence the SDK exposes for a hydraulic
//! leak (pressure/quantity effects on that circuit), edge-triggered off the
//! failure catalogue so a repair (MEL clear) turns it off again.

//! ## Engine seizure / flameout
//!
//! Both read only real outputs `fadec.rs`'s engine model already publishes
//! -- `ENGINE_N3:n` (HP spool, `fadec.rs:699`), `GENERAL ENG OIL PRESSURE:n`
//! (`engine_commands.rs`, the physical engine's oil pump off the same
//! spool) and `GENERAL ENG STARTER:n` (the master switch, already the
//! plugin-wide "should this engine be running" signal --
//! `aspects.rs`'s fire-pushbutton aspect and `engine_commands.rs` both treat
//! it the same way). No failure tag is read directly; the two X-Plane
//! effects are told apart by their different real physical signature:
//! - **Flameout** (`rel_engfai{n}`, "loss of power without smoke"): the
//!   master switch still commands the engine running, but the HP spool has
//!   fallen back to windmilling (below the self-sustaining-combustion floor
//!   `physics::engine::params::MIN_N3_FOR_COMBUSTION_PCT` the engine model
//!   itself uses to decide when the starter must re-engage) while oil
//!   pressure is still present -- fuel/air stopped burning, the core kept
//!   turning. Requires having actually been running the tick before (a cold
//!   start's `N3` starts at 0 too; that must not read as a flameout).
//! - **Seizure** (`rel_seize_{n}`): the master switch still commands the
//!   engine running, the spool has stopped rotating outright (below the
//!   windmill floor a seized shaft cannot exceed) *and* oil pressure has
//!   collapsed with it (the oil pump shares the same shaft) -- the
//!   mechanical signature a pure flameout does not have, since a
//!   windmilling-but-unlit core still turns the oil pump.

use crate::failures;
use crate::physics::engine::params::MIN_N3_FOR_COMBUSTION_PCT;
use crate::xp::{DataRef, Xplm};
use systems::simulation::VariableIdentifier;

/// `a380_failures()`: `29_000` = `ReservoirLeak(Green)`, `29_001` =
/// `ReservoirLeak(Yellow)` (`failures.rs:155-156`).
const HYD_LEAK_GREEN_ID: u64 = 29_000;
const HYD_LEAK_YELLOW_ID: u64 = 29_001;

/// Below this HP-spool speed a real core is windmilling on airflow alone,
/// not being turned by an oil pump under its own power -- the engine
/// model's own combustion floor (`physics::engine::params`), reused rather
/// than a second, independently-chosen number.
const WINDMILL_FLOOR_PCT: f64 = MIN_N3_FOR_COMBUSTION_PCT;
/// Oil pressure below this (psi) with the master still commanding run is
/// read as "the pump has stopped", not "pressure is merely low" -- a
/// generic low-psi floor (no cited Trent 900 minimum oil pressure found),
/// documented as generic like `damage.rs`'s other derived thresholds.
const SEIZED_OIL_PSI: f64 = 5.0;

pub struct XpEffects {
    eng_on_fire: [VariableIdentifier; 4],
    apu_on_fire: VariableIdentifier,
    mlg_on_fire: VariableIdentifier,
    fire_intensity_out: [VariableIdentifier; 4],
    n3: [VariableIdentifier; 4],
    oil_psi: [VariableIdentifier; 4],
    master: [VariableIdentifier; 4],

    rel_engfir: [Option<DataRef>; 4],
    is_on_fire: Option<DataRef>,
    rel_smoke_cpit: Option<DataRef>,
    rel_hydleak_green: Option<DataRef>,
    rel_hydleak_yellow: Option<DataRef>,
    rel_engfai: [Option<DataRef>; 4],
    rel_seize: [Option<DataRef>; 4],

    was_apu_or_mlg_fire: bool,
    was_hyd_leak: [bool; 2],
    /// Whether engine n was above the windmill floor last tick, so a cold
    /// start (N3 starts at 0) is never mistaken for a flameout/seizure.
    was_running: [bool; 4],
    was_flameout: [bool; 4],
    was_seized: [bool; 4],
}

impl XpEffects {
    pub fn new<W: systems::simulation::VariableRegistry>(vars: &mut W, xplm: Option<&Xplm>) -> Self {
        Self {
            eng_on_fire: std::array::from_fn(|i| vars.get(format!("ENG_{}_ON_FIRE", i + 1))),
            apu_on_fire: vars.get("APU_ON_FIRE".to_owned()),
            mlg_on_fire: vars.get("MLG_ON_FIRE".to_owned()),
            fire_intensity_out: std::array::from_fn(|i| vars.get(format!("XP_ENGINE_FIRE_INTENSITY:{}", i + 1))),
            n3: std::array::from_fn(|i| vars.get(format!("ENGINE_N3:{}", i + 1))),
            oil_psi: std::array::from_fn(|i| vars.get(format!("GENERAL ENG OIL PRESSURE:{}", i + 1))),
            master: std::array::from_fn(|i| vars.get(format!("GENERAL ENG STARTER:{}", i + 1))),
            rel_engfir: std::array::from_fn(|i| xplm.and_then(|x| x.find(&format!("sim/operation/failures/rel_engfir{i}")))),
            is_on_fire: xplm.and_then(|x| x.find("sim/flightmodel2/engines/is_on_fire")),
            rel_smoke_cpit: xplm.and_then(|x| x.find("sim/operation/failures/rel_smoke_cpit")),
            rel_hydleak_green: xplm.and_then(|x| x.find("sim/operation/failures/rel_hydleak")),
            rel_hydleak_yellow: xplm.and_then(|x| x.find("sim/operation/failures/rel_hydleak2")),
            rel_engfai: std::array::from_fn(|i| xplm.and_then(|x| x.find(&format!("sim/operation/failures/rel_engfai{i}")))),
            rel_seize: std::array::from_fn(|i| xplm.and_then(|x| x.find(&format!("sim/operation/failures/rel_seize_{i}")))),
            was_apu_or_mlg_fire: false,
            was_hyd_leak: [false; 2],
            was_running: [false; 4],
            was_flameout: [false; 4],
            was_seized: [false; 4],
        }
    }

    /// Pure classification of one engine's real state into (is_flameout,
    /// is_seized), so it is unit-testable without X-Plane or the engine
    /// model. `was_running` gates both: a cold engine's `n3`/`oil_psi`
    /// start at 0 too, and must never itself read as a failure.
    fn classify_engine(master_on: bool, n3_pct: f64, oil_psi: f64, was_running: bool) -> (bool, bool) {
        if !master_on || !was_running {
            return (false, false);
        }
        let core_stopped = n3_pct < 1.0; // below any real windmill speed
        let oil_pump_stopped = oil_psi < SEIZED_OIL_PSI;
        if core_stopped && oil_pump_stopped {
            (false, true) // seizure: shaft and its oil pump stopped together
        } else if n3_pct < WINDMILL_FLOOR_PCT && !oil_pump_stopped {
            (true, false) // flameout: core still turning (windmilling), oil pump still turning with it, but not burning fuel
        } else {
            (false, false)
        }
    }

    /// Whether cockpit smoke should be commanded on, given the previous tick's
    /// state and this tick's APU/MLG fire booleans -- an edge-triggered
    /// decision (only write on change) so a manual override (a pilot's own
    /// smoke-clearing action) is not fought every tick. Pure so it is
    /// testable without X-Plane.
    fn smoke_edge(was_fire: bool, is_fire: bool) -> Option<bool> {
        if is_fire != was_fire {
            Some(is_fire)
        } else {
            None
        }
    }

    pub fn update<W: systems::simulation::SimulatorReaderWriter>(&mut self, vars: &mut W, xplm: Option<&Xplm>) {
        let mut fire_buf = [0f32; 4];
        if let (Some(xplm), Some(d)) = (xplm, self.is_on_fire) {
            xplm.get_vf(d, &mut fire_buf);
        }

        for n in 0..4 {
            let on_fire = vars.read(&self.eng_on_fire[n]) != 0.0;
            if let (Some(xplm), Some(d)) = (xplm, self.rel_engfir[n]) {
                xplm.set_i(d, on_fire as i32);
            }
            // X-Plane's own continuous ramp is the shared intensity
            // interface; without a live X-Plane host (unit tests), fall
            // back to the boolean cause so the interface is still exercised.
            let intensity = if xplm.is_some() { fire_buf[n] as f64 } else if on_fire { 1.0 } else { 0.0 };
            vars.write(&self.fire_intensity_out[n], intensity);
        }

        let apu_fire = vars.read(&self.apu_on_fire) != 0.0;
        let mlg_fire = vars.read(&self.mlg_on_fire) != 0.0;
        let is_fire = apu_fire || mlg_fire;
        if let Some(want_on) = Self::smoke_edge(self.was_apu_or_mlg_fire, is_fire) {
            if let (Some(xplm), Some(d)) = (xplm, self.rel_smoke_cpit) {
                xplm.set_i(d, want_on as i32);
            }
        }
        self.was_apu_or_mlg_fire = is_fire;

        let active = failures::active_ids();
        let leaks = [(active.contains(&HYD_LEAK_GREEN_ID), self.rel_hydleak_green), (active.contains(&HYD_LEAK_YELLOW_ID), self.rel_hydleak_yellow)];
        for (i, (is_leaking, dataref)) in leaks.into_iter().enumerate() {
            if let Some(want_on) = Self::smoke_edge(self.was_hyd_leak[i], is_leaking) {
                if let (Some(xplm), Some(d)) = (xplm, dataref) {
                    xplm.set_i(d, want_on as i32);
                }
            }
            self.was_hyd_leak[i] = is_leaking;
        }

        for n in 0..4 {
            let master_on = vars.read(&self.master[n]) != 0.0;
            let n3_pct = vars.read(&self.n3[n]);
            let oil_psi = vars.read(&self.oil_psi[n]);
            let (flameout, seized) = Self::classify_engine(master_on, n3_pct, oil_psi, self.was_running[n]);

            if let Some(want_on) = Self::smoke_edge(self.was_flameout[n], flameout) {
                if let (Some(xplm), Some(d)) = (xplm, self.rel_engfai[n]) {
                    xplm.set_i(d, want_on as i32);
                }
            }
            if let Some(want_on) = Self::smoke_edge(self.was_seized[n], seized) {
                if let (Some(xplm), Some(d)) = (xplm, self.rel_seize[n]) {
                    xplm.set_i(d, want_on as i32);
                }
            }
            self.was_flameout[n] = flameout;
            self.was_seized[n] = seized;
            // Above the windmill floor counts as "was running" for next
            // tick's cold-start guard; once seized/flameout it stays
            // considered "was running" too (n3 may sit above 1% while
            // windmilling on airflow alone), so the effect keeps latching
            // until the master switch is cycled off and on (a real restart
            // attempt), matching how a real flight crew would clear it.
            self.was_running[n] = self.was_running[n] || n3_pct >= WINDMILL_FLOOR_PCT;
            if !master_on {
                self.was_running[n] = false;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::aspects::test_vars::TestVars;
    use systems::simulation::{SimulatorReaderWriter, VariableRegistry};

    #[test]
    fn a_cold_engine_with_zero_n3_is_never_a_flameout_or_seizure() {
        // was_running = false: the classic false-positive at plugin start.
        assert_eq!(XpEffects::classify_engine(true, 0.0, 0.0, false), (false, false));
    }

    #[test]
    fn a_pilot_commanded_shutdown_is_not_a_flameout() {
        // master off: N3/oil pressure decaying normally, not a failure.
        assert_eq!(XpEffects::classify_engine(false, 5.0, 2.0, true), (false, false));
    }

    #[test]
    fn core_still_turning_with_oil_pressure_but_below_combustion_floor_is_a_flameout() {
        assert_eq!(XpEffects::classify_engine(true, 10.0, 40.0, true), (true, false));
    }

    #[test]
    fn core_and_oil_pump_stopped_together_is_a_seizure_not_a_flameout() {
        assert_eq!(XpEffects::classify_engine(true, 0.0, 0.0, true), (false, true));
    }

    #[test]
    fn a_healthy_running_engine_is_neither() {
        assert_eq!(XpEffects::classify_engine(true, 95.0, 45.0, true), (false, false));
    }

    #[test]
    fn engine_fire_boolean_drives_the_intensity_interface_with_no_xplane_host() {
        // `failures::STATE` is process-wide and this test reaches it (directly, or
        // through model code such as `Breakers::pre_systems` / `Damage::arm`).
        let _serial = crate::failures::tests::serial();
        let mut vars = TestVars::default();
        let mut fx = XpEffects::new(&mut vars, None);
        vars.write(&fx.eng_on_fire[1], 1.0); // engine 2
        fx.update(&mut vars, None);
        // Read back through the same identifiers `fx` was constructed with
        // rather than `vars.value(name)`: `TestVars::get` prefixes any
        // space-free name with `A32NX_` (aspects.rs's registry), so a
        // string literal here would silently look up a different slot than
        // the one `fx` actually wrote.
        assert_eq!(vars.read(&fx.fire_intensity_out[1]), 1.0);
        assert_eq!(vars.read(&fx.fire_intensity_out[0]), 0.0);

        vars.write(&fx.eng_on_fire[1], 0.0); // extinguished
        fx.update(&mut vars, None);
        assert_eq!(vars.read(&fx.fire_intensity_out[1]), 0.0, "fire must stop being reported once extinguished");
    }

    #[test]
    fn smoke_edge_only_fires_on_a_genuine_transition() {
        assert_eq!(XpEffects::smoke_edge(false, false), None);
        assert_eq!(XpEffects::smoke_edge(false, true), Some(true));
        assert_eq!(XpEffects::smoke_edge(true, true), None);
        assert_eq!(XpEffects::smoke_edge(true, false), Some(false));
    }

    #[test]
    fn apu_or_mlg_fire_alone_requests_cockpit_smoke_on_then_off() {
        // `failures::STATE` is process-wide and this test reaches it (directly, or
        // through model code such as `Breakers::pre_systems` / `Damage::arm`).
        let _serial = crate::failures::tests::serial();
        let mut vars = TestVars::default();
        let mut fx = XpEffects::new(&mut vars, None);
        assert!(!fx.was_apu_or_mlg_fire);

        vars.write(&fx.apu_on_fire, 1.0);
        fx.update(&mut vars, None);
        assert!(fx.was_apu_or_mlg_fire);

        vars.write(&fx.apu_on_fire, 0.0);
        fx.update(&mut vars, None);
        assert!(!fx.was_apu_or_mlg_fire, "smoke source must clear once the fire is out");
    }

    #[test]
    fn a_green_reservoir_leak_failure_is_tracked_and_clears_on_repair() {
        let _guard = crate::failures::tests::serial();
        let _f = crate::failures::Failures::new();
        failures::replace([]);
        let mut vars = TestVars::default();
        let mut fx = XpEffects::new(&mut vars, None);
        assert!(!fx.was_hyd_leak[0]);

        failures::replace([HYD_LEAK_GREEN_ID]);
        fx.update(&mut vars, None);
        assert!(fx.was_hyd_leak[0]);
        assert!(!fx.was_hyd_leak[1], "yellow circuit unaffected by a green leak");

        failures::replace([]);
        fx.update(&mut vars, None);
        assert!(!fx.was_hyd_leak[0], "clears once the leak failure is repaired");
        failures::replace([]);
    }
}
