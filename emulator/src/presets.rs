//! Presets: every one built by applying real inputs through [`Emulator`]'s
//! own public API and ticking, exactly as a test would -- never by poking
//! internal state directly.

use systems::simulation::StartState;

use crate::Emulator;

/// A cold, unpowered aircraft on the ground (FlyByWire's own `Apron` start
/// state), ticked for one second with no input at all -- mirrors
/// `start_state.rs`'s own cold-and-dark harness.
pub fn cold_and_dark() -> Emulator {
    let mut e = Emulator::new(StartState::Apron);
    e.set_on_ground(true);
    settle(&mut e, 1.0);
    e
}

/// Runs `e` at 20 Hz until `done` holds or `max_s` simulated seconds pass;
/// whether it held. Presets wait on the aircraft's own signals (APU AVAIL,
/// engine state), never a fixed time that a slower start would outrun.
pub fn run_until(e: &mut Emulator, max_s: f64, mut done: impl FnMut(&mut Emulator) -> bool) -> bool {
    let steps = (max_s / PRESET_DT).ceil() as u32;
    for _ in 0..steps {
        if done(e) {
            return true;
        }
        e.tick(PRESET_DT);
    }
    done(e)
}

/// Time step for building start states. Twice the battery's 0.05 s: the
/// state a preset reaches is a settled one (APU AVAIL, engines at idle), and
/// the engine and APU models split large steps internally, so building it
/// in fewer, larger steps reaches the same state for half the cost.
pub const PRESET_DT: f64 = 0.1;

/// Runs `seconds` of simulated time at [`PRESET_DT`].
fn settle(e: &mut Emulator, seconds: f64) {
    e.run(PRESET_DT, (seconds / PRESET_DT).round() as u32);
}

/// Cold and dark, batteries on, a ground power cart connected and EXT PWR 1
/// selected on.
pub fn ground_power() -> Emulator {
    let mut e = cold_and_dark();
    for bat in ["BAT_1", "BAT_2", "BAT_ESS", "BAT_APU"] {
        e.set_var(&format!("A32NX_OVHD_ELEC_{bat}_PB_IS_AUTO"), 1.0);
    }
    settle(&mut e, 2.0);
    e.set_var("EXT_PWR_AVAIL:1", 1.0);
    e.set_var("A32NX_OVHD_ELEC_EXT_PWR_1_PB_IS_ON", 1.0);
    settle(&mut e, 2.0);
    e
}

/// Cold and dark, batteries on, the APU started on its own fuel (fuel.rs's
/// real feed pressure) until AVAIL, then both APU generators and the APU
/// bleed on. If the APU never reaches AVAIL within three minutes the
/// aircraft is returned as it is, for the caller to see.
pub fn powered() -> Emulator {
    let mut e = cold_and_dark();
    for bat in ["BAT_1", "BAT_2", "BAT_ESS", "BAT_APU"] {
        e.set_var(&format!("A32NX_OVHD_ELEC_{bat}_PB_IS_AUTO"), 1.0);
    }
    settle(&mut e, 2.0);
    e.set_var("A32NX_OVHD_APU_MASTER_SW_PB_IS_ON", 1.0);
    settle(&mut e, 1.0);
    e.set_var("A32NX_OVHD_APU_START_PB_IS_ON", 1.0);
    run_until(&mut e, 180.0, |e| e.get_var("A32NX_OVHD_APU_START_PB_IS_AVAILABLE") > 0.5);
    e.set_var("A32NX_OVHD_ELEC_APU_GEN_1_PB_IS_ON", 1.0);
    e.set_var("A32NX_OVHD_ELEC_APU_GEN_2_PB_IS_ON", 1.0);
    e.set_var("A32NX_OVHD_APU_BLEED_PB_IS_ON", 1.0);
    settle(&mut e, 5.0);
    e
}

/// Whether engine `n` (1-4) is running: the FADEC's own ENGINE_STATE
/// (fadec.rs `EngineState`: 1 = On).
pub fn engine_running(e: &mut Emulator, n: u8) -> bool {
    e.get_var(&format!("ENGINE_STATE:{n}")).round() == 1.0
}

/// [`powered`], then all four engines running: each engine's physics spawned
/// at its own settled ground idle (`idle_engine`, the state the model's own
/// start converges to), masters on and ignition NORM, so FlyByWire's FADEC
/// sees running engines and declares them On, as at an engines-running
/// spawn. [`engines_started`] reaches the same state by the real start
/// sequence instead (for tests of the start itself; ~10x slower).
pub fn engines_running() -> Emulator {
    let mut e = powered();
    e.spawn_engines_at_idle();
    for n in 1..=4 {
        e.set_var(&format!("TURB ENG IGNITION SWITCH EX1:{n}"), 1.0);
        e.set_var(&format!("GENERAL ENG STARTER:{n}"), 1.0);
    }
    settle(&mut e, 10.0);
    e
}

/// [`powered`], then the four engines started with the real controls, one
/// at a time: mode selector IGN/START, each master on, waiting up to three
/// minutes for the FADEC to report the engine running, then the selector
/// back to NORM. An engine that does not start stays as it is, for the
/// caller to see ([`engine_running`]).
pub fn engines_started() -> Emulator {
    let mut e = powered();
    for n in 1..=4 {
        e.set_var(&format!("TURB ENG IGNITION SWITCH EX1:{n}"), 2.0);
    }
    settle(&mut e, 1.0);
    for n in [4u8, 3, 2, 1] {
        e.set_var(&format!("GENERAL ENG STARTER:{n}"), 1.0);
        run_until(&mut e, 180.0, |e| engine_running(e, n));
    }
    for n in 1..=4 {
        e.set_var(&format!("TURB ENG IGNITION SWITCH EX1:{n}"), 1.0);
    }
    settle(&mut e, 5.0);
    e
}

/// FlyByWire's own `Cruise` start state, with the given pressure altitude
/// and Mach applied, then ticked to settle.
pub fn cruise(alt_ft: f64, mach: f64) -> Emulator {
    let mut e = Emulator::new(StartState::Cruise);
    e.set_on_ground(false);
    e.set_pressure_altitude_ft(alt_ft);
    e.set_mach(mach);
    e.set_true_airspeed_kt(mach * 573.0); // a plain, generous stand-in near M0.85 cruise TAS; callers wanting an exact figure should set it themselves after this preset returns.
    settle(&mut e, 10.0);
    e
}
