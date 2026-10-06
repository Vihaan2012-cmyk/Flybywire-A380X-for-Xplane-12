//! The live landing gear: one owned [`GearSystem`], stepped every frame
//! from [`Truth`] and published under the variable names `registry.rs`
//! names in its ECAM triggers.
//!
//! What it owns is the whole A380 gear: five legs (nose, two wing, two
//! body), each with its own oleo-pneumatic strut and its own
//! retraction/door/lock chain; all twenty wheels -- the sixteen braked
//! wheels of the wing and body bogies, each with its own carbon brake
//! stack and its own antiskid channel, plus the two nose wheels and the
//! two unbraked body-bogie rear-axle wheels that still carry their leg's
//! share of the load; the three steerable positions (nosewheel and both
//! body-gear rear axles), each with its own shimmy damper; and the parking
//! brake accumulator.
//!
//! ## What drives it
//!
//! Hydraulic supply comes straight from `truth.hydraulic_pressure_pa`
//! (green drives the wing legs, yellow the nose and body legs -- the split
//! `GearSystemInputs` already documents), brake stack cooling from
//! `truth.environment.sat_c`, and ground contact from `truth.on_ground`.
//!
//! ## Wired from `Truth` (2026-09-20 pass)
//!
//! This area used to run entirely on one interim [`GearCommands`] struct
//! for the airframe's motion and the cockpit controls the gear obeys, none
//! of it ever actually set: `deep::live::Deep::tick` drives every area
//! purely through the `Area` trait (`tick`/`publish`), so nothing in
//! production ever called a concrete setter on this type, and it ran at
//! [`GearCommands::default`]'s resting values forever. Most of that is
//! real now, read straight out of `Truth`/`Truth::controls` in
//! `inputs_from` every tick:
//!
//! * **Aircraft mass, pitch attitude and groundspeed** --
//!   `Truth::aircraft_mass_kg`/`pitch_deg`/`groundspeed_m_s`.
//! * **Per-leg ground contact and touchdown sink speed** --
//!   `Truth::leg_on_ground`/`leg_touchdown_sink_speed_ms`, both new. This
//!   area previously had only `Truth::on_ground`, one aircraft-wide flag
//!   with **no sink speed at all** -- the single number a hard-landing
//!   model is most sensitive to. `on_ground` is still ANDed in (no leg can
//!   be on the ground while the aircraft is not), but the per-leg flag and
//!   the real sink speed now drive the strut law directly.
//! * **Gear lever, parking brake and brake pedals** --
//!   `Truth::controls.gear_lever_down`/`parking_brake_on`/`brake_pedal_pos`.
//!
//! `GearCommands` now carries only what is genuinely still not on `Truth`:
//! per-leg side load (no real per-leg lateral-load source exists anywhere
//! in this port) and the gravity-extension handle (no dataref found). Mass
//! in particular used to be the one field here that could not honestly
//! default to zero -- an aircraft always weighs something -- so
//! `GearCommands::default` used the published A380-800 operating empty
//! weight; that default now lives on `Truth::default` instead (same
//! figure, same citation), since mass is a `Truth` reading now, not an
//! interim command.
//!
//! ## Steering command (2026-09-20 dead-failure pass)
//!
//! `Truth::controls.steering_command_deg[0]` (nosewheel tiller/pedal
//! command) now drives `nose_steer_in` in place of the removed
//! `GearCommands::nose_steering_command_deg`, which nothing in production
//! ever set: with the nose actuator's own target permanently pinned at
//! 0 deg, it never had anywhere to slew to, so its own `actuator_leak`
//! fault (which only throttles the *rate of change*, `steering::
//! SteeringActuator::step`'s `diff = target - base_angle_deg`) had no
//! motion to throttle and could never move a published variable
//! (`deep::integration::failure_audit`'s sweep found exactly this, for all
//! three steerable positions). `steering_command_deg[1]`/`[2]` (body
//! left/right) are **not** read directly: a real A380's body-gear rear
//! axle is not independently pilot-commanded at all, it is mechanically/
//! electronically slaved to the nosewheel's own angle and groundspeed
//! (`steering::body_steering_angle_deg`, already real, already-tested
//! physics, unchanged by this pass). Because that schedule is driven by
//! `nose_steer_out.base_angle_deg` -- the nose actuator's own real tracked
//! angle, not its raw command -- wiring the nose component alone gives
//! both body actuators a genuine nonzero target too, which is sufficient
//! to make all three actuators' `actuator_leak` failures live. If
//! `steering_command_deg[1]`/`[2]` are ever meant to carry an independent,
//! authoritative BSCU-computed body angle distinct from this module's own
//! GENERIC schedule, that is a `Truth`-sourcing question for whoever wires
//! `plugin.rs`'s publisher, not a guess to make here.

use crate::deep::api::Registry;
use crate::deep::live::{Faults, Truth};

use super::brakes::{BrakeFaults, ParkingBrakeFaults};
use super::retraction::RetractionFaults;
use super::steering::SteeringFaults;
use super::strut::StrutFaults;
use super::{GearSystem, GearSystemFaults, GearSystemInputs, GearSystemOutputs, LegFaults, LegTouchdownInputs};

/// A380-800 operating empty weight, kg. Airbus's own published Aircraft
/// Characteristics figure is about 277 t for the -800; this is the
/// airframe with neither fuel nor payload. Mass now comes straight from
/// `Truth::aircraft_mass_kg` every tick (module doc); this constant is kept
/// public as the same citation `Truth::default`'s own copy of it uses,
/// rather than duplicated without attribution.
pub const OEW_KG: f64 = 277_000.0;

/// A380 hydraulic system nominal pressure, Pa (5000 psi, `docs/deep/
/// BRIEF.md`'s own aircraft summary). `GearSystemInputs` wants each
/// system's supply as a fraction of nominal, and `Truth` carries it in Pa.
const HYDRAULIC_NOMINAL_PA: f64 = 5000.0 * 6894.757;

/// Antiskid channel self-test threshold. A real antiskid computer posts a
/// channel fault from its own BITE rather than from a skid outcome
/// (`registry.rs`'s module doc says so explicitly, and notes that
/// `brakes.rs` only takes the fault as a raw input today). **GENERIC**: a
/// channel that has lost half its release authority no longer passes a
/// self-test, so half is where the BITE reports.
const ANTISKID_BITE_THRESHOLD: f64 = 0.5;

/// `E-IND-DESIGN.md` 320800048 STEER ALTN STEER SYS HOT: the ALTN
/// nosewheel-steering circuit's own thermal law, the same static-heat/
/// convective-loss structure `brakes.rs`'s own stack thermal model uses
/// (module doc there), scaled down from a brake stack to a compact
/// hydraulic control unit. **GENERIC** throughout (no public A380-specific
/// figure exists for this circuit): heat input is reasoned as proportional
/// to how far the nosewheel is held deflected (a real hydraulic steering
/// valve has to keep flowing to hold a surface against airload/friction,
/// not only while it is slewing), and the other two constants are sized
/// only so a sustained, realistic deflection can plausibly cross the
/// (also GENERIC) 100 C threshold below within a few minutes of simulated
/// time -- an order-of-magnitude choice for a small underfloor unit, not a
/// claimed precise figure.
const ALTN_STEER_HEAT_PER_DEG_W: f64 = 20.0;
const ALTN_STEER_CONVECTION_W_K: f64 = 8.0;
const ALTN_STEER_THERMAL_CAPACITY_J_K: f64 = 5_000.0;
/// `E-IND-DESIGN.md` 320800048's own threshold: "GENERIC hot threshold
/// 100 C (order-of-magnitude below `brakes.rs`'s own `FIRE_TEMP_C` = 800 C,
/// reasoned the same documented-but-generic way)".
pub const ALTN_STEER_SYS_HOT_C: f64 = 100.0;

/// Number of legs and braked wheels, in `LEGS`/`LEG_WHEEL_INDICES` order.
const N_LEGS: usize = 5;
const N_BRAKED_WHEELS: usize = 16;

/// `registry.rs`'s own leg keys, in the `GEAR_*:n` instance order its
/// module doc fixes (1 nose, 2 left wing, 3 right wing, 4 left body, 5
/// right body).
const LEG_KEY: [&str; N_LEGS] = ["nose", "l_wing", "r_wing", "l_body", "r_body"];
/// The three steerable positions, in `NW_STEER_*` / `BODY_STEER_*:n` order.
const STEER_KEY: [&str; 3] = ["nose", "l_body", "r_body"];

// ---------------------------------------------------------------------------
// Inputs that `Truth` does not carry yet.
// ---------------------------------------------------------------------------

/// What the gear needs that is genuinely still not in [`Truth`] anywhere
/// (see this module's own doc): per-leg side load and the gravity-extension
/// handle.
#[derive(Clone, Copy, Debug)]
pub struct GearCommands {
    /// Side load at each leg's axle, N. `Truth` carries no per-leg lateral
    /// load source (no real dataref for it was found anywhere in this
    /// port).
    pub leg_side_load_n: [f64; N_LEGS],
    /// The free-fall/gravity extension handle. No real dataref found.
    pub gravity_extend_commanded: bool,
}

impl Default for GearCommands {
    /// No side load, gravity extension not commanded -- the resting values
    /// for the two inputs `Truth` still does not carry.
    fn default() -> Self {
        Self { leg_side_load_n: [0.0; N_LEGS], gravity_extend_commanded: false }
    }
}

// ---------------------------------------------------------------------------
// Failure ids.
// ---------------------------------------------------------------------------

/// Every failure id `registry.rs` assigns, resolved once at construction
/// from the registry itself rather than restated here. See
/// `deep::fuel::live::Ids` for why this is a lookup and not a table.
struct Ids {
    strut_gas_leak: [u64; N_LEGS],
    strut_oil_leak: [u64; N_LEGS],
    actuator_leak: [u64; N_LEGS],
    uplock_jam: [u64; N_LEGS],
    downlock_fail: [u64; N_LEGS],
    door_jam: [u64; N_LEGS],
    sensor_lies: [u64; N_LEGS],
    antiskid_inop: [u64; N_BRAKED_WHEELS],
    dragging: [u64; N_BRAKED_WHEELS],
    parking_brake_leak: u64,
    shimmy: [u64; 3],
    steer_actuator_leak: [u64; 3],

    /// `E-IND-DESIGN.md` 320800043/046: each strut's own pressure-
    /// monitoring BITE and weight-on-wheels sensing.
    strut_gas_charge_sensor_fail: [u64; N_LEGS],
    strut_wow_sensing_fail: [u64; N_LEGS],
    /// `E-IND-DESIGN.md` 320800032: the two body legs' own bogie-trim BITE.
    /// `0` (never a real failure id, `Faults::get` returns 0.0 for it) on
    /// the nose/wing legs, which have no such mechanism.
    bogie_trim_fail: [u64; N_LEGS],
    /// `E-IND-DESIGN.md` 320800056/057/059: the nosewheel-only disconnect
    /// mechanism and angle-limit-override failures.
    steer_disc_mechanism_fail: u64,
    steer_overtravel_fail: u64,

    /// `E-IND-DESIGN.md`'s new Brake System Controller (BSC): the two
    /// control channels, the normal/alternate pressure-monitoring pair, the
    /// autobrake function and the selector valve, in `registry.rs`'s own
    /// `register_bscu` channel order.
    bscu_ctl: [u64; 2],
    bscu_norm_press_sensor_fail: u64,
    bscu_alt_press_sensor_fail: u64,
    bscu_autobrake_fail: u64,
    bscu_sel_valve_jam: u64,
    /// `E-IND-DESIGN.md` 320800024: the two brake pedal position
    /// transducers, `[left, right]`.
    brake_pedal_sensor_fail: [u64; 2],

    /// `E-IND-DESIGN.md`'s new Steering System Controller (SSC): the two
    /// control channels and the selector valve.
    steer_ctl: [u64; 2],
    steer_sel_valve_jam: u64,
    /// `E-IND-DESIGN.md` 320800051/052/061: the tiller and pedal-steering
    /// transducers.
    capt_tiller_fail: u64,
    fo_tiller_fail: u64,
    pedal_steer_fail: u64,
}

/// The failures registered against `component`, in registration order.
fn fids(reg: &Registry, component: &str) -> Vec<u64> {
    reg.failures.iter().filter(|f| f.component == component).map(|f| f.id).collect()
}

impl Ids {
    fn resolve() -> Self {
        let mut reg = Registry::default();
        super::registry::register(&mut reg);

        let mut strut_gas_leak = [0u64; N_LEGS];
        let mut strut_oil_leak = [0u64; N_LEGS];
        let mut strut_gas_charge_sensor_fail = [0u64; N_LEGS];
        let mut strut_wow_sensing_fail = [0u64; N_LEGS];
        let mut actuator_leak = [0u64; N_LEGS];
        let mut uplock_jam = [0u64; N_LEGS];
        let mut downlock_fail = [0u64; N_LEGS];
        let mut door_jam = [0u64; N_LEGS];
        let mut sensor_lies = [0u64; N_LEGS];
        let mut bogie_trim_fail = [0u64; N_LEGS];
        for (i, key) in LEG_KEY.iter().enumerate() {
            let strut = fids(&reg, &format!("32_gear.{key}_strut"));
            assert_eq!(strut.len(), 4, "each strut registers a gas leak, an oil leak, a pressure-sensor failure and a weight-on-wheels-sensing failure");
            strut_gas_leak[i] = strut[0];
            strut_oil_leak[i] = strut[1];
            strut_gas_charge_sensor_fail[i] = strut[2];
            strut_wow_sensing_fail[i] = strut[3];

            let retraction = fids(&reg, &format!("32_gear.{key}_retraction"));
            // The two body legs (`registry.rs`'s `register_retractions`)
            // additionally register a sixth failure, `bogie_trim_fail`; the
            // nose/wing legs have no such mechanism and stay at five.
            assert!(retraction.len() == 5 || retraction.len() == 6, "each retraction chain registers five failures, plus a sixth (bogie trim) on the two body legs; got {}", retraction.len());
            actuator_leak[i] = retraction[0];
            uplock_jam[i] = retraction[1];
            downlock_fail[i] = retraction[2];
            door_jam[i] = retraction[3];
            sensor_lies[i] = retraction[4];
            if retraction.len() == 6 {
                bogie_trim_fail[i] = retraction[5];
            }
        }

        let mut antiskid_inop = [0u64; N_BRAKED_WHEELS];
        let mut dragging = [0u64; N_BRAKED_WHEELS];
        for wheel in 0..N_BRAKED_WHEELS {
            let brake = fids(&reg, &format!("32_gear.wheel_{}_brake", wheel + 1));
            assert_eq!(brake.len(), 2, "each braked wheel registers an antiskid and a dragging failure");
            antiskid_inop[wheel] = brake[0];
            dragging[wheel] = brake[1];
        }

        let mut shimmy = [0u64; 3];
        let mut steer_actuator_leak = [0u64; 3];
        let mut steer_disc_mechanism_fail = 0u64;
        let mut steer_overtravel_fail = 0u64;
        for (i, key) in STEER_KEY.iter().enumerate() {
            let steer = fids(&reg, &format!("32_gear.{key}_steering"));
            // The nose position (`registry.rs`'s own `key == "nose"` guard
            // in `register_steering`) additionally registers the
            // disconnect-mechanism and angle-limit-override failures; the
            // two body positions stay at two.
            assert!(steer.len() == 2 || steer.len() == 4, "each steerable position registers a shimmy and an actuator failure, plus two more on the nose position only; got {}", steer.len());
            shimmy[i] = steer[0];
            steer_actuator_leak[i] = steer[1];
            if steer.len() == 4 {
                steer_disc_mechanism_fail = steer[2];
                steer_overtravel_fail = steer[3];
            }
        }

        let parking = fids(&reg, "32_gear.parking_brake_accumulator");
        assert_eq!(parking.len(), 1);

        let bscu = fids(&reg, "32_gear.bscu");
        assert_eq!(bscu.len(), 6, "the BSCU registers six channels: ctl_1, ctl_2, norm/alt pressure sensors, autobrake, selector valve");
        let bscu_ctl = [bscu[0], bscu[1]];
        let bscu_norm_press_sensor_fail = bscu[2];
        let bscu_alt_press_sensor_fail = bscu[3];
        let bscu_autobrake_fail = bscu[4];
        let bscu_sel_valve_jam = bscu[5];

        let pedal = fids(&reg, "32_gear.brake_pedal_transducers");
        assert_eq!(pedal.len(), 2, "left and right brake pedal transducers");
        let brake_pedal_sensor_fail = [pedal[0], pedal[1]];

        let steer_ctl_ids = fids(&reg, "32_gear.steer_ctl");
        assert_eq!(steer_ctl_ids.len(), 3, "the SSC registers two channels and a selector valve");
        let steer_ctl = [steer_ctl_ids[0], steer_ctl_ids[1]];
        let steer_sel_valve_jam = steer_ctl_ids[2];

        let steer_input = fids(&reg, "32_gear.steer_input_transducers");
        assert_eq!(steer_input.len(), 3, "captain tiller, F/O tiller, pedal steering");
        let capt_tiller_fail = steer_input[0];
        let fo_tiller_fail = steer_input[1];
        let pedal_steer_fail = steer_input[2];

        Self {
            strut_gas_leak,
            strut_oil_leak,
            actuator_leak,
            uplock_jam,
            downlock_fail,
            door_jam,
            sensor_lies,
            antiskid_inop,
            dragging,
            parking_brake_leak: parking[0],
            shimmy,
            steer_actuator_leak,
            strut_gas_charge_sensor_fail,
            strut_wow_sensing_fail,
            bogie_trim_fail,
            steer_disc_mechanism_fail,
            steer_overtravel_fail,
            bscu_ctl,
            bscu_norm_press_sensor_fail,
            bscu_alt_press_sensor_fail,
            bscu_autobrake_fail,
            bscu_sel_valve_jam,
            brake_pedal_sensor_fail,
            steer_ctl,
            steer_sel_valve_jam,
            capt_tiller_fail,
            fo_tiller_fail,
            pedal_steer_fail,
        }
    }
}

// ---------------------------------------------------------------------------
// The live system.
// ---------------------------------------------------------------------------

/// The live A380 landing gear.
pub struct GearStructureLive {
    ids: Ids,
    system: GearSystem,
    outputs: GearSystemOutputs,
    /// This frame's antiskid BITE report, one per braked wheel.
    antiskid_channel_fault: [bool; N_BRAKED_WHEELS],
    /// This tick's `Truth`-sourced gear lever and parking brake position,
    /// kept because `Area::publish` takes no `Truth` of its own to read
    /// them back from.
    gear_lever_down: bool,
    parking_brake_set: bool,
    /// Inputs `Truth` still does not carry anywhere; see [`GearCommands`].
    pub commands: GearCommands,

    /// `E-IND-DESIGN.md`'s new Brake System Controller (BSC) and Steering
    /// System Controller (SSC): pure BITE-flag pass-throughs of their own
    /// failure ids, with no physics of their own to step -- computed once
    /// per tick here (the same "cache the Truth-sourced reading so
    /// `publish` has something to read back" shape this file already uses
    /// for `gear_lever_down`/`parking_brake_set` above) rather than
    /// threaded through `GearSystem`, which has no use for them.
    bscu_ctl_fault: [bool; 2],
    norm_brk_press_sensor_fault: bool,
    altn_brk_press_sensor_fault: bool,
    auto_brk_fault: bool,
    brake_sel_vlv_jammed: bool,
    brake_pedal_sensor_fault: [bool; 2],
    steer_ctl_fault: [bool; 2],
    steer_sel_vlv_jammed: bool,
    capt_tiller_fault: bool,
    fo_tiller_fault: bool,
    pedal_steer_fault: bool,
    /// `E-IND-DESIGN.md` 320800049/050 STEER B/W STEER FAULT: the two body
    /// steering positions' own `actuator_leak` fault reaching the same
    /// GENERIC BITE threshold every other channel fault in this design
    /// uses, surfaced directly with no new failure id
    /// (`[left body, right body]`).
    body_steer_fault: [bool; 2],
    /// `E-IND-DESIGN.md` 320800048 STEER ALTN STEER SYS HOT: the ALTN
    /// nosewheel-steering circuit's own temperature, deg C. See `tick`'s
    /// own doc comment on this field for the thermal law.
    altn_steer_sys_temp_c: f64,
    /// The nosewheel's own commanded angle last tick, deg -- kept only so
    /// `tick` can compute this tick's slew rate for the thermal model
    /// above.
    prev_nw_commanded_angle_deg: f64,
    /// `E-IND-DESIGN.md` 320800025/026 BRAKES RELEASED / RESIDUAL BRAKING:
    /// the pilot's own commanded braking fraction, `max(left, right)` pedal
    /// -- a sound (if not exhaustive) subset of the design's own
    /// "pedal_or_autobrake_demand" (this port has no separate autobrake
    /// demand signal to add to it, the same "sound if not exhaustive"
    /// reasoning this file's own module doc already uses for LRU power
    /// loss standing in for "faulted").
    brake_pedal_commanded_fraction: f64,
    /// `E-IND-DESIGN.md` 320800042: the gravity-extension handle selection,
    /// cached the same way `gear_lever_down` is above so `publish` (which
    /// gets no `Truth` of its own) can read it back as
    /// `GRAVITY_EXTEND_SELECTED`.
    gravity_extend_selected: bool,
    /// `E-IND-DESIGN.md` 320800057/059: the towing/disconnect lever's own
    /// selection, cached and published back the same way
    /// `gravity_extend_selected` is above.
    nw_steer_disc_selected: bool,
    /// `E-IND-DESIGN.md` 320800019 BRAKES MINOR FAULT: true when any
    /// braked wheel's own `antiskid_inop` fault is armed but below
    /// `ANTISKID_BITE_THRESHOLD` -- a partial degradation noticed but not
    /// yet channel-failed. No new failure id; derived from the same
    /// magnitudes `antiskid_channel_fault` above already reads.
    brakes_minor_fault: bool,
}

impl Default for GearStructureLive {
    fn default() -> Self {
        Self::new()
    }
}

impl GearStructureLive {
    pub fn new() -> Self {
        let mut system = GearSystem::new();
        let commands = GearCommands::default();
        let truth = Truth::default();
        // Step once so `outputs` is this gear's real state rather than a
        // zeroed struct: a parked aircraft's struts are compressed under
        // its own weight and its legs are down and locked.
        let outputs = system.step(&inputs_from(&truth, &commands), &GearSystemFaults::default());
        Self {
            ids: Ids::resolve(),
            system,
            outputs,
            antiskid_channel_fault: [false; N_BRAKED_WHEELS],
            gear_lever_down: truth.controls.gear_lever_down,
            parking_brake_set: truth.controls.parking_brake_on,
            commands,
            bscu_ctl_fault: [false; 2],
            norm_brk_press_sensor_fault: false,
            altn_brk_press_sensor_fault: false,
            auto_brk_fault: false,
            brake_sel_vlv_jammed: false,
            brake_pedal_sensor_fault: [false; 2],
            steer_ctl_fault: [false; 2],
            steer_sel_vlv_jammed: false,
            capt_tiller_fault: false,
            fo_tiller_fault: false,
            pedal_steer_fault: false,
            body_steer_fault: [false; 2],
            altn_steer_sys_temp_c: truth.environment.sat_c,
            prev_nw_commanded_angle_deg: truth.controls.steering_command_deg[0],
            gravity_extend_selected: truth.controls.gravity_extend_selected,
            brake_pedal_commanded_fraction: truth.controls.brake_pedal_pos[0].max(truth.controls.brake_pedal_pos[1]),
            nw_steer_disc_selected: truth.controls.nw_steer_disc_selected,
            brakes_minor_fault: false,
        }
    }

    /// The full gear state this frame, for anything that needs more than
    /// the published variables (the hard-landing report, for instance).
    pub fn outputs(&self) -> &GearSystemOutputs {
        &self.outputs
    }

    pub fn system(&self) -> &GearSystem {
        &self.system
    }

    fn faults_from(&self, faults: &Faults) -> GearSystemFaults {
        let leg = |i: usize| LegFaults {
            strut: StrutFaults {
                gas_leak: faults.get(self.ids.strut_gas_leak[i]),
                oil_leak: faults.get(self.ids.strut_oil_leak[i]),
                gas_charge_sensor_fail: faults.get(self.ids.strut_gas_charge_sensor_fail[i]),
                wow_sensing_fail: faults.get(self.ids.strut_wow_sensing_fail[i]),
            },
            retraction: RetractionFaults {
                actuator_leak: faults.get(self.ids.actuator_leak[i]),
                uplock_jam: faults.get(self.ids.uplock_jam[i]),
                downlock_fail: faults.get(self.ids.downlock_fail[i]),
                door_jam: faults.get(self.ids.door_jam[i]),
                sensor_lies: faults.get(self.ids.sensor_lies[i]),
                // `0` on the nose/wing legs -- `Faults::get` reads that as
                // 0.0 (healthy), which is correct since they have no
                // bogie-trim mechanism to fail.
                bogie_trim_fail: faults.get(self.ids.bogie_trim_fail[i]),
            },
        };
        // `disc_mechanism_fail`/`steer_overtravel_fail` only exist on the
        // nose position (`i == 0`); `steer_disc_mechanism_fail`/
        // `steer_overtravel_fail` are `0` (healthy) whenever resolved for a
        // position that has none, so gating on `i == 0` here is belt and
        // braces against ever reading the nose's own fault onto a body
        // position by mistake.
        let steer = |i: usize| SteeringFaults {
            shimmy_damper_fail: faults.get(self.ids.shimmy[i]),
            actuator_leak: faults.get(self.ids.steer_actuator_leak[i]),
            disc_mechanism_fail: if i == 0 { faults.get(self.ids.steer_disc_mechanism_fail) } else { 0.0 },
            steer_overtravel_fail: if i == 0 { faults.get(self.ids.steer_overtravel_fail) } else { 0.0 },
        };
        let mut wheel_brakes = [BrakeFaults::default(); N_BRAKED_WHEELS];
        for (wheel, slot) in wheel_brakes.iter_mut().enumerate() {
            *slot = BrakeFaults { antiskid_inop: faults.get(self.ids.antiskid_inop[wheel]), dragging: faults.get(self.ids.dragging[wheel]) };
        }
        GearSystemFaults {
            nose: leg(0),
            left_wing: leg(1),
            right_wing: leg(2),
            left_body: leg(3),
            right_body: leg(4),
            nose_steering: steer(0),
            left_body_steering: steer(1),
            right_body_steering: steer(2),
            wheel_brakes,
            parking_brake: ParkingBrakeFaults { leak: faults.get(self.ids.parking_brake_leak) },
        }
    }
}

/// One frame's `GearSystemInputs` from what `Truth`/`Truth::controls` now
/// carry, plus the three fields in `commands` that still do not exist
/// anywhere in this port (see [`GearCommands`]).
fn inputs_from(truth: &Truth, commands: &GearCommands) -> GearSystemInputs {
    let leg = |i: usize| LegTouchdownInputs {
        // `Truth::on_ground` is the aircraft-wide weight-on-wheels flag:
        // no leg can be on the ground while the aircraft is not, so the
        // two are ANDed rather than the per-leg flag simply overriding it.
        // `leg_on_ground`/`leg_touchdown_sink_speed_ms` are both real now
        // (module doc) -- previously `commands.leg_on_ground` defaulted to
        // `[true; N_LEGS]` and `commands.leg_sink_speed_ms` to zero forever,
        // since nothing in production ever set either.
        on_ground: truth.on_ground && truth.leg_on_ground[i],
        sink_speed_ms: truth.leg_touchdown_sink_speed_ms[i],
        side_load_n: commands.leg_side_load_n[i],
    };
    let fraction = |pa: f64| (pa / HYDRAULIC_NOMINAL_PA).clamp(0.0, 1.0);
    GearSystemInputs {
        mass_kg: truth.aircraft_mass_kg,
        pitch_deg: truth.pitch_deg,
        groundspeed_ms: truth.groundspeed_m_s,
        ambient_c: truth.environment.sat_c,
        nose: leg(0),
        left_wing: leg(1),
        right_wing: leg(2),
        left_body: leg(3),
        right_body: leg(4),
        gear_lever_down: truth.controls.gear_lever_down,
        // `E-IND-DESIGN.md` 320800042 L/G GRVTY EXTN FAULT: `Truth::
        // controls.gravity_extend_selected` is the real crew-selection
        // signal this module doc previously named as missing ("no real
        // dataref found... never set from Truth"). `commands.
        // gravity_extend_commanded` is kept (module doc, `GearCommands`)
        // for anything that still constructs it directly (e.g. this file's
        // own tests); ORed in rather than replaced so neither source can
        // silently lose the other.
        gravity_extend_commanded: commands.gravity_extend_commanded || truth.controls.gravity_extend_selected,
        green_hydraulic_fraction: fraction(truth.hydraulic_pressure_pa[0]),
        yellow_hydraulic_fraction: fraction(truth.hydraulic_pressure_pa[1]),
        nose_steering_command_deg: truth.controls.steering_command_deg[0],
        brake_pedal_left: truth.controls.brake_pedal_pos[0],
        brake_pedal_right: truth.controls.brake_pedal_pos[1],
        parking_brake_set: truth.controls.parking_brake_on,
        // `E-IND-DESIGN.md` 320800057/059 STEER N/W STEER DISC FAULT / NOT
        // DISC: the towing/disconnect lever selection.
        nw_steer_disc_selected: truth.controls.nw_steer_disc_selected,
        dt_s: truth.dt_s,
    }
}

impl crate::deep::live::Area for GearStructureLive {
    fn name(&self) -> &'static str {
        "gear_structure"
    }

    fn tick(&mut self, truth: &Truth, faults: &Faults) {
        let inputs = inputs_from(truth, &self.commands);
        let gear_faults = self.faults_from(faults);
        self.outputs = self.system.step(&inputs, &gear_faults);
        self.gear_lever_down = truth.controls.gear_lever_down;
        self.parking_brake_set = truth.controls.parking_brake_on;
        self.gravity_extend_selected = truth.controls.gravity_extend_selected;
        self.brake_pedal_commanded_fraction = truth.controls.brake_pedal_pos[0].max(truth.controls.brake_pedal_pos[1]);
        self.nw_steer_disc_selected = truth.controls.nw_steer_disc_selected;
        // `E-IND-DESIGN.md` 320800019: a wheel's `antiskid_inop` armed but
        // still below the BITE self-test threshold -- noticed, not yet
        // channel-failed.
        self.brakes_minor_fault = (0..N_BRAKED_WHEELS).any(|wheel| {
            let m = gear_faults.wheel_brakes[wheel].antiskid_inop;
            m > 0.0 && m < ANTISKID_BITE_THRESHOLD
        });

        // The antiskid computer's own BITE: a channel that has lost this
        // much of its release authority fails its self-test and reports,
        // independently of whether a skid has happened yet.
        for wheel in 0..N_BRAKED_WHEELS {
            self.antiskid_channel_fault[wheel] = gear_faults.wheel_brakes[wheel].antiskid_inop >= ANTISKID_BITE_THRESHOLD;
        }

        // `E-IND-DESIGN.md`'s new Brake System Controller (BSC) and
        // Steering System Controller (SSC): pure BITE-flag pass-throughs,
        // no physics to step (this file's own struct doc explains why they
        // are read here rather than threaded through `GearSystem`).
        self.bscu_ctl_fault = [faults.get(self.ids.bscu_ctl[0]) >= ANTISKID_BITE_THRESHOLD, faults.get(self.ids.bscu_ctl[1]) >= ANTISKID_BITE_THRESHOLD];
        self.norm_brk_press_sensor_fault = faults.get(self.ids.bscu_norm_press_sensor_fail) >= ANTISKID_BITE_THRESHOLD;
        self.altn_brk_press_sensor_fault = faults.get(self.ids.bscu_alt_press_sensor_fail) >= ANTISKID_BITE_THRESHOLD;
        self.auto_brk_fault = faults.get(self.ids.bscu_autobrake_fail) >= ANTISKID_BITE_THRESHOLD;
        self.brake_sel_vlv_jammed = faults.get(self.ids.bscu_sel_valve_jam) >= ANTISKID_BITE_THRESHOLD;
        self.brake_pedal_sensor_fault =
            [faults.get(self.ids.brake_pedal_sensor_fail[0]) >= ANTISKID_BITE_THRESHOLD, faults.get(self.ids.brake_pedal_sensor_fail[1]) >= ANTISKID_BITE_THRESHOLD];
        self.steer_ctl_fault = [faults.get(self.ids.steer_ctl[0]) >= ANTISKID_BITE_THRESHOLD, faults.get(self.ids.steer_ctl[1]) >= ANTISKID_BITE_THRESHOLD];
        self.steer_sel_vlv_jammed = faults.get(self.ids.steer_sel_valve_jam) >= ANTISKID_BITE_THRESHOLD;
        self.capt_tiller_fault = faults.get(self.ids.capt_tiller_fail) >= ANTISKID_BITE_THRESHOLD;
        self.fo_tiller_fault = faults.get(self.ids.fo_tiller_fail) >= ANTISKID_BITE_THRESHOLD;
        self.pedal_steer_fault = faults.get(self.ids.pedal_steer_fail) >= ANTISKID_BITE_THRESHOLD;
        // `E-IND-DESIGN.md` 320800049/050: the two body steering positions'
        // own `actuator_leak`, surfaced directly with no new failure id.
        self.body_steer_fault = [gear_faults.left_body_steering.actuator_leak >= ANTISKID_BITE_THRESHOLD, gear_faults.right_body_steering.actuator_leak >= ANTISKID_BITE_THRESHOLD];

        // `E-IND-DESIGN.md` 320800048: the ALTN nosewheel-steering
        // circuit's own thermal law (see the constants' own doc comment).
        // Heat input is reasoned from how far the nosewheel is commanded
        // deflected (a real valve has to keep flowing to hold a surface
        // against airload/friction, not only while slewing); convective
        // loss is proportional to the temperature rise above ambient.
        let dt = truth.dt_s.max(0.0);
        let commanded_angle_deg = truth.controls.steering_command_deg[0];
        self.prev_nw_commanded_angle_deg = commanded_angle_deg;
        let heat_in_w = ALTN_STEER_HEAT_PER_DEG_W * commanded_angle_deg.abs();
        let convective_w = ALTN_STEER_CONVECTION_W_K * (self.altn_steer_sys_temp_c - truth.environment.sat_c);
        self.altn_steer_sys_temp_c += (heat_in_w - convective_w) * dt / ALTN_STEER_THERMAL_CAPACITY_J_K;
    }

    fn publish(&self, out: &mut dyn FnMut(&str, f64)) {
        let b = |x: bool| if x { 1.0 } else { 0.0 };
        let o = &self.outputs;
        let legs = [&o.nose, &o.left_wing, &o.right_wing, &o.left_body, &o.right_body];
        let struts = [&self.system.nose_strut, &self.system.left_wing_strut, &self.system.right_wing_strut, &self.system.left_body_strut, &self.system.right_body_strut];

        for (i, leg) in legs.iter().enumerate() {
            let n = i + 1;
            // The pair the L/G GEAR DISAGREE alert compares: what the leg
            // really is, and what its proximity sensors say it is.
            out(&format!("GEAR_DOWNLOCKED:{n}"), b(leg.downlocked));
            out(&format!("GEAR_UPLOCKED:{n}"), b(leg.uplocked));
            out(&format!("SENSED_GEAR_DOWNLOCKED:{n}"), b(leg.sensed_downlocked));
            out(&format!("SENSED_GEAR_UPLOCKED:{n}"), b(leg.sensed_uplocked));
            out(&format!("GEAR_POSITION:{n}"), leg.gear_position);
            out(&format!("GEAR_DOOR_POSITION:{n}"), leg.door_position);
            out(&format!("GEAR_STUCK_LOCKED:{n}"), b(leg.stuck_locked));
            out(&format!("GEAR_LEG_FORCE_N:{n}"), leg.force_n);
            out(&format!("GEAR_LEG_COMPRESSION:{n}"), leg.compression_frac);
            out(&format!("GEAR_STRUT_COLLAPSED:{n}"), b(leg.collapsed));
            out(&format!("GEAR_STRUT_GAS_CHARGE_FRACTION:{n}"), struts[i].gas_charge_fraction);
            out(&format!("GEAR_STRUT_OIL_LEVEL_FRACTION:{n}"), struts[i].oil_level_fraction);
            out(&format!("GEAR_STRUT_LIFE_FRACTION:{n}"), struts[i].life_fraction_consumed);
            // `E-IND-DESIGN.md` 320800043/046/032.
            out(&format!("GEAR_STRUT_PRESS_SENSOR_FAULT:{n}"), b(leg.gas_charge_sensor_fault));
            out(&format!("SENSED_ON_GROUND:{n}"), b(leg.sensed_on_ground));
            out(&format!("BOGIE_TRIMMED:{n}"), b(leg.bogie_trimmed));
            // `E-IND-DESIGN.md` 320800046: the leg's true ground-contact
            // state as a boolean, so the WEIGHT ON WHEELS FAULT trigger can
            // compare it against `SENSED_ON_GROUND:n` with a plain
            // variable-vs-variable inequality rather than a value-vs-
            // threshold mismatch (`GEAR_LEG_FORCE_N:n` is a continuous
            // newton reading, not a 0/1 flag).
            out(&format!("TRUE_ON_GROUND:{n}"), b(leg.force_n > 0.0));
        }

        for wheel in 0..N_BRAKED_WHEELS {
            let n = wheel + 1;
            out(&format!("BRAKE_FIRE:{n}"), b(o.brake_wheel_fire[wheel]));
            out(&format!("BRAKE_STACK_TEMP_C:{n}"), o.brake_wheel_temps_c[wheel]);
            out(&format!("BRAKE_WEAR_FRACTION:{n}"), o.brake_wheel_wear_fraction[wheel]);
            out(&format!("BRAKE_SKIDDING:{n}"), b(o.brake_wheel_skidding[wheel]));
            out(&format!("ANTISKID_CHANNEL_FAULT:{n}"), b(self.antiskid_channel_fault[wheel]));
            // `E-IND-DESIGN.md` "BRAKE_APPLIED_FRACTION:n".
            out(&format!("BRAKE_APPLIED_FRACTION:{n}"), o.brake_wheel_applied_fraction[wheel]);
        }

        out("PARK_BRAKE_SET", b(self.parking_brake_set));
        out("PARK_BRAKE_HOLDING", b(o.parking_brake_holding));
        out("PARK_BRAKE_PRESS_PA", o.parking_brake_pressure_pa);

        out("NW_STEER_ANGLE_DEG", o.nose_wheel_angle_deg);
        out("NW_STEER_SHIMMY_UNSTABLE", b(o.nose_steer_shimmy_unstable));
        out("NW_STEER_DISCONNECTED", b(o.nw_steer_disconnected));
        for i in 0..2 {
            let n = i + 1;
            out(&format!("BODY_STEER_ANGLE_DEG:{n}"), o.body_steer_angle_deg[i]);
            out(&format!("BODY_STEER_SHIMMY_UNSTABLE:{n}"), b(o.body_steer_shimmy_unstable[i]));
            out(&format!("BODY_STEER_FAULT:{n}"), b(self.body_steer_fault[i]));
        }

        // Not `GEAR_LEVER_POSITION_REQUEST`: that is FlyByWire's own *input*
        // (gear.rs maps GEAR_UP/DOWN/TOGGLE/SET onto it, landing_gear/mod.rs
        // reads it), and this area's lever comes from FlyByWire's *output*
        // `GEAR_HANDLE_POSITION`. Publishing one back as the other -- deep
        // ticks after the systems and wins the name -- rewrote every gear-up
        // request to "down" before the systems could act on it, so the gear
        // could never be raised. The lever position the legs are acting on
        // goes out under this area's own name instead, for the L/G ECAM
        // procedures (`fbw/ata32.rs`) and the RECYCLE line below.
        out("GEAR_LEVER_SELECTED_DOWN", b(self.gear_lever_down));
        // `E-IND-DESIGN.md` 320800042 L/G GRVTY EXTN FAULT.
        out("GRAVITY_EXTEND_SELECTED", b(self.gravity_extend_selected));

        out("GEAR_WING_FATIGUE_INDEX", o.wing_fatigue_index);

        // `E-IND-DESIGN.md`'s new Brake System Controller (BSC).
        out("BSCU_CHANNEL_FAULT:1", b(self.bscu_ctl_fault[0]));
        out("BSCU_CHANNEL_FAULT:2", b(self.bscu_ctl_fault[1]));
        out("NORM_BRK_PRESS_SENSOR_FAULT", b(self.norm_brk_press_sensor_fault));
        out("ALTN_BRK_PRESS_SENSOR_FAULT", b(self.altn_brk_press_sensor_fault));
        out("AUTO_BRK_FAULT", b(self.auto_brk_fault));
        out("BRAKE_SEL_VLV_JAMMED", b(self.brake_sel_vlv_jammed));
        out("BRAKE_PEDAL_SENSOR_FAULT:1", b(self.brake_pedal_sensor_fault[0]));
        out("BRAKE_PEDAL_SENSOR_FAULT:2", b(self.brake_pedal_sensor_fault[1]));

        // `E-IND-DESIGN.md`'s new Steering System Controller (SSC).
        out("STEER_CTL_FAULT:1", b(self.steer_ctl_fault[0]));
        out("STEER_CTL_FAULT:2", b(self.steer_ctl_fault[1]));
        out("STEER_SEL_VLV_JAMMED", b(self.steer_sel_vlv_jammed));
        out("STEER_TILLER_FAULT:capt", b(self.capt_tiller_fault));
        out("STEER_TILLER_FAULT:fo", b(self.fo_tiller_fault));
        out("STEER_PEDAL_FAULT", b(self.pedal_steer_fault));
        out("ALTN_STEER_SYS_TEMP_C", self.altn_steer_sys_temp_c);
        out("BRAKE_PEDAL_COMMANDED_FRACTION", self.brake_pedal_commanded_fraction);
        out("NW_STEER_DISC_SELECTED", b(self.nw_steer_disc_selected));
        out("BRAKES_MINOR_FAULT", b(self.brakes_minor_fault));
    }
}

/// This area's live system.
pub fn live_system() -> Box<dyn crate::deep::live::Area> {
    Box::new(GearStructureLive::new())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::deep::live::Area as _;
    use std::collections::BTreeMap;

    fn published(area: &dyn crate::deep::live::Area) -> BTreeMap<String, f64> {
        let mut map = BTreeMap::new();
        area.publish(&mut |name, value| {
            map.insert(name.to_string(), value);
        });
        map
    }

    /// A pressurised aircraft rolling out: both systems up, on the ground,
    /// moving.
    fn rollout_truth() -> Truth {
        Truth { hydraulic_pressure_pa: [HYDRAULIC_NOMINAL_PA; 2], on_ground: true, ..Truth::default() }
    }

    fn run(live: &mut GearStructureLive, truth: &Truth, faults: &Faults, seconds: f64) -> BTreeMap<String, f64> {
        let steps = (seconds / truth.dt_s).ceil() as usize;
        for _ in 0..steps.max(1) {
            live.tick(truth, faults);
        }
        published(live)
    }

    #[test]
    fn a_parked_aircraft_has_all_five_legs_down_and_locked_and_raises_nothing() {
        let mut live = GearStructureLive::new();
        let out = run(&mut live, &rollout_truth(), &Faults::default(), 2.0);
        for n in 1..=5 {
            assert_eq!(out.get(&format!("GEAR_DOWNLOCKED:{n}")), Some(&1.0), "leg {n} should be down and locked on a parked aircraft");
            assert_eq!(out.get(&format!("SENSED_GEAR_DOWNLOCKED:{n}")), Some(&1.0));
            assert_eq!(out.get(&format!("GEAR_UPLOCKED:{n}")), Some(&0.0));
        }
        for n in 1..=16 {
            assert_eq!(out.get(&format!("BRAKE_FIRE:{n}")), Some(&0.0));
            assert_eq!(out.get(&format!("ANTISKID_CHANNEL_FAULT:{n}")), Some(&0.0));
        }
        assert_eq!(out.get("NW_STEER_SHIMMY_UNSTABLE"), Some(&0.0));
        assert_eq!(out.get("BODY_STEER_SHIMMY_UNSTABLE:1"), Some(&0.0));
    }

    #[test]
    fn every_variable_this_areas_alerts_trigger_on_is_published_by_this_live_system() {
        let mut reg = Registry::default();
        super::super::registry::register(&mut reg);
        let mut names = Vec::new();
        for alert in &reg.alerts {
            crate::deep::fuel::live::test_support::collect_vars(&alert.trigger, &mut names);
        }
        let mut live = GearStructureLive::new();
        live.tick(&rollout_truth(), &Faults::default());
        let out = published(&live);
        for name in names {
            assert!(out.contains_key(&name), "alert trigger reads {name}, which nothing publishes");
        }
    }

    #[test]
    fn a_dragging_brake_heats_its_own_wheel_and_only_its_own() {
        // registry.rs: "this wheel's stack heats and wears even with no
        // pedal/autobrake command, and can reach the fire threshold on a
        // long taxi".
        let mut truth = rollout_truth();
        truth.dt_s = 0.1;
        truth.environment.sat_c = 15.0;
        truth.groundspeed_m_s = 15.0; // taxi speed

        let mut live = GearStructureLive::new();
        let id = live.ids.dragging[0];
        let out = run(&mut live, &truth, &Faults::from_pairs([(id, 1.0)]), 600.0);

        let dragged = out["BRAKE_STACK_TEMP_C:1"];
        let neighbour = out["BRAKE_STACK_TEMP_C:2"];
        assert!(dragged > neighbour + 50.0, "a dragging brake must run far hotter than its neighbours: {dragged} vs {neighbour}");
        assert!(out["BRAKE_WEAR_FRACTION:1"] > out["BRAKE_WEAR_FRACTION:2"], "and must wear faster");
    }

    #[test]
    fn an_antiskid_channel_failure_is_reported_by_the_computers_own_bite() {
        let mut live = GearStructureLive::new();
        let id = live.ids.antiskid_inop[5];
        let out = run(&mut live, &rollout_truth(), &Faults::from_pairs([(id, 1.0)]), 1.0);
        assert_eq!(out.get("ANTISKID_CHANNEL_FAULT:6"), Some(&1.0));
        assert_eq!(out.get("ANTISKID_CHANNEL_FAULT:5"), Some(&0.0), "only the failed channel reports");
    }

    #[test]
    fn a_lying_lock_sensor_makes_the_indication_disagree_with_the_leg() {
        // registry.rs: "the cockpit indication disagrees with the leg's
        // true lock state, independent of it" -- which is exactly what the
        // L/G GEAR DISAGREE trigger compares.
        let mut live = GearStructureLive::new();
        let id = live.ids.sensor_lies[1]; // left wing leg
        let out = run(&mut live, &rollout_truth(), &Faults::from_pairs([(id, 1.0)]), 2.0);
        let truth_locked = out["GEAR_DOWNLOCKED:2"];
        let sensed_locked = out["SENSED_GEAR_DOWNLOCKED:2"];
        assert_ne!(truth_locked, sensed_locked, "a lying sensor must make the two disagree");

        // And the alert that compares them actually fires on it.
        let mut reg = Registry::default();
        super::super::registry::register(&mut reg);
        let alert = reg.alerts.iter().find(|a| a.key == "L_G_GEAR_DISAGREE").expect("registered");
        assert!(alert.trigger.eval(&|name: &str| out.get(name).copied().unwrap_or(0.0)));
    }

    #[test]
    fn a_leaking_parking_brake_accumulator_stops_holding_and_raises_the_low_pressure_caution() {
        // registry.rs: "the accumulator's stored pressure bleeds down while
        // the parking brake is set, eventually falling below the minimum
        // holding pressure".
        let mut truth = rollout_truth();
        truth.dt_s = 1.0;
        truth.controls.parking_brake_on = true;

        let mut healthy = GearStructureLive::new();
        let healthy_out = run(&mut healthy, &truth, &Faults::default(), 600.0);
        assert_eq!(healthy_out.get("PARK_BRAKE_HOLDING"), Some(&1.0), "a healthy accumulator still holds after ten minutes");

        let mut leaking = GearStructureLive::new();
        let id = leaking.ids.parking_brake_leak;
        let leak_out = run(&mut leaking, &truth, &Faults::from_pairs([(id, 1.0)]), 36_000.0);
        assert_eq!(leak_out.get("PARK_BRAKE_SET"), Some(&1.0));
        assert_eq!(leak_out.get("PARK_BRAKE_HOLDING"), Some(&0.0), "a fully leaking accumulator must eventually stop holding");
        assert!(leak_out["PARK_BRAKE_PRESS_PA"] < healthy_out["PARK_BRAKE_PRESS_PA"]);

        let mut reg = Registry::default();
        super::super::registry::register(&mut reg);
        let alert = reg.alerts.iter().find(|a| a.key == "L_G_PARK_BRK_LO_PR").expect("registered");
        assert!(alert.trigger.eval(&|name: &str| leak_out.get(name).copied().unwrap_or(0.0)));
    }

    #[test]
    fn a_failed_shimmy_damper_goes_unstable_above_its_own_reduced_critical_speed() {
        // registry.rs: "the critical (unstable) groundspeed for this
        // wheel's torsional shimmy mode falls; above it, a self-excited
        // oscillation grows instead of damping out".
        let healthy_critical = super::super::steering::SteeringActuator::critical_speed_ms(0.0);
        let failed_critical = super::super::steering::SteeringActuator::critical_speed_ms(1.0);
        assert!(failed_critical < healthy_critical);

        // A speed between the two: fine with a healthy damper, unstable
        // with a failed one. Nothing else changes.
        let speed = 0.5 * (failed_critical + healthy_critical);
        let mut truth = rollout_truth();
        truth.dt_s = 0.02;
        truth.groundspeed_m_s = speed;
        truth.controls.steering_command_deg[0] = 2.0;

        let mut healthy = GearStructureLive::new();
        let healthy_out = run(&mut healthy, &truth, &Faults::default(), 20.0);
        assert_eq!(healthy_out.get("NW_STEER_SHIMMY_UNSTABLE"), Some(&0.0));

        let mut failed = GearStructureLive::new();
        let id = failed.ids.shimmy[0];
        let failed_out = run(&mut failed, &truth, &Faults::from_pairs([(id, 1.0)]), 20.0);
        assert_eq!(failed_out.get("NW_STEER_SHIMMY_UNSTABLE"), Some(&1.0), "a failed nose damper must go unstable at {speed} m/s");
    }

    #[test]
    fn a_body_gear_shimmy_is_published_independently_of_the_nosewheels() {
        let failed_critical = super::super::steering::SteeringActuator::critical_speed_ms(1.0);
        let healthy_critical = super::super::steering::SteeringActuator::critical_speed_ms(0.0);
        let speed = 0.5 * (failed_critical + healthy_critical);
        let mut truth = rollout_truth();
        truth.dt_s = 0.02;
        truth.groundspeed_m_s = speed;
        truth.controls.steering_command_deg[0] = 20.0;

        let mut live = GearStructureLive::new();
        let id = live.ids.shimmy[1]; // left body gear
        let out = run(&mut live, &truth, &Faults::from_pairs([(id, 1.0)]), 20.0);
        assert_eq!(out.get("NW_STEER_SHIMMY_UNSTABLE"), Some(&0.0), "the nosewheel's own damper is healthy");
        assert_eq!(out.get("BODY_STEER_SHIMMY_UNSTABLE:1"), Some(&1.0));
    }

    /// `deep::integration::failure_audit`'s sweep found all three steering
    /// actuators' `actuator_leak` failures dead: with `GearCommands::
    /// nose_steering_command_deg` pinned at 0 forever (nothing in
    /// production ever set it), the actuator's own target never moved, and
    /// a fault that only throttles the *rate of a still-in-progress* slew
    /// (`steering::SteeringActuator::step`) has nothing to throttle at
    /// `diff == 0`. `Truth::controls::steering_command_deg[0]` fixes the
    /// nose actuator directly; the body actuators need no separate wiring
    /// (module doc) because their own target is derived from the nose
    /// actuator's real tracked angle.
    #[test]
    fn a_steering_actuator_leak_only_shows_once_a_real_commanded_angle_gives_it_something_to_chase() {
        let mut truth = rollout_truth();
        truth.dt_s = 0.05;
        truth.controls.steering_command_deg[0] = 45.0; // a real tiller deflection

        let mut healthy = GearStructureLive::new();
        let healthy_out = run(&mut healthy, &truth, &Faults::default(), 0.5);

        let mut leaking = GearStructureLive::new();
        let nose_id = leaking.ids.steer_actuator_leak[0];
        let leaking_out = run(&mut leaking, &truth, &Faults::from_pairs([(nose_id, 1.0)]), 0.5);

        assert!(healthy_out["NW_STEER_ANGLE_DEG"] > 0.0, "a healthy actuator must be visibly slewing toward the command");
        assert!(
            leaking_out["NW_STEER_ANGLE_DEG"] < healthy_out["NW_STEER_ANGLE_DEG"],
            "a leaking actuator must lag a healthy one chasing the same real command: {} vs {}",
            leaking_out["NW_STEER_ANGLE_DEG"],
            healthy_out["NW_STEER_ANGLE_DEG"]
        );

        // The body actuators, driven only through the derived schedule off
        // the nose's own real tracked angle, must show the same effect from
        // their own leak fault -- no separate `Truth` field needed.
        let mut body_leaking = GearStructureLive::new();
        let body_id = body_leaking.ids.steer_actuator_leak[1]; // left body
        let body_out = run(&mut body_leaking, &truth, &Faults::from_pairs([(body_id, 1.0)]), 0.5);
        assert_ne!(
            body_out["BODY_STEER_ANGLE_DEG:1"], healthy_out["BODY_STEER_ANGLE_DEG:1"],
            "the left body actuator's own leak must move its own angle away from the healthy case"
        );
    }

    /// **Investigation, not a bug.** `deep::integration::failure_audit`'s
    /// `gear_cycle` profile holds one fixed `Truth` for its whole 20 s
    /// window (that module's own doc: "Fixed rather than flown") and only
    /// ever commands the gear *up* (`gear_lever_down: false` throughout).
    /// `uplock_jam`'s release gate and `downlock_fail`'s engagement check
    /// (`retraction::Retraction::step`) both only run on the *other*
    /// transition -- commanding a leg that is currently up back down again
    /// -- which a profile whose `Truth` never changes cannot express. This
    /// is not a missing `Truth` input and not a gate that needs "more than
    /// the lever": the chain already reacts correctly to the lever alone.
    /// Proven here by doing what the fixed-profile audit structurally
    /// cannot -- commanding the gear up, then back down, in the same run --
    /// which is why this lives as a test in this file rather than a change
    /// to `failure_audit.rs` (outside this pass's directory).
    #[test]
    fn uplock_jam_and_downlock_failure_are_real_and_only_show_on_the_extend_that_gear_cycle_never_commands() {
        let mut truth = rollout_truth();
        truth.dt_s = 0.1;
        truth.controls.gear_lever_down = false; // retract first

        let mut jammed = GearStructureLive::new();
        let mut healthy = GearStructureLive::new();
        for _ in 0..300 {
            jammed.tick(&truth, &Faults::default());
            healthy.tick(&truth, &Faults::default());
        }
        assert_eq!(published(&jammed).get("GEAR_UPLOCKED:1"), Some(&1.0), "the nose leg must have fully retracted and up-locked");

        // Now command extend, with the nose uplock jammed on one run only.
        truth.controls.gear_lever_down = true;
        let jam_id = jammed.ids.uplock_jam[0];
        let jam_faults = Faults::from_pairs([(jam_id, 1.0)]);
        for _ in 0..300 {
            jammed.tick(&truth, &jam_faults);
            healthy.tick(&truth, &Faults::default());
        }
        let jammed_out = published(&jammed);
        let healthy_out = published(&healthy);

        assert_eq!(healthy_out.get("GEAR_DOWNLOCKED:1"), Some(&1.0), "a healthy nose leg must extend and downlock again");
        assert_eq!(jammed_out.get("GEAR_DOWNLOCKED:1"), Some(&0.0), "a jammed uplock must prevent the leg from ever extending");
        assert_eq!(jammed_out.get("GEAR_UPLOCKED:1"), Some(&1.0), "it must still be stuck up-locked");
        assert_eq!(jammed_out.get("GEAR_STUCK_LOCKED:1"), Some(&1.0), "and the stuck-locked hazard flag must be set");
    }

    /// The companion case: the leg reaches the geometric down position but
    /// the spring/linkage never truly seats, once it actually extends
    /// (`retraction.rs`'s own `a_failed_downlock_reaches_the_down_position_
    /// but_never_reports_locked` proves the underlying state machine
    /// directly; this proves the same thing reached through `Truth` via
    /// the live system, in the same up-then-down cycle the previous test
    /// uses).
    #[test]
    fn downlock_failure_reaches_the_down_position_but_never_locks_once_it_actually_extends() {
        let mut truth = rollout_truth();
        truth.dt_s = 0.1;
        truth.controls.gear_lever_down = false;

        let mut live = GearStructureLive::new();
        for _ in 0..300 {
            live.tick(&truth, &Faults::default());
        }
        assert_eq!(published(&live).get("GEAR_UPLOCKED:1"), Some(&1.0));

        truth.controls.gear_lever_down = true;
        let id = live.ids.downlock_fail[0];
        let faults = Faults::from_pairs([(id, 1.0)]);
        for _ in 0..300 {
            live.tick(&truth, &faults);
        }
        let out = published(&live);
        assert!(out["GEAR_POSITION:1"] > 0.95, "the leg must still reach the down position geometrically");
        assert_eq!(out.get("GEAR_DOWNLOCKED:1"), Some(&0.0), "but the downlock must never truly engage");

        // And the alert that watches exactly this actually fires.
        let mut reg = Registry::default();
        super::super::registry::register(&mut reg);
        let alert = reg.alerts.iter().find(|a| a.key == "L_G_GEAR_NOT_DOWNLOCKED").expect("registered");
        assert!(alert.trigger.eval(&|name: &str| out.get(name).copied().unwrap_or(0.0)));
    }

    #[test]
    fn a_strut_gas_leak_drains_the_precharge_and_is_visible_on_the_strut_variables() {
        // registry.rs: "gas_charge_fraction depletes; the leg sags to a
        // higher static compression for the same load".
        let mut truth = rollout_truth();
        truth.dt_s = 1.0;

        let mut live = GearStructureLive::new();
        let id = live.ids.strut_gas_leak[1];
        let before = run(&mut live, &truth, &Faults::default(), 1.0);
        let out = run(&mut live, &truth, &Faults::from_pairs([(id, 1.0)]), 20_000.0);
        assert!(out["GEAR_STRUT_GAS_CHARGE_FRACTION:2"] < 0.9, "a full gas leak must deplete the precharge");
        assert!(
            out["GEAR_LEG_COMPRESSION:2"] > before["GEAR_LEG_COMPRESSION:2"],
            "and the leg must sag further under the same load: {} -> {}",
            before["GEAR_LEG_COMPRESSION:2"],
            out["GEAR_LEG_COMPRESSION:2"]
        );
        assert_eq!(out["GEAR_STRUT_GAS_CHARGE_FRACTION:3"], 1.0, "the other legs are untouched");
    }

    #[test]
    fn a_hard_touchdown_via_truth_consumes_more_gear_life_than_a_gentle_one() {
        // `Truth::leg_touchdown_sink_speed_ms`/`leg_on_ground` are new
        // (module doc): this area previously had only one aircraft-wide
        // `on_ground` flag and no sink speed at all -- the single number a
        // hard-landing model is most sensitive to, and `commands.
        // leg_sink_speed_ms` defaulted to zero forever since nothing in
        // production ever set it. This proves the real reading actually
        // reaches the strut law, by landing the same leg at two different
        // real sink speeds and checking the harder one costs more fatigue
        // life (`strut.rs`'s own `fatigue_accumulates_more_from_a_harder_
        // landing_than_a_gentle_one` proves the underlying law from a
        // direct `Strut`; this proves the `Truth` wiring that reaches it).
        //
        // CS-25.473(a): design descent velocity not less than 10 fps
        // (3.05 m/s) at design landing weight -- the same limit sink speed
        // `strut.rs`'s own fatigue test cites, restated here since it is a
        // private constant of that file.
        const SINK_SPEED_LIMIT_MS: f64 = 3.05;

        let left_wing_life_fraction = |sink_speed_ms: f64| -> f64 {
            let mut live = GearStructureLive::new();
            let mut truth = rollout_truth();
            truth.dt_s = 0.02;

            // Airborne first: `sink_speed_ms` is only read on the tick a
            // leg's `on_ground` goes false -> true, so a genuine touchdown
            // transient needs the leg in the air before it lands (matching
            // `strut.rs`'s own `land` test helper).
            truth.on_ground = false;
            truth.leg_on_ground = [false; 5];
            for _ in 0..50 {
                live.tick(&truth, &Faults::default());
            }

            // Touch down on all five legs at once, at `sink_speed_ms`, and
            // hold there while the transient settles.
            truth.on_ground = true;
            truth.leg_on_ground = [true; 5];
            truth.leg_touchdown_sink_speed_ms = [sink_speed_ms; 5];
            for _ in 0..500 {
                live.tick(&truth, &Faults::default());
            }

            // Liftoff: closes out the ground-contact cycle and applies its
            // Miner's-rule fatigue increment (`strut.rs`'s own `close_cycle`
            // doc) -- without this the leg is still mid-cycle and the
            // increment has not landed yet.
            truth.on_ground = false;
            truth.leg_on_ground = [false; 5];
            live.tick(&truth, &Faults::default());

            published(&live)["GEAR_STRUT_LIFE_FRACTION:2"] // left wing leg
        };

        let gentle = left_wing_life_fraction(SINK_SPEED_LIMIT_MS * 0.3);
        let hard = left_wing_life_fraction(SINK_SPEED_LIMIT_MS * 0.95);
        assert!(hard > gentle, "a harder real touchdown must consume more fatigue life than a gentle one: {hard} vs {gentle}");
    }

    #[test]
    fn nothing_divides_by_zero_at_rest_with_zero_dt() {
        let mut live = GearStructureLive::new();
        live.tick(&Truth { dt_s: 0.0, ..Truth::default() }, &Faults::default());
        for (name, value) in published(&live) {
            assert!(value.is_finite(), "{name} is not finite");
        }
    }

    #[test]
    fn every_registered_failure_is_either_consumed_or_listed_as_not() {
        let mut reg = Registry::default();
        super::super::registry::register(&mut reg);
        let ids = Ids::resolve();
        let mut consumed: Vec<u64> = Vec::new();
        consumed.extend(ids.strut_gas_leak);
        consumed.extend(ids.strut_oil_leak);
        consumed.extend(ids.actuator_leak);
        consumed.extend(ids.uplock_jam);
        consumed.extend(ids.downlock_fail);
        consumed.extend(ids.door_jam);
        consumed.extend(ids.sensor_lies);
        consumed.extend(ids.antiskid_inop);
        consumed.extend(ids.dragging);
        consumed.push(ids.parking_brake_leak);
        consumed.extend(ids.shimmy);
        consumed.extend(ids.steer_actuator_leak);
        // `E-IND-DESIGN.md`'s additions: struts' pressure-monitoring and
        // weight-on-wheels-sensing BITE, the two body legs' bogie-trim
        // BITE, the nosewheel's disconnect-mechanism and angle-limit-
        // override failures, and the new BSC/SSC controller components.
        // `bogie_trim_fail` carries a real `0` placeholder on the
        // nose/wing legs (never a registered id) rather than a duplicate,
        // so it is filtered out before the "every registered id is
        // consumed" check below, exactly like every other `[u64; N_LEGS]`
        // array here would if a real id happened to collide with it (it
        // cannot: every real id is >= `failure_id(Area::GearStructure, ..)`
        // which is far larger than 0).
        consumed.extend(ids.strut_gas_charge_sensor_fail);
        consumed.extend(ids.strut_wow_sensing_fail);
        consumed.extend(ids.bogie_trim_fail.iter().copied().filter(|&id| id != 0));
        consumed.push(ids.steer_disc_mechanism_fail);
        consumed.push(ids.steer_overtravel_fail);
        consumed.extend(ids.bscu_ctl);
        consumed.push(ids.bscu_norm_press_sensor_fail);
        consumed.push(ids.bscu_alt_press_sensor_fail);
        consumed.push(ids.bscu_autobrake_fail);
        consumed.push(ids.bscu_sel_valve_jam);
        consumed.extend(ids.brake_pedal_sensor_fail);
        consumed.extend(ids.steer_ctl);
        consumed.push(ids.steer_sel_valve_jam);
        consumed.push(ids.capt_tiller_fail);
        consumed.push(ids.fo_tiller_fail);
        consumed.push(ids.pedal_steer_fail);
        consumed.sort_unstable();
        consumed.dedup();

        let registered: Vec<u64> = reg.failures.iter().map(|f| f.id).collect();
        assert_eq!(consumed.len(), registered.len(), "every gear failure should be consumed by the live system");
        for id in registered {
            assert!(consumed.contains(&id), "failure {id} is registered but never read by the live system");
        }
    }

    #[test]
    fn registering_twice_hands_out_the_same_ids_both_times() {
        // Without this, the ids the live system resolves would not be the
        // ids the plugin's own `deep::registry()` arms in `Faults`.
        let a = Ids::resolve();
        let b = Ids::resolve();
        assert_eq!(a.strut_gas_leak, b.strut_gas_leak);
        assert_eq!(a.dragging, b.dragging);
        assert_eq!(a.parking_brake_leak, b.parking_brake_leak);
        assert_eq!(a.bscu_ctl, b.bscu_ctl);
        assert_eq!(a.bogie_trim_fail, b.bogie_trim_fail);
        assert_eq!(a.steer_disc_mechanism_fail, b.steer_disc_mechanism_fail);
    }

    /// `E-IND-DESIGN.md`'s new Brake System Controller (BSC): each of its
    /// six channels is an independent BITE flag that fires only its own
    /// published name.
    #[test]
    fn bscu_channels_are_independent_bite_flags() {
        let mut live = GearStructureLive::new();
        let ctl1 = live.ids.bscu_ctl[0];
        let ctl2 = live.ids.bscu_ctl[1];
        let norm_sensor = live.ids.bscu_norm_press_sensor_fail;
        let autobrake = live.ids.bscu_autobrake_fail;
        let sel_valve = live.ids.bscu_sel_valve_jam;

        let healthy = run(&mut GearStructureLive::new(), &rollout_truth(), &Faults::default(), 1.0);
        for name in ["BSCU_CHANNEL_FAULT:1", "BSCU_CHANNEL_FAULT:2", "NORM_BRK_PRESS_SENSOR_FAULT", "ALTN_BRK_PRESS_SENSOR_FAULT", "AUTO_BRK_FAULT", "BRAKE_SEL_VLV_JAMMED"] {
            assert_eq!(healthy.get(name), Some(&0.0), "{name} must be quiet when healthy");
        }

        let out = run(&mut live, &rollout_truth(), &Faults::from_pairs([(ctl1, 1.0)]), 1.0);
        assert_eq!(out.get("BSCU_CHANNEL_FAULT:1"), Some(&1.0));
        assert_eq!(out.get("BSCU_CHANNEL_FAULT:2"), Some(&0.0), "only the armed channel must report");

        let mut live2 = GearStructureLive::new();
        let out2 = run(&mut live2, &rollout_truth(), &Faults::from_pairs([(ctl2, 1.0), (norm_sensor, 1.0), (autobrake, 1.0), (sel_valve, 1.0)]), 1.0);
        assert_eq!(out2.get("BSCU_CHANNEL_FAULT:2"), Some(&1.0));
        assert_eq!(out2.get("NORM_BRK_PRESS_SENSOR_FAULT"), Some(&1.0));
        assert_eq!(out2.get("AUTO_BRK_FAULT"), Some(&1.0));
        assert_eq!(out2.get("BRAKE_SEL_VLV_JAMMED"), Some(&1.0));
        assert_eq!(out2.get("BSCU_CHANNEL_FAULT:1"), Some(&0.0));
    }

    /// The new Steering System Controller (SSC)'s own channels, plus the
    /// two body positions' `BODY_STEER_FAULT` surfaced from their existing
    /// `actuator_leak`.
    #[test]
    fn ssc_channels_and_body_steer_fault_are_independent() {
        let mut live = GearStructureLive::new();
        let ctl1 = live.ids.steer_ctl[0];
        let capt_tiller = live.ids.capt_tiller_fail;
        let pedal_steer = live.ids.pedal_steer_fail;
        let body_leak_left = live.ids.steer_actuator_leak[1]; // left body

        let out = run(&mut live, &rollout_truth(), &Faults::from_pairs([(ctl1, 1.0), (capt_tiller, 1.0), (pedal_steer, 1.0)]), 1.0);
        assert_eq!(out.get("STEER_CTL_FAULT:1"), Some(&1.0));
        assert_eq!(out.get("STEER_CTL_FAULT:2"), Some(&0.0));
        assert_eq!(out.get("STEER_TILLER_FAULT:capt"), Some(&1.0));
        assert_eq!(out.get("STEER_TILLER_FAULT:fo"), Some(&0.0));
        assert_eq!(out.get("STEER_PEDAL_FAULT"), Some(&1.0));
        assert_eq!(out.get("BODY_STEER_FAULT:1"), Some(&0.0), "untouched by the SSC channels above");

        let mut body = GearStructureLive::new();
        let out2 = run(&mut body, &rollout_truth(), &Faults::from_pairs([(body_leak_left, 1.0)]), 1.0);
        assert_eq!(out2.get("BODY_STEER_FAULT:1"), Some(&1.0));
        assert_eq!(out2.get("BODY_STEER_FAULT:2"), Some(&0.0), "only the left body position's own leak must report");
    }

    /// `E-IND-DESIGN.md` 320800048 STEER ALTN STEER SYS HOT: sustained
    /// nosewheel deflection heats the modelled ALTN circuit past the
    /// GENERIC 100 C threshold; a healthy (centred) nosewheel stays near
    /// ambient.
    #[test]
    fn sustained_steering_deflection_heats_the_altn_circuit_past_the_hot_threshold() {
        let mut truth = rollout_truth();
        truth.dt_s = 1.0;
        truth.environment.sat_c = 15.0;

        let mut idle = GearStructureLive::new();
        let idle_out = run(&mut idle, &truth, &Faults::default(), 600.0);
        assert!(idle_out["ALTN_STEER_SYS_TEMP_C"] < 30.0, "a centred nosewheel must stay near ambient: {}", idle_out["ALTN_STEER_SYS_TEMP_C"]);

        truth.controls.steering_command_deg[0] = 45.0;
        let mut deflected = GearStructureLive::new();
        let out = run(&mut deflected, &truth, &Faults::default(), 3_000.0);
        assert!(out["ALTN_STEER_SYS_TEMP_C"] > super::ALTN_STEER_SYS_HOT_C, "sustained deflection must heat the ALTN circuit past the hot threshold: {}", out["ALTN_STEER_SYS_TEMP_C"]);
    }

    /// `E-IND-DESIGN.md` 320800057/059 STEER N/W STEER DISC FAULT / NOT
    /// DISC, reached through `Truth` end to end: a healthy mechanism tracks
    /// the towing-lever selection; `disc_mechanism_fail` freezes it.
    #[test]
    fn nw_steer_disconnect_selection_is_wired_from_truth_and_can_be_jammed() {
        let mut truth = rollout_truth();
        truth.dt_s = 0.1;

        let mut live = GearStructureLive::new();
        let not_selected = run(&mut live, &truth, &Faults::default(), 1.0);
        assert_eq!(not_selected.get("NW_STEER_DISCONNECTED"), Some(&0.0));

        truth.controls.nw_steer_disc_selected = true;
        let selected = run(&mut live, &truth, &Faults::default(), 1.0);
        assert_eq!(selected.get("NW_STEER_DISCONNECTED"), Some(&1.0), "a healthy mechanism must respond to Truth's own selection");

        let mut jammed = GearStructureLive::new();
        let disc_id = jammed.ids.steer_disc_mechanism_fail;
        let jam_faults = Faults::from_pairs([(disc_id, 1.0)]);
        truth.controls.nw_steer_disc_selected = false;
        run(&mut jammed, &truth, &jam_faults, 0.1);
        // Select while jammed: must not move.
        truth.controls.nw_steer_disc_selected = true;
        let jam_out = run(&mut jammed, &truth, &jam_faults, 1.0);
        assert_eq!(jam_out.get("NW_STEER_DISCONNECTED"), Some(&0.0), "a jammed mechanism must not respond to a new selection reaching it through Truth");
    }

    /// `E-IND-DESIGN.md` 320800042 L/G GRVTY EXTN FAULT, reached through
    /// `Truth`: `GRAVITY_EXTEND_SELECTED` mirrors the crew selection, and a
    /// severe uplock jam then defeats even gravity extension (the
    /// mechanical path `retraction.rs`'s own test already proves directly;
    /// this proves the `Truth` wiring reaches it).
    #[test]
    fn gravity_extend_selected_is_wired_from_truth_and_can_still_be_defeated() {
        let mut truth = rollout_truth();
        truth.dt_s = 0.1;
        truth.controls.gear_lever_down = false; // retract first

        let mut live = GearStructureLive::new();
        for _ in 0..300 {
            live.tick(&truth, &Faults::default());
        }
        assert_eq!(published(&live).get("GEAR_UPLOCKED:1"), Some(&1.0));

        truth.controls.gear_lever_down = true;
        truth.controls.gravity_extend_selected = true;
        let out = run(&mut live, &truth, &Faults::default(), 0.1);
        assert_eq!(out.get("GRAVITY_EXTEND_SELECTED"), Some(&1.0), "the selection must be published back");

        // A severe jam on a fresh nose leg must still defeat it, reached
        // this time through the Truth-sourced selection rather than
        // `GearCommands`.
        let mut jammed = GearStructureLive::new();
        let jam_id = jammed.ids.uplock_jam[0];
        let mut up_truth = truth.clone();
        up_truth.controls.gear_lever_down = false;
        up_truth.controls.gravity_extend_selected = false;
        for _ in 0..300 {
            jammed.tick(&up_truth, &Faults::default());
        }
        assert_eq!(published(&jammed).get("GEAR_UPLOCKED:1"), Some(&1.0));

        let mut down_truth = truth.clone();
        down_truth.controls.gravity_extend_selected = true;
        let jam_faults = Faults::from_pairs([(jam_id, 1.0)]);
        let jam_out = run(&mut jammed, &down_truth, &jam_faults, 0.1 * 5_000.0);
        assert_eq!(jam_out.get("GEAR_STUCK_LOCKED:1"), Some(&1.0), "a severe uplock jam must defeat gravity extension reached through Truth too");
    }

    /// `E-IND-DESIGN.md` 320800019 BRAKES MINOR FAULT: a wheel's own
    /// `antiskid_inop` below the BITE threshold is a real, noticed
    /// degradation, distinct from (and not the same trigger as) the
    /// deep-registry's own `L_G_BRAKES_ANTISKID_FAULT`, which only fires at
    /// or above that threshold.
    #[test]
    fn brakes_minor_fault_fires_below_the_antiskid_bite_threshold_and_not_above_it() {
        let mut minor = GearStructureLive::new();
        let id = minor.ids.antiskid_inop[3];
        let out = run(&mut minor, &rollout_truth(), &Faults::from_pairs([(id, 0.2)]), 1.0);
        assert_eq!(out.get("BRAKES_MINOR_FAULT"), Some(&1.0), "a sub-threshold antiskid degradation must be noticed");
        assert_eq!(out.get("ANTISKID_CHANNEL_FAULT:4"), Some(&0.0), "but must not itself trip the channel's own BITE fault");

        let mut full = GearStructureLive::new();
        let out2 = run(&mut full, &rollout_truth(), &Faults::from_pairs([(id, 1.0)]), 1.0);
        assert_eq!(out2.get("ANTISKID_CHANNEL_FAULT:4"), Some(&1.0));

        let healthy = run(&mut GearStructureLive::new(), &rollout_truth(), &Faults::default(), 1.0);
        assert_eq!(healthy.get("BRAKES_MINOR_FAULT"), Some(&0.0));
    }

    /// `E-IND-DESIGN.md` 320800046 L/G WEIGHT ON WHEELS FAULT, reached
    /// through `Truth`: `TRUE_ON_GROUND:n` and `SENSED_ON_GROUND:n` agree
    /// when healthy and disagree once `wow_sensing_fail` is armed, and only
    /// on the armed leg.
    #[test]
    fn true_and_sensed_on_ground_disagree_only_on_the_leg_with_a_wow_sensing_failure() {
        let mut live = GearStructureLive::new();
        let id = live.ids.strut_wow_sensing_fail[1]; // left wing leg
        let out = run(&mut live, &rollout_truth(), &Faults::from_pairs([(id, 1.0)]), 2.0);
        assert_ne!(out["TRUE_ON_GROUND:2"], out["SENSED_ON_GROUND:2"], "the armed leg's sensed state must disagree with its true state");
        assert_eq!(out["TRUE_ON_GROUND:3"], out["SENSED_ON_GROUND:3"], "an untouched sibling leg must still agree");
    }

    #[test]
    fn never_publishes_flybywires_own_gear_lever_input() {
        let mut area = GearStructureLive::new();
        let truth = Truth::default();
        crate::deep::live::Area::tick(&mut area, &truth, &Faults::default());
        let mut names = Vec::new();
        crate::deep::live::Area::publish(&area, &mut |name, _| names.push(name.to_owned()));
        assert!(!names.iter().any(|n| n == "GEAR_LEVER_POSITION_REQUEST"), "writing FlyByWire's gear lever input back from its handle output locks the gear down");
    }
}
