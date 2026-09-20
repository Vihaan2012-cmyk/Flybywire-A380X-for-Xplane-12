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
//! ## What is not in `Truth` yet
//!
//! Everything the gear needs that is *about the airframe's motion* rather
//! than about its systems: aircraft mass, pitch attitude, groundspeed,
//! per-leg touchdown sink speed and side load, and the four cockpit
//! controls the gear obeys (lever, gravity extension, tiller, pedals,
//! parking brake). They are collected in [`GearCommands`], documented one
//! by one, rather than guessed from `Truth::on_ground` and
//! `Truth::altitude_ft`. Mass is the one that cannot honestly default to
//! zero -- an aircraft always weighs something -- so it defaults to the
//! published A380-800 operating empty weight, i.e. the airframe with no
//! fuel and no payload, which is exactly the state
//! `deep::fuel::live`'s empty tanks describe.

use crate::deep::api::Registry;
use crate::deep::live::{Faults, Truth};

use super::brakes::{BrakeFaults, ParkingBrakeFaults};
use super::retraction::RetractionFaults;
use super::steering::SteeringFaults;
use super::strut::StrutFaults;
use super::{GearSystem, GearSystemFaults, GearSystemInputs, GearSystemOutputs, LegFaults, LegTouchdownInputs};

/// A380-800 operating empty weight, kg. Airbus's own published Aircraft
/// Characteristics figure is about 277 t for the -800; this is the
/// airframe with neither fuel nor payload, which is what the live systems
/// describe before the plugin tells them otherwise.
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

/// Everything the gear needs that is not in [`Truth`]: the airframe's own
/// motion, and the cockpit controls the gear obeys.
#[derive(Clone, Copy, Debug)]
pub struct GearCommands {
    /// Current all-up mass, kg -- sets every leg's static reaction and the
    /// overweight-landing check. Defaults to [`OEW_KG`].
    pub mass_kg: f64,
    /// Body pitch attitude, deg -- the tailstrike margin's other input.
    pub pitch_deg: f64,
    /// Groundspeed, m/s. Not `Truth::environment.tas_ms`: shimmy and brake
    /// energy are about speed over the ground, and the two differ by the
    /// wind.
    pub groundspeed_ms: f64,
    /// Per leg, in `LEG_KEY` order: whether *this* leg's wheels are on the
    /// ground, the vertical closing speed at the instant they touch, and
    /// the side load at its axle. `Truth::on_ground` is one aircraft-wide
    /// flag, so it cannot express one main gear touching before the other,
    /// and it carries no sink speed at all -- which is the single number a
    /// hard-landing model is most sensitive to.
    pub leg_on_ground: [bool; N_LEGS],
    pub leg_sink_speed_ms: [f64; N_LEGS],
    pub leg_side_load_n: [f64; N_LEGS],
    /// Gear lever down, and the free-fall/gravity extension handle.
    pub gear_lever_down: bool,
    pub gravity_extend_commanded: bool,
    /// Nosewheel tiller/rudder-pedal steering command, deg.
    pub nose_steering_command_deg: f64,
    /// Brake pedal deflection, 0..1 (or the autobrake's demand).
    pub brake_pedal_left: f64,
    pub brake_pedal_right: f64,
    pub parking_brake_set: bool,
}

impl Default for GearCommands {
    /// A parked aircraft: empty, stopped, gear down and locked, brakes
    /// released.
    fn default() -> Self {
        Self {
            mass_kg: OEW_KG,
            pitch_deg: 0.0,
            groundspeed_ms: 0.0,
            leg_on_ground: [true; N_LEGS],
            leg_sink_speed_ms: [0.0; N_LEGS],
            leg_side_load_n: [0.0; N_LEGS],
            gear_lever_down: true,
            gravity_extend_commanded: false,
            nose_steering_command_deg: 0.0,
            brake_pedal_left: 0.0,
            brake_pedal_right: 0.0,
            parking_brake_set: false,
        }
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
        let mut actuator_leak = [0u64; N_LEGS];
        let mut uplock_jam = [0u64; N_LEGS];
        let mut downlock_fail = [0u64; N_LEGS];
        let mut door_jam = [0u64; N_LEGS];
        let mut sensor_lies = [0u64; N_LEGS];
        for (i, key) in LEG_KEY.iter().enumerate() {
            let strut = fids(&reg, &format!("32_gear.{key}_strut"));
            assert_eq!(strut.len(), 2, "each strut registers a gas and an oil leak");
            strut_gas_leak[i] = strut[0];
            strut_oil_leak[i] = strut[1];

            let retraction = fids(&reg, &format!("32_gear.{key}_retraction"));
            assert_eq!(retraction.len(), 5, "each retraction chain registers five failures");
            actuator_leak[i] = retraction[0];
            uplock_jam[i] = retraction[1];
            downlock_fail[i] = retraction[2];
            door_jam[i] = retraction[3];
            sensor_lies[i] = retraction[4];
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
        for (i, key) in STEER_KEY.iter().enumerate() {
            let steer = fids(&reg, &format!("32_gear.{key}_steering"));
            assert_eq!(steer.len(), 2, "each steerable position registers a shimmy and an actuator failure");
            shimmy[i] = steer[0];
            steer_actuator_leak[i] = steer[1];
        }

        let parking = fids(&reg, "32_gear.parking_brake_accumulator");
        assert_eq!(parking.len(), 1);

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
    /// Inputs `Truth` does not carry; see [`GearCommands`].
    pub commands: GearCommands,
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
        Self { ids: Ids::resolve(), system, outputs, antiskid_channel_fault: [false; N_BRAKED_WHEELS], commands }
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
            strut: StrutFaults { gas_leak: faults.get(self.ids.strut_gas_leak[i]), oil_leak: faults.get(self.ids.strut_oil_leak[i]) },
            retraction: RetractionFaults {
                actuator_leak: faults.get(self.ids.actuator_leak[i]),
                uplock_jam: faults.get(self.ids.uplock_jam[i]),
                downlock_fail: faults.get(self.ids.downlock_fail[i]),
                door_jam: faults.get(self.ids.door_jam[i]),
                sensor_lies: faults.get(self.ids.sensor_lies[i]),
            },
        };
        let steer = |i: usize| SteeringFaults { shimmy_damper_fail: faults.get(self.ids.shimmy[i]), actuator_leak: faults.get(self.ids.steer_actuator_leak[i]) };
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

/// One frame's `GearSystemInputs` from what `Truth` carries plus what it
/// does not.
fn inputs_from(truth: &Truth, commands: &GearCommands) -> GearSystemInputs {
    let leg = |i: usize| LegTouchdownInputs {
        // `Truth::on_ground` is the aircraft-wide weight-on-wheels flag:
        // no leg can be on the ground while the aircraft is not, so the
        // two are ANDed rather than the per-leg flag simply overriding it.
        on_ground: truth.on_ground && commands.leg_on_ground[i],
        sink_speed_ms: commands.leg_sink_speed_ms[i],
        side_load_n: commands.leg_side_load_n[i],
    };
    let fraction = |pa: f64| (pa / HYDRAULIC_NOMINAL_PA).clamp(0.0, 1.0);
    GearSystemInputs {
        mass_kg: commands.mass_kg,
        pitch_deg: commands.pitch_deg,
        groundspeed_ms: commands.groundspeed_ms,
        ambient_c: truth.environment.sat_c,
        nose: leg(0),
        left_wing: leg(1),
        right_wing: leg(2),
        left_body: leg(3),
        right_body: leg(4),
        gear_lever_down: commands.gear_lever_down,
        gravity_extend_commanded: commands.gravity_extend_commanded,
        green_hydraulic_fraction: fraction(truth.hydraulic_pressure_pa[0]),
        yellow_hydraulic_fraction: fraction(truth.hydraulic_pressure_pa[1]),
        nose_steering_command_deg: commands.nose_steering_command_deg,
        brake_pedal_left: commands.brake_pedal_left,
        brake_pedal_right: commands.brake_pedal_right,
        parking_brake_set: commands.parking_brake_set,
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

        // The antiskid computer's own BITE: a channel that has lost this
        // much of its release authority fails its self-test and reports,
        // independently of whether a skid has happened yet.
        for wheel in 0..N_BRAKED_WHEELS {
            self.antiskid_channel_fault[wheel] = gear_faults.wheel_brakes[wheel].antiskid_inop >= ANTISKID_BITE_THRESHOLD;
        }
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
        }

        for wheel in 0..N_BRAKED_WHEELS {
            let n = wheel + 1;
            out(&format!("BRAKE_FIRE:{n}"), b(o.brake_wheel_fire[wheel]));
            out(&format!("BRAKE_STACK_TEMP_C:{n}"), o.brake_wheel_temps_c[wheel]);
            out(&format!("BRAKE_WEAR_FRACTION:{n}"), o.brake_wheel_wear_fraction[wheel]);
            out(&format!("BRAKE_SKIDDING:{n}"), b(o.brake_wheel_skidding[wheel]));
            out(&format!("ANTISKID_CHANNEL_FAULT:{n}"), b(self.antiskid_channel_fault[wheel]));
        }

        out("PARK_BRAKE_SET", b(self.commands.parking_brake_set));
        out("PARK_BRAKE_HOLDING", b(o.parking_brake_holding));
        out("PARK_BRAKE_PRESS_PA", o.parking_brake_pressure_pa);

        out("NW_STEER_ANGLE_DEG", o.nose_wheel_angle_deg);
        out("NW_STEER_SHIMMY_UNSTABLE", b(o.nose_steer_shimmy_unstable));
        for i in 0..2 {
            let n = i + 1;
            out(&format!("BODY_STEER_ANGLE_DEG:{n}"), o.body_steer_angle_deg[i]);
            out(&format!("BODY_STEER_SHIMMY_UNSTABLE:{n}"), b(o.body_steer_shimmy_unstable[i]));
        }

        // The lever position the gear system is actually acting on, so the
        // L/G GEAR NOT DOWNLOCKED procedure's "GEAR LEVER ... RECYCLE" line
        // reads back the same command the legs obeyed.
        out("GEAR_LEVER_POSITION_REQUEST", b(self.commands.gear_lever_down));

        out("GEAR_WING_FATIGUE_INDEX", o.wing_fatigue_index);
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

        let mut live = GearStructureLive::new();
        live.commands.groundspeed_ms = 15.0; // taxi speed
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

        let mut healthy = GearStructureLive::new();
        healthy.commands.parking_brake_set = true;
        let healthy_out = run(&mut healthy, &truth, &Faults::default(), 600.0);
        assert_eq!(healthy_out.get("PARK_BRAKE_HOLDING"), Some(&1.0), "a healthy accumulator still holds after ten minutes");

        let mut leaking = GearStructureLive::new();
        leaking.commands.parking_brake_set = true;
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

        let mut healthy = GearStructureLive::new();
        healthy.commands.groundspeed_ms = speed;
        healthy.commands.nose_steering_command_deg = 2.0;
        let healthy_out = run(&mut healthy, &truth, &Faults::default(), 20.0);
        assert_eq!(healthy_out.get("NW_STEER_SHIMMY_UNSTABLE"), Some(&0.0));

        let mut failed = GearStructureLive::new();
        failed.commands.groundspeed_ms = speed;
        failed.commands.nose_steering_command_deg = 2.0;
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

        let mut live = GearStructureLive::new();
        live.commands.groundspeed_ms = speed;
        live.commands.nose_steering_command_deg = 20.0;
        let id = live.ids.shimmy[1]; // left body gear
        let out = run(&mut live, &truth, &Faults::from_pairs([(id, 1.0)]), 20.0);
        assert_eq!(out.get("NW_STEER_SHIMMY_UNSTABLE"), Some(&0.0), "the nosewheel's own damper is healthy");
        assert_eq!(out.get("BODY_STEER_SHIMMY_UNSTABLE:1"), Some(&1.0));
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
    }
}
