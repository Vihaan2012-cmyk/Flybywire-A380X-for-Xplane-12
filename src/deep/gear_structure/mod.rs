//! Landing gear structure: per-leg oleo-pneumatic struts with CS-25-derived
//! limit/ultimate loads, overload/fatigue/collapse (`strut`); retraction,
//! doors, uplocks/downlocks and gravity extension (`retraction`); per-wheel
//! carbon brakes, antiskid and the parking brake accumulator (`brakes`);
//! nosewheel/body-gear steering and shimmy (`steering`); tailstrike
//! geometry, wing fatigue and hard-landing reporting (`structure`); and the
//! ECAM/failure/component registration (`registry`).
//!
//! Layout: nose gear (2 wheels, unbraked, steerable), 2 wing gears (4
//! wheels each, all braked) and 2 body gears (6 wheels each: forward/middle
//! axle 4 wheels braked, rear axle 2 wheels steerable and unbraked) --
//! 22 wheels total, 16 braked, matching the brief and the plugin's existing
//! `physics::tyre::Tyres`'s own 16-wheel layout (`LEG_WHEEL_INDICES` below
//! reproduces that exact grouping/ordering so the two can be lined up
//! wheel-for-wheel later).
//!
//! Self-contained per the workstream rules: nothing here reads X-Plane or
//! any other crate module. `GearSystemInputs`/`GearSystemFaults` are the
//! full input surface; whoever wires this into the plugin supplies them
//! from the real sim state and the failure catalogue.

pub mod brakes;
pub mod registry;
pub mod retraction;
pub mod steering;
pub mod strut;
pub mod structure;

/// Standard gravity, m/s^2 (exact SI definition).
pub const G_MS2: f64 = 9.806_65;
/// A380 MLW, kg. Airbus "A380 Aircraft Characteristics - Airport and
/// Maintenance Planning" (WV000 variant), matching
/// `physics::damage.rs`'s own `MLW_KG` citation.
pub const MLW_KG: f64 = 386_000.0;
/// A380 MTOW, kg, same source (WV000 variant).
pub const MTOW_KG: f64 = 510_000.0;

/// The three structurally distinct leg types (geometry/servicing constants
/// live on this in `strut.rs`/`retraction.rs`/`steering.rs`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LegKind {
    Nose,
    Wing,
    Body,
}

/// The plugin's existing `physics::tyre::Tyres`/`physics::damage::Damage`
/// wheel grouping and ordering (left wing, right wing, left body, right
/// body -- the nose has no entry, matching those modules), reproduced here
/// so this workstream's 16 `brakes::BrakeWheel`s line up wheel-for-wheel
/// with that existing per-wheel state without either module depending on
/// the other.
pub const LEG_WHEEL_INDICES: [[usize; 4]; 4] = [
    [0, 1, 4, 5],
    [2, 3, 6, 7],
    [8, 9, 12, 13],
    [10, 11, 14, 15],
];

/// Faults for one leg's strut and retraction system together.
#[derive(Clone, Copy, Debug, Default)]
pub struct LegFaults {
    pub strut: strut::StrutFaults,
    pub retraction: retraction::RetractionFaults,
}

pub struct GearSystemFaults {
    pub nose: LegFaults,
    pub left_wing: LegFaults,
    pub right_wing: LegFaults,
    pub left_body: LegFaults,
    pub right_body: LegFaults,
    pub nose_steering: steering::SteeringFaults,
    pub left_body_steering: steering::SteeringFaults,
    pub right_body_steering: steering::SteeringFaults,
    pub wheel_brakes: [brakes::BrakeFaults; 16],
    pub parking_brake: brakes::ParkingBrakeFaults,
}

impl Default for GearSystemFaults {
    fn default() -> Self {
        Self {
            nose: LegFaults::default(),
            left_wing: LegFaults::default(),
            right_wing: LegFaults::default(),
            left_body: LegFaults::default(),
            right_body: LegFaults::default(),
            nose_steering: steering::SteeringFaults::default(),
            left_body_steering: steering::SteeringFaults::default(),
            right_body_steering: steering::SteeringFaults::default(),
            wheel_brakes: std::array::from_fn(|_| brakes::BrakeFaults::default()),
            parking_brake: brakes::ParkingBrakeFaults::default(),
        }
    }
}

/// One leg's own touchdown/ground-contact state and lateral loading, so
/// each of the five legs can touch down at its own moment (rather than the
/// four main legs sharing one flag) and can carry its own side load
/// (CS-25.485 -- a crosswind/drift landing or a hard turn loads each leg's
/// axle differently, e.g. more on the leg toward the turn/into the wind).
#[derive(Clone, Copy, Debug, Default)]
pub struct LegTouchdownInputs {
    pub on_ground: bool,
    /// Vertical closing speed at the instant *this* leg's wheels touch, m/s
    /// (only read on the tick `on_ground` first becomes true).
    pub sink_speed_ms: f64,
    /// Lateral load applied at this leg's axle this tick, N.
    pub side_load_n: f64,
}

/// One tick's inputs to the whole gear system.
pub struct GearSystemInputs {
    pub mass_kg: f64,
    pub pitch_deg: f64,
    pub groundspeed_ms: f64,
    pub ambient_c: f64,

    /// Each leg's own ground contact, touchdown sink speed and side load --
    /// independent per leg, so e.g. one main gear touching before the
    /// other in a one-wheel-low crosswind landing, or the nose settling
    /// after the mains, is a real timing difference the caller can supply
    /// rather than one shared flag forcing them all to touch together.
    pub nose: LegTouchdownInputs,
    pub left_wing: LegTouchdownInputs,
    pub right_wing: LegTouchdownInputs,
    pub left_body: LegTouchdownInputs,
    pub right_body: LegTouchdownInputs,

    pub gear_lever_down: bool,
    pub gravity_extend_commanded: bool,
    /// Green hydraulic system pressure fraction, drives the wing legs'
    /// actuators (A380: green/yellow, 5000 psi nominal).
    pub green_hydraulic_fraction: f64,
    /// Yellow hydraulic system pressure fraction, drives the nose and body
    /// legs' actuators.
    pub yellow_hydraulic_fraction: f64,

    pub nose_steering_command_deg: f64,
    /// Left-side brake pedal/autobrake demand (left wing + left body
    /// wheels), 0..1.
    pub brake_pedal_left: f64,
    /// Right-side brake pedal/autobrake demand (right wing + right body
    /// wheels), 0..1.
    pub brake_pedal_right: f64,
    pub parking_brake_set: bool,

    pub dt_s: f64,
}

/// One leg's collapse state and load, the per-leg output the flight model
/// consumes (brief item 5).
#[derive(Clone, Copy, Debug, Default)]
pub struct LegOutput {
    pub force_n: f64,
    pub compression_frac: f64,
    pub collapsed: bool,
    pub gear_position: f64,
    pub uplocked: bool,
    pub downlocked: bool,
    pub stuck_locked: bool,
}

pub struct GearSystemOutputs {
    pub nose: LegOutput,
    pub left_wing: LegOutput,
    pub right_wing: LegOutput,
    pub left_body: LegOutput,
    pub right_body: LegOutput,
    pub brake_wheel_temps_c: [f64; 16],
    pub brake_wheel_fire: [bool; 16],
    pub parking_brake_pressure_pa: f64,
    pub parking_brake_holding: bool,
    pub wing_fatigue_index: f64,
    pub nose_wheel_angle_deg: f64,
}

pub struct GearSystem {
    pub nose_strut: strut::Strut,
    pub nose_retraction: retraction::Retraction,
    pub nose_steering: steering::SteeringActuator,

    pub left_wing_strut: strut::Strut,
    pub left_wing_retraction: retraction::Retraction,
    pub right_wing_strut: strut::Strut,
    pub right_wing_retraction: retraction::Retraction,

    pub left_body_strut: strut::Strut,
    pub left_body_retraction: retraction::Retraction,
    pub left_body_steering: steering::SteeringActuator,
    pub right_body_strut: strut::Strut,
    pub right_body_retraction: retraction::Retraction,
    pub right_body_steering: steering::SteeringActuator,

    pub brake_wheels: [brakes::BrakeWheel; 16],
    pub parking_brake: brakes::ParkingBrakeAccumulator,
    pub wing_fatigue: structure::WingFatigueTracker,

    /// This tick's log lines (cleared and refilled every `step`, the same
    /// convention `physics::damage::Damage::events` uses).
    pub events: Vec<String>,
}

impl GearSystem {
    pub fn new() -> Self {
        Self {
            nose_strut: strut::Strut::new(LegKind::Nose),
            nose_retraction: retraction::Retraction::new(LegKind::Nose),
            nose_steering: steering::SteeringActuator::new_nose(),

            left_wing_strut: strut::Strut::new(LegKind::Wing),
            left_wing_retraction: retraction::Retraction::new(LegKind::Wing),
            right_wing_strut: strut::Strut::new(LegKind::Wing),
            right_wing_retraction: retraction::Retraction::new(LegKind::Wing),

            left_body_strut: strut::Strut::new(LegKind::Body),
            left_body_retraction: retraction::Retraction::new(LegKind::Body),
            left_body_steering: steering::SteeringActuator::new_body(),
            right_body_strut: strut::Strut::new(LegKind::Body),
            right_body_retraction: retraction::Retraction::new(LegKind::Body),
            right_body_steering: steering::SteeringActuator::new_body(),

            brake_wheels: std::array::from_fn(|_| brakes::BrakeWheel::new(15.0)),
            parking_brake: brakes::ParkingBrakeAccumulator::new(),
            wing_fatigue: structure::WingFatigueTracker::new(),
            events: Vec::new(),
        }
    }

    #[allow(clippy::too_many_arguments)]
    fn step_leg(
        strut: &mut strut::Strut,
        retraction: &mut retraction::Retraction,
        kind: LegKind,
        touchdown: &LegTouchdownInputs,
        mass_kg: f64,
        gear_lever_down: bool,
        gravity_extend_commanded: bool,
        hydraulic_pressure_fraction: f64,
        leg_faults: &LegFaults,
        dt_s: f64,
    ) -> (strut::StrutOutputs, retraction::RetractionOutputs) {
        let retraction_inputs = retraction::RetractionInputs { gear_lever_down, gravity_extend_commanded, hydraulic_pressure_fraction, dt_s };
        let r_out = retraction.step(&retraction_inputs, &leg_faults.retraction);

        // The strut only reacts a load if the gear is actually extended
        // (a belly-up landing with the gear retracted is out of this
        // model's scope, per `GearSystemInputs`' doc comment).
        let strut_on_ground = touchdown.on_ground && r_out.gear_position > 0.9;
        let load_n = if strut_on_ground { mass_kg * G_MS2 * kind.static_fraction() } else { 0.0 };
        let strut_inputs =
            strut::StrutInputs { on_ground: strut_on_ground, sink_speed_ms: touchdown.sink_speed_ms, load_n, side_load_n: touchdown.side_load_n, locked_down: r_out.downlocked, dt_s };
        let s_out = strut.step(&strut_inputs, &leg_faults.strut);
        (s_out, r_out)
    }

    fn leg_output(s: &strut::StrutOutputs, r: &retraction::RetractionOutputs) -> LegOutput {
        LegOutput { force_n: s.force_n, compression_frac: s.compression_frac, collapsed: s.collapsed, gear_position: r.gear_position, uplocked: r.uplocked, downlocked: r.downlocked, stuck_locked: r.stuck_locked }
    }

    fn note_leg_events(&mut self, name: &str, s: &strut::StrutOutputs, r: &retraction::RetractionOutputs) {
        if s.overload_event {
            self.events.push(format!("{name} gear leg overload: seal damage accumulating"));
        }
        if r.stuck_locked {
            self.events.push(format!("{name} gear will not release from its uplock"));
        }
        if r.phase == retraction::Phase::Locked && r.gear_position > 0.95 && !r.downlocked {
            self.events.push(format!("{name} gear down but not locked"));
        }
    }

    pub fn step(&mut self, inputs: &GearSystemInputs, faults: &GearSystemFaults) -> GearSystemOutputs {
        self.events.clear();
        let dt = inputs.dt_s.max(0.0);

        let (nose_s, nose_r) = Self::step_leg(
            &mut self.nose_strut,
            &mut self.nose_retraction,
            LegKind::Nose,
            &inputs.nose,
            inputs.mass_kg,
            inputs.gear_lever_down,
            inputs.gravity_extend_commanded,
            inputs.yellow_hydraulic_fraction,
            &faults.nose,
            dt,
        );
        self.note_leg_events("nose", &nose_s, &nose_r);

        let (lw_s, lw_r) = Self::step_leg(
            &mut self.left_wing_strut,
            &mut self.left_wing_retraction,
            LegKind::Wing,
            &inputs.left_wing,
            inputs.mass_kg,
            inputs.gear_lever_down,
            inputs.gravity_extend_commanded,
            inputs.green_hydraulic_fraction,
            &faults.left_wing,
            dt,
        );
        self.note_leg_events("left wing", &lw_s, &lw_r);
        if lw_s.cycle_completed {
            self.wing_fatigue.note_wing_leg_cycle(lw_s.peak_force_last_cycle_n, LegKind::Wing.static_fraction() * MLW_KG * G_MS2);
        }

        let (rw_s, rw_r) = Self::step_leg(
            &mut self.right_wing_strut,
            &mut self.right_wing_retraction,
            LegKind::Wing,
            &inputs.right_wing,
            inputs.mass_kg,
            inputs.gear_lever_down,
            inputs.gravity_extend_commanded,
            inputs.green_hydraulic_fraction,
            &faults.right_wing,
            dt,
        );
        self.note_leg_events("right wing", &rw_s, &rw_r);
        if rw_s.cycle_completed {
            self.wing_fatigue.note_wing_leg_cycle(rw_s.peak_force_last_cycle_n, LegKind::Wing.static_fraction() * MLW_KG * G_MS2);
        }

        let (lb_s, lb_r) = Self::step_leg(
            &mut self.left_body_strut,
            &mut self.left_body_retraction,
            LegKind::Body,
            &inputs.left_body,
            inputs.mass_kg,
            inputs.gear_lever_down,
            inputs.gravity_extend_commanded,
            inputs.yellow_hydraulic_fraction,
            &faults.left_body,
            dt,
        );
        self.note_leg_events("left body", &lb_s, &lb_r);

        let (rb_s, rb_r) = Self::step_leg(
            &mut self.right_body_strut,
            &mut self.right_body_retraction,
            LegKind::Body,
            &inputs.right_body,
            inputs.mass_kg,
            inputs.gear_lever_down,
            inputs.gravity_extend_commanded,
            inputs.yellow_hydraulic_fraction,
            &faults.right_body,
            dt,
        );
        self.note_leg_events("right body", &rb_s, &rb_r);

        // Steering.
        let nose_steer_in = steering::SteeringInputs { commanded_angle_deg: inputs.nose_steering_command_deg, groundspeed_ms: inputs.groundspeed_ms, dt_s: dt };
        let nose_steer_out = self.nose_steering.step(&nose_steer_in, &faults.nose_steering);
        if nose_steer_out.shimmy_unstable {
            self.events.push("nose gear shimmy tendency active".to_string());
        }
        let body_command = steering::body_steering_angle_deg(nose_steer_out.base_angle_deg, inputs.groundspeed_ms);
        let lb_steer_in = steering::SteeringInputs { commanded_angle_deg: body_command, groundspeed_ms: inputs.groundspeed_ms, dt_s: dt };
        self.left_body_steering.step(&lb_steer_in, &faults.left_body_steering);
        let rb_steer_in = steering::SteeringInputs { commanded_angle_deg: body_command, groundspeed_ms: inputs.groundspeed_ms, dt_s: dt };
        self.right_body_steering.step(&rb_steer_in, &faults.right_body_steering);

        // Brakes: 16 wheels, `LEG_WHEEL_INDICES`' layout (even group index
        // = left, odd = right). Each wheel's normal load is its owning
        // leg's own strut reaction (already computed above) divided by
        // that leg's *total* wheel count (4 for a wing leg, 6 for a body
        // leg, since the body bogie's unbraked rear axle still carries its
        // share of the leg's weight even though it has no `BrakeWheel`
        // here) -- wheel speed is no longer a caller-supplied input at all
        // (`brakes.rs` now owns that state itself).
        let leg_force_n = [lw_s.force_n, rw_s.force_n, lb_s.force_n, rb_s.force_n];
        let leg_on_ground = [inputs.left_wing.on_ground, inputs.right_wing.on_ground, inputs.left_body.on_ground, inputs.right_body.on_ground];
        let leg_wheel_count = [4.0_f64, 4.0, 6.0, 6.0];
        let mut brake_temps = [0.0_f64; 16];
        let mut brake_fire = [false; 16];
        for (leg, wheels) in LEG_WHEEL_INDICES.iter().enumerate() {
            let commanded = if leg % 2 == 0 { inputs.brake_pedal_left } else { inputs.brake_pedal_right };
            let on_ground = leg_on_ground[leg];
            let normal_load_n = if on_ground { (leg_force_n[leg] / leg_wheel_count[leg]).max(0.0) } else { 0.0 };
            for &wheel in wheels {
                let wi = brakes::BrakeWheelInputs { commanded, on_ground, normal_load_n, groundspeed_ms: inputs.groundspeed_ms, ambient_c: inputs.ambient_c, dt_s: dt };
                let out = self.brake_wheels[wheel].step(&wi, &faults.wheel_brakes[wheel]);
                brake_temps[wheel] = out.stack_temp_c;
                if out.fire {
                    brake_fire[wheel] = true;
                    self.events.push(format!("wheel {} brake fire", wheel + 1));
                }
            }
        }

        let (parking_pressure_pa, parking_holding) = self.parking_brake.step(inputs.parking_brake_set, &faults.parking_brake, dt);
        if inputs.parking_brake_set && !parking_holding {
            self.events.push("parking brake accumulator pressure too low to hold".to_string());
        }

        GearSystemOutputs {
            nose: Self::leg_output(&nose_s, &nose_r),
            left_wing: Self::leg_output(&lw_s, &lw_r),
            right_wing: Self::leg_output(&rw_s, &rw_r),
            left_body: Self::leg_output(&lb_s, &lb_r),
            right_body: Self::leg_output(&rb_s, &rb_r),
            brake_wheel_temps_c: brake_temps,
            brake_wheel_fire: brake_fire,
            parking_brake_pressure_pa: parking_pressure_pa,
            parking_brake_holding: parking_holding,
            wing_fatigue_index: self.wing_fatigue.fatigue_index,
            nose_wheel_angle_deg: nose_steer_out.angle_deg,
        }
    }

    /// Builds a `structure::HardLandingReport` from the legs' current state
    /// -- call right after a touchdown (e.g. when any leg's strut output
    /// reports `cycle_completed` for the first time after a period
    /// airborne), not every tick.
    pub fn hard_landing_report(&self, mass_kg: f64, pitch_deg: f64) -> structure::HardLandingReport {
        let leg = |name: &'static str, s: &strut::Strut| {
            let force_n = s.current_force_n();
            structure::LegLoadSummary { name, peak_force_n: force_n, utilization: force_n / s.limit_load_n.max(1.0), overload: force_n > s.limit_load_n, collapsed: s.collapsed }
        };
        let legs = vec![
            leg("nose", &self.nose_strut),
            leg("left wing", &self.left_wing_strut),
            leg("right wing", &self.right_wing_strut),
            leg("left body", &self.left_body_strut),
            leg("right body", &self.right_body_strut),
        ];
        structure::build_hard_landing_report(legs, mass_kg, pitch_deg, self.left_body_strut.compression_frac_now())
    }
}

impl Default for GearSystem {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn grounded(sink_speed_ms: f64) -> LegTouchdownInputs {
        LegTouchdownInputs { on_ground: true, sink_speed_ms, side_load_n: 0.0 }
    }
    fn airborne() -> LegTouchdownInputs {
        LegTouchdownInputs { on_ground: false, sink_speed_ms: 0.0, side_load_n: 0.0 }
    }

    fn healthy_inputs(dt_s: f64) -> GearSystemInputs {
        GearSystemInputs {
            mass_kg: MLW_KG,
            pitch_deg: 2.0,
            groundspeed_ms: 0.0,
            ambient_c: 15.0,
            nose: grounded(0.0),
            left_wing: grounded(0.0),
            right_wing: grounded(0.0),
            left_body: grounded(0.0),
            right_body: grounded(0.0),
            gear_lever_down: true,
            gravity_extend_commanded: false,
            green_hydraulic_fraction: 1.0,
            yellow_hydraulic_fraction: 1.0,
            nose_steering_command_deg: 0.0,
            brake_pedal_left: 0.0,
            brake_pedal_right: 0.0,
            parking_brake_set: false,
            dt_s,
        }
    }

    #[test]
    fn a_healthy_system_settles_on_the_ground_with_no_events() {
        let mut gs = GearSystem::new();
        let faults = GearSystemFaults::default();
        let inputs = healthy_inputs(0.1);
        let mut out = gs.step(&inputs, &faults);
        for _ in 0..200 {
            out = gs.step(&inputs, &faults);
        }
        assert!(!out.left_wing.collapsed && !out.right_body.collapsed);
        assert!(out.left_wing.force_n > 0.0);
        assert!(gs.events.is_empty(), "a healthy, steady-state system must not log any events: {:?}", gs.events);
    }

    #[test]
    fn a_hard_landing_collapses_a_wing_leg_and_the_output_says_so() {
        let mut gs = GearSystem::new();
        let faults = GearSystemFaults::default();
        let mut inputs = healthy_inputs(0.001);
        inputs.nose = airborne();
        inputs.left_wing = airborne();
        inputs.right_wing = airborne();
        inputs.left_body = airborne();
        inputs.right_body = airborne();
        // Airborne settle first, then a very hard touchdown.
        let mut out = gs.step(&inputs, &faults);
        for _ in 0..10 {
            out = gs.step(&inputs, &faults);
        }
        let touchdown = grounded(12.0); // well past the certified limit sink speed
        inputs.left_wing = touchdown;
        inputs.right_wing = touchdown;
        inputs.left_body = touchdown;
        inputs.right_body = touchdown;
        for _ in 0..5_000 {
            out = gs.step(&inputs, &faults);
        }
        assert!(out.left_wing.collapsed || out.right_wing.collapsed || out.left_body.collapsed || out.right_body.collapsed, "an extreme sink speed must collapse at least one main leg");
    }

    #[test]
    fn an_uplock_jam_only_bites_when_re_extending_and_is_reported_as_stuck() {
        let mut gs = GearSystem::new();
        // The jam does not stop the leg reaching the uplock in the first
        // place (retracting overcomes the downlock, not the uplock) -- it
        // only stops that leg *leaving* the uplock afterwards, so retract
        // with the fault already present, then try to extend again.
        let mut faults = GearSystemFaults::default();
        faults.left_wing.retraction.uplock_jam = 0.9;
        let mut inputs = healthy_inputs(0.1);
        inputs.gear_lever_down = false;
        let mut out = gs.step(&inputs, &faults);
        for _ in 0..1_000 {
            out = gs.step(&inputs, &faults);
        }
        assert!(out.left_wing.uplocked && out.right_wing.uplocked, "both wing legs must reach the uplock; the jam does not block retraction");

        inputs.gear_lever_down = true;
        for _ in 0..1_000 {
            out = gs.step(&inputs, &faults);
        }
        assert!(out.left_wing.stuck_locked, "the jammed leg must fail to release from its uplock");
        assert!(out.right_wing.gear_position > 0.95, "the healthy wing leg should have extended normally");
    }

    #[test]
    fn braking_heats_the_correct_side_wheels_only() {
        let mut gs = GearSystem::new();
        let faults = GearSystemFaults::default();
        let mut inputs = healthy_inputs(1.0);
        inputs.groundspeed_ms = 40.0;
        inputs.brake_pedal_left = 1.0;
        inputs.brake_pedal_right = 0.0;
        let out = healthy_run(&mut gs, &inputs, &faults, 60);
        for &w in &LEG_WHEEL_INDICES[0] {
            assert!(out.brake_wheel_temps_c[w] > 15.0, "left wing wheel {w} should have heated");
        }
        for &w in &LEG_WHEEL_INDICES[1] {
            assert!((out.brake_wheel_temps_c[w] - 15.0).abs() < 1.0, "right wing wheel {w} should not have heated");
        }
    }

    fn healthy_run(gs: &mut GearSystem, inputs: &GearSystemInputs, faults: &GearSystemFaults, ticks: u32) -> GearSystemOutputs {
        let mut out = gs.step(inputs, faults);
        for _ in 0..ticks {
            out = gs.step(inputs, faults);
        }
        out
    }

    #[test]
    fn a_hard_landing_report_carries_the_real_per_leg_peak_force_and_overload() {
        let mut gs = GearSystem::new();
        let faults = GearSystemFaults::default();
        let mut inputs = healthy_inputs(0.001);
        inputs.nose = airborne();
        inputs.left_wing = airborne();
        inputs.right_wing = airborne();
        inputs.left_body = airborne();
        inputs.right_body = airborne();
        for _ in 0..10 {
            gs.step(&inputs, &faults);
        }
        // A firm but sub-collapse touchdown: past the certified limit sink
        // speed (so the wing/body legs overload) but well under the 2.2x
        // that collapses a leg outright (see the collapse test above), so
        // the report can show a real overloaded-but-intact leg.
        let touchdown = grounded(3.05 * 1.4);
        inputs.left_wing = touchdown;
        inputs.right_wing = touchdown;
        inputs.left_body = touchdown;
        inputs.right_body = touchdown;
        let mut peak_utilization = 0.0_f64;
        for _ in 0..3_000 {
            gs.step(&inputs, &faults);
            let report = gs.hard_landing_report(inputs.mass_kg, inputs.pitch_deg);
            for leg in &report.legs {
                peak_utilization = peak_utilization.max(leg.utilization);
            }
        }
        let report = gs.hard_landing_report(inputs.mass_kg, inputs.pitch_deg);
        assert!(!report.legs.is_empty());
        for leg in &report.legs {
            assert!(leg.peak_force_n.is_finite());
            // Every reported leg must carry the strut's own real force, not
            // a hardcoded placeholder -- either it is genuinely near zero
            // (nose, unloaded here) or it reflects real load-bearing.
        }
        let main_legs_overloaded = report.legs.iter().any(|l| l.name != "nose" && l.overload);
        assert!(main_legs_overloaded || peak_utilization > 1.0, "a sink speed past the certified limit must show a real overload on at least one main leg, not a hardcoded zero (peak utilisation seen: {peak_utilization})");
        for leg in &report.legs {
            if leg.name != "nose" {
                assert!(leg.peak_force_n > 0.0, "{}'s reported peak force must be the real, nonzero load it is carrying", leg.name);
            }
        }
        assert!(!report.legs.iter().any(|l| l.collapsed), "this sink speed must stay under ultimate: a collapse would invalidate the overload-without-collapse premise of this test");
    }

    #[test]
    fn each_leg_touches_down_independently_instead_of_sharing_one_flag() {
        // A one-wheel-low crosswind-style landing: the right wing gear
        // touches down hard while the left wing (and everything else)
        // stays airborne. With a single shared `main_on_ground` flag this
        // could not be expressed at all; per-leg `LegTouchdownInputs` lets
        // each leg's own strut react only to its own real contact.
        let mut gs = GearSystem::new();
        let faults = GearSystemFaults::default();
        let mut inputs = healthy_inputs(0.001);
        inputs.nose = airborne();
        inputs.left_wing = airborne();
        inputs.right_wing = airborne();
        inputs.left_body = airborne();
        inputs.right_body = airborne();
        let mut out = gs.step(&inputs, &faults);
        for _ in 0..10 {
            out = gs.step(&inputs, &faults);
        }
        inputs.right_wing = grounded(3.05);
        for _ in 0..2_000 {
            out = gs.step(&inputs, &faults);
        }
        assert!(out.right_wing.force_n > 0.0, "the leg that actually touched down must carry a real load");
        assert_eq!(out.left_wing.force_n, 0.0, "a leg that never touched down must carry none, independent of its sibling");
        assert_eq!(out.left_wing.compression_frac, 0.0);
    }

    #[test]
    fn a_side_load_alone_can_overload_a_leg_via_the_cs_25_485_lateral_path() {
        // CS-25.485's 0.8x-of-vertical-limit side load factor: a large
        // enough lateral load by itself (independent of any vertical sink
        // speed) must be able to push a grounded, otherwise gently loaded
        // leg into an overload/collapse, now that `side_load_n` is
        // threaded through from `GearSystemInputs` instead of always being
        // zero.
        let mut gs = GearSystem::new();
        let faults = GearSystemFaults::default();
        let mut inputs = healthy_inputs(0.001);
        // A side load well past the vertical limit load itself: since the
        // lateral limit is only 0.8x the vertical one, this comfortably
        // exceeds both the lateral limit and ultimate.
        let extreme_side_load = gs.left_body_strut.limit_load_n * 3.0;
        inputs.left_body.side_load_n = extreme_side_load;
        let mut out = gs.step(&inputs, &faults);
        for _ in 0..50 {
            out = gs.step(&inputs, &faults);
        }
        assert!(out.left_body.collapsed, "an extreme side load alone must be able to collapse a leg via the lateral load path");
        assert!(!out.right_body.collapsed, "an untouched sibling leg with zero side load must be unaffected");
    }

    #[test]
    fn numerically_safe_at_rest_and_dt_zero() {
        let mut gs = GearSystem::new();
        let faults = GearSystemFaults::default();
        let inputs = healthy_inputs(0.0);
        let out = gs.step(&inputs, &faults);
        assert!(out.left_wing.force_n.is_finite());
        assert!(out.parking_brake_pressure_pa.is_finite());
        assert!(out.nose_wheel_angle_deg.is_finite());
    }
}
