//! A 6-DOF flight model for the emulator's offline test bench.
//!
//! `emulator/src/lib.rs`'s own module doc says plainly what this crate
//! does *not* run: X-Plane's flight model. Every failure that should show
//! up as yaw after an engine failure, a hard landing's gear loads, a
//! tailstrike on rotation, or reduced braking on a contaminated runway
//! needs *some* physics standing in for X-Plane to produce those
//! consequences causally -- this module is that physics, built from public
//! data only (`docs/deep/BRIEF.md`'s aircraft description, FlyByWire's own
//! MSFS `flight_model.cfg`, EASA.E.012) and standard flight-dynamics theory
//! (Stevens & Lewis; Raymer; Pacejka), never a scripted "if failure X then
//! yaw" shortcut.
//!
//! ## Layout
//! - [`math`]: `Vec3`/`Quat` (no external crate, per the workstream rules).
//! - [`geometry`]: airframe geometry and the empty-mass baseline, sourced
//!   from `flight_model.cfg` (every constant cites its line).
//! - [`atmosphere`]: ISA, wind, shear, gusts.
//! - [`mass`]: current mass/CG/inertia from the plugin's own published
//!   gross weight and CG.
//! - [`actuator`]: the shared rate-limited control-surface actuator with
//!   jam/runaway/float fault modes.
//! - [`aerodynamics`]: lift/drag/side-force build-up (wing/tail strips,
//!   controls, flaps/slats, spoilers, ground effect, icing/damage).
//! - [`propulsion`]: engine thrust as a force vector at its real position.
//! - [`landing_gear`]: per-leg spring-damper, tyre friction, braking,
//!   steering, contamination, collapse, tailstrike.
//! - [`rigid_body`]: the 6-DOF quaternion/RK4 integrator every other module
//!   feeds forces and moments into.
//! - [`registry`]: this area's failure/component registration
//!   (`docs/deep/BRIEF.md`'s "Registering failures, components and ECAM
//!   alerts").
//! - [`trim`]: Newton-Raphson solvers for steady level flight (alpha,
//!   elevator, thrust) and steady on-ground static equilibrium (pitch,
//!   height), against this same module's own force models.
//!
//! See `PROGRESS.md` in this directory for exactly which [`FlightModel`]
//! output maps to which `Emulator` setter -- the wiring this module itself
//! cannot do (the emulator's own `lib.rs` is outside this agent's
//! directory).

pub mod actuator;
pub mod aerodynamics;
pub mod atmosphere;
pub mod geometry;
pub mod landing_gear;
pub mod mass;
pub mod math;
pub mod propulsion;
pub mod registry;
pub mod rigid_body;
pub mod trim;

use aerodynamics::{AeroFaults, AeroFlightState, Aerodynamics, ControlInputs};
use atmosphere::Wind;
use geometry::{Engine, GEAR_LEGS};
use landing_gear::{GearFaults, GearInputs, LandingGear};
use math::Vec3;
use mass::MassProperties;
use propulsion::{EngineThrustInput, PropulsionOutputs};
use rigid_body::{RigidBody, RigidBodyState};

/// World-frame (steady) wind and its shear/gust shaping for one tick --
/// see `atmosphere::Wind`'s own doc for what each field means. Only the
/// gust phase needs to persist tick to tick; [`FlightModel`] keeps that
/// itself so the caller can just supply the same config every tick.
#[derive(Clone, Copy, Debug, Default)]
pub struct WindInput {
    pub steady_10m_m_s: Vec3,
    pub shear_exponent: f64,
    pub gust_peak_m_s: f64,
    pub gust_period_s: f64,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct FlightModelInputs {
    /// FlyByWire's own `"fbw/wb/gross_weight_kg"` (published by
    /// `src/weight_balance.rs`) -- see `mass.rs`'s module doc.
    pub mass_kg: f64,
    /// `"fbw/wb/cg_z_ft" * geometry::FT_TO_M`.
    pub cg_x_forward_m: f64,
    pub controls: ControlInputs,
    pub aero_faults: AeroFaults,
    /// Per engine (`geometry::ENGINES` order), from `physics::engine`'s
    /// `EngineOutputs::net_thrust_n` (see `propulsion.rs`'s module doc).
    pub engines: [EngineThrustInput; 4],
    pub gear_faults: GearFaults,
    pub brake_cmd: f64,
    pub nose_steering_cmd: f64,
    /// 1.0 = dry pavement; see `landing_gear.rs`'s module doc.
    pub runway_mu_factor: f64,
    /// 0 (retracted) .. 1 (extended) -- aerodynamic drag only; the actual
    /// gear-leg contact model in `landing_gear.rs` reacts to penetration
    /// regardless of this flag (a retracted gear should simply never be
    /// close enough to the ground to matter).
    pub gear_extended_fraction: f64,
    /// World-frame z (down-positive) of the ground/runway plane: for a
    /// field at `elevation_m` above the same sea-level reference the ISA
    /// atmosphere uses, this is `-elevation_m`.
    pub ground_z_world_m: f64,
    /// ISA sea-level temperature offset, K (e.g. +15 for an ISA+15 day).
    pub oat_offset_k: f64,
    /// QNH sea-level pressure offset, Pa, from the ISA standard 101,325 Pa.
    pub qnh_offset_pa: f64,
    pub wind: WindInput,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct FlightModelOutputs {
    pub pitch_deg: f64,
    pub bank_deg: f64,
    pub heading_true_deg: f64,
    /// GENERIC: equivalent airspeed (`TAS * sqrt(rho/rho0)`) used directly
    /// as indicated airspeed -- position error and compressibility (the
    /// CAS/EAS split above ~M0.3) are not modelled.
    pub indicated_airspeed_kt: f64,
    pub true_airspeed_kt: f64,
    pub mach: f64,
    pub groundspeed_kt: f64,
    pub agl_ft: f64,
    pub altitude_ft: f64,
    pub on_ground: bool,
    /// Normal load factor, g (1.0 in level unaccelerated flight): the
    /// non-gravitational body-z force (aero+thrust+gear) divided by
    /// weight, sign flipped so wings-level 1g flight reads +1.0.
    pub vertical_load_factor_g: f64,
    /// One entry per `GEAR_LEGS` element, newtons.
    pub gear_leg_load_n: [f64; 5],
    pub total_weight_on_wheels_n: f64,
    pub tailstrike: bool,
    pub tailstrike_load_n: f64,
    pub stalled: bool,
}

/// Every point this model needs arms for, computed once per tick from the
/// current CG (`mass::arm`) so no other module needs to know about mass at
/// all.
struct Arms {
    aero: aerodynamics::AeroArms,
    gear: [Vec3; 5],
    tailstrike: Vec3,
    engines: [Vec3; 4],
}

fn arms_for(mp: &MassProperties) -> Arms {
    let aero = aerodynamics::AeroArms {
        wing_left_m: mass::arm(mp, geometry::wing_half_center_m(false)),
        wing_right_m: mass::arm(mp, geometry::wing_half_center_m(true)),
        htail_left_m: mass::arm(mp, geometry::htail_half_center_m(false)),
        htail_right_m: mass::arm(mp, geometry::htail_half_center_m(true)),
        vtail_m: mass::arm(mp, geometry::vtail().position_m),
    };
    let mut gear = [Vec3::ZERO; 5];
    for (i, leg) in GEAR_LEGS.iter().enumerate() {
        gear[i] = mass::arm(mp, geometry::gear_position_m(*leg));
    }
    let tailstrike = mass::arm(mp, geometry::tailstrike_point_m());
    let engines = [
        mass::engine_arm(mp, Engine::One),
        mass::engine_arm(mp, Engine::Two),
        mass::engine_arm(mp, Engine::Three),
        mass::engine_arm(mp, Engine::Four),
    ];
    Arms { aero, gear, tailstrike, engines }
}

pub struct FlightModel {
    rigid_body: RigidBody,
    aero: Aerodynamics,
    gear: LandingGear,
    gust_phase_s: f64,
}

impl FlightModel {
    pub fn new(initial_state: RigidBodyState) -> Self {
        Self { rigid_body: RigidBody::new(initial_state), aero: Aerodynamics::new(), gear: LandingGear::new(), gust_phase_s: 0.0 }
    }

    pub fn state(&self) -> RigidBodyState {
        self.rigid_body.state()
    }
    pub fn set_state(&mut self, state: RigidBodyState) {
        self.rigid_body.set_state(state);
    }

    fn wind_at(inputs: &FlightModelInputs, gust_phase_s: f64) -> Wind {
        Wind { steady_10m: inputs.wind.steady_10m_m_s, shear_exponent: inputs.wind.shear_exponent, gust_peak_m_s: inputs.wind.gust_peak_m_s, gust_period_s: inputs.wind.gust_period_s, gust_phase_s }
    }

    fn body_airspeed(state: &RigidBodyState, wind: &Wind, agl_m: f64) -> Vec3 {
        let wind_world = wind.at_height(agl_m.max(1.0));
        let wind_body = state.attitude.rotate_inverse(wind_world);
        state.velocity_body_m_s.sub(wind_body)
    }

    /// One tick: resolves this tick's control-surface positions and
    /// gear/propulsion forces once (see `aerodynamics.rs`'s module doc for
    /// why), then RK4-integrates the rigid body with aerodynamics and
    /// gravity re-evaluated at each stage.
    pub fn step(&mut self, inputs: &FlightModelInputs, dt_s: f64) -> FlightModelOutputs {
        let dt = dt_s.max(0.0);
        let mp = mass::current(inputs.mass_kg, inputs.cg_x_forward_m);
        let arms = arms_for(&mp);

        let resolved = self.aero.update_actuators(&inputs.controls, &inputs.aero_faults, dt);

        let state0 = self.rigid_body.state();
        let gear_out = self.gear.step(
            &GearInputs {
                position_world_m: state0.position_world_m,
                attitude: state0.attitude,
                velocity_body_m_s: state0.velocity_body_m_s,
                rate_body_rad_s: state0.rate_body_rad_s,
                ground_z_world_m: inputs.ground_z_world_m,
                runway_mu_factor: inputs.runway_mu_factor,
                brake_cmd: inputs.brake_cmd,
                nose_steering_cmd: inputs.nose_steering_cmd,
                arms: arms.gear,
                tailstrike_arm_m: arms.tailstrike,
            },
            &inputs.gear_faults,
            dt,
        );
        let PropulsionOutputs { force_body_n: prop_force, moment_about_cg_n_m: prop_moment } = propulsion::step(&inputs.engines, &arms.engines, state0.rate_body_rad_s);

        let mut wind = Self::wind_at(inputs, self.gust_phase_s);
        let gear_force = gear_out.force_body_n;
        let gear_moment = gear_out.moment_about_cg_n_m;
        let g = atmosphere::G0;

        let forces_body = |state: &RigidBodyState| -> (Vec3, Vec3) {
            let altitude_msl_m = state.altitude_m();
            let air = atmosphere::isa(altitude_msl_m, inputs.oat_offset_k, inputs.qnh_offset_pa);
            let agl_m = (inputs.ground_z_world_m - state.position_world_m.z).max(0.0);
            let body_airspeed = Self::body_airspeed(state, &wind, agl_m);
            let flight = AeroFlightState { body_airspeed_m_s: body_airspeed, body_rates_rad_s: state.rate_body_rad_s, air, height_agl_m: agl_m, gear_extended_fraction: inputs.gear_extended_fraction, arms: arms.aero };
            let aero_out = Aerodynamics::forces(&resolved, &flight, &inputs.aero_faults);
            let gravity_world = Vec3::new(0.0, 0.0, mp.mass_kg * g);
            let gravity_body = state.attitude.rotate_inverse(gravity_world);
            let force = aero_out.force_body_n.add(prop_force).add(gear_force).add(gravity_body);
            let moment = aero_out.moment_about_cg_n_m.add(prop_moment).add(gear_moment);
            (force, moment)
        };
        self.rigid_body.step(dt, &mp, forces_body);
        wind.advance(dt);
        self.gust_phase_s = wind.gust_phase_s;

        // ---- Outputs, from the settled end-of-tick state.
        let state = self.rigid_body.state();
        let altitude_msl_m = state.altitude_m();
        let air = atmosphere::isa(altitude_msl_m, inputs.oat_offset_k, inputs.qnh_offset_pa);
        let agl_m = (inputs.ground_z_world_m - state.position_world_m.z).max(0.0);
        let body_airspeed = Self::body_airspeed(&state, &wind, agl_m);
        let tas = body_airspeed.norm();
        let mach = if air.sound_speed_m_s > 1.0 { tas / air.sound_speed_m_s } else { 0.0 };
        let ias = tas * (air.density_kg_m3 / 1.225).max(0.0).sqrt();
        let ground_velocity_world = state.attitude.rotate(state.velocity_body_m_s);
        let groundspeed = (ground_velocity_world.x * ground_velocity_world.x + ground_velocity_world.y * ground_velocity_world.y).sqrt();
        let (roll, pitch, yaw) = state.euler_rad();
        let heading = (yaw.to_degrees() + 360.0) % 360.0;

        let flight = AeroFlightState { body_airspeed_m_s: body_airspeed, body_rates_rad_s: state.rate_body_rad_s, air, height_agl_m: agl_m, gear_extended_fraction: inputs.gear_extended_fraction, arms: arms.aero };
        let aero_final = Aerodynamics::forces(&resolved, &flight, &inputs.aero_faults);
        let nongravity_force = aero_final.force_body_n.add(prop_force).add(gear_force);
        let weight_n = (mp.mass_kg * g).max(1.0);
        let vertical_load_factor_g = -nongravity_force.z / weight_n;

        FlightModelOutputs {
            pitch_deg: pitch.to_degrees(),
            bank_deg: roll.to_degrees(),
            heading_true_deg: heading,
            indicated_airspeed_kt: ias / geometry::KT_TO_M_S,
            true_airspeed_kt: tas / geometry::KT_TO_M_S,
            mach,
            groundspeed_kt: groundspeed / geometry::KT_TO_M_S,
            agl_ft: agl_m / geometry::FT_TO_M,
            altitude_ft: altitude_msl_m / geometry::FT_TO_M,
            on_ground: gear_out.any_wheel_on_ground,
            vertical_load_factor_g,
            gear_leg_load_n: gear_out.leg_load_n,
            total_weight_on_wheels_n: gear_out.total_weight_on_wheels_n(),
            tailstrike: gear_out.tailstrike,
            tailstrike_load_n: gear_out.tailstrike_load_n,
            stalled: aero_final.stalled,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn parked_inputs(ground_z: f64) -> FlightModelInputs {
        FlightModelInputs {
            mass_kg: geometry::EMPTY_MASS_KG,
            cg_x_forward_m: geometry::empty_cg_m().x,
            ground_z_world_m: ground_z,
            runway_mu_factor: 1.0,
            ..Default::default()
        }
    }

    /// The full-system version of "gear static load equals weight share":
    /// parked, no thrust, no wind, gravity and the gear spring-dampers left
    /// to settle -- the aircraft should come to rest with the gear
    /// supporting essentially all of its own weight (a tiny residual sits
    /// in vertical velocity/damping noise at settling, not a systematic
    /// bias), never diverging.
    #[test]
    fn a_parked_aircraft_settles_so_the_gear_carries_its_own_weight() {
        // Find a ground level that guarantees initial contact, using the
        // same geometry the gear module itself uses, then let gravity
        // settle the aircraft onto it.
        let mp = mass::current(geometry::EMPTY_MASS_KG, geometry::empty_cg_m().x);
        let arms = arms_for(&mp);
        let deepest = arms.gear.iter().map(|a| a.z).fold(f64::MIN, f64::max);
        let ground_z = deepest + 0.05;
        let mut fm = FlightModel::new(RigidBodyState::default());
        let inputs = parked_inputs(ground_z);
        let mut out = FlightModelOutputs::default();
        for _ in 0..2000 {
            out = fm.step(&inputs, 0.02);
        }
        let weight_n = geometry::EMPTY_MASS_KG * atmosphere::G0;
        assert!(out.on_ground, "should have settled onto the gear");
        let ratio = out.total_weight_on_wheels_n / weight_n;
        assert!((ratio - 1.0).abs() < 0.15, "gear load {} should be close to weight {} (ratio {ratio})", out.total_weight_on_wheels_n, weight_n);
        assert!(fm.state().velocity_body_m_s.norm() < 1.0, "should have settled, not still falling/bouncing hard");
    }

    #[test]
    fn losing_an_outboard_engine_on_the_ground_roll_yaws_toward_it() {
        let mut fm = FlightModel::new(RigidBodyState { velocity_body_m_s: Vec3::new(60.0, 0.0, 0.0), ..Default::default() });
        let mp = mass::current(geometry::EMPTY_MASS_KG, geometry::empty_cg_m().x);
        let arms = arms_for(&mp);
        let ground_z = arms.gear.iter().map(|a| a.z).fold(f64::MIN, f64::max) + 0.1;
        let mut inputs = parked_inputs(ground_z);
        inputs.engines = [EngineThrustInput { net_thrust_n: 200_000.0, ..Default::default() }; 4];
        inputs.engines[0].net_thrust_n = 0.0; // Engine 1 (outboard left) fails.
        for _ in 0..50 {
            fm.step(&inputs, 0.02);
        }
        // Losing the left-outboard engine should yaw the nose left: a
        // negative body rate about +z (see propulsion.rs's own sign test).
        assert!(fm.state().rate_body_rad_s.z < 0.0, "should be yawing left toward the dead engine: r={}", fm.state().rate_body_rad_s.z);
    }

    #[test]
    fn a_glide_holds_together_without_diverging_or_going_nan() {
        // Level-ish entry speed, no thrust, well clear of the ground: a
        // basic longitudinal-stability check (not a trimmed-AoA solve --
        // this project has no trim algorithm yet) that the coupled
        // rigid-body/aerodynamics loop settles into a bounded glide rather
        // than diverging.
        let mut fm = FlightModel::new(RigidBodyState { velocity_body_m_s: Vec3::new(230.0, 0.0, 0.0), position_world_m: Vec3::new(0.0, 0.0, -11_000.0), ..Default::default() });
        let mut inputs = parked_inputs(1.0e6); // ground far below: stays airborne throughout.
        inputs.controls.elevator_cmd = -0.02; // a small nose-down bias, no trim solver available.
        // 15 s: enough to exercise the coupled rigid-body/aerodynamics loop
        // for many ticks without giving an untrimmed phugoid time to wander
        // far from the entry condition (there is no trim solver to hold it
        // there, so this is deliberately a short-horizon sanity check, not
        // a claim of long-term stability).
        for _ in 0..300 {
            let out = fm.step(&inputs, 0.05);
            assert!(out.true_airspeed_kt.is_finite() && out.pitch_deg.is_finite() && out.bank_deg.is_finite());
            assert!(out.true_airspeed_kt < 700.0, "should not run away to an unbounded speed: {}", out.true_airspeed_kt);
        }
        let s = fm.state();
        assert!(s.velocity_body_m_s.norm().is_finite() && s.velocity_body_m_s.norm() > 10.0);
        assert!((s.attitude.norm() - 1.0).abs() < 1e-6);
    }

    #[test]
    fn zero_dt_changes_nothing_and_never_panics() {
        let mut fm = FlightModel::new(RigidBodyState::default());
        let before = fm.state();
        fm.step(&parked_inputs(1.0e6), 0.0);
        let after = fm.state();
        assert_eq!(before.position_world_m, after.position_world_m);
        assert_eq!(before.velocity_body_m_s, after.velocity_body_m_s);
    }
}
