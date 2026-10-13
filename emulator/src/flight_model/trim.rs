//! Trim solvers: find the control/thrust setting that makes the rigid body
//! stay at a steady equilibrium, so a test can start *at* a known trim
//! condition instead of only being able to observe whether a perturbation
//! from an arbitrary starting point diverges (`mod.rs`'s current glide
//! test's own limitation, called out in `PROGRESS.md`).
//!
//! Both solvers are the same idea applied to two different equilibria:
//! find the small number of unknowns that drive the relevant force/moment
//! residuals to zero, by 2-D Newton-Raphson with a finite-difference
//! Jacobian (`newton_2d`) -- standard numerical trim-solving practice
//! (e.g. Stevens & Lewis, "Aircraft Control and Simulation", 3rd ed.,
//! section 3.5, "Trim"), not a lookup table or a scripted answer: the
//! solver calls the *same* `aerodynamics`/`landing_gear` force models this
//! whole flight model runs on, so a trim point is only ever whatever those
//! models say balances.
//!
//! - [`trim_level_flight`]: steady, wings-level, zero flight-path-angle
//!   flight at a given TAS/altitude/mass/CG. Unknowns: angle of attack and
//!   elevator (this project's model has no separate trimmable horizontal
//!   stabilizer surface -- `elevator_cmd` stands in for "elevator + THS"
//!   combined, per `aerodynamics.rs`'s single elevator abstraction);
//!   symmetric thrust is then read off the x-force balance directly (not
//!   an unknown the Newton solve needs), and the flight-path-angle-zero
//!   assumption ties pitch attitude to angle of attack (`theta = alpha`).
//! - [`trim_on_ground`]: steady, parked, zero-airspeed equilibrium on the
//!   gear at a given mass/CG. Unknowns: pitch attitude and height (the
//!   vertical position that sets every leg's penetration together).

use super::aerodynamics::{AeroFaults, AeroFlightState, Aerodynamics, ResolvedSurfaces};
use super::atmosphere::{self, AirState};
use super::geometry;
use super::landing_gear::{GearFaults, GearInputs, LandingGear};
use super::mass::{self, MassProperties};
use super::math::{Quat, Vec3};

/// 2-D Newton-Raphson with a forward-difference Jacobian. `f(x, y)`
/// returns the two residuals that should both be zero at the solution.
/// Bails out (reporting `converged: false`) on a singular Jacobian rather
/// than dividing by ~0, and always returns its best estimate so a caller
/// gets a usable (if imperfect) answer instead of a panic.
fn newton_2d(f: impl Fn(f64, f64) -> (f64, f64), x0: f64, y0: f64, max_iter: u32, tol: f64) -> (f64, f64, bool, u32) {
    let (mut x, mut y) = (x0, y0);
    const EPS: f64 = 1e-6;
    for i in 0..max_iter {
        let (fx, fy) = f(x, y);
        if fx.abs() < tol && fy.abs() < tol {
            return (x, y, true, i);
        }
        let (fx_dx, fy_dx) = f(x + EPS, y);
        let (fx_dy, fy_dy) = f(x, y + EPS);
        let j11 = (fx_dx - fx) / EPS;
        let j21 = (fy_dx - fy) / EPS;
        let j12 = (fx_dy - fx) / EPS;
        let j22 = (fy_dy - fy) / EPS;
        let det = j11 * j22 - j12 * j21;
        if !det.is_finite() || det.abs() < 1e-12 {
            return (x, y, false, i);
        }
        // Newton step: solve J*delta = -F for delta, via the closed-form
        // 2x2 inverse, then x/y += delta.
        let dx = (j12 * fy - j22 * fx) / det;
        let dy = (j21 * fx - j11 * fy) / det;
        x += dx;
        y += dy;
        if !x.is_finite() || !y.is_finite() {
            return (x0, y0, false, i);
        }
    }
    let (fx, fy) = f(x, y);
    (x, y, fx.abs() < tol * 10.0 && fy.abs() < tol * 10.0, max_iter)
}

// ---------------------------------------------------------------------
// Level flight.
// ---------------------------------------------------------------------

#[derive(Clone, Copy, Debug)]
pub struct LevelFlightInputs {
    pub mass_kg: f64,
    pub cg_x_forward_m: f64,
    pub tas_m_s: f64,
    pub altitude_msl_m: f64,
    pub oat_offset_k: f64,
    pub qnh_offset_pa: f64,
}

#[derive(Clone, Copy, Debug)]
pub struct LevelFlightTrim {
    pub alpha_rad: f64,
    /// `= alpha_rad` (flight-path angle held at zero -- steady level
    /// flight).
    pub pitch_rad: f64,
    /// Normalized -1..1, as `aerodynamics::ControlInputs::elevator_cmd`
    /// takes (see the module doc on why this stands in for "elevator +
    /// THS" combined).
    pub elevator_cmd: f64,
    /// Symmetric thrust needed per engine, newtons (from the x-force
    /// balance, not a Newton unknown).
    pub thrust_per_engine_n: f64,
    pub converged: bool,
    pub iterations: u32,
}

fn aero_arms_for(mp: &MassProperties) -> super::aerodynamics::AeroArms {
    super::aerodynamics::AeroArms {
        wing_left_m: mass::arm(mp, geometry::wing_half_center_m(false)),
        wing_right_m: mass::arm(mp, geometry::wing_half_center_m(true)),
        htail_left_m: mass::arm(mp, geometry::htail_half_center_m(false)),
        htail_right_m: mass::arm(mp, geometry::htail_half_center_m(true)),
        vtail_m: mass::arm(mp, geometry::vtail().position_m),
    }
}

fn engine_arms_for(mp: &MassProperties) -> [Vec3; 4] {
    use geometry::Engine;
    [mass::engine_arm(mp, Engine::One), mass::engine_arm(mp, Engine::Two), mass::engine_arm(mp, Engine::Three), mass::engine_arm(mp, Engine::Four)]
}

/// Residual force (body z) and moment (body y) for a trial `(alpha,
/// elevator_cmd)`, plus the per-engine thrust the x-force balance implies
/// at that trial point -- computed directly from `aerodynamics::forces`
/// and gravity, the exact same physics `FlightModel::step` runs, at zero
/// body rates (a trim point is by definition non-rotating).
fn level_flight_residual(inputs: &LevelFlightInputs, mp: &MassProperties, air: &AirState, arms: &super::aerodynamics::AeroArms, engine_arms: &[Vec3; 4], alpha: f64, elevator_cmd: f64) -> (f64, f64, f64) {
    let elevator_geom = geometry::elevator_geometry();
    let e = elevator_cmd.clamp(-1.0, 1.0) * elevator_geom.limit_rad;
    let resolved = ResolvedSurfaces { elevator_l_rad: e, elevator_r_rad: e, aileron_l_rad: 0.0, aileron_r_rad: 0.0, rudder_rad: 0.0, spoiler_l: 0.0, spoiler_r: 0.0, flap: 0.0, slat: 0.0 };
    let body_airspeed = Vec3::new(inputs.tas_m_s * alpha.cos(), 0.0, inputs.tas_m_s * alpha.sin());
    let flight = AeroFlightState { body_airspeed_m_s: body_airspeed, body_rates_rad_s: Vec3::ZERO, air: *air, height_agl_m: 10_000.0, gear_extended_fraction: 0.0, arms: *arms };
    let aero = Aerodynamics::forces(&resolved, &flight, &AeroFaults::default());

    // Zero flight-path angle: pitch attitude equals alpha.
    let attitude = Quat::from_euler(0.0, alpha, 0.0);
    let gravity_world = Vec3::new(0.0, 0.0, mp.mass_kg * atmosphere::G0);
    let gravity_body = attitude.rotate_inverse(gravity_world);

    // x-force balance gives the thrust this trial point needs (engines
    // produce body +x force only, per `propulsion.rs`): T + Fx_aero +
    // Fx_gravity = 0.
    let thrust_total = -(aero.force_body_n.x + gravity_body.x);
    let per_engine = thrust_total / 4.0;
    let mut thrust_moment_y = 0.0;
    for a in engine_arms.iter() {
        // (r x F).y = rz*Fx - rx*Fz, and thrust's F is (per_engine, 0, 0).
        thrust_moment_y += a.z * per_engine;
    }

    let fz_total = aero.force_body_n.z + gravity_body.z; // thrust has no z component.
    let my_total = aero.moment_about_cg_n_m.y + thrust_moment_y;
    (fz_total, my_total, per_engine)
}

/// Solves for the angle of attack and elevator command that null the
/// vertical-force and pitching-moment residuals at `inputs.tas_m_s`, then
/// reads the required symmetric thrust off the x-force balance.
pub fn trim_level_flight(inputs: &LevelFlightInputs) -> LevelFlightTrim {
    let mp = mass::current(inputs.mass_kg, inputs.cg_x_forward_m);
    let air = atmosphere::isa(inputs.altitude_msl_m, inputs.oat_offset_k, inputs.qnh_offset_pa);
    let arms = aero_arms_for(&mp);
    let engine_arms = engine_arms_for(&mp);

    // Initial guess: a small positive alpha (weight/dynamic-pressure gives
    // a rough starting CL), no elevator.
    let w = geometry::wing();
    let q = 0.5 * air.density_kg_m3 * inputs.tas_m_s * inputs.tas_m_s;
    let cl_guess = (mp.mass_kg * atmosphere::G0 / (q * w.area_m2).max(1.0)).clamp(0.05, 1.2);
    let alpha0 = (cl_guess / (2.0 * std::f64::consts::PI)).clamp(0.005, 0.25);

    let (alpha, elevator_cmd, converged, iterations) = newton_2d(|a, e| { let (fz, my, _) = level_flight_residual(inputs, &mp, &air, &arms, &engine_arms, a, e); (fz, my) }, alpha0, 0.0, 80, 25.0);
    let (_, _, thrust_per_engine_n) = level_flight_residual(inputs, &mp, &air, &arms, &engine_arms, alpha, elevator_cmd);

    LevelFlightTrim { alpha_rad: alpha, pitch_rad: alpha, elevator_cmd: elevator_cmd.clamp(-1.0, 1.0), thrust_per_engine_n, converged, iterations }
}

// ---------------------------------------------------------------------
// On-ground static equilibrium.
// ---------------------------------------------------------------------

#[derive(Clone, Copy, Debug)]
pub struct StaticGroundInputs {
    pub mass_kg: f64,
    pub cg_x_forward_m: f64,
}

#[derive(Clone, Copy, Debug)]
pub struct StaticGroundTrim {
    pub pitch_rad: f64,
    /// World-frame z (down-positive) of the CG at rest, ground plane at 0.
    pub cg_z_world_m: f64,
    pub leg_load_n: [f64; 5],
    pub total_weight_on_wheels_n: f64,
    pub converged: bool,
    pub iterations: u32,
}

fn gear_arms_for(mp: &MassProperties) -> ([Vec3; 5], Vec3) {
    let mut gear = [Vec3::ZERO; 5];
    for (i, leg) in geometry::GEAR_LEGS.iter().enumerate() {
        gear[i] = mass::arm(mp, geometry::gear_position_m(*leg));
    }
    (gear, mass::arm(mp, geometry::tailstrike_point_m()))
}

fn ground_residual(mp: &MassProperties, gear_arms: &[Vec3; 5], tailstrike_arm: Vec3, pitch: f64, cg_z: f64) -> (f64, f64, super::landing_gear::GearOutputs) {
    let attitude = Quat::from_euler(0.0, pitch, 0.0);
    let mut gear = LandingGear::new();
    let out = gear.step(
        &GearInputs {
            position_world_m: Vec3::new(0.0, 0.0, cg_z),
            attitude,
            velocity_body_m_s: Vec3::ZERO,
            rate_body_rad_s: Vec3::ZERO,
            ground_z_world_m: 0.0,
            runway_mu_factor: 1.0,
            brake_cmd: 0.0,
            nose_steering_cmd: 0.0,
            arms: *gear_arms,
            tailstrike_arm_m: tailstrike_arm,
        },
        &GearFaults::default(),
        0.02,
    );
    let weight_n = mp.mass_kg * atmosphere::G0;
    let force_err = out.total_weight_on_wheels_n() - weight_n;
    let moment_err = out.moment_about_cg_n_m.y;
    (force_err, moment_err, out)
}

/// Solves for the pitch attitude and CG height at which the (zero-velocity)
/// gear reaction exactly supports the aircraft's weight with no net
/// pitching moment -- the static ground trim every "parked aircraft"
/// scenario should start from instead of free-falling onto the gear first
/// (as `mod.rs`'s settling test still does; this solver is the closed-form
/// alternative for tests that want to start already settled).
pub fn trim_on_ground(inputs: &StaticGroundInputs) -> StaticGroundTrim {
    let mp = mass::current(inputs.mass_kg, inputs.cg_x_forward_m);
    let (gear_arms, tailstrike_arm) = gear_arms_for(&mp);

    // Initial guess: a CG height (world z, down-positive) that puts every
    // leg in contact with a bit of margin. A leg touches when
    // `cg_z + arm.z > 0`, i.e. `cg_z > -arm.z`; the shallowest leg (the
    // smallest `arm.z`) sets the binding threshold, `-shallowest_z`, so
    // starting a little past that (more negative `-shallowest_z` is a
    // *smaller* deficit, so add a small positive margin) guarantees every
    // leg starts already penetrating.
    let shallowest_z = gear_arms.iter().map(|a| a.z).fold(f64::MAX, f64::min);
    let cg_z0 = -shallowest_z + 0.35;

    let (pitch, cg_z, converged, iterations) = newton_2d(
        |p, z| {
            let (fe, me, _) = ground_residual(&mp, &gear_arms, tailstrike_arm, p, z);
            (fe, me)
        },
        geometry::STATIC_PITCH_DEG.to_radians(),
        cg_z0,
        80,
        50.0,
    );
    let (_, _, out) = ground_residual(&mp, &gear_arms, tailstrike_arm, pitch, cg_z);

    StaticGroundTrim { pitch_rad: pitch, cg_z_world_m: cg_z, leg_load_n: out.leg_load_n, total_weight_on_wheels_n: out.total_weight_on_wheels_n(), converged, iterations }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cruise_inputs() -> LevelFlightInputs {
        LevelFlightInputs { mass_kg: geometry::MTOW_KG * 0.85, cg_x_forward_m: geometry::empty_cg_m().x - 1.0, tas_m_s: 230.0, altitude_msl_m: 11_000.0, oat_offset_k: 0.0, qnh_offset_pa: 0.0 }
    }

    #[test]
    fn level_flight_trim_converges_to_a_plausible_cruise_attitude() {
        let trim = trim_level_flight(&cruise_inputs());
        assert!(trim.converged, "trim should converge: {trim:?}");
        // A widebody at high-subsonic cruise trims at a shallow positive
        // alpha, comfortably inside +/-10 deg.
        assert!(trim.alpha_rad > 0.0 && trim.alpha_rad.to_degrees() < 10.0, "alpha {} deg out of plausible range", trim.alpha_rad.to_degrees());
        assert!(trim.elevator_cmd.abs() <= 1.0);
        assert!(trim.thrust_per_engine_n.is_finite() && trim.thrust_per_engine_n > 0.0, "should need positive thrust to hold level flight: {}", trim.thrust_per_engine_n);
    }

    #[test]
    fn the_solved_trim_point_actually_zeroes_the_residuals() {
        let inputs = cruise_inputs();
        let trim = trim_level_flight(&inputs);
        let mp = mass::current(inputs.mass_kg, inputs.cg_x_forward_m);
        let air = atmosphere::isa(inputs.altitude_msl_m, inputs.oat_offset_k, inputs.qnh_offset_pa);
        let arms = aero_arms_for(&mp);
        let engine_arms = engine_arms_for(&mp);
        let (fz, my, _) = level_flight_residual(&inputs, &mp, &air, &arms, &engine_arms, trim.alpha_rad, trim.elevator_cmd);
        let weight_n = inputs.mass_kg * atmosphere::G0;
        assert!(fz.abs() < weight_n * 1e-3, "vertical force should balance at trim: {fz} N vs weight {weight_n} N");
        assert!(my.abs() < 1.0e5, "pitching moment should balance at trim: {my} N*m");
    }

    #[test]
    fn heavier_aircraft_needs_more_thrust_and_more_alpha_at_the_same_speed() {
        let light = LevelFlightInputs { mass_kg: geometry::MTOW_KG * 0.6, ..cruise_inputs() };
        let heavy = LevelFlightInputs { mass_kg: geometry::MTOW_KG * 0.95, ..cruise_inputs() };
        let light_trim = trim_level_flight(&light);
        let heavy_trim = trim_level_flight(&heavy);
        assert!(heavy_trim.alpha_rad > light_trim.alpha_rad, "heavier should need more lift coefficient/alpha");
        assert!(heavy_trim.thrust_per_engine_n > light_trim.thrust_per_engine_n, "heavier should need more thrust (more induced drag)");
    }

    #[test]
    fn ground_trim_supports_the_full_weight_with_no_net_pitching_moment() {
        let trim = trim_on_ground(&StaticGroundInputs { mass_kg: geometry::EMPTY_MASS_KG, cg_x_forward_m: geometry::empty_cg_m().x });
        assert!(trim.converged, "ground trim should converge: {trim:?}");
        let weight_n = geometry::EMPTY_MASS_KG * atmosphere::G0;
        assert!((trim.total_weight_on_wheels_n - weight_n).abs() < weight_n * 1e-3, "gear should support the full weight: {} vs {}", trim.total_weight_on_wheels_n, weight_n);
        for (i, load) in trim.leg_load_n.iter().enumerate() {
            assert!(*load >= 0.0 && load.is_finite(), "leg {i} load should be a non-negative finite reaction: {load}");
        }
        // Cross-check against the cfg's own published static ground pitch
        // (`static_pitch`, flight_model.cfg line 62, -0.13 deg): this
        // solver's GENERIC gear stiffness won't reproduce it exactly, but
        // should land in the same small-nose-down neighbourhood, not
        // wildly off (e.g. tail-sitting).
        assert!(trim.pitch_rad.to_degrees().abs() < 5.0, "static ground pitch should be small: {} deg", trim.pitch_rad.to_degrees());
    }

    #[test]
    fn ground_trim_is_deterministic_and_nan_free() {
        let a = trim_on_ground(&StaticGroundInputs { mass_kg: geometry::EMPTY_MASS_KG, cg_x_forward_m: geometry::empty_cg_m().x });
        let b = trim_on_ground(&StaticGroundInputs { mass_kg: geometry::EMPTY_MASS_KG, cg_x_forward_m: geometry::empty_cg_m().x });
        assert_eq!(a.pitch_rad, b.pitch_rad);
        assert_eq!(a.cg_z_world_m, b.cg_z_world_m);
        assert!(a.pitch_rad.is_finite() && a.cg_z_world_m.is_finite());
    }
}
