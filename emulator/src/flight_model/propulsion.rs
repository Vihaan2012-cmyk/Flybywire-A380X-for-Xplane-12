//! Engine thrust as a force vector at its real mount position: this is the
//! entire point of the "engines as thrust vectors" backlog item -- an
//! asymmetric thrust setting (an engine failure, one throttle back) yaws
//! and rolls the aircraft *because* the thrust acts off the centreline and
//! below the wing, exactly like the real EASA.E.012 Trent 972B-84
//! installation this project ports (`docs/deep/BRIEF.md`'s aircraft
//! description), never as a scripted "engine N failed -> apply yaw"
//! symptom.
//!
//! Net thrust magnitude itself is **not** computed here: `physics::engine`
//! in the main plugin crate already models the Trent 972B-84's gas path
//! (`EngineOutputs::net_thrust_n`, see that module's doc); this module only
//! turns up to four such scalars into a body force and moment, plus the
//! generic installation effects every jet transport's thrust line carries:
//! gyroscopic moment from the spooling fan/compressor rotors reacting to
//! the airframe's own rotation, and a `reverser_deployed` flag that redirects
//! thrust instead of applying it forward (mirroring how the real plugin's
//! `engine_commands.rs` avoids double-counting reverse thrust, per that
//! module's own doc comment quoted in `lib.rs`).

use super::math::Vec3;

/// One engine's inputs for this tick.
#[derive(Clone, Copy, Debug, Default)]
pub struct EngineThrustInput {
    /// `EngineOutputs::net_thrust_n` from `physics::engine` -- always
    /// forward-positive gas-path thrust; the reverser (below) decides
    /// whether it actually pushes the aircraft forward.
    pub net_thrust_n: f64,
    /// The engine's spooling rotor(s) angular momentum about its own spin
    /// axis (body +x, since the engines are wing-mounted with the spin
    /// axis parallel to the fuselage), kg*m^2/s -- `spool_inertia_kg_m2 *
    /// spin_rate_rad_s`, left to the caller (`physics::engine`'s spool
    /// model already has both); 0 disables the gyroscopic term.
    pub spin_angular_momentum_kg_m2_s: f64,
    /// True while the reverser is deployed and the FADEC has scheduled
    /// reverse thrust: `net_thrust_n` then acts aft instead of forward
    /// (idle reverse still has a small positive `net_thrust_n` from the
    /// gas-path model, which this correctly turns into a small aft force).
    pub reverser_deployed: bool,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct PropulsionOutputs {
    pub force_body_n: Vec3,
    pub moment_about_cg_n_m: Vec3,
}

/// CG-relative arms (metres, body frame) for the four engines, in ATA
/// engine-number order (1 outboard-left, 2 inboard-left, 3 inboard-right,
/// 4 outboard-right) -- see `geometry::engine_position_m`.
pub type EngineArms = [Vec3; 4];

/// Sums all four engines' thrust into one force and moment about the CG.
/// `body_rates_rad_s` is only needed for the (small) gyroscopic term.
pub fn step(engines: &[EngineThrustInput; 4], arms: &EngineArms, body_rates_rad_s: Vec3) -> PropulsionOutputs {
    let mut force = Vec3::ZERO;
    let mut moment = Vec3::ZERO;
    for i in 0..4 {
        let e = &engines[i];
        // Reverse thrust acts aft (body -x); forward thrust acts along
        // body +x (the engine's thrust line is parallel to the fuselage --
        // no toe-in/toe-out published for this airframe, GENERIC: none
        // assumed).
        let thrust_n = if e.reverser_deployed { -e.net_thrust_n } else { e.net_thrust_n };
        let f = Vec3::new(thrust_n, 0.0, 0.0);
        force = force.add(f);
        moment = moment.add(arms[i].cross(f));
        // Gyroscopic reaction moment: a spinning rotor with angular
        // momentum H about body +x, carried by an airframe rotating at
        // omega, feels a reaction moment M = -omega x H (the rigid-body
        // gyroscopic precession term, e.g. Stevens & Lewis section 1.7 --
        // this is the same mechanism that makes a spinning bicycle wheel
        // resist being tilted). `spin_angular_momentum` includes the
        // engine's own effective spin sign (fan rotation direction).
        let h = Vec3::new(e.spin_angular_momentum_kg_m2_s, 0.0, 0.0);
        moment = moment.sub(body_rates_rad_s.cross(h));
    }
    PropulsionOutputs { force_body_n: force, moment_about_cg_n_m: moment }
}

#[cfg(test)]
mod tests {
    use super::super::geometry::{self, Engine};
    use super::super::mass;
    use super::*;

    fn arms() -> EngineArms {
        let m = mass::current(geometry::EMPTY_MASS_KG, geometry::empty_cg_m().x);
        [
            mass::engine_arm(&m, Engine::One),
            mass::engine_arm(&m, Engine::Two),
            mass::engine_arm(&m, Engine::Three),
            mass::engine_arm(&m, Engine::Four),
        ]
    }

    #[test]
    fn symmetric_thrust_produces_pure_forward_force_and_no_yaw() {
        let engines = [EngineThrustInput { net_thrust_n: 200_000.0, ..Default::default() }; 4];
        let out = step(&engines, &arms(), Vec3::ZERO);
        assert!((out.force_body_n.x - 800_000.0).abs() < 1.0);
        assert!(out.force_body_n.y.abs() < 1e-6 && out.force_body_n.z.abs() < 1e-6);
        assert!(out.moment_about_cg_n_m.z.abs() < 1.0, "symmetric thrust should not yaw: {}", out.moment_about_cg_n_m.z);
    }

    #[test]
    fn losing_the_left_outboard_engine_yaws_the_nose_left() {
        let mut engines = [EngineThrustInput { net_thrust_n: 200_000.0, ..Default::default() }; 4];
        engines[0].net_thrust_n = 0.0; // Engine 1, outboard left, fails.
        let out = step(&engines, &arms(), Vec3::ZERO);
        // Losing thrust on the left means the right engines now produce a
        // net moment that yaws the nose toward the dead engine (left):
        // positive rotation about +z carries +x (nose) toward +y (right),
        // so yawing *left* (away from +y) is a *negative* Nz.
        assert!(out.moment_about_cg_n_m.z < -1000.0, "losing the left-outboard engine should yaw the nose left (-Nz): {}", out.moment_about_cg_n_m.z);
    }

    #[test]
    fn losing_the_right_outboard_engine_yaws_the_nose_right_the_opposite_sign() {
        let mut engines = [EngineThrustInput { net_thrust_n: 200_000.0, ..Default::default() }; 4];
        engines[3].net_thrust_n = 0.0; // Engine 4, outboard right, fails.
        let out = step(&engines, &arms(), Vec3::ZERO);
        assert!(out.moment_about_cg_n_m.z > 1000.0, "losing the right-outboard engine should yaw the nose right (+Nz): {}", out.moment_about_cg_n_m.z);
    }

    #[test]
    fn an_outboard_engine_failure_yaws_harder_than_an_inboard_one() {
        // A larger moment arm (outboard) must produce a larger yaw moment
        // than the same thrust loss on an inboard engine -- this is the
        // real reason V_MCA/rudder sizing cares which engine is critical.
        let mut outboard = [EngineThrustInput { net_thrust_n: 200_000.0, ..Default::default() }; 4];
        outboard[0].net_thrust_n = 0.0;
        let mut inboard = [EngineThrustInput { net_thrust_n: 200_000.0, ..Default::default() }; 4];
        inboard[1].net_thrust_n = 0.0;
        let out_outboard = step(&outboard, &arms(), Vec3::ZERO);
        let out_inboard = step(&inboard, &arms(), Vec3::ZERO);
        assert!(out_outboard.moment_about_cg_n_m.z.abs() > out_inboard.moment_about_cg_n_m.z.abs());
    }

    #[test]
    fn reverse_thrust_pushes_aft_instead_of_forward() {
        let engines = [EngineThrustInput { net_thrust_n: 50_000.0, reverser_deployed: true, ..Default::default() }; 4];
        let out = step(&engines, &arms(), Vec3::ZERO);
        assert!(out.force_body_n.x < 0.0, "reverse thrust should decelerate (act aft): {}", out.force_body_n.x);
    }

    #[test]
    fn no_thrust_and_no_rotation_gives_exactly_zero_with_no_nan() {
        let engines = [EngineThrustInput::default(); 4];
        let out = step(&engines, &arms(), Vec3::ZERO);
        assert_eq!(out.force_body_n, Vec3::ZERO);
        assert_eq!(out.moment_about_cg_n_m, Vec3::ZERO);
    }

    #[test]
    fn gyroscopic_moment_is_zero_with_no_body_rotation_and_finite_with_rotation() {
        let mut engines = [EngineThrustInput::default(); 4];
        for e in &mut engines {
            e.spin_angular_momentum_kg_m2_s = 500.0;
        }
        let still = step(&engines, &arms(), Vec3::ZERO);
        assert_eq!(still.moment_about_cg_n_m, Vec3::ZERO);
        let pitching = step(&engines, &arms(), Vec3::new(0.0, 0.2, 0.0));
        let m = pitching.moment_about_cg_n_m;
        assert!(m.x.is_finite() && m.y.is_finite() && m.z.is_finite());
        assert!(m.norm() > 0.0);
    }
}
