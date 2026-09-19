//! Six-degree-of-freedom rigid-body dynamics: the flat-earth, non-rotating-
//! Earth equations of motion every introductory flight-dynamics text
//! derives (e.g. Stevens & Lewis, "Aircraft Control and Simulation", 3rd
//! ed., chapter 1; B. Etkin, "Dynamics of Atmospheric Flight"), integrated
//! with classical 4th-order Runge-Kutta for accuracy at the emulator's
//! `dt=0.05 s` tick (`emulator/src/lib.rs`'s own `tick(dt)`).
//!
//! State (13 numbers): position in a local NED-style world frame (north,
//! east, down, metres -- flat-earth, no geodesy, adequate for a systems
//! test bench that never flies far enough to need it), the body-to-world
//! attitude quaternion, velocity in the **body** frame (m/s), and the body
//! angular rate (rad/s). Body axes throughout this project's flight model:
//! x forward, y right, z down (`geometry.rs`'s module doc explains why).
//!
//! Equations of motion (all body-frame unless noted):
//! - `d(pos_world)/dt = q.rotate(v_body)` (kinematic transport).
//! - `d(v_body)/dt = F_body/m - omega x v_body` (Newton's second law in a
//!   rotating frame; `F_body` must already include gravity, rotated into
//!   body axes by the caller -- see [`RigidBody::step`]).
//! - `d(q)/dt = 1/2 q (x) [0, omega]` (`math::Quat::derivative`).
//! - `I * d(omega)/dt = M_body - omega x (I * omega)` (Euler's rotation
//!   equation for a body of constant inertia over the step; `I` diagonal,
//!   per `mass.rs`'s symmetric-airframe assumption).
//!
//! Mass and inertia are **not** part of the integrated state: the caller
//! passes the current [`super::mass::MassProperties`] into each
//! [`RigidBody::step`] call (mass changes on a fuel-burn timescale, far
//! slower than one 0.05 s tick, so treating it as constant *within* one RK4
//! step -- but re-read from `mass.rs` every tick -- is exact enough without
//! needing a 14th state variable).

use super::mass::{Inertia, MassProperties};
use super::math::{Quat, Vec3};

#[derive(Clone, Copy, Debug)]
pub struct RigidBodyState {
    pub position_world_m: Vec3,
    pub attitude: Quat,
    pub velocity_body_m_s: Vec3,
    pub rate_body_rad_s: Vec3,
}

impl Default for RigidBodyState {
    fn default() -> Self {
        Self { position_world_m: Vec3::ZERO, attitude: Quat::IDENTITY, velocity_body_m_s: Vec3::ZERO, rate_body_rad_s: Vec3::ZERO }
    }
}

impl RigidBodyState {
    /// True airspeed/groundspeed magnitude, body frame (no wind
    /// subtraction here -- that is `FlightModel`'s job since only it knows
    /// the wind).
    pub fn speed_m_s(&self) -> f64 {
        self.velocity_body_m_s.norm()
    }

    pub fn altitude_m(&self) -> f64 {
        -self.position_world_m.z
    }

    /// (roll, pitch, yaw), radians.
    pub fn euler_rad(&self) -> (f64, f64, f64) {
        self.attitude.to_euler()
    }

    fn is_finite(&self) -> bool {
        let p = self.position_world_m;
        let v = self.velocity_body_m_s;
        let w = self.rate_body_rad_s;
        let q = self.attitude;
        p.x.is_finite()
            && p.y.is_finite()
            && p.z.is_finite()
            && v.x.is_finite()
            && v.y.is_finite()
            && v.z.is_finite()
            && w.x.is_finite()
            && w.y.is_finite()
            && w.z.is_finite()
            && q.w.is_finite()
            && q.x.is_finite()
            && q.y.is_finite()
            && q.z.is_finite()
    }
}

struct Derivative {
    d_position: Vec3,
    d_attitude: Quat,
    d_velocity: Vec3,
    d_rate: Vec3,
}

fn inertia_apply(i: Inertia, w: Vec3) -> Vec3 {
    Vec3::new(i.ixx * w.x, i.iyy * w.y, i.izz * w.z)
}
fn inertia_inverse_apply(i: Inertia, w: Vec3) -> Vec3 {
    Vec3::new(w.x / i.ixx.max(1.0), w.y / i.iyy.max(1.0), w.z / i.izz.max(1.0))
}

/// One derivative evaluation at `state`, given the force/moment callback
/// (evaluated fresh at every RK4 stage, so aerodynamic/thrust forces that
/// depend on the instantaneous velocity/attitude are integrated properly)
/// and the (held-constant-for-this-step) mass properties.
fn derivative(state: &RigidBodyState, mass: &MassProperties, forces: &dyn Fn(&RigidBodyState) -> (Vec3, Vec3)) -> Derivative {
    let (force_body, moment_body) = forces(state);
    let m = mass.mass_kg.max(1.0);
    let omega = state.rate_body_rad_s;
    let d_velocity = force_body.scale(1.0 / m).sub(omega.cross(state.velocity_body_m_s));
    let d_rate = inertia_inverse_apply(mass.inertia, moment_body.sub(omega.cross(inertia_apply(mass.inertia, omega))));
    Derivative { d_position: state.attitude.rotate(state.velocity_body_m_s), d_attitude: state.attitude.derivative(omega), d_velocity, d_rate }
}

fn advance(state: &RigidBodyState, d: &Derivative, h: f64) -> RigidBodyState {
    RigidBodyState {
        position_world_m: state.position_world_m.add(d.d_position.scale(h)),
        attitude: state.attitude.add_scaled(d.d_attitude, h),
        velocity_body_m_s: state.velocity_body_m_s.add(d.d_velocity.scale(h)),
        rate_body_rad_s: state.rate_body_rad_s.add(d.d_rate.scale(h)),
    }
}

pub struct RigidBody {
    state: RigidBodyState,
}

impl RigidBody {
    pub fn new(state: RigidBodyState) -> Self {
        Self { state }
    }

    pub fn state(&self) -> RigidBodyState {
        self.state
    }

    pub fn set_state(&mut self, state: RigidBodyState) {
        self.state = state;
    }

    /// Classical 4th-order Runge-Kutta step: `forces_body` is called at the
    /// current state and three RK4-perturbed states (`t`, `t+dt/2` (twice),
    /// `t+dt`), returning `(force_body_n, moment_about_cg_n_m)` for each --
    /// it must already include gravity (rotate the world-frame gravity
    /// vector into body axes with `state.attitude.rotate_inverse`) since
    /// only the caller knows the local `g`. The attitude quaternion is
    /// re-normalized after the step (RK4 does not preserve `|q|=1` exactly)
    /// and every component is guarded against non-finite results, which
    /// would otherwise persist forever once introduced.
    pub fn step(&mut self, dt_s: f64, mass: &MassProperties, forces_body: impl Fn(&RigidBodyState) -> (Vec3, Vec3)) {
        let dt = dt_s.max(0.0);
        if dt <= 0.0 {
            return;
        }
        let s0 = self.state;
        let k1 = derivative(&s0, mass, &forces_body);
        let s1 = advance(&s0, &k1, dt * 0.5);
        let k2 = derivative(&s1, mass, &forces_body);
        let s2 = advance(&s0, &k2, dt * 0.5);
        let k3 = derivative(&s2, mass, &forces_body);
        let s3 = advance(&s0, &k3, dt);
        let k4 = derivative(&s3, mass, &forces_body);

        let combine = |a: Vec3, b: Vec3, c: Vec3, d: Vec3| a.add(b.scale(2.0)).add(c.scale(2.0)).add(d).scale(dt / 6.0);
        let mut next = RigidBodyState {
            position_world_m: s0.position_world_m.add(combine(k1.d_position, k2.d_position, k3.d_position, k4.d_position)),
            attitude: {
                let sum = combine_quat(k1.d_attitude, k2.d_attitude, k3.d_attitude, k4.d_attitude, dt);
                s0.attitude.add_scaled(sum, 1.0)
            },
            velocity_body_m_s: s0.velocity_body_m_s.add(combine(k1.d_velocity, k2.d_velocity, k3.d_velocity, k4.d_velocity)),
            rate_body_rad_s: s0.rate_body_rad_s.add(combine(k1.d_rate, k2.d_rate, k3.d_rate, k4.d_rate)),
        };
        next.attitude = next.attitude.normalized();
        if next.is_finite() {
            self.state = next;
        }
        // else: keep the last good state rather than propagate a NaN --
        // this should not happen given the guards in every force module,
        // but a rigid-body integrator is exactly the place a distant
        // upstream NaN would otherwise become unrecoverable.
    }
}

fn combine_quat(k1: Quat, k2: Quat, k3: Quat, k4: Quat, dt: f64) -> Quat {
    let scale = |q: Quat, s: f64| Quat { w: q.w * s, x: q.x * s, y: q.y * s, z: q.z * s };
    let add = |a: Quat, b: Quat| Quat { w: a.w + b.w, x: a.x + b.x, y: a.y + b.y, z: a.z + b.z };
    let sum = add(add(k1, scale(k2, 2.0)), add(scale(k3, 2.0), k4));
    scale(sum, dt / 6.0)
}

#[cfg(test)]
mod tests {
    use super::*;
    use super::super::geometry;

    fn empty_mass() -> MassProperties {
        super::super::mass::current(geometry::EMPTY_MASS_KG, geometry::empty_cg_m().x)
    }

    #[test]
    fn no_force_no_motion_stays_at_rest_forever() {
        let mut rb = RigidBody::new(RigidBodyState::default());
        let mass = empty_mass();
        for _ in 0..200 {
            rb.step(0.05, &mass, |_| (Vec3::ZERO, Vec3::ZERO));
        }
        let s = rb.state();
        assert_eq!(s.velocity_body_m_s, Vec3::ZERO);
        assert_eq!(s.rate_body_rad_s, Vec3::ZERO);
        assert!((s.attitude.norm() - 1.0).abs() < 1e-9);
    }

    #[test]
    fn a_steady_forward_force_produces_constant_acceleration() {
        let mut rb = RigidBody::new(RigidBodyState::default());
        let mass = empty_mass();
        let force = Vec3::new(mass.mass_kg * 2.0, 0.0, 0.0); // 2 m/s^2
        for _ in 0..20 {
            rb.step(0.05, &mass, |_| (force, Vec3::ZERO));
        }
        // 20 steps * 0.05 s = 1.0 s at 2 m/s^2 => 2 m/s.
        assert!((rb.state().velocity_body_m_s.x - 2.0).abs() < 1e-6);
    }

    #[test]
    fn a_pure_yaw_moment_spins_up_yaw_rate_only() {
        let mut rb = RigidBody::new(RigidBodyState::default());
        let mass = empty_mass();
        let moment = Vec3::new(0.0, 0.0, mass.inertia.izz);
        for _ in 0..20 {
            rb.step(0.05, &mass, |_| (Vec3::ZERO, moment));
        }
        // 1.0 s at alpha = M/Izz = 1 rad/s^2 => r = 1 rad/s.
        assert!((rb.state().rate_body_rad_s.z - 1.0).abs() < 1e-3);
        assert!(rb.state().rate_body_rad_s.x.abs() < 1e-9 && rb.state().rate_body_rad_s.y.abs() < 1e-9);
    }

    #[test]
    fn gravity_applied_in_body_axes_makes_a_resting_aircraft_fall() {
        let mut rb = RigidBody::new(RigidBodyState::default());
        let mass = empty_mass();
        let g = 9.80665;
        for _ in 0..20 {
            rb.step(0.05, &mass, |s| {
                let gravity_world = Vec3::new(0.0, 0.0, mass.mass_kg * g);
                (s.attitude.rotate_inverse(gravity_world), Vec3::ZERO)
            });
        }
        // Free-fall for 1.0 s: v_down = g*t = 9.807 m/s (level attitude, so
        // body z IS world down here).
        assert!((rb.state().velocity_body_m_s.z - g).abs() < 0.01);
        assert!(rb.state().position_world_m.z > 0.0, "should have descended (world z is down)");
    }

    #[test]
    fn zero_dt_is_a_no_op_and_never_produces_nan() {
        let mut rb = RigidBody::new(RigidBodyState::default());
        let mass = empty_mass();
        let before = rb.state();
        rb.step(0.0, &mass, |_| (Vec3::new(1e9, 0.0, 0.0), Vec3::new(0.0, 1e9, 0.0)));
        let after = rb.state();
        assert_eq!(before.velocity_body_m_s, after.velocity_body_m_s);
        assert_eq!(before.position_world_m, after.position_world_m);
    }

    #[test]
    fn stays_stable_over_many_ticks_at_the_emulators_dt() {
        let mut rb = RigidBody::new(RigidBodyState { velocity_body_m_s: Vec3::new(230.0, 0.0, 0.0), ..Default::default() });
        let mass = empty_mass();
        for i in 0..2000 {
            let wobble = Vec3::new(0.0, (i as f64 * 0.01).sin() * 1_000_000.0, (i as f64 * 0.013).cos() * 1_000_000.0);
            rb.step(0.05, &mass, |_| (Vec3::ZERO, wobble));
        }
        let s = rb.state();
        assert!(s.velocity_body_m_s.norm().is_finite());
        assert!(s.rate_body_rad_s.norm().is_finite());
        assert!((s.attitude.norm() - 1.0).abs() < 1e-6);
    }
}
