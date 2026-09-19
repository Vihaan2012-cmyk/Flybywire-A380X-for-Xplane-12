//! Minimal 3-vector and unit-quaternion algebra for the rigid-body model.
//!
//! No external crate (`std` only, per the workstream's hard rules), so this
//! reimplements the handful of operations [`super::rigid_body`] needs.
//! Quaternion convention: `q` rotates a vector from **body** axes to the
//! **world** (NED-style, flat-earth) axes, i.e. `v_world = q.rotate(v_body)`.
//! Body axes are SAE aerospace: x forward, y right, z down (see
//! `geometry.rs`'s module doc for why the airframe's MSFS-convention
//! coordinates are converted into this frame). The Euler <-> quaternion
//! formulas are the standard aerospace 3-2-1 (yaw, pitch, roll) sequence,
//! e.g. J. Diebel, "Representing Attitude: Euler Angles, Unit Quaternions,
//! and Rotation Vectors" (2006), eq. 67 and its inverse.

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Vec3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Vec3 {
    pub const ZERO: Vec3 = Vec3 { x: 0.0, y: 0.0, z: 0.0 };

    pub fn new(x: f64, y: f64, z: f64) -> Self {
        Self { x, y, z }
    }

    pub fn dot(self, o: Vec3) -> f64 {
        self.x * o.x + self.y * o.y + self.z * o.z
    }

    pub fn cross(self, o: Vec3) -> Vec3 {
        Vec3::new(self.y * o.z - self.z * o.y, self.z * o.x - self.x * o.z, self.x * o.y - self.y * o.x)
    }

    pub fn norm(self) -> f64 {
        self.dot(self).sqrt()
    }

    pub fn scale(self, k: f64) -> Vec3 {
        Vec3::new(self.x * k, self.y * k, self.z * k)
    }

    pub fn add(self, o: Vec3) -> Vec3 {
        Vec3::new(self.x + o.x, self.y + o.y, self.z + o.z)
    }

    pub fn sub(self, o: Vec3) -> Vec3 {
        Vec3::new(self.x - o.x, self.y - o.y, self.z - o.z)
    }

    pub fn neg(self) -> Vec3 {
        Vec3::new(-self.x, -self.y, -self.z)
    }

    /// A unit vector along `self`, or `fallback` if `self` is (numerically)
    /// zero-length -- keeps every caller NaN-free at rest.
    pub fn normalized_or(self, fallback: Vec3) -> Vec3 {
        let n = self.norm();
        if n > 1e-9 {
            self.scale(1.0 / n)
        } else {
            fallback
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Quat {
    pub w: f64,
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Default for Quat {
    fn default() -> Self {
        Quat::IDENTITY
    }
}

impl Quat {
    pub const IDENTITY: Quat = Quat { w: 1.0, x: 0.0, y: 0.0, z: 0.0 };

    /// Body-to-world quaternion for the standard aerospace 3-2-1 Euler
    /// sequence (yaw about world z, then pitch about the new y, then roll
    /// about the new x), all in radians.
    pub fn from_euler(roll: f64, pitch: f64, yaw: f64) -> Self {
        let (sr, cr) = (roll * 0.5).sin_cos();
        let (sp, cp) = (pitch * 0.5).sin_cos();
        let (sy, cy) = (yaw * 0.5).sin_cos();
        Quat {
            w: cr * cp * cy + sr * sp * sy,
            x: sr * cp * cy - cr * sp * sy,
            y: cr * sp * cy + sr * cp * sy,
            z: cr * cp * sy - sr * sp * cy,
        }
    }

    /// Inverse of [`Self::from_euler`]: roll, pitch, yaw, radians. Guards
    /// the pitch = +/-90 deg gimbal singularity by clamping `asin`'s
    /// argument.
    pub fn to_euler(self) -> (f64, f64, f64) {
        let (w, x, y, z) = (self.w, self.x, self.y, self.z);
        let roll = (2.0 * (w * x + y * z)).atan2(1.0 - 2.0 * (x * x + y * y));
        let sin_pitch = (2.0 * (w * y - z * x)).clamp(-1.0, 1.0);
        let pitch = sin_pitch.asin();
        let yaw = (2.0 * (w * z + x * y)).atan2(1.0 - 2.0 * (y * y + z * z));
        (roll, pitch, yaw)
    }

    pub fn norm(self) -> f64 {
        (self.w * self.w + self.x * self.x + self.y * self.y + self.z * self.z).sqrt()
    }

    /// Normalizes, falling back to identity if numerically degenerate (dt=0
    /// or a corrupted state should never propagate a NaN attitude).
    pub fn normalized(self) -> Self {
        let n = self.norm();
        if n > 1e-9 {
            Quat { w: self.w / n, x: self.x / n, y: self.y / n, z: self.z / n }
        } else {
            Quat::IDENTITY
        }
    }

    /// Hamilton product `self * o`.
    pub fn mul(self, o: Quat) -> Quat {
        Quat {
            w: self.w * o.w - self.x * o.x - self.y * o.y - self.z * o.z,
            x: self.w * o.x + self.x * o.w + self.y * o.z - self.z * o.y,
            y: self.w * o.y - self.x * o.z + self.y * o.w + self.z * o.x,
            z: self.w * o.z + self.x * o.y - self.y * o.x + self.z * o.w,
        }
    }

    /// Rotates `v` (body axes) into world axes: `q * [0,v] * q^-1`, expanded
    /// with the standard closed-form (avoids building the conjugate quaternion).
    pub fn rotate(self, v: Vec3) -> Vec3 {
        let qv = Vec3::new(self.x, self.y, self.z);
        let t = qv.cross(v).scale(2.0);
        v.add(t.scale(self.w)).add(qv.cross(t))
    }

    /// Rotates `v` from world axes into body axes (the inverse of
    /// [`Self::rotate`]; for a unit quaternion the inverse is the
    /// conjugate).
    pub fn rotate_inverse(self, v: Vec3) -> Vec3 {
        self.conjugate().rotate(v)
    }

    pub fn conjugate(self) -> Quat {
        Quat { w: self.w, x: -self.x, y: -self.y, z: -self.z }
    }

    /// `dq/dt = 1/2 q (x) [0, omega_body]`, the kinematic relation between a
    /// body's angular rate and its attitude quaternion's derivative
    /// (Diebel 2006, eq. 26; also Stevens & Lewis, "Aircraft Control and
    /// Simulation", 3rd ed., eq. 1.4-14).
    pub fn derivative(self, omega_body: Vec3) -> Quat {
        let omega_q = Quat { w: 0.0, x: omega_body.x, y: omega_body.y, z: omega_body.z };
        let p = self.mul(omega_q);
        Quat { w: 0.5 * p.w, x: 0.5 * p.x, y: 0.5 * p.y, z: 0.5 * p.z }
    }

    pub fn add_scaled(self, d: Quat, k: f64) -> Quat {
        Quat { w: self.w + d.w * k, x: self.x + d.x * k, y: self.y + d.y * k, z: self.z + d.z * k }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn identity_round_trips_euler() {
        let (r, p, y) = Quat::IDENTITY.to_euler();
        assert!(r.abs() < 1e-12 && p.abs() < 1e-12 && y.abs() < 1e-12);
    }

    #[test]
    fn ninety_degree_yaw_turns_forward_into_the_world_y_axis() {
        let q = Quat::from_euler(0.0, 0.0, std::f64::consts::FRAC_PI_2);
        let world = q.rotate(Vec3::new(1.0, 0.0, 0.0));
        assert!((world.x).abs() < 1e-9);
        assert!((world.y - 1.0).abs() < 1e-9);
        assert!((world.z).abs() < 1e-9);
    }

    #[test]
    fn euler_round_trips_through_the_quaternion() {
        let (roll, pitch, yaw) = (0.2, -0.35, 1.1);
        let q = Quat::from_euler(roll, pitch, yaw);
        let (r2, p2, y2) = q.to_euler();
        assert!((roll - r2).abs() < 1e-9 && (pitch - p2).abs() < 1e-9 && (yaw - y2).abs() < 1e-9);
    }

    #[test]
    fn rotate_and_rotate_inverse_are_mutual_inverses() {
        let q = Quat::from_euler(0.3, 0.2, -0.7).normalized();
        let v = Vec3::new(3.0, -2.0, 1.0);
        let round_trip = q.rotate_inverse(q.rotate(v));
        assert!((round_trip.x - v.x).abs() < 1e-9);
        assert!((round_trip.y - v.y).abs() < 1e-9);
        assert!((round_trip.z - v.z).abs() < 1e-9);
    }

    #[test]
    fn zero_vector_normalizes_to_the_fallback_without_nan() {
        let n = Vec3::ZERO.normalized_or(Vec3::new(0.0, 0.0, 1.0));
        assert_eq!(n, Vec3::new(0.0, 0.0, 1.0));
    }
}
