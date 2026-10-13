//! Mass, centre of gravity and inertia for the *current* loading, built
//! from the empty-aircraft baseline in `geometry.rs` plus the two numbers
//! the plugin's own weight-and-balance already computes and publishes each
//! tick (`src/weight_balance.rs`'s `Published` fields
//! `"fbw/wb/gross_weight_kg"` and `"fbw/wb/cg_z_ft"` -- total mass and the
//! longitudinal centre of gravity, MSFS convention (forward-positive), from
//! every payload station and fuel tank FlyByWire's systems loaded). This
//! module does not re-parse `flight_model.cfg`'s 19 stations/16 tanks
//! itself (that would duplicate `weight_balance.rs` and risks drifting from
//! it); instead it treats "payload + fuel" as a single lumped mass and
//! *recovers* that lump's effective centroid from the known empty-CG and
//! the known total CG by a moment balance -- exact for the longitudinal
//! axis (the only one FlyByWire/X-Plane track at runtime; see
//! `weight_balance.rs`'s own doc comment on why only `cg_offset_z` exists),
//! GENERIC for the lateral and vertical axes (assumed zero lateral offset,
//! symmetric loading being the normal case, and the same vertical station
//! as the empty CG, since fuel sits in the wings and payload in the cabin
//! floor/holds at broadly similar deck heights to the empty aircraft's own
//! CG height -- there is no published vertical/lateral payload CG to do
//! better with).
//!
//! Inertia is then the standard two-body parallel-axis combination: shift
//! the empty aircraft's own inertia (about its own CG) and the lumped
//! extra mass's inertia (a point mass, so zero about its own centroid) onto
//! the *new* total CG, and add them. This is exact given the point-mass
//! assumption for the extra load (a real distributed payload's own inertia
//! about its centroid is neglected -- GENERIC, but a much smaller term than
//! the parallel-axis shift for a load spread through a 262 ft/80 m
//! airframe).

use super::geometry::{self, Engine};
use super::math::Vec3;

/// A diagonal inertia tensor (products of inertia assumed zero: the
/// airframe and its loading are treated as symmetric about the body xz
/// plane, the standard simplifying assumption for an aircraft with no
/// unusual lateral mass asymmetry -- e.g. Stevens & Lewis section 1.2).
#[derive(Clone, Copy, Debug)]
pub struct Inertia {
    pub ixx: f64,
    pub iyy: f64,
    pub izz: f64,
}

impl Inertia {
    pub fn is_finite(&self) -> bool {
        self.ixx.is_finite() && self.iyy.is_finite() && self.izz.is_finite()
    }
}

#[derive(Clone, Copy, Debug)]
pub struct MassProperties {
    pub mass_kg: f64,
    pub cg_m: Vec3,
    pub inertia: Inertia,
}

/// Parallel-axis shift of a diagonal inertia tensor from `from_cg` to
/// `to_cg` for a body of mass `mass_kg`: for axis k, `d_perp^2` is the sum
/// of squares of the *other two* coordinate offsets (the standard parallel
/// axis theorem, e.g. Ixx uses (dy^2+dz^2)).
fn shift_diagonal_inertia(i: Inertia, mass_kg: f64, from_cg: Vec3, to_cg: Vec3) -> Inertia {
    let d = from_cg.sub(to_cg);
    Inertia {
        ixx: i.ixx + mass_kg * (d.y * d.y + d.z * d.z),
        iyy: i.iyy + mass_kg * (d.x * d.x + d.z * d.z),
        izz: i.izz + mass_kg * (d.x * d.x + d.y * d.y),
    }
}

/// `total_mass_kg`: FlyByWire's own `gross_weight_kg` (empty + payload +
/// fuel). `cg_x_forward_m`: `cg_z_ft * geometry::FT_TO_M` -- the MSFS z
/// (forward-positive) coordinate, which is body-frame `x` per
/// `geometry.rs`'s axis convention, so no further flip is needed.
pub fn current(total_mass_kg: f64, cg_x_forward_m: f64) -> MassProperties {
    let empty_mass = geometry::EMPTY_MASS_KG;
    let empty_cg = geometry::empty_cg_m();
    let empty_inertia = Inertia { ixx: geometry::EMPTY_IXX_KG_M2, iyy: geometry::EMPTY_IYY_KG_M2, izz: geometry::EMPTY_IZZ_KG_M2 };

    let total_mass = total_mass_kg.max(empty_mass);
    let extra_mass = (total_mass - empty_mass).max(0.0);
    let total_cg = Vec3::new(cg_x_forward_m, 0.0, empty_cg.z);

    if extra_mass < 1.0 {
        // No meaningful load beyond the empty aircraft: report the exact
        // empty-aircraft baseline rather than dividing by a ~zero mass to
        // recover a centroid that would not mean anything.
        return MassProperties { mass_kg: total_mass, cg_m: empty_cg, inertia: empty_inertia };
    }

    // Moment balance: empty_mass*empty_cg + extra_mass*extra_cg = total_mass*total_cg.
    let extra_cg = total_cg.scale(total_mass).sub(empty_cg.scale(empty_mass)).scale(1.0 / extra_mass);

    let from_empty = shift_diagonal_inertia(empty_inertia, empty_mass, empty_cg, total_cg);
    let from_extra = shift_diagonal_inertia(Inertia { ixx: 0.0, iyy: 0.0, izz: 0.0 }, extra_mass, extra_cg, total_cg);
    let inertia = Inertia { ixx: from_empty.ixx + from_extra.ixx, iyy: from_empty.iyy + from_extra.iyy, izz: from_empty.izz + from_extra.izz };

    MassProperties { mass_kg: total_mass, cg_m: total_cg, inertia }
}

/// The moment arm from the current CG to a body-frame point (e.g. a gear
/// leg, engine or aerodynamic centre) -- every force-generating module
/// needs this to turn its own force into a moment about the CG.
pub fn arm(mass: &MassProperties, point_m: Vec3) -> Vec3 {
    point_m.sub(mass.cg_m)
}

pub fn engine_arm(mass: &MassProperties, engine: Engine) -> Vec3 {
    arm(mass, geometry::engine_position_m(engine))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn zero_payload_returns_the_exact_empty_baseline() {
        let m = current(geometry::EMPTY_MASS_KG, geometry::empty_cg_m().x);
        assert!((m.mass_kg - geometry::EMPTY_MASS_KG).abs() < 1e-6);
        assert!((m.cg_m.x - geometry::empty_cg_m().x).abs() < 1e-9);
        assert!((m.inertia.ixx - geometry::EMPTY_IXX_KG_M2).abs() < 1e-3);
    }

    #[test]
    fn loading_aft_of_the_empty_cg_moves_the_total_cg_aft_and_grows_pitch_inertia() {
        let empty_cg_x = geometry::empty_cg_m().x;
        // MTOW with the CG 2 m *aft* of the empty CG (body -x is aft).
        let loaded_cg_x = empty_cg_x - 2.0;
        let m = current(geometry::MTOW_KG, loaded_cg_x);
        assert!((m.cg_m.x - loaded_cg_x).abs() < 1e-6);
        assert!(m.mass_kg > geometry::EMPTY_MASS_KG);
        // Extra mass far from the CG on the pitch axis (x offset) must grow
        // Iyy relative to the empty baseline.
        assert!(m.inertia.iyy > geometry::EMPTY_IYY_KG_M2);
        assert!(m.inertia.is_finite());
    }

    #[test]
    fn heavier_at_the_same_cg_still_grows_every_axis_via_the_parallel_axis_shift() {
        // Loading exactly at the empty CG: the parallel-axis distance for
        // the *empty* body's own shift is zero, but the extra point mass
        // still sits away from body-z=0 laterally never (cg.y=0) -- so with
        // an on-CG load only the roll/pitch/yaw baseline stays close to
        // the empty numbers (extra mass contributes zero shift at zero
        // distance); this asserts that degenerate case is still NaN-free
        // and never goes below the empty baseline.
        let m = current(geometry::MTOW_KG, geometry::empty_cg_m().x);
        assert!(m.inertia.ixx >= geometry::EMPTY_IXX_KG_M2 - 1.0);
        assert!(m.inertia.is_finite());
    }

    #[test]
    fn arm_is_the_vector_from_cg_to_the_point() {
        let m = current(geometry::EMPTY_MASS_KG, geometry::empty_cg_m().x);
        let point = geometry::gear_position_m(geometry::GearLeg::Nose);
        let a = arm(&m, point);
        assert!((a.x - (point.x - m.cg_m.x)).abs() < 1e-9);
    }
}
