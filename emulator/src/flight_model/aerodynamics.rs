//! Lift, drag, side-force and their moments, built up from the wing,
//! horizontal tail and vertical tail as separate lifting surfaces (the
//! wing and horizontal tail each split left/right so aileron, elevator,
//! spoiler and gear-driven roll damping asymmetries and jams have separate
//! surfaces to act on), plus control-surface deflections (through
//! `actuator.rs`, so jam/runaway/float faults are real actuator states,
//! not scripted symptoms), flaps/slats, spoilers, ground effect, and
//! icing/damage penalties.
//!
//! ## Method
//!
//! Each lifting surface is one "strip": a local relative-airflow vector
//! (freestream plus the rotational velocity `omega x r` at that surface's
//! own arm from the CG -- the same term that gives a real aircraft its
//! `Cl_p`/`Cm_q`/`Cn_r` damping derivatives, so those emerge here from
//! geometry and rotation rate rather than being separate scripted
//! coefficients), a local angle of attack (or sideslip, for the vertical
//! tail) from that vector, and a lift-curve slope from finite-wing theory
//! (Raymer, "Aircraft Design: A Conceptual Approach", 6th ed., eq. 12.6 --
//! the standard subsonic swept/compressible finite-wing lift-curve-slope
//! estimate; `k=0.95` airfoil efficiency is Raymer's own typical value,
//! GENERIC for this airframe). The resulting lift/drag pair is rotated
//! from wind axes into the body frame with the standard 2-D wind-to-body
//! transform (Stevens & Lewis, "Aircraft Control and Simulation", 3rd ed.,
//! eq. 2.3-7, applied per-plane: the wing/tail strips in the body xz
//! plane, the vertical tail in the body xy plane using sideslip in the
//! angle-of-attack's place -- a lifting surface rotated 90 degrees behaves
//! the same way, which is exactly why this reuses one function for both).
//! Each strip's force is placed at its own aerodynamic-centre arm from the
//! CG (`geometry.rs`, `GENERIC` where noted) and summed into a moment: no
//! separate `Cl_beta`/`Cn_beta`/`Cm_alpha` coefficient is hand-picked
//! anywhere -- the restoring/destabilizing moments are consequences of
//! where the lift acts, exactly the brief's "never scripted" requirement.
//!
//! Parasite (zero-lift) drag is **not** split per surface: a single
//! whole-aircraft `CD0` (GENERIC, Raymer's wetted-area-ratio method: a
//! transport aircraft's wetted area is about 6x its wing reference area,
//! `Cf~0.003` turbulent flat-plate skin friction, so `CD0 ~= 6*0.003 =
//! 0.018`, rounded to 0.020 to include interference drag) is applied as a
//! single drag force through the CG (no moment) -- the same simplification
//! most conceptual-design-level tools make for the "form drag" bucket, and
//! a reasonable one here since lift-driven moments dominate trim/stability
//! while parasite-drag moments are a secondary pitching effect.
//!
//! ## Why actuators and forces are two separate calls
//!
//! [`RigidBody::step`](super::rigid_body::RigidBody::step) is an RK4
//! integrator: it evaluates the force/moment callback four times per tick,
//! at four different (intermediate) velocity/attitude states, but the
//! *actuators* (rate-limited, with persistent position state) must only
//! advance **once** per tick -- evaluating them four times would let every
//! surface slew four times too far. [`Aerodynamics::update_actuators`] is
//! therefore called once per tick (`FlightModel::step`) to resolve this
//! tick's actual surface positions, and the pure, state-free
//! [`Aerodynamics::forces`] is what the RK4 callback calls repeatedly.

use super::actuator::{Actuator, SurfaceFault};
use super::atmosphere::AirState;
use super::geometry;
use super::math::Vec3;

// ---------------------------------------------------------------------
// Finite-wing lift-curve slope (Raymer eq. 12.6).
// ---------------------------------------------------------------------

/// Lift-curve slope, per radian, of a finite wing/tail of `aspect_ratio`
/// and leading-edge `sweep_rad`, at `mach`. Clamped below M0.95 --
/// Prandtl-Glauert (the basis of this formula) is a subsonic-only
/// approximation and this model has no transonic/supersonic aerodynamics.
fn finite_wing_cl_alpha(aspect_ratio: f64, sweep_rad: f64, mach: f64) -> f64 {
    let m = mach.abs().clamp(0.0, 0.95);
    let beta2 = (1.0 - m * m).max(0.05);
    const AIRFOIL_EFFICIENCY: f64 = 0.95; // GENERIC, Raymer's typical value.
    let ar = aspect_ratio.max(0.1);
    let tan_sweep2 = sweep_rad.tan().powi(2);
    let inner = 4.0 + (ar * ar * beta2 / (AIRFOIL_EFFICIENCY * AIRFOIL_EFFICIENCY)) * (1.0 + tan_sweep2 / beta2);
    2.0 * std::f64::consts::PI * ar / (2.0 + inner.sqrt())
}

/// The 2-D wind-axes-to-body-frame lift/drag transform (Stevens & Lewis
/// eq. 2.3-7), applied within one plane: `angle` is the local
/// angle-of-attack (wing/tail, in the body xz plane) or sideslip (vertical
/// tail, in the body xy plane). Returns `(force_along_flow_axis,
/// force_along_lift_axis)`; `lift`/`drag` are already `q*S*C`, newtons.
fn wind_to_plane_force(angle_rad: f64, lift_n: f64, drag_n: f64) -> (f64, f64) {
    let (s, c) = angle_rad.sin_cos();
    let along_flow = -drag_n * c + lift_n * s;
    let along_lift = -drag_n * s - lift_n * c;
    (along_flow, along_lift)
}

/// One strip's contribution: local relative airflow, area, lift-curve
/// slope, and a `deflection`-driven CL increment (already scaled by the
/// control's own `effectiveness` and area ratio). `is_vertical` selects
/// whether the "lift axis" is body -z (wing/tail) or body y (fin, using
/// sideslip as its angle of attack). Returns `(force_body_n, local_cl)`.
#[allow(clippy::too_many_arguments)]
fn strip(
    local_velocity_body: Vec3,
    area_m2: f64,
    aspect_ratio: f64,
    sweep_rad: f64,
    mach: f64,
    extra_cl: f64,
    cl_alpha_scale: f64,
    cl_max: f64,
    oswald_e: f64,
    q_density_half: f64,
    is_vertical: bool,
) -> (Vec3, f64) {
    let (a, b) = if is_vertical { (local_velocity_body.x, local_velocity_body.y) } else { (local_velocity_body.x, local_velocity_body.z) };
    let local_speed2 = local_velocity_body.dot(local_velocity_body);
    let angle_rad = b.atan2(a.max(1e-6));
    let cl_alpha = finite_wing_cl_alpha(aspect_ratio, sweep_rad, mach) * cl_alpha_scale;
    let cl_raw = cl_alpha * angle_rad + extra_cl;
    // A simple post-stall model: beyond `cl_max` the surface starts
    // shedding lift roughly linearly back down (a crude but NaN-free
    // stand-in for a real stall polar -- enough to make "stalled" a real,
    // testable state without a wind-tunnel-derived post-stall curve,
    // which is not public for this airframe).
    let cl = if cl_raw.abs() <= cl_max { cl_raw } else { cl_max.copysign(cl_raw) * (1.0 - 0.5 * ((cl_raw.abs() - cl_max) / cl_max.max(0.1)).min(1.0)) };
    let cd_induced = cl * cl / (std::f64::consts::PI * aspect_ratio.max(0.1) * oswald_e.max(0.05));
    let q = q_density_half * local_speed2;
    let lift_n = q * area_m2 * cl;
    let drag_n = q * area_m2 * cd_induced;
    let (along_flow, along_lift) = wind_to_plane_force(angle_rad, lift_n, drag_n);
    let force = if is_vertical { Vec3::new(along_flow, along_lift, 0.0) } else { Vec3::new(along_flow, 0.0, along_lift) };
    (force, cl)
}

// ---------------------------------------------------------------------
// Inputs / outputs.
// ---------------------------------------------------------------------

#[derive(Clone, Copy, Debug, Default)]
pub struct ControlInputs {
    /// Normalized command, -1..1: positive = nose-up, scaled internally by
    /// the elevator's own travel limit (`geometry::elevator_geometry`).
    pub elevator_cmd: f64,
    /// Normalized trim command, -1..1, added to `elevator_cmd` before
    /// the surface limit clamp (the same travel limit; this model does not
    /// give trim its own smaller authority).
    pub elevator_trim_cmd: f64,
    /// Normalized command, -1..1: positive = right-wing-down (right roll);
    /// the left/right ailerons are commanded with opposite sign of this,
    /// each scaled by the aileron's own travel limit.
    pub aileron_cmd: f64,
    /// Normalized command, -1..1: positive = nose-right, scaled internally
    /// by the rudder's own travel limit.
    pub rudder_cmd: f64,
    /// Symmetric speedbrake/ground-spoiler command, 0..1.
    pub spoiler_cmd: f64,
    /// Differential spoiler assist for roll, -1 (right side extra) ..1
    /// (left side extra), added on top of `spoiler_cmd`.
    pub roll_spoiler_cmd: f64,
    pub flap_cmd: f64,
    pub slat_cmd: f64,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct AeroFaults {
    pub elevator_left: SurfaceFault,
    pub elevator_right: SurfaceFault,
    pub aileron_left: SurfaceFault,
    pub aileron_right: SurfaceFault,
    pub rudder: SurfaceFault,
    /// A stuck (jammed) or asymmetrically-deployed spoiler panel is a real,
    /// well-known emergency (uncommanded roll/drag) -- distinct from the
    /// wing/aileron faults above.
    pub spoiler_left: SurfaceFault,
    pub spoiler_right: SurfaceFault,
    /// 0 = no ice, 1 = severe wing ice: reduces CLmax and the lift-curve
    /// slope, and adds parasite drag (GENERIC magnitudes, `forces`'s doc).
    pub wing_ice_fraction: f64,
    /// 0 = no ice, 1 = severe tailplane ice: same penalties applied to the
    /// horizontal tail specifically -- the classic tailplane-icing hazard
    /// (reduced/reversed pitch authority, GENERIC magnitude).
    pub tail_ice_fraction: f64,
    /// 0 = undamaged, 1 = severe airframe aerodynamic damage (skin/rivet
    /// damage, missing fairings, hail/bird-strike denting): reduces CLmax
    /// and adds parasite drag, whole-aircraft (GENERIC magnitude).
    pub airframe_damage_fraction: f64,
}

/// Resolved (post-actuator) control-surface positions for one tick --
/// [`Aerodynamics::update_actuators`]'s output, [`Aerodynamics::forces`]'s
/// input. Radians for the flight controls, 0..1 for flap/slat.
#[derive(Clone, Copy, Debug, Default)]
pub struct ResolvedSurfaces {
    pub elevator_l_rad: f64,
    pub elevator_r_rad: f64,
    pub aileron_l_rad: f64,
    pub aileron_r_rad: f64,
    pub rudder_rad: f64,
    pub spoiler_l: f64,
    pub spoiler_r: f64,
    pub flap: f64,
    pub slat: f64,
}

/// CG-relative arms (metres, body frame) for every lifting surface's
/// aerodynamic centre -- computed by the caller from `geometry.rs`'s
/// static positions and `mass.rs`'s current CG (`mass::arm`), so this
/// module needs no knowledge of the current loading.
#[derive(Clone, Copy, Debug)]
pub struct AeroArms {
    pub wing_left_m: Vec3,
    pub wing_right_m: Vec3,
    pub htail_left_m: Vec3,
    pub htail_right_m: Vec3,
    pub vtail_m: Vec3,
}

/// Everything [`Aerodynamics::forces`] needs about the instantaneous
/// flight condition -- deliberately separate from [`ControlInputs`] (see
/// the module doc): this is evaluated several times per tick at different
/// RK4 stage states, `ControlInputs`/actuators only once.
#[derive(Clone, Copy, Debug)]
pub struct AeroFlightState {
    /// Velocity of the CG relative to the surrounding air, body frame, m/s
    /// (true airspeed vector with wind already subtracted and rotated into
    /// body axes -- `FlightModel::step`'s job).
    pub body_airspeed_m_s: Vec3,
    /// Body angular rate (p, q, r), rad/s.
    pub body_rates_rad_s: Vec3,
    pub air: AirState,
    /// Height above ground, m (ground-effect scaling).
    pub height_agl_m: f64,
    /// 0 (up) .. 1 (down): adds parasite drag through the CG.
    pub gear_extended_fraction: f64,
    pub arms: AeroArms,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct AeroOutputs {
    pub force_body_n: Vec3,
    pub moment_about_cg_n_m: Vec3,
    /// Whole-wing lift coefficient (both halves' average), for diagnostics
    /// and tests.
    pub wing_cl: f64,
    pub stalled: bool,
}

const SPOILER_CL_DUMP: f64 = 0.5; // GENERIC: full deployment kills ~50% of that half's lift.
const SPOILER_CD_ADD: f64 = 0.4; // GENERIC profile-drag coefficient added at full deployment.
const AIRFRAME_CD0: f64 = 0.020; // GENERIC, see module doc.
const GEAR_CD0: f64 = 0.018; // GENERIC, Roskam Part VI-style gear-down drag increment.
const TAIL_EFFICIENCY: f64 = 0.80; // GENERIC oswald-like factor for the (low-AR) tail surfaces.
// GENERIC: ~40 deg/s, typical widebody EHA/EBHA slew rate.
const SURFACE_RATE_RAD_S: f64 = 40.0 * geometry::DEG_TO_RAD;

pub struct Aerodynamics {
    elevator_left: Actuator,
    elevator_right: Actuator,
    aileron_left: Actuator,
    aileron_right: Actuator,
    rudder: Actuator,
    spoiler_left: Actuator,
    spoiler_right: Actuator,
    flap: Actuator,
    slat: Actuator,
}

impl Aerodynamics {
    pub fn new() -> Self {
        Self {
            elevator_left: Actuator::new(),
            elevator_right: Actuator::new(),
            aileron_left: Actuator::new(),
            aileron_right: Actuator::new(),
            rudder: Actuator::new(),
            spoiler_left: Actuator::new(),
            spoiler_right: Actuator::new(),
            flap: Actuator::new(),
            slat: Actuator::new(),
        }
    }

    /// Advances every actuator **once** by `dt_s` toward `controls`,
    /// subject to `faults` (see the module doc for why this is a separate
    /// call from [`Self::forces`]).
    pub fn update_actuators(&mut self, controls: &ControlInputs, faults: &AeroFaults, dt_s: f64) -> ResolvedSurfaces {
        let dt = dt_s.max(0.0);
        let elev_cmd = (controls.elevator_cmd + controls.elevator_trim_cmd).clamp(-1.0, 1.0);
        let elevator_geom = geometry::elevator_geometry();
        let aileron_geom = geometry::aileron_geometry();
        let rudder_geom = geometry::rudder_geometry();
        let elevator_l_rad = self.elevator_left.step(elev_cmd * elevator_geom.limit_rad, &faults.elevator_left, SURFACE_RATE_RAD_S, elevator_geom.limit_rad, dt);
        let elevator_r_rad = self.elevator_right.step(elev_cmd * elevator_geom.limit_rad, &faults.elevator_right, SURFACE_RATE_RAD_S, elevator_geom.limit_rad, dt);
        let aileron_cmd = controls.aileron_cmd.clamp(-1.0, 1.0);
        // A right-roll (right-wing-down) command raises the left wing's lift
        // and spoils the right wing's, so the left aileron gets the command
        // sign directly (positive local deflection = more lift, the same
        // convention `forces` uses for the elevator) and the right aileron
        // gets the opposite.
        let aileron_l_rad = self.aileron_left.step(aileron_cmd * aileron_geom.limit_rad, &faults.aileron_left, SURFACE_RATE_RAD_S, aileron_geom.limit_rad, dt);
        let aileron_r_rad = self.aileron_right.step(-aileron_cmd * aileron_geom.limit_rad, &faults.aileron_right, SURFACE_RATE_RAD_S, aileron_geom.limit_rad, dt);
        let rudder_rad = self.rudder.step(controls.rudder_cmd.clamp(-1.0, 1.0) * rudder_geom.limit_rad, &faults.rudder, SURFACE_RATE_RAD_S, rudder_geom.limit_rad, dt);
        let flap = self.flap.step(controls.flap_cmd.clamp(0.0, 1.0), &SurfaceFault::default(), 0.15, 1.0, dt);
        let slat = self.slat.step(controls.slat_cmd.clamp(0.0, 1.0), &SurfaceFault::default(), 0.3, 1.0, dt);
        let spoiler_l_cmd = (controls.spoiler_cmd + controls.roll_spoiler_cmd).clamp(0.0, 1.0);
        let spoiler_r_cmd = (controls.spoiler_cmd - controls.roll_spoiler_cmd).clamp(0.0, 1.0);
        // GENERIC: ~0.5/s (a 2 s full stroke), typical hydraulic spoiler PCU rate.
        const SPOILER_RATE_S: f64 = 0.5;
        let spoiler_l = self.spoiler_left.step(spoiler_l_cmd, &faults.spoiler_left, SPOILER_RATE_S, 1.0, dt);
        let spoiler_r = self.spoiler_right.step(spoiler_r_cmd, &faults.spoiler_right, SPOILER_RATE_S, 1.0, dt);
        ResolvedSurfaces { elevator_l_rad, elevator_r_rad, aileron_l_rad, aileron_r_rad, rudder_rad, spoiler_l, spoiler_r, flap, slat }
    }

    /// CLmax at MTOW, sea level, 1g, from the cfg's own published stall
    /// speeds (`geometry::FLAPS_UP_STALL_KT`/`FULL_FLAPS_STALL_KT`):
    /// `v_stall = sqrt(2*W / (rho*S*CLmax))`. The cfg does not say what
    /// weight those speeds assume; MTOW/sea-level/ISA is the GENERIC
    /// reference this calibrates against.
    fn clmax(air_sea_level_density: f64) -> (f64, f64) {
        let w = geometry::wing();
        let weight_n = geometry::MTOW_KG * super::atmosphere::G0;
        let clmax_of = |kt: f64| {
            let v = kt * geometry::KT_TO_M_S;
            2.0 * weight_n / (air_sea_level_density * w.area_m2 * v * v)
        };
        (clmax_of(geometry::FLAPS_UP_STALL_KT), clmax_of(geometry::FULL_FLAPS_STALL_KT))
    }

    /// Pure force/moment computation from already-resolved surface
    /// positions and the instantaneous flight state -- no actuator
    /// mutation, safe to call several times per tick (see the module doc).
    pub fn forces(resolved: &ResolvedSurfaces, flight: &AeroFlightState, faults: &AeroFaults) -> AeroOutputs {
        let v = flight.body_airspeed_m_s;
        let tas = v.norm();
        let mach = if flight.air.sound_speed_m_s > 1.0 { tas / flight.air.sound_speed_m_s } else { 0.0 };
        let omega = flight.body_rates_rad_s;

        let elevator_geom = geometry::elevator_geometry();
        let aileron_geom = geometry::aileron_geometry();
        let rudder_geom = geometry::rudder_geometry();

        let (clmax_clean, clmax_full) = Self::clmax(super::atmosphere::isa(0.0, 0.0, 0.0).density_kg_m3);
        let clmax_delta = clmax_full - clmax_clean;
        let wing = geometry::wing();
        let htail = geometry::htail();
        let vtail = geometry::vtail();
        let wing_ice = faults.wing_ice_fraction.clamp(0.0, 1.0);
        let tail_ice = faults.tail_ice_fraction.clamp(0.0, 1.0);
        let damage = faults.airframe_damage_fraction.clamp(0.0, 1.0);
        // Flaps mostly shift the usable CL at a given alpha (added camber);
        // 70% attributed to flaps, 30% to slats (no published breakdown --
        // GENERIC split), slats additionally push CLmax's *reach* out but
        // that is folded into the same additive term for simplicity.
        let high_lift_cl = clmax_delta * (0.7 * resolved.flap + 0.3 * resolved.slat);
        let wing_clmax = (clmax_clean + high_lift_cl) * (1.0 - 0.5 * wing_ice) * (1.0 - 0.3 * damage);
        let wing_cl_alpha_scale = (1.0 - 0.4 * wing_ice) * (1.0 - 0.2 * damage);
        let q_density_half = 0.5 * flight.air.density_kg_m3;

        let side = |arm: Vec3, aileron_rad: f64, spoiler_frac: f64| {
            let local_v = v.add(omega.cross(arm));
            let extra_cl = high_lift_cl
                + finite_wing_cl_alpha(wing.aspect_ratio, wing.sweep_rad, mach) * aileron_rad * aileron_geom.effectiveness * (aileron_geom.area_m2 / (wing.area_m2 / 2.0))
                - SPOILER_CL_DUMP * spoiler_frac.clamp(0.0, 1.0);
            let (f, cl) = strip(local_v, wing.area_m2 / 2.0, wing.aspect_ratio, wing.sweep_rad, mach, extra_cl, wing_cl_alpha_scale, wing_clmax, wing.oswald_e, q_density_half, false);
            let spoiler_drag = q_density_half * local_v.dot(local_v) * (wing.area_m2 / 2.0) * SPOILER_CD_ADD * spoiler_frac.clamp(0.0, 1.0);
            (Vec3::new(f.x - spoiler_drag, f.y, f.z), cl)
        };
        let (wing_l_force, wing_l_cl) = side(flight.arms.wing_left_m, resolved.aileron_l_rad, resolved.spoiler_l);
        let (wing_r_force, wing_r_cl) = side(flight.arms.wing_right_m, resolved.aileron_r_rad, resolved.spoiler_r);

        let htail_cl_alpha_scale = 1.0 - 0.5 * tail_ice;
        let htail_clmax = 0.9 * (1.0 - 0.6 * tail_ice); // GENERIC: tails run a lower CLmax margin than the wing.
        let tail_side = |arm: Vec3, elevator_rad: f64| {
            let local_v = v.add(omega.cross(arm));
            // A positive (nose-up) elevator command deflects the surface
            // trailing-edge-up, the real mechanism that *reduces* the
            // tail's own local lift coefficient (a tail-down force, aft of
            // the CG, is what actually pitches the nose up) -- hence the
            // minus sign, not an arbitrary flip.
            let extra_cl = -finite_wing_cl_alpha(htail.aspect_ratio, htail.sweep_rad, mach) * elevator_rad * elevator_geom.effectiveness * (elevator_geom.area_m2 / htail.area_m2);
            strip(local_v, htail.area_m2 / 2.0, htail.aspect_ratio, htail.sweep_rad, mach, extra_cl, htail_cl_alpha_scale, htail_clmax, TAIL_EFFICIENCY, q_density_half, false)
        };
        let (htail_l_force, _) = tail_side(flight.arms.htail_left_m, resolved.elevator_l_rad);
        let (htail_r_force, _) = tail_side(flight.arms.htail_right_m, resolved.elevator_r_rad);

        let vtail_local_v = v.add(omega.cross(flight.arms.vtail_m));
        let vtail_extra_cl = finite_wing_cl_alpha(vtail.aspect_ratio, vtail.sweep_rad, mach) * resolved.rudder_rad * rudder_geom.effectiveness * (rudder_geom.area_m2 / vtail.area_m2);
        let (vtail_force, _) = strip(vtail_local_v, vtail.area_m2, vtail.aspect_ratio, vtail.sweep_rad, mach, vtail_extra_cl, 1.0, 1.2, TAIL_EFFICIENCY, q_density_half, true);

        // ---- Parasite drag and ground effect: whole-aircraft, through the CG.
        let ground_effect = {
            let h_over_b = (flight.height_agl_m.max(0.0) / wing.span_m.max(1.0)).max(0.01);
            if h_over_b < 1.0 {
                let x = 16.0 * h_over_b;
                x * x / (1.0 + x * x)
            } else {
                1.0
            }
        };
        let cd0 = AIRFRAME_CD0 * (1.0 + 0.5 * wing_ice + 0.8 * damage) + GEAR_CD0 * flight.gear_extended_fraction.clamp(0.0, 1.0);
        let parasite_drag_n = q_density_half * tas * tas * wing.area_m2 * cd0;
        let parasite_force = if tas > 1e-6 { v.scale(-parasite_drag_n / tas) } else { Vec3::ZERO };

        // Ground effect (the classic Wieselsberger factor above) mainly
        // cuts *induced drag*, not lift; the along-flow (body x) component
        // of each strip's force is overwhelmingly the induced-drag term at
        // the low speeds/high alpha where ground effect is significant
        // (climb-out/landing), so it alone is scaled down here, leaving the
        // lift (body z/y) components untouched.
        let lifting_force = wing_l_force.add(wing_r_force).add(htail_l_force).add(htail_r_force).add(vtail_force);
        let lifting_force = Vec3::new(lifting_force.x * ground_effect, lifting_force.y, lifting_force.z);

        let force_body_n = lifting_force.add(parasite_force);
        let moment = flight.arms.wing_left_m.cross(wing_l_force)
            .add(flight.arms.wing_right_m.cross(wing_r_force))
            .add(flight.arms.htail_left_m.cross(htail_l_force))
            .add(flight.arms.htail_right_m.cross(htail_r_force))
            .add(flight.arms.vtail_m.cross(vtail_force));

        let wing_cl = 0.5 * (wing_l_cl + wing_r_cl);
        let stalled = wing_cl.abs() >= wing_clmax * 0.999;

        AeroOutputs { force_body_n, moment_about_cg_n_m: moment, wing_cl, stalled }
    }
}

impl Default for Aerodynamics {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use super::super::atmosphere::isa;

    fn arms() -> AeroArms {
        AeroArms {
            wing_left_m: geometry::wing_half_center_m(false),
            wing_right_m: geometry::wing_half_center_m(true),
            htail_left_m: geometry::htail_half_center_m(false),
            htail_right_m: geometry::htail_half_center_m(true),
            vtail_m: geometry::vtail().position_m,
        }
    }

    fn cruise_flight(alpha_rad: f64) -> AeroFlightState {
        let tas = 230.0;
        AeroFlightState {
            body_airspeed_m_s: Vec3::new(tas * alpha_rad.cos(), 0.0, tas * alpha_rad.sin()),
            body_rates_rad_s: Vec3::ZERO,
            air: isa(11_000.0, 0.0, 0.0),
            height_agl_m: 11_000.0,
            gear_extended_fraction: 0.0,
            arms: arms(),
        }
    }

    /// Runs one tick end to end: actuators, then the pure force call --
    /// what `FlightModel::step` does every RK4 stage in miniature.
    fn run(aero: &mut Aerodynamics, controls: &ControlInputs, flight: &AeroFlightState, faults: &AeroFaults, dt: f64) -> AeroOutputs {
        let resolved = aero.update_actuators(controls, faults, dt);
        Aerodynamics::forces(&resolved, flight, faults)
    }

    #[test]
    fn positive_alpha_produces_upward_lift_no_nan() {
        let mut aero = Aerodynamics::new();
        let out = run(&mut aero, &ControlInputs::default(), &cruise_flight(0.05), &AeroFaults::default(), 0.05);
        assert!(out.force_body_n.z < 0.0, "lift should act in body -z (up): {:?}", out.force_body_n);
        assert!(out.force_body_n.x.is_finite() && out.moment_about_cg_n_m.x.is_finite());
    }

    #[test]
    fn zero_airspeed_produces_no_nan_and_near_zero_force() {
        let mut aero = Aerodynamics::new();
        let mut flight = cruise_flight(0.0);
        flight.body_airspeed_m_s = Vec3::ZERO;
        let out = run(&mut aero, &ControlInputs::default(), &flight, &AeroFaults::default(), 0.05);
        assert!(out.force_body_n.x.is_finite() && out.force_body_n.y.is_finite() && out.force_body_n.z.is_finite());
        assert!(out.force_body_n.norm() < 1.0);
    }

    #[test]
    fn a_right_roll_command_produces_a_positive_roll_moment() {
        let mut aero = Aerodynamics::new();
        let controls = ControlInputs { aileron_cmd: 0.3, ..Default::default() };
        let flight = cruise_flight(0.03);
        let mut out = AeroOutputs::default();
        for _ in 0..200 {
            out = run(&mut aero, &controls, &flight, &AeroFaults::default(), 0.05);
        }
        // Body axes are x-forward, y-right, z-down, so a positive rotation
        // about +x carries +y (right) toward +z (down): right wing down,
        // left wing up -- exactly the commanded right roll. A positive
        // aileron command must therefore produce a positive Mx, not just a
        // nonzero one (this caught a left/right sign swap during
        // development: both directions give "a real moment" but only one
        // is the commanded one).
        assert!(out.moment_about_cg_n_m.x > 1000.0, "positive aileron command should roll right (+Mx): {}", out.moment_about_cg_n_m.x);
    }

    #[test]
    fn a_nose_up_elevator_command_produces_a_positive_pitching_moment() {
        let mut aero = Aerodynamics::new();
        let controls = ControlInputs { elevator_cmd: 0.3, ..Default::default() };
        let flight = cruise_flight(0.02);
        let mut out = AeroOutputs::default();
        for _ in 0..200 {
            out = run(&mut aero, &controls, &flight, &AeroFaults::default(), 0.05);
        }
        // Positive rotation about body +y carries +z (down) toward +x
        // (forward), i.e. the nose (+x) rises toward -z (up) under
        // positive q -- so a nose-up command must give a positive My.
        assert!(out.moment_about_cg_n_m.y > 1000.0, "nose-up elevator command should give +My: {}", out.moment_about_cg_n_m.y);
    }

    #[test]
    fn a_jammed_elevator_stops_responding_to_new_commands() {
        let mut aero = Aerodynamics::new();
        let flight = cruise_flight(0.02);
        let mut faults = AeroFaults::default();
        let mut controls = ControlInputs { elevator_cmd: 0.2, ..Default::default() };
        for _ in 0..100 {
            run(&mut aero, &controls, &flight, &faults, 0.05);
        }
        faults.elevator_left.jam_fraction = 1.0;
        faults.elevator_right.jam_fraction = 1.0;
        let before = run(&mut aero, &controls, &flight, &faults, 0.05).moment_about_cg_n_m.y;
        controls.elevator_cmd = -0.3;
        let mut after = 0.0;
        for _ in 0..50 {
            after = run(&mut aero, &controls, &flight, &faults, 0.05).moment_about_cg_n_m.y;
        }
        assert!((before - after).abs() < before.abs().max(1.0) * 0.05, "jammed elevator should not track the new command: {before} vs {after}");
    }

    #[test]
    fn a_stuck_deployed_left_spoiler_rolls_the_aircraft_right() {
        // An asymmetric spoiler deployment (a real emergency) should dump
        // lift on one side only and roll toward it -- with no aileron
        // command at all, so the moment comes purely from the jam.
        let mut aero = Aerodynamics::new();
        let mut faults = AeroFaults::default();
        faults.spoiler_left.jam_fraction = 1.0;
        // Command the left spoiler up first so it has somewhere to freeze
        // (a jam at zero deployment would have no effect to test).
        let controls_deploy = ControlInputs { spoiler_cmd: 0.0, roll_spoiler_cmd: 1.0, ..Default::default() };
        for _ in 0..20 {
            aero.update_actuators(&controls_deploy, &AeroFaults::default(), 0.05);
        }
        let flight = cruise_flight(0.03);
        let controls_retract = ControlInputs::default();
        let mut out = AeroOutputs::default();
        for _ in 0..50 {
            out = run(&mut aero, &controls_retract, &flight, &faults, 0.05);
        }
        // Left spoiler stuck deployed => less lift on the left wing => left
        // wing drops, right wing rises => a *negative* Mx (the opposite of
        // the right-roll-command sign convention proven above).
        assert!(out.moment_about_cg_n_m.x < -1000.0, "a stuck left spoiler should roll left (-Mx): {}", out.moment_about_cg_n_m.x);
    }

    #[test]
    fn wing_icing_reduces_the_achievable_lift_coefficient() {
        let mut healthy = Aerodynamics::new();
        let mut iced = Aerodynamics::new();
        let flight = cruise_flight(0.12);
        let controls = ControlInputs::default();
        let healthy_out = run(&mut healthy, &controls, &flight, &AeroFaults::default(), 0.05);
        let iced_out = run(&mut iced, &controls, &flight, &AeroFaults { wing_ice_fraction: 1.0, ..Default::default() }, 0.05);
        assert!(iced_out.wing_cl < healthy_out.wing_cl, "ice should reduce lift at the same alpha: {} vs {}", iced_out.wing_cl, healthy_out.wing_cl);
    }

    #[test]
    fn full_rudder_yaws_the_nose_right() {
        let mut aero = Aerodynamics::new();
        let controls = ControlInputs { rudder_cmd: 1.0, ..Default::default() };
        let flight = cruise_flight(0.0);
        let mut out = AeroOutputs::default();
        for _ in 0..200 {
            out = run(&mut aero, &controls, &flight, &AeroFaults::default(), 0.05);
        }
        assert!(out.moment_about_cg_n_m.z.is_finite());
        // Positive rotation about body +z (down) carries +x (forward)
        // toward +y (right): a nose-right command must give a positive Nz.
        assert!(out.moment_about_cg_n_m.z > 1000.0, "positive rudder command should yaw the nose right (+Nz): {}", out.moment_about_cg_n_m.z);
    }

    #[test]
    fn two_forces_calls_with_the_same_resolved_surfaces_are_identical() {
        // Confirms `forces` really is a pure function of (resolved, flight,
        // faults) -- no hidden actuator mutation -- which is the whole
        // point of the update_actuators/forces split for RK4.
        let mut aero = Aerodynamics::new();
        let resolved = aero.update_actuators(&ControlInputs { elevator_cmd: 0.4, ..Default::default() }, &AeroFaults::default(), 0.05);
        let flight = cruise_flight(0.03);
        let a = Aerodynamics::forces(&resolved, &flight, &AeroFaults::default());
        let b = Aerodynamics::forces(&resolved, &flight, &AeroFaults::default());
        assert_eq!(a.force_body_n, b.force_body_n);
        assert_eq!(a.moment_about_cg_n_m, b.moment_about_cg_n_m);
    }
}
