//! Landing gear ground contact: a spring-damper per leg (the oleo strut),
//! tyre friction (a simplified/"lite" Pacejka Magic-Formula brush-tyre
//! shape -- H. Pacejka, "Tyre and Vehicle Dynamics"), braking, nose-wheel
//! steering, runway contamination, per-leg structural collapse, and the
//! body tailstrike contact point -- everything backlog item 4 asked for.
//!
//! ## Contact model
//!
//! Each of the five legs (`geometry::GEAR_LEGS`: nose, two wing, two body
//! -- the real A380's own gear arrangement) is one point: its world
//! position and velocity are found from the rigid body's own state and the
//! leg's CG-relative arm (the same `omega x r` construction
//! `aerodynamics.rs` uses for rotational strip velocities). Penetration
//! below a flat ground plane compresses a spring-damper (GENERIC
//! stiffness/damping, sized so each leg's *static* share of MTOW compresses
//! it about 0.30 m -- Currey, "Aircraft Landing Gear Design", typical
//! widebody oleo static travel -- at a 0.7 damping ratio, a typical
//! shock-strut design target). Beyond a further 0.15 m of travel the strut
//! "bottoms": stiffness rises sharply, a crude but NaN-free stand-in for
//! metal-to-metal contact.
//!
//! Tyre friction is split into longitudinal (rolling/braking) and lateral
//! (cornering) components, each `mu(slip) = mu_peak * sin(C * atan(B *
//! slip))` -- the core shape of Pacejka's Magic Formula without its
//! camber/combined-slip refinements (GENERIC `B`/`C` chosen so peak
//! friction falls near 10-15% slip, the commonly cited range for aircraft
//! tyres), combined under one friction-circle cap. `mu_peak` itself (0.8
//! dry, scaled down by the caller's `runway_mu_factor` for wet/contaminated
//! conditions) is the commonly published dry-pavement aircraft braking
//! coefficient range (FAA AC 91-79A / ICAO's Global Reporting Format
//! braking-action bands use the same 0.05 (ice) .. 0.8 (dry) span).
//!
//! Everything here is computed algebraically from the *current* rigid-body
//! state each tick (no per-leg integrator state beyond the nose-steering
//! actuator) -- a real oleo's compression is exactly this: an algebraic
//! function of how far the wheel has penetrated and how fast, not a
//! separately-integrated variable.

use super::actuator::{Actuator, SurfaceFault};
use super::geometry::{self, GearLeg, GEAR_LEGS};
use super::math::{Quat, Vec3};

const NOSE_WEIGHT_FRACTION: f64 = 0.10; // GENERIC, typical transport nose-gear static load share (Raymer ch. 11: 8-15%).
const STATIC_COMPRESSION_M: f64 = 0.30; // GENERIC oleo static travel, widebody-typical (Currey).
const MAX_TRAVEL_M: f64 = 0.45; // GENERIC full stroke, ~1.5x static (Currey's typical ratio).
const DAMPING_RATIO: f64 = 0.7; // GENERIC oleo design target (Currey).
const BOTTOM_STIFFNESS_MULT: f64 = 20.0; // GENERIC: sharp stiffening once bottomed out.

const MU_PEAK_DRY: f64 = 0.8; // GENERIC, FAA AC 91-79A / ICAO GRF "dry" band.
const MU_ROLL: f64 = 0.02; // GENERIC free-rolling resistance coefficient.
const SLIP_B: f64 = 10.0; // GENERIC Pacejka-lite stiffness factor.
const SLIP_C: f64 = 1.9; // GENERIC Pacejka-lite shape factor.
const STEERING_LIMIT_RAD: f64 = 75.0 * geometry::DEG_TO_RAD; // GENERIC, typical widebody nosewheel steering travel.
const STEERING_RATE_RAD_S: f64 = 0.7; // GENERIC nosewheel steering slew rate.

fn leg_weight_fraction(leg: GearLeg) -> f64 {
    match leg {
        GearLeg::Nose => NOSE_WEIGHT_FRACTION,
        _ => (1.0 - NOSE_WEIGHT_FRACTION) / 4.0,
    }
}

fn leg_stiffness_n_m(leg: GearLeg) -> f64 {
    geometry::MTOW_KG * leg_weight_fraction(leg) * super::atmosphere::G0 / STATIC_COMPRESSION_M
}

fn leg_damping_n_s_m(leg: GearLeg) -> f64 {
    let k = leg_stiffness_n_m(leg);
    let m = geometry::MTOW_KG * leg_weight_fraction(leg);
    DAMPING_RATIO * 2.0 * (k * m).sqrt()
}

fn is_braked(leg: GearLeg) -> bool {
    !matches!(leg, GearLeg::Nose)
}
fn is_steerable(leg: GearLeg) -> bool {
    matches!(leg, GearLeg::Nose)
}

/// `mu(slip) = mu_peak * sin(C * atan(B * slip))` -- Pacejka-lite.
fn pacejka_lite(slip: f64, mu_peak: f64) -> f64 {
    mu_peak * (SLIP_C * (SLIP_B * slip).atan()).sin()
}

#[derive(Clone, Copy, Debug, Default)]
pub struct GearFaults {
    /// Per leg (`GEAR_LEGS` order), 0 = healthy strut .. 1 = fully
    /// collapsed (no vertical support at all).
    pub collapse_fraction: [f64; 5],
    /// Per leg, 0 = full grip .. 1 = no grip (a blown/deflated tyre).
    pub tyre_friction_loss_fraction: [f64; 5],
    /// Per leg, 0 = full brake authority .. 1 = fully faded (no braking);
    /// meaningless (ignored) on the unbraked nose leg.
    pub brake_fade_fraction: [f64; 5],
    pub nose_steering: SurfaceFault,
}

#[derive(Clone, Copy, Debug)]
pub struct GearInputs {
    /// Rigid-body CG position, world frame (NED-style, z down), metres.
    pub position_world_m: Vec3,
    pub attitude: Quat,
    pub velocity_body_m_s: Vec3,
    pub rate_body_rad_s: Vec3,
    /// World-frame z of the ground plane under the aircraft (flat, level
    /// runway -- GENERIC, no terrain slope modelled).
    pub ground_z_world_m: f64,
    /// 1.0 = dry; scales `MU_PEAK_DRY` down for wet/contaminated runways
    /// (e.g. ~0.6 wet, ~0.35 compacted snow, ~0.1 ice -- FAA AC 91-79A /
    /// ICAO GRF braking-action bands).
    pub runway_mu_factor: f64,
    /// Brake pedal position, 0..1, applied to both main-gear sets equally
    /// (no separate left/right pedal input in this pass).
    pub brake_cmd: f64,
    /// Nosewheel steering tiller/pedal command, -1..1.
    pub nose_steering_cmd: f64,
    /// CG-relative arms for every leg and the tailstrike point, in
    /// `GEAR_LEGS` order plus the tailstrike point last (computed by the
    /// caller from `geometry.rs` and the current `mass::arm`).
    pub arms: [Vec3; 5],
    pub tailstrike_arm_m: Vec3,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct GearOutputs {
    pub force_body_n: Vec3,
    pub moment_about_cg_n_m: Vec3,
    /// Vertical (normal) reaction, one per leg, `GEAR_LEGS` order, newtons.
    pub leg_load_n: [f64; 5],
    pub any_wheel_on_ground: bool,
    pub tailstrike: bool,
    pub tailstrike_load_n: f64,
}

impl GearOutputs {
    pub fn total_weight_on_wheels_n(&self) -> f64 {
        self.leg_load_n.iter().sum()
    }
}

pub struct LandingGear {
    nose_steering: Actuator,
}

impl LandingGear {
    pub fn new() -> Self {
        Self { nose_steering: Actuator::new() }
    }

    fn leg_world_point(inputs: &GearInputs, arm: Vec3) -> Vec3 {
        inputs.position_world_m.add(inputs.attitude.rotate(arm))
    }
    fn leg_velocity_body(inputs: &GearInputs, arm: Vec3) -> Vec3 {
        inputs.velocity_body_m_s.add(inputs.rate_body_rad_s.cross(arm))
    }

    /// One leg's vertical spring-damper reaction (newtons, always >= 0)
    /// and whether it is in contact.
    fn vertical_reaction(leg: GearLeg, inputs: &GearInputs, arm: Vec3, collapse_fraction: f64) -> (f64, bool) {
        let point = Self::leg_world_point(inputs, arm);
        let penetration = point.z - inputs.ground_z_world_m; // world z down: deeper = larger.
        if penetration <= 0.0 {
            return (0.0, false);
        }
        let vel_body = Self::leg_velocity_body(inputs, arm);
        let vel_world = inputs.attitude.rotate(vel_body);
        let closing_rate = vel_world.z; // positive = still compressing.
        let k = leg_stiffness_n_m(leg);
        let c = leg_damping_n_s_m(leg);
        let over = (penetration - MAX_TRAVEL_M).max(0.0);
        let force = k * penetration + c * closing_rate + BOTTOM_STIFFNESS_MULT * k * over * over;
        let collapse = collapse_fraction.clamp(0.0, 1.0);
        (force.max(0.0) * (1.0 - collapse).powi(2), true)
    }

    /// One leg's horizontal (friction) force, body-frame x/y, newtons.
    /// Approximates the runway plane as the body xy plane (exact at zero
    /// pitch/roll; a reasonable approximation for the near-level attitudes
    /// during taxi, takeoff and landing roll where friction dominates).
    #[allow(clippy::too_many_arguments)]
    fn friction(leg: GearLeg, inputs: &GearInputs, arm: Vec3, normal_n: f64, steering_rad: f64, tyre_loss: f64, brake_fade: f64) -> (f64, f64) {
        if normal_n <= 0.0 {
            return (0.0, 0.0);
        }
        let vel_body = Self::leg_velocity_body(inputs, arm);
        let (s, c) = steering_rad.sin_cos();
        // Rotate the contact-point velocity into the tyre's own rolling
        // axes (forward/lateral), so a steered nosewheel corners correctly.
        let v_fwd = vel_body.x * c + vel_body.y * s;
        let v_lat = -vel_body.x * s + vel_body.y * c;
        let mu_peak = MU_PEAK_DRY * inputs.runway_mu_factor.clamp(0.0, 1.0) * (1.0 - tyre_loss.clamp(0.0, 1.0));

        let brake = if is_braked(leg) { inputs.brake_cmd.clamp(0.0, 1.0) * (1.0 - brake_fade.clamp(0.0, 1.0)) } else { 0.0 };
        let mu_long = if brake > 1e-6 { pacejka_lite(brake, mu_peak) } else { MU_ROLL.min(mu_peak) };
        let long_force = -mu_long * normal_n * v_fwd.signum();

        // Cornering: slip angle from the lateral/forward speed ratio (a
        // speed floor avoids a singular slip angle at a dead stop).
        let slip_angle = v_lat.atan2(v_fwd.abs().max(0.5));
        let mu_lat = pacejka_lite(slip_angle / (std::f64::consts::FRAC_PI_2), mu_peak); // slip angle normalized to +/-1 at +/-90 deg.
        let lat_force = -mu_lat * normal_n * 1.0; // sign already carried by slip_angle's sign through pacejka_lite (odd function).

        // Friction circle: never exceed mu_peak*N combined.
        let mag = (long_force * long_force + lat_force * lat_force).sqrt();
        let cap = mu_peak * normal_n;
        let (long_force, lat_force) = if mag > cap && mag > 1e-9 { (long_force * cap / mag, lat_force * cap / mag) } else { (long_force, lat_force) };

        // Back to body axes.
        let fx = long_force * c - lat_force * s;
        let fy = long_force * s + lat_force * c;
        (fx, fy)
    }

    pub fn step(&mut self, inputs: &GearInputs, faults: &GearFaults, dt_s: f64) -> GearOutputs {
        let dt = dt_s.max(0.0);
        let steering_cmd = inputs.nose_steering_cmd.clamp(-1.0, 1.0) * STEERING_LIMIT_RAD;
        let steering_rad = self.nose_steering.step(steering_cmd, &faults.nose_steering, STEERING_RATE_RAD_S, STEERING_LIMIT_RAD, dt);

        let mut force = Vec3::ZERO;
        let mut moment = Vec3::ZERO;
        let mut leg_load_n = [0.0; 5];
        let mut any_on_ground = false;

        for (i, leg) in GEAR_LEGS.iter().enumerate() {
            let arm = inputs.arms[i];
            let (normal_n, on_ground) = Self::vertical_reaction(*leg, inputs, arm, faults.collapse_fraction[i]);
            leg_load_n[i] = normal_n;
            any_on_ground |= on_ground;
            if !on_ground {
                continue;
            }
            let steer = if is_steerable(*leg) { steering_rad } else { 0.0 };
            let (fx, fy) = Self::friction(*leg, inputs, arm, normal_n, steer, faults.tyre_friction_loss_fraction[i], faults.brake_fade_fraction[i]);
            // The normal reaction is a *world* -z (up) force; friction is
            // computed directly in the body xy-plane (see `friction`'s doc).
            let normal_body = inputs.attitude.rotate_inverse(Vec3::new(0.0, 0.0, -normal_n));
            let leg_force = Vec3::new(normal_body.x + fx, normal_body.y + fy, normal_body.z);
            force = force.add(leg_force);
            moment = moment.add(arm.cross(leg_force));
        }

        // Tailstrike: a rigid, high-stiffness/high-damping contact (a scrape,
        // not a shock strut), plus simple sliding friction along body -x
        // opposing forward motion once in contact -- GENERIC, mu=0.3 for
        // metal/composite scraping on pavement (a commonly cited rough
        // order-of-magnitude for unlubricated metal-on-concrete sliding).
        let ts_point = Self::leg_world_point(inputs, inputs.tailstrike_arm_m);
        let ts_penetration = ts_point.z - inputs.ground_z_world_m;
        let (tailstrike, tailstrike_load_n) = if ts_penetration > 0.0 {
            let vel_world = inputs.attitude.rotate(Self::leg_velocity_body(inputs, inputs.tailstrike_arm_m));
            const TS_K: f64 = 5.0e7; // GENERIC: rigid structure, no shock absorption.
            const TS_C: f64 = 2.0e6;
            let normal_n = (TS_K * ts_penetration + TS_C * vel_world.z).max(0.0);
            let normal_body = inputs.attitude.rotate_inverse(Vec3::new(0.0, 0.0, -normal_n));
            let vel_body = Self::leg_velocity_body(inputs, inputs.tailstrike_arm_m);
            let friction_x = -0.3 * normal_n * vel_body.x.signum();
            let leg_force = Vec3::new(normal_body.x + friction_x, normal_body.y, normal_body.z);
            force = force.add(leg_force);
            moment = moment.add(inputs.tailstrike_arm_m.cross(leg_force));
            (true, normal_n)
        } else {
            (false, 0.0)
        };

        GearOutputs { force_body_n: force, moment_about_cg_n_m: moment, leg_load_n, any_wheel_on_ground: any_on_ground, tailstrike, tailstrike_load_n }
    }
}

impl Default for LandingGear {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use super::super::mass;

    fn arms() -> [Vec3; 5] {
        let m = mass::current(geometry::EMPTY_MASS_KG, geometry::empty_cg_m().x);
        let mut out = [Vec3::ZERO; 5];
        for (i, leg) in GEAR_LEGS.iter().enumerate() {
            out[i] = mass::arm(&m, geometry::gear_position_m(*leg));
        }
        out
    }

    fn resting_inputs() -> GearInputs {
        // The five legs' static hardpoints are not at exactly the same
        // height (the cfg's own gear geometry differs leg to leg by up to
        // ~0.2 m), so the ground plane is placed 0.3 m below the *deepest*
        // leg -- comfortably more than that spread -- to guarantee every
        // leg is actually in contact for this test.
        let arms = arms();
        let deepest_z = arms.iter().map(|a| a.z).fold(f64::MIN, f64::max);
        GearInputs {
            position_world_m: Vec3::new(0.0, 0.0, 0.0),
            attitude: Quat::IDENTITY,
            velocity_body_m_s: Vec3::ZERO,
            rate_body_rad_s: Vec3::ZERO,
            ground_z_world_m: deepest_z - 0.3,
            runway_mu_factor: 1.0,
            brake_cmd: 0.0,
            nose_steering_cmd: 0.0,
            arms,
            tailstrike_arm_m: geometry::tailstrike_point_m(),
        }
    }

    #[test]
    fn all_legs_touching_supports_roughly_the_static_weight_share() {
        let mut gear = LandingGear::new();
        let out = gear.step(&resting_inputs(), &GearFaults::default(), 0.05);
        assert!(out.any_wheel_on_ground);
        let total: f64 = out.leg_load_n.iter().sum();
        // Each leg's penetration (0.3 m plus its own small geometric
        // offset) times its own stiffness gives the total; this is a fixed
        // penetration probe, not an equilibrium solve, so it need not equal
        // MTOW, but it must be positive, finite, and the right order of
        // magnitude for a widebody (hundreds of kN to a few MN at 0.3 m+
        // compression).
        assert!(total.is_finite() && total > 1.0e5 && total < 5.0e7, "total gear load {total} N");
    }

    #[test]
    fn no_penetration_means_no_load_and_no_nan() {
        let mut gear = LandingGear::new();
        let mut inputs = resting_inputs();
        inputs.ground_z_world_m -= 10.0; // ground far below: airborne.
        let out = gear.step(&inputs, &GearFaults::default(), 0.05);
        assert!(!out.any_wheel_on_ground);
        assert_eq!(out.leg_load_n, [0.0; 5]);
        assert!(out.force_body_n.x.is_finite() && out.force_body_n.z.is_finite());
    }

    #[test]
    fn a_fully_collapsed_leg_carries_no_load() {
        let mut gear = LandingGear::new();
        let inputs = resting_inputs();
        let mut faults = GearFaults::default();
        faults.collapse_fraction[1] = 1.0; // wing-left leg fully collapsed.
        let out = gear.step(&inputs, &faults, 0.05);
        assert_eq!(out.leg_load_n[1], 0.0);
        assert!(out.leg_load_n[2] > 0.0, "the other legs should still carry load");
    }

    #[test]
    fn braking_produces_a_rearward_force_that_grows_then_the_friction_circle_caps_it() {
        let mut gear = LandingGear::new();
        let mut inputs = resting_inputs();
        inputs.velocity_body_m_s = Vec3::new(50.0, 0.0, 0.0); // rolling forward at 50 m/s.
        let mut light_brake = inputs;
        light_brake.brake_cmd = 0.1;
        let mut heavy_brake = inputs;
        heavy_brake.brake_cmd = 1.0;
        inputs.brake_cmd = 0.0;
        let out_free = gear.step(&inputs, &GearFaults::default(), 0.05);
        let out_light = gear.step(&light_brake, &GearFaults::default(), 0.05);
        let out_heavy = gear.step(&heavy_brake, &GearFaults::default(), 0.05);
        assert!(out_free.force_body_n.x < 0.0, "even unbraked rolling resistance opposes motion");
        assert!(out_light.force_body_n.x < out_free.force_body_n.x, "braking should add more rearward force");
        assert!(out_heavy.force_body_n.x.is_finite());
        // Peak Pacejka-lite friction is near the low end of the slip range,
        // so full pedal (slip=1, past the peak) should not exceed the
        // friction-circle cap magnitude.
        let cap = MU_PEAK_DRY * out_heavy.leg_load_n.iter().sum::<f64>();
        assert!(out_heavy.force_body_n.x.abs() <= cap + 1.0);
    }

    #[test]
    fn contaminated_runway_reduces_the_available_braking_force() {
        let mut dry = LandingGear::new();
        let mut icy = LandingGear::new();
        let mut inputs = resting_inputs();
        inputs.velocity_body_m_s = Vec3::new(50.0, 0.0, 0.0);
        inputs.brake_cmd = 1.0;
        let dry_out = dry.step(&inputs, &GearFaults::default(), 0.05);
        inputs.runway_mu_factor = 0.1; // icy
        let icy_out = icy.step(&inputs, &GearFaults::default(), 0.05);
        assert!(icy_out.force_body_n.x.abs() < dry_out.force_body_n.x.abs());
    }

    #[test]
    fn nose_steering_produces_a_yawing_side_force_while_taxiing() {
        let mut gear = LandingGear::new();
        let mut inputs = resting_inputs();
        inputs.velocity_body_m_s = Vec3::new(10.0, 0.0, 0.0);
        inputs.nose_steering_cmd = 1.0;
        for _ in 0..200 {
            gear.step(&inputs, &GearFaults::default(), 0.05);
        }
        let out = gear.step(&inputs, &GearFaults::default(), 0.05);
        assert!(out.moment_about_cg_n_m.z.is_finite());
        assert!(out.moment_about_cg_n_m.z.abs() > 1.0, "steering should generate some yaw moment while rolling");
    }

    #[test]
    fn a_deep_tailstrike_penetration_produces_a_large_finite_reaction() {
        let mut gear = LandingGear::new();
        let mut inputs = resting_inputs();
        // Push the tail well below the ground plane (a hard derotation).
        inputs.ground_z_world_m = geometry::tailstrike_point_m().z + 0.1;
        let out = gear.step(&inputs, &GearFaults::default(), 0.05);
        assert!(out.tailstrike);
        assert!(out.tailstrike_load_n.is_finite() && out.tailstrike_load_n > 0.0);
    }

    #[test]
    fn zero_dt_and_rest_produce_no_nan() {
        let mut gear = LandingGear::new();
        let out = gear.step(&resting_inputs(), &GearFaults::default(), 0.0);
        assert!(out.force_body_n.x.is_finite() && out.moment_about_cg_n_m.x.is_finite());
    }
}
