//! Spoiler-specific behaviour that sits on top of the generic
//! `surface::ControlSurface`/`hinge_moment.rs` model every spoiler panel
//! already uses: the aerodynamic deflection limit a roll spoiler "blows
//! down" to at high dynamic pressure, a representative gust/manoeuvre load
//! alleviation schedule, and the ground-spoiler auto-deploy/retract logic
//! (armed by the speedbrake lever, triggered by touchdown, cleared by a
//! go-around) with its own two logic failures.
//!
//! Real A380 blowdown/load-alleviation numbers and ground-spoiler arming
//! logic are not public; `a380_systems/src/hydraulic/mod.rs`'s own ground
//! spoiler detection is a placeholder too (`SpoilerGroup::ground_spoilers_are_requested`,
//! mod.rs:6992-7002, checks only "both spoiler 1/2 commands above 0.55",
//! its own comment reading "TODO use actual signal from flight controls" --
//! no weight-on-wheels, lever or reverser interlock modelled there at all).
//! Everything numeric below is GENERIC; the logic shape (arm -> touchdown or
//! spin-up -> deploy -> go-around or lever stow -> retract) is the publicly
//! documented general behaviour of a transport's ground spoiler system.

use super::hinge_moment::{compressibility, HingeMomentCoefficients};
use super::surface::DEFAULT_MACH_CRIT;

/// Blowdown: the largest deflection this surface's actuator(s) can actually
/// hold against the aerodynamic hinge moment at the given flight condition,
/// solving `|hinge_moment(angle)| = max_torque_nm` for `angle` in the linear
/// Ch region. Above this dynamic pressure/deflection combination, a
/// `surface::ControlSurface` commanded past it will emergently blow back to
/// it (`SurfaceOutput::blown_back`) rather than needing a separate rule
/// here -- this function is the number a control law would consult *before*
/// commanding a deflection, to avoid ever asking for more than the
/// actuator(s) can deliver.
pub fn max_sustainable_deflection_rad(
    hinge: &HingeMomentCoefficients,
    max_torque_nm: f64,
    alpha_rad: f64,
    dynamic_pressure_pa: f64,
    mach: f64,
    mach_crit: f64,
) -> f64 {
    let k = dynamic_pressure_pa.max(0.0) * hinge.area_m2 * hinge.chord_m * compressibility(mach, mach_crit);
    if k <= 1e-9 || hinge.ch_delta_per_rad == 0.0 {
        // No aerodynamic load (on the ground, or a hinge with no restoring
        // slope): the actuator's own mechanical travel is the only limit,
        // which this function does not know, so it reports "unlimited".
        return f64::INFINITY;
    }
    // `hinge_moment_nm` itself saturates the coefficient at `ch_max`: the
    // largest aerodynamic moment possible at this q is `k * ch_max`
    // regardless of deflection. If the actuator's torque ceiling is at or
    // above that, no amount of deflection can ever overpower it, so there
    // is no blowdown limit at this flight condition at all.
    if max_torque_nm.abs() >= k * hinge.ch_max {
        return f64::INFINITY;
    }
    let bias = hinge.ch_alpha_per_rad * alpha_rad;
    (((max_torque_nm.abs() / k) - bias) / -hinge.ch_delta_per_rad).max(0.0)
}

/// A GENERIC representative gust/manoeuvre load alleviation schedule for a
/// roll spoiler: full authority near 1g at low dynamic pressure, tapering
/// off as either dynamic pressure or the deviation from 1g grows, to keep
/// combined manoeuvre+gust wing bending moment below a structural design
/// margin -- the same purpose real transport load-alleviation functions
/// serve (publicly described for various types, e.g. Airbus's MLA/GLA
/// papers, though not with A380-specific numbers). Returns a fraction 0..1
/// of the surface's full mechanical travel.
pub fn load_alleviation_authority(dynamic_pressure_pa: f64, load_factor_g: f64) -> f64 {
    let q = dynamic_pressure_pa.max(0.0);
    // GENERIC: authority starts tapering above 15 kPa (a representative
    // high-speed cruise/high-IAS dynamic pressure) and reaches half
    // authority by 30 kPa.
    let q_factor = (1.0 - 0.5 * ((q - 15_000.0) / 15_000.0).clamp(0.0, 1.0)).clamp(0.5, 1.0);
    // GENERIC: authority starts tapering once |n_z - 1| exceeds 0.5 g, down
    // to half authority by 1.5 g away from 1g (i.e. n_z <= -0.5 or >= 2.5).
    let g_dev = (load_factor_g - 1.0).abs();
    let g_factor = (1.0 - 0.5 * ((g_dev - 0.5) / 1.0).clamp(0.0, 1.0)).clamp(0.5, 1.0);
    q_factor * g_factor
}

/// Faults the ground-spoiler auto-deploy/retract logic itself can carry
/// (the arming/sequencing electronics, not the spoiler actuators
/// themselves, which already have their own `actuator::ActuatorFaults` via
/// `surface::ControlSurface`).
#[derive(Clone, Copy, Debug, Default)]
pub struct GroundSpoilerLogicFaults {
    /// 0 healthy .. 1 the logic never commands deployment even though its
    /// own arm/touchdown conditions are met -- a serious hazard (loss of
    /// lift dump and reduced wheel braking effectiveness on landing).
    pub fails_to_deploy: f64,
    /// 0 healthy .. 1 the logic never commands retraction on a go-around --
    /// also serious (reduced lift and increased drag exactly when climb
    /// performance matters most).
    pub fails_to_retract: f64,
}

/// This tick's inputs the ground-spoiler logic needs.
#[derive(Clone, Copy, Debug, Default)]
pub struct GroundSpoilerInputs {
    /// The speedbrake/ground-spoiler lever is armed for automatic
    /// deployment.
    pub lever_armed: bool,
    /// The manual speedbrake lever's own commanded ratio (0..1); the
    /// ground-spoiler system deploys fully regardless of this once
    /// triggered, but a lever moved to a flight-spoiler position while
    /// armed is not itself a touchdown trigger.
    pub main_gear_wow: [bool; 2],
    pub wheel_speed_kt: f64,
    pub radio_alt_ft: f64,
    /// True the moment a go-around is selected (thrust levers advanced out
    /// of idle/reverse); this module does not model the throttle/FADEC
    /// logic that would set it (see `throttle.rs`/`fadec.rs` elsewhere in
    /// the crate), it only consumes the discrete.
    pub go_around_selected: bool,
}

/// The speed above which a rejected take-off extends the ground spoilers,
/// and the "rolling on the runway" half of this model's auto-deploy OR
/// condition. **Sourced, and A380-specific**: the FCOM's rejected-take-off
/// logic extends the ground spoilers fully "when the thrust levers are at
/// idle and the speed is greater than 72 kt" (DSC-27-10-40 p.1848), with the
/// same 72 kt gating the reverser-selected case. This was carried as a
/// GENERIC order-of-magnitude guess until the FCOM was read, and the guess
/// turned out to be exactly the aircraft's own figure.
const WHEEL_SPINUP_KT: f64 = 72.0;
/// Low-height gate paired with the wheel-spin-up condition, so a fast taxi
/// run can never look like a touchdown. **Sourced**: the FCOM's own
/// auto-arm height -- "the ground spoilers are automatically armed if the
/// speed brakes are extended and the radio height is lower than 6 ft"
/// (DSC-27-10-40 p.1847).
const SPINUP_RADIO_ALT_FT: f64 = 6.0;

/// The ground-spoiler deploy/retract state machine: armed -> touchdown (WOW
/// on both main gear, or wheel spin-up at low height) -> deployed -> a
/// go-around or disarming the lever -> retracted. Deployment latches (it
/// does not chatter off if weight momentarily comes off one strut on a
/// bouncy touchdown) until an explicit retract condition.
#[derive(Clone, Copy, Debug, Default)]
pub struct GroundSpoilerLogic {
    deployed: bool,
}

/// What the logic commands this tick.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct GroundSpoilerOutput {
    /// 0 or 1: the ground-spoiler command every ground-spoiler-capable
    /// panel should be driven to, added on top of (or overriding, per the
    /// surface's own command mixing, not modelled here) any flight-spoiler
    /// command.
    pub deploy_command: f64,
    pub deployed: bool,
}

impl GroundSpoilerLogic {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn step(&mut self, inputs: &GroundSpoilerInputs, faults: &GroundSpoilerLogicFaults) -> GroundSpoilerOutput {
        let touchdown = inputs.main_gear_wow[0] && inputs.main_gear_wow[1];
        let spun_up_on_ground = inputs.wheel_speed_kt > WHEEL_SPINUP_KT && inputs.radio_alt_ft < SPINUP_RADIO_ALT_FT;

        if !self.deployed && inputs.lever_armed && (touchdown || spun_up_on_ground) {
            if faults.fails_to_deploy.clamp(0.0, 1.0) < 0.5 {
                self.deployed = true;
            }
        }

        if self.deployed && (inputs.go_around_selected || !inputs.lever_armed) {
            if faults.fails_to_retract.clamp(0.0, 1.0) < 0.5 {
                self.deployed = false;
            }
        }

        GroundSpoilerOutput { deploy_command: if self.deployed { 1.0 } else { 0.0 }, deployed: self.deployed }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use super::super::hinge_moment::HingeMomentCoefficients;

    #[test]
    fn no_nan_and_no_load_gives_no_blowdown_limit() {
        let hinge = HingeMomentCoefficients::spoiler();
        let limit = max_sustainable_deflection_rad(&hinge, 1000.0, 0.0, 0.0, 0.0, 0.75);
        assert!(limit.is_infinite());
        let mut logic = GroundSpoilerLogic::new();
        let out = logic.step(&GroundSpoilerInputs::default(), &GroundSpoilerLogicFaults::default());
        assert!(out.deploy_command.is_finite());
    }

    #[test]
    fn blowdown_limit_shrinks_as_dynamic_pressure_grows() {
        let hinge = HingeMomentCoefficients::spoiler();
        let low_q = max_sustainable_deflection_rad(&hinge, 5000.0, 0.0, 5000.0, 0.3, 0.75);
        let high_q = max_sustainable_deflection_rad(&hinge, 5000.0, 0.0, 40000.0, 0.3, 0.75);
        assert!(high_q < low_q, "high-q limit {high_q} should be tighter than low-q {low_q}");
        assert!(high_q > 0.0);
    }

    #[test]
    fn the_blowdown_formula_agrees_with_the_surfaces_own_equilibrium() {
        // Cross-check against `surface::ControlSurface`'s emergent
        // behaviour: commanding well past the formula's predicted limit
        // should settle almost exactly at it.
        use super::super::actuator::{ActuatorFaults, ActuatorGeometry, ActuatorMode, PowerControlUnit};
        use super::super::surface::{AeroInputs, ControlSurface, SurfaceDamping, SurfaceFaults, SurfaceLimits};

        let hinge = HingeMomentCoefficients::spoiler();
        // The real cited spoiler PCU (`ActuatorGeometry::spoiler`, ~44,000
        // N*m) comfortably out-powers this crate's GENERIC spoiler
        // hinge-moment sizing at any ordinary dynamic pressure -- it can
        // hold full travel right up to where `Ch` itself saturates, which
        // `blowdown_limit_shrinks_as_dynamic_pressure_grows` already
        // exercises. To cross-check the formula's *linear* branch against
        // `ControlSurface`'s emergent equilibrium at an ordinary, stable
        // dynamic pressure (rather than the extreme one it would take to
        // out-power the real actuator), this test uses a deliberately
        // smaller GENERIC test actuator with the same crank arm.
        let geometry = ActuatorGeometry::new(0.017, 0.0, 0.02, 0.1908);
        let aero = AeroInputs { dynamic_pressure_pa: 10_000.0, alpha_rad: 0.0, mach: 0.5, mach_crit: DEFAULT_MACH_CRIT };
        let max_torque = geometry.max_torque_nm(super::super::actuator::HYDRAULIC_SUPPLY_PA);
        let predicted = max_sustainable_deflection_rad(&hinge, max_torque, aero.alpha_rad, aero.dynamic_pressure_pa, aero.mach, aero.mach_crit);
        assert!(predicted.is_finite() && predicted < 50.0_f64.to_radians(), "test setup should land in the linear regime, got {predicted}");

        let mut surface = ControlSurface::new(
            [PowerControlUnit::new(geometry)],
            hinge,
            super::super::surface::inertia_uniform_plate_kg_m2(42.0, 0.685),
            SurfaceLimits { min_rad: -10.0_f64.to_radians(), max_rad: 50.0_f64.to_radians() },
            SurfaceDamping::spoiler(),
            0.0,
        );
        let mut angle = 0.0;
        for _ in 0..60_000 {
            let out = surface.step([ActuatorMode::Active], 50.0_f64.to_radians(), [1.0], [ActuatorFaults::default()], &SurfaceFaults::default(), &aero, 0.01);
            angle = out.angle_rad;
        }
        assert!((angle - predicted).abs() < 0.03, "settled at {angle} rad, predicted {predicted} rad");
    }

    #[test]
    fn load_alleviation_reduces_authority_at_high_speed_or_off_1g() {
        let cruise = load_alleviation_authority(3000.0, 1.0);
        let fast = load_alleviation_authority(35000.0, 1.0);
        let maneuvering = load_alleviation_authority(3000.0, 2.2);
        assert_eq!(cruise, 1.0);
        assert!(fast < 1.0);
        assert!(maneuvering < 1.0);
        assert!(fast >= 0.5 && maneuvering >= 0.5, "should taper, never remove all authority");
    }

    #[test]
    fn ground_spoilers_auto_deploy_on_touchdown_when_armed() {
        let mut logic = GroundSpoilerLogic::new();
        let inputs = GroundSpoilerInputs { lever_armed: true, main_gear_wow: [true, true], ..Default::default() };
        let out = logic.step(&inputs, &GroundSpoilerLogicFaults::default());
        assert_eq!(out.deploy_command, 1.0);
        assert!(out.deployed);
    }

    #[test]
    fn ground_spoilers_do_not_deploy_disarmed_even_at_touchdown() {
        let mut logic = GroundSpoilerLogic::new();
        let inputs = GroundSpoilerInputs { lever_armed: false, main_gear_wow: [true, true], ..Default::default() };
        let out = logic.step(&inputs, &GroundSpoilerLogicFaults::default());
        assert_eq!(out.deploy_command, 0.0);
    }

    #[test]
    fn wheel_spin_up_at_low_height_also_triggers_deployment() {
        let mut logic = GroundSpoilerLogic::new();
        let inputs = GroundSpoilerInputs {
            lever_armed: true,
            main_gear_wow: [false, false],
            wheel_speed_kt: 90.0,
            radio_alt_ft: 1.0,
            ..Default::default()
        };
        let out = logic.step(&inputs, &GroundSpoilerLogicFaults::default());
        assert!(out.deployed);
    }

    #[test]
    fn a_go_around_retracts_deployed_spoilers() {
        let mut logic = GroundSpoilerLogic::new();
        let touchdown = GroundSpoilerInputs { lever_armed: true, main_gear_wow: [true, true], ..Default::default() };
        assert!(logic.step(&touchdown, &GroundSpoilerLogicFaults::default()).deployed);
        let go_around = GroundSpoilerInputs { lever_armed: true, main_gear_wow: [true, true], go_around_selected: true, ..Default::default() };
        let out = logic.step(&go_around, &GroundSpoilerLogicFaults::default());
        assert!(!out.deployed);
    }

    #[test]
    fn a_deployment_failure_leaves_the_spoilers_stowed_despite_touchdown() {
        let mut logic = GroundSpoilerLogic::new();
        let inputs = GroundSpoilerInputs { lever_armed: true, main_gear_wow: [true, true], ..Default::default() };
        let out = logic.step(&inputs, &GroundSpoilerLogicFaults { fails_to_deploy: 1.0, ..Default::default() });
        assert!(!out.deployed);
    }

    #[test]
    fn a_retraction_failure_leaves_the_spoilers_stuck_out_through_a_go_around() {
        let mut logic = GroundSpoilerLogic::new();
        let touchdown = GroundSpoilerInputs { lever_armed: true, main_gear_wow: [true, true], ..Default::default() };
        assert!(logic.step(&touchdown, &GroundSpoilerLogicFaults::default()).deployed);
        let go_around = GroundSpoilerInputs { lever_armed: true, main_gear_wow: [true, true], go_around_selected: true, ..Default::default() };
        let faults = GroundSpoilerLogicFaults { fails_to_retract: 1.0, ..Default::default() };
        let out = logic.step(&go_around, &faults);
        assert!(out.deployed, "should be stuck deployed");
    }
}
