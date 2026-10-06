use super::hinge_moment::{compressibility, HingeMomentCoefficients};
use super::surface::DEFAULT_MACH_CRIT;

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
        return f64::INFINITY;
    }
    if max_torque_nm.abs() >= k * hinge.ch_max {
        return f64::INFINITY;
    }
    let bias = hinge.ch_alpha_per_rad * alpha_rad;
    (((max_torque_nm.abs() / k) - bias) / -hinge.ch_delta_per_rad).max(0.0)
}

pub fn load_alleviation_authority(dynamic_pressure_pa: f64, load_factor_g: f64) -> f64 {
    let q = dynamic_pressure_pa.max(0.0);
    let q_factor = (1.0 - 0.5 * ((q - 15_000.0) / 15_000.0).clamp(0.0, 1.0)).clamp(0.5, 1.0);
    let g_dev = (load_factor_g - 1.0).abs();
    let g_factor = (1.0 - 0.5 * ((g_dev - 0.5) / 1.0).clamp(0.0, 1.0)).clamp(0.5, 1.0);
    q_factor * g_factor
}

#[derive(Clone, Copy, Debug, Default)]
pub struct GroundSpoilerLogicFaults {
    pub fails_to_deploy: f64,
    pub fails_to_retract: f64,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct GroundSpoilerInputs {
    pub lever_armed: bool,
    pub main_gear_wow: [bool; 2],
    pub wheel_speed_kt: f64,
    pub radio_alt_ft: f64,
    pub go_around_selected: bool,
}

const WHEEL_SPINUP_KT: f64 = 72.0;
const SPINUP_RADIO_ALT_FT: f64 = 6.0;

#[derive(Clone, Copy, Debug, Default)]
pub struct GroundSpoilerLogic {
    deployed: bool,
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct GroundSpoilerOutput {
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
        use super::super::actuator::{ActuatorFaults, ActuatorGeometry, ActuatorMode, PowerControlUnit};
        use super::super::surface::{AeroInputs, ControlSurface, SurfaceDamping, SurfaceFaults, SurfaceLimits};

        let hinge = HingeMomentCoefficients::spoiler();
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
