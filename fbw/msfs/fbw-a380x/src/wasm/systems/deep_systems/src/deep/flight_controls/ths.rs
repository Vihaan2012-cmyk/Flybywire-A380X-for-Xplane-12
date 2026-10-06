use super::actuator::{
    servo_rate_step, ActuatorFaults, ActuatorMode, ElectricMotorPump, ElectricPumpFaults, PowerControlUnit, ServoLoad,
};
use super::high_lift::synthetic_rotary_geometry;
use super::hinge_moment::{hinge_moment_nm, HingeMomentCoefficients};
use super::surface::{inertia_uniform_plate_kg_m2, AeroInputs, SurfaceLimits, DEFAULT_MACH_CRIT};

#[derive(Clone, Copy, Debug, Default)]
pub struct ThsFaults {
    pub motor_green: ActuatorFaults,
    pub motor_yellow: ActuatorFaults,
    pub no_back_failure: f64,
    pub ballscrew_jam: f64,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct ThsOutput {
    pub angle_rad: f64,
    pub rate_rad_s: f64,
    pub motor_torque_nm: f64,
    pub hinge_moment_nm: f64,
    pub no_back_engaged: bool,
    pub at_stop: bool,
}

pub const THS_MIN_DEG: f64 = -2.0;
pub const THS_MAX_DEG: f64 = 10.0;

pub struct TrimmableHorizontalStabilizer {
    motor_green: PowerControlUnit,
    motor_yellow: PowerControlUnit,
    hinge: HingeMomentCoefficients,
    inertia_kg_m2: f64,
    limits: SurfaceLimits,
    no_back_holding_torque_nm: f64,
    no_back_active_threshold_nm: f64,
    jam_spring_nm_per_rad: f64,
    jam_damp_nm_s_per_rad: f64,
    structural_damping_nm_s_per_rad_s: f64,
    angle_rad: f64,
    rate_rad_s: f64,
    no_back_lock_angle: Option<f64>,
    jam_angle_rad: Option<f64>,
}

impl TrimmableHorizontalStabilizer {
    pub fn new_generic() -> Self {
        let motor = || PowerControlUnit::new(synthetic_rotary_geometry(80_000.0, 0.015));
        let hinge = HingeMomentCoefficients { ch_delta_per_rad: -0.30, ch_alpha_per_rad: -0.15, ch_max: 0.30, area_m2: 30.0, chord_m: 3.0 };
        let max_t = 160_000.0;
        Self {
            motor_green: motor(),
            motor_yellow: motor(),
            hinge,
            inertia_kg_m2: inertia_uniform_plate_kg_m2(700.0, 3.0),
            limits: SurfaceLimits { min_rad: THS_MIN_DEG.to_radians(), max_rad: THS_MAX_DEG.to_radians() },
            no_back_holding_torque_nm: 200_000.0,
            no_back_active_threshold_nm: 500.0,
            jam_spring_nm_per_rad: max_t * 50.0,
            jam_damp_nm_s_per_rad: max_t * 5.0,
            structural_damping_nm_s_per_rad_s: 4000.0,
            angle_rad: 0.0,
            rate_rad_s: 0.0,
            no_back_lock_angle: None,
            jam_angle_rad: None,
        }
    }

    pub fn angle_rad(&self) -> f64 {
        self.angle_rad
    }
    pub fn angle_deg(&self) -> f64 {
        self.angle_rad.to_degrees()
    }

    const MAX_SUBSTEP_S: f64 = 0.0005;

    pub fn step(
        &mut self,
        modes: [ActuatorMode; 2],
        commanded_angle_rad: f64,
        pressure_fraction: [f64; 2],
        faults: &ThsFaults,
        aero: &AeroInputs,
        dt_s: f64,
    ) -> ThsOutput {
        let dt_total = dt_s.max(0.0);
        let n = ((dt_total / Self::MAX_SUBSTEP_S).ceil() as usize).max(1);
        let sub_dt = dt_total / n as f64;
        let mut out = ThsOutput::default();
        for _ in 0..n {
            out = self.step_once(modes, commanded_angle_rad, pressure_fraction, faults, aero, sub_dt);
        }
        out
    }

    fn step_once(
        &mut self,
        modes: [ActuatorMode; 2],
        commanded_angle_rad: f64,
        pressure_fraction: [f64; 2],
        faults: &ThsFaults,
        aero: &AeroInputs,
        dt: f64,
    ) -> ThsOutput {
        let supply = [
            pressure_fraction[0].clamp(0.0, 1.0) * (1.0 - faults.motor_green.supply_loss.clamp(0.0, 1.0)),
            pressure_fraction[1].clamp(0.0, 1.0) * (1.0 - faults.motor_yellow.supply_loss.clamp(0.0, 1.0)),
        ];
        let driving_motors = supply.iter().filter(|s| **s > 1e-3).count();
        let gear = if driving_motors == 0 { 1.0 } else { 2.0 / driving_motors as f64 };

        let g = self.motor_green.step(
            modes[0],
            commanded_angle_rad * gear,
            self.angle_rad * gear,
            self.rate_rad_s * gear,
            pressure_fraction[0],
            &faults.motor_green,
        );
        let y = self.motor_yellow.step(
            modes[1],
            commanded_angle_rad * gear,
            self.angle_rad * gear,
            self.rate_rad_s * gear,
            pressure_fraction[1],
            &faults.motor_yellow,
        );
        let mut servo = g.servo.geared(gear);
        servo.add(&y.servo.geared(gear));
        let motor_torque = gear * (g.torque_nm + y.torque_nm);
        let motor_jam_torque = gear * (g.jam_torque_nm + y.jam_torque_nm);
        let motor_jam_damping = gear * gear * (g.jam_damping_nm_s_per_rad + y.jam_damping_nm_s_per_rad);

        let motor_capacity_nm = gear * (g.max_torque_nm + y.max_torque_nm);
        let driving = (modes[0] == ActuatorMode::Active || modes[1] == ActuatorMode::Active)
            && motor_capacity_nm > self.no_back_active_threshold_nm;
        if driving {
            self.no_back_lock_angle = None;
        } else if self.no_back_lock_angle.is_none() {
            self.no_back_lock_angle = Some(self.angle_rad);
        }
        let no_back_capacity = self.no_back_holding_torque_nm * (1.0 - faults.no_back_failure.clamp(0.0, 1.0));
        let (no_back_torque, no_back_damping) = match self.no_back_lock_angle {
            Some(lock) => {
                let k = no_back_capacity * 50.0;
                let c = no_back_capacity * 5.0;
                let raw = -k * (self.angle_rad - lock) - c * self.rate_rad_s;
                let held = raw.clamp(-no_back_capacity, no_back_capacity);
                (held, if raw == held { c } else { 0.0 })
            }
            None => (0.0, 0.0),
        };

        let jam = faults.ballscrew_jam.clamp(0.0, 1.0);
        if jam > 0.0 {
            if self.jam_angle_rad.is_none() {
                self.jam_angle_rad = Some(self.angle_rad);
            }
        } else {
            self.jam_angle_rad = None;
        }
        let jam_torque = match self.jam_angle_rad {
            Some(seize) => -self.jam_spring_nm_per_rad * jam * (self.angle_rad - seize) - self.jam_damp_nm_s_per_rad * jam * self.rate_rad_s,
            None => 0.0,
        };

        let hinge_m = hinge_moment_nm(&self.hinge, self.angle_rad, aero.alpha_rad, aero.dynamic_pressure_pa, aero.mach, aero.mach_crit);

        let other_torque = motor_jam_torque + no_back_torque + jam_torque + hinge_m
            - self.structural_damping_nm_s_per_rad_s * self.rate_rad_s;
        let other_damping = motor_jam_damping
            + no_back_damping
            + self.jam_damp_nm_s_per_rad * jam
            + self.structural_damping_nm_s_per_rad_s;
        self.rate_rad_s =
            servo_rate_step(self.rate_rad_s, &servo, other_torque, other_damping, self.inertia_kg_m2, dt);
        self.angle_rad += self.rate_rad_s * dt;

        let mut at_stop = false;
        if self.angle_rad <= self.limits.min_rad {
            self.angle_rad = self.limits.min_rad;
            if self.rate_rad_s < 0.0 {
                self.rate_rad_s = 0.0;
            }
            at_stop = true;
        } else if self.angle_rad >= self.limits.max_rad {
            self.angle_rad = self.limits.max_rad;
            if self.rate_rad_s > 0.0 {
                self.rate_rad_s = 0.0;
            }
            at_stop = true;
        }

        ThsOutput {
            angle_rad: self.angle_rad,
            rate_rad_s: self.rate_rad_s,
            motor_torque_nm: motor_torque,
            hinge_moment_nm: hinge_m,
            no_back_engaged: !driving,
            at_stop,
        }
    }
}

pub struct RudderTrimActuator {
    pcu: PowerControlUnit,
    pump: ElectricMotorPump,
    angle_rad: f64,
    rate_rad_s: f64,
    inertia_kg_m2: f64,
    limit_rad: f64,
}

impl RudderTrimActuator {
    pub fn new_generic() -> Self {
        Self {
            pcu: PowerControlUnit::new(synthetic_rotary_geometry(200.0, 0.05)),
            pump: ElectricMotorPump::new(0.3),
            angle_rad: 0.0,
            rate_rad_s: 0.0,
            inertia_kg_m2: 0.5,
            limit_rad: 20.0_f64.to_radians(),
        }
    }

    pub fn angle_rad(&self) -> f64 {
        self.angle_rad
    }

    pub fn step(
        &mut self,
        commanded_angle_rad: f64,
        electrical_power_fraction: f64,
        pump_faults: &ElectricPumpFaults,
        actuator_faults: &ActuatorFaults,
        dt_s: f64,
    ) -> f64 {
        let dt = dt_s.max(0.0);
        let pressure = self.pump.step(electrical_power_fraction, pump_faults, dt);
        let out = self.pcu.step(
            ActuatorMode::Active,
            commanded_angle_rad.clamp(-self.limit_rad, self.limit_rad),
            self.angle_rad,
            self.rate_rad_s,
            pressure,
            actuator_faults,
        );
        const MECHANISM_FRICTION_NM_S_PER_RAD: f64 = 5.0;
        let other_torque = out.jam_torque_nm - MECHANISM_FRICTION_NM_S_PER_RAD * self.rate_rad_s;
        let other_damping = out.jam_damping_nm_s_per_rad + MECHANISM_FRICTION_NM_S_PER_RAD;
        self.rate_rad_s =
            servo_rate_step(self.rate_rad_s, &out.servo, other_torque, other_damping, self.inertia_kg_m2, dt);
        self.angle_rad = (self.angle_rad + self.rate_rad_s * dt).clamp(-self.limit_rad, self.limit_rad);
        self.angle_rad
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const DT: f64 = 0.01;
    const ACTIVE: [ActuatorMode; 2] = [ActuatorMode::Active, ActuatorMode::Active];
    const IDLE: [ActuatorMode; 2] = [ActuatorMode::Damping, ActuatorMode::Damping];

    fn calm_aero() -> AeroInputs {
        AeroInputs::default()
    }

    #[test]
    fn no_nan_at_rest_or_zero_dt() {
        let mut ths = TrimmableHorizontalStabilizer::new_generic();
        let out = ths.step(ACTIVE, 0.0, [0.0, 0.0], &ThsFaults::default(), &calm_aero(), 0.0);
        assert!(out.angle_rad.is_finite());
        let mut trim = RudderTrimActuator::new_generic();
        let a = trim.step(0.0, 0.0, &ElectricPumpFaults::default(), &ActuatorFaults::default(), 0.0);
        assert!(a.is_finite());
    }

    #[test]
    fn both_motors_healthy_trim_to_the_commanded_angle() {
        let mut ths = TrimmableHorizontalStabilizer::new_generic();
        let target = 5.0_f64.to_radians();
        let mut out = ThsOutput::default();
        for _ in 0..40_000 {
            out = ths.step(ACTIVE, target, [1.0, 1.0], &ThsFaults::default(), &calm_aero(), DT);
        }
        assert!((out.angle_rad - target).abs() < 0.01, "settled at {} deg", out.angle_rad.to_degrees());
    }

    #[test]
    fn one_dead_motor_still_trims_just_slower() {
        let mut both = TrimmableHorizontalStabilizer::new_generic();
        let mut one = TrimmableHorizontalStabilizer::new_generic();
        let target = 5.0_f64.to_radians();
        let dead = ThsFaults { motor_yellow: ActuatorFaults { supply_loss: 1.0, ..Default::default() }, ..Default::default() };
        let mut both_angle = 0.0;
        let mut one_angle = 0.0;
        for _ in 0..300 {
            both_angle = both.step(ACTIVE, target, [1.0, 1.0], &ThsFaults::default(), &calm_aero(), DT).angle_rad;
            one_angle = one.step(ACTIVE, target, [1.0, 1.0], &dead, &calm_aero(), DT).angle_rad;
        }
        assert!(one_angle > 0.0, "the surviving motor should still move it");
        assert!(one_angle < both_angle, "but slower than with both motors");
        assert!((both_angle - 0.045).abs() < 1e-3, "both motors: 0.015 rad/s for 3 s, got {both_angle}");
        assert!((one_angle - 0.0225).abs() < 1e-3, "one motor through the differential: half that, got {one_angle}");
    }

    #[test]
    fn the_no_back_brake_holds_position_against_aero_load_between_trim_inputs() {
        let mut ths = TrimmableHorizontalStabilizer::new_generic();
        let target = 4.0_f64.to_radians();
        for _ in 0..20_000 {
            ths.step(ACTIVE, target, [1.0, 1.0], &ThsFaults::default(), &calm_aero(), DT);
        }
        let held_angle = ths.angle_rad();
        let aero = AeroInputs { dynamic_pressure_pa: 15000.0, alpha_rad: 0.02, mach: 0.5, mach_crit: DEFAULT_MACH_CRIT };
        let mut out = ThsOutput::default();
        for _ in 0..5000 {
            out = ths.step(IDLE, held_angle, [1.0, 1.0], &ThsFaults::default(), &aero, DT);
        }
        assert!(out.no_back_engaged);
        assert!((out.angle_rad - held_angle).abs() < 0.01, "should hold against airload: drifted to {}", out.angle_rad.to_degrees());
    }

    #[test]
    fn a_failed_no_back_brake_lets_the_stabiliser_back_drive_under_load() {
        let mut healthy = TrimmableHorizontalStabilizer::new_generic();
        let mut failed = TrimmableHorizontalStabilizer::new_generic();
        let target = 4.0_f64.to_radians();
        for sys in [&mut healthy, &mut failed] {
            for _ in 0..20_000 {
                sys.step(ACTIVE, target, [1.0, 1.0], &ThsFaults::default(), &calm_aero(), DT);
            }
        }
        let held = healthy.angle_rad();
        let aero = AeroInputs { dynamic_pressure_pa: 25000.0, alpha_rad: 0.05, mach: 0.6, mach_crit: DEFAULT_MACH_CRIT };
        let no_back_failed = ThsFaults { no_back_failure: 1.0, ..Default::default() };
        let mut healthy_drift = 0.0_f64;
        let mut failed_drift = 0.0_f64;
        for _ in 0..5000 {
            let ho = healthy.step(IDLE, held, [1.0, 1.0], &ThsFaults::default(), &aero, DT);
            let fo = failed.step(IDLE, held, [1.0, 1.0], &no_back_failed, &aero, DT);
            healthy_drift = (ho.angle_rad - held).abs();
            failed_drift = (fo.angle_rad - held).abs();
        }
        assert!(failed_drift > healthy_drift * 2.0, "failed {failed_drift} vs healthy {healthy_drift}");
    }

    #[test]
    fn the_no_back_brake_engages_on_lost_torque_capacity_even_while_still_commanded_active() {
        let mut ths = TrimmableHorizontalStabilizer::new_generic();
        let target = 4.0_f64.to_radians();
        for _ in 0..20_000 {
            ths.step(ACTIVE, target, [1.0, 1.0], &ThsFaults::default(), &calm_aero(), DT);
        }
        let held_angle = ths.angle_rad();
        let aero = AeroInputs { dynamic_pressure_pa: 15000.0, alpha_rad: 0.02, mach: 0.5, mach_crit: DEFAULT_MACH_CRIT };
        let mut out = ThsOutput::default();
        for _ in 0..5000 {
            out = ths.step(ACTIVE, target, [0.0, 0.0], &ThsFaults::default(), &aero, DT);
        }
        assert!(out.no_back_engaged, "unpowered but still commanded Active: the no-back must still take hold");
        assert!(
            (out.angle_rad - held_angle).abs() < 0.01,
            "and actually hold against the airload with nothing else able to: drifted to {}",
            out.angle_rad.to_degrees()
        );
    }

    #[test]
    fn the_no_back_brake_stays_disengaged_while_genuinely_powered_and_active() {
        let mut ths = TrimmableHorizontalStabilizer::new_generic();
        let out = ths.step(ACTIVE, 4.0_f64.to_radians(), [1.0, 1.0], &ThsFaults::default(), &calm_aero(), DT);
        assert!(!out.no_back_engaged, "a healthy, powered, actively-commanded THS must not have its no-back engaged");
    }

    #[test]
    fn a_ballscrew_jam_freezes_the_stabiliser() {
        let mut ths = TrimmableHorizontalStabilizer::new_generic();
        for _ in 0..10_000 {
            ths.step(ACTIVE, 3.0_f64.to_radians(), [1.0, 1.0], &ThsFaults::default(), &calm_aero(), DT);
        }
        let jammed_at = ths.angle_rad();
        let faults = ThsFaults { ballscrew_jam: 1.0, ..Default::default() };
        for _ in 0..5000 {
            ths.step(ACTIVE, THS_MAX_DEG.to_radians(), [1.0, 1.0], &faults, &calm_aero(), DT);
        }
        assert!((ths.angle_rad() - jammed_at).abs() < 0.02, "jam should hold near {jammed_at}, got {}", ths.angle_rad());
    }

    #[test]
    fn rudder_trim_tracks_command_and_a_jam_prevents_it() {
        let mut trim = RudderTrimActuator::new_generic();
        let mut a = 0.0;
        for _ in 0..40_000 {
            a = trim.step(0.1, 1.0, &ElectricPumpFaults::default(), &ActuatorFaults::default(), DT);
        }
        assert!((a - 0.1).abs() < 0.01, "settled at {a}");

        let mut jammed = RudderTrimActuator::new_generic();
        let faults = ActuatorFaults { jam: 1.0, ..Default::default() };
        let mut b = 0.0;
        for _ in 0..40_000 {
            b = jammed.step(0.1, 1.0, &ElectricPumpFaults::default(), &faults, DT);
        }
        assert!(b.abs() < 0.01, "a jam from rest should prevent any trim motion, got {b}");
    }
}
