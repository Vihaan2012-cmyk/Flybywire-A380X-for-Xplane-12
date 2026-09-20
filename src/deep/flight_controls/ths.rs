//! The trimmable horizontal stabiliser (THS): a ballscrew jackscrew driven
//! by two hydraulic motors (green and yellow, matching FlyByWire's own
//! `TrimmableHorizontalStabilizerActuator`, which likewise carries
//! `hydraulic_motors: [HydraulicDriveMotor; 2]`,
//! fbw-common/src/wasm/systems/systems/src/hydraulic/
//! trimmable_horizontal_stabilizer.rs:684,715-720) through a "no-back"
//! device that makes the screw irreversible: it free-wheels while a motor
//! is actively driving it, and locks the screw against being back-driven by
//! the tailplane's own aerodynamic load the instant neither motor is. This
//! is what lets a jackscrew hold position between trim inputs without the
//! motors fighting the airload continuously, and its failure is one of the
//! most consequential a THS can have (the FAA's AD 2000-15-15 for the
//! MD-80 jackscrew/nut assembly, following the 2000 Alaska Airlines 261
//! accident, documents exactly this failure mode on a different aircraft's
//! THS).
//!
//! # The two motors: a speed-summing differential
//! The two motors do not drive the screw on a common shaft: they drive it
//! through a **speed-summing differential gearbox**, so the screw turns at
//! the *average* of the two motor speeds while each motor sees the same
//! torque. Losing one hydraulic system therefore halves the trim *rate*
//! while leaving the torque capability intact: the unpowered motor is
//! braked by its own valve block, its input to the differential is held at
//! zero, and the surviving motor reaches its own rated speed at half the
//! screw rate, its torque doubled on the way through the gear. Modelled
//! here by reflecting each motor through a gear ratio `2 / (number of
//! motors with supply)`.
//!
//! **What is sourced and what is inferred, precisely:**
//! - *Half rate on one system* is corroborated by FlyByWire's own model.
//!   `TrimmableHorizontalStabilizerActuator::update_speed`
//!   (fbw-common/src/wasm/systems/systems/src/hydraulic/
//!   trimmable_horizontal_stabilizer.rs:767-773) accumulates
//!   `sum_of_speeds += motor.speed() * motor_to_ths_gearing_ratio` over
//!   both `hydraulic_motors`, so with one motor's supply gone the screw
//!   runs at half speed. Their A380 instantiation
//!   (fbw-a380x/src/wasm/systems/a380_systems/src/hydraulic/mod.rs:2131-2138)
//!   uses two such motors at 5000 rpm / 5000 psi.
//! - *That the arrangement must be a differential* is a kinematic
//!   deduction, not a citation, but it is a forced one: a common
//!   torque-summing shaft cannot produce that behaviour at all. A motor
//!   braked by its own valve block on a shared shaft holds the shaft, and
//!   the screw would not move. Half rate with one motor stopped is only
//!   possible if the stopped motor's input is a *differential* input. (FBW
//!   get the same number by summing speeds arithmetically without modelling
//!   the mechanical constraint; the differential is what makes their result
//!   physical.)
//! - *That torque is retained* then follows from the differential's own
//!   kinematics (speed halved through the gear <=> torque doubled) rather
//!   than from a document. Searched for an A380 FCOM trim-rate figure for
//!   one hydraulic system versus two, which would pin this directly: not
//!   public.
//!
//! The absolute trim rate remains GENERIC (0.015 rad/s ~ 0.86 deg/s per
//! motor, see `new_generic`). FBW's own figures imply about 1.5 deg/s per
//! motor and 3.0 deg/s combined (5000 rpm x their 0.00005 gearing), but
//! they did not source their gearing or displacement -- the displacement
//! carries their own comment "This value is just copied from the A320s THS.
//! No idea about the real value" -- so it is not adopted here.
//!
//! Also here: the smaller electric rudder-trim actuator, which biases the
//! SEC's rudder travel about its own screw the same way but at a much
//! smaller scale and without an aerodynamic hinge moment big enough to
//! matter against its own drive.
//!
//! No A380 THS motor torque, screw lead or no-back holding torque is
//! public; magnitudes are GENERIC, chosen (with the same margin philosophy
//! as `high_lift.rs`) so a healthy THS holds against a large tailplane
//! hinge moment (computed the same way as every other surface in this
//! crate, `hinge_moment.rs`) with room to spare.

use super::actuator::{
    servo_rate_step, ActuatorFaults, ActuatorMode, ElectricMotorPump, ElectricPumpFaults, PowerControlUnit, ServoLoad,
};
use super::high_lift::synthetic_rotary_geometry;
use super::hinge_moment::{hinge_moment_nm, HingeMomentCoefficients};
use super::surface::{inertia_uniform_plate_kg_m2, AeroInputs, SurfaceLimits, DEFAULT_MACH_CRIT};

/// Faults for the THS drive.
#[derive(Clone, Copy, Debug, Default)]
pub struct ThsFaults {
    pub motor_green: ActuatorFaults,
    pub motor_yellow: ActuatorFaults,
    /// 0 healthy .. 1 no holding torque left: the screw can be back-driven
    /// by aerodynamic load whenever the motors aren't actively driving it.
    pub no_back_failure: f64,
    /// 0 free .. 1 the ballscrew/nut is seized.
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

/// THS travel, degrees, positive nose up: -2 (nose down) .. +10 (nose up),
/// as already cited in `flight_controls.rs`'s doc comment from
/// `trimmable_horizontal_stabilizer.rs:806-809` and
/// `a380_systems/src/hydraulic/mod.rs:2111-2114`.
pub const THS_MIN_DEG: f64 = -2.0;
pub const THS_MAX_DEG: f64 = 10.0;

pub struct TrimmableHorizontalStabilizer {
    motor_green: PowerControlUnit,
    motor_yellow: PowerControlUnit,
    hinge: HingeMomentCoefficients,
    inertia_kg_m2: f64,
    limits: SurfaceLimits,
    no_back_holding_torque_nm: f64,
    /// Combined motor torque magnitude below which the drive is considered
    /// "not actively trimming", so the no-back brake takes hold.
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
    /// GENERIC sizing: 80,000 N*m per motor (a screw's huge mechanical
    /// advantage reflected back to the stabiliser's own angle, not the raw
    /// hydraulic motor shaft torque -- the entire point of a jackscrew over
    /// a direct ram for a surface this loaded), 0.015 rad/s (~0.86 deg/s)
    /// each. Hinge-moment sizing GENERIC: 30 m^2 x 3 m mean chord, in the
    /// same coefficient family as every other surface in `hinge_moment.rs`.
    /// Mass GENERIC 700 kg (a large stabiliser box structure), the same
    /// uniform-plate inertia approximation used throughout this crate.
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

    /// See `surface::ControlSurface::MAX_SUBSTEP_S`: the no-back brake and
    /// jam springs here are just as stiff and need the same treatment.
    const MAX_SUBSTEP_S: f64 = 0.0005;

    /// `modes`: normally `Active` while the FCU/pedestal wheel is
    /// commanding a trim change, `Damping` (motors depowered/idling, only
    /// resisting their own motion) the rest of the time, matching the real
    /// operational pattern -- the motors are not run as a continuous
    /// position servo between commands, the no-back device is what holds
    /// position then.
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
        // Speed-summing differential (see module doc): a motor whose supply
        // is gone is braked, its differential input is held, and the
        // surviving motor's shaft must turn twice as fast per unit of screw
        // rotation -- so it reaches its own rated speed at half the screw
        // rate, while its torque is doubled on the way through the gear.
        let supply = [
            pressure_fraction[0].clamp(0.0, 1.0) * (1.0 - faults.motor_green.supply_loss.clamp(0.0, 1.0)),
            pressure_fraction[1].clamp(0.0, 1.0) * (1.0 - faults.motor_yellow.supply_loss.clamp(0.0, 1.0)),
        ];
        let driving_motors = supply.iter().filter(|s| **s > 1e-3).count();
        // 2 motors -> 1.0 (each motor shaft turns with the screw); 1 motor
        // -> 2.0. With none left the ratio is irrelevant (no torque at all).
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
        // Torque scales with the gear ratio, rate gradients with its square
        // (the motor sees `gear * rate` and its torque is multiplied by
        // `gear` again on the way back out); `ServoLoad::geared` does both.
        let mut servo = g.servo.geared(gear);
        servo.add(&y.servo.geared(gear));
        let motor_torque = gear * (g.torque_nm + y.torque_nm);
        let motor_jam_torque = gear * (g.jam_torque_nm + y.jam_torque_nm);
        let motor_jam_damping = gear * gear * (g.jam_damping_nm_s_per_rad + y.jam_damping_nm_s_per_rad);

        // The no-back engages whenever neither motor is actively commanded
        // (a small residual damping torque from an idling motor does not
        // count as "driving").
        let driving = modes[0] == ActuatorMode::Active || modes[1] == ActuatorMode::Active;
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
                // Once it is slipping at capacity the torque no longer
                // responds to rate, so it contributes no gradient.
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

        // The motors' clamped servo law goes to the solver whole (so its
        // clamp is solved, not stepped across); everything else is ordinary
        // linear load on the screw.
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

/// The electric rudder trim actuator: a small screw biasing the SEC's
/// rudder-travel neutral point, driven by its own electric motor-pump (the
/// same `ElectricMotorPump` model as an EHA/EBHA's), with no aerodynamic
/// hinge moment of its own worth modelling (it works against the SEC's
/// internal mechanism, not directly against the rudder's airload).
pub struct RudderTrimActuator {
    pcu: PowerControlUnit,
    pump: ElectricMotorPump,
    angle_rad: f64,
    rate_rad_s: f64,
    inertia_kg_m2: f64,
    limit_rad: f64,
}

impl RudderTrimActuator {
    /// GENERIC: +-20 degree trim authority (a representative rudder-trim
    /// range for a large transport), a small 200 N*m/0.05 rad/s motor
    /// (reflected through its own small screw), light inertia.
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
        // Light structural damping, no aero load: this actuator only fights
        // its own mechanism's friction. The servo's own clamped rate law is
        // solved exactly (`actuator::servo_rate_step`): at
        // `k_torque_per_rate / inertia` ~ 3.2e4 s^-1 it is far too stiff for
        // any explicit step, which is why this actuator needs no sub-stepping
        // of its own -- nothing else here is stiff.
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
        // The window has to end before the *faster* configuration arrives,
        // or both have simply settled on the command and there is no rate
        // left to compare. Both motors run the screw at 0.015 rad/s, so a
        // 5 deg = 0.0873 rad trim takes 5.8 s; sample at 3 s, where the
        // travel so far is the rate times the time:
        //   both motors: 0.015  * 3 s = 0.045  rad
        //   one motor:   0.0075 * 3 s = 0.0225 rad   (the differential
        //                halves the screw rate, see the module doc)
        // The spin-up is negligible against those: 160 kN*m on 2100 kg*m^2
        // is 76 rad/s^2, so the rate limit is reached in under a
        // millisecond.
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
        // Trim out to a deflection first.
        let target = 4.0_f64.to_radians();
        for _ in 0..20_000 {
            ths.step(ACTIVE, target, [1.0, 1.0], &ThsFaults::default(), &calm_aero(), DT);
        }
        let held_angle = ths.angle_rad();
        // Now the motors idle (no new trim command) under a significant
        // dynamic pressure trying to blow the surface back.
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
        // Both motors idling (genuinely "between trim inputs"), a large
        // aero load, and a failed no-back on one of the two systems.
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
    fn a_ballscrew_jam_freezes_the_stabiliser() {
        let mut ths = TrimmableHorizontalStabilizer::new_generic();
        for _ in 0..10_000 {
            ths.step(ACTIVE, 3.0_f64.to_radians(), [1.0, 1.0], &ThsFaults::default(), &calm_aero(), DT);
        }
        let jammed_at = ths.angle_rad();
        let faults = ThsFaults { ballscrew_jam: 1.0, ..Default::default() };
        // Command full nose-up; the jam should keep it essentially frozen.
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
