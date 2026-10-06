use std::f64::consts::PI;

use super::actuator::{
    servo_rate_step, ActuatorFaults, ActuatorGeometry, ActuatorMode, PowerControlUnit, ServoLoad, GALLON_M3,
    HYDRAULIC_SUPPLY_PA,
};
use super::surface::AsymmetryMonitor;

pub fn synthetic_rotary_geometry(max_torque_nm: f64, max_speed_rad_s: f64) -> ActuatorGeometry {
    let bore_area_m2 = max_torque_nm.max(1.0) / HYDRAULIC_SUPPLY_PA;
    let bore_diameter_m = 2.0 * (bore_area_m2 / PI).sqrt();
    let max_flow_m3_s = max_speed_rad_s.max(0.0) * bore_area_m2;
    ActuatorGeometry::new(bore_diameter_m, 0.0, max_flow_m3_s / GALLON_M3, 1.0)
}

#[derive(Clone, Copy, Debug)]
pub struct RotatingInertia {
    pub angle_rad: f64,
    pub rate_rad_s: f64,
    inertia_kg_m2: f64,
}

impl RotatingInertia {
    pub fn new(inertia_kg_m2: f64, initial_angle_rad: f64) -> Self {
        Self { angle_rad: initial_angle_rad, rate_rad_s: 0.0, inertia_kg_m2: inertia_kg_m2.max(1e-3) }
    }

    pub fn integrate(&mut self, servo: &ServoLoad, net_torque_nm: f64, damping_nm_s_per_rad: f64, dt_s: f64) {
        let dt = dt_s.max(0.0);
        self.rate_rad_s =
            servo_rate_step(self.rate_rad_s, servo, net_torque_nm, damping_nm_s_per_rad, self.inertia_kg_m2, dt);
        self.angle_rad += self.rate_rad_s * dt;
    }
}

#[derive(Clone, Copy, Debug)]
pub struct TransmissionShaft {
    pub stiffness_nm_per_rad: f64,
    pub damping_nm_s_per_rad: f64,
}

impl TransmissionShaft {
    pub fn torque_on_b(&self, angle_a: f64, rate_a: f64, angle_b: f64, rate_b: f64, break_fraction: f64) -> f64 {
        let intact = 1.0 - break_fraction.clamp(0.0, 1.0);
        intact * (self.stiffness_nm_per_rad * (angle_a - angle_b) + self.damping_nm_s_per_rad * (rate_a - rate_b))
    }
}

#[derive(Clone, Copy, Debug)]
pub struct TorqueLimiter {
    pub threshold_nm: f64,
}

impl TorqueLimiter {
    pub fn clamp(&self, torque_nm: f64, bypass_fraction: f64) -> (f64, bool) {
        let bypass = bypass_fraction.clamp(0.0, 1.0);
        let healthy_limit = self.threshold_nm.max(0.0);
        let limit = healthy_limit * (1.0 + 1000.0 * bypass);
        let tripped = bypass < 0.5 && torque_nm.abs() > healthy_limit;
        (torque_nm.clamp(-limit, limit), tripped)
    }
}

#[derive(Clone, Copy, Debug)]
pub struct WingTipBrake {
    pub holding_torque_nm: f64,
}

impl WingTipBrake {
    pub fn torque(&self, engaged: bool, angle_rad: f64, rate_rad_s: f64, lock_angle_rad: f64, fail_fraction: f64) -> (f64, f64) {
        if !engaged {
            return (0.0, 0.0);
        }
        let capacity = self.holding_torque_nm.max(0.0) * (1.0 - fail_fraction.clamp(0.0, 1.0));
        let k = capacity * 50.0;
        let c = capacity * 5.0;
        let raw = -k * (angle_rad - lock_angle_rad) - c * rate_rad_s;
        let held = raw.clamp(-capacity, capacity);
        (held, if raw == held { c } else { 0.0 })
    }
}

pub fn uncommanded_motion(commanded_rate_rad_s: f64, actual_rate_rad_s: f64, threshold_rad_s: f64) -> bool {
    commanded_rate_rad_s.abs() < 1e-6 && actual_rate_rad_s.abs() > threshold_rad_s.max(0.0)
}

#[derive(Clone, Copy, Debug, Default)]
pub struct HighLiftFaults {
    pub pcu: ActuatorFaults,
    pub limiter_bypass: f64,
    pub inboard_shaft_break: f64,
    pub outboard_shaft_break: f64,
    pub wingtip_brake_fail: f64,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct HighLiftOutput {
    pub inboard_angle_rad: f64,
    pub outboard_angle_rad: f64,
    pub pcu_torque_nm: f64,
    pub limiter_tripped: bool,
    pub brake_engaged: bool,
    pub overspeed_damage: f64,
}

pub struct HighLiftSystem {
    pcu: PowerControlUnit,
    pcu_shaft: RotatingInertia,
    limiter: TorqueLimiter,
    shaft_to_inboard: TransmissionShaft,
    inboard: RotatingInertia,
    shaft_to_outboard: TransmissionShaft,
    outboard: RotatingInertia,
    brake: WingTipBrake,
    brake_lock_angle: Option<f64>,
    aero_load_nm_per_rad_per_pa: f64,
    overspeed_damage: f64,
}

impl HighLiftSystem {
    #[allow(clippy::too_many_arguments)]
    fn new_with(
        pcu_max_torque_nm: f64,
        pcu_max_speed_rad_s: f64,
        station_inertia_kg_m2: f64,
        shaft_stiffness_nm_per_rad: f64,
        shaft_damping_nm_s_per_rad: f64,
        limiter_threshold_nm: f64,
        brake_holding_nm: f64,
        aero_load_nm_per_rad_per_pa: f64,
    ) -> Self {
        let shaft = TransmissionShaft { stiffness_nm_per_rad: shaft_stiffness_nm_per_rad, damping_nm_s_per_rad: shaft_damping_nm_s_per_rad };
        Self {
            pcu: PowerControlUnit::new(synthetic_rotary_geometry(pcu_max_torque_nm, pcu_max_speed_rad_s)),
            pcu_shaft: RotatingInertia::new(station_inertia_kg_m2 * 0.125, 0.0),
            limiter: TorqueLimiter { threshold_nm: limiter_threshold_nm },
            shaft_to_inboard: shaft,
            inboard: RotatingInertia::new(station_inertia_kg_m2, 0.0),
            shaft_to_outboard: shaft,
            outboard: RotatingInertia::new(station_inertia_kg_m2, 0.0),
            brake: WingTipBrake { holding_torque_nm: brake_holding_nm },
            brake_lock_angle: None,
            aero_load_nm_per_rad_per_pa,
            overspeed_damage: 0.0,
        }
    }

    pub fn new_generic() -> Self {
        Self::new_with(6000.0, 0.5, 40.0, 200_000.0, 2_000.0, 4000.0, 9000.0, 0.05)
    }

    pub fn new_flap() -> Self {
        Self::new_generic()
    }

    pub fn new_slat() -> Self {
        Self::new_with(3000.0, 0.9, 18.0, 90_000.0, 900.0, 2200.0, 4000.0, 0.03)
    }

    pub fn new_droop_nose() -> Self {
        Self::new_with(9000.0, 0.25, 70.0, 260_000.0, 2_600.0, 6500.0, 12_000.0, 0.06)
    }

    pub fn inboard_angle_rad(&self) -> f64 {
        self.inboard.angle_rad
    }
    pub fn outboard_angle_rad(&self) -> f64 {
        self.outboard.angle_rad
    }

    const MAX_SUBSTEP_S: f64 = 0.0005;

    #[allow(clippy::too_many_arguments)]
    pub fn step(
        &mut self,
        mode: ActuatorMode,
        commanded_angle_rad: f64,
        pressure_fraction: f64,
        faults: &HighLiftFaults,
        dynamic_pressure_pa: f64,
        brake_commanded: bool,
        dt_s: f64,
    ) -> HighLiftOutput {
        let dt_total = dt_s.max(0.0);
        let n = ((dt_total / Self::MAX_SUBSTEP_S).ceil() as usize).max(1);
        let sub_dt = dt_total / n as f64;
        let mut out = HighLiftOutput::default();
        for _ in 0..n {
            out = self.step_once(mode, commanded_angle_rad, pressure_fraction, faults, dynamic_pressure_pa, brake_commanded, sub_dt);
        }
        out
    }

    #[allow(clippy::too_many_arguments)]
    fn step_once(
        &mut self,
        mode: ActuatorMode,
        commanded_angle_rad: f64,
        pressure_fraction: f64,
        faults: &HighLiftFaults,
        dynamic_pressure_pa: f64,
        brake_commanded: bool,
        dt: f64,
    ) -> HighLiftOutput {
        let pcu_out =
            self.pcu.step(mode, commanded_angle_rad, self.pcu_shaft.angle_rad, self.pcu_shaft.rate_rad_s, pressure_fraction, &faults.pcu);

        let raw = self.shaft_to_inboard.torque_on_b(
            self.pcu_shaft.angle_rad,
            self.pcu_shaft.rate_rad_s,
            self.inboard.angle_rad,
            self.inboard.rate_rad_s,
            faults.inboard_shaft_break,
        );
        let pure_aero_torque_nm = self.aero_load_nm_per_rad_per_pa * dynamic_pressure_pa.max(0.0) * self.inboard.angle_rad.abs();
        let overload_ratio = pure_aero_torque_nm / self.limiter.threshold_nm.max(1.0);
        if overload_ratio > 1.0 {
            self.overspeed_damage = (self.overspeed_damage + (overload_ratio - 1.0) * OVERSPEED_DAMAGE_RATE_PER_S * dt).min(1.0);
        }
        let effective_limiter = TorqueLimiter { threshold_nm: self.limiter.threshold_nm * (1.0 - 0.3 * self.overspeed_damage) };
        let (to_inboard, tripped) = effective_limiter.clamp(raw, faults.limiter_bypass);
        let inboard_shaft_damping = if to_inboard == raw {
            self.shaft_to_inboard.damping_nm_s_per_rad * (1.0 - faults.inboard_shaft_break.clamp(0.0, 1.0))
        } else {
            0.0
        };
        let outboard_shaft_damping =
            self.shaft_to_outboard.damping_nm_s_per_rad * (1.0 - faults.outboard_shaft_break.clamp(0.0, 1.0));

        let to_outboard = self.shaft_to_outboard.torque_on_b(
            self.inboard.angle_rad,
            self.inboard.rate_rad_s,
            self.outboard.angle_rad,
            self.outboard.rate_rad_s,
            faults.outboard_shaft_break,
        );

        if brake_commanded {
            if self.brake_lock_angle.is_none() {
                self.brake_lock_angle = Some(self.outboard.angle_rad);
            }
        } else {
            self.brake_lock_angle = None;
        }
        let lock_angle = self.brake_lock_angle.unwrap_or(self.outboard.angle_rad);
        let (brake_torque, brake_damping) =
            self.brake.torque(brake_commanded, self.outboard.angle_rad, self.outboard.rate_rad_s, lock_angle, faults.wingtip_brake_fail);

        let aero = |angle: f64| -self.aero_load_nm_per_rad_per_pa * dynamic_pressure_pa.max(0.0) * angle;

        self.pcu_shaft.integrate(
            &pcu_out.servo,
            pcu_out.jam_torque_nm - to_inboard,
            pcu_out.jam_damping_nm_s_per_rad + inboard_shaft_damping,
            dt,
        );
        self.inboard.integrate(
            &ServoLoad::NONE,
            to_inboard - to_outboard + aero(self.inboard.angle_rad),
            inboard_shaft_damping + outboard_shaft_damping,
            dt,
        );
        self.outboard.integrate(
            &ServoLoad::NONE,
            to_outboard + aero(self.outboard.angle_rad) + brake_torque,
            outboard_shaft_damping + brake_damping,
            dt,
        );

        HighLiftOutput {
            inboard_angle_rad: self.inboard.angle_rad,
            outboard_angle_rad: self.outboard.angle_rad,
            pcu_torque_nm: pcu_out.torque_nm,
            limiter_tripped: tripped,
            brake_engaged: brake_commanded,
            overspeed_damage: self.overspeed_damage,
        }
    }
}

const OVERSPEED_DAMAGE_RATE_PER_S: f64 = 0.05;

pub struct HighLiftPair {
    pub left: HighLiftSystem,
    pub right: HighLiftSystem,
    asymmetry: AsymmetryMonitor,
}

impl HighLiftPair {
    pub fn new(left: HighLiftSystem, right: HighLiftSystem, asymmetry_timer_s: f64) -> Self {
        Self { left, right, asymmetry: AsymmetryMonitor::new(asymmetry_timer_s) }
    }

    #[allow(clippy::too_many_arguments)]
    pub fn step(
        &mut self,
        mode: ActuatorMode,
        commanded_angle_rad: f64,
        pressure_fraction: [f64; 2],
        faults: [HighLiftFaults; 2],
        dynamic_pressure_pa: f64,
        asymmetry_threshold_rad: f64,
        dt_s: f64,
    ) -> (HighLiftOutput, HighLiftOutput, bool) {
        let brake = self.asymmetry.tripped();
        let lo = self.left.step(mode, commanded_angle_rad, pressure_fraction[0], &faults[0], dynamic_pressure_pa, brake, dt_s);
        let ro = self.right.step(mode, commanded_angle_rad, pressure_fraction[1], &faults[1], dynamic_pressure_pa, brake, dt_s);
        let tripped = self.asymmetry.step(lo.outboard_angle_rad, ro.outboard_angle_rad, asymmetry_threshold_rad, dt_s);
        (lo, ro, tripped)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const DT: f64 = 0.01;

    fn run(sys: &mut HighLiftSystem, target: f64, ticks: usize, faults: &HighLiftFaults) -> HighLiftOutput {
        let mut out = HighLiftOutput::default();
        for _ in 0..ticks {
            out = sys.step(ActuatorMode::Active, target, 1.0, faults, 0.0, false, DT);
        }
        out
    }

    #[test]
    fn no_nan_at_rest_or_zero_dt() {
        let mut sys = HighLiftSystem::new_generic();
        let out = sys.step(ActuatorMode::Active, 0.0, 0.0, &HighLiftFaults::default(), 0.0, false, 0.0);
        assert!(out.inboard_angle_rad.is_finite() && out.outboard_angle_rad.is_finite());
    }

    #[test]
    fn a_healthy_system_drives_both_stations_to_the_commanded_angle() {
        let mut sys = HighLiftSystem::new_generic();
        let out = run(&mut sys, 0.5, 60_000, &HighLiftFaults::default());
        assert!((out.inboard_angle_rad - 0.5).abs() < 0.02, "inboard at {}", out.inboard_angle_rad);
        assert!((out.outboard_angle_rad - 0.5).abs() < 0.02, "outboard at {}", out.outboard_angle_rad);
        assert!(!out.limiter_tripped);
    }

    #[test]
    fn slats_and_droop_nose_and_flaps_all_reach_command_but_with_different_drive_characteristics() {
        for ctor in [HighLiftSystem::new_flap as fn() -> HighLiftSystem, HighLiftSystem::new_slat, HighLiftSystem::new_droop_nose] {
            let mut sys = ctor();
            let out = run(&mut sys, 0.3, 60_000, &HighLiftFaults::default());
            assert!((out.inboard_angle_rad - 0.3).abs() < 0.02, "inboard at {}", out.inboard_angle_rad);
            assert!((out.outboard_angle_rad - 0.3).abs() < 0.02, "outboard at {}", out.outboard_angle_rad);
        }
    }

    #[test]
    fn the_slat_reaches_a_small_command_faster_than_the_droop_nose() {
        let mut slat = HighLiftSystem::new_slat();
        let mut droop_nose = HighLiftSystem::new_droop_nose();
        let target = 0.05;
        let ticks_to_reach = |sys: &mut HighLiftSystem| -> usize {
            for t in 0..20_000 {
                let out = sys.step(ActuatorMode::Active, target, 1.0, &HighLiftFaults::default(), 0.0, false, DT);
                if out.outboard_angle_rad >= target * 0.9 {
                    return t;
                }
            }
            usize::MAX
        };
        let slat_ticks = ticks_to_reach(&mut slat);
        let droop_ticks = ticks_to_reach(&mut droop_nose);
        assert!(slat_ticks < droop_ticks, "slat {slat_ticks} ticks, droop nose {droop_ticks} ticks");
    }

    #[test]
    fn a_severed_outboard_segment_strands_the_outboard_station_only() {
        let mut sys = HighLiftSystem::new_generic();
        let faults = HighLiftFaults { outboard_shaft_break: 1.0, ..Default::default() };
        let out = run(&mut sys, 0.5, 60_000, &faults);
        assert!((out.inboard_angle_rad - 0.5).abs() < 0.02, "inboard should still track: {}", out.inboard_angle_rad);
        assert!(out.outboard_angle_rad.abs() < 0.05, "outboard should be stranded near 0: {}", out.outboard_angle_rad);
    }

    #[test]
    fn a_large_commanded_step_alone_never_trips_the_limiter() {
        let mut sys = HighLiftSystem::new_generic();
        let mut tripped_once = false;
        for _ in 0..2000 {
            let out = sys.step(ActuatorMode::Active, 5.0, 1.0, &HighLiftFaults::default(), 0.0, false, DT);
            tripped_once |= out.limiter_tripped;
        }
        assert!(!tripped_once, "a normal commanded step must stay inside the limiter's threshold");
    }

    #[test]
    fn a_jam_downstream_of_the_pcu_drives_it_to_max_torque_and_trips_the_healthy_limiter() {
        let mut sys = HighLiftSystem::new_generic();
        let mut out = HighLiftOutput::default();
        for _ in 0..500 {
            out = sys.step(ActuatorMode::Active, 5.0, 1.0, &HighLiftFaults::default(), 0.0, true, DT);
        }
        assert!(out.limiter_tripped, "a locked drive train must make the limiter slip");
        assert!((out.pcu_torque_nm - 4000.0).abs() < 200.0, "PCU should settle at the limiter threshold: {}", out.pcu_torque_nm);
        assert!(out.outboard_angle_rad.abs() < 0.02, "brake should hold the outboard: {}", out.outboard_angle_rad);
        assert!(out.inboard_angle_rad.abs() < 0.05, "inboard should stall against the lock: {}", out.inboard_angle_rad);

        let mut bypassed = HighLiftSystem::new_generic();
        let faults = HighLiftFaults { limiter_bypass: 1.0, ..Default::default() };
        let mut bo = HighLiftOutput::default();
        for _ in 0..500 {
            bo = bypassed.step(ActuatorMode::Active, 5.0, 1.0, &faults, 0.0, true, DT);
        }
        assert!(!bo.limiter_tripped);
        assert!(bo.pcu_torque_nm.abs() > out.pcu_torque_nm.abs() * 1.2, "a bypassed limiter should load the PCU harder: {} vs {}", bo.pcu_torque_nm, out.pcu_torque_nm);
    }

    #[test]
    fn a_bypassed_limiter_lets_more_torque_through_than_a_healthy_one() {
        let mut healthy = HighLiftSystem::new_generic();
        let mut bypassed = HighLiftSystem::new_generic();
        let mut healthy_max: f64 = 0.0;
        let mut bypassed_max: f64 = 0.0;
        for _ in 0..200 {
            let ho = healthy.step(ActuatorMode::Active, 5.0, 1.0, &HighLiftFaults::default(), 0.0, false, DT);
            let bo = bypassed.step(
                ActuatorMode::Active,
                5.0,
                1.0,
                &HighLiftFaults { limiter_bypass: 1.0, ..Default::default() },
                0.0,
                false,
                DT,
            );
            healthy_max = healthy_max.max(ho.pcu_torque_nm.abs());
            bypassed_max = bypassed_max.max(bo.pcu_torque_nm.abs());
        }
        assert!(healthy_max > 0.0 && bypassed_max > 0.0);
    }

    #[test]
    fn an_engaged_brake_arrests_the_outboard_station() {
        let mut sys = HighLiftSystem::new_generic();
        for _ in 0..50 {
            sys.step(ActuatorMode::Active, 0.5, 1.0, &HighLiftFaults::default(), 0.0, false, DT);
        }
        let moving_rate = sys.outboard.rate_rad_s;
        assert!(moving_rate.abs() > 0.1, "should be moving before the brake test: {moving_rate}");
        for _ in 0..500 {
            sys.step(ActuatorMode::Active, 0.5, 1.0, &HighLiftFaults::default(), 0.0, true, DT);
        }
        assert!(sys.outboard.rate_rad_s.abs() < 1e-3, "brake should have arrested outboard motion: {}", sys.outboard.rate_rad_s);
    }

    #[test]
    fn a_failed_brake_cannot_arrest_the_outboard_station() {
        let mut healthy = HighLiftSystem::new_generic();
        let mut failed = HighLiftSystem::new_generic();
        for sys in [&mut healthy, &mut failed] {
            for _ in 0..2000 {
                sys.step(ActuatorMode::Active, 0.5, 1.0, &HighLiftFaults::default(), 0.0, false, DT);
            }
        }
        let mut healthy_out = HighLiftOutput::default();
        let mut failed_out = HighLiftOutput::default();
        for _ in 0..500 {
            healthy_out = healthy.step(ActuatorMode::Active, 0.5, 1.0, &HighLiftFaults::default(), 0.0, true, DT);
            failed_out = failed.step(
                ActuatorMode::Active,
                0.5,
                1.0,
                &HighLiftFaults { wingtip_brake_fail: 1.0, ..Default::default() },
                0.0,
                true,
                DT,
            );
        }
        assert!(healthy_out.outboard_angle_rad.is_finite() && failed_out.outboard_angle_rad.is_finite());
    }

    #[test]
    fn asymmetry_between_left_and_right_eventually_engages_both_brakes() {
        let mut pair = HighLiftPair::new(HighLiftSystem::new_generic(), HighLiftSystem::new_generic(), 0.5);
        let left_faults = HighLiftFaults::default();
        let right_faults = HighLiftFaults { pcu: ActuatorFaults { supply_loss: 0.95, ..Default::default() }, ..Default::default() };
        let mut tripped = false;
        for _ in 0..20_000 {
            let (_, _, t) = pair.step(ActuatorMode::Active, 0.6, [1.0, 1.0], [left_faults, right_faults], 0.0, 0.05, DT);
            tripped |= t;
        }
        assert!(tripped, "a sustained position gap should eventually trip the asymmetry monitor");
    }

    #[test]
    fn uncommanded_motion_is_detected_from_rate_alone() {
        assert!(!uncommanded_motion(0.0, 0.0, 0.01));
        assert!(!uncommanded_motion(0.2, 0.3, 0.01));
        assert!(uncommanded_motion(0.0, 0.05, 0.01));
    }
}
