use std::f64::consts::PI;

pub const PSI_PA: f64 = 6894.757;
pub const GALLON_M3: f64 = 0.003785411784;

pub const HYDRAULIC_SUPPLY_PSI: f64 = 5250.0;
pub const HYDRAULIC_SUPPLY_PA: f64 = HYDRAULIC_SUPPLY_PSI * PSI_PA;

#[derive(Clone, Copy, Debug)]
pub struct ActuatorGeometry {
    pub bore_area_m2: f64,
    pub rod_area_m2: f64,
    pub max_flow_m3_s: f64,
    pub arm_m: f64,
}

impl ActuatorGeometry {
    pub fn new(bore_diameter_m: f64, rod_diameter_m: f64, max_flow_gal_s: f64, arm_m: f64) -> Self {
        let area = |d: f64| PI * (d * 0.5).powi(2);
        Self {
            bore_area_m2: area(bore_diameter_m),
            rod_area_m2: area(rod_diameter_m),
            max_flow_m3_s: max_flow_gal_s * GALLON_M3,
            arm_m: arm_m.max(1e-3),
        }
    }

    pub fn aileron() -> Self {
        Self::new(0.07, 0.0, 0.0825, 0.119)
    }

    pub fn elevator() -> Self {
        Self::new(0.08, 0.0, 0.15, 0.15)
    }

    pub fn rudder() -> Self {
        let bore = 2.0 * (77.18e-4 / PI).sqrt();
        Self::new(bore, 0.0, 0.25, 0.18)
    }

    pub fn spoiler() -> Self {
        let arm = 0.685 * 0.10_f64.hypot(0.26);
        Self::new(0.09, 0.05, 0.23, arm)
    }

    pub fn max_force_n(&self, pressure_pa: f64) -> f64 {
        pressure_pa.max(0.0) * self.bore_area_m2
    }

    pub fn max_torque_nm(&self, pressure_pa: f64) -> f64 {
        self.max_force_n(pressure_pa) * self.arm_m
    }

    pub fn rate_limit_rad_s(&self, pressure_fraction: f64) -> f64 {
        let f = pressure_fraction.max(0.0).sqrt();
        (self.max_flow_m3_s * f / self.bore_area_m2) / self.arm_m
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ActuatorPower {
    Hydraulic,
    ElectroHydrostatic,
    ElectricalBackupHydraulic,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum ActuatorMode {
    #[default]
    Standby,
    Active,
    Damping,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct ActuatorFaults {
    pub supply_loss: f64,
    pub jam: f64,
    pub runaway: f64,
    pub runaway_sign: f64,
    pub transducer_frozen: bool,
    pub transducer_bias_rad: f64,
    pub valve_leakage: f64,
    pub piston_seal_wear: f64,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct ServoLoad {
    pub open_torque_nm: f64,
    pub damping_nm_s_per_rad: f64,
    pub max_torque_nm: f64,
}

impl ServoLoad {
    pub const NONE: Self = Self { open_torque_nm: 0.0, damping_nm_s_per_rad: 0.0, max_torque_nm: 0.0 };

    pub fn add(&mut self, other: &Self) {
        self.open_torque_nm += other.open_torque_nm;
        self.damping_nm_s_per_rad += other.damping_nm_s_per_rad;
        self.max_torque_nm += other.max_torque_nm;
    }

    pub fn geared(&self, g: f64) -> Self {
        Self {
            open_torque_nm: g * self.open_torque_nm,
            damping_nm_s_per_rad: g * g * self.damping_nm_s_per_rad,
            max_torque_nm: g.abs() * self.max_torque_nm,
        }
    }

    pub fn torque_at(&self, rate_rad_s: f64) -> f64 {
        (self.open_torque_nm - self.damping_nm_s_per_rad * rate_rad_s)
            .clamp(-self.max_torque_nm.max(0.0), self.max_torque_nm.max(0.0))
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub struct ActuatorOutput {
    pub torque_nm: f64,
    pub max_torque_nm: f64,
    pub saturated: bool,
    pub servo: ServoLoad,
    pub jam_torque_nm: f64,
    pub jam_damping_nm_s_per_rad: f64,
}

pub fn servo_rate_step(
    rate_rad_s: f64,
    servo: &ServoLoad,
    other_torque_nm: f64,
    other_damping_nm_s_per_rad: f64,
    inertia_kg_m2: f64,
    dt_s: f64,
) -> f64 {
    let dt = dt_s.max(0.0);
    if dt <= 0.0 {
        return rate_rad_s;
    }
    let i_over_dt = inertia_kg_m2.max(1e-9) / dt;
    let c_other = other_damping_nm_s_per_rad.max(0.0);
    let c_servo = servo.damping_nm_s_per_rad.max(0.0);
    let t_max = servo.max_torque_nm.max(0.0);
    let b = other_torque_nm + c_other * rate_rad_s;

    let linear = (i_over_dt * rate_rad_s + b + servo.open_torque_nm) / (i_over_dt + c_other + c_servo);
    let servo_torque = servo.open_torque_nm - c_servo * linear;
    if servo_torque.abs() <= t_max {
        return linear;
    }
    let pinned = if servo_torque > 0.0 { t_max } else { -t_max };
    (i_over_dt * rate_rad_s + b + pinned) / (i_over_dt + c_other)
}

pub struct PowerControlUnit {
    pub geometry: ActuatorGeometry,
    kp_rate_per_rad: f64,
    k_torque_per_rate: f64,
    k_damping: f64,
    k_standby_spring: f64,
    k_standby_damp: f64,
    k_jam_spring: f64,
    k_jam_damp: f64,
    prev_mode: ActuatorMode,
    standby_angle_rad: f64,
    jam_angle_rad: Option<f64>,
    frozen_transducer_rad: Option<f64>,
}

impl PowerControlUnit {
    pub fn new(geometry: ActuatorGeometry) -> Self {
        let max_t = geometry.max_torque_nm(HYDRAULIC_SUPPLY_PA).max(1.0);
        let rated_rate = geometry.rate_limit_rad_s(1.0).max(1e-3);
        Self {
            geometry,
            kp_rate_per_rad: 8.0,
            k_torque_per_rate: max_t / rated_rate * 4.0,
            k_damping: max_t / rated_rate,
            k_standby_spring: max_t * 200.0,
            k_standby_damp: max_t * 20.0,
            k_jam_spring: max_t * 50.0,
            k_jam_damp: max_t * 5.0,
            prev_mode: ActuatorMode::Standby,
            standby_angle_rad: 0.0,
            jam_angle_rad: None,
            frozen_transducer_rad: None,
        }
    }

    pub fn step(
        &mut self,
        mode: ActuatorMode,
        commanded_angle_rad: f64,
        angle_rad: f64,
        rate_rad_s: f64,
        pressure_fraction: f64,
        faults: &ActuatorFaults,
    ) -> ActuatorOutput {
        let supply = pressure_fraction.clamp(0.0, 1.0) * (1.0 - faults.supply_loss.clamp(0.0, 1.0));
        let jam = faults.jam.clamp(0.0, 1.0);
        let leak = faults.valve_leakage.clamp(0.0, 1.0);
        let wear = faults.piston_seal_wear.clamp(0.0, 1.0);
        let rate_derate = (1.0 - 0.6 * leak - 0.2 * wear).clamp(0.05, 1.0);
        let force_derate = (1.0 - 0.5 * wear - 0.2 * leak).clamp(0.05, 1.0);
        let max_torque = self.geometry.max_torque_nm(HYDRAULIC_SUPPLY_PA * supply) * force_derate * (1.0 - jam);
        let rate_limit = self.geometry.rate_limit_rad_s(supply) * rate_derate;
        let stiffness_factor = (rate_derate * force_derate).clamp(0.05, 1.0);

        if mode == ActuatorMode::Standby && self.prev_mode != ActuatorMode::Standby {
            self.standby_angle_rad = angle_rad;
        }
        self.prev_mode = mode;

        if jam > 0.0 {
            if self.jam_angle_rad.is_none() {
                self.jam_angle_rad = Some(angle_rad);
            }
        } else {
            self.jam_angle_rad = None;
        }

        let feedback = if faults.transducer_frozen {
            *self.frozen_transducer_rad.get_or_insert(angle_rad)
        } else {
            self.frozen_transducer_rad = None;
            angle_rad + faults.transducer_bias_rad
        };

        let servo = match mode {
            ActuatorMode::Active => {
                let normal_rate_cmd =
                    (self.kp_rate_per_rad * (commanded_angle_rad - feedback)).clamp(-rate_limit, rate_limit);
                let r = faults.runaway.clamp(0.0, 1.0);
                let rate_cmd = if r > 0.0 {
                    let sign = if faults.runaway_sign >= 0.0 { 1.0 } else { -1.0 };
                    (1.0 - r) * normal_rate_cmd + r * sign * rate_limit
                } else {
                    normal_rate_cmd
                };
                let k = stiffness_factor * self.k_torque_per_rate;
                ServoLoad { open_torque_nm: k * rate_cmd, damping_nm_s_per_rad: k, max_torque_nm: max_torque }
            }
            ActuatorMode::Damping => ServoLoad {
                open_torque_nm: 0.0,
                damping_nm_s_per_rad: stiffness_factor * self.k_damping,
                max_torque_nm: max_torque,
            },
            ActuatorMode::Standby => ServoLoad {
                open_torque_nm: -stiffness_factor * self.k_standby_spring * (angle_rad - self.standby_angle_rad),
                damping_nm_s_per_rad: stiffness_factor * self.k_standby_damp,
                max_torque_nm: max_torque,
            },
        };
        let servo = if max_torque <= 0.0 { ServoLoad::NONE } else { servo };
        let unclamped = servo.open_torque_nm - servo.damping_nm_s_per_rad * rate_rad_s;
        let saturated = unclamped.abs() > max_torque;
        let mut torque = servo.torque_at(rate_rad_s);

        let mut jam_torque = 0.0;
        let mut jam_damping = 0.0;
        if let Some(seize_angle) = self.jam_angle_rad {
            jam_damping = self.k_jam_damp * jam;
            jam_torque = -self.k_jam_spring * jam * (angle_rad - seize_angle) - jam_damping * rate_rad_s;
            torque += jam_torque;
        }

        ActuatorOutput {
            torque_nm: torque,
            max_torque_nm: max_torque,
            saturated,
            servo,
            jam_torque_nm: jam_torque,
            jam_damping_nm_s_per_rad: jam_damping,
        }
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub struct ElectricPumpFaults {
    pub motor_failure: f64,
}

pub struct ElectricMotorPump {
    speed_frac: f64,
    time_constant_s: f64,
}

impl ElectricMotorPump {
    pub fn new(time_constant_s: f64) -> Self {
        Self { speed_frac: 0.0, time_constant_s: time_constant_s.max(1e-3) }
    }

    pub fn step(&mut self, electrical_power_fraction: f64, faults: &ElectricPumpFaults, dt_s: f64) -> f64 {
        let target = electrical_power_fraction.clamp(0.0, 1.0) * (1.0 - faults.motor_failure.clamp(0.0, 1.0));
        let k = 1.0 / self.time_constant_s;
        self.speed_frac = target + (self.speed_frac - target) * (-k * dt_s.max(0.0)).exp();
        self.speed_frac.clamp(0.0, 1.0)
    }

    pub fn pressure_fraction(&self) -> f64 {
        self.speed_frac
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const DT: f64 = 0.01;

    #[test]
    fn aileron_geometry_reproduces_flybywires_cited_force_and_rate() {
        let g = ActuatorGeometry::aileron();
        let force_dan = g.max_force_n(HYDRAULIC_SUPPLY_PA) / 10.0;
        assert!((force_dan - 13934.0).abs() < 50.0, "{force_dan} daN");
        let piston_mm_s = g.rate_limit_rad_s(1.0) * g.arm_m * 1000.0;
        assert!((piston_mm_s - 81.0).abs() < 1.0, "{piston_mm_s} mm/s");
    }

    #[test]
    fn no_nan_at_rest_or_zero_dt() {
        let mut pcu = PowerControlUnit::new(ActuatorGeometry::elevator());
        let out = pcu.step(ActuatorMode::Active, 0.0, 0.0, 0.0, 0.0, &ActuatorFaults::default());
        assert!(out.torque_nm.is_finite() && out.max_torque_nm.is_finite());
        let mut pump = ElectricMotorPump::new(0.5);
        let p = pump.step(1.0, &ElectricPumpFaults::default(), 0.0);
        assert!(p.is_finite());
    }

    #[test]
    fn active_mode_drives_toward_the_commanded_angle() {
        let mut pcu = PowerControlUnit::new(ActuatorGeometry::aileron());
        let out = pcu.step(ActuatorMode::Active, 0.2, 0.0, 0.0, 1.0, &ActuatorFaults::default());
        assert!(out.torque_nm > 0.0);
        assert!(out.torque_nm <= out.max_torque_nm + 1e-6);
        let out = pcu.step(ActuatorMode::Active, -0.2, 0.0, 0.0, 1.0, &ActuatorFaults::default());
        assert!(out.torque_nm < 0.0);
    }

    #[test]
    fn valve_leakage_and_seal_wear_cause_standby_droop_under_a_sustained_load() {
        const FINE_DT: f64 = 0.0005;
        let mut healthy = PowerControlUnit::new(ActuatorGeometry::elevator());
        let mut degraded = PowerControlUnit::new(ActuatorGeometry::elevator());
        let degraded_faults = ActuatorFaults { valve_leakage: 1.0, piston_seal_wear: 1.0, ..Default::default() };
        let inertia = 50.0;
        let external_torque = 2000.0;
        let (mut ha, mut hr, mut da, mut dr) = (0.0_f64, 0.0_f64, 0.0_f64, 0.0_f64);
        for _ in 0..200_000 {
            let ho = healthy.step(ActuatorMode::Standby, 0.0, ha, hr, 1.0, &ActuatorFaults::default());
            hr = servo_rate_step(hr, &ho.servo, external_torque, 0.0, inertia, FINE_DT);
            ha += hr * FINE_DT;

            let deg_out = degraded.step(ActuatorMode::Standby, 0.0, da, dr, 1.0, &degraded_faults);
            dr = servo_rate_step(dr, &deg_out.servo, external_torque, 0.0, inertia, FINE_DT);
            da += dr * FINE_DT;
        }
        assert!(ha.abs() > 0.0 && da.abs() > 0.0);
        assert!((ha.abs() - 3.66e-4).abs() < 1e-5, "healthy droop {ha} should match the spring hand calculation");
        assert!((da.abs() - 6.11e-3).abs() < 1e-4, "degraded droop {da} should match the softened-spring hand calculation");
        assert!(da.abs() > ha.abs() * 5.0, "degraded droop {da} should far exceed healthy droop {ha}");
    }

    #[test]
    fn integrating_active_mode_actually_reaches_the_command() {
        const FINE_DT: f64 = 0.0005;
        let mut pcu = PowerControlUnit::new(ActuatorGeometry::elevator());
        let inertia = 50.0;
        let mut angle = 0.0_f64;
        let mut rate = 0.0_f64;
        for _ in 0..200_000 {
            let out = pcu.step(ActuatorMode::Active, 0.1, angle, rate, 1.0, &ActuatorFaults::default());
            rate = servo_rate_step(rate, &out.servo, 0.0, 0.0, inertia, FINE_DT);
            angle += rate * FINE_DT;
        }
        assert!((angle - 0.1).abs() < 0.01, "settled at {angle}");
    }

    #[test]
    fn damping_mode_only_resists_motion_never_drives() {
        let mut pcu = PowerControlUnit::new(ActuatorGeometry::spoiler());
        let out = pcu.step(ActuatorMode::Damping, 0.5, 0.0, 2.0, 1.0, &ActuatorFaults::default());
        assert!(out.torque_nm < 0.0);
        let out2 = pcu.step(ActuatorMode::Damping, 0.5, 0.0, 0.0, 1.0, &ActuatorFaults::default());
        assert_eq!(out2.torque_nm, 0.0);
    }

    #[test]
    fn standby_mode_holds_the_angle_it_was_entered_at() {
        let mut pcu = PowerControlUnit::new(ActuatorGeometry::rudder());
        pcu.step(ActuatorMode::Standby, 0.0, 0.3, 0.0, 1.0, &ActuatorFaults::default());
        let out = pcu.step(ActuatorMode::Standby, 0.0, 0.4, 0.0, 1.0, &ActuatorFaults::default());
        assert!(out.torque_nm < 0.0);
    }

    #[test]
    fn supply_loss_shrinks_both_force_and_rate_limit() {
        let mut healthy = PowerControlUnit::new(ActuatorGeometry::aileron());
        let mut starved = PowerControlUnit::new(ActuatorGeometry::aileron());
        let h = healthy.step(ActuatorMode::Active, 1.0, 0.0, 0.0, 1.0, &ActuatorFaults::default());
        let s = starved.step(
            ActuatorMode::Active,
            1.0,
            0.0,
            0.0,
            1.0,
            &ActuatorFaults { supply_loss: 0.9, ..Default::default() },
        );
        assert!(s.max_torque_nm < 0.2 * h.max_torque_nm);
    }

    #[test]
    fn a_full_jam_freezes_the_mechanism_against_a_moderate_load() {
        let mut pcu = PowerControlUnit::new(ActuatorGeometry::elevator());
        let faults = ActuatorFaults { jam: 1.0, ..Default::default() };
        let out = pcu.step(ActuatorMode::Active, 0.5, 0.1, 0.0, 1.0, &faults);
        assert_eq!(out.max_torque_nm, 0.0);
        let excursion = pcu.step(ActuatorMode::Active, 0.5, 0.11, 0.0, 1.0, &faults);
        assert!(excursion.torque_nm < 0.0);
    }

    #[test]
    fn runaway_drives_full_rate_in_its_own_direction_regardless_of_command() {
        let mut pcu = PowerControlUnit::new(ActuatorGeometry::rudder());
        let faults = ActuatorFaults { runaway: 1.0, runaway_sign: -1.0, ..Default::default() };
        let out = pcu.step(ActuatorMode::Active, 1.0, 0.0, 0.0, 1.0, &faults);
        assert!(out.torque_nm < 0.0);
    }

    #[test]
    fn a_frozen_transducer_chases_a_stale_reading_even_as_the_true_angle_moves() {
        let mut pcu = PowerControlUnit::new(ActuatorGeometry::aileron());
        let faults = ActuatorFaults { transducer_frozen: true, ..Default::default() };
        pcu.step(ActuatorMode::Active, 0.0, 0.0, 0.0, 1.0, &faults);
        let out = pcu.step(ActuatorMode::Active, 0.0, 0.2, 0.0, 1.0, &faults);
        assert!((out.torque_nm).abs() < 1e-9, "should see no error: {}", out.torque_nm);
    }

    #[test]
    fn electric_pump_ramps_up_and_respects_motor_failure() {
        let mut pump = ElectricMotorPump::new(0.2);
        let mut p = 0.0;
        for _ in 0..1000 {
            p = pump.step(1.0, &ElectricPumpFaults::default(), DT);
        }
        assert!(p > 0.99);
        let mut dead = ElectricMotorPump::new(0.2);
        for _ in 0..1000 {
            p = dead.step(1.0, &ElectricPumpFaults { motor_failure: 1.0 }, DT);
        }
        assert!(p < 1e-6);
    }
}
