use super::MLW_KG;

const MAX_BRAKING_DECEL_MS2: f64 = 2.8;
const BRAKED_WHEEL_COUNT: f64 = 16.0;
fn max_brake_force_n() -> f64 {
    MLW_KG * MAX_BRAKING_DECEL_MS2 / BRAKED_WHEEL_COUNT
}

const CARBON_SPECIFIC_HEAT_J_KG_K: f64 = 1650.0;
const STACK_MASS_KG: f64 = 100.0;
const STACK_CAPACITY_J_K: f64 = CARBON_SPECIFIC_HEAT_J_KG_K * STACK_MASS_KG;

const CONVECTION_STATIC_W_M2K: f64 = 12.0;
const CONVECTION_ROLLING_W_M2K: f64 = 120.0;
const CONVECTION_FULL_SPEED_MS: f64 = 40.0;
const SURFACE_AREA_M2: f64 = 1.2;
const EMISSIVITY: f64 = 0.85;
const STEFAN_BOLTZMANN: f64 = 5.670_374e-8;

const WEAR_LIFE_ENERGY_J: f64 = 5.0e10;

const FIRE_TEMP_C: f64 = 800.0;
const FIRE_ARM_SECONDS: f64 = 60.0;

const SKID_SLIP_THRESHOLD: f64 = 0.15;
const ANTISKID_RATE_PER_S: f64 = 4.0;

const WHEEL_RADIUS_M: f64 = 0.613;
const WHEEL_INERTIA_KG_M2: f64 = 25.0;
const M_EFF_KG: f64 = WHEEL_INERTIA_KG_M2 / (WHEEL_RADIUS_M * WHEEL_RADIUS_M);
const MU_TIRE_GROUND: f64 = 0.8;
const BEARING_DRAG_DECEL_MS2: f64 = 0.3;

#[derive(Clone, Copy, Debug, Default)]
pub struct BrakeFaults {
    pub antiskid_inop: f64,
    pub dragging: f64,
}

#[derive(Clone, Copy, Debug)]
pub struct BrakeWheelInputs {
    pub commanded: f64,
    pub on_ground: bool,
    pub normal_load_n: f64,
    pub groundspeed_ms: f64,
    pub ambient_c: f64,
    pub dt_s: f64,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct BrakeWheelOutputs {
    pub stack_temp_c: f64,
    pub applied_fraction: f64,
    pub wear_fraction: f64,
    pub fire: bool,
    pub skidding: bool,
    pub wheel_speed_ms: f64,
}

pub struct BrakeWheel {
    stack_temp_c: f64,
    wear_fraction: f64,
    antiskid_release: f64,
    fire: bool,
    seconds_above_fire_temp: f64,
    wheel_speed_ms: f64,
}

impl BrakeWheel {
    pub fn new(ambient_c: f64) -> Self {
        Self { stack_temp_c: ambient_c, wear_fraction: 0.0, antiskid_release: 0.0, fire: false, seconds_above_fire_temp: 0.0, wheel_speed_ms: 0.0 }
    }

    pub fn stack_temp_c(&self) -> f64 {
        self.stack_temp_c
    }

    pub fn wheel_speed_ms(&self) -> f64 {
        self.wheel_speed_ms
    }

    pub fn step(&mut self, inputs: &BrakeWheelInputs, faults: &BrakeFaults) -> BrakeWheelOutputs {
        let dt = inputs.dt_s.max(0.0);
        let gs = inputs.groundspeed_ms.max(0.0);

        let slip = if gs > 0.5 { ((gs - self.wheel_speed_ms.max(0.0)) / gs).clamp(0.0, 1.0) } else { 0.0 };
        let antiskid_effective = (1.0 - faults.antiskid_inop.clamp(0.0, 1.0)).max(0.0);
        let skidding = slip > SKID_SLIP_THRESHOLD;
        let release_target = if skidding { (slip - SKID_SLIP_THRESHOLD) * antiskid_effective } else { 0.0 };
        if release_target > self.antiskid_release {
            self.antiskid_release += (ANTISKID_RATE_PER_S * dt).min(release_target - self.antiskid_release);
        } else {
            self.antiskid_release -= (ANTISKID_RATE_PER_S * dt).min(self.antiskid_release - release_target);
        }
        self.antiskid_release = self.antiskid_release.clamp(0.0, 1.0);

        let commanded = inputs.commanded.clamp(0.0, 1.0);
        let applied_fraction = (commanded * (1.0 - self.antiskid_release) + faults.dragging.clamp(0.0, 1.0)).clamp(0.0, 1.0);
        let brake_resistance_n = applied_fraction * max_brake_force_n();

        if dt > 0.0 {
            let f_avail = if inputs.on_ground { MU_TIRE_GROUND * inputs.normal_load_n.max(0.0) } else { 0.0 };
            let desired_accel_force = M_EFF_KG * (gs - self.wheel_speed_ms) / dt;
            let f_req = desired_accel_force + brake_resistance_n;
            let f_friction = f_req.clamp(-f_avail, f_avail);
            let mut net_force = f_friction - brake_resistance_n;
            if !inputs.on_ground {
                net_force -= BEARING_DRAG_DECEL_MS2 * M_EFF_KG * self.wheel_speed_ms.signum();
            }
            self.wheel_speed_ms += net_force / M_EFF_KG * dt;
            self.wheel_speed_ms = self.wheel_speed_ms.max(0.0);
        }

        let heat_power_w = brake_resistance_n * self.wheel_speed_ms.max(0.0);

        let convection_w_m2k = CONVECTION_STATIC_W_M2K + (CONVECTION_ROLLING_W_M2K - CONVECTION_STATIC_W_M2K) * (gs / CONVECTION_FULL_SPEED_MS).min(1.0);
        let temp_k = self.stack_temp_c + 273.15;
        let ambient_k = inputs.ambient_c + 273.15;
        let radiative_w = EMISSIVITY * STEFAN_BOLTZMANN * SURFACE_AREA_M2 * (temp_k.powi(4) - ambient_k.powi(4));
        let convective_w = convection_w_m2k * SURFACE_AREA_M2 * (self.stack_temp_c - inputs.ambient_c);
        let net_w = heat_power_w - convective_w - radiative_w;
        self.stack_temp_c += net_w * dt / STACK_CAPACITY_J_K;

        self.wear_fraction = (self.wear_fraction + heat_power_w * dt / WEAR_LIFE_ENERGY_J).min(1.0);

        if self.stack_temp_c > FIRE_TEMP_C {
            self.seconds_above_fire_temp += dt;
            if self.seconds_above_fire_temp > FIRE_ARM_SECONDS {
                self.fire = true;
            }
        } else {
            self.seconds_above_fire_temp = 0.0;
        }

        BrakeWheelOutputs { stack_temp_c: self.stack_temp_c, applied_fraction, wear_fraction: self.wear_fraction, fire: self.fire, skidding, wheel_speed_ms: self.wheel_speed_ms }
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub struct ParkingBrakeFaults {
    pub leak: f64,
}

const PRECHARGE_PA: f64 = 10.0e6;
const GAS_VOLUME_M3: f64 = 0.0015;
const CHARGE_VOLUME_M3: f64 = 0.0006;
const LEAK_RATE_M3_S_AT_FULL_MAGNITUDE: f64 = CHARGE_VOLUME_M3 / (3.0 * 3600.0);
const MIN_HOLDING_PA: f64 = 4.0e6;

pub struct ParkingBrakeAccumulator {
    discharged_m3: f64,
    applied: bool,
}

impl ParkingBrakeAccumulator {
    pub fn new() -> Self {
        Self { discharged_m3: 0.0, applied: false }
    }

    pub fn pressure_pa(&self) -> f64 {
        PRECHARGE_PA * GAS_VOLUME_M3 / (GAS_VOLUME_M3 + self.discharged_m3)
    }

    pub fn step(&mut self, parking_brake_set: bool, faults: &ParkingBrakeFaults, dt_s: f64) -> (f64, bool) {
        let dt = dt_s.max(0.0);
        if parking_brake_set && !self.applied {
            self.discharged_m3 += CHARGE_VOLUME_M3;
        }
        self.applied = parking_brake_set;
        if !parking_brake_set {
            self.discharged_m3 = (self.discharged_m3 - CHARGE_VOLUME_M3).max(0.0);
        } else {
            self.discharged_m3 += LEAK_RATE_M3_S_AT_FULL_MAGNITUDE * faults.leak.clamp(0.0, 1.0) * dt;
        }
        self.discharged_m3 = self.discharged_m3.max(0.0);
        let pressure = self.pressure_pa();
        let holding = parking_brake_set && pressure >= MIN_HOLDING_PA;
        (pressure, holding)
    }
}

impl Default for ParkingBrakeAccumulator {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    #[test]
    fn the_wheel_radius_follows_from_the_published_tyre_size() {
        const MM_PER_IN: f64 = 25.4;
        let free_radius_mm = 1400.0 / 2.0;
        let rim_radius_mm = 23.0 * MM_PER_IN / 2.0;
        let section_height_mm = free_radius_mm - rim_radius_mm;
        const STANDARD_DEFLECTION: f64 = 0.32;
        let deflection_mm = STANDARD_DEFLECTION * section_height_mm;
        let slr_mm = free_radius_mm - deflection_mm;
        let rolling_mm = free_radius_mm - (2.0 / 3.0) * deflection_mm;

        assert!((section_height_mm - 407.9).abs() < 0.1, "{section_height_mm}");
        assert!((slr_mm - 569.5).abs() < 0.2, "{slr_mm}");
        assert!((rolling_mm / 1000.0 - WHEEL_RADIUS_M).abs() < 1e-3, "{rolling_mm} mm vs {WHEEL_RADIUS_M} m");
        assert!(slr_mm < rolling_mm && rolling_mm < free_radius_mm);
    }

    #[test]
    fn the_per_wheel_brake_force_is_the_btv_dry_deceleration_shared_out() {
        let expected = MLW_KG * MAX_BRAKING_DECEL_MS2 / BRAKED_WHEEL_COUNT;
        assert!((max_brake_force_n() - expected).abs() < 1e-6);
        assert!((max_brake_force_n() - 69_125.0).abs() < 50.0, "{}", max_brake_force_n());
        let total_n = max_brake_force_n() * BRAKED_WHEEL_COUNT;
        assert!((total_n / MLW_KG - MAX_BRAKING_DECEL_MS2).abs() < 1e-9);
    }

    use super::*;

    fn healthy() -> BrakeFaults {
        BrakeFaults::default()
    }

    #[test]
    fn braking_while_rolling_heats_the_stack_above_ambient() {
        let mut w = BrakeWheel::new(15.0);
        let inputs = BrakeWheelInputs { commanded: 1.0, on_ground: true, normal_load_n: 200_000.0, groundspeed_ms: 50.0, ambient_c: 15.0, dt_s: 1.0 };
        let mut out = BrakeWheelOutputs::default();
        for _ in 0..120 {
            out = w.step(&inputs, &healthy());
        }
        assert!((out.wheel_speed_ms - 50.0).abs() < 1.0, "with ample friction the wheel must track groundspeed, not skid: {}", out.wheel_speed_ms);
        assert!(out.stack_temp_c > 100.0, "sustained hard braking must heat the stack well above ambient: {}", out.stack_temp_c);
        assert!(out.wear_fraction > 0.0);
    }

    #[test]
    fn no_command_means_no_heating_and_no_wear() {
        let mut w = BrakeWheel::new(15.0);
        let inputs = BrakeWheelInputs { commanded: 0.0, on_ground: true, normal_load_n: 200_000.0, groundspeed_ms: 50.0, ambient_c: 15.0, dt_s: 1.0 };
        let mut out = BrakeWheelOutputs::default();
        for _ in 0..60 {
            out = w.step(&inputs, &healthy());
        }
        assert!((out.stack_temp_c - 15.0).abs() < 1.0);
        assert_eq!(out.wear_fraction, 0.0);
    }

    #[test]
    fn insufficient_friction_produces_a_genuine_physically_caused_lockup() {
        let mut w = BrakeWheel::new(15.0);
        let faults = BrakeFaults { antiskid_inop: 1.0, dragging: 0.0 };
        let inputs = BrakeWheelInputs { commanded: 1.0, on_ground: true, normal_load_n: 50_000.0, groundspeed_ms: 60.0, ambient_c: 15.0, dt_s: 0.1 };
        let mut out = BrakeWheelOutputs::default();
        for _ in 0..50 {
            out = w.step(&inputs, &faults);
        }
        assert!(out.wheel_speed_ms < 1.0, "brake torque past the friction limit must lock the wheel: {}", out.wheel_speed_ms);
        assert!(out.skidding);
        assert!((out.applied_fraction - 1.0).abs() < 1e-9, "an inoperative antiskid channel leaves full pressure applied even while skidding");
    }

    #[test]
    fn a_healthy_antiskid_channel_keeps_releasing_and_recovering_instead_of_a_permanent_lockup() {
        let mut w = BrakeWheel::new(15.0);
        let inputs = BrakeWheelInputs { commanded: 1.0, on_ground: true, normal_load_n: 50_000.0, groundspeed_ms: 60.0, ambient_c: 15.0, dt_s: 0.1 };
        let mut ever_skidded = false;
        let mut min_applied_fraction = 1.0_f64;
        let mut max_wheel_speed = 0.0_f64;
        for _ in 0..500 {
            let out = w.step(&inputs, &healthy());
            ever_skidded |= out.skidding;
            min_applied_fraction = min_applied_fraction.min(out.applied_fraction);
            max_wheel_speed = max_wheel_speed.max(out.wheel_speed_ms);
        }
        assert!(ever_skidded, "the wheel must skid at least once under this brake/friction mismatch");
        assert!(min_applied_fraction < 1.0, "a healthy antiskid channel must release pressure at some point");
        assert!(max_wheel_speed > 30.0, "unlike the disabled-antiskid case, the wheel must recover partway back toward groundspeed at some point");
    }

    #[test]
    fn a_wheel_keeps_spinning_for_a_while_after_liftoff_instead_of_stopping_instantly() {
        let mut w = BrakeWheel::new(15.0);
        let ground_inputs = BrakeWheelInputs { commanded: 0.0, on_ground: true, normal_load_n: 200_000.0, groundspeed_ms: 70.0, ambient_c: 15.0, dt_s: 0.5 };
        let mut out = BrakeWheelOutputs::default();
        for _ in 0..10 {
            out = w.step(&ground_inputs, &healthy());
        }
        assert!((out.wheel_speed_ms - 70.0).abs() < 1.0);

        let air_inputs = BrakeWheelInputs { commanded: 0.0, on_ground: false, normal_load_n: 0.0, groundspeed_ms: 90.0, ambient_c: 15.0, dt_s: 0.5 };
        let just_after = w.step(&air_inputs, &healthy());
        assert!(just_after.wheel_speed_ms > 60.0, "a wheel must not stop instantly the moment it leaves the ground: {}", just_after.wheel_speed_ms);

        let mut later = just_after;
        for _ in 0..400 {
            later = w.step(&air_inputs, &healthy());
        }
        assert!(later.wheel_speed_ms < just_after.wheel_speed_ms, "residual bearing drag must eventually slow a free-spinning wheel");
    }

    #[test]
    fn dragging_brake_heats_the_stack_with_zero_command() {
        let mut w = BrakeWheel::new(15.0);
        let faults = BrakeFaults { antiskid_inop: 0.0, dragging: 0.5 };
        let inputs = BrakeWheelInputs { commanded: 0.0, on_ground: true, normal_load_n: 100_000.0, groundspeed_ms: 30.0, ambient_c: 15.0, dt_s: 1.0 };
        let mut out = BrakeWheelOutputs::default();
        for _ in 0..120 {
            out = w.step(&inputs, &faults);
        }
        assert!(out.stack_temp_c > 20.0, "a dragging brake heats up even with no pedal command: {}", out.stack_temp_c);
    }

    #[test]
    fn sustained_extreme_heat_eventually_starts_a_brake_fire() {
        let mut w = BrakeWheel::new(15.0);
        let inputs = BrakeWheelInputs { commanded: 1.0, on_ground: true, normal_load_n: 150_000.0, groundspeed_ms: 80.0, ambient_c: 15.0, dt_s: 5.0 };
        let mut out = BrakeWheelOutputs::default();
        for _ in 0..400 {
            out = w.step(&inputs, &healthy());
            if out.fire {
                break;
            }
        }
        assert!(out.fire, "sustained extreme heat must eventually start a brake fire; reached {} C", out.stack_temp_c);
    }

    #[test]
    fn numerically_safe_at_rest() {
        let mut w = BrakeWheel::new(15.0);
        let inputs = BrakeWheelInputs { commanded: 0.0, on_ground: false, normal_load_n: 0.0, groundspeed_ms: 0.0, ambient_c: 15.0, dt_s: 0.0 };
        let out = w.step(&inputs, &healthy());
        assert!(out.stack_temp_c.is_finite());
        assert!(!out.applied_fraction.is_nan());
        assert!(out.wheel_speed_ms.is_finite());
    }

    #[test]
    fn the_parking_brake_holds_when_healthy_and_bleeds_down_when_leaking() {
        let mut acc = ParkingBrakeAccumulator::new();
        let (p0, holding0) = acc.step(true, &ParkingBrakeFaults::default(), 1.0);
        assert!(holding0, "a freshly applied, healthy accumulator must hold");
        assert!(p0 < PRECHARGE_PA, "applying it must have discharged some fluid, dropping pressure from the bare precharge");

        let mut leaking = ParkingBrakeAccumulator::new();
        leaking.step(true, &ParkingBrakeFaults::default(), 1.0);
        let faults = ParkingBrakeFaults { leak: 1.0 };
        let mut holding = true;
        let mut pressure = PRECHARGE_PA;
        for _ in 0..700 {
            let (p, h) = leaking.step(true, &faults, 60.0);
            pressure = p;
            holding = h;
            if !holding {
                break;
            }
        }
        assert!(!holding, "a full-magnitude leak must eventually bleed the accumulator below its holding minimum; ended at {pressure} Pa");
    }

    #[test]
    fn releasing_the_parking_brake_stops_it_reporting_as_holding() {
        let mut acc = ParkingBrakeAccumulator::new();
        acc.step(true, &ParkingBrakeFaults::default(), 1.0);
        let (_, holding) = acc.step(false, &ParkingBrakeFaults::default(), 1.0);
        assert!(!holding);
    }
}
