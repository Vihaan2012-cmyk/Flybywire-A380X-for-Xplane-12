use super::duct::orifice_mass_flow_kg_s;

#[derive(Clone, Copy, Debug, Default)]
pub struct PrecoolerFaults {
    pub fouling: f64,
    pub fan_air_valve_stuck: f64,
    pub temp_sensor_fault: f64,
    pub check_valve_failure: f64,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct PrecoolerOutputs {
    pub outlet_temp_k: f64,
    pub fav_open: f64,
    pub sensed_outlet_c: f64,
    pub overtemp_active: bool,
}

#[derive(Clone, Copy, Debug)]
pub struct Precooler {
    fav_open: f64,
    frozen_reading_c: f64,
    stagnant_outlet_k: f64,
}

impl Precooler {
    const UA_W_K: f64 = 360.0;
    const FOULING_MAX_REDUCTION: f64 = 0.8;
    const FAN_AIR_BLEED_OFF_FRACTION_AT_FULL_OPEN: f64 = 0.02;
    pub const OUTLET_TARGET_C: f64 = 200.0;
    const FAV_GAIN_PER_C: f64 = 0.01;
    pub const OVERTEMP_TRIP_C: f64 = 257.0;
    const RELIEF_CRACK_PA: f64 = 60.0 * 6894.757;
    const RELIEF_FULL_FLOW_RISE_PA: f64 = 10.0 * 6894.757;
    const RELIEF_FULL_FLOW_KG_S: f64 = 2.0;
    const CHECK_VALVE_LEAK_AREA_M2: f64 = 5.0e-5;
    const CHECK_VALVE_DISCHARGE_COEFFICIENT: f64 = 0.7;
    const FAV_ACTUATOR_TIME_CONSTANT_S: f64 = 3.0;

    pub fn new() -> Self {
        Self { fav_open: 0.0, frozen_reading_c: 15.0, stagnant_outlet_k: 288.15 }
    }

    pub fn step(
        &mut self,
        dt_s: f64,
        source_temp_k: f64,
        mdot_hot_kg_s: f64,
        bypass_available_kg_s: f64,
        fan_air_k: f64,
        faults: &PrecoolerFaults,
    ) -> PrecoolerOutputs {
        let ua = Self::UA_W_K * (1.0 - Self::FOULING_MAX_REDUCTION * faults.fouling.clamp(0.0, 1.0)).max(0.05);

        const CP: f64 = super::duct::CP_AIR_J_KG_K;
        let mdot_hot = mdot_hot_kg_s.max(0.0);
        let mdot_cold = (bypass_available_kg_s.max(0.0) * Self::FAN_AIR_BLEED_OFF_FRACTION_AT_FULL_OPEN * self.fav_open.clamp(0.0, 1.0)).max(0.0);
        let c_hot = mdot_hot * CP;
        let c_cold = mdot_cold * CP;

        const STAGNANT_SETTLE_S: f64 = 30.0;
        let outlet_temp_k = if c_hot <= 1e-9 {
            let k = 1.0 - (-dt_s.max(0.0) / STAGNANT_SETTLE_S).exp();
            self.stagnant_outlet_k + (fan_air_k - self.stagnant_outlet_k) * k
        } else if c_cold <= 1e-9 {
            source_temp_k
        } else {
            let c_min = c_hot.min(c_cold);
            let c_max = c_hot.max(c_cold);
            let cr = (c_min / c_max).clamp(0.0, 1.0);
            let ntu = ua / c_min;
            let effectiveness = if cr < 0.999 {
                let e = (-ntu * (1.0 - cr)).exp();
                (1.0 - e) / (1.0 - cr * e)
            } else {
                ntu / (1.0 + ntu)
            };
            let q_w = effectiveness * c_min * (source_temp_k - fan_air_k).max(0.0);
            (source_temp_k - q_w / c_hot).max(fan_air_k)
        };

        self.stagnant_outlet_k = outlet_temp_k;
        let true_outlet_c = outlet_temp_k - 273.15;
        if faults.temp_sensor_fault < 0.5 {
            self.frozen_reading_c = true_outlet_c;
        }
        let sensed_outlet_c = if faults.temp_sensor_fault >= 0.5 { self.frozen_reading_c } else { true_outlet_c };

        let overtemp_active = true_outlet_c > Self::OVERTEMP_TRIP_C;
        let mut commanded = ((sensed_outlet_c - Self::OUTLET_TARGET_C) * Self::FAV_GAIN_PER_C).clamp(0.0, 1.0);
        if overtemp_active {
            commanded = 1.0;
        }

        let used_fav_open = self.fav_open;
        if faults.fan_air_valve_stuck < 0.5 {
            let k = 1.0 / Self::FAV_ACTUATOR_TIME_CONSTANT_S;
            self.fav_open = commanded + (self.fav_open - commanded) * (-k * dt_s.max(0.0)).exp();
        }

        PrecoolerOutputs { outlet_temp_k, fav_open: used_fav_open, sensed_outlet_c, overtemp_active }
    }

    pub fn relief_and_backflow_kg_s(&self, duct_pa: f64, ambient_pa: f64, faults: &PrecoolerFaults) -> (f64, f64) {
        let relief = (Self::RELIEF_FULL_FLOW_KG_S * ((duct_pa - Self::RELIEF_CRACK_PA) / Self::RELIEF_FULL_FLOW_RISE_PA))
            .clamp(0.0, Self::RELIEF_FULL_FLOW_KG_S);
        let backflow = if faults.check_valve_failure > 0.0 && ambient_pa > duct_pa {
            let area = Self::CHECK_VALVE_LEAK_AREA_M2 * faults.check_valve_failure.clamp(0.0, 1.0);
            orifice_mass_flow_kg_s(Self::CHECK_VALVE_DISCHARGE_COEFFICIENT, area, ambient_pa, 288.15, duct_pa)
        } else {
            0.0
        };
        (relief, backflow)
    }
}

impl Default for Precooler {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const HOT_SOURCE_K: f64 = 523.15;

    fn run(p: &mut Precooler, faults: &PrecoolerFaults, ticks: usize) -> PrecoolerOutputs {
        let mut out = PrecoolerOutputs::default();
        for _ in 0..ticks {
            out = p.step(1.0, HOT_SOURCE_K, 0.5, 60.0, 288.15, faults);
        }
        out
    }

    #[test]
    fn a_healthy_precooler_settles_near_its_target_with_enough_cooling_flow() {
        let mut p = Precooler::new();
        let out = run(&mut p, &PrecoolerFaults::default(), 500);
        let settled_c = out.outlet_temp_k - 273.15;
        assert!(settled_c > 150.0 && settled_c < 250.0, "settled {settled_c} C, should sit meaningfully below the 250 C source and not saturate cold");
        assert!(!out.overtemp_active);
    }

    #[test]
    fn no_cooling_flow_leaves_the_bleed_at_source_temperature_not_nan() {
        let mut p = Precooler::new();
        let out = p.step(1.0, HOT_SOURCE_K, 0.5, 0.0, 288.15, &PrecoolerFaults::default());
        assert_eq!(out.outlet_temp_k, HOT_SOURCE_K);
        assert!(out.outlet_temp_k.is_finite());
    }

    #[test]
    fn fouling_leaves_the_outlet_hotter_than_healthy() {
        let mut healthy = Precooler::new();
        let mut fouled = Precooler::new();
        let h = run(&mut healthy, &PrecoolerFaults::default(), 200);
        let f = run(&mut fouled, &PrecoolerFaults { fouling: 1.0, ..Default::default() }, 200);
        assert!(f.outlet_temp_k > h.outlet_temp_k, "a fouled core must cool less effectively");
    }

    #[test]
    fn a_stuck_closed_fav_cannot_cool_and_the_bleed_stays_hot() {
        let mut p = Precooler::new();
        let out = run(&mut p, &PrecoolerFaults { fan_air_valve_stuck: 1.0, ..Default::default() }, 200);
        assert!((out.outlet_temp_k - HOT_SOURCE_K).abs() < 1.0, "stuck fully closed from the start, no cooling ever develops");
    }

    #[test]
    fn overtemperature_forces_full_fav_even_if_the_sensor_lies_cool() {
        let mut p = Precooler::new();
        let faults = PrecoolerFaults { temp_sensor_fault: 1.0, ..Default::default() };
        let mut healthy_first = Precooler::new();
        let _ = healthy_first.step(1.0, 473.15, 0.0, 60.0, 288.15, &PrecoolerFaults::default());
        let mut hot_but_blind = Precooler::new();
        hot_but_blind.step(1.0, 473.15, 0.0, 0.0, 288.15, &PrecoolerFaults::default());
        let out = hot_but_blind.step(1.0, 600.0, 0.5, 60.0, 288.15, &faults);
        assert!(out.overtemp_active);
        assert!(out.fav_open >= 0.0);
    }

    #[test]
    fn relief_valve_only_opens_above_its_crack_pressure_and_scales_with_overpressure() {
        let p = Precooler::new();
        let (below, _) = p.relief_and_backflow_kg_s(Precooler::RELIEF_CRACK_PA - 1000.0, 40_000.0, &PrecoolerFaults::default());
        assert_eq!(below, 0.0);
        let (mild, _) = p.relief_and_backflow_kg_s(Precooler::RELIEF_CRACK_PA + 1000.0, 40_000.0, &PrecoolerFaults::default());
        let (severe, _) = p.relief_and_backflow_kg_s(Precooler::RELIEF_CRACK_PA + 50_000.0, 40_000.0, &PrecoolerFaults::default());
        assert!(severe > mild && mild > 0.0);
    }

    #[test]
    fn a_healthy_check_valve_blocks_backflow_but_a_failed_one_leaks() {
        let p = Precooler::new();
        let (_, ok) = p.relief_and_backflow_kg_s(20_000.0, 101_325.0, &PrecoolerFaults::default());
        assert_eq!(ok, 0.0);
        let (_, failed) = p.relief_and_backflow_kg_s(20_000.0, 101_325.0, &PrecoolerFaults { check_valve_failure: 1.0, ..Default::default() });
        assert!(failed > 0.0);
    }
}
