const AIR_SPECIFIC_GAS_CONSTANT: f64 = 287.058;

pub const WATER_DENSITY_KG_L: f64 = 1.0;
pub const WATER_SPECIFIC_HEAT_J_KGK: f64 = 4186.0;
pub const ICE_LATENT_HEAT_J_KG: f64 = 334_000.0;
pub const PSI_TO_PA: f64 = 6894.757;

pub const TANK_CAPACITY_L: f64 = 800.0;
const ULLAGE_MIN_FRACTION: f64 = 0.05;
const TANK_VOLUME_L: f64 = TANK_CAPACITY_L / (1.0 - ULLAGE_MIN_FRACTION);

pub const TARGET_GAUGE_PA: f64 = 40.0 * PSI_TO_PA;
const RELIEF_MARGIN_PA: f64 = 10.0 * PSI_TO_PA;
const MIN_USEFUL_PRESSURE_FRACTION: f64 = 0.15;

const BLEED_MAX_AIR_KG_S: f64 = 0.02;
const COMPRESSOR_MAX_AIR_KG_S: f64 = 0.003;

const LEAK_MAX_L_S: f64 = 0.05;

pub const N_DRAIN_MASTS: usize = 2;
pub const N_HEATERS: usize = super::Zone::COUNT;
pub const N_SHOWERS: usize = 2;

pub const SHOWER_SESSION_L: f64 = 10.0;
pub const SHOWER_DURATION_S: f64 = 300.0;
pub const SHOWER_HEATER_W: f64 = 3000.0;

const HEATER_RATED_W: f64 = 500.0;
const HEATER_WATER_KG: f64 = 1.5;
const HEATER_SETPOINT_C: f64 = 50.0;
const HEATER_BAND_C: f64 = 5.0;
const HEATER_LOSS_W_K: f64 = 1.5;

const MAST_AREA_M2: f64 = 0.05;
const MAST_H0_W_M2K: f64 = 10.0;
const MAST_H_PER_TAS_W_M2K: f64 = 0.6;
const ISA_SEA_LEVEL_DENSITY_KG_M3: f64 = 1.225;
const MAST_H_DENSITY_EXPONENT: f64 = 0.8;
const MAST_CAPACITY_J_K: f64 = 500.0;
const MAST_HEATER_RATED_W: f64 = 250.0;
const MAST_HEATER_ON_OAT_C: f64 = 10.0;
const MAST_RESIDUAL_KG_S: f64 = 0.001;
const MAST_FULL_FREEZE_SPAN_C: f64 = 10.0;
pub const MAST_BLOCKAGE_KG: f64 = 0.3;
const MAST_MELT_KG_S_C: f64 = 0.0005;

fn air_pressure_pa(air_kg: f64, volume_m3: f64, temp_k: f64) -> f64 {
    if volume_m3 <= 0. || temp_k <= 0. {
        return 0.;
    }
    air_kg.max(0.) * AIR_SPECIFIC_GAS_CONSTANT * temp_k / volume_m3
}

fn air_mass_kg(pressure_pa: f64, volume_m3: f64, temp_k: f64) -> f64 {
    if temp_k <= 0. {
        return 0.;
    }
    (pressure_pa.max(0.) * volume_m3 / (AIR_SPECIFIC_GAS_CONSTANT * temp_k)).max(0.)
}

#[derive(Clone, Copy, Debug, Default)]
pub struct WaterFaults {
    pub leak: f64,
    pub bleed_valve_fault: f64,
    pub compressor_fault: f64,
    pub heater_fault: [f64; N_HEATERS],
    pub mast_heater_fault: [f64; N_DRAIN_MASTS],
    pub quantity_sensor_fault: f64,
}

#[derive(Clone, Copy, Debug)]
pub struct WaterInputs {
    pub bleed_available: bool,
    pub compressor_commanded: bool,
    pub cabin_pressure_pa: f64,
    pub cabin_temp_k: f64,
    pub oat_c: f64,
    pub tas_mps: f64,
    pub ambient_pressure_pa: f64,
    pub galley_demand_l_s: f64,
    pub lav_demand_l_s: f64,
    pub shower_requests: usize,
    pub heater_commanded: [bool; N_HEATERS],
}

impl Default for WaterInputs {
    fn default() -> Self {
        Self {
            bleed_available: true,
            compressor_commanded: false,
            cabin_pressure_pa: 101_325.0,
            cabin_temp_k: 297.0,
            oat_c: 15.0,
            tas_mps: 0.0,
            ambient_pressure_pa: 101_325.0,
            galley_demand_l_s: 0.0,
            lav_demand_l_s: 0.0,
            shower_requests: 0,
            heater_commanded: [false; N_HEATERS],
        }
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub struct WaterOutputs {
    pub quantity_percent: f64,
    pub gauge_pressure_pa: f64,
    pub flow_fraction: f64,
    pub leak_l_s: f64,
    pub mast_ice_kg: [f64; N_DRAIN_MASTS],
    pub mast_blocked: [bool; N_DRAIN_MASTS],
    pub heater_temp_c: [f64; N_HEATERS],
    pub heater_power_w: [f64; N_HEATERS],
    pub shower_active: [bool; N_SHOWERS],
    pub shower_heater_power_w: f64,
    pub water_mass_kg: f64,
    pub tank_empty: bool,
}

#[derive(Clone, Copy, Debug)]
struct Heater {
    temp_c: f64,
}

#[derive(Clone, Copy, Debug)]
struct Mast {
    temp_c: f64,
    ice_kg: f64,
}

#[derive(Clone, Copy, Debug)]
struct Shower {
    remaining_s: f64,
}

pub struct WaterSystem {
    water_l: f64,
    ullage_air_kg: f64,
    displayed_quantity_percent: f64,
    heaters: [Heater; N_HEATERS],
    masts: [Mast; N_DRAIN_MASTS],
    showers: [Shower; N_SHOWERS],
    showers_fitted: bool,
}

impl WaterSystem {
    pub fn new(showers_fitted: bool) -> Self {
        let cabin_temp_k = 297.0;
        let ullage_volume_m3 = (TANK_VOLUME_L - TANK_CAPACITY_L) / 1000.0;
        let full_gauge_target_pa = TARGET_GAUGE_PA;
        let ullage_air_kg = air_mass_kg(101_325.0 + full_gauge_target_pa, ullage_volume_m3, cabin_temp_k);
        Self {
            water_l: TANK_CAPACITY_L,
            ullage_air_kg,
            displayed_quantity_percent: 100.0,
            heaters: [Heater { temp_c: 20.0 }; N_HEATERS],
            masts: [Mast { temp_c: 20.0, ice_kg: 0.0 }; N_DRAIN_MASTS],
            showers: [Shower { remaining_s: 0.0 }; N_SHOWERS],
            showers_fitted,
        }
    }

    pub fn service(&mut self) {
        self.water_l = TANK_CAPACITY_L;
        for mast in &mut self.masts {
            mast.ice_kg = 0.0;
        }
    }

    pub fn step(&mut self, inputs: &WaterInputs, faults: &WaterFaults, dt: f64) -> WaterOutputs {
        let dt = dt.max(0.0);

        let ullage_volume_l = (TANK_VOLUME_L - self.water_l).max(TANK_VOLUME_L * ULLAGE_MIN_FRACTION);
        let ullage_volume_m3 = ullage_volume_l / 1000.0;
        let target_abs_pa = inputs.cabin_pressure_pa + TARGET_GAUGE_PA;
        let relief_abs_pa = inputs.cabin_pressure_pa + TARGET_GAUGE_PA + RELIEF_MARGIN_PA;

        let source_kg_s = if inputs.bleed_available && faults.bleed_valve_fault < 1.0 {
            BLEED_MAX_AIR_KG_S * (1.0 - faults.bleed_valve_fault)
        } else if inputs.compressor_commanded && faults.compressor_fault < 1.0 {
            COMPRESSOR_MAX_AIR_KG_S * (1.0 - faults.compressor_fault)
        } else {
            0.0
        };
        let current_abs_pa = air_pressure_pa(self.ullage_air_kg, ullage_volume_m3, inputs.cabin_temp_k);
        if current_abs_pa < target_abs_pa {
            let needed_kg = air_mass_kg(target_abs_pa, ullage_volume_m3, inputs.cabin_temp_k) - self.ullage_air_kg;
            self.ullage_air_kg += needed_kg.max(0.0).min(source_kg_s * dt);
        } else if current_abs_pa > relief_abs_pa {
            let excess_kg = self.ullage_air_kg - air_mass_kg(relief_abs_pa, ullage_volume_m3, inputs.cabin_temp_k);
            self.ullage_air_kg -= excess_kg.max(0.0).min(self.ullage_air_kg);
        }
        let gauge_pa = (air_pressure_pa(self.ullage_air_kg, ullage_volume_m3, inputs.cabin_temp_k) - inputs.cabin_pressure_pa).max(0.0);

        let pressure_ratio = (gauge_pa / TARGET_GAUGE_PA).clamp(0.0, 1.0);
        let flow_fraction = if pressure_ratio < MIN_USEFUL_PRESSURE_FRACTION { 0.0 } else { pressure_ratio.sqrt() };

        let mut shower_flow_l_s = 0.0;
        let mut shower_heater_power_w = 0.0;
        let mut requests_left = if self.showers_fitted { inputs.shower_requests } else { 0 };
        for shower in &mut self.showers {
            if shower.remaining_s <= 0.0 && requests_left > 0 {
                shower.remaining_s = SHOWER_DURATION_S;
                requests_left -= 1;
            }
            if shower.remaining_s > 0.0 {
                shower_flow_l_s += SHOWER_SESSION_L / SHOWER_DURATION_S;
                shower_heater_power_w += SHOWER_HEATER_W;
                shower.remaining_s = (shower.remaining_s - dt).max(0.0);
            }
        }

        let requested_flow_l_s = inputs.galley_demand_l_s + inputs.lav_demand_l_s + shower_flow_l_s;
        let actual_flow_l_s = requested_flow_l_s * flow_fraction;
        let leak_l_s = faults.leak.clamp(0.0, 1.0) * LEAK_MAX_L_S * flow_fraction.max(pressure_ratio);
        self.water_l = (self.water_l - (actual_flow_l_s + leak_l_s) * dt).max(0.0);
        let tank_empty = self.water_l <= 0.0;

        let mut heater_temp_c = [0.0; N_HEATERS];
        let mut heater_power_w = [0.0; N_HEATERS];
        for i in 0..N_HEATERS {
            let heater = &mut self.heaters[i];
            let on = inputs.heater_commanded[i] && heater.temp_c < HEATER_SETPOINT_C + HEATER_BAND_C && faults.heater_fault[i] < 1.0;
            let power_w = if on { HEATER_RATED_W * (1.0 - faults.heater_fault[i]) } else { 0.0 };
            let capacity_j_k = HEATER_WATER_KG * WATER_SPECIFIC_HEAT_J_KGK;
            let ambient_c = inputs.cabin_temp_k - 273.15;
            let target_c = ambient_c + power_w / HEATER_LOSS_W_K;
            let tau_s = (capacity_j_k / HEATER_LOSS_W_K).max(1e-6);
            heater.temp_c += (target_c - heater.temp_c) * (1.0 - (-dt / tau_s).exp());
            heater_temp_c[i] = heater.temp_c;
            heater_power_w[i] = power_w;
        }

        let mut mast_ice_kg = [0.0; N_DRAIN_MASTS];
        let mut mast_blocked = [false; N_DRAIN_MASTS];
        let air_temp_k = (inputs.oat_c + 273.15).max(1.0);
        let local_density_kg_m3 = inputs.ambient_pressure_pa.max(0.0) / (AIR_SPECIFIC_GAS_CONSTANT * air_temp_k);
        let density_ratio = (local_density_kg_m3 / ISA_SEA_LEVEL_DENSITY_KG_M3).clamp(0.0, 1.0);
        let h = MAST_H0_W_M2K + MAST_H_PER_TAS_W_M2K * density_ratio.powf(MAST_H_DENSITY_EXPONENT) * inputs.tas_mps.max(0.0);
        let ha = h * MAST_AREA_M2;
        for i in 0..N_DRAIN_MASTS {
            let mast = &mut self.masts[i];
            let heater_on = inputs.oat_c < MAST_HEATER_ON_OAT_C && faults.mast_heater_fault[i] < 1.0;
            let power_w = if heater_on { MAST_HEATER_RATED_W * (1.0 - faults.mast_heater_fault[i]) } else { 0.0 };
            let target_c = inputs.oat_c + power_w / ha.max(1e-9);
            let tau_s = (MAST_CAPACITY_J_K / ha.max(1e-9)).max(1e-6);
            mast.temp_c += (target_c - mast.temp_c) * (1.0 - (-dt / tau_s).exp());

            if mast.temp_c < 0.0 {
                let sub_cool = (-mast.temp_c).min(MAST_FULL_FREEZE_SPAN_C);
                let freeze_fraction = sub_cool / MAST_FULL_FREEZE_SPAN_C;
                mast.ice_kg += MAST_RESIDUAL_KG_S * freeze_fraction * dt;
            } else {
                let melt = (MAST_MELT_KG_S_C * mast.temp_c * dt).min(mast.ice_kg);
                mast.ice_kg -= melt;
            }
            mast_ice_kg[i] = mast.ice_kg;
            mast_blocked[i] = mast.ice_kg >= MAST_BLOCKAGE_KG;
        }

        let real_percent = 100.0 * self.water_l / TANK_CAPACITY_L.max(1e-9);
        if faults.quantity_sensor_fault < 0.5 {
            self.displayed_quantity_percent = real_percent;
        }

        WaterOutputs {
            quantity_percent: self.displayed_quantity_percent,
            gauge_pressure_pa: gauge_pa,
            flow_fraction,
            leak_l_s,
            mast_ice_kg,
            mast_blocked,
            heater_temp_c,
            heater_power_w,
            shower_active: std::array::from_fn(|i| self.showers[i].remaining_s > 0.0),
            shower_heater_power_w,
            water_mass_kg: self.water_l * WATER_DENSITY_KG_L,
            tank_empty,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn healthy_inputs() -> WaterInputs {
        WaterInputs::default()
    }

    #[test]
    fn tank_drains_by_exactly_the_delivered_flow_conserving_mass() {
        let mut w = WaterSystem::new(false);
        let mut inputs = healthy_inputs();
        inputs.galley_demand_l_s = 0.1;
        for _ in 0..600 {
            w.step(&inputs, &WaterFaults::default(), 1.0 / 60.0);
        }
        let before = w.water_l;
        let out = w.step(&inputs, &WaterFaults::default(), 1.0);
        let delivered = inputs.galley_demand_l_s * out.flow_fraction;
        assert!((before - w.water_l - delivered).abs() < 1e-6, "before={before} after={} delivered={delivered}", w.water_l);
        assert!(!out.tank_empty);
    }

    #[test]
    fn a_leak_drains_the_tank_with_no_demand_and_a_healthy_leak_does_not() {
        let mut healthy = WaterSystem::new(false);
        let mut leaking = WaterSystem::new(false);
        let inputs = healthy_inputs();
        for _ in 0..600 {
            healthy.step(&inputs, &WaterFaults::default(), 1.0 / 60.0);
            leaking.step(&inputs, &WaterFaults { leak: 1.0, ..Default::default() }, 1.0 / 60.0);
        }
        assert_eq!(healthy.water_l, TANK_CAPACITY_L, "no demand, no leak: full");
        assert!(leaking.water_l < TANK_CAPACITY_L, "a full leak fault should drain the tank");
    }

    #[test]
    fn ullage_pressure_follows_the_ideal_gas_law_and_settles_near_target() {
        let mut w = WaterSystem::new(false);
        let inputs = healthy_inputs();
        let mut out = WaterOutputs::default();
        for _ in 0..1200 {
            out = w.step(&inputs, &WaterFaults::default(), 1.0 / 60.0);
        }
        assert!((out.gauge_pressure_pa - TARGET_GAUGE_PA).abs() / TARGET_GAUGE_PA < 0.05, "{}", out.gauge_pressure_pa);
    }

    #[test]
    fn a_dead_bleed_valve_falls_back_to_the_slower_compressor() {
        let mut w = WaterSystem::new(false);
        let mut inputs = healthy_inputs();
        inputs.bleed_available = false;
        inputs.compressor_commanded = true;
        let mut out = WaterOutputs::default();
        for _ in 0..3600 {
            out = w.step(&inputs, &WaterFaults::default(), 1.0 / 60.0);
        }
        assert!(out.gauge_pressure_pa > TARGET_GAUGE_PA * 0.5, "compressor alone should still pressurise, slowly: {}", out.gauge_pressure_pa);
    }

    #[test]
    fn with_neither_source_pressure_cannot_be_maintained_and_flow_fails() {
        let mut w = WaterSystem::new(false);
        let mut inputs = healthy_inputs();
        inputs.bleed_available = false;
        inputs.compressor_commanded = false;
        inputs.galley_demand_l_s = 0.05;

        let mut out = WaterOutputs::default();
        for _ in 0..3600 {
            out = w.step(&inputs, &WaterFaults::default(), 1.0 / 60.0);
        }
        assert!(out.flow_fraction > 0.9, "one minute in, the stored air is barely touched: {}", out.flow_fraction);

        let mut failed_at_s = None;
        let mut water_at_failure_l = 0.0;
        for step in 0..14_400 {
            out = w.step(&inputs, &WaterFaults::default(), 0.25);
            if out.flow_fraction == 0.0 {
                failed_at_s = Some((step + 1) as f64 * 0.25);
                water_at_failure_l = w.water_l;
                break;
            }
        }
        assert!(failed_at_s.is_some(), "with no air source the taps must eventually die");
        assert!((water_at_failure_l - 730.8).abs() < 1.0, "{water_at_failure_l} L left when flow failed");
        assert!(out.gauge_pressure_pa < TARGET_GAUGE_PA * MIN_USEFUL_PRESSURE_FRACTION);
    }

    #[test]
    fn a_cold_drain_mast_freezes_without_its_heater_and_stays_clear_with_it() {
        let mut cold_no_heat = WaterSystem::new(false);
        let mut cold_heated = WaterSystem::new(false);
        let mut inputs = healthy_inputs();
        inputs.oat_c = -56.0;
        for _ in 0..36000 {
            cold_no_heat.step(&inputs, &WaterFaults { mast_heater_fault: [1.0; N_DRAIN_MASTS], ..Default::default() }, 1.0 / 60.0);
            cold_heated.step(&inputs, &WaterFaults::default(), 1.0 / 60.0);
        }
        assert!(cold_no_heat.masts[0].ice_kg > 0.0, "a failed mast heater at cruise OAT should let ice accumulate");
        assert!(cold_heated.masts[0].temp_c > 0.0, "a healthy mast heater should hold the mast above freezing");
        assert_eq!(cold_heated.masts[0].ice_kg, 0.0);
    }

    #[test]
    fn ice_above_the_blockage_threshold_is_reported_blocked() {
        let mut w = WaterSystem::new(false);
        w.masts[0].ice_kg = MAST_BLOCKAGE_KG + 0.01;
        let out = w.step(&healthy_inputs(), &WaterFaults::default(), 0.001);
        assert!(out.mast_blocked[0]);
        assert!(!out.mast_blocked[1]);
    }

    #[test]
    fn showers_are_off_when_not_fitted_even_if_requested() {
        let mut w = WaterSystem::new(false);
        let mut inputs = healthy_inputs();
        inputs.shower_requests = 2;
        let out = w.step(&inputs, &WaterFaults::default(), 1.0);
        assert!(out.shower_active.iter().all(|&a| !a));
        assert_eq!(out.shower_heater_power_w, 0.0);
    }

    #[test]
    fn a_requested_shower_runs_for_its_full_session_and_uses_its_published_volume() {
        let mut w = WaterSystem::new(true);
        let mut inputs = healthy_inputs();
        inputs.shower_requests = 1;
        let mut total_l = 0.0;
        let dt = 1.0;
        for _ in 0..(SHOWER_DURATION_S as usize + 5) {
            let before = w.water_l;
            let out = w.step(&inputs, &WaterFaults::default(), dt);
            inputs.shower_requests = 0;
            if out.shower_active[0] {
                total_l += (before - w.water_l).max(0.0);
            }
        }
        assert!((total_l - SHOWER_SESSION_L).abs() < 1.0, "{total_l}");
    }

    #[test]
    fn a_stuck_quantity_sensor_freezes_its_last_reading() {
        let mut w = WaterSystem::new(false);
        let mut inputs = healthy_inputs();
        inputs.galley_demand_l_s = 0.5;
        let mut faults = WaterFaults::default();
        for _ in 0..600 {
            w.step(&inputs, &faults, 1.0 / 60.0);
        }
        faults.quantity_sensor_fault = 1.0;
        let stuck_at = w.displayed_quantity_percent;
        for _ in 0..600 {
            w.step(&inputs, &faults, 1.0 / 60.0);
        }
        assert_eq!(w.displayed_quantity_percent, stuck_at);
        assert!(w.water_l / TANK_CAPACITY_L * 100.0 < stuck_at, "the real quantity should have kept dropping");
    }

    #[test]
    fn service_refills_the_tank_and_clears_mast_ice() {
        let mut w = WaterSystem::new(false);
        w.water_l = 10.0;
        w.masts[0].ice_kg = 1.0;
        w.service();
        assert_eq!(w.water_l, TANK_CAPACITY_L);
        assert_eq!(w.masts[0].ice_kg, 0.0);
    }

    #[test]
    fn no_nan_at_rest_or_dt_zero() {
        let mut w = WaterSystem::new(true);
        let inputs = healthy_inputs();
        let out = w.step(&inputs, &WaterFaults::default(), 0.0);
        assert!(!out.gauge_pressure_pa.is_nan());
        assert!(!out.water_mass_kg.is_nan());
        for t in out.heater_temp_c {
            assert!(!t.is_nan());
        }
        for t in out.mast_ice_kg {
            assert!(!t.is_nan());
        }
    }
}
