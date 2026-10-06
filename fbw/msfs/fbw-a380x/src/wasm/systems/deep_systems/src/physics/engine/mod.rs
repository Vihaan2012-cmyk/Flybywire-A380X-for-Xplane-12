pub mod bleed_limits;
pub mod combustor;
pub mod free_engine;
pub mod compressor;
pub mod gas;
pub mod gas_path;
pub mod governor;
pub mod hot_section;
pub mod inlet;
pub mod matching;
pub mod nozzle;
pub mod oil;
pub mod params;
pub mod spool;
pub mod starter;
pub mod turbine;

use params::{MECH_EFFICIENCY, N1_DESIGN_RPM, N2_DESIGN_RPM, N3_DESIGN_RPM};

#[derive(Clone, Copy, Debug)]
pub struct ShadowInputs {
    pub ambient_pressure_pa: f64,
    pub ambient_temp_k: f64,
    pub mach: f64,
    pub n1_pct: f64,
    pub n2_pct: f64,
    pub n3_pct: f64,
    pub wf_kg_s: f64,
    pub bleed_extraction_kg_s: f64,
    pub bleed_from_ip_port: bool,
    pub compressor_efficiency_loss_fraction: f64,
    pub compressor_flow_capacity_loss_fraction: f64,
    pub turbine_efficiency_loss_fraction: f64,
    pub oil_pressure_fraction: f64,
    pub oil_faults: oil::OilFaults,
    pub fuel_temp_k: f64,
    pub dt_s: f64,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct ShadowOutputs {
    pub egt_delta_c: f64,
    pub oil_temp_delta_c: f64,
    pub oil_press_delta_psi: f64,
    pub oil_quantity_delta_fraction: f64,
    pub stall_margin_loss_pct: f64,
    pub n2_capability_loss_pct: f64,
    pub n3_capability_loss_pct: f64,
    pub thrust_loss_pct: f64,
    pub surge: bool,

    pub oil_temp_c: f64,
    pub oil_press_psi: f64,
    pub oil_quantity_fraction: f64,
    pub oil_supply_c: f64,
    pub oil_filter_bypassed: bool,
    pub oil_relief_open: bool,
    pub fuel_heat_w: f64,
    pub fuel_out_c: f64,
    pub hot_section_soak_c: f64,
    pub egt_shadow_c: f64,
    pub ip_port_pressure_pa: f64,
    pub ip_port_temp_k: f64,
    pub combustor_rise_fraction: f64,
}

#[derive(Clone)]
pub struct ShadowEngine {
    healthy: gas_path::GasPath,
    actual: gas_path::GasPath,
    hot_healthy: hot_section::HotSection,
    hot_actual: hot_section::HotSection,
    oil_healthy: oil::OilSystem,
    oil_actual: oil::OilSystem,
    egt_lag_healthy_c: f64,
    egt_lag_actual_c: f64,
    soaked: bool,
}

const EGT_PROBE_TAU_S: f64 = 1.5;

impl ShadowEngine {
    pub fn new() -> Self {
        Self {
            healthy: gas_path::GasPath::new(),
            actual: gas_path::GasPath::new(),
            hot_healthy: hot_section::HotSection::new(params::T_REF_K),
            hot_actual: hot_section::HotSection::new(params::T_REF_K),
            oil_healthy: oil::OilSystem::new(params::T_REF_K),
            oil_actual: oil::OilSystem::new(params::T_REF_K),
            egt_lag_healthy_c: params::T_REF_K - 273.15,
            egt_lag_actual_c: params::T_REF_K - 273.15,
            soaked: false,
        }
    }

    pub fn step(&mut self, i: &ShadowInputs) -> ShadowOutputs {
        let ambient_p = i.ambient_pressure_pa.max(1.0);
        let ambient_t = i.ambient_temp_k.max(1.0);
        let dt = i.dt_s.max(0.0);

        if !self.soaked {
            self.soaked = true;
            self.hot_healthy = hot_section::HotSection::new(ambient_t);
            self.hot_actual = hot_section::HotSection::new(ambient_t);
            self.oil_healthy = oil::OilSystem::new(ambient_t);
            self.oil_actual = oil::OilSystem::new(ambient_t);
            self.egt_lag_healthy_c = ambient_t - 273.15;
            self.egt_lag_actual_c = ambient_t - 273.15;
            if i.n1_pct <= 0.05 && i.n2_pct <= 0.05 && i.n3_pct <= 0.05 {
                self.healthy.rest(ambient_p);
                self.actual.rest(ambient_p);
            }
        }

        let lp_rpm = N1_DESIGN_RPM * i.n1_pct.max(0.0) / 100.0;
        let ip_rpm = N2_DESIGN_RPM * i.n2_pct.max(0.0) / 100.0;
        let hp_rpm = N3_DESIGN_RPM * i.n3_pct.max(0.0) / 100.0;
        let bleed = i.bleed_extraction_kg_s.max(0.0);
        let (ip_bleed, hp_bleed) = if i.bleed_from_ip_port { (bleed, 0.0) } else { (0.0, bleed) };
        let wf = i.wf_kg_s.max(0.0);
        let mach = i.mach.max(0.0);

        let base = gas_path::Inputs {
            ambient_pressure_pa: ambient_p,
            ambient_temp_k: ambient_t,
            mach,
            true_airspeed_m_s: 0.0,
            wf_kg_s: wf,
            lp_rpm,
            ip_rpm,
            hp_rpm,
            ip_bleed_kg_s: ip_bleed,
            hp_bleed_kg_s: hp_bleed,
            hpc_efficiency: 1.0,
            hpc_flow_capacity: 1.0,
            turbine_efficiency: 1.0,
        };
        let actual_in = gas_path::Inputs {
            hpc_efficiency: (1.0 - i.compressor_efficiency_loss_fraction.clamp(0.0, 1.0)).max(0.05),
            hpc_flow_capacity: (1.0 - i.compressor_flow_capacity_loss_fraction.clamp(0.0, 1.0)).max(0.05),
            turbine_efficiency: (1.0 - i.turbine_efficiency_loss_fraction.clamp(0.0, 1.0)).max(0.05),
            ..base
        };

        let perturbed = i.compressor_efficiency_loss_fraction != 0.0
            || i.compressor_flow_capacity_loss_fraction != 0.0
            || i.turbine_efficiency_loss_fraction != 0.0
            || i.oil_pressure_fraction != 1.0
            || i.oil_faults.filter_clog != 0.0
            || i.oil_faults.leak != 0.0;

        let gp_h = self.healthy.step(&base, dt);
        let gp_a = if perturbed {
            self.actual.step(&actual_in, dt)
        } else {
            self.actual.state = self.healthy.state;
            gp_h
        };

        let ex_h = self.hot_healthy.step(gp_h.tt4_k, gp_h.tt45_k, gp_h.mdot_gas_kg_s, self.healthy.design.mdot_core_kg_s, ambient_t, dt);
        let ex_a = if perturbed {
            self.hot_actual.step(gp_a.tt4_k, gp_a.tt45_k, gp_a.mdot_gas_kg_s, self.actual.design.mdot_core_kg_s, ambient_t, dt)
        } else {
            self.hot_actual = self.hot_healthy;
            ex_h
        };
        let tau = (1.0 - (-dt / EGT_PROBE_TAU_S).exp()).clamp(0.0, 1.0);
        self.egt_lag_healthy_c += (ex_h.probe_target_k - 273.15 - self.egt_lag_healthy_c) * tau;
        self.egt_lag_actual_c += (ex_a.probe_target_k - 273.15 - self.egt_lag_actual_c) * tau;

        let friction_h = (gp_h.hpc_power_w + gp_h.ipc_power_w + gp_h.fan_power_w) * (1.0 - MECH_EFFICIENCY);
        let friction_a = (gp_a.hpc_power_w + gp_a.ipc_power_w + gp_a.fan_power_w) * (1.0 - MECH_EFFICIENCY);
        let surround_h = oil::Surroundings {
            n3_frac: i.n3_pct.max(0.0) / 100.0,
            pump_fraction: 1.0,
            friction_w: friction_h,
            front_air_k: gp_h.tt25_k,
            hot_metal_k: self.hot_healthy.metal_k(),
            exhaust_k: gp_h.tt5_k,
            fuel_kg_s: wf,
            fuel_k: i.fuel_temp_k,
            bypass_kg_s: gp_h.m_bypass,
            fan_air_k: gp_h.tt13_k,
            nacelle_k: ambient_t,
            dt_s: dt,
        };
        let surround_a = oil::Surroundings { pump_fraction: i.oil_pressure_fraction.clamp(0.0, 1.0), friction_w: friction_a, front_air_k: gp_a.tt25_k, hot_metal_k: self.hot_actual.metal_k(), exhaust_k: gp_a.tt5_k, bypass_kg_s: gp_a.m_bypass, fan_air_k: gp_a.tt13_k, ..surround_h };
        let oil_h = self.oil_healthy.step(&surround_h, &oil::OilFaults::default());
        let oil_a = if perturbed {
            self.oil_actual.step(&surround_a, &i.oil_faults)
        } else {
            self.oil_actual = self.oil_healthy;
            oil_h
        };

        let worst = |o: &gas_path::Outputs| o.fan_stall_margin.min(o.ipc_stall_margin).min(o.hpc_stall_margin);
        let margin_h = worst(&gp_h);
        let margin_a = worst(&gp_a);
        let pct_loss = |healthy: f64, actual: f64| { if healthy.abs() > 1.0 { (100.0 * (healthy - actual) / healthy.abs()).max(0.0) } else { 0.0 } };

        ShadowOutputs {
            egt_delta_c: self.egt_lag_actual_c - self.egt_lag_healthy_c,
            oil_temp_delta_c: (oil_a.temp_k - oil_h.temp_k),
            oil_press_delta_psi: (oil_a.pressure_psi - oil_h.pressure_psi),
            oil_quantity_delta_fraction: (self.oil_actual.quantity_fraction() - self.oil_healthy.quantity_fraction()),
            stall_margin_loss_pct: ((margin_h - margin_a) * 100.0).max(0.0),
            n2_capability_loss_pct: pct_loss(gp_h.ipt_power_w, gp_a.ipt_power_w),
            n3_capability_loss_pct: pct_loss(gp_h.hpt_power_w, gp_a.hpt_power_w),
            thrust_loss_pct: pct_loss(gp_h.net_thrust_n, gp_a.net_thrust_n),
            surge: margin_a < 1.0 && margin_h >= 1.0,

            oil_temp_c: oil_a.temp_k - 273.15,
            oil_press_psi: oil_a.pressure_psi,
            oil_quantity_fraction: self.oil_actual.quantity_fraction(),
            oil_supply_c: oil_a.supply_k - 273.15,
            oil_filter_bypassed: oil_a.filter_bypassed,
            oil_relief_open: oil_a.relief_open,
            fuel_heat_w: oil_a.fuel_heat_w,
            fuel_out_c: oil_a.fuel_out_k - 273.15,
            hot_section_soak_c: self.hot_actual.metal_k() - 273.15,
            egt_shadow_c: self.egt_lag_actual_c,
            ip_port_pressure_pa: self.actual.state.p25,
            ip_port_temp_k: gp_a.tt25_k,
            combustor_rise_fraction: if gp_a.tt4_k > gp_a.tt3_k && gp_a.tt4_k > 1.0 {
                (1.0 - gp_a.tt3_k / gp_a.tt4_k).clamp(0.0, 1.0)
            } else {
                0.0
            },
        }
    }
}

impl Default for ShadowEngine {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn healthy_running_inputs(n1: f64, n2: f64, n3: f64, wf: f64) -> ShadowInputs {
        ShadowInputs {
            ambient_pressure_pa: params::P_REF_PA,
            ambient_temp_k: params::T_REF_K,
            mach: 0.0,
            n1_pct: n1,
            n2_pct: n2,
            n3_pct: n3,
            wf_kg_s: wf,
            bleed_extraction_kg_s: 0.0,
            bleed_from_ip_port: false,
            compressor_efficiency_loss_fraction: 0.0,
            compressor_flow_capacity_loss_fraction: 0.0,
            turbine_efficiency_loss_fraction: 0.0,
            oil_pressure_fraction: 1.0,
            oil_faults: oil::OilFaults::default(),
            fuel_temp_k: params::T_REF_K,
            dt_s: 0.1,
        }
    }

    #[test]
    fn a_healthy_engine_publishes_exactly_zero_deltas() {
        let mut e = ShadowEngine::new();
        let mut out = ShadowOutputs::default();
        for _ in 0..50 {
            out = e.step(&healthy_running_inputs(20.0, 60.0, 70.0, 1.0));
        }
        assert_eq!(out.egt_delta_c, 0.0);
        assert_eq!(out.oil_temp_delta_c, 0.0);
        assert_eq!(out.oil_press_delta_psi, 0.0);
        assert_eq!(out.n3_capability_loss_pct, 0.0);
        assert!(!out.surge);
    }

    #[test]
    fn a_hp_compressor_efficiency_loss_moves_egt_and_n3_deltas_with_magnitude() {
        let mut small = ShadowEngine::new();
        let mut large = ShadowEngine::new();
        let mut small_out = ShadowOutputs::default();
        let mut large_out = ShadowOutputs::default();
        for _ in 0..50 {
            let mut i = healthy_running_inputs(20.0, 60.0, 70.0, 1.0);
            i.compressor_efficiency_loss_fraction = 0.2;
            small_out = small.step(&i);
        }
        for _ in 0..50 {
            let mut i = healthy_running_inputs(20.0, 60.0, 70.0, 1.0);
            i.compressor_efficiency_loss_fraction = 0.6;
            large_out = large.step(&i);
        }
        assert!(small_out.egt_delta_c > 0.0, "{}", small_out.egt_delta_c);
        assert!(large_out.egt_delta_c > small_out.egt_delta_c, "small={} large={}", small_out.egt_delta_c, large_out.egt_delta_c);
        assert!(large_out.n3_capability_loss_pct > small_out.n3_capability_loss_pct);
    }
}
