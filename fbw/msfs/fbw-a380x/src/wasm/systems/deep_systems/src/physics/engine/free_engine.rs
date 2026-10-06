use super::gas::CP_AIR;
use super::gas_path::{self, GasPath};
use super::governor::Governor;
use super::hot_section::HotSection;
use super::inlet;
use super::params::{
    inertia, COMBUSTOR_EFFICIENCY, LHV_JET_A1_J_KG, MECH_EFFICIENCY, MIN_N3_FOR_COMBUSTION_PCT, N1_DESIGN_RPM, N2_DESIGN_RPM, N3_DESIGN_RPM, P_REF_PA, T_REF_K,
};
use super::spool::Spool;
use super::starter;

pub const OUTER_STEP_S: f64 = 0.005;
const MAX_OUTER_STEPS: usize = 400;
const SPOOL_SUBSTEP_S: f64 = 0.005;
const EGT_PROBE_TAU_S: f64 = 1.5;
const LEAN_BLOWOUT_FAR: f64 = 0.0032;
const DECEL_MIN_FAR: f64 = 0.0045;
const ACCEL_TT4_SCHEDULE: [(f64, f64); 6] = [(0.0, 1100.0), (0.6, 1250.0), (0.8, 1500.0), (0.95, 1720.0), (1.03, 1900.0), (1.2, 1900.0)];
const N1_LEAD_S: f64 = 2.5;
const N1_RATE_FILTER_S: f64 = 0.15;
const BEARING_DRAG_FRACTION: f64 = 0.002;
const STATIC_DRAG_FRACTION: f64 = 0.0006;
const SETTLE_S: f64 = 3.0;
const FLAME_HOLD_S: f64 = 0.2;

#[derive(Clone, Copy, Debug)]
pub struct Inputs {
    pub ambient_pressure_pa: f64,
    pub ambient_temp_k: f64,
    pub mach: f64,
    pub true_airspeed_m_s: f64,
    pub n1_target_pct: f64,
    pub fuel_available: bool,
    pub ignition: bool,
    pub starter_supply_fraction: f64,
    pub ip_bleed_kg_s: f64,
    pub hp_bleed_kg_s: f64,
    pub hp_accessory_load_w: f64,
    pub hpc_efficiency: f64,
    pub hpc_flow_capacity: f64,
    pub turbine_efficiency: f64,
    pub hp_drag_torque_n_m: f64,
    pub fuel_metered_kg_s: Option<f64>,
    pub starter_torque_n_m: Option<f64>,
    pub dt_s: f64,
}

impl Inputs {
    pub fn at(ambient_pressure_pa: f64, ambient_temp_k: f64, mach: f64, true_airspeed_m_s: f64, dt_s: f64) -> Self {
        Self {
            ambient_pressure_pa,
            ambient_temp_k,
            mach,
            true_airspeed_m_s,
            n1_target_pct: 0.0,
            fuel_available: false,
            ignition: false,
            starter_supply_fraction: 0.0,
            ip_bleed_kg_s: 0.0,
            hp_bleed_kg_s: 0.0,
            hp_accessory_load_w: 0.0,
            hpc_efficiency: 1.0,
            hpc_flow_capacity: 1.0,
            turbine_efficiency: 1.0,
            hp_drag_torque_n_m: 0.0,
            fuel_metered_kg_s: None,
            starter_torque_n_m: None,
            dt_s,
        }
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub struct Outputs {
    pub n1_pct: f64,
    pub n2_pct: f64,
    pub n3_pct: f64,
    pub wf_kg_s: f64,
    pub wf_demand_kg_s: f64,
    pub egt_c: f64,
    pub tt4_k: f64,
    pub tt3_k: f64,
    pub pt3_pa: f64,
    pub net_thrust_n: f64,
    pub core_flow_kg_s: f64,
    pub fan_flow_kg_s: f64,
    pub bypass_flow_kg_s: f64,
    pub fan_pressure_ratio: f64,
    pub hpc_stall_margin: f64,
    pub lit: bool,
    pub surge: bool,
}

#[derive(Clone, Debug)]
pub struct FreeEngine {
    gas: GasPath,
    lp: Spool,
    ip: Spool,
    hp: Spool,
    governor: Governor,
    hot: HotSection,
    egt_c: f64,
    wf_kg_s: f64,
    n1_rate_pct_s: f64,
    lit: bool,
    starved_s: f64,
    soaked: bool,
    design_power_w: [f64; 3],
    last: Outputs,
}

impl Default for FreeEngine {
    fn default() -> Self {
        Self::new()
    }
}

impl FreeEngine {
    pub fn new() -> Self {
        let gas = GasPath::new();
        let d = &gas.design;
        let design_out = gas.clone().step(
            &gas_path::Inputs {
                ambient_pressure_pa: P_REF_PA,
                ambient_temp_k: T_REF_K,
                mach: 0.0,
                true_airspeed_m_s: 0.0,
                wf_kg_s: d.wf_kg_s,
                lp_rpm: N1_DESIGN_RPM,
                ip_rpm: N2_DESIGN_RPM,
                hp_rpm: N3_DESIGN_RPM,
                ip_bleed_kg_s: 0.0,
                hp_bleed_kg_s: 0.0,
                hpc_efficiency: 1.0,
                hpc_flow_capacity: 1.0,
                turbine_efficiency: 1.0,
            },
            0.001,
        );
        let design_power_w = [design_out.fan_power_w.max(1.0), design_out.ipc_power_w.max(1.0), design_out.hpc_power_w.max(1.0)];
        Self {
            gas,
            lp: Spool::new(inertia::i_lp()),
            ip: Spool::new(inertia::i_ip()),
            hp: Spool::new(inertia::i_hp()),
            governor: Governor::new(),
            hot: HotSection::new(T_REF_K),
            egt_c: T_REF_K - 273.15,
            wf_kg_s: 0.0,
            n1_rate_pct_s: 0.0,
            lit: false,
            starved_s: 0.0,
            soaked: false,
            design_power_w,
            last: Outputs::default(),
        }
    }

    pub fn design_wf_kg_s(&self) -> f64 {
        self.gas.design.wf_kg_s
    }

    pub fn hp_design_torque_n_m(&self) -> f64 {
        self.design_power_w[2] / gas_path::omega(N3_DESIGN_RPM)
    }

    pub fn fuel_demand_kg_s(&self) -> f64 {
        self.wf_kg_s
    }

    pub fn is_lit(&self) -> bool {
        self.lit
    }

    fn soak(&mut self, ambient_pa: f64, ambient_k: f64) {
        if self.soaked {
            return;
        }
        self.soaked = true;
        self.hot = HotSection::new(ambient_k);
        self.egt_c = ambient_k - 273.15;
        self.gas.rest(ambient_pa);
    }

    pub fn settle_running(&mut self, i: &Inputs, n1_pct: f64, n2_pct: f64, n3_pct: f64) {
        self.soak(i.ambient_pressure_pa, i.ambient_temp_k);
        self.lp.rpm = N1_DESIGN_RPM * n1_pct.max(0.0) / 100.0;
        self.ip.rpm = N2_DESIGN_RPM * n2_pct.max(0.0) / 100.0;
        self.hp.rpm = N3_DESIGN_RPM * n3_pct.max(0.0) / 100.0;
        self.lit = true;
        let steps = (SETTLE_S / OUTER_STEP_S) as usize;
        let held = Inputs { n1_target_pct: n1_pct, fuel_available: true, ..*i };
        for _ in 0..steps {
            let out = self.gas.step(&self.gas_inputs(&held), OUTER_STEP_S);
            let air = (out.mdot_gas_kg_s - self.wf_kg_s).max(0.0);
            self.wf_kg_s = self.govern(&held, air, out.tt3_k, OUTER_STEP_S);
            let ex = self.hot.step(out.tt4_k, out.tt45_k, out.mdot_gas_kg_s, self.gas.design.mdot_core_kg_s, held.ambient_temp_k, OUTER_STEP_S);
            self.egt_c = ex.probe_target_k - 273.15;
        }
        let theta = Self::sqrt_theta(&held);
        self.governor.track(self.wf_kg_s, self.n1_pct() / theta, self.design_wf_kg_s());
    }

    fn sqrt_theta(i: &Inputs) -> f64 {
        (inlet::station2(i.ambient_pressure_pa, i.ambient_temp_k, i.mach).tt_k / T_REF_K).max(1e-6).sqrt()
    }

    fn n1_pct(&self) -> f64 {
        100.0 * self.lp.rpm / N1_DESIGN_RPM
    }

    fn n2_pct(&self) -> f64 {
        100.0 * self.ip.rpm / N2_DESIGN_RPM
    }

    fn n3_pct(&self) -> f64 {
        100.0 * self.hp.rpm / N3_DESIGN_RPM
    }

    fn supply_kg_s(&self, i: &Inputs) -> f64 {
        i.fuel_metered_kg_s.unwrap_or(self.wf_kg_s).max(0.0)
    }

    fn burn_kg_s(&self, i: &Inputs) -> f64 {
        if self.lit {
            self.supply_kg_s(i)
        } else {
            0.0
        }
    }

    fn gas_inputs(&self, i: &Inputs) -> gas_path::Inputs {
        gas_path::Inputs {
            ambient_pressure_pa: i.ambient_pressure_pa.max(1.0),
            ambient_temp_k: i.ambient_temp_k.max(1.0),
            mach: i.mach.max(0.0),
            true_airspeed_m_s: i.true_airspeed_m_s.max(0.0),
            wf_kg_s: self.burn_kg_s(i),
            lp_rpm: self.lp.rpm,
            ip_rpm: self.ip.rpm,
            hp_rpm: self.hp.rpm,
            ip_bleed_kg_s: i.ip_bleed_kg_s.max(0.0),
            hp_bleed_kg_s: i.hp_bleed_kg_s.max(0.0),
            hpc_efficiency: i.hpc_efficiency.clamp(0.05, 1.0),
            hpc_flow_capacity: i.hpc_flow_capacity.clamp(0.05, 1.0),
            turbine_efficiency: i.turbine_efficiency.clamp(0.05, 1.0),
        }
    }

    fn far_for_tt4(tt4_k: f64, tt3_k: f64) -> f64 {
        let heat = LHV_JET_A1_J_KG * COMBUSTOR_EFFICIENCY - CP_AIR * tt4_k;
        if heat <= 0.0 {
            return 0.0;
        }
        (CP_AIR * (tt4_k - tt3_k) / heat).max(0.0)
    }

    fn accel_tt4_limit_k(n3_corrected_frac: f64) -> f64 {
        let table = &ACCEL_TT4_SCHEDULE;
        let x = n3_corrected_frac.clamp(table[0].0, table[table.len() - 1].0);
        for w in table.windows(2) {
            if x <= w[1].0 {
                let f = (x - w[0].0) / (w[1].0 - w[0].0);
                return w[0].1 + f * (w[1].1 - w[0].1);
            }
        }
        table[table.len() - 1].1
    }

    fn govern(&mut self, i: &Inputs, core_air_kg_s: f64, tt3_k: f64, h: f64) -> f64 {
        let theta = Self::sqrt_theta(i);
        let n1_c = self.n1_pct() / theta;
        let n3_c = self.n3_pct() / theta;
        let target_c = i.n1_target_pct.max(0.0) / theta;
        let air = core_air_kg_s.max(0.0);
        let led_target_c = target_c - N1_LEAD_S * self.n1_rate_pct_s / theta;
        let wf = self
            .governor
            .step(led_target_c, n1_c, n3_c, i.fuel_available, air, self.design_wf_kg_s(), h)
            .min(air * Self::far_for_tt4(Self::accel_tt4_limit_k(n3_c / 100.0), tt3_k));
        if self.lit && i.fuel_available && Governor::combustion_floor_met(n1_c, n3_c) {
            wf.max(air * DECEL_MIN_FAR)
        } else {
            wf
        }
    }

    fn bearing_drag_n_m(spool: &Spool, design_power_w: f64, design_rpm: f64) -> f64 {
        let omega_design = design_rpm * std::f64::consts::PI / 30.0;
        let design_torque = design_power_w / omega_design;
        if spool.rpm <= 0.0 {
            return 0.0;
        }
        BEARING_DRAG_FRACTION * design_torque * (spool.rpm / design_rpm) + STATIC_DRAG_FRACTION * design_torque
    }

    pub fn step(&mut self, i: &Inputs) -> Outputs {
        let dt = i.dt_s;
        if !dt.is_finite() || dt <= 0.0 {
            return self.last;
        }
        self.soak(i.ambient_pressure_pa, i.ambient_temp_k);
        let n = ((dt / OUTER_STEP_S).ceil() as usize).clamp(1, MAX_OUTER_STEPS);
        let h = dt / n as f64;
        let [design_fan_w, design_ip_w, design_hp_w] = self.design_power_w;
        let mut out = gas_path::Outputs::default();
        let mut thrust = 0.0;
        for _ in 0..n {
            out = self.gas.step(&self.gas_inputs(i), h);
            thrust += out.net_thrust_n;

            let starter_torque = i.starter_torque_n_m.unwrap_or_else(|| starter::torque_n_m(self.hp.rpm, i.starter_supply_fraction));
            let hp_torque = self.hp.torque_from_power(out.hpt_power_w * MECH_EFFICIENCY - out.hpc_power_w - i.hp_accessory_load_w.max(0.0)) + starter_torque
                - i.hp_drag_torque_n_m.max(0.0)
                - Self::bearing_drag_n_m(&self.hp, design_hp_w, N3_DESIGN_RPM);
            let ip_torque = self.ip.torque_from_power(out.ipt_power_w * MECH_EFFICIENCY - out.ipc_power_w) - Self::bearing_drag_n_m(&self.ip, design_ip_w, N2_DESIGN_RPM);
            let lp_torque = self.lp.torque_from_power(out.lpt_power_w * MECH_EFFICIENCY - out.fan_power_w) - Self::bearing_drag_n_m(&self.lp, design_fan_w, N1_DESIGN_RPM);
            let n1_before = self.n1_pct();
            self.hp.integrate(hp_torque, h, SPOOL_SUBSTEP_S, 4);
            self.ip.integrate(ip_torque, h, SPOOL_SUBSTEP_S, 4);
            self.lp.integrate(lp_torque, h, SPOOL_SUBSTEP_S, 4);
            let rate = (self.n1_pct() - n1_before) / h;
            self.n1_rate_pct_s += (rate - self.n1_rate_pct_s) * (1.0 - (-h / N1_RATE_FILTER_S).exp());

            let air = (out.mdot_gas_kg_s - self.burn_kg_s(i)).max(0.0);
            self.wf_kg_s = self.govern(i, air, out.tt3_k, h);
            let supply = self.supply_kg_s(i);
            if !self.lit {
                if i.fuel_available && i.ignition && self.n3_pct() >= MIN_N3_FOR_COMBUSTION_PCT && supply > 0.0 {
                    self.lit = true;
                }
            } else if !i.fuel_available || supply <= 0.0 || air <= 0.0 || supply / air.max(1e-6) < LEAN_BLOWOUT_FAR {
                self.starved_s += h;
                if self.starved_s >= FLAME_HOLD_S {
                    self.lit = false;
                }
            } else {
                self.starved_s = 0.0;
            }
            if !self.lit {
                self.starved_s = 0.0;
            }

            let ex = self.hot.step(out.tt4_k, out.tt45_k, out.mdot_gas_kg_s, self.gas.design.mdot_core_kg_s, i.ambient_temp_k, h);
            let tau = (1.0 - (-h / EGT_PROBE_TAU_S).exp()).clamp(0.0, 1.0);
            self.egt_c += (ex.probe_target_k - 273.15 - self.egt_c) * tau;
        }
        self.last = Outputs {
            n1_pct: self.n1_pct(),
            n2_pct: self.n2_pct(),
            n3_pct: self.n3_pct(),
            wf_kg_s: self.burn_kg_s(i),
            wf_demand_kg_s: self.wf_kg_s,
            egt_c: self.egt_c,
            tt4_k: out.tt4_k,
            tt3_k: out.tt3_k,
            pt3_pa: out.pt3_pa,
            net_thrust_n: thrust / n as f64,
            core_flow_kg_s: out.m_core,
            fan_flow_kg_s: out.m_fan,
            bypass_flow_kg_s: out.m_bypass,
            fan_pressure_ratio: self.gas.state.p13 / inlet::station2(i.ambient_pressure_pa, i.ambient_temp_k, i.mach).pt_pa.max(1.0),
            hpc_stall_margin: out.hpc_stall_margin,
            lit: self.lit,
            surge: out.hpc_stall_margin < 1.0 || out.ipc_stall_margin < 1.0 || out.fan_stall_margin < 1.0,
        };
        self.last
    }
}
