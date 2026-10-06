use super::combustor;
use super::gas::{CP_AIR, CP_GAS, GAMMA_AIR, GAMMA_GAS, R_AIR, R_GAS};
use super::nozzle;
use super::params::*;

const PHI_DESIGN: f64 = 0.5;
const X_DESIGN: f64 = 1.4;
const SHUTOFF_OVER_H: f64 = 0.7;
const ETA_FALLOFF: f64 = 2.5;
const ETA_SPEED_FALLOFF: f64 = 0.60;

const VSV_MIN: f64 = 0.80;
const VSV_OPEN_SPEED: f64 = 1.00;
const VSV_SHUT_SPEED: f64 = 0.70;

fn vsv_setting(corrected_speed_frac: f64) -> f64 {
    let t = ((corrected_speed_frac - VSV_SHUT_SPEED) / (VSV_OPEN_SPEED - VSV_SHUT_SPEED)).clamp(0.0, 1.0);
    VSV_MIN + (1.0 - VSV_MIN) * t
}

const VSV_FRONT_FRACTION: f64 = 0.9;

fn vsv_authority(index: usize, n_stages: usize) -> f64 {
    if n_stages == 0 {
        return 0.0;
    }
    let span = n_stages as f64 * VSV_FRONT_FRACTION;
    (1.0 - index as f64 / span.max(1e-9)).clamp(0.0, 1.0)
}
const K_THROTTLE: f64 = 0.5;
const K_REVERSE: f64 = 1.0;
const ETA_REVERSE: f64 = 0.3;

const X_FIT_MAX: f64 = 2.0;

fn cubic_bracket(x: f64) -> f64 {
    1.0 + 1.5 * x - 0.5 * x * x * x
}

fn cubic_slope(x: f64) -> f64 {
    1.5 - 1.5 * x * x
}

const CHOKE_FLOW_FN_AIR: f64 = 0.040_414_9;

const CX_CHOKE_PER_SQRT_K: f64 = CHOKE_FLOW_FN_AIR * R_AIR;

fn shaft_work(dh_s: f64, eta: f64) -> f64 {
    if dh_s > 0.0 {
        dh_s / eta
    } else {
        dh_s * eta
    }
}

#[derive(Clone, Copy, Debug)]
struct Stage {
    radius_m: f64,
    area_m2: f64,
    h: f64,
    w: f64,
    psi0: f64,
    eta_design: f64,
    corrected_u_design: f64,
    vsv_authority: f64,
}

impl Stage {
    fn corrected_speed(&self, omega: f64, tt: f64) -> f64 {
        (omega * self.radius_m).max(0.0) / (tt.max(1.0).sqrt() * self.corrected_u_design.max(1e-9))
    }

    fn vane_setting(&self, speed: f64) -> f64 {
        1.0 - self.vsv_authority * (1.0 - vsv_setting(speed))
    }

    fn w_eff(&self, speed: f64) -> f64 {
        self.w * self.vane_setting(speed)
    }

    fn axial_velocity(&self, mdot: f64, tt: f64, pt: f64, area_scale: f64) -> f64 {
        let rho = pt.max(1.0) / (R_AIR * tt.max(1.0));
        let cx_choke = CX_CHOKE_PER_SQRT_K * tt.max(1.0).sqrt();
        (mdot / (rho * self.area_m2 * area_scale).max(1e-9)).clamp(-cx_choke, cx_choke)
    }

    fn work(&self, mdot: f64, omega: f64, tt: f64, pt: f64, area_scale: f64, eta_scale: f64) -> (f64, f64) {
        let u = (omega * self.radius_m).max(0.0);
        let vane = self.vane_setting(self.corrected_speed(omega, tt));
        let cx = self.axial_velocity(mdot, tt, pt, area_scale * vane);
        if mdot < 0.0 {
            let dh_s = self.psi0 * u * u + K_REVERSE * cx * cx;
            let dh0 = self.psi0 * u * u / ETA_REVERSE;
            return (dh_s, dh0);
        }
        let w = self.w_eff(self.corrected_speed(omega, tt));
        let phi = cx / u.max(1e-6);
        let speed = self.corrected_speed(omega, tt);
        let island = (1.0 - ETA_FALLOFF * (phi / PHI_DESIGN - 1.0).powi(2)) * (1.0 - ETA_SPEED_FALLOFF * (1.0 - speed).powi(2));
        let eta = (self.eta_design * island).clamp(0.05, self.eta_design);
        let eta = (eta * eta_scale).max(0.02);
        let cx_fit = w * (1.0 + X_FIT_MAX) * u;
        if cx_fit > 0.0 && cx < cx_fit {
            let dh_s = (self.psi0 + self.h * cubic_bracket(phi / w - 1.0)) * u * u;
            (dh_s, shaft_work(dh_s, eta))
        } else {
            let over = cx - cx_fit;
            let dh_work = self.psi0 * u * u + self.h * cubic_slope(X_FIT_MAX) * (u / w) * over;
            (dh_work - K_THROTTLE * over * over, shaft_work(dh_work, eta))
        }
    }
}

#[derive(Clone, Debug)]
pub struct Compressor {
    stages: Vec<Stage>,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct Compression {
    pub tt_out_k: f64,
    pub pt_out_pa: f64,
    pub power_w: f64,
    pub stall_margin: f64,
    pub bleed_kg_s: f64,
}

#[derive(Clone, Copy, Debug)]
pub struct HandlingBleed {
    pub after_stage: usize,
    pub area_m2: f64,
    pub sink_pa: f64,
}

impl Compressor {
    fn design(n: usize, radius_m: f64, mdot: f64, omega: f64, tt: f64, pt: f64, pr: f64, eta: f64, has_vsv: bool) -> Self {
        let k = (GAMMA_AIR - 1.0) / GAMMA_AIR;
        let dh0_total = CP_AIR * tt * (pr.powf(k) - 1.0) / eta;
        let dh0 = dh0_total / n as f64;
        let u = omega * radius_m;
        let build = |eta_st: f64| {
            let mut stages = Vec::with_capacity(n);
            let (mut t, mut p) = (tt, pt);
            for index in 0..n {
                let dh_s = eta_st * dh0;
                let psi_d = dh_s / (u * u);
                let h = psi_d / (SHUTOFF_OVER_H + cubic_bracket(X_DESIGN));
                let w = PHI_DESIGN / (1.0 + X_DESIGN);
                let rho = p / (R_AIR * t);
                let area = mdot / (rho * PHI_DESIGN * u);
                stages.push(Stage {
                    radius_m,
                    area_m2: area,
                    h,
                    w,
                    psi0: SHUTOFF_OVER_H * h,
                    eta_design: eta_st,
                    corrected_u_design: u / t.sqrt(),
                    vsv_authority: if has_vsv { vsv_authority(index, n) } else { 0.0 },
                });
                p *= (1.0 + dh_s / (CP_AIR * t)).powf(1.0 / k);
                t += dh0 / CP_AIR;
            }
            (stages, p / pt)
        };
        let (mut lo, mut hi) = (eta * 0.8, 0.999);
        for _ in 0..60 {
            let mid = 0.5 * (lo + hi);
            if build(mid).1 < pr {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        Self { stages: build(0.5 * (lo + hi)).0 }
    }

    pub fn choke_kg_s(&self, tt: f64, pt: f64, flow_capacity: f64) -> f64 {
        match self.stages.first() {
            Some(s) => CHOKE_FLOW_FN_AIR * s.area_m2 * flow_capacity.max(0.0) * pt.max(0.0) / tt.max(1.0).sqrt(),
            None => 0.0,
        }
    }

    pub fn compress(&self, mdot: f64, omega: f64, tt: f64, pt: f64, flow_capacity: f64, efficiency: f64) -> Compression {
        self.compress_bled(mdot, omega, tt, pt, flow_capacity, efficiency, None)
    }

    pub fn compress_bled(&self, mdot: f64, omega: f64, tt: f64, pt: f64, flow_capacity: f64, efficiency: f64, bleed: Option<HandlingBleed>) -> Compression {
        let k = (GAMMA_AIR - 1.0) / GAMMA_AIR;
        let (mut t, mut p, mut power) = (tt, pt, 0.0);
        let mut margin = f64::INFINITY;
        let mut mdot = mdot;
        let mut bled = 0.0;
        let mut tap = |t: f64, p: f64, mdot: &mut f64, bled: &mut f64, index: usize| {
            if let Some(b) = bleed {
                if index == b.after_stage && b.area_m2 > 0.0 && *mdot > 0.0 {
                    *bled = nozzle::mass_flow_capacity(t, p, b.sink_pa, b.area_m2, GAMMA_AIR, R_AIR).min(*mdot);
                    *mdot -= *bled;
                }
            }
        };
        for (index, s) in self.stages.iter().enumerate() {
            tap(t, p, &mut mdot, &mut bled, index);
            let (dh_s, dh0) = s.work(mdot, omega, t, p, flow_capacity, efficiency);
            let u = omega * s.radius_m;
            if u > 1.0 && mdot > 0.0 {
                let vane = s.vane_setting(s.corrected_speed(omega, t));
                let phi = s.axial_velocity(mdot, t, p, flow_capacity * vane) / u;
                margin = margin.min(phi / (2.0 * s.w_eff(s.corrected_speed(omega, t))));
            }
            power += mdot.abs() * dh0;
            p *= (1.0 + dh_s / (CP_AIR * t.max(1.0))).max(1e-3).powf(1.0 / k);
            t = (t + dh0 / CP_AIR).max(1.0);
        }
        tap(t, p, &mut mdot, &mut bled, self.stages.len());
        Compression { tt_out_k: t, pt_out_pa: p, power_w: power, stall_margin: if margin.is_finite() { margin } else { 0.0 }, bleed_kg_s: bled }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct Turbine {
    flow_constant: f64,
    multistage: bool,
    eta_design: f64,
    radius_m: f64,
    nu_design: f64,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct Expansion {
    pub mdot_kg_s: f64,
    pub tt_out_k: f64,
    pub power_w: f64,
}

fn nozzle_flow_function(pr: f64) -> f64 {
    let g = GAMMA_GAS;
    let critical = (2.0 / (g + 1.0)).powf(g / (g - 1.0));
    let f = |r: f64| (2.0 / (g - 1.0) * (r.powf(2.0 / g) - r.powf((g + 1.0) / g))).max(0.0).sqrt();
    if pr <= critical {
        1.0
    } else if pr >= 1.0 {
        0.0
    } else {
        f(pr) / f(critical)
    }
}

impl Turbine {
    fn flow_parameter(&self, pr: f64) -> f64 {
        if self.multistage {
            (1.0 - pr * pr).max(0.0).sqrt()
        } else {
            nozzle_flow_function(pr)
        }
    }

    fn design(mdot: f64, tt: f64, pt: f64, pr: f64, eta: f64, radius_m: f64, omega: f64, multistage: bool) -> Self {
        let mut t = Self { flow_constant: 1.0, multistage, eta_design: eta, radius_m, nu_design: 1.0 };
        t.flow_constant = mdot * tt.sqrt() / (pt * t.flow_parameter(pr)).max(1e-9);
        let dh_is = CP_GAS * tt * (1.0 - pr.powf((GAMMA_GAS - 1.0) / GAMMA_GAS));
        t.nu_design = omega * radius_m / (2.0 * dh_is).max(1e-9).sqrt();
        t
    }

    pub fn expand(&self, pt_in: f64, tt_in: f64, pt_out: f64, omega: f64, efficiency: f64) -> Expansion {
        let pr = (pt_out / pt_in.max(1.0)).clamp(0.0, 1.0);
        let mdot = self.flow_constant * pt_in / tt_in.max(1.0).sqrt() * self.flow_parameter(pr);
        let dh_is = CP_GAS * tt_in * (1.0 - pr.powf((GAMMA_GAS - 1.0) / GAMMA_GAS));
        let c0 = (2.0 * dh_is).max(0.0).sqrt();
        let eta = if c0 > 1.0 {
            let nu = omega * self.radius_m / c0;
            (self.eta_design * (1.0 - (nu / self.nu_design - 1.0).powi(2))).clamp(0.05, self.eta_design)
        } else {
            0.05
        } * efficiency;
        let dh = eta * dh_is;
        Expansion { mdot_kg_s: mdot, tt_out_k: tt_in - dh / CP_GAS, power_w: mdot * dh }
    }
}

const FAN_MEAN_RADIUS_M: f64 = FAN_DIAMETER_M / 2.0 * 0.72;
const IPC_RADIUS_M: f64 = 0.45;
const HPC_RADIUS_M: f64 = 0.33;
const HPT_RADIUS_M: f64 = 0.40;
const IPT_RADIUS_M: f64 = 0.46;
const LPT_RADIUS_M: f64 = 0.72;
const FAN_HUB_FRACTION: f64 = 0.6;
const HP3_BLEED_AFTER_STAGE: usize = 3;
const HP3_BLEED_AREA_M2: f64 = 0.008;
const HP3_BLEED_SCHEDULE_PCT: (f64, f64) = (70.0, 85.0);
const IP_BLEED_AFTER_STAGE: usize = 8;
const IP_BLEED_AREA_M2: f64 = 0.05;
const IP_BLEED_SCHEDULE_PCT: (f64, f64) = (65.0, 80.0);

fn bleed_open(corrected_pct: f64, (open_below, shut_above): (f64, f64)) -> f64 {
    ((shut_above - corrected_pct) / (shut_above - open_below)).clamp(0.0, 1.0)
}

const V13: f64 = 8.0;
const V25: f64 = 0.25;
const V3: f64 = 0.25;
const V44: f64 = 0.08;
const V45: f64 = 0.25;
const V5: f64 = 1.2;
const A_OVER_L_FAN: f64 = 3.0;
const A_OVER_L_IPC: f64 = 0.35;
const A_OVER_L_HPC: f64 = 0.12;
pub const SUBSTEP_S: f64 = 0.001;
const MAX_SUBSTEPS: usize = 2000;

pub fn omega(rpm: f64) -> f64 {
    rpm * std::f64::consts::PI / 30.0
}

fn bypass_flow(tt13_k: f64, p13: f64, amb: f64, area_m2: f64, m_design: f64) -> (f64, f64) {
    let after = |m: f64| {
        let ratio = (m / m_design.max(1e-9)).powi(2);
        p13 * (1.0 - (BYPASS_DUCT_LOSS_FRAC * ratio).clamp(0.0, 0.5))
    };
    let capacity = |pt: f64| nozzle::mass_flow_capacity(tt13_k, pt, amb, area_m2, GAMMA_AIR, R_AIR);
    let pt = after(capacity(p13 * (1.0 - BYPASS_DUCT_LOSS_FRAC)));
    (capacity(pt), pt)
}

#[derive(Clone, Debug)]
pub struct Design {
    pub fan: Compressor,
    pub ipc: Compressor,
    pub hpc: Compressor,
    pub hpt: Turbine,
    pub ipt: Turbine,
    pub lpt: Turbine,
    pub core_nozzle_area_m2: f64,
    pub bypass_nozzle_area_m2: f64,
    pub wf_kg_s: f64,
    pub tt4_k: f64,
    pub mdot_core_kg_s: f64,
    pub mdot_bypass_kg_s: f64,
    pub opr: f64,
    pub thrust_n: f64,
    pub state: State,
}

#[derive(Clone, Copy, Debug)]
pub struct State {
    pub m_fan: f64,
    pub m_ipc: f64,
    pub m_hpc: f64,
    pub p13: f64,
    pub p25: f64,
    pub p3: f64,
    pub p44: f64,
    pub p45: f64,
    pub p5: f64,
}

const CRUISE_DESIGN_PRESSURE_PA: f64 = 23_842.0;
const CRUISE_DESIGN_TEMP_K: f64 = 218.81;
const CRUISE_DESIGN_MACH: f64 = 0.85;

fn design_at(tt4_k: f64) -> Option<Design> {
    let s2 = super::inlet::station2(P_REF_PA, T_REF_K, 0.0);
    let (p2, t2) = (s2.pt_pa, s2.tt_k);
    let m_total = MDOT_TOTAL_DESIGN_KG_S;
    let m_core = m_total / (1.0 + BYPASS_RATIO);
    let m_byp = m_total - m_core;
    let (w_lp, w_ip, w_hp) = (omega(N1_DESIGN_RPM), omega(N2_DESIGN_RPM), omega(N3_DESIGN_RPM));

    let fan = Compressor::design(1, FAN_MEAN_RADIUS_M, m_total, w_lp, t2, p2, PR_FAN_DESIGN, ETA_FAN_DESIGN, false);
    let f = fan.compress(m_total, w_lp, t2, p2, 1.0, 1.0);
    let (p21, t21) = (p2 + FAN_HUB_FRACTION * (f.pt_out_pa - p2), t2 + FAN_HUB_FRACTION * (f.tt_out_k - t2));
    let ipc = Compressor::design(8, IPC_RADIUS_M, m_core, w_ip, t21, p21, PR_IPC_DESIGN, ETA_IPC_DESIGN, true);
    let i = ipc.compress(m_core, w_ip, t21, p21, 1.0, 1.0);
    let hpc = Compressor::design(6, HPC_RADIUS_M, m_core, w_hp, i.tt_out_k, i.pt_out_pa, PR_HPC_DESIGN, ETA_HPC_DESIGN, true);
    let h = hpc.compress(m_core, w_hp, i.tt_out_k, i.pt_out_pa, 1.0, 1.0);

    let rise = tt4_k - h.tt_out_k;
    let wf = m_core * CP_AIR * rise / (LHV_JET_A1_J_KG * COMBUSTOR_EFFICIENCY - CP_AIR * rise);
    if !(wf > 0.0) {
        return None;
    }
    let c = combustor::burn(m_core, wf, h.tt_out_k, h.pt_out_pa);
    let m_gas = c.mdot_gas_kg_s;
    let kg = (GAMMA_GAS - 1.0) / GAMMA_GAS;
    let expand_for = |power: f64, tt: f64, pt: f64, eta: f64| -> Option<(f64, f64, f64)> {
        let dh = power / MECH_EFFICIENCY / m_gas;
        let base = 1.0 - dh / eta / (CP_GAS * tt);
        if base <= 0.0 {
            return None;
        }
        let pr = base.powf(1.0 / kg);
        Some((pr, tt - dh / CP_GAS, pt * pr))
    };
    let (pr_hpt, t44, p44) = expand_for(h.power_w, c.tt4_k, c.pt4_pa, ETA_HPT_DESIGN)?;
    let (pr_ipt, t45, p45) = expand_for(i.power_w, t44, p44, ETA_IPT_DESIGN)?;
    let (pr_lpt, t5, p5) = expand_for(f.power_w, t45, p45, ETA_LPT_DESIGN)?;
    if p5 <= P_REF_PA * 1.001 {
        return None;
    }
    let hpt = Turbine::design(m_gas, c.tt4_k, c.pt4_pa, pr_hpt, ETA_HPT_DESIGN, HPT_RADIUS_M, w_hp, false);
    let ipt = Turbine::design(m_gas, t44, p44, pr_ipt, ETA_IPT_DESIGN, IPT_RADIUS_M, w_ip, false);
    let lpt = Turbine::design(m_gas, t45, p45, pr_lpt, ETA_LPT_DESIGN, LPT_RADIUS_M, w_lp, true);
    let p13_byp = f.pt_out_pa * (1.0 - BYPASS_DUCT_LOSS_FRAC);
    let cruise = super::inlet::station2(CRUISE_DESIGN_PRESSURE_PA, CRUISE_DESIGN_TEMP_K, CRUISE_DESIGN_MACH);
    let (delta, theta) = (cruise.pt_pa / p2, cruise.tt_k / t2);
    let corrected = delta / theta.sqrt();
    let core_area = nozzle::design_area_m2(m_gas * corrected, t5 * theta, p5 * delta, CRUISE_DESIGN_PRESSURE_PA, GAMMA_GAS, R_GAS);
    let bypass_area = nozzle::design_area_m2(m_byp * corrected, f.tt_out_k * theta, p13_byp * delta, CRUISE_DESIGN_PRESSURE_PA, GAMMA_AIR, R_AIR);
    let thrust = nozzle::thrust(m_gas, t5, p5, P_REF_PA, 0.0, GAMMA_GAS, R_GAS).thrust_n
        + nozzle::thrust(m_byp, f.tt_out_k, p13_byp, P_REF_PA, 0.0, GAMMA_AIR, R_AIR).thrust_n;
    Some(Design {
        fan,
        ipc,
        hpc,
        hpt,
        ipt,
        lpt,
        core_nozzle_area_m2: core_area,
        bypass_nozzle_area_m2: bypass_area,
        wf_kg_s: wf,
        tt4_k: c.tt4_k,
        mdot_core_kg_s: m_core,
        mdot_bypass_kg_s: m_byp,
        opr: h.pt_out_pa / p2,
        thrust_n: thrust,
        state: State { m_fan: m_total, m_ipc: m_core, m_hpc: m_core, p13: f.pt_out_pa, p25: i.pt_out_pa, p3: h.pt_out_pa, p44, p45, p5 },
    })
}

pub fn design() -> Design {
    static DESIGN: std::sync::OnceLock<Design> = std::sync::OnceLock::new();
    DESIGN.get_or_init(solve_design).clone()
}

fn solve_design() -> Design {
    let (mut lo, mut hi) = (900.0, 2600.0);
    for _ in 0..80 {
        let mid = 0.5 * (lo + hi);
        match design_at(mid) {
            Some(d) if d.thrust_n >= STATIC_THRUST_N => hi = mid,
            _ => lo = mid,
        }
    }
    design_at(hi).expect("a design point exists between 900 and 2600 K")
}

#[derive(Clone, Copy, Debug)]
pub struct Inputs {
    pub ambient_pressure_pa: f64,
    pub ambient_temp_k: f64,
    pub mach: f64,
    pub true_airspeed_m_s: f64,
    pub wf_kg_s: f64,
    pub lp_rpm: f64,
    pub ip_rpm: f64,
    pub hp_rpm: f64,
    pub ip_bleed_kg_s: f64,
    pub hp_bleed_kg_s: f64,
    pub hpc_efficiency: f64,
    pub hpc_flow_capacity: f64,
    pub turbine_efficiency: f64,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct Outputs {
    pub fan_power_w: f64,
    pub ipc_power_w: f64,
    pub hpc_power_w: f64,
    pub hpt_power_w: f64,
    pub ipt_power_w: f64,
    pub lpt_power_w: f64,
    pub net_thrust_n: f64,
    pub m_fan: f64,
    pub m_core: f64,
    pub m_bypass: f64,
    pub tt13_k: f64,
    pub tt25_k: f64,
    pub pt25_pa: f64,
    pub tt3_k: f64,
    pub pt3_pa: f64,
    pub tt4_k: f64,
    pub tt44_k: f64,
    pub tt45_k: f64,
    pub tt5_k: f64,
    pub mdot_gas_kg_s: f64,
    pub fan_stall_margin: f64,
    pub ipc_stall_margin: f64,
    pub hpc_stall_margin: f64,
}

#[derive(Clone, Debug)]
pub struct GasPath {
    pub design: Design,
    pub state: State,
}

impl GasPath {
    pub fn new() -> Self {
        let design = design();
        let state = design.state;
        Self { design, state }
    }

    pub fn rest(&mut self, ambient_pa: f64) {
        self.state = State { m_fan: 0.0, m_ipc: 0.0, m_hpc: 0.0, p13: ambient_pa, p25: ambient_pa, p3: ambient_pa, p44: ambient_pa, p45: ambient_pa, p5: ambient_pa };
    }

    pub fn step(&mut self, i: &Inputs, dt: f64) -> Outputs {
        let s2 = super::inlet::station2(i.ambient_pressure_pa, i.ambient_temp_k, i.mach);
        let amb = i.ambient_pressure_pa.max(1.0);
        let (w_lp, w_ip, w_hp) = (omega(i.lp_rpm), omega(i.ip_rpm), omega(i.hp_rpm));
        let n = ((dt / SUBSTEP_S).ceil() as usize).clamp(1, MAX_SUBSTEPS);
        let h = dt.max(0.0) / n as f64;
        let d = &self.design;
        let mut sum = Outputs::default();
        let mut last = Outputs::default();
        let floor = 0.2 * amb;
        let theta = (s2.tt_k / T_REF_K).sqrt();
        let ip_open = bleed_open(i.ip_rpm / N2_DESIGN_RPM * 100.0 / theta, IP_BLEED_SCHEDULE_PCT);
        let hp_open = bleed_open(i.hp_rpm / N3_DESIGN_RPM * 100.0 / theta, HP3_BLEED_SCHEDULE_PCT);
        for _ in 0..n {
            let st = &mut self.state;
            let f = d.fan.compress(st.m_fan, w_lp, s2.tt_k, s2.pt_pa, 1.0, 1.0);
            let p21 = s2.pt_pa + FAN_HUB_FRACTION * (st.p13 - s2.pt_pa);
            let t21 = s2.tt_k + FAN_HUB_FRACTION * (f.tt_out_k - s2.tt_k);
            let ip_bleed = Some(HandlingBleed { after_stage: IP_BLEED_AFTER_STAGE, area_m2: IP_BLEED_AREA_M2 * ip_open, sink_pa: st.p13 });
            let hp_bleed = Some(HandlingBleed { after_stage: HP3_BLEED_AFTER_STAGE, area_m2: HP3_BLEED_AREA_M2 * hp_open, sink_pa: st.p13 });
            let ip = d.ipc.compress_bled(st.m_ipc, w_ip, t21, p21, 1.0, 1.0, ip_bleed);
            let hp = d.hpc.compress_bled(st.m_hpc, w_hp, ip.tt_out_k, st.p25, i.hpc_flow_capacity, i.hpc_efficiency, hp_bleed);
            let air = (st.m_hpc - hp.bleed_kg_s - i.hp_bleed_kg_s).max(0.0);
            let c = combustor::burn(air, i.wf_kg_s, hp.tt_out_k, st.p3);
            let hpt = d.hpt.expand(c.pt4_pa, c.tt4_k, st.p44, w_hp, i.turbine_efficiency);
            let ipt = d.ipt.expand(st.p44, hpt.tt_out_k, st.p45, w_ip, i.turbine_efficiency);
            let lpt = d.lpt.expand(st.p45, ipt.tt_out_k, st.p5, w_lp, 1.0);
            let m_core_out = nozzle::mass_flow_capacity(lpt.tt_out_k, st.p5, amb, d.core_nozzle_area_m2, GAMMA_GAS, R_GAS);
            let (m_byp, p13_byp) = bypass_flow(f.tt_out_k, st.p13, amb, d.bypass_nozzle_area_m2, d.mdot_bypass_kg_s);

            let implicit = |m: f64, a_over_l: f64, p_char: f64, p_down: f64, char_at: &dyn Fn(f64) -> f64| {
                let dm = (1e-3 * m.abs()).max(1e-3);
                let slope = (char_at(m + dm) - p_char) / dm;
                m + h * a_over_l * (p_char - p_down) / (1.0 - h * a_over_l * slope).max(1.0)
            };
            let (m_fan, m_ipc, m_hpc) = (st.m_fan, st.m_ipc, st.m_hpc);
            let fan_max = d.fan.choke_kg_s(s2.tt_k, s2.pt_pa, 1.0);
            let ipc_max = d.ipc.choke_kg_s(t21, p21, 1.0);
            let hpc_max = d.hpc.choke_kg_s(ip.tt_out_k, st.p25, i.hpc_flow_capacity);
            st.m_fan = implicit(m_fan, A_OVER_L_FAN, f.pt_out_pa, st.p13, &|m| d.fan.compress(m, w_lp, s2.tt_k, s2.pt_pa, 1.0, 1.0).pt_out_pa).clamp(-fan_max, fan_max);
            st.m_ipc = implicit(m_ipc, A_OVER_L_IPC, ip.pt_out_pa, st.p25, &|m| d.ipc.compress_bled(m, w_ip, t21, p21, 1.0, 1.0, ip_bleed).pt_out_pa).clamp(-ipc_max, ipc_max);
            let p25 = st.p25;
            st.m_hpc = implicit(m_hpc, A_OVER_L_HPC, hp.pt_out_pa, st.p3, &|m| {
                d.hpc.compress_bled(m, w_hp, ip.tt_out_k, p25, i.hpc_flow_capacity, i.hpc_efficiency, hp_bleed).pt_out_pa
            })
            .clamp(-hpc_max, hpc_max);
            let loss = 1.0 - COMBUSTOR_PRESSURE_LOSS_FRAC;
            let hpt_flow = |p3: f64, p44: f64| d.hpt.expand(p3 * loss, c.tt4_k, p44, w_hp, i.turbine_efficiency).mdot_kg_s;
            let ipt_flow = |p44: f64, p45: f64| d.ipt.expand(p44, hpt.tt_out_k, p45, w_ip, i.turbine_efficiency).mdot_kg_s;
            let lpt_flow = |p45: f64, p5: f64| d.lpt.expand(p45, ipt.tt_out_k, p5, w_lp, 1.0).mdot_kg_s;
            let core_flow = |p5: f64| nozzle::mass_flow_capacity(lpt.tt_out_k, p5, amb, d.core_nozzle_area_m2, GAMMA_GAS, R_GAS);
            let byp_flow = |p13: f64| bypass_flow(f.tt_out_k, p13, amb, d.bypass_nozzle_area_m2, d.mdot_bypass_kg_s).0;
            let step_p = |p: f64, capacitance: f64, net: &dyn Fn(f64) -> f64| {
                let dp = (1e-4 * p).max(1.0);
                let n0 = net(p);
                let slope = (net(p + dp) - n0) / dp;
                p + h * capacitance * n0 / (1.0 - h * capacitance * slope).max(1.0)
            };
            let (m_fan_new, m_ipc_new, m_hpc_new) = (st.m_fan, st.m_ipc, st.m_hpc);
            let (p13, p3, p44, p45, p5) = (st.p13, st.p3, st.p44, st.p45, st.p5);
            let t3_mix = 0.5 * (hp.tt_out_k + c.tt4_k);
            let dumped = ip.bleed_kg_s + hp.bleed_kg_s;
            st.p13 = step_p(p13, R_AIR * f.tt_out_k / V13, &|p| m_fan_new - m_ipc_new + dumped - byp_flow(p));
            st.p25 += h * R_AIR * ip.tt_out_k / V25 * (m_ipc_new - ip.bleed_kg_s - i.ip_bleed_kg_s - m_hpc_new);
            st.p3 = step_p(p3, R_AIR * t3_mix / V3, &|p| m_hpc_new - hp.bleed_kg_s - i.hp_bleed_kg_s + i.wf_kg_s - hpt_flow(p, p44));
            st.p44 = step_p(p44, R_GAS * hpt.tt_out_k / V44, &|p| hpt_flow(p3, p) - ipt_flow(p, p45));
            st.p45 = step_p(p45, R_GAS * ipt.tt_out_k / V45, &|p| ipt_flow(p44, p) - lpt_flow(p, p5));
            st.p5 = step_p(p5, R_GAS * lpt.tt_out_k / V5, &|p| lpt_flow(p45, p) - core_flow(p));
            let _ = (m_core_out, m_byp);
            for p in [&mut st.p13, &mut st.p25, &mut st.p3, &mut st.p44, &mut st.p45, &mut st.p5] {
                *p = p.max(floor);
            }

            let thrust = nozzle::thrust(m_core_out, lpt.tt_out_k, st.p5, amb, i.true_airspeed_m_s, GAMMA_GAS, R_GAS).thrust_n
                + nozzle::thrust(m_byp, f.tt_out_k, p13_byp, amb, i.true_airspeed_m_s, GAMMA_AIR, R_AIR).thrust_n;
            last = Outputs {
                fan_power_w: f.power_w,
                ipc_power_w: ip.power_w,
                hpc_power_w: hp.power_w,
                hpt_power_w: hpt.power_w,
                ipt_power_w: ipt.power_w,
                lpt_power_w: lpt.power_w,
                net_thrust_n: thrust,
                m_fan: st.m_fan,
                m_core: st.m_hpc,
                m_bypass: m_byp,
                tt13_k: f.tt_out_k,
                tt25_k: ip.tt_out_k,
                pt25_pa: st.p25,
                tt3_k: hp.tt_out_k,
                pt3_pa: st.p3,
                tt4_k: c.tt4_k,
                tt44_k: hpt.tt_out_k,
                tt45_k: ipt.tt_out_k,
                tt5_k: lpt.tt_out_k,
                mdot_gas_kg_s: hpt.mdot_kg_s,
                fan_stall_margin: f.stall_margin,
                ipc_stall_margin: ip.stall_margin,
                hpc_stall_margin: hp.stall_margin,
            };
            sum.fan_power_w += last.fan_power_w;
            sum.ipc_power_w += last.ipc_power_w;
            sum.hpc_power_w += last.hpc_power_w;
            sum.hpt_power_w += last.hpt_power_w;
            sum.ipt_power_w += last.ipt_power_w;
            sum.lpt_power_w += last.lpt_power_w;
            sum.net_thrust_n += last.net_thrust_n;
        }
        let k = 1.0 / n as f64;
        Outputs {
            fan_power_w: sum.fan_power_w * k,
            ipc_power_w: sum.ipc_power_w * k,
            hpc_power_w: sum.hpc_power_w * k,
            hpt_power_w: sum.hpt_power_w * k,
            ipt_power_w: sum.ipt_power_w * k,
            lpt_power_w: sum.lpt_power_w * k,
            net_thrust_n: sum.net_thrust_n * k,
            ..last
        }
    }
}

impl Default for GasPath {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn design_inputs(d: &Design) -> Inputs {
        Inputs {
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
        }
    }

    #[test]
    fn the_design_point_makes_the_certificated_thrust_at_a_plausible_cycle() {
        let d = design();
        println!("T4 {:.0} K, OPR {:.1}, core {:.1} kg/s, wf {:.3} kg/s, thrust {:.0} N", d.tt4_k, d.opr, d.mdot_core_kg_s, d.wf_kg_s, d.thrust_n);
        assert!((d.thrust_n - STATIC_THRUST_N).abs() / STATIC_THRUST_N < 1e-3);
        assert!(d.tt4_k > 1400.0 && d.tt4_k < 2100.0, "T4 {:.0} K", d.tt4_k);
        assert!(d.opr > 25.0 && d.opr < 50.0, "OPR {:.1}", d.opr);
    }

    #[test]
    fn the_design_point_is_an_equilibrium() {
        let settle = |lp_rpm: f64| -> (GasPath, Outputs) {
            let mut g = GasPath::new();
            let mut i = design_inputs(&g.design);
            i.lp_rpm = lp_rpm;
            let mut out = Outputs::default();
            for _ in 0..40 {
                out = g.step(&i, 0.05);
            }
            (g, out)
        };
        let power_error = |lp_rpm: f64| -> f64 {
            let (_, out) = settle(lp_rpm);
            out.lpt_power_w * MECH_EFFICIENCY - out.fan_power_w
        };

        let mut lo = N1_DESIGN_RPM * 0.9;
        let mut hi = N1_DESIGN_RPM * 1.1;
        for _ in 0..40 {
            let mid = 0.5 * (lo + hi);
            if power_error(lo).signum() == power_error(mid).signum() {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        let lp_rpm = 0.5 * (lo + hi);

        let (mut g, _) = settle(lp_rpm);
        let mut i = design_inputs(&g.design);
        i.lp_rpm = lp_rpm;
        let before = g.state;
        let out = g.step(&i, 0.5);
        let after = g.state;
        for (name, a, b) in [("m_fan", before.m_fan, after.m_fan), ("m_hpc", before.m_hpc, after.m_hpc), ("p3", before.p3, after.p3), ("p45", before.p45, after.p45), ("p5", before.p5, after.p5)] {
            assert!((a - b).abs() / a.abs().max(1.0) < 1e-3, "{name} drifted {a} -> {b}");
        }
        assert!((out.hpt_power_w * MECH_EFFICIENCY - out.hpc_power_w).abs() / out.hpc_power_w < 0.01);
        assert!((out.lpt_power_w * MECH_EFFICIENCY - out.fan_power_w).abs() / out.fan_power_w < 0.01);
    }

    #[test]
    fn less_fuel_at_design_speeds_settles_to_lower_pressures_without_blowing_up() {
        let mut g = GasPath::new();
        let mut i = design_inputs(&g.design);
        i.wf_kg_s *= 0.8;
        let mut out = Outputs::default();
        for k in 0..40 {
            out = g.step(&i, 0.05);
            if k % 4 == 0 {
                println!("t {:.2} p3 {:.0} p44 {:.0} p45 {:.0} p5 {:.0} m_hpc {:.2} m_ipc {:.2} m_fan {:.1} T4 {:.0} thrust {:.0}", (k + 1) as f64 * 0.05, g.state.p3, g.state.p44, g.state.p45, g.state.p5, g.state.m_hpc, g.state.m_ipc, g.state.m_fan, out.tt4_k, out.net_thrust_n);
            }
        }
        assert!(out.pt3_pa.is_finite() && out.net_thrust_n.is_finite());
        assert!(out.pt3_pa < g.design.state.p3 && out.tt4_k < g.design.tt4_k);
        assert!(out.m_core > 0.0);
    }

    #[test]
    fn choke_flow_function_is_the_textbook_value() {
        let g = GAMMA_AIR;
        let exact = (g / R_AIR).sqrt() * (2.0 / (g + 1.0)).powf((g + 1.0) / (2.0 * (g - 1.0)));
        assert!((CHOKE_FLOW_FN_AIR - exact).abs() / exact < 1e-6, "{CHOKE_FLOW_FN_AIR} vs {exact}");
        let d = design();
        let s2 = super::super::inlet::station2(P_REF_PA, T_REF_K, 0.0);
        let fan_choke = d.fan.choke_kg_s(s2.tt_k, s2.pt_pa, 1.0);
        assert!(fan_choke > MDOT_TOTAL_DESIGN_KG_S * 1.1 && fan_choke < MDOT_TOTAL_DESIGN_KG_S * 1.5, "fan choke {fan_choke:.0} kg/s");
        let hpc_choke = d.hpc.choke_kg_s(400.0, d.state.p25, 1.0);
        assert!(hpc_choke > d.mdot_core_kg_s, "HPC choke {hpc_choke:.1} kg/s vs design core {:.1}", d.mdot_core_kg_s);
    }

    #[test]
    fn far_off_design_flow_stays_physical_instead_of_following_the_cubic() {
        let d = design();
        let (w, t, p) = (omega(N3_DESIGN_RPM), 450.0, 400_000.0);
        for m in [0.0, 1.0, 50.0, 200.0, 1e4, 1e9] {
            let c = d.hpc.compress(m, w, t, p, 1.0, 1.0);
            assert!(c.power_w.is_finite() && c.tt_out_k.is_finite() && c.pt_out_pa.is_finite(), "{m}: {c:?}");
            assert!(c.tt_out_k > 0.5 * t, "{m}: stack chilled the flow to {} K", c.tt_out_k);
            assert!(c.pt_out_pa > 0.01 * p && c.pt_out_pa < 100.0 * p, "{m}: {} Pa from {p} Pa", c.pt_out_pa);
            let u2 = (omega(N3_DESIGN_RPM) * HPC_RADIUS_M).powi(2);
            assert!(c.power_w.abs() <= 20.0 * u2 * m.max(1.0) * 6.0, "{m}: {} W", c.power_w);
        }
    }

    #[test]
    fn reverse_flow_pushes_back_harder_the_further_it_reverses() {
        let d = design();
        let (w, t, p) = (omega(N3_DESIGN_RPM), 450.0, 400_000.0);
        let mut last = d.hpc.compress(0.0, w, t, p, 1.0, 1.0).pt_out_pa;
        for m in [-1.0, -10.0, -50.0, -150.0] {
            let c = d.hpc.compress(m, w, t, p, 1.0, 1.0);
            assert!(c.pt_out_pa > last, "reverse flow {m} kg/s gave {} Pa, no more restoring than {last} Pa", c.pt_out_pa);
            assert!(c.tt_out_k.is_finite() && c.tt_out_k > t, "churning must heat, not cool: {} K", c.tt_out_k);
            last = c.pt_out_pa;
        }
    }

    #[test]
    fn part_speed_costs_efficiency_and_closes_the_stators() {
        assert!((vsv_setting(1.0) - 1.0).abs() < 1e-12);
        assert!(vsv_setting(0.6) < 1.0 && vsv_setting(0.6) >= VSV_MIN);
        let d = design();
        let (t, p) = (400.0, 550_000.0);
        let vane = |frac: f64| vsv_setting(frac);
        let at = |frac: f64| {
            let w = omega(N3_DESIGN_RPM * frac);
            let m = d.mdot_core_kg_s * frac * vane(frac);
            let c = d.hpc.compress(m, w, t, p, 1.0, 1.0);
            let k = (GAMMA_AIR - 1.0) / GAMMA_AIR;
            CP_AIR * t * ((c.pt_out_pa / p).powf(k) - 1.0) / (c.power_w / m)
        };
        assert!(at(0.5) < at(1.0), "part speed must cost efficiency: {} vs {}", at(0.5), at(1.0));

    }

    #[test]
    fn a_stage_stalls_when_throttled_below_its_peak() {
        let d = design();
        let (w, t, p) = (omega(N3_DESIGN_RPM), 450.0, 400_000.0);
        let healthy = d.hpc.compress(d.mdot_core_kg_s * 1.0, w, t, p, 1.0, 1.0);
        let throttled = d.hpc.compress(d.mdot_core_kg_s * 0.6, w, t, p, 1.0, 1.0);
        assert!(throttled.stall_margin < 1.0 && healthy.stall_margin >= 1.0, "{} {}", healthy.stall_margin, throttled.stall_margin);
    }
}



#[cfg(test)]
mod handling_bleed_tests {
    use super::*;

    #[test]
    fn a_handling_bleed_dumps_a_fraction_of_the_core_flow_not_most_of_it() {
        let d = design();
        let m = d.mdot_core_kg_s * 0.55;
        let w_hp = omega(N3_DESIGN_RPM * 0.70);
        let ipc = d.ipc.compress(m, omega(N2_DESIGN_RPM * 0.75), T_REF_K, d.state.p13, 1.0, 1.0);
        let open = Some(HandlingBleed { after_stage: HP3_BLEED_AFTER_STAGE, area_m2: HP3_BLEED_AREA_M2, sink_pa: d.state.p13 });
        let hp = d.hpc.compress_bled(m, w_hp, ipc.tt_out_k, d.state.p25, 1.0, 1.0, open);
        let fraction = hp.bleed_kg_s / m;
        assert!(
            (0.05..=0.25).contains(&fraction),
            "the HP3 handling bleed dumps {:.0}% of the core flow when open; a handling bleed is a tenth to a fifth,              and most of the core is a starved combustor the FADEC answers with more fuel",
            fraction * 100.0
        );
    }
}
