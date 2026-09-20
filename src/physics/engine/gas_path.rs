//! The Trent 972 gas path as volumes and flows: the component matching a
//! real engine does physically, instead of the "downstream compressors take
//! whatever the fan sends" shortcut.
//!
//! - **Compressors** (fan, 8-stage IP, 6-stage HP) are stacked stage by
//!   stage. Each stage follows a Moore-Greitzer cubic characteristic
//!   (pressure-rise coefficient against flow coefficient), with its peak on
//!   the left: flow below the peak is stalled flow, and the characteristic
//!   carries on into reverse flow. Each compressor's mass flow is a state
//!   with duct inertia, `d(mdot)/dt = (A/L)(p_characteristic - p_plenum)`
//!   (Greitzer 1976), so surge and rotating-stall-like hang-ups come out of
//!   the dynamics rather than a flag.
//! - **Plenums** between components hold pressure states,
//!   `dp/dt = (R T / V)(mdot_in - mdot_out)`: fan exit/bypass duct, IP-HP
//!   duct, combustor, HP-IP turbine interstage, IP-LP interstage (the TGT
//!   plane), and the LP turbine exit.
//! - **Turbines** pass what their flow capacity allows: the single-stage HP
//!   and IP turbines through choking nozzle guide vanes (isentropic nozzle
//!   flow function), the 5-stage LP turbine by Stodola's ellipse law.
//!   Efficiency depends on the blade speed ratio `U/C0`.
//! - **Nozzles** are fixed-area convergent nozzles (`nozzle.rs`).
//!
//! The design point is sea-level static at 100% corrected speeds, the
//! package's certificated thrust; its turbine entry temperature is solved so
//! the model makes exactly that thrust, and every flow capacity and area is
//! backed out from it, so the design point is an exact equilibrium. Stage
//! counts are the TCDS's (E.012 section 2). Stage characteristics, radii,
//! volumes and duct inertances are GENERIC (no public Trent data), scaled to
//! public figures: fan diameter, rotor speeds, bypass ratio, dry weight.

use super::combustor;
use super::gas::{CP_AIR, CP_GAS, GAMMA_AIR, GAMMA_GAS, R_AIR, R_GAS};
use super::nozzle;
use super::params::*;

// ---- Stage characteristic (Moore-Greitzer cubic), GENERIC -----------------

/// Design flow coefficient `Cx / U`.
const PHI_DESIGN: f64 = 0.5;
/// Design point sits at `phi/W - 1 = 1.4`, right of the peak at 1.0: about
/// 17% of flow margin to the stage's stall point.
const X_DESIGN: f64 = 1.4;
/// Shut-off head over the cubic's semi-height, `psi0 / H`.
const SHUTOFF_OVER_H: f64 = 0.7;
/// Efficiency island curvature in flow coefficient.
const ETA_FALLOFF: f64 = 2.5;
/// Loss, in dynamic heads, of a stage passing flow it cannot do work on (a
/// barely turning rotor, or flow far beyond choke).
const K_THROTTLE: f64 = 0.5;
/// Loss, in dynamic heads, of flow forced backwards through a stage.
const K_REVERSE: f64 = 1.0;
/// Efficiency of the churning in reversed flow.
const ETA_REVERSE: f64 = 0.3;

fn cubic_bracket(x: f64) -> f64 {
    1.0 + 1.5 * x - 0.5 * x * x * x
}

#[derive(Clone, Copy, Debug)]
struct Stage {
    radius_m: f64,
    area_m2: f64,
    /// Cubic semi-height H and semi-width W.
    h: f64,
    w: f64,
    psi0: f64,
    eta_design: f64,
}

impl Stage {
    /// Isentropic enthalpy rise and actual work, J/kg, for flow `mdot`
    /// through this stage at inlet `tt`, `pt` and shaft speed `omega`.
    fn work(&self, mdot: f64, omega: f64, tt: f64, pt: f64, area_scale: f64, eta_scale: f64) -> (f64, f64) {
        let u = omega * self.radius_m;
        let rho = pt / (R_AIR * tt.max(1.0));
        let cx = mdot / (rho * self.area_m2 * area_scale).max(1e-9);
        if mdot >= 0.0 {
            let throttle = -K_THROTTLE * cx * cx;
            let (dh_s, eta) = if u < 1.0 {
                (throttle, self.eta_design)
            } else {
                let phi = cx / u;
                let psi = self.psi0 + self.h * cubic_bracket(phi / self.w - 1.0);
                let eta = self.eta_design * (1.0 - ETA_FALLOFF * (phi / PHI_DESIGN - 1.0).powi(2));
                ((psi * u * u).max(throttle), eta.clamp(0.05, self.eta_design))
            };
            let eta = (eta * eta_scale).max(0.02);
            let dh0 = if dh_s > 0.0 { dh_s / eta } else { dh_s * eta };
            (dh_s, dh0)
        } else {
            let dh_s = self.psi0 * u * u + K_REVERSE * cx * cx;
            (dh_s, dh_s / ETA_REVERSE)
        }
    }
}

/// A compressor as a stack of stages on one shaft.
#[derive(Clone, Debug)]
pub struct Compressor {
    stages: Vec<Stage>,
}

/// One pass through a compressor.
#[derive(Clone, Copy, Debug, Default)]
pub struct Compression {
    pub tt_out_k: f64,
    pub pt_out_pa: f64,
    /// Shaft power absorbed, W.
    pub power_w: f64,
    /// Lowest stage flow coefficient over its stall value (<1: a stage is
    /// stalled).
    pub stall_margin: f64,
    /// Flow out through an interstage handling bleed valve, kg/s.
    pub bleed_kg_s: f64,
}

/// An interstage handling bleed valve: air let out after `after_stage`
/// through an orifice of `area_m2` into `sink_pa` (the bypass duct).
#[derive(Clone, Copy, Debug)]
pub struct HandlingBleed {
    pub after_stage: usize,
    pub area_m2: f64,
    pub sink_pa: f64,
}

impl Compressor {
    /// Designs `n` equal-work stages at mean radius `radius_m` that give
    /// `pr` at design flow `mdot` and shaft speed `omega` from inlet `tt`,
    /// `pt`, with overall isentropic efficiency `eta`.
    fn design(n: usize, radius_m: f64, mdot: f64, omega: f64, tt: f64, pt: f64, pr: f64, eta: f64) -> Self {
        let k = (GAMMA_AIR - 1.0) / GAMMA_AIR;
        let dh0_total = CP_AIR * tt * (pr.powf(k) - 1.0) / eta;
        let dh0 = dh0_total / n as f64;
        let u = omega * radius_m;
        // Stage efficiency so the stack hits the overall pressure ratio.
        let build = |eta_st: f64| {
            let mut stages = Vec::with_capacity(n);
            let (mut t, mut p) = (tt, pt);
            for _ in 0..n {
                let dh_s = eta_st * dh0;
                let psi_d = dh_s / (u * u);
                let h = psi_d / (SHUTOFF_OVER_H + cubic_bracket(X_DESIGN));
                let w = PHI_DESIGN / (1.0 + X_DESIGN);
                let rho = p / (R_AIR * t);
                let area = mdot / (rho * PHI_DESIGN * u);
                stages.push(Stage { radius_m, area_m2: area, h, w, psi0: SHUTOFF_OVER_H * h, eta_design: eta_st });
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

    /// Flow `mdot` through the stack at shaft speed `omega` from inlet
    /// `tt`, `pt`. `flow_capacity` and `efficiency` scale every stage's
    /// annulus and efficiency (1 healthy): damage.
    pub fn compress(&self, mdot: f64, omega: f64, tt: f64, pt: f64, flow_capacity: f64, efficiency: f64) -> Compression {
        self.compress_bled(mdot, omega, tt, pt, flow_capacity, efficiency, None)
    }

    /// As `compress`, with a handling bleed valve letting air out between
    /// stages: the stages ahead of it pass the full inlet flow, those
    /// behind it only what is left.
    pub fn compress_bled(&self, mdot: f64, omega: f64, tt: f64, pt: f64, flow_capacity: f64, efficiency: f64, bleed: Option<HandlingBleed>) -> Compression {
        let k = (GAMMA_AIR - 1.0) / GAMMA_AIR;
        let (mut t, mut p, mut power) = (tt, pt, 0.0);
        let mut margin = f64::INFINITY;
        let mut mdot = mdot;
        let mut bled = 0.0;
        for (index, s) in self.stages.iter().enumerate() {
            if let Some(b) = bleed {
                if index == b.after_stage && b.area_m2 > 0.0 && mdot > 0.0 {
                    bled = nozzle::mass_flow_capacity(t, p, b.sink_pa, b.area_m2, GAMMA_AIR, R_AIR).min(mdot);
                    mdot -= bled;
                }
            }
            let (dh_s, dh0) = s.work(mdot, omega, t, p, flow_capacity, efficiency);
            let u = omega * s.radius_m;
            if u > 1.0 && mdot > 0.0 {
                let rho = p / (R_AIR * t.max(1.0));
                let phi = mdot / (rho * s.area_m2 * flow_capacity).max(1e-9) / u;
                margin = margin.min(phi / (2.0 * s.w));
            }
            power += mdot.abs() * dh0;
            p *= (1.0 + dh_s / (CP_AIR * t.max(1.0))).max(1e-3).powf(1.0 / k);
            t = (t + dh0 / CP_AIR).max(1.0);
        }
        Compression { tt_out_k: t, pt_out_pa: p, power_w: power, stall_margin: if margin.is_finite() { margin } else { 0.0 }, bleed_kg_s: bled }
    }
}

// ---- Turbines -----------------------------------------------------------

#[derive(Clone, Copy, Debug)]
pub struct Turbine {
    /// `mdot sqrt(Tt_in) / Pt_in` when fully choked (single stage) or the
    /// Stodola constant (multistage).
    flow_constant: f64,
    multistage: bool,
    eta_design: f64,
    radius_m: f64,
    /// Blade speed ratio `U / C0` at design.
    nu_design: f64,
}

/// One pass through a turbine.
#[derive(Clone, Copy, Debug, Default)]
pub struct Expansion {
    pub mdot_kg_s: f64,
    pub tt_out_k: f64,
    pub power_w: f64,
}

/// The isentropic nozzle flow function at pressure ratio `out/in`,
/// normalised to 1 when choked.
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

// ---- Geometry, volumes, inertances (GENERIC) ----------------------------

const FAN_MEAN_RADIUS_M: f64 = FAN_DIAMETER_M / 2.0 * 0.72;
const IPC_RADIUS_M: f64 = 0.45;
const HPC_RADIUS_M: f64 = 0.33;
const HPT_RADIUS_M: f64 = 0.40;
const IPT_RADIUS_M: f64 = 0.46;
const LPT_RADIUS_M: f64 = 0.72;
/// How much of the fan's pressure rise the core stream behind the fan hub
/// gets (the hub does less work than the tip).
const FAN_HUB_FRACTION: f64 = 0.6;
/// Handling bleed valves (the Trent's IP and HP3 handling bleeds, dumping
/// to the bypass duct): the EEC opens them at low corrected speed so the
/// front stages, which see too little flow there, stay out of stall. Valve
/// areas and schedules are GENERIC, sized so a start and ground idle keep
/// both compressors unstalled.
const HP3_BLEED_AFTER_STAGE: usize = 3;
const HP3_BLEED_AREA_M2: f64 = 0.03;
/// Fully open below the first corrected HP speed, shut above the second.
const HP3_BLEED_SCHEDULE_PCT: (f64, f64) = (70.0, 85.0);
const IP_BLEED_AFTER_STAGE: usize = 8;
const IP_BLEED_AREA_M2: f64 = 0.05;
const IP_BLEED_SCHEDULE_PCT: (f64, f64) = (65.0, 80.0);

fn bleed_open(corrected_pct: f64, (open_below, shut_above): (f64, f64)) -> f64 {
    ((shut_above - corrected_pct) / (shut_above - open_below)).clamp(0.0, 1.0)
}

/// Plenum volumes, m^3: bypass duct, IP-HP duct, combustor, HP-IP
/// interstage, IP-LP interstage, LP turbine exit.
const V13: f64 = 8.0;
const V25: f64 = 0.25;
const V3: f64 = 0.25;
const V44: f64 = 0.08;
const V45: f64 = 0.25;
const V5: f64 = 1.2;
/// Duct inertance `A / L`, m, of each compressor's flow path.
const A_OVER_L_FAN: f64 = 3.0;
const A_OVER_L_IPC: f64 = 0.35;
const A_OVER_L_HPC: f64 = 0.12;
/// Sub-step, s: the compressor flows are implicit; the stiffest explicit term
/// (the LP turbine exit plenum, ~900 /s) stays well inside it.
pub const SUBSTEP_S: f64 = 0.001;
const MAX_SUBSTEPS: usize = 2000;

pub fn omega(rpm: f64) -> f64 {
    rpm * std::f64::consts::PI / 30.0
}

/// Design point of the whole gas path.
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
    pub opr: f64,
    pub thrust_n: f64,
    pub state: State,
}

/// The dynamic states.
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

/// A candidate design at turbine entry temperature `tt4_k`, or `None`
/// when the turbines cannot drive the compressors at that temperature.
fn design_at(tt4_k: f64) -> Option<Design> {
    let (p2, t2) = (P_REF_PA * RAM_RECOVERY, T_REF_K);
    let m_total = MDOT_TOTAL_DESIGN_KG_S;
    let m_core = m_total / (1.0 + BYPASS_RATIO);
    let m_byp = m_total - m_core;
    let (w_lp, w_ip, w_hp) = (omega(N1_DESIGN_RPM), omega(N2_DESIGN_RPM), omega(N3_DESIGN_RPM));

    let fan = Compressor::design(1, FAN_MEAN_RADIUS_M, m_total, w_lp, t2, p2, PR_FAN_DESIGN, ETA_FAN_DESIGN);
    let f = fan.compress(m_total, w_lp, t2, p2, 1.0, 1.0);
    let (p21, t21) = (p2 + FAN_HUB_FRACTION * (f.pt_out_pa - p2), t2 + FAN_HUB_FRACTION * (f.tt_out_k - t2));
    let ipc = Compressor::design(8, IPC_RADIUS_M, m_core, w_ip, t21, p21, PR_IPC_DESIGN, ETA_IPC_DESIGN);
    let i = ipc.compress(m_core, w_ip, t21, p21, 1.0, 1.0);
    let hpc = Compressor::design(6, HPC_RADIUS_M, m_core, w_hp, i.tt_out_k, i.pt_out_pa, PR_HPC_DESIGN, ETA_HPC_DESIGN);
    let h = hpc.compress(m_core, w_hp, i.tt_out_k, i.pt_out_pa, 1.0, 1.0);

    // Fuel for tt4 from the combustor's own energy balance.
    let wf = m_core * CP_AIR * (tt4_k - h.tt_out_k) / (LHV_JET_A1_J_KG * COMBUSTOR_EFFICIENCY - CP_AIR * tt4_k);
    if !(wf > 0.0) {
        return None;
    }
    let c = combustor::burn(m_core, wf, h.tt_out_k, h.pt_out_pa);
    let m_gas = c.mdot_gas_kg_s;
    let kg = (GAMMA_GAS - 1.0) / GAMMA_GAS;
    // Each turbine delivers its compressor's power through the shaft's
    // mechanical efficiency.
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
    let core_area = nozzle::design_area_m2(m_gas, t5, p5, P_REF_PA, GAMMA_GAS, R_GAS);
    let p13_byp = f.pt_out_pa * (1.0 - BYPASS_DUCT_LOSS_FRAC);
    let bypass_area = nozzle::design_area_m2(m_byp, f.tt_out_k, p13_byp, P_REF_PA, GAMMA_AIR, R_AIR);
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
        opr: h.pt_out_pa / p2,
        thrust_n: thrust,
        state: State { m_fan: m_total, m_ipc: m_core, m_hpc: m_core, p13: f.pt_out_pa, p25: i.pt_out_pa, p3: h.pt_out_pa, p44, p45, p5 },
    })
}

/// The design point: the turbine entry temperature that makes the
/// certificated take-off thrust at sea level static.
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

// ---- The running gas path -------------------------------------------------

/// Everything the gas path needs from outside for one frame.
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
    /// Customer bleed off IP8 (the IP-HP duct) and HP6 (the combustor
    /// casing), kg/s.
    pub ip_bleed_kg_s: f64,
    pub hp_bleed_kg_s: f64,
    /// Damage, 1 healthy: HP compressor efficiency and flow capacity,
    /// turbine efficiency.
    pub hpc_efficiency: f64,
    pub hpc_flow_capacity: f64,
    pub turbine_efficiency: f64,
}

/// A frame's result, averaged over its sub-steps where it is a rate.
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
    /// The IP-LP interstage: the TGT plane.
    pub tt45_k: f64,
    pub tt5_k: f64,
    pub mdot_gas_kg_s: f64,
    /// Lowest stall margin of each compressor this frame (<1 stalled).
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
    /// At the design point, running.
    pub fn new() -> Self {
        let design = design();
        let state = design.state;
        Self { design, state }
    }

    /// Stopped: no flow, every plenum at ambient pressure.
    pub fn rest(&mut self, ambient_pa: f64) {
        self.state = State { m_fan: 0.0, m_ipc: 0.0, m_hpc: 0.0, p13: ambient_pa, p25: ambient_pa, p3: ambient_pa, p44: ambient_pa, p45: ambient_pa, p5: ambient_pa };
    }

    /// Advances the gas path by `dt` with the spool speeds held (they change
    /// far more slowly than the pressures; the caller integrates them with
    /// the powers returned).
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
            let p13_byp = st.p13 * (1.0 - BYPASS_DUCT_LOSS_FRAC);
            let m_byp = nozzle::mass_flow_capacity(f.tt_out_k, p13_byp, amb, d.bypass_nozzle_area_m2, GAMMA_AIR, R_AIR);

            // Flows (duct inertia), then plenums with the new flows. A
            // stacked compressor's characteristic is steep in flow, so each
            // flow is advanced implicitly, linearised about its own slope
            // `dp_char/dm` (a second pass through the stack): stable where the
            // slope is negative (the normal working line), and on the stalled,
            // positive-slope side it is left explicit, as unstable as the
            // physics.
            let implicit = |m: f64, a_over_l: f64, p_char: f64, p_down: f64, char_at: &dyn Fn(f64) -> f64| {
                let dm = (1e-3 * m.abs()).max(1e-3);
                let slope = (char_at(m + dm) - p_char) / dm;
                m + h * a_over_l * (p_char - p_down) / (1.0 - h * a_over_l * slope).max(1.0)
            };
            let (m_fan, m_ipc, m_hpc) = (st.m_fan, st.m_ipc, st.m_hpc);
            st.m_fan = implicit(m_fan, A_OVER_L_FAN, f.pt_out_pa, st.p13, &|m| d.fan.compress(m, w_lp, s2.tt_k, s2.pt_pa, 1.0, 1.0).pt_out_pa);
            st.m_ipc = implicit(m_ipc, A_OVER_L_IPC, ip.pt_out_pa, st.p25, &|m| d.ipc.compress_bled(m, w_ip, t21, p21, 1.0, 1.0, ip_bleed).pt_out_pa);
            let p25 = st.p25;
            st.m_hpc = implicit(m_hpc, A_OVER_L_HPC, hp.pt_out_pa, st.p3, &|m| {
                d.hpc.compress_bled(m, w_hp, ip.tt_out_k, p25, i.hpc_flow_capacity, i.hpc_efficiency, hp_bleed).pt_out_pa
            });
            // Plenums: each pressure advanced implicitly in its own
            // outflow's slope. Nozzle and turbine flows are very steep in
            // pressure close to ambient (flow ~ sqrt(dp)), which an explicit
            // step turns into chatter at low power; linearised about their
            // own slope, each plenum settles instead. Temperatures are held
            // over the sub-step.
            let loss = 1.0 - COMBUSTOR_PRESSURE_LOSS_FRAC;
            let hpt_flow = |p3: f64, p44: f64| d.hpt.expand(p3 * loss, c.tt4_k, p44, w_hp, i.turbine_efficiency).mdot_kg_s;
            let ipt_flow = |p44: f64, p45: f64| d.ipt.expand(p44, hpt.tt_out_k, p45, w_ip, i.turbine_efficiency).mdot_kg_s;
            let lpt_flow = |p45: f64, p5: f64| d.lpt.expand(p45, ipt.tt_out_k, p5, w_lp, 1.0).mdot_kg_s;
            let core_flow = |p5: f64| nozzle::mass_flow_capacity(lpt.tt_out_k, p5, amb, d.core_nozzle_area_m2, GAMMA_GAS, R_GAS);
            let byp_flow = |p13: f64| {
                nozzle::mass_flow_capacity(f.tt_out_k, p13 * (1.0 - BYPASS_DUCT_LOSS_FRAC), amb, d.bypass_nozzle_area_m2, GAMMA_AIR, R_AIR)
            };
            let step_p = |p: f64, capacitance: f64, net: &dyn Fn(f64) -> f64| {
                let dp = (1e-4 * p).max(1.0);
                let n0 = net(p);
                let slope = (net(p + dp) - n0) / dp;
                p + h * capacitance * n0 / (1.0 - h * capacitance * slope).max(1.0)
            };
            let (m_fan_new, m_ipc_new, m_hpc_new) = (st.m_fan, st.m_ipc, st.m_hpc);
            let (p13, p3, p44, p45, p5) = (st.p13, st.p3, st.p44, st.p45, st.p5);
            let t3_mix = 0.5 * (hp.tt_out_k + c.tt4_k);
            // The handling bleeds dump into the bypass duct.
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
        let mut g = GasPath::new();
        let before = g.state;
        let out = g.step(&design_inputs(&g.design), 0.5);
        let after = g.state;
        for (name, a, b) in [("m_fan", before.m_fan, after.m_fan), ("m_hpc", before.m_hpc, after.m_hpc), ("p3", before.p3, after.p3), ("p45", before.p45, after.p45), ("p5", before.p5, after.p5)] {
            assert!((a - b).abs() / a.abs().max(1.0) < 1e-3, "{name} drifted {a} -> {b}");
        }
        // Turbines drive their compressors through the shafts.
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
    fn a_stage_stalls_when_throttled_below_its_peak() {
        let d = design();
        let (w, t, p) = (omega(N3_DESIGN_RPM), 450.0, 400_000.0);
        let healthy = d.hpc.compress(d.mdot_core_kg_s * 1.0, w, t, p, 1.0, 1.0);
        let throttled = d.hpc.compress(d.mdot_core_kg_s * 0.6, w, t, p, 1.0, 1.0);
        assert!(throttled.stall_margin < 1.0 && healthy.stall_margin >= 1.0, "{} {}", healthy.stall_margin, throttled.stall_margin);
    }
}

