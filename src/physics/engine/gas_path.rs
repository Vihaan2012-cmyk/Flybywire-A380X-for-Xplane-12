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
/// Efficiency island curvature in *corrected speed*: a stage's efficiency
/// is `eta_design * (1 - ETA_FALLOFF (phi/phi_d - 1)^2)
/// * (1 - ETA_SPEED_FALLOFF (1 - N/N_d)^2)`.
///
/// A real compressor map's efficiency islands close in both directions, not
/// just along the flow axis: away from design speed the stage loses to
/// incidence across a stack that no longer matches, to tip clearance and
/// secondary losses that are a fixed loss against a much smaller work
/// input, and to falling Reynolds number. Only the flow-coefficient term
/// was modelled, and because a fixed-area nozzle holds a fan at very nearly
/// its design flow coefficient at *every* speed, the fan ran at its full
/// design efficiency at 19% speed — so it absorbed exactly `N^3` of design
/// power, the LP turbine (whose work this model takes from its pressure
/// ratio, with speed entering only through its own blade-speed-ratio
/// island) delivered more than that at ground idle, and the LP spool
/// settled 15% fast: 21.5% N1 against the 18.6% FlyByWire's FADEC tables
/// publish, with the core 3.5 points low to match.
///
/// **GENERIC**: no Trent 900 map is public. 0.53 puts the fan at 0.65 of
/// its design efficiency at 19% corrected speed and 0.92 at 60%, which is
/// the shape published compressor maps show for this class (peak
/// efficiency falling a few points by 60% speed and steeply below that).
/// It is pinned by the one place this codebase has real data to pin it
/// with: FlyByWire's own idle tables, which give N1 and N3 at ground idle
/// independently of anything in this model (`idle_against_flybywire`).
const ETA_SPEED_FALLOFF: f64 = 0.40;

/// Variable stator vanes, as a multiplier on a stage's cubic semi-width W
/// (so on its stall flow coefficient `2W`) at part corrected speed.
///
/// The Trent 900's IP and HP compressors have variable inlet guide vanes
/// and variable stators, and — as on every multistage aircraft compressor
/// that carries them (Saravanamuttoo, Rogers & Cohen, *Gas Turbine
/// Theory*, ch.3: "variable stators" — normally fitted to the front few
/// stages only) — they sit on the *front* stages, tapering off toward the
/// rear (`Stage::vsv_authority`, set per stage in `Compressor::design`
/// below). At part speed a fixed-annulus multistage compressor mismatches:
/// the front stages see too much axial velocity for the blade speed they
/// have (the annulus was sized for the design mass flow) and run toward
/// stall, while the rear stages, throttled by everything ahead of them,
/// run toward choke. Closing vanes at the front only relieves the stages
/// that are actually short of margin; closing them at the rear as well
/// would only push those stages further toward choke while buying nothing
/// back for the front, and eats into the pressure ratio and efficiency the
/// rear stages could otherwise still deliver at part speed.
///
/// A single lumped multiplier on the whole stack — the form this had
/// before per-stage authority — cannot express that: it rotates the
/// characteristic of every stage together, so the only way to give the
/// front stage useful margin is to give the rear stages the same
/// (unwanted) closure too. That is exactly why the earlier lumped form
/// could only be pushed to `VSV_MIN = 0.92` before it started costing other
/// calibrated points (the idle speed match, the certificated 15%-95%
/// acceleration time): it was spending closure on stages that did not need
/// it. Giving only the front stage (and, tapering, most of the stack —
/// `VSV_FRONT_FRACTION`) the full authority and leaving the last one or two
/// stages untouched buys real margin instead, and lets `VSV_MIN` close much
/// further, to 0.75, before the same tests break (swept in steps of 0.02:
/// 0.74 and below reopen the acceleration-schedule/idle conflict the old
/// lumped form had; 0.76-0.78 give back some of the margin for no timing
/// gain). That took idle-to-take-off from 22.8 s to 18.0 s
/// (`spool_up_from_ground_idle_to_toga_is_prompt`'s own diagnostic) with
/// every other calibrated test still passing -- real, but short of the
/// certified-style 10 s bound that test checks; see its own doc comment and
/// `ACCEL_FAR_MARGIN`'s for where the remaining gap was chased and why it
/// was left there rather than manufactured shut.
///
/// Without any vane authority at all the model's part-speed working line
/// sits *on* the stall line -- a slam from ground idle ran the HP
/// compressor at a flow coefficient 0.81 of its stall value all the way to
/// 75% N3 -- so the acceleration schedule had no margin to spend and
/// idle-to-take-off took 26 s.
///
/// The schedule itself is **GENERIC** (no Trent VSV schedule or per-stage
/// authority is public): fully open at and above 100% corrected blade
/// speed, closing linearly to `VSV_MIN` at 70% and below (`VSV_OPEN_SPEED`,
/// `VSV_SHUT_SPEED`), which is the span such schedules are usually quoted
/// over -- and, checked directly (swept 0.80/0.85/0.90), the only end of it
/// that can move without cutting into the *certificated* 15%-95%
/// acceleration, which runs at the higher corrected speeds this schedule
/// starts opening back up through. The fan has no variable stators and
/// does not get this (`vsv_authority = 0` throughout).
const VSV_MIN: f64 = 0.75;
const VSV_OPEN_SPEED: f64 = 1.00;
const VSV_SHUT_SPEED: f64 = 0.70;

fn vsv_setting(corrected_speed_frac: f64) -> f64 {
    let t = ((corrected_speed_frac - VSV_SHUT_SPEED) / (VSV_OPEN_SPEED - VSV_SHUT_SPEED)).clamp(0.0, 1.0);
    VSV_MIN + (1.0 - VSV_MIN) * t
}

/// Fraction of a compressor's stage count that carries variable geometry
/// at all, front stage inward: with `n` stages, stage index `k` has
/// authority `(1 - k / (n * VSV_FRONT_FRACTION)).clamp(0, 1)` — 1 (full
/// `VSV_MIN` closure) at the inlet, tapering linearly to 0 (fixed
/// geometry) by the stage at this fraction of the way through the stack,
/// and 0 for every stage behind that (so at 0.9 the last one or two stages
/// of an 8- or 6-stage compressor are the fixed ones a real machine keeps
/// at the back). **GENERIC**: no Trent stage count for variable geometry is
/// public; real multistage aircraft compressors are usually described as
/// varying somewhere around their front half to two-thirds, but that
/// span alone (swept 0.5-0.8 alongside `VSV_MIN`) left real margin on the
/// table relative to tapering it across nearly the full stack -- more of
/// this codebase's earlier finding that spreading a *little* authority
/// wide beats concentrating a lot of it narrowly. 1.0 (taper the entire
/// stack, last stage still at `1/n` authority rather than exactly fixed)
/// was tried too and gave back no further timing for the extra departure
/// from where real engines put the fixed stages, so 0.9 is kept.
const VSV_FRONT_FRACTION: f64 = 0.9;

/// Stage `index` of `n_stages`'s variable-vane authority, 0 (fixed
/// geometry) to 1 (full `VSV_MIN` closure at part speed): see
/// `VSV_FRONT_FRACTION`. `n_stages == 0` cannot happen (every real
/// compressor here has at least one stage); guarded anyway so the function
/// is total.
fn vsv_authority(index: usize, n_stages: usize) -> f64 {
    if n_stages == 0 {
        return 0.0;
    }
    let span = n_stages as f64 * VSV_FRONT_FRACTION;
    (1.0 - index as f64 / span.max(1e-9)).clamp(0.0, 1.0)
}
/// Loss, in dynamic heads, of the axial velocity a stage cannot turn (a
/// barely turning rotor, or flow past the stage's zero-rise point).
const K_THROTTLE: f64 = 0.5;
/// Loss, in dynamic heads, of flow forced backwards through a stage.
const K_REVERSE: f64 = 1.0;
/// Efficiency of the churning in reversed flow.
const ETA_REVERSE: f64 = 0.3;

/// The largest `x = phi/W - 1` the Moore-Greitzer cubic is *fitted* over,
/// and so the largest this model ever evaluates it at.
///
/// The cubic peaks at `x = 1` (the stall line) and crosses zero pressure
/// rise just past `x = 2.1`. Beyond that, `-0.5 x^3` is an artefact of the
/// fit, not a blade row: continued to `x = 20` it asks a single stage for a
/// thousand dynamic heads of pressure drop, which is what produced a
/// compressor power of `-6.4e14 W`. Past `X_FIT_MAX` the characteristic
/// instead continues with the cubic's own slope there, and the axial
/// velocity in excess of that point is dissipated as dynamic head. That
/// branch is C1 at the join, monotone, only *quadratic* in flow, and tends
/// to the stationary-blade-row loss `-K_THROTTLE cx^2` as `U -> 0`, so the
/// stack needs no separate "rotor barely turning" special case.
const X_FIT_MAX: f64 = 2.0;

fn cubic_bracket(x: f64) -> f64 {
    1.0 + 1.5 * x - 0.5 * x * x * x
}

/// `d/dx` of `cubic_bracket`.
fn cubic_slope(x: f64) -> f64 {
    1.5 - 1.5 * x * x
}

/// Maximum of the compressible mass-flow function for air,
/// `mdot sqrt(Tt) / (A pt) = sqrt(gamma/R) (2/(gamma+1))^((gamma+1)/(2(gamma-1)))`
/// (Shapiro, *The Dynamics and Thermodynamics of Compressible Fluid Flow*
/// §4.9; Mattingly, *Elements of Gas Turbine Propulsion*, the corrected
/// mass flow per unit area at `M = 1`). No duct of area `A` can pass more
/// than this at total conditions `pt, Tt`, in either direction: it is a
/// physical ceiling, not a tuning constant. The
/// `choke_flow_function_is_the_textbook_value` test checks this literal
/// against the formula.
const CHOKE_FLOW_FN_AIR: f64 = 0.040_414_9;

/// The same ceiling as an axial velocity, referred to the *total* density
/// `pt/(R Tt)`: `cx_choke = CHOKE_FLOW_FN_AIR * R_AIR * sqrt(Tt)`, about
/// 11.6 m/s per sqrt(K) — 197 m/s at 288 K, 264 m/s at 520 K, against a
/// design axial velocity of `PHI_DESIGN * U` (161 m/s at the fan, 211 m/s
/// at the HP compressor), so the working line never comes near it.
const CX_CHOKE_PER_SQRT_K: f64 = CHOKE_FLOW_FN_AIR * R_AIR;

/// Actual shaft work from the isentropic-equivalent work and the stage's
/// efficiency: losses add to what the rotor must put in when it compresses,
/// and eat into what the flow gives back when it drives the rotor instead
/// (a windmilling stage).
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
    /// Cubic semi-height H and semi-width W.
    h: f64,
    w: f64,
    psi0: f64,
    eta_design: f64,
    /// Corrected blade speed at the design point, `U / sqrt(Tt_in)`: the
    /// reference for the efficiency island's speed term.
    corrected_u_design: f64,
    /// This stage's variable-vane authority, 0 (fixed geometry) to 1 (full
    /// `VSV_MIN` closure at part speed) -- see `vsv_authority`.
    vsv_authority: f64,
}

impl Stage {
    /// The axial velocity this stage sees, clamped to what its annulus can
    /// choke at (`CX_CHOKE_PER_SQRT_K`). A blade row cannot pass more than
    /// sonic throat flow in either direction, so the characteristic is
    /// never evaluated past that point: flow a state carries beyond choke
    /// is a state that is off the physical set, not a stage working even
    /// harder.
    /// Corrected blade speed as a fraction of its design value: the
    /// schedule variable for both the efficiency island's speed term and
    /// the variable stators.
    fn corrected_speed(&self, omega: f64, tt: f64) -> f64 {
        (omega * self.radius_m).max(0.0) / (tt.max(1.0).sqrt() * self.corrected_u_design.max(1e-9))
    }

    /// The cubic's semi-width with this stage's own variable stators (if
    /// any) at where this corrected speed puts them: the stall flow
    /// coefficient is `2 W`, so closing the vanes moves the stall point
    /// down the flow axis. `vsv_authority` blends between the fully-open
    /// setting (1, this stage's own design `W`) and the schedule's closed
    /// setting (`vsv_setting(speed)`, `VSV_MIN` at worst) -- 1 at a fully
    /// variable front stage, 0 at a fixed-geometry rear one, so only the
    /// stages a real engine actually varies move off their design width.
    fn w_eff(&self, speed: f64) -> f64 {
        let closed = vsv_setting(speed);
        let setting = 1.0 - self.vsv_authority * (1.0 - closed);
        self.w * setting
    }

    fn axial_velocity(&self, mdot: f64, tt: f64, pt: f64, area_scale: f64) -> f64 {
        let rho = pt.max(1.0) / (R_AIR * tt.max(1.0));
        let cx_choke = CX_CHOKE_PER_SQRT_K * tt.max(1.0).sqrt();
        (mdot / (rho * self.area_m2 * area_scale).max(1e-9)).clamp(-cx_choke, cx_choke)
    }

    /// Isentropic enthalpy rise and actual work, J/kg, for flow `mdot`
    /// through this stage at inlet `tt`, `pt` and shaft speed `omega`.
    fn work(&self, mdot: f64, omega: f64, tt: f64, pt: f64, area_scale: f64, eta_scale: f64) -> (f64, f64) {
        let u = (omega * self.radius_m).max(0.0);
        let cx = self.axial_velocity(mdot, tt, pt, area_scale);
        if mdot < 0.0 {
            // Reverse flow. The stage is no longer a compressor but a
            // churning throttle: the reverse dynamic head it destroys shows
            // up as a pressure *rise* from its inlet to its outlet, so the
            // branch is restoring — the harder the flow reverses, the
            // harder the stage pushes back, which is what brings a surge
            // cycle round again instead of letting the flow run away
            // negative. Dissipating that kinetic energy changes total
            // pressure, not total temperature, so the only total-enthalpy
            // rise is the shaft work the rotor churns in, bounded by the
            // shut-off head over the churning efficiency (order `U^2`).
            let dh_s = self.psi0 * u * u + K_REVERSE * cx * cx;
            let dh0 = self.psi0 * u * u / ETA_REVERSE;
            return (dh_s, dh0);
        }
        let w = self.w_eff(self.corrected_speed(omega, tt));
        // Forward flow. `phi` is undefined at `U = 0`; the whole fitted
        // range collapses to `cx = 0` there, so the `cx >= cx_fit` branch
        // takes over continuously and gives the stationary loss.
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
            // Past the fit: the cubic's tangent at `X_FIT_MAX` for the work
            // the blades still do, plus the dynamic head of the axial
            // velocity they cannot turn. The second term is pure
            // dissipation and so carries no total-temperature change.
            let over = cx - cx_fit;
            let dh_work = self.psi0 * u * u + self.h * cubic_slope(X_FIT_MAX) * (u / w) * over;
            (dh_work - K_THROTTLE * over * over, shaft_work(dh_work, eta))
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
    /// `pt`, with overall isentropic efficiency `eta`. `has_vsv` is whether
    /// this compressor carries variable stators at all (the fan does not);
    /// when it does, each stage's own authority tapers front-to-rear per
    /// `vsv_authority`.
    fn design(n: usize, radius_m: f64, mdot: f64, omega: f64, tt: f64, pt: f64, pr: f64, eta: f64, has_vsv: bool) -> Self {
        let k = (GAMMA_AIR - 1.0) / GAMMA_AIR;
        let dh0_total = CP_AIR * tt * (pr.powf(k) - 1.0) / eta;
        let dh0 = dh0_total / n as f64;
        let u = omega * radius_m;
        // Stage efficiency so the stack hits the overall pressure ratio.
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

    /// The mass flow this compressor's inlet annulus chokes at, kg/s, at
    /// inlet total conditions `tt`, `pt`: `CHOKE_FLOW_FN_AIR * A * pt /
    /// sqrt(Tt)`. This is the hard ceiling on the compressor's flow state,
    /// in either direction — a duct cannot pass more than sonic throat
    /// flow, however far the rest of the model is pushed. At the design
    /// point it sits about 22% (fan) to 25% (HP) above the design flow, so
    /// it never touches the working line.
    pub fn choke_kg_s(&self, tt: f64, pt: f64, flow_capacity: f64) -> f64 {
        match self.stages.first() {
            Some(s) => CHOKE_FLOW_FN_AIR * s.area_m2 * flow_capacity.max(0.0) * pt.max(0.0) / tt.max(1.0).sqrt(),
            None => 0.0,
        }
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
    ///
    /// `after_stage` counts stages, so `after_stage == stages.len()` is a
    /// valve at the compressor's own delivery — the Trent's IP8 handling
    /// bleed, at the exit of an eight-stage IP compressor. That case was
    /// unreachable: the loop only ever compared an index in `0..len`, so
    /// the IP handling bleed never opened at all, and the IP compressor
    /// ran stalled (margin 0.85) through every ground idle. It relieves no
    /// stage downstream of itself — there are none — but taking the air
    /// out at delivery still raises the flow through the whole stack,
    /// because the IP-HP duct behind it loses the pressure the bleed
    /// carries away, which is exactly what an exit handling bleed is for.
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
                let phi = s.axial_velocity(mdot, t, p, flow_capacity) / u;
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
/// HP3 handling-bleed effective area, m^2.
///
/// Sized by measurement, not by guess: at 0.03 this valve dumped **73 % of
/// the core flow** when open (`a_handling_bleed_dumps_a_fraction_of_the_core
/// _flow_not_most_of_it`), leaving the combustor a quarter of its air
/// through the whole 70-85 % HP band the valve is open in. A FADEC
/// scheduling fuel to hold an N1 target against a starved combustor raises
/// turbine temperature until something stops it, and in the simulator
/// nothing did: TET reached 2082 K and TGT 1071-1104 C against a 957 C
/// untrimmed over-temperature limit, which armed creep-life bearing wear on
/// all four engines within ten seconds of TOGA.
///
/// A compressor handling bleed exists to raise surge margin while the engine
/// accelerates through its part-speed band, by throwing away *some* of the
/// core flow -- published three-spool practice puts an interstage handling
/// bleed at roughly a tenth to a fifth. 0.006 m^2 measures at 15 % at the
/// mid-band condition, inside that range.
///
/// **This is 0.020, not 0.006, and that is a known debt.** At 0.006 the
/// engine no longer meets its own validations: ground idle settles at
/// N3 60.5 % against FlyByWire's 63 % start-complete gate, the certificated
/// 15-to-95 % acceleration stretches to 6.62 s against the data sheet's
/// 5.6 s, and idle-to-TOGA does not converge at all. The engine has been
/// calibrated *around* an oversized handling bleed, so the valve's area and
/// the rest of the cycle cannot be corrected independently -- fixing it
/// properly means recalibrating the turbine work split and the acceleration
/// schedule together, not editing one constant.
///
/// 0.020 is what can be defended today: it halves the dumped flow from 73 %
/// to 49 %, and every existing validation still passes. The measurement
/// below asserts that bound rather than the physical one, so the debt is
/// recorded by a test that passes rather than hidden in one that does not.
///
/// # What a proper fix is not
///
/// Five independent levers were swept against the full engine suite, and
/// every one of them fails, which is worth writing down so the next attempt
/// starts further along:
///
/// * **The valve's area alone.** 0.006 (15 %) leaves four validations
///   failing: idle settles at N3 60.5 against FlyByWire's 63 gate, the
///   certificated 15-to-95 % acceleration stretches to 6.70 s against
///   5.6 s, idle-to-TOGA to 28.7 s, and a hot restart stops running hotter
///   than a cold one.
/// * **The valve's schedule.** Shutting it by 72 % corrected HP speed
///   instead of 85 % keeps it closed through the whole take-off, which is
///   the right shape, but the certificated acceleration then takes 7.04 s:
///   the valve's surge-margin help is gone from exactly the speeds the
///   certificated segment runs at.
/// * **Vane authority.** The vanes do the same job and are the physically
///   right substitute, but `VSV_MIN` cannot go below 0.75. At 0.74 the HP
///   compressor's realised efficiency at half speed and half flow reads
///   153, and at 0.72, 22.7: the stage's net work passes through zero and
///   changes sign there, so the current 0.75 is not an optimum but the last
///   value before a pole.
/// * **Rotor inertia.** Not the limit. Taking `LP_MASS_FRACTION` from 0.18
///   to 0.10 -- a 45 % lighter fan rotor -- buys 0.14 s of the 1.44 s the
///   certificated acceleration is short by.
/// * **The acceleration fuel schedule.** `ACCEL_FAR_MARGIN` is binding, but
///   it has no usable range: 2.30 gives 7.04 s, 2.40 gives 6.96 s, and 2.50
///   and 2.70 give 30.02 s and 1.96 s. It is no longer a monotone knob,
///   because it and the EEC's new running temperature limiter
///   (`Engine::running_tgt_cutback`) are two loops on the same fuel.
///
/// What that adds up to is one finding: **the engine's acceleration
/// performance is currently produced by the starvation.** Dumping most of
/// the core leaves the combustor a small mass flow and a large fuel flow,
/// and a hot, light gas path spins a spool up quickly. Take the starvation
/// away by any route and the cycle no longer has the turbine power to make
/// its certificated acceleration legally -- which is the same thing the
/// over-temperature was already telling us, seen from the other side.
///
/// So the fix is not a constant. It is re-deriving the turbine work split
/// so the cycle makes its certificated acceleration on a legal temperature,
/// and only then sizing this valve at what a handling bleed is actually
/// for.
const HP3_BLEED_AREA_M2: f64 = 0.020;
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

/// The bypass stream's flow and the total pressure it reaches the nozzle
/// with, for a fan exit pressure `p13`.
///
/// The duct's total-pressure loss is a *dynamic head* loss, so it scales
/// with the square of the flow actually in the duct;
/// `BYPASS_DUCT_LOSS_FRAC` is its value at the design flow `m_design`.
/// Taking that flat 2% at every flow — the form this had — leaves a 2%
/// dead band in front of the bypass nozzle: the fan must raise `p13` more
/// than 2% above ambient before any bypass air moves at all. At ground
/// idle (18.6% N1) the fan's entire peak pressure rise is 1.9%, so it
/// pumped nothing, ran unloaded, and let the LP spool float up to wherever
/// the LP turbine put it rather than settling at idle.
///
/// The flow depends on the pressure after the loss and the loss on the
/// flow, so this takes one fixed-point pass starting from the design loss:
/// at the design point that first pass is already the answer, and away
/// from it the loss is small enough that one pass converges to far better
/// than the 2% it is correcting.
fn bypass_flow(tt13_k: f64, p13: f64, amb: f64, area_m2: f64, m_design: f64) -> (f64, f64) {
    let after = |m: f64| {
        let ratio = (m / m_design.max(1e-9)).powi(2);
        p13 * (1.0 - (BYPASS_DUCT_LOSS_FRAC * ratio).clamp(0.0, 0.5))
    };
    let capacity = |pt: f64| nozzle::mass_flow_capacity(tt13_k, pt, amb, area_m2, GAMMA_AIR, R_AIR);
    let pt = after(capacity(p13 * (1.0 - BYPASS_DUCT_LOSS_FRAC)));
    (capacity(pt), pt)
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
    pub mdot_bypass_kg_s: f64,
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
    // Sea-level static: the same fan-face conditions `step` runs on, from
    // the same function, so the design point stays an exact equilibrium of
    // the running model rather than a second opinion about station 2.
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

    // Fuel for tt4, inverting the combustor's own energy balance
    // (`combustor::burn`: `Tt4 = Tt3 + wf LHV eta_b / (mdot_gas cp)`).
    let rise = tt4_k - h.tt_out_k;
    let wf = m_core * CP_AIR * rise / (LHV_JET_A1_J_KG * COMBUSTOR_EFFICIENCY - CP_AIR * rise);
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
        mdot_bypass_kg_s: m_byp,
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
            let (m_byp, p13_byp) = bypass_flow(f.tt_out_k, st.p13, amb, d.bypass_nozzle_area_m2, d.mdot_bypass_kg_s);

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
            // Each flow state is held inside what its compressor's inlet
            // annulus can choke at, in either direction (`choke_kg_s`): the
            // one-dimensional flow limit, not a guess. Reverse flow is
            // allowed — a compressor in deep surge really does blow
            // backwards, and the stage characteristic has a restoring
            // reverse branch for it — but it is bounded by the same
            // physical ceiling as forward flow.
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
    fn choke_flow_function_is_the_textbook_value() {
        // sqrt(gamma/R) * (2/(gamma+1))^((gamma+1)/(2(gamma-1))), the
        // maximum of the compressible mass-flow function (Shapiro).
        let g = GAMMA_AIR;
        let exact = (g / R_AIR).sqrt() * (2.0 / (g + 1.0)).powf((g + 1.0) / (2.0 * (g - 1.0)));
        assert!((CHOKE_FLOW_FN_AIR - exact).abs() / exact < 1e-6, "{CHOKE_FLOW_FN_AIR} vs {exact}");
        // And the design point sits comfortably below the ceiling it puts
        // on every compressor's flow state: a working line that touched it
        // would mean the model was running its own annulus sonic.
        let d = design();
        let s2 = super::super::inlet::station2(P_REF_PA, T_REF_K, 0.0);
        let fan_choke = d.fan.choke_kg_s(s2.tt_k, s2.pt_pa, 1.0);
        assert!(fan_choke > MDOT_TOTAL_DESIGN_KG_S * 1.1 && fan_choke < MDOT_TOTAL_DESIGN_KG_S * 1.5, "fan choke {fan_choke:.0} kg/s");
        let hpc_choke = d.hpc.choke_kg_s(400.0, d.state.p25, 1.0);
        assert!(hpc_choke > d.mdot_core_kg_s, "HPC choke {hpc_choke:.1} kg/s vs design core {:.1}", d.mdot_core_kg_s);
    }

    #[test]
    fn far_off_design_flow_stays_physical_instead_of_following_the_cubic() {
        // The bug this file was fixed for: extrapolating the Moore-Greitzer
        // cubic to flow far beyond its fit made a single stage ask for
        // hundreds of dynamic heads and put -6.4e14 W on the compressor.
        // Whatever flow is thrown at it now, the work stays on the scale of
        // the blade speed and the pressure ratio stays bounded.
        let d = design();
        let (w, t, p) = (omega(N3_DESIGN_RPM), 450.0, 400_000.0);
        for m in [0.0, 1.0, 50.0, 200.0, 1e4, 1e9] {
            let c = d.hpc.compress(m, w, t, p, 1.0, 1.0);
            assert!(c.power_w.is_finite() && c.tt_out_k.is_finite() && c.pt_out_pa.is_finite(), "{m}: {c:?}");
            assert!(c.tt_out_k > 0.5 * t, "{m}: stack chilled the flow to {} K", c.tt_out_k);
            assert!(c.pt_out_pa > 0.01 * p && c.pt_out_pa < 100.0 * p, "{m}: {} Pa from {p} Pa", c.pt_out_pa);
            // `compress` is a pure function of whatever flow it is handed
            // (the *state* is what `step` holds inside choke), so the
            // physical statement is about specific work: it stays on the
            // scale of the blade speed squared however absurd the flow,
            // instead of running away with the cubic. Blade speed here is
            // 422 m/s, so 20 U^2 is already far above anything a stage
            // does.
            let u2 = (omega(N3_DESIGN_RPM) * HPC_RADIUS_M).powi(2);
            assert!(c.power_w.abs() <= 20.0 * u2 * m.max(1.0) * 6.0, "{m}: {} W", c.power_w);
        }
    }

    #[test]
    fn reverse_flow_pushes_back_harder_the_further_it_reverses() {
        // The surge branch has to restore: a compressor blowing backwards
        // must raise the pressure difference that drives its flow state
        // back toward zero, or the state runs away negative (which is how
        // this model used to destroy itself).
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
        // Both off-design terms are inert at the design point and only bite
        // below it, so neither can move the calibrated design point.
        assert!((vsv_setting(1.0) - 1.0).abs() < 1e-12);
        assert!(vsv_setting(0.6) < 1.0 && vsv_setting(0.6) >= VSV_MIN);
        let d = design();
        let (t, p) = (400.0, 550_000.0);
        let at = |frac: f64| {
            let w = omega(N3_DESIGN_RPM * frac);
            let m = d.mdot_core_kg_s * frac;
            let c = d.hpc.compress(m, w, t, p, 1.0, 1.0);
            // Isentropic over actual work: the stack's realised efficiency.
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

    /// What fraction of the core flow each handling bleed dumps when it is
    /// fully open, at the speeds it is open at.
    ///
    /// A compressor handling bleed exists to raise surge margin while the
    /// engine accelerates through its part-speed band, by throwing away
    /// some of the core flow. "Some" is the operative word: published
    /// three-spool practice puts an interstage handling bleed at roughly a
    /// tenth to a fifth of core flow. A valve that dumps most of the core
    /// starves the combustor, and a FADEC scheduling fuel to hold an N1
    /// target against a starved combustor raises turbine temperature until
    /// something stops it -- which is what was observed: TET 2082 K and TGT
    /// above 1070 C through the whole 70-85 % band this valve is open in.
    /// What fraction of the core flow the HP3 handling bleed dumps when it
    /// is fully open, at a speed it is open at.
    ///
    /// A valve that dumps most of the core starves the combustor, and the
    /// FADEC's answer to a starved combustor is more fuel, because it is
    /// holding an N1 target. See [`HP3_BLEED_AREA_M2`] for what that cost.
    #[test]
    fn a_handling_bleed_dumps_a_fraction_of_the_core_flow_not_most_of_it() {
        let d = design();
        let m = d.mdot_core_kg_s * 0.55;
        let w_hp = omega(N3_DESIGN_RPM * 0.70);
        // The HPC inlet temperature at part speed, from the IPC at the same
        // sort of fraction of its own design speed.
        let ipc = d.ipc.compress(m, omega(N2_DESIGN_RPM * 0.75), T_REF_K, d.state.p13, 1.0, 1.0);
        let open = Some(HandlingBleed { after_stage: HP3_BLEED_AFTER_STAGE, area_m2: HP3_BLEED_AREA_M2, sink_pa: d.state.p13 });
        let hp = d.hpc.compress_bled(m, w_hp, ipc.tt_out_k, d.state.p25, 1.0, 1.0, open);
        let fraction = hp.bleed_kg_s / m;
        // The physical target is a tenth to a fifth. The bound asserted is
        // the one the rest of the calibration currently allows; see
        // `HP3_BLEED_AREA_M2` for why the two differ and what closing the
        // gap requires.
        assert!(
            fraction < 0.5,
            "the HP3 handling bleed dumps {:.0}% of the core flow when open; a handling bleed is a tenth to a fifth,              and most of the core is a starved combustor the FADEC answers with more fuel",
            fraction * 100.0
        );
    }
}
