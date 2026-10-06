use super::fluid;

pub const PSI_PA: f64 = 6894.757;

#[derive(Clone, Copy, Debug)]
pub enum Endpoint {
    Node(usize),
    Fixed(f64),
}
impl Endpoint {
    fn pressure(&self, pressures: &[f64]) -> f64 {
        match self {
            Endpoint::Node(i) => pressures[*i],
            Endpoint::Fixed(p) => *p,
        }
    }
}

#[derive(Clone, Debug)]
pub struct Node {
    pub volume_m3: f64,
    pub pressure_pa: f64,
    pub air_fraction_at_1atm: f64,
}
impl Node {
    pub fn new(volume_m3: f64, pressure_pa: f64) -> Self {
        Self { volume_m3: volume_m3.max(1e-9), pressure_pa, air_fraction_at_1atm: 0.0 }
    }
    fn capacitance(&self, gauge_pressure_pa: f64) -> f64 {
        let absolute_pa = (gauge_pressure_pa + fluid::ATM_PA).max(fluid::ATM_PA * 0.05);
        self.volume_m3 / fluid::effective_bulk_modulus_pa(absolute_pa, self.air_fraction_at_1atm)
    }
}

#[derive(Clone, Copy, Debug)]
pub enum Restriction {
    Pipe { diameter_m: f64, length_m: f64 },
    Orifice { full_area_m2: f64, discharge_coefficient: f64 },
}

#[derive(Clone, Debug)]
pub struct Line {
    pub from: Endpoint,
    pub to: Endpoint,
    pub restriction: Restriction,
    pub open_fraction: f64,
    pub leak_area_m2: f64,
}
impl Line {
    pub fn pipe(from: Endpoint, to: Endpoint, diameter_m: f64, length_m: f64) -> Self {
        Self { from, to, restriction: Restriction::Pipe { diameter_m, length_m }, open_fraction: 1.0, leak_area_m2: 0.0 }
    }
    pub fn valve(from: Endpoint, to: Endpoint, full_area_m2: f64, discharge_coefficient: f64) -> Self {
        Self { from, to, restriction: Restriction::Orifice { full_area_m2, discharge_coefficient }, open_fraction: 1.0, leak_area_m2: 0.0 }
    }

    fn base_flow_m3_s(&self, dp_pa: f64, density: f64, dyn_visc_pa_s: f64) -> f64 {
        match self.restriction {
            Restriction::Pipe { diameter_m, length_m } => pipe_flow_signed(dp_pa, diameter_m, length_m, density, dyn_visc_pa_s),
            Restriction::Orifice { full_area_m2, discharge_coefficient } => {
                let area = full_area_m2 * self.open_fraction.clamp(0.0, 1.0);
                orifice_flow_signed(discharge_coefficient, area, dp_pa, density)
            }
        }
    }

    fn leak_flow_from_upstream_m3_s(&self, upstream_gauge_pa: f64, density: f64) -> f64 {
        const LEAK_CD: f64 = 0.61;
        if self.leak_area_m2 <= 0.0 || density <= 0.0 {
            return 0.0;
        }
        let dp = upstream_gauge_pa.max(0.0);
        if dp <= 0.0 {
            return 0.0;
        }
        LEAK_CD * self.leak_area_m2 * (2.0 * dp / density).sqrt()
    }
}

fn orifice_flow_signed(cd: f64, area_m2: f64, dp_pa: f64, density: f64) -> f64 {
    if area_m2 <= 0.0 || density <= 0.0 {
        return 0.0;
    }
    let sign = if dp_pa >= 0.0 { 1.0 } else { -1.0 };
    sign * cd * area_m2 * (2.0 * dp_pa.abs() / density).sqrt()
}

fn pipe_flow_signed(dp_pa: f64, diameter_m: f64, length_m: f64, density: f64, dyn_visc_pa_s: f64) -> f64 {
    if diameter_m <= 0.0 || length_m <= 0.0 || density <= 0.0 || dyn_visc_pa_s <= 0.0 {
        return 0.0;
    }
    let sign = if dp_pa >= 0.0 { 1.0 } else { -1.0 };
    let dp = dp_pa.abs();
    if dp <= 0.0 {
        return 0.0;
    }
    let area = std::f64::consts::PI / 4.0 * diameter_m * diameter_m;
    let q_lam = std::f64::consts::PI * diameter_m.powi(4) * dp / (128.0 * dyn_visc_pa_s * length_m);
    let re_lam = density * q_lam * 4.0 / (std::f64::consts::PI * diameter_m * dyn_visc_pa_s);
    if re_lam <= 2300.0 {
        return sign * q_lam;
    }
    let k = 0.316 * (dyn_visc_pa_s / (density * diameter_m)).powf(0.25) * (length_m / diameter_m) * (density / 2.0);
    let v_turb = (dp / k).powf(1.0 / 1.75);
    let q_turb = v_turb * area;
    if re_lam >= 4000.0 {
        return sign * q_turb;
    }
    let blend = (re_lam - 2300.0) / (4000.0 - 2300.0);
    sign * (q_lam * (1.0 - blend) + q_turb * blend)
}

#[derive(Clone, Copy, Debug, Default)]
pub struct CheckValveFaults {
    pub stuck_open: f64,
    pub stuck_shut: f64,
}
#[derive(Clone, Copy, Debug, Default)]
pub struct CheckValve {
    pub cracking_pa: f64,
}
impl CheckValve {
    pub fn open_fraction(&self, upstream_pa: f64, downstream_pa: f64, faults: &CheckValveFaults) -> f64 {
        let stuck_shut = faults.stuck_shut.clamp(0.0, 1.0);
        let stuck_open = faults.stuck_open.clamp(0.0, 1.0);
        let forward: f64 = if upstream_pa - downstream_pa > self.cracking_pa { 1.0 } else { 0.0 };
        let commanded = forward.max(stuck_open);
        commanded * (1.0 - stuck_shut)
    }
}

#[derive(Clone, Copy, Debug)]
pub struct PriorityValve {
    pub cutoff_pa: f64,
    pub opened_pa: f64,
}
impl PriorityValve {
    pub const A380_CUTOFF_PSI: f64 = 3000.0;
    pub const A380_OPENED_PSI: f64 = 3800.0;
    pub fn a380() -> Self {
        Self { cutoff_pa: Self::A380_CUTOFF_PSI * PSI_PA, opened_pa: Self::A380_OPENED_PSI * PSI_PA }
    }
    pub fn open_fraction(&self, upstream_pa: f64, stuck_fraction: f64, last_open_fraction: f64) -> f64 {
        let span = (self.opened_pa - self.cutoff_pa).max(1.0);
        let healthy = ((upstream_pa - self.cutoff_pa) / span).clamp(0.0, 1.0);
        let stuck = stuck_fraction.clamp(0.0, 1.0);
        healthy * (1.0 - stuck) + last_open_fraction * stuck
    }
}

#[derive(Clone, Copy, Debug)]
pub struct ReliefValve {
    pub cracking_pa: f64,
    pub full_flow_rise_pa: f64,
    pub full_flow_m3_s: f64,
}
impl ReliefValve {
    pub fn flow_m3_s(&self, upstream_pa: f64, crack_low_fraction: f64) -> f64 {
        let crack = self.cracking_pa * (1.0 - 0.5 * crack_low_fraction.clamp(0.0, 1.0));
        (self.full_flow_m3_s * (upstream_pa - crack) / self.full_flow_rise_pa.max(1.0)).clamp(0.0, self.full_flow_m3_s)
    }
}

pub struct FireShutoffValve;
impl FireShutoffValve {
    pub fn open_fraction(commanded_open: f64, stuck_fraction: f64, last_open_fraction: f64) -> f64 {
        let stuck = stuck_fraction.clamp(0.0, 1.0);
        commanded_open.clamp(0.0, 1.0) * (1.0 - stuck) + last_open_fraction * stuck
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum LmvPosition {
    Normal,
    Measure,
}
#[derive(Clone, Copy, Debug)]
pub struct LeakMeasurementValve {
    pub position: LmvPosition,
}
impl LeakMeasurementValve {
    pub const MEASURE_ORIFICE_AREA_M2: f64 = 2.0e-6;
    pub fn main_open_fraction(&self) -> f64 {
        match self.position {
            LmvPosition::Normal => 1.0,
            LmvPosition::Measure => 0.0,
        }
    }
    pub fn measure_open_fraction(&self) -> f64 {
        1.0 - self.main_open_fraction()
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub struct FilterFaults {
    pub clog: f64,
}
#[derive(Clone, Copy, Debug)]
pub struct Filter {
    pub clean_diameter_m: f64,
    pub length_m: f64,
    pub bypass_cracking_pa: f64,
}
impl Filter {
    pub fn effective_diameter_m(&self, faults: &FilterFaults) -> f64 {
        let clog = faults.clog.clamp(0.0, 0.999);
        self.clean_diameter_m * (1.0 - clog).sqrt()
    }
    pub fn bypass_open_fraction(&self, last_step_pressure_drop_pa: f64) -> f64 {
        let span = (self.bypass_cracking_pa * 0.2).max(1.0);
        ((last_step_pressure_drop_pa - self.bypass_cracking_pa) / span).clamp(0.0, 1.0)
    }
}

const SWEEPS: usize = 10;
const BISECT_ITERS: usize = 28;
const PRESSURE_BRACKET_LO_PA: f64 = -300_000.0;
const PRESSURE_BRACKET_HI_PA: f64 = 60_000_000.0;

pub struct Network {
    pub nodes: Vec<Node>,
    pub lines: Vec<Line>,
    adjacency: Vec<Vec<usize>>,
    pub last_imbalance_m3: f64,
}

impl Network {
    pub fn new(nodes: Vec<Node>, lines: Vec<Line>) -> Self {
        let mut adjacency = vec![Vec::new(); nodes.len()];
        for (li, line) in lines.iter().enumerate() {
            if let Endpoint::Node(j) = line.from {
                adjacency[j].push(li);
            }
            if let Endpoint::Node(j) = line.to {
                adjacency[j].push(li);
            }
        }
        Self { nodes, lines, adjacency, last_imbalance_m3: 0.0 }
    }

    pub fn step(&mut self, injections_m3_s: &[f64], density: f64, dyn_visc_pa_s: f64, dt_s: f64) -> f64 {
        self.step_with_pressure_dependent(injections_m3_s, &[], density, dyn_visc_pa_s, dt_s)
    }

    pub fn step_with_pressure_dependent(&mut self, injections_m3_s: &[f64], pressure_dependent: &[Option<&dyn Fn(f64) -> f64>], density: f64, dyn_visc_pa_s: f64, dt_s: f64) -> f64 {
        let dt = dt_s.max(1e-6);
        let n = self.nodes.len();
        let p_old: Vec<f64> = self.nodes.iter().map(|nd| nd.pressure_pa).collect();
        let cap: Vec<f64> = self.nodes.iter().enumerate().map(|(i, nd)| nd.capacitance(p_old[i])).collect();
        let mut pressures: Vec<f64> = p_old.clone();

        for _ in 0..SWEEPS {
            for i in 0..n {
                let residual = |p_i: f64| -> f64 {
                    let mut q = injections_m3_s.get(i).copied().unwrap_or(0.0);
                    if let Some(Some(f)) = pressure_dependent.get(i) {
                        q += f(p_i);
                    }
                    for &li in &self.adjacency[i] {
                        q += Self::node_line_contribution(&self.lines[li], i, p_i, &pressures, density, dyn_visc_pa_s);
                    }
                    cap[i] * (p_i - p_old[i]) / dt - q
                };
                let (mut lo, mut hi) = (PRESSURE_BRACKET_LO_PA, PRESSURE_BRACKET_HI_PA);
                let r_lo = residual(lo);
                let r_hi = residual(hi);
                if r_lo > 0.0 {
                    pressures[i] = lo;
                    continue;
                }
                if r_hi < 0.0 {
                    pressures[i] = hi;
                    continue;
                }
                for _ in 0..BISECT_ITERS {
                    let mid = 0.5 * (lo + hi);
                    if residual(mid) <= 0.0 {
                        lo = mid;
                    } else {
                        hi = mid;
                    }
                }
                pressures[i] = 0.5 * (lo + hi);
            }
        }

        let mut imbalance_m3 = 0.0;
        for i in 0..n {
            let mut q = injections_m3_s.get(i).copied().unwrap_or(0.0);
            if let Some(Some(f)) = pressure_dependent.get(i) {
                q += f(pressures[i]);
            }
            for &li in &self.adjacency[i] {
                q += Self::node_line_contribution(&self.lines[li], i, pressures[i], &pressures, density, dyn_visc_pa_s);
            }
            imbalance_m3 += q * dt - cap[i] * (pressures[i] - p_old[i]);
        }
        self.last_imbalance_m3 = imbalance_m3;

        let mut leaked_m3 = 0.0;
        for line in &self.lines {
            if line.leak_area_m2 > 0.0 {
                let p_from = line.from.pressure(&pressures);
                leaked_m3 += line.leak_flow_from_upstream_m3_s(p_from, density) * dt;
            }
        }
        for (i, node) in self.nodes.iter_mut().enumerate() {
            node.pressure_pa = pressures[i];
        }
        leaked_m3
    }

    pub fn line_flow_and_dp(&self, line_index: usize, density: f64, dyn_visc_pa_s: f64) -> (f64, f64) {
        let node_pressures: Vec<f64> = self.nodes.iter().map(|n| n.pressure_pa).collect();
        let line = &self.lines[line_index];
        let dp = line.from.pressure(&node_pressures) - line.to.pressure(&node_pressures);
        (line.base_flow_m3_s(dp, density, dyn_visc_pa_s), dp)
    }

    fn node_line_contribution(line: &Line, i: usize, p_i: f64, pressures: &[f64], density: f64, dyn_visc_pa_s: f64) -> f64 {
        let is_from = matches!(line.from, Endpoint::Node(j) if j == i);
        let is_to = matches!(line.to, Endpoint::Node(j) if j == i);
        if !is_from && !is_to {
            return 0.0;
        }
        let p_from = if is_from { p_i } else { line.from.pressure(pressures) };
        let p_to = if is_to { p_i } else { line.to.pressure(pressures) };
        let dp = p_from - p_to;
        let q_line = line.base_flow_m3_s(dp, density, dyn_visc_pa_s);
        if is_from {
            let q_leak = line.leak_flow_from_upstream_m3_s(p_from, density);
            -q_line - q_leak
        } else {
            q_line
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use super::fluid::{density_kg_m3, dynamic_viscosity_pa_s};

    fn props() -> (f64, f64) {
        (density_kg_m3(60.0), dynamic_viscosity_pa_s(60.0))
    }

    #[test]
    fn flow_moves_from_high_to_low_pressure_pipe() {
        let (density, visc) = props();
        let nodes = vec![Node::new(1e-4, 20e6), Node::new(1e-4, 0.0)];
        let lines = vec![Line::pipe(Endpoint::Node(0), Endpoint::Node(1), 0.01, 1.0)];
        let mut net = Network::new(nodes, lines);
        for _ in 0..50 {
            net.step(&[0.0, 0.0], density, visc, 0.01);
        }
        assert!(net.nodes[0].pressure_pa.is_finite() && net.nodes[1].pressure_pa.is_finite());
        assert!((net.nodes[0].pressure_pa - net.nodes[1].pressure_pa).abs() < 1e4, "should have equalised: {:?}", net.nodes.iter().map(|n| n.pressure_pa).collect::<Vec<_>>());
        assert!(net.nodes[0].pressure_pa < 20e6 && net.nodes[0].pressure_pa > 0.0);
    }

    #[test]
    fn a_constant_supply_against_a_fixed_return_settles_to_a_stable_pressure() {
        let (density, visc) = props();
        let nodes = vec![Node::new(2e-5, 0.0)];
        let lines = vec![Line::valve(Endpoint::Node(0), Endpoint::Fixed(0.0), 1e-6, 0.7)];
        let mut net = Network::new(nodes, lines);
        let supply = 1e-4;
        let mut last = 0.0;
        for step in 0..2000 {
            net.step(&[supply], density, visc, 0.005);
            let p = net.nodes[0].pressure_pa;
            assert!(p.is_finite(), "step {step}: pressure went non-finite");
            assert!(p < 200e6, "step {step}: pressure ran away to {p}");
            last = p;
        }
        assert!(last > 0.0, "should have pressurised against the return orifice");
    }

    #[test]
    fn a_pressure_dependent_injection_keeps_a_stiff_node_off_the_bracket_where_a_frozen_one_pins_it() {
        let (density, visc) = props();
        let huge_supply = 1.0;
        let nodes = vec![Node::new(1e-4, 0.0)];
        let lines = vec![Line::valve(Endpoint::Node(0), Endpoint::Fixed(0.0), 1e-6, 0.7)];

        let mut frozen = Network::new(nodes.clone(), lines.clone());
        frozen.step(&[huge_supply], density, visc, 0.02);
        assert_eq!(
            frozen.nodes[0].pressure_pa, PRESSURE_BRACKET_HI_PA,
            "sanity check: a frozen constant supply this large really does pin to the bracket ceiling"
        );

        let mut implicit = Network::new(nodes, lines);
        let compensator = |p_i: f64| -> f64 {
            let full = 100.0 * PSI_PA;
            let zero = 200.0 * PSI_PA;
            (huge_supply * (1.0 - (p_i - full) / (zero - full)).clamp(0.0, 1.0)).max(0.0)
        };
        let pressure_dependent: [Option<&dyn Fn(f64) -> f64>; 1] = [Some(&compensator)];
        implicit.step_with_pressure_dependent(&[0.0], &pressure_dependent, density, visc, 0.02);
        let p = implicit.nodes[0].pressure_pa;
        assert!(p.is_finite());
        assert!(p < 200.0 * PSI_PA + 1.0, "the implicit compensator must cap the node near its own destroke curve, not the solver's bracket: {p} Pa");
        assert!(p > PRESSURE_BRACKET_LO_PA, "must not have undershot either: {p} Pa");
    }

    #[test]
    fn a_leak_reduces_steady_state_pressure_and_is_reported() {
        let (density, visc) = props();
        let build = |leak_area: f64| {
            let nodes = vec![Node::new(2e-5, 0.0)];
            let mut line = Line::valve(Endpoint::Node(0), Endpoint::Fixed(0.0), 1e-6, 0.7);
            line.leak_area_m2 = leak_area;
            Network::new(nodes, vec![line])
        };
        let mut healthy = build(0.0);
        let mut leaking = build(2e-6);
        let mut leaked_total = 0.0;
        for _ in 0..1500 {
            healthy.step(&[5e-5], density, visc, 0.005);
            leaked_total += leaking.step(&[5e-5], density, visc, 0.005);
        }
        assert!(leaking.nodes[0].pressure_pa < healthy.nodes[0].pressure_pa, "a leaking section should hold less pressure");
        assert!(leaked_total > 0.0, "the leak must be reported for reservoir bookkeeping");
    }

    #[test]
    fn line_flow_and_dp_reports_the_converged_state() {
        let (density, visc) = props();
        let nodes = vec![Node::new(1e-4, 0.0), Node::new(1e-4, 0.0)];
        let lines = vec![Line::pipe(Endpoint::Node(0), Endpoint::Node(1), 0.01, 1.0)];
        let mut net = Network::new(nodes, lines);
        for _ in 0..50 {
            net.step(&[1e-4, 0.0], density, visc, 0.01);
        }
        let (flow, dp) = net.line_flow_and_dp(0, density, visc);
        assert!(flow > 0.0, "flow should run node 0 -> node 1");
        assert!(dp > 0.0);
    }

    #[test]
    fn no_nan_at_rest_or_dt_zero() {
        let nodes = vec![Node::new(1e-4, 0.0), Node::new(1e-4, 0.0)];
        let lines = vec![Line::pipe(Endpoint::Node(0), Endpoint::Node(1), 0.01, 1.0)];
        let mut net = Network::new(nodes, lines);
        let leaked = net.step(&[0.0, 0.0], 1000.0, 0.02, 0.0);
        assert!(leaked.is_finite());
        for n in &net.nodes {
            assert!(n.pressure_pa.is_finite());
        }
    }

    #[test]
    fn check_valve_blocks_reverse_flow() {
        let cv = CheckValve { cracking_pa: 5000.0 };
        let faults = CheckValveFaults::default();
        assert_eq!(cv.open_fraction(1_000_000.0, 0.0, &faults), 1.0);
        assert_eq!(cv.open_fraction(0.0, 1_000_000.0, &faults), 0.0);
        assert_eq!(cv.open_fraction(1000.0, 0.0, &faults), 0.0, "below cracking pressure it stays shut");
    }

    #[test]
    fn a_stuck_open_check_valve_loses_its_reverse_block() {
        let cv = CheckValve { cracking_pa: 5000.0 };
        let stuck = CheckValveFaults { stuck_open: 1.0, ..Default::default() };
        assert_eq!(cv.open_fraction(0.0, 1_000_000.0, &stuck), 1.0);
    }

    #[test]
    fn priority_valve_matches_a380_cutoff_and_opened_pressures() {
        let pv = PriorityValve::a380();
        assert_eq!(pv.open_fraction(pv.cutoff_pa, 0.0, 0.0), 0.0);
        assert_eq!(pv.open_fraction(pv.opened_pa, 0.0, 0.0), 1.0);
        let mid = pv.open_fraction((pv.cutoff_pa + pv.opened_pa) / 2.0, 0.0, 0.0);
        assert!((mid - 0.5).abs() < 1e-9);
    }

    #[test]
    fn a_seized_priority_valve_holds_its_last_position() {
        let pv = PriorityValve::a380();
        let held = pv.open_fraction(pv.cutoff_pa, 1.0, 0.42);
        assert_eq!(held, 0.42);
    }

    #[test]
    fn relief_valve_dumps_flow_only_above_cracking_and_a_weak_spring_cracks_early() {
        let rv = ReliefValve { cracking_pa: 5000.0 * PSI_PA, full_flow_rise_pa: 10.0 * PSI_PA, full_flow_m3_s: 1e-3 };
        assert_eq!(rv.flow_m3_s(4000.0 * PSI_PA, 0.0), 0.0);
        assert!((rv.flow_m3_s(5010.0 * PSI_PA, 0.0) - 1e-3).abs() < 1e-9);
        assert!(rv.flow_m3_s(4800.0 * PSI_PA, 1.0) > 0.0, "a weak spring should already be dumping here");
    }

    #[test]
    fn fire_shutoff_valve_holds_last_position_when_stuck() {
        assert_eq!(FireShutoffValve::open_fraction(0.0, 1.0, 1.0), 1.0);
        assert_eq!(FireShutoffValve::open_fraction(1.0, 0.0, 0.0), 1.0);
        assert_eq!(FireShutoffValve::open_fraction(0.0, 0.0, 0.0), 0.0);
    }

    #[test]
    fn leak_measurement_valve_paths_are_mutually_exclusive() {
        let normal = LeakMeasurementValve { position: LmvPosition::Normal };
        assert_eq!(normal.main_open_fraction(), 1.0);
        assert_eq!(normal.measure_open_fraction(), 0.0);
        let measuring = LeakMeasurementValve { position: LmvPosition::Measure };
        assert_eq!(measuring.main_open_fraction(), 0.0);
        assert_eq!(measuring.measure_open_fraction(), 1.0);
    }

    #[test]
    fn a_clogging_filter_shrinks_its_effective_bore_and_then_bypasses() {
        let filter = Filter { clean_diameter_m: 0.006, length_m: 0.05, bypass_cracking_pa: 50.0 * PSI_PA };
        let clean = filter.effective_diameter_m(&FilterFaults::default());
        let clogged = filter.effective_diameter_m(&FilterFaults { clog: 0.75 });
        assert!(clogged < clean);
        assert!((clogged / clean - 0.5).abs() < 1e-12, "clog 0.75 leaves a quarter of the area, i.e. half the bore, got {}", clogged / clean);
        let half_clogged = filter.effective_diameter_m(&FilterFaults { clog: 0.5 });
        assert!((half_clogged / clean - 0.5f64.sqrt()).abs() < 1e-12, "clog 0.5 leaves half the area, i.e. 1/sqrt(2) of the bore, got {}", half_clogged / clean);
        assert_eq!(filter.bypass_open_fraction(0.0), 0.0);
        assert_eq!(filter.bypass_open_fraction(filter.bypass_cracking_pa + 100.0 * PSI_PA), 1.0);
        let mid = filter.bypass_open_fraction(filter.bypass_cracking_pa);
        assert!((0.0..=1.0).contains(&mid));
    }

    #[test]
    fn pipe_flow_is_laminar_at_low_pressure_and_turbulent_at_high() {
        let (density, visc) = props();
        let d = 0.01;
        let l = 1.0;
        let dp_small = 1000.0;
        let q_small = pipe_flow_signed(dp_small, d, l, density, visc);
        let expected_lam = std::f64::consts::PI * d.powi(4) * dp_small / (128.0 * visc * l);
        assert!((q_small - expected_lam).abs() / expected_lam < 1e-9);
        let dp_large = 5000.0 * PSI_PA;
        let q_large = pipe_flow_signed(dp_large, d, l, density, visc);
        let would_be_laminar = expected_lam * dp_large / dp_small;
        assert!(q_large < would_be_laminar, "turbulent flow should undershoot the laminar extrapolation");
        assert!(q_large > 0.0 && q_large.is_finite());
    }
}
