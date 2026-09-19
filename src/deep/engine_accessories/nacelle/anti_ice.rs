//! Engine (nacelle) anti-ice valve: a hot-bleed-air valve feeding the inlet
//! cowl lip's internal piccolo tube, the standard way large turbofans keep
//! their own inlet from accreting ice that could shed into the fan. Same
//! rate-limited valve model as every other pneumatic valve in this
//! directory (`starting::air_valve`, `airflow_control::bleed_valve`); the
//! physically meaningful outputs are the bled mass flow (an orifice off the
//! compressor delivery the gas path can subtract, exactly like a handling
//! bleed) and the cowl lip's own heat balance, since an anti-ice valve that
//! is open but delivering too little heat (a partially jammed valve, or
//! healthy but simply not enough flow for the icing severity) is a real,
//! physically distinct failure from the valve position disagreeing with
//! its command.
//!
//! No Trent-900/A380 anti-ice valve sizing or cowl thermal mass is public.
//! Every figure below is **GENERIC**, sized so a fully open valve keeps a
//! representative cowl lip comfortably above freezing in representative
//! icing conditions, and a closed valve lets it drift to ambient.

const R_AIR: f64 = 287.05;
const CP_AIR: f64 = 1005.0;

/// Full-open orifice area and discharge coefficient (**GENERIC**, sized so
/// bled flow is a small fraction of a representative HP compressor design
/// flow, consistent with `airflow_control::bleed_valve`'s own sizing
/// approach).
const AREA_M2: f64 = 2.0e-3;
const CD: f64 = 0.65;
/// The piccolo tube vents into the lip cavity at close to ambient static
/// pressure (**GENERIC**: standard sea-level reference, used only as the
/// downstream side of the valve's own orifice pressure drop -- this module
/// does not model the duct network between the valve and the lip).
const DOWNSTREAM_PA: f64 = 101_325.0;
/// Valve travel time, s (**GENERIC**, a pneumatic anti-ice valve).
const TRAVEL_TIME_S: f64 = 2.0;
/// Cowl lip thermal mass, J/K, and conductance to the ambient/icing airflow
/// outside, W/K (**GENERIC**, a thin aluminium/titanium lip skin).
const LIP_CAPACITY_J_K: f64 = 4_000.0;
const LIP_LOSS_W_K: f64 = 350.0;
/// Fraction of the bled heat (mass flow x cp x temperature drop to lip
/// temperature) that actually reaches the lip skin rather than being
/// ducted straight overboard (**GENERIC**).
const HEAT_TRANSFER_EFFECTIVENESS: f64 = 0.4;

#[derive(Clone, Copy, Debug, Default)]
pub struct AntiIceValveFaults {
    /// Mechanically stuck, 0 free .. 1 seized -- reads as stuck open (wastes
    /// bleed air, a performance penalty) or stuck closed (no ice
    /// protection) depending on the commanded direction at the time.
    pub stuck: f64,
}

#[derive(Clone, Copy, Debug)]
pub struct AntiIceValve {
    position: f64,
    lip_k: f64,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct AntiIceState {
    pub position: f64,
    pub bled_kg_s: f64,
    pub lip_k: f64,
}

impl AntiIceValve {
    pub fn new(ambient_k: f64) -> Self {
        Self { position: 0.0, lip_k: ambient_k }
    }

    /// One step. `commanded_open` from the crew/ice-detection logic;
    /// `upstream_pa`/`upstream_k` the compressor bleed source conditions;
    /// `ambient_k` the airflow outside the cowl lip (already accounting for
    /// icing-condition temperature).
    pub fn step(&mut self, commanded_open: bool, upstream_pa: f64, upstream_k: f64, ambient_k: f64, faults: &AntiIceValveFaults, dt_s: f64) -> AntiIceState {
        let dt = dt_s.max(0.0);
        let stuck = faults.stuck.clamp(0.0, 1.0);
        let rate = (1.0 / TRAVEL_TIME_S) * (1.0 - stuck);
        let target = if commanded_open { 1.0 } else { 0.0 };
        let max_step = rate * dt;
        let error = target - self.position;
        self.position = (self.position + error.clamp(-max_step, max_step)).clamp(0.0, 1.0);

        let rho = upstream_pa.max(0.0) / (R_AIR * upstream_k.max(1.0));
        let dp = (upstream_pa - DOWNSTREAM_PA).max(0.0);
        let bled_kg_s = CD * AREA_M2 * self.position * (2.0 * rho * dp).max(0.0).sqrt();

        let heat_capacity_w_k = bled_kg_s * CP_AIR * HEAT_TRANSFER_EFFECTIVENESS;
        let conductance = heat_capacity_w_k + LIP_LOSS_W_K;
        let target_k = (heat_capacity_w_k * upstream_k + LIP_LOSS_W_K * ambient_k) / conductance.max(1e-9);
        let k = conductance / LIP_CAPACITY_J_K;
        self.lip_k = target_k + (self.lip_k - target_k) * (-k * dt).exp();

        AntiIceState { position: self.position, bled_kg_s, lip_k: self.lip_k }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn settle(v: &mut AntiIceValve, commanded: bool, faults: &AntiIceValveFaults, seconds: f64) -> AntiIceState {
        let dt = 0.1;
        let mut out = AntiIceState::default();
        for _ in 0..(seconds / dt) as usize {
            out = v.step(commanded, 3.0e5, 450.0, 250.0, faults, dt);
        }
        out
    }

    #[test]
    fn closed_valve_bleeds_nothing_and_lip_drifts_to_ambient_no_nan() {
        let mut v = AntiIceValve::new(250.0);
        let s = settle(&mut v, false, &AntiIceValveFaults::default(), 120.0);
        assert!(s.bled_kg_s.abs() < 1e-9);
        assert!((s.lip_k - 250.0).abs() < 1.0);
        assert!(!s.lip_k.is_nan());
    }

    #[test]
    fn an_open_valve_bleeds_air_and_warms_the_lip_above_ambient() {
        let mut v = AntiIceValve::new(250.0);
        let s = settle(&mut v, true, &AntiIceValveFaults::default(), 120.0);
        assert!(s.bled_kg_s > 0.0);
        assert!(s.lip_k > 250.0 + 20.0, "{}", s.lip_k);
    }

    #[test]
    fn a_stuck_closed_valve_gives_no_ice_protection() {
        let mut v = AntiIceValve::new(250.0);
        let s = settle(&mut v, true, &AntiIceValveFaults { stuck: 1.0 }, 120.0);
        assert_eq!(s.position, 0.0);
        assert!((s.lip_k - 250.0).abs() < 1.0);
    }

    #[test]
    fn a_stuck_open_valve_keeps_bleeding_when_commanded_shut() {
        let mut v = AntiIceValve::new(250.0);
        settle(&mut v, true, &AntiIceValveFaults::default(), 5.0);
        let faults = AntiIceValveFaults { stuck: 1.0 };
        let s = settle(&mut v, false, &faults, 30.0);
        assert!(s.bled_kg_s > 0.0, "stuck open: still bleeding after being commanded off");
    }
}
