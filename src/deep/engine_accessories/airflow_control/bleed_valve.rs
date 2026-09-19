//! IP and HP compressor handling (surge) bleed valves: spring-loaded/
//! actuated valves that dump compressor delivery air overboard at low
//! spool speed, when a compressor's natural surge margin is at its
//! narrowest, then close as speed rises and margin widens on its own. Each
//! is modelled the same way `../fuel/manifold.rs` models an orifice off a
//! shared plenum: `Q = Cd * A_open * sqrt(2*rho*dP)`, so the mass flow
//! actually bled is a real physical quantity the gas path can subtract
//! from the core flow reaching the combustor, not an abstract fraction --
//! and a schedule error here has two distinct, physically opposite
//! failure directions: jammed *open* at high power bleeds air the core
//! needs (a real thrust and, downstream, fuel-air-ratio penalty), jammed
//! *closed* at low power removes the very margin the valve exists to
//! protect (this module's `stall_margin_delta_pct` output, alongside the
//! VSV's, `vsv.rs`, for the gas path's compressor model to consume).
//!
//! No Trent-900 handling bleed valve sizing is public. The schedule (open
//! below a corrected-speed threshold, closed above it, with a short
//! transition) and orifice area are **GENERIC**, sized so a fully open
//! valve at low-speed conditions bleeds a small-but-material fraction
//! (order 2-5%) of the IP/HP compressor's own design flow, typical of
//! published handling-bleed flow fractions for large turbofans.

use std::f64::consts::PI;

/// Standard dry-air gas constant, J/(kg K) (restated; this module cannot
/// import `physics::engine::gas`, per this directory's isolation rule).
const R_AIR: f64 = 287.05;

/// Orifice discharge coefficient (sharp-edged, standard textbook value) and
/// open area, sized per module docs.
const CD: f64 = 0.65;

#[derive(Clone, Copy, Debug, Default)]
pub struct BleedValveFaults {
    /// Mechanically jammed, 0 free .. 1 seized: scales travel rate to zero,
    /// freezing the valve wherever it is (reads as jammed-open or
    /// jammed-closed depending on the commanded direction at the time,
    /// exactly like the other rate-limited valves in this directory).
    pub jam: f64,
}

pub struct BleedValveSpec {
    /// Full-open orifice area, m^2.
    pub area_m2: f64,
    /// Corrected-speed fraction the valve starts closing at, and the width
    /// of that closing transition.
    pub close_start_frac: f64,
    pub close_span_frac: f64,
    /// Actuator travel time, full stroke, s.
    pub travel_time_s: f64,
}

/// IP handling bleed: closes early (the IPC's surge margin narrows first
/// at low core speed), **GENERIC** area sized to ~4% of a representative
/// IP-stage design flow.
pub const IP_HANDLING_BLEED: BleedValveSpec = BleedValveSpec { area_m2: 6.0e-3, close_start_frac: 0.55, close_span_frac: 0.15, travel_time_s: 1.5 };
/// HP handling bleed: closes a little later, **GENERIC** area sized to ~2%
/// of a representative HP-stage design flow (the HPC's smaller annulus).
pub const HP_HANDLING_BLEED: BleedValveSpec = BleedValveSpec { area_m2: 3.0e-3, close_start_frac: 0.70, close_span_frac: 0.15, travel_time_s: 1.5 };

/// The schedule: commanded open fraction at a given corrected-speed
/// fraction.
pub fn schedule_open_fraction(spec: &BleedValveSpec, corrected_frac: f64) -> f64 {
    (1.0 - (corrected_frac - spec.close_start_frac) / spec.close_span_frac).clamp(0.0, 1.0)
}

#[derive(Clone, Copy, Debug)]
pub struct BleedValve {
    position: f64,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct BleedValveState {
    pub position: f64,
    pub bled_kg_s: f64,
    /// Stall-margin contribution, percentage points (positive = margin
    /// gained by bleeding); a documented interface for the gas path's
    /// compressor model, mirroring `vsv::VsvState::stall_margin_delta_pct`.
    pub stall_margin_delta_pct: f64,
}

/// Margin gained per unit of bled mass flow relative to a representative
/// design flow, percentage points (**GENERIC**: a fully open valve at its
/// scheduled low-speed condition is worth a few points of margin, the
/// order of magnitude handling bleeds are sized to provide).
const MARGIN_PCT_PER_BLED_FRACTION: f64 = 40.0;

impl BleedValve {
    pub fn new() -> Self {
        Self { position: 1.0 }
    }

    /// One step. `corrected_frac` is this spool's corrected speed fraction;
    /// `upstream_pa`/`upstream_k` the compressor delivery conditions the
    /// valve bleeds from; `downstream_pa` the sink (bypass duct/ambient) it
    /// dumps to; `design_flow_kg_s` the representative design flow the
    /// margin contribution is normalised against.
    pub fn step(&mut self, spec: &BleedValveSpec, corrected_frac: f64, upstream_pa: f64, upstream_k: f64, downstream_pa: f64, design_flow_kg_s: f64, faults: &BleedValveFaults, dt_s: f64) -> BleedValveState {
        let dt = dt_s.max(0.0);
        let jam = faults.jam.clamp(0.0, 1.0);
        let target = schedule_open_fraction(spec, corrected_frac);
        let rate = (1.0 / spec.travel_time_s) * (1.0 - jam);
        let max_step = rate * dt;
        let error = target - self.position;
        self.position += error.clamp(-max_step, max_step);
        self.position = self.position.clamp(0.0, 1.0);

        let dp = (upstream_pa - downstream_pa).max(0.0);
        let rho = upstream_pa.max(0.0) / (R_AIR * upstream_k.max(1.0));
        let bled_kg_s = CD * spec.area_m2 * self.position * (2.0 * rho * dp).max(0.0).sqrt();

        let bled_fraction = if design_flow_kg_s > 1e-9 { bled_kg_s / design_flow_kg_s } else { 0.0 };
        let stall_margin_delta_pct = MARGIN_PCT_PER_BLED_FRACTION * bled_fraction;

        BleedValveState { position: self.position, bled_kg_s, stall_margin_delta_pct }
    }
}

/// A quick sanity bound: PI is unused directly but documents that the
/// orifice relation above is the incompressible/low-Mach form used
/// throughout this directory (`fuel::manifold`), consistent for the modest
/// pressure ratios a handling bleed operates across.
const _: f64 = PI;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fully_closed_at_high_speed_bleeds_nothing_and_no_nan() {
        let mut v = BleedValve::new();
        let mut s = BleedValveState::default();
        for _ in 0..50 {
            s = v.step(&IP_HANDLING_BLEED, 1.0, 5.0e5, 400.0, 1.0e5, 100.0, &BleedValveFaults::default(), 0.1);
        }
        assert!(s.bled_kg_s.abs() < 1e-6);
        assert!(!s.bled_kg_s.is_nan());
    }

    #[test]
    fn open_at_low_speed_bleeds_a_positive_flow_and_gains_margin() {
        let mut v = BleedValve::new();
        let mut s = BleedValveState::default();
        for _ in 0..50 {
            s = v.step(&IP_HANDLING_BLEED, 0.3, 3.0e5, 350.0, 1.0e5, 100.0, &BleedValveFaults::default(), 0.1);
        }
        assert!(s.bled_kg_s > 0.0);
        assert!(s.stall_margin_delta_pct > 0.0);
    }

    #[test]
    fn a_valve_jammed_open_at_high_speed_keeps_bleeding_air_the_core_needs() {
        let mut healthy = BleedValve::new();
        let mut jammed = BleedValve::new();
        let mut hs = BleedValveState::default();
        let mut js = BleedValveState::default();
        for _ in 0..80 {
            hs = healthy.step(&IP_HANDLING_BLEED, 1.0, 5.0e5, 400.0, 1.0e5, 100.0, &BleedValveFaults::default(), 0.1);
            js = jammed.step(&IP_HANDLING_BLEED, 1.0, 5.0e5, 400.0, 1.0e5, 100.0, &BleedValveFaults { jam: 1.0 }, 0.1);
        }
        assert!(hs.bled_kg_s < 1e-6);
        assert!(js.bled_kg_s > 1.0, "a stuck-open handling bleed should still be dumping core air: {}", js.bled_kg_s);
    }

    #[test]
    fn a_valve_jammed_closed_at_low_speed_loses_the_margin_it_would_have_given() {
        let mut jammed = BleedValve::new();
        let mut js = BleedValveState::default();
        // Start effectively closed (position starts at 1.0/open in `new()`,
        // so jam it after commanding closed once near-design speed, then
        // drop speed with the jam already engaged).
        for _ in 0..50 {
            jammed.step(&IP_HANDLING_BLEED, 1.0, 5.0e5, 400.0, 1.0e5, 100.0, &BleedValveFaults::default(), 0.1);
        }
        for _ in 0..50 {
            js = jammed.step(&IP_HANDLING_BLEED, 0.2, 3.0e5, 350.0, 1.0e5, 100.0, &BleedValveFaults { jam: 1.0 }, 0.1);
        }
        assert!(js.bled_kg_s.abs() < 1e-6, "jammed shut: no bleed even though the schedule now wants it open");
        assert_eq!(js.stall_margin_delta_pct, 0.0);
    }
}
