use std::f64::consts::PI;

const R_AIR: f64 = 287.05;

const CD: f64 = 0.65;

#[derive(Clone, Copy, Debug, Default)]
pub struct BleedValveFaults {
    pub jam: f64,
}

pub struct BleedValveSpec {
    pub area_m2: f64,
    pub close_start_frac: f64,
    pub close_span_frac: f64,
    pub travel_time_s: f64,
}

pub const IP_HANDLING_BLEED: BleedValveSpec = BleedValveSpec { area_m2: 6.0e-3, close_start_frac: 0.55, close_span_frac: 0.15, travel_time_s: 1.5 };
pub const HP_HANDLING_BLEED: BleedValveSpec = BleedValveSpec { area_m2: 3.0e-3, close_start_frac: 0.70, close_span_frac: 0.15, travel_time_s: 1.5 };

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
    pub stall_margin_delta_pct: f64,
}

const MARGIN_PCT_PER_BLED_FRACTION: f64 = 40.0;

impl BleedValve {
    pub fn new() -> Self {
        Self { position: 1.0 }
    }

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
