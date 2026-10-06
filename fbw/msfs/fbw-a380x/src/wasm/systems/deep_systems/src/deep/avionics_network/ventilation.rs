pub const SUPPLY_AIR_K: f64 = 288.15;

pub const CPIOM_HEAT_W: f64 = 150.0;
pub const IOM_HEAT_W: f64 = 80.0;

const FORCED_CONDUCTANCE_W_K: f64 = 12.0;
const NATURAL_CONDUCTANCE_W_K: f64 = 1.5;
const BAY_CAPACITY_J_K: f64 = 25_000.0;

#[derive(Clone, Copy, Debug, Default)]
pub struct FanFaults {
    pub failure: f64,
}
#[derive(Clone, Copy, Debug)]
pub struct Fan {
    pub powered: bool,
    pub faults: FanFaults,
}
impl Fan {
    pub fn new() -> Self {
        Self { powered: true, faults: FanFaults::default() }
    }

    pub fn output_frac(&self) -> f64 {
        if self.powered {
            1.0 - self.faults.failure.clamp(0.0, 1.0)
        } else {
            0.0
        }
    }
}
impl Default for Fan {
    fn default() -> Self {
        Self::new()
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub struct ExtractValveFaults {
    pub stuck_closed: f64,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct BayState {
    pub temp_k: f64,
    pub conductance_w_k: f64,
    pub airflow_frac: f64,
}

pub struct Bay {
    temp_k: f64,
}
impl Bay {
    pub fn new(initial_k: f64) -> Self {
        Self { temp_k: initial_k }
    }

    pub fn temp_k(&self) -> f64 {
        self.temp_k
    }

    pub fn step(&mut self, heat_w: f64, fans: &[Fan], valve_open: f64, valve_faults: &ExtractValveFaults, dt_s: f64) -> BayState {
        let fan_frac = fans.iter().map(|f| f.output_frac()).fold(0.0_f64, f64::max);
        let valve_frac = (valve_open.clamp(0.0, 1.0) * (1.0 - valve_faults.stuck_closed.clamp(0.0, 1.0))).clamp(0.0, 1.0);
        let airflow_frac = fan_frac * valve_frac;
        let conductance = NATURAL_CONDUCTANCE_W_K + airflow_frac * (FORCED_CONDUCTANCE_W_K - NATURAL_CONDUCTANCE_W_K);
        let dt = dt_s.max(0.0);
        let target = SUPPLY_AIR_K + heat_w.max(0.0) / conductance.max(1e-6);
        let k = conductance / BAY_CAPACITY_J_K;
        self.temp_k = target + (self.temp_k - target) * (-k * dt).exp();
        BayState { temp_k: self.temp_k, conductance_w_k: conductance, airflow_frac }
    }
}

pub const TRIP_TEMP_K: f64 = 273.15 + 70.0;
const TRIP_SPAN_K: f64 = 15.0;
const TRIP_TIME_CONSTANT_S: f64 = 5.0;

#[derive(Clone, Copy, Debug, Default)]
pub struct OverheatTrip {
    frac: f64,
}
impl OverheatTrip {
    pub fn frac(&self) -> f64 {
        self.frac
    }

    const ARRIVED_EPSILON: f64 = 1e-6;

    pub fn step(&mut self, bay_temp_k: f64, dt_s: f64) -> f64 {
        let target = ((bay_temp_k - TRIP_TEMP_K) / TRIP_SPAN_K).clamp(0.0, 1.0);
        let dt = dt_s.max(0.0);
        self.frac = target + (self.frac - target) * (-dt / TRIP_TIME_CONSTANT_S).exp();
        if (self.frac - target).abs() < Self::ARRIVED_EPSILON {
            self.frac = target;
        }
        self.frac
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn settle(bay: &mut Bay, heat_w: f64, fans: &[Fan], valve_open: f64, valve_faults: &ExtractValveFaults, seconds: f64) -> BayState {
        let dt = 1.0;
        let mut out = BayState::default();
        for _ in 0..(seconds / dt) as usize {
            out = bay.step(heat_w, fans, valve_open, valve_faults, dt);
        }
        out
    }

    #[test]
    fn healthy_forced_draught_settles_close_to_supply_air() {
        let mut bay = Bay::new(SUPPLY_AIR_K);
        let fans = [Fan::new(), Fan::new()];
        let out = settle(&mut bay, 2.0 * CPIOM_HEAT_W, &fans, 1.0, &ExtractValveFaults::default(), 5000.0);
        assert_eq!(out.airflow_frac, 1.0);
        let rise_k = out.temp_k - SUPPLY_AIR_K;
        assert!(rise_k > 0.0 && rise_k < 40.0, "rise {rise_k:.1} K");
    }

    #[test]
    fn losing_every_fan_collapses_to_natural_convection_and_runs_much_hotter() {
        let mut bay = Bay::new(SUPPLY_AIR_K);
        let fans = [Fan { powered: true, faults: FanFaults { failure: 1.0 } }, Fan { powered: true, faults: FanFaults { failure: 1.0 } }];
        let out = settle(&mut bay, 2.0 * CPIOM_HEAT_W, &fans, 1.0, &ExtractValveFaults::default(), 20_000.0);
        assert_eq!(out.airflow_frac, 0.0);
        let forced = SUPPLY_AIR_K + 2.0 * CPIOM_HEAT_W / FORCED_CONDUCTANCE_W_K;
        assert!(out.temp_k > forced, "natural convection ({:.1} K) should run hotter than forced ({:.1} K)", out.temp_k, forced);
    }

    #[test]
    fn one_healthy_fan_out_of_two_still_establishes_full_airflow() {
        let mut bay = Bay::new(SUPPLY_AIR_K);
        let fans = [Fan::new(), Fan { powered: false, faults: FanFaults::default() }];
        let out = bay.step(CPIOM_HEAT_W, &fans, 1.0, &ExtractValveFaults::default(), 1.0);
        assert_eq!(out.airflow_frac, 1.0);
    }

    #[test]
    fn a_stuck_closed_extract_valve_blocks_the_draught_even_with_healthy_fans() {
        let mut bay = Bay::new(SUPPLY_AIR_K);
        let fans = [Fan::new(), Fan::new()];
        let out = settle(&mut bay, 2.0 * CPIOM_HEAT_W, &fans, 1.0, &ExtractValveFaults { stuck_closed: 1.0 }, 20_000.0);
        assert_eq!(out.airflow_frac, 0.0);
    }

    #[test]
    fn overheat_trip_stays_untripped_below_threshold_and_ramps_above_it() {
        let mut trip = OverheatTrip::default();
        for _ in 0..100 {
            trip.step(TRIP_TEMP_K - 10.0, 1.0);
        }
        assert_eq!(trip.frac(), 0.0);
        for _ in 0..200 {
            trip.step(TRIP_TEMP_K + TRIP_SPAN_K, 1.0);
        }
        assert!(trip.frac() > 0.99, "trip fraction {}", trip.frac());
    }

    #[test]
    fn no_nan_at_zero_dt_or_at_rest() {
        let mut bay = Bay::new(SUPPLY_AIR_K);
        let out = bay.step(0.0, &[], 0.0, &ExtractValveFaults::default(), 0.0);
        assert!(!out.temp_k.is_nan());
        let mut trip = OverheatTrip::default();
        assert!(!trip.step(0.0, 0.0).is_nan());
    }
}

#[cfg(test)]
mod trip_tests {
    use super::*;

    #[test]
    fn a_sustained_overheat_trips_fully_rather_than_approaching_one_for_ever() {
        let mut trip = OverheatTrip::default();
        let mut t = 0.0;
        while trip.frac() < 1.0 && t < 600.0 {
            trip.step(TRIP_TEMP_K + 2.0 * TRIP_SPAN_K, 0.5);
            t += 0.5;
        }
        assert_eq!(trip.frac(), 1.0, "still ramping after {t} s");
        let mut t = 0.0;
        while trip.frac() > 0.0 && t < 600.0 {
            trip.step(SUPPLY_AIR_K, 0.5);
            t += 0.5;
        }
        assert_eq!(trip.frac(), 0.0, "still un-tripping after {t} s");
    }
}
