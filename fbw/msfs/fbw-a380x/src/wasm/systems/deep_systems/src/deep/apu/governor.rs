use super::combustor;
use super::gas;
use super::params;
use super::power_section;
use super::turbine_flow;

pub struct Governor {
    integral: f64,
    max_output_kg_s: f64,
    kp: f64,
    ki: f64,
}

impl Governor {
    pub fn new(max_output_kg_s: f64) -> Self {
        Self {
            integral: 0.0,
            max_output_kg_s: max_output_kg_s.max(0.0),
            kp: 0.0030,
            ki: 0.0018,
        }
    }

    pub fn reset(&mut self) {
        self.integral = 0.0;
    }

    pub fn step(&mut self, n_percent: f64, running: bool, max_allowed_kg_s: f64, dt_s: f64) -> f64 {
        if !running {
            self.integral = 0.0;
            return 0.0;
        }
        let dt = dt_s.max(0.0);
        let error = params::GOVERNED_N_PERCENT - n_percent;
        let ceiling = max_allowed_kg_s.min(self.max_output_kg_s).max(0.0);

        let trial_integral = self.integral + error * dt;
        let trial_output = self.kp * error + self.ki * trial_integral;
        let output = trial_output.clamp(0.0, ceiling);

        if (output - trial_output).abs() < 1e-12 {
            self.integral = trial_integral;
        }
        output
    }
}

const SCHEDULE_PASSES: u32 = 5;

pub fn egt_limit_fuel_flow_kg_s(
    calibration: &power_section::Calibration,
    n_percent: f64,
    ambient_temperature_k: f64,
    ambient_pressure_pa: f64,
    limit_c: f64,
) -> f64 {
    let n_frac = (n_percent / 100.0).max(0.02);
    let t1 = ambient_temperature_k.max(1.0);
    let p1 = ambient_pressure_pa.max(1.0);
    let healthy = power_section::PowerSectionFaults::default();
    let limit_k = (limit_c + 273.15).max(t1 + 1.0);

    let probe = power_section::gas_path(calibration, &healthy, t1, p1, n_frac, 0.0);
    let mdot_air = probe.compressor.mdot_kg_s;
    let tt3 = probe.compressor.tt_out_k;
    let pr_design = calibration.turbine_pressure_ratio_design.max(1.0 + 1e-6);

    let mut pressure_ratio = pr_design;
    let mut fuel = 0.0;
    for _ in 0..SCHEDULE_PASSES {
        let eta = turbine_flow::efficiency(&calibration.turbine_spec, pressure_ratio / pr_design);
        let temp_ratio_isentropic =
            gas::temperature_ratio_from_pressure_ratio(1.0 / pressure_ratio, gas::GAMMA_GAS);
        let drop_fraction = ((1.0 - temp_ratio_isentropic) * eta).clamp(0.0, 0.95);
        let tt4_target = limit_k / (1.0 - drop_fraction);
        fuel = combustor::fuel_flow_for_target_tt4_kg_s(mdot_air, tt3, tt4_target);
        pressure_ratio = power_section::gas_path(calibration, &healthy, t1, p1, n_frac, fuel)
            .turbine_pressure_ratio
            .max(1.0 + 1e-6);
    }
    fuel.max(0.0)
}

pub struct EgtLimiter {
    ceiling_kg_s: f64,
    max_output_kg_s: f64,
    gain_kg_s_per_k_s: f64,
}

impl EgtLimiter {
    const AUTHORITY_BAND_K: f64 = 100.0;
    const AUTHORITY_TIME_S: f64 = 2.0;

    pub fn new(max_output_kg_s: f64) -> Self {
        let max = max_output_kg_s.max(0.0);
        Self {
            ceiling_kg_s: max,
            max_output_kg_s: max,
            gain_kg_s_per_k_s: max / (Self::AUTHORITY_BAND_K * Self::AUTHORITY_TIME_S),
        }
    }

    pub fn reset(&mut self) {
        self.ceiling_kg_s = self.max_output_kg_s;
    }

    pub fn ceiling_kg_s(&self) -> f64 {
        self.ceiling_kg_s
    }

    pub fn step(&mut self, measured_egt_c: f64, limit_c: f64, running: bool, dt_s: f64) -> f64 {
        if !running {
            self.reset();
            return self.ceiling_kg_s;
        }
        let error_k = limit_c - measured_egt_c;
        self.ceiling_kg_s = (self.ceiling_kg_s + self.gain_kg_s_per_k_s * error_k * dt_s.max(0.0))
            .clamp(0.0, self.max_output_kg_s);
        self.ceiling_kg_s
    }
}

pub fn egt_limit_c(n_percent: f64) -> f64 {
    if n_percent < params::SELF_SUSTAINING_N_PERCENT {
        params::EGT_START_LIMIT_C
    } else {
        params::EGT_RUNNING_LIMIT_C
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cal() -> power_section::Calibration {
        power_section::calibrate()
    }

    #[test]
    fn not_running_commands_zero_fuel_and_resets_the_integral() {
        let mut g = Governor::new(0.05);
        assert_eq!(g.step(0.0, false, f64::MAX, 1.0), 0.0);
        assert_eq!(g.step(50.0, false, f64::MAX, 1.0), 0.0);
    }

    #[test]
    fn a_large_speed_error_saturates_at_the_ceiling_not_beyond() {
        let mut g = Governor::new(0.05);
        let out = g.step(0.0, true, 0.03, 0.05);
        assert!(out <= 0.03 + 1e-12, "{out}");
        assert!(out > 0.0);
    }

    #[test]
    fn at_the_setpoint_with_zero_history_it_commands_zero_extra_fuel() {
        let mut g = Governor::new(0.05);
        let out = g.step(params::GOVERNED_N_PERCENT, true, f64::MAX, 0.05);
        assert!((out).abs() < 1e-9, "{out}");
    }

    #[test]
    fn it_is_stable_across_a_wide_range_of_tick_rates() {
        const SIMULATED_SECONDS: f64 = 300.0;
        for dt in [0.01, 0.05, 0.1, 0.5, 1.0] {
            let mut g = Governor::new(0.06);
            let mut n = 80.0;
            let iterations = (SIMULATED_SECONDS / dt) as u64;
            for _ in 0..iterations {
                let fuel = g.step(n, true, f64::MAX, dt);
                n += (fuel - 0.02) * 40.0 * dt;
                assert!(n.is_finite() && n.abs() < 1000.0, "diverged at dt={dt}: n={n}");
            }
            assert!((n - params::GOVERNED_N_PERCENT).abs() < 5.0, "dt={dt} settled at {n}");
        }
    }

    #[test]
    fn the_egt_limit_schedule_allows_more_fuel_as_speed_rises_toward_governed() {
        let low = egt_limit_fuel_flow_kg_s(&cal(), 20.0, 288.15, 101_325.0, params::EGT_START_LIMIT_C);
        let high = egt_limit_fuel_flow_kg_s(&cal(), 90.0, 288.15, 101_325.0, params::EGT_START_LIMIT_C);
        assert!(high > low, "{low} {high}");
        assert!(low.is_finite() && high.is_finite());
    }

    #[test]
    fn the_running_limit_is_stricter_than_the_start_limit_at_the_same_speed() {
        let start_allowed =
            egt_limit_fuel_flow_kg_s(&cal(), 90.0, 288.15, 101_325.0, params::EGT_START_LIMIT_C);
        let running_allowed =
            egt_limit_fuel_flow_kg_s(&cal(), 90.0, 288.15, 101_325.0, params::EGT_RUNNING_LIMIT_C);
        assert!(running_allowed < start_allowed);
    }

    #[test]
    fn egt_limit_c_switches_from_the_start_to_the_running_limit_at_self_sustaining_speed() {
        assert_eq!(egt_limit_c(params::SELF_SUSTAINING_N_PERCENT - 1.0), params::EGT_START_LIMIT_C);
        assert_eq!(egt_limit_c(params::SELF_SUSTAINING_N_PERCENT + 1.0), params::EGT_RUNNING_LIMIT_C);
    }

    #[test]
    fn the_schedule_lands_on_its_own_egt_limit() {
        let cal = cal();
        let healthy = power_section::PowerSectionFaults::default();
        for n in [30.0, 50.0, 70.0, 85.0, 100.0f64] {
            for (t1, p1) in [(288.15, 101_325.0), (243.15, 70_000.0), (313.15, 101_325.0)] {
                let limit_c = egt_limit_c(n);
                let fuel = egt_limit_fuel_flow_kg_s(&cal, n, t1, p1, limit_c);
                let egt_k = power_section::gas_path(&cal, &healthy, t1, p1, n / 100.0, fuel)
                    .expansion
                    .tt_out_k;
                assert!(
                    (egt_k - 273.15 - limit_c).abs() < 1.0,
                    "n={n} t1={t1} p1={p1}: schedule fuel {fuel:.6} kg/s gives {:.1} C, limit {limit_c} C",
                    egt_k - 273.15
                );
            }
        }
    }

    #[test]
    fn a_cold_day_allows_more_fuel_before_the_same_egt_limit_than_a_hot_one() {
        let cal = cal();
        let cold = egt_limit_fuel_flow_kg_s(&cal, 100.0, 253.15, 101_325.0, params::EGT_RUNNING_LIMIT_C);
        let hot = egt_limit_fuel_flow_kg_s(&cal, 100.0, 313.15, 101_325.0, params::EGT_RUNNING_LIMIT_C);
        assert!(cold > hot, "cold {cold} hot {hot}");
    }

    #[test]
    fn the_measured_egt_limiter_is_wide_open_below_the_limit_and_closes_above_it() {
        let mut l = EgtLimiter::new(0.1);
        assert_eq!(l.ceiling_kg_s(), 0.1);
        for _ in 0..100 {
            l.step(500.0, params::EGT_RUNNING_LIMIT_C, true, 0.05);
        }
        assert_eq!(l.ceiling_kg_s(), 0.1);
        let mut t = 0.0;
        while l.ceiling_kg_s() > 0.0 && t < 30.0 {
            l.step(params::EGT_RUNNING_LIMIT_C + 100.0, params::EGT_RUNNING_LIMIT_C, true, 0.05);
            t += 0.05;
        }
        assert!(t <= EgtLimiter::AUTHORITY_TIME_S + 0.1, "took {t} s to close");
        assert_eq!(l.ceiling_kg_s(), 0.0);
        l.step(1000.0, params::EGT_RUNNING_LIMIT_C, false, 0.05);
        assert_eq!(l.ceiling_kg_s(), 0.1);
    }

    #[test]
    fn zero_or_near_zero_speed_never_produces_nan_in_the_schedule() {
        let f = egt_limit_fuel_flow_kg_s(&cal(), 0.0, 288.15, 101_325.0, params::EGT_START_LIMIT_C);
        assert!(f.is_finite() && f >= 0.0);
    }
}

