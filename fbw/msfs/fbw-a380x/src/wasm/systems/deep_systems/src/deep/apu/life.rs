use super::params;

#[derive(Clone, Copy, Debug, Default)]
pub struct CoreLife {
    pub operating_hours: f64,
    pub start_cycles: u32,
    creep_damage: f64,
}

impl CoreLife {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn record_start(&mut self) {
        self.start_cycles += 1;
    }

    pub fn accumulate(&mut self, running: bool, indicated_egt_c: f64, dt_s: f64) {
        if !running {
            return;
        }
        let dt_hours = dt_s.max(0.0) / 3600.0;
        self.operating_hours += dt_hours;

        if indicated_egt_c > params::CREEP_THRESHOLD_EGT_C {
            let exponent = (indicated_egt_c - params::CREEP_REFERENCE_EGT_C) / params::CREEP_DOUBLING_C;
            let rate_per_hour = params::CREEP_RATE_PER_HOUR_AT_DESIGN_EGT * 2f64.powf(exponent);
            self.creep_damage = (self.creep_damage + rate_per_hour * dt_hours).min(1.0);
        }
    }

    pub fn compressor_wear_frac(&self) -> f64 {
        (params::COMPRESSOR_WEAR_PER_HOUR * self.operating_hours
            + params::COMPRESSOR_WEAR_PER_START * self.start_cycles as f64)
            .min(1.0)
    }

    pub fn turbine_wear_frac(&self) -> f64 {
        (params::TURBINE_WEAR_PER_HOUR * self.operating_hours + self.creep_damage).min(1.0)
    }

    pub fn creep_damage_frac(&self) -> f64 {
        self.creep_damage
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub struct StarterDutyCycle {
    heat_s: f64,
}

impl StarterDutyCycle {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn heat_fraction(&self) -> f64 {
        (self.heat_s / params::STARTER_DUTY_LIMIT_S).clamp(0.0, 1.0)
    }

    pub fn locked_out(&self) -> bool {
        self.heat_s >= params::STARTER_DUTY_LIMIT_S
    }

    pub fn step(&mut self, engaged: bool, fault_bias: f64, dt_s: f64) {
        let dt = dt_s.max(0.0);
        if engaged {
            self.heat_s += dt * (1.0 - fault_bias.clamp(0.0, 1.0));
        } else {
            let k = 1.0 / params::STARTER_COOLDOWN_TIME_CONSTANT_S;
            self.heat_s *= (-k * dt).exp();
        }
        self.heat_s = self.heat_s.max(0.0);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn no_running_time_leaves_life_untouched() {
        let mut life = CoreLife::new();
        life.accumulate(false, 900.0, 3600.0);
        assert_eq!(life.operating_hours, 0.0);
        assert_eq!(life.compressor_wear_frac(), 0.0);
        assert_eq!(life.turbine_wear_frac(), 0.0);
    }

    #[test]
    fn running_time_below_the_creep_threshold_still_accrues_hours_but_no_creep() {
        let mut life = CoreLife::new();
        life.accumulate(true, 400.0, 3600.0);
        assert!((life.operating_hours - 1.0).abs() < 1e-9);
        assert_eq!(life.creep_damage_frac(), 0.0);
        assert!(life.compressor_wear_frac() > 0.0);
        assert!(life.turbine_wear_frac() > 0.0);
    }

    #[test]
    fn running_hotter_creeps_faster_than_running_cooler_for_the_same_time() {
        let mut cool = CoreLife::new();
        let mut hot = CoreLife::new();
        for _ in 0..10 {
            cool.accumulate(true, 550.0, 360.0);
            hot.accumulate(true, 650.0, 360.0);
        }
        assert!(hot.creep_damage_frac() > cool.creep_damage_frac());
        assert!(hot.turbine_wear_frac() > cool.turbine_wear_frac());
    }

    #[test]
    fn start_cycles_add_a_discrete_wear_increment() {
        let mut life = CoreLife::new();
        let before = life.compressor_wear_frac();
        life.record_start();
        assert!(life.compressor_wear_frac() > before);
        assert_eq!(life.start_cycles, 1);
    }

    #[test]
    fn wear_never_exceeds_one() {
        let mut life = CoreLife::new();
        for _ in 0..50 {
            life.record_start();
        }
        life.accumulate(true, 900.0, 1_000_000.0);
        assert!(life.compressor_wear_frac() <= 1.0);
        assert!(life.turbine_wear_frac() <= 1.0);
    }

    #[test]
    fn cranking_accumulates_heat_and_it_decays_once_disengaged() {
        let mut duty = StarterDutyCycle::new();
        for _ in 0..100 {
            duty.step(true, 0.0, 1.0);
        }
        let after_crank = duty.heat_fraction();
        assert!(after_crank > 0.0);
        for _ in 0..600 {
            duty.step(false, 0.0, 1.0);
        }
        assert!(duty.heat_fraction() < after_crank);
    }

    #[test]
    fn a_long_enough_crank_locks_the_starter_out() {
        let mut duty = StarterDutyCycle::new();
        for _ in 0..((params::STARTER_DUTY_LIMIT_S as u64) + 5) {
            duty.step(true, 0.0, 1.0);
        }
        assert!(duty.locked_out());
    }

    #[test]
    fn a_biased_duty_model_never_locks_out_even_after_a_very_long_crank() {
        let mut duty = StarterDutyCycle::new();
        for _ in 0..((params::STARTER_DUTY_LIMIT_S as u64) * 10) {
            duty.step(true, 1.0, 1.0);
        }
        assert!(!duty.locked_out(), "a fully biased model must never track any heat");
        assert_eq!(duty.heat_fraction(), 0.0);
    }

    #[test]
    fn negative_dt_never_goes_negative_or_nan() {
        let mut duty = StarterDutyCycle::new();
        duty.step(true, 0.0, -5.0);
        assert!(duty.heat_fraction() >= 0.0 && duty.heat_fraction().is_finite());
        let mut life = CoreLife::new();
        life.accumulate(true, 900.0, -1.0);
        assert!(life.operating_hours >= 0.0 && life.operating_hours.is_finite());
    }
}
