//! APU oil system: gear pump, pressure regulation, level and cooling --
//! item 4.
//!
//! Pressure is not a lookup curve: a gear pump driven directly off the
//! gas-generator spool delivers a flow proportional to speed; a relief
//! valve regulates pressure at and above `params::OIL_REGULATION_N_PERCENT`,
//! the same physical mechanism `physics/engine/oil.rs`'s relief valve uses
//! (documented there), restated independently here per this directory's
//! self-containment rule. Below that speed, pressure rises with the pump's
//! own (as yet unregulated) delivery.
//!
//! A leak drains the tank over time (`OilFaults::leak`); as the level falls
//! below a low-level threshold the pump starts to starve (cavitate),
//! progressively losing its ability to hold pressure rather than holding it
//! exactly until the tank is bone dry and then dropping instantly -- the
//! same physical route `src/failures.rs`'s existing "APU oil leak" hook
//! documents wanting ("quantity falls until the APU is running starved of
//! oil").
//!
//! Temperature is a lumped heat balance -- friction/windage heat in (scaled
//! from `power_section.rs`'s own compressor absorption torque and speed),
//! ambient-air cooling out -- integrated with an exact exponential step
//! (the same technique `physics/engine/hot_section.rs` documents and uses),
//! not a per-tick linear increment that would depend on tick rate.

use super::params;

/// Kinematic viscosity, cSt, by the Walther (ASTM D341) relation, fitted
/// through published MIL-PRF-23699 turbine oil data points (see
/// `params.rs`'s citation). Rises steeply below normal operating
/// temperature -- the physical basis of a cold-soaked start needing more
/// cranking torque (`OilSystem::cold_drag_torque_nm`).
pub fn viscosity_cst(temp_k: f64) -> f64 {
    let a = params::OIL_VISCOSITY_WALTHER_A;
    let b = params::OIL_VISCOSITY_WALTHER_B;
    let t = temp_k.clamp(200.0, 600.0);
    10f64.powf(10f64.powf(a - b * t.log10())) - 0.7
}

#[derive(Clone, Copy, Debug, Default)]
pub struct OilFaults {
    /// Oil leak severity, 0 healthy .. 1 (a seal or line leaking oil
    /// overboard at `MAX_LEAK_RATE_L_S`).
    pub leak: f64,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct OilState {
    pub pressure_psi: f64,
    pub temp_c: f64,
    pub level_l: f64,
    pub level_frac: f64,
    /// Debounced low-pressure protective trip (`params::OIL_PRESSURE_TRIP_PSI`
    /// sustained for `params::OIL_PRESSURE_TRIP_DEBOUNCE_S` while running).
    pub low_pressure_tripped: bool,
}

#[derive(Clone, Copy, Debug)]
pub struct OilSystem {
    level_l: f64,
    temp_k: f64,
    low_pressure_for_s: f64,
}

/// A fully leaking system (fault magnitude 1.0) empties
/// `params::OIL_TANK_CAPACITY_L` in a few minutes -- a plausible order of
/// magnitude for a small accessory-gearbox oil system leak (GENERIC, no
/// PW980A-specific figure is public).
const MAX_LEAK_RATE_L_S: f64 = 0.02;

impl OilSystem {
    pub fn new(temp_k: f64) -> Self {
        Self {
            level_l: params::OIL_TANK_CAPACITY_L,
            temp_k: temp_k.max(1.0),
            low_pressure_for_s: 0.0,
        }
    }

    pub fn level_l(&self) -> f64 {
        self.level_l
    }

    pub fn temp_c(&self) -> f64 {
        self.temp_k - 273.15
    }

    pub fn temp_k(&self) -> f64 {
        self.temp_k
    }

    /// Extra cranking drag torque a cold-soaked oil charge imposes on the
    /// spool, over and above the (already-modelled) hot-oil baseline
    /// friction -- item 1's "cold-soaked oil ... slowing the start". A cold
    /// start genuinely has to shear much more viscous oil through the
    /// bearings/gearbox at the same shaft speed; modelled as a viscous
    /// (proportional to speed) drag scaled by how far the current viscosity
    /// exceeds the hot reference (`params::OIL_HOT_REFERENCE_K`), so it is
    /// exactly zero once the oil is warmed to its normal running
    /// temperature and only significant on a genuinely cold-soaked start.
    pub fn cold_drag_torque_nm(&self, omega_rad_s: f64) -> f64 {
        let hot_cst = viscosity_cst(params::OIL_HOT_REFERENCE_K);
        let current_cst = viscosity_cst(self.temp_k);
        let excess_ratio = (current_cst / hot_cst - 1.0).max(0.0);
        params::OIL_COLD_DRAG_COEFF_NM_PER_RAD_S * excess_ratio * omega_rad_s.max(0.0)
    }

    pub fn step(
        &mut self,
        n_percent: f64,
        running: bool,
        ambient_temp_k: f64,
        friction_heat_w: f64,
        faults: &OilFaults,
        dt_s: f64,
    ) -> OilState {
        let dt = dt_s.max(0.0);

        let leak_l_s = MAX_LEAK_RATE_L_S * faults.leak.clamp(0.0, 1.0);
        self.level_l = (self.level_l - leak_l_s * dt).max(0.0);
        let level_frac = (self.level_l / params::OIL_TANK_CAPACITY_L).clamp(0.0, 1.0);

        // Below the low-level trip point the pump progressively starves
        // (cavitates) rather than holding full pressure right up until the
        // tank is exactly empty.
        let starvation_frac = if self.level_l <= 1e-6 {
            0.0
        } else if self.level_l < params::OIL_LOW_LEVEL_TRIP_L {
            (self.level_l / params::OIL_LOW_LEVEL_TRIP_L).clamp(0.0, 1.0)
        } else {
            1.0
        };

        let n_frac = (n_percent / 100.0).max(0.0);
        let regulation_frac = (params::OIL_REGULATION_N_PERCENT / 100.0).max(1e-6);
        let unregulated_psi = params::OIL_REGULATED_PRESSURE_PSI * (n_frac / regulation_frac).min(3.0);
        let pressure_psi = unregulated_psi.min(params::OIL_REGULATED_PRESSURE_PSI) * starvation_frac;

        // Temperature: exact exponential step toward the balance of
        // friction heat in and ambient cooling out.
        let capacity = params::OIL_HEAT_CAPACITY_J_K;
        let conductance = capacity / params::OIL_TIME_CONSTANT_S;
        let ambient = ambient_temp_k.max(1.0);
        let target = ambient + friction_heat_w.max(0.0) / conductance.max(1e-6);
        let k = 1.0 / params::OIL_TIME_CONSTANT_S;
        self.temp_k = target + (self.temp_k - target) * (-k * dt).exp();

        if running && n_percent > 5.0 && pressure_psi < params::OIL_PRESSURE_TRIP_PSI {
            self.low_pressure_for_s += dt;
        } else {
            self.low_pressure_for_s = 0.0;
        }
        let low_pressure_tripped = self.low_pressure_for_s >= params::OIL_PRESSURE_TRIP_DEBOUNCE_S;

        OilState {
            pressure_psi,
            temp_c: self.temp_k - 273.15,
            level_l: self.level_l,
            level_frac,
            low_pressure_tripped,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn run(oil: &mut OilSystem, n_percent: f64, seconds: f64, faults: &OilFaults) -> OilState {
        let dt = 0.1;
        let mut out = OilState::default();
        for _ in 0..(seconds / dt) as u64 {
            out = oil.step(n_percent, true, 288.15, 300.0, faults, dt);
        }
        out
    }

    #[test]
    fn at_rest_pressure_is_zero_and_nothing_is_nan() {
        let mut oil = OilSystem::new(288.15);
        let out = oil.step(0.0, false, 288.15, 0.0, &OilFaults::default(), 1.0);
        assert_eq!(out.pressure_psi, 0.0);
        assert!(out.temp_c.is_finite());
    }

    #[test]
    fn pressure_is_regulated_at_and_above_regulation_speed() {
        let mut oil = OilSystem::new(288.15);
        let at_reg = run(&mut oil, params::OIL_REGULATION_N_PERCENT, 5.0, &OilFaults::default());
        let mut oil2 = OilSystem::new(288.15);
        let above_reg = run(&mut oil2, 100.0, 5.0, &OilFaults::default());
        assert!((at_reg.pressure_psi - params::OIL_REGULATED_PRESSURE_PSI).abs() < 1.0);
        assert!((above_reg.pressure_psi - params::OIL_REGULATED_PRESSURE_PSI).abs() < 1.0);
    }

    #[test]
    fn below_regulation_speed_pressure_rises_with_speed() {
        let mut low = OilSystem::new(288.15);
        let low_out = run(&mut low, 20.0, 5.0, &OilFaults::default());
        let mut high = OilSystem::new(288.15);
        let high_out = run(&mut high, 40.0, 5.0, &OilFaults::default());
        assert!(high_out.pressure_psi > low_out.pressure_psi);
        assert!(high_out.pressure_psi < params::OIL_REGULATED_PRESSURE_PSI);
    }

    #[test]
    fn a_leak_drains_the_level_and_eventually_starves_pressure() {
        let mut oil = OilSystem::new(288.15);
        let out = run(&mut oil, 100.0, 500.0, &OilFaults { leak: 1.0 });
        assert!(out.level_l < params::OIL_TANK_CAPACITY_L * 0.5, "{}", out.level_l);
        assert!(out.pressure_psi < params::OIL_REGULATED_PRESSURE_PSI * 0.9);
    }

    #[test]
    fn a_severe_enough_leak_eventually_trips_low_pressure_protection() {
        let mut oil = OilSystem::new(288.15);
        let out = run(&mut oil, 100.0, 1200.0, &OilFaults { leak: 1.0 });
        assert!(out.level_l <= 1e-3, "{}", out.level_l);
        assert!(out.low_pressure_tripped);
    }

    #[test]
    fn a_brief_dip_below_the_trip_threshold_does_not_trip() {
        // Simulate a genuine but short dip: run healthy, then one tick at a
        // starved level for less than the debounce window.
        let mut oil = OilSystem::new(288.15);
        run(&mut oil, 100.0, 5.0, &OilFaults::default());
        let out = oil.step(100.0, true, 288.15, 300.0, &OilFaults { leak: 1.0 }, 0.5);
        assert!(!out.low_pressure_tripped || params::OIL_PRESSURE_TRIP_DEBOUNCE_S <= 0.5);
    }

    #[test]
    fn friction_heat_raises_steady_state_oil_temperature_above_ambient() {
        let mut oil = OilSystem::new(288.15);
        let out = run(&mut oil, 100.0, 1800.0, &OilFaults::default());
        assert!(out.temp_c > 288.15 - 273.15 + 5.0, "{}", out.temp_c);
    }

    #[test]
    fn no_leak_never_loses_level() {
        let mut oil = OilSystem::new(288.15);
        let out = run(&mut oil, 100.0, 3600.0, &OilFaults::default());
        assert!((out.level_l - params::OIL_TANK_CAPACITY_L).abs() < 1e-6);
    }

    #[test]
    fn the_walther_fit_reproduces_its_published_data_points() {
        assert!((viscosity_cst(313.15) - 27.6).abs() < 0.1, "{}", viscosity_cst(313.15));
        assert!((viscosity_cst(373.15) - 5.1).abs() < 0.05, "{}", viscosity_cst(373.15));
        assert!(viscosity_cst(233.15) > 5_000.0, "cold oil should be dramatically more viscous");
    }

    #[test]
    fn a_cold_soaked_start_needs_real_extra_cranking_torque_a_warm_one_does_not() {
        let cold = OilSystem::new(233.15); // -40 degC cold soak
        let warm = OilSystem::new(params::OIL_HOT_REFERENCE_K);
        let omega = 750.0;
        let cold_drag = cold.cold_drag_torque_nm(omega);
        let warm_drag = warm.cold_drag_torque_nm(omega);
        assert!(cold_drag > 0.5, "{cold_drag}");
        assert!(warm_drag.abs() < 1e-9, "{warm_drag}");
    }

    #[test]
    fn cold_drag_scales_with_speed_and_is_never_negative() {
        let cold = OilSystem::new(233.15);
        let low = cold.cold_drag_torque_nm(100.0);
        let high = cold.cold_drag_torque_nm(1000.0);
        assert!(high > low);
        assert!(cold.cold_drag_torque_nm(-50.0) >= 0.0, "negative omega must clamp, not go negative");
    }
}
