//! The fuel control unit's speed governor -- item 3's "speed governor (fuel
//! control unit) at 100% constant speed".
//!
//! Real APUs are constant-speed machines: the two generators need one fixed
//! shaft speed to hold 400 Hz regardless of electrical or bleed load, unlike
//! a main engine's throttle-set speed. This is therefore a single governed
//! setpoint (`params::GOVERNED_N_PERCENT`), not an idle/load-band schedule.
//!
//! A plain proportional-integral controller (no crate dependency:
//! FlyByWire's own `shared::pid::PidController` is out of reach under the
//! brief's self-containment rule, so this directory has its own), with
//! conditional-integration anti-windup (the integral term only accumulates
//! on a tick where doing so does not push the output further into
//! saturation -- the standard, simple anti-windup technique for a
//! saturating actuator).
//!
//! The EGT-limit fuel schedule below is the acceleration-schedule concept
//! every real FADEC/fuel control implements: cap the *commanded* fuel flow
//! before it is ever issued, from a first-order estimate of what EGT it
//! would produce, rather than issuing an unbounded command and clamping the
//! symptom afterward. It uses the same isentropic turbine relation
//! `turbine_flow::expand` uses, solved in reverse for the fuel flow that
//! reaches the limit -- an *estimate* (it approximates the turbine pressure
//! ratio a real Stodola solve would only know once the fuel flow itself is
//! fixed, since that is exactly the unknown being solved for), used only as
//! a protective ceiling; `power_section.rs`'s own Stodola-consistent chain
//! is the authoritative physics that actually produces EGT.

use super::combustor;
use super::compressor_map;
use super::gas;
use super::params;
use super::turbine_flow;

pub struct Governor {
    integral: f64,
    max_output_kg_s: f64,
    kp: f64,
    ki: f64,
}

impl Governor {
    /// `max_output_kg_s`: the fuel metering valve's own physical ceiling
    /// (`fuel_control.rs`), independent of the EGT-limit schedule.
    pub fn new(max_output_kg_s: f64) -> Self {
        Self {
            integral: 0.0,
            max_output_kg_s: max_output_kg_s.max(0.0),
            // GENERIC: gains chosen for a stable, reasonably quick response
            // across a wide range of tick rates (checked in this file's own
            // `governor_is_stable_across_tick_rates` test), the same
            // consideration FlyByWire's own governor tuning documents for
            // the identical reason, but chosen independently for this
            // model's own units/scale (fuel flow in kg/s here, not the same
            // absolute numbers as FBW's file).
            kp: 0.0030,
            ki: 0.0018,
        }
    }

    pub fn reset(&mut self) {
        self.integral = 0.0;
    }

    /// One control step. `n_percent` is the measured spool speed;
    /// `max_allowed_kg_s` is this tick's EGT-limit-schedule ceiling
    /// (`egt_limit_fuel_flow_kg_s`, or `f64::MAX` to not apply one).
    /// Returns the commanded fuel flow, kg/s -- still subject to
    /// `fuel_control.rs`'s own solenoid/metering-valve dynamics and faults
    /// before it ever reaches the combustor.
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

        // Conditional-integration anti-windup: commit the integral update
        // only if it did not need clamping this tick.
        if (output - trial_output).abs() < 1e-12 {
            self.integral = trial_integral;
        }
        output
    }
}

/// The EGT-limit fuel schedule -- see module docs. `limit_c` is
/// `params::EGT_START_LIMIT_C` while below `params::SELF_SUSTAINING_N_PERCENT`
/// (a real start's richer transient fuelling is allowed a higher transient
/// limit), `params::EGT_RUNNING_LIMIT_C` once self-sustaining and governed.
pub fn egt_limit_fuel_flow_kg_s(
    n_percent: f64,
    ambient_temperature_k: f64,
    ambient_pressure_pa: f64,
    limit_c: f64,
) -> f64 {
    let n_frac = (n_percent / 100.0).max(0.02);
    let t1 = ambient_temperature_k.max(1.0);
    let p1 = ambient_pressure_pa.max(1.0);

    let core_spec = compressor_map::Spec {
        pr_design: params::CORE_PRESSURE_RATIO_DESIGN,
        eta_design: params::CORE_COMPRESSOR_EFFICIENCY_DESIGN,
        mdot_corrected_design_kg_s: params::CORE_MDOT_DESIGN_KG_S,
        efficiency_falloff: params::CORE_COMPRESSOR_EFFICIENCY_FALLOFF,
        surge_margin_design_frac: params::CORE_SURGE_MARGIN_DESIGN_FRAC,
        surge_line_flatness: params::CORE_SURGE_LINE_FLATNESS,
        choke_flow_multiple: params::CORE_CHOKE_FLOW_MULTIPLE,
    };
    let compressor = compressor_map::evaluate(
        &core_spec,
        t1,
        p1,
        n_frac,
        params::CORE_MDOT_DESIGN_KG_S * n_frac,
    );

    // First-order estimate of the turbine pressure ratio at this speed
    // (see module docs for why this is an estimate, not the authoritative
    // Stodola solve).
    let pr_design = params::turbine_pressure_ratio_design();
    let pr_estimate = (1.0 + (pr_design - 1.0) * n_frac).max(1.0 + 1e-6);

    let turbine_spec = turbine_flow::Spec {
        eta_design: params::TURBINE_EFFICIENCY_DESIGN,
        efficiency_falloff: params::TURBINE_EFFICIENCY_FALLOFF,
    };
    let eta = turbine_flow::efficiency(&turbine_spec, pr_estimate / pr_design);

    let limit_k = (limit_c + 273.15).max(t1 + 1.0);
    let temp_ratio_isentropic =
        gas::temperature_ratio_from_pressure_ratio(1.0 / pr_estimate, gas::GAMMA_GAS);
    let drop_fraction = ((1.0 - temp_ratio_isentropic) * eta).clamp(0.0, 0.95);
    // tt_out = tt4 * (1 - drop_fraction)  =>  tt4 = tt_out / (1 - drop_fraction)
    let tt4_target = limit_k / (1.0 - drop_fraction);

    combustor::fuel_flow_for_target_tt4_kg_s(compressor.mdot_kg_s, compressor.tt_out_k, tt4_target)
}

/// Picks the applicable EGT limit for the current speed (see module docs).
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
        // A crude linear proxy plant (speed responds proportionally to
        // fuel above a nominal equilibrium value) closes into a damped
        // second-order loop with this controller's gains; its estimated
        // settling time is on the order of a minute, so each tested tick
        // rate simulates a fixed 300 s of real time (not a fixed iteration
        // count, which would simulate far less real time at a fine tick
        // rate than a coarse one and could look unsettled purely from not
        // having run long enough).
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
        let low = egt_limit_fuel_flow_kg_s(20.0, 288.15, 101_325.0, params::EGT_START_LIMIT_C);
        let high = egt_limit_fuel_flow_kg_s(90.0, 288.15, 101_325.0, params::EGT_START_LIMIT_C);
        assert!(high > low, "{low} {high}");
        assert!(low.is_finite() && high.is_finite());
    }

    #[test]
    fn the_running_limit_is_stricter_than_the_start_limit_at_the_same_speed() {
        let start_allowed =
            egt_limit_fuel_flow_kg_s(90.0, 288.15, 101_325.0, params::EGT_START_LIMIT_C);
        let running_allowed =
            egt_limit_fuel_flow_kg_s(90.0, 288.15, 101_325.0, params::EGT_RUNNING_LIMIT_C);
        assert!(running_allowed < start_allowed);
    }

    #[test]
    fn egt_limit_c_switches_from_the_start_to_the_running_limit_at_self_sustaining_speed() {
        assert_eq!(egt_limit_c(params::SELF_SUSTAINING_N_PERCENT - 1.0), params::EGT_START_LIMIT_C);
        assert_eq!(egt_limit_c(params::SELF_SUSTAINING_N_PERCENT + 1.0), params::EGT_RUNNING_LIMIT_C);
    }

    #[test]
    fn zero_or_near_zero_speed_never_produces_nan_in_the_schedule() {
        let f = egt_limit_fuel_flow_kg_s(0.0, 288.15, 101_325.0, params::EGT_START_LIMIT_C);
        assert!(f.is_finite() && f >= 0.0);
    }
}
