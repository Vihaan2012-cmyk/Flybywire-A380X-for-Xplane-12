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
//! Fuel flow is limited two ways, exactly as a real FADEC/ECB limits it,
//! and the governor takes the lower of the two (min-select):
//!
//! * [`egt_limit_fuel_flow_kg_s`] is the open-loop acceleration schedule --
//!   cap the *commanded* fuel flow before it is ever issued, from what EGT
//!   it would produce, rather than issuing an unbounded command and
//!   clamping the symptom afterward. It is solved on
//!   `power_section::gas_path`, the same chain that actually produces EGT,
//!   iterated to a Stodola-consistent turbine pressure ratio, so the
//!   schedule and the physics agree by construction.
//! * [`EgtLimiter`] closes the same limit on *measured* EGT (the ECB's own
//!   thermocouples), which is what protects a degraded machine the
//!   speed-and-inlet-conditions schedule cannot know about.

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
///
/// Solved on the *same* gas path the power section runs
/// (`power_section::gas_path`), closed on the Stodola-consistent turbine
/// pressure ratio: guess the pressure ratio, invert the expansion for the
/// turbine-inlet temperature that lands the turbine *exit* on the limit,
/// burn for the fuel flow that reaches it, then re-derive the pressure
/// ratio that fuel flow actually produces through Stodola's ellipse law,
/// and repeat. The map is a strong contraction (the pressure ratio moves
/// with the square root of turbine-inlet temperature, the temperature drop
/// only logarithmically with the pressure ratio), so a fixed small number
/// of passes converges to well inside a kelvin -- checked in this file's
/// `the_schedule_lands_on_its_own_egt_limit` test, which runs the
/// authoritative chain on the fuel flow the schedule returns and asserts it
/// produces the limit temperature. The earlier first-order stand-in
/// (`1 + (pr_design - 1) * n_frac`) could not: it over-estimated the
/// pressure ratio at part speed, which over-estimated the temperature drop
/// across the turbine and so under-estimated the EGT a given fuel flow
/// would produce.
///
/// The schedule is evaluated on a *healthy* gas path, because a real fuel
/// control's acceleration schedule is a fixed function of measured speed
/// and inlet conditions -- it does not know the compressor is eroded. That
/// is what the separate [`EgtLimiter`], which closes on the ECB's own
/// thermocouples, is for.
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

    // The compressor's own operating point does not depend on the fuel
    // flow being solved for, so it is evaluated once outside the loop.
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
        // tt_out = tt4 * (1 - drop_fraction)  =>  tt4 = tt_out / (1 - drop_fraction)
        let tt4_target = limit_k / (1.0 - drop_fraction);
        fuel = combustor::fuel_flow_for_target_tt4_kg_s(mdot_air, tt3, tt4_target);
        pressure_ratio = power_section::gas_path(calibration, &healthy, t1, p1, n_frac, fuel)
            .turbine_pressure_ratio
            .max(1.0 + 1e-6);
    }
    fuel.max(0.0)
}

/// The EGT limiter a real FADEC/ECB runs alongside the open-loop
/// acceleration schedule: it has thermocouples, so it does not have to
/// trust a schedule to know how hot the turbine actually is. It trims a
/// fuel-flow ceiling down whenever *measured* EGT is above the applicable
/// limit and lets it recover to unrestricted when it is below, and the
/// governor takes the lower of this ceiling and the schedule's
/// (min-select, the standard arrangement).
///
/// This is what makes a degraded machine behave: an eroded compressor or a
/// damaged turbine runs hotter on the same fuel, the schedule (which is a
/// function of speed and inlet conditions only) does not know that, and
/// without a measured-temperature loop nothing would stop the governor
/// burning whatever it takes to hold 100% N until the hard EGT trip fires.
/// With it the machine droops off governed speed instead, which is what a
/// real APU does.
pub struct EgtLimiter {
    ceiling_kg_s: f64,
    max_output_kg_s: f64,
    gain_kg_s_per_k_s: f64,
}

impl EgtLimiter {
    /// GENERIC gain, stated as an authority rather than fitted: a sustained
    /// 100 K overtemperature walks the ceiling across the whole metering
    /// range in 2 s. That is slow compared with the speed governor (so the
    /// two loops do not fight), slow enough to stay stable even at the
    /// coarsest tick this model is asked to run at, and fast compared with
    /// how long the turbine can sit above its running limit before the hard
    /// trip at `params::EGT_TRIP_C`, 200 K higher.
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

    /// `measured_egt_c` is the ECB's own indicated EGT (its voted
    /// thermocouples -- a biased or failed sensor therefore biases this
    /// loop exactly as it would a real one), `limit_c` the applicable
    /// limit.
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

    /// The point of solving the schedule on `power_section::gas_path`: run
    /// the authoritative chain on the fuel flow the schedule hands back and
    /// it must actually land on the limit temperature. The first-order
    /// pressure-ratio stand-in this replaced missed it by tens of kelvin at
    /// part speed, in the unconservative direction.
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
        // Comfortably cool: it stays at the metering valve's own ceiling.
        for _ in 0..100 {
            l.step(500.0, params::EGT_RUNNING_LIMIT_C, true, 0.05);
        }
        assert_eq!(l.ceiling_kg_s(), 0.1);
        // 100 K over the limit walks it shut inside the stated authority
        // time, and never below zero.
        let mut t = 0.0;
        while l.ceiling_kg_s() > 0.0 && t < 30.0 {
            l.step(params::EGT_RUNNING_LIMIT_C + 100.0, params::EGT_RUNNING_LIMIT_C, true, 0.05);
            t += 0.05;
        }
        assert!(t <= EgtLimiter::AUTHORITY_TIME_S + 0.1, "took {t} s to close");
        assert_eq!(l.ceiling_kg_s(), 0.0);
        // Shutting down re-opens it, so the next start is not begun against
        // a ceiling left over from the last run.
        l.step(1000.0, params::EGT_RUNNING_LIMIT_C, false, 0.05);
        assert_eq!(l.ceiling_kg_s(), 0.1);
    }

    #[test]
    fn zero_or_near_zero_speed_never_produces_nan_in_the_schedule() {
        let f = egt_limit_fuel_flow_kg_s(&cal(), 0.0, 288.15, 101_325.0, params::EGT_START_LIMIT_C);
        assert!(f.is_finite() && f >= 0.0);
    }
}

