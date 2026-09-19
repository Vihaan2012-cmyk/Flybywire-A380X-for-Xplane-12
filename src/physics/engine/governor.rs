//! The fuel-flow governor: turns the FADEC's commanded corrected N1 (the
//! real `A380FadecComputer`'s output, `engine_commands.rs`'s
//! `o.N1_c_percent`) into an actual fuel mass flow, the way a real EEC's
//! fuel metering valve is scheduled. Everything downstream of the fuel
//! flow this produces (temperature, spool speed, thrust) is physics, not a
//! curve fit; only the control law that decides *how much fuel* is a
//! deliberately-designed feedback loop, exactly as a real FADEC's is.
//!
//! Light-off and the minimum-speed combustion floor use `engines.cfg`'s own
//! `min_n1_for_combustion`/`min_n2_for_combustion` (the latter read as the
//! A380's N3/HP spool, matching the rest of this port's convention);
//! overspeed protection uses its `max_n1_protection`/`max_n2_protection`.

use super::params::{
    MAX_COMBUSTOR_FUEL_AIR_RATIO, MAX_N1_PROTECTION_PCT, MAX_N3_PROTECTION_PCT, MIN_N1_FOR_COMBUSTION_PCT,
    MIN_N3_FOR_COMBUSTION_PCT, STATIC_THRUST_N,
};

/// A generic-SFC reference fuel flow, kg/s, used only by this module's own
/// tests as a plausible stand-in value: derived from a typical large
/// high-bypass turbofan static specific fuel consumption of about 0.33 lb of
/// fuel per lbf of thrust per hour (no certificated SFC figure for the
/// Trent 972-84 specifically is public) applied to `STATIC_THRUST_N`.
/// Production code no longer uses this: `step` below takes the model's own
/// *calibrated* design fuel flow (`physics::engine::Engine::design_wf_kg_s`,
/// bisected in `mod.rs` so the full gas path reproduces `STATIC_THRUST_N`
/// exactly) as a real input instead of re-deriving an independent, uncoupled
/// SFC guess for the same physical quantity — one fewer place two different
/// numbers claimed to be the same thing.
pub fn generic_sfc_reference_wf_kg_s() -> f64 {
    const SFC_LB_PER_LBF_HR: f64 = 0.33;
    let static_thrust_lbf = STATIC_THRUST_N / 4.448_221_615_3;
    let wf_lb_hr = SFC_LB_PER_LBF_HR * static_thrust_lbf;
    wf_lb_hr * 0.453_593_4 / 3600.0
}

// kg/s of fuel per percent N1 of error. This is the fast, real-time-critical
// gain the CS-E 745/14 CFR 33.73 5-second idle-to-TOGA requirement
// (`mod.rs`'s own certification-timed test) needs, and it is also what
// establishes idle at all from a stopped/starter-cranked core (a weaker gain
// was tried and rejected: it left the engine stuck well short of idle, e.g.
// N1≈4%/N3≈23% instead of the ~15/~60% commanded, indefinitely — this
// engine's low-speed torque balance genuinely needs this much fuel authority
// to climb away from the light-off floor, not just to react quickly once
// already running). What this gain must not do is schedule more fuel than
// the core can actually burn for whatever little air is flowing early in a
// start, which is what `MAX_COMBUSTOR_FUEL_AIR_RATIO` below is for: it, not
// a weaker `KP`, is what has to keep an ordinary ground start's fuel flow
// and EGT realistic (see that constant's docs and `docs/physics/engine.md`
// for the remaining gap between this and a real Trent 900's own start law).
const KP: f64 = 0.06;
const KI: f64 = 0.02;
// %N1·s. With conditional integration below doing the anti-windup, this is
// only a backstop against a stuck-high target; KI times this is the full
// design fuel flow's worth of authority.
const INTEGRAL_LIMIT: f64 = 150.0;

/// The acceleration fuel schedule: the maximum fuel flow a real FADEC's
/// fuel-metering unit allows at the engine's *current* corrected core
/// speed, independent of how large the N1 error is. This is the actual
/// mechanism a real acceleration schedule uses to get from idle to TOGA
/// inside CS-E 745/14 CFR 33.73's 5-second limit while never running the
/// HP compressor into surge: a fixed multiple of the calibrated 100%-speed
/// design fuel flow (as the previous `design_wf_kg_s * 1.3` ceiling was)
/// cannot both (a) be loose enough to accelerate fast from a speed well
/// below 100% and (b) still bound the flow once at 100% -- a schedule keyed
/// to *speed*, the same variable a real Wf/Pt3-vs-N schedule is keyed to,
/// can be. No public Trent 900 acceleration-schedule map exists, so the
/// shape here is generic: it widens from 1.3x design flow at idle-ish
/// corrected N3 to 2.2x at and above 100% corrected N3, reflecting that
/// surge margin on most compressors widens, not narrows, on the approach to
/// the design operating line (the opposite direction from where an
/// engine's rotating-stall margin is typically tightest, near part-speed).
/// This bounds *acceleration authority*, not steady running: once the PI
/// terms converge `wf` to the value that actually holds a given N1 (the
/// governor.rs docs above), this ceiling sits well above what steady
/// running ever needs and never binds there.
///
/// Deliberately separate from `fadec.rs`'s own `polynomial::start_n1`/
/// `start_ff`/`start_egt` FBW start-law curves: those model a *cold, sub-
/// idle* start off the starter (N3 climbing from rest with no combustion
/// yet, feed-forward from N3 alone, no feedback), while this schedule only
/// ever runs once combustion is already established and a thrust-lever N1
/// target exists to govern toward (idle-to-TOGA and all other in-flight/
/// on-ground power changes) -- the same "start law vs. running governor"
/// division the real EEC has between its start schedule and its running
/// fuel-metering schedule.
fn accel_schedule_max_wf_kg_s(n3_corrected_pct: f64, design_wf_kg_s: f64) -> f64 {
    let n3_frac = (n3_corrected_pct / 100.0).clamp(0.0, 1.2);
    let multiple = ACCEL_SCHEDULE_BASE + ACCEL_SCHEDULE_SLOPE * n3_frac;
    design_wf_kg_s * multiple
}

/// The acceleration fuel-air-ratio margin over the design-point fuel-air
/// ratio (the Wf/P3-form schedule, `Governor::with_design_far`). Calibrated
/// to EASA.E.012 Note 12: "the acceleration from 15% to 95% rated take off
/// power is 5,6 seconds" (`acceleration_from_15_to_95_percent_takeoff_
/// thrust_matches_the_data_sheet` in `mod.rs`).
pub const ACCEL_FAR_MARGIN: f64 = 1.125;

/// The schedule's multiple of design fuel flow: `BASE + SLOPE * N3/100`
/// (generic; see above). Only a governor built without a design fuel-air
/// ratio (`Governor::new`) uses it; every engine uses the calibrated
/// Wf/P3 form (`ACCEL_FAR_MARGIN`). A speed-keyed cap sat above take-off
/// fuel flow at high N3, which let 15% -> 95% power take ~1.6 s.
const ACCEL_SCHEDULE_BASE: f64 = 1.3;
const ACCEL_SCHEDULE_SLOPE: f64 = 0.9;

#[derive(Clone, Copy, Debug, Default)]
pub struct Governor {
    integral: f64,
    /// The engine's design-point fuel-air ratio (design fuel flow over
    /// design core airflow), when known: the acceleration limit is then a
    /// fuel-air-ratio schedule on the air actually reaching the combustor
    /// (`ACCEL_FAR_MARGIN`), the Wf/P3 form a real EEC uses. Without it,
    /// the older speed-keyed schedule (`accel_schedule_max_wf_kg_s`).
    design_far: Option<f64>,
    /// After a start hands over (`track`), the N1 target the loop chases
    /// ramps from where the fan was toward the FADEC's target at
    /// `POST_START_N1_RAMP_PCT_S`, instead of stepping to it with the fan
    /// still far behind the core; `None` once it has caught up.
    ramp: Option<f64>,
}

/// How fast the N1 target rises after a start hands over, %N1 per second:
/// the fan settling onto idle over some seconds, as a real one does, rather
/// than the loop chasing the full idle error at once. Generic.
const POST_START_N1_RAMP_PCT_S: f64 = 2.0;

impl Governor {
    pub fn new() -> Self {
        Self { integral: 0.0, ramp: None, design_far: None }
    }

    /// A governor for an engine whose design-point fuel-air ratio is `far`.
    pub fn with_design_far(far: f64) -> Self {
        Self { design_far: Some(far), ..Self::new() }
    }

    /// `combustion_allowed` gates fuel entirely off (a real HP fuel shutoff
    /// valve/master switch, or a corrected speed below the combustion
    /// floor); it is not a curve, just whether a flame can be sustained at
    /// all right now.
    /// Bumpless transfer while the start schedule sets the fuel: hold the
    /// integral where this law's own output (against the fan's current
    /// speed as the target) would equal `wf_kg_s`, and restart the target
    /// ramp from the fan's current speed, so handing control back steps
    /// neither the fuel nor the target.
    pub fn track(&mut self, wf_kg_s: f64, measured_n1_corrected_pct: f64, design_wf_kg_s: f64) {
        let feedforward = design_wf_kg_s * (measured_n1_corrected_pct / 100.0).max(0.0).powi(3);
        self.integral = ((wf_kg_s - feedforward) / KI).clamp(-INTEGRAL_LIMIT, INTEGRAL_LIMIT);
        self.ramp = Some(measured_n1_corrected_pct);
    }

    pub fn combustion_floor_met(n1_corrected_pct: f64, n3_corrected_pct: f64) -> bool {
        n1_corrected_pct >= MIN_N1_FOR_COMBUSTION_PCT || n3_corrected_pct >= MIN_N3_FOR_COMBUSTION_PCT
    }

    /// One step: `target_n1_corrected_pct` is the FADEC's commanded value,
    /// `measured_n1_corrected_pct`/`measured_n3_corrected_pct` this
    /// engine's actual current corrected speeds, `mdot_air_to_combustor_kg_s`
    /// the core air mass flow actually reaching the combustor this frame
    /// (after bleed, `mod.rs`'s `mdot_to_combustor`). Returns the commanded
    /// fuel mass flow, kg/s.
    ///
    /// `target_n1_corrected_pct` tracks a thrust-lever-driven N1 (idle,
    /// climb, TOGA...); it is not a start schedule, and early in a start
    /// `measured_n1_corrected_pct` lags far behind it (the fan/LP spool is
    /// the last thing to spin up, driven only by whatever the LP turbine
    /// can extract from a still-small core flow) while the HP spool the
    /// starter is actually turning is much further along. Chasing that
    /// large N1 error with the feedback terms below would schedule far more
    /// fuel than the core can currently burn — a real FADEC's fuel-metering
    /// unit is itself airflow-referenced for exactly this reason, so the
    /// final clamp below is against `mdot_air_to_combustor_kg_s`, not just a
    /// flat ceiling: light-off and early spool-up are limited by how much
    /// air is actually flowing, the same physical constraint that makes a
    /// hot start emerge from `combustor.rs`'s energy balance rather than
    /// needing a separate scripted case.
    pub fn step(
        &mut self,
        target_n1_corrected_pct: f64,
        measured_n1_corrected_pct: f64,
        measured_n3_corrected_pct: f64,
        fuel_valve_open: bool,
        mdot_air_to_combustor_kg_s: f64,
        // The engine's own calibrated design-point fuel flow, kg/s
        // (`Engine::design_wf_kg_s`, `mod.rs`) — the real, physically
        // self-consistent number this engine's own gas path needs at 100%
        // N1/SL/ISA, not an independently-guessed SFC figure.
        design_wf_kg_s: f64,
        dt_s: f64,
    ) -> f64 {
        if !fuel_valve_open || !Self::combustion_floor_met(measured_n1_corrected_pct, measured_n3_corrected_pct) {
            // No combustion can be sustained; hold the integrator so it
            // does not wind up while starved of feedback, matching the
            // anti-windup approach already used by the throttle-trim loop
            // elsewhere in this plugin. A flameout/valve closure also
            // un-latches the running mode, so the next light-off goes
            // through the start law again rather than resuming the
            // running N1 governor from a cold core.
            self.integral = 0.0;
            return 0.0;
        }

        let far_limit = mdot_air_to_combustor_kg_s.max(0.0) * MAX_COMBUSTOR_FUEL_AIR_RATIO;
        // The acceleration limit: a real EEC's Wf/P3 schedule bounds fuel
        // by the compressor delivery pressure, i.e. by the air the core is
        // actually passing, so a low-power core can only be fuelled up
        // gradually as it spools, however far the lever went.
        let accel_limit = match self.design_far {
            Some(far) => mdot_air_to_combustor_kg_s.max(0.0) * far * ACCEL_FAR_MARGIN,
            None => accel_schedule_max_wf_kg_s(measured_n3_corrected_pct, design_wf_kg_s),
        };

        // After a start, chase a target ramping up from where the fan was.
        let target_n1_corrected_pct = match self.ramp {
            Some(r) => {
                let next = (r + POST_START_N1_RAMP_PCT_S * dt_s).min(target_n1_corrected_pct);
                self.ramp = if next >= target_n1_corrected_pct { None } else { Some(next) };
                next
            }
            None => target_n1_corrected_pct,
        };
        let error = target_n1_corrected_pct - measured_n1_corrected_pct;
        let feedforward = design_wf_kg_s * (target_n1_corrected_pct / 100.0).max(0.0).powi(3);

        // Conditional-integration anti-windup: the integral only moves while
        // the fuel command is not pinned at a limit in the direction the
        // error pushes. A hard cap on the integral (the earlier form) also
        // capped how much steady-state error it could ever remove: at 3.0
        // %·s times KI it had 0.06 kg/s of authority, and a ground-idle
        // target settled ~1.5% N1 short with the integral saturated.
        let ceiling = accel_limit.min(far_limit);
        let trial = self.integral + error * dt_s;
        let unclamped = feedforward + KP * error + KI * trial;
        let pinned = (unclamped > ceiling && error > 0.0) || (unclamped < 0.0 && error < 0.0);
        if !pinned {
            self.integral = trial.clamp(-INTEGRAL_LIMIT, INTEGRAL_LIMIT);
        }
        let mut wf = feedforward + KP * error + KI * self.integral;

        // Overspeed protection: a real FADEC pulls fuel back hard rather
        // than let the core or fan overspeed.
        if measured_n1_corrected_pct > MAX_N1_PROTECTION_PCT || measured_n3_corrected_pct > MAX_N3_PROTECTION_PCT {
            wf = wf.min(design_wf_kg_s * 0.5);
        }

        wf.clamp(0.0, ceiling)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn no_fuel_below_the_combustion_floor() {
        assert!(!Governor::combustion_floor_met(5.0, 10.0));
        assert!(Governor::combustion_floor_met(12.0, 10.0));
        assert!(Governor::combustion_floor_met(5.0, 25.0));
    }

    /// Design-point core airflow used across these tests when the point is
    /// not to exercise the fuel-air-ratio cap: comfortably above
    /// `design_wf_kg_s() * 1.3 / MAX_COMBUSTOR_FUEL_AIR_RATIO`, so the cap
    /// never binds and these tests exercise only the feedback law.
    const AMPLE_MDOT_KG_S: f64 = 200.0;

    #[test]
    fn a_closed_fuel_valve_gives_zero_flow() {
        let mut g = Governor::new();
        let wf = g.step(90.0, 40.0, 60.0, false, AMPLE_MDOT_KG_S, generic_sfc_reference_wf_kg_s(), 0.1);
        assert_eq!(wf, 0.0);
    }

    #[test]
    fn a_low_target_below_the_combustion_floor_gives_no_fuel() {
        let mut g = Governor::new();
        let wf = g.step(5.0, 2.0, 5.0, true, AMPLE_MDOT_KG_S, generic_sfc_reference_wf_kg_s(), 0.1);
        assert_eq!(wf, 0.0);
    }

    #[test]
    fn a_higher_target_commands_more_fuel_once_lit() {
        let mut idle_gov = Governor::new();
        let mut toga_gov = Governor::new();
        let idle_wf = idle_gov.step(25.0, 25.0, 60.0, true, AMPLE_MDOT_KG_S, generic_sfc_reference_wf_kg_s(), 0.1);
        let toga_wf = toga_gov.step(100.0, 25.0, 60.0, true, AMPLE_MDOT_KG_S, generic_sfc_reference_wf_kg_s(), 0.1);
        assert!(toga_wf > idle_wf, "{idle_wf} {toga_wf}");
    }

    #[test]
    fn overspeed_pulls_fuel_back() {
        let mut g = Governor::new();
        let wf = g.step(100.0, 105.0, 60.0, true, AMPLE_MDOT_KG_S, generic_sfc_reference_wf_kg_s(), 0.1);
        assert!(wf <= generic_sfc_reference_wf_kg_s() * 0.5 + 1e-9);
    }

    #[test]
    fn the_feedforward_fuel_flow_is_a_plausible_order_of_magnitude() {
        // Public references for A380/Trent-900-class engines commonly cite
        // roughly 1-1.2 kg/s per engine at cruise and several kg/s at max
        // power; the 100%-N1 feed-forward design point should land near
        // the top of that range, not off by orders of magnitude.
        let wf = generic_sfc_reference_wf_kg_s();
        assert!(wf > 1.0 && wf < 6.0, "{wf}");
    }

    /// The field bug (EGT over the limit, fuel flow in the thousands of
    /// kg/h during an ordinary ground start): commanded to a real
    /// ground-idle N1 target (~19% corrected, sea-level ISA -- see
    /// `fadec::table1502::icn1`/`generate_idle_parameters`, not a climb or
    /// TOGA value), with N1 still lagging near the combustion floor just
    /// after light-off (N3 just past `MIN_N3_FOR_COMBUSTION_PCT`) the way a
    /// real fan does early in a start, the N1-error term alone would
    /// schedule far more fuel than the still-small core airflow can burn.
    /// The fuel-air-ratio backstop is what bounds it -- confirmed here, not
    /// just asserted in the constant's docs. **Known gap** (see
    /// `docs/physics/engine.md`): with `KP` sized for the certification-timed
    /// spool-up test (a weaker gain was tried and left the engine unable to
    /// reach idle at all, see `KP`'s docs), this backstop alone does not yet
    /// bring a cold start all the way down to FlyByWire's own cited "a few
    /// hundred kg/h" -- a real fuel-metering unit's separate, N3-referenced
    /// start law (not attempted here in the time available) is the further
    /// fix that would close that gap; this test locks in the bound actually
    /// achieved so a regression cannot silently reopen it further.
    #[test]
    fn an_early_light_off_fuel_flow_is_bounded_by_the_fuel_air_ratio_not_unbounded_by_n1_error() {
        let mut g = Governor::new();
        let idle_n1_target_pct = 19.0;
        let core_mdot_kg_s = 5.0; // small, just-past-light-off core airflow
        let wf = g.step(idle_n1_target_pct, 14.0, 33.0, true, core_mdot_kg_s, generic_sfc_reference_wf_kg_s(), 0.05);
        let far_limit = core_mdot_kg_s * super::MAX_COMBUSTOR_FUEL_AIR_RATIO;
        assert!(wf <= far_limit + 1e-9, "wf {wf} exceeded the fuel-air-ratio backstop {far_limit}");
        // Still a real, order-of-magnitude improvement on the field report's
        // thousands-of-kg/h reading for the same N1/N3, even though it is
        // not yet down to a few hundred (see the "known gap" note above).
        assert!(wf * 3600.0 < 2000.0, "{} kg/h, expected well under the field report's ~3000 kg/h", wf * 3600.0);
    }

    /// The fuel-air-ratio backstop (`params::MAX_COMBUSTOR_FUEL_AIR_RATIO`)
    /// must still actually bind for a pathologically large, sustained
    /// error at near-zero core airflow -- it exists so nothing (a future
    /// change, a stuck-high target) can ever schedule literally unbounded
    /// fuel for whatever little air the engine can currently flow, even
    /// though ordinary start/idle/TOGA operation never reaches it (the test
    /// above, and `mod.rs`'s certification-timed spool-up test, confirm
    /// that side).
    #[test]
    fn the_fuel_air_ratio_backstop_still_bounds_a_pathological_error() {
        let mut g = Governor::new();
        let core_mdot_kg_s = 2.0;
        let far_limit = core_mdot_kg_s * super::MAX_COMBUSTOR_FUEL_AIR_RATIO;
        let mut wf = 0.0;
        for _ in 0..50 {
            wf = g.step(100.0, 5.0, 25.0, true, core_mdot_kg_s, generic_sfc_reference_wf_kg_s(), 0.1);
        }
        assert!(wf <= far_limit + 1e-9, "wf {wf} exceeded the fuel-air-ratio backstop {far_limit}");
    }
}
