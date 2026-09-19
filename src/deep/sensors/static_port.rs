//! Static ports: the flush ports on the fuselage that feed the ADR its
//! reference (ambient) pressure. Models blockage (ice, tape left on before
//! flight, corrosion), a leak in the line downstream of the port (relevant
//! because a leak *inside the pressurised fuselage* pulls the reading toward
//! cabin pressure rather than true ambient), and position error (the port
//! never sits in perfectly undisturbed flow, so its reading differs slightly
//! from the free-stream static pressure by an amount that depends on AoA and
//! Mach).
//!
//! ## Blockage
//! Unlike the pitot tube, a static port has no drain hole distinction -- it
//! is a small flush orifice, not a forward-facing tube. A blocked static
//! port simply stops updating: instead of two failure modes it has one,
//! "frozen at the last good reading" (FAA *Airplane Flying Handbook*
//! (FAA-H-8083-3), ch. 4: "a blocked static system ... altimeter will freeze
//! at the altitude at which the blockage occurred").
//!
//! ## Leak toward cabin
//! Most of a transport's static line runs through the pressurised fuselage.
//! A crack or loose fitting there lets pressurised cabin air bleed into the
//! line; the port's own (much larger) orifice to the outside world normally
//! dominates, so the port still mostly wins, but a large enough leak
//! measurably biases the reading toward cabin pressure -- documented, e.g.,
//! in in-service reports of "blocked/leaking static system" altimeter
//! errors that correlate with cabin pressurisation changes. Modelled here as
//! a simple resistance-divider blend between true ambient and cabin
//! pressure, weighted by the leak's fraction of the port's own conductance
//! -- GENERIC (no published leak-orifice size for this component), but the
//! qualitative form (leak conductance in parallel with the port's own,
//! biasing the node pressure toward whichever side has more flow area) is
//! standard pneumatic-network reasoning.
//!
//! ## Position error
//! A generic transport-category static-source error curve: small at the
//! aircraft's normal cruise AoA/attitude, growing at low speed/high AoA
//! (where the local flow angle over the fuselage flush port changes most).
//! Not A380-specific (no public static-source error calibration chart for
//! it exists) -- GENERIC, shaped like the typical curves found in FAA
//! Airplane Flying Handbook ch. 4 and generic flight-test static-position-
//! error literature (error grows with AoA, is small near cruise AoA, and has
//! a mild Mach dependence from local compressibility near the port).

/// Position error coefficients (GENERIC, see module docs): the AoA at which
/// the curve is defined to read zero error (roughly a typical cruise AoA for
/// a large swept-wing transport), and the coefficient turning `qc` (impact
/// pressure, a proxy for how much the local flow field around the port is
/// being disturbed) and `(alpha - reference)` into a static pressure error.
const REFERENCE_AOA_DEG: f64 = 2.5;
const POSITION_ERROR_COEFF: f64 = 0.0025;
/// Small Mach-dependent term (local compressibility increases the port's
/// sensitivity to flow angle at higher Mach): GENERIC.
const MACH_SENSITIVITY: f64 = 0.4;

/// Reference pneumatic time constant used to build a restricted port's
/// response lag (see [`StaticPort::step`]): GENERIC, shorter than the
/// pitot's own [`super::pitot`] pneumatic lag since a static port is a
/// simple flush orifice without an extended tube's added line volume.
const STATIC_PORT_TAU_S: f64 = 0.2;

/// Fault inputs, each a fraction `0.0` (healthy) `..1.0` (fully failed).
#[derive(Clone, Copy, Debug, Default)]
pub struct StaticPortFaults {
    /// Ice, tape left over a pitot-static cover, debris: `1.0` fully sealed.
    pub blocked: f64,
    /// A breach in the line inside the pressurised fuselage, as a fraction
    /// of the port's own conductance (`1.0` roughly means the leak passes as
    /// much flow as the port itself, at which point the reading is heavily
    /// biased toward cabin pressure).
    pub leak_to_cabin: f64,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct StaticPortOutput {
    /// Pressure the ADR actually receives from this port, Pa.
    pub sensed_static_pressure_pa: f64,
    pub blocked: bool,
}

/// One static port's state.
#[derive(Clone, Copy, Debug)]
pub struct StaticPort {
    sensed_pa: f64,
}

impl StaticPort {
    pub fn new(initial_static_pa: f64) -> Self {
        Self { sensed_pa: initial_static_pa }
    }

    /// `true_static_pa`: the free-stream static pressure at the airframe
    /// this instant (before position error). `cabin_pa`: the pressurised
    /// fuselage's internal pressure, for the leak model. `alpha_deg`,
    /// `mach`: for the position-error curve.
    pub fn step(
        &mut self,
        true_static_pa: f64,
        cabin_pa: f64,
        alpha_deg: f64,
        mach: f64,
        faults: &StaticPortFaults,
        dt_s: f64,
    ) -> StaticPortOutput {
        let dt = dt_s.max(0.0);
        let blocked = faults.blocked.clamp(0.0, 1.0) >= 0.98;
        if blocked {
            // No path in or out: the last good reading simply holds.
            return StaticPortOutput { sensed_static_pressure_pa: self.sensed_pa, blocked: true };
        }

        // Position error: signed, grows with AoA away from the reference
        // and with Mach (local compressibility); `qc`-free formulation here
        // since a static port's own error is conventionally expressed
        // directly as a fraction of ambient pressure rather than of impact
        // pressure (there is no local total-pressure reading at this port).
        let position_error_pa = true_static_pa
            * POSITION_ERROR_COEFF
            * (alpha_deg - REFERENCE_AOA_DEG)
            * (1.0 + MACH_SENSITIVITY * mach.max(0.0));
        let true_reading_pa = true_static_pa + position_error_pa;

        // A partial blockage (e.g. partially iced-over port, or the same
        // fraction used continuously rather than as a hard on/off) narrows
        // the port's own conductance relative to any leak: blend toward the
        // frozen value proportionally, on top of the leak blend below.
        let own_conductance = (1.0 - faults.blocked.clamp(0.0, 1.0)).max(0.0);
        let leak_conductance = faults.leak_to_cabin.max(0.0);
        let total_conductance = own_conductance + leak_conductance;
        let ambient_and_leak_pa = if total_conductance > 1e-9 {
            (own_conductance * true_reading_pa + leak_conductance * cabin_pa) / total_conductance
        } else {
            self.sensed_pa
        };
        // A restricted (but not fully blocked) port also responds sluggishly
        // to the node pressure above rather than instantaneously; conductance
        // toward 0 stretches the effective response, same reasoning as the
        // pitot tube's restricted-orifice lag. `own_conductance` alone (not
        // `total_conductance`) sets this, since a leak does not help the
        // port itself respond faster to the *outside* world. A fully open
        // port (`own_conductance == 1`) has zero added resistance and so
        // tracks instantly (`tau == 0`); a restricted one gets a time
        // constant that grows as its conductance falls
        // (`tau = TAU * (1/conductance - 1)`, the standard reciprocal
        // relation between a restrictor's conductance and the RC lag it
        // adds). This is an *exact* exponential step in `dt` (matching
        // `pitot.rs`'s `(-dt/tau).exp()` pattern), not a fixed per-call
        // blend fraction -- a fixed fraction would make the effective time
        // constant depend on how often `step` is called, which is wrong.
        let conductance = own_conductance.max(0.02);
        let tau = STATIC_PORT_TAU_S * (1.0 / conductance - 1.0);
        let k = if tau <= 1e-9 { 0.0 } else { (-dt / tau).exp() };
        self.sensed_pa = ambient_and_leak_pa + (self.sensed_pa - ambient_and_leak_pa) * k;

        StaticPortOutput { sensed_static_pressure_pa: self.sensed_pa, blocked: false }
    }
}

// ---------------------------------------------------------------------
// Left/right static-port pneumatic averaging.
// ---------------------------------------------------------------------

/// A system's left and right static ports are commonly tied together by a
/// pneumatic averaging line before the ADR specifically so their readings
/// can be blended: in sideslip, one side's local flow speeds up (reading
/// low) while the other slows down (reading high) relative to the
/// free-stream static pressure, and averaging the two cancels that
/// sideslip-driven error to first order -- a documented rationale for
/// paired static ports on transport aircraft (FAA Airplane Flying Handbook
/// ch. 4's static-position-error discussion covers the general effect;
/// pairing/averaging is the standard mitigation). The averaging line
/// itself is a real, separately failable physical part (see
/// [`StaticAveragingLineFaults`]).
#[derive(Clone, Copy, Debug, Default)]
pub struct StaticAveragingLineFaults {
    /// The averaging line itself is blocked (corrosion, crimped tubing):
    /// `1.0` (>=0.98) fully blocked, isolating the two sides from each
    /// other (each port still works fine on its own -- this is not the
    /// same as either port being blocked).
    pub line_blocked: f64,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct StaticPortPairOutput {
    /// The pressure the ADR actually receives after averaging (or, with
    /// the line blocked, the left port's own reading -- see below).
    pub averaged_pa: f64,
    /// True whenever the averaging benefit is not fully available: either
    /// port is blocked (the average has fallen back to the healthy side
    /// alone) or the averaging line itself is blocked (the sides are
    /// isolated).
    pub degraded: bool,
}

/// Combines a system's already-computed left/right [`StaticPortOutput`]s
/// (from two independent [`StaticPort`] instances) into the single
/// pressure the ADR receives.
///
/// - Both ports healthy: a straight 50/50 average (cancelling first-order
///   sideslip position error, see the module docs above).
/// - One port blocked: the average falls back entirely to the healthy
///   side's reading -- the averaging line's own orifice is small next to
///   the port's, so a fully sealed port's now-frozen pressure has
///   negligible pull on the shared node once it can no longer breathe
///   (the same small-orifice-dominates reasoning [`StaticPort::step`]'s
///   own leak model uses, just between two ports here instead of a port
///   and cabin pressure).
/// - The averaging line itself blocked: the two sides are pneumatically
///   isolated from each other. Neither port's own reading is invalidated
///   by this (this function reports the left side's own reading as the
///   representative value in that case), but the sideslip-cancellation
///   benefit averaging exists for is lost -- `degraded` flags this.
pub fn average_pair(left: StaticPortOutput, right: StaticPortOutput, faults: &StaticAveragingLineFaults) -> StaticPortPairOutput {
    if faults.line_blocked.clamp(0.0, 1.0) >= 0.98 {
        return StaticPortPairOutput { averaged_pa: left.sensed_static_pressure_pa, degraded: true };
    }
    let (left_weight, right_weight) = match (left.blocked, right.blocked) {
        (true, false) => (0.0, 1.0),
        (false, true) => (1.0, 0.0),
        _ => (0.5, 0.5),
    };
    let averaged_pa = left_weight * left.sensed_static_pressure_pa + right_weight * right.sensed_static_pressure_pa;
    StaticPortPairOutput { averaged_pa, degraded: left.blocked || right.blocked }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn healthy_port_matches_true_static_pressure_at_reference_aoa() {
        let mut port = StaticPort::new(101_325.0);
        let out = port.step(95_000.0, 79_000.0, REFERENCE_AOA_DEG, 0.3, &StaticPortFaults::default(), 0.1);
        assert!((out.sensed_static_pressure_pa - 95_000.0).abs() < 1.0);
        assert!(!out.blocked);
    }

    #[test]
    fn position_error_flips_sign_either_side_of_reference_aoa() {
        let mut low = StaticPort::new(95_000.0);
        let mut high = StaticPort::new(95_000.0);
        let lo = low.step(95_000.0, 79_000.0, REFERENCE_AOA_DEG - 5.0, 0.3, &StaticPortFaults::default(), 0.1);
        let hi = high.step(95_000.0, 79_000.0, REFERENCE_AOA_DEG + 5.0, 0.3, &StaticPortFaults::default(), 0.1);
        assert!(lo.sensed_static_pressure_pa < 95_000.0);
        assert!(hi.sensed_static_pressure_pa > 95_000.0);
    }

    #[test]
    fn blocked_port_freezes_the_last_reading() {
        let mut port = StaticPort::new(95_000.0);
        let before = port.step(95_000.0, 79_000.0, REFERENCE_AOA_DEG, 0.3, &StaticPortFaults::default(), 0.1);
        let faults = StaticPortFaults { blocked: 1.0, ..Default::default() };
        // The aircraft climbs a lot; a blocked port must not follow.
        let mut out = before;
        for _ in 0..50 {
            out = port.step(40_000.0, 79_000.0, REFERENCE_AOA_DEG, 0.8, &faults, 0.1);
        }
        assert!(out.blocked);
        assert_eq!(out.sensed_static_pressure_pa, before.sensed_static_pressure_pa);
    }

    #[test]
    fn a_large_leak_pulls_the_reading_toward_cabin_pressure() {
        let mut port = StaticPort::new(30_000.0);
        let faults = StaticPortFaults { leak_to_cabin: 5.0, ..Default::default() };
        let mut out = StaticPortOutput::default();
        for _ in 0..500 {
            out = port.step(30_000.0, 79_000.0, REFERENCE_AOA_DEG, 0.8, &faults, 0.1);
        }
        // A leak conductance five times the port's own should land the
        // reading much closer to cabin pressure than to true static.
        assert!(out.sensed_static_pressure_pa > 65_000.0, "{}", out.sensed_static_pressure_pa);
    }

    #[test]
    fn no_leak_is_unaffected_by_cabin_pressure() {
        let mut port = StaticPort::new(95_000.0);
        let out = port.step(95_000.0, 40_000.0, REFERENCE_AOA_DEG, 0.3, &StaticPortFaults::default(), 0.1);
        assert!((out.sensed_static_pressure_pa - 95_000.0).abs() < 1.0);
    }

    #[test]
    fn partial_restriction_response_time_constant_is_independent_of_step_size() {
        // A fixed per-call blend fraction (the bug this test guards against)
        // would make the effective time constant depend on how often
        // `step` is called; the exact exponential must give (very nearly)
        // the same result after the same *elapsed time* regardless of how
        // it is chopped into steps.
        let faults = StaticPortFaults { blocked: 0.5, ..Default::default() };
        let mut coarse = StaticPort::new(101_325.0);
        let mut fine = StaticPort::new(101_325.0);
        // One time constant's worth of elapsed time (tau = 0.2 s here), so
        // the port is partway converged -- neither instantly at the target
        // nor untouched -- regardless of how that time is chopped up.
        let total_s = 0.2;
        let coarse_steps = 4; // dt = 0.05 s
        let fine_steps = 200; // dt = 0.001 s
        let mut coarse_out = StaticPortOutput::default();
        for _ in 0..coarse_steps {
            coarse_out = coarse.step(95_000.0, 79_000.0, REFERENCE_AOA_DEG, 0.3, &faults, total_s / coarse_steps as f64);
        }
        let mut fine_out = StaticPortOutput::default();
        for _ in 0..fine_steps {
            fine_out = fine.step(95_000.0, 79_000.0, REFERENCE_AOA_DEG, 0.3, &faults, total_s / fine_steps as f64);
        }
        assert!(
            (coarse_out.sensed_static_pressure_pa - fine_out.sensed_static_pressure_pa).abs() < 1.0,
            "coarse {} fine {}",
            coarse_out.sensed_static_pressure_pa,
            fine_out.sensed_static_pressure_pa
        );
        // And it should have made real (non-instant) progress, not zero
        // movement or exact-target-in-one-step -- otherwise the test above
        // would trivially pass by both stalling at the initial value.
        assert!((coarse_out.sensed_static_pressure_pa - 101_325.0).abs() > 1.0);
        assert!((coarse_out.sensed_static_pressure_pa - 95_000.0).abs() > 1.0);
    }

    #[test]
    fn no_nan_at_zero_dt_or_full_blockage() {
        let mut port = StaticPort::new(0.0);
        let faults = StaticPortFaults { blocked: 1.0, leak_to_cabin: 1.0 };
        let out = port.step(0.0, 0.0, 0.0, 0.0, &faults, 0.0);
        assert!(out.sensed_static_pressure_pa.is_finite());
    }

    // ---- Left/right averaging.

    #[test]
    fn both_ports_healthy_average_fifty_fifty() {
        let left = StaticPortOutput { sensed_static_pressure_pa: 95_000.0, blocked: false };
        let right = StaticPortOutput { sensed_static_pressure_pa: 95_200.0, blocked: false };
        let out = average_pair(left, right, &StaticAveragingLineFaults::default());
        assert!((out.averaged_pa - 95_100.0).abs() < 1e-6, "{}", out.averaged_pa);
        assert!(!out.degraded);
    }

    #[test]
    fn one_side_blocked_falls_back_fully_to_the_healthy_side() {
        let left_blocked = StaticPortOutput { sensed_static_pressure_pa: 80_000.0, blocked: true };
        let right_healthy = StaticPortOutput { sensed_static_pressure_pa: 95_000.0, blocked: false };
        let out = average_pair(left_blocked, right_healthy, &StaticAveragingLineFaults::default());
        assert!((out.averaged_pa - 95_000.0).abs() < 1e-6, "{}", out.averaged_pa);
        assert!(out.degraded);
    }

    #[test]
    fn a_blocked_averaging_line_isolates_the_sides_without_invalidating_either_reading() {
        let left = StaticPortOutput { sensed_static_pressure_pa: 95_000.0, blocked: false };
        let right = StaticPortOutput { sensed_static_pressure_pa: 96_000.0, blocked: false };
        let faults = StaticAveragingLineFaults { line_blocked: 1.0 };
        let out = average_pair(left, right, &faults);
        assert!(out.degraded);
        // Left side's own reading still used, not a frozen/garbage value.
        assert!((out.averaged_pa - 95_000.0).abs() < 1e-6, "{}", out.averaged_pa);
    }

    #[test]
    fn no_nan_in_average_pair_defaults() {
        let out = average_pair(StaticPortOutput::default(), StaticPortOutput::default(), &StaticAveragingLineFaults::default());
        assert!(out.averaged_pa.is_finite());
    }
}
