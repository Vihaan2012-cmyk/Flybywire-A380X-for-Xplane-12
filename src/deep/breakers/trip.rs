//! Trip physics for every breaker in [`super::catalog`]: a thermal I^2t
//! bimetal model with ambient-temperature compensation, a magnetic
//! instantaneous trip, and a separate solid-state power controller (SSPC)
//! electronic trip curve with arc-fault detection -- the real A380
//! Electrical Load Management System mixes both technologies (conventional
//! thermal-magnetic breakers for higher-current feeders/motors, SSPCs with
//! remote reset via the Cockpit Display System for the majority of smaller
//! avionics/valve/solenoid circuits; real, publicly documented architecture,
//! e.g. Airbus/TE Connectivity/Data Device Corporation solid-state power
//! distribution literature for this aircraft generation).
//!
//! Self-contained per `docs/deep/BRIEF.md` hard rule 2: this is an
//! independent re-derivation of the same *class* of I^2t bimetal curve
//! `crate::physics::electrical::trip_step` already implements for
//! `src/breakers.rs`'s existing catalogue (real precedent in this codebase
//! for the modelling approach), not an import of it or of anything else in
//! the crate -- nothing outside `src/deep/breakers` references this module
//! yet, and this module references nothing outside its own directory.
//!
//! Two continuous health faults, `docs/deep/BRIEF.md`'s required pair:
//! - `trip_calibration_drift` (0 healthy .. 1 fully drifted) -- a bimetal
//!   spring's aging fatigue, or an SSPC's reference/firmware drift, that
//!   lowers the *effective* trip threshold below the breaker's true rating,
//!   so it opens under a load it should carry: a nuisance trip.
//! - `contact_resistance` (0 healthy .. 1 fully welded/pitted) -- repeated
//!   arcing pits and eventually welds a breaker's output contacts (a
//!   documented aerospace contactor/breaker failure mode); this raises how
//!   much I^2t heat / how far past the magnetic multiple the current must
//!   go before the trip mechanism can still force the welded contacts
//!   apart, up to "never" as the fault approaches 1.0: a fails-to-trip.

/// Reference ambient the healthy curve and every catalogue rating assume,
/// deg C -- the datum MIL-PRF-39019 (the real, published US mil-spec for
/// aircraft thermal circuit breakers) quotes its own ampere ratings at.
pub const REFERENCE_AMBIENT_C: f64 = 25.0;

/// MIL-PRF-39019 publishes an ambient-temperature current-derating curve for
/// thermal aircraft circuit breakers: the current a part can carry all day
/// without nuisance-tripping falls as ambient rises, on the order of 15%
/// lower at its published 71 C high-temperature end point than at the 25 C
/// reference point (real, publicly documented class of curve). The exact
/// per-part-number curve is proprietary to the manufacturer, so this is a
/// GENERIC linear interpolation between those two public end points, then
/// clamped so an ambient outside the documented -55..71 C qualification
/// range cannot extrapolate the derate past its own end values.
fn thermal_ambient_derate(ambient_c: f64) -> f64 {
    let c = ambient_c.clamp(-55.0, 71.0);
    let above_ref = ((c - REFERENCE_AMBIENT_C) / (71.0 - REFERENCE_AMBIENT_C)).max(0.0);
    1.0 - 0.15 * above_ref
}

/// GENERIC: a typical thermal-magnetic aircraft breaker's magnetic
/// (instantaneous, no intentional time delay) element fires somewhere in
/// the 8-15x rated-current range on a real short circuit; 10x is the
/// mid-point used here. A real SSPC's own hardware overcurrent comparator
/// serves the identical instantaneous-protection role and is given the same
/// multiple for consistency (no public per-part figure differs enough to
/// justify a second constant).
const MAGNETIC_TRIP_MULTIPLE: f64 = 10.0;

/// SSPC arc-fault detection proxy. A real arc-fault detector analyses
/// high-frequency current noise/spectral signature (well beyond this
/// simulation's fidelity); this uses the tick-to-tick current slope as a
/// GENERIC stand-in for that signature -- a step larger than this many
/// amps/second, while carrying a meaningful fraction of rated current, is
/// treated as an arcing event and trips instantly, independent of the
/// I^2t/magnetic elements. This is the one behaviour a real bimetal breaker
/// categorically cannot do, reflected here by gating it on `kind == Sspc`.
const ARC_FAULT_DI_DT_A_PER_S: f64 = 500.0;

/// SSPC time constant for the emulated I^2t curve: a microprocessor-timed
/// curve is deliberately tighter/more repeatable than a bimetal's
/// mechanical one (a real, documented SSPC advantage -- precise, consistent
/// trip time against a bimetal's much wider manufacturing tolerance band).
/// GENERIC values, chosen so the SSPC curve is visibly faster than the
/// thermal one at the same overload ratio, not calibrated to a specific
/// datasheet.
const SSPC_TAU_S: f64 = 8.0;
const THERMAL_TAU_S: f64 = 20.0;

/// A real SSPC's own microprocessor logic latches into a maintenance-only
/// lockout after repeated trips in a short window, rather than letting the
/// flight crew (or an automated test harness) simply cycle a genuinely
/// faulted circuit indefinitely from the CDS/OIT CB page -- real,
/// documented solid-state power controller behaviour (TE Connectivity/Data
/// Device Corporation SSPC application literature: "trip lockout"/"nuisance
/// trip lockout" is a named feature of this device class). A conventional
/// thermal breaker has no such logic -- it has no microprocessor to latch
/// anything, a human simply keeps finding it tripped. GENERIC thresholds:
/// 3 trips within a rolling 300 s (5 min) window locks it out.
const LOCKOUT_TRIP_COUNT: usize = 3;
const LOCKOUT_WINDOW_S: f64 = 300.0;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BreakerKind {
    Thermal,
    Sspc,
}

/// Live SSPC status for a CDS/OIT CB-page style display -- a thermal
/// breaker has no remote status reporting at all (its `closed`/`trip_cause`
/// fields are only readable by this simulation, not by any real cockpit
/// system), so this is meaningful for `BreakerKind::Sspc` only, though
/// [`Breaker::status`] returns it for either kind for convenience.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SspcStatus {
    Closed,
    /// Open because it was *commanded* open (a real "OPEN" pushbutton/CDS
    /// command, or a plain mechanical pull), not because it tripped.
    OpenCommanded,
    Tripped(TripCause),
    /// Repeated trips within the lockout window: open, and refuses
    /// `remote_reset`/`reset` until [`Breaker::maintenance_clear_lockout`].
    LockedOut,
}

/// Why a CDS/OIT-style remote command was refused.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RemoteControlError {
    /// A conventional thermal breaker has no remote-control interface at
    /// all -- it must be pushed back in by hand at its own physical panel.
    NotRemoteCapable,
    /// Locked out after repeated trips; needs
    /// [`Breaker::maintenance_clear_lockout`], not a flight-deck reset.
    LockedOut,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TripCause {
    None,
    /// I^2t thermal element (bimetal, or an SSPC's emulated thermal model).
    Thermal,
    /// Instantaneous magnetic (or SSPC hardware overcurrent comparator).
    Magnetic,
    /// SSPC-only arc-fault detection.
    ArcFault,
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct BreakerFaults {
    pub trip_calibration_drift: f64,
    pub contact_resistance: f64,
}

/// One breaker's live trip state. `catalog::BreakerDef` is the static
/// (rating/basis/panel) description; this is the per-instance runtime
/// model a consumer of this module ticks.
pub struct Breaker {
    kind: BreakerKind,
    rated_a: f64,
    heat: f64,
    prev_current_a: f64,
    pub closed: bool,
    pub trip_cause: TripCause,
    /// Running simulation clock this instance has been ticked for, s --
    /// the timestamp base for the lockout window (SSPC only; stays at 0.0
    /// and unused for a thermal breaker).
    elapsed_s: f64,
    /// Timestamps (in `elapsed_s`) of trips still inside the lockout
    /// window; pruned every trip. Empty for a thermal breaker (it never
    /// records into this).
    trip_times_s: Vec<f64>,
    locked_out: bool,
}

impl Breaker {
    pub fn new(kind: BreakerKind, rated_a: f64) -> Self {
        Self { kind, rated_a, heat: 0.0, prev_current_a: 0.0, closed: true, trip_cause: TripCause::None, elapsed_s: 0.0, trip_times_s: Vec::new(), locked_out: false }
    }

    /// Manual reset at the breaker's own physical panel position: always
    /// available for a thermal breaker (a human just pushes it back in);
    /// for an SSPC this is the same maintenance-level action as
    /// [`Self::maintenance_clear_lockout`] would otherwise require, so it
    /// is refused while locked out -- a real SSPC in lockout needs the
    /// underlying fault actually fixed, not just a reset attempt, cooling
    /// its thermal element the way a real bimetal strip cools with current
    /// removed.
    pub fn reset(&mut self) -> Result<(), RemoteControlError> {
        if self.locked_out {
            return Err(RemoteControlError::LockedOut);
        }
        self.closed = true;
        self.heat = 0.0;
        self.trip_cause = TripCause::None;
        Ok(())
    }

    /// CDS/OIT CB-page remote close command. SSPC only -- a real thermal
    /// breaker has no remote-control interface at all.
    pub fn remote_reset(&mut self) -> Result<(), RemoteControlError> {
        if self.kind != BreakerKind::Sspc {
            return Err(RemoteControlError::NotRemoteCapable);
        }
        self.reset()
    }

    /// CDS/OIT CB-page remote open command: commands the breaker open
    /// without it being a fault trip (real A380 SSPCs can be commanded
    /// open from the CDS/OIT CB page for maintenance isolation, not only
    /// reset). SSPC only.
    pub fn remote_open(&mut self) -> Result<(), RemoteControlError> {
        if self.kind != BreakerKind::Sspc {
            return Err(RemoteControlError::NotRemoteCapable);
        }
        self.closed = false;
        Ok(())
    }

    /// A physical pull at the breaker's own panel position: always
    /// available for either kind (this is the one action every real
    /// breaker, thermal or solid-state, still supports by hand).
    pub fn pull(&mut self) {
        self.closed = false;
    }

    /// Ground-maintenance-level action (via the aircraft's maintenance
    /// system, not available from the flight deck CDS) that clears a
    /// lockout latch once the underlying fault has actually been
    /// addressed. Real SSPC application practice: repeated-trip lockout is
    /// a deliberate barrier against a flight crew simply cycling a
    /// genuinely faulted circuit closed again and again.
    pub fn maintenance_clear_lockout(&mut self) {
        self.locked_out = false;
        self.trip_times_s.clear();
    }

    pub fn is_locked_out(&self) -> bool {
        self.locked_out
    }

    /// Live status for a CDS/OIT CB-page style display.
    pub fn status(&self) -> SspcStatus {
        if self.locked_out {
            SspcStatus::LockedOut
        } else if self.closed {
            SspcStatus::Closed
        } else if self.trip_cause == TripCause::None {
            SspcStatus::OpenCommanded
        } else {
            SspcStatus::Tripped(self.trip_cause)
        }
    }

    pub fn heat_fraction(&self) -> f64 {
        self.heat
    }

    /// Record a trip against the rolling lockout window (SSPC only -- a
    /// thermal breaker has no microprocessor to count with) and latch
    /// lockout once [`LOCKOUT_TRIP_COUNT`] trips fall inside
    /// [`LOCKOUT_WINDOW_S`] of each other.
    fn record_trip(&mut self) {
        if self.kind != BreakerKind::Sspc {
            return;
        }
        self.trip_times_s.push(self.elapsed_s);
        let window_start = self.elapsed_s - LOCKOUT_WINDOW_S;
        self.trip_times_s.retain(|&t| t >= window_start);
        if self.trip_times_s.len() >= LOCKOUT_TRIP_COUNT {
            self.locked_out = true;
        }
    }

    /// Common tail for every trip site below: sets the cause, records it
    /// against the lockout window, and reports a fresh trip.
    fn trip(&mut self, cause: TripCause) -> bool {
        self.closed = false;
        self.trip_cause = cause;
        self.record_trip();
        true
    }

    /// Advance one tick given the real current this breaker is carrying
    /// (A), its own bay's ambient temperature (deg C), and its two health
    /// faults. Returns whether it tripped *this* tick (it may already have
    /// been open; this only reports a fresh trip event).
    pub fn step(&mut self, current_a: f64, ambient_c: f64, faults: BreakerFaults, dt_s: f64) -> bool {
        let current_a = current_a.max(0.0);
        let dt = dt_s.max(0.0);
        self.elapsed_s += dt;
        if self.locked_out {
            // A locked-out SSPC stays open no matter what a flight-deck
            // reset attempt does; only maintenance can clear it.
            self.closed = false;
        }
        if !self.closed {
            self.prev_current_a = current_a;
            return false;
        }

        let drift = faults.trip_calibration_drift.clamp(0.0, 1.0);
        let weld = faults.contact_resistance.clamp(0.0, 1.0);

        // Calibration drift lowers the effective rating the curve reacts
        // to -- up to 40% low at full drift (GENERIC: a real drifted part
        // is out of tolerance, but a breaker that drifted further than that
        // would already have been pulled at the last maintenance check).
        // The thermal ambient derate stacks on top of it; a real SSPC's
        // electronic curve does not need an ambient term (its own
        // datasheet-documented advantage: temperature-stable trip point).
        let ambient_factor = if self.kind == BreakerKind::Thermal { thermal_ambient_derate(ambient_c) } else { 1.0 };
        let effective_rated_a = (self.rated_a * (1.0 - 0.4 * drift) * ambient_factor).max(1e-6);
        let ratio = current_a / effective_rated_a;

        // Welded/pitted contacts do not have a single failure point; the
        // more fused they are, the more thermal/magnetic force it takes to
        // still force them open. This must diverge as `weld` approaches
        // 1.0 (not just scale by a fixed multiple) -- a fixed multiple
        // would still eventually trip given a large enough sustained
        // overload, which is not what "welded shut" means physically.
        // `1 / (1 - weld)` gives exactly that: 1x (no change) at weld = 0,
        // ~1000x approaching weld = 1 (clamped at 0.999 so the divide stays
        // finite/NaN-free), so a fully welded breaker's I^2t/magnetic
        // threshold sits far above anything a sustained overload can reach
        // -- "never trips", continuously reached as the fault approaches
        // its limit rather than a hard cutoff. Applied identically to both
        // the magnetic multiple and the I^2t trip threshold below.
        let weld_factor = 1.0 / (1.0 - weld.min(0.999));
        let magnetic_multiple = MAGNETIC_TRIP_MULTIPLE * weld_factor;

        let di_dt = (current_a - self.prev_current_a).abs() / dt.max(1e-6);
        self.prev_current_a = current_a;

        if ratio >= magnetic_multiple {
            return self.trip(TripCause::Magnetic);
        }

        if self.kind == BreakerKind::Sspc && di_dt > ARC_FAULT_DI_DT_A_PER_S && current_a > effective_rated_a * 0.5 {
            return self.trip(TripCause::ArcFault);
        }

        // I^2t heat accumulator: heats with (I/Ir)^2 above 1, cools
        // exponentially otherwise, so a brief inrush does not trip it but a
        // sustained overload does -- the defining behaviour of a real
        // thermal breaker (and of an SSPC's own emulated thermal model,
        // which real parts run for exactly this reason: compatibility with
        // downstream wiring's own I^2t withstand rating).
        let tau_s = if self.kind == BreakerKind::Thermal { THERMAL_TAU_S } else { SSPC_TAU_S };
        let heat_in = if ratio > 1.0 { ratio * ratio - 1.0 } else { 0.0 };
        let cool = self.heat / tau_s;
        self.heat = (self.heat + (heat_in - cool) * dt).max(0.0);

        if self.heat >= weld_factor {
            self.trip(TripCause::Thermal)
        } else {
            false
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_healthy_breaker_at_rest_never_trips_and_never_produces_nan() {
        let mut b = Breaker::new(BreakerKind::Thermal, 10.0);
        for _ in 0..1000 {
            let tripped = b.step(0.0, REFERENCE_AMBIENT_C, BreakerFaults::default(), 0.0);
            assert!(!tripped);
            assert!(!b.heat_fraction().is_nan());
        }
        assert!(b.closed);
    }

    #[test]
    fn a_sustained_overload_trips_the_thermal_element_but_a_brief_inrush_does_not() {
        let mut b = Breaker::new(BreakerKind::Thermal, 10.0);
        // 1.5x rated for 0.5 s: a real inrush-class transient, should not trip.
        for _ in 0..50 {
            assert!(!b.step(15.0, REFERENCE_AMBIENT_C, BreakerFaults::default(), 0.01));
        }
        assert!(b.closed);
        // Same 1.5x sustained for much longer: must eventually trip.
        let mut tripped = false;
        for _ in 0..20_000 {
            if b.step(15.0, REFERENCE_AMBIENT_C, BreakerFaults::default(), 0.01) {
                tripped = true;
                break;
            }
        }
        assert!(tripped, "a sustained 1.5x overload never tripped the thermal element");
        assert_eq!(b.trip_cause, TripCause::Thermal);
        assert!(!b.closed);
    }

    #[test]
    fn the_trip_curve_is_monotonic_time_to_trip_falls_as_overload_current_rises() {
        fn time_to_trip(multiple: f64) -> f64 {
            let mut b = Breaker::new(BreakerKind::Thermal, 10.0);
            let mut t = 0.0;
            for _ in 0..200_000 {
                t += 0.01;
                if b.step(10.0 * multiple, REFERENCE_AMBIENT_C, BreakerFaults::default(), 0.01) {
                    return t;
                }
            }
            f64::INFINITY
        }
        let t_1_2 = time_to_trip(1.2);
        let t_1_5 = time_to_trip(1.5);
        let t_2_0 = time_to_trip(2.0);
        let t_5_0 = time_to_trip(5.0);
        assert!(t_1_2 > t_1_5, "{t_1_2} should be slower to trip than {t_1_5}");
        assert!(t_1_5 > t_2_0, "{t_1_5} should be slower to trip than {t_2_0}");
        assert!(t_2_0 > t_5_0, "{t_2_0} should be slower to trip than {t_5_0}");
    }

    #[test]
    fn magnetic_trip_fires_instantly_on_a_hard_short_regardless_of_thermal_history() {
        let mut b = Breaker::new(BreakerKind::Thermal, 10.0);
        let tripped = b.step(200.0, REFERENCE_AMBIENT_C, BreakerFaults::default(), 0.01);
        assert!(tripped);
        assert_eq!(b.trip_cause, TripCause::Magnetic);
    }

    #[test]
    fn calibration_drift_causes_a_nuisance_trip_at_a_current_a_healthy_breaker_would_carry_all_day() {
        let healthy_faults = BreakerFaults::default();
        let drifted_faults = BreakerFaults { trip_calibration_drift: 1.0, contact_resistance: 0.0 };
        let mut healthy = Breaker::new(BreakerKind::Thermal, 10.0);
        let mut drifted = Breaker::new(BreakerKind::Thermal, 10.0);
        // 0.9x rated: a healthy breaker must never trip on this.
        for _ in 0..50_000 {
            assert!(!healthy.step(9.0, REFERENCE_AMBIENT_C, healthy_faults, 0.01));
        }
        let mut drifted_tripped = false;
        for _ in 0..50_000 {
            if drifted.step(9.0, REFERENCE_AMBIENT_C, drifted_faults, 0.01) {
                drifted_tripped = true;
                break;
            }
        }
        assert!(drifted_tripped, "full calibration drift should nuisance-trip well below true rated current");
    }

    #[test]
    fn welded_contacts_fail_to_trip_even_on_a_severe_sustained_overload() {
        let welded_faults = BreakerFaults { trip_calibration_drift: 0.0, contact_resistance: 1.0 };
        let mut b = Breaker::new(BreakerKind::Thermal, 10.0);
        for _ in 0..500_000 {
            b.step(30.0, REFERENCE_AMBIENT_C, welded_faults, 0.01);
        }
        assert!(b.closed, "fully welded contacts must not open even under a severe sustained overload");
    }

    #[test]
    fn an_sspc_trips_faster_than_a_thermal_breaker_at_the_same_overload() {
        fn time_to_trip(kind: BreakerKind) -> f64 {
            let mut b = Breaker::new(kind, 10.0);
            let mut t = 0.0;
            for _ in 0..200_000 {
                t += 0.01;
                if b.step(20.0, REFERENCE_AMBIENT_C, BreakerFaults::default(), 0.01) {
                    return t;
                }
            }
            f64::INFINITY
        }
        assert!(time_to_trip(BreakerKind::Sspc) < time_to_trip(BreakerKind::Thermal));
    }

    #[test]
    fn an_sspc_trips_on_a_fast_current_step_an_arc_fault_signature_even_below_its_thermal_curve() {
        let mut b = Breaker::new(BreakerKind::Sspc, 10.0);
        // One steady tick to establish a baseline, then a very fast step.
        assert!(!b.step(2.0, REFERENCE_AMBIENT_C, BreakerFaults::default(), 0.01));
        let tripped = b.step(9.0, REFERENCE_AMBIENT_C, BreakerFaults::default(), 0.001);
        assert!(tripped);
        assert_eq!(b.trip_cause, TripCause::ArcFault);
    }

    #[test]
    fn a_thermal_breaker_never_detects_an_arc_fault_it_has_no_such_capability() {
        let mut b = Breaker::new(BreakerKind::Thermal, 10.0);
        assert!(!b.step(2.0, REFERENCE_AMBIENT_C, BreakerFaults::default(), 0.01));
        let tripped = b.step(9.0, REFERENCE_AMBIENT_C, BreakerFaults::default(), 0.001);
        // May or may not trip thermally this tick, but it must never be ArcFault.
        if tripped {
            assert_ne!(b.trip_cause, TripCause::ArcFault);
        }
    }

    #[test]
    fn ambient_heat_makes_a_thermal_breaker_trip_sooner_than_at_reference_ambient() {
        fn time_to_trip(ambient_c: f64) -> f64 {
            let mut b = Breaker::new(BreakerKind::Thermal, 10.0);
            let mut t = 0.0;
            for _ in 0..500_000 {
                t += 0.01;
                if b.step(13.0, ambient_c, BreakerFaults::default(), 0.01) {
                    return t;
                }
            }
            f64::INFINITY
        }
        assert!(time_to_trip(71.0) < time_to_trip(REFERENCE_AMBIENT_C));
    }

    #[test]
    fn reset_clears_heat_and_trip_cause_and_recloses() {
        let mut b = Breaker::new(BreakerKind::Thermal, 10.0);
        b.step(200.0, REFERENCE_AMBIENT_C, BreakerFaults::default(), 0.01);
        assert!(!b.closed);
        b.reset().unwrap();
        assert!(b.closed);
        assert_eq!(b.trip_cause, TripCause::None);
        assert_eq!(b.heat_fraction(), 0.0);
    }

    #[test]
    fn a_thermal_breaker_has_no_remote_control_interface_at_all() {
        let mut b = Breaker::new(BreakerKind::Thermal, 10.0);
        b.step(200.0, REFERENCE_AMBIENT_C, BreakerFaults::default(), 0.01);
        assert_eq!(b.remote_reset(), Err(RemoteControlError::NotRemoteCapable));
        assert_eq!(b.remote_open(), Err(RemoteControlError::NotRemoteCapable));
        // The physical reset still works -- only remote control is refused.
        assert!(b.reset().is_ok());
    }

    #[test]
    fn an_sspc_can_be_remotely_reset_and_remotely_opened_from_the_cds() {
        let mut b = Breaker::new(BreakerKind::Sspc, 10.0);
        b.step(200.0, REFERENCE_AMBIENT_C, BreakerFaults::default(), 0.01);
        assert!(!b.closed);
        assert_eq!(b.status(), SspcStatus::Tripped(TripCause::Magnetic));
        b.remote_reset().unwrap();
        assert!(b.closed);
        assert_eq!(b.status(), SspcStatus::Closed);
        b.remote_open().unwrap();
        assert!(!b.closed);
        assert_eq!(b.status(), SspcStatus::OpenCommanded, "commanded open, not tripped");
    }

    #[test]
    fn repeated_trips_within_the_lockout_window_latch_an_sspc_out_and_a_flight_deck_reset_cannot_clear_it() {
        let mut b = Breaker::new(BreakerKind::Sspc, 10.0);
        for i in 0..LOCKOUT_TRIP_COUNT {
            assert!(!b.is_locked_out(), "should not be locked out before trip {i}");
            let tripped = b.step(200.0, REFERENCE_AMBIENT_C, BreakerFaults::default(), 0.01);
            assert!(tripped, "trip {i} should have tripped");
            if i + 1 < LOCKOUT_TRIP_COUNT {
                b.remote_reset().unwrap();
            }
        }
        assert!(b.is_locked_out(), "3 trips within the lockout window should latch it out");
        assert_eq!(b.status(), SspcStatus::LockedOut);
        assert_eq!(b.remote_reset(), Err(RemoteControlError::LockedOut));
        assert_eq!(b.reset(), Err(RemoteControlError::LockedOut));
        // Stays open even if something calls step() again.
        b.step(0.0, REFERENCE_AMBIENT_C, BreakerFaults::default(), 0.01);
        assert!(!b.closed);
    }

    #[test]
    fn maintenance_clear_lockout_lets_a_locked_out_sspc_be_reset_again() {
        let mut b = Breaker::new(BreakerKind::Sspc, 10.0);
        for i in 0..LOCKOUT_TRIP_COUNT {
            b.step(200.0, REFERENCE_AMBIENT_C, BreakerFaults::default(), 0.01);
            if i + 1 < LOCKOUT_TRIP_COUNT {
                b.remote_reset().unwrap();
            }
        }
        assert!(b.is_locked_out());
        b.maintenance_clear_lockout();
        assert!(!b.is_locked_out());
        b.remote_reset().unwrap();
        assert!(b.closed);
    }

    #[test]
    fn trips_spaced_outside_the_lockout_window_do_not_accumulate_toward_lockout() {
        let mut b = Breaker::new(BreakerKind::Sspc, 10.0);
        for i in 0..LOCKOUT_TRIP_COUNT {
            b.step(200.0, REFERENCE_AMBIENT_C, BreakerFaults::default(), 0.01);
            assert!(!b.is_locked_out(), "trip {i} alone should never lock it out");
            b.remote_reset().unwrap();
            // Let the lockout window fully elapse before the next trip.
            b.step(0.0, REFERENCE_AMBIENT_C, BreakerFaults::default(), LOCKOUT_WINDOW_S + 1.0);
        }
        assert!(!b.is_locked_out(), "trips spaced outside the window must not latch a lockout");
    }
}
