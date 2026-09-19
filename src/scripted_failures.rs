//! Scripted failure arming: trigger a catalogue id when a condition on
//! elapsed time, altitude, speed or flight phase becomes true, instead of
//! only by a manual arm or the MTBF random engine (`random_failures.rs`).
//! This is the "arm by time, altitude, speed, or flight phase" half of the
//! hyperrealism brief's trigger-infrastructure item; MTBF-random and
//! wear-driven (`physics/damage.rs`) arming already exist separately.
//!
//! A scripted trigger is one-shot: once its condition is met it fires
//! `failures::set_active(id, true)` (the caller does this, using the ids
//! this module returns) and is removed, the same way an instructor-station
//! "fail this at FL350" entry in a type-rating sim consumes itself once it
//! has fired. Re-arming means adding it again.
//!
//! Wired into `Plugin::tick` (`lib.rs`): each tick samples the real
//! `sim/cockpit2/gauges/indicators/altitude_ft_pilot`/
//! `sim/flightmodel/position/indicated_airspeed` X-Plane datarefs (the same
//! ones `physics/damage.rs` already reads for its own exceedance model)
//! and FlyByWire's own `A32NX_FMGC_FLIGHT_PHASE` LVar (`prim.rs`/`radios.rs`
//! already read this same LVar; `js/msfs/environment.js`'s
//! `FLIGHT_PHASE_*` constants are its published numbering, mirrored by
//! [`Phase::from_fmgc`]), and calls `failures::set_active(id, true)` for
//! every id `Scripted::update` returns.

#![allow(dead_code)] // Study-panel schedule/cancel API, not yet reachable from a page.

/// Flight phase, matching FlyByWire's own `A32NX_FMGC_FLIGHT_PHASE`
/// numbering (`js/msfs/environment.js`'s `FLIGHT_PHASE_*` constants: 0
/// preflight, 1 taxi, 2 takeoff, 3 climb, 4 cruise, 5 descent, 6 approach,
/// 7 go-around) rather than an invented phase list, so `OnFlightPhase`
/// arms against the real value the FMGC publishes.
#[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum Phase {
    Preflight,
    Taxi,
    Takeoff,
    Climb,
    Cruise,
    Descent,
    Approach,
    GoAround,
}

impl Phase {
    /// `A32NX_FMGC_FLIGHT_PHASE`'s own value (`js/msfs/environment.js`'s
    /// `FLIGHT_PHASE_*`) to a `Phase`, or `None` for a value outside that
    /// range (e.g. before the FMS has written it, `prim.rs`'s
    /// `A32NX LVars default to 0 until an FMS writes them` note -- 0 reads
    /// as `Preflight`, which is the correct phase before an FMS is up
    /// anyway, so no ambiguity there).
    pub fn from_fmgc(n: f64) -> Option<Phase> {
        match n.round() as i64 {
            0 => Some(Phase::Preflight),
            1 => Some(Phase::Taxi),
            2 => Some(Phase::Takeoff),
            3 => Some(Phase::Climb),
            4 => Some(Phase::Cruise),
            5 => Some(Phase::Descent),
            6 => Some(Phase::Approach),
            7 => Some(Phase::GoAround),
            _ => None,
        }
    }
}

/// The live values a condition is checked against, sampled once per tick
/// by the caller.
#[derive(Clone, Copy, Debug, Default)]
pub struct Sample {
    pub elapsed_hours: f64,
    pub altitude_ft: f64,
    pub speed_kt: f64,
    pub phase: Option<Phase>,
}

/// One arming condition. `Random` is deliberately not here: that is
/// `random_failures.rs`'s own MTBF engine, a continuously-live hazard
/// rather than a one-shot scripted arm.
#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub enum ArmCondition {
    /// Fires once elapsed real flight time reaches this many hours.
    ElapsedHours(f64),
    /// Fires once altitude climbs through this many feet.
    AboveAltitudeFt(f64),
    /// Fires once altitude descends through this many feet.
    BelowAltitudeFt(f64),
    /// Fires once indicated airspeed reaches this many knots.
    AboveSpeedKt(f64),
    /// Fires on entering this flight phase.
    OnFlightPhase(Phase),
}

impl ArmCondition {
    fn met(self, s: &Sample) -> bool {
        match self {
            ArmCondition::ElapsedHours(h) => s.elapsed_hours >= h,
            ArmCondition::AboveAltitudeFt(ft) => s.altitude_ft >= ft,
            ArmCondition::BelowAltitudeFt(ft) => s.altitude_ft <= ft,
            ArmCondition::AboveSpeedKt(kt) => s.speed_kt >= kt,
            ArmCondition::OnFlightPhase(p) => s.phase == Some(p),
        }
    }
}

/// One scheduled scripted trigger, persisted-shape (the Study panel builds
/// these; `persistence.rs` can round-trip `Vec<ScriptedTrigger>` the same
/// way it already round-trips `mel::Deferral`).
#[derive(Clone, Copy, Debug, serde::Serialize, serde::Deserialize)]
pub struct ScriptedTrigger {
    pub id: u64,
    pub condition: ArmCondition,
}

/// The live set of not-yet-fired scripted triggers.
#[derive(Default)]
pub struct Scripted {
    pending: Vec<ScriptedTrigger>,
}

impl Scripted {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn restore(&mut self, pending: Vec<ScriptedTrigger>) {
        self.pending = pending;
    }

    pub fn snapshot(&self) -> Vec<ScriptedTrigger> {
        self.pending.clone()
    }

    /// Schedule `id` to arm when `condition` is met. Replaces any existing
    /// scheduled trigger for the same id (one pending trigger per id).
    pub fn schedule(&mut self, id: u64, condition: ArmCondition) {
        self.pending.retain(|t| t.id != id);
        self.pending.push(ScriptedTrigger { id, condition });
    }

    /// Cancel a scheduled trigger before it fires. Returns whether one was
    /// removed.
    pub fn cancel(&mut self, id: u64) -> bool {
        let before = self.pending.len();
        self.pending.retain(|t| t.id != id);
        self.pending.len() != before
    }

    pub fn list(&self) -> &[ScriptedTrigger] {
        &self.pending
    }

    /// One tick's check: every pending trigger whose condition is now met
    /// fires (removed from `pending`, returned). The caller should apply
    /// `failures::set_active(id, true)` for each and log it, the same
    /// pattern `random_failures::update`'s callers use.
    pub fn update(&mut self, sample: &Sample) -> Vec<u64> {
        let (fired, still_pending): (Vec<_>, Vec<_>) = self.pending.drain(..).partition(|t| t.condition.met(sample));
        self.pending = still_pending;
        fired.into_iter().map(|t| t.id).collect()
    }
}

/// The Study panel's "schedule a failure" action: queued the same way
/// `mel::request_defer`/`random_failures::request_config` are, since the
/// page has no direct access to the running `Scripted` (owned by
/// `Plugin`).
enum Request {
    Schedule(u64, ArmCondition),
    Cancel(u64),
}

static REQUESTS: std::sync::Mutex<Vec<Request>> = std::sync::Mutex::new(Vec::new());

pub fn request_schedule(id: u64, condition: ArmCondition) {
    if let Ok(mut r) = REQUESTS.lock() {
        r.push(Request::Schedule(id, condition));
    }
}

pub fn request_cancel(id: u64) {
    if let Ok(mut r) = REQUESTS.lock() {
        r.push(Request::Cancel(id));
    }
}

/// Test-isolation helper (see `scenarios::reset_global_state`): drops any
/// queued schedule/cancel request left over from a previous test.
#[cfg(any(test, feature = "test-support"))]
pub fn reset_for_tests() {
    if let Ok(mut r) = REQUESTS.lock() {
        r.clear();
    }
}

impl Scripted {
    /// Drain the Study panel's queued schedule/cancel requests. Call once
    /// per tick, same pattern as `mel::Mel::apply_requests`.
    pub fn apply_requests(&mut self) -> Vec<String> {
        let mut log = Vec::new();
        for req in REQUESTS.lock().map(|mut r| std::mem::take(&mut *r)).unwrap_or_default() {
            match req {
                Request::Schedule(id, condition) => {
                    self.schedule(id, condition);
                    log.push(format!("failure {id} scheduled: {condition:?}"));
                }
                Request::Cancel(id) => {
                    if self.cancel(id) {
                        log.push(format!("scheduled failure {id} cancelled"));
                    }
                }
            }
        }
        log
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sample(elapsed_hours: f64, altitude_ft: f64, speed_kt: f64, phase: Option<Phase>) -> Sample {
        Sample { elapsed_hours, altitude_ft, speed_kt, phase }
    }

    #[test]
    fn elapsed_hours_fires_once_the_threshold_is_reached() {
        let mut s = Scripted::new();
        s.schedule(1, ArmCondition::ElapsedHours(2.0));
        assert!(s.update(&sample(1.0, 0., 0., None)).is_empty());
        assert_eq!(s.update(&sample(2.0, 0., 0., None)), vec![1]);
        // One-shot: gone from `pending`, never fires again.
        assert!(s.update(&sample(3.0, 0., 0., None)).is_empty());
        assert!(s.list().is_empty());
    }

    #[test]
    fn altitude_and_speed_thresholds_fire_on_crossing() {
        let mut s = Scripted::new();
        s.schedule(10, ArmCondition::AboveAltitudeFt(35_000.0));
        s.schedule(11, ArmCondition::BelowAltitudeFt(1_000.0));
        s.schedule(12, ArmCondition::AboveSpeedKt(250.0));
        let fired = s.update(&sample(0., 36_000.0, 260.0, None));
        assert_eq!(fired.len(), 2, "altitude-above and speed-above fire, altitude-below does not");
        assert!(fired.contains(&10));
        assert!(fired.contains(&12));
        assert!(!fired.contains(&11));
    }

    #[test]
    fn flight_phase_fires_only_on_the_matching_phase() {
        let mut s = Scripted::new();
        s.schedule(20, ArmCondition::OnFlightPhase(Phase::Approach));
        assert!(s.update(&sample(0., 0., 0., Some(Phase::Cruise))).is_empty());
        assert_eq!(s.update(&sample(0., 0., 0., Some(Phase::Approach))), vec![20]);
    }

    #[test]
    fn rescheduling_the_same_id_replaces_the_pending_condition() {
        let mut s = Scripted::new();
        s.schedule(1, ArmCondition::ElapsedHours(5.0));
        s.schedule(1, ArmCondition::ElapsedHours(1.0));
        assert_eq!(s.list().len(), 1, "still one pending trigger for id 1");
        assert_eq!(s.update(&sample(1.0, 0., 0., None)), vec![1], "the replaced (1h) condition is the one that applies");
    }

    #[test]
    fn cancel_removes_a_pending_trigger_before_it_fires() {
        let mut s = Scripted::new();
        s.schedule(1, ArmCondition::ElapsedHours(1.0));
        assert!(s.cancel(1));
        assert!(!s.cancel(1), "already gone");
        assert!(s.update(&sample(10.0, 0., 0., None)).is_empty());
    }

    #[test]
    fn snapshot_and_restore_round_trip() {
        let mut s = Scripted::new();
        s.schedule(7, ArmCondition::AboveAltitudeFt(10_000.0));
        let snap = s.snapshot();
        let mut restored = Scripted::new();
        restored.restore(snap);
        assert_eq!(restored.list().len(), 1);
        assert_eq!(restored.update(&sample(0., 20_000.0, 0., None)), vec![7]);
    }

    #[test]
    fn apply_requests_schedules_and_cancels() {
        let mut s = Scripted::new();
        request_schedule(1, ArmCondition::ElapsedHours(1.0));
        let log = s.apply_requests();
        assert_eq!(log.len(), 1);
        assert_eq!(s.list().len(), 1);
        request_cancel(1);
        s.apply_requests();
        assert!(s.list().is_empty());
    }
}
