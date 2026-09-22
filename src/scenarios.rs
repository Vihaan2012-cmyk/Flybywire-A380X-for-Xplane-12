//! Shared test-only setup for cross-system scenario tests: multi-failure
//! intersection tests that couple two or more of fuel/hydraulics/engine/
//! electrical at once (docs/briefs/hyperrealism.md's "continuous, partial,
//! interacting failures" contract).
//!
//! # Why this exists
//!
//! `cargo test` runs every `#[test]` in this crate in one process, by
//! default on multiple threads. Several modules keep failure/wear/breaker
//! state in process-global `static ... Mutex<...>` items (not per-test,
//! per-instance state), because that is how the running plugin's own
//! Study-panel threads talk to the ticking simulation:
//!
//! - `failures::STATE` -- the active failure set and each one's continuous
//!   magnitude (`failures::set_active`/`set_magnitude`).
//! - `breakers`'s and `circuits`'s own request queues -- pulled/reset
//!   breakers not yet applied.
//! - `random_failures::CONFIG_REQUEST` -- a queued MTBF-engine config.
//! - `scripted_failures`'s request queue -- queued schedule/cancel.
//! - `physics::damage::LATEST_WEAR`/`REPAIR_REQUESTS` -- the Study panel's
//!   published per-engine wear snapshot and any queued repair.
//!
//! A test that activates a failure (or sets a magnitude, or queues a
//! breaker pull) and never clears it leaks that state into whatever test
//! the harness happens to run next in the same process -- an
//! order-dependent failure this workstream has already hit once with
//! several agents writing multi-system failure tests concurrently.
//!
//! A test's own model instances (`Engine`, `FuelNetwork`, `Damage`, ...)
//! are NOT global -- they are owned locally by each test, so constructing a
//! fresh one already starts clean. Only the `static`s above need explicit
//! clearing.
//!
//! # Usage
//!
//! Call [`reset_global_state`] as the **first** line of every scenario
//! test's setup, before calling any failure/breaker/wear API:
//!
//! ```ignore
//! #[test]
//! fn valve_restriction_plus_pump_derate_crosses_the_feed_margin() {
//!     scenarios::reset_global_state();
//!     // ... build the coupled model, arm x%/y%/z%, tick, assert ...
//! }
//! ```
//!
//! It is safe to call from any test in any file in this crate; it only
//! touches the global queues/state listed above.

#![cfg(any(test, feature = "test-support"))]

/// Clears every process-global failure/wear/breaker-request `static` this
/// crate defines, so a scenario test starts from a known-clean slate
/// regardless of what any previously-run test in this process left active.
///
/// Extending this: when a new module adds its own global failure/wear/
/// request `static`, give that module a `#[cfg(test)] pub fn
/// reset_for_tests()` (see `failures::reset_for_tests` for the pattern) and
/// call it from here. Keep this function itself infallible (never panics,
/// swallows a poisoned-lock `Err` the same way each module's own accessors
/// already do) so one broken module can never take down every other test's
/// setup.
pub fn reset_global_state() {
    crate::failures::reset_for_tests();
    crate::breakers::reset_for_tests();
    crate::circuits::reset_for_tests();
    crate::random_failures::reset_for_tests();
    crate::scripted_failures::reset_for_tests();
    crate::physics::damage::reset_for_tests();
    crate::physics::motor::reset_for_tests();
    crate::components::reset_all();
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Proves the helper actually clears what it claims to, using
    /// `failures::STATE` as the representative case: activate a real
    /// registered failure id, confirm it reads back active, reset, confirm
    /// it is gone. 24_020 is cited by `failures::trigger_condition` as an
    /// MTBF-eligible id, so it is guaranteed to be in `a380_failures()`'s
    /// registered set once a `Failures` has been constructed.
    #[test]
    fn reset_global_state_clears_failure_activation() {
        // This test asserts the GLOBAL active set is empty, and resets it.
        // `failures::STATE` is process-global, so this test and any other
        // touching it must take turns -- the same lock breakers.rs already
        // uses for exactly this reason.
        let _g = crate::failures::tests::serial();
        let _f = crate::failures::Failures::new();
        crate::failures::set_active(24_020, true);
        assert!(crate::failures::active_ids().contains(&24_020), "setup: id should be active before reset");
        reset_global_state();
        assert!(crate::failures::active_ids().is_empty(), "reset_global_state must clear active failures");
        assert_eq!(crate::failures::magnitude(24_020), 0.0, "reset_global_state must clear magnitude back to inactive");
    }

    /// Same proof for the breaker request queue: a queued pull must not
    /// survive a reset.
    #[test]
    fn reset_global_state_clears_queued_breaker_requests() {
        // `failures::STATE` is process-wide and this test reaches it (directly, or
        // through model code such as `Breakers::pre_systems` / `Damage::arm`).
        let _serial = crate::failures::tests::serial();
        crate::breakers::request_pull("TEST-NONEXISTENT-BREAKER".to_string());
        reset_global_state();
        // No public "peek the queue" accessor exists (by design: only the
        // owning `Breakers` instance drains it), so this only proves the
        // call does not panic and clears without leaving a stale lock; the
        // module-internal drain in breakers.rs's own tests covers the
        // queue's content directly.
    }
}
