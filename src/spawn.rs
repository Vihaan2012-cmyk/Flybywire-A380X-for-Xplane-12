//! Process-wide "which spawn is this" counter, for any settle/debounce guard
//! elsewhere in this crate that needs "how long has it been since THIS
//! flight began" rather than "how long has the plugin DLL been loaded".
//!
//! `XPluginEnable` builds the whole aircraft exactly once per X-Plane
//! session (`lib.rs`'s `Plugin::new`, called from `XPluginEnable`); X-Plane
//! does not unload/reload the plugin for a same-aircraft reposition,
//! restart-flight, or airport change, so a struct field initialised only at
//! construction time (a settle guard's own "seconds since `Foo::new`") gets
//! settle protection on the first spawn of the session and none at all on
//! any later one, because nothing in production ever told that struct a
//! new flight had begun. `XPluginReceiveMessage` (`lib.rs`) is the X-Plane
//! callback that carries exactly this notification
//! (`XPLM_MSG_PLANE_LOADED` for the user's own aircraft,
//! `XPLM_MSG_AIRPORT_LOADED` on every reposition/restart-flight) and used
//! to be an empty stub.
//!
//! A construction-time settle guard should keep its own `age_s`, reset to
//! `0.` whenever [`epoch`] no longer matches the value it last saw, rather
//! than accumulating unconditionally from its own `new()` (see
//! `efb.rs`'s `SPAWN_SETTLE_S`/`moving_disconnect_armed` for the pattern).
//! A guard built on a live signal's own persistence instead -- holding for
//! N seconds, `efb.rs`'s `GPU_SETTLE_S` debounce on the GPU flag, or
//! `physics::damage.rs`'s `time_airborne_s` (reset every tick `on_ground`
//! reads true) -- does not need this module at all: it already re-arms
//! itself on every spawn without being told, reposition or not.

use std::ffi::c_int;
use std::sync::atomic::{AtomicU64, Ordering};

/// X-Plane plugin message IDs this module cares about (`XPLMPlugin.h`,
/// `developer.x-plane.com/sdk/XPLMPlugin`): `inParam` for
/// `XPLM_MSG_PLANE_LOADED` is the aircraft index being loaded, 0 the
/// user's own aircraft (every other index is a different AI plane, not
/// us); `XPLM_MSG_AIRPORT_LOADED` has no meaningful `inParam` and fires on
/// every reposition/restart-flight even to the same airport.
pub const XPLM_MSG_PLANE_LOADED: c_int = 102;
pub const XPLM_MSG_AIRPORT_LOADED: c_int = 103;

static SPAWN_EPOCH: AtomicU64 = AtomicU64::new(0);

/// Marks the start of a fresh spawn. Call this, and only this, from
/// `XPluginReceiveMessage` on the two messages above -- see the module doc.
pub fn mark_new_spawn() {
    SPAWN_EPOCH.fetch_add(1, Ordering::Relaxed);
}

/// The current spawn epoch. A settle guard compares this against the
/// value it last saw and, on a change, resets its own `age_s` to `0.`
/// before accumulating `delta` into it again.
pub fn epoch() -> u64 {
    SPAWN_EPOCH.load(Ordering::Relaxed)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn mark_new_spawn_strictly_advances_the_epoch() {
        // `>` rather than `== + 1`: this is process-global state
        // (`invariants.rs`'s own `ELAPSED_SECS` doc comment explains why
        // that matters), and `cargo test` runs this crate's tests in
        // parallel threads, so another test bumping the same epoch between
        // these two reads must not fail this one.
        let e0 = epoch();
        mark_new_spawn();
        assert!(epoch() > e0, "epoch must strictly increase after mark_new_spawn");
    }
}
