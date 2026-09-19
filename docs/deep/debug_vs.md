# Debug: PFD showing about -8800 ft/min vertical speed on the ground

Reported bug (real X-Plane 12 session): the PFD showed about -8800 ft/min
vertical speed with the aircraft on the ground and not descending.

## What the PFD's vertical speed actually is

The PFD's V/S tape reads FBW's own `A32NX_ADIRS_IR_n_VERTICAL_SPEED`
(`src/prim.rs:717`, `r.inertial_vertical_speed_ft_s = ir(&mut self.names,
vars, "VERTICAL_SPEED")`), which is FBW's *unmodified* `InertialReference`
(`D:\fbw-aircraft\fbw-common\...\navigation\adirs.rs`) simply re-publishing
whatever this plugin wrote to the `A32NX_ADIRS_SENSED_n_VERTICAL_SPEED`
simulator variable (`adirs.rs:634-636` of FBW's own file: "Feet per minute,
like `AdirsSimulatorData`'s own `vertical_speed`" — units checked and
confirmed consistent end-to-end, not a units bug). That value is written by
this plugin at `src/physics/adirs.rs:1361` (line numbers below are after the
fix):

```rust
w(vars, &self.v.vertical_speed, -self.v_down * M_TO_FT * 60.0);
```

`self.v_down` (m/s, positive down) is this `Adiru`'s own strapdown-mechanized
downward velocity estimate, corrected by a baro-inertial complementary filter
(`Adiru::baro_inertial_correct`). The root cause is in how that estimate is
initialised, not in any unit conversion or sign flip.

## Root cause

Two things combine:

**1. FBW's own ADIRS reports "Aligned" from literally the first tick.**
`InertialReference::new` (fbw-common's `navigation/adirs.rs:1952-1970`)
constructs with `remaining_align_duration: Some(Duration::from_secs(0))`
unconditionally (line 1958), and `is_fully_aligned()` (line 2588-2590) is
`self.remaining_align_duration == Some(Duration::ZERO)` — true immediately.
`InertialReferenceModeSelector::new` (line 232-240) likewise defaults its
mode to `InertialReferenceMode::Navigation`, not `Off`. The comment at
line 235-237 says why: *"We start in an aligned state to support starting on
the runway or in the air."* This is FBW's own, deliberate design (also
covered by its own test `starts_aligned`, line ~3777), read-only reference —
not something this plugin may edit.

The practical consequence for this plugin: `ADIRS_ADIRU_n_STATE` (this
plugin's `fbw_state` input, `src/physics/adirs.rs`'s `AdirsPhysics::update`)
reads `2` (`AlignState::Aligned`) essentially from the very first tick of
every single flight — cold-and-dark, ramp start, runway start, or airborne —
not only after a real 300 s alignment wait.

**2. This plugin's own `Adiru` assumed alignment always takes a while, so it
never bothered to sync its position/velocity state at the exact instant
mechanization starts.**

Before the fix, `Adiru::advance` (`src/physics/adirs.rs`) computed:

```rust
let should_run = fbw_state >= 1.999 && powered;
...
} else {
    if !self.running {
        // Just finished aligning: freeze whatever gyrocompass error
        // remains and start free-inertial dead reckoning from here.
        self.running = true;
    }
    self.mechanize(dt, t);
    self.baro_inertial_correct(dt, t);
    ...
```

The comment says the unit starts "free-inertial dead reckoning from here",
but the code never actually re-synced `alt_m`/`lat_rad`/`lon_rad`/`v_north`/
`v_east`/`v_down` to the current truth at that moment — it silently trusted
that the `!should_run` branch above (which *does* continuously copy
`t.lat_deg`/`lon_deg`/`alt_m` into the struct every tick while not running)
had already run at least once. That assumption is false whenever `fbw_state`
is already `>= 1.999` on `Adiru`'s very first `advance()` call — exactly what
point 1 guarantees on every normal flight start. `Adiru::new_with`
(`src/physics/adirs.rs`, constructor) defaults `alt_m: 0.`, `lat_rad: 0.`,
`lon_rad: 0.`, `v_north/v_east/v_down: 0.` — literal zeros, not "unknown".

So on the very first tick, `should_run` was already `true`, `self.running`
flips `false -> true`, and `mechanize`/`baro_inertial_correct` ran
immediately with `self.alt_m == 0.0` against the real truth altitude
(`t.alt_m`, typically tens to thousands of metres — a field elevation, not
0). `baro_inertial_correct` (`src/physics/adirs.rs:1180-1185`) is a
critically-damped 2nd-order loop with a **100 s** natural period
(`BARO_LOOP_PERIOD_S`):

```rust
let w = 2.0 * std::f64::consts::PI / BARO_LOOP_PERIOD_S;
self.alt_m += (2.0 * w * dt).min(1.0) * error;
self.v_down -= (w * w * dt).min(w) * error;
```

For a critically damped loop nulling out a one-shot error `e0`, the peak
implied rate is of order `w * e0`. With `w = 2*pi/100 ~= 0.0628 rad/s` and a
several-hundred-metre `e0` (a perfectly ordinary field elevation), `w * e0`
is tens of `m/s` — several **thousand** ft/min — sustained for a good
fraction of the loop's 100 s time constant while it decays back to zero. That
is the observed "-8800 ft/min, on the ground, not descending": a completely
synthetic transient from the baro-inertial loop nulling out an altitude error
that should never have existed, not a real sensed rate. The existing
`MAX_PLAUSIBLE_ALT_JUMP_M` clamp in the same function (20,000 m) does not
help here — a few hundred to a few thousand metres is well inside "plausible"
by design (it exists to reject X-Plane's pre-placement "millions of feet"
garbage, a different failure mode already fixed in an earlier pass; see the
`agl_looks_placed`/`last_valid_*` machinery in `TrueStateSource`).

This also explains "on a real X-Plane session" rather than only in a
synthetic test: it needs no reload, no saved situation, no unusual
configuration — it is latent on *every* flight start, and simply was not
caught by the existing unit tests because they all call
`Adiru::update_for_test`/`set_free_inertial_for_test` starting from `fbw_state
== 0` and stepping through `Aligning` first (matching the *comment's*
assumption, not FBW's actual `starts_aligned` behaviour), so `self.alt_m` was
always already synced to truth by the time their tests reached
`fbw_state == 2`.

## Fix

`src/physics/adirs.rs`:

1. Added `TrueState::placed: bool` (mirrors `agl_looks_placed`, set by
   `TrueStateSource::read` from the same `placed` local it already computed
   for `on_ground`). `should_run` now additionally requires `t.placed`, so
   the `!should_run` branch — the one that keeps `alt_m`/`lat_rad`/`lon_rad`
   synced to truth every tick — keeps running until X-Plane has handed this
   module a real position, regardless of how early FBW's own ADIRS reports
   "Aligned". Without this half of the fix, the first `!self.running -> true`
   transition could itself happen against X-Plane's still-frozen
   pre-placement truth (0/parked), latch `self.running = true`, and then miss
   the *real* jump a few ticks later when X-Plane actually places the
   aircraft (`self.running` never goes back to `false` on its own).

2. In the `if !self.running { ... }` transition block (the one the old
   comment said would "start free-inertial dead reckoning from here" but
   didn't actually implement), added the sync itself:

   ```rust
   self.running = true;
   self.pitch_deg = t.theta_xp;
   self.roll_deg = t.phi_xp;
   self.heading_deg = wrap_360(t.psi_xp + self.heading_error_deg);
   self.lat_rad = t.lat_deg.to_radians();
   self.lon_rad = t.lon_deg.to_radians();
   self.alt_m = t.alt_m;
   self.v_north = t.v_north_ms;
   self.v_east = t.v_east_ms;
   self.v_down = 0.0;
   ```

   Because `t.placed` now gates `should_run`, the tick this transition first
   fires on is guaranteed to carry a genuine, just-placed truth reading, so
   the baro-inertial loop's error is zero at the moment it starts — no
   transient to null out.

Tests added in `src/physics/adirs.rs` (`mod tests`):

- `adiru_starting_already_aligned_does_not_spike_vertical_speed_on_the_ground`
  — the direct regression test: a fresh `Adiru`, first-ever `advance()` call
  with `fbw_state = 2.0` (already "Aligned", exactly as FBW reports at spawn)
  and a real, non-zero truth altitude. Asserts the implied vertical speed
  stays under 100 fpm (before the fix this was many thousands of fpm) and
  that `alt_m` is synced to truth.
- `adiru_starting_already_aligned_also_syncs_position_heading_and_ground_speed`
  — same scenario, checking the other outputs the identical root cause also
  corrupted (see below).
- `should_run_waits_for_x_plane_to_place_the_aircraft_even_when_fbw_reports_aligned`
  — the `t.placed` gating half of the fix in isolation.

## Other ADIRS outputs wrong at spawn (same root cause, same fix)

Because the pre-fix code trusted stale/default struct fields whenever
`should_run` could turn true before any `!should_run` tick had run, every
other quantity `Adiru` derives from `lat_rad`/`lon_rad`/`v_north`/`v_east`/
`pitch_deg`/`roll_deg`/`heading_deg` had the same latent spawn bug, just less
dramatic-looking than the vertical speed spike (no equivalent of the
baro-inertial loop's 100 s "amplification" for these):

- **Ground speed / true track** (`src/physics/adirs.rs`, `write()`:
  `ground_speed_kt` from `v_north`/`v_east`; `track_deg` from
  `atan2(v_east, v_north)`). With `v_north = v_east = 0.` at spawn, a unit
  starting already-aligned *and already moving* (an "on the runway or in the
  air" start — exactly the case FBW's own comment names) would show 0 kt
  ground speed and an arbitrary `atan2(0, 0) == 0` degree track at the first
  tick(s), instead of the real value. Now fixed by syncing `v_north`/`v_east`
  to `t.v_north_ms`/`t.v_east_ms` in the same transition block.
- **Position (`LATITUDE`/`LONGITUDE`)**: `lat_rad`/`lon_rad` defaulted to
  `0.0`/`0.0` — Null Island, off the coast of west Africa — and would only
  have been dragged toward the real position by `gpirs_correct`'s
  complementary filter over `GPIRS_TIME_CONSTANT_S` (120 s). A nav display or
  FMS reading this during that window would show the aircraft roughly 0-120 s
  into a multi-thousand-mile "flight" toward its real position. Now fixed by
  syncing `lat_rad`/`lon_rad` in the same transition block.
- **Pitch/roll/heading**: `pitch_deg`/`roll_deg` defaulted to level (`0.0`)
  and `heading_deg` to `0.0` (true north) regardless of the aircraft's actual
  attitude at spawn (e.g. an aircraft parked nose-up on its gear, or an
  airborne start on any heading but 360). Now fixed by syncing all three
  (`heading_deg` including the still-applicable residual gyrocompass error,
  `heading_error_deg`, so an instantly-aligned unit still shows the same
  class of small heading uncertainty a real "quick align" would have).

All four are fixed by the single sync added in the `if !self.running { ... }`
block above, since they share the exact same trigger (mechanization starting
before this struct's fields were ever set from truth) — this is one root
cause, not four separate bugs, so one fix and one set of tests covers all of
them (`adiru_starting_already_aligned_also_syncs_position_heading_and_ground_speed`
checks position, velocity, pitch, roll and heading together).

## Files reviewed, not changed

- `src/sensors.rs`: reviewed for anything computing altitude/vertical-speed
  or holding persistent physics state that could glitch at spawn the same
  way. It only mirrors instantaneous X-Plane state (gear, radios, ILS,
  antiskid switch, cloud state) each tick; nothing it owns accumulates a
  filtered/integrated quantity the way `Adiru`'s baro-inertial loop does, so
  it has no equivalent spawn-transient class of bug. No changes needed.
- `src/xplane_mirror.rs`: reviewed the autopilot mirror
  (`sim/cockpit2/autopilot/vvi_dial_fpm` <- `A32NX_AUTOPILOT_VS_SELECTED`) —
  this is the FCU's *selected* V/S target, a pass-through of whatever value
  FBW's own systems already hold, not a sensed/filtered quantity this plugin
  computes; it carries no state of its own (`update` is a free function, no
  `Plugin` field, per the module's own doc comment) so it cannot itself
  produce a spurious transient. No changes needed.
