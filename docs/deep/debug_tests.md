# Debug: five failing unit tests + one intermittent one

Scope (per the debug brief): `src/extra_backend_fcdc.rs`, `src/physics/tyre.rs`,
`src/breakers.rs` (tests only), `src/failures.rs` (test-isolation helper only,
not needed in the end — `failures.rs` already had one). For each test: is the
test or the code wrong, fix the right one, cite evidence.

## 1-4. The four `extra_backend_fcdc` failures — one root cause, code was right, the test harness was wrong

- `fcdcs_come_up_with_the_prims_and_carry_the_master_law` (was failing at the
  `A32NX_FCDC_{n}_HEALTHY` assert, ~line 1232)
- `no_cpiom_no_healthy_fcdc` (~line 1274)
- `the_fcdc_failure_ids_reach_the_fcdc_through_the_global_failures_state`
  (~line 1297)
- `spoiler_lvars_and_fcdc_bits_follow_the_speedbrake_handle` (~line 1375)

### Cause

`ExtraBackendFcdc::update_fcdc` (`src/extra_backend_fcdc.rs:1130-1132`) gates
`Fcdc::update`'s `is_powered` argument on `A32NX_CPIOM_C{1,2}_AVAIL` alone:

```rust
let cpiom_available = n.is(vars, &format!("A32NX_CPIOM_C{}_AVAIL", fcdc_index + 1));
fcdc.update(sample_time, failure_active, cpiom_available);
```

This is exactly what FlyByWire's C++ does — verified against
`FlyByWireInterface.cpp:2299`:

```cpp
fcdcs[fcdcIndex].update(sampleTime, failuresConsumer.isActive(failureIndex), idCpiomCxAvailable[fcdcIndex]->get());
```

`idCpiomCxAvailable[i]` is literally `A32NX_CPIOM_C<n>_AVAIL`
(`FlyByWireInterface.cpp:781`), and in the real linked simulation that LVar is
written every tick by `CoreProcessingInputOutputModule::is_available()`
(`fbw-common/.../core_processing_input_output_module.rs:60,67`, `is_powered &
!failure_indication`, driven by the CPIOM's own `ElectricalBusType`). Inside
`Fcdc::update`, `is_powered == false` accumulates `power_supply_outage_time`
and after `MINIMUM_POWER_OUTAGE_TIME_FOR_FAILURE = 0.01s`
(`extra_backend_fcdc.rs:319,816-827`) sets `power_supply_fault = true`, which
makes `monitoring_healthy` false forever (`monitor_self`, line 807-809) — the
FCDC never gets a chance to run `startup()`/complete its self-test.

The test harness's `Rig` (`src/prim.rs`'s `tests::Rig`) only ticks
PRIMs/SECs/FADECs (`Rig::tick_with_failures`, `prim.rs:1976-1988`) — it never
runs the actual `a380_systems`/ADCN/CPIOM simulation that would publish
`A32NX_CPIOM_C1_AVAIL`/`_C2_AVAIL` in production. The tests' own `network_up()`
helper (`extra_backend_fcdc.rs`, before the fix at line ~1189) already stands
in for that missing simulation for the AFDX switch AVAIL/REACHABLE vars, but
never set the two CPIOM AVAIL vars — so `cpiom_available` read back `0.` (a
missing `MapVars` entry reads as `0.0`, see `prim.rs`'s `MapVars::read`) for
every FCDC, on every tick, in every test. That makes `is_powered` false from
tick 1, so both FCDCs are permanently "unhealthy" regardless of anything else
the tests do — explaining all four failures (word 1 healthy check never true;
FCDC 2 never reads back healthy even though nothing in that test touches it;
bus outputs never leave their `SSM_FW`/all-fail-warning state so the spoiler
armed/lever bits on word 4 never follow the handle).

### Fix

`src/extra_backend_fcdc.rs`, `tests::network_up()`: added
`vars.set("A32NX_CPIOM_C1_AVAIL", 1.); vars.set("A32NX_CPIOM_C2_AVAIL", 1.);`,
with a comment tying it to the same "this harness stands in for the real ADCN"
rationale already documented on that function. This is a test-only fix — the
production code already matched FlyByWire's real gating.

`no_cpiom_no_healthy_fcdc` additionally had its own, separate premise error
(not just the missing default): it turned off `A32NX_AFDX_SWITCH_3_AVAIL` /
`_13_AVAIL` and expected FCDC 1 to go unhealthy from that. But `afdxCommAvailable`
(cpp:2229-2230) only gates whether `updateFcdc` refreshes the FCDC's discrete/
bus inputs from the network (cpp:2233-2266) — it is never passed to
`fcdcs[fcdcIndex].update(...)` at all, so it cannot affect
`monitoring_healthy`/`fcdc_valid`. Only `idCpiomCxAvailable` does. Renamed the
test's intent to match its own name: now zeroes `A32NX_CPIOM_C1_AVAIL`
directly (leaving the AFDX vars from `network_up()` alone), which does make
FCDC 1 unhealthy (and, correctly, leaves FCDC 2 alone since only its own
CPIOM C2 was left available).

## 5. `physics::tyre::tests::slow_leak_plus_short_turnaround_together_melt_a_fuse_plug_that_neither_does_alone`

### Cause

The test's hand-derived "Phase B" (`src/physics/tyre.rs`, the big doc comment
above the test) closed-form prediction for the *decoupled* wheel's final
temperature froze `pressure_ratio` at its Phase-A-end value (`0.64`) for the
whole 250 s window. But `TyreWheel::step` (`tyre.rs:180-224`) keeps applying
the leak every tick regardless of phase — the test's own Phase B loop calls
`decoupled.step(own_temp, T_AMB, GROUNDSPEED_MS, MAGNITUDE, 1.0)` with
`MAGNITUDE = 0.8` (`tyre.rs`, test body), so `leaked_fraction` (and so
`pressure_ratio`, and so `flex_heat = ROLL_HEAT_COEFF*(1/pressure_ratio)*
groundspeed`) keeps moving through Phase B too (`0.64` → `0.54` over 250 s, a
~19% rise in flex-heat rate by the end). The frozen-ratio formula ignores that
rise entirely.

This is a genuine, not a cosmetic, gap: hand-integrating (Euler, dt=25s, 10
steps, by the coupling `dT/dt = flex(t) - k_cool*(T-20)` with `flex(t) =
0.3/(0.64 - 0.0004t)`) gives a decoupled final temperature around 134-136 C,
roughly 10-12 C above the frozen-ratio prediction of 123.7 C the test asserted
against with a tolerance of only 1 C — comfortably enough to fail
`(decoupled.temp_c - predicted_decoupled_final).abs() < 1.0` on every run.
`TyreWheel::step` itself is correct here (the leak really should keep running
through Phase B — that is the physically correct, continuous behaviour the
module's own doc comment describes); the test's math was incomplete.

### Fix

`src/physics/tyre.rs`, test module:
- Added `integrate_phase_b(t0, brake_temp, initial_pressure_ratio,
  soak_coupled)`, a 1-second Euler integration written independently of
  (never calling) `TyreWheel::step`, but using the same governing equations
  and the same per-tick order (leak first, then flex from the *updated*
  pressure ratio, then the temperature update) — this is what the closed-form
  `relax()` helper could not do for a time-varying coefficient with no
  elementary antiderivative (`∫ e^{k s}/(p0 - r s) ds`).
- `predicted_decoupled_final` (the one figure asserted to within 1 C) now
  comes from `integrate_phase_b(..., soak_coupled=false)` instead of the
  frozen-ratio `relax(...)`.
- Left `predicted_leaking_final`'s frozen-ratio estimate (203.2 C) in place:
  it is only used for a `>` melt-point sanity check, and freezing the ratio
  makes it a conservative *lower* bound (the real, continuing-leak trajectory
  only runs hotter), so the direction of that inequality still holds — no
  precision issue there since it is not compared for an exact value. Same for
  the no-leak control wheel (`pressure_ratio` is `1.0` throughout, so its
  closed form was already exact and untouched).
- Updated the doc comment above the test to explain the frozen-ratio
  approximation's scope (fine for the sanity checks, not for the tight
  decoupled-wheel assertion) instead of presenting it as exact.

Not a weakened assertion: the `< 1.0` tolerance on the decoupled wheel is
unchanged; the *prediction* feeding it was corrected instead.

## 6. `breakers::tests::pulling_a_breaker_activates_its_bridged_failure_and_reset_clears_it` (intermittent in the full parallel run)

### Cause

`crate::failures`' active/magnitude state is one process-wide `static STATE:
Mutex<State>` (`src/failures.rs:1117-1123`). `Failures::new()`
(`failures.rs:1377-1391`) unconditionally does `s.active.clear();
s.magnitudes.clear();` as part of its own reset — and this test calls
`crate::failures::Failures::new()` (`breakers.rs`, test body) with no
synchronization.

Separately, and more broadly: `Breakers::pre_systems`
(`breakers.rs:1277-1289`) loops over *every* live breaker in the catalogue
(100+ entries) and, for each one with a non-empty `failures` list, calls
`crate::failures::set_active(fail, !closed)` — including every breaker this
test never touches. Since most breakers start closed, this unconditionally
forces `set_active(<that breaker's failure id>, false)` for the whole
catalogue on every call, for any id that happens to already be
`registered` (which, once *any* test process-wide has called
`Failures::new()`, is effectively the full `a380_failures()` +
`COMPUTER_FAILURES` + `extra_failures()` list — see `failures.rs:1382-1383`).

`failures.rs` already documents this exact hazard and already has the fix
pattern for it: `#[cfg(test)] pub(crate) mod tests { pub static SERIAL:
Mutex<()>; }` (`failures.rs:1614-1623`, "The failure state is process-wide...
tests touching it take turns"), and `extra_backend_fcdc.rs`'s own
`the_fcdc_failure_ids_reach_the_fcdc_through_the_global_failures_state` test
already takes this lock for the same reason. `pulling_a_breaker_activates_
its_bridged_failure_and_reset_clears_it` was simply never given the same
treatment, so under `cargo test`'s default parallel threads it races any other
test in the crate that reads/writes `failures::STATE` concurrently (most
directly, its own sibling `extra_backend_fcdc` failure-id test) — whichever
side's `Failures::new()`/`set_active`/`pre_systems` call lands between this
test's `request_pull("tr-1")` and its `active_ids()` assert wins, and about
half the time that assert now sees the wrong set.

No fix was needed in `failures.rs`: the isolation helper the brief allowed
adding there already exists (`tests::SERIAL`).

### Fix

`src/breakers.rs`, tests module: added `let _g =
crate::failures::tests::SERIAL.lock().unwrap_or_else(|e| e.into_inner());` at
the top of `pulling_a_breaker_activates_its_bridged_failure_and_reset_clears_it`.

### Related latent bug found and fixed while looking at this

The same hazard applies to every other test in `breakers.rs` that constructs
a `Breakers` and calls `pre_systems` (which touches the *entire* catalogue's
worth of failure ids on the shared global every call) or calls
`Failures::new()`/`set_magnitude` directly, even though their own assertions
don't look at `failures::active_ids()`  — they can still silently clear or
overwrite a failure id armed by a *different*, concurrently-running test
elsewhere in the crate (most plausibly the `extra_backend_fcdc` FCDC/PRIM
failure-id tests, or `failures.rs`'s own tests). Took the same `SERIAL` lock
in:
- `pulling_an_absorbed_circuit_breaker_really_opens_its_systems_cfg_circuit`
  (calls `pre_systems` twice, touching every catalogued failure id each time)
- `pulling_a_plugin_var_breaker_writes_one_and_reset_writes_zero` (same)
- `bearing_wear_magnitude_predicts_current_by_the_back_emf_relation` (calls
  `Failures::new()` directly, plus `set_magnitude`)

Tests that only call `catalog()`/read-only static lists
(`every_id_is_unique`, `every_rating_is_positive_and_every_bus_resolves`,
`catalog_is_well_over_a_hundred`, `failure_ids_are_unique_across_the_whole_
catalog`, `every_bridged_failure_id_is_registered_in_failures_rs`,
`panel_node_breakers_reuse_a_real_panel_cb_node`) and tests that only call
`post_systems`/read `failures::magnitude()` without arming anything of their
own (`an_overloaded_breaker_trips_...`, `reset_all_closes_every_pulled_
breaker`, the real-current/thermal-ambient tests, `snapshot_reports_every_
field_...`) were left alone — they don't mutate `failures::STATE`'s
`active`/`magnitudes` maps, so they are not a source of this race (though a
concurrently-running racy test could in principle still perturb what
`bearing_overcurrent_multiplier` reads for them; that residual risk already
existed before this pass and is unchanged by it).

## Other things checked, not changed

- `extra_backend_fcdc.rs`: read the whole file end to end for further latent
  bugs beyond the four above; nothing else stood out against the cited FBW
  `cpp:` line ranges (bit conventions, master-PRIM selection, BTV/ROW/ROP
  logic, spoiler bits) — all matched their citations.
- `physics/tyre.rs`: the `invariants::check` divide-by-zero path
  (`flex_heat_raw` when `pressure_ratio == 0.0` after a fuse-plug melt sets
  `leaked_fraction = 1.0`) is caught and clamped to `0.0` by `invariants.rs`
  (non-finite values are always caught regardless of bound), exactly as the
  module's own doc comment on `step()` says it should be — not a bug, and not
  in the editable file set for this pass anyway.
