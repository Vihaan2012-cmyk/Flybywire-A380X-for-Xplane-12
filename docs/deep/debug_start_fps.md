# Debug: 900 C EGT on a stopped engine, ~4 fps, and a NaN-producing start-state test

Scope (per the debug brief): may edit `src/start_state.rs`, `src/fadec.rs`,
`src/state_dump.rs`, `src/lib.rs` (flight-loop scheduling / dump gating only).
Read-only-but-important: `src/engine_commands.rs`,
`src/physics/engine/mod.rs` (`idle_engine()`, the engines-running spawn),
`src/xp.rs`. Never touch `D:\fbw-aircraft` (FlyByWire's own ported source).

## A. ~900 C EGT with the engines not started

### Root cause: a pre-existing, self-documented start-transient overshoot in the physics engine's fuel/combustor coupling, displayed with almost no trim

`ENGINE_EGT:n` is written twice a tick and the second writer wins (module
docs, `fadec.rs:35-46`): `fadec.rs`'s own polynomial estimate
(`update_egt`, `fadec.rs:1218-1241`) runs first and is unconditionally
overwritten later the same tick by `engine_commands.rs`'s physical value
(`engine_commands.rs:505-507`):

```rust
let takeoff_rating = matches!(o.thrust_limit_type, 3 | 4); // FLEX, TOGA
let trim = crate::physics::damage::tgt_trim_c(takeoff_rating);
let ambient_c = ambient_temp_k - 273.15;
let egt_displayed = (phys.egt_c - trim).max(phys.egt_c.min(ambient_c));
vars.write(&e.egt_untrimmed, phys.egt_c);
vars.write(&e.egt, egt_displayed);
```

`trim` (`physics/damage.rs::tgt_trim_c`) is only 56-89 C
(`TGT_TAKEOFF_UNTRIMMED_C - TGT_TAKEOFF_C = 956-900`, or
`TGT_MAX_CONTINUOUS_UNTRIMMED_C - TGT_MAX_CONTINUOUS_C = 939-850`,
`physics/damage.rs:38-50`) — nowhere near enough to mask what
`phys.egt_c` (`physics/engine::Engine::step`, `mod.rs:944`,
`egt_c: self.egt_lag_c`) can actually reach during a normal engine start.
I verified with `fadec.rs`'s and `engine_commands.rs`'s own logic (both
read-only, traced but not edited) that a **stopped, un-started** engine
(`fuel_valve_open = master == false`) cannot produce this on its own:
`governor.rs::step` returns `0.0` fuel flow outright when the valve is shut
(`governor.rs:194-204`), the combustor then reproduces inlet temperature
unchanged (`combustor.rs:35-36`, `no_fuel_leaves_temperature_unchanged`
test), and `Engine::step` resets `egt_lag_c` to the frame's real ambient on
its very first call regardless of history (`mod.rs:554-558`,
`!self.soaked`). So a genuinely off engine cannot read 900 C through this
path — the 900 C has to come from an engine that is (or was moments
earlier) actually combusting, at low N3/N1 ("not started" in the sense of
not yet at idle), during a real start the pilot commanded.

That is exactly what the engine model's own test suite already documents,
unfixed, as a known gap:

`physics/engine/mod.rs:1176-1224`,
`a_ground_idle_start_reaches_idle_within_the_sourced_start_time`:

```rust
// **Known gap, reported rather than forced**: peak start EGT here
// (`peak_egt_c`) comes out around 1100-1200°C, well above the
// certificated Trent 900 continuous TGT (850°C) and even its
// 920°C over-temperature limit -- not asserted against that
// bound here because the fuel authority needed to climb the HP
// compressor's own drag hump (the field bug this start law fixes)
// pushes fuel flow up against `params::MAX_COMBUSTOR_FUEL_AIR_RATIO`
// (0.08, near-stoichiometric) while core airflow is still small at
// ~20-30% N3, and `combustor.rs`'s energy balance has no ceiling of
// its own on the resulting T4/EGT at that fuel-air ratio.
assert!(peak_egt_c.is_finite() && peak_egt_c > 0.0, "{peak_egt_c}");
```

`governor.rs:37-50` (the governor's own module docs) and
`physics/engine/params.rs:72-91` (`MAX_COMBUSTOR_FUEL_AIR_RATIO`'s own doc
comment) independently confirm the same mechanism and name the same two
possible fixes: "tightening the FAR backstop and/or giving `combustor.rs`
its own temperature ceiling." At ~20-30% N3 the fan/N1 is still very low —
easily read by a pilot as "the engine hasn't started" — while the TGT probe
is already past 900 C on its way to the documented 1100-1200 C peak. The
user's "~900 C" sits squarely inside that transient.

### Why I did not patch this myself

The fix lives in `governor.rs`/`combustor.rs`/`physics/engine/params.rs` and
`physics/engine/mod.rs` — none are in this task's editable set (`mod.rs` is
explicitly read-only-but-important), and the brief bars builds/tests, so I
cannot verify a physics retune against the model's own extensive test suite
(idle-time, hot-day, mass-conservation, etc.) before handing it back. A
governor retune (reducing `KP`/the accel schedule) risks missing the
CS-E 745/14 5-second idle-to-TOGA requirement the same file cites as already
tuned against (`governor.rs:37-50`); tightening
`MAX_COMBUSTOR_FUEL_AIR_RATIO` is explicitly documented as *not* the
intended fix (`params.rs:87-90`, "a backstop, not the thing actually
keeping a cold start's fuel flow realistic"). The lowest-risk, most
surgical option — the one that does not touch spool/thrust dynamics at
all — is a ceiling on the derived TGT *probe* reading only, downstream of
where torque/thrust are already computed from `comb.tt4_k`:

```diff
--- a/src/physics/engine/mod.rs
+++ b/src/physics/engine/mod.rs
@@ struct Engine already has: egt_lag_c: f64,
+// A generic ceiling on the displayed/measured TGT probe reading only —
+// downstream of every torque/thrust calculation in `step` (which all use
+// `comb.tt4_k` directly, untouched by this), so this cannot perturb spool
+// dynamics, thrust, or any of this module's existing calibrated tests
+// (idle-reach-time, hot-day, mass-conservation). GENERIC: comfortably
+// above the Trent 900's own 920 C over-temperature limit (EASA E.012) so
+// a real transient exceedance is still visible and still trips
+// `physics/damage.rs`'s exceedance tracking, but below the ~1100-1200 C
+// this model's sub-idle start law is documented to reach
+// (`a_ground_idle_start_reaches_idle_within_the_sourced_start_time`,
+// this file) pending the governor/combustor retune that test already
+// calls for.
+const EGT_PROBE_CEILING_C: f64 = 1000.0;
@@ in step(), immediately after:
-        self.egt_lag_c += (egt_raw_c - self.egt_lag_c) * (1.0 - (-dt / egt_tau_s).exp());
+        self.egt_lag_c += (egt_raw_c - self.egt_lag_c) * (1.0 - (-dt / egt_tau_s).exp());
+        self.egt_lag_c = self.egt_lag_c.min(EGT_PROBE_CEILING_C);
```

Plus a unit test alongside the existing ones in that `mod.rs` test module:

```rust
#[test]
fn a_ground_start_never_displays_past_the_generic_probe_ceiling() {
    let mut engine = Engine::new();
    let mut inputs = EngineInputs { target_n1_corrected_pct: IDLE_N1_PCT, ..isa_sea_level() };
    let mut out = EngineOutputs::default();
    for _ in 0..(90.0 / inputs.dt_s) as usize {
        inputs.starter_engaged = out.n3_pct < starter::CUTOFF_N3_FRAC * 100.0;
        out = engine.step(&inputs);
        assert!(out.egt_c <= EGT_PROBE_CEILING_C + 1e-9, "{}", out.egt_c);
    }
}
```

This is a mitigation, not the real fix — the underlying start-law/FAR gap
(`governor.rs`/`combustor.rs`) should still get the proper retune the
model's own comments already call for, with the full test suite run
against it (which this task cannot do).

### A second, related, and independently real bug I did fix: `spawn_at_idle()` was dead code

While tracing "how a takeoff/runway start state initialises engines (spawn
at idle vs off)" I found `EngineCommands::spawn_at_idle`
(`engine_commands.rs:217-223`) — which puts every physical engine at
`physics::engine::idle_engine()`, "the state a spawn with engines running
starts each engine in, as a simulator's in-flight or engines-running spawn
does" (`physics/engine/mod.rs:482-486`) — was **never called from the real
plugin**. `EngineCommands::new` (`engine_commands.rs:269-271`) always
builds `[Engine::new(); 4]` (stone cold), and the only caller of
`spawn_at_idle` was the offline test harness (`offline_chain.rs:86`), not
`Plugin::new` (`lib.rs`).

Consequence: for any start state other than Hangar/Apron (Taxi, Runway,
Climb, Cruise, Approach, Final — "every state but Hangar and Apron starts
with the engines running", `start_state.rs:10-14`), `fadec.rs`'s own state
machine promotes every engine straight to `On` on the first real tick
(`fadec.rs`'s `next_state`, `Off` arm: `igniter == 1 && starter && sim_n3 >
20.` — true immediately for a genuinely-running spawn) while the physical
engine sits at `Engine::new()`'s N1=N2=N3=0. Because state is already `On`
and not `Starting`/`Restarting`, `engine_commands.rs`'s own
`starter_engaged` gate (`engine_commands.rs:445-448`) never latches, so the
cold core has no torque source at all (`governor.rs` requires
`combustion_floor_met`, itself requiring spool speed the starter would
normally provide) and stays at 0 % forever — the cockpit's masters and
`ENGINE_STATE` say running throughout, but N1/N2/N3 never move. This is a
second, independently real bug in the same neighbourhood the brief
pointed at.

**Fix applied** (`src/lib.rs`, `Plugin::new`, right after constructing
`engine_commands`):

```rust
let mut engine_commands = engine_commands::EngineCommands::new(&mut vars, xplm);
if !matches!(start_state, StartState::Hangar | StartState::Apron) {
    engine_commands.spawn_at_idle();
}
```

This calls an already-implemented, already-cached (`OnceLock`), previously
untriggered public API exactly as its own doc comment says it is meant to
be used, so it carries very little risk: it changes nothing for a cold
start (the common case this bug report was not about) and gives every
"engines running" spawn a physics engine that is actually at idle from
tick 1, instead of a permanently-stuck-at-zero one.

## B. ~4 fps and the state dump

### 1. `xphfbw.stateDumps` defaults to on — fix needed in a file outside this task's scope

`state_dump.rs`'s own module docs say dumps write "every ...
`xphfbw.stateDumpFrames` frames ... default [`DEFAULT_EVERY_TICKS`]"
(200) — not free, but not the main cost per tick either (see below). The
default that actually matters is `AppSettings::state_dumps`, which is
`true` (`src/app_settings.rs:74-86`):

```rust
impl Default for AppSettings {
    fn default() -> Self {
        Self {
            ...
            state_dumps: true,
```

`app_settings.rs` is not in this task's editable file list, so I did not
touch it. Exact patch for the lead/build agent to apply:

```diff
--- a/src/app_settings.rs
+++ b/src/app_settings.rs
@@ impl Default for AppSettings
-            state_dumps: true,
+            state_dumps: false,
```

`app_settings.rs:298` already has `assert!(!s.state_dumps);` in
`parses_every_key_the_app_writes` but that is asserting a `false` value
*explicitly written* in the test's own map, not the default — flipping the
`Default` impl does not touch that test.

### 2. What I did fix in the allowed files

**a. The dump-gating call locked the shared snapshot `Mutex` every tick,
even with dumps off** (`src/lib.rs`, `Plugin::tick`). `EVERY_TICKS` is
`1` (`state_dump.rs:28`, "the real interval ... is checked live inside
`StateDump::tick` itself"), so `self.ticks % state_dump::EVERY_TICKS == 0`
was always true and `snapshot().lock()` — the same `Mutex` the panel
thread reads — ran unconditionally every tick, purely to hand
`StateDump::tick` a reference it discards on its very next line whenever
`xphfbw.stateDumps` is off. Added `StateDump::enabled()`
(`src/state_dump.rs:75-87`, an `Option::is_some` plus one small `RwLock`
read, no disk I/O — see `app_settings::current`'s own doc comment) and
gated the lock on it (`src/lib.rs`, the `state_dump::EVERY_TICKS` call
site): the snapshot lock is now only taken on a tick that could actually
produce a dump.

**b. `msfs_derived` allocated and hashed four `String`s a tick to look up
identifiers that never change after the first tick** (`src/lib.rs`). It
called `vars.get("FCU_FD_LIGHT_ON".to_string())`,
`vars.get(format!("AUTOPILOT FLIGHT DIRECTOR ACTIVE:{n}"))` (×2) and
`vars.get("GEAR TOTAL PCT EXTENDED".to_string())` — four heap allocations
plus four `HashMap<String, _>` lookups (`Vars::add`,
`self.ids.get(&name)`), every tick, for values whose `VariableIdentifier`
is fixed after the very first call. Moved all three identifiers into the
`Computed` struct (built once in `Plugin::new`, the same place
`yaw_moi`/`pitch_moi`/`is_ready` already live) and changed `msfs_derived`
to read the cached IDs directly — no allocation, no string hashing, same
observable behaviour (identical variable names, so identical
`VariableIdentifier`s).

**c. A per-tick `Vec` allocation for a four-element check**
(`src/fadec.rs`, `update_thrust_limits`):

```rust
let tla_at_flex = self.engines.iter().map(|e| e.tla).collect::<Vec<_>>();
let all_at_flex = tla_at_flex.iter().all(|id| vars.read(id) == 35.0);
```

heap-allocated a 4-element `Vec<VariableIdentifier>` every tick only to
immediately iterate it once. Replaced with a direct iterator `.all()` over
`self.engines` — identical result, zero allocation.

### 3. Other per-frame costs found, not fixed (outside this task's file scope, or too invasive to change untested)

- `Plugin::tick` unconditionally stages `self.persistence.state.wear =
  wear::snapshot(); ... self.persistence.state.components =
  components::snapshot_direct(); self.persistence.state.deferred =
  self.mel.snapshot();` (`src/lib.rs`, in the tick body, ~20 lines before
  the state-dump call) **every tick**, even though `Persistence::tick`
  only actually writes to disk periodically (`src/persistence.rs:200-206`,
  `SAVE_INTERVAL_S` gate, `since_save`). If `wear::snapshot`/
  `components::snapshot_direct` clone anything nontrivial, that is a
  real per-tick allocation spent on data that is thrown away on every
  tick but the rare save one. Fixing this properly needs a public
  "due to save soon" accessor added to `persistence.rs`'s private
  `since_save` state, which is not in this task's editable set.
- `Plugin::tick` builds `let mut already =
  std::collections::BTreeSet::from_iter(failures::active_ids());` plus an
  `.extend(...)` every tick regardless of whether any random/MEL failure
  logic has anything to do that tick — a real but likely small allocation
  (bounded by the active-failure count); touching it means touching
  `failures.rs`/`random_failures.rs`, outside this task's scope.
- `Plugin::take_snapshot` (`src/lib.rs`) clears and rebuilds
  `snapshot.values`/`snapshot.sources` from every registered variable's
  slot, every tick — `O(total variable count)` copies each frame. This is
  needed for the live panel (not just dumps) and is not a bug by itself,
  but it is the single largest fixed per-tick cost in `lib.rs` proportional
  to how many variables this plugin publishes (likely several thousand for
  the full A380 system set); flagging it for awareness rather than
  changing it, since altering it risks the panel silently going stale.

## C. The failing NaN test

`start_state::tests::a_cold_and_dark_apron_start_then_apu_start_never_produce_nan_runaway_or_a_state_a_cold_aircraft_could_not_be_in`

### Cause

The test builds `Simulation`/`aspects::a380` directly against a bespoke
`TestVars` (`aspects.rs::test_vars`, `#[derive(Default)]`, every unset
variable reads a bare `0.0`) instead of going through FlyByWire's own
`SimulationTestBed`. Their own harness
(`fbw-common/src/wasm/systems/systems/src/simulation/test.rs:263-269`,
`SimulationTestBed::new_with_start_state`) always seeds two atmospheric
values before ticking *anything*, for every single one of their own
systems tests:

```rust
test_bed.set_ambient_temperature(ThermodynamicTemperature::new::<degree_celsius>(0.));
test_bed.set_ambient_pressure(Pressure::new::<inch_of_mercury>(29.92));
```

Our `start_state.rs` test already had to seed `TOTAL WEIGHT` for exactly
this class of bug (`start_state.rs`, the existing comment: dividing by a
genuinely-zero weight in `wing_flex.rs`'s ground-weight ratio is `0/0` =
NaN). `AMBIENT PRESSURE` left at `TestVars`'s default `0.0` is the same
shape of hazard, just wider-reaching: `UpdateContext::ambient_pressure()`
(`update_context.rs:600-601,786-788`) reads it straight off `AMBIENT
PRESSURE` with no clamp, and the ported electrical/air-conditioning/
pneumatic code divides by it in more than one place — e.g.
`air_cycle_machine.rs:363-364`,
`rho_ambient = context.ambient_pressure().get::<pascal>() / (R_AIR *
context.ambient_temperature().get::<kelvin>())`, and multiple
pressure-ratio terms through `pneumatic.rs`. A pressure of exactly `0.0`
is a vacuum no ported formula was written to see, and it is present for
the *entire* two-minute run (both the cold-and-dark phase and the APU
start phase), unlike the real plugin, which always supplies a real
barometric pressure from X-Plane
(`lib.rs`'s own `mapping()`, `"AMBIENT PRESSURE" => (
"sim/weather/aircraft/barometer_current_pas", ...)`) and FlyByWire's own
test harness, which always supplies the same 29.92 inHg baseline.

### Fix

`src/start_state.rs`, the test, immediately after the existing
`vars.set("TOTAL WEIGHT", 600_000.0);`:

```rust
vars.set("AMBIENT PRESSURE", 29.92);
vars.set("AMBIENT TEMPERATURE", 0.0);
```

matching FlyByWire's own `SimulationTestBed` defaults exactly (same
values, same units — inHg and Celsius, the same units this crate's own
`Vars`/`TestVars` convention already uses for these two names). This is
not a scripted/fake value: it is the same "aircraft is sitting in the open
air" baseline every one of FlyByWire's own tests already assumes, and the
one condition no real aircraft — cold and dark or with the APU running —
is ever actually without.

I could not run `cargo test` to confirm the NaN is gone (barred by the
debug brief), so this is delivered as the best-evidenced fix rather than a
verified one: `AMBIENT PRESSURE` reading as a literal vacuum for the whole
two-minute run, with FlyByWire's own equivalent harness never allowing
that, plus the concrete divide sites above, is strong circumstantial
evidence, but if a NaN remains after this, the next place to look is the
same `rho_ambient`-style pattern (a physical quantity divided by an
`UpdateContext` reading `TestVars` never seeds) elsewhere in
`a380_systems`/`fbw-common`, using the same technique: grep the ported
crate for `context.ambient_pressure()`/`context.ambient_temperature()`
consumers and check which of them divide by it.

## Files changed

- `src/start_state.rs` — seeded `AMBIENT PRESSURE`/`AMBIENT TEMPERATURE` in
  the offline systems test (bug C).
- `src/fadec.rs` — removed the per-tick `Vec` allocation in
  `update_thrust_limits` (bug B).
- `src/state_dump.rs` — added `StateDump::enabled()` for cheap pre-lock
  gating (bug B).
- `src/lib.rs` — cached `msfs_derived`'s three identifiers in `Computed`
  (bug B); gated the state-dump snapshot lock on `enabled()` (bug B);
  wired up `EngineCommands::spawn_at_idle()` for non-cold start states
  (bug A, the related bug).

## Files I could not edit, with exact patches above

- `src/app_settings.rs` — flip `state_dumps` default to `false` (bug B).
- `src/physics/engine/mod.rs` — generic EGT-probe ceiling, plus a test
  (bug A; a mitigation, not the underlying governor/combustor fix, which
  needs the full engine test suite run against it).
