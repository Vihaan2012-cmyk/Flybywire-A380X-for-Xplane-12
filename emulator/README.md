# fbw_a380_emulator

A deterministic, X-Plane-free test bench for the FlyByWire A380X X-Plane
port (`D:\A380\fbw-xp-systems`). `Emulator` owns FlyByWire's real
`Simulation<A380>`, the plugin's real `Vars`, breaker/circuit catalogues,
failure catalogue, and the engine-coupling physics modules, and ticks them
in the plugin's own `Plugin::tick` order -- no X-Plane process, no
reimplemented behaviour.

## Build and test

```
cd /d/A380/fbw-xp-systems/emulator
CARGO_TARGET_DIR=/d/A380/fbw-build/target-main cargo +stable-x86_64-pc-windows-gnu test --release
```

## Using it

```rust
use fbw_a380_emulator::{presets, Emulator};
use systems::simulation::StartState;

let mut e = Emulator::new(StartState::Apron);
e.set_oat_c(15.0);
e.set_total_weight_lb(650_000.0);
e.pull_breaker("some-breaker-id");   // takes effect on the next tick()
e.set_failure_magnitude(1234, 0.5);  // continuous 0..=1
e.tick(0.05);
let violations = e.invariant_report();
let value = e.get_var("A32NX_ELEC_AC_1_BUS_IS_POWERED");
```

Or start from a preset built from real inputs, not internal state:
`presets::cold_and_dark()`, `presets::powered()`,
`presets::engines_running()`, `presets::cruise(alt_ft, mach)`.

Everything is discoverable, generated from the plugin's own catalogues (not
hand-copied):

- `Emulator::list_breakers()` -> `breakers::catalog()`
- `Emulator::list_failures()` -> `Failures::ids()`
- `Emulator::list_controls()` -> parsed from `cockpit_bindings.txt`
- `Emulator::snapshot_all()` -> every registered variable and its value

The raw escape hatch (`set_var`/`get_var` for FlyByWire's own named/
simulator variables, `set_dataref`/`get_dataref` for a cockpit control's
exact `fbw/cockpit/...` name) covers anything a typed method does not.

## Writing an emergence test

The pattern (see `tests/emergence.rs` for a worked example): make an
**independent prediction** from a different source than the code under
test (a physical formula, a documented constant, a hand calculation -- not
something read back out of the same model), assert the measured effect
matches it within a generous tolerance, then **decouple**: repeat the same
input change but skip the one contract step that should carry the effect
(e.g. never tick, or tick a different subsystem only), and assert the
effect now vanishes. That proves the measured effect came from the
specific contract you're testing, not from simulation noise or an
unrelated path.

```rust
// 1. Change one independent input.
e.set_var("ELEC_ENG_GEN_1_SHAFT_POWER_DEMAND", raised_w);
// 2. Run only the contract under test (or a full tick, if that's the unit).
e.update_electrical_loads();
// 3. Read the effect and compare to an independent prediction.
let gearbox_w = e.get_var("ENGINE_GEARBOX_ELEC_LOAD_W:1");
assert_eq!(gearbox_w, raised_w);
// 4. Decouple: on a fresh Emulator, change the same input but never run
//    the contract step. The effect must be absent.
```

## What is covered

- FlyByWire's full `Simulation<A380>` tick (electrical, hydraulics,
  pneumatics, ADIRS state machine, flight controls, APU, fire protection,
  payload) -- unmodified, real code.
- Every catalogued breaker (`breakers::catalog()`): pull/reset/read.
- Every registered failure id, with continuous `0.0..=1.0` magnitude.
- `systems.cfg`'s circuits (`circuits.rs`).
- The three "hyperrealism" engine-coupling physics modules: `physics::
  electrical::EngineLoads`/`CircuitProtection`, `physics::air::
  EngineBleedLoads`, `physics::hydraulics::Hydraulics`.
- The gas-turbine engine model (`physics::engine::Engine`), drivable
  directly (`Emulator::step_engine`).
- Wear (`wear.rs`), invariants (`invariants.rs`'s clamp-violation log),
  random failure triggers (`random_failures.rs`).
- Environment/flight-state/weight inputs, written directly into the same
  named `Vars` slots the live plugin's X-Plane dataref mapping would feed
  (see `Emulator::set_var`'s doc comment for exactly what that means
  offline).
- Every `fbw/cockpit/*` control dataref named in `cockpit_bindings.txt`
  (`Emulator::list_controls`/`set_dataref`/`get_dataref`).

## What is not covered, and why

See the crate doc comment at the top of `src/lib.rs` for the full list and
reasoning; in short:

- `fuel.rs`, the radios/sensors/PRIM/FADEC/`engine_commands.rs` modules,
  `physics::adirs`'s sensor glue, doors, efb, sound, mapdata: still
  `Xplm`-bound in the plugin (they call the live XPLM API directly).
  Opening these up would need a *recording* `Xplm` test double (an
  in-memory dataref store) added to the plugin crate; this pass only
  widened visibility, it did not build that double.
- The JS/QuickJS cockpit instruments (feature `js`): out of scope for a
  headless physics/systems bench; not linked in.
- MEL's own per-tick deferral processing (needs a running
  `airframe_hours` total this bench does not track); the catalogue itself
  is still reachable through `test_support::mel`.
- Passenger-count -> gross-weight/CG is only partially wired: FlyByWire's
  own per-station `PAYLOAD STATION WEIGHT:n` is settable
  (`Emulator::set_payload_station_lb`), but this pass did not derive the
  FCOM standard per-station passenger/cargo split table, and `TOTAL
  WEIGHT` is a separate X-Plane-computed input in the live plugin (its own
  weight-and-balance solver) that this bench does not reproduce -- set it
  directly with `Emulator::set_total_weight_lb`.

## Known open finding

`tests/emergence.rs`'s `cold_and_dark_builds_and_ticks_with_no_nan` and
`generator_load_raises_engine_fuel_flow_through_the_gearbox_contract`
currently **fail**:

- `A32NX_ELEC_TR_1_CURRENT` goes `NaN` by the first tick of a cold-and-dark
  start once `physics::electrical`/`physics::hydraulics`/`breakers` are
  ticked alongside FlyByWire's own systems tick -- a combination
  `start_state.rs`'s existing cold-and-dark test does not exercise (it
  runs `Simulation`/`aspects` only). Not root-caused or fixed in this pass.
- The generator-load -> fuel-flow prediction is outside its 35% tolerance
  (measured ~37% of the independent LHV-based prediction). This is **not**
  new: `fbw_a380_systems_xp`'s own `src/offline_harness.rs` test
  (`raising_generator_1_electrical_load_raises_engine_1_fuel_flow_through_the_gearbox_contract`)
  fails with the identical numbers when run directly
  (`cargo test --lib offline_harness`), so this reflects the state of that
  harness (under concurrent development per this task's brief), not a bug
  introduced by this crate.

Both are left as real, reproducible, honestly-failing tests rather than
softened to pass.
