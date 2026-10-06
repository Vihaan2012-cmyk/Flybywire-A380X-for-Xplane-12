# MSFS circuit protection: full physics port (design)

Date: 2026-09-29. Status: for review.

## Goal

The MSFS 2020 FlyByWire A380X (Development build `672384b9602a84d6832b8852cd6ec0abdd006f85`) runs the X-Plane port's model of the A380's circuit protection, with the same physics as X-Plane:

- **What runs:** the Electrical Load Management System's 399 units (375 solid-state power controllers and 24 thermal breakers), on the port's electrical network.
- **Tripping:** units trip on the same curves: ambient-derated I²t, magnetic, the SSPC arc-fault channel and the repeated-trip lockout.
- **Operating them:** the crew can open and reset every unit. That's from the EFB for all 399, the equivalent of the real aircraft's OIT circuit-breaker page, and from the overhead RESET panel for the systems it serves.
- **Consequences:** an open unit takes power away from its real FlyByWire consumer.

## Non-goals

- **Changing X-Plane behaviour.** The plugin keeps its behaviour, and its full test suite (2800 tests, with one known failure) must stay green.
- **Unifying the plugin's older breaker systems on X-Plane.** `circuits.rs` and `breakers.rs` stay as they are there. This is noted as a follow-up.
- **MSFS 2024 or FlyByWire mainline packaging.** The crate and patches carry over later.
- **A pulled or popped visual for the RESET buttons.** FlyByWire's template already animates push and latch, and that is all the model has.
- **MSFS's own `systems.cfg` circuits.** MSFS simulates those itself.

## What exists today (facts this design rests on)

| Piece | Where | Notes |
|---|---|---|
| 399-unit catalogue and trip physics | plugin `src/deep/breakers/{catalog,trip,registry,live}.rs` (≈3,100 lines) | Publishes `BKR_<id>_OPEN` and `BKR_<id>_STATUS`. IDs are lower-case with hyphens (`fms-1-normal-bkr`). |
| Electrical network (the one solve) | plugin `src/deep/electrical/{network,loads,sources,shedding,registry,live}.rs` (≈8,200 lines) | Opens contacts one frame after a unit publishes `OPEN`. Its `coupling_table()` turns machine verdicts into FlyByWire failure IDs. |
| Wiring (arc and bundle faults) | plugin `src/deep/wiring/*.rs` (≈2,200 lines) | Only per-frame input is SAT. |
| External dependencies of those three | `deep::api`, `deep::live::{Faults, PublishedFrame, DerivedFailure}`, `deep::apu::oil` (+ its `params` constants), `deep::integration::weather_truth::EnvironmentTruth`, `physics::electrical::{trip_step, trip_step_with_ambient, THERMAL_TRIP_K, nominal_bus_voltage, TripCause}`, `breakers::Bus`, `failures::{a380_failures, failure_name}` | None depend on X-Plane. |
| Per-frame inputs the three read | `Truth`: `dt_s, on_ground, environment.{sat_c,tas_ms}, ac_bus_powered, ac_bus_volts, dc_bus_powered, dc_bus_volts, apu_running, engine_running, engine_n, engine_oil_temp_c, gpu_plugged_in, controls.{apu_gen_pb_on, bat_pb_auto, eng_gen_pb_on}`, plus three values from other areas: `CABIN_CARGO_DOOR_CMD:1`, `CABIN_CARGO_DOOR_PERCENT:1`, `THERMAL_ZONE_MAINAVIONICS_TEMPERATURE_C` | These come from `deep/plugin.rs`'s `truth()`. |
| FlyByWire gate inputs | the port's FlyByWire Rust patches `patches/fbw-rust/{power-path-*,bus-feeder-breakers*,breaker-open-polarity,breakers,generator-overload-trip,tr-thermal-model,static-inverter-efficiency,battery-thermal-runaway,electrical}.patch` | FlyByWire consumers read `ELEC_<X>_BREAKER_{OPEN,CLOSED}`. Today the older `breakers.rs` writes them, not the 399 units. Against `672384b`: `breaker-open-polarity`, `breakers` and `power-path-pending-TR-GEN` fail to apply; the rest apply. |
| MSFS failure path | `systems_wasm/src/failures.rs`, `lib.rs::read_failures_into_simulation` | The EFB sends a JSON list of IDs, which becomes one set passed to `update_active_failures`. There is no second source. |
| Cockpit RESET panel | `A380_COCKPIT.xml` + `behaviour/overhead/reset.xml` template `FBW_Airbus_RESET_PANEL_BUTTON` | The glTF has 52 `CB_<NAME>` nodes plus 8 `CB_EMPTY*`. 10 are wired to `L:A32NX_RESET_PANEL_<NAME>` (AESU1/2, ARPT_NAV, FMC_A/B/C, FWS1/2, NSS_AVNCS, NSS_FLT_OPS). `FlightManagementComputer.ts` and `ResetPanelPublisher.ts` read them. |
| EFB Study tab | untracked in `D:/fbw-aircraft/fbw-common/src/systems/instruments/src/EFB/Study/` (Study.tsx, catalogue.ts, Pages/*: 768 lines) + `Efb.tsx`/`ToolBar.tsx` route and button | Static: it fetches `catalogue.json` and reads no live state. |

## Design

### 1. The `deep_electrical` crate

- **Canonical source:** the plugin repo, `crates/deep_electrical/`. It has no dependencies beyond `std` (and `serde` where the moved code already uses it): no X-Plane and no FlyByWire `systems`.
- **Contents:** the three areas moved whole, plus exactly the external dependencies in the table above (moved, not copied):
  - `electrical`, `breakers` and `wiring` (their `live.rs` becomes `system.rs`);
  - `api`; `Faults`, `PublishedFrame` and `DerivedFailure`;
  - `apu::oil` and its `params` constants;
  - the five `physics::electrical` trip items;
  - `Bus`;
  - a `failure_ids` table (id → name) replacing the `failures` lookups.

  The plugin re-exports each item from its old path, so the other 16 areas and all of `physics` compile unchanged.
- **Input:** each system takes a new `ElectricalInputs` struct instead of `&Truth`. Its fields are exactly the inputs listed above; the three cross-area values are `Option<f64>` (`None` → the area's existing default).
- **Public API:**
  ```rust
  pub struct ElectricalInputs { /* the fields above, typed as in Truth today */ }
  pub struct DeepElectrical { breakers: BreakersSystem, electrical: ElectricalSystem, wiring: WiringSystem, published: PublishedFrame }
  impl DeepElectrical {
      pub fn new() -> Self;
      pub fn register(&self, registry: &mut Registry);        // components + failures, as today
      pub fn published_names(&self) -> Vec<String>;           // every name tick() can publish
      pub fn tick(&mut self, inputs: &ElectricalInputs, faults: &Faults, publish: &mut dyn FnMut(&str, f64));
      pub fn derived_failures(&self) -> Vec<DerivedFailure>;  // FlyByWire failure ids + magnitude
      pub fn command(&mut self, id: &str, command: BreakerCommand) -> bool;  // Open, Close (reset)
      pub fn is_open(&self, id: &str) -> bool;
      pub fn current_a(&self, id: &str) -> f64;               // the unit's own current this frame
  }
  pub fn lvar_key(id: &str) -> String;  // "fms-1-normal-bkr" -> "FMS_1_NORMAL_BKR"
  pub const GATES: &[Gate];             // breaker id -> FlyByWire gate variable + polarity
  ```
- **Frame order:** `tick` steps breakers, then electrical, then wiring (the plugin's alphabetical order). Each reads the previous frame's `PublishedFrame`, so the one-frame lag is identical to X-Plane.
- **The gate table:** comes from `breakers.rs`'s `plugin_var` entries, keyed by the deep unit with the same ID. On MSFS, the host drives every gate from the 399-unit state, which unifies the two systems there.

### 2. The plugin, unchanged in behaviour

- **Thin wrappers:** the plugin's `deep::{breakers,electrical,wiring}` become wrappers. Each `Area` impl builds `ElectricalInputs` from `Truth` and forwards. `all_areas()` is unchanged.
- **Proof that nothing changed:** before any code moves, record a golden run. That's 600 frames of a scripted `Truth` sequence: cold start, then generators on, then a pulled unit, then an overcurrent. Keep every published name and value. After the move, the same run must reproduce it bit for bit, and the full plugin suite must pass.

### 3. The MSFS host: `a380_systems/src/electrical/circuit_protection.rs`

`CircuitProtection`, a `SimulationElement` owned by the A380's electrical system:

- **`new(context)`** registers identifiers for:
  - every input (table below);
  - every `published_names()` entry, keyed through `lvar_key` (the registry adds `A32NX_`);
  - per unit `BKR_<KEY>_CMD` (write 1 = open, 2 = close/reset; the host writes it back to 0 once consumed);
  - every `GATES` variable;
  - the 52 `RESET_PANEL_<NAME>` variables.
- **`read()`** reads inputs, commands and the RESET panel.
- **`update(context)`** runs from `A380::update` after the electrical system.
- **`write()`** writes the published values, gates (from `is_open`, with each gate's polarity) and cleared commands.
- **Per-unit current:** the host also writes `BKR_<KEY>_CURRENT_A` from `current_a`. The plugin publishes no per-unit current, and adding one to the crate's published set would change X-Plane's output, so only the host writes it.
- **Input mapping:** each `ElectricalInputs` field reads the same variable the plugin's `truth()` reads for it, where that is a FlyByWire-published variable (identical in MSFS). Where the plugin reads an X-Plane dataref, the host reads the MSFS simvar with the same meaning, registered in `a380_systems_wasm` (e.g. `SIM ON GROUND`, `AMBIENT TEMPERATURE`, `AIRSPEED TRUE`). The implementation plan lists every field → variable pair.
- **RESET panel:** while `RESET_PANEL_<NAME>` is 1 (latched), the units mapped to `<NAME>` are held open, and they close again when it returns to 0. This is the real panel's effect: a reset cycles the computer's power through its SSPC.
- **The mapping table** (`RESET_PANEL` name → unit ids) contains only correspondences with evidence:
  - the 7 `breakers.rs` `panel_node` ones (TR1, TR_2A, ESS_TR, LGCIS1/2, PACK1/2_CTL);
  - function matches to the catalogue (FMC_A/B/C → `fms-1/2/3-{normal,2nd}-bkr`; LGCIS → `lgciu-*`; CPCS/TCS/VCS 1/2 → `cpiom-b1/b2-*`; ATC → `xpdr-*`), each confirmed by reading the unit's `consumer` field in the catalogue.

  Names with no modelled unit still latch, as FlyByWire's own template does, but open nothing, and the table records them as unmapped.
- **`derived_failure_ids()`** maps `derived_failures()` to u64 IDs for the failure merge.

### 4. Failure merge: `systems` + `systems_wasm`

- **`systems::simulation::Aircraft` gains `fn derived_failure_ids(&self) -> Vec<u64> { Vec::new() }`.** The default leaves the A32NX unchanged.
- **`systems_wasm::Failures` keeps the crew set from the EFB.** Every tick, the host sorts the aircraft's derived IDs. When the crew set or the derived IDs change, it calls `update_active_failures(crew ∪ derived)`, mapping IDs through the same `identifier_to_failure_type`. The merge is a pure function with its own unit tests.
- **`A380::derived_failure_ids`** returns `circuit_protection.derived_failure_ids()`.

### 5. FlyByWire gate patches on `672384b`

- **Apply the electrical subset** listed in the table to the Development-build worktree (`D:/A380/fbw-xp-worktrees/fs2020-672384b`). Port `breaker-open-polarity`, `breakers` and `power-path-pending-TR-GEN` by hand.
- **Safe default:** every gate must read "open" polarity, so a variable nothing writes (0) means closed. An MSFS load without the host, or before its first write, then keeps every consumer powered. A test enforces this.

### 6. Operating it

- **EFB:** port the Study tab into the Development-build tree, adding the route, the toolbar button and the four pages.
  - **Live `BreakerPanels`:** per unit it shows open or tripped and the current, from `A32NX_BKR_<KEY>_OPEN`, `_STATUS` and `_CURRENT_A`. Open and close buttons write `A32NX_BKR_<KEY>_CMD`.
  - **Catalogue:** `catalogue.json` is regenerated from the current plugin (so it includes today's failure IDs). It ships beside `efb.html` in `html_ui/Pages/VCockpit/Instruments/A380X/EFB/`, which is where `CATALOGUE_URL = 'catalogue.json'` resolves.
- **Cockpit:** in `A380_COCKPIT.xml`, add a `FBW_Airbus_RESET_PANEL_BUTTON` use for each of the 42 unwired `CB_<NAME>` nodes (the template is unchanged). The 8 `CB_EMPTY*` nodes stay unwired.

### 7. Build and install

- **WASM:** built in FlyByWire's dev-env image with the `build-a380x:systems` recipe. Before each build, `scripts/sync-deep-electrical.sh` copies the crate into the worktree at `fbw-a380x/src/wasm/systems/deep_electrical/` and records a content hash. `a380_systems` depends on it by path, and a unit test compares the recorded hash, so a stale copy fails the build.
- **EFB:** `npm ci`, then `build-a380x:instruments` in the same image. Only the `EFB/` output folder and `catalogue.json` ship.
- **Install script:** `install-systems-wasm.ps1` grows into `install-msfs-circuit-protection.ps1`. It installs `systems.wasm`, the EFB folder, `catalogue.json` and `A380_COCKPIT.xml`, updates every file's `layout.json` entry, keeps originals and checks the exact commit. `-Restore` puts everything back.

## Testing

- **Crate:** every test moved with the code, plus the golden run.
- **Plugin:** the full suite, at 2800 with one known failure.
- **FlyByWire:** the `systems` and `a380_systems` suites on `672384b`, plus these native integration tests of `CircuitProtection` in an A380 test bed:
  1. Opening `tr-1` via `BKR_TR_1_CMD` stops FlyByWire's TR 1 converting.
  2. An overcurrent on a unit trips it and publishes `STATUS`.
  3. Latching `RESET_PANEL_FMC_A` opens the FMC A units and releasing restores them.
  4. With the host absent, or before its first write, every gate reads closed.
  5. A derived failure and an EFB-armed failure are both active, and removing one leaves the other.
- **Performance:**
  - a native benchmark of `DeepElectrical::tick` (budget: 2 ms per frame, the X-Plane figure);
  - in MSFS, the host publishes its own tick time as `A32NX_CIRCUIT_PROTECTION_TICK_US`, so you can read it in the sim.
- **Manual (you, in MSFS):** a checklist in the guide: open a unit from the EFB and see its consumer drop; trip-free normal flight for 30 minutes; the RESET panel resets an FMC.

## Risks

- **WASM cost of the network solve:** it's about 2 ms per frame natively in X-Plane. The tick-time variable exposes it, and if it's too slow the fallback is to step the network every other frame.
- **MSFS simvar availability for inputs:** each must be registered in `a380_systems_wasm`. The plan verifies every one.
- **The RESET panel XML can only be tested by you in MSFS.**
- **Development channel updates** replace the package. The install script refuses a mismatched commit, and you rebuild from the new commit.

## Phases

1. **Golden run** recorded on the plugin as it is now.
2. **Crate extraction.** The plugin stays green and the golden run matches.
3. **Gate patches** onto `672384b` (FlyByWire suites green).
4. **Host, failure merge, integration tests and benchmark.**
5. **EFB Study tab with live breakers, and RESET panel XML.** Run as parallel agents: Sonnet, at most 6, not nested.
6. **Build, extended install script, guide.**
7. **Your MSFS test.**
