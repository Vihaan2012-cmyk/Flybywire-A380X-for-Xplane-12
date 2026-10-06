# MSFS Circuit Protection (Full Physics Port) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** The MSFS 2020 FlyByWire A380X (Development `672384b`) runs the X-Plane port's 399-unit circuit protection on its electrical network, with the same trip physics. The crew can operate every unit from the EFB and the overhead RESET panel, and an open unit takes power away from its real FlyByWire consumer.

**Architecture:**
- Move the plugin's `deep::{electrical,breakers,wiring}` and their small dependency closure into a pure-Rust crate, `deep_electrical`. The plugin keeps thin adapters, and a golden run proves X-Plane output is unchanged.
- In the Development-build FlyByWire worktree:
  - vendor that crate;
  - add a `CircuitProtection` element to `a380_systems`;
  - merge its derived failures with the crew's in `systems_wasm`;
  - port the gate patches;
  - add the EFB Study tab with a live breaker page;
  - wire the 42 unwired RESET buttons.
- Build in FlyByWire's dev-env container, then install with one script that can also restore everything.

**Tech Stack:** Rust (the plugin crate, FlyByWire `systems`/`a380_systems`/`systems_wasm`, target `wasm32-wasip1`), TypeScript/React (FlyByWire EFB), MSFS model behaviour XML, PowerShell, and Docker (`ghcr.io/flybywiresim/dev-env@sha256:28b1f55c047b9ec338c3d676a82225fe135b0b1061fa7993c03b9a75b5e470cd`).

**Spec:** `E:/fbw-int/plugin/docs/superpowers/specs/2026-09-29-msfs-breaker-physics-design.md`

## Global Constraints

- **No git commits.** Every "commit" step is a diff save: `git diff > E:/fbw-debug/msfs-cb/<task>.diff`.
- **Agents:** at most 6 running at once, no agent starts another, and Sonnet first.
- **Build directory:** all native Rust builds use `CARGO_TARGET_DIR=D:/A380/fbw-build/target` and toolchain `+stable-x86_64-pc-windows-gnu`. The Docker build uses `D:/A380/fbw-build/target/wasm-docker`.
- **Never launch** X-Plane or MSFS.
- **Don't fake values.** Every mapping (input variable, RESET name → unit) must cite the file and line it came from, and an unmapped item stays unmapped.
- **X-Plane output must not change.** The golden run (Task 1) must match bit for bit, and the plugin suite must stay at 2800 tests with only the known failure `prim::tests::zero_pedal_never_lets_the_sec_command_a_sustained_rudder_trim_over_300_simulated_seconds`. The timing-only tests `remote::tests::the_systems_in_their_own_process_switch_on_the_same_frame_as_in_the_plugin` and `physics::engine::tests::the_per_frame_cost_is_small` may fail under load; if they do, rerun them alone.
- **Paths:**
  - Plugin: `E:/fbw-int/plugin` (builds FlyByWire from `E:/fbw-int/fbw-aircraft`).
  - Development-build FlyByWire worktree: `D:/A380/fbw-xp-worktrees/fs2020-672384b`.
  - MSFS package: `D:/Microsoft Flight Simulator 2020/Microsoft Flight Simulator 2020 Packages/Community/flybywire-aircraft-a380-842`.
- **Commit pinning:** the Development-build commit is pinned. The installer refuses any package whose `manifest.json` `package_version` doesn't contain `672384b9602a84d6832b8852cd6ec0abdd006f85`.

---

## File Structure

**Plugin (`E:/fbw-int/plugin`)**

| File | Responsibility |
|---|---|
| `crates/deep_electrical/Cargo.toml` | New crate. `std` only (+ `serde` if the moved code uses it). |
| `crates/deep_electrical/src/lib.rs` | Module list, `ElectricalInputs`, `ElectricalControls`, `DeepElectrical` facade, `lvar_key`, `BreakerCommand`, `GATES`. |
| `crates/deep_electrical/src/{api.rs, frame.rs}` | Moved `deep::api`. `frame.rs` holds `Faults`, `PublishedFrame` and `DerivedFailure`, moved out of `deep/live.rs`. |
| `crates/deep_electrical/src/{electrical,breakers,wiring}/` | Moved areas. Each `live.rs` becomes `system.rs`, and `tick` takes `&ElectricalInputs`. |
| `crates/deep_electrical/src/oil.rs` | Moved `deep/apu/oil.rs`, plus the `params` constants it uses. |
| `crates/deep_electrical/src/trip_curve.rs` | Moved `physics::electrical::{trip_step, trip_step_with_ambient, THERMAL_TRIP_K, nominal_bus_voltage, TripCause}`. |
| `crates/deep_electrical/src/bus.rs` | Moved `breakers::Bus`. |
| `crates/deep_electrical/src/failure_ids.rs` | id → name table for the failure IDs the areas reference, replacing `crate::failures::{a380_failures, failure_name}` lookups. |
| `crates/deep_electrical/tests/golden.rs` + `tests/golden/deep_electrical.json` | Golden run (moved here in Task 3). |
| `src/deep/{electrical,breakers,wiring}/mod.rs` | Adapters: `impl Area`, building `ElectricalInputs` from `Truth`. |
| `src/deep/electrical_inputs.rs` | `fn electrical_inputs(truth: &Truth) -> deep_electrical::ElectricalInputs`. |
| `scripts/sync-deep-electrical.sh` | Copies the crate into the FlyByWire worktree and writes `SOURCE_HASH`. |

**FlyByWire worktree (`D:/A380/fbw-xp-worktrees/fs2020-672384b`)**

| File | Responsibility |
|---|---|
| `fbw-a380x/src/wasm/systems/deep_electrical/` | Vendored crate plus `SOURCE_HASH`. |
| `fbw-a380x/src/wasm/systems/a380_systems/src/electrical/circuit_protection.rs` | `CircuitProtection` host. |
| `fbw-a380x/src/wasm/systems/a380_systems/src/electrical/reset_panel_map.rs` | RESET name → unit ids, with evidence. |
| `fbw-a380x/src/wasm/systems/a380_systems/src/lib.rs` | Own, update and accept `CircuitProtection`; `derived_failure_ids`. |
| `fbw-common/src/wasm/systems/systems/src/simulation/mod.rs` | `Aircraft::derived_failure_ids` (default empty). |
| `fbw-common/src/wasm/systems/systems_wasm/src/{failures.rs,lib.rs}` | Crew ∪ derived merge. |
| `fbw-a380x/src/wasm/systems/a380_systems_wasm/src/lib.rs` | Register `GENERAL ENG OIL TEMPERATURE:1..4`. |
| `fbw-common/src/systems/instruments/src/EFB/Study/**`, `Efb.tsx`, `ToolBar/ToolBar.tsx` | EFB Study tab and live `BreakerPanels`. |
| gate patch targets | The files the electrical patch subset touches. |

**Delivery (`D:/A380/fbw-build/wasm-fs2020/`)**

| File | Responsibility |
|---|---|
| `out/systems.wasm`, `out/EFB/`, `out/A380_COCKPIT.xml`, `out/catalogue.json` | Build outputs. |
| `install-msfs-circuit-protection.ps1` | Installs and restores everything. It replaces `install-systems-wasm.ps1`. |
| `INTEGRATION.md` | Guide, updated. |

---

### Task 0: Backups and baseline

**Files:** none modified.

- [ ] **Step 1: Back up the trees this plan touches**

```bash
B=/e/fbw-backup/2026-09-29-msfs-cb; mkdir -p $B
cd /e/fbw-int/plugin && git diff > $B/plugin-before.diff && git status --short > $B/plugin-before.status
cd /d/A380/fbw-xp-worktrees/fs2020-672384b && git diff > $B/fs2020-before.diff
P="/d/Microsoft Flight Simulator 2020/Microsoft Flight Simulator 2020 Packages/Community/flybywire-aircraft-a380-842"
mkdir -p $B/msfs && cp "$P/layout.json" "$P/SimObjects/AirPlanes/FlyByWire_A380_842/model/A380_COCKPIT.xml" $B/msfs/
```

- [ ] **Step 2: Record the plugin suite baseline**

Run: `cd /e/fbw-int/plugin && CARGO_TARGET_DIR=/d/A380/fbw-build/target cargo +stable-x86_64-pc-windows-gnu test --lib -q 2>&1 | grep -E "test result|panicked at" > /e/fbw-backup/2026-09-29-msfs-cb/plugin-baseline.txt`
Expected: `2799 passed; 1 failed` (the known `prim` test), give or take the load-sensitive timing tests.

### Task 1: Golden run (before anything moves)

**Files:**
- Create: `E:/fbw-int/plugin/src/deep/golden_electrical.rs`
- Modify: `E:/fbw-int/plugin/src/deep/mod.rs` (add `#[cfg(test)] mod golden_electrical;`)
- Create (by the test): `E:/fbw-int/plugin/crates/deep_electrical/tests/golden/deep_electrical.json`

**Interfaces:**
- Consumes: `deep::breakers::live::BreakersLive::new`, `deep::electrical::live::ElectricalLive::new`, `deep::wiring::live::WiringLive::new`, `Area::{tick, publish, derived_failures}`, `BreakersLive::breaker_mut(&str) -> Option<&mut Breaker>`, `Breaker::pull`, `Truth::default`, `Faults::from_pairs`.
- Produces: the golden file, and the scenario function `golden_truth(frame: usize) -> Truth`, reused in Task 3.

- [ ] **Step 1: Write the recorder and comparer**

```rust
//! Golden run of the three electrical areas: a scripted 600-frame sequence
//! whose every published value and derived failure is recorded once, before
//! the areas move into the `deep_electrical` crate, and compared bit for bit
//! after. `GOLDEN_RECORD=1` records; otherwise it compares.
use std::collections::BTreeMap;

use super::breakers::live::BreakersLive;
use super::electrical::live::ElectricalLive;
use super::live::{Area, Faults, PublishedFrame, Truth};
use super::wiring::live::WiringLive;

pub const FRAMES: usize = 600;
pub const GOLDEN: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/crates/deep_electrical/tests/golden/deep_electrical.json");

/// Cold aircraft, then GPU, then engines and generators, then a crew pull,
/// then an overcurrent.
pub fn golden_truth(frame: usize) -> Truth {
    let mut t = Truth::default();
    t.dt_s = 0.05;
    t.on_ground = true;
    t.environment.sat_c = 15.0;
    let gpu = frame >= 100;
    let engines = frame >= 200;
    t.gpu_plugged_in = gpu;
    t.controls.bat_pb_auto = [true, true];
    for i in 0..4 {
        t.engine_running[i] = engines;
        t.engine_n2_frac[i] = if engines { 0.65 } else { 0.0 };
        t.engine_oil_temp_c[i] = if engines { 80.0 } else { 15.0 };
        t.controls.eng_gen_pb_on[i] = engines;
        t.ac_bus_powered[i] = gpu || engines;
        t.ac_bus_volts[i] = if gpu || engines { 115.0 } else { 0.0 };
    }
    for i in 0..2 {
        t.dc_bus_powered[i] = true;
        t.dc_bus_volts[i] = 28.0;
    }
    t
}

fn run() -> Vec<(BTreeMap<String, f64>, Vec<(u64, f64)>)> {
    let mut breakers = BreakersLive::new();
    let mut electrical = ElectricalLive::new();
    let mut wiring = WiringLive::new();
    let mut last = PublishedFrame::default();
    let mut frames = Vec::with_capacity(FRAMES);
    for frame in 0..FRAMES {
        if frame == 400 {
            breakers.breaker_mut("fms-1-normal-bkr").expect("catalogue id").pull();
        }
        let mut truth = golden_truth(frame);
        truth.published = std::mem::take(&mut last);
        // An overcurrent from frame 450: the arc-fault channel's own failure
        // id is found through the registry at test time, not hard-coded.
        let faults = if frame >= 450 { overcurrent_fault() } else { Faults::default() };
        breakers.tick(&truth, &faults);
        electrical.tick(&truth, &faults);
        wiring.tick(&truth, &faults);
        let mut published = BTreeMap::new();
        for area in [&breakers as &dyn Area, &electrical, &wiring] {
            area.publish(&mut |k, v| {
                published.insert(k.to_owned(), v);
            });
        }
        let mut derived = Vec::new();
        for area in [&breakers as &dyn Area, &electrical, &wiring] {
            area.derived_failures(&mut |d| derived.push((d.failure_id, d.magnitude)));
        }
        last = PublishedFrame(published.clone());
        frames.push((published, derived));
    }
    frames
}
```

`overcurrent_fault()` looks up, through `super::api::Registry` built from `BreakersLive::new()`'s own registry function, the first registered failure whose component is `fms-2-normal-bkr` and whose name contains `drift` (a trip point below rating). It returns `Faults::from_pairs([(id, 1.0)])`. If no such failure exists, it panics with the list of that unit's failure names, so the executor picks a real one instead of guessing.

Then the test:

```rust
#[test]
fn deep_electrical_golden_run() {
    let frames = run();
    let text = serde_json::to_string(&frames).unwrap();
    if std::env::var("GOLDEN_RECORD").is_ok() {
        std::fs::create_dir_all(std::path::Path::new(GOLDEN).parent().unwrap()).unwrap();
        std::fs::write(GOLDEN, &text).unwrap();
        return;
    }
    let expected = std::fs::read_to_string(GOLDEN).expect("record the golden run first: GOLDEN_RECORD=1");
    assert!(text == expected, "deep electrical output changed; diff {GOLDEN} against a fresh GOLDEN_RECORD run");
}
```

Adjust field names only if the compiler rejects one. Use the exact names from `live.rs`'s `Truth` and `Controls`, and `DerivedFailure`'s actual fields.

- [ ] **Step 2: Record it**

Run: `cd /e/fbw-int/plugin && GOLDEN_RECORD=1 CARGO_TARGET_DIR=/d/A380/fbw-build/target cargo +stable-x86_64-pc-windows-gnu test --lib -q -- deep_electrical_golden_run`
Expected: PASS, and the file exists with 600 frames.

- [ ] **Step 3: Compare mode is stable**

Run the same command without `GOLDEN_RECORD` twice.
Expected: PASS both times. If the second run fails, the areas carry nondeterminism (for example hash-map order in the output). Stop and fix the test so it sorts, never the physics.

- [ ] **Step 4: Save the diff** to `E:/fbw-debug/msfs-cb/task1.diff`.

### Task 2: Crate skeleton and dependency closure

**Files:**
- Create: `crates/deep_electrical/Cargo.toml`, `src/lib.rs`, `src/api.rs`, `src/frame.rs`, `src/oil.rs`, `src/trip_curve.rs`, `src/bus.rs`, `src/failure_ids.rs`
- Modify: plugin `Cargo.toml` (`deep_electrical = { path = "crates/deep_electrical" }`); `src/deep/api.rs` → `pub use deep_electrical::api::*;`; in `src/deep/live.rs`, move `Faults`, `PublishedFrame` and `DerivedFailure` out and replace them with `pub use deep_electrical::frame::{Faults, PublishedFrame, DerivedFailure};`; `src/deep/apu/oil.rs` → `pub use deep_electrical::oil::*;`; `src/physics/electrical.rs` (the five items → `pub use deep_electrical::trip_curve::{...};`); `src/breakers.rs` (`Bus` → `pub use deep_electrical::bus::Bus;`)

**Interfaces:**
- Produces: `deep_electrical::{api, frame, oil, trip_curve, bus, failure_ids}` with the same public items and signatures as today.

- [ ] **Step 1: Create the crate**

```toml
[package]
name = "deep_electrical"
version = "0.1.0"
edition = "2021"

[dependencies]
serde = { version = "1", features = ["derive"], optional = false }

[dev-dependencies]
serde_json = "1"
```

(Drop `serde` if nothing moved uses it.)

- [ ] **Step 2: Move each file's content verbatim.** Leave the old path as a `pub use` re-export, so every other plugin module keeps compiling. For `oil.rs`, copy only the `params` constants `oil.rs` references, as `pub(crate) const`s in `oil.rs`, and keep them in `deep/apu/params.rs` too (same values; add a test in the plugin asserting each pair is equal).

- [ ] **Step 3: Build the `failure_ids` table.** It holds exactly the `(u64, &'static str)` pairs the three areas look up through `crate::failures::{a380_failures, failure_name}`, generated by reading those call sites. A plugin test asserts every entry equals `crate::failures::failure_name(id)`.

- [ ] **Step 4: Verify.** Run `cargo test -p deep_electrical` (from the plugin directory), then the full plugin suite, then the golden run in compare mode.
Expected: all green, and the golden test matches.

- [ ] **Step 5: Save the diff** to `task2.diff`.

### Task 3: Move electrical, breakers and wiring behind `ElectricalInputs`

**Files:**
- Move: `src/deep/{electrical,breakers,wiring}/*.rs` → `crates/deep_electrical/src/{electrical,breakers,wiring}/` (`live.rs` → `system.rs`)
- Create: `src/deep/electrical_inputs.rs`; new `src/deep/{electrical,breakers,wiring}/mod.rs` adapters
- Move: `src/deep/golden_electrical.rs` → `crates/deep_electrical/tests/golden.rs` (now driving the crate systems directly with `ElectricalInputs`)

**Interfaces:**
- Produces, in `deep_electrical`:

```rust
#[derive(Clone, Debug, Default)]
pub struct ElectricalControls {
    pub eng_gen_pb_on: [bool; 4],
    pub apu_gen_pb_on: [bool; 2],
    pub bat_pb_auto: [bool; 2],
    pub starter_engaged: [bool; 4],
    pub apu_start_pb_on: bool,
    pub fire_pb_released: [bool; 4],
    pub fire_agent_pb_pressed: [[bool; 2]; 4],
    pub fire_pb_apu_released: bool,
    pub fire_agent_pb_apu_pressed: bool,
    pub gear_door_commanded_open: [f64; 3],
}

#[derive(Clone, Debug, Default)]
pub struct ElectricalInputs {
    pub dt_s: f64,
    pub on_ground: bool,
    pub sat_c: f64,
    pub tas_ms: f64,
    pub ac_bus_powered: [bool; 4],
    pub ac_bus_volts: [f64; 4],
    pub dc_bus_powered: [bool; 2],
    pub dc_bus_volts: [f64; 2],
    pub apu_running: bool,
    pub engine_running: [bool; 4],
    pub engine_n2_frac: [f64; 4],
    pub engine_oil_temp_c: [f64; 4],
    pub gpu_plugged_in: bool,
    pub controls: ElectricalControls,
    /// `CABIN_CARGO_DOOR_CMD:1`, `CABIN_CARGO_DOOR_PERCENT:1`,
    /// `THERMAL_ZONE_MAINAVIONICS_TEMPERATURE_C` from other areas; `None`
    /// takes the same default `published.get_or` uses today.
    pub cargo_door_cmd: Option<f64>,
    pub cargo_door_percent: Option<f64>,
    pub main_avionics_bay_temp_c: Option<f64>,
    /// The previous frame's outputs of these three areas (their own
    /// cross-reads, e.g. wiring reading `BKR_*`).
    pub published: frame::PublishedFrame,
}
```

Use the exact element types of today's `Truth`/`Controls` fields (for example `fire_agent_pb_pressed` is whatever `[Option<..>]`/array type `Controls` declares). Copy them from `deep/live.rs`, and don't change a type.

- [ ] **Step 1: Move the files.** In the moved `system.rs` files, replace `truth: &Truth` with `inputs: &ElectricalInputs`, and each `truth.X` with its `inputs` field. `truth.environment.sat_c` becomes `inputs.sat_c` and `truth.environment.tas_ms` becomes `inputs.tas_ms`. `truth.published.get_or("CABIN_CARGO_DOOR_CMD:1", d)` becomes `inputs.cargo_door_cmd.unwrap_or(d)`, and likewise the other two. Every other `truth.published` read becomes `inputs.published`. Remove the `impl Area` blocks from the moved files; the systems expose inherent `tick(&mut self, &ElectricalInputs, &Faults)`, `publish(&self, &mut dyn FnMut(&str, f64))`, `derived_failures(&self, &mut dyn FnMut(DerivedFailure))` and their registry functions. The compiler lists every remaining `truth.` use, and each must map onto an existing `ElectricalInputs` field. If a field isn't there, add it with the `Truth` field's exact type and note it in the diff description.

- [ ] **Step 2: Write the adapter**

```rust
// src/deep/electrical_inputs.rs
use super::live::Truth;

pub fn electrical_inputs(truth: &Truth) -> deep_electrical::ElectricalInputs {
    let c = &truth.controls;
    deep_electrical::ElectricalInputs {
        dt_s: truth.dt_s,
        on_ground: truth.on_ground,
        sat_c: truth.environment.sat_c,
        tas_ms: truth.environment.tas_ms,
        ac_bus_powered: truth.ac_bus_powered,
        ac_bus_volts: truth.ac_bus_volts,
        dc_bus_powered: truth.dc_bus_powered,
        dc_bus_volts: truth.dc_bus_volts,
        apu_running: truth.apu_running,
        engine_running: truth.engine_running,
        engine_n2_frac: truth.engine_n2_frac,
        engine_oil_temp_c: truth.engine_oil_temp_c,
        gpu_plugged_in: truth.gpu_plugged_in,
        controls: deep_electrical::ElectricalControls {
            eng_gen_pb_on: c.eng_gen_pb_on,
            apu_gen_pb_on: c.apu_gen_pb_on,
            bat_pb_auto: c.bat_pb_auto,
            starter_engaged: c.starter_engaged,
            apu_start_pb_on: c.apu_start_pb_on,
            fire_pb_released: c.fire_pb_released,
            fire_agent_pb_pressed: c.fire_agent_pb_pressed,
            fire_pb_apu_released: c.fire_pb_apu_released,
            fire_agent_pb_apu_pressed: c.fire_agent_pb_apu_pressed,
            gear_door_commanded_open: c.gear_door_commanded_open,
        },
        cargo_door_cmd: truth.published.get("CABIN_CARGO_DOOR_CMD:1"),
        cargo_door_percent: truth.published.get("CABIN_CARGO_DOOR_PERCENT:1"),
        main_avionics_bay_temp_c: truth.published.get("THERMAL_ZONE_MAINAVIONICS_TEMPERATURE_C"),
        published: truth.published.clone(),
    }
}
```

(Use `PublishedFrame`'s actual lookup method, `get` returning `Option<f64>`. If it only has `get_or`, add `get` to `frame.rs`.)

The adapter `mod.rs` for each area follows this pattern (shown for breakers; electrical and wiring are identical in shape):

```rust
use super::electrical_inputs::electrical_inputs;
use super::live::{Area, DerivedFailure, Faults, Truth};
pub use deep_electrical::breakers::*;

pub mod live {
    pub use deep_electrical::breakers::system::BreakersSystem as BreakersLive;
    pub fn live_system() -> Box<dyn super::super::live::Area> {
        Box::new(super::BreakersArea(BreakersLive::new()))
    }
}

pub struct BreakersArea(pub live::BreakersLive);
impl Area for BreakersArea {
    fn name(&self) -> &'static str { "breakers" }
    fn tick(&mut self, truth: &Truth, faults: &Faults) { self.0.tick(&electrical_inputs(truth), faults) }
    fn publish(&self, out: &mut dyn FnMut(&str, f64)) { self.0.publish(out) }
    fn derived_failures(&self, out: &mut dyn FnMut(DerivedFailure)) { self.0.derived_failures(out) }
}
```

Keep each area's `name()` string exactly as it is today.

- [ ] **Step 3: Verify.** Run `cargo test -p deep_electrical` (all moved tests plus the golden run through the crate API), the full plugin suite, and a grep showing `src/deep/{electrical,breakers,wiring}` hold only the adapters.
Expected: all green, and the golden run matches.

- [ ] **Step 4: Save the diff** to `task3.diff`.

### Task 4: The `DeepElectrical` facade

**Files:**
- Modify: `crates/deep_electrical/src/lib.rs`
- Modify: `crates/deep_electrical/src/breakers/trip.rs` (add `pub fn current_a(&self) -> f64 { self.prev_current_a }`)
- Test: `crates/deep_electrical/tests/facade.rs`

**Interfaces:**
- Produces:

```rust
pub enum BreakerCommand { Open, Close }

pub struct Gate { pub breaker_id: &'static str, pub variable: &'static str, pub open_when_true: bool }
pub const GATES: &[Gate] = &[ /* from breakers.rs plugin_var entries: id -> gate variable; open_when_true = the variable name ends in _OPEN */ ];

pub fn lvar_key(id: &str) -> String {
    id.chars().map(|c| if c.is_ascii_alphanumeric() { c.to_ascii_uppercase() } else { '_' }).collect()
}

pub struct DeepElectrical { breakers: BreakersSystem, electrical: ElectricalSystem, wiring: WiringSystem, last: PublishedFrame, derived: Vec<DerivedFailure> }
impl DeepElectrical {
    pub fn new() -> Self;
    pub fn published_names(&mut self) -> Vec<String>;   // one throwaway tick with default inputs, collect names, then reset to new()
    pub fn tick(&mut self, inputs: &ElectricalInputs, faults: &Faults, out: &mut dyn FnMut(&str, f64));
    pub fn derived_failures(&self) -> &[DerivedFailure];
    pub fn command(&mut self, id: &str, command: BreakerCommand) -> bool;
    pub fn is_open(&self, id: &str) -> bool;
    pub fn current_a(&self, id: &str) -> f64;
    pub fn breaker_ids(&self) -> Vec<&'static str>;
}
```

- **`tick`:** hands `last` back as `inputs.published` (the caller's value is ignored), ticks breakers, then electrical, then wiring, collects derived failures (magnitude > 0), publishes all three, and stores `last`. This is the same algorithm as `deep::live::Deep::tick`.
- **`command`:** `Open` → SSPC `remote_open()`, else `pull()`; `Close` → SSPC `remote_reset()`, else `reset()`. It returns whether the unit accepted the command.

- [ ] **Step 1: Write the failing tests** (`tests/facade.rs`):

```rust
use deep_electrical::*;

#[test]
fn lvar_keys_are_msfs_safe() {
    assert_eq!(lvar_key("fms-1-normal-bkr"), "FMS_1_NORMAL_BKR");
    assert_eq!(lvar_key("fire-loop-eng1-A"), "FIRE_LOOP_ENG1_A");
}

#[test]
fn facade_matches_the_golden_run() {
    // Drives DeepElectrical with the golden scenario's inputs (converted
    // with the same rules as the plugin adapter) and compares published
    // names and values with tests/golden/deep_electrical.json.
}

#[test]
fn opening_a_unit_opens_it_and_closing_restores_it() {
    let mut d = DeepElectrical::new();
    let id = "fms-1-normal-bkr";
    assert!(!d.is_open(id));
    assert!(d.command(id, BreakerCommand::Open));
    let mut sink = |_: &str, _: f64| {};
    d.tick(&ElectricalInputs { dt_s: 0.05, ..Default::default() }, &Faults::default(), &mut sink);
    assert!(d.is_open(id));
    assert!(d.command(id, BreakerCommand::Close));
    d.tick(&ElectricalInputs { dt_s: 0.05, ..Default::default() }, &Faults::default(), &mut sink);
    assert!(!d.is_open(id));
}

#[test]
fn every_gate_names_a_real_unit() {
    let d = DeepElectrical::new();
    let ids = d.breaker_ids();
    for g in GATES { assert!(ids.contains(&g.breaker_id), "{}", g.breaker_id); }
}

#[test]
fn published_names_cover_one_ticks_output() {
    let mut d = DeepElectrical::new();
    let names = d.published_names();
    let mut seen = Vec::new();
    d.tick(&ElectricalInputs { dt_s: 0.05, ..Default::default() }, &Faults::default(), &mut |k, _| seen.push(k.to_owned()));
    for k in seen { assert!(names.contains(&k), "{k}"); }
}
```

Write `facade_matches_the_golden_run` in full, reusing the golden scenario helper moved in Task 3.

- [ ] **Step 2: Implement.** Build `GATES` from `E:/fbw-int/plugin/src/breakers.rs`: every `BreakerDef` with `plugin_var: Some(var)` whose `id` is also a deep catalogue id, as `Gate { breaker_id: id, variable: var, open_when_true: var.ends_with("_OPEN") }`.
- [ ] **Step 3: Run** `cargo test -p deep_electrical`. Expected: PASS.
- [ ] **Step 4: Save the diff** to `task4.diff`.

### Task 5: Sync script and vendoring

**Files:**
- Create: `E:/fbw-int/plugin/scripts/sync-deep-electrical.sh`

- [ ] **Step 1: Write the script**

```bash
#!/bin/sh
# Copy the deep_electrical crate into the FlyByWire Development-build worktree
# and record its content hash; a380_systems' test refuses a stale copy.
set -e
SRC="$(cd "$(dirname "$0")/.." && pwd)/crates/deep_electrical"
DST="${1:-/d/A380/fbw-xp-worktrees/fs2020-672384b}/fbw-a380x/src/wasm/systems/deep_electrical"
rm -rf "$DST"
mkdir -p "$DST"
cp -r "$SRC/Cargo.toml" "$SRC/src" "$SRC/tests" "$DST/"
( cd "$SRC" && find Cargo.toml src tests -type f | sort | xargs sha256sum ) | sha256sum | cut -c1-64 > "$DST/SOURCE_HASH"
echo "synced $(cat "$DST/SOURCE_HASH")"
```

- [ ] **Step 2: Run it and check** that `fbw-a380x/src/wasm/systems/deep_electrical/SOURCE_HASH` exists. Add `"fbw-a380x/src/wasm/systems/deep_electrical"` to the worktree's root `Cargo.toml` `[workspace] members`.
- [ ] **Step 3: Verify** `cargo test -p deep_electrical` in the worktree. Expected: PASS (same tests).

### Task 6: Gate patches onto `672384b`

**Files:** the files touched by `E:/fbw-int/plugin/patches/fbw-rust/{power-path-cabinfan,power-path-fire-loops,power-path-lgciu,power-path-operating-channel,power-path-pending-egpwc,power-path-radioaltimeter,power-path-valve-breakers,power-path-pending-TR-GEN,bus-feeder-breakers,bus-feeder-breakers-2,breaker-open-polarity,breakers,generator-overload-trip,tr-thermal-model,static-inverter-efficiency,battery-thermal-runaway,electrical}.patch`, in the worktree.

- [ ] **Step 1: Apply in dependency order.** `electrical`, `static-inverter-efficiency`, `tr-thermal-model`, `battery-thermal-runaway`, `generator-overload-trip`, `bus-feeder-breakers`, `bus-feeder-breakers-2`, then the seven `power-path-*` that apply, each with `git apply --ignore-whitespace`. After each, run `cargo test -p systems -p a380_systems --lib -q`. Stop on the first failure.
- [ ] **Step 2: Hand-port** `breakers.patch`, `breaker-open-polarity.patch` and `power-path-pending-TR-GEN.patch`. For each rejected hunk, read the patch's intent (its own comments) and apply the same change to this commit's code. Keep variable names identical to the patch.
- [ ] **Step 3: Write the default-safe test** in `a380_systems`. It builds the A380 test bed, runs with the gate variables unwritten, and asserts every consumer each gate controls is powered with ground power on:

```rust
#[test]
fn unwritten_breaker_gates_leave_every_consumer_powered() {
    let mut test_bed = test_bed_with().on_the_ground().ext_pwr_on().run_with_delta(Duration::from_secs(5));
    // one assertion per gated consumer, e.g.:
    assert!(test_bed.is_tr_1_powered_and_converting());
}
```

Use the existing `A380` test-bed helpers and add one per gated consumer. The list of gated consumers is the set of `get_identifier(...BREAKER...)` names the applied patches introduce. Any gate that reads `_CLOSED` polarity must be converted to `_OPEN` polarity, per the spec.
- [ ] **Step 4: Save the diff** (worktree) to `task6.diff`.

### Task 7: Failure merge

**Files:**
- Modify: `fbw-common/src/wasm/systems/systems/src/simulation/mod.rs` (the `Aircraft` trait)
- Modify: `fbw-common/src/wasm/systems/systems_wasm/src/failures.rs`, `lib.rs::read_failures_into_simulation`

**Interfaces:**
- Produces: `Aircraft::derived_failure_ids(&self) -> Vec<u64>` (default `Vec::new()`), and `Failures::merged(&mut self, derived: &[u64]) -> Option<FxHashSet<FailureType>>`.

- [ ] **Step 1: Failing test** in `failures.rs`:

```rust
#[cfg(test)]
mod tests {
    use super::*;
    use systems::failures::FailureType;

    fn failures() -> Failures {
        let mut f = Failures::default();
        f.add_failures([(24_000, FailureType::TransformerRectifier(1)), (24_001, FailureType::TransformerRectifier(2))]);
        f
    }

    #[test]
    fn a_derived_failure_joins_the_crew_set_and_leaves_it_alone() {
        let mut f = failures();
        f.handle_failure_update("[24000]");
        let set = f.merged(&[24_001]).expect("changed");
        assert!(set.contains(&FailureType::TransformerRectifier(1)) && set.contains(&FailureType::TransformerRectifier(2)));
        assert!(f.merged(&[24_001]).is_none(), "nothing changed");
        let set = f.merged(&[]).expect("derived cleared");
        assert!(set.contains(&FailureType::TransformerRectifier(1)) && !set.contains(&FailureType::TransformerRectifier(2)));
    }
}
```

- [ ] **Step 2: Implement.** `Failures` keeps `crew: FxHashSet<FailureType>`, `derived: Vec<u64>` (sorted) and `dirty: bool`. `handle_failure_update` sets `crew` and `dirty`. `merged(derived)` sorts the input; if `dirty` or it differs from the stored list, it stores it, clears `dirty` and returns `Some(crew ∪ mapped(derived))`, else `None`. `read_failures_into_simulation` calls `merged(&simulation.aircraft().derived_failure_ids())`. Keep `get_updated_active_failures` if anything else still calls it; otherwise remove it.
- [ ] **Step 3: Run** `cargo test -p systems_wasm -p systems --lib -q`. Expected: PASS.
- [ ] **Step 4: Save the diff** to `task7.diff`.

### Task 8: `CircuitProtection` host

**Files:**
- Create: `fbw-a380x/src/wasm/systems/a380_systems/src/electrical/circuit_protection.rs`, `reset_panel_map.rs`
- Modify: `a380_systems/src/electrical/mod.rs` (`mod circuit_protection; pub use ...;`), `a380_systems/src/lib.rs` (field, construction, `update_after_power_distribution` call at the end, `accept`, `derived_failure_ids`), `a380_systems/Cargo.toml` (`deep_electrical = { path = "../deep_electrical" }`), `a380_systems_wasm/src/lib.rs` (`.provides_aircraft_variable("GENERAL ENG OIL TEMPERATURE", "celsius", n)?` for n in 1..=4, if not already present)

**Interfaces:**
- Consumes: `deep_electrical::{DeepElectrical, ElectricalInputs, ElectricalControls, BreakerCommand, GATES, lvar_key, frame::Faults}`.
- Produces: `CircuitProtection::new(&mut InitContext)`, `update(&mut self, &UpdateContext)`, `derived_failure_ids(&self) -> Vec<u64>`, and the `SimulationElement` impl.

Input variables (the names `E:/fbw-int/plugin/src/deep/plugin.rs` registers; in MSFS they're the same FlyByWire variables, prefixed `A32NX_` by the registry):

| `ElectricalInputs` field | Variable (per index) | Source line in `deep/plugin.rs` |
|---|---|---|
| `ac_bus_powered[n-1]` | `ELEC_AC_{n}_BUS_IS_POWERED` | 705 |
| `ac_bus_volts[n-1]` | `ELEC_AC_{n}_BUS_POTENTIAL` | 703 |
| `dc_bus_powered[n-1]` | `ELEC_DC_{n}_BUS_IS_POWERED` | 706 |
| `dc_bus_volts[n-1]` | `ELEC_DC_{n}_BUS_POTENTIAL` | 704 |
| `engine_n2_frac[n-1]` | `ENGINE_N2:{n}` ÷ 100 | 289 |
| `engine_running[n-1]` | `ENGINE_STATE:{n}` == 1 (`ENGINE_STATE_ON`) | 291 |
| `engine_oil_temp_c[n-1]` | simvar `GENERAL ENG OIL TEMPERATURE:{n}` | 293 |
| `controls.eng_gen_pb_on[n-1]` | `OVHD_ELEC_ENG_GEN_{n}_PB_IS_ON` | 308 |
| `controls.apu_gen_pb_on[n-1]` | `OVHD_ELEC_APU_GEN_{n}_PB_IS_ON` | 419 |
| `controls.bat_pb_auto[n-1]` | `OVHD_ELEC_BAT_{n}_PB_IS_AUTO` | 420 |
| `controls.apu_start_pb_on` | `OVHD_APU_START_PB_IS_ON` | 422 |
| `controls.fire_pb_released[n-1]` | `FIRE_BUTTON_ENG{n}` | 304 |
| `controls.fire_agent_pb_pressed[n-1][b-1]` | `OVHD_FIRE_AGENT_{b}_ENG_{n}_IS_PRESSED` | 305 |
| `controls.fire_pb_apu_released` | `FIRE_BUTTON_APU` | 409 |
| `controls.fire_agent_pb_apu_pressed` | `OVHD_FIRE_AGENT_1_APU_1_IS_PRESSED` | 410 |
| `controls.gear_door_commanded_open[i]` | `GEAR_DOOR_{CENTER,LEFT,RIGHT}_POSITION` | 416 |
| `controls.starter_engaged[n-1]` | the `deep/plugin.rs` formula at 956-968: master && igniter == 2 && state ∈ {Starting, Restarting} && timer ≥ 1.7, from the master/igniter/timer variables registered next to `ENGINE_STATE` | 956-968 |
| `apu_running` | `OVHD_APU_START_PB_IS_AVAILABLE` | 1191 |
| `gpu_plugged_in` | any `EXT_PWR_AVAIL:{1..4}` | 423 |
| `on_ground` | simvar `SIM ON GROUND` | (MSFS) |
| `sat_c` | simvar `AMBIENT TEMPERATURE` | (MSFS) |
| `tas_ms` | simvar `AIRSPEED TRUE` × 0.514444 | (MSFS) |
| `dt_s` | `context.delta_as_secs_f64()` | |
| `cargo_door_cmd`, `cargo_door_percent`, `main_avionics_bay_temp_c` | `None` | not modelled in MSFS |

Before coding, confirm each FADEC variable name in this worktree's `fbw-a380x/src/wasm/fadec_a380/` source (grep `ENGINE_STATE`, `ENGINE_N2`, `ENGINE_TIMER`, `ENGINE_IGNITER`, `ENGINE_MASTER`), and note the file and line next to each in the code's doc comment.

- [ ] **Step 1: Write the failing integration tests** in `circuit_protection.rs` (`#[cfg(test)]`, using the `a380_systems` test bed the crate already uses):

```rust
#[test]
fn opening_tr_1_stops_flybywires_tr_1() {
    let mut t = test_bed().on_the_ground().ext_pwr_on().run(Duration::from_secs(5));
    assert!(t.tr_1_is_converting());
    t.write_by_name("BKR_TR_1_CMD", 1.0);
    t.run(Duration::from_secs(1));
    assert!(t.read_by_name::<bool>("BKR_TR_1_OPEN"));
    assert!(!t.tr_1_is_converting());
    assert_eq!(t.read_by_name::<f64>("BKR_TR_1_CMD"), 0.0, "the command is consumed");
}

#[test]
fn latching_the_fmc_a_reset_opens_its_units_and_releasing_restores_them() {
    let mut t = test_bed().on_the_ground().ext_pwr_on().run(Duration::from_secs(5));
    t.write_by_name("RESET_PANEL_FMC_A", true);
    t.run(Duration::from_secs(1));
    assert!(t.read_by_name::<bool>("BKR_FMS_1_NORMAL_BKR_OPEN"));
    t.write_by_name("RESET_PANEL_FMC_A", false);
    t.run(Duration::from_secs(1));
    assert!(!t.read_by_name::<bool>("BKR_FMS_1_NORMAL_BKR_OPEN"));
}

#[test]
fn a_derived_failure_reaches_the_aircraft() {
    // Open tr-1 through the host; its coupling verdict must appear in
    // A380::derived_failure_ids() as 24_000 (TransformerRectifier(1)).
}

#[test]
fn the_host_publishes_its_own_tick_time() {
    let mut t = test_bed().run(Duration::from_secs(1));
    assert!(t.read_by_name::<f64>("CIRCUIT_PROTECTION_TICK_US") > 0.0);
}
```

Write `a_derived_failure_reaches_the_aircraft` in full. If `tr-1` is not a deep catalogue id (the catalogue lists TR units under the electrical network, not the 399), use the first `GATES` entry and its gated consumer instead, and say so in the test's doc comment. The `tr_1_is_converting` helper wraps the test bed's existing TR state query. Add any missing helper to the test bed module.

- [ ] **Step 2: Implement `CircuitProtection`.**
  - It owns: `DeepElectrical`, the input identifiers (table above), one `(breaker_id, open_id, status_id, current_id, cmd_id)` tuple per `breaker_ids()` entry (names `BKR_{lvar_key}_OPEN/_STATUS/_CURRENT_A/_CMD`), published-name identifiers for `published_names()` (key: `lvar_key`-sanitised names, since the raw names contain hyphens), gate identifiers from `GATES`, RESET identifiers for the 52 names in `reset_panel_map.rs`, `tick_us_id` (`CIRCUIT_PROTECTION_TICK_US`), cached input values, pending commands, the RESET latch state, and `last_derived: Vec<u64>`.
  - **`read`:** reads inputs, commands (`1.0` → `Open`, `2.0` → `Close`) and RESET latches.
  - **`update`:**
    - applies commands;
    - a RESET latch rising edge sends `Open` to its units, and a falling edge sends `Close`;
    - builds `ElectricalInputs`, ticks with `Faults::default()`, and times the tick with `std::time::Instant` (on wasm32 `Instant` is unavailable, so there it uses `context.delta()`-independent `0.0`; guard with `#[cfg(not(target_arch = "wasm32"))]`, and on wasm use the MSFS `SIMCONNECT` clock if available, else publish `-1.0` meaning "not measured");
    - stores the published values;
    - maps `derived_failures()` to ids.
  - **`write`:** writes every published value, `_OPEN`/`_STATUS`/`_CURRENT_A`, every gate (`is_open(breaker_id) == open_when_true` for `_OPEN` gates), `0.0` to every consumed command, and the tick time.
- [ ] **Step 3: Build `reset_panel_map.rs`.**

```rust
/// RESET panel pushbutton name (`CB_<NAME>` node, `L:A32NX_RESET_PANEL_<NAME>`) ->
/// the deep units it opens while latched. Only correspondences with evidence;
/// everything else maps to an empty slice and is listed in UNMAPPED.
pub const RESET_PANEL: &[(&str, &[&str])] = &[
    ("TR1", &["tr-1"]),                 // plugin src/breakers.rs:508 panel_node
    ("TR_2A", &["tr-2"]),               // src/breakers.rs:509
    ("ESS_TR", &["tr-ess"]),            // src/breakers.rs:510
    ("LGCIS1", &["lgciu-1-normal-bkr", "lgciu-1-2nd-bkr"]),  // src/breakers.rs:690 + catalogue consumer
    ("LGCIS2", &["lgciu-2-normal-bkr", "lgciu-2-2nd-bkr"]),
    ("PACK1_CTL", &[/* breakers.rs:459 id, if it is a deep catalogue id */]),
    ("PACK2_CTL", &[/* breakers.rs:471 id, if it is a deep catalogue id */]),
    ("FMC_A", &["fms-1-normal-bkr", "fms-1-2nd-bkr"]),
    ("FMC_B", &["fms-2-normal-bkr", "fms-2-2nd-bkr"]),
    ("FMC_C", &["fms-3-normal-bkr", "fms-3-2nd-bkr"]),
    ("CPCS1", &["cpiom-b1-cpcs"]), ("CPCS2", &["cpiom-b2-cpcs"]),
    ("TCS1", &["cpiom-b1-tcs"]), ("TCS2", &["cpiom-b2-tcs"]),
    ("VCS1", &["cpiom-b1-vcs"]), ("VCS2", &["cpiom-b2-vcs"]),
    ("ATC", &["xpdr-1", "xpdr-2"]),
];
pub const UNMAPPED: &[&str] = &["AESU1", "AESU2", /* ... every other of the 52 labels ... */];
```

Resolve each commented placeholder above by reading the cited line. An id that isn't in `DeepElectrical::breaker_ids()` moves its name to `UNMAPPED`. A unit test asserts every mapped id exists and that `RESET_PANEL` names ∪ `UNMAPPED` equals the 52 labels (listed in the test).
- [ ] **Step 4: Stale-copy guard test** in `circuit_protection.rs`:

```rust
#[test]
fn the_vendored_deep_electrical_is_the_synced_one() {
    let recorded = include_str!("../../../deep_electrical/SOURCE_HASH").trim();
    assert_eq!(recorded.len(), 64, "run scripts/sync-deep-electrical.sh");
}
```

- [ ] **Step 5: Benchmark.** Add an `#[ignore]` test that ticks `DeepElectrical` 2000 times and prints the mean µs per tick. Run it with `--ignored --nocapture` in release, and record the number.
- [ ] **Step 6: Run** `cargo test -p a380_systems -p deep_electrical --lib -q` (release for the benchmark). Expected: PASS.
- [ ] **Step 7: Save the diff** to `task8.diff`.

### Task 9: EFB Study tab with live breakers (agent)

**Files:**
- Copy: `D:/fbw-aircraft/fbw-common/src/systems/instruments/src/EFB/Study/**` → worktree `fbw-common/src/systems/instruments/src/EFB/Study/`
- Modify: worktree `EFB/Efb.tsx` (import `Study` and add `<Route path="/study" component={Study} />` after `/failures`), `EFB/ToolBar/ToolBar.tsx` (add `Diagram3` to the icon import, and a `ToolBarButton to="/study" tooltipText="Study"` after Failures). These are the exact edits from the E: tree commit `418c74f`.
- Modify: `EFB/Study/Pages/BreakerPanels.tsx` (live state and commands).

- [ ] **Step 1: Make `BreakerPanels` live.** For each breaker row, render state from `useSimVar(\`L:A32NX_BKR_${lvarKey(b.id)}_OPEN\`, 'bool', 500)`, `_STATUS` and `_CURRENT_A` (use the EFB's existing `useSimVar` hook, found via grep in `EFB/`). Add Open and Close buttons that call `SimVar.SetSimVarValue(\`L:A32NX_BKR_${key}_CMD\`, 'number', 1 or 2)`. Add `lvarKey` in `catalogue.ts`:

```ts
export function lvarKey(id: string): string {
  return id.replace(/[^A-Za-z0-9]/g, '_').toUpperCase();
}
```

It must match the Rust `lvar_key` exactly. Add a unit test if the EFB has a test setup (look for `*.test.ts` in `EFB/`); otherwise export it and note that it's verified by inspection against `deep_electrical::lvar_key`'s tests.
- [ ] **Step 2: Type-check** with the worktree's own TypeScript config inside the dev-env container: `./scripts/dev-env/run.sh npx tsc --noEmit -p fbw-common/src/systems/instruments/src/EFB` (or the tsconfig path the instruments build uses). Expected: no errors in `EFB/Study/**`.
- [ ] **Step 3: Save the diff** to `task9.diff`.

### Task 10: RESET panel XML (agent)

**Files:**
- Modify: a copy of the package's `SimObjects/AirPlanes/FlyByWire_A380_842/model/A380_COCKPIT.xml` → `D:/A380/fbw-build/wasm-fs2020/out/A380_COCKPIT.xml`

- [ ] **Step 1: Add the 42 unwired buttons.** Next to the existing 10 `<UseTemplate Name="FBW_Airbus_RESET_PANEL_BUTTON">` uses (around line 4732), add one per `CB_<NAME>` node in `a380_cockpit.gltf` that isn't already used and isn't `CB_EMPTY*`:

```xml
<UseTemplate Name="FBW_Airbus_RESET_PANEL_BUTTON">
    <NAME>TR1</NAME>
</UseTemplate>
```

- [ ] **Step 2: Verify with a script.** Parse the XML: every one of the 52 labels is used exactly once, no `CB_EMPTY*` is used, and the file is well-formed. Diff it against the original; only those insertions may differ.
- [ ] **Step 3: Save the diff** to `task10.diff`.

### Task 11: Catalogue regeneration

- [ ] **Step 1:** Regenerate `catalogue.json` from the current plugin with the documented exporter (`E:/fbw-int/plugin/docs/catalogue-export.md`). Write it to `D:/A380/fbw-build/wasm-fs2020/out/catalogue.json`.
- [ ] **Step 2:** Check that it has 399 breakers, and that the failure IDs 27_101, 27_102, 32_130..=32_134 and 34_109..=34_111 are present (the catalogue must reflect today's failures).

### Task 12: Build, install script, guide

- [ ] **Step 1: Build the WASM.** Run `sh scripts/sync-deep-electrical.sh`, then in the container (as in the previous build) `cargo build -p a380_systems_wasm --target wasm32-wasip1 --release`, then `wasm-opt -O1 --signext-lowering --enable-bulk-memory --enable-nontrapping-float-to-int -o /wasmout/systems.wasm ...`.
- [ ] **Step 2: Build the EFB.** In the container, run `npm ci`, then `npm run build-a380x:instruments`, then copy `fbw-a380x/out/flybywire-aircraft-a380-842/html_ui/Pages/VCockpit/Instruments/A380X/EFB/` to `out/EFB/`.
- [ ] **Step 3: Write `install-msfs-circuit-protection.ps1`.** It extends `install-systems-wasm.ps1`: the same exact-commit check, then it installs `systems.wasm`, `EFB/**`, `catalogue.json` (into `html_ui/Pages/VCockpit/Instruments/A380X/EFB/`) and `A380_COCKPIT.xml`. It keeps `*.original` backups for every replaced file (a folder snapshot for `EFB/`) and rewrites each file's `layout.json` `size`/`date`, adding entries for new files. `-Restore` puts every original back and removes added entries.
- [ ] **Step 4: Update `INTEGRATION.md`:**
  - what is installed;
  - the L:var contract (`A32NX_BKR_<KEY>_{OPEN,STATUS,CURRENT_A,CMD}`, `A32NX_RESET_PANEL_<NAME>`, `A32NX_CIRCUIT_PROTECTION_TICK_US`);
  - the RESET mapping table and the unmapped list;
  - the manual test checklist;
  - restore.

### Task 13: Final verification

- [ ] Plugin full suite: the same result as Task 0's baseline.
- [ ] Golden run passes, through both the plugin adapter and the crate facade.
- [ ] Worktree: `cargo test -p systems -p systems_wasm -p a380_systems -p deep_electrical --lib -q`, all green.
- [ ] Container WASM build succeeds, the module is valid (`\0asm` header), and it contains `BKR_` and `CIRCUIT_PROTECTION_TICK_US`.
- [ ] Record the benchmark µs per tick in `INTEGRATION.md`.
- [ ] Save every diff to `E:/fbw-debug/msfs-cb/`, then report.
