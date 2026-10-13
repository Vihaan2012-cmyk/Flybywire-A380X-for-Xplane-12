# Porting the deep systems layer to MSFS

Scope of this document: can the ~19-area deep systems layer under `src/deep/`,
`src/physics/`, `src/electrical` (there is no such directory — see §2),
and `src/breakers.rs` reach FlyByWire's official A380X in Microsoft Flight
Simulator, riding on a Rust→WASM pipeline FlyByWire already operates. This is
a plan, not a build: nothing in this repository or in `D:\fbw-aircraft`
(read-only reference) was changed to produce it. Every number below was
counted from the tree on 2026-09-20 with `wc`, `grep` and reading the actual
source; where a count could not be obtained without compiling the crate
(which risked colliding with two other agents editing this tree live), that
limit is stated rather than guessed around.

## Verdict, up front

**Yes, and the path already exists.** FlyByWire does not merely build Rust to
WASM for MSFS as a side project — it is the actual, shipping mechanism behind
the A380X's and A32NX's own systems (`systems.wasm`, loaded as one of four
sibling gauge modules in `panel.cfg`). The layering FlyByWire already uses —
a sim-agnostic `systems` crate, a sim-agnostic aircraft crate (`a380_systems`),
a generic MSFS glue crate (`systems_wasm`), and a thin per-aircraft `cdylib`
entry point — is structurally the same shape this project's own `deep::live`
contract (`Truth` in, published names out) already assumes. The port is
mostly about reaching that pipeline with our own `cdylib`, not inventing a
new one.

The single largest obstacle is not the systems physics — 92%+ of it is
already platform-free (§2) — it is that **roughly a sixth of this crate
(≈30,700 lines) exists purely to make FlyByWire's *compiled* JavaScript/
TypeScript avionics run inside X-Plane at all**, a problem MSFS does not
have because it hosts that TypeScript natively. That code does not port; it
is deleted, and what replaces it (real TypeScript, built by FlyByWire's own
toolchain) is less code and structurally sounder than what it replaces. That
is the one place where "port" is the wrong word for "this gets smaller."

## 0a. CHOSEN APPROACH (post-plan): a fifth WASM module, not a port

The body of this document weighs two routes: bring our crate into MSFS
inside the sandbox (section 7), or run it out of process (7b). A third
route was found afterwards and is the one to build. It is better than
both, and most of the constraints the rest of this document reasons about
do not apply to it.

**MSFS already loads four independent WASM modules for this aircraft.**
From `SimObjects/AirPlanes/FlyByWire_A380X/attachments/flybywire/Part_Interior_Cockpit/panel/panel.cfg`:

```
[VCockpit21]
htmlgauge00=...wasm_module=systems.wasm&wasm_gauge=systems, 0,0,1,1
htmlgauge01=...wasm_module=fbw.wasm&wasm_gauge=fbw, 0,0,1,1
htmlgauge02=...wasm_module=fadec-a380x.wasm&wasm_gauge=Gauge_Fadec,0,0,1,1
htmlgauge03=...wasm_module=extra-backend-a380x.wasm&wasm_gauge=Gauge_Extra_Backend,0,0,1,1
```

We add a fifth. No fork of their Rust, no replacement of `systems.wasm`,
no decompilation, no rebasing onto their releases. The two modules talk
through LVars and simvars, which is exactly the authority inversion
already built and proven in X-Plane (`docs/deep/authority.md`, 49
couplings): our model computes the physics and drives their failure
variables at their resolution.

**Integration is additive, not a patch.** The aircraft uses MSFS's
modular SimObject system -- `common/`, `attachments/`, `presets/`, with no
`base_container` anywhere -- and the preset's own `panel.cfg` is nothing
but `[MODULAR_MERGE] auto = true`. Attachments are auto-discovered and
merged. So the module is registered by adding one attachment folder
carrying a `panel.cfg` with a single extra `htmlgauge` line. **No file of
FlyByWire's is edited.**

One question is still open and worth settling before building: whether
that attachment folder must sit inside their package directory (so an
installer writes files into their install, still adding only, never
editing) or whether a *separate* package can contribute an attachment (so
the mod is entirely self-contained). Either way the integration is
additive; the difference is only whether their folder is touched at all.

**What this route costs**, stated plainly, because it is not free:

- Our module influences theirs only through variables. It cannot reach
  inside their models, so anything we want to override must have a
  variable they read.
- Their failure channel is a *set* of active ids with no magnitude (see
  the RESOLVED note in section 8), so the authority couplings threshold
  to on/off.
- We share the frame budget with four other modules, and the output-side
  question in section 4 -- what it costs to publish thousands of values
  per frame as LVars -- is unchanged and still unmeasured. **That
  measurement remains the first thing to do before writing code.**

Sections 1, 2, 3 and 4 (the existing pipeline, how much of our code is
platform-free, the `Truth` mapping and the output side) all still apply,
because our module is still a Rust WASM module built the way theirs are.
Sections 5, 6 and 7 are written for a route we are no longer taking: the
ECAM/ECL bridge and the EFB pages become ordinary TypeScript work in
their repo, and persistence is a question for whatever hosts our
catalogue, not for the sandbox.

## 0b. The authority couplings need no LVars at all

Checked directly, and it shrinks the boundary again.

`docs/deep/authority.md` expresses all 49 couplings as a FlyByWire
`FailureType` plus a numeric id -- `24_elec.vfg-1` drives
`Generator(1)` at `24_020`, the AC buses drive
`ElectricalBus(AlternatingCurrent(1..4))` at `24_100..24_103`, and so on.

Those ids are **FlyByWire's own**, not a parallel numbering. From
`fbw-a380x/src/wasm/systems/a380_systems_wasm/src/lib.rs:148`:

```
(24_020, FailureType::Generator(1)),
(24_021, FailureType::Generator(2)),
(24_022, FailureType::Generator(3)),
(24_023, FailureType::Generator(4)),
(24_030, FailureType::ApuGenerator(1)),
```

and `systems_wasm/src/failures.rs` takes exactly that map through
`.with_failures(...)`, then accepts a **JSON array of ids** at runtime via
`Failures::handle_failure_update`, which `MsfsHandler` applies each frame
through `simulation.update_active_failures`.

So the couplings do not cross as 49 LVars. They cross as **one array of
integers**, on a channel that already exists, carrying ids we already use.
That is the cheapest possible shape and it needs no new mechanism.

**The real problem on this path is ownership, not cost.** That channel is
already written by FlyByWire's own front end -- the EFB failures page is
how a user arms a failure today. Two writers on one channel means the last
write wins and the crew's armed failures and our derived verdicts erase
each other every frame. Three ways out, in order of preference:

1. **Send the union.** Our module reads the current active set, adds its
   derived ids, writes back. Needs a read path on that channel, which has
   not been confirmed to exist -- `get_updated_active_failures` is
   consumed by the handler, not exposed.
2. **A channel of our own**, with a small patch to their handler to merge
   two sources. This is the one place a change to FlyByWire's code would
   genuinely be warranted, and it is a few lines, not a fork.
3. **Drive their failures through LVars instead** where a failure has an
   equivalent variable, keeping the id channel untouched. Falls back to
   the per-variable cost the benchmark measures, but only for the subset
   that needs it.

Settle this before writing the coupling layer. It is the one genuine
architectural decision left on the boundary, and unlike the throughput
question it will not resolve itself by being measured.

## 0. Method

Every count in this document was produced by one of:

- `wc -l` over `find ... -name '*.rs'` for line counts,
- `grep -rl`/`grep -rc` for file/occurrence counts, always checked by hand
  against a sample to rule out doc-comment false positives (done explicitly
  in §2),
- reading a file in full and citing its line numbers,
- a test already committed in this tree whose assertion is a lower bound on
  a real, code-verified quantity (cited as such, not as the true total),
- direct reads of `D:\fbw-aircraft`'s Cargo.toml/toolchain files and a
  cloned copy of `msfs-rs` (FlyByWire's own MSFS bindings fork) read from a
  scratch directory, never written into either tree.

Two things this document does **not** do: run `cargo build`/`cargo test` on
this crate (two other agents are editing it live; a build could collide with
in-flight edits or take an unbounded time on 176,880 lines), or touch
anything under `D:\fbw-aircraft`.

## 1. FlyByWire's existing Rust→WASM pipeline

Confirmed real and shipping, not aspirational. The proof chain:

**Crate layering** (`D:\fbw-aircraft\Cargo.toml`, workspace):

| Crate | Path | Depends on | MSFS-aware? |
|---|---|---|---|
| `systems` | `fbw-common/src/wasm/systems/systems` | uom, nalgebra, rand, bitflags — no `msfs` | No. Pure computation. |
| `a380_systems` | `fbw-a380x/src/wasm/systems/a380_systems` | `systems` (path), serde/toml | No. The A380 model (`struct A380`), still sim-agnostic. |
| `systems_wasm` | `fbw-common/src/wasm/systems/systems_wasm` | `systems`, `msfs` (git) | Yes. Generic glue: `MsfsSimulationBuilder`, `Variable`, `MsfsHandler`. |
| `a380_systems_wasm` | `fbw-a380x/src/wasm/systems/a380_systems_wasm` | all of the above | Yes. `crate-type = ["cdylib"]` — the actual binary MSFS loads. |

This is exactly this project's own `Truth`/`Area` split, one layer further
factored: FlyByWire already separates "physics" from "aircraft" from "sim
glue" from "the loaded binary." A ported deep-systems module would add a
fifth crate at the `a380_systems_wasm` tier, not restructure anything above
it.

**Build**: target `wasm32-wasip1` (not `wasm32-unknown-unknown`), Rust
1.96.0 pinned in `rust-toolchain.toml`, linked against the proprietary
**MSFS SDK**'s WASM sysroot (`.cargo/config.toml`, path
`/workdir/MSFS_SDK/WASM/wasi-sysroot`) — a licensed download, not on
crates.io or in this tree. The whole toolchain is packaged as the Docker
image `ghcr.io/flybywiresim/dev-env`, invoked through
`scripts/dev-env/run.sh`. The exact command that produces the shipped
artifact (`package.json`, `build-a380x:systems`):

```
cargo build -p a380_systems_wasm --target wasm32-wasip1 --release
wasm-opt -O1 --signext-lowering --enable-bulk-memory --enable-nontrapping-float-to-int \
  -o .../panel/systems.wasm target/wasm32-wasip1/release/a380_systems_wasm.wasm
```

orchestrated by an `igniter.config.mjs` task graph (`build-a380x:systems-host`
→ `build-a380x:systems` → `build-a380x:fbw`).

**The `msfs` crate** is `msfs-rs`
(`https://github.com/flybywiresim/msfs-rs`, branch `main`), FlyByWire's own
fork, a git dependency — not vendored anywhere, so it was cloned to a scratch
directory and read directly rather than guessed at. What it gives:

- `#[msfs::gauge(name=...)]` — an attribute macro turning an `async fn` into
  the panel's gauge entry point, with an event loop (`gauge.next_event()`)
  delivering `MSFSEvent::PreDraw`/`SimConnect`/etc.
- `AircraftVariable::from(name, units, index) -> Result<Self, _>`, `.get::<T>()`
  — the simvar read path (`msfs/src/legacy.rs:44-86`).
- `NamedVariable::from(name)`, `.get_value::<T>()`/`.set_value(v)` — the LVar
  (`L:`) read/write path (`msfs/src/legacy.rs:87-107`). This is the direct
  analogue of this project's `fbw/`-prefixed datarefs: an LVar is globally
  readable by every other gauge and add-on in the sim, the same broadcast
  property `Vars::publish` relies on today.
- `SimConnect` (`msfs/src/sim_connect.rs`, 791 lines): data definitions,
  client events, client data areas, AI object creation, system-event
  subscriptions — richer than X-Plane's dataref-only model, not poorer.
- `nvg` (NanoVG) for drawing directly on a glass panel from Rust, if a
  future instrument needed it — not required for this plan's scope.
- **What it does not expose**: no filesystem API surfaced anywhere in
  `msfs-rs`, and confirmed by direct search — the *only* `File::create` in
  the entirety of `fbw-a380x/src/wasm/systems` and
  `fbw-common/src/wasm/systems` is inside a `#[cfg(test)]` host-side dev
  tool in `a380_systems/src/pneumatic.rs` (dumps a debug graph on the
  developer's own machine), never compiled into the shipped `.wasm`. This
  project's own file-based persistence (§6) has no demonstrated MSFS
  equivalent in this SDK.

**Loading**: `panel.cfg` (`FlyByWire_A380X/.../panel/panel.cfg`, section
`[VCockpit21]`) loads **four separate wasm modules** as sibling gauges, each
through the shared `WasmInstrument.html?wasm_module=X&wasm_gauge=Y` loader:
`systems.wasm` (this Rust pipeline), `fbw.wasm` (the C++ flight-control-law
module, a separate build entirely), `fadec-a380x.wasm`,
`extra-backend-a380x.wasm`. The architecture is already multi-module by
design — a fifth module (the deep-systems addon) is one more `wasm_module=`
line, not a merge into an existing binary.

## 2. How much of our code is already platform-free

| Directory | Files (.rs) | Lines | Real `use crate::xp` import? |
|---|---|---|---|
| `src/deep/` | 221 | 88,556 | 11 files (below) |
| `src/physics/` | 28 | 12,357 | 4 files (below) |
| `src/breakers.rs` (top-level, not under `deep/`) | 1 | 1,829 | No — written against `systems::simulation::{SimulatorReaderWriter, VariableIdentifier, VariableRegistry}`, FlyByWire's own sim-abstraction traits, the same ones FlyByWire's MSFS glue (`systems_wasm::MsfsSimulationBuilder`) implements. |

No `src/electrical/` directory exists as such — electrical modelling lives
entirely under `src/deep/electrical/` (7,140 lines), inside the platform-free
count above.

A first `grep -rl -iE 'xplm|XPLM|DataRef'` pass over `src/deep` returned 22
files; checked by hand, 11 of those are doc-comment prose *explaining* the
no-X-Plane-dependency rule (`live.rs`, `api.rs`, `mod.rs`,
`thermal_zones/{network,registry}.rs`), not real dependencies — confirmed
by grepping specifically for `use crate::xp` / `crate::Vars` imports rather
than the word "XPLM." The real, load-bearing count:

**11 files touch X-Plane directly, out of 249 files across `deep/`+`physics/`
(4.4%), totalling 7,728 lines out of 100,913 (7.7%):**

```
src/deep/environment/live.rs                              1,220
src/deep/integration/environment_events_adapter.rs           293
src/deep/integration/fire_ice_adapter.rs                       87
src/deep/integration/sensors_adapter.rs                       119
src/deep/integration/weather_truth.rs                         375
src/deep/integration/xp_consequences.rs                       245
src/deep/plugin.rs                                          1,060
src/physics/adirs.rs                                        2,152
src/physics/damage.rs                                       1,144
src/physics/tyre.rs                                           667
src/physics/xp_effects.rs                                     366
```

`src/deep/plugin.rs` is *supposed* to be here — it is the one file the
`deep::live` module doc names as "the only place that knows both the
[`Truth`] contract and `crate::Vars`/X-Plane," i.e. the seam by design
(§3). `src/deep/environment/live.rs` pulls in X-Plane only for
`XPLMGetWeatherAtLocation` (real weather sampling — the same call
`weather_truth.rs`/`plugin.rs` already rate-limit to 10 Hz because X-Plane's
own header warns it is not for per-frame use). `physics/adirs.rs`,
`damage.rs`, `tyre.rs`, `xp_effects.rs` are the four physics modules that
read raw X-Plane state directly instead of through `Truth` (IRS/ADR/radio-alt
truth-vs-sensed modelling, exceedance tracking against real dataref limits,
per-wheel nitrogen physics, and mirroring this crate's own failure state
onto X-Plane's native failure datarefs, respectively) — each is a plausible,
named candidate for a small MSFS-side shim (§3), not a design flaw.

**The other 92.3% of this layer's lines** — every area's `mod.rs`, its
physics structs, its `registry.rs` (18 files, one per area), its tests — is
plain Rust over plain structs, with no dependency on X-Plane, MSFS, or any
simulator API at all. That is what `docs/deep/BRIEF.md`'s hard rule 2 ("std
only, no dependency on `Vars` or X-Plane") was written to guarantee, and the
grep above is the first time it has been checked mechanically rather than by
convention.

## 3. The platform boundary — `Truth`, field by field

`src/deep/live.rs` defines the contract every area is written against:
`Truth` in (everything an area may read about the rest of the aircraft this
tick), `Faults` in (armed failure magnitudes, 0..1), `Area::publish` out
(named values), `Area::derived_failures` out (verdicts on components
FlyByWire also models — `docs/deep/authority.md`, §4 below). `src/deep/
plugin.rs` is the *only* file that fills `Truth` from X-Plane, and — this is
the useful part — **it already documents its own sourcing table for every
field**, comment-for-comment, as part of its module doc (`plugin.rs:10-97`).
That table is the actual field-by-field mapping this section would otherwise
have to re-derive; what follows re-expresses it by *MSFS source type* rather
than repeating it, so the two documents can be read side by side.

| `Truth` field group | X-Plane source (today) | MSFS source | Gap? |
|---|---|---|---|
| `dt_s` | frame delta, clamped | `sGaugeDrawData::delta_time()` (`msfs-rs`'s own `MSFSEvent::PreDraw` payload) | None. |
| `environment` (SAT, TAS, ambient pressure, precipitation, cloud layers) | `Truth::environment`, mostly real X-Plane `Var`s already mapped through `lib.rs`'s `mapping()` table, plus `XPLMGetWeatherAtLocation` for clouds | The plain scalars (`AMBIENT TEMPERATURE`, `AIRSPEED TRUE`, `AMBIENT PRESSURE`, `AMBIENT PRECIP RATE`) are all **already in FlyByWire's own `provides_aircraft_variable` list** in `a380_systems_wasm/src/lib.rs` — they read these MSFS simvars today for their own systems. Cloud-layer/turbulence data has no `msfs-rs`-exposed equivalent to `XPLMGetWeatherAtLocation`; MSFS's live-weather API is a separate, less granular SimConnect facility. | Partial — the scalar atmosphere is free (FlyByWire already publishes it); structured cloud/precip-type data is the real gap. |
| `altitude_ft`, `on_ground`, `pitch_deg`, `groundspeed_m_s`, `angle_of_attack_deg`, `radio_height_ft`, `aircraft_mass_kg` | Named X-Plane datarefs | All are named MSFS simvars FlyByWire's own `a380_systems_wasm` **already requests** (`PLANE ALT ABOVE GROUND`, `SIM ON GROUND`, `PLANE PITCH DEGREES`, `GPS GROUND SPEED`, `INCIDENCE ALPHA`, `TOTAL WEIGHT`, etc. — see the `provides_aircraft_variable` list read in §1) | None — same simvars, already wired for FlyByWire's own use; our module adds its own `AircraftVariable::from(...)` calls, unrelated to theirs. |
| `engine_n1_frac`/`n2_frac`/`n3_frac`, `engine_running`, oil pressure/temp/quantity, TGT, T25, HP-port pressure/temp, fuel flow | This crate's **own** `physics::engine` gas-path model output, written to plugin `Var`s, then read back into `Truth` | These are not X-Plane-native at all today — they are this project's own engine physics. Porting `physics::engine` (part of the platform-free 92%) means the same numbers exist in MSFS; they just need writing to MSFS `NamedVariable`s (LVars) instead of X-Plane `fbw/` datarefs, and reading back the same way. | None in principle — this is our own model, ported wholesale, not sourced from the platform either way. |
| `tyre_pressure_pa` | This crate's own `physics::tyre` nitrogen model | Same as above: our own physics, LVar in/out. | None. |
| `door_open_fraction` | `src/doors.rs`'s own X-Plane interactive-point model | MSFS's own door system is exposed as simvars (`INTERACTIVE POINT OPEN` is itself an **MSFS-native simvar name** that FlyByWire's `a380_systems_wasm` already requests — X-Plane's dataref of the same purpose was mapped onto this MSFS name by this crate's `lib.rs`, not the other way around) | None — likely simpler in MSFS, since the MSFS-native name already exists and this project invented the X-Plane mapping to match it. |
| `apu_running`, `apu_bleed_pressure_pa` | FlyByWire's own compiled A380 systems output, via named `Var`s (`A32NX_OVHD_APU_START_PB_IS_AVAILABLE`, ARINC 429 word) | Identical: FlyByWire's real TypeScript/Rust systems publish the same LVars in MSFS by the same names (this project deliberately reuses FlyByWire's own naming to stay unambiguous). | None — same publisher, different host. |
| `ac_bus_volts`, `dc_bus_volts`, `hydraulic_pressure_pa` | FlyByWire's own electrical/hydraulic system output, `A32NX_ELEC_*`/`A32NX_HYD_*` `Var`s | Same LVars, published by the same `a380_systems` code running in MSFS's `systems.wasm` instead of this port's `remote::Systems`. | None. |
| `controls.*` (≈40 fields: pushbuttons, selectors, levers) | Named `A32NX_*` `Var`s FlyByWire's compiled systems already write, or raw X-Plane cockpit datarefs for a few (brake pedal position, spoiler lever) | The `A32NX_*` LVars are host-independent — same names in MSFS. The handful sourced from raw X-Plane cockpit datarefs (`sim/cockpit2/controls/{left,right}_brake_ratio`, `speedbrake_ratio`) need MSFS equivalents: `BRAKE LEFT POSITION`/`BRAKE RIGHT POSITION` and `SPOILERS HANDLE POSITION` are the standard MSFS simvars for the same physical inputs. | Small, named, one-line-per-field remap — not a blocker. |
| `commanded_surfaces` (29 `HYD_*_DEFLECTION` `Var`s) | FlyByWire's own actuator output, `Var`s this crate already reads generically | Same `Var`s, same names, published by the same compiled systems code inside `systems.wasm`. | None. |
| `leg_on_ground`, `leg_touchdown_sink_speed_ms` | FlyByWire's own LGCIU output (`A32NX_LGCIU_1_*_GEAR_COMPRESSED`) plus X-Plane's `local_vy` for the edge capture | LGCIU output: same LVar. Vertical speed at the touchdown edge: `VERTICAL SPEED` is a standard MSFS simvar FlyByWire's own systems could equally read. | None. |
| `cabin_pressure_pa`, `cabin_temp_k` | FlyByWire's own pressurisation controller ARINC 429 word plus ambient | Same LVar; identical. | None. |
| `sun_elevation_deg` | `sim/graphics/scenery/sun_pitch_degrees` | MSFS has no simvar for solar elevation exposed through `msfs-rs`'s aircraft-variable catalogue in the same form — `fire_ice`'s own live system already documents that it passes `0` here today rather than inventing a flux (see `deep/live.rs`'s doc on `sun_elevation_deg`), so this is a pre-existing, already-labelled gap, not a new one MSFS introduces. | Real gap, but already a known no-op on the X-Plane side too — not a regression. |
| `published` (`PublishedFrame`) | In-process, one area reading another's previous-frame output | Unaffected by the host at all — pure Rust, carries over unchanged. | None. |

**The honest summary**: of the roughly 70 leaf fields in `Truth`
(40-odd top-level + ~30 `Controls`), the large majority are either (a) this
project's own physics output, which ports wholesale and only needs its I/O
calls retargeted from X-Plane datarefs to MSFS LVars, or (b) FlyByWire's own
published `A32NX_*` variables, which are **host-independent by name** and
require no remapping at all because the same compiled systems code runs in
both hosts. The fields that genuinely need a new MSFS-side source are a
short, named list: structured cloud/weather data (no MSFS equivalent found),
brake-pedal and spoiler-handle raw input (trivial MSFS simvar substitutes
exist), and solar elevation (already a documented no-op today). Nothing in
`Truth` was found with *no* MSFS path at all except cloud/weather structure.

## 4. The output side

Two publishing mechanisms exist today, and both need a retarget, not a
redesign:

**Per-frame published values.** Every area publishes through a closure
(`&mut dyn FnMut(&str, f64)`) — deliberately not given the variable registry
directly, so the same area can publish into the live plugin, a test harness,
or nothing (`deep/live.rs`'s own "Publishing" doc section). `deep::plugin::
Publisher` resolves each published name to a `VariableIdentifier` once at
startup and reuses it every frame; its own doc comment states the measured
scale: **"the 2445 values ten areas publish"** (`plugin.rs:417-421`), against
17 areas assembled in `all_areas()` today, so the true current figure is
higher than 2,445. In MSFS, `Vars::write`/`Vars::publish` (this crate's own
abstraction, `src/lib.rs`) is replaced by `NamedVariable::from(name).
set_value(v)` per published value — the exact same shape, since `msfs-rs`'s
`NamedVariable` is a direct LVar handle, not a batch API. The
`VariableIdentifier`-caching structure in `Publisher` carries over unchanged;
only the leaf write call changes.

**`failures::set_derived_levels`.** `docs/deep/authority.md`'s whole design
(§ "Where the lever actually is") is that the deep model does *not* argue
with FlyByWire's own systems about voltages or pressures for the three
systems modelled twice (electrical, hydraulics, pneumatics) — it expresses
its verdict through FlyByWire's own failure system, the same
`FailuresConsumer`/`FailureType` mechanism FlyByWire's compiled systems
already accept as an input. Crucially, **this mechanism is not X-Plane
specific at all**: `a380_systems_wasm/src/lib.rs`'s own `.with_failures([...])`
call (read directly, §1) registers the exact same `FailureType` catalogue
(`FailureType::Generator(1)`, `FailureType::TransformerRectifier(1)`, etc.)
against numeric ids, for MSFS, today. `crate::failures::set_derived_levels`
in this port and `.with_failures(...)`'s runtime consumer in
`systems_wasm::MsfsSimulationBuilder` are two front doors to the identical
underlying `a380_systems::A380::update_fault_flags` (or equivalent) logic.
Porting this output path means calling `systems_wasm`'s own failure-injection
API with the same ids `deep::live::DerivedFailure::fbw_id` already carries —
arguably **less** work than the current X-Plane path, which has to fake
being FlyByWire's own `FailuresConsumer::drive` from outside the compiled
systems binary; in MSFS, the deep-systems module and `a380_systems` are two
peer WASM modules that could share the SimConnect failure-bus mechanism
`systems_wasm` already defines for exactly this purpose.

## 5. The ECAM/ECL bridge

This is the section where MSFS is not merely "as good as" X-Plane but
strictly simpler, and it is worth stating why in numbers.

On X-Plane, FlyByWire's EWD/SD/ECL instruments are **already-compiled**
JavaScript bundles (there is no X-Plane-native TypeScript build step this
project can hook), so this port had to solve a much harder problem than
"add an alert": it had to (a) build a JS engine and enough of a DOM/MSFS
global-object emulation that FlyByWire's compiled bundles run at all, then
(b) splice new behaviour into *already-minified* JS as literal find/replace
text patches, matched exactly once against a specific build's line numbers.
Measured:

- **≈30,670 lines** exist to make (a) possible at all: `src/js/` (16,112:
  `dom/` + `msfs/` subtrees — a from-scratch DOM and MSFS-global-object
  shim), `src/display/` (5,400), `src/navdata/` (4,735), `src/wxr/`
  (1,063), `src/oans/` (768), plus `src/js_bridge.rs` (1,391),
  `src/js_worker.rs` (469), `src/ecam_patches.rs` (353) and
  `src/ecam_patches/ecl.rs` (379) — **17.3% of the entire 176,880-line
  crate**, none of which has any reason to exist once the host runs
  FlyByWire's TypeScript natively.
- **33** literal `SourcePatch { .. }` construction sites exist across the
  crate to do (b): 20 in `ecam_patches.rs`, 5 in `deep/ecam/patches.rs`, 4 in
  `ecam_patches/ecl.rs`, and one each defining/assembling the mechanism in
  `js/msfs/mod.rs`/`js_bridge.rs`, plus one each in `oans/plugin.rs` and
  `wxr/mod.rs`. Every one of them is anchored to an exact, byte-verified
  string from *one specific FlyByWire development build*
  (`docs/deep/ecam_bridge.md` §4 documents the verification command,
  `rg -F '<find text>' <file>`, run against
  `D:\fbw-aircraft\fbw-a380x\out\...\html_ui`) and breaks the moment
  FlyByWire's build changes that line — a standing maintenance liability
  this project already carries and documents (`ecam_bridge.md` §7).

`src/deep/ecam/` (997 lines: `codegen.rs`, `ids.rs`, `cond_json.rs`,
`patches.rs`, `deep_ecam_bridge.js`, `tests.rs`) is the part of this
mechanism that is genuinely reusable, because it is not X-Plane-specific at
all — it is a **declarative-to-JS compiler**: it takes `deep::api::
EcamAlert` (already platform-free, §2) and emits (1) static
title/procedure-text JS objects, (2) a small tagged-array encoding of
`Cond` (`['var', name, unit, cmp, value]`, never JS source text — nothing
here ever calls `eval`), and (3) `deep_ecam_bridge.js`, one hand-written,
static file that installs alerts into FlyByWire's `FwsCore` at runtime by
building `EwdAbnormalItem` objects with a small hand-rolled
`{get,set,sub}` stand-in for a real `Subject` (necessary on X-Plane only
because a real `@microsoft/msfs-sdk` `Subject` class is a private, non-exported
binding inside FlyByWire's own compiled bundle — unreachable from a
separately loaded script).

**In MSFS, none of that last constraint holds.** FlyByWire's TypeScript
builds normally there, from real, editable `.ts` source, with real imports
and a real `@microsoft/msfs-sdk` dependency available to *any* module in the
same build — because this deep-systems module's own TypeScript, if it has
any, is compiled by the same TypeScript toolchain into the same kind of
output FlyByWire's own `EcamMessages`/`FwsAbnormalSensed.ts` are. Concretely,
what changes:

1. `deep::ecam::codegen`'s static-data emission (`EcamAbnormalSensedProcedures[id] = {...}`)
   becomes a normal TypeScript module exporting a `Record<number,
   AbnormalProcedure>`, merged into FlyByWire's own via a real `import` and
   `Object.assign` (or, cleaner, a PR-style extension point if FlyByWire's
   own source is patched in the `D:\fbw-aircraft`-is-read-only sense —
   see the constraint note below) — no find/replace against a specific
   line number.
2. `deep_ecam_bridge.js`'s fake `Subscribable` becomes unnecessary: a real
   `Subject`/`MappedSubject` from `@microsoft/msfs-sdk` can be constructed
   directly, because the new code is a first-class module in the same
   compiled bundle, not a splice into someone else's — removing the entire
   "why a fake Subscribable is correct" argument (`ecam_bridge.md` §3) along
   with the object it justified.
3. The 33 `SourcePatch` sites collapse to however many real source-level
   extension points FlyByWire's TypeScript already has (or a small, stable
   diff against `D:\fbw-aircraft`, applied as a **patch file** per this
   project's existing constraint that the reference tree is read-only —
   the same shape this crate already uses for `docs/deep/integration.md`'s
   "exact patches, none applied" sections, just targeting `.ts` instead of
   compiled `.js`). A patch against readable TypeScript source is something
   a human can review and a merge tool can 3-way-merge across FlyByWire
   version bumps; a byte-exact string match against a specific minified
   build's line 166034 cannot survive FlyByWire shipping a new version at
   all, and this project's own docs already flag that fragility.
4. `deep::ecam::ids::assign`'s 10-digit id space (`1_000_000_000 +
   sorted_index`) and the `EcamAlert`/`Cond`/`ProcLine` data model in
   `deep/api.rs` need no change at all — they are the part of this bridge
   that was never X-Plane-specific, and they are what a TypeScript-side
   consumer would read instead of a JS array literal.

Net effect: the ECAM/ECL bridge goes from "17.3% of the crate re-implementing
a browser to run someone else's compiled output, plus a line-number-fragile
patcher" to "a TypeScript module compiled by the same toolchain as the
aircraft it extends." That is a simplification large enough to change which
stage of this plan is riskiest (§9) — in the current X-Plane build, the JS
bridge is a standing maintenance cost; in MSFS, it mostly stops needing to
exist.

## 6. What genuinely cannot come across

- **RESOLVED (post-plan): persistence has a ready-made MSFS mechanism, and
  a third of these lines are not persistence at all.** Split the 1,054 three
  ways before treating any of it as a gap:

  1. `src/mel_catalog.rs` (284 lines) is *authored static data*, not state.
     It never changes at runtime. Compile it into the binary as a constant
     and the filesystem question does not arise for it.
  2. `src/persistence.rs` + `src/wear.rs` (557 lines) are genuinely mutable
     and must survive shutdown — but `AirframeState` is a single
     `serde::Serialize`/`Deserialize` struct written as one whole JSON
     document. That is a key-value shape already.
  3. `src/state_dump.rs` (213 lines) is a developer diagnostic, not a
     product requirement; it can be dropped or routed to the log.

  FlyByWire already ships the store for (2): `fbw-common/src/systems/shared/src/persistence.ts`
  (`NXDataStore`) is a persistent key-value store over MSFS's `GetStoredData`
  / `SetStoredData` globals, keyed per aircraft project, and it already
  handles the FS2020-returns-`''` / FS2024-returns-`null` difference. Their
  aircraft persists its own settings through it today.

  The WASM module cannot call those globals itself — they are JS. The flow is
  therefore: the TypeScript side reads the key at boot and pushes the blob to
  WASM over a SimConnect client-data channel (the same transport FlyByWire's
  failure list already uses, see the RESOLVED note in §8), and on save the
  WASM side pushes the serialised state back out for `SetStoredData`. In MSFS
  that TypeScript is built normally, so adding it is ordinary work rather
  than bundle patching.

  Two things still to measure rather than assume: whether `SetStoredData`
  copes with a blob the size of per-component wear across ~2,000 components
  (it is a string KV store, not a database), and what it costs to write. If
  the blob is too large the answer is to persist *aggregates* — per-component
  wear is derivable from hours and cycles — not to add a filesystem.

  A read-only `persistence.ini` alongside the aircraft would not solve this
  on its own: reading is the easy half, and WASM has no filesystem to read it
  with. Wear only means anything if it is written back.

- **File-based persistence** (`src/persistence.rs` 325, `src/wear.rs` 232,
  `src/mel_catalog.rs` 284, `src/state_dump.rs` 213 — 1,054 lines total) uses
  real `std::fs::{read_to_string, write, rename, create_dir_all}` calls
  against `Output/preferences/fbw_a380x_airframe.json`-style paths. Nothing
  in `msfs-rs` exposes a filesystem API, and the only file I/O found
  anywhere in FlyByWire's own MSFS wasm crates is inside a `#[cfg(test)]`
  developer tool never compiled into the shipped binary (§1). This is a
  named, real gap: component wear, persisted failures and airframe hours
  either need a SimConnect-based persistence channel (if one exists — not
  confirmed in `msfs-rs`, would need direct testing against a build) or a
  companion process outside the WASM sandbox, the same architectural move
  this project's own `remote::Systems`/out-of-process systems host already
  makes for a different reason on the X-Plane side.
- **Structured cloud/precipitation-type weather** (`XPLMGetWeatherAtLocation`'s
  cloud layers, coverage, base/top — feeding `deep::environment`'s icing,
  lightning, hail and turbulence models). No `msfs-rs`-exposed equivalent
  was found; MSFS's live-weather surface (through SimConnect weather
  stations/METAR injection) is a different, coarser API that would need its
  own adapter, written and tested against a real MSFS session — out of
  scope to design blind here.
- **Sun elevation for `fire_ice`'s solar-flux term** — already a documented
  no-op on X-Plane (`deep::live`'s own doc: "`fire_ice`'s live system
  passes 0 today rather than inventing a flux"), so this is not a
  regression MSFS introduces, just a gap neither host currently closes.
- **This crate's entire JS/DOM/MSFS-emulation layer** (§5, ≈30,670 lines) —
  not "cannot come across" in the sense of a blocker, but in the sense that
  porting it would be actively wrong: it exists to solve a problem MSFS does
  not have.
- **`D:\fbw-aircraft` is read-only reference.** Every change this plan needs
  inside FlyByWire's own TypeScript or Rust — a new abnormal-procedure
  export, a failure-bus hookup, an LVar name — has to live as a **patch
  file** against that tree (the same discipline `docs/deep/integration.md`
  already uses: "Exact patches (none applied; for the lead)"), applied by
  whoever controls the actual FlyByWire build, or maintained as a fork.
  This is a process constraint on every stage below, not a one-time cost.

## 7. Staged plan

Each stage below produces something that runs and can be checked without
needing the next stage to exist.

### Stage 0 — Toolchain and skeleton (no physics yet)

Stand up a fifth `cdylib` crate, `a380_deep_wasm` (name arbitrary), inside
a fork of `D:\fbw-aircraft`'s workspace (or a sibling workspace patched to
find it — a build-time decision, not a design one), depending on nothing
but `msfs` and `std`. It declares one `NamedVariable` in each direction, and
`panel.cfg` gains one more `htmlgauge0N=WasmInstrument/WasmInstrument.html?
wasm_module=deep.wasm&wasm_gauge=Deep` line (a patch file against the
read-only tree, per §6).

**It works** when: the aircraft loads in MSFS with the new module present,
`sim/msfs` console output shows the gauge's own start-up log line (the
`println!`+`report_diagnostic` pattern `a380_systems_wasm/src/lib.rs:51-54`
already uses), and toggling the test `NamedVariable` from the Dev console
changes what the module reads back next frame.

Effort: a Docker/MSFS-SDK toolchain bring-up (the licensed SDK download,
the pinned dev-env image, `wasm32-wasip1` target) is the dominant cost here,
not code — call it 2-3 days for someone doing this build for the first
time, most of it environment setup already fully specified by FlyByWire's
own `rust-toolchain.toml`/`.cargo/config.toml`/`scripts/dev-env/run.sh`.

### Stage 1 — Port the platform-free 92% verbatim

Copy `src/deep/` and `src/physics/` (minus the 11 files named in §2) into
the new crate unchanged — no dependency on `crate::Vars` or X-Plane means no
dependency on anything MSFS-specific either, by the same rule
(`docs/deep/BRIEF.md` hard rule 2). This is a copy, not a rewrite: 92.3% of
100,913 lines (≈93,200 lines) by the measurement in §2.

**It works** when: `cargo test` on the new crate (host target, not wasm —
these are plain unit tests) passes with the same pass count this repo's own
`#[test]` count already establishes as a floor: 1,666 tests under `src/deep`
alone, 2,479 crate-wide. A test failing here means the copy broke something
platform-independent, which should never happen from a straight copy.

Effort: mechanical — a day or two of import-path fixups (`crate::deep::`
paths, `mod.rs` re-declarations per `docs/deep/integration.md`'s own "Patch
A" pattern), not physics work.

### Stage 2 — The platform boundary: `Truth` from MSFS

Write the MSFS equivalent of `deep::plugin.rs`'s `truth()` function: resolve
every `Truth`/`Controls` field to an `AircraftVariable`/`NamedVariable`
handle once at start-up (mirroring `DeepLayer::new`'s own
resolve-once-not-per-frame discipline, `plugin.rs:487-538`), read them every
`MSFSEvent::PreDraw`, call `Deep::tick`. Use §3's table directly: most
fields are a rename of an already-known LVar or MSFS simvar; the short list
of genuine gaps (cloud/weather structure, sun elevation, two raw control
inputs) gets its named fallback from §3/§6, not an invented number.

**It works** when: with the engines running and systems live in MSFS, the
new module's own diagnostic output (a `NamedVariable` dump, the same shape
`Snapshot`/`state_dump.rs` already produce for X-Plane) shows physically
sane values — engine N1/N2/N3 tracking the throttle, bus voltages matching
FlyByWire's own ELEC page, tyre pressures near their cold-inflation figure
on the ground — with no NaN, matching the existing
`every_assembled_area_has_its_own_name_and_can_be_stepped_cold` test's own
bar, now checked against real MSFS data instead of `Truth::default()`.

Effort: the dominant cost is testing against a live MSFS session (each
iteration is a sim restart), not writing the resolver — the resolver itself
is a mechanical rewrite of `plugin.rs`'s existing 1,060 lines against a
different variable-access API with the same shape. Call it a week of
iteration once Stage 1 tests pass, most of it flight-testing rather than
coding.

### Stage 3 — Output: publish, and drive FlyByWire's failures

Wire `Deep::publish` to `NamedVariable::set_value` (§4) and
`Deep::derived_magnitudes()` to whatever failure-injection call
`systems_wasm::MsfsSimulationBuilder` exposes for the `FailureType` ids it
already registers (`.with_failures([...])`, confirmed real in §1/§4) — this
needs one focused investigation into `systems_wasm`'s own runtime API (not
yet read in this pass; `lib.rs`'s construction-time `.with_failures(...)`
was read, its *runtime setter* was not) before this stage can be scoped
precisely.

**It works** when: arming a deep electrical failure (e.g. a VFG's own I²t
overload tripping, per `authority.md`'s existing 25-coupling table) makes
FlyByWire's own ELEC page in MSFS show that generator failed, the same
end-to-end proof this project's own
`a_derived_bus_failure_changes_flybywires_own_solve`-style tests already
demonstrate on X-Plane — reproduced here as a manual MSFS check first, then
as an automated one once a WASM test harness exists (none is assumed here).

Effort: a few days once the `systems_wasm` failure-bus API is read in
detail — this is the one stage whose scope is currently bounded by "we
haven't read the one file that answers it yet," named honestly rather than
estimated around.

### Stage 4 — ECAM/ECL bridge, rebuilt for a normal TypeScript build

Per §5: reimplement `deep::ecam::codegen`'s output as real `.ts` modules
(same `Cond`/`EcamAlert` data model, different target language shape — the
1,000+ lines of `deep::ecam`'s Rust-side logic that build the *data*, not
the JS-splicing plumbing, carry over almost unchanged), and drop the
`SourcePatch`/find-replace mechanism entirely in favour of patch files
against `D:\fbw-aircraft`'s real `.ts` sources (or a proper extension point,
if FlyByWire's maintainers would take one upstream — outside this plan's
control to assume).

**It works** when: arming the same three worked-example alerts already in
`src/deep/ecam/tests.rs` (`ENG_2_OIL_LO_PR`, `GREEN_RSVR_LOW`,
`ENG_3_FUEL_FILTER_CLOG`) makes them appear on the real EWD in MSFS with
correct level, aural, procedure lines that tick complete when the named
cockpit control moves, and correct STATUS/INOP SYS entries — the exact
worked example `ecam_bridge.md` §6 already specifies, now checked against a
real compiled TypeScript build instead of a spliced JS bundle.

Effort: smaller than the X-Plane original despite doing the same job,
because there is no browser/DOM emulation to build first (§5) — most of the
cost is learning FlyByWire's real TypeScript build and patch-review process,
not writing new logic.

### Stage 5 — Scale up: all 17 areas, all couplings, persistence

Repeat Stage 2's per-field verification across every area (not just
electrical/hydraulics/pneumatics), wire all 25+16+8 = 49 `authority.md`
couplings (§4), and solve or explicitly scope out the persistence gap (§6).

**It works** when: a full flight — cold and dark to shutdown — with several
failures armed produces the same ECAM behaviour, component wear and (where
persistence exists) saved airframe state a comparable X-Plane session
produces, checked by the same kind of scenario tests
`src/offline_harness.rs`/`scenarios.rs` already run on this side, ported to
run against the new crate's own test target.

## 7b. RESOLVED (post-plan): the out-of-process route, and what it dissolves

This plan assumes the deep layer lives inside the MSFS WASM sandbox, and
several of its hardest constraints follow from that assumption alone — no
filesystem, no sockets, no threads, a shared per-frame budget with four
other wasm modules. **This project already has the other architecture**, and
the plan should be read with it in mind.

`src/remote/` (940 lines: `mod.rs` 170, `client.rs` 266, `server.rs` 216,
`win.rs` 150, `wire.rs` 138) runs FlyByWire's `Simulation<A380>` in a
separate process, `fbw_a380_systems_server.exe`. The plugin keeps the sim's
side; the server ticks the systems; they share one block of memory holding
every registered variable and hand it back and forth **once a frame in
lockstep**, so the systems tick on the same frame's inputs and their outputs
reach the same frame's later steps — no frame of lag on the controls. A
crash in the systems stops the systems, not the simulator. `Systems::Local`
versus `Systems::Remote` already selects between them at runtime, with
`FBW_SYSTEMS_IN_PROCESS=1` as the fallback.

If the deep layer runs out-of-process under MSFS, then:

- **Persistence stops being a question.** A normal process has a filesystem.
  SQLite, or any real database, is available; the `NXDataStore` key-value
  route in §6 becomes a fallback rather than the plan.
- **The browser Study app carries over unchanged.** `src/study/web.rs`
  (981 lines, the HTTP server and its JSON) and `app/ui/index.html` have
  **zero** `crate::xp` imports — verified by grep. They need a socket, which
  WASM does not have and an ordinary process does.
- **The per-frame WASM budget risk in §8 largely evaporates**, because the
  expensive work is no longer inside the sandbox.

What it costs, and what is genuinely unresolved:

- **The transport is the open question, not the idea.** The X-Plane version
  uses Windows shared memory with a synchronous once-a-frame handoff. Under
  MSFS the in-sim side would be a thin WASM shim relaying over SimConnect
  client data, which is queued and asynchronous. Whether true lockstep
  survives that — or whether the port accepts one frame of lag — has to be
  measured, and it matters most for flight controls.
- **LVars are not reachable from an external SimConnect client directly.**
  This is a well-known MSFS limitation, and it is solved by construction
  here: the thin in-sim WASM shim exists anyway and can relay them.
- `win.rs` is Windows-specific. MSFS is a Windows target, so this costs
  nothing.

**What does not carry over** is the in-sim drawn Study window. The other
eight files in `src/study/` each import exactly one thing from `crate::xp`,
and in every case it is X-Plane's native drawing and windowing API
(`FONT_BASIC`, `FONT_PROPORTIONAL`, `WindowId`, `MenuId`, `MOUSE_DOWN`).
The drawing calls are X-Plane's and have no MSFS equivalent; the *page
logic* underneath them is reusable, rendered either in the browser app or as
a flyPad EFB page.

## 8. Risks — specific, not generic

- **RESOLVED (post-plan, by the session that commissioned this doc): the
  runtime failure API exists, but it is binary.** `systems_wasm/src/failures.rs`
  and `lib.rs` were read directly. `.with_failures([...])` registers the
  `u64 -> FailureType` map at construction; at runtime a SimConnect
  client-data callback calls `Failures::handle_failure_update(data)` with a
  JSON array of ids, and every frame `MsfsHandler::read_failures_into_simulation`
  takes any pending update and calls `simulation.update_active_failures(set)`.
  So per-frame runtime updates are already plumbed and Stage 3 does **not**
  need the redesign this bullet originally feared.

  The real constraint is a different one and is now the thing to design
  against: that channel carries an `FxHashSet<FailureType>` — a *set of
  active ids*, with **no magnitude**. This project's `Faults` is `id -> 0..1`
  throughout. Two consequences:

  - Our own failures are unaffected. `Faults` lives inside our crate and
    our areas read it directly; it never needs to cross FlyByWire's channel.
  - The ~49 authority couplings in `docs/deep/authority.md`, where we drive
    *FlyByWire's own* failure at their resolution, do lose their continuous
    magnitude — they would threshold to on/off. For couplings that are
    genuinely machine-level verdicts that is faithful; for a partially
    degraded pump or a partially blocked duct it is a real loss of fidelity,
    and each of the 49 should be classified before Stage 3 rather than
    thresholded wholesale.

  If continuous magnitudes are wanted, the route is a second SimConnect
  client-data channel of our own carrying `(id, magnitude)` pairs — the same
  mechanism FlyByWire already uses, not a new one. That is now an optional
  fidelity upgrade rather than a required rescue.
- **The MSFS WASM sandbox's real behaviour under sustained per-frame work
  from a *second* module has not been measured.** `plugin.rs`'s own "Frame
  cost" section (§ read in full) measures this project's per-frame cost
  against X-Plane's flight loop precisely (2.4 ns/value on the fast path,
  ~180-270 reads a frame); no equivalent measurement exists for MSFS's
  `PreDraw` budget with *four* wasm modules already resident
  (`systems.wasm`, `fbw.wasm`, `fadec-a380x.wasm`, `extra-backend-a380x.wasm`)
  plus a fifth. MSFS's own wasm host has historically been stricter about
  per-frame CPU budget than X-Plane's plugin loop; a naive port of 17
  areas' worth of physics could miss frame budget in a way that only shows
  up under real MSFS load, not in a host-side `cargo test`.
- **File persistence has no confirmed MSFS path** (§6) — if none exists,
  every persistent feature this project has (component wear, MEL, saved
  failures, airframe hours — `wear.rs`, `mel_catalog.rs`, `persistence.rs`,
  1,054 lines' worth of design) needs an architectural answer before Stage 5,
  not a workaround bolted on afterward.
- **`D:\fbw-aircraft` being read-only reference is not just a repo-hygiene
  rule here — it is a distribution problem.** Every stage from 2 onward
  needs *some* patch applied to FlyByWire's real source (an LVar FlyByWire
  doesn't yet publish, a failure-bus hookup, a TypeScript extension point in
  Stage 4) to actually load in a real installation. This plan can specify
  every patch precisely (the same "exact patches, none applied" discipline
  `docs/deep/integration.md` already uses), but it cannot make them apply
  themselves — whoever owns the actual shipped aircraft build has to accept
  or fork them, which is a project-management dependency this plan cannot
  discharge by itself.
- **The `SourcePatch`-to-TypeScript-patch transition in Stage 4 assumes
  FlyByWire's real `.ts` source is structured the way `ecam_bridge.md`'s
  study of the *compiled* output implies** (`EcamAbnormalSensedProcedures`,
  `FwsAbnormalSensed.ts`'s `ewdAbnormalSensed` dict, etc.) — that study was
  done against compiled JS reverse-engineered back to its documented
  TypeScript shape (`ecam_bridge.md` §1 cites exact `.ts` files and line
  numbers, so this is likely correct), but it was not confirmed by reading
  the live `.ts` source itself in this pass, only by this project's own
  prior documentation of it.

## Appendix — the counts this plan is built on

| Quantity | Count | Method |
|---|---|---|
| Total crate size | 176,880 lines | `find src -name '*.rs' \| xargs wc -l` |
| `src/deep/` | 88,556 lines, 221 files, 19 area subdirectories | same |
| `src/physics/` | 12,357 lines, 28 files | same |
| `src/breakers.rs` | 1,829 lines | same |
| Files under `deep/`+`physics/` with a real X-Plane import | 11 of 249 (4.4%), 7,728 of 100,913 lines (7.7%) | `grep -rl 'use crate::xp'`, hand-checked against a raw `XPLM`-keyword grep to exclude doc comments |
| `#[test]` functions | 1,666 under `src/deep`, 2,479 crate-wide | `grep -ro '#\[test\]'` |
| `registry.rs` files (one per area) | 18 | `find src/deep -name registry.rs` |
| Electrical area's own registered catalogue | >200 components, >400 failures (code-asserted lower bound, `src/deep/electrical/registry.rs:271-272`) | reading the test, not running it |
| Documented aggregate (this project's own docs) | ~1,643 electrical failures (`authority.md:8`); ~5,200 failures/~2,000 components/300 alerts crate-wide (task brief) | cited, not independently re-derived — a literal grep for `r.failure(FailureDef {`/`r.component(ComponentDef {` call sites returns only 163/119, which the electrical test above proves is an undercount: most registration goes through per-instance loops (`register_component` over a channel list, `for n in 1..=4`, etc.), invisible to a textual call-site grep |
| `EcamAlert::new(` call sites | 144 (same undercount caveat as above) | `grep -ro` |
| Values published per frame | "2445 values ten areas publish" (measured and quoted in `plugin.rs:417-421`); 17 areas assembled in `all_areas()` today, so the true current figure is higher | reading the source's own measured comment |
| `SourcePatch { .. }` construction sites | 33 crate-wide | `grep -ro 'SourcePatch {'` |
| JS/DOM/MSFS-emulation-only code (§5) | ≈30,670 lines (17.3% of the crate) | `wc -l` over `src/js/`, `src/display/`, `src/navdata/`, `src/wxr/`, `src/oans/`, `src/js_bridge.rs`, `src/js_worker.rs`, `src/ecam_patches.rs`, `src/ecam_patches/ecl.rs` |
| `authority.md` FlyByWire-failure couplings | 25 electrical + 16 hydraulic + 8 pneumatic = 49 | reading `authority.md`'s own tables |
| `Truth` leaf fields | ~40 top-level + ~30 `Controls` fields | reading `live.rs`'s struct definitions |
| FlyByWire's MSFS systems build | confirmed real, `wasm32-wasip1`, Rust 1.96.0, `msfs-rs` git dependency, 4-module `panel.cfg` load | direct reads of `D:\fbw-aircraft`'s `Cargo.toml`/`rust-toolchain.toml`/`.cargo/config.toml`/`panel.cfg` plus a scratch clone of `msfs-rs` |

## What actually stands between us and running the deep layer in MSFS

Measured 2026-09-21 against this copy, not estimated.

The WASM module needs a core that compiles for `wasm32-wasip1`, which
means a core with no X-Plane in it. The question is how much of the tree
that is. Counting the outward references:

| module | what it reaches for outside itself |
| --- | --- |
| `deep` | 291 `crate::deep`, then `physics` 35, `failures` 26, **`xp` 20**, `aspects` 6, and single figures for eight more |
| `physics` | `failures` 55, **`xp` 30**, `fadec` 13, `invariants` 10 |
| `failures` | `components` 4, **`xp` 3**, `mel` 3, `aspects` 2, `study`, `remote` |
| `aspects` | nothing |

**Correction (2026-09-21).** The sentence that stood here -- "all twenty
references are the weather API and nothing else" -- was wrong, and the
claim was mine. Two of the twenty are not weather:

- `deep/integration/xp_consequences.rs` writes the plug-force datarefs
  (`sim/flightmodel/forces/*_plug_acf`) and a gear leg's `deploy_ratio`.
  That is how ice drag, bird-strike drag and a collapsed leg act on the
  airframe. It has no weather content to abstract.
- `deep/plugin.rs` builds essentially the whole `Truth` struct from about
  180 dataref reads. Weather is one delegated piece of it, not the reason
  for the import.

A weather trait alone therefore takes `deep` from 20 references to 2, not
to 0. `integration/mod.rs`'s own module doc also asserted that depending
on `crate::xp` broadly was Integration's deliberate purpose -- true while
X-Plane was the only host, false as soon as there are two.

### The boundary, drawn (step 2)

Counting what the rest of the prospective core actually touches:

| module | `crate::xp` refs | files | lines |
| --- | --- | --- | --- |
| `physics` | 30 | 4 | 12,384 |
| `failures.rs` | 3 | 1 | 1,969 |
| `fadec.rs` | 3 (one is a test's `Xplm::dummy()`) | 1 | 1,571 |
| `invariants.rs` | 0 | -- | 466 |
| `components.rs` | 0 | -- | 418 |
| `mel.rs` | 0 | -- | 411 |
| `aspects.rs` | 0 | -- | 962 |

The thirty-three references that made this look like a wide boundary are
concentrated in **four files**, and reading them shows they are not
thirty-three different problems. Every one falls into one of three
shapes:

1. **Host inputs.** A struct of `Option<DataRef>` fields, resolved once
   and read every frame. `physics/adirs.rs` is 23 of the 30 on its own --
   attitude, body rates, accelerations, position, static pressure, SAT,
   Mach, alpha, weight-on-wheels. `deep/plugin.rs` and `failures.rs` are
   the same shape.
2. **Host effects.** Writing back: forces, dataref sets, a retracting
   gear leg. `physics/xp_effects.rs`, `physics/damage.rs`,
   `physics/tyre.rs` and `deep/integration/xp_consequences.rs`, 33 write
   call sites between them.
3. **Host queries.** Two, both point lookups: `probe_terrain_y` and the
   weather API.

So the decision is: **the core is everything except `src/xp`**, and it
reaches the host through three narrow traits -- inputs, effects, queries
-- rather than through a module boundary drawn between systems. Nothing
in the list above belongs to X-Plane; all of it is aircraft behaviour
that happens to have been written against the only host there was.

`fadec.rs` needs no trait at all: its only non-test reference is the
import, and the live one is a test constructing `Xplm::dummy()`. It moves
for free alongside `invariants`, `components`, `mel` and `aspects`.

### Measured (2026-09-21): the crate already checks for wasm

Rather than gate modules by reading, the plugin crate was type-checked
for `wasm32-wasip1` inside FlyByWire's dev-env image, with `cc` pointed
at the SDK's clang and sysroot for the C++ computers:

    cargo check --target wasm32-wasip1 --lib      -> 2 errors

Both in `src/remote/client.rs`: a `std::os::windows` import and a
`creation_flags` call. FlyByWire's C++ computers (FADEC, PRIMs, SECs,
FCUs) compiled through clang without complaint, as they do for their own
modules. Everything else -- `deep`, `physics`, `study`, `display`'s host
parts, `mapdata`, the lot -- checks. Gated the flag behind `cfg(windows)`
and the count is zero.

So the boundary the sections above draw so carefully is, at the level of
*compiling*, two lines. What remains is the level of *instantiating*: the
`#[link(name = "kernel32")]` blocks (`xp.rs`, `big_stack.rs`,
`sensors.rs`, `remote/win.rs`, `xphfbw_bridge.rs`) become undefined
symbols, which `--allow-undefined` turns into imports MSFS's runtime does
not provide. The module would link and then fail to load. Listing those
imports from the built module is the next measurement, and each one is
either gated for wasm or given a host implementation.

`msfs/deep-wasm` is the fifth module. Stage 0 -- load, resolve the ten
inputs, publish through the bridge -- built (107 KB) and ran as
`htmlgauge04`. **Stage 1 has linked**: the same crate with the plugin
crate linked whole, `deep::live::all_areas()` instantiated at
PreInitialize, 6.8 MB after wasm-opt. Beyond the two process flags it took
one overflow lint (a refcon shifted by 32 on a 32-bit usize) and six
`kernel32` extern blocks -- two of them bare, with no `#[link]`, relying on
Windows' default link set, which is why they only showed up as imports in
the built module.

The built module's import section is the boundary, measured rather than
drawn:

| module | imports | provided? |
| --- | --- | --- |
| `env` | 5: `aircraft_varget`, `get_aircraft_var_enum`, `get_units_enum`, `register_named_variable`, `set_named_variable_value` | yes -- MSFS's gauge API, all five also imported by FlyByWire's modules |
| `wasi_snapshot_preview1` | 22 | 18 are imported by `systems.wasm`, `fbw.wasm` or `extra-backend-a380x.wasm` and so provably provided (including the filesystem calls -- `fbw.wasm` imports `path_open`, `fd_readdir`, `path_remove_directory`). **Four are not proven by anything that ships**: `fd_filestat_get`, `fd_sync`, `path_rename`, `poll_oneoff`. |

An import the runtime does not provide fails instantiation outright, so
those four are the one remaining risk between the linked module and a
running one. They come from host-only code -- an atomic file save, a
`sync_all`, a `thread::sleep` -- not from the systems, and the honest
fix if the runtime rejects one is to gate that code for wasm, not to
supply a stand-in. The console names the missing import if it happens.

Two things about the image that cost time and are now in `build.ps1`:
its entrypoint `cd`s to the mount root regardless of `-w`, so the script
must `cd` into the crate itself or cargo builds the plugin at the
repository root; and the crate's `../fbw-aircraft` path dependency means
the aircraft checkout is mounted at `/fbw-aircraft`.

### Order of work

1. ~~The weather trait in `deep`, plus a force-injection trait for
   `xp_consequences.rs`~~ -- done. `WeatherSource` and `ForceSink`,
   each with a live X-Plane implementation and an offline one.
   `deep`'s real `crate::xp` imports went from 8 files to 2: 18 of the
   original 20 references are gone, and the 2 left are both
   `DataRef`/`Xplm` for general Truth-building, not weather --
   `plugin.rs` (the documented seam) and `integration/weather_truth.rs`,
   which turned out to import them for a second reason nobody had
   noticed: three raw atmosphere datarefs (`leading_edge_c`,
   `ambient_pressure_pa`, `precipitation_on_aircraft_ratio`) that the
   icing and lightning models read directly. That is the same shape of
   problem as `plugin.rs`, just smaller, and it belongs to the inputs
   trait in step 3.
2. ~~Draw the core boundary~~ -- done, above.
3. The inputs trait, whose first and largest implementation is
   `physics/adirs.rs`'s 23 fields, then `deep/plugin.rs`'s ~180. This is
   the bulk of the remaining work and it is mechanical, not a judgement
   call, now that the shape is known.
4. Move the core into its own crate, both simulators depending on it.
5. Only then the gauge: `deep::lvar_bridge::VarStore` over msfs-rs's
   `NamedVariable`, `deep::msfs_inputs::INPUTS` for the ten host values,
   published every frame. The measurements in `msfs/lvar-bench` say that
   part is affordable -- 4,228 variables in 0.048 ms.
