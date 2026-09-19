# Flight controls (ATA 27) — audit notes

Scope: how the port drives X-Plane's control surfaces from FlyByWire's real
A380 fly-by-wire simulation, and what's still a gap. Written after a ~2h
audit; no source changes were made this pass (see "Found, not yet fixed"
below) because a hard deadline landed mid-investigation.

## What's already authentic (not reimplemented — the real FBW code, compiled/linked in)

- **Control laws and computers**: `src/fbw_computers.rs` links
  `fbw_controllers` (built from FlyByWire's own C++/Simulink-generated
  `fbw_a380/src/prim`, `sec`, `fcu` — see `CMakeLists.txt`,
  `src/fbw_cpp`) via FFI (`fbw_prim_create/set_inputs/update/...`). PRIM 1-3,
  SEC 1-3 and FCU 1-2 are FlyByWire's unmodified compiled control laws:
  normal/alternate/direct reversion, flight envelope protections, flight
  guidance, all come from the real model, not a Rust re-derivation.
- **Actuator physics**: the plugin depends directly on FlyByWire's own
  `a380_systems` crate (`Cargo.toml`: `a380_systems = { path =
  "../fbw-aircraft/fbw-a380x/src/wasm/systems/a380_systems" }`) and
  `systems` (`fbw-common`). `a380_systems::A380` (instantiated once via
  `Simulation::new(A380::new, ...)`, `src/lib.rs:1604`) is the same
  aircraft-system simulation FlyByWire ships in MSFS: real per-actuator
  hydraulic/EHA/EBHA modelling (servo solenoids, electric-mode/hydraulic-mode
  solenoids, commanded-position LVars — `a380_systems/src/hydraulic/mod.rs`),
  each on its documented AC/hydraulic source (e.g. `AC_EHA_BUS` =
  `AlternatingCurrentNamed("247XP")`, spoiler 6 EBHA on
  `AlternatingCurrentEssential`, rudder EBHAs split green/yellow hydraulic +
  AC electric per panel). Losing a source, blowdown, damping mode and rate
  limiting are therefore FlyByWire's own logic, inherited whole.
- **The closed loop**: `prim.rs::update_prim` reads each surface's *actual*
  hydraulic actuator position back from the systems simulation (e.g.
  `A32NX_HYD_AILERON_LEFT_INWARD_DEFLECTION`, mod.rs:993-1038) as PRIM
  position feedback — the same architecture as the real aircraft/MSFS build,
  not a shortcut. `flight_controls.rs` then takes the actuators' final
  written positions and spreads FlyByWire's panel geometry onto X-Plane's
  differently-cut surfaces by span-weighted mean (documented in its module
  doc and covered by 9 unit tests: neutral, travel ends, sign inversion,
  aileron blending, elevator/rudder span averaging, spoiler grouping,
  spreading conserves area, stabiliser trim ratio, axis sign).
- **Power gating**: PRIM/SEC/FCU `is_powered` comes from real electrical bus
  LVars (`A32NX_ELEC_108PH_BUS_IS_POWERED`, `A32NX_ELEC_247PP_BUS_IS_POWERED`,
  `A32NX_ELEC_DC_1_BUS_IS_POWERED`, etc. — `prim.rs:817-818, 1199-1200,
  ~1617`), not hardcoded true; `fault_active` comes from
  `random_failures`/`FAILURE_PRIM`/`FAILURE_SEC`/`FAILURE_FCU`
  (`FailuresConsumer` ids, `prim.rs:51-59, 596-609`). Reversion (normal →
  alternate → direct) is therefore a real consequence of losing power/ADIRU/
  computers, per FlyByWire's own DSC-27 logic — not an if/state shortcut
  layered on top.
- **Ground spoilers, rudder trim, pitch trim switches**: wired as one-frame
  pulses through `TrimPulses`/`key_events.rs` into the PRIM/SEC inputs the
  same tick (`prim.rs` module doc, `UNAVAILABLE` list at the top documents
  exactly what upstream FBW itself hard-codes vs. what's genuinely unported).

## Found, not yet fixed: sidestick priority takeover is unwired

`A32NX_PRIORITY_TAKEOVER:1` / `:2` (capt/FO priority pushbutton, held while
pressed) are read by both PRIM and SEC discrete inputs
(`prim.rs:1067-1068, 1559-1560`, matching FlyByWireInterface.cpp:1562-1563,
2108-2109) and feed the real dual-input-resolution logic inside the compiled
PRIM control law. In MSFS this LVar is written by a 3D cockpit click-spot on
each sidestick (`A32NX_Interior_Misc.xml:385-388`: `1 (>L:A32NX_PRIORITY_
TAKEOVER:#ID#)` on press, `0 (>L:...)` on release) — a hold state, not a
one-frame event.

**Nothing in this port ever writes that LVar.** Grepped every `.rs` file:
only reads exist (`prim.rs`), no writer. Result: the priority takeover
pushbutton — the actual mechanism a real A380 pilot uses to lock out the
other sidestick during a dual-input conflict — cannot be triggered at all in
X-Plane. (The FCDC's priority *lights* are separately always-off in
upstream FBW itself — `extra_backend_fcdc.rs:227-228`, citing
`Fcdc.cpp:204-207` — so that part is not a port gap, just an upstream one.)

**Fix (not yet applied — ran out of time this pass):** add two held-state
X-Plane commands (e.g. `fbw/event/A32NX_PRIORITY_TAKEOVER_CAPT/_FO`),
registered like `afs_events::Commands` but handling both command-begin
(phase 0 → true) and command-end (phase 2 → false) rather than begin-only,
since `afs_events`'s existing `on_command` (`afs_events.rs:269-277`) only
takes phase 0 and models one-frame pulses — wrong shape for a held pushbutton.
Write the resulting bool into `PRIORITY_TAKEOVER:1`/`:2` (bare name; the
`A32NX_` prefix is stripped by the registry's prefixed lookup, see
`prim.rs`'s `Names::id`) every tick, before `self.prims.update(...)`
(`lib.rs:1134`). Suggested home: a small addition next to
`afs_events::Commands`, or a new tiny module, invoked from `lib.rs` right
before the PRIM/SEC update call. Should ship with a test asserting that
holding CAPT's command sets the PRIM/SEC discrete input and releasing it
clears it within one tick.

## Not reached this pass (future audit should check)

- Sidestick RVDT dual-channel failure modelling depth (each transducer's own
  failure vs. whole-stick failure) — not directly inspected; likely lives in
  `a380_systems`'s sidestick model and should be verified against FCOM
  DSC-27 rather than re-derived.
- Slats/flaps SFCC/PCU wingtip-brake asymmetry detection — not directly
  inspected this pass; `handling.rs` covers gear/brakes/flaps/steering input
  routing but the SFCC/PCU physics itself lives in `a380_systems` and should
  be spot-checked the same way hydraulics was here (confirm it's the real
  model, confirm the port feeds it real inputs).
- No dedicated `tests/` integration test proves a full causal chain end to
  end (e.g. "lose green + yellow hydraulics on one aileron panel → PRIM
  reports the loss → surface authority visibly drops in X-Plane"); the
  existing coverage is unit-level (`flight_controls.rs`'s 9 tests, which
  prove the geometry/sign conversions, not the degradation chain).
