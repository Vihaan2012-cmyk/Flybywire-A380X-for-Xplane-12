# Flight model — progress

Coupling agent: gives the X-Plane-free emulator (`emulator/src/lib.rs`) a 6-DOF
flight model, so yaw after an engine failure, gear loads, tail strikes and
runway friction can be tested without a live X-Plane process. Directory:
`emulator/src/flight_model/`. Nothing outside this directory was edited (per
the hard rules); `mod.rs` here is not yet `mod`-declared from
`emulator/src/lib.rs` — that one-line wiring is for the lead, listed below.

Sourcing: FlyByWire's own public MSFS `flight_model.cfg`
(`D:\fbw-aircraft\fbw-a380x\...\FlyByWire_A380X\common\config\flight_model.cfg`,
every constant in `geometry.rs` cites its line), cross-checked against
`src/weight_balance.rs`'s own already-verified numbers from the same file. No
`*.acf` was found under the given download folder (only textures/objects, no
aircraft file present at the time of writing) — the cfg's own contact-point/
geometry sections turned out to have everything needed (wing/tail geometry,
gear positions, engine-adjacent contact points), so that wasn't a blocker.

## Backlog

- [done] 1. 6-DOF rigid body — `rigid_body.rs` (quaternion attitude, body
  rates, RK4), `mass.rs` (current mass/CG/inertia via parallel-axis
  combination of the empty-aircraft baseline and a moment-balance-recovered
  "extra load" point mass), `geometry.rs` (empty mass/CG/inertia baseline,
  sourced). Stable at dt=0.05 (tested at the emulator's own dt in
  `mod.rs`'s tests and `rigid_body.rs`'s `stays_stable_over_many_ticks...`).
- [done] 2. Aerodynamics — `aerodynamics.rs`: strip-theory wing/htail (split
  left/right)/vtail build-up, finite-wing lift-curve slope (Raymer),
  elevator/aileron/rudder/spoiler/flap/slat, ground effect, icing/damage
  penalties. Roll damping (Cl_p), pitch damping (Cm_q) and yaw damping
  (Cn_r) all emerge from the `omega x r` rotational-velocity term at each
  strip's own arm, not separate scripted coefficients.
- [done] 3. Engines as thrust vectors — `propulsion.rs`: sums each engine's
  `net_thrust_n` (from `physics::engine` in the main plugin, not
  recomputed here) as a force at its real mount position
  (`geometry::engine_position_m`), so asymmetric thrust yaws/rolls the
  aircraft causally (tested: losing an outboard engine yaws toward it, and
  outboard loss yaws harder than inboard loss).
- [done] 4. Landing gear — `landing_gear.rs`: five legs (nose, two wing,
  two body, the real A380 arrangement), spring-damper contact, tyre
  friction (Pacejka-lite longitudinal+lateral with a friction-circle cap),
  braking, nosewheel steering (rate-limited, fault-capable), runway
  contamination (`runway_mu_factor`), per-leg structural collapse, and a
  separate rigid tailstrike contact point.
- [done] 5. Atmosphere and wind — `atmosphere.rs`: ISA (troposphere +
  isothermal stratosphere), wind with a power-law shear profile and a
  half-sine gust train.
- [done] 6. Output mapping — see below.
- [in progress, ongoing] Registered every fault this model accepts into
  the shared API (`registry.rs`, per the 2026-09-19 brief update) instead
  of the earlier CATALOGUE.md/ECAM.md pass (deleted — see below).
  `FAILURES.md` alongside this file keeps the plain-table version the
  original brief also asked for.
- [done] A trim solver — `trim.rs`: `trim_level_flight` (2-D
  Newton-Raphson on angle of attack + elevator against the vertical-force
  and pitching-moment residuals from `aerodynamics::forces`, at zero
  flight-path angle; symmetric thrust is then read directly off the
  x-force balance rather than being a third Newton unknown) and
  `trim_on_ground` (2-D Newton-Raphson on pitch + CG height against
  `landing_gear`'s own total-load and pitching-moment residuals at zero
  velocity). Both call this module's real force models, not a lookup
  table. Tests: converges to a plausible cruise alpha/thrust, the solved
  point actually zeroes the residuals, a heavier aircraft needs more
  alpha/thrust at the same speed, ground trim supports full weight with
  zero net pitching moment (and lands within a few degrees of the cfg's
  own published `static_pitch`, -0.13 deg), determinism/no-NaN.
  `mod.rs`'s glide test still uses a fixed small elevator bias rather than
  calling this solver — wiring the glide test to start *from*
  `trim_level_flight`'s answer is the natural next step, not done yet for
  lack of remaining time.
- [not started] Full per-spoiler-panel (the real A380 has ~7-8 panels per
  side) and per-slat-section detail — this pass models one lumped
  left/right pair for spoilers (with its own fault hooks) and single
  flap/slat scalars, which is enough for the coupling-level forces this
  model owns; a systems-level agent modelling the hydraulic/PCU health of
  each individual panel would plug into `AeroFaults`/`ControlInputs` at a
  finer grain later without changing this module's shape.
- [not started] Wheel spin-up/tyre rotational dynamics — braking is
  modelled as a commanded slip fraction (0..1) directly, not a wheel
  angular-velocity state; adequate for the friction/yaw coupling this
  model owns, not for ABS-cycling-level detail.

## Output mapping (for the lead to wire)

`FlightModel::step` returns `FlightModelOutputs`. None of `emulator/src/lib.rs`
or `Emulator` was edited (outside this directory); the lead's wiring is a loop
around `Emulator::tick` that also calls `FlightModel::step` and feeds the
result back in with the existing setters:

| `FlightModelOutputs` field | `Emulator` setter |
|---|---|
| `pitch_deg` | `set_pitch_deg` |
| `bank_deg` | `set_bank_deg` |
| `heading_true_deg` | `set_heading_true_deg` |
| `indicated_airspeed_kt` | `set_indicated_airspeed_kt` |
| `true_airspeed_kt` | `set_true_airspeed_kt` |
| `mach` | `set_mach` |
| `groundspeed_kt` | `set_groundspeed_kt` |
| `agl_ft` | `set_agl_ft` |
| `on_ground` | `set_on_ground` |
| (none yet) | `set_wind_ms`, `set_structural_icing_fraction`, `set_qnh_pa`/`set_oat_c` are **inputs** the lead should feed *into* `FlightModelInputs` (`wind`, `aero_faults.wing_ice_fraction`, `oat_offset_k`/`qnh_offset_pa`) each tick, not outputs from it |

Not yet exposed as an `Emulator` setter (no such dataref/method exists there
today — new ones the lead may want to add, or publish via `Published` the
way `weight_balance.rs` does for `fbw/wb/*`):
- `vertical_load_factor_g` — g-load.
- `gear_leg_load_n` (5, `geometry::GEAR_LEGS` order) / `total_weight_on_wheels_n`
  — gear loads; `Emulator`/FlyByWire already has
  `total_weight_on_wheels`/`total_weight()` (per `lib.rs`'s own doc comment)
  computed from `TOTAL WEIGHT`, but nothing populates a *per-leg* load today.
- `tailstrike` / `tailstrike_load_n` — no tailstrike dataref exists in the
  emulator today.
- `stalled` — no stall dataref exists in the emulator today.
- `altitude_ft` — the emulator has `set_pressure_altitude_ft` for the
  *environment* (an input, QNH-derived); this is the flight model's own
  integrated MSL altitude, closer to what a real `PLANE ALTITUDE` sim
  variable would carry.

Inputs `FlightModelInputs` needs each tick, and where they should come from:
- `mass_kg`/`cg_x_forward_m`: `WeightBalance`'s own published
  `"fbw/wb/gross_weight_kg"` / `"fbw/wb/cg_z_ft" * geometry::FT_TO_M`
  (`src/published.rs`'s `published::get`-style read, or `Emulator::get_var`
  if exposed that way) — this model deliberately does not re-derive them
  from station weights itself (see `mass.rs`'s module doc) to avoid
  duplicating `weight_balance.rs`'s own logic.
- `engines`: `physics::engine::EngineOutputs::net_thrust_n` from each of
  the emulator's four `Engine`s (`Emulator::step_engine`/`Emulator::engines`).
- `controls`: the FBW flight-control laws' resolved surface commands (not
  modelled by this agent — this is a physics layer under the flight
  controls, per `registry.rs`'s doc comment on why it raises no ECAM
  alerts itself).
- `wind`/`oat_offset_k`/`qnh_offset_pa`: whatever scenario/atmosphere
  module owns those; today's `Emulator::set_oat_c`/`set_qnh_pa`/
  `set_wind_ms` are the *fake* environment setters this model would let a
  test drive procedurally instead.

## Notes

- No new `Vars`/dataref is published by this module — it is pure physics,
  with no dependency on `Vars`/`VariableRegistry`/the plugin crate's
  internals (only `fbw_a380_systems::deep::api` for `registry.rs`, per the
  lead's explicit instruction that this is the one allowed crate reference).
- `registry.rs` supersedes an earlier CATALOGUE.md/ECAM.md pass per the
  2026-09-19 lead update; those files were never left in this directory
  (this agent moved straight to `registry.rs` once that update arrived, so
  there was nothing to delete).
- Axis convention (`geometry.rs`'s module doc): body x-forward, y-right,
  z-down (SAE aerospace convention), which is *not* the MSFS cfg's own
  `(z, x, y)` forward/right/up convention — `geometry::msfs_point` is the
  one conversion point.
