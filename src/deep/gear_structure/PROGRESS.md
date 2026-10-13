# Gear structure workstream — progress log

- [done] Per-leg oleo-pneumatic strut model — `src/deep/gear_structure/strut.rs` — polytropic gas spring + quadratic
  orifice damping; limit/ultimate loads derived by literally running the same model through both required
  CS-25.473(a) drop-test conditions (10 fps at MLW, 6 fps at MTOW) rather than assuming a load factor; CS 25.303's
  1.5 ultimate factor of safety; CS 25.485's 0.8 side-load factor; servicing (gas/oil) leak faults; overload events
  that leave lasting seal damage which itself accelerates the gas leak (emergent, not scripted); Miner's-rule
  fatigue per ground-contact cycle; collapse on ultimate exceedance or on reacting a real load while unlocked.
  16 unit tests (energy/equilibrium closed-form cross-checks, collapse thresholds, fatigue ordering, numerical
  safety at rest/dt=0).
- [done] Retraction/extension system — `src/deep/gear_structure/retraction.rs` — door/gear/lock state machine
  (`Locked -> DoorsOpening -> Traveling -> DoorsClosing -> Locked`), uplock release gated on hydraulic pressure and
  jam severity (with gravity extension's separate, more resistant release path), spring-loaded downlock engagement
  with a fail mode, door jam freezing/slowing the sequence, and a possibly-lying proximity-sensor indication
  layered independently over the true lock state. 9 tests, including the "position vs lock truth" distinction a
  failed downlock needs (a leg that reaches the down position but never locks must still register a later retract
  command).
- [done] Per-wheel carbon brakes, antiskid, parking brake — `src/deep/gear_structure/brakes.rs` — carbon-carbon
  heat-sink thermal model (friction power in, natural/forced convection + Stefan-Boltzmann radiation out), energy-
  budgeted wear, a per-wheel antiskid channel (slip-ratio release, defeatable by a channel fault), brake fire from
  sustained extreme heat, and a nitrogen-precharged parking-brake accumulator (Boyle's law, the same form
  `physics/tyre.rs` already uses for a sealed gas volume) with a leak fault that lets it silently stop holding.
  `BrakeWheel::stack_temp_c` is the documented coordination point for `physics/tyre.rs`'s `brake_temp_c` input
  (this workstream cannot edit that file itself). 9 tests.
- [done] Nosewheel/body-gear steering + shimmy damper — `src/deep/gear_structure/steering.rs` — rate-limited
  steering actuator, a GENERIC low-speed opposing/phase-out schedule for the body gear's rear-axle assist, and a
  genuine linear shimmy model (`I*delta'' + c_net(v)*delta' + K*delta = disturbance`, `c_net` falling with speed and
  with damper-fault severity) whose critical speed the damper fault pushes from "never" down into a realistic taxi
  range — an emergent instability, not a scripted vibration. 8 tests.
- [done] Structural cross-cutting effects — `src/deep/gear_structure/structure.rs` — tailstrike geometry coupled to
  the *actual* (not nominal) body-leg compression (re-derives `physics/damage.rs`'s own published contact-point
  geometry independently, since this workstream cannot depend on that crate-internal module, and cross-checks
  against its cited 12.99 deg figure), a wing-root bending fatigue index fed directly from the wing legs' own
  Miner's-rule landing-cycle peaks, a GENERIC-tiered overweight-landing inspection trigger, and a combined
  hard-landing load report. 8 tests.
- [done] System aggregator — `src/deep/gear_structure/mod.rs` — `GearSystem` ties all five legs (nose, 2x wing,
  2x body), the 16 braked wheels (in `physics::tyre::Tyres`'s own `LEG_WHEEL_INDICES` layout, so the two line up
  wheel-for-wheel once wired together), the parking brake and the wing fatigue tracker into one `step`, producing
  the per-leg collapse state and loads the flight model needs (brief item 5) plus an events log. `hard_landing_report`
  builds a `structure::HardLandingReport` from each leg's own live force/utilisation (not placeholders) for
  on-demand use right after a touchdown. 7 tests, including one confirming the report threads through real nonzero
  peak force and overload on the legs that actually took a hard landing (reviewer-flagged fix).
- [done] Registration — `src/deep/gear_structure/registry.rs` — every component (30: 5 struts, 5 retraction
  systems, 16 wheel brakes, 1 parking-brake accumulator, 3 steering axles), every failure (74, all 12 fault types
  from `FAILURES.md` expanded per leg/wheel/axle instance) and 6 ECAM alerts (gear not down-locked, gear position
  disagree via a genuine sensed-vs-true `Cond::VarVar`, wheel brake fire, antiskid channel fault, parking brake low
  pressure, and `L_G_STEER_SHIMMY` covering all three steerable positions — nose, left body, right body) registered
  in code via `Area::GearStructure`, per the lead's replacement of CATALOGUE.md/ECAM.md. New Vars this model would
  need to publish once wired into the plugin are documented in `registry.rs`'s own module doc comment (none exist
  yet; this workstream stayed fully self-contained per the brief's hard rules). 6 tests, including a full
  `Registry::validate()` pass and two reviewer-flagged fixes: `GearSystem::hard_landing_report` was threading
  hardcoded zero/false peak-force/utilisation/overload through instead of each leg's real computed values (fixed,
  with a regression test asserting a hard landing reports a real nonzero peak force and overload); and the
  `L_G_STEER_SHIMMY` alert's `raised_by` covered all three steerable positions while its trigger originally read
  only the nosewheel's variable, so a body-gear-only shimmy could never raise it (fixed by widening the trigger to
  all three positions' variables, with a regression test and a scope-audit test pinning the other multi-position
  alerts' trigger/raised_by counts).

## New Vars this model must publish (not yet wired; see `registry.rs`'s doc comment for the full list and why)

`GEAR_STRUT_GAS_CHARGE_FRACTION:n`, `GEAR_STRUT_OIL_LEVEL_FRACTION:n`, `GEAR_STRUT_LIFE_FRACTION:n`,
`GEAR_STRUT_COLLAPSED:n`, `GEAR_POSITION:n`, `GEAR_DOWNLOCKED:n`, `GEAR_UPLOCKED:n`, `GEAR_DOOR_POSITION:n`,
`GEAR_STUCK_LOCKED:n`, `SENSED_GEAR_DOWNLOCKED:n`, `SENSED_GEAR_UPLOCKED:n`, `BRAKE_STACK_TEMP_C:n`,
`BRAKE_WEAR_FRACTION:n`, `BRAKE_FIRE:n`, `ANTISKID_CHANNEL_FAULT:n`, `PARK_BRAKE_PRESS_PA`, `PARK_BRAKE_HOLDING`,
`PARK_BRAKE_SET`, `NW_STEER_ANGLE_DEG`, `NW_STEER_SHIMMY_UNSTABLE`, `BODY_STEER_ANGLE_DEG:n`.

- [done] Closed all three coordinator-flagged gaps in one increment:
  - **Side loads driven at the `GearSystem` level** — `mod.rs`'s `GearSystemInputs` replaced its single shared
    `main_on_ground`/`nose_on_ground`/`sink_speed_ms` fields with a `LegTouchdownInputs { on_ground, sink_speed_ms,
    side_load_n }` per leg (`nose`/`left_wing`/`right_wing`/`left_body`/`right_body`), and `side_load_n` now flows
    straight through `step_leg` into `strut::StrutInputs::side_load_n` (previously always zero at this level).
    New test: `a_side_load_alone_can_overload_a_leg_via_the_cs_25_485_lateral_path`.
  - **Individual per-leg touchdown timing** — the same `LegTouchdownInputs` split means each leg now has its own
    `on_ground`/`sink_speed_ms`, so e.g. a one-wheel-low crosswind landing (one main gear touching while its
    sibling stays airborne) is expressible and each leg's `strut::Strut` reacts only to its own real contact
    (the underlying per-leg touchdown-edge detection already lived in `strut.rs`; the aggregator's input surface
    was the artificial bottleneck). New test: `each_leg_touches_down_independently_instead_of_sharing_one_flag`.
  - **Wheel rotational dynamics owned by the model** — `brakes.rs`'s `BrakeWheel` no longer takes `wheel_speed_ms`
    as an external input; it now owns the wheel's own spin state and integrates it from first principles: inertia
    reduced to an effective contact-patch mass (`M_eff = I/r^2`), a friction complementarity solve against
    Coulomb's limit (`mu*N`, `N` = this wheel's own normal load, itself now derived in `mod.rs` from the owning
    leg's real strut force / that leg's total wheel count), and brake torque always resisting the wheel's own
    rotation. Unclamped, this reproduces exact zero-slip rolling; clamped, it produces a genuine, physically
    caused skid/lockup rather than skidding being told to the model via an external `wheel_speed_ms`. Brake heat
    now correctly scales with the wheel's *own* surface speed (a locked wheel's disc isn't rotating, so it stops
    generating brake heat even while its tyre is scrubbing — that heat now correctly belongs to `physics::tyre.rs`,
    not this module, an emergent distinction that falls out of owning the real state). `BrakeWheelInputs` gained
    `on_ground`/`normal_load_n` in place of `wheel_speed_ms`; `BrakeWheelOutputs` gained `wheel_speed_ms` as a real
    output. New/rewritten tests: `insufficient_friction_produces_a_genuine_physically_caused_lockup`,
    `a_healthy_antiskid_channel_keeps_releasing_and_recovering_instead_of_a_permanent_lockup`,
    `a_wheel_keeps_spinning_for_a_while_after_liftoff_instead_of_stopping_instantly`, plus the pre-existing
    heat/wear/fire tests updated to the new input shape (all still pass under the new physics, verified by hand).
  - No registry.rs changes were needed: none of these three changes add or rename an injectable fault (side load
    and per-leg timing are physical *inputs*, not faults; the wheel dynamics rewrite changes *how* `antiskid_inop`
    and `dragging` act, not their names or meaning), so all 74 registered failures' `model_field` strings are
    still accurate.
  - Files touched: `mod.rs` (`GearSystemInputs`/`LegTouchdownInputs`, `step_leg`, the brake-wheel loop, 5 tests
    updated/added), `brakes.rs` (`BrakeWheel`'s new wheel-dynamics state and constants, doc comment, 4 tests
    updated + 3 new). `strut.rs`, `retraction.rs`, `steering.rs`, `structure.rs`, `registry.rs` untouched by this
    increment.

## Scope notes / known gaps (for the next pass)

- No persistence/save-state wiring (matches the brief: nothing outside this directory references this code yet).
- The wheel dynamics model uses a single fixed Coulomb friction coefficient (`MU_TIRE_GROUND = 0.8`, dry runway);
  no wet/icy/contaminated-runway friction reduction is modelled yet, which would be the natural next step for the
  antiskid/lockup physics now that the underlying wheel-speed state is real.
- `structure.rs`'s wing fatigue tracker only reads the two wing legs' own cycle peaks; ground-induced loads
  reaching the wing through the body legs (if any, geometrically) are not modelled.
- The Vars listed above are still not wired to any real dataref (this workstream stayed self-contained per the
  brief's hard rules); that wiring, plus the runway-friction-condition input for the wheel dynamics, are the next
  most valuable items if continued.

## Live system (deep push, `live.rs`)

- [done] `live.rs` — the area's live instance behind `crate::deep::live::Area`
  (`live_system() -> Box<dyn Area>`), declared in this directory's `mod.rs`.
  `tick` drives the real models from `deep::live::Truth` and applies every
  failure `registry.rs` registers by reading `Faults::get(id)` into the exact
  `model_field` that entry names; `publish` emits every variable this area's
  ECAM triggers cite, plus the state behind them for the EFB Study pages.
  Failure ids are resolved at construction by registering into a throw-away
  `Registry` and looking each one up by component + `model_field`, so a
  renumbering in `registry.rs` fails loudly instead of silently unhooking a
  failure. Inputs `Truth` does not carry yet are collected in one documented
  `...Commands` struct per area rather than invented.
- [done] sourced-constants pass — strut.rs, brakes.rs — Descent velocities re-cited to the A380's *own* FAA special conditions (Docket NM341, FR 28 Mar 2006, cond. A.2: 3.05 m/s at MLW, 1.83 m/s at MTOW) instead of the generic CS-25 minima; fixed a real mis-citation (the 6 fps/MTOW case was labelled "reserve energy", it is a second *limit* case). Added the actual reserve-energy condition, CS/14 CFR 25.723(b) 12 fps at MLW, as its own drop case; `ultimate_load_n` is now `max(1.5 x limit, reserve-energy peak)` and a new test runs the 12 fps drop as a real touchdown. Brakes: `MAX_BRAKING_DECEL_MS2` = 2.8 from FBW's A380 BTV `MAX_DECEL_DRY_MS2`; `WHEEL_RADIUS_M` 0.6 -> 0.613 derived from the published 1400x530R23 main tyre at the 32% aircraft-tyre standard deflection, with the arithmetic in a test.

## 2026-09-20 — dead-failure audit follow-up (13 failures: 3 wired, 10 investigated as a real audit-coverage gap)

`deep::integration::failure_audit`'s sweep found 13 of this area's failures dead: 3 steering-actuator leaks (nose + 2 body) and 10 gear-retraction ones (uplock jam + downlock failure x5 legs).

**Steering actuator leaks (3, fixed).** `GearCommands::nose_steering_command_deg` was pinned at 0.0 forever -- nothing in production ever set it -- so `steering::SteeringActuator::step`'s own target never moved, and `actuator_leak` (which only throttles the *rate of an in-progress slew*, `diff = target - base_angle_deg`) had nothing to throttle. `Truth::controls::steering_command_deg[0]` (nosewheel tiller/pedal) now drives it directly, and `GearCommands::nose_steering_command_deg` is removed. `steering_command_deg[1]`/`[2]` (body left/right) are deliberately **not** read directly: a real A380 body-gear rear axle is not independently pilot-commanded, it is mechanically/electronically slaved to the nosewheel's own angle and groundspeed (`steering::body_steering_angle_deg`, already real, already-tested physics, left untouched). Because that schedule is driven by the nose actuator's own *tracked* angle (post-lag), wiring the nose command alone gives both body actuators a genuine nonzero target too, which is what their own `actuator_leak` faults need to become observable -- confirmed by the new test rather than assumed. If `steering_command_deg[1]`/`[2]` are ever meant to carry an independent, authoritative BSCU-computed body angle, that is a `Truth`-sourcing question for whoever wires `plugin.rs`'s publisher, not something to guess-consume here. New test: `a_steering_actuator_leak_only_shows_once_a_real_commanded_angle_gives_it_something_to_chase` (nose directly, left body through the derived schedule).

**Uplock jam + downlock failure, all 5 legs each (10 failures) — investigated, not a bug in this area.** Read `retraction::Retraction::step` line by line: `uplock_jam`'s release gate only runs on the `!self.position_down && commanded_down` branch (extending a leg that is currently *up*), and `downlock_fail`'s engagement check only runs when `target_down` reaches `GEAR_DOWN_THRESHOLD` (the same extend transition). Both are real, already correctly gated on the lever alone -- no missing `Truth` input, no gate that needs "more than the lever" (`retraction.rs`'s own unit tests, `a_jammed_uplock_prevents_ever_leaving_the_uplocked_state_hydraulically` and `a_failed_downlock_reaches_the_down_position_but_never_reports_locked`, already prove the state machine works, direct of `Truth`). The reason the audit's `gear_cycle` profile cannot see either: `deep::integration::failure_audit`'s own module doc states profiles are "held fixed for the whole run" (`Fixed rather than flown`), and `gear_cycle`'s `Truth` commands `gear_lever_down: false` (retract) for its entire 20 s window -- never back to `true`. A leg that only ever retracts can up-lock (which needs no uplock-jam gate at all, per the module's own "no release gate modelled" note on that direction) but can never reach the *second* transition -- extending again -- where either fault's own gate lives. This is the audit's own documented category (d), "genuinely only observable in a state this harness does not reach" -- not fixable from inside `src/deep/gear_structure/`, since `failure_audit.rs` is outside this pass's directory. Proven live with two new tests that do what the fixed-profile audit structurally cannot: command the gear up, then back down, in the same run — `uplock_jam_and_downlock_failure_are_real_and_only_show_on_the_extend_that_gear_cycle_never_commands` and `downlock_failure_reaches_the_down_position_but_never_locks_once_it_actually_extends` (the latter also checks `L_G_GEAR_NOT_DOWNLOCKED` actually fires on it). **Recommendation for whoever owns `failure_audit.rs`:** a second profile that starts up-locked and commands the lever down (or a `gear_cycle` that flips the lever partway through its own fixed window) would close this the same way for every other area's retract-then-extend-shaped failure, without this area inventing a fake input to force it.
