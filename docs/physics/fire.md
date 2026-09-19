# ATA 26 -- Fire Protection

## What already exists (FBW's own Rust, unmodified except as noted)

The physical fire system is *not* reimplemented in this plugin: `a380_systems`
(`a380_fire_and_smoke_protection.rs`, 1563 lines, compiled straight from
`D:\fbw-aircraft` per `Cargo.toml`) already models, causally and with its own
unit tests:

- **Heat source.** `SetOnFireModule` turns the `26_001..26_006` failures
  (`Z::Engine(n)` / `Z::Apu` / `Z::Mlg`, `src/failures.rs:141-151`) into
  `ENG_n_ON_FIRE` / `APU_ON_FIRE` / `MLG_ON_FIRE`; `aspects.rs:636-644`
  mirrors those onto MSFS's `ENG ON FIRE:n`.
- **Dual detection loops A/B**, each on its own DC bus (loop A: DC ESS, loop
  B: DC 2), each with its own `FireDetectionLoopID`-keyed failure
  (`26_007..26_014`, one pair per zone) -- AND logic when both loops
  disagree, OR-with-failed-loop logic, and a "both loops died within 5s"
  fallback so an electrical loss alone doesn't manufacture a false fire.
- **FDU** aggregating that into `FIRE_DETECTED_ENGn` / `_APU` / `_MLG` and an
  ARINC429 discrete word (`FIRE_FDU_DISCRETE_WORD`) for the FWS side.
- **Fire test pushbutton**, with its 500ms delay, forcing every loop/zone
  detected.
- **Fire pushbutton effects already wired in FBW's own code** (not this
  plugin, and not touched here): `EngineFireOverheadPanel`/`FirePushButton`
  (`is_released` on `FIRE_BUTTON_ENGn`/`FIRE_BUTTON_APU`) already trips the
  IDG/generator (`electrical/mod.rs:427-428`, `idg_push_button_is_released`),
  closes the hydraulic fire shutoff valve (`hydraulic/mod.rs:3404-3429`) and
  the bleed valve (`pneumatic.rs:498-504`), and arms the squibs
  (`ExtinguishingAgentBottle::update`).
- **Squibs/bottles**: 2 per engine + 1 for the APU, each powered by two
  buses (DC HOT 1, DC ESS) so one dead bus doesn't disarm it; a 1s timer
  between arming+press and `bottle_is_discharged` (irreversible once true).
- **APU auto-discharge on the ground**, 10s after `FIRE_DETECTED_APU` while
  compressed, skipped during the fire test.

## Gaps found and fixed here

1. **`zone_extinguishing_determination` decided whether a discharged bottle
   put the fire out with `rand::random()`** (`fire_and_smoke_protection.rs`,
   pre-patch line ~751-788) -- a 50/50 coin flip standing in for "the agent
   was sufficient", i.e. exactly the kind of faked value the project rules
   forbid. **Fixed** in `patches/fbw-rust/fire.patch` (not yet applied to the
   shared `D:\fbw-aircraft` checkout -- direct edits there were blocked in
   this session as a "shared resource", since 11 other agents run against
   the same checkout concurrently; the patch is verified with
   `git apply --check` and ready to apply). The fix makes extinguishing
   deterministic: a bottle that newly discharges always puts its zone's fire
   out. This is causally sound rather than an arbitrary relaxation, because
   by the time a squib can discharge, `engine_fire_push_buttons.is_released`
   is already true, which (after the fix below) already cut that zone's fuel
   source -- a full Halon 1301 charge into an already fuel-starved, sealed
   nacelle/APU bay has no real-world failure mode left to model. The second
   bottle per engine still matters causally: it's the fallback for a squib or
   bottle that fails to discharge at all (`bottle_discharge` stays false,
   e.g. a dead squib bus), not for agent that discharged but "wasn't enough".

2. **The engine fire pushbutton did not close the LP fuel valve.** FBW's own
   crate implements the generator trip, hydraulic shutoff and bleed valve
   closure on `is_released`, but nothing in either FBW's `a380_systems` or
   this plugin gated the engine's own fuel supply -- grepping FBW's fuel
   crate and this plugin's `fuel.rs`/`fuel_network.rs`/`engine_commands.rs`
   for `EngineFirePushButtons`/`is_released` turned up nothing. `engine_
   commands.rs` (out of this workstream's scope -- engines/fuel are other
   agents' files) already treats the cockpit's `GENERAL ENG STARTER:n`
   simvar as the engine's fuel valve (`engine_commands.rs:379,394`:
   `fuel_valve_open: master`). **Fixed** in `src/aspects.rs` (the "fire
   pushbutton -> LP fuel valve" aspect, next to the existing fire aspect):
   for each engine, `FIRE_BUTTON_ENGn` (FlyByWire's own pushbutton-released
   output) forces `GENERAL ENG STARTER:n` to 0 every tick it is released,
   and passes the pilot's own switch through unchanged otherwise -- it only
   ever forces fuel *off*; un-pulling the guard returns fuel control to the
   pilot (matching FBW's own
   `bottle_stays_discharged_after_fire_pb_is_reset` test: the *agent* used
   stays spent, but the *valve* is not latched). This runs `PostTick`, so it
   sees this tick's systems output and is in place for next tick's
   `engine_commands.update()` (which runs before this plugin's own
   `correctness.after_previous_tick()`/aspects post_tick -- see
   `lib.rs:1107,1226` and `correctness.rs`'s module doc for that ordering),
   the same one-tick lag the existing `ENG_n_ON_FIRE` roundtrip already has.

## Verified, not touched

- Detection loop AND/OR/failed-loop logic, the FDU discrete word, the fire
  test delay, squib dual-bus power, APU auto-discharge timing: read FBW's
  own 30-test suite in `fire_and_smoke_protection.rs` and this plugin's
  `failures.rs::an_engine_fire_is_detected_through_the_fire_aspect` (still
  passing) -- all already causal, no changes made.
- Generator trip / hydraulic fire valve / bleed valve closure on the fire
  pushbutton: already implemented in FBW's `electrical`, `hydraulic` and
  `pneumatic` modules; out of this workstream's file ownership regardless.

## Fire intensity, exposed for the visual/damage workstream

`aspects.rs`'s new "fire intensity" aspect publishes `A32NX_FIRE_INTENSITY_
ENGn` (n=1..4) and `A32NX_FIRE_INTENSITY_APU`, 0..1, mirroring the FDU's own
`FIRE_DETECTED_ENGn`/`FIRE_DETECTED_APU` (already the AND of both loops, so
false alarms from a single failed loop don't show up here either). This is
the "is a fire burning in nacelle n, intensity" contract for the workstream
driving X-Plane's own engine-fire failure/smoke/damage. It is 0 or 1 today,
not a smooth build-up/knockdown curve -- adding one honestly needs either a
continuous heat-source model or a timed ramp keyed off detection/bottle
state, which was not implemented for time; see remaining gaps.

## `A32NX_FIRE_SQUIB_*_IS_DISCHARGED` read by FWS, appears unwritten (diagnosed, not fixed)

FlyByWire's own `ExtinguishingAgentBottle::write` (`fire_and_smoke_
protection.rs`) unconditionally writes `FIRE_SQUIB_<id>_IS_DISCHARGED` every
tick (id in `1_ENG_1, 2_ENG_1, ..., 1_APU_1`), and other FBW-internal named
variables with the same "automatic dataref" pattern
(`FIRE_DETECTED_ENGn`) are already confirmed live and readable
(`aspects::tests::a380_copies_reach_the_systems_names` and this session's
own `fire_pushbutton_released_closes_the_lp_fuel_valve` test both read/write
such names with no special registration). No manual per-name dataref
registration table exists for FBW vars (`grep FIRE_SQUIB` in
`xplane_mirror.rs`/`xp.rs`: no hits, and none needed for `FIRE_DETECTED_*`
either), which rules out a registration gap as the cause.

That leaves a causal-input gap, not a wiring gap: `bottle_is_discharged`
only ever flips true if `squib_is_armed` (`engine_fire_push_button.is_
released`, i.e. `FIRE_BUTTON_ENGn`/`_APU`) *and* the agent pushbutton
(`OVHD_FIRE_AGENT_1_ENG_n_IS_PRESSED`) both go true for a tick. Both are
cockpit-control inputs -- out of this workstream's files -- and this
session found no evidence anything currently writes either one (no hits
searching for `FIRE_BUTTON_ENG` or `OVHD_FIRE_AGENT` as a write target
outside `fire_and_smoke_protection.rs`'s own read-then-echo). If the 3D
cockpit's fire pushbuttons/agent buttons aren't bound yet, the whole
extinguishing chain is correct and tested but never gets a live input, so
the FWS sees it permanently 0 -- indistinguishable from "unwritten" without
inspecting the cause. Recommend: confirm with the cockpit-bindings owner
whether `FIRE_BUTTON_ENG1..4`/`_APU` and `OVHD_FIRE_AGENT_1_ENG_1..4_IS_
PRESSED` have a write source; if not, that's the actual fix, in their
files, not this workstream's.

## APU fuel feed pressure (scoped, not implemented -- ran out of time)

The APU's `FuelPressureSwitch` (`systems::apu::mod.rs`) is, by its own doc
comment, a placeholder: `has_fuel_remaining: bool` from `A380Fuel::feed_
four_tank_has_fuel()` (a380_systems `fuel/mod.rs:170`, itself just "does
feed tank 4 have any mass", no valve/pump/pressure causality at all) --
"this type exists because we don't have a full fuel implementation yet."

This plugin's own `fuel_network.rs` already solves real per-line psi,
including the APU's own feed line -- `Valve.51 APULPValve`'s destination
line `APULPValveToExtraAPU` (cfg `Line.141`), fed from `Pump.21 APUFeedPump`
(matches `fuel_transfer.rs`'s `A380_APU_FUEL_PUMP = 21`) through `Valve.50
APUIsoValve` -- and already exposes it via `FuelNetwork::read_simvar(
"FUELSYSTEM LINE FUEL PRESSURE", 141)`. That *is* the full fuel
implementation the switch's comment is waiting for; it just isn't piped to
the APU yet. (One inconsistency spotted in passing, not mine to fix:
`fuel_transfer.rs`'s `A380_APU_FUEL_VALVE = 8` maps to cfg `Valve.8 =
FeedTank2FwdTransferValve1_2`, not an APU valve -- worth the fuel agent
checking whether `ApuFuelAspect`'s valve open/close is hitting the wrong
valve.)

The planned fix (not started -- do this first next session):
1. `fuel.rs`: one additive method, `pub fn apu_feed_pressure_psi(&self) ->
   f64 { self.net.read_simvar("FUELSYSTEM LINE FUEL PRESSURE",
   141).unwrap_or(0.0) }` (or the correct line index once cross-checked
   against `A380_APU_FUEL_VALVE`'s fix above).
2. `lib.rs`, one line per tick: publish that into a plugin Var, e.g.
   `APU_FUEL_FEED_PRESSURE_PSI`.
3. `patches/fbw-rust/apu.patch` (new): give `FuelPressureSwitch` (`systems::
   apu::mod.rs`) its own `VariableIdentifier` for that var, read it in
   `AuxiliaryPowerUnit::read` the same way `apu_bleed_extraction` already is
   (shared-contract "0 until another workstream writes it" pattern,
   `apu/mod.rs:326-329`), and replace the raw boolean with real hysteresis:
   `>=17psi` sets `has_pressure = true`, `<=16psi` sets it false, matching
   the switch's own doc comment (currently unimplemented -- today's
   `update(&mut self, has_fuel_remaining: bool)` has no hysteresis at all,
   just latches the last boolean). Remove `A380Fuel::feed_four_tank_has_
   fuel()`'s use at the `a380_systems/lib.rs` call site since the switch
   would source pressure itself.

## Remaining gaps (not reached; ranked)

1. **The fire.patch above is not yet applied to the live `D:\fbw-aircraft`
   checkout**, so the current running build still has the RNG-based
   extinguishing. Apply with
   `cd D:\fbw-aircraft && git apply D:\fbw-xp-systems\patches\fbw-rust\fire.patch`
   once no other agent is using that checkout, then rebuild.
2. **Cockpit input for `FIRE_BUTTON_ENGn`/`FIRE_BUTTON_APU`/`OVHD_FIRE_TEST_
   PB_IS_PRESSED`/`OVHD_FIRE_AGENT_1_ENG_n_IS_PRESSED`** was not audited here
   (cockpit control bindings are another agent's files); if nothing writes
   those from the X-Plane 3D pushbuttons yet, the whole chain above is
   correct but inert from the pilot's seat. Worth a live check with the
   `dref.mjs` tool once X-Plane is reachable (it was not reachable this
   session -- connection to `127.0.0.1:8086` refused).
3. **Bottle pressure is still a boolean (`bottle_is_discharged`), not a
   continuous quantity** -- the brief asked for pressure that "falls"; FBW's
   model only has full/empty, no partial-discharge state. Modeling a
   continuous psi value was scoped out this session for time; the
   deterministic fix above already satisfies "extinguished only if the
   agent is sufficient" at FBW's existing full/empty granularity.
4. **Heat source is binary** (`SetOnFireModule`, on/off from the failure),
   not derived from a continuous overheat/oil-loss/hot-section-damage model.
   This matches FBW's own real-aircraft simulation depth and the `SetOnFire`
   failure is the intended trigger for testing, so it was not treated as a
   gap.

## X-Plane visible/physical failure effects (separate session, time-boxed)

New module `src/physics/xp_effects.rs` (`physics::xp_effects::XpEffects`),
built in `Plugin::new` alongside `damage` and ticked in `Plugin::tick` right
after `self.damage.update(...)` (`lib.rs`, the `damage`/`xp_effects` fields
and the `[slot new: ...]` construction block). It reads only already-causal
state other workstreams derive and mirrors it onto X-Plane's own native
failure/effect datarefs; it never derives a cause itself.

1. **Nacelle fire -> X-Plane's own fire effect.** `ENG_{1..4}_ON_FIRE`
   (`SetOnFireModule` above, unchanged -- still the sole cause) is mirrored
   every tick onto `sim/operation/failures/rel_engfir{0..3}` (`0` working /
   `1` failed, the same convention `failures.rs::extra::drive` already uses
   for tyres/brakes). The dataref turns off the same tick FlyByWire's own
   extinguishing logic clears `ENG_n_ON_FIRE`, so "the fire stops when
   extinguished" holds by construction. Test:
   `engine_fire_boolean_drives_the_intensity_interface_with_no_xplane_host`.
2. **Shared fire-intensity interface, defined for this workstream.**
   `XP_ENGINE_FIRE_INTENSITY:n` (Vars float, `n` = 1..=4, 0.0..1.0) is the
   "fire burning in nacelle n (intensity)" state the brief asked for.
   Nothing else had added it at the time of writing (grepped
   `FIRE_INTENSITY`/`NACELLE_FIRE`/`fire_intensity`/`HeatSource`, no hits).
   Sourced from X-Plane's own `sim/flightmodel2/engines/is_on_fire`
   (float[16], read-only -- X-Plane's core flight model's own continuous
   ramp off the `rel_engfir` boolean this module sets), not a fabricated
   ramp rate, consistent with gap #4 above. **If a future heat-source model
   lands in the fire chain, retarget this `Var`'s source to it and keep the
   name** -- it is the contract other systems should read.
3. **Cockpit smoke from an uncontained APU/MLG-bay fire.** X-Plane's SDK has
   no APU-bay or MLG-bay fire *visual* dataref at all (grepped
   `DataRefs.txt` for `fire`/`smoke`/`nacelle`; only the four engine
   `rel_engfir*` exist). The nearest real consequence the SDK exposes is
   `sim/operation/failures/rel_smoke_cpit`: an uncontained fire aft of the
   pressure bulkhead or in the gear bay can vent combustion products into
   the ECS ducting that also feeds the cabin/cockpit. Edge-triggered off
   `APU_ON_FIRE || MLG_ON_FIRE`. Test:
   `apu_or_mlg_fire_alone_requests_cockpit_smoke_on_then_off`.
4. **Hydraulic reservoir leak -> X-Plane's native leak effect.**
   `FailureType::ReservoirLeak(Green|Yellow)` (ids `29_000`/`29_001`) is
   already consumed by FlyByWire's own ported hydraulic model, which drains
   `A32NX_HYD_{GREEN,YELLOW}_RESERVOIR_LEVEL` for real (`study/hyd.rs:111`
   reads the same variable). This module mirrors "that failure is active"
   onto `sim/operation/failures/rel_hydleak`/`rel_hydleak2`
   (green/yellow), edge-triggered so an MEL repair clears the effect. Test:
   `a_green_reservoir_leak_failure_is_tracked_and_clears_on_repair`.
5. **Verified, not duplicated.** Tyre burst / brake wear-out
   (`rel_tireN`/`rel_lbrakes`/`rel_rbrakes`) was already wired before this
   session by `failures.rs::extra::gear` + `physics/damage.rs`'s
   brake-energy model; confirmed against the existing passing test
   `tyre_and_brake_failures_map_to_distinct_native_datarefs`, nothing added.

### Remaining gaps in this workstream (ranked; a hard deadline cut this
session to a few minutes of implementation time)

1. **Engine seizure/flameout** (`rel_seize_0..3`/`rel_engfai0..3`) -- needs
   a real cause from the engine workstream's own state (oil pressure loss,
   N2/N3 collapse versus a pilot-commanded shutdown); var names not
   confirmed this session. Do not wire these from a failure tag directly.
2. **Engine separation** (`rel_engsep0..3`) -- the physically correct
   trigger is a sustained *uncontained* fire (extinguishing agent for that
   engine already exhausted, `ENG_n_ON_FIRE` still true) or catastrophic
   structural failure, not a bare timer; needs the fire chain's bottle-
   discharge state or a new structural-integrity accumulator in
   `damage.rs` (another workstream's file).
3. **Fuel leak visible effect** (`sim/operation/failures/rel_fuel_leak`) --
   same pattern as the hydraulic leak above, once the fuel workstream's own
   tank-quantity-loss failure and var names are confirmed
   (`fuel_network.rs`/`fuel_transfer.rs`).
4. **Windshield cracking, lightning strike, bird strike visuals, structural
   damage, gear collapse when not downlocked** -- datarefs exist and were
   catalogued this session (`sim/operation/failures/rel_bird_strike*`,
   `rel_wing*L/R`, `rel_fcon_*_gone`, `rel_gear_act`,
   `A32NX_LGCIU_1_{LEFT,RIGHT,NOSE}_GEAR_DOWNLOCKED`, `study/pages.rs:238-
   252`) but not wired: none had a confirmed, already-causal plugin-side
   trigger verified in time (gear collapse from a downlock failure at
   touchdown is the most promising next step). No native dataref exists at
   all for windshield cracking or a lightning-strike visual specifically
   (checked `DataRefs.txt`).

## ATA 49 -- Auxiliary Power Unit

Not reached this session (time was spent entirely on the fire chain above,
including the multi-hour-feeling detour of discovering the parallel-agent
"shared resource" restriction on `D:\fbw-aircraft` and building a patch-file
workflow around it). For the next session: FBW's APU is also not
reimplemented here -- `a380_systems` composes the generic `systems::apu`
module (`apu/mod.rs`, `apu/electronic_control_box.rs`,
`apu/air_intake_flap.rs`) with A380-specific `apu/pw980.rs` +
`apu/pw980_physics.rs` (the PW980, ported from the A32NX's APS3200 pattern --
see `apu/PW980.md`). `patches/fbw-rust/electrical.patch` already touches
`pw980.rs` (adds `ProvideCurrent` for `Pw980ApuGenerator`, i.e. starter
current draw), so some of the "start sequence on a real starter" brief may
already be done by a previous session -- read that patch and `pw980.rs`
first before assuming a gap. Audit needed: ECB power source, flap/inlet
door causal timing, N/EGT from real fuel flow and APU feed pressure,
bleed/load-compressor EGT coupling, generator load EGT coupling, and auto
shutdown causes -- none of this was verified in this session.
