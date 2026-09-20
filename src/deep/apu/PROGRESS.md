# APU deep model — progress log

Area: `Area::Apu` (7). Directory: `src/deep/apu/`. See `docs/deep/BRIEF.md`.

New plugin Vars this model will eventually need published (none exist yet from
this directory — nothing outside `src/deep/apu` references this code yet, per
the brief's self-containment rule; these are forward declarations for
whoever wires this model into `AuxiliaryPowerUnit`/the plugin's variable
registry):

- `APU_LOAD_COMPRESSOR_SURGE` (bool) — `load_compressor::Outputs.in_surge`.
- `APU_IGV_POSITION` (0..1) — `load_compressor::Outputs.igv_position_frac`.
- `APU_SCV_POSITION` (0..1) — `load_compressor::Outputs.scv_position_frac`.
- `APU_STARTER_CURRENT_A`, `APU_BATTERY_VOLTAGE_V` — `starter.rs` outputs.
- `APU_GEN_1_OVERLOAD` / `APU_GEN_2_OVERLOAD` (bool) — `generators.rs`.
- `APU_FUEL_SOLENOID_OPEN`, `APU_FUEL_METERED_KG_S` — `fuel_control.rs`.
- `APU_INLET_DOOR_JAMMED` (bool) — `inlet_door.rs` (reuses existing
  `APU_FLAP_OPEN_PERCENTAGE` for position).
- `APU_FIRE_LOOP_DETECTED`, `APU_FIRE_BOTTLE_DISCHARGED` — `fire.rs`.

Existing plugin Vars reused as-is (already published by the ported
`AuxiliaryPowerUnit`, `src/failures.rs`, `src/study/*`): `APU_N`, `APU_EGT`,
`APU_EGT_WARNING`, `APU_OIL_PRESSURE_PSI`, `APU_OIL_TEMPERATURE_C`,
`APU_BLEED_SUPPLY_KG_S`, `APU_BLEED_SUPPLY_PSI`, `APU_PROTECTIVE_TRIP`,
`APU_FLAP_OPEN_PERCENTAGE`, `FAIL_APU_FUEL_CONTROL_HOOK`,
`FAIL_APU_STARTER_HOOK`.

## Log

- [done] gas.rs — `src/deep/apu/gas.rs` — shared gas-property constants and
  isentropic/corrected-flow relations, self-contained copy (brief requires no
  cross-directory dependency).
- [done] params.rs — `src/deep/apu/params.rs` — cited (rated power, two-shaft
  architecture) and GENERIC (everything else) PW980A-class parameters,
  independently derived from FlyByWire's own file (not read from it beyond
  the two public facts both files cite).
- [done] compressor_map.rs — generic scaled compressor map (corrected speed
  lines, surge line, choke line, efficiency island), shared by the
  power-section and load compressors — item 1/backlog.
- [done] turbine_flow.rs — Stodola's ellipse law turbine flow capacity
  (forward and closed-form inverse) plus an efficiency-island expansion —
  item 1/backlog.
- [done] combustor.rs — real fuel/air energy balance (own copy of the
  standard method, not FBW's fitted shortcut) plus its own inverse used to
  calibrate the design point.
- [done] power_section.rs — item 1: single-spool gas path (core compressor +
  combustor + Stodola turbine + torque balance), replacing FBW's fixed
  "fraction of fuel energy extracted" shortcut with the real chain end to
  end. Tests: rest/no-NaN, starter-only spin-up, design-point torque
  self-consistency, accessory overload slows the spool, compressor erosion
  and turbine damage each independently raise EGT for the same speed/fuel,
  overspeed/EGT hard-trip thresholds, blocked inlet reduces delivered
  pressure.
- [done] actuator.rs — shared rate-limited-actuator-with-jam-fault helper
  (DRY: used by IGV, SCV and, next, the inlet door).
- [done] load_compressor.rs — item 1's separate load/bleed compressor, plus
  item 2: IGVs (meter incoming flow to demand) and a surge control valve
  (its own anti-surge control law, `SCV_SURGE_SAFETY_MARGIN_FRAC` above the
  surge line). Tests include the brief's own named scenario: a sudden bleed
  demand drop with the SCV jammed causes real surge (`in_surge` from
  `compressor_map`), which a healthy SCV avoids by opening in time; a jammed
  IGV starves delivery even with demand present; erosion lowers delivered
  pressure.

- [done] governor.rs — item 3's constant-100%-speed FCU governor: own
  proportional-integral controller (no crate dependency), conditional-
  integration anti-windup, plus an EGT-limit fuel schedule (start vs.
  running limits) using the same isentropic relation `turbine_flow::expand`
  uses, solved in reverse. Tests: zero fuel when not running, saturates at
  its ceiling not beyond, zero extra fuel at the setpoint, stable across
  tick rates 0.01-1.0 s against a proxy plant, schedule allows more fuel as
  speed rises, running limit stricter than start limit, no NaN near zero
  speed.
- [done] starter.rs — item 3's start sequence and starter motor: series DC
  motor circuit solved simultaneously with the battery's own internal
  resistance (so battery voltage sag under starter inrush current is a
  causal consequence, not scripted), phase machine (crank/light-off/self-
  sustaining) driven purely by measured speed. `StarterFaults` covers
  starter degradation (more current, less torque at the same speed) and
  igniter failure (raises the effective light-off speed threshold past what
  the starter alone can reach at full failure -- a hung start with no
  special-case branch). Tests cover all of the above plus no-battery and
  phase-transition behaviour.
- [done] generators.rs — item 3's two generators' shaft load:
  `shaft_power = electrical_load / efficiency`, rated real power derived
  from the public "two 120 kVA generators" fact times a generic 0.8 power
  factor. `GeneratorFaults` covers winding/bearing wear (more shaft power
  for the same output) and overload-protection failure (removes the normal
  clamp at rated power, item 5's "generator overload").

- [done] oil.rs — item 4's oil system: gear pump (flow ∝ N, relief-valve
  regulated above `OIL_REGULATION_N_PERCENT`), a leak that drains tank level
  over time and progressively starves pump pressure as level falls below a
  low-level threshold (rather than an instant cliff at empty), and a lumped
  friction/ambient heat balance via an exact exponential step. Debounced low
  -pressure protective trip. Tests cover regulation, sub-regulation-speed
  scaling, leak drainage, eventual low-pressure trip, brief-dip non-trip,
  friction heating, and no drift with no leak.
- [done] fuel_control.rs — item 4's fuel control unit: instantaneous
  shutoff solenoid gate plus a rate-limited metering valve
  (`actuator.rs`) tracking the governor's commanded flow. `metering_valve_jam`
  is item 5's "fuel control unit fault": the valve freezes at whatever flow
  it was holding, over/under-fuelling from then on. Tests cover the solenoid
  gate, settling on command, the rate limit, the jam freezing position and
  ignoring later commands, and zero-max-flow safety.
- [done] inlet_door.rs — item 4's inlet door actuator (`actuator.rs`) plus
  the pressure-loss-vs-position relation `power_section.rs` consumes as
  `inlet_pressure_loss_frac`: a jammed door (item 5's "inlet door jam")
  causes a real, continuing inlet pressure loss, not a scripted symptom.
- [done] fire.rs — item 4's fire interface: confirms a fire from an external
  loop-detected input (bay/loop physics is `deep/fire_ice`'s area, not
  duplicated here), commands fuel shutoff and bleed valve close, and models
  a genuinely single-shot pyrotechnic extinguisher bottle (exponential
  discharge, cannot refire once spent). `FireFaults` covers loop failure
  (never confirms) and squib failure (confirms but never discharges).

- [done] faults.rs — item 5's `ApuFaults` aggregate, `Default` = all healthy,
  collecting every subsystem's own faults plus two signal-level faults that
  belong above any single subsystem: `speed_sensor_bias` (governor overspeed)
  and `egt_sensor_bias` (EGT indication).
- [done] apu.rs — top-level `Apu`/`Inputs`/`Outputs`, wiring every subsystem
  into one `step` in the documented per-tick order, including the one-tick
  lag on accessory torque and the previous-tick omega fed to the starter.
  Tests: a full healthy start reaching "available" with no NaN anywhere
  (door -> crank -> light-off -> self-sustaining -> governed, nothing
  scripted), no start with no battery, a mid-run fire confirming shutoff and
  zeroing bleed delivery, a fully biased speed sensor driving true N above a
  healthy run's N, and an EGT sensor fault reading exactly
  `EGT_SENSOR_MAX_BIAS_C` low without moving the true physics.
- [done] mod.rs — declares every submodule and re-exports `Apu`/`Inputs`/
  `Outputs`/`ApuFaults`. Nothing outside this directory references it yet;
  the lead adds `pub mod apu;` to `src/deep/mod.rs` once every area is in
  place (no sibling area has self-registered there either, confirmed by
  reading the file before touching anything outside this directory).
- [done] registry.rs — registers all 18 failures, 16 components and 8 ECAM
  alerts through `crate::deep::api` (`Area::Apu`, ATA 49), per
  `docs/deep/BRIEF.md`'s "Registering failures, components and ECAM alerts"
  section (superseding the earlier CATALOGUE.md/ECAM.md instruction, which
  this directory never created files for, so there was nothing to delete).
  Tests: registers with zero `Registry::validate()` errors, every component
  names at least one real failure, and no duplicate failure ids.
- [done] FAILURES.md — the human-readable index of the same 18 failures
  `registry.rs` registers in code (item 5 in full: compressor erosion,
  turbine damage, IGV jam, surge valve stuck, starter failure, igniter
  failure, fuel control unit fault, oil leak/low pressure, inlet door jam,
  overspeed (via a governor speed-sensor fault), EGT sensor fault, generator
  overload -- plus three natural extensions built alongside them: load
  compressor erosion, generator winding wear, and the fire loop/squib pair).

## A calibration bug found and fixed by hand-checking the model's own numbers

While writing `apu.rs`'s end-to-end start test, I hand-derived (not simulated
-- no builds/tests can run per the brief) the starter-vs-compressor-drag
torque balance at `LIGHT_OFF_N_PERCENT` from this file's own equations
(`compressor_map.rs`'s n^2.8 power-vs-speed scaling, the starter's series-
motor torque relation) using the *first* choice of `STARTER_KE_KT`/
`STARTER_ARMATURE_RESISTANCE_OHM`. It showed the core's own compressor drag
overtaking starter torque *before* reaching light-off (net torque ~ -0.15
N*m at 12% N) -- every start would have stalled just short of light-off and
never lit. Re-derived `STARTER_KE_KT`/`STARTER_ARMATURE_RESISTANCE_OHM`
(see `params.rs`'s own derivation in the doc comment there, using the
series-motor torque-vs-speed optimum `Ke = V/(2*omega)` evaluated at light-
off's own angular speed) so starter torque there is ~1.8x the compressor
drag; hand-checked forward from there that once fuel introduces at light-
off, turbine torque so overwhelms compressor drag (tens of N*m within a few
percent more N, against single-digit N*m of drag) that the starter's own
much-lower stall point above light-off never matters. Also added bounded-
substep integration to `power_section.rs::PowerSection::step` (the brief's
own "sub-stepping where stiff" convention): this spool's small inertia
against its real torques is exactly the stiffness FlyByWire's own
`pw980_physics.rs` module docs note requiring the same technique for, an
independent instance of the same standard explicit-integration practice
rather than anything copied from that file.

## Extending past the backlog

The five backlog items are done. Natural next steps if picked back up:
constant-volume-plenum dynamics for the load compressor (currently
quasi-steady, matched instantaneously with the core spool's speed and
demand, which is fine for the surge scenarios tested here but would matter
for faster transients); a second, lighter free-turbine rotor if a future
task ever wants to drop the single-spool simplification `params.rs`
documents; wiring real bleed/electrical demand schedules in from the other
workstreams once those Vars exist; and the plugin-side `SimulationElement`
glue (`read`/`write`, Vars) once the lead wires `pub mod apu;` into
`src/deep/mod.rs` -- none of that belongs in this self-contained pass.

## Second round (coordinator's 5 new items) -- status at hard stop

- [done] gas.rs -- added `total_temperature_k`/`total_pressure_pa` (ram
  relations) and `density_ratio` (ideal-gas altitude density), with tests.
- [done] oil.rs -- added `viscosity_cst` (Walther relation, public
  MIL-PRF-23699 data points) and `OilSystem::cold_drag_torque_nm` (extra
  cranking drag from cold-soaked oil, zero once warm), with tests.
- [done] start_envelope.rs (NEW) -- item 1: `FlightCondition`, ram inlet
  conditions, `relight_permitted` (density-ratio altitude ceiling),
  `windmill_torque_nm` (modest scoop-inlet windmilling). Tests cover ground/
  airspeed/altitude behaviour and NaN-safety.
- [done] interfaces.rs (NEW) -- item 5: `BleedOutput`/`GeneratorOutput`
  (this APU's real outputs to `deep::pneumatic_ducts`/`deep::electrical`)
  and `BatteryInput` (the Thevenin-equivalent battery source
  `deep::electrical`'s own model is meant to supply the starter).
- [done] starter.rs -- refactored to take `interfaces::BatteryInput`
  instead of a bare voltage + this file's own hardcoded resistance
  constant; added `duty_cycle_locked_out` and `relight_permitted` gating
  inputs (items 1 and 3). Tests updated; two new ones added.
- [done] life.rs (NEW) -- item 3: `CoreLife` (operating hours, start
  cycles, Arrhenius-style hot-section creep above a threshold EGT ->
  `compressor_wear_frac`/`turbine_wear_frac`) and `StarterDutyCycle`
  (lumped cranking-heat budget, cool-down, lockout, own model-fault bias).
  Full test coverage.
- [done] ecb.rs (NEW) -- item 2: dual-channel ECB, per-channel speed/EGT/
  oil-pressure sensors (bias or outright failure) and a per-channel
  processing fault that excludes the whole channel; control signals
  averaged/selected/lost across still-valid channels; protections
  (overspeed, EGT, oil) voted OR-across-valid-channels with debounce; fire
  and the inlet-door start interlock hard-wired, not voted;
  `dual_channel_speed_loss` is its own protective condition. 12 tests.
- [done] load_compressor.rs -- item 4: `electrical_load_frac` input and
  `igv_load_shed_scale` (sheds IGV-tracked bleed demand as combined
  generator load rises), plus `delivered_temperature_k` output for
  `BleedOutput`. Two new tests.
- [done] faults.rs -- removed the old flat `speed_sensor_bias`/
  `egt_sensor_bias` fields (superseded by `ecb::EcbFaults`); added
  `ecb: EcbFaults` and `starter_duty_model_fault`.
- [done] apu.rs -- fully rewired: ram inlet conditions feed
  `power_section`'s ambient inputs; windmill torque added to the
  driving-torque side; cold-oil drag added to the accessory-torque side;
  starter takes `BatteryInput`/duty-lockout/relight-permitted; ECB sits
  between the true N/EGT/oil signals (one-tick lag) and the governor, and
  its `any_trip` gates `running` alongside fire; life tracking accumulates
  every tick/start (wear combined with any injected fault via `max()`);
  duty-cycle heat accumulates/decays around every starter engagement;
  `Outputs` carries the `BleedOutput`/`GeneratorOutput` interfaces and the
  new ECB/life/envelope fields. Five tests, including dual-channel speed
  loss being a real protective condition, not a silent freeze.
- [done] registry.rs -- updated the two speed/EGT sensor `FailureDef`
  entries' `model_field`/`effect` text to point at `ecb.rs`'s per-channel
  fields instead of the removed flat bias fields (same ids/component, "18
  distinct failures" test still holds).
- [NOT done, flagged] `FAILURES.md`'s two speed/EGT sensor rows still name
  the *old* flat `ApuFaults` fields -- stale, needs the same text sync
  `registry.rs` just got (exact text already written there). Everything
  else in `FAILURES.md` is still accurate.
- [NOT started] no new `EcamAlert`/`FailureDef`/`ComponentDef` entries for
  the ECB's own per-channel processing fault or the starter duty-cycle
  model fault (both exist in code, `ChannelFaults.processing_fault` and
  `ApuFaults.starter_duty_model_fault`, but are not yet registered as their
  own failures/components); no new ECAM alerts for dual-channel loss,
  creep-life exceedance, or duty-cycle lockout.

### Next, in order, if resumed

1. Sync `FAILURES.md`'s two stale rows to `registry.rs`'s new text.
2. Register `ChannelFaults.processing_fault` (x2, A/B) and
   `starter_duty_model_fault` as their own failures/components.
3. Re-run the kind of hand calibration check that caught last round's
   starter-vs-compressor-drag bug, this time for windmill torque + cold-oil
   drag interacting with that same tight light-off margin on a cold,
   high-altitude start -- not yet re-verified for this round's additions.

- [done] Live system — `live.rs` (`live_system() -> Box<dyn deep::live::Area>`), `mod.rs` — `LiveApu` owns one `Apu`,
  driven from `Truth` (ambient p/T, TAS, `dc_bus_volts` for the starter's supply, `apu_running` as the run command)
  and from all 18 registered failure ids via new `registry::ids` constants. Publishes `APU_N`, `APU_EGT`,
  `APU_OIL_PRESSURE_PSI`, `APU_LOAD_COMPRESSOR_SURGE`, `APU_GEN_1_OVERLOAD`, `APU_GEN_2_OVERLOAD`,
  `APU_FIRE_LOOP_DETECTED` (every var this area's ECAM triggers name) plus 24 Study vars. Truth gaps named in
  `live.rs`'s module doc: overhead panel (master/start pb), bleed demand, generator load, bay fire loop, battery
  source resistance.
- [done] Governor EGT limit — `governor.rs`, `power_section.rs` — the acceleration schedule is now solved on
  `power_section::gas_path` (the authoritative compressor/combustor/Stodola chain, factored out of `step_once`),
  iterated to a Stodola-consistent turbine pressure ratio, replacing the `1 + (pr_design-1)*n_frac` stand-in; a new
  `EgtLimiter` closes the same limit on the ECB's measured EGT (min-select), which is what protects a degraded
  machine the open-loop schedule cannot know about. `Apu::step` now sub-steps its whole control chain at 0.05 s
  like `PowerSection::step` already did, so a half-second post-pause frame no longer overshoots governed speed.
  Measured: governs 100.000% / 603 C at dt 1/30..0.5, healthy; the reported 77.6% governing did not reproduce.
- [done] sourced-constants pass — params.rs, registry.rs — Split the EGT limits into control / warning-caution / protective-trip, which they were conflating. Sourced to FBW's own PW980A model: start control limit 900 C (`pw980_physics.rs:547`), protective trip 950 C (`pw980_physics.rs:664`), running warning 900 C (`pw980.rs:32` -- note the A320 APS3200's is 682 C, so this is genuinely aircraft-specific), start warning 900/982 C by FL250 and caution = warning - 33 C (`electronic_control_box.rs:276-277,341-344`). Added `egt_warning_c`/`egt_caution_c` plus 4 tests. `EGT_RUNNING_LIMIT_C` 750 stays GENERIC but is now explicitly a *control* limit below the 900 warning. The ECAM trigger now references `EGT_TRIP_C` instead of a bare 950 literal. Not yet published as vars: `APU_EGT_WARNING_C`/`APU_EGT_CAUTION_C` need a pressure altitude in the APU publisher, which it does not currently receive.
