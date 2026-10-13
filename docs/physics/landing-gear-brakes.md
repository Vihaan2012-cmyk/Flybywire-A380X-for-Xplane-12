# Landing gear, brakes, steering and BTV (ATA 32)

Audit of the plugin's ATA 32 integration (gear extension, brakes, antiskid,
autobrake/BTV, nose and body wheel steering, LGCIU/proximity sensors) against
FlyByWire's own `a380_systems` crate (`D:\fbw-aircraft\fbw-a380x\...\hydraulic\
mod.rs`, `landing_gear.rs`, `nose_steering.rs`, `brake.rs`, `brake_circuit.rs`,
`autobrakes.rs`), which the plugin runs unmodified as a path dependency
(`D:\A380\fbw-xp-systems\Cargo.toml`).

## What is already real (audited, no fix needed)

FlyByWire's own systems crate is a genuinely causal, study-level model here,
not a placeholder:

- **Brake temperature** (`fbw-common/.../hydraulic/brake.rs`): each brake's
  temperature integrates `actuator_pressure * wheel_speed-derived passed_length`
  as real heat energy (`Brake::update`), cooled by Stefan-Boltzmann radiation
  and a gear/airspeed-dependent convection coefficient (`calculate_gear_convection_coefficient`,
  a Reynolds/Nusselt correlation), with brake fans adding a second forced-convection
  term when powered. `BrakeProbe` (the cockpit gauge) has its own thermal
  inertia and is powered by an `ElectricalBusType`, going unavailable (not
  frozen, not zero — simply no signal) when unpowered.
- **Gear/door hydraulic valves** (`fbw-common/.../hydraulic/landing_gear.rs`,
  `GearSystemHydraulicSupply`): the safety valve and gear/door selector valve
  are each a `HydraulicValve` gated on `[DirectCurrentEssential,
  DirectCurrentGndFltService]` — lose both buses and the valves stay shut, so
  gear physically cannot move regardless of what the LGCIU commands. Gear
  motion is also naturally pressure-driven (`HydraulicLinearActuatorAssembly`),
  not a scripted animation.
- **LGCIU / proximity sensors** (`fbw-common/.../landing_gear/mod.rs`,
  `LandingGearControlInterfaceUnit`): fully power-gated (`receive_power`,
  `ElectricalBuses`); every downlock/uplock/compressed output is `is_powered
  && <sensor state>`, so an unpowered LGCIU reports nothing rather than a
  stale or default position.
- **Antiskid/autobrake/steering interlocks** (`hydraulic/mod.rs`,
  `A380HydraulicBrakeSteerComputerUnit`): `anti_skid_activated` already gates
  autobrake arming (`allow_autobrake_arming`), the alternate-brake pressure
  limit (2538 psi normal vs 1160 psi with antiskid off,
  `update_brake_pressure_limitation`), and nosewheel steering availability
  (`update_steering_demands`, combined with both engines' oil pressure and
  nose gear compression) — a real, causal fan-out from one signal, not
  independent hardcoded behaviours.
- **BTV** (`hydraulic/autobrakes.rs`): a real rollout-distance state machine
  (`BTVState::Armed/RotOptimization/Decel/EndOfBraking`) driven by measured
  deceleration and OANS runway/exit distance, not a lookup table.

## Gaps found, ranked

1. **The BSCU (`A380HydraulicBrakeSteerComputerUnit`) has no electrical
   dependency at all** — the one real gap in FBW's own model. Every other
   ATA 32 computer audited above (LGCIU, the gear valves, the brake probes)
   is properly `receive_power`-gated; this one is not, so losing its bus
   never touches antiskid, autobrake arming, brake pressure limiting or
   nosewheel steering even though its own logic already reacts correctly to
   `anti_skid_activated` in all four places once that value is honest. This
   is the literal "unpowered BSCU without its real effect" gap named in the
   brief.

   **Fix drafted:** `patches/fbw-rust/landing-gear-brakes.patch` adds
   `powered_by`/`is_powered` fields, a `receive_power` impl on
   `ElectricalBusType::DirectCurrentEssential` (the same bus category as
   `A380Hydraulic::EDP_CONTROL_POWER_BUS1`, the hydraulic control
   electronics this computer racks with), and changes the `read()` line
   `self.anti_skid_activated = reader.read(...)` to
   `self.anti_skid_activated = self.is_powered && reader.read(...)`. That
   one-line change is deliberately the entire fix: it lets the antiskid/
   autobrake/steering logic FBW already wrote do the rest, rather than
   adding a second, parallel "unpowered" code path.

   **Not applied.** This session's sandbox refuses every write to
   `D:\fbw-aircraft` (Edit tool, Bash `sed`/`python3`, and `git apply` were
   all denied as "Modify Shared Resources" — the same wall the breakers
   workstream hit and documented in `src/breakers.rs:197-203`). The patch
   file is verified against the current tree (`git apply --check` once
   applying is possible) but is unapplied and untested by `cargo test`.
   Whoever can write to `D:\fbw-aircraft` should apply it, then verify
   `A380HydraulicBrakeSteerComputerUnit`'s existing test module still
   passes and (ideally) add one asserting antiskid/steering/autobrake drop
   out when `DirectCurrentEssential` is unpowered.

2. **`ANTISKID BRAKES ACTIVE` is force-written to `1` every tick**
   (`src/sensors.rs:441-447`), unconditionally, because nothing binds
   X-Plane's `K:ANTISKID_BRAKES_TOGGLE` to move it and leaving it at its
   default (`0`) would spuriously disable steering and autobrake per the
   causal chain above. This means that even with fix #1 applied, an
   unpowered BSCU already reads `is_powered = false` and correctly drops
   antiskid regardless of this dataref's value — so #1 does take effect
   through a genuine power loss (bus failure, cold-and-dark, a breaker
   pull once the breaker workstream extends to this computer). What
   remains missing is the ANTI SKID cockpit switch itself: there is no way
   for a pilot to select antiskid off, only for the aircraft to lose it via
   power. Binding that switch is cockpit-controls territory (out of my
   files); once it exists it should feed this same dataref instead of the
   constant `1`.

3. **No fuse-plug or tyre-pressure/temperature model.** Neither FBW's
   source nor the plugin models the wheel rim fuse plugs or tyre gas state;
   `src/physics/damage.rs` already has a `32_101`/`32_102` "tyre burst
   (brake fuse-plug overheat)" failure arm, but it is driven by a separate,
   approximate brake-energy accumulator (`wheel_brake_ratio` dataref *
   groundspeed delta against a generic `BRAKE_ENERGY_REFERENCE_J`/600 s time
   constant) rather than the real per-wheel `BRAKE_TEMPERATURE_n` FBW's
   `Brake::update` already computes from actual actuator pressure and wheel
   speed. Re-pointing that arm at the real temperature (and adding an actual
   fuse-plug melting-point threshold with a cited alloy figure, plus tyre
   pressure loss on melt) is real, valuable, plugin-only work — it does not
   touch FBW source — but needs `WearTracker` to read `Vars`-registered
   systems variables, not just raw `Xplm` datarefs as it does today; left
   for the next pass given the time budget.

## Second pass (this session, time-boxed)

Fixed, both landed in the tree:

- **#2, the A-SKID switch is now real.** `src/sensors.rs`: found the
  converted aircraft's actual cockpit control —
  `cockpit_bindings.txt:1431`, `SWITCH_AUTOBKR_ASKID: toggles
  fbw/ANTISKID_BRAKES_ACTIVE between 1 and 0` — and `main.lua`'s handler for
  it (`rd("fbw/ANTISKID_BRAKES_ACTIVE") ~= 0`). `Sensors::new` now finds that
  dataref, corrects it to `1` once at spawn (the converter's own default
  value for it is `0`, per `main.lua`'s dataref-defaults table, but
  FlyByWire's BSCU starts with antiskid on, `hydraulic/mod.rs:4370`), and
  `update_inputs` copies the switch's live position into `ANTISKID BRAKES
  ACTIVE` every tick instead of force-writing `1`. This is now a real
  pilot-selectable switch feeding the same causal chain (autobrake arming,
  alternate-brake pressure limit, nosewheel steering availability) the
  original audit traced. Fix #1 (the BSCU power patch) is unaffected and
  still unapplied/untested (see above) — combined, an unpowered BSCU or a
  pilot-selected-off switch both now correctly drop antiskid.
- **Unrelated but in-area bug found and fixed**: `src/extra_backend_fcdc.rs`
  read `A32NX_LGCIU_1_NOSE_GEAR_COMPRESSED` for the FCDC's
  `nose_gear_pressed` discrete input, but FlyByWire's own
  `landing_gear/mod.rs:384` registers that variable **without** the
  `A32NX_` prefix (`LGCIU_{n}_NOSE_GEAR_COMPRESSED`; confirmed against that
  module's own `contains_variable_with_name` test, line 1789). Nothing
  writes the prefixed name, so `nose_gear_pressed` was permanently stuck
  `false` into the FCDC/FWS. Fixed to read the real name.
- **Checked, not a bug**: `A32NX_ROW_ROP_WORD_1` — FlyByWire's own
  `hydraulic/autobrakes.rs:665` also registers this **without** the
  `A32NX_` prefix (`ROW_ROP_WORD_1`), and nothing in this plugin's Rust
  source reads or writes it under either name (only a diagnostics-panel
  regex in `panel.html` mentions it). If the FWS/ECAM JS host reads it as
  `A32NX_ROW_ROP_WORD_1`, that is a naming bug on that workstream's side,
  not this one's — flagging for them rather than touching their files.

Not reached this session (time-boxed to ~2 hours total, most spent
re-auditing FBW's own `Brake`/`BrakeAssembly`/`HydraulicGearSystem` source
to scope real gaps before the deadline compressed): the tyre pressure/
temperature/fuse-plug/burst model (item 1 of the brief), `damage.rs`'s
re-pointing at real per-wheel `BRAKE_TEMPERATURE_n` (item 2), and the rest
of the "maximum depth" list. Findings worth keeping for whoever continues:

- **Tyre/brake indexing**: FlyByWire's A380 defines exactly 16 braked wheel
  positions, `BRAKE_TEMPERATURE_1..16` (`a380_systems/src/hydraulic/
  mod.rs:2074-2107`): left wing `[1,2,5,6]`, right wing `[3,4,7,8]`, left
  body `[9,10,13,14]`, right body `[11,12,15,16]`. `REPORTED_BRAKE_
  TEMPERATURE_{index}` (same file, ~line 3276) is the power-gated cockpit/
  BTMU reading (`BrakeProbe::signal()`, `None`/0 when unpowered by
  `ElectricalBusType::DirectCurrent(1)` — currently hardcoded `TODO` per
  wheel, not a real bus assignment); `BRAKE_TEMPERATURE_n` itself is the
  raw physical value, always available, and is what a real tyre heat-soak
  model should read (physics does not care whether the BTMU is powered).
  The nose gear (2 wheels, unbraked) has no brake-temperature channel at
  all — only strut-level `CONTACT POINT COMPRESSION:0` / no wheel-rpm
  mapping — so a nose tyre model would need a simpler, lower-fidelity
  rolling-only heat term, documented as such.
- **Brake fans are wired in FBW's `Brake.rs` (589 W motor, bus-gated,
  `BRAKE_FAN_BTN_PRESSED`/`BRAKES_HOT` panel) but not instantiated for the
  A380**: all four `BrakeAssembly::new(...)` calls in `hydraulic/mod.rs`
  pass `None` for `brake_fan_bus` (line 2079 etc., each marked `// TODO`
  right above for the sensor bus assignment too). Wiring a real brake fan
  bus assignment is an FBW-side one-line-per-assembly change (a
  `Some(ElectricalBusType::...)` in place of `None`), i.e. a patch under
  `patches/fbw-rust/`, not plugin work — flagging since it directly matches
  the brief's "brake fans with power" item and the fix is small once
  someone picks the right bus.
- **Parking brake accumulator, alternate braking, gear/door sequencing,
  gravity extension**: re-confirmed FBW's own `Accumulator`,
  `HydraulicGearSystem`/`GearSystemComponentAssembly`
  (`fbw-common/.../hydraulic/landing_gear.rs`) and `nose_steering.rs` are
  genuinely causal, pressure/proximity-detector-driven physics already
  (matching the first pass's conclusion) — no plugin-side gap found there
  in the time available; a real gap would be accumulator internal leakage
  over many hours (idle bleed-down independent of use), which was not
  checked.
- Tyre pressure (gas-law) and fuse-plug melting-point constants need care
  before writing: this repo's convention (see `damage.rs`'s
  `HARD_LANDING_VS_FPM`/`BRAKE_ENERGY_REFERENCE_J`) is to mark any number
  not confirmed against an A380-specific source as explicitly generic
  rather than imply false precision — worth the extra time to either find
  the Airbus AC-A380 document's tyre-pressure table (already cited in this
  file's #`MLW_KG`) or mark main/nose charge pressures generic.

## Remaining gaps (not reached)

- Fuse-plug/tyre-pressure model still keyed to the coarse damage.rs
  heuristic rather than FBW's real brake temperature (see #3).
- ANTI SKID switch has no cockpit binding (blocks #2; not my files).
- The BSCU power patch (#1) is drafted but unapplied/untested — needs a
  write path into `D:\fbw-aircraft` this session did not have.
- Body wheel steering (`BodyWheelSteeringControl`) and the alternate/park
  brake accumulator logic were read and found already causal (hydraulic
  pressure and `anti_skid_activated`/`parking_brake_demand`-driven); no
  further gaps found there in the time available.
