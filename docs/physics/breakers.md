# Circuit breakers (stage-3 follow-on): a study-level catalogue over FlyByWire's own consumers

Scope: `docs/analysis/cockpit-study-cbs.md` CB-001/CB-002/STUDY-001 --
`src/circuits.rs` (B+E workstream) already models a real, working breaker
for every one of the 154 `systems.cfg` circuits (fuel pumps/valves, lights,
gear, radios, ...) plus 52 clickable-but-unwired avionics-bay `CB_*` panel
positions (`circuits::PANEL_CB_NODES`). The user's complaint was that this
is "far too little" next to a real A380 flight deck, and that it stops at
the cockpit's own wiring instead of the much larger set of consumers
FlyByWire's *systems* crate genuinely simulates (generators, TRUs, flight
control computers, air-conditioning LRUs, fire-detection loops, hydraulic
pumps, ...), none of which had a breaker at all.

## Checkpoint update (expansion + real-power-path pass)

Three coordinator directions landed mid-session, in order: (1) expand the
catalogue to "basically every breaker," chapter by chapter; (2) audit and
fold in all 154 `systems.cfg` circuits, dropping any that gate nothing; (3)
**pulling a breaker must open a real electrical path, not set a
`FailureType`** -- a priority change to the mechanism itself, not just the
practical effect. This section reports where each stands; the rest of this
file (below) is the original derivation table, still accurate for the
entries it covers, now supplemented by what follows.

### 1. The "no FailureType" requirement: an audit, not a rewrite for most of it

The instinct "a breaker should cut power, not set a failure flag" is right,
but auditing *where* `FailureType::is_active()` is actually read in
FlyByWire's own Rust turned up something the instruction didn't anticipate:
for the five electrical-**source** failure types this catalogue already
bridges (`TransformerRectifier`, `StaticInverter`, `Generator`,
`ApuGenerator`, `ElectricalBus`, 29 breakers total -- all of ATA24's
TR/GEN/APU-GEN/static-inverter/bus-tie entries), `is_active()` is read
**at the exact point FlyByWire's own Kirchhoff solver decides whether that
element contributes real potential to the bus** -- not a separate,
parallel "failure" effect:

| FailureType | Real site (file:function) | What it gates |
|---|---|---|
| `TransformerRectifier(n)` | `transformer_rectifier.rs::transform()` | `!failure.is_active() && input.is_powered()` -- the TR's own output `Potential`; false means `Potential::none()`, a real loss of DC bus power downstream |
| `StaticInverter` | `static_inverter.rs` | `report.is_powered(self) && !failure.is_active()` gates `has_output` |
| `Generator(n)` | `engine_generator.rs::should_provide_output()` | `&& !failure.is_active()`, read by `ElectricitySource::output_potential()` -- the VFG's Kirchhoff contribution |
| `ApuGenerator(n)` | `pw980.rs::should_provide_output()` | same pattern, `output_potential()` |
| `ElectricalBus(bus)` | `electrical/mod.rs::ElectricalBus::is_conductive()` | literally `!failure.is_active()` -- the bus itself becomes non-conductive, the solver excludes it from the equipotential group, and *every* real downstream consumer (FlyByWire Rust or its own TypeScript reading `..._BUS_IS_POWERED`) sees a genuine loss of power |

So these 29 breakers already satisfy the requirement in substance: pulling
one really does open an electrical path in FlyByWire's own solved circuit,
cascading to ARINC/display/FWS consumers exactly as asked, because
`FailureType` *is* FlyByWire's own name for "this source/bus stops
providing power" for these five types specifically. `BreakerDef::gate_kind()`
(new) reports this distinction explicitly rather than leaving it implied:
`"failurePower"` for these 29, vs `"failureSoft"` for a `FailureType` that
is real but does not cut power (see the table below). `is_power_path_
failure()` in `breakers.rs` is the single source of truth for the
classification, cited to the exact line read above.

**What is genuinely soft** (a real, checked FlyByWire effect, just not a
power-path cut) -- audited the same way, one file per group:

| FailureType group | Real site | What it actually does |
|---|---|---|
| `CabinFan`/`HotAir`/`FwdIsolValve`/`BulkIsolValve`/`CargoHeater`/`Fdac`/`Tadd`/`Vcm`/`Ocsm`/`OcsmAutoPartition`/CPIOM-B apps (ATA21, 49 ids) | `air_conditioning/mod.rs` and its local controllers | each is its own dedicated failure flag consumed inside that LRU's own logic, independent of `ElectricalBus`/`receive_power` |
| `FireDetectionLoop(loop, zone)` (ATA26, 12 ids) | `fire_and_smoke_protection.rs::fire_detected_in_loop`/`loop_has_failed` | `!failure.is_active() && self.is_powered && ...` -- `is_powered` is the real bus-power flag (already gated by the loop's own bus, not independently breaker-able); the `FailureType` only disables *detection logic* on top of that |
| `LgciuPowerSupply`/`LgciuInternalError` (ATA32, 4 ids), `GearProxSensorDamage`/`GearActuatorJammed` (18 ids, newly bridged this pass) | `landing_gear`/proximity-sensor code | simulated sensor/actuator damage, not a power cutoff |
| `RadioAltimeter`/`RadioAntennaInterrupted`/`RadioAntennaDirectCoupling` (ATA34, 9 ids, 6 newly bridged this pass) | `navigation.rs` | simulated receiver/antenna fault |
| `ROLLOUT`/`FCU 1`/`FCU 2`/`PRIM 1-3`/`SEC 1-3`/`FCDC 1`/`FCDC 2` (`COMPUTER_FAILURES`, 11 ids) | FlyByWire's C++ side, `FailuresConsumer.isActive` (not this plugin's Rust electrical model at all) | the computer's own self-declared failed state; there is no Rust-side bus/power concept for these computers to hook |

Converting any of these to a genuine power-path breaker needs a new
`breaker_id`/`breaker_closed` field on its own FlyByWire struct -- the same
pattern already live for the four electric hydraulic pumps and the
autobrake solenoid (`patches/fbw-rust/breakers.patch`, from an earlier
pass) -- which needs a `patches/fbw-rust` change *applied* to
`D:\fbw-aircraft`'s working tree (Cargo builds `systems`/`a380_systems`
directly from that path; there is no vendored copy a patch file alone
changes anything for).

### 2. What blocked scaling the mechanism out this session

Checkpoint 1 (mechanism + 5 consumers) was attempted for `TransformerRectifier`
(all 4: TR1/TR2/TR-ESS/TR-APU share one struct, one patch) and `Generator`
(all 4 VFGs, one struct, one patch) -- the same `breaker_id`/`breaker_closed`/
`read()` pattern as the existing pump/solenoid patch, inserted at
`transform()`/`should_provide_output()`. The patch was drafted, diffed, and
verified to `git apply --check` cleanly, but applying it for real (`git
apply`, the same mechanism every existing `patches/fbw-rust/*.patch` was
applied with) was **blocked by this session's sandbox** ("Modify Shared
Resources") -- confirmed twice, once for `git apply` and once for a
follow-up read-only `grep` run immediately after it, both against
`D:\fbw-aircraft`. Unlike the pump/autobrake-solenoid patch (which this
session found *already applied* in the working tree from an earlier pass
and only needed to record), this session could not make a new FlyByWire
Rust change live at all.

Given that hard limit, and given the audit in section 1 above already
covers 29 of the highest-value breakers (every electrical source and every
bus-tie) for free, the pragmatic call was: don't chase a blocked mechanism
further; spend the remaining time on what *is* achievable purely inside
`fbw-xp-systems` (the systems.cfg absorption below, and the catalogue
expansion), and leave the TR/GEN power-path conversion as a **drafted,
unapplied** patch for the lead to apply -- once applied, flipping
`breakers.rs`'s `tr-*`/`gen-*` entries from `d(...)` (failures) to
`plugin_var: Some("ELEC_TR<n>_BREAKER_OPEN")`/`"ELEC_GEN<n>_BREAKER_
CLOSED")` is the only other change needed; no mechanism work. The same
per-consumer pattern (find the `receive_power`/`transform`/`should_
provide_output`/`output_potential` site, add a breaker field, AND it in)
is what every future soft-to-power-path conversion in the table above
would need.

### 3. systems.cfg audit: every one of the 154 circuits, kept or dropped

Read: `circuits.rs`'s own consumers, and only those (grepped, not
assumed) -- `fuel.rs`'s `power_circuits` (feeds `fuel_network.rs`'s
`pump_circuits`/`valve_circuits`, which really gate `PumpType::Electric`
and valve open/closed state) and `lights.rs`'s `circuits.powered`/
`any_powered` (lights) plus its by-name wiper handling
(`circuit_number_named("WipersLeft"/"WipersRIght")`). No other module in
the plugin ever calls `Circuits::powered`/`any_powered`/`breaker_closed`.

- **Real, kept, now in the catalogue** (`breakers::absorbed_systems_cfg`,
  128 circuits): 25 `CIRCUIT_FUEL_PUMP` + 60 `CIRCUIT_FUEL_VALVE` (ATA28),
  41 `CIRCUIT_LIGHT_*` (ATA33), 2 wiper circuits (ATA30). Each keeps its
  own real rated wattage from the embedded systems.cfg `Power:` field
  (`CircuitDef::rated_w`, new parsing in `circuits.rs` -- the file's own
  literal number, not a typical/derived table) and its own `Name:` field as
  its display name. `pre_systems` mirrors the catalogue breaker's closed
  state onto `Circuits::set_breaker` every tick, so pulling "sys-2" (the
  catalogue wrapper for circuit.2, a real fuel pump) really opens circuit
  2, which `fuel.rs` already reads for a real effect -- proven by
  `breakers::tests::pulling_an_absorbed_circuit_breaker_really_opens_its_
  systems_cfg_circuit`.
- **Gate nothing, dropped from the Breakers tab** (26 circuits):
  `CIRCUIT_GENERAL_PANEL`, `CIRCUIT_STANDBY_VACUUM` (no A380 vacuum
  instruments), `CIRCUIT_GEAR_MOTOR` (A380 gear is hydraulic, not an
  electric motor), `CIRCUIT_GEAR_WARNING`, `CIRCUIT_NAV`/`CIRCUIT_COM` x3
  each/`CIRCUIT_XPNDR`/`CIRCUIT_MARKER_BEACON`/`CIRCUIT_ADC_AHRS`/
  `CIRCUIT_FIS`/`CIRCUIT_ADF_DME`/`CIRCUIT_AUDIO`/`CIRCUIT_AUTOPILOT`/
  `CIRCUIT_DIRECTIONAL_GYRO_SLAVING`/`CIRCUIT_PITOT_HEAT` (no probe-heat
  consumer exists anywhere -- `icing.rs`, a380_systems' own ATA30 module,
  is an empty stub), `CIRCUIT_PFD`/`CIRCUIT_MFD`/`CIRCUIT_XML` variants
  (Warnings/Alt Field/STBY Indicator/EICAS1/EICAS2/CDU/FCU), `CIRCUIT_
  AVIONICS`, `HotBatteryCircuit`. None of these has a consumer anywhere in
  the plugin or FlyByWire; several duplicate a breaker this catalogue
  already has a *real* gate for under a better name (`FCU` circuit vs.
  the failure-bridged `fcu-1`/`fcu-2`; `HotBatteryCircuit` vs. the
  `DC_HOT1 BUS FEED` bus-tie breaker) -- kept once, under the mechanism
  that actually gates something, not twice.
- **`GET /study/breakers` no longer emits a separate `"source":
  "systemsCfg"` array at all** -- every entry is now `"source":"catalogue"`
  (absorbed circuits included), and the response carries a new top-level
  `"systemsCfgAbsorbed": true` flag so a client can tell the shape changed.
  `study::web::breakers_json_absorbs_systems_cfg_instead_of_listing_it_
  separately` (test) checks both.
- **The in-X-Plane Study window's own Circuit Breakers page**
  (`study::services::breakers`, `PageKind::Breakers`) reads/writes
  `circuits.rs`'s raw `CIRCUIT BREAKER CLOSED:n` datarefs directly,
  independent of this catalogue, and keeps working unmodified for display
  (same dataref `pre_systems` now also writes for an absorbed circuit).
  **Known trade-off, not fixed this session**: for the 128 absorbed
  circuits, that window's own pull/reset button is overridden the next
  tick by the catalogue's mirror (the web app's Circuit Breakers tab is now
  the controlling UI for those numbers); for the other 26 dropped ones,
  nothing changed, since this catalogue never touches them.

### 4. New JSON field

`"gates"` (string, every catalogue entry, `BreakerDef::gate_kind()`):
`"pluginVar"` | `"circuit"` | `"failurePower"` | `"failureSoft"` | `"none"`
(see section 1). Purely additive -- every existing field is unchanged, per
the UI agent's "only add fields" constraint.

### 5. Catalogue growth this session

| Addition | Count | Mechanism |
|---|---|---|
| Gear/door proximity sensors (`GearProxSensorDamage`, ATA32) | 12 | `failureSoft` (already-registered, previously unbridged failure ids) |
| Gear/gear-door actuators (`GearActuatorJammed`, ATA32) | 6 | `failureSoft` |
| RA antenna interrupt/direct-coupling (ATA34) | 6 | `failureSoft` |
| Absorbed systems.cfg fuel pumps/valves (ATA28) | 85 | `circuit` |
| Absorbed systems.cfg lights (ATA33) | 41 | `circuit` |
| Absorbed systems.cfg wipers (ATA30) | 2 | `circuit` |
| Removed (fabricated, no real gate) | -2 | FWS 1/2, see "Left out, not faked" below, unchanged from the prior pass |

Catalogue total: 111 (prior pass) - 2 (FWS removed) + 18 (gear/door) + 6
(RA antenna) + 128 (absorbed) = **261 breakers**, spanning ATA21, 24, 26,
27/22, 28, 29, 30, 32, 33, 34.

### 6. Requested chapters not reached this session

The user's chapter list (23 comms, 25 cabin, 28 fuel-beyond-circuits, 30
ice protection beyond wipers, 31 CDS/FWS/recorders/clocks, 34 full nav
sensor suite, 35 oxygen, 36 pneumatic, 38 water/waste, 42 IMA/AFDX, 45/46
CMS/OIS, 49 APU, 52 doors, 70-80 engines/FADEC, EHA/EBHA) is **not**
covered beyond what ATA28/30/32/33/34 picked up above. Reconnaissance
before the time limit found real, sourced candidates for several -- ATA36
(`CoreProcessingInputOutputModuleA`/`EngineBleedAirSystem`/`CrossBleedValve`
in `a380_systems/pneumatic.rs`, all with real `powered_by`/`receive_power`
and no existing failure id, needing the same blocked plugin_var-patch
mechanism), ATA42 (`avionics_data_communication_network.rs`'s 16 real,
named, real-bus-cited AFDX switches and ~26 CPIOM/IOM units -- already
covered indirectly by the existing DC1/DC2/DC_ESS/108PH bus-tie breakers,
same situation as FWS, no independent per-unit breaker modelled), ATA49
(`Pw980StartMotor`, `pw980.rs`, real `Sub("49-42-00")` bus, `is_powered`/
`receive_power` already real) -- none could be wired without either the
blocked FBW-Rust-patch mechanism or risking a fabricated `"circuit"`/
`"failurePower"` claim. `EHA/EBHA` remains infeasible for the reason
`docs/physics/fluids.md`'s "Left as-is" section already gives (no
per-instance identity to hook). ATA25/38/45/46 turned up no FlyByWire
electrical model at all in the time available; adding "gates":"none"
entries for these was deliberately not done rather than guess at real A380
breaker names/panel identities without a source to check them against.

### 7. Late addition: EGPWC (TAWS) power path, plus a new `pending_patch` field

With the extended time, one more consumer power-path patch was drafted and
verified: `EnhancedGroundProximityWarningComputer` (`enhanced_gpwc/mod.rs`,
ATA34 TAWS) -- real `powered_by`/`receive_power` on AC_ESS
(`a380_systems/lib.rs`'s own instantiation), no existing failure id.
Same pattern as the pump/solenoid patch: `breaker_id`/`breaker_closed`
fields, `read()`, AND-gated into `receive_power`. Saved at
`patches/fbw-rust/power-path-pending-egpwc.patch`, `git -C /d/fbw-aircraft
apply --check`-verified clean, **not applied** (same sandbox block as
TR/GEN). `BreakerDef` gained a `pending_patch: Option<&'static str>` field
for exactly this state -- a real, sourced consumer with a drafted-but-
unapplied patch -- reported as `"gates":"pluginVarPending"` rather than
silently claiming a live `pluginVar` gate it doesn't have yet. The `egpwc`
catalogue entry uses it. Once applied, flip its `plugin_var` to
`Some("ELEC_EGPWC_BREAKER_OPEN")` and clear `pending_patch`.

Further consumer patches investigated but not completed, for the record:
`ElectroPneumaticValve` (fbw-common `pneumatic/valve.rs`, real
`is_powered`/`powered_by`, used 12x for engine bleed HP/PRV/fan-air valves
plus 2x pack flow valves) and `CoreProcessingInputOutputModuleA`/
`Pw980StartMotor` -- all real, all `is_powered`/`receive_power`-gated, but
their own constructors take no `InitContext` (only their parent does), so
a breaker variable needs threading a new parameter through multiple
call-site signatures rather than a single-file, single-call-site change --
judged too large to author safely without compiler feedback in the
remaining time. Good next-session candidates, in priority order:
`ElectroPneumaticValve` (highest leverage, 14 consumers via one struct),
then `Pw980StartMotor` (ATA49), then `CoreProcessingInputOutputModuleA`/
`CrossBleedValve` (ATA36).

### Time-box note

This checkpoint stopped on an explicit coordinator time limit while eight
other agents work concurrently on the same `D:\fbw-aircraft` checkout
(hydraulics, air/bleed/pressurisation, fuel, gear/brakes, fire+APU,
ice/rain, cockpit control bindings, converter textures) -- `cargo build`
hit one transient, external compile error mid-session
(`fbw-common/.../hydraulic/mod.rs`, `EngineDrivenPump` missing
`cavitation_id`) from one of those concurrent edits, unrelated to this
module; it cleared on its own by the final build/test pass below.

Code:
- `src/breakers.rs`: the catalogue (`BreakerDef`, `catalog()`), live state
  (`Breakers::new`/`apply_requests`/`pre_systems`/`post_systems`/
  `snapshot`), and the pull/reset request queue
  (`request_pull`/`request_reset`/`request_reset_all`).
- `src/lib.rs`: `breakers: breakers::Breakers` field, built alongside
  `circuits`; `apply_requests` at tick start, `pre_systems` before the
  systems tick, `post_systems` after it (same tick placement as
  `physics::electrical::CircuitProtection`, `docs/physics/electrical.md`
  section 6).
- `src/study/web.rs`: `breakers_json()` (`GET /study/breakers`) appends the
  catalogue (`"source":"catalogue"`) after the existing `systems.cfg` list
  (`"source":"systemsCfg"`); `apply_action` (`POST /study/action`) accepts
  `"pullBreaker"`/`"resetBreaker"` (by the catalogue's own string `id`) and
  `"resetAllBreakers"`, queued through the same `Breakers` request queue
  `apply_requests` drains on the plugin's own thread.
- FBW patch (new breaker gates for the two consumers with no existing
  failure id to bridge): `D:\fbw-aircraft\fbw-common\src\wasm\systems\
  systems\src\hydraulic\electrical_pump_physics.rs`,
  `D:\fbw-aircraft\fbw-a380x\src\wasm\systems\a380_systems\src\hydraulic\
  autobrakes.rs`. Diff saved at
  `D:\fbw-xp-systems\patches\fbw-rust\breakers.patch`.

## The three gating mechanisms

Every catalogue entry's *effect* is a real gate on a consumer the plugin or
FlyByWire's own Rust systems already model -- never a cosmetic toggle with
no downstream effect -- in one of three ways:

1. **`failures`** (106 of 111 entries): bridges to a
   [`crate::failures::FailureType`] FlyByWire's own systems crate already
   consumes for a real physical effect, verified by reading the
   `Failure::new(FailureType::...)`/`is_active()` call site for every group
   below, not assumed. Pulling "TR 1" calls `failures::set_active(24_000,
   true)`, and `transformer_rectifier.rs`'s own `if !self.failure.is_active()
   && input.is_powered()` really stops that TRU converting AC to DC. A
   breaker's *rating* still comes from this module's own real-consumer-
   demand derivation below, independent of (and finer-grained than) the
   failure system, which has no notion of current at all.
2. **`plugin_var`** (5 entries): consumers with no existing FlyByWire
   failure id at all get a *new* breaker gate patched directly into
   FlyByWire's Rust (`patches/fbw-rust/breakers.patch`): the four electric
   hydraulic pumps (green A/B, yellow A/B -- `ElectricalPumpPhysics::
   receive_power`/`read`) and the autobrake knob-disarm solenoid
   (`A380AutobrakeKnobSelectorSolenoid::receive_power`/`read`). Pulling
   these writes the named simulation variable to 0; FlyByWire's own
   `SimulatorReader` reads it every tick (a registered simulation variable,
   not a Rust global, as the remote/out-of-process systems host requires)
   and ANDs it into the consumer's own `is_powered` check, so it stops
   drawing current for real -- see "Verifying the patch" below.
3. **`panel_node`** (5 entries): for the subset of `circuits::
   PANEL_CB_NODES` whose label has a defensible, checked correspondence to
   one of the above (`CB_TR1` -> `TransformerRectifier(1)`, `CB_LGCIS1` ->
   `LgciuPowerSupply(Lgciu1)`, ...), this module reuses that node's
   *existing* `CIRCUIT BREAKER CLOSED:<panel_number>` dataref (already
   registered, default closed, by `circuits::Circuits::new`) as the
   breaker's storage, instead of registering a second one -- so if the
   cockpit model's own `CB_*` geometry is ever wired to a click handler, it
   lands on the exact same state this catalogue's Study-panel entry already
   drives. Every other `PANEL_CB_NODES` label had no checked correspondence
   to a modelled consumer and is left as-is (reported, not guessed).

Breakers with no physical panel position get a new synthetic circuit number
in `breakers::EXTRA_BASE`'s range (20,000+), stored the same way
`circuits.rs` stores a real circuit's breaker (`CIRCUIT BREAKER CLOSED/
CURRENT/TRIP CAUSE:n`), so the Study panel's existing polling/JSON
conventions keep working unchanged.

Thermal/magnetic trip physics are **not** reimplemented here: both
`breakers.rs` and `physics::electrical::CircuitProtection` call the same
`crate::physics::electrical::trip_step` curve (I²t thermal accumulator,
`K = 30`; magnetic instant trip at 10x rated), so there is exactly one trip
curve in the whole plugin, applied to a wider set of breakers
(`docs/physics/electrical.md` section 6 has the full curve derivation).
`post_systems` (called once per tick, after the systems tick) evaluates
every catalogue breaker's *real, modelled* rated current against that curve
and opens it on a thermal or magnetic trip; `pre_systems` (called before the
systems tick) then bridges the resulting closed/open state onto the
consumer's `failures`/`plugin_var` effect for that tick to act on -- the same
one-tick lag `physics::electrical::CircuitProtection` already documents
relative to `fuel.rs`/`lights.rs`.

## Rating derivation (per ATA chapter, 111 breakers total)

"Don't fake values" governs every figure below: where FlyByWire's own Rust
already computes a real number for a consumer (a generator's rated apparent
power, a pump's `ELECTRIC_PUMP_MAX_CURRENT_AMPERE`, a bus's own
`FlightPhasePowerConsumer` peak wattage), that number is cited and used
directly ("real, FBW-sourced" below). Where no FBW or public A380-specific
figure exists for a consumer class -- true for the majority of small
avionics LRUs, valve actuators and fans, since FlyByWire's systems crate
models their electrical *bus feed* but not an individual wattage --  a
typical/derived large-transport-aircraft figure is used and labelled
**typical/derived**, the same allowance `docs/physics/electrical.md`
section 6 already used for `physics::electrical::rated_watts`'s per-circuit
table. No entry invents a number with no stated basis; every `BreakerDef`
carries its `basis` string verbatim into `/study/breakers`' JSON.

### ATA21 -- air conditioning (49 breakers)

Source buses are cited to `a380_systems/air_conditioning/mod.rs`'s own
constructors (e.g. `CabinFan::new(id, ..., ElectricalBusType::
AlternatingCurrent(id))`), read and checked per group, not assumed:

| Group | Count | Rated load | Basis |
|---|---|---|---|
| Cabin recirculation fans 1-4 | 4 | 500 W | Typical large-transport cabin recirculation fan motor (typical/derived); bus per `CabinFan::new` |
| Hot-air valves 1/2, cargo isolation valves/extract fans (fwd + bulk) | 6 | 50-150 W | Typical motor-operated valve actuator / small extraction fan motor (typical/derived) |
| Bulk cargo heater | 1 | 1000 W | Typical cargo-bay heater element (typical/derived); `AirHeater::new(AlternatingCurrent(2))`, FBW's own "`// 200XP4`" comment |
| FDAC 1/2 channels 1/2, TADD channels 1/2 | 6 | 50 W | Generic avionics LRU (typical/derived); bus set per `FullDigitalAGUController::new`/`TrimAirDriveDevice::new` |
| VCM Fwd/Aft channels 1/2 | 4 | 50 W | Generic avionics LRU (typical/derived); bus set per `VentilationControlModule::new` |
| OCSM auto-partition (4 units) + OCSM channels (4 units x 2) | 12 | 50 W | Generic avionics LRU (typical/derived); bus set per `OutflowValveControlModule::new` |
| CPIOM B1-4 applications (AGS/TCS/VCS/CPCS) | 16 | 50 W | Generic avionics LRU (typical/derived); CPIOM bus map, `mod.rs:1334-1337` |

### ATA24 -- electrical (29 breakers)

The one chapter where the rated current is FBW's own real, cited figure
throughout, not typical/derived:

| Group | Count | Rated load | Basis |
|---|---|---|---|
| TR 1/2/ESS/APU | 4 | 200 A | Typical Airbus TRU continuous rating (typical/derived rating figure, but the output impedance/idle voltage it is checked against is already real in `transformer_rectifier.rs`, `docs/physics/electrical.md` section 4) |
| Static inverter | 1 | 135 W / 115 V | **Real, FBW-sourced**: `power_consumption.rs`'s own `AC_STAT_INV` bus demand |
| GEN 1-4 (VFG) | 4 | 150,000 / 0.8 / 115 A | **Real, FBW-sourced**: `alternating_current.rs:393`'s own rated apparent power at the plugin's 115 V nominal (`docs/physics/electrical.md` section 1) |
| APU GEN 1/2 | 2 | 120,000 / 0.8 / 115 A | **Real, FBW-sourced**: `Pw980ApuGenerator::MAXIMUM_LOAD_WATT` |
| Bus-tie/feeder breakers (AC1-4, AC_ESS, AC_ESS_SHED, AC_247XP, AC_GND_FLT_SVC, DC1/2, DC_ESS, DC_247PP, DC_309PP, DC_HOT1-4, DC_GND_FLT_SVC) | 18 | bus-specific | **Real, FBW-sourced** where `power_consumption.rs`'s `FlightPhasePowerConsumer` models that bus (11 of 18); the other 7 (AC3/AC4, AC_247XP, DC_HOT2-4, DC_247PP/309PP) have no aggregate consumer modelled in FBW's own crate at all -- each rated identically to the nearest same-voltage-class bus that *is* modelled (a documented approximation, not a second source; see `bus_peak_w` in `breakers.rs`) |

Pulling a bus-tie breaker bridges `FailureType::ElectricalBus(bus_type)`,
which really zeroes that bus's Kirchhoff-solved potential
(`ElectricalBus::is_conductive` -> `!failure.is_active()` ->
non-conductive -> the connectivity solver excludes it, `electrical/mod.rs`)
-- every downstream consumer on that bus, FBW-Rust or FBW-TypeScript alike
(any code reading `ELEC_<bus>_BUS_IS_POWERED`, e.g. FwsCore.ts, see "Left
out, not faked" below), really loses power, not just the entries this
catalogue lists individually.

### ATA26/31 -- fire detection and warning (12 breakers)

| Group | Count | Rated load | Basis |
|---|---|---|---|
| Fire detection loops A/B x 6 zones (ENG 1-4, APU, MLG BAY) | 12 | 20 W | Typical fire/smoke detection loop controller electronics (typical/derived); bridges `FailureType::FireDetectionLoop(loop_id, zone)`, which `FireDetectionLoop::fire_detected_in_loop`/`loop_has_failed` (`fire_and_smoke_protection.rs`) really consumes; bus per that struct's own `powered_by` |

`SetOnFire`-class failure ids are deliberately excluded: a fire-loop breaker
gates *detection*, not the fire itself, so an "activate fire" effect has no
breaker-representable real-world analogue.

### ATA22/27 -- autoflight and flight control computers (11 breakers)

| Group | Count | Rated load | Basis |
|---|---|---|---|
| ROLLOUT, FCU 1/2 (ATA22, failure ids 22_00x) | 3 | 100 W | Typical flight-control computer LRU (typical/derived: FBW's FCCs are ported from its C++ side and carry no Rust-side wattage) |
| PRIM 1-3, SEC 1-3, FCDC 1/2 (ATA27, failure ids 27_00x) | 8 | 100 W | Same basis; alternating DC_ESS/DC2 across the set, a real Airbus-style redundant-bus feed pattern (no FBW Rust source gives the exact per-computer bus, since the FCCs' C++ side owns that) |

### ATA29/32 -- hydraulics and landing gear (7 breakers)

| Group | Count | Rated load | Basis |
|---|---|---|---|
| LGCIU 1/2 | 2 | 50 W | Generic avionics LRU (typical/derived); bridges `FailureType::LgciuPowerSupply`, reuses `CB_LGCIS1`/`CB_LGCIS2` panel nodes |
| Electric hydraulic pumps: Green A/B, Yellow A/B | 4 | 75 A | **Real, FBW-sourced**: `ELECTRIC_PUMP_MAX_CURRENT_AMPERE`, `hydraulic/mod.rs:1750`; gated by the new `plugin_var` patch (see below), not a `failures` bridge -- no existing failure id covers an individual electric pump |
| Autobrake disarm solenoid | 1 | 2 A | Typical small aircraft solenoid valve (56 W / 28 V, typical/derived); `A380AutobrakeKnobSelectorSolenoid`'s own `DirectCurrent(2)` bus; `plugin_var`-gated, same reason as the pumps |

### ATA34 -- radio altimeters (3 breakers)

| Group | Count | Rated load | Basis |
|---|---|---|---|
| RA SYS A/B/C | 3 | 50 W | Generic avionics LRU (typical/derived); `navigation.rs`'s own `A380RadioAltimeters` bus set (AC1/AC2/AC_ESS) |

## Left out, not faked

The module's own contract is that every entry's effect must be a real gate
on a real consumer; a panel-node placeholder with no `failures`/
`plugin_var` behind it gates nothing and is a fabricated breaker, caught by
`every_rating_is_positive_and_every_bus_resolves`'s own assertion
(`assert!(!def.failures.is_empty() || def.plugin_var.is_some(), ...)`).
Two entries an earlier pass of this catalogue carried -- **FWS 1**/**FWS
2**, reusing the `CB_FWS1`/`CB_FWS2` panel nodes -- were removed for exactly
this reason: FlyByWire's Flight Warning System has no Rust presence at all.
It is `FwsCore.ts` (`fbw-a380x/src/systems/systems-host/CpiomC/
FlightWarningSystem/FwsCore.ts`), which reads bus power directly
(`SimVar.GetSimVarValue('L:A32NX_ELEC_DC_ESS_BUS_IS_POWERED', ...)`/
`..._DC_2_BUS_...`, `FwsCore.ts:2981-2983`) with no per-computer power gate
to hook. Pulling this module's own "DC_ESS BUS FEED"/"DC2 BUS FEED"
bus-tie breakers already cuts that same simvar for real (see ATA24 above)
-- just not independently of the rest of the bus, which is genuinely all a
`CB_FWS1`/`CB_FWS2` breaker could ever be without a new `FwsCore.ts`
SourcePatch (a JS-side change, outside this Rust-only workstream's remit;
`docs/js-build.md` describes that separate pipeline if a future pass wants
to add one). Reported here rather than guessed at, per the same rule that
already left the rest of `PANEL_CB_NODES` unmapped.

Other consumers this catalogue does not reach, for the same reason (no
checked real gate available within this pass's scope): per-actuator EHA/
EBHA electric hydraulic actuators (`VariableSpeedPump`, constructed
anonymously per aileron/elevator/spoiler inside `a380_systems/hydraulic/
mod.rs`, no per-instance breaker-sized identity to hook -- `docs/physics/
fluids.md`'s own "Left as-is" section already documents this for the same
reason), and the Control and Display System's keyboard/cursor control units
(`control_display_system.rs`'s `KeyboardCursorControlUnit`, real
`ElectricalBusType` feeds but no failure id and no natural per-unit
`plugin_var` name beyond what the existing DC1/DC2/DC_ESS bus-tie breakers
already cover for the whole bus).

## Verifying the `plugin_var` patch

`patches/fbw-rust/breakers.patch` adds, purely additively (a build that
never runs under this plugin -- the MSFS build, or any test in either
file, none of which write these variables -- behaves exactly as before):

- `ElectricalPumpPhysics`: a `breaker_id`/`breaker_closed` field
  (`ELEC_PUMP_<id>_BREAKER_OPEN`, default closed), a new `read()` that
  reads it, and `receive_power` ANDing it into `is_powered` and zeroing
  `available_potential` when open.
- `A380AutobrakeKnobSelectorSolenoid`: the same pattern,
  `ELEC_AUTOBRAKE_DISARM_SOLENOID_BREAKER_OPEN`.

Verified two ways: `breakers::tests::pulling_a_plugin_var_breaker_writes_
zero_and_reset_writes_one` (the plugin's own side: pulling writes 0, reset
writes 1) and by construction against the live `D:\fbw-aircraft` working
tree (the consumer side: `git diff` on both files, confirmed to match
`breakers.patch` hunk-for-hunk before this file was written, since Cargo
builds `systems`/`a380_systems` directly from that path -- there is no
vendored copy to fall out of sync with).

## Study panel wiring

`GET /study/breakers` (`breakers_json`) emits the catalogue after the
`systems.cfg` list, tagged `"source":"catalogue"`, carrying `id`, `name`,
`ata`, `chapter`, `bus`, `ratingA`, `currentA`, `closed`, `trip`,
`consumers`, `basis` -- the last two only on catalogue entries, since a
`systems.cfg` circuit has neither. `POST /study/action` accepts
`{"kind":"pullBreaker","id":"tr-1"}`, `{"kind":"resetBreaker","id":"tr-1"}`
(both reject an unknown catalogue id) and `{"kind":"resetAllBreakers"}`,
queued the same way every other Study action is (`breakers::request_pull`/
`request_reset`/`request_reset_all` push onto a `Mutex<Vec<Action>>`
`Breakers::apply_requests` drains at the next tick's start, on the plugin's
own thread -- never touched directly from the panel's HTTP thread).

## Tests

`src/breakers.rs`'s own `#[cfg(test)]` module: catalogue invariants (unique
ids, positive+finite ratings with a resolvable bus, every entry lists at
least one consumer and a non-empty basis, every entry gates something,
unique failure ids, every bridged failure id actually registered in
`failures.rs`, every `panel_node` a real `PANEL_CB_NODES` entry, catalogue
size > 100), plus behavioural tests: pulling/resetting a `failures`-bridged
breaker (`tr-1` -> failure 24000) and a `plugin_var`-bridged one
(`hyd-epump-ga`), a steady rated load never tripping across 600 ticks vs. a
manual pull holding across ticks, `resetAllBreakers` closing every pulled
breaker, and the snapshot carrying every field the Study panel's JSON
needs. `src/study/web.rs`'s own tests: `breakers_json` lists exactly the
`systems.cfg` circuit count plus the catalogue count (by `source`), and
`apply_action` accepts `pullBreaker`/`resetBreaker`/`resetAllBreakers` and
rejects an unknown catalogue id or a missing `id`.

## Left as-is / follow-on

- The `ata`/`chapter` tag drives the Study panel's grouping only; it does
  not change which failure id or `plugin_var` a breaker bridges.
- No new thermal/magnetic curve was written (reuses `physics::electrical::
  trip_step`, see above); a future pass tuning that curve tunes it for
  every breaker in the plugin at once, catalogue and `systems.cfg` alike.
- `EXTRA_BASE`-numbered breakers (all but the 5 `panel_node` ones) have no
  physical cockpit-panel position at all yet; wiring one to an actual
  clickable 3-D breaker requires the same cockpit-model geometry work
  `PANEL_CB_NODES`' own unmapped labels are already waiting on, not
  anything in this module.
- patches/fbw-rust/breaker-open-polarity.patch (after the power-path patches): the breaker variables are ..._BREAKER_OPEN, 0 = closed, so an unwritten variable leaves the consumer powered.

## Actual current + thermal ambient (real-current-and-trips pass)

**Problem found and fixed**: `Breakers::post_systems` (`src/breakers.rs`)
computed every breaker's current as `rating_a * bearing_overcurrent_
multiplier(id)` -- a synthetic estimate derived from the breaker's own
*rating*, not any consumer's real draw. A steady load at exactly rated
current could never trip (a test asserted exactly that), and the only
"overload" the model could produce was a hand-picked multiplier constant
for four breakers (`bearing_overcurrent_multiplier`), not a real current
reading. `docs/analysis` never audited this because the catalogue's own
JSON already labelled these currents as an estimate (`basis` string) --
the estimate just had no real-current escape hatch even where FlyByWire
already publishes one.

### The real-current contract

`real_current_var(id: &str) -> Option<&'static str>` (`src/breakers.rs`,
next to `bearing_overcurrent_multiplier`) is the single place a breaker
opts into a real, live-drawn current instead of the synthetic estimate.
`Breakers::new` resolves it to a `VariableIdentifier` once
(`Live::real_current_id`); `post_systems` reads it every tick when present
and only falls back to the synthetic estimate when it is `None`.

Today, wired: the 4 electric hydraulic pumps --
`A32NX_HYD_{GA,GB,YA,YB}_EPUMP_CURRENT`, FlyByWire's own
`ElectricalPumpPhysics::current_id` (`fbw-common/.../hydraulic/
electrical_pump_physics.rs`) -- a real PID-controlled motor current derived
from `resistant_torque` (section pressure x pump displacement, plus an
`overheat_resistant_torque_factor` that already answers a seized/dragging
bearing or cavitation). No FBW patch was needed: this dataref already
existed, unpublished-to-the-breaker-model until this pass.

**Contract for other agents adding real current** (e.g. the motor-fault
agent driving bearing wear/seizure, cavitation, fan obstruction, actuator
binding, winding breakdown, chafed wiring, fluid ingress, loss of cooling):
publish your consumer's real drawn current as its own FlyByWire dataref
(any name; `ELEC_<CONSUMER>_CURRENT_A` or the consumer's own natural name,
e.g. the pump's `_EPUMP_CURRENT` above), then add one `real_current_var`
match arm here mapping the breaker id to that dataref name (with the
`A32NX_` prefix FlyByWire's `InitContext` gives every registered var --
see `study/hyd.rs`'s own `A32NX_HYD_...` reads for the established
pattern). No other file changes: `Breakers::new`/`post_systems` pick it up
automatically. Every breaker with no `real_current_var` entry keeps the
old synthetic `rating_a * bearing_overcurrent_multiplier` estimate,
audited and left that way deliberately (not invented as a measurement) --
`docs/physics/breakers.md`'s own audit found no other consumer in
FlyByWire's Rust with a *per-consumer* (as opposed to per-bus-aggregate)
published current.

### The rating and the current must be on the same basis

A published current may only be fed to the trip curve if the breaker's
`rating_a` was derived the same way. This is not a formality -- it is the
one mistake this section has already made in flight.

The absorbed `systems.cfg` fuel pumps were wired to `fuel.rs`'s real
`FUEL_PUMP_CURRENT_A:<n>`, which is the pump's delivered hydraulic power at
`FUEL_PUMP_VOLTAGE_V` = 115 V, the AC motor bus. Their `rating_a`, though,
comes from `absorbed_systems_cfg`: the `Power:` field over the bus voltage,
and every fuel pump line in that file carries the same `Power:3, 5, 20.0`
placeholder -- MSFS's 5 W, 28 V DC token for a circuit it only needs to
switch, with `; Fuel Pump 5W` written beside it. That is 0.179 A.

A feed pump carrying one engine's cruise fuel flow runs 30 psi at about
1660 gal/h: 361 W delivered, so 4.19 A on the 115 V basis. Against 0.179 A
that is a ratio of **23.4**, past the 10x magnetic pickup, so the breaker
fired the instant the pump carried real fuel -- and its partner tripped
straight after it, picking up the flow. Both FEED 4 pumps went that way in
flight.

The test that shipped with the wiring wrote its current as `rated * 0.44`,
so it assumed the very compatibility that does not hold and could not catch
it. Its replacement,
`breakers::tests::a_fuel_pump_carrying_its_design_flow_keeps_its_breaker`,
asserts the scales instead: no `sys-<n>` breaker may take the real-current
branch, and a pump at its measured design point keeps its breaker for an
hour.

So, before adding a `real_current_var` arm, check two things:

1. **Same voltage.** The dataref's amps and the rating's amps must be on
   the same bus. A 115 V AC motor current cannot rate against a 28 V DC
   circuit token.
2. **A real rating, not a placeholder.** `basis` saying "real, from FBW's
   own embedded systems.cfg Power field" means the *provenance* is real; it
   does not mean the number describes the physical consumer. Where the
   consumer's own full-load current is not stated anywhere in FlyByWire or
   the aircraft config, there is no rating to trip against, and the honest
   answer is the multiplier path -- `rated_a` times
   `bearing_overcurrent_multiplier` /
   `published_load_current_multiplier` /
   `published_hydraulic_power_current_multiplier`. Those are *fractions of
   rated*, so every cavitation, wear and hydraulic-load coupling survives
   on whatever basis the rating happens to be on, and nothing is invented.
   Publish the real amps for the Study panel regardless; displaying a
   number is not the same as rating a breaker with it.

### Thermal ambient

`physics::electrical::trip_step_with_ambient(heat, ratio, delta,
ambient_c)` (`src/physics/electrical.rs`) generalises the existing I^2t
curve: a bimetal thermal element trips at a fixed *absolute* temperature,
so folding ambient into the same `ratio^2` term the curve already used --
`thermal_input = ratio^2 + (ambient_c - REFERENCE_AMBIENT_C) /
ELEMENT_RISE_AT_TRIP_C` -- reduces to exactly the old `trip_step` at
`REFERENCE_AMBIENT_C` (25 degC, the standard ambient reference aerospace
thermal-breaker datasheets quote their curve at -- derived/typical, no
A380-specific datasheet is public) and lets a *sub-rated* load (which
alone never adds heat in this curve) still cross the trip threshold once
the bay is hot enough. Cooldown also slows in a hot bay
(`cooldown_scale`), since the same delta-T that drives the element toward
trip is what it sheds to the bay air. `trip_step` (unchanged signature) is
now a thin wrapper calling this at the reference ambient, so every
existing caller/test is bit-for-bit unaffected.

**Contract**: `BAY_<bay>_TEMPERATURE_C` (plugin-internal simulation
variable, not an FBW one), where `bay` is `breakers::bay_for(def)`'s coarse
mapping (`AVIONICS` default; `WING_ROOT` for the 4 electric pumps and
ATA29/32; `CARGO_FWD`/`CARGO_AFT` for the named ATA21 cargo entries).
`Breakers::new` registers and initialises each bay's dataref to
`REFERENCE_AMBIENT_C` so a build with nothing publishing real bay
temperatures yet is inert (identical to the pre-ambient curve). **Another
agent publishes real bay temperatures to these same names**; until then,
or as a fallback for a bay it does not cover, this reads back the ISA
reference. `Breakers::post_systems` reads `Live::ambient_id` every tick and
passes it to `trip_step_with_ambient`; `physics::electrical::
CircuitProtection` (the 154 `systems.cfg` circuits) is not yet wired to
ambient -- left for a follow-up pass, same mechanism.

### Tests (src/breakers.rs `tests` module)

- `real_pump_current_trips_within_its_independently_predicted_i2t_time`:
  writes `A32NX_HYD_GA_EPUMP_CURRENT` to `1.5 * rated` directly (standing
  in for FlyByWire's own publish, since this unit-test binary does not run
  FBW's hydraulic physics) and checks the trip lands within 0.5 s of `30 /
  (1.5^2 - 1) = 24 s`, computed from `THERMAL_TRIP_K`'s own published
  formula independently of the code under test.
- `without_the_real_current_feed_the_same_window_never_trips` (decouple):
  the real-current dataref left unwritten (0) -- the same 40 s window that
  trips a real overload above must not trip, and the published
  `CIRCUIT CURRENT` must read back 0, not a synthetic rated estimate.
- `a_severe_real_overcurrent_trips_the_magnetic_curve_instantly`: `12x`
  rated real current trips within one tick, cause 2 (magnetic).
- `a_sub_rated_load_trips_in_a_hot_bay_at_its_predicted_time_but_never_at_
  reference_ambient`: `0.9x` rated current never trips at 20 degC bay
  ambient (predicted, since `thermal_input < 1` always below rated at any
  ambient <= ~39 degC), then trips near `30 / (0.81 + 35/75 - 1) = 121.6 s`
  at 60 degC bay ambient -- hand-computed from `trip_step_with_ambient`'s
  own published formula.
- `holding_ambient_at_reference_a_sub_rated_load_never_trips` (decouple):
  ambient held at `REFERENCE_AMBIENT_C` -- the same 0.9x-rated load that
  trips in a hot bay above runs 3600 ticks with no trip, proving the effect
  is the ambient term, not something else in the curve.

### What is not done this pass

- `physics::electrical::CircuitProtection` (154 `systems.cfg` circuits)
  still uses the old ambient-free `trip_step` and its own `fault_multiple`
  test-injection current, not a real per-consumer draw -- out of scope for
  this pass's time budget, same `trip_step_with_ambient` mechanism applies.
- The magnetic pickup multiple (`MAGNETIC_TRIP_MULTIPLE = 10x`) was left
  unchanged rather than recalibrated to a specific cited locked-rotor
  multiple (typically 5-7x for an induction/PMSM motor) -- a magnetic
  threshold *above* locked-rotor is actually the realistic design (locked
  rotor trips thermally, not instantaneously, avoiding nuisance trips on
  motor inrush), so this was a deliberate no-op, not an oversight, but it
  was not independently re-derived/cited this pass.
- `bay_for`'s ATA-chapter-to-bay mapping is coarse (4 buckets) and
  publishes nothing itself; a bay-temperature-publishing agent should
  write after `Breakers::new` runs (construction order across modules is
  not controlled here) so its first real reading is not overwritten by the
  reference-ambient default.
- Tasks 3 (bus feeder breakers replacing `FailureType::ElectricalBus`) and
  4 (two breakers sharing a bus) from the brief were not reached this pass.
