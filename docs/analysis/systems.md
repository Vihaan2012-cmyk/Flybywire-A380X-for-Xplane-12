# Stage 2 analysis: systems

Scope: electrical, APU, engine systems interface, fire, oxygen, lights, communications; hydraulics,
gear/LGCIU, brakes/anti-skid/autobrake, steering; flight control actuators, SFCC/flaps/slats;
pneumatics/bleed, air conditioning, pressurisation, ice and rain protection; fuel (every tank, pump,
valve, transfer, jettison, trim, CG control, fuel temperature); ADIRS, radio altimeters, navigation
receivers, TCAS, transponder, EGPWC; payload, doors, start states. Files: sensors.rs, aspects.rs,
correctness.rs, handling/, flight_controls.rs, fuel*.rs, radios.rs, doors.rs, weight_balance.rs,
mapdata/, and lib.rs's input mapping.

`docs/cl650-reference.md` does not exist yet, so this analysis uses FBW's own source/comments and
public A380 references as the yardstick, as the brief allows.

## Method and a finding about the codebase itself

This area is unusually far along. `sensors.rs` carries an exhaustive test
(`every_simulator_variable_the_systems_read_is_accounted_for`, sensors.rs:563-591) that builds the
whole `A380` against a recording registry and fails the build if any MSFS simulator variable the
systems register has no plugin source — so "input silently left at zero" is largely structurally
prevented for the core physics loop already. `aspects.rs`, `handling/aspects.rs`, `flight_controls.rs`,
`fuel.rs`/`fuel_network.rs`/`fuel_transfer.rs`, `doors.rs`, `weight_balance.rs`, `prim.rs`, `failures.rs`
and `mapdata/` are all close ports of FlyByWire's own MSFS glue (`systems_wasm`, `a380_systems_wasm`,
`FlyByWireInterface.cpp`, `LegacyFuel.ts`), each carrying the FBW source line numbers behind every
mapping, with unit tests reproducing FBW's own arithmetic. As a result the gap list below is shorter
and narrower than for a from-scratch port: most of what remains is (a) a short, explicitly documented
list of inputs the port itself says are unavailable (`prim::UNAVAILABLE`), (b) a few X-Plane data
sources that exist but are not yet wired in, (c) systems the real A380 has that FlyByWire's own Rust
crate does not model at all (so no amount of plugin wiring fixes them), and (d) one systemic gap
(lighting) that follows a pattern the plugin has already solved for fuel circuits but not yet applied
to lights.

## Summary (sorted by impact, then effort)

| ID | System (ATA) | Impact | Effort | One line |
|---|---|---|---|---|
| LIGHT-001 | Lights (33) | 4 | M | Cockpit/cabin/exterior lights are wired to X-Plane's own switches only; none follow FlyByWire's electrical bus power, so lights stay lit through bus/generator failures and total elec loss |
| OXY-001 | Oxygen (35) | 4 | XL | No oxygen system anywhere (FBW's crate has none): no crew/pax/portable O2, no mask-drop on rapid decompression, no low-pressure cautions |
| ICE-001 | Ice/rain protection (30) | 3 | S | `AMBIENT IN CLOUD` is permanently unfed, so airframe icing only accretes from measurable precipitation, missing ordinary dry sub-zero cloud icing |
| FUEL-001 | Fuel (28) | 3 | M | Jettison valves are parsed from the network topology but nothing ever opens them; fuel cannot be dumped overboard |
| FUEL-003 | Fuel (28) | 3 | XL (upstream) | Trim-tank transfer is FlyByWire's simplified `LegacyFuel.ts` drain sequence, not the real FQMS's active CG-target scheduling |
| FCTL-006 | Flight controls (27) | 3 | XL (upstream) | PRIM/SEC accelerometers, rate gyros and ISIS inputs are zeroed — an FBW upstream simplification, not a plugin gap, but a real depth ceiling |
| FCDC-004 | Flight controls (27/31) | 3 | S | Autoland warning latch is not gated by the FWS (`// TODO autoland warning is a function of the FWS`) |
| DOORS-001 | Doors (52) | 3 | M (converter) | Upper-deck doors (U1L-U3R) animate nowhere in the converted exterior; FlyByWire's door state is invisible from outside for 6 of 16 passenger/crew doors |
| ICE-002 | Ice/rain protection (30) | 2 | L | Windshield/window heat and wipers are entirely unmodelled, in FBW's crate and the plugin alike |
| FUEL-002 | Fuel (28) | 2 | S | Manual fuel pump ON/OFF/TOGGLE clicks are not routed to the native fuel network; pumps only ever follow circuit power and the transfer logic |
| FUEL-004 | Fuel (28) | 2 | L | No fuel temperature model (no tank temperature, freeze point or FOB LO TEMP) |
| FCDC-001 | Flight controls (27) | 2 | S | `any_aileron_fault` is hard-coded `false`, so BTV's landing-performance-affected logic never sees an aileron double fault |
| FCDC-002 | Flight controls (27) | 2 | S | No source for the speedbrake lever's analog command degrees on the FCDC bus |
| FCDC-003 | Flight controls (27/32) | 2 | S | FCDC steering fault bit is a stub pending a "steering system" that already exists in handling/ |
| FCTL-002 | Navigation/flight controls (34/27) | 2 | S | The PRIMs' own ILS receiver has no DME distance, so PRIM-side localizer-without-DME logic never has real range |
| FCTL-003 | Flight controls (27) | 2 | S | Pitch and rudder trim switch discretes are hard-wired `false` |
| WB-001 | Payload (25) | 1 | S | Only longitudinal CG is written to X-Plane; lateral CG offset is never set (symmetric airframe, ~1 lb effect) |
| COMM-001 | Communications (23) | 1 | XL (upstream) | No HF radios or SATCOM anywhere; FBW's own systems-host only models VHF |
| COMM-002 | Communications (23) | 1 | XL (X-Plane limit) | X-Plane has no third COM radio; COM3 lives only in the plugin's own state with no receiver physics behind it |
| FCTL-001 | Flight controls (27) | 1 | — (matches default) | Calculated radio receiver option left off, same as FlyByWire's own default |
| FCTL-004 | Flight controls (27) | 1 | — (integration) | FMS LVars read 0 until an FMS (JS runtime) writes them |
| FCTL-005 | Flight controls (27) | 1 | S | FCU value-set/EFIS-panel events have no input path outside spawn initialisation |
| FCDC-005 | Flight controls (27) | 1 | S | `// FIXME inaccurate atm, improve` on the EFCS status 4 spoiler-extended/valid word |
| RA-001 | ADIRS/RA (34) | 1 | — (matches upstream) | All three radio altimeters share one X-Plane AGL reading, same as FlyByWire's own MSFS build |

Fewer than 50 candidates were found with real, actionable evidence in this scope; see "Top candidates"
at the end for the ranked list actually produced by this analysis (24 items, all included above).

## Electrical (ATA 24)

The `A380` electrical crate (`fbw-a380x/.../electrical/`) runs unmodified. Bus power state
(`ELEC_<bus>_BUS_IS_POWERED`) is already consumed correctly by fuel.rs's circuit powering
(fuel.rs:581-591) and by the generator/external power pushbutton aspects (aspects.rs:600-618). No
electrical input is left unfed (sensors.rs's exhaustive test covers it). The one systemic gap is
lighting, which the electrical bus state should gate and does not:

### LIGHT-001 — Lighting circuits are not tied to FlyByWire's electrical buses
- **System / ATA:** Lights, 33.
- **Evidence:** key_events.rs:32 documents that `BEACON_LIGHTS_ON/_OFF, NAV_LIGHTS_SET, LOGO_LIGHTS_SET,
  TAXI_LIGHTS_ON/_OFF, LANDING_LIGHTS_ON/_OFF` go straight to "X-Plane's light switches the converted
  model's lights use" — there is no read of any `ELEC_*_BUS_IS_POWERED` variable anywhere in
  key_events.rs, aspects.rs or extra_backend/lighting_presets.rs. The real bus topology already exists
  in the same `systems.cfg` fuel.rs already parses: `circuit.11`-`circuit.24`
  (`Part_Interior_Cockpit/config/systems.cfg:375-388`) tie nav lights to bus 16
  (AC_GND_FLT_SVC_BUS), beacon/landing/taxi/strobe to buses 2 and 3 (AC_BUS_1/2), each with its own
  `Type:CIRCUIT_LIGHT_*:n#Connections:bus.N` line, exactly the format `fuel.rs::parse_fuel_circuits`
  already parses for `CIRCUIT_FUEL_PUMP`/`CIRCUIT_FUEL_VALVE`.
- **What the real aircraft does:** exterior and most interior lighting is bus-fed; a major AC bus or
  generator failure takes out the lights on that bus, and only battery/DC-ESS-fed emergency lighting
  stays on. FlyByWire's own MSFS build gets this for free because MSFS's generic circuit model powers
  `lightdef`s from the same buses (systems.cfg:357-363 documents the `lightdef.Index`/`circuit.Type`
  relationship).
- **Concrete proposal:** extend `fuel.rs`'s `parse_fuel_circuits`/`bus_power_variable` pattern (or
  factor it out) to also parse `CIRCUIT_LIGHT_*` circuits from the same embedded `systems.cfg`, and gate
  each light command's effect (or the `LIGHT POTENTIOMETER`/on-off state written to X-Plane) by its
  circuit's bus power, the same way `fuel.rs::power_circuits` already gates pump/valve circuits.
- **Realism impact:** 4. **Effort:** M.

### APU (ATA 49)
`apu/` (aps3200.rs, pw980.rs, electronic_control_box.rs, air_intake_flap.rs) is FlyByWire's own,
unmodified. Its pushbuttons and bleed-air interaction are internal L:vars set directly by the cockpit
and read back by fuel_transfer.rs's `ApuFuelAspect` (fuel.rs:487-489, 506-508) and fuel_network.rs's
`set_apu_running`. No gap found: every simulator input the APU model needs (ambient pressure/density/
temperature, N-something feedback) is already in lib.rs's `mapping()` or sensors.rs. The APU fuel valve/
pump aspect is explicitly cross-referenced and tested (fuel.rs: `a_dead_dc_ess_bus_stops_the_apu_pump`).

## Engine systems interface (ATA 70-80, plugin glue)

`fadec.rs` is a full Rust translation of `EngineControl_A380X.cpp`/`Polynomials`/`Table1502`/
`ThrustLimits` (fadec.rs:1-50 doc), including EGT, oil temperature/pressure and fuel flow computed from
FlyByWire's own polynomials (fadec.rs:188-325), not read from X-Plane's generic engine model. TAT reuses
X-Plane's own compressibility-corrected `ice_inlet_heat`-adjacent leading-edge-temperature dataref
(engine_commands.rs:274-278) rather than reimplementing Mach-heating, which is a good, low-risk choice.
No gap found in this file beyond the PRIM/SEC input list under Flight controls below (the FADEC reads
the PRIM buses those gaps describe).

## Fire (ATA 26)

`fire_and_smoke_protection.rs` (a380_systems, unmodified) reads back `ENG ON FIRE:n`, which
aspects.rs's `fire` aspect (aspects.rs:636-644) writes every tick from the systems' own
`ENG_n_ON_FIRE`, itself fed by sensors.rs from `cockpit2/annunciators/engine_fires` (sensors.rs:501-507,
39). This is a complete, tested loop (aspects.rs: `a380_copies_reach_the_systems_names`). No plugin-side
gap found for engine or APU fire detection/handles/bottles: those are FlyByWire's own internal logic,
driven by L:vars the cockpit sets directly. Not checked further: cargo-compartment smoke detection
inputs, which appear to need no MSFS/X-Plane simulator variable beyond what sensors.rs already supplies.

## Oxygen (ATA 35)

### OXY-001 — No oxygen system exists anywhere in scope
- **Evidence:** neither `fbw-common/src/wasm/systems/systems/src` nor
  `fbw-a380x/src/wasm/systems/a380_systems/src` contains an `oxygen` module (directory search for
  `*oxygen*` in both trees returns nothing); nothing in the plugin substitutes for it.
- **What the real aircraft does:** the A380 has a crew fixed oxygen system (regulators, quick-don
  masks, a pressure gauge and low-pressure ECAM caution), a passenger system (chemical generators or a
  gaseous system depending on build, masks that drop automatically above roughly 14,000 ft cabin
  altitude — a threshold the plugin already has, since `PRESS_MAN_CABIN_DELTA_PRESSURE` and the CPCS's
  cabin altitude are already read by other modules, e.g. doors.rs:317), and portable/PBE bottles for
  crew.
- **Concrete proposal:** this is the one system in scope with no FBW source to port at all, so it would
  need a native module (like fuel_network.rs was built for MSFS's fuel system): crew/pax bottle
  pressure depleting with mask flow, an automatic pax mask-drop L:var driven off the systems' own cabin
  altitude output, and an ECAM/EICAS caution hookup. Out of proportion with the rest of this port's
  "run FlyByWire's own code" philosophy, so lowest priority despite the impact.
- **Realism impact:** 4. **Effort:** XL.

## Communications (ATA 23)

`radios.rs` is thorough: BCD16/BCD32/ADF-BCD32 codecs matching FlyByWire's `RadioUtils.ts` exactly
(radios.rs:79-110, tested), the full NAV/VOR/ADF/COM simulator-variable table (radios.rs:21-60), and the
MFD's manual LS tuning. Two gaps are noted for completeness, both low priority:

- **COMM-001** — no HF radios or SATCOM: FlyByWire's own `systems-host/Misc/Communications` only
  implements `VhfRadio.ts` (confirmed by source search); there is no `HfRadio.ts` to port. Real A380
  long-range/oceanic comms use two HF sets and a SATCOM datalink. Impact 1 (rarely used in a study sim
  without real oceanic ops), effort XL and blocked on FBW upstream.
- **COMM-002** — X-Plane has only two COM radios; COM3 (radios.rs:26) "lives here only", i.e. it is
  tuned and displayed but has no X-Plane receiver physics (squelch, range, readback) behind it. Inherent
  X-Plane limitation, not fixable without emulating a receiver from COM1/2's own reception. Impact 1.

## Hydraulics (ATA 29)

`hydraulic/mod.rs` (a380_systems, unmodified) drives gear, brakes, steering and flight controls; all of
it is unmodified FBW code. No plugin-side shortcut found: RAT/emergency generator windmilling needs only
airspeed, which is already mapped (lib.rs `AIRSPEED TRUE`/`AIRSPEED INDICATED`). PTU and engine-driven/
electric pump logic is internal. handling.rs correctly overrides X-Plane's own gearbrake, toe-brake and
wheel-steer physics (handling.rs:531-533, `r.overrides()`) so nothing double-simulates braking or
steering.

## Gear/LGCIU, brakes/anti-skid/autobrake, steering (ATA 32)

This is one of the best-covered areas in the codebase: handling.rs, handling/aspects.rs and
handling/physics.rs are close, tested ports of gear.rs, brakes.rs, autobrakes.rs, nose_wheel_steering.rs
and body_wheel_steering.rs, action for action, debounce for debounce (handling/aspects.rs:1-16). Wheel
RPM :1/:2 matching only the body gear (not wing gear :3/:4) is FlyByWire's own MSFS quirk, reproduced
correctly (hydraulic/mod.rs:2076,2083 "Should be WHEEL RPM:3 ... but MSFS has weird definitions"), not a
plugin gap. Anti-skid starts on, matching MSFS's default (sensors.rs:420-426). No gap found.

## Flight control actuators (ATA 27), SFCC/flaps/slats

flight_controls.rs correctly re-spreads FlyByWire's finer-grained aileron/elevator/rudder/spoiler panels
onto the converted .acf's coarser surface sets by span-weighted mean (flight_controls.rs:152-174), with
`FLIGHT_CONTROLS_TRACKING_MODE` respected as FBW's own "stop writing" flag (flight_controls.rs:318-321).
`prim.rs` assembles the FCU/PRIM/SEC/RA/LGCIU/SFCC/ILS/ADR/IR/FQMS/TCAS/AESU buses from the systems'
own variables, matching `FlyByWireInterface.cpp`'s call order exactly (prim.rs:1-21). The remaining gaps
are the ones the port's own author already found and listed:

### FCTL-001..006 — `prim::UNAVAILABLE` (prim.rs:34-46)
| ID | Input | What stands in for it | Note |
|---|---|---|---|
| FCTL-001 | Calculated radio receiver (`CalculatedRadioReceiver.cpp`) | receiver 3's raw sim data, option off | Matches FlyByWire's own default (option off); not a regression |
| FCTL-002 | Localizer distance without DME (cpp:1158) | 0 | The PRIMs' internal localizer-without-DME range logic never has a real distance |
| FCTL-003 | Pitch/rudder trim switches (`SimInputPitchTrim/RudderTrim`) | `false` | A380 uses a sidestick + pedestal trim wheel/knob, not yoke trim switches, so real-world relevance is limited; rudder trim is already applied via `RUDDER_TRIM_SET` elsewhere (key_events.rs:31) |
| FCTL-004 | FMS LVars (`A32NX_FMGC_FLIGHT_PHASE`, speeds, `A32NX_FG_*`, `A32NX_FM1_*`) | 0 until an FMS writes them | Integration dependency on the JS runtime FMS, not a plugin defect |
| FCTL-005 | `A32NX.FCU_SPD_SET/HDG_SET/ALT_SET/VS_SET`, EFIS panel events | -1/no input except FCU init | The FCU panel presumably drives these through variables in the JS instrument rather than sim events; worth confirming with the display/JS engineers rather than assuming a gap |
| FCTL-006 | Vertical/lateral accelerometers, ISIS, rate gyros (cpp:1605-1629) | 0 / zeroed bus | Documented as "as FBW hard-codes": this is an **upstream FlyByWire simplification**, reproduced faithfully rather than introduced by the plugin |

- **Concrete proposal:** FCTL-002 (DME on the PRIMs' own receiver) can reuse the same
  `nav_dme_distance_nm`/`nav_has_dme` datarefs sensors.rs's `Ils` struct already reads
  (sensors.rs:251-252) — a small, self-contained fix. FCTL-003/005 need confirmation from whichever
  engineer owns the FCU/pedestal cockpit click regions (converter) before concluding they are really
  unfed. FCTL-004/006 are not fixable at this layer (FCTL-004 waits on the FMS; FCTL-006 waits on FBW
  upstream).
- **Realism impact:** 1-3 per row (FCTL-002 is the most actionable at 2; FCTL-006 is the most
  significant in principle but not actionable here). **Effort:** S for FCTL-002/005, not applicable for
  FCTL-004/006.

### FCDC-001..005 — extra_backend_fcdc.rs (adjacent to SFCC/flight-control-monitoring scope)
This file is someone else's declared work in progress (systems-coverage.md: "in progress (delegated by
coverage engineer)"), not one of this brief's owned files, but it is evidence-rich and squarely inside
"flight control actuators"/SFCC monitoring scope, so its already-flagged gaps are listed for completeness
rather than re-discovered:
- **FCDC-001** `let any_aileron_fault = false; // FIXME add` (extra_backend_fcdc.rs:757) — BTV's
  `ldg_perf_affected_btv_lost` logic can never see an aileron double fault, understating a degraded
  landing-performance-monitoring case.
- **FCDC-002** `// FIXME no speed_brake_lever_command_deg in prim out bus (where to get it from?)`
  (extra_backend_fcdc.rs:1024) — the FCDC bus's speedbrake-lever analog command has no source.
- **FCDC-003** `// FIXME when steering control system implemented` (extra_backend_fcdc.rs:1007) — the
  note pre-dates handling.rs's steering port; the steering system now exists (handling.rs, handling/
  aspects.rs) and could supply this bit.
- **FCDC-004** `// TODO autoland warning is a function of the FWS` (extra_backend_fcdc.rs:1099) — the
  FCDC's own autoland-warning output is not gated by the Flight Warning System yet.
- **FCDC-005** `// FIXME inaccurate atm, improve` on the EFCS status 4 spoiler word (extra_backend_fcdc.rs:508).
- **Realism impact:** 1-2 each. **Effort:** S each (all are narrow, local fixes once the FCDC engineer
  reaches them).

## Pneumatics/bleed (ATA 36), air conditioning, pressurisation (ATA 21)

`pneumatic.rs` and `air_conditioning/` (cpiom_b.rs, local_controllers/) run unmodified. Every ambient
input they need (temperature, pressure, density, altitude, airspeed) is already in lib.rs's `mapping()`.
`INTERACTIVE POINT OPEN:0`/`:3`, which `air_conditioning/mod.rs:257-258` reads for ground-cart/door-open
interaction, is fed by doors.rs. No gap found.

## Ice and rain protection (ATA 30)

### ICE-001 — `AMBIENT IN CLOUD` is never fed
- **Evidence:** sensors.rs:579 lists `AMBIENT IN CLOUD` under `no_source` ("No X-Plane source: left at
  the dataref's value"), which starts and stays at 0/false. FlyByWire's own icing model,
  `icing_state/mod.rs::is_in_icing_conditions` (icing_state/mod.rs:90-95), requires
  `ambient_temperature() < 0.1 C` **and** `(is_in_cloud() || precipitation_rate() > 0.1 mm)`. With
  `is_in_cloud()` permanently false, icing only ever accretes when `AMBIENT PRECIP RATE` (mapped from
  `sim/weather/region/rain_percent`, lib.rs:449) is non-zero.
- **What the real aircraft does:** structural icing commonly accretes in cold cloud with no measurable
  precipitation (stratus layers, for example); FBW's own model already accounts for this via
  `is_in_cloud()`, it is only the X-Plane side that never sets it.
- **Concrete proposal:** X-Plane exposes `sim/weather/aircraft/cloud_base_msl_m[3]`,
  `cloud_tops_msl_m[3]` and `cloud_coverage_percent[3]` (per-layer, at the aircraft's position,
  DataRefs.txt). Compute "in cloud" as: any of the three layers has the aircraft's MSL altitude between
  its base and top and a coverage above some threshold (e.g. > 0.25, "scattered" or denser), then
  `vars.write_from_xplane` the `AMBIENT IN CLOUD` slot from sensors.rs alongside the other per-tick
  reads.
- **Realism impact:** 3. **Effort:** S.

### ICE-002 — Windshield/window heat and wipers are entirely unmodelled
- **Evidence:** no `window`/`windshield` heat module anywhere in `fbw-common`/`a380_systems` source
  (grep for "window heat"/"windshield" across both trees returns nothing); correctness.rs only proxies
  engine inlet heat and wing/structural deice heat (correctness.rs:50-54, 82-92) — nothing for the six
  cockpit windows or their wipers.
- **What the real aircraft does:** the A380 has electrically heated windshields and side windows
  (anti-ice and anti-fog) with WINDOW HEAT pushbuttons on the overhead panel, and two windshield wiper
  motors.
- **Concrete proposal:** since FBW's own crate has nothing to port, this would be a small native module
  (like the engine-anti-ice/wing-anti-ice aspects already in aspects.rs) driving X-Plane's window-heat/
  wiper-adjacent visuals (defog) from an `A32NX_`-prefixed pushbutton L:var the cockpit sets — mostly
  cosmetic (no flight-model effect), so it is a converter+plugin pairing rather than a systems-only fix.
- **Realism impact:** 2. **Effort:** L.

## Fuel (ATA 28) — every tank, pump, valve, transfer, jettison, trim, CG control, temperature

fuel.rs/fuel_network.rs/fuel_transfer.rs are the most substantial native port in scope: a generic MSFS
fuel-network engine (parsing the real `flight_model.cfg` `[FUEL_SYSTEM]`, tanks/lines/valves/pumps/
triggers/junctions), FlyByWire's own `LegacyFuel.ts` transfer logic ported action for action, and the APU
fuel aspect, all run after the systems tick as FlyByWire's own glue does (fuel.rs:29-31). Electrical
circuit powering for pumps/valves, including the `CIRCUIT CONNECTION ON:n` pushbutton connection
(fuel.rs:393-403, key_events.rs:216-217), is already correctly wired — contrary to
`docs/systems-coverage.md`'s note that it is missing, which appears stale.

### FUEL-001 — Fuel jettison is parsed but never actuated
- **Evidence:** `flight_model.cfg` defines `Valve.57`/`Valve.58` (`JettisonNozzleValveLeft`/`Right`) and
  the lines feeding them (`flight_model.cfg:308-309,414-415`), which `FuelNetwork::from_cfg` parses
  generically along with every other valve/line. Nothing in `src/*.rs` mentions "jettison" at all (grep
  across the whole plugin returns nothing) — no key event opens these valves, and no cockpit switch or
  aspect drives them.
- **What the real aircraft does:** the A380 fuel jettison system lets the crew dump fuel from the wing
  tanks in flight (down to a landing-weight target) for an emergency landing, through two jettison
  nozzles under FQMS control (jettison mentioned in FBW's own EICAS/checklist/FMS-fuel-load code:
  `ata28.ts`, `MfdFmsFuelLoad.tsx`, `FuelPage.tsx`, confirming the UI expects it even though the systems
  crate does not implement it).
- **Concrete proposal:** add a jettison aspect (native, since FBW's Rust crate has no jettison logic to
  port either): a cockpit JETTISON pushbutton/lever L:var opens `Valve.57`/`58` through
  `FuelNetwork::open_valve`, with fuel routed to those valves treated as leaving the aircraft (a new
  "vent to atmosphere" sink in `fuel_network.rs::update`, rather than accumulating at a dead end), gated
  by a simple jettison-limit weight check the FMS fuel-load page already expects.
- **Realism impact:** 3. **Effort:** M.

### FUEL-002 — Manual fuel pump switch clicks are not routed to the native fuel network
- **Evidence:** `fuel_network.rs::handle_key_event` already implements `FUELSYSTEM_PUMP_ON/OFF/SET/
  TOGGLE` (fuel_network.rs:1498-1501), and `fuel_transfer.rs` calls it internally for the transfer
  logic's own pump control (fuel_transfer.rs:689,692). But `Fuel` has no `handle_event` method, and
  `lib.rs::tick`'s external-event dispatch chain only offers events to `radios`, `doors`,
  `extra_backend` and `key_events` (lib.rs:767-770, 783-786) — `FUELSYSTEM_PUMP_*` from a cockpit click
  on the overhead FUEL panel never reaches `fuel.net`.
- **What the real aircraft does:** each tank pump has an overhead pushbutton the crew can select on or
  off manually (normal ops leaves them on/auto and lets the FQMS sequence them, but manual override is
  a real, trained procedure for abnormal fuel configurations).
- **Concrete proposal:** give `Fuel` a `handle_event` that forwards `FUELSYSTEM_PUMP_ON/OFF/TOGGLE/SET`
  (and, if ever wanted, `FUELSYSTEM_VALVE_SET` for manual valve overrides distinct from the engine
  masters) to `self.net.handle_key_event`, and add it to lib.rs's dispatch chain next to the other
  handlers.
- **Realism impact:** 2. **Effort:** S.

### FUEL-003 — Trim-tank transfer is FlyByWire's own simplified MSFS logic, not the real FQMS
- **Evidence:** `fuel_transfer.rs`'s `LegacyFuel` drains the trim tank to the feed tanks on simple
  triggers (`trim_transfers_active_for_feed_tank`, fuel_transfer.rs:171-172,434,452-453) until it is
  empty (`trim_transfer_runs_until_trim_tank_is_empty`, fuel_transfer.rs:871-901) — a port of FlyByWire's
  own `LegacyFuel.ts`, faithfully reproduced, not a plugin shortcut.
- **What the real aircraft does:** the A380 FQMS actively manages fuel in the trim tank and inner tanks
  through cruise to hold the centre of gravity within a target band that trades stabiliser trim drag
  against tank weight, continuously, rather than a one-shot drain.
- **Concrete proposal:** out of scope to fix in the plugin alone without diverging from "run FlyByWire's
  own code": this is an FBW-upstream simplification (FlyByWire has not built a full FQMS CG-scheduler
  for the A380X either). Note for whoever tracks upstream FBW feature requests; not actionable here.
- **Realism impact:** 3. **Effort:** XL, and upstream.

### FUEL-004 — No fuel temperature model
- **Evidence:** no "temperature" handling anywhere in fuel.rs/fuel_network.rs/fuel_transfer.rs (grep
  found none); `fuel_network.rs::set_fuel_density_lbs_per_gal` (fuel.rs:370) is a fixed constant
  (`JET_A_LBS_PER_GAL = 6.699`), not temperature-varying.
- **What the real aircraft does:** the A380 FQMS displays and monitors fuel temperature per tank against
  the fuel's freeze point, with a FOB LO TEMP caution; temperature also affects density (and so range/
  weight calculations) in reality.
- **Concrete proposal:** a low-fidelity model (ambient/TAT-driven tank temperature converging over a
  long time constant, using the already-fed `AMBIENT TEMPERATURE`/TAT) published as new
  `A32NX_FUEL_TEMP_*` variables the SD Fuel page and MFD would need anyway, would be enough to feed
  ECAM/EICAS realistically. No FBW source to lean on, so this is a native addition.
- **Realism impact:** 2. **Effort:** L.

## ADIRS, radio altimeters, navigation receivers, TCAS, transponder, EGPWC (ATA 34)

prim.rs assembles ADR/IR (ADIRS), RA, ILS, FQMS, TCAS and AESU buses from the systems' own output every
tick, matching `updateAdirs`/`updateRa`/`updateIls`/`updateFqms`/`updateTcas`/`updateAesu`
(prim.rs:8-9). `sensors.rs::Ils` is a careful, tested port of `CalculatedRadioReceiver`-adjacent logic
for the PRIMs' own multi-mode receiver on NAV 3, including a fallback nav-database lookup for glide
slope angle when the tuned receiver's own slope dataref is unavailable (sensors.rs:305-362). TCAS
traffic (mapdata/traffic.rs) is fed live from X-Plane's own `sim/cockpit2/tcas/targets/*`, matching
FlyByWire's `JS_NPCPlane` shape exactly (mapdata/traffic.rs:1-14, tested). No gap found in the EGPWC/
terrain path beyond what mapdata's own (out of scope) engineers are already tracking.

- **RA-001** (noted only, not a gap): all three RA buses (`prim.rs` `self.ra[i]`) are read from
  `A32NX_RA_{i}_RADIO_ALTITUDE` — FlyByWire's own `radio_altimeter.rs` instances, each fed from the same
  shared `PLANE ALT ABOVE GROUND`. This mirrors FlyByWire's own MSFS behaviour (MSFS also gives every RA
  instance the same one AGL simulator variable), so it is not a plugin regression, only a ceiling shared
  with the real MSFS build.
- **FCTL-002** (DME distance on the PRIMs' ILS) is listed under Flight controls above since it is part
  of `prim::UNAVAILABLE`, but is equally a navigation-receiver gap; see there for the fix.

## Payload (ATA 25), doors (ATA 52), start states

weight_balance.rs is a careful, tested reproduction of MSFS's own weight-and-balance arithmetic
(`empty_weight` + every payload station + every fuel tank at its own arm, weight_balance.rs:1-37), with
the crew stations correctly excluded from FlyByWire's payload aspect and left at the cfg's fixed weight
(weight_balance.rs:218-223). doors.rs is similarly thorough: all 20 interactive points, the passenger-
door pressure interlock (doors.rs:69-70,152-168, tested), and both `fbw/door/<name>/*` and X-Plane's own
`sim/flight_controls/door_*_N` commands. start_state.rs documents and tests its situation-classification
rule in full (start_state.rs:1-51).

### WB-001 — Only longitudinal CG is written
- **Evidence:** weight_balance.rs:34-37 documents that `cg_offset_x` "can only be set on aircraft with
  stations off the centreline", and the A380's own stations are symmetric but for the 1 lb crew
  stations, so only `cg_offset_z` is written.
- **What the real aircraft does:** real loading is never perfectly symmetric (asymmetric cargo,
  passenger distribution), giving a small but real lateral CG that affects roll trim.
- **Concrete proposal:** low priority given the ~1 lb magnitude with the current .acf; would need the
  converter to define stations off the centreline first, which is a bigger change than this brief's
  scope. Noted for completeness only.
- **Realism impact:** 1. **Effort:** S once stations exist; effectively blocked on the converter today.

### DOORS-001 — Upper-deck doors have no exterior animation
- **Evidence:** doors.rs:34-36: "the converted exterior animates the main deck doors M1L-M5R only; the
  upper deck doors have no clip in it."
- **What the real aircraft does:** the A380's upper deck has its own passenger/crew doors (U1L-U3R
  simulated here) that open independently and are visible from outside.
- **Concrete proposal:** this is the converter's (msfs2xp-aircraft, the lead's) responsibility per
  team.md; reported here as the exact wording to hand over: the door template only clips M1L-M5R
  (Anims_Door.xml:37-148 range), and U1L-U3R (points 10-15) need the same treatment. The plugin side
  (doors.rs) already looks up `fbw/anim/ANIM_DOOR_U1L` etc. and will drive them the moment the clip
  exists (doors.rs:227,293-297).
- **Realism impact:** 2 (cosmetic, but six of sixteen door state changes are otherwise invisible).
  **Effort:** M, converter-side.

## Top candidates (highest impact-per-effort, in this scope)

Ranked by impact-per-effort (S counts higher than M/L/XL for the same impact); items with no realistic
fix at this layer (FBW-upstream-only, integration dependencies, X-Plane hardware limits) are placed
below the actionable ones even where their raw impact is high, since stage 2.5 cannot act on them.

1. **ICE-001** — feed `AMBIENT IN CLOUD` from X-Plane's per-layer cloud datarefs (S, impact 3).
2. **FUEL-002** — route `FUELSYSTEM_PUMP_ON/OFF/TOGGLE` clicks to `fuel.net` (S, impact 2).
3. **FCTL-002** — feed the PRIMs' own ILS receiver's DME distance from the datarefs sensors.rs already
   reads (S, impact 2).
4. **FCDC-001** — wire a real `any_aileron_fault` from the PRIM bus's aileron status words already
   available in extra_backend_fcdc.rs (S, impact 2).
5. **FCDC-002** — source `speed_brake_lever_command_deg` for the FCDC bus from the spoiler handle
   position already read in prim.rs's `SimReadings` (S, impact 2).
6. **FCDC-003** — connect the FCDC steering fault bit to handling.rs's now-existing steering system (S,
   impact 2).
7. **FCDC-004** — gate the FCDC's autoland warning on the FWS's own output once it publishes one (S,
   impact 2-3).
8. **FCDC-005** — fix the EFCS status 4 spoiler word's accuracy (S, impact 1-2).
9. **FCTL-005** — confirm with the FCU/converter owner whether `A32NX.FCU_*_SET`/EFIS events need a
   sim-event path or are already covered by direct variable writes from the JS panel (S, impact 1).
10. **LIGHT-001** — parse `CIRCUIT_LIGHT_*` from `systems.cfg` (reusing fuel.rs's existing parser) and
    gate lighting commands by bus power (M, impact 4 — highest raw impact of anything actionable here).
11. **FUEL-001** — implement fuel jettison (new valve-to-atmosphere sink plus a cockpit switch aspect)
    (M, impact 3).
12. **DOORS-001** — converter fix: animate U1L-U3R (M, impact 2, not this engineer's file but a clean
    hand-off).
13. **FUEL-004** — add a low-fidelity fuel temperature model (L, impact 2).
14. **ICE-002** — add a native window-heat/wiper proxy aspect (L, impact 2).
15. **WB-001** — lateral CG, blocked on the converter defining off-centreline stations (S once
    unblocked, impact 1).
16. **OXY-001** — build a native oxygen system from scratch (XL, impact 4 — high impact but by far the
    largest single item in this scope; sequence after the smaller wins above).
17. **FUEL-003** — real FQMS CG-target fuel scheduling (XL, upstream FBW limitation, impact 3).
18. **FCTL-006** — PRIM/SEC accelerometer/rate-gyro/ISIS zeroing (upstream FBW limitation, impact 3, not
    actionable in this repository).
19. **COMM-001** — HF/SATCOM (XL, upstream FBW limitation, impact 1).
20. **COMM-002** — COM3 receiver physics (X-Plane hardware limit, impact 1, likely permanent).
21. **FCTL-001**, **FCTL-003**, **FCTL-004**, **RA-001** — matches FlyByWire's own MSFS defaults/
    limitations; recorded for completeness, not recommended fixes.

(24 items total across this scope; there is no 25th-50th to add without inventing unevidenced gaps,
which the brief prohibits — "no fake values or behaviours" applies equally to the gap list.)
