# Progress — src/deep/sensors

Area: `Area::Sensors` (6). All items below are code + `#[cfg(test)]` tests, self-contained
(std only, no crate-internal dependencies per `docs/deep/BRIEF.md` hard rule 2), registered
in `registry.rs`. Nothing here is wired into the rest of the crate yet (`mod.rs` is not yet
declared from `src/deep/mod.rs` — that is the lead's integration step).

**Stopped at the lead's 23:29 hard stop.** State at stop: `mod.rs` declares all 15 submodules
that exist on disk (`adr`, `aoa_vane`, `brake_wear`, `discrete`, `engine_sensors`,
`float_level`, `gps`, `ice_detector`, `pitot`, `radio_altimeter`, `registry`, `rng` (private),
`sideslip`, `smoke_detector`, `static_port`, `tat_probe`); `registry.rs`'s `register()` calls
all 25 of its own `register_*` functions exactly once (verified by grep, listed below); nothing
is half-written. Braces/parens balance-checked across every `.rs` file in this directory.

## Third pass (the lead's deferred-items list) — this session's work

- [done] `float_level.rs` — float-type liquid level transmitter, done as its own real model
  (not a relabelled `PressureTransducer`/`temperature_sensor_reading_c`): thermal expansion of
  the hydraulic fluid moves the indicated level with **no** real volume change (GENERIC
  phosphate-ester expansion coefficient), float lag (exact dt-scaled exponential), binding
  (freezes), sender bias, open-circuit fail-safe-to-zero. 7 tests. Registered as the hydraulic
  reservoir **quantity** transmitter (green/yellow, ATA 29) in `register_hydraulic_sensors`,
  alongside the pre-existing pressure/temperature transducers for those same two systems.
- [done] `brake_wear.rs` — brake wear indication with the explicitly requested
  indicated-vs-real divergence: a bound pin/sensor follows only a `(1-binding)` share of true
  wear (exact dt-scaled exponential, literal freeze at `binding>=0.98` to avoid a
  floating-point-tau's imperceptible-but-nonzero drift), so it can overstate remaining brake
  life while the real stack wears to nothing — the dangerous case the brief asked for. Plus
  sender bias and open-circuit-to-zero. 6 tests. Registered per wheel (16 main-gear wheels,
  same population as brake temperature) as `register_brake_wear`, ATA 32.
- [done] Oxygen pressure transducers — `register_oxygen_sensors`, ATA 35 (new chapter):
  crew and passenger gaseous-supply bottle pressure transducers, reusing
  `discrete::PressureTransducer` (a real pressure transducer is exactly what this is — no new
  physics needed). Flagged GENERIC: the passenger system's actual architecture (gaseous vs.
  single-use chemical generators, which would have no transducer at all) is not verified for
  the A380 specifically.
- [done] Cabin pressure sensors (the CPCs' own transducers) — `register_cabin_pressure_sensors`,
  ATA 21 (new chapter): 2 CPCs (GENERIC dual-redundant count) x absolute + differential
  pressure transducer each, reusing `discrete::PressureTransducer`.
- [done] Outside air temperature — `register_oat_probe`, ATA 34: one standby (ISIS) OAT probe,
  reusing `discrete::temperature_sensor_reading_c`, flagged GENERIC (dedicated-probe
  assumption; the A380's actual standby architecture, vs. deriving OAT from a TAT probe, is
  not verified).
- [done] TGT circumferential hot-streak modelling — extended `engine_sensors.rs`: junctions now
  carry a real `angle_deg` (`TgtJunction`), and `tgt_harness_average_c` takes an optional
  `HotStreak { center_deg, width_deg, peak_delta_k }` **plain input** (Gaussian angular
  profile, GENERIC shape) — this directory does not model *why* a hot streak exists (a coked
  fuel nozzle group is the engine_accessories agent's model); it only senses one correctly by
  position, including angular wraparound at 0/360. 5 new tests (localized-vs-average magnitude,
  exact-centre reads near full peak, wraparound, plain-input independence). Existing TGT tests
  updated to the new `TgtJunction`/`Option<HotStreak>` signature.
- [done] Left/right static-port averaging — `static_port.rs`'s new `average_pair` +
  `StaticAveragingLineFaults`: two ports average 50/50 when both healthy (the real,
  documented reason for pairing: cancelling sideslip-driven left/right position error), fall
  back fully to the healthy side if the other is blocked (small-orifice-dominates reasoning,
  same as the existing cabin-leak model), and — the averaging **line** itself being a real,
  separately failable part — a blocked averaging line isolates the two sides (losing the
  cancellation benefit, `degraded: true`) without invalidating either port's own reading. 4
  new tests. Registered as its own component, `register_static_averaging_lines`, ATA 34, one
  per system (4).
- [done] Angle-of-sideslip — `sideslip.rs`: the A380 (Airbus FBW family) has **no physical
  sideslip vane**; sideslip is derived from other sensors. Implemented the standard textbook
  small-angle body-axis sideslip kinematic relation (`beta_dot = a_y/V - r + (g/V) sin(phi)
  cos(theta)`, GENERIC/textbook, explicitly not claimed as the A380's actual proprietary
  algorithm) as an integrator with a slow washout (bounds drift the way a real synthetic
  estimator must). **Not registered** in `registry.rs` — no physical part, no failure of its
  own; its accuracy is entirely inherited from the accelerometer/gyro models
  `src/physics/adirs.rs` already owns. 4 tests.
- [done] **Removed** the earlier per-tank fuel quantity-probe-array and fuel-temperature
  registrations (`register_fuel_probes`/`register_fuel_temperature`, ATA 28) per the lead's
  instruction that a new fuel agent now owns `src/deep/fuel/` end to end, to avoid a
  component-id collision. The reusable capacitance-probe physics
  (`discrete::fuel_probe_indicated_level`/`discrete::capacitance_probe_indicated_level`) stays
  in `discrete.rs` (this directory's own `oil_probe_indicated_level` still uses the shared
  function for engine oil quantity, ATA 79) in case the fuel agent finds it useful, but this
  directory registers zero ATA 28 components/failures now.

## Next most valuable items if this continues

- Wire a real angular position + `HotStreak` source into the engine_accessories agent's
  nozzle-coking model once that exists, rather than the placeholder `Option<HotStreak>` this
  session leaves as a plain, disconnected input.
- CPC pressure sensors here are not yet tied to `src/physics`'s own cabin pressure model (if
  one exists) — check for a collision/overlap the way the fuel hand-off needed.
- The averaging-line model is a 50/50-or-fallback simplification; a fuller pneumatic-network
  model (finite line conductance blending gradually rather than a hard fallback) would be a
  refinement, same category of simplification the leak-to-cabin model already accepts.
- Brake wear pin's `true_remaining_frac` input has no wear-accumulation model behind it in
  this codebase yet (out of this directory's scope, per its own doc comment).

- [done] Pitot probe (heater/heat-balance, drain-hole-dependent blockage behaviour,
  insect/tape blockage, mechanical damage, pneumatic lag) — `pitot.rs` — 9 tests. Drain-open
  vs drain-blocked give the two distinct historically-documented failure signatures (decay to
  static vs. frozen/altitude-coupled).
- [done] Static ports (blockage, leak to cabin, position error vs AoA/Mach) —
  `static_port.rs` — 6 tests.
- [done] AoA vane (heater, icing jam, mechanical stuck, resolver drift, damage) —
  `aoa_vane.rs` — 8 tests. Reuses the pitot's Messinger-style heat-balance approach at vane
  scale (own constants, independently derived).
- [done] TAT probe (heater, icing thermal-lag growth, recovery factor, self-heating error at
  low TAS) — `tat_probe.rs` — 6 tests. Self-heating modelled via a `LOCAL_COUPLING_FRACTION`
  (GENERIC) representing the probe's deliberate thermal isolation between heater and sensing
  junction.
- [done] ADR computation (CAS/Mach/pressure-altitude/TAS/SAT from total/static pressure and
  TAT, using the standard ICAO/FAA compressible pitot-static and standard-atmosphere
  relations) plus a generic median-select 3-way voter/monitor — `adr.rs` — 9 tests.
- [done] Radio altimeters (antenna/transceiver fault, false fixed-offset reading, multipath
  noise scaling with terrain type and height, range limit -> NCD not clamping) —
  `radio_altimeter.rs` — 8 tests.
- [done] GPS receivers (geometry-free HDOP-proxy error model, minimum-4-satellites fix rule,
  jamming reducing effective satellites, gradual spoofing walk-off) — `gps.rs` — 8 tests.
- [done] Discrete sensors: inductive proximity sensor (gap error, stuck near/far, hysteresis),
  capacitance fuel quantity probe (water contamination over-read, open circuit), RTD
  temperature sensor (open/short pegs to range extremes), pressure transducer (drift, stuck)
  — `discrete.rs` — 15 tests.
- [done] Registered every component/failure/ECAM alert above in `registry.rs`
  (`pub fn register(r: &mut Registry)`), per `docs/deep/BRIEF.md`'s "Registering failures,
  components and ECAM alerts" section. No `CATALOGUE.md`/`ECAM.md` were created (the
  CATALOGUE/ECAM-file instruction was superseded by the registry.rs instruction before this
  directory produced either file, so there is nothing to delete).

## Reviewer fix

- [done] `static_port.rs`'s partial-restriction response blended by a fixed per-call fraction
  instead of a dt-scaled exact exponential, so its time constant depended on frame rate (a
  fixed fraction converges faster the more often `step()` is called). Fixed to the same
  `(-dt/tau).exp()` pattern `pitot.rs` uses, with `tau` derived from conductance so a fully
  open port still tracks instantly (`tau == 0`, unchanged behaviour) and a restricted port
  gets a real, step-size-independent lag. Added
  `partial_restriction_response_time_constant_is_independent_of_step_size` as a regression
  test (same elapsed time, chopped into 4 vs. 200 steps, must land within 1 Pa of each other).

## Second pass ("go much deeper" — every individual sensor its own component/failure set)

- [done] New physical sensor models, each a genuinely distinct technology (not a renamed
  copy of an existing one):
  - `ice_detector.rs` — magnetostrictive ice detector (resonant-frequency mass loading,
    detect/deice duty cycle, failed-heater latching, frequency-sensor/probe-damage bias
    false positive/negative) — 6 tests.
  - `engine_sensors.rs` — N1/N2/N3 variable-reluctance speed pickups (air-gap-dependent
    amplitude vs. the EEC's detection floor, open circuit), TGT thermocouple harness
    (per-junction open/drift averaging), vibration pickup (bias, stuck, intermittent
    dropout), fuel flow transmitter (turbine K-factor: bearing wear vs. debris blockage as
    two distinct under-reading causes, stuck rotor) — 16 tests.
  - `smoke_detector.rs` — photoelectric light-scattering smoke detector (sensitivity loss,
    spurious bias, stuck) — 6 tests.
- [done] Generalised `discrete.rs`'s fuel capacitance-probe physics into
  `capacitance_probe_indicated_level` (own/contaminant permittivity parameters), with
  `fuel_probe_indicated_level` as a thin wrapper (field renamed
  `water_contamination_frac` -> `contamination_frac`, `FuelProbeFaults` now a type alias) and
  a new `oil_probe_indicated_level` wrapper for oil-quantity capacitance probes contaminated
  by water (a real, documented failure mode distinct from fuel contamination) — 2 new tests.
- [done] Split `radio_altimeter.rs`'s single `antenna_or_transceiver_fault` into three real,
  separately-failable parts: `transceiver_fault`, `tx_antenna_fault`, `rx_antenna_fault` (any
  one fully failed loses the reading), plus `rx_antenna_degradation` (antenna gain loss,
  physically distinct from `tracking_loop_degradation` but the same noise-raising symptom) —
  2 new tests, existing tests updated to the new field names.
- [done] Split `gps.rs`'s faults into `receiver_fault` and a separate `antenna_fault`
  (physically distinct LRUs, same total-loss effect) plus `antenna_degradation` (acts like
  jamming on the effective satellite count, distinct physical cause) — 2 new tests.
- [done] Rewrote `registry.rs` around a generic `register_instance` helper (a `Counter` per
  ATA chapter + a `FaultSpec` table) so every individual sensor gets its own component and
  failure ids without hand-duplicating near-identical registration blocks per instance. Now
  registers (component counts): pitot x4, static port x8 (4 systems x L/R), AoA vane x3, TAT
  probe x2, ice detector x2, radio altimeter transceiver x3 + tx antenna x3 + rx antenna x3,
  GPS receiver x3 + antenna x3, landing gear proximity x15 (5 legs x uplock/downlock/WOW),
  door proximity x16 (8 door points from `src/sensors.rs`'s own doc comment x open/closed),
  fuel quantity probe arrays x11 (one per tank, GENERIC probe-count parameter scaled by a
  GENERIC tank-size ranking — explicitly not per-probe components, per the lead's own
  instruction to use a GENERIC per-tank count), fuel temperature x11, hydraulic pressure x2 +
  reservoir temperature x2, engine speed pickups x24 (4 engines x N1/N2/N3 x EEC channel A/B),
  engine TGT harness x4, engine vibration pickup x8 (2 locations x 4 engines), engine P30 x4 +
  T25 x4, engine fuel flow transmitter x4, engine oil pressure/temperature/quantity x4 each,
  brake temperature x16 (wing/body main gear wheels only, matching the commonly published
  A380 2-wheel wing / 6-wheel body bogie configuration), tyre pressure x18 (adds the 2 nose
  wheels), smoke detectors x12 (4 cargo + 8 lavatory, both GENERIC counts, documented as
  such), duct temperature x6 (GENERIC representative location set). Total: roughly 190
  components, several hundred per-instance failures.
- [done] Re-ATA'd several sensor categories to their correct real ATA-100 chapters rather
  than the first pass's coarser groupings: ice detectors moved to 30 (Ice & Rain Protection),
  door proximity sensors to 52 (Doors), hydraulic sensors to 29 (Hydraulic Power), engine
  instrumentation to 77 (Engine Indicating)/73 (Engine Fuel and Control)/79 (Oil), smoke
  detectors to 26 (Fire Protection), duct temperature to 36 (Pneumatic).
- [done] Added a `NAV TAT PROBE FAULT` advisory ECAM alert (not present in the first pass);
  updated `NAV ADR DISAGREE`/`PROBE PITOT HEAT FAULT`/`NAV AOA DISAGREE`/`NAV RA 1 FAULT`/
  `NAV GPS 1 FAULT` to `raised_by` the real per-instance failure ids the new registry produces
  instead of placeholder single-component ids.

## New Vars this directory's models would need to publish (not yet wired)

None of these exist in the crate yet; a future wiring pass (or the lead) would add them
alongside a `SimulatorReaderWriter`-style struct like `src/sensors.rs`/`src/physics/adirs.rs`
use, reading each `step()`'s `Output` struct. All prefixed `DEEP_` to avoid colliding with
`src/physics/adirs.rs`'s existing, already-wired `ADIRS_SENSED_<n>_*`/`ADIRS_STUDY_<n>_*`
variables (a separate, parallel sensor model for the same physical systems — see that
module's own doc comment for why two exist):

- `DEEP_ADR_<n>_CAS_KT`, `DEEP_ADR_<n>_ALT_FT` — per-channel ADR outputs (n=1..3).
- `DEEP_ADR_VOTE_DISAGREE` — the 3-way voter's `disagree` flag (0/1), published by whatever
  wires `adr::vote3` across the 3 channels' CAS (or another chosen quantity).
- `DEEP_PITOT_<n>_BLOCKED`, `DEEP_PITOT_<n>_HEATER_FAILED` (n=1..4: ADR 1-3 + standby).
- `DEEP_STATIC_<n>_BLOCKED` (n=1..4).
- `DEEP_AOA_<n>_JAMMED` (n=1..3).
- `DEEP_TAT_<n>_HEATER_FAILED` (n=1..2).
- `DEEP_RA_<n>_VALID`, `DEEP_RA_<n>_AGL_FT` (n=1..3).
- `DEEP_GPS_<n>_VALID`, `DEEP_GPS_<n>_EFFECTIVE_SATS` (n=1..3).

## Existing Vars this directory's registry.rs procedures reference (already published)

- `OVHD_ADIRS_ADR_<n>_PB_IS_ON`, `OVHD_ADIRS_IR_<n>_PB_IS_ON` — confirmed by grepping
  `fbw-common/.../overhead/mod.rs`'s `OnOffFaultPushButton::new_on`
  (`format!("OVHD_{}_PB_IS_ON", name)`) against
  `AirDataInertialReferenceSystemOverheadPanel::new`'s
  `OnOffFaultPushButton::new_on(context, "ADIRS_ADR_1")` etc. in
  `fbw-common/.../navigation/adirs.rs`.

## Known depth limits / follow-ons (next most valuable items if continued)

- The ADR voter (`adr::vote3`) is generic; it is not yet fed by 3 live `PitotProbe`/
  `StaticPort`/`TatProbe` instances driven by a shared `TrueState` the way
  `src/physics/adirs.rs::Adiru` is — that integration (and reconciling which of the two
  parallel sensor models the crate should ultimately keep/merge) is a lead-level decision,
  out of this agent's directory-only write scope.
- Radio altimeter beam geometry (off-boresight terrain slope, antenna installation offset
  overlaid on this directory's terrain-independent AGL truth) is not modelled here — this
  directory's radio altimeter takes true AGL as an input and adds the fault/noise layer only;
  `src/physics/adirs.rs::RadioAlimeterProbe` already does the terrain-boresight raycast this
  would need to combine with.
- Proximity sensor targets (actual gear/door position kinematics) are out of scope; this
  directory supplies only the sensor reading a gear/door model would feed it.
- The TGT thermocouple harness averages a uniform `true_tgt_c`; real circumferential
  temperature spread across the turbine annulus (hot streaks) is a real effect not modelled
  -- `engine_sensors::tgt_harness_average_c`'s doc comment flags this.
  `TgtJunctionFaults` is applied per-junction in code, but `registry.rs`'s per-engine harness
  component necessarily registers *representative* open-circuit/drift failures (one harness
  = many junctions in reality) rather than one component per junction, documented as such.
- Not yet covered: hydraulic reservoir *quantity* sensors (a float/level sensor, not
  pressure or temperature -- deliberately not modelled by misusing
  `discrete::PressureTransducer`/`temperature_sensor_reading_c` under a wrong label), brake
  wear pins, oxygen system pressure transducers (the last is the same
  `discrete::PressureTransducer` model at a different instance/owner, straightforward to add
  next), cargo/lavatory smoke detector *exact* counts (registered as clearly-flagged GENERIC
  representative counts since real counts are cabin-configuration-dependent), and pitot/
  static probe position-error cross-coupling between left/right ports on the same system
  (currently modelled as independent ports rather than an averaged pair).

- [done] Live system — `live.rs` (`live_system() -> Box<dyn deep::live::Area>`), `mod.rs` — `LiveSensors` owns the
  real complement: 4 pitot probes, 8 static ports + 4 averaging lines, 3 AoA vanes, 2 TAT probes, 3 ADRs + the
  3-way voter, 2 ice detectors, 3 radio altimeters with their tx/rx antennas, 3 GPS receivers with their antennas,
  4x2 engine N1 speed pickups and the standby OAT probe. Failure ids are resolved from the real registry by
  (component, model field) — `FaultIndex` — since this area's ids come from a counter, not constants. Publishes
  `DEEP_ADR_VOTE_DISAGREE`, `DEEP_PITOT_1..4_HEATER_FAILED`, `DEEP_AOA_1..3_JAMMED`, `DEEP_TAT_1..2_HEATER_FAILED`,
  `DEEP_RA_1..3_VALID`, `DEEP_GPS_1..3_VALID` (every var this area's ECAM triggers name) plus ~60 Study vars.
  109 of 508 registered failures are consumed; the rest are the discrete instrumentation and the engine sensors
  `Truth` has no quantity for yet — listed in `live.rs`'s module doc together with the `Truth` fields needed
  (true AoA, cabin pressure, radio height, satellite visibility, engine N2/N3/TGT/fuel flow/vibration).
