# Progress — pneumatic_ducts

Coupling agent: engines/APU -> packs, wing anti-ice, engine start, hydraulic reservoir
pressurisation, and leaks -> airframe zone heat. `docs/deep/BRIEF.md` backlog + the
"registering failures/components/ECAM" addendum.

## Lead follow-up increment (real cross-bleed topology + upstream valve stage)

Done, in response to the lead's explicit follow-up instruction:

- [done] **Real L/C/R cross-bleed topology**: removed the single shared manifold
  `DuctSection`; `network.rs` now connects the 4 engine ducts directly via three valves
  matching FBW's own `CrossBleedValve` numbering/topology exactly (left = valve 9, engine
  1<->2; centre = valve 10, engine 1<->4; right = valve 11, engine 3<->4;
  `a380_systems/pneumatic.rs:277-281,448-453`). APU feeds engine 1 directly (matches FBW's
  `apu_bleed_air_valve` target). Packs now have two feed valves each (pack1<-eng1+eng2,
  pack2<-eng3+eng4, matching the real per-pack dual-FCV architecture); hydraulic reservoir
  green/yellow tap engine 1/engine 4 directly, matching FBW's own assignment exactly.
  Isolation now closes an engine's own PR valve **and** every cross-bleed/APU connection
  touching it (not just one "isolation valve to manifold" as before).
- [done] **Upstream HP/IP/PR valve stage**: new per-engine chain, IP8 passive check-valve
  tap (arctan characteristic, cites FBW's own `PurelyPneumaticValve`/`SPRING_CHARACTERISTIC`,
  new `duct::passive_valve_open_fraction`) + HP6 electro-pneumatic valve (cited orifice area)
  into a transfer-pipe volume (cited FBW volume), then a PR/shutoff valve (cited orifice
  area) into the precooler -- fed directly by `EngineBleedInput::ip_port_pressure_pa`/
  `hp_port_pressure_pa` etc. (matching `physics::engine::EngineOutputs`' exact field names
  for a clean future integration point). Regulation is this module's *own* simple
  proportional law (gain + actuator lag, not a copy of FBW's tuned PID), citing the same
  public EASA TCDS E.012 switch-over pressure and 40 psi regulation target FBW's own CPIOM
  software targets. New faults: `hp_valve_stuck`, `pr_valve_stuck`,
  `ip_check_valve_stuck_closed` (registered as failure ids 36/15-17, reusing the ids freed
  by removing the manifold).
- [done] **Structural/wiring damage interface** (item 2): `leak::LeakResult` gained
  `jet_impact_flux_w_m2` -- the same `heat_to_zone_w` energy (never more, tested for
  conservation) re-expressed as a local W/m^2 flux at the jet's own impact spot, zero below
  the jet-impingement onset. `NetworkOutputs::jet_impact_flux_w_m2` aggregates it per zone
  (max across sections, since it is an intensity not an extensive quantity) as the
  interface a future wiring-harness/structural-burn-through damage model consumes.
- [done] **Zone names keyed exactly to `thermal_zones`**: read `deep::thermal_zones::
  topology_a380.rs`'s own `Zone::new(...)` calls and switched every zone key from this
  workstream's own invented names to the exact real ones: `"PylonEngine1..4"`, `"TailCone"`
  (the APU duct run's real zone -- confirmed by reading `thermal_zones::registry.rs`'s own
  placeholder APU-duct-leak fault, which already targets `zones.tail_cone`, not
  `apu_compartment`), `"WingLeLeft"`/`"WingLeRight"`, plus two new non-ODLS zones for the
  consumers that needed one: `"BellyFairingPacks"` (packs) and `"WingGearWell"` (hydraulic
  reservoir pressurisation ducts). `network::ZONE_NAMES`/`ZONE_COUNT`/`ODLS_ZONE_COUNT`
  updated accordingly (9 zones total, first 7 carry a real ODLS loop).
  **Cross-check finding**: `thermal_zones::registry.rs` already registers its own interim
  placeholder pylon/APU-duct-leak failures (`Area::ThermalZones`, ATA 36/49, fixed-reference
  watts, no real pressure/mass-flow model) -- documented in `network.rs`'s module docs as
  the thing a future integration pass should retire in favour of this workstream's real
  `zone_heat_w` output, not resolved unilaterally since it is that other workstream's own
  registered component.
- [done] `registry.rs`/`FAILURES.md` updated: removed the 3 manifold failures, added the 3
  upstream-stage failures (reusing ids 15-17), fixed every zone-name string in ECAM
  Var-name construction and the `ODLS_ZONES` table to the real names above.
- [done] `duct.rs` gained `passive_valve_open_fraction` (+ test) and a doc/test pass;
  `leak.rs` gained `jet_impact_flux_w_m2` (+ 2 new tests); `network.rs` rewritten in full
  with 10 tests covering the new topology (IP-preferred/HP-backup switch-over, cross-bleed
  feeding a non-running neighbour, cross-bleed shut isolates it, PR-valve + cross-bleed
  isolation on ODLS trip, jet flux only in the ruptured zone, WAI, both check-valve cases,
  dt=0 safety).

**Not done / next** (ran out of time before the lead's hard stop, in priority order):
1. `registry.rs`'s pack/hyd-reservoir/WAI failure *effect* prose still says "manifold" in a
   couple of places (cosmetic only -- the model_field/magnitude/ids are all correct and the
   Var names used by ECAM were fixed) -- a quick text pass, not a structural fix.
2. No dedicated leak/rupture fault on the new `transfer_pipe` volume itself (documented as a
   scope simplification in `network.rs`'s doc comment on `UpstreamFaults`) -- could be added
   as a 4th upstream fault if a future pass wants that level of detail.
3. `CATALOGUE.md`/`ECAM.md` were never created (the registry-in-code instruction arrived
   before this agent wrote those, per the coordinator's own message), so there is nothing to
   delete there.
4. Everything above is untested by an actual `cargo build`/`cargo test` (hard rule 2: no
   builds this pass) -- the lead's own build pass is the first real compile check.

## Research before coding

- Read FBW's ported `a380_systems/src/pneumatic.rs` (valve orifice areas/Cd, precooler
  `heat_transfer_coefficient = 180.*2.`, HP/PR valve regulation, hydraulic reservoir
  pressurisation topology, engine starter valve/container) and fbw-common
  `systems/src/pneumatic/mod.rs`/`valve.rs` (`compressible_orifice_mass_flow_rate`,
  `PneumaticContainer`'s ideal-gas charge/discharge relations, `PneumaticContainerConnector`'s
  equilibrium clamp) as the physics/citation basis — all reproduced independently in plain
  `f64` per the self-contained-module rule, not imported.
- **Scope-narrowing findings, all documented in `network.rs`'s own module docs:**
  - `deep::cabin::water.rs` already fully models potable-water pneumatic pressurisation
    (ATA 38) as its own self-contained system — did **not** add a redundant water
    duct/consumer, per "no duplicated logic".
  - `deep::thermal_zones::network.rs` is the authoritative arbitrary-zone thermal graph for
    this push (`ThermalNetwork::inject_heat_w(ZoneId, watts)`); this directory cannot import
    its `ZoneId` (a `Vec` index assigned at that network's own construction, self-contained
    rule), so `leak.rs`/`network.rs` output heat keyed by plain zone-name strings
    (`network::ZONE_NAMES`) for a future integration pass to map onto that network's real
    `ZoneId`s instead.
  - `deep::fire_ice::fire_loops.rs` already covers engine/APU/gear-bay/cargo/avionics *fire*
    detection (ATA 26, 9 zones). Confirmed ODLS (ATA 36, bleed-duct overheat) is a genuinely
    distinct, unbuilt system — different zones (pylon/wing-root/wing-leading-edge duct runs,
    not engine-core/gear-bay/cargo), same general continuous-loop technology, independently
    modelled per the self-contained rule.
  - `docs/physics/ice-protection.md`'s own audit + grep confirmed the A380 **does** use real
    pneumatic (hot bleed air) wing anti-ice (`ice_surface_hot_bleed_air_left_on`/`_right_on`),
    but `a380_systems` has **no** `pneumatic/wing_anti_ice.rs` at all (unlike the A320's public
    one) — `PNEU_WING_ANTI_ICE_SYSTEM_ON` today is only a cockpit-switch mirror with zero
    physical duct/valve/heat behind it. This is the genuine gap item 1's "wing anti-ice" targets;
    the A320's public `wing_anti_ice.rs` (same open-source project) was read for its real,
    citable geometry (47 mm restrictor, ~2 m^3 duct volume, ~22.5 psig target) and control
    concept, reused as GENERIC/representative since no A380-specific figures are public.
  - `deep::apu::power_section.rs` owns the APU's own gas-generator core; this network takes
    the APU's bleed port condition as an external input, exactly like each engine's.
  - `deep::engine_accessories/nacelle` (empty dir, another agent's area) — deliberately did
    **not** add engine cowl/nacelle anti-ice; only wing anti-ice, which is this task's own
    listed item and outside that agent's likely nacelle-systems scope.

## Done

- [done] duct.rs: `orifice_mass_flow_kg_s` (isentropic compressible flow, choked/subsonic,
  cited Anderson via FBW's own citation), `DuctVolume` (ideal-gas charge/discharge, matching
  `PneumaticContainer`'s relations), `conduct_to` (Cv-based isochoric heat loss, exact
  exponential), `transfer_kg`/`one_way_transfer_kg` (linearised equilibrium-overshoot clamp,
  a real one-way check-valve path with a separately-faultable backflow fraction),
  `DuctSection`/`DuctSectionFaults` (leak/rupture/insulation_damage, bare-pipe-multiplier
  insulation damage model) — files: `src/deep/pneumatic_ducts/duct.rs`. Tests: choked-flow
  plateau, zero/NaN safety, charge/discharge round-trip, transfer conservation + no-overshoot,
  check-valve block/leak, insulation damage speeding heat loss.
- [done] leak.rs: per-section leak/rupture orifice to the zone, energy-conservation-bounded
  heat delivery (`mdot*cp*dT` bound) with a diffuse-vs-impingement **effectiveness** ramp
  (not a fabricated "bonus" — corrected mid-design after checking it against conservation) —
  `src/deep/pneumatic_ducts/leak.rs`. Tests: no-fault zero, monotonic severity, rupture >>
  full-severity leak, heat never exceeds the enthalpy bound, impingement more effective per
  kg/s, no negative heat from a cold duct.
- [done] odls.rs: dual-loop (A/B) overheat detection, first-order sensing-element lag,
  open/short/false-detection fault taxonomy (open -> loop FAULT, short -> that loop pegs hot,
  false_detection -> system-level fictitious-degree bump so the same threshold/confirm logic
  applies uniformly), fail-safe hottest-valid-reading voting, confirm-delay trip —
  `src/deep/pneumatic_ducts/odls.rs`. Tests: healthy never trips, sustained overheat trips
  after confirm not instantly, brief transient does not trip, double-open reports FAULT not
  trip, one shorted loop trips alone, false detection alone trips a cold zone, dt=0 safety.
- [done] precooler.rs: proper eps-NTU two-stream heat exchanger (cites Incropera & DeWitt,
  a strictly better method than FBW's own fixed-conductance exponential relaxation, same UA),
  fixed-cold-side-temperature simplification (fan bleed-off a small % of bypass flow), P
  controller + **first-order actuator lag** (added after discrete-map stability analysis
  showed an un-lagged one-tick P-loop could hunt between FAV extremes — the lag is not just
  numerical convenience, a real actuator has finite travel time too), true-temperature hard
  overtemperature trip independent of a possibly-biased sensor, graduated overpressure relief
  (oil.rs's own relief-valve style), check-valve backflow — `src/deep/pneumatic_ducts/
  precooler.rs`. Tests: settles near target, no-flow-no-NaN, fouling hotter, stuck-closed
  stays hot, overtemp trips past a frozen sensor, relief valve graduated, check valve blocks/leaks.
- [done] network.rs: full topology — engine x4 (source -> precooler -> local duct, always
  flowing) -> isolation valve (ODLS-latched) -> shared cross-bleed manifold -> packs x2, wing
  anti-ice x2 (own proportional regulator, A320-derived control concept), engine start x4
  (fed from the manifold through a real check valve so a lit engine can't push pressure back),
  hydraulic reservoir pressurisation x2. 15 independently-faultable duct sections, 8 ODLS
  zones. Found and fixed a real design bug during testing: the start duct's control-valve area
  and its check-valve's own protective area were the same variable, meaning "starter not
  engaged" (area 0) also silently disabled the check valve's reverse-flow protection —
  split `one_way_transfer_kg` into separate `forward_area_m2`/`check_valve_seat_area_m2`
  parameters so the check valve protects regardless of the control valve's own position (the
  real-world reason a check valve exists at all). — `src/deep/pneumatic_ducts/network.rs`.
  Tests: manifold/pack pressurisation from healthy bleed, confirmed pylon overheat isolates
  and latches that engine only, rupture heats only its own pylon (not others, beyond ordinary
  insulation loss), WAI only flows when selected, check valve blocks/leaks correctly
  (isolated from ordinary bleed pressurisation via a dedicated `no_bleed_inputs` fixture),
  dt=0 safety.
- [done] registry.rs: 10 components (`36_pneu.*` x9, `30_pneu.wing_anti_ice_duct`), 35
  failures (32 ATA 36 + 3 ATA 30, `Area::PneumaticDucts`), 12 ECAM alerts (per-engine/APU
  bleed leak, per-side wing duct leak, per-engine precooler overheat, an aggregate ODLS-fault
  advisory) — `src/deep/pneumatic_ducts/registry.rs`. Followed `deep::sensors::registry`'s
  established precedent: one failure id per fault *mechanism* per component class (instance
  count noted in the component name), not one id per numbered instance.
- [done] FAILURES.md — flat ATA/name/element/magnitude/effect table, all 35 faults.

## New Vars this directory's model would need to publish (not wired yet, per hard rule 2)

`DEEP_PNEU_ODLS_<ZONE>_TRIP` / `_FAULT` (one per `network::ZONE_NAMES` entry: `PYLON_1..4`,
`WING_ROOT`, `APU_BAY`, `WING_LEADING_EDGE_L`/`_R`), `DEEP_PNEU_ENG_<n>_PRECOOLER_OVHT`,
`DEEP_PNEU_APU_PRECOOLER_OVHT`, `DEEP_PNEU_ENG_<n>_ISOLATION_OPEN`,
`DEEP_PNEU_APU_ISOLATION_OPEN`, `DEEP_PNEU_MANIFOLD_PRESSURE_PA`, and the proposed
crew-control completion Vars `DEEP_PNEU_ENG_<n>_BLEED_PB_ON`, `DEEP_PNEU_APU_BLEED_PB_ON`,
`DEEP_PNEU_WAI_L_SELECTED`/`_R_SELECTED` (a future integration pass should point these at
FBW's real `A380PneumaticOverheadPanel` push-button Vars once their exact published `_PB_IS_*`
suffix is confirmed, rather than this module inventing a permanent parallel state).

## Next most valuable items (extending the backlog; not yet started)

- A real bleed-duct **rupture-triggers-structural/wiring-damage** coupling: `leak.rs` already
  flags `is_impinging_jet`; a future pass could feed that into a bay/wiring-harness damage
  model the way `physics::bays`' own module docs describe for the electrical workstream.
- Zone-side integration: wire `network::NetworkOutputs::zone_heat_w` into
  `deep::thermal_zones::network::ThermalNetwork::inject_heat_w` once this directory is allowed
  to depend on that one (currently blocked by the self-contained rule, by design).
- Manifold segmentation: FBW's own A380 topology uses 3 cross-bleed valves (L/C/R) rather than
  one shared manifold; this network deliberately simplified to one shared volume (documented
  in `network.rs`) — a refinement pass could split it to match exactly.
- A leak/rupture's effect on the *pressure-regulating valve's own* upstream regulation
  (currently this network takes the regulated bleed condition as a fixed external input,
  per its own documented scope boundary) — would need this directory to also model the HP/PR
  valve stage, currently left to FBW's own ported systems.

- [done] live system — `live.rs` (`live_system() -> Box<dyn deep::live::Area>`), `mod.rs`,
  plus additive outputs on `network::NetworkOutputs` (precooler outlet temperatures, duct gas
  temperatures, and a refresh of the engine-duct pressures after the cross-bleed block so the
  published value is post-cross-bleed). Owns one `DuctNetwork` driven from `Truth`: ambient,
  the engine bleed port as the IP8 tap, fan-duct cooling air derived from `engine_n1_frac` and
  ambient density against the public Trent 900 bypass flow, and the APU's load-compressor
  discharge temperature from its own published pressure ratio. Consumes all 35 registered
  failures (36/1-32, 30/1-3). Publishes `DEEP_PNEU_ODLS_<zone>_TRIP`/`_FAULT` (7 zones),
  `DEEP_PNEU_ENG_<n>_PRECOOLER_OVHT`, `DEEP_PNEU_APU_PRECOOLER_OVHT`, isolation states, every
  duct pressure/temperature, valve position and per-zone leak heat/jet flux. 8 tests.
  Still missing from `Truth`: separate HP6 port conditions (the HP branch is held shut rather
  than fed a fabricated pressure), the zone air temperatures ODLS watches (every zone is given
  the recovery temperature), and the bleed/cross-bleed/pack/WAI/starter selections (interim
  positions documented in `live::ControlAssumptions`).

- [done] truth-wiring pass (`docs/deep/truth-requests.md`'s 2026-09-20 pass) — removed
  `ControlAssumptions` entirely; `live.rs` now reads `truth.controls.pack_pb_on` (both feed
  valves of a pack open together), `wing_anti_ice_selected` (one pushbutton, both sides),
  `apu_bleed_pb_on` (gates `apu_bleed_available`), `cross_bleed_selector` (raw 0 SHUT/1 AUTO/
  2 OPEN, AUTO keeping the old APU-sole-source heuristic), and `starter_engaged`. Added
  `NetworkInputs::engine_bleed_pb_auto: [bool; 4]` (`network.rs`) and folded it into every gate
  that already checked ODLS's own `isolated[i]` latch (PR valve, all three cross-bleed valves,
  the APU valve, both pack feeds, both WAI feeds) through a new *local*, non-latching
  `source_shut` closure -- the pushbutton is a normal switch, so it must never touch
  `self.engine_isolated`/`out.engine_isolated`, which stay the ODLS trip's own latch. Also wired
  `truth.engine_hp_port_pressure_pa`/`_temp_k` into the HP6 branch (removed the fixed
  `HP_PORT_UNAVAILABLE_PA` = 0), and `Self::zone_air_k` now reads
  `truth.published.get_or("THERMAL_ZONE_<NAME>_TEMPERATURE_C", recovery_c)` instead of pinning
  every zone at recovery temperature -- `thermal_zones`' own ATA 30/36 duct-leak failures
  (registered independently under `Area::ThermalZones`) now reach this area's own ODLS. 7 new
  tests: the HP6 branch opening the HP valve off a real HP6 port, the cross-bleed selector's
  SHUT and OPEN positions, a pack pushbutton stopping its own pack, the ENG BLEED pushbutton
  shutting (and non-latchingly restoring) an engine's own source (`network.rs`), and an
  end-to-end test arming `thermal_zones`' own WingLeLeft anti-ice-duct-leak failure and
  confirming it now trips this area's own `DEEP_PNEU_ODLS_WingLeLeft_TRIP` through the published
  frame. That same test found the equivalent pylon leak (`PYLON_BLEED_LEAK_MAX_HEAT_W` = 40 kW
  against this area's own 0.5 kg/s pylon ram vent) settles only ~75 K above ambient -- real,
  substantial heating, but short of the fixed 100 K confirm margin; not fixed here since it is a
  gap in `thermal_zones`' own interim leak magnitude, not in this area's consumption of it.
  Still missing from `Truth`: nothing new; per-FCV pack valve positions were confirmed to not
  exist as a separate cockpit control (only the pushbutton is real, `docs/deep/
  truth-requests.md`).
  positions documented in `live::ControlAssumptions`).

## 2026-09-20 — dead-failure audit follow-up

- **ODLS threshold, form fixed**: `odls.rs`'s `THRESHOLD_ABOVE_AMBIENT_K` (ambient-relative) replaced
  with two absolute set points, `THRESHOLD_WING_FUSELAGE_K` (~124 C) and `THRESHOLD_PYLON_STRUT_K`
  (~200 C), assigned per zone in `network.rs::odls_threshold_k` (pylons + APU tail-cone = hot class,
  wing leading edge = cool class), per `thermal_zones::PROGRESS.md`'s derivation. `OdlsOutputs`
  gained `loop_a_fault`/`loop_b_fault` (published as `DEEP_PNEU_ODLS_<zone>_LOOP_A/B_FAULT`) so a
  single open loop is visible even though it correctly cannot move the aggregate fault/trip alone
  (the dual-loop masking the task named) — new tests in `odls.rs`.
- **This area's own duct leaks now reach its own ODLS**: `thermal_zones` never consumed this area's
  `zone_heat_w`, so a real leak/rupture here could move duct pressure/gas temperature but never the
  bay temperature its own ODLS reads. `PneumaticDuctsLive::relax_own_zone_excess` adds a stored,
  exponentially-lagged local hotspot excess (steady-state `zone_heat_w/(mdot_vent*cp)`, reusing
  `thermal_zones`' own cited 0.5 kg/s pylon ventilation figure) on top of whatever `thermal_zones`
  publishes. An earlier memoryless version of this (recomputed fresh each tick, no state) oscillated
  violently in the closed loop with `leak::step`'s own delta-T (verified by instrumentation: 946 kW /
  0 / 144 kW / 0 before settling) — the lagged, stateful version is unconditionally stable. New test:
  `this_areas_own_engine_duct_rupture_now_reaches_its_own_odls`. Fixed
  `arming_the_engine_bleed_duct_rupture_sags_the_duct_and_heats_the_pylon`'s setup to check heat
  early (self-consistent cooling now makes the 100-tick number honestly smaller, the bay-cannot-
  exceed-the-duct ceiling this same investigation names for the wing case below).
- **`WINGLELEFT > 150 C` test**: left as is (still passes) — `thermal_zones`' own `WING_DUCT_LEAK_
  MAX_HEAT_W` fixed-wattage form is that area's file, out of scope here; comment corrected to say so
  and to stop citing the old relative-margin threshold.
- **`PNEU_APU_BLEED_DEMAND_KG_S`** published (`NetworkOutputs::apu_bleed_demand_kg_s` = the real
  precooler hot-side mdot, zero when bleed not selected) for `deep::apu`'s load-compressor coupling.
  Could not confirm this also explains this area's own 3 "dead" APU precooler/FAV/sensor failures:
  reading `precooler.rs::step` shows `mdot_hot` was already nonzero under `ground_apu`/
  `apu_start_soak` before this change (bleed available, real orifice flow) — if those three are
  still dead post-sweep, the cause is something else, not an unloaded path.
- **Tyre pressure request (coordinator)**: `deep::gear_structure` does not model tyre pressure at
  all; the real model (`physics::tyre::Tyres`, nitrogen/Gay-Lussac, per-wheel) already exists and
  already publishes `TYRE_PRESSURE_PA:n` for all 16 wheels, but through the plugin's own Var/
  `SimulatorReaderWriter` channel, not `Truth`. No deep area can see it without a `Truth` field
  sourced from that model — not invented here.
- **Fuel tanks / leak timing / controls wiring**: see `deep/fuel/PROGRESS.md`, `deep/cabin/
  PROGRESS.md`, `deep/gear_structure/PROGRESS.md` (different areas, same task).
