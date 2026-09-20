# Electrical area — progress log

Area: `Area::Electrical` (`docs/deep/BRIEF.md`). Directory:
`D:\fbw-xp-systems\src\deep\electrical\`.

- [done] Backlog 1 — network data model & per-tick solver — `network.rs` —
  `BusId` (17 A380 buses: AC1-4, AC ESS, AC ESS SHED, AC EMER, AC GND FLT
  SVC, DC1/2, DC ESS, DC ESS SHED, DC BAT, DC HOT1/2, DC APU, DC GND FLT
  SVC), `Load`/`LoadSpec`/`LoadFaults`, `Breaker` with a real I²t/instant-
  magnetic trip curve (same K/multiple/cooldown constants as
  `physics::electrical.rs`'s own cited curve, reimplemented independently
  per the no-crate-internals rule), `Contactor` (open/closed/welded/fails-
  to-close) unifying generator-line/bus-tie/feeder/battery-direct into one
  electrically-identical type, `Diode` (one-way, forward drop, fails-open),
  `Bus`/`BusFaults` (short to structure). Solved every tick as a resistive
  network with constant-power loads and constant-conductance short faults,
  via Millman's-theorem Thevenin combination + the same
  `V² - V_rated·V + S·Xs = 0`-shaped quadratic FBW's own TRU/generator
  models use (generalised here to
  `(1+R_th·G)·V² - V_th·V + R_th·P = 0`), relaxed over closed contactor/tie
  loops by Gauss-Seidel/Jacobi sweeps (`Network::relax`, `ITERATIONS = 15`).
  17 tests, including a closed-form-quadratic check, a bus-tie/diode
  propagation check, an iteration-budget convergence check, and one test per
  fault channel. Also added `Network::add_feeder_breaker` (a whole-bus
  breaker, distinct from a per-load one) after review found the original
  bus-fault test's assertion had no breaker that could actually see the
  fault current — a real, useful primitive matching `breakers.rs`'s own
  "AC1 BUS FEED"-class entries, not just a test fix. 20 tests total.
- [done] Backlog 2 — load catalogue — `loads.rs` — every named consumer
  group in `D:\fbw-xp-systems\src\breakers.rs`'s ATA21/26/27/32/34/36
  functions (cabin fans, FDAC/TADD/VCM/OCSM/CPIOM-B, pack flow valves, fire
  detection loops, PRIM/SEC/FCDC/FCU/rollout, LGCIU, electric hydraulic
  pumps, autobrake solenoid, gear/door proximity sensors and actuators,
  radio altimeters + antennas, EGPWC, bleed valve sets) plus the major loads
  `breakers.rs` does not enumerate: all 25 fuel pumps + 60 fuel valves
  individually (`circuits.rs`'s own embedded count, reusing
  `physics::electrical.rs::rated_watts`'s already-established 600 W/50 W
  precedent), 12 lighting feeders lumped per `CIRCUIT_LIGHT_*` type (brief's
  own instruction), window/probe/pitot heat, 6 galley zones, 5 IFE cabin
  zones, 12 avionics computers/radios not reached elsewhere. ~245 loads
  total (verified by test, `> 200`), each with its own breaker, rated
  power/pf/min-operating-voltage/inrush/wiring-resistance and a cited
  `basis` string (real/FBW-sourced or explicitly `GENERIC`/typical). TR/GEN/
  APU-GEN/static-inverter/bus-feeder entries from `breakers.rs::ata24` are
  *not* duplicated as loads (they are sources + their own protection
  interface, built in `sources.rs` instead).
- [done] Backlog 3 — faults — implemented directly on the models above:
  per-load open circuit/short-to-ground/high-resistance/intermittent;
  per-breaker fails-to-trip/nuisance-trip; per-bus short-to-structure;
  per-contactor fails-to-close/welded-closed; per-diode fails-open. See
  `FAILURES.md` for the full table.
- [done] Backlog 4 — source models — `sources.rs` — `Vfg` (4, frequency
  tracks engine speed 360-800 Hz variable, own I²t-shaped overload
  accumulator), `ApuGenerator` (2, same machine class, governed constant
  400 Hz), `Tru` (4, real FBW `INTERNAL_RESISTANCE_OHM`/`IDLE_OUTPUT_
  VOLTAGE`/thermal constants, exact-exponential thermal step), `Battery` (2,
  real FBW capacity/resistance/thermal/Peukert constants, charge-state
  coulomb counting, OCV-vs-SOC, resistance-vs-temperature), `StaticInverter`
  (real FBW efficiency constants), `Rat` (real FBW propeller diameter/max-
  power, aerodynamic power sized via the max-power-transfer relationship
  directly into the network's own solver), `GroundPower` (real FBW rated
  VA/regulation). `Wiring::build` assembles the real ATA24-equivalent
  source/contactor/breaker set onto a `Network` (GEN 1-4, APU GEN 1-2, TR
  1/2/ESS/APU, STATIC INVERTER, BAT 1/2, GPU, RAT, plus one real hot-bus
  isolation diode) and orchestrates the pre/post-tick measured-feedback loop
  every stateful source needs (documented one-tick-lagged pattern, same
  shape as `physics::electrical.rs`'s own `EngineLoads` contract). 15 tests.
- [done] Backlog 5 — shedding — `shedding.rs` — `SheddingRelays` (galley
  shed on any commanded power-budget trim; galley + commercial/IFE shed in
  an emergency configuration; both with their own fails-to-shed/spurious-
  shed faults), `power_budget` (real solved demand vs. caller-supplied
  capacity), `category_demand_w`, `bus_total_current_a`. The bus-transfer
  transient is *not* a new mechanism — it is the existing per-load inrush
  model re-triggering for real when a bus dies and is re-energised through
  a tie (`bus_transfer_produces_an_inrush_transient` proves the emergent
  behaviour rather than adding a second one). 7 tests.
- [done] Registration — `registry.rs` — builds one real `Network`/`Catalog`/
  `Wiring` and walks its actual contents to register a `ComponentDef` +
  `FailureDef`(s) per real instance (so the registry can never drift from
  the real catalogue): ~600 components/failures total (verified `> 200`
  components, `> 400` failures). Plus 8 curated top-level ECAM alerts (GEN/
  APU GEN/TR/BAT FAULT, AC BUS FAULT, EMER CONFIG, galley/commercial shed
  advisories). 4 tests, including full `Registry::validate()` passing clean.

## Round 2 (in progress when stopped at the lead's hard-stop time)

The lead asked for six extensions, then (higher priority, mid-round) for a
real multi-feed load model since "breaker count is too low" and most real
A380 LRUs take power from 2-3 separate buses with internal OR-ing. Status:

- [done] **Multi-feed loads with priority OR-ing, one breaker per feed** —
  `network.rs`: new `LoadFeed { bus, breaker, priority }`; `Load` now holds
  `feeds: Vec<LoadFeed>` (was a single `breaker: usize`); `Load::select_feed`
  picks the first feed (by construction order) whose own breaker is closed
  and whose bus clears the load's under-voltage threshold — exactly a real
  dual-fed LRU's internal power-supply OR-ing (losing one feed alone does
  not lose the unit). `Network::add_load` (single-feed) is unchanged and
  still the common case; new `Network::add_load_multi_feed` for the rest.
  Applied so far in `loads.rs` via two new builders, `add_dual`/`add_triple`
  (`add_triple` written, not yet called anywhere — ready for the FWS/CPIOM
  work below): the 11 flight-control/autoflight computers (ata27, now DC1/
  DC2 normal + DC ESS backup, 22 breakers instead of 11), both LGCIUs (dual
  DC ESS/DC2), FMS x3 + ADIRU x3 + TCAS (dual normal + ESS/backup). Each
  electric hydraulic pump also gained its own separate small contactor-coil
  supply load (distinct from the pump motor's own 75 A feed, as asked).
  Still single-feed and NOT YET converted: PRIM/SEC/FCDC individually beyond
  the ata27 loop above (they're already dual via that loop), the FDAC/TADD/
  VCM/OCSM/CPIOM-B channel loads (already redundant *by channel*, arguably
  don't also need per-channel dual-feed), CDS display units/DMC/FWS/CIDS
  (not modelled as their own loads at all yet — see catalogue growth below).
- [done] **AC frequency is now a real, load-affecting quantity, not just
  informational** — `network.rs`: frequency is resolved once per tick
  (`Network::resolve_frequency`, pure topology, before the voltage
  relaxation sweep since it has no circular dependency on load current) and
  fed into `Load::contribution` via a new `LoadSpec::rated_frequency_hz`
  field. `Load::frequency_multiplier` applies real fan/pump affinity-law
  scaling (`P2/P1 = (N2/N1)^3`) for the handful of loads that are genuinely
  simple direct-drive AC induction motors with no speed control of their
  own — `loads.rs`'s cabin fans (x4) and avionics-bay fans (x4, grown from
  2) now carry `rated_frequency_hz = 400.0` and will visibly speed up/slow
  down and change power draw as engine-driven VFG frequency (360-800 Hz)
  varies. Motor-driven pumps/valves were deliberately *not* marked
  frequency-sensitive (they run through their own motor controllers, which
  regulate speed independent of input frequency on a real aircraft).
- [done] **Power quality: undervoltage dropout with real automatic-restart
  hysteresis** — `Load` now latches (`undervoltage_latched`) on a genuine
  under-voltage condition (at least one feed's breaker closed but its bus
  below `min_operating_voltage`) and requires the recovering bus to clear
  `min_operating_voltage * UNDERVOLTAGE_RESTART_MARGIN` (1.05x, GENERIC
  comparator-hysteresis margin) before resuming — stops a load right at its
  own dropout point chattering on/off as its own inrush sags the bus back
  down. Large-load-switching voltage transients were already emergent from
  the existing resistive solve + inrush model (see `shedding.rs`'s
  `bus_transfer_produces_an_inrush_transient`); not separately re-verified
  this round.
- [not started, next up] **More real diode paths** (hot-bus feeds beyond
  the one existing `bat-cross-feed-diode`, a DC ESS battery-backup diode, a
  battery-to-battery-bus isolation diode) — planned (DC BAT → DC ESS backup,
  DC HOT1 → DC BAT backup, DC ESS → DC ESS SHED backup) but not written.
- [not started, next up] **Bidirectional battery charging** (BCRU/charger
  current limit, CC/CV phases, temperature/SOC-derated charge acceptance,
  a `fails` fault showing a dead charger) — `sources.rs`'s `Battery::step`
  already accepts a signed current (positive discharge, negative charge) and
  integrates SOC/thermal correctly either way; what's missing is a
  `BatteryCharger` model that *derives* that charge current from the real
  bus-vs-OCV differential each tick (clamped to a CC/CV-phase limit) instead
  of an external caller supplying `WiringInputs::measured_battery_current_a`
  by hand. Design worked out (Ohm's-law-implied current, capped by
  `charge_limit_a(soc, temp_c, faults)`, CV taper above ~80% SOC) but not
  implemented.
- [not started, next up] **TR ESS/TR APU AC input, APU generator's own
  bus** — still approximated (`Wiring::build`'s `apu_gen_bus = Ac3` shared
  by both APU generators; TR ESS/TR APU both check `AC_ESS`). Planned fix:
  split apu-gen-1 → AC2, apu-gen-2 → AC3 (two distinct ties instead of one
  shared bus), and make TR APU check `AC2 || AC3` (pairing it with APU
  generation specifically) instead of AC_ESS.
- [not started, next up] **Catalogue growth past 373 individually modelled
  loads** — currently ~250 (up from ~245 pre-round-2; still short of the
  373 target). Planned categories, sized to comfortably clear the target:
  EHA/EBHA flight-control actuator motors (ATA27, ~6, real per the brief's
  own aircraft description), a much more granular lighting fixture list
  (currently 12 lumped types → split into ~30 real fixtures: per-side
  landing/taxi/nav/beacon/strobe/wing/logo, per-zone cabin lighting, cargo,
  wheel-well), more avionics LRUs (CVR, FDR, SATCOM, ACARS, CIDS, DME/VOR/
  ILS, standby instruments, cockpit displays x8 — several of these as
  dual-fed via `add_dual`/`add_triple`, which also directly grows the
  breaker count as the lead asked), gear uplock/downlock solenoids distinct
  from the sensors already modelled, passenger service units per zone,
  engine igniters (x8, 2/engine), lavatory/cargo smoke detectors, cargo
  door actuators, wiper motors, and a few more galley/IFE zones. None of
  this is written yet.
- **Diodes/frequency/TR-APU/catalogue-growth items above are genuinely
  incomplete, not silently approximated** — every one is called out
  explicitly here rather than left implicit, per the hard stop instruction
  to leave nothing half-written: everything currently on disk compiles
  logically end-to-end (mod.rs declares every file; `registry.rs` walks the
  real catalogue so it can never point at a removed id; every test file was
  updated for the new `LoadSpec::rated_frequency_hz` field and the new
  `Load::step`/`Network::relax` signatures) — what's incomplete is scope
  (features not yet started), not broken code left mid-edit.

## Vars an integration layer would need to publish (none published yet —
this crate is self-contained per the workstream's hard rules; the ECAM
alerts in `registry.rs` reference these by the plugin's existing `ELEC_*`
naming convention, but nothing in this directory writes them):

- `ELEC_GEN_<1-4>_FAULT`, `ELEC_GEN_<1-4>_PB_ON`, `ELEC_GEN_<1-4>_SMOKE`
- `ELEC_APU_GEN_FAULT`, `ELEC_APU_GEN_PB_ON`
- `ELEC_TR_<1|2|ESS|APU>_FAULT`
- `ELEC_BAT_<1-2>_FAULT`, `ELEC_BAT_<1-2>_PB_ON`
- `ELEC_AC_<1-4>_BUS_POTENTIAL`, `ELEC_AC_<1-4>_BUS_IS_POWERED`
- `ELEC_RAT_DEPLOYED`
- `ELEC_GALLEY_SHED_ACTIVE`, `ELEC_COMMERCIAL_SHED_ACTIVE`

Each maps directly onto an existing model output (`Vfg::overload_tripped`/
`Breaker::trip_cause`/`Bus::voltage`/`Rat::deployed`/`ShedOutputs`) — wiring
them is a mechanical `VariableRegistry::get`/`write` pass, not a new model.
- [done] frame-rate and stability fixes behind the phantom breaker trips: `Load::inrush_multiplier` now returns the frame *average* of the exp decay (was sampled at the frame leading edge and held, so a 0.5 s inrush cost a full second of I^2t at dt=1); the AC ESS transfer logic no longer reads the emergency inverter's own back-feed through the bidirectional ess-feed tie as evidence that emergency config is not needed (the whole network limit-cycled at the frame rate cold and dark); added a real under-voltage lockout *timer* (`UNDERVOLTAGE_LOCKOUT_S`) beside the existing restart margin; the heavy AC motor loads (4 electric hydraulic pumps, 25 fuel pumps) are shed with no generation on line; `board::clear()` is now called by `ElectricalLive::new` so a second aircraft in one process cannot inherit the first's board; published `ELEC_ENG_GEN_{1..4}_LOAD_W` and `ELEC_APU_GEN_{1,2}_LOAD_W` -- live.rs, network.rs
