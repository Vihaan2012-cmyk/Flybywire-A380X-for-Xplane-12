# Fuel deep model — progress log

**Area note (read first):** this directory's `registry.rs` and every doc
comment is written against `Area::Fuel` (intended value 19). At the time of
writing, `src/deep/api.rs`'s `Area` enum did not yet exist in the checked-out
tree this task began from; per the task instructions, `Area::Fuel = 19` is
the lead's to add to `api.rs` (do **not** add `Area::Integration` or
`Area::Breakers` for this directory — those are wrong). Everything here
compiles once that one enum variant exists and `src/deep/mod.rs` gains
`pub mod fuel;`.

Directory: `src/deep/fuel/`. ATA 28 throughout. See `docs/deep/BRIEF.md` for
the shared brief.

Self-containment: nothing in this directory calls into `crate::fuel`,
`crate::fuel_network`, `crate::fuel_transfer` or `crate::physics::fluids` --
every module's doc comment names the specific existing function it goes
deeper than, without duplicating it. Modules within this directory freely
reference each other (`cg_transfer`/`jettison`/`leak`/`gauging` all use
`super::geometry`; `leak` reuses `jettison`'s orifice-flow primitives) since
that is all still "this directory's own code" under the brief's rule 2.

## New plugin Vars this model will eventually need published

None exist yet from this directory (nothing outside `src/deep/fuel`
references this code yet, per the brief's self-containment rule); these are
forward declarations for whoever wires this model into `fuel.rs`/
`fuel_network.rs`/the plugin's variable registry:

- `FUEL_LEAK_DETECTED` (bool), optionally `FUEL_LEAK_SIDE` (0 none/1 left/2
  right) -- `leak::LeakDetector::update` / `leak::likely_leaking_side`.
- `FUEL_TRIM_TRANSFER_FAULT` (bool) -- `cg_transfer::TransferFaultDetector`
  applied to the trim-tank transfer path.
- `FUEL_CG_TRANSFER_DEGRADED` (bool) -- the same detector applied to the
  outer/inner/mid-to-feed paths.
- `FUEL_CROSSFEED_FAULT` (bool), `FUEL_CROSSFEED_OPEN` (bool, likely already
  derivable from the network's own valve state once wired) --
  `cg_transfer::wing_balance_transfer_needed`/`heavy_side` plus the
  cross-feed valve faults.
- `FUEL_FQMS_LOW_CONFIDENCE` (bool, aggregate) and, per tank,
  `FUEL_FQMS_CONFIDENCE_PCT:n` -- `gauging::fqms_indicated_fraction`'s
  `confidence` output.
- `FUEL_FILTER_ICE_DETECTED` (bool, aggregate), per engine
  `FUEL_FILTER_BLOCKAGE_PCT:n` -- `thermal::filter_blockage_fraction`.
- `FUEL_TANK_BAFFLE_DAMAGE_DETECTED` (bool, aggregate) --
  `geometry::TankShape.slosh_damping_ratio` faulted low.
- `FUEL_JETTISON_L_VALVE_FAULT` / `FUEL_JETTISON_R_VALVE_FAULT` (bool) --
  `jettison::JettisonValve` stuck-fraction past some threshold.

## Log

- [done] `geometry.rs` -- per-tank shape (11 real tanks, `flight_model.cfg`
  capacities cited), pitch/bank/acceleration surface tilt, pump/probe
  unporting, attitude-dependent usable/unusable fuel, sloshing period and
  settle time constant. Tests: 10.
- [done] `gauging.rs` -- capacitance-probe-count-by-size, per-probe
  failure/bias/exclusion, FQMS averaging with confidence, densitometer
  failure (mass from a stale default density). Tests: 8.
- [done] `cg_transfer.rs` -- named trim/outer/inner/mid/cross-feed valve and
  pump health, achieved-transfer-rate model, a sustained-shortfall fault
  detector, wing-root bending-moment relief (quantifies *why* outer-tank
  retention matters), outer-tank retention scheduling, wing-balance
  cross-feed decision. Tests: 9.
- [done] `thermal.rs` -- FCOC (engine-oil-cooler) heat into the tank energy
  balance (a real, already-published var, `ENGINE_FCOC_HEAT_W:n`, that
  `fuel.rs`'s own thermal model never consumes today), fuel-type freeze
  points (Jet A/Jet A-1/JP-8), wax formation ramp, free-water filter icing
  and combined filter blockage. Tests: 9.
- [done] `jettison.rs` -- per-nozzle valve transit-time dynamics with a
  partial-authority stuck fault, nozzle blockage, and a real orifice-flow
  model (tank head + optional pump assist vs. ambient static pressure) in
  place of the existing single lumped-orifice, instant-valve model. Tests:
  10.
- [done] `leak.rs` -- tank-wall and gallery-section leak orifice flow (reuses
  `jettison.rs`'s orifice primitives, no duplicate physics), a rolling-window
  fuel-used-vs-quantity-change leak detector with multi-window confirmation,
  and per-side isolation. Tests: 8.
- [done] `registry.rs` -- registers all of the above: 11 tank-geometry
  (baffle damage) + 33 gauging (probe/compensator/densitometer x 11 tanks) +
  16 named transfer valve/pump + 12 thermal (FCOC fouling x4, filter
  water/heater x4x2) + 4 jettison + 13 leak (11 tank wall + 2 gallery) = 89
  failures, matching components, and 9 ECAM alerts. Tests: 5 (clean
  validation, every component has a failure, id uniqueness/area/ATA, all 11
  tanks covered across geometry/gauging/leak, every alert only names
  registered failures).

## Next (not started -- stopped at the lead's hard-stop call)

- Wire `cg_transfer::wing_bending_relief_nm`'s `span_m` argument to the real
  per-tank lateral `Position` values (`flight_model.cfg` lines 142-152,
  second `Position` field, feet -> metres) once this module is connected to
  a caller that has them -- this directory intentionally does not read that
  cfg file itself (self-containment).
- A `FuelType`-aware density/viscosity pairing in `thermal.rs` (currently
  only freeze point varies by type; density/viscosity still assume the
  existing Jet A curves) if a caller needs a non-Jet-A dispatch.
- `gauging.rs` currently biases probe readings deterministically by index
  parity for reproducibility; a caller wanting per-probe identity (which
  physical probe number is failed, for a maintenance page) should track
  that alongside `ProbeFault` rather than inferring it from the bias sign.
- No leak-vs-jettison cross-talk is modelled (a jettison in progress at the
  same time as a real leak would confuse `leak::LeakDetector`'s fuel-used
  accounting, since jettisoned fuel is neither "used" nor "leaked" in the
  detector's current two-term model); a full integration would need a third,
  jettison-flow term in the balance.
