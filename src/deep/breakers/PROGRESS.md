# Circuit breakers — progress log

Area: `Area::Breakers` (17). Directory: `src/deep/breakers/`. See
`docs/deep/BRIEF.md` and this directory's own `docs/deep/BRIEF.md`-mandated
task: expand circuit breakers from the plugin's existing 265-entry
`src/breakers.rs` toward the real A380's much larger set, one breaker per
modelled load, with full trip physics.

## Sources read in full this session

- `docs/deep/BRIEF.md` (the shared brief).
- `src/breakers.rs` (the existing 265-ish-entry catalogue: `BreakerDef`
  fields, `Bus`, rating-basis constants, per-ATA builder pattern — used as
  the field/structure precedent this new catalogue extends, not imported).
- `src/deep/api.rs` (the `Registry`/`FailureDef`/`ComponentDef`/`ParamDef`
  API every area registers through).
- `src/deep/electrical/loads.rs` (592 lines, read in full): the ~245-entry
  load catalogue this breaker table protects, transcribed group-by-group
  (same `ata21`/`ata26`/.../`ata36_bleed` function names, same ids, buses,
  wattages, power factors) so every id here matches that file's own id
  exactly.
- `src/deep/electrical/network.rs` (struct definitions only: `BusId`,
  `LoadSpec`, `Breaker`, `Load`) — confirmed the 115 V AC / 28 V DC nominal
  split and the load-current formula `loads.rs` itself uses, both
  independently re-derived here per the brief's self-containment rule
  (nothing in `src/deep/breakers` imports `deep::electrical`).
- `src/deep/sensors/mod.rs` and `src/deep/cabin/registry.rs` as worked
  examples of the self-contained-directory / `registry.rs` conventions
  other finished areas already established.

## Done

- [done] `catalog.rs`: `Bus`/`Panel` types, [`standard_size`] (AS39019/
  MIL-PRF-39019 standard ampere-rating series, rounds up only),
  `kind_for` (Thermal vs SSPC split at 25 A, GENERIC, cited), `panel_for`
  (Primary/Secondary Power Centre for SSPC by bus; overhead fwd/aft or
  avionics bay for thermal by ATA group). 296 `BreakerDef` entries: 245
  matching every `deep::electrical::loads.rs` load 1:1 by id (`ata21`
  through `ata36_bleed`, mirroring that file's own group functions), plus
  51 more real A380 breaker-protected equipment items with no load model
  anywhere in this codebase yet (`ata24_power_sources`, `ata23_comms`,
  `ata31_recorders`, `ata35_oxygen`, `ata49_apu`, `ata7x_engine` — FADEC A/B
  and ignition exciter A/B per engine, `ata26_extinguishing` — fire-bottle
  squibs, `ata29_hydraulics_extra` — RAT solenoid/PTU valve, `ata52_doors`,
  `ata33_emergency_lighting`), each honestly carrying `protected_load: None`
  and a `GENERIC`-labelled basis rather than a fabricated gate. Tests:
  unique ids, rating ≥ load current (both the plain and margined figure),
  `standard_size` never rounds down, every ATA chapter used is a real A380
  chapter, kind follows its own rating split, SSPC entries land in a power
  centre, group-2 equipment is honestly undocumented-load + GENERIC. Files:
  `src/deep/breakers/catalog.rs`.
- [done] `trip.rs`: thermal I²t bimetal model (heat accumulator, heats
  with `(I/Ir)² - 1` above rated, cools exponentially, tau 20 s) with
  MIL-PRF-39019-cited ambient derating (linear interpolation between the
  25 C reference and 71 C high-temperature published end points); magnetic
  instantaneous trip at 10x rated (GENERIC, typical thermal-magnetic
  breaker class); a separate SSPC path — same I²t accumulator but tau 8 s
  (tighter, more repeatable microprocessor-timed curve, no ambient term —
  a real, documented SSPC advantage) plus arc-fault detection (a GENERIC
  di/dt-threshold proxy for a real spectral arc signature, since true arc
  detection is beyond this simulation's fidelity). Two continuous health
  faults per breaker: `trip_calibration_drift` (nuisance trip — lowers the
  effective rated current up to 40%) and `contact_resistance` (fails to
  trip — raises the I²t heat threshold and the magnetic multiple by
  `1 / (1 - contact_resistance)`, clamped at 0.999 to stay finite/NaN-free
  — a fixed multiplier would still eventually trip given a large enough
  sustained overload, which is not what "welded shut" means physically;
  this diverges instead, so a fully welded breaker's threshold sits far
  above anything a sustained overload can reach). 13 unit tests: no NaN at rest, sustained
  overload trips but brief inrush does not, monotonic trip-time curve,
  magnetic trip fires instantly, drift causes a real nuisance trip, weld
  prevents tripping under a severe sustained overload, SSPC trips faster
  than thermal at the same overload, SSPC detects a fast current-step arc
  signature a thermal breaker categorically cannot, ambient heat trips a
  thermal breaker sooner, reset clears state. Files:
  `src/deep/breakers/trip.rs`.
- [done] `registry.rs`: walks `catalog::all()` once, registers one
  `ComponentDef` per breaker (id `17_breakers.<breaker-id>`, the two health
  params above) and its two `FailureDef`s (nuisance trip, fails to trip),
  ids assigned sequentially per ATA chapter via `failure_id(Area::Breakers,
  ata, n)`. Deliberately registers **no** ECAM alerts: a real A380 breaker
  fault has no alert of its own — the *consuming system's* own loss-of-
  power (or, for a fails-to-trip overload, that system's own overheat/
  smoke) alert is what actually annunciates, and authoring one here would
  duplicate whatever the owning area (`deep::electrical`, `deep::fire_ice`,
  ...) already does or will do. 4 tests: whole catalogue registers with no
  `Registry::validate()` errors, exactly one component + two failures per
  breaker, no ATA chapter's failure count overflows the id encoding
  (`n < 1000`), component ids unique and every failure's `component`
  resolves. Files: `src/deep/breakers/registry.rs`.
- [done] `mod.rs` declaring `catalog`, `registry`, `trip`.
- [done] `COUNTS.md` (totals by ATA/panel/type, computed exactly from the
  same rating/kind/panel logic `catalog.rs` implements — cross-checked with
  an independent Python re-implementation of that logic during authoring,
  not hand-estimated).

## Session 2 (coordinator follow-up: keep in step with electrical's growth,
## SSPC remote control, panel layout, integration test)

Re-read `src/deep/electrical/loads.rs` in full again (now 609 lines, was
592): the electrical agent has added a `rated_frequency_hz` field to
`LoadSpec` and a `frequency_sensitive_motor_spec` helper (for direct-drive
induction-motor loads with no speed control of their own), but **every load
id, bus and wattage is still identical to session 1** — no re-sync of
`catalog.rs`'s own 245 `push_electrical` entries was needed yet. The
coordinator flagged that the electrical agent is *about to* split lumped
loads into per-unit/per-side/per-channel loads and past 373 total; that
split has not landed in `loads.rs` on disk as of this session's last read.
**Re-read `loads.rs` again before starting the next work item** — if ids
have changed by then, reconcile immediately (see checklist below) before
adding anything else.

- [done] `trip.rs`: SSPC remote control/status/lockout, per the
  coordinator's explicit ask. `Breaker::reset`/`remote_reset`/`remote_open`
  now return `Result<(), RemoteControlError>`: a thermal breaker refuses
  `remote_reset`/`remote_open` with `NotRemoteCapable` (it has no CDS/OIT
  interface at all — must be pushed back in by hand), matching the real
  hardware split. `Breaker::status()` returns `SspcStatus` (`Closed` /
  `OpenCommanded` / `Tripped(TripCause)` / `LockedOut`) for a CDS/OIT
  CB-page style display. Lockout: an SSPC that trips
  `LOCKOUT_TRIP_COUNT` (3, GENERIC) times within a rolling
  `LOCKOUT_WINDOW_S` (300 s, GENERIC) window latches `locked_out = true`
  (cited to real TE Connectivity/Data Device Corporation SSPC application-
  literature "trip lockout" behaviour) — `reset`/`remote_reset` then refuse
  with `LockedOut` until `maintenance_clear_lockout()` (a ground-maintenance
  action, not available from the flight deck). 6 new unit tests: thermal
  has no remote interface at all, SSPC remote reset/open work and update
  `status()` correctly, 3 trips in the window latch lockout and a
  flight-deck reset cannot clear it, `maintenance_clear_lockout` does clear
  it, trips spaced outside the window never accumulate toward lockout.
- [done] `catalog.rs`: `PanelPosition { row, column, label }` field added
  to `BreakerDef`, filled by a new deterministic second pass
  (`assign_positions`, called once at the end of `build_catalog`) so the
  Study/Breakers page can draw a real grid per panel instead of a flat
  list — breakers grouped by `Panel` (via a new `Panel::code()` short
  mnemonic, since `Panel` has no natural `Ord`), then ATA, then id, 12
  columns per row (GENERIC panel-module width), `label` a <=14-char
  truncation of the breaker's own `name` (a real CB cap's own printed
  legend width class; GENERIC formatting, no photographed real A380
  panel-plate diagram is public at per-breaker resolution). 2 new tests:
  every breaker has a unique (panel, row, column), every label is
  non-empty and within the 14-char cap width.
- [done] `catalog.rs` Group 3 — control/excitation supplies, per the
  coordinator's "include control/excitation supplies (relay coils, valve
  actuator supplies, sensor excitation) as their own protected circuits
  where real" instruction: 77 new position-indication microswitch/LVDT
  excitation breakers (`<parent-id>-pos-ind`, 5 W GENERIC each), one paired
  with every existing valve-actuator breaker in the catalogue (60 fuel
  valves, 2 hot-air valves, forward/bulk cargo isolation valves, 4 pack
  flow valves, the 4 engine bleed valve sets, the PTU control valve, the
  APU fuel shutoff valve, the crew oxygen shutoff valve, and the fwd/aft
  cargo door actuator controls) — real large-transport practice: a valve's
  actuator power and its position-indication circuit are commonly two
  separate small CBs, so maintenance can safe one without losing the
  other's signal. All `protected_load: None` (honest: `deep::electrical`
  models the valve's actuator draw as its one `Load`, not a separate
  sensing tap, so there is no load id to point at yet — promote to
  `Some(...)` if/when `loads.rs` ever splits a valve into an actuator +
  excitation pair the same way). 2 new tests: every `-pos-ind` entry has a
  real parent breaker and carries no load, catalogue-wide id uniqueness
  still holds. **Total catalogue: 373** (245 + 51 + 77).
- [done] `integration_test.rs` — the coordinator's explicitly requested
  cross-check, declared `#[cfg(test)] mod integration_test;` from `mod.rs`.
  Unlike every other file in this directory, it **does** import
  `crate::deep::electrical` (an explicit, coordinator-authorized exception
  to the self-containment rule, not a self-initiated one) and therefore
  only compiles once the lead's build adds both `pub mod electrical;` and
  `pub mod breakers;` to `src/deep/mod.rs` — confirmed by reading
  `src/deep/electrical/mod.rs` that `loads`/`network` are both `pub mod`
  and `Network::{new, loads}`/`loads::build` are all `pub`, so the paths
  used are correct today even though nothing can compile them yet. 3
  tests: every `protected_load` id resolves against `deep::electrical`'s
  live catalogue, every `deep::electrical` load has a protecting breaker
  here, no load id is claimed by more than one breaker.

## Known limitations / next loop

- **The coordinator's "one breaker per FEED (not per unit), per side and
  per channel" instruction is only partially actionable yet.** Most of
  this catalogue's avionics LRUs already follow a real per-channel/per-unit
  redundancy pattern (FDAC/TADD/VCM/OCSM channels 1/2, LGCIU 1/2, PRIM 1-3/
  SEC 1-3/FCDC 1-2, radio altimeter systems A/B/C, fire loops A/B — each
  already its own id/breaker). What is *not* yet reflected is a single
  physical unit with two independent power feeds (e.g. one computer with
  both a normal-bus and an essential-bus power input, each its own
  breaker) — `deep::electrical::loads.rs` does not model any such unit
  today (every load still has exactly one `bus` field), so there is
  nothing real to split against without fabricating ids (hard rule 3). The
  77-entry control/excitation-supply pass above (Group 3) is the concrete,
  honest piece of this instruction that *was* actionable without waiting
  on `loads.rs`. **Next loop: re-read `loads.rs` for the promised
  per-unit/per-side/per-channel split; for every unit that gains a second
  `bus`/becomes two separate load ids, add the matching second breaker
  here (reuse `push_electrical` twice, once per feed id) and update
  `COUNTS.md`/`FAILURES.md`.**
- **Cross-directory id verification is now a real compiled test**
  (`integration_test.rs`), superseding the "manual only" limitation noted
  in session 1 — but it still cannot run until the lead wires both areas
  into `deep/mod.rs`. Until then, keep re-reading `loads.rs` by hand each
  loop and reconciling `catalog.rs`'s `push_electrical` calls immediately.
- **`deep::electrical` had no `registry.rs`/`mod.rs` at the time session 1
  read it; it now has both** (`mod.rs` declares `pub mod loads; pub mod
  network; pub mod registry; pub mod shedding; pub mod sources;`) — still
  not wired into `src/deep/mod.rs` itself (`deep/mod.rs` only has `pub mod
  api;` as of this session), so neither area is reachable from the crate
  root yet.
- Next loop, in order: (1) re-read `loads.rs` for the per-feed/per-side
  split and reconcile as above — highest priority per the coordinator;
  (2) engine-mounted sensors/actuators breakers not yet covered (N1/N2/EGT/
  vibration probe excitation, thrust reverser actuator control); (3) ATA21
  avionics/environmental extras beyond the OCSM set already covered;
  (4) ATA27 secondary flight control surfaces once `deep::flight_controls`
  publishes its own load ids; (5) revisit the 25 A SSPC/thermal split and
  the overhead-panel ATA grouping against any more specific public figure
  if one turns up.

## Session 3 (hard-stop pass, ~23:05-23:29)

Re-read `deep::electrical::loads.rs` again per the coordinator's "keep in
exact step" instruction and found it had grown 609 -> 686 lines: the real
dual-feed split landed via a new `network::LoadFeed`/`add_dual`/`add_triple`
API. Reconciled immediately (this is the highest-priority item session 2's
own PROGRESS.md flagged):

- Added `push_electrical_feed`/`push_electrical_dual` to `catalog.rs`: a
  dual-fed load gets exactly two breakers, `<load>-normal-bkr` and
  `<load>-2nd-bkr`, both `protected_load: Some(load)` -- matching
  `add_dual`'s own breaker-id scheme exactly (verified by reading
  `add_dual`'s source in `loads.rs`, not guessed).
- `ata27`: all 11 units (ROLLOUT/FCU 1-2/PRIM 1-3/SEC 1-3/FCDC 1-2) are now
  dual-fed -- normal bus alternates Dc1/Dc2 by lane (changed from the old
  single DcEss/Dc2 alternation), DC ESS is the new universal backup feed.
  11 -> 22 breakers.
- `ata32`: LGCIU 1/2 now dual-fed (each one's old single bus is now its
  "normal" feed, the other of Dc2/DcEss is its new "2nd" feed) -- 2 -> 4
  breakers. Each of the 4 electric hydraulic pumps also gained a new
  sibling load, its own line-contactor holding-coil supply
  (`<pump-id>-coil`, 20 W GENERIC, DC ESS) -- 4 new single-fed breakers.
- `avionics_misc`: FMS 1-3/ADIRU 1-3/TCAS (7 units) now dual-fed (their old
  single bus is "normal", DC ESS or DC2 is the new "2nd" feed depending on
  unit); XPDR 1-2/VHF 1-2/WXR (5 units) confirmed still single-fed,
  unchanged. 12 -> 19 breakers.
- Fixed `catalog.rs`'s
  `every_electrical_group_entry_protects_itself_or_its_own_dual_feed_load`
  test (renamed from the old `..._one_breaker_per_load`): a dual-fed
  breaker's own id is now `<load>-normal-bkr`/`<load>-2nd-bkr`, not `load`
  itself, so the old exact-match assertion would have failed on every new
  dual-fed entry -- now accepts either form.
- Fixed `integration_test.rs`'s uniqueness test (renamed
  `every_protected_load_has_either_one_breaker_or_exactly_as_many_as_its_
  own_real_feed_count`): a real dual-fed load legitimately has *two*
  breakers now, so "no id claimed twice" was wrong; it now checks the
  claim count against `Load::feeds.len()` for every real load.
- `add_triple` exists in `loads.rs` (for a future triple-fed unit, e.g. the
  FWS) but has no call site yet -- nothing to reconcile against it.
- **New total: 397** (was 373). `mod.rs` still declares exactly
  `catalog`/`registry`/`trip` (+ `#[cfg(test)] mod integration_test;`),
  matching every `.rs` file on disk in this directory. `registry.rs` needed
  no changes -- it walks `catalog::all()` generically and does not hardcode
  a count, so the 24 new entries register automatically with their own
  nuisance-trip/fails-to-trip failure pair like every other entry.
- `COUNTS.md`'s ATA table was hand-recomputed and re-summed to confirm it
  equals 397 exactly; its panel/type tables are **not** re-verified this
  session (still the session-2/373 breakdown) due to the coordinator's
  hard stop at 23:29 -- next loop should re-run the Python cross-check
  (see session 1/2's own methodology) against the now-397 catalogue before
  trusting those two tables' numbers.

### Not started (per "do not start anything you cannot finish")

- Did not attempt engine-mounted sensor/actuator breakers, ATA21
  extras, or ATA27 secondary flight-control surfaces (session 1/2's own
  "next loop" list) -- no time left in the window; still the right next
  items once re-started.
- Did not re-verify `COUNTS.md`'s panel/type tables against the 397-entry
  catalogue (see above).
