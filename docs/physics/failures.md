# Failures, damage, MEL and persistence (hyperrealism workstream 6)

Source table for every threshold/parameter used by `src/failures.rs`,
`src/physics/damage.rs`, `src/mel.rs`, `src/random_failures.rs` and
`src/persistence.rs`. "Generic" means no public A380-specific figure was
found; the value is a defensible, order-of-magnitude industry convention,
clearly marked in the code as not cited to Airbus/Rolls-Royce.

## Engine exceedance (damage.rs, Trent 900)

| Parameter | Value | Source |
|---|---|---|
| Max continuous TGT | 850 C | EASA.E.012 Issue 12, Trent 970-84/972-84/972E-84 TCDS §IV.1.2 "Turbine Gas Temperature (TGT) - Trimmed" |
| Max take-off TGT (5 min) | 900 C | EASA.E.012 §IV.1.2 Note 5 |
| Max overtemperature (20 s) | 920 C | EASA.E.012 §IV.1.2 Note 14 |
| Take-off time limit | 5 min | EASA.E.012 §IV.1.2 Note 5 |
| OEI take-off time limit | 10 min | EASA.E.012 §IV.1.2 Note 5 ("the take-off rating ... may be used for up to 10 minutes in the event of an engine failure") |
| Overtemperature time limit | 20 s | EASA.E.012 §IV.1.2 Note 14 |
| GP7200 equivalent figures | noted, unused | EASA.IM.E.026 Issue 03 (4 Jan 2013) — kept for when an engine choice is added; FlyByWire's package models a Trent 972B-84 (`physics/engine/params.rs`), so Trent 900 is primary |
| Compressor efficiency loss ceiling | 15%, saturating on creep-life fraction | Derived, order-of-magnitude ceiling; not a cited figure |
| "Engine running" EGT proxy | 150 C | Derived heuristic (idle EGT is far above ambient, `fadec.rs`'s `idle_egt`); only used for the hours/cycles wear clock, never for a certification threshold |

## Flap/gear/speed limits (damage.rs)

| Parameter | Value | Source |
|---|---|---|
| VFE by CONF (0-4) | inf, 263, 220, 196, 182 kt | FlyByWire `flight_model.cfg` `[FLAPS.1]` `flaps-position.N` airspeed-limit column, lines 887-892 |
| VLE/VLO | 250 kt | FlyByWire `flight_model.cfg` `[REFERENCE SPEEDS] max_gear_extended`, line 801 |
| VMO | 390 kt | FlyByWire `flight_model.cfg` `[REFERENCE SPEEDS] max_indicated_speed`, line 787 |
| MMO | 0.97 | FlyByWire `flight_model.cfg` `[REFERENCE SPEEDS] max_mach`, line 788 |
| MLW | 386,000 kg | Airbus "A380 Aircraft Characteristics - Airport and Maintenance Planning" (WV000, MTOW 510,000 kg), matching FlyByWire's own `max_gross_weight` (flight_model.cfg:16) |
| Tailstrike pitch | 12.99 deg | Derived from FlyByWire's own contact-point geometry: `atan((15.397 ft height diff) / (66.702 ft distance))` between `point.1`/`point.2` (aft body gear) and `point.17` (tailstrike point), `flight_model.cfg` |
| Hard landing | VS <= -600 fpm or Nz >= 1.6 g | Generic transport-category maintenance-manual inspection trigger; no public A380-specific figure found |

## Brake energy (damage.rs)

| Parameter | Value | Source |
|---|---|---|
| Reference kinetic energy | 0.5 x MLW x (72 m/s)^2 | Derived: MLW at a typical heavy-jet reference approach speed (140 kt), first-principles kinetic-energy scale; no published brake-specific energy limit found — marked generic |
| Thermal cooling time constant | 600 s | Generic transport-category brake thermal time-constant order of magnitude (no forced-fan cooling); marked generic |
| Fuse-plug melt threshold | 1.5x the reference brake-energy scale | Derived scale, generic |

## APU (damage.rs)

| Parameter | Value | Source |
|---|---|---|
| EGT overtemperature trigger | live `APU_EGT` above live `APU_EGT_WARNING` for 60 s | The two variables are the ported systems' own published values; the 60 s sustained-overtemperature window is a generic trigger — no published APU time limit found |

## MEL (mel.rs)

| Parameter | Value | Source |
|---|---|---|
| Category A interval | 24 h (model default; no universal bound) | FAA AC 25-1591 / EASA MMEL Policy: category A uses the interval the item itself states; this model has no per-item text for its generic items, so it uses the tightest of the standard bands as the conservative default |
| Category B interval | 3 calendar days (72 h) | FAA AC 25-1591 / EASA MMEL Policy |
| Category C interval | 10 calendar days (240 h) | FAA AC 25-1591 / EASA MMEL Policy |
| Category D interval | 120 calendar days (2880 h) | FAA AC 25-1591 / EASA MMEL Policy |
| Calendar-day to flight-hour conversion | x24 (counts whether or not the aircraft flies) | Conservative reading of the MMEL policy's "calendar day" wording |
| Item categories (extra catalogue) | per item, `failures::extra` | Assigned by failure-mode type (LRU redundancy = C, flight-control/fuel-metering-critical = B, structural/no-fix = None) since no cited Airbus A380 MMEL item text exists for these generic items; explicitly marked as generic in `failures.rs`'s `mel_item` doc comment |

## Random failures (random_failures.rs)

| Parameter | Value | Source |
|---|---|---|
| Avionics/electronic LRU MTBF | 25,000 FH | Low end of the 10,000-50,000 FH band commonly quoted for certified avionics LRUs (no public per-LRU A380 figure exists; Airbus does not publish component-level reliability data) |
| Hydraulic/fuel pump, valve MTBF | 6,000 FH | Low end of the 4,000-8,000 FH band commonly quoted for aircraft hydraulic pumps |
| Mechanical/structural MTBF | 15,000 FH | Generic mid-band estimate for bearing wear / tyre-burst arming components |
| APU component MTBF | 12,000 FH | Generic estimate, between the avionics and hydraulic bands |
| FADEC/fuel-control/ignition/reverser/oil/starter MTBF | 20,000 FH | Generic estimate for certified engine-accessory LRUs |
| Reliability model | constant hazard rate, `p = 1 - exp(-dt/mtbf)` | Standard exponential/constant-hazard-rate reliability model (ARP4761-style safety assessment convention) |
| PRNG | xorshift64* | Marsaglia 2003 / Vigna 2014 multiplier; chosen for determinism (seeded, replayable), not cryptographic use |

## Persistence (persistence.rs)

| Parameter | Value | Source |
|---|---|---|
| Save interval | 120 s real time | Chosen so a crash loses at most 2 minutes of wear/damage/MEL history; not a cited figure |
| Write method | temp file + atomic rename | `std::fs::rename` is atomic on the same volume on both Windows and POSIX; corruption-safe against a crash/power-loss mid-write |
| Save path | `Output/preferences/fbw_a380x_airframe.json` | X-Plane's own per-aircraft preferences convention (`Output/preferences/`) |

## Failure catalogue (failures.rs)

| Parameter | Value | Source |
|---|---|---|
| Original systems catalogue | 146 ids | `a380_systems_wasm` `lib.rs:86-428`, mirrored id for id |
| Computer faults | 11 ids | `fbw_a380`'s `FailuresConsumer` (`FailuresConsumer.cpp:27-39`, `FailureList.h`) |
| Extra catalogue (workstream 6) | ~136 ids | New; every id sits in an ATA chapter/range the original catalogue never used, so no collisions. Per-item sourcing is in each `failures::extra` function's doc comment (engine components generic industry failure modes, ADIRS/pneumatic/hydraulic/fuel items generic system failure modes with no FlyByWire `FailureType` equivalent to extend) |
| Total | 250-300 (currently 293) | Per the hyperrealism brief's target band; enforced by `failures::extra::tests::the_catalogue_totals_within_the_requested_band` |

## Continuous failure magnitude (failures.rs)

A failure is a continuous physical quantity, not a binary flag or a
severity bucket. `failures::magnitude(id) -> f64` returns a fraction in
`0.0..=1.0`; `failures::is_active(id)` is defined as `magnitude(id) > 0.0`
and stays the source of truth for every existing binary consumer
(`FailuresConsumer::isActive`, `Simulation::update_active_failures`, the
Study panel's old `"active"` bool) — nothing that already reads `is_active`
needed to change.

**The contract for a consumer:** `magnitude(id)` is the fraction of the
physical quantity the failure takes away or perturbs, applied directly —
never mapped through a severity tier ("minor/major/hazardous"), a
lookup table of discrete levels, or a step function. A hydraulic-pump
failure at `magnitude = 0.4` means the pump delivers 60% of its rated
flow/pressure (a 40% loss), continuously variable as the magnitude
changes tick to tick — not "40% chance the pump is fully failed" and not
"snap to the nearest of {0%, 50%, 100% loss}". A sensor failure at
`magnitude = 0.7` means its reading is perturbed/degraded by a factor tied
to 0.7, not "70% of the time report garbage." Consumers that have not yet
been updated to read `magnitude` and still branch only on `is_active`
implicitly treat every active failure as `magnitude = 1.0` (full loss) —
correct for a legacy binary activation (`set_active`/`toggle`, which arm at
`1.0`), but they should move to reading `magnitude` directly once their own
physical model supports a partial loss, rather than staying a flag forever.

**API** (`failures.rs`):
- `set_magnitude(id, m)`: arms `id` at continuous fraction `m` (clamped to
  `0.0..=1.0`); `m <= 0.0` clears the failure the same as `set_active(id,
  false)`.
- `magnitude(id) -> f64`: `id`'s current fraction, `0.0` if inactive or
  unregistered.
- `set_active`/`toggle`: unchanged legacy entry points, arming/clearing at
  full magnitude (`1.0`) — a plain on/off caller (an X-Plane command, a
  random-failure draw that doesn't pick a magnitude) still gets "fully
  failed," which is a special case of the continuous contract, not a
  separate code path.
- `active_magnitudes() -> BTreeMap<u64, f64>`: every active id with its
  magnitude, for persistence (`persistence::AirframeState::
  active_failure_magnitudes`) and the Study panel's `/study/failures`
  JSON (`"magnitude"` field, additive alongside the existing `"active"`
  bool).
- `restore_magnitudes(...)`: persistence-load counterpart to
  `active_magnitudes()`.
- `reset_all()`: test-only, clears every active id and magnitude
  (process-global state — call at test SETUP, same reason `wear::
  reset_all()` exists).

**Pickers:** `scripted_failures.rs` and `random_failures.rs` can each choose
a magnitude for the failure they arm, rather than always arming at `1.0`
(a scripted trigger can specify a partial-loss magnitude in its
definition; a random-failure draw can sample one). Neither is required to
by this contract — arming at `1.0` remains a valid draw — but a fixed
`1.0` everywhere would make every random/scripted failure binary in
practice, defeating the point.

**Study panel:** `POST /study/action` accepts
`{"kind":"setFailureMagnitude","id":<u64>,"magnitude":<0.0..=1.0>}`
(`study/web.rs`'s `apply_action`), calling `failures::set_magnitude`
directly — the same queued-then-applied-on-tick path every other Study
action uses.

## Hook variables, by owning model

Each hook is a plugin `Var` this workstream drives to 1.0/0.0 with the
failure's active state; the owning model reads it and applies the physical
effect. None are consumed yet (each owning workstream's report should note
when it starts reading its column).

## Study/Failures tab JSON: cause/components/trigger fields

Added to `study/web.rs`'s `failures_json()` (additive only, shape kept):
`cause` (`failures::cause_description`), `affectedComponents`
(`failures::affected_components`), `triggerCondition`
(`failures::trigger_condition`). For the extra catalogue these reuse the
per-item description/effect already in `failures::extra`; for the original
157-id catalogue and the 11 computer faults they are derived generically
from the `FailureType`/computer name (the LRU the id itself names), since
no further per-id source text exists for those. `trigger_condition`
mirrors `random_failures.rs`'s own `damage_armed` exclusion list (kept in
sync by hand -- if that list changes, update both) so the tab can tell a
manual-only id apart from one the MTBF engine or `physics/damage.rs`'s
wear/exceedance model can also arm.

**Known gap, not done this session:** the MEL (`mel.rs`) has no Study-panel
UI yet -- `mel::request_defer`/`request_repair` exist and are ticked
(`lib.rs`), but no window or web action calls them, and `failures_json`
does not yet expose per-id MEL/deferred state (would need a
`mel::publish`/`snapshot` pair mirroring `physics/damage.rs`'s pattern,
since `Mel` is a private `Plugin` field). Time-boxed out by an interim
10-minute deadline; next session should add that plumbing plus a
Study "MEL" page and Ground Services repair buttons. Time/altitude/speed/
flight-phase scripted arming (brief item 4's first bullet) is also not yet
built -- only manual arm and MTBF-random exist as trigger sources today.

## Hook variables, by owning model

| Var | Owner | Failures using it |
|---|---|---|
| `FAIL_BUS_SHORT_HOOK` | Electrical | 18 bus short-circuit ids, 24_200-24_217 |
| `FAIL_FUEL_HOOK` | Fluids | Feed pumps, trim transfer pump, cross-feed valves, jettison valve, 28_000-28_012 |
| `FAIL_HYDRAULIC_HOOK` | Fluids | Filter clogging, PTU, local electric pumps, contamination, 29_100-29_105 |
| `FAIL_ADIRU_SENSOR_HOOK` | ADIRS | Per-ADIRU pitot/static/AOA faults, 34_100-34_108 |
| `FAIL_ADIRU_INTERNAL_HOOK` | ADIRS | Per-ADIRU internal fault, 34_109-34_111 |
| `FAIL_BLEED_DUCT_LEAK_HOOK` | Air | Per-engine bleed duct leak, 36_000-36_003 |
| `FAIL_PRECOOLER_HOOK` | Air | Per-engine precooler fault, 36_004-36_007 |
| `FAIL_BLEED_VALVE_HOOK` | Air | Per-engine HP bleed valve stuck, 36_008-36_011 |
| `FAIL_APU_FUEL_CONTROL_HOOK` | Fluids | APU fuel control fault, 49_001 |
| `FAIL_APU_STARTER_HOOK` | Electrical | APU starter fault, 49_002 |
| `FAIL_APU_BLEED_VALVE_HOOK` | Air | APU bleed valve stuck, 49_003 |
| `FAIL_APU_OIL_LOW_HOOK` | Fluids | APU oil low, 49_004 |
| `FAIL_ENGINE_COMPONENT_HOOK` | Engine | 14 per-engine component families, 72_000-80_003 (56 ids) |
| `ENGINE_CREEP_LIFE_FRACTION:n` | Engine | Wired now (damage.rs writes it every tick); the engine model should turn it into higher EGT/lower efficiency |
| `ENGINE_COMPRESSOR_EFFICIENCY_LOSS:n` | Engine | Wired now (damage.rs writes it every tick, saturating at 15%) |

## Study panel quantities

Per the team brief, every fix must be visible in the Study panel. Fields to
show, all already exposed through a small public API (`mel::Mel::list`,
`physics::damage::snapshot`, `random_failures::RandomFailures`'s `config`):

- Per engine (4): `creep_life_fraction`, `compressor_efficiency_loss`,
  `hours`, `cycles` (`physics::damage::snapshot`).
- Active failures: `failures::active_ids()` with `failures::any_failure_name`.
- MEL page: `mel::Mel::list(now_hours)` — id, name, category, hours
  remaining; actions `mel::request_defer(id)` / `mel::request_repair(id)`.
- Random failures page: `RandomFailures::config` (enabled, rate multiplier);
  action `random_failures::request_config(Config { .. })`.
- Airframe totals: `persistence::AirframeState::airframe_hours`,
  `apu_hours`, `consumables_last_observed`; action: a "new airframe" reset
  button calling `Persistence::reset`.
- Engine maintenance: a "repair engine n" button calling
  `physics::damage::request_repair_engine(n)`.
