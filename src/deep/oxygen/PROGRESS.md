# deep/oxygen — progress

New area. ATA 35, `deep::api::Area::Oxygen` (area code 20).

- [done] `gas.rs` — oxygen as a real gas — `src/deep/oxygen/gas.rs` — van der Waals
  EOS and its two inversions (mass for a pressure, volume for a charge), compressible
  orifice flow (choked and sub-critical branches), rigid-vessel blowdown cooling, the
  ICAO standard-atmosphere pressure-altitude relation. Re-derived here rather than taken
  from `crate::physics::gas` (BRIEF hard rule 2). Tests pin the ~10 % non-ideality of a
  charged cylinder and the exactness of the inversions.
- [done] `cylinder.rs` — a charged cylinder group — `src/deep/oxygen/cylinder.rs` —
  two-node thermal model (gas + steel wall), mass balance, leak orifice, overpressure
  discharge disc. Wall mass derived from the hoop stress a DOT 3AA/ISO 9809 burst factor
  allows, not quoted. Tests: pressure falls with use, falls with temperature at constant
  mass, sags further than the mass justifies during a discharge and recovers afterwards,
  burst disc relieves when a bay fire heats the bottle.
- [done] `regulator.rs` — pressure reducer and diluter-demand schedule —
  `src/deep/oxygen/regulator.rs` — the dilution schedule is solved from the alveolar gas
  equation, so the pure-oxygen crossover comes out at ~33 000 ft rather than being drawn
  in; a test asserts it against the published 33-34 000 ft figure.
- [done] `crew.rs` — 35-10 — `src/deep/oxygen/crew.rs` — cylinder group, motor-operated
  supply shutoff valve, reducer, low-pressure distribution, four mask regulators,
  temperature-corrected low-pressure logic, endurance.
- [done] `generator.rs` — one chemical oxygen generator — `src/deep/oxygen/generator.rs` —
  candle chemistry (chlorate decomposition + iron fuel burn-back) from standard enthalpies
  of formation, so the oxygen yield *and* the heat come from one derivation; case
  temperature from an energy balance, cross-checked against the published ~260 C exterior.
- [done] `pax.rs` — 35-20 — `src/deep/oxygen/pax.rs` — two decks of generators,
  presentation vs ignition as separate mechanisms, automatic (cabin altitude) and manual
  deployment paths, per-deck cabin heat load in watts.
- [done] `therapeutic.rs` — 35-30 — `src/deep/oxygen/therapeutic.rs` — first-aid cylinder
  sized from 14 CFR 121.333(e), continuous-flow outlets.
- [done] `registry.rs` — 14 components, 21 failures, 3 ECAM alerts, with `registry::ids`
  as the single place the failure numbering is written down.
- [done] `live.rs` — the `deep::live::Area` implementation, plus the three wiring lines
  (`api.rs`'s `Area::Oxygen`, `deep/mod.rs`'s module + `registry()` call, `deep/live.rs`'s
  `all_areas()` entry).

## New Vars this area publishes

`OXYGEN_BOTTLE_PRESSURE_PA:1` (crew cylinder group) and `:2` (first-aid cylinder), both
**absolute** Pa — the name `deep::sensors::live_discrete::BLOCKED` has been waiting on.
Everything else is prefixed `DEEP_OXY_` so that it cannot collide with the crate-root
`src/oxygen.rs` module, which is live in the plugin and writes `OXYGEN_CREW_*` /
`OXYGEN_PAX_*` every tick. See `live.rs` for the full list; the ECAM triggers in
`registry.rs` read `DEEP_OXY_CREW_LOW_PRESSURE`, `DEEP_OXY_CREW_SUPPLY_AVAILABLE`,
`DEEP_OXY_CREW_BOTTLE_GAUGE_PSI`, `DEEP_OXY_PAX_MASKS_DEPLOYED` and
`DEEP_OXY_PAX_CABIN_ALTITUDE_FT`, all of which this area publishes itself.

## `Truth` requests

Three real states have no `Truth` field and no published source anywhere under `deep/`.
They are held at their real resting values rather than invented (`live.rs` says so in its
header), and this is the list for whoever extends `Truth`:

1. **`crew_oxygen_mask_donned: [bool; 4]`** and **`crew_oxygen_mask_mode: [u8; 4]`**
   (N / 100% / EMER). Without them the crew cylinder is only drawn down by modelled
   leaks, so normal consumption never runs in the sim. There *is* a real Var for the
   first — `src/oxygen.rs` reads `"OXYGEN CREW MASK ON"` — so this is a plumbing job, not
   a sourcing one.
2. **`pax_oxygen_mask_man_on: bool`** — the flight deck's MASK MAN ON command. The
   automatic altitude path works today; the manual override is modelled and unreachable.
3. **`first_aid_oxygen_outlets_in_use: f64`** — how many therapeutic outlets the cabin
   crew have a mask plugged into. No Var exists for this anywhere in the port.

## For other areas

- `DEEP_OXY_PAX_MAIN_DECK_HEAT_W` / `DEEP_OXY_PAX_UPPER_DECK_HEAT_W` are a real heat
  source for `deep::thermal_zones`' `CabinMainDeck` / `CabinUpperDeck` zones — tens of
  kilowatts during a deployment. Nothing consumes them yet.
- `deep::sensors` can now instantiate `35_oxy.pressure_1` and `35_oxy.pressure_2` and
  drop that row from its `BLOCKED` table.
