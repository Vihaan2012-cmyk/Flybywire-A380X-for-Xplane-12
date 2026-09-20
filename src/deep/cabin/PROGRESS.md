# Progress — deep/cabin (ATA 25/38/44/52)

Order followed: the backlog in `docs/deep/BRIEF.md`'s task, items 1-6, then
registration (mandatory addendum) backfilled for each finished item.

- [done] 1. Potable water (ATA 38) — `water.rs` — tanks/quantity, bleed/
  compressor pneumatic pressurisation via the ideal gas law, distribution to
  galleys/lavatories/first-class showers (airline option flag), point-of-use
  water heaters, drain-mast heaters and freezing (heat balance + latent-heat
  ice accumulation), leaks (orifice flow), stuck quantity sensor, and
  `water_mass_kg` output as the CG-input hook the backlog asks for (not
  wired into `weight_balance.rs`'s `Mass`/`Balance` — out of this directory
  per the self-containment rule; whoever wires this area in adds a `Mass`
  entry from `WaterOutputs::water_mass_kg` at the tank's real station).
- [done] 2. Waste (ATA 38) — `waste.rs` — vacuum toilets using the cabin/
  ambient differential (natural vacuum) with an electric generator fallback,
  zoned waste tanks, tank-full -> lavatories inoperative by zone, stuck
  level sensors, flush valve stuck-open/stuck-closed faults, rinse water
  draw returned per zone (`WasteOutputs::rinse_used_l`) for whoever wires it
  against `water.rs`'s tank.
- [done] 3. IFE and cabin power (ATA 44/25) — `ife.rs` — per-zone seat
  electronics loads, two redundant IFE head-end servers, commercial/seat-
  power shedding, a per-zone short->overheat->smoke thermal fault chain
  (with a flavour-only seat/port label generator, e.g. "seat 43K USB-C"),
  IFE server failure with redundancy.
- [done] 4. Galleys — `galley.rs` — ovens (thermostat + stuck-closed
  overheat-to-smoke fault), chillers (heat-balance pulldown, compressor
  failure lets the compartment drift to cabin ambient), water boilers,
  and a galley bus-feed fault (independent of the aircraft-wide commercial
  shed input both this and `ife.rs` accept).
- [done] 5. Doors and slides (ATA 52) — `doors_slides.rs` — door seal leak
  as a pressurisation-interface orifice, slide arm/fire logic with a real
  nitrogen-bottle ideal-gas pressure and a slow-leak fault, door not-latched
  sensor with a stuck-reading fault, hydraulically driven cargo door
  actuator with jam and hydraulic-loss faults. Deliberately independent of
  `doors.rs` (crate root): takes door position/hydraulic pressure as plain
  inputs rather than importing `doors::Door`, per the self-containment rule.
- [done] 6. Cabin crew calls — `crew_calls.rs` — attendant/purser/emergency/
  cockpit call buttons with edge-triggering and real interphone priority
  (emergency > purser > normal), plus automatic calls raised from a plain
  `CabinSnapshot` (smoke, waste-tank-full, water-system-fault, door/slide
  conditions) so this module stays decoupled from the other five's own
  types. One "highest priority unacknowledged call" state for a simple
  flight-deck annunciator, with `acknowledge`/`acknowledge_all`.
- [done] Registration addendum — `registry.rs` — every component, failure
  and ECAM alert above registered through `deep::api::Registry` (`pub fn
  register(r: &mut Registry)`, declared in `mod.rs`). See "New Vars" below:
  none of them exist in the plugin yet (this directory's models are not
  wired to `Vars`), so every `var(...)` name in `registry.rs`'s alert
  triggers/procedures is a contract for whoever wires this area in.

## New Vars this area's registry.rs assumes (none published today)

- `CABIN_WATER_QTY_PERCENT`, `CABIN_WATER_PRESS_PSI`,
  `CABIN_WATER_COMPRESSOR_CMD`, `CABIN_MAST_BLOCKED:1`/`:2`
- `CABIN_WASTE_TANK_FULL:1..3` (not currently read by any alert trigger,
  reserved for a future per-zone alert if the lead wants one; `lav_inoperative`
  is already an `WasteOutputs` field)
- `CABIN_IFE_ZONE_SMOKE:1..3`, `CABIN_SEAT_POWER_CMD`,
  `CABIN_IFE_SERVER_FAIL:1`/`:2`
- `CABIN_GALLEY_OVEN_SMOKE:1..3`, `CABIN_GALLEY_OVEN_CMD`,
  `CABIN_GALLEY_BUS_FAULT:1..3`
- `CABIN_DOOR_LATCHED:<n>`, `CABIN_SLIDE_PRESSURE_LOW:<n>`,
  `CABIN_CARGO_DOOR_JAMMED:<n>`, `CABIN_CARGO_DOOR_CMD:<n>` (registered for
  door index 1 as the representative instance; `doors_slides.rs`'s model is
  generic per door, so the real integration repeats these per `doors::NAMES`
  entry that has a seal/slide/latch/actuator)

## Notes for whoever wires `deep::cabin` in

- Every model here is std-only, X-Plane/`Vars`-free, and independently
  tested (`cargo test` per module once the crate builds). None of the six
  modules imports another; `crew_calls.rs`'s `CabinSnapshot` and
  `doors_slides.rs`'s plain `f64`/`bool` inputs are the intended seams.
- `water.rs` and `waste.rs` are meant to be cross-wired (waste's per-flush
  rinse volume subtracted from water's tank) by whoever integrates, not by
  either module importing the other.
- Extending the backlog: natural next items in this area would be lavatory
  smoke detectors as their own component (currently folded into
  `crew_calls.rs`'s snapshot as a plain bool with no dedicated physical
  model — a real ATA 26/38 lavatory smoke detector with an obscuration
  threshold and a battery/self-test fault would be the next physically
  grounded addition), and a passenger oxygen/cabin-crew-call cross-check
  with `oxygen.rs`'s existing mask-deployment state.

## 2026-09-20 — dead-failure audit follow-up (12 failures closed or explained)

`deep::integration::failure_audit`'s sweep found 12 of this area's failures dead, both traced to the same root cause: `CabinCommands` (this area's own stand-in for inputs `Truth` did not carry) held `galley_demand_l_s`/`lav_demand_l_s`/`cargo_door_target_percent` at a permanent 0.0 default that nothing in `Truth` or the audit ever set, so the potable-water system never actually flowed and the cargo-door actuator never had anywhere to go. `Truth::controls` now carries both for real (`water_demand_l_s: [galley, lavatory]`, `cargo_door_commanded_open: [fwd, aft, bulk]`), so both fields were removed from `CabinCommands` entirely and `CabinLive::tick` reads the real values instead:

- `water_inputs.galley_demand_l_s`/`lav_demand_l_s` now come from `truth.controls.water_demand_l_s[0]`/`[1]` (plus the waste system's own rinse draw, unchanged).
- `door_inputs.cargo_door_target_percent` now comes from `truth.controls.cargo_door_commanded_open[0]` (the forward door — this model registers one cargo-door actuator as its representative class, per `registry.rs`), scaled to the 0..100 percent the actuator model expects and stored in a new `cargo_door_commanded_percent` field so `publish` (which only ever sees `&self`) can still report `CABIN_CARGO_DOOR_CMD:1`.

Traced which of the 12 failures needed which half of the fix, by reading `water.rs::WaterSystem::step` directly rather than guessing:

- **Needed real demand:** the potable water quantity sensor (`38_wtr.qty_sensor`) — with `water_l` never draining, "frozen at the last reading" and "tracking the real level" read identically (both ~100%). New test: `a_stuck_water_quantity_sensor_only_shows_once_real_demand_drains_the_tank`, driven purely through `Truth` (no `CabinCommands` write at all).
- **Needed a real target:** both cargo-door actuator failures (jam, hydraulic loss) — a target that never moved off 0% meant a jam's `(1-jam)*100%` cap and a hydraulic loss's "cannot move at all" were both trivially satisfied already at rest. `a_jammed_cargo_door_actuator_caps_its_travel_and_reports_the_fault` (pre-existing) now drives this through `truth.controls.cargo_door_commanded_open` instead of the removed `CabinCommands` field; its assertions are unchanged.
- **Already independently live (not part of the demand fix, checked by reading `water::WaterSystem::step`):** the three zone water-heater faults and both drain-mast heater faults are a standalone thermal balance against `heater_commanded`/OAT with no dependency on flow at all (`a_failed_drain_mast_heater_lets_the_mast_ice_up_in_cold_air`, pre-existing, still passes); the potable-water leak scales with `flow_fraction.max(pressure_ratio)`, so a pressurised tank leaks whether or not anything is drawing from it. If any of these five were still in the dead list, the cause is a different one this pass did not find (e.g. `bleed_available`'s own gauge-pressure threshold against the profiles' actual bleed pressures) and is not claimed fixed here.
- The remaining waste-system failures (generator, level sensors, flush valves) are driven by `flush_commanded`, which stayed in `CabinCommands` — no real `Truth::controls` field exists for a passenger flushing a toilet, so those are left exactly as before (genuinely `d`, a state this pass does not reach, not touched to avoid a fake input).

## Live system (deep push, `live.rs`)

- [done] `live.rs` — the area's live instance behind `crate::deep::live::Area`
  (`live_system() -> Box<dyn Area>`), declared in this directory's `mod.rs`.
  `tick` drives the real models from `deep::live::Truth` and applies every
  failure `registry.rs` registers by reading `Faults::get(id)` into the exact
  `model_field` that entry names; `publish` emits every variable this area's
  ECAM triggers cite, plus the state behind them for the EFB Study pages.
  Failure ids are resolved at construction by registering into a throw-away
  `Registry` and looking each one up by component + `model_field`, so a
  renumbering in `registry.rs` fails loudly instead of silently unhooking a
  failure. Inputs `Truth` does not carry yet are collected in one documented
  `...Commands` struct per area rather than invented.
