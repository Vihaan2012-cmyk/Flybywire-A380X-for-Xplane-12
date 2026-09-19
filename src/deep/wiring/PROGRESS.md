# Wiring -- progress

Directory: `D:\fbw-xp-systems\src\deep\wiring\`. Self-contained (no dependency on
`crate::breakers`/`crate::circuits`/`crate::physics`/other `deep::*` areas' code --
this push's hard rule 2); `breakers.rs`/`circuits.rs` were read in full for context
(real bus assignments, consumer descriptions, the 265-breaker catalogue's own ids)
and reproduced as literal data in `routing.rs`, never imported.

- [done] Wire bundle model (backlog 1) -- `gauge.rs` (AWG resistance/metre vs temperature,
  cited NEC Chapter 9 Table 8 + IACS alpha; MIL-DTL-22759's 3 insulation classes),
  `bundle.rs` (`Segment`, `CircuitWire`, `WireBundleNetwork` with per-circuit route
  resistance summed across its own segments, each at its own segment's ambient). Tests:
  resistance scales with gauge/length/temperature, no NaN/negative at extreme cold,
  unlisted AWG falls back safely (fixed an infinite-recursion bug in the fallback path
  during review -- `nearest_tabulated` must never return an untabulated size).
- [done] A380 routing (backlog 2) -- `routing.rs`: `Side` (Side1/Side2/Ess/Apu/Ground)
  segregation classes from each circuit's real bus label (`side_of_bus`, matching
  `breakers.rs`'s own AC1-4/DC1-2/ESS assignments), `side_of_engine` (1/2 left,
  3/4 right), a ~50-circuit representative slice of `breakers.rs`'s real catalogue
  (TRs, generators, APU generators, static inverter, LGCIUs, gear/door proximity
  sensors and actuators, fire loops A/B x6 zones, radio altimeters, EGPWC, PRIM/SEC/
  FCDC, engine bleed valves, electric hydraulic pumps, cabin fans, cargo isolation/
  extract/heater), each with a real-bus-grounded, GENERIC-labelled zone path and
  gauge/insulation. `build_generic_a380_network()` builds one segment per
  `(zone, side)` actually used, so Side1/Side2 circuits crossing the same zone never
  share a physical bundle, and fire loop A/B get their own per-loop segment id even
  though nominally the same side, matching real physical loop separation. Tests:
  no Side1/Side2 segment ever mixes (`segregation_violations` empty), gen-1/gen-3
  cross wing-root in different segments, loop A/B never share a segment but both
  genuinely cross the same engine zone, every catalogued circuit gets a route.
- [done] Faults (backlog 3) -- `faults.rs`: `CircuitEffect` (Open/ShortToStructure/
  HighResistance/CrosstalkShort) and 7 fault kinds (chafe, bundle overheat/fire,
  connector corrosion, water ingress, rodent damage, maintenance damage, open wire),
  each a continuous function of magnitude, cited/GENERIC per function doc. Chafe's
  arc-floor resistance cites public arc-fault voltage-drop literature (UL 1699 /
  IEEE series-arc characterization, ~25-40 V, taken as 30 V). Zone-wide overheat
  checks each circuit's own `Insulation::max_temp_c()` against the fire's own
  severity-scaled temperature, so a 260 C-rated wire in the same bundle as a
  150 C-rated one genuinely survives a fire that opens its neighbour -- a real,
  derived difference, not asserted. Tests per function plus the cross-cutting one
  (mixed-insulation segment, one wire opens, the other survives) and a full-zone
  fire opening both fire-loop A and B (the zone-wide/segment-wide distinction the
  backlog draws).
- [done] Arc model (backlog 4) -- `arc.rs`: `arc_current_a` (KVL with the arc's own
  constant voltage drop, lower than a bolted short by exactly that drop),
  `arc_heat_w` (localized `I*V` at the fault point -- a plain wattage for whichever
  thermal-zone model owns that location to `inject_heat_w`, never called directly:
  plain-data interface, per the task), `thermal_equivalent_current_a`/
  `thermal_breaker_sees_ratio` (RMS-of-a-duty-cycle reasoning showing a low-duty
  intermittent arc can keep a conventional thermal breaker's own I^2t element under
  its rated current indefinitely -- the real, documented motivation for arc-fault-
  specific protection, cited). Tests: arc current below bolted-short current by the
  arc's own drop, an arc that cannot sustain reads zero, low duty keeps the ratio
  under 1.0 while continuous duty exceeds it, duty is clamped not amplified, no NaN
  at zero resistance/rating.
- [done] Queries (backlog 5) -- `query.rs`: `circuits_through_zone`, `bundle_burn_effects`
  (every circuit sharing one physical segment opens -- proven distinct from a
  zone-wide fire by the fire-loop A/B test), `bundle_mates`. Tests included.
- [done] Registration -- `registry.rs`: `Area::Wiring` (12), ATA 91 ("Wiring Diagrams",
  the real ATA100/iSpec2200 chapter for wiring diagram manuals). 13 harness
  components (one per zone the routing catalogue actually threads a circuit
  through -- `UpperAvionics` excluded, nothing routed there yet), each with the 7
  fault-kind health params; 91 failures (13 zones x 7 kinds); 13 `BundleOverheat`
  ECAM cautions (one per zone) -- the only fault kind this module raises its own
  alert for, since every other kind's real crew-visible symptom is whatever
  downstream system loses power (that system's own already-registered alert, not
  duplicated here). `Registry::validate()` passes (own test).

## New Vars this model will need once integrated (none published yet -- self-contained per rule 2)

- `WIRING_ZONE_<ZONE_NAME>_OVERHEAT_SEVERITY` (0..1): drives `registry.rs`'s 13
  `WIRING_OVHT_<ZONE>` ECAM cautions. Whoever wires a real chafe/overheat scenario
  in should publish this from `faults::zone_overheat_effects`'s own input severity.
- `WIRING_SEGMENT_<SEGMENT_ID>_FAULT_SEVERITY` (0..1, per catalogue segment id):
  the natural place for a scenario/failure-injection harness to drive `chafe_effect`/
  `rodent_damage_effect`/etc.'s own magnitude per segment; not yet consumed by any
  code (`bundle`/`faults`/`routing`/`arc` all take magnitude as a plain argument).

## Not done / left for a future pass

- `routing::catalogue()` covers ~50 of `breakers.rs`'s ~265 real breakers (every ATA
  chapter it defines, but not every individual entry within ATA21's FDAC/TADD/VCM/
  OCSM channel set or every CPIOM-B application) -- extend the table, not the
  network-building code, to cover more.
- `UpperAvionics` zone has no circuit routed through it yet in the catalogue, so it
  has no registered harness component either; add both together if a future
  circuit needs it.
- No live integration with `deep::electrical::network` or `deep::thermal_zones::network`
  (by design -- plain circuit-id/zone-id interface only, per this task). A real
  integration layer would: (a) feed each circuit's `bundle::circuit_resistance_ohm`
  into the electrical network's own per-load wiring resistance term, (b) feed
  `arc::arc_heat_w`'s output into the thermal network's `inject_heat_w` for the
  fault's own zone, and (c) feed a zone's own live temperature back into
  `faults::zone_overheat_effects`'s severity input instead of a scenario-supplied
  constant.
