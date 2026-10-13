# Fuel deep model — failure catalogue

Area: `Area::Fuel` (19, see `PROGRESS.md`'s note at the top). ATA 28
throughout. One line per genuinely distinct physical fault; the source of
truth is `registry.rs` (this file is the human-readable index of it).

Format: `ATA | proposed name | model element it acts on | magnitude meaning (0..1) | effect`

| ATA | name | model element | magnitude (0..1) | effect |
|---|---|---|---|---|
| 28 | `<TANK>` tank baffle/rib damage (x11 tanks) | `geometry.rs::TankShape.slosh_damping_ratio` | fraction of nominal slosh damping lost | free surface sloshes longer/further per disturbance; higher transient unporting and structural slosh load risk |
| 28 | `<TANK>` FQMS probe failure (x11) | `gauging.rs::ProbeFault.failure_fraction` | one probe's own failure fraction | past 0.9 BITE excludes it (confidence falls); below that it drifts, biasing indicated quantity |
| 28 | `<TANK>` FQMS compensator fault (x11) | `gauging.rs::fqms_indicated_fraction` common-mode bias | fraction of `MAX_PROBE_BIAS_FRACTION` applied to the whole array | every probe biased together, undetectable by cross-check between probes |
| 28 | `<TANK>` densitometer failure (x11) | `gauging.rs::indicated_mass_kg` densitometer_failed | 0/1 switch | mass computed from a stale default density instead of the true (temperature-dependent) one |
| 28 | Trim tank left/right pump degradation (x2) | `cg_transfer.rs::TransferFaults.pump_degradation_fraction` | flow/pressure lost to wear | trim transfer (CG-aft/CG-fwd scheduling) runs slower; both together stops it |
| 28 | Trim tank inlet valve 1/2 sticks (x2) | `cg_transfer.rs::TransferFaults.valve_stuck_fraction` | seized fraction, frozen at seize position | forward trim transfer throttled/blocked |
| 28 | Trim line isolation valve fwd/aft sticks (x2) | same | same | isolates or fails to isolate the trim line from a gallery leg |
| 28 | Left/right outer tank transfer valve sticks (x2) | same | same | load-alleviation outer-tank-last sequencing cannot move fuel inboard on demand |
| 28 | Left/right inner tank transfer valve sticks (x2) | same | same | inner tank cannot feed the feed tanks on schedule |
| 28 | Left/right mid tank transfer valve sticks (x2) | same | same | mid tank cannot feed the feed tanks on schedule |
| 28 | Cross-feed valve 1-4 sticks (x4) | same | same | wing-balance cross-feed cannot move fuel between wings through that valve |
| 28 | ENG 1-4 FCOC fouling (x4) | `thermal.rs::fcoc_temperature_rise_k` fcoc_heat_w attenuation | fouling fraction | less oil heat rejected into fuel: feed tank runs colder (worse cold-soak/wax margin), oil runs hotter |
| 28 | ENG 1-4 fuel filter water contamination (x4) | `thermal.rs::filter_ice_blockage_fraction` free_water_fraction | free water volume fraction (GENERIC ceiling) | below 0 C freezes at the filter mesh, progressively blocking it |
| 28 | ENG 1-4 fuel filter anti-ice heater failure (x4) | same, anti_ice_heater_on forced false | 0/1 | removes the one mitigation; cold fuel free to block the filter |
| 28 | Left/right jettison nozzle valve sticks (x2) | `jettison.rs::JettisonValve` stuck_fraction | seized fraction | stuck shut denies that nozzle's capacity; stuck open risks uncommanded loss once upstream isolation opens |
| 28 | Left/right jettison nozzle blockage (x2) | `jettison.rs::effective_cda_m2` blockage_fraction | throat area lost | jettison rate through that nozzle falls proportionally |
| 28 | `<TANK>` structural fuel leak (x11) | `leak.rs::tank_wall_leak_kg_s` orifice area | leak orifice size fraction of a GENERIC max area | fuel lost overboard at a rate set by the tank's own head, unmetered -- the discrepancy `LeakDetector` catches |
| 28 | Forward transfer gallery leak | `cg_transfer.rs::TransferFaults.gallery_leak_fraction` and `leak.rs::gallery_leak_kg_s` (one fault, two consumers) | diverted flow fraction | forward transfers throttled; diverted fuel is an unmetered, detectable loss |
| 28 | Aft transfer gallery leak | same, aft-gallery paths | same | aft transfers and the jettison feed path throttled; unmetered loss |

Total: 89 registered failures (11+11+11+11 + 2+2+2+2+2+4 + 4+4+4 + 2+2 + 11+1+1),
matching `registry.rs`'s own test asserting id uniqueness and `Area::Fuel`/ATA
28 on every one.

## ECAM alerts raised (see `registry.rs` for full trigger/procedure detail)

- `FUEL_LEAK` (Caution) -- any tank-wall or gallery leak.
- `FUEL_TRIM_TRANSFER_FAULT` (Caution) -- trim pump/valve/isolation/gallery
  faults sustained past the shortfall detector's confirm time.
- `FUEL_CG_TRANSFER_DEGRADED` (Advisory) -- outer/inner/mid transfer valve
  faults.
- `FUEL_IMBALANCE_XFEED_FAULT` (Caution) -- cross-feed valve faults.
- `FUEL_FOB_LO_TEMP` (Caution, existing plugin Var `FUEL_FOB_LO_TEMP`) --
  FCOC fouling.
- `FUEL_FILTER_ICING` (Advisory) -- filter water contamination/heater
  failure.
- `FUEL_JETTISON_FAULT` (Caution) -- jettison valve/nozzle faults.
- `FUEL_QTY_DEGRADED` (Advisory) -- probe/compensator/densitometer faults.
- `FUEL_TANK_SLOSH_ADVISORY` (Advisory) -- baffle damage.
