# thermal_zones — progress log

Coupling agent: an airframe thermal network connecting other systems' heat/smoke outputs to a
shared, physically-grounded medium. Directory: `src/deep/thermal_zones/`.

- [done] Airframe thermal network engine (backlog item 1) — `network.rs` — generic
  `ThermalNetwork`: zones as lumped air+structure nodes, conduction links between neighbours,
  ventilation links (zone-to-zone or zone-to-outside), outside-air ram/recovery temperature and
  altitude (`OutsideAir`, `isa_static_temp_c`), solar load on the structure node. Self-contained
  per hard rule 2 (no `Vars`/X-Plane/crate-internal dependency — every formula reused from
  `physics::bays.rs`/`physics::fluids.rs` is cited and reproduced independently, not imported).
  Explicit-Euler sub-stepping (`stable_substep_count`) keeps stiff zone/link combinations
  numerically stable at large `dt`. 9 unit tests: hand-solved conduction-chain steady state,
  sun-load steady state, ventilation-fault comparison, insulation-fault comparison, dt=0/rest
  safety, stiff-substep stability, ISA/recovery-temperature sanity.
- [done] Heat-source interface (backlog item 2) — `network.rs`
  (`ThermalNetwork::inject_heat_w`/`inject_smoke_kg_s`, `Zone::air_temp_c`/
  `smoke_concentration_kg_per_kg` via `ThermalNetwork::air_temp_c`/`smoke_concentration`) — any
  system accumulates watts/smoke into a zone per tick; covered by the same tests as item 1 plus
  `topology_a380.rs`'s cargo-fire test.
- [done] Damage interface (backlog item 3) — `damage.rs` — `ThermalDamageRegistry`/
  `ThermalComponent`, Montsinger's-rule (10-degree doubling) damage-rate model
  (`montsinger_rate_per_s`), reading a component's zone temperature from `ThermalNetwork` each
  tick. 5 unit tests: no damage below limit, doubling verified at +10/+20/+30 C, hand-computed
  time to failure, clamping at 1.0, zero-dt/cold-component no-op.
- [done] A380 zone topology (backlog item 4) — `topology_a380.rs` — 26 zones (2 avionics bays,
  3 gear wells, 4 wing LE/TE compartments, 4 pylons, 4 nacelle cowls, the APU compartment, 3
  cargo holds, the crown area, 2 cabin decks, the tail cone, the belly fairing pack bays), ~24
  conduction links and ~18 ventilation links (avionics/cargo bays vented against the modelled
  cabin, gear bays/nacelles/pylons/APU/pack bay/tail cone vented against the outside), 5 example
  `ThermalComponent` registrations. All dimensions/loads GENERIC from Airbus's public A380
  dimensions (cited in the module doc) — no AMM zone drawing is public. 6 unit tests: zone count/
  sanity, no-NaN full-network run, cargo-fire cross-zone conduction propagation (headline
  coupling test), fan-blockage comparison, damage-registry integration, gear-door transient
  comparison.
- [done] Smoke transport (backlog item 5) — `smoke.rs` — CSTR-style mass-fraction advection
  (`concentration_kg_per_kg`, `advected_smoke_flux_kg_s`, `step_smoke_kg`), wired into
  `network.rs`'s ventilation-link solve so smoke moves with the same flows as heat; readable by
  any future detector via `ThermalNetwork::smoke_concentration`. 6 unit tests.
- [done] `FAILURES.md` — 34 failures across ATA 21/26/30/32/36/49/53 (human-readable index; the
  registered-in-code copy in `registry.rs` is canonical).
- [done] Registered every failure/component/ECAM alert in code (per the lead's "Registering
  failures, components and ECAM alerts" change, superseding the earlier CATALOGUE.md/ECAM.md
  file convention — neither file was created here, so there was nothing to delete) — `registry.rs`
  (`pub fn register(r: &mut Registry)`, declared in `mod.rs`), `Area::ThermalZones` (area code
  11): 34 `FailureDef`s, 30 `ComponentDef`s (7 ventilation + 8 fire-load + 6 duct/ice-blockage
  [2 wing anti-ice ducts + 4 nacelle duct/scoop assemblies, each nacelle one shared by its two
  ATA 30 failures] + 3 gear doors + 4 pylon bleed ducts + 1 APU duct + 1 crown insulation
  blanket), 13 `EcamAlert`s (3x `CARGO SMOKE <FWD/AFT/BULK>`, 4x `ENG <n> NAC OVHT`, `APU FIRE`,
  `APU COMPT OVHT`, 2x `<L/R> WING A ICE DUCT LEAK`, `ECS PACK BAY OVHT`, `AVIONICS VENT FAULT`).
  4 unit tests, including a cross-check that every zone name used in the registry's `effect`
  strings is a real zone `topology_a380::build()` actually creates.

## New Vars this model's alerts assume (not yet published — integration TODO)

`registry.rs`'s `EcamAlert::trigger`s read variable names this module's own code does not
publish yet (it is deliberately `Vars`/X-Plane-independent, hard rule 2). Whoever wires
`topology_a380::build()`'s `ThermalNetwork`/`ThermalDamageRegistry` into the running simulation
each tick needs to publish, per zone: `THERMAL_ZONE_<NAME>_TEMPERATURE_C` (air temperature) and
`THERMAL_ZONE_<NAME>_SMOKE_CONCENTRATION`, and per damage component:
`THERMAL_COMPONENT_<NAME>_DAMAGE` — the same naming convention `physics::bays.rs` already uses
for `BAY_<NAME>_TEMPERATURE_C`. The alerts' procedure-line `done_when` variables
(`CARGO_HEAT_SW:<FWD/AFT/BULK>`, `CARGO_VENT_SYS_SW`, `ENGINE_MASTER:<n>`, `APU_MASTER_SW`,
`APU_FIRE_PB_PUSHED`, `APU_FIRE_EXTINGUISHER_DISCHARGED`, `WING_ANTI_ICE_SW:<L/R>`,
`PACK_<1/2>_SW`, `AVIONICS_VENT_OVRD_SW`) are the real cockpit controls; several
(`ENGINE_MASTER:n`) likely already exist elsewhere in the plugin (not verified here — this
module never reads `crate::Vars` per hard rule 2) and should be reused rather than duplicated
once wired in.

## Next up (extending the backlog)

- Wire `topology_a380`'s network into the plugin's live `Vars` (publish the temperature/smoke/
  damage variable names above) — integration work, likely the lead's or a dedicated integration
  agent's, since it crosses into `crate::Vars`/simulation-loop territory this module deliberately
  stays out of.
- A dedicated cabin/ECS thermal balance (this module's `CabinMainDeck`/`CabinUpperDeck` are
  deliberately crude boundary nodes, module doc in `topology_a380.rs`) is out of this module's
  scope — that workstream already exists elsewhere per `physics::bays.rs`'s own citation of
  `docs/physics/air.md`.
- More `ThermalComponent` registrations per zone (only 5 representative ones are registered in
  `topology_a380::build()`; every zone could plausibly host one) if another agent's failure needs
  a specific one that does not exist yet.
