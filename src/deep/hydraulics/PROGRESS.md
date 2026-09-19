# Progress — deep/hydraulics

- [done] Item 1: line/volume network solver — `network.rs` — `Node` (trapped
  fluid volume, pressure state, entrained-air content), `Line` (`Restriction::Pipe`
  laminar/Hagen-Poiseuille + turbulent/Darcy-Weisbach-Blasius blend, or
  `Restriction::Orifice` for valves), `Endpoint::{Node,Fixed}`, `CheckValve`,
  `PriorityValve` (A380 3000/3800 psi cited from FBW), `ReliefValve`,
  `FireShutoffValve`, `LeakMeasurementValve`, `Filter` (clog + bypass, reuses
  `physics::engine::oil.rs`'s clog law). `Network::step` solves every node's
  own backward-Euler capacitor equation implicitly by bisection against the
  current neighbour estimates (nonlinear Gauss-Seidel), unconditionally
  stable regardless of `dt` or fluid stiffness — verified in
  `a_constant_supply_against_a_fixed_return_settles_to_a_stable_pressure`
  (2000 steps, stiff small-volume/high-bulk-modulus case, never diverges).
  `fluid.rs` carries the fluid's own properties (Skydrol LD-4 density,
  viscosity, bulk modulus, cited to the vendor technical bulletin) including
  the entrained-air bulk-modulus correction (Merritt).
- [done] Item 1 (accumulator/reservoir): `accumulator.rs` (gas precharge,
  polytropic, sub-stepped explicit integration — the brief's allowed
  alternative to a fully implicit solve, safe here since state is clamped to
  `(0, shell_volume]` every sub-step) and `reservoir.rs` (bootstrap
  air-pressurised level, `fbw-common`'s exact `MIN_USABLE_VOLUME_GAL`/5 L
  warning/25-21.76 psi switch figures reused, decoupled from this same
  circuit's own hydraulic pressure to avoid a startup deadlock — see that
  file's module doc for why).
- [done] Item 2: A380 green/yellow topology — `topology.rs` — 4 EDPs/circuit
  (green: engines 1/2, yellow: engines 3/4, matching FlyByWire's own
  `A380Hydraulic` field names), one electric pump (yellow only, matching
  FlyByWire), manifold → priority valve → non-essential branch (gear,
  brakes, steering, cargo doors, reversers) with GENERIC line lengths/
  diameters derived from A380 overall dimensions, essential (flight
  controls) branch, accumulator and relief valve on the manifold, return
  manifold → filter (+ bypass) → reservoir closing the loop. `Circuit::step`
  is the per-tick entry point; `A380Hydraulics` owns both circuits
  (independent, no PTU, matching FlyByWire's own A380 model).
- [done] Item 3: fluid thermal model — `thermal.rs` — heat from pump
  mechanical/volumetric loss (`pump::PUMP_MECHANICAL_EFFICIENCY`, cited from
  `physics::hydraulics.rs`'s own documented figure) and from every
  throttling pressure drop network-wide (`throttling_heat_w`, summed over
  every `Network` line each tick in `Circuit::step`); cooled by a
  fuel-cooled heat exchanger (`HHX_EFFECTIVENESS = 0.6`, reused from
  `physics::fluids.rs`'s own cited A380 HHX figure) and a passive bay loss;
  viscosity feedback closes the loop (`Circuit::step` recomputes
  `fluid::dynamic_viscosity_pa_s` from the thermal state each tick before
  calling `Network::step`); `OVERHEAT_K` = Skydrol LD-4's own 107 C
  continuous limit (Eastman technical bulletin).
- [done] Item 4: pumps — `pump.rs` — `EngineDrivenPump`/`ElectricPump` share
  a pressure-compensated variable-displacement model (FlyByWire's own A380
  EDP/electric-pump displacement tables, `pumps.rs` lines 51-71, reproduced
  as plain breakpoint tables); case drain flow is the modelled health
  indicator (rises with `wear`, `pump::PumpFaults.wear`, a persisted 0..1
  health parameter registered as a `ComponentDef` `ParamDef`, not a
  discrete failure); cavitation at low reservoir inlet pressure reuses
  FlyByWire's own `AIR_PRESSURE_BREAKPTS_PSI`/`CAVITATION_MAP_RATIO` table.
- [done] Item 5: faults — every fault field listed in the brief's item 5 is
  implemented and wired through `topology::CircuitFaults`/`EdpFaults`: leak
  per line section (`Line.leak_area_m2`, per branch), check valve stuck
  open/leaking and stuck shut, priority valve stuck, relief valve cracking
  low, accumulator precharge loss, reservoir leak, fire SOV stuck, pump
  displacement loss, pump seizure, contamination (filter clog with bypass),
  air ingestion (`Node.air_fraction_at_1atm`, network-wide per circuit,
  softens the fluid via `fluid::effective_bulk_modulus_pa`). See
  `FAILURES.md` for the full list with ATA/magnitude/effect.
- [done] Registration — `registry.rs` — every component/failure/ECAM alert
  above registered through `crate::deep::api::Registry` (`Area::Hydraulics`,
  ATA 29), expanded per instance via loops (8 EDPs, 2 reservoirs, 2
  accumulators, etc., ~30 components, ~65 failures). 8 ECAM alerts (4 per
  circuit: reservoir level lo, reservoir air pressure lo, reservoir overheat,
  system low pressure), each `raised_by` the actual failure ids that cause
  it in this model (not empty placeholders). `registry.rs`'s own tests
  include `Registry::validate()` passing with zero errors.

## New simulator variables this model needs published (none exist yet)

- `HYD_{GREEN,YELLOW}_MANIFOLD_PRESSURE_PSI` — new; from
  `Circuit::step`'s `CircuitOutputs.manifold_pressure_pa` (convert to psi at
  the publish site).
- `HYD_{GREEN,YELLOW}_RESERVOIR_LEVEL_IS_LOW` — matches FlyByWire's own
  `Reservoir::new` identifier convention (`fbw-common/hydraulic/mod.rs` line
  2301-2302); from `CircuitOutputs.reservoir_low_level_warning`.
- `HYD_{GREEN,YELLOW}_RESERVOIR_AIR_PRESSURE_IS_LOW` — matches FlyByWire's
  own identifier (line 2303-2304); from
  `CircuitOutputs.reservoir_low_pressure_warning`.
- `HYD_{GREEN,YELLOW}_RESERVOIR_OVHT` — matches FlyByWire's own identifier
  (line 2305); from `CircuitOutputs.fluid_overheat`.

## Known simplifications (documented, not bugs)

- The network solver's pressure bracket (`network::PRESSURE_BRACKET_LO_PA`
  = -300,000 Pa) is a numerical search bound, not a physical vacuum limit; a
  branch cut off from all supply (e.g. by a seized-shut priority valve)
  under continued demand clamps there rather than modelling a graceful
  cavitating-dry transition. Exercised deliberately in
  `topology::tests::a_seized_priority_valve_can_starve_the_non_essential_branch`.
  A follow-on pass could give the network solver a dedicated "ran dry"
  state per node instead of relying on the bracket floor.
- Reservoir bootstrap air pressurisation is modelled as driven by cabin/
  bleed air supply (`CircuitInputs.pressurization_supply_fraction`), not by
  this same hydraulic circuit's own pressure — a deliberate choice to avoid
  a startup deadlock (the pump cavitation map is exactly zero efficiency at
  zero inlet pressure), see `reservoir.rs`'s module doc. Whoever wires this
  input should feed it from electrical/pneumatic system state, not from
  `CircuitOutputs.manifold_pressure_pa`.
- Consumer actuators (flight controls/gear/brakes/steering/cargo doors/
  reversers) are represented purely as commanded supply/return flow
  (`ConsumerDemands`); actuator dynamics themselves (position, force,
  differential-area asymmetry) belong to other areas (`flight_controls`,
  `GearStructure`) and are not modelled here.
- Pump `wear` is exposed as a fault input each tick but nothing in this
  directory yet integrates it forward over engine hours/contamination
  exposure — that accumulation (and its persistence across flights) is a
  natural next extension, likely belonging with whichever system owns
  component-health persistence generally.

## Next most valuable extensions (not yet started)

- Wear accumulation model: integrate `pump::PumpFaults.wear` upward over
  operating hours, faster under contamination (`filter_clog`) or cavitation.
- A dedicated brake hydraulic circuit model (metering valve, autobrake
  pressure modulation, accumulator-backed emergency braking) — currently
  brakes are just a demand-flow consumer node.
- Nose wheel steering ram/metering valve model with the same treatment.
- Reverser hydraulic actuator (deploy/stow time from `Line`/`Restriction`
  sizing) rather than a flat demand.
