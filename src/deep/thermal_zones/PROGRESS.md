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

- [done] live system — `live.rs` (`live_system() -> Box<dyn deep::live::Area>`), `mod.rs` —
  owns one `topology_a380::build()` network + its `ThermalDamageRegistry`, stepped from `Truth`
  (outside air from `environment`, electric extract fans gated on any main AC bus >= 100 V).
  Consumes every failure this area registers (21/1-7, 26/1-8, 30/1-10, 32/1-3, 36/1-4, 49/1,
  53/1) onto the exact `model_field` each names. Publishes
  `THERMAL_ZONE_<NAME>_TEMPERATURE_C` / `_STRUCTURE_TEMPERATURE_C` / `_SMOKE_CONCENTRATION`
  for all 26 zones and `THERMAL_COMPONENT_<NAME>_DAMAGE` for all 5 registered components (83
  variables). 9 tests, including failure-to-published-variable tests for the cargo fire, the
  avionics fan, the crown blanket, a nacelle fire and a vent-scoop blockage.
  Still missing from `Truth`: solar irradiance (solar load passes 0), commanded gear-door
  position (the ATA 32 jam latch is implemented but has nothing to diverge from).

- [done] truth-wiring pass (`docs/deep/truth-requests.md`'s 2026-09-20 pass) — `apply_gear_door_
  failures` now takes `truth.controls.gear_door_commanded_open` (`[nose, left, right]`) and
  drives every healthy door's ventilation-link health to it every tick (a jam still blends
  toward its own stuck value exactly as before) instead of leaving every door pinned at
  `topology_a380`'s own resting closed state forever. Added `solar_flux_w_m2(&Truth)`, a
  **GENERIC** clear-sky flux (`1361 W/m^2 * 0.75 transmittance * sin(sun_elevation_deg)`, zero
  below the horizon) from the now-real `truth.sun_elevation_deg`, replacing the flat
  `SOLAR_FLUX_W_M2 = 0.0` handed to `ThermalNetwork::step`. 3 new tests: a commanded-open gear
  door ventilating its bay faster than one held closed, a high sun elevation warming
  CrownArea's structure (highest `sun_exposure_fraction` of any zone) more than no sun, and a
  unit check that a below-horizon sun never produces a nonzero/negative flux. Cloud attenuation
  is not modelled (no optical-depth input exists on `Truth`) -- this is a clear-sky figure only.


## 2026-09-20 — pylon bleed duct leak vs. `pneumatic_ducts` ODLS: which constant was wrong

**Reported symptom.** A full-severity pylon bleed duct leak (this area's ATA 36/1-4) could not
trip `pneumatic_ducts`' own overheat detection loop for the same pylon: 40 kW into a 0.5 kg/s
ram-vented bay settles ~75 K above ambient, short of that area's 100 K
`THRESHOLD_ABOVE_AMBIENT_K` confirm margin. Four candidates were put up: the leak magnitude
being too small, the ventilation being too generous, the confirm margin being wrong, or the
bay's thermal path being wrong.

**Finding: the leak magnitude was wrong — but in the opposite direction, and the value was the
smaller of its two errors.**

`PYLON_BLEED_LEAK_MAX_HEAT_W = 40_000.0` was, per `registry.rs`'s own note, meant to be "a
fraction of the ~200 C/44 psi bleed source's enthalpy flow through a small crack". Nobody had
done that arithmetic. Doing it:

- Duct bore 4 in (0.1016 m) -> bore area `pi/4 * 0.1016^2 = 8.107e-3 m^2` (FBW's own
  `a380_systems/pneumatic.rs` bleed pipework diameter, the same figure
  `pneumatic_ducts::network::ENGINE_DUCT_DIAMETER_M` cites).
- A full-severity *leak* (a cracked weld / partly-let-go V-band coupling, an order of magnitude
  below a severance) at 2% of bore = `1.621e-4 m^2`, a 14 mm equivalent hole. `pneumatic_ducts::
  leak::LEAK_AREA_FRACTION_OF_BORE` derives the same 2% independently; the agreement is
  deliberate so one physical fault is one size on both sides.
- At 200 C / 44 psig (473 K, 405 kPa abs) the crack is choked (`p_amb/p0 = 0.25 < 0.528`):
  `mdot = 0.65 * 1.621e-4 * 404694/sqrt(287.06*473.15) * 0.6847 = 0.0792 kg/s`.
- Its sensible enthalpy above a 15 C bay is `0.0792 * 1005 * 185 = 14.7 kW`.

So 40 kW overstated its own stated source by **2.7x**. That is the value error.

The form error is worse. A fixed wattage is not bounded by the duct it comes from, so it keeps
heating a bay that is already hotter than the air leaking into it. This is not hypothetical:
the sibling constant `WING_DUCT_LEAK_MAX_HEAT_W = 30_000.0` settles `WingLeLeft` at **853 C**
(measured, 4000 s run at rest) from a duct whose air is at 200 C. The wing leading-edge case
that was used as the *proof the chain works end to end* only trips because of that: an
unventilated compartment fed an unbounded wattage runs away past any threshold. See the
recommendation below.

**Fix (this area).** `PYLON_BLEED_LEAK_MAX_HEAT_W` is gone. `live::pylon_bleed_leak_heat_w`
computes the leak the way the physics does: choked-orifice mass flow through the crack at the
engine's *real* bleed port condition — `Truth::engine_bleed_pressure_pa`/`_temp_k`, which
`deep::live` documents as "bleed air available **at the pylon**", i.e. exactly the duct run the
`PylonEngine<n>` zone contains — times its sensible enthalpy above the bay's current air
temperature. Consequences, all measured:

| duct condition | mdot | heat into a 15 C bay |
| --- | --- | --- |
| engine shut down (ambient) | 0 | **0 W** (was 40 kW) |
| 200 C / 44 psig (old stated basis) | 0.0792 kg/s | 14.73 kW |
| Trent 972 IP8 at take-off, 9.7 bar / 590 K | 0.170 kg/s | 51.60 kW |

(The take-off IP8 condition is derived in the function's doc comment: fan hub PR ~1.75 through
an IPC PR ~5.5 is ~9.6x ambient, and the same ratio through a ~0.90 polytropic efficiency gives
`288 * 9.6^(0.2857/0.90) = 590 K`.) The heat is now larger than 40 kW at take-off power and
zero on a cold aircraft, which is the behaviour a fixed number could not have either way.

**It still does not trip, and that is the honest answer.** At take-off port conditions
`PylonEngine1` settles at 88.5 C against an unaffected sibling at 15.6 C — a real, substantial
**73 K** rise — and `DEEP_PNEU_ODLS_PylonEngine1_TRIP` stays 0. The bay's steady rise is set
almost entirely by its ventilation (0.5 kg/s = 502 W/K, against ~25 W/K through structure):

    mdot_leak*cp*(T_duct - T_bay) = mdot_vent*cp*(T_bay - T_out)

To reach 100 K above ambient from a 590 K duct you need
`mdot_leak = mdot_vent * 100/202 = 0.248 kg/s` — **half the bay's entire ventilation flow**,
from a 17 mm crack instead of a 14 mm one. Choosing 17 mm would be picking the number that
makes the detector fire. It was not taken.

**The other two candidates are clear.** The 0.5 kg/s pylon vent is not too generous: it is one
air change every 12 s in a 5 m^3 bay, and a designated fire zone (CS/FAR 25.1187) is ventilated
at least that hard — if anything the real figure is larger, making the bay cooler still. The
thermal path cannot move it either: heat capacity sets only the time constant, and ventilation
is 95% of the steady-state conductance, so the structure terms cannot change the answer.

### Recommendation for `pneumatic_ducts` (not edited — that area's constants)

`odls::OverheatDetectionLoop::THRESHOLD_ABOVE_AMBIENT_K = 100.0` is wrong in two ways.

1. **Wrong form: the threshold should be absolute, not a margin above ambient.** A real
   bleed-leak/overheat loop (continuous eutectic-salt sensing element) alarms at a fixed
   temperature chosen for *that compartment's* own structural and wiring limit. `odls.rs`'s own
   doc comment states exactly that rationale ("2000-series aluminium begins losing temper above
   roughly 150 C, so a detector is conventionally set with margin below that") and then
   implements a relative rule, which only agrees with it at ISA sea level. The same rule trips
   at 115 C at 15 C ambient, at 145 C on a 45 C ramp (above the stated margin to the structural
   limit), and at +45 C at cruise with SAT -55 C — the last being *below* what a pylon bay sits
   at in normal operation from engine proximity alone, i.e. a nuisance trip. Suggested
   replacement: a per-zone absolute set point, with the two compartment classes this network
   already distinguishes kept apart — roughly **124 C for the wing/fuselage leading-edge duct
   runs** and **~200 C for the pylon/strut runs**, the strut figure being higher precisely
   because that bay is hot in normal operation. (Both are representative continuous-loop alarm
   temperatures for those two compartment classes; no A380-specific figure is public, so they
   would be GENERIC, but the *form* is right where the current one is not.)

2. **Wrong quantity: the loop should not be reading bulk bay air.** `live::zone_air_k` reads
   `THERMAL_ZONE_<NAME>_TEMPERATURE_C`, which is this area's well-mixed **air node** for the
   whole compartment. A real detection loop is routed *along the duct inside its shroud*, so it
   senses the escaping plume. This matters because bulk bay air is bounded above by the duct
   gas: a 200 C (or even a 320 C) duct leaking into a ram-ventilated 5 m^3 pylon cannot put the
   bulk air more than ~73 K above ambient for any crack size that is still a leak. There is no
   absolute threshold that is simultaneously *below* the duct temperature and *above* normal
   bay temperature for bulk pylon air — so with fix (1) alone the pylon still would not trip.
   Closing the chain needs a local duct-run temperature, not the zone air node. This area can
   publish one (`THERMAL_ZONE_<NAME>_BLEED_DUCT_HOTSPOT_C`) if `pneumatic_ducts` wants to read
   it, but it was **not** added in this pass: the plume temperature at the sensing element
   depends on the entrainment ratio at the element's standoff from the crack, and no public
   figure exists for A380 pylon duct/loop routing. At 2 duct-diameters standoff it is 229 C and
   trips a 200 C set point; at 5 it is 179 C and does not. That standoff is the number that
   would decide the answer, and inventing it is exactly the thing this push forbids. It needs a
   sourced routing figure, or an explicit GENERIC decision taken jointly, before either area
   builds on it.

### Follow-up owed by this area (not done in this pass)

`WING_DUCT_LEAK_MAX_HEAT_W` (30 kW), `NACELLE_DUCT_LEAK_MAX_HEAT_W` (20 kW) and
`APU_DUCT_LEAK_MAX_HEAT_W` (25 kW) still have the unbounded fixed-wattage form, and the wing one
demonstrably produces an impossible 853 C bay. They should get the same treatment as the pylon
constant. They were left alone deliberately in this pass because:

- `pneumatic_ducts::live::tests::a_thermal_areas_own_wing_duct_leak_heats_the_bay_enough_to_trip_
  this_areas_odls` asserts `THERMAL_ZONE_WINGLELEFT_TEMPERATURE_C > 150.0` as its *setup*, and
  that setup asserts a thermodynamic impossibility: it needs the bay hotter than 150 C from a
  200 C duct, which requires `mdot_leak*cp` to beat the bay's ~31 W/K loss path by 2.7:1, i.e.
  `mdot_leak >= 0.083 kg/s` — a 15 mm hole in a 50 mm WAI duct, 8.7% of its bore, a rupture not
  a leak. Honest physics gives ~0.012 kg/s and a ~68 C bay. Fixing the wing constant therefore
  breaks that test, and the test is in the other area, which this agent must not edit.
- `Truth` has no APU bleed *temperature* (only `apu_bleed_pressure_pa`), so the tail-cone APU
  duct leak cannot be derived the same way without inventing one. That is a `truth-requests.md`
  item: **`apu_bleed_temp_k`, the PW980 load-compressor discharge temperature**, which
  `pneumatic_ducts::live` already derives internally from the APU's published pressure ratio and
  which this area would use directly.

- [done] pylon bleed duct leak derived from real duct conditions — `live.rs`
  (`pylon_bleed_leak_heat_w`, `orifice_mass_flow_kg_s`, `PYLON_BLEED_DUCT_BORE_M`,
  `PYLON_BLEED_LEAK_AREA_FRACTION_OF_BORE`, `BLEED_LEAK_DISCHARGE_COEFFICIENT`), `registry.rs`
  (`model_field`/`magnitude` now describe the crack and the port condition, not a reference
  wattage) — replaces `PYLON_BLEED_LEAK_MAX_HEAT_W = 40_000.0`. 4 new tests: the derivation's
  two point values (51.60 kW at take-off IP8, 14.73 kW at the old constant's own claimed
  200 C/44 psig basis) plus linearity in crack area; a shut-down engine's leak warming its pylon
  by nothing at all; the bay never exceeding the duct feeding it (the conservation property the
  fixed-wattage form lacks); and an end-to-end run with `pneumatic_ducts` in the frame showing
  the leak heats its own bay ~73 K and only its own bay, staying under the 100 K ODLS margin —
  asserted on the physics rather than that area's trip flag, so it stays true when the
  recommendation above is acted on. Full `deep::thermal_zones` suite green (49 tests).

## 2026-09-21 — the wing/nacelle duct leak follow-up owed above, done

`WING_DUCT_LEAK_MAX_HEAT_W`/`NACELLE_DUCT_LEAK_MAX_HEAT_W` are gone, replaced by
`live::anti_ice_duct_leak_heat_w` (`live.rs`): the same choked-orifice-crack derivation as
`pylon_bleed_leak_heat_w`, factored out into a shared `duct_leak_heat_w`, sized to a 50 mm bore
(`ANTI_ICE_DUCT_BORE_M`, `pneumatic_ducts::network::WAI_DUCT_DIAMETER_M`'s own cited figure,
reused for the nacelle case too since no separate public nacelle anti-ice duct diameter exists).
Fed from `Truth::engine_bleed_pressure_pa`/`_temp_k`: for the wing case, whichever of that
wing's own two engines (1/2 left, 3/4 right, `topology_a380`'s own numbering) is at the higher
bleed pressure this tick; for the nacelle case, that engine directly. `APU_DUCT_LEAK_MAX_HEAT_W`
is untouched — `Truth` still carries no APU bleed *temperature*, the same gap the pylon pass
already documented above.

Consequences, measured: a full-severity wing/nacelle leak on a cold, shut-down aircraft now
warms its bay by nothing (previously 30/20 kW regardless); at take-off IP8 port conditions
WingLeLeft settles well below the 590 K duct feeding it rather than at an impossible 853 C.
Two new tests (`live.rs`): the cold/hot pair for the wing case (mirrors the pylon tests'
pattern), and `a_blocked_nacelle_vent_scoop_makes_the_same_duct_leak_hotter` updated to use
`takeoff_truth()` instead of `powered_ground_truth()` (AC power alone no longer exercises a duct
leak that needs a running engine's own bleed air — the old test's continuing to pass was itself
evidence the old constant hid the missing state dependency). Full `deep::thermal_zones` suite
green (50 tests).

**Known cross-area consequence, not fixed here (outside this area's owned files):**
`pneumatic_ducts::live::tests::a_thermal_areas_own_wing_duct_leak_heats_the_bay_enough_to_trip_this_areas_odls`
now fails its own setup assertion (`THERMAL_ZONE_WINGLELEFT_TEMPERATURE_C > 150.0`): with the
honest physics and that test's own `Truth::default()` fixture (engine bleed at ambient, i.e. a
cold aircraft), the bay reaches only ~21.8 C, not >150 C — which is the *correct* answer for a
cold aircraft, and exactly what that test's own comment already predicted ("even the honest
figure would clear this area's 124 C threshold only if `thermal_zones` also fixes its own
leak-model form" — it has, and a cold-aircraft fixture cannot clear it; the fixture needs a
running engine, e.g. this area's own `takeoff_truth()`, to exercise the now-real bleed
dependency). That test lives in `deep::pneumatic_ducts`, outside this pass's owned files, so it
was read and confirmed but not edited.
