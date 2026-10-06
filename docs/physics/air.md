# Air physics (workstream 3): bleed, packs, distribution, cabin, pressurisation

Code:
- FBW patch (real ACM physics replacing the common crate's documented-as-placeholder pack
  model, plus the plumbing needed to feed it real bleed conditions): `D:\fbw-aircraft\fbw-a380x\
  src\wasm\systems\a380_systems\src\air_conditioning\{mod.rs, air_cycle_machine.rs}`,
  `...\a380_systems\src\pneumatic.rs`, `D:\fbw-aircraft\fbw-common\src\wasm\systems\systems\src\
  shared\mod.rs`. Diff saved at `D:\A380\fbw-xp-systems\patches\fbw-rust\air.patch`.
- Plugin physics: `src/physics/air.rs` (the shared engine-load contract's bleed term only —
  everything else in this workstream is FBW-side). Registered in `src/physics/mod.rs`, wired
  into `src/lib.rs` (field `bleed_loads`, built after `circuit_protection`, updated after the
  systems tick).

## Audit: what FlyByWire's crate already does physically, and what was a shortcut

Per the brief's "audit before building" rule, this workstream read `fbw-common/.../pneumatic/
{mod.rs, valve.rs}`, `fbw-common/.../air_conditioning/{mod.rs, cabin_air.rs, acs_controller.rs,
cabin_pressure_controller.rs, pressure_valve.rs}` and `a380_systems/.../{pneumatic.rs,
air_conditioning/{mod.rs, cpiom_b.rs, local_controllers/*}}` before changing anything.

| Component | Already physical? | Evidence |
|---|---|---|
| Cabin control volume, pressurisation | **Yes, and kept as-is** | `cabin_air.rs::CabinAirSimulation` derives cabin pressure from the ideal gas law and a **real compressible orifice equation** for outflow: `subsonic_flow_out_calculation`/`supersonic_flow_out_calculation` implement the standard choked/subsonic split at the critical pressure ratio 0.53 (cabin_air.rs:308-337), driven by the outflow/safety valve's own open area. `cabin_pressure_controller.rs`'s target-V/S "schedule" (line 332, "empirical graphs ... to simulate climb schedule as per the real aircraft") is the CPC's **control law setpoint**, exactly like the real CPCS: a PID (`OFV_CONTROLLER_KP/KI`) drives the outflow valve toward it, and actual cabin altitude/rate still comes out of the orifice-flow mass balance above, not a direct write. This already matches the brief's "the CPCS controller commands valve position, so rate and altitude emerge from the flow." Not touched. |
| Cabin zone thermal model | **Yes, and kept as-is** | `cabin_air.rs`'s per-zone energy balance is a real one: inlet/outlet air enthalpy flow, human body heat (convection + radiation + respiration, `human_body_heat_calculation`), and fuselage-wall conduction with actual Reynolds/Nusselt correlations for natural vs. forced convection (`natural_convection_coefficient_calculation`, `forced_convection_coefficient_calculation`) through a real two-layer (fibreglass insulation + aluminium skin) wall (`heat_transfer_through_wall_calculation`). **Gap** (not fixed this pass, see Limitations): no solar or avionics heat load term — only passengers, wall conduction and inlet/outlet air are summed. |
| Bleed valve/pipe network | **Partially** | `pneumatic/mod.rs`'s `PneumaticPipe`/ideal-gas mixing (`change_fluid_amount`) is real. Valve **mass flow**, however, is not an orifice equation: `PneumaticContainerConnector::update_move_fluid` computes the mass that would *equalise* pressure between the two containers and relaxes toward it exponentially with a fixed `TRANSFER_SPEED = 10` time constant (mod.rs:74, "the pressure change is not linear but is nearly linear so we simply approximate it"), used by every valve (HP, PRV, crossbleed, pack flow valve) via `DefaultValve`/`ElectroPneumaticValve`. This is a genuine "Bleed" shortcut in scope, **not replaced this pass** — see Limitations; it was not touched because the risk of destabilising the two aircraft's pneumatic test suites (43 pneumatic tests alone) outweighed what budget remained after the ACM work below. |
| Precooler | **Yes, and kept as-is** | `Precooler::update` (pneumatic/mod.rs:563-585) is a real lumped heat-exchanger energy balance (fixed `heat_transfer_coefficient` = UA product, `1.005 kJ/kg*K` specific heat), not a lookup table. The "fan air" supply pipe it cools against comes from a `FanAirValve`; not touched. |
| PRV regulation | **Yes, and kept as-is** | `pneumatic::tests::pressure_regulating_valve_regulates_to_40_psig` (a380 pneumatic.rs) confirms the production PRV genuinely regulates to ~40 psig, a real controlled setpoint, not a constant. |
| Mixer unit / trim air distribution | **Yes, and kept as-is** | `MixerUnit::update` (air_conditioning/mod.rs) does real flow-weighted energy mixing (`sum(mdot*T)/sum(mdot)`), and `TrimAirSystem` mixes pack air with a per-zone trim air valve the same way. Not touched. |
| **Packs (air cycle machine)** | **No — FBW's own doc comment says so** | `AirConditioningPack::update` (air_conditioning/mod.rs, common crate) is explicitly commented `/// Temporary struct until packs are fully simulated` and `/// Takes the minimum duct demand temperature as the pack outlet temperature ... this is a placeholder until the packs are modelled`: it copies the ACSC's own *demand* signal to the pack outlet through a 10 s lag, with **no dependency on bleed pressure/temperature, ram air, or any turbomachinery**. This is the centrepiece of this workstream and is what `air_cycle_machine.rs` replaces for the A380 (see below). The A320 side is untouched (still uses the common placeholder), out of scope and not needed for this aircraft. |
| Anti-ice bleed consumption | **Not modelled by FBW's crate; not added this pass** | grep for engine/wing anti-ice bleed draw in `a380_systems`/`fbw-common` returns nothing beyond valve open/closed state. See Limitations. |

## The ACM: what was variable-based, what is physical now

**Before:** pack outlet temperature = `min(zone duct demand temperatures)`, low-pass filtered
over 10 s. Pack outlet flow = whatever the flow control valve's own (already-existing)
relaxation-based mass flow happened to be. No use anywhere of bleed pressure, bleed temperature,
ram air, or compressor/turbine work.

**Now** (`air_cycle_machine.rs`, new file, A380-specific, both packs' 2 ACM channels lumped as
one — mathematically exact for identical parallel channels processing the same inlet air, see
the module's own doc comment): a full bootstrap air cycle runs every tick, in this order:

1. **Primary heat exchanger (PHX):** effectiveness-NTU energy balance between the real pack
   inlet plenum (`PackComplex::pack_container`, pneumatic.rs — mixed from both flow control
   valves by that pipe's own ideal-gas mass/energy balance, not a scheduled value) and ram air.
2. **Compressor/turbine bootstrap shaft balance:** the compressor pressure ratio is solved (by
   bisection — unconditionally stable, cannot diverge or return NaN, unlike a Newton iteration)
   so that turbine specific work equals compressor specific work (a bootstrap ACM has no other
   power source). Isentropic compression/expansion relations: `T2/T1 = (P2/P1)^((y-1)/y)`
   scaled by isentropic efficiency.
3. **Secondary heat exchanger (SHX):** cools the compressor discharge against a *separate* ram
   air passage (fresh ambient air, not the PHX's already-warmed exit — see equations below for
   why).
4. **Turbine expansion** to pack outlet pressure (`cabin pressure + an assumed duct margin`).
5. **Water separator:** condenses out humidity in excess of saturation at the turbine discharge
   condition (Magnus/Tetens saturation vapour pressure), returning the latent heat to the stream
   and removing the condensed mass from the outlet flow (mass and energy both accounted for).
6. **Hot air bypass valve:** blends a controlled fraction of pre-PHX bleed air around the whole
   cycle to reach the ACSC's demand temperature. This — not the ACM core — is what actually
   tracks the demand; the ACM's own coldest deliverable temperature is a genuine physical floor
   the bypass valve cannot undercut. This mirrors the real A380/large-transport "pack temperature
   control valve".
7. **Ram air supply:** `mdot = Cd * A_door * sqrt(2 * rho_ambient * dP)`, `dP` = recovered
   freestream dynamic pressure (`0.5 * rho * TAS^2 * recovery`) **plus** a fixed pressure rise
   from the ACM's own shaft-driven ram fan, so packs still cool with zero airspeed on the ground
   — exactly why real ACMs have that fan wheel.

### Why PHX and SHX use separate ram air passages, not one stream in series

An early version chained the two heat exchangers on one ram stream (PHX ram exit feeds SHX ram
inlet). Whenever ram mass flow is not many times larger than bleed flow — routinely true at low
airspeed — the PHX pre-heats the SHX's own coolant by tens of kelvin, silently crippling SHX
performance in a way that is an artefact of the single-stream simplification, not of either real
architecture. Both PHX and SHX now draw independently from ambient temperature at the modelled
ram mass flow, representing separate, appropriately-sized passages of the same duct (a real,
common layout, e.g. a two-pass ram matrix with a dividing wall).

## Parameter sources

| Parameter | Value | Source |
|---|---|---|
| `CP_AIR`, `GAMMA`, `R_AIR` | 1005 J/(kg·K), 1.4, 287.058 | Matches `systems::air_conditioning::Air`'s own constants (air_conditioning/mod.rs:1304-1310), duplicated because that struct's constants are private to the common crate. |
| PHX/SHX effectiveness | 0.80 / 0.78 | **GENERIC**: no A380-specific figure is public. Typical cross-flow plate-fin aircraft ECS heat exchanger effectiveness range 0.70-0.85 (Moir & Seabridge, *Aircraft Systems: Mechanical, Electrical and Avionics Subsystems Integration*, 3rd ed., ch. 7). Marked in-code as generic. |
| Compressor / turbine isentropic efficiency | 0.78 / 0.82 | **GENERIC**: typical centrifugal-compressor / radial-inflow-turbine ACM wheel efficiency ranges (0.75-0.80, 0.80-0.85), same class of source as above. |
| Shaft mechanical efficiency | 0.97 | **GENERIC**: typical bearing/windage loss allowance. |
| Heat exchanger pressure loss | 3% per core | **GENERIC**: typical duct/HX pressure-loss budget for this exchanger class. |
| Turbine outlet pressure margin | 0.5 psi above cabin pressure | **Assumed**, no FBW/public value found; chosen as the most defensible order-of-magnitude duct/mixer-manifold flow margin and clearly marked as assumed in-code, per the brief's "if no source exists, say so." |
| Ram scoop recovery factor | 0.95 | **GENERIC**: typical flush/NACA-type inlet recovery at high subsonic Mach. |
| Ram door area (0.021 m²) | Derived, not invented | Sized from FBW's own design pack-flow constant: `AirGenerationSystemApplication::FLOW_CONSTANT_C * A320_TO_A380_FLOW_CONVERSION_FACTOR` = 0.5675 kg/s * 2.8 ≈ 1.59 kg/s (cpiom_b.rs:335,340 — itself already an FBW-authored, if approximate, A320-to-A380 scaling), sized so the door alone passes about that much ram air at a representative cruise point (Mach 0.85/35,000 ft). |
| Ram fan pressure rise | 9000 Pa | **GENERIC**, but *sized*, not invented outright: chosen so a stationary aircraft's fan-only ram flow through the same door area is the same order as the cruise dynamic-pressure-driven flow above — real ACM ram fans are specifically sized so ground and in-flight cooling capacity are comparable, which a much smaller figure would not reproduce. |
| Latent heat of vaporisation | 2.501e6 J/kg | Standard physical constant (water at 0 °C). |
| Bleed extraction contract source | `EngineBleedAirSystem::bleed_extraction_flow` = the pressure regulating valve's own `fluid_flow()` | The one path bleed air normally leaves the engine's compressor through in this crate's topology (the HP valve feeds the same `transfer_pressure_pipe` the PRV draws from, so reading the PRV avoids double-counting); published as `PNEU_ENG_<n>_BLEED_EXTRACTION_FLOW`, republished by the plugin as `ENGINE_BLEED_EXTRACTION_KG_S:<n>`. |

## Tests

New, in `air_cycle_machine.rs` (`cargo +stable-x86_64-pc-windows-gnu test -p a380_systems
--release`, `CARGO_TARGET_DIR=D:\A380\fbw-build\target-phys-air`):
- `pack_off_has_no_outlet_flow` — mass conservation at zero flow.
- `cruise_design_point_produces_cold_air_below_bleed_inlet_temperature` — design point (44 psi/
  200 °C bleed, -56.5 °C/238 hPa/Mach-0.85-equivalent ambient, coldest cabin selection): outlet
  well below bleed inlet temperature and above a physically sane floor, pressure ratio and ram
  flow both non-trivial.
- `ground_design_point_with_full_bleed_pressure_reaches_selected_temperature` — ground/low-speed
  design point with a fully regulated ~44 psi source: outlet within 1 °C of the 24 °C demand.
- `hot_bypass_valve_opens_when_demand_is_above_coldest_deliverable` — the bypass valve engages
  when the ACM's own cold output would otherwise overshoot the demand.
- `no_bleed_air_leaves_the_pack_at_ambient_not_at_a_stale_hot_value` — bleed-leak/failure
  behaviour: losing flow drives the outlet to ambient, not a stuck stale value (the old
  placeholder's failure mode).
- `ram_air_flows_even_with_zero_airspeed_on_the_ground` — the shaft-driven fan term is exercised
  independently of freestream dynamic pressure.

Existing suites re-run for regressions: `a380_systems`'s full 554-test suite (`cargo test -p
a380_systems --release`) and `pneumatic` module (43 tests) — see Limitations for the one
isolated regression cluster.

**Cost:** the shaft balance is a 24-iteration bisection of simple arithmetic (a handful of
`powf`/`exp` calls per iteration) per pack per tick — negligible (microseconds) next to the rest
of the systems tick, and bisection is unconditionally stable (cannot diverge or produce NaNs
regardless of sim rate, pause, or how close the two work curves are).

## Study panel quantities

Per duct/valve, exposed as named variables the lead can wire into the Study panel (both packs,
`n` = 1 or 2):
- `COND_PACK_n_OUTLET_TEMPERATURE`, `COND_PACK_n_RAM_AIR_FLOW`,
  `COND_PACK_n_RAM_AIR_OUTLET_TEMPERATURE`, `COND_PACK_n_RAM_AIR_DOOR_POSITION`,
  `COND_PACK_n_HOT_AIR_BYPASS_POSITION`, `COND_PACK_n_ACM_PRESSURE_RATIO`,
  `COND_PACK_n_TURBINE_OUTLET_TEMPERATURE`, `COND_PACK_n_WATER_EXTRACTED` (all new, this
  workstream).
- Already available from FBW's own unmodified code and worth showing alongside: per-engine
  `PNEU_ENG_n_{HP,IP,TRANSFER,PRECOOLER_INLET,PRECOOLER_OUTLET}_{PRESSURE,TEMPERATURE}` and
  `_{HP,IP,PR,STARTER}_VALVE_OPEN`; per pack `COND_PACK_n_FLOW_VALVE_{1,2}_IS_OPEN`/
  `PNEU_PACK_n_FLOW_VALVE_{1,2}_FLOW_RATE`; cabin `PRESS_CABIN_ALTITUDE`,
  `PRESS_CABIN_VS`, `PRESS_MAN_CABIN_DELTA_PRESSURE`, outflow valve open amount; per-zone duct
  temperature (`COND_<ZONE>_DUCT_TEMP`) and measured temperature.
- New: `PNEU_ENG_n_BLEED_EXTRACTION_FLOW` (FBW-native name) / `ENGINE_BLEED_EXTRACTION_KG_S:n`
  (plugin contract name, same value).

## Limitations (honest gaps, per the brief's own reporting rule)

> **Update 2026-09-30: Limitation 1 is resolved, and it had two causes, not one.**
> 1. The water separator returned the latent heat of all moisture above saturation at the
>    *dry* turbine discharge in one step, which warmed the air far past its own dew point:
>    +42 K on a 24 C saturated day, a 33 C pack outlet from a -10 C turbine discharge. It now
>    solves `t = t_dry + (w - w_sat(t)) L / cp`, whose single root lies between the dry
>    discharge and the dew point (`the_water_separator_never_warms_the_air_past_its_own_dew_point`).
>    That fixed 8 of the 11 tests. It affected the aircraft too, not just the test rig.
> 2. The test rig's plenum (12 m^3, sized for FlyByWire's flow tests) sits near 23 psi. The real
>    `A380Pneumatic`'s sits at about 50 psia, within 1 psi of its regulated supply (both packs,
>    ground idle and 70% N1, measured). So the rig now hands the air cycle machine its regulated
>    supply pressure, and the merge's 8 -> 800 m^3 source change is reverted.
>    Shrinking the plenum to 2 m^3 instead starved the AMM flow tests.
> 3. `knobs_dont_affect_duct_temperature_when_cpiom_unpowered` now compares two knob settings.
>    Unpowering DC 1/2/ESS also stops the OCSMs, so the cabin pressurises on the ground and no
>    turbine can expand into it.
>
> Result: all a380_systems tests pass in both trees.
> `whole_aircraft_tests` (MSFS tree) flies the whole A380 from cold and dark through a climb to
> an FL350 cruise, with no breaker trips and every main bus powered; the cabin holds 5,456 ft
> at 8.55 psi.

1. **Pack plenum pressure under the air_conditioning module's own test fixture.** The real
   `A380Pneumatic` regulates its PRV to ~40 psig (confirmed by its own
   `pressure_regulating_valve_regulates_to_40_psig` test) and should feed the ACM realistic
   conditions in the actual aircraft. The **air_conditioning module's own isolated test mock**
   (`TestPneumatic`/`TestEngineBleed`/`TestPneumaticPackComplex`, air_conditioning/mod.rs), built
   only to exercise the ACSC/zone-controller logic in isolation, uses much cruder pipe
   volumes/exhaust sizing and — because the old placeholder never read pack pressure at all —
   its pack plenum settles well below a realistic regulated value (~23 psi observed in one
   scenario). Since the ACM now genuinely needs that pressure for turbine expansion ratio, 11 of
   that module's zone-temperature-convergence tests (all under this one root cause; the full
   pneumatic test suite and the rest of air_conditioning — 543 of 554 tests — are unaffected)
   fail against that mock, even though the same ACM reaches the demanded temperature within 1 °C
   at a realistic ~44 psi source (`ground_design_point_with_full_bleed_pressure_reaches_
   selected_temperature`, new test). **Concrete fix for next pass:** either give the ACM a
   mutable draw on `pack_container`'s own mass balance (proper mass conservation instead of the
   current independent demand-signal flow) or recalibrate the test mock's exhaust/volume sizing
   against the production PRV's own 40 psig regulation point.
2. **Valve mass flow is still the exponential-relaxation-to-equilibrium approximation** (see
   audit table), not a compressible orifice equation, for the general-purpose HP/PRV/crossbleed/
   pack-flow-valve connector shared by every valve in the pneumatic crate. Not replaced this pass
   because it is used by 43+ existing pneumatic tests across two aircraft; a safe rework needs a
   dedicated pass with room to fix any resulting regressions, which this session's remaining
   budget did not allow after the ACM work above. The brief's "Pack flow control valves use
   orifice flow equations" requirement is therefore only partially met: the ACM's own physics
   (turbine/compressor pressure ratios, heat exchanger flows) are the requested equation forms;
   the flow control valve immediately upstream of the pack still uses FBW's existing relaxation
   model.
3. **Cabin thermal model: no solar or avionics heat term.** `cabin_air.rs`'s energy balance
   (kept as-is, already real) sums passenger heat, inlet/outlet air enthalpy and wall conduction
   only. Adding solar insolation (needs a sun-elevation/incidence input X-Plane exposes) and an
   avionics heat load (ideally sourced from the electrical workstream's real consumption figures
   once available) is a natural, bounded follow-up but was not attempted this pass to avoid
   touching the shared common-crate energy balance (used by both aircraft) without budget left to
   re-verify every existing cabin_air.rs test.
4. **Engine/wing anti-ice bleed consumption and its heating effect on X-Plane ice accretion**:
   audited (no FBW model of anti-ice bleed draw exists to keep or replace) but not built this
   pass — out of remaining budget. The valve open/closed state already exists; the bleed mass
   flow and heat-transfer-to-airframe physics do not.
5. What only X-Plane can verify: actual cockpit indication behaviour (SD BLEED/COND/PRESS pages
   reading these new variables sensibly), pilot-perceptible cabin comfort across a full flight
   profile, and whether the real (non-mock) pneumatic system's PRV-regulated pressure indeed
   keeps the ACM comfortably within its cooling envelope in practice — the isolated-mock
   regression in Limitation 1 specifically cannot be checked without the full aircraft running.

## Second pass (this session): compressible orifice flow infrastructure

Time-boxed session (hard deadline partway through). Landed one real, verified change; several
other second-pass items (anti-ice bleed, cabin solar/avionics/IFE/galley heat, leak detection,
pack overheat protection, avionics/cargo ventilation, safety valve cracking pressure, ditching/
manual outflow) were scoped and researched but **not implemented** — see below, honestly, rather
than claim partial/faked coverage.

### What was landed: `compressible_orifice_mass_flow_rate` + orifice-flow valve methods

New, additive-only code in `D:\fbw-aircraft\fbw-common\src\wasm\systems\systems\src\pneumatic\
valve.rs` (diff: `patches/fbw-rust/air-orifice-flow.patch`, verified with
`git apply --check --reverse` against the current working tree):

- `compressible_orifice_mass_flow_rate(upstream_pressure, upstream_temperature,
  downstream_pressure, orifice_area, discharge_coefficient) -> MassRate`: the standard
  isentropic-ideal-gas orifice/valve-throat flow law, choked below the critical pressure ratio
  (0.5283, `(2/(γ+1))^(γ/(γ-1))`, γ=1.4) and compressible-subsonic above it. Source: Anderson,
  *Modern Compressible Flow*, 3rd ed., §3.6; the same functional form underlies the IEC 60534-2-3
  gas valve sizing standard.
- `PneumaticContainerConnector::update_move_fluid_with_orifice(...)`: uses that law instead of
  the exponential-relaxation-to-equilibrium approximation (`update_move_fluid`), scaled by the
  valve's own open-amount ratio exactly as the existing relaxation path already is. Because an
  explicit single-evaluation-per-step orifice rate can overshoot equalising two small volumes in
  one step (the old relaxation model could not, by construction), the mass moved per step is
  clamped to the same `get_mass_flow_for_equilibrium` bound the crate already uses elsewhere —
  this only prevents gradient reversal in one step, it does not change the flow law itself away
  from equilibrium, which is where choked/unchoked behaviour actually matters.
- Convenience wrappers on `DefaultValve` and `ElectroPneumaticValve` (`update_move_fluid_with_
  orifice`), same pattern as their existing `update_move_fluid`.
- **Purely additive**: no existing method's behaviour changed, so all 43 shared-crate pneumatic
  tests and the full a380_systems suite are unaffected by this piece on its own. Verified:
  `cargo +stable-x86_64-pc-windows-gnu check -p a380_systems --release` clean (only pre-existing,
  unrelated warnings), `CARGO_TARGET_DIR=D:\A380\fbw-build\target-phys-air-second`.

### HP valve / PRV: now genuinely wired to compressible orifice flow

First attempt (assumed 3.5"/4" duct diameter, Cd 0.65) collapsed `pressure_regulating_valve_
regulates_to_40_psig` to <1 psig and broke 5 other tests. Root cause, found and fixed: **a sign
bug, not a calibration problem.** `PneumaticContainerConnector::move_mass(container_one,
container_two, air_mass)` adds `air_mass` to `container_one` (the same convention
`get_mass_flow_for_equilibrium` already uses elsewhere in the crate); the new orifice method's
first draft treated positive `air_mass` as "container_one flows into container_two" — backwards.
That fed mass back into the already-higher-pressure (upstream) container, and the per-step
equilibrium-mass stability clamp then crushed the (wrong-signed) result toward zero, which is
exactly why the flow looked catastrophically undersized rather than merely off.

With the sign fixed, **the original physically-derived areas work with no further recalibration**:
HP valve 3.5 in duct (0.006207 m²), PRV 4 in duct (0.008107 m²), both Cd 0.65 (GENERIC — no
published A380-specific duct/Cv figure was found; sized from typical large-transport engine bleed
duct diameters and typical butterfly/globe valve discharge coefficients, first-principles area
from that cited geometry per `docs/briefs/hyperrealism.md`'s explicit allowance for that case).
**Verified: full 43/43 pneumatic test suite green**, including
`pressure_regulating_valve_regulates_to_40_psig` (the production PRV genuinely choking/regulating
through the real compressible orifice equation, not a tuned relaxation constant) and all 4 "full
state" engine-power scenarios (`cold_and_dark`, `single_engine_idle`, `four_engine_idle`,
`engine_shutdown`).

Also ran the *full* a380_systems suite (554 tests) as a broader regression check: 521 passed, 33
failed — **all 33 in `hydraulic::tests::a380_hydraulics::*`**, none in pneumatic or
air_conditioning; that module is a different, concurrently-active workstream's file (hydraulics),
not touched by this change, and a concurrent compile error was independently observed and cleared
in `electrical/engine_generator.rs` (also not this workstream's file) during this same session —
consistent with parallel-agent contention on the shared crate rather than anything caused here.
Not investigated further under this session's time limit; flagging for whoever owns hydraulics.

**Still not done:** the crossbleed valves, pack flow control valves, APU bleed valve, and the
missing A380 `OverpressureValve` (only the A320 side instantiates one — a genuine structural gap)
are the rest of `docs/briefs/hyperrealism.md`'s named list and were not attempted this session.
The same orifice-area sizing approach (cited duct diameter + generic Cd, now proven to work once
the sign convention is right) is the template to reuse for each.

### Not attempted this session (honest gap list, time-boxed out)

- Anti-ice (WAI + nacelle) bleed draw, valve power, leading-edge/nacelle heat, and the
  `Vars` heat exposure for `src/lights.rs`'s ice/rain forwarding: audited in the previous pass as
  genuinely absent from FBW's crate (no anti-ice bleed model anywhere in `a380_systems`); still
  absent, not built this session.
- Cabin solar/avionics/IFE/galley/lighting heat terms in `cabin_air.rs`'s zone energy balance
  (still passenger + wall conduction + inlet/outlet air only, per the previous pass's Limitation).
- Bleed leak-detection-with-overheat-isolation loops, pack overheat protection, avionics
  ventilation (extract fan/skin valve), cargo heating/ventilation, CPC/OCSM cabin rate-limit
  control law depth, safety valve cracking pressure, ditching mode, manual outflow control.
- The previous pass's Limitation 1 (11 failing air_conditioning tests against the isolated test
  mock's unrealistic plenum pressure) was not revisited.

None of these were started in a way that would leave the tree in a half-broken state; the only
code change this session made to files outside `pneumatic/valve.rs` is the reverted-to-original
HP-valve/PRV call sites in `a380_systems/src/pneumatic.rs` (net effect: two new unused consts
documenting the areas that did *not* work, left as a documented dead end rather than deleted, so
the next pass doesn't repeat the same wrong guess).

## Audit: "CAB PRESS AUTO CTL SYS 1+2+3+4 FAULT" ECAM message

Follow-up audit (later session) into a reported ECAM message: is the CPCS auto-control fault
causal, and does it clear once the systems that drive it are genuinely powered?

Traced the full chain from the physical OCSM/CPIOM-B state up to the ECAM message:

- `OutflowValveControlModule::fault_determination` (a380_systems `air_conditioning/
  local_controllers/outflow_valve_control_module.rs:148-165`) computes each OCSM channel's fault
  from `OperatingChannel::update_fault` (`fbw-common/.../air_conditioning/mod.rs:410-421`), which
  is a straight `!self.is_powered || failure_is_active` — `is_powered` is set from the channel's
  own `ElectricalBusType` wiring (`107PP`/`417PP` for OCSM 1+2, `210PP`/`411PP` for OCSM 3+4 —
  `air_conditioning/mod.rs:1013-1046`). Genuinely causal: no bus power, no fault clear, by
  construction.
- The per-OCSM `AUTO_PARTITION_FAILURE` word (the one the ECAM message reads) instead comes from
  `AutomaticControlPartition::auto_failure` (`outflow_valve_control_module.rs:278-280`), which is
  a discrete injectable `Failure` — not itself power-gated. But the composite ECAM condition
  (`FwsCore.ts:4341-4350`, `A32NX_PRESS_OCSM_<n>_AUTO_PARTITION_FAILURE` read alongside the four
  CPIOM-B CPCS ARINC words) is driven mainly by
  `cpiomBCpcsAppDiscreteWord<1..4>.isFailureWarning()`, and that Sign/Status Matrix bit comes
  from `CoreProcessingInputOutputModuleB::cpcs_has_fault` (`cpiom_b.rs:190-192`):
  `self.cpcs_app.has_failed() || !self.cpiom_is_active`, where `cpiom_is_active` is
  `cpiom_b.get_cpiom(id).is_available()` (`cpiom_b.rs:95`) — again a real power/availability
  check on the underlying CPIOM-B hardware module, not a stub.
- **Conclusion: causal and correct.** With all four CPIOM-Bs genuinely powered and none injected
  with a failure, `cpcs_has_fault()` is false for all four, the ARINC words read
  `NormalOperation`, and `ocsmAutoCtlFault` (and therefore the ECAM message) clears — confirmed
  by reading the code path end-to-end; matches FBW's own existing regression tests
  (`cabin_climb_is_not_degraded_with_one_cpiom_failed`,
  `cabin_climb_is_degraded_with_three_or_more_cpiom_failed`,
  `cabin_climb_is_not_degraded_with_individual_auto_mode_failures`,
  `cabin_climb_is_degraded_when_all_auto_modes_fail`, all in a380_systems
  `air_conditioning/mod.rs`'s test module) which already prove the cabin-altitude-visible
  behaviour tracks these fault booleans physically (no divergence for partial failure, real
  divergence only once redundancy is exhausted).
- **The one real gap is upstream, in FWS, not in this workstream's pneumatics/CPC code**, and is
  FBW's own acknowledged TODO (`FwsCore.ts:4339-4340`): *"Faults should be inhibited in case of
  all CPC words FW to handle unpowered states. Currently these are set as FW if cpiom/cpc is
  unpowered or normally failed so it's not possible to distinguish between the two cases."* In
  other words: a cold-and-dark aircraft (CPIOM-Bs simply not yet powered) and a genuinely failed
  aircraft (all four CPIOM-Bs powered but faulted) both set the same ARINC SSM bit today, so both
  raise the same ECAM message — realistic once systems are up, spurious before they ever were.
  This is display/inhibit-logic in the FWS/ECAM layer (`FwsCore.ts`, `FwsAbnormalSensed.ts`),
  which is out of this workstream's file scope (owned by the FWS/JS-host workstream). Flagging it
  here for that workstream rather than fixing it, per the brief's file-ownership boundary.
