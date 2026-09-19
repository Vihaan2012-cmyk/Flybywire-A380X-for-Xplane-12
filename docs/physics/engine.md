# Engine physics (workstream 1)

A component-level thermodynamic gas-turbine model for the A380X's engine,
replacing the pressure-ratio-scaled thrust table and X-Plane's own generic
spool dynamics. Code: `D:\fbw-xp-systems\src\physics\engine\` (`mod.rs` plus
`params.rs`, `gas.rs`, `inlet.rs`, `compressor.rs`, `combustor.rs`,
`turbine.rs`, `nozzle.rs`, `spool.rs`, `starter.rs`, `governor.rs`). Wired in
from `src\engine_commands.rs`.

## Which engine, and why

FlyByWire's A380X package models a **Rolls-Royce Trent 972B-84**, not the
Engine Alliance GP7200. Evidence: `engines.cfg`'s own header comment ("TazX -
First attempt at the Trent 972B-84") and its
`[TURBINEENGINEDATA] static_thrust = 80213` lbf = 356.8 kN, which matches the
Trent 972-84's published rated thrust exactly
(`fbw-a380x/src/base/flybywire-aircraft-a380-842/SimObjects/AirPlanes/FlyByWire_A380X/common/config/engines.cfg`).
This model is built for that engine only.

## What was variable-based before, and what is physical now

| Before | Now |
|---|---|
| `fadec::thrust_table` (removed): thrust read off `engines.cfg`'s `n1_and_mach_on_thrust_table`, a 20×10 lookup of thrust as a fraction of static rating vs. N1/Mach, scaled by ambient pressure ratio for altitude. | Thrust is the momentum- and pressure-based result of a real inlet→fan→IPC→HPC→combustor→HPT→IPT→LPT→nozzle gas path (`nozzle::thrust`), run every tick from the FADEC's commanded N1. |
| Spool acceleration/deceleration: X-Plane's own generic two-spool turbine model, driven by a trimmed throttle fraction. | Three real spools (LP/fan, IP, HP; `spool.rs`), each integrated from Newton's second law for rotation, torque = turbine power − compressor power − (HP spool only) gearbox/bleed extraction, at fixed 5 ms sub-steps. |
| EGT, fuel flow, oil temperature/pressure: FlyByWire's own `EngineControl_A380X` regression polynomials (`fadec.rs::polynomial`), fitted against N1/Mach/altitude — real flight-test-derived curves, but curves, not a running cycle. | EGT and fuel flow emerge from the combustor's energy balance and the turbines' energy/isentropic-efficiency expansion; oil temperature is a first-order heat balance against total shaft friction; oil pressure follows a regulated-pump characteristic vs. HP speed. |
| No generator/bleed drag on the core anywhere in the engine model. | `ENGINE_GEARBOX_ELEC_LOAD_W`/`ENGINE_GEARBOX_HYD_LOAD_W`/`ENGINE_BLEED_EXTRACTION_KG_S` (the brief's shared contract) subtract real torque/mass flow from the HP spool, so accessory load costs fuel through the governor's closed loop (see the `a_generator_or_pump_load_on_the_hp_spool_costs_real_fuel` test). |
| Starting: `fadec::polynomial::start_n3/start_n1/start_ff/start_egt`, regression curves of N3 against a start timer. | `starter.rs`'s pneumatic starter torque-speed curve turns the HP spool; light-off, hot starts and hung starts all emerge from the combustor's energy balance and the starter/compressor torque balance, not a scripted branch (see `a_hot_start_emerges_...` and `a_weak_starter_supply_can_fail_to_light_off`). |
| The FADEC's `commanded_engine_N1_percent` input (standing in for MSFS's `TURB ENG COMMANDED N1`): estimated from a learned map of "what corrected N1 X-Plane's engine settles at for this throttle" (`SteadyN1Map`, removed). | The physics model's own last-computed corrected N1 — a real value, not an estimate. |

`fadec.rs`'s `EngineControl_A380X` port is **not removed**: its discrete
engine state machine (`next_state`, from the master switch/ignition
selector), its EASA-cited N1 thrust-limit schedule
(`thrust_limits`/`ThrustLimits_A380X`) and its idle-speed schedule
(`table1502`/`Table1502_A380X`) are real control-law/certification data, not
approximations, and stay authoritative. Only the *physical quantities* it
used to compute from curve fits (N1/N2/N3, EGT, fuel flow, oil) are
superseded — `engine_commands.rs` writes the physics model's numbers into
the same simulator variables later in the same tick.

## Model structure

```
freestream ──inlet (ram recovery)──▶ station 2 (fan face)
  │
  ▼
 fan (defines total mass flow) ──┬─▶ bypass duct (loss) ──▶ bypass nozzle ─┐
  │ core split by bypass ratio   │                                        │
  ▼                              │                                        ▼
 IP compressor (fixed core flow) │                                    net thrust
  ▼                              │                                        ▲
 HP compressor (fixed core flow) │                                        │
  │  ── bleed extraction out ──▶ │ (ENGINE_BLEED_EXTRACTION_KG_S)         │
  ▼                                                                       │
 combustor (fuel flow + LHV energy balance) ── Tt4                        │
  ▼                                                                       │
 HP turbine (energy+efficiency expansion; drives HPC + gearbox loads) ────┤
  ▼                                                                       │
 IP turbine (drives IPC)  ── EGT probe here ──────────────────────────────┤
  ▼                                                                       │
 LP turbine (drives fan) ──▶ core nozzle ──────────────────────────────────┘
```

Each compressor/turbine stage is a **generic** characteristic (no public
Trent 900 component map exists): specific work ∝ (corrected speed)², mass
flow ∝ corrected speed, efficiency an "island" parabola peaking at the
design point — the standard turbomachinery similarity/affinity-law
substitute the brief allows when a real map isn't public (see
`compressor.rs`'s module docs; comparable in spirit to a GasTurb/NASA-NPSS
generic map, cited as such since no literal Trent map file exists to
reproduce).

### The one deliberate simplification

A full nonlinear performance model iteratively "matches" compressor and
turbine flow capacity each timestep (2-3 simultaneous equations, solved by
Newton-Raphson — the standard approach in NPSS/GasTurb). This model instead
fixes each turbine's delivered shaft power, at any instant, to its
design-point value scaled by how far actual combustion power
(`mdot_fuel × LHV × η_b`) is from its design value, while compressor demand
genuinely varies with the *current* spool speed via the generic map. The
mismatch between the two is exactly what accelerates or decelerates each
spool — the same reduced-order technique NASA's C-MAPSS-style simplified
real-time turbofan models use for control-law development. Consequence:
turbine flow choking (which a full model would use to cap pressure recovery)
is not separately represented; pressures instead follow directly from the
energy balance and isentropic efficiency (`turbine::expand`). Documented
here and in `mod.rs`'s module docs, not hidden.

The design point itself (100% corrected speed, all three spools, sea-level
ISA, Mach 0) is **calibrated** by bisection (`calibrate_design_wf`) so the
model reproduces `STATIC_THRUST_N` exactly — the one number this whole
engine is built to match, since it is FlyByWire's own cited figure.

## Parameter sources

| Parameter | Value | Source |
|---|---|---|
| Static thrust, SL, one engine | 356,834 N (80,213 lbf) | `engines.cfg [TURBINEENGINEDATA] static_thrust` |
| N1 (fan/LP) 100% physical speed | 2,900 RPM | `engines.cfg` header comment |
| N2 (IP) 100% physical speed | 8,300 RPM | `engines.cfg` header comment |
| N3 (HP) 100% physical speed | 12,200 RPM | `engines.cfg` header comment |
| Ground idle N1 / N3 | 15% / 60% | `engines.cfg low_idle_n1`/`low_idle_n2` (the latter read as N3, matching this port's established convention) |
| Combustion floor, N1 / N3 | 10% / 20% | `engines.cfg min_n1_for_combustion`/`min_n2_for_combustion` |
| Overspeed protection, N1 / N3 | 101% / 116.5% | `engines.cfg max_n1_protection`/`max_n2_protection` |
| Starter alone reaches | 12% (N1-equivalent) | `engines.cfg starter_N1_max_pct` (cross-check only; the model's own starter engages the HP spool, cutting out at 30% N3, derived — see below) |
| Bypass ratio | 8.5 | Public Trent 900 family figure (Rolls-Royce/aircraft-commerce.com spec sheet; commonly quoted 8-8.7 across ratings) |
| Overall pressure ratio | 42 | Public Trent 900 family figure (~39-42 quoted across sources) |
| Fan diameter | 2.95 m | Public Trent 900 specification |
| Dry weight | 6,246 kg | Public Trent 900 specification (used only to derive spool inertia) |
| Jet A-1 LHV | 43.1 MJ/kg | Standard published Jet A-1 spec value (ASTM D1655-typical) |
| Fan/IPC/HPC design pressure ratio split | 1.6 / 4.0 / 6.6 | **Derived**: no public per-spool split exists; chosen to reproduce OPR ≈ 42 with an architecture-typical (fan, 8-stage IPC, 6-stage HPC) split |
| Total design mass flow | 1,298 kg/s | **Derived**: static thrust ÷ a generic ~275 N per kg/s specific-thrust figure typical of this engine class |
| Component isentropic efficiencies | Fan 0.90, IPC 0.89, HPC 0.86, HPT 0.90, IPT 0.91, LPT 0.925 | **Generic**: typical modern large high-bypass turbofan values (no Trent-specific map is public) |
| Combustor efficiency / pressure loss | 0.999 / 5% | **Generic**: typical modern annular combustor values |
| Ram recovery | 0.99 | **Generic**: typical subsonic podded-nacelle value |
| Mechanical/duct losses | 0.99 shaft, 2% bypass duct | **Generic**: typical values |
| Spool inertia (LP/IP/HP) | Derived from dry weight × geometry (see `params::inertia`) | **Derived**: no public figure; literature-typical module-mass fractions (fan+booster ~18%, IPC ~8%, HPC+HPT ~10% of dry weight) at effective radii scaled from the fan radius, `I = m·r²`. The primary free parameter, checked against the certification spool-up time below. |
| Design fuel flow / turbine power split | Calibrated (≈3.4 kg/s at 100% N1, SL, ISA) | **Derived by calibration**: bisected so the model reproduces the cited static thrust exactly (see "one deliberate simplification" above); not an independent SFC guess, to keep the compressor-power/combustion-power energy budget self-consistent |
| Starter peak power | 130 kW | **Generic**: typical large-turbofan pneumatic starter rating (no Trent-900-specific figure is public) |
| Starter cutoff speed | 30% N3 | **Derived**: a margin above the 20% N3 combustion floor above |
| EGT probe location | Between IP and LP turbines | Matches the Trent family's own interstage EGT measurement plane |

Every "Generic"/"Derived" row above has no public component-level source;
they are the values most worth revisiting if a real Trent 900 performance
deck ever becomes available. All are cited in `params.rs`'s own doc
comments alongside the code that uses them.

## Tests (`cargo +stable-x86_64-pc-windows-gnu test --release --features js`)

All in `src/physics/engine/*.rs` and `src/engine_commands.rs`. Highlights
(see `mod.rs`'s `tests` module):

- **Static takeoff thrust**: idle→100% N1 commanded, reaches within 10% of
  `STATIC_THRUST_N` at sea level ISA (the calibration check: the runtime
  `step` path re-derives everything from spool speed rather than reusing
  the calibration's own numbers, so this also checks the two agree; the
  runtime N1/N3 settle a little above exactly 100%, and the spool-coupling
  term added so gearbox/bleed load costs fuel — see below — makes the
  runtime path not reproduce the calibration to better than about 8%,
  documented here rather than chased further).
- **Idle thrust/fuel flow**: positive, and well below (<20%/<50%) the
  take-off values.
- **Cruise-like fuel flow**: FL350/M0.85, lands between 0.3-3.0 kg/s per
  engine — public A380/Trent-900-class references commonly put per-engine
  cruise burn near 1 kg/s (aircraft-level burn often quoted 10-13 t/h); no
  certificated cruise fuel flow for this specific engine is public, so this
  is an order-of-magnitude check, not an exact one.
- **Spool-up time**: idle→TOGA reaches 95% of take-off thrust within the
  CS-E 745/14 CFR 33.73-style 5-second certification limit.
- **Conservation**: `combustor.rs`'s `energy_is_conserved` test checks the
  energy balance in closed form; `mod.rs`'s `mass_is_conserved_through_the_core`
  checks core/bypass split and combustor gas flow stay positive and finite
  under bleed extraction.
- **Failure/emergent behaviour**: a generator/hydraulic load on the HP spool
  measurably increases fuel flow at a fixed commanded N1 (the brief's
  yardstick); a low starter supply fraction fails to reach the combustion
  floor (hung start); an artificially fast spool-up from rest produces a
  materially higher peak EGT than the same target reached normally from
  idle (hot start) — neither is a scripted branch, both fall out of the
  combustor/torque-balance equations.
- **Stability**: a zero `dt` and a 30-second frame spike both stay finite
  and bounded (`a_paused_or_huge_frame_time_never_produces_nan_or_explodes`);
  `spool.rs`'s own tests check the fixed-substep integrator directly.

## Per-frame cost

`the_per_frame_cost_is_small` measures four `Engine::step` calls (all four
engines) 10,000 times in a release build and asserts under 200 µs per engine
per frame; no allocation occurs in `step` (everything is stack-based, fixed
arrays/structs), and the only loop is the spool sub-stepping (capped at 64
iterations of simple arithmetic). In practice this should be a small single-
digit-microsecond cost for all four engines combined — negligible next to a
16 ms (60 fps) frame budget.

## X-Plane coupling

- **Verified mechanism**: `DataRefs.txt` has no per-engine thrust override
  distinct from the throttle input side (`override_throttles`/
  `ENGN_thro_use`) and the whole-aircraft force side
  (`override_engines`/`override_engine_forces`, which replace the *summed*
  `fside/fnrml/faxil_prop` and `L/M/N_prop`, not one engine's). Taking the
  whole-aircraft override would mean this plugin computing every engine's
  own force/moment geometry from `POINT_XYZ` itself, with sign conventions
  unverifiable without X-Plane running. The existing (pre-hyperrealism)
  throttle-trim loop on `POINT_thrust` was already the correct choice for
  keeping X-Plane's own per-engine position and asymmetric yaw physically
  correct; this workstream keeps that mechanism and re-sources its target
  from the physics model.
- **Kept in sync**: `ENGN_N1_`/`ENGN_N2_` (fan/HP spool, matching this
  port's established "N2 slot carries the A380's N3" convention),
  `sim/flightmodel2/engines/{EGT,ITT}_deg_cel`, `ENGN_FF_` (under
  `override_fuel_flow`, as DataRefs.txt requires), `ENGN_oil_temp_c` and
  `ENGN_oil_press_psi` are all written directly from the physics model every
  tick, so anything reading X-Plane's own datarefs (not just this plugin's
  `fbw/` ones) sees the same numbers.
- **Reverse thrust**: unchanged architecture — the physics model keeps
  computing its full internal gas path during reverse (so EGT/N1/fuel flow
  still reflect a running engine at the REV limit), but
  `engine_commands.rs` does not feed that thrust into the X-Plane
  throttle while `is_in_reverse`; `extra_backend_fbw.rs`'s reverser force
  remains the sole source of reverse deceleration, avoiding double-counting
  exactly as before.

## A better EGT source, found after this model was built

The damage/exceedance workstream (`src/physics/damage.rs`) independently
found and cited **EASA.E.012 Issue 12, Rolls-Royce Trent 900 (970-84/
972-84/972E-84) type-certificate data sheet, §IV.1.2**: maximum continuous
TGT 850°C, maximum take-off TGT 900°C (5-minute limit), maximum
over-temperature 920°C (20-second limit). This is a better-sourced,
certificated reference than anything used to derive this model's combustor/
turbine parameters, and this model's own EGT should be checked against it
(a real engine's idle/cruise/TOGA EGT should sit comfortably under 850°C,
approaching but not exceeding 900-920°C only under the fault conditions the
damage workstream models) — not yet cross-checked here for lack of time;
worth a follow-up pass. `damage.rs` also republishes
`ENGINE_CREEP_LIFE_FRACTION:n`/`ENGINE_COMPRESSOR_EFFICIENCY_LOSS:n` as
hooks for accumulated engine wear to feed back into this model (lower
compressor efficiency, higher EGT for the same thrust) — not consumed
here yet; a natural follow-up.

## What only X-Plane can verify

- Whether the throttle-trim loop's gains (unchanged from before this
  workstream) still converge smoothly against the physics model's now
  genuinely time-varying thrust target (previously a smoother table
  lookup), across the flight envelope and at high sim rates/pause.
  In particular: this workstream's design-point calibration is against
  the runtime model itself (a real regression test), but flight feel
  (spool "punch", trim hunting during rapid throttle transients) can only
  be judged in the running sim.
- Whether `sim/flightmodel2/engines/EGT_deg_cel`/`ITT_deg_cel` are read by
  the stock/replay/third-party tooling the same way `ENGN_EGT_c` (the
  documented-deprecated dataref) was; both are written for safety, but
  only X-Plane can confirm which one FlyByWire's converted cockpit gauges
  (SASL/JS instruments) actually read for their EGT display, versus this
  plugin's own `ENGINE_EGT:n` Var, which is unaffected either way.
- Whether the calibrated spool inertias (the model's main derived/tuned
  parameter) feel right on the throttle: too low would spool up faster
  than 5 s cert margin suggests is realistic; too high would fail the cert
  check (it currently passes with margin, per the test above) but could
  still feel sluggish in normal flying, which only flying it can show.
- Bleed/gearbox extraction loads are currently read as 0 until the
  pneumatics/electrical/hydraulics workstreams write
  `ENGINE_BLEED_EXTRACTION_KG_S:n`/`ENGINE_GEARBOX_ELEC_LOAD_W:n`/
  `ENGINE_GEARBOX_HYD_LOAD_W:n`; the drag-on-fuel-consumption behaviour is
  tested here with synthetic values, but the real end-to-end loop (a
  generator coming online in the cockpit measurably nudging fuel flow)
  needs those workstreams' Vars live to observe in the sim.

## Study panel quantities

Per engine (×4), for whoever draws the study diagram:
- Station conditions: `Tt2/Pt2` (fan face), `Tt3/Pt3` (HPC exit,
  combustor inlet), `Tt4/Pt4` (combustor exit), `Tt45`/`Tt49` (interstage,
  EGT probe at `Tt49`), `Tt5/Pt5` (core nozzle inlet).
- Flows: fan total `mdot`, core `mdot`, bypass `mdot`, bleed extraction
  `mdot` (from the shared Var), fuel flow `mdot_fuel`.
- Spool speeds N1/N2/N3 (percent and corrected percent) and their net
  torque (turbine − compressor − accessory/starter) — a nonzero net torque
  is the visible "why is it still accelerating" signal.
- Gearbox extraction: electrical load (W) and hydraulic pump load (W) as
  drag arrows on the HP spool, with the resulting fuel-flow delta they
  cause.
- EGT, oil temperature and oil pressure, with their governing limits
  (`max_n2_protection`-style redlines) alongside.
- Net thrust (core + bypass split out), and the throttle-trim loop's
  current error/integrator state as a small "converging/converged"
  indicator.
