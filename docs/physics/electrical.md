# Electrical physics (workstream 2): a real circuit over FlyByWire's A380 topology

Code:
- FBW patch (real physics replacing flat-voltage/arbitrary-current shortcuts):
  `D:\fbw-aircraft\fbw-common\src\wasm\systems\systems\src\electrical\{mod.rs,
  battery.rs, engine_generator.rs, external_power_source.rs}`,
  `D:\fbw-aircraft\fbw-common\src\wasm\systems\systems\src\apu\pw980.rs`. Diff
  saved at `D:\A380\fbw-xp-systems\patches\fbw-rust\electrical.patch`.
- Plugin physics: `src/physics/electrical.rs` (engine-load contract glue,
  circuit-protection/breaker model). Registered in `src/physics/mod.rs`,
  wired into `src/lib.rs` (fields `electrical_loads`/`circuit_protection`,
  built after `circuits`, updated after the systems tick and before
  `fuel`/`lights`).
- Existing plugin module read, not modified: `src/circuits.rs` (the
  systems.cfg circuit/breaker model; this workstream's breaker trips call its
  existing `Circuits::set_breaker`/`Circuits::powered`/`Circuits::breaker_closed`).

## Audit finding: FlyByWire's electrical crate is a topology graph, not a solved circuit

Per the brief's "audit before building" rule, this workstream read
`fbw-common/.../electrical/{mod.rs, battery.rs, engine_generator.rs,
transformer_rectifier.rs, static_inverter.rs, external_power_source.rs,
emergency_generator.rs, ram_air_turbine.rs, consumption.rs}`,
`apu/pw980.rs`'s generator, and `a380_systems/.../electrical/{mod.rs,
alternating_current.rs, direct_current.rs}` before changing anything.

| Component | Already physical? | Evidence |
|---|---|---|
| Bus/contactor topology (which source feeds which bus) | **Yes, and kept as-is** | `Electricity`/`Potential` (`electrical/mod.rs`) merge elements into equipotential groups only when a chain of *closed* `Contactor`s connects them to a source (`Electricity::flow`/`supplied_by`). For near-zero-impedance busbars and contactors this **is** the real graph-connectivity half of Kirchhoff's laws, and paralleled sources of nearly-equal voltage share load by splitting consumption evenly across origins (`PotentialCollection::consume_from`), which is also how real paralleled AC generators are made to load-share (droop control). Not a shortcut; not touched. |
| Generator/TRU/battery/external-power **terminal voltage** | **No — the actual gap** | Every source wrote a flat nameplate voltage regardless of current: `EngineGenerator::process_power_consumption_report` set `output_potential = 115V` unconditionally (`engine_generator.rs`, pre-patch); `Pw980ApuGenerator::calculate_potential` returned a flat 115 V "after 78% N1 ... and stays there throughout"; `ExternalPowerSource`/`StaticInverter`/`EmergencyGenerator` did the same. Only `TransformerRectifier` was already real (see below). |
| Battery internal resistance/current limit | **No — explicitly a placeholder in FBW's own comment** | `Battery::calculate_charging_current` (pre-patch): `// Internal resistance = 0.011 ohm. However that would make current go through the roof. Thus we add some fake wire resistance here too.` — a fixed 0.15 ohm and a flat 10 A cap, with the real 0.011 ohm cell figure computed and then discarded. Discharge current was `-(consumption / output_potential)` with no IR drop at all (the battery's own resistance was applied on charge only, and even then capped arbitrarily). |
| TRU (`TransformerRectifier`) output impedance | **Yes, already real — a useful precedent** | `transformer_rectifier.rs` already had a real 0.0135 ohm output impedance and idle voltage (30.2 V), solving `V^2 - V_idle*V + P*R = 0` for the loaded terminal voltage (`calc_potential_for_power`). This workstream's generator/battery/external-power fixes reuse exactly this quadratic-in-V technique for consistency, rather than inventing a different method. Not modified except to remain internally consistent with the newly-real sources feeding it. |
| RAT/emergency generator shaft drag | **Yes, already real — a useful precedent** | `ram_air_turbine.rs::resistant_torque` already computes a real resistant torque from the emergency generator's own electrical power output and feeds it back as RAT drag — exactly the "generator load imposes mechanical drag" pattern the brief's yardstick describes, already present for the RAT. This workstream's engine/APU generator shaft-power exposure (below) extends the *same idea* to the four VFGs and two APU generators, which had no equivalent at all. |
| Engine generator shaft power onto the engine | **No — no writer existed** | Nothing in the systems crate or the plugin ever computed a VFG's or APU generator's shaft power or fed it anywhere; the shared contract's `ENGINE_GEARBOX_ELEC_LOAD_W:n`/`:0` had no writer at all. |
| Circuit breakers / SSPCs | **No — FBW's crate has none** | Grep for "breaker"/"SSPC"/"trip" across `fbw-common`/`a380_systems` electrical code returns nothing. `src/circuits.rs` (this plugin, pre-existing, B+E workstream) already models a manually-toggleable breaker (`CIRCUIT BREAKER CLOSED:n`) for the MSFS-side `systems.cfg` circuits, but "not called anywhere yet" — no current/trip physics existed for it. |

## Equations and parameter sources

### 1. Generators (VFG/`EngineGenerator`, APU/`Pw980ApuGenerator`): Kirchhoff loop across an equivalent synchronous reactance

Real electrical power delivered, `P`, comes from FlyByWire's own
`PowerConsumptionReport::total_consumption_of` (unchanged — this was already
the real sum of every consumer's demand on that origin; only what the
generator *did* with that number was a shortcut). Apparent power at the
nameplate power factor (`POWER_FACTOR = 0.8`, FBW's own pre-existing
constant, `engine_generator.rs`/`pw980.rs`): `S = P / 0.8`.

Terminal voltage solves the same quadratic-in-V Kirchhoff loop
`transformer_rectifier.rs` already used for its own output impedance:

```
V = V_rated - I*Xs,   I = S/V   =>   V^2 - V_rated*V + S*Xs = 0
V = (V_rated + sqrt(V_rated^2 - 4*S*Xs)) / 2
```

When `S` exceeds the reactance's own maximum power transfer
(`V_rated^2/(4*Xs)`, discriminant negative), the source is power-limited, not
current-limited: `V = V_rated/2`, matching a real generator's own current
limiter collapsing terminal voltage under a severe overload rather than
allowing the old code's unbounded current at near-zero voltage.

`Xs` is sized so that voltage sags exactly `RATED_VOLTAGE_REGULATION` at
rated apparent power `S_rated = max_true_power / 0.8`:
`Xs = V_target*(V_rated - V_target) / S_rated`, `V_target = V_rated*(1 -
RATED_VOLTAGE_REGULATION)`.

| Parameter | Value | Source |
|---|---|---|
| `V_rated` (VFG/APU gen) | 115 V | FBW's own pre-existing constant (`engine_generator.rs`, `pw980.rs`) |
| `POWER_FACTOR` | 0.8 | FBW's own pre-existing constant, both files |
| VFG rated power | 150 kW | FBW's own `VariableFrequencyGenerator::new(..., Power::new::<kilowatt>(150.), ...)` (`alternating_current.rs:393`) — matches the real A380's four 150 kVA variable-frequency generators |
| APU generator rated power | 120 kW | FBW's own pre-existing `Pw980ApuGenerator::MAXIMUM_LOAD_WATT` (`pw980.rs`, already used for the `load` percentage) |
| `RATED_VOLTAGE_REGULATION` | 3% | **Derived, no FBW/public figure.** Chosen so a generator loaded to exactly 100% of its rating still reads inside FBW's own `potential_normal` band (110-120 V for 115 V nominal) — consistent with `output_within_normal_parameters`'s own doc comment that reaching rated load should not by itself flip the generator abnormal (real overload trips via time-integrated overtemperature, not an instant voltage collapse) |
| `GENERATOR_EFFICIENCY` | 0.88 | **Derived/typical**, no FBW figure: a typical peak efficiency for a high-power brushless aircraft AC generator |

Shaft power for the engine-load contract:
`shaft_power_demand = real_power_output / GENERATOR_EFFICIENCY`, published as
a plain simulator variable (`ELEC_ENG_GEN_n_SHAFT_POWER_DEMAND`,
`ELEC_APU_GEN_n_SHAFT_POWER_DEMAND`) so the plugin can read it without a Rust
API across the plugin/systems crate boundary — the same pattern
`physics::hydraulics::Hydraulics` already uses for the hydraulic term.
`physics::electrical::EngineLoads` sums these into
`ENGINE_GEARBOX_ELEC_LOAD_W:n` (n=1..4) and `:0` (both APU generators
summed onto the APU's shared gearbox).

### 2. External power (ground power unit)

Same quadratic, sized to a stiffer, better-regulated source:

| Parameter | Value | Source |
|---|---|---|
| Rated apparent power | 90 kVA | **Typical/derived**: a common widebody ground power cart rating; no FBW figure |
| Regulation at rated load | 2% | **Typical/derived**: ground carts are tightly regulated compared with an engine-driven generator |

### 3. Batteries: equivalent circuit (OCV(SOC), internal resistance vs temperature, thermal model)

The open-circuit-voltage-vs-charge polynomial (`Battery::
calculate_output_potential_for_charge`, `Battery.md`'s curve-fit) is
FlyByWire's own and unchanged — it is a real equivalent-circuit OCV(SOC)
curve, not a shortcut.

**Internal resistance vs temperature** (new):

```
R(T) = (R_cell + R_wiring) * (1 + ALPHA * max(0, T_ref - T))
```

| Parameter | Value | Source |
|---|---|---|
| `R_cell` | 0.011 ohm | **FBW's own figure**, previously computed and then discarded (`battery.rs`'s old comment) |
| `R_wiring` | 0.02 ohm | **Derived**: a short, heavy-gauge battery-to-busbar run; no FBW figure |
| `ALPHA` (temp coefficient) | 2%/C below 20 C | **Typical/derived**: Ni-Cd internal resistance rises markedly in the cold; no FBW/A380-specific figure |
| `T_ref` | 20 C | Standard reference temperature |

**Thermal model** (new): first-order energy balance,
`dT/dt = (I^2*R - h*(T - T_ambient)) / C_thermal`, integrated explicitly each
tick (`context.delta_as_secs_f64()`).

| Parameter | Value | Source |
|---|---|---|
| `C_thermal` (thermal mass) | 6000 J/K | **Derived**: ~20 kg Ni-Cd cell stack at ~0.3 J/(g*K); no FBW/public per-cell mass figure |
| `h` (cooling coefficient) | 2.5 W/K | **Derived**: natural convection to the avionics-bay air; no FBW figure |

**Discharge** (new): the same quadratic-in-V technique as the generators,
applied to the battery's own OCV/resistance loop:
`V^2 - OCV*V + P*R = 0`, taking the physical (`+`) root. When the requested
load exceeds the equivalent circuit's own maximum power transfer
(`OCV^2/(4R)`), the battery is power-limited (`V = OCV/2`), not
current-limited — this replaces the old code's unbounded current at very low
voltage (which is what let the old model instantly and unphysically drain a
near-dead cell; see "Tests" below for the resulting, deliberate test
behaviour change).

**Charging** (`calculate_charging_current`, changed): the same series
resistance `R(T)` above, Ohm's law `(V_bus - OCV) / R`, capped at
`MAX_CHARGE_CURRENT_AMPERES = RATED_CAPACITY_AMPERE_HOURS` (a physically
motivated 1C fast-charge rate, replacing the old arbitrary flat 10 A cap
FBW's own comment called "fake").

**Numerical floor**: below `PRACTICALLY_EMPTY_AMPERE_HOURS` (1e-4 Ah, a
small fraction of the 23 Ah rated capacity), the cell is snapped to exactly
empty rather than left to asymptotically decay over simulated hours (a real
cell's own low-voltage protection/BMS would already have declared it dead at
that OCV, which by the polynomial curve is already a small fraction of a
volt).

### 4. TRU, RAT/emergency generator: left as-is (already real); static inverter conversion loss (new)

`transformer_rectifier.rs`'s output-impedance solve and
`ram_air_turbine.rs`'s resistant-torque feedback were already physical (see
audit table) and are unmodified. `emergency_generator.rs` was **not** given
the same equivalent-source-impedance treatment in this pass (flat 115 V
regardless of load, unlike every other AC source in this file); still
flagged as a documented, lower-priority follow-up (its GCU-vs-RPM max-power
map, `ram_air_turbine.rs::GeneratorControlUnit`, already is a real "generator
output vs RAT rpm" curve, so what's missing is only the terminal-voltage sag
under load, not the emergency power chain as a whole).

**Static inverter conversion efficiency** (new, closes a gap this file
previously flagged as a follow-up): `static_inverter.rs`'s own comment said
plainly "Currently static inverter inefficiency isn't modelled" — DC
(battery) input was set exactly equal to AC output. `consume_power_in_converters`
now divides by `EFFICIENCY = 0.85` (typical/derived peak efficiency for a
small solid-state PWM DC-AC inverter; no FBW/A380 figure exists) so the
ESS battery sees a real, somewhat-higher DC draw than the AC load it is
powering through the inverter. Terminal-voltage equivalent-source-impedance
sag (matching the treatment given every other AC source in this file) is
still outstanding for this component — flagged as the next step, not
silently dropped.

### 4b. Battery overcharge thermal runaway (new)

`Battery::update_temperature`'s existing I^2R/convective thermal model (see
above) had no path for a charger that keeps forcing current into an
already-full cell to do anything but linearly warm it via I^2R. Real Ni-Cd
cells pushed past rated capacity while still on charge electrolyse
(gas) instead of storing energy, converting essentially all of that
overcharge power directly to heat, and — because that heat is not bounded by
a fixed series resistance the way I^2R is — an already-warm, still-
overcharging cell gets a genuine positive-feedback thermal-runaway curve
rather than an asymptotic steady temperature (Linden & Reddy, "Handbook of
Batteries", Ni-Cd overcharge/thermal-runaway section). Implemented as
`Battery::overcharge_heating_watt`, added into the existing thermal balance:
active only while charging (`current < 0`, this file's own sign convention)
and `charge > RATED_CAPACITY_AMPERE_HOURS`, contributing
`|current| * output_potential * GASSING_HEAT_FRACTION * overcharge_ratio`
watts, multiplied by an Arrhenius-like `2^((T - RUNAWAY_ONSET_C) /
RUNAWAY_DOUBLING_C)` once the cell is already above `RUNAWAY_ONSET_C`. All
three constants (`GASSING_HEAT_FRACTION = 0.5`, `RUNAWAY_ONSET_C = 45`,
`RUNAWAY_DOUBLING_C = 10`) are derived/typical — no FBW/A380-specific
figure exists — chosen so ordinary overcharge (already exercised by the
pre-existing `can_charge_beyond_rated_capacity` test, which charges for
1000 s past full) stays a gentle warm-up rather than an instant runaway,
while a charger left connected to an already-hot, already-overcharged pack
does accelerate. `docs/physics/electrical.md`'s existing internal-resistance
model is unaffected (that term still only depends on temperature below the
20 C reference, not above it).

### 5. Per-bus voltage publish

`ElectricalBus::write` (`electrical/mod.rs`) now also writes
`ELEC_<bus>_BUS_POTENTIAL` (raw volts) for every non-sub bus, not only the
battery bus's existing `_BUS_POTENTIAL_NORMAL` boolean — the brief's
explicit Study quantity ("per-bus voltage"). Since the bus's own `potential`
field is fed from the merged `Potential` group's raw value
(`ElectricalBus::receive_power`), this now carries the real, load-sagged
voltage from whichever Kirchhoff-solved source feeds it.

### 6. Circuit protection (plugin-side; stage-3 foundation)

FBW's crate has no breaker/SSPC model at all, so this is entirely new,
built on top of the existing `src/circuits.rs` (B+E workstream)'s
`CIRCUIT BREAKER CLOSED:n` state, which until now was "not called anywhere
yet".

**Per-circuit real current**: `src/circuits.rs`'s `circuit.N` entries come
from the embedded `systems.cfg`'s `[ELECTRICAL]` section (buses, pumps,
valves, lights, avionics, ...), not from FlyByWire's own systems crate,
which only models load in per-bus aggregate (`a380_systems/
power_consumption.rs`'s `FlightPhasePowerConsumer`s, "the watts in this
function are all provided by komp") rather than per physical circuit. So a
per-circuit current needed a per-circuit load assignment; `rated_watts()`
(`physics/electrical.rs`) assigns each `CIRCUIT_*` type a generic
large-transport-aircraft equipment rating (all **typical/derived, no FBW or
type-certificate source**, per the brief's allowance for "public A380/
engine references ... or first-principles derivation", here public/typical
generic equipment figures since no A380-specific number is public per
circuit):

| Circuit type | Rated load | Basis |
|---|---|---|
| `CIRCUIT_FUEL_PUMP` | 600 W | Typical AC boost-pump motor, large transport aircraft |
| `CIRCUIT_FUEL_VALVE` | 50 W | Typical motor-operated shutoff valve actuator |
| `CIRCUIT_LIGHT_LANDING` | 600 W | Typical transport-category landing light |
| `CIRCUIT_LIGHT_TAXI` | 250 W | Typical taxi light |
| `CIRCUIT_LIGHT_NAV`/`RECOGNITION` | 40 W | Typical position/recognition light |
| `CIRCUIT_LIGHT_BEACON` | 100 W | Typical rotating beacon |
| `CIRCUIT_LIGHT_STROBE` | 300 W | Typical xenon strobe power supply |
| `CIRCUIT_LIGHT_LOGO`/`WING` | 150 W | Typical wing/tail floodlight |
| `CIRCUIT_LIGHT_CABIN` | 200 W | Typical cabin lighting zone |
| `CIRCUIT_LIGHT_PANEL`/`PEDESTAL`/`GLARESHIELD` | 20-30 W | Typical instrument-panel lighting |
| `CIRCUIT_GEAR_MOTOR` | 1500 W | Typical landing-gear extension/retraction motor |
| `CIRCUIT_GEAR_WARNING` | 20 W | Small annunciator/horn circuit |
| `CIRCUIT_PITOT_HEAT` | 600 W | Typical pitot-probe heater element |
| `CIRCUIT_STARTER`/`APU_STARTER` | 2000 W | Contactor/control-circuit load (not the full cranking-motor current, which is a separate, much larger DC-bus event not modelled at circuit-breaker granularity) |
| `CIRCUIT_STANDBY_VACUUM` | 100 W | Typical standby vacuum motor |
| everything else (avionics LRUs: `CIRCUIT_COM`/`NAV`/`XPNDR`/`ADF_DME`/`AUTOPILOT`/`AUDIO`/`FIS`/`MFD`/`PFD`/`SAI`/`ADC_AHRS`/`AVIONICS`/`GENERAL_PANEL`/... ) | 50 W | Generic small-avionics-box fallback rating |

Current is then `I = rated_watts / V_bus`, using the real, Kirchhoff-solved
`ELEC_<bus>_BUS_POTENTIAL` from item 5 above (AC buses at their nominal
115 V, DC buses at 28 V, matched by MSFS bus number), and only drawn when
`Circuits::powered()` is true (bus powered, breaker closed, connection
pushbutton made) — a documented simplification: a circuit that is powered
but whose consumer is not actually active (e.g. a fuel pump circuit powered
but the FQMS has not commanded that pump on) is treated as drawing its full
rated current, which can overestimate load on lightly-loaded ties. Getting
this exactly right would need each consumer module (fuel.rs, lights.rs, ...)
to publish its own real-time on/off state per circuit, out of scope for this
pass.

**Trip curves**: a real A380 uses SSPCs (solid-state power controllers)
almost throughout in place of thermal-magnetic breakers; this module
implements the two curve shapes both technologies share:

- **I^2t thermal** (inverse-time): a normalised heat accumulator,
  `d(heat)/dt = ((I/I_rated)^2 - 1) / K` for `I > I_rated`, tripping at
  `heat >= 1`, cooling at `-1/COOLDOWN_SECONDS` otherwise. `K = 30` is
  chosen so 2x rated current trips in `K/(2^2-1) = 10` s (verified by test,
  see below) — a **typical/derived** shape and time constant for aerospace
  SSPC/breaker practice; no FBW source, since FBW has none at all.
- **Magnetic instant trip**: `I >= 10 * I_rated` trips within the same tick
  (`MAGNETIC_TRIP_MULTIPLE = 10`, **typical/derived** pickup multiple).

**Fault injection**: `CIRCUIT FAULT CURRENT MULTIPLE:n` (default 0, meaning
"draw rated current when live") lets a short or other overcurrent failure be
simulated (a value above 1 multiplies the assumed rated current), satisfying
the brief's "shorts as failures" test requirement; `failures.rs` or the
Study panel can drive this in a future pass.

**Trip effect and reset**: `Protection::trip` calls
`Circuits::set_breaker(..., false)` — the *same* breaker state
`fuel.rs`/`lights.rs` already gate their own power on, so a trip immediately
and really cuts that consumer, and `Circuits::list`/`breaker_closed` (which
the Study CB page reads) shows it tripped. A pulled/tripped breaker's
thermal state resets to zero while open (matching a real bimetal element
cooling), so a manual reset (`Circuits::set_breaker(..., true)`, e.g. from a
future Study CB page) can carry load again immediately if the fault has
cleared, or re-trip on the same curve if it has not.

## Study quantities

- Per bus: `ELEC_<bus>_BUS_POTENTIAL` (volts, new), `ELEC_<bus>_BUS_IS_POWERED`
  (existing).
- Per generator/TRU/battery/external-power source: `ELEC_<id>_POTENTIAL`,
  `ELEC_<id>_CURRENT` (new for engine/APU generators and external power;
  already existed for TRUs/batteries), `ELEC_<id>_LOAD` (generators),
  `ELEC_ENG_GEN_n_SHAFT_POWER_DEMAND`/`ELEC_APU_GEN_n_SHAFT_POWER_DEMAND`
  (new — the engine-load contract's own basis), `ELEC_BAT_n_TEMPERATURE`,
  `ELEC_BAT_n_INTERNAL_RESISTANCE` (new).
- Shared engine-load contract: `ENGINE_GEARBOX_ELEC_LOAD_W:1..4` and `:0`
  (APU).
- Per circuit breaker (`circuits.rs`'s `circuit.N`): `CIRCUIT CURRENT:n`
  (new), `CIRCUIT BREAKER CLOSED:n` (existing, now also driven by a real
  trip), `CIRCUIT TRIP CAUSE:n` (new: 0 none, 1 thermal, 2 magnetic),
  `CIRCUIT FAULT CURRENT MULTIPLE:n` (new, test/failure-injection input).

## Tests

FBW-side (`cargo +stable-x86_64-pc-windows-gnu test --release`, both
`fbw-common/src/wasm/systems/systems` and `fbw-a380x/src/wasm/systems/
a380_systems`):
- Whole-crate regression: 1396 + 12 doctests (`systems`), 87 electrical
  circuit tests + 199 APU tests (`a380_systems`) all still pass after the
  patch.
- New: `battery.rs` — `internal_resistance_is_higher_in_the_cold`,
  `discharge_terminal_voltage_sags_with_load_like_a_real_cell`,
  `an_overload_beyond_maximum_power_transfer_is_power_limited_not_infinite`
  (the generator-overload/no-NaN check), plus the existing discharge-curve
  tests (`when_discharging_loses_charge`, `batteries_charge_each_other_
  until_relatively_equal_charge`, `dissimilar_charged_batteries_in_
  parallel_deplete`) updated where the new, more physical (self-limiting,
  lossy) behaviour changed their old, unphysical-shortcut-dependent
  expectations (documented inline at each change — see "Behaviour changes"
  below).
- New: `engine_generator.rs` — `terminal_voltage_sags_more_under_a_heavier_
  load`, `current_scales_with_real_power_delivered`, `shaft_power_demand_
  is_real_power_output_over_efficiency`, `no_load_means_no_shaft_power_
  demand`.

Plugin-side (`src/physics/electrical.rs`, `cargo test --features js`):
- `a_normal_load_never_trips` (KCL/steady-state sanity).
- `a_short_circuit_fault_trips_instantly_on_the_magnetic_curve` (shorts as
  failures; magnetic curve).
- `a_moderate_overload_trips_on_the_thermal_curve_near_its_predicted_time`
  — **breaker trip time versus the curve**: asserts the trip happens at
  `K/(2^2-1) = 10s +/- 0.2s` for a 2x overload, i.e. the actual simulated
  trip time matches the curve's own closed-form prediction, not just "trips
  eventually".
- `a_heavier_overload_trips_faster_than_a_lighter_one` (inverse-time shape).
- `a_reset_breaker_can_carry_load_again_once_the_fault_clears` (pop and
  reset).

### Behaviour changes in existing FBW tests (documented, not silent)

Two pre-existing `battery.rs` tests asserted the *old* model's unphysical
consequence of an uncapped low-voltage current (a near-dead cell instantly
and fully discharging within 15-50 simulated seconds under load). With a
real internal resistance, a near-dead cell's own maximum-power-transfer
limit means it can only deliver a vanishingly small power at its own
(fraction-of-a-volt) OCV, so it self-limits rather than instantly dumping
its last charge — a more realistic failure mode (a dying battery's voltage
sags rather than instantly cutting off). Both tests were updated to check
the physically meaningful property (no free energy created; charge
converges/decreases; a moderate discharge still empties within the same
order of time) instead of the old exact-zero-within-N-seconds timing that
depended on the removed shortcut; each change is commented in place in
`battery.rs` with the reasoning.

## Cost

All per-tick work here is O(number of sources) + O(number of circuits)
(151 circuits in the embedded `systems.cfg`), each a handful of scalar
floating-point operations (one `sqrt` per Kirchhoff-loop solve, one
multiply-accumulate per breaker's thermal state) — no allocation, no
iteration to convergence. Negligible next to the systems tick itself.

## What only X-Plane can verify

- That the Study panel's per-bus voltage/current and per-breaker
  current/trip-state actually render and update live in X-Plane (this
  workstream only verified the underlying values via `cargo test`).
- Perceived realism of the generator voltage sag and battery discharge
  curve during an actual electrical failure scenario flown in the sim
  (engine-out, battery-only, external power connect/disconnect), including
  whether `RATED_VOLTAGE_REGULATION`/`Xs` feel right against the cockpit's
  own ELEC SD page needles once the Study/ECAM lead wires them to these new
  variables.
- Whether the circuit-protection load table's current estimates, applied
  across all 151 real `systems.cfg` circuits simultaneously in a full
  flight, produce a sensible total current picture (this pass validated the
  model's mechanics with unit tests on a couple of representative circuits,
  not an end-to-end sum across the whole aircraft in a running sim).
