# Fluid physics (workstream 5): hydraulics, fuel, oxygen

Code:
- FBW patch (real, cited efficiency added to already-real pump physics):
  `D:\fbw-aircraft\fbw-common\src\wasm\systems\systems\src\hydraulic\mod.rs`,
  `electrical_pump_physics.rs`. Diff saved at
  `D:\A380\fbw-xp-systems\patches\fbw-rust\fluids.patch`.
- Plugin physics: `src/physics/fluids.rs` (orifice flow, heat transfer, Jet A
  properties, pump curves), `src/physics/gas.rs` (ideal/real gas law, O2
  regulator physics), `src/physics/hydraulics.rs` (engine-load contract
  glue).
- Existing plugin modules extended: `src/fuel.rs`, `src/oxygen.rs`,
  `src/fuel_network.rs` (two small accessors added).

## Audit finding: hydraulics was already mostly physical

Per the brief's "audit before building" rule, this workstream read
`hydraulic/mod.rs`, `electrical_pump_physics.rs` and `linear_actuator.rs`
(a380_systems/fbw-common) before changing anything. Findings:

| Component | Already physical? | Evidence |
|---|---|---|
| Engine-driven pump flow/pressure | Yes | `PumpCharacteristics::a380_edp` real displacement-vs-pressure curve at the real A380 5000 psi design pressure (`pumps.rs`); `Pump::update`/`update_after_pressure_regulation` solve real flow against circuit demand. |
| Electric hydraulic pump current draw | Yes, already real | `ElectricalPumpPhysics` (`electrical_pump_physics.rs`) computes real torque from pressure x displacement, a PID speed/current controller, and calls `consume_power`/`consume_from_bus` — the pump already draws real current from its electrical bus. **Not a shortcut**; this workstream only added a `write()` publish for the already-computed current/power (see below), not new physics. |
| A380 EHA/EBHA (electro-hydrostatic backup) actuators | Yes, already real | `VariableSpeedPump` (`linear_actuator.rs`) computes `consumed_power` from real actuator hydraulic power (`pressure x flow`) through a documented 1.15 hydraulic-to-electrical conversion factor, and calls `consume_power`/`consume_from_bus` — also already a real electrical load. Not touched further (see "Left as-is" below). |
| Engine-driven pump **shaft power back onto the engine** | **No — the one real gap** | Nothing anywhere (systems crate or plugin) ever computed the pump's own shaft power or fed it to the engine model. `ENGINE_GEARBOX_HYD_LOAD_W:n` (the shared contract) had no writer. This is the one hydraulics item this workstream had to add. |
| Fluid temperature/viscosity | No model existed | `Fluid`'s `HeatingProperties` is a qualitative overheat-state timer, not a temperature in degrees. Out of scope for this pass beyond feeding the fuel tank thermal model (see Fuel below); a full hydraulic-fluid degree-Celsius model was not built (cost/benefit: no consumer of that number currently exists in the systems crate itself; the fuel-side heat *rejected by* hydraulics into the tanks was the part with a real downstream effect, so that is what was built). |

### What changed

`EngineDrivenPump::shaft_power()` (new): `pressure x flow / SHAFT_EFFICIENCY`.
`SHAFT_EFFICIENCY = 0.90`, an aviation axial-piston-pump efficiency figure
(Power & Motion Technology's pump-efficiency survey: ~92% for a pump "in
good condition", up to 94% for modern designs; 0.90 used as a slightly
derated, defensible figure — no A380 EDP test-stand number is public). Each
pump now also publishes its own flow (`HYD_<id>_EDPUMP_FLOW`) and shaft
power (`HYD_<id>_EDPUMP_SHAFT_POWER_W`); `ElectricPump`/`ElectricalPumpPhysics`
now also publish flow, current and power (`HYD_<id>_EPUMP_FLOW`,
`HYD_<id>_EPUMP_CURRENT`, `HYD_<id>_EPUMP_POWER_W`) — all previously
computed internally but never exposed to a variable, so nothing outside the
systems crate (the plugin, the Study panel) could see them.

`physics::hydraulics::Hydraulics` (plugin) sums each engine's two EDPs
(A380: `Edp{n}a`/`Edp{n}b`, engines 1-2 on the Green circuit, 3-4 on Yellow;
`shared/mod.rs`'s `AirbusEngineDrivenPumpId::Display`) and writes
`ENGINE_GEARBOX_HYD_LOAD_W:n`, once per tick, right after the systems tick.

### Left as-is (documented, not implemented)

- Per-actuator EHA/EBHA power publish: `VariableSpeedPump` has no natural
  per-instance name (it is constructed anonymously inside four aileron/
  elevator/spoiler actuator factories in `a380_systems/hydraulic/mod.rs`),
  and threading a unique `InitContext`-registered variable through those
  four call sites was judged disproportionate to the benefit for this pass,
  since the underlying physics (current draw) is already real and already
  flows into the electrical bus load whether or not it is separately
  displayed. Left as a documented follow-up.
- A hydraulic-fluid degree-Celsius model in the systems crate itself: not
  built (see table above). The fuel-side heat exchanger (HHX) coupling
  below still connects hydraulic pump losses to fuel temperature using the
  loss *fraction* directly, without needing a fluid-temperature state
  variable of its own.

## Fuel (`src/fuel.rs`, `src/fuel_network.rs`)

### Tank temperature (FUEL-004): real heat transfer, not a 2-hour lag

**Before:** `thermal_lag(current, ambient, delta, tau=7200s)`, a flat
first-order lag toward `AMBIENT TEMPERATURE` with no wetted area, no airflow
speed, no hydraulic coupling, and no mixing between tanks on transfer — the
exact "2 h fuel-temperature time constant" the brief names for removal.

**Now**, per tank, per tick (explicit Euler, `src/fuel.rs::update_temperatures`):

```
m * cp * dT = [ U * A_wetted * (T_recovery - T_fuel) + Q_hhx ] * delta
```

| Term | Equation | Source |
|---|---|---|
| Wetted area `A_wetted` | `tank_wetted_area_m2(volume, aspect=6)`: a long, shallow box of aspect ratio 6:1:6 (span-direction:depth:span-direction), `area = 2*w^2*(2a+a^2)`, `w=(V/a^2)^(1/3)` | Geometric derivation (no public A380 tank AMM drawing); aspect ratio chosen to reflect a wing tank's long, shallow shape rather than a cube, flagged as a derived placeholder. |
| Recovery temperature `T_recovery` | `T_recovery = T_static*(1 + r*(gamma-1)/2*M^2)`, `r=0.9` | Standard compressible-flow adiabatic-wall relation; `r=0.9` is the standard turbulent-boundary-layer recovery factor. |
| Overall heat transfer coefficient `U` | Two convective resistances in series: `U = h_int*h_ext/(h_int+h_ext)` (wall conduction resistance negligible for thin aluminium skin) | `h_int = 150 W/m^2K`: typical natural-convection-to-liquid figure (Incropera & DeWitt, *Fundamentals of Heat and Mass Transfer*, typical range ~50-1000 W/m^2K for natural convection to a liquid — mid-range value, not fuel-specific). `h_ext = 10 + 5*sqrt(v_TAS)`: a common flat-plate-in-airflow engineering approximation, not wind-tunnel-derived for the A380. |
| Fuel specific heat `cp` | 2000 J/(kg K) | Commonly cited typical figure for kerosene-type jet fuel near ambient/cruise temperatures (CRC Handbook of Aviation Fuel Properties-adjacent references); not a temperature-resolved correlation. |
| Fuel density `rho(T)` | `rho(T) = rho_15 / (1 + beta*(T-15))`, `rho_15 = 802.5 kg/m^3` (= the existing `JET_A_LBS_PER_GAL = 6.699 lb/gal` MSFS/FBW constant, converted), `beta = 9e-4 /K` | `rho_15` matches FBW's own MSFS fuel-density constant exactly (no new number invented for the reference density). `beta` is a typical petroleum-distillate thermal expansion coefficient (API/ASTM D1250 volume-correction tables imply approximately this slope; not a single-sourced Jet A constant). |
| Hydraulic/IDG heat into the HHX, `Q_hhx` | `Q_hhx = HHX_EFFECTIVENESS * (hyd_load_total * 0.10 + elec_load_total * 0.15)`, split evenly across the 4 engine feed tanks | Real A380 architecture: "two fuel/hydraulic heat exchangers per circuit, one per pylon... transfer heat into the fuel flow from the outer-engine feed circuits" (Power & Motion Technology, "Hydraulics onboard the A380"). `hyd_load_total` = this workstream's own `ENGINE_GEARBOX_HYD_LOAD_W:n` sum; 0.10 is `1 - SHAFT_EFFICIENCY` (the pump's own internal loss fraction, from the same FBW patch). `elec_load_total` = `ENGINE_GEARBOX_ELEC_LOAD_W:n` (0 until electrical writes it, per the shared contract's own rule); 0.15 is a typical aviation generator (IDG) loss fraction (~85-90% efficiency), not a specific A380 IDG figure. `HHX_EFFECTIVENESS = 0.6`: a mid-range plate/shell-and-tube effectiveness-NTU figure — the qualitative HHX description above has no published effectiveness number. |
| Mixing on transfer | A tank that gains fuel this tick blends to `(T_old*V_old + T_incoming*dV)/(V_old+dV)`; `T_incoming` is the volume-weighted mean temperature of every tank that *lost* fuel that same tick | Energy conservation (mass/volume-weighted mixing), a real physical process the old per-tank-independent lag could not represent at all. Approximation: uses the network-wide loser/gainer split rather than exact per-edge routing, since `fuel_network.rs`'s flow solver does not expose which specific tank fed which. |

Freeze point (`FUEL_FREEZE_POINT_C = -40 C`, ASTM D1655 specification
maximum) is unchanged and re-exported from `physics::fluids` so both modules
agree on one number. `physics::fluids::jet_a_viscosity_cst`/
`viscosity_flow_derate` model Jet A's kinematic viscosity rising sharply near
the freeze point (a Walther-type log-log fit through two commonly published
reference points: ~1.5 cSt at 20 C, the ASTM D1655/DEF STAN 91-091 8 cSt at
-20 C specification limit) and the resulting pump flow derating.

**Now wired into `fuel_network.rs`** (second pass, this was the prior pass's
documented follow-up): `FuelNetwork::set_viscosity_derate` stores a 0.05-1.0
multiplier applied uniformly to `solve_lines`'s `gph_per_psi_unit`
conductance term. Physically this is the Hagen-Poiseuille sense in which the
network already behaves — every line's capacity is modelled as a linear
`flow = k * dP` conductance (`fuel_flow_at_1psi`), which is exactly the form
real laminar pipe flow takes with `k` proportional to `1/mu` — so a uniform
viscosity derate on that same `k` is the physically consistent hook, not a
new shortcut. `fuel.rs::update` calls it once per tick from the average of
the 4 engine-feed tanks' own temperature (`ENGINE_FEED_TANKS`, one tick
behind the temperature model that runs after the network solve; fuel thermal
mass makes that lag physically insignificant).

### Jettison flow (FUEL-001): real orifice flow, not 500 gal/h

**Before:** a flat `JETTISON_GAL_PER_HOUR = 500` constant, independent of
tank level or altitude — the exact shortcut the brief names for removal.

**Now:** the standard sharp-edged orifice equation,
`Q = Cd * A * sqrt(2 * dP / rho)` (`physics::fluids::orifice_flow_m3_s`),
with:

- `Cd = 0.7`: a sharp-edged-orifice discharge coefficient (Crane Technical
  Paper 410-style handbooks commonly cite 0.6-0.8; 0.7 is the midpoint, not
  an A380-specific nozzle test figure).
- `dP` = the jettisoning wing tanks' own gravity head (`rho*g*h`, `h` from
  the same aspect-ratio box geometry as the tank temperature model, scaled
  by current fill fraction) **plus** the drop in ambient static pressure
  below sea level (the nozzle discharges to ambient, so a lower ambient
  pressure at altitude increases the driving `dP` — jettison genuinely
  flows faster at altitude in this model, the correct physical sense).
- `Cd*A` (the nozzles' combined effective throat area) is calibrated once at
  load time from the *same* sourced reference number the old code hard-coded
  outright (`GravityBasedFuelFlow:500`, `flight_model.cfg:305-306`), at an
  assumed reference head of 2 m (an order-of-magnitude A380 outer-wing
  tank-depth guess, not a cited AMM dimension). So the nominal rate at that
  one reference condition is unchanged, but the rate now correctly falls as
  the tanks drain and responds to altitude, instead of being flat regardless
  of tank state.

### Engine burn: reads the engine model's own fuel flow

Per the shared contract, `fuel.rs` now reads `ENGINE_FUEL_DEMAND_KG_S:n`
first; if it is greater than 0 (the engine workstream's real combustor-energy-
balance fuel flow, `docs/physics/engine.md`), that mass flow burns directly
from the feed tank. If it reads 0 ("no model yet", the contract's own rule),
the existing FADEC-`ENGINE_FF`-derived volumetric path runs unchanged as a
fallback — never faking the contract variable, only choosing which real
source to burn from.

### Feed pressure and suction feed

`fuel_network.rs`'s own pump-pressure/flow solve (a generic, cfg-driven
network engine, not a shortcut) already reports 0 once a feed tank's boost
pumps are off/failed. On the real aircraft the tank's own head still
gravity/suction-feeds the engine at a much lower but non-zero pressure at low
altitude; `fuel.rs::publish` now floors the published `FUELSYSTEM ENGINE
PRESSURE:n` at a hydrostatic value (`rho*g*h` from the feed tank's own fill
height, same geometry model) instead of reporting a flat zero on pump loss.

### Pumps: real per-pump pressure/flow, and a current-draw figure

Two small, additive accessors were added to `fuel_network.rs`
(`pump_pressure_psi`, `pump_flow_gph`), exposing state the network already
computes internally (`pump_own_pressure`, the pump's destination line's own
flow) but never published per pump. `fuel.rs::publish` turns these into a
current-draw figure (`physics::fluids::pump_current_a`, hydraulic power over
`115 V * 0.75` motor efficiency — a generic aviation AC boost-pump figure,
not a per-pump AMM rating, since none is public per tank position) for the
Study panel.

### Second audit pass (fuel workstream): pump unporting from low quantity and pitch/bank

Per the brief's own gap checklist, this pass re-read `fuel_network.rs`'s pump
model before changing it. `pump_own_pressure` (all pump types) already
correctly zeroes out on loss of drive power/RPM (`refresh_pumps`'s
`drive_ok`) and already ran the tank fully dry to a hard `unusablecapacity`
cliff (`refresh_pumps`'s `tank_ok`) — genuinely not shortcuts. But an
Electric/APU-driven pump's *pressure* itself (`d.pressure`, the cfg's rated
setpoint) was a flat constant right up to that cliff: full rated pressure at
1% usable fuel, identical to a full tank, and no sensitivity at all to
aircraft attitude. This was the one item from the brief's own example list
this pass found still unaddressed: "pump outlet pressure not dropping with
low tank quantity, unporting in pitch."

**Now:** `physics::fluids::unporting_factor(fill_fraction, pitch_deg,
bank_deg, box_height_m, aspect, margin_fraction)` models a submerged boost
pump's inlet as sitting a small margin above the tank floor (the same
physical origin as `unusable_capacity`'s own hard cliff, but modelling the
*dynamic* case on top of it). Pitch and bank tilt the fuel's free surface
relative to the tank, displacing it toward one end/side:

```
tilt_m = (aspect * box_height_m / 2) * tan(|bank|) + (box_height_m / 2) * tan(|pitch|)
depth_at_inlet_m = fill_fraction * box_height_m - tilt_m
factor = clamp(depth_at_inlet_m / (margin_fraction * box_height_m), 0, 1)
```

using the same aspect-ratio tank-box geometry (`TANK_ASPECT_RATIO = 6`,
`physics::fluids::tank_box_height_m`) the tank-temperature and jettison
models already use. Bank acts over the tank's long, span-direction box
dimension (A380 wing tanks are long span-wise); pitch acts over the short
depth dimension alone, since no public chord-wise tank extent exists to
derive a second long dimension from — deliberately kept conservative/small
rather than inventing an unsupported chordwise length. The inlet's exact
position within the tank is likewise not public (no AMM drawing); the model
assumes the conservative case of an inlet at the tank's low corner, an upper
bound on unporting risk rather than a late one. `margin_fraction = 0.15` (a
derived order-of-magnitude engineering standpipe-clearance figure, not a
sourced AMM number) sets how wide a submersion band the derate ramps
smoothly across — a progressive loss of pressure as the inlet nears
uncovering, not a binary cliff, since a partially uncovered inlet ingests air
progressively.

`fuel_network.rs::FuelNetwork::set_tank_pump_derate(tank, factor)` (new, a
per-tank 0-1 multiplier, clamped, defaulting to 1.0/no derate) is applied
inside `pump_own_pressure` itself — after the existing per-type rated-pressure
calculation, uniformly for every pump type (electric, engine/APU-driven,
manual, anemometer alike, since all sit in the same tank and share the same
physical inlet) — so every downstream consumer of pump pressure (the line
solve, `pump_active`'s own `> 0.0` check, the published `pump_pressure_psi`/
`pump_flow_gph`/current-draw Study figures) sees the derated value for free,
with no separate plumbing. `fuel.rs::apply_pump_unporting` recomputes every
network tank's factor once per tick, from that tank's live total fill
fraction (`tank_total_gallons`, including unusable fuel, since physical fuel
depth is what tilts — not the usable-only quantity) and the aircraft's own
live `PLANE PITCH/BANK DEGREES`, right before `FuelNetwork::update`.

This closes a real causal chain the brief calls for directly: a low, tilted
feed tank now genuinely starves its downstream feed-line pseudo-tank (the
pump can no longer replenish it against demand), which in turn starves the
engine through the existing `starved[i]` check — engine flameout from a real
pressure/quantity failure, not a separate shortcut, falls out of this pump
model plus the pre-existing feed-line/engine-burn logic without further
change.

### Third pass (fuel second pass, ATA 28 deep dive): APU feed pressure

**Before:** the APU feed line (`fuel_network.rs`'s `Node::Apu`) had no
pressure concept at all -- only a demand/delivered gallon-per-hour flow
(`apu_demand_gph`/`apu_delivered_gph`). The engine feed lines already had a
continuous pressure (`engine_pressure`, tracked per solved line each tick),
but nothing mirrored that for the APU.

**Now:** `apu_pressure: Vec<f64>` (`fuel_network.rs`) is tracked the exact
same way `engine_pressure` already was -- inside `solve_lines`'s own per-line
loop, the highest pressure of any line entering the `Node::Apu` node this
solve, zeroed and recomputed every tick, from the same pumps/suction pressure
solve the engine feed already uses (not a separate/parallel calculation).
`FuelNetwork::apu_feed_pressure_psi(apu_index)` exposes it, mirroring
`engine_pressure_psi`'s own signature exactly.

`fuel.rs::publish` writes it to a new `APU_FUEL_FEED_PRESSURE_PSI` Vars
value (for the fire+APU workstream's own APU feed-pressure consumer, per the
brief's shared-contract convention), floored the same way the engine feed
pressure already is: the APU line pseudo-tank's own gravity/suction head
(tank 16, `fluids::tank_box_height_m`/`hydrostatic_pressure_pa`, the same
box-geometry model the engine feed pressure floor and jettison model use),
at 15 C reference density since tank 16 is an untracked pseudo-node (no
`temp_c` entry, indices 1..11 only). So a pump-off APU feed line now reads a
real, non-zero low-altitude suction pressure instead of a flat zero, the same
causal floor the engine feed pressure already had.

### FQMS quantity indication: audited, not yet implemented (documented gap)

Per the brief's item 1 ("FQMS quantity indication, independent of true
quantity"), this pass read FBW's own A380 FQMS source
(`fbw-a380x/src/wasm/systems/a380_systems/src/fuel/{mod.rs,cpiom_f/mod.rs,
cpiom_f/fuel_measuring.rs,fuel_quantity_data_concentrator.rs}`) before
touching anything, per the brief's own audit-before-building rule. Finding,
confirmed by FBW's own source comment
(`fuel_quantity_data_concentrator.rs:56-57`): *"Fuel quantities as
'calculated' by the AGP and published on an arinc 429 bus -- these values are
also used [by] the FQMS because in the sim there only exists this value."*
The FQDC's per-tank ARINC 429 quantity word is populated directly from
`A380FuelSystem::tank_mass`, which itself is fed straight from the plugin's
own `FUEL_TANK_QUANTITY_n` simvars (`fuel.rs::publish`'s `aspect_quantity`
write of `net.tank_gallons`, the network's true value) -- i.e. the entire
FQDC -> FQMS -> ECAM SD Fuel page chain is today a perfect, error-free mirror
of true fuel state. There is no capacitance-probe count, no wet/dry-vs-
attitude model, no densitometer/compensator, and no per-channel failure
distinct from the whole-CPIOM `is_powered` gate already in
`A380FuelQuantityManagementSystem`. This is exactly the "independent of true
quantity" shortcut the brief names.

Also confirmed real and already causal, so *not* a shortcut: `FuelPage.tsx`
already implements the correct real fallback chain client-side
(`fqmsWeight.valueOr(fqdc1Weight.valueOr(fqdc2Weight.valueOr(null)))`,
`FuelPage.tsx:763-785`, amber/dashes on `null`), and FQDC's own `is_powered`
already gates its whole word to `FailureWarning` SSM. Only the *quantity
measurement itself* has no independent physical model.

**Update: implemented in a final fast pass** (`fuel.rs::update_fqms`, called
once per tick right after `publish`). Kept below as the design record; the
"not implemented" framing is superseded -- see the follow-up note appended
after this section for exactly what landed and what is still simplified.

Original design (still accurate to what was built, modulo the follow-up
note): overwrite `A32NX_FQDC_{1,2}_<tank>_QUANTITY` and
`A32NX_FQMS_<tank>_QUANTITY` from the plugin, in `fuel.rs::update`, strictly
*after* FlyByWire's own systems tick (the same position `apply_pump_unporting`
etc already run at), computed from:
- a per-tank probe count derived from tank capacity (larger tank -> more
  probes to resolve a larger free-surface tilt; no AMM probe count is
  public), each probe's local wetted depth found from the same tilt geometry
  `unporting_factor` already computes (pitch/bank against the aspect-ratio
  tank box), averaged -- this recovers the true volume exactly at a level or
  mildly tilted surface (matching that real multi-probe FQIs are accurate in
  normal flight) and only biases away from true once the tilt runs the
  surface off one end of the probe array at low quantity or extreme
  attitude, the real source of FQI indication error;
- density from a first-order-lagged "densitometer/compensator" (a sensor
  settling-time filter on `physics::fluids::jet_a_density_kg_m3`, distinct
  from the removed bulk-thermal-lag shortcut: this lags a *sensor reading*,
  not the physics itself) so indicated mass can diverge from
  `true_volume * true_density` during a fast temperature transient;
- each of the two FQDC channels' own power, gated on the exact bus names
  FBW's own source cites (`cpiom_f/mod.rs:745-746`: FQDC_1 on "501PP", FQDC_2
  on "109PP"/"101PP"/"107PP"), read via the same `ELEC_<name>_BUS_IS_POWERED`
  L:var convention `bus_power_variable`/`prim.rs` already use elsewhere in
  this plugin -- reading 0 (unpowered) until the electrical workstream
  publishes those specific named-bus vars, the same shared-contract
  "reads 0 until written" rule `ENGINE_GEARBOX_HYD_LOAD_W`/`_ELEC_LOAD_W`
  already use, so the FQMS degrades to `NoComputedData`/amber XX (via FBW's
  own already-real fallback chain above) rather than silently staying
  perfect;
- ARINC 429 words encoded with the exact bit layout FBW's own
  `to_arinc429`/`from_arinc429` use (`shared/arinc429.rs:147-159`:
  `u64 = f32_bits(value) | (ssm_bits << 32)`, stored as `that u64 as f64`),
  so the plugin's overwrite round-trips through FBW's TS `useArinc429Var`
  unchanged.

No FBW patch is proposed for this item: the fix is entirely plugin-side
(overwriting the already-existing FQDC/FQMS output vars post-tick), so no
`patches/fbw-rust/` file was needed or created for it.

**What actually landed, and known simplifications** (final fast pass, time-
boxed): `physics::fluids::probe_indicated_fill_fraction`/`tank_tilt_fraction`
(new, unit-tested: exact at true fill when level, diverges only once tilt
clips the probe array) implement the multi-probe reconstruction exactly as
designed above. `fuel.rs::update_fqms` computes each tank's indicated mass
from that fraction times true Jet A density at the tank's own tracked
temperature, and writes real ARINC 429 words (verified against FBW's own
`to_arinc429` bit layout) to `FQDC_1_*`/`FQDC_2_*`/`FQMS_*` for all 11 real
tanks, gated on the two channels' real cited power buses. Two things from the
original design were *not* built in the time available, both flagged rather
than silently skipped: (1) the densitometer/compensator first-order lag --
density here uses the *true* tracked tank temperature directly, so it is
still temperature-dependent and still independent of the probe-derived
volume, but does not yet lag during a fast temperature transient; (2) the
FQMS/FQDC channel selection here is "channel 1 if powered else channel 2"
rather than FBW's own more detailed CPIOM-pairing rule
(`cpiom_f/mod.rs:809-813`, "F1&F3 default to FQDC1 - F2&F4 default to
FQDC2") -- close to FBW's own simplification (FBW's own comment there reads
"TODO: replace with better logic") rather than a new shortcut. Both are
one-tank-quantity-only refinements on top of an already-real, already-
independent measurement chain, not missing physics.

### Also audited, not yet implemented (documented gaps, brief item 4)

Ran out of session time before reaching these; each was read against the
brief's checklist but not modelled:

- **Real A380 ground feed-tank replenishment** (brief item 3): FBW's
  `LegacyFuel` (`fuel_transfer.rs`, ported from `LegacyFuel.ts`) only
  transfers in flight (module doc: "Transfers are FlyByWire's
  `LegacyFuel.ts`"); no FCOM DSC-28 ground-transfer logic was sourced or
  implemented this pass. Left as the brief's own documented fallback ("without
  it, document what you found") rather than guessed at.
- Fuel freezing/cold-soak *viscosity* is modelled (`jet_a_viscosity_cst`,
  wired into pump derate); a distinct *freezing/waxing flow blockage* beyond
  the existing viscosity derate was not added.
- Pump cavitation at altitude from vapour pressure: not modelled. The
  existing pump-pressure chain (`pump_own_pressure`, unporting) has no vapour-
  pressure/NPSH term.
- Engine fuel/oil heat exchanger return heat: the HHX (hydraulic/IDG ->
  fuel) heat path exists; a separate engine oil-cooler fuel heat return path
  does not.
- Trim tank CG-target transfers (real FCOM CG target law) and outer tank
  transfer logic: FBW's own trim tank TOML tables
  (`cpiom_f/trim_tank_targets.toml`, read by `RefuelApplication`, see above)
  already implement a real ZFW/ZFWCG-indexed trim target for *refuelling*;
  whether the same table (or a separate in-flight CG-schedule) also governs
  *in-flight* trim transfer was not checked against `LegacyFuel`'s own
  transfer logic this pass.
- Crossfeed use in abnormal procedures and fuel leak/imbalance detection: not
  audited this pass.

## Oxygen (`src/oxygen.rs`)

### Crew bottle: ideal gas law, not a flat percent-per-hour drain

**Before:** `crew_percent` depleted at a flat `100%/CREW_ENDURANCE_HOURS`
(2 h, uncited) whenever a mask was donned; pressure was just
`crew_percent/100 * 1850 psi`. No gas law, no dilution, no consumption
citation.

**Now:**

| Quantity | Equation/source |
|---|---|
| Crew bottle physical volume | Boyle's law from the commonly published A320 crew oxygen cylinder figures (repeated across multiple ATA-35 aviation training references): 1850 psig charge, 3260 L (115 ft^3) free-air capacity. `V_320 = 3260 L * 14.696/1850 = 25.9 L`. No public A380-specific AMM bottle size was found, so the A320 figure is scaled by crew count: `V_380 = V_320 * 4/2 = 51.8 L`, assuming the A380 flight deck's bottle serves up to 4 occupants (2 pilots + 2 observer seats on an augmented long-haul crew) against the A320's 2 — a documented derivation, not a sourced A380 number. |
| Bottle pressure | `P = m * R_specific * T / V` (ideal gas law), `R_specific(O2) = R/M = 8.314462618/0.0319988 = 259.8 J/(kg K)` (CODATA universal gas constant / O2 molar mass, both standard physical constants) |
| Bottle temperature | Fixed at 20 C (293.15 K): the plugin does not currently read a flight-deck interior air temperature, so a fixed cabin-ambient figure stands in — a documented simplification. |
| Real-gas check | `physics::gas::van_der_waals_pressure_pa` (standard tabulated O2 van der Waals constants, `a=0.1382 Pa*m^6/mol^2`, `b=3.186e-5 m^3/mol`) agrees with the ideal-gas figure to within ~5% at crew-bottle conditions (tested in both `physics::gas` and `oxygen.rs`), confirming the ideal gas law is an adequate model here rather than a hidden approximation. |
| Regulator consumption | `flow = CREW_COUNT * o2_mass_flow_kg_s(8 L/min * dilution_fraction)` while a mask is donned. 8 L/min is a commonly cited resting adult minute-ventilation figure from aeromedical/respiratory-physiology references (typically quoted 6-10 L/min; not aviation-specific — the closest defensible figure without a cited FAA/EASA aeromedical table in hand). |
| Altitude dilution | `physics::gas::diluter_demand_o2_fraction(cabin_alt_ft)`: ramps linearly from 21% (no dilution needed) at sea level to 100% pure oxygen by ~34,000 ft, the standard qualitative diluter-demand schedule taught in aeromedical references (not a specific A380 regulator calibration curve, which is proprietary). |
| Low-pressure caution | Unchanged: a quarter of full pressure (a common design margin), now measured against the ideal-gas-derived pressure rather than a flat percent. |

### Passenger system: chemical generators, published duration and output

The A380 passenger system is chemical oxygen generators (sodium chlorate/
iron candles, once ignited cannot be shut off or reset — a real property of
the exothermic reaction, correctly modelled by "runs to exhaustion even if
altitude drops back"). Sourced figures (Transportation Safety Board of
Canada A98H0003 supporting technical information, and the widely repeated
13/15/22-minute generator-family figure): a two-person generator yields at
least 42 L of O2, a three-person at least 62 L, a four-person at least 84 L,
each over a 15-minute decomposition. The model reports the three-person
size's flow rate (`62 L / 15 min ~= 4.13 L/min`) as the representative Study
figure (`OXYGEN_PAX_FLOW_LPM`), since the aggregate model does not track
individual seat-row generators; automatic deployment remains gated on cabin
altitude >= 14,000 ft (the existing, separately-cited Airbus/FAA/EASA
threshold).

## Tests

- `src/physics/fluids.rs`: 18 unit tests — orifice flow's sqrt(dP) scaling
  and zero-input edge cases, effective-CdA calibration round-trips, recovery
  temperature bounds, wetted-area/box-height volume scaling, Jet A density/
  viscosity against their reference points and monotonic cold-thickening,
  viscosity flow derate clamping, pump curve linearity and shutoff clamp,
  pump current scaling, heat-exchanger effectiveness zero-dT case,
  hydrostatic pressure scaling.
- `src/physics/gas.rs`: 5 unit tests — ideal-gas mass/pressure round-trip
  and hand-calculation check, van der Waals agreement with ideal gas,
  mass-flow linearity and zero/negative-input guards, diluter-demand ramp
  and clamping.
- `src/fuel.rs`: existing `parse_ini`/`split_into`/`jettison_shares`/network
  tests kept; the two obsolete `thermal_lag` tests were removed with the
  function itself (the brief's named shortcut for removal) and replaced with
  jettison-nozzle-calibration reproduction, flow-falls-with-head, and a
  pump-loss/zero-pressure never-NaN-or-negative case.
- `src/oxygen.rs`: rewritten test suite exercises the real `Oxygen::step`
  (via `crate::aspects::test_vars::TestVars`, no `Xplm` needed) rather than
  a parallel hand-rolled `step()` shim: mask-on-only depletion, ideal-gas
  pressure tracking mass within 1%, higher cabin altitude draining the
  bottle faster (a genuine physical consequence of the dilution model, not
  achievable with the old flat-rate drain), low-pressure threshold, pax mask
  deployment/exhaustion/no-reset, generator flow matching its published
  total over its duration, ideal-vs-real-gas agreement, and the crew-bottle
  volume scaling formula itself.
- `patches/fbw-rust/fluids.patch` (FBW crate): `edp_tests::
  shaft_power_is_pressure_times_flow_over_efficiency` and
  `shaft_power_is_zero_with_no_flow`, exercising `EngineDrivenPump`'s new
  method directly.

Conservation/failure coverage against the brief's checklist: mass
conservation (`jettison_shares` never removes more than present, tested;
tank mixing conserves volume by construction), a leak/failure case (jettison
and suction-feed pressure both settle at exactly zero, never negative or
NaN, with zero driving pressure), and pump loss (the suction-feed floor and
the `zero_head_and_zero_pressure_never_produce_nan_or_negative_flow` test).

## Cost

All new per-tick work is O(1) per tank/pump/engine (11 tanks, up to ~29
pumps, 4 engines) — closed-form equations, no iteration, no allocation in
the hot path. Comparable in cost to the code it replaced (a single
`exp()` call per tank before; a `sqrt()`/`cbrt()`/`powf()` or two per tank
now). Not separately profiled beyond that order-of-magnitude argument.

## What only X-Plane can verify

- Whether the tank temperature model's chosen `h_ext(TAS)`/aspect-ratio
  combination produces ECAM SD Fuel page temperatures that feel right across
  a full cold-soak-to-cruise-to-descent profile (only observable in a real
  multi-hour flight).
- Whether the jettison flow's calibrated nozzle area gives a jettison
  duration that matches FCOM/QRH guidance for a given overweight amount (no
  FCOM jettison-rate table was available to check against).
- Whether `ENGINE_FUEL_DEMAND_KG_S`/`ENGINE_GEARBOX_HYD_LOAD_W` actually
  reach the engine/electrical workstreams' consumers correctly end-to-end in
  a running sim (each side's unit tests pass independently; only a live tick
  loop exercises the full chain).
- Crew/passenger oxygen bottle depletion rates "feeling" right against a
  real rapid-decompression profile, since no A380-specific consumption/bottle
  AMM figures were available to check the derived numbers against.

## ATA-29 hydraulics restart (ATA-29 hydraulics workstream)

Picked up from a previous hydraulics agent stopped mid-edit: the lead had
already declared/initialised `EngineDrivenPump::cavitation_id` and an unused
`CAVITATION_OVERHEAT_EFFICIENCY_RATIO = 0.3` constant, but neither was wired
up. Patch: `patches/fbw-rust/hydraulics-edp-cavitation-overheat.patch`
(`git apply --check` passes against the current working tree).

**Audit finding (before touching anything further):** EDP displacement vs
pressure (`PumpCharacteristics::a380_edp`), clutch/declutch spooldown
(`SPEED_SPOOLDOWN_WHEN_DECLUTCHED_RPM_PER_S`), and cavitation from reservoir
air pressure/quantity (`Pump::update_cavitation`, `is_empty()` forcing
`cavitation_efficiency` to 0, `PumpCharacteristics::cavitation_efficiency`'s
air-pressure map) were **already real**, not shortcuts — same conclusion as
the earlier fluids pass. `ElectricPump`/`ElectricalPumpPhysics` (motor
current/torque/power), `Accumulator` (Boyle's-law gas precharge), `FireValve`
(powered, holds last position unpowered), `PriorityValve`/
`LeakMeasurementValve` (pressure-driven opening ratios) were also already
real. The one genuine gap: nothing turned a cavitating EDP into a *heating*
cause — `heat_state` was driven only by the instructor-triggered
`overheat_failure`, not by real fluid starvation.

**Fix:** `EngineDrivenPump::update` now recomputes
`is_heating_this_tick()` *after* `self.pump.update()` (so it uses this
tick's freshly-solved `cavitation_efficiency`, not last tick's), which is
true when `overheat_failure.is_active()` **or** the pump is active and its
cavitation efficiency has fallen below `CAVITATION_OVERHEAT_EFFICIENCY_RATIO`
(0.3) — modelling a starved pump losing its lubricating film and cavitation
bubbles collapsing against the pump internals generating real heat, a
documented hydraulic-pump failure mode (e.g. Parker Hannifin's "Cavitation in
Hydraulic Pumps" application note: cavitation "generates heat and can
quickly damage a pump"). `cavitation_id` is now published
(`HYD_<id>_EDPUMP_CAVITATION`, mirroring `ElectricPump`'s existing
`HYD_<id>_EPUMP_CAVITATION`) via a new `pub fn cavitation_efficiency()`
accessor. Four new tests in `edp_tests`: severe cavitation heats without any
failure injected, mild cavitation does not, an inactive (declutched) pump
with a bad cavitation reading does not, and the published variable round
-trips.

### Remaining gaps (not reached before the time limit)

- **Case-drain flow**: no explicit internal-leakage/case-drain flow path is
  modelled or published per EDP; only the net commanded flow is. A follow-up
  should add a published (not flow-balance-altering, to avoid destabilising
  the shared circuit solve) case-drain estimate using the pump's own
  pre-cavitation theoretical displacement (`PumpCharacteristics::
  current_displacement`, already accessible via `self.pump.pump_characteristics`
  in the same module) scaled by a healthy-pump leakage fraction and an
  inverse-cavitation-efficiency multiplier.
- **System fluid temperature in degrees**: still not built (same conclusion
  as the earlier fluids pass) inside the FBW systems crate. The plugin side
  (`src/physics/hydraulics.rs`) already has everything needed to build one
  additively without touching FBW's flow-conservation code: per-pump loss
  power is directly recoverable from already-published variables
  (`HYD_<id>_EDPUMP_SHAFT_POWER_W * (1 - 0.90)` for EDPs,
  `HYD_<id>_EPUMP_POWER_W * (1 - 0.95)` for electric pumps, 0.90/0.95 being
  `EngineDrivenPump::SHAFT_EFFICIENCY`/`ElectricalPumpPhysics::
  ELECTRICAL_EFFICIENCY`), and the existing fuel-side HHX model
  (`HHX_EFFECTIVENESS = 0.6`, see above) already accounts for the fraction of
  that loss rejected into fuel — the remaining `(1 - 0.6)` fraction is what a
  circuit fluid-temperature energy balance would need to track against a
  passive ambient rejection term.
- **ECAM sensor sourcing** (pressure transducers with power dependency,
  quantity sensors) and **consumer pressure sag** (flight controls, gear,
  brakes, steering, doors) were not audited in the time available; likely
  already real given how much of this file's own architecture already is,
  but unverified.
