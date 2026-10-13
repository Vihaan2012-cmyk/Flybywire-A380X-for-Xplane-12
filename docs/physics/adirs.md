# ADIRS navigation sensors (physics workstream 4)

Scope: the 3 ADIRUs' IRS (strapdown inertial navigation) and ADR (pitot-static
air data), and the 3 radio altimeters' terrain range. Brief:
`docs/briefs/hyperrealism.md`. Code: `src/physics/adirs.rs` (the plugin's new
sensor physics), plus a small additive patch to FlyByWire's own
`fbw-common/.../navigation/adirs.rs` and `ala52b.rs`
(`patches/fbw-rust/navigation-sensors.patch`).

## Audit: what was here before

FlyByWire's own `adirs.rs` (`fbw-common/src/wasm/systems/systems/src/navigation/adirs.rs`)
is not a naive stub. It already has:

- a **real, latitude-dependent alignment timer**
  (`total_alignment_duration_from_configuration`, adirs.rs:2350-2368): 300s at
  the equator, scaling as `300 / cos(latitude)` up to 60° (matching the
  physical reason gyrocompassing slows toward the poles -- Earth rate's
  horizontal component is `Omega_ie * cos(latitude)`), 600s from 60-73°,
  1020s beyond, plus an "excess motion during alignment restarts it" check
  (`update_remaining_align_duration`) and a max-latitude cutoff (82°). This
  formula, and the excess-motion/latitude logic around it, are **kept
  unmodified** -- exactly what the brief asks for ("from FBW's existing
  formula").
- a **full ARINC 429/SSM/fault-handling output stage**: `AirDataReference`
  and `InertialReference` compute altitude from static pressure with FBW's
  own barometric formula, gate every label's SSM on alignment/on/off state,
  and already have a static-pressure low-pass filter
  (`STATIC_PORT_TIME_CONSTANT`, an 8Hz filter modelling the pneumatic lag of
  the static line) and a vertical-speed filter. **Kept unmodified.**
- FBW's radio altimeter (`ala52b.rs`, `Ala52BTransceiverPair`) already has a
  **real antenna-geometry/reflection model**: transmitter/receiver height
  over ground from their installed offsets, the ground-reflection triangle
  geometry giving obliquity with pitch, a 44°/45° bank/pitch cutoff, and a
  travel-time-to-height conversion at the speed of radio waves. **Kept
  unmodified.**

What none of this had is a *sensor* between X-Plane's truth and that
computation:

- `AdirsSimulatorData` (adirs.rs:291-482) read `PLANE LATITUDE`,
  `PLANE PITCH DEGREES`, `PLANE BANK DEGREES`, body rotation rates,
  `AIRSPEED TRUE`, `AIRSPEED MACH`, `TOTAL AIR TEMPERATURE` and
  `INCIDENCE ALPHA` **directly off X-Plane's flight model, every tick,
  with zero error** -- and because this is a *single* struct shared by all
  3 ADIRUs, every one of them saw byte-identical truth.
- `AirDataReference::update_values` read `context.ambient_pressure()` and
  `context.indicated_airspeed()` -- again X-Plane truth -- for static
  pressure and CAS.
- `Ala52BTransceiverPair` read `PLANE ALT ABOVE GROUND`
  (`sim/flightmodel/position/y_agl`), X-Plane's own single, generally
  straight-down AGL value, not a boresight terrain probe (RA-001 in
  `docs/analysis/systems.md`).

## Integration: how the sensor sits in front of FBW's pipeline

Per-ADIRU sensor-realistic values are computed by the plugin
(`Adiru`/`AdirsPhysics` in `src/physics/adirs.rs`) and written to new
`A32NX_ADIRS_SENSED_<n>_<FIELD>` variables. FBW's `adirs.rs` gained one new
struct, `AdirsSensedData` (one instance per ADIRU, inside
`AirDataInertialReferenceUnit`), which reads those variables and a `valid`
flag. At the top of `AirDataInertialReferenceUnit::update`:

```rust
let simulator_data = self.sensed.apply_to(simulator_data);
```

`apply_to` overwrites `simulator_data`'s pitch/roll/true_heading/true_track/
body rotation rates/latitude/longitude/ground_speed/vertical_speed/
true_airspeed/mach/total_air_temperature/angle_of_attack fields with the
sensed ones **only when `valid` is true**; every existing test in adirs.rs
never sets `valid`, so `apply_to` is the identity function for them and all
244 existing tests in that file (and all 259 in `navigation::`) pass
unmodified. The corrected `simulator_data` copy then flows through every
existing, untouched line of `update_attitude_values`, `update_heading_values`,
`update_non_attitude_values` and `update_wind_velocity` -- no ARINC/SSM/fault
logic was touched. `AirDataReference::update_values` gets the same treatment
for the two fields that live outside `AdirsSimulatorData` (static pressure,
computed airspeed), via a small `sensed: AdirsSensedData` parameter.

For the radio altimeter, `Ala52BTransceiverPair` gained
`terrain_probe_alt_above_ground`/`terrain_probe_valid` fields read the same
additive way; `ground_clearance()` uses the terrain-probed value when valid,
otherwise `PLANE ALT ABOVE GROUND` exactly as before. All 9 `ala52b` tests
pass unmodified.

Field-by-field diff: `patches/fbw-rust/navigation-sensors.patch`.

## IRS: strapdown mechanization

Each of the 3 `Adiru`s (`src/physics/adirs.rs`) has its own gyro/accelerometer
error draw and its own integrated attitude/velocity/position state --
independent of the other two (see "Known limitation" below for what is still
shared).

### Sensors

- **Gyro** (`GyroAxis`): `sensed = true*(1+scale_factor) + bias + noise`.
  `bias` and `scale_factor` are drawn once per unit+axis at construction
  (Gaussian, session-stable). `noise` is angle random walk, discretized as
  `noise_rad_s = N(0,1) * ARW_rad_per_sqrt_s / sqrt(dt)` -- the standard way
  to keep a *fixed* angle-random-walk growth rate independent of the
  integration step (a random walk's variance grows linearly with elapsed
  time, so sampling more often needs a proportionally larger per-sample
  variance; see Woodman, *An introduction to inertial navigation*, Cambridge
  UCAM-CL-TR-696, section on gyro noise).
- **Accelerometer** (`AccelAxis`): identical structure, in g, with velocity
  random walk in place of angle random walk.

### Attitude

Sensed body rates integrate pitch/roll/true heading via the standard
aviation Euler-rate kinematics (Stevens & Lewis, *Aircraft Control and
Simulation*):

```
theta_dot = q*cos(phi) - r*sin(phi)
phi_dot   = p + (q*sin(phi) + r*cos(phi))*tan(theta)
psi_dot   = (q*sin(phi) + r*cos(phi)) / cos(theta)
```

using `p, q, r` = the sensed rate **relative to the local-level navigation
frame**, i.e. the sensed (inertial) rate minus Earth-rate-plus-transport-rate
expressed in body axes (`ned_to_body(...)`, Groves eq. 5.9). This is the term
that closes the Schuler loop: a position/velocity error changes the
transport rate, which changes the *computed* local vertical relative to the
attitude estimate, which reintroduces a corrective horizontal specific-force
component. Sign convention: `p = -P_xp, q = -Q_xp, r = +R_xp` and
`pitch = theta_xp, roll = phi_xp` match FBW's own already-tested
`update_attitude_values`/`update_non_attitude_values` exactly (adirs.rs),
which is how this module avoids re-deriving X-Plane's own axis convention
from scratch -- it reuses the relationship FBW's own 244 passing tests
already exercise.

### Velocity and position

Specific force is rotated body-to-NED with the standard DCM (Stevens &
Lewis, eq. 1.3-23) and integrated against gravity, Earth rate and transport
rate (Groves eq. 5.9-5.10 / Titterton & Weston, *Strapdown Inertial
Navigation Technology*):

```
v_n_dot = C_b^n * f_b - (2*Omega_ie_n + Omega_en_n) x v_n + g_n
lat_dot = v_north / (R+h)
lon_dot = v_east / ((R+h)*cos(lat))
h_dot   = -v_down
```

`g_n` is the 1980 International Gravity Formula (Geodetic Reference System
1980, "Somigliana equation") with the standard free-air correction
(`normal_gravity`): sea-level gravity as a function of latitude, minus
`3.086e-6 * altitude_m`. `R` is a single mean Earth radius (6,371,000 m); a
full WGS84 ellipsoid is the textbook-complete version, but a mean radius
changes the Schuler period by a fraction of a percent, immaterial next to the
sensor error budget.

**Numerical stability (empirically found while building this module, see
the report):** a *pure, undamped* implementation of the equations above is
only neutrally stable in continuous time (that is the entire point of
Schuler tuning), and forward-Euler-integrating a neutrally stable oscillator
is a textbook numerical-analysis trap: explicit Euler adds spurious energy
every step, so a discretized undamped Schuler loop diverges within roughly
one period regardless of step size. A stand-alone reproduction (outside
cargo, since the equations don't need the rest of the crate) confirmed this:
free-inertial dead reckoning at 45N grows without bound starting around the
first Schuler period. Real ADIRUs never run the pure undamped case either
(GPIRS aiding, or a deliberately damped "third order" mechanization); this
module always runs GPIRS-damped for the same reason -- see below. The
free-inertial (GPS-lost) failure case, with its own damping removed, is
flagged in the report as a follow-on needing a proper integrator (e.g. RK4,
or a Kalman filter's implicit damping) before it can be exposed as a
selectable failure mode.

### Alignment (gyrocompassing)

FBW's own alignment timer (unmodified) decides *when* an ADIRU is aligned.
This module reads that decision back (`A32NX_ADIRS_ADIRU_<n>_STATE`) and
shapes its own attitude/position accordingly:

- **Off**: mechanization re-seeds from truth every tick (so the next
  alignment starts fresh).
- **Aligning**: pitch/roll track truth (a real ADIRU's accelerometer-based
  leveling is fast and accurate, so this is a reasonable simplification);
  heading gyrocompasses in from an initial coarse error
  (`GYROCOMPASS_INITIAL_ERROR_DEG`, 6 degrees, a defensible derived starting
  value -- see the parameter table) with an exponential time constant
  `GYROCOMPASS_TAU_S / cos(latitude)`, mirroring the same physical
  `1/cos(latitude)` scaling FBW's own alignment-time formula uses, for the
  same reason (Earth rate's horizontal component shrinks toward the poles).
  Position is taken from truth (as if GPS-aided, matching a real GPIRS-aided
  align) and velocity is zero.
- **Aligned**: free-inertial dead reckoning begins from wherever the
  gyrocompass left off, with GPIRS blending active from the first tick.

### GPIRS

A simple complementary filter (`gpirs_correct`) blends the free-inertial
position **and velocity** toward GPS truth (Airbus/Honeywell public GPIRS
material describes it as "a combined position from its IRU mixed with the
GPS the ADIRU has selected" giving "faster align times and 100 percent
availability" -- Boeing/Airbus GNSS-loss references cited in the parameter
table): `x += (dt/tau)*(gps - x)`, `tau = 120s`. GPS itself is modelled as
X-Plane's true position plus 3m 1-sigma noise (a typical civil GPS
horizontal accuracy figure); there is no separate GPS receiver/multipath
model in scope. As noted above, the velocity term is also what keeps this
module's Schuler loop numerically well-behaved.

### Loss of alignment on power loss / ATT mode

Each ADIRU reads an assumed electrical bus (`AdirsPhysics::power_vars`; see
the parameter table -- the real A380's ADIRU-to-bus assignment is not
published anywhere in scope, so ADIRU 1 and 3 are assigned to the essential
bus and ADIRU 2 to a main DC bus, the usual Airbus pattern of keeping the
primary and backup unit essential-powered, marked as a derived assumption).
When unpowered, the sensed data's `valid` flag goes false (FBW's own
passthrough resumes, matching pre-existing behaviour) and the mechanization
re-seeds from truth, so realignment starts fresh once power returns -- this
module's model of "loss of alignment on power loss". **Known limitation**:
FBW's own `AirDataInertialReferenceSystemOverheadPanel`/mode-selector logic
has no electrical dependency at all (confirmed: no `Adiru`/`ADIRU` match
anywhere in `a380_systems`'s electrical crate), so the ADIRU push-button/
fault annunciation itself does not change during a power loss -- only this
module's sensed inputs do. Wiring FBW's own overhead panel to bus power is a
further, larger upstream change, out of scope here.

ATT mode reversion is not separately modelled: this module's `valid` flag is
gated on FBW's own `ADIRU_STATE` reaching "Aligned", which (per adirs.rs's
`update_remaining_align_duration`) never happens in ATT mode, so ATT-mode
attitude continues to fall back to the pre-existing truth passthrough --
matching, not regressing, the baseline.

## ADR: pitot-static air data

Computed every tick regardless of IR alignment (a real ADR's own ~18s
initialization timer, in FBW's `AirDataReference`, is independent of IR
gyrocompassing).

- **Total/static pressure**: `qc = Ps_true * ((1+0.2*M^2)^3.5 - 1)` (the
  standard subsonic compressible pitot-static relation), `Pt_true = Ps_true +
  qc`.
- **Static source position error**: `qc * 0.003 * clamp(alpha_deg - 3, -10,
  15)` -- a small, AoA-driven pressure error at the static ports. No
  A380-specific static-source-error-correction curve is published; this is a
  generic, transport-category-derived placeholder, clearly marked (see the
  parameter table), consistent in *shape* with a typical underwing-static
  characteristic (reads slightly high pressure / low altitude above the
  reference AoA).
- **CAS/Mach inversion** (standard ICAO/FAA air data computer formulae):
  `CAS = a0 * sqrt(5*((qc/P0 + 1)^(2/7) - 1))`,
  `M = sqrt(5*((qc/Ps + 1)^(2/7) - 1))`.
- **TAT** from the recovery factor: `Tt = Ts*(1 + r*(gamma-1)/2*M^2)`,
  `r = 0.99` (Rosemount-style total-temperature-probe recovery factors are
  commonly quoted 0.98-1.0; the A380-specific figure is not published).
- **AoA vane**: `alpha_sensed = alpha_true * 1.10` -- a generic
  fuselage-upwash amplification factor (not A380-specific).
- **Probe heat / icing**: probe heat is assumed commanded on whenever the
  (assumed) bus is powered (normal AUTO logic; a manual OFF selection is not
  yet modelled as a separate switch). In icing conditions (SAT<0C, airborne)
  with heat off, ice accretes at a clearly-marked derived rate
  (`PROBE_ICE_ACCRETION_KG_S`, sized to block an unheated probe within about
  a minute, consistent with why probe heat is mandatory equipment); once
  blocked, the reported pressure **freezes at its last value**, which is
  exactly the real "blocked pitot"/"blocked static" symptom (e.g. a frozen
  pitot in a climb makes CAS read increasingly high, because the frozen
  total pressure no longer falls with the now-lower static pressure) --
  this falls out of the pressure-freeze physics directly, nothing
  special-cased. Probe heater electrical load (350W/probe pitot, 80W TAT) is
  computed and exposed on the Study panel; it is **not** wired into the
  electrical workstream's load balance (no shared-contract variable exists
  for it, and doing so is that workstream's call).
- **Altitude**: still FBW's own, unmodified ISA-consistent
  `AirDataReference::calculate_altitude_from_static_pressure` -- this module
  only supplies a better *static pressure* input.

## Radio altimeter

`RadioAltimeterProbe` (`src/physics/adirs.rs`) probes X-Plane's terrain
(`XPLMProbeTerrainXYZ`, `src/xp.rs::probe_terrain_y`) at the aircraft's local
position, sampled every 8th tick (`PROBE_EVERY_TICKS`) to keep the scenery
raycast off most frames -- the same budget discipline `src/wxr` already
documents for X-Plane's weather API. All 3 RAs share this one probe (their
antennas are only metres apart on the same airframe; FBW's own geometry
already applies each antenna's *own* installation offset on top of this
shared ground clearance, matching how the original `PLANE ALT ABOVE GROUND`
reading was shared too). Pitch/roll obliquity and the valid range (44°/45°
bank/pitch cutoffs, and the roughly 10,200 ft maximum path length) are
unchanged, existing FBW physics (`Ala52BTransceiverPair::response`).

## Parameter sources

| Parameter | Value | Source |
|---|---|---|
| Gyro bias instability | 0.01 deg/hr | Commonly published navigation-grade RLG figure ("better than 0.01 deg/hr" is the stated navigation-grade objective); ARINC 704's own numeric table is a paywalled standard this brief could not fetch. Defensible derived value. |
| Gyro angle random walk | 0.002 deg/sqrt(hr) | Published figure for a comparable high-performance RLG. |
| Gyro scale factor error | 5 ppm | Typical RLG-class figure. |
| Accelerometer bias | 8 micro-g | Published range for strapdown navigation-grade accelerometers is 5-10 micro-g. |
| Accelerometer velocity random walk | 0.01 m/s/sqrt(hr) | Derived estimate scaled from the gyro-class noise density; not independently published. |
| Accelerometer scale factor error | 20 ppm | Typical RLG-IMU-class figure. |
| Earth rotation rate | 7.292115e-5 rad/s | WGS84 (standard geodetic constant). |
| Mean Earth radius | 6,371,000 m | WGS84 mean radius; simplification vs. the full ellipsoid (documented above). |
| Normal gravity formula | 1980 International Gravity Formula + free-air correction | Geodetic Reference System 1980 ("Somigliana equation"); Hofmann-Wellenhof & Moritz, *Physical Geodesy*. |
| Schuler period | ~84.4 min = 2*pi*sqrt(R/g) | Wikipedia, "Schuler tuning"; Max Schuler, 1923. |
| GPIRS blend time constant | 120 s | Not published; Airbus/Honeywell GPIRS material describes the blend qualitatively only. Defensible derived value. |
| GPS receiver noise | 3 m 1-sigma | Typical civil GPS horizontal accuracy figure. |
| Gyrocompass initial heading error | 6 degrees | Not published (no A380-specific coarse-align figure); chosen so the residual error after FBW's own 300s equatorial alignment timer is of the commonly-quoted order (a few tenths of a degree). Defensible derived value. |
| Gyrocompass time constant | 60 s (at the equator) | Derived so alignment converges within FBW's own existing timer; the `1/cos(latitude)` scaling itself is the same physical relationship FBW's alignment-time formula already uses. |
| TAT recovery factor | 0.99 | Commonly quoted range for Rosemount-style total temperature probes is 0.98-1.0; not A380-specific. |
| AoA vane upwash factor | 1.10 | Generic transport-aircraft planning figure; not A380-specific. |
| Static source position error coefficient | 0.003 * qc per degree of AoA | Not sourced from A380 data (no public SSEC curve); a small, generic, transport-derived placeholder. Clearly marked. |
| Probe ice accretion rate / block mass | 6e-5 kg/s, 2e-3 kg | Not published for this orifice size; sized so an unheated probe blocks within about a minute of continuous icing, consistent with why probe heat is mandatory equipment. Clearly marked as derived. |
| Probe heater load | 350 W (pitot) / 80 W (TAT) | Generic, commonly cited transport-category pitot/TAT heater wattages; not A380-specific. |
| ADIRU-to-bus assignment | ADIRU 1, 3 -> DC ESS; ADIRU 2 -> DC 2 | Not published for the A380; assigned by analogy to the general Airbus pattern of essential-bus-powering the primary and backup/standby unit. Clearly marked as an assumption. |
| ISA sea-level pressure/speed of sound | 101,325 Pa / 661.4788 kt | ICAO Standard Atmosphere (public). |
| Radio altimeter geometry, cutoffs, max range | Unmodified | FBW's own `ala52b.rs` (Ala52BTransceiverPair), not sourced or modified by this workstream. |

## Tests

`src/physics/adirs.rs`'s own `#[cfg(test)] mod tests`:

- `stationary_drift_over_one_hour_is_within_1_nm_per_hour` -- the required
  "stationary drift over 1h within spec" check, GPIRS-aided (this module's
  normal operating mode): asserts under 1 nm after 1 simulated hour
  stationary at 45N (measured well under budget, ~0.35-0.7 nm depending on
  the session's random bias draw).
- `mechanization_stays_bounded_and_finite_over_a_long_run` -- the brief's
  "no NaNs or explosions at any sim rate or pause": 3 hours stationary stays
  finite and bounded.
- `schuler_period_constant_matches_the_classical_84_4_minutes` -- verifies
  `2*pi*sqrt(R/g)` (the constants this module's mechanization is built on)
  is within 1 minute of 84.4 min. See "Numerical stability" above for why
  this tests the underlying relationship rather than a multi-cycle
  transient of the full, always-damped mechanization.
- `gyrocompass_alignment_takes_longer_near_the_poles` -- the alignment-time-
  vs-latitude requirement, on this module's gyrocompass time constant.
- `heading_error_decays_during_a_realistic_alignment` -- residual
  gyrocompass error after a realistic (300s, equatorial) alignment is under
  1 degree.
- `unheated_pitot_blocks_in_icing_conditions_and_freezes_its_reading` /
  `heated_probes_never_ice_up` -- the pitot blockage behaviour requirement.
- `static_pressure_at_zero_position_error_matches_isa_and_true_pressure` --
  the ISA altitude check (via the static pressure this module feeds FBW's
  own unmodified ISA altitude formula).
- `gyro_and_accelerometer_biases_differ_between_adirus` -- confirms the 3
  ADIRUs actually draw different sensor errors.

FBW-side: all 244 pre-existing `navigation::adirs` tests and all 9
`navigation::ala52b` tests pass unmodified (verified with
`cargo +stable-x86_64-pc-windows-gnu test --lib navigation::`, run directly
against `fbw-common/src/wasm/systems/systems`).

## Cost

Per tick, per ADIRU: a handful of trigonometric evaluations and 3x3
vector/matrix operations (no allocation) -- negligible next to the systems
tick itself. The radio altimeter's terrain probe (the only X-Plane-API-bound
cost) is throttled to 1-in-8 ticks.

## Study panel quantities

Per ADIRU (`A32NX_ADIRS_STUDY_<n>_*`, written every tick regardless of
`valid`, so the panel can show "Off"/"Aligning" states too):

- alignment state (mirrors FBW's own `A32NX_ADIRS_ADIRU_<n>_STATE`: 0 Off, 1
  Aligning, 2 Aligned) and, from FBW's own existing output, remaining
  alignment time;
- position error (nm) and a smoothed drift-rate estimate (nm/hr);
- this unit's fixed gyro bias (deg/hr) and accelerometer bias (micro-g);
- pitot/static blocked flags and probe heater load (W);
- raw sensed static and total pressure (Pa).

## What only X-Plane can verify

- Whether `XPLMProbeTerrainXYZ` behaves as documented against real scenery
  (ortho/photo scenery, sloped terrain, water) -- the terrain-probe code
  path cannot be exercised by a unit test.
- Whether the throttled (1-in-8-tick) radio altimeter sampling is smooth
  enough in practice during a real approach/flare, where the true AGL is
  changing quickly.
- Real-world "feel" of the sensed attitude/position/CAS during actual
  flight (turns, climbs, icing) -- the unit tests use stationary, level, or
  short synthetic scenarios only.
