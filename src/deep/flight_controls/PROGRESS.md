# Progress — deep/flight_controls

Backlog items are BRIEF.md's 1-6 in order. Each line: `- [done] item — files — notes`.
Second-layer backlog (from the lead, "keep going" message) is listed after item 6.

- [done] 1. Per-surface actuator model (servo valve, piston area, rate limit, stall
  force, active/damping/standby modes, mode switching, EHA/EBHA) — `actuator.rs` —
  `PowerControlUnit` is a piston-domain servo (force = pressure*area, rate =
  flow/area via the orifice sqrt(pressure) law), geometry cited from FlyByWire's
  A380 aileron/spoiler/elevator/rudder actuator constructions
  (a380_systems/src/hydraulic/mod.rs) where its comments give real bore/flow
  numbers, GENERIC crank arms elsewhere (spoiler's and aileron's arms are
  derived from FBW's own body geometry vectors, so those two are also cited,
  not GENERIC). `ActuatorMode::{Active,Damping,Standby}` matches FlyByWire's
  own `LinearActuatorMode` (PositionControl/ActiveDamping/ClosedValves).
  `ElectricMotorPump` models an EHA/EBHA's own motor-pump with a first-order
  spin-up. Tests cover geometry sanity against FBW's cited numbers, mode
  behaviour, supply loss, jam, runaway and a frozen transducer.
- [done] 2. Surface aerodynamic hinge moments + blow-back — `hinge_moment.rs`,
  `surface.rs` — `hinge_moment_nm` = q*S*c̄*Ch(delta,alpha) with a
  Prandtl-Glauert/transonic-falloff compressibility factor; GENERIC
  coefficients (order of NACA TR-868's plain-flap Ch_delta) scaled to
  GENERIC per-surface areas/chords. `ControlSurface<N>` sums N actuators'
  torque against the hinge moment and its own structural/flutter damping in
  a rigid-body integrator (semi-implicit Euler, internally sub-stepped at
  0.5 ms for the stiff standby/jam springs in `actuator.rs` — see
  `ControlSurface::MAX_SUBSTEP_S`'s doc comment for the stability
  derivation). Blow-back is emergent: when the actuators saturate and the
  angle drifts off command, `SurfaceOutput::blown_back` is set from that
  observation, not a scripted rule.
- [done] 3. Faults: actuator jam, runaway (servo hardover), loss of hydraulic
  supply, position transducer failure/drift, surface disconnect/free-float,
  flutter damper loss — `actuator.rs` (`ActuatorFaults`), `surface.rs`
  (`SurfaceFaults`) — jam is a spring-pin at the seize angle whose authority
  scales with the jam fraction (partial jams fight the servo rather than
  simply capping it); runaway blends the servo's own rate command toward a
  full-rate hardover; a frozen transducer feeds a stale reading into the
  position loop, which can drive the surface away from its true commanded
  position while its own loop reads "on target"; disconnect removes 100% of
  actuator torque so the surface free-floats under aerodynamics alone;
  flutter damper loss is a reduced-order negative-damping term that can flip
  net damping negative at high dynamic pressure (a proxy for real flutter
  analysis, which needs unsteady/CFD data this crate doesn't have — noted as
  GENERIC in `surface.rs`'s doc comment).
- [done] 4. High-lift: flap/slat PCUs, transmission shafts, torque limiters,
  wingtip brakes, asymmetry/skew detection, uncommanded motion — all with
  faults — `high_lift.rs` — the PCU reuses `actuator::PowerControlUnit` via
  `synthetic_rotary_geometry` (a hydraulic rotary motor obeys the same two
  laws as a linear ram with the crank arm fixed at 1), driving a 3-mass
  torque-tube chain (PCU shaft -> inboard station -> outboard station)
  through torsional spring/damper `TransmissionShaft` segments and a
  `TorqueLimiter`. `WingTipBrake` locks the outboard station at the angle it
  was commanded to; `AsymmetryMonitor` (from `surface.rs`, reused) debounces
  a left/right position gap before commanding it, via `HighLiftPair`.
  `uncommanded_motion` is a free function on rate vs. command, usable by any
  caller. Faults: PCU jam/runaway/supply loss (via `ActuatorFaults`),
  torque-limiter bypass (fails to trip, so a downstream jam/overload
  transmits full torque into the shaft), inboard/outboard shaft break, and
  wingtip brake failure.
- [done] 5. THS: dual motors, ballscrew, no-back brake, jam; rudder trim —
  `ths.rs` — `TrimmableHorizontalStabilizer` sums two
  `actuator::PowerControlUnit`s (green/yellow, matching FlyByWire's own
  `hydraulic_motors: [HydraulicDriveMotor; 2]`,
  trimmable_horizontal_stabilizer.rs:684) against the same `hinge_moment.rs`
  aerodynamics as every other surface; the no-back brake is a spring/damper
  that locks the screw at whatever angle it was at the instant *neither*
  motor's `ActuatorMode` is `Active` (idling between trim inputs), and
  disengages the instant either motor is commanded `Active` again — this is
  why `TrimmableHorizontalStabilizer::step` takes the motors' modes
  explicitly rather than hardcoding `Active`, matching how a real jackscrew
  is not run as a continuous position servo between commands. Ballscrew jam
  reuses the same spring-pin pattern as `actuator.rs`'s own jam.
  `RudderTrimActuator` is a small reused `PowerControlUnit` + `ElectricMotorPump`
  pair (no aerodynamic hinge moment modelled: it works against the SEC's own
  mechanism, not directly against rudder airload). Tests include a no-back
  brake A/B comparison under identical aero load with/without the fault, per
  BRIEF's failure-changes-outcome requirement.
- [done] 6. Outputs as surface positions and forces a flight model can consume —
  `output.rs` — `FlightControlOutputs` gathers every surface's
  `SurfaceState` (angle/rate in both radians internally and the degrees
  `flight_controls.rs` already consumes, plus the hinge torque a
  structural/force-feedback model would want) in the same `[side][...]`
  shape as `flight_controls::Actuators`, so a future wiring pass can fill it
  directly. Pure data + conversions; no dependency beyond the field types.

## Second-layer backlog (from the lead's "keep going" message)

- [done] Droop-nose slats and slats as distinct high-lift systems, per-section —
  `high_lift.rs` — refactored `HighLiftSystem::new_generic` into a shared
  parameterised `new_with(...)`, kept `new_generic`/`new_flap` as-is, and added
  `new_slat` (lower torque, faster, lighter stations/shaft/brake — outboard
  slotted slats on tracks) and `new_droop_nose` (higher torque, slower,
  heavier — the A380's inboard single-hinge rotating leading edge, publicly
  documented as an A380-specific design choice; not modelled anywhere in
  `a380_systems`, which only has one A320-family slat design). `registry.rs`
  now registers 6 high-lift systems (flap/slat/droop-nose x L/R) instead of
  4, each individually, with its own model path in the failure list.
- [done] Per-actuator position transducers (LVDT/RVDT) with drift/open/
  intermittent failure modes feeding PRIM/SEC monitoring, and the actuator-
  to-computer allocation — `sensors.rs`, `allocation.rs`. Researched first
  (background agent, no files touched): confirmed neither
  `a380_systems/src/hydraulic/mod.rs` nor `fbw-common`'s `linear_actuator.rs`
  model transducers at all (no LVDT/RVDT/transducer/sensor-fault code
  anywhere), and that `a380_systems/src/hydraulic/mod.rs` itself carries no
  PRIM/SEC-to-actuator allocation table — it only consumes
  `*_SOLENOID_ENERGIZED` booleans. The real allocation table lives in this
  port's own `prim.rs::update_servo_solenoid_status` (1660-1740), which
  `allocation.rs` transcribes directly (aileron/elevator/rudder/THS
  actuator-to-PRIM/SEC/hydraulic-circuit tables, including the real
  asymmetry that the aileron outboard panel's two actuators have *no* SEC
  backup, unlike every other aileron actuator) and reconstructs a GENERIC
  priority-list arbitration for which actuator goes `Active` (the real
  arbitration is inside FBW's compiled PrimComputer/SecComputer, invisible
  to this port; the one visible clue, `a380_systems`'s own `filter_dual_control`,
  confirms only the shape: never two actuators active at once).
  `sensors.rs`'s `PositionTransducer`/`DualTransducer` model the *separate*
  electrical LVDT/RVDT layer a computer reads for monitoring (distinct from
  `actuator::ActuatorFaults.transducer_frozen`, which is whatever feedback a
  PCU's own servo loop closes on internally); `DualTransducer` reuses
  `surface::AsymmetryMonitor`'s debounce logic to flag a monitoring fault
  from two channels disagreeing.
- [done] Servo-valve internal leakage and piston seal wear as continuous
  degradations (slower rates, drooping under load) — `actuator.rs` — added
  `ActuatorFaults.valve_leakage`/`.piston_seal_wear`; both derate the force
  and rate ceilings (leakage costs rate more, seal wear costs force more,
  since leaked/bypassed flow either never reaches the piston or recirculates
  across it) and, more importantly, derate every mode's own loop gain via a
  `stiffness_factor`, so a degraded actuator's standby/holding stiffness
  softens and it visibly creeps/droops under sustained external load rather
  than only losing its instantaneous force ceiling.
- [done] Spoiler load alleviation / roll-spoiler blowdown limits; ground
  spoiler deployment specifics — `spoiler.rs` — `max_sustainable_deflection_rad`
  solves the actuator-vs-hinge-moment equilibrium in closed form (accounting
  for `hinge_moment.rs`'s own `ch_max` saturation, without which the formula
  gives nonsense whenever the actuator is strong enough to hold full travel
  outright — caught by cross-checking it against `surface::ControlSurface`'s
  emergent equilibrium in `spoiler.rs`'s own test); `load_alleviation_authority`
  is a GENERIC gust/manoeuvre schedule. `GroundSpoilerLogic` is a small
  latching state machine (armed -> touchdown/wheel-spin-up -> deployed ->
  go-around/disarm -> retracted) with its own two logic failures
  (fails-to-deploy, fails-to-retract) — confirmed via the same research pass
  that `a380_systems`'s own ground-spoiler detection is an explicit
  placeholder (`SpoilerGroup::ground_spoilers_are_requested`, mod.rs:6992-7002,
  its own comment reading "TODO use actual signal from flight controls", no
  weight-on-wheels/lever/reverser interlock modelled there at all), so this
  is new modelling, not a port.

## State at hand-off (hard stop 23:29)

Everything above is complete: code + tests + doc comments, `mod.rs` declares
all 10 submodules (matches every `.rs` file on disk), `registry.rs` registers
all of it (37 components, ~360 individual failures, 12 ECAM alerts),
`Registry::validate()` is asserted clean by `registry.rs`'s own test. Nothing
is half-written; brace-balance-checked across every file as a final sanity
pass. Not yet done (next, if resumed): (1) per-actuator LVDT/RVDT transducers
for the high-lift and rudder-trim components (only the 6 primary
servo-hydraulic surface families and the THS got them, for time); (2)
wiring `allocation.rs`'s `ComputerHealth`/`PowerAvailability` to this port's
actual `prim.rs` failure-consumer/hydraulic-circuit outputs (currently
free-standing, tested only against synthetic health inputs); (3) wiring
`output.rs`/`spoiler.rs`'s ground-spoiler command into `flight_controls.rs`'s
actual surface outputs; (4) a per-ATA-chapter review of whether any of the
28 high-lift/ground-spoiler failures should also raise a STATUS-page/INOP
entry beyond what's already in `registry.rs`.

## New variables this area will need to publish once wired in

Per the "Registering failures, components and ECAM alerts" update to
BRIEF.md: this directory is still self-contained (no variable registry
access), so `registry.rs`'s ECAM triggers reference one planned aggregate
variable per component, `FCTL_<COMPONENT>_FAULT` (0..1, the worst of that
component's active fault magnitudes), e.g. `FCTL_AIL_L1_FAULT`,
`FCTL_THS_FAULT`, `FCTL_FLAP_L_FAULT`. The exact per-field breakdown each
aggregate should be the max of is in `registry.rs`'s `ComponentDef.params`
for that component. Whoever wires fault injection into this area should
either publish these aggregates directly or have the ECAM layer compute them
from the individual fault fields.

## Notes on numerical stability (for whoever extends this further)

`actuator::PowerControlUnit`'s `Standby`/jam springs and `high_lift`'s
wingtip-brake spring are deliberately very stiff (a trapped-fluid/seizure
model should be stiff). Both `surface::ControlSurface::step` and
`high_lift::HighLiftSystem::step` (and `ths::TrimmableHorizontalStabilizer::step`)
sub-step internally at a fixed 0.5 ms regardless of the caller's own tick,
per the crate convention "sub-stepping where stiff" — semi-implicit Euler on
an undamped harmonic oscillator is stable exactly while `omega*dt <= 2`, and
0.5 ms has comfortable margin for every constant used here. If a future
change increases any spring/inertia ratio meaningfully, re-check that bound
before assuming a naive per-tick integration is safe.

- [done] Numerical: the PCU's inner rate loop is a damper of `c/I` = 3e3..4e7
  s^-1, so every caller's semi-implicit Euler step (`c*dt/I` of 3 to 320 at
  the 0.5 ms sub-step, 320 for the un-sub-stepped rudder trim) was
  unconditionally unstable and chattered between the torque clamps instead of
  tracking. Worse, a rate-limited servo's *linear band* is only `2*Tmax/c`
  wide (0.0075 rad/s for the THS) — narrower than one sub-step's rate change
  at the torque ceiling — so no affordable sub-step fixes it either.
  `actuator.rs` now publishes each mode's torque law as a `ServoLoad`
  (`clamp(open - c*rate, +-max)`, which all three modes are) and
  `servo_rate_step` solves the step exactly for the rate, clamp included:
  backward Euler in every velocity-proportional term, a two-branch case split
  on the clamp. Callers updated: `surface.rs`, `high_lift.rs` (three bodies),
  `ths.rs` (both the THS and the rudder trim). Equilibria are unchanged; a
  *negative* net damping (`SurfaceDamping`'s flutter drive) stays explicit on
  purpose, since divergence there is physics.
  Files: actuator.rs, surface.rs, high_lift.rs, ths.rs.
- [done] THS: the two motors drive the screw through a speed-summing
  differential, not in parallel — which is why losing one hydraulic system
  halves the Airbus trim *rate* while the torque capability survives (the
  dead motor is braked and reacts torque). Modelled as a gear ratio
  `2 / (motors with supply)` reflected through `ServoLoad::geared`. Also
  fixed: an actuator with no torque ceiling left now reports
  `ServoLoad::NONE`, so it gets no vote in a lumped multi-actuator law
  (before, a dead motor's rate-loop gradient still dragged the aggregate,
  quartering the screw rate instead of halving it). Files: ths.rs, actuator.rs.
- [done] live system — src/deep/flight_controls/live.rs, mod.rs — `FlightControlsLive` owns all 37 registered components (6 ailerons, 4 elevators, 2 rudders as ControlSurface<2>; 16 spoilers as ControlSurface<1>; THS; rudder trim; flap/slat/droop-nose HighLiftPairs = 8 stations; ground-spoiler logic) plus one DualTransducer per surface, and implements `deep::live::Area`. Truth -> q (ideal gas from real static p/T and TAS), Mach, per-actuator supply fraction from hydraulic_pressure_pa via allocation.rs, EHA/computer availability from ac_bus_volts. All 360 registered failures bound, ids read out of registry.rs itself. Publishes the 37 FCTL_<COMPONENT>_FAULT trigger variables plus deflection/blow-back/monitor/wingtip-brake study variables. `surface_angles()` fills integration::flight_control_surfaces::PhysicalSurfaces. Needs on Truth: commanded surface positions, alpha, ground-spoiler lever/go-around discretes — setters provided. 13 tests.
