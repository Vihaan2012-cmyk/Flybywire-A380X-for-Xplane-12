# Failures — deep/flight_controls

Format: `ATA | proposed name | model element it acts on | magnitude meaning (0..1) | effect`.

All entries below are ATA 27 (Flight Controls). Each is registered in code, individually
per physical surface/component, in `registry.rs` (`register()`); this file lists the
*distinct physical fault types* once per surface family rather than once per instance —
see `registry.rs`'s `every_a380_surface_family_is_represented_individually` test for the
full per-instance component list (37 components, ~360 individual failure ids).

## Servo-hydraulic/EHA/EBHA surfaces (ailerons x6, elevators x4, rudders x2, spoilers x16)

| ATA | proposed name | model element | magnitude (0..1) | effect |
|---|---|---|---|---|
| 27 | `<surface>` actuator jam | `actuator::PowerControlUnit` via `ActuatorFaults.jam` | 0 free .. 1 fully seized | authority falls toward zero and a resistive spring pins the surface near its jam angle, overridable only by an external torque exceeding the jam's own resistance |
| 27 | `<surface>` servo hardover (runaway) | `actuator::PowerControlUnit` via `ActuatorFaults.runaway`/`.runaway_sign` | 0 none .. 1 full-rate drive in `runaway_sign`'s direction regardless of command | surface drives toward one stop unless other actuators on the surface, or the aerodynamic hinge moment, can override it |
| 27 | `<surface>` hydraulic/electrical supply loss | `actuator::PowerControlUnit` via `ActuatorFaults.supply_loss` | 0 full supply .. 1 none | force ceiling falls linearly, rate ceiling as sqrt(supply) (orifice law) |
| 27 | `<surface>` position transducer fault (inner-loop) | `actuator::PowerControlUnit` via `ActuatorFaults.transducer_frozen`/`.transducer_bias_rad` | 0 healthy .. 1 feedback frozen at the angle seen when the fault began | the position loop chases a stale/wrong reading, driving the true surface off command while reading "on target" |
| 27 | `<surface>` servo valve internal leakage | `actuator::PowerControlUnit` via `ActuatorFaults.valve_leakage` | 0 healthy .. 1 fully worn | rate ceiling falls (bypassed flow does no useful work) and holding stiffness softens, causing creep/droop under sustained load |
| 27 | `<surface>` piston seal wear | `actuator::PowerControlUnit` via `ActuatorFaults.piston_seal_wear` | 0 healthy .. 1 fully worn | force ceiling falls (flow recirculates across the piston) and, like valve leakage, softens holding stiffness |
| 27 | `<surface>` disconnect | `surface::ControlSurface` via `SurfaceFaults.disconnected` | boolean in practice | zero actuator torque reaches the surface; it free-floats under aerodynamics + structural damping alone |
| 27 | `<surface>` flutter damper loss | `surface::ControlSurface` via `SurfaceFaults.flutter_damper_loss` | 0 healthy .. 1 dedicated damper failed | net hinge damping can go negative at high dynamic pressure (reduced-order flutter proxy) |
| 27 | `<surface>` position transducer drift (monitoring) | `sensors::PositionTransducer` via `TransducerFaults.drift` | 0 healthy .. 1 drifting at modelled max rate | the computer's own position monitoring (separate from the actuator's mechanical/inner-loop feedback) slowly diverges, eventually tripping a dual-channel disagreement |
| 27 | `<surface>` position transducer open circuit (monitoring) | `sensors::PositionTransducer` via `TransducerFaults.open_circuit` | 0 healthy .. 1 fully open | that channel's signal is lost (rails to zero); the computer relies on the other channel alone |
| 27 | `<surface>` position transducer intermittent (monitoring) | `sensors::PositionTransducer` via `TransducerFaults.intermittent` | 0 none .. 1 dropped essentially all the time | the channel cuts in and out; a monitor sees repeated brief losses rather than one clean failure |

`<surface>` ranges individually over: left/right outward/middle/inward aileron (x6),
left/right inboard/outboard elevator (x4), upper/lower rudder (x2, each with 2
EBHA-capable actuators), left/right spoiler 1-8 (x16, spoiler 6 each side is
EBHA-capable). 11 fault types x 28 surfaces = 308 individually-registered failures.

Note the two *distinct* position-feedback layers, both modelled: `ActuatorFaults.transducer_*`
is whatever feedback a PCU's own servo loop uses to close on itself (in a real
mechanical-follow-up hydraulic servo valve this needs no electronics at all); `sensors::PositionTransducer`
is the separate LVDT/RVDT electrical instrumentation a flight control computer reads for
monitoring/consolidation. Neither `a380_systems/src/hydraulic/mod.rs` nor `fbw-common`'s
`linear_actuator.rs` models the latter (confirmed by search: no LVDT/RVDT/transducer/
sensor-fault code exists in either), so `sensors.rs` is new physical modelling.

## Trimmable horizontal stabiliser (`ths.rs`)

| ATA | proposed name | model element | magnitude (0..1) | effect |
|---|---|---|---|---|
| 27 | THS green motor failure | `ths::TrimmableHorizontalStabilizer` via `ThsFaults.motor_green` (`ActuatorFaults.supply_loss`) | 0 healthy .. 1 no hydraulic drive from the green motor | trim rate roughly halves; yellow motor alone still trims and holds |
| 27 | THS yellow motor failure | `ths::TrimmableHorizontalStabilizer` via `ThsFaults.motor_yellow` | 0 healthy .. 1 no hydraulic drive from the yellow motor | trim rate roughly halves; green motor alone still trims and holds |
| 27 | THS no-back brake failure | `ths::TrimmableHorizontalStabilizer` via `ThsFaults.no_back_failure` | 0 healthy .. 1 no holding torque left | between trim inputs the stabiliser can be back-driven by its own hinge moment, drifting off commanded trim |
| 27 | THS ballscrew jam | `ths::TrimmableHorizontalStabilizer` via `ThsFaults.ballscrew_jam` | 0 free .. 1 fully seized | trim freezes at the jam angle regardless of motor command |
| 27 | THS position transducer drift | `sensors::PositionTransducer` feeding the THS | 0 healthy .. 1 drifting at modelled max rate | trim position monitoring diverges from truth -- runaway detection depends on trusting this |
| 27 | THS position transducer open circuit | `sensors::PositionTransducer` feeding the THS | 0 healthy .. 1 fully open | that channel's signal is lost; computer relies on the other channel |

## Rudder trim actuator (`ths.rs`)

| ATA | proposed name | model element | magnitude (0..1) | effect |
|---|---|---|---|---|
| 27 | Rudder trim motor failure | `ths::RudderTrimActuator` via `ElectricPumpFaults.motor_failure` | 0 healthy .. 1 dead | rudder trim can no longer be commanded; holds its last value |
| 27 | Rudder trim jam | `ths::RudderTrimActuator` via `ActuatorFaults.jam` | 0 free .. 1 seized | rudder trim freezes at its jammed value |

## High-lift drive lines: flap, outboard slat, inboard droop nose — each L/R (`high_lift.rs`)

The A380 uses a leading-edge "droop nose" (a single-hinge rotating panel, no track or
slot) inboard and conventional slotted slats outboard — a public Airbus/A380 design
choice not modelled anywhere in `a380_systems` (that source only has one A320-family-style
slat design everywhere). `high_lift::HighLiftSystem::{new_flap, new_slat, new_droop_nose}`
give each its own GENERIC drive sizing (slat: lower torque, faster; droop nose: higher
torque, slower, heavier — see `high_lift.rs`'s doc comments for the reasoning), all sharing
the same PCU -> limiter -> inboard station -> outboard station -> brake architecture and
therefore the same fault list:

| ATA | proposed name | model element | magnitude (0..1) | effect |
|---|---|---|---|---|
| 27 | `<system>` PCU jam | `high_lift::HighLiftSystem` via `HighLiftFaults.pcu` (`ActuatorFaults.jam`) | 0 free .. 1 seized | drive line freezes at the PCU's own shaft angle |
| 27 | `<system>` PCU hardover | `high_lift::HighLiftSystem` via `HighLiftFaults.pcu` (`ActuatorFaults.runaway`) | 0 none .. 1 full-rate uncommanded drive | uncommanded motion until the wingtip brake, a shaft break or the limiter's own authority stops it |
| 27 | `<system>` PCU supply loss | `high_lift::HighLiftSystem` via `HighLiftFaults.pcu` (`ActuatorFaults.supply_loss`) | 0 full .. 1 none | drive torque and rate both fall toward zero |
| 27 | `<system>` torque limiter failure | `high_lift::HighLiftSystem` via `HighLiftFaults.limiter_bypass` | 0 trips at design threshold .. 1 seized/bypassed, never trips | a downstream jam/overload transmits full PCU torque straight into the shaft the limiter exists to protect |
| 27 | `<system>` inboard shaft break | `high_lift::HighLiftSystem` via `HighLiftFaults.inboard_shaft_break` | 0 intact .. 1 severed | both inboard and outboard stations lose drive, free-floating under airload |
| 27 | `<system>` outboard shaft break | `high_lift::HighLiftSystem` via `HighLiftFaults.outboard_shaft_break` | 0 intact .. 1 severed | only the outboard station loses drive; inboard still tracks |
| 27 | `<system>` wingtip brake failure | `high_lift::HighLiftSystem` via `HighLiftFaults.wingtip_brake_fail` | 0 healthy .. 1 no holding torque | an asymmetry or overspeed condition can no longer be contained mechanically |

`<system>` ranges over left/right flap, left/right outboard slat, left/right inboard droop
nose (x6). 7 fault types x 6 systems = 42 individually-registered failures.
Asymmetry/skew detection (`surface::AsymmetryMonitor`, reused by `high_lift::HighLiftPair`)
and uncommanded-motion detection (`high_lift::uncommanded_motion`) are protective/
diagnostic logic, not faults — they observe the emergent state the faults above produce;
no separate failure entry is warranted for them per BRIEF's "one line per genuinely
distinct physical fault".

## Ground spoiler deploy/retract logic (`spoiler.rs`, shared, not per panel)

| ATA | proposed name | model element | magnitude (0..1) | effect |
|---|---|---|---|---|
| 27 | Ground spoiler logic fails to deploy | `spoiler::GroundSpoilerLogic` via `GroundSpoilerLogicFaults.fails_to_deploy` | 0 healthy .. 1 never deploys despite armed + touchdown/spin-up | loss of lift dump and reduced wheel braking effectiveness on landing |
| 27 | Ground spoiler logic fails to retract | `spoiler::GroundSpoilerLogic` via `GroundSpoilerLogicFaults.fails_to_retract` | 0 healthy .. 1 stuck deployed through a go-around | reduced lift and increased drag exactly when climb performance matters most |

This is the arming/sequencing logic itself (touchdown/spin-up detection, lever-armed
interlock, go-around retraction), not any one spoiler actuator — those already have their
own faults above via the generic servo-hydraulic surface list. `spoiler.rs` also provides
`max_sustainable_deflection_rad` (the roll-spoiler/aileron blowdown limit: the largest
deflection an actuator can hold against its own hinge moment at a given flight condition,
solved in closed form and cross-checked in `spoiler.rs`'s own tests against
`surface::ControlSurface`'s emergent equilibrium) and `load_alleviation_authority` (a
GENERIC gust/manoeuvre load-alleviation schedule). Neither is a fault source — they are
control-law-facing utility functions describing normal, healthy behaviour — so neither gets
a failure entry.

## Actuator-to-computer allocation (`allocation.rs`) — logic, not a new failure source

`allocation.rs` reconstructs, from this port's own `prim.rs::update_servo_solenoid_status`
(lines 1660-1740, the one place the real PRIM/SEC-to-actuator wiring is visible — neither
`a380_systems/src/hydraulic/mod.rs` nor the compiled PrimComputer/SecComputer expose it),
which computer(s) and which hydraulic/electric supply drive each actuator, and picks the
one `Active` actuator per surface as computers/supplies fail. It *consumes* PRIM/SEC health
(`ComputerHealth`) rather than producing a new failure: those computers' own failures are
already tracked by the crate's existing `failures.rs` (`FAILURE_PRIM`/`FAILURE_SEC` ids,
below the 100,000 threshold `api.rs` reserves for the pre-existing catalogue), not by this
area's registry. No new `registry.rs` entries are added for it.

## Totals

Distinct failure *types*: 11 (surface family) + 6 (THS, incl. 2 transducer) + 2 (rudder
trim) + 7 (high-lift) + 2 (ground spoiler logic) = 28, individually instantiated
308 + 6 + 2 + 42 + 2 = 360 times across every real A380 surface/system this area models.
See `registry.rs` for the authoritative, machine-checked (`Registry::validate()`) version
of this list, and its own tests for the full per-instance component roster (37 components).
