# deep/oxygen — failures

`ATA | name | model element | magnitude (0..1) | effect`

All ids are `deep::api::failure_id(Area::Oxygen, 35, n)`, i.e. `20_035_00n`; the named
constants are in `registry::ids`.

## 35-10 Crew oxygen

| ATA | name | model element | magnitude | effect |
|---|---|---|---|---|
| 35 | Crew oxygen cylinder leak | `cylinder::CylinderFaults.leak` on the crew group | 0..1 of a 1 mm² equivalent orifice | choked leak to the flight deck; indicated and temperature-corrected bottle pressure both fall, low-pressure caution comes up, group empty in ~20 min at full magnitude |
| 35 | Crew oxygen overpressure disc degraded | `cylinder::CylinderFaults.disc_weakened` | 0..1 reduction of the 2775 psig rupture pressure | above ~0.34 the disc ruptures at or below full charge and dumps the group overboard in under a minute; below that it still relieves early if the bay heats the bottle |
| 35 | Crew oxygen supply valve seized | `crew::CrewOxygenFaults.valve_jam` | 0..1 of travel lost | at full magnitude the distribution falls to zero and the mask regulators stop delivering, with the bottle still full — invisible on the quantity gauge |
| 35 | Crew oxygen reducer setting low | `regulator::RegulatorFaults.setpoint_shift` (negative) | 0..1 of setpoint lost | distribution pressure falls; below half setpoint the demand regulators cannot open at all |
| 35 | Crew oxygen reducer setting high | `regulator::RegulatorFaults.setpoint_shift` (positive) | 0..1 added to setpoint | low-pressure relief lifts at 1.5× setpoint and holds it there |
| 35 | Crew oxygen reducer seat leak | `regulator::RegulatorFaults.seat_leak` | 0..1 of a 0.01 mm² orifice past the poppet | cylinder bleeds into the low-pressure side and out through its relief with nobody breathing it; a full group empties overnight |
| 35 | Crew oxygen distribution / mask hose leak | `crew::CrewOxygenFaults.distribution_leak` | 0..1 of a 0.5 mm² orifice at 85 psi | leaks whether or not a mask is donned; drains the cylinder in ~5 h at full magnitude |
| 35 | Crew mask *n* diluter jammed to ambient (×4, one per station) | `crew::CrewOxygenFaults.dilution_stuck_ambient[n]` | 0..1 of diluter travel toward the air inlet | that station's delivered oxygen fraction falls toward 0.2095 — the mask breathes normally and contains cabin air; the bottle stops being drawn, so neither the mask nor the gauge announces it |

## 35-20 Passenger oxygen (chemical generators)

| ATA | name | model element | magnitude | effect |
|---|---|---|---|---|
| 35 | Generator initiators dud | `pax::PassengerOxygenFaults.dud_initiators` | 0..1 of the units | masks still present; that fraction delivers nothing when pulled; cabin oxygen flow and generator heat fall in proportion; the unfired generators are still full |
| 35 | Candle quenches early | `pax::PassengerOxygenFaults.candle_quench` | 0..1 of the candle the front fails to reach | burn stops early — at 0.5 the cabin supply runs out after ~7½ min instead of 15 — leaving unburnt candle behind |
| 35 | Generator inadvertent ignition | `pax::PassengerOxygenFaults.inadvertent_ignition` | 0..1 of the units | those units burn inside a closed PSU with no masks presented, reaching ~300 °C case temperature and putting their full chemical heat into the cabin zone; the supply they represent is spent and unrecoverable in flight |
| 35 | PSU oxygen latches seized | `pax::PassengerOxygenFaults.latch_failed` | 0..1 of the doors | that fraction of the cabin gets no mask at all and no generator in it is ever lit; presented fraction, oxygen flow and heat all fall together |
| 35 | Automatic deployment controller failed | `pax::PassengerOxygenFaults.auto_deploy_controller` | 0..1 of deployment authority | at 1.0 the cabin-altitude trigger never presents the masks however high the cabin goes; the manual command is a separate path and still works |

## 35-30 First-aid (therapeutic) oxygen

| ATA | name | model element | magnitude | effect |
|---|---|---|---|---|
| 35 | First-aid cylinder leak | `cylinder::CylinderFaults.leak` on the therapeutic cylinder | 0..1 of a 1 mm² orifice | indicated first-aid pressure falls; below half the 50 psi delivery setting the outlets stop entirely |
| 35 | First-aid overpressure disc degraded | `cylinder::CylinderFaults.disc_weakened` (therapeutic) | 0..1 reduction of the 2700 psig rupture pressure | past ~⅓ the disc ruptures at charge pressure and dumps the cylinder in well under a minute |
| 35 | First-aid regulator setting low | `regulator::RegulatorFaults.setpoint_shift` (negative, therapeutic) | 0..1 of the 50 psi setting lost | below half setting the continuous-flow outlets stop delivering, with the bottle still full |
| 35 | First-aid regulator seat leak | `regulator::RegulatorFaults.seat_leak` (therapeutic) | 0..1 of a 0.01 mm² orifice | cylinder bleeds down over hours with nobody using it; found short at the next check |
| 35 | First-aid outlet stuck open | `therapeutic::TherapeuticFaults.outlets_stuck_open` | 0..1 of the installed outlets | a continuous-flow outlet does not care whether anyone is breathing through it; each stuck outlet drains the cylinder at its full 4 L/min |

## ECAM alerts raised

| key | title | level | raised by |
|---|---|---|---|
| `OXY_CKPT_SYS_LO_PR` | OXYGEN CKPT SYS LO PR | Caution | crew cylinder leak, disc rupture, reducer seat leak, distribution leak |
| `OXY_CREW_SUPPLY_LO_PR` | OXYGEN CREW SUPPLY LO PR *(GENERIC wording)* | Caution | supply valve seized, reducer set low |
| `OXY_PAX_SYS_ON` | OXYGEN PAX SYS ON | Caution | generator inadvertent ignition (and normal deployment) |
