# Gear structure workstream — failure catalogue

One line per genuinely distinct physical fault type this area's models
support (each fraction `0.0` = healthy .. `1.0` = fully failed). Every one
of these is expanded per instance (per leg / per wheel / per steering axle)
in code in `registry.rs`'s `register()` — this table lists the physical
fault *type*, not each of its ~74 individual instances.

| ATA | Proposed name | Model element it acts on | Magnitude meaning (0..1) | Effect |
|---|---|---|---|---|
| 32 | Shock strut nitrogen precharge leak | `strut::Strut::step`'s `StrutFaults.gas_leak`, drains `gas_charge_fraction` | fraction of the full-leak rate (magnitude 1.0 empties the precharge over ~24 h) | The gas spring law is scaled by `gas_charge_fraction`, so the leg settles at a higher static compression for the same load (`equilibrium_y`), eating into stroke margin and making the next landing more likely to overload or bottom out. |
| 32 | Shock strut hydraulic oil leak | `strut::Strut::step`'s `StrutFaults.oil_leak`, drains `oil_level_fraction` | fraction of the full-leak rate | Orifice damping force is scaled by `oil_level_fraction`; a depleted strut absorbs far less of a touchdown's kinetic energy, raising the peak reaction force for a given sink speed. |
| 32 | Gear extend/retract actuator internal leak | `retraction::Retraction::step`'s `RetractionFaults.actuator_leak` (gear travel rate) | fraction of nominal actuator speed/force lost | The leg extends/retracts proportionally slower; at 1.0 it does not move at all even with hydraulic pressure available. |
| 32 | Uplock hook jam | `retraction::Retraction::step`'s `RetractionFaults.uplock_jam` (release gate) | `>=0.5` defeats the normal hydraulic uplock release; `>=0.95` also defeats gravity extension's separate mechanical/pneumatic release | The leg cannot leave the up-locked state on a gear-down selection. A moderate jam is still overcome by gravity extension (the reason that system exists); a severe jam is not, and the leg will not extend by any means modelled. |
| 32 | Downlock spring/linkage failure | `retraction::Retraction::step`'s `RetractionFaults.downlock_fail` | `>=0.5` the downlock fails to fully seat | The leg reaches the geometric down position (door sequence and position sensing proceed normally) but `downlocked` stays false; a real touchdown load then folds the leg via `strut.rs`'s unlocked-collapse path — the classic "gear down but not locked" hazard. |
| 32 | Gear door actuator/track jam | `retraction::Retraction::step`'s `RetractionFaults.door_jam` (door travel rate) | `>=0.5` the door seizes completely; below that, proportionally slowed | The door sequence stalls, which — through this model's door/gear travel interlock — blocks the gear from ever starting to move at all. |
| 32 | Gear lock proximity sensor failure | `retraction::Retraction::step`'s `RetractionFaults.sensor_lies` (`sensed_uplocked`/`sensed_downlocked`) | `>=0.5` the sensed indication is the opposite of the true lock state | The cockpit indication disagrees with the leg's true mechanical state, independent of it — a crew reading only the sensed value cannot tell the difference. |
| 32 | Wheel antiskid channel failure | `brakes::BrakeWheel::step`'s `BrakeFaults.antiskid_inop` | `1.0` = no skid protection on this wheel's channel | Commanded brake pressure is no longer released during a skid (`slip > SKID_SLIP_THRESHOLD`), so the wheel locks and drags at aircraft speed instead of rolling — hotter, faster wear, and (if coordinated with `physics::tyre.rs`) tyre damage. |
| 32 | Dragging brake | `brakes::BrakeWheel::step`'s `BrakeFaults.dragging` | added uncommanded brake-force fraction that never releases | The wheel's carbon stack heats and wears with zero pedal/autobrake command, and can reach the fire threshold on a long taxi/flight with no braking ever commanded. |
| 32 | Parking brake accumulator leak | `brakes::ParkingBrakeAccumulator::step`'s `ParkingBrakeFaults.leak` | fraction of the full-leak (few-hour bleed-down) rate | The accumulator's stored (Boyle's-law) pressure bleeds down while the parking brake is set, eventually falling below the minimum holding pressure — the parking brake silently stops holding. |
| 32 | Nosewheel/body-gear steering shimmy damper failure | `steering::SteeringActuator::step`'s `SteeringFaults.shimmy_damper_fail` | mechanical damping coefficient reduced toward its residual structural minimum | The torsional shimmy mode's critical (unstable) groundspeed falls (`critical_speed_ms`); above it, a self-excited oscillation grows instead of damping out — real wheel shimmy, not a scripted vibration. |
| 32 | Steering actuator internal leak | `steering::SteeringActuator::step`'s `SteeringFaults.actuator_leak` | fraction of nominal steering slew rate lost | The wheel tracks a commanded steering angle more slowly. |

Emergent consequences that come out of the physics above rather than being
independently fault-injectable (so they are *not* additional rows here,
per the brief's "consequences come out of the model, never scripted"):
leg collapse (`strut::StrutOutputs::collapsed`, from exceeding ultimate load
or reacting a real load unlocked), an overload event and its lasting seal
damage, Miner's-rule fatigue life consumption, a brake fire, a tailstrike
(`structure::tailstrike_margin_deg`), an overweight-landing inspection
trigger, and wing-root bending fatigue accumulation.
