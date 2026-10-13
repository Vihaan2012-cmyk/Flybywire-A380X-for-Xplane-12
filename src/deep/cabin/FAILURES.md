# Failures — deep/cabin (ATA 25/38/44/52)

One line per genuinely distinct physical fault. Full detail (component,
exact `model_field`, magnitude, effect) is registered in code in
`registry.rs`; this table is the required prose summary.

| ATA | proposed name | model element it acts on | magnitude (0..1) meaning | effect |
|---|---|---|---|---|
| 38 | Potable water tank/line leak | `water::WaterSystem` tank/line | 0 sealed .. 1 fully open leak, orifice flow scaled by gauge pressure | continuous water loss, faster depletion, eventual dry tank |
| 38 | Potable water bleed valve fault | `water::WaterFaults.bleed_valve_fault` | 0 healthy .. 1 no bleed pressurisation air | falls back to the slower backup compressor |
| 38 | Potable water backup compressor fault | `water::WaterFaults.compressor_fault` | 0 healthy .. 1 no backup pressurisation air | no pressurisation at all if bleed is also down: distribution fails |
| 38 | Potable water quantity sensor stuck | `water::WaterFaults.quantity_sensor_fault` | 0 healthy .. 1 (>0.5 latches) freezes last reading | displayed quantity stops tracking the real tank |
| 38 | Zone water heater fault (x3, FWD/MID/AFT) | `water::WaterFaults.heater_fault[zone]` | 0 healthy .. 1 no heating element output | no hot water at that zone's tap |
| 38 | Drain mast heater fault (x2, FWD/AFT) | `water::WaterFaults.mast_heater_fault[mast]` | 0 healthy .. 1 no heater output | mast can ice below freezing at cold OAT and block |
| 38 | Vacuum toilet generator fault | `waste::WasteFaults.generator_fault` | 0 healthy .. 1 no assisted suction | on ground/low altitude, flushes weak or ineffective |
| 38 | Zone waste tank level sensor stuck (x3) | `waste::WasteFaults.tank_level_sensor_fault[zone]` | 0 healthy .. 1 (>0.5 latches) freezes reading | crew cannot see the tank filling toward full |
| 38 | Zone toilet flush valve stuck open (x3) | `waste::WasteFaults.valve_stuck_open[zone]` | 0 healthy .. 1 (>0.5 latches) continuous small leak | tank fills without use, reaches full early |
| 38 | Zone toilet flush valve stuck closed (x3) | `waste::WasteFaults.valve_stuck_closed[zone]` | 0 healthy .. 1 flush proportionally ineffective | bowl does not clear on flush |
| 44 | Zone seat power/IFE wiring short (x3) | `ife::IfeFaults.seat_fault[zone]` | 0 healthy .. 1 fraction of zone wiring shorted | I^2R self-heating: overheat then smoke, then the zone's own protection trips it dead |
| 44 | IFE head-end server failure (x2) | `ife::IfeFaults.server_fault[server]` | 0 healthy .. 1 (>=0.95 fails outright) | that server's content lost; redundant server covers until both fail |
| 25 | Galley bus feed fault (x3 galleys) | `galley::GalleyFaults.bus_fault[zone]` | 0 healthy .. 1 galley dead | oven, chiller and boiler all lose power in that galley only |
| 25 | Galley oven thermostat stuck (x3) | `galley::GalleyFaults.oven_overheat[zone]` | 0 healthy cycling .. >0 stuck closed | cavity temperature runs away past setpoint to the smoke threshold |
| 25 | Galley chiller compressor failure (x3) | `galley::GalleyFaults.chiller_fault[zone]` | 0 healthy .. 1 no cooling capacity | compartment warms back to cabin ambient over tens of minutes |
| 25 | Galley water boiler fault (x3) | `galley::GalleyFaults.boiler_fault[zone]` | 0 healthy .. 1 no heating element output | no hot water for beverages from that galley |
| 52 | Door seal leak | `doors_slides::DoorSlideFaults.seal_leak` | 0 sealed .. 1 fully open (2 cm^2), `mdot=Cd*A*sqrt(2*rho*dP)` | continuous cabin air loss at that door, extra load on the pressurisation outflow valves |
| 52 | Evacuation slide bottle leak | `doors_slides::DoorSlideFaults.bottle_leak` | 0 healthy .. 1 empties the bottle over 24 h | slide may fail to inflate to a usable pressure once armed/fired |
| 52 | Door not-latched sensor stuck | `doors_slides::DoorSlideFaults.latch_sensor_fault` | 0 healthy .. 1 (>=0.5 latches) freezes reading | indication can disagree with the door's real position |
| 52 | Cargo door actuator jam | `doors_slides::DoorSlideFaults.actuator_jam` | 0 free .. 1 fully seized | caps door travel at `(1-jam)*100%`; cannot reach a target beyond it |
| 52 | Cargo door actuator hydraulic circuit loss | `doors_slides::DoorSlideFaults.hydraulic_loss` | 0 full pressure .. 1 no driving pressure | actuator cannot move at all |

41 distinct fault instances total (9 water + 10 waste + 5 IFE + 12 galley +
5 doors/slides, counting each per-zone/per-instance fault separately as the
brief asks — "one line per genuinely distinct physical fault", grouped
above by family to avoid padding with renamings of the same fault repeated
per zone).
