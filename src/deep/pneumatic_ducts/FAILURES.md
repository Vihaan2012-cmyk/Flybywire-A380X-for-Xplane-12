# Failures — pneumatic_ducts

Format: `ATA | proposed name | model element it acts on | magnitude meaning (0..1) | effect`.
Full catalogue detail (components, params, ECAM) is registered in code via
`registry.rs` (`crate::deep::api`); this file is the flat list the brief asks for.

| ATA | proposed name | model element | magnitude (0..1) | effect |
|---|---|---|---|---|
| 36 | Engine bleed duct leak | `duct::DuctSectionFaults.leak` (`network::DuctNetworkFaults.engine_duct[n]`) | crack area, 0 healthy .. 1 = 2% of duct bore | mass escapes to the pylon zone (`leak.rs`); manifold pressure sags; heats the pylon |
| 36 | Engine bleed duct rupture | `duct::DuctSectionFaults.rupture` (`engine_duct[n]`) | severance area, 0 .. 1 = full bore | large near-sonic jet, impingement-effectiveness heat into the pylon; reliably trips that pylon's ODLS and latches the isolation valve |
| 36 | Engine bleed duct insulation damage | `duct::DuctSectionFaults.insulation_damage` (`engine_duct[n]`) | lagging condition, 0 intact .. 1 bare pipe (10x conductance) | raises ordinary (non-fault) heat loss to the pylon, no trip alone but primes ODLS to trip sooner on a subsequent leak |
| 36 | Engine precooler fouling | `precooler::PrecoolerFaults.fouling` (`engine_precooler[n]`) | core conductance loss, 0 clean .. 1 = 80% lost | outlet runs hotter for the same cooling flow; FAV opens further to compensate, may not reach target at full fouling |
| 36 | Engine precooler fan air valve stuck | `precooler::PrecoolerFaults.fan_air_valve_stuck` (`engine_precooler[n]`) | 0 healthy .. 1 seized at current position | cooling flow frozen at whatever it was; hard overtemp trip (true-temperature path) still evaluates but a fully-open-stuck valve cannot heed a "close" it never needed and a closed-stuck one cannot heed "open" |
| 36 | Engine precooler outlet sensor fault | `precooler::PrecoolerFaults.temp_sensor_fault` (`engine_precooler[n]`) | 0 healthy .. 1 frozen reading | modulating FAV loop under/over-corrects against a stale reading; independent true-temperature overtemp trip unaffected |
| 36 | Engine precooler check valve failure | `precooler::PrecoolerFaults.check_valve_failure` (`engine_precooler[n]`) | 0 healthy .. 1 fully open in reverse | ambient air drawn back into the duct when duct pressure sags below ambient, cooling/diluting it uncontrolled |
| 36 | APU bleed duct leak | `network::DuctNetworkFaults.apu_duct.leak` | same as engine duct leak | heats the APU bay zone |
| 36 | APU bleed duct rupture | `network::DuctNetworkFaults.apu_duct.rupture` | same as engine duct rupture | can trip the APU bay ODLS and latch the APU isolation valve |
| 36 | APU bleed duct insulation damage | `network::DuctNetworkFaults.apu_duct.insulation_damage` | same as engine duct insulation damage | same, APU bay zone |
| 36 | APU precooler fouling | `network::DuctNetworkFaults.apu_precooler.fouling` | same as engine precooler fouling | same |
| 36 | APU precooler fan air valve stuck | `network::DuctNetworkFaults.apu_precooler.fan_air_valve_stuck` | same as engine precooler FAV stuck | same |
| 36 | APU precooler outlet sensor fault | `network::DuctNetworkFaults.apu_precooler.temp_sensor_fault` | same as engine precooler sensor fault | same |
| 36 | APU precooler check valve failure | `network::DuctNetworkFaults.apu_precooler.check_valve_failure` | same as engine precooler check valve | same |
| 36 | Engine HP valve stuck | `network::UpstreamFaults.hp_valve_stuck` (`upstream[n]`) | 0 healthy .. 1 seized at current position | stuck shut: no HP6 backup once IP8 falls below the EASA switch-over pressure; stuck open: needless HP6 draw once IP8 alone would do |
| 36 | Engine PR (shutoff) valve stuck | `network::UpstreamFaults.pr_valve_stuck` (`upstream[n]`) | 0 healthy .. 1 seized at current position | stuck shut: that engine can no longer supply its own duct at all; stuck open: an ODLS trip can no longer isolate it |
| 36 | Engine IP8 check valve stuck closed | `network::UpstreamFaults.ip_check_valve_stuck_closed` (`upstream[n]`) | 0 healthy .. 1 fully shut | forces reliance on the HP valve, a real shift from the efficient IP8 source to the costlier HP6 one |
| 36 | Pack supply duct leak | `network::DuctNetworkFaults.packs[n].leak` | same as engine duct leak | starves that pack's supply pressure; other consumers largely unaffected |
| 36 | Pack supply duct rupture | `network::DuctNetworkFaults.packs[n].rupture` | same as engine duct rupture | same, far more severely |
| 36 | Pack supply duct insulation damage | `network::DuctNetworkFaults.packs[n].insulation_damage` | same as engine duct insulation damage | same, wing-root zone |
| 30 | Wing anti-ice duct leak | `network::DuctNetworkFaults.wai[n].leak` | same as engine duct leak | heats that wing's leading-edge zone directly |
| 30 | Wing anti-ice duct rupture | `network::DuctNetworkFaults.wai[n].rupture` | same as engine duct rupture | same, far more severely; reliably trips that side's leading-edge ODLS |
| 30 | Wing anti-ice duct insulation damage | `network::DuctNetworkFaults.wai[n].insulation_damage` | same as engine duct insulation damage | same |
| 36 | Engine start duct leak | `network::DuctNetworkFaults.start[n].leak` | same as engine duct leak | weakens starter torque available; heats that engine's pylon |
| 36 | Engine start duct rupture | `network::DuctNetworkFaults.start[n].rupture` | same as engine duct rupture | same, far more severely |
| 36 | Engine start duct insulation damage | `network::DuctNetworkFaults.start[n].insulation_damage` | same as engine duct insulation damage | same |
| 36 | Engine start duct check valve failure | `network::DuctNetworkFaults.start_check_valve_failure[n]` (`duct::one_way_transfer_kg`) | 0 healthy (perfect non-return) .. 1 fully open in reverse | once that engine lights, its own pressure leaks backward into its own duct (and from there, the cross-bleed chain) instead of being blocked |
| 36 | Hydraulic reservoir pressurisation duct leak | `network::DuctNetworkFaults.hyd_reservoir[n].leak` | same as engine duct leak | reduces reservoir air pressurisation, raising pump cavitation risk at high demand |
| 36 | Hydraulic reservoir pressurisation duct rupture | `network::DuctNetworkFaults.hyd_reservoir[n].rupture` | same as engine duct rupture | same, far more severely |
| 36 | Hydraulic reservoir pressurisation duct insulation damage | `network::DuctNetworkFaults.hyd_reservoir[n].insulation_damage` | same as engine duct insulation damage | same |
| 36 | ODLS loop A open circuit | `odls::OdlsFaults.loop_a_open` (`network::DuctNetworkFaults.odls[zone]`) | 0 healthy .. 1 open (>=0.5 fully open) | loop A stops providing a valid reading; loop B alone still governs (fail-safe voting); both open at once reports a FAULT with no detection |
| 36 | ODLS loop A short circuit | `odls::OdlsFaults.loop_a_short` (`odls[zone]`) | 0 healthy .. 1 shorted (>=0.5 fully shorted) | loop A pegs hot, tripping that zone's isolation on its own (a real false alarm from a wiring fault) |
| 36 | ODLS loop B open circuit | `odls::OdlsFaults.loop_b_open` (`odls[zone]`) | same as loop A open | same, loop B |
| 36 | ODLS loop B short circuit | `odls::OdlsFaults.loop_b_short` (`odls[zone]`) | same as loop A short | same, loop B |
| 36 | ODLS false detection | `odls::OdlsFaults.false_detection` (`odls[zone]`) | 0 healthy .. 1 = trips a cold zone on its own | trips and latches that zone's isolation valve shut with no real overheat present — a nuisance trip, distinct from a loop wiring fault |

35 distinct physical faults (32 under ATA 36, 3 under ATA 30). None are renamings: each row acts on a
different struct field / physical component, and (leak vs. rupture vs. insulation damage) are three
different physical damage mechanisms on the same duct, not the same fault at different severities —
a small leak and a full severance follow different area-sizing rules and, for leak specifically, the
leak-vs-impingement heat-transfer effectiveness ramp in `leak.rs` makes them behave differently, not
just "more of the same number".
