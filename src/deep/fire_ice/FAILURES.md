# fire_ice failures

One line per genuinely distinct physical fault this directory's models
support, matching `registry.rs` exactly (same names/ids). 120 failures
total: 36 fire-detection-loop + 9 combustion-zone + 25 extinguishing + 50
anti-ice.

`ATA | proposed name | model element it acts on | magnitude meaning (0..1) | effect`

## Fire detection loops (`fire_loops.rs`) — ATA 26, x9 zones x2 loops

Zones: ENG 1-4, APU, MLG BAY, CARGO FWD, CARGO AFT, AVIONICS. Each zone has
a thermistor loop (A) and a pneumatic loop (B); each loop has the same two
fault modes.

| ATA | proposed name | model element | magnitude (0..1) | effect |
|---|---|---|---|---|
| 26 | `<ZONE>` fire loop A open circuit | `fire_loops::LoopFaults.open_circuit` (loop A) | blends resistance reading toward the out-of-physical-range fault value | loop reports `loop_fault=true`; FDU trusts the other loop alone, or fails toward presumed fire if both loops fault together |
| 26 | `<ZONE>` fire loop A short circuit | `fire_loops::LoopFaults.short_circuit` (loop A) | blends resistance reading toward the fire-mimicking short value | loop reports `fire_signal=true` indistinguishably from real heat: false fire under OR logic or once the other loop also faults |
| 26 | `<ZONE>` fire loop B open circuit | `fire_loops::LoopFaults.open_circuit` (loop B) | blends pneumatic pressure reading toward the below-cold-soak fault floor | same loop-fault/fallback behaviour as loop A |
| 26 | `<ZONE>` fire loop B short circuit | `fire_loops::LoopFaults.short_circuit` (loop B) | blends pneumatic pressure reading toward a fire-mimicking high pressure | same false-fire behaviour as loop A |

Applies once per zone for `<ZONE>` in: ENG 1, ENG 2, ENG 3, ENG 4, APU, MLG
BAY, CARGO FWD, CARGO AFT, AVIONICS (9 x 4 = 36 rows).

## Combustion zone fire sources (`combustion.rs`) — ATA 26, x9 zones

| ATA | proposed name | model element | magnitude (0..1) | effect |
|---|---|---|---|---|
| 26 | ENG 1 flammable-fluid leak (fire source) | `combustion::ZoneSupply.fuel_available_kg_s` | scales leak rate 0..0.05 kg/s | fuel/air-limited combustion once an ignition source or crossed autoignition point is present; sustains until fuel/air/suppression removed |
| 26 | ENG 2 flammable-fluid leak (fire source) | `combustion::ZoneSupply.fuel_available_kg_s` | scales leak rate 0..0.05 kg/s | as ENG 1 |
| 26 | ENG 3 flammable-fluid leak (fire source) | `combustion::ZoneSupply.fuel_available_kg_s` | scales leak rate 0..0.05 kg/s | as ENG 1 |
| 26 | ENG 4 flammable-fluid leak (fire source) | `combustion::ZoneSupply.fuel_available_kg_s` | scales leak rate 0..0.05 kg/s | as ENG 1 |
| 26 | APU flammable-fluid leak (fire source) | `combustion::ZoneSupply.fuel_available_kg_s` | scales leak rate 0..0.03 kg/s | as ENG 1 |
| 26 | MLG BAY flammable-fluid leak (fire source) | `combustion::ZoneSupply.fuel_available_kg_s` | scales leak rate 0..0.01 kg/s | as ENG 1 |
| 26 | CARGO FWD flammable-fluid leak (fire source) | `combustion::ZoneSupply.fuel_available_kg_s` | scales leak rate 0..0.02 kg/s | as ENG 1 |
| 26 | CARGO AFT flammable-fluid leak (fire source) | `combustion::ZoneSupply.fuel_available_kg_s` | scales leak rate 0..0.02 kg/s | as ENG 1 |
| 26 | AVIONICS flammable-fluid leak (fire source) | `combustion::ZoneSupply.fuel_available_kg_s` | scales leak rate 0..0.005 kg/s | as ENG 1; can cross the conductive link into a neighbouring zone's own leak and ignite it there too |

## Extinguishing (`extinguishing.rs`) — ATA 26, 25 failures

| ATA | proposed name | model element | magnitude (0..1) | effect |
|---|---|---|---|---|
| 26 | ENG n bottle {1,2} leak (x8: n=1..4, bottle 1-2) | `extinguishing::BottleFaults.leak` | scales leak orifice area; full severity empties the bottle over ~9-10 h | bottle mass/pressure fall over time; delivers less (or no) agent if fired before repair |
| 26 | ENG n bottle {1,2} squib failure (x8) | `extinguishing::BottleFaults.squib_failure` | reduces achieved discharge orifice area; 1.0 = disc never ruptures | reduced or zero agent delivered when fired, regardless of a correct pushbutton sequence |
| 26 | APU bottle leak | `extinguishing::BottleFaults.leak` | as above | as above |
| 26 | APU bottle squib failure | `extinguishing::BottleFaults.squib_failure` | as above | as above |
| 26 | Cargo FWD suppression bottle leak | `extinguishing::BottleFaults.leak` (via `CargoSuppressionSystem`) | as above | as above |
| 26 | Cargo FWD suppression bottle squib failure | `extinguishing::BottleFaults.squib_failure` | as above | as above |
| 26 | Cargo AFT suppression bottle leak | `extinguishing::BottleFaults.leak` | as above | as above |
| 26 | Cargo AFT suppression bottle squib failure | `extinguishing::BottleFaults.squib_failure` | as above | as above |
| 26 | Cargo FWD smoke detector lens obscured | `extinguishing::SmokeDetectorFaults.lens_obscured` | derates effective smoke density used for the alarm threshold | delays, and at 1.0 fully prevents, a real smoke alarm despite genuine smoke |
| 26 | Cargo AFT smoke detector lens obscured | `extinguishing::SmokeDetectorFaults.lens_obscured` | as above | as above |
| 26 | Lavatory fusible link degraded | `extinguishing::LavatoryFaults.link_degraded` | raises effective melt temperature up to 50 C above the 77 C design rating | delays the automatic extinguisher's discharge past the design trigger temperature |

(8 + 8 + 2 + 2 + 2 + 2 + 1 = 25 rows once the "x8"/"x2" groups are expanded
one row per physical bottle/detector, matching `registry.rs`'s per-instance
components.)

## Anti-ice (`anti_ice.rs`) — ATA 30, 50 failures

| ATA | proposed name | model element | magnitude (0..1) | effect |
|---|---|---|---|---|
| 30 | L WING / R WING anti-ice valve stuck closed (x2) | `anti_ice::BleedAntiIceFaults.valve_stuck_closed` | reduces commanded bleed flow toward zero | leading edge gets no anti-ice heat, ices as an unheated surface would |
| 30 | L WING / R WING anti-ice valve stuck open (x2) | `anti_ice::BleedAntiIceFaults.valve_stuck_open` | floors delivered bleed flow at full regardless of command | continues heating once icing demand ends, drives skin/duct temperature into an overheat trip |
| 30 | L WING / R WING anti-ice duct leak (x2) | `anti_ice::BleedAntiIceFaults.duct_leak` | fraction of commanded flow lost before the piccolo tube | reduced bleed heat delivery weakens anti-ice protection, partial ice accretion |
| 30 | ENG n NACELLE anti-ice valve stuck closed (x4) | `anti_ice::BleedAntiIceFaults.valve_stuck_closed` | as wing | inlet lip gets no anti-ice heat, ices |
| 30 | ENG n NACELLE anti-ice valve stuck open (x4) | `anti_ice::BleedAntiIceFaults.valve_stuck_open` | as wing | overheats once demand ends |
| 30 | ENG n NACELLE anti-ice duct leak (x4) | `anti_ice::BleedAntiIceFaults.duct_leak` | as wing | weakened protection |
| 30 | `<PROBE>` heater open circuit (x8: pitot1-3, aoa1-3, tat1-2) | `anti_ice::ProbeHeaterFaults.heater_open_circuit` | fraction reduction of rated power deliverable | probe cannot hold above freezing, ices, risks blockage |
| 30 | `<PROBE>` heater controller fault (x8) | `anti_ice::ProbeHeaterFaults.controller_fault` | at 1.0 controller never commands power | heater never energises even in icing conditions |
| 30 | `<PROBE>` heater sensor fault (x8) | `anti_ice::ProbeHeaterFaults.sensor_fault` | blends sensed temperature toward a fixed stuck-warm reading | silent failure: controller believes probe warm, withholds heat while it genuinely ices |
| 30 | L WINDSHIELD / R WINDSHIELD film defect (x2) | `anti_ice::WindowHeatFaults.film_defect` | concentrates nameplate power by `1/(1-defect)^2` into a local hot spot | severe defect drives the hot spot past delamination then crack thresholds |
| 30 | L WINDSHIELD / R WINDSHIELD heat controller fault (x2) | `anti_ice::WindowHeatFaults.controller_fault` | at 1.0 heater commanded full-on unconditionally, no working cutout | film runs away to an overheat condition |
| 30 | L WINDSHIELD / R WINDSHIELD heat sensor fault (x2) | `anti_ice::WindowHeatFaults.sensor_fault` | blends sensed temperature toward a fixed stuck-warm reading | silent under-heating: controller withholds heat believing the window already warm |
| 30 | L WINDSHIELD / R WINDSHIELD rain removal system fault (x2) | `anti_ice::RainRemovalFaults.system_fault` | reduces the jet's effective dynamic pressure/shear-removal rate | water film clears more slowly or not at all, degrading visibility in rain |

(2+2+2 wing + 4+4+4 nacelle + 8+8+8 probes + 2+2+2 window + 2 rain =
6+12+24+6+2 = 50 rows once each named instance is counted individually,
matching `registry.rs`'s per-instance components.)
