# APU deep model — failures

`ATA | proposed name | model element it acts on | magnitude meaning (0..1) | effect`.
Registered in code through `crate::deep::api` — see `registry.rs` for the exact
`FailureDef`/`ComponentDef`/`EcamAlert` entries (this file is the human-readable
index of the same 18 failures; `registry.rs` is authoritative).

| ATA | Name | Model element | Magnitude (0..1) | Effect |
|---|---|---|---|---|
| 49 | APU core compressor erosion | `power_section.rs::PowerSectionFaults.compressor_efficiency_loss` | isentropic efficiency loss fraction | Same Euler blade work but less becomes useful pressure rise: compressor exit pressure falls for the same speed; EGT rises for the same fuel flow/speed to hold governed N under load. |
| 49 | APU turbine damage | `power_section.rs::PowerSectionFaults.turbine_efficiency_loss` | isentropic efficiency loss fraction | Less shaft work extracted per unit expansion: more of the gas's enthalpy survives to the exit (higher EGT); the governor must burn more fuel to hold governed N under the same load. |
| 49 | APU load/bleed compressor erosion | `load_compressor.rs::LoadCompressorFaults.efficiency_loss` | isentropic efficiency loss fraction | Lower delivered bleed pressure for the same demand and spool speed. |
| 49 | APU load compressor IGV actuator jam | `load_compressor.rs::LoadCompressorFaults.igv_jam` (via `actuator.rs::Actuator`) | actuator seizure fraction, 0=free .. 1=frozen wherever it currently is | Vanes freeze at their current opening; if that is closed relative to current demand, bleed delivery is starved even though the aircraft calls for more. |
| 49 | APU surge control valve actuator jam | `load_compressor.rs::LoadCompressorFaults.scv_jam` (via `actuator.rs::Actuator`) | actuator seizure fraction; freezes at whatever recirculation position it held — stuck open or stuck closed depending on the demand history when it jams | Cannot open to recirculate flow when bleed demand drops faster than it can follow: the load compressor's operating point falls below its surge line and genuinely surges (`compressor_map.rs::Point.in_surge`). |
| 49 | APU starter motor degradation/failure | `starter.rs::StarterFaults.starter_degradation` | weakens the shared back-EMF/torque constant | More current for less torque at the same speed: a slower, weaker start; in severe cases the core never reaches light-off speed. |
| 49 | APU igniter failure | `starter.rs::StarterFaults.igniter_failure` | raises the effective light-off speed threshold | At full failure the threshold sits at/above self-sustaining speed (unreachable by the starter alone): a hung start with fuel never lit. |
| 49 | APU fuel control unit (metering valve) fault | `fuel_control.rs::FuelControlFaults.metering_valve_jam` | metering valve actuator seizure fraction | Fuel flow freezes at whatever it was delivering; over- or under-fuels the combustor from then on regardless of the governor's command. |
| 49 | APU governor speed sensor fault | `faults.rs::ApuFaults.speed_sensor_bias` (applied in `apu.rs::Apu::step`) | under-reads true N by up to `params::N_SENSOR_MAX_BIAS_PERCENT` | The governor keeps commanding fuel for a speed error that no longer exists, driving true N up toward the physical overspeed protection (`power_section.rs::PowerSection::overspeed_tripped`). |
| 49 | APU oil leak | `oil.rs::OilFaults.leak` | fraction of `MAX_LEAK_RATE_L_S` | Tank level falls; the pump progressively starves as level drops below the low-level threshold, pressure falls, and sustained low pressure while running trips low oil pressure protection. |
| 49 | APU inlet door actuator jam | `inlet_door.rs::InletDoorFaults.jam` | actuator seizure fraction | Door fails to reach fully open, imposing a continuing inlet total-pressure loss that reduces available power and raises EGT for the same demand. |
| 49 | APU generator 1 winding/bearing wear | `generators.rs::GeneratorFaults.efficiency_loss` (gen1) | winding/bearing wear fraction | More shaft power needed for the same electrical output, loading the core's torque balance. |
| 49 | APU generator 1 overload protection failure | `generators.rs::GeneratorFaults.overload_protection_failed` (gen1) | boolean, represented 0/1 | Removes the normal clamp at rated shaft power: an overloaded generator keeps demanding ever more shaft torque instead of being current-limited. |
| 49 | APU generator 2 winding/bearing wear | `generators.rs::GeneratorFaults.efficiency_loss` (gen2) | winding/bearing wear fraction | Same as generator 1, generator 2. |
| 49 | APU generator 2 overload protection failure | `generators.rs::GeneratorFaults.overload_protection_failed` (gen2) | boolean, represented 0/1 | Same as generator 1, generator 2. |
| 49 | APU EGT sensor fault | `faults.rs::ApuFaults.egt_sensor_bias` (applied in `apu.rs::Apu::step`) | under-reads true EGT by up to `params::EGT_SENSOR_MAX_BIAS_C` | Cockpit-indicated EGT reads low relative to the true turbine-exit temperature, masking a real overtemperature from the crew/ECAM; the true physics and the hard protective trip are unaffected (they read the true value). |
| 49 | APU fire loop failure | `fire.rs::FireFaults.loop_failure` | loop failure fraction | A real fire is never confirmed: automatic fuel/bleed shutoff never commands and the bottle cannot be commanded to discharge. |
| 49 | APU fire bottle squib failure | `fire.rs::FireFaults.squib_failure` | squib failure fraction | Fire is confirmed and shutoff still commands, but the extinguisher bottle never discharges. |

18 distinct physical faults, one line each — no renamed duplicates. Each is
exercised by at least one `#[cfg(test)]` in its own file showing the fault
changing a real output (see `PROGRESS.md` for the per-file test summaries).

`starter_degradation` (row 6 above) also has a second, legacy entry point:
`failures::extra` id 49_002 ("APU starter fault", the flat catalogue's own
pre-`deep` entry for the same physical fault) folds into it by `max` in
`live.rs::faults_from`, since `deep::live` areas cannot otherwise see a
legacy id at all. This is not a 19th physical fault, just a second armer
for the same one.
