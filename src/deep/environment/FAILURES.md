# Environment area — failure catalogue

`ATA | proposed name | model element it acts on | magnitude meaning (0..1) | effect`.
Registered in code (with components and ECAM) in `registry.rs`; this file
is the same catalogue in prose for quick scanning.

## Bird strike (`bird_strike.rs`)

| ATA | name | model element | magnitude | effect |
|---|---|---|---|---|
| 72 | Bird strike fan blade damage | `StrikeOutcome.fan_damage_frac`, per engine | 0 none .. 1 destructive (blade impact energy vs. CS-E 800 large-bird reference) | vibration, thrust loss, possible surge/flameout |
| 72 | Bird strike core ingestion FOD | `StrikeOutcome.core_ingestion_frac`, per engine | fraction of ingested mass reaching the IP compressor (bypass-ratio flow split) | compressor blade damage, efficiency loss |
| 56 | Bird strike windshield damage | `StrikeOutcome.windshield_crack`/`.windshield_penetrated`, per panel | impact energy / CS-25.775(b) reference; >0.6 cracks, >=1.0 penetrates | visibility loss, possible depressurisation |
| 53 | Bird strike radome damage | `StrikeOutcome.radome_damage_frac` | impact energy / GENERIC radome reference (half the windshield's) | weather radar loss, drag, possible departure |
| 57 | Bird strike leading edge dent | `StrikeOutcome.leading_edge_dent_drag_delta_cd`, per segment | impact energy / CS-25.631 reference | local drag increment; 1.0 = inspection item |
| 32 | Bird strike nose gear damage | `StrikeOutcome.nose_gear_damage_frac` | impact energy / CS-25.631 reference | retraction/steering fault at high magnitude |
| 34 | Bird strike air data probe blockage | `StrikeOutcome.probe_blocked`, per probe | 0 clear, 1 blocked (binary) | unreliable airspeed/AoA on that probe |

## Lightning strike (`lightning.rs`)

| ATA | name | model element | magnitude | effect |
|---|---|---|---|---|
| 53 | Lightning radome damage | `LightningEvent.radome_damage_frac` | peak current / ARP5412 200 kA reference, x20 worse missing the diverter strip | weather radar loss, possible departure |
| 53 | Lightning composite extremity burn | `LightningEvent.structure_damage_frac` | GENERIC 0.05 x (peak current / 200 kA) | mesh/paint damage, inspection item |
| 34 | Lightning standby compass deviation | `LightningEvent.compass_error_deg` | GENERIC 0..10 deg, scales with current and nose proximity | fixed heading offset until compass swing |
| 24 | Lightning induced bus/computer transient | `LightningEvent.transients[].peak_volts`, per bus | peak current x GENERIC exposure factor (0.06..0.35) | possible reset/data corruption above 50 V (GENERIC) |

## Hail (`hail.rs`) — persistent/cumulative, see `registry.rs`'s `register_hail`

| ATA | name | model element | magnitude | effect |
|---|---|---|---|---|
| 53 | Hail radome damage | `HailOutcome.damage_frac`/`.wxr_attenuation_frac`/`.drag_delta_cd` (Radome) | cumulative energy density / 20mm-at-VMO x 15 hits (C-grade radome limit, cited) | WXR attenuation/beam distortion, drag, persists after the storm |
| 56 | Hail windshield damage | `HailOutcome.damage_frac`/`.window_heat_fault`/`.visibility_loss_frac`/`.leak_area_m2` (Windshield) | cumulative energy density / GENERIC 30mm-at-VMO x 15 hits | visibility loss; window-heat fault >0.3; pressurisation leak >0.9 |
| 57 | Hail leading edge/slat damage | `HailOutcome.damage_frac`/`.clmax_delta`/`.slat_jam_risk_frac` (WingLeadingEdge) | cumulative energy density / GENERIC 30mm-at-VMO x 15 hits | drag, max-lift penalty, slat jam risk >0.5 |
| 72 | Hail/ice engine ingestion | `HailOutcome.fan_damage_frac`/`.compressor_efficiency_loss_frac`/`.flameout_risk_frac` (EngineInlet) | fan: energy density/reference; compressor: GENERIC erosion/kg, capped 0.25; flameout: ingested mass / GENERIC N1-scaled tolerance (CS-E 790/14 CFR 33.68) | fan damage, permanent efficiency loss, flameout/roll-back risk (worse at low power) |
| 71 | Hail nacelle/cowl damage | `HailOutcome.damage_frac`/`.drag_delta_cd` (Nacelle) | cumulative energy density / GENERIC 30mm-at-VMO x 15 hits | drag increase, inspection item |
| 34 | Hail probe/antenna damage | `HailOutcome.damage_frac` (Probe) | cumulative energy density / GENERIC 15mm-at-VMO x 15 hits | unreliable airspeed/AoA or lost antenna |

## Volcanic ash (`volcanic_ash.rs`)

| ATA | name | model element | magnitude | effect |
|---|---|---|---|---|
| 72 | Volcanic ash NGV glassing | `AshOutputs.flow_capacity_loss_frac` | deposited molten-ash mass / GENERIC 2 kg full-blockage reference | reduced core flow capacity, surge-margin loss |
| 72 | Volcanic ash compressor erosion | `AshOutputs.compressor_efficiency_loss_frac` | GENERIC erosion integral, capped 0.25 | permanent compressor efficiency loss |
| 56 | Volcanic ash windshield abrasion | `AshOutputs.windshield_visibility_loss_frac` | GENERIC cumulative sandblasting rate | progressive, permanent visibility loss |
| 34 | Volcanic ash pitot blockage | `AshOutputs.pitot_blockage_frac` | GENERIC blockage growth x concentration | unreliable airspeed |

## Ice crystal icing (`ice_crystal_icing.rs`)

| ATA | name | model element | magnitude | effect |
|---|---|---|---|---|
| 72 | Ice crystal core accretion | `IceCrystalOutputs.flow_capacity_loss_frac` | accreted mass / GENERIC 0.5 kg reference, only near a 0 C surface | reduced core flow capacity, periodic shedding |
| 72 | Ice crystal engine roll-back/flameout risk | `IceCrystalOutputs.rollback_risk_frac`/`.flameout_risk_frac` | loss^2 (GENERIC), +0.4 spike on shedding | core roll-back, possible flameout |

## Runway contamination (`runway_contamination.rs`)

| ATA | name | model element | magnitude | effect |
|---|---|---|---|---|
| 32 | Runway contamination friction/hydroplaning loss | `RunwayFrictionOutput.mu_effective` | 1 - mu_effective/0.40 (GENERIC dry reference) | longer stopping distance, reduced directional control |

## Wind shear and turbulence (`wind_shear.rs`)

Not a component fault: a direct hazard-detection metric (F-factor) and a
gust field, both feeding the flight model/PWS directly, matching the real
aircraft (no `FailureDef`/`ComponentDef` registered; see `registry.rs`'s
`register_wind_shear` for the ECAM entry this still raises).
