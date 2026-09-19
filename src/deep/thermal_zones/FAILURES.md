# thermal_zones — failures

Every failure this model supports (also registered in code in `registry.rs` via
`crate::deep::api::Registry`, `Area::ThermalZones`, area code 11 — the canonical source; this
file is the human-readable index). Format: `ATA | name | model element | magnitude meaning | effect`.

## ATA 21 — zone ventilation

| ATA | name | model element | magnitude (0..1) | effect |
|---|---|---|---|---|
| 21 | Main avionics bay ventilation fan failure | `network::VentilationLink.health` (`vents.main_avionics_fan`) | fraction of nameplate flow lost | MainAvionics loses purge airflow; steady-state temperature rises toward its baseline-heat/skin-conduction balance. |
| 21 | Upper avionics bay ventilation fan failure | `network::VentilationLink.health` (`vents.upper_avionics_fan`) | fraction of nameplate flow lost | UpperAvionics runs hot. |
| 21 | Forward cargo extract fan failure | `network::VentilationLink.health` (`vents.cargo_fwd_fan`) | fraction of nameplate flow lost | CargoFwd loses purge/cooling; a fire there burns hotter and smokes up faster. |
| 21 | Aft cargo extract fan failure | `network::VentilationLink.health` (`vents.cargo_aft_fan`) | fraction of nameplate flow lost | CargoAft runs hot. |
| 21 | Bulk cargo extract fan failure | `network::VentilationLink.health` (`vents.cargo_bulk_fan`) | fraction of nameplate flow lost | CargoBulk runs hot. |
| 21 | Belly fairing pack bay ram-air scoop/drain blockage | `network::VentilationLink.health` (`vents.belly_pack_bay_vent`) | fraction of nameplate flow lost | BellyFairingPacks loses its dominant cooling path; standing pack-bay heat accumulates. |
| 21 | APU compartment ventilation blockage | `network::VentilationLink.health` (`vents.apu_compartment_vent`) | fraction of nameplate flow lost | ApuCompartment overheats faster under any heat load, precursor to an APU fire. |

## ATA 26 — fire/smoke sources

| ATA | name | model element | magnitude (0..1) | effect |
|---|---|---|---|---|
| 26 | CargoFwd compartment fire | `network::Zone.injected_heat_w`/`.injected_smoke_kg_s` (`zones.cargo_fwd`) | fraction of reference 200 kW / 0.01 kg/s full-severity fire | CargoFwd heats and smokes; heat conducts to MainAvionics through the real link. |
| 26 | CargoAft compartment fire | as above (`zones.cargo_aft`) | as above | CargoAft heats and smokes; conducts to BellyFairingPacks/TailCone. |
| 26 | CargoBulk compartment fire | as above (`zones.cargo_bulk`) | as above | CargoBulk heats and smokes. |
| 26 | Engine 1-4 nacelle fire (×4) | `network::Zone.injected_heat_w`/`.injected_smoke_kg_s` (`zones.nacelle_cowl[n]`) | fraction of reference 500 kW / 0.005 kg/s full-severity fire | NacelleCowl_n heats rapidly; conducts into PylonEngine_n, threatening its wiring. |
| 26 | APU compartment fire | `network::Zone.injected_heat_w`/`.injected_smoke_kg_s` (`zones.apu_compartment`) | fraction of reference 300 kW / 0.008 kg/s full-severity fire | ApuCompartment heats and smokes; conducts into TailCone. |

## ATA 30 — ice & rain protection

| ATA | name | model element | magnitude (0..1) | effect |
|---|---|---|---|---|
| 30 | WingLeLeft anti-ice duct leak | `network::Zone.injected_heat_w` (`zones.wing_le_left`) | fraction of reference 30 kW full-severity leak | WingLeLeft overheats past its normal anti-ice cycle, threatening its insulation blanket. |
| 30 | WingLeRight anti-ice duct leak | as above (`zones.wing_le_right`) | as above | WingLeRight overheats. |
| 30 | Engine 1-4 nacelle anti-ice duct leak (×4) | `network::Zone.injected_heat_w` (`zones.nacelle_cowl[n]`) | fraction of reference 20 kW full-severity leak | NacelleCowl_n overheats, threatening its wiring. |
| 30 | Engine 1-4 nacelle vent scoop ice blockage (×4) | `network::VentilationLink.health` (`vents.nacelle_vent[n]`) | fraction of ram-air flow blocked by ice | NacelleCowl_n loses its large ram-air ventilation term; any heat present accumulates faster. |

## ATA 32 — landing gear bay doors

| ATA | name | model element | magnitude (0..1) | effect |
|---|---|---|---|---|
| 32 | NoseGearWell bay door jam | `network::VentilationLink.health` (`vents.nose_gear_door`) | degree door is stuck away from commanded position | Stuck open: bay runs cold at altitude. Stuck closed: bay retains brake/hydraulic heat. |
| 32 | WingGearWell bay door jam | as above (`vents.wing_gear_door`) | as above | Same effect for the wing-mounted MLG bay. |
| 32 | BodyGearWell bay door jam | as above (`vents.body_gear_door`) | as above | Same effect for the body-mounted MLG bay. |

## ATA 36 — pneumatic ducts

| ATA | name | model element | magnitude (0..1) | effect |
|---|---|---|---|---|
| 36 | Pylon 1-4 bleed duct leak (×4) | `network::Zone.injected_heat_w` (`zones.pylon[n]`) | fraction of reference 40 kW full-severity leak | PylonEngine_n overheats, conducting into its nacelle and wing trailing edge. |

## ATA 49 — APU

| ATA | name | model element | magnitude (0..1) | effect |
|---|---|---|---|---|
| 49 | APU bleed duct leak (tail cone run) | `network::Zone.injected_heat_w` (`zones.tail_cone`) | fraction of reference 25 kW full-severity leak | TailCone overheats, conducting into ApuCompartment/BodyGearWell. |

## ATA 53 — fuselage insulation

| ATA | name | model element | magnitude (0..1) | effect |
|---|---|---|---|---|
| 53 | Crown area insulation blanket damage | `network::Zone.insulation_effectiveness` (`zones.crown_area`) | fraction of insulating effectiveness lost | CrownArea tracks outside recovery temperature much more closely; conducts a colder/hotter ceiling into CabinUpperDeck. |

34 failures total. Every id, component and ECAM alert is registered in code in `registry.rs`
(`pub fn register(r: &mut Registry)`); this table must not drift from it — see
`registry.rs`'s own `topology_zone_names_used_in_effect_text_exist_in_the_built_network` test.
