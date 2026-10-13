# A380 ground support equipment

Hand-modelled, high-quality ground support equipment (GSE) for the Unofficial FlyByWire A380X for X-Plane 12, called from FlyByWire's own flyPad.

## Goal

A full A380 turnaround, as Airbus lays it out in *A380 Aircraft Characteristics – Airport and Maintenance Planning* (section 5-1, "Typical Ramp Layout"). Every unit is a real product, modelled from scratch from manufacturer data and photos. Each has working moving parts and wears the livery of the operators at the airport you are parked at.

## Licensing rules

- Every mesh, texture and livery we ship is made by us.
- GSX Pro (FSDreamTeam) and other commercial add-ons may be looked at, like photos, to see what equipment and services exist and what operators' vehicles look like. Nothing from them is copied, traced, sampled or converted.
- Reference photos are kept outside every repository and never ship.
- Operator names and logos (SATS, LSG Sky Chefs, dnata …) are trademarks, used only to identify the real operator. They are listed in `THIRD_PARTY_NOTICES.txt` like the airline liveries.

## Fleet

| Airbus symbol | Units | Real equipment |
|---|---|---|
| TOW | 1 | TLD TPX-500 towbarless tug (A380-rated) |
| GPU | 2 | TLD GPU-409-E-CUP 90 kVA diesel trailer (remote stands); jet-bridge-mounted units are the airport's |
| AC | 1 | Mobile pre-conditioned air unit, two hoses |
| AS | 1 | Air start unit |
| FUEL | 1 | Hydrant dispenser; Mercedes Econic tanker at airports without hydrants |
| CAT | 3 | Main-deck catering high-loader |
| UDCAT | 1 | Upper-deck catering high-loader (≈ 8 m lift) |
| PS | 3 | 2 main-deck motorised stairs, 1 upper-deck stair (open apron) |
| LDCL | 2 | Lower-deck container loader |
| CB | 1 | Belt loader |
| ULD | 2 | Tractor with LD3 dollies, AKE containers and PMC pallets |
| BULK | 1 | Tractor with bulk baggage carts |
| LV, WV, CLEAN | 1 each | Lavatory truck, potable water truck, cleaning van |
| extras | – | De-icing truck, cones, marshaller, wing-walkers (people last) |

Wheel chocks already exist (walkaround).

## Behaviour in the sim

FlyByWire's flyPad Ground > Services buttons (`A380Services.tsx`) call services. They send K events, which the plugin's `key_events.rs` currently forwards to X-Plane's `sim/ground_ops/*`. The plugin takes them over:

| flyPad button | K event | Brings in |
|---|---|---|
| Stairs | `TOGGLE_RAMPTRUCK` | PS ×3 (none while a jet bridge is attached) |
| Catering | `REQUEST_CATERING` | CAT ×3 + UDCAT, right-side doors |
| Baggage | `REQUEST_LUGGAGE` | LDCL ×2, CB, ULD ×2, BULK |
| Fuel | `REQUEST_FUEL_KEY` | FUEL at the wing refuel coupling |
| GPU | `REQUEST_POWER_SUPPLY` | GPU ×2 + AC |
| Jet bridge | `TOGGLE_JETWAY` | unchanged (`sim/ground_ops/jetway`) |

- **More services.** Water, lavatory, cleaning, air start, de-icing, cones and the marshaller go in a "More services" section on our own EFB page.
- **Automatic boarding services.** Water, lavatory and cleaning may also run automatically during boarding.
- **Safety area.** Equipment may enter only when the aircraft is on the ground and stationary, chocks or parking brake are set, the engines are off and the beacon is off. When the beacon comes on, everything leaves except the tug and the GPU.
- **Loading.** The plugin loads a model only when its service is called and unloads it when it leaves (`XPLMLoadObjectAsync`, `XPLMCreateInstance`). Only the current airport's livery variant is loaded.
- **Placement.** Equipment is placed in the world, not attached to the aircraft.
  - Positions come from Airbus's ramp layout and from the door and service-point positions on our model.
  - Ground height comes from `XPLMProbeTerrainXYZ`.
- **Animation.** Equipment appears in position. Its parts then move: the catering box rises to the door sill, the stairs extend, loader platforms rise and the fuel hose connects.
  - Instance datarefs (`fbw/gse/<part>`) drive the parts.
  - Sill heights are measured when the equipment arrives.
- **Branding.** The nearest airport's ICAO code picks the operator set: caterer, ground handler and fueller. Any other airport gets a generic, unbranded set.

## Build tooling (`msfs2xp-aircraft/tools/gse`)

- **References.** One sheet per vehicle with manufacturer dimensions and photos. The photos are stored outside the repository.
- **Models.** One Blender 5.2 Python script per vehicle, run headless with `blender --background --python`. It builds the vehicle to real dimensions, with separately animated parts, bevelled hard edges, UVs and materials. Shared helpers cover wheels and tyres, lights, hydraulic rams, railings, decals and materials.
- **Verification.** The script renders the model from the camera angles of its reference photos and puts each render beside its photo. This is repeated until the two match.
- **Textures.** Bakes produce albedo (with ambient occlusion), a normal map and a metal/gloss map in one 2048 set per vehicle. An operator livery is a decal layer over the base albedo, one variant per operator.
- **Export.** An exporter writes X-Plane OBJ8 directly from Blender: animated parts as `ANIM` blocks on the `fbw/gse/*` datarefs, plus `TEXTURE`, `normal` and `material_gloss` maps. `objdecimate` adds two lower levels of detail.

## Budgets

- About 30,000–80,000 triangles per vehicle at the closest level of detail, plus two levels of detail.
- One 2048 texture set per vehicle. The three CAT units share one model.
- A full turnaround is roughly 12 distinct models and about 100 MB of VRAM while all are out, and nothing when none are.

## Phases

1. **Pipeline, proven on the GPU.**
   - Reference sheet, model script and render check.
   - Bake, OBJ8 exporter and levels of detail.
   - Plugin instance loading, placement, and the flyPad GPU button taking over `REQUEST_POWER_SUPPLY`.
   - Verified in the sim before anything else is modelled.
2. **Core turnaround:** AC, AS, FUEL (dispenser and tanker), CAT/UDCAT, PS, and TOW with the pushback hookup.
3. **Cargo:** LDCL, CB, ULD and BULK trains, containers and pallets.
4. **Service trucks:** LV, WV, CLEAN; then the extras.
5. **Operator liveries:** added per airport alongside each vehicle.

## Verification

- **Models.** Every model passes the render-beside-photo check.
- **Exports.** Exporter output is tested for structure: balanced `ANIM` blocks, counts that match `POINT_COUNTS`, and level-of-detail ranges.
- **Plugin.** The placement and state logic gets unit tests: the safety-area rules, beacon-on departure, and airport operator lookup.
- **In the sim.** Every phase ends with a check in the sim. X-Plane is launched only when the user asks.
