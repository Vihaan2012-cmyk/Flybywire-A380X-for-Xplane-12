# Aircraft hydrant dispenser on a 6x4 truck chassis

## Sources

- Aircraft side: GSE requirements article, International Airport Review (airport.internationalairportreview.com, "A380 GSE requirements"), which cites the Airbus aircraft characteristics: two refuel couplings per wing about 18 m outboard of the centreline, maximum connector height 5.94 m; operator platforms on dispensers raise to about 4 m. AviationPros "The Airbus A380" gives underwing ports 5.9 m above ground.
- Truck proportions are generic of the Garsite / Rampmaster / BETA hydrant dispenser class (light truck chassis, two filter vessels, meter, hose reels, elevating operator platform). No manufacturer drawing was retrieved in the time available; every truck dimension below is an estimate (p).
- Coupling distance from the nose was not found; it is not needed for the model.
- No photos downloaded; `E:/fbw-gse/refs/fuel_hydrant/` is empty.

## Frame

- Units are millimetres.
- x: from the rear bumper (platform end) towards the cab.
- y: from the centreline, positive to the right when looking from the cab end.
- z: up from the ground.
- World x in Blender is mirrored (Frame offset 3700): the cab lies at negative world x.

## Key dimensions (all p)

| Item | Value |
|---|---|
| Overall length | 7560 |
| Width | 2500 |
| Cab roof | 3150 |
| Tyres | Ø 1060 x 295, twin rear axles (x 1500, 2850), front axle x 6050, track 2050 |
| Cab | x 5150 to 7350 |
| Equipment body | x 1650 to 5000, top 2560 |
| Filter vessels | 2 x Ø 680, 1600 long, axis z 1780, y +-430 |
| Meter vessel | Ø 500, 1200 long, z 1500 |
| Platform deck, stowed | z 2300, x 120 to 1500; rails to z 3400 |
| Platform lift | 1.8 m, deck up to z 4100 (matches the ~4 m of real platforms) |
| Wing coupler hose | stowed z about 2440; lifts 3.1 m to about 5.5 m, near the 5.94 m coupling |

## Animations

- `fbw/gse/fuel_hydrant/platform`, 0..1: translate z 0 to 1.8 m, on all `platform_*` parts.
- `fbw/gse/fuel_hydrant/hose`, 0..1: translate z 0 to 3.1 m, on `wing_hose_a` and `wing_coupler_*`. The hose is a rigid stub, not a flexing hose.
