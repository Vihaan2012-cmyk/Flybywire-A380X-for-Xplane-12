# Self-propelled belt loader (TLD NBL-class) for the A380 bulk door

## Sources

- TLD NBL conveyor belt datasheet (aeroservicios.com/uploads/manuals/NBL_datasheet.pdf), saved to `E:/fbw-gse/refs/belt_loader/nbl.pdf`, tolerance 5 %:
  - boom 7800 long, 700 wide, belt 600 wide, belt speed 18 m/min;
  - tyres 225/70 R15 front and rear, rear drive axle, front steer axle;
  - 4-cylinder diesel, two lift cylinders, rotating beacon, optional cab, rub rails.
- TLD NBL-E listings (Fortbrand, Aviaco): about 8100 long, 2100 wide, 1880 high (boom lowered).
- Aircraft side: Airbus A380 Aircraft Characteristics (Dec 2023) section 2-3-0 ground clearances. The lowest door, F1, is taken to be the bulk door. Sill height 2.34-2.59 m from ground (2.45 m mid), 4.41 m on jacks. The table does not label F1 by name, so this is an inference.
- Photos: none downloaded. Dimensions were taken from the datasheet only.

## Frame

- Units are millimetres.
- *x*: from the chassis rear end towards the boom tip (the aircraft end).
- *y*: from the centreline, positive to the right looking along +x.
- *z*: up from the ground.

## Key dimensions

| Item | Value |
|---|---|
| Chassis length | 4600 |
| Chassis width over rub rails | 1660 |
| Track | 1560 |
| Rear axle x | 900 |
| Front axle x | 3700 |
| Tyres | 225/70 R15, diameter 640, width 225 |
| Boom hinge | x 700, z 820, y -250 |
| Boom length (hinge to tip) | 7400, plus a 150 bumper |
| Boom overall width | 700 |
| Belt width | 600 |
| Belt top height at hinge | 970 |
| Raise angle | 0 to 14 deg |
| Belt top at tip when raised | about 2600 |
| Cab | x 2750-4350, y 40-840, roof 2000-2120, right side |
| Handrail height over belt | 560 |

## Animation

- `fbw/gse/belt/boom` 0 to 1 rotates the boom group 0 to 14 deg about the hinge axis (lateral).
- The sign of the XP rotation is not verified.
- The lift cylinder is static and not animated.

## Liveries

Three own text-and-colour schemes (dnata, Swissport, Menzies) are drawn by `draw_liveries()` into `E:/fbw-gse/out/belt_loader/liveries/`.
