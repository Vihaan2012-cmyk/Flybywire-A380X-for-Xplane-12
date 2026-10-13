# A380 upper-deck passenger stair (generic self-propelled stair truck)

## Sources

- Upper-deck door height about 7.86 m above the tarmac, from a secondary source (Simple Flying article on the A380 upper deck). Airbus A380 Aircraft Characteristics (AC) was not read in this wave, so the exact sill is unverified.
- No manufacturer datasheet (Mallaghan, TLD, Fast Global) was found by web search. The model is a generic design from the general layout of such trucks: two flights, an intermediate landing, a raising upper flight, a sliding top platform with rubber bumper, a canopy and a cab.
- No photos were downloaded; `E:/fbw-gse/refs/stairs_upper/` is empty.

## Frame

- Units are millimetres.
- *x*: from the stair foot (x 0) towards the aircraft; the model origin is x 6000.
- *y*: from the centreline, positive to the right looking along +x.
- *z*: up from the ground.

## Dimensions (all (p), estimated)

| Item | Value |
|---|---|
| Sill target | 7860 |
| Platform deck | z 7820, x 10550-12050, width 1800 |
| Platform slide extension | 450 along x, `fbw/gse/stairs_upper/platform` |
| Intermediate landing | z 4000, x 4300-5900, width 1280 |
| Lower flight | z 420 to 4000, x 40-4300, riser about 179 |
| Upper flight | z 4000 to 7820, x 5900-10550, riser about 174 |
| Stair width | 1200 between stringers at y +-640 |
| Handrail height | 1000, mid rail at 500 |
| Canopy | roof at z 10170, over the platform |
| Chassis | x 3400-11200, width 2100, frame z 430-740 |
| Wheels | 4 axles, diameter 920, width 300 |
| Cab | x 9600-11300, top z 2500 |
| Lift travel | 3900 (`fbw/gse/stairs_upper/lift`: 0 is lowered, 1 is raised) |

Animated parts: the upper flight, platform, canopy, mast inner and supports lift together. The platform slide also extends along +x (Blender +Y).

## Uncertain

- Exact sill height and door position; the real stair length and slope.
- Wheel layout and cab design are generic.
