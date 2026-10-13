# Boom de-icing truck (Vestergaard Elephant Beta 15 class)

## Sources

- Vestergaard Elephant BETA-15 product page and brochure (vestergaardcompany.com): boom lifts the operator's eye level above 15.5 m and the nozzle above 25 m; fluid application 20-240 L/min.
- Standard Elephant BETA brochure (vestergaardcompany.com/wp-content/uploads/2018/04/Beta.pdf): 12 m or 14 m boom, nozzle reach up to 22.6 m.
- Inavia GSE datasheet for a Volvo FL7 / Elephant Beta: service height 13.1 m. A used-unit listing gives an 8,000 L tank, 25.1 t GVW and 17.1 t empty.
- Overall length, width and height of the Beta-15 were not found. They are estimates (e) for a 6x4 chassis with a tank behind the cab.

No photos were downloaded; `E:/fbw-gse/refs/deice_truck/` is empty.

## Frame

- Units are millimetres.
- *x*: from the rear bumper towards the cab.
- *y*: positive to the right when facing the cab.
- *z*: up from the ground.

## Dimensions

| Item | Value |
|---|---|
| Overall length (e) | 8,850 |
| Width over lockers (e) | 2,500 |
| Cab roof height (e) | 3,050 |
| Stowed height over beacon (e) | 3,500 |
| Rear axles at x | 1,700 and 3,000 |
| Front axle at x | 7,350 |
| Tyre | 1,050 diameter, 315 wide |
| Tank | horizontal cylinder, radius 1,000, x 350 to 5,600 |
| Boom hinge | x 1,250, z 3,350 |
| Boom sections | 3 x 4,300, nested |
| Extension travel | mid 3,600, top 7,200 |
| Boom elevation | 0 to 72 degrees |
| Basket | 1,500 x 1,440 x 1,900 with nozzle on the roof |

Fully raised and extended, the basket floor is about 17 m high and the nozzle about 20 m. This is short of the 25 m the Beta-15 reaches, so a fourth section or longer travel is needed for an A380 fin tip.

## Animation

- `fbw/gse/deice/boom`: rotate about the hinge, axis +x in Blender coordinates, 0 to 72 degrees. The sign in X-Plane is unchecked.
- `fbw/gse/deice/extend`: translate along the boom. Each section moves by its own travel. The basket and its mount use the full top travel.
- `fbw/gse/deice/basket`: rotate about the vertical pivot under the basket, -90 to 90 degrees.
