# Main-deck catering high-loader (generic Mallaghan/Doll class)

## Sources

- Aviation Pros, "Catering to the A380" (https://www.aviationpros.com/gse/catering-cleaning-equipment/article/21257016/catering-to-the-a380): A380 catering trucks need about 2.5 m more reach than A330-class units; LSG A380 truck reaches 8.4 m (upper deck).
- Airbus A380 Aircraft Characteristics, main-deck door sill about 5.1 m (taken as 5100 mm; not re-checked in this wave).
- No manufacturer datasheet was found for the GH9000; chassis, box and cab sizes are generic estimates (e).
- No photos downloaded; folder `E:/fbw-gse/refs/cat_highloader/` is empty.

## Frame

- Units are millimetres, origin at the truck midpoint (x).
- x: from the rear (box door end) towards the cab.
- y: positive to the right when looking from the cab end.
- z: up from the ground.

## Dimensions

| Item | Value |
|---|---|
| Overall length | 8200 (e) |
| Width over bodies | 2500 (e) |
| Box body | 5100 x 2400 x 2300 (e) |
| Box floor, stowed | 1450 (e) |
| Box floor, raised | 5100 (A380 sill) |
| Lift travel | 3650 |
| Bridge extension | 1300 (e) |
| Cab top | 3050 (e) |
| Axles from rear end | 1300, 2600, 6500 (e) |
| Tyre diameter | 1020 (e) |

## Animation

- `fbw/gse/cat/lift` 0..1: box, bridge and livery rise 3650 mm; mid telescoping column stage and ram rise half that.
- `fbw/gse/cat/bridge` 0..1: bridge extends 1300 mm towards the cab end.
- The lift is modelled as a three-stage telescoping column, not a scissor.
