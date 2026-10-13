# Towed diesel air start unit (TLD ASU-600 class)

## Sources

- TLD ASU-600 design sheet: https://aeroservicios.com/uploads/manuals/4492_ASU-600_TLD.pdf (trailer mounted 5180 x 2080 x 2230 mm, skid 5030 x 2080 x 1650 mm, tolerance 5 %; the larger 270-300 ppm class).
- Aerospecialties and Fortbrand listings for the ASU-600 150/180 ppm: https://www.aerospecialties.com/product/tld-asu-600-100-150-ppm-jet-engine-air-start-unit/ and https://fortbrand.com/?p=1100 : 156 x 60 x 76 in, 3660 kg. This is the size modelled.
- TLD product page: https://www.tld-group.com/products/air-starters/asu-600-100-150-180/
- Aircraft side: Airbus A380 Aircraft Characteristics for Airport Planning, section 5-4 (ground connections). The HP air ground connector (ISO 1034, 3 in) sits in the belly; its exact station was not read from the manual and is estimated here.
- No photos downloaded; none stored in E:/fbw-gse/refs/asu_unit/.

## Frame

- Millimetres. x from the engine end (rear) towards the towbar, y positive to the right looking from the towbar end, z up from the ground. Frame origin at x 1980.
- (e) marks estimates.

## Dimensions

| Item | Value |
|---|---|
| Length including towbar | 3962 |
| Width | 1524 |
| Height to hood top | 1930 |
| Enclosure | x 120-3250, z 650-1930 (e) |
| Wheelbase | 1900 (e) |
| Track | 1400 (e) |
| Tyres | 700 diameter x 230 wide (e) |
| Hose | 100 outside diameter, ISO 1034 nozzle, stowed over a front cradle |
| Aircraft connector | x ~20000 aft of the nose, centreline, belly height ~2300 (e) |

## Animation

- `fbw/gse/asu/hose`, 0 to 1: hose end and nozzle translate from the cradle (0) to the belly connector (1). The model is built in the deployed pose.
