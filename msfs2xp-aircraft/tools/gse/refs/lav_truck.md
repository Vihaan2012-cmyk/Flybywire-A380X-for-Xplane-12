# Lavatory service truck (light-truck chassis, Wollard/TLD/Mallaghan LV class)

## Sources

- Airbus A380 Aircraft Characteristics (airbus.com, Dec 2023 rev, section 5-4-10; local copy E:/fbw-gse/refs/lav_truck/ac.pdf): waste water ground service panel, access door 171AL, 53.31 m aft of nose, 0.26 m left of centreline, 3.40 m above ground. Connectors: drain ISO 17775 4 in, rinse/fill ISO 17775 1 in.
- Truck dimensions are class estimates for a light cab-over truck with a 4-5 m3 waste tank and a 1-2 m3 flush tank, not read off a single brochure. No manufacturer photos were downloaded.

## Frame

- Units are millimetres.
- *x*: from the rear end towards the cab (origin offset X0 = 2800 in the script).
- *y*: from the centreline, positive to the right when looking from the cab end.
- *z*: up from the ground.

## Key dimensions (estimates)

| Item | Value |
|---|---|
| Body length without platform | 5420 |
| Platform reach behind the rear end | 1420 |
| Width | 2150 |
| Cab roof | 2350 |
| Wheelbase | 3100 |
| Track | 1800 |
| Tyre diameter | 780 |
| Waste tank | diameter 1200, length 3000, axis 1500 above ground |
| Tank-top walkway | 2110 |
| Platform floor, stowed | 1250 |
| Platform travel | 1150 (floor at 2400 when raised) |
| Hose coupling, stowed | 1215 |
| Hose coupling, raised | 3415 (panel at 3400) |

## Animation

- `fbw/gse/lav/platform` 0..1: platform floor, rails and scissor legs translate up 1.15 m.
- `fbw/gse/lav/hose` 0..1: hose and coupling translate up 2.2 m.
