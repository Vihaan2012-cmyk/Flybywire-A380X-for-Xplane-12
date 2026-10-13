# Lower-deck container/pallet loader (TLD TXL-838 / JBT Commander 15i class)

## Sources

- Generic spec of this class (two scissor platforms, roller beds, side cab, outriggers): TLD and JBT product pages; no web fetch of the brochures succeeded in this session, so dimensions below are class-typical estimates (p), not measured drawings.
- IAG Cargo A380-800 page: forward lower-deck cargo door 283 x 173 cm, aft door 254 x 173 cm (width x height).
- Airbus A380 Aircraft Characteristics (ac_a380_1223.pdf, airport and maintenance planning): door clearance figures for forward and aft cargo doors hold the sill heights. NOT read in this session; the model assumes a bridge deck that raises to about 2.8 m, which still needs checking against that document.

Photos: none downloaded; folder `E:/fbw-gse/refs/ldcl_loader/` is reserved.

## Frame

- Units are millimetres.
- *x*: from the rear end towards the bridge platform (aircraft side).
- *y*: from the centreline, positive to the right when looking from the bridge end.
- *z*: up from the ground.

## Key dimensions (p)

| Item | Value |
|---|---|
| Overall length | 7660 |
| Overall width incl. outriggers | 2720 |
| Chassis width | 2500 |
| Rear platform | x 1250-4850, deck width 2300 |
| Bridge platform | x 5000-7300 (+ lip to 7540) |
| Deck height lowered | 1110 (roller top about 1145) |
| Bridge travel | 1750 |
| Rear travel | 1650 |
| Wheelbase | 4600 (axles x 1000 and 5600), track 2000 |
| Tyres | 760 diameter x 270 |
| Roller pitch | 95, two lanes of 960 |
| Cab | x 5000-5950, right side, roof z 1520 |

## Animation

- `fbw/gse/ldcl/bridge` 0..1: bridge deck, rails, scissors and lip rise 1.75 m.
- `fbw/gse/ldcl/rear` 0..1: rear deck, rails and scissors rise 1.65 m.
- Scissor legs translate with the deck rather than scissoring.
