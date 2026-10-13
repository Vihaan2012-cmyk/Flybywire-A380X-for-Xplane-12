# ULD set: LD3 dolly, AKE container, PMC pallet on dolly

## Sources

- IATA ULD Technical Manual dimensions for the LD3 (AKE) and PMC (96 × 125 in pallet), taken from the widely published spec tables: AKE base 1534 × 1562 mm, 2007 mm wide at the top, 1626 mm high, tare about 85 kg, max gross 1588 kg. PMC 3175 × 2438 mm, contoured net load about 1600 mm high.
- No web search and no photos were done in the time box. Dolly and pallet-dolly dimensions are **(e)** estimates from typical industry units (deck about 1.7 × 1.66 m for the LD3 dolly, 3.4 × 2.6 m for the pallet dolly).
- Photos, if added later, go in `E:/fbw-gse/refs/uld_set/` (empty now).

## Frame

- Millimetres. *x*: along the vehicle, towards the drawbar. *y*: lateral, positive to the right looking from the drawbar end. *z*: up from ground.
- The three sub-assemblies sit side by side, each with its own name prefix: `ld3dolly_*` at y = 0, `ake_*` at y = 2500, `pmcdolly_*` and `pmc_*` at y = -3300. Each object carries a `gse_part` custom property with its prefix.

## Key dimensions

| Item | Value (mm) |
|---|---|
| AKE base footprint | 1534 deep × 1562 wide |
| AKE top width / height | 2007 / 1626 |
| AKE door | 1520 wide fabric curtain on the +x face, two straps |
| LD3 dolly deck | 1700 × 1660, deck z 330-400 (e) |
| LD3 dolly drawbar | 780 long (e) |
| PMC pallet | 3175 × 2438, plate 25 thick |
| PMC load | contoured, 1560 above plate (net lattice) |
| PMC dolly deck | 3400 × 2600, deck z 360-430 (e) |
| Wheels | 250 mm (LD3 dolly), 300 mm (pallet dolly) (e) |

## Liveries

Three own-artwork variants (serial plus 3-letter owner code): a = EK, b = QR, c = LH. Decals are drawn by `draw_liveries()` into `E:/fbw-gse/out/uld_set/liveries/` (`ake_*`, `band_*`, `pmc_*`). The model uses variant a. `liveries(out)` returns `{}` for now.
