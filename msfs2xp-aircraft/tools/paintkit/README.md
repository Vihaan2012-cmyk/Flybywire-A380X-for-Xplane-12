# Paint kit generator

`make_paintkit.py` builds the FlyByWire A380X (X-Plane 12 port) livery paint
kit: layered PSDs, plain PNG layers, UV-wireframe previews, MAT/NORM notes,
a README, and a tiny example livery. It reads the aircraft install
**read-only** and writes everything to a kit folder outside the X-Plane tree
(default `D:/FlyByWire A380X Paint Kit/`).

## Requirements

```
pip install pillow numpy scipy psd-tools pytoshop
```

(`psd-tools` is only used to spot-check the PSDs this script writes, not by
the generator itself; harmless to skip if you trust the output.)

`pytoshop`'s RLE (packbits) compression needs a compiled Cython extension
that its PyPI wheel does not ship prebuilt on Windows, so this script writes
PSDs with **zip** compression instead (also a standard PSD compression
Photoshop reads natively, no extra dependency).

## Usage

```
python make_paintkit.py
```

Options:
- `--aircraft PATH` -- the installed aircraft folder (default: the Steam
  X-Plane 12 install this was built against).
- `--out PATH` -- the kit output folder (default `D:/FlyByWire A380X Paint Kit`).
- `--only SUBSTR,SUBSTR` -- build only core textures whose file name contains
  one of the given substrings (case-insensitive), e.g. `--only FUSE1,FUSE2`
  for a quick iteration loop. Skips the example livery and README when set,
  since those describe the full kit.

## When to re-run this

Any time the converter changes how it names, tints, or packs exterior
textures (`Converter/src/texture/mod.rs`, `msfs2xp-aircraft/src/model/obj8.rs`),
or the aircraft build's OBJ8 geometry changes (new panel lines, a re-UV'd
sheet, a new exterior part). The script re-derives everything from what is
actually on disk -- the texture-to-OBJ mapping, resolutions, which liveries
provide which sheet -- rather than hard-coding paths, so a re-run after a
converter/aircraft rebuild should just work. Exceptions worth checking by eye
after a big geometry change:
- The fin-detection heuristic in `label_islands()` (tall/narrow/centred/aft)
  assumes the fin's proportions stay roughly what they are today; a
  reworked tail could need its thresholds nudged.
- `CORE_TEXTURES` / `EXTRA_TEXTURES` / `TEXTURE_INFO` are curated lists, not
  auto-discovered -- if a future aircraft revision adds or renames a
  paintable exterior sheet, add it there too (cross-check against what
  `liveries/*/objects/` override, the same way this list was built).

## How it works, briefly

1. `scan_texture_map()` scans every `objects/*.obj` header for its `TEXTURE`
   line, so texture <-> OBJ lookup reflects the actual install, not a
   hard-coded guess.
2. `parse_obj()` reads one OBJ8's `VT` vertex table and its whole `IDX`/`IDX10`
   index buffer as one flat triangle list -- since a file declares exactly
   one `TEXTURE`, the whole buffer is that texture's geometry, independent of
   how many `TRIS`/`ANIM` draw calls slice it up for doors and animated parts.
3. `find_islands()` gets UV islands for free via union-find over shared
   vertex indices: an OBJ8 exporter already duplicates a vertex at every UV
   seam, so connected components of vertex *indices* are exactly UV islands,
   no UV-space geometry needed.
4. `render_wireframe()` draws every triangle edge and bolds edges used by
   only one triangle (island/mesh boundaries). Remember OBJ8 UV v=0 is the
   **bottom** of the texture (`uv_to_px()` flips it) -- get this backwards
   and every wireframe silently renders upside down while still looking
   plausible at a glance.
5. `make_panel_detail()` divides the base texture's luminance by a heavily
   blurred copy of itself, so flat paint reads as white (Multiply = no
   change) and only real panel lines/rivets/AO darken.
6. `build_psd()` assembles the layer stack with `pytoshop.user.nested_layers`
   and writes a real flattened preview into the PSD's merged-image section
   (pytoshop defaults that to blank black, which is only cosmetic to a
   layer-aware app but makes the file look broken to anything that isn't).

## Verifying a change

There is no Photoshop available in this environment. Sanity-check output with
`psd-tools`:

```python
from psd_tools import PSDImage
psd = PSDImage.open("path/to/sheet.psd")
for l in psd:
    print(l.name, l.visible, l.blend_mode)
psd.composite(force=True).save("check.png")
```

and always look at `Preview/<sheet>_preview.png` for at least one changed
sheet -- a misaligned wireframe (wrong v-flip, wrong texture picked up for an
OBJ) is a correctness bug, not a cosmetic one, and it will not throw.
