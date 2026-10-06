# Exterior walkaround object (covers, streamers, pins, chocks)

`make_walkaround.py` builds `a380_walkaround.obj` (OBJ8) plus its texture:
covers/streamers on the pitot, AoA and static probes, engine inlet/exhaust
covers, gear downlock pins and chocks, positioned from the real FlyByWire
A380X MSFS geometry (`a380_exterior.gltf`/`.bin`). It does **not** touch the
plugin, the Rust converter, or the interface (datarefs/commands) that another
agent is wiring up in parallel:

```
dataref  fbw/walkaround/<id>            1 = installed, 0 = removed
command  fbw/walkaround/<id>_toggle
```

### Fitted engine inlet/exhaust covers (2026-09-30)

The 4 inlet and 4 exhaust items were flat discs (`gen_disc`) in the first
version. They are now fitted fabric covers: a domed front face (sagging
more on the lower half, gravity) over a snug skirt that wraps 0.25 m back
along the real cowl, plus a strap/drawstring band at the skirt's edge and 3
"REMOVE BEFORE FLIGHT" streamers (~1 m) hanging from the band's lower arc.
Shape comes from `angular_profile()`: the real lip and cowl cross-sections
are sampled in 40 angle bins around the engine axis from the nacelle/core
vertices themselves (not assumed circular), then interpolated between the
two bands for the skirt. `build_wrap_cover()` in `make_walkaround.py` builds
this as one mesh per cover (~680 triangles; well inside the 3-5k budget) so
it costs one shadow-casting draw call, not a pile of tiny parts. A
`min_clearance()` check measures actual vertex-to-vertex distance from the
built cover back to the real nacelle/core surface it was fitted over, and
the run prints it per cover (see **Corrections made 2026-09-30** below for
the measured numbers -- most engines land in the 0.6-2.2 cm range, all
positive/non-clipping, some tighter than the 1-2 cm target: the linear
interpolation between the two sampled cross-sections doesn't always track a
non-linear real taper). The old flat-disc code path is kept as an
exception-fallback inside `_fitted_cover_item()` so the object always builds
even if a profile fit degenerates on some future glTF revision.

Texture: two new atlas tiles, `fabric_plain` and `fabric_print` (dark red
weave + seam lines baked straight into the albedo -- no separate normal map
in this pass; see the source comment for why). `fabric_print` also carries
the white REMOVE BEFORE FLIGHT text, applied as a small flat patch on the
cover's face rather than wrapped around the dome (a polar UV unwrap would
distort the text badly at this budget). The other 4 tiles (`cover_red`,
`cover_red_text`, `streamer`, `chock`) are unchanged -- pitot, AoA, static,
gear pin and chock items still use them exactly as before.

**Not finished this pass**: Blender preview renders of the covers over the
nacelle were attempted (`--background` script importing the OBJ8 text
directly plus the same glTF loader this tool uses) and do produce PNGs, but
the per-engine vertex selection in that *preview script* (not in
`make_walkaround.py` itself) picked up vertices from the whole airframe
instead of just one engine, so the camera framing is wrong. Not debugged in
the time available -- see the task's final report for what to do next.

### Cover shape fixed to actually wrap the lip / centre-body (2026-09-30, F7)

The first pass above built a cap and a skirt as two separate, disconnected
pieces: the cap sat recessed inside the inlet lip, and the skirt was a thin
band ~0.3 m further aft, with the real grey lip exposed between them (the
exhaust plug missed the visible centre-body the same way). Root causes and
fixes, all in `build_wrap_cover()` plus the exhaust branch of
`extract_engines()`:

- The cap's rim radius was the *mean* radius of a z-band that blends the
  outer cowl skin with the smaller-radius inner duct wall -- switched to
  `angular_profile(..., agg="max")`, which picks the outer envelope instead.
- The skirt was a straight 2-point lerp between the lip and cowl profiles,
  which cut inside a non-linear real taper; it's now `n_wrap_rings`
  (default 4) rings, each freshly sampled from the real geometry at that
  depth.
- The exhaust plug was anchored on the mean of an arbitrary window along
  the core-duct material, landing well short of the real tip; it's now
  anchored on the nacelle's own aft-most point (mirroring how the inlet
  uses the nacelle's own forward-most point) -- which also turned out to be
  the actual visible centre-body cone, not the engine-cores/reverser
  materials. The exhaust skirt depth is no longer the fixed 0.25 m (too
  short for this near-point tip); it's walked out per engine until the
  nacelle's own radius reaches the surrounding core/reverser cowl's radius.
- `build_wrap_cover()` now takes `skirt_depth` as a per-call override
  (`_fitted_cover_item(..., skirt_depth=...)`) for this reason.
- `clearance` default lowered from 0.032 m to 0.016 m now that the
  outer-envelope sampling is reliable (the old value was partly
  compensating for the mean-based under-sampling).

Current clearance: inlets 1.3-1.7 cm (within the 1-3 cm target); exhausts
1.0 cm (engine 1, no reverser) to 3.1-3.7 cm (engines 2-4, with a
reverser -- a little over target, not fixed further for time). Triangle
count per cover: 760 (still well inside the 3-5k budget). Verified visually
via `render_preview.py` (see below) -- one continuous red surface from cap
through skirt to band, no grey nacelle visible in between.

## Running it

```
python make_walkaround.py \
  --gltf "D:/Microsoft Flight Simulator 2020/Microsoft Flight Simulator 2020 Packages/Community/flybywire-aircraft-a380-842/SimObjects/AirPlanes/FlyByWire_A380_842/model/a380_exterior.gltf" \
  --out  "D:/A380/fbw-build/walkaround-stage" \
  --gear-acf "D:/Steam Games/steamapps/common/X-Plane 12/Aircraft/FlyByWire A380X/FlyByWire A380X.acf"
```

### Corrections made at integration (2026-09-29)

Three defects in the first version, fixed in the script; the item table and
gear sections further down describe that first version:

- **Removed items never disappeared.** Each item was wrapped in
  `ANIM_show 0.5 1.5`; OBJ8 geometry starts out visible and `ANIM_show` only
  ever shows it. Now `ANIM_hide -0.5 0.5 fbw/walkaround/<id>`.
- **Chocks and gear pins floated inside the wheel wells.** They were placed
  on the glTF's own tyre vertices, but that rest pose has every leg
  *retracted* (the nose leg's `c_gear` clip swings it 108 degrees down from
  there; the wing gear swings in sideways). They now come from X-Plane's own
  gear in the `.acf` (`--gear-acf`, read only): contact point x/z and the
  uncompressed tyre bottom (`_gear_y - _leg_len - _tire_radius`). Evaluated
  through its animation, the extended nose wheel lands at z -30.07 m, bottom
  -4.67 m; the `.acf` nose contact is z -30.22 m, bottom -4.62 m -- they
  agree, so the physics is the ground truth.
- **Chocks sat below the tarmac on a loaded aircraft.** On the ground the
  struts compress and the ground rises toward the airframe by the tyre
  deflection, so each chock rides `sim/flightmodel2/gear/tire_vertical_deflection_mtr[n]`
  up by exactly that much. Each wheel set's width and axle span are measured
  from its own tyre nodes (`WHEEL_SETS`), since distances between a bogie's
  own tyres are the same in any gear pose.

`--acf` now names the object bare (`a380_walkaround.obj`, like every other
entry -- X-Plane already looks in `objects/`); it used to write
`objects/a380_walkaround.obj`.

Add `--acf "<path>.acf"` to also append one `_obja` entry pointing at the
generated object (off by default; see **Click-test finding** below before
turning this on for the installed aircraft). Requires Python 3 with numpy and
Pillow (both present; no scipy/matplotlib used).

Output, all under `--out`:

- `objects/a380_walkaround.obj` -- the OBJ8 object.
- `objects/a380_walkaround.png` -- its texture atlas (red cover fabric, red
  "REMOVE BEFORE FLIGHT" panel, streamer strip, yellow/black chock stripes).
- `walkaround_top.png`, `walkaround_side.png` -- a coarse aircraft outline
  (sampled straight from the glTF, not used for placement) with every item's
  position boxed and labelled, for a by-eye placement check. Regenerated
  every run.

The script also prints an item table (position, the model feature it sits on,
and where that position came from) and a verification pass (point/index
counts, `ANIM_begin`/`ANIM_end` balance, index range) on every run.

## Coordinate frame

Positions are extracted in glTF/world space (X, Y up, Z forward -- the glTF
convention) by composing each node's TRS up its parent chain, exactly like
`src/model/glb.rs::load_glb` (reimplemented here in plain Python/numpy so
this tool does not need to touch the Rust converter). The OBJ8 write step
applies the same flip `src/model/obj8.rs::write_obj8_animated` uses:
`x_obj = -x, y_obj = y, z_obj = -z`, scale 1, offset_y 0 (the object is meant
to be attached at the .acf reference point with zero offset, per the task).

**Frame verified against the installed aircraft**: summing every vertex this
script decodes under material `A380_EXTERIOR_TYRES` gives exactly
**764,781** vertices -- the same number as the `POINT_COUNTS` line in the
real, installed
`D:/Steam Games/.../FlyByWire A380X/objects/a380_exterior_042.obj` (the
tyres object, read-only reference). That is an exact match of the whole
tyre geometry's vertex count between this script's independent glTF decode
and the converter's real output, not just a bounding-box check.

## Item table (glTF/world frame, metres)

| id | position (x, y, z) | feature | source |
|---|---|---|---|
| pitot_cover_1 | (-2.425, 1.285, 31.441) | MFP1 probe body | node `MFP1_BASE`, material `A380X_ACCESSORIES` |
| pitot_cover_2 | (-2.445, 0.816, 31.446) | MFP2 probe body | node `MFP2_BASE` |
| pitot_cover_3 | (2.426, 1.284, 31.441) | MFP3 probe body | node `MFP3_BASE` |
| aoa_cover_1 | (-2.492, 1.293, 31.416) | MFP1 vane | node `MFP1_FIN` |
| aoa_cover_2 | (-2.511, 0.813, 31.419) | MFP2 vane | node `MFP2_FIN` |
| aoa_cover_3 | (2.493, 1.293, 31.416) | MFP3 vane | node `MFP3_FIN` |
| static_covers | 4 pads: (±2.485,-0.587,26.079) + (±1.813,0.982,32.986) | primary + standby static ports, both sides | nodes `STATIC_PORT`, `STBY_STATIC` |
| eng_inlet_cover_1 | (25.77,-0.55,4.70) r=1.68 | engine 1 nacelle lip | material `A380_EXTERIOR_ENG_LH`, forward ring fit |
| eng_inlet_cover_2 | (14.87,-0.79,12.72) r=1.60 | engine 2 nacelle lip | `A380_EXTERIOR_ENG_LH` |
| eng_inlet_cover_3 | (-14.88,-0.79,12.72) r=1.60 | engine 3 nacelle lip | `A380_EXTERIOR_ENG_RH` |
| eng_inlet_cover_4 | (-25.81,-0.39,4.72) r=1.66 | engine 4 nacelle lip | `A380_EXTERIOR_ENG_RH` |
| eng_exhaust_cover_1 | tip z=-3.54 | engine 1 nozzle + centre-body | material `A380_EXTERIOR_ENG_LH`, aft-most point (F7: was `ENG_CORES_LH`, see "Cover shape fixed" above) |
| eng_exhaust_cover_2 | tip z~4.4 (engine 2's own aft-most point) | engine 2 nozzle + centre-body | `A380_EXTERIOR_ENG_LH` (F7: was `REVERSER_CORE`) |
| eng_exhaust_cover_3 | tip z~4.4 (engine 3's own aft-most point) | engine 3 nozzle + centre-body | `A380_EXTERIOR_ENG_RH` (F7: was `REVERSER_CORE`) |
| eng_exhaust_cover_4 | tip z=-3.53 | engine 4 nozzle + centre-body | `A380_EXTERIOR_ENG_RH` (F7: was `ENG_CORES_RH`) |
| gear_pin_nose | (-0.12, 0.29, 33.01) | nose leg, above axle | `A380_EXTERIOR_TYRES` cluster |
| gear_pin_lwing | (2.42,-0.86, 2.35) | left wing leg, above axle | `A380_EXTERIOR_TYRES` cluster |
| gear_pin_rwing | (-2.42,-0.86, 2.35) | right wing leg, above axle | `A380_EXTERIOR_TYRES` cluster |
| gear_pin_lbody | (2.62,-0.77,-3.41) | left body leg, above axle | `A380_EXTERIOR_TYRES` cluster |
| gear_pin_rbody | (-2.61,-0.78,-3.05) | right body leg, above axle | `A380_EXTERIOR_TYRES` cluster |
| chocks_nose | fore/aft pair at z 32.15/33.86, y=-0.93 | nose tyre contact patch | same cluster, own min-Y |
| chocks_lwing | fore/aft pair at z 0.77/3.92, y=-2.21 | left wing tyre contact patch | same cluster |
| chocks_rwing | fore/aft pair at z 0.77/3.92, y=-2.21 | right wing tyre contact patch | same cluster |
| chocks_lbody | fore/aft pair at z -5.95/-1.03, y=-1.95 | left body tyre contact patch | same cluster |
| chocks_rbody | fore/aft pair at z -5.95/-1.03, y=-1.95 | right body tyre contact patch | same cluster |

Exact numbers (the script computes these at runtime, they are not
hard-coded): re-run and read the printed item table.

## How each group was found

**Pitot + AoA (multi-function probes).** The task flagged that the real
pitot heads and AoA vanes might not exist as separate named nodes and told
me to search near the nose. They turned out to be named directly: three
`MFP{1,2,3}_BASE` + `MFP{1,2,3}_FIN` node pairs (plus `_SCREWS`, not used) at
around glTF z=31.4, x=±2.4 -- MFP = **Multi-Function Probe**, the real A380's
combined pitot/AoA/TAT sensor (the A380 does not carry separate pitot heads
and AoA vanes the way older Airbus types do; each MFP replaces both). The
`_BASE` mesh (the tube) is used for `pitot_cover_N`, the `_FIN` mesh (a
small flat blade, sized very differently from `_BASE` -- consistent with a
vane rather than a probe body) for `aoa_cover_N`. Each item's cylinder/pad is
sized and oriented from a principal-component fit (`principal_axis()`) of
that mesh's own vertices, not a guessed local axis.

A fourth, separate `STBY_PITOT` node also exists (a real standby pitot
feeding the ISIS) but there is no 4th slot in the fixed interface
(`pitot_cover_1..3` only) -- it is left uncovered and reported, not invented
a home for.

**Static ports.** `STATIC_PORT` and `STBY_STATIC` each bundle a mirrored
left+right pair of ports in one mesh (split here by the sign of X). Both
pairs (4 pads total) are grouped under the single `static_covers` item,
matching the interface's one dataref for the whole static-port set.

**Engines.** `VFX_Contrails_Engine_1..4` nodes are explicitly numbered 1-4 in
the asset and give a clean per-engine reference point; every nacelle-material
vertex is assigned to its nearest engine by (X,Z) distance. Cross-checks,
both from the asset itself:
- `engines.cfg`'s `Engine.0..3` lateral offsets are -84, -47.5, +47.5, +84 ft
  -- left outboard, left inboard, right inboard, right outboard, in that
  order -- exactly the interface's 1=left outboard .. 4=right outboard.
  `VFX_Contrails_Engine_1..4`'s |x| ordering and sign match that one-to-one,
  which is also how **+X was confirmed to be the left side** of this glTF.
- Only engines 2 and 3 (the inboard pair) have `A380_EXTERIOR_REVERSER_CORE`
  geometry; engines 1 and 4 (outboard) do not. That matches the real A380,
  which only fits reversers to its inboard engines -- a sanity check that
  the per-engine clustering is grouping the right vertices together, not an
  invented fact used for placement.

Inlet lip = a circle fit to the forward-most 3% (by Z) of each engine's
`A380_EXTERIOR_ENG_LH`/`_RH` nacelle vertices, pulled back ~6 cm so the cover
disc sits slightly inside the lip as asked. Exhaust = a circle fit to the
`A380_EXTERIOR_REVERSER_CORE` ring (engines 2/3) or the largest ring in the
aft 22% of the `A380_EXTERIOR_ENG_CORES_LH/RH` cluster (engines 1/4, which
have no reverser geometry to fit instead).

**Gear/tyres.** All `A380_EXTERIOR_TYRES` vertices are grouped by their
source node, then nodes within 2.2 m of each other (in X/Z) are merged into
one wheel-set cluster with a union-find (same-leg wheels, single or dual,
sit under ~2.0 m apart on this asset; different legs are 3+ m apart -- this
was checked against the actual inter-node distances during development, not
guessed). The forward-most cluster is the nose; the rest split into
left/right by the sign of X and into wing/body by Z (the more-aft cluster on
each side is body gear -- it sits further from the nose than wing gear on
this airframe, confirmed from the clusters' own centroids: body z≈-3.8,
wing z≈+2.3). Total vertex count across all 5 clusters equals the installed
tyre object's `POINT_COUNTS` exactly (see **Coordinate frame** above).

No downlock, drag-brace, or other lock-point geometry could be identified
with confidence for *any* leg by name (the wing-gear top-pivot nodes,
`WLG_MAIN_LEG.L`/`.L_1`, sit up at the wing attachment near x=±6.1, nowhere
near the wheels, and no equivalent node exists for the body gear at all), so
every `gear_pin_*` uses the task's own fallback: on the leg centreline
(the wheel cluster's own X/Z centroid), a small height above the axle
(midpoint of the cluster's Y bounding box + 0.28 m generic offset).

Chocks use each wheel cluster's **own** lowest tyre vertex as that wheel
set's local ground plane (per the task: derive tyre bottoms per wheel set,
not one shared ground Y) -- nose, body and wing ground contact are at
noticeably different Y because the model's rest pose is not a single flat
"parked" pose but each leg's own geometry, exactly as instructed.

## Generic dimensions (not modeled, chosen to look right)

- Streamer ribbon: 0.4-0.6 m long, 0.06 m wide.
- Chock wedge: 0.28 m long x 0.22 m wide x 0.16 m tall.
- Gear pin: 0.22 m tall, 0.018 m radius cylinder, plus its own streamer.
- Pitot sleeve radius is the probe's own PCA radius x1.15 (a loose-fitting
  sock), never invented outright.

## Click-test finding (for the lead)

**X-Plane's own documentation says manipulators are cockpit-only.** Both
developer.x-plane.com's [Manipulators](https://developer.x-plane.com/article/manipulators/)
article ("Manipulations can only be used on cockpit objects!") and the
[OBJ8 file format spec](https://developer.x-plane.com/article/obj8-file-format-specification/)
("Only triangles in the cockpit object can be in the manipulation plane")
restrict `ATTR_manip_*` to the aircraft's designated cockpit object in the
.acf. An `_obja` entry like the one `--acf` appends here is an ordinary
exterior attachment, not the cockpit object -- so a plain mouse click on
this walkaround OBJ8 in the external/free-camera view is **not guaranteed
to fire `fbw/walkaround/<id>_toggle` through X-Plane's native manipulator
path**, even though the OBJ8 syntax itself (`ANIM_show`/`ATTR_manip_command`)
is correct and matches how the spec documents it.

This also matches the wider ecosystem: X-Plane 12 has no first-party
walkaround mode (`C` gives a flying Free Camera, not a body); third-party
add-ons like VFRScenery's WalkAround plugin exist specifically to add
exterior interaction, which strongly implies they do their own screen-ray
hit-testing via the XPLM SDK rather than relying on native manipulators
firing outside the cockpit. Nothing found in the X-Plane 12 release notes
(12.0 through 12.4) changes this.

**Recommendation for whoever wires up the interaction**: keep the
`ANIM_show`/`ATTR_manip_command` pairs in this OBJ8 (they cost nothing and
match the spec), but don't rely on them alone. The FBW plugin most likely
needs to either do its own mouse-ray hit test against the known item
positions (this script's item table gives exact coordinates) and call the
matching `_toggle` command itself, or expose the toggles through a 2D
UI/EFB page instead of a 3D click. This is a finding to plan around, not
something this script can fix by itself -- it only builds the geometry and
the (spec-correct) manipulator tags.

## What could not be placed

- A 4th, standby pitot probe (`STBY_PITOT`) exists in the model but has no
  slot in the fixed interface; left uncovered (see above).

Everything else in the fixed item list (21 items total: 3 pitot + 3 AoA + 1
static + 4 inlet + 4 exhaust + 5 pins + 5 chocks) was placed from real model
geometry.
