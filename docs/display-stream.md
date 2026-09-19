# Display command stream

The contract between the JavaScript side (DOM, SVG, CSS, Canvas2D: `src/js/dom/`)
and the native renderer (`src/display/`). One stream per screen. The JS side
sends a new stream only when something on that screen changed. The renderer
keeps the last stream and draws it every frame.

## Submitting

```js
__host.submitDisplay(screen /* string id, e.g. "PFD_L" */, ops /* Float64Array */, strings /* string[] */)
```

- `ops` is a flat list of opcodes followed by their operands.
- Strings (text, font family names, image URLs) are indices into `strings`.
- Colours are four numbers, r g b a, each 0..1 and not premultiplied.
- Coordinates are CSS pixels of the screen's viewport (its `<svg>`/root element
  size). The renderer scales them to the device's pixels.

```js
__host.measureText(fontFamily, fontSizePx, text) -> width   // CSS px, synchronous
__host.fontMetrics(fontFamily, fontSizePx) -> [ascent, descent] // CSS px, both positive
__host.screenSize(screen) -> [width, height]  // CSS px the renderer expects
```

Mouse input comes back as `__screenEvent(screen, type /* "down"|"up"|"move"|"wheel" */, x, y, button, delta)`,
in the same CSS pixels. The DOM side turns it into DOM events.

## Opcodes

| op | name | operands |
|---:|---|---|
| 1 | SAVE | none |
| 2 | RESTORE | none |
| 3 | TRANSFORM | a b c d e f (multiplies the current matrix, as in canvas `transform`) |
| 4 | SET_TRANSFORM | a b c d e f |
| 5 | GLOBAL_ALPHA | alpha (multiplies) |
| 6 | CLIP_RECT | x y w h (intersects with the current clip) |
| 10 | BEGIN_PATH | none |
| 11 | MOVE_TO | x y |
| 12 | LINE_TO | x y |
| 13 | QUAD_TO | cx cy x y |
| 14 | CUBIC_TO | c1x c1y c2x c2y x y |
| 15 | ARC | cx cy r start end ccw(0/1) |
| 16 | ELLIPSE | cx cy rx ry rotation start end ccw |
| 17 | RECT | x y w h |
| 18 | CLOSE_PATH | none |
| 20 | FILL | r g b a rule(0 nonzero, 1 evenodd) |
| 21 | STROKE | r g b a width cap(0 butt,1 round,2 square) join(0 miter,1 round,2 bevel) miterLimit dashCount dash... dashOffset |
| 22 | CLIP_PATH | rule (intersects the current clip with the current path) |
| 30 | TEXT | string fontString size weight(100..900) italic(0/1) x y align(0 left,1 center,2 right) baseline(0 alphabetic,1 middle,2 top,3 bottom,4 hanging) r g b a strokeWidth(0 = fill only) sr sg sb sa |
| 40 | IMAGE | urlString sx sy sw sh dx dy dw dh |
| 50 | LINEAR_GRADIENT_FILL | x0 y0 x1 y1 stopCount (offset r g b a)... rule |
| 60 | NATIVE_IMAGE | idString dx dy dw dh |

SVG and CSS are resolved on the JS side into these ops. The renderer knows
nothing about the DOM. `fontString` is a family name, which the renderer maps
to a font file (the FBW instrument fonts, e.g. Ecam, ...). `urlString` is
resolved relative to the instrument's folder.

## NATIVE_IMAGE (map data)

Draws an image the plugin makes natively, looked up by id, into the
rectangle (current transform, clip and global alpha apply, as for IMAGE).
The image is straight RGBA, row 0 at the top; the renderer re-uploads it
only when its `generation` changes.

```rust
crate::mapdata::plugin::native_image(id: &str) -> Option<Arc<NativeImage>>
// NativeImage { width: u32, height: u32, generation: u64, rgba: Vec<u8> }
```

Ids: `TERRONND_L` and `TERRONND_R`, FlyByWire's terronnd gauge for the
captain's and first officer's ND (docs/map-data.md).

`WXR_L` and `WXR_R` are the weather radar picture (docs/wxr.md), composed
into the same NDs in the same "under the HTML gauges" slot as `TERRONND_L`/
`TERRONND_R`, even though panel.cfg has no gauge line for it to come from
(FlyByWire ships none -- `src/js/msfs/mod.rs`'s `Cockpit::new` adds it
itself, one `WXR_<side>` per `TERRONND_<side>` it finds). The two ids are
mutually exclusive in practice (the EFIS CP's TERR/WXR overlay selector),
not by construction: `crate::wxr::native_image` returns `None` whenever this
ND is not currently showing WXR, so only one of the two ever draws anything.

Why: in MSFS a panel.cfg texture holds several gauges drawn in order, and
each ND is `htmlgauge00=WasmInstrument/WasmInstrument.html?wasm_module=
terronnd.wasm&wasm_gauge=terronnd,0,0,768,1024,L` then `htmlgauge01=A380X/ND/
nd.html?Index=1&duID=1, 0,0,768,1024` (panel.cfg `[VCockpit07]`, lines 70-71;
`[VCockpit08]` lines 78-79 with `,R` and `Index=2`). terronnd draws an opaque
black screen with the terrain image over it (terronnd `displaybase.cpp`
`render`), and nd.html draws over that with a transparent background. So
the op goes first in the SCREEN_DU_NDL/NDR streams, `60 TERRONND_L 0 0 768
1024`, emitted by whatever composes a screen's panel.cfg gauges, and the
nd.html stream follows. The image is the gauge's whole output (black when
unpowered, the terrain dimmed by its `LIGHT POTENTIOMETER` as terronnd does),
so the renderer needs nothing else from it. Before the terrain worker has
drawn, `native_image` returns `None`: draw nothing.
