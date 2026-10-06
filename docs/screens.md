# Cockpit screens (src/display)

How the display command streams (docs/display-stream.md) reach X-Plane 12's
cockpit. Code: `src/display/`, host functions in `src/js_bridge.rs`
(`register_display`).

## Devices

One X-Plane cockpit device per screen, made with `XPLMCreateAvionicsEx`
(XPLMDisplay.h, XPLM410, X-Plane 12.1+) when the plugin is enabled and
destroyed when it is disabled. The device id is the panel.cfg texture name
without `$`, exactly as the converter writes `ATTR_cockpit_device`:

| device id | px | dimming (LIGHT POTENTIOMETER, buses) |
|---|---|---|
| SCREEN_DU_PFDL | 768x1024 | 88, DC ESS |
| SCREEN_DU_NDL | 768x1024 | 89, DC ESS or DC 1 |
| SCREEN_DU_PFDR | 768x1024 | 90, DC 2 |
| SCREEN_DU_NDR | 768x1024 | 91, DC 1 or DC 2 |
| SCREEN_DU_EWD | 768x1024 | 92, DC ESS |
| SCREEN_DU_SD | 768x1024 | 93, DC 2 |
| SCREEN_DU_MFD | 1646x1024 | x 0-768: 98, DC ESS or DC 1; x 878-1646: 99, DC 1 or DC 2 |
| FCU | 2560x1280 | 87, DC ESS or DC 2 |
| SCREEN_DU_RMP_1/2/3 | 1664x1024 | 80/81/82, DC ESS/DC ESS/DC 1 |
| SCREEN_ISIS_1, Clock, RTPI, BAT | 512², 256², 338x128, 256x128 | none: they dim in their own drawing |

Sources: panel.cfg (sizes), A380_COCKPIT.xml component `Screens` and
`Standby_Indicator`, model/behaviour/rmp.xml.

Each device is drawn every frame (`drawOnDemand` 0): the stream is
tessellated once when it arrives, and the draw callback only uploads a new
mesh when there is one and replays its batches. Devices not bound to the
loaded aircraft (`XPLMIsAvionicsBound`) are skipped.

### Brightness

FlyByWire's model dims each screen mesh by `LIGHT POTENTIOMETER:n` (percent
over 100) times its `EMISSIVE_CODE` (the bus-powered L:vars above). The
renderer does the same per region, as a black quad at `1 - brightness`
drawn over the region last, so the two MFDs on one texture dim separately.
The device's brightness callback returns 1 and ignores X-Plane's rheostat,
ambient and bus values. A potentiometer nothing has written reads 0 (dark),
as it does for the instruments, which go to standby at 0.

The RMPs set their potentiometers with `K:LIGHT_POTENTIOMETER_SET <n>
<percent>` (RmpStateController.ts:102); whoever handles key events has to
store `percent / 100` into `LIGHT POTENTIOMETER:<n>`.

### What the converter writes

- `ATTR_cockpit_device <id> <bus> <lighting channel> <auto adjust>`: the
  OBJ8 spec makes the bus a bitfield of X-Plane bus indices the device takes
  power from, the lighting channel an `instrument_brightness_ratio` index,
  and auto adjust a daylight boost. XPLMDisplay.h documents the brightness
  callback as setting "the absolute brightness of the device's screen" and
  passes rheostat, ambient and bus ratio to it as inputs -- but it also says
  of the callback field itself, "Set to NULL to use X-Plane's default
  behaviour" (`XPLMCreateAvionics_t::brightnessCallback`). Ours is never
  NULL (it returns 1 unconditionally, see Brightness above), so whatever
  X-Plane's own bus/rheostat-driven default does with an unpowered bus is
  exactly the behaviour a non-NULL callback replaces; the bus bitfield is
  otherwise unused by the callback and has nothing left to blank. `0 0 0` is
  correct as written: no X-Plane bus, no daylight boost, FlyByWire decides
  power and brightness entirely.
- Mouse: XPLMDisplay.h, "you must add a `ATTR_manip_device` manipulator on
  top of your screen in order to receive mouse events from the 3D cockpit".
  The converter writes none yet. Each screen mesh (both SCREEN_DU_MFD
  meshes too) needs, after its `ATTR_cockpit_device` and before its `TRIS`,
  `ATTR_manip_device hand <id> <tooltip>` (OBJ8 spec, new in 12.1.0: same
  shape and UV mapping as the screen, and also tagged with
  `ATTR_cockpit_device`).

## Input

Screen touch (left, and right as button 2), drag, cursor movement and the
vertical wheel become `__screenEvent(screen, "down"|"move"|"up"|"wheel", x,
y, button, delta)` in the screen's CSS pixels (texel centre, y flipped from
X-Plane's bottom-left origin). A wheel notch is `delta` -100 away from the
user, +100 towards, as a DOM `deltaY`. Events queue (at most 256) and reach
the scripts at the start of each tick.

## Drawing

- **Tessellation** (lyon): fills nonzero and evenodd; strokes with butt,
  round and square caps, miter (with limit), round and bevel joins, dashes
  with offset, stroked in user space so a scaled transform scales the line
  as on a canvas. Arcs, ellipses and béziers are flattened to 0.1 device
  pixels through the current transform (Wang's formula for curves, the
  chord error for arcs), so a bigger device gets more segments.
- **Batching:** flat colour samples an opaque block of the glyph atlas, so
  paths and text share one texture and consecutive draws in one clip are
  one draw call. Images and gradients (256-texel ramps, one texture per
  screen) are their own batches. A PFD-like stream is 7 draw calls; 9000
  mixed strokes, labels and symbols are 1.
- **Clips:** axis-aligned clip rectangles are scissor boxes; any other clip
  is a stencil clip path (seven bits count nested clip paths, the eighth
  marks pixels a translucent stroke already covered, so a self-overlapping
  translucent stroke is painted once, as on a canvas).
- **Transforms, alpha** are baked into the vertices.
- **Anti-aliasing:** 4x multisampling of one 1024x1024 RGBA8 + D24S8
  framebuffer shared by every screen (32 MB), drawn in tiles and resolved
  into each device with `glBlitFramebuffer`. Chosen over analytic edge
  fringes because it anti-aliases stencil clip paths and the
  once-per-pixel stroke marking consistently, with no extra geometry.
  Measured against tiny-skia's analytic coverage on the golden shapes, the
  mean error per pixel is 1.64 at 1x, 0.59 at 4x, 0.37 at 8x and 0.36 at
  16x: 4x takes most of the gain at half 8x's memory (`gl::SAMPLES`). Text
  is already anti-aliased in the atlas (quarter-pixel horizontal
  positioning). Without framebuffer objects, or when X-Plane's own target
  is multisampled, the mesh is drawn straight into the device's target.
- **OpenGL under Vulkan** ("Plugin Guidance for OpenGL Drawing"): blending
  and texturing through `XPLMSetGraphicsState`, 2-D textures through
  `XPLMBindTexture2d` and `XPLMGenerateTextureNumbers`; shader, VAO and VBO
  unbound, only the fixed-function vertex array left enabled, matrices,
  scissor, stencil, colour mask and blend function put back; the previous
  FBO is re-bound from `sim/graphics/view/current_gl_fbo`; framebuffer
  completeness checked once at creation; no `glGetError`.

## Fonts

Loaded at run time from `<aircraft>/html_ui/Fonts/fbw-a380x/` (the plugin
lives in `<aircraft>/plugins/<plugin>/`), rasterised on demand into a
glyph atlas (1024², growing to 4096², started again when full). The family
table is FlyByWire's `@font-face` rules (`text::FACES`, with file:line), so
`Ecam` on SCREEN_ISIS_1 is the ISIS font and `Digital` is a different file
on the FCU and on BAT. `measureText` and drawing use the same layout
(advances plus `kern`); the test `text_is_drawn_where_it_is_measured`
checks a right-aligned run lands where the measured width puts it.

**Install:** copy these from the MSFS package
`flybywire-aircraft-a380-842/html_ui/Fonts/fbw-a380x/` (built from
`fbw-a380x/src/base/flybywire-aircraft-a380-842/html_ui/Fonts/fbw-a380x/`
in FlyByWire's repository) to `<aircraft>/html_ui/Fonts/fbw-a380x/`; the
EFB subfolder is not needed.

| file | family | licence (font name table; repository is GPL-3.0) |
|---|---|---|
| FBW-Display-EIS-A380.ttf, -SlashedZero.ttf | Ecam, FBW-Display-EIS-A380-SlashedZero | "Licenced under GPLv3" |
| ISISFontTemporary.ttf | Ecam (ISIS) | "Licenced under GPLv3" |
| FBW-Display-RMP-10/11/13/16/19.ttf | RMP-10 ... RMP-19 | (c) 2022 FlyByWire Simulations, GPLv3 |
| NDChrono.ttf | NDChrono | (c) FlyByWire Simulations; SIL OFL template text |
| Poppins-SemiBold.ttf | Poppins-SemiBold (FCU) | SIL Open Font License 1.1 |
| AirbusBAT.ttf, AirbusChronometer.ttf | Digital (BAT), AirbusChronometer | (c) 2021 Tyler Knox; no licence in the file, shipped in FlyByWire's GPL-3.0 repository |
| AirbusRTPI.ttf | AirbusRTPI | adapted from AirbusRMP, (c) 2021 Tyler Knox; as above |
| A380X_FCU.ttf | Digital (FCU) | (c) 2021 Jkaled777, named S4F_A350_AFSCP_display; no licence in the file, shipped in FlyByWire's GPL-3.0 repository |

Not available: `A1000` (RMP/style.scss:37, `/Fonts/A1000/a1000.ttf`) is
MSFS's own font, not in FlyByWire's package; text in it is not drawn and
the family is logged once.

FlyByWire's EIS fonts have no lowercase outlines; the instruments write in
capitals.

## Images

PNG files resolved from `<aircraft>/html_ui/` (`/Images/fbw-a380x/...`),
and PNG data URIs; decoded once with a premultiplied-average mipmap chain
and drawn trilinear. Install: copy the MSFS package's
`html_ui/Images/fbw-a380x/` (`oit/` included now the OIT is drawn, docs/oit.md)
to `<aircraft>/html_ui/Images/fbw-a380x/`.
Native images (NATIVE_IMAGE, the terronnd terrain under each ND) are looked
up from `mapdata::plugin::native_image` every frame, one texture per id,
re-uploaded only when their `generation` or size changes, and drawn with
linear filtering; nothing is drawn while the lookup gives `None`. The ND's
dimming region applies over them, as MSFS's emissive multiplies the whole
texture, terronnd's own potentiometer dimming included.
SVG images (e.g. `Common/thalesTest.svg`, the display unit self-test) are
not rasterised here: the DOM side sends SVG as drawing commands.

## Tests

`src/display/tests.rs`, with the software renderer `soft.rs` carrying out
the same plan as the GL renderer (multisample positions, stencil,
scissor, filtering, blending). Snapshots go to
`D:\A380\fbw-build\display-snapshots`: `shapes.png` (with `shapes-tiny-skia.png`
and `shapes-diff.png`), `text.png`, `images.png`, `pfd-like.png`,
`large.png`, `mfd-dimming.png`.
