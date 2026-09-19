# What FBW's A380X instruments need from a DOM

A survey of the displays in scope (PFD L/R, ND L/R, EWD, SD/SDv2, MFD, FCU,
RMP 1-3, ISIS, Clock, RTPI, BAT; not EFB, OIT, OITlegacy, ATCCOM or popup),
done to size `src/js/dom/`. Two sources were read:

- the TypeScript: `fbw-a380x/src/systems/instruments/src/{PFD,ND,EWD,SD,SDv2,MFD,FCU,RMP,Clock,ISISlegacy,BAT,RTPI,MsfsAvionicsCommon,Common}`
  plus the fbw-common pieces they import (`ND`, `OANC`, `RMP`, `PFD`,
  `MsfsAvionicsCommon`, `react`), 482 files;
- the compiled package (`html_ui/Pages/VCockpit/Instruments/A380X/*/*.js|css`),
  which includes `@microsoft/msfs-sdk` 2.3.3 and React 17.0.2 as bundled,
  and the CSS as FBW's build emits it.

Counts are occurrences in those files. The Chromium reference harness
(`tools/reference/`, `docs/dom-usage-recorded.md` when it lands) records
what actually runs and should be used to confirm this.

## Screens (panel.cfg)

| screen | gauge | CSS px |
|---|---|---|
| MFD (both, one texture) | A380X/MFD/mfd.html | 1646 x 1024 |
| EWD, SD (+SDv2 overlay) | ewd.html, sd.html, sdv2.html | 768 x 1024 |
| PFD L/R, ND L/R | pfd.html, nd.html | 768 x 1024 |
| FCU | FCU.html | 2560 x 1280 |
| ISIS | isislegacy.html | 512 x 512 |
| Clock | clock.html | 256 x 256 |
| RTPI | rtpi.html | 338 x 128 |
| BAT | bat.html | 256 x 128 |
| RMP 1-3 | rmp.html | 1664 x 1024 |

Frameworks: PFD, ND, EWD, SDv2, MFD, FCU, RMP, Clock are msfs-sdk
FSComponent. SD (legacy, 86 files), ISISlegacy (18), BAT and RTPI are React
17 (`react-dom` `render` into `#MSFS_REACT_MOUNT`).

## DOM APIs

### How FSComponent builds and renders (msfs-sdk 2.3.3)

- `buildComponent`: `document.createElementNS(SVG_NS, tag)` for the tags in
  its `svgTags` list (circle, clipPath, defs, ellipse, g, image, line,
  linearGradient, marker, mask, path, pattern, polygon, polyline,
  radialGradient, rect, stop, svg, text, tspan, ...), otherwise
  `document.createElement(tag)`. Anything else inside an `<svg>` (e.g.
  `use`, `symbol`, `foreignObject`) is an *HTML* element there, as in the
  browser.
- Props: `ref.instance = element`; `class` as string, `SubscribableSet`
  (`classList.add/remove`) or record (`classList.toggle(name, bool)`);
  `style` as string (attribute), `SubscribableMap`/`ObjectSubject`/record
  (`style.setProperty(name, value)`, `null`/`''` to remove); everything
  else `setAttribute` (`setAttributeNS(XLINK, ...)` for `xlink:*`), bound
  subscribables re-set the attribute on change.
- Text children and subscribable text: `insertAdjacentHTML('beforeend', text)`
  then `node.root = element.lastChild`, later `node.root.nodeValue = v`
  (`' '` for empty). So `insertAdjacentHTML` must parse HTML (entities,
  and markup when the string has some) and `nodeValue` must work on Text.
- `render(node, element, position)`: `appendChild`, `insertAdjacentElement('beforebegin'|'afterend')`,
  `lastChild`, `previousSibling`, `nextSibling`, `instanceof HTMLElement || instanceof SVGElement`.
- `FSComponent.remove` = `element.remove()`.

### Calls made by the instruments (source, in scope)

| API | uses | notes |
|---|---:|---|
| `style.display` / `style.visibility` / `style.transform` | 166 / 119 / 37 | property assignment, not only `setProperty` |
| `style.backgroundColor,top,left,fill,width,height,color,...` | ~45 | camelCase properties |
| `style.transition` | 2 | OANC pan (`transform 150ms linear`) |
| `classList.add / remove / toggle / contains / replace` | 88 / 69 / 30 / 4 / 2 | plus FSComponent's own |
| `setAttribute` | ~40 | `d` 8, `transform` 5, `visibility` 4, `class` 4, `x`,`y`, `fill`, `style`, `disabled` |
| `getAttribute('url')` | 5 | on `vcockpit-panel > *` custom element |
| `.remove()` / `removeChild` / `appendChild` | 78 / 18 / 7 | `while (el.firstChild) el.removeChild(el.firstChild)` 13x |
| `innerHTML` | 14 | `''` to clear; markup: `<tspan class=..>` in SVG text (PFD ILS/FMA), `<span style=..>` (MFD F-PLN) |
| `innerText` | 21 | MFD input fields and F-PLN |
| `textContent` | 18 | |
| `document.getElementById` | 30 | |
| `document.querySelectorAll('vcockpit-panel > *')` | 3 | child combinator on custom tags |
| `document.documentElement.classList` | 8 | `animationsEnabled` |
| `document.body.addEventListener` | 10 | React `Common/input.tsx`: keydown, wheel, mouse* |
| `addEventListener` | 58 | click 30, mousemove 7, mousedown/up 2, wheel 1, mouseover/out 1, dblclick 1, keydown/keypress 3, focus/blur 2, update 2, unload 1, change 1 |
| `onClick` JSX (React) | 247 | React synthetic events via root-container delegation |
| `getBoundingClientRect` | 1 | MFD dropdown menu positioning |
| `scrollTop` / `offsetTop` | 1 / 1 | MFD dropdown scroll-to-selection |
| `focus()` / `blur()` / `contains()` | 2 / 2 / 7 | MFD input fields |
| `getContext('2d')` | 9 | ND CanvasMap, ND VerticalDisplay, OANC layers, OANS BTV |
| `new Path2D(...)` | ~23 | ND symbols (SVG path strings incl. arcs), OANC geometry |

Not used anywhere in scope (checked in bundles too): `getBBox`,
`getComputedTextLength`, `getTotalLength`, `getCTM`, `createSVGPoint`,
`transform.baseVal`, `dataset`, `getComputedStyle`, `cssText`, `cloneNode`,
`replaceChild`, `MutationObserver`, `ResizeObserver`, `offsetWidth/Height`,
`clientWidth/Height`, Web Animations (`animate`, `getAnimations`),
`OffscreenCanvas`, shadow DOM, custom elements registry.

React 17's `react-dom` (SD, ISIS, BAT, RTPI) additionally uses:
`createTextNode`, `createElementNS`, `insertBefore`, `removeAttribute`,
`setAttributeNS`, `nodeType`, `nodeName`, `namespaceURI`, `ownerDocument`,
`parentNode`, `style[camelCase] = value` and `style.setProperty('--x')`,
`textContent`/`nodeValue`, `dispatchEvent`/`CustomEvent`, `activeElement`,
`window.HTMLIFrameElement` (for `instanceof` in selection restoration),
and `addEventListener(type, fn, {capture, passive})` on the root container
for every event type it delegates.

## SVG

Elements (JSX intrinsic tags): `text` 747, `g` 729, `path` 422, `line` 377,
`svg` 87, `rect` 68, `polygon` 39, `circle` 39, `clipPath` 25, `tspan` 21,
`stop` 13, `polyline` 7, `defs` 5, `linearGradient` 4, `image` 4.
Not used: `use`, `symbol`, `marker`, `mask`, `pattern`, `ellipse`,
`textPath`, `foreignObject`, `radialGradient`, filters.

Attributes: `x`/`y` 1512, `d` 605, `transform` 507 (`rotate(` 404,
`translate(` 185, `scale(` 7, no `matrix`/`skew`), `stroke-width` 230,
`visibility` 179, `fill` 175, `font-size` 155, `text-anchor` 135, `stroke`
117, `viewBox` 66, `points` 50, `cx/cy/r` 41, `clip-path` 17,
`stroke-dasharray` 15, `stroke-linecap` 15, `alignment-baseline` 13,
`stop-color` 13, `dy` 9, `opacity` 8, `preserveAspectRatio` 6 (`meet`),
`stroke-linejoin` 5, `fill-opacity` 4, `xlink:href`/`href` 2,
`xml:space="preserve"` 1 (FMA), `dominant-baseline` 2.
Percent coordinates appear (`x="50%"` in the self-test screens).
One malformed transform is load-bearing: the altitude tape writes
`translate(0 ${offset}` without the closing parenthesis, so transform
parsing must be lenient.

PFD root: `<svg class="pfd-svg" viewBox="0 0 158.75 211.6">` sized by CSS to
768x1024, so user units are ~4.84 px and stroke widths are written in `mm`
(`0.16mm`).

## CSS

Compiled sizes: MFD 53 KB, ND 33 KB, EWD 30 KB, SDv2 25 KB, PFD 7 KB, SD 7 KB,
RMP 4 KB, FCU 3 KB, ISIS 2 KB, Clock/BAT/RTPI < 1 KB.

Selectors: class and compound classes (`.Cyan.Fill.Stroke`, `text.Cyan`),
ids, tags, descendant (all), child `>` (MFD 33, RMP), sibling `~` and `+`
(MsfsAvionicsCommon widgets and MFD radio buttons),
attribute (`input[type=radio]`, `#Electricity[state=off]`),
`:hover` (97), `:disabled`, `:checked`, `:not(.x)`, `:first-child`,
`:last-child`, `:nth-child(2)`, `::before`/`::after` (MFD `content:
attr(data-label)` and `"CRS"`), `!important` (PFD 8, EWD 79, SDv2 46, SD 35,
ND 25). No `:root` variables outside popup/EFB (so `var()` is not needed in
scope; tailwind lives in popup, which is out of scope).

Properties by use (in-scope bundles): `display` (flex 322, none 32, grid 23,
block 19, inline 8), `color`, `font-size`, `fill`, `stroke`, `width`,
`flex-direction`, `justify-content`, `background-color`, `height`,
`align-items`, `border`, `position` (absolute 98, relative 32, fixed 1),
`font-family`, `padding`, `stroke-width`, `top`, `flex` (1, 3, 5, 8, `1 1 auto`),
`margin*`, `left`, `text-align`, `opacity`, `background`, `z-index`,
`text-anchor`, `visibility`, `stroke-linecap`, `transform` (translate/X/Y,
scale, rotateX(0)), `overflow` (auto 12, hidden 11), `font-weight`,
`letter-spacing`, `line-height`, `right`, `font-style`,
`grid-template-columns/rows` (px, %, fr, auto, `repeat(3, 1fr)`),
`grid-column: span 3`, `row-gap`/`column-gap`, `align-self`,
`animation*`, `bottom`, `pointer-events`, `box-shadow`, `white-space`
(pre, nowrap, pre-wrap), `vertical-align`, `border-radius` (50% circles),
`background-image` (url, data URIs), `background-size: cover`, `text-decoration: underline`,
`outline`, `stroke-linejoin`, `paint-order` (text outlines), `fill-rule`,
`dominant-baseline: middle`, `stroke-miterlimit`, `box-sizing`, `flex-wrap`,
`transition` (Clock, OANC).

Units: px, %, `mm` (PFD strokes), `em` and `rem` (MFD), unitless SVG numbers,
one `calc()` (RMP). Colours: hex (3/6/8 digit), `rgba()`, `hsl()`, names
(`olive`, `black`, `white`, ...).

Fonts (`@font-face`, family -> file, per instrument): Ecam ->
FBW-Display-EIS-A380.ttf (ISIS maps Ecam to ISISFontTemporary.ttf as well),
FBW-Display-EIS-A380-SlashedZero, NDChrono, Digital -> A380X_FCU.ttf in FCU
but AirbusBAT.ttf in BAT, Poppins-SemiBold, A1000, RMP-10/11/13/16/19,
AirbusChronometer, AirbusRTPI. The same family name maps to different files
in different instruments, so the family alone does not identify a font.

Animations (28 `@keyframes`): all flashing is done with keyframes whose
stops sit 1% apart (`0%,50% {opacity:0} 51%,100% {opacity:1}`), with
`animation-name/duration/iteration-count` (infinite or 9) and the
shorthand (`green-pulse 1s step-end infinite`, `Disappear 0s 10s forwards`).
Animated properties: `opacity`, `fill`, `stroke`, `visibility`.
`animation-timing-function` appears 3 times (steps/linear).

## HTML layout

- PFD, ND, EWD, SDv2, FCU, Clock: absolutely positioned `div`/`svg` layers;
  nearly all drawing is SVG.
- MFD, RMP, and the MsfsAvionicsCommon UI widgets (input fields, buttons,
  dropdowns, page headers) in ND/EWD/SDv2: flexbox (row/column,
  `justify-content` center/space-between/flex-end/space-evenly,
  `align-items` center/baseline/flex-start/flex-end, `flex: n`, wrap),
  grid for MFD PERF/FUEL/SURV tables and RMP pages, margins/padding/borders,
  inline text in spans, `overflow: auto/hidden` with `scrollTop` for dropdowns.
- ND OANC: stacked `<canvas>` layers moved with CSS `transform:
  translate(..) scale(..) rotate(..)` on HTML elements.

## Canvas2D

Used by the ND map (`fbw-common/.../ND/shared/map`), the ND vertical display
and OANC. Calls: `strokeStyle` 52, `lineTo` 47, `stroke` 38 (also
`stroke(path)`), `translate` 36, `lineWidth` 35, `moveTo` 32, `beginPath`
30, `resetTransform` 23, `closePath` 20, `setLineDash` 17, `rotate` 16,
`fillStyle` 12, `font` 10 (`'21px Ecam'`), `fill` 9 (also `fill(path)`),
`save`/`restore` 8, `strokeRect` 5, `ellipse` 5, `clip` 4 (with Path2D),
`clearRect` 3 (whole canvas), `textAlign`/`textBaseline` 2,
`strokeText`/`fillText` 2, `scale`, `rect`, `arc`. `Path2D` from SVG path
strings (with `a` arcs) and built with `moveTo/lineTo/closePath`.
OANC draws its layers incrementally over several frames (a retained
bitmap), clearing with `clearRect(0, 0, w, h)`.

## Consequences for the implementation

- Must have: namespaced elements, attributes, classList, style with
  camelCase properties, text nodes with `nodeValue`, `insertAdjacentHTML`
  and `innerHTML` with a real fragment parser (SVG namespace inside SVG
  parents), the tree operations above, `querySelector(All)` with
  descendant/child/sibling/attribute/pseudo selectors, capture/bubble
  events with options objects, `getBoundingClientRect`.
- SVG: path (all commands), line, rect, circle, ellipse, polygon, polyline,
  text/tspan (x, y, dx, dy, anchor, baselines, xml:space), g transforms
  (lenient parsing), clipPath, defs, linearGradient, image, dasharray,
  viewBox/preserveAspectRatio, percent lengths, `mm` units.
- CSS: cascade with specificity/`!important`/presentation attributes/
  inline, inheritance, the properties above, keyframe animations (numbers,
  colours, discrete), `:hover`, `@font-face` per document.
- Layout: block, inline text with wrapping, absolute/relative, flex, basic
  grid, overflow clip and scroll offset, `::before/::after` text content.
- Canvas2D: the calls listed, retained across frames until cleared.
- Not needed: `use`/markers/masks/patterns/filters, `getBBox` and friends,
  Web Animations, transitions (only OANC pan and Clock), `var()` in scope.
