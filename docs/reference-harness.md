# Reference harness (Chromium)

Ground truth for the displays: FBW's built A380X instruments rendered in real
Chromium (Playwright) for a named simulator state, then compared with the
native pipeline's software-raster PNGs. Code: `tools/reference/`.

## Commands

From `D:\A380\fbw-xp-systems\tools\reference` (`npm install` once; Chromium lives
in `D:\A380\fbw-build\browsers`, `npm run install-browser` fetches it):

| command | does |
|---|---|
| `npm run reference -- --state approach` | renders the default screens of one state |
| `npm run reference` | every state (`--state all`) |
| `npm run reference -- --state climb --screen SCREEN_DU_PFDL,FCU` | chosen screens (short aliases PFD_L, PFD_R, ND_L, ND_R, EWD, SD, MFD, ISIS, CLOCK work too) |
| `npm run reference -- --state all --record` | also records DOM/CSS/SVG usage |
| `npm run dom-usage` | writes docs/dom-usage-recorded.md from the recordings |
| `npm run compare` | compares every pair found and writes the report |
| `npm run compare -- --state approach --screen SCREEN_DU_EWD --fail-below 0.9` | a subset; exit code 2 if any SSIM is below 0.9 |

Other `reference` options: `--settle <ms>` (override the state's settle
time), `--out <dir>`, `--headed`, `-v` (print console errors).
Other `compare` options: `--ref <dir>`, `--ours <dir>`, `--out <dir>`,
`--threshold <0-255>` (per-channel difference counted as differing, default 32).

Default screens: SCREEN_DU_PFDL, SCREEN_DU_PFDR, SCREEN_DU_NDL, SCREEN_DU_NDR,
SCREEN_DU_EWD, SCREEN_DU_SD, FCU, Clock. MFD, ISIS, RTPI, BAT and RMPs render
too when named (the MFD has no FMS behind it here, see Limits).

## File layout and naming

Screen names are panel.cfg texture names without `$`, the ids the renderer
(`src/display/screens.rs`) and the cockpit meshes (`ATTR_cockpit_device`) use.

| path | written by |
|---|---|
| `D:\A380\fbw-build\reference-png\<state>\<screen>.png` | `npm run reference` |
| `D:\A380\fbw-build\reference-png\<state>\<screen>.access.json` | same: every variable read (key, units, count, found), variables read but missing from the state (`missingInputs`), writes, storage keys, Coherent calls and listeners, errors, virtual clock stats |
| `D:\A380\fbw-build\reference-png\<state>\<screen>.console.log` | same: the page console |
| `D:\A380\fbw-build\reference-png\<state>\state.resolved.json` | same: the flat variable set used, with each value's `src` |
| **`D:\A380\fbw-build\display-snapshots\<state>\<screen>.png`** | **the native pipeline** (the input `npm run compare` expects) |
| `D:\A380\fbw-build\reference-report\index.html`, `report.json`, `<state>\<screen>.{ref,ours,diff,overlay,ssim}.png` | `npm run compare` |
| `D:\A380\fbw-build\reference-dom-usage\<state>.<screen>.json` | `npm run reference -- --record` |

Snapshots for comparison must be:
- the screen's full texture at panel.cfg `pixel_size` (another size is
  bilinear-resampled to the reference size and flagged in the report);
- composited over black, **before** potentiometer dimming and power gating
  (`Dimming` in `screens.rs`). In MSFS those are applied by the cockpit
  model's emissive material, not by the HTML, so the reference is undimmed;
- rendered from `state.resolved.json` of the same state (load the `vars`
  into the Vars registry, advance the JS clock `time.settleMs` in
  `time.frameMs` steps starting at `time.epochMs - settleMs`).

## States

`tools/reference/states/<name>.json`, one scenario per file, built from
parts in `states/parts/` (`common`: electrical buses; `systems-<state>`:
engines, FADEC, EWD/SD data, air, hydraulics, fuel; `avionics-<state>`:
ADIRS, RA, PRIM/FMA, FCU, EFIS, FM, ILS, landing gear, flaps). Every
scenario uses the same UTC time (2026-06-21T12:00:00Z, an arbitrary fixed
date so the clock and ETA fields repeat) and settles 20 s of simulated
frames, which covers FBW's display power-up
(`MsfsAvionicsCommon/CdsDisplayUnit.tsx`: 0.25-0.45 s Thales boot, then
`CONFIG_SELF_TEST_TIME`, default 15 s, when spawned cold and dark).

| state | flight file | scenario |
|---|---|---|
| cold-dark-powered | apron.FLT | At the gate. Batteries and external power on, so every AC and DC bus is powered. Engines off, APU off. ADIRS mode selectors OFF (apron.FLT), so no air data or attitude. FWC flight phase 1. |
| engines-running-ground | taxi.flt | Parked with the parking brake set, all four engines running stabilised at idle, ADIRS aligned in NAV, flaps 1+F selected, on ground, zero ground speed. FWC flight phase 2. |
| climb | Climb.flt | Clean, gear up, 300 kt IAS (Climb.flt), passing FL150 in a managed climb: AP1 and A/THR engaged, A/THR in THR CLB, FMA THR CLB / CLB / NAV, CLB thrust limit. FWC flight phase 6. |
| approach | approach.FLT | ILS approach in CONF 3, gear down, at VAPP with A/THR in SPEED, AP1 engaged, LOC and G/S captured, about 1500 ft radio altitude. FMGC flight phase approach (approach.FLT sets `A32NX_INITIAL_FLIGHT_PHASE = 5`). |

Each value is derived from FBW's code paths and carries a `src` (file:line);
variables with no real source are left out. Which variables a screen needs
comes from its `access.json` (`reads`, `missingInputs`): the page logs every
read, so a render of a state lists what is still missing.

### Format

```json
{
  "name": "approach",
  "description": "...",
  "extends": ["parts/common", "parts/systems-approach", "parts/avionics-approach"],
  "flt": "approach.FLT",
  "time": { "utc": "2026-06-21T12:00:00Z", "settleMs": 20000, "frameMs": 16.667 },
  "screens": ["SCREEN_DU_PFDL", "SCREEN_DU_EWD"],
  "vars": {
    "L:A32NX_ENGINE_N1:1": { "value": 18.9, "unit": "percent", "src": "fbw-a380x/src/wasm/fadec_a380x/...:123", "why": "idle N1" },
    "L:A32NX_ADIRS_ADR_1_ALTITUDE": { "arinc429": { "value": 1650, "ssm": "NormalOperation" }, "src": "..." },
    "L:A32NX_FCDC_1_DISCRETE_WORD_1": { "arinc429": { "bits": [11, 12], "ssm": "NormalOperation" } }
  },
  "storage": { "A32NX_CONFIG_SELF_TEST_TIME": "15" }
}
```

Resolution order, lowest priority first: the flight file (its `[LocalVars.0]`
as L-vars, `[Switches.0] Potentiometer.N` as `LIGHT POTENTIOMETER:N`,
`[Gauges.0] KollsmanSetting` as `KOHLSMAN SETTING MB:1` in inHg; read from
the installed package's SimObjects folder), then each parent in order, then
the state's own `vars`. `E:` time variables (ZULU TIME, LOCAL TIME, ZULU
DAY/MONTH/YEAR, ABSOLUTE TIME, SIMULATION TIME) are derived from `time.utc`.

- Keys: `L:NAME`, `A:NAME[:index]` (the `A:` prefix is optional), `E:NAME`,
  `GAME:NAME`. A/E names are case-insensitive, L names are exact.
- `unit` is the unit the value is given in. A read in another unit of the
  same dimension is converted (`host/units.js`); reads in
  `number`/`enum`/`bool` return the stored number.
- `arinc429`: FBW's `Arinc429Word` encoding (float32 bits of the value plus
  SSM × 2^32). `bits` sets discrete bits, 1-based as FBW's `bitValueOr(bit)`.
  SSM: FailureWarning 0, NoComputedData 1, FunctionalTest 2, NormalOperation 3.
- `storage`: `GetStoredData` keys (NXDataStore adds the `A32NX_` prefix).
  Missing keys read as `''`, as in MSFS.
- File names starting with `_` are skipped by `--state all`.

## How a screen is rendered

1. `src/panel.mjs` reads panel.cfg (the installed MSFS package, else FBW's
   tracked copy): one `[VCockpitNN]` is one screen at `pixel_size`, with its
   `htmlgaugeNN=url, x, y, w, h` gauges in order (SD is sd.html with
   sdv2.html over it). WASM gauges (`WasmInstrument/...terronnd`) cannot run
   in a browser and are skipped: the ND's terrain layer is absent.
2. `src/server.mjs` serves an MSFS-like virtual file system on 127.0.0.1:
   `/Pages/VCockpit/Instruments/A380X/...` from FBW's build output
   (`fbw-a380x/out/flybywire-aircraft-a380-842/html_ui`, the runtime
   engineer's build, docs/js-build.md; falls back to
   `D:\A380\fbw-build\reference-bundles`), `/Fonts`, `/Images`, `/JS/fbw-a380x`
   from FBW's tracked base package, and `/JS/dataStorage.js` plus the host
   page from `tools/reference/host`. Lookups are case-insensitive like the
   MSFS VFS. No Asobo file is served or copied.
3. `host/vcockpit.html` is the VCockpit view. `host/msfs-base.js`
   reimplements what the instruments use of the MSFS environment:
   `vcockpit-panel` loading each gauge's html the way the VCockpit does
   (`text/html` templates, `import-script`, stylesheets; the `Url` attribute
   carries panel.cfg's query string), `BaseInstrument` (URL config and
   index, the requestAnimationFrame main loop gated on
   `OnAllInstrumentsLoaded`, `Update`, `requestCall`, electricity state),
   `TemplateElement`, `registerInstrument`, the native `simvar` binding and
   `SimVar` (registered ids, batches, game vars, struct getters),
   `Coherent` (on/off/trigger/call), `RegisterViewListener` (generic data,
   shared global), `GetStoredData`, `Simplane` getters over their
   variables, `LatLong`/`LatLongAlt`/..., `Avionics.Utils`, `Utils`,
   `diffAndSet*`. The variable store is loaded from the state; instrument
   writes go into it, key events are logged. Every access is logged for
   `access.json`.
4. `host/freeze.js` makes the run repeatable: a virtual clock replaces
   `Date`, `performance.now`, timers, `requestAnimationFrame` and
   `requestIdleCallback`; it starts `settleMs` before the state's time,
   stands still while the panel loads, and the runner then advances it one
   `frameMs` step at a time (due timers in order, each followed by a real
   macrotask so promise chains settle, then the frame callbacks).
   `Math.random` is seeded. CSS animations/transitions are stamped with the
   virtual time they started and pinned to that phase before the
   screenshot. Chromium runs headless with `--force-color-profile=srgb
   --font-render-hinting=none --disable-lcd-text --disable-gpu`, device
   scale 1. Two runs of the same state give byte-identical PNGs.
   (Playwright's own `page.clock` is not used: it yields a clamped real
   timer after every fake timer and takes minutes per simulated second.)
5. After settling: fonts ready, animations frozen, screenshot of the
   viewport (black behind transparent pixels).

## Compare metrics

Per pair (`compare/compare.mjs`), both images composited over black:

- **SSIM** on luminance, 11-tap Gaussian window, sigma 1.5, K1 0.01, K2
  0.03 (Wang et al. 2004); 1 is identical. The SSIM map image is dark where
  structure differs.
- **Edge F1**: Sobel edges of both images matched within 2 px. Recall is
  the share of the reference's edges we draw, precision the share of ours
  the reference has. Insensitive to small offsets and antialiasing, strict
  about missing or extra line work and text.
- **MAE, RMSE, PSNR** per channel (0-255).
- **Differing**: share of pixels with any channel off by more than
  `--threshold`.
- **Lit**: share of pixels brighter than 24 in each image (missing or extra
  content at a glance).
- Images: heat-map diff, overlay (reference magenta, ours green, both
  white), SSIM map, and copies of both inputs.

The report rows are green for SSIM ≥ 0.95 and edge F1 ≥ 0.9, amber for SSIM
≥ 0.8, red below; pairs with only one side present are listed as missing.

## Recorded DOM usage

With `--record`, `host/recorder.js` wraps every method and accessor of the
DOM, CSSOM, SVG, Canvas and animation interface prototypes (and `.style`
through a proxy for `el.style.prop = v`), attributing each call to the
first script on the stack: instrument bundle, harness host shim, or the
runner. At the end it walks the style sheets (rule types, properties,
keyframes, value functions, selector features) and the final document
(elements, attributes, inline styles, transform functions, path commands,
computed properties that differ from a bare element). `npm run dom-usage`
merges all recordings into docs/dom-usage-recorded.md. Calls the
instruments make through host shims (e.g. `diffAndSetAttribute`) are
attributed to the host.

## Limits

- One screen per page, as in MSFS, but no other instruments run beside it:
  data the instruments exchange over the event bus (`JS_LISTENER_GENERICDATA`
  sync), shared globals, the FMS flight plan (MFD/ND), facility and nav data
  loads, and Bing/terrain maps get no answer. The ND shows no route or
  navaids, the MFD no FMS pages.
- Values the Rust/C++ systems and the systems host compute continuously are
  frozen at the state's values for the whole settle time.
- Two images FBW references are not in its repository or package
  (`/Images/fbw-a380x/oans/oans-cross.svg`, `oans-flag.svg`) and 404 in MSFS too.
- Coherent GT is not Chromium: text shaping, antialiasing and a few CSS
  behaviours differ in the simulator. The reference is the instruments'
  intended rendering by a standards browser, which is what our DOM and
  renderer implement.
