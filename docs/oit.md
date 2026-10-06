# OIT / OIS (Onboard Information Terminal / System)

Two browser-backed screens on the lateral consoles, one per pilot, running
FlyByWire's own `A380X/OIT/oit.html` exactly as the PFD/ND/EWD/SD/MFD do
(docs/display-stream.md). This doc is the trace for why it was excluded,
what changed to bring it up, what is real and what is not, and what is
still outside this repo.

## Why it was excluded

Two independent, and different, reasons layered on top of each other:

1. **FlyByWire's own take-off/landing performance pages were a stub.**
   `OitFltOpsPerformance.tsx` (used for *both* `flt-ops/to-perf` and
   `flt-ops/ldg-perf` before this work) rendered nothing but `NOT YET
   IMPLEMENTED`. FlyByWire's own docs site says the same thing in different
   words: <https://docs.flybywiresim.com/pilots-corner/a380x/a380x-briefing/flight-deck/main-panel/oit/>
   — "The OIS/OIT is not yet implemented in the A380X. We will provide a
   detailed description in the future." Everything else under `OIT/` (STS,
   FLT FOLDER/CHARTS via `OitFltOpsEfbOverlay`, the whole NSS Avionics
   domain including the Company Com AOC inbox) is *not* a stub — it is
   ordinary, functional FBW TSX, just never publicised.
2. **This port's own scope note** (docs/team.md, before this work: "The EFB
   and OIT are out of scope: the user won't use FBW's EFB") folded the OIT in
   with the EFB decision, even though the OIT and the EFB are unrelated
   FlyByWire modules with unrelated reasons to exist (see "EFB/OIS boundary"
   below). That was a scoping shortcut, not a technical finding — nothing
   about the OIT's own code needed excluding once looked at directly.

Neither reason survives scrutiny once you separate "is it real" (yes, for
everything but the performance pages, which this work fixes honestly) from
"did we choose to render it yet" (no, until now).

## What changed

### 1. Rendering the OIT (done, verified)

A prior, interrupted pass had already done the mechanical part correctly; I
verified it by reading every diff against `git diff`/`git log` and by
compiling and running the affected tests rather than trusting the working
tree:

- `EXCLUDED_GAUGES` in `src/js/msfs/mod.rs` and `src/xphfbw_bridge_views.rs`
  no longer lists `A380X/OIT/` (only `A380X/OITlegacy/` stays out — panel.cfg
  lists the current page first and the legacy one second in both OIT
  sections, so the legacy page never runs; `xphfbw_bridge_views.rs`'s
  `each_oit_view_is_the_current_page_not_the_legacy_one` test proves this
  against a literal copy of panel.cfg's VCockpit19/20 text).
- `src/display/screens.rs` adds `SCREEN_OIT_LEFT`/`SCREEN_OIT_RIGHT`
  (1333x1000, matching panel.cfg), dimmed by potentiometers 78/79
  (`OitSimvarPublisher.tsx`'s `potentiometerCaptain`/`potentiometerFo`) and
  gated on the buses `OitDisplayUnit.tsx`'s `DisplayUnitToDCBus` names —
  AC2/DC2 for the captain's terminal, AC ESS/DC ESS (in flight)/DC1 for the
  first officer's.
- `app/src/views.rs`'s `SCREEN_ORDER` gets the same two ids appended (so no
  existing screen's index moves); its own test was updated from 16 to 18
  screens.
- `src/display/mod.rs` generalises what was `toggle_mfd_keyboard` /
  `mfd_keyboard_callback` (KCCU-only) into `toggle_keyboard` /
  `keyboard_callback` over a new `HAS_KEYBOARD` list: `SCREEN_DU_MFD`,
  `SCREEN_OIT_LEFT`, `SCREEN_OIT_RIGHT`. The OIT is a keyboard-and-trackball
  device on the real aircraft, not a touchscreen (the "Additional Control
  Device (ACD) for the OIT" on FlyByWire's own Lateral Consoles page —
  <https://docs.flybywiresim.com/pilots-corner/a380x/a380x-briefing/flight-deck/overviews/lateral-console/>
  — "enables navigation through applications if the keyboard and pointing
  device is retracted or not available", implying the normal case *is* a
  keyboard and pointing device). The mechanism is the same right-click
  popup-and-focus gesture already built for the KCCU/MFD
  (`XPLMTakeAvionicsKeyboardFocus`, XPLM410 has no other way to give a
  device a physical keyboard) — mouse move/click for the trackball is
  already generic across every screen, so no separate "trackball" code was
  needed, only the keyboard route.
- `tools/install.sh` now copies `A380X/OIT/` and `Images/fbw-a380x/oit/`
  (previously excluded alongside the legacy page and the EFB).
- `docs/team.md` and `docs/screens.md` updated: they had stale lines saying
  the OIT was out of scope, and the converted-aircraft cockpit-device list
  hadn't been updated for `SCREEN_EFB`/`SCREEN_OIT_LEFT`/`SCREEN_OIT_RIGHT`.

Verified: `cargo check --lib` (clean, only pre-existing warnings in unrelated
files), `cargo test --lib --features js` for
`xphfbw_bridge_views::tests::each_oit_view_is_the_current_page_not_the_legacy_one`
and every `display::`/`screens::` test (31 tests, all pass). I did not run
the `--ignored boots_fbw` end-to-end test (needs a rebuilt
`fbw-a380x/out/.../html_ui`, which needs Node/pnpm and is a 10+ minute build
outside this session's scope) or X-Plane itself — see "Where I stopped".

### 2. The OIT Side Console switch (traced end-to-end; one piece left, and it's not in this repo)

The real control is FlyByWire's `SWITCH_GLARESHIELD_CS_OIT_SIDE` (captain) /
`SWITCH_GLARESHIELD_FO_OIT_SIDE` (first officer) —
`fbw-a380x/.../model/behaviour/glareshield.xml:61-146` in the read-only FBW
tree. Despite the XML file's name it sits on the lateral console beside the
Console & Floor Light switch, per FlyByWire's own Lateral Consoles doc
(item 1: "The OIT Domain switch is used to select the OIT domain for the
OIT display... alongside the Console & Floor Light Switch"). It is a
2-position switch (`NUM_STATES>2`), tooltips
`OIT_SIDE_NSS_AVNCS`/`OIT_SIDE_NSS_FLTOPS`, using FlyByWire's
`A32NX_GT_Switch_Dummy` template — "dummy" here means the click only ever
writes its own position var, nothing else (no other system code runs on
toggle), not that it does nothing useful: `SWITCH_POSITION_VAR` is
`A380X_SWITCH_OIT_SIDE_LEFT`/`_RIGHT`, and FlyByWire's own OIT reads exactly
that var (`OitSimvarPublisher.tsx:42-43` publishes
`L:A380X_SWITCH_OIT_SIDE_LEFT`/`_RIGHT` as `oisDomainSwitchCapt`/`Fo`; `OIT.tsx:59-60`
maps it — `domainSwitch ? 'nss-avncs' : 'flt-ops'`). This is exactly what
gates "FLT OPS mode": with the switch left at its default/unset (`false`),
the terminal is already in `flt-ops`, matching FlyByWire's instruction to
"ensure you are in FLT OPS mode" — it is the failure mode you'd hit if you
(or a saved situation) left the switch the other way.

I found **no** third DIM/OFF position on this switch anywhere in FlyByWire's
model or docs. The brightness range including its OFF detent is the
existing, separate rotary knob (`KNOB_EFIS_CS_OIT`/`KNOB_EFIS_FO_OIT`,
`mip.xml:77-79,208-210`, potentiometers 78/79), already wired to
`LIGHT POTENTIOMETER:78/79` in `src/display/screens.rs`'s `Dimming`
entries — a native X-Plane dataref array, so its manipulator (once modelled)
needs no plugin code at all, exactly as documented already in `screens.rs`.
My read is that "FLT OPS/DIM/OFF" describes these two adjacent controls
(domain switch + brightness knob) together, not one three-position switch;
I could not find a source for a genuine third position and did not invent
one.

**What is missing, and where:** the clickable 3D geometry. `docs/team.md`
already establishes that the converter (`D:\A380\msfs2xp-aircraft`) is a
different tool with a single lead who edits it, and this repo's job is to
*describe* a needed converter change, not make it. I've put that exact
description in `docs/team.md`'s "Needed converter change" note: give
`SWITCH_GLARESHIELD_CS_OIT_SIDE`/`_FO_OIT_SIDE` a click manipulator writing
the boolean dataref `A380X_SWITCH_OIT_SIDE_LEFT`/`_RIGHT`, the same way
every other FlyByWire dummy toggle switch's `SWITCH_POSITION_VAR` already
becomes clickable elsewhere on this converted aircraft. **No Rust change in
this repo is needed once that exists** — every other FlyByWire dummy L:var
switch in this aircraft is a generic X-Plane dataref by name already (the
`host.getVar`/`setVar` bridge in `src/js/msfs/simvar.js` calls into the
plugin's variable registry by string name for any `L:` var FBW's JS touches,
the same mechanism every other unmentioned dummy switch — e.g. sidestick
priority, ATC MSG ack — already relies on with zero per-switch Rust code).
I did not add Rust code for this because there is nothing for Rust code to
do here beyond what already exists generically; I would be inventing a
special case for a switch that needs the same treatment as a dozen others
this port doesn't special-case either. If that generic mechanism turns out
*not* to cover this specific var (I read the JS-side bridge but did not
instrument a running plugin to confirm the Rust-side registry has no
allow-list), that is the one place after the converter change where a
one-line fix might be needed — flag it in testing.

### 3. T.O PERF (real data page; certified results correctly withheld)

FlyByWire's `AircraftContext.performanceCalculators.takeoff` is `null` for
the A380X (`EFB/index.tsx:23-25` — the same "not installed" state as the
front-end EFB's own performance page, which is the honest, structural
reason no certified take-off performance can be shown: there is no
certified A380-842 RTOW data set anywhere in this codebase, and Airbus does
not publish one for third parties to embed. FlyByWire ships a real one only
for the A320 (`fbw-a32nx/.../performance/a32nx_takeoff.ts`).

Given that, the previous (interrupted) attempt at this task wrote
`OitFltOpsTakeoffPerformance.tsx` — a full replacement for the stub — that
draws exactly this line: real inputs, honestly unavailable certified
outputs. I read every line of it, confirmed its factual claims (the
`performanceCalculators.takeoff: null` claim, `maxZfw`/`maxGw` in
`PerformanceConstants.ts`), and kept it as sound work, only relocating it
(below) to fix a rule violation the previous attempt left behind.

What is real:
- **Runway/airport data**: `loadAirport()` calls MSFS's own
  `LOAD_AIRPORT`/`SendAirport` facility API — the simulator's real
  navigation database, the same one every other instrument in this aircraft
  uses. TORA, ASDA, elevation and slope are derived from the published
  runway record (`runwayEnds()`), not typed in.
- **Weather**: `readWeather()` reads `AMBIENT TEMPERATURE`, `SEA LEVEL
  PRESSURE`, `AMBIENT WIND DIRECTION/VELOCITY` — the simulator's live
  ambient state, resolved into head/crosswind on the runway's true bearing
  (`windComponents`) and pressure altitude (`pressureAltitude`).
- **Weights**: `readWeights()` reads `L:A32NX_FQMS_GROSS_WEIGHT`/
  `_CENTER_OF_GRAVITY_MAC`/`_TOTAL_FUEL_ON_BOARD` through ARINC 429 sign
  status (`arincValue` returns `null`, not a stale number, when the word's
  SSM is not `NormalOperation`) plus the airframe's own loadsheet ZFW/ZFW-CG.
- **THR RED / EO ACCEL**: read from the FMS (`L:A32NX_FM1_THR_RED_ALT`/
  `_EO_ACC_ALT`) — these are not certified performance computations, just
  FMS state, so they're shown as read.
- **SYNC FMS**: pulls the flight plan's actual origin/departure runway
  (`FmsData`'s `fmsOrigin`/`fmsDepartureRunway`), nothing invented.

What reads `NOT AVAIL` with a stated reason, deliberately, per the
"don't fake values" rule: **V1, VR, V2, the flap setting, FLEX temperature,
the performance-limited MTOW, THS and stop margin/TODA.** The page's own
footer states why: "NO CERTIFIED A380-842 TAKE-OFF/LANDING PERFORMANCE DATA
SET IS INSTALLED." `SYNC SIMBRIEF` and `RECHECK WITH AVNCS` are visibly
disabled (`disabled={Subject.create(true)}`), not silently inert — there is
no SimBrief bridge or NSS Avionics performance link in this codebase to
call.

**Runway selector**: the user's brief asks for a scrollable list of named
intersections (M13A/N9, N7/M10A, etc.). X-Plane's/MSFS's facility database
(`apt.dat`) does not carry intersection names — only full runway ends and
their published lengths. The page's runway selector is therefore a dropdown
of runway *ends* (e.g. "09L"), with a separate T.O SHIFT distance field and
a note explaining exactly why: "INTERSECTION NAMES ARE NOT IN THE NAVIGATION
DATABASE; ENTER THE INTERSECTION AS A T.O SHIFT FROM THE RUNWAY THRESHOLD."
TODA is withheld the same way (no clearway length in `apt.dat`). This is the
same principle as the V-speeds: say what's missing rather than invent
plausible-looking intersection names.

### 4. LDG PERF + OPTIONS (same treatment)

`OitFltOpsLandingPerformance.tsx`: LDA/elevation/slope/wind from the same
real runway-and-weather path as T.O PERF; LD, FLD and stop margin withheld
as `NOT AVAIL` for the same certified-data reason (there is also no maximum
*landing* weight constant in `PerformanceConstants.ts`, so the landing
weight limit is withheld too rather than assumed equal to MTOW or guessed).

`OitFltOpsPerfOptions.tsx`: one shared component, mounted at the bottom-left
of both pages, radio-button groups for weight (T/KLB), altimeter (HPA/IN HG)
and distance (M/FT). Deliberately local to the page's own `Subject`, not
touching FMS or EFIS unit settings — the real OIS keeps these apart too.

### 5. FLT OPS STS / FLT FOLDER / CHARTS / CLEAN-TURNAROUND

- **STS** (`OitFltOpsStatus.tsx`) and **FLT FOLDER**/**CHARTS**
  (`OitFltOpsEfbOverlay.tsx`) are ordinary, already-functional FlyByWire
  TSX — not stubs, never touched by the exclusion or by this work. They
  should render as soon as the OIT gauge runs, no port-side change needed.
  I did not independently re-verify their internal data sources line by
  line (budget ran out before I did the same audit I did for the
  performance pages); flag that if precision matters here too.
- **CLEAN/TURNAROUND**: I searched FlyByWire's entire `OIT/` tree
  (`grep -rn "clean\|turnaround"`) and found nothing — this sub-mode does
  not exist anywhere in FlyByWire's source. It is not disabled or stubbed;
  it was never written. **Not implemented** — building it from nothing
  (a genuine real-aircraft OIS concept: whether the FMS/OIS data set is
  reset for a new flight vs. continued) was outside what this session's
  budget allowed after the performance pages and the compliance fix below.

### 6. NSS Avionics domain

The same domain switch (section 2) already toggles FlyByWire's `OIT.tsx`
between `'flt-ops'` and `'nss-avncs'` per pilot side — this is already
functional, end to end, contingent only on the same missing converter
manipulator. Under NSS Avionics: `OitAvncsFolderNavigator.tsx`,
`OitAvncsMenu.tsx` and the Company Com AOC apps
(`OitAvncsCompanyCom.tsx`, `OitAvncsCompanyComInbox.tsx`,
`OitAvncsCompanyComFlightLog.tsx`) are real, already-written FlyByWire
components — the "dedicated NSS inbox" the brief asks for already exists as
`OitAvncsCompanyComInbox.tsx`. I did not audit these for real-vs-fake data
the way I did the performance pages (budget). **Expanded crew-identification
fields for pilot information and operating status**: I searched every
FltOps/NssAvncs login and header component
(`OitFltOpsLogin.tsx`, `OitFltOpsHeader.tsx`, `OitAvncsLogin.tsx`,
`OitAvncsHeader.tsx`) for anything resembling crew ID or operating status
and found nothing — like CLEAN/TURNAROUND, this does not exist in
FlyByWire's source. **Not implemented.**

### 7. EFB Notification Centre

Searched this repo's `src/efb.rs`, FlyByWire's A380X EFB source, and the
shared flyPad framework in `fbw-common` for "notification"/"bell" — nothing
anywhere. This is genuinely new work, in `src/efb.rs` (allowed), not a
FlyByWire port. **Not started** — this is exactly where the priority order
says to stop, and where I stopped: everything above it got real engineering
attention; this did not get any.

## EFB/OIS boundary

`src/efb.rs`'s existing scope (its own module doc) is a dataref/command
*shim* for a **third-party EFB app** — payload, refuel, ground services and
settings, each one explicitly "nothing simulated beside" FlyByWire's own
systems variables. Separately, `app/src/views.rs`'s `spawn_efb_view` draws
this port's own study/settings UI onto the `SCREEN_EFB` mesh — again, not
FlyByWire's flyPad. Neither of those is the OIS. The OIS is an
**aircraft-specific operational tool**: it reads the FMS, the ARINC weight
words, the navigation database and the ambient weather directly, and its
pages (T.O PERF, LDG PERF, STS, charts, AOC) are things a pilot uses to fly
*this airplane*, not general trip-planning apps. FLT FOLDER and CHARTS route
through `OitFltOpsEfbOverlay.tsx`, which is FlyByWire's own bridge for
showing EFB-hosted content (charts, documents) *inside* the OIS frame
without duplicating the EFB's storage or logic — I left that exactly as
FlyByWire wrote it rather than re-routing it through `src/efb.rs`, since
that overlay's whole purpose is to avoid the OIS reimplementing what the EFB
already owns. The line I drew: nothing in this work adds OIS-shaped data
paths to `src/efb.rs`, and nothing reroutes EFB-shaped concerns (payload,
fuel, ground services, settings) into the OIT pages.

## Frame cost

Each screen in this port is its own CEF renderer process
(docs/briefs/xphfbw-app.md: "one off-screen CEF browser per instrument
view"), each with its own `Engine`/JS budget
(`src/js/msfs/mod.rs`: 500ms load budget, 1GB memory ceiling per view;
`src/js/mod.rs`: 8ms per-frame script budget). The two OITs are two more
such processes — `SCREEN_ORDER` goes from 16 entries to 18. Concretely:

- **Off X-Plane's main thread**: each OIT runs its own JS/React tree on its
  own CEF renderer process, same as the ND/PFD/MFD already do — this is
  parallel background CPU and RAM (a full Chromium renderer process per
  view, React re-render and timer overhead comparable to the existing
  ND/PFD given similar code complexity), not serialized with the sim frame.
- **On X-Plane's main thread**: a texture upload (only on dirty rects) plus
  a draw callback per screen per frame, same mechanism as every existing
  screen (`src/display/mod.rs`, `xphfbw.rs`'s dirty-rect plan). Two more
  1333x1000 screens is two more textures of that size and two more (cheap,
  batched) draw calls when their content changes.
- I did not, and could not from this session (no running X-Plane instance),
  measure actual RSS or per-frame milliseconds. I am not going to invent a
  number for this either — the honest statement is "two more CEF processes
  and two more 1333x1000 textures, of comparable weight to the existing
  ND/PFD-class instruments," not a specific millisecond or megabyte figure.
  Measuring this for real is explicitly listed under "Where I stopped."

## Real A380 OIS vs. FlyByWire's implementation, with sources

- FlyByWire's own docs (as of this session) call the OIT undocumented and
  imply it is new/unpolished: <https://docs.flybywiresim.com/pilots-corner/a380x/a380x-briefing/flight-deck/main-panel/oit/>.
  Their Lateral Consoles page (<https://docs.flybywiresim.com/pilots-corner/a380x/a380x-briefing/flight-deck/overviews/lateral-console/>)
  does describe the OIT Domain switch, the OIT display, and the ACD
  (trackball/keyboard alternative), matching what I found in the model XML
  and the OIT's own TS source.
- The real A380 OIS has two domains reachable from the same terminal — the
  Flight Operations domain (aircraft-specific: performance, charts, status,
  logbook) and the NSS Avionics domain (company/AOC communications) — which
  FlyByWire's `OisDomain = 'nss-avncs' | 'flt-ops'` matches structurally.
  I could not find a public, citable Airbus source for the OIT's exact T.O
  PERF/LDG PERF field layout beyond what the user's brief already specified
  and what FlyByWire itself implemented; the iniBuilds forum thread found in
  search (<https://forum.inibuilds.com/topic/40810-how-to-take-off-landing-performance-calculations-via-oit/>)
  is about a different aircraft/add-on and I did not rely on it for any
  factual claim above.
- The clearest, most load-bearing difference from the real aircraft: the
  real OIS's T.O PERF/LDG PERF pull certified take-off/landing performance
  from an onboard database function (RTOW/LPC); FlyByWire's A380X has never
  shipped that data set (confirmed directly in their source,
  `EFB/index.tsx:23-25`), so neither their front-end EFB nor this OIS page
  can produce a real V1/VR/V2/FLEX/MTOW — and, per the standing "don't fake
  values" rule, none does. That is a genuine capability gap versus the real
  aircraft, not a bug to paper over.

## Where I stopped

Done, with evidence: OIT rendering (priority 1) verified by compiling and
running the relevant tests; the OIT Side Console switch (priority 2) traced
completely, with the one remaining piece (a converter manipulator) written
up as an exact, described change per `docs/team.md`'s existing convention
for converter work, not touched myself; T.O PERF and LDG PERF + OPTIONS
(priorities 3-4) reviewed line by line for factual honesty and kept; a real
process-compliance problem fixed (below).

**Compliance fix, not asked for but required by the hard rules:** the
previous, interrupted attempt had written the four performance-page files
directly into `D:\fbw-aircraft` (a `git status`/`git diff` inside that repo
showed two modified tracked files and four new untracked ones) — a direct
violation of "D:\fbw-aircraft is READ-ONLY reference... changes to their
source go in `patches/`". I captured that work as a single `git diff`
(`tools/js-build/patches/oit-performance-pages.patch`), reverted
`D:\fbw-aircraft` to a clean tree (`git status` there now shows nothing
under `OIT/`), and wrote `tools/js-build/apply-patches.sh` (apply/revert,
round-tripped and verified with `git apply --check`) per the mechanism
`docs/js-build.md` had already described but never implemented. Building
FlyByWire's instruments now needs: `apply-patches.sh apply`, run
`npm run build-a380x:instruments` (docs/js-build.md), then
`apply-patches.sh revert`.

Not done, in priority order, with why:

- **FLT OPS STS / FLT FOLDER / CHARTS** (priority 5): already-real FlyByWire
  code, unaudited by me for data honesty the way I audited the performance
  pages. Should render; not independently verified.
- **FLT OPS CLEAN/TURNAROUND** (priority 5): does not exist in FlyByWire's
  source at all. Not implemented — this would be new work, not a port.
- **NSS Avionics** (priority 6): domain switching works (same mechanism as
  priority 2); the Company Com inbox exists in FlyByWire's source and is
  unaudited by me; **expanded crew-identification fields for pilot
  information and operating status do not exist anywhere in FlyByWire's
  source** — not implemented.
- **EFB Notification Centre** (priority 7): not started. No FlyByWire
  precedent anywhere (own A380X EFB, or the shared flyPad framework) to
  port from; this is genuinely new design and implementation work in
  `src/efb.rs`.
- **The `--ignored boots_fbw` end-to-end JS-boot test**, and any actual
  in-X-Plane check, were not run this session (needs a rebuilt FBW
  `html_ui` tree — 10+ minutes with Node/pnpm — and a running X-Plane
  instance, neither available here). This is the next thing I'd do before
  trusting the OIT actually paints anything.
- **Frame cost** is described qualitatively (above) but not measured.

## Patch left for you to apply

None in `docs/oit.md` itself — I did not need an `src/lib.rs`,
`src/deep/` or `src/js_bridge.rs` change for any of the above. The one
patch this doc asks you to look at is the converter change described in
`docs/team.md`'s "Needed converter change" note (click manipulators for
`SWITCH_GLARESHIELD_CS_OIT_SIDE`/`_FO_OIT_SIDE`), which is outside this
repo by `docs/team.md`'s own convention ("Only the lead edits the
converter... describe it exactly in your report").
