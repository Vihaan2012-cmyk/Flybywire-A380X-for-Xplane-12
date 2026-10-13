# The Electronic Checklist (ECL) in this port

Scope: the A380's normal checklists — the ones the C/L pushbutton on the ECAM
control panel opens on the EWD. Code: `src/ecam_patches/ecl.rs` (this
workstream's only source file), reached from `ecam_patches::source_patches()`.

**No change to `src/lib.rs`, `src/deep/` or `src/js_bridge.rs` was needed**, so
there is no patch in this file waiting to be applied by hand. `ecam_patches.rs`
gained three lines (a `mod ecl;`, and `let mut patches` / `patches.extend
(ecl::source_patches()); patches` around the existing `vec![...]`).

FlyByWire's tree at `D:\fbw-aircraft` was not modified. Both changes below are
`SourcePatch`es against FlyByWire's *built* JS, the mechanism `docs/js-build.md`
prescribes for changes this small ("most of the fixes in `src/ecam_patches.rs`
are one assignment or a few lines"); nothing needed a file under
`patches/`, which by the convention there holds diffs against FlyByWire's
**Rust** (`patches/fbw-rust/*.patch`, applied with `git apply` in
`D:\fbw-aircraft` before a build). A `patches/` file for these would have to be
applied *and* would double up with the `SourcePatch`, so it would be a hazard,
not a safety net;
`ecam_patches::ecl::tests::every_ecl_patch_matches_its_built_file_exactly_once`
is the guard instead — it fails the moment a FlyByWire rebuild moves the text
either patch keys off.

## 1. What the ECL did before this pass

**The machinery was already running, and already drawing.** This was checked,
not assumed:

- `SystemsHost` and the EWD both boot in the sim (`Log.txt`: `js: xphfbw view 15
  loaded: A380X_SYSTEMSHOST`, `js: xphfbw view 1 loaded: A380X_EWD`).
- `FwsNormalChecklists` is constructed inside `FwsCore`, which `SystemsHost.ts:
  248-259` ticks from the `Clock` instrument's `simTimeHiFreq` at 50 Hz. `Clock`
  is the *first* instrument in the backplane (`SystemsHost.ts:204`), so the
  `EfisTawsBridge` exception in the log (see §5) does not starve it.
- The built bundles really carry the content and the renderer: `COCKPIT
  PREPARATION` and `fws_normal_checklists` both appear in
  `SystemsHost/SystemsHost.js` *and* `EWD/ewd.js`.
- The new `ecl_opens_and_closes_on_every_c_l_press` test drives the real
  FlyByWire SystemsHost and EWD and reads the rendered EWD DOM back. With the
  ECL open, the first `.ProceduresContainer` (`EWD.tsx:386`) is `display: flex`
  and contains, in order:

  ```
  CHECKLISTS  COCKPIT PREPARATION  BEFORE START  AFTER START  TAXI  LINE-UP
  <<DEPARTURE CHANGE>>  APPROACH  LANDING  PARKING  SECURING THE AIRCRAFT
  ```

**What did not work was the crew's side of it.** The A380 opens the normal
checklists with `C/L` on the ECAM control panel and works them with the ECP's
own `✓` (CHECK), `UP` and `DOWN` — not with the KCCU; the KCCU is the MFD's
(`docs` for the ECP: *"The EWD displays the normal checklist menu"*, *"Validates
/devalidates the item that is surrounded by a blue box on the EWD"*, *"moves the
blue box up/down"*). Those five pushbuttons were effectively dead after one
click each:

- FlyByWire's `ecam-cp.xml` (`FBW_ECAM_BUTTON_SubTemplate`, and the CLR/MORE
  templates) gives every ECP pushbutton both a `LEFT_SINGLE_CODE`
  (`1 (>L:A32NX_BTN_<name>)`) and a `LEFT_LEAVE_CODE` (`0 (>L:...)`) — a
  momentary button.
- The converted aircraft's SASL binding carries only the press. In
  `<aircraft>/plugins/sasl/data/modules/main.lua`, every ECP block looks like
  `do -- PUSH_ECAM_CL` → `press()` writes `fbw/A32NX_BTN_CL = 1`, and
  `local function release()` is **empty**. `cockpit_bindings.txt:333` describes
  the same control as `PUSH_ECAM_CL: click command running the control's MSFS
  code in SASL`.
- `FwsCore.update` (`FwsCore.ts:2842-2861`) feeds each button into an
  `NXLogicMemoryNode` and then an `NXLogicPulseNode` (`clPulseNode`,
  `clCheckPulseNode`, `clUpPulseNode`, `clDownPulseNode`, `abnProcPulseNode`,
  written at `ts:2902-2908`). Pulse nodes fire on the **rising** edge only, and
  the memory node is reset at the end of every FWS cycle (`ts:6143-6150`). A
  variable latched at 1 therefore produces exactly one pulse and then nothing,
  for the rest of the session.

  Net effect in the cockpit: the first press of `C/L` opened the checklist menu,
  and after that `C/L`, `✓`, `UP` and `DOWN` did nothing at all, ever. The
  checklist could not be closed, no checklist could be entered, no item could be
  ticked.

- The same conversion also dropped the `<Condition NotEmpty="SIMVAR">` branch of
  `FBW_ECAM_BUTTON_SubTemplate`, so four buttons write the template's *fallback*
  `A32NX_BTN_#BASE_NAME#` name instead of the `#SIMVAR#` one FwsCore reads:

  | the cockpit writes (`main.lua`) | `FwsCore.ts` reads | |
  |---|---|---|
  | `A32NX_BTN_TOCONF` | `A32NX_BTN_TOCONFIG` | **mismatch** |
  | `A32NX_BTN_CLR_LH` | `A32NX_BTN_CLR` | **mismatch** |
  | `A32NX_BTN_CLR_RH` | `A32NX_BTN_CLR2` | **mismatch** |
  | `A32NX_BTN_RCLLAST` | `A32NX_BTN_RCL` | **mismatch** (RCL LAST only; the RCL button itself matches) |
  | `A32NX_BTN_CL`, `_CHECK_LH`, `_CHECK_RH`, `_UP`, `_DOWN`, `_ABNPROC`, `_RCL` | same | match |

  `T.O CONFIG` is an ECL matter: the TAXI checklist's last sensed item is
  `T.O CONFIG ... TEST/NORM`, fed by `toConfigNormal`, which only goes true
  after the T.O CONFIG TEST pushbutton. With the name mismatch, that item could
  never tick and the TAXI checklist never reached its sensed end state.

Everything else the sensed items read was checked and is genuinely wired in this
port: `A:CABIN SEATBELTS ALERT SWITCH` (cockpit → `fbw/CABIN_SEATBELTS_ALERT_
SWITCH`, and `key_events.rs:236`), `A:LIGHT BEACON` (`sensors.rs:440`),
`L:XMLVAR_SWITCH_OVHD_INTLT_EMEREXIT_Position` and `L:PUSH_OVHD_OXYGEN_CREW`
(`cockpit_bindings.txt:1461` and `:1244`), `A32NX_AUTOBRAKES_ARMED_MODE`
(`a380_systems/hydraulic/autobrakes.rs:248`), the FCDC spoiler-armed words and
the flap handle. `L:`-name lookups from the JS go through `js_bridge.rs:287-290`
(`vars.add(named, NAMED)`, **no** implicit `A32NX_` prefix), so
`L:XMLVAR_...` really is `fbw/XMLVAR_...`, which is the name the cockpit
binding steps.

## 2. What changed

### `ecl_ecp_buttons_are_momentary` (`SystemsHost.js`)

Rewrites FwsCore's ECP acquisition block so each button is read through one
helper that also writes it back to 0 — exactly what `LEFT_LEAVE_CODE` does in
MSFS — and so the four fallback names above are read as well as the canonical
ones. Holding the button down still produces exactly one action, as it does in
MSFS, because the pulse node is rising-edge either way.

Sources: `fbw-a380x/src/base/.../model/behaviour/ecam-cp.xml`
(`LEFT_SINGLE_CODE`/`LEFT_LEAVE_CODE` pairs); FlyByWire's own on-screen ECL soft
keys `EWD/elements/EclSoftKeys.tsx`, which do the identical thing explicitly
(`SetSimVarValue('L:A32NX_BTN_CHECK_LH', .., 1)` then `setTimeout(() =>
SetSimVarValue(.., 0), 50)`); the converted aircraft's own `main.lua` and
`cockpit_bindings.txt` for the four fallback names.

The root cause is in the converter (`D:\A380\msfs2xp-aircraft`, which this workstream
is to stay out of) — it should emit `LEFT_LEAVE_CODE` and honour
`<Condition NotEmpty="SIMVAR">`. Fixing it here also fixes the already-installed
aircraft without a reconversion. **Worth passing to whoever owns the converter**:
the empty `release()` is specific to these `ASOBO_GT_Push_Button` /
`ASOBO_GT_Push_Button_Airliner` uses inside a `<Condition>` — other controls in
the same `main.lua` do get non-empty `release()` bodies, so it is not a blanket
gap.

### `ecl_rudder_trim_neutral_is_signed` (`SystemsHost.js`)

AFTER START's sensed `RUDDER TRIM ... NEUTRAL` tested
`this.fws.rudderTrimPosition.get() < 0.35` (`FwsNormalChecklists.ts:545`).
`rudderTrimPosition` (`FwsCore.ts:4764`) is the SEC's **signed** trim position in
degrees, straight from `Arinc429Word.fromSimVarValue('L:A32NX_SEC_1_RUDDER_
ACTUAL_POSITION')` (`ts:4755-4756`; written as `rudder_trim_actual_pos_deg`,
`fbw_a380/src/FlyByWireInterface.cpp:647,2139`). Without `Math.abs` every *left*
(negative) trim setting satisfies `< 0.35`, so the item ticked itself as NEUTRAL
with the trim wound fully left — a sensed item reporting a state the aircraft is
not in.

Sources, both FlyByWire's own: `FwsCore.ts:4761-4762` takes
`Math.abs(sec1RudderTrimActualPos.valueOr(0)) > 3.6` of the *same* word for the
rudder-trim-not-in-T.O-config warning; and FlyByWire's own pre-2020 ECL variant,
kept commented out at `NormalProceduresBefore2020.ts:448-449`, uses
`Math.abs(...) < 0.35`. Only the comparison changed — not the 0.35° threshold,
not the item text.

## 3. Content: FlyByWire's A380 ECL against the real aircraft

### Which checklist set the A380 uses

FlyByWire ships two: the live `NormalProcedures.ts` (post-2020 format) and
`NormalProceduresBefore2020.ts`, which is **dead code** — nothing imports
`EcamNormalProceduresBefore2020`, and its own header says *"Left this one here if
we decide to also implement the older ECL from before 2020."*

The live set is the post-2021 Airbus format. Airbus amended its SOPs and
checklists in November 2021 across "all the commercial airbus families (except
for A300 & A220)": the "down to the line / below the line" concept was dropped,
**BEFORE TAKEOFF was split into TAXI and LINE UP**, and *"items that are
monitored by the aircraft system … were removed"*
([Travel Radar, "Airbus Publishes New SOP and Checklist for 2022"](https://travelradar.aero/new-airbus-sop-and-checklist-2022/)).
`NormalProcedures.ts` has exactly that shape (COCKPIT PREPARATION, BEFORE START,
AFTER START, **TAXI**, **LINE-UP**, APPROACH, LANDING, PARKING, SECURING THE
AIRCRAFT, `<<DEPARTURE CHANGE>>`), so the *structure* is right for a current
A380. `NormalProceduresBefore2020.ts` has the pre-amendment shape (BEFORE
TAKEOFF, AFTER TAKEOFF/CLIMB, AFTER LANDING) and matches FlyByWire's own paper
checklist card
([FBW_A380X_Checklist.pdf](https://docs.flybywiresim.com/pilots-corner/a380x/assets/sop/FBW_A380X_Checklist.pdf),
29 NOV 2021, marked "FOR SIMULATION PURPOSES") and their SOP
([FBW_A380X_SOP.pdf](https://docs.flybywiresim.com/pilots-corner/a380x/assets/sop/FBW_A380X_SOP.pdf),
which calls for "BEFORE START CHECKLIST down to the line").

### The honest limit on this comparison

**No A380-specific normal-checklist card, FCOM PRO-NOR-SOP page or QRH page
could be found on the open web.** Searched for: A380 FCOM/QRH normal checklists;
Emirates/operator A380 FCOM PRO-NOR; A380 ECL item lists; A380-specific strings
(`GND SPLRs`, `ALL 4 BATs`, `ALL 3 LAPTOPS`) alongside "A380"; FlyByWire's own
docs and manuals repo; scribd/pdfcoffee copies (landing pages only, no
extractable text). Everything returned is either the *A320 family* card, a
simulator-community checklist derived from the FlyByWire A380X itself
(circular), or FlyByWire's own pre-2020-format PDF.

The closest item-level source in the *right* (post-2021) format is the
**A319/A320/A321 NORMAL CHECKLIST, revision C3, Oct 2025**
([aviationlads](https://aviationlads.com/wp-content/uploads/2025/10/Airbus-Checklist-2025.pdf);
marked "CREATED BY AVIATIONLADS. FOR SIMULATION USE ONLY"). It is a different
type, and a simulator card, so **it cannot on its own justify adding an item to
the A380's ECL** — the standing rule here is that a checklist item that is not in
the real aircraft's checklist is worse than a missing one. It is used below only
to *locate* the deltas, which are then listed as candidates needing an A380
source, not applied.

### Per checklist

| ECL checklist | Verdict | Notes |
|---|---|---|
| COCKPIT PREPARATION (1000001) | present, structure correct; 1 candidate delta | GEAR PINS & COVERS / FUEL QUANTITY / SEAT BELTS (sensed) / BARO REF. The A320 C3 card also carries **ADIRS … NAV** between SIGNS and BARO REF. FlyByWire's own pre-2020 A380 set has `ADIRS … NAV` as a sensed item too (`NormalProceduresBefore2020.ts:45-50`, sensed from `ir1/2/3MaintWord` bit 3). Candidate, unsourced for the A380. |
| BEFORE START (1000002) | present, structure correct; 1 candidate delta | PARKING BRAKE / T.O SPEEDS & THRUST / BEACON (sensed). The A320 C3 card also carries **WINDOWS … CLOSED (BOTH)**, and FlyByWire's own pre-2020 A380 set has `WINDOWS/DOORS … CLOSE (BOTH)`. Candidate, unsourced for the A380. |
| AFTER START (1000003) | present; **one sensed item was wrong — fixed**; 1 candidate delta | ANTI ICE / PITCH TRIM / RUDDER TRIM (sensed). RUDDER TRIM's sensing is fixed above. The A320 C3 card also carries **ECAM STATUS**; FlyByWire's pre-2020 A380 set has `ECAM STS … CHECK/NORMAL` sensed from `ecamStsNormal` (which exists in `FwsCore`). Candidate, unsourced for the A380. |
| TAXI (1000004) | present and correct for the A380; 1 candidate delta | FLIGHT CONTROLS / FLAPS SETTING / RADAR, then the `T.O` block: SEAT BELTS, GND SPLRs ARM, FLAPS T.O, **AUTO BRK RTO**, T.O CONFIG TEST→NORM, all sensed. `AUTO BRK … RTO` is A380-correct (the A320 card says MAX; `A380AutobrakeMode::RTO = 6`, `a380_systems/hydraulic/autobrakes.rs:57` — and the sensing reads `=== 6`), as is the `GND SPLRs` spelling. The A320 C3 card additionally has **CABIN READY** in the T.O block and **ENG MODE SEL**; the A380 has no ENG MODE selector of that kind, and CABIN READY is unsourced for the A380. Candidate. |
| LINE-UP (1000005) | present, structure correct; 1 candidate delta | T.O RWY / PACK 1 & 2 (both unsensed). `PACK 1 & 2` is A380-correct (two packs). The A320 C3 card also carries **TCAS … TA/RA**. Candidate, unsourced for the A380. |
| `<<DEPARTURE CHANGE>>` (1000006) | present and correct | RWY & SID / FLAPS SETTING / T.O SPEEDS & THRUST / FCU ALT, `onlyActivatedByRequest: true`. Same four items and same order as the A320 C3 card's `<< DEPARTURE CHANGE >>`. |
| ALL PHASES / AT TOP OF DESCENT / FOR APPROACH / FOR LANDING deferred (1000007/8/9/11) | present and correct | Placeholders filled at runtime from `EcamDeferredProcedures`; hidden from the menu until one exists (`itemsToShow`, `FwsNormalChecklists.ts:519-530`). Confirmed hidden in the rendered EWD menu by the new test. |
| APPROACH (1000010) | present, structure correct; 1 candidate delta | BARO REF / SEAT BELTS (sensed) / MINIMUM / AUTO BRAKE. The A320 C3 card also carries **ENG MODE SEL**, which does not transfer to the A380. No A380-sourced delta. |
| LANDING (1000012) | present, structure correct; 1 candidate delta | `LDG` block: SEAT BELTS, LDG GEAR DOWN, GND SPLRs ARM, FLAPS LDG, all sensed. The A320 C3 card also has **CABIN READY** in the LDG block. Candidate, unsourced for the A380. |
| **AFTER LANDING** | **missing** | Not in `NormalProcedures.ts` at all. It *is* in the A320 C3 card (one item, `RADAR & PRED W/S … OFF`) and in FlyByWire's own pre-2020 A380 set (`GND SPLRs DISARM / FLAPS 0 / APU START`, all sensed) and their A380X paper card (`EXTERIOR LIGHTS SET / SPLRS DISARM / FLAPS 0 / APU START`). Whether the post-2021 A380 ECL still has an AFTER LANDING checklist, and with which items, **could not be sourced**. Not added — see §4. |
| PARKING (1000013) | present and correct | PARKING BRAKE OR CHOCKS / ENGINES OFF (sensed) / WING LIGHTS OFF / FUEL PUMPs OFF (sensed). Item-for-item the same as the A320 C3 card's PARKING. |
| SECURING THE AIRCRAFT (1000014) | present, structure correct; 2 candidate deltas | OXYGEN OFF (sensed) / EMER EXIT LIGHT OFF (sensed) / EFBs OFF / BATTERIES OFF. Item-for-item the same as the A320 C3 card. FlyByWire's own A380 material is more specific — `ALL 3 LAPTOPS … OFF` and `ALL 4 BATs … OFF` in `NormalProceduresBefore2020.ts:410-421`, `ALL 3 LAPTOPS`/`ALL 4 BATTERIES` in their A380X card — and the A380 does have four batteries. Whether the post-2021 A380 ECL words them that way is unsourced; not changed, because guessing the wording would make it *less* likely to match, not more. |

Sensed/unsensed split: every `sensed: true` item in `NormalProcedures.ts` has a
real system reading behind it in `FwsNormalChecklists.sensedItems`
(`FwsNormalChecklists.ts:537-610`); none is hardcoded to tick. The two `false`
entries (the `T.O` and `LDG` sub-headlines) are headline rows, excluded from
selection by `ProcedureLinesGenerator.nonSelectableItemStyles`, and a checklist
is completed by its explicit `C/L COMPLETE` line, not by all items being
checked — so they do not block completion. `EMER EXIT LIGHT … OFF` reads
`XMLVAR_SWITCH_OVHD_INTLT_EMEREXIT_Position === 2`, which is correct: the
switch's three states are 0 = ON, 1 = ARM, 2 = OFF
(`A380_Cockpit_Behavior.xml:1950-1963` `ANIMTIP_0/1/2`; confirmed by
`fbw-common/.../checklists/CheckItemStates.ts:133` `checkEmerExtLtOff`). The
pre-2020 block's commented-out `=== 0` is the wrong one, and is not live.

## 4. What could not be sourced (nothing was invented)

Listed as candidates above and **deliberately not applied**:

- `ADIRS … NAV` in COCKPIT PREPARATION
- `WINDOWS … CLOSED (BOTH)` in BEFORE START
- `ECAM STATUS` in AFTER START
- `CABIN READY` in the TAXI `T.O` block and the LANDING `LDG` block
- `TCAS … TA/RA` in LINE-UP
- an `AFTER LANDING` checklist, and its items
- `ALL 3 LAPTOPS` / `ALL 4 BATs` wording in SECURING THE AIRCRAFT

Each has a real source for the **A320 family** (the C3 Oct-2025 card) and/or for
a **pre-2020 A380** (FlyByWire's own dead `NormalProceduresBefore2020.ts` and
their A380X paper card), but none for a current A380's ECL. To resolve them, the
needed document is an A380 FCOM `PRO-NOR-SOP` normal-checklist page or an
operator's A380 normal-checklist card at revision 2022 or later. With that in
hand, each candidate above is a one-item edit to `NormalProcedures.ts` plus a
matching-length entry in `FwsNormalChecklists.sensedItems` (the two arrays must
stay the same length) — i.e. two more `SourcePatch`es beside the ones here.

## 5. Adjacent findings, not changed here

- **`EfisTawsBridge` throws every frame it runs.** `Log.txt`: `Unhandled
  rejection: TypeError: Cannot read properties of null (reading 'bitValueOr') at
  EfisTawsBridge.onUpdate (SystemsHost.js:180971)` — `this.validIrMaintWord`
  is `null` there. `onUpdate` is `async`, so the backplane does not await it and
  the other instruments (including the `Clock` that drives `FwsCore`) keep
  running; the ECL is unaffected. Belongs to whoever owns the surveillance/TAWS
  area.
- **CLR and RCL LAST were entirely dead** before this pass (both name
  mismatches, above). They are now read, which mainly matters for the abnormal
  side of the ECAM rather than the ECL.
- **FlyByWire's on-screen ECL soft keys exist** (`EclSoftKeys.tsx`: ✓, UP, DOWN
  drawn on the EWD itself) but are hidden unless the `NXDataStore` key
  `CONFIG_A380X_SHOW_ECL_SOFTKEYS` is `'1'`. Left at FlyByWire's default (off),
  because the real aircraft has no such keys and the ECP buttons now work; it is
  a ready-made fallback if clicking the 3-D pushbuttons ever proves awkward in
  X-Plane.

## 6. Verifying

```
CARGO_TARGET_DIR=D:/A380/fbw-xp-systems/target-c1 cargo test --release --features js --lib ecl
CARGO_TARGET_DIR=D:/A380/fbw-xp-systems/target-c1 cargo test --release --features js --lib -- --ignored --nocapture ecl_opens
```

The first is always-on and checks both patches still match the built tree
exactly once. The second boots the real SystemsHost and EWD, presses `C/L`
twice the way the converted cockpit does (set the variable, never clear it), and
asserts the checklist opens, draws its menu on the EWD, is released each time,
closes again, and that `DOWN` then `✓` opens BEFORE START. Run with
`options.patches = Vec::new()` it fails on the first press — `A32NX_BTN_CL was
never released` — which is the symptom this pass was given.

In the sim: press `C/L` on the ECAM control panel. The EWD shows the checklist
menu; press it again and the menu goes away. `UP`/`DOWN` move the cyan box, `✓`
opens the highlighted checklist and ticks unsensed items. T.O CONFIG TEST on the
same panel now reaches the FWS, so the TAXI checklist's `T.O CONFIG` line turns
`NORM`.
