# The ECAM bridge

Turns every area's registered `deep::api::EcamAlert`s into alerts that
appear, sound, sort and resolve inside FlyByWire's own A380 ECAM exactly
like its native ones. Code lives in `src/deep/ecam/` (`mod.rs`, `ids.rs`,
`cond_json.rs`, `codegen.rs`, `patches.rs`, `deep_ecam_bridge.js`, `tests.rs`).
This directory does not itself register any failure, component or alert —
it is the infrastructure every other area's `EcamAlert`s run through.

## 1. FlyByWire's own data model

Studied under `D:\fbw-aircraft\fbw-a380x\src\systems\` (read-only reference).

### Where an abnormal procedure's *text* lives

`instruments/src/MsfsAvionicsCommon/EcamMessages/index.ts` defines the
shapes and `AbnormalSensed/ata*.ts` (one file per ATA chapter group) fill
them:

```ts
export interface ChecklistAction extends AbstractChecklistItem {
  labelNotCompleted: string;        // shown after the item name until done
  labelCompleted?: string;
  colonIfCompleted?: boolean;
}
export interface AbnormalProcedure {
  title: string;                    // "\x1b<4m\x1b4mHYD\x1bm G ENG 1 PMP A PRESS LO"
  sensed: boolean;
  items: (ChecklistAction | ChecklistCondition | ChecklistSpecialItem | TimedChecklistAction | TimedChecklistCondition)[];
  recommendation?: 'LAND ASAP' | 'LAND ANSA';
}
export const EcamAbnormalSensedProcedures: { [n: number]: AbnormalProcedure } = {
  ...EcamAbnormalSensedAta212223, ...EcamAbnormalSensedAta24, /* … */ ...EcamAbnormalSecondaryFailures,
};
export const EcamAbnormalProcedures = EcamAbnormalSensedProcedures; // bare alias, same object
```

Ids are a 9-digit convention documented at the top of every `ata*.ts` file:
`ATA(2 digits) + sub-chapter(1) + kind(1, 8=ABN sensed) + sequence(3)`, e.g.
`290800005` = ATA 29, kind 8 (sensed), sequence 005. This is text/data only —
no function, no `SimVar` reference anywhere in this half.

### Where an abnormal procedure's *behaviour* lives

`systems-host/CpiomC/FlightWarningSystem/FwsAbnormalSensed.ts` pairs every
same-numbered id with an `EwdAbnormalItem`:

```ts
export interface EwdAbnormalItem extends FwsSuppressableItem {
  flightPhaseInhib: number[];                 // FwcFlightPhase values, see below
  monitorConfirmTime?: number;                // default 0.6 s
  whichItemsToShow: () => boolean[];          // per item, same order as items[]
  whichItemsChecked: () => boolean[];         // per item; forced false where items[i].sensed is false
  failure: number;                            // 3=warning, 2=caution, 1=advisory
  sysPage: SdPages;
  inopSysAllPhases?: (checked: boolean[]) => (string | null)[];   // -> EcamInopSys ids
  info?: (checked: boolean[]) => (string | null)[];               // -> EcamMemos ids (STATUS "INFO")
}
export interface FwsSuppressableItem {
  simVarIsActive: Subscribable<boolean>;      // the actual trigger
}
public ewdAbnormalSensed: EwdAbnormalDict = {
  211800001: { flightPhaseInhib: [3,4,5,6,7,8,9,10,11], simVarIsActive: this.fws.pack1Ctl1Fault, whichItemsToShow: () => [], whichItemsChecked: () => [], failure: 1, sysPage: SdPages.Bleed },
  /* … */
};
```

`simVarIsActive` for FlyByWire's own alerts is one of hundreds of
hand-written `Subject`/`MappedSubject` fields on `FwsCore`, each set every
tick inside `FwsCore.update(deltaTime)` from raw `SimVar.GetSimVarValue`
reads and `NXLogicConfirmNode` timers — there is no declarative condition
language on FlyByWire's side; every fault's logic is bespoke TypeScript.

`FwcFlightPhase` (`FwsFlightPhases.ts:12-25`), used by `flightPhaseInhib`:

```
1 ElecPwr, 2 FirstEngineStarted, 3 SecondEngineTakeOffPower, 4 AtOrAboveEightyKnots,
5 AtOrAboveV1, 6 LiftOff, 7 AtOrAbove400Feet, 8 AtOrAbove1500FeetTo800Feet,
9 AtOrBelow800Feet, 10 TouchDown, 11 AtOrBelowEightyKnots, 12 EnginesShutdown
```

`SdPages` (`shared/src/EcamSystemPages.ts`): `Eng=0, Apu=1, Bleed=2, Cond=3,
Press=4, Door=5, ElecAc=6, ElecDc=7, Fuel=8, Wheel=9, Hyd=10, Fctl=11, Cb=12,
Crz=13, Status=14, Video=15`.

### How active faults become what the pilot sees

`FwsCore.ts` (constructor, `FwsCore.ts:2339-2356`):

```ts
this.ewdAbnormal = Object.assign({}, this.abnormalSensed.ewdAbnormalSensed, this.abnormalNonSensed.ewdAbnormalNonSensed);
this.allSuppressableItems = Object.assign({}, this.abnormalSensed.ewdAbnormalSensed, this.abnormalNonSensed.ewdAbnormalNonSensed, this.inopSys.inopSys, this.information.info, this.limitations.limitations);
for (const [key, item] of Object.entries(this.allSuppressableItems)) {
  item.simVarIsActive.sub((v) => { /* records failureActivationTime, for monitorConfirmTime */ }, true);
}
```

and the per-tick loop (`FwsCore.ts:5537-5580`, inside `update(_deltaTime)`):

```ts
for (const [key, value] of Object.entries(this.ewdAbnormal)) {
  if (itemIsActiveConsideringFaultSuppression(value, key, 0.6)) {
    if (newWarning) {
      failureKeys.push(key);
      if (value.failure === 3) this.requestMasterWarningFromFaults = true;
      if (value.failure === 2) this.requestMasterCautionFromFaults = true;
    }
    const itemsChecked = value.whichItemsChecked().map((v, i) => (!proc.items[i]?.sensed ? false : !!v));
    /* … sorted into presentedAbnormalProceduresList, rendered by the EWD, aurals driven off requestMasterWarning/CautionFromFaults … */
  }
}
```

**This is the load-bearing fact the whole bridge rests on**: `this.ewdAbnormal`
and `this.allSuppressableItems` are read *live* every tick via
`Object.entries`, not a snapshot cached once — a plain JS object mutated
after construction is picked up on the very next tick. Everything past that
point (sorting by failure level, `presentedAbnormalProceduresList`,
`ChecklistState`, master light requests, `FwsSoundManager`'s CRC/SC, STATUS
page INOP SYS/INFO via `pushKeyUnique(value.inopSysAllPhases/.info, …)`
resolved against `EcamInopSys`/`EcamMemos`) is FlyByWire's own code, needing
nothing further from this bridge once an entry is in those three objects.

## 2. How this port runs FlyByWire's JS

`src/js/msfs/mod.rs`'s `Cockpit` gives every `panel.cfg` section (EWD,
SystemsHost, SDv2, PFD, …) its **own** JS engine (`Engine::new`, one per
`View`). Each loads its own `.js` bundle independently — confirmed by
`grep`ping a shared literal (`290800001`) across the built tree: it exists
verbatim in both `EWD/ewd.js` and `SystemsHost/SystemsHost.js`, proving each
bundle inlines its own copy of the shared TS modules it imports. There is no
cross-view JS object identity: EWD and SystemsHost cannot see each other's
variables directly, only through simvars/events, same as real MSFS.

`SourcePatch { path, find, replace, reason }` (`src/js/msfs/mod.rs:174-184`)
is the mechanism already used by `ecam_patches.rs`, `wxr/mod.rs` and
`oans/plugin.rs`, assembled into one list by `js_bridge.rs::native_ports`
(`js_bridge.rs:970-993`) and applied as a literal, must-match-exactly-once
find/replace on a file's source text as it loads (`js/msfs/mod.rs:723-752`).
This is a **source-text** patch, applied before the engine parses the file —
not a runtime monkey-patch — so whatever it inserts becomes ordinary code in
that bundle's own module scope, with full access to that bundle's own
top-level `var`s (`EcamAbnormalSensedProcedures`, `EcamInopSys`, `EcamMemos`)
and classes.

`SimVar` (`src/js/msfs/simvar.js`) is the one thing that genuinely is a
global (`globalThis.SimVar = SimVar`) across every bundle, backed by
`__host.getVar(name, unit)` → the plugin's `Host::get_var` trait → whatever
Rust module publishes that dataref/LVar. Everything else from
`@microsoft/msfs-sdk` (`Subject`, `MappedSubject`, …) is a local, non-exported
binding private to each compiled bundle — unreachable from a separately
loaded script, only from text spliced directly into that same bundle.

The FWS itself does run in this port: `SystemsHost.js` instantiates two
`FwsCore`s (FWS1/FWS2, the A380's two flight-warning computers), each
running the exact TypeScript studied above, ticking every frame through the
same `Cockpit::tick` → `Engine` → timers/animation-frame plumbing every
other instrument uses.

## 3. Design

Two kinds of generated JS (`src/deep/ecam/codegen.rs`), both derived from
`ids::assign(alerts)` (`src/deep/ecam/ids.rs`), which sorts by `EcamAlert.key`
and assigns `id = 1_000_000_000 + i`. FlyByWire's own ids are at most 9
digits (`ATA(2) + sub(1) + kind(1) + seq(3)`, largest possible
`999_999_999`, confirmed against the actual compiled catalogue's own largest
ids, the `999800005..999800007` "secondary failure" entries at
`SystemsHost.js:172521-172530`) — every id this bridge mints is 10 digits,
outside that whole space **by construction**, not merely by what happens to
be registered today.

**Static data** — plain text/data, merged once at module-load time,
touching neither `SimVar` nor `FwsCore`:

- `codegen::procedures_merge_js` → `EcamAbnormalSensedProcedures[id] =
  { title, sensed: true, items: [{name, sensed, labelNotCompleted?, style}] }`
  — needed in **both** `EWD.js` and `SystemsHost.js` (each has its own copy;
  `SystemsHost.js` needs it too because `FwsCore.ts:5546`,
  `const proc = EcamAbnormalProcedures[key]`, reads `items[i].sensed` for the
  `whichItemsChecked` masking shown above).
- `codegen::inop_merge_js` / `codegen::info_merge_js` →
  `EcamInopSys[id] = 'TEXT'` / `EcamMemos[id] = 'TEXT'`, one entry per
  `.inop_sys(...)` / `.status_line(...)` line, at ids derived from the
  alert's own id (`alert_id * 1000 + 0..500` for INOP, `+500..1000` for
  STATUS — always inside a 1000-wide block unique to that alert, so two
  alerts' lines can never collide, and never anywhere near 500 lines each in
  practice). Needed only in `SystemsHost.js`: neither `EWD.js` nor
  `SDv2.js`/`SD.js` reference `EcamInopSys`/`EcamMemos` at all in this build
  (`grep -c` returns 0 in all three for both names) — `FwsCore` resolves the
  id to text itself before publishing it over the bus, so only its own copy
  needs the merge.

**Behaviour data**, `codegen::alerts_js_array` → the `DEEP_ECAM_ALERTS`
array `deep_ecam_bridge.js` reads: `id`, `failure` (3/2/1, `Level::Warning`/
`Caution`/`Advisory`+`Memo`), `sysPage` (a GENERIC heuristic from the ATA
chapter — `EcamAlert` carries no page of its own — defaulting to `Status`
when no chapter mapping is obviously better), `flightPhaseInhib` (mapped
from `Phase` to `FwcFlightPhase`, documented per-arm in `codegen.rs`; lossy
in one direction only — our `Phase` has ten variants against FlyByWire's
twelve, no separate V1/400ft phase, so the mapping is conservative, never
narrower than intended), `confirmS`, `trigger` and each procedure line's
`appliesIf`/`doneWhen`/`afterS`, all three as the small tagged-array
encoding `cond_json::cond_to_js` produces for `Cond` (`['always']`,
`['var',name,unit,cmp,value]`, `['varvar',…]`, `['and'/'or',[…]]`,
`['not',[…]]`) — never JS source text, so nothing ever calls `eval`.

Var name convention (`cond_json.rs`, matching
`src/deep/engine_accessories/registry.rs`, the one area registered so far):
a name already carrying a recognised namespace prefix (`L:`, `A:`, `E:`,
`K:`, `H:`, `Z:`, `GAME:`) or containing a space (a bare default MSFS simvar,
e.g. `"GENERAL ENG STARTER:2"`) is passed through as-is; any other bare name
(`"A32NX_ENG_2_HP_PUMP_LOW_FLOW"`) is this plugin's own published variable,
which FlyByWire's JS always reaches as an `L:` var, so `"L:"` is prepended.
Unit is inferred: `.on()`/`.off()` (`Eq`/`Ne` against exactly `0.0`) reads as
`"bool"`, everything else as `"number"`.

`src/deep/ecam/deep_ecam_bridge.js` is the one hand-written, static JS file
(no per-alert data of its own — see its own file doc comment for the full
reasoning). It defines `installDeepEcam(fws, defs)`, which, for each alert:
builds an `EwdAbnormalItem` whose `simVarIsActive` is a tiny stand-in object
(`{get, set, sub}`, **not** a real `Subject` — see below for why that is
both necessary and sufficient) driven by its own confirm-delay/shown-time
state machine (mirroring `Ecam::update` in `api.rs` line for line: `heldS`↔
`held_s`, `shownS`↔`shown_s`), and adds it directly into
`fws.ewdAbnormalSensed`/`fws.ewdAbnormal`/`fws.allSuppressableItems`. It
returns a stepper function the caller calls every tick.

**Why a fake `Subscribable` is correct, not a shortcut**: a real
`Subject`/`MappedSubject` is an `@microsoft/msfs-sdk` class, bundled as a
local, non-exported binding private to `SystemsHost.js`'s own compiled
output — unreachable from a JS file loaded any other way, so
`deep_ecam_bridge.js` could not construct one even if it wanted to.
Checking every call FlyByWire's own code makes on a suppressable item's
`simVarIsActive` (`FwsCore.ts:5497`, `:5511`, and the one-time `.sub()` loop
at `:2358-2366`) shows only `.get()` is ever called on an entry added after
construction — the `.sub()` bookkeeping loop already ran once, over a
snapshot that predates these entries, at `FwsCore`'s own construction time,
before `installDeepEcam` ever runs (see next paragraph) — so `.sub()` on our
entries is provably never invoked, and a no-op is enough for correctness,
not merely adequate in practice.

## 4. The five `SourcePatch`es (`patches.rs::source_patches`)

Every `find` below was copied verbatim from a fresh development build at
`D:\fbw-aircraft\fbw-a380x\out\flybywire-aircraft-a380-842\html_ui` (the
same tree `ecam_patches.rs` already targets) and confirmed to occur exactly
once with:

```
rg -F '<find text>' <file>   # each returns exactly 1
```

| # | file | anchor | effect |
|---|------|--------|--------|
| 1 | `EWD/ewd.js` | the `var EcamAbnormalSensedProcedures = __spreadValues(…)` line (`ewd.js:68967`) | `Object.assign(EcamAbnormalSensedProcedures, <title/items data>)` |
| 2 | `SystemsHost/SystemsHost.js` | the same line, byte-identical (`SystemsHost.js:166034`) | same merge, this bundle's own copy |
| 3 | `SystemsHost/SystemsHost.js` | end of `var EcamInopSys = {…}` (`SystemsHost.js:165977`) | `Object.assign(EcamInopSys, <INOP text data>)` |
| 4 | `SystemsHost/SystemsHost.js` | end of `var EcamMemos = {…}` (`SystemsHost.js:165535`) | `Object.assign(EcamMemos, <STATUS text data>)` |
| 5 | `SystemsHost/SystemsHost.js` | first line of `FwsCore.update(_deltaTime)` (`SystemsHost.js:177576`, unique: `fwsUpdateThrottler` only exists on `FwsCore`) | once per `FwsCore` instance: inlines `deep_ecam_bridge.js`, declares `__deepEcamAlerts` **and `__deepFbwAlerts`**, calls `installDeepEcam` **and `installDeepEcamFbw`**; every tick after that, steps both |

Patch 5 carries the second half of this bridge as well: the entries that
give **FlyByWire's own** abnormal-sensed procedures the trigger they never
had. FlyByWire defines 1004 of them and triggers 273; the other 732 are
text, and -- because the ECL's ABN PROC page renders the same table --
electronic checklists the crew can never be shown. `installDeepEcamFbw`
adds an `EwdAbnormalItem` for FlyByWire's *own* nine-digit id, emitting none
of its text, and sizes each entry's item vectors from the live procedure
(`EcamAbnormalProcedures[id].items.length`, passed in at the call site,
which is in scope there: `SystemsHost.js:166035` declares it and
`FwsCore.update` already reads it at `:179259`). See
`docs/deep/fbw_unwired.md` and `src/deep/ecam/fbw/`.

Patch 5's inserted text, structurally:

```js
const deltaTime = this.fwsUpdateThrottler.canUpdate(_deltaTime);
if (!this.__deepEcamInstalled) {
  this.__deepEcamInstalled = true;
  /* deep_ecam_bridge.js, verbatim */
  var __deepEcamAlerts = [ /* codegen::alerts_js_array */ ];
  this.__deepEcamTick = globalThis.installDeepEcam(this, __deepEcamAlerts);
  var __deepFbwAlerts = [ /* fbw_codegen::fbw_alerts_js_array */ ];
  this.__deepFbwTick = globalThis.installDeepEcamFbw(this, __deepFbwAlerts, EcamAbnormalProcedures);
}
if (this.__deepEcamTick) { this.__deepEcamTick(); }
if (this.__deepFbwTick) { this.__deepFbwTick(); }
```

Guarded to run once per `FwsCore` instance (there are two, FWS1/FWS2,
sharing this compiled text — each gets its own closures, its own
`ewdAbnormalSensed`, entirely independent, exactly like FlyByWire's own two
computers); the actual per-tick trigger/line evaluation runs unconditionally
every tick after that, regardless of `fwsUpdateThrottler`'s own throttling,
so this bridge is at least as responsive as FlyByWire's own alerts, never
less.

Empty input (no area has registered an alert yet) still returns all five
patches, each merging an empty object or installing an empty array — a
harmless no-op, so wiring this in costs nothing before any area is ready.

## 5. Wiring it in (the lead's one-line change)

This directory may not edit any existing file, so this is the exact,
minimal change needed, mirroring how `oans`/`wxr`/`ecam_patches` are already
wired into the same list:

`src/js_bridge.rs`, inside `native_ports()` (around line 991, right after
the existing `ecam_patches` line):

```diff
     // [ecam_patches] ECAM/FWS and instrument gaps (top50 #9, 16, 18-27, 36-40):
     // src/ecam_patches.rs.
     patches.extend(crate::ecam_patches::source_patches());
+    // [deep ecam bridge] registered deep::api::EcamAlert -> FlyByWire's own
+    // FWS tables: src/deep/ecam/patches.rs.
+    patches.extend(crate::deep::ecam::patches::source_patches(&registry.alerts));
     patches
```

(`registry` is whatever `Registry` the lead builds by calling every area's
`register(&mut r)` — `native_ports()` would need it passed in or built
inline; today it takes no arguments, so this also needs threading a
`&Registry` or `&[EcamAlert]` into `native_ports()`'s own signature, the
lead's call).

And `src/deep/mod.rs` (currently only `pub mod api;`, same situation as
every other area's directory here, all likewise not yet wired in):

```diff
 pub mod api;
+pub mod ecam;
```

## 6. Worked example (also `src/deep/ecam/tests.rs`)

Three alerts, one of each level that reaches the abnormal-procedure table:

```rust
EcamAlert::new("ENG_2_OIL_LO_PR", 79, "ENG 2 OIL LO PR", Level::Warning, var("ENGINE_OIL_PRESSURE_PSI:2").lt(25.0))
    .confirm(2.0).inhibit(&[Phase::LiftOff])
    .step(line("THR LEVER 2", "IDLE").done(var("AUTOTHRUST_TLA:2").le(0.0)))
    .step(line("ENG 2 MASTER", "OFF").done(var("ENGINE_MASTER:2").off()).after(30.0))
    .inop_sys("ENG 2 OIL SYSTEM");

EcamAlert::new("GREEN_RSVR_LOW", 29, "GREEN RSVR LO LEVEL", Level::Caution, var("A32NX_HYD_GREEN_RESERVOIR_LEVEL").lt(0.1))
    .confirm(5.0)
    .step(line("GREEN ELEC PUMP", "OFF").done(var("A32NX_OVHD_HYD_EPUMPG_ON_PB_IS_AUTO").off()))
    .status_line("GREEN HYD SYS").inop_sys("GREEN HYD SYS");

EcamAlert::new("ENG_3_FUEL_FILTER_CLOG", 73, "ENG 3 FUEL FILTER CLOG", Level::Advisory, var("A32NX_ENG_3_FUEL_FILTER_IMPENDING_BYPASS").on())
    .confirm(10.0).status_line("ENG 3 FUEL FILTER");
```

`ids::assign` (sorted by key): `ENG_2_OIL_LO_PR` → `1_000_000_000`,
`ENG_3_FUEL_FILTER_CLOG` → `1_000_000_001`, `GREEN_RSVR_LOW` →
`1_000_000_002`.

Static merge into `EcamAbnormalSensedProcedures` (both bundles), abridged:

```js
Object.assign(EcamAbnormalSensedProcedures, {
  1000000000:{title:'ENG 2 OIL LO PR',sensed:true,items:[
    {name:'THR LEVER 2',sensed:true,labelNotCompleted:'IDLE',style:'Cyan'},
    {name:'ENG 2 MASTER',sensed:true,labelNotCompleted:'OFF',style:'Cyan'}]},
  1000000001:{title:'ENG 3 FUEL FILTER CLOG',sensed:true,items:[]},
  1000000002:{title:'GREEN RSVR LO LEVEL',sensed:true,items:[
    {name:'GREEN ELEC PUMP',sensed:true,labelNotCompleted:'OFF',style:'Cyan'}]}
});
```

`EcamInopSys`/`EcamMemos` merges (`SystemsHost.js` only):

```js
Object.assign(EcamInopSys, {1000000000000:'ENG 2 OIL SYSTEM', 1000000002000:'GREEN HYD SYS'});
Object.assign(EcamMemos,   {1000000001500:'ENG 3 FUEL FILTER', 1000000002500:'GREEN HYD SYS'});
```

Behaviour array fed to `installDeepEcam`, abridged (one entry shown):

```js
{id:1000000000,failure:3,sysPage:0,flightPhaseInhib:[6],confirmS:2,
 trigger:['var','L:ENGINE_OIL_PRESSURE_PSI:2','number','lt',25],
 items:[
   {appliesIf:['always'],doneWhen:['var','L:AUTOTHRUST_TLA:2','number','le',0],afterS:0},
   {appliesIf:['always'],doneWhen:['var','L:ENGINE_MASTER:2','bool','eq',0],afterS:30}],
 inopIds:[1000000000000],infoIds:[]}
```

End to end: oil pressure below 25 psi for 2 s → `simVarIsActive` flips true
→ `FwsCore`'s own loop (unmodified) sees `failure:3`, requests the master
warning and CRC, shows `ENG 2 OIL LO PR` on the EWD with its two lines
(`THR LEVER 2 …IDLE`, `ENG 2 MASTER …OFF` after 30 s), and lists `ENG 2 OIL
SYSTEM` under INOP SYS on the STATUS page. Retarding thrust lever 2 writes
`AUTOTHRUST_TLA:2`, which the very next tick's `doneWhen` check reads
through the same `Cond` → line ticks cyan→green, exactly as pressing the
real cockpit control would.

## 7. Known scoping decisions / limitations

- **Phase mapping is lossy in one direction only**: our `Phase` (10
  variants) has no separate V1 or 400 ft phase; each maps to the nearest
  `FwcFlightPhase`, always erring toward a *wider* inhibit window, never
  narrower — see `codegen::fwc_phase_number`'s doc comment for the exact
  per-arm mapping.
- **`sysPage` is a heuristic** from the ATA chapter (`codegen::sys_page_for_ata`),
  since `EcamAlert` carries no SD page of its own; defaults to `Status`.
  Adding a real field for it is `api.rs`'s owner's call, not made here.
- **"AFTER n S" is not rendered as text**: FlyByWire's own `TimedChecklistAction`/
  `TimedChecklistCondition` machinery (a `time` field plus
  `whichItemsTimer()`/`appendTimeIfElapsed`) would be needed for the literal
  countdown label; this bridge instead hides the line until `afterS` elapses
  (via `whichItemsToShow`) without appending that label, which is correct
  timing with a cosmetic gap, documented rather than half-implemented against
  a shape not fully sourced.
- **LIMITATIONS (`EcamLimitations`) is not wired**: only STATUS "INFO"
  (`.status_line`) and INOP SYS (`.inop_sys`) are, both explicitly named by
  the task; a third dict/mechanism for limitations would follow the exact
  same pattern if a future alert needs it.
- **Title escape-code category tags** (FlyByWire's own `\x1b<4m…\x1bm`
  chapter-colour prefix convention) are deliberately not reproduced — no
  full per-ATA table for it was sourced, and `EcamAlert::title` is already
  documented as "the title as shown"; adding an unsourced guess at the
  colour-code table would violate the no-fake-values rule.
- **Id stability**: ids are recomputed by sorting every registered alert's
  `key` each time `ids::assign` runs, not persisted anywhere; stable within
  one run (registration order in code is irrelevant, only the *set* of keys
  matters), but a key added or removed between two runs can shift other
  alerts' ids. Nothing in this bridge or in FlyByWire's own model persists
  a `ChecklistState` keyed by these ids across a restart, so this has no
  observed effect; flagged here in case a future persistence feature would
  need a stable id instead (e.g. a hash of `key`, deliberately not done here
  to keep ids human-traceable during development).

## 8. Tests

`src/deep/ecam/{ids,cond_json,codegen,patches}.rs` each carry `#[cfg(test)]`
unit tests of their own piece (id uniqueness/determinism, var-name/unit
inference, per-`Cond`-shape encoding, static-vs-behaviour data separation,
patch anchor/replace well-formedness, empty-input no-ops). `tests.rs` is the
worked example above, end to end: the same three alerts carried through
`ids::assign` → every `codegen` function → `patches::source_patches`,
checking every id agrees across all four outputs and nothing from one alert
leaks into another's entry or dict slot.
