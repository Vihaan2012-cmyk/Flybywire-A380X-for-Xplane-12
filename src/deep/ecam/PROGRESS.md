- [done] Studied FlyByWire's A380 FWS (FwsCore.ts, FwsAbnormalSensed.ts, EcamMessages/index.ts + AbnormalSensed/ata*.ts, EcamSystemPages.ts, FwsFlightPhases.ts) and this port's JS runtime (src/js/msfs/mod.rs's per-view `Cockpit`/`Engine`, `SourcePatch` mechanism in js_bridge.rs::native_ports/ecam_patches.rs, src/js/msfs/simvar.js's `SimVar` global) — findings in docs/deep/ecam_bridge.md sections 1-2. Confirmed against the actual built tree at `D:\fbw-aircraft\fbw-a380x\out\flybywire-aircraft-a380-842\html_ui` (exists, pre-built).
- [done] `src/deep/ecam/ids.rs` — deterministic 10-digit id assignment (`ID_BASE = 1_000_000_000`), outside FlyByWire's whole 9-digit id space by construction. Tests: uniqueness, order-independence.
- [done] `src/deep/ecam/cond_json.rs` — `Cond` -> tagged-array JS encoding (`['var',name,unit,cmp,value]` etc., no `eval`), var-name L:-prefix convention, unit inference (`.on()/.off()` -> `"bool"`). Tests: prefix rule, unit inference, nesting, escaping.
- [done] `src/deep/ecam/codegen.rs` — static data (`procedures_merge_js`, `inop_merge_js`, `info_merge_js`) and behaviour data (`alerts_js_array`) generators; ATA->SdPages heuristic; Phase->FwcFlightPhase mapping. Tests: shape/content per function, failure-level mapping, id uniqueness across procedure + derived STATUS/INOP ids.
- [done] `src/deep/ecam/deep_ecam_bridge.js` — static, hand-written JS shim (`installDeepEcam`): builds a fake `Subscribable`-shaped flag (no real msfs-sdk `Subject` needed or reachable — see its file doc comment), runs its own confirm-delay/shown-time state machine mirroring `Ecam::update` in api.rs, adds entries into `fws.ewdAbnormalSensed`/`ewdAbnormal`/`allSuppressableItems`.
- [done] `src/deep/ecam/patches.rs` — the 5 `SourcePatch`es (2x `EcamAbnormalSensedProcedures` in EWD.js+SystemsHost.js, `EcamInopSys`, `EcamMemos`, `FwsCore.update()` install/step, all SystemsHost.js), every anchor's exact text confirmed to occur exactly once in the built tree via `rg -F`. Tests: patch count/target file, anchor preserved verbatim, empty-input no-ops, generated data embedded correctly.
- [done] `src/deep/ecam/tests.rs` — worked example, 3 alerts (Warning/Caution/Advisory) carried end to end through ids -> every codegen function -> patches::source_patches, cross-checking ids agree everywhere and nothing leaks between alerts.
- [done] `docs/deep/ecam_bridge.md` — full design doc: FlyByWire's data model, this port's JS bridge, the design, all 5 patches with anchors, the exact one-line `src/js_bridge.rs` + `src/deep/mod.rs` changes the lead applies (not made here, existing files), the worked example, scoping decisions/limitations, test summary.

New Vars this bridge itself needs published: none — it only reads whatever
Vars each area's own `EcamAlert`s already name (documented in each area's
own PROGRESS.md), through the existing `SimVar.GetSimVarValue` / `L:` bridge
already used throughout this port.

Not done (out of this directory's scope, or deferred, see ecam_bridge.md
section 7 for why): `EcamLimitations` (STATUS "LIMITATIONS" section — only
STATUS "INFO" and INOP SYS are wired, the two the task named explicitly);
FlyByWire's own `\x1b<4m…` chapter-colour title tag convention (no sourced
per-ATA table); a real per-alert `sysPage` field (would need a change to
`api.rs`, owned by the lead); `TimedChecklistAction`'s literal "AFTER n S"
label (timing is correct via `whichItemsToShow`, the countdown text is not
rendered).
