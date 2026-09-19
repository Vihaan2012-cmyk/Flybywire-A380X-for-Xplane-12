# Stage 2 analysis: ECAM, instruments/FMS, C++ computers, X-Plane coupling, JS runtime

Read-only analysis (no source files edited). No sub-agents were used; all
findings below come from direct grep/read of D:\fbw-aircraft (FBW's
TypeScript/C++/Rust, unmodified), D:\fbw-xp-systems (the Rust plugin) and
D:\msfs2xp-aircraft (converter). `docs/cl650-reference.md` does not exist yet,
so it was not used as a yardstick.

**Method note (scope vs. effort):** the in-scope FWS/instrument TypeScript is
~38,000 lines (FwsCore.ts alone is 6,266 lines; FlightManagementComputer.ts +
FmcAircraftInterface.ts + fmgc.ts are ~4,000; MFD pages/common ~218 marked
spots). A single pass with grep + targeted reads (no sub-agents, per brief)
cannot cite every one of FBW's own several hundred `TODO`/`FIXME` comments
individually without turning this report into a mechanical transcript. Where
several markers describe the same underlying shortcut (e.g. a run of
`// TODO secondary flight plans` returning `null`), they are reported as one
gap citing the full line range. Counts below (alert/procedure/INOP-system
counts, TODO counts per directory) are exact `grep -c` results, not estimates.

## 1. Summary of all gaps (sorted by impact, then effort)

| Impact | Effort | ID | System / ATA | One-line gap |
|---|---|---|---|---|
| 4 | S | INST-005 | FMS / ATA 22,34 | V-speeds-too-low takeoff cross-check hard-disabled |
| 4 | L | CPU-010 | FCDC / ATA 27,31 | FCDC bus words + spoiler LVars still WIP — no live F/CTL data on PFD/SD/FWS |
| 4 | L | INST-004 | FMS VNAV / ATA 22 | Decel distance and speed-constraint math are placeholders |
| 4 | XL | INST-001 | FMS / ATA 22,34 | Secondary flight plan entirely unimplemented (10+ accessors return null) |
| 4 | XL | XP-003 | Flight model / ATA 27,57 | `[AERODYNAMICS]`/`[FLIGHT_TUNING]`/`[STALL PROTECTION]`/`[FLAPS.*]` not read by converter; XP's own blade-element aero used |
| 3 | M | ECAM-003 | Air/pressurisation / ATA 21,52 | On-ground pack-overheat + door fault conditions never added |
| 3 | M | ECAM-010 | Elec/hyd/reversers / ATA 24,36,78 | GEN/IDG/bleed/reverser INOP derived only from "engine out", no contactor/hydraulic checks |
| 3 | M | CPU-003 | PRIM/SEC / ATA 27 | Pitch trim switches hard-coded `false` |
| 3 | M | CPU-011 | PRIM/SEC/FCU / ATA 22,27 | FailuresConsumer never wired to the compiled C++ computers |
| 3 | M | XP-001 | Reversers / ATA 78 | Reverser delta-speed/asymmetry-yaw model MISSING; X-Plane's own reverse thrust substitutes |
| 3 | M | XP-004 | Fuel / ATA 28 | Fuel-pump `CIRCUIT CONNECTION ON:n` pushbuttons not honoured |
| 3 | M | JS-010 | SD / ATA 31 | 10 of 14 SD system pages exist only as legacy React; DOM fidelity unverified |
| 3 | L | ECAM-006 | Pressurisation / ATA 21 | Manual pressurisation / sensor-failure logic not simulated |
| 3 | L | ECAM-014 | FADEC/ECU / ATA 73,77 | ECU doesn't expose discrete words FWS needs; engine-running/idle detection falls back to TLA-only |
| 3 | L | ECAM-015 | Multiple / ATA 27,34 | Attitude source from knob (not CDS), no SFCC1/2 switching, simplified flap/slat detection, no rudder-fault detection |
| 3 | L | ECAM-018 | ECAM control panel / ATA 31 | Dual-FWS-both-failed backup path incomplete/"convoluted" |
| 3 | L | INST-003 | ATSU / ATA 23,46 | ATSU/CPDLC never hooked up for the A380X variant |
| 3 | L | INST-006 | ND VD / ATA 34 | Vertical Display descent profile never drawn |
| 3 | XL | INST-002 | FMS / ATA 22,34 | No FMC-A/B/C sync; "master" is always the first FMC, no voting |
| 2 | S | ECAM-005 | F/CTL / ATA 27 | Speedbrake-INOP source is the FCDC command signal, not lever position |
| 2 | S | ECAM-007 | Brakes / ATA 32 | Antiskid fault checks only the switch, not power/fault signal |
| 2 | S | ECAM-009 | Autocallouts / ATA 78 | Max-reverse callout never checks reverser INOP |
| 2 | S | ECAM-013 | Air data / ATA 34 | FWS ground speed sourced incorrectly, not from CDS |
| 2 | S | CPU-005 | PRIM/SEC/FCU / ATA 22,34 | FMS LVars (flight phase, V2, FG/FM words) read as 0 until the JS FMS starts writing them |
| 2 | M | ECAM-001 | Perf limitations / ATA 22,34 | "More restrictive speed limitation" logic not implemented |
| 2 | M | ECAM-002 | Engine/anti-ice / ATA 30,73 | Derated-climb, soft-GA and MFP-heating conditions hard-coded `false` |
| 2 | M | ECAM-008 | Surveillance / ATA 34 | SURV SYS INOP logic stubbed to `false` pending "once implemented" |
| 2 | M | ECAM-012 | Cabin/elec / ATA 21,24 | Elec galley/pax-sys "off" reuses the pushbutton simvar instead of an independent sense |
| 2 | M | ECAM-017 | Aurals / ATA 31,44 | FwsSoundManager: some aurals (e.g. CIDS chimes) never wired |
| 2 | M | INST-008 | PFD / ATA 34 | LS indicator: RNAV path and LOC/G-S-invalid (MMR word) checks not implemented |
| 2 | M | INST-009 | PFD/F-CTL / ATA 27,32 | Spoiler indication uses one spoiler's commanded position; LGCIS not modeled |
| 2 | M | CPU-001 | PRIM/SEC / ATA 34 | `CalculatedRadioReceiver` not ported; raw receiver-3 data used regardless of the option |
| 2 | M | CPU-004 | PRIM/SEC / ATA 27 | Rudder trim switches hard-coded `false` |
| 2 | M | CPU-006 | FCU / ATA 22 | FCU value-set events and EFIS panel events read as "no input" outside initialisation |
| 2 | M | XP-005 | Electrical/lighting / ATA 33 | MSFS light circuits don't follow FBW buses; X-Plane's own lighting decides |
| 2 | M | JS-003 | Canvas / ATA 34 | `drawImage()` only supports `<img>` sources (no canvas/video compositing) |
| 2 | L | ECAM-016 | TCAS/STATUS / ATA 31,34 | Single TCAS fault channel; STATUS ordering is colour-only, not importance-ranked |
| 2 | L | INST-012 | MFD input / ATA 22,31 | KCCU (Keyboard & Cursor Control Unit) not modeled anywhere |
| 2 | L | JS-004 | Canvas / ATA 34 | `getImageData`/`putImageData` unsupported (pixel-level compositing breaks) |
| 2 | L | JS-005 | SVG / ATA 31 | `getCTM`/`getScreenCTM`/`getTotalLength` unsupported |
| 2 | XL | XP-002 | Engine / ATA 71,73 | No true gas-generator/core physics; thrust and spool dynamics are X-Plane's generic turbofan model |
| 1 | S | ECAM-004 | Cabin/IFEC / ATA 23,44 | IFEC overhead pushbutton check stubbed to always-checked |
| 1 | S | ECAM-011 | Cabin/elec / ATA 24 | (see ECAM-012; low-impact half of the same finding) |
| 1 | S | INST-007 | ND/OANS / ATA 34 | Airport auto-select uses raw GPS, not GPS+IRS blended position |
| 1 | S | INST-010 | PFD / ATA 27 | Pitch-trim display: no-CG-available / WBBC-fallback case unhandled |
| 1 | S | INST-013 | RMP / ATA 23 | Audio management: "only two TX active" enforced by a TODO stub |
| 1 | S | CPU-002 | PRIM/SEC / ATA 34 | Localizer distance without DME hard-coded 0 |
| 1 | S | CPU-007 | PRIM/SEC / ATA 34 | Vertical/lateral accelerometers, ISIS, rate gyros zeroed (inherited from FBW's own MSFS hard-code) |
| 1 | S | CPU-008 | FCU / ATA 22 | `idFm1BackbeamSelected` never created upstream (inherited FBW gap) |
| 1 | S | CPU-009 | FCDC / ATA 27 | `SimData.simData` never read by `Fcdc.cpp` upstream (documentation-only gap) |
| 1 | S | XP-006 | Autoflight / ATA 22 | Sim rate not limited while AP is engaged (`handleSimulationRate` MISSING) |
| 1 | S | XP-007 | Lighting / ATA 33 | LightSync auto-brightness inputs (`GLASSCOCKPIT AUTOMATIC BRIGHTNESS`, time of day, on-runway) unfed |
| 1 | S | XP-008 | Performance monitor / ATA 34 | `A32NX_PERFORMANCE_WARNING_ACTIVE` never set (MISSING) |
| 1 | S | JS-008 | CSS / ATA 31 | `animation-play-state: paused` ignored, animations always run |
| 1 | S | JS-009 | DOM/window / ATA 31 | `location.reload()` and similar window APIs unsupported |
| 1 | M | INST-011 | PFD / ATA 34 | ARINC/air-data-source selection logic lives in the instrument instead of the PRIM output |
| 1 | M | JS-001 | Canvas / ATA 34 | Canvas fill/stroke patterns unsupported (ignored) |
| 1 | M | JS-002 | Canvas / ATA 31 | `isPointInPath`/`isPointInStroke` always return `false` |
| 1 | M | JS-006 | CSS / ATA 31 | CSS gradients unsupported entirely (backgrounds render flat) |
| 1 | M | JS-007 | CSS / ATA 31 | CSS grid `repeat()`/named lines partly unsupported |

60 gaps total (18 ECAM, 13 instruments/FMS, 11 C++ computers, 8 X-Plane
coupling, 10 JS runtime). See per-system sections below for evidence,
real-aircraft behaviour and proposals. The "Top 50" list is at the end.

## 2. ECAM (FlightWarningSystem, EWD, SD/SDv2)

### 2.1 What's there (counts, `grep -c` on
`fbw-a380x/src/systems/systems-host/CpiomC/FlightWarningSystem`)

| Item | Count | Source |
|---|---|---|
| Abnormal **sensed** procedures (`EcamAbnormalSensedProcedures`) | 273 | FwsAbnormalSensed.ts |
| Abnormal **non-sensed** procedures | 18 | FwsAbnormalNonSensed.ts |
| Normal checklists (phases, `NormalProcedures.ts`) | 14 (COCKPIT PREP … SECURING) | MFD/FMC's `EcamMessages/NormalProcedures.ts` |
| Sensed (auto-tickable) normal-checklist items | 14 | FwsNormalChecklists.ts |
| Limitations entries | 9 | FwsLimitations.ts |
| Memo entries (9-digit codes found) | 11 (38 keys total; some use other id widths) | FwsMemos.ts |
| INOP SYS entries | 119 | FwsInopSys.ts |
| Abnormal-sensed by ATA prefix | 21: 85, 34: 35, 27: 32, 26: 28, 29: 22, 22: 14, 52: 11, 70: 10, 32/28: 9 each, 99: 7, 23: 7, 31: 4 | FwsAbnormalSensed.ts |

This is FBW's real, dual-FWS architecture running unmodified as JS: `fwsNumber:
1 | 2` (FwsCore.ts:2332), independent `masterWarning`/`masterCaution`
subjects driving `L:A32NX_MASTER_WARNING`/`_CAUTION` (FwsCore.ts:310-326,
2562-2564), `NXLogicConfirmNode` confirmation timers from 3s (takeoff/landing
inhibit, FwsCore.ts:1471,1473) to 180s (FwsCore.ts:385), phase-based
inhibition sets `phase12561112Inhibition`/`phase56Inhibition`
(FwsCore.ts:2008,2010), and Arinc429 SSM handling (`isFw`/`isNo`,
extra_backend_fcdc.rs:62-68, ported line-for-line from `Arinc429Utils.cpp`).
This is genuinely deep and should not be re-implemented; the gaps below are
specific logic FBW itself has not finished for the real A380 (not
X-Plane-port shortcuts), plus the plugin-side wiring the FWS still needs.

### 2.2 Gaps

| ID | Evidence (fbw-a380x/src/systems/systems-host/CpiomC/FlightWarningSystem/) | What the real aircraft does | Proposal |
|---|---|---|---|
| ECAM-001 | FwsAbnormalNonSensed.ts:592,605 | Overspeed/limitation ECAM logic on the real aircraft picks the *most restrictive* active speed limitation among several sources | Add the "more restrictive" comparison FBW's own comment calls for |
| ECAM-002 | FwsAbnormalNonSensed.ts:671,707,715,734,789,864 | Derated-climb-engaged, MFP-heating-failed (≥2) and soft-GA-lost are real discrete engine/anti-ice states that gate several abnormal procedures | Wire the real conditions once the upstream engine/anti-ice model exposes them; until then these procedures under-trigger |
| ECAM-003 | FwsAbnormalSensed.ts:530; FwsCore.ts:4151-4152 | Door-open-with-engine-running-on-ground and pack-overheat are separate sensed faults | Add the conditions FBW's own comments describe |
| ECAM-004 | FwsAbnormalSensed.ts:973 | IFEC overhead pushbutton state should be read, not assumed pressed | Read the actual IFEC PB simvar once modeled |
| ECAM-005 | FwsAbnormalSensed.ts:1028 | Speedbrake-related logic should use the physical lever position, not the FCDC's commanded signal | Swap the source once the FWS's own FIXME is addressed upstream |
| ECAM-006 | FwsAbnormalSensed.ts:1088,1094,1099,1310,1313 | Real A380 CPC has a manual backup with its own sensor-failure and ambient-pressure-unavailable logic | Not simulated at all currently; needs the CPC manual mode modeled first |
| ECAM-007 | FwsAbnormalSensed.ts:3615 | Antiskid fault should also reflect power loss / a hardware fault signal, not just the switch position | Add the fault-signal input once available |
| ECAM-008 | FwsAbnormalSensed.ts:4229 | SURV SYS (transponder/TCAS/weather radar/terrain) group INOP display should reflect real system health | Hard-coded to never show; wire once a SURV SYS aggregate exists |
| ECAM-009 | FwsAutoCallouts.ts:56,59 | MAX REVERSE auto callout should suppress if a reverser is INOP | Add the INOP check FBW's own FIXME calls for |
| ECAM-010 | FwsCore.ts:2261,2286-2298,2290-2296 | GEN LO/bleed/reverser-INOP indications on the real aircraft come from contactor-open, IDG-disconnect, hydraulic-loss and power-loss discretes, not just "engine out" | Layer in the extra discretes as the ECU/hydraulic models mature |
| ECAM-011/012 | FwsCore.ts:2991 | ELEC GALLEY/PAX SYS "OFF" should be sensed independently of the pushbutton (real aircraft: contactor position) | Split the simvar so pushbutton state and actual contactor state differ under a fault |
| ECAM-013 | FwsCore.ts:3822 | Ground speed used by FWS logic should come from the CDS, not the substitute source currently used | Route through the CDS-equivalent bus once available |
| ECAM-014 | FwsCore.ts:1906,1931,3847,4267,5266 | Real ECU/FADEC outputs discrete words for engine-running/idle/starting state; FWS currently infers this from TLA and ignition timing alone | Requires FADEC (fadec.rs) to expose the missing discretes; until then, thrust-lever-only inference can mis-detect engine state transients |
| ECAM-015 | FwsCore.ts:4460,4638,4699,4774 | IR3 selection should come from the CDS, not straight from the attitude knob; SFCC1/2 switching and rudder-fault detection are real discrete logic paths | Multiple related simplifications in one function group; fix together once the CDS/SFCC bus words exist |
| ECAM-016 | FwsCore.ts:5017,4339,6206,5985-6027 | Real A380 has genuinely independent TCAS 1/2 fault reporting; STATUS page item ordering is procedure-priority based, not purely colour | FBW's own comment: "TODO order by decreasing importance" (appears 4×, FwsCore.ts:5985,5996,5999,6027) — currently colour-only |
| ECAM-017 | FwsSoundManager.ts:278 | Every ECAM aural (including CIDS door chimes) should play | FBW's own comment flags some sounds as not wired; low effort once sound module (owned by sound engineer) exposes them |
| ECAM-018 | FwsSystemDisplayLogic.ts:75,100-101 | ECAM Control Panel pushbuttons are hard-wired to the CDS even with both FWS failed on the real aircraft | FBW's own comment calls the current logic "convoluted" and flags the both-FWS-failed backup path as needing a rewrite |

### 2.3 SD page data liveness

SD (legacy React, `SD/Pages/*`) has all 13 system pages (ENG, BLEED, COND,
DOOR, ELEC AC/DC, HYD, PRESS, WHEEL, APU, FUEL, CB) plus STATUS. SDv2
(FSComponent rewrite) currently only has CRUISE, F-CTL, STATUS and VIDEO
(`fbw-a380x/src/systems/instruments/src/SDv2/Pages/`: `Cruise/`, `Fctl/`,
`Generic/`, `Status/`, `Video/` — no ENG/BLEED/ELEC/HYD/etc.). All system-page
data therefore still flows through the legacy React 17 SD, so its data
liveness depends on whatever simvars/LVars the plugin publishes for
electrical, hydraulic, bleed, fuel and pressurisation systems (already
"ported" per `docs/systems-coverage.md`). The one page that is live only on
SDv2 today, F-CTL, depends on FCDC bus words and the SPOILERS LVars that
`extra_backend_fcdc.rs` marks "in progress" — see CPU-010.

## 3. Instruments and FMS

### 3.1 TODO/FIXME density (`grep -c` per instrument directory,
`fbw-a380x/src/systems/instruments/src/`)

| Instrument | Count |
|---|---|
| MFD (incl. FMC) | 218 |
| PFD | 12 |
| ND | 9 |
| RMP | 13 |
| FCU | 0 |
| ISISlegacy | 0 |

MFD/FMC breaks down as: `pages/common` 98 (shared widgets: dropdowns, keypad,
scratchpad — mostly code-quality, not simulation), `FMC/` 59 (the actual
flight-management logic — the gaps that matter), `pages/FMS/F-PLN/` 21,
`pages/FMS/POSITION/` 8, direct `pages/FMS/*.tsx` (INIT/PERF/FUEL&LOAD/MSG
LIST) 4 files with hits, `pages/SURV` 1, `pages/ATCCOM` 0, `pages/FMS/SEC/`
and `pages/FMS/DATA/` 0 (no markers — but SEC pages exist only as shells, see
INST-001).

### 3.2 Gaps

| ID | Evidence | What the real aircraft does | Proposal |
|---|---|---|---|
| INST-001 | `MFD/FMC/FlightManagementComputer.ts:703,711,719,727,735,742,750,758,765` | The A380 FMS supports a full secondary flight plan (alternate routing, comparison, activation) | Every accessor (`secondaryFlightPlan`, etc.) currently `return null`. This is upstream FBW work, not a port issue — needs `fmsv2`/FlightPlanService secondary-plan support, which FBW itself hasn't shipped for the A380 yet |
| INST-002 | `MFC/FmcService.ts:99`; `FlightManagementComputer.ts:310,343,1455` | Real A380 has FMC-A/B/C in a master/slave/standby arrangement with cross-talk sync | FBW's own comment: "as soon as we have multiple real FMCs, we need to properly implement master/slave/standby logic" — currently the first FMC is always master, others null |
| INST-003 | `MFC/FlightManagementComputer.ts:1734` | ATSU/CPDLC datalink | FBW's own comment: "reset ATSU when it is added to A380X" — not yet added for this variant. Depends on SimBridge/Hoppie networking (teammate scope per `docs/systems-coverage.md`) |
| INST-004 | `MFC/FmcAircraftInterface.ts:899,1467,1486` | VNAV deceleration-distance and speed-constraint calculations are real trajectory-prediction math | FBW's own comments: "big hack until VNAV can do this", "TODO better decel distance calc", "FIXME proper decel calc" |
| INST-005 | `MFC/FmcAircraftInterface.ts:635` | Real FMS cross-checks entered V-speeds against performance limits and flags them if too low | `const toSpeedsTooLow = false; // FIXME revert once speeds are checked this.getToSpeedsTooLow();` — the check exists but is disabled. Safety-relevant, one-line fix once its dependency is verified |
| INST-006 | `ND/VerticalDisplay/VerticalDisplayCanvasMap.tsx:161` | ND Vertical Display shows the predicted descent profile | FBW's own comment: "add descent profile when its display conditions are better understood" — never drawn |
| INST-007 | `ND/OansControlPanel.tsx:188` | Real ND/OANS airport auto-selection uses GPS position interpolated with IRS velocity for smoothness/accuracy | Currently raw GPS only |
| INST-008 | `PFD/LandingSystemIndicator.tsx:450,679,688` | LOC/G-S scale-invalid flagging comes from the MMR (multi-mode receiver) ARINC words; RNAV approach path uses its own simvar | Both marked TODO pending RNAV/MMR-word wiring |
| INST-009 | `PFD/LowerArea.tsx:59,540` | Spoiler position shown should be the FCDC-averaged/maxed value across all spoiler panels; ground-spoiler indication uses real LGCIS (landing gear control/interface system) | Currently one spoiler's commanded position stands in, and LGCIS input is a placeholder pending its implementation (ties to CPU-010) |
| INST-010 | `PFD/PitchTrimDisplay.tsx:90-91` | Pitch trim reference uses FQMS CG, falling back to WBBC CG if FQMS is unavailable | No-CG-available case unhandled |
| INST-011 | `PFD/SpeedIndicator.tsx:998` | Air data reference speed (ARS) selection is a PRIM output on the real aircraft | Currently computed in the PFD instrument itself — architecturally should move to prim.rs |
| INST-012 | (no hits anywhere in `fbw-a380x/src/systems` or `fbw-xp-systems/src`) | The real A380 pedestal has physical KCCUs (trackball + keyboard) as an alternate MFD input device, independent of the touchscreen | Not modeled at all — MFD is touch/click only. Low priority unless a hardware KCCU device is wanted in the cockpit |
| INST-013 | `RMP/Systems/AudioControlManager.ts:186` | Real AMU enforces transmitter-selection priority rules across all RMPs | `// TODO only two tx can be active at a time` — a simplified cap stands in |

## 4. The C++ computers (PRIM, SEC, FCU, FCDC, FADEC, autoflight)

`prim.rs`'s `UNAVAILABLE` const (prim.rs:34-46) and
`extra_backend_fcdc.rs`'s (extra_backend_fcdc.rs:42-45) are the plugin's own
authoritative lists of what these compiled C++ computers read from
placeholders instead of real inputs. `afs_events.rs` and `fbw_controllers.rs`
(autoflight event handling) have no such markers — no gaps found there
beyond what's listed.

| ID | Evidence | Computer reads instead | Real input | Proposal |
|---|---|---|---|---|
| CPU-001 | prim.rs:36-38 | Receiver-3 raw sim data (option-off behaviour always) | `CalculatedRadioReceiver.cpp` (cpp:1138-1154) filters/validates the raw receiver | Port `CalculatedRadioReceiver`; low priority since FBW ships with the option off by default |
| CPU-002 | prim.rs:39 | `0` | Localizer distance derived from DME when available (cpp:1158) | Feed DME distance once ILS/DME sensing (sensors.rs) exposes it |
| CPU-003 | prim.rs:40 | `false` | Manual pitch-trim switch discretes (`SimInputPitchTrim`, cpp:1576-1577,2113-2114) | Wire the cockpit's pitch trim switch axis/buttons; affects manual trim feel in alternate/direct law |
| CPU-004 | prim.rs:41 | `false` | Manual rudder-trim switch discretes (cpp:2110-2112) | Same as above for rudder trim |
| CPU-005 | prim.rs:42 | `0` | `A32NX_FMGC_FLIGHT_PHASE`, `AIRLINER_V2_SPEED`, `A32NX_SPEEDS_*`, `A32NX_FG_*`, `A32NX_FM1_*` | These are written once the JS-runtime FMS starts (systems-host); the note documents a startup-sequencing dependency worth verifying explicitly, since a PRIM tick before the JS host is up would see stale zeros |
| CPU-006 | prim.rs:43 | `-1` / no input | `A32NX.FCU_SPD_SET/HDG_SET/ALT_SET/VS_SET` value events and EFIS panel events, except at FCU init | Route these MSFS-style value events through key_events.rs once the FCU cockpit controls generate them |
| CPU-007 | prim.rs:44 | `0` / zeroed bus | Vertical/lateral accelerometers, ISIS, rate gyros (cpp:1605-1610,1622-1629) | FBW's own C++ hard-codes these even in the MSFS build, so this is inherited, not port-specific — low priority |
| CPU-008 | prim.rs:45 | `false` | `idFm1BackbeamSelected` (cpp:1658) — never created in FBW's own `setupLocalVariables` | Inherited FBW gap, not actionable from the port side |
| CPU-009 | extra_backend_fcdc.rs:42-45 | n/a (documentation only) | `SimData.simData` (FcdcIO.h:73) — `Fcdc.cpp` itself never reads it upstream | No action needed; documents that this isn't a port omission |
| CPU-010 | extra_backend_fcdc.rs (whole file, "in progress" per team.md); `docs/systems-coverage.md`: "FCDC x2 … in progress", "updateSpoilers … in progress" | FCDC bus words feed FWS, SD F/CTL page and PFD; `A32NX_SPOILERS_ARMED`/`_HANDLE_POSITION` feed PFD, FWS, sound.xml and presets | Until this lands, F/CTL-page and PFD spoiler/flight-control-law indications either read defaults or are incomplete. Highest-impact item in the C++ computer set because it gates INST-009, part of ECAM's F/CTL SD page and PFD lower area | Finish the FCDC port (owned per team.md, in progress) |
| CPU-011 | `docs/systems-coverage.md`: "FailuresConsumer for the C++ computers … MISSING: failures reach the Rust systems only", prim.rs `UNAVAILABLE` region | Real PRIM/SEC/FCU respond to their own failure injection (`FailureList.h` ids: Fcu1/2=22002/22003, Prim1-3=27000-27002, Sec1-3=27003-27005 — already defined as consts in prim.rs:48-56 but not consumed by the compiled computers) | Wire `failures::active_ids()` into the C++ shim's FailuresConsumer equivalent so injected PRIM/SEC/FCU failures actually change computer behaviour, not just the Rust systems' view of them |

## 5. X-Plane coupling

### 5.1 What already comes from FBW, not X-Plane (verified, not gaps)

- **Surface rates**: ailerons/elevators/rudder/spoilers/THS positions are
  written directly from `a380_systems`' hydraulic actuator model
  (flight_controls.rs:1-33), with X-Plane's own joystick-to-surface path
  switched off via `override_control_surfaces`. Rates are FBW's, not X-Plane's
  generic actuator-rate limiter.
- **Brake force**: `BRAKE LEFT/RIGHT FORCE FACTOR` (FBW's own brakes.rs output,
  already accounting for autobrake/antiskid) drives X-Plane's
  `left_brake_ratio`/`right_brake_ratio` (handling.rs:554-558). Brake
  temperature (`L:A32NX_REPORTED_BRAKE_TEMPERATURE_n`, read by
  `SD/Pages/Wheel/elements/Wheel.tsx:21`) is written by
  `a380_systems/src/hydraulic/mod.rs` (FBW's own thermal model), which runs
  unchanged in the plugin — not X-Plane's brake-temp dataref.

### 5.2 Gaps

| ID | Evidence | What decides today | What the real aircraft/FBW model does | Proposal |
|---|---|---|---|---|
| XP-001 | `docs/systems-coverage.md` reversers row; lib.rs/throttle.rs | X-Plane's own engine reverse-thrust model | FBW applies `REVERSER_DELTA_SPEED` to `VELOCITY BODY Z` and yaw from `REVERSER_ANGULAR_ACCELERATION` for asymmetric reverse | Apply FBW's reverser delta-speed/yaw the way pushback.rs already writes velocity directly, instead of relying on X-Plane's reverse-thrust animation |
| XP-002 | fadec.rs:9-13 (file header, verbatim: "this is not a thrust model … the simulator's own engine makes the thrust") | X-Plane's generic two-spool turbofan model provides thrust, N1, N2; FBW's FADEC only overlays N3/EGT/fuel-flow/oil computations on top | Real A380 Trent 900 has a three-spool gas generator with its own surge margins and transient response | This mirrors FBW's own MSFS architecture (also not a thrust model there), so it is an inherited limitation, not a port regression. A true core-physics model is out of scope for this port; flag for awareness only |
| XP-003 | `src/acf.rs` (D:\msfs2xp-aircraft): section-name grep finds `WEIGHT_AND_BALANCE`, `CONTACT_POINTS`, `FUEL_SYSTEM`, `AIRPLANE_GEOMETRY`, `REFERENCE SPEEDS` read, but no occurrence of `AERODYNAMICS`, `FLIGHT_TUNING`, `STALL PROTECTION`, or `FLAPS.0/1/2` (flight_model.cfg sections at lines 642, 741, 848, 858/877/897) | X-Plane derives lift/drag/moments from the .acf's blade-element geometry and the aircraft's own control-surface/flap deflection tables, not from MSFS's coefficient tables | FBW tunes `[AERODYNAMICS]`/`[FLIGHT_TUNING]`/`[STALL PROTECTION]`/flap lift-drag increments specifically for the MSFS flight model; X-Plane's aero is a structurally different model that can't be "converted" 1:1 | For the lead (converter-only area): confirm the hand-tuned X-Plane blade-element coefficients (stall AoA, flap CLmax/CD increments, control-surface hinge moments) were validated against FBW's own numbers/POH data rather than left at X-Plane's geometry-derived defaults. This report cannot verify numeric agreement without flying the model; flagged as the single highest-value flight-model check for the lead |
| XP-004 | `docs/systems-coverage.md`: "fuel pump pushbuttons `CIRCUIT CONNECTION ON:n` … MISSING: fuel.rs powers circuits from buses only" | fuel.rs power source only | Real aircraft: fuel pump pushbuttons toggle the bus-to-circuit connection, which should gate pump power independently of bus availability | fuel.rs owner (per team.md) to honour `CIRCUIT CONNECTION ON:n` |
| XP-005 | `docs/systems-coverage.md`: "MSFS light circuits do not [follow FBW buses]" | X-Plane's own lighting system | Real aircraft: cabin/cockpit lights are powered from specific electrical buses and should go dark on bus loss | Route lighting through the same circuit-power lookups fuel pumps/valves already use |
| XP-006 | `docs/systems-coverage.md`: "handleSimulationRate (limit sim rate while AP engaged) | MISSING" | X-Plane's sim-rate control, unconstrained | FBW's FlyByWireInterface limits sim rate increases while the autopilot is engaged, to avoid AP divergence at high sim rates | Low effort, low realism impact; port the check into fbw_interface_extras.rs (owner: FlyByWireInterface leftovers per team.md) |
| XP-007 | `docs/systems-coverage.md`: LightSync row — "Reads `GLASSCOCKPIT AUTOMATIC BRIGHTNESS`, `E:TIME OF DAY`, `A:ON ANY RUNWAY`, which nothing feeds yet" | Defaults / unfed | Real auto-brightness logic reacts to ambient light and runway occupancy | Feed these three from X-Plane equivalents (sun angle/time, runway proximity) |
| XP-008 | `docs/systems-coverage.md`: "updatePerformanceMonitoring (`A32NX_PERFORMANCE_WARNING_ACTIVE`) | MISSING (low impact)" | Never set | FBW's FlyByWireInterface tracks simulator performance (frame time) and warns | Low priority; port if frame-time telemetry is easy to source from X-Plane |

## 6. The plugin's JS runtime (src/js/msfs, src/js/dom)

`warnOnce`-gated "not supported" paths found by grep in
`src/js/dom/*.js` and `src/js/msfs/*.js` — each fires at most once per
session and is a genuine, cited approximation:

| ID | Evidence (fbw-xp-systems/src/js/) | MSFS/DOM feature | Effect on instruments |
|---|---|---|---|
| JS-001 | dom/canvas.js:445,781 | Canvas fill/stroke patterns | Ignored; any texture-fill (e.g. hatching, dashed regions) on a `<canvas>`-based display renders as solid/blank instead |
| JS-002 | dom/canvas.js:638,642 | `isPointInPath`/`isPointInStroke` | Always return `false` — any canvas hit-testing (e.g. touch/click detection drawn via canvas rather than DOM elements) never matches |
| JS-003 | dom/canvas.js:749 | `drawImage()` with a canvas/video source | Only `<img>` sources supported; canvas-to-canvas or video-frame compositing (potentially used by weather radar or terrain rendering) is unsupported |
| JS-004 | dom/canvas.js:785,788 | `getImageData`/`putImageData` | `getImageData` throws ("this canvas draws to a vector stream"), `putImageData` is a no-op; any pixel-level read/write breaks |
| JS-005 | dom/core.js:1454,1458,1471 | SVG `getCTM`/`getScreenCTM`/`getTotalLength` | Unsupported; path-length-based animations (e.g. a needle or trend vector following an SVG path fraction) can't compute their position correctly |
| JS-006 | dom/css.js:1074 | CSS gradients | Entirely unsupported/ignored; any gradient background in FBW's CSS renders as a flat fill |
| JS-007 | dom/css.js:948,957,963 | CSS grid `repeat()`, named lines, explicit tracks | Falls back to `auto`; grid-based layouts may not size/position cells exactly as designed |
| JS-008 | dom/css.js:1410 | `animation-play-state: paused` | Ignored — a CSS animation meant to freeze (e.g. a paused indicator) keeps running |
| JS-009 | msfs/window.js:160 | `location.reload()` | Warns and no-ops; any instrument code path that reloads its own view (recovery from an error state) won't work |
| JS-010 | (directory listing, §2.3) | React 17 DOM rendering fidelity for 10 of 14 SD legacy pages | Not itself a missing DOM feature, but the highest-leverage thing to verify: since SDv2 only covers 4 of the SD's pages, ENG/BLEED/ELEC AC/ELEC DC/HYD/COND/DOOR/WHEEL/APU/PRESS/CB and CB pages all depend on `src/js/dom`'s React 17 support being complete. `docs/dom-survey.md` should be checked against `docs/dom-usage-recorded.md` (the Chromium reference harness) specifically for these pages |

`src/js/msfs/coherent.js`'s `once()` warns for any `Coherent.call`/`.on`
name the plugin doesn't answer (coherent.js:18-23); this report did not find
a complete enumeration of which call names the instruments actually invoke
vs. which `src/js/msfs/mod.rs` and `js_bridge.rs` answer — a runtime trace
(`boots_fbw_cockpit_views --nocapture`, which team.md says already runs with
"0 script errors") is a better source of truth for that than static grep,
since providers are registered across several teammates' files.

## 7. Top 50 candidates (highest impact ÷ effort, S=1/M=2/L=3/XL=4)

1. INST-005 — V-speeds-too-low check hard-disabled (4/1)
2. ECAM-005 — speedbrake INOP source is FCDC command, not lever (2/1)
3. ECAM-007 — antiskid fault ignores power/fault signal (2/1)
4. ECAM-009 — max-reverse callout ignores reverser INOP (2/1)
5. ECAM-013 — FWS ground speed not from CDS (2/1)
6. CPU-005 — FMS LVars read as 0 before JS FMS starts writing them (2/1)
7. CPU-010 — FCDC bus words + spoiler LVars still WIP (4/3)
8. INST-004 — VNAV decel/speed-constraint placeholders (4/3)
9. ECAM-003 — door/pack-overheat-on-ground faults missing (3/2)
10. ECAM-010 — GEN/IDG/bleed/reverser INOP from engine-out only (3/2)
11. CPU-003 — pitch trim switches hard-coded false (3/2)
12. CPU-011 — FailuresConsumer never reaches the C++ computers (3/2)
13. XP-001 — reverser delta-speed/yaw MISSING (3/2)
14. XP-004 — fuel pump circuit-connection pushbuttons ignored (3/2)
15. JS-010 — 10/14 SD pages depend on unverified React17 DOM fidelity (3/2)
16. ECAM-001 — "more restrictive" speed limitation not implemented (2/2)
17. ECAM-002 — derated climb/soft-GA/MFP heating hard-coded false (2/2)
18. ECAM-008 — SURV SYS INOP stubbed false (2/2)
19. ECAM-012 — elec galley/pax-sys "off" reuses pushbutton simvar (2/2)
20. ECAM-017 — some FWS aurals (e.g. CIDS chimes) never wired (2/2)
21. INST-008 — LS indicator RNAV/MMR-word checks missing (2/2)
22. INST-009 — spoiler indication single-source; LGCIS not modeled (2/2)
23. CPU-001 — CalculatedRadioReceiver not ported (2/2)
24. CPU-004 — rudder trim switches hard-coded false (2/2)
25. CPU-006 — FCU value-set/EFIS panel events unavailable (2/2)
26. XP-005 — MSFS light circuits don't follow FBW buses (2/2)
27. JS-003 — drawImage() only supports `<img>` sources (2/2)
28. XP-003 — flight_model.cfg aero sections not converted; verify XP blade-element tuning (4/4)
29. INST-001 — secondary flight plans entirely unimplemented (4/4)
30. ECAM-006 — manual pressurisation/sensor-failure logic missing (3/3)
31. ECAM-014 — ECU discrete words missing; TLA-only engine state (3/3)
32. ECAM-015 — attitude-source/SFCC-switching/flap-slat/rudder-fault gaps (3/3)
33. ECAM-018 — dual-FWS-failed ECP backup path incomplete (3/3)
34. INST-003 — ATSU/CPDLC never hooked up for A380X (3/3)
35. INST-006 — ND VD descent profile never drawn (3/3)
36. ECAM-016 — single TCAS fault channel; STATUS ordering colour-only (2/3)
37. INST-012 — KCCU not modeled at all (2/3)
38. JS-004 — getImageData/putImageData unsupported (2/3)
39. JS-005 — SVG getCTM/getScreenCTM/getTotalLength unsupported (2/3)
40. INST-002 — no FMC-A/B/C sync/voting (3/4)
41. XP-002 — no gas-generator core physics (inherited from FBW) (2/4)
42. ECAM-004 — IFEC overhead PB check stubbed true (1/1)
43. ECAM-011 — (see ECAM-012) (1/1)
44. INST-007 — OANS airport auto-select uses raw GPS only (1/1)
45. INST-010 — pitch-trim display no-CG-available case unhandled (1/1)
46. INST-013 — RMP audio "only two TX" cap is a TODO stub (1/1)
47. CPU-002 — localizer distance without DME hard-coded 0 (1/1)
48. XP-006 — sim rate not limited with AP engaged (1/1)
49. XP-007 — LightSync auto-brightness inputs unfed (1/1)
50. XP-008 — A32NX_PERFORMANCE_WARNING_ACTIVE never set (1/1)

(Remaining 10 of 60 — CPU-007, CPU-008, CPU-009, JS-001, JS-002, JS-006,
JS-007, JS-008, JS-009, INST-011 — are lowest priority: either inherited FBW
hard-codes with no port-side fix, or cosmetic DOM/CSS approximations.)
