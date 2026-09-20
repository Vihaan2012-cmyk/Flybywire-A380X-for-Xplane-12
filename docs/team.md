# Team brief (read first)

The goal is a study-level FlyByWire A380X in X-Plane 12: FBW's own code (Rust systems, C++ computers, TypeScript instruments and hosts), running unmodified wherever possible, with only the MSFS-specific interfaces replaced by X-Plane equivalents.

## Where things are
- **Plugin (Rust cdylib):** D:\fbw-xp-systems. It is not a git repo.
  - `src/lib.rs` has the tick order, the Vars registry and `// [slot ...: name]` comments marking where each engineer inserts lines. Add your own slot lines and never rearrange others'.
  - `src/xp.rs` has the XPLM bindings through GetProcAddress, with no SDK crate. Append only.
  - Headers for exact struct layouts are in D:\fbw-build\xpsdk.
- **Systems modules already done:** fadec, throttle, engine_commands, prim/fbw_controllers (compiled FBW C++), fuel/fuel_network/fuel_transfer, flight_controls, handling (gear, brakes, flaps, steering), sensors (incl. ILS on nav 3), aspects, correctness, failures, start_state, study (the in-sim study panel).
- **JS engine:** `src/js/` (QuickJS via rquickjs 0.13, Oxc TS/JSX transpile, module resolver, prelude) and `src/js_bridge.rs`, behind cargo feature `js`.
- **Display contract:** docs/display-stream.md.
- **FBW source:** D:\fbw-aircraft. Its pnpm workspace may be partly installed. Never modify tracked FBW sources.
- **MSFS package:** D:\Microsoft Flight Simulator 2020\Microsoft Flight Simulator 2020 Packages\Community\flybywire-aircraft-a380-842. The aircraft cfgs are under SimObjects/AirPlanes/FlyByWire_A380_842.
- **X-Plane 12:** D:\Steam Games\steamapps\common\X-Plane 12. The dataref and command references are Resources\plugins\DataRefs.txt and Commands.txt; verify every name against them.
- **Converted aircraft:** X-Plane 12\Aircraft\FlyByWire A380X. The converter that generates it is D:\msfs2xp-aircraft.
  - Only the lead edits the converter. If you need a converter change, describe it exactly in your report.
  - Screen meshes carry `ATTR_cockpit_device <id> 0 0 1`, where the id is panel.cfg's texture name without `$`, in panel.cfg's case: BAT, Clock, FCU, RTPI, SCREEN_DU_EWD, SCREEN_DU_MFD (two meshes, one texture), SCREEN_DU_NDL, SCREEN_DU_NDR, SCREEN_DU_PFDL, SCREEN_DU_PFDR, SCREEN_DU_RMP_1/2/3, SCREEN_DU_SD, SCREEN_ISIS_1, SCREEN_EFB, SCREEN_OIT_LEFT, SCREEN_OIT_RIGHT (display/screens.rs's `SCREENS`).
  - The EFB is out of scope: the user won't use FBW's EFB (the app draws its own study/settings UI on that mesh instead). The OIT is drawn now (docs/oit.md) — the two lateral-console terminals, from FlyByWire's own `A380X/OIT/oit.html`.
  - **Needed converter change (undone; describe, don't implement):** the OIT Side Console switches (`SWITCH_GLARESHIELD_CS_OIT_SIDE`/`_FO_OIT_SIDE` in FBW's glareshield.xml — despite the "glareshield" file name, real position is the lateral console per FBW's own Lateral Consoles doc) need a click manipulator writing the boolean dataref `L:A380X_SWITCH_OIT_SIDE_LEFT` (captain) / `L:A380X_SWITCH_OIT_SIDE_RIGHT` (first officer), the same way any other FBW dummy toggle switch's `SWITCH_POSITION_VAR` already becomes a clickable X-Plane manipulator elsewhere in this aircraft. No plugin-side change is needed once that manipulator exists: `false`/unset already reads as FLT OPS (FlyByWire's own `OIT.tsx`: `domainSwitch ? 'nss-avncs' : 'flt-ops'`), which is the terminal's working mode, and the OIT's own JS already reads that dataref (`OitSimvarPublisher.tsx`). docs/oit.md has the full trace.

## Rules
- Deliver working, tested code. Partial files from earlier engineers may exist: read them, keep what is sound, and finish.
- **Keep the whole tree building** at every stopping point, because everyone builds the same tree. If a build error is in someone else's file and trivially fixable (a warning or a lifetime), fix it minimally; otherwise leave it and mention it.
- **Build and test** with `cargo +stable-x86_64-pc-windows-gnu test --release --features js` and `CARGO_TARGET_DIR=D:\fbw-build\target-<your area>`. Never build on C: (nearly full).
- **No fake values or behaviours.** Everything comes from FBW source, the MSFS package, X-Plane or official specs, cited with file:line where it isn't obvious. If there's no real source, leave it out and say so.
- Don't install into X-Plane, don't launch X-Plane and don't commit.
- Match the surrounding code's style and comment density. No warnings in your files.
- **Report** when finished: files, lib.rs slot lines, what works with test evidence, what's left, and anything only X-Plane can verify.

## Current owners (stage 2.5: fix the top 50, docs/analysis/top50.md)
Stay inside your files. Minimal, additive edits elsewhere (lib.rs slot lines, one-line hooks) must be mentioned in your report. **Never start sub-agents.** Keep reading lean: grep and targeted reads only. Back up with /d/fbw-build/backup-plugin.sh before large edits.

| Workstream | Items | Owns |
|---|---|---|
| A coupling | top50 #1-2 | engine_commands.rs, fadec.rs, throttle.rs; D:\msfs2xp-aircraft\srccf.rs |
| B+E systems glue | #5, 6, 12-15, 31, 34, 35, 47 (the CB model, not the UI) | new src/lights.rs, src/oxygen.rs, src/circuits.rs; fuel.rs, sensors.rs (cloud), aspects.rs |
| C controls | #3, 4, 7, 8, 17, 29, 30, 32, 33, 49 | D:\msfs2xp-aircraft\srcehaviour\ (not acf.rs), key_events.rs, prim.rs input wiring |
| D ECAM/instruments | #9, 16, 18-27, 36-40 | extra_backend_fcdc.rs, fadec discrete outputs (coordinate with A through the report), tools/js-build/patches, SourcePatches |
| F runtime | #10, 41-46 | src/js/dom/, src/js/msfs/ |
| weather radar | brief wxr.md | src/wxr/ |
| Lead | #11, 28, 48, 50, Study UI for everything | src/study/, converter doors |

Every fix must be visible in the Study panel. Name the variables or fields to show in your report, and the lead adds them.
