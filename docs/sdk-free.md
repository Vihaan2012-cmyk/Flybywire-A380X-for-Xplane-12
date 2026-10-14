# SDK-free instruments (started 2026-10-13)

Goal: the shipped X-Plane instruments contain no msfs-sdk code, names or patterns. FlyByWire's TypeScript stays untouched in its repo; a build-time transform moves it onto our own library. Behaviour must be identical to the golden build.

## Rules
- Never open msfssdk.js / msfssdk.d.ts, node_modules/@microsoft, or E:/fbw-int/adapt-orig.
- Never render or test with an MSFS build (D:\fbw-aircraft out): it bundles msfs-sdk.
- No explanatory comments. CARGO_TARGET_DIR=D:/A380/fbw-build/target only. Never launch X-Plane.
- Commits as vihaan2012-cmyk noreply, no co-author line.

## Pieces
- Golden (old) build, frozen: D:/A380/fbw-build/golden/2026-10-13 (html_ui, panel.cfg, panel.xml, manifest.sha256). Installed screens as of 8036e6d, xp-avionics based, 0 msfs-sdk.
- Library: engine = xp-avionics code with classes renamed and unexported; public surface = cell/derive/feed/list (values), listen (bus chains), Widget + mounted (components), our JSX builder, renamed utilities. Generic member names kept (get, set, map, sub).
- Transform (esbuild plugin, on/off switch): factory calls -> surface functions; bus chains -> listen(...); DisplayComponent/onAfterRender/FSComponent -> Widget/mounted/our JSX; everything else renamed. Anything unknown is reported.
- Check: differential fuzzing in the QuickJS boot test (worktree E:/fbw-int/wt-fuzz, branch wip/fuzz-harness): seeded inputs, per-frame draw-op hashes + var writes, old vs new, first divergence.
- FMS offline (worktree E:/fbw-int/wt-fms, branch wip/fms-offline): navdata provider in the boot test, canned SimBrief OFP, MFD clicks, route loaded.
- Ship: own fork files through the transform and committed; install.sh refuses bundles with distinctive SDK names; XPHFBW harness confirms every screen renders in CEF.

## Progress
- [x] 0 plan file
- [x] golden frozen
- [x] 1 library: E:/fbw-int/xp-kit (copy of xp-avionics src, renamed by tools/rename-lib.mjs from names.json; tools/make-names.mjs made names.json; component files renamed Widget/Kit/Ref/CssTransform; cell/derive/feed/list in index.ts). tsc 0 errors. dist/index.js 174 KB, 0 SDK names.
- [x] 2 transform: E:/fbw-int/xp-kit/tools/transform.cjs (imports aliased to new names, Subject/MappedSubject/ConsumerSubject/ArraySubject .create -> $cell/$derive/$feed/$list, 29 distinctive members whole-word renamed, pub -> send on access/declaration, JSX pragma FSComponent.h). Hook: fbw-aircraft-ts build-utils.js xpKitPlugin (env XP_KIT=E:/fbw-int/xp-kit/dist) + fbw-a380x/mach.config.js. Bus chains keep their fluent shape (renamed words only).
- [ ] 3 build both, boot test 0 script errors. New build assembled at D:/A380/fbw-build/kit-new (golden + 41 rebuilt files, no maps, same file list as golden, 0 SDK names). Transform now strips comments (minifyWhitespace, legalComments none) and imports from 'xp-kit'. install.sh: deletes html_ui maps, refuses SDK-shaped bundles.
- [x] 3 boot test: all views load on both builds.
- [x] 4 fuzz harness (src/js/msfs/fuzz.rs): seeds 1 (60 s), 2, 3, 4 (120 s) identical in every frame of 20 screens and every var write; only change is the library's log label "Hub" for "EventBus". Loop: E:/fbw-int/xp-kit/tools/cycle.sh <seed> <seconds>.
- [x] 4b fixes: pub -> send only on calls (a field named pub broke the RMP); GameStateProvider/Instrument renamed to GameStateWatch/GaugeCore (GameState and Gauge are sim globals; the class shadowed the enum and delayed the FMS).
- [x] 5 FMS offline (src/js/msfs/fms_offline.rs, test fms_offline_flight_plan): EGLL-KJFK from a canned OFP, 13 idents; golden and new identical.
- [x] 6 installed 2026-10-13 22:3x (guard passed, 0 maps, 0 SDK names, all 137 shipped files byte-identical to the tested build); XPHFBW harness: 21/21 displays load and draw in CEF. Test defaults point at the installed aircraft, never the MSFS build. A380 repo commit 4050042 (xp-kit replaces xp-avionics, hook, notices) not pushed: push with the release.
- [x] Our own fork files (15 OIT files + 4 rewritten ARINC bus files) written against xp-kit in source: fork e3554e0, A380 repo d69eda6; build output byte-identical, fuzz seed 1 identical.
- [x] Offline ND draws the route (63d9f6c): the test hosts held E:SIMULATION TIME still and the ND canvas map redraws on simTime. Both test hosts now advance E:SIMULATION TIME/E:ABSOLUTE TIME; fuzz seed 1 rerun identical with the clock moving (20539 writes). Golden traces regenerated.

## Notes
- Build (new): cd E:/fbw-int/fbw-aircraft-ts; XP_KIT=E:/fbw-int/xp-kit/dist XP_RELEASE=1 NODE_OPTIONS=--max-old-space-size=8192, XP_AVIONICS unset; node E:/fbw-int/xp-avionics/tools/run-clean.cjs on systems-host/build.js, extras-host/build.js, then mach build --config fbw-a380x/mach.config.js --work-in-config-dir. Output overwrites fbw-a380x/out/.../html_ui (golden is frozen separately).
- Shipping: .map files carry the original TypeScript (SDK names) in sourcesContent: do not ship maps from the new build.
- Stage 6: make the transform emit `from 'xp-kit'` (resolved by the plugin) instead of an absolute path before committing transformed own files.
