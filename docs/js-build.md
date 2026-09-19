# Building FlyByWire's JS (for SourcePatch text and the patches/ pipeline)

FlyByWire's own TypeScript is never edited in place (team.md). Two mechanisms
change what it does at runtime instead:

- **`SourcePatch`** (`src/js/msfs/mod.rs`, applied by `js_bridge.rs`'s
  `native_ports()` and each area's own `source_patches()`, e.g.
  `src/oans/plugin.rs`, `src/wxr/mod.rs`, `src/ecam_patches.rs`): a find/replace
  against the *built* JS text as a view loads it. Use this for small changes —
  most of the fixes in `src/ecam_patches.rs` are one assignment or a few
  lines.
- **A patch file** under `tools/js-build/patches/`, applied to FlyByWire's
  TypeScript *before* their own build runs, for changes too large for a single
  find/replace (new functions, multi-file changes). None of workstream D's
  fixes needed this; `tools/js-build/patches/` does not exist yet because
  nothing has used it. When something does, add the patch files there and a
  `tools/js-build/apply-patches.sh` that applies them (e.g. via `patch` or
  `git apply`) against `D:\fbw-aircraft` before the build below runs, and
  reverts them after (FBW's sources must stay unmodified between runs).

## Where the build comes from

FlyByWire's own root `package.json` (in `D:\fbw-aircraft`) has the real build
scripts; this plugin's `tools/js-build/` only works around running them on
Windows. The pieces workstream D's patches depend on:

- `npm run build-a380x:systems-host` → `node fbw-a380x/src/systems/systems-host/build.js`
  — SystemsHost.js (FwsCore/FwsAbnormalSensed/FwsAbnormalNonSensed/
  FwsAutoCallouts/FwsSoundManager and the rest of the CpiomC FWS).
- `npm run build-a380x:extras-host` → `node fbw-a380x/src/systems/extras-host/build.js`.
- `npm run build-a380x:instruments` → `mach build --config fbw-a380x/mach.config.js
  --work-in-config-dir` — PFD, ND, MFD, RMP, ISISlegacy, FCU, SD/SDv2. This is
  FBW's own multi-instrument orchestrator (`mach`); it was not run standalone
  while writing this doc; if it needs the same environment workaround as
  `build.js` scripts do (see below), wrap it the same way.

All of the above write into `fbw-a380x/out/flybywire-aircraft-a380-842/html_ui`,
which is:

- what `tools/install.sh` copies into the converted aircraft's `html_ui/`
  (minus EFB/OIT/popup, out of scope per team.md);
- what `js/msfs/tests.rs`'s `boots_fbw_cockpit_views` (`cargo test --release
  --features js -- --ignored boots_fbw`) loads directly.

So a `SourcePatch`'s `find` text must match that tree exactly (it is FBW's
real esbuild output — not minified, mostly readable, comments are sometimes
kept and sometimes stripped depending on where they sit — never assume the
source `.ts` file's text carries over verbatim; grep the built file). A
patch that stops matching (FBW's build changed) is reported by the runtime
("...the patch (...) matches N times, not once; the file runs unchanged")
rather than silently skipped or silently double-applied.

## Windows build environment (`fbw-env.cjs`)

FlyByWire's `build-utils.js` turns every environment variable into an esbuild
`define`. Their CI runs on Linux, where values are plain; on Windows many
values are paths with backslashes, which esbuild rejects as define values, and
the whole build fails. `tools/js-build/fbw-env.cjs` keeps only the variables
esbuild's own process needs (with forward slashes), drops everything else,
and sets the variables FlyByWire's CI sets for the A380X
(`AIRCRAFT_PROJECT_PREFIX=a380x`, `AIRCRAFT_VARIANT=a380-842`,
`VITE_BUILD=false`). Run a `build.js` script through it instead of directly,
from the FlyByWire workspace root (`D:\fbw-aircraft`):

```
node D:/fbw-xp-systems/tools/js-build/fbw-env.cjs fbw-a380x/src/systems/systems-host/build.js
node D:/fbw-xp-systems/tools/js-build/fbw-env.cjs fbw-a380x/src/systems/extras-host/build.js
```

`tools/js-build/hevents.txt` is unrelated (the plugin's own H: event name
list for `fbw/hevent/*` X-Plane commands, read by `js_bridge.rs`, not part of
FlyByWire's build).

## Verifying a patch

1. Grep the exact text in `fbw-a380x/out/flybywire-aircraft-a380-842/html_ui`
   (rebuild first if it's stale — check the file's mtime against when the
   `.ts` source you're citing last changed).
2. Add the `SourcePatch` (or extend an existing `source_patches()`).
3. `cargo test --release --features js -- --ignored boots_fbw` (needs that
   `out/` tree and the MSFS package's `panel.cfg`/`panel.xml`, per the test's
   own doc comment) and check the patch's reason string appears in the log
   ("MSFS runtime: <path>: <reason>") with no "matches N times, not once"
   warning, and that the run's own error/warning count does not go up.
