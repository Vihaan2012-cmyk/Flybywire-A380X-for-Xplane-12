# XPHFBW: plugin launch, app settings, and FlyByWire's JS in CEF

Read first: docs/briefs/xphfbw-app.md (the app), docs/briefs/debug.md (rules
and storage rules — all apply), src/xphfbw_bridge.rs (THE protocol; do not
change its layouts or record formats — if you truly need a change, stop and
report it instead), src/remote/ (systems process bridge, already working).

Build commands
- Plugin: `CARGO_TARGET_DIR=/d/fbw-build/target-main cargo +stable-x86_64-pc-windows-gnu build --release --features js` (tests: `... test --release --features js --lib <filter>`)
- App: `cd D:/fbw-xp-systems/app && CEF_PATH=D:/fbw-build/cef CARGO_TARGET_DIR=/d/fbw-build/target-app cargo +stable-x86_64-pc-windows-msvc build --release`
- Staged app for manual runs: D:\fbw-build\xphfbw-stage (XPHFBW.exe + CEF runtime + ui/).
- X-Plane: do not launch/close it; do not run tools/install.sh except agent A who edits it (A may run it only if X-Plane is not running: `tasklist | grep -i x-plane`).
- No sub-agents. Shared target dirs; builds queue on cargo's lock — expected.

## Architecture recap

Plugin (win.xpl, GNU toolchain) creates every shared object for a session tag
and starts `XPHFBW.exe <tag> <X-Plane pid> --xp-root=<X-Plane dir> --aircraft=<aircraft dir>`.
XPHFBW (MSVC, CEF 152) runs FBW's Rust systems (remote::server, lockstep, done),
and runs every FBW JS view (panel.cfg `[VCockpitNN]` htmlgauge00, including the
screenless SystemsHost/ExtrasHost) in an off-screen CEF browser (CPU rendering,
`disable-gpu`). The plugin's QuickJS engine stays as the fallback when XPHFBW
is absent or its displays are off.

## Protocol semantics (src/xphfbw_bridge.rs)

Session objects, all created by the plugin with the same tag as the systems block:
- `SlotTable` (`slots`): variables. Views register `(name, unit)` (cross-process
  mutex). The plugin resolves new slots each frame with the same logic the
  QuickJS host uses (js_bridge.rs `VarsHost::resolve`/`read`: L:/A:/E:/GAME:,
  unit conversions), sets `resolved`, and writes every slot's value.
- `uplink` Ring (views → plugin): `Uplink` records.
- `downlinks[view]` Ring (plugin → one view): `Downlink` records.
- `input` Ring (plugin → app): `Input` records.
- `ScreenBlock` per cockpit screen id (display::screens SCREENS ids, e.g.
  `SCREEN_DU_PFDL`, sized from panel.cfg's gauge size): app writes pixels.

View numbering: the index of the view's `[VCockpitNN]` section in panel.cfg
order, counting only sections with an active `htmlgauge00` (commented `;`
lines skipped), EFB/OIT/popup views skipped exactly as src/js/msfs/mod.rs
`Cockpit::new` skips them. Both sides must use one shared function: agent C
adds `pub fn view_list(panel_cfg: &str) -> Vec<ViewDef>` to src/xphfbw_bridge_views.rs
(plugin lib, pub) returning {index, section, gauge url, width, height, screen id
(texture name without `$`, empty for screenless)}; the app uses it.

### Race and desync rules (mandatory)

1. **Slot values are published under a sequence lock.** Plugin: `frame` += 1
   (odd = writing), write `time_ms` and all values, `frame` += 1 (even).
   Readers copy the slots they need, then re-check `frame`; if it changed or is
   odd, copy again. A view takes ONE such snapshot per animation frame and
   answers every `getVar` of that JS tick from the snapshot, so a script never
   sees half of one frame and half of the next.
2. **Unresolved slots read 0** (slot >= `resolved`), as the QuickJS worker does
   for the first read.
3. **Read-your-writes.** A view's `setVar` updates its snapshot value at once
   and remembers `(slot, value, frame_at_write)`; until the snapshot's `frame`
   is at least `frame_at_write + 4` (two published frames later) the view keeps
   returning its own written value. The plugin applies uplink writes at the
   start of its frame, before the systems tick, then publishes after the tick.
4. **Ordering.** The uplink is FIFO under its mutex; the plugin applies records
   in order, once per frame. Events and writes from one JS call stay in order.
5. **Calls.** Ids are `(view << 48) | counter` per view. A reply goes to that
   view's downlink only. Pending calls simply get their reply in a later frame.
6. **Screens.** App: `writing` = 1, copy dirty rects into pixels, set
   `dirty`/`dirty_count`, `frame` += 1, `writing` = 0. Plugin: read `frame`;
   skip if `writing` = 1; upload dirty rects (or the whole screen if frames were
   missed since the last upload: frame > last + 1); re-read `frame` after — if
   it moved during upload, upload the whole screen next frame.
7. **Displays switch-over.** `displays_active` = 1 is set by the plugin only
   after every screened view reported `Loaded { ok: true }`; until then the
   QuickJS screens stay; if XPHFBW goes away (process exits) the plugin sets it
   0 and restarts its own engine. Never both engines drawing one screen.
8. **Files.** FlyByWire's stored data (datastore JSON, flyPad ini) and
   xphfbw.json are written by several processes: every writer holds the named
   mutex `Local\XPHFBW_settings_files` (NamedMutex) around read-modify-write,
   writes to a temp file and renames.
9. **Session identity.** Every open checks MAGIC/VERSION; the tag is unique per
   plugin start (pid + nanos), so a stale app can never attach to a new session.

### The renderer-side API contract (agent E implements natives, agent F uses them)

Installed on `window.__xphfbw` in every view's main frame before any page script:
- `view: number`, `screen: string`, `aircraftDir: string`
- `snapshot(): void` — take the seqlock snapshot (the runtime calls it at the start of each rAF tick; getVar calls it lazily if none this tick)
- `getVar(name: string, unit: string): number` (sync)
- `setVar(name: string, unit: string, value: number): void`
- `getString(name: string): string`, `setString(name: string, value: string): void`
- `sendEvent(name: string, values: number[]): void` (K:/H: and single-value events)
- `call(name: string, argsJson: string): Promise<string>` (resolves JSON text, rejects with message)
- `poll(): Array<[kind, name, payload]>` — drains this view's downlink (kinds: "h", "provider", "game", "string")
- `gameString(name: string): string` — last value received ("" until it arrives; requests it on first use)
- `storedData(op: "get"|"set"|"delete"|"search", key: string, value: string): string` (sync, mutex rule 8)
- `readFile(path: string): string|null` (sync, html_ui relative or /VFS/ path, from the aircraft dir)
- `log(level: 0|1|2, text: string): void`
- `magVar(lat: number, lon: number): number` (sync; see agent E)
- `loaded(ok: boolean, text: string): void` (reports Uplink::Loaded)

## Custom datarefs and commands (plugin, agent A)

Datarefs (read-only unless noted): `xphfbw/app_running` (int), `xphfbw/displays_active`
(int), `xphfbw/systems_remote` (int), `xphfbw/systems_round_trip_ms` (float),
`xphfbw/systems_late_ticks` (int), `xphfbw/views_loaded` (int).
Commands: `xphfbw/show_app`, `xphfbw/restart_displays`, `xphfbw/restart_app`.
Menu (aircraft menu): "XPHFBW settings" (show_app).

## Agents and file ownership

- **A — Launch & install:** lib.rs `start_systems` + menu; new src/xphfbw_datarefs.rs;
  tools/install.sh (build app with MSVC, copy XPHFBW.exe, CEF runtime
  (*.dll *.pak *.bin *.dat vk_swiftshader_icd.json locales/), app/ui/ into
  aircraft/plugins/fbw_a380_systems/XPHFBW/); app/src/web.rs `status` with real
  numbers (add `pub` stats statics to src/remote/server.rs: ticks, last tick µs,
  variables count; app reads them); fallback chain XPHFBW → fbw_a380_systems_server.exe → in-process.
- **B — App settings in the plugin:** new src/app_settings.rs reading
  xphfbw.json (settings_files::app_settings_path, reload when mtime changes,
  checked every 2 s); wire stateDumps/stateDumpFrames/stateDumpKeep (state_dump.rs),
  randomFailures/failureRate (random_failures.rs), persistence (persistence.rs),
  coldStartGroundPower (efb.rs cold-start block only), systemsOutOfProcess
  (read by A's start_systems via a pub fn you provide); mutex rule 8 in app/src/settings.rs save and the plugin's FlyPadSettings writer.
- **C — Plugin JS host over the bridge:** new src/xphfbw_host.rs and
  src/xphfbw_bridge_views.rs; refactor js_bridge.rs so the QuickJS path and the
  bridge path share resolution/apply code; rules 1–5, 7; H: events from the
  cockpit and provider events to every view's downlink; GAME strings; calls to
  providers (navdata, mapdata, wxr, sound PLAY_INSTRUMENT_SOUND, stored data if
  routed); turn the QuickJS worker off while displays_active. Owns js_bridge.rs, js_worker.rs, the new files.
- **D — Plugin screens & input:** src/display/**: create ScreenBlocks per
  screen, upload per rule 6 into each device's texture (keep the brightness
  dimming regions working on top), route device mouse/wheel to the input ring
  when displays_active, keep the QuickJS drawing path otherwise.
- **E — App renderer natives:** app/src/renderer.rs (CefRenderProcessHandler:
  on_context_created installs `window.__xphfbw` per the contract with V8
  handlers; the renderer opens the Session by tag from the command line switch
  `--xphfbw-tag` that the browser process appends in
  on_before_child_process_launch; view index / screen from the browser's
  extra_info dictionary), hook into window.rs App (`render_process_handler`,
  `on_before_child_process_launch`) — minimal edits there. magVar: implement a
  real World Magnetic Model (WMM2025 coefficients, spherical harmonic degree 12)
  in the renderer, not a stub.
- **F — MSFS runtime shims for a real browser:** app/js/msfs-runtime.js (and
  helpers under app/js/): port src/js/msfs/{environment,simvar,coherent,instrument,window}.js
  to a real Chromium page on top of `window.__xphfbw` — SimVar,
  Coherent.call/on/trigger, RegisterViewListener, BaseInstrument/TemplateElement/
  VCockpit lifecycle and Update loop, GameState, GetStoredData/SetStoredData,
  NXDataStore backing, Facilities.getMagVar, fetch of /VFS/ (served by G's
  scheme), LaunchFlowEvent, Include, etc. Drop everything that exists only for
  our DOM stand-in. Injected before page scripts (E evaluates it in
  on_context_created, or G's scheme handler injects a <script> into the gauge
  HTML — coordinate through the brief: G injects `<script src="xphfbw://runtime/msfs-runtime.js">`
  as the first element of <head> of every gauge HTML).
- **G — App OSR views & schemes:** app/src/views.rs, app/src/scheme.rs: scheme
  handlers `coui://html_ui/...` and `xphfbw://runtime/...` (serves app/js) and
  `/VFS/` equivalents from the aircraft dir; one windowless browser per view
  (size from view_list, windowless_frame_rate from xphfbw.json displayFps, CPU);
  on_paint → ScreenBlock per rule 6; drain `input` ring each UI tick → send
  mouse move/click/wheel to that screen's browser (MFD: one screen); headless
  hosts at 1×1; "restart displays" action (web.rs action wiring: coordinate
  with A — A owns web.rs, G exposes `pub fn restart_displays()`); hook into
  main.rs/window.rs minimally (start views once the session opens).
- **H — Integration tests & review (starts after A–G report):** end-to-end
  offline test harness: plugin-side host + app renderer runtime against a fake
  aircraft page; review races against the rules; fix integration breaks.
