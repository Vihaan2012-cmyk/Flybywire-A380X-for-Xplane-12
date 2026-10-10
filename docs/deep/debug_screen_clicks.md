# Debug: cockpit screen touches (MFD/OIT/EFB/ND/PFD) do nothing

Reported bug: clicking on the cockpit screens in a real X-Plane 12 session
does nothing — in particular fields/buttons on the A380's MFD, and other
touch-interactive screens (OIT, EFB, ND/PFD softkeys). The KCCU hardware keys
are a separate path, already audited in `docs/deep/debug_mcdu.md`.

## The two engines, and which one draws in a real session

Every cockpit screen is one X-Plane 12 avionics device
(`XPLMCreateAvionicsEx`, `src/display/mod.rs::start`). Two independent
renderers can draw into it (rule 7, `docs/briefs/xphfbw-js-bridge.md`): the
plugin's own in-process QuickJS `Cockpit` (`src/js/msfs/mod.rs` +
`src/js/dom/`), or XPHFBW's own CEF browsers (`app/src/views.rs`), never both
at once — `Displays::bridge_active()` (`src/display/mod.rs:574-576`) picks
one per tick from `displays_active` in the shared `SlotTable` header. In a
real installation with XPHFBW running, the CEF path is what the user
actually sees almost all the time; the QuickJS path is the fallback while
XPHFBW is starting or not installed. Both paths were traced end to end, and
each had a real, distinct defect.

## The path, traced end to end

1. **X-Plane's device and mesh** (`src/display/mod.rs::start`,
   `src/xp.rs:901-929` `avionics()`). `XPLMCreateAvionicsEx` is called once
   per `SCREENS` entry (`src/display/screens.rs`) with `screen_touch`,
   `screen_right_touch`, `screen_scroll`, `screen_cursor` and (MFD only)
   `keyboard` all wired (`mod.rs:882-892`); `device_id`/`device_name` are the
   screen's `id` (e.g. `SCREEN_DU_MFD`), matching exactly what the converted
   OBJ writes. **Checked against both the installed aircraft**
   (`D:\Steam Games\...\FlyByWire A380X\objects\a380_cockpit_000.obj`) **and
   the fresh conversion** (`%USERPROFILE%\Downloads\a380x-xp12\FlyByWire
   A380X\objects\a380_cockpit_000.obj`): every screen mesh already has, in
   order, `ATTR_cockpit_device <id> 0 0 0`, then `ATTR_manip_device hand <id>
   <id>`, then its `TRIS`, then `ATTR_manip_none` (e.g. lines 9708-9711 of the
   installed OBJ for `SCREEN_DU_MFD`). **This is not broken** —
   `docs/screens.md`'s "the converter writes none yet" note about
   `ATTR_manip_device` is stale; the converter (`D:\A380\msfs2xp-aircraft`,
   read-only) was already fixed to emit it, in both builds checked. `.acf`
   (`FlyByWire A380X.acf`) has no panel/avionics setting that would block
   this; the `_obja` attachment list only carries geometry file names and LOD
   bookkeeping, nothing 3-D-cockpit-input-related.
2. **Coordinate mapping** (`src/display/xphfbw.rs::device_to_css`, unit
   tested for both 1x and 2x device scale in the same file). X-Plane's texel
   (origin bottom-left, whatever scale the device is actually drawn at this
   frame — `Displays::draw_screen` recomputes `scale` from the GL viewport
   every frame and feeds it back through `RESCALED`) converts correctly to
   the instrument's CSS pixel space (origin top-left). Verified consistent
   between the two consumers that must agree on it: `Displays::to_screen`
   (QuickJS `ScreenEvent`s) and `Displays::dispatch_pointer` (XPHFBW's
   `Input` records) both call the same function. **No bug found here.**
3. **Delivery — device id and screen key agreement**
   (`src/display/screens.rs`, `src/js/msfs/mod.rs::screen_event`,
   `src/xphfbw_bridge_views.rs::SCREEN_ORDER`). `ScreenDef::id` (no `$`),
   `PanelView::texture` (also stripped of `$`, `mod.rs:108-111`) and
   `SCREEN_ORDER` are all cross-checked by existing tests
   (`display/screens.rs::screens_are_in_the_order_xphfbw_numbers_them`,
   `xphfbw_bridge_views.rs`'s own numbering tests) and agree. **No bug
   found.**
4. **Delivery — `dispatch_pointer`** (`src/display/mod.rs:592-608`). Correctly
   branches on `bridge_active()`: pushes an `Input` record (`InputKind::Down/
   Up/Move/Wheel`) onto the shared `Session::input` ring for XPHFBW, or a
   `ScreenEvent` onto the QuickJS engine's own queue, never both. Checked the
   `Input` wire format round-trip (`xphfbw_bridge.rs::records_round_trip`)
   and the ring's mutex-protected push/drain (`Ring::push`/`drain`). **No bug
   found.**

### Bug 1 (fixed here): the QuickJS path silently drops a click under load

`src/js/msfs/mod.rs::Cockpit::tick` delivers a view's queued `ScreenEvent`s
(among other things) as one `__msfsDeliver(batchJson)` call
(`src/js/msfs/coherent.js:200-211`, which itself dispatches to
`src/js/dom/install.js::__screenEvent` — the DOM's mousedown/mouseup/click
synthesis, hit-testing and hover chain there were read in full and are
correct: focus handling, double-click detection, `pointer-events: none`
respected via `paint.js`'s `receives()`, all present and already exercised
by `src/js/dom/tests/unit.js` and the MFD/ND/EWD/PFD fixtures).

The bug was in `Cockpit::tick` itself (`src/js/msfs/mod.rs`, was around lines
413-418 before this fix):

```rust
if !view.inbox.is_empty() {
    let batch = format!("[{}]", view.inbox.join(","));
    view.inbox.clear();                                   // <- cleared unconditionally
    if let Err(e) = view.engine.invoke(&mut vh, "__msfsDeliver", &[&batch]) {
        vh.log(LogLevel::Error, &format!("{}: {e}", view.panel.name));
    }
}
```

`view.engine.invoke` runs under the engine's own runaway-script watchdog
(`src/js/mod.rs::Engine::budgeted`, default 500 ms per `CockpitOptions::new`,
`src/js/msfs/mod.rs:168`, unchanged by both the test harness and production
(`js_bridge.rs::CockpitRecipe::make`)). If that one call is interrupted — a
view's scripts genuinely running long for a single frame (a busy dropdown
re-layout, say), not necessarily a hang — `inbox` had *already* been cleared,
so whatever was in the batch (most consequentially: a cockpit screen click)
was gone for good. Nothing else redelivers it; the next tick's `route()`
only adds *new* events.

This is not a hypothetical: it is exactly what an existing, more-recent debug
pass already found and documented in the ignored integration test
`src/js/msfs/tests.rs::mfd_dropdown_click_and_overlap` (dated 2026-09-17,
its own comments at `tests.rs:262-321`): "the same click, same coordinates,
opens the dropdown when the following tick runs to completion, and leaves it
closed when that tick gets interrupted... That correlation, not a routing
bug, looks like the real explanation for 'cannot click anything on the MFD'."
That test clicks through `__screenEvent` (the same entry point X-Plane's
touch callback uses) and asserts the dropdown opens only when the settling
tick was not itself watchdog-interrupted — i.e. it already isolated this
exact mechanism, just hadn't traced it back to the inbox-clearing order yet.

**Fix applied** (`src/js/msfs/mod.rs`): the batch is only cleared once
`__msfsDeliver` actually returns `Ok`. On an interrupt specifically (the
error text contains "interrupt", matching `js/mod.rs`'s own
`a_runaway_script_is_interrupted` test), the batch is kept queued and
retried on the next tick(s) — up to `INBOX_RETRY_LIMIT` (3) consecutive
tries, so a genuinely hung script's batch is still dropped eventually rather
than growing forever as later frames' own events keep appending to it. Any
non-interrupt error (a real script bug) still clears the batch immediately,
exactly as before, so a persistently broken handler cannot spin forever
retrying the same doomed call.

New field: `View::inbox_stalls: u32` (consecutive interrupted-retry count for
the current batch, reset on success or on a give-up). New pure function
`should_retry_delivery(error: &str, stalls: u32) -> bool`, pulled out
specifically so the decision is unit-testable without booting an engine (the
same "pure logic, tested standalone" pattern as `display/xphfbw.rs`).

New unit test (`src/js/msfs/tests.rs`):
`should_retry_delivery_only_for_an_interrupted_batch_and_only_up_to_the_limit`
— checks an ordinary script error is never retried, an interrupt is retried
while `stalls <= INBOX_RETRY_LIMIT` (case-insensitive), and is not retried
past the limit. A full end-to-end reproduction would need a real interrupted
`__msfsDeliver` call, which needs FlyByWire's built `html_ui` and the
MSFS/VCockpit gauge-loading machinery (`instrument.js`'s `__vcockpit.load`,
fetching real gauge pages) to construct even a minimal loaded view; that
territory is already covered (unignored manually) by the existing
`mfd_dropdown_click_and_overlap` test, which should now also be re-run to
confirm the fix (needs `D:\fbw-aircraft\...\html_ui` and MSFS's installed
panel files, per that test's own doc comment — not run here per this task's
"no builds, no cargo" rule).

### Bug 2 (found, not fixable in this task's files): XPHFBW's CEF browsers never claim focus

`app/src/views.rs` is not in this debug task's editable file list (only
`src/display/mod.rs`, `src/xp.rs` (avionics device creation only),
`src/xphfbw_host.rs`, `src/xphfbw_bridge.rs`, `src/js/dom/install.js`,
`src/js/msfs/mod.rs` are), so this is documented here for whoever owns it,
as `debug_mcdu.md` did for `src/lib.rs`.

`spawn_view` and `spawn_efb_view` (`app/src/views.rs:148-201`) each create one
off-screen (`WindowInfo::set_as_windowless`) CEF browser per instrument view
via `browser_host_create_browser_sync`, and `send_input_event`
(`views.rs:375-391`) drives clicks into it with `BrowserHost::
send_mouse_click_event`. Neither ever calls `BrowserHost::set_focus(true)`
(`_cef_browser_host_t::SetFocus`, `D:\A380\fbw-build\cef\include\cef_browser.h:398-401`,
bound as `set_focus(&self, focus: c_int)` in the vendored `cef` crate,
version pinned in `Cargo.lock` to `152.3.0+152.0.6`) on any browser, ever.

For an off-screen/windowless CEF browser there is no real OS window to carry
focus, so CEF never infers it; `document.hasFocus()` stays `false` and no
element ever becomes the page's focused element unless the host app calls
`SetFocus`. Plain `onclick`/pointer listeners on buttons generally still fire
without it (as the routing half of `mfd_dropdown_click_and_overlap` already
confirms working when the tick isn't interrupted — see Bug 1), but this
plugin's own bug report specifically says "fields/buttons": an `<input>` or
contenteditable field needs focus to show a caret and accept subsequent
typing, and any UI that keys off `:focus`/`document.activeElement` (menus
that close on blur, form validation styling, `:focus-visible` outlines) will
misbehave or appear inert without it. This is a second, independent way for
"clicking a field does nothing" to be literally true even after Bug 1's fix,
specific to the XPHFBW/CEF rendering path (most real sessions).

**Exact patch** (outside this task's files): in `app/src/views.rs`, right
after each `browser_host_create_browser_sync(...)` call succeeds, focus the
new browser once (each view has its own independent off-screen browser, so
this never steals focus from another screen):

```rust
fn spawn_view(tag: &str, def: &ViewDef, fps: i32) -> Option<Browser> {
    ...
    let browser = browser_host_create_browser_sync(
        Some(&window_info),
        Some(&mut client),
        Some(&CefString::from(url.as_str())),
        Some(&browser_settings),
        Some(&mut extra),
        None,
    )?;
    if let Some(host) = browser.host() {
        host.set_focus(1);
    }
    Some(browser)
}
```

and the same three lines (`if let Some(host) = browser.host() { host.set_focus(1); } Some(browser)`)
in `spawn_efb_view` around its own `browser_host_create_browser_sync(...)`
call (`views.rs:166`). `Browser::host()` is already used exactly this way in
`send_input_event`/`drain_input` (`views.rs:368-370`), so this needs no new
imports.

## Everything else checked and found correct (no changes)

- **`src/xp.rs` avionics device creation** (`avionics()`, `900-929`):
  `XPLMCreateAvionicsEx`/`XPLMDestroyAvionics`/`XPLMIsAvionicsBound`/
  `XPLMSetAvionicsPopupVisible`/`XPLMTakeAvionicsKeyboardFocus`/
  `XPLMHasAvionicsKeyboardFocus` all resolved and typed correctly against the
  XPLM410 headers; every callback's C signature
  (`AvionicsMouse`/`AvionicsWheel`/`AvionicsCursor`/`AvionicsKey`) matches
  `XPLMDisplay.h` exactly, argument order included (`scroll_callback`'s
  `(x, y, wheel, clicks, refcon)` is not swapped). `xp::MOUSE_DOWN`(1)/
  `MOUSE_DRAG`(2)/`MOUSE_UP`(3) match `XPLMDefs.h`'s `xplm_Mouse*` exactly.
- **`CreateAvionics` fields** (`src/display/mod.rs:870-897`): `screen_width`/
  `screen_height` set from the same `ScreenDef` the OBJ's `ATTR_cockpit_device`
  uses; `bezel_width`/`height` equal to the screen size and `bezel_draw:
  None` (no separate bezel art — a supported, documented shape for a
  panel-embedded device); `draw_on_demand: 0` (matches "screens.md"'s stated
  design, replay-cached-batches-every-frame). `keyboard` correctly gated to
  only the MFD device.
- **`touch_callback`/`right_touch_callback`/`scroll_callback`/
  `cursor_callback`** (`src/display/mod.rs:949-988`): always return `1`
  (X-Plane requires this on `MOUSE_DOWN` to keep receiving `MOUSE_DRAG`/
  `MOUSE_UP` for the same gesture); `right_touch_callback`'s KCCU
  pop-up/focus gesture only intercepts the MFD's initial `MOUSE_DOWN` and
  correctly falls through to an ordinary button-2 click for every other
  status/screen.
- **`Displays::mouse`/`cursor`** (`578-608`, `663-686`): status→kind mapping
  correct; hover suppressed for a no-op cursor move; `pressed` tracked so a
  drag reads `InputKind::Move` (not spurious `Down`/`Up`) between the two.
- **XPHFBW bridge wiring** (`src/xphfbw_host.rs`, `src/xphfbw_bridge.rs`):
  `Session::input` is one shared `Ring`, `Input::screen` indexes `SCREENS`/
  `SCREEN_ORDER` order (both sides cross-checked by existing tests); the
  ring's encode/decode round-trips (already tested); `bridge_active()`/
  `displays_active` gating (rule 7) correctly prevents the two engines from
  ever drawing (or receiving input for) the same screen at once, aside from
  the *H: event* double-delivery window already found and documented in
  `debug_mcdu.md` (a different bug class — events, not screen pointer input;
  `Input` records are not affected by that ordering issue since they are
  produced directly from X-Plane's touch callbacks, not from the shared
  `h_events` slice `debug_mcdu.md`'s bug is about).
- **`app/src/views.rs` input delivery** (`355-391`, read-only reference):
  `drain_input`/`send_input_event` correctly map `InputKind` to CEF's
  `send_mouse_click_event`(down/up)/`send_mouse_move_event`/
  `send_mouse_wheel_event`, `screen_browsers` indexed by `SCREEN_ORDER`
  exactly like the plugin's `Input.screen`, and `view_size` returns the
  panel.cfg pixel size unchanged (no extra device-pixel scaling to
  reconcile against the plugin's already-CSS-pixel `Input.x/y`) — confirmed
  by `views.rs`'s own `view_size_is_the_panel_cfg_size_except_headless_
  hosts_are_1x1` test. Only the missing `set_focus` (Bug 2 above) was found
  wrong here.
- **Converter output** (`D:\A380\msfs2xp-aircraft`, read-only): both the
  installed aircraft's and a fresh conversion's OBJs already carry
  `ATTR_cockpit_device`+`ATTR_manip_device` correctly on every screen mesh
  (Bug candidate ruled out — see above); `docs/screens.md`'s note that "the
  converter writes none yet" should be updated by whoever owns that doc,
  since it now reads as an open TODO that is actually done.

## Files changed

- `src/js/msfs/mod.rs`: `Cockpit::tick`'s `__msfsDeliver` batch is no longer
  cleared before delivery succeeds; added `View::inbox_stalls`,
  `INBOX_RETRY_LIMIT`, `should_retry_delivery`.
- `src/js/msfs/tests.rs`: new unit test
  `should_retry_delivery_only_for_an_interrupted_batch_and_only_up_to_the_limit`.

## Files reviewed, not changed

- `src/display/mod.rs`, `src/xp.rs` (avionics device creation), `src/display/xphfbw.rs`,
  `src/display/screens.rs`, `src/xphfbw_host.rs`, `src/xphfbw_bridge.rs`,
  `src/xphfbw_bridge_views.rs`, `src/js/dom/install.js`, `src/js/dom/paint.js`,
  `src/js/msfs/coherent.js` — audited as described above, no defects found.
- `app/src/views.rs`, `app/src/window.rs`, `app/src/main.rs` — read-only
  (not in this task's editable files); Bug 2 documented with an exact patch
  above for whoever owns them.
- Installed aircraft (`D:\Steam Games\...\FlyByWire A380X`) and fresh
  conversion (`%USERPROFILE%\Downloads\a380x-xp12\FlyByWire A380X`) OBJs,
  and the `.acf` — inspected directly (read-only), no defect found; converter
  output is already correct for `ATTR_cockpit_device`/`ATTR_manip_device`.
