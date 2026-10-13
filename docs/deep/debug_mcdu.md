# Debug: MCDU/KCCU cockpit keys unreliable (presses not registering, wrong key, double presses, only some keys working)

Reported bug: pressing MCDU-style keys in the 3D cockpit is unreliable —
sometimes a press does nothing, sometimes the wrong thing happens, sometimes
it happens twice, and sometimes only some of the keys on a panel work at all.

## What "the MCDU" actually is on this aircraft

The A380X has no A320-style MCDU. Its equivalent is the **KCCU**
(Keyboard/Cursor Control Unit), one per side (L = captain, R = first
officer), a physical keypad next to each MFD. Confirmed by grepping
`D:\fbw-aircraft\fbw-a380x\src\systems\instruments\src` for `MCDU`/`CDU`: no
`A320_Neo_CDU`/`CDU_1_BTN` listener exists anywhere in the A380X's own
TypeScript. The KCCU's keys feed `MFD.tsx` (`hEvent` subscriber,
`MFD.tsx:231-283`) and text-entry widgets (`InputField.tsx`,
`DropdownMenu.tsx`) through an internal `kccuKeyEvent` bus topic.

`hevents.txt` also carries ~140 legacy `A320_Neo_CDU_1_BTN_*`/`_2_BTN_*`
names inherited from the A32NX's own generic ModelBehaviorDefs library.
Confirmed (`grep -r "A320_Neo_CDU\|CDU_1_BTN\|CDU_2_BTN" D:\fbw-aircraft\fbw-a380x`
and `fbw-common`: **zero matches**) that nothing in the A380X reacts to any
of them — this is FlyByWire's own limitation (the A380 simply never wired
those events to anything, same as it never built an MCDU screen), not
something this plugin broke or can fix. `hevents.txt:558-561` documents this
explicitly for the `_CLR`/`_CLR_Held` pair ("not found in the bundles' own
listeners"); the rest of the `CDU_*` family is the same story, just
undocumented. Not a bug — no action needed.

## The real input path, traced end to end

1. **Cockpit model** (`D:\fbw-aircraft\fbw-a380x\src\base\...\Part_Interior_Cockpit\model\behaviour\kccu.xml`,
   read-only reference). Every KCCU key (0-9, A-Z, DOT, PLUSMINUS, ESC, UP/
   DOWN/LEFT/RIGHT, DIR, PERF, INIT, NAVAID, MAILBOX, FPLN, DEST, SECINDEX,
   SURV, ATCCOM, ND, SLASH, ESC2, KBD, REWIND, FORWARD, ENT, BACKSPACE, SP,
   CLRINFO — 63 keys, template `FBW_A380_KCCU_Generic_Button`, `kccu.xml:5-26`)
   fires exactly one thing on left-click:
   `<LEFT_SINGLE_CODE>(>H:#PLANE_NAME#_KCCU_#SIDE#_#KEY#)</LEFT_SINGLE_CODE>`
   (`kccu.xml:24`, `#PLANE_NAME#` = `A32NX`, `kccu.xml:13`) — a single H:
   event, once per click, never a hold/repeat.

   **`KBD` and `CCD` are switches, not these buttons** (`kccu.xml:28-56`,
   template `FBW_A380_KCCU_Generic_Switch` → `FBW_Anim_Interactions` with
   `ANIM_TYPE=SWITCH`, `SWITCH_POSITION_VAR=A32NX_KCCU_#SIDE#_#COMPONENT#_ON_OFF`).
   They write `L:A32NX_KCCU_{L,R}_{KBD,CCD}_ON_OFF` directly, through the
   model's own switch mechanism — **no H: event, no K: event, ever**. This
   local var alone is what `MFD.tsx:223-229` and `OansControlPanel.tsx:450-458`
   read to switch `interactionMode` between `Kccu` and `Touchscreen`, which
   gates whether `InputField`/`DropdownMenu` accept KCCU key input at all
   (`InputField.tsx:57`, `DropdownMenu.tsx:36`: "Only handles KCCU input for
   respective side"). See "Highest-risk node" below — this is the one place
   in the whole path where a misclassification would silently disable *every*
   character key on one side while the named navigation keys (DIR/PERF/...)
   kept working, which is exactly the "only some keys working" report.

2. **Cockpit → X-Plane command** (`D:\A380\msfs2xp-aircraft\src\behaviour\bind.rs`,
   `events.rs`; read-only, out of this task's editable files). The
   `LEFT_SINGLE_CODE`'s `(>H:NAME)` RPN resolves to `Click::Command` for the
   `ASOBO_GT_Push_Button_Airliner` template (`bind.rs:333-339`, one-shot per
   click, i.e. `ATTR_manip_command` in the converted OBJ — not
   `ATTR_manip_command_axis`/a repeat manipulator, so no built-in X-Plane
   repeat-while-held behaviour to cause a double fire). Every KCCU H: name
   falls through `events.rs`'s `h_event()` to the generic case
   (`events.rs:332-335`): `Cmd::Fixed(format!("fbw/hevent/{other}"))`. The
   `KBD`/`CCD` *switches* resolve through the generic ASOBO switch path
   (`bind.rs:319,330`, `own_component`/`pos_var`, tested for exactly this
   template shape in `expand.rs`'s and `bind.rs`'s own test suites) to a
   `Click::Toggle` on a dataref mirroring `L:A32NX_KCCU_{side}_{KBD,CCD}_ON_OFF`
   — not a command, matching the model exactly. I found no defect in this
   converter code; flagged only because it is architecturally the most
   fragile point (see below).

3. **X-Plane command → plugin queue** (`src/js_bridge.rs`). `JsHost::find`
   creates one `fbw/hevent/<name>` command per line of
   `tools/js-build/hevents.txt` and registers `on_hevent` on it, keyed by
   that line's index (`js_bridge.rs`, `find`, originally lines ~802-810).
   `on_hevent` queues the index on the command's **begin** phase only
   (`phase == 0`), so one push = one queue entry regardless of how long the
   button is held — confirmed correct, no repeat-while-held bug.
   `take_hevents()` drains the thread-local queue once a tick and resolves
   indices back to names.

4. **Plugin → both renderers** (`src/lib.rs`, `Plugin::tick`, originally
   lines 1568-1594). `h_events` is taken once, then handed to **both**
   `js.update(..)` (the plugin's own in-process QuickJS cockpit) **and**
   `host.post_tick(..)` (XPHFBW's shared-memory bridge), by design (comment
   at `lib.rs:1563-1567`: "so XPHFBW's views see the same events the plugin's
   own QuickJS cockpit does without either draining the other's share of the
   queue"). Rule 7 ("never both engines drawing one screen") is enforced by
   suspending one of them — but see **Bug found** below: the suspend
   decision is computed *after* both consumers have already acted on this
   tick's events.

5. **Worker → instruments** (`src/js_worker.rs:355-356`,
   `src/js/msfs/mod.rs:366-371`). The worker hands each frame's `h_events` to
   `Cockpit::h_event`, which posts `OnInteractionEvent` to **every** view
   (`To::All`) — matching MSFS's real Coherent event exactly, and correctly
   *not* filtering by side here (each MFD instance self-filters).

6. **XPHFBW's bridge → its own browser views** (`src/xphfbw_host.rs:196-204`,
   `src/xphfbw_bridge.rs`'s `Downlink::HEvent`). `post_tick` broadcasts every
   name from step 4 to every view's downlink `Ring` unconditionally (not
   gated on `displays_active`) — harmless while nobody is reading it, but see
   **Bug found**.

7. **Instrument code** (`D:\fbw-aircraft\fbw-a380x\src\systems\instruments\src\MFD\MFD.tsx:231-283`,
   read-only reference). Subscribes to the `hEvent` bus topic, filters by
   `A32NX_KCCU_L`/`A32NX_KCCU_R` matching `captOrFo`, publishes
   `kccuKeyEvent` for every matching key (consumed by `InputField.tsx`/
   `DropdownMenu.tsx`, gated on `interactionMode` from step 1's switches),
   and additionally navigates the MFD for the named keys (DIR, PERF, INIT,
   NAVAID, MAILBOX, FPLN, DEST, SECINDEX, SURV, ATCCOM, CLRINFO). This part
   is FlyByWire's own code and is correct as written.

## Bug found: one-tick double delivery of H: events at the `displays_active` transition

**File: `src/lib.rs`, `Plugin::tick` (not in this debug task's editable file
list — the fix is given here for whoever owns that file).**

```rust
if let Some(js) = self.js.as_mut() {
    js.update(&mut self.vars, delta, self.time, &h_events, &provider_events);   // (A)
    events.extend(js.take_events());
}
if let Some(host) = self.xphfbw.as_mut() {
    host.post_tick(&mut self.vars, self.time, &h_events, &provider_events);      // (B)
    events.extend(host.take_events());
    ...
    let active = host.displays_active();                                        // (C) computed from (B)'s work
    if let Some(js) = self.js.as_mut() {
        if active && js.running() { js.suspend(); }                             // (D) too late for this tick
        else if !active && !js.running() { js.resume(); }
    }
}
```

`(A)` hands this tick's `h_events` to the plugin's own QuickJS `Cockpit`
*before* `(B)`/`(C)`/`(D)` have run. `host.displays_active()` can flip on the
very same tick a view's `Uplink::Loaded` ack is processed
(`xphfbw_host.rs`'s `apply_deferred` → `update_displays_active`, both called
inside `post_tick`, i.e. inside step `(B)`, i.e. *after* `(A)` already ran).
Concretely, on the one tick `displays_active` goes `false -> true` (every
XPHFBW view has just finished loading):

- `js.running()` is still `true` at `(A)` — `js.suspend()` at `(D)` has not
  run yet — so the plugin's own QuickJS `Cockpit` processes this tick's
  `h_events` through `Cockpit::h_event` (step 5 above).
- `(B)` (which ran *after* `(A)` in this same tick, but *contains* the work
  that decides `active`) has already broadcast the identical `h_events` to
  every XPHFBW view's downlink (step 6), which the just-finished-loading
  browser views now consume for real (step 7).

The same physical KCCU/CDU keystroke is applied by two independent
instances of FlyByWire's instrument code on that one tick: a duplicate
keystroke ("double presses"). The reverse transition (`true -> false`, e.g.
one MFD view's script throws and reloads under load, sending
`Uplink::Loaded { ok: false }` — `xphfbw_host.rs:311-320`, not only on a full
process exit) calls `js.resume()`, which restarts a *fresh* QuickJS worker
with reset slots (`js_bridge.rs`'s `resume`) while XPHFBW's browser may still
be alive and report `Loaded { ok: true }` again a tick or two later, flipping
back through the same window again. Because a reload can be triggered by any
uncaught script exception (not only a process crash), this is not a one-off
startup artifact — it can recur mid-flight, which matches the intermittent,
seemingly-random nature of the report ("sometimes it double-presses,
sometimes a key seems to do nothing") better than a pure race that only ever
fires once per session.

**Exact fix for `src/lib.rs`** (settle `active`/suspend-or-resume from the
*previous* tick's state before feeding `js.update` this tick's events, by
moving the `xphfbw` block above the `js` block and reading `displays_active`
from before this tick's `post_tick` ran):

```rust
let was_active = self.xphfbw.as_ref().is_some_and(|h| h.displays_active());
if let Some(host) = self.xphfbw.as_mut() {
    host.post_tick(&mut self.vars, self.time, &h_events, &provider_events);
    events.extend(host.take_events());
    crate::perf::lap("tick-after-systems: xphfbw_datarefs");
    xphfbw_datarefs::set_views_loaded(host.views_loaded());
    let active = host.displays_active();
    if let Some(js) = self.js.as_mut() {
        if active && js.running() {
            js.suspend();
        } else if !active && !js.running() {
            js.resume();
        }
    }
}
if let Some(js) = self.js.as_mut() {
    // Skip this tick's h_events for the engine that is not authoritative
    // for it: `was_active` is this tick's state as of *before* post_tick
    // could have just flipped it, so a transition tick hands h_events to
    // neither engine twice.
    if !was_active {
        js.update(&mut self.vars, delta, self.time, &h_events, &provider_events);
    } else {
        js.update(&mut self.vars, delta, self.time, &[], &provider_events);
    }
    events.extend(js.take_events());
}
```

(Any equivalent reordering that settles `displays_active` for this tick
before deciding what `h_events` slice `js.update` receives fixes it; the
above is one way that keeps every other call in place.) This plugin's own
`docs/deep/BRIEF.md` scopes this debug task to `src/key_events.rs`,
`src/xphfbw_bridge.rs`, `src/xphfbw_bridge_views.rs`,
`src/xphfbw_datarefs.rs`, `src/js_bridge.rs` and `src/js/`, so `src/lib.rs`
is not edited here.

### Defensive fix applied here (in scope): duplicate command registration

While `src/lib.rs` cannot be touched from this task, `src/js_bridge.rs`
*can* be, and the same "one click, two deliveries" failure mode has a second
possible cause entirely inside it: if `tools/js-build/hevents.txt` ever
listed the same event name twice (a plausible mistake — it is a 573-line
hand-maintained text file, not generated or validated at build time),
`JsHost::find`'s registration loop would call
`xplm.register_command_handler(command, on_hevent, i as *mut c_void)` twice
on the *same* `CommandRef` (`XPLMCreateCommand` returns the same ref for the
same name) under two different `i`s that both resolve back to the identical
name in `take_hevents`. X-Plane calls every registered handler on a command,
so one physical click would then queue that one event name **twice** —
indistinguishable from the manipulator itself double-firing.

I verified `hevents.txt` currently has no duplicates (`awk` + `sort | uniq
-d` found none), so this is not live today, but nothing previously enforced
that invariant. Fixed in `src/js_bridge.rs`'s `JsHost::find`: the
registration loop now tracks names already registered and skips (logging
once) any repeat, so a future duplicate line degrades to "logged, first
registration kept" instead of silently doubling every press of that key.
Covered by two new tests in `src/js_bridge.rs`'s `mod tests`:

- `hevent_names_have_no_duplicates_that_would_double_register_a_command` —
  fails the build the moment `hevents.txt` ever gains a duplicate name,
  before it can reach the registration loop at all.
- `every_kccu_key_from_the_real_cockpits_behaviour_has_a_command` — the
  mirror-image regression guard for "only some keys working": hardcodes the
  63-key list from `kccu.xml`'s `FBW_A380X_KCCU_Template` and asserts
  `hevents.txt` still has `A32NX_KCCU_{L,R}_<key>` for every one of them, so
  a future edit that drops or renames one is caught immediately instead of
  silently making that one physical key inert.

Both tests currently pass against `tools/js-build/hevents.txt` as it stands
(confirmed all 63×2 KCCU names present, byte-for-byte, no duplicates
anywhere in the file's ~572 event names).

## Highest-risk node for a future regression: KBD/CCD misclassification

The single place in this whole path where a conversion mistake would produce
exactly "only some keys working" is step 1/2 above: if a future
re-conversion of the KCCU ever classified the `PUSH_KCCU{L,R}_KBD_ON_OFF` /
`_CCD_ON_OFF` **switch** nodes the same way as the 63 **button** nodes
(`Click::Command` instead of `Click::Toggle`), clicking them would fire
`fbw/hevent/A32NX_KCCU_{L,R}_KBD_ON_OFF` — a command nothing in
`hevents.txt` lists and nothing in FlyByWire's bundle listens for (confirmed:
no `KBD_ON_OFF`/`CCD_ON_OFF` string anywhere in `fbw-a380x`) — instead of
writing `L:A32NX_KCCU_{L,R}_KBD_ON_OFF`. `interactionMode` would then never
leave `Touchscreen`, and `InputField`/`DropdownMenu` would silently ignore
every character key on that side while the named navigation keys (which
reach `MFD.tsx`'s `switch` regardless of `interactionMode`) kept working —
precisely "only some keys working". I found no evidence this has happened
(the generic ASOBO-switch path this depends on is exercised by existing
tests in `msfs2xp-aircraft`'s `expand.rs`/`bind.rs` for this exact template
shape), but it is the one dependency this debug task could not verify
in-sim (no builds/cargo per `BRIEF.md`), so it is the first thing to check
physically if "only some keys" recurs: click the physical `KBD` switch (not
the `KBD` push-key) on each side and confirm the MFD's cursor changes from
touchscreen to KCCU mode.

## Audit: other cockpit input paths for the same bug classes

Two distinct bug classes were checked for across the rest of the cockpit:
**(a)** a control converted as an X-Plane *command* (H:/K: event) when the
real aircraft drives it as a *switch* writing a dataref directly (or vice
versa) — the KBD/CCD failure mode above — and **(b)** duplicate/overlapping
command registration causing a double delivery — the `hevents.txt` failure
mode above.

- **FCU** (`src/key_events.rs`'s `afs_event`, `D:\A380\msfs2xp-aircraft\src\behaviour\events.rs`'s
  `h_event`). Cross-checked every `A32NX.FCU_*`/`AUTO_THROTTLE_*` name
  `events.rs::h_event` can produce (17 names: `AP_1_PUSH`, `AP_2_PUSH`,
  `LOC_PUSH`, `APPR_PUSH`, `ALT_PUSH`, `ALT_PULL`, `SPD_INC`, `SPD_DEC`,
  `SPD_PUSH`, `SPD_PULL`, `HDG_INC`, `HDG_DEC`, `HDG_PUSH`, `HDG_PULL`,
  `VS_INC`, `VS_DEC`, `VS_PULL`) against `key_events.rs`'s `afs_event` match
  arms (`key_events.rs:140-190`): all 17 present. The FCU's knobs are
  intercepted by `events.rs::h_event` (lines 313-331) *before* they could
  ever fall through to the generic `fbw/hevent/<name>` path, so they carry
  none of class (b)'s registration-duplication risk (they never register a
  per-name X-Plane command at all; they are K: events resolved once through
  `afs_events::send`). No bug found. Already correctly documented in
  `key_events.rs`'s own module doc comment (lines 1-60), which is itself a
  citation-backed audit of every K: event this plugin applies.
- **EFIS control panel**: already identified and documented (not by this
  task) in `key_events.rs:55-60` as the same class (a) pattern as KBD/CCD —
  `A32NX_FCU_EFIS_{L,R}_*` are written directly as `L:` vars by the panel's
  own RPN, never sent as `K:`/`H:` events, and `key_events.rs` correctly
  does nothing with them. Consistent with the KCCU KBD/CCD finding above:
  this is a recurring, intentional pattern across the cockpit (some controls
  are events, some are direct-var switches/knobs), not specific to one
  panel, so any future audit of a new panel should check its actual
  behaviour-XML resolution (switch/toggle/axis vs push/command) rather than
  assume from how the control looks physically.
- **Overhead pushbuttons**: sampled via `msfs2xp-aircraft`'s
  `tests_cockpit.rs:479` (`A32NX_OVHD_ELEC_IDG_1_PB_IS_RELEASED`) — same
  direct-`L:`-var pattern as EFIS/KBD/CCD, not a command, so class (b) does
  not apply; `ELECTRICAL_CIRCUIT_TOGGLE`, `ELECTRICAL_BUS_TO_CIRCUIT_
  CONNECTION_TOGGLE`, `APU_GENERATOR_SWITCH_TOGGLE` and the fuel-valve/
  ignition K: events in `key_events.rs`'s `apply()` (lines 222-280) are all
  K: events (script-originated, not per-name X-Plane commands), so they also
  carry none of class (b)'s risk. No bug found.
- **KCCU**: covered fully above — class (a) risk identified (KBD/CCD,
  currently believed correctly handled but unverified in-sim), class (b)
  risk closed by this task's fix and tests.

## Files changed

- `src/js_bridge.rs`: `JsHost::find`'s command-registration loop now
  deduplicates by event name (logs and skips a repeat instead of
  double-registering `on_hevent`); two new regression tests in `mod tests`
  (`hevent_names_have_no_duplicates_that_would_double_register_a_command`,
  `every_kccu_key_from_the_real_cockpits_behaviour_has_a_command`).

## Files reviewed, not changed

- `src/key_events.rs`: K: event handling audited (FCU, lights, transponder,
  fuel valves, ignition, seatbelts, spoilers, rudder trim, sim rate, ground
  services) — all correctly scoped, no H:-event/command overlap, no changes
  needed.
- `src/xphfbw_bridge.rs`, `src/xphfbw_bridge_views.rs`: protocol/view-
  numbering review found no framing, encode/decode round-trip, or view-index
  bugs relevant to input (the `Ring`/`SlotTable`/`Input` encode-decode round
  trips are already exercised by this file's own tests).
- `src/xphfbw_datarefs.rs`: unrelated to the input path (publishes plugin
  status datarefs only); reviewed to confirm it does not itself gate or
  delay any KCCU/MCDU event, which it does not.
- `src/js/msfs/mod.rs`, `src/js_worker.rs`: `Cockpit::h_event`/
  `OnInteractionEvent` broadcast and the worker's frame-merge-on-backlog
  logic (`old.h_events.extend(frame.h_events)`) reviewed and found correct —
  events are never lost across skipped frames and never duplicated by the
  worker itself.
- `src/display/mod.rs`, `src/xp.rs`, `src/xphfbw_host.rs`: not in this
  task's editable file list; reviewed read-only. `display/mod.rs`'s
  `dispatch_key`/`mfd_keyboard_callback`/`toggle_mfd_keyboard` are a
  *different* input surface (typing on a real PC keyboard into the popped-out
  MFD window) from the physical KCCU/CDU cockpit clicks this task is about,
  and were not found to share either bug class. `xphfbw_host.rs`'s
  `update_displays_active`/`apply_deferred` are the other half of the
  `src/lib.rs` ordering bug above; no defect found in `xphfbw_host.rs` itself
  beyond participating in that ordering.
