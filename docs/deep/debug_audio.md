# Debug: cabin "prepare for landing" PA looping nonstop on the ground

Reported bug (real X-Plane 12 session): the cabin/crew "prepare for landing"
announcement played nonstop, looping, apparently on the ground at spawn.

## Sound identified

`sound.xml` (FBW package, `SimObjects/AirPlanes/FlyByWire_A380X/common/sound/sound.xml:1062`):

```xml
<Sound WwiseData="true" WwiseEvent="cabin_crew_seats_landing" LocalVar="A32NX_CABIN_READY" NodeName="PEDALS_LEFT" Continuous="false">
    <Range LowerBound="1" />
    <Requires LocalVar="AIRLINER_FLIGHT_PHASE">
        <Range LowerBound="6" UpperBound="6" />
    </Requires>
</Sound>
```

This is the "cabin crew, seats for landing" PA. It is meant to play once,
only during descent (`AIRLINER_FLIGHT_PHASE == 6`), when `A32NX_CABIN_READY`
becomes 1.

`A32NX_CABIN_READY=1` is also the package's own ground default — set
unconditionally at spawn by
`SimObjects/AirPlanes/FlyByWire_A380X/common/flt/runway.FLT:273`. So on a
runway/ground spawn, the entry's own `LocalVar` condition (`A32NX_CABIN_READY
>= 1`) is satisfied immediately, long before descent. The only thing meant to
keep the PA silent on the ground is the `<Requires
LocalVar="AIRLINER_FLIGHT_PHASE"><Range LowerBound="6" UpperBound="6"
/></Requires>` gate.

## Two compounding bugs in this port

### Bug 1 — `<Requires>` was parsed and then dropped

`src/sound/triggers.rs::parse()` only ever looked at a `<Sound>`'s `<Range>`
child; `<Requires>` children were never read into `Trigger` at all (the old
`Trigger` struct had no field for them). Evidence: the pre-fix `parse()` body
matched only on `sound.children.iter().find(|c| c.name == "Range")` — no
`"Requires"` branch existed anywhere in the file.

Effect: **every** `<Requires>`-gated entry in the package's `sound.xml` — 30
of the 273 `<Sound>` entries — fired (or looped) purely on its own
`LocalVar`/`SimVar` range, ignoring the second condition that MSFS also
requires. For `cabin_crew_seats_landing` that means the flight-phase gate
was silently absent, so `A32NX_CABIN_READY` reaching 1 on the ground was
enough by itself to fire the PA — a flight-phase bug, exactly the kind the
task brief called out.

Fix (`src/sound/triggers.rs`): `Trigger` gained a `requires: Vec<Require>`
field; `parse()` now reads every `<Requires>` child's `LocalVar`/`SimVar`
(+`Index`) and `<Range>` the same way it already did for the entry's own
condition. `TriggerState::step` now takes a `requires_hold: bool` (computed
by the caller, since evaluating an arbitrary second variable needs `Vars`,
which `triggers.rs` deliberately does not depend on) and treats the entry as
outside its range whenever any `Requires` does not hold, regardless of the
entry's own value. `src/sound/mod.rs::evaluate_triggers` computes
`requires_hold` by reading each `Require`'s variable through the same
`read_named` used for the main variable, and passes it into `step`.

Because `AIRLINER_FLIGHT_PHASE` is not yet tracked anywhere else in this
port (checked: no other file registers or reads that name), `read_named`
returns its documented "absent" value of `0.0` for it — which is outside
`[6, 6]`, so `requires_hold` is `false` and the PA now stays silent until
that variable is wired up and genuinely reaches 6. This is the same
"real-absent-value-over-fabricated-one" contract `read_named` already
documents, so no separate fix was needed to make the gate safe by default.

### Bug 2 — a one-shot (`Continuous="false"`) sound could still loop forever, unstoppably

`src/sound/mod.rs::evaluate_triggers` handled `Action::PlayOnce` (a
`Continuous="false"` entry) with:

```rust
Action::PlayOnce => self.fire_event(&event, None, None),
```

`force_loop: None` means `flatten()` falls back to the leaf's own Wwise
`Loop` property (`looped: force_loop.unwrap_or(s.loop_count == 0)`,
`src/sound/mod.rs` `flatten`), where `loop_count == 0` means "loop forever"
(`src/sound/wwise.rs:683-685`, `SoundPlay::loop_count` doc). That fallback is
correct for `AvionicSounds`/instrument sounds (`drain_instrument_queue`),
which have no `sound.xml` `Continuous` of their own and are meant to honour
whatever the bank says — but a `sound.xml` entry with `Continuous="false"` is
by contract a one-shot ("plays once each time the value enters", per this
file's own module doc), and nothing in this reader ever sends the event's
Stop actions (also documented: "an event's own Stop actions ... are not
applied, since they are rare in these banks"). So if the announcement's own
Wwise object was authored with an infinite `Loop` (a real, plausible content
pattern for a PA/chime that the *original* engine's own event graph, or a
script-issued Stop, would end — machinery this reader does not reproduce),
`PlayOnce` would start it looping and then have no way to ever stop it:
`Action::PlayOnce` also passes `trigger: None` to `fire_event`, and
`play_pcm` only remembers a channel in `state.active` when `trigger` is
`Some`, so even `Sound::release()` (which only iterates `state.active`)
could not silence it. That is the "nonstop, looping" half of the report.

Fix (`src/sound/mod.rs::evaluate_triggers`):

```rust
Action::PlayOnce => self.fire_event(&event, Some(false), None),
```

`PlayOnce` now always forces non-looping playback, regardless of the bank's
own `Loop` property, matching the `Continuous="false"` contract. The
`AvionicSounds`/instrument-sound path (`drain_instrument_queue`) is
unchanged and still passes `force_loop: None`, so it keeps honouring the
bank's own `Loop` property as intended there.

## Why "on the ground at spawn"

Both bugs line up exactly with the report: the missing flight-phase gate
(bug 1) let the PA fire from the ground-default `A32NX_CABIN_READY=1`
instead of only during descent, and the missing loop override (bug 2) meant
that once fired it could never be told to stop.

## Files changed

- `src/sound/triggers.rs`: added `Require`/`Trigger::requires`, `<Requires>`
  parsing, `TriggerState::step`'s new `requires_hold` parameter. Added test
  `a_requires_gate_blocks_the_trigger_until_its_own_condition_holds`
  (reproduces the exact `cabin_crew_seats_landing`/`AIRLINER_FLIGHT_PHASE`
  shape). Updated the two pre-existing `step(...)` call sites in tests for
  the new parameter.
- `src/sound/mod.rs`: `evaluate_triggers` now computes and passes
  `requires_hold`; `Action::PlayOnce` forces `force_loop: Some(false)`.
  Added test `a_forced_non_loop_overrides_the_banks_own_infinite_loop_property`
  (constructs a leaf with `loop_count: 0` and checks `PlayOnce`'s forced
  `Some(false)` overrides it, while the unforced/instrument-sound path still
  honours it). Updated the module doc comment describing loop precedence.

## Audit of every other sound trigger for the same edge/level bug class

Went through `src/sound/triggers.rs`, `src/sound/mod.rs`, `src/sound/wwise.rs`,
`src/sound/vorbis.rs`, and `src/afs_events.rs` (the files this task allowed
touching) for the same two bug shapes — a level condition treated without
its full gate, and a one-shot re-armed or left unstoppable.

- **`src/afs_events.rs`**: no sound-triggering logic at all (FCU/autopilot
  key-event → one-frame-input translation only). `on_command` takes only a
  command's begin phase and `PENDING` is drained and cleared every tick
  (`take()`), so each press is a genuine one-shot edge, not re-armed.
  `on_priority_command`'s held state is set on begin/cleared on end, which is
  correctly level-tracked (it is meant to be held, not one-shot) and has its
  own passing test (`begin_holds_and_end_releases`). No changes needed here.
- **`triggers.rs`/`mod.rs`'s edge state machine** (`TriggerState::step`):
  the `(continuous, previous, inside)` match is edge-correct for both
  continuous (`StartLoop`/`StopLoop` only on a transition) and one-shot
  (`PlayOnce` only on a `false -> true` transition, never on a value already
  true at the very first read) — unaffected by this fix beyond gating
  `inside` on `requires_hold` too, which is exactly the missing case.
- **Every `<Requires>`-gated entry (30 of 273 `<Sound>` entries in the
  package's `sound.xml`)** was affected by bug 1 identically — the fix in
  `triggers::parse`/`evaluate_triggers` is general, not special-cased to
  `cabin_crew_seats_landing`, so all 30 are fixed by the same change:
  `apuflapopen`, `TR380`, `Engine_{1,2,3,4}_FF`, `noseroll380`,
  `cabingroll380`, `Centerthump380`, `smoothtouch380`, `medtouch380`,
  `hardtouch380`, `cabtouchsmooth1`, `cabtouchmed1`, `a380cabtouchhard`,
  `Spoiler{0,50,100}`, `rattle_ground`, `wiper_{slow,fast}{L,R}`,
  `cockpit_cabin_call_fwd` (×2), `cockpit_cabin_call_aft`, `emercabincall`,
  `evachorncockpit`, `cabin_crew_seats_landing`, `cvr_test`.
- **Every `Continuous="false"` entry (110 of 273)** was affected by bug 2
  identically for the same reason — the fix forces non-looping for the whole
  `Action::PlayOnce` path, not just this one event, so all 110 are covered by
  the same one-line change.
- **`Action::StartLoop`/`StopLoop`** (`Continuous="true"` entries) already
  forced `Some(true)` and tracked the trigger index for a later `StopLoop`
  to reach — this path was already correct and untouched.
- **`play_instrument_sound`/`drain_instrument_queue`** (`AvionicSounds`,
  script-driven, e.g. `PLAY_INSTRUMENT_SOUND`): each call is a genuine
  one-time request from the instrument's own script, not a level condition
  this plugin polls, so there is no edge to get wrong here; left as-is,
  still deferring to the bank's own `Loop` property by design (documented,
  and now explicitly contrasted with `PlayOnce` in the module doc).
- **`wwise.rs`/`vorbis.rs`**: pure bank/media parsing and Vorbis decode, no
  trigger or playback-lifetime logic; nothing in this class applies.
