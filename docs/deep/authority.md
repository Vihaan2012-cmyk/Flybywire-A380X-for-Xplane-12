# Authority: making the deep models the aircraft, not a commentary on it

Three systems are now modelled twice. FlyByWire's ported `a380_systems`
has an electrical system, a hydraulic system and a pneumatic system, and
so does `src/deep/`. FlyByWire's is coarse and complete; ours is fine and
partial. Today FlyByWire's is the aircraft and ours watches it: the live
layer hands the deep areas `Truth::ac_bus_volts` and friends, read from
what FlyByWire published, so our 1,643 electrical failures can compute
whatever they like and the crew will never see it. The ELEC page, the
packs, the hydraulics and every FlyByWire consumer go on believing
FlyByWire's own solve.

This document is how that inverts.

## The obvious approach does not work

The tempting move is the one `integration::flight_control_surfaces`
already uses for control surfaces: let FlyByWire's systems tick, then
overwrite the variables it published with ours before anything reads
them. For surfaces that is exactly right, because the *only* consumer of
`HYD_AIL_LEFT_INWARD_DEFLECTION` is the code that drives X-Plane's
surfaces — overwrite it and the whole pipeline follows.

Electrical is not like that. Inside `a380_systems`, nothing reads the
electrical variables. The packs, the hydraulics, the avionics and the
displays' own source data all reach the electrical system through Rust
objects (`ElectricalBus`, the `ElectricalBuses` trait), and the variables
are write-only — a rendering of internal state for the displays.

So overwriting them buys the worst of both: the ELEC page would show our
numbers while every system in the aircraft continued to run on
FlyByWire's, and the two would silently disagree. A bus would read 0 V on
the display while the packs it feeds ran normally. That is not authority;
it is a lie on a screen.

## Where the lever actually is

FlyByWire's systems do read variables — just not those ones. They read
their *inputs*: pushbutton positions, engine speeds, external power
availability, and their own failure activations. Anything that changes
those changes FlyByWire's internal solve, and therefore changes the
displays, the consumers and the crew's experience together, with no
possibility of disagreement.

Two of those are unsuitable. Pushbuttons belong to the crew, and writing
them would mean fighting the pilot for the switch. Engine speeds are
already ours and already flow through.

The third is exactly the right shape. FlyByWire's own failure system
(`FailuresConsumer`, driven from `src/failures.rs`'s `drive`) exists to
tell `a380_systems` that a component it models has failed, and
`a380_systems` responds by solving as though it had. That is a supported
input, it is the mechanism FlyByWire's own failure list uses, and it is
already wired in this crate.

## The rule

**The deep model is authoritative. It expresses its authority through the
coarsest FlyByWire input that can carry the verdict, and publishes
directly only what FlyByWire does not model at all.**

Concretely, three levels:

1. **Below FlyByWire's resolution — publish directly.** Per-load current,
   per-feed breaker state, I²t heating, arc-fault energy, individual load
   shedding. FlyByWire has no concept of these, so nothing competes and
   the deep area's published variables stand on their own. This is where
   most of the 1,643 failures live, and it needs no coupling at all.

2. **At FlyByWire's resolution — drive its failure.** When the deep
   network concludes that a component FlyByWire *also* models has failed
   — a generator, a TR, a battery, a contactor, an engine-driven pump, a
   bleed valve — the deep area does not argue with FlyByWire about
   voltages. It activates FlyByWire's own failure for that component.
   FlyByWire then re-solves, the ELEC/HYD/BLEED pages show it, and every
   consumer inside `a380_systems` sees it. One aircraft, one answer.

3. **Above the deep model — FlyByWire keeps it.** Where FlyByWire models
   something the deep area does not, FlyByWire stays authoritative and
   the deep area keeps reading it from `Truth`, exactly as now.

Level 2 is the whole of the work. It needs one artifact per system: a
table mapping a deep component to the FlyByWire failure that represents
it, plus the condition in the deep model that trips it.

## What this changes in the live layer

`Truth`'s `ac_bus_volts` / `dc_bus_volts` / `hydraulic_pressure_pa`
fields keep their current meaning — they are still what FlyByWire
published last frame — but their doc comments are now wrong about *why*.
They are no longer "the truth the deep areas defer to"; they are
FlyByWire's answer after it has already been told what the deep model
concluded. The deep areas read them to stay consistent with the coarse
solve, not because the coarse solve outranks them.

The `Area` trait gains a second output alongside `publish`: a way for an
area to say "this FlyByWire failure should now be active". The plugin
collects those each frame and feeds them into `failures::drive` next to
the ones the crew armed from the EFB, so a failure the deep model
*derived* and a failure the crew *armed* reach `a380_systems` by the same
path.

## The honest limits

- **One frame of lag.** The deep area concludes a generator has failed on
  frame N; FlyByWire solves without it on frame N+1. At 30-60 Hz this is
  invisible for anything electrical, and it is the same lag
  `extra_backend_fbw.rs` already accepts. It is not invisible for
  anything that must be simultaneous, and nothing here is.
- **Granularity is FlyByWire's, not ours, at level 2.** If our model
  concludes that generator 1 is producing 60% of rated current because of
  a partially failed exciter, FlyByWire's failure system can only be told
  "generator 1 failed" or nothing. Continuous degradation of a
  FlyByWire-modelled component either rounds to a trip or stays invisible
  to FlyByWire. Where that matters, the fix is a `SourcePatch` giving
  FlyByWire's component a continuous input — which is a real option, not a
  hypothetical, but it is per-component work and should be done only where
  the partial state is worth it.
- **It can fight the crew.** A derived failure that the crew cannot clear
  is correct when the component really has failed and infuriating when
  the model is wrong. Every level-2 coupling must be visible on the EFB's
  Study page as "derived from <deep component>", so a pilot can always see
  why the aircraft thinks something is broken.

## Order of work

1. Electrical, because it is the largest and it feeds the other two.
2. Hydraulics, whose deep model already computes pump and reservoir
   state that FlyByWire represents coarsely.
3. Pneumatics, last, because the deep duct model and FlyByWire's bleed
   system overlap most and the mapping needs the most care.

Each is: build the component-to-failure table, add the derived-failure
output to that area's live system, and prove it with a test that arms a
deep failure and asserts the corresponding FlyByWire failure goes active
and the coarse solve changes.
