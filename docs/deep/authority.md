# Authority: making the deep models the aircraft, not a commentary on it

Three systems are now modelled twice. FlyByWire's ported `a380_systems`
has an electrical system, a hydraulic system and a pneumatic system, and
so does `src/deep/`. FlyByWire's is coarse and complete; ours is fine and
partial. FlyByWire's used to be the aircraft and ours only watched it:
the live layer handed the deep areas `Truth::ac_bus_volts` and friends,
read from what FlyByWire published, so our 1,643 electrical failures
could compute whatever they liked and the crew would never see it. The
ELEC page, the packs, the hydraulics and every FlyByWire consumer went
on believing FlyByWire's own solve.

This document is how that inverts. The design is below; the tables,
what was built and what could not be are further down, and it is all
implemented apart from one plugin-side patch, which is written out in
full under "The plugin-side patch".

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
it, plus the condition in the deep model that trips it. All three are
built: 25 couplings in electrical, 16 in hydraulics, 8 in pneumatics.
Note that a battery and a contactor turn out to be in the list above by
mistake -- FlyByWire has no failure for either, so both are level 1.

## What this changes in the live layer

`Truth`'s `ac_bus_volts` / `dc_bus_volts` / `hydraulic_pressure_pa`
fields keep their current meaning -- they are still what FlyByWire
published last frame -- but their doc comments were wrong about *why*.
They are no longer "the truth the deep areas defer to"; they are
FlyByWire's answer after it has already been told what the deep model
concluded. The deep areas read them to stay consistent with the coarse
solve, not because the coarse solve outranks them. `src/deep/live.rs`'s
own doc comments now say so.

The `Area` trait has a second output alongside `publish`:

```rust
fn derived_failures(&self, _out: &mut dyn FnMut(DerivedFailure)) {}
```

where a `DerivedFailure` carries the FlyByWire failure id, a magnitude in
0..1, the deep component that concluded it and a one-phrase reason. The
default is empty, which is the right answer for the thirteen areas that
model something FlyByWire does not model at all.

An area emits its **whole coupling table every frame**, healthy entries
at magnitude 0.0, so the derived set is a *level* and not an event:
nothing has to remember to clear a derived failure when the component
recovers. `Deep::tick` collects every entry above zero -- after every
area has ticked and before any has published, so a verdict is this
frame's and not half of one -- and exposes them as
`Deep::derived_failures()` (with component and reason) and
`Deep::derived_magnitudes()` (id to magnitude, worst wins). The plugin
feeds those into `crate::failures` next to the ones the crew armed from
the EFB, so a failure the deep model *derived* and a failure the crew
*armed* reach `a380_systems` by the same path.

Every coupling is also **published**, as
`A32NX_DEEP_DERIVED_FBW_FAILURE_<id>` = its magnitude, by the area that
owns it, every frame, healthy or not. That is what makes a derived
failure visible rather than a mystery: the Study page reads the same
variables as everything else, and `Deep::derived_failures()` carries the
component and the reason to print beside it.

## The tables

### Electrical (`src/deep/electrical/live.rs`)

25 couplings. Every verdict is **machine-level** -- the machine's own
overload element, its own regulated terminal, its own health parameter,
or, for a bus, its own feeder breaker. Deliberately *not* the area's own
published `ELEC_*_FAULT` annunciations: those include bus-caused
undervoltage, because that is what the crew is shown, but a verdict
handed to FlyByWire must not, or a busbar fault would leave a healthy
generator failed on FlyByWire's side long after the feeder cleared it.
There is a test for exactly that
(`a_bus_short_is_never_blamed_on_the_generator_feeding_that_bus`).

| deep component | FlyByWire failure | id | condition in the deep model |
| --- | --- | --- | --- |
| `24_elec.vfg-1..4` | `Generator(1..4)` | 24_020..24_023 | its own I^2t overload element ran to the trip; **or** driven and excited but regulating outside MIL-STD-704F's 108-118 V band; **or** `winding_degradation >= 0.5` |
| `24_elec.apu-gen-1..2` | `ApuGenerator(1..2)` | 24_030..24_031 | as above, against APU speed |
| `24_elec.tr-1`, `tr-2`, `tr-ess`, `tr-apu` | `TransformerRectifier(1..4)` | 24_000..24_003 | `winding_degradation >= 0.5` |
| `24_elec.static-inv` | `StaticInverter` | 24_004 | `efficiency_loss >= 0.5` |
| `24_elec.bus.AC1..AC4` | `ElectricalBus(AlternatingCurrent(1..4))` | 24_100..24_103 | that bus's feeder breaker has tripped open |
| `24_elec.bus.AC_ESS` | `ElectricalBus(AlternatingCurrentEssentialShed)` | 24_105 | feeder breaker open |
| `24_elec.bus.AC_EMER` | `ElectricalBus(AlternatingCurrentEssential)` | 24_104 | feeder breaker open |
| `24_elec.bus.AC_GND_FLT_SVC` | `ElectricalBus(AlternatingCurrentGndFltService)` | 24_107 | feeder breaker open |
| `24_elec.bus.DC1`, `DC2` | `ElectricalBus(DirectCurrent(1..2))` | 24_108..24_109 | feeder breaker open |
| `24_elec.bus.DC_ESS` | `ElectricalBus(DirectCurrentEssential)` | 24_110 | feeder breaker open |
| `24_elec.bus.DC_HOT1`, `DC_HOT2` | `ElectricalBus(DirectCurrentHot(1..2))` | 24_113..24_114 | feeder breaker open |
| `24_elec.bus.DC_APU` | `ElectricalBus(DirectCurrentNamed("309PP"))` | 24_112 | feeder breaker open |
| `24_elec.bus.DC_GND_FLT_SVC` | `ElectricalBus(DirectCurrentGndFltService)` | 24_117 | feeder breaker open |

Two of those mappings look wrong and are not. FlyByWire's A380 uses
`AlternatingCurrentEssentialShed` for the **AC ESS** bus (400XP) and
`AlternatingCurrentEssential` for **AC EMER** (491XP), with a `// TODO`
saying so at `a380_systems/src/electrical/alternating_current.rs:46-56`;
`failures::failure_name` names 24_104 "AC EMER" and 24_105 "AC ESS" for
the same reason. And `DirectCurrentNamed("309PP")` is the APU battery
bus, which is what FlyByWire's TR APU feeds
(`direct_current.rs:204-207`) -- the busbar this model calls `DC_APU`.

A bus is derived as failed only when its **feeder breaker has tripped**,
never when it is merely unpowered. FlyByWire works out an unpowered bus
for itself, and claiming it here would keep the bus dead after its own
source came back.

### Hydraulics (`src/deep/hydraulics/live.rs`)

16 couplings.

| deep component | FlyByWire failure | id | condition in the deep model |
| --- | --- | --- | --- |
| `29_hyd.green_edp_1a`, `1b`, `2a`, `2b` | `EnginePumpOverheat(Edp1a, Edp1b, Edp2a, Edp2b)` | 29_010..29_013 | `(1-seizure)(1-displacement_loss) < 0.5` |
| `29_hyd.yellow_edp_3a`, `3b`, `4a`, `4b` | `EnginePumpOverheat(Edp3a, Edp3b, Edp4a, Edp4b)` | 29_014..29_017 | as above |
| `29_hyd.green_electric_pump_a`/`_b` | `ElecPumpOverheat(GreenA, GreenB)` | 29_006..29_007 | as above |
| `29_hyd.yellow_electric_pump_a`/`_b` | `ElecPumpOverheat(YellowA, YellowB)` | 29_008..29_009 | as above |
| `29_hyd.green_reservoir`, `yellow_reservoir` | `ReservoirLeak(Green, Yellow)` | 29_000..29_001 | that reservoir has run down to its own low-level switch |
| `29_hyd.green_reservoir`, `yellow_reservoir` | `ReservoirAirLeak(Green, Yellow)` | 29_002..29_003 | the bootstrap air supply is present (at least half the regulator's setting) and the reservoir is still below its pressure switch |

A pump's verdict is about the **machine**, not its operating point. A
pump shut down by a pulled fire handle, destroked by its own compensator
or starved of bus power is not a failed pump, and none of those touch the
capability product (`a_pulled_fire_handle_is_not_a_failed_pump`). The air
verdict is gated on a supply being present for the same reason: a cold,
dark aircraft has unpressurised reservoirs because nothing is
pressurising them, not because they leak.

### Pneumatics (`src/deep/pneumatic_ducts/live.rs`)

8 couplings -- the narrowest of the three, and the only one that keeps
the deep model's own granularity.

| deep component | FlyByWire failure | id | condition in the deep model |
| --- | --- | --- | --- |
| `36_pneu.engine_upstream_valve_stage` (HP valve, per engine) | "Engine n HP bleed valve stuck", `PNEU_VALVE_FAILED:1..4` | 36_008..36_011 | `upstream[n].hp_valve_stuck`, passed across **unrounded** |
| `36_pneu.engine_upstream_valve_stage` (PR valve, per engine) | "Engine n bleed valve stuck", `PNEU_VALVE_FAILED:5..8` | 36_012..36_015 | `upstream[n].pr_valve_stuck`, unrounded |

FlyByWire's `ValveSeizure` (`a380_systems/src/pneumatic.rs:53-90`) takes a
continuous 0..1 loss of valve authority measured from where the valve
stood when it seized -- the same physical fault, the same meaning and the
same scale as this model's own stuck-valve parameters. So a half-seized
valve is half-seized on both sides, and level 2 does not have to round.

These are ids in the plugin's own extra catalogue rather than FlyByWire's
146: `failures::extra::write_pneumatic_valves` turns their magnitude into
`PNEU_VALVE_FAILED:n` on every tick, which is the input `a380_systems`
actually reads. They are still "FlyByWire's own failure for that
component" in every sense that matters: a supported, continuous,
per-component input that `a380_systems` re-solves against.

The deep catalogue registers one id per fault mechanism per component
*class*, not per instance, so one armed HP-valve failure seizes the HP
valve on all four engines -- here and on FlyByWire's side alike. That is
the deep registry's granularity, not the coupling's.

## What is not coupled, and why

Level 1 -- the great majority, and the reason the rule is worth stating
in this direction: FlyByWire has no concept of any of it, so there is
nothing to couple and the published variables stand alone. Per-load
current, per-feed breaker state and I^2t heating, arc energy, the
load catalogue, wire-bundle chafe, every contactor and diode, both
batteries, the RAT, ground power, load shedding, the hydraulic
compensator curve, case drain, cavitation, accumulators, priority and
relief valves, the return filter, the five consumer branch line leaks,
fluid temperature, every duct, leak, rupture, insulation failure,
precooler and overheat-detection loop, and every bay temperature.

Genuinely wanted and **not** done, with the reason:

* **An ODLS trip cannot shut FlyByWire's bleed.** When the deep duct
  model's overheat-detection loop trips and latches an engine's bleed
  isolated, there is no FlyByWire input that *shuts* a valve. Seizure
  freezes a valve where it stands, which for an open valve would leave
  FlyByWire bleeding from an engine this model has isolated. Seizing it
  would not be the same statement, so it is not made. The fix is a
  `SourcePatch` giving `CoreProcessingInputOutputModuleAUnit` an external
  close command, which is real work outside this pass.
* **Batteries, the RAT, ground power, contactors and diodes** have no
  FlyByWire `FailureType` at all. Level 1.
* **`AC_ESS_SHED`, `DC_ESS_SHED`, `DC_BAT`.** FlyByWire's A380 has no
  separate shed busbar (it reuses the shed bus *type* for AC ESS itself,
  above) and no DC BAT busbar, and its DC ESS sub-bus `108PH` is not in
  the registered failure catalogue. Mapping a shed bus onto the bus it
  sheds from would fail the wrong bus.
* **`ReservoirReturnLeak`** (29_004, 29_005) has no deep counterpart:
  this model has no separate return-line volume to conclude anything
  about. Left to the crew to arm directly.
* **Cross-bleed valves, the APU bleed valve and the pack flow valves**
  (`PNEU_VALVE_FAILED:9..16`) are modelled by FlyByWire and have no
  stuck-valve fault in the deep duct model. Nothing to derive; if the
  deep model grows one, the coupling is one row of the table.
* **A deep model that says a component is *healthy* cannot un-fail
  FlyByWire's.** Level 2 only adds. If the crew pulls an engine fire
  handle, FlyByWire shuts its own fire valve whatever the deep model's
  stuck-valve state says.
* **Level 3, FlyByWire's to keep:** its AC 247XP and DC 247PP EHA buses,
  DC HOT ESS and DC HOT APU, the trim-air hot-air valves, the FDAC, TADD,
  VCM, OCSM and CPIOM computers, and everything else the deep areas do
  not model. The deep areas keep reading those from `Truth`, exactly as
  before.

## The plugin-side patch

`src/deep/plugin.rs`, `src/lib.rs` and `src/failures.rs` were not edited
by this pass (another agent is in them). This is the patch to apply, and
it is the only thing left between `Deep::derived_magnitudes()` and
`a380_systems`.

### The recommended form: a derived level in `crate::failures`

`failures::State` already merges two sources into one effective set --
the crew's `active`/`magnitudes` and the components system's
`component_levels`, with `magnitude()` taking the larger. A derived level
is a third source of exactly the same shape, and adding it that way is
what keeps a derived failure from ever clobbering, or being clobbered by,
one the crew armed.

In `src/failures.rs`:

```rust
 struct State {
     registered: Vec<u64>,
     active: BTreeSet<u64>,
     magnitudes: std::collections::BTreeMap<u64, f64>,
     dirty: bool,
     component_levels: std::collections::BTreeMap<u64, f64>,
+    /// What `deep::live`'s areas concluded this frame about components
+    /// FlyByWire also models (`docs/deep/authority.md` level 2). A third
+    /// source alongside the crew's own arming and the components system,
+    /// combined the same way: a failure acts at the largest of the three,
+    /// so a derived failure can never clear one the crew armed, and the
+    /// crew can never hide one the deep model is presently concluding.
+    derived_levels: std::collections::BTreeMap<u64, f64>,
 }

 static STATE: Mutex<State> = Mutex::new(State {
     registered: Vec::new(),
     active: BTreeSet::new(),
     magnitudes: std::collections::BTreeMap::new(),
     dirty: true,
     component_levels: std::collections::BTreeMap::new(),
+    derived_levels: std::collections::BTreeMap::new(),
 });

 impl State {
     fn effective(&self) -> BTreeSet<u64> {
         let mut set = self.active.clone();
         set.extend(self.component_levels.keys().copied());
+        set.extend(self.derived_levels.keys().copied());
         set
     }
 }

+/// The deep areas' level-2 verdicts (`Deep::derived_magnitudes`), after
+/// every tick. Marks the set changed when the *ids* differ, matching
+/// `set_component_levels`: a magnitude that merely moves is read fresh
+/// by `magnitude()` and needs no re-apply.
+pub fn set_derived_levels(levels: std::collections::BTreeMap<u64, f64>) {
+    with_state(|s| {
+        if s.derived_levels != levels {
+            let keys_changed = !s.derived_levels.keys().eq(levels.keys());
+            s.derived_levels = levels;
+            s.dirty |= keys_changed;
+        }
+    });
+}
```

and two reads, each beside the `component_levels` line it mirrors:

```rust
 pub fn is_active(id: u64) -> bool {
-    with_state(|s| s.active.contains(&id) || s.component_levels.contains_key(&id)).unwrap_or(false)
+    with_state(|s| s.active.contains(&id) || s.component_levels.contains_key(&id) || s.derived_levels.contains_key(&id))
+        .unwrap_or(false)
 }

 pub fn magnitude(id: u64) -> f64 {
     with_state(|s| {
         let armed = if s.active.contains(&id) { s.magnitudes.get(&id).copied().unwrap_or(1.0) } else { 0.0 };
-        armed.max(s.component_levels.get(&id).copied().unwrap_or(0.0))
+        armed
+            .max(s.component_levels.get(&id).copied().unwrap_or(0.0))
+            .max(s.derived_levels.get(&id).copied().unwrap_or(0.0))
     })
     .unwrap_or(0.0)
 }
```

plus `s.derived_levels.clear();` in `reset_all` and `reset_for_tests`
beside the existing `s.component_levels.clear();`.

`armed_magnitude` and `active_magnitudes` are deliberately left alone:
the first is what the components system combines *before* its own
settings, and the second is what persistence saves. A derived failure is
recomputed from the model every frame and must not be written to the save
file as though the crew had armed it.

In `src/deep/plugin.rs`, one line at the end of `DeepLayer::tick`:

```rust
     pub fn tick(&mut self, vars: &mut Vars, xplm: Option<&Xplm>, delta: f64) {
         let truth = self.truth(vars, xplm, delta);
         let faults = self.faults();
         let Self { deep, publisher, .. } = self;
         let mut at = 0usize;
         deep.tick(truth, &faults, &mut |name, value| {
             let (id, in_step) = publisher.resolve(vars, at, name);
             at += in_step as usize;
             vars.write(&id, value);
         });
+        // `docs/deep/authority.md` level 2: what the areas concluded this
+        // frame about components FlyByWire also models, handed to the
+        // failure system beside the ones the crew armed. `Failures::apply`
+        // picks them up in the next frame's `before_systems`, which is the
+        // one frame of lag that document records.
+        crate::failures::set_derived_levels(self.deep.derived_magnitudes());
     }
```

Nothing in `src/lib.rs` changes: `correctness::before_systems` already
calls `Failures::apply` every frame, and `apply` already calls
`extra::write_pneumatic_valves` unconditionally, so the pneumatic
magnitudes reach `PNEU_VALVE_FAILED:n` on the frame they change without
waiting for the active set to go dirty.

### The fallback, if the `failures.rs` hunk cannot be taken

All of it can be done inside `plugin.rs` alone, at the cost of not being
able to tell a derived failure from an identically-armed crew one:

```rust
 pub struct DeepLayer {
     ...
+    /// What this layer last set from the areas' verdicts, so a verdict
+    /// going away clears exactly what it armed.
+    derived_applied: std::collections::BTreeMap<u64, f64>,
 }
```

```rust
+    fn apply_derived_failures(&mut self) {
+        let now = self.deep.derived_magnitudes();
+        for (&id, &was) in &self.derived_applied {
+            // Only clear what is still exactly what this layer set: if the
+            // crew has since changed it, it is theirs.
+            if !now.contains_key(&id) && (crate::failures::armed_magnitude(id) - was).abs() < 1e-9 {
+                crate::failures::set_magnitude(id, 0.0);
+            }
+        }
+        for (&id, &m) in &now {
+            if crate::failures::armed_magnitude(id) < m {
+                crate::failures::set_magnitude(id, m);
+            }
+        }
+        self.derived_applied = now;
+    }
```

called from the same place. The ambiguity is real: a crew failure armed
at exactly the derived magnitude is cleared along with the derived one
when the component recovers, and persistence saves derived failures as
though the crew had armed them. That is why the first form is the
recommended one.

## The honest limits

- **One frame of lag.** The deep area concludes a generator has failed on
  frame N; `Failures::apply` hands it to `a380_systems` in frame N+1's
  `before_systems`, and FlyByWire solves without it from there. At 30-60
  Hz this is invisible for anything electrical, and it is the same lag
  `extra_backend_fbw.rs` already accepts. It is not invisible for
  anything that must be simultaneous, and nothing here is.
- **Granularity is FlyByWire's, not ours, at level 2 -- except for the
  bleed valves.** A generator at 60% output from a partly failed exciter
  either rounds to a trip or stays invisible: `DEGRADED_BEYOND_HALF` and
  `PUMP_LOST_CAPABILITY` are both a plain 0.5, chosen as the point at
  which a machine has lost more of its range than it has left, and both
  are labelled GENERIC in the code because there is nothing to cite for a
  rounding point. Below it, the deep model keeps the real degradation --
  a stator that sags harder under load is still a real, published,
  ELEC-page-visible thing -- and FlyByWire is told nothing. Two tests
  pin both halves of that
  (`a_partly_destroked_pump_stays_below_flybywires_resolution`,
  `a_stator_degraded_past_half_reaches_flybywire_as_a_failed_generator`).
  Where the partial state is worth more than that, the fix is a
  `SourcePatch` giving FlyByWire's component a continuous input -- which
  is what `PNEU_VALVE_FAILED:n` already is, and is why the pneumatic
  couplings need no rounding at all.
- **FlyByWire's response can be slower than the verdict.** Its pump
  failures are an overheat *process*, not an instant loss: the heat state
  has a 30 s time constant and the pump is damaged and declutched only
  after a further 2-3 minutes (`hydraulic/mod.rs`'s
  `HEATING_TIME_CONSTANT_MEAN_S` / `DAMAGE_TIME_CONSTANT`), and it only
  heats while the pump is actually turning. A deep pump that seized while
  its engine was shut down stays invisible to FlyByWire until the engine
  is started. The deep model's own pressure and flow are right from the
  first frame either way.
- **It can fight the crew.** A derived failure that the crew cannot clear
  is correct when the component really has failed and infuriating when
  the model is wrong. That is why every coupling publishes
  `DEEP_DERIVED_FBW_FAILURE_<id>` every frame and carries its own
  component and reason, and why every condition above is a statement
  about a machine rather than about a symptom.

## Order of work

1. Electrical, because it is the largest and it feeds the other two.
   Done: 25 couplings.
2. Hydraulics, whose deep model already computes pump and reservoir
   state that FlyByWire represents coarsely. Done: 16 couplings.
3. Pneumatics, last, because the deep duct model and FlyByWire's bleed
   system overlap most and the mapping needs the most care. Done: 8
   couplings, plus the recorded ODLS gap above.

Each was: build the component-to-failure table, add the derived-failure
output to that area's live system, and prove it with a test that arms a
deep failure and asserts the corresponding FlyByWire failure goes active
and the coarse solve changes. All three of those end-to-end tests build a
real `Simulation<A380>` and read FlyByWire's own variables back:

- `a_derived_bus_failure_changes_flybywires_own_solve` -- a deep busbar
  short clears through its feeder, and `A32NX_ELEC_DC_HOT_1_BUS_IS_
  POWERED` goes from 1 to 0 in FlyByWire's own solve.
- `a_derived_reservoir_leak_changes_flybywires_own_solve` -- a deep
  reservoir drained to its switch drains
  `A32NX_HYD_GREEN_RESERVOIR_LEVEL` too.
- `a_derived_valve_seizure_changes_flybywires_own_solve` -- a deep PR
  valve seizure holds `A32NX_PNEU_ENG_1_PR_VALVE_OPEN` shut where it
  would otherwise have opened.

None of the three touches the process-wide `crate::failures` state: each
maps the derived id to its `FailureType` (or its `PNEU_VALVE_FAILED:n`
variable) exactly as `Failures::apply` does, so they prove the coupling
without making the test suite order-dependent.
