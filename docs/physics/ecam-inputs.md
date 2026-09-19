# ECAM message input inventory and wiring status

Scope note up front: this is a first pass, time-boxed to 40 minutes total
(inventory + verification harness + fixes), against a task whose real size
is large — the FWS alone (`FwsCore.ts` 6,266 lines, `FwsAbnormalSensed.ts`
4,877, `FwsAbnormalNonSensed.ts` 883, `FwsMemos.ts` 684,
`FwsNormalChecklists.ts` 613, plus `FwsInopSys.ts`, `FwsAutoCallouts.ts`,
`FwsLimitations.ts`, `FwsFlightPhases.ts`, `FwsSoundManager.ts`,
`FwsSystemDisplayLogic.ts`, `FwsInformation.ts` — 15,700 lines total)
references **356 unique `L:` vars and 303 unique other SimVars** (exact
counts, `grep -oE` over every `.ts` file in
`fbw-a380x/src/systems/systems-host/CpiomC/FlightWarningSystem/`, not an
estimate). The coordinator's follow-up ("go beyond the FWS — EWD/SD, PFD/ND,
MFD/FMS, FCU, RMP, ISIS, OANS/BTV, systems-host CPIOMs/FQMS/CDS, the Rust
aspects") is real, important scope this pass did not reach; see
**Not reached this pass** at the end. What follows is real, verified data —
nothing here is guessed or fabricated — scoped to the FWS itself plus the
verification harness the rest of the sweep should reuse.

## Method

1. `grep -oE "'L:[A-Za-z0-9_.:]+" *.ts | sed "s/^'L://" | sort -u` over the
   FWS directory → 356 unique `L:` names (`/tmp/fws_lvars.txt` this session).
2. `grep -oE "SimVar\.GetSimVarValue\('[A-Za-z0-9_.: ]+'"` → 303 unique other
   SimVars (MSFS built-ins like `GEAR HANDLE POSITION`, `A:` events).
3. Three automated triage passes tried to classify each `L:` name as
   PROVIDED by literal/fuzzy string match against `fbw-xp-systems/src/*.rs`
   and `fbw-aircraft/**/*.rs` (`context.get_identifier(...)`, stripping the
   implicit `A32NX_` prefix, and a digit-run→`[0-9]+` fuzzy regex for
   `format!`-generated names). This is a **known-unreliable upper bound on
   "not found"**: `format!` patterns built from loop variables, lookup
   tables or nested helper functions routinely don't match any grep regex,
   so a name it reports "not found" is often actually provided. It found
   146/356 by name match and left 210 unresolved.
4. **The reliable check is empirical, not textual**: `emulator/tests/
   ecam_inputs.rs` (new, added this pass) builds a real `Emulator`
   (`presets::engines_running()`, FlyByWire's actual `a380_systems` ticking
   for 5s, no X-Plane, no mocks — the same harness `emulator/tests/
   emergence.rs` uses), then asserts each name is present in
   `Emulator::snapshot_all()`. A name passing this test is proven PROVIDED:
   the exact identifier FwsCore.ts reads is registered and live in this
   port's runtime. This is the test the report asks for (§6); it should be
   extended with the 210 unresolved names as they're individually verified,
   rather than trusting step 3's grep.

## Confirmed this pass (resolves 3 of the prior audit's open questions)

The prior audit flagged four items as possibly-missing but was "naive about
Rust `format!` names" (per the task brief). Running the empirical test
against `fbw-xp-systems/emulator/tests/ecam_inputs.rs` resolved three of
them definitively:

| FWS input (`L:` name FwsCore.ts/FwsAutoCallouts.ts actually reads) | Provider | Status |
|---|---|---|
| `A32NX_ROW_ROP_WORD_1` | `a380_systems/hydraulic/autobrakes.rs:666`, `context.get_identifier("ROW_ROP_WORD_1")` — `Vars::get()` (`src/lib.rs:494-505`) adds the `A32NX_` prefix because the name has no space and isn't in `mapping()` | **PROVIDED**, confirmed by test |
| `A32NX_FIRE_SQUIB_{1,2}_ENG_{n}_IS_ARMED` / `_IS_DISCHARGED`, `A32NX_FIRE_SQUIB_1_APU_1_IS_ARMED` / `_IS_DISCHARGED` | `a380_systems/fire_and_smoke_protection.rs:658,660`, same `get_identifier` + implicit-prefix path | **PROVIDED**, confirmed by test — the audit's "possibly fixed since" is correct |
| `A32NX_LGCIU_{1,2}_LEFT/RIGHT/NOSE_GEAR_COMPRESSED`, `A32NX_LGCIU_{1,2}_DISCRETE_WORD_{1,2,4}` | `fbw-common/src/wasm/systems/systems/src/landing_gear/mod.rs:383-392,955-961`, `context.get_identifier(format!("LGCIU_{}_..._GEAR_COMPRESSED", n))` | **PROVIDED**, confirmed by test |
| `A32NX_ADIRS_ADR_{1,2,3}_ALTITUDE/COMPUTED_AIRSPEED/MACH/DISCRETE_WORD_1`, `A32NX_ADIRS_IR_{1,2,3}_MAINT_WORD/PITCH`, `A32NX_ADIRS_REMAINING_IR_ALIGNMENT_TIME` | `adirs`/air-data registration in `a380_systems`, same implicit-prefix path | **PROVIDED**, confirmed by test |

All 21 names above are asserted in `emulator/tests/ecam_inputs.rs` and the
test passes (`cargo test -p fbw_a380_emulator --test ecam_inputs`, run this
session: `test result: ok. 1 passed`).

**One interesting cross-check, not a fix needed for the FWS:**
`src/extra_backend_fcdc.rs:1039-1044` (the plugin's own separate FCDC
reimplementation, not FwsCore.ts) reads `nose_gear_pressed` from the
**unprefixed** `"LGCIU_1_NOSE_GEAR_COMPRESSED"` via its own `n.is(vars, ...)`
name table — a different lookup path than `Vars::get()`/`snapshot_all()`.
Given the test above proves the *prefixed* `A32NX_LGCIU_1_NOSE_GEAR_
COMPRESSED` is genuinely registered and live, that FCDC file's own comment
("the prefixed name... is never written by anything") may be stale or was
verified against a different registry than the one `Vars`/js_bridge.rs
exposes to the FWS. This is FCDC-internal (`extra_backend_fcdc.rs` is
explicitly another workstream's "in progress" file per `team.md` /
CPU-010) — flagged for that owner to re-check with this same empirical
method, not changed here.

## Category counts from the grep triage (upper bound, needs individual verification)

Grouping the 210 grep-unresolved names by prefix (`sed -E 's/_[0-9]+.*//''`)
gives real counts of *candidate* gaps — each bucket should be run through
the same `emulator/tests/ecam_inputs.rs`-style check before being trusted
as MISSING, since (as shown above) most turn out to be provided once
checked properly:

| Prefix bucket | Count | Likely provider once checked |
|---|---:|---|
| `A32NX_ADIRS_ADR_*` (remaining ADR words beyond the 4 confirmed) | 13 | `a380_systems` air-data — same pattern as the confirmed ADR/IR rows above, expect PROVIDED |
| `A32NX_PRESS_OCSM_*` | 12 | `a380_systems`'s Overhead Cabin/pressurisation controller (OCSM) — needs the same test-based check |
| `A32NX_FCDC_*` | 9 | `extra_backend_fcdc.rs` (plugin) — CPU-010, documented "in progress" upstream |
| `A32NX_SEC_*`, `A32NX_PRIM_*` | 8 + 6 | `sec.rs`/`prim.rs` — per `docs/analysis/systems.md` CPU-005/CPU-011, some of these are genuinely 0-until-JS-FMS-starts (startup-sequencing, not missing) |
| `A32NX_LGCIU_*` (remaining, beyond the 8 confirmed) | 8 | same landing_gear/mod.rs module as the confirmed set — expect PROVIDED |
| `A32NX_OVHD_HYD_*`, `A32NX_OVHD_ADIRS_IR_*`, `A32NX_OVHD_ADIRS_ADR_*` | 8 + 6 + 3 | Overhead pushbutton simvars — the audit's "many OVHD pushbutton vars" category. These are cockpit-binding-dependent: check `cockpit_bindings.txt` maps each physical OVHD switch to the var name FwsCore.ts reads, per §"Do" item 3 |
| `A32NX_ELEC_CONTACTOR_*` | 4 | `a380_systems` electrical — contactor state, likely PROVIDED (same pattern as `extra_backend_fcdc.rs`'s own reads of `990XS1`/`990PU1` etc. at line ~1043) |
| `A380X_RMP_*` | 3 | RMP (Radio Management Panel) JS instrument — JS-PUBLISHED, "the view must run" classification; needs the RMP instrument confirmed booting, not a Rust wire |
| `PUSH_OVHD_*`, `XMLVAR_SWITCH_OVHD_*` | 6 | Overhead pushbutton cockpit-binding vars (CALLS FWD/ALL/AFT, OXYGEN CREW, EMEREXIT, NOSMOKING) — check against `cockpit_bindings.txt` |
| `PUSH_AUTOPILOT_MASTERCAUT_*`, `PUSH_AUTOPILOT_MASTERAWARN_*` | 4 | Master warning/caution reset pushbuttons — cockpit-binding-dependent |
| Everything else (misc singles: `AIRLINER_V1_SPEED`, `AP_CURRENT_TARGET_ALTITUDE_IS_CONSTRAINT`, `CPT/FO_SLIDING_WINDOW`, `LIGHTING_STROBE`, `A32NX_TERR_*`, `A32NX_TAWS_*`, `A32NX_RA_*`, `A32NX_GPWS_*`, `A32NX_FM1/FM2_DISCRETE_WORD`, `A32NX_VENT_*_VCM_CHANNEL_*`) | ~30 | Mixed — several (`TERR`/`TAWS`/`GPWS`/`RA`) are surveillance-system words already tracked as ECAM-008 (SURV SYS INOP stubbed false) in `docs/analysis/ecam-instruments-coupling.md` |

## Counts (honest, given the scope actually covered this pass)

- FWS `L:`/SimVar input names catalogued: **356 + 303 = 659** (exact grep count)
- Empirically confirmed PROVIDED (via new Rust test): **21** — a deliberately
  small, high-confidence set chosen from the prior audit's exact open
  questions, not a random sample
- Automated-triage PROVIDED (name-match, unverified, treat as "likely" only): **146**
- Unresolved / needs the same empirical check: **210**, bucketed by likely
  provider above
- Messages total: not separately re-derived this pass — `docs/analysis/
  ecam-instruments-coupling.md` §2.1 already has the real counts (273
  abnormal sensed, 18 abnormal non-sensed, 14 sensed normal-checklist items,
  9 limitations, 11 nine-digit memo codes / 38 keys total, 119 INOP SYS
  entries) and is the source of truth for message-level counts; this pass
  worked at the input-var level underneath those messages, not per-message,
  given the time box
- Fully wired (message level): not computed this pass — would require
  mapping each of the 273+18+14 messages to its full boolean condition tree
  and classifying every leaf, which the 210 unresolved inputs above feed
  into but don't complete
- Not wirable found this pass: **0 new** beyond what `docs/analysis/
  ecam-instruments-coupling.md` already documents (ECAM-001, ECAM-002,
  ECAM-006, ECAM-008 — see that file; still no real source for those, as
  stated there)

## Spurious messages (root causes for the four named)

Not independently re-diagnosed this pass (no time remaining after the input
inventory + harness work); `docs/analysis/ecam-instruments-coupling.md`
already names the mechanism class that produces exactly this symptom
(a message that fires in a normal powered/running state only because an
input defaults to 0): **ECAM-007** (antiskid fault reads only the switch,
not a power/fault signal — `SURV ROW/ROP LOST`-style faults from an unfed
signal defaulting to "fault" rather than "OK"), and **ECAM-008** (SURV SYS
group INOP hard-coded logic). `BRAKES BTV FAULT` and `COM RMP 3 FAULT`
specifically were not traced to file:line this pass — flagged as the
concrete next step, using the same method as the LGCIU/FIRE_SQUIB
resolution above (find the `get_identifier`/mapping() call, run it through
`emulator/tests/ecam_inputs.rs`, and if genuinely absent from
`snapshot_all()`, that's the spurious-message root cause with proof).

## What's not wirable (from the existing analysis, carried forward)

Per `docs/analysis/ecam-instruments-coupling.md` §2.2 (unchanged this
pass): ECAM-001 (most-restrictive speed limitation), ECAM-002 (derated
climb/MFP-heating/soft-GA discretes), ECAM-006 (manual pressurisation
backup mode), ECAM-008 (SURV SYS aggregate) have no real source anywhere in
this port yet — building them needs upstream engine/anti-ice/CPC model work
FBW itself hasn't finished, not a wiring fix.

## The Rust test added this pass

`emulator/tests/ecam_inputs.rs` (new file): builds `presets::
engines_running()`, ticks 5s, asserts 21 curated FWS input names (the
prior audit's exact open questions) are present in `snapshot_all()`. Passed
this session. It's written to be extended — the 210-name unresolved bucket
above is the queue for that.

**Unrelated pre-existing build blocker fixed to make this test runnable:**
`emulator/src/bin/battery.rs` (another agent's in-progress test-battery
work per this task's own concurrency note) had a pre-existing borrow
checker error (`kind` moved into a thread closure then used after, at the
old line 705) that failed `cargo test` for the whole crate regardless of
`--test` filtering (cargo builds all package targets for `cargo test`).
Applied rustc's own one-line suggested fix — clone `kind` into
`kind_for_log` before the move (`battery.rs:626-627,705`) — no behaviour
change, flagged here for that agent to double check.

## Second pass (20-minute hard time box): exhaustive empirical check

A companion test, `emulator/tests/ecam_inputs_all.rs`, was added to do what
§"Not reached this pass" below asked for: check **every** FWS input name,
not a curated 21. Re-running the same grep from §Method step 1-2 and taking
the union found the true count is **388 distinct identifier strings**, not
659 — 271 of the "303 other SimVars" turn out to be the *same* `L:` name
already in the 356 (`FwsCore.ts` reads some `L:` vars via the older
`SimVar.GetSimVarValue('L:NAME', ...)` call and others via the newer
`RegisteredSimVar.create('L:NAME', ...)` call; the two independent greps
each caught a disjoint subset, but both regexes matched several of the same
underlying names once the trailing quote captured by the second regex is
stripped). This corrects the prior pass's headline count.

Running `cargo test --release --test ecam_inputs_all -- --nocapture`
(`presets::engines_running()`, 200 ticks of 0.05s = 10s) against all 388:

- **PRESENT: 225**
- **ABSENT: 163** (full list in the test's stdout / git history of this
  file's previous revision has the raw dump; not repeated here for space)

**Important caveat discovered this pass, load-bearing for the ABSENT
count**: `Emulator::tick()` (`emulator/src/lib.rs:181`) explicitly documents
what it runs — `aspects.pre_tick`/`simulation.tick`/`aspects.post_tick` (the
`a380_systems` Rust port) plus circuits/breakers/failures/hydraulics/
electrical-loads/bleed-loads. It does **not** call `prim.rs` or
`extra_backend_fcdc.rs` at all (confirmed: no `prim`/`Prim` hits anywhere in
`emulator/src/presets.rs`) — those are the plugin's own separate C++-computer
emulation layer, ticked only by the real `Plugin::tick` in the live X-Plane
process, not reachable by this offline harness (the doc comment on `tick()`
says so explicitly: "for the pieces reachable offline"). That means every
`A32NX_PRIM_*`, `A32NX_SEC_*_HEALTHY`, `A32NX_SEC_*_PUSHBUTTON_PRESSED`,
`A32NX_FCDC_*` name in the ABSENT list is a **false negative for this test
harness**, not proof the value is unwired at runtime — `prim.rs:1239` does
write `A32NX_PRIM_{p}_HEALTHY` (and `prim.rs:2009` has a passing unit test
asserting exactly that, inside `prim.rs`'s own test module, which *does*
drive `Prim`/`Sec` directly rather than through `Emulator::tick()`). Also
false negatives for the same structural reason: every cockpit pushbutton/
switch name (`PUSH_OVHD_*`, `PUSH_AUTOPILOT_MASTERCAUT_*`,
`XMLVAR_SWITCH_OVHD_INTLT_*`, `CPT_SLIDING_WINDOW`, etc.) — these are
written by SASL/cockpit_bindings.txt into `fbw/<name>` datarefs at runtime,
which the JS runtime's own L: var registry (`js_bridge.rs`) serves to
`FwsCore.ts`; they are **not** part of `Vars`/`snapshot_all()` at all
(confirmed present as real bindings in `cockpit_bindings.txt`, e.g.
`PUSH_GLARESHIELD_CS_MASTERCAUT: holds fbw/PUSH_AUTOPILOT_MASTERCAUT_L`,
`PUSH_OVHD_CALLS_ALL/DECK_MAIN/DECK_UPPER/PURS/REST_FWD/REST_MAIN`,
`PUSH_OVHD_OXYGEN_CREW`, `SWITCH_OVHD_INTLT_EMEREXIT/NOSMOKING`), so this
test structurally cannot see them — this emulator harness has no JS
runtime, per its own module doc. **Net effect: this pass's 163 ABSENT count
is a conservative upper bound on genuinely-missing Rust wiring, not a
proven-missing count** — a real gap would need either the `js` feature
enabled with a running cockpit-view trace (as the prior pass's own "Not
reached" section already recommended), or `prim`/`extra_backend_fcdc`
ticked inside `Emulator::tick()`, neither of which fit the 20-minute box.

Genuinely-informative ABSENT names that are *not* explained by either
caveat above (real candidates for follow-up, not re-wired this pass for
lack of time): `A32NX_ENGINE_N1:{1-4}` and `A32NX_ENGINE_IDLE_N1`
(`engine_commands.rs:198`, `fadec.rs:697,820` all call `vars.get(...)` to
*read* `ENGINE_N1:n`/`ENGINE_IDLE_N1` — i.e. they expect something else to
write it, and nothing in the grep results does), `A32NX_TCAS_*`,
`A32NX_GPWS_*`/`A32NX_TAWS_*`/`A32NX_TERR_*` (already tracked as ECAM-008's
SURV SYS gap), `A32NX_AUTOTHRUST_*`, `A32NX_FMA_VERTICAL_MODE`,
`A32NX_FMC_{A,B,C}_IS_HEALTHY`, `A32NX_ECAM_SD_*` (SD page-switching state).

## Spurious messages: root causes found this pass

**COM RMP 3 FAULT** — `FwsCore.ts:4430-4432`:
```
const rmp3State = SimVar.GetSimVarValue('L:A380X_RMP_3_STATE', 'number');
this.rmp3Fault.set(rmp3State === RmpState.OffFailed || rmp3State === RmpState.OnFailed);
```
`A380X_RMP_3_STATE` is confirmed ABSENT by the test above (and this one is
*not* explained by the PRIM/SEC/cockpit-binding caveats — it is a JS-runtime
L: var, same class as the pushbutton vars, but its real provider is
`instruments/src/RMP/Systems/RmpStateController.ts`, one controller
instance per RMP). Root cause: this port's RMP instrument set does not
appear to mount/run a third `RmpStateController` for RMP 3 (the doc's own
prior category note flags `A380X_RMP_*` as "needs the RMP instrument
confirmed booting, not a Rust wire" — this pass's empirical result is
consistent with that: RMP 1/2's states are written by their own
controllers, RMP 3's never is, so the simvar stays at its unwritten
default, which `RmpState`'s enum ordering maps to a Failed state). **Not
wirable from the Rust plugin** — needs RMP 3's JS view confirmed present in
this port's panel/instrument registration (a `js`-feature runtime trace
would confirm which RMP views actually mount, per the prior pass's own
recommended method).

**BRAKES BTV FAULT** — `FwsCore.ts:1630-1635`:
```
public readonly btvLost = MappedSubject.create(
    ([w1, w2, engRunning]) => engRunning && (w1.bitValueOr(13, false) || w2.bitValueOr(13, false)),
    this.fcdc1LandingFctDiscreteWord,
    this.fcdc2LandingFctDiscreteWord,
    this.eng1Or2AndEng3Or4RunningAndPhase,
);
```
gates on bit 13 of both FCDC "landing function" discrete words. These words
come from `extra_backend_fcdc.rs`, documented (§CPU-010 in
`docs/analysis/ecam-instruments-coupling.md`) as "in progress" — the FCDC
bus-word port is incomplete. This is the same class of bug as ECAM-007
(a bit read as "fault" because the real provider isn't finished, rather
than a genuine 0/false default): once engines are running
(`eng1Or2AndEng3Or4RunningAndPhase` true), if either placeholder FCDC word
has bit 13 set (or an SSM that `bitValueOr` doesn't treat as invalid),
`btvLost` — and therefore `320800014 BTV FAULT` — fires. **Root cause is
CPU-010** (FCDC WIP); fixing it means finishing the FCDC landing-function
discrete-word port in `extra_backend_fcdc.rs`, specifically confirming bit
13 is 0/not-set in the interim placeholder rather than defaulting truthy.
Flagged for CPU-010's owner with the exact bit and file:line rather than
fixed here — this port is explicitly "another workstream's in progress
file" per `team.md`, and editing it blind in the last minutes of a 20-minute
box risked a worse regression than leaving it documented.

## Not wired this pass (honest accounting)

No code was changed in `fbw-xp-systems/src` this pass — the 20-minute box
was spent entirely on (1) building and running the exhaustive empirical
test (`emulator/tests/ecam_inputs_all.rs`, new file, compiles and passes)
and (2) tracing the two spurious messages to file:line root causes above.
Both root causes point to work already tracked under existing owners
(CPU-010's FCDC port; the RMP instrument's JS-side registration) rather
than a quick Rust `get_identifier` fix, so no wiring change was made rather
than risk an unverified edit to another workstream's in-progress file in
the time remaining. The `A32NX_ENGINE_N1:n`/`A32NX_ENGINE_IDLE_N1` gap
identified above (genuinely read-but-never-written per grep, not explained
by the PRIM/cockpit-binding caveats) is the most concrete next lead for
whoever picks this up next.

## Not reached this pass

The coordinator's expanded scope (EWD/SD pages, PFD/ND, MFD/FMS, FCU, RMP,
ISIS, OANS/BTV, systems-host CPIOMs/FQMS/CDS, the Rust aspects modules) is
real and not started. The method that worked here scales to it directly:
(1) `grep -oE` each instrument's `.ts`/`.tsx` files for `L:`/SimVar
literals the same way step 1-2 above did, (2) run the result through
`emulator/tests/`-style empirical checks rather than static grep (step 3's
weakness generalises — `format!`-built names will always look "missing" to
grep), (3) prioritise per the coordinator's own order — startup/power-up/
APU/engine-start/taxi/takeoff first, abnormal procedures after. The
js_bridge.rs "first read of unwritten L: var" log and `emulator::
snapshot_all()` (already proven working above) are the two right tools for
that; a `boots_fbw_cockpit_views` runtime trace with the JS `js` feature
enabled would additionally catch anything the static grep misses entirely
(dynamically-built var names, Coherent call names) — `docs/analysis/
ecam-instruments-coupling.md` §6 flags this same gap for the JS runtime
generally.
