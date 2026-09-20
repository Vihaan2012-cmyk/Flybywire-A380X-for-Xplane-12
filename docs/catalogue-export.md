# Static catalogue export

`src/study/catalogue.rs` turns the entirely static half of this crate's
failure catalogue into one JSON document, so an MSFS EFB bundle can ship it
as a file instead of crossing the LVar boundary for it at runtime
(`docs/msfs-port.md` section 0a: the live half -- which failures are armed
and at what magnitude, which breakers are open, accumulated wear -- is
hundreds of values; this static half is thousands, and none of it changes
while the sim runs).

It is the sibling of `src/study/web.rs`'s `/study/components` and
`/study/breakers` endpoints, which already serve a mix of this same static
data plus live values read from the running simulation's snapshot. This
module deliberately does not change either endpoint's JSON shape (both are
covered by their own tests, and potentially by an existing HTTP client) --
it reuses the same underlying accessor functions those endpoints read from
(`deep::registry()`, `failures::all_ids()`, `study::failures::{ata_of,
chapter}`, `deep::breakers::catalog`, `mel_catalog::MEL_FAILURES`), so the
static catalogue and the live Study endpoints can never describe two
different universes of failures or components, but the result is an
additive document, not a patch to either endpoint.

## How to regenerate

```
CATALOGUE_OUT=/path/to/catalogue.json CARGO_TARGET_DIR=D:/fbw-xp-systems/target-p4 \
  cargo test --lib -- --ignored --exact study::catalogue::tests::dump_catalogue
```

This runs entirely offline: no X-Plane process, no plugin load, no
`Truth`. It mirrors `src/study/web.rs`'s own `dump_pages` test (same
env-var-gated, `#[ignore]`d pattern, already this crate's convention for
producing a JSON dump outside X-Plane) rather than introducing a new
binary or example target. The test reads the real clock once and calls the
module's pure `catalogue_json(generated_at_unix_seconds: u64) -> String`;
every other test in the same file calls that pure function directly with a
fixed timestamp, so they need no clock and no file I/O at all.

## Determinism

`catalogue_json` is a pure function of its one argument. `serde_json`'s
`Map` in this crate is backed by a `BTreeMap` (the `preserve_order` feature
is not enabled anywhere in this dependency tree), so every JSON object's
keys are already emitted in a fixed, sorted order regardless of the order
the code inserts them in. Every array is explicitly sorted by a stable key
before being handed to `json!` (ids, keys, or references, as noted per
section below) rather than relying on registration order. Together this
means two calls with the same timestamp are byte-identical --
`tests::catalogue_json_is_byte_identical_for_the_same_input` asserts it
directly, and `tests::every_collection_is_sorted_by_its_stable_key` asserts
the sorting that guarantee depends on.

The generation timestamp is a parameter rather than something the pure
function reads off the clock itself, specifically so this determinism
property holds without the test needing to control wall-clock time. The
real build step (`dump_catalogue`) is the only place that reads
`SystemTime::now()`.

## Schema

Top-level object:

| Key | Type | Meaning |
|---|---|---|
| `schemaVersion` | integer | Bump when a field's *meaning* changes, not when a count changes -- counts are expected to grow as the `deep` areas do. Currently `1`. |
| `generatedAtUnixSeconds` | integer | Unix epoch seconds at generation time (no date library in this crate's dependencies; an integer is unambiguous without one). |
| `chapters` | array | Every ATA chapter number actually referenced by `failures`/`components`/`alerts`/`breakers` below, sorted by number -- not a hand-kept list of its own, so it cannot silently drift from what the other sections reference. |
| `failures` | array | Every failure id `failures::all_ids()` knows (FlyByWire's own catalogue, the flight control computers', this project's "extra" catalogue, and the `deep` areas'), sorted and deduplicated by id. |
| `components` | array | Every `deep::registry()` component (see "What this leaves out" below), sorted by id. |
| `alerts` | array | Every `deep::registry()` ECAM alert, sorted by key. |
| `breakers` | array | The 399-entry `deep::breakers::catalog` ELMS set, sorted by id (see "What this leaves out"). |
| `melEntries` | array | `mel_catalog::MEL_FAILURES`, the MEL-reference-to-failure-id cross-reference table, sorted by reference (see "What this leaves out"). |

### `chapters[]`

| Key | Type |
|---|---|
| `number` | integer (ATA chapter number) |
| `name` | string, e.g. `"24 Electrical Power"` |

### `failures[]`

| Key | Type | Notes |
|---|---|---|
| `id` | integer | |
| `name` | string | |
| `ataChapterNumber` | integer | |
| `ataChapterName` | string | |
| `component` | string | The component the failure acts on, as `failures::affected_components` names it -- for a `deep` failure this matches a `components[].id` exactly; for the legacy/extra catalogues it is a human-readable name with no guaranteed matching entry in `components[]` (those catalogues predate the `deep` component registry). |
| `cause` | string | `failures::cause_description(id)`. |
| `magnitudeSemantics` | string or `null` | What the failure's 0..1 magnitude means physically (e.g. `"leak orifice area, 0..20 mm2"`), present **only** for the `deep` catalogue's failures, which are the only ones that state it. The legacy/extra catalogues' magnitude is a plain 0..1 "how failed" fraction with nothing further to add; this is `null` for them rather than a fabricated `"fraction, 0..1"` placeholder. |

### `components[]`

Only `deep::registry().components` -- see "What this leaves out".

| Key | Type | Notes |
|---|---|---|
| `id` | string | e.g. `"29_hyd.green_edp_1"`. |
| `name` | string | |
| `ataChapterNumber` | integer | |
| `ataChapterName` | string | |
| `instance` | integer or `null` | The id's trailing instance number, recognised only when the id's final `_`/`-`/`.`-separated token is a plain integer (`"49_apu.generator_1"` -> `1`). `ComponentDef` carries no explicit instance field, and areas encode "which one" inconsistently: a trailing number, a colour word (`"...green_accumulator"`), or a letter-suffixed number (`"...green_edp_1a"`). Rather than guess at the inconsistent cases this recognises only the unambiguous one and leaves the rest `null` -- see "Found while walking the registry" below. |
| `parameters` | array of `{name, meaning, healthyValue}` | The component's own health parameters (`ParamDef`), already merged with any other area's `extend_component` contribution (`deep::registry()` calls `resolve()` before this module reads it). |
| `failures` | array of integers | Failure ids that act on this component, sorted. |

### `alerts[]`

| Key | Type | Notes |
|---|---|---|
| `key` | string | Unique ECAM alert key. |
| `ataChapterNumber` / `ataChapterName` | integer / string | |
| `title` | string | As shown on the ECAM. |
| `level` | `"warning"` \| `"caution"` \| `"advisory"` \| `"memo"` | |
| `aural` | `"continuousRepetitiveChime"` \| `"singleChime"` \| `"cavalryCharge"` \| `"named:<sound>"` \| `"none"` | |
| `masterLight` | `"warning"` \| `"caution"` \| `"none"` | |
| `confirmSeconds` | number | How long the trigger must hold before the alert shows. |
| `inhibitedInPhases` | array of strings | Flight phases the alert is inhibited in. |
| `statusPageLines` | array of strings | |
| `inoperativeSystemsListEntries` | array of strings | The alert's INOP SYS entries. |
| `failures` | array of integers | Failure ids that can raise this alert, sorted (includes other areas' `contribute`d failures -- `resolve()` has already unioned them in). |
| `procedureLineCount` | integer | **Not** the procedure text itself -- see "What this leaves out". |

### `breakers[]`

Only `deep::breakers::catalog::all()` (399 entries) -- see "What this leaves out".

| Key | Type | Notes |
|---|---|---|
| `id` | string | |
| `name` | string | |
| `ataChapterNumber` / `ataChapterName` | integer / string | |
| `bus` | string | e.g. `"AC1"`, `"DC_ESS"` -- standard aviation/electrical abbreviations, not internal shorthand, kept as `Bus::label()` already renders them. |
| `ratingAmperes` | number | |
| `kind` | `"thermal"` \| `"solidStatePowerController"` | |
| `consumer` | string | |
| `basis` | string | How the rating was derived. |
| `panel` | string | One of `overheadForward`, `overheadAft`, `avionicsBay`, `primaryPowerCentre1`..`4`, `secondaryPowerCentreForward`, `secondaryPowerCentreAft`. |
| `row` / `column` | integer | 1-based position within `panel`'s own grid. |
| `label` | string | The breaker's own cap legend text (<=14 characters). |
| `protectsModelledLoad` | boolean | `false` for a real, named A380 breaker this codebase has no load model for yet -- an honest gap, not hidden. |

### `melEntries[]`

| Key | Type | Notes |
|---|---|---|
| `melReference` | string | e.g. `"24-21-01"`. |
| `ataChapterNumber` / `ataChapterName` | integer / string, or `null` | Parsed from the reference's own first two digits; `null` if that doesn't parse as a number. |
| `failures` | array of integers | The failure ids (one per unit of redundant equipment) this MEL item covers, sorted. |

## What this leaves out, and why

- **The operator's own MEL text** (`mel_catalog::catalog()`/`Item`/`SubItem`
  -- titles, placards, ops procedures, repair-interval categories) is not
  exported. It is the user's own copyrighted PDF, parsed by their own copy
  of `tools/parse_mel.py` onto their own disk at
  `<X-Plane>/Output/preferences/fbw_a380x_mel.json`; nothing is bundled
  today, and without that file `mel_catalog::catalog()` returns `None`.
  What is genuinely ours to ship is the cross-reference table
  (`MEL_FAILURES`) saying which of *our* failure ids a given MEL reference
  covers -- that is `melEntries[]` above.
- **`crate::components`'s runtime registry** (the one `web.rs`'s
  `components_json` reads for its non-`deep` rows) is not exported. It is
  populated by physics models registering their own parameters as they are
  *constructed*, which only happens while building a `Truth` -- there is
  nothing to enumerate without one. Only `deep::registry().components`
  (built once, in code, independent of any running simulation) qualifies
  for a sim-free build step.
- **The legacy gating-breaker catalogue** (`crate::breakers::catalog()`,
  `web.rs`'s `"source":"catalogue"` rows: absorbed `systems.cfg` circuits
  and FlyByWire power-path gates) is not exported. It carries no panel
  position at all -- `"panel"`/`"row"`/`"column"` do not exist on that
  type -- because it is really about *how* a breaker gates a live power
  path (a plugin variable, an absorbed circuit, a `FailureType`), which is
  a live-`Truth` question, not a catalogue one. Only
  `deep::breakers::catalog`'s 399-entry ELMS set has a real panel/row/
  column position, which is exactly what the brief asked this export to
  carry.
- **ECAM procedure lines' own conditions** (`EcamAlert::procedure`'s
  `applies_if`/`done_when`, `deep::api::Cond`) are not exported, only their
  count. Each condition is evaluated against a *live* variable (an ADIRU
  reading, a switch position) -- unlike everything else in this document,
  a procedure line's completion state is not itself static, so flattening
  it to text without its condition would be misleading, and serialising
  the condition tree is a live-evaluation feature, not a catalogue one.

## Found while walking the registry

- **`ComponentDef` has no explicit instance field.** Multi-instance
  components (four APU generators, two hydraulic pumps per colour, four
  engines' worth of harnesses) are disambiguated only by however the
  registering area chose to spell the id: a trailing plain number
  (`"...generator_1"`), a colour word (`"...green_accumulator"`), or a
  number-plus-letter (`"...green_edp_1a"`, half `a`/`b` of a shared
  pump housing). This export's `instance` field only recognises the first
  form; the other two are real ids with no fabricated instance rather than
  a guessed one. Worth a real `instance: Option<InstanceKey>` field on
  `ComponentDef` itself if the EFB ever wants to group by instance
  reliably.
- **`mel_catalog::MEL_FAILURES` has an ATA-chapter mismatch between its own
  comment and its own reference.** `src/mel_catalog.rs:174`:
  ```rust
  // 30 Ice and rain: the ADIRUs' probe heating.
  ("34-11-05", &[30_000, 30_001, 30_002]),
  ```
  The comment and the covered failure ids (`30_0xx`, this crate's own ATA
  30 range) both say ATA 30 Ice and Rain Protection; the MEL reference
  itself is filed under ATA 34 (Navigation). A cross-check of every entry
  against the ATA range of the failure ids it covers found exactly one
  other mismatch, `("49-20-01", &[28_110])` (APU chapter reference
  covering a fuel-chapter failure id) -- plausibly intentional, since an
  APU MMEL item can legitimately concern its own fuel feed, but worth the
  same second look. `melEntries[].ataChapterNumber` is parsed from the
  reference's own first two digits as-is (not silently "corrected" against
  the failure ids it covers), so this mismatch is visible in the exported
  data rather than papered over.
- **The `deep` failure/component/alert counts are moving targets.** At the
  time this was written, `deep::registry()` reported 5,189 failures, 2,010
  components and 304 ECAM alerts -- already past the 5,166/~1,994/301
  figures this brief quoted, because `src/deep/electrical` is under active
  development by another workstream concurrently with this one. This
  module's own tests assert counts against the live registry functions
  themselves (`crate::failures::all_ids().len()`, etc.), not hard-coded
  numbers, for exactly this reason; see the measured sizes below for
  whatever the count was at the moment this file was last regenerated.

## Measured size

Generated 2026-09-20 with the command above:

| | Count |
|---|---|
| `failures` | 5,549 |
| `components` | 2,010 |
| `alerts` | 304 |
| `breakers` | 399 |
| `melEntries` | 64 |
| `chapters` | 35 |

| | Bytes |
|---|---|
| Raw JSON | 4,070,866 (~3.9 MiB) |
| Gzip (`-9`) | 197,531 (~193 KiB) |

~20.6x compression. 193 KiB gzipped is comfortably embeddable in an EFB
bundle; 3.9 MiB raw is still fine to ship uncompressed if the bundling
pipeline doesn't gzip, but gzipping (or serving it pre-compressed) is
clearly worth doing if the bundle has any size budget at all.

The failure count above (5,549) is already well past both this brief's
5,166 figure and the 5,189 this module's own tests measured earlier in the
same session -- `src/deep/electrical` grew again in between. Re-run the
command above for a current number; do not treat any count in this
document as a target to match.
