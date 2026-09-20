# Study overlay for the MSFS A380X EFB

Six pages of what the systems model actually contains, drawn over
FlyByWire's EFB: failures, components, ECAM alerts, circuit breakers, MEL
cross-references, and a per-chapter depth summary.

## Why it is an overlay and not a page in their EFB

Their Failures page is React compiled into `efb.js`. Adding a page to it
means rebuilding that bundle, and the rebuild has to come from **the same
source commit as the installed aircraft** -- a bundle built from a different
commit fails to instantiate and the EFB shows "instrument didn't load".
That was tried and it broke exactly that way.

So this does not touch `efb.js` at all:

| file | what happens to it |
| --- | --- |
| `efb.js` | untouched, byte-identical to theirs |
| `efb.html` | one `import-script` line appended |
| `study.js` | ours, new |
| `catalogue.json` | ours, new |

A FlyByWire update replaces `efb.js` and nothing here breaks. That is the
whole reason for the shape: the installer ships our work, not their
aircraft.

The integrated React version exists too -- `fbw-common/src/systems/
instruments/src/EFB/Study/` on the `deep-systems` branch of the aircraft
repo, typechecking clean -- and becomes usable if the matching source
commit is ever available to build against.

## Data

Everything comes from `catalogue.json`, generated from the registry:

```
CATALOGUE_OUT=msfs/efb-study/catalogue.json \
  cargo test --lib -- --ignored --exact study::catalogue::tests::dump_catalogue
```

3.8 MB, ~193 KB gzipped. It is static -- none of it changes while the
aircraft runs -- so it ships as a file and never crosses the simulator
variable boundary. See `docs/catalogue-export.md`.

**Live state is deliberately absent.** Whether a breaker is open or a
failure armed comes from the deep model, which does not run in MSFS yet.
The pages say so rather than showing a plausible value.

## Install

```sh
./install.sh                      # default package path
./install.sh /path/to/package     # explicit
./install.sh /path/to/package --remove
```

`layout.json` must also list both new files or MSFS will not serve them.
