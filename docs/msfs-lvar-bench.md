# Measuring what an LVar read or write costs

`msfs/lvar-bench` is a small, throwaway MSFS WASM module that measures the cost
of reading and writing LVars, so that the MSFS port's boundary design rests on
a measured per-operation number instead of a guess.

**It is a sizing measurement, not a gate.** An earlier framing had it deciding
the whole port. That framing rested on a wrong premise and is corrected below.

---

## What actually crosses the boundary — and what does not

The deep systems layer moves 4,228 values per frame. That figure is real, but
it is the **internal cross-area bus**: `PublishedFrame`, the mechanism by which
one deep area reads another a frame behind.

In X-Plane every one of those has to become a variable, because FlyByWire's
JavaScript runs in a separate runtime and the variable system is the only way
to reach it. **In MSFS our module is a single process, and its areas read each
other directly in memory.** None of that traffic needs to be an LVar. The
4,228 figure does not cross the boundary at all.

What crosses the boundary in MSFS is only what something *outside* our module
reads:

- the **~49 authority couplings**, where we drive FlyByWire's own failure
  variables (`docs/deep/authority.md`);
- **live state for our EFB pages** — what is armed, what is open, wear. The
  static catalogue ships in the bundle, so this is only the moving part;
- anything we **deliberately** want their instruments to display.

Hundreds, not thousands. The benchmark is weighted accordingly: the sweep is
50, 100, 250, 500, 1,000, 2,000, with 4,228 kept only as a ceiling check in
case a future design does decide to push the internal bus across after all.
The interesting region is 100–500 and that is where the resolution goes.

This crate is standalone. It is deliberately **not** a member of
`D:\A380\fbw-xp-systems\Cargo.toml` — it targets `wasm32-wasip1` and links against
the proprietary MSFS SDK, and must never be dragged into the X-Plane build. Its
own `Cargo.toml` opens with an empty `[workspace]` table, which stops cargo
walking up the tree, so the isolation is structural rather than a matter of
remembering. Verified: `cargo metadata` at the repository root still reports
exactly one workspace member, `fbw_a380_systems_xp`.

---

## What it measures

1. **Write cost, and its shape across the sweep.** Flat
   nanoseconds-per-operation means the cost is linear and can be extrapolated
   to whatever the export set turns out to be; a climb shows where a ceiling
   sits.
2. **Registered handle versus name lookup each frame.** `NamedVariable::from`
   calls `register_named_variable`, which takes a string and allocates a
   `CString`. The X-Plane layer resolves every identifier once at construction.
   With a few hundred variables this is a straightforward design choice, and
   the benchmark reports its size as a derived "name lookup overhead" line.
3. **Reads, on the same low counts.** The `Truth` side reads from the sim every
   frame, and with a small export set that is proportionally a *larger* share
   of boundary traffic than it was under the old framing. Reads are swept
   through held handles and through per-operation name lookups, exactly like
   writes.
4. **Writing a value that has not changed.** This matters *more* with a small
   export set, not less: if "write only what moved" is cheap, per-frame
   boundary traffic drops to near nothing.

Timings use `std::time::Instant`, a monotonic clock, verified to work inside
MSFS WASM by FlyByWire's own shipping use of it in
`systems_wasm/src/aspects.rs:655`. The module sanity-checks the clock at startup
and refuses to give a verdict if it returns zero-length intervals.

At n = 50 a single pass is only a few microseconds, which could fall at or
below the resolution of a WASM clock and be measured as zero. Every timed block
therefore repeats its inner loop until it has performed at least 20,000
operations. The working set stays n variables wide, so cache behaviour at that
size is still what is measured; only the interval being timed grows. The
changed-write mode offsets its value per repetition, so no repetition
accidentally becomes an unchanged write and contaminates the comparison being
made.

The sweep is spread **one cell per frame** rather than run in a burst, so the
sim stays responsive and the numbers describe steady state. Cells are visited
round-robin, so drift in machine state spreads evenly across every cell rather
than poisoning whichever one it landed on. The full run is 5 modes × 7 counts ×
50 repeats = 1,750 measured frames, about 29 seconds at 60 fps after a 60-frame
warm-up.

---

## Build

```powershell
cd D:\A380\fbw-xp-systems\msfs\lvar-bench
.\build.ps1
```

That is the whole thing. The output is `dist\lvar-bench.wasm`.

**It needs Docker, not the MSFS SDK.** The default route builds inside
`ghcr.io/flybywiresim/dev-env` — the same image, pinned to the same digest,
that FlyByWire use for their own `systems.wasm` (`scripts/dev-env/run.sh`).
That image already contains the SDK at `/workdir/MSFS_SDK`, so nothing has to
be installed or licensed on this machine. The first run pulls the image, which
is several GB; later runs take seconds.

The equivalent single command line, if you would rather not use the script:

```powershell
docker run --rm -v "D:/A380/fbw-xp-systems/msfs/lvar-bench:/external" `
  ghcr.io/flybywiresim/dev-env@sha256:28b1f55c047b9ec338c3d676a82225fe135b0b1061fa7993c03b9a75b5e470cd `
  bash -c "cd /external && mkdir -p dist && cargo build --target wasm32-wasip1 --release && wasm-opt -O1 --signext-lowering --enable-bulk-memory --enable-nontrapping-float-to-int -o dist/lvar-bench.wasm target/wasm32-wasip1/release/lvar_bench.wasm"
```

### Building without Docker

```powershell
.\build.ps1 -Native
```

This needs:

- the **MSFS SDK** installed at `C:\MSFS SDK`, or anywhere with `$env:MSFS_SDK`
  pointing at it. It is a licensed Microsoft download, offered inside MSFS under
  *Options → General → Developers → SDK Installer*, or from
  <https://docs.flightsimulator.com/>. It is not on crates.io, not in this
  repository, and not in `D:\fbw-aircraft`. **It is not installed on this
  machine** — `C:\MSFS SDK` does not exist and `MSFS_SDK` is unset — which is
  why the Docker route is the default.
- **clang and llvm-ar on PATH**, because `msfs-rs`'s build script compiles the
  SDK's `nanovg.cpp` for every wasm32 target, whether the module draws anything
  or not.
- Rust with the `wasm32-wasip1` target. FlyByWire pin 1.96.0
  (`D:\fbw-aircraft\rust-toolchain.toml`); the script passes `+1.96.0` if that
  toolchain is installed.

If the SDK is missing, `build.ps1 -Native` says so before invoking cargo, with
the exact paths it looked in and what to install. (`build.rs` carries the same
check and the same message, but on a machine with no SDK you would hit
`msfs-rs`'s own terser failure first, because dependency build scripts run
before ours — hence the check in the wrapper.)

### Toolchain provenance

Everything here is copied from FlyByWire's working setup rather than invented:
the `wasm32-wasip1` target, the `msfs` git dependency on
`github.com/flybywiresim/msfs-rs`, the `cdylib` crate type, the `lto`/`strip`
release profile, the `wasm-opt -O1 --signext-lowering --enable-bulk-memory
--enable-nontrapping-float-to-int` pass from `package.json`'s
`build-a380x:systems`, and the linker flags from
`D:\fbw-aircraft\.cargo\config.toml`.

One deliberate difference: FlyByWire keep the SDK-dependent linker flags in
`.cargo/config.toml` with the path hardcoded to `/workdir/MSFS_SDK`, which only
works inside their container. `rustflags` cannot interpolate environment
variables, so this crate emits those flags from `build.rs` instead, resolving
the SDK path at build time. Same flags in the same order — including the split
`-l` / `c` and `-L` / *path* pairs — they just find the SDK wherever it is.

---

## Install

```powershell
.\install.ps1
```

It finds the `Community` folder from `UserCfg.opt`, finds
`flybywire-aircraft-a380-842`, and does exactly three things, backing up every
file it touches first:

1. copies `lvar-bench.wasm` into the panel folder next to FlyByWire's
   `systems.wasm`;
2. adds **one** `htmlgauge` line to `panel.cfg`;
3. adds **one** entry to `layout.json`.

To undo it completely:

```powershell
.\install.ps1 -Uninstall
```

which restores both files byte-for-byte from the backups and deletes the
`.wasm`. Re-running install resets from the backup first, so lines never
accumulate. Both paths were tested against a copy of the real package: the
uninstalled files hash identically to the originals.

### What it changes, if you would rather do it by hand

**The `.wasm` goes next to the other four.** On the build currently installed on
this machine (`a380x-v2020.16.0-dev.b77af77`, the `fs2020-master` branch, which
still uses the older flat layout):

```
D:\Microsoft Flight Simulator 2020\Microsoft Flight Simulator 2020 Packages\Community\
  flybywire-aircraft-a380-842\SimObjects\AirPlanes\FlyByWire_A380_842\panel\lvar-bench.wasm
```

On a current source-tree build, which uses the modular SimObject layout
described in section 0a of `docs/msfs-port.md`, the same folder is:

```
  flybywire-aircraft-a380-842\SimObjects\AirPlanes\FlyByWire_A380X\
    attachments\flybywire\Part_Interior_Cockpit\panel\lvar-bench.wasm
```

The install script does not care which: it puts the module wherever
`systems.wasm` already is.

**The `panel.cfg` line.** Section 0a lists the four existing gauges in
`[VCockpit21]`:

```ini
htmlgauge00=WasmInstrument/WasmInstrument.html?wasm_module=systems.wasm&wasm_gauge=systems, 0,0,1,1
htmlgauge01=WasmInstrument/WasmInstrument.html?wasm_module=fbw.wasm&wasm_gauge=fbw, 0,0,1,1
htmlgauge02=WasmInstrument/WasmInstrument.html?wasm_module=fadec-a380x.wasm&wasm_gauge=Gauge_Fadec,0,0,1,1
htmlgauge03=WasmInstrument/WasmInstrument.html?wasm_module=extra-backend-a380x.wasm&wasm_gauge=Gauge_Extra_Backend,0,0,1,1
```

The benchmark is the fifth, appended to that same section:

```ini
htmlgauge04=WasmInstrument/WasmInstrument.html?wasm_module=lvar-bench.wasm&wasm_gauge=lvarbench, 0,0,1,1
```

`wasm_gauge=lvarbench` must match the module's exported entry point.
`#[msfs::gauge(name = lvarbench)]` in `src/lib.rs` exports
`lvarbench_gauge_callback`, confirmed present in the built artifact.

**The `layout.json` entry is not optional.** MSFS will not load a file that is
not listed in the package's `layout.json`, and a missing entry fails silently —
no console message, no gauge, nothing. The entry matches the others in shape:

```json
    {
      "path": "SimObjects/AirPlanes/FlyByWire_A380_842/panel/lvar-bench.wasm",
      "size": 110462,
      "date": 134343835709127437
    },
```

`size` is the file's byte count and `date` is its modification time as a Windows
`FILETIME` (100 ns ticks since 1601). The install script computes both. If you
edit it by hand, re-check the size after every rebuild.

### A note on additive integration

Section 0a's claim that a fifth module can be registered "without editing a file
of FlyByWire's" rests on MSFS's modular SimObject system: attachments are
auto-discovered and merged, and the preset's own `panel.cfg` is nothing but
`[MODULAR_MERGE] auto = true`. That is the right shape for the real module.

This benchmark does not bother. It is a throwaway that appends two lines and
removes them again, and it is worth more to measure inside the real package —
contending for the frame with all four of their modules — than to be
architecturally pure about a tool that gets deleted next week.

---

## Run it, and read the result

1. Start MSFS and load the A380X. Any airport, cold and dark at a gate is fine;
   the benchmark does not care what the aircraft is doing.
2. Turn on *Options → General → Developers → Developer Mode* and open the dev
   toolbar's **Console** window. Filter on `LVARBENCH`.
3. Wait about a minute from the moment the aircraft finishes loading — longer
   than the nominal 29 seconds, because the sweep itself slows the sim down
   (see below). It prints progress at 20% intervals, then prints the block.

The whole block is **26 lines and re-prints every 60 seconds, for as long as
the module is loaded**, so a run is captured in one screenshot and a user who
looked away never has to reload the aircraft. The interval is wall-clock, not
frame-counted, precisely because the frame rate is not constant during a sweep.

```
================================ LVARBENCH RESULTS ================================
 frames 1750   sim 30.2 s   wall .... s   avg .... fps   clock ok=true zero-blocks=0
 registration 4228 vars in ..... ms = ... ns each   inner reps: >=8000 ops per timed block
----------------------------------------------------------------------------------
          |   write hdl  |  write name  |   read hdl   |  read name   |  write same
    count | ns/op    ms  | ns/op    ms  | ns/op    ms  | ns/op    ms  | ns/op    ms
       50 |   ...  ..... |   ...  ..... |   ...  ..... |   ...  ..... |   ...  .....
      100 |   ...  ..... |   ...  ..... |   ...  ..... |   ...  ..... |   ...  .....
      250 |   ...  ..... |   ...  ..... |   ...  ..... |   ...  ..... |   ...  .....
      500 |   ...  ..... |   ...  ..... |   ...  ..... |   ...  ..... |   ...  .....
     1000 |   ...  ..... |   ...  ..... |   ...  ..... |   ...  ..... |   ...  .....
     2000 |   ...  ..... |   ...  ..... |   ...  ..... |   ...  ..... |   ...  .....
     4228 |   ...  ..... |   ...  ..... |   ...  ..... |   ...  ..... |   ...  .....
----------------------------------------------------------------------------------
 HEADLINE n=250 (representative export set: ~49 authority couplings + EFB live state)
   write ... ns   read ... ns   name-lookup overhead ... ns   unchanged-write saves ...%
   fits in 1 ms: ..... writes / ..... reads     read+write at n=250: ...... ms/frame
   at n=500: ...... ms/frame     ceiling n=4228 (internal bus, does NOT cross): ...... ms
   linearity 4228 vs 50: ....x writes, ....x reads (linear)   unchanged: ...
----------------------------------------------------------------------------------
 VERDICT n: ...
----------------------------------------------------------------------------------
 FILESYSTEM PROBE -- does MSFS WASM have one?
   std::fs  \work\lvarbench-results.txt        <result or exact error>
   ... one line per path, std::fs then C fopen ...
   => YES / NO
=========================== END LVARBENCH RESULTS =================================
```

The numbers above are placeholders, but the **first real run confirmed the
module works**: it loaded, the monotonic clock passed its check (24,300 ns over
a 200k-iteration spin), and it registered 4,228 variables in 1.093 ms —
**258 ns per `register_named_variable`** — before stepping through the sweep
with no errors or warnings. That run's per-operation table was lost when the
sim was closed, which is why the block now re-prints and tries to write itself
to disk.

### The sim slows down during the sweep — that is itself a result

The first run decelerated badly: 350 frames in the first 16 s (~22 fps), then
350 more in the next 50 s (**~7 fps**). The high counts visibly hurt.

Two things are going on and they should not be confused.

- **An artifact of the benchmark.** Each timed block repeats its inner loop to
  a minimum operation count so that small working sets stay above clock
  resolution. That minimum was 20,000 on the first run, so the n=4,228 cells
  performed about 21,000 operations *in a single frame* — roughly five times
  the nominal count. It is now 8,000, which roughly halves the worst frame. The
  clock has ample resolution, so nothing is lost by the change.
- **A real signal.** Even allowing for that amplification, thousands of LVar
  operations in one frame cost visible frame time. That corroborates the design
  conclusion from the other direction: a few hundred is comfortable, thousands
  are not, and keeping the internal cross-area bus *inside* the module rather
  than publishing it is the right call. Had the original 4,228 premise held,
  this deceleration is what it would have felt like every frame, forever.

The block now reports `wall` seconds and `avg fps` next to the sim time, so the
deceleration shows up in the output rather than only in the sim.

### Does the WASM sandbox have a filesystem?

MSFS keeps no console log on disk, so until now a screenshot was the only way
to capture a run. The module therefore tries to write its results to a file and
**reports the outcome either way** — the exact path and the exact error.

This settles a question that matters well beyond the benchmark.
`docs/msfs-port.md` §6 records that no filesystem API is surfaced anywhere in
*msfs-rs*, and concludes that persistence must go through `NXDataStore`. That
finding is correct but narrower than the conclusion drawn from it: "the
bindings expose no filesystem" is not the same claim as "the host provides
none". The crate targets `wasm32-wasip1`, and WASI *does* have a filesystem
interface; whether MSFS preopens a directory for it is a separate question, and
the MSFS SDK's own `\work\` convention is a third.

Six paths are tried, first through Rust's `std::fs` (raw WASI syscalls) and
then through the MSFS libc's `fopen` (which is where a `\work\` convention
would be implemented, and which `std::fs` would not see):

```
\work\lvarbench-results.txt      /work/lvarbench-results.txt
work/lvarbench-results.txt       lvarbench-results.txt
./lvarbench-results.txt          /lvarbench-results.txt
```

`\work\` is first because it is the SDK's documented convention for WASM module
persistence, mapping to `LocalState\packages\<package>\work\`.

**If any path succeeds**, the results block is written there and refreshed on
every reprint — and `docs/msfs-port.md` §6 needs correcting, because persistence
would not have to go through `NXDataStore` at all. That is a materially better
answer for the deep layer's catalogue and wear state than a key-value store.
Say so loudly and update §6.

**If every path fails**, that is equally worth having: a definitive "MSFS WASM
cannot write files, here is the error for each path tried" closes the question
and confirms §6's conclusion on firmer ground than the absence of a binding.

The probe runs **after** the results are printed, deliberately. An
unimplemented WASI call traps rather than returning an error, and a WASM trap
is not a Rust panic — it cannot be caught, and it takes the gauge down. Running
the probe last means the worst case still leaves the full results block on
screen, and each attempt announces itself before it is made, so even a trap
says exactly which call caused it.

#### The fallback build, if the module stops loading

Probing the filesystem adds eight WASI imports to the module: `path_open`,
`fd_prestat_get`, `fd_prestat_dir_name`, `fd_close`, `fd_read`, `fd_seek`,
`fd_fdstat_get`, `fd_fdstat_set_flags`. If MSFS's runtime does not implement
them, the module could fail to *instantiate* — losing the whole run, not just
the probe.

`build.ps1` therefore produces two artifacts. `dist\lvar-bench-nofs.wasm`
compiles the probe out, and its imports are exactly the set the first
successful run already proved MSFS provides: `clock_time_get`, `fd_write`,
`environ_get`, `environ_sizes_get`, `proc_exit`, `sched_yield`, `commit_pages`.
It cannot be worse than the build that already worked.

```powershell
.\install.ps1 -NoFsProbe
```

installs it under the same filename, so `panel.cfg` does not change. Use it
**only** if the default build fails to load at all — a missing
`LVARBENCH: module loaded.` line is the symptom.

### Reading it without the console

Every headline number is also published as an LVar, readable from the dev
toolbar's **Behaviors** window or any variable watch — search `LVARBENCH`:

| LVar | Meaning |
|---|---|
| `LVARBENCH_DONE` | 1 when the run has finished. Read nothing else until this is 1. |
| `LVARBENCH_PROGRESS` | 0 → 1. |
| `LVARBENCH_VERDICT` | 0 invalid, 1 / 2 / 3 — see below. |
| **`LVARBENCH_WRITES_PER_MS`** | **The number that generalises.** How many LVar writes fit in one millisecond. Divide by it to size any export set. |
| **`LVARBENCH_READS_PER_MS`** | The same for reads. |
| `LVARBENCH_NS_PER_WRITE` | Nanoseconds per write at n = 250, through a handle held since construction. |
| `LVARBENCH_NS_PER_READ` | Nanoseconds per read at n = 250, through a held handle. |
| `LVARBENCH_NS_PER_WRITE_LOOKUP` | Per write when the name is resolved every operation. |
| `LVARBENCH_NS_PER_READ_LOOKUP` | Per read, likewise. |
| `LVARBENCH_NS_PER_LOOKUP` | The derived difference: what one name lookup costs. |
| `LVARBENCH_NS_PER_WRITE_UNCHANGED` | Per write of a value the variable already holds. |
| `LVARBENCH_NS_PER_REGISTER` | Per `register_named_variable` at startup. |
| `LVARBENCH_MS_PER_FRAME_250` | Writing 250 values, in milliseconds. |
| `LVARBENCH_MS_PER_FRAME_500` | Writing 500 values. |
| `LVARBENCH_MS_PER_FRAME_READ_250` | Reading 250 values. |
| `LVARBENCH_MS_PER_FRAME_4228` | The ceiling check — the internal bus size, which does *not* cross the boundary. |
| `LVARBENCH_LINEARITY_X100` | ns/op at 4,228 ÷ ns/op at 50, ×100. 100 means perfectly linear. |
| `LVARBENCH_CLOCK_OK` | 1 if the monotonic clock passed its startup check. |
| `LVARBENCH_ZERO_INTERVAL_CELLS` | Should be 0. Anything else means some timed blocks fell below clock resolution. |
| `LVARBENCH_FRAMES` | Measured frames; 1750 on a complete run. |
| `LVARBENCH_FS_OK` | 1 if some path accepted a file write — see the filesystem section. |

`LVARBENCH_X00000` … `LVARBENCH_X04227` are the variables the sweep hammers.
Ignore them.

### If nothing appears

- Check the console, unfiltered, for a line starting `LVARBENCH PANIC:`. The
  module installs a panic hook that prints before the WASM instance aborts.
- No `LVARBENCH: module loaded.` line at all usually means the `layout.json`
  entry is missing, or its `size` is stale after a rebuild — **or** that the
  filesystem probe's extra WASI imports stopped the module instantiating, in
  which case install the fallback with `.\install.ps1 -NoFsProbe`.
- Console output that stops mid-probe, right after a `trying ...` line, means
  that call trapped. That line names the path and the API, and is the answer.
- `LVARBENCH_CLOCK_OK = 0`, or verdict 0, means `std::time::Instant` did not
  work in this build of the sim. Report that, not the timings — they are
  worthless. Nothing else in the module depends on the clock, so the frame
  counter and `sim time spanned` still prove the run happened.

---

## How to interpret it

The frame budget at 60 fps is **16.67 ms**, and MSFS runs gauge callbacks on the
main thread, so all five WASM modules spend the same budget. A defensible
allocation for "everything our module pushes across the boundary and pulls back"
is about **0.5 ms — 3% of a frame**, and the comfortable target is a fifth of
that.

The verdict is judged on **reads plus writes at n = 250**, a representative
export set: ~49 authority couplings plus EFB live state.

### Verdict 1 — under 0.10 ms/frame for 250 reads + 250 writes

**The boundary is effectively free.** Export everything the outside needs, every
frame, through handles resolved once at module construction — the same
discipline the X-Plane layer already follows. No change-detection machinery, no
staging, no rate tiers. Note the figure in section 4 of `docs/msfs-port.md` and
move on.

For scale: 0.10 ms across 500 operations is about 200 ns each, so verdict 1
covers anything up to roughly 5,000 operations per millisecond.

### Verdict 2 — 0.10 to 0.50 ms/frame

**Affordable, but not free. Write only what moved.** With an export set of a few
hundred, most values are static frame to frame, so a shadow-array comparison
typically removes 80–95% of the writes for a handful of instructions each.

**Read the `unchanged-write saving` line before deciding how to do it.** If
writing an unchanged value costs the same as writing a changed one — and it
probably does, since MSFS has no reason to compare — then the saving comes
entirely from the writes you *skip*, and the comparison must happen on our side.
That is still a win; it is simply our win to implement rather than a free one.
If instead MSFS is visibly cheaper for unchanged values, the naive
publish-everything loop is already most of the way there and the shadow array
buys less than it looks.

### Verdict 3 — over 0.50 ms/frame

**Even a few hundred costs real frame time.** At that price the boundary itself
needs narrowing, not optimising: cut what crosses to the ~49 authority couplings
and a minimal EFB set, and make the rest pull-on-demand — written only when an
EFB page is actually open — rather than published every frame. This would be a
surprising result and worth double-checking against the per-count table before
acting on it; check in particular that `LVARBENCH_ZERO_INTERVAL_CELLS` is 0 and
that the low counts are not just noise.

### The other numbers, and what each decides

- **`LVARBENCH_NS_PER_LOOKUP`** — what resolving a name each frame costs versus
  holding a handle. If it is a large fraction of the write cost, then resolving
  identifiers once at construction becomes a stated design requirement for the
  MSFS glue, exactly as it is in X-Plane. If it is negligible, we have one less
  constraint on how that glue is written.
- **The read row** — with a small export set, reads from the sim are
  proportionally a larger share of boundary traffic than writes. If reads cost
  as much as writes, the input side deserves the same scrutiny as the output
  side, and SimConnect data definitions — which fetch many simvars in one call —
  become worth investigating as an alternative to per-variable reads. Note that
  the `Truth` side reads mostly *simvars* (`AircraftVariable`), not LVars; this
  benchmark times the LVar read path, which is the right proxy for reading
  FlyByWire's own state but not for reading the sim's.
- **`LVARBENCH_LINEARITY_X100`** — near 100 means the cost is linear and the
  numbers extrapolate if the export set grows. Much above 150 means there is a
  ceiling inside the sweep; read the per-count table to find where the per-op
  cost starts climbing and treat that count as a cap.
- **The 4,228 ceiling check** — only relevant if some future design decides to
  push the internal cross-area bus across the boundary after all. Under the
  current design it does not, and this row is there to show what that decision
  would cost rather than to influence anything today.

---

## What this does not measure

Stated so the number is not over-read:

- **The consumer side.** This times our call into MSFS. It does not time what
  MSFS, the ModelBehaviors XML, or FlyByWire's JavaScript gauges do in response
  to a changed LVar. "Write only what moved" may pay for itself there even if
  our own write cost is flat — a separate measurement this module cannot make.
- **Contention from our own physics.** The benchmark does nothing but touch
  LVars. The real module will also run the deep systems model in the same frame.
- **The simvar read path.** `AircraftVariable::get` is a different call from
  `NamedVariable::get_value`; the `Truth` side uses both.
- **MSFS 2024.** The numbers come from whichever sim you install it into. The
  package currently installed on this machine is the FS2020 build
  (`a380x-v2020.16.0-dev.b77af77`). If the port targets MSFS 2024, run it there
  too — the WASM runtime is not the same one.

Run it once, record the block, uninstall it, and put the per-operation figures
into section 4 of `docs/msfs-port.md` alongside the corrected note that the
4,228 are the internal bus and do not cross the boundary in MSFS.
