//! What does an LVar read or write cost in Microsoft Flight Simulator?
//!
//! **This is a sizing measurement, not a gate.** An earlier framing had it
//! deciding the whole MSFS port, on the premise that the deep systems layer
//! must publish 4,228 variables per frame. That premise was wrong. The 4,228
//! are the *internal cross-area bus* -- `PublishedFrame`, how one deep area
//! reads another a frame behind. In X-Plane every one of them has to become a
//! variable, because FlyByWire's JavaScript runs in a separate runtime and the
//! variable system is the only way to reach it. In MSFS our module is a single
//! process and its areas read each other directly in memory. None of that
//! traffic crosses the boundary.
//!
//! What actually crosses it is only what something *outside* our module reads:
//! the ~49 authority couplings that drive FlyByWire's own failure variables,
//! live state for our EFB pages (the static catalogue ships in the bundle, so
//! this is what is armed, what is open, and wear), and anything we deliberately
//! want their instruments to display. That is **hundreds, not thousands**.
//!
//! So the sweep is weighted to the low end -- 50, 100, 250, 500, 1,000, 2,000 --
//! with 4,228 kept only as a ceiling check, in case a future design does decide
//! to push the internal bus across the boundary after all. The interesting
//! region is 100 to 500 and that is where the resolution goes.
//!
//! It measures four things:
//!
//! 1. **Write cost, and its shape across the sweep.** Flat
//!    nanoseconds-per-operation means the cost is linear and predictable; a
//!    climb shows where the ceiling is.
//! 2. **Registered handle versus name lookup each frame.**
//!    `NamedVariable::from` calls `register_named_variable`, which takes a
//!    string and allocates a `CString`. The X-Plane layer resolves every
//!    identifier once at construction. With a few hundred variables that is a
//!    straightforward design choice and we should know its size.
//! 3. **Reads, on the same low counts.** The `Truth` side reads from the sim
//!    every frame, and with a small export set that is proportionally a larger
//!    share of the boundary traffic than it was.
//! 4. **Writing a value that has not changed.** This matters *more* with a
//!    small export set, not less: if "write only what moved" is cheap, per-frame
//!    boundary traffic drops to near nothing.
//!
//! Timings use `std::time::Instant`, a monotonic clock, verified to work inside
//! MSFS WASM by FlyByWire's own shipping use of it in
//! `systems_wasm/src/aspects.rs:655`. The module sanity-checks it at startup and
//! refuses to report a verdict if it returns zero-length intervals.
//!
//! Results go to the MSFS console via `println!` -- the channel FlyByWire's own
//! modules use (`a380_systems_wasm/src/lib.rs:52`) -- as one delimited block,
//! *and* to LVars named `LVARBENCH_*` so they can be read from the dev toolbar's
//! variable watch without a console.

use msfs::legacy::NamedVariable;
use msfs::MSFSEvent;
use std::error::Error;
use std::time::Instant;

/// The sweep, weighted to the real export set. 50-2,000 is the region that
/// matters; 4,228 (the size of the internal cross-area bus, which does *not*
/// cross the boundary in MSFS) is kept only as a ceiling check.
const COUNTS: [usize; 7] = [50, 100, 250, 500, 1_000, 2_000, 4_228];

/// Index into `COUNTS` of the count the headline is quoted at: 250, a
/// representative export set (~49 authority couplings plus EFB live state).
const HEADLINE_IDX: usize = 2;
/// Index of the second headline, 500, the upper end of "hundreds".
const HEADLINE_HI_IDX: usize = 3;
/// Index of the ceiling check, 4,228.
const CEILING_IDX: usize = 6;

/// How many variables to register. The largest sweep count.
const MAX_VARS: usize = 4_228;

/// At n = 50 a single pass is a few microseconds, which is at or below the
/// resolution of some WASM clocks and would be measured as zero. Every timed
/// block therefore repeats its loop until it has done at least this many
/// operations, so the interval being timed is always comfortably above clock
/// resolution. The working set stays n variables wide, so cache behaviour at
/// that size is still what is being measured.
///
/// This was 20,000 in the first run, which made the high counts do ~21,000
/// operations in a single frame and visibly dragged the sim down (~22 fps at
/// the low counts, ~7 fps at the high ones). The clock turned out to have
/// plenty of resolution -- it measured 24,300 ns over a 200k spin -- so 8,000
/// is still far above it and roughly halves the worst frame.
const MIN_OPS_PER_TIMED_BLOCK: usize = 8_000;

/// How many times each (mode, count) cell is visited before reporting. Each
/// visit is one frame: 5 modes * 7 counts * 50 repeats = 1,750 frames, around
/// 29 seconds at 60 fps.
const REPEATS: u32 = 50;

/// Frames to let the sim settle before the first measurement.
const WARMUP_FRAMES: u64 = 60;

/// Progress is printed this often so a long run does not look hung.
const PROGRESS_EVERY: u64 = 350;

/// Once the results are in, the whole block is re-printed this often, forever,
/// so a user who looked away does not have to reload the aircraft. Measured in
/// wall-clock seconds rather than frames on purpose: the sweep itself drops the
/// frame rate, so a frame count is not a reliable interval.
const REPRINT_EVERY_SECONDS: u64 = 60;

/// Candidate paths for the filesystem probe, tried in order.
///
/// `\work\` first because that is the MSFS SDK's own documented convention for
/// WASM module persistence -- it maps to the package's work folder under
/// `LocalState\packages\<package>\work\`. The rest cover what WASI might have
/// preopened instead. Note that `\work\` is implemented by the MSFS libc's
/// `fopen`, so it is probed through C as well as through Rust's `std::fs`,
/// which talks raw WASI syscalls and may not see it at all.
#[cfg(feature = "fs-probe")]
const FS_PROBE_PATHS: [&str; 6] = [
    r"\work\lvarbench-results.txt",
    "/work/lvarbench-results.txt",
    "work/lvarbench-results.txt",
    "lvarbench-results.txt",
    "./lvarbench-results.txt",
    "/lvarbench-results.txt",
];

/// The value used by the "write an unchanged value" mode.
const STATIC_VALUE: f64 = 0.5;

#[derive(Clone, Copy, PartialEq, Eq)]
enum Mode {
    /// Write through a handle held since construction. The design we would use.
    WriteHandle = 0,
    /// Resolve the name, then write, every single operation.
    WriteLookup = 1,
    /// Read through a held handle.
    ReadHandle = 2,
    /// Resolve the name, then read.
    ReadLookup = 3,
    /// Write, through a held handle, the value the variable already holds.
    WriteUnchanged = 4,
}

const MODES: [Mode; 5] = [
    Mode::WriteHandle,
    Mode::WriteLookup,
    Mode::ReadHandle,
    Mode::ReadLookup,
    Mode::WriteUnchanged,
];

const N_MODES: usize = MODES.len();
const N_COUNTS: usize = COUNTS.len();
const CELLS: usize = N_MODES * N_COUNTS;
const TOTAL_MEASURED_FRAMES: u64 = (CELLS as u64) * (REPEATS as u64);

/// Names of the variables the sweep hammers. Deliberately prefixed
/// `LVARBENCH_X` so they cannot collide with the `LVARBENCH_NS_*` /
/// `LVARBENCH_MS_*` result variables published at the end.
fn bench_var_name(i: usize) -> String {
    format!("LVARBENCH_X{i:05}")
}

/// How many times the inner loop repeats for a working set of `n` variables.
fn inner_reps(n: usize) -> usize {
    (MIN_OPS_PER_TIMED_BLOCK + n - 1) / n
}

struct Bench {
    /// Pre-built names, so the lookup mode measures the cost of
    /// `register_named_variable` and the `CString` it needs -- not the cost of
    /// `format!`. A real module would hold its names as `&str` too.
    names: Vec<String>,
    /// Handles resolved once, at construction. The discipline under test.
    vars: Vec<NamedVariable>,

    /// Accumulated nanoseconds per (mode, count) cell.
    nanos: [[u128; N_COUNTS]; N_MODES],
    /// Accumulated operations per cell, so ns/op survives a short run.
    ops: [[u64; N_COUNTS]; N_MODES],

    /// Nanoseconds to register `MAX_VARS` variables at construction.
    register_nanos: u128,

    /// Sum of everything read, written out at the end so no read can be
    /// optimised away as dead.
    sink: f64,

    /// Non-zero if the monotonic clock ever produced a zero-length interval for
    /// a whole timed block, which would mean those numbers are worthless.
    zero_interval_cells: u32,
    /// Whether the clock survived a startup sanity check.
    clock_ok: bool,

    frame: u64,
    measured: u64,
    /// Sim time accumulated from the frame delta, an independent witness that
    /// the run really happened even if `Instant` turns out to be a stub.
    sim_seconds: f64,
    /// Wall clock at construction, so the report can state the real elapsed
    /// time and the average frame rate the sweep actually ran at.
    started: Instant,
    done: bool,
    /// When the block was last printed, for the 60-second reprint.
    last_report: Option<Instant>,
    /// One line per probed path, filled in after the results are printed.
    fs_report: Vec<String>,
    /// The path that worked, if any.
    fs_written_to: Option<String>,
}

impl Bench {
    fn new() -> Self {
        println!("LVARBENCH: registering {MAX_VARS} named variables...");

        let names: Vec<String> = (0..MAX_VARS).map(bench_var_name).collect();

        // Sanity-check the clock before trusting it for anything. A monotonic
        // clock that always returns the same instant would silently produce
        // "0 ns per write", which is the most dangerous possible wrong answer.
        let probe = Instant::now();
        let mut spin = 0u64;
        for i in 0..200_000u64 {
            spin = spin.wrapping_add(i).wrapping_mul(2_654_435_761);
        }
        let probe_ns = probe.elapsed().as_nanos();
        let clock_ok = probe_ns > 0;
        std::hint::black_box(spin);
        if clock_ok {
            println!("LVARBENCH: monotonic clock OK ({probe_ns} ns over a 200k-iteration spin).");
        } else {
            println!(
                "LVARBENCH: WARNING -- std::time::Instant reported a zero-length \
                 interval over a 200k-iteration spin. Every timing below is \
                 unusable. Report this rather than the numbers."
            );
        }

        let t0 = Instant::now();
        let vars: Vec<NamedVariable> = names.iter().map(|n| NamedVariable::from(n)).collect();
        let register_nanos = t0.elapsed().as_nanos();

        println!(
            "LVARBENCH: registered {} variables in {:.3} ms ({} ns each).",
            vars.len(),
            register_nanos as f64 / 1.0e6,
            register_nanos / vars.len().max(1) as u128
        );
        println!(
            "LVARBENCH: sweeping {N_MODES} modes x {N_COUNTS} counts x {REPEATS} repeats \
             = {TOTAL_MEASURED_FRAMES} frames (~{:.0} s at 60 fps). Results print when done.",
            TOTAL_MEASURED_FRAMES as f64 / 60.0
        );

        Self {
            names,
            vars,
            nanos: [[0; N_COUNTS]; N_MODES],
            ops: [[0; N_COUNTS]; N_MODES],
            register_nanos,
            sink: 0.0,
            zero_interval_cells: 0,
            clock_ok,
            frame: 0,
            measured: 0,
            sim_seconds: 0.0,
            started: Instant::now(),
            done: false,
            last_report: None,
            fs_report: Vec::new(),
            fs_written_to: None,
        }
    }

    fn tick(&mut self, delta_seconds: f64) {
        self.frame += 1;
        self.sim_seconds += delta_seconds;

        if self.done {
            let due = self
                .last_report
                .map(|t| t.elapsed().as_secs() >= REPRINT_EVERY_SECONDS)
                .unwrap_or(true);
            if due {
                self.emit_report();
            }
            return;
        }

        if self.frame <= WARMUP_FRAMES {
            return;
        }

        // Walk the cells in round-robin order rather than finishing one cell
        // before starting the next, so a drift in machine state (thermal, other
        // add-ons loading) spreads evenly over every cell instead of poisoning
        // whichever one happened to run during it.
        let cell = (self.measured as usize) % CELLS;
        let mode_idx = cell / N_COUNTS;
        let count_idx = cell % N_COUNTS;
        self.run_cell(MODES[mode_idx], mode_idx, count_idx);
        self.measured += 1;

        if self.measured % PROGRESS_EVERY == 0 {
            let pct = 100.0 * self.measured as f64 / TOTAL_MEASURED_FRAMES as f64;
            println!(
                "LVARBENCH: {:.0}% ({}/{} frames measured)",
                pct, self.measured, TOTAL_MEASURED_FRAMES
            );
            self.publish_progress();
        }

        if self.measured >= TOTAL_MEASURED_FRAMES {
            self.done = true;
            self.publish_results();
            // Console first, unconditionally. The filesystem probe below calls
            // into syscalls this sandbox may not implement at all, and an
            // unimplemented import traps rather than returning an error, which
            // would take the gauge down with it. Printing the results before
            // touching the filesystem means the worst case still leaves the
            // whole block on screen -- and the probe announces each path before
            // trying it, so even a trap says which call did it.
            self.emit_report();
            self.probe_filesystem();
            // Re-emit, now including the filesystem verdict and, if a path
            // worked, having written the block to disk.
            self.emit_report();
        }
    }

    fn run_cell(&mut self, mode: Mode, mode_idx: usize, count_idx: usize) {
        let n = COUNTS[count_idx];
        let reps = inner_reps(n);
        // A value that differs every visit, so "write" genuinely changes the
        // stored value and cannot be confused with the unchanged-write case.
        let base = 1.0 + (self.measured % 1024) as f64;

        if mode == Mode::WriteUnchanged {
            // Prime, untimed, so the timed loop really is writing the value
            // that is already there. Other modes run in between and disturb it.
            for i in 0..n {
                self.vars[i].set_value(STATIC_VALUE);
            }
        }

        let mut sum = 0.0f64;
        let t0 = Instant::now();
        match mode {
            Mode::WriteHandle => {
                for k in 0..reps {
                    // Offset per inner repetition, so every write in the block
                    // changes the stored value. Without this the second and
                    // later repetitions would silently be unchanged-writes and
                    // contaminate the very comparison being made.
                    let kb = base + (k as f64) * 65_536.0;
                    for i in 0..n {
                        self.vars[i].set_value(kb + i as f64);
                    }
                }
            }
            Mode::WriteLookup => {
                for k in 0..reps {
                    let kb = base + (k as f64) * 65_536.0;
                    for i in 0..n {
                        NamedVariable::from(&self.names[i]).set_value(kb + i as f64);
                    }
                }
            }
            Mode::ReadHandle => {
                for _ in 0..reps {
                    for i in 0..n {
                        sum += self.vars[i].get_value::<f64>();
                    }
                }
            }
            Mode::ReadLookup => {
                for _ in 0..reps {
                    for i in 0..n {
                        sum += NamedVariable::from(&self.names[i]).get_value::<f64>();
                    }
                }
            }
            Mode::WriteUnchanged => {
                for _ in 0..reps {
                    for i in 0..n {
                        self.vars[i].set_value(STATIC_VALUE);
                    }
                }
            }
        }
        let elapsed = t0.elapsed().as_nanos();
        self.sink += std::hint::black_box(sum);

        if elapsed == 0 {
            self.zero_interval_cells += 1;
        }
        self.nanos[mode_idx][count_idx] += elapsed;
        self.ops[mode_idx][count_idx] += (n * reps) as u64;
    }

    /// Nanoseconds per operation for a cell, or 0 if nothing was measured.
    fn ns_per_op(&self, mode_idx: usize, count_idx: usize) -> f64 {
        let ops = self.ops[mode_idx][count_idx];
        if ops == 0 {
            0.0
        } else {
            self.nanos[mode_idx][count_idx] as f64 / ops as f64
        }
    }

    /// Milliseconds a single frame would spend doing `COUNTS[count_idx]`
    /// operations of this kind.
    fn ms_per_frame(&self, mode_idx: usize, count_idx: usize) -> f64 {
        self.ns_per_op(mode_idx, count_idx) * COUNTS[count_idx] as f64 / 1.0e6
    }

    /// How many operations of this kind fit in one millisecond. The number that
    /// generalises: divide the frame budget you are willing to spend by this.
    fn ops_per_ms(&self, mode_idx: usize, count_idx: usize) -> f64 {
        let ns = self.ns_per_op(mode_idx, count_idx);
        if ns <= 0.0 {
            0.0
        } else {
            1.0e6 / ns
        }
    }

    /// How far the per-op cost at the top of the sweep has drifted from the
    /// bottom. 1.0 means perfectly linear; above about 1.5 the cost is
    /// superlinear and extrapolation past 4,228 is not safe.
    fn linearity(&self, mode_idx: usize) -> f64 {
        let low = self.ns_per_op(mode_idx, 0);
        let high = self.ns_per_op(mode_idx, N_COUNTS - 1);
        if low <= 0.0 {
            0.0
        } else {
            high / low
        }
    }

    /// Judged at the headline count (250), an export set of a few hundred.
    /// 1 = the boundary is effectively free. 2 = affordable, but write only
    /// what moved. 3 = even a few hundred is expensive; rethink what crosses.
    /// Thresholds are justified in docs/msfs-lvar-bench.md.
    fn verdict(&self) -> u8 {
        let ms = self.ms_per_frame(Mode::WriteHandle as usize, HEADLINE_IDX)
            + self.ms_per_frame(Mode::ReadHandle as usize, HEADLINE_IDX);
        if !self.clock_ok || self.zero_interval_cells > 0 {
            0
        } else if ms < 0.10 {
            1
        } else if ms < 0.50 {
            2
        } else {
            3
        }
    }

    fn publish_progress(&self) {
        NamedVariable::from("LVARBENCH_PROGRESS")
            .set_value(self.measured as f64 / TOTAL_MEASURED_FRAMES as f64);
        NamedVariable::from("LVARBENCH_DONE").set_value(0.0);
    }

    fn publish_results(&self) {
        let w = Mode::WriteHandle as usize;
        let wl = Mode::WriteLookup as usize;
        let r = Mode::ReadHandle as usize;
        let rl = Mode::ReadLookup as usize;
        let wu = Mode::WriteUnchanged as usize;
        let h = HEADLINE_IDX;

        let set = |name: &str, v: f64| NamedVariable::from(name).set_value(v);

        // Per-operation costs, quoted at the headline count.
        set("LVARBENCH_NS_PER_WRITE", self.ns_per_op(w, h));
        set("LVARBENCH_NS_PER_WRITE_LOOKUP", self.ns_per_op(wl, h));
        set("LVARBENCH_NS_PER_READ", self.ns_per_op(r, h));
        set("LVARBENCH_NS_PER_READ_LOOKUP", self.ns_per_op(rl, h));
        set("LVARBENCH_NS_PER_WRITE_UNCHANGED", self.ns_per_op(wu, h));
        set(
            "LVARBENCH_NS_PER_LOOKUP",
            self.ns_per_op(wl, h) - self.ns_per_op(w, h),
        );
        set(
            "LVARBENCH_NS_PER_REGISTER",
            self.register_nanos as f64 / MAX_VARS as f64,
        );

        // The numbers that generalise to whatever the export set turns out to be.
        set("LVARBENCH_WRITES_PER_MS", self.ops_per_ms(w, h));
        set("LVARBENCH_READS_PER_MS", self.ops_per_ms(r, h));

        // Totals at plausible export-set sizes, and the ceiling check.
        set("LVARBENCH_MS_PER_FRAME_250", self.ms_per_frame(w, h));
        set(
            "LVARBENCH_MS_PER_FRAME_500",
            self.ms_per_frame(w, HEADLINE_HI_IDX),
        );
        set("LVARBENCH_MS_PER_FRAME_READ_250", self.ms_per_frame(r, h));
        set(
            "LVARBENCH_MS_PER_FRAME_4228",
            self.ms_per_frame(w, CEILING_IDX),
        );

        set("LVARBENCH_LINEARITY_X100", self.linearity(w) * 100.0);
        set("LVARBENCH_CLOCK_OK", if self.clock_ok { 1.0 } else { 0.0 });
        set(
            "LVARBENCH_ZERO_INTERVAL_CELLS",
            self.zero_interval_cells as f64,
        );
        set("LVARBENCH_FRAMES", self.measured as f64);
        set("LVARBENCH_VERDICT", self.verdict() as f64);
        set("LVARBENCH_SINK", self.sink);
        // Overwritten by the probe, which runs after the results are printed.
        set("LVARBENCH_FS_OK", 0.0);
        set("LVARBENCH_PROGRESS", 1.0);
        set("LVARBENCH_DONE", 1.0);
    }

    /// The whole results block as one string, so it can go to the console and
    /// to a file without the two ever drifting apart.
    fn report_text(&self) -> String {
        use std::fmt::Write as _;
        let mut o = String::with_capacity(4096);
        let w = Mode::WriteHandle as usize;
        let wl = Mode::WriteLookup as usize;
        let r = Mode::ReadHandle as usize;
        let wu = Mode::WriteUnchanged as usize;
        let h = HEADLINE_IDX;
        let wall = self.started.elapsed().as_secs_f64();

        let _ = writeln!(o, "================================ LVARBENCH RESULTS ================================");
        let _ = writeln!(
            o,
            " frames {}   sim {:.1} s   wall {:.1} s   avg {:.1} fps   clock ok={} zero-blocks={}",
            self.measured,
            self.sim_seconds,
            wall,
            if wall > 0.0 { self.frame as f64 / wall } else { 0.0 },
            self.clock_ok,
            self.zero_interval_cells
        );
        let _ = writeln!(
            o,
            " registration {} vars in {:.3} ms = {:.0} ns each   inner reps: >={} ops per timed block",
            MAX_VARS,
            self.register_nanos as f64 / 1.0e6,
            self.register_nanos as f64 / MAX_VARS as f64,
            MIN_OPS_PER_TIMED_BLOCK
        );
        let _ = writeln!(o, "----------------------------------------------------------------------------------");
        let _ = writeln!(o, "          |   write hdl  |  write name  |   read hdl   |  read name   |  write same ");
        let _ = writeln!(o, "    count | ns/op    ms  | ns/op    ms  | ns/op    ms  | ns/op    ms  | ns/op    ms  ");
        for (ci, count) in COUNTS.iter().enumerate() {
            let _ = write!(o, "  {count:7} ");
            for mi in 0..N_MODES {
                let _ = write!(
                    o,
                    "| {:5.0} {:6.3} ",
                    self.ns_per_op(mi, ci),
                    self.ms_per_frame(mi, ci)
                );
            }
            let _ = writeln!(o);
        }
        let _ = writeln!(o, "----------------------------------------------------------------------------------");
        let _ = writeln!(
            o,
            " HEADLINE n={} (representative export set: ~49 authority couplings + EFB live state)",
            COUNTS[h]
        );
        let lookup_ns = self.ns_per_op(wl, h) - self.ns_per_op(w, h);
        let saved = self.ms_per_frame(w, h) - self.ms_per_frame(wu, h);
        let saved_pct = if self.ms_per_frame(w, h) > 0.0 {
            100.0 * saved / self.ms_per_frame(w, h)
        } else {
            0.0
        };
        let _ = writeln!(
            o,
            "   write {:.0} ns   read {:.0} ns   name-lookup overhead {:.0} ns   unchanged-write saves {:.1}%",
            self.ns_per_op(w, h),
            self.ns_per_op(r, h),
            lookup_ns,
            saved_pct
        );
        let _ = writeln!(
            o,
            "   fits in 1 ms: {:.0} writes / {:.0} reads     read+write at n={}: {:.4} ms/frame",
            self.ops_per_ms(w, h),
            self.ops_per_ms(r, h),
            COUNTS[h],
            self.ms_per_frame(w, h) + self.ms_per_frame(r, h)
        );
        let _ = writeln!(
            o,
            "   at n={}: {:.4} ms/frame     ceiling n={} (internal bus, does NOT cross): {:.4} ms",
            COUNTS[HEADLINE_HI_IDX],
            self.ms_per_frame(w, HEADLINE_HI_IDX) + self.ms_per_frame(r, HEADLINE_HI_IDX),
            COUNTS[CEILING_IDX],
            self.ms_per_frame(w, CEILING_IDX)
        );
        let _ = writeln!(
            o,
            "   linearity {} vs {}: {:.2}x writes, {:.2}x reads ({})   unchanged: {}",
            COUNTS[N_COUNTS - 1],
            COUNTS[0],
            self.linearity(w),
            self.linearity(r),
            if self.linearity(w) < 1.5 { "linear" } else { "SUPERLINEAR" },
            if saved_pct < 2.0 { "no saving, skipping is ours to do" } else { "MSFS is cheaper" }
        );
        let _ = writeln!(o, "----------------------------------------------------------------------------------");
        let v = self.verdict();
        let _ = writeln!(o, " VERDICT {v}: {}", verdict_text(v));
        let _ = writeln!(o, "----------------------------------------------------------------------------------");
        if self.fs_report.is_empty() {
            let _ = writeln!(o, " FILESYSTEM: not probed yet.");
        } else {
            let _ = writeln!(o, " FILESYSTEM PROBE -- does MSFS WASM have one?");
            for line in &self.fs_report {
                let _ = writeln!(o, "   {line}");
            }
            match &self.fs_written_to {
                Some(p) => {
                    let _ = writeln!(o, "   => YES. This block was also written to: {p}");
                }
                None => {
                    let _ = writeln!(
                        o,
                        "   => NO. Every path failed. Screenshot this block; it is the only copy."
                    );
                }
            }
        }
        let _ = writeln!(o, "=========================== END LVARBENCH RESULTS =================================");
        o
    }

    /// Print the block, and refresh the file if the probe found a path that
    /// works. Called again every `REPRINT_EVERY_SECONDS` for as long as the
    /// module lives, so a user who looked away never has to reload.
    fn emit_report(&mut self) {
        let text = self.report_text();
        println!("\n{text}");
        #[cfg(feature = "fs-probe")]
        if let Some(path) = self.fs_written_to.clone() {
            let _ = std::fs::write(&path, &text);
        }
        self.last_report = Some(Instant::now());
    }

    /// Settle, once and for all, whether this sandbox has a filesystem.
    ///
    /// The earlier finding was that *msfs-rs* exposes no filesystem API, which
    /// is a different claim from the host providing none: this crate targets
    /// `wasm32-wasip1`, and WASI does have a filesystem interface. The open
    /// questions are whether MSFS preopens any directory for it, and separately
    /// whether the MSFS libc's own `\work\` convention is reachable.
    ///
    /// Both are probed, for every candidate path, and every outcome is printed
    /// with the exact path and the exact error. A definitive "no, and here is
    /// the error" is worth as much as a success.
    ///
    /// Each attempt announces itself *before* it is made. If an unimplemented
    /// syscall traps rather than returning an error -- which would take the
    /// whole gauge down, uncatchably, because a WASM trap is not a Rust panic --
    /// the console still shows exactly which call did it.
    #[cfg(not(feature = "fs-probe"))]
    fn probe_filesystem(&mut self) {
        self.fs_report
            .push("not probed: built with --no-default-features".to_string());
        println!("LVARBENCH: filesystem probe compiled out (fs-probe feature off).");
    }

    #[cfg(feature = "fs-probe")]
    fn probe_filesystem(&mut self) {
        let payload = self.report_text();
        println!(
            "LVARBENCH: probing for a filesystem ({} paths, Rust std::fs then C fopen)...",
            FS_PROBE_PATHS.len()
        );

        for path in FS_PROBE_PATHS {
            println!("LVARBENCH:   trying std::fs::write({path:?})");
            match std::fs::write(path, payload.as_bytes()) {
                Ok(()) => {
                    let line = format!("std::fs  {path:<34} OK");
                    println!("LVARBENCH:   {line}");
                    self.fs_report.push(line);
                    self.fs_written_to = Some(path.to_string());
                    break;
                }
                Err(e) => {
                    let line = format!("std::fs  {path:<34} {e}");
                    println!("LVARBENCH:   {line}");
                    self.fs_report.push(line);
                }
            }
        }

        if self.fs_written_to.is_none() {
            for path in FS_PROBE_PATHS {
                println!("LVARBENCH:   trying C fopen({path:?}, \"w\")");
                match c_write(path, payload.as_bytes()) {
                    Ok(n) => {
                        let line = format!("fopen    {path:<34} OK, {n} bytes");
                        println!("LVARBENCH:   {line}");
                        self.fs_report.push(line);
                        self.fs_written_to = Some(path.to_string());
                        break;
                    }
                    Err(e) => {
                        let line = format!("fopen    {path:<34} {e}");
                        println!("LVARBENCH:   {line}");
                        self.fs_report.push(line);
                    }
                }
            }
        }

        let ok = self.fs_written_to.is_some();
        NamedVariable::from("LVARBENCH_FS_OK").set_value(if ok { 1.0 } else { 0.0 });
        println!(
            "LVARBENCH: filesystem probe finished -- {}",
            match &self.fs_written_to {
                Some(p) => format!("WRITES WORK, wrote {p}"),
                None => "NO PATH ACCEPTED A WRITE".to_string(),
            }
        );
    }
}

#[cfg(feature = "fs-probe")]
// The MSFS libc, linked in through the SDK's wasi-sysroot (`-lc`). Rust's
// `std::fs` talks raw WASI syscalls and would not see a path convention
// implemented inside their libc, so the candidates are tried through C too.
extern "C" {
    fn fopen(
        path: *const core::ffi::c_char,
        mode: *const core::ffi::c_char,
    ) -> *mut core::ffi::c_void;
    fn fwrite(
        ptr: *const core::ffi::c_void,
        size: usize,
        nmemb: usize,
        stream: *mut core::ffi::c_void,
    ) -> usize;
    fn fclose(stream: *mut core::ffi::c_void) -> core::ffi::c_int;
}

#[cfg(feature = "fs-probe")]
/// Write `bytes` to `path` through the C runtime. Returns the byte count.
fn c_write(path: &str, bytes: &[u8]) -> Result<usize, String> {
    let cpath = match std::ffi::CString::new(path) {
        Ok(c) => c,
        Err(_) => return Err("path contains a NUL byte".to_string()),
    };
    let mode = std::ffi::CString::new("w").unwrap();
    unsafe {
        let f = fopen(cpath.as_ptr(), mode.as_ptr());
        if f.is_null() {
            return Err("fopen returned NULL".to_string());
        }
        let n = fwrite(
            bytes.as_ptr() as *const core::ffi::c_void,
            1,
            bytes.len(),
            f,
        );
        fclose(f);
        if n == 0 && !bytes.is_empty() {
            Err("fopen succeeded but fwrite wrote 0 bytes".to_string())
        } else {
            Ok(n)
        }
    }
}

fn verdict_text(v: u8) -> &'static str {
    match v {
        0 => {
            "MEASUREMENT INVALID -- the monotonic clock did not work. \
             Report the clock line, not the numbers."
        }
        1 => {
            "The boundary is effectively free at a few hundred variables. \
             Export everything the outside needs, every frame, through handles \
             resolved at construction. No change detection needed."
        }
        2 => {
            "Affordable, but not free. Export through held handles and write \
             only the values that moved -- cheap to implement and it removes \
             most of the traffic."
        }
        _ => {
            "Even a few hundred costs real frame time. Cut what crosses the \
             boundary to the authority couplings and a minimal EFB set, and \
             poll the rest on demand rather than publishing it."
        }
    }
}

#[msfs::gauge(name = lvarbench)]
async fn lvarbench(mut gauge: msfs::Gauge) -> Result<(), Box<dyn Error>> {
    // The default panic output is lost when the WASM instance aborts, so print
    // the panic to the MSFS console first. Copied from FlyByWire's own module
    // (fbw-a380x/src/wasm/systems/a380_systems_wasm/src/lib.rs:52).
    std::panic::set_hook(Box::new(|panic_info| {
        println!("LVARBENCH PANIC: {panic_info}");
    }));

    println!("LVARBENCH: module loaded.");
    let mut bench = Bench::new();

    while let Some(event) = gauge.next_event().await {
        if let MSFSEvent::PreDraw(data) = event {
            bench.tick(data.delta_time().as_secs_f64());
        }
    }

    Ok(())
}
