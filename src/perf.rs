//! Where the plugin's time goes on X-Plane's main thread: the frame's work
//! split at its `[slot ...]` markers, and the screens' draw callbacks,
//! summed and written to the log every ten seconds as milliseconds per frame.

use std::cell::RefCell;
use std::time::{Duration, Instant};

const REPORT_EVERY: Duration = Duration::from_secs(10);

/// Screen pixels copied into X-Plane's textures since the last report.
pub static UPLOADED_BYTES: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);

struct Laps {
    last: Option<(Instant, &'static str)>,
    totals: Vec<(&'static str, Duration)>,
    frames: u32,
    since: Instant,
}

thread_local! {
    static LAPS: RefCell<Laps> = RefCell::new(Laps { last: None, totals: Vec::new(), frames: 0, since: Instant::now() });
}

fn add(totals: &mut Vec<(&'static str, Duration)>, label: &'static str, d: Duration) {
    match totals.iter_mut().find(|(l, _)| *l == label) {
        Some(entry) => entry.1 += d,
        None => totals.push((label, d)),
    }
}

/// Ends the section before this point and starts `label`'s.
pub fn lap(label: &'static str) {
    LAPS.with(|l| {
        let mut l = l.borrow_mut();
        let now = Instant::now();
        if let Some((start, previous)) = l.last {
            add(&mut l.totals, previous, now - start);
        }
        l.last = Some((now, label));
    });
}

/// Ends the frame's last section; reports now and then.
pub fn end_frame() {
    LAPS.with(|l| {
        let mut l = l.borrow_mut();
        if let Some((start, previous)) = l.last.take() {
            add(&mut l.totals, previous, start.elapsed());
        }
        l.frames += 1;
        if l.since.elapsed() >= REPORT_EVERY {
            let frames = l.frames.max(1) as f64;
            let mut totals = std::mem::take(&mut l.totals);
            totals.sort_by(|a, b| b.1.cmp(&a.1));
            let sum: f64 = totals.iter().map(|(_, d)| d.as_secs_f64()).sum::<f64>() * 1000. / frames;
            let top: Vec<String> =
                totals.iter().take(12).map(|(label, d)| format!("{label} {:.2}", d.as_secs_f64() * 1000. / frames)).collect();
            let fps = frames / l.since.elapsed().as_secs_f64();
            let mb = UPLOADED_BYTES.swap(0, std::sync::atomic::Ordering::Relaxed) as f64 / frames / 1_048_576.;
            crate::log(&format!("perf: {fps:.1} plugin frames/s, {sum:.2} ms per frame in the plugin, {mb:.1} MB of screen pixels uploaded per frame; ms per frame: {}", top.join(", ")));
            l.frames = 0;
            l.since = Instant::now();
        }
    });
}

/// Times one call (a draw callback) into the current report.
pub fn time<R>(label: &'static str, f: impl FnOnce() -> R) -> R {
    let start = Instant::now();
    let r = f();
    let d = start.elapsed();
    LAPS.with(|l| add(&mut l.borrow_mut().totals, label, d));
    r
}
