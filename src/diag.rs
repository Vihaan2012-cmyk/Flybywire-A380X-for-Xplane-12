//! One-launch diagnostics capture, on when `Output/preferences/fbw_a380x_diagnostics.ini`
//! says `enabled=1`. Everything lands in `Output/fbw-a380x/diagnostics/<start time>/`.

use std::collections::HashMap;
use std::ffi::c_void;
use std::io::Write;
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::mpsc::{channel, RecvTimeoutError, Sender};
use std::sync::{Mutex, OnceLock};
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use crate::xp::{DataRef, Xplm};

pub const PREFERENCES: &str = "Output/preferences/fbw_a380x_diagnostics.ini";
const ROOT: &str = "Output/fbw-a380x/diagnostics";
const SAMPLE_EVERY: Duration = Duration::from_millis(200);
const PROCESSES_EVERY: Duration = Duration::from_secs(2);

pub static DATAREF_READS: AtomicU64 = AtomicU64::new(0);
pub static DRAW_NS: AtomicU64 = AtomicU64::new(0);
pub static TICK_NS: AtomicU64 = AtomicU64::new(0);
static TOUR_REQUESTED: AtomicBool = AtomicBool::new(false);
static SCREENS_REQUESTED: AtomicBool = AtomicBool::new(false);
static SAVE_REQUESTED: AtomicBool = AtomicBool::new(false);
static MARKS: AtomicU64 = AtomicU64::new(0);

const ENV: &[(&str, &str)] = &[
    ("FBW_LOG_ALL", "1"),
    ("FBW_LOG_KEY", "5"),
    ("FBW_ENG_STATS", "1"),
    ("FBW_FLEX_STATS", "1"),
    ("FBW_NAV_STATS", "1"),
    ("FBW_SCREEN_STATS", "1"),
    ("FBW_FCTL_STATS", "1"),
    ("FBW_HYD_STATS", "1"),
    ("FBW_DUMP", "3600"),
];

#[derive(Clone, Copy, PartialEq, Eq, Hash, Debug)]
pub enum Stream {
    Events,
    State,
    Samples,
    Processes,
}

impl Stream {
    fn file(self) -> &'static str {
        match self {
            Stream::Events => "events.log",
            Stream::State => "state.log",
            Stream::Samples => "samples.csv",
            Stream::Processes => "processes.csv",
        }
    }
}

struct Session {
    dir: PathBuf,
    started: Instant,
    started_wall: SystemTime,
    tx: Mutex<Sender<(Stream, String)>>,
}

static SESSION: OnceLock<Option<Session>> = OnceLock::new();

fn session() -> Option<&'static Session> {
    SESSION.get().and_then(Option::as_ref)
}

pub fn enabled_in(ini: &str) -> bool {
    crate::efb::parse_ini(ini).get("enabled").is_some_and(|v| matches!(v.trim().to_ascii_lowercase().as_str(), "1" | "true" | "yes" | "on"))
}

pub fn set_enabled_for_next_launch(on: bool) -> std::io::Result<()> {
    if let Some(dir) = Path::new(PREFERENCES).parent() {
        std::fs::create_dir_all(dir)?;
    }
    std::fs::write(PREFERENCES, format!("enabled={}\n", u8::from(on)))
}

pub fn enabled_for_next_launch() -> bool {
    enabled_in(&std::fs::read_to_string(PREFERENCES).unwrap_or_default())
}

pub fn active() -> bool {
    session().is_some()
}

pub fn dir() -> Option<&'static Path> {
    session().map(|s| s.dir.as_path())
}

pub fn elapsed() -> f64 {
    session().map_or(0.0, |s| s.started.elapsed().as_secs_f64())
}

pub fn start() {
    SESSION.get_or_init(|| {
        if !enabled_for_next_launch() {
            return None;
        }
        for (key, value) in ENV {
            if std::env::var_os(key).is_none() {
                std::env::set_var(key, value);
            }
        }
        let started_wall = SystemTime::now();
        let dir = PathBuf::from(ROOT).join(stamp(started_wall));
        std::fs::create_dir_all(&dir).ok()?;
        let (tx, rx) = channel::<(Stream, String)>();
        let writer_dir = dir.clone();
        std::thread::Builder::new()
            .name("fbw-diag-writer".into())
            .spawn(move || {
                let mut files: HashMap<Stream, std::io::BufWriter<std::fs::File>> = HashMap::new();
                let mut last_flush = Instant::now();
                loop {
                    match rx.recv_timeout(Duration::from_millis(500)) {
                        Ok((stream, line)) => {
                            let file = files.entry(stream).or_insert_with(|| {
                                let f = std::fs::OpenOptions::new().create(true).append(true).open(writer_dir.join(stream.file())).expect("diag file");
                                std::io::BufWriter::with_capacity(1 << 16, f)
                            });
                            let _ = writeln!(file, "{line}");
                            if stream == Stream::Events {
                                let _ = file.flush();
                            }
                        }
                        Err(RecvTimeoutError::Timeout) => {}
                        Err(RecvTimeoutError::Disconnected) => break,
                    }
                    if last_flush.elapsed() >= Duration::from_secs(1) {
                        for f in files.values_mut() {
                            let _ = f.flush();
                        }
                        last_flush = Instant::now();
                    }
                }
                for f in files.values_mut() {
                    let _ = f.flush();
                }
            })
            .ok()?;
        let session = Session { dir: dir.clone(), started: Instant::now(), started_wall, tx: Mutex::new(tx) };
        let _ = std::fs::write(dir.join("README.txt"), test_plan());
        let _ = std::fs::write(dir.join("session.txt"), environment(started_wall));
        std::thread::Builder::new().name("fbw-diag-processes".into()).spawn(processes_loop).ok();
        Some(session)
    });
}

fn aircraft_dir() -> Option<PathBuf> {
    #[cfg(feature = "js")]
    {
        crate::js_bridge::plugin_dir().and_then(|d| d.parent().and_then(Path::parent).map(Path::to_path_buf))
    }
    #[cfg(not(feature = "js"))]
    {
        None
    }
}

fn send(stream: Stream, line: String) {
    if let Some(s) = session() {
        if let Ok(tx) = s.tx.lock() {
            let _ = tx.send((stream, line));
        }
    }
}

fn wall_clock() -> String {
    let secs = SystemTime::now().duration_since(UNIX_EPOCH).map_or(0.0, |d| d.as_secs_f64());
    let (h, m, s) = (((secs / 3600.0) as u64) % 24, ((secs / 60.0) as u64) % 60, secs % 60.0);
    format!("{h:02}:{m:02}:{s:06.3}Z")
}

pub fn event(category: &str, message: &str) {
    if active() {
        send(Stream::Events, format!("{:>9.3} {} [{category}] {message}", elapsed(), wall_clock()));
    }
}

pub fn mark(label: &str) -> u64 {
    capture(label, true)
}

static CAPTURES: Mutex<Vec<Capture>> = Mutex::new(Vec::new());
static CURRENT_STEP: Mutex<String> = Mutex::new(String::new());

struct Capture {
    folder: String,
    label: String,
    screenshot: bool,
    from_file: Option<PathBuf>,
}

pub fn set_step(label: &str) {
    let label = label.trim().to_string();
    event("STEP", &label);
    if let Ok(mut step) = CURRENT_STEP.lock() {
        *step = label;
    }
}

fn current_step() -> String {
    CURRENT_STEP.lock().map(|s| s.clone()).unwrap_or_default()
}

fn slug(text: &str) -> String {
    let mut out: String = text.chars().map(|c| if c.is_ascii_alphanumeric() { c.to_ascii_lowercase() } else { '-' }).collect();
    while out.contains("--") {
        out = out.replace("--", "-");
    }
    out.trim_matches('-').chars().take(48).collect()
}

pub fn capture(label: &str, screenshot: bool) -> u64 {
    queue_capture(label, screenshot, None)
}

fn queue_capture(label: &str, screenshot: bool, from_file: Option<PathBuf>) -> u64 {
    let n = MARKS.fetch_add(1, Ordering::Relaxed) + 1;
    let step = current_step();
    let label = match (step.is_empty(), label.trim().is_empty()) {
        (true, true) => format!("capture {n}"),
        (true, false) => label.trim().to_string(),
        (false, true) => step,
        (false, false) => format!("{step} - {}", label.trim()),
    };
    event("MARK", &format!("#{n} {label}"));
    crate::log(&format!("diagnostics capture #{n} {label}"));
    let folder = format!("{n:03}_{}", slug(&label));
    if let Ok(mut q) = CAPTURES.lock() {
        q.push(Capture { folder, label, screenshot, from_file });
    }
    n
}

pub fn state_line(line: &str) -> bool {
    if !active() {
        return false;
    }
    send(Stream::State, format!("{:>9.3} {line}", elapsed()));
    true
}

pub fn request_photo_tour() {
    TOUR_REQUESTED.store(true, Ordering::Relaxed);
}

pub fn request_screens() {
    SCREENS_REQUESTED.store(true, Ordering::Relaxed);
}

pub fn request_save() {
    SAVE_REQUESTED.store(true, Ordering::Relaxed);
}

struct Watch {
    failures: std::collections::BTreeSet<u64>,
    breakers: std::collections::BTreeSet<String>,
    next: Option<Instant>,
}

static WATCH: Mutex<Option<Watch>> = Mutex::new(None);

pub fn watch(failures: impl FnOnce() -> Vec<u64>, open_breakers: impl FnOnce() -> Vec<String>) {
    if !active() {
        return;
    }
    let Ok(mut guard) = WATCH.lock() else { return };
    let w = guard.get_or_insert_with(|| Watch { failures: Default::default(), breakers: Default::default(), next: None });
    let now = Instant::now();
    if w.next.is_some_and(|n| now < n) {
        return;
    }
    w.next = Some(now + Duration::from_secs(1));
    let f: std::collections::BTreeSet<u64> = failures().into_iter().collect();
    for id in f.difference(&w.failures) {
        event("failure", &format!("armed {id} {}", crate::failures::any_failure_name(*id)));
    }
    for id in w.failures.difference(&f) {
        event("failure", &format!("cleared {id} {}", crate::failures::any_failure_name(*id)));
    }
    w.failures = f;
    let b: std::collections::BTreeSet<String> = open_breakers().into_iter().collect();
    for name in b.difference(&w.breakers) {
        event("breaker", &format!("opened {name}"));
    }
    for name in w.breakers.difference(&b) {
        event("breaker", &format!("closed {name}"));
    }
    w.breakers = b;
}

pub fn health_text() -> String {
    let Some(s) = session() else { return "Diagnostics are not capturing this session.".into() };
    let read = |name: &str| std::fs::read_to_string(s.dir.join(name)).unwrap_or_default();
    crate::diag_health::report(&read("samples.csv"), &read("events.log"), &read("processes.csv"))
}

pub fn status_json() -> String {
    serde_json::json!({
        "active": active(),
        "enabledNextLaunch": enabled_for_next_launch(),
        "dir": dir().map(|d| d.display().to_string()),
        "elapsed": elapsed(),
        "marks": MARKS.load(Ordering::Relaxed),
        "tour": TOUR_STATE.lock().map(|t| t.as_ref().map(|t| format!("{}/{}", t.index + 1, SPOTS.len()))).ok().flatten(),
        "steps": STEPS,
    })
    .to_string()
}

fn stamp(t: SystemTime) -> String {
    let secs = t.duration_since(UNIX_EPOCH).map_or(0, |d| d.as_secs()) as i64;
    let (days, rem) = (secs.div_euclid(86_400), secs.rem_euclid(86_400));
    let (y, mo, d) = civil(days);
    format!("{y:04}-{mo:02}-{d:02}_{:02}-{:02}-{:02}Z", rem / 3600, rem / 60 % 60, rem % 60)
}

fn civil(days: i64) -> (i64, i64, i64) {
    let z = days + 719_468;
    let era = z.div_euclid(146_097);
    let doe = z - era * 146_097;
    let yoe = (doe - doe / 1460 + doe / 36_524 - doe / 146_096) / 365;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let d = doy - (153 * mp + 2) / 5 + 1;
    let m = if mp < 10 { mp + 3 } else { mp - 9 };
    (yoe + era * 400 + i64::from(m <= 2), m, d)
}

fn environment(started: SystemTime) -> String {
    let mut out = String::new();
    let mut line = |k: &str, v: String| out.push_str(&format!("{k}: {v}\n"));
    line("started", stamp(started));
    line("plugin", env!("CARGO_PKG_VERSION").to_string());
    let aircraft = aircraft_dir();
    if let Some(a) = &aircraft {
        line("aircraft", a.display().to_string());
        line("installed version", std::fs::read_to_string(a.join("fbw_xp_installed_version.txt")).unwrap_or_else(|_| "(none: not installed by the installer)".into()).trim().to_string());
        line("sasl", std::fs::read_to_string(a.join("plugins/sasl/version.txt")).unwrap_or_default().lines().find(|l| l.starts_with("Version")).unwrap_or("(missing)").to_string());
        for f in ["plugins/fbw_a380_systems/64/win.xpl", "plugins/sasl/data/modules/main.lua", "objects/a380_cockpit_click.obj", "A380.acf"] {
            let p = a.join(f);
            let meta = std::fs::metadata(&p).ok();
            line(f, meta.map_or("(missing)".into(), |m| format!("{} bytes, modified {}", m.len(), m.modified().map(stamp).unwrap_or_default())));
        }
        let sound = a.join("plugins/fbw_a380_systems/sound");
        let banks: Vec<String> = std::fs::read_dir(&sound).map(|d| d.filter_map(Result::ok).map(|e| e.file_name().to_string_lossy().into_owned()).collect()).unwrap_or_default();
        line("sound folder", if banks.is_empty() { "(empty or missing)".into() } else { banks.join(", ") });
    }
    for k in ["PROCESSOR_IDENTIFIER", "NUMBER_OF_PROCESSORS", "OS", "LOCALAPPDATA"] {
        line(k, std::env::var(k).unwrap_or_default());
    }
    for (k, _) in ENV {
        line(k, std::env::var(k).unwrap_or_default());
    }
    for f in [PREFERENCES, "Output/preferences/fbw_a380x_settings.ini", "Output/preferences/fbw_a380x_oans.ini"] {
        out.push_str(&format!("\n--- {f}\n{}\n", std::fs::read_to_string(f).unwrap_or_else(|_| "(missing)".into())));
    }
    out
}

fn test_plan() -> String {
    let mut out = String::from("A380 for X-Plane 12: diagnostics session\n\nOne launch, in this order. In the EFB Diagnostics tab press Capture on a step: that makes it the current step and saves a capture. Every X-Plane screenshot you take afterwards is saved as a capture of that step too, with every screen and every value at that moment (captures/ in this folder). The fbw/diag/mark command does the same from a key.\n\n");
    for (i, (title, detail)) in STEPS.iter().enumerate() {
        out.push_str(&format!("{:>2}. {title}\n    {detail}\n", i + 1));
    }
    out.push_str("\nWhen done: EFB Diagnostics > Save bundle, then quit X-Plane. Send the zip next to this folder.\n");
    out
}

pub const STEPS: &[(&str, &str)] = &[
    ("Cold and dark at a gate", "Load at a gate, cold and dark. Wait 30 s. Then PHOTO TOUR."),
    ("Fuselage from outside", "External view near door 1 on the left side: take X-Plane screenshots of the forward fuselage."),
    ("Internal camera moved outside", "3D cockpit view, then move the camera out through the side (or walk out of the door) and screenshot the forward fuselage."),
    ("Upper deck forward", "Walk back into the cabin, upper deck at the first door. Screenshot looking forward, left and right."),
    ("Wings and lights", "External view of each wing tip from the front and from behind: screenshot the tip shape and the nav/strobe light covers."),
    ("EXT PWR 1 to 4", "Ground power on in the EFB. Click EXT PWR 1, 2, 3, 4 on the overhead one at a time and press Capture after each."),
    ("Batteries and EFIS", "With power on, press LS, TRAF, CSTR, ARPT and VV on the captain's EFIS. Screenshot the EFIS panel."),
    ("MFD SURV page", "Open the SURV page on the MFD and press Capture."),
    ("Night cockpit lights", "Set the time to night. Turn the FLOOD LT knobs (main panel, pedestal), table, dome and reading lights up. Screenshot the cockpit."),
    ("EFB brightness", "Still at night: screenshot the EFB."),
    ("Sidestick and speedbrake", "Move your stick in all directions and your speedbrake axis. Screenshot the sidestick and the speedbrake lever mid-movement."),
    ("Seats", "Listen for buzzing with nothing touched. Move a seat with its switch, then stop. Capture with a note on what you heard."),
    ("EFB preset to Ready for Takeoff", "EFB presets: Ready for Takeoff. Wait until done. Then try to switch the STROBE and BEACON off. Capture."),
    ("EFB preset to Cold and Dark", "EFB presets: Cold and Dark once. Capture with a note on which lights are still on."),
    ("Parking brake", "Power up again, set the parking brake, turn the electric hydraulic pumps on. Capture."),
    ("Throttles", "Engines running: move each of your throttle levers on its own. Capture with both at idle, at half and at full."),
    ("Rudder on the ground", "Taxiing slowly: hold full twist left for 3 s, then full right for 3 s. Capture during each hold."),
    ("Takeoff and gear up", "Take off. Watch the bogies as the wheels leave the ground, then gear up. Screenshot the gear doors during retraction."),
    ("Turns", "In the air: 30-40 degree turns each way, then let go. Hold full twist for 3 s once. Capture."),
    ("Save", "Press Health check, then Save bundle, then quit X-Plane."),
];

fn processes_loop() {
    let cores = std::thread::available_parallelism().map_or(1, |n| n.get()) as f64;
    let mut last: HashMap<String, (u64, Instant)> = HashMap::new();
    let mut header = false;
    loop {
        std::thread::sleep(PROCESSES_EVERY);
        if !active() {
            continue;
        }
        if !header {
            header = true;
            send(Stream::Processes, "t,process,instances,cpu_cores,cpu_percent_of_machine,working_set_mb,private_mb".into());
        }
        let now = Instant::now();
        for (name, instances, cpu_100ns, ws, private) in process_stats(&["X-Plane.exe", "XPHFBW.exe", "fbw_a380_systems_server.exe"]) {
            let cores_used = match last.get(&name) {
                Some(&(prev, at)) if cpu_100ns >= prev => (cpu_100ns - prev) as f64 / 1e7 / now.duration_since(at).as_secs_f64(),
                _ => 0.0,
            };
            last.insert(name.clone(), (cpu_100ns, now));
            send(
                Stream::Processes,
                format!("{:.1},{name},{instances},{cores_used:.2},{:.1},{:.0},{:.0}", elapsed(), cores_used / cores * 100.0, ws as f64 / 1_048_576.0, private as f64 / 1_048_576.0),
            );
        }
    }
}

#[repr(C)]
struct ProcessEntry {
    size: u32,
    usage: u32,
    pid: u32,
    heap: usize,
    module: u32,
    threads: u32,
    parent: u32,
    priority: i32,
    flags: u32,
    exe: [u16; 260],
}

#[repr(C)]
#[derive(Default)]
struct MemoryCounters {
    cb: u32,
    page_faults: u32,
    peak_working_set: usize,
    working_set: usize,
    quota_peak_paged: usize,
    quota_paged: usize,
    quota_peak_nonpaged: usize,
    quota_nonpaged: usize,
    pagefile: usize,
    peak_pagefile: usize,
}

#[link(name = "kernel32")]
extern "system" {
    fn CreateToolhelp32Snapshot(flags: u32, pid: u32) -> *mut c_void;
    fn Process32FirstW(snapshot: *mut c_void, entry: *mut ProcessEntry) -> i32;
    fn Process32NextW(snapshot: *mut c_void, entry: *mut ProcessEntry) -> i32;
    fn OpenProcess(access: u32, inherit: i32, pid: u32) -> *mut c_void;
    fn GetProcessTimes(process: *mut c_void, creation: *mut u64, exit: *mut u64, kernel: *mut u64, user: *mut u64) -> i32;
    fn K32GetProcessMemoryInfo(process: *mut c_void, counters: *mut MemoryCounters, size: u32) -> i32;
    fn CloseHandle(handle: *mut c_void) -> i32;
}

fn process_stats(names: &[&str]) -> Vec<(String, u32, u64, u64, u64)> {
    let mut totals: Vec<(String, u32, u64, u64, u64)> = names.iter().map(|n| (n.to_string(), 0, 0, 0, 0)).collect();
    unsafe {
        let snapshot = CreateToolhelp32Snapshot(0x2, 0);
        if snapshot.is_null() || snapshot as isize == -1 {
            return totals;
        }
        let mut entry: ProcessEntry = std::mem::zeroed();
        entry.size = std::mem::size_of::<ProcessEntry>() as u32;
        let mut ok = Process32FirstW(snapshot, &mut entry) != 0;
        while ok {
            let len = entry.exe.iter().position(|&c| c == 0).unwrap_or(entry.exe.len());
            let exe = String::from_utf16_lossy(&entry.exe[..len]);
            if let Some(t) = totals.iter_mut().find(|t| t.0.eq_ignore_ascii_case(&exe)) {
                let h = OpenProcess(0x1000, 0, entry.pid);
                if !h.is_null() {
                    let (mut c, mut e, mut k, mut u) = (0u64, 0u64, 0u64, 0u64);
                    if GetProcessTimes(h, &mut c, &mut e, &mut k, &mut u) != 0 {
                        t.2 += k + u;
                    }
                    let mut m = MemoryCounters { cb: std::mem::size_of::<MemoryCounters>() as u32, ..Default::default() };
                    if K32GetProcessMemoryInfo(h, &mut m, m.cb) != 0 {
                        t.3 += m.working_set as u64;
                        t.4 += m.pagefile as u64;
                    }
                    CloseHandle(h);
                    t.1 += 1;
                }
            }
            ok = Process32NextW(snapshot, &mut entry) != 0;
        }
        CloseHandle(snapshot);
    }
    totals
}

#[derive(Clone, Copy)]
enum Kind {
    Float,
    Int,
    Floats(usize),
    Ints(usize),
}

struct Channel {
    name: &'static str,
    kind: Kind,
    edge: bool,
    dataref: Option<DataRef>,
    last: Vec<f64>,
}

const CHANNELS: &[(&str, Kind, bool)] = &[
    ("sim/time/paused", Kind::Int, true),
    ("sim/time/framerate_period", Kind::Float, false),
    ("sim/time/gpu_time_per_frame_sec_approx", Kind::Float, false),
    ("sim/operation/misc/frame_rate_period", Kind::Float, false),
    ("fbw/diag/sasl_update_ms", Kind::Float, false),
    ("sim/graphics/view/view_type", Kind::Int, true),
    ("sim/time/local_time_sec", Kind::Float, false),
    ("sim/graphics/scenery/sun_pitch_degrees", Kind::Float, false),
    ("sim/flightmodel/position/groundspeed", Kind::Float, false),
    ("sim/flightmodel/forces/g_axil", Kind::Float, false),
    ("sim/flightmodel/forces/g_side", Kind::Float, false),
    ("sim/flightmodel/forces/g_nrml", Kind::Float, false),
    ("sim/flightmodel/position/theta", Kind::Float, false),
    ("sim/flightmodel/position/phi", Kind::Float, false),
    ("sim/flightmodel/position/local_vx", Kind::Float, false),
    ("sim/flightmodel/position/local_vz", Kind::Float, false),
    ("sim/flightmodel/position/psi", Kind::Float, false),
    ("sim/flightmodel/position/y_agl", Kind::Float, false),
    ("sim/flightmodel/failures/onground_any", Kind::Int, true),
    ("sim/flightmodel2/misc/has_crashed", Kind::Int, true),
    ("sim/flightmodel/weight/m_total", Kind::Float, false),
    ("sim/cockpit2/controls/gear_handle_down", Kind::Int, true),
    ("sim/flightmodel2/gear/deploy_ratio", Kind::Floats(5), true),
    ("sim/flightmodel2/gear/tire_vertical_force_n_mtr", Kind::Floats(5), false),
    ("sim/flightmodel2/gear/tire_vertical_deflection_mtr", Kind::Floats(5), false),
    ("sim/flightmodel2/wing/wing_tip_deflection_deg", Kind::Floats(4), false),
    ("sim/flightmodel/engine/ENGN_running", Kind::Ints(4), true),
    ("sim/flightmodel2/engines/engine_is_burning_fuel", Kind::Ints(4), true),
    ("sim/flightmodel2/engines/starter_is_running", Kind::Ints(4), true),
    ("sim/cockpit2/engine/actuators/ignition_on", Kind::Ints(4), true),
    ("sim/flightmodel/engine/ENGN_N1_", Kind::Floats(4), false),
    ("sim/flightmodel/engine/ENGN_N2_", Kind::Floats(4), false),
    ("sim/flightmodel/engine/ENGN_EGT_c", Kind::Floats(4), false),
    ("sim/flightmodel/engine/ENGN_FF_", Kind::Floats(4), false),
    ("sim/flightmodel/engine/ENGN_mixt", Kind::Floats(4), true),
    ("sim/cockpit2/fuel/fuel_quantity", Kind::Floats(9), false),
    ("sim/cockpit2/switches/landing_lights_on", Kind::Int, true),
    ("sim/cockpit2/switches/taxi_light_on", Kind::Int, true),
    ("sim/cockpit2/switches/navigation_lights_on", Kind::Int, true),
    ("sim/cockpit2/switches/beacon_on", Kind::Int, true),
    ("sim/cockpit2/switches/strobe_lights_on", Kind::Int, true),
    ("sim/flightmodel2/lights/landing_lights_brightness_ratio", Kind::Floats(4), true),
    ("sim/flightmodel2/lights/nav_lights_brightness_ratio", Kind::Floats(1), true),
    ("sim/flightmodel2/lights/beacon_brightness_ratio", Kind::Floats(2), false),
    ("sim/flightmodel2/lights/strobe_brightness_ratio", Kind::Floats(2), false),
    ("sim/cockpit2/switches/generic_lights_switch", Kind::Floats(32), true),
    ("sim/flightmodel2/lights/generic_lights_brightness_ratio", Kind::Floats(32), true),
    ("sim/cockpit2/switches/panel_brightness_ratio", Kind::Floats(4), true),
    ("sim/cockpit2/switches/instrument_brightness_ratio", Kind::Floats(16), true),
    ("sim/operation/sound/sound_on", Kind::Int, true),
    ("sim/operation/sound/master_volume_ratio", Kind::Float, true),
    ("sim/operation/sound/interior_volume_ratio", Kind::Float, true),
    ("sim/operation/sound/exterior_volume_ratio", Kind::Float, true),
    ("sim/operation/sound/engine_volume_ratio", Kind::Float, true),
    ("sim/cockpit2/engine/actuators/throttle_ratio", Kind::Floats(4), false),
    ("sim/flightmodel/engine/ENGN_thro", Kind::Floats(4), false),
    ("sim/flightmodel/engine/POINT_thrust", Kind::Floats(4), false),
    ("sim/joystick/joy_mapped_axis_value", Kind::Floats(32), false),
    ("sim/cockpit2/switches/landing_lights_switch", Kind::Floats(6), true),
    ("fbw/A32NX_AUTOTHRUST_TLA_1", Kind::Float, false),
    ("fbw/A32NX_AUTOTHRUST_TLA_2", Kind::Float, false),
    ("fbw/A32NX_AUTOTHRUST_TLA_3", Kind::Float, false),
    ("fbw/A32NX_AUTOTHRUST_TLA_4", Kind::Float, false),
    ("fbw/GENERAL_ENG_STARTER_1", Kind::Float, true),
    ("fbw/GENERAL_ENG_STARTER_2", Kind::Float, true),
    ("fbw/GENERAL_ENG_STARTER_3", Kind::Float, true),
    ("fbw/GENERAL_ENG_STARTER_4", Kind::Float, true),
    ("fbw/A32NX_ENGINE_STATE_1", Kind::Float, true),
    ("fbw/A32NX_ENGINE_STATE_2", Kind::Float, true),
    ("fbw/A32NX_ENGINE_STATE_3", Kind::Float, true),
    ("fbw/A32NX_ENGINE_STATE_4", Kind::Float, true),
    ("fbw/A32NX_ADIRS_SENSED_1_GROUND_SPEED", Kind::Float, false),
    ("fbw/A32NX_ADIRS_SENSED_2_GROUND_SPEED", Kind::Float, false),
    ("fbw/A32NX_ADIRS_SENSED_3_GROUND_SPEED", Kind::Float, false),
    ("fbw/A32NX_WING_FLEX_LEFT_INBOARD", Kind::Float, false),
    ("fbw/A32NX_WING_FLEX_LEFT_INBOARD_MID", Kind::Float, false),
    ("fbw/A32NX_WING_FLEX_LEFT_OUTBOARD_MID", Kind::Float, false),
    ("fbw/A32NX_WING_FLEX_LEFT_OUTBOARD", Kind::Float, false),
    ("fbw/A32NX_FIRE_DETECTED_ENG1", Kind::Float, true),
    ("fbw/A32NX_FIRE_DETECTED_ENG2", Kind::Float, true),
    ("fbw/A32NX_FIRE_DETECTED_ENG3", Kind::Float, true),
    ("fbw/A32NX_FIRE_DETECTED_ENG4", Kind::Float, true),
    ("fbw/A32NX_FIRE_DETECTED_APU", Kind::Float, true),
    ("fbw/A32NX_FIRE_DETECTED_MLG", Kind::Float, true),
    ("fbw/A32NX_ELEC_DC_ESS_BUS_IS_POWERED", Kind::Float, true),
    ("fbw/A32NX_ELEC_DC_1_BUS_IS_POWERED", Kind::Float, true),
    ("fbw/A32NX_ELEC_DC_2_BUS_IS_POWERED", Kind::Float, true),
    ("fbw/A32NX_ELEC_AC_ESS_BUS_IS_POWERED", Kind::Float, true),
    ("fbw/A32NX_ELEC_AC_1_BUS_IS_POWERED", Kind::Float, true),
    ("fbw/A32NX_ELEC_AC_2_BUS_IS_POWERED", Kind::Float, true),
    ("fbw/cockpit/SWITCH_OVHD_EXTLT_BEACON", Kind::Float, true),
    ("fbw/cockpit/SWITCH_OVHD_EXTLT_LANDL", Kind::Float, true),
    ("fbw/cockpit/SWITCH_OVHD_EXTLT_LOGO", Kind::Float, true),
    ("fbw/cockpit/SWITCH_OVHD_EXTLT_NAVLOGO", Kind::Float, true),
    ("fbw/cockpit/SWITCH_OVHD_EXTLT_NOSE", Kind::Float, true),
    ("fbw/cockpit/SWITCH_OVHD_EXTLT_RWY", Kind::Float, true),
    ("fbw/cockpit/SWITCH_OVHD_EXTLT_STROBE", Kind::Float, true),
    ("fbw/cockpit/SWITCH_OVHD_EXTLT_WING", Kind::Float, true),
    ("fbw/LIGHT_POTENTIOMETER_7", Kind::Float, true),
    ("fbw/LIGHT_POTENTIOMETER_8", Kind::Float, true),
    ("fbw/LIGHT_POTENTIOMETER_9", Kind::Float, true),
    ("fbw/LIGHT_POTENTIOMETER_10", Kind::Float, true),
    ("fbw/LIGHT_POTENTIOMETER_11", Kind::Float, true),
    ("fbw/LIGHT_POTENTIOMETER_24", Kind::Float, true),
    ("fbw/LIGHT_POTENTIOMETER_76", Kind::Float, true),
    ("fbw/LIGHT_POTENTIOMETER_78", Kind::Float, true),
    ("fbw/LIGHT_POTENTIOMETER_79", Kind::Float, true),
    ("fbw/LIGHT_POTENTIOMETER_80", Kind::Float, true),
    ("fbw/LIGHT_POTENTIOMETER_81", Kind::Float, true),
    ("fbw/LIGHT_POTENTIOMETER_82", Kind::Float, true),
    ("fbw/LIGHT_POTENTIOMETER_83", Kind::Float, true),
    ("fbw/LIGHT_POTENTIOMETER_84", Kind::Float, true),
    ("fbw/LIGHT_POTENTIOMETER_85", Kind::Float, true),
    ("fbw/LIGHT_POTENTIOMETER_86", Kind::Float, true),
    ("fbw/LIGHT_POTENTIOMETER_87", Kind::Float, true),
    ("fbw/LIGHT_POTENTIOMETER_88", Kind::Float, true),
    ("fbw/LIGHT_POTENTIOMETER_89", Kind::Float, true),
    ("fbw/LIGHT_POTENTIOMETER_90", Kind::Float, true),
    ("fbw/LIGHT_POTENTIOMETER_91", Kind::Float, true),
    ("fbw/LIGHT_POTENTIOMETER_92", Kind::Float, true),
    ("fbw/LIGHT_POTENTIOMETER_93", Kind::Float, true),
    ("fbw/LIGHT_POTENTIOMETER_94", Kind::Float, true),
    ("fbw/LIGHT_POTENTIOMETER_95", Kind::Float, true),
    ("fbw/LIGHT_POTENTIOMETER_96", Kind::Float, true),
    ("fbw/LIGHT_POTENTIOMETER_97", Kind::Float, true),
    ("fbw/LIGHT_POTENTIOMETER_98", Kind::Float, true),
    ("fbw/LIGHT_POTENTIOMETER_99", Kind::Float, true),
    ("sim/graphics/view/view_is_external", Kind::Int, true),
    ("sim/graphics/view/pilots_head_x", Kind::Float, false),
    ("sim/graphics/view/pilots_head_y", Kind::Float, false),
    ("sim/graphics/view/pilots_head_z", Kind::Float, false),
    ("fbw/anim/c_gear", Kind::Float, false),
    ("fbw/anim/l_b_gear", Kind::Float, false),
    ("fbw/anim/l_w_gear", Kind::Float, false),
    ("fbw/anim/c_gear_door1", Kind::Float, false),
    ("fbw/anim/l_body_gear_door", Kind::Float, false),
    ("fbw/anim/l_wing_gear_door", Kind::Float, false),
    ("fbw/anim/l_blg_inner_door", Kind::Float, false),
    ("fbw/anim/l_b_gear_tilt", Kind::Float, false),
    ("fbw/anim/l_w_gear_tilt", Kind::Float, false),
    ("fbw/A32NX_GEAR_CENTER_POSITION", Kind::Float, false),
    ("fbw/A32NX_GEAR_LEFT_POSITION", Kind::Float, false),
    ("fbw/A32NX_GEAR_DOOR_CENTER_POSITION", Kind::Float, false),
    ("fbw/A32NX_GEAR_DOOR_LEFT_POSITION", Kind::Float, false),
    ("fbw/A32NX_SECONDARY_GEAR_DOOR_LEFT_POSITION", Kind::Float, false),
    ("fbw/A32NX_GEAR_1_TILT_POSITION", Kind::Float, false),
    ("fbw/A32NX_GEAR_3_TILT_POSITION", Kind::Float, false),
    ("fbw/anim/left_outboard_flex", Kind::Float, false),
    ("fbw/anim/left_outer_midboard_flex", Kind::Float, false),
    ("fbw/anim/left_inner_midboard_flex", Kind::Float, false),
    ("fbw/anim/left_inboard_flex", Kind::Float, false),
    ("fbw/A32NX_RUDDER_PEDAL_POSITION", Kind::Float, false),
    ("fbw/A32NX_FCTL_RUD_UPPER_DEFLECTION_DEG", Kind::Float, false),
    ("fbw/A32NX_FCTL_AIL_L1_DEFLECTION_DEG", Kind::Float, false),
    ("fbw/A32NX_SIDESTICK_POSITION_X", Kind::Float, false),
    ("fbw/A32NX_SIDESTICK_POSITION_Y", Kind::Float, false),
    ("fbw/cockpit/yoke_lever_stick1_l_r", Kind::Float, false),
    ("fbw/cockpit/yoke_lever_stick_fore1_aft", Kind::Float, false),
    ("fbw/cockpit/lever_speed_brake", Kind::Float, true),
    ("sim/cockpit2/controls/speedbrake_ratio", Kind::Float, true),
    ("fbw/A32NX_SPOILERS_HANDLE_POSITION", Kind::Float, false),
    ("sim/cockpit2/engine/actuators/throttle_jet_rev_ratio", Kind::Floats(4), false),
    ("sim/joystick/joy_mapped_axis_avail", Kind::Ints(32), true),
    ("fbw/A32NX_EXT_PWR_AVAIL_1", Kind::Float, true),
    ("fbw/A32NX_EXT_PWR_AVAIL_2", Kind::Float, true),
    ("fbw/A32NX_EXT_PWR_AVAIL_3", Kind::Float, true),
    ("fbw/A32NX_EXT_PWR_AVAIL_4", Kind::Float, true),
    ("fbw/A32NX_OVHD_ELEC_EXT_PWR_1_PB_IS_ON", Kind::Float, true),
    ("fbw/A32NX_OVHD_ELEC_EXT_PWR_2_PB_IS_ON", Kind::Float, true),
    ("fbw/A32NX_OVHD_ELEC_EXT_PWR_3_PB_IS_ON", Kind::Float, true),
    ("fbw/A32NX_OVHD_ELEC_EXT_PWR_4_PB_IS_ON", Kind::Float, true),
    ("fbw/A32NX_ELEC_AC_3_BUS_IS_POWERED", Kind::Float, true),
    ("fbw/A32NX_ELEC_AC_4_BUS_IS_POWERED", Kind::Float, true),
    ("fbw/A32NX_PARK_BRAKE_LEVER_POS", Kind::Float, true),
    ("fbw/A32NX_HYD_BRAKE_ALTN_ACC_PRESS", Kind::Float, false),
    ("fbw/A32NX_HYD_BRAKE_ALTN_LEFT_PRESS", Kind::Float, false),
    ("fbw/A32NX_HYD_BRAKE_NORM_LEFT_PRESS", Kind::Float, false),
    ("sim/cockpit2/controls/parking_brake_ratio", Kind::Float, true),
    ("sim/cockpit2/controls/left_brake_ratio", Kind::Float, false),
    ("fbw/A380X_OVHD_INTLT_ANN", Kind::Float, true),
    ("fbw/GLASSCOCKPIT_AUTOMATIC_BRIGHTNESS", Kind::Float, false),
    ("fbw/A380X_EFIS_L_LS_BUTTON_IS_ON", Kind::Float, true),
    ("fbw/A380X_EFIS_L_TRAF_BUTTON_IS_ON", Kind::Float, true),
    ("fbw/A380X_EFIS_L_CSTR_BUTTON_IS_ON", Kind::Float, true),
    ("fbw/A380X_EFIS_L_ARPT_BUTTON_IS_ON", Kind::Float, true),
    ("fbw/A380X_EFIS_L_VV_BUTTON_IS_ON", Kind::Float, true),
    ("fbw/cockpit/lt/PUSH_EFIS_CS_LS", Kind::Float, false),
    ("fbw/cockpit/lt/PUSH_EFIS_CS_TRAF", Kind::Float, false),
    ("fbw/cockpit/lt/PUSH_EFIS_CS_ARPT", Kind::Float, false),
    ("fbw/SEAT_CPT_MOVE_FWD_AFT", Kind::Float, true),
    ("fbw/SEAT_CPT_MOVE_UP_DOWN", Kind::Float, true),
];

pub struct Sampler {
    channels: Vec<Channel>,
    next: Instant,
    frames: u64,
    header: bool,
    last_find: Instant,
    tour: PhotoTour,
    shots_seen: std::collections::HashSet<String>,
    shots_scanned: Option<Instant>,
    pending_shot: Option<(PathBuf, Instant)>,
    clicks: ClickLog,
    auto_screens: Instant,
}

const SCREENSHOTS: &str = "Output/screenshots";
const SCREENSHOT_WAIT: Duration = Duration::from_secs(10);

fn screenshot_files() -> Vec<(String, SystemTime)> {
    let Ok(entries) = std::fs::read_dir(SCREENSHOTS) else { return Vec::new() };
    entries
        .filter_map(Result::ok)
        .filter_map(|e| {
            let meta = e.metadata().ok()?;
            meta.is_file().then(|| (e.file_name().to_string_lossy().into_owned(), meta.modified().unwrap_or(UNIX_EPOCH)))
        })
        .collect()
}

#[derive(Default)]
struct ClickLog {
    registered: usize,
    attempts: u32,
}

static CLICK_NAMES: OnceLock<Vec<String>> = OnceLock::new();

unsafe extern "C" fn click_seen(_command: crate::xp::CommandRef, phase: std::ffi::c_int, refcon: *mut c_void) -> std::ffi::c_int {
    if phase == 0 {
        if let Some(name) = CLICK_NAMES.get().and_then(|n| n.get(refcon as usize)) {
            event("click", name);
        }
    }
    1
}

fn cockpit_commands(main_lua: &str) -> Vec<String> {
    let opening = "command(\"fbw/cockpit/";
    let mut out = Vec::new();
    let mut rest = main_lua;
    while let Some(i) = rest.find(opening) {
        rest = &rest[i + "command(\"".len()..];
        if let Some(end) = rest.find('"') {
            out.push(rest[..end].to_string());
            rest = &rest[end..];
        }
    }
    out.sort();
    out.dedup();
    out
}

impl ClickLog {
    fn tick(&mut self, xplm: &Xplm) {
        let due = match self.attempts {
            0 => elapsed() > 20.0,
            1 => elapsed() > 90.0,
            _ => false,
        };
        if !due || self.registered > 0 {
            return;
        }
        self.attempts += 1;
        let Some(lua) = aircraft_dir().map(|d| d.join("plugins/sasl/data/modules/main.lua")).and_then(|p| std::fs::read_to_string(p).ok()) else { return };
        let names = CLICK_NAMES.get_or_init(|| cockpit_commands(&lua));
        for (i, name) in names.iter().enumerate() {
            if let Some(command) = crate::xp::find_command(name) {
                xplm.register_command_handler(command, click_seen, i as *mut c_void);
                self.registered += 1;
            }
        }
        event("click", &format!("watching {} of {} cockpit commands", self.registered, names.len()));
    }
}

impl Sampler {
    pub fn new() -> Self {
        let channels = CHANNELS.iter().map(|&(name, kind, edge)| Channel { name, kind, edge, dataref: None, last: Vec::new() }).collect();
        let shots_seen = screenshot_files().into_iter().map(|(name, _)| name).collect();
        Self {
            channels,
            next: Instant::now(),
            frames: 0,
            header: false,
            last_find: Instant::now() - Duration::from_secs(10),
            tour: PhotoTour::default(),
            shots_seen,
            shots_scanned: None,
            pending_shot: None,
            clicks: ClickLog::default(),
            auto_screens: Instant::now(),
        }
    }

    fn header(&self) -> String {
        let mut cols = vec!["t".to_string(), "frames".into(), "fps".into(), "plugin_tick_ms".into(), "plugin_draw_ms".into(), "dataref_reads_per_frame".into()];
        for c in &self.channels {
            match c.kind {
                Kind::Floats(n) | Kind::Ints(n) => cols.extend((0..n).map(|i| format!("{}[{i}]", c.name))),
                _ => cols.push(c.name.to_string()),
            }
        }
        cols.join(",")
    }

    pub fn frame(&mut self, xplm: &Xplm) {
        if !active() {
            return;
        }
        self.frames += 1;
        self.tour.tick(xplm);
        if TOUR_REQUESTED.swap(false, Ordering::Relaxed) {
            self.tour.start();
        }
        if SCREENS_REQUESTED.swap(false, Ordering::Relaxed) {
            dump_screens();
        }
        if SAVE_REQUESTED.swap(false, Ordering::Relaxed) {
            save_bundle(false);
        }
        self.clicks.tick(xplm);
        self.take_captures(xplm);
        self.watch_screenshots();
        if self.auto_screens.elapsed() >= AUTO_SCREENS_EVERY {
            self.auto_screens = Instant::now();
            dump_changed_screens();
        }
        let now = Instant::now();
        if now < self.next {
            return;
        }
        let span = SAMPLE_EVERY.as_secs_f64() + now.duration_since(self.next).as_secs_f64();
        self.next = now + SAMPLE_EVERY;
        if self.last_find.elapsed() >= Duration::from_secs(5) {
            self.last_find = Instant::now();
            for c in self.channels.iter_mut().filter(|c| c.dataref.is_none()) {
                c.dataref = xplm.find(c.name);
            }
        }
        if !self.header {
            self.header = true;
            send(Stream::Samples, self.header());
        }
        let frames = self.frames.max(1) as f64;
        let tick_ms = TICK_NS.swap(0, Ordering::Relaxed) as f64 / 1e6 / frames;
        let draw_ms = DRAW_NS.swap(0, Ordering::Relaxed) as f64 / 1e6 / frames;
        let reads = DATAREF_READS.swap(0, Ordering::Relaxed) as f64 / frames;
        let mut row = vec![format!("{:.2}", elapsed()), format!("{}", self.frames), format!("{:.1}", self.frames as f64 / span), format!("{tick_ms:.2}"), format!("{draw_ms:.2}"), format!("{reads:.0}")];
        self.frames = 0;
        let mut edges = Vec::new();
        for c in &mut self.channels {
            let values = read_channel(xplm, c);
            if c.edge && !c.last.is_empty() {
                for (i, (a, b)) in c.last.iter().zip(&values).enumerate() {
                    if (a - b).abs() > 1e-3 {
                        edges.push(match c.kind {
                            Kind::Floats(_) | Kind::Ints(_) => format!("{}[{i}] {a} -> {b}", c.name),
                            _ => format!("{} {a} -> {b}", c.name),
                        });
                    }
                }
            }
            row.extend(values.iter().map(|v| if v.is_nan() { String::new() } else { format!("{v:.4}").trim_end_matches('0').trim_end_matches('.').to_string() }));
            c.last = values;
        }
        send(Stream::Samples, row.join(","));
        for e in edges {
            event("xplane", &e);
        }
    }
}

impl Sampler {
    fn take_captures(&mut self, xplm: &Xplm) {
        let queued = CAPTURES.lock().map(|mut q| std::mem::take(&mut *q)).unwrap_or_default();
        let Some(root) = dir() else { return };
        for c in queued {
            for ch in self.channels.iter_mut().filter(|ch| ch.dataref.is_none()) {
                ch.dataref = xplm.find(ch.name);
            }
            let folder = root.join("captures").join(&c.folder);
            if std::fs::create_dir_all(&folder).is_err() {
                continue;
            }
            let mut datarefs = String::new();
            for ch in &self.channels {
                let values = read_channel(xplm, ch);
                let text: Vec<String> = values.iter().map(|v| if v.is_nan() { "-".into() } else { format!("{v}") }).collect();
                datarefs.push_str(&format!("{} = {}\n", ch.name, text.join(" ")));
            }
            let vars = crate::snapshot()
                .lock()
                .map(|snap| snap.names.iter().zip(&snap.values).map(|(n, v)| format!("{n} = {v}\n")).collect::<String>())
                .unwrap_or_default();
            let note = format!("{}\nelapsed {:.1} s, {}\nstep: {}\n", c.label, elapsed(), wall_clock(), current_step());
            let files = folder.clone();
            std::thread::Builder::new()
                .name("fbw-diag-capture".into())
                .spawn(move || {
                    let _ = std::fs::write(files.join("note.txt"), note);
                    let _ = std::fs::write(files.join("datarefs.txt"), datarefs);
                    let _ = std::fs::write(files.join("variables.txt"), vars);
                })
                .ok();
            dump_screens_into(folder.join("screens"));
            if let Some(file) = &c.from_file {
                let _ = std::fs::copy(file, folder.join(file.file_name().unwrap_or_default()));
            } else if c.screenshot {
                crate::xp::command_once("sim/operation/screenshot");
                self.pending_shot = Some((folder, Instant::now()));
            }
        }
    }

    fn watch_screenshots(&mut self) {
        if self.shots_scanned.is_some_and(|t| t.elapsed() < Duration::from_millis(700)) {
            return;
        }
        self.shots_scanned = Some(Instant::now());
        if self.pending_shot.as_ref().is_some_and(|(_, at)| at.elapsed() > SCREENSHOT_WAIT) {
            self.pending_shot = None;
        }
        let settled = SystemTime::now() - Duration::from_millis(800);
        for (name, modified) in screenshot_files() {
            if modified > settled || self.shots_seen.contains(&name) {
                continue;
            }
            self.shots_seen.insert(name.clone());
            let path = Path::new(SCREENSHOTS).join(&name);
            match self.pending_shot.take() {
                Some((folder, _)) => {
                    let _ = std::fs::copy(&path, folder.join(&name));
                    event("screenshot", &format!("{name} -> {}", folder.display()));
                }
                None => {
                    queue_capture(&format!("screenshot {name}"), false, Some(path));
                }
            }
        }
    }
}

impl Default for Sampler {
    fn default() -> Self {
        Self::new()
    }
}

fn read_channel(xplm: &Xplm, c: &Channel) -> Vec<f64> {
    let n = match c.kind {
        Kind::Floats(n) | Kind::Ints(n) => n,
        _ => 1,
    };
    let Some(d) = c.dataref else { return vec![f64::NAN; n] };
    match c.kind {
        Kind::Float => vec![xplm.get_f(d) as f64],
        Kind::Int => vec![xplm.get_i(d) as f64],
        Kind::Floats(n) => {
            let mut out = vec![0f32; n];
            let got = xplm.get_vf(d, &mut out);
            out.iter().enumerate().map(|(i, v)| if i < got { *v as f64 } else { f64::NAN }).collect()
        }
        Kind::Ints(n) => {
            let mut out = vec![0; n];
            let got = xplm.get_vi(d, &mut out);
            out.iter().enumerate().map(|(i, v)| if i < got { *v as f64 } else { f64::NAN }).collect()
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Spot {
    pub name: &'static str,
    pub interior: bool,
    pub eye: [f64; 3],
    pub look: [f64; 3],
}

pub const SPOTS: &[Spot] = &[
    Spot { name: "wing-left-top", interior: false, eye: [-18.0, 10.0, 16.0], look: [-30.0, 1.0, 4.0] },
    Spot { name: "wing-right-top", interior: false, eye: [18.0, 10.0, 16.0], look: [30.0, 1.0, 4.0] },
    Spot { name: "side-left-low", interior: false, eye: [-60.0, 0.0, 0.0], look: [0.0, 0.0, 0.0] },
    Spot { name: "front-low", interior: false, eye: [0.0, 1.0, -90.0], look: [0.0, 1.0, 0.0] },
    Spot { name: "tailcone-apu", interior: false, eye: [6.0, 6.0, 52.0], look: [0.0, 6.0, 36.0] },
    Spot { name: "rear-left-quarter", interior: false, eye: [-40.0, 8.0, 70.0], look: [0.0, 3.0, 0.0] },
    Spot { name: "top-down", interior: false, eye: [0.0, 90.0, 0.1], look: [0.0, 0.0, 0.0] },
    Spot { name: "cockpit-forward", interior: true, eye: [0.0, 0.0, 0.0], look: [0.25, -0.45, -1.0] },
    Spot { name: "cockpit-pedestal", interior: true, eye: [0.27, 0.09, 0.37], look: [0.52, -0.81, -0.53] },
    Spot { name: "cockpit-overhead", interior: true, eye: [0.0, 0.0, 0.0], look: [0.4, 1.0, -0.5] },
];

const PILOT_EYE: [f64; 3] = [-0.52, 2.30, -29.63];

pub fn camera(spot: &Spot, plane: [f64; 3], psi_deg: f64, head: [f64; 3]) -> CameraPosition {
    let (s, c) = psi_deg.to_radians().sin_cos();
    let base = if spot.interior { head } else { [0.0; 3] };
    let world = |p: [f64; 3]| {
        let p = [p[0] + base[0], p[1] + base[1], p[2] + base[2]];
        [plane[0] + p[0] * c - p[2] * s, plane[1] + p[1], plane[2] + p[0] * s + p[2] * c]
    };
    let (eye, look) = (world(spot.eye), world(spot.look));
    let (dx, dy, dz) = (look[0] - eye[0], look[1] - eye[1], look[2] - eye[2]);
    CameraPosition {
        x: eye[0] as f32,
        y: eye[1] as f32,
        z: eye[2] as f32,
        pitch: dy.atan2(dx.hypot(dz)).to_degrees() as f32,
        heading: dx.atan2(-dz).to_degrees().rem_euclid(360.0) as f32,
        roll: 0.0,
        zoom: 1.0,
    }
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct CameraPosition {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub pitch: f32,
    pub heading: f32,
    pub roll: f32,
    pub zoom: f32,
}

static CAMERA: Mutex<Option<CameraPosition>> = Mutex::new(None);

unsafe extern "C" fn camera_callback(out: *mut CameraPosition, losing: i32, _refcon: *mut c_void) -> i32 {
    if losing != 0 {
        return 0;
    }
    match CAMERA.lock().ok().and_then(|c| *c) {
        Some(p) if !out.is_null() => {
            *out = p;
            1
        }
        Some(_) => 1,
        None => 0,
    }
}

struct TourState {
    index: usize,
    since: Instant,
    shot: bool,
    view_set: bool,
    switching: bool,
    head: Option<[f64; 3]>,
}

static TOUR_STATE: Mutex<Option<TourState>> = Mutex::new(None);

#[derive(Default)]
struct PhotoTour {
    refs: Option<[Option<DataRef>; 7]>,
}

impl PhotoTour {
    fn start(&mut self) {
        if let Ok(mut t) = TOUR_STATE.lock() {
            *t = Some(TourState { index: 0, since: Instant::now(), shot: false, view_set: false, switching: false, head: None });
        }
        event("photo-tour", "started");
    }

    fn tick(&mut self, xplm: &Xplm) {
        let Ok(mut guard) = TOUR_STATE.lock() else { return };
        let Some(state) = guard.as_mut() else { return };
        let refs = *self.refs.get_or_insert_with(|| {
            [
                "sim/flightmodel/position/local_x",
                "sim/flightmodel/position/local_y",
                "sim/flightmodel/position/local_z",
                "sim/flightmodel/position/psi",
                "sim/graphics/view/pilots_head_x",
                "sim/graphics/view/pilots_head_y",
                "sim/graphics/view/pilots_head_z",
            ]
            .map(|n| xplm.find(n))
        });
        let Some(spot) = SPOTS.get(state.index) else {
            crate::xp::dont_control_camera();
            if let Ok(mut c) = CAMERA.lock() {
                *c = None;
            }
            crate::xp::command_once("sim/view/3d_cockpit_cmnd_look");
            event("photo-tour", "done; screenshots are in Output/screenshots and get copied into the bundle");
            *guard = None;
            return;
        };
        if !state.view_set {
            let needs_switch = state.index == 0 || SPOTS[state.index - 1].interior != spot.interior;
            if needs_switch {
                if !state.switching {
                    crate::xp::dont_control_camera();
                    crate::xp::command_once(if spot.interior { "sim/view/3d_cockpit_cmnd_look" } else { "sim/view/chase" });
                    state.switching = true;
                    state.since = Instant::now();
                    return;
                }
                if state.since.elapsed() < Duration::from_millis(600) {
                    return;
                }
                if spot.interior {
                    let head = [refs[4], refs[5], refs[6]].map(|d| d.map(|d| xplm.get_f(d) as f64));
                    state.head = match head {
                        [Some(x), Some(y), Some(z)] if z.abs() > 1.0 => Some([x, y, z]),
                        _ => None,
                    };
                }
                crate::xp::control_camera(camera_callback);
                state.switching = false;
            }
            state.view_set = true;
            state.since = Instant::now();
        }
        let read = |i: usize| refs[i].map_or(0.0, |d| if i < 3 { xplm.get_d(d) } else { xplm.get_f(d) as f64 });
        if let Ok(mut c) = CAMERA.lock() {
            *c = Some(camera(spot, [read(0), read(1), read(2)], read(3), state.head.unwrap_or(PILOT_EYE)));
        }
        if !state.shot && state.since.elapsed() >= Duration::from_millis(2500) {
            crate::xp::command_once("sim/operation/screenshot");
            event("photo-tour", &format!("screenshot {} ({})", state.index + 1, spot.name));
            state.shot = true;
        }
        if state.shot && state.since.elapsed() >= Duration::from_millis(3300) {
            state.index += 1;
            state.since = Instant::now();
            state.shot = false;
            state.view_set = SPOTS.get(state.index).is_some_and(|next| next.interior == spot.interior);
        }
    }
}

#[cfg(feature = "js")]
fn dump_screens() {
    let Some(dir) = dir() else { return };
    let n = MARKS.load(Ordering::Relaxed);
    dump_screens_into(dir.join("screens").join(format!("{:03}_{:.0}s", n, elapsed())));
}

const AUTO_SCREENS_EVERY: Duration = Duration::from_secs(30);

#[cfg(feature = "js")]
fn dump_changed_screens() {
    use std::hash::{Hash, Hasher};
    static LAST: Mutex<Option<HashMap<String, u64>>> = Mutex::new(None);
    let Some(dir) = dir() else { return };
    let shots = crate::display::snapshot_screens();
    let target = dir.join("screens").join(format!("auto_{:05.0}s", elapsed()));
    std::thread::Builder::new()
        .name("fbw-diag-auto-screens".into())
        .spawn(move || {
            let Ok(mut last) = LAST.lock() else { return };
            let last = last.get_or_insert_with(HashMap::new);
            let changed: Vec<_> = shots
                .into_iter()
                .filter(|(id, _, _, bgra)| {
                    let mut h = std::collections::hash_map::DefaultHasher::new();
                    bgra.hash(&mut h);
                    let digest = h.finish();
                    last.insert(id.clone(), digest) != Some(digest)
                })
                .collect();
            if !changed.is_empty() {
                write_pngs(&target, changed);
            }
        })
        .ok();
}

#[cfg(not(feature = "js"))]
fn dump_changed_screens() {}

#[cfg(feature = "js")]
fn write_pngs(target: &Path, shots: Vec<(String, u32, u32, Vec<u8>)>) {
    if std::fs::create_dir_all(target).is_err() {
        return;
    }
    for (id, width, height, mut bgra) in shots {
        for px in bgra.chunks_exact_mut(4) {
            px.swap(0, 2);
        }
        write_png_rgba(&target.join(format!("{id}.png")), width, height, &bgra);
    }
}

#[cfg(feature = "js")]
pub(crate) fn write_png_rgba(path: &Path, width: u32, height: u32, rgba: &[u8]) {
    let Ok(file) = std::fs::File::create(path) else { return };
    let mut encoder = png::Encoder::new(std::io::BufWriter::new(file), width, height);
    encoder.set_color(png::ColorType::Rgba);
    encoder.set_depth(png::BitDepth::Eight);
    if let Ok(mut w) = encoder.write_header() {
        let _ = w.write_image_data(rgba);
    }
}

#[cfg(not(feature = "js"))]
pub(crate) fn write_png_rgba(_path: &Path, _width: u32, _height: u32, _rgba: &[u8]) {}

#[cfg(feature = "js")]
fn dump_screens_into(target: PathBuf) {
    crate::display::request_device_grabs(target.clone());
    let shots = crate::display::snapshot_screens();
    std::thread::Builder::new().name("fbw-diag-screens".into()).spawn(move || write_pngs(&target, shots)).ok();
}

#[cfg(not(feature = "js"))]
fn dump_screens() {}

#[cfg(not(feature = "js"))]
fn dump_screens_into(_target: PathBuf) {}

fn copy_newer(from: &Path, to: &Path, since: SystemTime, filter: &dyn Fn(&str) -> bool) -> usize {
    let Ok(entries) = std::fs::read_dir(from) else { return 0 };
    let mut n = 0;
    for e in entries.filter_map(Result::ok) {
        let path = e.path();
        let name = e.file_name().to_string_lossy().into_owned();
        let Ok(meta) = e.metadata() else { continue };
        if meta.is_dir() {
            n += copy_newer(&path, &to.join(&name), since, filter);
        } else if filter(&name) && meta.modified().is_ok_and(|m| m >= since) {
            if std::fs::create_dir_all(to).is_ok() && std::fs::copy(&path, to.join(&name)).is_ok() {
                n += 1;
            }
        }
    }
    n
}

pub fn save_bundle(compress: bool) {
    let Some(s) = session() else { return };
    let since = s.started_wall - Duration::from_secs(5);
    let collected = s.dir.join("collected");
    let _ = std::fs::create_dir_all(&collected);
    let mut copied = 0;
    for f in ["Log.txt"] {
        if std::fs::copy(f, collected.join(f)).is_ok() {
            copied += 1;
        }
    }
    copied += copy_newer(Path::new("Output/screenshots"), &collected.join("screenshots"), since, &|_| true);
    copied += copy_newer(Path::new("Output/crash_reports"), &collected.join("crash_reports"), since, &|_| true);
    copied += copy_newer(Path::new("Output/preferences"), &collected.join("preferences"), SystemTime::UNIX_EPOCH, &|n| n.starts_with("fbw_a380x") || n.starts_with("xphfbw"));
    if let Ok(local) = std::env::var("LOCALAPPDATA") {
        let xphfbw = Path::new(&local).join("XPHFBW");
        copied += copy_newer(&xphfbw.join("logs"), &collected.join("xphfbw-logs"), since, &|_| true);
        copied += copy_newer(&xphfbw, &collected.join("xphfbw"), since, &|n| n.ends_with(".log") || n.ends_with(".json") || n.ends_with(".dmp") || n.ends_with(".txt"));
    }
    copied += copy_newer(&crate::state_dump::root(), &collected.join("state-dumps"), since, &|_| true);
    if let Some(aircraft) = aircraft_dir() {
        copied += copy_newer(&aircraft.join("plugins/sasl/data/output"), &collected.join("sasl"), since, &|_| true);
        copied += copy_newer(&aircraft, &collected.join("aircraft"), SystemTime::UNIX_EPOCH, &|n| n.ends_with("airframe.json") || n == "fbw_xp_installed_version.txt");
    }
    event("bundle", &format!("{copied} files collected into {}", collected.display()));
    let _ = std::fs::write(s.dir.join("status.json"), status_json());
    let _ = std::fs::write(s.dir.join("health.txt"), health_text());
    if compress {
        let source = s.dir.display().to_string();
        let zip = format!("{source}.zip");
        let script = format!("Start-Sleep -Seconds 4; Compress-Archive -Path '{source}\\*' -DestinationPath '{zip}' -Force");
        let mut command = std::process::Command::new("powershell");
        command.args(["-NoProfile", "-WindowStyle", "Hidden", "-Command", &script]);
        {
            use std::os::windows::process::CommandExt;
            command.creation_flags(0x0800_0000 | 0x0000_0008);
        }
        let _ = command.spawn();
    }
}

pub fn finish() {
    if active() {
        event("session", "X-Plane is unloading the plugin");
        save_bundle(true);
        std::thread::sleep(Duration::from_millis(300));
    }
}

unsafe extern "C" fn diag_command(_command: crate::xp::CommandRef, phase: std::ffi::c_int, refcon: *mut c_void) -> std::ffi::c_int {
    if phase == 0 {
        match refcon as usize {
            0 => {
                mark("");
            }
            1 => request_photo_tour(),
            2 => request_screens(),
            _ => request_save(),
        }
    }
    1
}

unsafe extern "C" fn active_dataref(_refcon: *mut c_void) -> std::ffi::c_int {
    std::ffi::c_int::from(active())
}

static ACTIVE_NAME: OnceLock<std::ffi::CString> = OnceLock::new();

pub fn register_commands(xplm: &Xplm) {
    let name = ACTIVE_NAME.get_or_init(|| std::ffi::CString::new("fbw/diag/active").unwrap_or_default());
    let accessors = crate::xp::Accessors { get_i: Some(active_dataref), set_i: None, get_f: None, set_f: None, get_d: None, set_d: None, get_b: None, set_b: None, get_vf: None };
    let _ = crate::xp::register_accessor(name, crate::xp::DATA_TYPE_INT, false, &accessors, std::ptr::null_mut());
    for (i, (name, description)) in [
        ("fbw/diag/mark", "FlyByWire diagnostics: capture (screenshot, screens and every value)"),
        ("fbw/diag/photo_tour", "FlyByWire diagnostics: photo tour of the aircraft"),
        ("fbw/diag/screens", "FlyByWire diagnostics: save every screen as PNG"),
        ("fbw/diag/save", "FlyByWire diagnostics: collect the bundle now"),
    ]
    .into_iter()
    .enumerate()
    {
        if let Some(command) = xplm.create_command(name, description) {
            xplm.register_command_handler(command, diag_command, i as *mut c_void);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn cockpit_click_commands_are_found_in_the_lua_and_captures_get_tidy_folder_names() {
        let lua = "command(\"fbw/cockpit/PUSH_OVHD_ELEC_EXTPWR3_click\", \"x\", f)\ncommand(\"fbw/cockpit/KNOB_A_up\", \"y\", up)\ncommand(\"fbw/cockpit/KNOB_A_up\", \"y\", up)\nlook(\"fbw/cockpit/other\")";
        assert_eq!(cockpit_commands(lua), vec!["fbw/cockpit/KNOB_A_up".to_string(), "fbw/cockpit/PUSH_OVHD_ELEC_EXTPWR3_click".to_string()]);
        assert_eq!(slug("6. EXT PWR 1 to 4 - unit 3 pressed!"), "6-ext-pwr-1-to-4-unit-3-pressed");
    }

    #[test]
    fn the_preferences_switch_reads_like_the_other_inis() {
        assert!(enabled_in("enabled=1\n"));
        assert!(enabled_in("; on for the next launch\nenabled = yes\n"));
        assert!(!enabled_in("enabled=0\n"));
        assert!(!enabled_in(""));
    }

    #[test]
    fn session_folders_are_named_by_utc_start_time() {
        assert_eq!(stamp(UNIX_EPOCH + Duration::from_secs(1_791_547_200)), "2026-10-09_12-00-00Z");
        assert_eq!(stamp(UNIX_EPOCH), "1970-01-01_00-00-00Z");
    }

    #[test]
    fn a_camera_spot_turns_with_the_aircraft_and_looks_at_its_target() {
        let spot = Spot { name: "t", interior: false, eye: [0.0, 0.0, 10.0], look: [0.0, 0.0, 0.0] };
        let north = camera(&spot, [100.0, 5.0, 200.0], 0.0, PILOT_EYE);
        assert_eq!((north.x, north.y, north.z), (100.0, 5.0, 210.0), "behind the tail is south of a north-facing aircraft");
        assert!((north.heading - 0.0).abs() < 1e-3 && north.pitch.abs() < 1e-3);
        let east = camera(&spot, [100.0, 5.0, 200.0], 90.0, PILOT_EYE);
        assert!((east.x - 90.0).abs() < 1e-3 && (east.z - 200.0).abs() < 1e-3, "behind the tail is west of an east-facing aircraft: {east:?}");
        assert!((east.heading - 90.0).abs() < 1e-3);
        let down = camera(&Spot { name: "d", interior: false, eye: [0.0, 50.0, 0.0], look: [0.0, 0.0, 0.0] }, [0.0; 3], 0.0, PILOT_EYE);
        assert!((down.pitch + 90.0).abs() < 1e-3);
    }

    #[test]
    fn cockpit_shots_start_at_the_pilots_own_eye_point() {
        let forward = SPOTS.iter().find(|s| s.name == "cockpit-forward").unwrap();
        let cam = camera(forward, [100.0, 5.0, 200.0], 0.0, [-0.52, 2.30, -29.63]);
        assert!((cam.x - 99.48).abs() < 1e-3 && (cam.y - 7.30).abs() < 1e-3 && (cam.z - 170.37).abs() < 1e-3, "{cam:?}");
        assert!(cam.pitch < 0.0, "looks down at the panel: {cam:?}");
    }

    #[test]
    fn every_test_step_has_a_title_and_instructions_and_the_plan_lists_them_all() {
        let plan = test_plan();
        for (title, detail) in STEPS {
            assert!(!title.is_empty() && !detail.is_empty());
            assert!(plan.contains(title));
        }
    }

    #[test]
    fn the_csv_header_has_a_column_for_every_array_element() {
        let s = Sampler::new();
        let header = s.header();
        assert!(header.starts_with("t,frames,fps,plugin_tick_ms"));
        assert!(header.contains("sim/flightmodel/engine/ENGN_running[3]"));
        assert!(!header.contains("sim/flightmodel/engine/ENGN_running[4]"));
        let expected: usize = 6 + CHANNELS.iter().map(|(_, k, _)| match k { Kind::Floats(n) | Kind::Ints(n) => *n, _ => 1 }).sum::<usize>();
        assert_eq!(header.split(',').count(), expected);
    }

    #[test]
    fn process_stats_finds_the_running_test_process_by_name() {
        let exe = std::env::current_exe().unwrap();
        let name = exe.file_name().unwrap().to_string_lossy().into_owned();
        let stats = process_stats(&[name.as_str()]);
        assert_eq!(stats[0].1, 1);
        assert!(stats[0].3 > 0, "working set measured");
    }
}
