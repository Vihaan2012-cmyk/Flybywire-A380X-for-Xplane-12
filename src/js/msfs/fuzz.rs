use super::*;
use std::collections::BTreeMap;
use std::io::Write;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;

const TICK_MS: f64 = 50.;

fn env_required(name: &str) -> String {
    std::env::var(name).unwrap_or_else(|_| panic!("{name} must be set"))
}

fn env_or<T: std::str::FromStr>(name: &str, default: T) -> T {
    std::env::var(name).ok().and_then(|s| s.parse().ok()).unwrap_or(default)
}

fn mix(mut x: u64) -> u64 {
    x = x.wrapping_add(0x9E37_79B9_7F4A_7C15);
    x = (x ^ (x >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
    x = (x ^ (x >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
    x ^ (x >> 31)
}

fn name_hash(name: &str) -> u64 {
    let mut h = 0xCBF2_9CE4_8422_2325u64;
    for b in name.bytes() {
        h = (h ^ b as u64).wrapping_mul(0x0000_0100_0000_01B3);
    }
    h
}

fn unit_fraction(h: u64) -> f64 {
    (h >> 11) as f64 / (1u64 << 53) as f64
}

struct Rng(u64);

impl Rng {
    fn new(seed: u64) -> Self {
        Rng(mix(seed ^ 0xA5A5_5A5A_1234_5678) | 1)
    }
    fn next(&mut self) -> u64 {
        let mut x = self.0;
        x ^= x << 13;
        x ^= x >> 7;
        x ^= x << 17;
        self.0 = x;
        x.wrapping_mul(0x2545_F491_4F6C_DD1D)
    }
    fn frac(&mut self) -> f64 {
        unit_fraction(self.next())
    }
}

fn round2(v: f64) -> f64 {
    (v * 100.).round() / 100.
}

fn value_for_unit(unit: &str, h: u64) -> f64 {
    let u = unit.to_ascii_lowercase();
    let r = unit_fraction(mix(h));
    let span = |lo: f64, hi: f64| round2(lo + r * (hi - lo));
    match u.as_str() {
        "bool" | "boolean" => ((h >> 40) & 1) as f64,
        "enum" | "number" => ((h >> 40) % 16) as f64,
        _ if u.contains("knot") => span(0., 400.),
        _ if u.contains("feet") => span(-1000., 45000.),
        _ if u.contains("degree") => span(0., 360.),
        _ if u.contains("percent over 100") => span(0., 1.),
        _ if u.contains("percent") => span(0., 100.),
        _ => {
            if (h >> 41) & 1 == 0 {
                span(-1., 1.)
            } else {
                span(0., 100.)
            }
        }
    }
}

#[derive(PartialEq)]
enum Handle {
    Gear,
    Flaps,
}

fn handle_kind(name: &str) -> Option<Handle> {
    let u = name.to_ascii_uppercase();
    if u.contains("GEAR_HANDLE") || u.contains("GEAR HANDLE") {
        Some(Handle::Gear)
    } else if u.contains("FLAPS_HANDLE") || u.contains("FLAPS HANDLE") {
        Some(Handle::Flaps)
    } else {
        None
    }
}

const BUSES: [&str; 14] = ["AC_1", "AC_2", "AC_3", "AC_4", "AC_ESS", "DC_1", "DC_2", "DC_ESS", "DC_HOT_1", "DC_HOT_2", "AC_ESS_SHED", "DC_ESS_SHED", "247PP", "108PH"];

fn is_pinned(name: &str) -> bool {
    name.starts_with("K:")
        || name.starts_with("E:")
        || name.starts_with("H:")
        || (name.starts_with("L:A32NX_ELEC_") && name.ends_with("_BUS_IS_POWERED"))
        || name == "L:A32NX_FWC_FLIGHT_PHASE"
        || name == "L:A32NX_COLD_AND_DARK_SPAWN"
        || (name.starts_with("L:A32NX_ENGINE_N2:") && name.len() == "L:A32NX_ENGINE_N2:1".len())
        || name.starts_with("LIGHT POTENTIOMETER")
        || handle_kind(name).is_some()
}

struct Store {
    vars: BTreeMap<String, f64>,
    strings: BTreeMap<String, String>,
    units: BTreeMap<String, String>,
    writes: Vec<(String, f64)>,
    errors: Vec<String>,
    now_ms: f64,
}

impl Host for Store {
    fn get_var(&mut self, name: &str, unit: &str) -> f64 {
        if !self.units.contains_key(name) {
            self.units.insert(name.to_string(), unit.to_string());
        }
        match name {
            "E:ABSOLUTE TIME" => 63_895_000_000. + self.now_ms / 1000.,
            "E:SIMULATION TIME" => 100. + self.now_ms / 1000.,
            _ => *self.vars.get(name).unwrap_or(&0.),
        }
    }
    fn set_var(&mut self, name: &str, _unit: &str, value: f64) {
        self.writes.push((name.to_string(), value));
        self.vars.insert(name.to_string(), value);
    }
    fn get_string(&mut self, name: &str) -> String {
        self.strings.get(name).cloned().unwrap_or_default()
    }
    fn set_string(&mut self, name: &str, value: &str) {
        self.strings.insert(name.to_string(), value.to_string());
    }
    fn log(&mut self, level: LogLevel, message: &str) {
        if level == LogLevel::Error {
            self.errors.push(message.lines().next().unwrap_or("").chars().take(200).collect());
        }
    }
}

fn json_number(v: f64) -> serde_json::Value {
    if v.is_finite() {
        serde_json::json!(v)
    } else {
        serde_json::Value::String(format!("{v}"))
    }
}

fn apply_inputs(host: &mut Store, seed: u64, tick: u64, rate: f64) {
    for bus in BUSES {
        host.vars.insert(format!("L:A32NX_ELEC_{bus}_BUS_IS_POWERED"), 1.);
    }
    let threshold = (rate * (1u64 << 24) as f64) as u64;
    let mut changes: Vec<(String, f64)> = Vec::new();
    for (name, unit) in &host.units {
        if is_pinned(name) {
            continue;
        }
        let h = mix(seed ^ mix(tick.wrapping_mul(0x9E37_79B9) ^ name_hash(name)));
        if (h >> 8) & 0xFF_FFFF < threshold {
            changes.push((name.clone(), value_for_unit(unit, mix(h))));
        }
    }
    let ms = tick * TICK_MS as u64;
    let sweep = |period: u64| ms % period == 0 && ms > 0;
    if sweep(4000) {
        host.vars.insert("L:A32NX_FWC_FLIGHT_PHASE".into(), ((ms / 4000) % 12 + 1) as f64);
    }
    if sweep(5000) {
        for engine in 1..=4 {
            let name = format!("L:A32NX_ENGINE_N2:{engine}");
            let h = mix(seed ^ mix(ms ^ name_hash(&name)));
            host.vars.insert(name, round2(unit_fraction(h) * 110.));
        }
    }
    if sweep(6000) {
        let names: Vec<(String, Handle)> = host.units.keys().filter_map(|n| handle_kind(n).map(|k| (n.clone(), k))).collect();
        for (name, kind) in names {
            let h = mix(seed ^ mix(ms ^ name_hash(&name)));
            host.vars.insert(name, if kind == Handle::Gear { (h % 2) as f64 } else { (h % 5) as f64 });
        }
    }
    for (name, value) in changes {
        host.vars.insert(name, value);
    }
}

fn frame_hook(dump_frame: i64) -> String {
    format!(
        r#"(() => {{
          const D = globalThis.__dom;
          if (!D || !D.ScreenDocument) return 'no dom';
          const frame = D.ScreenDocument.prototype.frame;
          let n = 0;
          globalThis.__fz = [];
          D.ScreenDocument.prototype.frame = function () {{
            const r = frame.call(this);
            n++;
            if (r) {{
              const s = JSON.stringify([this._lastOps, this._strings]);
              let a = 0x811c9dc5 | 0, b = 0x1234567 | 0;
              for (let i = 0; i < s.length; i++) {{ const c = s.charCodeAt(i); a = Math.imul(a ^ c, 16777619); b = (Math.imul(b, 31) + c) | 0; }}
              globalThis.__fz.push([n, this.__screen || '', (a >>> 0).toString(16).padStart(8, '0') + (b >>> 0).toString(16).padStart(8, '0'), s.length]);
              if (n === {dump_frame}) globalThis.__fzDump = JSON.stringify({{ frame: n, screen: this.__screen || '', ops: this._lastOps, strings: this._strings }});
            }}
            return r;
          }};
          return 'ok';
        }})()"#
    )
}

const DRAIN_JS: &str = "(() => { const a = globalThis.__fz; if (!a || !a.length) return ''; globalThis.__fz = []; return JSON.stringify(a); })()";

fn text_point_js(screen: &str, frac: f64, width: u32, height: u32) -> String {
    format!(
        r#"(() => {{
          const doc = [...globalThis.__dom.documents.values()].find(d => d.__screen === {screen});
          if (!doc) return 'null';
          const out = [];
          for (const el of doc.querySelectorAll('*')) {{
            let text = false, child = false;
            for (const c of el.childNodes) {{
              if (c.nodeType === 3) {{ if ((c.data || '').trim()) text = true; }} else if (c.nodeType === 1) child = true;
            }}
            if (!text || child) continue;
            const r = el.getBoundingClientRect();
            if (r.width > 0 && r.height > 0 && r.left >= 0 && r.top >= 0 && r.left + r.width <= {width} && r.top + r.height <= {height}) out.push([r.left + r.width / 2, r.top + r.height / 2]);
          }}
          if (!out.length) return 'null';
          return JSON.stringify(out[Math.floor({frac} * out.length) % out.length]);
        }})()"#,
        screen = json_string(screen)
    )
}

#[test]
#[ignore]
fn fuzz_trace() {
    let html_ui = PathBuf::from(env_required("BOOT_HTML_UI"));
    let panel = PathBuf::from(env_required("BOOT_PANEL_DIR"));
    let out_path = PathBuf::from(env_required("FUZZ_TRACE_OUT"));
    let seed: u64 = env_or("FUZZ_SEED", 1);
    let seconds: f64 = env_or("FUZZ_SECONDS", 60.);
    let rate: f64 = env_or("FUZZ_RATE", 0.002);
    let dump_view = std::env::var("FUZZ_DUMP_VIEW").ok();
    let dump_frame: i64 = env_or("FUZZ_DUMP_FRAME", -1);

    let panel_cfg = std::fs::read_to_string(panel.join("panel.cfg")).expect("panel.cfg");
    let views_cfg = parse_panel_cfg(&panel_cfg);
    let mut options = CockpitOptions::new(html_ui, panel_cfg);
    options.panel_xml = std::fs::read_to_string(panel.join("panel.xml")).unwrap_or_default();
    options.budget = Duration::from_secs(600);

    let sim_ms = Arc::new(AtomicU64::new(0f64.to_bits()));
    let view_counter = std::cell::Cell::new(0u64);
    let setup_clock = sim_ms.clone();
    let mut cockpit = Cockpit::new(options, &move |engine: &Engine| {
        let clock = setup_clock.clone();
        let wall = setup_clock.clone();
        engine.with_host_object(|ctx, host| {
            host.set("hrtime", rquickjs::Function::new(ctx.clone(), move || f64::from_bits(clock.load(Ordering::Relaxed)))?)?;
            host.set("wallClock", rquickjs::Function::new(ctx.clone(), move || f64::from_bits(wall.load(Ordering::Relaxed)))?)?;
            Ok(())
        })?;
        let index = view_counter.get();
        view_counter.set(index + 1);
        let view_seed = (mix(seed.wrapping_mul(0x1000_0001).wrapping_add(index)) & 0xFFFF_FFFF) as u32;
        engine.eval(
            "fuzz-random",
            &format!(
                "(() => {{ let s = {view_seed}; Math.random = () => {{ s = (s + 0x6D2B79F5) | 0; let t = Math.imul(s ^ (s >>> 15), 1 | s); t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t; return ((t ^ (t >>> 14)) >>> 0) / 4294967296; }}; }})()"
            ),
        )?;
        engine.eval(
            "fuzz-date",
            "(() => { const Real = Date; const now = () => 1760000000000 + (globalThis.performance ? globalThis.performance.now() : 0); \
             class SimDate extends Real { constructor(...a) { if (a.length === 0) super(now()); else super(...a); } static now() { return now(); } } \
             globalThis.Date = SimDate; })()",
        )?;
        Ok(())
    })
    .unwrap();

    let view_names = cockpit.view_names();
    for name in &view_names {
        let dump = if dump_view.as_deref() == Some(name.as_str()) { dump_frame } else { -1 };
        let r = cockpit.eval_in(name, &frame_hook(dump));
        assert_eq!(r.as_deref(), Ok("ok"), "frame hook in {name}");
    }

    let mfd: Vec<(String, String, (u32, u32))> = views_cfg
        .iter()
        .filter_map(|v| v.texture.as_ref().filter(|t| t.to_ascii_uppercase().contains("MFD")).map(|t| (v.name.clone(), t.clone(), v.display)))
        .collect();

    let mut host = Store { vars: BTreeMap::new(), strings: BTreeMap::new(), units: BTreeMap::new(), writes: Vec::new(), errors: Vec::new(), now_ms: 0. };
    host.vars.insert("L:A32NX_COLD_AND_DARK_SPAWN".into(), 0.);
    for potentiometer in [80, 81, 82, 87, 88, 89, 90, 91, 92, 93, 98, 99] {
        host.vars.insert(format!("LIGHT POTENTIOMETER:{potentiometer}"), 1.);
    }

    if let Some(dir) = out_path.parent() {
        let _ = std::fs::create_dir_all(dir);
    }
    let mut out = std::io::BufWriter::new(std::fs::File::create(&out_path).expect("trace file"));
    let meta = serde_json::json!({"meta": {"seed": seed, "seconds": seconds, "rate": rate, "views": view_names}});
    writeln!(out, "{meta}").unwrap();

    let started = Instant::now();
    let mut click_rng = Rng::new(seed);
    let mut next_click = 100u64;
    let ticks = (seconds * 1000. / TICK_MS) as u64;
    let mut frames_total = 0u64;
    let mut writes_total = 0u64;
    for tick in 0..ticks {
        let t = tick as f64 * TICK_MS;
        sim_ms.store(t.to_bits(), Ordering::Relaxed);
        host.now_ms = t;
        apply_inputs(&mut host, seed, tick, rate);

        let mut clicks: Vec<serde_json::Value> = Vec::new();
        if tick >= next_click && !mfd.is_empty() {
            let pick = (click_rng.next() % mfd.len() as u64) as usize;
            let (view, screen, (w, h)) = &mfd[pick];
            let want_text = click_rng.next() % 10 < 3;
            let frac = click_rng.frac();
            let (rx, ry) = (click_rng.frac() * *w as f64, click_rng.frac() * *h as f64);
            let mut point = None;
            if want_text {
                if let Ok(s) = cockpit.eval_in(view, &text_point_js(screen, frac, *w, *h)) {
                    point = serde_json::from_str::<Vec<f64>>(&s).ok().filter(|p| p.len() == 2).map(|p| (p[0], p[1]));
                }
            }
            let (x, y, kind) = match point {
                Some((x, y)) => (x, y, "text"),
                None => (round2(rx), round2(ry), "rand"),
            };
            cockpit.screen_event(screen, "down", x, y, 0, 0.);
            cockpit.screen_event(screen, "up", x, y, 0, 0.);
            clicks.push(serde_json::json!([screen, x, y, kind]));
            next_click = tick + 30 + click_rng.next() % 21;
        }

        cockpit.tick(&mut host, t);

        let mut frames = serde_json::Map::new();
        for name in &view_names {
            if let Ok(s) = cockpit.eval_in(name, DRAIN_JS) {
                if let Ok(serde_json::Value::Array(list)) = serde_json::from_str::<serde_json::Value>(&s) {
                    frames_total += list.len() as u64;
                    frames.insert(name.clone(), serde_json::Value::Array(list));
                }
            }
        }
        let writes: Vec<serde_json::Value> = host.writes.drain(..).map(|(n, v)| serde_json::json!([n, json_number(v)])).collect();
        writes_total += writes.len() as u64;
        let errors: Vec<String> = host.errors.drain(..).collect();
        let mut line = serde_json::Map::new();
        line.insert("t".into(), tick.into());
        if !writes.is_empty() {
            line.insert("w".into(), writes.into());
        }
        if !frames.is_empty() {
            line.insert("f".into(), frames.into());
        }
        if !clicks.is_empty() {
            line.insert("c".into(), clicks.into());
        }
        if !errors.is_empty() {
            line.insert("e".into(), errors.into());
        }
        writeln!(out, "{}", serde_json::Value::Object(line)).unwrap();
    }
    out.flush().unwrap();
    drop(out);

    if let (Some(view), true) = (&dump_view, dump_frame >= 0) {
        let raw = cockpit.eval_in(view, "globalThis.__fzDump || ''").unwrap_or_default();
        let path = std::env::var("FUZZ_DUMP_OUT").map(PathBuf::from).unwrap_or_else(|_| PathBuf::from(format!("{}.dump.json", out_path.display())));
        let text = serde_json::from_str::<serde_json::Value>(&raw).ok().and_then(|v| serde_json::to_string_pretty(&v).ok()).unwrap_or(raw);
        if text.is_empty() {
            println!("dump: view {view} frame {dump_frame} was never drawn");
        } else {
            std::fs::write(&path, text).expect("dump file");
            println!("dump: wrote {}", path.display());
        }
    }

    let size = std::fs::metadata(&out_path).map(|m| m.len()).unwrap_or(0);
    println!(
        "fuzz_trace: seed {seed}, {ticks} ticks, {} variables read, {frames_total} frames, {writes_total} writes, {} bytes, {:.1} s, all loaded {}",
        host.units.len(),
        size,
        started.elapsed().as_secs_f64(),
        cockpit.all_loaded()
    );
}

#[derive(Default)]
struct Trace {
    seed: Option<u64>,
    frames: BTreeMap<String, Vec<(u64, u64, String, String)>>,
    writes: Vec<(u64, String, String)>,
    errors: Vec<(u64, String)>,
}

fn load_trace(path: &str) -> Trace {
    let text = std::fs::read_to_string(path).unwrap_or_else(|e| panic!("{path}: {e}"));
    let mut trace = Trace::default();
    for line in text.lines().filter(|l| !l.trim().is_empty()) {
        let v: serde_json::Value = serde_json::from_str(line).unwrap_or_else(|e| panic!("{path}: bad line: {e}"));
        if let Some(meta) = v.get("meta") {
            trace.seed = meta["seed"].as_u64();
            continue;
        }
        let tick = v["t"].as_u64().unwrap_or(0);
        if let Some(map) = v["f"].as_object() {
            for (view, list) in map {
                for f in list.as_array().into_iter().flatten() {
                    trace.frames.entry(view.clone()).or_default().push((tick, f[0].as_u64().unwrap_or(0), f[1].as_str().unwrap_or("").to_string(), f[2].as_str().unwrap_or("").to_string()));
                }
            }
        }
        for w in v["w"].as_array().into_iter().flatten() {
            trace.writes.push((tick, w[0].as_str().unwrap_or("").to_string(), w[1].to_string()));
        }
        for e in v["e"].as_array().into_iter().flatten() {
            trace.errors.push((tick, e.as_str().unwrap_or("").to_string()));
        }
    }
    trace
}

#[test]
#[ignore]
fn fuzz_compare() {
    let a = load_trace(&env_required("FUZZ_TRACE_A"));
    let b = load_trace(&env_required("FUZZ_TRACE_B"));
    let mut identical = true;
    if a.seed != b.seed {
        println!("warning: the traces were made with different seeds ({:?} and {:?})", a.seed, b.seed);
    }
    let mut views: Vec<&String> = a.frames.keys().chain(b.frames.keys()).collect();
    views.sort();
    views.dedup();
    let empty = Vec::new();
    for view in views {
        let fa = a.frames.get(view).unwrap_or(&empty);
        let fb = b.frames.get(view).unwrap_or(&empty);
        let diff = (0..fa.len().max(fb.len())).find(|&i| match (fa.get(i), fb.get(i)) {
            (Some(x), Some(y)) => x.1 != y.1 || x.2 != y.2 || x.3 != y.3,
            _ => true,
        });
        match diff {
            None => println!("{view}: identical ({} frames)", fa.len()),
            Some(i) => {
                identical = false;
                let show = |f: Option<&(u64, u64, String, String)>| match f {
                    Some((tick, n, screen, hash)) => format!("tick {tick} frame {n} {screen} {hash}"),
                    None => "no frame".to_string(),
                };
                println!("{view}: first differing frame #{i}: A = {} | B = {}", show(fa.get(i)), show(fb.get(i)));
            }
        }
    }
    let diff = (0..a.writes.len().max(b.writes.len())).find(|&i| a.writes.get(i) != b.writes.get(i));
    match diff {
        None => println!("writes: identical ({} writes)", a.writes.len()),
        Some(i) => {
            identical = false;
            let show = |w: Option<&(u64, String, String)>| match w {
                Some((tick, name, value)) => format!("tick {tick} {name} = {value}"),
                None => "no write".to_string(),
            };
            println!("writes: first difference at write #{i}: A = {} | B = {}", show(a.writes.get(i)), show(b.writes.get(i)));
        }
    }
    match (0..a.errors.len().max(b.errors.len())).find(|&i| a.errors.get(i) != b.errors.get(i)) {
        None => println!("errors: identical ({} logged)", a.errors.len()),
        Some(i) => {
            identical = false;
            println!("errors: first difference at #{i}: A = {:?} | B = {:?}", a.errors.get(i), b.errors.get(i));
        }
    }
    println!("RESULT: {}", if identical { "identical" } else { "DIVERGED" });
}
