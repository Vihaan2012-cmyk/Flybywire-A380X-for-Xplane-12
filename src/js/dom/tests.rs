//! The DOM's tests, run in QuickJS with a mock host (tests/harness.js):
//! unit tests, golden op streams from fixtures modelled on FlyByWire's
//! components, and msfs-sdk's own FSComponent rendering through the DOM.
//!
//! Set `DOM_BLESS=1` to write the golden files from the current output.

use std::path::{Path, PathBuf};
use std::time::{Duration, Instant};

use super::SCRIPTS;
use crate::js::{Engine, EngineOptions, Host, LogLevel};

const HARNESS: &str = include_str!("tests/harness.js");
const UNIT: &str = include_str!("tests/unit.js");
const COMMON: &str = include_str!("tests/fixtures/common.js");

#[derive(Default)]
struct TestHost {
    log: Vec<(LogLevel, String)>,
    /// Buses powered and screen potentiometers up.
    powered: bool,
}

impl Host for TestHost {
    fn get_var(&mut self, name: &str, _unit: &str) -> f64 {
        if self.powered && name.ends_with("_IS_POWERED") {
            1.
        } else if self.powered && name.starts_with("LIGHT POTENTIOMETER") {
            100.
        } else {
            0.
        }
    }
    fn set_var(&mut self, _name: &str, _unit: &str, _value: f64) {}
    fn log(&mut self, level: LogLevel, message: &str) {
        self.log.push((level, message.to_string()));
    }
}

fn engine() -> (Engine, TestHost) {
    let engine = Engine::new(EngineOptions { budget: Duration::from_secs(60), ..Default::default() }).unwrap();
    let mut host = TestHost::default();
    engine.run_script(&mut host, "harness.js", HARNESS).unwrap();
    for (name, source) in SCRIPTS {
        engine.run_script(&mut host, name, source).unwrap_or_else(|e| panic!("{name}: {e}"));
    }
    (engine, host)
}

fn tests_dir() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("src/js/dom/tests")
}

#[test]
fn unit_tests_pass() {
    let (engine, mut host) = engine();
    engine.run_script(&mut host, "unit.js", UNIT).unwrap();
    let report = engine.eval("run", "__runUnitTests()").unwrap();
    let report: serde_json::Value = serde_json::from_str(&report).unwrap();
    let failures = report["failures"].as_array().unwrap();
    assert!(failures.is_empty(), "{}", failures.iter().map(|f| f.as_str().unwrap_or("")).collect::<Vec<_>>().join("\n\n"));
    assert!(report["passed"].as_u64().unwrap() >= 19, "{report}");
}

fn golden(name: &str) {
    let (engine, mut host) = engine();
    engine.run_script(&mut host, "fixtures/common.js", COMMON).unwrap();
    let source = std::fs::read_to_string(tests_dir().join("fixtures").join(format!("{name}.js"))).unwrap();
    let out = engine.eval(&format!("fixtures/{name}.js"), &source).unwrap();
    let path = tests_dir().join("golden").join(format!("{name}.txt"));
    if std::env::var_os("DOM_BLESS").is_some() {
        std::fs::write(&path, &out).unwrap();
        return;
    }
    let expected = std::fs::read_to_string(&path).unwrap_or_else(|e| panic!("{}: {e} (DOM_BLESS=1 writes it)", path.display()));
    if out.replace("\r\n", "\n") != expected.replace("\r\n", "\n") {
        let line = out.lines().zip(expected.lines()).position(|(a, b)| a != b).unwrap_or(out.lines().count().min(expected.lines().count()));
        panic!(
            "{name}: the stream differs from {} at line {}:\n  got:      {}\n  expected: {}",
            path.display(),
            line + 1,
            out.lines().nth(line).unwrap_or("(end)"),
            expected.lines().nth(line).unwrap_or("(end)")
        );
    }
}

#[test]
fn golden_pfd_attitude_fma_and_blinking() {
    golden("pfd");
}

#[test]
fn golden_ewd_gauge_and_clipped_tape() {
    golden("ewd");
}

#[test]
fn golden_mfd_flex_grid_hover_and_click() {
    golden("mfd");
}

#[test]
fn golden_nd_canvas_map() {
    golden("nd");
}

/// msfs-sdk 2.3.3's FSComponent (the build FlyByWire's workspace installs)
/// rendering JSX through this DOM, with the runtime's MSFS environment the
/// SDK's bundle needs at load. Skipped when the workspace has no SDK.
#[test]
fn msfs_sdk_fscomponent_renders_through_the_dom() {
    let sdk = Path::new(env!("CARGO_MANIFEST_DIR")).join("../fbw-aircraft/node_modules/@microsoft/msfs-sdk/msfssdk-iife.js");
    let Ok(sdk_source) = std::fs::read_to_string(&sdk) else {
        eprintln!("skipped: {} is not there", sdk.display());
        return;
    };
    let engine = Engine::new(EngineOptions { budget: Duration::from_secs(60), ..Default::default() }).unwrap();
    let mut host = TestHost::default();
    engine.run_script(&mut host, "harness.js", HARNESS).unwrap();
    engine.run_script(&mut host, "msfs/window.js", include_str!("../msfs/window.js")).unwrap();
    for (name, source) in SCRIPTS {
        engine.run_script(&mut host, name, source).unwrap();
    }
    engine.run_script(&mut host, "document", "globalThis.document = __createDocument('PFD', 768, 1024);").unwrap();
    for (name, source) in [
        ("msfs/coherent.js", include_str!("../msfs/coherent.js")),
        ("msfs/simvar.js", include_str!("../msfs/simvar.js")),
        ("msfs/environment.js", include_str!("../msfs/environment.js")),
        ("msfs/instrument.js", include_str!("../msfs/instrument.js")),
    ] {
        engine.run_script(&mut host, name, source).unwrap_or_else(|e| panic!("{name}: {e}"));
    }
    engine.run_script(&mut host, "msfssdk-iife.js", &sdk_source).unwrap();
    // FlyByWire's AttitudeIndicatorFixedUpper markup (PFD/AttitudeIndicatorFixed.tsx),
    // repeated to the size of a PFD, as a real DisplayComponent.
    let tsx = r#"
        const { FSComponent, DisplayComponent, Subject, MappedSubject } = (globalThis as any).msfssdk;
        class Upper extends DisplayComponent<any> {
          render() {
            return (
              <g id="AttitudeUpperInfoGroup" visibility={this.props.visibility}>
                <g id="RollProtGroup" class="SmallStroke Green" style={{ display: this.props.normalLaw.map((nl: boolean) => (nl ? 'block' : 'none')) }}>
                  <path id="RollProtRight" d="m105.64 62.887 1.5716-0.8008m-1.5716-0.78293 1.5716-0.8008" />
                  <path id="RollProtLeft" d="m32.064 61.303-1.5716-0.8008m1.5716 2.3845-1.5716-0.8008" />
                </g>
                <g class="SmallStroke White">
                  <path d="m98.645 51.067 2.8492-2.8509" />
                  <path d="m90.858 44.839a42.133 42.158 0 0 0-43.904 0" />
                </g>
                <path class="NormalStroke Yellow CornerRound" d="m68.906 38.650-2.5184-3.7000h5.0367l-2.5184 3.7000" />
                <text class="FontMedium MiddleAlign Green" x="20" y="100">{this.props.speed}</text>
              </g>
            );
          }
        }
        const doc = (globalThis as any).document;
        const style = doc.createElement('style');
        style.textContent = '.pfd-svg { position: absolute; width: 768px; height: 1024px; background: #000; font-family: Ecam; } .SmallStroke { stroke-width: 0.1mm; } .NormalStroke { stroke-width: 0.16mm; } .Green { stroke: #0f0; fill: none; } text.Green { fill: #0f0; stroke: none; } .White { stroke: #fff; fill: none; } .Yellow { stroke: #ff0; fill: none; } .FontMedium { font-size: 6px; } .MiddleAlign { text-anchor: middle; }';
        doc.head.appendChild(style);
        const visibility = Subject.create('visible');
        const normalLaw = Subject.create(true);
        const speed = Subject.create(250);
        const copies = [];
        for (let i = 0; i < 200; i++) copies.push(<g transform={`translate(0 ${i * 0.5})`}><Upper visibility={visibility} normalLaw={normalLaw} speed={speed} /></g>);
        FSComponent.render(<svg class="pfd-svg" viewBox="0 0 158.75 211.6">{copies}</svg>, doc.body);
        (globalThis as any).__sdk = { doc, visibility, normalLaw, speed };
        'rendered';
    "#;
    let r = engine.eval_typescript("sdk_fixture.tsx", tsx).unwrap();
    assert_eq!(r, "rendered");
    let started = Instant::now();
    engine.tick(&mut host, 16.).unwrap();
    let first = started.elapsed();
    let summary = |engine: &Engine| {
        engine
            .eval("s", "(() => { const s = __test.last('PFD'); return JSON.stringify({ submits: __test.submits.length, ops: s ? s.ops.length : 0, stats: __sdk.doc.stats }); })()")
            .unwrap()
    };
    let s1: serde_json::Value = serde_json::from_str(&summary(&engine)).unwrap();
    assert_eq!(s1["submits"], 1, "{s1}");
    let dump = engine.eval("d", "__test.dumpLast('PFD')").unwrap();
    assert!(dump.contains("TEXT \"250\" \"Ecam\" 6 400 0 15.5 100"), "{}", &dump[..dump.len().min(3000)]);
    assert!(dump.contains("ELLIPSE 68.906 80.823 42.133 42.158 0 -1.023 -2.119 1"), "the arc");
    assert_eq!(dump.matches("TEXT \"250\"").count(), 200);

    // A value every copy is bound to changes: one text per copy repaints.
    engine.eval("set", "__sdk.speed.set(251)").unwrap();
    let started = Instant::now();
    engine.tick(&mut host, 32.).unwrap();
    let update = started.elapsed();
    let dump = engine.eval("d", "__test.dumpLast('PFD')").unwrap();
    assert_eq!(dump.matches("TEXT \"251\"").count(), 200);
    // Nothing changes: nothing is submitted.
    let started = Instant::now();
    engine.tick(&mut host, 48.).unwrap();
    let idle = started.elapsed();
    let s3: serde_json::Value = serde_json::from_str(&summary(&engine)).unwrap();
    assert_eq!(s3["submits"], 2, "{s3}");
    // Style: roll protection hidden through the style record.
    engine.eval("set", "__sdk.normalLaw.set(false)").unwrap();
    engine.tick(&mut host, 64.).unwrap();
    let dump = engine.eval("d", "__test.dumpLast('PFD')").unwrap();
    assert!(!dump.contains("MOVE_TO 105.64 62.887"), "roll protection hidden");
    eprintln!(
        "FSComponent PFD-sized tree (200 x 8 elements): first frame {:.1} ms, text update {:.1} ms, idle frame {:.2} ms; {}",
        first.as_secs_f64() * 1000.,
        update.as_secs_f64() * 1000.,
        idle.as_secs_f64() * 1000.,
        s3["stats"]
    );
    for (level, line) in host.log.iter().filter(|(l, _)| *l != LogLevel::Info) {
        eprintln!("{level:?}: {line}");
    }
}

thread_local! {
    static RECORDED: std::cell::RefCell<Vec<(String, Vec<f64>, Vec<String>)>> = const { std::cell::RefCell::new(Vec::new()) };
    static DOM_MS: std::cell::RefCell<Vec<f64>> = const { std::cell::RefCell::new(Vec::new()) };
}

/// FlyByWire's own A380X PFD, as the MSFS package ships it (pfd.html,
/// pfd.css, pfd.js), loaded by the runtime (src/js/msfs) into a view with
/// this DOM, painting through the mock measurements. Every stream it
/// submits must read as the renderer reads it. Skipped when the package is
/// not installed.
#[test]
fn fbw_pfd_from_the_package_paints_valid_streams() {
    use rquickjs::Function;

    let package = Path::new("D:/Microsoft Flight Simulator 2020/Microsoft Flight Simulator 2020 Packages/Community/flybywire-aircraft-a380-842");
    let Ok(panel_cfg) = std::fs::read_to_string(package.join("SimObjects/AirPlanes/FlyByWire_A380_842/panel/panel.cfg")) else {
        eprintln!("skipped: the FlyByWire A380X package is not installed");
        return;
    };
    // The captain's PFD section only.
    let start = panel_cfg.find("[VCockpit05]").unwrap();
    let end = panel_cfg[start..].find("[VCockpit06]").map_or(panel_cfg.len(), |e| start + e);
    let mut options = crate::js::msfs::CockpitOptions::new(package.join("html_ui"), panel_cfg[start..end].to_string());
    options.panel_xml = std::fs::read_to_string(package.join("SimObjects/AirPlanes/FlyByWire_A380_842/panel/panel.xml")).unwrap_or_default();
    options.budget = Duration::from_secs(30);
    let setup = |engine: &Engine| -> Result<(), String> {
        engine.eval("harness.js", HARNESS)?;
        engine.with_host_object(|ctx, host| {
            let record = |screen: String, ops: Vec<f64>, strings: Vec<String>| RECORDED.with(|r| r.borrow_mut().push((screen, ops, strings)));
            host.set("recordSubmit", Function::new(ctx.clone(), record)?)?;
            let epoch = Instant::now();
            host.set("wallClock", Function::new(ctx.clone(), move || epoch.elapsed().as_secs_f64() * 1000.)?)?;
            host.set("domCost", Function::new(ctx.clone(), |ms: f64| DOM_MS.with(|d| d.borrow_mut().push(ms)))?)?;
            Ok(())
        })?;
        engine.eval("record", "(() => { const record = __host.recordSubmit; __host.submitDisplay = (s, ops, strings) => record(s, Array.from(ops), strings); })()")?;
        Ok(())
    };
    let mut cockpit = crate::js::msfs::Cockpit::new(options, &setup).unwrap();
    let mut host = TestHost { powered: true, ..Default::default() };
    let mut now = 0.;
    let mut ticks = Vec::new();
    // Past the display unit's self test (CONFIG_SELF_TEST_TIME, 15 s).
    for _ in 0..500 {
        now += 50.;
        let started = Instant::now();
        cockpit.tick(&mut host, now);
        ticks.push(started.elapsed().as_secs_f64() * 1000.);
    }
    let recorded = RECORDED.with(|r| std::mem::take(&mut *r.borrow_mut()));
    let errors: Vec<&String> = host.log.iter().filter(|(l, _)| *l == LogLevel::Error).map(|(_, m)| m).collect();
    eprintln!("{} submits; last tick {:.1} ms; errors: {}", recorded.len(), ticks.last().unwrap(), errors.len());
    for e in errors.iter().take(10) {
        eprintln!("  {}", &e[..e.len().min(400)]);
    }
    for (level, line) in host.log.iter().filter(|(l, m)| *l == LogLevel::Warn && m.contains("[dom]")).take(30) {
        eprintln!("{level:?}: {line}");
    }
    assert!(!recorded.is_empty(), "the PFD submitted nothing");
    for (screen, ops, strings) in &recorded {
        assert_eq!(screen, "SCREEN_DU_PFDL");
        crate::display::stream::parse(ops, strings.len()).unwrap_or_else(|e| panic!("a stream the renderer refuses: {e:?}"));
    }
    let (_, ops, strings) = recorded.last().unwrap();
    let late: Vec<f64> = ticks[ticks.len() - 100..].to_vec();
    let dom = DOM_MS.with(|d| std::mem::take(&mut *d.borrow_mut()));
    let dom_late = &dom[dom.len().saturating_sub(100)..];
    eprintln!(
        "DOM frame (style, layout, paint, submit): first {:.1} ms, max {:.1} ms, last 100 mean {:.2} ms",
        dom.first().copied().unwrap_or(0.),
        dom.iter().cloned().fold(0., f64::max),
        dom_late.iter().sum::<f64>() / dom_late.len().max(1) as f64
    );
    eprintln!(
        "last stream: {} numbers, {} strings, e.g. {:?}; last 100 ticks: mean {:.1} ms, max {:.1} ms",
        ops.len(),
        strings.len(),
        &strings[..strings.len().min(40)],
        late.iter().sum::<f64>() / late.len() as f64,
        late.iter().cloned().fold(0., f64::max)
    );
}
