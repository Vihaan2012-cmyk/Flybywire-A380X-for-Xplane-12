//! A live run of XPHFBW's JS pipeline with no X-Plane: this test plays the
//! plugin's part (docs/briefs/xphfbw-js-bridge.md) — it creates the session
//! and every cockpit screen, starts the real XPHFBW.exe on the real aircraft,
//! ticks the remote systems and the bridge in lockstep at 60 Hz, prints what
//! the views report, and writes each screen's last picture to a PNG.
//!
//! ```text
//! XPHFBW_EXE=".../plugins/fbw_a380_systems/XPHFBW/XPHFBW.exe" \
//! XPHFBW_AIRCRAFT=".../Aircraft/FlyByWire A380X" \
//! XPHFBW_XP_ROOT=".../X-Plane 12" XPHFBW_OUT="D:/A380/fbw-build/xphfbw-harness" \
//! XPHFBW_SECONDS=90 cargo test --release --features js --lib xphfbw_harness -- --ignored --nocapture
//! ```

use std::cell::RefCell;
use std::path::PathBuf;
use std::sync::atomic::Ordering;
use std::time::{Duration, Instant};

use systems::simulation::StartState;

use crate::display::screens::SCREENS;
use crate::remote::client::RemoteSystems;
use crate::xp::Xplm;
use crate::xphfbw_bridge::ScreenBlock;
use crate::xphfbw_host::XphfbwHost;

fn env_path(name: &str) -> PathBuf {
    PathBuf::from(std::env::var(name).unwrap_or_else(|_| panic!("set {name}")))
}

fn write_png(path: &std::path::Path, block: &ScreenBlock) {
    let h = block.header();
    let (w, hgt) = (h.width.load(Ordering::Relaxed), h.height.load(Ordering::Relaxed));
    let mut rgba = block.pixels().to_vec();
    for px in rgba.chunks_exact_mut(4) {
        px.swap(0, 2);
        // Premultiplied and possibly transparent: show it over black.
        px[3] = 255;
    }
    let file = std::fs::File::create(path).unwrap();
    let mut enc = png::Encoder::new(std::io::BufWriter::new(file), w, hgt);
    enc.set_color(png::ColorType::Rgba);
    enc.set_depth(png::BitDepth::Eight);
    enc.write_header().unwrap().write_image_data(&rgba).unwrap();
}

#[test]
#[ignore]
fn xphfbw_harness() {
    let exe = env_path("XPHFBW_EXE");
    let aircraft = env_path("XPHFBW_AIRCRAFT");
    let xp_root = env_path("XPHFBW_XP_ROOT");
    let out = env_path("XPHFBW_OUT");
    let seconds: u64 = std::env::var("XPHFBW_SECONDS").ok().and_then(|s| s.parse().ok()).unwrap_or(60);
    std::fs::create_dir_all(&out).unwrap();

    let xplm: &'static Xplm = Box::leak(Box::new(Xplm::dummy()));
    let mut vars = crate::Vars::new(xplm);
    let panel_cfg = std::fs::read_to_string(aircraft.join("panel").join("panel.cfg")).expect("panel.cfg");

    // The facility database, as JsHost::find starts it on the plugin's main
    // thread (the providers are that thread's).
    match crate::navdata::NavData::load(&xp_root) {
        Ok(nav) => crate::js_bridge::add_provider(Box::new(nav)),
        Err(e) => eprintln!("harness: navdata: {e}"),
    }

    let host: RefCell<Option<XphfbwHost>> = RefCell::new(None);
    let screens: RefCell<Vec<(&'static str, ScreenBlock)>> = RefCell::new(Vec::new());
    let mut remote = RemoteSystems::connect(StartState::Taxi, &mut vars, |tag| {
        eprintln!("harness: session {tag}");
        *host.borrow_mut() = Some(XphfbwHost::start(tag, xplm, &panel_cfg, None).ok_or("the bridge host did not start")?);
        for def in SCREENS {
            let (dw, dh) = crate::xphfbw_bridge_views::device_size(def.id, def.width, def.height);
            match ScreenBlock::create(tag, def.id, dw, dh) {
                Some(b) => screens.borrow_mut().push((def.id, b)),
                None => eprintln!("harness: could not create screen {}", def.id),
            }
        }
        std::process::Command::new(&exe)
            .arg(tag)
            .arg(std::process::id().to_string())
            .arg(format!("--xp-root={}", xp_root.display()))
            .arg(format!("--aircraft={}", aircraft.display()))
            .spawn()
            .map(Some)
            .map_err(|e| format!("cannot start {}: {e}", exe.display()))
    })
    .expect("XPHFBW's systems did not come up");
    let mut host = host.into_inner().unwrap();
    let screens = screens.into_inner();
    eprintln!("harness: systems up, {} views, {} screens", host.view_count(), screens.len());

    // Nothing X-Plane would feed is here, so power the aircraft the way a
    // pilot on the stand would: batteries on auto, ground power in and on.
    {
        use systems::simulation::{SimulatorReaderWriter, VariableRegistry};
        let mut set = |name: &str, value: f64| {
            let id = vars.get_unprefixed(name.to_owned());
            vars.write(&id, value);
        };
        set("TOTAL WEIGHT", 400_000.);
        set("AMBIENT PRESSURE", 29.92);
        set("AMBIENT TEMPERATURE", 15.);
        set("SEA LEVEL PRESSURE", 1013.25);
        for bat in ["BAT_1", "BAT_2", "BAT_ESS", "BAT_APU"] {
            set(&format!("A32NX_OVHD_ELEC_{bat}_PB_IS_AUTO"), 1.);
        }
        // Display brightness knobs up (stored as a 0-1 ratio, bind.rs).
        for n in 80..=100 {
            set(&format!("LIGHT POTENTIOMETER:{n}"), 1.);
        }
        for n in 1..=4 {
            set(&format!("A32NX_EXT_PWR_AVAIL:{n}"), 1.);
            set(&format!("A32NX_OVHD_ELEC_EXT_PWR_{n}_PB_IS_ON"), 1.);
        }
    }

    let started = Instant::now();
    let mut last = started;
    let mut report_at = 0.;
    let mut time = 0.;
    while started.elapsed() < Duration::from_secs(seconds) {
        let now = Instant::now();
        let delta = now - last;
        last = now;
        time += delta.as_secs_f64();
        host.pre_tick(&mut vars);
        if let Some(message) = remote.tick(delta, time, &mut vars) {
            eprintln!("harness: systems: {message}");
        }
        host.post_tick(&mut vars, time, &[], &[]);
        if time >= report_at {
            report_at = time + 5.;
            let frames: Vec<String> = screens.iter().map(|(id, b)| format!("{}={}", id.trim_start_matches("SCREEN_"), b.header().frame.load(Ordering::Relaxed))).collect();
            eprintln!(
                "harness: t={time:.0}s loaded {}/{} displays_active={} gone={} frames {}",
                host.views_loaded(),
                host.view_count(),
                host.displays_active(),
                host.gone(),
                frames.join(" ")
            );
        }
        std::thread::sleep(Duration::from_millis(16));
    }
    for (id, block) in &screens {
        if block.header().frame.load(Ordering::Relaxed) > 0 {
            write_png(&out.join(format!("{id}.png")), block);
        }
    }
    eprintln!("harness: pictures in {}", out.display());
}
