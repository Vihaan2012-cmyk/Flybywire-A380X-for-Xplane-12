use std::collections::HashMap;

struct Samples {
    cols: HashMap<String, usize>,
    rows: Vec<Vec<f64>>,
}

impl Samples {
    fn parse(csv: &str) -> Self {
        let mut lines = csv.lines();
        let cols = lines.next().unwrap_or("").split(',').enumerate().map(|(i, c)| (c.to_string(), i)).collect();
        let rows = lines.map(|l| l.split(',').map(|v| v.parse::<f64>().unwrap_or(f64::NAN)).collect()).collect();
        Self { cols, rows }
    }

    fn col(&self, name: &str) -> Option<usize> {
        self.cols.get(name).copied()
    }

    fn get(&self, row: &[f64], name: &str) -> f64 {
        self.col(name).and_then(|i| row.get(i).copied()).unwrap_or(f64::NAN)
    }

    fn has(&self, name: &str) -> bool {
        self.cols.contains_key(name)
    }

    fn values(&self, name: &str) -> Vec<f64> {
        self.rows.iter().map(|r| self.get(r, name)).filter(|v| v.is_finite()).collect()
    }
}

fn median(mut v: Vec<f64>) -> f64 {
    if v.is_empty() {
        return f64::NAN;
    }
    v.sort_by(|a, b| a.total_cmp(b));
    v[v.len() / 2]
}

struct Report {
    lines: Vec<String>,
    problems: usize,
}

impl Report {
    fn ok(&mut self, s: String) {
        self.lines.push(format!("OK       {s}"));
    }
    fn problem(&mut self, s: String) {
        self.problems += 1;
        self.lines.push(format!("PROBLEM  {s}"));
    }
    fn info(&mut self, s: String) {
        self.lines.push(format!("INFO     {s}"));
    }
    fn section(&mut self, s: &str) {
        self.lines.push(String::new());
        self.lines.push(format!("== {s}"));
    }
}

fn spans(s: &Samples, mut hit: impl FnMut(&[f64]) -> bool) -> Vec<(f64, f64)> {
    let mut out = Vec::new();
    let mut start: Option<f64> = None;
    let mut last_t = 0.0;
    for r in &s.rows {
        let t = s.get(r, "t");
        if hit(r) {
            start.get_or_insert(t);
        } else if let Some(t0) = start.take() {
            out.push((t0, last_t));
        }
        last_t = t;
    }
    if let Some(t0) = start {
        out.push((t0, last_t));
    }
    out
}

fn fmt_spans(v: &[(f64, f64)]) -> String {
    v.iter().take(6).map(|(a, b)| format!("{a:.0}-{b:.0} s")).collect::<Vec<_>>().join(", ") + if v.len() > 6 { " ..." } else { "" }
}

pub fn report(samples_csv: &str, events_log: &str, processes_csv: &str) -> String {
    let s = Samples::parse(samples_csv);
    let mut r = Report { lines: Vec::new(), problems: 0 };
    let parked = |s: &Samples, row: &[f64]| s.get(row, "sim/flightmodel/failures/onground_any") > 0.5 && s.get(row, "sim/flightmodel/position/groundspeed") < 0.3;

    r.section("Performance");
    let fps = median(s.values("fps"));
    let frame_ms = median(s.values("sim/time/framerate_period")) * 1000.0;
    let gpu_ms = median(s.values("sim/time/gpu_time_per_frame_sec_approx")) * 1000.0;
    let plugin_ms = median(s.rows.iter().map(|row| s.get(row, "plugin_tick_ms") + s.get(row, "plugin_draw_ms")).filter(|v| v.is_finite()).collect());
    let sasl_ms = median(s.values("fbw/diag/sasl_update_ms"));
    let line = format!("median {fps:.0} fps, frame {frame_ms:.1} ms, GPU {gpu_ms:.1} ms, this plugin {plugin_ms:.1} ms, SASL {sasl_ms:.1} ms");
    if plugin_ms > 10.0 {
        r.problem(format!("{line}: the plugin takes over 10 ms a frame"));
    } else if plugin_ms.is_finite() {
        r.ok(line);
    }

    r.section("Ground speed and wings while parked");
    let ir_gs: Vec<f64> = s.rows.iter().filter(|row| parked(&s, row)).flat_map(|row| (1..=3).map(|n| s.get(row, &format!("fbw/A32NX_ADIRS_SENSED_{n}_GROUND_SPEED")))).filter(|v| v.is_finite()).collect();
    match ir_gs.iter().copied().fold(None, |m: Option<f64>, v| Some(m.map_or(v, |m| m.max(v)))) {
        Some(max) if max > 2.0 => r.problem(format!("IR ground speed reached {max:.1} kt while parked")),
        Some(max) => r.ok(format!("IR ground speed parked: max {max:.1} kt")),
        None if !s.has("fbw/A32NX_ADIRS_SENSED_1_GROUND_SPEED") => r.info("IR ground speed was not recorded in this capture".into()),
        None => r.info("never parked long enough to judge IR ground speed".into()),
    }
    let flex: Vec<f64> = s.rows.iter().filter(|row| parked(&s, row)).map(|row| s.get(row, "fbw/A32NX_WING_FLEX_LEFT_INBOARD_MID")).filter(|v| v.is_finite()).collect();
    if !flex.is_empty() {
        let m = median(flex);
        if m > 1.5 {
            r.problem(format!("wing flex parked {m:+.2} deg (bent up as in flight; parked should droop below 0)"));
        } else {
            r.ok(format!("wing flex parked {m:+.2} deg"));
        }
    }
    let belly = spans(&s, |row| s.get(row, "sim/flightmodel/failures/onground_any") > 0.5 && (0..5).any(|i| s.get(row, &format!("sim/flightmodel2/gear/deploy_ratio[{i}]")) < 0.99));
    if belly.is_empty() {
        r.ok("gear down whenever on the ground".into());
    } else {
        r.problem(format!("on the ground with a gear leg not fully down: {}", fmt_spans(&belly)));
    }
    if s.values("sim/flightmodel2/misc/has_crashed").iter().any(|v| *v > 0.5) {
        r.problem("X-Plane flagged a crash".into());
    }

    r.section("Engines");
    for n in 1..=4 {
        let state = format!("fbw/A32NX_ENGINE_STATE_{n}");
        let ff = format!("sim/flightmodel/engine/ENGN_FF_[{}]", n - 1);
        let master = format!("fbw/GENERAL_ENG_STARTER_{n}");
        let starved = spans(&s, |row| (s.get(row, &state) - 1.0).abs() < 0.1 && s.get(row, &ff) < 0.01);
        if !starved.is_empty() {
            r.problem(format!("engine {n} running with no fuel flow: {}", fmt_spans(&starved)));
        }
        let mut prev: Option<f64> = None;
        for row in &s.rows {
            let st = s.get(row, &state);
            if let Some(p) = prev {
                if (p - 1.0).abs() < 0.1 && (st.round() == 0.0 || st.round() == 4.0) && s.get(row, &master) > 0.5 {
                    r.problem(format!("engine {n} stopped at {:.0} s with its master switch still on", s.get(row, "t")));
                }
            }
            if st.is_finite() {
                prev = Some(st);
            }
        }
        let tla: Vec<f64> = s.rows.iter().take(300).map(|row| s.get(row, &format!("fbw/A32NX_AUTOTHRUST_TLA_{n}"))).filter(|v| v.is_finite()).collect();
        if let Some(max) = tla.iter().copied().reduce(f64::max) {
            if max > 5.0 {
                r.problem(format!("engine {n} thrust lever at {max:.0} deg within the first minute (nobody touched it?)"));
            }
        }
    }
    let exceed = events_log.lines().filter(|l| l.contains("exceedance:")).count();
    if exceed > 0 {
        r.info(format!("{exceed} engine exceedance lines in the log"));
    }

    r.section("Fire");
    let mut fires = 0;
    for zone in ["ENG1", "ENG2", "ENG3", "ENG4", "APU", "MLG"] {
        let fire = spans(&s, |row| s.get(row, &format!("fbw/A32NX_FIRE_DETECTED_{zone}")) > 0.5);
        if !fire.is_empty() {
            fires += 1;
            r.problem(format!("{zone} FIRE detected: {} (expected only during a FIRE TEST or an armed fire)", fmt_spans(&fire)));
        }
    }
    if fires == 0 && s.has("fbw/A32NX_FIRE_DETECTED_ENG1") {
        r.ok("no fire detected".into());
    }

    r.section("Exterior lights against their switches");
    let checks: [(&str, &str, &dyn Fn(&Samples, &[f64]) -> bool); 3] = [
        ("STROBE", "fbw/cockpit/SWITCH_OVHD_EXTLT_STROBE", &|s, row| s.get(row, "sim/cockpit2/switches/strobe_lights_on") > 0.5),
        ("NOSE", "fbw/cockpit/SWITCH_OVHD_EXTLT_NOSE", &|s, row| s.get(row, "sim/cockpit2/switches/landing_lights_switch[0]") > 0.5 || s.get(row, "sim/cockpit2/switches/taxi_light_on") > 0.5),
        ("LOGO", "fbw/cockpit/SWITCH_OVHD_EXTLT_LOGO", &|s, row| s.get(row, "sim/cockpit2/switches/generic_lights_switch[2]") > 0.5),
    ];
    for (name, anim, lit) in checks {
        let wrong = spans(&s, |row| (s.get(row, anim) - 1.0).abs() < 0.05 && lit(&s, row));
        if wrong.is_empty() {
            r.ok(format!("{name}: never lit with its switch at OFF"));
        } else {
            r.problem(format!("{name} lit with its switch at OFF: {}", fmt_spans(&wrong)));
        }
    }

    r.section("Cockpit lighting");
    let panel = s.values("sim/cockpit2/switches/panel_brightness_ratio[0]");
    if let (Some(lo), Some(hi)) = (panel.iter().copied().reduce(f64::min), panel.iter().copied().reduce(f64::max)) {
        r.info(format!("X-Plane panel brightness (default glow of unbound lettering) ranged {lo:.2}-{hi:.2}"));
    }

    r.section("Sound");
    let mut sound_lines = 0;
    let before = r.problems;
    for l in events_log.lines().filter(|l| l.contains("sound:")) {
        sound_lines += 1;
        if l.contains("cockpit sounds are off") || l.contains("no FlyByWire A380X package") {
            r.problem(l.trim().to_string());
        }
        for part in l.split("pitch x").skip(1) {
            let v: f64 = part.split(|c: char| !(c.is_ascii_digit() || c == '.')).next().unwrap_or("").parse().unwrap_or(1.0);
            if !(0.4..=2.0).contains(&v) {
                r.problem(format!("a voice plays at {v:.2}x its recorded pitch: {}", l.trim()));
                break;
            }
        }
    }
    if sound_lines == 0 {
        r.problem("no sound lines in the log at all".into());
    } else if r.problems == before {
        r.ok(format!("sound loaded and playing ({sound_lines} sound log lines, no mis-pitched voice)"));
    }

    r.section("Display process");
    let mut seen = false;
    let mut lost = Vec::new();
    for l in processes_csv.lines().filter(|l| l.contains(",XPHFBW.exe,")) {
        let f: Vec<&str> = l.split(',').collect();
        let instances: u32 = f.get(2).and_then(|v| v.parse().ok()).unwrap_or(0);
        if instances > 0 {
            seen = true;
        } else if seen {
            lost.push(f.first().copied().unwrap_or("?").to_string());
            seen = false;
        }
    }
    if lost.is_empty() {
        r.ok("XPHFBW never disappeared".into());
    } else {
        r.problem(format!("XPHFBW disappeared at {} s (crash?)", lost.join(", ")));
    }

    r.section("Failures and breakers");
    for l in events_log.lines().filter(|l| l.contains("[failure]") || l.contains("[breaker]")).take(60) {
        r.info(l.trim().to_string());
    }

    r.section("Errors in the log");
    let errors: Vec<&str> = events_log.lines().filter(|l| {
        let low = l.to_ascii_lowercase();
        let noise = low.contains("warning") || low.contains("rmpstate") || low.contains("first read of unwritten");
        !noise && ["panic", "error:", "failed to", "failed:", "crash", "not answering"].iter().any(|k| low.contains(k))
    }).collect();
    if errors.is_empty() {
        r.ok("no error lines".into());
    } else {
        r.problem(format!("{} error lines; the first ones:", errors.len()));
        for e in errors.iter().take(8) {
            r.info(e.trim().to_string());
        }
    }

    r.section("Your marks");
    for l in events_log.lines().filter(|l| l.contains("[MARK]")) {
        r.info(l.trim().to_string());
    }

    let mut out = format!("A380 for X-Plane 12 diagnostics health report: {} problem(s)\n", r.problems);
    out.push_str(&r.lines.join("\n"));
    out.push('\n');
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    fn csv(rows: &[&str]) -> String {
        let header = "t,fps,plugin_tick_ms,plugin_draw_ms,sim/flightmodel/failures/onground_any,sim/flightmodel/position/groundspeed,fbw/A32NX_ADIRS_SENSED_1_GROUND_SPEED,fbw/A32NX_WING_FLEX_LEFT_INBOARD_MID,fbw/A32NX_ENGINE_STATE_1,sim/flightmodel/engine/ENGN_FF_[0],fbw/GENERAL_ENG_STARTER_1,fbw/cockpit/SWITCH_OVHD_EXTLT_STROBE,sim/cockpit2/switches/strobe_lights_on";
        std::iter::once(header).chain(rows.iter().copied()).collect::<Vec<_>>().join("\n")
    }

    #[test]
    fn a_healthy_parked_session_reports_no_problems_from_the_samples() {
        let samples = csv(&["1,30,3,1,1,0,0.1,-0.2,1,0.17,1,1,0", "2,30,3,1,1,0,0.2,-0.2,1,0.17,1,0,1"]);
        let out = report(&samples, "1 [log] sound: 150 voices (60 audible); loudest: media 1 trigger 2 -10 dB pitch x1.00\n", "1,XPHFBW.exe,27,1,1,1,1\n");
        assert!(out.starts_with("A380 for X-Plane 12 diagnostics health report: 0 problem(s)"), "{out}");
    }

    #[test]
    fn the_bugs_this_session_fixed_each_show_as_a_problem() {
        let samples = csv(&["1,8,85,2,1,0,9.5,9.9,1,0.0,1,1,1", "2,8,85,2,1,0,10.0,9.9,0,0.0,1,1,1"]);
        let out = report(&samples, "1 [log] sound: loudest: media 9 trigger 3 -5 dB pitch x4.10\n", "1,XPHFBW.exe,27,1,1,1,1\n3,XPHFBW.exe,0,0,0,0,0\n");
        for needle in ["plugin takes over 10 ms", "IR ground speed reached 10.0 kt", "wing flex parked +9.90", "engine 1 running with no fuel flow", "engine 1 stopped at 2 s", "STROBE lit with its switch at OFF", "4.10x its recorded pitch", "XPHFBW disappeared at 3 s"] {
            assert!(out.contains(needle), "missing {needle:?} in\n{out}");
        }
    }
}

#[cfg(test)]
mod session_tool {
    #[test]
    #[ignore]
    fn report_for_a_session_folder() {
        let Ok(dir) = std::env::var("DIAG_SESSION") else { return };
        let read = |n: &str| std::fs::read_to_string(std::path::Path::new(&dir).join(n)).unwrap_or_default();
        println!("{}", super::report(&read("samples.csv"), &read("events.log"), &read("processes.csv")));
    }
}
