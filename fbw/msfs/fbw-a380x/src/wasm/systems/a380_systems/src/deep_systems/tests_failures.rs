#[allow(unused_imports)]
use std::time::Duration;

#[allow(unused_imports)]
use systems::simulation::test::{ReadByName, SimulationTestBed, TestBed, WriteByName};

#[allow(unused_imports)]
use super::tests::{aircraft, arm, computers_healthy, failure_id, run};
#[allow(unused_imports)]
use crate::A380;

#[test]
#[ignore]
fn provoked_audit() {
    use deep_systems::deep::integration::provocation::{provoked_verdict, ProvokedVerdict};
    use std::fmt::Write as _;

    deep_systems::set_log_quiet(true);
    let dir_in = std::path::Path::new("D:/A380/msfs-a380/reports/dead-failures");
    let dir_out = std::path::Path::new("D:/A380/msfs-a380/reports/dead-failures-v2");
    std::fs::create_dir_all(dir_out).ok();
    let areas: Vec<String> = match std::env::var("DEEP_AUDIT_AREA") {
        Ok(a) if !a.is_empty() => a.split(',').map(|s| s.trim().to_owned()).collect(),
        _ => ["breakers", "electrical", "avionics", "fire-gear-apu", "engines", "pneumatic", "fuel", "hydraulics", "flight-controls"]
            .iter()
            .map(|s| s.to_string())
            .collect(),
    };
    let registry = deep_systems::deep::registry();
    for area in areas {
        let Ok(text) = std::fs::read_to_string(dir_in.join(format!("{area}.md"))) else {
            println!("{area}: no dead list");
            continue;
        };
        let ids: Vec<u64> = text
            .lines()
            .filter_map(|l| l.strip_prefix("## "))
            .filter_map(|l| l.split(" -- ").next())
            .filter_map(|id| id.trim().parse().ok())
            .collect();
        let only: Option<Vec<u64>> = std::env::var("DEEP_AUDIT_IDS").ok().filter(|v| !v.is_empty()).map(|v| v.split(',').filter_map(|id| id.trim().parse().ok()).collect());
        let mut failures: Vec<_> = ids
            .iter()
            .filter(|id| only.as_ref().map_or(true, |only| only.contains(id)))
            .filter_map(|id| registry.failures.iter().find(|f| f.id == *id).cloned())
            .collect();
        if failures.is_empty() {
            println!("{area}: none of DEEP_AUDIT_IDS in its list");
            continue;
        }
        let frames_of = |f: &deep_systems::deep::api::FailureDef| deep_systems::deep::integration::provocation::for_failure(f).map_or(0, |p| (p.provoke)(f, &registry).frames);
        failures.sort_by_key(|f| std::cmp::Reverse(frames_of(f)));
        let threads = std::thread::available_parallelism().map(|n| n.get().min(8)).unwrap_or(4).max(1);
        let next = std::sync::atomic::AtomicUsize::new(0);
        let done = std::sync::Mutex::new(Vec::with_capacity(failures.len()));
        std::thread::scope(|s| {
            for _ in 0..threads {
                s.spawn(|| {
                    let registry = deep_systems::deep::registry();
                    loop {
                        let i = next.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
                        let Some(f) = failures.get(i) else { break };
                        let v = provoked_verdict(f, &registry);
                        done.lock().expect("provoked audit results").push((f.id, f.clone(), v));
                    }
                });
            }
        });
        let mut done = done.into_inner().expect("provoked audit results");
        done.sort_by_key(|(id, _, _)| *id);
        let verdicts: Vec<(deep_systems::deep::api::FailureDef, ProvokedVerdict)> = done.into_iter().map(|(_, f, v)| (f, v)).collect();
        let (mut live, mut still, mut none) = (String::new(), String::new(), String::new());
        let (mut n_live, mut n_still, mut n_none) = (0, 0, 0);
        for (f, v) in &verdicts {
            match v {
                ProvokedVerdict::Live { why, magnitude, moved, moved_count } => {
                    n_live += 1;
                    let _ = writeln!(live, "- {} {} -- at {magnitude}, {moved_count} moved {moved:?} -- {why}", f.id, f.name);
                }
                ProvokedVerdict::StillDead { why } => {
                    n_still += 1;
                    let _ = writeln!(still, "- {} {} (`{}`) -- provoked by: {why}", f.id, f.name, f.model_field);
                }
                ProvokedVerdict::UnknownProfile(p) => {
                    n_still += 1;
                    let _ = writeln!(still, "- {} {} -- provocation names unknown profile `{p}`", f.id, f.name);
                }
                ProvokedVerdict::NoProvocation => {
                    n_none += 1;
                    let _ = writeln!(none, "- {} {} (`{}`)", f.id, f.name, f.model_field);
                }
            }
        }
        let report = format!(
            "# Dead failures under provocation: {area}\n\n{} were dead alone: {n_live} live when provoked, {n_still} still dead when provoked, {n_none} with no provocation yet.\n\n## Still dead when provoked (real gaps)\n\n{still}\n## No provocation yet\n\n{none}\n## Live when provoked\n\n{live}",
            verdicts.len()
        );
        let file = if only.is_some() { format!("{area}-subset.md") } else { format!("{area}.md") };
        std::fs::write(dir_out.join(file), report).ok();
        println!("{area}: {} dead alone -> {n_live} live when provoked, {n_still} still dead, {n_none} no provocation", verdicts.len());
    }
}

#[test]
#[ignore]
fn export_consequences() {
    use deep_systems::deep::integration::consequences::{Consequences, Tracer};
    use std::fmt::Write as _;

    enum Job {
        Failure(deep_systems::deep::api::FailureDef),
        Breaker(&'static str),
    }

    fn json_str(out: &mut String, s: &str) {
        out.push('"');
        for c in s.chars() {
            match c {
                '"' => out.push_str("\\\""),
                '\\' => out.push_str("\\\\"),
                c if (c as u32) < 0x20 => {
                    let _ = write!(out, "\\u{:04x}", c as u32);
                }
                c => out.push(c),
            }
        }
        out.push('"');
    }

    fn json_consequences(out: &mut String, c: &Consequences) {
        out.push_str("{\"p\":");
        json_str(out, c.profile);
        out.push_str(",\"s\":[");
        for (i, stage) in c.stages.iter().enumerate() {
            out.push_str(if i == 0 { "[" } else { ",[" });
            for (j, a) in stage.iter().enumerate() {
                out.push_str(if j == 0 { "[" } else { ",[" });
                json_str(out, a.area);
                let _ = write!(out, ",{},[", a.moved);
                for (k, e) in a.examples.iter().enumerate() {
                    if k > 0 {
                        out.push(',');
                    }
                    json_str(out, e);
                }
                out.push_str("]]");
            }
            out.push(']');
        }
        out.push_str("],\"a\":[");
        for (i, key) in c.alerts.iter().enumerate() {
            if i > 0 {
                out.push(',');
            }
            json_str(out, key);
        }
        out.push_str("],\"f\":[");
        for (i, f) in c.fbw.iter().enumerate() {
            let _ = write!(out, "{}[{},", if i > 0 { "," } else { "" }, f.fbw_id);
            json_str(out, f.deep_component);
            out.push(',');
            json_str(out, f.reason);
            out.push(']');
        }
        out.push(']');
        if let Some(d) = c.develops_over_s {
            let _ = write!(out, ",\"d\":{d:.0}");
        }
        out.push('}');
    }

    deep_systems::set_log_quiet(true);
    {
        let t = std::time::Instant::now();
        let mut deep = deep_systems::deep::integration::failure_audit::fresh_areas();
        let build = t.elapsed();
        let truth = (deep_systems::deep::integration::failure_audit::profiles()[0].truth)();
        let faults = deep_systems::deep::integration::failure_audit::reference_faults();
        let t = std::time::Instant::now();
        for _ in 0..50 {
            deep.tick(truth.clone(), &faults, &mut |_, _| {});
        }
        println!("timing: build {:.1} ms, tick {:.2} ms", build.as_secs_f64() * 1e3, t.elapsed().as_secs_f64() * 1e3 / 50.0);
        if std::env::var("DEEP_CONSEQ_TIMING_ONLY").is_ok() {
            return;
        }
    }
    let started = std::time::Instant::now();
    let registry = deep_systems::deep::registry();
    let dead_in_profiles: std::collections::BTreeSet<u64> = std::fs::read_dir("D:/A380/msfs-a380/reports/dead-failures")
        .map(|dir| {
            dir.flatten()
                .filter_map(|e| std::fs::read_to_string(e.path()).ok())
                .flat_map(|text| text.lines().filter_map(|l| l.strip_prefix("## ")?.split(" -- ").next()?.trim().parse().ok()).collect::<Vec<u64>>())
                .collect()
        })
        .unwrap_or_default();
    println!("{} ids known dead in every profile", dead_in_profiles.len());
    let mut jobs: Vec<Job> = registry.failures.iter().filter(|f| !deep_systems::msfs_excluded::is_msfs_excluded(f.id)).cloned().map(Job::Failure).collect();
    let catalogue: serde_json::Value =
        serde_json::from_str(&std::fs::read_to_string("D:/A380/msfs-a380/install/out/EFB/catalogue.json").expect("the EFB catalogue is built")).unwrap();
    for &id in super::consumer_failures::CONSUMER_FAILURES {
        if registry.failures.iter().any(|f| f.id == id) {
            continue;
        }
        let Some(entry) = catalogue["failures"].as_array().and_then(|fs| fs.iter().find(|f| f["id"].as_u64() == Some(id))) else { continue };
        jobs.push(Job::Failure(deep_systems::deep::api::FailureDef {
            id,
            area: deep_systems::deep::api::Area::Integration,
            ata: entry["ataChapterNumber"].as_u64().unwrap_or(0) as u16,
            name: entry["name"].as_str().unwrap_or_default().to_owned(),
            component: entry["component"].as_str().unwrap_or_default().to_owned(),
            model_field: String::new(),
            magnitude: String::new(),
            effect: String::new(),
        }));
    }
    let breaker_ids: Vec<&'static str> = deep_systems::deep::integration::failure_audit::fresh_areas().breakers().map(|b| b.units().map(|(id, _, _)| id).collect()).unwrap_or_default();
    jobs.extend(breaker_ids.iter().map(|&id| Job::Breaker(id)));

    let threads = std::thread::available_parallelism().map(|n| n.get()).unwrap_or(4).max(1);
    let next = std::sync::atomic::AtomicUsize::new(0);
    let finished = std::sync::atomic::AtomicUsize::new(0);
    let done = std::sync::Mutex::new(Vec::<(usize, Option<Consequences>)>::with_capacity(jobs.len()));
    std::thread::scope(|s| {
        for _ in 0..threads {
            s.spawn(|| {
                let registry = deep_systems::deep::registry();
                let tracer = Tracer::new(&registry);
                loop {
                    let i = next.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
                    let Some(job) = jobs.get(i) else { break };
                    let c = match job {
                        Job::Failure(f) => tracer.failure(f, &registry, dead_in_profiles.contains(&f.id)),
                        Job::Breaker(id) => tracer.breaker(id),
                    };
                    done.lock().expect("consequence results").push((i, c));
                    let n = finished.fetch_add(1, std::sync::atomic::Ordering::Relaxed) + 1;
                    if n % 250 == 0 || n == jobs.len() {
                        println!("progress: {n}/{} after {:.0} s", jobs.len(), started.elapsed().as_secs_f64());
                    }
                }
            });
        }
    });
    let mut done = done.into_inner().expect("consequence results");
    done.sort_by_key(|(i, _)| *i);

    let (mut failures, mut breakers) = (String::new(), String::new());
    let (mut n_failures, mut n_breakers, mut n_none) = (0usize, 0usize, 0usize);
    for (i, c) in &done {
        let Some(c) = c else {
            n_none += 1;
            continue;
        };
        let (out, n, key) = match &jobs[*i] {
            Job::Failure(f) => (&mut failures, &mut n_failures, f.id.to_string()),
            Job::Breaker(id) => (&mut breakers, &mut n_breakers, (*id).to_owned()),
        };
        if *n > 0 {
            out.push(',');
        }
        *n += 1;
        json_str(out, &key);
        out.push(':');
        json_consequences(out, c);
    }
    let json = format!("{{\"schemaVersion\":1,\"failures\":{{{failures}}},\"breakers\":{{{breakers}}}}}");
    let path = std::path::Path::new("D:/A380/msfs-a380/install/out/EFB/consequences.json");
    std::fs::write(path, &json).expect("write consequences.json");

    let mut values = String::new();
    for (i, c) in &done {
        let (Some(c), Job::Failure(f)) = (c, &jobs[*i]) else { continue };
        if c.values.is_empty() {
            continue;
        }
        if !values.is_empty() {
            values.push(',');
        }
        json_str(&mut values, &f.id.to_string());
        values.push_str(":[");
        for (k, (name, healthy, faulted)) in c.values.iter().enumerate() {
            if k > 0 {
                values.push(',');
            }
            values.push('[');
            json_str(&mut values, name);
            let num = |v: f64| if v.is_finite() { format!("{v}") } else { "null".to_owned() };
            values.push_str(&format!(",{},{}]", num(*healthy), num(*faulted)));
        }
        values.push(']');
    }
    std::fs::write("D:/A380/msfs-a380/wiring/consequence-values.json", format!("{{{values}}}")).expect("write consequence-values.json");
    println!(
        "consequences: {n_failures} failures, {n_breakers} breakers charted, {n_none} with no modelled consequence; {} KB in {:.0} s on {threads} threads",
        json.len() / 1024,
        started.elapsed().as_secs_f64()
    );
}

#[test]
fn two_identical_aircraft_stepped_side_by_side_never_differ() {
    use deep_systems::deep::integration::{consequences::stepped_self_check, failure_audit::profiles};
    for p in profiles().into_iter().take(3) {
        let found = stepped_self_check(&(p.truth)(), 20);
        assert!(found.is_none(), "{}: identical aircraft differed: {:?}", p.name, found.map(|c| c.stages));
    }
}
