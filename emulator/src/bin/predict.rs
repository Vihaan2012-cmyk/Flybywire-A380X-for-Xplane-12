//! Predict the outcome of any combination of failures and breakers, of any
//! size, from a finished tiered battery run (`battery --tiered`), without
//! running it.
//!
//!     predict <run dir> "preset|f1,f2,...|b1,b2,..."
//!
//! `preset` is the start state (0 cold and dark, 1 ground power, 2 APU
//! powered, 3 engines running). What the run measured decides, in order:
//!
//! 1. **measured**: this exact combination was run: its real result.
//! 2. **fails (certain)**: an element failed alone on this start state, or a
//!    combination inside it was run and failed.
//! 3. **passes (confident)**: every pair inside it was run and passed, or
//!    cannot interact (the variables each moves alone do not overlap,
//!    ignoring ones most faults move).
//! 4. **passes (uncertain)**: it contains coupled pairs that were never run;
//!    they are listed. The run's tier-5 score (TIERS.txt) is how often a
//!    prediction like this held for combinations of 5-12.

use std::collections::{BTreeMap, BTreeSet};
use std::path::Path;

type Elements = BTreeSet<String>;

fn parse_spec(line: &str) -> Option<(usize, Elements)> {
    let mut parts = line.trim().splitn(3, '|');
    let preset = parts.next()?.trim().parse().ok()?;
    let mut els = BTreeSet::new();
    for f in parts.next().unwrap_or("").split(',').filter(|x| !x.trim().is_empty()) {
        els.insert(format!("f:{}", f.trim()));
    }
    for b in parts.next().unwrap_or("").split(',').filter(|x| !x.trim().is_empty()) {
        els.insert(format!("b:{}", b.trim()));
    }
    Some((preset, els))
}

struct Run {
    /// Every combination run, all tiers: its result.
    tested: BTreeMap<(usize, Elements), bool>,
    /// Each single's moved variables, per start state.
    reach: BTreeMap<(usize, String), BTreeSet<String>>,
    /// Variables more than a quarter of singles move.
    hubs: BTreeSet<String>,
}

fn load(dir: &Path) -> Run {
    let mut tested = BTreeMap::new();
    let mut reach = BTreeMap::new();
    let mut df: BTreeMap<String, usize> = BTreeMap::new();
    let mut singles = 0usize;
    for tier in 1..=5 {
        let tdir = dir.join(format!("tier_{tier}"));
        let Ok(plan) = std::fs::read_to_string(tdir.join("plan.txt")) else { continue };
        let specs: Vec<Option<(usize, Elements)>> = plan.lines().map(parse_spec).collect();
        for entry in std::fs::read_dir(&tdir).into_iter().flatten().flatten() {
            let name = entry.file_name().to_string_lossy().into_owned();
            if !(name.starts_with("reach_") && name.ends_with(".txt")) {
                continue;
            }
            for line in std::fs::read_to_string(entry.path()).unwrap_or_default().lines() {
                let mut f = line.splitn(3, '\t');
                let (Some(i), Some(ok)) = (f.next().and_then(|x| x.parse::<usize>().ok()), f.next()) else { continue };
                let Some(Some((preset, els))) = specs.get(i) else { continue };
                tested.insert((*preset, els.clone()), ok == "true");
                if tier == 1 && els.len() == 1 {
                    let vars: BTreeSet<String> = f.next().unwrap_or("").split(',').filter(|x| !x.is_empty()).map(str::to_owned).collect();
                    for v in &vars {
                        *df.entry(v.clone()).or_insert(0) += 1;
                    }
                    singles += 1;
                    reach.insert((*preset, els.iter().next().cloned().unwrap_or_default()), vars);
                }
            }
        }
    }
    let hubs = df.into_iter().filter(|(_, n)| *n as f64 > 0.25 * singles.max(1) as f64).map(|(v, _)| v).collect();
    Run { tested, reach, hubs }
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let (Some(dir), Some(spec)) = (args.get(1), args.get(2)) else {
        eprintln!("usage: predict <run dir> \"preset|f1,f2|b1,b2\"");
        std::process::exit(2);
    };
    let Some((preset, els)) = parse_spec(spec) else {
        eprintln!("combination must be preset|failures|breakers");
        std::process::exit(2);
    };
    let run = load(Path::new(dir));
    println!("{} combinations measured in this run", run.tested.len());

    if let Some(&passed) = run.tested.get(&(preset, els.clone())) {
        println!("measured: {}", if passed { "PASSES" } else { "FAILS" });
        return;
    }
    let alone: Vec<&String> = els.iter().filter(|e| run.tested.get(&(preset, BTreeSet::from([(*e).clone()]))) == Some(&false)).collect();
    let failing_inside: Vec<&Elements> = run
        .tested
        .iter()
        .filter(|((p, set), passed)| *p == preset && !**passed && set.len() > 1 && set.is_subset(&els))
        .map(|((_, set), _)| set)
        .collect();
    if !alone.is_empty() || !failing_inside.is_empty() {
        println!("fails (certain)");
        for e in alone {
            println!("  {e} fails alone on this start state");
        }
        for set in failing_inside.iter().take(10) {
            println!("  contains {} , which was run and failed", set.iter().cloned().collect::<Vec<_>>().join(" + "));
        }
        return;
    }
    let list: Vec<&String> = els.iter().collect();
    let mut untested_coupled = Vec::new();
    for i in 0..list.len() {
        for j in (i + 1)..list.len() {
            let pair = BTreeSet::from([list[i].clone(), list[j].clone()]);
            if run.tested.contains_key(&(preset, pair)) {
                continue;
            }
            let a = run.reach.get(&(preset, list[i].clone()));
            let b = run.reach.get(&(preset, list[j].clone()));
            let coupled = match (a, b) {
                (Some(a), Some(b)) => a.intersection(b).any(|v| !run.hubs.contains(v)),
                _ => true,
            };
            if coupled {
                untested_coupled.push(format!("{} + {}", list[i], list[j]));
            }
        }
    }
    if untested_coupled.is_empty() {
        println!("passes (confident): no element fails alone, no failing combination inside it, and every pair inside it was run clean or cannot interact");
    } else {
        println!("passes (uncertain): {} coupled pairs inside it were never run:", untested_coupled.len());
        for p in untested_coupled.iter().take(20) {
            println!("  {p}");
        }
    }
}
