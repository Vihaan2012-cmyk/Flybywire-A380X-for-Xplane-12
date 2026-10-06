//! A development tool, not a check: writes out the two lists anyone wiring
//! an ECAM trigger has to work against, so the question "can this alert be
//! raised from what the aircraft actually publishes?" is answered from a
//! file rather than from memory.
//!
//! Run it with
//!
//! ```text
//! CARGO_TARGET_DIR=D:/A380/fbw-xp-systems/target-j1 \
//!   cargo test --lib deep::ecam::dump_published -- --nocapture
//! ```
//!
//! and it prints where it put them (the system temp directory, the same
//! place `integration::failure_audit`'s own reports go):
//!
//! * `deep_published_vars.txt` -- every name `Deep::published_names()`
//!   reports, one per line, sorted. This set is authoritative: a trigger
//!   naming anything outside it reads 0 for ever.
//! * `deep_own_alerts.txt` -- every alert `deep::registry()` carries, as
//!   `ATA \t key \t level \t title \t trigger variables`. This is the list
//!   to check a FlyByWire procedure against before wiring it, so the crew
//!   is not shown the same warning twice under two ids.
//!
//! It asserts only what a dump can honestly assert -- that there is
//! something to dump -- so it can never fail for a reason that is not about
//! this tool.

#[cfg(test)]
mod t {
    fn report_path(name: &str) -> std::path::PathBuf {
        std::env::temp_dir().join(name)
    }

    #[test]
    fn dump_the_published_variables_and_our_own_alerts() {
        let deep = crate::deep::live::all_areas();
        let mut names: Vec<String> = deep.published_names();
        names.sort();
        names.dedup();
        assert!(!names.is_empty(), "no area published anything");
        let vars = report_path("deep_published_vars.txt");
        std::fs::write(&vars, names.join("\n")).expect("write the published-variable list");
        println!("DUMP {} published variables at {}", names.len(), vars.display());

        let r = crate::deep::registry();
        let mut lines: Vec<String> = r
            .alerts
            .iter()
            .map(|a| {
                let t = crate::deep::integration::failure_audit::trigger_vars(a);
                format!("{}\t{}\t{:?}\t{}\t{}", a.ata, a.key, a.level, a.title, t.join(","))
            })
            .collect();
        lines.sort();
        assert!(!lines.is_empty(), "the registry carries no alerts");
        let alerts = report_path("deep_own_alerts.txt");
        std::fs::write(&alerts, lines.join("\n")).expect("write our own alert list");
        println!("DUMP {} of our own ECAM alerts at {}", lines.len(), alerts.display());
    }
}
