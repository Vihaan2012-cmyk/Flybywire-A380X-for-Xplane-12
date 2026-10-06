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
