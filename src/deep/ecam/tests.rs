//! Worked example: three representative alerts (one of each level that
//! actually reaches FlyByWire's abnormal-procedure table -- Warning,
//! Caution, Advisory) carried end to end through `ids::assign`,
//! every `codegen` function, and `patches::source_patches`, checking that
//! every piece agrees: the same ids appear in the static text merges, the
//! behaviour array and the generated patches, and nothing from one alert
//! leaks into another's entry.

use crate::deep::api::{line, var, EcamAlert, Level, Phase};
use crate::deep::ecam::{codegen, ids, patches};

fn oil_pressure_warning() -> EcamAlert {
    EcamAlert::new("ENG_2_OIL_LO_PR", 79, "ENG 2 OIL LO PR", Level::Warning, var("ENGINE_OIL_PRESSURE_PSI:2").lt(25.0))
        .confirm(2.0)
        .inhibit(&[Phase::LiftOff])
        .step(line("THR LEVER 2", "IDLE").done(var("AUTOTHRUST_TLA:2").le(0.0)))
        .step(line("ENG 2 MASTER", "OFF").done(var("ENGINE_MASTER:2").off()).after(30.0))
        .inop_sys("ENG 2 OIL SYSTEM")
}

fn hydraulic_caution() -> EcamAlert {
    EcamAlert::new("GREEN_RSVR_LOW", 29, "GREEN RSVR LO LEVEL", Level::Caution, var("A32NX_HYD_GREEN_RESERVOIR_LEVEL").lt(0.1))
        .confirm(5.0)
        .step(line("GREEN ELEC PUMP", "OFF").done(var("A32NX_OVHD_HYD_EPUMPG_ON_PB_IS_AUTO").off()))
        .status_line("GREEN HYD SYS")
        .inop_sys("GREEN HYD SYS")
}

fn fuel_advisory() -> EcamAlert {
    EcamAlert::new("ENG_3_FUEL_FILTER_CLOG", 73, "ENG 3 FUEL FILTER CLOG", Level::Advisory, var("A32NX_ENG_3_FUEL_FILTER_IMPENDING_BYPASS").on()).confirm(10.0).status_line("ENG 3 FUEL FILTER")
}

fn the_three_alerts() -> Vec<EcamAlert> {
    vec![oil_pressure_warning(), hydraulic_caution(), fuel_advisory()]
}

#[test]
fn ids_are_assigned_uniquely_and_deterministically_across_all_three() {
    let alerts = the_three_alerts();
    let assigned = ids::assign(&alerts);
    assert_eq!(assigned.len(), 3);
    // Sorted by key: "ENG_2_OIL_LO_PR" < "ENG_3_FUEL_FILTER_CLOG" < "GREEN_RSVR_LOW".
    assert_eq!(assigned[0].alert.key, "ENG_2_OIL_LO_PR");
    assert_eq!(assigned[1].alert.key, "ENG_3_FUEL_FILTER_CLOG");
    assert_eq!(assigned[2].alert.key, "GREEN_RSVR_LOW");
    assert_eq!(assigned[0].id, ids::ID_BASE);
    assert_eq!(assigned[1].id, ids::ID_BASE + 1);
    assert_eq!(assigned[2].id, ids::ID_BASE + 2);
}

#[test]
fn every_alert_appears_with_the_same_id_in_the_static_and_behaviour_outputs() {
    let alerts = the_three_alerts();
    let assigned = ids::assign(&alerts);

    let proc_js = codegen::procedures_merge_js(&assigned);
    let behaviour_js = codegen::alerts_js_array(&assigned);
    let inop_js = codegen::inop_merge_js(&assigned);
    let info_js = codegen::info_merge_js(&assigned);

    for a in &assigned {
        let id = a.id;
        assert!(proc_js.contains(&format!("{id}:{{title:")), "procedure entry for {} at id {id}", a.alert.key);
        assert!(behaviour_js.contains(&format!("{{id:{id},")), "behaviour entry for {} at id {id}", a.alert.key);
    }

    // Only the two alerts that registered INOP/STATUS lines produced any
    // entries in those dicts, at ids derived from their own procedure id.
    let oil = &assigned[0]; // ENG_2_OIL_LO_PR
    let fuel = &assigned[1]; // ENG_3_FUEL_FILTER_CLOG
    let hyd = &assigned[2]; // GREEN_RSVR_LOW
    assert!(inop_js.contains(&format!("{}:'ENG 2 OIL SYSTEM'", oil.id * 1000)));
    assert!(inop_js.contains(&format!("{}:'GREEN HYD SYS'", hyd.id * 1000)));
    assert!(!inop_js.contains("FUEL FILTER"), "the advisory registered no inop_sys line");
    assert!(info_js.contains(&format!("{}:'GREEN HYD SYS'", hyd.id * 1000 + 500)));
    assert!(info_js.contains(&format!("{}:'ENG 3 FUEL FILTER'", fuel.id * 1000 + 500)));
}

#[test]
fn the_three_alerts_map_to_the_right_failure_level_and_carry_their_own_procedure_only() {
    let alerts = the_three_alerts();
    let assigned = ids::assign(&alerts);
    let behaviour_js = codegen::alerts_js_array(&assigned);
    // Split on the top-level array entries by their id marker to check each
    // alert's own slice carries only its own confirm delay and phase list.
    let oil_entry = behaviour_js.split(&format!("id:{}", assigned[0].id)).nth(1).unwrap();
    assert!(oil_entry.starts_with(",failure:3"), "ENG_2_OIL_LO_PR is a Warning");
    assert!(oil_entry.contains("confirmS:2"));
    assert!(oil_entry.contains("flightPhaseInhib:[6]"), "LiftOff=6");

    let hyd_entry = behaviour_js.split(&format!("id:{}", assigned[2].id)).nth(1).unwrap();
    assert!(hyd_entry.starts_with(",failure:2"), "GREEN_RSVR_LOW is a Caution");
    assert!(hyd_entry.contains("confirmS:5"));

    let fuel_entry = behaviour_js.split(&format!("id:{}", assigned[1].id)).nth(1).unwrap();
    assert!(fuel_entry.starts_with(",failure:1"), "ENG_3_FUEL_FILTER_CLOG is an Advisory");
    assert!(fuel_entry.contains("confirmS:10"));
}

#[test]
fn the_full_patch_set_carries_all_three_alerts_and_stays_well_formed() {
    let alerts = the_three_alerts();
    let generated = patches::source_patches(&alerts);
    assert_eq!(generated.len(), 5);

    let ewd_titles = &generated.iter().find(|p| p.path.ends_with("ewd.js")).unwrap().replace;
    for title in ["ENG 2 OIL LO PR", "GREEN RSVR LO LEVEL", "ENG 3 FUEL FILTER CLOG"] {
        assert!(ewd_titles.contains(title), "EWD.js patch must carry {title}");
    }

    // Every SystemsHost.js patch's replace text is still balanced braces,
    // brackets and parens once `//` line comments are stripped (English
    // prose in a comment is not held to JS's own balance rules) -- a cheap
    // syntactic sanity check without a JS parser.
    fn strip_line_comments(src: &str) -> String {
        src.lines()
            .map(|line| match line.find("//") {
                Some(i) => &line[..i],
                None => line,
            })
            .collect::<Vec<_>>()
            .join("\n")
    }
    for p in generated.iter().filter(|p| p.path.contains("SystemsHost")) {
        let code = strip_line_comments(&p.replace);
        let (mut curly, mut square, mut paren) = (0i32, 0i32, 0i32);
        for ch in code.chars() {
            match ch {
                '{' => curly += 1,
                '}' => curly -= 1,
                '[' => square += 1,
                ']' => square -= 1,
                '(' => paren += 1,
                ')' => paren -= 1,
                _ => {}
            }
        }
        assert_eq!(curly, 0, "unbalanced {{}} in patch: {}", p.reason);
        assert_eq!(square, 0, "unbalanced [] in patch: {}", p.reason);
        assert_eq!(paren, 0, "unbalanced () in patch: {}", p.reason);
    }
}
