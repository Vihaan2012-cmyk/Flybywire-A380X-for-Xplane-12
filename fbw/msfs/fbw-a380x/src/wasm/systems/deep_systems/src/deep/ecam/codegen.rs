use crate::deep::api::{Cond, EcamAlert, Level, Phase, ProcLine};
use crate::deep::ecam::cond_json::{cond_to_js, js_string};
use crate::deep::ecam::ids::Assigned;

fn failure_number(level: Level) -> u8 {
    match level {
        Level::Warning => 3,
        Level::Caution => 2,
        Level::Advisory | Level::Memo => 1,
    }
}

fn fwc_phase_number(p: Phase) -> u32 {
    match p {
        Phase::ElecPower => 1,
        Phase::FirstEngineStarted => 2,
        Phase::FirstEngineTakeoffPower => 3,
        Phase::Above80Kt => 4,
        Phase::LiftOff => 6,
        Phase::Above1500Ft => 8,
        Phase::Below800Ft => 9,
        Phase::Touchdown => 10,
        Phase::Below80Kt => 11,
        Phase::SecondEngineShutdown => 12,
    }
}

fn sys_page_for_ata(ata: u16) -> i32 {
    match ata {
        21 | 30 => 3,
        22 | 23 | 31 | 33 | 34 | 45 | 46 => 14,
        24 => 6,
        25 => 5,
        26 => 14,
        27 => 11,
        28 => 8,
        29 => 10,
        32 => 9,
        35 | 36 => 2,
        49 => 1,
        52 => 5,
        70..=80 => 0,
        _ => 14,
    }
}

fn style_js(colour: &str) -> &'static str {
    match colour {
        "cyan" => "Cyan",
        "white" => "White",
        "green" => "Green",
        "amber" => "Amber",
        "red" => "Red",
        _ => "Standard",
    }
}

fn num(v: f64) -> String {
    if v == v.trunc() && v.is_finite() && v.abs() < 1e15 {
        format!("{}", v as i64)
    } else {
        format!("{v}")
    }
}

fn sub_ids(alert_id: u64, offset: u64, count: usize) -> Vec<u64> {
    (0..count as u64).map(|i| alert_id * 1000 + offset + i).collect()
}

fn inop_ids(alert_id: u64, alert: &EcamAlert) -> Vec<u64> {
    sub_ids(alert_id, 0, alert.inop.len())
}

fn info_ids(alert_id: u64, alert: &EcamAlert) -> Vec<u64> {
    sub_ids(alert_id, 500, alert.status.len())
}

fn id_list_js(ids: &[u64]) -> String {
    format!("[{}]", ids.iter().map(u64::to_string).collect::<Vec<_>>().join(","))
}

fn procedure_item_js(item: &ProcLine) -> String {
    let mut fields = vec![format!("name:{}", js_string(&item.text)), format!("sensed:{}", item.done_when.is_some())];
    if !item.action_text.is_empty() {
        fields.push(format!("labelNotCompleted:{}", js_string(&item.action_text)));
    }
    fields.push(format!("style:{}", js_string(style_js(item.colour))));
    format!("{{{}}}", fields.join(","))
}

fn procedure_entry_js(a: &Assigned<'_>) -> String {
    let items = a.alert.procedure.iter().map(procedure_item_js).collect::<Vec<_>>().join(",");
    format!("{}:{{title:{},sensed:true,items:[{}]}}", a.id, js_string(&a.alert.title), items)
}

pub fn procedures_merge_js(assigned: &[Assigned<'_>]) -> String {
    format!("{{{}}}", assigned.iter().map(procedure_entry_js).collect::<Vec<_>>().join(","))
}

pub fn inop_merge_js(assigned: &[Assigned<'_>]) -> String {
    let mut out = Vec::new();
    for a in assigned {
        for (id, text) in inop_ids(a.id, a.alert).iter().zip(a.alert.inop.iter()) {
            out.push(format!("{}:{}", id, js_string(text)));
        }
    }
    format!("{{{}}}", out.join(","))
}

pub fn info_merge_js(assigned: &[Assigned<'_>]) -> String {
    let mut out = Vec::new();
    for a in assigned {
        for (id, text) in info_ids(a.id, a.alert).iter().zip(a.alert.status.iter()) {
            out.push(format!("{}:{}", id, js_string(text)));
        }
    }
    format!("{{{}}}", out.join(","))
}

fn behaviour_item_js(item: &ProcLine) -> String {
    let done_when = match &item.done_when {
        Some(c) => cond_to_js(c),
        None => "null".to_string(),
    };
    format!("{{appliesIf:{},doneWhen:{},afterS:{}}}", cond_to_js(&item.applies_if), done_when, num(item.after_s))
}

fn behaviour_entry_js(a: &Assigned<'_>) -> String {
    let alert: &EcamAlert = a.alert;
    let phases: Vec<u32> = {
        let mut v: Vec<u32> = match &alert.fcom_phases {
            Some(fcom) => fcom.clone(),
            None => alert.inhibited_in.iter().map(|p| fwc_phase_number(*p)).collect(),
        };
        v.sort_unstable();
        v.dedup();
        v
    };
    let items_js = alert.procedure.iter().map(behaviour_item_js).collect::<Vec<_>>().join(",");
    format!(
        "{{id:{},failure:{},sysPage:{},flightPhaseInhib:[{}],confirmS:{},notActiveWhenItemActive:[{}],trigger:{},items:[{}],inopIds:{},infoIds:{}}}",
        a.id,
        failure_number(alert.level),
        sys_page_for_ata(alert.ata),
        phases.iter().map(u32::to_string).collect::<Vec<_>>().join(","),
        num(alert.confirm_s),
        alert.suppressed_by.iter().map(|id| format!("'{id}'")).collect::<Vec<_>>().join(","),
        cond_to_js(&alert.trigger),
        items_js,
        id_list_js(&inop_ids(a.id, alert)),
        id_list_js(&info_ids(a.id, alert)),
    )
}

pub fn alerts_js_array(assigned: &[Assigned<'_>]) -> String {
    format!("[{}]", assigned.iter().map(behaviour_entry_js).collect::<Vec<_>>().join(","))
}

pub fn all_ids(assigned: &[Assigned<'_>]) -> Vec<u64> {
    let mut ids = Vec::new();
    for a in assigned {
        ids.push(a.id);
        ids.extend(inop_ids(a.id, a.alert));
        ids.extend(info_ids(a.id, a.alert));
    }
    ids
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::deep::api::{line, var, EcamAlert, Level, Phase};
    use crate::deep::ecam::ids;

    fn sample() -> EcamAlert {
        EcamAlert::new("ENG_2_OIL_LO_PR", 79, "ENG 2 OIL LO PR", Level::Warning, var("ENGINE_OIL_PRESSURE_PSI:2").lt(25.0))
            .confirm(2.0)
            .inhibit(&[Phase::LiftOff, Phase::Above1500Ft])
            .step(line("THR LEVER 2", "IDLE").done(var("AUTOTHRUST_TLA:2").le(0.0)))
            .step(line("ENG 2 MASTER", "OFF").done(var("ENGINE_MASTER:2").off()).after(30.0))
            .status_line("ENG 2 OIL PRESSURE")
            .inop_sys("ENG 2 OIL SYSTEM")
    }

    fn sample_two() -> EcamAlert {
        EcamAlert::new("ENG_3_OIL_LO_PR", 79, "ENG 3 OIL LO PR", Level::Warning, var("ENGINE_OIL_PRESSURE_PSI:3").lt(25.0))
            .status_line("ENG 3 OIL PRESSURE")
            .inop_sys("ENG 3 OIL SYSTEM")
    }

    fn balanced(js: &str) -> bool {
        let (mut opens, mut closes) = (0i32, 0i32);
        for ch in js.chars() {
            match ch {
                '[' | '{' => opens += 1,
                ']' | '}' => closes += 1,
                _ => {}
            }
        }
        opens == closes
    }

    #[test]
    fn the_static_procedure_merge_carries_title_and_items_text_only() {
        let alerts = vec![sample()];
        let assigned = ids::assign(&alerts);
        let js = procedures_merge_js(&assigned);
        assert!(balanced(&js), "{js}");
        assert!(js.contains("title:'ENG 2 OIL LO PR'"));
        assert!(js.contains("name:'THR LEVER 2'"));
        assert!(js.contains("sensed:true"));
        assert!(js.contains("labelNotCompleted:'IDLE'"));
        assert!(js.contains("style:'Cyan'"));
        assert!(!js.contains("SimVar"));
        assert!(!js.contains("function"));
    }

    #[test]
    fn the_status_and_inop_merges_carry_the_right_text_at_derived_ids() {
        let alerts = vec![sample()];
        let assigned = ids::assign(&alerts);
        let id = assigned[0].id;
        assert_eq!(inop_merge_js(&assigned), format!("{{{}:'ENG 2 OIL SYSTEM'}}", id * 1000));
        assert_eq!(info_merge_js(&assigned), format!("{{{}:'ENG 2 OIL PRESSURE'}}", id * 1000 + 500));
    }

    #[test]
    fn the_behaviour_array_carries_trigger_and_line_logic_only() {
        let alerts = vec![sample()];
        let assigned = ids::assign(&alerts);
        let js = alerts_js_array(&assigned);
        assert!(balanced(&js), "{js}");
        assert!(js.contains("failure:3"), "Warning must map to failure 3");
        assert!(js.contains("confirmS:2"));
        assert!(js.contains("flightPhaseInhib:[6,8]"), "LiftOff=6, Above1500Ft=8, sorted");
        assert!(js.contains("afterS:30"));
        assert!(js.contains("'ENGINE_OIL_PRESSURE_PSI:2'") || js.contains("L:ENGINE_OIL_PRESSURE_PSI:2"));
        assert!(!js.contains("ENG 2 OIL LO PR"));
        assert!(!js.contains("THR LEVER"));
    }

    #[test]
    fn caution_and_advisory_map_to_the_right_failure_number() {
        let c = EcamAlert::new("C", 21, "C", Level::Caution, Cond::Always);
        let a = EcamAlert::new("A", 21, "A", Level::Advisory, Cond::Always);
        let m = EcamAlert::new("M", 21, "M", Level::Memo, Cond::Always);
        let alerts = [c, a, m];
        let assigned = ids::assign(&alerts);
        let js = alerts_js_array(&assigned);
        assert_eq!(js.matches("failure:1,").count(), 2, "Advisory and Memo: {js}");
        assert_eq!(js.matches("failure:2,").count(), 1, "Caution: {js}");
    }

    #[test]
    fn all_generated_ids_are_unique_and_outside_flybywires_space() {
        let alerts = vec![sample(), sample_two()];
        let assigned = ids::assign(&alerts);
        let mut ids_list = all_ids(&assigned);
        let before = ids_list.len();
        ids_list.sort_unstable();
        ids_list.dedup();
        assert_eq!(ids_list.len(), before, "no id collisions, including derived STATUS/INOP ids");
        assert!(ids_list.iter().all(|&i| i > 999_999_999));
    }
}
