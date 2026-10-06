use crate::deep::api::Level;
use crate::deep::ecam::cond_json::cond_to_js;
use crate::deep::ecam::fbw::{FbwItem, FbwProc};

fn failure_number(level: Level) -> u8 {
    match level {
        Level::Warning => 3,
        Level::Caution => 2,
        Level::Advisory | Level::Memo => 1,
    }
}

fn num(v: f64) -> String {
    if v == v.trunc() && v.is_finite() && v.abs() < 1e15 {
        format!("{}", v as i64)
    } else {
        format!("{v}")
    }
}

fn item_js(it: &FbwItem) -> String {
    let show = it.show.as_ref().map_or_else(|| "null".to_string(), cond_to_js);
    let checked = it.checked.as_ref().map_or_else(|| "null".to_string(), cond_to_js);
    format!("{{index:{},show:{},checked:{}}}", it.index, show, checked)
}

fn entry_js(p: &FbwProc) -> String {
    let phases = p.inhibit.iter().map(u32::to_string).collect::<Vec<_>>().join(",");
    let suppressed = p.suppressed_by.iter().map(|id| format!("'{id}'")).collect::<Vec<_>>().join(",");
    let items = p.items.iter().map(item_js).collect::<Vec<_>>().join(",");
    format!(
        "{{id:{},failure:{},sysPage:{},flightPhaseInhib:[{}],confirmS:{},notActiveWhenItemActive:[{}],trigger:{},items:[{}]}}",
        p.id,
        failure_number(p.level),
        p.sys_page,
        phases,
        num(p.confirm_s),
        suppressed,
        cond_to_js(&p.trigger),
        items,
    )
}

pub fn fbw_alerts_js_array(procs: &[FbwProc]) -> String {
    format!("[{}]", procs.iter().map(entry_js).collect::<Vec<_>>().join(","))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::deep::api::var;
    use crate::deep::ecam::fbw;

    #[test]
    fn an_entry_carries_behaviour_and_never_the_procedure_text() {
        let js = fbw_alerts_js_array(&fbw::wirings());
        assert!(!js.is_empty());
        for title in ["ELEC AC BUS 2 FAULT", "OIL PRESS LO", "LAVATORY", "TIRE PRESS"] {
            assert!(!js.contains(title), "{title} leaked into the behaviour array: FlyByWire's own table owns it");
        }
        assert!(js.contains("trigger:"));
        assert!(js.contains("flightPhaseInhib:"));
    }

    #[test]
    fn brackets_balance_and_bare_names_are_l_prefixed() {
        let js = fbw_alerts_js_array(&fbw::wirings());
        let (mut open, mut close) = (0i32, 0i32);
        for c in js.chars() {
            match c {
                '[' | '{' => open += 1,
                ']' | '}' => close += 1,
                _ => {}
            }
        }
        assert_eq!(open, close, "unbalanced JS literal");
        assert!(js.contains("'L:ELEC_AC_2_BUS_IS_POWERED'"));
        assert!(!js.contains("'ELEC_AC_2_BUS_IS_POWERED'"), "a bare plugin variable must reach SimVar as an L: var");
    }

    #[test]
    fn an_item_with_no_condition_encodes_as_null_not_as_a_tick() {
        let one = vec![fbw::item(3)];
        assert_eq!(item_js(&one[0]), "{index:3,show:null,checked:null}");
        let sensed = fbw::item(0).checked(var("X").on());
        assert!(item_js(&sensed).contains("checked:['var','L:X','bool','ne',0]"));
    }

    #[test]
    fn levels_map_to_flybywires_own_failure_numbers() {
        assert_eq!(failure_number(Level::Warning), 3);
        assert_eq!(failure_number(Level::Caution), 2);
        assert_eq!(failure_number(Level::Advisory), 1);
        assert_eq!(failure_number(Level::Memo), 1);
    }
}
