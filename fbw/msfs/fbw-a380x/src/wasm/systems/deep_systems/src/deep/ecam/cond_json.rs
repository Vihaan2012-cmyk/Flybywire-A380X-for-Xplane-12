use crate::deep::api::{Cmp, Cond};

fn is_bool_cmp(cmp: Cmp, value: f64) -> bool {
    matches!(cmp, Cmp::Eq | Cmp::Ne) && value == 0.0
}

fn unit_for(cmp: Cmp, value: f64) -> &'static str {
    if is_bool_cmp(cmp, value) {
        "bool"
    } else {
        "number"
    }
}

const NAMESPACES: [&str; 7] = ["l:", "a:", "e:", "k:", "h:", "z:", "game:"];

pub fn js_var_name(name: &str) -> String {
    let lower = name.to_ascii_lowercase();
    if NAMESPACES.iter().any(|p| lower.starts_with(p)) || name.contains(' ') {
        name.to_string()
    } else {
        format!("L:{name}")
    }
}

fn cmp_tag(cmp: Cmp) -> &'static str {
    match cmp {
        Cmp::Lt => "lt",
        Cmp::Le => "le",
        Cmp::Gt => "gt",
        Cmp::Ge => "ge",
        Cmp::Eq => "eq",
        Cmp::Ne => "ne",
    }
}

pub fn js_string(s: &str) -> String {
    let mut out = String::with_capacity(s.len() + 2);
    out.push('\'');
    for c in s.chars() {
        match c {
            '\\' => out.push_str("\\\\"),
            '\'' => out.push_str("\\'"),
            '\n' => out.push_str("\\n"),
            _ => out.push(c),
        }
    }
    out.push('\'');
    out
}

fn num(v: f64) -> String {
    if v == v.trunc() && v.is_finite() && v.abs() < 1e15 {
        format!("{}", v as i64)
    } else {
        format!("{v}")
    }
}

pub fn cond_to_js(c: &Cond) -> String {
    match c {
        Cond::Always => "['always']".to_string(),
        Cond::Var { name, cmp, value } => {
            format!("['var',{},{},{},{}]", js_string(&js_var_name(name)), js_string(unit_for(*cmp, *value)), js_string(cmp_tag(*cmp)), num(*value))
        }
        Cond::VarVar { a, cmp, b } => {
            format!(
                "['varvar',{},{},{},{},{}]",
                js_string(&js_var_name(a)),
                js_string("number"),
                js_string(cmp_tag(*cmp)),
                js_string(&js_var_name(b)),
                js_string("number")
            )
        }
        Cond::And(v) => format!("['and',[{}]]", v.iter().map(cond_to_js).collect::<Vec<_>>().join(",")),
        Cond::Or(v) => format!("['or',[{}]]", v.iter().map(cond_to_js).collect::<Vec<_>>().join(",")),
        Cond::Not(c) => format!("['not',[{}]]", cond_to_js(c)),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::deep::api::{all, any, not, var};

    #[test]
    fn a_bare_custom_name_gets_the_l_prefix_a_spaced_default_simvar_does_not() {
        assert_eq!(js_var_name("A32NX_ENG_2_FMU_FAULT"), "L:A32NX_ENG_2_FMU_FAULT");
        assert_eq!(js_var_name("GENERAL ENG STARTER:2"), "GENERAL ENG STARTER:2");
        assert_eq!(js_var_name("L:A32NX_ALREADY_PREFIXED"), "L:A32NX_ALREADY_PREFIXED");
        assert_eq!(js_var_name("A:PLANE ALTITUDE"), "A:PLANE ALTITUDE");
    }

    #[test]
    fn on_and_off_read_as_bool_a_numeric_threshold_reads_as_number() {
        assert!(cond_to_js(&var("X").on()).contains("'bool'"));
        assert!(cond_to_js(&var("X").off()).contains("'bool'"));
        assert!(cond_to_js(&var("X").lt(25.0)).contains("'number'"));
        assert!(cond_to_js(&var("X").eq(3.0)).contains("'number'"));
    }

    #[test]
    fn compound_conditions_nest_correctly() {
        let c = all(vec![var("A").on(), any(vec![var("B").lt(1.0), not(var("C").off())])]);
        let js = cond_to_js(&c);
        assert!(js.starts_with("['and',["));
        assert!(js.contains("['or',["));
        assert!(js.contains("['not',["));
        let (mut opens, mut closes) = (0, 0);
        for ch in js.chars() {
            match ch {
                '[' => opens += 1,
                ']' => closes += 1,
                _ => {}
            }
        }
        assert_eq!(opens, closes);
    }

    #[test]
    fn always_encodes_with_no_operands() {
        assert_eq!(cond_to_js(&Cond::Always), "['always']");
    }

    #[test]
    fn strings_with_quotes_are_escaped() {
        let s = js_string("it's \\ a test");
        assert!(s.starts_with('\'') && s.ends_with('\''));
        assert!(s.contains("\\'"));
        assert!(s.contains("\\\\"));
    }
}
