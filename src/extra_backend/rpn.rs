//! MSFS's reverse Polish notation calculator code, as the aircraft presets'
//! procedure steps use it (`execute_calculator_code`, AircraftPresets.cpp:
//! 220-271).
//!
//! What the A380X's procedures use (config/a380x/a380-842/
//! aircraft_preset_procedures.xml): numbers; `(L:NAME)` and `(A:NAME:INDEX,
//! UNIT)` reads; `(>L:NAME)` writes; `(>K:EVENT)` and `(>K:2:EVENT)` key
//! events; `== != < > <= >= ! && || and or not`; `if{ ... }` and
//! `els{ ... }`. The rest of MSFS's arithmetic is here too, since it is
//! small. The top of the stack is an event's first argument, as the
//! converter's evaluator for the cockpit has it (msfs2xp-aircraft
//! behaviour/rpn.rs:293-302): `2 1 (>K:2:ELECTRICAL_BUS_TO_CIRCUIT_CONNECTION_TOGGLE)`
//! is bus 1, circuit 2.

/// Where the code's variables and events go.
pub trait RpnHost {
    /// `kind` is `L` or `A`; `name` is as written, with any `:index`.
    fn get(&mut self, kind: &str, name: &str, unit: &str) -> f64;
    fn set(&mut self, kind: &str, name: &str, unit: &str, value: f64);
    /// A key event with its arguments, first argument first.
    fn key_event(&mut self, name: &str, args: &[f64]);
}

#[derive(Clone, Debug, PartialEq)]
enum Tok {
    Num(f64),
    Var { write: bool, kind: String, name: String, unit: String },
    If,
    Els,
    End,
    Word(String),
}

fn tokenize(code: &str) -> Vec<Tok> {
    let mut out = Vec::new();
    let chars: Vec<char> = code.chars().collect();
    let mut i = 0;
    while i < chars.len() {
        let c = chars[i];
        if c.is_whitespace() {
            i += 1;
        } else if c == '(' {
            let end = chars[i..].iter().position(|&c| c == ')').map_or(chars.len(), |p| i + p);
            let inner: String = chars[i + 1..end].iter().collect();
            i = end + 1;
            let inner = inner.trim();
            let (write, inner) = match inner.strip_prefix('>') {
                Some(rest) => (true, rest.trim()),
                None => (false, inner),
            };
            let (reference, unit) = match inner.split_once(',') {
                Some((r, u)) => (r.trim(), u.trim()),
                None => (inner, ""),
            };
            let (kind, name) = reference.split_once(':').unwrap_or(("A", reference));
            out.push(Tok::Var { write, kind: kind.trim().to_ascii_uppercase(), name: name.trim().to_string(), unit: unit.to_string() });
        } else {
            let start = i;
            while i < chars.len() && !chars[i].is_whitespace() && chars[i] != '(' {
                i += 1;
            }
            let word: String = chars[start..i].iter().collect();
            out.push(match word.as_str() {
                "if{" => Tok::If,
                "els{" => Tok::Els,
                "}" => Tok::End,
                w => w.parse::<f64>().map_or_else(|_| Tok::Word(w.to_string()), Tok::Num),
            });
        }
    }
    out
}

/// Index just past the block whose opening token is before `i`.
fn skip_block(toks: &[Tok], mut i: usize) -> usize {
    let mut depth = 1;
    while i < toks.len() {
        match toks[i] {
            Tok::If | Tok::Els => depth += 1,
            Tok::End => {
                depth -= 1;
                if depth == 0 {
                    return i + 1;
                }
            }
            _ => {}
        }
        i += 1;
    }
    i
}

fn is_bool_unit(unit: &str) -> bool {
    matches!(unit.to_ascii_lowercase().as_str(), "bool" | "boolean")
}

/// Run `code`; the value left on top of the stack, 0 if none (the `fvalue`
/// `execute_calculator_code` hands back).
pub fn execute(code: &str, host: &mut dyn RpnHost) -> f64 {
    let toks = tokenize(code);
    let mut stack = Vec::new();
    let mut i = 0;
    exec(&toks, &mut i, &mut stack, host);
    stack.last().copied().unwrap_or(0.)
}

/// Runs to the end of the code or of the current block.
fn exec(toks: &[Tok], i: &mut usize, st: &mut Vec<f64>, host: &mut dyn RpnHost) {
    let pop = |st: &mut Vec<f64>| st.pop().unwrap_or(0.);
    let b = |x: bool| if x { 1. } else { 0. };
    while *i < toks.len() {
        let t = toks[*i].clone();
        *i += 1;
        match t {
            Tok::Num(n) => st.push(n),
            Tok::End => return,
            // Reached after a taken if-block: its else-block is skipped.
            Tok::Els => *i = skip_block(toks, *i),
            Tok::If => {
                if pop(st) != 0. {
                    exec(toks, i, st, host);
                    if toks.get(*i) == Some(&Tok::Els) {
                        *i = skip_block(toks, *i + 1);
                    }
                } else {
                    *i = skip_block(toks, *i);
                    if toks.get(*i) == Some(&Tok::Els) {
                        *i += 1;
                        exec(toks, i, st, host);
                    }
                }
            }
            Tok::Var { write: true, kind, name, unit } if kind == "K" => {
                let (count, event) = match name.split_once(':') {
                    Some((n, e)) if n.trim().parse::<usize>().is_ok() => (n.trim().parse().unwrap_or(1), e.trim().to_string()),
                    _ => (1, name),
                };
                let _ = unit;
                let args: Vec<f64> = (0..count).filter_map(|_| st.pop()).collect();
                host.key_event(&event, &args);
            }
            Tok::Var { write: true, kind, name, unit } => {
                let mut v = pop(st);
                if is_bool_unit(&unit) {
                    v = b(v != 0.);
                }
                host.set(&kind, &name, &unit, v);
            }
            Tok::Var { write: false, kind, name, unit } => {
                let mut v = host.get(&kind, &name, &unit);
                if is_bool_unit(&unit) {
                    v = b(v != 0.);
                }
                st.push(v);
            }
            Tok::Word(w) => {
                let unary = |st: &mut Vec<f64>, f: fn(f64) -> f64| {
                    let x = pop(st);
                    st.push(f(x));
                };
                let binary = |st: &mut Vec<f64>, f: &dyn Fn(f64, f64) -> f64| {
                    let (y, x) = (pop(st), pop(st));
                    st.push(f(x, y));
                };
                match w.to_ascii_lowercase().as_str() {
                    "+" => binary(st, &|x, y| x + y),
                    "-" => binary(st, &|x, y| x - y),
                    "*" => binary(st, &|x, y| x * y),
                    "/" => binary(st, &|x, y| if y != 0. { x / y } else { 0. }),
                    "%" => binary(st, &|x, y| if y != 0. { x % y } else { 0. }),
                    "min" => binary(st, &|x, y| x.min(y)),
                    "max" => binary(st, &|x, y| x.max(y)),
                    "==" | "eq" => binary(st, &|x, y| b(x == y)),
                    "!=" | "ne" => binary(st, &|x, y| b(x != y)),
                    "<" | "lt" => binary(st, &|x, y| b(x < y)),
                    ">" | "gt" => binary(st, &|x, y| b(x > y)),
                    "<=" | "le" => binary(st, &|x, y| b(x <= y)),
                    ">=" | "ge" => binary(st, &|x, y| b(x >= y)),
                    "&&" | "and" => binary(st, &|x, y| b(x != 0. && y != 0.)),
                    "||" | "or" => binary(st, &|x, y| b(x != 0. || y != 0.)),
                    "!" | "not" => unary(st, |x| if x == 0. { 1. } else { 0. }),
                    "neg" => unary(st, |x| -x),
                    "abs" => unary(st, f64::abs),
                    "flr" => unary(st, f64::floor),
                    "ceil" => unary(st, f64::ceil),
                    "near" | "rnd" => unary(st, f64::round),
                    "d" => {
                        let x = pop(st);
                        st.push(x);
                        st.push(x);
                    }
                    "p" => {
                        pop(st);
                    }
                    "r" => {
                        let (y, x) = (pop(st), pop(st));
                        st.push(y);
                        st.push(x);
                    }
                    "quit" => {
                        *i = toks.len();
                        return;
                    }
                    // Nothing in the A380X's procedures uses anything else;
                    // an unknown word leaves the stack alone.
                    _ => {}
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::HashMap;

    #[derive(Default)]
    struct Host {
        vars: HashMap<String, f64>,
        events: Vec<(String, Vec<f64>)>,
    }

    impl RpnHost for Host {
        fn get(&mut self, kind: &str, name: &str, _: &str) -> f64 {
            self.vars.get(&format!("{kind}:{name}")).copied().unwrap_or(0.)
        }
        fn set(&mut self, kind: &str, name: &str, _: &str, value: f64) {
            self.vars.insert(format!("{kind}:{name}"), value);
        }
        fn key_event(&mut self, name: &str, args: &[f64]) {
            self.events.push((name.to_string(), args.to_vec()));
        }
    }

    #[test]
    fn procedure_conditions_evaluate_as_msfs() {
        let mut h = Host::default();
        h.vars.insert("L:A32NX_ENGINE_STATE:1".into(), 1.);
        h.vars.insert("L:A32NX_ENGINE_STATE:2".into(), 1.);
        // aircraft_preset_procedures.xml:553-559, two engines of four running.
        let code = "(L:A32NX_ENGINE_STATE:1) 1 == (L:A32NX_ENGINE_STATE:2) 1 == (L:A32NX_ENGINE_STATE:3) 1 == (L:A32NX_ENGINE_STATE:4) 1 == 1 && && &&";
        assert_eq!(execute(code, &mut h), 0.);
        h.vars.insert("L:A32NX_ENGINE_STATE:3".into(), 1.);
        h.vars.insert("L:A32NX_ENGINE_STATE:4".into(), 1.);
        assert_eq!(execute(code, &mut h), 1.);
        // A bool unit reads 0 or 1; `!` negates.
        h.vars.insert("L:A32NX_OVHD_ELEC_BAT_1_PB_IS_AUTO".into(), 5.);
        assert_eq!(execute("(L:A32NX_OVHD_ELEC_BAT_1_PB_IS_AUTO, BOOL) !", &mut h), 0.);
        // `1 == &&` with one operand is false (xml:643): the missing operand is 0.
        h.vars.insert("A:LIGHT TAXI:3".into(), 1.);
        assert_eq!(execute("(A:LIGHT TAXI:3, Number) 1 == &&", &mut h), 0.);
    }

    #[test]
    fn actions_write_variables_and_send_events_with_their_arguments() {
        let mut h = Host::default();
        execute("0 (>L:XMLVAR_SWITCH_OVHD_INTLT_SEATBELT_Position) (A:CABIN SEATBELTS ALERT SWITCH:1, BOOL) ! if{ 1 (>K:CABIN_SEATBELTS_ALERT_SWITCH_TOGGLE) }", &mut h);
        assert_eq!(h.vars["L:XMLVAR_SWITCH_OVHD_INTLT_SEATBELT_Position"], 0.);
        assert_eq!(h.events, vec![("CABIN_SEATBELTS_ALERT_SWITCH_TOGGLE".to_string(), vec![1.])]);
        h.events.clear();
        execute("2 1 (>K:2:ELECTRICAL_BUS_TO_CIRCUIT_CONNECTION_TOGGLE)", &mut h);
        assert_eq!(h.events, vec![("ELECTRICAL_BUS_TO_CIRCUIT_CONNECTION_TOGGLE".to_string(), vec![1., 2.])]);
        h.events.clear();
        // One value for a two-argument event.
        execute("1 (>K:2:LOGO_LIGHTS_SET) 1 (>K:2:NAV_LIGHTS_SET)", &mut h);
        assert_eq!(h.events.len(), 2);
        assert_eq!(h.events[1], ("NAV_LIGHTS_SET".to_string(), vec![1.]));
    }

    #[test]
    fn else_blocks_run_only_when_the_condition_is_false() {
        let mut h = Host::default();
        execute("1 if{ 5 (>L:A) } els{ 6 (>L:A) } 0 if{ 7 (>L:B) } els{ 8 (>L:B) }", &mut h);
        assert_eq!(h.vars["L:A"], 5.);
        assert_eq!(h.vars["L:B"], 8.);
    }
}
