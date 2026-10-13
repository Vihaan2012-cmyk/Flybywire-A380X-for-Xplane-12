//! MSFS reverse Polish notation: enough of it to run a control's click code
//! on a given state and see what it writes.
//!
//! Variables are keyed `L:NAME`, `A:NAME:INDEX`, `O:NAME` and so on, without
//! units. `K:` events go to a handler that turns them into variable writes.

use std::collections::{BTreeSet, HashMap};

#[derive(Clone, Debug, PartialEq)]
pub enum Val {
    Num(f64),
    Str(String),
}

impl Val {
    fn num(&self) -> f64 {
        match self {
            Val::Num(n) => *n,
            Val::Str(s) => s.trim().parse().unwrap_or(0.0),
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub enum Tok {
    Num(f64),
    Str(String),
    /// A variable or event reference: `write`, type letter(s), name, unit.
    Var { write: bool, kind: String, name: String, unit: String },
    If,
    Els,
    End,
    Word(String),
}

pub fn tokenize(code: &str) -> Vec<Tok> {
    let b: Vec<char> = code.chars().collect();
    let mut i = 0;
    let mut out = Vec::new();
    while i < b.len() {
        let c = b[i];
        if c.is_whitespace() {
            i += 1;
            continue;
        }
        if c == '(' {
            let start = i + 1;
            let mut j = start;
            let mut depth = 1;
            while j < b.len() {
                match b[j] {
                    '(' => depth += 1,
                    ')' => {
                        depth -= 1;
                        if depth == 0 {
                            break;
                        }
                    }
                    _ => {}
                }
                j += 1;
            }
            let inner: String = b[start..j.min(b.len())].iter().collect();
            i = j + 1;
            let inner = inner.trim();
            if inner.starts_with('*') || inner.is_empty() {
                // "(* comment *)"
                continue;
            }
            let (write, inner) = match inner.strip_prefix('>') {
                Some(r) => (true, r.trim()),
                None => (false, inner),
            };
            let Some((kind, rest)) = inner.split_once(':') else {
                out.push(Tok::Word(inner.to_string()));
                continue;
            };
            let (name, unit) = match rest.split_once(',') {
                Some((n, u)) => (n.trim(), u.trim()),
                None => (rest.trim(), ""),
            };
            out.push(Tok::Var {
                write,
                kind: kind.trim().to_ascii_uppercase(),
                name: name.to_string(),
                unit: unit.to_string(),
            });
            continue;
        }
        if c == '\'' {
            let mut j = i + 1;
            while j < b.len() && b[j] != '\'' {
                j += 1;
            }
            out.push(Tok::Str(b[i + 1..j.min(b.len())].iter().collect()));
            i = j + 1;
            continue;
        }
        let mut j = i;
        while j < b.len() && !b[j].is_whitespace() && b[j] != '(' && b[j] != '\'' {
            j += 1;
            // "if{" and "els{" end at their brace; "}" stands alone.
            if b[j - 1] == '{' || b[j - 1] == '}' {
                break;
            }
        }
        let w: String = b[i..j].iter().collect();
        i = j;
        out.push(match w.as_str() {
            "if{" => Tok::If,
            "els{" => Tok::Els,
            "}" => Tok::End,
            _ => match w.parse::<f64>() {
                Ok(n) => Tok::Num(n),
                Err(_) => Tok::Word(w),
            },
        });
    }
    out
}

/// A variable's key: `A:` names upper-cased with their index, others as written.
pub fn key(kind: &str, name: &str) -> String {
    let name = name.trim();
    if kind == "A" {
        format!("A:{}", name.to_ascii_uppercase())
    } else {
        format!("{kind}:{name}")
    }
}

/// The factor between the unit code reads a variable in and the unit its
/// dataref holds. The systems plugin holds `LIGHT POTENTIOMETER` as a ratio,
/// 0 to 1, as FlyByWire's instruments read it ("percent over 100",
/// CdsDisplayUnit.tsx) and as its SET events store it (percent / 100,
/// key_events.rs); code reading it in plain "percent" sees 0 to 100.
pub fn unit_scale(kind: &str, name: &str, unit: &str) -> f64 {
    let u = unit.trim().to_ascii_lowercase();
    let n = name.trim().to_ascii_uppercase();
    if kind == "A" && n.starts_with("LIGHT POTENTIOMETER") && u == "percent" {
        100.0
    } else {
        1.0
    }
}

/// What running code did.
#[derive(Debug, Default, Clone)]
pub struct Run {
    /// Variable writes in order (keys as [`key`]).
    pub writes: Vec<(String, f64)>,
    /// Events fired that the handler did not turn into writes (`K:X`, `H:X`, `B:X`).
    pub events: Vec<String>,
    /// Variables read.
    pub reads: BTreeSet<String>,
    /// Words this interpreter does not know.
    pub unknown: Vec<String>,
    pub stack: Vec<Val>,
    /// Register 0 when the code ended (`GET_STATE_EXTERNAL` leaves its value there).
    pub reg0: Option<f64>,
    /// X-Plane commands the events became, in order.
    pub commands: Vec<String>,
}

/// Turns `K:` events into variable writes; returns false for events it does
/// not know.
pub trait Events {
    fn fire(&self, event: &str, args: &[f64], env: &mut Env) -> bool;
    /// An `H:` event (FlyByWire's instruments); false when it is not known.
    fn fire_h(&self, _event: &str, _env: &mut Env) -> bool {
        false
    }
    /// A `B:` input event, with its argument when the stack had one.
    fn fire_b(&self, _event: &str, _arg: Option<f64>, _env: &mut Env) -> bool {
        false
    }
}

/// The variables code runs against, with a log of what it wrote.
pub struct Env<'a> {
    pub vars: HashMap<String, f64>,
    base: &'a HashMap<String, f64>,
    pub run: Run,
}

impl Env<'_> {
    pub fn get(&mut self, k: &str) -> f64 {
        self.run.reads.insert(k.to_string());
        self.vars.get(k).or_else(|| self.base.get(k)).copied().unwrap_or(0.0)
    }
    pub fn set(&mut self, k: &str, v: f64) {
        self.vars.insert(k.to_string(), v);
        self.run.writes.push((k.to_string(), v));
    }
}

struct NoEvents;
impl Events for NoEvents {
    fn fire(&self, _: &str, _: &[f64], _: &mut Env) -> bool {
        false
    }
}

/// Evaluate a plain arithmetic expression (template `Process="Int"`).
pub fn eval_number(code: &str) -> Option<f64> {
    let base = HashMap::new();
    let r = run(code, &base, &NoEvents);
    match r.stack.last() {
        Some(v) if r.unknown.is_empty() => Some(v.num()),
        _ => None,
    }
}

/// Run code on `state` (unset variables read 0).
pub fn run(code: &str, state: &HashMap<String, f64>, events: &dyn Events) -> Run {
    run_with(code, state, events, Vec::new())
}

/// Run code with values already on the stack (bottom first).
pub fn run_with(code: &str, state: &HashMap<String, f64>, events: &dyn Events, initial: Vec<f64>) -> Run {
    let toks = tokenize(code);
    let mut env = Env { vars: HashMap::new(), base: state, run: Run::default() };
    let mut stack: Vec<Val> = initial.into_iter().map(Val::Num).collect();
    let mut reg: HashMap<String, Val> = HashMap::new();
    exec(&toks, &mut 0, &mut stack, &mut reg, &mut env, events, 0);
    env.run.stack = stack;
    env.run.reg0 = reg.get("0").map(|v| v.num());
    env.run
}

/// Index just past the block starting at `i` (after an `if{` or `els{`).
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

/// Returns false when the code quits.
fn exec(
    toks: &[Tok],
    i: &mut usize,
    st: &mut Vec<Val>,
    reg: &mut HashMap<String, Val>,
    env: &mut Env,
    events: &dyn Events,
    depth: usize,
) -> bool {
    let pop = |st: &mut Vec<Val>| st.pop().unwrap_or(Val::Num(0.0));
    let popn = |st: &mut Vec<Val>| st.pop().map_or(0.0, |v| v.num());
    let b = |x: bool| Val::Num(if x { 1.0 } else { 0.0 });
    while *i < toks.len() {
        let t = toks[*i].clone();
        *i += 1;
        match t {
            Tok::Num(n) => st.push(Val::Num(n)),
            Tok::Str(s) => st.push(Val::Str(s)),
            Tok::End => return true,
            Tok::Els => {
                // Reached after a taken if-block's end: handled there.
                *i = skip_block(toks, *i);
            }
            Tok::If => {
                let c = popn(st) != 0.0;
                if c {
                    if !exec(toks, i, st, reg, env, events, depth + 1) {
                        return false;
                    }
                    if matches!(toks.get(*i), Some(Tok::Els)) {
                        *i = skip_block(toks, *i + 1);
                    }
                } else {
                    *i = skip_block(toks, *i);
                    if matches!(toks.get(*i), Some(Tok::Els)) {
                        *i += 1;
                        if !exec(toks, i, st, reg, env, events, depth + 1) {
                            return false;
                        }
                    }
                }
            }
            Tok::Var { write, kind, name, unit } => {
                // MSFS converts simulation variables to a Bool unit but hands
                // an L: variable back as it holds it: FlyByWire's apron step
                // `(L:A32NX_START_STATE, Bool) 2 ==` reads 2, not 1.
                let bool_unit = kind != "L" && (unit.eq_ignore_ascii_case("bool") || unit.eq_ignore_ascii_case("boolean"));
                match (write, kind.as_str()) {
                    (true, "K") => {
                        // K:2:EVENT takes two arguments, K:EVENT one (if given).
                        let (n, ev) = match name.split_once(':') {
                            Some((c, e)) if c.trim().parse::<usize>().is_ok() => (c.trim().parse::<usize>().unwrap_or(1), e.trim().to_string()),
                            _ => (1, name.trim().to_string()),
                        };
                        // The top of the stack is the event's first argument.
                        let args: Vec<f64> = (0..n).filter_map(|_| st.pop().map(|v| v.num())).collect();
                        if !events.fire(&ev, &args, env) {
                            env.run.events.push(format!("K:{ev}"));
                        }
                    }
                    (true, "F") if name.trim().eq_ignore_ascii_case("KeyEvent") => {
                        // 'EVENT' (>F:KeyEvent): the key event named on the stack.
                        match pop(st) {
                            Val::Str(ev) => {
                                if !events.fire(ev.trim(), &[], env) {
                                    env.run.events.push(format!("K:{}", ev.trim()));
                                }
                            }
                            Val::Num(_) => env.run.unknown.push("F:KeyEvent without an event name".into()),
                        }
                    }
                    (true, "H") => {
                        if !events.fire_h(name.trim(), env) {
                            env.run.events.push(format!("H:{}", name.trim()));
                        }
                    }
                    (true, "B") => {
                        let arg = st.pop().map(|v| v.num());
                        if !events.fire_b(name.trim(), arg, env) {
                            env.run.events.push(format!("B:{}", name.trim()));
                        }
                    }
                    (true, "E" | "P") => {
                        env.run.events.push(format!("{kind}:{}", name.trim()));
                    }
                    (true, _) => {
                        let mut v = popn(st);
                        if bool_unit {
                            v = if v != 0.0 { 1.0 } else { 0.0 };
                        }
                        v /= unit_scale(&kind, &name, &unit);
                        env.set(&key(&kind, &name), v);
                    }
                    (false, "M" | "R" | "S") => st.push(Val::Str(String::new())),
                    (false, _) => {
                        let mut v = env.get(&key(&kind, &name)) * unit_scale(&kind, &name, &unit);
                        if bool_unit {
                            v = if v != 0.0 { 1.0 } else { 0.0 };
                        }
                        st.push(Val::Num(v));
                    }
                }
            }
            Tok::Word(w) => {
                let lw = w.to_ascii_lowercase();
                match lw.as_str() {
                    "+" => {
                        let (y, x) = (popn(st), popn(st));
                        st.push(Val::Num(x + y));
                    }
                    "-" => {
                        let (y, x) = (popn(st), popn(st));
                        st.push(Val::Num(x - y));
                    }
                    "*" => {
                        let (y, x) = (popn(st), popn(st));
                        st.push(Val::Num(x * y));
                    }
                    "/" => {
                        let (y, x) = (popn(st), popn(st));
                        st.push(Val::Num(if y != 0.0 { x / y } else { 0.0 }));
                    }
                    "%" => {
                        let (y, x) = (popn(st), popn(st));
                        st.push(Val::Num(if y != 0.0 { x % y } else { 0.0 }));
                    }
                    "++" | "--" => {
                        let x = popn(st);
                        st.push(Val::Num(if lw == "++" { x + 1.0 } else { x - 1.0 }));
                    }
                    "neg" => {
                        let x = popn(st);
                        st.push(Val::Num(-x));
                    }
                    "abs" => {
                        let x = popn(st);
                        st.push(Val::Num(x.abs()));
                    }
                    "flr" => {
                        let x = popn(st);
                        st.push(Val::Num(x.floor()));
                    }
                    "ceil" => {
                        let x = popn(st);
                        st.push(Val::Num(x.ceil()));
                    }
                    "near" | "rnd" => {
                        let x = popn(st);
                        st.push(Val::Num(x.round()));
                    }
                    "int" => {
                        let x = popn(st);
                        st.push(Val::Num(x.trunc()));
                    }
                    "!" | "not" => {
                        let x = popn(st);
                        st.push(b(x == 0.0));
                    }
                    "and" | "&&" => {
                        let (y, x) = (popn(st), popn(st));
                        st.push(b(x != 0.0 && y != 0.0));
                    }
                    "or" | "||" => {
                        let (y, x) = (popn(st), popn(st));
                        st.push(b(x != 0.0 || y != 0.0));
                    }
                    "&" => {
                        let (y, x) = (popn(st), popn(st));
                        st.push(Val::Num(((x as i64) & (y as i64)) as f64));
                    }
                    "|" => {
                        let (y, x) = (popn(st), popn(st));
                        st.push(Val::Num(((x as i64) | (y as i64)) as f64));
                    }
                    "^" => {
                        let (y, x) = (popn(st), popn(st));
                        st.push(Val::Num(((x as i64) ^ (y as i64)) as f64));
                    }
                    "~" => {
                        let x = popn(st);
                        st.push(Val::Num(!(x as i64) as f64));
                    }
                    "==" | "eq" | "!=" | "ne" | "<" | "lt" | ">" | "gt" | "<=" | "le" | ">=" | "ge" => {
                        let (y, x) = (popn(st), popn(st));
                        st.push(b(match lw.as_str() {
                            "==" | "eq" => x == y,
                            "!=" | "ne" => x != y,
                            "<" | "lt" => x < y,
                            ">" | "gt" => x > y,
                            "<=" | "le" => x <= y,
                            _ => x >= y,
                        }));
                    }
                    "min" | "max" => {
                        let (y, x) = (popn(st), popn(st));
                        st.push(Val::Num(if lw == "min" { x.min(y) } else { x.max(y) }));
                    }
                    "rng" => {
                        // lo hi x rng: whether lo <= x <= hi.
                        let (x, hi, lo) = (popn(st), popn(st), popn(st));
                        st.push(b(lo <= x && x <= hi));
                    }
                    "?" => {
                        // a b c ?: a if c else b.
                        let (c, y, x) = (popn(st), pop(st), pop(st));
                        st.push(if c != 0.0 { x } else { y });
                    }
                    "d" => {
                        let x = st.last().cloned().unwrap_or(Val::Num(0.0));
                        st.push(x);
                    }
                    "r" => {
                        let (y, x) = (pop(st), pop(st));
                        st.push(y);
                        st.push(x);
                    }
                    "p" => {
                        st.pop();
                    }
                    "c" => st.clear(),
                    "quit" => return false,
                    "scmp" | "scmi" => {
                        let (y, x) = (pop(st), pop(st));
                        let (mut xs, mut ys) = match (x, y) {
                            (Val::Str(a), Val::Str(c)) => (a, c),
                            (a, c) => (format!("{a:?}"), format!("{c:?}")),
                        };
                        if lw == "scmi" {
                            xs = xs.to_ascii_lowercase();
                            ys = ys.to_ascii_lowercase();
                        }
                        st.push(Val::Num(match xs.cmp(&ys) {
                            std::cmp::Ordering::Less => -1.0,
                            std::cmp::Ordering::Equal => 0.0,
                            std::cmp::Ordering::Greater => 1.0,
                        }));
                    }
                    "pi" => st.push(Val::Num(std::f64::consts::PI)),
                    "sqr" => {
                        let x = popn(st);
                        st.push(Val::Num(x * x));
                    }
                    "sqrt" => {
                        let x = popn(st);
                        st.push(Val::Num(x.max(0.0).sqrt()));
                    }
                    "pow" => {
                        let (y, x) = (popn(st), popn(st));
                        st.push(Val::Num(x.powf(y)));
                    }
                    "true" => st.push(Val::Num(1.0)),
                    "false" => st.push(Val::Num(0.0)),
                    _ => {
                        if let Some(n) = lw.strip_prefix("sp").and_then(|n| n.parse::<u32>().ok()) {
                            let v = pop(st);
                            reg.insert(n.to_string(), v);
                        } else if let Some(n) = lw.strip_prefix('s').and_then(|n| n.parse::<u32>().ok()) {
                            let v = st.last().cloned().unwrap_or(Val::Num(0.0));
                            reg.insert(n.to_string(), v);
                        } else if let Some(n) = lw.strip_prefix('l').and_then(|n| n.parse::<u32>().ok()) {
                            st.push(reg.get(&n.to_string()).cloned().unwrap_or(Val::Num(0.0)));
                        } else {
                            env.run.unknown.push(w.clone());
                        }
                    }
                }
            }
        }
        if depth > 64 {
            return false;
        }
    }
    true
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn an_l_variable_read_as_bool_keeps_its_value_and_a_sim_variable_does_not() {
        // FlyByWire's apron step: `(L:A32NX_START_STATE, Bool) 2 ==`.
        let st = HashMap::from([("L:A32NX_START_STATE".to_string(), 2.0), ("A:X".to_string(), 2.0)]);
        let r = run("(L:A32NX_START_STATE, Bool) 2 == if{ 1 (>L:OFF) }", &st, &NoEvents);
        assert_eq!(r.writes, vec![("L:OFF".into(), 1.0)]);
        let r = run("(A:X, Bool) (>L:Y)", &st, &NoEvents);
        assert_eq!(r.writes, vec![("L:Y".into(), 1.0)]);
    }

    #[test]
    fn arithmetic_and_branches() {
        assert_eq!(eval_number("4 1 -"), Some(3.0));
        assert_eq!(eval_number("3 1 - 100 *"), Some(200.0));
        let st = HashMap::from([("L:X".to_string(), 1.0)]);
        let r = run("(L:X, Bool) if{ 0 (>L:X) } els{ 1 (>L:X) } 5 (>L:Y)", &st, &NoEvents);
        assert_eq!(r.writes, vec![("L:X".into(), 0.0), ("L:Y".into(), 5.0)]);
        let r = run("(L:X, Bool) ! if{ 0 (>L:X) } els{ 7 (>L:X) } 5 (>L:Y)", &st, &NoEvents);
        assert_eq!(r.writes, vec![("L:X".into(), 7.0), ("L:Y".into(), 5.0)]);
    }

    #[test]
    fn registers_and_events() {
        let st = HashMap::new();
        let r = run("2 sp0 l0 l0 + (>L:Z) (>H:A380X_BTN) 1 (>K:TOGGLE_X)", &st, &NoEvents);
        assert_eq!(r.writes, vec![("L:Z".into(), 4.0)]);
        assert_eq!(r.events, vec!["H:A380X_BTN".to_string(), "K:TOGGLE_X".to_string()]);
        let toks = tokenize("(A:GENERAL ENG STARTER:2, Bool) (>K:2:LIGHT_POTENTIOMETER_SET)");
        assert_eq!(toks[0], Tok::Var { write: false, kind: "A".into(), name: "GENERAL ENG STARTER:2".into(), unit: "Bool".into() });
    }
}
