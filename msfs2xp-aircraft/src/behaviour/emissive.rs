//! Cockpit lights: each node's emissive code (how brightly its legend,
//! backlight or annunciator glows) and visibility code, as MSFS's own
//! templates write them (`<Material><EmissiveFactor><Parameter><Code>` and
//! `<Visibility><Parameter><Code>`), turned into what X-Plane can draw:
//!
//! - a code that reads one systems variable linearly (`(L:X, Bool)`,
//!   `(L:X, Bool) !`, a potentiometer in percent over 100) drives the
//!   node's `ATTR_light_level` (or `ANIM_hide`) on that variable's dataref;
//! - any other code runs in SASL every frame, translated from the RPN, into
//!   a helper dataref per node that `ATTR_light_level` or `ANIM_hide` reads;
//! - a code with no variables is a constant: always lit, or never.

use std::collections::{BTreeMap, BTreeSet, HashMap};

use super::bind::{self, to_lua_with};
use super::expand::{LightCode, LightKind};
use super::rpn;

/// How a node's light or visibility is driven.
#[derive(Clone, Debug, PartialEq)]
pub enum Drive {
    /// Always this value (0: never lit or never shown).
    Const(f64),
    /// Brightness `(dref - v1) / (v2 - v1)`; shown while `dref` is not `v1`.
    Direct { dref: String, v1: f64, v2: f64 },
    /// The code runs in SASL (`lua[index]`, a function body returning the
    /// value) into a helper dataref.
    Lua(usize),
}

#[derive(Clone, Debug)]
pub struct Light {
    pub node: String,
    pub template: String,
    pub kind: LightKind,
    pub code: String,
    pub drive: Result<Drive, String>,
    /// Systems datarefs the code reads.
    pub reads: BTreeSet<String>,
}

#[derive(Clone, Debug, Default)]
pub struct Lights {
    pub lights: Vec<Light>,
    /// Distinct Lua function bodies, shared by nodes with the same code.
    pub lua: Vec<String>,
    /// Nodes given more than one code of one kind (the last one is kept, as
    /// MSFS's components run in order and the last sets the material).
    pub overridden: usize,
}

/// Run code with one variable at `x` (and the constants), returning its value.
fn value_at(code: &str, key: Option<&str>, x: f64, consts: &HashMap<String, f64>) -> Option<f64> {
    let mut st = consts.clone();
    if let Some(k) = key {
        st.insert(k.to_string(), x);
    }
    let r = rpn::run(code, &st, &bind::KEvents);
    if !r.unknown.is_empty() || !r.writes.is_empty() {
        return None;
    }
    match r.stack.last() {
        Some(rpn::Val::Num(n)) => Some(*n),
        Some(rpn::Val::Str(_)) => None,
        None => Some(0.0),
    }
}

/// Every variable the code reads, whichever branches run.
fn reads_of(code: &str, consts: &HashMap<String, f64>) -> BTreeSet<String> {
    let mut reads: BTreeSet<String> = BTreeSet::new();
    for round in 0..4 {
        let mut st = consts.clone();
        for k in &reads {
            st.insert(k.clone(), if round % 2 == 1 { 1.0 } else { 0.0 });
        }
        reads.extend(rpn::run(code, &st, &bind::KEvents).reads);
    }
    reads.retain(|k| !consts.contains_key(k));
    reads
}

/// Whether every read of `key` in the code is in a boolean unit.
fn bool_reads(code: &str, key: &str) -> bool {
    rpn::tokenize(code).iter().all(|t| match t {
        rpn::Tok::Var { write: false, kind, name, unit } if rpn::key(kind, name) == key => {
            unit.eq_ignore_ascii_case("bool") || unit.eq_ignore_ascii_case("boolean")
        }
        _ => true,
    })
}

/// How one code drives its node.
fn drive(code: &str, consts: &HashMap<String, f64>, lua: &mut Vec<String>, node: &str) -> (Result<Drive, String>, BTreeSet<String>) {
    let mut reads = BTreeSet::new();
    let body = match to_lua_with(code, &format!("lt_{}", bind::sanitize_id(node)), consts) {
        Ok(b) => b,
        Err(e) => return (Err(e), reads),
    };
    let keys = reads_of(code, consts);
    for k in &keys {
        if let Some(d) = bind::sys_dataref(k) {
            reads.insert(d);
        }
    }
    if keys.is_empty() {
        return (value_at(code, None, 0.0, consts).map(Drive::Const).ok_or_else(|| "code does not evaluate".to_string()), reads);
    }
    if keys.len() == 1 {
        let k = keys.iter().next().unwrap();
        if let Some(dref) = bind::sys_dataref(k) {
            let samples: &[f64] = if bool_reads(code, k) { &[0.0, 1.0] } else { &[0.0, 1.0, 0.25, 0.5, 2.0, 100.0, -1.0] };
            let vals: Option<Vec<f64>> = samples.iter().map(|&x| value_at(code, Some(k), x, consts)).collect();
            if let Some(vals) = vals {
                let (b, a) = (vals[0], vals[1] - vals[0]);
                let linear = samples.iter().zip(&vals).all(|(x, y)| (a * x + b - y).abs() < 1e-9);
                if linear && a.abs() > 1e-12 {
                    return (Ok(Drive::Direct { dref, v1: -b / a, v2: (1.0 - b) / a }), reads);
                }
            }
        }
    }
    let f = format!("{}\n{body}return s[#s] or 0", bind::locals_for(&body));
    let i = match lua.iter().position(|x| x == &f) {
        Some(i) => i,
        None => {
            lua.push(f);
            lua.len() - 1
        }
    };
    (Ok(Drive::Lua(i)), reads)
}

/// Resolve every node's light and visibility code.
pub fn resolve(codes: &[LightCode], consts: &HashMap<String, f64>) -> Lights {
    let mut out = Lights::default();
    // The last code per node and kind.
    let mut last: BTreeMap<(String, LightKind), &LightCode> = BTreeMap::new();
    for c in codes {
        let Some(node) = c.node.as_deref().map(str::trim).filter(|n| !n.is_empty()) else { continue };
        if last.insert((node.to_ascii_lowercase(), c.kind), c).is_some_and(|old| old.code != c.code) {
            out.overridden += 1;
        }
    }
    for ((_, kind), c) in last {
        let node = c.node.as_deref().unwrap_or("").trim().to_string();
        let (drive, reads) = drive(&c.code, consts, &mut out.lua, &node);
        out.lights.push(Light { node, template: c.template.clone(), kind, code: c.code.clone(), drive, reads });
    }
    out
}
