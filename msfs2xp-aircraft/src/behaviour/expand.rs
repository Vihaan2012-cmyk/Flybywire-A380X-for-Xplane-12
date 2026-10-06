//! Template expansion: walks the model's components the way MSFS does,
//! carrying template parameters down through `UseTemplate` calls, and stops
//! at every template the package does not define (Asobo's own, like
//! `ASOBO_GT_Push_Button`), recording the call with its parameters.

use std::collections::HashMap;

use super::rpn;
use super::xml::{expand_macros, El, Library};

pub type Scope = HashMap<String, String>;

/// MSFS's own `ModelBehaviors` engine matches a `UseTemplate` parameter's
/// element name against `#NAME#` references (and `Check`/`Valid`/`Switch
/// Param` attributes) case-insensitively: FBW's own A32NX-derived legacy
/// templates rely on this (`Node_ID` set by
/// `A32NX_AIRBUS_DATA_SWITCHING_TEMPLATE`'s `DefaultTemplateParameters` and
/// by its and `A32NX_AIRBUS_XBLEED_SELECTOR_TEMPLATE`'s callers, read back
/// as `#NODE_ID#` by `ASOBO_GT_Switch_3States` and the templates' own
/// `Component`/`ANIM_NAME`/`SWITCH_POSITION_VAR`) — a working, physically
/// modelled knob the game ships, so the engine cannot be case-sensitive
/// here. Every scope key is canonicalized through this so `Node_ID` and
/// `NODE_ID` are the same parameter.
fn canon(name: &str) -> String {
    name.trim().to_ascii_uppercase()
}

/// A call to a template the package does not define.
#[derive(Clone, Debug)]
pub struct Leaf {
    pub template: String,
    /// The innermost enclosing component's node.
    pub node: Option<String>,
    pub params: Scope,
    /// The package templates called to reach this leaf, outermost first
    /// (Asobo's own routing helpers excluded, they push their own name).
    /// Diagnostic: which `UseTemplate` chain left a parameter unresolved.
    pub chain: Vec<String>,
}

impl Leaf {
    pub fn get(&self, name: &str) -> Option<&str> {
        self.params.get(&canon(name)).map(String::as_str)
    }
}

/// What a component's material or visibility code drives.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum LightKind {
    /// `<Material><EmissiveFactor><Parameter>`: how bright the node glows.
    Emissive,
    /// `<Visibility><Parameter>`: whether the node is drawn.
    Visibility,
}

/// An emissive or visibility code on a component's node, as MSFS runs it.
#[derive(Clone, Debug)]
pub struct LightCode {
    pub node: Option<String>,
    pub kind: LightKind,
    pub code: String,
    /// The innermost package template that made it (e.g. `FBW_Push_Toggle`).
    pub template: String,
}

/// Code a component runs on its own (`<Update>`, `ASOBO_GT_Update`).
#[derive(Clone, Debug)]
pub struct UpdateCode {
    pub node: Option<String>,
    /// The innermost package template it is in.
    pub template: String,
    pub code: String,
    /// Runs per second; every frame when not given.
    pub frequency: Option<f64>,
    /// Runs once, at load.
    pub once: bool,
}

/// One `<SoundEvent>` inside an `<AnimationTriggers>` block, as Asobo's own
/// `ASOBO_GT_AnimTriggers_SoundEvent`/`_SoundEvents_Same`/`_2SoundEvents`
/// templates write it (only reached when expanded with MSFS's own template
/// definitions, exactly like [`LightCode`]). `sound.xml`'s own
/// `<AnimationSounds>` entries give the `WwiseEvent` names but almost never
/// which animation plays them (141 of the FlyByWire A380X's 143 carry no
/// `NodeName`); this is the only place that pairing exists at all, since
/// MSFS compiles it straight from the model behaviour XML, never into
/// sound.xml.
#[derive(Clone, Debug)]
pub struct SoundTriggerCode {
    pub node: Option<String>,
    /// The innermost package template that made it (e.g.
    /// `FBW_A380X_BacklightIndicator_Button_Template`).
    pub template: String,
    /// `<AnimationTriggers Animation="...">`: the clip name this plugin's
    /// own click position already tracks as `fbw/cockpit/<anim>` (`rig.rs`).
    pub anim: String,
    /// `<EventTrigger NormalizedTime="...">`: where in the clip's 0..1
    /// travel this fires. Mutually exclusive with `count` below —
    /// `ASOBO_GT_AnimTriggers_SoundEvent`/`_2SoundEvents` set this and never
    /// `Count`; `ASOBO_GT_AnimTriggers_SoundEvents_Same` is the reverse
    /// (confirmed by reading both templates: `.../Asobo/Generic/AnimationTriggers.xml`).
    pub normalized_time: Option<f64>,
    /// `<EventTrigger Count="...">`: fire this many times, evenly spaced
    /// across the clip's 0..1 travel, instead of once at a fixed point
    /// (`ASOBO_GT_AnimTriggers_SoundEvents_Same`; FlyByWire uses `Count=1`
    /// for the flap-lever cover and `Count=3` for the speedbrake lever,
    /// `A32NX_Interior_Handling.xml`). The evenly-spaced points themselves
    /// are computed downstream, in the plugin's `sound::anim_triggers`, not
    /// here — this converter stays a straight transcription of what the
    /// XML says.
    pub count: Option<u32>,
    /// `<EventTrigger Direction="...">`: `"Forward"`, `"Backward"`, or
    /// `"Both"` (Asobo's default for the single-event template).
    pub direction: String,
    /// `<SoundEvent Action="...">`: `"Play"` is the only one this converter
    /// keeps — a `"Stop"` action needs runtime state (which channel to stop)
    /// this reader does not have, the same scope choice the plugin's own
    /// `sound/triggers.rs` already documents for sound.xml's own actions.
    pub action: String,
    pub wwise_event: String,
}

#[derive(Default)]
pub struct Expansion {
    pub leaves: Vec<Leaf>,
    pub warnings: Vec<String>,
    /// Emissive and visibility codes (only when expanded with MSFS's own
    /// template definitions, which is where they are written).
    pub lights: Vec<LightCode>,
    /// The package's update codes (from the expansion without MSFS's own
    /// definitions, so each is counted once).
    pub updates: Vec<UpdateCode>,
    /// Animation-triggered sound events (only when expanded with MSFS's own
    /// template definitions; see [`SoundTriggerCode`]).
    pub sounds: Vec<SoundTriggerCode>,
}

/// Replace `#NAME#` with parameters in scope; unknown names stay as they are.
pub fn subst(text: &str, scope: &Scope) -> String {
    let mut cur = text.to_string();
    for _ in 0..6 {
        let next = subst_once(&cur, scope);
        if next == cur {
            break;
        }
        cur = next;
    }
    cur
}

fn subst_once(text: &str, scope: &Scope) -> String {
    let mut out = String::with_capacity(text.len());
    let mut rest = text;
    while let Some(i) = rest.find('#') {
        out.push_str(&rest[..i]);
        let after = &rest[i + 1..];
        let end = after.find(|c: char| !(c.is_ascii_alphanumeric() || c == '_'));
        match end {
            Some(e) if e > 0 && after[e..].starts_with('#') => {
                let name = &after[..e];
                match scope.get(&canon(name)) {
                    Some(v) => out.push_str(v),
                    None => {
                        out.push('#');
                        out.push_str(name);
                        out.push('#');
                    }
                }
                rest = &after[e + 1..];
            }
            _ => {
                out.push('#');
                rest = after;
            }
        }
    }
    out.push_str(rest);
    out
}

fn is_false(v: &str) -> bool {
    let t = v.trim();
    t.is_empty() || t.eq_ignore_ascii_case("false") || t == "0"
}

/// A `Check`/`Valid`/`NotEmpty` test on an element's attributes, if it has one.
fn attr_test(el: &El, scope: &Scope) -> Option<bool> {
    if let Some(p) = el.attr("Check") {
        let v = scope.get(&canon(p));
        return Some(match el.attr("Match") {
            Some(m) => v.is_some_and(|v| v.trim() == subst(m, scope).trim()),
            None => v.is_some(),
        });
    }
    if let Some(p) = el.attr("Valid") {
        return Some(scope.get(&canon(p)).is_some_and(|v| !is_false(v)));
    }
    if let Some(p) = el.attr("NotEmpty") {
        return Some(scope.get(&canon(p)).is_some_and(|v| !v.trim().is_empty()));
    }
    None
}

fn operand(el: &El, scope: &Scope) -> String {
    let t = subst(el.text.trim(), scope);
    match el.name.as_str() {
        "Value" => scope.get(&canon(&t)).map(|v| subst(v, scope)).unwrap_or_default(),
        _ => t,
    }
}

fn eval_test(el: &El, scope: &Scope) -> bool {
    match el.name.as_str() {
        "Test" => el.kids.iter().all(|k| eval_test(k, scope)),
        "And" => el.kids.iter().all(|k| eval_test(k, scope)),
        "Or" => el.kids.iter().any(|k| eval_test(k, scope)),
        "Not" => !el.kids.iter().all(|k| eval_test(k, scope)),
        "Arg" => attr_test(el, scope).unwrap_or(false),
        "Equal" | "NotEqual" | "Greater" | "Lower" | "GreaterOrEqual" | "LowerOrEqual" => {
            let ops: Vec<String> = el.kids.iter().map(|k| operand(k, scope)).collect();
            if ops.len() != 2 {
                return false;
            }
            let (a, b) = (ops[0].trim(), ops[1].trim());
            let ord = match (a.parse::<f64>(), b.parse::<f64>()) {
                (Ok(x), Ok(y)) => x.partial_cmp(&y),
                _ => Some(a.cmp(b)),
            };
            let Some(o) = ord else { return false };
            use std::cmp::Ordering::*;
            match el.name.as_str() {
                "Equal" => o == Equal,
                "NotEqual" => o != Equal,
                "Greater" => o == Greater,
                "Lower" => o == Less,
                "GreaterOrEqual" => o != Less,
                _ => o != Greater,
            }
        }
        _ => attr_test(el, scope).unwrap_or(false),
    }
}

/// The children a `Condition` or `Switch` selects.
fn select<'a>(el: &'a El, scope: &Scope) -> Vec<&'a El> {
    match el.name.as_str() {
        "Condition" => {
            let ok = match attr_test(el, scope) {
                Some(v) => v,
                None => el.kids.iter().find(|k| k.name == "Test").is_none_or(|t| eval_test(t, scope)),
            };
            let branch = el.kids.iter().find(|k| k.name.eq_ignore_ascii_case(if ok { "True" } else { "False" }));
            let has_branches = el.kids.iter().any(|k| k.name.eq_ignore_ascii_case("True") || k.name.eq_ignore_ascii_case("False"));
            if has_branches {
                branch.map(|b| b.kids.iter().collect()).unwrap_or_default()
            } else if ok {
                el.kids.iter().filter(|k| k.name != "Test").collect()
            } else {
                Vec::new()
            }
        }
        "Switch" => {
            let value = el.attr("Param").map(|p| scope.get(&canon(p)).map(|v| subst(v, scope)).unwrap_or_default());
            for case in &el.kids {
                let hit = match case.name.as_str() {
                    "Case" => match (case.attr("Value"), &value) {
                        (Some(cv), Some(v)) => {
                            let (cv, v) = (subst(cv, scope), v.trim().to_string());
                            match (cv.trim().parse::<f64>(), v.parse::<f64>()) {
                                (Ok(x), Ok(y)) => x == y,
                                _ => cv.trim() == v,
                            }
                        }
                        _ => attr_test(case, scope).unwrap_or(false),
                    },
                    "Default" => true,
                    _ => false,
                };
                if hit {
                    return case.kids.iter().collect();
                }
            }
            Vec::new()
        }
        _ => Vec::new(),
    }
}

/// Set parameters from a parameter list (a `UseTemplate`'s children, or a
/// `DefaultTemplateParameters` block). `force`: overwrite ones already set.
fn apply_params(kids: &[&El], scope: &mut Scope, force: bool) {
    for el in kids {
        match el.name.as_str() {
            "Condition" | "Switch" => {
                let sel = select(el, scope);
                apply_params(&sel, scope, force);
            }
            "DefaultTemplateParameters" => apply_params(&el.kids.iter().collect::<Vec<_>>(), scope, false),
            "OverrideTemplateParameters" => apply_params(&el.kids.iter().collect::<Vec<_>>(), scope, true),
            "Parameters" => {
                let f = el.attr("Type").is_some_and(|t| t.eq_ignore_ascii_case("Override"));
                apply_params(&el.kids.iter().collect::<Vec<_>>(), scope, f);
            }
            name => {
                // MSFS builds some parameter names from parameters.
                let name = canon(&subst(name, scope));
                if !force && scope.contains_key(&name) {
                    continue;
                }
                let raw = subst(&el.text, scope);
                let value = match el.attr("Process").map(|p| p.to_ascii_lowercase()) {
                    Some(p) if p == "int" => rpn::eval_number(&raw).map_or(raw.clone(), |n| format!("{}", n.trunc() as i64)),
                    Some(p) if p == "float" => rpn::eval_number(&raw).map_or(raw.clone(), |n| format!("{n}")),
                    Some(p) if p == "param" => scope.get(&canon(raw.trim())).cloned().unwrap_or_default(),
                    _ => raw,
                };
                scope.insert(name, value);
            }
        }
    }
}

struct Walker<'a> {
    lib: &'a Library,
    /// MSFS's own templates, for calls the package does not define.
    base: Option<&'a Library>,
    out: Expansion,
    chain: Vec<String>,
}

/// A `<Parameter>`'s value as RPN: its `<Code>`, or its `<Sim>` variable
/// with scale and bias (as ASOBO_GT_Material_Emissive_Sim writes it).
fn parameter_code(param: &El, scope: &Scope) -> Option<String> {
    if let Some(code) = param.kids.iter().find(|k| k.name == "Code") {
        return Some(subst(code.text.trim(), scope));
    }
    let sim = param.kids.iter().find(|k| k.name == "Sim")?;
    let field = |n: &str| sim.kids.iter().find(|k| k.name == n).map(|k| subst(k.text.trim(), scope));
    let var = field("Variable")?;
    let units = field("Units").unwrap_or_else(|| "Number".into());
    let scale = field("Scale").unwrap_or_else(|| "1".into());
    let bias = field("Bias").unwrap_or_else(|| "0".into());
    Some(format!("(A:{var}, {units}) {scale} * {bias} +"))
}

impl Walker<'_> {
    fn body(&mut self, kids: &[&El], scope: &mut Scope, node: &Option<String>) {
        for el in kids {
            match el.name.as_str() {
                "Component" => {
                    let mut inner = scope.clone();
                    let n = el.attr("Node").map(|n| subst(n, scope).trim().to_string()).filter(|n| !n.is_empty() && !n.contains('#'));
                    let node = n.or_else(|| node.clone());
                    self.body(&el.kids.iter().collect::<Vec<_>>(), &mut inner, &node);
                }
                "UseTemplate" => {
                    let name = subst(el.attr("Name").unwrap_or(""), scope).trim().to_string();
                    let mut callee = scope.clone();
                    apply_params(&el.kids.iter().collect::<Vec<_>>(), &mut callee, true);
                    self.call(&name, callee, node);
                }
                "Condition" | "Switch" => {
                    let sel = select(el, scope);
                    self.body(&sel, scope, node);
                }
                "DefaultTemplateParameters" | "OverrideTemplateParameters" | "Parameters" => {
                    apply_params(&[el], scope, false);
                }
                "Loop" => {
                    // MSFS's own input-event templates loop over bindings;
                    // nothing an emissive or a control needs is in them.
                    let name = self.chain.last().map_or("", String::as_str);
                    if self.lib.templates.contains_key(name) {
                        self.out.warnings.push(format!("Loop in {name} not expanded"));
                    }
                }
                "Update" if self.base.is_none() => {
                    let code = subst(el.text.trim(), scope);
                    if !code.trim().is_empty() {
                        let frequency = el.attr("Frequency").and_then(|f| subst(f, scope).trim().parse().ok());
                        let once = el.attr("Once").is_some_and(|o| subst(o, scope).trim().eq_ignore_ascii_case("true"));
                        let template = self.chain.last().cloned().unwrap_or_default();
                        let code = expand_macros(&code, &self.lib.macros);
                        self.out.updates.push(UpdateCode { node: node.clone(), template, code, frequency, once });
                    }
                }
                "Material" | "Visibility" if self.base.is_some() => {
                    let (kind, holder) = if el.name == "Material" {
                        (LightKind::Emissive, el.kids.iter().find(|k| k.name == "EmissiveFactor"))
                    } else {
                        (LightKind::Visibility, Some(*el))
                    };
                    let Some(param) = holder.and_then(|h| h.kids.iter().find(|k| k.name == "Parameter")) else { continue };
                    let Some(code) = parameter_code(param, scope) else { continue };
                    let template = self.chain.iter().rev().find(|t| self.lib.templates.contains_key(*t)).cloned().unwrap_or_default();
                    let code = expand_macros(&code, &self.lib.macros);
                    self.out.lights.push(LightCode { node: node.clone(), kind, code, template });
                }
                // Asobo's own `ASOBO_GT_AnimTriggers_SoundEvent` /
                // `_SoundEvents_Same` / `_2SoundEvents` templates (only
                // reached with MSFS's own definitions loaded, same as
                // Material/Visibility above): the animation-name -> Wwise
                // event pairing sound.xml itself never carries.
                "AnimationTriggers" if self.base.is_some() => {
                    let Some(anim_raw) = el.attr("Animation") else { continue };
                    let anim = subst(anim_raw, scope).trim().to_string();
                    if anim.is_empty() || anim.contains('#') {
                        continue;
                    }
                    let template = self.chain.iter().rev().find(|t| self.lib.templates.contains_key(*t)).cloned().unwrap_or_default();
                    for trigger in el.kids.iter().filter(|k| k.name == "EventTrigger") {
                        let normalized_time = trigger.attr("NormalizedTime").and_then(|v| subst(v, scope).trim().parse().ok());
                        // `ASOBO_GT_AnimTriggers_SoundEvents_Same` sets
                        // `Count` instead of `NormalizedTime` (the two are
                        // never both present on a real `<EventTrigger>` —
                        // see `SoundTriggerCode::count`'s doc).
                        let count = trigger.attr("Count").and_then(|v| subst(v, scope).trim().parse().ok());
                        let direction = trigger.attr("Direction").map(|v| subst(v, scope).trim().to_string()).unwrap_or_else(|| "Both".to_string());
                        for sound in trigger.kids.iter().filter(|k| k.name == "SoundEvent") {
                            let Some(raw_event) = sound.attr("WwiseEvent") else { continue };
                            let wwise_event = subst(raw_event, scope).trim().to_string();
                            if wwise_event.is_empty() || wwise_event.contains('#') {
                                continue;
                            }
                            let action = sound.attr("Action").map(|v| subst(v, scope).trim().to_string()).unwrap_or_else(|| "Play".to_string());
                            self.out.sounds.push(SoundTriggerCode {
                                node: node.clone(),
                                template: template.clone(),
                                anim: anim.clone(),
                                normalized_time,
                                count,
                                direction: direction.clone(),
                                action,
                                wwise_event,
                            });
                        }
                    }
                }
                _ => {}
            }
        }
    }

    fn call(&mut self, name: &str, mut scope: Scope, node: &Option<String>) {
        if self.chain.len() > 96 {
            self.out.warnings.push(format!("template recursion too deep at {name}"));
            return;
        }
        let helper = matches!(name, "ASOBO_GT_Helper_Suffix_ID_Appender" | "ASOBO_GT_Helper_Recursive_ID");
        let def = self.lib.templates.get(name).or_else(|| if helper { None } else { self.base.and_then(|b| b.templates.get(name)) });
        match def {
            // Asobo's routing helpers, which the package calls but does not
            // define, done here as Asobo's Generic/Helpers.xml does them.
            None if name == "ASOBO_GT_Helper_Suffix_ID_Appender" => {
                if !scope.contains_key("SUFFIX_ID") {
                    let valid = |k: &str| scope.get(k).is_some_and(|v| !is_false(v));
                    let suffix = if valid("DONT_APPEND_ID") {
                        String::new()
                    } else if scope.get("CONTAINER_ID").is_some_and(|v| !v.trim().is_empty()) {
                        format!("_{}", scope["CONTAINER_ID"])
                    } else {
                        format!("_{}", scope.get("ID").cloned().unwrap_or_default())
                    };
                    scope.insert("SUFFIX_ID".into(), suffix);
                }
                let target = subst(scope.get("TEMPLATE_TO_CALL").map_or("", String::as_str), &scope).trim().to_string();
                self.chain.push(name.to_string());
                self.call(&target, scope, node);
                self.chain.pop();
            }
            None if name == "ASOBO_GT_Helper_Recursive_ID" => {
                let int = |k: &str, d: i64| scope.get(k).and_then(|v| rpn::eval_number(&subst(v, &scope))).map_or(d, |n| n as i64);
                let (first, max) = (int("FIRST_ID", 1), int("MAX_ID", 1));
                scope.entry("PARAM1".into()).or_insert_with(|| "ID".into());
                let exit = subst(scope.get("EXIT_TEMPLATE").map_or("", String::as_str), &scope).trim().to_string();
                self.chain.push(name.to_string());
                for id in first..=max.min(first + 256) {
                    let mut sc = scope.clone();
                    sc.insert("NEXT_ID".into(), id.to_string());
                    for k in 1.. {
                        let Some(pname) = sc.get(&format!("PARAM{k}")).filter(|v| !is_false(v)).map(|v| v.trim().to_string()) else { break };
                        let pre = sc.get(&format!("PARAM{k}_PREFIX")).cloned().unwrap_or_default();
                        let post = sc.get(&format!("PARAM{k}_SUFFIX")).cloned().unwrap_or_default();
                        let mut v = format!("{pre}{id}{post}");
                        if sc.get(&format!("PROCESS_PARAM{k}")).is_some_and(|x| !is_false(x)) {
                            v = sc.get(&canon(&v)).cloned().unwrap_or_default();
                        }
                        sc.insert(canon(&pname), v);
                    }
                    self.call(&exit, sc, node);
                }
                self.chain.pop();
            }
            Some(t) => {
                self.chain.push(name.to_string());
                self.body(&t.kids.iter().collect::<Vec<_>>(), &mut scope, node);
                self.chain.pop();
            }
            None => {
                let keys: Vec<String> = scope.keys().cloned().collect();
                for k in keys {
                    let v = expand_macros(&subst(&scope[&k], &scope), &self.lib.macros);
                    scope.insert(k, v);
                }
                if name == "ASOBO_GT_Update" && self.base.is_none() {
                    // Asobo's Generic/Updates.xml: UPDATE_CODE every FREQUENCY
                    // per second, or once (UPDATE_ONCE).
                    let get = |k: &str| scope.get(k).map(|v| v.trim().to_string()).filter(|v| !v.is_empty());
                    if let Some(code) = get("UPDATE_CODE") {
                        let template = self.chain.last().cloned().unwrap_or_default();
                        let frequency = get("FREQUENCY").and_then(|f| f.parse().ok());
                        let once = get("UPDATE_ONCE").is_some_and(|o| o.eq_ignore_ascii_case("true"));
                        let code = expand_macros(&code, &self.lib.macros);
                        self.out.updates.push(UpdateCode { node: node.clone(), template, code, frequency, once });
                    }
                }
                self.out.leaves.push(Leaf { template: name.to_string(), node: node.clone(), params: scope, chain: self.chain.clone() });
            }
        }
    }
}

/// Expand every top-level component of the library.
pub fn expand(lib: &Library) -> Expansion {
    expand_with(lib, None)
}

/// Expand with MSFS's own template definitions for the calls the package
/// does not define: down to the materials and visibility codes those
/// templates write.
pub fn expand_with(lib: &Library, base: Option<&Library>) -> Expansion {
    let mut w = Walker { lib, base, out: Expansion::default(), chain: Vec::new() };
    for c in &lib.components {
        let mut scope = Scope::new();
        w.body(&[c], &mut scope, &None);
    }
    w.out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parameters_substitute_and_default() {
        let s: Scope = [("ID".to_string(), "2".to_string())].into();
        assert_eq!(subst("ENG_#ID#_#X#", &s), "ENG_2_#X#");
        let lib = Library::from_text(
            r##"<ModelBehaviors>
                <Template Name="T">
                    <DefaultTemplateParameters><NODE_ID>N_#ID#</NODE_ID><ID>1</ID></DefaultTemplateParameters>
                    <Component ID="#NODE_ID#" Node="#NODE_ID#">
                        <Condition Check="EXTRA"><True><UseTemplate Name="LEAF"><A>yes</A></UseTemplate></True>
                        <False><UseTemplate Name="LEAF"><A>no</A></UseTemplate></False></Condition>
                    </Component>
                </Template>
                <Component ID="root">
                    <UseTemplate Name="T"><ID>3</ID></UseTemplate>
                    <UseTemplate Name="T"><EXTRA/></UseTemplate>
                </Component>
            </ModelBehaviors>"##,
        )
        .unwrap();
        let e = expand(&lib);
        assert_eq!(e.leaves.len(), 2);
        // NODE_ID defaults before ID does, so the first call has no ID yet
        // when the default is read... unless it was passed in.
        assert_eq!(e.leaves[0].node.as_deref(), Some("N_3"));
        assert_eq!(e.leaves[0].get("A"), Some("no"));
        assert_eq!(e.leaves[1].get("A"), Some("yes"));
    }

    /// The pairing that makes W101's fix possible at all: MSFS's own
    /// `ASOBO_GT_Push_Button` (Asobo's template, standing in for the real
    /// one) bottoms out in an `<AnimationTriggers>` this walker must reach
    /// through the package's `FBW_...` wrapper template, and must NOT reach
    /// without `base` (a package expanding without MSFS's own definitions
    /// must not silently invent sound triggers).
    #[test]
    fn animation_triggers_pair_the_clip_with_its_wwise_events() {
        let base = Library::from_text(
            r##"<ModelBehaviors>
                <Template Name="ASOBO_GT_Push_Button">
                    <AnimationTriggers Animation="#ANIM_NAME#">
                        <EventTrigger NormalizedTime="0.1" Direction="Forward">
                            <SoundEvent WwiseEvent="#WWISE_EVENT_1#" Action="Play"/>
                        </EventTrigger>
                        <EventTrigger NormalizedTime="0.9" Direction="Backward">
                            <SoundEvent WwiseEvent="#WWISE_EVENT_2#" Action="Play"/>
                        </EventTrigger>
                    </AnimationTriggers>
                </Template>
            </ModelBehaviors>"##,
        )
        .unwrap();
        let lib = Library::from_text(
            r##"<ModelBehaviors>
                <Template Name="FBW_Wrap">
                    <UseTemplate Name="ASOBO_GT_Push_Button">
                        <ANIM_NAME>#ANIM_NAME#</ANIM_NAME>
                        <WWISE_EVENT_1>#WWISE_EVENT_1#</WWISE_EVENT_1>
                        <WWISE_EVENT_2>#WWISE_EVENT_2#</WWISE_EVENT_2>
                    </UseTemplate>
                </Template>
                <Component ID="c" Node="BATTERY_SWITCH">
                    <UseTemplate Name="FBW_Wrap">
                        <ANIM_NAME>BATTERY_SWITCH</ANIM_NAME>
                        <WWISE_EVENT_1>battery_switch_on</WWISE_EVENT_1>
                        <WWISE_EVENT_2>battery_switch_off</WWISE_EVENT_2>
                    </UseTemplate>
                </Component>
            </ModelBehaviors>"##,
        )
        .unwrap();
        let full = expand_with(&lib, Some(&base));
        assert_eq!(full.sounds.len(), 2);
        assert_eq!(full.sounds[0].anim, "BATTERY_SWITCH");
        assert_eq!(full.sounds[0].direction, "Forward");
        assert_eq!(full.sounds[0].normalized_time, Some(0.1));
        assert_eq!(full.sounds[0].count, None);
        assert_eq!(full.sounds[0].wwise_event, "battery_switch_on");
        assert_eq!(full.sounds[0].action, "Play");
        assert_eq!(full.sounds[1].direction, "Backward");
        assert_eq!(full.sounds[1].wwise_event, "battery_switch_off");

        // Without MSFS's own definitions, ASOBO_GT_Push_Button is an
        // unresolved leaf, not something whose body gets walked: no sounds.
        let no_base = expand(&lib);
        assert!(no_base.sounds.is_empty());
    }

    /// `ASOBO_GT_AnimTriggers_SoundEvents_Same` (real FlyByWire usage: the
    /// flap-lever cover, `Count=1`, and the speedbrake lever, `Count=3`,
    /// `A32NX_Interior_Handling.xml`): `Count`, not `NormalizedTime`, and
    /// this walker must capture it instead of silently leaving
    /// `normalized_time` (and now `count`) both absent.
    #[test]
    fn animation_triggers_capture_count_instead_of_normalized_time() {
        let base = Library::from_text(
            r##"<ModelBehaviors>
                <Template Name="ASOBO_GT_AnimTriggers_SoundEvents_Same">
                    <AnimationTriggers Animation="#ANIM_NAME#">
                        <EventTrigger Count="#COUNT#" Direction="#AUDIO_DIRECTION#">
                            <SoundEvent WwiseEvent="#WWISE_EVENT#" Action="Play"/>
                        </EventTrigger>
                    </AnimationTriggers>
                </Template>
            </ModelBehaviors>"##,
        )
        .unwrap();
        let lib = Library::from_text(
            r##"<ModelBehaviors>
                <Component ID="c" Node="HANDLING_Lever_Spoilers">
                    <UseTemplate Name="ASOBO_GT_AnimTriggers_SoundEvents_Same">
                        <ANIM_NAME>HANDLING_Lever_Spoilers</ANIM_NAME>
                        <COUNT>3</COUNT>
                        <AUDIO_DIRECTION>Both</AUDIO_DIRECTION>
                        <WWISE_EVENT>lever_speedbrakes</WWISE_EVENT>
                    </UseTemplate>
                </Component>
            </ModelBehaviors>"##,
        )
        .unwrap();
        let full = expand_with(&lib, Some(&base));
        assert_eq!(full.sounds.len(), 1);
        assert_eq!(full.sounds[0].anim, "HANDLING_Lever_Spoilers");
        assert_eq!(full.sounds[0].normalized_time, None, "Count-based triggers must not get a fabricated NormalizedTime here");
        assert_eq!(full.sounds[0].count, Some(3));
        assert_eq!(full.sounds[0].wwise_event, "lever_speedbrakes");
    }

    #[test]
    fn recursion_with_int_processing() {
        let lib = Library::from_text(
            r##"<ModelBehaviors>
                <Template Name="R">
                    <Condition><Test><Greater><Value>N</Value><Number>0</Number></Greater></Test>
                        <True><UseTemplate Name="R"><CODE>#N# #CODE#</CODE><N Process="Int">#N# 1 -</N></UseTemplate></True>
                        <False><UseTemplate Name="LEAF"/></False>
                    </Condition>
                </Template>
                <Component ID="c" Node="X"><UseTemplate Name="R"><N>3</N><CODE></CODE></UseTemplate></Component>
            </ModelBehaviors>"##,
        )
        .unwrap();
        let e = expand(&lib);
        assert_eq!(e.leaves.len(), 1);
        assert_eq!(e.leaves[0].get("CODE").map(str::split_whitespace).map(|w| w.collect::<Vec<_>>()), Some(vec!["1", "2", "3"]));
        assert_eq!(e.leaves[0].node.as_deref(), Some("X"));
    }

    // FBW's own A32NX-derived legacy templates (Airbus.xml
    // A32NX_AIRBUS_DATA_SWITCHING_TEMPLATE and its
    // A32NX_AIRBUS_XBLEED_SELECTOR_TEMPLATE caller) set a caller-given
    // `Node_ID` but read it back as `#NODE_ID#` for the `Component` and the
    // nested `ASOBO_GT_Switch_3States`'s `ANIM_NAME`/`SWITCH_POSITION_VAR` —
    // a knob the game ships and that works, so MSFS's own engine must match
    // parameter names case-insensitively. Without that, this caller's
    // `Component Node="#NODE_ID#"` itself never resolves (no model node at
    // all, not just an unresolved leaf).

    #[test]
    fn a_parameter_set_in_one_case_is_read_back_in_another() {
        let lib = Library::from_text(
            r##"<ModelBehaviors>
                <Template Name="A32NX_AIRBUS_XBLEED_SELECTOR_TEMPLATE">
                    <Component ID="#NODE_ID#" Node="#NODE_ID#">
                        <UseTemplate Name="LEAF">
                            <ANIM_NAME>#NODE_ID#</ANIM_NAME>
                            <SWITCH_POSITION_VAR>A32NX_#NODE_ID#_Position</SWITCH_POSITION_VAR>
                        </UseTemplate>
                    </Component>
                </Template>
                <Component ID="root">
                    <UseTemplate Name="A32NX_AIRBUS_XBLEED_SELECTOR_TEMPLATE">
                        <Node_ID>KNOB_OVHD_AIRCOND_XBLEED</Node_ID>
                    </UseTemplate>
                </Component>
            </ModelBehaviors>"##,
        )
        .unwrap();
        let e = expand(&lib);
        assert_eq!(e.leaves.len(), 1);
        assert_eq!(e.leaves[0].node.as_deref(), Some("KNOB_OVHD_AIRCOND_XBLEED"));
        assert_eq!(e.leaves[0].get("ANIM_NAME"), Some("KNOB_OVHD_AIRCOND_XBLEED"));
        assert_eq!(e.leaves[0].get("SWITCH_POSITION_VAR"), Some("A32NX_KNOB_OVHD_AIRCOND_XBLEED_Position"));
    }

    #[test]
    fn a_default_template_parameter_set_in_one_case_is_read_back_in_another() {
        // A32NX_AIRBUS_DATA_SWITCHING_TEMPLATE's own DefaultTemplateParameters
        // sets `Node_ID`; its body and nested UseTemplate read `#NODE_ID#`.
        let lib = Library::from_text(
            r##"<ModelBehaviors>
                <Template Name="T">
                    <DefaultTemplateParameters>
                        <Node_ID>KNOB_SWITCHING_#ID#</Node_ID>
                    </DefaultTemplateParameters>
                    <Component ID="#NODE_ID#" Node="#NODE_ID#">
                        <UseTemplate Name="LEAF"><ANIM_NAME>#NODE_ID#</ANIM_NAME></UseTemplate>
                    </Component>
                </Template>
                <Component ID="root"><UseTemplate Name="T"><ID>1</ID></UseTemplate></Component>
            </ModelBehaviors>"##,
        )
        .unwrap();
        let e = expand(&lib);
        assert_eq!(e.leaves.len(), 1);
        assert_eq!(e.leaves[0].node.as_deref(), Some("KNOB_SWITCHING_1"));
        assert_eq!(e.leaves[0].get("ANIM_NAME"), Some("KNOB_SWITCHING_1"));
    }

    #[test]
    fn a_condition_check_matches_a_parameter_regardless_of_its_case() {
        let lib = Library::from_text(
            r##"<ModelBehaviors>
                <Template Name="T">
                    <Condition Check="toggle_simvar">
                        <True><UseTemplate Name="LEAF"><HIT>yes</HIT></UseTemplate></True>
                        <False><UseTemplate Name="LEAF"><HIT>no</HIT></UseTemplate></False>
                    </Condition>
                </Template>
                <Component ID="root"><UseTemplate Name="T"><TOGGLE_SIMVAR>L:X</TOGGLE_SIMVAR></UseTemplate></Component>
            </ModelBehaviors>"##,
        )
        .unwrap();
        let e = expand(&lib);
        assert_eq!(e.leaves.len(), 1);
        assert_eq!(e.leaves[0].get("HIT"), Some("yes"));
    }

    // FBW's own upstream gaps that the expander must still surface as
    // unresolved (not paper over): a caller with no cover-lock parameters at
    // all for a template that always builds one (keyboard.xml's
    // `#KEY)`-typo case is the same "leave it literal" shape, just with a
    // malformed placeholder instead of a merely-absent one).
    #[test]
    fn a_parameter_truly_never_given_stays_a_literal_placeholder() {
        let lib = Library::from_text(
            r##"<ModelBehaviors>
                <Template Name="COVERED">
                    <UseTemplate Name="LEAF">
                        <NODE_ID>#LOCK_NODE_ID#</NODE_ID>
                        <LEFT_SINGLE_CODE>(#TOGGLE_SIMVAR#_LOCK) ! (&gt;#TOGGLE_SIMVAR#_LOCK)</LEFT_SINGLE_CODE>
                    </UseTemplate>
                </Template>
                <Component ID="root">
                    <UseTemplate Name="COVERED"><NODE_ID>PUSH_GND_HF_DATA_LINK</NODE_ID></UseTemplate>
                </Component>
            </ModelBehaviors>"##,
        )
        .unwrap();
        let e = expand(&lib);
        assert_eq!(e.leaves.len(), 1);
        // LOCK_NODE_ID was never given by this caller, so it is left as a
        // literal, unresolved placeholder rather than falling back to
        // something invented.
        assert_eq!(e.leaves[0].get("NODE_ID"), Some("#LOCK_NODE_ID#"));
        assert_eq!(e.leaves[0].get("LEFT_SINGLE_CODE"), Some("(#TOGGLE_SIMVAR#_LOCK) ! (>#TOGGLE_SIMVAR#_LOCK)"));
        assert_eq!(e.leaves[0].chain, vec!["COVERED".to_string()]);
    }
}
