//! From expanded behaviours to X-Plane bindings: which variable each cockpit
//! control changes and how, as a direct manipulator where X-Plane can express
//! it and as SASL Lua (translated from the control's own MSFS code) where it
//! cannot.

use std::collections::{BTreeMap, BTreeSet, HashMap};

use super::events::{self, KAction, Step, Val};
use super::expand::Leaf;
use super::rpn::{self, Env, Events, Tok};

/// Engines on the aircraft, for events that set every engine at once.
pub const ENGINES: usize = 4;

/// A dataref name as the systems plugin publishes it: `fbw/` and the MSFS
/// variable name with everything but letters, digits, `_` and `/` made `_`.
pub fn dataref(var: &str) -> String {
    let s: String = var.chars().map(|c| if c.is_ascii_alphanumeric() || c == '_' || c == '/' { c } else { '_' }).collect();
    if let Ok(mut names) = VARIABLES.lock() {
        names.insert(var.to_string());
    }
    format!("fbw/{s}")
}

/// Every variable a binding reads or writes, by its own name (`A32NX_...`,
/// `GENERAL ENG MASTER ALTERNATOR:1`, `cockpit/...`), for the systems plugin
/// to publish before the cockpit's scripts look for them.
static VARIABLES: std::sync::Mutex<BTreeSet<String>> = std::sync::Mutex::new(BTreeSet::new());

/// The variables `dataref` has named so far.
pub fn variables() -> Vec<String> {
    VARIABLES.lock().map(|v| v.iter().cloned().collect()).unwrap_or_default()
}

/// Where a variable lives in X-Plane.
#[derive(Clone, Debug, PartialEq)]
enum Home {
    /// A systems dataref (`fbw/...`), or one of the cockpit's own
    /// (`fbw/cockpit/...`: covers without a variable, input events).
    Sys(String),
    /// XML-only state (`L:XMLVAR_*`, component `O:` variables), kept in Lua.
    Local(String),
    /// An X-Plane dataref X-Plane owns, times a factor into the variable's unit.
    XPlane(&'static str, f64),
}

/// Prefix of the L: variables standing for a cover that FlyByWire's XML
/// gives no variable (`FBW_Covered_Push_Toggle` without TOGGLE_SIMVAR).
pub const COVER_VAR: &str = "FBW_COCKPIT_COVER_";

/// The cockpit's own dataref for a cover without a variable.
pub fn cover_dataref(lock_node: &str) -> String {
    format!("fbw/cockpit/cover/{}", sanitize_id(lock_node))
}

/// `key` is `rpn::key` form (`L:NAME`, `A:NAME:1`, `O:NAME`).
fn home(key: &str, control: &str) -> Option<Home> {
    if let Some((d, f)) = events::xplane_var(key) {
        return Some(Home::XPlane(d, f));
    }
    // LGT-004: unlike most XMLVAR_ locals (Asobo/MSFS-internal, never read
    // by FBW), these ARE real inputs FlyByWire's own TypeScript reads
    // directly by this exact name: the overhead NO SMOKING/EMER EXIT signs
    // (fbw-a380x/src/systems/systems-host/CpiomC/FlightWarningSystem/
    // FwsNormalChecklists.ts:607, FwsCore.ts:4466: `SimVar.GetSimVarValue(
    // 'L:XMLVAR_SWITCH_OVHD_INTLT_*_Position', ...)`), and the pedestal
    // engine mode selector (`KNOB_ENGINES_MODE`, pedestal.xml,
    // A32NX_ENGINE_MODE_SELECTOR_TEMPLATE): FwsSystemDisplayLogic.ts:265
    // `checkEnginePage` reads `L:XMLVAR_ENG_MODE_SEL` by this exact name to
    // auto-select the SD ENG page on start/crank, alongside the same
    // control's K:TURBINE_IGNITION_SWITCH_SET# writes (already Home::Sys
    // via the generic "A:" branch below). So all three publish as fbw/<name>
    // (Home::Sys) like any other simulator variable instead of staying in
    // the click's own local Lua state.
    if matches!(
        key,
        "L:XMLVAR_SWITCH_OVHD_INTLT_EMEREXIT_Position" | "L:XMLVAR_SWITCH_OVHD_INTLT_NOSMOKING_Position" | "L:XMLVAR_ENG_MODE_SEL"
    ) {
        let (_, name) = key.split_once(':').unwrap();
        return Some(Home::Sys(dataref(name)));
    }
    // FBW_Airbus_FCU_Altitude_Knob_SubTemplate's INCREMENT parameter reads
    // this MSFS-local-only XMLVAR (never written anywhere in the A380X's own
    // behaviour XML, so it would otherwise stay 0 and turn every altitude
    // step into a divide/modulo by zero): routed at the real FBW increment
    // selector's dataref instead (feet, 100 or 1000; prim.rs
    // update_fcu_afs_lvars publishes it from A32NX_FCU_ALT_INCREMENT_1000).
    if key == "L:XMLVAR_Autopilot_Altitude_Increment" {
        return Some(Home::Sys(dataref("XP_FCU_ALT_INCREMENT_FT")));
    }
    // The plugin's oxygen and fuel-jettison models read these two as new,
    // MSFS-simvar-style names (oxygen.rs `crew_mask_on` reads "OXYGEN CREW
    // MASK ON", fuel.rs `jettison.switch` reads "FUEL JETTISON SWITCH" --
    // see those files' doc comments) rather than an FBW L:var, because
    // nothing in the stock A380X ever gave the crew mask or the jettison
    // switch its own FBW dataref. Route the overhead's crew oxygen supply
    // push button (PUSH_OVHD_OXYGEN_CREW, A380_Cockpit_Behavior.xml) and
    // fuel jettison ACTIVE push button (PUSH_OVHD_JETTISON_ACTIVE) straight
    // to those names instead of their own L:var, so pressing them feeds the
    // systems that read the vars, and their own lamp/look state (driven off
    // the same key) stays in sync for free.
    if key == "L:PUSH_OVHD_OXYGEN_CREW" {
        return Some(Home::Sys(dataref("OXYGEN CREW MASK ON")));
    }
    if key == "L:A380X_OVHD_FUEL_JETTISON_ACTIVE_PB_IS_ON" {
        return Some(Home::Sys(dataref("FUEL JETTISON SWITCH")));
    }
    let (kind, name) = key.split_once(':')?;
    match kind {
        "A" => Some(Home::Sys(dataref(name))),
        "L" if name.starts_with(COVER_VAR) => Some(Home::Sys(cover_dataref(&name[COVER_VAR.len()..]))),
        "L" if name.to_ascii_uppercase().starts_with("XMLVAR_") => Some(Home::Local(key.to_string())),
        "L" => Some(Home::Sys(dataref(name))),
        "O" => Some(Home::Local(format!("{control}/{key}"))),
        // Input events: their value, shared by every component using them.
        "B" => Some(Home::Sys(format!("fbw/cockpit/ie/{}", sanitize_id(name)))),
        _ => None,
    }
}

/// The dataref a simulator or systems variable (keyed as [`rpn::key`]) lives in.
pub fn sys_dataref(key: &str) -> Option<String> {
    match home(key, "x") {
        Some(Home::Sys(d)) => Some(d),
        _ => None,
    }
}

fn step_var(s: &Step, args: &[f64]) -> String {
    match s.index {
        Some(i) => s.var.replace("{}", &format!("{}", args.get(i).copied().unwrap_or(0.0).round() as i64)),
        None => s.var.clone(),
    }
}

/// Input events the cockpit's covers use: `<id>_Toggle` flips the input
/// event's value, `<id>_Set` sets it (MSFS's COMMON cover preset,
/// PushButton.xml ASOBO_GT_Push_Button_Airliner_SubTemplate: the cover's
/// click is `(>B:<id>_Toggle)` and its state `(B:<id>, Bool)`).
fn b_event(name: &str) -> Option<(&str, bool)> {
    name.strip_suffix("_Toggle").map(|b| (b, true)).or_else(|| name.strip_suffix("_Set").map(|b| (b, false)))
}

pub struct KEvents;
impl Events for KEvents {
    fn fire(&self, event: &str, args: &[f64], env: &mut Env) -> bool {
        let Some(action) = events::k_event(event) else { return false };
        match action {
            KAction::Vars(steps) => {
                for s in &steps {
                    let k = rpn::key(s.kind, &step_var(s, args));
                    let v = match s.val {
                        Val::Const(c) => c,
                        Val::Arg(i) => args.get(i).copied().unwrap_or(0.0),
                        Val::Percent(i) => args.get(i).copied().unwrap_or(0.0) / 100.0,
                        Val::Not => {
                            if env.get(&k) != 0.0 {
                                0.0
                            } else {
                                1.0
                            }
                        }
                    };
                    env.set(&k, v);
                }
            }
            KAction::Command(c) => {
                if let Some(n) = c.name(args) {
                    env.run.commands.push(n);
                }
            }
            KAction::Dataref { index, drefs, value, scale } => {
                let i = args.get(index).copied().unwrap_or(0.0).round() as i64;
                if let Some((_, d)) = drefs.iter().find(|(k, _)| *k == i) {
                    let _ = (value, scale);
                    env.run.commands.push(format!("dataref {d}"));
                }
            }
            KAction::Nothing(_) => {}
        }
        true
    }

    fn fire_h(&self, event: &str, env: &mut Env) -> bool {
        match events::h_event(event).and_then(|c| c.name(&[])) {
            Some(n) => {
                env.run.commands.push(n);
                true
            }
            None => false,
        }
    }

    fn fire_b(&self, event: &str, arg: Option<f64>, env: &mut Env) -> bool {
        let Some((base, toggle)) = b_event(event) else { return false };
        let k = format!("B:{base}");
        let v = if toggle { if env.get(&k) != 0.0 { 0.0 } else { 1.0 } } else { arg.unwrap_or(0.0) };
        env.set(&k, v);
        true
    }
}

/// How a control is operated in MSFS, with its code.
#[derive(Clone, Debug)]
pub enum Action {
    /// Click (and release) code.
    Click { press: String, release: Option<String> },
    /// N positions, each with the code run on reaching it; `current` gives
    /// the position now. Momentary switches spring back to `rest` on release.
    States { codes: Vec<String>, current: String, knob: bool, horizontal: bool, momentary: Option<usize> },
    /// Turned both ways (wheel or drag), with an optional push.
    Rotary { cw: String, ccw: String },
    /// A lever dragged through its range.
    Lever,
}

/// How a control looks: its clip position (0 to `length`) from code, or a
/// momentary press.
#[derive(Clone, Debug)]
pub enum Look {
    Code { code: String, length: f64 },
    Press,
    None,
}

#[derive(Clone, Debug)]
pub struct Control {
    /// The MSFS animation it moves (the glTF clip name).
    pub anim: String,
    pub node: String,
    pub template: String,
    /// The `UseTemplate` chain that reached `template` (`expand::Leaf::chain`,
    /// outermost first): which package templates to blame when a call deep
    /// in the chain never passed a parameter down.
    pub chain: Vec<String>,
    pub action: Result<Action, String>,
    pub look: Look,
}

fn get<'a>(l: &'a Leaf, k: &str) -> Option<&'a str> {
    l.get(k).map(str::trim)
}

fn nonempty<'a>(l: &'a Leaf, k: &str) -> Option<&'a str> {
    get(l, k).filter(|v| !v.is_empty())
}

fn num(l: &Leaf, k: &str) -> Option<f64> {
    get(l, k).and_then(|v| v.parse().ok())
}

fn truthy(l: &Leaf, k: &str) -> bool {
    get(l, k).is_some_and(|v| !(v.is_empty() || v.eq_ignore_ascii_case("false") || v == "0"))
}

/// The controls a leaf template call makes: its own, and the cover MSFS's
/// push button template makes for it.
fn controls_of(l: &Leaf) -> Vec<Control> {
    let mut out: Vec<Control> = control_of(l).into_iter().collect();
    // (MSFS's input-event buttons make theirs through the same template.)
    if matches!(l.template.as_str(), "ASOBO_GT_Push_Button_Airliner" | "ASOBO_Interaction_Base_Template") && !l.params.contains_key("DUMMY_BUTTON") {
        if let (Some(cover), Some(c)) = (nonempty(l, "COVER_NODE_ID"), out.first()) {
            // PushButton.xml ASOBO_GT_Push_Button_Airliner_SubTemplate: the
            // cover component on COVER_NODE_ID toggles the cover input event
            // and shows its value.
            let ie = asobo_cover_event(l);
            let control = Control {
                anim: nonempty(l, "COVER_ANIM_NAME").unwrap_or(cover).to_string(),
                node: cover.to_string(),
                template: format!("{} (cover)", c.template),
                chain: c.chain.clone(),
                action: Ok(Action::Click { press: format!("(>B:{ie}_Toggle)"), release: None }),
                look: Look::Code { code: format!("(B:{ie}, Bool) 100 *"), length: 100.0 },
            };
            out.push(control);
        }
    }
    // A push-pull knob's push and pull (Knob.xml ASOBO_GT_Knob_Infinite_PushPull)
    // move ANIM_NAME_PUSHPULL, a node above the turning one, and are clicked on
    // the same part: X-Plane gives a part one manipulator, which the turn takes.
    if l.template == "ASOBO_GT_Knob_Infinite_PushPull" {
        if let (Some(anim), Some(c)) = (nonempty(l, "ANIM_NAME_PUSHPULL"), out.first()) {
            let codes: Vec<String> = ["PUSH_CODE", "PULL_CODE"].iter().filter_map(|k| nonempty(l, k)).map(|v| v.split_whitespace().collect::<Vec<_>>().join(" ")).collect();
            out.push(Control {
                anim: anim.to_string(),
                node: c.node.clone(),
                template: format!("{} (push/pull)", c.template),
                chain: c.chain.clone(),
                action: Err(format!("push/pull on the part the turn's manipulator takes (one manipulator per part): {}", codes.join(" / "))),
                look: Look::None,
            });
        }
    }
    out
}

/// The input event of an MSFS push button's cover: `<source>_<name>`, the
/// source INPUT_EVENT_ID or COMMON and the name `<NODE_ID>_Cover` unless
/// given (PushButton.xml ASOBO_GT_Push_Button_Airliner_SubTemplate).
fn asobo_cover_event(l: &Leaf) -> String {
    // Input-event buttons pass USE_INPUT_EVENT_ID as INPUT_EVENT_ID to the
    // airliner push button (Inputs/Templates.xml:590-594).
    let ie_id = nonempty(l, "INPUT_EVENT_ID").or_else(|| (l.template == "ASOBO_Interaction_Base_Template").then(|| nonempty(l, "USE_INPUT_EVENT_ID")).flatten());
    let src = nonempty(l, "COVER_IE_ID_SOURCE").or(ie_id).unwrap_or("COMMON");
    let name = nonempty(l, "COVER_IE_NAME").map(str::to_string).unwrap_or_else(|| format!("{}_Cover", get(l, "NODE_ID").unwrap_or("")));
    format!("{src}_{name}")
}

/// A guarded button's press code, run only while its cover is open: MSFS's
/// cover (COVER_NODE_ID) gates the click itself (`(B:<cover>, Bool) if{ ..
/// }`, PushButton.xml); FlyByWire's FBW_Covered_Push_Toggle
/// (A32NX_Interior_Generics.xml) puts the cover (LOCK_NODE_ID, variable
/// `<TOGGLE_SIMVAR>_LOCK`) over the button, whose geometry takes the click
/// while closed.
fn guarded(l: &Leaf, press: String) -> String {
    if l.template != "ASOBO_GT_Push_Button_Airliner" {
        return press;
    }
    if nonempty(l, "COVER_NODE_ID").is_some() {
        return format!("(B:{}, Bool) if{{ {press} }}", asobo_cover_event(l));
    }
    match (nonempty(l, "LOCK_NODE_ID"), nonempty(l, "TOGGLE_SIMVAR").filter(|v| !v.contains('#'))) {
        (Some(_), Some(var)) => format!("({var}_LOCK, Bool) if{{ {press} }}"),
        (Some(lock), None) if !lock.contains('#') => format!("(L:{COVER_VAR}{}, Bool) if{{ {press} }}", sanitize_id(lock)),
        _ => press,
    }
}

/// The control a leaf template call makes, if it makes one.
fn control_of(l: &Leaf) -> Option<Control> {
    let t = l.template.as_str();
    let anim = nonempty(l, "ANIM_NAME").or_else(|| nonempty(l, "NODE_ID")).or(l.node.as_deref())?.to_string();
    // Templates that make their own component sit on NODE_ID; interactions
    // sit on the enclosing component's node.
    let own_component = matches!(t, "ASOBO_GT_Push_Button_Airliner" | "ASOBO_GT_Switch_Dummy")
        || (t == "ASOBO_Interaction_Base_Template" && !get(l, "CREATE_COMPONENT").is_some_and(|v| v.eq_ignore_ascii_case("false")));
    let node = if own_component { nonempty(l, "NODE_ID").map(str::to_string).or_else(|| l.node.clone()) } else { l.node.clone().or_else(|| nonempty(l, "NODE_ID").map(str::to_string)) }
        .unwrap_or_else(|| anim.clone());
    let mk = |action: Result<Action, String>, look: Look| Control { anim: anim.clone(), node: node.clone(), template: t.to_string(), chain: l.chain.clone(), action, look };
    let code = |k: &str| nonempty(l, k).map(str::to_string);
    let anim_code = || code("ANIM_CODE").map(|c| Look::Code { code: c, length: num(l, "ANIM_LENGTH").unwrap_or(100.0) }).unwrap_or(Look::None);
    let click = |press: Option<String>, release: Option<String>| match press.or_else(|| release.clone()) {
        Some(p) => Ok(Action::Click { press: guarded(l, p), release }),
        None => Err("no click code".to_string()),
    };
    let pos_var = || format!("{}:{}", get(l, "SWITCH_POSITION_TYPE").unwrap_or("O"), get(l, "SWITCH_POSITION_VAR").unwrap_or("SwitchState"));
    let horizontal = get(l, "SWITCH_DIRECTION").is_some_and(|d| d.eq_ignore_ascii_case("Horizontal"));
    match t {
        "ASOBO_GT_Push_Button_Airliner" => {
            if l.params.contains_key("DUMMY_BUTTON") {
                return None;
            }
            // Some FBW nodes pair LEFT_SINGLE_CODE with a LEFT_LEAVE_CODE
            // that resets a momentary "pressed" simvar on mouse-up (mip.xml
            // PUSH_RTO_ARM: LEFT_SINGLE_CODE sets
            // A32NX_OVHD_AUTOBRK_RTO_ARM_IS_PRESSED to 1, LEFT_LEAVE_CODE
            // resets it to 0). a380_systems' PressSingleSignalButton only
            // fires on that variable's 0->1 edge (overhead/mod.rs
            // read()), so dropping the leave code here left such a button
            // stuck at 1 after its first press -- every later press was a
            // no-op. Nodes with no LEFT_LEAVE_CODE param are unaffected:
            // `code()` still returns None for them.
            match code("DOWN_STATE_CODE") {
                Some(d) => Some(mk(click(code("LEFT_SINGLE_CODE"), code("LEFT_LEAVE_CODE")), Look::Code { code: format!("{d} 100 *"), length: 100.0 })),
                None => Some(mk(click(code("LEFT_SINGLE_CODE"), code("LEFT_LEAVE_CODE")), Look::Press)),
            }
        }
        "ASOBO_GT_Push_Button" | "ASOBO_GT_Push_Button_Held" => {
            let release = l.params.contains_key("LEFT_LEAVE_CODE").then(|| code("LEFT_LEAVE_CODE")).flatten();
            let look = match code("OVERRIDE_ANIM_CODE") {
                Some(c) => Look::Code { code: c, length: num(l, "ANIM_LENGTH").unwrap_or(100.0) },
                None => Look::Press,
            };
            Some(mk(click(code("LEFT_SINGLE_CODE"), release), look))
        }
        "ASOBO_GT_Switch_Code" => Some(mk(click(code("LEFT_SINGLE_CODE"), None), anim_code())),
        "ASOBO_GT_Interaction_LeftSingle_Code" => Some(mk(click(code("LEFT_SINGLE_CODE"), None), anim_code())),
        "ASOBO_GT_Interaction_LeftSingle_Leave_Code" => Some(mk(click(code("LEFT_SINGLE_CODE"), code("LEFT_LEAVE_CODE")), anim_code())),
        "ASOBO_GT_Switch_Dummy" => {
            let states = num(l, "NUM_STATES").unwrap_or(2.0) as usize;
            let v = pos_var();
            if states == 2 {
                let mut code = get(l, "LEFT_SINGLE_CODE").unwrap_or("").to_string();
                // FBW_Covered_Push_Toggle without TOGGLE_SIMVAR: its cover
                // (the LOCK node) has no variable; it gets the cockpit's own.
                if code.contains("#TOGGLE_SIMVAR#_LOCK") {
                    if let Some(lock) = nonempty(l, "LOCK_NODE_ID").or_else(|| nonempty(l, "NODE_ID")).filter(|n| !n.contains('#')) {
                        code = code.replace("#TOGGLE_SIMVAR#_LOCK", &format!("L:{COVER_VAR}{}", sanitize_id(lock)));
                    }
                }
                let press = format!("({v}) ! (>{v}) {code}");
                Some(mk(Ok(Action::Click { press, release: None }), Look::Code { code: format!("({v}) 100 *"), length: 100.0 }))
            } else {
                let codes = (0..states).map(|i| format!("{i} (>{v})")).collect();
                Some(mk(Ok(Action::States { codes, current: format!("({v})"), knob: false, horizontal, momentary: None }), Look::Code { code: format!("({v}) 100 *"), length: ((states - 1) * 100) as f64 }))
            }
        }
        _ if t.starts_with("ASOBO_GT_Switch_") && t.ends_with("States") => {
            let n = t.trim_start_matches("ASOBO_GT_Switch_").trim_end_matches("States");
            let states = n.parse::<usize>().ok().or_else(|| num(l, "NUM_STATES").map(|x| x as usize)).unwrap_or(2);
            let v = pos_var();
            let codes: Vec<String> = (0..states).map(|i| format!("{i} (>{v}) {}", get(l, &format!("CODE_POS_{i}")).unwrap_or(""))).collect();
            let tests: Vec<Option<&str>> = (0..states).map(|i| nonempty(l, &format!("STATE{i}_TEST"))).collect();
            // The position now: the first state whose test holds (MSFS keeps
            // the switch in step with them), else the position variable.
            let current = if tests.iter().all(Option::is_some) && tests.iter().any(|t| t.is_some_and(|t| t != "1")) {
                let mut c = format!("({v})");
                for (i, t) in tests.iter().enumerate().rev() {
                    c = format!("{} if{{ {i} }} els{{ {c} }}", t.unwrap_or("0"));
                }
                c
            } else {
                format!("({v})")
            };
            let momentary = truthy(l, "MOMENTARY_SWITCH").then_some(if states == 3 { 1 } else { 0 });
            let look = Look::Code { code: format!("{current} 100 *"), length: ((states.max(2) - 1) * 100) as f64 };
            let knob = anim.to_ascii_uppercase().contains("KNOB");
            Some(mk(Ok(Action::States { codes, current, knob, horizontal, momentary }), look))
        }
        "ASOBO_GT_Interaction_WheelAndContinuousLeft" | "ASOBO_GT_Knob_Infinite" | "ASOBO_GT_Knob_Infinite_Push" | "ASOBO_GT_Knob_Infinite_PushPull" => {
            // The wheel's step is the one a click should take.
            let cw = code("WHEEL_CLOCKWISE_CODE").or_else(|| code("CLOCKWISE_CODE"));
            let ccw = code("WHEEL_ANTICLOCKWISE_CODE").or_else(|| code("ANTICLOCKWISE_CODE"));
            let action = match (cw, ccw) {
                (Some(cw), Some(ccw)) => Ok(Action::Rotary { cw, ccw }),
                _ => Err("knob without turn code".to_string()),
            };
            let look = match t {
                "ASOBO_GT_Interaction_WheelAndContinuousLeft" => Look::None,
                _ => code("OVERRIDE_ANIM_CODE").map(|c| Look::Code { code: c, length: num(l, "ANIM_LENGTH").unwrap_or(100.0) }).unwrap_or(Look::None),
            };
            let mut c = mk(action, look);
            // Knob.xml ASOBO_GT_Knob_Infinite_Push(Pull): the turning clip is
            // ANIM_NAME_KNOB (ANIM_NAME is a leftover default).
            if let Some(k) = nonempty(l, "ANIM_NAME_KNOB").filter(|_| t != "ASOBO_GT_Knob_Infinite") {
                c.anim = k.to_string();
            }
            Some(c)
        }
        "ASOBO_Interaction_Base_Template" | "ASOBO_Interaction_Push_Event_Base_Template" => {
            let kind = get(l, "INTERACTION_TYPE").unwrap_or("");
            let source = get(l, "INPUT_EVENT_ID_SOURCE").unwrap_or("");
            let ie = get(l, "IE_NAME").unwrap_or("");
            let pos = format!("O:{source}_{ie}_Position");
            let set = code("SET_STATE_EXTERNAL");
            // Input event SET: the new position on the stack and in the
            // preset's position variable.
            let set_to = |v: usize| -> Option<String> {
                code(&format!("SET_STATE_{v}")).or_else(|| set.clone()).map(|s| {
                    // p0 is the input event's parameter: the new position.
                    let s = s.split_whitespace().map(|w| if w == "p0" { v.to_string() } else { w.to_string() }).collect::<Vec<_>>().join(" ");
                    format!("{v} {v} (>{pos}) {s}")
                })
            };
            let get_state = code("GET_STATE_EXTERNAL");
            // GET_STATE_EXTERNAL leaves the state in register 0 (`sp0`) or on
            // the stack.
            let current = get_state
                .clone()
                .map(|g| if g.split_whitespace().any(|w| w.eq_ignore_ascii_case("sp0")) { format!("{g} l0") } else { g })
                .unwrap_or_else(|| format!("({pos})"));
            // SET_STATE_EXTERNAL with p0, the event's value, taken from register 9.
            let set_from_r9 = set.clone().map(|s| s.split_whitespace().map(|w| if w == "p0" { "l9" } else { w }).collect::<Vec<_>>().join(" "));
            let states = num(l, "NUM_STATES").map(|x| x as usize).unwrap_or(2);
            let anim = match kind {
                "Knob" => nonempty(l, "ANIM_NAME_KNOB").or_else(|| nonempty(l, "ANIM_NAME")).unwrap_or(&anim).to_string(),
                _ => nonempty(l, "ANIM_NAME").or_else(|| nonempty(l, "ANIM_NAME_SWITCH")).unwrap_or(&anim).to_string(),
            };
            let mut c = mk(Err(String::new()), Look::None);
            c.anim = anim;
            c.template = format!("{t} ({kind})");
            match kind {
                "Push" if get(l, "EXTRA_OPTION").is_some_and(|o| o.contains("Held")) => {
                    c.action = match (set_to(1), set_to(0)) {
                        (Some(p), Some(r)) => Ok(Action::Click { press: p, release: Some(r) }),
                        _ => Err("input event without SET_STATE_EXTERNAL".into()),
                    };
                    c.look = Look::Press;
                }
                "Push" | "Switch" => {
                    let (s0, s1) = (set_to(0), set_to(1));
                    c.action = match (s0, s1, &get_state) {
                        (Some(s0), Some(s1), Some(_)) => Ok(Action::States { codes: vec![s0, s1], current: current.clone(), knob: false, horizontal, momentary: None }),
                        (_, Some(s1), None) => Ok(Action::Click { press: s1, release: None }),
                        _ => Err("input event without SET_STATE_EXTERNAL".into()),
                    };
                    c.look = if get_state.is_some() { Look::Code { code: format!("{current} 100 *"), length: 100.0 } } else { Look::Press };
                }
                "Knob" if get(l, "KNOB_TYPE") == Some("X_STATES") => {
                    let codes: Option<Vec<String>> = (0..states).map(set_to).collect();
                    c.action = match codes {
                        Some(codes) => Ok(Action::States { codes, current: current.clone(), knob: true, horizontal: false, momentary: None }),
                        None => Err("input event without SET_STATE_EXTERNAL".into()),
                    };
                    c.look = Look::Code { code: format!("{current} 100 *"), length: ((states.max(2) - 1) * 100) as f64 };
                }
                "Knob" if get_state.is_some() && set.is_some() => {
                    // A FLOAT input event (Asobo Inputs/Generic.xml ASOBO_GIE
                    // FLOAT): each wheel or click step is `1 (>B:<ie>_Inc)`
                    // (Inputs/Templates.xml CLOCKWISE_CODE), INC_CODE sets the
                    // state plus 1 (`GET p0 + (>B:<ie>_Set)`) and SET_CODE runs
                    // SET_STATE_EXTERNAL with that value; the state reads 0..100.
                    let set9 = set_from_r9.clone().unwrap_or_default();
                    c.action = Ok(Action::Rotary { cw: format!("{current} 1 + sp9 {set9}"), ccw: format!("{current} 1 - sp9 {set9}") });
                    c.look = Look::Code { code: current.clone(), length: 100.0 };
                }
                "" if t == "ASOBO_Interaction_Push_Event_Base_Template" && get_state.is_some() && set.is_some() => {
                    // A two-state push input event: each push sets the other
                    // state (Inputs/Generic.xml INTEGER, NUM_STATES 2, its
                    // Toggle binding `0 1 (O:<ie>_Position) 1 == ?`).
                    let set9 = set_from_r9.clone().unwrap_or_default();
                    c.action = Ok(Action::Click { press: format!("{current} 0 == sp9 {set9}"), release: None });
                    c.look = Look::Code { code: format!("{current} 100 *"), length: 100.0 };
                }
                _ => c.action = Err(format!("input event interaction {kind} {} (continuous knob or lever) not translated", get(l, "KNOB_TYPE").unwrap_or(""))),
            }
            Some(c)
        }
        "ASOBO_GT_MouseRect" => Some(mk(Err("raw mouse rectangle callback (event-dispatch code) not translated".into()), Look::None)),
        // The altitude-increment (100/1000 ft) selector: its own RPN isn't
        // reachable (a pure Asobo base-game template, undefined in FBW's own
        // behaviour XML), but its real function is well documented and FBW's
        // own compiled interface already implements it 1:1 (AP_ALT_HOLD /
        // A32NX.FCU_ALT_INCREMENT_TOGGLE flip A32NX_FCU_ALT_INCREMENT_1000,
        // SimConnectInterface.cpp:2402-2408) — bound directly to that real
        // dataref rather than left unresolved.
        "ASOBO_AUTOPILOT_Switch_Altitude_Increment_Template" => {
            let codes = vec!["0 (>L:A32NX_FCU_ALT_INCREMENT_1000)".to_string(), "1 (>L:A32NX_FCU_ALT_INCREMENT_1000)".to_string()];
            Some(mk(
                Ok(Action::States { codes, current: "(L:A32NX_FCU_ALT_INCREMENT_1000)".to_string(), knob: false, horizontal: false, momentary: None }),
                Look::Code { code: "(L:A32NX_FCU_ALT_INCREMENT_1000) 100 *".to_string(), length: 100.0 },
            ))
        }
        // Overhead EXT LT switches: these ASOBO_LIGHTING_Switch_* templates
        // are pure Asobo base-game interaction logic (no RPN of their own to
        // read, same as the altitude-increment case above), but unlike most
        // other rejected Asobo templates below they are NOT MSFS-only --
        // fbw-xp-systems' own lights.rs reads exactly the X-Plane switch
        // datarefs the converted model's exterior lights are keyed to
        // (`sim/cockpit2/switches/*`, module doc: "The converted model's
        // exterior lights are X-Plane's own named OBJ8 light types ... which
        // X-Plane's own engine visually drives straight off those same
        // switch datarefs"), gated by GatedSwitch so a click still works
        // while its bus is dead. Bound to those real datarefs the same way
        // the altitude-increment selector is bound to its real L: var, via
        // the invented keys events::xplane_var() resolves (see there).
        // AUTO/AUTO-lit positions (STROBE, LOGO: A380_Cockpit_Behavior.xml
        // TYPE=Auto) collapse to ON: the plugin's GatedSwitch has no beacon-
        // linked or altitude-linked auto logic to drive a true middle
        // detent, so ON is the safer, visible default (matches the real
        // switch's usual in-flight position) rather than leaving strobes/
        // logo dark. The 3-position switches keep their own display state in
        // an XMLVAR_ local (Home::Local) so the clip shows the real detent
        // (OFF vs AUTO vs ON) even though the system dataref itself only
        // has two values.
        "ASOBO_LIGHTING_Switch_Light_Strobe_Template" if get(l, "NODE_ID") == Some("SWITCH_OVHD_EXTLT_STROBE") => {
            let pos = "L:XMLVAR_SWITCH_OVHD_EXTLT_STROBE_Position";
            let codes = vec![
                format!("0 (>{pos}) 1 (>A:LIGHT STROBE)"),
                format!("1 (>{pos}) 1 (>A:LIGHT STROBE)"),
                format!("2 (>{pos}) 0 (>A:LIGHT STROBE)"),
            ];
            Some(mk(
                Ok(Action::States { codes, current: format!("({pos})"), knob: false, horizontal: false, momentary: None }),
                Look::Code { code: format!("({pos}) 100 *"), length: 200.0 },
            ))
        }
        "ASOBO_LIGHTING_Switch_Light_Taxi_Template" if get(l, "NODE_ID") == Some("SWITCH_OVHD_EXTLT_RWY") => {
            // RWY TURN OFF: TYPE OnOff_TwoSimvars, a plain 2-position switch
            // (lights.rs's "Taxi (turn-off)" group, CIRCUIT_LIGHT_TAXI:2/:3).
            let codes = vec!["0 (>A:LIGHT TAXI:2)".to_string(), "1 (>A:LIGHT TAXI:2)".to_string()];
            Some(mk(
                Ok(Action::States { codes, current: "(A:LIGHT TAXI:2)".to_string(), knob: false, horizontal: false, momentary: None }),
                Look::Code { code: "(A:LIGHT TAXI:2) 100 *".to_string(), length: 100.0 },
            ))
        }
        "ASOBO_LIGHTING_Switch_Light_Navigation_Template" if get(l, "NODE_ID") == Some("SWITCH_OVHD_EXTLT_NAVLOGO") => {
            // NAV LT: TYPE OnOff, a plain 2-position switch.
            let codes = vec!["0 (>A:LIGHT NAV)".to_string(), "1 (>A:LIGHT NAV)".to_string()];
            Some(mk(
                Ok(Action::States { codes, current: "(A:LIGHT NAV)".to_string(), knob: false, horizontal: false, momentary: None }),
                Look::Code { code: "(A:LIGHT NAV) 100 *".to_string(), length: 100.0 },
            ))
        }
        "ASOBO_LIGHTING_Switch_Light_Logo_Template" if get(l, "NODE_ID") == Some("SWITCH_OVHD_EXTLT_LOGO") => {
            let pos = "L:XMLVAR_SWITCH_OVHD_EXTLT_LOGO_Position";
            let codes = vec![
                format!("0 (>{pos}) 1 (>A:LIGHT LOGO)"),
                format!("1 (>{pos}) 1 (>A:LIGHT LOGO)"),
                format!("2 (>{pos}) 0 (>A:LIGHT LOGO)"),
            ];
            Some(mk(
                Ok(Action::States { codes, current: format!("({pos})"), knob: false, horizontal: false, momentary: None }),
                Look::Code { code: format!("({pos}) 100 *"), length: 200.0 },
            ))
        }
        // EXT LT NOSE TAXI/TO: FBW's own FBW_Anim_Interactions wrapper
        // (A380_Cockpit_Behavior.xml ~2144) expands ANIM_TEMPLATE=
        // ASOBO_LIGHTING_Switch_Light_Landing_Template onto NODE_ID
        // SWITCH_OVHD_EXTLT_NOSE, distinct from the real landing-light
        // switch (SWITCH_OVHD_EXTLT_LANDL, FBW's own
        // FBW_Airbus_LIGHTING_Switch_Light_Landing_Template, already
        // resolved elsewhere). TYPE TwoSimvars, 3 positions TO/TAXI/OFF
        // (ANIMTIP_0/1/2); the XML's own SET_SIMVAR_2 lets TAXI join an
        // already-on TO light instead of replacing it, which this
        // simplified binding does not reproduce (a disclosed limitation,
        // not modelled by fbw-xp-systems lights.rs either, which just gates
        // two independent GatedSwitch groups).
        "ASOBO_LIGHTING_Switch_Light_Landing_Template" if get(l, "NODE_ID") == Some("SWITCH_OVHD_EXTLT_NOSE") => {
            let pos = "L:XMLVAR_SWITCH_OVHD_EXTLT_NOSE_Position";
            let codes = vec![
                format!("0 (>{pos}) 1 (>A:LIGHT LANDING:1) 0 (>A:LIGHT TAXI:1)"),
                format!("1 (>{pos}) 0 (>A:LIGHT LANDING:1) 1 (>A:LIGHT TAXI:1)"),
                format!("2 (>{pos}) 0 (>A:LIGHT LANDING:1) 0 (>A:LIGHT TAXI:1)"),
            ];
            Some(mk(
                Ok(Action::States { codes, current: format!("({pos})"), knob: false, horizontal: false, momentary: None }),
                Look::Code { code: format!("({pos}) 100 *"), length: 200.0 },
            ))
        }
        _ if t.contains("Dragging") || t.contains("_Lever_") => Some(mk(Ok(Action::Lever), Look::None)),
        _ if t.starts_with("ASOBO_LIGHTING_Switch") || t.starts_with("ASOBO_LIGHTING_Knob") || t.starts_with("ASOBO_AUTOPILOT_") || t.starts_with("ASOBO_HANDLING_") => {
            Some(mk(Err(format!("Asobo template {t}: acts on MSFS's own systems, not an FBW variable")), Look::None))
        }
        _ => None,
    }
}

/// Anim-only calls, for a control's looks when its interaction gives none.
fn look_of(l: &Leaf) -> Option<(String, Look)> {
    match l.template.as_str() {
        "ASOBO_GT_Anim_Code" | "ASOBO_GT_Anim" => {
            let anim = nonempty(l, "ANIM_NAME")?.to_string();
            let code = nonempty(l, "ANIM_CODE")?.to_string();
            Some((anim, Look::Code { code, length: num(l, "ANIM_LENGTH").unwrap_or(100.0) }))
        }
        _ => None,
    }
}

// ---------------------------------------------------------------------------
// Lua translation

/// RPN code as Lua statements over a stack `s` and registers `r`, reading
/// and writing variables through `rd`/`wr` (datarefs) and `LV` (local state).
pub fn to_lua(code: &str, control: &str) -> Result<String, String> {
    to_lua_with(code, control, &HashMap::new())
}

/// Lua for a key event: its variable writes, command or dataref.
fn k_event_lua(ev: &str, nargs_given: usize) -> Result<String, String> {
    let Some(action) = events::k_event(ev) else {
        return Err(if ev.contains('.') {
            format!("MSFS event K:{ev}: a FlyByWire module event the systems plugin registers no fbw/event command for")
        } else {
            format!("MSFS event K:{ev} has no FBW variable")
        });
    };
    let args = |n: usize| (0..n.max(nargs_given.min(n))).map(|a| format!(" local a{a} = Q(s)")).collect::<String>();
    Ok(match action {
        KAction::Vars(steps) => {
            let nargs = steps
                .iter()
                .map(|s| match (s.index, &s.val) {
                    (Some(i), Val::Arg(j) | Val::Percent(j)) => i.max(*j) + 1,
                    (Some(i), _) => i + 1,
                    (None, Val::Arg(j) | Val::Percent(j)) => j + 1,
                    _ => 0,
                })
                .max()
                .unwrap_or(0);
            let mut l = format!("do{}", args(nargs));
            for (n, st) in steps.iter().enumerate() {
                let name = |var: &str| if st.kind == "A" { dataref(var) } else { dataref(var) };
                let dr = match st.index {
                    Some(ix) => {
                        let (pre, post) = st.var.split_once("{}").unwrap_or((&st.var, ""));
                        format!("{:?} .. string.format(\"%d\", a{ix}) .. {:?}", name(pre), dataref_tail(post))
                    }
                    None => format!("{:?}", name(&st.var)),
                };
                let v = match st.val {
                    Val::Const(c) => lua_num(c),
                    Val::Arg(j) => format!("a{j}"),
                    Val::Percent(j) => format!("(a{j} / 100)"),
                    Val::Not => format!("B(rd({dr}) == 0)"),
                };
                l.push_str(&format!(" local v{n} = {v} wr({dr}, v{n})"));
            }
            l.push_str(" end");
            l
        }
        KAction::Command(c) => format!("do{} CMD({}) end", args(c.args()), c.lua()),
        KAction::Dataref { index, drefs, value, scale } => {
            let t: Vec<String> = drefs.iter().map(|(k, d)| format!("[{k}] = {d:?}")).collect();
            format!(
                "do{} local d = ({{{}}})[math.floor(a{index} + 0.5)] if d ~= nil then XSET(d, a{value} * {}) end end",
                args(index.max(value) + 1),
                t.join(", "),
                scale
            )
        }
        KAction::Nothing(why) => format!("-- (>K:{ev}): {why}"),
    })
}

/// [`to_lua`], with simulator variables that hold a constant (`consts`,
/// keyed as [`rpn::key`]) read as that constant.
pub fn to_lua_with(code: &str, control: &str, consts: &HashMap<String, f64>) -> Result<String, String> {
    // FBW_A380X_BacklightIndicator_Button_Template (generic/buttons.xml) has
    // no DefaultTemplateParameters entry for INDICATOR_POWERED (only an
    // OverrideTemplateParameters that sets it to 0 when INOP is given), so
    // every caller that does not override it (the SURV panel's 8 buttons,
    // EFIS CS/FO BLANK and TAXI, ...) leaves the literal placeholder in its
    // LEFT_SINGLE_CODE/EMISSIVE_CODE. The template's own INOP handling shows
    // its intent (powered unless explicitly marked INOP), so the missing
    // case defaults to powered (1) rather than leaving the whole button dark
    // and unclickable.
    // Same template, same file: INDICATOR_CODE also has no default and
    // PUSH_EFIS_CS/FO_BLANK and _TAXI (both explicitly INOP, TOOLTIPID
    // "INOP", fcu.xml) never give one. FBW's own convention for a legend
    // with nothing to show is a literal `0` (PUSH_FCU_ALT's own
    // <INDICATOR_CODE>0</INDICATOR_CODE>, fcu.xml) — never lit, not a
    // fabricated signal.
    let code = code.replace("#INDICATOR_POWERED#", "1").replace("#INDICATOR_CODE#", "0");
    let code = &code;
    if let Some(i) = code.find('#') {
        // `#NAME#` is one XML element name (MSFS's own `ModelBehaviors`
        // substitution syntax): report just that, not 40 raw characters of
        // whatever RPN happened to follow it, so callers (and the static
        // triage in probe.rs) can group and count by the exact parameter
        // name instead of by an arbitrary, code-dependent snippet.
        let after = &code[i + 1..];
        return Err(match after.find('#') {
            Some(j) => format!("template parameter never given: #{}#", &after[..j]),
            // No closing '#': a stray marker, not a real placeholder. Keep
            // the old bounded snippet so this case is still diagnosable.
            None => format!("template parameter never given: {}", code[i..].chars().take(40).collect::<String>()),
        });
    }
    let toks = rpn::tokenize(code);
    let mut out = String::new();
    let mut depth = 1usize;
    let mut i = 0;
    let ind = |d: usize| "  ".repeat(d);
    while i < toks.len() {
        let t = &toks[i];
        i += 1;
        let line: String = match t {
            Tok::Num(n) => format!("P(s, {})", lua_num(*n)),
            Tok::Str(ev) => match toks.get(i) {
                // 'EVENT' (>F:KeyEvent): the key event it names.
                Some(Tok::Var { write: true, kind, name, .. }) if kind == "F" && name.trim().eq_ignore_ascii_case("KeyEvent") => {
                    i += 1;
                    k_event_lua(ev.trim(), 0)?
                }
                // A plain string literal (e.g. 'WheelUp' before `scmp`/`scmi`):
                // pushed as a Lua string, like rpn.rs's interpreter's Val::Str.
                _ => format!("P(s, {ev:?})"),
            },
            Tok::If => {
                out.push_str(&format!("{}if Q(s) ~= 0 then\n", ind(depth)));
                depth += 1;
                continue;
            }
            Tok::End => {
                depth = depth.saturating_sub(1).max(1);
                if matches!(toks.get(i), Some(Tok::Els)) {
                    i += 1;
                    out.push_str(&format!("{}else\n", ind(depth)));
                    depth += 1;
                } else {
                    out.push_str(&format!("{}end\n", ind(depth)));
                }
                continue;
            }
            Tok::Els => return Err("unpaired els{".into()),
            Tok::Var { write, kind, name, unit } => {
                // MSFS converts simulation variables to a Bool unit but hands
                // an L: variable back as it holds it: FlyByWire's apron step
                // `(L:A32NX_START_STATE, Bool) 2 ==` reads 2, not 1.
                let bool_unit = kind != "L" && (unit.eq_ignore_ascii_case("bool") || unit.eq_ignore_ascii_case("boolean"));
                if *write && kind == "K" {
                    let (n, ev) = match name.split_once(':') {
                        Some((c, e)) if c.trim().parse::<usize>().is_ok() => (c.trim().parse::<usize>().unwrap_or(1), e.trim()),
                        _ => (1, name.trim()),
                    };
                    k_event_lua(ev, n)?
                } else if *write && kind == "H" {
                    match events::h_event(name) {
                        Some(c) => format!("CMD({})", c.lua()),
                        // For FlyByWire's JavaScript instruments, which X-Plane does not run.
                        None => format!("-- (>H:{})", name.trim()),
                    }
                } else if *write && kind == "B" {
                    match b_event(name.trim()) {
                        Some((base, true)) => {
                            let d = format!("fbw/cockpit/ie/{}", sanitize_id(base));
                            format!("do if #s > 0 then Q(s) end wr({d:?}, B(rd({d:?}) == 0)) end")
                        }
                        Some((base, false)) => format!("wr({:?}, Q(s))", format!("fbw/cockpit/ie/{}", sanitize_id(base))),
                        None => return Err(format!("B: event {name} (input event)")),
                    }
                } else if *write && kind == "L" && matches!(name.trim(), "A32NX_PRIORITY_TAKEOVER:1" | "A32NX_PRIORITY_TAKEOVER:2") {
                    // Sidestick priority takeover (FBW_Airbus_Sidestick_Priority
                    // template, A32NX_Interior_Misc.xml:372-392): MSFS's cockpit
                    // writes this colon-indexed L:var to 1 while the pushbutton
                    // is held, 0 on release. home()'s generic "L" case would
                    // sanitize the colon into fbw/A32NX_PRIORITY_TAKEOVER_1/_2,
                    // a dataref nothing reads: the plugin republishes the colon
                    // form itself, every tick, from its own command state
                    // (D:/A380/fbw-xp-systems/src/lib.rs:1523-1533,
                    // afs_events::priority_takeover_held), overwriting whatever
                    // SASL wrote here regardless of name. Its only real input is
                    // the begin/end phase of its own command
                    // (afs_events::PriorityTakeoverCommands: fbw/event/
                    // A32NX_PRIORITY_TAKEOVER_CAPT for :1, _FO for :2), which a
                    // plain dataref write cannot express. Fire that command's
                    // begin/end directly from the value this write carries.
                    let suffix = if name.trim().ends_with(":1") { "CAPT" } else { "FO" };
                    format!("CMD_HELD({:?}, Q(s))", format!("fbw/event/A32NX_PRIORITY_TAKEOVER_{suffix}"))
                } else if *write && matches!(kind.as_str(), "E" | "P" | "M" | "R" | "F") {
                    return Err(format!("{kind}: event {name} (input event)"));
                } else if !*write && matches!(kind.as_str(), "M" | "R" | "S") {
                    // M:Event/R:.../S:... carry MSFS's own mouse/joystick
                    // event context (which button, which axis...), which
                    // this port has no counterpart for (rpn.rs's interpreter
                    // treats them the same way, `Val::Str(String::new())`):
                    // an empty string, so e.g. the common `(M:Event) 'WheelUp'
                    // scmi 0 == if{ ... } els{ <plain click> }` idiom always
                    // falls through to the plain click.
                    "P(s, \"\")".to_string()
                } else {
                    let k = rpn::key(kind, name);
                    let scale = rpn::unit_scale(kind, name, unit);
                    let norm = |v: &str| if bool_unit { format!("B({v} ~= 0)") } else { v.to_string() };
                    let scaled = |v: String| if scale != 1.0 { format!("{v} * {}", lua_num(scale)) } else { v };
                    if !*write {
                        if let Some(c) = consts.get(&k) {
                            let v = if bool_unit { if *c != 0.0 { 1.0 } else { 0.0 } } else { *c };
                            out.push_str(&format!("{}P(s, {})\n", ind(depth), lua_num(v)));
                            continue;
                        }
                    }
                    let Some(h) = home(&k, control) else { return Err(format!("reads {kind}:{name}")) };
                    match (write, h) {
                        (true, Home::Sys(d)) => format!("wr({d:?}, {})", scaled_write(&norm("Q(s)"), scale)),
                        (true, Home::Local(n)) => format!("LV[{n:?}] = {}", norm("Q(s)")),
                        (true, Home::XPlane(d, f)) => format!("XSET({d:?}, {} / {})", norm("Q(s)"), lua_num(f)),
                        (false, Home::Sys(d)) => format!("P(s, {})", norm(&scaled(format!("rd({d:?})")))),
                        (false, Home::Local(n)) => format!("P(s, {})", norm(&format!("(LV[{n:?}] or 0)"))),
                        (false, Home::XPlane(d, f)) => format!("P(s, {})", norm(&format!("XGET({d:?}) * {}", lua_num(f)))),
                    }
                }
            }
            Tok::Word(w) => {
                let lw = w.to_ascii_lowercase();
                let bin = |e: &str| format!("do local y, x = Q(s), Q(s) P(s, {e}) end");
                let un = |e: &str| format!("do local x = Q(s) P(s, {e}) end");
                match lw.as_str() {
                    "+" => bin("x + y"),
                    "-" => bin("x - y"),
                    "*" => bin("x * y"),
                    "/" => bin("(y ~= 0) and x / y or 0"),
                    "%" => bin("(y ~= 0) and math.fmod(x, y) or 0"),
                    "min" => bin("math.min(x, y)"),
                    "max" => bin("math.max(x, y)"),
                    "pow" => bin("x ^ y"),
                    "==" | "eq" => bin("B(x == y)"),
                    "!=" | "ne" => bin("B(x ~= y)"),
                    "<" | "lt" => bin("B(x < y)"),
                    ">" | "gt" => bin("B(x > y)"),
                    "<=" | "le" => bin("B(x <= y)"),
                    ">=" | "ge" => bin("B(x >= y)"),
                    // String compare (scmp, case-sensitive) / (scmi,
                    // case-insensitive), MSFS RPN's strcmp-style -1/0/1;
                    // `tostring` copes with a mixed string/number operand
                    // the way rpn.rs's own Val formats one (`{v:?}`).
                    "scmp" => bin("(tostring(x) == tostring(y)) and 0 or ((tostring(x) < tostring(y)) and -1 or 1)"),
                    "scmi" => {
                        bin("(tostring(x):lower() == tostring(y):lower()) and 0 or ((tostring(x):lower() < tostring(y):lower()) and -1 or 1)")
                    }
                    "and" | "&&" => bin("B(x ~= 0 and y ~= 0)"),
                    "or" | "||" => bin("B(x ~= 0 or y ~= 0)"),
                    "&" => bin("bit.band(x, y)"),
                    "|" => bin("bit.bor(x, y)"),
                    "^" => bin("bit.bxor(x, y)"),
                    "!" | "not" => un("B(x == 0)"),
                    "neg" => un("-x"),
                    "++" => un("x + 1"),
                    "--" => un("x - 1"),
                    "abs" => un("math.abs(x)"),
                    "flr" => un("math.floor(x)"),
                    "ceil" => un("math.ceil(x)"),
                    "near" | "rnd" => un("math.floor(x + 0.5)"),
                    "int" => un("(x >= 0) and math.floor(x) or math.ceil(x)"),
                    "sqr" => un("x * x"),
                    "sqrt" => un("math.sqrt(math.max(x, 0))"),
                    "~" => un("bit.bnot(x)"),
                    "rng" => "do local x, hi, lo = Q(s), Q(s), Q(s) P(s, B(lo <= x and x <= hi)) end".into(),
                    "?" => "do local c, y, x = Q(s), Q(s), Q(s) P(s, (c ~= 0) and x or y) end".into(),
                    "d" => "P(s, s[#s] or 0)".into(),
                    "r" => "do local y, x = Q(s), Q(s) P(s, y) P(s, x) end".into(),
                    "p" => "Q(s)".into(),
                    "c" => "s = {}".into(),
                    "quit" => "do return end".into(),
                    "pi" => "P(s, math.pi)".into(),
                    "true" => "P(s, 1)".into(),
                    "false" => "P(s, 0)".into(),
                    _ => {
                        if let Some(n) = lw.strip_prefix("sp").and_then(|n| n.parse::<u32>().ok()) {
                            format!("r[{n}] = Q(s)")
                        } else if let Some(n) = lw.strip_prefix('s').and_then(|n| n.parse::<u32>().ok()) {
                            format!("r[{n}] = s[#s] or 0")
                        } else if let Some(n) = lw.strip_prefix('l').and_then(|n| n.parse::<u32>().ok()) {
                            format!("P(s, r[{n}] or 0)")
                        } else {
                            return Err(format!("RPN word {w:?}"));
                        }
                    }
                }
            }
        };
        out.push_str(&ind(depth));
        out.push_str(&line);
        out.push('\n');
    }
    Ok(out)
}

fn scaled_write(v: &str, scale: f64) -> String {
    if scale != 1.0 {
        format!("({v}) / {}", lua_num(scale))
    } else {
        v.to_string()
    }
}

fn dataref_tail(s: &str) -> String {
    s.chars().map(|c| if c.is_ascii_alphanumeric() || c == '_' || c == '/' { c } else { '_' }).collect()
}

fn lua_num(n: f64) -> String {
    if n.fract() == 0.0 && n.abs() < 1e15 {
        format!("{}", n as i64)
    } else {
        format!("{n}")
    }
}

// ---------------------------------------------------------------------------
// Resolution

/// What a click does in X-Plane.
#[derive(Clone, Debug, PartialEq)]
pub enum Click {
    /// `ATTR_manip_toggle`: each click flips the dataref between two values.
    Toggle { dref: String, on: f64, off: f64 },
    /// `ATTR_manip_push`: the dataref holds `down` while pressed.
    Hold { dref: String, down: f64, up: f64 },
    /// `ATTR_manip_axis_*`: the dataref steps from `v0` to `v1`.
    Axis { dref: String, v0: f64, v1: f64, step: f64, knob: bool, horizontal: bool },
    /// Commands run by SASL: Lua function bodies.
    Script(Script),
    /// `ATTR_manip_command`: the click is one command (the systems plugin's
    /// `fbw/event/*` or X-Plane's own).
    Command { cmd: String },
    /// `ATTR_manip_command_knob`: a turn each way is one command.
    CommandKnob { up: String, down: String },
}

#[derive(Clone, Debug, PartialEq, Default)]
pub struct Script {
    /// Button: on press / release.
    pub press: Option<String>,
    pub release: Option<String>,
    /// Two-way: step up / down (knobs, multi-position switches).
    pub up: Option<String>,
    pub down: Option<String>,
    /// Momentary switch: back to rest on release.
    pub up_release: Option<String>,
    pub down_release: Option<String>,
    pub knob: bool,
    pub horizontal: bool,
}

#[derive(Clone, Debug)]
pub struct Binding {
    pub anim: String,
    pub node: String,
    pub template: String,
    pub click: Click,
    /// Lua function body returning the clip position 0..1, or `None` for a
    /// momentary press animation.
    pub look: Option<String>,
    /// Systems datarefs the control writes.
    pub targets: BTreeSet<String>,
    /// Systems datarefs its code and animation read.
    pub reads: BTreeSet<String>,
    /// X-Plane commands (and `dataref <name>` X-Plane datarefs) it fires.
    pub commands: BTreeSet<String>,
}

#[derive(Clone, Debug)]
pub struct Unresolved {
    pub anim: String,
    pub node: String,
    pub template: String,
    /// The `UseTemplate` chain that reached `template` (outermost first),
    /// carried from `Control::chain`: for a "template parameter never given"
    /// `reason`, this says which caller in the chain is the one that should
    /// have passed it (usually the outermost, or wherever a
    /// `DefaultTemplateParameters`/`OverrideTemplateParameters` should add a
    /// default).
    pub chain: Vec<String>,
    pub reason: String,
}

/// One `fbw/cockpit/<anim>` dataref crossing a point in its 0..1 travel that
/// should fire a Wwise click sound (`sound/anim_triggers.rs` in the plugin
/// reads these, one per line, from `sound_triggers.txt`).
#[derive(Clone, Debug, PartialEq)]
pub struct SoundTrigger {
    /// The full dataref name (`fbw/cockpit/<sanitized anim>`), matching what
    /// `rig.rs` actually created for this animation — not just the raw
    /// `Animation` name, so a mismatch is caught here, at convert time, not
    /// silently at runtime.
    pub dataref: String,
    pub direction: String,
    /// `<EventTrigger NormalizedTime="...">`. Mutually exclusive with
    /// `count` below — see `expand::SoundTriggerCode::normalized_time`.
    pub normalized_time: Option<f64>,
    /// `<EventTrigger Count="...">` (`ASOBO_GT_AnimTriggers_SoundEvents_Same`,
    /// FlyByWire's flap-lever cover `Count=1` and speedbrake lever
    /// `Count=3`): fire this many evenly spaced times instead of once at a
    /// fixed point. Expanded into concrete points in the plugin's
    /// `sound::anim_triggers::parse`, not here — see
    /// `expand::SoundTriggerCode::count`.
    pub count: Option<u32>,
    pub wwise_event: String,
}

#[derive(Default)]
pub struct Resolution {
    pub bindings: Vec<Binding>,
    pub unresolved: Vec<Unresolved>,
    /// Levers: dragged, left on their own datarefs.
    pub levers: Vec<String>,
    /// Animations with no control of their own that follow variables the
    /// bound controls set: (clip, Lua body returning 0..1).
    pub mirrors: Vec<(String, String)>,
    /// Systems datarefs those animations read.
    pub mirror_reads: BTreeSet<String>,
    /// Every node's emissive and visibility code, when MSFS's own template
    /// definitions were at hand.
    pub lights: Option<super::emissive::Lights>,
    /// The components' own update codes.
    pub updates: Vec<Update>,
    /// Click/switch sounds resolved to a real `fbw/cockpit/<anim>` dataref,
    /// when MSFS's own template definitions were at hand (see
    /// [`SoundTrigger`]).
    pub sound_triggers: Vec<SoundTrigger>,
}

/// A component's update code, as SASL runs it.
#[derive(Clone, Debug)]
pub struct Update {
    pub node: String,
    pub template: String,
    /// Runs per second (every frame when `None`), or once.
    pub frequency: Option<f64>,
    pub once: bool,
    /// Lua function body, or why the code cannot run.
    pub lua: Result<String, String>,
    /// Systems datarefs it reads or writes.
    pub datarefs: BTreeSet<String>,
}

/// Translate the components' update codes. Their MSFS-only key events
/// (MSFS's own lights and autopilot slots) have no FBW variable; an update
/// firing one is left out, as a control firing one is.
pub fn resolve_updates(updates: &[super::expand::UpdateCode]) -> Vec<Update> {
    updates
        .iter()
        .enumerate()
        .map(|(i, u)| {
            let id = format!("update_{}_{i}", sanitize_id(u.node.as_deref().unwrap_or("")));
            let lua = to_lua(&u.code, &id).map(|b| wrap_fn(&b));
            let mut datarefs = BTreeSet::new();
            if lua.is_ok() {
                let codes = [u.code.clone()];
                datarefs.extend(sys_reads(&codes));
                datarefs.extend(targets_of(&[u.code.as_str()]).0);
            }
            Update { node: u.node.clone().unwrap_or_default(), template: u.template.clone(), frequency: u.frequency, once: u.once, lua, datarefs }
        })
        .collect()
}

/// A short, groupable form of an unresolved reason.
pub fn reason_group(reason: &str) -> String {
    if reason.starts_with("only fires H:") {
        "only drives FBW's JavaScript instruments (H: events)".into()
    } else if reason.starts_with("MSFS event K:") && reason.contains("no fbw/event command") {
        "FlyByWire module key event with no fbw/event command in the systems plugin".into()
    } else if reason.starts_with("push/pull on the part") {
        "push-pull knob's push/pull: its part carries the turn's manipulator".into()
    } else if reason.starts_with("MSFS event K:") {
        "MSFS key event with no FBW variable (MSFS's own systems)".into()
    } else if reason.starts_with("template parameter never given") {
        "a template parameter the XML never gives (guards and covers without a variable)".into()
    } else if reason.starts_with("input event interaction") {
        "MSFS input-event knob or push not translated (RMP audio knobs)".into()
    } else if reason.starts_with("Asobo template") {
        "Asobo lighting/autopilot template acting on MSFS's own systems".into()
    } else {
        reason.split(':').next().unwrap_or(reason).trim().to_string()
    }
}

impl Resolution {
    pub fn binding(&self, anim: &str) -> Option<&Binding> {
        let a = anim.trim();
        self.bindings.iter().find(|b| b.anim.eq_ignore_ascii_case(a))
    }
}

/// The systems variables code writes (as keys), over a few states.
fn probe_writes(code: &str, states: &[HashMap<String, f64>], stack: &[f64]) -> (BTreeSet<String>, Vec<rpn::Run>) {
    let mut keys = BTreeSet::new();
    let mut runs = Vec::new();
    for st in states {
        let r = rpn::run_with(code, st, &KEvents, stack.to_vec());
        for (k, _) in &r.writes {
            if matches!(home(k, "x"), Some(Home::Sys(_) | Home::XPlane(..))) {
                keys.insert(k.clone());
            }
        }
        // A command is a change too: code firing one is no plain toggle or set.
        if !r.commands.is_empty() {
            keys.insert("command".into());
        }
        runs.push(r);
    }
    (keys, runs)
}

/// States to probe code on: every systems variable it reads at 0 and 1 (all
/// combinations up to six variables).
fn probe_states(codes: &[&str]) -> Vec<HashMap<String, f64>> {
    let mut reads: BTreeSet<String> = BTreeSet::new();
    // Branches hide reads: run with what was found at 0, then at 1.
    for round in 0..3 {
        let st: HashMap<String, f64> = reads.iter().map(|k| (k.clone(), if round == 1 { 1.0 } else { 0.0 })).collect();
        for c in codes {
            let r = rpn::run(c, &st, &KEvents);
            reads.extend(r.reads.into_iter());
        }
    }
    let reads: Vec<String> = reads.into_iter().take(6).collect();
    (0..(1usize << reads.len()))
        .map(|m| reads.iter().enumerate().map(|(i, k)| (k.clone(), ((m >> i) & 1) as f64)).collect())
        .collect()
}

fn final_value(r: &rpn::Run, key: &str) -> Option<f64> {
    r.writes.iter().rev().find(|(k, _)| k == key).map(|(_, v)| *v)
}

/// A single systems variable that every run toggles between 0 and 1.
fn as_toggle(code: &str) -> Option<String> {
    let states = probe_states(&[code]);
    let (keys, runs) = probe_writes(code, &states, &[]);
    if keys.len() != 1 {
        return None;
    }
    let k = keys.into_iter().next()?;
    for (st, r) in states.iter().zip(&runs) {
        let old = st.get(&k).copied().unwrap_or(0.0);
        if final_value(r, &k)? != if old != 0.0 { 0.0 } else { 1.0 } {
            return None;
        }
        // Nothing else of the systems' may change.
        if r.writes.iter().any(|(w, _)| w != &k && matches!(home(w, "x"), Some(Home::Sys(_)))) {
            return None;
        }
    }
    Some(k)
}

/// A single systems variable set to one constant whatever the state.
fn as_set(code: &str) -> Option<(String, f64)> {
    let states = probe_states(&[code]);
    let (keys, runs) = probe_writes(code, &states, &[]);
    if keys.len() != 1 {
        return None;
    }
    let k = keys.into_iter().next()?;
    let v = final_value(runs.first()?, &k)?;
    runs.iter().all(|r| final_value(r, &k) == Some(v)).then_some((k, v))
}

/// Code turning a single variable by a fixed step within limits:
/// `(V) d + hi min (>V)`. Returns (key, step, lo, hi).
fn as_turn(cw: &str, ccw: &str) -> Option<(String, f64, f64, f64)> {
    let first = rpn::run(cw, &HashMap::new(), &KEvents);
    let (keys, _) = probe_writes(cw, &[HashMap::new()], &[]);
    if keys.len() != 1 || !first.reads.iter().all(|r| keys.contains(r)) {
        return None;
    }
    let k = keys.into_iter().next()?;
    let at = |code: &str, v: f64| -> Option<f64> {
        let st = HashMap::from([(k.clone(), v)]);
        let r = rpn::run(code, &st, &KEvents);
        if !r.commands.is_empty() || r.writes.iter().any(|(w, _)| w != &k && !matches!(home(w, "x"), Some(Home::Local(_)))) {
            return None;
        }
        final_value(&r, &k)
    };
    // The stops, by turning far past them; the step, from the middle.
    let hi = at(cw, 1e9)?;
    let lo = at(ccw, -1e9)?;
    if !(hi < 1e8 && lo > -1e8 && hi > lo) {
        return None;
    }
    let mid = lo + ((hi - lo) / 2.0).floor();
    let step = at(cw, mid)? - mid;
    if step <= 0.0 || (at(ccw, mid)? - (mid - step)).abs() > 1e-9 {
        return None;
    }
    Some((k, step, lo, hi))
}

/// Positions setting one variable linearly: `codes[i]` sets V to v0 + i*d.
fn as_positions(codes: &[String]) -> Option<(String, f64, f64)> {
    let mut key: Option<String> = None;
    let mut vals = Vec::new();
    for c in codes {
        let (k, v) = as_set(c)?;
        if key.as_ref().is_some_and(|x| x != &k) {
            return None;
        }
        key = Some(k);
        vals.push(v);
    }
    let (v0, v1) = (*vals.first()?, *vals.last()?);
    let n = vals.len();
    if n < 2 || v0 == v1 {
        return None;
    }
    let d = (v1 - v0) / (n - 1) as f64;
    vals.iter().enumerate().all(|(i, v)| (v - (v0 + d * i as f64)).abs() < 1e-9).then(|| (key.unwrap_or_default(), v0, v1))
}

/// `local s, r = {}, {}` when `body` uses the RPN register table (`spN`/`sN`
/// store, `lN` load — see the `r[` writes in `to_lua_with`'s word match);
/// `local s = {}` alone otherwise, so a body that never touches a register
/// does not allocate an empty table for one every time it runs (a
/// `look()`/`look_slow()` body every frame or 20 Hz, see rig.rs; a
/// `light()` body the same).
pub fn locals_for(body: &str) -> &'static str {
    if body.contains("r[") {
        "local s, r = {}, {}"
    } else {
        "local s = {}"
    }
}

fn wrap_fn(body: &str) -> String {
    format!("{}\n{body}", locals_for(body))
}

/// The datarefs code writes, and the commands (and X-Plane datarefs) it
/// fires, over the states it can meet.
fn targets_of(codes: &[&str]) -> (BTreeSet<String>, BTreeSet<String>) {
    let mut out = BTreeSet::new();
    let mut cmds = BTreeSet::new();
    for c in codes {
        let states = probe_states(&[c]);
        let (keys, runs) = probe_writes(c, &states, &[]);
        for k in keys {
            if let Some(Home::Sys(d)) = home(&k, "x") {
                out.insert(d);
            }
        }
        for r in &runs {
            cmds.extend(r.commands.iter().cloned());
            for (k, _) in &r.writes {
                if let Some(Home::XPlane(d, _)) = home(k, "x") {
                    cmds.insert(format!("dataref {d}"));
                }
            }
        }
    }
    (out, cmds)
}

/// One command fired whatever the state, and nothing else changed.
fn as_command(code: &str) -> Option<String> {
    let states = probe_states(&[code]);
    let (_, runs) = probe_writes(code, &states, &[]);
    let first = runs.first()?.commands.first()?.clone();
    runs.iter()
        .all(|r| r.commands.len() == 1 && r.commands[0] == first && r.writes.is_empty() && !first.starts_with("dataref "))
        .then_some(first)
}

/// Whether code reads state that only exists in MSFS or in the Lua layer
/// (a direct manipulator never runs the code that would change it).
fn reads_local(code: &str) -> bool {
    let r = rpn::run(code, &HashMap::new(), &KEvents);
    r.reads.iter().any(|k| !matches!(home(k, "x"), Some(Home::Sys(_))))
}

/// Whether every local (`O:`, MSFS `XMLVAR_`) variable `code` reads is state
/// the same code also writes: a self-contained per-frame accumulator (the
/// wiper's sweep position and direction, FBW_Airbus_Wiper's `O:AnimCode` et
/// al, A32NX_Exterior.xml), safe for a `look()` callback to re-run every
/// frame like any other mirror, unlike state some other, unreproduced MSFS
/// control sets that this code only reads (what `reads_local` above guards
/// against everywhere else it is used). Real X-Plane state (`Home::XPlane`)
/// counts as fine here too: `(A:ANIMATION DELTA TIME, seconds)`
/// (`events::xplane_var`) is exactly that, not local state.
fn locals_self_contained(code: &str) -> bool {
    let r = rpn::run(code, &HashMap::new(), &KEvents);
    let written: BTreeSet<&str> = r.writes.iter().map(|(k, _)| k.as_str()).collect();
    r.reads.iter().all(|k| {
        matches!(home(k, "x"), Some(Home::Sys(_)) | Some(Home::XPlane(..)))
            || written.contains(k.as_str())
            // `O:AnimCode` is the MSFS SDK's own reserved name for an
            // ASOBO_GT_Anim_Code control's previously-evaluated ANIM_CODE
            // result -- the sim feeds it back in automatically every tick
            // (the wiper's own FAILURE_CODE, unused here, spells the same
            // convention out explicitly: `(O:NewAnimCode) (>O:AnimCode)`).
            // The base Asobo template that provides this feedback is not
            // expanded for animation codes (see resolve()'s own doc), so
            // this converter never sees an explicit write for it in FBW's
            // own literal ANIM_CODE text -- reading it is still as safe as
            // any other externally-fed state (`Home::XPlane` above), not a
            // read of state some other, unreproduced control sets.
            || k == "O:AnimCode"
    })
}

fn look_code(look: &Look) -> Option<&str> {
    match look {
        Look::Code { code, .. } => Some(code),
        _ => None,
    }
}

/// The systems datarefs code reads.
fn sys_reads(codes: &[String]) -> BTreeSet<String> {
    let refs: Vec<&str> = codes.iter().map(String::as_str).collect();
    let mut out = BTreeSet::new();
    for st in probe_states(&refs) {
        for c in &refs {
            for k in rpn::run(c, &st, &KEvents).reads {
                if let Some(Home::Sys(d)) = home(&k, "x") {
                    out.insert(d);
                }
            }
        }
    }
    out
}

/// Why code that changes no FBW variable cannot be bound.
fn no_target_reason(codes: &[&str]) -> String {
    let mut h = BTreeSet::new();
    for c in codes {
        let r = rpn::run(c, &HashMap::new(), &KEvents);
        h.extend(r.events.into_iter().filter(|e| e.starts_with("H:")));
    }
    if h.is_empty() {
        "changes no FBW variable (MSFS-local state only)".into()
    } else {
        format!("only fires H: events for FBW's JavaScript instruments ({})", h.into_iter().take(3).collect::<Vec<_>>().join(", "))
    }
}

fn look_lua(look: &Look, control: &str) -> Result<Option<String>, String> {
    match look {
        Look::Press => Ok(None),
        Look::None => Err("no animation code".into()),
        Look::Code { code, length } => {
            let body = to_lua(code, control)?;
            let result = if body.contains("r[") { "s[#s] or r[0] or 0" } else { "s[#s] or 0" };
            Ok(Some(format!("{}\n{body}return ({result}) / {}", locals_for(&body), lua_num(length.max(1e-6)))))
        }
    }
}

/// Resolve one control to a binding.
fn resolve(c: &Control, extra_look: Option<&Look>) -> Result<Binding, String> {
    let action = c.action.clone()?;
    let look = match (&c.look, extra_look) {
        (Look::None, Some(l)) => l.clone(),
        (l, _) => l.clone(),
    };
    let id = sanitize_id(&c.anim);
    let mut all_codes: Vec<String> = match &action {
        Action::Click { press, release } => std::iter::once(press.clone()).chain(release.clone()).collect(),
        Action::States { codes, current, .. } => codes.iter().cloned().chain(std::iter::once(current.clone())).collect(),
        Action::Rotary { cw, ccw } => vec![cw.clone(), ccw.clone()],
        Action::Lever => Vec::new(),
    };
    all_codes.extend(look_code(&look).map(str::to_string));
    let reads = sys_reads(&all_codes);
    let mk = |click: Click, look: Option<String>, (targets, commands): (BTreeSet<String>, BTreeSet<String>)| Binding {
        anim: c.anim.clone(),
        node: c.node.clone(),
        template: c.template.clone(),
        click,
        look,
        targets,
        reads: reads.clone(),
        commands,
    };
    let sys_dref = |k: &str| match home(k, &id) {
        Some(Home::Sys(d)) => Some(d),
        _ => None,
    };
    match action {
        Action::Lever => Err("lever".into()),
        Action::Click { press, release } => {
            let codes: Vec<&str> = std::iter::once(press.as_str()).chain(release.as_deref()).collect();
            let targets = targets_of(&codes);
            // Translate first: this is what says whether the code can run at all.
            let press_lua = to_lua(&press, &id)?;
            let release_lua = release.as_deref().map(|r| to_lua(r, &id)).transpose()?;
            if targets.0.is_empty() && targets.1.is_empty() {
                return Err(no_target_reason(&codes));
            }
            let looks = look_lua(&look, &id);
            // One command, whatever the state: the click fires it directly.
            if release.as_deref().is_none_or(|r| r.trim().is_empty()) {
                if let Some(cmd) = as_command(&press) {
                    let l = looks.as_ref().ok().cloned().flatten();
                    return Ok(mk(Click::Command { cmd }, l, targets));
                }
            }
            // Latching: one variable toggled, and the control shows it.
            if release.is_none() {
                if let (Some(k), Ok(Some(l))) = (as_toggle(&press), &looks) {
                    if let Some(d) = sys_dref(&k) {
                        let l = if look_code(&look).is_some_and(reads_local) { format!("return (rd({d:?}) ~= 0) and 1 or 0") } else { l.clone() };
                        return Ok(mk(Click::Toggle { dref: d, on: 1.0, off: 0.0 }, Some(l), targets));
                    }
                }
            }
            // Held: one variable, set on press and reset on release.
            if let Some(r) = &release {
                if let (Some((k1, down)), Some((k2, up))) = (as_set(&press), as_set(r)) {
                    if k1 == k2 && down != up {
                        if let Some(d) = sys_dref(&k1) {
                            let look = match &looks {
                                Ok(Some(l)) if !look_code(&look).is_some_and(reads_local) => l.clone(),
                                _ => format!("return (rd({d:?}) == {}) and 1 or 0", lua_num(down)),
                            };
                            return Ok(mk(Click::Hold { dref: d, down, up }, Some(look), targets));
                        }
                    }
                }
            }
            let script = Script { press: Some(wrap_fn(&press_lua)), release: release_lua.map(|r| wrap_fn(&r)), ..Default::default() };
            let look = match looks {
                Ok(l) => l,
                Err(_) => None,
            };
            Ok(mk(Click::Script(script), look, targets))
        }
        Action::States { codes, current, knob, horizontal, momentary } => {
            let refs: Vec<&str> = codes.iter().map(String::as_str).collect();
            let targets = targets_of(&refs);
            let luas: Vec<String> = codes.iter().map(|x| to_lua(x, &id)).collect::<Result<_, _>>()?;
            let cur = to_lua(&current, &id)?;
            if targets.0.is_empty() && targets.1.is_empty() {
                return Err(no_target_reason(&refs));
            }
            let looks = look_lua(&look, &id).ok().flatten();
            let n = codes.len();
            if momentary.is_none() {
                // Strip the switch's own position write when judging positions:
                // positions that set one systems variable linearly.
                let sys_only: Vec<String> = codes.clone();
                if let Some((k, v0, v1)) = as_positions(&sys_only) {
                    if let Some(d) = sys_dref(&k) {
                        let step = ((v1 - v0) / (n - 1) as f64).abs();
                        let derived = format!("return (rd({d:?}) - {}) / {}", lua_num(v0), lua_num(v1 - v0));
                        let look = looks.clone().filter(|_| !look_code(&look).is_some_and(reads_local)).unwrap_or(derived);
                        return Ok(mk(Click::Axis { dref: d, v0, v1, step, knob, horizontal }, Some(look), targets));
                    }
                }
            }
            // SASL: position now, step, run that position's code.
            let mut table = String::from("local codes = {\n");
            for l in &luas {
                table.push_str(&format!("function(s, r)\n{l}end,\n"));
            }
            table.push_str("}\n");
            let go = |delta: i32| {
                let result = if cur.contains("r[") { "s[#s] or r[0] or 0" } else { "s[#s] or 0" };
                format!(
                    "{}\n{cur}local now = math.floor(({result}) + 0.5)\n{table}local to = math.max(0, math.min({}, now + ({delta})))\nif to ~= now then codes[to + 1]({{}}, {{}}) end",
                    locals_for(&cur), n - 1
                )
            };
            let back = momentary.map(|rest| format!("{table}codes[{}]({{}}, {{}})", rest + 1));
            let script = Script {
                up: Some(go(1)),
                down: Some(go(-1)),
                up_release: back.clone(),
                down_release: back,
                knob,
                horizontal,
                ..Default::default()
            };
            let look = looks.or_else(|| {
                let result = if cur.contains("r[") { "s[#s] or r[0] or 0" } else { "s[#s] or 0" };
                Some(format!("{}\n{cur}return ({result}) / {}", locals_for(&cur), n.max(2) - 1))
            });
            Ok(mk(Click::Script(script), look, targets))
        }
        Action::Rotary { cw, ccw } => {
            let targets = targets_of(&[&cw, &ccw]);
            let (cw_lua, ccw_lua) = (to_lua(&cw, &id)?, to_lua(&ccw, &id)?);
            if targets.0.is_empty() && targets.1.is_empty() {
                return Err(no_target_reason(&[&cw, &ccw]));
            }
            let looks = look_lua(&look, &id).ok().flatten();
            if let (Some(up), Some(down)) = (as_command(&cw), as_command(&ccw)) {
                return Ok(mk(Click::CommandKnob { up, down }, looks, targets));
            }
            if let Some((k, step, lo, hi)) = as_turn(&cw, &ccw) {
                if let Some(d) = sys_dref(&k) {
                    let derived = format!("return (rd({d:?}) - {}) / {}", lua_num(lo), lua_num(hi - lo));
                    let look = looks.clone().filter(|_| !look_code(&look).is_some_and(reads_local)).unwrap_or(derived);
                    return Ok(mk(Click::Axis { dref: d, v0: lo, v1: hi, step, knob: true, horizontal: false }, Some(look), targets));
                }
            }
            let script = Script { up: Some(wrap_fn(&cw_lua)), down: Some(wrap_fn(&ccw_lua)), knob: true, ..Default::default() };
            Ok(mk(Click::Script(script), looks, targets))
        }
    }
}

pub fn sanitize_id(s: &str) -> String {
    s.trim().chars().map(|c| if c.is_ascii_alphanumeric() || c == '_' { c } else { '_' }).collect()
}

/// Every control of the expanded behaviours, one per animation: the
/// interaction that moves it, first found wins, with raw mouse rectangles
/// only where nothing better is known.
pub fn controls(leaves: &[Leaf]) -> (Vec<Control>, HashMap<String, Look>) {
    let mut by_anim: BTreeMap<String, Control> = BTreeMap::new();
    let mut looks: HashMap<String, Look> = HashMap::new();
    for l in leaves {
        if let Some((a, look)) = look_of(l) {
            looks.entry(a.trim().to_ascii_lowercase()).or_insert(look);
        }
        for c in controls_of(l) {
            let key = c.anim.trim().to_ascii_lowercase();
            match by_anim.get(&key) {
                None => {
                    by_anim.insert(key, c);
                }
                Some(old) => {
                    // A translatable interaction beats a raw mouse rectangle or an
                    // unsupported one.
                    if old.action.is_err() && c.action.is_ok() {
                        by_anim.insert(key, c);
                    }
                }
            }
        }
    }
    (by_anim.into_values().collect(), looks)
}

/// Resolve every control.
pub fn resolve_all(leaves: &[Leaf]) -> Resolution {
    let (controls, looks) = controls(leaves);
    let mut res = Resolution::default();
    for c in &controls {
        let extra = looks.get(&c.anim.trim().to_ascii_lowercase());
        match resolve(c, extra) {
            Ok(b) => res.bindings.push(b),
            Err(e) if e == "lever" => res.levers.push(c.anim.clone()),
            Err(reason) => res.unresolved.push(Unresolved { anim: c.anim.clone(), node: c.node.clone(), template: c.template.clone(), chain: c.chain.clone(), reason }),
        }
    }
    // Animations of their own (a mask, a door, a seat) that follow what a
    // bound control sets.
    let written: BTreeSet<String> = res.bindings.iter().flat_map(|b| b.targets.iter().cloned()).collect();
    let taken: BTreeSet<String> = controls.iter().map(|c| c.anim.trim().to_ascii_lowercase()).collect();
    let mut mirrors: Vec<(String, String)> = Vec::new();
    for (anim, look) in &looks {
        let Look::Code { code, .. } = look else { continue };
        // A clip's own per-frame accumulator (the wiper sweep's `O:AnimCode`
        // et al, written and read only within this same code) is not the
        // "state some other, unreproduced control sets" reads_local guards
        // against, so it does not need reads_local's stricter, Sys-only
        // test to become a mirror: locals_self_contained accepts it.
        if taken.contains(anim) || !locals_self_contained(code) {
            continue;
        }
        let reads = sys_reads(std::slice::from_ref(code));
        if reads.is_empty() || !reads.iter().all(|r| written.contains(r)) {
            continue;
        }
        if let Ok(Some(l)) = look_lua(look, &sanitize_id(anim)) {
            res.mirror_reads.extend(reads);
            mirrors.push((anim.clone(), l));
        }
    }
    mirrors.sort();
    res.mirrors = mirrors;
    res
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn locals_for_only_declares_the_register_table_when_a_body_uses_it() {
        // "1 s0": push 1, then RPN word s0 stores it into register 0 (see
        // the `r[{n}] = ...` arms of the word match above).
        let with_register = to_lua("1 s0", "test").unwrap();
        assert!(with_register.contains("r[0]"), "{with_register}");
        assert_eq!(locals_for(&with_register), "local s, r = {}, {}");

        let without_register = to_lua("1 1 +", "test").unwrap();
        assert!(!without_register.contains("r["), "{without_register}");
        assert_eq!(locals_for(&without_register), "local s = {}");
    }

    #[test]
    fn wrap_fn_drops_the_register_table_when_the_body_never_uses_one() {
        assert_eq!(wrap_fn("P(s, 1)\n"), "local s = {}\nP(s, 1)\n");
        assert!(wrap_fn("r[0] = Q(s)\n").starts_with("local s, r = {}, {}\n"));
    }

    #[test]
    fn look_lua_omits_the_r0_fallback_when_nothing_wrote_a_register() {
        let plain = look_lua(&Look::Code { code: "1 1 +".into(), length: 100.0 }, "test").unwrap().unwrap();
        assert!(plain.starts_with("local s = {}\n"), "{plain}");
        assert!(!plain.contains("r["), "{plain}");

        let staged = look_lua(&Look::Code { code: "1 s0".into(), length: 100.0 }, "test").unwrap().unwrap();
        assert!(staged.starts_with("local s, r = {}, {}\n"), "{staged}");
        assert!(staged.contains("r[0] or 0"), "{staged}");
    }
}
