//! What drives each animation, and the SASL script that does it.
//!
//! Exterior: every clip gets a dataref, `fbw/anim/<clip>`, running 0..1 over
//! the clip. SASL sets the ones X-Plane knows about every frame: gear, doors,
//! bogie tilt, steering, wheels, flaps, slats, spoilers, ailerons, elevators,
//! stabiliser trim, rudders, fans and reversers, and the wing flex and engine
//! wobble from FlyByWire's own flex model. The rest (passenger and cargo
//! doors, RAT, outflow valves) start at their rest position, ready for the
//! systems to drive.
//! Which way each surface moves is measured from the model, not assumed.
//!
//! Cockpit: every clip gets a dataref, `fbw/cockpit/<clip>`, and a
//! manipulator on the parts it moves. Pushbuttons press while held and spring
//! back; knobs turn by click or wheel through their detents; switches step
//! through their positions; guards, covers and breakers toggle; levers, seats,
//! tables, shades and windows drag.
//!
//! Where FlyByWire's behaviour XML says what a control does (see
//! `behaviour`), its click acts on the systems instead: the manipulator
//! writes the systems' own `fbw/<variable>` dataref, or fires a command
//! whose SASL handler runs the control's MSFS code translated to Lua, and
//! the control's animation follows the variable. The manipulator also goes
//! on the node the XML makes clickable, so parts clicked through a separate
//! node (oxygen masks, seats) get one. Controls the XML does not resolve
//! keep the behaviour above.
//!
//! Lights: every legend, backlight and annunciator glows as its emissive
//! code in the XML says (see `behaviour::emissive`): `ATTR_light_level` on
//! the systems dataref the code reads, or on a helper dataref
//! (`fbw/cockpit/lt/<node>`, `fbw/cockpit/vis/<node>`) SASL computes from the
//! code every frame; visibility codes hide their nodes the same way.

use std::collections::{BTreeMap, BTreeSet, HashMap, HashSet};
use std::fmt::Write as _;

use crate::behaviour::emissive::Drive;
use crate::behaviour::expand::LightKind;
use crate::behaviour::sim::SimState;
use crate::behaviour::{bind, Click, Resolution};
use crate::model::anim::{apply, inverse, norm, num, qangle, qconj, qmul, quat_of, sub, xp, ClipIndex, V3};
use crate::model::glb::{mul, M4};
use crate::model::Model;

/// Everything needed to animate one model.
#[derive(Default)]
pub struct Rig {
    /// Dataref playing each animated clip.
    pub drefs: HashMap<usize, String>,
    /// Show/hide commands per node.
    pub vis: HashMap<usize, Vec<String>>,
    /// Manipulator per clip, with its priority when clips share a node.
    pub manips: HashMap<usize, (u8, String)>,
    /// Datarefs SASL creates, with their start values.
    pub created: Vec<(String, f64)>,
    /// Lua statements run every frame.
    pub update: Vec<String>,
    pub report: Vec<String>,
    /// Per node (and the geometry below it), `ATTR_light_level` arguments
    /// "v1 v2 dataref".
    pub light_levels: HashMap<usize, String>,
    /// Nodes whose emissive code says whether they glow: lit even where the
    /// material's own emissive factor is zero (MSFS's code overrides it), or
    /// never lit (a code that is always 0).
    pub light_driven: HashMap<usize, bool>,
    /// Lua bodies computing helper datarefs every frame: (body, datarefs).
    pub light_lua: Vec<(String, Vec<String>)>,
    /// The components' update codes: (Lua body, runs per second or every
    /// frame, once only, what it is).
    pub updates: Vec<(String, Option<f64>, bool, String)>,
    /// Manipulators on the nodes FlyByWire's XML makes clickable, with
    /// their priority (these win over a clip's own on the same node).
    pub node_manips: HashMap<usize, (u8, String)>,
    /// Lua run once at load: control commands and animation sources.
    pub lua: Vec<String>,
    /// Systems datarefs the cockpit reads or writes.
    pub targets: BTreeSet<String>,
    /// One line per XML control: what its click does, or why it keeps its
    /// own dataref.
    pub bindings: Vec<String>,
    /// Each gear leg's own strut/tyre compression travel (metres), measured
    /// from this leg's own clip (see `exterior()`'s `gear_travel_m`):
    /// `[nose, l_body, r_body, l_wing, r_wing]`. `None` where the model has
    /// no such clip (a non-exterior rig, or one missing that leg) -- callers
    /// fall back to a flat assumption rather than a real measurement.
    pub gear_travel_m: [Option<f64>; 5],
}

/// A clip name as a dataref name part.
pub fn sanitize(name: &str) -> String {
    let s: String = name
        .trim()
        .chars()
        .map(|c| if c.is_ascii_alphanumeric() || c == '_' { c } else { '_' })
        .collect();
    if s.is_empty() {
        "unnamed".into()
    } else {
        s
    }
}

fn unique(used: &mut HashSet<String>, base: String) -> String {
    if used.insert(base.clone()) {
        return base;
    }
    (2..).map(|i| format!("{base}_{i}")).find(|n| used.insert(n.clone())).unwrap_or(base)
}

/// Node index by lower-case name (the first of duplicates).
fn node_names(model: &Model) -> HashMap<String, usize> {
    let mut m = HashMap::new();
    for (i, n) in model.nodes.iter().enumerate() {
        m.entry(n.name.trim().to_ascii_lowercase()).or_insert(i);
    }
    m
}

fn at(pairs: &[(usize, f64)]) -> HashMap<usize, f64> {
    pairs.iter().copied().collect()
}

fn origin(m: &M4) -> V3 {
    [m[12], m[13], m[14]]
}

/// The node a clip moves most over its range.
fn main_node(model: &Model, idx: &ClipIndex, clip: usize) -> Option<usize> {
    idx.nodes_of(model, clip)
        .into_iter()
        .map(|n| {
            let a = idx.world_at(model, n, &at(&[(clip, 0.0)]));
            let b = idx.world_at(model, n, &at(&[(clip, 1.0)]));
            let ang = qangle(qmul(quat_of(&b), qconj(quat_of(&a))));
            (ang + norm(sub(origin(&b), origin(&a))), n)
        })
        .max_by(|x, y| x.0.total_cmp(&y.0))
        .map(|x| x.1)
}

/// How far a point, `offset` from the node's origin (glTF world axes),
/// moves when the clip goes from `fa` to `fb` with other clips at `base`.
#[allow(clippy::too_many_arguments)]
fn moved(model: &Model, idx: &ClipIndex, node: usize, offset: V3, base: &HashMap<usize, f64>, clip: usize, fa: f64, fb: f64) -> V3 {
    let mut va = base.clone();
    va.insert(clip, fa);
    let mut vb = base.clone();
    vb.insert(clip, fb);
    let wa = idx.world_at(model, node, &va);
    let wb = idx.world_at(model, node, &vb);
    let o = origin(&wa);
    let pa = [o[0] + offset[0], o[1] + offset[1], o[2] + offset[2]];
    sub(apply(&wb, apply(&inverse(&wa), pa)), pa)
}

/// Total turn of a node through a clip's keys, in degrees.
fn sweep_deg(model: &Model, idx: &ClipIndex, clip: usize, node: usize) -> f64 {
    let mut prev = None;
    let mut total = 0.0;
    for f in idx.key_fractions(model, clip) {
        let q = quat_of(&idx.world_at(model, node, &at(&[(clip, f)])));
        if let Some(p) = prev {
            total += qangle(qmul(q, qconj(p)));
        }
        prev = Some(q);
    }
    total.to_degrees()
}

/// Half the turn of a node between the clip's ends, in degrees.
fn half_range_deg(model: &Model, idx: &ClipIndex, clip: usize, node: usize, base: &HashMap<usize, f64>) -> f64 {
    let mut va = base.clone();
    va.insert(clip, 0.0);
    let mut vb = base.clone();
    vb.insert(clip, 1.0);
    let (a, b) = (idx.world_at(model, node, &va), idx.world_at(model, node, &vb));
    qangle(qmul(quat_of(&b), qconj(quat_of(&a)))).to_degrees() / 2.0
}

/// Is `n` the node `anc` or below it?
fn under(model: &Model, mut n: usize, anc: usize) -> bool {
    for _ in 0..128 {
        if n == anc {
            return true;
        }
        match model.nodes[n].parent {
            Some(p) => n = p,
            None => return false,
        }
    }
    false
}

/// The tilt fraction that sets a bogie level with the gear down: the one
/// where its front and rear wheels reach equally low.
fn level_fraction(model: &Model, idx: &ClipIndex, tilt: usize, gear: Option<usize>) -> f64 {
    let Some(pivot) = main_node(model, idx, tilt) else { return 1.0 };
    let pts: Vec<(usize, V3)> = model
        .meshes
        .iter()
        .filter_map(|m| m.node.filter(|&n| under(model, n, pivot)).map(|n| (n, m)))
        .flat_map(|(n, m)| {
            m.vertices
                .iter()
                .step_by(5)
                .map(move |v| (n, [v.pos[0] as f64, v.pos[1] as f64, v.pos[2] as f64]))
        })
        .collect();
    if pts.is_empty() {
        return 1.0;
    }
    let owners: HashSet<usize> = pts.iter().map(|p| p.0).collect();
    let mut best = (f64::INFINITY, 1.0);
    for k in 0..=50 {
        let f = k as f64 / 50.0;
        let mut vals = at(&[(tilt, f)]);
        if let Some(g) = gear {
            vals.insert(g, 0.5);
        }
        let mats: HashMap<usize, M4> = owners
            .iter()
            .map(|&o| (o, mul(&idx.world_at(model, o, &vals), &inverse(&model.nodes[o].world))))
            .collect();
        let moved: Vec<V3> = pts.iter().map(|(o, p)| apply(&mats[o], *p)).collect();
        let (zmin, zmax) = moved.iter().fold((f64::INFINITY, f64::NEG_INFINITY), |a, p| (a.0.min(p[2]), a.1.max(p[2])));
        let third = (zmax - zmin) / 3.0;
        let (mut front, mut rear) = (f64::INFINITY, f64::INFINITY);
        for p in &moved {
            if p[2] > zmax - third {
                front = front.min(p[1]);
            } else if p[2] < zmin + third {
                rear = rear.min(p[1]);
            }
        }
        let d = (front - rear).abs();
        if d < best.0 {
            best = (d, f);
        }
    }
    best.1
}

/// Which of FlyByWire's own per-side actuator variables (`HYD_AIL_<side>_*`,
/// `HYD_ELEV_<side>_*`) a clip named `l_...`/`r_...` belongs to.
fn side_of(n: &str) -> &'static str {
    if n.starts_with('l') {
        "LEFT"
    } else {
        "RIGHT"
    }
}

/// Which of FlyByWire's three real aileron panels (`a380_aileron_body`:
/// inward, middle, outward) a clip's trailing digit names. MSFS's model has
/// one clip per panel (`l_aileron1/2/3_percent_key`); FlyByWire keeps the
/// same three panels apart until `flight_controls.rs` spans them onto
/// X-Plane's two aileron surfaces for the aerodynamic model, so the panel's
/// own actuator ratio is available and is a closer match for this clip than
/// that spread-out pair would be.
fn aileron_panel(n: &str) -> &'static str {
    match n.chars().filter(char::is_ascii_digit).collect::<String>().parse::<u32>().unwrap_or(1) {
        1 => "INWARD",
        2 => "MIDDLE",
        _ => "OUTWARD",
    }
}

/// `raw` is a FlyByWire actuator ratio in its own convention (0 down/1 up
/// for ailerons and elevators, 0 left/1 right for the rudder); invert it
/// when this clip's own geometry runs the opposite way (`flip`, measured by
/// the caller from the model, not assumed).
fn maybe_flip(raw: String, flip: bool) -> String {
    if flip {
        format!("1 - {raw}")
    } else {
        raw
    }
}

/// Which of the A380's eight spoiler panels a clip's trailing digit names
/// (`l_spoiler1_key`..`l_spoiler8_key`, `r_spoiler1_key`..`r_spoiler8_key`:
/// A380_EXTERIOR.xml's own `SPOILERS_LEFT`/`SPOILERS_RIGHT` components,
/// 1 inboard to 8 at the tip, matching FlyByWire's own F/CTL page,
/// SDv2 FctlPage.tsx:31-51). No digit: defaults to panel 1 rather than
/// panicking, the same convention `aileron_panel` uses above.
fn spoiler_panel(n: &str) -> u32 {
    n.chars().filter(char::is_ascii_digit).collect::<String>().parse::<u32>().unwrap_or(1).clamp(1, 8)
}

/// The Lua expression driving one spoiler panel clip from FlyByWire's own
/// per-panel actuator ratio, scaled exactly as the MSFS model behaviour XML
/// scales the same clip (see `exterior`'s own doc comment on the spoiler
/// match arm for the 50/65 derivation). Clamped: the ratio's own 0..1 is a
/// property of the real actuator's range, not the (larger) authored clip
/// range this scales it into, so nothing here guarantees the result stays
/// inside 0..1 on its own.
fn spoiler_source(n: &str) -> String {
    format!("clamp(rd(\"fbw/A32NX_HYD_SPOILER_{}_{}_DEFLECTION\") * 50 / 65, 0, 1)", spoiler_panel(n), side_of(n))
}

/// The Lua expression driving one of the model's structural-flex clips (the
/// wing's four sections, the engines' wobble, the tailplane and the aft
/// fuselage) from FlyByWire's own flex model (fbw-common structural_flex,
/// which the plugin runs), with the variable and frame formula the model
/// behaviour XML gives the same clip (A380_EXTERIOR.xml `ASOBO_GT_Anim`, 100
/// frames), e.g. `(L:A32NX_WING_FLEX_LEFT_INBOARD, number) 23.25 * 55.35 +`.
fn flex_source(clip: &str) -> Option<String> {
    let wing = |side: &str, section: &str, scale: f64, offset: f64| (format!("WING_FLEX_{side}_{section}"), scale, offset);
    let (var, scale, offset) = match clip {
        "left_inboard_flex" => wing("LEFT", "INBOARD", 23.25, 55.35),
        "left_inner_midboard_flex" => wing("LEFT", "INBOARD_MID", 16.125, 67.0),
        "left_outer_midboard_flex" => wing("LEFT", "OUTBOARD_MID", 29.41, 31.9),
        "left_outboard_flex" => wing("LEFT", "OUTBOARD", 29.41, 50.0),
        "right_inboard_flex" => wing("RIGHT", "INBOARD", 23.25, 55.35),
        "right_inner_midboard_flex" => wing("RIGHT", "INBOARD_MID", 16.125, 67.0),
        "right_outer_midboard_flex" => wing("RIGHT", "OUTBOARD_MID", 29.41, 31.9),
        "right_outboard_flex" => wing("RIGHT", "OUTBOARD", 29.41, 50.0),
        "left_elevator_flex" => ("ELEVATOR_LEFT_WOBBLE_Y_POSITION".into(), 100.0, 0.0),
        "right_elevator_flex" => ("ELEVATOR_RIGHT_WOBBLE_Y_POSITION".into(), 100.0, 0.0),
        "aft_flex" => ("AFT_FLEX_POSITION".into(), 100.0, 0.0),
        n if n.starts_with("eng") && n.ends_with("_wobble") => (format!("ENGINE_{}_WOBBLE_X_POSITION", &n[3..n.len() - 7]), 100.0, 0.0),
        _ => return None,
    };
    Some(format!("clamp((rd(\"fbw/A32NX_{var}\") * {} + {}) / 100, 0, 1)", num(scale), num(offset)))
}

/// The Lua expression driving a flap clip from FlyByWire's own real FPPU
/// animation-position variable (`fbw/A32NX_<side>_FLAPS_ANIMATION_POSITION`,
/// a 0..100 percent MSFS's own exterior model reads directly for this exact
/// clip -- see `exterior`'s own doc comment on the flap match arm).
fn flap_source(side: &str) -> String {
    format!("rd(\"fbw/A32NX_{side}_FLAPS_ANIMATION_POSITION\") / 100")
}

/// The Lua expression driving a slat clip from the real, surface-angle
/// ratio MSFS's own exterior model feeds this clip through the native
/// "LEADING EDGE FLAPS ... PERCENT" SimVar (`fbw/A32NX_<side>_
/// SLATS_POSITION_PERCENT`, a 0..100 percent -- see `exterior`'s own doc
/// comment on the slat match arm for why this is a different FlyByWire
/// variable to the flaps' own `flap_source` above, not just a renamed
/// copy of it).
fn slat_source(side: &str) -> String {
    format!("rd(\"fbw/A32NX_{side}_SLATS_POSITION_PERCENT\") / 100")
}

/// Exterior: datarefs, X-Plane sources and show/hide rules.
pub fn exterior(model: &Model, idx: &ClipIndex, res: Option<&Resolution>) -> Rig {
    let mut rig = Rig::default();
    let mut used = HashSet::new();
    let by_name: HashMap<String, usize> = model
        .clips
        .iter()
        .enumerate()
        .map(|(i, c)| (c.name.trim().to_ascii_lowercase(), i))
        .collect();
    let clip = |n: &str| by_name.get(n).copied();
    let (fwd, aft, up) = ([0.0, 0.0, 1.0], [0.0, 0.0, -1.0], [0.0, 1.0, 0.0]);
    let none = HashMap::new();
    let mut driven = 0;
    // Hoisted out of the clip loop below (it does not depend on `ci`/`c`):
    // gear index -> (its clip's MSFS name, that clip's index if this model
    // has it), also used by `gear_travel_m` just below.
    let gear_of = |g: usize| -> (&str, Option<usize>) {
        match g {
            0 => ("c_gear", clip("c_gear")),
            1 => ("l_b_gear", clip("l_b_gear")),
            2 => ("r_b_gear", clip("r_b_gear")),
            3 => ("l_w_gear", clip("l_w_gear")),
            _ => ("r_w_gear", clip("r_w_gear")),
        }
    };
    // Each gear leg's own strut/tyre compression travel (metres), measured
    // from its own clip rather than assumed: main.lua's `comp()` blends the
    // wheel toward this clip's last keyframe as X-Plane's real deflection
    // (`tire_vertical_deflection_mtr`) rises, so the fraction it feeds that
    // clip must reach 1.0 at the same metres of deflection this clip's own
    // keyframes were authored for. A single hardcoded divisor for every leg
    // (the previous code used half a metre) is only right for a leg whose
    // own travel happens to be half a metre; on the A380 it was close for
    // some legs and not for others, and the acf's own gear-travel figure
    // (acf.rs's `_strut_max_wgt_def`, padded past this model's real travel
    // to stop the nose bottoming out) can send X-Plane's real deflection
    // well past whatever this clip can show at all, which is the other,
    // acf-side half of the tyres sinking into the runway (see acf.rs).
    //
    // The nose's compression is its own clip ("c_gear_comp"; "c_gear" is
    // only the up/down swing, driven separately by `g(DEP, 0)` below); each
    // main leg's compression is the second half of its single combined clip
    // (the `0.5 * g(DEP, i) + 0.5 * comp(i)` match arm below), so its travel
    // is measured over that clip's own second half.
    //
    // Measured with the leg extended: the glTF rest pose has the nose leg
    // retracted (its "c_gear" swing at 0), where its strut lies nearly
    // horizontal and the vertical part of its compression is close to none
    // -- the A380 nose measured 0.05 m (the floor below) there against
    // ~0.48 m once swung down. A main leg's second half is already extended.
    //
    // And for the nose, the largest vertical movement of any node the clip
    // moves, not `main_node`'s: "c_gear_comp" also folds the torque links
    // (NLG_LOWER_LINK1/2) through a bigger angle than the strut slides, so
    // `main_node` picked a link, whose own pivot does not move at all. The
    // mains keep `main_node`: over their combined clip it is the leg
    // itself, which the deployment swing makes the node that moves most.
    let extended: HashMap<usize, f64> = clip("c_gear").map(|c| at(&[(c, 1.0)])).unwrap_or_default();
    let vertical = |n: usize, base: &HashMap<usize, f64>, ci: usize, fa: f64| moved(model, idx, n, [0.0, 0.0, 0.0], base, ci, fa, 1.0)[1].abs();
    let mut gear_travel_m = [None; 5];
    for (g, slot) in gear_travel_m.iter_mut().enumerate() {
        let (comp_clip, fa, base) = if g == 0 { (clip("c_gear_comp"), 0.0, &extended) } else { (gear_of(g).1, 0.5, &none) };
        *slot = comp_clip.and_then(|ci| {
            let travel = if g == 0 {
                idx.nodes_of(model, ci).into_iter().map(|n| vertical(n, base, ci, fa)).reduce(f64::max)?
            } else {
                vertical(main_node(model, idx, ci)?, base, ci, fa)
            };
            // `.max(0.05)`: a degenerate/near-zero measurement (a clip with
            // no real vertical motion) must not become a division that
            // blows the fraction up to a huge multiple for a tiny real
            // deflection -- fall through to a small floor instead of NaN or
            // an absurd ratio.
            Some(travel.max(0.05))
        });
    }
    rig.gear_travel_m = gear_travel_m;

    for (ci, c) in model.clips.iter().enumerate() {
        let lname = c.name.trim().to_ascii_lowercase();
        if idx.nodes_of(model, ci).is_empty() {
            continue;
        }
        let dr = unique(&mut used, format!("fbw/anim/{}", sanitize(&c.name)));
        rig.created.push((dr.clone(), idx.rest_fraction(model, ci)));
        rig.drefs.insert(ci, dr.clone());
        let Some(main) = main_node(model, idx, ci) else { continue };
        // Which end of the clip a hinged surface's trailing edge goes down (y)
        // or right (-x in glTF axes).
        let te = |base: &HashMap<usize, f64>| moved(model, idx, main, aft, base, ci, 0.5, 1.0);
        let down_base = |g: usize| -> HashMap<usize, f64> {
            let (name, c) = gear_of(g);
            c.map(|c| at(&[(c, if name == "c_gear" { 1.0 } else { 0.5 })])).unwrap_or_default()
        };
        let side_gear = |s: &str| -> usize {
            match s {
                "l_b" => 1,
                "r_b" => 2,
                "l_w" => 3,
                _ => 4,
            }
        };
        let expr: Option<String> = match lname.as_str() {
            n if flex_source(n).is_some() => flex_source(n),
            "c_gear" => Some("g(DEP, 0)".into()),
            "c_gear_comp" => Some("comp(0)".into()),
            "l_b_gear" | "r_b_gear" | "l_w_gear" | "r_w_gear" => {
                let i = side_gear(&lname[..3]);
                Some(format!("0.5 * g(DEP, {i}) + 0.5 * comp({i})"))
            }
            n if n.starts_with("c_gear_door") => Some("door(g(DEP, 0))".into()),
            "l_body_gear_door" | "l_blg_inner_door" => Some("door(g(DEP, 1))".into()),
            "r_body_gear_door" | "r_blg_inner_door" => Some("door(g(DEP, 2))".into()),
            "l_wing_gear_door" => Some("door(g(DEP, 3))".into()),
            "r_wing_gear_door" => Some("door(g(DEP, 4))".into()),
            n if n.ends_with("_gear_tilt") => {
                let i = side_gear(&n[..3]);
                let level = level_fraction(model, idx, ci, gear_of(i).1);
                rig.report.push(format!("{}: level on the ground at {:.2}", c.name, level));
                Some(format!("(g(GND, {i}) > 0.5) and {} or 1", num(level)))
            }
            "c_wheel" => {
                let base = down_base(0);
                let half = half_range_deg(model, idx, ci, main, &base).max(1.0);
                let right = moved(model, idx, main, fwd, &base, ci, 0.5, 1.0)[0] < 0.0;
                rig.report.push(format!("nose wheel steering: {half:.0} degrees each way"));
                Some(format!("bi(g(STEER, 0) / {}, {})", num(half), if right { 1 } else { -1 }))
            }
            "left_bws" | "right_bws" => Some("0.5".into()),
            n if n.ends_with("_tire_key") || n.starts_with("c_tire_anim") => {
                let g = match n {
                    "ll_tire_key" | "lr_tire_key" | "b_tire_key" => 1,
                    "rl_tire_key" | "rr_tire_key" => 2,
                    "l_tire_key" => 3,
                    "r_tire_key" => 4,
                    _ => 0,
                };
                let forward = moved(model, idx, main, up, &down_base(g), ci, 0.0, 0.02)[2] > 0.0;
                Some(if forward {
                    format!("(g(TIRE, {g}) % 360) / 360")
                } else {
                    format!("1 - (g(TIRE, {g}) % 360) / 360")
                })
            }
            // FlyByWire's own real surface position, not X-Plane's own
            // flap1_deploy_ratio/slat1_deploy_ratio (which only track the
            // handle's commanded detent, not how far the hydraulics have
            // actually driven the panels): the model behaviour XML
            // (A380_EXTERIOR.xml) drives this exact clip from this exact
            // variable, `<ANIM_CODE>(L:A32NX_LEFT_FLAPS_ANIMATION_POSITION)</
            // ANIM_CODE>`, `<ANIM_LENGTH>100</ANIM_LENGTH>` -- a 0..100
            // percent over the clip -- matching FlyByWire's own
            // `animation_left_id`/`animation_right_id`
            // (fbw-common/systems/src/hydraulic/flap_slat.rs:199-200,
            // written at :579-582 as the raw FPPU/synchro-angle ratio, not
            // through the surface-angle interpolation curve the slats use
            // below).
            "l_flap_percent_key" => Some(flap_source("LEFT")),
            "r_flap_percent_key" => Some(flap_source("RIGHT")),
            // Unlike flaps, MSFS's exterior model does not read a dedicated
            // animation-position L:var for slats: this clip runs through
            // Asobo's own `ASOBO_HANDLING_Slats_Template`
            // (fs-base-aircraft-common/ModelBehaviorDefs/Asobo/Exterior.xml),
            // whose `ANIM_SIMVAR_LEFT`/`_RIGHT` is the native "LEADING EDGE
            // FLAPS LEFT/RIGHT PERCENT" SimVar -- and FlyByWire's own wasm
            // glue (a380_systems_wasm/src/flaps.rs's `SlatsSurface`) fills
            // that native SimVar from `LEFT_SLATS_POSITION_PERCENT`/
            // `RIGHT_SLATS_POSITION_PERCENT`, not from
            // `LEFT_SLATS_ANIMATION_POSITION`. Those `..._POSITION_PERCENT`
            // variables are `SecondarySurface`'s own averaged,
            // surface-angle-interpolated ratio (fbw-common
            // hydraulic/flap_slat.rs:96-97, written at :129-130) -- a
            // different curve to the flaps' raw FPPU ratio above, so slats
            // and flaps deliberately read two different real variables here,
            // exactly as MSFS does.
            "l_slat_percent_key" => Some(slat_source("LEFT")),
            "r_slat_percent_key" => Some(slat_source("RIGHT")),
            n if n.contains("aileron") => {
                // FlyByWire's own actuator ratio for this side and panel
                // (`HYD_AIL_<side>_<panel>_DEFLECTION`, published by the
                // plugin as `fbw/A32NX_HYD_AIL_<side>_<panel>_DEFLECTION`;
                // 0 is down, 1 is up: flight_controls.rs's doc comment,
                // a380_systems ailerons.rs:151-162), not X-Plane's raw
                // stick ratio (`total_roll_ratio`): under autopilot or the
                // protections the surface can move more or less than the
                // stick shows, and the drawn surface should show what
                // FlyByWire actually commanded, the same thing
                // `flight_controls.rs` is writing to `sim/flightmodel2/
                // wing/aileron*_deg` this same tick.
                let te_down = te(&none)[1] < 0.0;
                let raw = format!("rd(\"fbw/A32NX_HYD_AIL_{}_{}_DEFLECTION\")", side_of(n), aileron_panel(n));
                Some(maybe_flip(raw, te_down))
            }
            n if n.contains("elevator_deflection") => {
                // Same idea: FlyByWire's own per-side, per-panel elevator
                // ratio (`HYD_ELEV_<side>_INWARD/OUTWARD_DEFLECTION`), which
                // MSFS's two elevator clips per side already match 1:1.
                let te_down = te(&none)[1] > 0.0;
                let panel = if n.contains("inner") { "INWARD" } else { "OUTWARD" };
                let raw = format!("rd(\"fbw/A32NX_HYD_ELEV_{}_{}_DEFLECTION\")", side_of(n), panel);
                Some(maybe_flip(raw, te_down))
            }
            "trimtab_elevator_key" => {
                // Already faithful, left alone: `flight_controls.rs::update`
                // writes FlyByWire's real commanded trim ratio straight into
                // X-Plane's own `sim/cockpit2/controls/elevator_trim` every
                // tick (from `HYD_FINAL_THS_DEFLECTION` via `trim_ratio`,
                // under `override_control_surfaces`) -- unlike
                // `total_roll/pitch/heading_ratio`, X-Plane's trim input is
                // one of the two datarefs the plugin overrides, so `TRIM`
                // here already is FlyByWire's real stabiliser deflection,
                // not the pilot's own trim wheel.
                let k = if te(&none)[1] > 0.0 { 1 } else { -1 };
                Some(format!("bi(TRIM, {k})"))
            }
            n if n.contains("rudder") => {
                // FlyByWire's own per-panel rudder ratio (`HYD_UPPER/LOWER_
                // RUD_DEFLECTION`, 0 left/1 right: rudder.rs:296-308), which
                // MSFS's two rudder clips (upper, lower) already match 1:1.
                let te_right = te(&none)[0] < 0.0;
                let panel = if n.contains("upper") { "UPPER" } else { "LOWER" };
                let raw = format!("rd(\"fbw/A32NX_HYD_{panel}_RUD_DEFLECTION\")");
                Some(maybe_flip(raw, !te_right))
            }
            // FlyByWire's own per-panel spoiler actuator ratio
            // (`A32NX_HYD_SPOILER_<k>_<side>_DEFLECTION`, published by the
            // plugin as `fbw/A32NX_HYD_SPOILER_<k>_<side>_DEFLECTION`;
            // written by the SECs, prim.rs's `update_sec`/`update_prim`
            // spoiler closures, from a380_systems' hydraulic/mod.rs
            // actuators, mod.rs:6840-6843), not X-Plane's own
            // speedbrake_ratio and roll input, which never reflects a jam,
            // a failed panel or an override the SECs actually commanded.
            // Its own convention (0 stowed .. 1 at the real 50-degree max:
            // flight_controls.rs's doc comment, "Spoilers: request = surface
            // degrees / 50") matches this clip's own 0 stowed .. 1 direction
            // directly, the same as `SB` did before -- the model behaviour
            // XML's own scale (A380_EXTERIOR.xml, FBW_Spoiler_Surface_
            // Template: "#DEFLECTION_CODE# 100 * 50 * 65 /") is mirrored
            // here: the glTF clip is authored to 65 degrees of travel, but
            // the real actuator only ever reaches 50, so the ratio is
            // rescaled by 50/65 to land at the same fraction of the clip
            // MSFS does, rather than overshooting it.
            n if n.contains("spoiler") => Some(spoiler_source(n)),
            n if n.starts_with("n1_") && n.ends_with("_anim") => {
                let e: usize = n[3..n.len() - 5].parse().unwrap_or(1);
                Some(format!("FAN[{e}] / 360"))
            }
            "thrust_rev_1" => Some("g(REV, 1)".into()),
            "thrust_rev_2" => Some("g(REV, 2)".into()),
            // The RAT: created but never set before (frozen at its rest
            // pose). FlyByWire's own RAT publishes these three under the
            // plugin's `fbw/A32NX_RAT_*` convention (ram_air_turbine.rs,
            // wind_turbine/mod.rs); MSFS's clips happen to be named after
            // the same variables.
            "a32nx_hyd_rat_stow_position" => Some("rd(\"fbw/A32NX_RAT_STOW_POSITION\")".into()),
            "a32nx_hyd_rat_propeller_angle" => Some("rd(\"fbw/A32NX_RAT_PROPELLER_ANGLE\")".into()),
            // `RAT_RPM` is a real RPM (up to ~4300), not a 0..1 ratio: like
            // the engines' `N1_x_anim`, it drives a continuous spin, not a
            // deploy/pitch clip, so it accumulates into `RAT_SPIN` (see
            // `main_lua`) the same way `FAN` does for `N1_x_anim`, with an
            // exact rpm-to-degrees-per-second conversion (RPM * 6).
            "a32nx_hyd_rat_rpm" => Some("RAT_SPIN / 360".into()),
            // No travel to measure a blur onset from (there is nothing to
            // move -- it is a blend between a sharp and a blurred texture),
            // so this is a judgement call pending a look in the sim: fully
            // blurred by 1000 rpm, well under the ~3400+ rpm the governor
            // holds the RAT at once deployed and unloaded (`RPM_GOVERNOR_
            // BREAKPTS`, ram_air_turbine.rs), so it is not stuck part-blurred
            // in the case that matters.
            "rat_blur" => Some("clamp(rd(\"fbw/A32NX_RAT_RPM\") / 1000, 0, 1)".into()),
            // The four outflow valves: created but never set before. The
            // emergency pressurisation partition always transmits the real
            // open amount here regardless of manual/auto mode ("We always
            // transmit the outflow valve open amount",
            // outflow_valve_control_module.rs EppEmergencySignals::write),
            // unlike its other, emergency-only fields, so it tracks the
            // valve continuously; the ARINC auto-mode word the SD page
            // prefers when valid (`PRESS_OUTFLOW_VALVE_<n>_OPEN_PERCENTAGE_
            // B<system>`) is packed SSM+value and not worth decoding in Lua
            // for an animation. Open direction (clip 1 = open) is assumed,
            // unverified -- these valves sit flush in the aft fuselage skin
            // and are rarely visible; flip with `1 -` if backwards in the sim.
            n if n.starts_with("outflow_valve_") => {
                let id: u32 = n.trim_start_matches("outflow_valve_").parse().unwrap_or(1);
                Some(format!("clamp(rd(\"fbw/A32NX_PRESS_MAN_OUTFLOW_VALVE_{id}_OPEN_PERCENTAGE\") / 100, 0, 1)"))
            }
            _ => None,
        };
        if let Some(e) = expr {
            rig.update.push(format!("set(D[\"{dr}\"], {e})"));
            driven += 1;
        }
    }

    // Show and hide: fan blades still or blurred by N1, chocks and cones on the
    // ground only, the RAT's blur disc while stowed.
    let names = node_names(model);
    let add_vis = |rig: &mut Rig, node: &str, line: String| {
        if let Some(&n) = names.get(&node.to_ascii_lowercase()) {
            rig.vis.entry(n).or_default().push(line);
        }
    };
    for e in 1..=4 {
        let dr = format!("fbw/anim/N1_pct_{e}");
        rig.created.push((dr.clone(), 0.0));
        rig.update.push(format!("set(D[\"{dr}\"], g(N1, {}))", e - 1));
        let bands = [(-1000.0, 10.0), (10.0, 30.0), (30.0, 50.0), (50.0, 75.0), (75.0, 1000.0)];
        for (k, (lo, hi)) in bands.iter().enumerate() {
            let parts = if k == 0 {
                vec![format!("FAN_BLADE_STILL_{e}"), format!("SPINNER_STILL_{e}")]
            } else {
                vec![format!("FAN_BLADE_BLUR{k}_{e}"), format!("SPINNER_BLUR{k}_{e}")]
            };
            for p in parts {
                if *lo > -1000.0 {
                    add_vis(&mut rig, &p, format!("ANIM_hide -1000 {} {dr}", num(lo - 0.001)));
                }
                if *hi < 1000.0 {
                    add_vis(&mut rig, &p, format!("ANIM_hide {} 1000 {dr}", num(*hi)));
                }
            }
        }
    }
    rig.created.push(("fbw/anim/ground_equipment".into(), 0.0));
    rig.update.push(
        "set(D[\"fbw/anim/ground_equipment\"], (g(GND, 0) > 0.5 and PBRK > 0.5 and g(N1, 0) < 5 and g(N1, 1) < 5 and g(N1, 2) < 5 and g(N1, 3) < 5) and 1 or 0)"
            .into(),
    );
    for n in [
        "CHOCKS_NOSE_GEAR",
        "CHOCKS_MAIN_GEAR_RIGHT",
        "CHOCKS_MAIN_GEAR_LEFT",
        "CONE_B",
        "CONE_R",
        "CONE_L",
        "CONE_ENG_L",
        "CONE_ENG_R",
    ] {
        add_vis(&mut rig, n, "ANIM_hide -1 0.5 fbw/anim/ground_equipment".into());
    }
    // (The model has a "rat_blur" clip too, so this one needs its own name.)
    // Same blur ratio as the "rat_blur" clip above (kept identical on
    // purpose: this one only gates the blurred-disc mesh's visibility at
    // 0.5, not a texture blend, but they should agree on when the RAT looks
    // blurred).
    let rat = unique(&mut used, "fbw/anim/rat_blur_disc".into());
    rig.created.push((rat.clone(), 0.0));
    rig.update.push(format!("set(D[\"{rat}\"], clamp(rd(\"fbw/A32NX_RAT_RPM\") / 1000, 0, 1))"));
    add_vis(&mut rig, "a380_exterior_rat_blur", format!("ANIM_hide -1 0.5 {rat}"));

    // Manipulators for behaviour-XML controls whose node lives on THIS
    // model's own geometry. `cockpit`'s own pass over the same
    // `Resolution` (below, in `cockpit`) emits every bound control's SASL
    // click command blind to which model's geometry it lands on -- for a
    // control resolved only because a variable *elsewhere* animates it,
    // that is correct even with no geometry here at all. But when the node
    // genuinely lives on the exterior model -- the 16 passenger-door
    // handles (`PAX_DOOR_*_HANDLE`) are the case that motivated this;
    // `cockpit`'s own `names.get(&b.node...)` never finds them, since they
    // are not in the cockpit glTF, so their click command already exists in
    // main.lua but nothing ever gave it a click spot to fire from -- give
    // it one here.
    for (n, m) in manips_on(&names, res) {
        rig.node_manips.entry(n).or_insert(m);
    }
    let click_targets = rig.node_manips.len();

    rig.report.insert(
        0,
        format!(
            "exterior: {} clips animated, {driven} following X-Plane and FlyByWire (flex clips included), {click_targets} behaviour-XML controls found a click spot on this model's own geometry",
            rig.drefs.len()
        ),
    );
    rig
}

/// Manipulators for behaviour-XML controls whose node is one of `names`
/// (matched the same case-insensitive, trimmed way `cockpit` matches its
/// own), skipping a node more than one control sits on (a panel, not a
/// control -- mirrors `cockpit`'s own `own_node`, computed the same way
/// over the whole aircraft's bindings so a node shared with a control
/// resolved against a *different* model still counts as shared here).
fn manips_on(names: &HashMap<String, usize>, res: Option<&Resolution>) -> HashMap<usize, (u8, String)> {
    let mut out = HashMap::new();
    let Some(r) = res else { return out };
    let mut per_node: HashMap<String, usize> = HashMap::new();
    for n in r.bindings.iter().map(|b| &b.node).chain(r.unresolved.iter().map(|u| &u.node)) {
        *per_node.entry(n.trim().to_ascii_lowercase()).or_default() += 1;
    }
    for b in &r.bindings {
        let key = b.node.trim().to_ascii_lowercase();
        if per_node.get(&key) != Some(&1) {
            continue;
        }
        let Some(&n) = names.get(&key) else { continue };
        let san = sanitize(&b.anim);
        let tip = b.anim.trim().replace('_', " ");
        out.entry(n).or_insert((20, bound_manip(b, &san, &tip)));
    }
    out
}

fn describe(c: &Click) -> String {
    match c {
        Click::Toggle { dref, on, off } => format!("toggles {dref} between {} and {}", num(*on), num(*off)),
        Click::Hold { dref, down, up } => format!("holds {dref} at {} while pressed ({} released)", num(*down), num(*up)),
        Click::Axis { dref, v0, v1, step, .. } => format!("steps {dref} from {} to {} by {}", num(*v0), num(*v1), num(*step)),
        Click::Script(sc) => {
            let mut w: Vec<&str> = Vec::new();
            if sc.press.is_some() {
                w.push("click command");
            }
            if sc.up.is_some() {
                w.push("up/down commands");
            }
            format!("{} running the control's MSFS code in SASL", w.join(" and "))
        }
        Click::Command { cmd } => format!("fires {cmd}"),
        Click::CommandKnob { up, down } => format!("fires {up} / {down}"),
    }
}

fn k_priority(k: Kind) -> u8 {
    match k {
        Kind::Push => 1,
        Kind::Drag => 2,
        Kind::Toggle => 3,
        Kind::Switch => 4,
        Kind::Knob => 5,
        Kind::Fixed => 0,
    }
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum Kind {
    Push,
    Knob,
    Switch,
    Toggle,
    Drag,
    Fixed,
}

fn kind(name: &str) -> Kind {
    let u = name.trim().to_ascii_uppercase();
    if u.ends_with("_PUSH_ANIM") {
        Kind::Push
    } else if u.contains("KNOB") || u.contains("_ROTATE") {
        Kind::Knob
    } else if u.starts_with("PUSH") || u.starts_with("ANIM_PUSH") || u.starts_with("ATHR") || u.starts_with("LOCK") || u.contains("_LOCK") {
        Kind::Push
    } else if u.contains("SWITCH") {
        Kind::Switch
    } else if u.starts_with("CB_") || u.contains("COVER") || u.contains("GUARD") {
        Kind::Toggle
    } else if u.starts_with("INSTRUMENT") || u.contains("INDICATOR") || u.starts_with("PRESS_ARC") || u.starts_with("ACCU") || u.ends_with("_CLICK") || u.starts_with("HANDLING_WIPER") {
        Kind::Fixed
    } else {
        Kind::Drag
    }
}

/// The X-Plane manipulator for a resolved control, and whether it is a
/// command handled in Lua.
fn bound_manip(b: &bind::Binding, san: &str, tip: &str) -> String {
    let n = num;
    match &b.click {
        Click::Toggle { dref, on, off } => format!("ATTR_manip_toggle hand {} {} {dref} {tip}", n(*on), n(*off)),
        Click::Hold { dref, down, up } => format!("ATTR_manip_push hand {} {} {dref} {tip}", n(*down), n(*up)),
        Click::Axis { dref, v0, v1, step, knob, horizontal } => {
            let (lo, hi) = (v0.min(*v1), v0.max(*v1));
            let (kind, cursor) = match (knob, horizontal) {
                (true, _) => ("axis_knob", "rotate_medium"),
                (false, true) => ("axis_switch_left_right", "hand"),
                (false, false) => ("axis_switch_up_down", "hand"),
            };
            format!("ATTR_manip_{kind} {cursor} {} {} {s} {s} {dref} {tip}\nATTR_manip_wheel {s}", n(lo), n(hi), s = n(*step))
        }
        Click::Command { cmd } => format!("ATTR_manip_command hand {cmd} {tip}"),
        Click::CommandKnob { up, down } => format!("ATTR_manip_command_knob rotate_medium {up} {down} {tip}"),
        Click::Script(sc) => {
            if sc.up.is_some() {
                let (kind, cursor) = match (sc.knob, sc.horizontal) {
                    (true, _) => ("command_knob", "rotate_medium"),
                    (false, true) => ("command_switch_left_right", "hand"),
                    (false, false) => ("command_switch_up_down", "hand"),
                };
                format!("ATTR_manip_{kind} {cursor} fbw/cockpit/{san}_up fbw/cockpit/{san}_down {tip}")
            } else {
                format!("ATTR_manip_command hand fbw/cockpit/{san}_click {tip}")
            }
        }
    }
}

/// Lua defining a resolved control's commands and animation source.
fn bound_lua(b: &bind::Binding, san: &str, clip_dref: Option<&str>, up: f64) -> String {
    let mut s = format!("do -- {} ({})\n", b.anim, b.template);
    let func = |name: &str, body: &str| format!("local function {name}()\n{body}\nend\n");
    if let Click::Script(sc) = &b.click {
        let press_anim = |v: f64| clip_dref.filter(|_| b.look.is_none()).map(|d| format!("set(D[\"{d}\"], {}) ", num(v))).unwrap_or_default();
        if let Some(p) = &sc.press {
            s.push_str(&func("press", p));
            s.push_str(&func("release", sc.release.as_deref().unwrap_or("")));
            s.push_str(&format!(
                "command(\"fbw/cockpit/{san}_click\", \"{}\", function() {}press() end, function() {}release() end)\n",
                b.anim,
                press_anim(1.0 - up),
                press_anim(up)
            ));
        }
        if let (Some(u), Some(d)) = (&sc.up, &sc.down) {
            s.push_str(&func("up", u));
            s.push_str(&func("down", d));
            s.push_str(&func("up_release", sc.up_release.as_deref().unwrap_or("")));
            s.push_str(&func("down_release", sc.down_release.as_deref().unwrap_or("")));
            s.push_str(&format!("command(\"fbw/cockpit/{san}_up\", \"{} up\", up, up_release)\n", b.anim));
            s.push_str(&format!("command(\"fbw/cockpit/{san}_down\", \"{} down\", down, down_release)\n", b.anim));
        }
    }
    if let (Some(look), Some(d)) = (&b.look, clip_dref) {
        // A native X-Plane axis manipulator (ATTR_manip_axis_*, Click::Axis)
        // writes the systems dataref directly every frame while the user
        // drags or rotates it; this clip's own dataref (what the geometry
        // actually animates from) only tracks that through the look below,
        // so it must keep up at the same rate or the part visibly lags the
        // mouse mid-drag. Every other click (toggle, push, a script's
        // discrete up/down/press/release) changes state once per click, so
        // look_slow (20 Hz, see main_lua) is smooth enough for it.
        let f = if matches!(b.click, Click::Axis { .. }) { "look" } else { "look_slow" };
        s.push_str(&format!("{f}(\"{d}\", function()\n{look}\nend)\n"));
    }
    s.push_str("end\n");
    s
}

/// Cockpit: a dataref and manipulator per clip, latching legends. With
/// FlyByWire's behaviours resolved, clicks act on the systems.
pub fn cockpit(model: &Model, idx: &ClipIndex, res: Option<&Resolution>) -> Rig {
    let mut rig = Rig::default();
    let mut used = HashSet::new();
    let names = node_names(model);
    // Clips whose animation follows a variable other controls set: moved by
    // the systems, not dragged.
    let mirrored: HashMap<String, &String> = res.map(|r| r.mirrors.iter().map(|(a, l)| (a.trim().to_ascii_lowercase(), l)).collect()).unwrap_or_default();
    let mut bound = 0;
    let mut clip_manips: HashMap<String, (u8, String)> = HashMap::new();

    // Centroid of all geometry hanging from each node.
    let mut sums: HashMap<usize, (V3, f64)> = HashMap::new();
    for m in &model.meshes {
        let Some(n0) = m.node else { continue };
        let mut s = [0.0; 3];
        for v in &m.vertices {
            for k in 0..3 {
                s[k] += v.pos[k] as f64;
            }
        }
        let c = m.vertices.len() as f64;
        let mut n = Some(n0);
        for _ in 0..128 {
            let Some(k) = n else { break };
            let e = sums.entry(k).or_insert(([0.0; 3], 0.0));
            for j in 0..3 {
                e.0[j] += s[j];
            }
            e.1 += c;
            n = model.nodes[k].parent;
        }
    }

    let mut counts: HashMap<&str, usize> = HashMap::new();
    for (ci, c) in model.clips.iter().enumerate() {
        if idx.nodes_of(model, ci).is_empty() {
            continue;
        }
        let san = sanitize(&c.name);
        let dr = unique(&mut used, format!("fbw/cockpit/{san}"));
        let rest = idx.rest_fraction(model, ci);
        rig.created.push((dr.clone(), rest));
        rig.drefs.insert(ci, dr.clone());
        let Some(main) = main_node(model, idx, ci) else { continue };
        let tip = c.name.trim().replace('_', " ");
        let keys = idx.key_fractions(model, ci).len().max(2);
        let up = if rest < 0.5 { 0.0 } else { 1.0 };
        let mut k = kind(&c.name);
        let centroid = sums.get(&main).filter(|s| s.1 > 0.0).map(|s| s.0.map(|x| x / s.1));
        let drag = centroid.map(|p| {
            let a = idx.carry(model, main, p, &at(&[(ci, 0.0)]));
            let b = idx.carry(model, main, p, &at(&[(ci, 1.0)]));
            xp(sub(b, a))
        });
        if k == Kind::Drag && drag.is_none_or(|d| norm(d) < 0.005) {
            k = Kind::Toggle;
        }
        let manip = match k {
            Kind::Push => Some((1, format!("ATTR_manip_push hand {} {} {dr} {tip}", num(1.0 - up), num(up)))),
            Kind::Toggle => Some((3, format!("ATTR_manip_toggle hand {} {} {dr} {tip}", num(1.0 - up), num(up)))),
            Kind::Knob => {
                let sweep = sweep_deg(model, idx, ci, main);
                let step = if (3..=12).contains(&keys) && sweep < 300.0 { 1.0 / (keys - 1) as f64 } else { 0.05 };
                Some((
                    5,
                    format!(
                        "ATTR_manip_axis_knob rotate_medium 0 1 {s} {s} {dr} {tip}\nATTR_manip_wheel {s}",
                        s = num(step)
                    ),
                ))
            }
            Kind::Switch => {
                let step = 1.0 / (keys - 1) as f64;
                Some((
                    4,
                    format!(
                        "ATTR_manip_axis_switch_up_down hand 0 1 {s} {s} {dr} {tip}\nATTR_manip_wheel {s}",
                        s = num(step)
                    ),
                ))
            }
            Kind::Drag => {
                let d = drag.unwrap_or([0.0; 3]);
                Some((2, format!("ATTR_manip_drag_axis hand {} {} {} 0 1 {dr} {tip}", num(d[0]), num(d[1]), num(d[2]))))
            }
            Kind::Fixed => None,
        };
        let binding = res.and_then(|r| r.binding(&c.name));
        let lname = c.name.trim().to_ascii_lowercase();
        let manip = match binding {
            Some(b) => {
                bound += 1;
                rig.targets.extend(b.targets.iter().cloned());
                rig.targets.extend(b.reads.iter().cloned());
                rig.lua.push(bound_lua(b, &san, Some(&dr), up));
                Some((k_priority(k) + 10, bound_manip(b, &san, &tip)))
            }
            None if mirrored.contains_key(&lname) => {
                let look = mirrored[&lname];
                // No manipulator of its own (a mask, a door, a seat
                // following what a bound control sets, see
                // Resolution::mirrors): nothing drags this clip directly,
                // so look_slow (20 Hz, see main_lua) is as smooth to the eye
                // as every frame, at a fraction of the cost.
                rig.lua.push(format!("look_slow(\"{dr}\", function()\n{look}\nend)\n"));
                None
            }
            None => manip,
        };
        if let Some(m) = &manip {
            clip_manips.insert(lname.clone(), m.clone());
        }
        *counts
            .entry(match k {
                Kind::Push => "pushbuttons",
                Kind::Knob => "knobs",
                Kind::Switch => "switches",
                Kind::Toggle => "guards, breakers and toggles",
                Kind::Drag => "levers and handles",
                Kind::Fixed => "gauges (not clickable)",
            })
            .or_default() += 1;
        if let Some(m) = manip {
            rig.manips.insert(ci, m);
        }

    }

    // The passenger cabin is drawn only from inside, as in MSFS: seen through
    // the windows from outside it pokes through the skin, and it is most of
    // the model's triangles.
    // Hidden from inside too unless fbw/cockpit/show_cabin is set: it is
    // most of the model's triangles, all behind the cockpit door.
    if let Some(&i) = names.get("a380_cabin") {
        rig.vis.entry(i).or_default().push("ANIM_hide 0.5 1.5 sim/graphics/view/view_is_external".into());
        rig.vis.entry(i).or_default().push("ANIM_hide -1 0.5 fbw/cockpit/show_cabin".into());
        rig.created.push(("fbw/cockpit/show_cabin".into(), 0.0));
    }

    // Parts MSFS keeps hidden.
    rig.created.push(("fbw/cockpit/hidden_parts".into(), 0.0));
    for n in ["lights_overhead", "hose0815"] {
        if let Some(&i) = names.get(n) {
            rig.vis.entry(i).or_default().push("ANIM_hide -1 0.5 fbw/cockpit/hidden_parts".into());
        }
    }

    // Weather radar tilt and gain knobs: real nodes in the cockpit glTF
    // (KNOB_CPT/FO_WXR_ELEV, KNOB_CPT/FO_WXR_GAIN), but with no animation
    // clip of their own, unlike their neighbour KNOB_CPT/FO_WXR_VD_AZIM
    // (which does have a clip and so already gets a manipulator from the
    // per-clip loop above). FBW_AIRBUS_WeatherRadar_Template -- the
    // template that would normally drive and bind them, per
    // pedestal.xml's <UseTemplate Name="FBW_AIRBUS_WeatherRadar_Template">
    // -- is Asobo-SDK-only and not in FBW's own source tree, so neither the
    // clip loop nor the XML resolution above ever reaches these nodes. Give
    // each its own rotate manipulator by node name, the same node-name
    // fallback this file already uses for the cabin and hidden-parts
    // special cases, publishing a fresh dataref each.
    for (node, dref) in [
        ("KNOB_CPT_WXR_ELEV", "fbw/cockpit/KNOB_RADAR_TILT"),
        ("KNOB_CPT_WXR_GAIN", "fbw/cockpit/KNOB_RADAR_GAIN"),
        ("KNOB_FO_WXR_ELEV", "fbw/cockpit/KNOB_RADAR_TILT_FO"),
        ("KNOB_FO_WXR_GAIN", "fbw/cockpit/KNOB_RADAR_GAIN_FO"),
    ] {
        if let Some(&i) = names.get(&node.to_ascii_lowercase()) {
            let dref = unique(&mut used, dref.to_string());
            rig.created.push((dref.clone(), 0.5));
            let tip = node.replace('_', " ");
            rig.node_manips.entry(i).or_insert((
                20,
                format!("ATTR_manip_axis_knob rotate_medium 0 1 0.02 0.02 {dref} {tip}\nATTR_manip_wheel 0.02"),
            ));
        }
    }

    // Manipulators on the nodes the XML makes clickable, and a line per XML
    // control for the report.
    if let Some(r) = res {
        // A node several controls sit on is a panel, not a control: those
        // keep their clips' manipulators.
        let mut per_node: HashMap<String, usize> = HashMap::new();
        for n in r.bindings.iter().map(|b| &b.node).chain(r.unresolved.iter().map(|u| &u.node)) {
            *per_node.entry(n.trim().to_ascii_lowercase()).or_default() += 1;
        }
        let own_node = |n: &str| per_node.get(&n.trim().to_ascii_lowercase()).is_some_and(|&c| c == 1);
        let mut orphaned = 0;
        for b in &r.bindings {
            let san = sanitize(&b.anim);
            let tip = b.anim.trim().replace('_', " ");
            let has_clip = model.clips.iter().any(|c| c.name.trim().eq_ignore_ascii_case(b.anim.trim()));
            if !has_clip {
                // Clicked here, animated elsewhere (through a variable).
                rig.lua.push(bound_lua(b, &san, None, 0.0));
                rig.targets.extend(b.targets.iter().cloned());
                rig.targets.extend(b.reads.iter().cloned());
            }
            let node_manip = names.get(&b.node.trim().to_ascii_lowercase()).filter(|_| own_node(&b.node));
            if let Some(&n) = node_manip {
                rig.node_manips.insert(n, (20, bound_manip(b, &san, &tip)));
            }
            let fires = if b.commands.is_empty() || !matches!(b.click, Click::Script(_)) {
                String::new()
            } else {
                format!(" (fires {})", b.commands.iter().cloned().collect::<Vec<_>>().join(", "))
            };
            let unreachable = control_has_no_geometry(has_clip, node_manip.is_some());
            if unreachable {
                orphaned += 1;
            }
            rig.bindings.push(format!("{}: {}{fires}{}", b.anim, describe(&b.click), if unreachable { " [UNREACHABLE: no clip and no node of this name in the model]" } else { "" }));
        }
        for u in &r.unresolved {
            let own = clip_manips.get(&u.anim.trim().to_ascii_lowercase());
            if let (Some(&n), Some(m)) = (names.get(&u.node.trim().to_ascii_lowercase()).filter(|_| own_node(&u.node)), own) {
                rig.node_manips.entry(n).or_insert((15, m.1.clone()));
            }
            rig.bindings.push(format!("{}: unresolved ({}), keeps fbw/cockpit/{}: {}", u.anim, u.template, sanitize(&u.anim), u.reason));
        }
        for l in &r.levers {
            rig.bindings.push(format!("{l}: lever, keeps fbw/cockpit/{}", sanitize(l)));
        }
        for (a, _) in &r.mirrors {
            rig.bindings.push(format!("{a}: animation follows the systems"));
        }
        for m in &r.mirror_reads {
            rig.targets.insert(m.clone());
        }
        rig.report.push(format!(
            "cockpit behaviour: {} controls in FlyByWire's XML: {} bound to the systems ({bound} on animated parts), {} unresolved, {} levers; {} animations follow the systems",
            r.bindings.len() + r.unresolved.len() + r.levers.len(),
            r.bindings.len(),
            r.unresolved.len(),
            r.levers.len(),
            r.mirrors.len()
        ));
        if orphaned > 0 {
            // Bound (has working SASL state/click logic) but neither an
            // animation clip nor a fallback node of the same name exists in
            // the converted model: nothing in the cockpit can ever trigger
            // or show it (e.g. LOCK_OVHD_HYD_RATMANON / PUSH_OVHD_HYD_RATMANON,
            // LOCK_OVHD_EMERELECPWR_EMERTEST / PUSH_OVHD_EMERELECPWR_EMERTEST:
            // no geometry for these two switches anywhere in the model).
            rig.report.push(format!("cockpit behaviour: {orphaned} bound but unreachable: no clip and no node of that name in the model (see [UNREACHABLE] lines above)"));
        }
        let mut why: BTreeMap<String, usize> = BTreeMap::new();
        for u in &r.unresolved {
            *why.entry(bind::reason_group(&u.reason)).or_default() += 1;
        }
        for (k, v) in why {
            rig.report.push(format!("cockpit behaviour: {v} unresolved: {k}"));
        }
    }

    // The cockpit's own datarefs the bound controls use (covers without a
    // variable, input events).
    let mut own: BTreeSet<String> = BTreeSet::new();
    if let Some(r) = res {
        for b in &r.bindings {
            own.extend(b.targets.iter().chain(&b.reads).filter(|d| d.starts_with("fbw/cockpit/")).cloned());
        }
    }
    for d in own {
        if !rig.created.iter().any(|(n, _)| n == &d) {
            rig.created.push((d.clone(), 0.0));
        }
        rig.targets.remove(&d);
    }

    if let Some(lights) = res.and_then(|r| r.lights.as_ref()) {
        cockpit_lights(model, &names, lights, &mut rig);
    }

    // The components' own update codes (variable mappings, synchronised
    // switches), run by SASL at their frequency.
    if let Some(r) = res {
        let mut why: BTreeMap<String, usize> = BTreeMap::new();
        for u in &r.updates {
            let what = format!("{} ({})", if u.node.is_empty() { "-" } else { &u.node }, u.template);
            match &u.lua {
                Ok(body) => {
                    rig.targets.extend(u.datarefs.iter().cloned());
                    rig.updates.push((body.clone(), u.frequency, u.once, what));
                }
                Err(e) => {
                    *why.entry(bind::reason_group(e)).or_default() += 1;
                    rig.bindings.push(format!("update code of {what}: not run: {e}"));
                }
            }
        }
        rig.report.push(format!("cockpit update codes: {} in the XML, {} run in SASL, {} not run", r.updates.len(), rig.updates.len(), r.updates.len() - rig.updates.len()));
        for (w, n) in why {
            rig.report.push(format!("cockpit update codes: {n} not run: {w}"));
        }
    }

    let mut parts: Vec<String> = counts.iter().map(|(k, v)| format!("{v} {k}")).collect();
    parts.sort();
    rig.report.push(format!("cockpit: {} clips animated ({})", rig.drefs.len(), parts.join(", ")));
    rig
}

/// The lights of the cockpit's nodes, from their emissive and visibility
/// codes: light levels and hide rules per node, SASL for codes X-Plane
/// cannot express, and a count of what was found.
fn cockpit_lights(model: &Model, names: &HashMap<String, usize>, lights: &crate::behaviour::emissive::Lights, rig: &mut Rig) {
    let mut helpers: HashMap<usize, Vec<String>> = HashMap::new();
    let (mut direct, mut lua, mut constant, mut unresolved, mut absent) = ([0usize; 2], [0usize; 2], [0usize; 2], [0usize; 2], [0usize; 2]);
    let mut why: BTreeMap<String, usize> = BTreeMap::new();
    let mut used = HashSet::new();
    for l in &lights.lights {
        let k = if l.kind == LightKind::Emissive { 0 } else { 1 };
        let Some(&n) = names.get(&l.node.to_ascii_lowercase()) else {
            absent[k] += 1;
            continue;
        };
        let drive = match &l.drive {
            Ok(d) => d,
            Err(e) => {
                unresolved[k] += 1;
                *why.entry(bind::reason_group(e)).or_default() += 1;
                let code: String = l.code.split_whitespace().collect::<Vec<_>>().join(" ").chars().take(120).collect();
                rig.bindings.push(format!("{} ({}): {} code unresolved: {e} [{code}]", l.node, l.template, if k == 0 { "emissive" } else { "visibility" }));
                continue;
            }
        };
        let san = sanitize(&l.node);
        let helper = |rig: &mut Rig, used: &mut HashSet<String>, helpers: &mut HashMap<usize, Vec<String>>, i: usize, prefix: &str| {
            let d = unique(used, format!("fbw/cockpit/{prefix}/{san}"));
            rig.created.push((d.clone(), 0.0));
            helpers.entry(i).or_default().push(d.clone());
            d
        };
        rig.targets.extend(l.reads.iter().cloned());
        match (l.kind, drive) {
            (LightKind::Emissive, Drive::Const(c)) => {
                constant[k] += 1;
                rig.light_driven.insert(n, *c != 0.0);
            }
            (LightKind::Emissive, Drive::Direct { dref, v1, v2 }) => {
                direct[k] += 1;
                rig.light_driven.insert(n, true);
                rig.light_levels.insert(n, format!("{} {} {dref}", num(*v1), num(*v2)));
            }
            (LightKind::Emissive, Drive::Lua(i)) => {
                lua[k] += 1;
                let d = helper(rig, &mut used, &mut helpers, *i, "lt");
                rig.light_driven.insert(n, true);
                rig.light_levels.insert(n, format!("0 1 {d}"));
            }
            (LightKind::Visibility, Drive::Const(c)) => {
                constant[k] += 1;
                if *c == 0.0 {
                    rig.vis.entry(n).or_default().push("ANIM_hide -1 0.5 fbw/cockpit/hidden_parts".into());
                }
            }
            (LightKind::Visibility, Drive::Direct { dref, v1, .. }) => {
                direct[k] += 1;
                rig.vis.entry(n).or_default().push(format!("ANIM_hide {} {} {dref}", num(v1 - 1e-4), num(v1 + 1e-4)));
            }
            (LightKind::Visibility, Drive::Lua(i)) => {
                lua[k] += 1;
                let d = helper(rig, &mut used, &mut helpers, *i, "vis");
                rig.vis.entry(n).or_default().push(format!("ANIM_hide -0.0001 0.0001 {d}"));
            }
        }
    }
    let mut ids: Vec<usize> = helpers.keys().copied().collect();
    ids.sort_unstable();
    for i in ids {
        rig.light_lua.push((lights.lua[i].clone(), helpers.remove(&i).unwrap_or_default()));
    }
    let _ = model;
    for (k, what) in ["emissive", "visibility"].iter().enumerate() {
        let found = direct[k] + lua[k] + constant[k] + unresolved[k];
        rig.report.push(format!(
            "cockpit lights: {} {what} codes on model nodes ({} more on nodes the model lacks): {} on a systems dataref directly, {} through SASL, {} constant, {} unresolved",
            found, absent[k], direct[k], lua[k], constant[k], unresolved[k]
        ));
    }
    for (w, n) in why {
        rig.report.push(format!("cockpit lights: {n} unresolved: {w}"));
    }
    if lights.overridden > 0 {
        rig.report.push(format!("cockpit lights: {} nodes given several codes of one kind (the last kept, as MSFS runs them in order)", lights.overridden));
    }
    rig.report.push(format!("cockpit lights: {} distinct codes run in SASL", rig.light_lua.len()));
}

/// The SASL main module: creates every dataref and drives the exterior.
pub fn main_lua(title: &str, rigs: &[&Rig], sim: Option<&SimState>) -> String {
    let mut s = String::new();
    let _ = writeln!(s, "-- {title}: animation and cockpit datarefs.");
    // Each gear leg's own measured compression travel (see `exterior()`'s
    // `gear_travel_m`), falling back to the flat half-metre this code used
    // before per-leg measurement existed for any leg no rig could measure
    // (a cockpit-only rig, or a model missing that leg's clip).
    let mut gear_travel_m = [0.5; 5];
    for rig in rigs {
        for (i, t) in rig.gear_travel_m.iter().enumerate() {
            if let Some(t) = t {
                gear_travel_m[i] = *t;
            }
        }
    }
    s.push_str(
        "-- Generated by msfs2xp-aircraft from FlyByWire's A380X model (GPL-3.0).\n\
         -- fbw/anim/*    exterior animations, 0..1 over each MSFS clip; the ones X-Plane\n\
         --               knows about are set below every frame.\n\
         -- fbw/cockpit/* every cockpit control's position (0..1), written by the\n\
         --               cockpit's click spots; fbw/cockpit/lt/* and vis/* light levels and\n\
         --               visibility of the cockpit's legends, from their emissive codes.\n\
         -- A systems port reads and writes these; nothing else depends on this file.\n\n\
         -- No SASL panels, 3D drawing or mouse handling: the cockpit's clicks\n\
         -- are X-Plane manipulators, which SASL's own click system would take.\n\
         sasl.options.setAircraftPanelRendering(false)\n\
         sasl.options.set3DRendering(false)\n\
         sasl.options.setInteractivity(false)\n\n\
         -- Crash log: gear, attitude and weight to Log.txt every 2 s for the\n\
         -- first minute and when X-Plane flags a crash, to tell why it did.\n\
         local x_crash = globalPropertyi(\"sim/flightmodel2/misc/has_crashed\")\n\
         local x_theta = globalPropertyf(\"sim/flightmodel/position/theta\")\n\
         local x_phi = globalPropertyf(\"sim/flightmodel/position/phi\")\n\
         local x_agl = globalPropertyf(\"sim/flightmodel/position/y_agl\")\n\
         local x_vs = globalPropertyf(\"sim/flightmodel/position/vh_ind\")\n\
         local x_mass = globalPropertyf(\"sim/flightmodel/weight/m_total\")\n\
         local x_force = globalPropertyfa(\"sim/flightmodel2/gear/tire_vertical_force_n_mtr\")\n\
         local x_time = globalPropertyf(\"sim/time/total_running_time_sec\")\n\
         local log_start, log_next, was_crashed = nil, 0, 0\n\n\
         local function clamp(x, lo, hi) if x < lo then return lo elseif x > hi then return hi end return x end\n\
         local function g(t, i) if type(t) == \"table\" then return t[i + 1] or 0 end return 0 end\n\
         local function val(p) if p == nil then return 0 end return get(p) or 0 end\n\
         local function door(x) return clamp(math.min(x, 1 - x) * 8, 0, 1) end\n\
         local function bi(x, k) return clamp(0.5 + 0.5 * k * x, 0, 1) end\n\n\
         local x_dep = globalPropertyfa(\"sim/flightmodel2/gear/deploy_ratio\")\n\
         local x_defl = globalPropertyfa(\"sim/flightmodel2/gear/tire_vertical_deflection_mtr\")\n\
         local x_steer = globalPropertyfa(\"sim/flightmodel2/gear/tire_steer_actual_deg\")\n\
         local x_tire = globalPropertyfa(\"sim/flightmodel2/gear/tire_rotation_angle_deg\")\n\
         local x_gnd = globalPropertyia(\"sim/flightmodel2/gear/on_ground\")\n\
         local x_flap = globalPropertyf(\"sim/flightmodel2/controls/flap1_deploy_ratio\")\n\
         local x_slat = globalPropertyf(\"sim/flightmodel2/controls/slat1_deploy_ratio\")\n\
         local x_sb = globalPropertyf(\"sim/flightmodel2/controls/speedbrake_ratio\")\n\
         local x_roll = globalPropertyf(\"sim/cockpit2/controls/total_roll_ratio\")\n\
         local x_pitch = globalPropertyf(\"sim/cockpit2/controls/total_pitch_ratio\")\n\
         local x_yaw = globalPropertyf(\"sim/cockpit2/controls/total_heading_ratio\")\n\
         local x_trim = globalPropertyf(\"sim/cockpit2/controls/elevator_trim\")\n\
         local x_pbrk = globalPropertyf(\"sim/cockpit2/controls/parking_brake_ratio\")\n\
         local x_n1 = globalPropertyfa(\"sim/cockpit2/engine/indicators/N1_percent\")\n\
         local x_rev = globalPropertyfa(\"sim/flightmodel2/engines/thrust_reverser_deploy_ratio\")\n\
         local x_dt = globalPropertyf(\"sim/operation/misc/frame_rate_period\")\n\n\
         local DEP, DEFL, STEER, TIRE, GND, N1, REV = {}, {}, {}, {}, {}, {}, {}\n\
         local FAN = {0, 0, 0, 0}\n\
         -- The RAT's continuous spin, degrees, wrapped like FAN (see the\n\
         -- \"a32nx_hyd_rat_rpm\" clip in rig.rs): RAT_RPM is a real rpm, not\n\
         -- a 0..1 ratio.\n\
         local RAT_SPIN = 0\n",
    );
    // Strut compression, 0..1 over each leg's own real travel (metres),
    // measured from its own clip by `exterior()` -- not a divisor borrowed
    // from another aircraft. GEAR_TRAVEL is nose/l_body/r_body/l_wing/r_wing,
    // matching DEFL's own gear index (0-based; g(i) mapping is +1 into this
    // 1-based Lua table).
    let _ = write!(
        s,
        "local GEAR_TRAVEL = {{{}, {}, {}, {}, {}}}\n\
         local function comp(i) return clamp(g(DEFL, i) / GEAR_TRAVEL[i + 1], 0, 1) end\n\n\
         local D = {{}}\n",
        num(gear_travel_m[0]),
        num(gear_travel_m[1]),
        num(gear_travel_m[2]),
        num(gear_travel_m[3]),
        num(gear_travel_m[4]),
    );
    for rig in rigs {
        for (name, v) in &rig.created {
            let _ = writeln!(s, "D[\"{name}\"] = createGlobalPropertyf(\"{name}\", {}, false, true, false)", num(*v));
        }
    }
    // A runtime, no-reconversion way to hide the passenger cabin for frame
    // rate: every grafted cabin mesh wraps itself in `ANIM_show 1 1
    // fbw/options/cabin_visible` (see `main.rs`'s `visible` closure), so
    // X-Plane reads this every frame with no Lua polling needed -- SASL's
    // only job is creating it. Default 1 (shown); DataRefTool or a later
    // settings UI can write 0 to hide it. Declared once, unconditionally
    // (harmless if this build has no cabin): a build's dataref set should
    // not depend on whether --no-cabin happened to be given this time.
    s.push_str("D[\"fbw/options/cabin_visible\"] = createGlobalPropertyf(\"fbw/options/cabin_visible\", 1, false, true, false)\n");
    // Cockpit controls bound to FlyByWire's systems.
    s.push_str(
        "\n-- Cockpit controls bound to FlyByWire's systems (fbw/<variable>, published by\n\
         -- the systems plugin). Datarefs are looked up when first used; ones still\n\
         -- missing a few seconds after start are created here, so controls whose\n\
         -- variables only FBW's instruments use still move.\n\
         local REF, MISS, NOW = {}, {}, 0\n\
         local function ref(n)\n\
         \x20 local p = REF[n]\n\
         \x20 if p ~= nil then return p end\n\
         \x20 local m = MISS[n]\n\
         \x20 -- Retried every 5s, forever: a dataref another plugin publishes\n\
         \x20 -- late (load order, or the systems plugin still starting up)\n\
         \x20 -- must still be found once it exists, not orphaned by a tries\n\
         \x20 -- limit. `TARGETS` below papers over this for a control's own\n\
         \x20 -- read/write targets after a few seconds; this keeps every\n\
         \x20 -- other lookup (mirrors, look() reads) correct too.\n\
         \x20 if m ~= nil and NOW < m.at then return nil end\n\
         \x20 p = globalPropertyf(n)\n\
         \x20 if p ~= nil then REF[n] = p else MISS[n] = { at = NOW + 5 } end\n\
         \x20 return p\n\
         end\n\
         local function rd(n) local p = ref(n) if p == nil then return 0 end return get(p) or 0 end\n\
         local function wr(n, v) local p = ref(n) if p ~= nil then set(p, v) end end\n\
         -- RPN stack helpers for the translated MSFS code; LV holds MSFS-local state.\n\
         local LV = {}\n\
         local function P(s, v) s[#s + 1] = v end\n\
         local function Q(s) local v = s[#s] s[#s] = nil return v or 0 end\n\
         local function B(x) if x then return 1 end return 0 end\n\
         local LOOKS = {}\n\
         local function look(d, f) LOOKS[#LOOKS + 1] = {p = D[d], f = f} end\n\
         -- A position that only changes on a discrete click (toggle, push,\n\
         -- a script's up/down) or as a mirrored follower with no\n\
         -- manipulator of its own: sampled at the same 20 Hz as the lights\n\
         -- below, not every frame (a native axis manipulator dragged live\n\
         -- keeps using `look`, so the geometry tracks the mouse).\n\
         local SLOW = {}\n\
         local function look_slow(d, f) SLOW[#SLOW + 1] = {p = D[d], f = f} end\n\
         local LIGHTS, lights_next = {}, 0\n\
         local function light(ds, f) local ps = {} for i, d in ipairs(ds) do ps[i] = D[d] end LIGHTS[#LIGHTS + 1] = {ps = ps, f = f} end\n\
         -- X-Plane commands (the systems plugin's fbw/event/* and X-Plane's own) and datarefs.\n\
         local CMDREF, XREF = {}, {}\n\
         local function CMD(n)\n\
         \x20 if n == nil then return end\n\
         \x20 local c = CMDREF[n]\n\
         \x20 if c == nil then c = sasl.findCommand(n) or false CMDREF[n] = c end\n\
         \x20 if c then sasl.commandOnce(c) end\n\
         end\n\
         -- A held command: begin while the written value is non-zero, end once\n\
         -- it is zero. For MSFS click templates whose LEFT_SINGLE_CODE and\n\
         -- LEFT_LEAVE_CODE write 1 and 0 to the same variable (a pushbutton\n\
         -- held down), where the systems plugin's real input is the command's\n\
         -- own begin/end phase, not a variable value -- CMD() run twice would\n\
         -- fire two one-shots, never holding the command between them.\n\
         local function CMD_HELD(n, v)\n\
         \x20 if n == nil then return end\n\
         \x20 local c = CMDREF[n]\n\
         \x20 if c == nil then c = sasl.findCommand(n) or false CMDREF[n] = c end\n\
         \x20 if not c then return end\n\
         \x20 if v ~= 0 then sasl.commandBegin(c) else sasl.commandEnd(c) end\n\
         end\n\
         -- Array-element datarefs (\"name[index]\", e.g. generic_lights_switch[2])\n\
         -- are NOT understood by globalPropertyf(): it looks up the literal\n\
         -- bracketed string as the dataref name, which never matches a real\n\
         -- one, so the lookup silently fails and XGET/XSET on it become\n\
         -- permanent no-ops (W130, following on from W13/W110). globalProperty()\n\
         -- (auto-typed) parses the \"[n]\" suffix itself\n\
         -- (initProperties.lua's own `string.match(name, '(.+)%[(%d+)%]$')`)\n\
         -- and binds that one array element, so route bracketed names there;\n\
         -- every other name keeps the typed float accessor unchanged.\n\
         local function xref(n)\n\
         \x20 local p = XREF[n]\n\
         \x20 if p == nil then\n\
         \x20 \x20 if string.find(n, \"%[%d+%]$\") then p = globalProperty(n) or false else p = globalPropertyf(n) or false end\n\
         \x20 \x20 XREF[n] = p\n\
         \x20 end\n\
         \x20 return p\n\
         end\n\
         local function XGET(n) local p = xref(n) if not p then return 0 end return get(p) or 0 end\n\
         local function XSET(n, v) local p = xref(n) if p then set(p, v) end end\n\
         local function command(name, desc, on_begin, on_end)\n\
         \x20 local c = sasl.createCommand(name, desc)\n\
         \x20 sasl.registerCommandHandler(c, 0, function(phase)\n\
         \x20   if phase == SASL_COMMAND_BEGIN and on_begin ~= nil then on_begin()\n\
         \x20   elseif phase == SASL_COMMAND_END and on_end ~= nil then on_end() end\n\
         \x20   return 1\n\
         \x20 end)\n\
         end\n",
    );
    // Start values of XML-only state (the aircraft's flight file).
    if let Some(sim) = sim {
        let mut locals: Vec<(&String, &f64)> = sim.locals.iter().collect();
        locals.sort_by(|a, b| a.0.cmp(b.0));
        for (k, v) in locals {
            let _ = writeln!(s, "LV[{k:?}] = {}", num(*v));
        }
    }
    for rig in rigs {
        for chunk in &rig.lua {
            s.push_str(chunk);
        }
        for (body, drefs) in &rig.light_lua {
            let list: Vec<String> = drefs.iter().map(|d| format!("{d:?}")).collect();
            let _ = writeln!(s, "light({{{}}}, function()\n{body}\nend)", list.join(", "));
        }
    }
    // The components' update codes.
    s.push_str("local UPDATES = {}\n");
    for rig in rigs {
        for (body, freq, once, what) in &rig.updates {
            let period = freq.filter(|f| *f > 0.0).map_or("0".to_string(), |f| num(1.0 / f));
            let _ = writeln!(s, "-- {what}\nUPDATES[#UPDATES + 1] = {{period = {period}, once = {once}, next = 0, f = function()\n{body}\nend}}");
        }
    }
    // Systems datarefs, with the start value the aircraft's own files give
    // those nothing publishes.
    s.push_str("local TARGETS = {\n");
    let mut all: BTreeSet<&String> = BTreeSet::new();
    for rig in rigs {
        all.extend(rig.targets.iter());
    }
    for t in all {
        let v = sim.and_then(|m| m.defaults.get(t)).copied().unwrap_or(0.0);
        let _ = writeln!(s, "  {{\"{t}\", {}}},", num(v));
    }
    s.push_str("}\nlocal targets_checked = false\n");

    // Click log: each cockpit control's name the first time it moves after
    // load. No baseline value here (unlike `TARGETS`' start values): a bound
    // control's clip follows the systems from the very first frame (its
    // `look()` runs before the first watch check), which can differ a lot
    // from the model's rest pose and is not a click. The first watch check
    // below only records that starting point; only a change after it logs.
    s.push_str("\nlocal watch = {\n");
    for rig in rigs {
        for (name, _) in &rig.created {
            if name.starts_with("fbw/cockpit/") && !name.starts_with("fbw/cockpit/lt/") && !name.starts_with("fbw/cockpit/vis/") {
                let _ = writeln!(s, "  {{n = \"{name}\", p = D[\"{name}\"]}},");
            }
        }
    }
    s.push_str("}\nlocal watch_next = 0\n");
    s.push_str(
        "\n\
         function update()\n\
         \x20 DEP, DEFL, STEER, TIRE = val(x_dep), val(x_defl), val(x_steer), val(x_tire)\n\
         \x20 GND, N1, REV = val(x_gnd), val(x_n1), val(x_rev)\n\
         \x20 local FLAP, SLAT, SB = val(x_flap), val(x_slat), val(x_sb)\n\
         \x20 local ROLL, PITCH, YAW, TRIM = val(x_roll), val(x_pitch), val(x_yaw), val(x_trim)\n\
         \x20 local PBRK, dt = val(x_pbrk), val(x_dt)\n\
         \x20 for i = 1, 4 do FAN[i] = (FAN[i] + g(N1, i - 1) * 24 * dt) % 360 end\n\
         \x20 RAT_SPIN = (RAT_SPIN + rd(\"fbw/A32NX_RAT_RPM\") * 6 * dt) % 360\n",
    );
    for rig in rigs {
        for line in &rig.update {
            let _ = writeln!(s, "  {line}");
        }
    }
    s.push_str(
        "  local now = val(x_time)\n\
         \x20 NOW = now\n\
         \x20 if log_start == nil then log_start = now end\n\
         \x20 if not targets_checked and now - log_start > 3 then\n\
         \x20   targets_checked = true\n\
         \x20   local made = 0\n\
         \x20   for _, t in ipairs(TARGETS) do\n\
         \x20     local n = t[1]\n\
         \x20     MISS[n] = nil\n\
         \x20     if ref(n) == nil then REF[n] = createGlobalPropertyf(n, t[2], false, false, false) made = made + 1 end\n\
         \x20   end\n\
         \x20   logInfo(string.format(\"A380 cockpit: %d systems datarefs, %d not published by the systems and created here\", #TARGETS, made))\n\
         \x20 end\n\
         \x20 for _, u in ipairs(UPDATES) do\n\
         \x20   if not u.done and now >= u.next and (targets_checked or not u.once) then\n\
         \x20     pcall(u.f)\n\
         \x20     if u.once then u.done = true end\n\
         \x20     u.next = now + u.period\n\
         \x20   end\n\
         \x20 end\n\
         \x20 for _, l in ipairs(LOOKS) do\n\
         \x20   local ok, v = pcall(l.f)\n\
         \x20   if ok and type(v) == \"number\" then set(l.p, clamp(v, 0, 1)) end\n\
         \x20 end\n\
         \x20 -- Lights, and the SLOW positions above, at 20 Hz: enough for\n\
         \x20 -- the XML's 1 Hz blinking legends, and a click already shows\n\
         \x20 -- through its own command handler, so up to 50 ms for the\n\
         \x20 -- displayed position to catch up is not felt.\n\
         \x20 if now >= lights_next then\n\
         \x20   lights_next = now + 0.05\n\
         \x20   for _, l in ipairs(LIGHTS) do\n\
         \x20     local ok, v = pcall(l.f)\n\
         \x20     if ok and type(v) == \"number\" then for _, p in ipairs(l.ps) do set(p, v) end end\n\
         \x20   end\n\
         \x20   for _, l in ipairs(SLOW) do\n\
         \x20     local ok, v = pcall(l.f)\n\
         \x20     if ok and type(v) == \"number\" then set(l.p, clamp(v, 0, 1)) end\n\
         \x20   end\n\
         \x20 end\n\
         \x20 local crashed = val(x_crash)\n\
         \x20 if (now - log_start < 60 and now >= log_next) or (crashed == 1 and was_crashed ~= 1) then\n\
         \x20   log_next = now + 2\n\
         \x20   local F = val(x_force)\n\
         \x20   local msg = string.format(\"A380 gear: crashed=%d pitch=%.1f roll=%.1f agl=%.2fm vs=%.2fm/s mass=%.0fkg\", crashed, val(x_theta), val(x_phi), val(x_agl), val(x_vs), val(x_mass))\n\
         \x20   for i = 0, 4 do msg = msg .. string.format(\" | g%d defl %.3fm force %.0fN\", i, g(DEFL, i), g(F, i)) end\n\
         \x20   logInfo(msg)\n\
         \x20 end\n\
         \x20 was_crashed = crashed\n\
         \x20 -- Diagnostic only (which control moved first): bounded to the\n\
         \x20 -- same 60 s window as the crash log above rather than polling\n\
         \x20 -- 1091+ entries for the rest of the flight.\n\
         \x20 if now - log_start < 60 and now >= watch_next then\n\
         \x20   watch_next = now + 0.5\n\
         \x20   for _, w in ipairs(watch) do\n\
         \x20     local v = get(w.p)\n\
         \x20     if v ~= nil then\n\
         \x20       -- The first sample after load is the starting point (a\n\
         \x20       -- bound control's look() may have already moved it away\n\
         \x20       -- from the model's rest pose before this ever runs), not\n\
         \x20       -- a click: only a change after that baseline logs.\n\
         \x20       if w.v ~= nil and not w.logged and math.abs(v - w.v) > 0.001 then\n\
         \x20         logInfo(\"A380 click: \" .. w.n .. \" = \" .. string.format(\"%.2f\", v))\n\
         \x20         w.logged = true\n\
         \x20       end\n\
         \x20       w.v = v\n\
         \x20     end\n\
         \x20   end\n\
         \x20 end\n\
         end\n",
    );
    s
}

/// True when a control FlyByWire's XML resolved to working SASL state and
/// click logic (`b` in `r.bindings`) has nothing in the converted model
/// that could ever trigger or show it: no animation clip carries its
/// `anim` name (the per-clip loop above binds those) and the node-name
/// fallback (same idea as the WXR knob fallback above) found no node
/// either. Two real examples this caught: LOCK_OVHD_HYD_RATMANON /
/// PUSH_OVHD_HYD_RATMANON and LOCK_OVHD_EMERELECPWR_EMERTEST /
/// PUSH_OVHD_EMERELECPWR_EMERTEST — FlyByWire's XML resolves both, SASL
/// gets full cover/press logic for both, but no installed object carries
/// a node or clip by either name, so the switches are permanently dead.
fn control_has_no_geometry(has_clip: bool, has_node_manip: bool) -> bool {
    !has_clip && !has_node_manip
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::model::glb::{trs_matrix, AnimPath, Channel, Clip, Node};

    fn rest_node(name: &str) -> Node {
        Node { name: name.into(), parent: None, translation: [0.0; 3], rotation: [0.0, 0.0, 0.0, 1.0], scale: [1.0; 3], world: trs_matrix([0.0; 3], [0.0, 0.0, 0.0, 1.0], [1.0; 3]) }
    }

    /// The nose's compression is measured with the leg extended. The glTF
    /// rest pose has the A380's nose leg retracted (its "c_gear" swing at
    /// 0), where the strut lies nearly horizontal, so the vertical part of
    /// "c_gear_comp"'s motion measured there came out under the 0.05 m
    /// floor -- the installed nose travel was 0.05 m against ~0.48 m real,
    /// and the drawn strut sat fully compressed under any load. The clip
    /// also turns the torque links (FBW's NLG_LOWER_LINK1/2) through a
    /// larger angle than the strut's slide, and measuring at the pivot of
    /// whichever node turned most read no travel at all; the travel is the
    /// largest vertical movement of any node the clip moves.
    #[test]
    fn nose_travel_is_measured_with_the_leg_extended_not_in_the_rest_pose() {
        let mut model = Model::default();
        model.nodes.push(rest_node("nose_leg"));
        let mut wheel = rest_node("nose_wheel");
        wheel.parent = Some(0);
        model.nodes.push(wheel);
        let mut link = rest_node("nose_torque_link");
        link.parent = Some(0);
        model.nodes.push(link);
        let s = std::f32::consts::FRAC_1_SQRT_2;
        let (s30, c30) = (0.5f32, 0.866_025_4f32);
        model.clips.push(Clip {
            name: "c_gear".into(),
            channels: vec![Channel {
                node: 0,
                path: AnimPath::Rotation,
                times: vec![0.0, 1.0],
                // Retracted (rest) to extended: 90 degrees about X.
                values: vec![[0.0, 0.0, 0.0, 1.0], [s, 0.0, 0.0, s]],
                step: false,
            }],
        });
        model.clips.push(Clip {
            name: "c_gear_comp".into(),
            channels: vec![
                Channel {
                    node: 1,
                    path: AnimPath::Translation,
                    times: vec![0.0, 1.0],
                    // Along the strut: horizontal while retracted, vertical
                    // once the leg has swung down.
                    values: vec![[0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.2, 0.0]],
                    step: false,
                },
                Channel {
                    node: 2,
                    path: AnimPath::Rotation,
                    times: vec![0.0, 1.0],
                    // The torque link folds 60 degrees about its own pivot.
                    values: vec![[0.0, 0.0, 0.0, 1.0], [s30, 0.0, 0.0, c30]],
                    step: false,
                },
            ],
        });
        let idx = ClipIndex::new(&model);
        let rig = exterior(&model, &idx, None);
        assert!((rig.gear_travel_m[0].unwrap() - 0.2).abs() < 1e-4, "{:?}", rig.gear_travel_m[0]);
    }

    /// A translating gear leg's compression travel, measured from the model
    /// rather than assumed: the nose's own separate "c_gear_comp" clip lifts
    /// its wheel 0.2 m over its own 0..1 (no "c_gear" clip here -- extension
    /// is not what `gear_travel_m` measures); the left body gear's single
    /// combined clip swings its wheel down 2 m over the first half (bay to
    /// extended) then up 0.3 m over the second (comp()'s own 0.5..1.0,
    /// extended to compressed) -- 0.3 m is what `comp(1)` must divide by,
    /// not the 2 m of swing or the clip's full 0..1 span.
    #[test]
    fn gear_travel_m_is_measured_per_leg_not_assumed() {
        let mut model = Model::default();
        model.nodes.push(rest_node("nose_wheel"));
        model.nodes.push(rest_node("l_b_wheel"));
        model.clips.push(Clip {
            name: "c_gear_comp".into(),
            channels: vec![Channel {
                node: 0,
                path: AnimPath::Translation,
                times: vec![0.0, 1.0],
                values: vec![[0.0, 0.0, 0.0, 0.0], [0.0, 0.2, 0.0, 0.0]],
                step: false,
            }],
        });
        model.clips.push(Clip {
            name: "l_b_gear".into(),
            channels: vec![Channel {
                node: 1,
                path: AnimPath::Translation,
                times: vec![0.0, 1.0, 2.0],
                values: vec![[0.0, 2.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0], [0.0, 0.3, 0.0, 0.0]],
                step: false,
            }],
        });
        let idx = ClipIndex::new(&model);
        let rig = exterior(&model, &idx, None);

        assert!((rig.gear_travel_m[0].unwrap() - 0.2).abs() < 1e-6, "{:?}", rig.gear_travel_m[0]);
        assert!((rig.gear_travel_m[1].unwrap() - 0.3).abs() < 1e-6, "{:?}", rig.gear_travel_m[1]);
        // No clip for the right body or either wing gear in this minimal
        // model: no measurement, not a panic or a made-up number.
        assert_eq!(rig.gear_travel_m[2], None);
        assert_eq!(rig.gear_travel_m[3], None);
        assert_eq!(rig.gear_travel_m[4], None);

        // main_lua bakes each measured leg into GEAR_TRAVEL and drives
        // comp() from it; the other three fall back to the flat 0.5 m every
        // leg used to share, since nothing measured them here.
        let lua = main_lua("Test", &[&rig], None);
        assert!(lua.contains("local GEAR_TRAVEL = {0.2, 0.3, 0.5, 0.5, 0.5}"), "{lua}");
        assert!(lua.contains("local function comp(i) return clamp(g(DEFL, i) / GEAR_TRAVEL[i + 1], 0, 1) end"), "{lua}");
        assert!(!lua.contains("g(DEFL, i) / 0.5"), "{lua}");
    }

    #[test]
    fn controls_are_classified_by_name() {
        assert_eq!(kind("PUSH_OVHD_ELEC_BAT1"), Kind::Push);
        assert_eq!(kind("KNOB_RMP_1_CAB_ROTATE_ANIM "), Kind::Knob);
        assert_eq!(kind("KNOB_RMP_1_CAB_PUSH_ANIM"), Kind::Push);
        assert_eq!(kind("SWITCH_AUTOBKR_ASKID"), Kind::Switch);
        assert_eq!(kind("CB_CIDS2"), Kind::Toggle);
        assert_eq!(kind("A380X_OVHD_ENG1_FIRE_GUARD"), Kind::Toggle);
        assert_eq!(kind("throttle_lever_1"), Kind::Drag);
        assert_eq!(kind("INSTRUMENT_Dial_Compass"), Kind::Fixed);
        assert_eq!(kind("LIGHTING_Knob_Panel"), Kind::Knob);
    }

    #[test]
    fn names_become_dataref_parts() {
        assert_eq!(sanitize("KNOB_RMP_1_CAB_ROTATE_ANIM "), "KNOB_RMP_1_CAB_ROTATE_ANIM");
        assert_eq!(sanitize("RUDDER LOWER.001"), "RUDDER_LOWER_001");
        let mut used = HashSet::new();
        assert_eq!(unique(&mut used, "a".into()), "a");
        assert_eq!(unique(&mut used, "a".into()), "a_2");
    }

    fn test_binding(click: Click) -> bind::Binding {
        bind::Binding {
            anim: "TEST".into(),
            node: "n".into(),
            template: "t".into(),
            click,
            look: Some("return 1".into()),
            targets: BTreeSet::new(),
            reads: BTreeSet::new(),
            commands: BTreeSet::new(),
        }
    }

    #[test]
    fn bound_lua_keeps_a_live_axis_drag_at_full_rate_but_throttles_every_other_click() {
        let axis = test_binding(Click::Axis { dref: "fbw/x".into(), v0: 0.0, v1: 1.0, step: 0.05, knob: true, horizontal: false });
        let out = bound_lua(&axis, "test", Some("fbw/cockpit/test"), 1.0);
        assert!(out.contains("look(\"fbw/cockpit/test\""), "{out}");
        assert!(!out.contains("look_slow("), "{out}");

        let toggle = test_binding(Click::Toggle { dref: "fbw/x".into(), on: 1.0, off: 0.0 });
        let out = bound_lua(&toggle, "test", Some("fbw/cockpit/test"), 1.0);
        assert!(out.contains("look_slow(\"fbw/cockpit/test\""), "{out}");
    }

    #[test]
    fn main_lua_drops_the_dead_component_update_and_bounds_the_click_watch() {
        let out = main_lua("Test", &[], None);
        assert!(!out.contains("components = {}"), "{out}");
        assert!(!out.contains("updateAll("), "{out}");
        assert!(out.contains("local function look_slow(d, f)"), "{out}");
        assert!(out.contains("now - log_start < 60 and now >= watch_next"), "{out}");
    }

    #[test]
    fn xref_resolves_bracketed_array_element_datarefs() {
        // W130: globalPropertyf() cannot resolve "dataref[n]" (it looks the
        // literal bracketed string up as a whole dataref name and always
        // fails), so xref() must route those names through the auto-typed
        // globalProperty(), which does parse the "[n]" suffix.
        let lua = main_lua("A380X", &[], None);
        assert!(
            lua.contains("string.find(n, \"%[%d+%]$\") then p = globalProperty(n)"),
            "xref() must detect a bracketed array-element name and resolve it with globalProperty(), not globalPropertyf()"
        );
        assert!(
            !lua.contains("local function xref(n) local p = XREF[n] if p == nil then p = globalPropertyf(n) or false XREF[n] = p end return p end"),
            "the old xref() that resolves every bracketed name to nil must be gone"
        );
    }

    #[test]
    fn aileron_panels_follow_the_clip_number() {
        assert_eq!(aileron_panel("l_aileron1_percent_key"), "INWARD");
        assert_eq!(aileron_panel("r_aileron2_percent_key"), "MIDDLE");
        assert_eq!(aileron_panel("l_aileron3_percent_key"), "OUTWARD");
        // No digit: defaults to the first panel rather than panicking.
        assert_eq!(aileron_panel("aileron"), "INWARD");
    }

    #[test]
    fn side_of_reads_the_l_r_prefix() {
        assert_eq!(side_of("l_aileron1_percent_key"), "LEFT");
        assert_eq!(side_of("r_aileron1_percent_key"), "RIGHT");
    }

    #[test]
    fn maybe_flip_inverts_only_when_asked() {
        assert_eq!(maybe_flip("X".into(), false), "X");
        assert_eq!(maybe_flip("X".into(), true), "1 - X");
    }

    #[test]
    fn spoiler_panels_follow_the_clip_number_inboard_to_outboard() {
        assert_eq!(spoiler_panel("l_spoiler1_key"), 1);
        assert_eq!(spoiler_panel("r_spoiler8_key"), 8);
        // No digit: defaults to the first panel rather than panicking.
        assert_eq!(spoiler_panel("spoiler"), 1);
    }

    #[test]
    fn spoiler_source_reads_the_real_panel_and_side_scaled_like_msfs() {
        // A380_EXTERIOR.xml's own FBW_Spoiler_Surface_Template scales the
        // same clip by "#DEFLECTION_CODE# 100 * 50 * 65 /"; over X-Plane's
        // own 0..1 clip fraction (not MSFS's 0..100 ANIM_CODE) that is the
        // same *50/65 this mirrors.
        assert_eq!(spoiler_source("l_spoiler3_key"), "clamp(rd(\"fbw/A32NX_HYD_SPOILER_3_LEFT_DEFLECTION\") * 50 / 65, 0, 1)");
        assert_eq!(spoiler_source("r_spoiler8_key"), "clamp(rd(\"fbw/A32NX_HYD_SPOILER_8_RIGHT_DEFLECTION\") * 50 / 65, 0, 1)");
    }

    #[test]
    fn flex_clips_follow_fbws_flex_model_by_the_behaviour_xmls_own_formula() {
        // A380_EXTERIOR.xml: `(L:A32NX_WING_FLEX_LEFT_INBOARD, number) 23.25 *
        // 55.35 +` over 100 frames, and `(L:A32NX_ENGINE_1_WOBBLE_X_POSITION,
        // number) 100 *`.
        assert_eq!(
            flex_source("left_inboard_flex").unwrap(),
            "clamp((rd(\"fbw/A32NX_WING_FLEX_LEFT_INBOARD\") * 23.25 + 55.35) / 100, 0, 1)"
        );
        assert_eq!(
            flex_source("right_outer_midboard_flex").unwrap(),
            "clamp((rd(\"fbw/A32NX_WING_FLEX_RIGHT_OUTBOARD_MID\") * 29.41 + 31.9) / 100, 0, 1)"
        );
        assert_eq!(flex_source("eng1_wobble").unwrap(), "clamp((rd(\"fbw/A32NX_ENGINE_1_WOBBLE_X_POSITION\") * 100 + 0) / 100, 0, 1)");
        assert_eq!(flex_source("aft_flex").unwrap(), "clamp((rd(\"fbw/A32NX_AFT_FLEX_POSITION\") * 100 + 0) / 100, 0, 1)");
        assert!(flex_source("l_flap_percent_key").is_none());
    }

    #[test]
    fn flap_source_reads_fbws_real_fppu_animation_position() {
        // A380_EXTERIOR.xml's Flaps component reads this exact variable
        // directly: `<ANIM_CODE>(L:A32NX_LEFT_FLAPS_ANIMATION_POSITION)</ANIM_CODE>`.
        assert_eq!(flap_source("LEFT"), "rd(\"fbw/A32NX_LEFT_FLAPS_ANIMATION_POSITION\") / 100");
        assert_eq!(flap_source("RIGHT"), "rd(\"fbw/A32NX_RIGHT_FLAPS_ANIMATION_POSITION\") / 100");
    }

    #[test]
    fn slat_source_reads_the_position_percent_msfs_actually_feeds_its_native_simvar_from() {
        // Not `..._SLATS_ANIMATION_POSITION` (the flaps' own variable, by
        // analogy): a380_systems_wasm/src/flaps.rs's `SlatsSurface` feeds
        // the "LEADING EDGE FLAPS LEFT/RIGHT PERCENT" SimVar the exterior
        // model's slat clip actually reads from `LEFT_SLATS_POSITION_PERCENT`/
        // `RIGHT_SLATS_POSITION_PERCENT`, not from the animation-position
        // variable flaps use.
        assert_eq!(slat_source("LEFT"), "rd(\"fbw/A32NX_LEFT_SLATS_POSITION_PERCENT\") / 100");
        assert_eq!(slat_source("RIGHT"), "rd(\"fbw/A32NX_RIGHT_SLATS_POSITION_PERCENT\") / 100");
    }

    /// A passenger-door handle: its click's node (`PAX_DOOR_M1L_HANDLE`)
    /// lives on the exterior model, not on the clip named after it
    /// (`ANIM_DOOR_M1L_CLICK`, which has no geometry of its own here); this
    /// is exactly the case `manips_on` exists for.
    #[test]
    fn manips_on_finds_a_control_whose_node_is_not_the_clip_it_is_named_after() {
        let mut names = HashMap::new();
        names.insert("pax_door_m1l_handle".to_string(), 7);
        let binding = bind::Binding {
            anim: "ANIM_DOOR_M1L_CLICK".into(),
            node: "PAX_DOOR_M1L_HANDLE".into(),
            template: "FBW_Airbus_Door".into(),
            click: Click::Command { cmd: "sim/flight_controls/door_toggle_1".into() },
            look: None,
            targets: BTreeSet::new(),
            reads: BTreeSet::new(),
            commands: BTreeSet::new(),
        };
        let res = Resolution { bindings: vec![binding], ..Default::default() };

        let m = manips_on(&names, Some(&res));
        let (priority, manip) = m.get(&7).expect("PAX_DOOR_M1L_HANDLE's node gets a manipulator");
        assert_eq!(*priority, 20);
        assert_eq!(manip, "ATTR_manip_command hand sim/flight_controls/door_toggle_1 ANIM DOOR M1L CLICK");

        // No matching node name in this model: no entry, no panic.
        assert!(manips_on(&HashMap::new(), Some(&res)).is_empty());
        // No behaviour XML resolved for this aircraft at all: a no-op.
        assert!(manips_on(&names, None).is_empty());
    }

    #[test]
    fn manips_on_skips_a_node_two_controls_share() {
        let mut names = HashMap::new();
        names.insert("shared_node".to_string(), 3);
        let mk = |anim: &str| bind::Binding {
            anim: anim.into(),
            node: "SHARED_NODE".into(),
            template: "t".into(),
            click: Click::Command { cmd: "fbw/cockpit/x".into() },
            look: None,
            targets: BTreeSet::new(),
            reads: BTreeSet::new(),
            commands: BTreeSet::new(),
        };
        let res = Resolution { bindings: vec![mk("A"), mk("B")], ..Default::default() };
        assert!(manips_on(&names, Some(&res)).is_empty(), "a shared node is a panel, not a control");
    }

    #[test]
    fn a_control_with_no_clip_and_no_node_is_reported_unreachable() {
        // The animated-elsewhere case (has a clip but no own node, e.g. a
        // click plate whose animation lives on a different part): reachable.
        assert!(!control_has_no_geometry(true, false));
        // A node-only fallback manipulator with no clip (the WXR knobs):
        // reachable.
        assert!(!control_has_no_geometry(false, true));
        // Neither a clip nor a node by that name exists anywhere in the
        // model: the LOCK_OVHD_HYD_RATMANON / LOCK_OVHD_EMERELECPWR_EMERTEST
        // shape this task found. Unreachable.
        assert!(control_has_no_geometry(false, false));
        // Has both: reachable (ordinary bound control).
        assert!(!control_has_no_geometry(true, true));
    }
}
