//! Exterior lights, at the light nodes the MSFS systems.cfg names.
//!
//! MSFS attaches each `lightdef` to a model node (`Node:LIGHT_ASOBO_...`), so
//! the positions come straight from the model. X-Plane's own airplane lights
//! (`airplane_*_pm` for the light cast, `airplane_*_bb` for the glare) are
//! used, with Laminar's A330 values, so X-Plane's light switches work them:
//! nav (incl. the wingtip obstruction lights, the same switch/effect as the
//! tail nav light), beacon and strobe switches, landing lights on the
//! nose-gear (`landing_lights_switch` 0/1) and fuselage (2..5) slots the
//! systems.cfg Index field -- not lightdef order -- tells apart, the taxi
//! light, and generic lights 0 (runway turn-off), 1 (wing scan) and 2 (logo).
//! Lights on moving parts (the nose gear's) follow its animation.
//!
//! The MSFS light-node *meshes* (housings, lenses, glass domes) are a
//! separate, model-side concern handled in main.rs where the exterior
//! model's meshes are filtered: see that filter's own doc comment for why
//! only meshes with no real authored material are left out now, not every
//! mesh under a `LIGHT_ASOBO_*`/`LIGHT_AMBIENT_*` node.

use std::fmt::Write as _;

use crate::model::anim::{num, xp, Animator};
use crate::model::Model;

/// (type, lightdef.Index, node name) of each light definition, first per
/// node, without the ambient-only effects. The `Index` field is what ties a
/// lightdef to the circuit that powers it -- systems.cfg's own doc says so
/// directly ("lightdef.Index and circuit.Type are related. When the
/// lightdef.Index and circuit.Type numeric suffix match, then that circuit
/// affects the powered state of the light") -- so it is carried through here
/// as the reliable way to route a Type:6 (CIRCUIT_LIGHT_TAXI) node onto the
/// right X-Plane switch; see `is_taxi_light_index` and fixes/W207.md.
fn lightdefs(systems_cfg: &str) -> Vec<(u32, u32, String)> {
    let mut out: Vec<(u32, u32, String)> = Vec::new();
    for line in systems_cfg.lines() {
        let line = line.split(';').next().unwrap_or("").trim();
        let Some((key, value)) = line.split_once('=') else { continue };
        if !key.trim().to_ascii_lowercase().starts_with("lightdef.") {
            continue;
        }
        let mut ty = None;
        let mut idx = None;
        let mut node = None;
        for part in value.trim().split('#') {
            if let Some((k, v)) = part.split_once(':') {
                match k.trim().to_ascii_lowercase().as_str() {
                    "type" => ty = v.trim().parse::<u32>().ok(),
                    "index" => idx = v.trim().parse::<u32>().ok(),
                    "node" => node = Some(v.trim().to_string()),
                    _ => {}
                }
            }
        }
        if let (Some(t), Some(n)) = (ty, node) {
            let upper = n.to_ascii_uppercase();
            if upper.starts_with("LIGHT_") && !upper.contains("AMBIENT") && !out.iter().any(|(_, _, m)| m == &n) {
                out.push((t, idx.unwrap_or(0), n));
            }
        }
    }
    out
}

/// Whether a Type:6 (CIRCUIT_LIGHT_TAXI) lightdef's `Index` puts it on the
/// taxi_light_on switch (Index:1, systems.cfg circuit.20 "Taxi_Light") as
/// opposed to the runway turn-off switch, generic_lights_switch:0
/// (Index:2/:3, circuit.21/circuit.22 "Taxi_Light_TurnOff_Left/Right").
/// `lights_obj` used to tell these apart by whether "TAKEOFF" appeared in
/// the node name, which is true for the nose gear's LIGHT_ASOBO_TAKEOFF_1
/// but not for LIGHT_ASOBO_TAXI_WING_LH/RH -- real wing-root taxi lights on
/// that very same Index:1 circuit -- so they silently ended up wired to the
/// turn-off switch instead: dark when TAXI was on, lit when RWY TURN OFF
/// was on. Routing by Index instead of the name substring fixes both nodes
/// at once. See fixes/W207.md.
fn is_taxi_light_index(idx: u32) -> bool {
    idx == 1
}

/// The model nodes MSFS lights up as glow meshes (`EmMesh:`), lower case.
/// They are drawn only while their light is on; X-Plane's own lights
/// replace them, so the meshes are left out (drawn always, they hang off
/// the aircraft as translucent beams and boxes).
pub fn glow_meshes(systems_cfg: &str) -> std::collections::HashSet<String> {
    systems_cfg
        .lines()
        .map(|l| l.split(';').next().unwrap_or(""))
        .filter(|l| l.trim_start().to_ascii_lowercase().starts_with("lightdef."))
        .flat_map(|l| l.split('#').map(str::to_string).collect::<Vec<_>>())
        .filter_map(|p| {
            let (k, v) = p.split_once(':')?;
            k.trim().eq_ignore_ascii_case("emmesh").then(|| v.trim().to_ascii_lowercase())
        })
        .filter(|v| !v.is_empty())
        .collect()
}

/// The lights object, and how many lights it holds.
pub fn lights_obj(model: &Model, anim: &Animator, systems_cfg: &str) -> (String, usize) {
    let mut body = String::new();
    let mut count = 0;
    let (mut nose_landing, mut main_landing, mut n_turnoff) = (0, 0, 0);
    let white = "0.94730663 0.82278603 0.7230553";
    let beam = "0.76052475 0.65837479 0.57758057";
    for (ty, idx, name) in lightdefs(systems_cfg) {
        let Some(node) = model
            .nodes
            .iter()
            .position(|n| n.name == name)
            .or_else(|| model.nodes.iter().position(|n| n.name.eq_ignore_ascii_case(&name)))
        else {
            continue;
        };
        let w = model.nodes[node].world;
        let p = xp([w[12], w[13], w[14]]);
        let at = format!("{} {} {}", num(p[0]), num(p[1]), num(p[2]));
        let u = name.to_ascii_uppercase();
        let left = u.contains("LEFT") || u.contains("_LH") || u.ends_with("LH");
        let right = u.contains("RIGHT") || u.contains("_RH") || u.ends_with("RH");
        let side = if left { -1.0 } else if right { 1.0 } else { 0.0 };
        let mut lines = Vec::new();
        match ty {
            3 if u.contains("RED") => {
                for k in ["pm", "bb"] {
                    lines.push(format!("LIGHT_PARAM airplane_nav_{k} {at} 1 0 0 0 3000cd -1 0 0 0.043619335"));
                }
            }
            3 if u.contains("GREEN") => {
                for k in ["pm", "bb"] {
                    lines.push(format!("LIGHT_PARAM airplane_nav_{k} {at} 0.020288363 0.73791075 0.36130685 0 3000cd 1 0 0 0.043619335"));
                }
            }
            3 if u.contains("TAIL") => {
                for k in ["pm", "bb"] {
                    lines.push(format!("LIGHT_PARAM airplane_nav_{k} {at} {white} 0 3000cd 0 0 1 0.043619335"));
                }
            }
            // Wingtip clearance/obstruction lights (LIGHT_ASOBO_OBSTRUCTION_
            // LH/RH): a real A380 fixture, not invented for this port -- the
            // wingspan is wide enough that Airbus fits extra white lights at
            // the tips so ground crews can see them. systems.cfg wires them
            // as CIRCUIT_LIGHT_NAV:3/:4 (circuit.13/.14) on PotentiometerIndex:1
            // and effect LIGHT_A380X_NavigationWhite -- the exact same effect
            // file the tail light above uses -- so they are the nav switch's
            // white lights, not a light of their own. They used to be
            // silently dropped: `lightdefs()` keeps this node (unlike its
            // AMBIENT-suffixed, mesh-less sibling LIGHT_AMBIENT_OBSTRUCTION_
            // LH/RH, filtered as an ambient duplicate), but nothing in this
            // match handled the name "OBSTRUCTION", so it fell through to
            // `_ => {}` and never became a light.
            3 if u.contains("OBSTRUCTION") => {
                for k in ["pm", "bb"] {
                    lines.push(format!("LIGHT_PARAM airplane_nav_{k} {at} {white} 0 3000cd {} 0 0 0.043619335", num(side)));
                }
            }
            1 => {
                let dy = if u.contains("BELLY") { -1 } else { 1 };
                lines.push(format!("LIGHT_PARAM airplane_beacon_pm {at} 1 0 0 0 32500cd 0 {dy} 0 0.043619335"));
                lines.push(format!("LIGHT_PARAM airplane_beacon_bb {at} 1 0 0 0 40000cd 0 0 0 1"));
            }
            2 => {
                let (dx, dz) = if u.contains("TAIL") { (0.0, 1.0) } else { (side, 0.0) };
                for k in ["pm", "bb"] {
                    lines.push(format!("LIGHT_PARAM airplane_strobe_{k} {at} 1 1 1 0 100000cd {} 0 {} 0.043619335", num(dx), num(dz)));
                }
            }
            // Type:5 = CIRCUIT_LIGHT_LANDING. X-Plane's "index" param on
            // airplane_landing_pm/bb *is* the slot into the
            // landing_lights_switch array (Laminar's own A330 Lights.obj
            // gives its two nose landing lights index 0/2, its two belly
            // lights 1/0, etc. -- always a real switch-array slot, never
            // just a running count). systems.cfg's own Index field already
            // tells the nose-gear "takeoff" lights (Index:1, circuit.17,
            // LIGHT_ASOBO_TAKEOFF_2/_3) apart from the fuselage lights
            // (Index:2/:3, circuit.18/.19, LIGHT_ASOBO_LAND_1/2_LH/RH), and
            // plugin/src/lights.rs's `landing_nose`/`landing_main` groups
            // gate those same two sets onto landing_lights_switch[0,1] and
            // [2,3,4,5] respectively -- so the index handed to each node
            // here must come from that same Index, not from a single
            // shared counter that only happened to land in the right slots
            // because systems.cfg's TAKE OFF LIGHTS section is textually
            // ahead of its LANDING LIGHTS section (nothing enforces that).
            5 => {
                let n = if idx == 1 {
                    let v = nose_landing;
                    nose_landing += 1;
                    v
                } else {
                    let v = 2 + main_landing;
                    main_landing += 1;
                    v
                };
                for k in ["pm", "bb"] {
                    lines.push(format!("LIGHT_PARAM airplane_landing_{k} {at} {beam} {n} 765000cd 0 -0.087155685 -0.99619472 0.97629601"));
                }
            }
            // Type:6 = CIRCUIT_LIGHT_TAXI. Index:1 (circuit.20, "Taxi_Light")
            // is the taxi_light_on switch: LIGHT_ASOBO_TAKEOFF_1 (nose gear)
            // and the wing-root taxi lights LIGHT_ASOBO_TAXI_WING_LH/RH share
            // this one circuit, so both belong on airplane_taxi_*, not just
            // the node whose name happens to say "TAKEOFF" (fixes/W207.md).
            // Index:2/:3 (circuit.21/.22, "Taxi_Light_TurnOff_Left/Right")
            // are the real runway turn-off lights on generic_lights_switch:0.
            6 if is_taxi_light_index(idx) => {
                // The nose light points straight ahead (side==0 for
                // TAKEOFF_1, so dx stays 0, matching the old output exactly);
                // the wing taxi lights point outboard, at the same beam
                // angle the turn-off lights use below (the real angle is not
                // documented anywhere in FBW's source; see OPEN QUESTIONS).
                let dx = if side == 0.0 { 0.0 } else { side * 0.64278761 };
                for k in ["pm", "bb"] {
                    lines.push(format!("LIGHT_PARAM airplane_taxi_{k} {at} {beam} 0 150000cd {} -0.1 -0.99498744 0.9", num(dx)));
                }
            }
            6 => {
                // Runway turn-off lights, 40 degrees out from the nose.
                let dx = if side == 0.0 { 0.0 } else { side * 0.64278761 };
                for k in ["pm", "bb"] {
                    lines.push(format!("LIGHT_PARAM airplane_generic_{k} {at} {beam} 0 65000cd {} -0.05 -0.76604444 0.92387953", num(dx)));
                }
                n_turnoff += 1;
            }
            8 => {
                // Wing scan lights look out and back along the leading edge.
                for k in ["pm", "bb"] {
                    lines.push(format!("LIGHT_PARAM airplane_generic_{k} {at} {white} 1 20000cd {} -0.1 0.81 0.93969262", num(side * 0.57)));
                }
            }
            9 => {
                // Logo lights on the tailplane shine up and in at the fin.
                for k in ["pm", "bb"] {
                    lines.push(format!("LIGHT_PARAM airplane_generic_{k} {at} {white} 2 20000cd {} 0.8660254 0 0.79335335", num(-side * 0.5)));
                }
            }
            _ => {}
        }
        if lines.is_empty() {
            continue;
        }
        count += 1;
        let cmds = anim.commands(Some(node));
        if !cmds.is_empty() {
            body.push_str("ANIM_begin\n");
            body.push_str(&cmds);
        }
        for l in lines {
            let _ = writeln!(body, "{l}");
        }
        if !cmds.is_empty() {
            body.push_str("ANIM_end\n");
        }
    }
    let _ = n_turnoff;
    (format!("I\n800\nOBJ\n\nPOINT_COUNTS 0 0 0 0\n\n{body}"), count)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::model::glb::{Node, IDENTITY};
    use crate::model::ClipIndex;
    use std::collections::HashMap;

    /// A node at `(x, y, z)` (glTF/MSFS axes), no parent, no rotation.
    fn node(name: &str, x: f64, y: f64, z: f64) -> Node {
        let mut world = IDENTITY;
        world[12] = x;
        world[13] = y;
        world[14] = z;
        Node { name: name.to_string(), parent: None, translation: [x, y, z], rotation: [0.0, 0.0, 0.0, 1.0], scale: [1.0; 3], world }
    }

    fn no_op_animator(model: &Model) -> (ClipIndex, HashMap<usize, String>, HashMap<usize, Vec<String>>) {
        (ClipIndex::new(model), HashMap::new(), HashMap::new())
    }

    #[test]
    fn wingtip_obstruction_lights_are_white_nav_lights_not_dropped() {
        // Real A380 fixture (systems.cfg circuit.13/.14, CIRCUIT_LIGHT_NAV:3/
        // :4, PotentiometerIndex:1, EffectFile:LIGHT_A380X_NavigationWhite --
        // the same effect the tail nav light uses): a wingtip clearance
        // light, not a fabricated one. It used to silently produce no light
        // at all because nothing matched the "OBSTRUCTION" node name.
        let cfg = "lightdef.15=Type:3#Index:3#Node:LIGHT_ASOBO_OBSTRUCTION_LH\n\
                   lightdef.16=Type:3#Index:3#Node:LIGHT_ASOBO_OBSTRUCTION_RH\n";
        let mut model = Model::default();
        model.nodes.push(node("LIGHT_ASOBO_OBSTRUCTION_LH", -33.0, 6.5, -0.1));
        model.nodes.push(node("LIGHT_ASOBO_OBSTRUCTION_RH", 33.0, 6.5, -0.1));
        let (index, drefs, vis) = no_op_animator(&model);
        let anim = Animator::new(&model, &index, drefs, vis);
        let (obj, n) = lights_obj(&model, &anim, cfg);
        assert_eq!(n, 2);
        assert!(obj.contains("LIGHT_PARAM airplane_nav_pm"), "{obj}");
        assert!(obj.contains("LIGHT_PARAM airplane_nav_bb"), "{obj}");
    }

    #[test]
    fn landing_lights_route_by_circuit_index_not_lightdef_order() {
        // systems.cfg's LANDING LIGHTS section normally comes after its
        // TAKE OFF LIGHTS section, so a plain running counter would land in
        // the right landing_lights_switch slots ([0,1] nose, [2,3,4,5]
        // main) by coincidence. This cfg deliberately reverses that order
        // (main-gear lights declared first) to prove the routing now comes
        // from the Index field itself (plugin/src/lights.rs's
        // landing_nose/landing_main groups), not from where a lightdef
        // happens to sit in the file.
        let cfg = "lightdef.120=Type:5#Index:2#Node:LIGHT_ASOBO_LAND_1_LH\n\
                   lightdef.128=Type:5#Index:2#Node:LIGHT_ASOBO_LAND_1_RH\n\
                   lightdef.84=Type:5#Index:1#Node:LIGHT_ASOBO_TAKEOFF_3\n\
                   lightdef.88=Type:5#Index:1#Node:LIGHT_ASOBO_TAKEOFF_2\n";
        let mut model = Model::default();
        // Distinct Z per node so each LIGHT_PARAM line can be traced back
        // to the node it came from, independent of emission order.
        for (i, n) in ["LIGHT_ASOBO_LAND_1_LH", "LIGHT_ASOBO_LAND_1_RH", "LIGHT_ASOBO_TAKEOFF_3", "LIGHT_ASOBO_TAKEOFF_2"].into_iter().enumerate() {
            model.nodes.push(node(n, 0.0, 0.0, i as f64 + 1.0));
        }
        let (index, drefs, vis) = no_op_animator(&model);
        let anim = Animator::new(&model, &index, drefs, vis);
        let (obj, n) = lights_obj(&model, &anim, cfg);
        assert_eq!(n, 4);
        let slot_at_z = |z: f64| -> &str {
            let needle = format!(" {} ", num(-z)); // xp() negates Z
            obj.lines()
                .find(|l| l.contains("airplane_landing_pm") && l.contains(&needle))
                .and_then(|l| l.split_whitespace().nth(8))
                .unwrap()
        };
        // LAND_1_LH/RH (Index:2, z=1,2) are the fuselage/main lights, on
        // landing_lights_switch[2,3]; TAKEOFF_3/2 (Index:1, z=3,4) are the
        // nose-gear lights, on [0,1] -- even though the main lights are
        // declared first in this (deliberately reversed) cfg.
        assert_eq!(slot_at_z(1.0), "2", "{obj}");
        assert_eq!(slot_at_z(2.0), "3", "{obj}");
        assert_eq!(slot_at_z(3.0), "0", "{obj}");
        assert_eq!(slot_at_z(4.0), "1", "{obj}");
    }

    #[test]
    fn lightdefs_keep_one_real_light_per_node() {
        let cfg = "lightdef.10=Type:3#Index:1#LocalPosition:0,0,0#Node:LIGHT_ASOBO_NavigationRed\n\
                   lightdef.11 = Type:3#Node:LIGHT_ASOBO_NavigationRed ; again\n\
                   lightdef.12=Type:5#Node:LIGHT_AMBIENT_LAND_1_LH\n\
                   ; lightdef.13=Type:1#Node:LIGHT_X\n\
                   lightdef.14=Type:10#Node:Cube.054\n";
        assert_eq!(lightdefs(cfg), vec![(3, 1, "LIGHT_ASOBO_NavigationRed".to_string())]);
    }

    #[test]
    fn lightdefs_carries_the_index_that_ties_a_node_to_its_circuit() {
        // Real systems.cfg case (fixes/W207.md): LIGHT_ASOBO_TAKEOFF_1 and
        // LIGHT_ASOBO_TAXI_WING_LH are both Index:1 (circuit.20,
        // "Taxi_Light") even though only one has "TAKEOFF" in its name;
        // LIGHT_ASOBO_TURNOFF_LH is Index:2 (circuit.21, a different
        // circuit). lightdefs() must expose the Index so the caller can
        // route by circuit, not by guessing from the node name.
        let cfg = "lightdef.57=Type:6#Index:1#Node:LIGHT_ASOBO_TAKEOFF_1\n\
                   lightdef.51=Type:6#Index:1#Node:LIGHT_ASOBO_TAXI_WING_LH\n\
                   lightdef.68=Type:6#Index:2#Node:LIGHT_ASOBO_TURNOFF_LH\n";
        let out = lightdefs(cfg);
        assert!(out.contains(&(6, 1, "LIGHT_ASOBO_TAKEOFF_1".to_string())));
        assert!(out.contains(&(6, 1, "LIGHT_ASOBO_TAXI_WING_LH".to_string())));
        assert!(out.contains(&(6, 2, "LIGHT_ASOBO_TURNOFF_LH".to_string())));
    }

    #[test]
    fn taxi_circuit_is_told_apart_from_turn_off_by_index_not_name() {
        // LIGHT_ASOBO_TAKEOFF_1 (Index:1) and LIGHT_ASOBO_TAXI_WING_LH/RH
        // (also Index:1) are the same real circuit (systems.cfg circuit.20,
        // "Taxi_Light") and must both land on the taxi switch even though
        // only one of their node names contains "TAKEOFF". TURNOFF_LH/RH
        // (Index:2/:3) must not.
        assert!(is_taxi_light_index(1));
        assert!(!is_taxi_light_index(2));
        assert!(!is_taxi_light_index(3));
    }
}
