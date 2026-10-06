//! What each cockpit control does in MSFS, read from the aircraft's own model
//! behaviour XML, so its X-Plane click can do the same to FlyByWire's systems,
//! and what lights each legend, backlight and annunciator.

pub mod bind;
pub mod emissive;
pub mod events;
pub mod expand;
#[cfg(test)]
mod probe;
pub mod rpn;
pub mod sim;
pub mod xml;

#[cfg(test)]
mod tests;
#[cfg(test)]
mod tests_cockpit;

use std::path::Path;

pub use bind::{Click, Resolution};

/// Read a model XML with its includes and resolve every control it declares;
/// with MSFS's own template definitions (`asobo`), every node's emissive
/// and visibility code too.
pub fn resolve(model_xml: &Path, asobo: Option<&Path>, sim: &sim::SimState) -> anyhow::Result<(Resolution, Vec<String>)> {
    let lib = xml::Library::load(model_xml)?;
    let e = expand::expand(&lib);
    let mut notes = e.warnings.clone();
    if !lib.duplicates.is_empty() {
        notes.push(format!("templates defined twice (first kept): {}", lib.duplicates.join(", ")));
    }
    let mut res = bind::resolve_all(&e.leaves);
    res.updates = bind::resolve_updates(&e.updates);
    match asobo {
        Some(dir) => {
            let base = xml::Library::load_templates(dir);
            notes.push(format!(
                "MSFS behaviour definitions: {} templates from {} files in {}{}",
                base.templates.len(),
                base.files.len(),
                dir.display(),
                if base.missing.is_empty() { String::new() } else { format!(" ({} files did not parse)", base.missing.len()) }
            ));
            let full = expand::expand_with(&lib, Some(&base));
            res.lights = Some(emissive::resolve(&full.lights, &sim.constants));
            let mut dropped = 0usize;
            res.sound_triggers = full
                .sounds
                .iter()
                .filter(|s| s.action.eq_ignore_ascii_case("Play"))
                .filter_map(|s| {
                    let known = res.bindings.iter().any(|b| b.anim == s.anim) || res.mirrors.iter().any(|(c, _)| c == &s.anim);
                    if !known {
                        dropped += 1;
                        return None;
                    }
                    let dataref = format!("fbw/cockpit/{}", super::rig::sanitize(&s.anim));
                    // `s.normalized_time`/`s.count` are already mutually
                    // exclusive from `expand.rs`'s capture (Asobo's own
                    // templates never set both) -- pass both through as-is;
                    // no more `unwrap_or(0.5)` fabrication for the `Count`
                    // case (W177's finding: that silently turned every
                    // `Count`-based switch, e.g. the speedbrake lever's
                    // Count=3, into a single midpoint click).
                    Some(bind::SoundTrigger {
                        dataref,
                        direction: s.direction.clone(),
                        normalized_time: s.normalized_time,
                        count: s.count,
                        wwise_event: s.wwise_event.clone(),
                    })
                })
                .collect();
            if dropped > 0 {
                notes.push(format!("sound: {dropped} <AnimationTriggers> named an animation with no bound click or mirror (dropped, not written to sound_triggers.txt)"));
            }
        }
        None => notes.push("no MSFS behaviour definitions (--asobo-behaviours): cockpit lights keep their textures".into()),
    }
    Ok((res, notes))
}
