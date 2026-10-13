//! Every cockpit control's own `fbw/cockpit/*` dataref, discovered from
//! FlyByWire's own converter output rather than hand-maintained: the
//! converter lists, for each control in the A380X's behaviour XML, the
//! dataref X-Plane's click/lever/knob code keeps
//! (`cockpit_bindings.txt`, next to the installed aircraft).
//!
//! This is a *reachability* catalogue, not a "what does each one do"
//! catalogue: `cockpit_bindings.txt` also records, per line, when a control
//! "changes no FBW variable" (MSFS-local visual state only, e.g. the coffee
//! cup or the footrest) -- those are still listed (so the coverage test
//! below can assert every control is *settable*), but setting them has no
//! effect on the systems, exactly as the real cockpit has none either.

use std::path::Path;

/// One `fbw/cockpit/*` dataref, with whatever `cockpit_bindings.txt` said
/// about it on the line(s) it appeared.
#[derive(Debug, Clone)]
pub struct Control {
    pub dataref: String,
    /// The raw line(s) `cockpit_bindings.txt` gave for this dataref,
    /// joined with `"; "` if it appeared more than once (e.g. a control
    /// with both a click command and "no click spot" line).
    pub notes: String,
}

/// The default installed location the task names. Not a build-time
/// dependency (the emulator still builds and runs without it -- see
/// [`list`]'s doc comment); only read at call time.
pub const DEFAULT_COCKPIT_BINDINGS_PATH: &str = "D:\\Steam Games\\steamapps\\common\\X-Plane 12\\Aircraft\\FlyByWire A380X\\cockpit_bindings.txt";

/// Parses every `fbw/cockpit/...` dataref name out of `text`
/// (`cockpit_bindings.txt`'s own format: free-text lines, each possibly
/// mentioning one dataref), deduplicated, in first-seen order.
pub fn parse(text: &str) -> Vec<Control> {
    let mut order: Vec<String> = Vec::new();
    let mut notes: std::collections::HashMap<String, Vec<String>> = std::collections::HashMap::new();
    for line in text.lines() {
        let mut rest = line;
        while let Some(at) = rest.find("fbw/cockpit/") {
            let tail = &rest[at..];
            let end = tail
                .find(|c: char| !(c.is_ascii_alphanumeric() || c == '_' || c == '/'))
                .unwrap_or(tail.len());
            let dataref = tail[..end].to_owned();
            if !dataref.is_empty() && !dataref.ends_with('/') {
                if !notes.contains_key(&dataref) {
                    order.push(dataref.clone());
                }
                notes.entry(dataref).or_default().push(line.trim().to_owned());
            }
            rest = &tail[end..];
        }
    }
    order
        .into_iter()
        .map(|dataref| {
            let n = notes.remove(&dataref).unwrap_or_default();
            Control { dataref, notes: n.join("; ") }
        })
        .collect()
}

/// Reads and parses `path` (default: [`DEFAULT_COCKPIT_BINDINGS_PATH`]).
/// Returns an empty list (not an error) if the file is not present in this
/// environment -- callers that need the file to exist should check
/// `list(None).is_empty()` and report it themselves; a missing file must
/// not panic a test bench that has nothing to do with the installed
/// aircraft's file layout.
pub fn list(path: Option<&Path>) -> Vec<Control> {
    let path = path.map(Path::to_path_buf).unwrap_or_else(|| Path::new(DEFAULT_COCKPIT_BINDINGS_PATH).to_path_buf());
    match std::fs::read_to_string(&path) {
        Ok(text) => parse(&text),
        Err(_) => Vec::new(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_a_representative_line() {
        let controls = parse(
            "BIGARMREST_CPT_TILT_CLICK: lever, keeps fbw/cockpit/BIGARMREST_CPT_TILT_CLICK\n\
             COVER_RMP_1_STBY_NAV: toggles fbw/cockpit/ie/A380X_PED_RMP_1_STBY_RAD_NAV_COVER between 1 and 0\n",
        );
        assert_eq!(controls.len(), 2);
        assert_eq!(controls[0].dataref, "fbw/cockpit/BIGARMREST_CPT_TILT_CLICK");
        assert_eq!(controls[1].dataref, "fbw/cockpit/ie/A380X_PED_RMP_1_STBY_RAD_NAV_COVER");
    }

    #[test]
    fn dedupes_a_dataref_seen_on_two_lines() {
        let controls = parse(
            "ANIM_DOOR_M1L_CLICK: click command running the control's MSFS code in SASL (fires sim/flight_controls/door_toggle_1) fbw/cockpit/ANIM_DOOR_M1L_CLICK\n\
             ANIM_DOOR_M1L_CLICK: no click spot fbw/cockpit/ANIM_DOOR_M1L_CLICK\n",
        );
        assert_eq!(controls.len(), 1);
        assert!(controls[0].notes.contains("no click spot"));
    }
}
