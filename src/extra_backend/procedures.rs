//! The aircraft preset procedures: FlyByWire's ProcedureStep.hpp and
//! PresetProcedures.hpp, read from their own XML
//! (config/a380x/a380-842/aircraft_preset_procedures.xml, the file
//! Gauge_Extra_Backend.cpp:39 names).
//!
//! FlyByWire re-reads the file on every request so it can be edited while the
//! simulator runs (PresetProcedures.hpp:71-95). The plugin carries the file
//! from the FlyByWire checkout it is built against, and reads a copy next to
//! the plugin instead when one is there (see `AircraftPresets`).

/// The file FlyByWire ships, as built into the plugin.
pub const PROCEDURES_XML: &str = include_str!(
    "../../../fbw-aircraft/fbw-a380x/src/base/flybywire-aircraft-a380-842/config/a380x/a380-842/aircraft_preset_procedures.xml"
);

/// StepType flags (ProcedureStep.hpp:31-48).
pub const ACTION: u32 = 0b00001;
pub const CONDITION: u32 = 0b00010;
pub const NORMAL_MODE: u32 = 0b00100;
pub const EXPEDITED_MODE: u32 = 0b01000;
pub const EXPEDITED_DELAY: u32 = 0b10000;
pub const STEP: u32 = ACTION | NORMAL_MODE | EXPEDITED_MODE;
pub const PROC: u32 = ACTION | NORMAL_MODE;
pub const NOEX: u32 = STEP | EXPEDITED_DELAY;
pub const EXON: u32 = ACTION | EXPEDITED_MODE;
pub const COND: u32 = CONDITION | NORMAL_MODE | EXPEDITED_MODE;
pub const NCON: u32 = CONDITION | NORMAL_MODE;
pub const ECON: u32 = CONDITION | EXPEDITED_MODE;

/// ProcedureStep::StepTypeMap (ProcedureStep.hpp:114-122).
pub fn step_type(name: &str) -> Option<u32> {
    Some(match name {
        "STEP" => STEP,
        "PROC" => PROC,
        "NOEX" => NOEX,
        "EXON" => EXON,
        "COND" => COND,
        "NCON" => NCON,
        "ECON" => ECON,
        _ => return None,
    })
}

#[derive(Clone, Debug, PartialEq)]
pub struct ProcedureStep {
    pub description: String,
    pub step_type: u32,
    /// Milliseconds.
    pub delay_after: f64,
    pub expected_state_check_code: String,
    pub action_code: String,
}

/// The procedure names, in PresetProcedures::initializeProcedureListMap's
/// order (PresetProcedures.hpp:181-192).
const PROCEDURES: [&str; 8] = [
    "POWERUP_CONFIG_ON",
    "POWERUP_CONFIG_OFF",
    "PUSHBACK_CONFIG_ON",
    "PUSHBACK_CONFIG_OFF",
    "TAXI_CONFIG_ON",
    "TAXI_CONFIG_OFF",
    "TAKEOFF_CONFIG_ON",
    "TAKEOFF_CONFIG_OFF",
];

/// Which procedures each preset runs (PresetProcedures.hpp:198-213): 1 cold
/// and dark, 2 powered, 3 ready for pushback, 4 ready for taxi, 5 ready for
/// takeoff.
const PRESETS: [[&str; 4]; 5] = [
    ["TAKEOFF_CONFIG_OFF", "TAXI_CONFIG_OFF", "PUSHBACK_CONFIG_OFF", "POWERUP_CONFIG_OFF"],
    ["TAKEOFF_CONFIG_OFF", "TAXI_CONFIG_OFF", "PUSHBACK_CONFIG_OFF", "POWERUP_CONFIG_ON"],
    ["TAKEOFF_CONFIG_OFF", "TAXI_CONFIG_OFF", "POWERUP_CONFIG_ON", "PUSHBACK_CONFIG_ON"],
    ["TAKEOFF_CONFIG_OFF", "POWERUP_CONFIG_ON", "PUSHBACK_CONFIG_ON", "TAXI_CONFIG_ON"],
    ["POWERUP_CONFIG_ON", "PUSHBACK_CONFIG_ON", "TAXI_CONFIG_ON", "TAKEOFF_CONFIG_ON"],
];

/// PresetProcedures::getProcedure: the steps of preset `id` (1-5) from the
/// XML text, or why not.
pub fn preset(xml: &str, id: i64) -> Result<Vec<ProcedureStep>, String> {
    if !(1..=5).contains(&id) {
        return Err(format!("AircraftPresets: The procedure ID {id} is not valid. Valid IDs are 1-5."));
    }
    let procedures = parse(xml)?;
    let mut steps = Vec::new();
    for name in PRESETS[(id - 1) as usize] {
        if let Some((_, list)) = procedures.iter().find(|(n, _)| n == name) {
            steps.extend(list.iter().cloned());
        }
    }
    Ok(steps)
}

/// PresetProcedures::processProcedures: every known procedure's valid steps.
pub fn parse(xml: &str) -> Result<Vec<(String, Vec<ProcedureStep>)>, String> {
    let root = xml::parse(xml)?;
    let mut out: Vec<(String, Vec<ProcedureStep>)> = PROCEDURES.iter().map(|n| (n.to_string(), Vec::new())).collect();
    for procedure in &root.children {
        let name = procedure.attr("Name").unwrap_or_default();
        // "The procedure {} is not valid. Skipping the whole procedure."
        let Some((_, steps)) = out.iter_mut().find(|(n, _)| *n == name) else { continue };
        for step in &procedure.children {
            // Invalid step type or a negative delay skip the step.
            let Some(step_type) = step.attr("Type").and_then(|t| step_type(&t)) else { continue };
            // tinyxml2's IntAttribute: 0 when missing or not a number.
            let delay = step.attr("Delay").and_then(|d| d.trim().parse::<i64>().ok()).unwrap_or(0);
            if delay < 0 {
                continue;
            }
            let text = |tag: &str| step.children.iter().find(|c| c.name == tag).map(|c| c.text.clone()).unwrap_or_default();
            steps.push(ProcedureStep {
                description: step.attr("Name").unwrap_or_default(),
                step_type,
                delay_after: delay as f64,
                expected_state_check_code: text("Condition"),
                action_code: text("Action"),
            });
        }
    }
    Ok(out)
}

/// Just enough XML for this file: elements, attributes, text, comments and
/// the five predefined entities. Also reads the package's sound.xml
/// (sound/triggers.rs).
pub(crate) mod xml {
    #[derive(Debug, Default)]
    pub struct Element {
        pub name: String,
        pub attrs: Vec<(String, String)>,
        pub children: Vec<Element>,
        pub text: String,
    }

    impl Element {
        pub fn attr(&self, name: &str) -> Option<String> {
            self.attrs.iter().find(|(k, _)| k == name).map(|(_, v)| v.clone())
        }
    }

    pub fn unescape(s: &str) -> String {
        s.replace("&lt;", "<").replace("&gt;", ">").replace("&quot;", "\"").replace("&apos;", "'").replace("&amp;", "&")
    }

    /// The root element.
    pub fn parse(text: &str) -> Result<Element, String> {
        let mut stack: Vec<Element> = vec![Element::default()];
        let mut rest = text;
        while let Some(lt) = rest.find('<') {
            let before = &rest[..lt];
            if let Some(top) = stack.last_mut() {
                top.text.push_str(&unescape(before));
            }
            rest = &rest[lt..];
            if let Some(after) = rest.strip_prefix("<!--") {
                let end = after.find("-->").ok_or("unterminated comment")?;
                rest = &after[end + 3..];
            } else if rest.starts_with("<?") {
                let end = rest.find("?>").ok_or("unterminated declaration")?;
                rest = &rest[end + 2..];
            } else if let Some(after) = rest.strip_prefix("</") {
                let end = after.find('>').ok_or("unterminated end tag")?;
                let name = after[..end].trim();
                rest = &after[end + 1..];
                let done = stack.pop().ok_or("unbalanced end tag")?;
                if done.name != name {
                    return Err(format!("</{name}> closes <{}>", done.name));
                }
                stack.last_mut().ok_or("unbalanced end tag")?.children.push(done);
            } else {
                let end = tag_end(rest).ok_or("unterminated tag")?;
                let inner = &rest[1..end];
                rest = &rest[end + 1..];
                let (inner, empty) = match inner.strip_suffix('/') {
                    Some(i) => (i, true),
                    None => (inner, false),
                };
                let element = start_tag(inner)?;
                if empty {
                    stack.last_mut().ok_or("no parent")?.children.push(element);
                } else {
                    stack.push(element);
                }
            }
        }
        if stack.len() != 1 {
            return Err("unclosed elements".into());
        }
        let mut document = stack.pop().unwrap_or_default();
        document.children.pop().ok_or_else(|| "no root element".to_string())
    }

    /// The `>` ending a start tag, outside quoted attribute values.
    fn tag_end(s: &str) -> Option<usize> {
        let mut quote = None;
        for (i, c) in s.char_indices() {
            match (quote, c) {
                (None, '"' | '\'') => quote = Some(c),
                (Some(q), c) if c == q => quote = None,
                (None, '>') => return Some(i),
                _ => {}
            }
        }
        None
    }

    fn start_tag(inner: &str) -> Result<Element, String> {
        let inner = inner.trim();
        let name_end = inner.find(char::is_whitespace).unwrap_or(inner.len());
        let mut element = Element { name: inner[..name_end].to_string(), ..Default::default() };
        let mut rest = inner[name_end..].trim_start();
        while !rest.is_empty() {
            let eq = rest.find('=').ok_or_else(|| format!("attribute without value in <{}>", element.name))?;
            let key = rest[..eq].trim().to_string();
            let after = rest[eq + 1..].trim_start();
            let quote = after.chars().next().ok_or("missing attribute value")?;
            let close = after[1..].find(quote).ok_or("unterminated attribute value")?;
            element.attrs.push((key, unescape(&after[1..1 + close])));
            rest = after[close + 2..].trim_start();
        }
        Ok(element)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn flybywire_procedures_parse_into_their_presets() {
        let procedures = parse(PROCEDURES_XML).unwrap();
        for (name, steps) in &procedures {
            assert!(!steps.is_empty(), "{name} has no steps");
        }
        let power_on = &procedures.iter().find(|(n, _)| n == "POWERUP_CONFIG_ON").unwrap().1;
        // aircraft_preset_procedures.xml:50-53.
        assert_eq!(power_on[0].description, "BAT1 On");
        assert_eq!(power_on[0].step_type, STEP);
        assert_eq!(power_on[0].delay_after, 1000.);
        assert_eq!(power_on[0].expected_state_check_code, "(L:A32NX_OVHD_ELEC_BAT_1_PB_IS_AUTO)");
        assert_eq!(power_on[0].action_code, "1 (>L:A32NX_OVHD_ELEC_BAT_1_PB_IS_AUTO)");
        // Ready for takeoff is the four ON procedures in order.
        let takeoff = preset(PROCEDURES_XML, 5).unwrap();
        let count = |n: &str| procedures.iter().find(|(p, _)| p == n).unwrap().1.len();
        assert_eq!(
            takeoff.len(),
            count("POWERUP_CONFIG_ON") + count("PUSHBACK_CONFIG_ON") + count("TAXI_CONFIG_ON") + count("TAKEOFF_CONFIG_ON")
        );
        assert!(preset(PROCEDURES_XML, 6).is_err());
        // Every step type in the file is one FlyByWire knows.
        let types: Vec<u32> = procedures.iter().flat_map(|(_, s)| s.iter().map(|s| s.step_type)).collect();
        assert!(types.contains(&COND) && types.contains(&NCON) && types.contains(&PROC));
    }
}
