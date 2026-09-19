//! When FlyByWire's sounds play: the package's sound.xml, as MSFS reads it.
//!
//! `<SimVarSounds>` entries play a Wwise event while their `LocalVar` (an
//! `L:` variable) or `SimVar` (with `Index`) is inside `<Range LowerBound
//! UpperBound>`: a `Continuous="true"` sound loops for as long as the value
//! stays inside and stops when it leaves; a `Continuous="false"` one plays once
//! each time the value enters. MSFS's SDK (Sound.xml documentation,
//! SimVarSounds) gives `Continuous` as defaulting to true.
//!
//! Only entries without `<WwiseRTPC>` children are taken. An RTPC drives the
//! sound's volume or pitch from a variable through curves inside the Wwise
//! project (engines, wind, APU spool, packs); those curves are Wwise runtime
//! behaviour this plugin does not have, and a sound played at a fixed level in
//! their place would not be FlyByWire's sound. X-Plane's own engine, wind and
//! rolling sounds stay in their place.
//!
//! `<AvionicSounds>` have no trigger of their own: FlyByWire's scripts play
//! them by name (`Coherent.call('PLAY_INSTRUMENT_SOUND', event)`,
//! LegacySoundManager.ts, FwsSoundManager.ts:374-376).

use crate::extra_backend::procedures::xml;

/// One triggered sound.
#[derive(Clone, Debug, PartialEq)]
pub struct Trigger {
    pub event: String,
    /// The plugin variable name: `L:` names as written, simulator variables
    /// with `:index` when the index is not 0.
    pub variable: String,
    pub is_local: bool,
    pub lower: Option<f64>,
    pub upper: Option<f64>,
    pub continuous: bool,
}

impl Trigger {
    /// Inside the range: at or above the lower bound and at or below the
    /// upper one. With no range, any non-zero value.
    pub fn inside(&self, value: f64) -> bool {
        match (self.lower, self.upper) {
            (None, None) => value != 0.,
            (lower, upper) => lower.map_or(true, |l| value >= l) && upper.map_or(true, |u| value <= u),
        }
    }
}

/// What sound.xml gives: the triggered sounds taken, the names of those left
/// out for their RTPCs, and every avionic sound's event.
#[derive(Debug, Default)]
pub struct SoundXml {
    pub triggers: Vec<Trigger>,
    pub skipped_rtpc: Vec<String>,
    pub avionic_events: Vec<String>,
}

pub fn parse(text: &str) -> Result<SoundXml, String> {
    let root = xml::parse(text)?;
    let mut out = SoundXml::default();
    for section in &root.children {
        match section.name.as_str() {
            "SimVarSounds" => {
                for sound in section.children.iter().filter(|c| c.name == "Sound") {
                    let Some(event) = sound.attr("WwiseEvent") else { continue };
                    if sound.children.iter().any(|c| c.name == "WwiseRTPC") {
                        out.skipped_rtpc.push(event);
                        continue;
                    }
                    let (variable, is_local) = match (sound.attr("LocalVar"), sound.attr("SimVar")) {
                        (Some(l), _) => (l, true),
                        (None, Some(s)) => {
                            let index = sound.attr("Index").and_then(|i| i.trim().parse::<u32>().ok()).unwrap_or(0);
                            (if index > 0 { format!("{s}:{index}") } else { s }, false)
                        }
                        _ => continue,
                    };
                    let range = sound.children.iter().find(|c| c.name == "Range");
                    let bound = |k: &str| range.and_then(|r| r.attr(k)).and_then(|v| v.trim().parse::<f64>().ok());
                    let continuous = sound.attr("Continuous").map_or(true, |c| c.trim().eq_ignore_ascii_case("true"));
                    out.triggers.push(Trigger { event, variable, is_local, lower: bound("LowerBound"), upper: bound("UpperBound"), continuous });
                }
            }
            "AvionicSounds" => {
                out.avionic_events.extend(section.children.iter().filter(|c| c.name == "Sound").filter_map(|s| s.attr("WwiseEvent")));
            }
            _ => {}
        }
    }
    Ok(out)
}

/// What a trigger asks of the player this tick.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Action {
    None,
    PlayOnce,
    StartLoop,
    StopLoop,
}

/// A trigger's state between ticks.
#[derive(Clone, Copy, Debug, Default)]
pub struct TriggerState {
    was_inside: Option<bool>,
}

impl TriggerState {
    /// The action for this tick's value. The first reading only records
    /// where the value is, so nothing already true at start-up fires a
    /// one-shot sound, but a continuous sound whose value starts inside
    /// starts.
    pub fn step(&mut self, trigger: &Trigger, value: f64) -> Action {
        let inside = trigger.inside(value);
        let previous = self.was_inside.replace(inside);
        match (trigger.continuous, previous, inside) {
            (true, None | Some(false), true) => Action::StartLoop,
            (true, Some(true), false) => Action::StopLoop,
            (false, Some(false), true) => Action::PlayOnce,
            _ => Action::None,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const XML: &str = r#"<SoundInfo Version="0.1">
        <SimVarSounds>
            <Sound WwiseData="true" WwiseEvent="CRC_380" NodeName="WIPER_BASE_L" ViewPoint="Inside" LocalVar="A32NX_FWC_CRC">
                <Range LowerBound="0.5" />
            </Sound>
            <Sound WwiseData="true" WwiseEvent="cavcharge" LocalVar="A32NX_FWC_CAVALRY_CHARGE" Continuous="true">
                <Range LowerBound="0.5" />
                <WwiseRTPC LocalVar="A32NX_FWS_AUDIO_VOLUME" Units="number" Index="0" RTPCName="LOCALVAR_FWC_VOLUME" />
            </Sound>
            <Sound WwiseData="true" WwiseEvent="mastercaution" LocalVar="A32NX_FWC_SC" Continuous="false">
                <Range LowerBound="0.5" />
            </Sound>
            <Sound WwiseEvent="Engine_1_Rear" SimVar="TURB ENG N1" Units="percent" Index="1" Continuous="true"/>
        </SimVarSounds>
        <AvionicSounds>
            <Sound WwiseData="true" WwiseEvent="new_retard" NodeName="WIPER_BASE_L" />
        </AvionicSounds>
    </SoundInfo>"#;

    #[test]
    fn sound_xml_triggers_are_read_as_msfs_reads_them() {
        let s = parse(XML).unwrap();
        assert_eq!(s.triggers.len(), 3);
        assert_eq!(s.skipped_rtpc, vec!["cavcharge"]);
        assert_eq!(s.avionic_events, vec!["new_retard"]);
        let crc = &s.triggers[0];
        assert!(crc.continuous && crc.is_local && crc.lower == Some(0.5));
        assert_eq!(s.triggers[2].variable, "TURB ENG N1:1");
    }

    #[test]
    fn continuous_sounds_loop_while_inside_and_one_shots_fire_on_entry() {
        let s = parse(XML).unwrap();
        let (crc, sc) = (&s.triggers[0], &s.triggers[1]);
        let mut a = TriggerState::default();
        assert_eq!(a.step(crc, 0.), Action::None);
        assert_eq!(a.step(crc, 1.), Action::StartLoop);
        assert_eq!(a.step(crc, 1.), Action::None);
        assert_eq!(a.step(crc, 0.), Action::StopLoop);
        let mut b = TriggerState::default();
        // Already set at start: no chime.
        assert_eq!(b.step(sc, 1.), Action::None);
        assert_eq!(b.step(sc, 0.), Action::None);
        assert_eq!(b.step(sc, 1.), Action::PlayOnce);
    }

    #[test]
    fn the_packages_sound_xml_parses() {
        let path = std::path::Path::new(r"D:\Microsoft Flight Simulator 2020\Microsoft Flight Simulator 2020 Packages\Community\flybywire-aircraft-a380-842\SimObjects\AirPlanes\FlyByWire_A380_842\sound\sound.xml");
        let Ok(text) = std::fs::read_to_string(path) else {
            eprintln!("no FlyByWire A380X package here; skipped");
            return;
        };
        let s = parse(&text).unwrap();
        eprintln!("{} triggers, {} with RTPCs left out, {} avionic sounds", s.triggers.len(), s.skipped_rtpc.len(), s.avionic_events.len());
        assert!(s.triggers.iter().any(|t| t.event == "CRC_380" && t.variable == "A32NX_FWC_CRC"));
        assert!(s.avionic_events.iter().any(|e| e == "new_retard"));
    }
}
