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
//!
//! A `<Sound>` may carry one or more `<Requires>` children: a second
//! variable/range that gates the whole entry, independent of its own
//! `LocalVar`/`SimVar` range (MSFS SDK, SimVarSounds/Requires). The package's
//! `sound.xml` uses this on 30 entries, e.g. `cabin_crew_seats_landing`
//! (`A32NX_CABIN_READY`) additionally `Requires` `AIRLINER_FLIGHT_PHASE` in
//! `[6, 6]` (descent), so the "cabin crew, seats for landing" PA only plays
//! during that phase even though `A32NX_CABIN_READY` alone can already be 1
//! on the ground (the package's `runway.FLT` sets it there at spawn).

use crate::extra_backend::procedures::xml;

/// A `<Requires>` gate: a second variable that must also be inside its range
/// for the trigger to be considered inside its own.
#[derive(Clone, Debug, PartialEq)]
pub struct Require {
    pub variable: String,
    pub is_local: bool,
    pub lower: Option<f64>,
    pub upper: Option<f64>,
}

impl Require {
    /// At or above the lower bound and at or below the upper one. With no
    /// range, any non-zero value (same rule as [`Trigger::inside`]).
    pub fn holds(&self, value: f64) -> bool {
        inside_range(self.lower, self.upper, value)
    }
}

fn inside_range(lower: Option<f64>, upper: Option<f64>, value: f64) -> bool {
    match (lower, upper) {
        (None, None) => value != 0.,
        (lower, upper) => lower.map_or(true, |l| value >= l) && upper.map_or(true, |u| value <= u),
    }
}

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
    /// Every `<Requires>` gate on this entry; all must hold, alongside this
    /// trigger's own range, for it to count as inside (empty: no gate).
    pub requires: Vec<Require>,
}

impl Trigger {
    /// Inside the range: at or above the lower bound and at or below the
    /// upper one. With no range, any non-zero value. This is the entry's own
    /// variable only; combine with [`Require::holds`] for every entry in
    /// [`Trigger::requires`] to get what MSFS actually plays on.
    pub fn inside(&self, value: f64) -> bool {
        inside_range(self.lower, self.upper, value)
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
                    let requires = sound
                        .children
                        .iter()
                        .filter(|c| c.name == "Requires")
                        .filter_map(|r| {
                            let (variable, is_local) = match (r.attr("LocalVar"), r.attr("SimVar")) {
                                (Some(l), _) => (l, true),
                                (None, Some(s)) => {
                                    let index = r.attr("Index").and_then(|i| i.trim().parse::<u32>().ok()).unwrap_or(0);
                                    (if index > 0 { format!("{s}:{index}") } else { s }, false)
                                }
                                _ => return None,
                            };
                            let range = r.children.iter().find(|c| c.name == "Range");
                            let bound = |k: &str| range.and_then(|rg| rg.attr(k)).and_then(|v| v.trim().parse::<f64>().ok());
                            Some(Require { variable, is_local, lower: bound("LowerBound"), upper: bound("UpperBound") })
                        })
                        .collect();
                    out.triggers.push(Trigger { event, variable, is_local, lower: bound("LowerBound"), upper: bound("UpperBound"), continuous, requires });
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
    /// The action for this tick's value. `requires_hold` is whether every
    /// [`Trigger::requires`] gate currently holds (`true` when the entry has
    /// none): with it false, the entry is treated as outside regardless of
    /// its own value, exactly as MSFS would not play a `<Sound>` whose
    /// `<Requires>` condition is not met. The first reading only records
    /// where the value is, so nothing already true at start-up fires a
    /// one-shot sound, but a continuous sound whose value starts inside
    /// starts.
    pub fn step(&mut self, trigger: &Trigger, value: f64, requires_hold: bool) -> Action {
        let inside = requires_hold && trigger.inside(value);
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
        assert_eq!(a.step(crc, 0., true), Action::None);
        assert_eq!(a.step(crc, 1., true), Action::StartLoop);
        assert_eq!(a.step(crc, 1., true), Action::None);
        assert_eq!(a.step(crc, 0., true), Action::StopLoop);
        let mut b = TriggerState::default();
        // Already set at start: no chime.
        assert_eq!(b.step(sc, 1., true), Action::None);
        assert_eq!(b.step(sc, 0., true), Action::None);
        assert_eq!(b.step(sc, 1., true), Action::PlayOnce);
    }

    /// Real bug (X-Plane 12 session, 2026): the cabin "prepare for landing"
    /// PA (`cabin_crew_seats_landing`, `A32NX_CABIN_READY`) played nonstop on
    /// the ground at spawn. Root cause part 1: the package's `runway.FLT`
    /// sets `A32NX_CABIN_READY=1` at spawn, and the entry's `<Requires
    /// LocalVar="AIRLINER_FLIGHT_PHASE"><Range LowerBound="6" UpperBound="6"
    /// /></Requires>` (descent only) was parsed and then silently dropped:
    /// with no gate, `A32NX_CABIN_READY` reaching 1 fired the PA regardless
    /// of flight phase. This checks the gate is honoured both ways: held low
    /// while the phase is wrong, and firing exactly once when the phase
    /// reaches the required range while the main variable is already set.
    #[test]
    fn a_requires_gate_blocks_the_trigger_until_its_own_condition_holds() {
        const XML: &str = r#"<SoundInfo Version="0.1">
            <SimVarSounds>
                <Sound WwiseData="true" WwiseEvent="cabin_crew_seats_landing" LocalVar="A32NX_CABIN_READY" NodeName="PEDALS_LEFT" Continuous="false">
                    <Range LowerBound="1" />
                    <Requires LocalVar="AIRLINER_FLIGHT_PHASE">
                        <Range LowerBound="6" UpperBound="6" />
                    </Requires>
                </Sound>
            </SimVarSounds>
        </SoundInfo>"#;
        let s = parse(XML).unwrap();
        let pa = &s.triggers[0];
        assert_eq!(pa.requires.len(), 1);
        assert_eq!(pa.requires[0].variable, "AIRLINER_FLIGHT_PHASE");

        // Cabin already flagged ready (as at spawn on the ground, phase 1):
        // the gate does not hold, so the PA must never fire, no matter how
        // long the ground sits at CABIN_READY == 1.
        let mut st = TriggerState::default();
        assert_eq!(st.step(pa, 1., false), Action::None);
        assert_eq!(st.step(pa, 1., false), Action::None);
        assert_eq!(st.step(pa, 1., false), Action::None);

        // Descent begins, flight phase reaches 6 while cabin is still ready:
        // the gate now holds, so the combined condition has a fresh 0->1
        // edge and fires exactly once.
        assert_eq!(st.step(pa, 1., true), Action::PlayOnce);
        assert_eq!(st.step(pa, 1., true), Action::None, "one-shot: must not re-fire while still held");

        // Leaving the phase and coming back must not replay it either
        // (Continuous="false" fires once per entry, not once per tick).
        assert_eq!(st.step(pa, 1., false), Action::None);
        assert_eq!(st.step(pa, 1., true), Action::PlayOnce);
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
