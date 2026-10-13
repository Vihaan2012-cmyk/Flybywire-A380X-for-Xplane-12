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
//! `<AnimationSounds>` (cockpit switch/button clicks) have no trigger in
//! sound.xml either, and unlike `<AvionicSounds>` almost none carry a
//! `NodeName` to say which control: which animation fires which one is
//! authored in MSFS's *model behaviour XML*, compiled through Asobo's own
//! `ASOBO_GT_AnimTriggers_*` templates into `<AnimationTriggers>` blocks
//! that never reach sound.xml at all. This reader still records every
//! `<AnimationSounds>` event name (`animation_events`) for visibility, but
//! the pairing this plugin actually plays from comes from the converter's
//! `sound_triggers.txt` instead (see `sound::anim_triggers`), built by
//! walking that XML. `<GroundSounds>` (wing/fuselage scrape, wheel
//! touchdown) are not simvar-triggered at all — MSFS's own collision system
//! fires them from wheel-contact telemetry (`WwiseRTPC Derived="true"`),
//! which this plugin does not model; `<MiscellaneousSounds>`'s one entry
//! (`AP_PREFLIGHT_CHECK_OVER`) has no trigger either, name-played like
//! `<AvionicSounds>`. Both are recorded (`ground_events`, `misc_events`) but
//! not wired to anything.
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
    /// `ViewPoint="Outside"`: heard from outside the aircraft only (the
    /// APU's exhaust). Every other sound is a flight-deck sound.
    pub outside: bool,
    pub inside_only: bool,
    pub rtpcs: Vec<RtpcInput>,
    pub node: String,
    pub fade_out_s: f64,
    pub stop_delay_s: f64,
    pub cone_heading: Option<f64>,
    pub always: bool,
}

#[derive(Clone, Debug, PartialEq)]
pub struct RtpcInput {
    pub game_param: u32,
    pub name: String,
    pub variable: String,
    pub is_local: bool,
    pub index: u32,
    pub units: String,
    pub derived: bool,
}

fn variable_of(el: &xml::Element) -> Option<(String, bool)> {
    match (el.attr("LocalVar"), el.attr("SimVar")) {
        (Some(l), _) => Some((l, true)),
        (None, Some(s)) => {
            let index = el.attr("Index").and_then(|i| i.trim().parse::<u32>().ok()).unwrap_or(0);
            Some((if index > 0 { format!("{s}:{index}") } else { s }, false))
        }
        _ => None,
    }
}

fn rtpc_inputs(sound: &xml::Element) -> Vec<RtpcInput> {
    sound
        .children
        .iter()
        .filter(|c| c.name == "WwiseRTPC")
        .filter_map(|c| {
            let name = c.attr("RTPCName")?;
            let is_local = c.attr("LocalVar").is_some();
            let variable = c.attr("LocalVar").or_else(|| c.attr("SimVar"))?;
            Some(RtpcInput {
                game_param: crate::sound::wwise::short_id(&name),
                name,
                variable,
                is_local,
                index: c.attr("Index").and_then(|i| i.trim().parse::<u32>().ok()).unwrap_or(0),
                units: c.attr("Units").unwrap_or_default(),
                derived: c.attr("Derived").is_some_and(|d| d.trim().eq_ignore_ascii_case("true")),
            })
        })
        .collect()
}

fn number(el: &xml::Element, key: &str) -> Option<f64> {
    el.attr(key).and_then(|v| v.trim().parse::<f64>().ok())
}

impl Trigger {
    /// Inside the range: at or above the lower bound and at or below the
    /// upper one. With no range, any non-zero value. This is the entry's own
    /// variable only; combine with [`Require::holds`] for every entry in
    /// [`Trigger::requires`] to get what MSFS actually plays on.
    pub fn inside(&self, value: f64) -> bool {
        self.always || inside_range(self.lower, self.upper, value)
    }

    pub fn exterior(&self) -> bool {
        let n = self.node.to_ascii_uppercase();
        self.outside || ["FX_", "SOUND_ENGINE", "SOUND_PACK", "SOUND_APU", "WHEELS", "WING_", "TAIL_", "AILERON"].iter().any(|p| n.starts_with(p))
    }

    pub fn index(&self) -> Option<u32> {
        let n = self.node.to_ascii_uppercase();
        n.strip_prefix("SOUND_ENGINE_")
            .and_then(|i| i.parse().ok())
            .or_else(|| self.variable.rsplit_once(':').and_then(|(_, i)| i.trim().parse().ok()))
    }
}

#[derive(Debug, Default)]
pub struct SoundXml {
    pub triggers: Vec<Trigger>,
    pub avionic_events: Vec<String>,
    /// `<AnimationSounds>` event names, recorded for visibility only: which
    /// animation plays which is not in sound.xml (see the module doc).
    pub animation_events: Vec<String>,
    /// `<GroundSounds>` event names (wheel/wing/fuselage contact): recorded
    /// only, MSFS's own collision system fires these, not a simvar.
    pub ground_events: Vec<String>,
    /// `<MiscellaneousSounds>` event names: name-triggered, same shape as
    /// `avionic_events`.
    pub misc_events: Vec<String>,
}

const XPLANE_MAKES_ITS_OWN: &[&str] = &["rain_sound", "wind_sound"];

pub fn parse(text: &str) -> Result<SoundXml, String> {
    let root = xml::parse(text)?;
    let mut out = SoundXml::default();
    for section in &root.children {
        match section.name.as_str() {
            "SimVarSounds" | "AlwaysPlaySounds" => {
                let always = section.name != "SimVarSounds";
                for sound in section.children.iter().filter(|c| c.name == "Sound") {
                    let Some(event) = sound.attr("WwiseEvent") else { continue };
                    if XPLANE_MAKES_ITS_OWN.contains(&event.as_str()) {
                        continue;
                    }
                    let (variable, is_local) = match variable_of(sound) {
                        Some(v) => v,
                        None if always => (String::new(), false),
                        None => continue,
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
                    let outside = sound.attr("ViewPoint").is_some_and(|v| v.trim().eq_ignore_ascii_case("Outside"));
                    let inside_only = sound.attr("ViewPoint").is_some_and(|v| v.trim().eq_ignore_ascii_case("Inside"));
                    out.triggers.push(Trigger {
                        event,
                        variable,
                        is_local,
                        lower: bound("LowerBound"),
                        upper: bound("UpperBound"),
                        continuous: continuous || always,
                        requires,
                        outside,
                        inside_only,
                        rtpcs: rtpc_inputs(sound),
                        node: sound.attr("NodeName").unwrap_or_default(),
                        fade_out_s: number(sound, "FadeOutTime").unwrap_or(0.),
                        stop_delay_s: number(sound, "ContinuousStopDelay").unwrap_or(0.),
                        cone_heading: number(sound, "ConeHeading"),
                        always,
                    });
                }
            }
            "AvionicSounds" => {
                out.avionic_events.extend(section.children.iter().filter(|c| c.name == "Sound").filter_map(|s| s.attr("WwiseEvent")));
            }
            "AnimationSounds" => {
                out.animation_events.extend(section.children.iter().filter(|c| c.name == "Sound").filter_map(|s| s.attr("WwiseEvent")));
            }
            "GroundSounds" => {
                out.ground_events.extend(section.children.iter().filter(|c| c.name == "Sound").filter_map(|s| s.attr("WwiseEvent")));
            }
            "MiscellaneousSounds" => {
                out.misc_events.extend(section.children.iter().filter(|c| c.name == "Sound").filter_map(|s| s.attr("WwiseEvent")));
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
    /// Whether the entry's own variable was inside its range last tick,
    /// regardless of its gates: a one-shot fires on this edge only.
    value_was_inside: Option<bool>,
    last_value: Option<f64>,
    still_ticks: u32,
}

const STILL_TICKS_BEFORE_STOP: u32 = 10;

impl Trigger {
    pub fn plays_while_moving(&self) -> bool {
        self.continuous && self.lower.is_none() && self.upper.is_none() && !self.rtpcs.is_empty() && self.rtpcs.iter().all(|r| r.derived)
    }
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
    ///
    /// A one-shot (`Continuous="false"`) fires when its *own* variable enters
    /// its range while every gate holds at that moment -- a `<Requires>` is an
    /// extra condition on the trigger, not a trigger of its own. Firing on a
    /// gate's edge as well replayed the touchdown thumps (`smoothtouch380`,
    /// `cabtouchsmooth1`: `SIM ON GROUND`, gated on the radio vertical speed
    /// being -150..-50 ft/min) every few seconds on a parked aircraft, each
    /// time X-Plane's radio altimeter jitter flicked the filtered rate into
    /// that band. A loop still runs exactly while value and gates both hold.
    pub fn step(&mut self, trigger: &Trigger, value: f64, requires_hold: bool) -> Action {
        let moved = self.last_value.is_some_and(|last| last != value);
        self.last_value = Some(value);
        self.still_ticks = if moved { 0 } else { self.still_ticks.saturating_add(1) };
        let moving = moved || self.still_ticks < STILL_TICKS_BEFORE_STOP && self.was_inside == Some(true);
        let value_inside = if trigger.plays_while_moving() { moving } else { trigger.inside(value) };
        let inside = requires_hold && value_inside;
        let previous = self.was_inside.replace(inside);
        let value_previous = self.value_was_inside.replace(value_inside);
        match (trigger.continuous, previous, inside) {
            (true, None | Some(false), true) => Action::StartLoop,
            (true, Some(true), false) => Action::StopLoop,
            (false, _, true) if value_previous == Some(false) => Action::PlayOnce,
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
        assert_eq!(s.triggers.len(), 4);
        assert_eq!(s.avionic_events, vec!["new_retard"]);
        let crc = &s.triggers[0];
        assert!(crc.continuous && crc.is_local && crc.lower == Some(0.5) && crc.inside_only);
        let cav = &s.triggers[1];
        assert_eq!(cav.rtpcs.len(), 1);
        assert_eq!((cav.rtpcs[0].variable.as_str(), cav.rtpcs[0].game_param), ("A32NX_FWS_AUDIO_VOLUME", crate::sound::wwise::short_id("LOCALVAR_FWC_VOLUME")));
        assert_eq!(s.triggers[3].variable, "TURB ENG N1:1");
        assert_eq!(s.triggers[3].index(), Some(1));
    }

    #[test]
    fn engine_sounds_are_exterior_and_know_their_engine() {
        let s = parse(
            r#"<SoundInfo><SimVarSounds>
                <Sound WwiseEvent="Engine_2_Front" NodeName="SOUND_ENGINE_2" LocalVar="A32NX_ENGINE_N1:2" Continuous="true" FadeOutTime="2">
                    <Range LowerBound="0" />
                    <WwiseRTPC LocalVar="A32NX_ENGINE_N1:2" RTPCName="SIMVAR_TURB_ENG_N1" />
                    <WwiseRTPC LocalVar="A32NX_ENGINE_N1:2" Derived="true" RTPCName="SIMVAR_TURB_ENG_N1_DERIVED" />
                </Sound>
                <Sound WwiseEvent="TR380" NodeName="SPEAKER_OHVD_LP" LocalVar="A32NX_ELEC_AC_1_BUS_IS_POWERED"/>
            </SimVarSounds><AlwaysPlaySounds><Sound WwiseEvent="rattle_kinematic"/></AlwaysPlaySounds></SoundInfo>"#,
        )
        .unwrap();
        let e = &s.triggers[0];
        assert!(e.exterior() && e.index() == Some(2) && e.fade_out_s == 2.);
        assert!(e.rtpcs[1].derived);
        assert!(!s.triggers[1].exterior());
        let always = &s.triggers[2];
        assert!(always.always && always.continuous && always.inside(0.));
    }

    #[test]
    fn a_seat_motor_runs_only_while_the_seat_moves() {
        let xml = r#"<SoundInfo><SimVarSounds>
            <Sound WwiseData="true" WwiseEvent="seatmotorfwd" FadeOutType="1" FadeOutTime="0.1" NodeName="SEAT_CPT_MOVE_FWD_AFT" LocalVar="SEAT_CPT_MOVE_FWD_AFT" Continuous="true">
                <WwiseRTPC LocalVar="SEAT_CPT_MOVE_FWD_AFT" Derived="true" RTPCName="LOCALVAR_SEAT_POS" />
            </Sound>
        </SimVarSounds></SoundInfo>"#;
        let s = parse(xml).unwrap();
        let seat = &s.triggers[0];
        assert!(seat.plays_while_moving());
        let mut st = TriggerState::default();
        for _ in 0..30 {
            assert_eq!(st.step(seat, 50., true), Action::None, "a seat parked mid-travel is silent");
        }
        assert_eq!(st.step(seat, 50.2, true), Action::StartLoop);
        assert_eq!(st.step(seat, 50.4, true), Action::None);
        let stopped = (0..STILL_TICKS_BEFORE_STOP + 2).map(|_| st.step(seat, 50.4, true)).collect::<Vec<_>>();
        assert!(stopped.contains(&Action::StopLoop), "the motor stops once the seat does");
        let engine = &parse(XML).unwrap().triggers[0];
        assert!(!engine.plays_while_moving());
    }

    #[test]
    fn continuous_sounds_loop_while_inside_and_one_shots_fire_on_entry() {
        let s = parse(XML).unwrap();
        let (crc, sc) = (&s.triggers[0], &s.triggers[2]);
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

        // The phase reaching 6 on its own does not fire it: a `<Requires>` is
        // a condition on the trigger, not a trigger (see `step`'s doc -- gate
        // edges firing one-shots replayed the touchdown thump every few
        // seconds on a parked aircraft).
        assert_eq!(st.step(pa, 1., true), Action::None);

        // The cabin reporting ready (CABIN_READY 0 -> 1) during descent, gate
        // held: fires exactly once.
        assert_eq!(st.step(pa, 0., true), Action::None);
        assert_eq!(st.step(pa, 1., true), Action::PlayOnce);
        assert_eq!(st.step(pa, 1., true), Action::None, "one-shot: must not re-fire while still held");
    }

    #[test]
    fn a_touchdown_thump_fires_on_touchdown_and_never_from_a_jittering_gate_while_parked() {
        const XML: &str = r#"<SoundInfo Version="0.1">
            <SimVarSounds>
                <Sound WwiseEvent="smoothtouch380" WwiseData="true" Continuous="false" SimVar="SIM ON GROUND" Units="BOOLEAN" Index="0">
                    <Range LowerBound="1.0" />
                    <Requires LocalVar="A32NX_AUTOPILOT_H_DOT_RADIO" Units="FEET PER MINUTE" Index="0">
                        <Range LowerBound="-150" UpperBound="-50" />
                    </Requires>
                </Sound>
            </SimVarSounds>
        </SoundInfo>"#;
        let s = parse(XML).unwrap();
        let t = &s.triggers[0];
        let mut st = TriggerState::default();
        // Airborne, descending gently: nothing yet.
        assert_eq!(st.step(t, 0., true), Action::None);
        // Touchdown with the rate inside the band: one thump.
        assert_eq!(st.step(t, 1., true), Action::PlayOnce);
        // Parked: the radio rate jitters in and out of the band for ever.
        for i in 0..50 {
            assert_eq!(st.step(t, 1., i % 3 == 0), Action::None, "tick {i}: a parked aircraft must not thump");
        }
    }

    #[test]
    fn the_packages_sound_xml_parses() {
        let path = std::path::Path::new(r"D:\Microsoft Flight Simulator 2020\Microsoft Flight Simulator 2020 Packages\Community\flybywire-aircraft-a380-842\SimObjects\AirPlanes\FlyByWire_A380_842\sound\sound.xml");
        let Ok(text) = std::fs::read_to_string(path) else {
            eprintln!("no FlyByWire A380X package here; skipped");
            return;
        };
        let s = parse(&text).unwrap();
        eprintln!(
            "{} triggers, {} with RTPCs, {} avionic sounds, {} animation sounds, {} ground sounds, {} misc sounds",
            s.triggers.len(),
            s.triggers.iter().filter(|t| !t.rtpcs.is_empty()).count(),
            s.avionic_events.len(),
            s.animation_events.len(),
            s.ground_events.len(),
            s.misc_events.len(),
        );
        assert!(s.triggers.iter().any(|t| t.event == "CRC_380" && t.variable == "A32NX_FWC_CRC"));
        assert!(s.avionic_events.iter().any(|e| e == "new_retard"));
        // <AnimationSounds> is 143 entries in the package as of this
        // writing; a real regression (section renamed/moved) would drop
        // this to 0, which is exactly the bug this fix addresses.
        assert!(!s.animation_events.is_empty(), "no <AnimationSounds> events parsed");
        assert!(s.animation_events.iter().any(|e| e == "battery_switch_on"));
        assert!(s.ground_events.iter().any(|e| e == "FUSELAGE_SCRAPE"));
        assert!(s.misc_events.iter().any(|e| e == "AP_PREFLIGHT_CHECK_OVER"));
        assert!(s.triggers.iter().any(|t| t.event == "V1" && !t.outside));
        assert!(s.triggers.iter().any(|t| t.event == "apuwhineext" && t.outside && t.exterior()));
        assert!(s.triggers.iter().any(|t| t.event == "Engine_1_Front" && t.rtpcs.len() >= 5));
    }

    #[test]
    fn a_sound_is_outside_only_when_its_viewpoint_says_so() {
        let s = parse(
            r#"<SoundInfo><SimVarSounds>
                <Sound WwiseEvent="apuwhineext" ViewPoint="Outside" LocalVar="A32NX_APU_N_RAW"/>
                <Sound WwiseEvent="yoke" ViewPoint="Inside" LocalVar="A"/>
                <Sound WwiseEvent="V1" LocalVar="A32NX_AUDIO_V1_CALLOUT" Continuous="false"/>
            </SimVarSounds></SoundInfo>"#,
        )
        .unwrap();
        let outside: Vec<(&str, bool)> = s.triggers.iter().map(|t| (t.event.as_str(), t.outside)).collect();
        assert_eq!(outside, [("apuwhineext", true), ("yoke", false), ("V1", false)]);
    }
}
