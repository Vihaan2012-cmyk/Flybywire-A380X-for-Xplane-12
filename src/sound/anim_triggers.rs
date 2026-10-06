//! Cockpit switch/button click sounds: the converter's own
//! `sound_triggers.txt` (written next to `cockpit_variables.txt`), built
//! from the package's `<AnimationSounds>` `WwiseEvent` names and the
//! `<AnimationTriggers>` blocks MSFS's own `ModelBehaviorDefs` compile them
//! from (`ASOBO_GT_AnimTriggers_SoundEvent`/`_SoundEvents_Same`/`_2SoundEvents`
//! — see `D:/A380/msfs2xp-aircraft/src/behaviour/expand.rs`).
//!
//! `sound.xml`'s own `<AnimationSounds>` section lists only the *event
//! names*: which switch plays which one is a property of the aircraft's
//! model behaviour XML, not sound.xml (`triggers` module doc), so this
//! plugin cannot recover it from the MSFS package alone. The converter
//! already walks that XML to know each click's `fbw/cockpit/<anim>` dataref
//! (0..1 click position); `sound_triggers.txt` is where it writes the
//! pairing down for this plugin to read, one line per `<EventTrigger>`:
//! `<dataref>\t<Direction>\t<NormalizedTime>\t<WwiseEvent>\t<Count>`, with
//! `NormalizedTime`/`Count` mutually exclusive per line (exactly one column
//! is ever non-empty).
//!
//! `ASOBO_GT_AnimTriggers_SoundEvents_Same` (FlyByWire uses it for the
//! flap-lever cover, `Count=1`, and the speedbrake lever, `Count=3` --
//! `A32NX_Interior_Handling.xml`) has no `NormalizedTime` at all: it fires
//! `Count` times, evenly spaced across the clip, instead of once at a fixed
//! point. MSFS's own SDK docs give the worked example: `Count="2"` "will
//! play each time the animation timeline reaches 25% and 75%"
//! (docs.flightsimulator.com, Content Configuration > Sounds > Sound XML
//! Examples), i.e. threshold `i` (0-based, of `N`) sits at `(i + 0.5) / N`
//! -- the midpoint of the `i`-th of `N` equal bins, never the 0 or 1
//! endpoint. [`parse`] expands a `Count` line into that many concrete
//! [`AnimTrigger`]s right here, so [`AnimTriggerState`] itself only ever
//! has to compare a reading against one fixed `normalized_time`, exactly as
//! before -- no change to the per-tick hot path.
//!
//! At runtime this plugin already has the dataref's current value every
//! tick (the same click-position variable the cockpit's own animation
//! reads); [`AnimTriggerState::step`] is the direction-of-travel state
//! machine that turns a crossing of `NormalizedTime` — moving the recorded
//! way — into a single fire, mirroring what MSFS's own compiled
//! `<EventTrigger>` does when the clip plays past that point. A discrete
//! switch (the overwhelming majority: no interpolated travel, just 0 or 1)
//! crosses any threshold strictly between them in one tick, so this reduces
//! to "which way did it just flip", exactly as intended.

/// `<EventTrigger Direction="...">`: which way the clip must be moving for
/// this event to fire. `Both` is Asobo's own default for the single-event
/// template (`ASOBO_GT_AnimTriggers_SoundEvent`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Direction {
    Forward,
    Backward,
    Both,
}

impl Direction {
    fn parse(s: &str) -> Self {
        if s.eq_ignore_ascii_case("backward") {
            Direction::Backward
        } else if s.eq_ignore_ascii_case("both") {
            Direction::Both
        } else {
            Direction::Forward
        }
    }
}

/// One line of `sound_triggers.txt` (after a `Count` line has been expanded
/// into its `N` concrete points).
#[derive(Clone, Debug, PartialEq)]
pub struct AnimTrigger {
    /// The `fbw/cockpit/<anim>` dataref to watch (already registered by
    /// `Plugin::register_cockpit_variables`, `cockpit_variables.txt`).
    pub dataref: String,
    pub direction: Direction,
    /// Where in the clip's 0..1 travel this fires.
    pub normalized_time: f64,
    /// The `sound.xml` `<AnimationSounds>` `WwiseEvent` name to play, same
    /// as `Sound::fire_event`'s `event_name` argument.
    pub event: String,
}

/// Tab-separated: `<dataref>\t<Direction>\t<NormalizedTime>\t<WwiseEvent>\t<Count>`.
/// Exactly one of `NormalizedTime`/`Count` is a parseable number on a given
/// line, the other left empty -- Asobo's own templates never set both an
/// `EventTrigger`'s `NormalizedTime` and its `Count` (see the module doc).
/// A `Count` line expands into that many evenly spaced [`AnimTrigger`]s,
/// each its own entry -- and so, from [`fresh_states`], its own independent
/// state, the same "two triggers can share a dataref but must not share
/// state" rule [`States`] already documents for a plain two-event switch. A
/// malformed line, a short line, or one with neither field parseable is
/// skipped, not an error: one bad row must not lose every other click
/// sound.
pub fn parse(text: &str) -> Vec<AnimTrigger> {
    text.lines()
        .flat_map(|line| {
            let mut f = line.splitn(5, '\t');
            let (Some(dataref), Some(direction), Some(normalized_time), Some(event), Some(count)) = (f.next(), f.next(), f.next(), f.next(), f.next()) else {
                return Vec::new();
            };
            let (dataref, event) = (dataref.trim(), event.trim());
            if dataref.is_empty() || event.is_empty() {
                return Vec::new();
            }
            let direction = Direction::parse(direction.trim());
            // The common case: one fixed point
            // (`ASOBO_GT_AnimTriggers_SoundEvent`/`_2SoundEvents`).
            if let Ok(nt) = normalized_time.trim().parse::<f64>() {
                return vec![AnimTrigger { dataref: dataref.to_string(), direction, normalized_time: nt, event: event.to_string() }];
            }
            // `ASOBO_GT_AnimTriggers_SoundEvents_Same`: fire `n` times,
            // evenly spaced, never at the 0/1 endpoint -- confirmed against
            // MSFS's own SDK docs: `Count="2"` fires at 25% and 75%, i.e.
            // `(i + 0.5) / n` (see the module doc).
            if let Ok(n) = count.trim().parse::<u32>() {
                if n == 0 {
                    return Vec::new();
                }
                return (0..n)
                    .map(|i| AnimTrigger {
                        dataref: dataref.to_string(),
                        direction,
                        normalized_time: (f64::from(i) + 0.5) / f64::from(n),
                        event: event.to_string(),
                    })
                    .collect();
            }
            Vec::new()
        })
        .collect()
}

/// One [`AnimTrigger`]'s state between ticks: only the dataref's own last
/// reading, so the very first tick can never fire (spawn state, whatever it
/// is, is not a click).
#[derive(Clone, Copy, Debug, Default)]
pub struct AnimTriggerState {
    last: Option<f64>,
}

impl AnimTriggerState {
    /// Whether this tick's move across `trigger.normalized_time` should fire
    /// its event.
    pub fn step(&mut self, trigger: &AnimTrigger, value: f64) -> bool {
        let prev = self.last.replace(value);
        let Some(prev) = prev else { return false };
        let nt = trigger.normalized_time;
        let crossed_up = prev < nt && value >= nt;
        let crossed_down = prev > nt && value <= nt;
        match trigger.direction {
            Direction::Forward => crossed_up,
            Direction::Backward => crossed_down,
            Direction::Both => crossed_up || crossed_down,
        }
    }
}

/// One [`AnimTriggerState`] per line of `sound_triggers.txt`, keyed by index
/// (parallel to `triggers::TriggerState` in `Sound::State`), not by
/// dataref: two triggers legitimately share a dataref (the Forward and
/// Backward halves of a 2-event switch) and must not share state, or the
/// second one's "first reading" guard would never arm.
pub type States = Vec<AnimTriggerState>;

pub fn fresh_states(triggers: &[AnimTrigger]) -> States {
    vec![AnimTriggerState::default(); triggers.len()]
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parse_reads_tab_separated_lines_and_skips_malformed_ones() {
        let text = "fbw/cockpit/BATTERY_SWITCH\tForward\t0.1\tbattery_switch_on\t\nfbw/cockpit/BATTERY_SWITCH\tBackward\t0.9\tbattery_switch_off\t\nnot enough columns\n\nfbw/cockpit/X\tBoth\t\tevt\t\n";
        let t = parse(text);
        assert_eq!(t.len(), 2, "the two malformed lines must be skipped, not panic or poison the rest: 'not enough columns' has no tabs at all, and the last line has neither NormalizedTime nor Count filled in");
        assert_eq!(t[0], AnimTrigger { dataref: "fbw/cockpit/BATTERY_SWITCH".into(), direction: Direction::Forward, normalized_time: 0.1, event: "battery_switch_on".into() });
        assert_eq!(t[1].direction, Direction::Backward);
        assert_eq!(t[1].event, "battery_switch_off");
    }

    /// `ASOBO_GT_AnimTriggers_SoundEvents_Same`: a `Count` line with no
    /// `NormalizedTime` expands into that many triggers at `(i + 0.5) / n`,
    /// matching FlyByWire's real `Count=1` (flap-lever cover) and `Count=3`
    /// (speedbrake lever) usages (`A32NX_Interior_Handling.xml`).
    #[test]
    fn parse_expands_a_count_line_into_evenly_spaced_triggers() {
        let text = "fbw/cockpit/HANDLING_Lever_Flaps\tBoth\t\tflapcover\t1\nfbw/cockpit/HANDLING_Lever_Spoilers\tBoth\t\tlever_speedbrakes\t3\n";
        let t = parse(text);
        assert_eq!(t.len(), 4, "1 trigger for Count=1, 3 for Count=3");
        assert_eq!(t[0].normalized_time, 0.5, "Count=1's one trigger sits at the midpoint -- coincidentally the same number the old unwrap_or(0.5) default used, but for a real reason now (N=1: (0+0.5)/1)");
        assert_eq!(t[0].dataref, "fbw/cockpit/HANDLING_Lever_Flaps");

        let spoiler: Vec<f64> = t[1..4].iter().map(|a| a.normalized_time).collect();
        for (got, want) in spoiler.iter().zip([1.0 / 6.0, 0.5, 5.0 / 6.0]) {
            assert!((got - want).abs() < 1e-9, "Count=3 must fire at (i+0.5)/3: got {spoiler:?}");
        }
        assert!(t[1..4].iter().all(|a| a.dataref == "fbw/cockpit/HANDLING_Lever_Spoilers" && a.event == "lever_speedbrakes"));
    }

    /// MSFS's own SDK docs' worked example, verbatim (Content Configuration
    /// > Sounds > Sound XML Examples): `Count="2"` fires at 25% and 75%,
    /// never at the 0/1 endpoint.
    #[test]
    fn parse_count_two_matches_the_sdk_docs_worked_example() {
        let t = parse("fbw/cockpit/X\tBoth\t\tclick\t2\n");
        assert_eq!(t.len(), 2);
        assert!((t[0].normalized_time - 0.25).abs() < 1e-9);
        assert!((t[1].normalized_time - 0.75).abs() < 1e-9);
    }

    /// A malformed `Count` (non-numeric, or `0`) must drop the line, not
    /// panic or divide by zero.
    #[test]
    fn parse_skips_a_zero_or_unparseable_count() {
        assert!(parse("fbw/cockpit/X\tBoth\t\tclick\t0\n").is_empty());
        assert!(parse("fbw/cockpit/X\tBoth\t\tclick\tnotanumber\n").is_empty());
    }

    #[test]
    fn a_discrete_switch_fires_once_per_flip_never_on_the_first_reading() {
        let fwd = AnimTrigger { dataref: "d".into(), direction: Direction::Forward, normalized_time: 0.1, event: "on".into() };
        let mut s = AnimTriggerState::default();
        // Spawn state, whatever it reads first: must not fire.
        assert!(!s.step(&fwd, 0.0));
        // A discrete switch's click jumps 0 -> 1 in one tick: crosses 0.1
        // moving up, fires once.
        assert!(s.step(&fwd, 1.0));
        // Held at 1: no repeat.
        assert!(!s.step(&fwd, 1.0));

        let bwd = AnimTrigger { dataref: "d".into(), direction: Direction::Backward, normalized_time: 0.9, event: "off".into() };
        let mut s2 = AnimTriggerState::default();
        assert!(!s2.step(&bwd, 1.0));
        assert!(s2.step(&bwd, 0.0), "1 -> 0 must cross 0.9 moving down and fire the Backward event");
        assert!(!s2.step(&bwd, 0.0));

        let mut s3 = AnimTriggerState::default();
        assert!(!s3.step(&fwd, 1.0)); // first reading, already "on"
        assert!(!s3.step(&fwd, 0.0), "moving down must not fire a Forward-only trigger");
    }

    #[test]
    fn direction_both_fires_either_way() {
        let both = AnimTrigger { dataref: "d".into(), direction: Direction::Both, normalized_time: 0.5, event: "click".into() };
        let mut s = AnimTriggerState::default();
        assert!(!s.step(&both, 0.0));
        assert!(s.step(&both, 1.0), "up through 0.5 must fire a Both trigger");
        assert!(s.step(&both, 0.0), "back down through 0.5 must also fire it");
    }

    #[test]
    fn fresh_states_gives_one_independent_state_per_trigger_even_when_they_share_a_dataref() {
        let triggers = parse("fbw/cockpit/X\tForward\t0.1\ton\t\nfbw/cockpit/X\tBackward\t0.9\toff\t\n");
        let mut states = fresh_states(&triggers);
        assert_eq!(states.len(), 2);
        assert!(!states[0].step(&triggers[0], 0.0) && !states[1].step(&triggers[1], 0.0));
        assert!(states[0].step(&triggers[0], 1.0));
        assert!(!states[1].step(&triggers[1], 1.0));
    }

    /// `fresh_states` must give a `Count` expansion's triggers independent
    /// state too, the same guarantee the two-event case above already has
    /// -- otherwise the speedbrake lever's 3 thresholds (or the flap
    /// cover's 1) sharing a dataref would corrupt each other's "first
    /// reading" guard.
    #[test]
    fn fresh_states_gives_independent_state_to_each_expanded_count_trigger() {
        let triggers = parse("fbw/cockpit/HANDLING_Lever_Spoilers\tBoth\t\tlever_speedbrakes\t3\n");
        assert_eq!(triggers.len(), 3);
        let mut states = fresh_states(&triggers);
        assert_eq!(states.len(), 3);
        // Baseline reads for all three, at a value below every threshold.
        assert!(!states.iter_mut().zip(&triggers).any(|(s, t)| s.step(t, 0.0)));
        // Move straight to 1.0: crosses all three (1/6, 1/2, 5/6) at once,
        // one fire per threshold, independently.
        assert!(states.iter_mut().zip(&triggers).all(|(s, t)| s.step(t, 1.0)));
    }
}
