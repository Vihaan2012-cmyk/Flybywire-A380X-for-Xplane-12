#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Phase {
    Preflight,
    Taxi,
    Takeoff,
    Climb,
    Cruise,
    Descent,
    Approach,
    GoAround,
}

impl Phase {
    pub fn from_fmgc(n: f64) -> Option<Phase> {
        match n.round() as i64 {
            0 => Some(Phase::Preflight),
            1 => Some(Phase::Taxi),
            2 => Some(Phase::Takeoff),
            3 => Some(Phase::Climb),
            4 => Some(Phase::Cruise),
            5 => Some(Phase::Descent),
            6 => Some(Phase::Approach),
            7 => Some(Phase::GoAround),
            _ => None,
        }
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub struct Sample {
    pub elapsed_hours: f64,
    pub altitude_ft: f64,
    pub speed_kt: f64,
    pub phase: Option<Phase>,
    pub ias_kt: f64,
    pub radio_height_ft: f64,
    pub airborne_s: f64,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum ArmCondition {
    ElapsedHours(f64),
    AboveAltitudeFt(f64),
    BelowAltitudeFt(f64),
    AboveSpeedKt(f64),
    OnFlightPhase(Phase),
    AboveIasKt(f64),
    AboveRadioHeightFt(f64),
    BelowRadioHeightFt { ft: f64, seen_above: bool },
    SecondsAfterLiftoff(f64),
}

impl ArmCondition {
    fn observe(&mut self, s: &Sample) {
        if let ArmCondition::BelowRadioHeightFt { ft, seen_above } = self {
            if s.airborne_s > 0.0 && s.radio_height_ft > *ft {
                *seen_above = true;
            }
        }
    }

    fn met(self, s: &Sample) -> bool {
        match self {
            ArmCondition::ElapsedHours(h) => s.elapsed_hours >= h,
            ArmCondition::AboveAltitudeFt(ft) => s.altitude_ft >= ft,
            ArmCondition::BelowAltitudeFt(ft) => s.altitude_ft <= ft,
            ArmCondition::AboveSpeedKt(kt) => s.speed_kt >= kt,
            ArmCondition::OnFlightPhase(p) => s.phase == Some(p),
            ArmCondition::AboveIasKt(kt) => s.ias_kt >= kt,
            ArmCondition::AboveRadioHeightFt(ft) => s.airborne_s > 0.0 && s.radio_height_ft >= ft,
            ArmCondition::BelowRadioHeightFt { ft, seen_above } => seen_above && s.airborne_s > 0.0 && s.radio_height_ft <= ft,
            ArmCondition::SecondsAfterLiftoff(t) => s.airborne_s > 0.0 && s.airborne_s >= t,
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct ScriptedTrigger {
    pub id: u64,
    pub condition: ArmCondition,
    pub magnitude: f64,
    pub external: bool,
}

#[derive(Default)]
pub struct Scripted {
    pending: Vec<ScriptedTrigger>,
}

impl Scripted {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn restore(&mut self, pending: Vec<ScriptedTrigger>) {
        self.pending = pending;
    }

    pub fn snapshot(&self) -> Vec<ScriptedTrigger> {
        self.pending.clone()
    }

    pub fn schedule(&mut self, id: u64, condition: ArmCondition) {
        self.schedule_at(id, condition, 1.0);
    }

    pub fn schedule_at(&mut self, id: u64, condition: ArmCondition, magnitude: f64) {
        self.push(ScriptedTrigger { id, condition, magnitude: magnitude.clamp(0.0, 1.0), external: false });
    }

    pub fn schedule_external(&mut self, id: u64, condition: ArmCondition) {
        self.push(ScriptedTrigger { id, condition, magnitude: 1.0, external: true });
    }

    fn push(&mut self, trigger: ScriptedTrigger) {
        self.pending.retain(|t| t.id != trigger.id);
        self.pending.push(trigger);
    }

    pub fn clear(&mut self) {
        self.pending.clear();
    }

    pub fn cancel(&mut self, id: u64) -> bool {
        let before = self.pending.len();
        self.pending.retain(|t| t.id != id);
        self.pending.len() != before
    }

    pub fn list(&self) -> &[ScriptedTrigger] {
        &self.pending
    }

    pub fn update(&mut self, sample: &Sample) -> Vec<ScriptedTrigger> {
        for t in &mut self.pending {
            t.condition.observe(sample);
        }
        let (fired, still_pending): (Vec<_>, Vec<_>) = self.pending.drain(..).partition(|t| t.condition.met(sample));
        self.pending = still_pending;
        fired
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sample(ias_kt: f64, radio_height_ft: f64, airborne_s: f64) -> Sample {
        Sample { ias_kt, radio_height_ft, airborne_s, ..Sample::default() }
    }

    fn fired(s: &mut Scripted, sample: &Sample) -> Vec<(u64, f64)> {
        s.update(sample).into_iter().map(|t| (t.id, t.magnitude)).collect()
    }

    #[test]
    fn an_external_trigger_keeps_its_route_when_it_fires() {
        let mut s = Scripted::new();
        s.schedule_external(26_002, ArmCondition::AboveIasKt(100.0));
        let due = s.update(&sample(120.0, 0.0, 0.0));
        assert_eq!(due.len(), 1);
        assert!(due[0].external && due[0].id == 26_002);
    }

    #[test]
    fn an_ias_trigger_fires_once_at_the_speed_with_its_own_severity() {
        let mut s = Scripted::new();
        s.schedule_at(7, ArmCondition::AboveIasKt(140.0), 0.6);
        assert!(fired(&mut s, &sample(139.0, 0.0, 0.0)).is_empty());
        assert_eq!(fired(&mut s, &sample(141.0, 0.0, 0.0)), vec![(7, 0.6)]);
        assert!(fired(&mut s, &sample(150.0, 0.0, 0.0)).is_empty());
        assert!(s.list().is_empty());
    }

    #[test]
    fn a_radio_height_or_liftoff_trigger_never_fires_on_the_ground() {
        let mut s = Scripted::new();
        s.schedule(1, ArmCondition::AboveRadioHeightFt(0.0));
        s.schedule(2, ArmCondition::SecondsAfterLiftoff(0.0));
        assert!(fired(&mut s, &sample(0.0, 0.0, 0.0)).is_empty());
        let due = fired(&mut s, &sample(160.0, 5.0, 0.1));
        assert_eq!(due.len(), 2);
    }

    #[test]
    fn a_below_radio_height_trigger_waits_until_the_aircraft_has_been_above_it() {
        let mut s = Scripted::new();
        s.schedule(3, ArmCondition::BelowRadioHeightFt { ft: 1000.0, seen_above: false });
        assert!(fired(&mut s, &sample(0.0, 0.0, 0.0)).is_empty(), "on the ground before takeoff");
        assert!(fired(&mut s, &sample(170.0, 300.0, 10.0)).is_empty(), "climbing through it after takeoff");
        assert!(fired(&mut s, &sample(250.0, 2500.0, 120.0)).is_empty());
        assert_eq!(fired(&mut s, &sample(160.0, 990.0, 1800.0)), vec![(3, 1.0)], "descending through it on approach");
    }

    #[test]
    fn rescheduling_a_failure_replaces_its_trigger_and_clear_empties_the_queue() {
        let mut s = Scripted::new();
        s.schedule(5, ArmCondition::AboveIasKt(100.0));
        s.schedule_at(5, ArmCondition::AboveIasKt(200.0), 0.5);
        assert_eq!(s.list().len(), 1);
        assert!(fired(&mut s, &sample(150.0, 0.0, 0.0)).is_empty());
        s.clear();
        assert!(s.list().is_empty());
    }
}
