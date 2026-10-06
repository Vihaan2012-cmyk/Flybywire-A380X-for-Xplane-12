const VLE_VLO_KT: f64 = 250.0;
const ULTIMATE_FACTOR_OF_SAFETY: f64 = 1.5;
const OVERSPEED_HOLD_S: f64 = 3.0;

pub(super) const GEAR_DOOR_JAM_IDS: [u64; 3] = [32_023, 32_024, 32_025];

#[derive(Default)]
pub(super) struct SimEffects {
    gear_ultimate_seconds: f64,
    gear_doors_jammed: bool,
}

impl SimEffects {
    pub(super) fn new() -> Self {
        Self::default()
    }

    pub(super) fn update(&mut self, ias_kt: f64, gear_lever_down: bool, delta_s: f64) {
        if delta_s <= 0.0 {
            return;
        }
        let ultimate_kt = VLE_VLO_KT * ULTIMATE_FACTOR_OF_SAFETY.sqrt();
        let exceeded = gear_lever_down && ias_kt > ultimate_kt;
        if exceeded {
            self.gear_ultimate_seconds += delta_s;
        } else {
            self.gear_ultimate_seconds = 0.0;
        }
        if self.gear_ultimate_seconds >= OVERSPEED_HOLD_S {
            self.gear_doors_jammed = true;
        }
    }

    pub(super) fn derived_failure_ids(&self) -> Vec<u64> {
        if self.gear_doors_jammed {
            GEAR_DOOR_JAM_IDS.to_vec()
        } else {
            Vec::new()
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_healthy_profile_arms_nothing() {
        let mut fx = SimEffects::new();
        for _ in 0..1000 {
            fx.update(240.0, true, 1.0);
        }
        assert!(fx.derived_failure_ids().is_empty());
    }

    #[test]
    fn gear_up_at_any_speed_arms_nothing() {
        let mut fx = SimEffects::new();
        for _ in 0..10 {
            fx.update(400.0, false, 1.0);
        }
        assert!(fx.derived_failure_ids().is_empty());
    }

    #[test]
    fn a_brief_excursion_past_ultimate_is_not_damage() {
        let mut fx = SimEffects::new();
        fx.update(320.0, true, 1.0);
        fx.update(320.0, true, 1.0);
        fx.update(200.0, true, 1.0);
        assert!(fx.derived_failure_ids().is_empty(), "a 2 s excursion is inside the hold");
    }

    #[test]
    fn held_past_ultimate_for_the_hold_time_jams_the_gear_doors() {
        let mut fx = SimEffects::new();
        for _ in 0..2 {
            fx.update(320.0, true, 1.0);
        }
        assert!(fx.derived_failure_ids().is_empty(), "still inside the 3 s hold");
        fx.update(320.0, true, 1.0);
        assert_eq!(fx.derived_failure_ids(), GEAR_DOOR_JAM_IDS.to_vec());
    }

    #[test]
    fn a_paused_frame_does_nothing() {
        let mut fx = SimEffects::new();
        fx.update(320.0, true, 0.0);
        assert!(fx.derived_failure_ids().is_empty());
    }
}
