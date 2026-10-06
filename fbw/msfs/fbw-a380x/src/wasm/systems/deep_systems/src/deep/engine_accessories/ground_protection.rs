pub const KEEP_OUT_ZONE_N1_PCT: (f64, f64) = (64.0, 72.0);
pub const KEEP_OUT_ZONE_BELOW_KT: f64 = 60.0;
pub const MODIFIED_TAKEOFF_N1_CAP_PCT: f64 = 78.0;
pub const MODIFIED_TAKEOFF_BELOW_KT: f64 = 32.5;
pub const TLA_CLIMB_DEG: f64 = 25.0;
pub const TLA_FORWARD_IDLE_DEG: f64 = 0.0;
const KT_PER_M_S: f64 = 1.943_844;

#[derive(Clone, Copy, Debug)]
pub struct GroundProtection {
    armed: bool,
}

impl Default for GroundProtection {
    fn default() -> Self {
        Self { armed: true }
    }
}

impl GroundProtection {
    pub fn n1_target(&mut self, commanded_pct: f64, on_ground: bool, groundspeed_m_s: f64, tla_deg: f64) -> f64 {
        let groundspeed_kt = groundspeed_m_s * KT_PER_M_S;
        if !on_ground || groundspeed_kt >= KEEP_OUT_ZONE_BELOW_KT {
            self.armed = false;
        } else if tla_deg < TLA_CLIMB_DEG {
            self.armed = true;
        }
        if !self.armed || !on_ground || tla_deg < TLA_FORWARD_IDLE_DEG {
            return commanded_pct;
        }

        let mut target = commanded_pct;
        if groundspeed_kt < MODIFIED_TAKEOFF_BELOW_KT {
            target = target.min(MODIFIED_TAKEOFF_N1_CAP_PCT);
        }
        let (low, high) = KEEP_OUT_ZONE_N1_PCT;
        if target > low && target < high {
            target = if target < (low + high) / 2.0 { low } else { high };
        }
        target
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const KT: f64 = 1.0 / KT_PER_M_S;
    const TOGA_TLA: f64 = 45.0;

    #[test]
    fn standing_still_at_toga_is_held_at_seventy_eight_percent() {
        let mut p = GroundProtection::default();
        assert_eq!(p.n1_target(88.3, true, 0.0, TOGA_TLA), MODIFIED_TAKEOFF_N1_CAP_PCT);
        assert_eq!(p.n1_target(88.3, true, 30.0 * KT, TOGA_TLA), MODIFIED_TAKEOFF_N1_CAP_PCT);
    }

    #[test]
    fn rolling_past_thirty_two_and_a_half_knots_releases_full_takeoff_thrust() {
        let mut p = GroundProtection::default();
        p.n1_target(88.3, true, 10.0 * KT, TOGA_TLA);
        assert_eq!(p.n1_target(88.3, true, 33.0 * KT, TOGA_TLA), 88.3);
    }

    #[test]
    fn the_keep_out_band_cannot_be_held_below_sixty_knots_but_can_be_passed_through() {
        let mut p = GroundProtection::default();
        assert_eq!(p.n1_target(66.0, true, 5.0 * KT, 20.0), 64.0);
        assert_eq!(p.n1_target(70.0, true, 5.0 * KT, 20.0), 72.0);
        assert_eq!(p.n1_target(63.0, true, 5.0 * KT, 20.0), 63.0);
        assert_eq!(p.n1_target(75.0, true, 5.0 * KT, 20.0), 75.0);
        assert_eq!(p.n1_target(66.0, true, 45.0 * KT, 20.0), 64.0, "the band still applies between 32.5 and 60 kt");
    }

    #[test]
    fn idle_reverse_and_flight_are_untouched() {
        let mut p = GroundProtection::default();
        assert_eq!(p.n1_target(18.5, true, 0.0, 0.0), 18.5);
        assert_eq!(p.n1_target(66.0, true, 40.0 * KT, -20.0), 66.0, "reverse thrust is not protected");
        assert_eq!(p.n1_target(88.3, false, 150.0, TOGA_TLA), 88.3);
    }

    #[test]
    fn once_takeoff_power_is_set_it_stays_off_until_the_levers_come_back_below_climb() {
        let mut p = GroundProtection::default();
        p.n1_target(88.3, true, 70.0 * KT, TOGA_TLA);
        assert_eq!(p.n1_target(88.3, true, 20.0 * KT, TOGA_TLA), 88.3, "rejected or slowed take-off with the levers still forward");
        p.n1_target(18.5, true, 20.0 * KT, 0.0);
        assert_eq!(p.n1_target(88.3, true, 20.0 * KT, TOGA_TLA), MODIFIED_TAKEOFF_N1_CAP_PCT, "re-armed by bringing the levers back");
    }
}
