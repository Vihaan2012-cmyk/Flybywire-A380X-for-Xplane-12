pub fn arc_current_a(source_v: f64, r_wire_ohm: f64, arc_voltage_v: f64) -> f64 {
    if r_wire_ohm <= 0.0 {
        return 0.0;
    }
    ((source_v - arc_voltage_v) / r_wire_ohm).max(0.0)
}

pub fn arc_heat_w(arc_current_a: f64, arc_voltage_v: f64) -> f64 {
    (arc_current_a * arc_voltage_v).max(0.0)
}

pub fn thermal_equivalent_current_a(arc_current_a: f64, duty: f64) -> f64 {
    arc_current_a * duty.clamp(0.0, 1.0).sqrt()
}

pub fn thermal_breaker_sees_ratio(baseline_load_a: f64, arc_current_a: f64, duty: f64, rated_a: f64) -> f64 {
    if rated_a <= 0.0 {
        return 0.0;
    }
    (baseline_load_a + thermal_equivalent_current_a(arc_current_a, duty)) / rated_a
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn arc_current_is_lower_than_a_bolted_short_by_the_arcs_own_voltage_drop() {
        let bolted = 115.0 / 0.5;
        let arcing = arc_current_a(115.0, 0.5, 30.0);
        assert!(arcing < bolted);
        assert!((arcing - (115.0 - 30.0) / 0.5).abs() < 1e-9);
    }

    #[test]
    fn an_arc_voltage_at_or_above_the_source_cannot_sustain() {
        assert_eq!(arc_current_a(28.0, 0.1, 30.0), 0.0, "28 V source cannot sustain a 30 V arc");
    }

    #[test]
    fn arc_heat_is_zero_at_zero_current_and_positive_otherwise() {
        assert_eq!(arc_heat_w(0.0, 30.0), 0.0);
        assert!(arc_heat_w(10.0, 30.0) > 0.0);
    }

    #[test]
    fn low_duty_cycle_can_keep_an_intermittent_arc_under_the_breakers_trip_threshold() {
        let i_arc = arc_current_a(115.0, 1.7, 30.0);
        assert!((i_arc - 50.0).abs() < 1e-9);
        let ratio = thermal_breaker_sees_ratio(20.0, i_arc, 0.04, 35.0);
        assert!((ratio - 30.0 / 35.0).abs() < 1e-9);
        assert!(ratio < 1.0, "ratio {ratio} should stay under 1.0 -- the documented thermal-breaker blind spot");
        let continuous_ratio = thermal_breaker_sees_ratio(20.0, i_arc, 1.0, 35.0);
        assert!((continuous_ratio - 2.0).abs() < 1e-9);
        assert!(continuous_ratio > 1.0);
    }

    #[test]
    fn duty_above_one_is_clamped_not_amplified() {
        let normal = thermal_equivalent_current_a(10.0, 1.0);
        let clamped = thermal_equivalent_current_a(10.0, 5.0);
        assert!((normal - clamped).abs() < 1e-9);
    }

    #[test]
    fn no_nan_at_zero_resistance_or_zero_rating() {
        assert_eq!(arc_current_a(115.0, 0.0, 30.0), 0.0);
        assert_eq!(thermal_breaker_sees_ratio(1.0, 1.0, 1.0, 0.0), 0.0);
    }
}
