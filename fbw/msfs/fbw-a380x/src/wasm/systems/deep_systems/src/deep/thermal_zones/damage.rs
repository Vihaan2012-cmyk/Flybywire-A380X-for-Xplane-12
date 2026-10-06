use super::network::{ThermalNetwork, ZoneId};

pub const MONTSINGER_DOUBLING_INTERVAL_C: f64 = 10.0;

pub type ComponentId = usize;

pub struct ThermalComponent {
    pub name: &'static str,
    pub zone: ZoneId,
    pub temp_limit_c: f64,
    pub rate_per_s_at_first_doubling: f64,
    pub damage_fraction: f64,
}

pub fn montsinger_rate_per_s(zone_temp_c: f64, temp_limit_c: f64, rate_per_s_at_first_doubling: f64) -> f64 {
    let over_c = zone_temp_c - temp_limit_c;
    if over_c <= 0.0 || !over_c.is_finite() {
        return 0.0;
    }
    let doublings = over_c / MONTSINGER_DOUBLING_INTERVAL_C;
    rate_per_s_at_first_doubling.max(0.0) * 2f64.powf(doublings - 1.0)
}

#[derive(Default)]
pub struct ThermalDamageRegistry {
    pub components: Vec<ThermalComponent>,
}

impl ThermalDamageRegistry {
    pub fn new() -> Self {
        Self { components: Vec::new() }
    }

    pub fn register(&mut self, name: &'static str, zone: ZoneId, temp_limit_c: f64, rate_per_s_at_first_doubling: f64) -> ComponentId {
        self.components.push(ThermalComponent { name, zone, temp_limit_c, rate_per_s_at_first_doubling, damage_fraction: 0.0 });
        self.components.len() - 1
    }

    pub fn update(&mut self, network: &ThermalNetwork, dt_s: f64) {
        if dt_s <= 0.0 {
            return;
        }
        for c in self.components.iter_mut() {
            let t = network.air_temp_c(c.zone);
            if !t.is_finite() {
                continue;
            }
            let rate = montsinger_rate_per_s(t, c.temp_limit_c, c.rate_per_s_at_first_doubling);
            c.damage_fraction = (c.damage_fraction + rate * dt_s).clamp(0.0, 1.0);
        }
    }

    pub fn damage_fraction(&self, id: ComponentId) -> f64 {
        self.components.get(id).map(|c| c.damage_fraction).unwrap_or(0.0)
    }

    pub fn failed(&self, id: ComponentId) -> bool {
        self.damage_fraction(id) >= 1.0
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::deep::thermal_zones::network::{OutsideAir, Zone};

    #[test]
    fn no_damage_at_or_below_the_temperature_limit() {
        assert_eq!(montsinger_rate_per_s(70.0, 70.0, 1.0), 0.0);
        assert_eq!(montsinger_rate_per_s(50.0, 70.0, 1.0), 0.0);
    }

    #[test]
    fn damage_rate_doubles_every_ten_degrees_over_the_limit() {
        let base = montsinger_rate_per_s(80.0, 70.0, 2.0e-4);
        assert!((base - 2.0e-4).abs() < 1e-12, "at exactly the first doubling point, rate should equal the input rate");
        let plus20 = montsinger_rate_per_s(90.0, 70.0, 2.0e-4);
        let plus30 = montsinger_rate_per_s(100.0, 70.0, 2.0e-4);
        assert!((plus20 / base - 2.0).abs() < 1e-9, "ratio {}", plus20 / base);
        assert!((plus30 / base - 4.0).abs() < 1e-9, "ratio {}", plus30 / base);
    }

    #[test]
    fn a_component_fails_at_the_hand_computed_time() {
        let mut reg = ThermalDamageRegistry::new();
        let rate_at_first_doubling = 1.0 / 3600.0;
        let id = reg.register("TestWire", 0, 70.0, rate_at_first_doubling);

        let mut net = crate::deep::thermal_zones::network::ThermalNetwork::new();
        net.add_zone(Zone::new("Zone", 1.0, 1.0e6, 1.0e6, 0.0, 0.0, 0.0, 100.0));
        let outside = OutsideAir { static_temp_c: 100.0, mach: 0.0, true_airspeed_m_s: 0.0 };

        let rate = montsinger_rate_per_s(100.0, 70.0, rate_at_first_doubling);
        let predicted_fail_s = 1.0 / rate;

        let dt = 1.0;
        let mut t = 0.0;
        while t < predicted_fail_s * 1.5 {
            net.step(dt, &outside, 0.0);
            reg.update(&net, dt);
            t += dt;
            if reg.failed(id) {
                break;
            }
        }
        assert!(reg.failed(id), "component should have failed by {predicted_fail_s}s, damage={}", reg.damage_fraction(id));
        assert!((t - predicted_fail_s).abs() < predicted_fail_s * 0.05 + 5.0, "failed at {t}s vs predicted {predicted_fail_s}s");
    }

    #[test]
    fn damage_fraction_clamps_at_one_and_never_decreases_while_hot() {
        let mut reg = ThermalDamageRegistry::new();
        let id = reg.register("TestComponent", 0, 50.0, 1.0);
        let mut net = crate::deep::thermal_zones::network::ThermalNetwork::new();
        net.add_zone(Zone::new("Zone", 1.0, 1.0e6, 1.0e6, 0.0, 0.0, 0.0, 200.0));
        let outside = OutsideAir { static_temp_c: 200.0, mach: 0.0, true_airspeed_m_s: 0.0 };
        for _ in 0..1000 {
            net.step(1.0, &outside, 0.0);
            reg.update(&net, 1.0);
        }
        assert_eq!(reg.damage_fraction(id), 1.0);
    }

    #[test]
    fn no_time_passes_no_damage_and_a_cold_component_never_accrues_any() {
        let mut reg = ThermalDamageRegistry::new();
        let id = reg.register("ColdComponent", 0, 100.0, 1.0);
        let mut net = crate::deep::thermal_zones::network::ThermalNetwork::new();
        net.add_zone(Zone::new("Zone", 1.0, 1.0e6, 1.0e6, 0.0, 0.0, 0.0, 20.0));
        reg.update(&net, 0.0);
        assert_eq!(reg.damage_fraction(id), 0.0);
        let outside = OutsideAir { static_temp_c: 20.0, mach: 0.0, true_airspeed_m_s: 0.0 };
        for _ in 0..500 {
            net.step(1.0, &outside, 0.0);
            reg.update(&net, 1.0);
        }
        assert_eq!(reg.damage_fraction(id), 0.0, "a component well below its limit must never accrue damage");
    }
}
