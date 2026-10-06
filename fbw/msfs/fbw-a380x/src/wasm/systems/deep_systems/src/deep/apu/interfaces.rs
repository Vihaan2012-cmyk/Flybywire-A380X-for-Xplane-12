#[derive(Clone, Copy, Debug, Default)]
pub struct BleedOutput {
    pub mass_flow_kg_s: f64,
    pub pressure_pa: f64,
    pub temperature_k: f64,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct GeneratorOutput {
    pub real_power_w: f64,
    pub shaft_power_w: f64,
    pub overloaded: bool,
}

#[derive(Clone, Copy, Debug)]
pub struct BatteryInput {
    pub open_circuit_v: f64,
    pub internal_resistance_ohm: f64,
    pub available: bool,
}

impl BatteryInput {
    pub fn healthy() -> Self {
        Self {
            open_circuit_v: super::params::BATTERY_NOMINAL_OPEN_CIRCUIT_V,
            internal_resistance_ohm: super::params::BATTERY_INTERNAL_RESISTANCE_OHM,
            available: true,
        }
    }
}

impl Default for BatteryInput {
    fn default() -> Self {
        Self::healthy()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use super::super::params;

    #[test]
    fn the_healthy_default_matches_the_nominal_battery_constants() {
        let b = BatteryInput::healthy();
        assert_eq!(b.open_circuit_v, params::BATTERY_NOMINAL_OPEN_CIRCUIT_V);
        assert!(b.available);
    }

    #[test]
    fn plain_structs_carry_exactly_what_they_are_given() {
        let bleed = BleedOutput { mass_flow_kg_s: 1.1, pressure_pa: 300_000.0, temperature_k: 460.0 };
        assert_eq!(bleed.mass_flow_kg_s, 1.1);
        let gen = GeneratorOutput { real_power_w: 50_000.0, shaft_power_w: 58_000.0, overloaded: false };
        assert!(!gen.overloaded);
    }
}
