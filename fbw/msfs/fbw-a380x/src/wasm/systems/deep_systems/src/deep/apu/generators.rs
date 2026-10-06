use super::params;

#[derive(Clone, Copy, Debug, Default)]
pub struct GeneratorFaults {
    pub efficiency_loss: f64,
    pub overload_protection_failed: bool,
}

#[derive(Clone, Copy, Debug)]
pub struct Generator {
    rated_real_power_w: f64,
}

impl Generator {
    pub fn new() -> Self {
        Self {
            rated_real_power_w: params::GENERATOR_RATED_APPARENT_VA * params::GENERATOR_RATED_POWER_FACTOR,
        }
    }

    pub fn rated_real_power_w(&self) -> f64 {
        self.rated_real_power_w
    }

    pub fn shaft_power_w(&self, electrical_load_w: f64, faults: &GeneratorFaults) -> (f64, bool) {
        let load = electrical_load_w.max(0.0);
        let overloaded = load > self.rated_real_power_w;
        let effective_load = if overloaded && !faults.overload_protection_failed {
            self.rated_real_power_w
        } else {
            load
        };
        let eta = (params::GENERATOR_EFFICIENCY_DESIGN * (1.0 - faults.efficiency_loss.clamp(0.0, 1.0)))
            .max(0.3 * params::GENERATOR_EFFICIENCY_DESIGN);
        let shaft_power = if eta > 1e-6 { effective_load / eta } else { 0.0 };
        (shaft_power, overloaded)
    }
}

impl Default for Generator {
    fn default() -> Self {
        Self::new()
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub struct Generators {
    pub gen1: Generator,
    pub gen2: Generator,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct GeneratorsOutputs {
    pub total_shaft_power_w: f64,
    pub gen1_overloaded: bool,
    pub gen2_overloaded: bool,
}

impl Generators {
    pub fn new() -> Self {
        Self { gen1: Generator::new(), gen2: Generator::new() }
    }

    pub fn step(
        &self,
        gen1_load_w: f64,
        gen2_load_w: f64,
        gen1_faults: &GeneratorFaults,
        gen2_faults: &GeneratorFaults,
    ) -> GeneratorsOutputs {
        let (p1, o1) = self.gen1.shaft_power_w(gen1_load_w, gen1_faults);
        let (p2, o2) = self.gen2.shaft_power_w(gen2_load_w, gen2_faults);
        GeneratorsOutputs { total_shaft_power_w: p1 + p2, gen1_overloaded: o1, gen2_overloaded: o2 }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_healthy_generator_below_rated_load_needs_load_over_efficiency_shaft_power() {
        let g = Generator::new();
        let load = 0.5 * g.rated_real_power_w();
        let (shaft, overloaded) = g.shaft_power_w(load, &GeneratorFaults::default());
        assert!(!overloaded);
        assert!((shaft - load / params::GENERATOR_EFFICIENCY_DESIGN).abs() < 1.0);
    }

    #[test]
    fn overload_is_clamped_at_rated_power_when_protection_is_healthy() {
        let g = Generator::new();
        let (shaft_at_rated, _) = g.shaft_power_w(g.rated_real_power_w(), &GeneratorFaults::default());
        let (shaft_way_over, overloaded) =
            g.shaft_power_w(3.0 * g.rated_real_power_w(), &GeneratorFaults::default());
        assert!(overloaded);
        assert!((shaft_way_over - shaft_at_rated).abs() < 1.0, "protection should clamp the shaft demand");
    }

    #[test]
    fn a_failed_overload_protection_lets_shaft_demand_keep_rising_past_rated() {
        let g = Generator::new();
        let faults = GeneratorFaults { overload_protection_failed: true, ..Default::default() };
        let (shaft_at_rated, _) = g.shaft_power_w(g.rated_real_power_w(), &GeneratorFaults::default());
        let (shaft_way_over, overloaded) = g.shaft_power_w(3.0 * g.rated_real_power_w(), &faults);
        assert!(overloaded);
        assert!(shaft_way_over > shaft_at_rated * 2.5);
    }

    #[test]
    fn winding_wear_raises_shaft_power_for_the_same_electrical_output() {
        let g = Generator::new();
        let load = 0.4 * g.rated_real_power_w();
        let (healthy_shaft, _) = g.shaft_power_w(load, &GeneratorFaults::default());
        let (worn_shaft, _) =
            g.shaft_power_w(load, &GeneratorFaults { efficiency_loss: 0.3, overload_protection_failed: false });
        assert!(worn_shaft > healthy_shaft);
    }

    #[test]
    fn zero_load_needs_zero_shaft_power_with_no_nan() {
        let g = Generator::new();
        let (shaft, overloaded) = g.shaft_power_w(0.0, &GeneratorFaults::default());
        assert_eq!(shaft, 0.0);
        assert!(!overloaded);
        let (shaft2, _) = g.shaft_power_w(-10.0, &GeneratorFaults::default());
        assert!(shaft2.is_finite());
    }

    #[test]
    fn both_generators_combine_additively() {
        let gens = Generators::new();
        let load = 0.3 * gens.gen1.rated_real_power_w();
        let out = gens.step(load, load, &GeneratorFaults::default(), &GeneratorFaults::default());
        let (single, _) = gens.gen1.shaft_power_w(load, &GeneratorFaults::default());
        assert!((out.total_shaft_power_w - 2.0 * single).abs() < 1.0);
        assert!(!out.gen1_overloaded && !out.gen2_overloaded);
    }
}
