use super::gas;

const HF_NACLO3_J_PER_MOL: f64 = -365_400.0;
const HF_NACL_J_PER_MOL: f64 = -411_150.0;
const HF_FE2O3_J_PER_MOL: f64 = -824_200.0;
const M_NACLO3_KG_PER_MOL: f64 = 0.106_44;
const M_FE_KG_PER_MOL: f64 = 0.055_845;
const CANDLE_SPECIFIC_HEAT_J_PER_KG_K: f64 = 100.1 / M_NACLO3_KG_PER_MOL;

const CANDLE_NACLO3_MASS_FRACTION: f64 = 0.90;
const CANDLE_IRON_MASS_FRACTION: f64 = 0.05;

const CANDLE_DENSITY_KG_M3: f64 = 2000.0;

const CANDLE_LENGTH_OVER_DIAMETER: f64 = 4.0;

const CASE_WALL_THICKNESS_M: f64 = 0.005;

const CASE_STEEL_MASS_KG: f64 = 0.1;
const STEEL_SPECIFIC_HEAT_J_PER_KG_K: f64 = 490.0;

const CASE_CONVECTION_W_PER_M2_K: f64 = 10.0;
const CASE_EMISSIVITY: f64 = 0.9;
const STEFAN_BOLTZMANN: f64 = 5.670_374_419e-8;

const OUTPUT_SHAPE: f64 = 0.5;

const BURN_COMPLETE_EPS: f64 = 1e-9;

const REFERENCE_TEMP_K: f64 = 294.15;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CandleYield {
    pub o2_mol_per_kg: f64,
    pub heat_j_per_kg: f64,
}

pub fn candle_yield() -> CandleYield {
    let chlorate_mol = CANDLE_NACLO3_MASS_FRACTION / M_NACLO3_KG_PER_MOL;
    let chlorate_heat = chlorate_mol * (HF_NACLO3_J_PER_MOL - HF_NACL_J_PER_MOL).abs();
    let chlorate_o2 = chlorate_mol * 1.5;

    let iron_mol = CANDLE_IRON_MASS_FRACTION / M_FE_KG_PER_MOL;
    let iron_heat = iron_mol * (2.0 * HF_FE2O3_J_PER_MOL / 4.0).abs();
    let iron_o2 = iron_mol * 0.75;

    CandleYield { o2_mol_per_kg: (chlorate_o2 - iron_o2).max(0.0), heat_j_per_kg: chlorate_heat + iron_heat }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GeneratorSpec {
    pub persons: f64,
    pub rated_output_liters: f64,
    pub rated_duration_s: f64,
}

impl GeneratorSpec {
    pub fn two_person() -> Self {
        Self { persons: 2.0, rated_output_liters: 42.0, rated_duration_s: 900.0 }
    }
    pub fn three_person() -> Self {
        Self { persons: 3.0, rated_output_liters: 62.0, rated_duration_s: 900.0 }
    }
    pub fn four_person() -> Self {
        Self { persons: 4.0, rated_output_liters: 84.0, rated_duration_s: 900.0 }
    }

    pub fn rated_output_kg(&self) -> f64 {
        gas::mass_from_free_air_kg(self.rated_output_liters, REFERENCE_TEMP_K)
    }

    pub fn candle_mass_kg(&self) -> f64 {
        let moles = self.rated_output_kg() / gas::M_O2_KG_PER_MOL;
        let yield_ = candle_yield();
        if yield_.o2_mol_per_kg > 0.0 {
            moles / yield_.o2_mol_per_kg
        } else {
            0.0
        }
    }

    pub fn rated_heat_j(&self) -> f64 {
        self.candle_mass_kg() * candle_yield().heat_j_per_kg
    }

    pub fn case_area_m2(&self) -> f64 {
        let v = self.candle_mass_kg() / CANDLE_DENSITY_KG_M3;
        if !(v > 0.0) {
            return 0.0;
        }
        let l = CANDLE_LENGTH_OVER_DIAMETER;
        let d = (4.0 * v / (std::f64::consts::PI * l)).cbrt() + 2.0 * CASE_WALL_THICKNESS_M;
        let length = l * (4.0 * v / (std::f64::consts::PI * l)).cbrt() + 2.0 * CASE_WALL_THICKNESS_M;
        std::f64::consts::PI * d * length + 2.0 * std::f64::consts::PI * d * d / 4.0
    }

    pub fn heat_capacity_j_per_k(&self) -> f64 {
        (self.candle_mass_kg() * CANDLE_SPECIFIC_HEAT_J_PER_KG_K + CASE_STEEL_MASS_KG * STEEL_SPECIFIC_HEAT_J_PER_KG_K).max(1.0)
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct GeneratorFaults {
    pub dud_initiator: f64,
    pub quench_fraction: f64,
    pub inadvertent_ignition: f64,
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct GeneratorOutputs {
    pub firing: bool,
    pub spent: bool,
    pub burned_fraction: f64,
    pub o2_kg_s: f64,
    pub o2_l_per_min: f64,
    pub heat_w: f64,
    pub case_temp_k: f64,
}

#[derive(Clone, Debug)]
pub struct ChemicalOxygenGenerator {
    spec: GeneratorSpec,
    candle_mass_kg: f64,
    heat_capacity_j_per_k: f64,
    case_area_m2: f64,
    mean_o2_kg_s: f64,
    mean_heat_w: f64,
    liters_per_kg: f64,
    burned_fraction: f64,
    lit: bool,
    case_temp_k: f64,
}

impl ChemicalOxygenGenerator {
    pub fn new(spec: GeneratorSpec) -> Self {
        let candle_mass_kg = spec.candle_mass_kg();
        let duration = spec.rated_duration_s.max(1.0);
        Self {
            spec,
            candle_mass_kg,
            heat_capacity_j_per_k: spec.heat_capacity_j_per_k(),
            case_area_m2: spec.case_area_m2(),
            mean_o2_kg_s: spec.rated_output_kg() / duration,
            mean_heat_w: spec.rated_heat_j() / duration,
            liters_per_kg: 1.0 / gas::mass_from_free_air_kg(1.0, REFERENCE_TEMP_K).max(1e-12),
            burned_fraction: 0.0,
            lit: false,
            case_temp_k: REFERENCE_TEMP_K,
        }
    }

    pub fn spec(&self) -> GeneratorSpec {
        self.spec
    }

    pub fn candle_mass_kg(&self) -> f64 {
        self.candle_mass_kg
    }

    pub fn case_area_m2(&self) -> f64 {
        self.case_area_m2
    }

    pub fn burned_fraction(&self) -> f64 {
        self.burned_fraction
    }

    pub fn is_lit(&self) -> bool {
        self.lit
    }

    pub fn replace(&mut self) {
        self.burned_fraction = 0.0;
        self.lit = false;
        self.case_temp_k = REFERENCE_TEMP_K;
    }

    pub fn set_case_temp_k(&mut self, temp_k: f64) {
        if temp_k > 0.0 && !self.lit {
            self.case_temp_k = temp_k;
        }
    }

    fn shape(burned_fraction: f64) -> f64 {
        1.0 + OUTPUT_SHAPE * (1.0 - 2.0 * burned_fraction.clamp(0.0, 1.0))
    }

    pub fn step(&mut self, lanyard_pulled: bool, cabin_temp_k: f64, faults: GeneratorFaults, dt_s: f64) -> GeneratorOutputs {
        let dt = dt_s.max(0.0);
        let cabin = if cabin_temp_k > 0.0 { cabin_temp_k } else { REFERENCE_TEMP_K };

        let quench_at = (1.0 - faults.quench_fraction.clamp(0.0, 1.0)).min(1.0);
        let done = |burned: f64| burned >= quench_at - BURN_COMPLETE_EPS;
        if !self.lit && !done(self.burned_fraction) {
            let dud = faults.dud_initiator.clamp(0.0, 1.0) >= 1.0;
            let spontaneous = faults.inadvertent_ignition.clamp(0.0, 1.0) >= 1.0;
            if (lanyard_pulled && !dud) || spontaneous {
                self.lit = true;
            }
        }
        if self.lit && done(self.burned_fraction) {
            self.lit = false;
        }

        let mut o2_kg_s = 0.0;
        let mut heat_w = 0.0;
        if self.lit && dt > 0.0 {
            let shape = Self::shape(self.burned_fraction);
            o2_kg_s = self.mean_o2_kg_s * shape;
            heat_w = self.mean_heat_w * shape;
            let advance = dt / self.spec.rated_duration_s.max(1.0);
            let next = (self.burned_fraction + advance).min(quench_at.min(1.0));
            let actually = (next - self.burned_fraction).max(0.0);
            if advance > 0.0 {
                let fraction_of_frame = actually / advance;
                o2_kg_s *= fraction_of_frame;
                heat_w *= fraction_of_frame;
            }
            self.burned_fraction = next;
            if done(self.burned_fraction) {
                self.lit = false;
            }
        } else if self.lit {
            let shape = Self::shape(self.burned_fraction);
            o2_kg_s = self.mean_o2_kg_s * shape;
            heat_w = self.mean_heat_w * shape;
        }

        let t = self.case_temp_k.max(1.0);
        let radiative = CASE_EMISSIVITY * STEFAN_BOLTZMANN * self.case_area_m2 * (t * t + cabin * cabin) * (t + cabin);
        let convective = CASE_CONVECTION_W_PER_M2_K * self.case_area_m2;
        let carried = o2_kg_s * gas::CP_O2_J_PER_KG_K;
        let k = (radiative + convective + carried).max(1e-6);
        let target = cabin + heat_w / k;
        let tau = self.heat_capacity_j_per_k / k;
        self.case_temp_k = gas::relax(self.case_temp_k, target, tau, dt).max(1.0);

        GeneratorOutputs {
            firing: self.lit,
            spent: done(self.burned_fraction),
            burned_fraction: self.burned_fraction,
            o2_kg_s,
            o2_l_per_min: o2_kg_s * self.liters_per_kg * 60.0,
            heat_w,
            case_temp_k: self.case_temp_k,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_chemistry_gives_the_oxygen_the_published_figures_expect() {
        let y = candle_yield();
        assert!(y.o2_mol_per_kg > 11.0 && y.o2_mol_per_kg < 12.7, "{}", y.o2_mol_per_kg);
        assert!(y.heat_j_per_kg > 600_000.0 && y.heat_j_per_kg < 900_000.0, "{}", y.heat_j_per_kg);
    }

    #[test]
    fn the_candle_mass_the_chemistry_implies_is_a_plausible_object() {
        let s = GeneratorSpec::three_person();
        let m = s.candle_mass_kg();
        assert!(m > 0.10 && m < 0.40, "{m} kg");
        assert!(GeneratorSpec::two_person().candle_mass_kg() < m);
        assert!(GeneratorSpec::four_person().candle_mass_kg() > m);
    }

    #[test]
    fn a_generator_burns_for_its_rated_duration_and_is_then_spent() {
        let mut g = ChemicalOxygenGenerator::new(GeneratorSpec::three_person());
        let mut out = g.step(true, 297.15, GeneratorFaults::default(), 1.0);
        assert!(out.firing);
        let mut total_kg = out.o2_kg_s;
        for _ in 1..899 {
            out = g.step(true, 297.15, GeneratorFaults::default(), 1.0);
            total_kg += out.o2_kg_s;
        }
        assert!(out.firing, "at {} s it should still be burning", 899);
        assert!(!out.spent);
        out = g.step(true, 297.15, GeneratorFaults::default(), 1.0);
        total_kg += out.o2_kg_s;
        assert!(out.spent, "at the rated duration it must be done");
        let rated = GeneratorSpec::three_person().rated_output_kg();
        assert!((total_kg - rated).abs() / rated < 0.01, "delivered {total_kg} kg against a rated {rated} kg");
        let after = g.step(true, 297.15, GeneratorFaults::default(), 60.0);
        assert_eq!(after.o2_kg_s, 0.0);
        assert!(!after.firing && after.spent);
    }

    #[test]
    fn the_output_is_front_loaded_the_way_an_emergency_descent_needs() {
        let mut g = ChemicalOxygenGenerator::new(GeneratorSpec::three_person());
        let first = g.step(true, 297.15, GeneratorFaults::default(), 1.0).o2_kg_s;
        for _ in 0..800 {
            g.step(true, 297.15, GeneratorFaults::default(), 1.0);
        }
        let last = g.step(true, 297.15, GeneratorFaults::default(), 1.0).o2_kg_s;
        assert!(first > last * 2.0, "first {first} last {last}");
    }

    #[test]
    fn a_running_generator_reaches_the_case_temperature_the_literature_records() {
        let mut g = ChemicalOxygenGenerator::new(GeneratorSpec::three_person());
        let mut out = GeneratorOutputs::default();
        for _ in 0..600 {
            out = g.step(true, 297.15, GeneratorFaults::default(), 1.0);
        }
        let c = out.case_temp_k - 273.15;
        println!("OXYGEN generator case temperature after 600 s: {c:.1} C (published: up to about 260 C)");
        assert!(c > 200.0 && c < 350.0, "case reached {c} C; the published figure is about 260 C");
    }

    #[test]
    fn a_spent_generator_cools_back_down_to_the_cabin() {
        let mut g = ChemicalOxygenGenerator::new(GeneratorSpec::three_person());
        for _ in 0..1000 {
            g.step(true, 297.15, GeneratorFaults::default(), 1.0);
        }
        let mut out = GeneratorOutputs::default();
        for _ in 0..3600 {
            out = g.step(false, 297.15, GeneratorFaults::default(), 1.0);
        }
        assert!((out.case_temp_k - 297.15).abs() < 2.0, "{} K", out.case_temp_k);
        assert_eq!(out.heat_w, 0.0);
    }

    #[test]
    fn a_dud_initiator_never_lights_it() {
        let mut g = ChemicalOxygenGenerator::new(GeneratorSpec::three_person());
        let faults = GeneratorFaults { dud_initiator: 1.0, ..Default::default() };
        let mut out = GeneratorOutputs::default();
        for _ in 0..600 {
            out = g.step(true, 297.15, faults, 1.0);
        }
        assert!(!out.firing);
        assert_eq!(out.o2_kg_s, 0.0);
        assert_eq!(out.burned_fraction, 0.0);
        assert!((out.case_temp_k - 297.15).abs() < 2.0, "a dud unit stays cold: {} K", out.case_temp_k);
    }

    #[test]
    fn an_inadvertent_ignition_lights_it_with_nothing_pulling_on_it() {
        let mut g = ChemicalOxygenGenerator::new(GeneratorSpec::three_person());
        let faults = GeneratorFaults { inadvertent_ignition: 1.0, ..Default::default() };
        let out = g.step(false, 297.15, faults, 1.0);
        assert!(out.firing);
        assert!(out.heat_w > 0.0);
        let mut hot = out;
        for _ in 0..600 {
            hot = g.step(false, 297.15, faults, 1.0);
        }
        assert!(hot.case_temp_k > 420.0, "{} K", hot.case_temp_k);
    }

    #[test]
    fn a_quenching_candle_runs_short_and_leaves_the_rest_unburnt() {
        let mut healthy = ChemicalOxygenGenerator::new(GeneratorSpec::three_person());
        let mut sick = ChemicalOxygenGenerator::new(GeneratorSpec::three_person());
        let faults = GeneratorFaults { quench_fraction: 0.6, ..Default::default() };
        let mut healthy_total = 0.0;
        let mut sick_total = 0.0;
        let mut sick_out = GeneratorOutputs::default();
        for _ in 0..900 {
            healthy_total += healthy.step(true, 297.15, GeneratorFaults::default(), 1.0).o2_kg_s;
            sick_out = sick.step(true, 297.15, faults, 1.0);
            sick_total += sick_out.o2_kg_s;
        }
        assert!(sick_total < healthy_total * 0.75, "sick {sick_total} healthy {healthy_total}");
        assert!(sick_out.spent);
        assert!(sick_out.burned_fraction < 0.45, "{}", sick_out.burned_fraction);
        assert_eq!(sick_out.o2_kg_s, 0.0);
    }

    #[test]
    fn nothing_moves_and_nothing_breaks_at_rest() {
        let mut g = ChemicalOxygenGenerator::new(GeneratorSpec::three_person());
        let a = g.step(false, 297.15, GeneratorFaults::default(), 0.0);
        let b = g.step(false, 297.15, GeneratorFaults::default(), 0.0);
        assert_eq!(a, b);
        assert!(a.case_temp_k.is_finite() && a.o2_kg_s == 0.0 && a.heat_w == 0.0);
        let c = g.step(true, 0.0, GeneratorFaults::default(), 0.0);
        assert!(c.case_temp_k.is_finite());
    }
}
