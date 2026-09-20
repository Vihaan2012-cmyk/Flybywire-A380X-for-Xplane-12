//! A passenger chemical oxygen generator: a sodium chlorate candle that
//! is lit once, burns for its rated time, gets extremely hot doing it, and
//! is then scrap.
//!
//! This is deliberately *not* the bottle model with different numbers. A
//! generator has no pressure, no quantity gauge and no way to be turned
//! off; it has a state of charge that only ever goes one way, an output
//! that is fixed by chemistry rather than by demand, and a case that
//! reaches a few hundred degrees. Modelling it as a small bottle would get
//! every one of those wrong.
//!
//! ## Where the numbers come from
//!
//! The published figures for this class of generator are its *duration*
//! (about 15 minutes) and its *oxygen output* (a two-person unit yields at
//! least 42 litres, a three-person at least 62, a four-person at least 84,
//! over that duration). Neither of those says anything about heat, and the
//! heat is the interesting part -- it is what melts a PSU liner, what
//! makes an inadvertent firing a cargo-hold fire, and what the cabin
//! notices when two hundred of them light at once.
//!
//! So the heat is *derived*, from the same chemistry that produces the
//! oxygen. [`candle_yield`] takes the published candle composition and the
//! standard enthalpies of formation of the reactants and products and
//! computes both the net oxygen and the net heat per kilogram of candle;
//! the candle mass then follows from the published output, and the heat
//! follows from the candle mass. One chemistry, both numbers, nothing
//! fitted.
//!
//! The check that it is right is that the case temperature this produces
//! -- from an energy balance against free convection, radiation and the
//! enthalpy carried away by the oxygen itself -- lands near the 500 F
//! (260 C) exterior temperature the accident literature records for these
//! units. That is asserted below rather than assumed.
//!
//! ## The reaction
//!
//! ```text
//! NaClO3(s) -> NaCl(s) + 1.5 O2(g)      exothermic, but only just
//! 4 Fe(s) + 3 O2(g) -> 2 Fe2O3(s)       strongly exothermic
//! ```
//!
//! The chlorate decomposition is what makes the oxygen; on its own it
//! barely sustains itself, which is why a few percent of iron powder is
//! blended in as fuel. The iron burns some of the oxygen back and releases
//! roughly as much heat again as the chlorate does. Both effects are in
//! [`candle_yield`], which is why the net oxygen is a little below the
//! stoichiometric chlorate yield and the heat is about twice the chlorate
//! reaction's own.
//!
//! ## Sources
//!
//! * Standard enthalpies of formation (CRC Handbook): NaClO3(s)
//!   -365.4 kJ/mol, NaCl(s) -411.15 kJ/mol, Fe2O3(s) -824.2 kJ/mol.
//! * Molar masses: NaClO3 106.44 g/mol, Fe 55.845 g/mol.
//! * Specific heat of NaClO3, 100.1 J/(mol K) -- the candle's own thermal
//!   mass.
//! * Candle composition ~90 % sodium chlorate with ~4-5 % iron powder
//!   fuel and a few percent of barium peroxide and binder: the composition
//!   published across the chemical-oxygen-generator literature. The exact
//!   split inside that range is GENERIC and flagged below.
//! * Duration and per-size output: the figures quoted for transport
//!   chemical oxygen generators (e.g. the technical material supporting
//!   Transportation Safety Board of Canada report A98H0003, and TSO-C64
//!   style minimum outputs).
//! * Exterior temperature "up to about 500 F": the FAA/NTSB material on
//!   chemical oxygen generators following the ValuJet 592 investigation.
//! * Automatic presentation of passenger oxygen before the cabin exceeds
//!   15 000 ft: CS-25/FAR 25.1447(c)(1). See [`super::pax`].

use super::gas;

/// Standard enthalpy of formation of solid sodium chlorate, J/mol.
const HF_NACLO3_J_PER_MOL: f64 = -365_400.0;
/// Standard enthalpy of formation of solid sodium chloride, J/mol.
const HF_NACL_J_PER_MOL: f64 = -411_150.0;
/// Standard enthalpy of formation of solid haematite, J/mol.
const HF_FE2O3_J_PER_MOL: f64 = -824_200.0;
/// Molar mass of sodium chlorate, kg/mol.
const M_NACLO3_KG_PER_MOL: f64 = 0.106_44;
/// Molar mass of iron, kg/mol.
const M_FE_KG_PER_MOL: f64 = 0.055_845;
/// Specific heat of sodium chlorate, J/(kg K), from 100.1 J/(mol K).
const CANDLE_SPECIFIC_HEAT_J_PER_KG_K: f64 = 100.1 / M_NACLO3_KG_PER_MOL;

/// Sodium chlorate mass fraction of the candle. **GENERIC** within the
/// published 85-92 % band.
const CANDLE_NACLO3_MASS_FRACTION: f64 = 0.90;
/// Iron fuel mass fraction. **GENERIC** within the published 4-5 % band;
/// it sets how much of the chlorate's own oxygen is burnt back and how
/// much of the heat there is, so it is the single most consequential
/// composition number here.
const CANDLE_IRON_MASS_FRACTION: f64 = 0.05;

/// Pressed-candle density, kg/m^3. **GENERIC**: sodium chlorate crystals
/// are 2490 kg/m^3 and a pressed, binder-loaded candle is necessarily
/// less; 2000 is a plausible pressed density. It sets only the candle's
/// geometry, and through that its external area.
const CANDLE_DENSITY_KG_M3: f64 = 2000.0;

/// Length-to-diameter ratio of the candle. **GENERIC**: these units are
/// visibly long and slim so the reaction front has somewhere to travel.
const CANDLE_LENGTH_OVER_DIAMETER: f64 = 4.0;

/// Thickness of the insulation and outer case around the candle, m.
/// **GENERIC**: a real generator is lagged, both to protect its
/// surroundings and to keep the front from quenching. It only sets the
/// outside area, and a thicker case would give a *hotter* model, so this
/// is the conservative end.
const CASE_WALL_THICKNESS_M: f64 = 0.005;

/// Mass of the steel outer case, kg. **GENERIC**: small compared with the
/// candle's own thermal mass, which is what dominates the warm-up.
const CASE_STEEL_MASS_KG: f64 = 0.1;
/// Specific heat of steel, J/(kg K).
const STEEL_SPECIFIC_HEAT_J_PER_KG_K: f64 = 490.0;

/// Free-convection coefficient from the case to still cabin air,
/// W/(m^2 K). The textbook free-convection figure for a gas.
const CASE_CONVECTION_W_PER_M2_K: f64 = 10.0;
/// Emissivity of the painted case. **GENERIC**: a painted or oxidised
/// non-metallic surface is close to a black body in the infrared, and 0.9
/// is the standard engineering value for one.
const CASE_EMISSIVITY: f64 = 0.9;
/// Stefan-Boltzmann constant, W/(m^2 K^4). Exact in the 2019 SI.
const STEFAN_BOLTZMANN: f64 = 5.670_374_419e-8;

/// How front-loaded the output is: the flow starts at `1 + SHAPE` times
/// the mean and ends at `1 - SHAPE` times it, with the same total.
/// **GENERIC** in its exact value; the *behaviour* is real and required --
/// a generator's output is graded to follow an emergency descent, so it is
/// highest when the cabin is highest, and a flat output would understate
/// the first minutes and overstate the last.
const OUTPUT_SHAPE: f64 = 0.5;

/// How close to the end of the candle counts as the end of it. The burn
/// fraction is accumulated a frame at a time, so it arrives a few parts in
/// 1e13 short; one tolerance, used by every test of whether the burn is
/// over, keeps "spent" and "still alight" from disagreeing.
const BURN_COMPLETE_EPS: f64 = 1e-9;

/// Reference conditions the published litre figures are quoted at: one
/// standard atmosphere, and the same 21 C this area uses for cylinders, so
/// there is one reference state in the whole area.
const REFERENCE_TEMP_K: f64 = 294.15;

/// What a kilogram of candle yields.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CandleYield {
    /// Net moles of oxygen delivered (chlorate's yield less what the iron
    /// fuel burns back).
    pub o2_mol_per_kg: f64,
    /// Net heat released, J.
    pub heat_j_per_kg: f64,
}

/// The chemistry, once.
pub fn candle_yield() -> CandleYield {
    // NaClO3(s) -> NaCl(s) + 1.5 O2(g)
    let chlorate_mol = CANDLE_NACLO3_MASS_FRACTION / M_NACLO3_KG_PER_MOL;
    let chlorate_heat = chlorate_mol * (HF_NACLO3_J_PER_MOL - HF_NACL_J_PER_MOL).abs();
    let chlorate_o2 = chlorate_mol * 1.5;

    // 4 Fe(s) + 3 O2(g) -> 2 Fe2O3(s)
    let iron_mol = CANDLE_IRON_MASS_FRACTION / M_FE_KG_PER_MOL;
    let iron_heat = iron_mol * (2.0 * HF_FE2O3_J_PER_MOL / 4.0).abs();
    let iron_o2 = iron_mol * 0.75;

    CandleYield { o2_mol_per_kg: (chlorate_o2 - iron_o2).max(0.0), heat_j_per_kg: chlorate_heat + iron_heat }
}

/// One generator size, as the published figures specify it.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GeneratorSpec {
    /// How many masks this unit feeds.
    pub persons: f64,
    /// Published oxygen output over the full burn, litres at one
    /// atmosphere and [`REFERENCE_TEMP_K`].
    pub rated_output_liters: f64,
    /// Published burn duration, s.
    pub rated_duration_s: f64,
}

impl GeneratorSpec {
    /// The two-person unit: at least 42 litres over 15 minutes.
    pub fn two_person() -> Self {
        Self { persons: 2.0, rated_output_liters: 42.0, rated_duration_s: 900.0 }
    }
    /// The three-person unit: at least 62 litres over 15 minutes. The
    /// representative size for a cabin laid out in threes.
    pub fn three_person() -> Self {
        Self { persons: 3.0, rated_output_liters: 62.0, rated_duration_s: 900.0 }
    }
    /// The four-person unit: at least 84 litres over 15 minutes.
    pub fn four_person() -> Self {
        Self { persons: 4.0, rated_output_liters: 84.0, rated_duration_s: 900.0 }
    }

    /// Total oxygen delivered over a full burn, kg.
    pub fn rated_output_kg(&self) -> f64 {
        gas::mass_from_free_air_kg(self.rated_output_liters, REFERENCE_TEMP_K)
    }

    /// Candle mass needed to deliver that, kg -- derived from the
    /// chemistry, not quoted.
    pub fn candle_mass_kg(&self) -> f64 {
        let moles = self.rated_output_kg() / gas::M_O2_KG_PER_MOL;
        let yield_ = candle_yield();
        if yield_.o2_mol_per_kg > 0.0 {
            moles / yield_.o2_mol_per_kg
        } else {
            0.0
        }
    }

    /// Total heat the candle releases over a full burn, J.
    pub fn rated_heat_j(&self) -> f64 {
        self.candle_mass_kg() * candle_yield().heat_j_per_kg
    }

    /// External area of the finished unit, m^2.
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

    /// Thermal capacity of the unit, J/K: the candle plus its case.
    pub fn heat_capacity_j_per_k(&self) -> f64 {
        (self.candle_mass_kg() * CANDLE_SPECIFIC_HEAT_J_PER_KG_K + CASE_STEEL_MASS_KG * STEEL_SPECIFIC_HEAT_J_PER_KG_K).max(1.0)
    }
}

/// What can be wrong with one generator.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct GeneratorFaults {
    /// The percussion initiator does not light the candle when the lanyard
    /// is pulled: 0 fires normally, 1 never fires at all.
    pub dud_initiator: f64,
    /// The reaction front quenches part-way: the burn stops once this
    /// fraction of the candle is left, so the unit runs short and leaves
    /// unburnt candle behind.
    pub quench_fraction: f64,
    /// The initiator fires with nothing pulling on it: 1 lights the candle
    /// on the spot, with the masks still stowed.
    pub inadvertent_ignition: f64,
}

/// One frame of a generator's state.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct GeneratorOutputs {
    pub firing: bool,
    pub spent: bool,
    /// Fraction of the candle consumed so far.
    pub burned_fraction: f64,
    pub o2_kg_s: f64,
    /// The same flow in the units a cabin crew procedure uses.
    pub o2_l_per_min: f64,
    /// Chemical heat released this frame, W. All of it ends up in the
    /// cabin: what does not leave through the case leaves with the hot
    /// oxygen.
    pub heat_w: f64,
    pub case_temp_k: f64,
}

/// A generator, cold and ready.
#[derive(Clone, Debug)]
pub struct ChemicalOxygenGenerator {
    spec: GeneratorSpec,
    candle_mass_kg: f64,
    heat_capacity_j_per_k: f64,
    case_area_m2: f64,
    mean_o2_kg_s: f64,
    mean_heat_w: f64,
    /// Litres of oxygen (at the reference state) per kilogram, resolved
    /// once: the free-air inversion bisects, and this is read every frame.
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

    /// A turnaround: the spent unit is unbolted and a new one fitted.
    /// There is no other way to reset one, which is the whole difference
    /// between this and a bottle.
    pub fn replace(&mut self) {
        self.burned_fraction = 0.0;
        self.lit = false;
        self.case_temp_k = REFERENCE_TEMP_K;
    }

    /// Set the case temperature, so a cold-and-dark aircraft starts at the
    /// temperature of the cabin it is sitting in rather than at 21 C.
    pub fn set_case_temp_k(&mut self, temp_k: f64) {
        if temp_k > 0.0 && !self.lit {
            self.case_temp_k = temp_k;
        }
    }

    /// The output shaping factor at a given point through the burn: real
    /// generators are graded to deliver most when the cabin is highest.
    fn shape(burned_fraction: f64) -> f64 {
        1.0 + OUTPUT_SHAPE * (1.0 - 2.0 * burned_fraction.clamp(0.0, 1.0))
    }

    /// Advance one frame. `lanyard_pulled` is the mask being pulled down
    /// onto the wearer's face, which is what fires the initiator.
    pub fn step(&mut self, lanyard_pulled: bool, cabin_temp_k: f64, faults: GeneratorFaults, dt_s: f64) -> GeneratorOutputs {
        let dt = dt_s.max(0.0);
        let cabin = if cabin_temp_k > 0.0 { cabin_temp_k } else { REFERENCE_TEMP_K };

        // Ignition. A dud initiator is a failure to *start*; once a candle
        // is lit nothing about the initiator matters any more.
        let quench_at = (1.0 - faults.quench_fraction.clamp(0.0, 1.0)).min(1.0);
        // The burn fraction is reached by adding dt/duration a frame at a
        // time, so it lands a few parts in 1e13 short of its end rather
        // than exactly on it. One tolerance, used everywhere the burn is
        // asked whether it is over, so that "spent" and "still alight"
        // cannot disagree about the last frame.
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
            // The front travels at a fixed speed; the shaping is in the
            // composition it travels through, so the *mass* rate is the
            // plain one and both outputs share the same factor.
            let advance = dt / self.spec.rated_duration_s.max(1.0);
            let next = (self.burned_fraction + advance).min(quench_at.min(1.0));
            // Conservation over a partial last frame: only what actually
            // burnt counts.
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

        // Case temperature: chemical heat in, free convection and
        // radiation out, plus the enthalpy the oxygen itself carries away.
        // Linearised about the current temperature and stepped exactly, so
        // it is stable at any frame length and cannot overshoot.
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
        // The chlorate alone would give 12.7 mol/kg; the iron fuel burns
        // some of it back, so the net has to be a little under that and
        // clearly above zero.
        assert!(y.o2_mol_per_kg > 11.0 && y.o2_mol_per_kg < 12.7, "{}", y.o2_mol_per_kg);
        // The iron contributes about as much heat again as the chlorate,
        // which is the reason it is in there.
        assert!(y.heat_j_per_kg > 600_000.0 && y.heat_j_per_kg < 900_000.0, "{}", y.heat_j_per_kg);
    }

    #[test]
    fn the_candle_mass_the_chemistry_implies_is_a_plausible_object() {
        let s = GeneratorSpec::three_person();
        let m = s.candle_mass_kg();
        // A three-person generator is a hand-sized canister; a couple of
        // hundred grams of candle in it.
        assert!(m > 0.10 && m < 0.40, "{m} kg");
        // And the sizes scale with the published outputs.
        assert!(GeneratorSpec::two_person().candle_mass_kg() < m);
        assert!(GeneratorSpec::four_person().candle_mass_kg() > m);
    }

    #[test]
    fn a_generator_burns_for_its_rated_duration_and_is_then_spent() {
        let mut g = ChemicalOxygenGenerator::new(GeneratorSpec::three_person());
        let mut out = g.step(true, 297.15, GeneratorFaults::default(), 1.0);
        assert!(out.firing);
        let mut total_kg = out.o2_kg_s;
        // One second short of the rated duration it must still be running.
        for _ in 1..899 {
            out = g.step(true, 297.15, GeneratorFaults::default(), 1.0);
            total_kg += out.o2_kg_s;
        }
        assert!(out.firing, "at {} s it should still be burning", 899);
        assert!(!out.spent);
        out = g.step(true, 297.15, GeneratorFaults::default(), 1.0);
        total_kg += out.o2_kg_s;
        assert!(out.spent, "at the rated duration it must be done");
        // And the delivered oxygen is the published output.
        let rated = GeneratorSpec::three_person().rated_output_kg();
        assert!((total_kg - rated).abs() / rated < 0.01, "delivered {total_kg} kg against a rated {rated} kg");
        // A spent generator is scrap: it produces nothing however long it
        // is left, and pulling the mask again does nothing.
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
        // The cross-check on the derived heat: nothing in this model was
        // told what temperature a generator reaches, and it has to land
        // near the published "up to about 500 F".
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
        // Which is a fire hazard precisely because the heat does not stop.
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
