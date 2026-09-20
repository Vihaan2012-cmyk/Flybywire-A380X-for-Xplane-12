//! The passenger oxygen system: two decks of chemical generators, what
//! makes them drop, and what happens to the cabin when they all light.
//!
//! The A380's passenger supply is chemical, not gaseous, so this is a
//! different system from [`super::crew`] and not a second copy of it. It
//! has no pressure, no gauge, no shutoff and no way back: the generators
//! sit inert for the life of the panel, and the first time they are used
//! they are used up.
//!
//! ## Deployment
//!
//! CS-25/FAR 25.1447(c)(1) requires the masks to present *automatically*
//! before the cabin altitude exceeds 15 000 ft, and Airbus's own trigger
//! is commonly quoted at about 14 000 ft. Both are in
//! [`AUTO_DEPLOY_CABIN_ALT_FT`]: the modelled threshold is the quoted
//! 14 000 ft, and the regulatory 15 000 ft is the ceiling the model must
//! not exceed, which this module's tests check.
//!
//! Presentation and ignition are two separate things, which is why they
//! are two separate failures. The latch releases the PSU door and the mask
//! falls; nothing burns until somebody pulls the mask down onto their
//! face, which is what fires the percussion initiator. A latch that does
//! not release and an initiator that does not fire look identical from the
//! flight deck and are completely different in the cabin.
//!
//! ## How many generators there are
//!
//! There is no published A380 generator count. There is a published
//! typical three-class seat count (525), a regulatory mask count (seats
//! plus ten percent spares, 25.1447(c)(1)) and a generator size that feeds
//! two, three or four masks. The count here is derived from those and
//! labelled as derived; it is the number that scales the cabin heat load,
//! so it is called out rather than buried.
//!
//! ## Why the heat matters
//!
//! One generator is about 180 watts of chemistry in a canister the size of
//! a fist, and there are the better part of two hundred of them. A full
//! deployment puts tens of kilowatts into the cabin at the exact moment
//! the packs are being asked to handle an emergency descent -- and does it
//! whether or not anybody wanted it, which is what makes an inadvertent
//! ignition in a stowed PSU a fire and not an inconvenience. The heat is
//! published per deck so that the thermal and air-conditioning areas can
//! take it as a real load rather than a flag.

use super::gas;
use super::generator::{ChemicalOxygenGenerator, GeneratorFaults, GeneratorSpec};

/// Cabin altitude at which the masks present automatically, ft. Airbus's
/// commonly quoted trigger; the regulatory limit it has to sit under is
/// [`REGULATORY_MAX_DEPLOY_CABIN_ALT_FT`].
pub const AUTO_DEPLOY_CABIN_ALT_FT: f64 = 14_000.0;

/// The cabin altitude CS-25/FAR 25.1447(c)(1) requires automatic
/// presentation before, ft.
pub const REGULATORY_MAX_DEPLOY_CABIN_ALT_FT: f64 = 15_000.0;

/// Airbus's published typical three-class A380-800 seat count.
pub const TYPICAL_THREE_CLASS_SEATS: f64 = 525.0;

/// Spare masks beyond the seat count, as a fraction: CS-25/FAR
/// 25.1447(c)(1)'s "at least ten percent more outlets and masks than
/// seats".
pub const SPARE_MASK_FRACTION: f64 = 0.10;

/// Masks per generator. **GENERIC** within the published family of two-,
/// three- and four-person units: a cabin laid out mostly in threes is fed
/// mostly by three-person generators, which is also the size whose
/// published output (62 litres) the representative unit here uses.
pub const MASKS_PER_GENERATOR: f64 = 3.0;

/// Share of the cabin on the main deck. **GENERIC**, from the decks'
/// relative floor areas -- the A380's main deck is appreciably the larger
/// of the two. It splits the generator count and so the heat between the
/// two thermal zones; the total is unaffected by it.
pub const MAIN_DECK_SHARE: f64 = 0.65;

/// Total generators in the cabin, derived from the seat count, the
/// regulatory spare allowance and the unit size.
pub fn total_generator_count() -> f64 {
    (TYPICAL_THREE_CLASS_SEATS * (1.0 + SPARE_MASK_FRACTION) / MASKS_PER_GENERATOR).ceil()
}

/// What the passenger system is being asked to do this frame.
#[derive(Clone, Copy, Debug)]
pub struct PassengerOxygenInputs {
    pub cabin_pressure_pa: f64,
    pub cabin_temp_k: f64,
    /// The flight deck's MASK MAN ON selection: a direct command that
    /// bypasses the altitude trigger.
    pub manual_deploy_commanded: bool,
    /// Whether the deployment control circuit has a bus. The automatic
    /// trigger is electrical; without it the masks only come down if
    /// somebody commands them.
    pub control_circuit_powered: bool,
    /// Whether the passengers pull the masks down once they appear. Real,
    /// and not automatic: a presented mask that nobody pulls never fires
    /// its generator.
    pub masks_pulled: bool,
}

impl Default for PassengerOxygenInputs {
    fn default() -> Self {
        Self { cabin_pressure_pa: 101_325.0, cabin_temp_k: 297.15, manual_deploy_commanded: false, control_circuit_powered: true, masks_pulled: true }
    }
}

/// What can be wrong with it. Every magnitude is a *fraction of the
/// units*, because there are two hundred of them and they do not all fail
/// together -- which is the whole reason these are fractions and not
/// flags.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct PassengerOxygenFaults {
    /// Fraction of PSU latches that do not release when commanded, so
    /// those masks never appear.
    pub latch_failed: f64,
    /// Fraction of presented units whose percussion initiator is dud.
    pub dud_initiators: f64,
    /// How far the candles quench short of the end of their burn.
    pub candle_quench: f64,
    /// Fraction of units that light with nothing commanding them, masks
    /// still stowed.
    pub inadvertent_ignition: f64,
    /// The automatic deployment controller's own failure: 1.0 means the
    /// altitude trigger never fires at all and only the manual command
    /// works.
    pub auto_deploy_controller: f64,
}

/// One deck's worth of generators.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct BankOutputs {
    pub name: &'static str,
    pub unit_count: f64,
    /// Fraction of this deck's masks that have actually presented.
    pub presented_fraction: f64,
    /// Fraction of this deck's units that are burning or have burnt.
    pub lit_fraction: f64,
    pub burned_fraction: f64,
    pub spent: bool,
    /// Oxygen this deck is producing, kg/s, summed over its lit units.
    pub o2_kg_s: f64,
    /// Per lit unit, in the units a cabin procedure uses.
    pub unit_o2_l_per_min: f64,
    /// Heat this deck is putting into its cabin zone, W.
    pub heat_w: f64,
    /// One unit's case temperature, K.
    pub case_temp_k: f64,
}

/// The passenger system's frame.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct PassengerOxygenOutputs {
    pub banks: [BankOutputs; 2],
    pub cabin_altitude_ft: f64,
    /// Whether any masks have presented.
    pub masks_deployed: bool,
    /// Whether anything is burning anywhere.
    pub generators_running: bool,
    /// Supply remaining across the whole cabin, 0..1, weighted by unit
    /// count. Only ever falls.
    pub supply_remaining_fraction: f64,
    pub total_o2_kg_s: f64,
    pub total_heat_w: f64,
    /// How long the burning generators have left, s.
    pub remaining_duration_s: f64,
}

#[derive(Clone, Debug)]
struct Bank {
    name: &'static str,
    unit_count: f64,
    generator: ChemicalOxygenGenerator,
    presented_fraction: f64,
    lit_fraction: f64,
}

/// The passenger oxygen system, running.
#[derive(Clone, Debug)]
pub struct PassengerOxygenSystem {
    banks: [Bank; 2],
}

impl Default for PassengerOxygenSystem {
    fn default() -> Self {
        Self::new()
    }
}

impl PassengerOxygenSystem {
    pub fn new() -> Self {
        let total = total_generator_count();
        let main = (total * MAIN_DECK_SHARE).round();
        let upper = total - main;
        Self {
            banks: [
                Bank {
                    name: "MAIN_DECK",
                    unit_count: main,
                    generator: ChemicalOxygenGenerator::new(GeneratorSpec::three_person()),
                    presented_fraction: 0.0,
                    lit_fraction: 0.0,
                },
                Bank {
                    name: "UPPER_DECK",
                    unit_count: upper,
                    generator: ChemicalOxygenGenerator::new(GeneratorSpec::three_person()),
                    presented_fraction: 0.0,
                    lit_fraction: 0.0,
                },
            ],
        }
    }

    pub fn unit_counts(&self) -> [f64; 2] {
        [self.banks[0].unit_count, self.banks[1].unit_count]
    }

    /// A turnaround: every spent generator is replaced and every PSU door
    /// is latched shut again. There is no in-flight equivalent, which is
    /// the point.
    pub fn service(&mut self) {
        for b in self.banks.iter_mut() {
            b.generator.replace();
            b.presented_fraction = 0.0;
            b.lit_fraction = 0.0;
        }
    }

    pub fn step(&mut self, inputs: PassengerOxygenInputs, faults: PassengerOxygenFaults, dt_s: f64) -> PassengerOxygenOutputs {
        let cabin_altitude_ft = gas::pressure_altitude_ft(inputs.cabin_pressure_pa);

        let auto_healthy = 1.0 - faults.auto_deploy_controller.clamp(0.0, 1.0);
        let auto_fires = inputs.control_circuit_powered && cabin_altitude_ft >= AUTO_DEPLOY_CABIN_ALT_FT && auto_healthy > 0.0;
        // The manual command is a separate path and is not fed through the
        // automatic controller: that is the whole reason it exists.
        let commanded = inputs.manual_deploy_commanded || auto_fires;
        // A part-failed controller releases part of the cabin.
        let command_reach = if inputs.manual_deploy_commanded { 1.0 } else { auto_healthy };

        let latch_ok = 1.0 - faults.latch_failed.clamp(0.0, 1.0);
        let initiator_ok = 1.0 - faults.dud_initiators.clamp(0.0, 1.0);
        let spontaneous = faults.inadvertent_ignition.clamp(0.0, 1.0);

        let mut banks_out = [BankOutputs::default(); 2];
        let mut total_o2 = 0.0;
        let mut total_heat = 0.0;
        let mut weighted_burn = 0.0;
        let mut total_units = 0.0;
        let mut running = false;
        let mut deployed = false;
        let mut remaining_duration_s = 0.0f64;

        for (i, bank) in self.banks.iter_mut().enumerate() {
            if commanded {
                // Latching: doors that have opened stay open.
                bank.presented_fraction = bank.presented_fraction.max(command_reach * latch_ok);
            }
            // Units that have caught light, ever. A mask has to have
            // presented *and* been pulled for its initiator to fire;
            // inadvertent ignition needs neither.
            let pulled = if inputs.masks_pulled { bank.presented_fraction * initiator_ok } else { 0.0 };
            bank.lit_fraction = bank.lit_fraction.max(pulled).max(spontaneous);

            let gen_faults = GeneratorFaults { dud_initiator: 0.0, quench_fraction: faults.candle_quench, inadvertent_ignition: 0.0 };
            let out = bank.generator.step(bank.lit_fraction > 0.0, inputs.cabin_temp_k, gen_faults, dt_s);

            let units = bank.unit_count * bank.lit_fraction;
            let o2 = out.o2_kg_s * units;
            let heat = out.heat_w * units;
            total_o2 += o2;
            total_heat += heat;
            // Only the units that actually lit have used anything up: a
            // cabin where half the initiators were duds still has half a
            // cabin's worth of unfired generators in it.
            weighted_burn += out.burned_fraction * bank.unit_count * bank.lit_fraction;
            total_units += bank.unit_count;
            running |= out.firing;
            deployed |= bank.presented_fraction > 0.0;
            if out.firing {
                let left = (1.0 - faults.candle_quench.clamp(0.0, 1.0) - out.burned_fraction).max(0.0) * bank.generator.spec().rated_duration_s;
                remaining_duration_s = remaining_duration_s.max(left);
            }

            banks_out[i] = BankOutputs {
                name: bank.name,
                unit_count: bank.unit_count,
                presented_fraction: bank.presented_fraction,
                lit_fraction: bank.lit_fraction,
                burned_fraction: out.burned_fraction,
                spent: out.spent && bank.lit_fraction > 0.0,
                o2_kg_s: o2,
                unit_o2_l_per_min: out.o2_l_per_min,
                heat_w: heat,
                case_temp_k: out.case_temp_k,
            };
        }

        PassengerOxygenOutputs {
            banks: banks_out,
            cabin_altitude_ft,
            masks_deployed: deployed,
            generators_running: running,
            supply_remaining_fraction: if total_units > 0.0 { (1.0 - weighted_burn / total_units).clamp(0.0, 1.0) } else { 0.0 },
            total_o2_kg_s: total_o2,
            total_heat_w: total_heat,
            remaining_duration_s,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The ISA pressure at a cabin altitude, for setting up a cabin.
    fn cabin_pressure_at_ft(ft: f64) -> f64 {
        101_325.0 * (1.0 - ft * 0.3048 / 44_330.77).powf(1.0 / 0.190_263_1)
    }

    fn run(sys: &mut PassengerOxygenSystem, inputs: PassengerOxygenInputs, faults: PassengerOxygenFaults, seconds: usize) -> PassengerOxygenOutputs {
        let mut out = sys.step(inputs, faults, 0.0);
        for _ in 0..seconds {
            out = sys.step(inputs, faults, 1.0);
        }
        out
    }

    #[test]
    fn the_derived_generator_count_is_a_plausible_cabin() {
        let n = total_generator_count();
        assert!(n > 150.0 && n < 250.0, "{n} generators");
        let sys = PassengerOxygenSystem::new();
        let [main, upper] = sys.unit_counts();
        assert!((main + upper - n).abs() < 1e-9);
        assert!(main > upper, "the main deck is the larger of the two");
    }

    #[test]
    fn the_masks_stay_up_below_the_threshold_and_drop_above_it() {
        let mut sys = PassengerOxygenSystem::new();
        let low = PassengerOxygenInputs { cabin_pressure_pa: cabin_pressure_at_ft(9000.0), ..Default::default() };
        let out = run(&mut sys, low, PassengerOxygenFaults::default(), 60);
        assert!(!out.masks_deployed);
        assert_eq!(out.total_o2_kg_s, 0.0);
        assert_eq!(out.supply_remaining_fraction, 1.0);

        let high = PassengerOxygenInputs { cabin_pressure_pa: cabin_pressure_at_ft(14_500.0), ..Default::default() };
        let out = run(&mut sys, high, PassengerOxygenFaults::default(), 10);
        assert!(out.masks_deployed);
        assert!(out.generators_running);
        assert!(out.total_o2_kg_s > 0.0);
    }

    #[test]
    fn the_trigger_sits_under_the_regulatory_ceiling() {
        // 25.1447(c)(1) says the masks must present *before* the cabin
        // exceeds 15 000 ft. This checks the model is compliant rather
        // than just close.
        let mut sys = PassengerOxygenSystem::new();
        let at_limit = PassengerOxygenInputs { cabin_pressure_pa: cabin_pressure_at_ft(REGULATORY_MAX_DEPLOY_CABIN_ALT_FT - 100.0), ..Default::default() };
        let out = sys.step(at_limit, PassengerOxygenFaults::default(), 1.0);
        assert!(out.masks_deployed);
        assert!(AUTO_DEPLOY_CABIN_ALT_FT < REGULATORY_MAX_DEPLOY_CABIN_ALT_FT);
    }

    #[test]
    fn once_fired_the_generators_run_to_exhaustion_and_do_not_come_back() {
        let mut sys = PassengerOxygenSystem::new();
        let high = PassengerOxygenInputs { cabin_pressure_pa: cabin_pressure_at_ft(20_000.0), ..Default::default() };
        let out = run(&mut sys, high, PassengerOxygenFaults::default(), 10);
        assert!(out.generators_running);

        // The crew get the cabin back down. The generators do not care.
        let back_down = PassengerOxygenInputs { cabin_pressure_pa: cabin_pressure_at_ft(6000.0), ..Default::default() };
        let mid = run(&mut sys, back_down, PassengerOxygenFaults::default(), 300);
        assert!(mid.generators_running, "a lit candle cannot be put out");
        assert!(mid.masks_deployed);

        let done = run(&mut sys, back_down, PassengerOxygenFaults::default(), 700);
        assert!(!done.generators_running);
        assert!(done.supply_remaining_fraction < 0.02, "{}", done.supply_remaining_fraction);
        assert_eq!(done.total_o2_kg_s, 0.0);

        // And the only way back is a turnaround.
        sys.service();
        let after = sys.step(back_down, PassengerOxygenFaults::default(), 1.0);
        assert_eq!(after.supply_remaining_fraction, 1.0);
        assert!(!after.masks_deployed);
    }

    #[test]
    fn a_full_deployment_is_a_serious_heat_load_on_the_cabin() {
        let mut sys = PassengerOxygenSystem::new();
        let high = PassengerOxygenInputs { cabin_pressure_pa: cabin_pressure_at_ft(20_000.0), ..Default::default() };
        let out = run(&mut sys, high, PassengerOxygenFaults::default(), 60);
        // Two hundred units at about 180 W apiece, front-loaded.
        assert!(out.total_heat_w > 20_000.0 && out.total_heat_w < 80_000.0, "{} W", out.total_heat_w);
        assert!(out.banks[0].heat_w > out.banks[1].heat_w, "the bigger deck carries the bigger load");
        // And the total is the sum of the decks, so the thermal areas can
        // take them separately without double counting.
        assert!((out.banks[0].heat_w + out.banks[1].heat_w - out.total_heat_w).abs() < 1e-6);
    }

    #[test]
    fn a_manual_command_works_with_the_automatic_controller_dead() {
        let mut sys = PassengerOxygenSystem::new();
        let faults = PassengerOxygenFaults { auto_deploy_controller: 1.0, ..Default::default() };
        let high = PassengerOxygenInputs { cabin_pressure_pa: cabin_pressure_at_ft(20_000.0), ..Default::default() };
        let out = run(&mut sys, high, faults, 60);
        assert!(!out.masks_deployed, "a dead controller must not present the masks");
        assert_eq!(out.total_o2_kg_s, 0.0);

        let manual = PassengerOxygenInputs { manual_deploy_commanded: true, ..high };
        let out = run(&mut sys, manual, faults, 10);
        assert!(out.masks_deployed, "the manual path is independent of the controller");
        assert!(out.total_o2_kg_s > 0.0);
    }

    #[test]
    fn an_unpowered_controller_does_not_deploy_either() {
        let mut sys = PassengerOxygenSystem::new();
        let high = PassengerOxygenInputs { cabin_pressure_pa: cabin_pressure_at_ft(20_000.0), control_circuit_powered: false, ..Default::default() };
        let out = run(&mut sys, high, PassengerOxygenFaults::default(), 60);
        assert!(!out.masks_deployed);
    }

    #[test]
    fn jammed_latches_leave_that_fraction_of_the_cabin_with_no_mask() {
        let mut sys = PassengerOxygenSystem::new();
        let faults = PassengerOxygenFaults { latch_failed: 0.4, ..Default::default() };
        let high = PassengerOxygenInputs { cabin_pressure_pa: cabin_pressure_at_ft(20_000.0), ..Default::default() };
        let out = run(&mut sys, high, faults, 60);
        assert!((out.banks[0].presented_fraction - 0.6).abs() < 1e-9, "{}", out.banks[0].presented_fraction);
        // Fewer masks is fewer generators is less heat, which is the only
        // way this failure is visible from outside the cabin.
        let mut healthy = PassengerOxygenSystem::new();
        let full = run(&mut healthy, high, PassengerOxygenFaults::default(), 60);
        assert!(out.total_heat_w < full.total_heat_w * 0.7, "{} vs {}", out.total_heat_w, full.total_heat_w);
    }

    #[test]
    fn dud_initiators_present_the_masks_and_deliver_nothing_through_them() {
        let mut sys = PassengerOxygenSystem::new();
        let faults = PassengerOxygenFaults { dud_initiators: 1.0, ..Default::default() };
        let high = PassengerOxygenInputs { cabin_pressure_pa: cabin_pressure_at_ft(20_000.0), ..Default::default() };
        let out = run(&mut sys, high, faults, 60);
        assert!(out.masks_deployed, "the doors still open -- that is the latch's job, not the initiator's");
        assert_eq!(out.total_o2_kg_s, 0.0);
        assert_eq!(out.total_heat_w, 0.0);
        assert_eq!(out.supply_remaining_fraction, 1.0, "an unlit candle is still a candle");
    }

    #[test]
    fn an_inadvertent_ignition_burns_with_the_masks_still_stowed() {
        let mut sys = PassengerOxygenSystem::new();
        let faults = PassengerOxygenFaults { inadvertent_ignition: 0.02, ..Default::default() };
        let out = run(&mut sys, PassengerOxygenInputs::default(), faults, 600);
        assert!(!out.masks_deployed, "nothing has commanded the doors open");
        assert!(out.generators_running);
        assert!(out.total_heat_w > 0.0, "a few units alight is still a few hundred watts inside a panel");
        assert!(out.banks[0].case_temp_k > 420.0, "{} K", out.banks[0].case_temp_k);
        assert!(out.supply_remaining_fraction < 1.0);
    }

    #[test]
    fn quenching_candles_run_short_of_their_rated_duration() {
        let mut sys = PassengerOxygenSystem::new();
        let faults = PassengerOxygenFaults { candle_quench: 0.5, ..Default::default() };
        let high = PassengerOxygenInputs { cabin_pressure_pa: cabin_pressure_at_ft(20_000.0), ..Default::default() };
        let out = run(&mut sys, high, faults, 460);
        assert!(!out.generators_running, "half a candle is gone at 450 s and the front has quenched");
        let mut healthy = PassengerOxygenSystem::new();
        let ok = run(&mut healthy, high, PassengerOxygenFaults::default(), 460);
        assert!(ok.generators_running);
        assert!(ok.remaining_duration_s > 400.0, "{}", ok.remaining_duration_s);
    }

    #[test]
    fn masks_that_nobody_pulls_never_fire() {
        let mut sys = PassengerOxygenSystem::new();
        let high = PassengerOxygenInputs { cabin_pressure_pa: cabin_pressure_at_ft(20_000.0), masks_pulled: false, ..Default::default() };
        let out = run(&mut sys, high, PassengerOxygenFaults::default(), 120);
        assert!(out.masks_deployed);
        assert!(!out.generators_running);
        assert_eq!(out.total_o2_kg_s, 0.0);
    }

    #[test]
    fn nothing_breaks_at_rest() {
        let mut sys = PassengerOxygenSystem::new();
        let a = sys.step(PassengerOxygenInputs::default(), PassengerOxygenFaults::default(), 0.0);
        let b = sys.step(PassengerOxygenInputs::default(), PassengerOxygenFaults::default(), 0.0);
        assert_eq!(a, b);
        assert!(a.cabin_altitude_ft.abs() < 1.0);
        assert!(a.total_heat_w.is_finite() && a.total_o2_kg_s.is_finite());
    }
}
