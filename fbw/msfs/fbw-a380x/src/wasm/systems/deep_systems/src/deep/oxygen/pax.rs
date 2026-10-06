use super::gas;
use super::generator::{ChemicalOxygenGenerator, GeneratorFaults, GeneratorSpec};

pub const AUTO_DEPLOY_CABIN_ALT_FT: f64 = 14_000.0;

pub const REGULATORY_MAX_DEPLOY_CABIN_ALT_FT: f64 = 15_000.0;

pub const TYPICAL_THREE_CLASS_SEATS: f64 = 525.0;

pub const SPARE_MASK_FRACTION: f64 = 0.10;

pub const MASKS_PER_GENERATOR: f64 = 3.0;

pub const MAIN_DECK_SHARE: f64 = 0.65;

pub fn total_generator_count() -> f64 {
    (TYPICAL_THREE_CLASS_SEATS * (1.0 + SPARE_MASK_FRACTION) / MASKS_PER_GENERATOR).ceil()
}

#[derive(Clone, Copy, Debug)]
pub struct PassengerOxygenInputs {
    pub cabin_pressure_pa: f64,
    pub cabin_temp_k: f64,
    pub manual_deploy_commanded: bool,
    pub control_circuit_powered: bool,
    pub masks_pulled: bool,
}

impl Default for PassengerOxygenInputs {
    fn default() -> Self {
        Self { cabin_pressure_pa: 101_325.0, cabin_temp_k: 297.15, manual_deploy_commanded: false, control_circuit_powered: true, masks_pulled: true }
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct PassengerOxygenFaults {
    pub latch_failed: f64,
    pub dud_initiators: f64,
    pub candle_quench: f64,
    pub inadvertent_ignition: f64,
    pub auto_deploy_controller: f64,
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct BankOutputs {
    pub name: &'static str,
    pub unit_count: f64,
    pub presented_fraction: f64,
    pub lit_fraction: f64,
    pub burned_fraction: f64,
    pub spent: bool,
    pub o2_kg_s: f64,
    pub unit_o2_l_per_min: f64,
    pub heat_w: f64,
    pub case_temp_k: f64,
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct PassengerOxygenOutputs {
    pub banks: [BankOutputs; 2],
    pub cabin_altitude_ft: f64,
    pub masks_deployed: bool,
    pub generators_running: bool,
    pub supply_remaining_fraction: f64,
    pub total_o2_kg_s: f64,
    pub total_heat_w: f64,
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
        let commanded = inputs.manual_deploy_commanded || auto_fires;
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
                bank.presented_fraction = bank.presented_fraction.max(command_reach * latch_ok);
            }
            let pulled = if inputs.masks_pulled { bank.presented_fraction * initiator_ok } else { 0.0 };
            bank.lit_fraction = bank.lit_fraction.max(pulled).max(spontaneous);

            let gen_faults = GeneratorFaults { dud_initiator: 0.0, quench_fraction: faults.candle_quench, inadvertent_ignition: 0.0 };
            let out = bank.generator.step(bank.lit_fraction > 0.0, inputs.cabin_temp_k, gen_faults, dt_s);

            let units = bank.unit_count * bank.lit_fraction;
            let o2 = out.o2_kg_s * units;
            let heat = out.heat_w * units;
            total_o2 += o2;
            total_heat += heat;
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

        let back_down = PassengerOxygenInputs { cabin_pressure_pa: cabin_pressure_at_ft(6000.0), ..Default::default() };
        let mid = run(&mut sys, back_down, PassengerOxygenFaults::default(), 300);
        assert!(mid.generators_running, "a lit candle cannot be put out");
        assert!(mid.masks_deployed);

        let done = run(&mut sys, back_down, PassengerOxygenFaults::default(), 700);
        assert!(!done.generators_running);
        assert!(done.supply_remaining_fraction < 0.02, "{}", done.supply_remaining_fraction);
        assert_eq!(done.total_o2_kg_s, 0.0);

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
        assert!(out.total_heat_w > 20_000.0 && out.total_heat_w < 80_000.0, "{} W", out.total_heat_w);
        assert!(out.banks[0].heat_w > out.banks[1].heat_w, "the bigger deck carries the bigger load");
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
