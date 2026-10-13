//! Every fault this directory's models accept, collected into one struct --
//! item 5 of the backlog. `Default` is all-healthy, matching the brief's
//! convention. See `FAILURES.md` for the full list with each fault's model
//! field, magnitude meaning and effect, and `registry.rs` for its
//! registration through `deep::api`.
//!
//! Speed and EGT sensor faults now live under `ecb` (`ecb::EcbFaults`,
//! per-channel) rather than as flat top-level fields: a real dual-channel
//! ECB (item 2) means a single sensor's fault is graceful (the other
//! channel/voting covers it) rather than immediately hazardous, which is
//! exactly what per-channel fault modelling is for.
//!
//! `starter_duty_model_fault` has no single subsystem home: it biases
//! `life::StarterDutyCycle`'s own tracked cranking-heat estimate (item 3),
//! which is life-tracking state `apu.rs` owns directly, not a field on any
//! one subsystem's own `...Faults` struct.

use super::ecb::EcbFaults;
use super::fire::FireFaults;
use super::fuel_control::FuelControlFaults;
use super::generators::GeneratorFaults;
use super::inlet_door::InletDoorFaults;
use super::load_compressor::LoadCompressorFaults;
use super::oil::OilFaults;
use super::power_section::PowerSectionFaults;
use super::starter::StarterFaults;

#[derive(Clone, Copy, Debug, Default)]
pub struct ApuFaults {
    pub power_section: PowerSectionFaults,
    pub load_compressor: LoadCompressorFaults,
    pub starter: StarterFaults,
    pub gen1: GeneratorFaults,
    pub gen2: GeneratorFaults,
    pub oil: OilFaults,
    pub fuel_control: FuelControlFaults,
    pub inlet_door: InletDoorFaults,
    pub fire: FireFaults,
    pub ecb: EcbFaults,
    /// Starter duty-cycle thermal model fault, 0 healthy .. 1 (see module
    /// docs); at 1.0 the model tracks no cranking heat at all.
    pub starter_duty_model_fault: f64,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_is_fully_healthy() {
        let f = ApuFaults::default();
        assert_eq!(f.power_section.compressor_efficiency_loss, 0.0);
        assert_eq!(f.power_section.turbine_efficiency_loss, 0.0);
        assert_eq!(f.load_compressor.igv_jam, 0.0);
        assert_eq!(f.load_compressor.scv_jam, 0.0);
        assert_eq!(f.starter.starter_degradation, 0.0);
        assert_eq!(f.starter.igniter_failure, 0.0);
        assert_eq!(f.gen1.efficiency_loss, 0.0);
        assert!(!f.gen1.overload_protection_failed);
        assert_eq!(f.oil.leak, 0.0);
        assert_eq!(f.fuel_control.metering_valve_jam, 0.0);
        assert_eq!(f.inlet_door.jam, 0.0);
        assert_eq!(f.fire.loop_failure, 0.0);
        assert_eq!(f.fire.squib_failure, 0.0);
        assert_eq!(f.ecb.channel_a.speed_sensor.bias, 0.0);
        assert!(!f.ecb.channel_a.speed_sensor.failed);
        assert_eq!(f.ecb.channel_b.processing_fault, 0.0);
        assert_eq!(f.starter_duty_model_fault, 0.0);
    }
}
