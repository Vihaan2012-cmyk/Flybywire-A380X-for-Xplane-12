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
