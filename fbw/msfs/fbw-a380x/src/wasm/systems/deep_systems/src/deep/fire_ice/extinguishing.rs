use super::util::{clamp01, orifice_mass_flow_kg_s, PSI_TO_PA};

const HALON_MOLAR_MASS_KG_MOL: f64 = 0.1489;
const R_UNIVERSAL: f64 = 8.314462618;
const CHARGE_PRESSURE_21C_PA: f64 = 360.0 * PSI_TO_PA;
const CHARGE_REF_TEMP_K: f64 = 294.15;
pub const DESIGN_CONCENTRATION_VOLUME_FRACTION: f64 = 0.05;

#[derive(Clone, Copy, Debug, Default)]
pub struct BottleFaults {
    pub leak: f64,
    pub squib_failure: f64,
}

pub struct Bottle {
    agent_mass_kg: f64,
    design_charge_kg: f64,
    pressure_pa: f64,
    last_ambient_c: f64,
    volume_m3: f64,
    discharged: bool,
}

const LEAK_AREA_MAX_M2: f64 = 4.0e-8;
const LEAK_DISCHARGE_COEFFICIENT: f64 = 0.62;

const DISCHARGE_AREA_MAX_M2: f64 = 8.0e-5;

pub fn low_pressure_threshold_pa(ambient_c: f64) -> f64 {
    let temp_k = (ambient_c + 273.15).max(1.0);
    let full_pressure_at_temp = CHARGE_PRESSURE_21C_PA * (temp_k / CHARGE_REF_TEMP_K);
    full_pressure_at_temp * 0.8
}

impl Bottle {
    pub fn new(design_charge_kg: f64, volume_m3: f64) -> Self {
        Self {
            agent_mass_kg: design_charge_kg,
            design_charge_kg,
            pressure_pa: CHARGE_PRESSURE_21C_PA,
            last_ambient_c: CHARGE_REF_TEMP_K - 273.15,
            volume_m3,
            discharged: false,
        }
    }

    pub fn agent_mass_kg(&self) -> f64 {
        self.agent_mass_kg
    }
    pub fn pressure_pa(&self) -> f64 {
        self.pressure_pa
    }
    pub fn is_discharged(&self) -> bool {
        self.discharged
    }
    pub fn is_low_pressure(&self) -> bool {
        self.pressure_pa < low_pressure_threshold_pa(self.last_ambient_c)
    }

    pub fn charge_loss_fraction(&self) -> f64 {
        (1.0 - self.agent_mass_kg / self.design_charge_kg.max(1e-6)).clamp(0.0, 1.0)
    }

    const RESIDUAL_LIQUID_FRACTION: f64 = 0.05;

    fn retarget_pressure(&mut self, ambient_c: f64) {
        let charge_fraction = (self.agent_mass_kg / self.design_charge_kg.max(1e-6)).clamp(0.0, 1.0);
        let temp_k = (ambient_c + 273.15).max(1.0);
        let full_pressure = CHARGE_PRESSURE_21C_PA * (temp_k / CHARGE_REF_TEMP_K);
        self.pressure_pa = if charge_fraction > Self::RESIDUAL_LIQUID_FRACTION {
            full_pressure
        } else {
            full_pressure * (charge_fraction / Self::RESIDUAL_LIQUID_FRACTION)
        };
        self.last_ambient_c = ambient_c;
    }

    pub fn step(&mut self, ambient_c: f64, fire_command: bool, zone_pressure_pa: f64, faults: &BottleFaults, dt_s: f64) -> f64 {
        let dt = dt_s.max(0.0);

        let leak_area = LEAK_AREA_MAX_M2 * clamp01(faults.leak);
        let leak_kg_s = orifice_mass_flow_kg_s(LEAK_DISCHARGE_COEFFICIENT, leak_area, self.pressure_pa, (ambient_c + 273.15).max(1.0), 101_325.0);
        self.agent_mass_kg = (self.agent_mass_kg - leak_kg_s * dt).max(0.0);

        if fire_command && !self.discharged && self.agent_mass_kg > 0.0 && clamp01(faults.squib_failure) < 1.0 {
            self.discharged = true;
        }
        let discharge_kg_s = if self.discharged && self.agent_mass_kg > 0.0 {
            let effective_area = DISCHARGE_AREA_MAX_M2 * (1.0 - clamp01(faults.squib_failure));
            let flow = orifice_mass_flow_kg_s(LEAK_DISCHARGE_COEFFICIENT, effective_area, self.pressure_pa, (ambient_c + 273.15).max(1.0), zone_pressure_pa);
            let delivered = flow.min(self.agent_mass_kg / dt.max(1e-6));
            self.agent_mass_kg = (self.agent_mass_kg - delivered * dt).max(0.0);
            delivered
        } else {
            0.0
        };

        self.retarget_pressure(ambient_c);
        discharge_kg_s
    }
}

pub struct ZoneConcentration {
    volume_fraction: f64,
    volume_m3: f64,
}

impl ZoneConcentration {
    pub fn new(volume_m3: f64) -> Self {
        Self { volume_fraction: 0.0, volume_m3 }
    }

    pub fn volume_fraction(&self) -> f64 {
        self.volume_fraction
    }

    pub fn suppression_fraction(&self) -> f64 {
        clamp01(self.volume_fraction / DESIGN_CONCENTRATION_VOLUME_FRACTION)
    }

    pub fn step(&mut self, agent_inflow_kg_s: f64, ambient_c: f64, ambient_pressure_pa: f64, ventilation_m3_s: f64, dt_s: f64) {
        let dt = dt_s.max(0.0);
        let temp_k = (ambient_c + 273.15).max(1.0);
        let total_moles = ambient_pressure_pa.max(1000.0) * self.volume_m3 / (R_UNIVERSAL * temp_k);
        let agent_moles = self.volume_fraction * total_moles;
        let inflow_moles_s = agent_inflow_kg_s.max(0.0) / HALON_MOLAR_MASS_KG_MOL;
        let washout_moles_s = (ventilation_m3_s.max(0.0) / self.volume_m3.max(1e-6)) * agent_moles;
        let new_moles = (agent_moles + (inflow_moles_s - washout_moles_s) * dt).max(0.0);
        self.volume_fraction = clamp01(new_moles / total_moles.max(1e-6));
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub struct CrossFeed {
    pub valve_open: bool,
}

impl CrossFeed {
    pub fn routing(&self, own_zone_bottles_available: bool, other_zone_bottles_available: bool) -> (bool, bool) {
        if own_zone_bottles_available {
            (true, false)
        } else {
            (false, self.valve_open && other_zone_bottles_available)
        }
    }
}

const SMOKE_SPECIFIC_EXTINCTION_M2_KG: f64 = 8700.0;
const SOOT_YIELD_KG_PER_KG_FUEL: f64 = 0.05;
const ALARM_OBSCURATION_PER_M: f64 = 0.02 / 0.3048;

#[derive(Clone, Copy, Debug, Default)]
pub struct SmokeDetectorFaults {
    pub lens_obscured: f64,
}

pub struct OpticalSmokeDetector {
    path_length_m: f64,
    smoke_density_kg_m3: f64,
    volume_m3: f64,
}

impl OpticalSmokeDetector {
    pub fn new(path_length_m: f64, volume_m3: f64) -> Self {
        Self { path_length_m, smoke_density_kg_m3: 0.0, volume_m3 }
    }

    pub fn smoke_density_kg_m3(&self) -> f64 {
        self.smoke_density_kg_m3
    }

    pub fn step(&mut self, burn_rate_kg_s: f64, ventilation_m3_s: f64, faults: &SmokeDetectorFaults, dt_s: f64) -> bool {
        self.step_with_ambient(burn_rate_kg_s, ventilation_m3_s, 0.0, faults, dt_s)
    }

    pub fn step_with_ambient(&mut self, burn_rate_kg_s: f64, ventilation_m3_s: f64, ambient_smoke_kg_m3: f64, faults: &SmokeDetectorFaults, dt_s: f64) -> bool {
        let dt = dt_s.max(0.0);
        let soot_in_kg_s = burn_rate_kg_s.max(0.0) * SOOT_YIELD_KG_PER_KG_FUEL;
        let washout_per_s = ventilation_m3_s.max(0.0) / self.volume_m3.max(1e-6);
        let mass_kg = self.smoke_density_kg_m3 * self.volume_m3;
        let new_mass = (mass_kg + (soot_in_kg_s - washout_per_s * mass_kg) * dt).max(0.0);
        self.smoke_density_kg_m3 = new_mass / self.volume_m3.max(1e-6);

        let effective_density = (self.smoke_density_kg_m3 + ambient_smoke_kg_m3.max(0.0)) * (1.0 - clamp01(faults.lens_obscured));
        let total_obscuration = 1.0 - (-SMOKE_SPECIFIC_EXTINCTION_M2_KG * effective_density * self.path_length_m).exp();
        let alarm_threshold_total = 1.0 - (1.0 - ALARM_OBSCURATION_PER_M).powf(self.path_length_m);
        total_obscuration >= alarm_threshold_total
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub struct CargoSuppressionFaults {
    pub leak: f64,
    pub knockdown_squib_fault: f64,
    pub extended_squib_fault: f64,
    pub distribution_fault: f64,
}

pub struct CargoSuppressionSystem {
    pub bottle: Bottle,
    knockdown_area_m2: f64,
    metered_area_m2: f64,
    knocked_down: bool,
}

impl CargoSuppressionSystem {
    const METERED_AREA_FRACTION: f64 = 0.01;

    pub fn new(design_charge_kg: f64, volume_m3: f64) -> Self {
        Self { bottle: Bottle::new(design_charge_kg, volume_m3), knockdown_area_m2: DISCHARGE_AREA_MAX_M2, metered_area_m2: DISCHARGE_AREA_MAX_M2 * Self::METERED_AREA_FRACTION, knocked_down: false }
    }

    pub fn step(&mut self, ambient_c: f64, fire_command: bool, zone_pressure_pa: f64, zone_concentration: &ZoneConcentration, faults: &CargoSuppressionFaults, dt_s: f64) -> f64 {
        if fire_command && zone_concentration.suppression_fraction() >= 1.0 {
            self.knocked_down = true;
        }
        let (area, stage_squib_fault) = if self.knocked_down { (self.metered_area_m2, faults.extended_squib_fault) } else { (self.knockdown_area_m2, faults.knockdown_squib_fault) };
        let effective_area = area * (1.0 - clamp01(stage_squib_fault)) * (1.0 - clamp01(faults.distribution_fault));
        let dt = dt_s.max(0.0);
        let leak_area = LEAK_AREA_MAX_M2 * clamp01(faults.leak);
        let leak_kg_s = orifice_mass_flow_kg_s(LEAK_DISCHARGE_COEFFICIENT, leak_area, self.bottle.pressure_pa, (ambient_c + 273.15).max(1.0), 101_325.0);
        self.bottle.agent_mass_kg = (self.bottle.agent_mass_kg - leak_kg_s * dt).max(0.0);

        let discharge_kg_s = if fire_command && self.bottle.agent_mass_kg > 0.0 {
            let flow = orifice_mass_flow_kg_s(LEAK_DISCHARGE_COEFFICIENT, effective_area, self.bottle.pressure_pa, (ambient_c + 273.15).max(1.0), zone_pressure_pa);
            let delivered = flow.min(self.bottle.agent_mass_kg / dt.max(1e-6));
            self.bottle.agent_mass_kg = (self.bottle.agent_mass_kg - delivered * dt).max(0.0);
            delivered
        } else {
            0.0
        };
        self.bottle.discharged = self.bottle.agent_mass_kg <= 1e-6;
        self.bottle.retarget_pressure(ambient_c);
        discharge_kg_s
    }

    pub fn is_metering(&self) -> bool {
        self.knocked_down
    }
}

pub const FUSIBLE_LINK_MELT_C: f64 = 77.0;
const FUSIBLE_LINK_DEGRADED_MARGIN_C: f64 = 50.0;

#[derive(Clone, Copy, Debug, Default)]
pub struct LavatoryFaults {
    pub link_degraded: f64,
}

pub struct LavatoryProtection {
    pub smoke_detector: OpticalSmokeDetector,
    link_melted: bool,
}

impl LavatoryProtection {
    pub fn new(volume_m3: f64) -> Self {
        Self { smoke_detector: OpticalSmokeDetector::new(0.3, volume_m3), link_melted: false }
    }

    pub fn is_discharged(&self) -> bool {
        self.link_melted
    }

    #[allow(clippy::too_many_arguments)]
    pub fn step(&mut self, local_temp_c: f64, burn_rate_kg_s: f64, ventilation_m3_s: f64, smoke_faults: &SmokeDetectorFaults, link_faults: &LavatoryFaults, dt_s: f64) -> bool {
        let smoke_alarm = self.smoke_detector.step(burn_rate_kg_s, ventilation_m3_s, smoke_faults, dt_s);
        let melt_c = FUSIBLE_LINK_MELT_C + FUSIBLE_LINK_DEGRADED_MARGIN_C * clamp01(link_faults.link_degraded);
        if local_temp_c >= melt_c {
            self.link_melted = true;
        }
        let _ = smoke_alarm;
        self.link_melted
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn bottle_pressure_rises_with_temperature_and_falls_with_lost_charge() {
        let mut b = Bottle::new(5.0, 0.005);
        b.retarget_pressure(21.0);
        let cold_full = b.pressure_pa();
        b.retarget_pressure(50.0);
        let hot_full = b.pressure_pa();
        assert!(hot_full > cold_full);

        b.agent_mass_kg = 0.1;
        b.retarget_pressure(21.0);
        assert!(b.pressure_pa() < cold_full);
    }

    #[test]
    fn firing_a_healthy_squib_opens_the_valve_which_then_drains_on_its_own_until_empty() {
        let mut b = Bottle::new(5.0, 0.005);
        b.step(20.0, true, 101_325.0, &BottleFaults::default(), 0.1);
        assert!(b.is_discharged(), "the squib firing must immediately show the bottle as discharged (valve open)");
        let mass_just_after_firing = b.agent_mass_kg();
        assert!(mass_just_after_firing < 5.0);

        for _ in 0..200 {
            b.step(20.0, false, 101_325.0, &BottleFaults::default(), 0.1);
        }
        assert!(b.agent_mass_kg() < mass_just_after_firing, "an open valve must keep draining the bottle");
        assert!(b.agent_mass_kg() < 0.05, "should be essentially empty after several seconds at full discharge area, got {} kg", b.agent_mass_kg());
    }

    #[test]
    fn a_fully_failed_squib_prevents_any_discharge() {
        let mut b = Bottle::new(5.0, 0.005);
        let mut delivered_total = 0.0;
        for _ in 0..20 {
            delivered_total += b.step(20.0, true, 101_325.0, &BottleFaults { squib_failure: 1.0, ..Default::default() }, 0.1);
        }
        assert_eq!(delivered_total, 0.0);
        assert!((b.agent_mass_kg() - 5.0).abs() < 1e-6, "no leak fault active, mass should be unchanged");
    }

    #[test]
    fn a_leaking_bottle_loses_mass_and_pressure_over_time_even_unfired() {
        let mut b = Bottle::new(5.0, 0.005);
        for _ in 0..36000 {
            b.step(20.0, false, 101_325.0, &BottleFaults { leak: 1.0, ..Default::default() }, 1.0);
        }
        assert!(b.agent_mass_kg() < 5.0, "a full-severity leak must lose mass over hours");
        assert!(b.is_low_pressure() || b.agent_mass_kg() < 4.0, "should show meaningfully depleted after 10h leaking");
    }

    #[test]
    fn a_leaky_bottle_delivers_less_punch_when_actually_needed() {
        let mut healthy = Bottle::new(5.0, 0.005);
        let mut leaky = Bottle::new(5.0, 0.005);
        for _ in 0..33_500 {
            leaky.step(20.0, false, 101_325.0, &BottleFaults { leak: 1.0, ..Default::default() }, 1.0);
        }
        assert!(leaky.agent_mass_kg() < 5.0 * Bottle::RESIDUAL_LIQUID_FRACTION, "setup: leak must have driven the bottle below the residual-liquid fraction, got {} kg", leaky.agent_mass_kg());

        let mut healthy_delivered = 0.0;
        let mut leaky_delivered = 0.0;
        for _ in 0..20 {
            healthy_delivered += healthy.step(20.0, true, 101_325.0, &BottleFaults::default(), 0.1);
            leaky_delivered += leaky.step(20.0, true, 101_325.0, &BottleFaults::default(), 0.1);
        }
        assert!(leaky_delivered < healthy_delivered, "a bottle weakened by a slow leak must deliver less agent when fired: leaky {leaky_delivered} vs healthy {healthy_delivered}");
    }

    #[test]
    fn zone_concentration_rises_with_discharge_and_decays_with_ventilation() {
        let mut zone = ZoneConcentration::new(20.0);
        for _ in 0..50 {
            zone.step(0.05, 20.0, 101_325.0, 0.0, 0.1);
        }
        let peak = zone.volume_fraction();
        assert!(peak > 0.0);
        for _ in 0..500 {
            zone.step(0.0, 20.0, 101_325.0, 2.0, 0.1);
        }
        assert!(zone.volume_fraction() < peak, "ventilation must wash the agent back out");
    }

    #[test]
    fn suppression_fraction_saturates_at_one_once_design_concentration_is_reached() {
        let mut zone = ZoneConcentration::new(5.0);
        for _ in 0..2000 {
            zone.step(0.05, 20.0, 101_325.0, 0.0, 0.1);
        }
        assert_eq!(zone.suppression_fraction(), 1.0);
    }

    #[test]
    fn cross_feed_routes_to_the_neighbour_only_when_own_bottles_are_gone_and_valve_open() {
        let cf_open = CrossFeed { valve_open: true };
        let cf_closed = CrossFeed { valve_open: false };
        assert_eq!(cf_open.routing(true, true), (true, false), "own bottles available: use them, no need for cross-feed");
        assert_eq!(cf_open.routing(false, true), (false, true));
        assert_eq!(cf_closed.routing(false, true), (false, false), "valve closed: no cross-feed even if the neighbour has agent");
    }

    #[test]
    fn optical_smoke_detector_alarms_once_soot_accumulates_and_not_at_rest() {
        let mut d = OpticalSmokeDetector::new(1.0, 10.0);
        assert!(!d.step(0.0, 0.0, &SmokeDetectorFaults::default(), 1.0), "no fire, no smoke");
        let mut alarmed = false;
        for _ in 0..600 {
            if d.step(0.01, 0.05, &SmokeDetectorFaults::default(), 1.0) {
                alarmed = true;
                break;
            }
        }
        assert!(alarmed, "sustained burning must eventually alarm the smoke detector");
    }

    #[test]
    fn a_fully_obscured_lens_never_alarms_no_matter_how_much_smoke() {
        let mut d = OpticalSmokeDetector::new(1.0, 10.0);
        let faults = SmokeDetectorFaults { lens_obscured: 1.0 };
        let mut alarmed = false;
        for _ in 0..2000 {
            if d.step(0.02, 0.02, &faults, 1.0) {
                alarmed = true;
                break;
            }
        }
        assert!(!alarmed, "a fully obscured detector must never alarm regardless of real smoke present");
        assert!(d.smoke_density_kg_m3() > 0.0, "smoke must still genuinely be accumulating -- the fault blinds the detector, not the fire");
    }

    #[test]
    fn cargo_suppression_switches_from_knockdown_to_metered_once_design_concentration_is_reached() {
        let mut system = CargoSuppressionSystem::new(10.0, 30.0);
        let mut zone = ZoneConcentration::new(30.0);
        for _ in 0..600 {
            let delivered = system.step(20.0, true, 101_325.0, &zone, &CargoSuppressionFaults::default(), 0.1);
            zone.step(delivered, 20.0, 101_325.0, 0.05, 0.1);
            if system.is_metering() {
                break;
            }
        }
        assert!(system.is_metering(), "must reach design concentration and switch to metered discharge within the test window");
        let mass_at_switch = system.bottle.agent_mass_kg();
        assert!(mass_at_switch > 0.0, "must still have agent left for the extended metered phase");
    }

    #[test]
    fn knockdown_and_extended_squib_faults_are_independent_stages() {
        let mut zone = ZoneConcentration::new(30.0);
        let mut system = CargoSuppressionSystem::new(10.0, 30.0);
        let faults = CargoSuppressionFaults { knockdown_squib_fault: 1.0, ..Default::default() };
        let mut total = 0.0;
        for _ in 0..200 {
            let delivered = system.step(20.0, true, 101_325.0, &zone, &faults, 0.1);
            zone.step(delivered, 20.0, 101_325.0, 0.05, 0.1);
            total += delivered;
        }
        assert_eq!(total, 0.0, "a fully failed knockdown squib must deliver nothing in the knockdown stage");
        assert!(!system.is_metering());

        let mut zone2 = ZoneConcentration::new(30.0);
        let mut system2 = CargoSuppressionSystem::new(10.0, 30.0);
        let extended_fault = CargoSuppressionFaults { extended_squib_fault: 1.0, ..Default::default() };
        for _ in 0..600 {
            let delivered = system2.step(20.0, true, 101_325.0, &zone2, &extended_fault, 0.1);
            zone2.step(delivered, 20.0, 101_325.0, 0.05, 0.1);
            if system2.is_metering() {
                break;
            }
        }
        assert!(system2.is_metering(), "the knockdown stage must be unaffected by the extended-stage fault");
        let delivered_while_metering = system2.step(20.0, true, 101_325.0, &zone2, &extended_fault, 0.1);
        assert_eq!(delivered_while_metering, 0.0, "a fully failed extended squib must deliver nothing once metering");
    }

    #[test]
    fn a_distribution_fault_reduces_delivered_agent_with_a_healthy_squib() {
        let zone = ZoneConcentration::new(30.0);
        let mut healthy = CargoSuppressionSystem::new(10.0, 30.0);
        let mut faulted = CargoSuppressionSystem::new(10.0, 30.0);
        let healthy_delivered = healthy.step(20.0, true, 101_325.0, &zone, &CargoSuppressionFaults::default(), 0.1);
        let faulted_delivered = faulted.step(20.0, true, 101_325.0, &zone, &CargoSuppressionFaults { distribution_fault: 1.0, ..Default::default() }, 0.1);
        assert!(healthy_delivered > 0.0);
        assert_eq!(faulted_delivered, 0.0, "a fully failed distribution path must deliver nothing even with a healthy bottle/squib");
    }

    #[test]
    fn lavatory_fusible_link_melts_once_and_stays_melted() {
        let mut lav = LavatoryProtection::new(2.0);
        assert!(!lav.step(20.0, 0.0, 0.01, &SmokeDetectorFaults::default(), &LavatoryFaults::default(), 1.0));
        assert!(lav.step(FUSIBLE_LINK_MELT_C + 5.0, 0.0, 0.01, &SmokeDetectorFaults::default(), &LavatoryFaults::default(), 1.0));
        assert!(lav.is_discharged());
        assert!(lav.step(20.0, 0.0, 0.01, &SmokeDetectorFaults::default(), &LavatoryFaults::default(), 1.0));
    }

    #[test]
    fn a_degraded_fusible_link_delays_discharge_past_the_design_temperature() {
        let mut lav = LavatoryProtection::new(2.0);
        let faults = LavatoryFaults { link_degraded: 1.0 };
        assert!(!lav.step(FUSIBLE_LINK_MELT_C + 5.0, 0.0, 0.01, &SmokeDetectorFaults::default(), &faults, 1.0));
        assert!(lav.step(FUSIBLE_LINK_MELT_C + FUSIBLE_LINK_DEGRADED_MARGIN_C + 5.0, 0.0, 0.01, &SmokeDetectorFaults::default(), &faults, 1.0));
        assert!(lav.is_discharged());
    }

    #[test]
    fn no_nan_at_rest() {
        let mut b = Bottle::new(5.0, 0.005);
        let out = b.step(0.0, false, 101_325.0, &BottleFaults::default(), 0.0);
        assert!(!out.is_nan());
        assert!(!b.pressure_pa().is_nan());
        let mut zone = ZoneConcentration::new(10.0);
        zone.step(0.0, 0.0, 101_325.0, 0.0, 0.0);
        assert!(!zone.volume_fraction().is_nan());
    }
}
