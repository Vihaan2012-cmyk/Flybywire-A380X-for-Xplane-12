use super::fluid;

pub const PSI_PA: f64 = 6894.757;
pub const LITER_M3: f64 = 1.0e-3;
pub const GALLON_M3: f64 = 3.785_411_784e-3;

pub const LOW_PRESSURE_WARN_PSI: f64 = 21.76;
pub const LOW_PRESSURE_CLEAR_PSI: f64 = 25.0;

const NOMINAL_BOOST_PA: f64 = 50.0 * PSI_PA;

#[derive(Clone, Copy, Debug, Default)]
pub struct ReservoirFaults {
    pub leak_area_m2: f64,
    pub pressurization_loss: f64,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct ReservoirOutputs {
    pub fluid_volume_m3: f64,
    pub fill_fraction: f64,
    pub inlet_air_pressure_pa: f64,
    pub low_pressure_warning: bool,
    pub low_level_warning: bool,
    pub leaked_m3_s: f64,
}

#[derive(Clone, Debug)]
pub struct Reservoir {
    capacity_m3: f64,
    unusable_m3: f64,
    low_level_warn_m3: f64,
    fluid_volume_m3: f64,
    low_pressure_switch_on: bool,
}

impl Reservoir {
    pub fn new(capacity_m3: f64, unusable_m3: f64, low_level_warn_m3: f64, initial_fluid_m3: f64) -> Self {
        Self {
            capacity_m3: capacity_m3.max(1e-6),
            unusable_m3: unusable_m3.max(0.0),
            low_level_warn_m3: low_level_warn_m3.max(0.0),
            fluid_volume_m3: initial_fluid_m3.clamp(0.0, capacity_m3),
            low_pressure_switch_on: false,
        }
    }

    const MIN_USABLE_GAL: f64 = 0.2;

    pub fn a380_green() -> Self {
        Self::new(GALLON_M3 * 12.0, GALLON_M3 * Self::MIN_USABLE_GAL, LITER_M3 * 5.0, GALLON_M3 * 12.0)
    }
    pub fn a380_yellow() -> Self {
        Self::new(GALLON_M3 * 12.7, GALLON_M3 * Self::MIN_USABLE_GAL, LITER_M3 * 5.0, GALLON_M3 * 12.7)
    }

    pub fn fluid_volume_m3(&self) -> f64 {
        self.fluid_volume_m3
    }

    pub fn step(&mut self, inflow_m3_s: f64, outflow_m3_s: f64, pressurization_supply_fraction: f64, faults: &ReservoirFaults, dt_s: f64) -> ReservoirOutputs {
        let dt = dt_s.max(0.0);

        let density = fluid::density_kg_m3(60.0);
        let leak_area = faults.leak_area_m2.max(0.0);
        let leak_dp = self.regulated_boost_pa(pressurization_supply_fraction, faults).max(0.0);
        let raw_leaked_m3_s = if leak_area > 0.0 && leak_dp > 0.0 { 0.61 * leak_area * (2.0 * leak_dp / density).sqrt() } else { 0.0 };
        let available_m3_s = if dt > 0.0 { self.fluid_volume_m3 / dt + inflow_m3_s - outflow_m3_s } else { f64::INFINITY };
        let leaked_m3_s = raw_leaked_m3_s.clamp(0.0, available_m3_s.max(0.0));

        let net = inflow_m3_s - outflow_m3_s - leaked_m3_s;
        self.fluid_volume_m3 = (self.fluid_volume_m3 + net * dt).clamp(0.0, self.capacity_m3);

        let inlet_air_pressure_pa = self.boost_pa(pressurization_supply_fraction, faults);
        let low_psi = inlet_air_pressure_pa / PSI_PA;
        if low_psi <= LOW_PRESSURE_WARN_PSI {
            self.low_pressure_switch_on = true;
        } else if low_psi >= LOW_PRESSURE_CLEAR_PSI {
            self.low_pressure_switch_on = false;
        }

        ReservoirOutputs {
            fluid_volume_m3: self.fluid_volume_m3,
            fill_fraction: self.fluid_volume_m3 / self.capacity_m3,
            inlet_air_pressure_pa,
            low_pressure_warning: self.low_pressure_switch_on,
            low_level_warning: self.fluid_volume_m3 < self.low_level_warn_m3,
            leaked_m3_s,
        }
    }

    fn regulated_boost_pa(&self, pressurization_supply_fraction: f64, faults: &ReservoirFaults) -> f64 {
        let pressurization_fraction = pressurization_supply_fraction.clamp(0.0, 1.0) * (1.0 - faults.pressurization_loss.clamp(0.0, 1.0));
        NOMINAL_BOOST_PA * pressurization_fraction
    }

    fn boost_pa(&self, pressurization_supply_fraction: f64, faults: &ReservoirFaults) -> f64 {
        let margin_m3 = (self.capacity_m3 * 0.1).max(1e-6);
        let level_factor = ((self.fluid_volume_m3 - self.unusable_m3) / margin_m3).clamp(0.0, 1.0);
        self.regulated_boost_pa(pressurization_supply_fraction, faults) * level_factor
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn healthy_full_reservoir_with_pressurization_supplied_shows_full_boost() {
        let mut res = Reservoir::a380_green();
        let out = res.step(0.0, 0.0, 1.0, &ReservoirFaults::default(), 0.02);
        assert!((out.inlet_air_pressure_pa - NOMINAL_BOOST_PA).abs() < 1.0);
        assert!(!out.low_pressure_warning);
    }

    #[test]
    fn draining_the_reservoir_below_unusable_collapses_inlet_pressure() {
        let unusable = GALLON_M3 * Reservoir::MIN_USABLE_GAL;
        let mut res = Reservoir::new(GALLON_M3 * 12.0, unusable, LITER_M3 * 5.0, unusable);
        let out = res.step(0.0, 0.0, 1.0, &ReservoirFaults::default(), 0.02);
        assert!(out.inlet_air_pressure_pa < 1.0, "at the unusable line there should be ~no usable head: {}", out.inlet_air_pressure_pa);
        assert!(out.low_level_warning, "well below the 5 L warning threshold too");
    }

    #[test]
    fn losing_pressurization_drops_boost_even_when_full() {
        let mut res = Reservoir::a380_green();
        let faults = ReservoirFaults { pressurization_loss: 1.0, ..Default::default() };
        let out = res.step(0.0, 0.0, 1.0, &faults, 0.02);
        assert_eq!(out.inlet_air_pressure_pa, 0.0);
        assert!(out.low_pressure_warning);
    }

    #[test]
    fn no_pressurization_supply_also_drops_boost_even_with_no_fault() {
        let mut res = Reservoir::a380_green();
        let out = res.step(0.0, 0.0, 0.0, &ReservoirFaults::default(), 0.02);
        assert_eq!(out.inlet_air_pressure_pa, 0.0);
    }

    #[test]
    fn low_pressure_switch_has_hysteresis() {
        let mut res = Reservoir::a380_green();
        let low_faults = ReservoirFaults { pressurization_loss: 1.0, ..Default::default() };
        let out = res.step(0.0, 0.0, 1.0, &low_faults, 0.02);
        assert!(out.low_pressure_warning);
        let mid_psi = (LOW_PRESSURE_WARN_PSI + LOW_PRESSURE_CLEAR_PSI) / 2.0;
        let needed_fraction = mid_psi * PSI_PA / NOMINAL_BOOST_PA;
        let out2 = res.step(0.0, 0.0, needed_fraction, &ReservoirFaults::default(), 0.02);
        assert!(out2.low_pressure_warning, "must stay latched between clear and warn thresholds");
    }

    #[test]
    fn mass_balance_inflow_minus_outflow() {
        let mut res = Reservoir::a380_green();
        let start = res.fluid_volume_m3();
        for _ in 0..100 {
            res.step(0.0, 1.0e-5, 1.0, &ReservoirFaults::default(), 0.02);
        }
        assert!(res.fluid_volume_m3() < start);
        let drained = start - res.fluid_volume_m3();
        assert!((drained - 1.0e-5 * 100.0 * 0.02).abs() / drained < 0.05);
    }

    #[test]
    fn a_leak_drains_the_reservoir_even_with_balanced_flows() {
        let mut healthy = Reservoir::a380_green();
        let mut leaking = Reservoir::a380_green();
        let leak_faults = ReservoirFaults { leak_area_m2: 5e-6, ..Default::default() };
        for _ in 0..500 {
            healthy.step(1.0e-5, 1.0e-5, 1.0, &ReservoirFaults::default(), 0.02);
            leaking.step(1.0e-5, 1.0e-5, 1.0, &leak_faults, 0.02);
        }
        assert!(leaking.fluid_volume_m3() < healthy.fluid_volume_m3());
    }

    #[test]
    fn never_overfills_or_goes_negative() {
        let mut res = Reservoir::a380_green();
        for _ in 0..1000 {
            res.step(1.0, 0.0, 1.0, &ReservoirFaults::default(), 0.1);
        }
        assert!(res.fluid_volume_m3() <= GALLON_M3 * 12.0 + 1e-9);
        for _ in 0..1000 {
            res.step(0.0, 1.0, 1.0, &ReservoirFaults::default(), 0.1);
        }
        assert!(res.fluid_volume_m3() >= 0.0);
    }

    #[test]
    fn no_nan_at_dt_zero() {
        let mut res = Reservoir::a380_green();
        let out = res.step(0.0, 0.0, 0.0, &ReservoirFaults::default(), 0.0);
        assert!(out.inlet_air_pressure_pa.is_finite());
        assert!(out.fill_fraction.is_finite());
    }
}
