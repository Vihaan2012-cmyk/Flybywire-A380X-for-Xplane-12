use super::cylinder::{CylinderFaults, CylinderOutputs, CylinderSpec, HpCylinder};
use super::gas;
use super::regulator::{self, MaskMode, PressureRegulator, RegulatorFaults, RegulatorOutputs};

pub const CREW_MASK_COUNT: usize = 4;

pub const CYLINDER_FREE_AIR_LITERS: f64 = 3260.0;
pub const CYLINDER_COUNT: f64 = 2.0;
pub const CYLINDER_CHARGE_PSI: f64 = 1850.0;
pub const BURST_DISC_PSI: f64 = 2775.0;
pub const REFERENCE_TEMP_K: f64 = 294.15;

pub const LOW_PRESSURE_FRACTION: f64 = 0.25;

pub const VALVE_SLEW_PER_S: f64 = 0.2;

pub const VALVE_FULL_AREA_M2: f64 = 1e-5;

pub const VALVE_AREA_TRAVEL_EXPONENT: f64 = 2.0;

pub const MASK_MINIMUM_SUPPLY_GAUGE_PA: f64 = 0.5 * regulator::DISTRIBUTION_SETPOINT_GAUGE_PA;

pub const DISTRIBUTION_LEAK_FULL_SCALE_M2: f64 = 5e-7;

#[derive(Clone, Copy, Debug)]
pub struct CrewOxygenInputs {
    pub cabin_pressure_pa: f64,
    pub bay_temp_k: f64,
    pub masks_donned: [bool; CREW_MASK_COUNT],
    pub mask_mode: [MaskMode; CREW_MASK_COUNT],
    pub supply_valve_commanded_open: bool,
    pub valve_actuator_powered: bool,
}

impl Default for CrewOxygenInputs {
    fn default() -> Self {
        Self {
            cabin_pressure_pa: 101_325.0,
            bay_temp_k: REFERENCE_TEMP_K,
            masks_donned: [false; CREW_MASK_COUNT],
            mask_mode: [MaskMode::Normal; CREW_MASK_COUNT],
            supply_valve_commanded_open: true,
            valve_actuator_powered: true,
        }
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct CrewOxygenFaults {
    pub cylinder: CylinderFaults,
    pub regulator: RegulatorFaults,
    pub valve_jam: f64,
    pub distribution_leak: f64,
    pub dilution_stuck_ambient: [f64; CREW_MASK_COUNT],
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct CrewOxygenOutputs {
    pub cylinder: CylinderOutputs,
    pub regulator: RegulatorOutputs,
    pub valve_position: f64,
    pub distribution_gauge_pa: f64,
    pub delivered_o2_fraction: [f64; CREW_MASK_COUNT],
    pub mask_flow_kg_s: [f64; CREW_MASK_COUNT],
    pub total_mask_flow_kg_s: f64,
    pub distribution_leak_kg_s: f64,
    pub supply_available: bool,
    pub low_pressure: bool,
    pub endurance_s: f64,
}

#[derive(Clone, Debug)]
pub struct CrewOxygenSystem {
    cylinder: HpCylinder,
    reducer: PressureRegulator,
    valve_position: f64,
    low_pressure_threshold_pa: f64,
}

impl Default for CrewOxygenSystem {
    fn default() -> Self {
        Self::new()
    }
}

impl CrewOxygenSystem {
    pub fn new() -> Self {
        let spec = CylinderSpec {
            count: CYLINDER_COUNT,
            free_air_liters: CYLINDER_FREE_AIR_LITERS * CYLINDER_COUNT,
            charge_gauge_pa: CYLINDER_CHARGE_PSI * gas::PSI_TO_PA,
            reference_temp_k: REFERENCE_TEMP_K,
            burst_disc_gauge_pa: BURST_DISC_PSI * gas::PSI_TO_PA,
        };
        Self {
            cylinder: HpCylinder::new(spec),
            reducer: PressureRegulator::new(regulator::DISTRIBUTION_SETPOINT_GAUGE_PA),
            valve_position: 1.0,
            low_pressure_threshold_pa: LOW_PRESSURE_FRACTION * CYLINDER_CHARGE_PSI * gas::PSI_TO_PA,
        }
    }

    pub fn cylinder(&self) -> &HpCylinder {
        &self.cylinder
    }

    pub fn cylinder_mut(&mut self) -> &mut HpCylinder {
        &mut self.cylinder
    }

    pub fn valve_position(&self) -> f64 {
        self.valve_position
    }

    pub fn service(&mut self) {
        self.cylinder.service();
        self.valve_position = 1.0;
    }

    pub fn step(&mut self, inputs: CrewOxygenInputs, faults: CrewOxygenFaults, dt_s: f64) -> CrewOxygenOutputs {
        let dt = dt_s.max(0.0);
        let cabin = inputs.cabin_pressure_pa.max(1.0);

        let jam = faults.valve_jam.clamp(0.0, 1.0);
        let reachable = (1.0 - jam).clamp(0.0, 1.0);
        let commanded: f64 = if inputs.supply_valve_commanded_open { 1.0 } else { 0.0 };
        let target = commanded.min(reachable);
        if inputs.valve_actuator_powered && dt > 0.0 {
            let travel = VALVE_SLEW_PER_S * dt;
            self.valve_position += (target - self.valve_position).clamp(-travel, travel);
        }
        self.valve_position = self.valve_position.clamp(0.0, reachable);

        let mut delivered_o2_fraction = [0.0; CREW_MASK_COUNT];
        let mut mask_flow_kg_s = [0.0; CREW_MASK_COUNT];
        let mut demand = 0.0;
        for i in 0..CREW_MASK_COUNT {
            let mode = inputs.mask_mode[i];
            let fraction = regulator::delivered_o2_fraction(mode, cabin, faults.dilution_stuck_ambient[i]);
            delivered_o2_fraction[i] = fraction;
            if inputs.masks_donned[i] {
                let f = regulator::mask_demand_kg_s(mode, cabin, fraction, regulator::RESTING_MINUTE_VOLUME_L_PER_MIN);
                mask_flow_kg_s[i] = f;
                demand += f;
            }
        }

        let cylinder_gauge = (self.cylinder.absolute_pressure_pa() - cabin).max(0.0);
        let reduced = self.reducer.step(cylinder_gauge, demand, faults.regulator);
        let distribution_abs = reduced.outlet_gauge_pa + cabin;

        let leak_area = faults.distribution_leak.clamp(0.0, 1.0) * DISTRIBUTION_LEAK_FULL_SCALE_M2;
        let distribution_leak = gas::orifice_mass_flow_kg_s(leak_area, distribution_abs, inputs.bay_temp_k.max(1.0), cabin);

        let seat_area = faults.regulator.seat_leak.clamp(0.0, 1.0) * regulator::SEAT_LEAK_FULL_SCALE_M2;
        let seat_leak = gas::orifice_mass_flow_kg_s(seat_area, self.cylinder.absolute_pressure_pa(), self.cylinder.gas_temp_k(), distribution_abs);

        let valve_area = VALVE_FULL_AREA_M2 * self.valve_position.clamp(0.0, 1.0).powf(VALVE_AREA_TRAVEL_EXPONENT);
        let capacity = gas::orifice_mass_flow_kg_s(valve_area, self.cylinder.absolute_pressure_pa(), self.cylinder.gas_temp_k(), distribution_abs);
        let wanted = demand + distribution_leak + seat_leak;
        let sag = if wanted > 0.0 { (capacity / wanted).clamp(0.0, 1.0) } else { 1.0 };
        let starved = capacity < wanted;
        let distribution_gauge_pa = if self.valve_position <= 0.0 { 0.0 } else { reduced.outlet_gauge_pa * sag };
        let drawn = if starved { capacity } else { wanted };
        let scale = if wanted > 0.0 { drawn / wanted } else { 0.0 };

        let supply_available = distribution_gauge_pa >= MASK_MINIMUM_SUPPLY_GAUGE_PA;
        let mask_scale = if supply_available { scale } else { 0.0 };
        let mut actual_leak = (distribution_leak + seat_leak) * scale;
        let mut actual_mask_total = demand * mask_scale;
        for f in mask_flow_kg_s.iter_mut() {
            *f *= mask_scale;
        }
        if !actual_leak.is_finite() {
            actual_leak = 0.0;
        }
        if !actual_mask_total.is_finite() {
            actual_mask_total = 0.0;
        }

        let cylinder = self.cylinder.step(actual_mask_total + actual_leak, cabin, inputs.bay_temp_k, faults.cylinder, dt);

        let low_pressure = cylinder.corrected_gauge_pressure_pa < self.low_pressure_threshold_pa;
        let total_out = cylinder.delivered_kg_s + cylinder.leak_kg_s + cylinder.discharge_kg_s;
        let endurance_s = if total_out > 1e-12 { cylinder.mass_kg / total_out } else { f64::MAX };

        CrewOxygenOutputs {
            cylinder,
            regulator: reduced,
            valve_position: self.valve_position,
            distribution_gauge_pa,
            delivered_o2_fraction,
            mask_flow_kg_s,
            total_mask_flow_kg_s: actual_mask_total,
            distribution_leak_kg_s: actual_leak,
            supply_available,
            low_pressure,
            endurance_s,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn donned_at(cabin_pressure_pa: f64) -> CrewOxygenInputs {
        CrewOxygenInputs { cabin_pressure_pa, masks_donned: [true; CREW_MASK_COUNT], ..Default::default() }
    }

    fn run(sys: &mut CrewOxygenSystem, inputs: CrewOxygenInputs, faults: CrewOxygenFaults, seconds: usize) -> CrewOxygenOutputs {
        let mut out = sys.step(inputs, faults, 0.0);
        for _ in 0..seconds {
            out = sys.step(inputs, faults, 1.0);
        }
        out
    }

    #[test]
    fn a_full_system_reads_its_charge_and_has_pressure_at_the_masks() {
        let mut sys = CrewOxygenSystem::new();
        let out = sys.step(CrewOxygenInputs::default(), CrewOxygenFaults::default(), 0.1);
        let psi = out.cylinder.gauge_pressure_pa / gas::PSI_TO_PA;
        assert!((psi - CYLINDER_CHARGE_PSI).abs() < 2.0, "{psi} psi");
        assert!(out.supply_available);
        assert!(!out.low_pressure);
        assert!((out.distribution_gauge_pa - regulator::DISTRIBUTION_SETPOINT_GAUGE_PA).abs() < 1.0);
    }

    #[test]
    fn masks_donned_on_the_ground_consume_nothing_at_all() {
        let mut sys = CrewOxygenSystem::new();
        let out = run(&mut sys, donned_at(101_325.0), CrewOxygenFaults::default(), 3600);
        assert_eq!(out.total_mask_flow_kg_s, 0.0);
        assert!((out.cylinder.quantity_fraction - 1.0).abs() < 1e-12);
        assert!((out.delivered_o2_fraction[0] - gas::AIR_O2_MOLE_FRACTION).abs() < 1e-9);
    }

    #[test]
    fn indicated_pressure_falls_with_use_at_altitude() {
        let mut sys = CrewOxygenSystem::new();
        let inputs = donned_at(23_842.0);
        let start = sys.step(inputs, CrewOxygenFaults::default(), 0.0);
        let after = run(&mut sys, inputs, CrewOxygenFaults::default(), 7200);
        assert!(after.total_mask_flow_kg_s > 0.0);
        assert!(after.cylinder.gauge_pressure_pa < start.cylinder.gauge_pressure_pa);
        assert!(after.cylinder.corrected_gauge_pressure_pa < start.cylinder.corrected_gauge_pressure_pa);
        assert!(after.cylinder.quantity_fraction < 0.95 && after.cylinder.quantity_fraction > 0.75, "{}", after.cylinder.quantity_fraction);
        assert!(after.endurance_s > 3600.0 && after.endurance_s < 40.0 * 3600.0, "{} s", after.endurance_s);
    }

    #[test]
    fn indicated_pressure_falls_with_temperature_and_the_caution_does_not() {
        let mut warm = CrewOxygenSystem::new();
        let mut cold = CrewOxygenSystem::new();
        let warm_in = CrewOxygenInputs { bay_temp_k: 294.15, ..Default::default() };
        let cold_in = CrewOxygenInputs { bay_temp_k: 233.15, ..Default::default() };
        let w = run(&mut warm, warm_in, CrewOxygenFaults::default(), 8 * 3600);
        let c = run(&mut cold, cold_in, CrewOxygenFaults::default(), 8 * 3600);
        assert!(c.cylinder.gauge_pressure_pa < w.cylinder.gauge_pressure_pa - 200.0 * gas::PSI_TO_PA, "cold {} warm {}", c.cylinder.gauge_pressure_pa, w.cylinder.gauge_pressure_pa);
        assert!(!c.low_pressure && !w.low_pressure);
        assert!((c.cylinder.corrected_gauge_pressure_pa - w.cylinder.corrected_gauge_pressure_pa).abs() < gas::PSI_TO_PA);
    }

    #[test]
    fn the_low_pressure_caution_comes_up_at_a_quarter_charge() {
        let mut sys = CrewOxygenSystem::new();
        let full = sys.cylinder().full_mass_kg();
        sys.cylinder_mut().set_mass_kg(full * 0.30);
        let ok = sys.step(CrewOxygenInputs::default(), CrewOxygenFaults::default(), 0.1);
        assert!(!ok.low_pressure, "{} psi", ok.cylinder.corrected_gauge_pressure_pa / gas::PSI_TO_PA);
        sys.cylinder_mut().set_mass_kg(full * 0.15);
        let low = sys.step(CrewOxygenInputs::default(), CrewOxygenFaults::default(), 0.1);
        assert!(low.low_pressure, "{} psi", low.cylinder.corrected_gauge_pressure_pa / gas::PSI_TO_PA);
    }

    #[test]
    fn a_seized_shutoff_valve_starves_the_masks() {
        let mut sys = CrewOxygenSystem::new();
        let faults = CrewOxygenFaults { valve_jam: 1.0, ..Default::default() };
        let out = run(&mut sys, donned_at(23_842.0), faults, 30);
        assert_eq!(out.valve_position, 0.0);
        assert_eq!(out.distribution_gauge_pa, 0.0);
        assert!(!out.supply_available);
        assert_eq!(out.total_mask_flow_kg_s, 0.0);
        assert!((out.cylinder.quantity_fraction - 1.0).abs() < 1e-12);
    }

    #[test]
    fn an_unpowered_valve_stays_where_it_is_rather_than_shutting() {
        let mut sys = CrewOxygenSystem::new();
        let inputs = CrewOxygenInputs { valve_actuator_powered: false, supply_valve_commanded_open: false, ..Default::default() };
        let out = run(&mut sys, inputs, CrewOxygenFaults::default(), 60);
        assert_eq!(out.valve_position, 1.0, "a valve with no bus cannot travel");
        assert!(out.supply_available);
    }

    #[test]
    fn a_distribution_leak_drains_the_bottle_with_nobody_wearing_a_mask() {
        let mut sys = CrewOxygenSystem::new();
        let faults = CrewOxygenFaults { distribution_leak: 1.0, ..Default::default() };
        let out = run(&mut sys, CrewOxygenInputs::default(), faults, 3600);
        assert!(out.distribution_leak_kg_s > 0.0);
        assert!(out.cylinder.quantity_fraction < 0.95, "{}", out.cylinder.quantity_fraction);
        let long = run(&mut sys, CrewOxygenInputs::default(), faults, 8 * 3600);
        assert!(long.cylinder.quantity_fraction < 0.4, "{}", long.cylinder.quantity_fraction);
    }

    #[test]
    fn a_cylinder_leak_drains_it_and_raises_the_caution() {
        let mut sys = CrewOxygenSystem::new();
        let faults = CrewOxygenFaults { cylinder: CylinderFaults { leak: 0.3, ..Default::default() }, ..Default::default() };
        let out = run(&mut sys, CrewOxygenInputs::default(), faults, 3600);
        assert!(out.cylinder.leak_kg_s > 0.0);
        assert!(out.low_pressure, "a third-of-a-square-millimetre leak for an hour should be well below a quarter charge: {}", out.cylinder.quantity_fraction);
    }

    #[test]
    fn a_leaking_reducer_seat_empties_the_bottle_into_the_low_pressure_relief() {
        let mut sys = CrewOxygenSystem::new();
        let faults = CrewOxygenFaults { regulator: RegulatorFaults { seat_leak: 1.0, ..Default::default() }, ..Default::default() };
        let out = run(&mut sys, CrewOxygenInputs::default(), faults, 3600);
        assert!(out.distribution_leak_kg_s > 0.0, "the seat leak has to come out of the bottle");
        assert!(out.cylinder.quantity_fraction < 0.95, "{}", out.cylinder.quantity_fraction);
        assert_eq!(out.total_mask_flow_kg_s, 0.0);
        assert!(out.endurance_s < 24.0 * 3600.0, "{} s", out.endurance_s);
    }

    #[test]
    fn a_failed_reducer_starves_the_masks_without_touching_the_bottle() {
        let mut sys = CrewOxygenSystem::new();
        let faults = CrewOxygenFaults { regulator: RegulatorFaults { setpoint_shift: -1.0, ..Default::default() }, ..Default::default() };
        let out = run(&mut sys, donned_at(23_842.0), faults, 60);
        assert_eq!(out.distribution_gauge_pa, 0.0);
        assert!(!out.supply_available);
        assert!((out.cylinder.quantity_fraction - 1.0).abs() < 1e-9);
    }

    #[test]
    fn a_stuck_diluter_leaves_the_crew_breathing_cabin_air_at_altitude() {
        let mut sys = CrewOxygenSystem::new();
        let faults = CrewOxygenFaults { dilution_stuck_ambient: [1.0; CREW_MASK_COUNT], ..Default::default() };
        let healthy = run(&mut CrewOxygenSystem::new(), donned_at(23_842.0), CrewOxygenFaults::default(), 60);
        let sick = run(&mut sys, donned_at(23_842.0), faults, 60);
        assert!((healthy.delivered_o2_fraction[0] - 1.0).abs() < 1e-9);
        assert!((sick.delivered_o2_fraction[0] - gas::AIR_O2_MOLE_FRACTION).abs() < 1e-9);
        assert_eq!(sick.total_mask_flow_kg_s, 0.0);
        assert!(healthy.total_mask_flow_kg_s > 0.0);

        let mut one = CrewOxygenSystem::new();
        let mut per_mask = CrewOxygenFaults::default();
        per_mask.dilution_stuck_ambient[0] = 1.0;
        let out = run(&mut one, donned_at(23_842.0), per_mask, 60);
        assert!((out.delivered_o2_fraction[0] - gas::AIR_O2_MOLE_FRACTION).abs() < 1e-9);
        assert!((out.delivered_o2_fraction[1] - 1.0).abs() < 1e-9);
        assert_eq!(out.mask_flow_kg_s[0], 0.0);
        assert!(out.mask_flow_kg_s[1] > 0.0);
    }

    #[test]
    fn a_hundred_percent_selection_costs_far_more_than_normal() {
        let mut normal = CrewOxygenSystem::new();
        let mut pure = CrewOxygenSystem::new();
        let at_altitude = donned_at(57_182.0);
        let pure_in = CrewOxygenInputs { mask_mode: [MaskMode::Pure; CREW_MASK_COUNT], ..at_altitude };
        let n = run(&mut normal, at_altitude, CrewOxygenFaults::default(), 600);
        let p = run(&mut pure, pure_in, CrewOxygenFaults::default(), 600);
        assert!(p.total_mask_flow_kg_s > 2.0 * n.total_mask_flow_kg_s, "{} vs {}", p.total_mask_flow_kg_s, n.total_mask_flow_kg_s);
        assert!(p.cylinder.mass_kg < n.cylinder.mass_kg);
    }

    #[test]
    fn nothing_breaks_on_a_cold_dark_first_frame() {
        let mut sys = CrewOxygenSystem::new();
        let out = sys.step(CrewOxygenInputs::default(), CrewOxygenFaults::default(), 0.0);
        for v in [
            out.cylinder.absolute_pressure_pa,
            out.cylinder.gas_temp_k,
            out.distribution_gauge_pa,
            out.total_mask_flow_kg_s,
            out.endurance_s,
        ] {
            assert!(v.is_finite(), "{v}");
        }
        sys.cylinder_mut().set_mass_kg(0.0);
        let broken = CrewOxygenFaults {
            cylinder: CylinderFaults { leak: 1.0, disc_weakened: 1.0 },
            regulator: RegulatorFaults { setpoint_shift: -1.0, seat_leak: 1.0 },
            valve_jam: 1.0,
            distribution_leak: 1.0,
            dilution_stuck_ambient: [1.0; CREW_MASK_COUNT],
        };
        let out = sys.step(donned_at(10_000.0), broken, 1.0);
        assert!(out.cylinder.absolute_pressure_pa.is_finite() && out.endurance_s.is_finite());
    }
}
