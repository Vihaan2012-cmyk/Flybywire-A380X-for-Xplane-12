//! First-aid (therapeutic) oxygen: the installed gaseous cylinder the
//! cabin crew plug a mask into for a sick passenger.
//!
//! This is the *other* real gaseous bottle on an A380, and it is what this
//! area attaches its second pressure transducer to. `deep::sensors`
//! registers two ATA 35 bottle-pressure transducers and labels the second
//! one "Passenger [GENERIC: assumes gaseous supply, not chemical
//! generators]", with a note that the A380's passenger architecture was
//! not verified there. It is verified here: the A380's passenger supply is
//! chemical ([`super::pax`]) and has no pressure anywhere in it for a
//! transducer to read. The second transducer therefore belongs on this
//! cylinder, which is a real installed bottle with a real gauge, and
//! `OXYGEN_BOTTLE_PRESSURE_PA:2` is published from here.
//!
//! ## Why this one is sized by regulation
//!
//! Unlike the crew cylinder, whose capacity had to be taken from a
//! published cylinder spec, the first-aid supply has a *requirement*:
//! 14 CFR 121.333(e) (and the equivalent operational rules elsewhere)
//! demand enough oxygen for at least two percent of the passengers,
//! minimum one person, for at least one hour, delivered at not less than
//! four litres a minute. That fixes the capacity arithmetically from the
//! seat count, and the model's own endurance test is the check that the
//! arithmetic still holds.
//!
//! Delivery is continuous flow, not demand: the regulator passes a fixed
//! volumetric rate whatever the wearer is doing, which is why an outlet
//! left open with nobody on it drains the bottle at exactly the rate an
//! outlet in use does.

use super::cylinder::{CylinderFaults, CylinderOutputs, CylinderSpec, HpCylinder};
use super::gas;
use super::regulator::{PressureRegulator, RegulatorFaults, RegulatorOutputs};

/// Fraction of passengers the first-aid supply must cover (121.333(e)).
pub const FIRST_AID_PASSENGER_FRACTION: f64 = 0.02;
/// Minimum duration the supply must last, s (one hour).
pub const FIRST_AID_DURATION_S: f64 = 3600.0;
/// Minimum continuous flow per outlet, litres per minute at the reference
/// state (121.333(e)).
pub const OUTLET_HIGH_FLOW_L_PER_MIN: f64 = 4.0;
/// The low setting on the same continuous-flow regulator, litres per
/// minute. The two-and-four-litre pair is the standard aviation portable
/// oxygen regulator's own settings.
pub const OUTLET_LOW_FLOW_L_PER_MIN: f64 = 2.0;

/// Charge pressure of the installed cylinder, psig: the standard aviation
/// oxygen cylinder charge for portable and therapeutic bottles.
pub const CHARGE_PSI: f64 = 1800.0;
/// Overpressure discharge disc rupture pressure, psig. **GENERIC**: the
/// same 1.5 times charge relationship the crew cylinder's published disc
/// figure has, applied to this cylinder's lower charge.
pub const BURST_DISC_PSI: f64 = 2700.0;
/// Reference temperature, K -- the same 21 C the rest of this area uses.
pub const REFERENCE_TEMP_K: f64 = 294.15;

/// Delivery pressure of the continuous-flow regulator, Pa gauge.
/// **GENERIC**: a therapeutic outlet runs at a low pressure into a soft
/// mask; a few tens of psi is what that class of regulator delivers, and
/// it is well under the crew system's 85 psi distribution.
pub const OUTLET_SETPOINT_GAUGE_PA: f64 = 50.0 * gas::PSI_TO_PA;

/// How many outlets the regulation obliges the supply to cover, from the
/// seat count -- rounded up, and never less than one.
pub fn required_outlet_count(seats: f64) -> f64 {
    (seats * FIRST_AID_PASSENGER_FRACTION).ceil().max(1.0)
}

/// Free-air capacity the regulation obliges the cylinder to *deliver*,
/// litres. Not the same as what it is charged with -- see
/// [`USABLE_CHARGE_MARGIN`].
pub fn required_free_air_liters(seats: f64) -> f64 {
    required_outlet_count(seats) * OUTLET_HIGH_FLOW_L_PER_MIN * FIRST_AID_DURATION_S / 60.0
}

/// How much more than the delivered requirement the cylinder is charged
/// with. **GENERIC**, and derived rather than picked: a continuous-flow
/// regulator stops delivering once its inlet falls to about its own
/// setpoint, so the last few percent of the charge is not usable. Five
/// percent covers that dropout with a little left over, which is what a
/// cylinder sized against a delivery requirement actually has to do. The
/// test below is the check: the bottle must still be flowing at the end of
/// the regulation's hour, not merely have had the right number of litres
/// in it at the start.
pub const USABLE_CHARGE_MARGIN: f64 = 1.05;

/// What the cylinder is actually charged with, litres of free air.
pub fn charged_free_air_liters(seats: f64) -> f64 {
    required_free_air_liters(seats) * USABLE_CHARGE_MARGIN
}

/// What the therapeutic system is being asked to do.
#[derive(Clone, Copy, Debug)]
pub struct TherapeuticInputs {
    pub cabin_pressure_pa: f64,
    pub bay_temp_k: f64,
    /// How many outlets the cabin crew have a mask plugged into.
    pub outlets_in_use: f64,
    /// Whether those outlets are on the high setting.
    pub high_flow: bool,
}

impl Default for TherapeuticInputs {
    fn default() -> Self {
        Self { cabin_pressure_pa: 101_325.0, bay_temp_k: REFERENCE_TEMP_K, outlets_in_use: 0.0, high_flow: true }
    }
}

/// What can be wrong with it.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct TherapeuticFaults {
    pub cylinder: CylinderFaults,
    pub regulator: RegulatorFaults,
    /// Outlets stuck open with nobody on them, as a fraction of the
    /// installed complement. A continuous-flow outlet does not care
    /// whether anyone is breathing through it, so this drains the bottle
    /// at the full delivery rate.
    pub outlets_stuck_open: f64,
}

/// One frame of the therapeutic system.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct TherapeuticOutputs {
    pub cylinder: CylinderOutputs,
    pub regulator: RegulatorOutputs,
    pub outlet_gauge_pa: f64,
    /// Outlets actually flowing, wanted or not.
    pub outlets_flowing: f64,
    pub total_flow_kg_s: f64,
    /// The same flow in the units a cabin crew procedure uses.
    pub total_flow_l_per_min: f64,
    pub supply_available: bool,
    /// How long the remaining charge would last at the present draw, s.
    pub endurance_s: f64,
}

/// The installed first-aid oxygen supply, running.
#[derive(Clone, Debug)]
pub struct TherapeuticOxygenSystem {
    cylinder: HpCylinder,
    regulator: PressureRegulator,
    outlet_count: f64,
    kg_per_liter: f64,
}

impl Default for TherapeuticOxygenSystem {
    fn default() -> Self {
        Self::new(super::pax::TYPICAL_THREE_CLASS_SEATS)
    }
}

impl TherapeuticOxygenSystem {
    pub fn new(seats: f64) -> Self {
        let spec = CylinderSpec {
            count: 1.0,
            free_air_liters: charged_free_air_liters(seats),
            charge_gauge_pa: CHARGE_PSI * gas::PSI_TO_PA,
            reference_temp_k: REFERENCE_TEMP_K,
            burst_disc_gauge_pa: BURST_DISC_PSI * gas::PSI_TO_PA,
        };
        Self {
            cylinder: HpCylinder::new(spec),
            regulator: PressureRegulator::new(OUTLET_SETPOINT_GAUGE_PA),
            outlet_count: required_outlet_count(seats),
            kg_per_liter: gas::mass_from_free_air_kg(1.0, REFERENCE_TEMP_K),
        }
    }

    pub fn cylinder(&self) -> &HpCylinder {
        &self.cylinder
    }

    pub fn cylinder_mut(&mut self) -> &mut HpCylinder {
        &mut self.cylinder
    }

    pub fn outlet_count(&self) -> f64 {
        self.outlet_count
    }

    pub fn service(&mut self) {
        self.cylinder.service();
    }

    pub fn step(&mut self, inputs: TherapeuticInputs, faults: TherapeuticFaults, dt_s: f64) -> TherapeuticOutputs {
        let cabin = inputs.cabin_pressure_pa.max(1.0);
        let per_outlet_l_min = if inputs.high_flow { OUTLET_HIGH_FLOW_L_PER_MIN } else { OUTLET_LOW_FLOW_L_PER_MIN };

        let stuck = faults.outlets_stuck_open.clamp(0.0, 1.0) * self.outlet_count;
        // An outlet cannot both be in use and be stuck open; the greater
        // of the two is how many are flowing.
        let flowing = inputs.outlets_in_use.max(0.0).min(self.outlet_count).max(stuck);
        let wanted = flowing * per_outlet_l_min / 60.0 * self.kg_per_liter;

        let cylinder_gauge = (self.cylinder.absolute_pressure_pa() - cabin).max(0.0);
        let reduced = self.regulator.step(cylinder_gauge, wanted, faults.regulator);
        let outlet_abs = reduced.outlet_gauge_pa + cabin;

        let seat_area = faults.regulator.seat_leak.clamp(0.0, 1.0) * super::regulator::SEAT_LEAK_FULL_SCALE_M2;
        let seat_leak = gas::orifice_mass_flow_kg_s(seat_area, self.cylinder.absolute_pressure_pa(), self.cylinder.gas_temp_k(), outlet_abs);

        // A continuous-flow regulator delivers its rated flow while it has
        // pressure behind it and nothing at all once it has not.
        let supply_available = reduced.outlet_gauge_pa >= 0.5 * OUTLET_SETPOINT_GAUGE_PA;
        let delivered = if supply_available { wanted } else { 0.0 };

        let cylinder = self.cylinder.step(delivered + seat_leak, cabin, inputs.bay_temp_k, faults.cylinder, dt_s);
        let total_out = cylinder.delivered_kg_s + cylinder.leak_kg_s + cylinder.discharge_kg_s;

        TherapeuticOutputs {
            cylinder,
            regulator: reduced,
            outlet_gauge_pa: reduced.outlet_gauge_pa,
            outlets_flowing: if supply_available { flowing } else { 0.0 },
            total_flow_kg_s: delivered,
            total_flow_l_per_min: delivered / self.kg_per_liter.max(1e-12) * 60.0,
            supply_available,
            endurance_s: if total_out > 1e-12 { cylinder.mass_kg / total_out } else { f64::MAX },
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn run(sys: &mut TherapeuticOxygenSystem, inputs: TherapeuticInputs, faults: TherapeuticFaults, seconds: usize) -> TherapeuticOutputs {
        let mut out = sys.step(inputs, faults, 0.0);
        for _ in 0..seconds {
            out = sys.step(inputs, faults, 1.0);
        }
        out
    }

    #[test]
    fn the_cylinder_is_the_size_the_regulation_demands() {
        let sys = TherapeuticOxygenSystem::default();
        // Two percent of 525 seats, rounded up.
        assert_eq!(sys.outlet_count(), 11.0);
        // Eleven outlets for an hour at four litres a minute.
        let liters = required_free_air_liters(super::super::pax::TYPICAL_THREE_CLASS_SEATS);
        assert!((liters - 2640.0).abs() < 1e-9, "{liters} L");
        // And a small aeroplane still gets one outlet's worth.
        assert_eq!(required_outlet_count(20.0), 1.0);
    }

    #[test]
    fn a_full_bottle_lasts_exactly_the_hour_the_regulation_asks_for() {
        // The end-to-end check that the sizing arithmetic and the flow
        // arithmetic agree: the regulation's own worst case run against
        // the real gas model.
        let mut sys = TherapeuticOxygenSystem::default();
        let inputs = TherapeuticInputs { outlets_in_use: sys.outlet_count(), ..Default::default() };
        let out = run(&mut sys, inputs, TherapeuticFaults::default(), 3600);
        assert!(out.supply_available && out.total_flow_kg_s > 0.0, "it has to still be delivering at the end of the hour");
        assert!(out.cylinder.quantity_fraction < 0.10, "{} left after the hour", out.cylinder.quantity_fraction);
        // And not much past it: a bottle that lasted three hours would be
        // several times the size the regulation asks for.
        let later = run(&mut sys, inputs, TherapeuticFaults::default(), 600);
        assert!(!later.supply_available, "the regulator has to drop out once the charge is used: {}", later.cylinder.quantity_fraction);
    }

    #[test]
    fn the_indicated_pressure_falls_with_use_and_with_temperature() {
        let mut sys = TherapeuticOxygenSystem::default();
        let inputs = TherapeuticInputs { outlets_in_use: 2.0, ..Default::default() };
        let start = sys.step(inputs, TherapeuticFaults::default(), 0.0);
        let used = run(&mut sys, inputs, TherapeuticFaults::default(), 1800);
        assert!(used.cylinder.gauge_pressure_pa < start.cylinder.gauge_pressure_pa);

        let mut cold = TherapeuticOxygenSystem::default();
        let cold_out = run(&mut cold, TherapeuticInputs { bay_temp_k: 243.15, ..Default::default() }, TherapeuticFaults::default(), 8 * 3600);
        let mut warm = TherapeuticOxygenSystem::default();
        let warm_out = run(&mut warm, TherapeuticInputs::default(), TherapeuticFaults::default(), 8 * 3600);
        assert!((cold_out.cylinder.mass_kg - warm_out.cylinder.mass_kg).abs() < 1e-12);
        assert!(cold_out.cylinder.gauge_pressure_pa < warm_out.cylinder.gauge_pressure_pa - 100.0 * gas::PSI_TO_PA);
    }

    #[test]
    fn the_low_setting_lasts_twice_as_long_as_the_high_one() {
        let mut high = TherapeuticOxygenSystem::default();
        let mut low = TherapeuticOxygenSystem::default();
        let h = run(&mut high, TherapeuticInputs { outlets_in_use: 4.0, high_flow: true, ..Default::default() }, TherapeuticFaults::default(), 600);
        let l = run(&mut low, TherapeuticInputs { outlets_in_use: 4.0, high_flow: false, ..Default::default() }, TherapeuticFaults::default(), 600);
        assert!((h.total_flow_kg_s / l.total_flow_kg_s - 2.0).abs() < 1e-9);
        assert!(l.cylinder.mass_kg > h.cylinder.mass_kg);
    }

    #[test]
    fn an_outlet_stuck_open_drains_the_bottle_with_nobody_on_it() {
        let mut sys = TherapeuticOxygenSystem::default();
        let faults = TherapeuticFaults { outlets_stuck_open: 1.0 / 11.0, ..Default::default() };
        let out = run(&mut sys, TherapeuticInputs::default(), faults, 3600);
        assert!(out.outlets_flowing > 0.0);
        assert!(out.cylinder.quantity_fraction < 0.95, "{}", out.cylinder.quantity_fraction);
    }

    #[test]
    fn a_leaking_cylinder_empties_and_the_outlets_stop() {
        let mut sys = TherapeuticOxygenSystem::default();
        let faults = TherapeuticFaults { cylinder: CylinderFaults { leak: 1.0, ..Default::default() }, ..Default::default() };
        let inputs = TherapeuticInputs { outlets_in_use: 1.0, ..Default::default() };
        let out = run(&mut sys, inputs, faults, 1200);
        assert!(out.cylinder.quantity_fraction < 0.05, "{}", out.cylinder.quantity_fraction);
        assert!(!out.supply_available);
        assert_eq!(out.total_flow_kg_s, 0.0);
    }

    #[test]
    fn nothing_breaks_at_rest() {
        let mut sys = TherapeuticOxygenSystem::default();
        let a = sys.step(TherapeuticInputs::default(), TherapeuticFaults::default(), 0.0);
        let b = sys.step(TherapeuticInputs::default(), TherapeuticFaults::default(), 0.0);
        assert_eq!(a, b);
        assert!(a.cylinder.absolute_pressure_pa > 0.0 && a.endurance_s.is_finite());
        sys.cylinder_mut().set_mass_kg(0.0);
        let empty = sys.step(TherapeuticInputs { outlets_in_use: 11.0, ..Default::default() }, TherapeuticFaults::default(), 1.0);
        assert_eq!(empty.total_flow_kg_s, 0.0);
        assert!(empty.cylinder.absolute_pressure_pa.is_finite());
    }
}
