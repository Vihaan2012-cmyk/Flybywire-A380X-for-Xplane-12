//! The two regulators between a crew oxygen cylinder and a pilot's lungs:
//! the pressure reducer at the bottle, and the diluter-demand regulator on
//! the mask.
//!
//! ## The dilution schedule is derived, not drawn
//!
//! Every description of a diluter-demand regulator says the same two
//! things: it mixes cabin air with bottle oxygen so that the wearer
//! breathes as if at sea level, and by somewhere in the mid thirty
//! thousands of feet it has run out of mixing to do and is delivering pure
//! oxygen. The usual way to model that is to draw a curve between those
//! two points, which is an invented curve with real endpoints.
//!
//! [`diluter_demand_o2_fraction`] instead solves the alveolar gas equation
//! -- the standard respiratory-physiology relation
//!
//! ```text
//! P_A,O2 = (P_ambient - P_H2O) * FiO2 - P_A,CO2 / RQ
//! ```
//!
//! for the inspired oxygen fraction that holds alveolar oxygen tension at
//! its sea-level value. The pure-oxygen crossover then comes *out* of the
//! model: it lands at about 33 000 ft, which is where the published figure
//! (commonly quoted as 33 700 or 34 000 ft) actually is. That agreement is
//! asserted in this module's tests, so the physiology is checked against
//! the literature rather than fitted to it.
//!
//! ## Sources
//!
//! * Alveolar gas equation, and its constants: saturated water vapour
//!   pressure at body temperature 47 mmHg, alveolar CO2 tension 40 mmHg,
//!   respiratory quotient 0.8 -- standard respiratory-physiology values
//!   (West, *Respiratory Physiology*, and every aviation-medicine text).
//! * Oxygen mole fraction of dry air 0.2095: [`super::gas`].
//! * Body temperature 310.15 K (37 C) for the BTPS conditions a minute
//!   volume is quoted at.
//! * Low-pressure distribution setpoint 85 psi: the figure published
//!   across ATA 35 descriptions of the Airbus narrowbody crew oxygen
//!   system. No A380-specific public value was found, and it is labelled
//!   as that rather than as an A380 figure.

use super::gas;

/// Saturated water vapour pressure at body temperature, Pa (47 mmHg).
pub const ALVEOLAR_WATER_VAPOUR_PA: f64 = 6266.2;
/// Alveolar carbon dioxide tension, Pa (40 mmHg).
pub const ALVEOLAR_CO2_PA: f64 = 5332.9;
/// Respiratory quotient (CO2 produced per O2 consumed), dimensionless.
pub const RESPIRATORY_QUOTIENT: f64 = 0.8;
/// Body temperature, K -- the T in BTPS.
pub const BODY_TEMP_K: f64 = 310.15;

/// Resting adult minute ventilation under a mask, litres per minute BTPS.
/// **GENERIC**: respiratory-physiology references put resting adult minute
/// volume in the 6-10 L/min band and nothing aviation-specific narrows it,
/// so this is the midpoint. It scales the whole crew consumption rate
/// linearly, which is why it is called out here rather than buried.
pub const RESTING_MINUTE_VOLUME_L_PER_MIN: f64 = 8.0;

/// Extra continuous flow an emergency (pressure-breathing) selection
/// costs, litres per minute at delivery conditions. **GENERIC**: in
/// EMERGENCY the regulator holds the mask above cabin pressure, so it
/// leaks continuously around the face seal -- the flow is real and its
/// direction is certain, the figure is not published. Sized at about
/// half a resting minute volume, which keeps it a meaningful penalty on
/// endurance without dominating it.
pub const EMERGENCY_SEAL_LEAK_L_PER_MIN: f64 = 5.0;

/// Low-pressure distribution setpoint, Pa gauge (85 psi; see module doc).
pub const DISTRIBUTION_SETPOINT_GAUGE_PA: f64 = 85.0 * gas::PSI_TO_PA;

/// How far the inlet has to stay above the outlet for a single-stage
/// reducer to still hold its setpoint, Pa. **GENERIC**: every reducer has
/// a dropout margin, below which the outlet simply follows the inlet down.
/// 5 psi is a small one, so the modelled regulator holds its setpoint
/// almost until the cylinder is empty -- which is the behaviour that makes
/// a nearly-empty bottle dangerous rather than obviously dead.
pub const REGULATOR_DROPOUT_MARGIN_PA: f64 = 5.0 * gas::PSI_TO_PA;

/// Outlet droop per unit of delivered mass flow, Pa/(kg/s). **GENERIC**,
/// sized so that the largest flow this system can legitimately draw (four
/// masks in emergency at altitude, a little over a gram a minute) costs a
/// few percent of setpoint: a real reducer droops under flow, and a model
/// with no droop at all would make the regulator's own state invisible.
pub const REGULATOR_DROOP_PA_PER_KG_S: f64 = 1.5e8;

/// Relief-valve setting on the low-pressure side, as a multiple of
/// setpoint. **GENERIC**: low-pressure oxygen distribution is protected
/// against a failed-open reducer, and the relief has to sit far enough
/// above setpoint that droop and normal transients never reach it.
pub const LOW_PRESSURE_RELIEF_MULTIPLE: f64 = 1.5;

/// What the mask regulator has been selected to do.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum MaskMode {
    /// N: diluter demand, mixing cabin air with bottle oxygen.
    #[default]
    Normal,
    /// 100%: undiluted oxygen on demand.
    Pure,
    /// EMER: undiluted oxygen at a positive mask pressure.
    Emergency,
}

/// Alveolar oxygen tension breathing cabin air at sea level, Pa.
///
/// The target the diluter schedule holds. Computed rather than quoted, so
/// that it is consistent with the same equation used at altitude -- the
/// round textbook figure (103 mmHg) is the same number to within its own
/// rounding, which this module's tests check.
pub fn sea_level_alveolar_po2_pa() -> f64 {
    (101_325.0 - ALVEOLAR_WATER_VAPOUR_PA) * gas::AIR_O2_MOLE_FRACTION - ALVEOLAR_CO2_PA / RESPIRATORY_QUOTIENT
}

/// The inspired oxygen fraction a diluter-demand regulator delivers at a
/// given cabin pressure: whatever holds alveolar oxygen tension at its
/// sea-level value, bounded below by what cabin air already provides and
/// above by pure oxygen.
pub fn diluter_demand_o2_fraction(cabin_pressure_pa: f64) -> f64 {
    let dry = cabin_pressure_pa - ALVEOLAR_WATER_VAPOUR_PA;
    if dry <= 0.0 {
        return 1.0;
    }
    let required = (sea_level_alveolar_po2_pa() + ALVEOLAR_CO2_PA / RESPIRATORY_QUOTIENT) / dry;
    required.clamp(gas::AIR_O2_MOLE_FRACTION, 1.0)
}

/// The fraction of each breath that has to come out of the bottle to reach
/// `o2_fraction`, given the rest of it is cabin air.
///
/// `FiO2 = x * 1 + (1 - x) * f_air`, so `x = (FiO2 - f_air)/(1 - f_air)`.
/// At sea level this is zero: a diluter-demand regulator draws *nothing*
/// from the bottle with a mask donned on the ground, which is why crew
/// oxygen endurance is quoted against cabin altitude and not against time.
pub fn bottle_draw_fraction(o2_fraction: f64) -> f64 {
    let f_air = gas::AIR_O2_MOLE_FRACTION;
    ((o2_fraction.clamp(0.0, 1.0) - f_air) / (1.0 - f_air)).clamp(0.0, 1.0)
}

/// The oxygen fraction one mask regulator delivers this frame.
///
/// `dilution_stuck_ambient` is the failure magnitude: 1.0 is a diluter
/// jammed fully to its air inlet, which delivers cabin air at any altitude
/// and is silently lethal -- the mask still breathes, it just contains
/// nothing useful.
pub fn delivered_o2_fraction(mode: MaskMode, cabin_pressure_pa: f64, dilution_stuck_ambient: f64) -> f64 {
    let healthy = match mode {
        MaskMode::Normal => diluter_demand_o2_fraction(cabin_pressure_pa),
        MaskMode::Pure | MaskMode::Emergency => 1.0,
    };
    let stuck = dilution_stuck_ambient.clamp(0.0, 1.0);
    healthy * (1.0 - stuck) + gas::AIR_O2_MOLE_FRACTION * stuck
}

/// Mass flow one donned mask draws from the bottle, kg/s.
///
/// The volume the wearer breathes is at cabin pressure and body
/// temperature; the share of it that comes from the bottle is
/// [`bottle_draw_fraction`]; the mass of that share follows from the
/// density of oxygen at those conditions. At sea level in NORMAL the
/// answer is exactly zero, and it has to be -- anything else would be a
/// bottle draining on the ground.
pub fn mask_demand_kg_s(mode: MaskMode, cabin_pressure_pa: f64, delivered_fraction: f64, minute_volume_l_per_min: f64) -> f64 {
    if !(cabin_pressure_pa > 0.0) {
        return 0.0;
    }
    let seal_leak = if mode == MaskMode::Emergency { EMERGENCY_SEAL_LEAK_L_PER_MIN } else { 0.0 };
    let volume_m3_s = (minute_volume_l_per_min.max(0.0) + seal_leak) * 1e-3 / 60.0;
    let density = gas::delivered_density_kg_m3(cabin_pressure_pa, BODY_TEMP_K);
    let draw = bottle_draw_fraction(delivered_fraction);
    // A pure-oxygen or emergency selection draws the whole breath from the
    // bottle whatever the altitude, which is what makes 100% expensive.
    let share = match mode {
        MaskMode::Normal => draw,
        MaskMode::Pure | MaskMode::Emergency => delivered_fraction.clamp(0.0, 1.0),
    };
    volume_m3_s * density * share
}

/// A single-stage pressure reducer between the cylinder and the masks.
#[derive(Clone, Copy, Debug)]
pub struct PressureRegulator {
    pub setpoint_gauge_pa: f64,
}

/// What can be wrong with the reducer.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct RegulatorFaults {
    /// Signed shift of the setpoint as a fraction of it: -1 drives the
    /// outlet to nothing (the masks starve), +1 doubles it and lifts the
    /// low-pressure relief.
    pub setpoint_shift: f64,
    /// Seat leak past the reducer, as a fraction of
    /// [`SEAT_LEAK_FULL_SCALE_M2`]: bottle pressure bleeding continuously
    /// into the low-pressure side and out through its relief.
    pub seat_leak: f64,
}

/// Orifice area a fully failed reducer seat leaks through, m^2.
/// **GENERIC**: a seat leak is a leak past a poppet, so it is an orifice
/// like any other. At a hundredth of a square millimetre a fully failed
/// seat empties a crew cylinder overnight rather than in minutes, which is
/// what makes it the fault that is found on the next walkround rather than
/// in the air -- and what makes a bottle that was full yesterday and is
/// half empty today a real diagnostic signature.
pub const SEAT_LEAK_FULL_SCALE_M2: f64 = 1e-8;

/// What the reducer did this frame.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct RegulatorOutputs {
    /// Delivered low-pressure distribution pressure, Pa gauge.
    pub outlet_gauge_pa: f64,
    /// Whether the low-pressure relief is lifting.
    pub relief_lifted: bool,
    /// Whether the inlet has fallen far enough that the outlet is simply
    /// following it down -- the reducer has dropped out.
    pub dropped_out: bool,
}

impl PressureRegulator {
    pub fn new(setpoint_gauge_pa: f64) -> Self {
        Self { setpoint_gauge_pa }
    }

    /// Outlet pressure for an inlet gauge pressure and a delivered flow.
    pub fn step(&self, inlet_gauge_pa: f64, flow_kg_s: f64, faults: RegulatorFaults) -> RegulatorOutputs {
        let shifted = self.setpoint_gauge_pa * (1.0 + faults.setpoint_shift.clamp(-1.0, 1.0));
        let droop = REGULATOR_DROOP_PA_PER_KG_S * flow_kg_s.max(0.0);
        let commanded = (shifted - droop).max(0.0);
        let ceiling = (inlet_gauge_pa - REGULATOR_DROPOUT_MARGIN_PA).max(0.0);
        let dropped_out = commanded > ceiling;
        let mut outlet = commanded.min(ceiling);
        let relief_at = self.setpoint_gauge_pa * LOW_PRESSURE_RELIEF_MULTIPLE;
        let relief_lifted = outlet > relief_at;
        if relief_lifted {
            outlet = relief_at;
        }
        RegulatorOutputs { outlet_gauge_pa: outlet, relief_lifted, dropped_out }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_alveolar_target_agrees_with_the_textbook_figure() {
        // 103 mmHg is the number every physiology text quotes for
        // sea-level alveolar oxygen tension. Ours is computed; they should
        // be the same to within the rounding in that figure.
        let quoted = 103.0 * 133.322_387_415;
        let ours = sea_level_alveolar_po2_pa();
        assert!((ours - quoted).abs() < 800.0, "computed {ours} Pa vs quoted {quoted} Pa");
    }

    #[test]
    fn the_pure_oxygen_crossover_lands_where_the_published_figure_is() {
        // The model is not told where the crossover is: it solves for the
        // fraction that holds alveolar oxygen, and the crossover is where
        // that reaches 1.0. It has to land near the published 33-34 000 ft
        // or the physiology is wrong.
        let mut crossover_ft = 0.0;
        for ft in 20_000..45_000 {
            // ISA pressure at this cabin altitude.
            let p = 101_325.0 * (1.0 - ft as f64 * 0.3048 / 44_330.77).powf(1.0 / 0.190_263_1);
            if diluter_demand_o2_fraction(p) >= 1.0 {
                crossover_ft = ft as f64;
                break;
            }
        }
        assert!(crossover_ft > 31_000.0 && crossover_ft < 36_000.0, "crossover at {crossover_ft} ft");
    }

    #[test]
    fn a_diluter_demand_mask_draws_nothing_at_all_on_the_ground() {
        let f = diluter_demand_o2_fraction(101_325.0);
        assert!((f - gas::AIR_O2_MOLE_FRACTION).abs() < 1e-12, "{f}");
        assert!(bottle_draw_fraction(f) < 1e-12);
        assert!(mask_demand_kg_s(MaskMode::Normal, 101_325.0, f, RESTING_MINUTE_VOLUME_L_PER_MIN) < 1e-15);
        // Selecting 100% on the ground, by contrast, costs the full breath.
        assert!(mask_demand_kg_s(MaskMode::Pure, 101_325.0, 1.0, RESTING_MINUTE_VOLUME_L_PER_MIN) > 0.0);
    }

    #[test]
    fn the_draw_rises_with_cabin_altitude_and_tops_out_at_pure_oxygen() {
        let mut last = -1.0;
        for p in [101_325.0, 75_262.0, 57_182.0, 46_563.0, 35_600.0, 26_000.0] {
            let f = diluter_demand_o2_fraction(p);
            let d = mask_demand_kg_s(MaskMode::Normal, p, f, RESTING_MINUTE_VOLUME_L_PER_MIN);
            assert!(f >= gas::AIR_O2_MOLE_FRACTION && f <= 1.0);
            assert!(d > last, "draw should climb with cabin altitude: {d} after {last} at {p} Pa");
            last = d;
        }
        assert_eq!(diluter_demand_o2_fraction(10_000.0), 1.0);
        // Above the crossover the draw falls again, and that is correct
        // rather than a bug: the regulator is already delivering pure
        // oxygen, so a thinner cabin means less mass in every breath. It is
        // why crew oxygen endurance is *longest* in the worst cabin.
        let very_high = mask_demand_kg_s(MaskMode::Normal, 18_750.0, 1.0, RESTING_MINUTE_VOLUME_L_PER_MIN);
        assert!(very_high < last, "{very_high} vs {last}");
    }

    #[test]
    fn a_crew_bottle_lasts_a_realistic_number_of_hours_at_a_depressurised_cruise() {
        // The one end-to-end sanity check on the consumption rate: four
        // masks, cabin at 35 000 ft, a 6520 litre free-air supply. Published
        // crew oxygen duration charts for this class of cylinder run to
        // many hours, not minutes and not days.
        let p = 23_842.0;
        let f = diluter_demand_o2_fraction(p);
        let per_mask = mask_demand_kg_s(MaskMode::Normal, p, f, RESTING_MINUTE_VOLUME_L_PER_MIN);
        let total = 4.0 * per_mask;
        let supply_kg = gas::mass_from_free_air_kg(6520.0, 294.15);
        let hours = supply_kg / total / 3600.0;
        assert!(hours > 5.0 && hours < 40.0, "{hours} hours");
    }

    #[test]
    fn a_stuck_diluter_delivers_cabin_air_at_any_altitude() {
        let healthy = delivered_o2_fraction(MaskMode::Normal, 18_750.0, 0.0);
        let stuck = delivered_o2_fraction(MaskMode::Normal, 18_750.0, 1.0);
        assert_eq!(healthy, 1.0);
        assert!((stuck - gas::AIR_O2_MOLE_FRACTION).abs() < 1e-12, "{stuck}");
        // And the bottle stops being drawn down, which is exactly what
        // makes this failure invisible on the quantity gauge.
        assert_eq!(mask_demand_kg_s(MaskMode::Normal, 18_750.0, stuck, RESTING_MINUTE_VOLUME_L_PER_MIN), 0.0);
        // A 100% selection is mechanically downstream of the diluter's air
        // inlet, so it is not saved by selecting it -- half stuck is half
        // the oxygen.
        let half = delivered_o2_fraction(MaskMode::Pure, 18_750.0, 0.5);
        assert!(half > gas::AIR_O2_MOLE_FRACTION && half < 1.0, "{half}");
    }

    #[test]
    fn emergency_costs_more_than_pure_which_costs_more_than_normal() {
        let p = 57_182.0; // about 15 000 ft
        let f = diluter_demand_o2_fraction(p);
        let normal = mask_demand_kg_s(MaskMode::Normal, p, f, RESTING_MINUTE_VOLUME_L_PER_MIN);
        let pure = mask_demand_kg_s(MaskMode::Pure, p, 1.0, RESTING_MINUTE_VOLUME_L_PER_MIN);
        let emer = mask_demand_kg_s(MaskMode::Emergency, p, 1.0, RESTING_MINUTE_VOLUME_L_PER_MIN);
        assert!(normal < pure && pure < emer, "{normal} {pure} {emer}");
    }

    #[test]
    fn the_reducer_holds_its_setpoint_until_the_bottle_nearly_gives_out() {
        let r = PressureRegulator::new(DISTRIBUTION_SETPOINT_GAUGE_PA);
        let full = r.step(1850.0 * gas::PSI_TO_PA, 0.0, RegulatorFaults::default());
        assert!((full.outlet_gauge_pa - DISTRIBUTION_SETPOINT_GAUGE_PA).abs() < 1.0);
        assert!(!full.dropped_out && !full.relief_lifted);
        // Down at 100 psi in the bottle it still holds.
        let low = r.step(100.0 * gas::PSI_TO_PA, 0.0, RegulatorFaults::default());
        assert!((low.outlet_gauge_pa - DISTRIBUTION_SETPOINT_GAUGE_PA).abs() < 1.0);
        // At 50 psi it cannot, and follows the inlet down.
        let empty = r.step(50.0 * gas::PSI_TO_PA, 0.0, RegulatorFaults::default());
        assert!(empty.dropped_out);
        assert!(empty.outlet_gauge_pa < DISTRIBUTION_SETPOINT_GAUGE_PA);
        assert!(empty.outlet_gauge_pa > 0.0);
    }

    #[test]
    fn a_shifted_setpoint_starves_the_masks_or_lifts_the_relief() {
        let r = PressureRegulator::new(DISTRIBUTION_SETPOINT_GAUGE_PA);
        let inlet = 1850.0 * gas::PSI_TO_PA;
        let low = r.step(inlet, 0.0, RegulatorFaults { setpoint_shift: -0.8, ..Default::default() });
        assert!(low.outlet_gauge_pa < 0.3 * DISTRIBUTION_SETPOINT_GAUGE_PA, "{}", low.outlet_gauge_pa);
        let high = r.step(inlet, 0.0, RegulatorFaults { setpoint_shift: 1.0, ..Default::default() });
        assert!(high.relief_lifted);
        assert!((high.outlet_gauge_pa - DISTRIBUTION_SETPOINT_GAUGE_PA * LOW_PRESSURE_RELIEF_MULTIPLE).abs() < 1.0);
    }

    #[test]
    fn nothing_here_divides_by_zero() {
        assert!(diluter_demand_o2_fraction(0.0).is_finite());
        assert_eq!(mask_demand_kg_s(MaskMode::Normal, 0.0, 1.0, 8.0), 0.0);
        let r = PressureRegulator::new(DISTRIBUTION_SETPOINT_GAUGE_PA);
        let out = r.step(0.0, 0.0, RegulatorFaults::default());
        assert_eq!(out.outlet_gauge_pa, 0.0);
        assert!(out.dropped_out);
    }
}
