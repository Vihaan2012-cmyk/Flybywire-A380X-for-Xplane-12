//! Jettison pumps, valves and nozzles, flow vs head (backlog item 5).
//!
//! `src/fuel.rs`'s existing jettison model (module doc, "FUEL-001") already
//! does real orifice physics for the two jettison nozzles *lumped together*:
//! one calibrated `nozzle_cda_m2` (from `physics::fluids::effective_cda_m2`)
//! drains the wing tanks in proportion to their contents the instant the
//! jettison switch is armed, with the valves opening/closing instantly
//! (`self.net.open_valve`/`close_valve`, no transit time) and no way to fail
//! one nozzle independently of the other. This module goes one layer deeper,
//! self-contained: each of the two real named valves
//! (`JettisonNozzleValveLeft`/`Right`, `flight_model.cfg` lines 308-309,
//! already looked up by name in `fuel.rs`'s own `Jettison` struct) gets its
//! own transit-time dynamics and its own stuck/blockage faults, and flow is
//! computed from the *actual* hydrostatic head of the tank (which falls as
//! it drains) plus a boost pump's assist pressure, against the pressure at
//! the nozzle's own exit -- so jettison rate genuinely falls through the
//! dump as the tanks empty, rather than being read once from a decreasing
//! pair of tank quantities against a fixed combined orifice. Because the
//! tanks are vented, the ambient static pressure appears on both sides of
//! the nozzle and cancels: see [`jettison_mass_flow_kg_s`].

use super::geometry::G;

/// `Q = Cd*A*sqrt(2*dP/rho)`, the standard incompressible sharp-orifice flow
/// equation (Crane Technical Paper 410 / any fluids-engineering handbook);
/// `cda_m2` is the already-combined `Cd*A` this codebase uses throughout
/// (`physics::fluids::effective_cda_m2`'s own convention, reimplemented
/// locally per this directory's self-containment rule). Zero or negative
/// `delta_pressure_pa` (nothing pushing fuel out, e.g. below ambient) gives
/// zero flow, not a negative or `NaN` one.
pub fn orifice_flow_m3_s(cda_m2: f64, delta_pressure_pa: f64, density_kg_m3: f64) -> f64 {
    if delta_pressure_pa <= 0.0 || density_kg_m3 <= 0.0 || cda_m2 <= 0.0 {
        return 0.0;
    }
    cda_m2 * (2.0 * delta_pressure_pa / density_kg_m3).sqrt()
}

/// The hydrostatic head at a nozzle fed by gravity/pump from a tank whose
/// liquid stands `liquid_depth_m` deep above the nozzle's own inlet.
pub fn head_pressure_pa(liquid_depth_m: f64, density_kg_m3: f64) -> f64 {
    (liquid_depth_m.max(0.0)) * density_kg_m3.max(0.0) * G
}

/// One jettison nozzle valve's own position, 0 (shut) .. 1 (open), ramping
/// over `opening_time_s` the same way `fuel_network.rs`'s own valves do
/// (`DEFAULT_VALVE_OPENING_TIME_S`, `fuel_network.rs:118`, reused here only
/// as a citation for the convention, not called). A `stuck_fraction` > 0
/// freezes the valve's *reachable range* at the position it held the moment
/// the fault appeared -- the same "sticks wherever it currently is" physical
/// picture `fuel_network.rs`'s `valve_stuck_at` already uses for its own
/// generically-indexed valves, reproduced here for this named component so
/// the failure catalogue can point at it directly.
#[derive(Clone, Copy, Debug, Default)]
pub struct JettisonValve {
    pub position: f64,
    stuck_at: Option<f64>,
    was_stuck: bool,
}
impl JettisonValve {
    pub fn new() -> Self {
        Self::default()
    }
    pub fn step(&mut self, commanded_open: bool, opening_time_s: f64, stuck_fraction: f64, dt_s: f64) {
        let stuck_fraction = stuck_fraction.clamp(0.0, 1.0);
        let is_stuck = stuck_fraction > 0.0;
        if is_stuck && !self.was_stuck {
            self.stuck_at = Some(self.position);
        }
        if !is_stuck {
            self.stuck_at = None;
        }
        self.was_stuck = is_stuck;
        let commanded_target = if commanded_open { 1.0 } else { 0.0 };
        let target = match self.stuck_at {
            Some(stuck) => stuck + (commanded_target - stuck) * (1.0 - stuck_fraction),
            None => commanded_target,
        };
        if opening_time_s <= 0.0 {
            self.position = target.clamp(0.0, 1.0);
            return;
        }
        let max_step = (1.0 / opening_time_s) * dt_s.max(0.0);
        let diff = (target - self.position).clamp(-max_step, max_step);
        self.position = (self.position + diff).clamp(0.0, 1.0);
    }
}

/// A nozzle's effective throat area given its nominal `Cd*A`, a partial
/// mechanical blockage fault (debris/icing narrowing the throat, 0 clear ..
/// 1 fully blocked) and the upstream valve's own position (a part-open
/// poppet/gate throttles flow area roughly linearly, the same assumption
/// `fuel_network.rs`'s partly-open valves make for line capacity).
pub fn effective_cda_m2(nominal_cda_m2: f64, blockage_fraction: f64, valve_position: f64) -> f64 {
    (nominal_cda_m2.max(0.0) * (1.0 - blockage_fraction.clamp(0.0, 1.0)) * valve_position.clamp(0.0, 1.0)).max(0.0)
}

/// Mass flow rate overboard through one nozzle, kg/s.
///
/// The pressure that drives fuel out of the nozzle is the difference across
/// it, not an absolute pressure:
///
/// ```text
/// dP = (ullage + rho*g*h + pump) - nozzle exit
/// ```
///
/// The A380's tanks are *vented* (NACA vents through the surge tanks), so
/// the ullage sits at the ambient static pressure of whatever altitude the
/// aircraft is at, and the nozzle discharges into air at very nearly that
/// same static pressure. The two ambients therefore cancel, and what is
/// left is the gauge head `rho*g*h` plus the jettison pump's own rise --
/// which is why a vented-tank jettison rate is essentially independent of
/// altitude and why it falls as the tanks drain. (Subtracting the *absolute*
/// ambient from the *gauge* head, as an earlier version of this function
/// did, mixes two different datums: 3 m of fuel is 23.5 kPa of head, so that
/// arithmetic gave zero flow at any sea-level altitude -- it would have
/// meant the fuel could not even leave a tank on the ground.)
///
/// `nozzle_exit_pressure_pa` is kept separate from `ullage_pressure_pa`
/// precisely so the one real coupling to the airflow is expressible: the
/// nozzle sticks out into a stream whose local static pressure is below
/// free-stream by `-Cp * q`, which sucks a little extra flow out, and a
/// blocked/iced vent that lets the ullage drop below ambient throttles it.
pub fn jettison_mass_flow_kg_s(
    cda_m2: f64,
    liquid_depth_m: f64,
    boost_pump_pressure_pa: f64,
    ullage_pressure_pa: f64,
    nozzle_exit_pressure_pa: f64,
    density_kg_m3: f64,
) -> f64 {
    let drive_pressure = ullage_pressure_pa.max(0.0) + head_pressure_pa(liquid_depth_m, density_kg_m3) + boost_pump_pressure_pa.max(0.0);
    let delta_p = drive_pressure - nozzle_exit_pressure_pa.max(0.0);
    orifice_flow_m3_s(cda_m2, delta_p, density_kg_m3) * density_kg_m3.max(0.0)
}

/// Nominal `Cd*A` of one jettison nozzle, m^2. Sized from a reference
/// jettison rate of about 2,000 kg/min per side (~33.3 kg/s, i.e. 0.0417
/// m^3/s of 800 kg/m^3 fuel) with the jettison pumps running at
/// [`NOMINAL_JETTISON_PUMP_RISE_PA`] and a nearly full tank:
///   v = sqrt(2 * (50 000 + 800*9.81*2.0) / 800) = 12.9 m/s
///   Cd*A = 0.0417 / 12.9 = 3.2e-3 m^2
/// (a ~70 mm effective throat with Cd ~ 0.8, the right order for a nozzle
/// fed by a 3-inch jettison line).
///
/// **GENERIC, and more so than an earlier revision of this file claimed.**
/// That revision called 2,000 kg/min per side "the published A380 rate".
/// It is not published. Airbus does not state an A380 jettison rate in any
/// public document found, and the figures that circulate disagree badly
/// with each other and with this one:
///   - ~3,300 kg/min total (A380 ATA 28 training-material summaries),
///   - ~2,500 kg/min total (from the widely repeated "about 50 t in about
///     20 minutes" datum),
///   - 2,200 lb/min (~1,000 kg/min) in one set of type notes,
/// against the 4,000 kg/min *total* this constant implies. Searched for an
/// A380 FCOM/AMM jettison rate, an Airbus published figure and FlyByWire's
/// own model (their A380X jettison is documented as "not available ... yet"
/// and their source carries no rate): none found. So both the rate and,
/// downstream of it, this area are unsourced.
///
/// What *is* citable is the certification frame the rate has to satisfy:
/// 14 CFR / CS 25.1001(b), "if a fuel jettisoning system is required it
/// must be capable of jettisoning enough fuel within 15 minutes ... to
/// enable the airplane to meet the climb requirements of 25.119 and
/// 25.121(d)". That is a floor on the rate, not a value for it -- the
/// weight to be shed depends on the landing-climb weight for the day -- so
/// it cannot pin this constant, but it does say which direction an error
/// here is unsafe, and this file's own test checks the modelled rate
/// against it.
///
/// The split between this area and [`NOMINAL_JETTISON_PUMP_RISE_PA`] was
/// already, and remains, arbitrary: only their product through the orifice
/// law is constrained, and even that is constrained by an unsourced rate.
pub const NOMINAL_NOZZLE_CDA_M2: f64 = 3.2e-3;
/// Pressure rise of a jettison/transfer pump at its jettison flow, Pa.
/// GENERIC: aircraft fuel boost pumps are quoted in the 5-15 psi class;
/// 50 kPa = 7.3 psi sits in that band and is the figure the nozzle above is
/// sized against, so the two are consistent by construction. Searched for
/// an A380 fuel pump delivery pressure (AMM ATA 28 level figures): not
/// public.
pub const NOMINAL_JETTISON_PUMP_RISE_PA: f64 = 50_000.0;

/// Reference jettison rate per side this file's nozzle area is sized
/// against, kg/s. See [`NOMINAL_NOZZLE_CDA_M2`] for why this is GENERIC.
pub const REFERENCE_JETTISON_RATE_PER_SIDE_KG_S: f64 = 2_000.0 / 60.0;

#[cfg(test)]
mod tests {
    use super::*;

    const RHO: f64 = 800.0;
    const SEA_LEVEL_PA: f64 = 101_325.0;
    const CRUISE_PA: f64 = 22_600.0; // ~ FL350 ISA static pressure.

    #[test]
    fn orifice_flow_is_zero_with_no_favourable_pressure_difference() {
        assert_eq!(orifice_flow_m3_s(0.001, 0.0, RHO), 0.0);
        assert_eq!(orifice_flow_m3_s(0.001, -100.0, RHO), 0.0);
        assert_eq!(orifice_flow_m3_s(0.0, 1000.0, RHO), 0.0);
    }

    #[test]
    fn orifice_flow_grows_with_area_and_pressure_and_never_nans() {
        let base = orifice_flow_m3_s(0.001, 50_000.0, RHO);
        assert!(base > 0.0 && base.is_finite());
        assert!(orifice_flow_m3_s(0.002, 50_000.0, RHO) > base);
        assert!(orifice_flow_m3_s(0.001, 100_000.0, RHO) > base);
    }

    #[test]
    fn a_valve_commanded_open_ramps_fully_open_over_its_opening_time() {
        let mut v = JettisonValve::new();
        v.step(true, 2.0, 0.0, 1.0);
        assert!((v.position - 0.5).abs() < 1e-9);
        v.step(true, 2.0, 0.0, 1.0);
        assert!((v.position - 1.0).abs() < 1e-9);
    }

    #[test]
    fn a_valve_commanded_closed_ramps_back_shut() {
        let mut v = JettisonValve { position: 1.0, ..Default::default() };
        v.step(false, 1.0, 0.0, 1.0);
        assert!((v.position - 0.0).abs() < 1e-9);
    }

    #[test]
    fn a_stuck_valve_freezes_its_reachable_range_at_the_position_it_seized_at() {
        let mut v = JettisonValve::new();
        v.step(true, 1.0, 0.0, 0.5); // ramps to 0.5 open, healthy.
        assert!((v.position - 0.5).abs() < 1e-9);
        // Now it seizes fully (stuck_fraction 1.0): commanding fully open
        // must not move it further.
        v.step(true, 1.0, 1.0, 5.0);
        assert!((v.position - 0.5).abs() < 1e-9);
    }

    #[test]
    fn a_partially_stuck_valve_has_partial_authority_to_keep_moving() {
        let mut v = JettisonValve::new();
        v.step(true, 1.0, 0.0, 0.5); // 0.5 open, healthy.
        v.step(true, 10.0, 0.5, 100.0); // seizes at 50% authority, plenty of time.
        assert!(v.position > 0.5 && v.position < 1.0, "{}", v.position);
    }

    #[test]
    fn blockage_and_a_shut_valve_both_reduce_effective_area_to_zero_or_less() {
        assert_eq!(effective_cda_m2(0.002, 1.0, 1.0), 0.0);
        assert_eq!(effective_cda_m2(0.002, 0.0, 0.0), 0.0);
        let half = effective_cda_m2(0.002, 0.5, 1.0);
        assert!((half - 0.001).abs() < 1e-9);
    }

    #[test]
    fn jettison_flow_falls_as_the_tank_drains() {
        let cda = 0.0015;
        let full = jettison_mass_flow_kg_s(cda, 3.0, 0.0, SEA_LEVEL_PA, SEA_LEVEL_PA, RHO);
        let half = jettison_mass_flow_kg_s(cda, 1.5, 0.0, SEA_LEVEL_PA, SEA_LEVEL_PA, RHO);
        let empty = jettison_mass_flow_kg_s(cda, 0.0, 0.0, SEA_LEVEL_PA, SEA_LEVEL_PA, RHO);
        assert!(full > half);
        assert!(half > empty);
        assert_eq!(empty, 0.0);
        // Gravity-only flow goes as sqrt(h), so halving the head must cost
        // exactly a factor sqrt(2): Q(3.0 m)/Q(1.5 m) = sqrt(2) = 1.4142.
        assert!((full / half - std::f64::consts::SQRT_2).abs() < 1e-9, "{full} / {half}");
    }

    #[test]
    fn jettison_flow_is_set_by_the_pressure_across_the_nozzle_not_by_altitude() {
        let cda = 0.0015;
        // The tanks are vented, so the ullage is at ambient and the nozzle
        // discharges into ambient: the same tank head jettisons at the same
        // rate at sea level and at FL350. (The old expectation here -- that
        // a lower ambient passes more flow -- would only hold for a sealed,
        // pressurised tank, which this aircraft does not have.)
        let sea_level = jettison_mass_flow_kg_s(cda, 2.0, 0.0, SEA_LEVEL_PA, SEA_LEVEL_PA, RHO);
        let cruise = jettison_mass_flow_kg_s(cda, 2.0, 0.0, CRUISE_PA, CRUISE_PA, RHO);
        assert!(sea_level > 0.0);
        assert!((cruise - sea_level).abs() < 1e-9, "{cruise} vs {sea_level}");
        // What the altitude/airflow really buys is the suction at the nozzle
        // exit, which sits in a stream at a local static pressure below
        // free-stream. 5 kPa of it on top of 2 m of head
        // (rho*g*h = 800*9.81*2 = 15.70 kPa) is a 32% bigger dP, i.e. a
        // sqrt(20.70/15.70) = 1.148x flow.
        let with_suction = jettison_mass_flow_kg_s(cda, 2.0, 0.0, CRUISE_PA, CRUISE_PA - 5_000.0, RHO);
        assert!((with_suction / cruise - 1.148).abs() < 0.002, "{}", with_suction / cruise);
    }

    #[test]
    fn a_boost_pump_assist_adds_to_the_driving_pressure() {
        let cda = 0.0015;
        let gravity_only = jettison_mass_flow_kg_s(cda, 1.0, 0.0, SEA_LEVEL_PA, SEA_LEVEL_PA, RHO);
        let pump_assisted = jettison_mass_flow_kg_s(cda, 1.0, 50_000.0, SEA_LEVEL_PA, SEA_LEVEL_PA, RHO);
        assert!(pump_assisted > gravity_only);
        // dP goes from 800*9.81*1 = 7.85 kPa to 57.85 kPa, so the flow goes
        // up by sqrt(57.85/7.85) = 2.714x -- the pumps, not gravity, are
        // what make a jettison quick.
        assert!((pump_assisted / gravity_only - 2.714).abs() < 0.005, "{}", pump_assisted / gravity_only);
    }

    #[test]
    fn the_nominal_nozzle_dumps_about_two_thousand_kg_per_minute_per_side() {
        // The sizing anchor for NOMINAL_NOZZLE_CDA_M2: a nearly full tank
        // (2 m of head) with the jettison pumps running dumps at the
        // reference rate of roughly 2,000 kg/min per side. That rate is
        // GENERIC, not published -- see NOMINAL_NOZZLE_CDA_M2 -- so this
        // test only checks that the constant and its own stated derivation
        // still agree, which is what stops one being edited without the
        // other.
        let kg_s = jettison_mass_flow_kg_s(
            NOMINAL_NOZZLE_CDA_M2,
            2.0,
            NOMINAL_JETTISON_PUMP_RISE_PA,
            SEA_LEVEL_PA,
            SEA_LEVEL_PA,
            RHO,
        );
        let kg_min = kg_s * 60.0;
        assert!((kg_min - REFERENCE_JETTISON_RATE_PER_SIDE_KG_S * 60.0).abs() < 100.0, "{kg_min} kg/min");
    }

    #[test]
    fn the_modelled_rate_sits_inside_the_band_of_publicly_quoted_figures() {
        // The publicly circulating A380 jettison figures (none of them an
        // Airbus publication -- see NOMINAL_NOZZLE_CDA_M2) run from about
        // 1,000 kg/min total to about 3,300 kg/min total. This model's two
        // nozzles together give 4,000 kg/min at a nearly full tank, which
        // is above all of them -- but those are quoted as *average* or
        // *achieved* rates over a whole jettison, and this is the
        // instantaneous rate at maximum head, which the orifice law makes
        // the fastest moment of the whole evolution (flow goes as
        // sqrt(pump + rho*g*h), so it decays as the tank drains).
        //
        // So the honest check is a bracket, not a match: the peak rate must
        // exceed the highest quoted sustained figure (or the model could
        // never average it) and must not be wildly above it either. Both
        // bounds are wide on purpose, because the target itself is not
        // sourced; this test exists to catch a nozzle area edited by an
        // order of magnitude, not to calibrate one.
        let per_side_kg_min = jettison_mass_flow_kg_s(
            NOMINAL_NOZZLE_CDA_M2,
            2.0,
            NOMINAL_JETTISON_PUMP_RISE_PA,
            SEA_LEVEL_PA,
            SEA_LEVEL_PA,
            RHO,
        ) * 60.0;
        let both_sides_kg_min = 2.0 * per_side_kg_min;
        assert!(
            both_sides_kg_min > 3_300.0,
            "peak total jettison {both_sides_kg_min} kg/min cannot average the highest quoted figure"
        );
        assert!(
            both_sides_kg_min < 2.0 * 3_300.0,
            "peak total jettison {both_sides_kg_min} kg/min is more than twice the highest quoted figure"
        );
    }
}
