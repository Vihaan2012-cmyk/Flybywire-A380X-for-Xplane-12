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
//! it drains) plus a boost pump's assist pressure, against the ambient
//! static pressure at altitude the real orifice equation already includes --
//! so jettison rate genuinely falls through the dump as the tanks empty, and
//! genuinely varies with altitude, rather than being read once from a
//! decreasing pair of tank quantities against a fixed combined orifice.

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

/// Mass flow rate overboard through one nozzle, kg/s: head pressure from the
/// tank's own remaining liquid depth, plus an optional boost-pump assist
/// pressure, against ambient static pressure at the aircraft's current
/// altitude (falls with altitude -- the real reason published jettison rates
/// are usually quoted at a reference altitude: the *same* tank head jettisons
/// faster once ambient back-pressure has dropped).
pub fn jettison_mass_flow_kg_s(cda_m2: f64, liquid_depth_m: f64, boost_pump_pressure_pa: f64, ambient_pressure_pa: f64, density_kg_m3: f64) -> f64 {
    let drive_pressure = head_pressure_pa(liquid_depth_m, density_kg_m3) + boost_pump_pressure_pa.max(0.0);
    let delta_p = drive_pressure - ambient_pressure_pa.max(0.0);
    orifice_flow_m3_s(cda_m2, delta_p, density_kg_m3) * density_kg_m3.max(0.0)
}

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
        let full = jettison_mass_flow_kg_s(cda, 3.0, 0.0, SEA_LEVEL_PA, RHO);
        let half = jettison_mass_flow_kg_s(cda, 1.5, 0.0, SEA_LEVEL_PA, RHO);
        let empty = jettison_mass_flow_kg_s(cda, 0.0, 0.0, SEA_LEVEL_PA, RHO);
        assert!(full > half);
        assert!(half > empty);
        assert_eq!(empty, 0.0);
    }

    #[test]
    fn jettison_flow_is_higher_at_altitude_for_the_same_tank_head() {
        let cda = 0.0015;
        let sea_level = jettison_mass_flow_kg_s(cda, 2.0, 0.0, SEA_LEVEL_PA, RHO);
        let cruise = jettison_mass_flow_kg_s(cda, 2.0, 0.0, CRUISE_PA, RHO);
        assert!(cruise > sea_level, "lower ambient back-pressure should pass more flow");
    }

    #[test]
    fn a_boost_pump_assist_adds_to_the_driving_pressure() {
        let cda = 0.0015;
        let gravity_only = jettison_mass_flow_kg_s(cda, 1.0, 0.0, SEA_LEVEL_PA, RHO);
        let pump_assisted = jettison_mass_flow_kg_s(cda, 1.0, 50_000.0, SEA_LEVEL_PA, RHO);
        assert!(pump_assisted > gravity_only);
    }
}
