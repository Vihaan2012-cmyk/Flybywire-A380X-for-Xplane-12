//! Avionics bay ventilation and cooling: the fan(s) and extract valve that
//! draw conditioned air across a bay's CPIOM/IOM racks, the resulting bay
//! temperature, and the overheat trip that takes a module off the network
//! (`faults::ModuleFaults::overheat_trip_frac`) when cooling is lost long
//! enough. This is the interface `faults`/`graph` document: nothing here
//! reaches into `ModuleFaults` itself (this module has no dependency on
//! `faults`), it only produces the number the crate's integration layer
//! hands to it.
//!
//! Public commercial-aircraft avionics cooling is forced-air: a fan (or
//! two, run together for margin rather than one held as a cold standby)
//! pulls cabin/ECS-conditioned air through the equipment racks and an
//! extract valve ducts it overboard or back to the mix manifold. Losing
//! the fan(s) *or* the extract valve both remove the forced draught (a
//! series duct — air has to get both pulled *and* let out), collapsing
//! heat transfer to whatever natural convection the still bay air manages
//! on its own; a bay's modules then heat up on their own dissipation until
//! their own thermal supervisors trip.
//!
//! The A380's actual bay cooling architecture, airflow rates and trip
//! setpoints are not public; every constant below is GENERIC, derived as
//! documented at each one from public general-aviation-electronics
//! cooling and environmental-qualification practice.

/// ECS-conditioned supply air temperature reaching the avionics bay
/// (GENERIC: a typical avionics-bay supply target, a few degrees below
/// standard cabin temperature to give cooling margin).
pub const SUPPLY_AIR_K: f64 = 288.15;

/// Per-module heat dissipation, W (GENERIC: ARINC 600-size avionics LRU
/// dissipation commonly falls in the 50-300 W range in public thermal
/// design guidance for airborne electronics; a CPIOM, doing the heavier
/// computation, is taken at the upper end, an IOM — mostly I/O — lower).
pub const CPIOM_HEAT_W: f64 = 150.0;
pub const IOM_HEAT_W: f64 = 80.0;

/// Bay-to-air conductance with the forced draught fully established, W/K
/// (GENERIC, sized so a bay dissipating a few hundred watts settles a few
/// tens of K above supply air — typical of forced-air avionics cooling).
const FORCED_CONDUCTANCE_W_K: f64 = 12.0;
/// Bay-to-air conductance with no forced draught at all — free convection
/// in the still bay air only, W/K (GENERIC: an order of magnitude below
/// forced, as free convection typically is relative to forced).
const NATURAL_CONDUCTANCE_W_K: f64 = 1.5;
/// Bay thermal capacitance (rack structure + the air it holds), J/K
/// (GENERIC).
const BAY_CAPACITY_J_K: f64 = 25_000.0;

/// One extraction/circulation fan serving a bay.
#[derive(Clone, Copy, Debug, Default)]
pub struct FanFaults {
    /// 0 healthy .. 1 fully failed (seized bearing, burnt winding): the
    /// fan moves this much less air than commanded.
    pub failure: f64,
}
#[derive(Clone, Copy, Debug)]
pub struct Fan {
    pub powered: bool,
    pub faults: FanFaults,
}
impl Fan {
    pub fn new() -> Self {
        Self { powered: true, faults: FanFaults::default() }
    }

    /// This fan's own contribution to establishing the draught, 0..1.
    fn output_frac(&self) -> f64 {
        if self.powered {
            1.0 - self.faults.failure.clamp(0.0, 1.0)
        } else {
            0.0
        }
    }
}
impl Default for Fan {
    fn default() -> Self {
        Self::new()
    }
}

/// The extract valve ducting bay air overboard/to the mix manifold.
#[derive(Clone, Copy, Debug, Default)]
pub struct ExtractValveFaults {
    /// 0 healthy (follows command) .. 1 stuck fully closed regardless of
    /// command, blocking the draught even with a healthy fan.
    pub stuck_closed: f64,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct BayState {
    pub temp_k: f64,
    pub conductance_w_k: f64,
    /// 0 no forced draught at all .. 1 fully established.
    pub airflow_frac: f64,
}

/// One avionics bay: a single thermal-capacitance node its modules all
/// share (matching `topology::EndSystemSpec::bay` grouping — every module
/// naming the same bay string is assumed to sit in this one compartment).
pub struct Bay {
    temp_k: f64,
}
impl Bay {
    pub fn new(initial_k: f64) -> Self {
        Self { temp_k: initial_k }
    }

    pub fn temp_k(&self) -> f64 {
        self.temp_k
    }

    /// One step. `heat_w` is the sum of every currently-dissipating
    /// module's heat in this bay (an unpowered module dissipates nothing —
    /// callers zero it out, this model does not know which modules are
    /// powered). `fans` are every fan serving this bay: real avionics bays
    /// commonly run two fans together for margin rather than holding one
    /// as a cold standby, so their contributions combine as the *best*
    /// one running (one working fan alone still establishes the draught;
    /// a second failing one does not multiply the loss), not a sum.
    /// `valve_open` is the extract valve's commanded position, 0..1,
    /// degraded toward closed by `valve_faults`.
    pub fn step(&mut self, heat_w: f64, fans: &[Fan], valve_open: f64, valve_faults: &ExtractValveFaults, dt_s: f64) -> BayState {
        let fan_frac = fans.iter().map(|f| f.output_frac()).fold(0.0_f64, f64::max);
        let valve_frac = (valve_open.clamp(0.0, 1.0) * (1.0 - valve_faults.stuck_closed.clamp(0.0, 1.0))).clamp(0.0, 1.0);
        // Fan and valve are in series in the one duct: both have to pass
        // air for the forced draught to exist at all.
        let airflow_frac = fan_frac * valve_frac;
        let conductance = NATURAL_CONDUCTANCE_W_K + airflow_frac * (FORCED_CONDUCTANCE_W_K - NATURAL_CONDUCTANCE_W_K);
        let dt = dt_s.max(0.0);
        let target = SUPPLY_AIR_K + heat_w.max(0.0) / conductance.max(1e-6);
        let k = conductance / BAY_CAPACITY_J_K;
        self.temp_k = target + (self.temp_k - target) * (-k * dt).exp();
        BayState { temp_k: self.temp_k, conductance_w_k: conductance, airflow_frac }
    }
}

/// A module's own thermal supervisor: trips (ramps toward 1, not a
/// discontinuous snap) once its local bay air is hot enough for long
/// enough, and un-trips the same way once it cools — this is the number
/// handed to `faults::ModuleFaults::overheat_trip_frac`.
///
/// Trip threshold, K: where the thermal supervisor is fully tripped.
///
/// The ramp's **lower end** is now sourced. The A380's avionics bays are
/// pressurised, temperature-controlled locations, which is RTCA DO-160
/// **Temperature and Altitude Category A1**: "equipment intended for
/// installation in a controlled temperature and pressurized location"
/// whose pressures are "normally no lower than the altitude equivalent of
/// 15,000 ft", qualified over an operating range of **-15 C to +55 C**
/// (DO-160 section 4, category A1). +55 C is thus the highest ambient the
/// equipment is qualified to run at indefinitely, and is exactly where this
/// model starts degrading the module: `TRIP_TEMP_K - TRIP_SPAN_K = 70 - 15
/// = 55 C`.
///
/// The ramp's **upper end**, 70 C, is where the supervisor is taken to have
/// tripped completely. That is the commonly quoted short-time / survival
/// ceiling for the same category, but DO-160G's Table 4-1 text could not be
/// obtained from any public source to confirm it against category A1's own
/// short-time operating high temperature, so it stays **GENERIC**.
/// Searched: DO-160G section 4 category tables, test-house summaries of
/// them, FAA AC 21-16G. Only the -15/+55 operating pair is reproduced
/// publicly.
pub const TRIP_TEMP_K: f64 = 273.15 + 70.0;
/// Span over which the trip ramps 0..1 rather than snapping, K. No longer a
/// free parameter: it is set so the ramp *begins* at DO-160 category A1's
/// +55 C operating high temperature (see [`TRIP_TEMP_K`]) -- the module
/// starts to be affected exactly where it stops being qualified.
const TRIP_SPAN_K: f64 = 15.0;
/// Thermal-supervisor response time constant, s (GENERIC: fast enough to
/// protect the electronics without chattering on a brief transient).
const TRIP_TIME_CONSTANT_S: f64 = 5.0;

#[derive(Clone, Copy, Debug, Default)]
pub struct OverheatTrip {
    frac: f64,
}
impl OverheatTrip {
    pub fn frac(&self) -> f64 {
        self.frac
    }

    /// How close to its target the ramp has to get before it is taken to
    /// have arrived. An exponential approach never reaches its target
    /// exactly, and `faults::ModuleFaults::is_available` tests
    /// `overheat_trip_frac < 1.0`: without this, a module baking in a
    /// bay well past its trip temperature would sit at 0.999... for ever
    /// and never actually drop off the network, which is the one thing
    /// this supervisor exists to make happen. A real thermal supervisor
    /// is a comparator that latches when it trips, not an asymptote.
    const ARRIVED_EPSILON: f64 = 1e-6;

    pub fn step(&mut self, bay_temp_k: f64, dt_s: f64) -> f64 {
        let target = ((bay_temp_k - TRIP_TEMP_K) / TRIP_SPAN_K).clamp(0.0, 1.0);
        let dt = dt_s.max(0.0);
        self.frac = target + (self.frac - target) * (-dt / TRIP_TIME_CONSTANT_S).exp();
        if (self.frac - target).abs() < Self::ARRIVED_EPSILON {
            self.frac = target;
        }
        self.frac
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn settle(bay: &mut Bay, heat_w: f64, fans: &[Fan], valve_open: f64, valve_faults: &ExtractValveFaults, seconds: f64) -> BayState {
        let dt = 1.0;
        let mut out = BayState::default();
        for _ in 0..(seconds / dt) as usize {
            out = bay.step(heat_w, fans, valve_open, valve_faults, dt);
        }
        out
    }

    #[test]
    fn healthy_forced_draught_settles_close_to_supply_air() {
        let mut bay = Bay::new(SUPPLY_AIR_K);
        let fans = [Fan::new(), Fan::new()];
        let out = settle(&mut bay, 2.0 * CPIOM_HEAT_W, &fans, 1.0, &ExtractValveFaults::default(), 5000.0);
        assert_eq!(out.airflow_frac, 1.0);
        let rise_k = out.temp_k - SUPPLY_AIR_K;
        assert!(rise_k > 0.0 && rise_k < 40.0, "rise {rise_k:.1} K");
    }

    #[test]
    fn losing_every_fan_collapses_to_natural_convection_and_runs_much_hotter() {
        let mut bay = Bay::new(SUPPLY_AIR_K);
        let fans = [Fan { powered: true, faults: FanFaults { failure: 1.0 } }, Fan { powered: true, faults: FanFaults { failure: 1.0 } }];
        let out = settle(&mut bay, 2.0 * CPIOM_HEAT_W, &fans, 1.0, &ExtractValveFaults::default(), 20_000.0);
        assert_eq!(out.airflow_frac, 0.0);
        let forced = SUPPLY_AIR_K + 2.0 * CPIOM_HEAT_W / FORCED_CONDUCTANCE_W_K;
        assert!(out.temp_k > forced, "natural convection ({:.1} K) should run hotter than forced ({:.1} K)", out.temp_k, forced);
    }

    #[test]
    fn one_healthy_fan_out_of_two_still_establishes_full_airflow() {
        let mut bay = Bay::new(SUPPLY_AIR_K);
        let fans = [Fan::new(), Fan { powered: false, faults: FanFaults::default() }];
        let out = bay.step(CPIOM_HEAT_W, &fans, 1.0, &ExtractValveFaults::default(), 1.0);
        assert_eq!(out.airflow_frac, 1.0);
    }

    #[test]
    fn a_stuck_closed_extract_valve_blocks_the_draught_even_with_healthy_fans() {
        let mut bay = Bay::new(SUPPLY_AIR_K);
        let fans = [Fan::new(), Fan::new()];
        let out = settle(&mut bay, 2.0 * CPIOM_HEAT_W, &fans, 1.0, &ExtractValveFaults { stuck_closed: 1.0 }, 20_000.0);
        assert_eq!(out.airflow_frac, 0.0);
    }

    #[test]
    fn overheat_trip_stays_untripped_below_threshold_and_ramps_above_it() {
        let mut trip = OverheatTrip::default();
        for _ in 0..100 {
            trip.step(TRIP_TEMP_K - 10.0, 1.0);
        }
        assert_eq!(trip.frac(), 0.0);
        for _ in 0..200 {
            trip.step(TRIP_TEMP_K + TRIP_SPAN_K, 1.0);
        }
        assert!(trip.frac() > 0.99, "trip fraction {}", trip.frac());
    }

    #[test]
    fn no_nan_at_zero_dt_or_at_rest() {
        let mut bay = Bay::new(SUPPLY_AIR_K);
        let out = bay.step(0.0, &[], 0.0, &ExtractValveFaults::default(), 0.0);
        assert!(!out.temp_k.is_nan());
        let mut trip = OverheatTrip::default();
        assert!(!trip.step(0.0, 0.0).is_nan());
    }
}

#[cfg(test)]
mod trip_tests {
    use super::*;

    #[test]
    fn a_sustained_overheat_trips_fully_rather_than_approaching_one_for_ever() {
        let mut trip = OverheatTrip::default();
        // Well past the trip temperature plus the whole ramp span: the
        // target is 1.0 and the supervisor must actually get there, since
        // `faults::ModuleFaults::is_available` tests `< 1.0`.
        let mut t = 0.0;
        while trip.frac() < 1.0 && t < 600.0 {
            trip.step(TRIP_TEMP_K + 2.0 * TRIP_SPAN_K, 0.5);
            t += 0.5;
        }
        assert_eq!(trip.frac(), 1.0, "still ramping after {t} s");
        // And it comes back the same way once the bay is cool again.
        let mut t = 0.0;
        while trip.frac() > 0.0 && t < 600.0 {
            trip.step(SUPPLY_AIR_K, 0.5);
            t += 0.5;
        }
        assert_eq!(trip.frac(), 0.0, "still un-tripping after {t} s");
    }
}
