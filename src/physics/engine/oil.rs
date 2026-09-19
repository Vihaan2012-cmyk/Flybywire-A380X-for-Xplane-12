//! The engine oil system: tank, pressure pump, pressure relief valve,
//! filter with its bypass valve, the three bearing chambers, scavenge, and
//! the two coolers (fuel-cooled and air-cooled).
//!
//! Pressure is not a curve: a positive-displacement pump driven off the HP
//! spool pushes a flow set by its speed through the filter, the lines and
//! the bearing jets. The lines and filter resist viscously (so cold oil runs
//! high pressure and the relief valve opens), the jets as orifices. A
//! clogged filter backs pressure up until its bypass valve opens; an
//! unhealthy pump (the damage model's leak/starvation/pump-fault fraction)
//! delivers less.
//!
//! Temperature is a heat balance per bearing chamber: the oil jetted in
//! carries away the bearings' friction heat and whatever soaks in from the
//! chamber's surroundings (the front chamber from compressor air, the HP/IP
//! chamber from the hot section's metal, the tail chamber from the exhaust).
//! With the pump stopped, that soak-back keeps heating the chambers after
//! shutdown. Scavenged oil returns to the tank and is cooled on its way back
//! out by the fuel-cooled oil cooler, which heats the fuel going to the
//! burners, and by the air-cooled oil cooler, which the EEC opens when the
//! fuel would get too hot or the oil is running hot.
//!
//! Oil properties are public (MIL-PRF-23699 turbine oil, e.g. Mobil Jet Oil
//! II's data sheet). No Trent 900 oil-system data is public: pump size,
//! resistances, chamber and cooler sizes and the ACOC schedule are GENERIC,
//! chosen so the pressures meet EASA.E.012's minimums with margin and the
//! heat loads sit where large turbofans typically run.

use super::gas::CP_AIR;

pub const PSI_PA: f64 = 6894.757;
/// MIL-PRF-23699 oil density, kg/m^3, and specific heat, J/(kg K).
const OIL_DENSITY: f64 = 1000.0;
const OIL_CP: f64 = 1950.0;
const FUEL_CP: f64 = 2010.0;

/// Kinematic viscosity, cSt, by the Walther (ASTM D341) relation fitted
/// through Mobil Jet Oil II's 27.6 cSt at 40 C and 5.1 cSt at 100 C.
pub fn viscosity_cst(temp_k: f64) -> f64 {
    const A: f64 = 9.3116;
    const B: f64 = 3.6661;
    let t = temp_k.clamp(200.0, 600.0);
    10f64.powf(10f64.powf(A - B * t.log10())) - 0.7
}

/// Pump delivery at 100% N3, m^3/s (GENERIC: 150 L/min).
const PUMP_DESIGN_M3_S: f64 = 2.5e-3;
/// At the design flow with oil at 100 C: jet (orifice) pressure drop and
/// the viscous drops of the lines and the filter, psi (GENERIC).
const JET_DROP_DESIGN_PSI: f64 = 80.0;
const LINE_DROP_DESIGN_PSI: f64 = 12.0;
const FILTER_DROP_DESIGN_PSI: f64 = 4.0;
/// The filter bypass valve's cracking differential, psi (GENERIC).
const FILTER_BYPASS_PSI: f64 = 30.0;
/// Pressure relief valve cracking pressure, psi, and the rise above it at
/// which it passes the whole design flow (GENERIC; the package's engines.cfg
/// gauge tops out at 149 psi).
const RELIEF_CRACK_PSI: f64 = 145.0;
const RELIEF_FULL_FLOW_RISE_PSI: f64 = 10.0;

/// The three bearing chambers: share of the jet flow, share of the
/// bearings' friction heat, thermal capacity (housing metal and the oil it
/// holds), J/K, and conductance to its surroundings, W/K (GENERIC).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Chamber {
    /// Fan and IP compressor front bearings.
    Front,
    /// HP and IP turbine bearings, inside the core next to the combustor.
    HpIp,
    /// LP turbine rear bearing, in the exhaust.
    Tail,
}
pub const CHAMBERS: [Chamber; 3] = [Chamber::Front, Chamber::HpIp, Chamber::Tail];
const FLOW_SHARE: [f64; 3] = [0.35, 0.40, 0.25];
const HEAT_SHARE: [f64; 3] = [0.35, 0.45, 0.20];
const CHAMBER_CAPACITY_J_K: [f64; 3] = [20_000.0, 19_000.0, 12_000.0];
const CHAMBER_SOAK_W_K: [f64; 3] = [40.0, 8.0, 10.0];

/// Tank: oil held, kg, and loss to the nacelle, W/K (GENERIC).
const TANK_OIL_KG: f64 = 20.0;
const TANK_LOSS_W_K: f64 = 15.0;
/// Fuel-cooled oil cooler effectiveness (GENERIC).
const FCOC_EFFECTIVENESS: f64 = 0.8;
/// Air-cooled oil cooler: effectiveness, the fan air it takes fully open as
/// a fraction of the bypass flow, and the EEC's schedule: start opening at
/// these fuel and oil temperatures, fully open this far above (GENERIC).
const ACOC_EFFECTIVENESS: f64 = 0.7;
const ACOC_AIR_FRACTION: f64 = 0.004;
const ACOC_FUEL_OPEN_K: f64 = 273.15 + 110.0;
const ACOC_OIL_OPEN_K: f64 = 273.15 + 120.0;
const ACOC_SPAN_K: f64 = 15.0;

/// Faults the oil system can carry, each a fraction 0 (healthy) .. 1.
#[derive(Clone, Copy, Debug, Default)]
pub struct OilFaults {
    /// Filter element blocked with debris: its viscous resistance grows as
    /// `1 / (1 - clog)^2`.
    pub filter_clog: f64,
}

/// Where the oil exchanges heat with the engine this frame.
#[derive(Clone, Copy, Debug)]
pub struct Surroundings {
    /// HP spool speed as a fraction of 100%.
    pub n3_frac: f64,
    /// The damage model's pump delivery fraction (leak, starvation, pump
    /// fault): 1 healthy.
    pub pump_fraction: f64,
    /// Total bearing and gearbox friction heat going into the oil, W.
    pub friction_w: f64,
    /// Front chamber's surroundings (IP compressor delivery air), the hot
    /// section's metal, and the exhaust gas behind the LP turbine, K.
    pub front_air_k: f64,
    pub hot_metal_k: f64,
    pub exhaust_k: f64,
    /// Fuel through the FCOC, kg/s, and its temperature arriving, K.
    pub fuel_kg_s: f64,
    pub fuel_k: f64,
    /// Fan bypass flow, kg/s, and its temperature, K, for the ACOC.
    pub bypass_kg_s: f64,
    pub fan_air_k: f64,
    pub nacelle_k: f64,
    pub dt_s: f64,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct OilState {
    /// Indicated oil pressure at the bearing feed manifold, psi above the
    /// chamber vents.
    pub pressure_psi: f64,
    /// Indicated oil temperature (the tank), K.
    pub temp_k: f64,
    /// Oil supplied to the chambers after both coolers, K.
    pub supply_k: f64,
    pub chamber_k: [f64; 3],
    /// Oil flow to the bearing jets, m^3/s.
    pub jet_flow_m3_s: f64,
    pub filter_bypassed: bool,
    pub relief_open: bool,
    /// Heat the FCOC put into the fuel, W, and the fuel's temperature
    /// leaving it, K.
    pub fuel_heat_w: f64,
    pub fuel_out_k: f64,
    /// ACOC valve position, 0..1.
    pub acoc_open: f64,
}

#[derive(Clone, Copy, Debug)]
pub struct OilSystem {
    tank_k: f64,
    chamber_k: [f64; 3],
}

impl OilSystem {
    pub fn new(temp_k: f64) -> Self {
        Self { tank_k: temp_k, chamber_k: [temp_k; 3] }
    }

    pub fn tank_k(&self) -> f64 {
        self.tank_k
    }

    /// The flow reaching the jets and the pressures that go with it, for a
    /// pump delivering `pump_m3_s` of oil at `temp_k`.
    fn hydraulics(pump_m3_s: f64, temp_k: f64, faults: &OilFaults) -> (f64, f64, bool, bool) {
        let viscosity_ratio = viscosity_cst(temp_k) / viscosity_cst(373.15);
        let clog = faults.filter_clog.clamp(0.0, 0.999);
        let q_ref = PUMP_DESIGN_M3_S;
        let line = |q: f64| LINE_DROP_DESIGN_PSI * viscosity_ratio * q / q_ref;
        let jets = |q: f64| JET_DROP_DESIGN_PSI * (q / q_ref).powi(2);
        let filter = |q: f64| (FILTER_DROP_DESIGN_PSI * viscosity_ratio * q / q_ref / (1.0 - clog).powi(2)).min(FILTER_BYPASS_PSI);
        let pump_psi = |q: f64| filter(q) + line(q) + jets(q);
        let relief = |p: f64| PUMP_DESIGN_M3_S * ((p - RELIEF_CRACK_PSI) / RELIEF_FULL_FLOW_RISE_PSI).max(0.0);
        // Flow to the jets plus flow through the relief valve is what the
        // pump delivers; both grow with the jet flow, so bisect on it.
        let (mut lo, mut hi) = (0.0, pump_m3_s.max(0.0));
        for _ in 0..48 {
            let q = 0.5 * (lo + hi);
            if q + relief(pump_psi(q)) > pump_m3_s {
                hi = q;
            } else {
                lo = q;
            }
        }
        let q = lo;
        let bypassed = FILTER_DROP_DESIGN_PSI * viscosity_ratio * q / q_ref / (1.0 - clog).powi(2) > FILTER_BYPASS_PSI;
        (q, line(q) + jets(q), bypassed, pump_psi(q) > RELIEF_CRACK_PSI)
    }

    pub fn step(&mut self, s: &Surroundings, faults: &OilFaults) -> OilState {
        let dt = s.dt_s.max(0.0);
        let pump = PUMP_DESIGN_M3_S * s.n3_frac.max(0.0) * s.pump_fraction.clamp(0.0, 1.0);

        // ---- Coolers, tank to chambers.
        let oil_capacity = pump * OIL_DENSITY * OIL_CP; // W/K through the pump
        let fuel_capacity = s.fuel_kg_s.max(0.0) * FUEL_CP;
        let fcoc_w = FCOC_EFFECTIVENESS * oil_capacity.min(fuel_capacity) * (self.tank_k - s.fuel_k);
        let after_fcoc = if oil_capacity > 0.0 { self.tank_k - fcoc_w / oil_capacity } else { self.tank_k };
        let fuel_out_k = if fuel_capacity > 0.0 { s.fuel_k + fcoc_w / fuel_capacity } else { s.fuel_k };
        let open = |t: f64, from: f64| ((t - from) / ACOC_SPAN_K).clamp(0.0, 1.0);
        let acoc_open = open(fuel_out_k, ACOC_FUEL_OPEN_K).max(open(after_fcoc, ACOC_OIL_OPEN_K));
        let air_capacity = acoc_open * ACOC_AIR_FRACTION * s.bypass_kg_s.max(0.0) * CP_AIR;
        let acoc_w = ACOC_EFFECTIVENESS * oil_capacity.min(air_capacity) * (after_fcoc - s.fan_air_k);
        let supply_k = if oil_capacity > 0.0 { after_fcoc - acoc_w / oil_capacity } else { self.tank_k };

        // ---- Hydraulics.
        let (jet_flow, pressure_psi, filter_bypassed, relief_open) = Self::hydraulics(pump, supply_k, faults);

        // ---- Chambers: each an exact first-order step toward the balance
        // of the oil jetted in, its friction heat and its surroundings.
        let surround = [s.front_air_k, s.hot_metal_k, s.exhaust_k];
        let mut scavenge_w_k = 0.0;
        let mut scavenge_k_w_k = 0.0;
        for i in 0..3 {
            let flow_w_k = jet_flow * FLOW_SHARE[i] * OIL_DENSITY * OIL_CP;
            let soak = CHAMBER_SOAK_W_K[i];
            let heat = s.friction_w.max(0.0) * HEAT_SHARE[i];
            let conductance = flow_w_k + soak;
            let target = (flow_w_k * supply_k + soak * surround[i] + heat) / conductance.max(1e-9);
            let k = conductance / CHAMBER_CAPACITY_J_K[i];
            self.chamber_k[i] = target + (self.chamber_k[i] - target) * (-k * dt).exp();
            scavenge_w_k += flow_w_k;
            scavenge_k_w_k += flow_w_k * self.chamber_k[i];
        }
        // Oil the relief valve spilled goes straight back to the tank at
        // supply temperature.
        let spilled_w_k = (pump - jet_flow).max(0.0) * OIL_DENSITY * OIL_CP;
        let returning_w_k = scavenge_w_k + spilled_w_k;
        let returning_k = if returning_w_k > 0.0 { (scavenge_k_w_k + spilled_w_k * supply_k) / returning_w_k } else { self.tank_k };

        // ---- Tank: mixing with what returns, losing heat to the nacelle.
        let tank_capacity = TANK_OIL_KG * OIL_CP;
        let conductance = returning_w_k + TANK_LOSS_W_K;
        let target = (returning_w_k * returning_k + TANK_LOSS_W_K * s.nacelle_k) / conductance;
        self.tank_k = target + (self.tank_k - target) * (-conductance / tank_capacity * dt).exp();

        OilState {
            pressure_psi,
            temp_k: self.tank_k,
            supply_k,
            chamber_k: self.chamber_k,
            jet_flow_m3_s: jet_flow,
            filter_bypassed,
            relief_open,
            fuel_heat_w: fcoc_w,
            fuel_out_k,
            acoc_open,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn surroundings(n3_frac: f64) -> Surroundings {
        Surroundings {
            n3_frac,
            pump_fraction: 1.0,
            friction_w: 190_000.0 * n3_frac.powi(3),
            front_air_k: 288.0 + 150.0 * n3_frac,
            hot_metal_k: 288.0 + 800.0 * n3_frac,
            exhaust_k: 288.0 + 500.0 * n3_frac,
            fuel_kg_s: 2.0 * n3_frac.powi(3),
            fuel_k: 288.0,
            bypass_kg_s: 1000.0 * n3_frac,
            fan_air_k: 300.0,
            nacelle_k: 288.0,
            dt_s: 0.1,
        }
    }

    fn settle(oil: &mut OilSystem, s: &Surroundings, faults: &OilFaults, seconds: f64) -> OilState {
        let mut out = OilState::default();
        for _ in 0..(seconds / s.dt_s) as usize {
            out = oil.step(s, faults);
        }
        out
    }

    #[test]
    fn the_walther_fit_reproduces_its_two_data_points() {
        assert!((viscosity_cst(313.15) - 27.6).abs() < 0.1);
        assert!((viscosity_cst(373.15) - 5.1).abs() < 0.05);
        assert!(viscosity_cst(233.15) > 5_000.0);
    }

    #[test]
    fn warm_oil_pressure_meets_the_data_sheet_minimums() {
        // EASA.E.012: 25 psi from idle to 70% HP, 50 psi above 95% HP.
        let (_, idle, _, _) = OilSystem::hydraulics(PUMP_DESIGN_M3_S * 0.62, 363.15, &OilFaults::default());
        let (_, high, _, _) = OilSystem::hydraulics(PUMP_DESIGN_M3_S * 0.96, 363.15, &OilFaults::default());
        assert!(idle > 25.0 && high > 50.0, "idle {idle:.1} psi, high {high:.1} psi");
    }

    #[test]
    fn cold_oil_opens_the_relief_valve_and_bypasses_the_filter() {
        let (_, p, bypassed, relief) = OilSystem::hydraulics(PUMP_DESIGN_M3_S * 0.62, 263.15, &OilFaults::default());
        assert!(relief && bypassed, "{p:.1} psi");
        assert!(p > 100.0);
    }

    #[test]
    fn a_clogging_filter_opens_its_bypass() {
        let clean = OilSystem::hydraulics(PUMP_DESIGN_M3_S, 363.15, &OilFaults::default());
        let clogged = OilSystem::hydraulics(PUMP_DESIGN_M3_S, 363.15, &OilFaults { filter_clog: 0.8 });
        assert!(!clean.2 && clogged.2);
    }

    #[test]
    fn running_it_settles_well_below_the_limit_and_heats_the_fuel() {
        let mut oil = OilSystem::new(288.0);
        let out = settle(&mut oil, &surroundings(0.97), &OilFaults::default(), 1800.0);
        let c = out.temp_k - 273.15;
        assert!(c > 50.0 && c < 196.0, "oil {c:.1} C");
        assert!(out.fuel_out_k > 288.0 && out.fuel_heat_w > 0.0);
    }

    #[test]
    fn after_shutdown_the_hp_ip_chamber_soaks_back_hotter_than_it_ran() {
        let mut oil = OilSystem::new(288.0);
        let running = settle(&mut oil, &surroundings(0.97), &OilFaults::default(), 1800.0);
        let stopped = Surroundings { n3_frac: 0.0, friction_w: 0.0, fuel_kg_s: 0.0, bypass_kg_s: 0.0, front_air_k: 288.0, exhaust_k: 400.0, hot_metal_k: 1000.0, ..surroundings(0.0) };
        let soaked = settle(&mut oil, &stopped, &OilFaults::default(), 900.0);
        assert!(soaked.chamber_k[1] > running.chamber_k[1] + 20.0, "ran {:.0} K, soaked to {:.0} K", running.chamber_k[1], soaked.chamber_k[1]);
        assert_eq!(soaked.pressure_psi, 0.0);
    }

    #[test]
    fn a_failing_pump_loses_pressure() {
        let mut oil = OilSystem::new(363.15);
        let s = Surroundings { pump_fraction: 0.3, ..surroundings(0.97) };
        let weak = oil.step(&s, &OilFaults::default());
        let healthy = OilSystem::new(363.15).step(&surroundings(0.97), &OilFaults::default());
        assert!(weak.pressure_psi < 0.3 * healthy.pressure_psi);
    }
}
