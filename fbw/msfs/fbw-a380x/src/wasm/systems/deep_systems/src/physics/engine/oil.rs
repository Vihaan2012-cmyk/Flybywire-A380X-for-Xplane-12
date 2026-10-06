use super::gas::CP_AIR;

pub const PSI_PA: f64 = 6894.757;
const OIL_DENSITY: f64 = 1000.0;
const OIL_CP: f64 = 1950.0;
const FUEL_CP: f64 = 2010.0;

pub fn viscosity_cst(temp_k: f64) -> f64 {
    const A: f64 = 9.3116;
    const B: f64 = 3.6661;
    let t = temp_k.clamp(200.0, 600.0);
    10f64.powf(10f64.powf(A - B * t.log10())) - 0.7
}

const PUMP_DESIGN_M3_S: f64 = 2.5e-3;
const JET_DROP_DESIGN_PSI: f64 = 80.0;
const LINE_DROP_DESIGN_PSI: f64 = 12.0;
const FILTER_DROP_DESIGN_PSI: f64 = 4.0;
const FILTER_BYPASS_PSI: f64 = 30.0;
const RELIEF_CRACK_PSI: f64 = 145.0;
const RELIEF_FULL_FLOW_RISE_PSI: f64 = 10.0;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Chamber {
    Front,
    HpIp,
    Tail,
}
pub const CHAMBERS: [Chamber; 3] = [Chamber::Front, Chamber::HpIp, Chamber::Tail];
const FLOW_SHARE: [f64; 3] = [0.35, 0.40, 0.25];
const HEAT_SHARE: [f64; 3] = [0.35, 0.45, 0.20];
const CHAMBER_CAPACITY_J_K: [f64; 3] = [20_000.0, 19_000.0, 12_000.0];
const CHAMBER_SOAK_W_K: [f64; 3] = [40.0, 8.0, 10.0];

const TANK_OIL_KG: f64 = 20.0;
const TANK_LOSS_W_K: f64 = 15.0;
pub const TANK_CAPACITY_M3: f64 = TANK_OIL_KG / OIL_DENSITY;
const TANK_RESIDUAL_KG: f64 = 0.5;

const SEAL_LOSS_L_PER_H_AT_DESIGN_FLOW: f64 = 0.3;
const SEAL_LOSS_FRACTION_OF_JET_FLOW: f64 = SEAL_LOSS_L_PER_H_AT_DESIGN_FLOW * 1e-3 / 3600.0 / PUMP_DESIGN_M3_S;

const LEAK_FULL_DRAIN_S: f64 = 360.0;
const LEAK_FULL_SCALE_M3_S: f64 = TANK_CAPACITY_M3 / LEAK_FULL_DRAIN_S;
const LEAK_REFERENCE_PSI: f64 = 80.0;

const PUMP_INLET_UNCOVERS_FRACTION: f64 = 0.15;
const FCOC_EFFECTIVENESS: f64 = 0.8;
const ACOC_EFFECTIVENESS: f64 = 0.7;
const ACOC_AIR_FRACTION: f64 = 0.004;
const ACOC_FUEL_OPEN_K: f64 = 273.15 + 110.0;
const ACOC_OIL_OPEN_K: f64 = 273.15 + 120.0;
const ACOC_SPAN_K: f64 = 15.0;

#[derive(Clone, Copy, Debug, Default)]
pub struct OilFaults {
    pub filter_clog: f64,
    pub leak: f64,
}

#[derive(Clone, Copy, Debug)]
pub struct Surroundings {
    pub n3_frac: f64,
    pub pump_fraction: f64,
    pub friction_w: f64,
    pub front_air_k: f64,
    pub hot_metal_k: f64,
    pub exhaust_k: f64,
    pub fuel_kg_s: f64,
    pub fuel_k: f64,
    pub bypass_kg_s: f64,
    pub fan_air_k: f64,
    pub nacelle_k: f64,
    pub dt_s: f64,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct OilState {
    pub pressure_psi: f64,
    pub temp_k: f64,
    pub supply_k: f64,
    pub chamber_k: [f64; 3],
    pub jet_flow_m3_s: f64,
    pub filter_bypassed: bool,
    pub relief_open: bool,
    pub fuel_heat_w: f64,
    pub fuel_out_k: f64,
    pub acoc_open: f64,
    pub quantity_m3: f64,
    pub quantity_fraction: f64,
    pub seal_loss_m3_s: f64,
    pub leak_m3_s: f64,
    pub pump_prime_fraction: f64,
}

#[derive(Clone, Copy, Debug)]
pub struct OilSystem {
    tank_k: f64,
    chamber_k: [f64; 3],
    oil_m3: f64,
}

impl OilSystem {
    pub fn new(temp_k: f64) -> Self {
        Self { tank_k: temp_k, chamber_k: [temp_k; 3], oil_m3: TANK_CAPACITY_M3 }
    }

    pub fn tank_k(&self) -> f64 {
        self.tank_k
    }

    pub fn oil_m3(&self) -> f64 {
        self.oil_m3
    }

    pub fn quantity_fraction(&self) -> f64 {
        (self.oil_m3 / TANK_CAPACITY_M3).clamp(0.0, 1.0)
    }

    fn prime_fraction(quantity_fraction: f64) -> f64 {
        (quantity_fraction / PUMP_INLET_UNCOVERS_FRACTION).clamp(0.0, 1.0)
    }

    fn hydraulics(pump_m3_s: f64, temp_k: f64, faults: &OilFaults) -> (f64, f64, bool, bool) {
        let viscosity_ratio = viscosity_cst(temp_k) / viscosity_cst(373.15);
        let clog = faults.filter_clog.clamp(0.0, 0.999);
        let q_ref = PUMP_DESIGN_M3_S;
        let line = |q: f64| LINE_DROP_DESIGN_PSI * viscosity_ratio * q / q_ref;
        let jets = |q: f64| JET_DROP_DESIGN_PSI * (q / q_ref).powi(2);
        let filter = |q: f64| (FILTER_DROP_DESIGN_PSI * viscosity_ratio * q / q_ref / (1.0 - clog).powi(2)).min(FILTER_BYPASS_PSI);
        let pump_psi = |q: f64| filter(q) + line(q) + jets(q);
        let relief = |p: f64| PUMP_DESIGN_M3_S * ((p - RELIEF_CRACK_PSI) / RELIEF_FULL_FLOW_RISE_PSI).max(0.0);
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
        let quantity_fraction = self.quantity_fraction();
        let prime = Self::prime_fraction(quantity_fraction);
        let pump = PUMP_DESIGN_M3_S * s.n3_frac.max(0.0) * s.pump_fraction.clamp(0.0, 1.0) * prime;

        let oil_capacity = pump * OIL_DENSITY * OIL_CP;
        let fuel_capacity = s.fuel_kg_s.max(0.0) * FUEL_CP;
        let fcoc_w = FCOC_EFFECTIVENESS * oil_capacity.min(fuel_capacity) * (self.tank_k - s.fuel_k);
        let after_fcoc = if oil_capacity > 0.0 { self.tank_k - fcoc_w / oil_capacity } else { self.tank_k };
        let fuel_out_k = if fuel_capacity > 0.0 { s.fuel_k + fcoc_w / fuel_capacity } else { s.fuel_k };
        let open = |t: f64, from: f64| ((t - from) / ACOC_SPAN_K).clamp(0.0, 1.0);
        let acoc_open = open(fuel_out_k, ACOC_FUEL_OPEN_K).max(open(after_fcoc, ACOC_OIL_OPEN_K));
        let air_capacity = acoc_open * ACOC_AIR_FRACTION * s.bypass_kg_s.max(0.0) * CP_AIR;
        let acoc_w = ACOC_EFFECTIVENESS * oil_capacity.min(air_capacity) * (after_fcoc - s.fan_air_k);
        let supply_k = if oil_capacity > 0.0 { after_fcoc - acoc_w / oil_capacity } else { self.tank_k };

        let (jet_flow, pressure_psi, filter_bypassed, relief_open) = Self::hydraulics(pump, supply_k, faults);

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
        let spilled_w_k = (pump - jet_flow).max(0.0) * OIL_DENSITY * OIL_CP;
        let returning_w_k = scavenge_w_k + spilled_w_k;
        let returning_k = if returning_w_k > 0.0 { (scavenge_k_w_k + spilled_w_k * supply_k) / returning_w_k } else { self.tank_k };

        let seal_loss_m3_s = SEAL_LOSS_FRACTION_OF_JET_FLOW * jet_flow;
        let leak = faults.leak.clamp(0.0, 1.0);
        let leak_m3_s = leak * LEAK_FULL_SCALE_M3_S * (pressure_psi.max(0.0) / LEAK_REFERENCE_PSI).sqrt();
        let wanted_m3 = (seal_loss_m3_s + leak_m3_s) * dt;
        let taken_m3 = wanted_m3.min(self.oil_m3.max(0.0));
        self.oil_m3 = (self.oil_m3 - taken_m3).clamp(0.0, TANK_CAPACITY_M3);

        let tank_capacity = (self.oil_m3 * OIL_DENSITY).max(TANK_RESIDUAL_KG) * OIL_CP;
        let conductance = returning_w_k + TANK_LOSS_W_K;
        let target = (returning_w_k * returning_k + TANK_LOSS_W_K * s.nacelle_k) / conductance;
        self.tank_k = target + (self.tank_k - target) * (-conductance / tank_capacity * dt).exp();

        OilState {
            quantity_m3: self.oil_m3,
            quantity_fraction: self.quantity_fraction(),
            seal_loss_m3_s,
            leak_m3_s,
            pump_prime_fraction: prime,
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
        let clogged = OilSystem::hydraulics(PUMP_DESIGN_M3_S, 363.15, &OilFaults { filter_clog: 0.8, ..Default::default() });
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
    fn a_cold_engine_stands_with_a_full_tank() {
        let oil = OilSystem::new(288.0);
        assert!((oil.quantity_fraction() - 1.0).abs() < 1e-12);
        assert!((oil.oil_m3() - 20.0e-3).abs() < 1e-9, "20 L of oil: {}", oil.oil_m3());
    }

    #[test]
    fn oil_is_consumed_past_the_seals_only_while_the_bearings_are_being_fed() {
        let mut oil = OilSystem::new(363.15);
        let s = Surroundings { dt_s: 1.0, ..surroundings(0.97) };
        let start = oil.oil_m3();
        let mut out = OilState::default();
        for _ in 0..3600 {
            out = oil.step(&s, &OilFaults::default());
        }
        let litres_per_hour = (start - oil.oil_m3()) * 1000.0;
        assert!(
            litres_per_hour < SEAL_LOSS_L_PER_H_AT_DESIGN_FLOW
                && litres_per_hour > 0.8 * SEAL_LOSS_L_PER_H_AT_DESIGN_FLOW,
            "{litres_per_hour:.3} L/h against a rated {SEAL_LOSS_L_PER_H_AT_DESIGN_FLOW} L/h"
        );
        assert!(out.seal_loss_m3_s > 0.0 && out.leak_m3_s == 0.0);
        assert!(oil.quantity_fraction() > 0.98, "{}", oil.quantity_fraction());

        let stopped = Surroundings { n3_frac: 0.0, friction_w: 0.0, fuel_kg_s: 0.0, bypass_kg_s: 0.0, dt_s: 1.0, ..surroundings(0.0) };
        let before = oil.oil_m3();
        for _ in 0..3600 {
            oil.step(&stopped, &OilFaults::default());
        }
        assert_eq!(oil.oil_m3(), before, "a stopped engine consumes no oil");
    }

    #[test]
    fn a_leak_drains_the_tank_and_the_pressure_follows_it_down() {
        let mut oil = OilSystem::new(363.15);
        let s = Surroundings { dt_s: 1.0, ..surroundings(0.97) };
        let faults = OilFaults { leak: 1.0, ..Default::default() };

        let healthy = oil.step(&s, &OilFaults::default());
        assert!(healthy.pressure_psi > 50.0, "{:.1} psi", healthy.pressure_psi);

        let mut half = healthy;
        while oil.quantity_fraction() > 0.5 {
            half = oil.step(&s, &faults);
        }
        assert!(half.leak_m3_s > 0.0);
        assert_eq!(half.pump_prime_fraction, 1.0, "the inlet is still covered at half a tank");
        assert!(
            (half.pressure_psi - healthy.pressure_psi).abs() < 0.05 * healthy.pressure_psi,
            "pressure must barely move while the inlet is still covered: {:.1} -> {:.1} psi",
            healthy.pressure_psi,
            half.pressure_psi
        );

        let mut low = half;
        while oil.quantity_fraction() > 0.05 {
            low = oil.step(&s, &faults);
        }
        assert!(low.pressure_psi < 0.5 * healthy.pressure_psi, "{:.1} psi at {:.2} full", low.pressure_psi, low.quantity_fraction);
        assert!(low.pump_prime_fraction < 0.4);

        let mut dry = low;
        for _ in 0..600 {
            dry = oil.step(&s, &faults);
        }
        assert_eq!(dry.quantity_m3, 0.0);
        assert_eq!(dry.quantity_fraction, 0.0);
        assert_eq!(dry.pressure_psi, 0.0);
        assert!(dry.leak_m3_s >= 0.0 && dry.quantity_m3 >= 0.0, "the tank never goes negative");
    }

    #[test]
    fn a_leak_runs_at_the_pressure_behind_it() {
        let faults = OilFaults { leak: 1.0, ..Default::default() };
        let at = |n3: f64| {
            let mut oil = OilSystem::new(363.15);
            oil.step(&Surroundings { dt_s: 0.1, ..surroundings(n3) }, &faults).leak_m3_s
        };
        let (idle, takeoff) = (at(0.62), at(0.97));
        assert!(takeoff > idle && idle > 0.0, "idle {idle:.3e}, take-off {takeoff:.3e} m^3/s");

        let mut stopped_engine = OilSystem::new(300.0);
        let stopped = Surroundings { n3_frac: 0.0, friction_w: 0.0, fuel_kg_s: 0.0, bypass_kg_s: 0.0, dt_s: 1.0, ..surroundings(0.0) };
        let out = stopped_engine.step(&stopped, &faults);
        assert_eq!(out.leak_m3_s, 0.0, "an unpressurised gallery does not squirt");
        assert_eq!(stopped_engine.quantity_fraction(), 1.0);
    }

    #[test]
    fn a_full_leak_empties_the_tank_in_the_minutes_it_is_sized_for() {
        let mut oil = OilSystem::new(363.15);
        let s = Surroundings { dt_s: 1.0, ..surroundings(0.97) };
        let faults = OilFaults { leak: 1.0, ..Default::default() };
        let mut seconds = 0u32;
        while oil.quantity_fraction() > 0.0 && seconds < 3600 {
            oil.step(&s, &faults);
            seconds += 1;
        }
        assert!((120..=900).contains(&seconds), "emptied in {seconds} s; sized for {LEAK_FULL_DRAIN_S} s");
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
