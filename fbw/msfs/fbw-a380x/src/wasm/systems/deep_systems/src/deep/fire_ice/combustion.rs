use super::util::clamp01;

#[derive(Clone, Copy, Debug)]
pub struct Fluid {
    pub heating_value_j_kg: f64,
    pub autoignition_c: f64,
}

pub const JET_FUEL: Fluid = Fluid { heating_value_j_kg: 43.1e6, autoignition_c: 210.0 };
pub const TURBINE_OIL: Fluid = Fluid { heating_value_j_kg: 42.0e6, autoignition_c: 344.0 };
pub const HYDRAULIC_FLUID: Fluid = Fluid { heating_value_j_kg: 27.0e6, autoignition_c: 468.0 };

const STOICH_AFR: f64 = 14.7;

const QUENCH_MARGIN_C: f64 = 80.0;

#[derive(Clone, Copy, Debug, Default)]
pub struct ZoneSupply {
    pub fuel_available_kg_s: f64,
    pub air_available_kg_s: f64,
    pub ignition_source: bool,
    pub suppression_fraction: f64,
}

#[derive(Clone, Copy, Debug)]
pub struct ZoneCombustion {
    fluid: Fluid,
    temp_c: f64,
    thermal_mass_j_k: f64,
    ambient_conductance_w_k: f64,
    burning: bool,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct CombustionState {
    pub temp_c: f64,
    pub burning: bool,
    pub heat_release_w: f64,
    pub burn_rate_kg_s: f64,
}

impl ZoneCombustion {
    pub fn new(fluid: Fluid, ambient_c: f64, thermal_mass_j_k: f64, ambient_conductance_w_k: f64) -> Self {
        Self { fluid, temp_c: ambient_c, thermal_mass_j_k, ambient_conductance_w_k, burning: false }
    }

    pub fn temp_c(&self) -> f64 {
        self.temp_c
    }

    pub fn is_burning(&self) -> bool {
        self.burning
    }

    pub fn step(&mut self, supply: &ZoneSupply, ambient_c: f64, extra_heat_w: f64, dt_s: f64) -> CombustionState {
        let dt = dt_s.max(0.0);
        let fuel = supply.fuel_available_kg_s.max(0.0);
        let air = supply.air_available_kg_s.max(0.0);

        let hot_enough_to_ignite = self.temp_c >= self.fluid.autoignition_c;
        let can_ignite = fuel > 0.0 && air > 0.0 && (supply.ignition_source || hot_enough_to_ignite);

        let quenched_by_suppression = clamp01(supply.suppression_fraction) >= 1.0;
        let starved = fuel <= 0.0 || air <= 0.0;
        let cooled_out = !supply.ignition_source && self.temp_c < self.fluid.autoignition_c - QUENCH_MARGIN_C;

        self.burning = if self.burning {
            !(starved || cooled_out || quenched_by_suppression)
        } else {
            can_ignite && !quenched_by_suppression
        };

        let (burn_rate_kg_s, heat_release_w) = if self.burning {
            let air_limited_fuel = air / STOICH_AFR;
            let burn_rate = fuel.min(air_limited_fuel);
            let suppression_derate = 1.0 - clamp01(supply.suppression_fraction);
            (burn_rate, burn_rate * self.fluid.heating_value_j_kg * suppression_derate)
        } else {
            (0.0, 0.0)
        };

        let ambient_loss_w = self.ambient_conductance_w_k * (self.temp_c - ambient_c);
        let net_w = heat_release_w + extra_heat_w - ambient_loss_w;
        self.temp_c += net_w / self.thermal_mass_j_k.max(1e-6) * dt;

        CombustionState { temp_c: self.temp_c, burning: self.burning, heat_release_w, burn_rate_kg_s }
    }
}

pub fn conductive_link_w(conductance_w_k: f64, hot_zone_c: f64, cold_zone_c: f64) -> f64 {
    conductance_w_k.max(0.0) * (hot_zone_c - cold_zone_c)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn run(zone: &mut ZoneCombustion, supply: &ZoneSupply, ambient_c: f64, extra_heat_w: f64, dt_s: f64, ticks: u32) -> CombustionState {
        let mut out = CombustionState::default();
        for _ in 0..ticks {
            out = zone.step(supply, ambient_c, extra_heat_w, dt_s);
        }
        out
    }

    fn leak_and_air() -> ZoneSupply {
        ZoneSupply { fuel_available_kg_s: 0.02, air_available_kg_s: 2.0, ignition_source: true, suppression_fraction: 0.0 }
    }

    #[test]
    fn a_fuel_leak_with_air_and_an_ignition_source_ignites_and_heats_the_zone() {
        let mut zone = ZoneCombustion::new(HYDRAULIC_FLUID, 20.0, 50_000.0, 30.0);
        let out = run(&mut zone, &leak_and_air(), 20.0, 0.0, 1.0, 600);
        assert!(out.burning);
        assert!(out.temp_c > 200.0, "zone should heat well above ambient once burning, got {}", out.temp_c);
    }

    #[test]
    fn with_no_ignition_source_and_a_cold_zone_fuel_just_pools_unburnt() {
        let mut zone = ZoneCombustion::new(HYDRAULIC_FLUID, 20.0, 50_000.0, 30.0);
        let supply = ZoneSupply { ignition_source: false, ..leak_and_air() };
        let out = run(&mut zone, &supply, 20.0, 0.0, 1.0, 600);
        assert!(!out.burning);
        assert!((out.temp_c - 20.0).abs() < 5.0);
    }

    #[test]
    fn without_air_a_leak_cannot_sustain_combustion_even_with_ignition() {
        let mut zone = ZoneCombustion::new(JET_FUEL, 20.0, 50_000.0, 30.0);
        let starved = ZoneSupply { air_available_kg_s: 0.0, ..leak_and_air() };
        let out = run(&mut zone, &starved, 20.0, 0.0, 1.0, 600);
        assert!(!out.burning, "no oxidiser available, nothing can burn");
    }

    #[test]
    fn isolating_the_fuel_after_a_fire_starts_lets_it_burn_out_and_cool() {
        let mut zone = ZoneCombustion::new(JET_FUEL, 20.0, 50_000.0, 30.0);
        let burning_state = run(&mut zone, &leak_and_air(), 20.0, 0.0, 1.0, 300);
        assert!(burning_state.burning);

        let isolated = ZoneSupply { fuel_available_kg_s: 0.0, ignition_source: false, ..leak_and_air() };
        let out = run(&mut zone, &isolated, 20.0, 0.0, 1.0, 1200);
        assert!(!out.burning, "starved of fuel the flame must go out");
        assert!(out.temp_c < burning_state.temp_c, "the zone must cool once the fire is out");
    }

    #[test]
    fn suppression_agent_at_design_concentration_extinguishes_the_fire() {
        let mut zone = ZoneCombustion::new(JET_FUEL, 20.0, 50_000.0, 30.0);
        let burning_state = run(&mut zone, &leak_and_air(), 20.0, 0.0, 1.0, 300);
        assert!(burning_state.burning);

        let suppressed = ZoneSupply { suppression_fraction: 1.0, ..leak_and_air() };
        let out = zone.step(&suppressed, 20.0, 0.0, 1.0);
        assert!(!out.burning);
        assert_eq!(out.heat_release_w, 0.0);
    }

    #[test]
    fn heat_crossing_a_link_can_ignite_a_neighbours_own_fuel_without_a_local_ignition_source() {
        let mut zone1 = ZoneCombustion::new(JET_FUEL, 20.0, 50_000.0, 30.0);
        let mut zone2 = ZoneCombustion::new(HYDRAULIC_FLUID, 20.0, 20_000.0, 20.0);
        let supply2 = ZoneSupply { ignition_source: false, ..leak_and_air() };

        for _ in 0..3000 {
            zone1.step(&leak_and_air(), 20.0, 0.0, 1.0);
            let link_w = conductive_link_w(200.0, zone1.temp_c(), zone2.temp_c());
            zone2.step(&supply2, 20.0, link_w, 1.0);
        }

        assert!(zone1.is_burning());
        assert!(zone2.is_burning(), "sustained heat from the neighbouring fire should have crossed the link and ignited zone 2's own leak, zone2 temp {}", zone2.temp_c());
    }

    #[test]
    fn cutting_the_link_prevents_the_same_spread() {
        let mut zone1 = ZoneCombustion::new(JET_FUEL, 20.0, 50_000.0, 30.0);
        let mut zone2 = ZoneCombustion::new(HYDRAULIC_FLUID, 20.0, 20_000.0, 20.0);
        let supply2 = ZoneSupply { ignition_source: false, ..leak_and_air() };

        for _ in 0..3000 {
            zone1.step(&leak_and_air(), 20.0, 0.0, 1.0);
            let link_w = conductive_link_w(0.0, zone1.temp_c(), zone2.temp_c());
            zone2.step(&supply2, 20.0, link_w, 1.0);
        }

        assert!(zone1.is_burning());
        assert!(!zone2.is_burning(), "with the link cut, zone 2 must not ignite from zone 1's fire");
    }

    #[test]
    fn no_nan_at_rest_with_zero_dt_and_zero_supply() {
        let mut zone = ZoneCombustion::new(JET_FUEL, 20.0, 50_000.0, 30.0);
        let out = zone.step(&ZoneSupply::default(), 20.0, 0.0, 0.0);
        assert!(!out.temp_c.is_nan());
        assert_eq!(out.heat_release_w, 0.0);
    }
}
