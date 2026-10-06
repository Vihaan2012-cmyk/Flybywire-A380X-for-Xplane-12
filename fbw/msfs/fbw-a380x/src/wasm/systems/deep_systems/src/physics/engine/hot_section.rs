use super::gas::CP_GAS;
use super::params::DRY_WEIGHT_KG;

const MASS_KG: f64 = 0.12 * DRY_WEIGHT_KG;
const SPECIFIC_HEAT_J_KG_K: f64 = 450.0;
const DESIGN_TIME_CONSTANT_S: f64 = 60.0;
const RADIATING_AREA_M2: f64 = 6.0;
const EMISSIVITY: f64 = 0.7;
const NATURAL_CONVECTION_W_M2K: f64 = 5.0;
const STEFAN_BOLTZMANN: f64 = 5.670_374e-8;
const PROBE_METAL_WEIGHT_AT_DESIGN_FLOW: f64 = 0.01;

#[derive(Clone, Copy, Debug)]
pub struct HotSection {
    metal_k: f64,
}

#[derive(Clone, Copy, Debug)]
pub struct Exchange {
    pub tgt_gas_k: f64,
    pub probe_target_k: f64,
}

impl HotSection {
    pub fn new(temp_k: f64) -> Self {
        Self { metal_k: temp_k }
    }

    pub fn metal_k(&self) -> f64 {
        self.metal_k
    }

    pub fn step(&mut self, tt4_k: f64, tgt_gas_k: f64, mdot_gas_kg_s: f64, mdot_design_kg_s: f64, nacelle_k: f64, dt_s: f64) -> Exchange {
        let capacity = MASS_KG * SPECIFIC_HEAT_J_KG_K;
        let flow_fraction = (mdot_gas_kg_s.max(0.0) / mdot_design_kg_s.max(1e-6)).min(2.0);
        let h_a = capacity / DESIGN_TIME_CONSTANT_S * flow_fraction.powf(0.8);
        let gas_k = 0.5 * (tt4_k + tgt_gas_k);
        let gas_capacity_w_k = mdot_gas_kg_s.max(0.0) * CP_GAS;
        let h_a = h_a.min(gas_capacity_w_k);
        let before = self.metal_k;
        let exchanged = (gas_k - before) * (1.0 - (-h_a * dt_s.max(0.0) / capacity).exp());
        let to_metal_w = if dt_s > 0.0 { exchanged * capacity / dt_s } else { 0.0 };
        let loss_w = RADIATING_AREA_M2
            * (EMISSIVITY * STEFAN_BOLTZMANN * (before.powi(4) - nacelle_k.powi(4)) + NATURAL_CONVECTION_W_M2K * (before - nacelle_k));
        self.metal_k = before + exchanged - loss_w * dt_s.max(0.0) / capacity;
        let tgt_gas = if gas_capacity_w_k > 1e-6 {
            let dt_gas = to_metal_w / gas_capacity_w_k;
            let bound = tgt_gas_k - self.metal_k;
            tgt_gas_k - dt_gas.clamp(bound.min(0.0), bound.max(0.0))
        } else {
            tgt_gas_k
        };
        let gas_weight = flow_fraction.powf(0.8);
        let metal_weight = PROBE_METAL_WEIGHT_AT_DESIGN_FLOW;
        let probe_target_k = (gas_weight * tgt_gas + metal_weight * self.metal_k) / (gas_weight + metal_weight);
        Exchange { tgt_gas_k: tgt_gas, probe_target_k }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn running_gas_brings_the_metal_to_its_temperature_within_minutes() {
        let mut hot = HotSection::new(288.0);
        for _ in 0..(300.0 / 0.05) as usize {
            hot.step(1500.0, 1000.0, 130.0, 130.0, 288.0, 0.05);
        }
        assert!(hot.metal_k() > 1150.0 && hot.metal_k() < 1250.0, "{}", hot.metal_k());
    }

    #[test]
    fn stopped_it_keeps_its_heat_for_many_minutes_and_the_probe_reads_it() {
        let mut hot = HotSection::new(1200.0);
        let mut ex = hot.step(288.0, 288.0, 0.0, 130.0, 288.0, 0.05);
        for _ in 0..(600.0 / 0.05) as usize {
            ex = hot.step(288.0, 288.0, 0.0, 130.0, 288.0, 0.05);
        }
        assert!(hot.metal_k() > 500.0 && hot.metal_k() < 1100.0, "{}", hot.metal_k());
        assert!((ex.probe_target_k - hot.metal_k()).abs() < 1.0);
    }

    #[test]
    fn cool_gas_over_hot_metal_arrives_hotter() {
        let mut hot = HotSection::new(900.0);
        let ex = hot.step(500.0, 400.0, 20.0, 130.0, 288.0, 0.05);
        assert!(ex.tgt_gas_k > 400.0 && ex.tgt_gas_k <= 900.0, "{}", ex.tgt_gas_k);
    }
}
