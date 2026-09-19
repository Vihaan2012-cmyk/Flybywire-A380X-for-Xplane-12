//! The hot section's metal: combustor liner, turbine casings, vanes and
//! discs, lumped into one thermal mass the core gas washes over.
//!
//! Running, the gas heats (or, decelerating, is heated by) that metal
//! through forced convection, which scales with gas mass flow to the 0.8
//! power (turbulent pipe flow, Dittus-Boelter). Stopped, the metal loses its
//! heat only by radiation and natural convection to the nacelle, so it stays
//! hot for a long time: a restart soon after shutdown lights into hot metal
//! and runs a hotter start (the reason for the dry-motoring cool-down before
//! a hot restart), and the TGT probe, sitting in stagnant gas among that
//! metal, reads its residual heat until it has soaked away.
//!
//! No Trent 900 thermal data is public. Every size below is GENERIC and
//! derived from public figures: the mass as a fraction of the published dry
//! weight, the heat capacity of a nickel superalloy, a warm-up time
//! constant typical of large turbofan casings (the lag active clearance
//! control is scheduled around, around a minute), and radiating area from
//! the published fan diameter's core.

use super::gas::CP_GAS;
use super::params::DRY_WEIGHT_KG;

/// Hot-section mass: combustor, HP and IP turbines, about an eighth of the
/// engine's dry weight (GENERIC).
const MASS_KG: f64 = 0.12 * DRY_WEIGHT_KG;
/// Nickel superalloy specific heat, J/(kg K) (Inconel 718: ~435-460).
const SPECIFIC_HEAT_J_KG_K: f64 = 450.0;
/// Warm-up time constant at design core flow, s (GENERIC, typical casing
/// thermal lag).
const DESIGN_TIME_CONSTANT_S: f64 = 60.0;
/// Outer surface radiating to the nacelle, m^2: a core casing about 1.1 m
/// across and 1.7 m long (GENERIC, sized off the 2.95 m fan and a bypass
/// ratio of 8.5).
const RADIATING_AREA_M2: f64 = 6.0;
/// Oxidised nickel alloy emissivity.
const EMISSIVITY: f64 = 0.7;
/// Natural convection to still nacelle air, W/(m^2 K).
const NATURAL_CONVECTION_W_M2K: f64 = 5.0;
const STEFAN_BOLTZMANN: f64 = 5.670_374e-8;
/// How strongly the TGT probe sees the surrounding metal against the gas
/// flowing over it at design flow: a thermocouple in a fast gas stream
/// reads the gas, in stagnant gas it settles to what surrounds it.
const PROBE_METAL_WEIGHT_AT_DESIGN_FLOW: f64 = 0.01;

#[derive(Clone, Copy, Debug)]
pub struct HotSection {
    metal_k: f64,
}

/// What the hot section did to the gas this frame.
#[derive(Clone, Copy, Debug)]
pub struct Exchange {
    /// Gas temperature at the TGT plane after giving heat to (or taking it
    /// from) the metal, K.
    pub tgt_gas_k: f64,
    /// What the probe settles to: the gas where it flows, the metal where
    /// it does not, K.
    pub probe_target_k: f64,
}

impl HotSection {
    pub fn new(temp_k: f64) -> Self {
        Self { metal_k: temp_k }
    }

    pub fn metal_k(&self) -> f64 {
        self.metal_k
    }

    /// One frame. `tt4_k` and `tgt_gas_k` bound the gas the metal sits in
    /// (combustor exit and the IP turbine exit, the TGT plane); the metal
    /// sees their mean. `mdot_gas_kg_s` is the core gas flow and
    /// `mdot_design_kg_s` the design core flow the convection is scaled
    /// from. `nacelle_k` is the air the outer casing loses heat to.
    pub fn step(&mut self, tt4_k: f64, tgt_gas_k: f64, mdot_gas_kg_s: f64, mdot_design_kg_s: f64, nacelle_k: f64, dt_s: f64) -> Exchange {
        let capacity = MASS_KG * SPECIFIC_HEAT_J_KG_K;
        let flow_fraction = (mdot_gas_kg_s.max(0.0) / mdot_design_kg_s.max(1e-6)).min(2.0);
        let h_a = capacity / DESIGN_TIME_CONSTANT_S * flow_fraction.powf(0.8);
        let gas_k = 0.5 * (tt4_k + tgt_gas_k);
        // Heat into the metal from the gas: an exact exponential approach
        // over the frame (never past the gas), bounded by the flow's own
        // capacity (the gas cannot give more than it carries past the metal).
        let gas_capacity_w_k = mdot_gas_kg_s.max(0.0) * CP_GAS;
        let h_a = h_a.min(gas_capacity_w_k);
        let before = self.metal_k;
        let exchanged = (gas_k - before) * (1.0 - (-h_a * dt_s.max(0.0) / capacity).exp());
        let to_metal_w = if dt_s > 0.0 { exchanged * capacity / dt_s } else { 0.0 };
        let loss_w = RADIATING_AREA_M2
            * (EMISSIVITY * STEFAN_BOLTZMANN * (before.powi(4) - nacelle_k.powi(4)) + NATURAL_CONVECTION_W_M2K * (before - nacelle_k));
        self.metal_k = before + exchanged - loss_w * dt_s.max(0.0) / capacity;
        // The gas reaching the TGT plane carries the heat it gave up (or
        // picked up) along the way.
        let tgt_gas = if gas_capacity_w_k > 1e-6 {
            let dt_gas = to_metal_w / gas_capacity_w_k;
            // No overshoot past the metal it exchanged with.
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
        // Ten minutes after shutdown: still well above ambient, and cooler
        // than it was.
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
