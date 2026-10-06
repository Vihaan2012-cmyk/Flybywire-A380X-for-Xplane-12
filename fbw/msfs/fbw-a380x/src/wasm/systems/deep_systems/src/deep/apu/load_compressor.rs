use super::actuator::Actuator;
use super::compressor_map;
use super::gas;
use super::params;

#[derive(Clone, Copy, Debug, Default)]
pub struct LoadCompressorFaults {
    pub efficiency_loss: f64,
    pub igv_jam: f64,
    pub scv_jam: f64,
}

pub struct Inputs {
    pub n_frac: f64,
    pub ambient_pressure_pa: f64,
    pub ambient_temperature_k: f64,
    pub bleed_demand_kg_s: f64,
    pub electrical_load_frac: f64,
    pub dt_s: f64,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct Outputs {
    pub delivered_bleed_kg_s: f64,
    pub delivered_pressure_pa: f64,
    pub delivered_temperature_k: f64,
    pub shaft_power_w: f64,
    pub igv_position_frac: f64,
    pub scv_position_frac: f64,
    pub in_surge: bool,
    pub surge_margin: f64,
}

pub fn igv_load_shed_scale(electrical_load_frac: f64) -> f64 {
    let load = electrical_load_frac.max(0.0);
    let start = params::IGV_LOAD_SHED_START_FRAC;
    let full = params::IGV_LOAD_SHED_FULL_FRAC;
    let floor = params::IGV_LOAD_SHED_FLOOR;
    if load <= start {
        1.0
    } else if load >= full {
        floor
    } else {
        let t = (load - start) / (full - start).max(1e-9);
        1.0 + t * (floor - 1.0)
    }
}

#[derive(Clone, Debug)]
pub struct LoadCompressor {
    spec: compressor_map::Spec,
    igv: Actuator,
    scv: Actuator,
}

impl LoadCompressor {
    pub fn new() -> Self {
        Self {
            spec: compressor_map::Spec {
                pr_design: params::LOAD_PRESSURE_RATIO_DESIGN,
                eta_design: params::LOAD_COMPRESSOR_EFFICIENCY_DESIGN,
                mdot_corrected_design_kg_s: params::LOAD_MDOT_DESIGN_KG_S,
                efficiency_falloff: params::LOAD_COMPRESSOR_EFFICIENCY_FALLOFF,
                surge_margin_design_frac: params::LOAD_SURGE_MARGIN_DESIGN_FRAC,
                surge_line_flatness: params::LOAD_SURGE_LINE_FLATNESS,
                choke_flow_multiple: params::LOAD_CHOKE_FLOW_MULTIPLE,
                erosion_efficiency_loss: 0.0,
            },
            igv: Actuator::new(0.0, params::IGV_FULL_TRAVEL_RATE_PER_S),
            scv: Actuator::new(0.0, params::SCV_FULL_TRAVEL_RATE_PER_S),
        }
    }

    pub fn igv_position_frac(&self) -> f64 {
        self.igv.position_frac()
    }

    pub fn scv_position_frac(&self) -> f64 {
        self.scv.position_frac()
    }

    pub fn step(&mut self, inputs: &Inputs, faults: &LoadCompressorFaults) -> Outputs {
        let dt = inputs.dt_s.max(0.0);
        let n_frac = inputs.n_frac.max(0.0);
        let t1 = inputs.ambient_temperature_k.max(1.0);
        let p1 = inputs.ambient_pressure_pa.max(1.0);
        let spec = self.spec.degraded(faults.efficiency_loss);

        let demand_corrected = gas::corrected_flow_kg_s(inputs.bleed_demand_kg_s.max(0.0), t1, p1);
        let load_shed = igv_load_shed_scale(inputs.electrical_load_frac);
        let igv_desired = (demand_corrected / params::LOAD_MDOT_DESIGN_KG_S * load_shed).clamp(0.0, 1.0);
        let igv_pos = self.igv.step(igv_desired, faults.igv_jam, dt);
        let demand_requested = demand_corrected.min(params::LOAD_MDOT_DESIGN_KG_S * igv_pos);

        let surge_flow = compressor_map::surge_corrected_flow_kg_s(&spec, n_frac);
        let target_total = surge_flow * (1.0 + params::SCV_SURGE_SAFETY_MARGIN_FRAC);
        let needed_recirculation = (target_total - demand_requested).max(0.0);
        let scv_desired = (needed_recirculation / params::LOAD_MDOT_DESIGN_KG_S).clamp(0.0, 1.0);
        let scv_pos = self.scv.step(scv_desired, faults.scv_jam, dt);
        let scv_recirculation_corrected = params::LOAD_MDOT_DESIGN_KG_S * scv_pos;

        let requested_corrected = demand_requested + scv_recirculation_corrected;
        let point = compressor_map::evaluate(&spec, t1, p1, n_frac, requested_corrected);

        let delivered_share = if requested_corrected > 1e-9 {
            (demand_requested / requested_corrected).clamp(0.0, 1.0)
        } else {
            0.0
        };

        Outputs {
            delivered_bleed_kg_s: point.mdot_kg_s * delivered_share,
            delivered_pressure_pa: point.pt_out_pa,
            delivered_temperature_k: point.tt_out_k,
            shaft_power_w: point.power_w,
            igv_position_frac: igv_pos,
            scv_position_frac: scv_pos,
            in_surge: point.in_surge,
            surge_margin: point.surge_margin,
        }
    }
}

impl Default for LoadCompressor {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn no_faults() -> LoadCompressorFaults {
        LoadCompressorFaults::default()
    }

    fn inputs(n_frac: f64, bleed_demand_kg_s: f64, dt_s: f64) -> Inputs {
        Inputs {
            n_frac,
            ambient_pressure_pa: 101_325.0,
            ambient_temperature_k: 288.15,
            bleed_demand_kg_s,
            electrical_load_frac: 0.0,
            dt_s,
        }
    }

    #[test]
    fn at_rest_with_no_demand_nothing_moves_and_nothing_is_nan() {
        let mut lc = LoadCompressor::new();
        let out = lc.step(&inputs(0.0, 0.0, 1.0), &no_faults());
        assert!(out.delivered_bleed_kg_s.abs() < 1e-9);
        assert!(!out.in_surge);
        assert!(out.delivered_pressure_pa.is_finite());
    }

    #[test]
    fn steady_full_demand_at_design_speed_delivers_close_to_the_demand() {
        let mut lc = LoadCompressor::new();
        let mut out = Outputs::default();
        for _ in 0..40 {
            out = lc.step(&inputs(1.0, params::LOAD_MDOT_DESIGN_KG_S, 0.1), &no_faults());
        }
        assert!(!out.in_surge, "{:?}", out);
        assert!(
            (out.delivered_bleed_kg_s - params::LOAD_MDOT_DESIGN_KG_S).abs()
                < 0.1 * params::LOAD_MDOT_DESIGN_KG_S,
            "{}",
            out.delivered_bleed_kg_s
        );
    }

    #[test]
    fn a_sudden_demand_drop_with_a_stuck_surge_valve_causes_surge_that_a_healthy_one_avoids() {
        let n_frac = 1.0;
        let mut healthy = LoadCompressor::new();
        let mut jammed = LoadCompressor::new();

        for _ in 0..40 {
            healthy.step(&inputs(n_frac, params::LOAD_MDOT_DESIGN_KG_S, 0.1), &no_faults());
            jammed.step(
                &inputs(n_frac, params::LOAD_MDOT_DESIGN_KG_S, 0.1),
                &LoadCompressorFaults { scv_jam: 1.0, ..LoadCompressorFaults::default() },
            );
        }

        let mut healthy_out = Outputs::default();
        let mut jammed_out = Outputs::default();
        for _ in 0..40 {
            healthy_out = healthy.step(&inputs(n_frac, 0.01, 0.1), &no_faults());
            jammed_out = jammed.step(
                &inputs(n_frac, 0.01, 0.1),
                &LoadCompressorFaults { scv_jam: 1.0, ..LoadCompressorFaults::default() },
            );
        }

        assert!(!healthy_out.in_surge, "{:?}", healthy_out);
        assert!(jammed_out.in_surge, "{:?}", jammed_out);
        assert!(healthy_out.scv_position_frac > jammed_out.scv_position_frac);
    }

    #[test]
    fn a_jammed_igv_can_starve_delivery_even_though_the_aircraft_demands_more() {
        let mut healthy = LoadCompressor::new();
        let mut jammed = LoadCompressor::new();
        let mut healthy_out = Outputs::default();
        let mut jammed_out = Outputs::default();
        for _ in 0..40 {
            healthy_out = healthy.step(&inputs(1.0, params::LOAD_MDOT_DESIGN_KG_S, 0.1), &no_faults());
            jammed_out = jammed.step(
                &inputs(1.0, params::LOAD_MDOT_DESIGN_KG_S, 0.1),
                &LoadCompressorFaults { igv_jam: 1.0, ..LoadCompressorFaults::default() },
            );
        }
        assert!(jammed_out.delivered_bleed_kg_s < 0.1 * healthy_out.delivered_bleed_kg_s);
    }

    #[test]
    fn the_igv_load_shed_schedule_is_full_below_its_start_and_floored_above_its_full_point() {
        assert_eq!(igv_load_shed_scale(0.0), 1.0);
        assert_eq!(igv_load_shed_scale(params::IGV_LOAD_SHED_START_FRAC), 1.0);
        assert_eq!(igv_load_shed_scale(params::IGV_LOAD_SHED_FULL_FRAC), params::IGV_LOAD_SHED_FLOOR);
        assert_eq!(igv_load_shed_scale(params::IGV_LOAD_SHED_FULL_FRAC + 1.0), params::IGV_LOAD_SHED_FLOOR);
        let mid = igv_load_shed_scale(
            (params::IGV_LOAD_SHED_START_FRAC + params::IGV_LOAD_SHED_FULL_FRAC) / 2.0,
        );
        assert!(mid < 1.0 && mid > params::IGV_LOAD_SHED_FLOOR);
    }

    #[test]
    fn high_electrical_load_sheds_delivered_bleed_for_the_same_demand_and_speed() {
        let mut low_elec = LoadCompressor::new();
        let mut high_elec = LoadCompressor::new();
        let mut low_out = Outputs::default();
        let mut high_out = Outputs::default();
        for _ in 0..60 {
            low_out = low_elec.step(
                &Inputs { electrical_load_frac: 0.0, ..inputs(1.0, params::LOAD_MDOT_DESIGN_KG_S, 0.1) },
                &no_faults(),
            );
            high_out = high_elec.step(
                &Inputs {
                    electrical_load_frac: params::IGV_LOAD_SHED_FULL_FRAC,
                    ..inputs(1.0, params::LOAD_MDOT_DESIGN_KG_S, 0.1)
                },
                &no_faults(),
            );
        }
        assert!(high_out.delivered_bleed_kg_s < low_out.delivered_bleed_kg_s);
        assert!(high_out.igv_position_frac < low_out.igv_position_frac);
    }

    #[test]
    fn erosion_lowers_delivered_pressure_for_the_same_demand_and_speed() {
        let mut healthy = LoadCompressor::new();
        let mut eroded = LoadCompressor::new();
        let mut healthy_out = Outputs::default();
        let mut eroded_out = Outputs::default();
        for _ in 0..40 {
            healthy_out = healthy.step(&inputs(1.0, params::LOAD_MDOT_DESIGN_KG_S, 0.1), &no_faults());
            eroded_out = eroded.step(
                &inputs(1.0, params::LOAD_MDOT_DESIGN_KG_S, 0.1),
                &LoadCompressorFaults { efficiency_loss: 0.4, ..LoadCompressorFaults::default() },
            );
        }
        assert!(eroded_out.delivered_pressure_pa < healthy_out.delivered_pressure_pa);
    }
}
