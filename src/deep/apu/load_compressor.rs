//! The load (customer bleed) compressor -- item 1's "separate load
//! compressor (bleed)" -- with its own inlet guide vanes and surge control
//! valve (item 2).
//!
//! Per `params.rs`'s module docs, this is folded onto the same single
//! modelled spool as the power section (`power_section.rs`): the PW980A's
//! real free/power section (which the load compressor and generators
//! actually sit on) is treated as rigidly geared to the gas-generator core,
//! the same reduced-order choice FlyByWire's own file documents for the
//! identical reason.
//!
//! **Inlet guide vanes**: meter how much of the aircraft's own bleed demand
//! the stage is even allowed to try to pass -- closing them restricts
//! *incoming* flow, so a jammed-shut IGV can starve the bleed system even
//! while the aircraft's own valve calls for more. Modelled as a
//! rate-limited actuator (`actuator.rs`) auto-tracking demand.
//!
//! **Surge control valve (SCV)**: a fast-acting anti-surge/recirculation
//! valve, standard practice on centrifugal compressors whose downstream
//! demand can drop faster than the compressor's own speed can follow: it
//! opens to recirculate flow through the stage (dumped rather than
//! delivered to the aircraft) whenever the IGV-throttled demand alone would
//! let the operating point fall toward the surge line, so the stage always
//! has *some* minimum through-flow regardless of how little the aircraft is
//! asking for. This is also modelled as a rate-limited actuator, with its
//! own control law (`params::SCV_SURGE_SAFETY_MARGIN_FRAC` of headroom above
//! the surge line) rather than a crew/FADEC command -- exactly the causal
//! route the brief's item 2 asks for: "surge when demand drops with valve
//! stuck" falls straight out of a *jammed* SCV actuator failing to open when
//! the control law calls for it, not a scripted surge event.
//!
//! **Bleed vs. generator load priority** (item 4): the IGV schedule sheds
//! bleed demand as the combined electrical load on both generators rises
//! (`igv_load_shed_scale`), the real load-management logic behind a shaft
//! that has to supply both loads at once -- electrical load has priority
//! (bleed is the more sheddable of the two on a real aircraft) and this
//! keeps the load compressor's own corrected flow, and with it its surge
//! margin, from being pushed down purely by IGVs chasing full bleed demand
//! while the shaft is already heavily loaded electrically. Surge margin
//! genuinely gets worse as *both* rise together even with this schedule
//! (less shed happens at any one electrical load the higher the bleed
//! demand is relative to it) -- this schedule only bounds how much worse,
//! it does not eliminate the coupling, which is real (both loads share the
//! one shaft `apu.rs` sums torque on).

use super::actuator::Actuator;
use super::compressor_map;
use super::gas;
use super::params;

#[derive(Clone, Copy, Debug, Default)]
pub struct LoadCompressorFaults {
    /// Load compressor erosion/damage: isentropic efficiency loss.
    pub efficiency_loss: f64,
    /// Inlet guide vane actuator seizure, 0 healthy .. 1 fully jammed.
    pub igv_jam: f64,
    /// Surge control valve actuator seizure, 0 healthy .. 1 fully jammed.
    pub scv_jam: f64,
}

pub struct Inputs {
    /// Core/gas-generator spool speed fraction (0..~1.1); the load
    /// compressor is on the same modelled shaft.
    pub n_frac: f64,
    pub ambient_pressure_pa: f64,
    pub ambient_temperature_k: f64,
    /// The aircraft's own requested bleed flow, actual kg/s (not corrected).
    pub bleed_demand_kg_s: f64,
    /// Combined electrical load on both generators as a fraction of their
    /// combined rated real power (`apu.rs`, from `generators.rs`) -- the
    /// bleed/generator load-priority input, item 4.
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

/// The IGV demand-scaling factor as a function of combined electrical load
/// (item 4): full authority below `IGV_LOAD_SHED_START_FRAC`, linearly
/// shed down to `IGV_LOAD_SHED_FLOOR` by `IGV_LOAD_SHED_FULL_FRAC`.
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

        // IGVs auto-track the aircraft's demand, as a fraction of design
        // corrected flow, scaled back as combined electrical load rises
        // (item 4's bleed/generator load priority).
        let demand_corrected = gas::corrected_flow_kg_s(inputs.bleed_demand_kg_s.max(0.0), t1, p1);
        let load_shed = igv_load_shed_scale(inputs.electrical_load_frac);
        let igv_desired = (demand_corrected / params::LOAD_MDOT_DESIGN_KG_S * load_shed).clamp(0.0, 1.0);
        let igv_pos = self.igv.step(igv_desired, faults.igv_jam, dt);
        let demand_requested = demand_corrected.min(params::LOAD_MDOT_DESIGN_KG_S * igv_pos);

        // The SCV's control law: keep total corrected flow at least
        // `SCV_SURGE_SAFETY_MARGIN_FRAC` above the surge line for the
        // *current* speed. If the IGV-throttled demand alone already clears
        // that, the SCV wants to be shut.
        let surge_flow = compressor_map::surge_corrected_flow_kg_s(&spec, n_frac);
        let target_total = surge_flow * (1.0 + params::SCV_SURGE_SAFETY_MARGIN_FRAC);
        let needed_recirculation = (target_total - demand_requested).max(0.0);
        let scv_desired = (needed_recirculation / params::LOAD_MDOT_DESIGN_KG_S).clamp(0.0, 1.0);
        let scv_pos = self.scv.step(scv_desired, faults.scv_jam, dt);
        let scv_recirculation_corrected = params::LOAD_MDOT_DESIGN_KG_S * scv_pos;

        let requested_corrected = demand_requested + scv_recirculation_corrected;
        let point = compressor_map::evaluate(&spec, t1, p1, n_frac, requested_corrected);

        // Only the aircraft's own share of what the stage actually passed
        // reaches the bleed manifold; the SCV's recirculated share is
        // dumped, not delivered.
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

    /// The causal reproduction of the brief's item 2: bleed demand collapses
    /// suddenly; a healthy surge control valve opens in time to keep the
    /// stage above its surge line, but a *jammed* one (frozen at the near-
    /// zero recirculation position it held while demand was high) cannot,
    /// and the stage surges.
    #[test]
    fn a_sudden_demand_drop_with_a_stuck_surge_valve_causes_surge_that_a_healthy_one_avoids() {
        let n_frac = 1.0;
        let mut healthy = LoadCompressor::new();
        let mut jammed = LoadCompressor::new();

        // Run both up to a steady, high-demand state first (SCV settles
        // near shut on both).
        for _ in 0..40 {
            healthy.step(&inputs(n_frac, params::LOAD_MDOT_DESIGN_KG_S, 0.1), &no_faults());
            jammed.step(
                &inputs(n_frac, params::LOAD_MDOT_DESIGN_KG_S, 0.1),
                &LoadCompressorFaults { scv_jam: 1.0, ..LoadCompressorFaults::default() },
            );
        }

        // Demand collapses to (near) zero.
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
        // The IGV faults in fully jammed *shut* (starts at position 0).
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
