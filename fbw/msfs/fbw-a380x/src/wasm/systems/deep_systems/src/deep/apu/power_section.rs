use super::compressor_map;
use super::gas;
use super::params;
use super::turbine_flow;
use super::{combustor, combustor::Combustion};

#[derive(Clone, Copy, Debug, Default)]
pub struct PowerSectionFaults {
    pub compressor_efficiency_loss: f64,
    pub turbine_efficiency_loss: f64,
}

#[derive(Clone, Copy, Debug)]
pub struct Calibration {
    pub core_spec: compressor_map::Spec,
    pub turbine_spec: turbine_flow::Spec,
    pub turbine_capacity_coefficient: f64,
    pub turbine_pressure_ratio_design: f64,
    pub fuel_flow_design_kg_s: f64,
    pub omega_rated_rad_s: f64,
    pub design_turbine_shaft_power_w: f64,
    pub design_compressor_power_w: f64,
}

impl Calibration {
    pub fn design_available_accessory_power_w(&self) -> f64 {
        (self.design_turbine_shaft_power_w - self.design_compressor_power_w).max(0.0)
    }
}

pub fn calibrate() -> Calibration {
    let core_spec = compressor_map::Spec {
        pr_design: params::CORE_PRESSURE_RATIO_DESIGN,
        eta_design: params::CORE_COMPRESSOR_EFFICIENCY_DESIGN,
        mdot_corrected_design_kg_s: params::CORE_MDOT_DESIGN_KG_S,
        efficiency_falloff: params::CORE_COMPRESSOR_EFFICIENCY_FALLOFF,
        surge_margin_design_frac: params::CORE_SURGE_MARGIN_DESIGN_FRAC,
        surge_line_flatness: params::CORE_SURGE_LINE_FLATNESS,
        choke_flow_multiple: params::CORE_CHOKE_FLOW_MULTIPLE,
        erosion_efficiency_loss: 0.0,
    };
    let turbine_spec = turbine_flow::Spec {
        eta_design: params::TURBINE_EFFICIENCY_DESIGN,
        efficiency_falloff: params::TURBINE_EFFICIENCY_FALLOFF,
    };

    let design_compressor =
        compressor_map::evaluate(&core_spec, gas::T_REF_K, gas::P_REF_PA, 1.0, params::CORE_MDOT_DESIGN_KG_S);

    let fuel_flow_design = combustor::fuel_flow_for_target_tt4_kg_s(
        design_compressor.mdot_kg_s,
        design_compressor.tt_out_k,
        params::T4_DESIGN_K,
    );
    let design_combustion: Combustion = combustor::burn(
        design_compressor.mdot_kg_s,
        fuel_flow_design,
        design_compressor.tt_out_k,
        design_compressor.pt_out_pa,
    );

    let turbine_pressure_ratio_design = design_combustion.pt4_pa / gas::P_REF_PA;
    let corrected_flow_turbine_design = gas::corrected_flow_kg_s(
        design_combustion.mdot_gas_kg_s,
        design_combustion.tt4_k,
        design_combustion.pt4_pa,
    );
    let turbine_capacity_coefficient = turbine_flow::calibrate_capacity_coefficient(
        corrected_flow_turbine_design,
        turbine_pressure_ratio_design,
    );

    let design_expansion = turbine_flow::expand(
        &turbine_spec,
        design_combustion.tt4_k,
        design_combustion.pt4_pa,
        design_combustion.mdot_gas_kg_s,
        turbine_pressure_ratio_design,
        1.0,
    );

    Calibration {
        core_spec,
        turbine_spec,
        turbine_capacity_coefficient,
        turbine_pressure_ratio_design,
        fuel_flow_design_kg_s: fuel_flow_design,
        omega_rated_rad_s: params::N_DESIGN_RPM * std::f64::consts::TAU / 60.0,
        design_turbine_shaft_power_w: design_expansion.shaft_power_w,
        design_compressor_power_w: design_compressor.power_w,
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub struct GasPath {
    pub compressor: compressor_map::Point,
    pub combustion: Combustion,
    pub expansion: turbine_flow::Expansion,
    pub turbine_pressure_ratio: f64,
}

pub fn gas_path(
    calibration: &Calibration,
    faults: &PowerSectionFaults,
    t1_k: f64,
    p1_pa: f64,
    n_frac: f64,
    fuel_flow_kg_s: f64,
) -> GasPath {
    let n = n_frac.max(0.0);
    let t1 = t1_k.max(1.0);
    let p1 = p1_pa.max(1.0);
    let core_spec = calibration.core_spec.degraded(faults.compressor_efficiency_loss);
    let flow_capacity_frac = (1.0 - faults.compressor_efficiency_loss.clamp(0.0, 1.0)).max(0.3);
    let requested_corrected = params::CORE_MDOT_DESIGN_KG_S * n * flow_capacity_frac;
    let compressor = compressor_map::evaluate(&core_spec, t1, p1, n, requested_corrected);

    let combustion = combustor::burn(
        compressor.mdot_kg_s,
        fuel_flow_kg_s.max(0.0),
        compressor.tt_out_k,
        compressor.pt_out_pa,
    );

    let turbine_spec = calibration.turbine_spec.degraded(faults.turbine_efficiency_loss);
    let corrected_flow_turbine =
        gas::corrected_flow_kg_s(combustion.mdot_gas_kg_s, combustion.tt4_k, combustion.pt4_pa);
    let turbine_pressure_ratio = turbine_flow::pressure_ratio_for_flow(
        corrected_flow_turbine,
        calibration.turbine_capacity_coefficient,
    );
    let pr_frac_of_design =
        turbine_pressure_ratio / calibration.turbine_pressure_ratio_design.max(1.0 + 1e-6);
    let expansion = turbine_flow::expand(
        &turbine_spec,
        combustion.tt4_k,
        combustion.pt4_pa,
        combustion.mdot_gas_kg_s,
        turbine_pressure_ratio,
        pr_frac_of_design,
    );

    GasPath { compressor, combustion, expansion, turbine_pressure_ratio }
}

pub struct Inputs {
    pub ambient_pressure_pa: f64,
    pub ambient_temperature_k: f64,
    pub inlet_pressure_loss_frac: f64,
    pub fuel_flow_kg_s: f64,
    pub starter_torque_nm: f64,
    pub accessory_torque_nm: f64,
    pub dt_s: f64,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct Outputs {
    pub n_percent: f64,
    pub egt_c: f64,
    pub core_mdot_kg_s: f64,
    pub core_pt3_pa: f64,
    pub compressor_surge_margin: f64,
    pub compressor_in_surge: bool,
    pub turbine_shaft_power_w: f64,
    pub compressor_power_w: f64,
    pub omega_rad_s: f64,
}

#[derive(Clone, Debug)]
pub struct PowerSection {
    calibration: Calibration,
    n_percent: f64,
    egt_k: f64,
}

impl PowerSection {
    pub fn new(ambient_temperature_k: f64) -> Self {
        Self {
            calibration: calibrate(),
            n_percent: 0.0,
            egt_k: ambient_temperature_k.max(1.0),
        }
    }

    pub fn calibration(&self) -> &Calibration {
        &self.calibration
    }

    pub fn n_percent(&self) -> f64 {
        self.n_percent
    }

    pub fn egt_c(&self) -> f64 {
        self.egt_k - 273.15
    }

    pub fn omega_rated_rad_s(&self) -> f64 {
        self.calibration.omega_rated_rad_s
    }

    pub fn omega_rad_s(&self) -> f64 {
        (self.n_percent / 100.0).max(0.0) * self.calibration.omega_rated_rad_s
    }

    pub fn design_available_accessory_power_w(&self) -> f64 {
        self.calibration.design_available_accessory_power_w()
    }

    pub fn fuel_flow_design_kg_s(&self) -> f64 {
        self.calibration.fuel_flow_design_kg_s
    }

    pub fn overspeed_tripped(&self) -> bool {
        self.n_percent > params::OVERSPEED_TRIP_PERCENT
    }

    pub fn egt_over_hard_trip(&self) -> bool {
        self.egt_c() > params::EGT_TRIP_C
    }

    const MAX_PHYSICS_SUBSTEP_S: f64 = 0.05;

    pub fn step(&mut self, inputs: &Inputs, faults: &PowerSectionFaults) -> Outputs {
        let total_dt = inputs.dt_s.max(0.0);
        let substeps = (total_dt / Self::MAX_PHYSICS_SUBSTEP_S).ceil().max(1.0) as u32;
        let dt = total_dt / substeps as f64;
        let mut out = Outputs::default();
        for _ in 0..substeps {
            out = self.step_once(inputs, faults, dt);
        }
        out
    }

    fn step_once(&mut self, inputs: &Inputs, faults: &PowerSectionFaults, dt: f64) -> Outputs {
        let n_frac = (self.n_percent / 100.0).max(0.0);
        let omega = n_frac * self.calibration.omega_rated_rad_s;

        let p1 = inputs.ambient_pressure_pa.max(1.0)
            * (1.0 - inputs.inlet_pressure_loss_frac.clamp(0.0, 0.5));
        let t1 = inputs.ambient_temperature_k.max(1.0);

        let gp = gas_path(
            &self.calibration,
            faults,
            t1,
            p1,
            n_frac,
            inputs.fuel_flow_kg_s,
        );
        let compressor = gp.compressor;
        let expansion = gp.expansion;

        let compressor_torque_nm = if omega > 1.0 { compressor.power_w / omega } else { 0.0 };
        let turbine_torque_nm = if omega > 1.0 { expansion.shaft_power_w / omega } else { 0.0 };
        let net_torque_nm = inputs.starter_torque_nm + turbine_torque_nm
            - compressor_torque_nm
            - inputs.accessory_torque_nm.max(0.0);

        let angular_accel = net_torque_nm / params::ROTOR_INERTIA_KG_M2;
        let new_omega = (omega + angular_accel * dt).max(0.0);
        self.n_percent = (new_omega / self.calibration.omega_rated_rad_s * 100.0).clamp(0.0, 140.0);

        self.egt_k = if inputs.fuel_flow_kg_s > 1e-9 {
            expansion.tt_out_k.max(t1)
        } else {
            (self.egt_k - 1.0 * dt).max(t1)
        };

        Outputs {
            n_percent: self.n_percent,
            egt_c: self.egt_c(),
            core_mdot_kg_s: compressor.mdot_kg_s,
            core_pt3_pa: compressor.pt_out_pa,
            compressor_surge_margin: compressor.surge_margin,
            compressor_in_surge: compressor.in_surge,
            turbine_shaft_power_w: expansion.shaft_power_w,
            compressor_power_w: compressor.power_w,
            omega_rad_s: new_omega,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn no_faults() -> PowerSectionFaults {
        PowerSectionFaults::default()
    }

    #[test]
    fn at_rest_with_nothing_driving_it_stays_at_rest_with_no_nan() {
        let mut ps = PowerSection::new(288.15);
        let out = ps.step(
            &Inputs {
                ambient_pressure_pa: 101_325.0,
                ambient_temperature_k: 288.15,
                inlet_pressure_loss_frac: 0.0,
                fuel_flow_kg_s: 0.0,
                starter_torque_nm: 0.0,
                accessory_torque_nm: 0.0,
                dt_s: 1.0,
            },
            &no_faults(),
        );
        assert_eq!(out.n_percent, 0.0);
        assert!(out.egt_c.is_finite());
        assert!(!out.compressor_in_surge);
    }

    #[test]
    fn starter_torque_alone_spins_the_core_up_from_rest() {
        let mut ps = PowerSection::new(288.15);
        let mut n = 0.0;
        for _ in 0..200 {
            let out = ps.step(
                &Inputs {
                    ambient_pressure_pa: 101_325.0,
                    ambient_temperature_k: 288.15,
                    inlet_pressure_loss_frac: 0.0,
                    fuel_flow_kg_s: 0.0,
                    starter_torque_nm: 40.0,
                    accessory_torque_nm: 0.0,
                    dt_s: 0.1,
                },
                &no_faults(),
            );
            n = out.n_percent;
        }
        assert!(n > 0.0 && n.is_finite(), "{n}");
    }

    #[test]
    fn the_design_point_is_a_stable_equilibrium_of_its_own_torque_balance() {
        let mut ps = PowerSection::new(288.15);
        ps.n_percent = 100.0;
        let accessory_power_w = ps.design_available_accessory_power_w();
        let fuel = ps.fuel_flow_design_kg_s();
        let omega = ps.omega_rated_rad_s();
        let accessory_torque_nm = accessory_power_w / omega;

        for _ in 0..50 {
            ps.step(
                &Inputs {
                    ambient_pressure_pa: 101_325.0,
                    ambient_temperature_k: 288.15,
                    inlet_pressure_loss_frac: 0.0,
                    fuel_flow_kg_s: fuel,
                    starter_torque_nm: 0.0,
                    accessory_torque_nm,
                    dt_s: 0.05,
                },
                &no_faults(),
            );
        }
        assert!(
            (ps.n_percent() - 100.0).abs() < 5.0,
            "drifted to {:.2}%",
            ps.n_percent()
        );
    }

    #[test]
    fn more_accessory_load_than_the_design_point_provides_slows_the_spool() {
        let mut ps = PowerSection::new(288.15);
        ps.n_percent = 100.0;
        let fuel = ps.fuel_flow_design_kg_s();
        let omega = ps.omega_rated_rad_s();
        let overload_torque_nm = 2.0 * ps.design_available_accessory_power_w() / omega;

        let mut n = 100.0;
        for _ in 0..30 {
            let out = ps.step(
                &Inputs {
                    ambient_pressure_pa: 101_325.0,
                    ambient_temperature_k: 288.15,
                    inlet_pressure_loss_frac: 0.0,
                    fuel_flow_kg_s: fuel,
                    starter_torque_nm: 0.0,
                    accessory_torque_nm: overload_torque_nm,
                    dt_s: 0.05,
                },
                &no_faults(),
            );
            n = out.n_percent;
        }
        assert!(n < 99.0, "{n}");
    }

    #[test]
    fn compressor_erosion_raises_egt_for_the_same_speed_and_fuel_flow() {
        let mut healthy = PowerSection::new(288.15);
        healthy.n_percent = 100.0;
        let fuel = healthy.fuel_flow_design_kg_s();
        let out_healthy = healthy.step(
            &Inputs {
                ambient_pressure_pa: 101_325.0,
                ambient_temperature_k: 288.15,
                inlet_pressure_loss_frac: 0.0,
                fuel_flow_kg_s: fuel,
                starter_torque_nm: 0.0,
                accessory_torque_nm: 0.0,
                dt_s: 0.05,
            },
            &no_faults(),
        );

        let mut eroded = PowerSection::new(288.15);
        eroded.n_percent = 100.0;
        let out_eroded = eroded.step(
            &Inputs {
                ambient_pressure_pa: 101_325.0,
                ambient_temperature_k: 288.15,
                inlet_pressure_loss_frac: 0.0,
                fuel_flow_kg_s: fuel,
                starter_torque_nm: 0.0,
                accessory_torque_nm: 0.0,
                dt_s: 0.05,
            },
            &PowerSectionFaults {
                compressor_efficiency_loss: 0.4,
                turbine_efficiency_loss: 0.0,
            },
        );

        assert!(out_eroded.egt_c > out_healthy.egt_c, "{} {}", out_eroded.egt_c, out_healthy.egt_c);
    }

    #[test]
    fn turbine_damage_raises_egt_for_the_same_speed_and_fuel_flow() {
        let mut healthy = PowerSection::new(288.15);
        healthy.n_percent = 100.0;
        let fuel = healthy.fuel_flow_design_kg_s();
        let out_healthy = healthy.step(
            &Inputs {
                ambient_pressure_pa: 101_325.0,
                ambient_temperature_k: 288.15,
                inlet_pressure_loss_frac: 0.0,
                fuel_flow_kg_s: fuel,
                starter_torque_nm: 0.0,
                accessory_torque_nm: 0.0,
                dt_s: 0.05,
            },
            &no_faults(),
        );

        let mut damaged = PowerSection::new(288.15);
        damaged.n_percent = 100.0;
        let out_damaged = damaged.step(
            &Inputs {
                ambient_pressure_pa: 101_325.0,
                ambient_temperature_k: 288.15,
                inlet_pressure_loss_frac: 0.0,
                fuel_flow_kg_s: fuel,
                starter_torque_nm: 0.0,
                accessory_torque_nm: 0.0,
                dt_s: 0.05,
            },
            &PowerSectionFaults {
                compressor_efficiency_loss: 0.0,
                turbine_efficiency_loss: 0.4,
            },
        );

        assert!(out_damaged.egt_c > out_healthy.egt_c, "{} {}", out_damaged.egt_c, out_healthy.egt_c);
    }

    #[test]
    fn overspeed_and_egt_trip_thresholds_fire_only_past_their_limits() {
        let mut ps = PowerSection::new(288.15);
        assert!(!ps.overspeed_tripped());
        ps.n_percent = params::OVERSPEED_TRIP_PERCENT + 0.1;
        assert!(ps.overspeed_tripped());

        assert!(!ps.egt_over_hard_trip());
        ps.egt_k = params::EGT_TRIP_C + 273.15 + 1.0;
        assert!(ps.egt_over_hard_trip());
    }

    #[test]
    fn no_combustion_egt_decays_at_one_degree_c_per_second_not_faster() {
        let mut ps = PowerSection::new(288.15);
        ps.egt_k = 900.0 + 273.15;
        ps.n_percent = 50.0;

        let out = ps.step(
            &Inputs {
                ambient_pressure_pa: 101_325.0,
                ambient_temperature_k: 288.15,
                inlet_pressure_loss_frac: 0.0,
                fuel_flow_kg_s: 0.0,
                starter_torque_nm: 0.0,
                accessory_torque_nm: 0.0,
                dt_s: 1.0,
            },
            &no_faults(),
        );
        assert!(
            (out.egt_c - 899.0).abs() < 0.5,
            "one second of no-combustion decay should cost about 1 deg C, got {} (started at 900)",
            out.egt_c
        );

        let mut egt_after_30 = out.egt_c;
        for _ in 0..30 {
            let step_out = ps.step(
                &Inputs {
                    ambient_pressure_pa: 101_325.0,
                    ambient_temperature_k: 288.15,
                    inlet_pressure_loss_frac: 0.0,
                    fuel_flow_kg_s: 0.0,
                    starter_torque_nm: 0.0,
                    accessory_torque_nm: 0.0,
                    dt_s: 1.0,
                },
                &no_faults(),
            );
            egt_after_30 = step_out.egt_c;
        }
        assert!(
            egt_after_30 > 860.0,
            "31 s of no-combustion decay at 1 deg C/s should still be well above 860 deg C, got {egt_after_30}"
        );
    }

    #[test]
    fn a_blocked_inlet_reduces_delivered_compressor_power_at_the_same_speed() {
        let mut clear = PowerSection::new(288.15);
        clear.n_percent = 100.0;
        let out_clear = clear.step(
            &Inputs {
                ambient_pressure_pa: 101_325.0,
                ambient_temperature_k: 288.15,
                inlet_pressure_loss_frac: 0.0,
                fuel_flow_kg_s: 0.0,
                starter_torque_nm: 0.0,
                accessory_torque_nm: 0.0,
                dt_s: 0.05,
            },
            &no_faults(),
        );

        let mut blocked = PowerSection::new(288.15);
        blocked.n_percent = 100.0;
        let out_blocked = blocked.step(
            &Inputs {
                ambient_pressure_pa: 101_325.0,
                ambient_temperature_k: 288.15,
                inlet_pressure_loss_frac: 0.3,
                fuel_flow_kg_s: 0.0,
                starter_torque_nm: 0.0,
                accessory_torque_nm: 0.0,
                dt_s: 0.05,
            },
            &no_faults(),
        );

        assert!(out_blocked.core_pt3_pa < out_clear.core_pt3_pa);
    }
}
