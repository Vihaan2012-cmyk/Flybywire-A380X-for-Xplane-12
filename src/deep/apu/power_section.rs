//! The gas-generator core: one rotating spool carrying the power-section
//! (core) compressor, driven by a combustor/turbine energy-and-flow balance
//! -- item 1 of this directory's backlog.
//!
//! Unlike FlyByWire's own `pw980_physics.rs` (which bypasses a real
//! combustor/turbine gas path with a single fitted "fraction of fuel energy
//! extracted as shaft work" constant, `ETA_TURBINE_EXTRACTION`, documented
//! there as a deliberate reduced-order shortcut), this module runs the
//! actual chain: the compressor map (`compressor_map.rs`) sets compressor
//! exit conditions, the combustor (`combustor.rs`) is a real fuel/air energy
//! balance, and the turbine's flow capacity is Stodola's ellipse law
//! (`turbine_flow.rs`), which -- because continuity fixes the mass flow the
//! turbine must pass -- gives the operating pressure ratio (and with it,
//! EGT and shaft work) in closed form, with no iterative "engine matching"
//! solve needed:
//!
//! 1. The compressor runs its nominal corrected-flow operating line at the
//!    current spool speed (`compressor_map::evaluate`), fixing core airflow,
//!    exit temperature and exit pressure.
//! 2. The combustor's energy balance (`combustor::burn`) fixes turbine-inlet
//!    temperature from the fuel actually being metered this tick.
//! 3. Whatever mass the compressor is sending downstream, the turbine must
//!    pass (this lumped, quasi-steady model has no volume/plenum dynamics to
//!    store a mismatch in -- documented simplification, appropriate for a
//!    real-time model exactly as `physics/engine/turbine.rs`'s own module
//!    docs describe for the analogous main-engine case). Stodola's law,
//!    inverted, gives the pressure ratio that requires (`turbine_flow::
//!    pressure_ratio_for_flow`); expanding through it
//!    (`turbine_flow::expand`) gives the temperature the gas leaves at
//!    (this is where a real EGT probe sits: turbine exit, not combustor
//!    exit) and the shaft work recovered.
//! 4. A torque balance (`I * domega/dt = starter + turbine - compressor -
//!    accessories`) integrates the one modelled spool.
//!
//! The one-tick lag on `accessory_torque_nm` (load compressor + generators +
//! fixed windage, all computed from the *previous* tick's speed before this
//! tick's torque balance runs) is the same acausal-loop-breaking pattern
//! FlyByWire's own APU code documents for `last_elec_shaft_power`: closing
//! the loop exactly within one tick would need solving the whole gas path
//! and every shaft consumer simultaneously, which a fixed-timestep sim does
//! not need to do to be stable, provided the timestep is short compared to
//! the spool's own inertia -- true here (see this file's own governor
//! stability tests once wired to `governor.rs`).

use super::compressor_map;
use super::gas;
use super::params;
use super::turbine_flow;
use super::{combustor, combustor::Combustion};

/// Continuous wear/damage fractions this module accepts (0 healthy .. 1
/// fully degraded). Other fault types (IGV jam, surge valve, starter,
/// generator, oil, fuel control, inlet door, EGT sensor) live with the
/// subsystem they actually act on -- see `FAILURES.md`.
#[derive(Clone, Copy, Debug, Default)]
pub struct PowerSectionFaults {
    /// Core compressor erosion/damage: isentropic efficiency loss.
    pub compressor_efficiency_loss: f64,
    /// Turbine damage (erosion, FOD, cracked shrouds): isentropic
    /// efficiency loss.
    pub turbine_efficiency_loss: f64,
}

/// The design-point calibration: the compressor and turbine map shapes
/// (`params.rs`) plus the two free scale parameters this task's brief
/// allows deriving rather than measuring -- design fuel flow (solved so the
/// combustor reaches the chosen design turbine-inlet temperature) and the
/// turbine's Stodola flow-capacity coefficient (solved so the ellipse law
/// is satisfied exactly at that same design point).
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
    /// Net shaft power left over for accessories (electrical + bleed +
    /// windage) at the design point, once the compressor has taken its own
    /// share -- used both as this file's own self-consistency test target
    /// and as the governor's feed-forward starting point (`governor.rs`).
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

/// One evaluation of the authoritative steady gas-path chain at a given
/// operating point: compressor map -> combustor energy balance -> Stodola
/// turbine. [`PowerSection::step`] integrates the spool's torque balance
/// around exactly this, and `governor.rs`'s EGT-limit fuel schedule solves
/// this same chain for the fuel flow that lands on the limit -- so the
/// schedule and the physics cannot disagree about what a given fuel flow
/// does, which is what a separate first-order stand-in for the turbine
/// pressure ratio could not guarantee.
#[derive(Clone, Copy, Debug, Default)]
pub struct GasPath {
    pub compressor: compressor_map::Point,
    pub combustion: Combustion,
    pub expansion: turbine_flow::Expansion,
    /// The turbine pressure ratio Stodola's law requires to pass this flow.
    pub turbine_pressure_ratio: f64,
}

/// See [`GasPath`]. `n_frac` is spool speed as a fraction of rated,
/// `t1_k`/`p1_pa` the compressor's own inlet total conditions (already
/// reduced by whatever inlet loss the door/ice/obstruction imposes).
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
    // Erosion costs the compressor *flow capacity* as well as
    // efficiency: worn blading has lost chord and gained tip clearance,
    // so at the same corrected speed its throat passes less corrected
    // flow. Gas-path-analysis practice treats the two as running
    // together for erosion -- roughly one for one, a percent of
    // isentropic efficiency per percent of flow capacity (Kurz, R. &
    // Brun, K., "Degradation in Gas Turbine Systems", J. Eng. Gas
    // Turbines Power 123 (2001), 70-77, which tabulates exactly this
    // pairing for compressor fouling and erosion). This is the term that
    // makes an eroded core run *hotter* on the same metered fuel: less
    // air through the same combustor is a richer mixture, a higher
    // turbine-inlet temperature and a higher EGT. The efficiency loss on
    // its own cannot do that -- Euler work, and with it compressor-exit
    // temperature, is set by blade speed, not by efficiency.
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
    /// Fractional total-pressure loss the inlet (door position, icing,
    /// obstruction) is imposing on the compressor's inlet this tick, 0..~0.5.
    pub inlet_pressure_loss_frac: f64,
    /// Actual fuel mass flow the fuel control unit is metering this tick
    /// (`fuel_control.rs`'s output, not the governor's raw command).
    pub fuel_flow_kg_s: f64,
    pub starter_torque_nm: f64,
    /// Load compressor + generators + fixed accessory/windage torque this
    /// tick, evaluated at the *previous* tick's speed (one-tick lag, see
    /// module docs).
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

    /// The design-point calibration this spool was built with -- read by
    /// `governor.rs` so its EGT-limit schedule solves the *same* gas path
    /// (see [`gas_path`]) rather than a separately parameterised copy.
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

    /// The spool's actual angular velocity at its current speed -- used by
    /// `apu.rs` to feed `starter.rs`'s back-EMF calculation with the
    /// previous tick's value (the one-tick-lag pattern this directory uses
    /// throughout, see this file's own module docs).
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

    /// The rotor's inertia (`params::ROTOR_INERTIA_KG_M2`) is small relative
    /// to the torques this gas path produces (order of a hundred newton-
    /// metres at design power), the same combination FlyByWire's own
    /// `pw980_physics.rs` module docs note requires subdividing a caller's
    /// `dt` into bounded substeps for an explicit-Euler integration to stay
    /// accurate -- restated independently here as the brief's own
    /// "sub-stepping where stiff" convention requires, not copied from that
    /// file (the technique -- bound the step, average the compressor/
    /// combustor/turbine chain over each bounded step -- is standard
    /// explicit-integration practice, not specific to either file).
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

    /// One bounded-`dt` Euler step; `step`'s substep loop is the only
    /// caller.
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
            // No combustion: EGT relaxes toward the (now cooling) inlet air
            // as residual heat and windmilling airflow carry it away,
            // rather than jumping instantly to the cold compressor-exit
            // value the (fuel-less) expansion above would otherwise imply.
            //
            // Rate: 1 deg C/s, not an invented figure. A real hot section
            // is a substantial thermal mass that cools over *minutes*, not
            // tens of seconds -- the same reasoning FlyByWire's own
            // pw980_physics.rs documents for its EGT relaxation (that file
            // was itself found and fixed mid-audit: a previous 40 deg C/s
            // there "desynchronised from n2's own, much slower mechanical
            // coast-down", the same failure mode a bare, uncited 15 deg C/s
            // here would reproduce -- e.g. a 900 deg C EGT would wrongly
            // reach ambient in one minute, in step with a rotor that is
            // nowhere near spooled down that fast). 1 deg C/s also matches
            // this codebase's own sibling relaxation, `fadec.rs::polynomial
            // ::shutdown_egt`'s slow leg (0.00072756 * previous_egt-ish
            // decay constant, i.e. a long time constant once near the
            // steady baseline), and FlyByWire's `ShutdownPw980Turbine`/
            // `aps3200.rs`, both of which this file's module docs already
            // cite as using 1 deg C/s for exactly this state.
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

    /// The design point is, by construction, where turbine output exactly
    /// equals compressor absorption plus whatever accessory torque this
    /// calibration says is available -- an internal self-consistency check,
    /// not a number asserted from outside the model.
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
        // Twice the design-available accessory power: the spool cannot hold
        // speed without the (not-yet-modelled-here) governor raising fuel.
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

    /// A previous version of this decay used an uncited 15 deg C/s, which
    /// would cool a hot EGT to ambient in well under a minute -- far faster
    /// than the rotor's own mechanical coast-down (tens of seconds just to
    /// reach the light-off/self-sustaining boundary, see `apu.rs`'s own
    /// coast-down behaviour) and inconsistent with this file's cited 1 deg
    /// C/s (matching FlyByWire's own fixed `pw980_physics.rs`/`aps3200.rs`
    /// relaxation). A 900 deg C EGT losing fuel must still be within a few
    /// degrees of 900 after one second, and nowhere near ambient after 30.
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
        // Within a whisker of 900, not the ~15 deg C a 15 deg C/s rate (or
        // worse, ~900 minus substep artefacts) would have produced.
        assert!(
            (out.egt_c - 899.0).abs() < 0.5,
            "one second of no-combustion decay should cost about 1 deg C, got {} (started at 900)",
            out.egt_c
        );

        // After 30 more seconds, well above ambient: nowhere near the ~470
        // deg C total drop a 15 deg C/s rate would have produced over the
        // full 31 s.
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
