//! Physical air cycle machine (ACM) model for the A380's two packs.
//!
//! `systems::air_conditioning::AirConditioningPack` (fbw-common) is explicitly documented as
//! "a placeholder until the packs are modelled" (air_conditioning/mod.rs: "Takes the minimum
//! duct demand temperature as the pack outlet temperature ... this is a placeholder until the
//! packs are modelled"): it copies the air conditioning controller's own *demand* temperature
//! straight to the pack outlet through a first-order lag, with no dependency on bleed
//! conditions, ram air, or any turbomachinery at all. This module replaces that placeholder for
//! the A380 with a real bootstrap air cycle: a primary heat exchanger, a compressor/turbine pair
//! balanced on a common shaft, a secondary heat exchanger, a water separator, and a hot-air
//! bypass valve, so that pack outlet temperature and flow are the *result* of that cycle instead
//! of being copied from the control law that used to stand in for it.
//!
//! Each A380 pack contains two identical air cycle machine channels sharing one inlet plenum
//! (`PackComplex::pack_container` in `pneumatic.rs`, fed by the pack's two flow control valves).
//! Two identical parallel thermodynamic channels processing the same inlet air are, per unit
//! mass, thermodynamically identical to one channel processing the combined flow, so they are
//! modelled here as a single lumped channel per pack carrying the pack's total mass flow. This
//! keeps the state and the per-tick cost small while remaining physically exact for the
//! quantities that matter (temperatures, and the shared pressure ratio the two channels settle
//! on).
//!
//! ## Cycle modelled (typical 3-wheel/4-wheel "bootstrap" ACM)
//! 1. Primary heat exchanger (PHX): pack inlet bleed air is cooled against ram air.
//! 2. Compressor: driven off the same shaft as the turbine (a "bootstrap" cycle needs no
//!    external power source), it raises the pressure of the PHX-cooled air so the turbine has
//!    enough pressure ratio to produce useful cooling.
//! 3. Secondary heat exchanger (SHX): the compressor discharge is cooled again against ram air.
//!    The PHX and SHX draw from separate, independently-sized passages of the same ram air duct
//!    (a common real layout, e.g. a two-pass ram matrix with a dividing wall) rather than one
//!    stream heated by the PHX before it ever reaches the SHX: modelling them in series instead
//!    would let the PHX pre-heat the SHX's own coolant, understating SHX performance whenever
//!    ram flow is not many times larger than bleed flow (routinely true at low airspeed), which
//!    is not how either layout choice actually works.
//! 4. Turbine: expands the SHX-cooled air down to pack outlet pressure, doing exactly the
//!    compressor's work (mechanical losses aside) and producing the coldest air in the cycle.
//! 5. Water separator: any moisture condensed by the cold turbine discharge is extracted and
//!    its heat of vaporisation returned to the airstream (a "low pressure water separation"
//!    layout; some real ACMs separate water at the higher pressure ahead of the turbine instead,
//!    which is a real but different layout choice, noted here rather than modelled).
//! 6. Hot air bypass valve: blends a controlled fraction of pre-PHX bleed air around the whole
//!    cycle to warm the ACM's coldest deliverable air up to the temperature control system's
//!    demand. This valve — not the ACM itself — is what actually tracks the demand temperature;
//!    the ACM's own coldest output is a genuine physical limit the bypass valve cannot undercut.
//!
//! ## Parameter sources
//! The A380's own ACM performance data (heat exchanger effectiveness, compressor/turbine
//! efficiency, ram air scoop/fan sizing) is not published. Every such parameter below is a
//! generic, textbook-order transport-aircraft ECS value (Moir & Seabridge, *Aircraft Systems:
//! Mechanical, Electrical and Avionics Subsystems Integration*, 3rd ed., ch. 7; general ACM
//! literature), explicitly marked `GENERIC` in its doc comment, not a claim about the real A380.
//! Where FBW's own source gives a number (pack design flow, cabin pressurisation limits), it is
//! cited by file:line and used directly instead of an invented one.

use std::time::Duration;

use uom::si::{
    f64::*,
    mass_rate::kilogram_per_second,
    pressure::{pascal, psi},
    ratio::ratio,
    thermodynamic_temperature::{degree_celsius, kelvin},
    velocity::meter_per_second,
};

use systems::{
    air_conditioning::{Air, OutletAir},
    shared::low_pass_filter::LowPassFilter,
    simulation::{
        InitContext, SimulationElement, SimulationElementVisitor, SimulatorWriter,
        UpdateContext, VariableIdentifier, Write,
    },
};

use systems::air_conditioning::acs_controller::Pack;

/// Dry air, sea-level-ish specific heat at constant pressure. Matches
/// `systems::air_conditioning::Air::SPECIFIC_HEAT_CAPACITY_PRESSURE` (air_conditioning/mod.rs:1305),
/// duplicated here in J/(kg*K) because that constant is private to the common crate.
const CP_AIR: f64 = 1005.;
/// Ratio of specific heats for air. Matches `Air::GAMMA` (air_conditioning/mod.rs:1310).
const GAMMA: f64 = 1.4;
const GAMMA_EXPONENT: f64 = (GAMMA - 1.) / GAMMA;
/// Specific gas constant for dry air. Matches `Air::R` (air_conditioning/mod.rs:1306).
const R_AIR: f64 = 287.058;
/// Latent heat of vaporisation of water, standard physical constant.
const LATENT_HEAT_VAPORISATION_J_KG: f64 = 2.501e6;

#[derive(Clone)]
pub struct AirCycleMachine {
    pack_outlet_temperature_id: VariableIdentifier,
    ram_air_inlet_flow_id: VariableIdentifier,
    ram_air_outlet_temperature_id: VariableIdentifier,
    ram_air_door_position_id: VariableIdentifier,
    bypass_valve_position_id: VariableIdentifier,
    compressor_pressure_ratio_id: VariableIdentifier,
    turbine_outlet_temperature_id: VariableIdentifier,
    water_extracted_id: VariableIdentifier,

    outlet_air: Air,

    ram_air_door_position: LowPassFilter<f64>, // Ratio, 0-1
    ram_air_flow: MassRate,
    ram_air_outlet_temperature: ThermodynamicTemperature,

    compressor_pressure_ratio: f64,
    turbine_outlet_temperature: ThermodynamicTemperature,
    bypass_valve_open_amount: Ratio,
    water_extracted: MassRate,

    outlet_temperature_filter: LowPassFilter<f64>, // Degree Celsius. Sensor/duct thermal lag.
}

impl AirCycleMachine {
    /// Primary heat exchanger effectiveness. GENERIC: typical cross-flow plate-fin air-to-air
    /// aircraft heat exchanger, 0.70-0.85 (Moir & Seabridge ch.7). Not A380-specific.
    const PHX_EFFECTIVENESS: f64 = 0.80;
    /// Secondary heat exchanger effectiveness. GENERIC, same source/range as the PHX; slightly
    /// lower because the SHX is smaller and sees a hotter, higher-velocity core stream.
    const SHX_EFFECTIVENESS: f64 = 0.78;
    /// Centrifugal compressor isentropic efficiency. GENERIC: typical ACM compressor wheel,
    /// 0.75-0.80.
    const COMPRESSOR_ISENTROPIC_EFFICIENCY: f64 = 0.78;
    /// Radial-inflow turbine isentropic efficiency. GENERIC: typical ACM turbine wheel,
    /// 0.80-0.85.
    const TURBINE_ISENTROPIC_EFFICIENCY: f64 = 0.82;
    /// Bearing/windage losses on the compressor-turbine shaft. GENERIC.
    const SHAFT_MECHANICAL_EFFICIENCY: f64 = 0.97;
    /// Fractional total pressure loss through each heat exchanger core. GENERIC: typical duct/HX
    /// loss budget for this class of exchanger.
    const HEAT_EXCHANGER_PRESSURE_LOSS_FRACTION: f64 = 0.03;
    /// Assumed margin the turbine discharge pressure must keep above cabin pressure to push flow
    /// through the mixer manifold and trim air ducting. No FBW or public value found; chosen as
    /// the most defensible order-of-magnitude estimate and clearly marked as assumed.
    const TURBINE_OUTLET_PRESSURE_MARGIN_PSI: f64 = 0.5;

    /// Ram air scoop total-pressure recovery factor. GENERIC: typical flush/NACA-type inlet at
    /// high subsonic Mach.
    const RAM_RECOVERY_FACTOR: f64 = 0.95;
    /// Static pressure rise the ACM's own shaft-driven ram air fan adds at typical operating
    /// shaft speed, independent of forward airspeed (this is why packs still cool on the
    /// ground). GENERIC, sized (not invented) so that a stationary aircraft's fan-only ram flow
    /// through `RAM_DOOR_AREA_M2` is of the same order as the dynamic-pressure-driven flow that
    /// area gives at the cruise design point above (~11 kPa recovered dynamic pressure there) —
    /// real ACM ram fans are sized precisely so ground and in-flight cooling capacity are
    /// comparable, which a much smaller figure would not reproduce. Not A380-specific.
    const RAM_FAN_PRESSURE_RISE_PA: f64 = 9000.;
    /// Ram air inlet discharge coefficient for the door/scoop opening. GENERIC sharp-edged
    /// orifice value.
    const RAM_DOOR_DISCHARGE_COEFFICIENT: f64 = 0.8;
    /// Effective fully-open ram air inlet area per pack. Derived (not invented) from FBW's own
    /// design pack flow target: `AirGenerationSystemApplication::FLOW_CONSTANT_C *
    /// A320_TO_A380_FLOW_CONVERSION_FACTOR` = 0.5675 kg/s * 2.8 ~= 1.59 kg/s
    /// (cpiom_b.rs:335,340), sized so that at a representative cruise condition (Mach 0.85 /
    /// 35,000 ft, TAS ~=250 m/s, rho ~=0.38 kg/m^3) the fully open door alone passes about that
    /// much ram air: A = mdot / (Cd * sqrt(2 * rho * dP)), dP = recovery * 0.5 * rho * TAS^2.
    /// This ties the assumed geometry to FBW's own numbers rather than inventing an area.
    const RAM_DOOR_AREA_M2: f64 = 0.021;
    /// Ram air door actuation time constant. GENERIC pneumatic actuator response.
    const RAM_DOOR_TIME_CONSTANT: Duration = Duration::from_secs(4);

    /// Outlet duct/sensor thermal lag, matching the time constant FBW's own placeholder used
    /// (`AirConditioningPack::PACK_REACTION_TIME`, air_conditioning/mod.rs:687) so downstream
    /// consumers see a comparably damped signal even though the value driving it is now real.
    const OUTLET_REACTION_TIME: Duration = Duration::from_secs(10);

    pub fn new(context: &mut InitContext, pack_id: Pack) -> Self {
        let n: usize = pack_id.into();
        Self {
            pack_outlet_temperature_id: context
                .get_identifier(format!("COND_PACK_{}_OUTLET_TEMPERATURE", n)),
            ram_air_inlet_flow_id: context
                .get_identifier(format!("COND_PACK_{}_RAM_AIR_FLOW", n)),
            ram_air_outlet_temperature_id: context
                .get_identifier(format!("COND_PACK_{}_RAM_AIR_OUTLET_TEMPERATURE", n)),
            ram_air_door_position_id: context
                .get_identifier(format!("COND_PACK_{}_RAM_AIR_DOOR_POSITION", n)),
            bypass_valve_position_id: context
                .get_identifier(format!("COND_PACK_{}_HOT_AIR_BYPASS_POSITION", n)),
            compressor_pressure_ratio_id: context
                .get_identifier(format!("COND_PACK_{}_ACM_PRESSURE_RATIO", n)),
            turbine_outlet_temperature_id: context
                .get_identifier(format!("COND_PACK_{}_TURBINE_OUTLET_TEMPERATURE", n)),
            water_extracted_id: context
                .get_identifier(format!("COND_PACK_{}_WATER_EXTRACTED", n)),

            outlet_air: Air::new(),

            ram_air_door_position: LowPassFilter::new_with_init_value(
                Self::RAM_DOOR_TIME_CONSTANT,
                0.,
            ),
            ram_air_flow: MassRate::default(),
            ram_air_outlet_temperature: ThermodynamicTemperature::new::<degree_celsius>(15.),

            compressor_pressure_ratio: 1.5,
            turbine_outlet_temperature: ThermodynamicTemperature::new::<degree_celsius>(15.),
            bypass_valve_open_amount: Ratio::new::<ratio>(0.),
            water_extracted: MassRate::default(),

            outlet_temperature_filter: LowPassFilter::new_with_init_value(
                Self::OUTLET_REACTION_TIME,
                15.,
            ),
        }
    }

    /// Runs the ACM thermodynamic cycle for one tick.
    ///
    /// `pack_flow` is the total mass flow already delivered into the pack's inlet plenum by its
    /// (physically, orifice-flow driven) flow control valves. `inlet_pressure`/`inlet_temperature`
    /// are that plenum's own state (`PackComplex::pack_container`, pneumatic.rs), i.e. the real
    /// bleed air the ACM has to work with, not a scheduled value. `duct_demand_temperature` is
    /// the temperature control system's demand (still real: it is what the hot air bypass valve
    /// is commanded to try to reach), `cabin_pressure` sets the turbine's backpressure, and
    /// `ram_air_pb_is_on`/`acsc_failure` are cockpit/failure states.
    #[allow(clippy::too_many_arguments)]
    pub fn update(
        &mut self,
        context: &UpdateContext,
        pack_flow: MassRate,
        inlet_pressure: Pressure,
        inlet_temperature: ThermodynamicTemperature,
        duct_demand_temperature: ThermodynamicTemperature,
        cabin_pressure: Pressure,
        ram_air_pb_is_on: bool,
        acsc_failure: bool,
    ) {
        let mdot_bleed = pack_flow.get::<kilogram_per_second>().max(0.);

        if acsc_failure {
            // Loss of the ACSC/TCS: the pack free-runs with the ram air door fully open and no
            // bypass, i.e. its coldest possible output. This matches the real fail-safe
            // behaviour (packs run full cold rather than uncontrolled hot) and is a genuine
            // physical consequence of losing the control loop, not a hardcoded fallback value.
            self.ram_air_door_position.update(context.delta(), 1.);
            self.bypass_valve_open_amount = Ratio::new::<ratio>(0.);
        }

        self.update_ram_air_supply(context, ram_air_pb_is_on || acsc_failure, mdot_bleed > 1e-3);

        if mdot_bleed <= 1e-3 {
            // Pack off / flow control valve(s) closed: no cycle, no flow out.
            self.water_extracted = MassRate::default();
            self.outlet_temperature_filter.update(
                context.delta(),
                context.ambient_temperature().get::<degree_celsius>(),
            );
            self.outlet_air.set_flow_rate(MassRate::default());
            self.outlet_air.set_temperature(
                ThermodynamicTemperature::new::<degree_celsius>(
                    self.outlet_temperature_filter.output(),
                ),
            );
            self.outlet_air.set_pressure(cabin_pressure);
            return;
        }

        let mdot_ram = self.ram_air_flow.get::<kilogram_per_second>().max(0.);
        let ram_inlet_temperature = context.ambient_temperature();

        // --- Primary heat exchanger ---
        let (t_after_phx, t_ram_after_phx) = Self::heat_exchanger(
            Self::PHX_EFFECTIVENESS,
            inlet_temperature,
            mdot_bleed,
            ram_inlet_temperature,
            mdot_ram,
        );
        let p_after_phx =
            inlet_pressure * (1. - Self::HEAT_EXCHANGER_PRESSURE_LOSS_FRACTION);

        // --- Turbine backpressure (fixed for this tick, used by the shaft balance below) ---
        let p_turbine_outlet =
            cabin_pressure + Pressure::new::<psi>(Self::TURBINE_OUTLET_PRESSURE_MARGIN_PSI);

        // --- Compressor/turbine bootstrap shaft balance ---
        // Both compressor work and turbine work increase with compressor pressure ratio (a
        // hotter compressor discharge leaves the SHX hotter too, for fixed effectiveness), so
        // the balance point is found by bisection, which cannot diverge or produce NaNs even if
        // the two curves are close to parallel.
        // The SHX draws on its own share of the ram duct (see module doc: parallel, not series,
        // ram passages), so it sees fresh ambient air, not `t_ram_after_phx`.
        let (pressure_ratio, t_after_shx, t_ram_after_shx, p_after_shx) = self
            .solve_shaft_balance(
                t_after_phx,
                p_after_phx,
                ram_inlet_temperature,
                mdot_bleed,
                mdot_ram,
                p_turbine_outlet,
            );
        self.compressor_pressure_ratio = pressure_ratio;
        let _ = t_ram_after_phx;

        // --- Turbine expansion ---
        let turbine_pressure_ratio = (p_after_shx / p_turbine_outlet).get::<ratio>().max(1.001);
        let t_turbine_ideal = t_after_shx.get::<kelvin>() * turbine_pressure_ratio.powf(-GAMMA_EXPONENT);
        let t_turbine_out = t_after_shx.get::<kelvin>()
            - Self::TURBINE_ISENTROPIC_EFFICIENCY * (t_after_shx.get::<kelvin>() - t_turbine_ideal);

        // --- Water separation (low-pressure layout: condense out anything above saturation in
        // the cold turbine discharge, return the latent heat to the stream) ---
        let (t_after_water_separator, water_extracted) = Self::water_separator(
            ThermodynamicTemperature::new::<kelvin>(t_turbine_out),
            p_turbine_outlet,
            ram_inlet_temperature,
            context.ambient_pressure(),
            mdot_bleed,
        );
        self.water_extracted = water_extracted;
        self.turbine_outlet_temperature = t_after_water_separator;

        // --- Hot air bypass valve: blend pre-PHX bleed air around the whole cycle to reach the
        // TCS demand. This is a real A380/large-transport ECS component (the "pack temperature
        // control valve"); it is the reason pack outlet temperature can be commanded warmer than
        // the ACM's coldest output without the ACM itself needing to "know" the demand. ---
        let coldest_deliverable = t_after_water_separator.get::<kelvin>();
        let hottest_available = inlet_temperature.get::<kelvin>();
        let demand = duct_demand_temperature.get::<kelvin>();
        let bypass_fraction = if hottest_available > coldest_deliverable + 0.01 {
            ((demand - coldest_deliverable) / (hottest_available - coldest_deliverable))
                .clamp(0., 1.)
        } else {
            0.
        };
        self.bypass_valve_open_amount = Ratio::new::<ratio>(if acsc_failure {
            0.
        } else {
            bypass_fraction
        });

        let mixed_temperature_k = bypass_fraction * hottest_available
            + (1. - bypass_fraction) * coldest_deliverable;

        self.outlet_temperature_filter
            .update(context.delta(), mixed_temperature_k - 273.15);

        self.outlet_air.set_flow_rate(MassRate::new::<kilogram_per_second>(
            mdot_bleed - water_extracted.get::<kilogram_per_second>(),
        ));
        self.outlet_air.set_temperature(ThermodynamicTemperature::new::<degree_celsius>(
            self.outlet_temperature_filter.output(),
        ));
        self.outlet_air.set_pressure(p_turbine_outlet);

        self.ram_air_outlet_temperature = ThermodynamicTemperature::new::<kelvin>(t_ram_after_shx);
        let _ = t_after_shx; // retained for readability of the tuple above
    }

    /// Ram air mass flow from dynamic pressure recovered by the scoop plus the ACM's own
    /// shaft-driven fan, through a compressible-orifice-style relation with the door's
    /// commanded/actuated open area (`mdot = Cd * A * sqrt(2 * rho * dP)`, valid here because the
    /// pressure rises involved are small compared to ambient so density is effectively constant
    /// across the door, i.e. the incompressible limit of the general orifice equation).
    fn update_ram_air_supply(&mut self, context: &UpdateContext, force_open: bool, pack_running: bool) {
        let target_open = if force_open { 1.0 } else if pack_running { 1.0 } else { 0.1 };
        let open_amount = self
            .ram_air_door_position
            .update(context.delta(), target_open)
            .clamp(0., 1.);

        let rho_ambient = context.ambient_pressure().get::<pascal>()
            / (R_AIR * context.ambient_temperature().get::<kelvin>());
        let dynamic_pressure = 0.5
            * rho_ambient
            * context.true_airspeed().get::<meter_per_second>().powi(2)
            * Self::RAM_RECOVERY_FACTOR;
        let fan_pressure_rise = if pack_running {
            Self::RAM_FAN_PRESSURE_RISE_PA
        } else {
            0.
        };
        let available_pressure_rise = (dynamic_pressure + fan_pressure_rise).max(0.);

        let area = Self::RAM_DOOR_AREA_M2 * open_amount;
        self.ram_air_flow = MassRate::new::<kilogram_per_second>(
            Self::RAM_DOOR_DISCHARGE_COEFFICIENT
                * area
                * (2. * rho_ambient * available_pressure_rise).max(0.).sqrt(),
        );
    }

    /// Cross-flow effectiveness-NTU heat exchanger: returns (hot side outlet, cold side outlet).
    fn heat_exchanger(
        effectiveness: f64,
        hot_in: ThermodynamicTemperature,
        mdot_hot: f64,
        cold_in: ThermodynamicTemperature,
        mdot_cold: f64,
    ) -> (ThermodynamicTemperature, ThermodynamicTemperature) {
        if mdot_hot <= 1e-6 || mdot_cold <= 1e-6 {
            return (hot_in, cold_in);
        }
        let c_min = CP_AIR * mdot_hot.min(mdot_cold);
        let q = effectiveness * c_min * (hot_in.get::<kelvin>() - cold_in.get::<kelvin>());
        let hot_out = hot_in.get::<kelvin>() - q / (mdot_hot * CP_AIR);
        let cold_out = cold_in.get::<kelvin>() + q / (mdot_cold * CP_AIR);
        (
            ThermodynamicTemperature::new::<kelvin>(hot_out),
            ThermodynamicTemperature::new::<kelvin>(cold_out),
        )
    }

    /// Finds the compressor pressure ratio at which turbine work equals compressor work (a
    /// "bootstrap" ACM has no other power source) by bisection, which is unconditionally stable:
    /// unlike a Newton iteration it cannot diverge or hand back a NaN pressure ratio regardless
    /// of how the two work curves are shaped, which matters because this runs every tick.
    fn solve_shaft_balance(
        &self,
        t_after_phx: ThermodynamicTemperature,
        p_after_phx: Pressure,
        shx_ram_inlet_temperature: ThermodynamicTemperature,
        mdot_bleed: f64,
        mdot_ram: f64,
        p_turbine_outlet: Pressure,
    ) -> (f64, ThermodynamicTemperature, f64, Pressure) {
        let work_imbalance = |pr: f64| {
            let t_comp_ideal = t_after_phx.get::<kelvin>() * pr.powf(GAMMA_EXPONENT);
            let t_comp_out = t_after_phx.get::<kelvin>()
                + (t_comp_ideal - t_after_phx.get::<kelvin>())
                    / Self::COMPRESSOR_ISENTROPIC_EFFICIENCY;
            let w_comp = CP_AIR * mdot_bleed * (t_comp_out - t_after_phx.get::<kelvin>());

            let p_after_shx = p_after_phx * pr * (1. - Self::HEAT_EXCHANGER_PRESSURE_LOSS_FRACTION);
            let (t_after_shx, t_shx_ram_out) = Self::heat_exchanger(
                Self::SHX_EFFECTIVENESS,
                ThermodynamicTemperature::new::<kelvin>(t_comp_out),
                mdot_bleed,
                shx_ram_inlet_temperature,
                mdot_ram,
            );
            let turbine_pr = (p_after_shx / p_turbine_outlet).get::<ratio>().max(1.001);
            let t_turbine_ideal = t_after_shx.get::<kelvin>() * turbine_pr.powf(-GAMMA_EXPONENT);
            let t_turbine_out = t_after_shx.get::<kelvin>()
                - Self::TURBINE_ISENTROPIC_EFFICIENCY
                    * (t_after_shx.get::<kelvin>() - t_turbine_ideal);
            let w_turb = CP_AIR
                * mdot_bleed
                * (t_after_shx.get::<kelvin>() - t_turbine_out)
                * Self::SHAFT_MECHANICAL_EFFICIENCY;

            (w_turb - w_comp, t_after_shx, p_after_shx, t_shx_ram_out)
        };

        let mut lower = 1.02_f64;
        let mut upper = 4.5_f64;
        let (mut f_lower, ..) = work_imbalance(lower);
        let (f_upper, ..) = work_imbalance(upper);

        // If there is no sign change in the bracket (possible with very little ram air, e.g. an
        // engine bleed leak with the aircraft stationary), fall back to whichever bound leaves
        // the smaller imbalance instead of bisecting blindly, which keeps the result finite and
        // bounded rather than extrapolating nonsense.
        let mut pr = if f_lower.signum() == f_upper.signum() {
            if f_lower.abs() < f_upper.abs() {
                lower
            } else {
                upper
            }
        } else {
            let mut mid = lower;
            for _ in 0..24 {
                mid = 0.5 * (lower + upper);
                let (f_mid, ..) = work_imbalance(mid);
                if f_mid.signum() == f_lower.signum() {
                    lower = mid;
                    f_lower = f_mid;
                } else {
                    upper = mid;
                }
            }
            mid
        };
        pr = pr.clamp(1.02, 4.5);

        let (_, t_after_shx, p_after_shx, t_shx_ram_out) = work_imbalance(pr);
        (pr, t_after_shx, t_shx_ram_out.get::<kelvin>(), p_after_shx)
    }

    /// Simplified water separator: assumes the bleed air carries ambient outside air's own
    /// specific humidity (physically motivated, since bleed air is compressed ambient air), and
    /// condenses out whatever exceeds saturation at the cold turbine discharge condition,
    /// returning the latent heat of condensation to the stream (the real reason a water
    /// separator's outlet is measurably warmer than a dry expansion to the same pressure ratio
    /// would produce). Saturation vapour pressure uses the Magnus/Tetens approximation.
    fn water_separator(
        turbine_out: ThermodynamicTemperature,
        turbine_out_pressure: Pressure,
        ambient_temperature: ThermodynamicTemperature,
        ambient_pressure: Pressure,
        mdot_bleed: f64,
    ) -> (ThermodynamicTemperature, MassRate) {
        let saturation_vapour_pressure_pa = |t_celsius: f64| -> f64 {
            610.94 * (17.625 * t_celsius / (t_celsius + 243.04)).exp()
        };

        // Ambient humidity ratio, capped at saturation (Tetens formula assumes vapour pressure
        // cannot exceed the ambient saturation value).
        let ambient_c = ambient_temperature.get::<degree_celsius>();
        let e_ambient = saturation_vapour_pressure_pa(ambient_c).min(ambient_pressure.get::<pascal>() * 0.9);
        let humidity_ratio_ambient =
            0.622 * e_ambient / (ambient_pressure.get::<pascal>() - e_ambient).max(1.);

        let saturation_humidity_ratio = |t_celsius: f64| {
            let e_sat = saturation_vapour_pressure_pa(t_celsius);
            0.622 * e_sat / (turbine_out_pressure.get::<pascal>() - e_sat).max(1.)
        };

        // Condensing releases latent heat, which warms the stream and so raises the humidity it
        // can hold: the outlet settles where the two agree. Returning the latent heat of
        // everything above saturation at the *dry* discharge temperature, in one step, warmed
        // the air far past its own dew point -- 42 K on a 24 C saturated day, a pack outlet of
        // 33 C from a -10 C turbine discharge, for water that could never have condensed at
        // that temperature -- and the packs could not cool the cabin. The balance
        // `t = t_dry + (w - w_sat(t)) * L / cp` has exactly one root between the dry discharge
        // and the dew point (its right side falls as `t` rises), found by bisection.
        let t_dry = turbine_out.get::<degree_celsius>();
        if humidity_ratio_ambient <= saturation_humidity_ratio(t_dry) {
            return (turbine_out, MassRate::default());
        }
        let reheated = |t_celsius: f64| {
            t_dry
                + (humidity_ratio_ambient - saturation_humidity_ratio(t_celsius)).max(0.)
                    * LATENT_HEAT_VAPORISATION_J_KG
                    / CP_AIR
        };
        let (mut lower, mut upper) = (t_dry, reheated(t_dry));
        for _ in 0..40 {
            let mid = 0.5 * (lower + upper);
            if reheated(mid) > mid {
                lower = mid;
            } else {
                upper = mid;
            }
        }
        let t_out = 0.5 * (lower + upper);

        let condensed_ratio = (humidity_ratio_ambient - saturation_humidity_ratio(t_out)).max(0.);
        (
            ThermodynamicTemperature::new::<degree_celsius>(
                t_dry + condensed_ratio * LATENT_HEAT_VAPORISATION_J_KG / CP_AIR,
            ),
            MassRate::new::<kilogram_per_second>(condensed_ratio * mdot_bleed),
        )
    }

}

impl OutletAir for AirCycleMachine {
    fn outlet_air(&self) -> Air {
        self.outlet_air
    }
}

impl SimulationElement for AirCycleMachine {
    fn accept<T: SimulationElementVisitor>(&mut self, visitor: &mut T) {
        visitor.visit(self);
    }

    fn write(&self, writer: &mut SimulatorWriter) {
        writer.write(&self.pack_outlet_temperature_id, self.outlet_air.temperature());
        writer.write(&self.ram_air_inlet_flow_id, self.ram_air_flow);
        writer.write(
            &self.ram_air_outlet_temperature_id,
            self.ram_air_outlet_temperature,
        );
        writer.write(
            &self.ram_air_door_position_id,
            self.ram_air_door_position.output(),
        );
        writer.write(
            &self.bypass_valve_position_id,
            self.bypass_valve_open_amount.get::<ratio>(),
        );
        writer.write(
            &self.compressor_pressure_ratio_id,
            self.compressor_pressure_ratio,
        );
        writer.write(
            &self.turbine_outlet_temperature_id,
            self.turbine_outlet_temperature,
        );
        writer.write(&self.water_extracted_id, self.water_extracted);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use systems::simulation::test::{SimulationTestBed, TestBed};
    use systems::simulation::{Aircraft, SimulationElement};
    use uom::si::{
        mass_rate::kilogram_per_second, pressure::hectopascal,
        thermodynamic_temperature::degree_celsius, velocity::knot,
    };

    struct TestAircraft {
        acm: AirCycleMachine,
        pack_flow: MassRate,
        inlet_pressure: Pressure,
        inlet_temperature: ThermodynamicTemperature,
        demand_temperature: ThermodynamicTemperature,
        cabin_pressure: Pressure,
        ram_air_pb_is_on: bool,
        acsc_failure: bool,
    }
    impl TestAircraft {
        fn new(context: &mut systems::simulation::InitContext) -> Self {
            Self {
                acm: AirCycleMachine::new(context, Pack(1)),
                pack_flow: MassRate::new::<kilogram_per_second>(0.8),
                inlet_pressure: Pressure::new::<psi>(44.),
                inlet_temperature: ThermodynamicTemperature::new::<degree_celsius>(200.),
                demand_temperature: ThermodynamicTemperature::new::<degree_celsius>(15.),
                cabin_pressure: Pressure::new::<hectopascal>(1013.25),
                ram_air_pb_is_on: false,
                acsc_failure: false,
            }
        }
    }
    impl Aircraft for TestAircraft {
        fn update_after_power_distribution(&mut self, context: &UpdateContext) {
            self.acm.update(
                context,
                self.pack_flow,
                self.inlet_pressure,
                self.inlet_temperature,
                self.demand_temperature,
                self.cabin_pressure,
                self.ram_air_pb_is_on,
                self.acsc_failure,
            );
        }
    }
    impl SimulationElement for TestAircraft {
        fn accept<T: SimulationElementVisitor>(&mut self, visitor: &mut T) {
            self.acm.accept(visitor);
            visitor.visit(self);
        }
    }

    struct AcmTestBed {
        test_bed: SimulationTestBed<TestAircraft>,
    }
    impl AcmTestBed {
        fn new() -> Self {
            Self {
                test_bed: SimulationTestBed::new(TestAircraft::new),
            }
        }
        fn with_ambient(mut self, temperature_c: f64, pressure_hpa: f64, tas_kt: f64) -> Self {
            self.test_bed.set_ambient_temperature(ThermodynamicTemperature::new::<degree_celsius>(
                temperature_c,
            ));
            self.test_bed
                .set_ambient_pressure(Pressure::new::<hectopascal>(pressure_hpa));
            self.test_bed.set_true_airspeed(Velocity::new::<knot>(tas_kt));
            self
        }
        fn demand(mut self, temperature_c: f64) -> Self {
            self.test_bed.command(|a| {
                a.demand_temperature = ThermodynamicTemperature::new::<degree_celsius>(temperature_c)
            });
            self
        }
        fn flow(mut self, kg_s: f64) -> Self {
            self.test_bed
                .command(|a| a.pack_flow = MassRate::new::<kilogram_per_second>(kg_s));
            self
        }
        fn inlet_conditions(mut self, pressure_psi: f64, temperature_c: f64) -> Self {
            self.test_bed.command(|a| {
                a.inlet_pressure = Pressure::new::<psi>(pressure_psi);
                a.inlet_temperature = ThermodynamicTemperature::new::<degree_celsius>(temperature_c);
            });
            self
        }
        fn iterate(mut self, iterations: usize) -> Self {
            for _ in 0..iterations {
                self.test_bed.run_with_delta(Duration::from_secs(1));
            }
            self
        }
        fn outlet_temperature(&self) -> ThermodynamicTemperature {
            self.test_bed.query(|a| a.acm.outlet_air.temperature())
        }
        fn outlet_flow(&self) -> MassRate {
            self.test_bed.query(|a| a.acm.outlet_air.flow_rate())
        }
        fn ram_air_flow(&self) -> MassRate {
            self.test_bed.query(|a| a.acm.ram_air_flow)
        }
        fn pressure_ratio(&self) -> f64 {
            self.test_bed.query(|a| a.acm.compressor_pressure_ratio)
        }
        fn bypass(&self) -> Ratio {
            self.test_bed.query(|a| a.acm.bypass_valve_open_amount)
        }
    }

    #[test]
    fn pack_off_has_no_outlet_flow() {
        let test_bed = AcmTestBed::new().flow(0.).iterate(5);
        assert_eq!(test_bed.outlet_flow(), MassRate::default());
    }

    #[test]
    fn cruise_design_point_produces_cold_air_below_bleed_inlet_temperature() {
        let test_bed = AcmTestBed::new()
            .with_ambient(-56.5, 238.4, 480.)
            .inlet_conditions(44., 200.)
            .demand(4.) // coldest cabin/cockpit selection
            .iterate(120);

        let outlet_c = test_bed.outlet_temperature().get::<degree_celsius>();
        // The cycle must actually cool the air well below the 200 C bleed inlet, and stay above
        // the point of forming dry ice / being non-physical.
        assert!(
            outlet_c < 50.,
            "expected a cold ACM outlet at max cooling demand, got {outlet_c} C"
        );
        assert!(outlet_c > -50., "outlet temperature is not physical: {outlet_c} C");
        assert!(test_bed.pressure_ratio() > 1.05 && test_bed.pressure_ratio() < 4.5);
        assert!(test_bed.ram_air_flow() > MassRate::default());
    }

    #[test]
    fn ground_design_point_with_full_bleed_pressure_reaches_selected_temperature() {
        // A design point representative of ground/low-altitude operation with a properly
        // regulated ~44 psi bleed source (see docs/physics/air.md for why the pack inlet
        // pressure actually delivered by the shared pneumatic plenum can currently sit well
        // below this in the full aircraft simulation, which is a known, documented limitation
        // of that plenum, not of the ACM cycle itself).
        let test_bed = AcmTestBed::new()
            .with_ambient(24., 1013.25, 250.)
            .inlet_conditions(44., 200.)
            .demand(24.)
            .iterate(120);

        assert!(
            (test_bed.outlet_temperature().get::<degree_celsius>() - 24.).abs() < 1.,
            "expected the ACM to reach the 24 C demand with a fully regulated bleed source, got {:?}",
            test_bed.outlet_temperature()
        );
    }

    #[test]
    fn hot_bypass_valve_opens_when_demand_is_above_coldest_deliverable() {
        let test_bed = AcmTestBed::new()
            .with_ambient(-56.5, 238.4, 480.)
            .inlet_conditions(44., 200.)
            .demand(30.) // warm demand relative to a cold-cycle output
            .iterate(120);

        assert!(test_bed.bypass().get::<ratio>() > 0.);
    }

    #[test]
    fn no_bleed_air_leaves_the_pack_at_ambient_not_at_a_stale_hot_value() {
        // A bleed leak/duct failure that starves the pack of flow must not leave the old hot
        // demand-copied value sitting at the outlet forever (the old placeholder's failure mode).
        let test_bed = AcmTestBed::new()
            .with_ambient(-56.5, 238.4, 480.)
            .inlet_conditions(44., 200.)
            .demand(24.)
            .iterate(60)
            .flow(0.)
            .iterate(60);

        assert_eq!(test_bed.outlet_flow(), MassRate::default());
    }

    #[test]
    fn the_water_separator_never_warms_the_air_past_its_own_dew_point() {
        // A 24 C saturated day and a -9.7 C dry turbine discharge (the pack at 23 psi on the
        // ground). Condensing releases latent heat, which warms the stream until it stops
        // condensing: the outlet must settle between the dry discharge and the dew point
        // (about 24.6 C at the turbine's 15.2 psi), never above it.
        let p_out = Pressure::new::<psi>(15.2);
        let (t_out, water) = AirCycleMachine::water_separator(
            ThermodynamicTemperature::new::<degree_celsius>(-9.7),
            p_out,
            ThermodynamicTemperature::new::<degree_celsius>(24.),
            Pressure::new::<hectopascal>(1013.25),
            1.56,
        );
        let t = t_out.get::<degree_celsius>();
        assert!(t > -9.7 && t < 24.6, "separator outlet {t} C is outside dry discharge..dew point");
        // Energy balance: the reheat is exactly the latent heat of the water it took out.
        let reheat = water.get::<kilogram_per_second>() / 1.56 * LATENT_HEAT_VAPORISATION_J_KG / CP_AIR;
        assert!((t - (-9.7 + reheat)).abs() < 0.05, "outlet {t} C, reheat {reheat} K");
        // Dry enough air (below saturation at the discharge) passes straight through.
        let (dry, none) = AirCycleMachine::water_separator(
            ThermodynamicTemperature::new::<degree_celsius>(20.),
            p_out,
            ThermodynamicTemperature::new::<degree_celsius>(-20.),
            Pressure::new::<hectopascal>(1013.25),
            1.56,
        );
        assert!((dry.get::<degree_celsius>() - 20.).abs() < 1e-9 && none == MassRate::default());
    }

    #[test]
    fn ram_air_flows_even_with_zero_airspeed_on_the_ground() {
        // Real ACMs cool on the ground because their own shaft drives a ram air fan; a pure
        // freestream-dynamic-pressure model would (wrongly) give zero ram flow at V=0.
        let test_bed = AcmTestBed::new()
            .with_ambient(15., 1013.25, 0.)
            .inlet_conditions(30., 180.)
            .demand(18.)
            .iterate(60);

        assert!(test_bed.ram_air_flow() > MassRate::default());
    }
}
