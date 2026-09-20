//! ATA 38: potable water. Tanks and quantity, pneumatic pressurisation
//! (bleed air normally, an electric compressor as backup), distribution to
//! galleys, lavatories and an airline-configurable first-class shower
//! option, water heaters as electrical loads, drain-mast heaters and
//! freezing at altitude, and leaks. Exposes total water mass for a future
//! centre-of-gravity input (see `weight_balance.rs`'s `Mass`/`Balance`,
//! which this module does not import, to stay self-contained per the deep
//! systems brief).
//!
//! No FlyByWire source exists to port (see `deep/cabin/mod.rs`'s doc for the
//! search that confirmed this), so — like `oxygen.rs` — this is a native
//! addition built from public reference material.
//!
//! **Sourcing:**
//! - Large-transport potable water systems are commonly documented (Airbus/
//!   Boeing maintenance-training texts covering ATA 38) as: one or more
//!   tanks pressurised by bleed air (normal) or an electric compressor
//!   (backup/ground), gravity- and pressure-fed distribution lines to
//!   galleys and lavatories, and heated drain masts venting waste water
//!   overboard to stop ice forming and shedding in flight. This module
//!   models that architecture; no A380-specific AMM tank size, pressure or
//!   heater wattage is public, so every such figure below is `GENERIC`.
//! - Water's density (1000 kg/m^3), specific heat (4186 J/(kg*K)) and latent
//!   heat of fusion (334,000 J/kg) are standard physical-property values
//!   (e.g. any engineering data table / NIST webbook).
//! - Dry air's specific gas constant (287.058 J/(kg*K), `R / M` with the
//!   universal gas constant and dry air's mean molar mass, standard values
//!   any thermodynamics reference gives) is restated locally below rather
//!   than imported from `physics::gas` (which carries the same figure under
//!   `AIR_SPECIFIC_GAS_CONSTANT`), per the deep systems brief's rule that
//!   this directory takes no dependency on crate internals.
//! - The tank capacity (800 L) is `GENERIC`, scaled the same way
//!   `oxygen.rs`'s crew bottle is: commonly cited narrowbody potable tank
//!   sizes are in the 200 L class; scaled up roughly for the A380's
//!   multiple-times-larger long-haul cabin.
//! - The pressurisation target (40 psi gauge) is `GENERIC`, the midpoint of
//!   the commonly cited large-transport potable-system pressurisation range
//!   (roughly 30-45 psi in various A320/A330-family technical references).
//! - The first-class shower's session volume (10 L) and duration (5 min)
//!   are `GENERIC`, chosen to match the well-publicised "strict water
//!   budget" of airline shower-spa products (e.g. Emirates' A380 shower
//!   spas are widely reported as time-limited, water-conserving showers),
//!   not a specific manufacturer figure.
//! - Drain-mast heating to prevent in-flight icing (and the resulting "blue
//!   ice" hazard when accumulated ice sheds) is a widely reported real
//!   aviation phenomenon (FAA/CAA service-difficulty and safety literature);
//!   the mast's thermal mass, exposed area and convective coefficients here
//!   are `GENERIC` (no public AMM figures), sized only so the *qualitative*
//!   behaviour is right: a healthy heater holds the mast above freezing at
//!   cruise OAT, a failed one does not.

/// Dry air's specific gas constant, J/(kg*K) (standard: `R_universal /
/// M_air`, restated locally rather than imported from `physics::gas`'s own
/// `AIR_SPECIFIC_GAS_CONSTANT` — see module doc).
const AIR_SPECIFIC_GAS_CONSTANT: f64 = 287.058;

/// Kilograms per litre of fresh water at typical cabin/potable temperatures
/// (standard, e.g. NIST webbook near 20-40 C is ~0.99-0.998; 1.0 used as the
/// standard engineering approximation).
pub const WATER_DENSITY_KG_L: f64 = 1.0;
/// Water's specific heat, J/(kg*K) (standard).
pub const WATER_SPECIFIC_HEAT_J_KGK: f64 = 4186.0;
/// Water's latent heat of fusion, J/kg (standard).
pub const ICE_LATENT_HEAT_J_KG: f64 = 334_000.0;
/// Pascals per psi (exact).
pub const PSI_TO_PA: f64 = 6894.757;

/// GENERIC tank usable capacity, litres (module doc).
pub const TANK_CAPACITY_L: f64 = 800.0;
/// Headspace kept above the water even at a 100% quantity reading, so the
/// tank always has ullage to pressurise against (a sealed-solid tank could
/// not be pneumatically pressurised at all). GENERIC.
const ULLAGE_MIN_FRACTION: f64 = 0.05;
/// Total tank shell volume, litres: capacity plus the fixed ullage headspace.
const TANK_VOLUME_L: f64 = TANK_CAPACITY_L / (1.0 - ULLAGE_MIN_FRACTION);

/// GENERIC pressurisation target, gauge (above cabin pressure), Pa (module
/// doc: 40 psi).
pub const TARGET_GAUGE_PA: f64 = 40.0 * PSI_TO_PA;
/// Relief valve margin above target at which the tank vents excess air
/// overboard to the cabin: a real pressure vessel always has a relief valve
/// above its working pressure. GENERIC.
const RELIEF_MARGIN_PA: f64 = 10.0 * PSI_TO_PA;
/// Below this fraction of the target gauge pressure, distribution flow is
/// negligible: too little pressure to push water through the lines against
/// friction and any static head. GENERIC.
const MIN_USEFUL_PRESSURE_FRACTION: f64 = 0.15;

/// Bleed air pressurises the tank quickly once the valve is open: a modest
/// air mass flow is enough to refill a small ullage headspace against
/// leakage in well under a second of gap. GENERIC.
const BLEED_MAX_AIR_KG_S: f64 = 0.02;
/// The backup electric compressor is markedly slower: a small compressor
/// motor, not a regulated bleed tap. GENERIC.
const COMPRESSOR_MAX_AIR_KG_S: f64 = 0.003;

/// A leaking tank/line vents to cabin pressure through an unintended
/// orifice; at `leak = 1.0` (full fault) this is the flow at full gauge
/// pressure. GENERIC.
const LEAK_MAX_L_S: f64 = 0.05;

/// Number of drain masts modelled (forward and aft), GENERIC per module doc.
pub const N_DRAIN_MASTS: usize = 2;
/// Number of point-of-use water heaters modelled (one per cabin zone),
/// matching `Zone::COUNT`.
pub const N_HEATERS: usize = super::Zone::COUNT;
/// Number of first-class shower stations modelled when the airline option
/// is fitted, GENERIC (module doc: two shower spas is a commonly reported
/// A380 first-class configuration).
pub const N_SHOWERS: usize = 2;

/// GENERIC shower session volume and duration (module doc).
pub const SHOWER_SESSION_L: f64 = 10.0;
pub const SHOWER_DURATION_S: f64 = 300.0;
/// GENERIC instantaneous shower water heater rating, matching the class of
/// load a small point-of-use electric water heater draws.
pub const SHOWER_HEATER_W: f64 = 3000.0;

/// GENERIC point-of-use water heater (lavatory/galley hot tap): element
/// rating, reservoir mass and thermostat band.
const HEATER_RATED_W: f64 = 500.0;
const HEATER_WATER_KG: f64 = 1.5;
const HEATER_SETPOINT_C: f64 = 50.0;
const HEATER_BAND_C: f64 = 5.0;
/// Heater reservoir's standing heat loss to the surrounding cabin air,
/// W/K (GENERIC, a small insulated tank).
const HEATER_LOSS_W_K: f64 = 1.5;

/// Drain mast: exposed area (m^2), convective coefficient at zero TAS and
/// its growth with airspeed (W/(m^2*K) and W/(m^2*K per m/s)), and thermal
/// mass (J/K, pipe metal plus retained water). All GENERIC: no public AMM
/// figures; sized only for the right qualitative behaviour (module doc).
const MAST_AREA_M2: f64 = 0.05;
const MAST_H0_W_M2K: f64 = 10.0;
const MAST_H_PER_TAS_W_M2K: f64 = 0.6;
const MAST_CAPACITY_J_K: f64 = 500.0;
/// GENERIC electric mast heater rating and its automatic thermostatic
/// on-threshold (a real drain mast heater is commanded by an OAT-referenced
/// thermostat/timer, not left on continuously).
const MAST_HEATER_RATED_W: f64 = 150.0;
const MAST_HEATER_ON_OAT_C: f64 = 10.0;
/// A trickle of waste/rinse water is present in the mast whenever
/// lavatories are in use; below freezing, the fraction of it that freezes
/// (rather than draining clear) grows over this span below 0 C. GENERIC.
const MAST_RESIDUAL_KG_S: f64 = 0.001;
const MAST_FULL_FREEZE_SPAN_C: f64 = 10.0;
/// Ice accumulated above this mass is treated as a full blockage. GENERIC.
pub const MAST_BLOCKAGE_KG: f64 = 0.3;
/// How fast accumulated ice melts once the mast is back above freezing,
/// kg/s per degree C above 0. GENERIC.
const MAST_MELT_KG_S_C: f64 = 0.0005;

/// Air pressure from a mass of dry air in a fixed volume at a temperature:
/// the ideal gas law with air's own specific gas constant (not
/// `physics::gas`'s oxygen-specific helpers, which bake in O2's molar
/// mass).
fn air_pressure_pa(air_kg: f64, volume_m3: f64, temp_k: f64) -> f64 {
    if volume_m3 <= 0. || temp_k <= 0. {
        return 0.;
    }
    air_kg.max(0.) * AIR_SPECIFIC_GAS_CONSTANT * temp_k / volume_m3
}

fn air_mass_kg(pressure_pa: f64, volume_m3: f64, temp_k: f64) -> f64 {
    if temp_k <= 0. {
        return 0.;
    }
    (pressure_pa.max(0.) * volume_m3 / (AIR_SPECIFIC_GAS_CONSTANT * temp_k)).max(0.)
}

/// Faults this system carries, each a fraction 0.0 (healthy) .. 1.0 (fully
/// failed).
#[derive(Clone, Copy, Debug, Default)]
pub struct WaterFaults {
    /// A leak in the tank or its lines, vented to cabin pressure.
    pub leak: f64,
    /// The bleed air pressurisation valve/regulator.
    pub bleed_valve_fault: f64,
    /// The backup electric pressurisation compressor.
    pub compressor_fault: f64,
    /// Each zone's point-of-use water heater element.
    pub heater_fault: [f64; N_HEATERS],
    /// Each drain mast's electric anti-ice heater.
    pub mast_heater_fault: [f64; N_DRAIN_MASTS],
    /// The tank quantity sensor: freezes its last good reading instead of
    /// tracking the real quantity (a stuck-probe failure).
    pub quantity_sensor_fault: f64,
}

/// What the rest of the aircraft feeds in this tick.
#[derive(Clone, Copy, Debug)]
pub struct WaterInputs {
    pub bleed_available: bool,
    pub compressor_commanded: bool,
    /// Cabin absolute pressure, Pa (the pressurisation target above is
    /// gauge, relative to this).
    pub cabin_pressure_pa: f64,
    pub cabin_temp_k: f64,
    pub oat_c: f64,
    pub tas_mps: f64,
    pub galley_demand_l_s: f64,
    pub lav_demand_l_s: f64,
    /// New shower sessions requested to start this tick (ignored if the
    /// system was not built with showers fitted, or no station is free).
    pub shower_requests: usize,
    pub heater_commanded: [bool; N_HEATERS],
}

impl Default for WaterInputs {
    fn default() -> Self {
        Self {
            bleed_available: true,
            compressor_commanded: false,
            cabin_pressure_pa: 101_325.0,
            cabin_temp_k: 297.0,
            oat_c: 15.0,
            tas_mps: 0.0,
            galley_demand_l_s: 0.0,
            lav_demand_l_s: 0.0,
            shower_requests: 0,
            heater_commanded: [false; N_HEATERS],
        }
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub struct WaterOutputs {
    pub quantity_percent: f64,
    pub gauge_pressure_pa: f64,
    /// Fraction (0..1) of requested distribution flow actually delivered,
    /// from the orifice-like pressure/flow relationship.
    pub flow_fraction: f64,
    pub leak_l_s: f64,
    pub mast_ice_kg: [f64; N_DRAIN_MASTS],
    pub mast_blocked: [bool; N_DRAIN_MASTS],
    pub heater_temp_c: [f64; N_HEATERS],
    pub heater_power_w: [f64; N_HEATERS],
    pub shower_active: [bool; N_SHOWERS],
    pub shower_heater_power_w: f64,
    /// Total water mass presently carried, kg — the centre-of-gravity input
    /// item 1 of the backlog asks for.
    pub water_mass_kg: f64,
    pub tank_empty: bool,
}

#[derive(Clone, Copy, Debug)]
struct Heater {
    temp_c: f64,
}

#[derive(Clone, Copy, Debug)]
struct Mast {
    temp_c: f64,
    ice_kg: f64,
}

#[derive(Clone, Copy, Debug)]
struct Shower {
    remaining_s: f64,
}

pub struct WaterSystem {
    water_l: f64,
    ullage_air_kg: f64,
    displayed_quantity_percent: f64,
    heaters: [Heater; N_HEATERS],
    masts: [Mast; N_DRAIN_MASTS],
    showers: [Shower; N_SHOWERS],
    showers_fitted: bool,
}

impl WaterSystem {
    /// `showers_fitted`: the airline configuration option (backlog item 1).
    pub fn new(showers_fitted: bool) -> Self {
        let cabin_temp_k = 297.0;
        let ullage_volume_m3 = (TANK_VOLUME_L - TANK_CAPACITY_L) / 1000.0;
        let full_gauge_target_pa = TARGET_GAUGE_PA;
        let ullage_air_kg = air_mass_kg(101_325.0 + full_gauge_target_pa, ullage_volume_m3, cabin_temp_k);
        Self {
            water_l: TANK_CAPACITY_L,
            ullage_air_kg,
            displayed_quantity_percent: 100.0,
            heaters: [Heater { temp_c: 20.0 }; N_HEATERS],
            masts: [Mast { temp_c: 20.0, ice_kg: 0.0 }; N_DRAIN_MASTS],
            showers: [Shower { remaining_s: 0.0 }; N_SHOWERS],
            showers_fitted,
        }
    }

    /// Ground/turnaround service: refills the tank and clears mast ice.
    pub fn service(&mut self) {
        self.water_l = TANK_CAPACITY_L;
        for mast in &mut self.masts {
            mast.ice_kg = 0.0;
        }
    }

    pub fn step(&mut self, inputs: &WaterInputs, faults: &WaterFaults, dt: f64) -> WaterOutputs {
        let dt = dt.max(0.0);

        // --- Pressurisation: ullage air mass tracks the target via the
        // ideal gas law (air_pressure_pa/air_mass_kg), fed by whichever
        // source is healthy and available, capped by a relief valve.
        let ullage_volume_l = (TANK_VOLUME_L - self.water_l).max(TANK_VOLUME_L * ULLAGE_MIN_FRACTION);
        let ullage_volume_m3 = ullage_volume_l / 1000.0;
        let target_abs_pa = inputs.cabin_pressure_pa + TARGET_GAUGE_PA;
        let relief_abs_pa = inputs.cabin_pressure_pa + TARGET_GAUGE_PA + RELIEF_MARGIN_PA;

        let source_kg_s = if inputs.bleed_available && faults.bleed_valve_fault < 1.0 {
            BLEED_MAX_AIR_KG_S * (1.0 - faults.bleed_valve_fault)
        } else if inputs.compressor_commanded && faults.compressor_fault < 1.0 {
            COMPRESSOR_MAX_AIR_KG_S * (1.0 - faults.compressor_fault)
        } else {
            0.0
        };
        let current_abs_pa = air_pressure_pa(self.ullage_air_kg, ullage_volume_m3, inputs.cabin_temp_k);
        if current_abs_pa < target_abs_pa {
            let needed_kg = air_mass_kg(target_abs_pa, ullage_volume_m3, inputs.cabin_temp_k) - self.ullage_air_kg;
            self.ullage_air_kg += needed_kg.max(0.0).min(source_kg_s * dt);
        } else if current_abs_pa > relief_abs_pa {
            let excess_kg = self.ullage_air_kg - air_mass_kg(relief_abs_pa, ullage_volume_m3, inputs.cabin_temp_k);
            // A relief valve passes flow fast relative to the slow
            // pressurisation sources above; vent most of the excess this
            // tick without going negative.
            self.ullage_air_kg -= excess_kg.max(0.0).min(self.ullage_air_kg);
        }
        let gauge_pa = (air_pressure_pa(self.ullage_air_kg, ullage_volume_m3, inputs.cabin_temp_k) - inputs.cabin_pressure_pa).max(0.0);

        // --- Distribution flow: an orifice-like relation to available
        // gauge pressure (real taps/spray nozzles are small orifices; flow
        // through an orifice scales with sqrt(delta pressure)).
        let pressure_ratio = (gauge_pa / TARGET_GAUGE_PA).clamp(0.0, 1.0);
        let flow_fraction = if pressure_ratio < MIN_USEFUL_PRESSURE_FRACTION { 0.0 } else { pressure_ratio.sqrt() };

        // --- Showers: fixed-volume timed sessions.
        let mut shower_flow_l_s = 0.0;
        let mut shower_heater_power_w = 0.0;
        let mut requests_left = if self.showers_fitted { inputs.shower_requests } else { 0 };
        for shower in &mut self.showers {
            if shower.remaining_s <= 0.0 && requests_left > 0 {
                shower.remaining_s = SHOWER_DURATION_S;
                requests_left -= 1;
            }
            if shower.remaining_s > 0.0 {
                shower_flow_l_s += SHOWER_SESSION_L / SHOWER_DURATION_S;
                shower_heater_power_w += SHOWER_HEATER_W;
                shower.remaining_s = (shower.remaining_s - dt).max(0.0);
            }
        }

        let requested_flow_l_s = inputs.galley_demand_l_s + inputs.lav_demand_l_s + shower_flow_l_s;
        let actual_flow_l_s = requested_flow_l_s * flow_fraction;
        let leak_l_s = faults.leak.clamp(0.0, 1.0) * LEAK_MAX_L_S * flow_fraction.max(pressure_ratio);
        self.water_l = (self.water_l - (actual_flow_l_s + leak_l_s) * dt).max(0.0);
        let tank_empty = self.water_l <= 0.0;

        // --- Heaters: first-order thermal lag to a source term (element
        // power minus standing loss) over the reservoir's thermal capacity.
        let mut heater_temp_c = [0.0; N_HEATERS];
        let mut heater_power_w = [0.0; N_HEATERS];
        for i in 0..N_HEATERS {
            let heater = &mut self.heaters[i];
            let on = inputs.heater_commanded[i] && heater.temp_c < HEATER_SETPOINT_C + HEATER_BAND_C && faults.heater_fault[i] < 1.0;
            let power_w = if on { HEATER_RATED_W * (1.0 - faults.heater_fault[i]) } else { 0.0 };
            let capacity_j_k = HEATER_WATER_KG * WATER_SPECIFIC_HEAT_J_KGK;
            let ambient_c = inputs.cabin_temp_k - 273.15;
            // dT/dt = (P - loss*(T-ambient)) / C -> exponential to a target
            // temperature with time constant C/loss.
            let target_c = ambient_c + power_w / HEATER_LOSS_W_K;
            let tau_s = (capacity_j_k / HEATER_LOSS_W_K).max(1e-6);
            heater.temp_c += (target_c - heater.temp_c) * (1.0 - (-dt / tau_s).exp());
            heater_temp_c[i] = heater.temp_c;
            heater_power_w[i] = power_w;
        }

        // --- Drain masts: heat balance against OAT with a thermostatic
        // heater, and freezing of the residual trickle below 0 C.
        let mut mast_ice_kg = [0.0; N_DRAIN_MASTS];
        let mut mast_blocked = [false; N_DRAIN_MASTS];
        let h = MAST_H0_W_M2K + MAST_H_PER_TAS_W_M2K * inputs.tas_mps.max(0.0);
        let ha = h * MAST_AREA_M2;
        for i in 0..N_DRAIN_MASTS {
            let mast = &mut self.masts[i];
            let heater_on = inputs.oat_c < MAST_HEATER_ON_OAT_C && faults.mast_heater_fault[i] < 1.0;
            let power_w = if heater_on { MAST_HEATER_RATED_W * (1.0 - faults.mast_heater_fault[i]) } else { 0.0 };
            let target_c = inputs.oat_c + power_w / ha.max(1e-9);
            let tau_s = (MAST_CAPACITY_J_K / ha.max(1e-9)).max(1e-6);
            mast.temp_c += (target_c - mast.temp_c) * (1.0 - (-dt / tau_s).exp());

            if mast.temp_c < 0.0 {
                let sub_cool = (-mast.temp_c).min(MAST_FULL_FREEZE_SPAN_C);
                let freeze_fraction = sub_cool / MAST_FULL_FREEZE_SPAN_C;
                mast.ice_kg += MAST_RESIDUAL_KG_S * freeze_fraction * dt;
            } else {
                let melt = (MAST_MELT_KG_S_C * mast.temp_c * dt).min(mast.ice_kg);
                mast.ice_kg -= melt;
            }
            mast_ice_kg[i] = mast.ice_kg;
            mast_blocked[i] = mast.ice_kg >= MAST_BLOCKAGE_KG;
        }

        // --- Quantity sensor: a stuck-probe fault freezes the last good
        // reading instead of tracking the real level.
        let real_percent = 100.0 * self.water_l / TANK_CAPACITY_L.max(1e-9);
        if faults.quantity_sensor_fault < 0.5 {
            self.displayed_quantity_percent = real_percent;
        }

        WaterOutputs {
            quantity_percent: self.displayed_quantity_percent,
            gauge_pressure_pa: gauge_pa,
            flow_fraction,
            leak_l_s,
            mast_ice_kg,
            mast_blocked,
            heater_temp_c,
            heater_power_w,
            shower_active: std::array::from_fn(|i| self.showers[i].remaining_s > 0.0),
            shower_heater_power_w,
            water_mass_kg: self.water_l * WATER_DENSITY_KG_L,
            tank_empty,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn healthy_inputs() -> WaterInputs {
        WaterInputs::default()
    }

    #[test]
    fn tank_drains_by_exactly_the_delivered_flow_conserving_mass() {
        let mut w = WaterSystem::new(false);
        let mut inputs = healthy_inputs();
        inputs.galley_demand_l_s = 0.1;
        // Let pressure build first.
        for _ in 0..600 {
            w.step(&inputs, &WaterFaults::default(), 1.0 / 60.0);
        }
        let before = w.water_l;
        let out = w.step(&inputs, &WaterFaults::default(), 1.0);
        let delivered = inputs.galley_demand_l_s * out.flow_fraction;
        assert!((before - w.water_l - delivered).abs() < 1e-6, "before={before} after={} delivered={delivered}", w.water_l);
        assert!(!out.tank_empty);
    }

    #[test]
    fn a_leak_drains_the_tank_with_no_demand_and_a_healthy_leak_does_not() {
        let mut healthy = WaterSystem::new(false);
        let mut leaking = WaterSystem::new(false);
        let inputs = healthy_inputs();
        for _ in 0..600 {
            healthy.step(&inputs, &WaterFaults::default(), 1.0 / 60.0);
            leaking.step(&inputs, &WaterFaults { leak: 1.0, ..Default::default() }, 1.0 / 60.0);
        }
        assert_eq!(healthy.water_l, TANK_CAPACITY_L, "no demand, no leak: full");
        assert!(leaking.water_l < TANK_CAPACITY_L, "a full leak fault should drain the tank");
    }

    #[test]
    fn ullage_pressure_follows_the_ideal_gas_law_and_settles_near_target() {
        let mut w = WaterSystem::new(false);
        let inputs = healthy_inputs();
        let mut out = WaterOutputs::default();
        for _ in 0..1200 {
            out = w.step(&inputs, &WaterFaults::default(), 1.0 / 60.0);
        }
        assert!((out.gauge_pressure_pa - TARGET_GAUGE_PA).abs() / TARGET_GAUGE_PA < 0.05, "{}", out.gauge_pressure_pa);
    }

    #[test]
    fn a_dead_bleed_valve_falls_back_to_the_slower_compressor() {
        let mut w = WaterSystem::new(false);
        let mut inputs = healthy_inputs();
        inputs.bleed_available = false;
        inputs.compressor_commanded = true;
        let mut out = WaterOutputs::default();
        for _ in 0..3600 {
            out = w.step(&inputs, &WaterFaults::default(), 1.0 / 60.0);
        }
        assert!(out.gauge_pressure_pa > TARGET_GAUGE_PA * 0.5, "compressor alone should still pressurise, slowly: {}", out.gauge_pressure_pa);
    }

    #[test]
    fn with_neither_source_pressure_cannot_be_maintained_and_flow_fails() {
        let mut w = WaterSystem::new(false);
        let mut inputs = healthy_inputs();
        inputs.bleed_available = false;
        inputs.compressor_commanded = false;
        inputs.galley_demand_l_s = 0.05;

        // The tank is a pressure vessel, so losing its air source does not
        // stop the taps at once: the trapped ullage air expands as water
        // leaves and the pressure bleeds down along p*V = const.
        // Hand calculation (isothermal, no air mass entering or leaving):
        //   V_tank  = 800/(1 - 0.05)                     = 842.1 L
        //   V_u0    = 842.1 - 800                        =  42.1 L
        //   p_abs0  = 101 325 + 40 psi                   = 377 115 Pa
        //   p*V     = 377 115 * 0.042105 m^3             = 15 878 Pa*m^3
        // Flow stops when the gauge falls below 15% of target, i.e. at
        //   p_abs = 101 325 + 0.15*275 790               = 142 694 Pa
        //   V_u   = 15 878 / 142 694                     = 111.3 L
        // so the tank has to give up 111.3 - 42.1 = 69.2 L of water first,
        // leaving 730.8 L aboard. At 0.05 L/s that is well over half an
        // hour, not the one minute this test used to allow.
        let mut out = WaterOutputs::default();
        for _ in 0..3600 {
            out = w.step(&inputs, &WaterFaults::default(), 1.0 / 60.0);
        }
        assert!(out.flow_fraction > 0.9, "one minute in, the stored air is barely touched: {}", out.flow_fraction);

        let mut failed_at_s = None;
        let mut water_at_failure_l = 0.0;
        for step in 0..14_400 {
            out = w.step(&inputs, &WaterFaults::default(), 0.25);
            if out.flow_fraction == 0.0 {
                failed_at_s = Some((step + 1) as f64 * 0.25);
                water_at_failure_l = w.water_l;
                break;
            }
        }
        assert!(failed_at_s.is_some(), "with no air source the taps must eventually die");
        assert!((water_at_failure_l - 730.8).abs() < 1.0, "{water_at_failure_l} L left when flow failed");
        assert!(out.gauge_pressure_pa < TARGET_GAUGE_PA * MIN_USEFUL_PRESSURE_FRACTION);
    }

    #[test]
    fn a_cold_drain_mast_freezes_without_its_heater_and_stays_clear_with_it() {
        let mut cold_no_heat = WaterSystem::new(false);
        let mut cold_heated = WaterSystem::new(false);
        let mut inputs = healthy_inputs();
        inputs.oat_c = -56.0; // typical cruise OAT
        for _ in 0..36000 {
            cold_no_heat.step(&inputs, &WaterFaults { mast_heater_fault: [1.0; N_DRAIN_MASTS], ..Default::default() }, 1.0 / 60.0);
            cold_heated.step(&inputs, &WaterFaults::default(), 1.0 / 60.0);
        }
        assert!(cold_no_heat.masts[0].ice_kg > 0.0, "a failed mast heater at cruise OAT should let ice accumulate");
        assert!(cold_heated.masts[0].temp_c > 0.0, "a healthy mast heater should hold the mast above freezing");
        assert_eq!(cold_heated.masts[0].ice_kg, 0.0);
    }

    #[test]
    fn ice_above_the_blockage_threshold_is_reported_blocked() {
        let mut w = WaterSystem::new(false);
        w.masts[0].ice_kg = MAST_BLOCKAGE_KG + 0.01;
        let out = w.step(&healthy_inputs(), &WaterFaults::default(), 0.001);
        assert!(out.mast_blocked[0]);
        assert!(!out.mast_blocked[1]);
    }

    #[test]
    fn showers_are_off_when_not_fitted_even_if_requested() {
        let mut w = WaterSystem::new(false);
        let mut inputs = healthy_inputs();
        inputs.shower_requests = 2;
        let out = w.step(&inputs, &WaterFaults::default(), 1.0);
        assert!(out.shower_active.iter().all(|&a| !a));
        assert_eq!(out.shower_heater_power_w, 0.0);
    }

    #[test]
    fn a_requested_shower_runs_for_its_full_session_and_uses_its_published_volume() {
        let mut w = WaterSystem::new(true);
        let mut inputs = healthy_inputs();
        inputs.shower_requests = 1;
        let mut total_l = 0.0;
        let dt = 1.0;
        for _ in 0..(SHOWER_DURATION_S as usize + 5) {
            let before = w.water_l;
            let out = w.step(&inputs, &WaterFaults::default(), dt);
            inputs.shower_requests = 0;
            if out.shower_active[0] {
                total_l += (before - w.water_l).max(0.0);
            }
        }
        // Pressure is essentially at target throughout, so delivered volume
        // should be close to the published session volume.
        assert!((total_l - SHOWER_SESSION_L).abs() < 1.0, "{total_l}");
    }

    #[test]
    fn a_stuck_quantity_sensor_freezes_its_last_reading() {
        let mut w = WaterSystem::new(false);
        let mut inputs = healthy_inputs();
        inputs.galley_demand_l_s = 0.5;
        let mut faults = WaterFaults::default();
        for _ in 0..600 {
            w.step(&inputs, &faults, 1.0 / 60.0);
        }
        faults.quantity_sensor_fault = 1.0;
        let stuck_at = w.displayed_quantity_percent;
        for _ in 0..600 {
            w.step(&inputs, &faults, 1.0 / 60.0);
        }
        assert_eq!(w.displayed_quantity_percent, stuck_at);
        assert!(w.water_l / TANK_CAPACITY_L * 100.0 < stuck_at, "the real quantity should have kept dropping");
    }

    #[test]
    fn service_refills_the_tank_and_clears_mast_ice() {
        let mut w = WaterSystem::new(false);
        w.water_l = 10.0;
        w.masts[0].ice_kg = 1.0;
        w.service();
        assert_eq!(w.water_l, TANK_CAPACITY_L);
        assert_eq!(w.masts[0].ice_kg, 0.0);
    }

    #[test]
    fn no_nan_at_rest_or_dt_zero() {
        let mut w = WaterSystem::new(true);
        let inputs = healthy_inputs();
        let out = w.step(&inputs, &WaterFaults::default(), 0.0);
        assert!(!out.gauge_pressure_pa.is_nan());
        assert!(!out.water_mass_kg.is_nan());
        for t in out.heater_temp_c {
            assert!(!t.is_nan());
        }
        for t in out.mast_ice_kg {
            assert!(!t.is_nan());
        }
    }
}
