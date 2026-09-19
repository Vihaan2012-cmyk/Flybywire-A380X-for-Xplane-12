//! Fluid thermal model: a lumped fluid temperature per hydraulic circuit,
//! heated by pump mechanical/volumetric loss and by every throttling
//! pressure drop (a relief valve dumping to return, a priority valve
//! holding back flow, a leak -- lost hydraulic power that is not extracted
//! as useful work across a restriction is dissipated as heat right there,
//! `Q_w = dP_pa * flow_m3_s`), cooled by a fuel-cooled heat exchanger and a
//! passive loss to the surrounding bay, with the result feeding back into
//! the network's own line resistances through `fluid::dynamic_viscosity_pa_s`
//! (cold fluid runs high pressure drops, exactly like this crate's existing
//! engine oil model, `physics::engine::oil.rs`'s own module doc).
//!
//! Cooling: this crate's own `physics::fluids.rs` already documents the
//! A380's hydraulic/fuel heat exchangers ("two fuel/hydraulic heat
//! exchangers per circuit, one per pylon", cited there to Power & Motion
//! Technology, "Hydraulics onboard the A380") and settles on
//! `HHX_EFFECTIVENESS = 0.6`, a mid-range plate/shell-and-tube figure (no
//! public A380 HHX effectiveness exists); the same figure and reasoning is
//! reused here, restated to keep this directory self-contained. Skydrol
//! LD-4's own technical bulletin gives a 225 F (107 C) maximum *continuous*
//! operating temperature and a -40 C..205 C overall usable range (Eastman
//! Pub. No. 7249153C); `OVERHEAT_K` below is the continuous limit.

pub const OVERHEAT_K: f64 = 107.0 + 273.15;

/// GENERIC: no public A380 hydraulic bay thermal figures exist. Sized so a
/// circuit's fluid takes minutes, not seconds, to change temperature (a real
/// hydraulic system's thermal response is slow), and so the fuel-cooled heat
/// exchanger can hold cruise temperatures in a plausible 50-90 C band at
/// representative flow -- the same "settles in a sane band" sanity check
/// `physics::engine::oil.rs`'s own tests use for its tank temperature.
#[derive(Clone, Copy, Debug)]
pub struct ThermalSizing {
    /// Total trapped fluid mass this circuit holds (reservoir + lines +
    /// manifolds), kg -- the thermal mass that stores/releases heat.
    pub fluid_mass_kg: f64,
    pub ambient_loss_w_per_k: f64,
}
impl ThermalSizing {
    /// ~12-12.7 US gal reservoir plus the circuit's own trapped line/
    /// manifold volume, order of magnitude 60 L total, at this fluid's
    /// ~1000 kg/m^3 (`fluid::density_kg_m3`).
    pub fn a380_circuit() -> Self {
        Self { fluid_mass_kg: 60.0, ambient_loss_w_per_k: 40.0 }
    }
}

/// GENERIC: no published specific heat for this fluid family; phosphate
/// esters run somewhat below mineral oil's ~2000 J/(kg K)
/// (`physics::engine::oil.rs`'s `OIL_CP`), a representative order of
/// magnitude for a similar-density synthetic fluid.
const FLUID_CP_J_KG_K: f64 = 1900.0;
const HHX_EFFECTIVENESS: f64 = 0.6;
/// Matches this crate's own Jet A specific heat, `physics::engine::oil.rs`'s
/// `FUEL_CP`.
const FUEL_CP_J_KG_K: f64 = 2010.0;

#[derive(Clone, Copy, Debug, Default)]
pub struct ThermalOutputs {
    pub temp_k: f64,
    pub temp_c: f64,
    pub overheat: bool,
    pub fuel_heat_w: f64,
    pub fuel_out_k: f64,
}

#[derive(Clone, Copy, Debug)]
pub struct ThermalState {
    temp_k: f64,
}
impl ThermalState {
    pub fn new(temp_k: f64) -> Self {
        Self { temp_k }
    }
    pub fn temp_k(&self) -> f64 {
        self.temp_k
    }
    pub fn temp_c(&self) -> f64 {
        self.temp_k - 273.15
    }

    /// `pump_heat_w`/`throttling_heat_w`: this tick's dissipated power (both
    /// always >= 0 physically). `fluid_flow_kg_s`: the circuit's own flow
    /// passing through its heat exchanger (pump delivery, roughly); only the
    /// flowing fraction of the fluid mass is actively cooled each tick, the
    /// same convention `physics::engine::oil.rs`'s FCOC uses. `fuel_kg_s`/
    /// `fuel_temp_k`: the engine feed fuel available to absorb that heat.
    /// `ambient_k`: the hydraulic bay/nacelle temperature the passive loss
    /// term sinks to. An exact exponential step (unconditionally stable
    /// regardless of `dt_s`), matching this crate's established convention
    /// for first-order thermal lags.
    pub fn step(
        &mut self,
        sizing: &ThermalSizing,
        pump_heat_w: f64,
        throttling_heat_w: f64,
        fluid_flow_kg_s: f64,
        fuel_kg_s: f64,
        fuel_temp_k: f64,
        ambient_k: f64,
        dt_s: f64,
    ) -> ThermalOutputs {
        let dt = dt_s.max(0.0);
        let heat_in_w = pump_heat_w.max(0.0) + throttling_heat_w.max(0.0);

        let fluid_capacity_w_per_k = fluid_flow_kg_s.max(0.0) * FLUID_CP_J_KG_K;
        let fuel_capacity_w_per_k = fuel_kg_s.max(0.0) * FUEL_CP_J_KG_K;
        let hx_conductance_w_per_k = HHX_EFFECTIVENESS * fluid_capacity_w_per_k.min(fuel_capacity_w_per_k);
        let fuel_heat_w = hx_conductance_w_per_k * (self.temp_k - fuel_temp_k);
        let fuel_out_k = if fuel_capacity_w_per_k > 0.0 { fuel_temp_k + fuel_heat_w / fuel_capacity_w_per_k } else { fuel_temp_k };

        let ambient_loss_w_per_k = sizing.ambient_loss_w_per_k.max(0.0);
        let conductance = (hx_conductance_w_per_k + ambient_loss_w_per_k).max(1e-9);
        let target_k = (heat_in_w + hx_conductance_w_per_k * fuel_temp_k + ambient_loss_w_per_k * ambient_k) / conductance;
        let thermal_mass_j_per_k = sizing.fluid_mass_kg.max(1e-6) * FLUID_CP_J_KG_K;
        let rate = conductance / thermal_mass_j_per_k;
        self.temp_k = target_k + (self.temp_k - target_k) * (-rate * dt).exp();

        ThermalOutputs { temp_k: self.temp_k, temp_c: self.temp_c(), overheat: self.temp_k > OVERHEAT_K, fuel_heat_w, fuel_out_k }
    }
}

/// Heat dissipated by a throttling restriction carrying `flow_m3_s` (always
/// >= 0, direction irrelevant to the heat produced) across `dp_pa` -- the
/// lost hydraulic power a relief valve, priority valve, or leak converts
/// entirely to heat rather than useful work. Callers (`topology.rs`) sum
/// this across every throttling element in a circuit each tick.
pub fn throttling_heat_w(flow_m3_s: f64, dp_pa: f64) -> f64 {
    flow_m3_s.abs() * dp_pa.abs()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn heating_with_no_cooling_path_raises_temperature() {
        let mut t = ThermalState::new(288.15);
        let sizing = ThermalSizing::a380_circuit();
        let mut out = ThermalOutputs::default();
        for _ in 0..600 {
            out = t.step(&sizing, 2000.0, 0.0, 0.0, 0.0, 288.15, 288.15, 1.0);
        }
        assert!(out.temp_k > 288.15);
    }

    #[test]
    fn the_fuel_heat_exchanger_removes_heat_and_warms_the_fuel() {
        let mut t = ThermalState::new(360.0);
        let sizing = ThermalSizing::a380_circuit();
        let out = t.step(&sizing, 0.0, 0.0, 1.0, 1.0, 288.15, 288.15, 0.01);
        assert!(out.fuel_heat_w > 0.0);
        assert!(out.fuel_out_k > 288.15);
    }

    #[test]
    fn settles_to_a_stable_temperature_under_constant_heat_load() {
        let mut t = ThermalState::new(288.15);
        let sizing = ThermalSizing::a380_circuit();
        let mut last = 0.0;
        for _ in 0..20000 {
            let out = t.step(&sizing, 3000.0, 500.0, 0.5, 1.5, 288.15, 288.15, 1.0);
            last = out.temp_k;
        }
        assert!(last.is_finite());
        assert!(last < 500.0, "should settle, not run away: {last} K");
        assert!(last > 288.15);
    }

    #[test]
    fn overheat_flag_trips_above_the_skydrol_continuous_limit() {
        let mut t = ThermalState::new(OVERHEAT_K + 5.0);
        let out = t.step(&ThermalSizing::a380_circuit(), 0.0, 0.0, 0.0, 0.0, 288.15, 288.15, 0.0);
        assert!(out.overheat);
        let mut cool = ThermalState::new(320.0);
        let out2 = cool.step(&ThermalSizing::a380_circuit(), 0.0, 0.0, 0.0, 0.0, 288.15, 288.15, 0.0);
        assert!(!out2.overheat);
    }

    #[test]
    fn throttling_heat_scales_with_flow_and_pressure_drop() {
        assert_eq!(throttling_heat_w(0.0, 1000.0), 0.0);
        let h1 = throttling_heat_w(1e-4, 1.0e6);
        let h2 = throttling_heat_w(2e-4, 1.0e6);
        assert!((h2 / h1 - 2.0).abs() < 1e-9);
    }

    #[test]
    fn no_nan_at_dt_zero_or_rest() {
        let mut t = ThermalState::new(288.15);
        let out = t.step(&ThermalSizing::a380_circuit(), 0.0, 0.0, 0.0, 0.0, 288.15, 288.15, 0.0);
        assert!(out.temp_k.is_finite());
        assert!((out.temp_k - 288.15).abs() < 1e-9, "dt=0 must not move the state: {}", out.temp_k);
    }
}
