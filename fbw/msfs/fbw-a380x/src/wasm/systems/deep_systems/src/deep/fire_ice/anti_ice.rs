use super::util::{clamp01, equilibrium_surface_c_with_bleed, equilibrium_surface_c_with_heater, relax_toward_equilibrium_c, surface_net_loss_w_m2, CP_AIR, LATENT_HEAT_FUSION_WATER_J_KG};

pub const BLEED_SUPPLY_TEMP_C: f64 = 200.0;

#[derive(Clone, Copy, Debug, Default)]
pub struct BleedAntiIceFaults {
    pub valve_stuck_closed: f64,
    pub valve_stuck_open: f64,
    pub duct_leak: f64,
}

#[derive(Clone, Copy, Debug)]
pub struct BleedSurfaceParams {
    pub area_m2: f64,
    pub h_w_m2k: f64,
    pub effectiveness: f64,
    pub max_bleed_kg_s: f64,
    pub thermal_tau_s: f64,
}

pub const WING_ANTI_ICE: BleedSurfaceParams = BleedSurfaceParams { area_m2: 6.0, h_w_m2k: 120.0, effectiveness: 0.75, max_bleed_kg_s: 0.5, thermal_tau_s: 25.0 };
pub const NACELLE_ANTI_ICE: BleedSurfaceParams = BleedSurfaceParams { area_m2: 3.0, h_w_m2k: 150.0, effectiveness: 0.8, max_bleed_kg_s: 0.3, thermal_tau_s: 15.0 };

pub const OVERHEAT_TRIP_C: f64 = 60.0;

pub struct BleedAntiIceSurface {
    params: BleedSurfaceParams,
    surface_c: f64,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct BleedAntiIceOutputs {
    pub surface_c: f64,
    pub bleed_delivered_kg_s: f64,
    pub freezing_fraction: f64,
    pub overheat: bool,
}

impl BleedAntiIceSurface {
    pub fn new(params: BleedSurfaceParams, initial_c: f64) -> Self {
        Self { params, surface_c: initial_c }
    }

    pub fn surface_c(&self) -> f64 {
        self.surface_c
    }

    #[allow(clippy::too_many_arguments)]
    pub fn step(
        &mut self,
        valve_command: f64,
        static_air_c: f64,
        recovery_c: f64,
        lwc_kg_m3: f64,
        beta0: f64,
        tas_m_s: f64,
        ambient_pressure_pa: f64,
        faults: &BleedAntiIceFaults,
        dt_s: f64,
    ) -> BleedAntiIceOutputs {
        let commanded = clamp01(valve_command) * (1.0 - clamp01(faults.valve_stuck_closed));
        let position = commanded.max(clamp01(faults.valve_stuck_open));
        let mdot_bleed = position * self.params.max_bleed_kg_s * (1.0 - clamp01(faults.duct_leak));

        let bleed_slope_w_m2k = self.params.effectiveness * mdot_bleed * CP_AIR / self.params.area_m2.max(1e-6);
        let equilibrium_c = equilibrium_surface_c_with_bleed(self.params.h_w_m2k, static_air_c, recovery_c, lwc_kg_m3, beta0, tas_m_s, ambient_pressure_pa, bleed_slope_w_m2k, BLEED_SUPPLY_TEMP_C);
        self.surface_c = relax_toward_equilibrium_c(self.surface_c, equilibrium_c, self.params.thermal_tau_s, dt_s);

        let freezing_fraction = if self.surface_c > 0.0 {
            0.0
        } else {
            let (net_loss, impingement) = surface_net_loss_w_m2(self.params.h_w_m2k, static_air_c, recovery_c, lwc_kg_m3, beta0, tas_m_s, ambient_pressure_pa, self.surface_c);
            if impingement > 1e-9 {
                clamp01(net_loss / (impingement * LATENT_HEAT_FUSION_WATER_J_KG))
            } else {
                0.0
            }
        };

        BleedAntiIceOutputs { surface_c: self.surface_c, bleed_delivered_kg_s: mdot_bleed, freezing_fraction, overheat: self.surface_c > OVERHEAT_TRIP_C }
    }
}

pub const PROBE_HEATER_RATED_W: f64 = 80.0;
const PROBE_AREA_M2: f64 = 0.0006;
const PROBE_H_W_M2K: f64 = 200.0;
const PROBE_TARGET_C: f64 = 5.0;
const PROBE_THERMAL_TAU_S: f64 = 5.0;
const PROBE_SENSOR_STUCK_READING_C: f64 = 15.0;

#[derive(Clone, Copy, Debug, Default)]
pub struct ProbeHeaterFaults {
    pub heater_open_circuit: f64,
    pub controller_fault: f64,
    pub sensor_fault: f64,
}

pub struct ProbeHeater {
    surface_c: f64,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct ProbeHeaterOutputs {
    pub surface_c: f64,
    pub power_w: f64,
    pub freezing_fraction: f64,
}

impl ProbeHeater {
    pub fn new(initial_c: f64) -> Self {
        Self { surface_c: initial_c }
    }

    pub fn surface_c(&self) -> f64 {
        self.surface_c
    }

    #[allow(clippy::too_many_arguments)]
    pub fn step(&mut self, static_air_c: f64, recovery_c: f64, lwc_kg_m3: f64, beta0: f64, tas_m_s: f64, ambient_pressure_pa: f64, faults: &ProbeHeaterFaults, dt_s: f64) -> ProbeHeaterOutputs {
        let sensed_c = self.surface_c + (PROBE_SENSOR_STUCK_READING_C - self.surface_c) * clamp01(faults.sensor_fault);
        let commanded_on = sensed_c < PROBE_TARGET_C;
        let controller_ok = clamp01(faults.controller_fault) < 1.0;
        let power_w = if commanded_on && controller_ok {
            PROBE_HEATER_RATED_W * (1.0 - clamp01(faults.heater_open_circuit))
        } else {
            0.0
        };
        let heater_w_m2 = power_w / PROBE_AREA_M2;

        let equilibrium_c = equilibrium_surface_c_with_heater(PROBE_H_W_M2K, static_air_c, recovery_c, lwc_kg_m3, beta0, tas_m_s, ambient_pressure_pa, heater_w_m2);
        self.surface_c = relax_toward_equilibrium_c(self.surface_c, equilibrium_c, PROBE_THERMAL_TAU_S, dt_s);

        let freezing_fraction = if self.surface_c > 0.0 {
            0.0
        } else {
            let (net_loss, impingement) = surface_net_loss_w_m2(PROBE_H_W_M2K, static_air_c, recovery_c, lwc_kg_m3, beta0, tas_m_s, ambient_pressure_pa, self.surface_c);
            if impingement > 1e-9 {
                clamp01(net_loss / (impingement * LATENT_HEAT_FUSION_WATER_J_KG))
            } else {
                0.0
            }
        };

        ProbeHeaterOutputs { surface_c: self.surface_c, power_w, freezing_fraction }
    }
}

const WINDOW_AREA_M2: f64 = 1.5;
const WINDOW_H_W_M2K: f64 = 80.0;
pub const WINDOW_TARGET_C: f64 = 40.0;
pub const WINDOW_RATED_W: f64 = 10_000.0;
pub const WINDOW_OVERHEAT_PROTECT_C: f64 = 65.0;
const WINDOW_DELAMINATION_MARGIN_C: f64 = 15.0;
const WINDOW_CRACK_MARGIN_C: f64 = 40.0;
const WINDOW_THERMAL_TAU_S: f64 = 40.0;
const WINDOW_SENSOR_STUCK_READING_C: f64 = 50.0;

#[derive(Clone, Copy, Debug, Default)]
pub struct WindowHeatFaults {
    pub film_defect: f64,
    pub controller_fault: f64,
    pub sensor_fault: f64,
}

pub struct WindowHeat {
    surface_c: f64,
    hot_spot_c: f64,
    delaminated: bool,
    cracked: bool,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct WindowHeatOutputs {
    pub surface_c: f64,
    pub hot_spot_c: f64,
    pub power_w: f64,
    pub overheat_tripped: bool,
    pub delaminated: bool,
    pub cracked: bool,
}

impl WindowHeat {
    pub fn new(initial_c: f64) -> Self {
        Self { surface_c: initial_c, hot_spot_c: initial_c, delaminated: false, cracked: false }
    }

    pub fn is_cracked(&self) -> bool {
        self.cracked
    }
    pub fn is_delaminated(&self) -> bool {
        self.delaminated
    }

    #[allow(clippy::too_many_arguments)]
    pub fn step(&mut self, static_air_c: f64, recovery_c: f64, lwc_kg_m3: f64, beta0: f64, tas_m_s: f64, ambient_pressure_pa: f64, faults: &WindowHeatFaults, dt_s: f64) -> WindowHeatOutputs {
        let sensed_c = self.surface_c + (WINDOW_SENSOR_STUCK_READING_C - self.surface_c) * clamp01(faults.sensor_fault);
        let controller_ok = clamp01(faults.controller_fault) < 1.0;
        let commanded_on = if controller_ok { sensed_c < WINDOW_TARGET_C } else { true };
        let nameplate_power_w = if commanded_on { WINDOW_RATED_W } else { 0.0 };

        let bulk_heater_w_m2 = nameplate_power_w / WINDOW_AREA_M2;
        let bulk_equilibrium_c = equilibrium_surface_c_with_heater(WINDOW_H_W_M2K, static_air_c, recovery_c, lwc_kg_m3, beta0, tas_m_s, ambient_pressure_pa, bulk_heater_w_m2);
        self.surface_c = relax_toward_equilibrium_c(self.surface_c, bulk_equilibrium_c, WINDOW_THERMAL_TAU_S, dt_s);

        let defect = clamp01(faults.film_defect);
        let concentration = 1.0 / (1.0 - defect.min(0.995)).powi(2);
        let hot_spot_heater_w_m2 = bulk_heater_w_m2 * concentration;
        let hot_spot_equilibrium_c = equilibrium_surface_c_with_heater(WINDOW_H_W_M2K, static_air_c, recovery_c, lwc_kg_m3, beta0, tas_m_s, ambient_pressure_pa, hot_spot_heater_w_m2);
        self.hot_spot_c = relax_toward_equilibrium_c(self.hot_spot_c, hot_spot_equilibrium_c, WINDOW_THERMAL_TAU_S, dt_s);

        if self.hot_spot_c > WINDOW_OVERHEAT_PROTECT_C + WINDOW_DELAMINATION_MARGIN_C {
            self.delaminated = true;
        }
        if self.hot_spot_c > WINDOW_OVERHEAT_PROTECT_C + WINDOW_CRACK_MARGIN_C {
            self.cracked = true;
        }

        WindowHeatOutputs {
            surface_c: self.surface_c,
            hot_spot_c: self.hot_spot_c,
            power_w: nameplate_power_w,
            overheat_tripped: self.surface_c > WINDOW_OVERHEAT_PROTECT_C,
            delaminated: self.delaminated,
            cracked: self.cracked,
        }
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub struct RainRemovalFaults {
    pub system_fault: f64,
}

const SHEAR_COEFFICIENT_KG_M2_S_PA: f64 = 2.0e-5;
const JET_AIR_DENSITY_KG_M3: f64 = 1.0;

pub struct RainRemoval {
    film_kg_m2: f64,
}

impl RainRemoval {
    pub fn new() -> Self {
        Self { film_kg_m2: 0.0 }
    }

    pub fn film_kg_m2(&self) -> f64 {
        self.film_kg_m2
    }

    pub fn step(&mut self, catch_kg_m2_s: f64, jet_velocity_m_s: f64, evaporation_kg_m2_s: f64, faults: &RainRemovalFaults, dt_s: f64) -> f64 {
        let dt = dt_s.max(0.0);
        let effective_velocity = jet_velocity_m_s.max(0.0) * (1.0 - clamp01(faults.system_fault));
        let dynamic_pressure_pa = 0.5 * JET_AIR_DENSITY_KG_M3 * effective_velocity * effective_velocity;
        let shear_removal_kg_m2_s = SHEAR_COEFFICIENT_KG_M2_S_PA * dynamic_pressure_pa;
        let net_kg_m2_s = catch_kg_m2_s.max(0.0) - evaporation_kg_m2_s.max(0.0) - shear_removal_kg_m2_s;
        self.film_kg_m2 = (self.film_kg_m2 + net_kg_m2_s * dt).max(0.0);
        self.film_kg_m2
    }
}

impl Default for RainRemoval {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use super::super::util::recovery_temperature_c;

    fn icing_condition() -> (f64, f64, f64, f64, f64, f64) {
        let static_c = -10.0;
        let tas = 100.0;
        (static_c, recovery_temperature_c(static_c, tas, 0.9), 5e-4, 0.6, tas, 80_000.0)
    }

    #[test]
    fn healthy_wing_anti_ice_holds_the_surface_at_or_above_freezing_in_icing_conditions() {
        let (static_c, recovery_c, lwc, beta0, tas, p) = icing_condition();
        let mut surface = BleedAntiIceSurface::new(WING_ANTI_ICE, static_c);
        let mut out = BleedAntiIceOutputs::default();
        for _ in 0..300 {
            out = surface.step(1.0, static_c, recovery_c, lwc, beta0, tas, p, &BleedAntiIceFaults::default(), 1.0);
        }
        assert!(out.surface_c >= -0.5, "healthy anti-ice should hold near/above freezing, got {}", out.surface_c);
        assert!(out.bleed_delivered_kg_s > 0.0);
    }

    #[test]
    fn a_valve_stuck_closed_leaves_the_surface_to_ice_like_the_unheated_case() {
        let (static_c, recovery_c, lwc, beta0, tas, p) = icing_condition();
        let mut surface = BleedAntiIceSurface::new(WING_ANTI_ICE, static_c);
        let faults = BleedAntiIceFaults { valve_stuck_closed: 1.0, ..Default::default() };
        let mut out = BleedAntiIceOutputs::default();
        for _ in 0..300 {
            out = surface.step(1.0, static_c, recovery_c, lwc, beta0, tas, p, &faults, 1.0);
        }
        assert_eq!(out.bleed_delivered_kg_s, 0.0);
        assert!(out.surface_c < 0.0);
    }

    #[test]
    fn a_valve_stuck_open_keeps_heating_and_can_overheat_once_no_longer_needed() {
        let mut surface = BleedAntiIceSurface::new(WING_ANTI_ICE, 15.0);
        let faults = BleedAntiIceFaults { valve_stuck_open: 1.0, ..Default::default() };
        let mut out = BleedAntiIceOutputs::default();
        for _ in 0..300 {
            out = surface.step(0.0, 15.0, 15.0, 0.0, 0.0, 100.0, 101_325.0, &faults, 1.0);
        }
        assert!(out.overheat, "a valve stuck open with no icing load should overheat the skin, got surface {}", out.surface_c);
    }

    #[test]
    fn a_duct_leak_reduces_delivered_bleed_flow() {
        let (static_c, recovery_c, lwc, beta0, tas, p) = icing_condition();
        let mut healthy = BleedAntiIceSurface::new(WING_ANTI_ICE, static_c);
        let mut leaky = BleedAntiIceSurface::new(WING_ANTI_ICE, static_c);
        let healthy_out = healthy.step(1.0, static_c, recovery_c, lwc, beta0, tas, p, &BleedAntiIceFaults::default(), 1.0);
        let leaky_out = leaky.step(1.0, static_c, recovery_c, lwc, beta0, tas, p, &BleedAntiIceFaults { duct_leak: 0.6, ..Default::default() }, 1.0);
        assert!(leaky_out.bleed_delivered_kg_s < healthy_out.bleed_delivered_kg_s);
    }

    #[test]
    fn probe_heater_keeps_a_healthy_probe_above_freezing() {
        let (static_c, recovery_c, lwc, beta0, tas, p) = icing_condition();
        let mut probe = ProbeHeater::new(static_c);
        let mut min_tail_c = f64::INFINITY;
        for i in 0..150 {
            let out = probe.step(static_c, recovery_c, lwc, beta0, tas, p, &ProbeHeaterFaults::default(), 1.0);
            if i >= 100 {
                min_tail_c = min_tail_c.min(out.surface_c);
            }
        }
        assert!(min_tail_c > 0.0, "healthy probe heat must stay above freezing once settled, min {min_tail_c}");
    }

    #[test]
    fn a_faulted_controller_leaves_the_probe_unheated_and_icing() {
        let (static_c, recovery_c, lwc, beta0, tas, p) = icing_condition();
        let mut probe = ProbeHeater::new(static_c);
        let faults = ProbeHeaterFaults { controller_fault: 1.0, ..Default::default() };
        let mut out = ProbeHeaterOutputs::default();
        for _ in 0..60 {
            out = probe.step(static_c, recovery_c, lwc, beta0, tas, p, &faults, 1.0);
        }
        assert_eq!(out.power_w, 0.0, "a faulted controller must never command heat");
        assert!(out.surface_c < 0.0, "an unheated probe in icing conditions must run below freezing");
    }

    #[test]
    fn a_sensor_stuck_reading_warm_silently_leaves_the_probe_unheated_and_icing() {
        let (static_c, recovery_c, lwc, beta0, tas, p) = icing_condition();
        let mut probe = ProbeHeater::new(static_c);
        let faults = ProbeHeaterFaults { sensor_fault: 1.0, ..Default::default() };
        let mut out = ProbeHeaterOutputs::default();
        for _ in 0..60 {
            out = probe.step(static_c, recovery_c, lwc, beta0, tas, p, &faults, 1.0);
        }
        assert_eq!(out.power_w, 0.0, "a sensor stuck reading warm must silently suppress heating");
        assert!(out.surface_c < 0.0, "the probe itself must still be cold/icing despite the system believing it is warm");
    }

    #[test]
    fn a_fully_open_circuit_heater_delivers_no_power_even_when_commanded() {
        let (static_c, recovery_c, lwc, beta0, tas, p) = icing_condition();
        let mut probe = ProbeHeater::new(static_c);
        let faults = ProbeHeaterFaults { heater_open_circuit: 1.0, ..Default::default() };
        let out = probe.step(static_c, recovery_c, lwc, beta0, tas, p, &faults, 1.0);
        assert_eq!(out.power_w, 0.0);
    }

    #[test]
    fn window_heat_settles_near_its_target_temperature_when_healthy() {
        let mut window = WindowHeat::new(-10.0);
        let mut min_tail_c = f64::INFINITY;
        let mut max_tail_c = f64::NEG_INFINITY;
        let mut any_delaminated = false;
        for i in 0..600 {
            let out = window.step(-10.0, -10.0, 0.0, 0.0, 100.0, 101_325.0, &WindowHeatFaults::default(), 1.0);
            any_delaminated |= out.delaminated || out.cracked;
            if i >= 400 {
                min_tail_c = min_tail_c.min(out.surface_c);
                max_tail_c = max_tail_c.max(out.surface_c);
            }
        }
        assert!(min_tail_c > 20.0 && max_tail_c < WINDOW_OVERHEAT_PROTECT_C, "settled band [{min_tail_c},{max_tail_c}] should bracket the {WINDOW_TARGET_C} C target well clear of the {WINDOW_OVERHEAT_PROTECT_C} C cutout");
        assert!(!any_delaminated);
    }

    #[test]
    fn a_film_defect_concentrates_heat_into_a_hot_spot_that_can_delaminate() {
        let mut window = WindowHeat::new(-10.0);
        let faults = WindowHeatFaults { film_defect: 0.9, ..Default::default() };
        let mut out = WindowHeatOutputs::default();
        for _ in 0..60 {
            out = window.step(-10.0, -10.0, 0.0, 0.0, 100.0, 101_325.0, &faults, 1.0);
        }
        assert!(out.hot_spot_c > out.surface_c, "hot spot {} vs bulk {}", out.hot_spot_c, out.surface_c);
        assert!(out.delaminated, "a severe film defect must overheat its own hot spot into delamination");
    }

    #[test]
    fn a_healthy_controller_and_overheat_cutout_prevent_delamination() {
        let mut window = WindowHeat::new(-10.0);
        for _ in 0..600 {
            let out = window.step(-10.0, -10.0, 0.0, 0.0, 100.0, 101_325.0, &WindowHeatFaults::default(), 1.0);
            assert!(!out.delaminated && !out.cracked);
        }
    }

    #[test]
    fn a_controller_fault_can_stick_the_heater_on_and_overheat_a_healthy_film() {
        let mut window = WindowHeat::new(-10.0);
        let faults = WindowHeatFaults { controller_fault: 1.0, ..Default::default() };
        let mut out = WindowHeatOutputs::default();
        for _ in 0..300 {
            out = window.step(-10.0, -10.0, 0.0, 0.0, 100.0, 101_325.0, &faults, 1.0);
        }
        assert!(out.overheat_tripped, "a stuck-on heater with no working cutout must overheat, surface {}", out.surface_c);
    }

    #[test]
    fn crack_is_irreversible_once_reached() {
        let mut window = WindowHeat::new(-10.0);
        let faults = WindowHeatFaults { film_defect: 0.97, ..Default::default() };
        for _ in 0..60 {
            window.step(-10.0, -10.0, 0.0, 0.0, 100.0, 101_325.0, &faults, 1.0);
        }
        assert!(window.is_cracked());
        for _ in 0..60 {
            window.step(-10.0, -10.0, 0.0, 0.0, 100.0, 101_325.0, &WindowHeatFaults::default(), 1.0);
        }
        assert!(window.is_cracked());
    }

    #[test]
    fn rain_removal_reduces_film_thickness_when_jet_is_on() {
        let mut off = RainRemoval::new();
        let mut on = RainRemoval::new();
        for _ in 0..100 {
            off.step(0.02, 0.0, 0.0, &RainRemovalFaults::default(), 0.1);
            on.step(0.02, 200.0, 0.0, &RainRemovalFaults::default(), 0.1);
        }
        assert!(on.film_kg_m2() < off.film_kg_m2(), "on {} vs off {}", on.film_kg_m2(), off.film_kg_m2());
    }

    #[test]
    fn a_faulted_rain_removal_system_is_less_effective() {
        let mut healthy = RainRemoval::new();
        let mut faulted = RainRemoval::new();
        for _ in 0..100 {
            healthy.step(0.02, 200.0, 0.0, &RainRemovalFaults::default(), 0.1);
            faulted.step(0.02, 200.0, 0.0, &RainRemovalFaults { system_fault: 1.0 }, 0.1);
        }
        assert!(faulted.film_kg_m2() > healthy.film_kg_m2());
    }

    #[test]
    fn no_nan_at_rest_or_zero_dt() {
        let mut surface = BleedAntiIceSurface::new(WING_ANTI_ICE, 15.0);
        let out = surface.step(0.0, 15.0, 15.0, 0.0, 0.0, 0.0, 101_325.0, &BleedAntiIceFaults::default(), 0.0);
        assert!(!out.surface_c.is_nan());
        let mut probe = ProbeHeater::new(15.0);
        let probe_out = probe.step(15.0, 15.0, 0.0, 0.0, 0.0, 101_325.0, &ProbeHeaterFaults::default(), 0.0);
        assert!(!probe_out.surface_c.is_nan());
        let mut window = WindowHeat::new(15.0);
        let window_out = window.step(15.0, 15.0, 0.0, 0.0, 0.0, 101_325.0, &WindowHeatFaults::default(), 0.0);
        assert!(!window_out.surface_c.is_nan() && !window_out.hot_spot_c.is_nan());
        let mut rain = RainRemoval::new();
        let film = rain.step(0.0, 0.0, 0.0, &RainRemovalFaults::default(), 0.0);
        assert!(!film.is_nan());
    }
}
