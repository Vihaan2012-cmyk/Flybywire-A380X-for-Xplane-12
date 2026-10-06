use super::smoke;

pub type ZoneId = usize;

pub const CP_AIR_J_PER_KG_K: f64 = 1005.0;

pub const SEA_LEVEL_AIR_DENSITY_KG_M3: f64 = 1.225;

const GAMMA_AIR: f64 = 1.4;

const RECOVERY_FACTOR_TURBULENT: f64 = 0.9;

const SOLAR_ABSORPTIVITY_TYPICAL: f64 = 0.3;

const INSULATION_ATTENUATION: f64 = 0.7;

const ABSOLUTE_ZERO_C: f64 = -273.15;

fn floor_temp_c(t: f64) -> f64 {
    if !t.is_finite() {
        return ABSOLUTE_ZERO_C;
    }
    t.max(ABSOLUTE_ZERO_C)
}

fn external_h_w_m2k(true_airspeed_m_s: f64) -> f64 {
    10.0 + 5.0 * true_airspeed_m_s.abs().sqrt()
}

fn recovery_temperature_c(static_temp_c: f64, mach: f64, recovery_factor: f64) -> f64 {
    let static_k = static_temp_c + 273.15;
    let recovered_k = static_k * (1.0 + recovery_factor * (GAMMA_AIR - 1.0) / 2.0 * mach * mach);
    recovered_k - 273.15
}

pub fn isa_static_temp_c(altitude_m: f64) -> f64 {
    const SEA_LEVEL_TEMP_C: f64 = 15.0;
    const LAPSE_RATE_K_PER_M: f64 = -0.0065;
    const TROPOPAUSE_ALTITUDE_M: f64 = 11_000.0;
    const TROPOPAUSE_TEMP_C: f64 = -56.5;
    if altitude_m <= TROPOPAUSE_ALTITUDE_M {
        SEA_LEVEL_TEMP_C + LAPSE_RATE_K_PER_M * altitude_m
    } else {
        TROPOPAUSE_TEMP_C
    }
}

#[derive(Clone, Copy, Debug)]
pub struct OutsideAir {
    pub static_temp_c: f64,
    pub mach: f64,
    pub true_airspeed_m_s: f64,
}
impl OutsideAir {
    pub fn recovery_temp_c(&self) -> f64 {
        recovery_temperature_c(self.static_temp_c, self.mach, RECOVERY_FACTOR_TURBULENT)
    }
}

pub struct Zone {
    pub name: &'static str,
    pub air_temp_c: f64,
    pub structure_temp_c: f64,
    pub air_mass_kg: f64,
    pub structure_thermal_mass_j_per_k: f64,
    pub air_structure_ua_w_per_k: f64,
    pub exterior_skin_area_m2: f64,
    pub sun_exposure_fraction: f64,
    pub insulation_effectiveness: f64,
    pub baseline_heat_w: f64,
    injected_heat_w: f64,
    pub smoke_kg: f64,
    injected_smoke_kg_s: f64,
    pub smoke_decay_per_s: f64,
}

impl Zone {
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        name: &'static str,
        volume_m3: f64,
        structure_thermal_mass_j_per_k: f64,
        air_structure_ua_w_per_k: f64,
        exterior_skin_area_m2: f64,
        sun_exposure_fraction: f64,
        baseline_heat_w: f64,
        initial_temp_c: f64,
    ) -> Self {
        Self {
            name,
            air_temp_c: initial_temp_c,
            structure_temp_c: initial_temp_c,
            air_mass_kg: (volume_m3.max(0.0) * SEA_LEVEL_AIR_DENSITY_KG_M3).max(1.0),
            structure_thermal_mass_j_per_k: structure_thermal_mass_j_per_k.max(1.0),
            air_structure_ua_w_per_k: air_structure_ua_w_per_k.max(0.0),
            exterior_skin_area_m2: exterior_skin_area_m2.max(0.0),
            sun_exposure_fraction: sun_exposure_fraction.clamp(0.0, 1.0),
            insulation_effectiveness: 1.0,
            baseline_heat_w,
            injected_heat_w: 0.0,
            smoke_kg: 0.0,
            injected_smoke_kg_s: 0.0,
            smoke_decay_per_s: 0.0,
        }
    }

    pub fn air_thermal_mass_j_per_k(&self) -> f64 {
        self.air_mass_kg * CP_AIR_J_PER_KG_K
    }

    pub fn smoke_concentration_kg_per_kg(&self) -> f64 {
        smoke::concentration_kg_per_kg(self.smoke_kg, self.air_mass_kg)
    }
}

pub struct ConductionLink {
    pub a: ZoneId,
    pub b: ZoneId,
    pub ua_w_per_k: f64,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ZoneRef {
    Zone(ZoneId),
    OutsideAir,
}

pub struct VentilationLink {
    pub vented_zone: ZoneId,
    pub reference: ZoneRef,
    pub nameplate_flow_kg_s: f64,
    pub health: f64,
}
impl VentilationLink {
    pub fn flow_kg_s(&self) -> f64 {
        self.nameplate_flow_kg_s.max(0.0) * self.health.clamp(0.0, 1.0)
    }
}

pub struct ThermalNetwork {
    pub zones: Vec<Zone>,
    pub conduction_links: Vec<ConductionLink>,
    pub ventilation_links: Vec<VentilationLink>,
}

impl Default for ThermalNetwork {
    fn default() -> Self {
        Self::new()
    }
}

impl ThermalNetwork {
    pub fn new() -> Self {
        Self { zones: Vec::new(), conduction_links: Vec::new(), ventilation_links: Vec::new() }
    }

    pub fn add_zone(&mut self, zone: Zone) -> ZoneId {
        self.zones.push(zone);
        self.zones.len() - 1
    }

    pub fn add_conduction_link(&mut self, a: ZoneId, b: ZoneId, ua_w_per_k: f64) {
        self.conduction_links.push(ConductionLink { a, b, ua_w_per_k });
    }

    pub fn add_ventilation_link(&mut self, vented_zone: ZoneId, reference: ZoneRef, nameplate_flow_kg_s: f64) -> usize {
        self.ventilation_links.push(VentilationLink { vented_zone, reference, nameplate_flow_kg_s, health: 1.0 });
        self.ventilation_links.len() - 1
    }

    pub fn set_ventilation_health(&mut self, link_index: usize, health: f64) {
        if let Some(l) = self.ventilation_links.get_mut(link_index) {
            l.health = health.clamp(0.0, 1.0);
        }
    }

    pub fn inject_heat_w(&mut self, zone: ZoneId, watts: f64) {
        if let Some(z) = self.zones.get_mut(zone) {
            z.injected_heat_w += watts;
        }
    }

    pub fn inject_smoke_kg_s(&mut self, zone: ZoneId, kg_s: f64) {
        if let Some(z) = self.zones.get_mut(zone) {
            z.injected_smoke_kg_s += kg_s.max(0.0);
        }
    }

    pub fn air_temp_c(&self, zone: ZoneId) -> f64 {
        self.zones.get(zone).map(|z| z.air_temp_c).unwrap_or(f64::NAN)
    }

    pub fn structure_temp_c(&self, zone: ZoneId) -> f64 {
        self.zones.get(zone).map(|z| z.structure_temp_c).unwrap_or(f64::NAN)
    }

    pub fn smoke_concentration(&self, zone: ZoneId) -> f64 {
        self.zones.get(zone).map(|z| z.smoke_concentration_kg_per_kg()).unwrap_or(0.0)
    }

    pub fn step(&mut self, dt_s: f64, outside: &OutsideAir, solar_flux_w_m2: f64) {
        if dt_s <= 0.0 {
            return;
        }
        let substeps = self.stable_substep_count(dt_s);
        let h = dt_s / substeps as f64;
        for _ in 0..substeps {
            self.substep(h, outside, solar_flux_w_m2);
        }
        for z in self.zones.iter_mut() {
            z.injected_heat_w = 0.0;
            z.injected_smoke_kg_s = 0.0;
        }
    }

    fn stable_substep_count(&self, dt_s: f64) -> u32 {
        const CONSERVATIVE_MAX_EXTERNAL_H_W_M2K: f64 = 200.0;
        const STABILITY_FRACTION: f64 = 0.2;

        let mut fastest_rate_per_s: f64 = 0.0;
        for (i, z) in self.zones.iter().enumerate() {
            let vent_ua: f64 = self
                .ventilation_links
                .iter()
                .filter(|l| l.vented_zone == i)
                .map(|l| l.nameplate_flow_kg_s.max(0.0) * CP_AIR_J_PER_KG_K)
                .sum();
            let air_ua = z.air_structure_ua_w_per_k + vent_ua;
            let air_c = z.air_thermal_mass_j_per_k();
            if air_c > 0.0 {
                fastest_rate_per_s = fastest_rate_per_s.max(air_ua / air_c);
            }

            let cond_ua: f64 = self.conduction_links.iter().filter(|l| l.a == i || l.b == i).map(|l| l.ua_w_per_k).sum();
            let ext_ua = CONSERVATIVE_MAX_EXTERNAL_H_W_M2K * z.exterior_skin_area_m2;
            let struct_ua = z.air_structure_ua_w_per_k + ext_ua + cond_ua;
            if z.structure_thermal_mass_j_per_k > 0.0 {
                fastest_rate_per_s = fastest_rate_per_s.max(struct_ua / z.structure_thermal_mass_j_per_k);
            }
        }
        if fastest_rate_per_s <= 0.0 {
            return 1;
        }
        let stable_dt = STABILITY_FRACTION / fastest_rate_per_s;
        let n = (dt_s / stable_dt).ceil();
        n.clamp(1.0, 2000.0) as u32
    }

    fn substep(&mut self, dt_s: f64, outside: &OutsideAir, solar_flux_w_m2: f64) {
        let n = self.zones.len();
        let mut q_air = vec![0.0_f64; n];
        let mut q_structure = vec![0.0_f64; n];
        let mut smoke_flux_kg_s = vec![0.0_f64; n];

        let recovery_c = outside.recovery_temp_c();
        let ext_h = external_h_w_m2k(outside.true_airspeed_m_s);

        for (i, z) in self.zones.iter().enumerate() {
            q_air[i] += z.baseline_heat_w + z.injected_heat_w;
            let conv = z.air_structure_ua_w_per_k * (z.structure_temp_c - z.air_temp_c);
            q_air[i] += conv;
            q_structure[i] -= conv;
            let insulation_factor = 1.0 - z.insulation_effectiveness.clamp(0.0, 1.0) * INSULATION_ATTENUATION;
            q_structure[i] += ext_h * z.exterior_skin_area_m2 * insulation_factor * (recovery_c - z.structure_temp_c);
            q_structure[i] += solar_flux_w_m2.max(0.0) * z.exterior_skin_area_m2 * z.sun_exposure_fraction * SOLAR_ABSORPTIVITY_TYPICAL;
        }

        for link in &self.conduction_links {
            let term = link.ua_w_per_k * (self.zones[link.b].structure_temp_c - self.zones[link.a].structure_temp_c);
            q_structure[link.a] += term;
            q_structure[link.b] -= term;
        }

        for link in &self.ventilation_links {
            let flow = link.flow_kg_s();
            if flow <= 0.0 {
                continue;
            }
            let (ref_temp, ref_conc) = match link.reference {
                ZoneRef::Zone(z) => (self.zones[z].air_temp_c, self.zones[z].smoke_concentration_kg_per_kg()),
                ZoneRef::OutsideAir => (recovery_c, 0.0),
            };
            let own = &self.zones[link.vented_zone];
            q_air[link.vented_zone] += flow * CP_AIR_J_PER_KG_K * (ref_temp - own.air_temp_c);
            let flux_in = smoke::advected_smoke_flux_kg_s(flow, ref_conc);
            let flux_out = smoke::advected_smoke_flux_kg_s(flow, own.smoke_concentration_kg_per_kg());
            smoke_flux_kg_s[link.vented_zone] += flux_in - flux_out;
        }

        for (i, z) in self.zones.iter_mut().enumerate() {
            z.air_temp_c = floor_temp_c(z.air_temp_c + q_air[i] / z.air_thermal_mass_j_per_k() * dt_s);
            z.structure_temp_c = floor_temp_c(z.structure_temp_c + q_structure[i] / z.structure_thermal_mass_j_per_k * dt_s);
            z.smoke_kg = smoke::step_smoke_kg(z.smoke_kg, z.injected_smoke_kg_s, smoke_flux_kg_s[i], z.smoke_decay_per_s, dt_s);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn calm_ground_air(static_temp_c: f64) -> OutsideAir {
        OutsideAir { static_temp_c, mach: 0.0, true_airspeed_m_s: 0.0 }
    }

    #[test]
    fn conduction_chain_reaches_hand_solved_steady_state() {
        let mut net = ThermalNetwork::new();
        let z0 = net.add_zone(Zone::new("Source", 1.0, 1.0e5, 5000.0, 0.0, 0.0, 1000.0, 15.0));
        let z1 = net.add_zone(Zone::new("Sink", 1.0, 1.0e5, 5000.0, 5.0, 0.0, 0.0, 15.0));
        net.add_conduction_link(z0, z1, 20.0);

        let outside = calm_ground_air(15.0);
        for _ in 0..150_000 {
            net.step(1.0, &outside, 0.0);
        }

        let ext_ua = 10.0 * 5.0 * (1.0 - INSULATION_ATTENUATION);
        let t1_predicted = 15.0 + 1000.0 / ext_ua;
        let t0_predicted = t1_predicted + 1000.0 / 20.0;

        assert!((net.air_temp_c(z0) - t0_predicted).abs() < 1.0, "z0 {} vs predicted {}", net.air_temp_c(z0), t0_predicted);
        assert!((net.air_temp_c(z1) - t1_predicted).abs() < 1.0, "z1 {} vs predicted {}", net.air_temp_c(z1), t1_predicted);
        assert!(net.air_temp_c(z0) > net.air_temp_c(z1), "heat must flow from the source zone to the sink zone");
    }

    #[test]
    fn smoke_reaches_hand_solved_steady_state_with_ventilation() {
        let mut net = ThermalNetwork::new();
        let z0 = net.add_zone(Zone::new("Cargo", 10.0, 1.0e5, 50.0, 2.0, 0.0, 0.0, 15.0));
        net.add_ventilation_link(z0, ZoneRef::OutsideAir, 0.2);

        let outside = calm_ground_air(15.0);
        const PRODUCED_KG_S: f64 = 0.0005;
        for _ in 0..20_000 {
            net.inject_smoke_kg_s(z0, PRODUCED_KG_S);
            net.step(1.0, &outside, 0.0);
        }

        let predicted_conc = PRODUCED_KG_S / 0.2;
        assert!((net.smoke_concentration(z0) - predicted_conc).abs() / predicted_conc < 0.02, "conc {} vs predicted {}", net.smoke_concentration(z0), predicted_conc);
        assert!(net.smoke_concentration(z0) > 0.0);
    }

    #[test]
    fn smoke_never_negative_and_zero_without_injection() {
        let mut net = ThermalNetwork::new();
        let z0 = net.add_zone(Zone::new("Bay", 5.0, 1.0e5, 50.0, 2.0, 0.0, 0.0, 15.0));
        net.add_ventilation_link(z0, ZoneRef::OutsideAir, 0.3);
        let outside = calm_ground_air(15.0);
        for _ in 0..1000 {
            net.step(1.0, &outside, 0.0);
        }
        assert_eq!(net.smoke_concentration(z0), 0.0);
    }

    #[test]
    fn sun_exposure_raises_steady_state_structure_temperature() {
        let outside = calm_ground_air(15.0);
        const FLUX_W_M2: f64 = 800.0;

        let mut shaded = ThermalNetwork::new();
        let z_shaded = shaded.add_zone(Zone::new("Shaded", 5.0, 2.0e5, 100.0, 10.0, 0.0, 0.0, 15.0));
        let mut sunny = ThermalNetwork::new();
        let z_sunny = sunny.add_zone(Zone::new("Sunny", 5.0, 2.0e5, 100.0, 10.0, 0.5, 0.0, 15.0));

        for _ in 0..60_000 {
            shaded.step(1.0, &outside, FLUX_W_M2);
            sunny.step(1.0, &outside, FLUX_W_M2);
        }

        assert!((shaded.structure_temp_c(z_shaded) - 15.0).abs() < 0.5, "no sun exposure: should stay near ambient, got {}", shaded.structure_temp_c(z_shaded));
        let q_sun_w = FLUX_W_M2 * 10.0 * 0.5 * SOLAR_ABSORPTIVITY_TYPICAL;
        let ext_ua = 10.0 * 10.0 * (1.0 - INSULATION_ATTENUATION);
        let predicted_sunny = 15.0 + q_sun_w / ext_ua;
        assert!((sunny.structure_temp_c(z_sunny) - predicted_sunny).abs() < 1.0, "sunny {} vs predicted {}", sunny.structure_temp_c(z_sunny), predicted_sunny);
        assert!(sunny.structure_temp_c(z_sunny) > shaded.structure_temp_c(z_shaded));
    }

    #[test]
    fn blocking_ventilation_raises_steady_state_temperature() {
        let outside = calm_ground_air(15.0);

        let mut healthy = ThermalNetwork::new();
        let z_h = healthy.add_zone(Zone::new("Bay", 5.0, 2.0e5, 200.0, 2.0, 0.0, 800.0, 15.0));
        let ref_h = healthy.add_zone(Zone::new("Supply", 100.0, 1.0e6, 1000.0, 0.0, 0.0, 0.0, 24.0));
        healthy.add_ventilation_link(z_h, ZoneRef::Zone(ref_h), 0.3);

        let mut blocked = ThermalNetwork::new();
        let z_b = blocked.add_zone(Zone::new("Bay", 5.0, 2.0e5, 200.0, 2.0, 0.0, 800.0, 15.0));
        let ref_b = blocked.add_zone(Zone::new("Supply", 100.0, 1.0e6, 1000.0, 0.0, 0.0, 0.0, 24.0));
        let vent = blocked.add_ventilation_link(z_b, ZoneRef::Zone(ref_b), 0.3);
        blocked.set_ventilation_health(vent, 0.0);

        for _ in 0..20_000 {
            healthy.step(1.0, &outside, 0.0);
            blocked.step(1.0, &outside, 0.0);
        }

        assert!(blocked.air_temp_c(z_b) > healthy.air_temp_c(z_h) + 10.0, "blocked {} vs healthy {}", blocked.air_temp_c(z_b), healthy.air_temp_c(z_h));
    }

    #[test]
    fn zero_dt_is_a_no_op_and_rest_state_is_finite() {
        let mut net = ThermalNetwork::new();
        let z0 = net.add_zone(Zone::new("Zone", 1.0, 1.0e5, 10.0, 1.0, 0.0, 0.0, 15.0));
        let outside = calm_ground_air(15.0);
        let before = net.air_temp_c(z0);
        net.step(0.0, &outside, 0.0);
        assert_eq!(net.air_temp_c(z0), before);
        for _ in 0..100 {
            net.step(1.0, &outside, 0.0);
        }
        assert!(net.air_temp_c(z0).is_finite());
        assert!(net.structure_temp_c(z0).is_finite());
        assert!((net.air_temp_c(z0) - 15.0).abs() < 0.01, "an unforced zone at ambient should stay at ambient");
    }

    #[test]
    fn substepping_keeps_a_stiff_zone_stable_at_a_large_timestep() {
        let mut net = ThermalNetwork::new();
        const U_W_PER_K: f64 = 100.0;
        const VENT_UA_W_PER_K: f64 = 50.0;
        let z0 = net.add_zone(Zone::new("Stiff", 0.01, 50.0, U_W_PER_K, 10.0, 0.0, 500.0, 15.0));
        net.zones[z0].insulation_effectiveness = 0.0;
        net.add_ventilation_link(z0, ZoneRef::OutsideAir, VENT_UA_W_PER_K / CP_AIR_J_PER_KG_K);
        let outside = calm_ground_air(15.0);
        net.step(100.0, &outside, 0.0);

        assert!(net.structure_temp_c(z0).is_finite());
        assert!(net.air_temp_c(z0).is_finite());
        let predicted_air = 15.0 + 5.0;
        let predicted_structure = 15.0 + 2.5;
        assert!((net.air_temp_c(z0) - predicted_air).abs() < 0.1, "air {} vs predicted {}", net.air_temp_c(z0), predicted_air);
        assert!((net.structure_temp_c(z0) - predicted_structure).abs() < 0.1, "structure {} vs predicted {}", net.structure_temp_c(z0), predicted_structure);
    }

    #[test]
    fn damaged_insulation_lets_a_zone_track_outside_temperature_faster() {
        let cold_outside = calm_ground_air(-40.0);

        let mut intact = ThermalNetwork::new();
        let z_intact = intact.add_zone(Zone::new("Compartment", 5.0, 2.0e5, 100.0, 8.0, 0.0, 0.0, 20.0));

        let mut damaged = ThermalNetwork::new();
        let z_damaged = damaged.add_zone(Zone::new("Compartment", 5.0, 2.0e5, 100.0, 8.0, 0.0, 0.0, 20.0));
        damaged.zones[z_damaged].insulation_effectiveness = 0.0;

        for _ in 0..600 {
            intact.step(1.0, &cold_outside, 0.0);
            damaged.step(1.0, &cold_outside, 0.0);
        }

        assert!(damaged.structure_temp_c(z_damaged) < intact.structure_temp_c(z_intact) - 2.0, "damaged {} vs intact {}", damaged.structure_temp_c(z_damaged), intact.structure_temp_c(z_intact));
        assert!(damaged.structure_temp_c(z_damaged).is_finite());
    }

    #[test]
    fn isa_temperature_falls_with_altitude_then_holds_at_tropopause() {
        assert!((isa_static_temp_c(0.0) - 15.0).abs() < 1e-9);
        assert!(isa_static_temp_c(5000.0) < isa_static_temp_c(0.0));
        assert!((isa_static_temp_c(11_000.0) - (-56.5)).abs() < 1e-6);
        assert_eq!(isa_static_temp_c(15_000.0), isa_static_temp_c(11_000.0));
    }

    #[test]
    fn recovery_temperature_exceeds_static_at_speed_and_matches_at_zero_mach() {
        let outside_fast = OutsideAir { static_temp_c: -50.0, mach: 0.85, true_airspeed_m_s: 250.0 };
        assert!(outside_fast.recovery_temp_c() > outside_fast.static_temp_c);
        let outside_slow = OutsideAir { static_temp_c: 15.0, mach: 0.0, true_airspeed_m_s: 0.0 };
        assert!((outside_slow.recovery_temp_c() - 15.0).abs() < 1e-9);
    }
}
