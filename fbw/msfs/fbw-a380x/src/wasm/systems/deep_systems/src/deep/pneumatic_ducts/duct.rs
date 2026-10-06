pub const GAMMA: f64 = 1.4;
pub const R_AIR_J_KG_K: f64 = 287.057005;
pub const CP_AIR_J_KG_K: f64 = 1005.0;
pub const CV_AIR_J_KG_K: f64 = CP_AIR_J_KG_K - R_AIR_J_KG_K;
const CRITICAL_PRESSURE_RATIO: f64 = 0.528_281_787_717_685_7;

pub fn orifice_mass_flow_kg_s(
    discharge_coefficient: f64,
    area_m2: f64,
    upstream_pa: f64,
    upstream_k: f64,
    downstream_pa: f64,
) -> f64 {
    let p1 = upstream_pa.max(0.0);
    let p2 = downstream_pa.max(0.0);
    let t1 = upstream_k.max(1.0);
    let a = area_m2.max(0.0);
    if p1 <= 0.0 || a <= 0.0 || p2 >= p1 {
        return 0.0;
    }
    let pressure_ratio = (p2 / p1).clamp(0.0, 1.0);
    let flow_function = if pressure_ratio <= CRITICAL_PRESSURE_RATIO {
        (GAMMA * (2.0 / (GAMMA + 1.0)).powf((GAMMA + 1.0) / (GAMMA - 1.0))).sqrt()
    } else {
        (2.0 * GAMMA / (GAMMA - 1.0)
            * (pressure_ratio.powf(2.0 / GAMMA) - pressure_ratio.powf((GAMMA + 1.0) / GAMMA)))
        .max(0.0)
        .sqrt()
    };
    discharge_coefficient.max(0.0) * a * p1 / (R_AIR_J_KG_K * t1).sqrt() * flow_function
}

#[derive(Clone, Copy, Debug)]
pub struct DuctVolume {
    volume_m3: f64,
    pressure_pa: f64,
    temp_k: f64,
    mass_kg: f64,
}

impl DuctVolume {
    pub fn new(volume_m3: f64, pressure_pa: f64, temp_k: f64) -> Self {
        let volume_m3 = volume_m3.max(1e-6);
        let temp_k = temp_k.max(1.0);
        let pressure_pa = pressure_pa.max(0.0);
        let mass_kg = pressure_pa * volume_m3 / (R_AIR_J_KG_K * temp_k);
        Self { volume_m3, pressure_pa, temp_k, mass_kg }
    }

    pub fn volume_m3(&self) -> f64 {
        self.volume_m3
    }
    pub fn pressure_pa(&self) -> f64 {
        self.pressure_pa
    }
    pub fn temp_k(&self) -> f64 {
        self.temp_k
    }
    pub fn mass_kg(&self) -> f64 {
        self.mass_kg
    }

    pub fn add_mass(&mut self, dm_kg: f64, at_temp_k: f64, at_pa: f64) {
        let mass = self.mass_kg.max(0.0);
        let new_mass = (mass + dm_kg).max(0.0);
        if dm_kg > 0.0 {
            let at_temp_k = at_temp_k.max(1.0);
            let at_pa = at_pa.max(1.0);
            if new_mass <= 0.0 {
                return;
            }
            let incoming_volume = dm_kg * R_AIR_J_KG_K * at_temp_k / at_pa;
            let m_c_t = mass * self.temp_k + dm_kg * at_temp_k;
            let volume_quotient = 1.0 + incoming_volume / self.volume_m3;
            let new_temp = (m_c_t / new_mass) * volume_quotient.powf(GAMMA - 1.0);
            let new_pressure = m_c_t * R_AIR_J_KG_K / (self.volume_m3 + incoming_volume)
                * volume_quotient.powf(GAMMA);
            self.temp_k = new_temp.max(1.0);
            self.pressure_pa = new_pressure.max(0.0);
            self.mass_kg = new_mass;
        } else if mass <= 0.0 {
            self.mass_kg = new_mass;
        } else {
            let ratio = (new_mass / mass).max(0.0);
            self.pressure_pa = (self.pressure_pa * ratio.powf(GAMMA)).max(0.0);
            self.temp_k = (self.temp_k * ratio.powf(GAMMA - 1.0)).max(1.0);
            self.mass_kg = new_mass;
        }
    }

    pub fn conduct_to(&mut self, ua_w_k: f64, sink_k: f64, dt_s: f64) -> f64 {
        let dt = dt_s.max(0.0);
        let capacity = (self.mass_kg.max(1e-9) * CV_AIR_J_KG_K).max(1e-6);
        let k = ua_w_k.max(0.0) / capacity;
        let before = self.temp_k;
        let after = sink_k + (before - sink_k) * (-k * dt).exp();
        let heat_w = if dt > 0.0 { (before - after) * capacity / dt } else { ua_w_k.max(0.0) * (before - sink_k) };
        if before > 1e-6 {
            self.pressure_pa = (self.pressure_pa * after / before).max(0.0);
        }
        self.temp_k = after.max(1.0);
        heat_w
    }
}

fn pressures_after_transfer(up: &DuctVolume, down: &DuctVolume, dm_kg: f64) -> (f64, f64) {
    let m_up = up.mass_kg.max(0.0);
    let remaining = (m_up - dm_kg).max(0.0);
    let up_pa = if m_up > 0.0 { up.pressure_pa * (remaining / m_up).powf(GAMMA) } else { 0.0 };
    let v_in = dm_kg * R_AIR_J_KG_K * up.temp_k.max(1.0) / up.pressure_pa.max(1.0);
    let m_c_t = down.mass_kg.max(0.0) * down.temp_k + dm_kg * up.temp_k.max(1.0);
    let down_pa = m_c_t * R_AIR_J_KG_K / down.volume_m3 * (1.0 + v_in / down.volume_m3).powf(GAMMA - 1.0);
    (up_pa, down_pa)
}

fn equilibrium_clamp_kg(up: &DuctVolume, down: &DuctVolume, requested_kg: f64) -> f64 {
    let requested = requested_kg.max(0.0);
    let m_up = up.mass_kg.max(0.0);
    if requested <= 0.0 || m_up <= 0.0 || up.pressure_pa <= down.pressure_pa {
        return 0.0;
    }
    let overshoot = |dm: f64| {
        let (p_up, p_down) = pressures_after_transfer(up, down, dm);
        p_up - p_down
    };
    if overshoot(requested.min(m_up)) >= 0.0 {
        return requested.min(m_up);
    }
    let (mut lo, mut hi) = (0.0_f64, requested.min(m_up));
    for _ in 0..60 {
        let mid = 0.5 * (lo + hi);
        if overshoot(mid) > 0.0 {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    lo
}

pub fn transfer_kg(dt_s: f64, discharge_coefficient: f64, area_m2: f64, a: &mut DuctVolume, b: &mut DuctVolume) -> f64 {
    let dt = dt_s.max(0.0);
    if dt <= 0.0 || area_m2 <= 0.0 {
        return 0.0;
    }
    let (upstream, downstream, sign) = if a.pressure_pa >= b.pressure_pa { (&*a, &*b, 1.0) } else { (&*b, &*a, -1.0) };
    let rate = orifice_mass_flow_kg_s(discharge_coefficient, area_m2, upstream.pressure_pa, upstream.temp_k, downstream.pressure_pa);
    let mut dm = equilibrium_clamp_kg(upstream, downstream, rate * dt);
    let (from_temp, from_pa) = (upstream.temp_k, upstream.pressure_pa);
    if sign > 0.0 {
        a.add_mass(-dm, from_temp, from_pa);
        b.add_mass(dm, from_temp, from_pa);
    } else {
        b.add_mass(-dm, from_temp, from_pa);
        a.add_mass(dm, from_temp, from_pa);
        dm = -dm;
    }
    dm
}

pub fn one_way_transfer_kg(
    dt_s: f64,
    discharge_coefficient: f64,
    forward_area_m2: f64,
    check_valve_seat_area_m2: f64,
    upstream: &mut DuctVolume,
    downstream: &mut DuctVolume,
    backflow_leak_fraction: f64,
) -> f64 {
    let dt = dt_s.max(0.0);
    if dt <= 0.0 {
        return 0.0;
    }
    if upstream.pressure_pa >= downstream.pressure_pa {
        if forward_area_m2 <= 0.0 {
            return 0.0;
        }
        let rate = orifice_mass_flow_kg_s(discharge_coefficient, forward_area_m2, upstream.pressure_pa, upstream.temp_k, downstream.pressure_pa);
        let dm = equilibrium_clamp_kg(upstream, downstream, rate * dt);
        let (t, p) = (upstream.temp_k, upstream.pressure_pa);
        upstream.add_mass(-dm, t, p);
        downstream.add_mass(dm, t, p);
        dm
    } else {
        let leak = backflow_leak_fraction.clamp(0.0, 1.0);
        if leak <= 0.0 || check_valve_seat_area_m2 <= 0.0 {
            return 0.0;
        }
        let rate = orifice_mass_flow_kg_s(discharge_coefficient * leak, check_valve_seat_area_m2, downstream.pressure_pa, downstream.temp_k, upstream.pressure_pa);
        let dm = equilibrium_clamp_kg(downstream, upstream, rate * dt);
        let (t, p) = (downstream.temp_k, downstream.pressure_pa);
        downstream.add_mass(-dm, t, p);
        upstream.add_mass(dm, t, p);
        -dm
    }
}

pub fn passive_valve_open_fraction(pressure_diff_pa: f64, spring_pa: f64) -> f64 {
    (2.0 / std::f64::consts::PI * (pressure_diff_pa / spring_pa.max(1.0)).atan()).clamp(0.0, 1.0)
}

#[derive(Clone, Copy, Debug, Default)]
pub struct DuctSectionFaults {
    pub leak: f64,
    pub rupture: f64,
    pub insulation_damage: f64,
}

#[derive(Clone, Debug)]
pub struct DuctSection {
    pub gas: DuctVolume,
    pub zone: &'static str,
    diameter_m: f64,
    insulation_ua_w_k: f64,
}

impl DuctSection {
    const BARE_PIPE_MULTIPLIER: f64 = 10.0;

    pub fn new(zone: &'static str, volume_m3: f64, diameter_m: f64, insulation_ua_w_k: f64, initial_pa: f64, initial_k: f64) -> Self {
        Self {
            gas: DuctVolume::new(volume_m3, initial_pa, initial_k),
            zone,
            diameter_m: diameter_m.max(1e-3),
            insulation_ua_w_k: insulation_ua_w_k.max(0.0),
        }
    }

    pub fn full_bore_area_m2(&self) -> f64 {
        std::f64::consts::PI / 4.0 * self.diameter_m * self.diameter_m
    }

    pub fn effective_ua_w_k(&self, faults: &DuctSectionFaults) -> f64 {
        let damage = faults.insulation_damage.clamp(0.0, 1.0);
        self.insulation_ua_w_k * (1.0 + (Self::BARE_PIPE_MULTIPLIER - 1.0) * damage)
    }

    pub fn step_insulation(&mut self, zone_k: f64, dt_s: f64, faults: &DuctSectionFaults) -> f64 {
        self.gas.conduct_to(self.effective_ua_w_k(faults), zone_k, dt_s)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn choked_flow_does_not_increase_past_the_critical_ratio() {
        let at_ratio = |pr: f64| orifice_mass_flow_kg_s(0.65, 0.001, 300_000.0, 350.0, 300_000.0 * pr);
        let choked = at_ratio(CRITICAL_PRESSURE_RATIO * 0.5);
        let at_critical = at_ratio(CRITICAL_PRESSURE_RATIO);
        assert!((choked - at_critical).abs() / at_critical < 1e-9, "choked flow must be independent of how far below critical the ratio is");
        let just_below_critical = at_ratio(CRITICAL_PRESSURE_RATIO * 0.99);
        assert!(just_below_critical <= at_critical * 1.0001);
    }

    #[test]
    fn zero_pressure_or_area_gives_zero_flow_not_nan() {
        assert_eq!(orifice_mass_flow_kg_s(0.65, 0.0, 300_000.0, 300.0, 100_000.0), 0.0);
        assert_eq!(orifice_mass_flow_kg_s(0.65, 0.001, 0.0, 300.0, 100_000.0), 0.0);
        assert_eq!(orifice_mass_flow_kg_s(0.65, 0.001, 100_000.0, 300.0, 200_000.0), 0.0, "reversed gradient gives no flow, not a negative one");
    }

    #[test]
    fn adding_and_removing_the_same_mass_returns_the_mass_but_not_the_state() {
        let mut v = DuctVolume::new(1.0, 300_000.0, 400.0);
        let (p0, t0, m0) = (v.pressure_pa(), v.temp_k(), v.mass_kg());
        v.add_mass(0.05, 500.0, 400_000.0);
        assert!(v.pressure_pa() > p0, "adding mass at higher pressure raises this volume's pressure");
        v.add_mass(-0.05, v.temp_k(), v.pressure_pa());

        assert!((v.mass_kg() - m0).abs() < 1e-9);

        let t2_expected = 401.6887;
        let p2_expected = 301_266.5;
        assert!((v.temp_k() - t2_expected).abs() < 0.01, "temp {} vs hand-solved {}", v.temp_k(), t2_expected);
        assert!((v.pressure_pa() - p2_expected).abs() / p2_expected < 1e-5, "pressure {} vs hand-solved {}", v.pressure_pa(), p2_expected);
        assert!(v.pressure_pa() > p0 && v.temp_k() > t0);
        assert!((v.pressure_pa() - v.mass_kg() * R_AIR_J_KG_K * v.temp_k() / v.volume_m3()).abs() / p0 < 1e-9);
    }

    #[test]
    fn an_empty_volume_accepts_mass_without_producing_nan() {
        let mut v = DuctVolume::new(0.01, 0.0, 288.0);
        assert_eq!(v.mass_kg(), 0.0);
        v.add_mass(-0.001, 288.0, 100_000.0);
        assert!(v.pressure_pa().is_finite() && v.temp_k().is_finite());
        v.add_mass(0.001, 500.0, 300_000.0);
        assert!(v.pressure_pa() > 0.0 && v.pressure_pa().is_finite());
    }

    #[test]
    fn transfer_kg_moves_gas_from_high_to_low_pressure_and_conserves_total_mass() {
        let mut hot = DuctVolume::new(0.5, 300_000.0, 480.0);
        let mut cold = DuctVolume::new(0.5, 100_000.0, 288.0);
        let total_before = hot.mass_kg() + cold.mass_kg();
        let moved = transfer_kg(0.1, 0.65, 0.005, &mut hot, &mut cold);
        assert!(moved > 0.0, "flow goes from the higher-pressure hot volume to the lower-pressure cold one");
        assert!((hot.mass_kg() + cold.mass_kg() - total_before).abs() < 1e-9, "mass is conserved across the transfer");
        assert!(hot.pressure_pa() < 300_000.0 && cold.pressure_pa() > 100_000.0);
    }

    #[test]
    fn transfer_kg_never_reverses_the_pressure_gradient_in_one_big_step() {
        let mut hot = DuctVolume::new(0.01, 300_000.0, 480.0);
        let mut cold = DuctVolume::new(0.01, 100_000.0, 288.0);
        transfer_kg(5.0, 0.65, 0.02, &mut hot, &mut cold);
        assert!(hot.pressure_pa() >= cold.pressure_pa() - 1.0, "the equilibrium clamp must stop a single step from overshooting past equalisation, hot={} cold={}", hot.pressure_pa(), cold.pressure_pa());
    }

    #[test]
    fn insulation_loses_heat_toward_the_zone_and_damage_speeds_it_up() {
        let mut healthy = DuctSection::new("PYLON_1", 0.2, 0.1, 5.0, 300_000.0, 473.15);
        let mut damaged = healthy.clone();
        let ok = DuctSectionFaults::default();
        let bad = DuctSectionFaults { insulation_damage: 1.0, ..Default::default() };
        for _ in 0..600 {
            healthy.step_insulation(288.15, 1.0, &ok);
            damaged.step_insulation(288.15, 1.0, &bad);
        }
        assert!(healthy.gas.temp_k() > 288.15, "still cooling toward, not past, the zone");
        assert!(damaged.gas.temp_k() < healthy.gas.temp_k(), "a torn blanket loses heat faster");
    }

    #[test]
    fn a_healthy_check_valve_blocks_reverse_flow_but_a_failed_one_leaks_backward() {
        let mut low = DuctVolume::new(0.05, 100_000.0, 288.0);
        let mut high = DuctVolume::new(0.05, 300_000.0, 400.0);
        let moved = one_way_transfer_kg(1.0, 0.7, 0.001, 0.001, &mut low, &mut high, 0.0);
        assert_eq!(moved, 0.0, "a healthy check valve must block reverse flow entirely");
        let mut low2 = DuctVolume::new(0.05, 100_000.0, 288.0);
        let mut high2 = DuctVolume::new(0.05, 300_000.0, 400.0);
        let moved2 = one_way_transfer_kg(1.0, 0.7, 0.001, 0.001, &mut low2, &mut high2, 1.0);
        assert!(moved2 < 0.0, "a fully failed check valve must let the higher-pressure side push mass backward");
    }

    #[test]
    fn a_shut_control_valve_still_gets_check_valve_protection() {
        let mut upstream = DuctVolume::new(0.05, 300_000.0, 400.0);
        let mut downstream = DuctVolume::new(0.05, 100_000.0, 288.0);
        let moved = one_way_transfer_kg(1.0, 0.7, 0.0, 0.001, &mut upstream, &mut downstream, 0.0);
        assert_eq!(moved, 0.0, "a shut control valve must stop forward flow");
        let mut low = DuctVolume::new(0.05, 100_000.0, 288.0);
        let mut high = DuctVolume::new(0.05, 300_000.0, 400.0);
        let moved2 = one_way_transfer_kg(1.0, 0.7, 0.0, 0.001, &mut low, &mut high, 0.0);
        assert_eq!(moved2, 0.0, "a shut control valve with a healthy check valve must still block reverse flow");
    }

    #[test]
    fn no_nan_at_rest_dt_zero() {
        let mut s = DuctSection::new("WING_ROOT", 0.3, 0.15, 4.0, 101_325.0, 288.15);
        let w = s.step_insulation(288.15, 0.0, &DuctSectionFaults::default());
        assert_eq!(w, 0.0);
        assert!(s.gas.pressure_pa().is_finite() && s.gas.temp_k().is_finite());
    }

    #[test]
    fn passive_valve_opens_further_with_more_differential_and_never_goes_negative() {
        assert_eq!(passive_valve_open_fraction(-50_000.0, 6894.757), 0.0, "no reverse opening");
        assert_eq!(passive_valve_open_fraction(0.0, 6894.757), 0.0);
        let half_ish = passive_valve_open_fraction(6894.757, 6894.757);
        assert!((half_ish - 0.5).abs() < 1e-9);
        let more = passive_valve_open_fraction(3.0 * 6894.757, 6894.757);
        assert!(more > half_ish && more <= 1.0);
    }
}
