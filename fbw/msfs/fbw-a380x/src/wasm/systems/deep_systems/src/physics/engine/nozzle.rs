use super::gas::{self, critical_pressure_ratio};

#[derive(Clone, Copy, Debug, Default)]
pub struct NozzleResult {
    pub thrust_n: f64,
    pub exit_velocity_m_s: f64,
    pub choked: bool,
}

pub fn thrust(mdot_kg_s: f64, tt_k: f64, pt_pa: f64, ambient_pressure_pa: f64, flight_velocity_m_s: f64, gamma: f64, r_specific: f64) -> NozzleResult {
    if mdot_kg_s <= 1e-6 || pt_pa <= ambient_pressure_pa {
        return NozzleResult { thrust_n: -mdot_kg_s.max(0.0) * flight_velocity_m_s, exit_velocity_m_s: 0.0, choked: false };
    }
    let cp = gamma * r_specific / (gamma - 1.0);
    let pr_available = pt_pa / ambient_pressure_pa;
    let pr_critical = critical_pressure_ratio(gamma);

    if pr_available <= pr_critical {
        let t_exit = tt_k * gas::temperature_ratio_from_pressure_ratio(ambient_pressure_pa / pt_pa, gamma);
        let v_exit = (2.0 * cp * (tt_k - t_exit)).max(0.0).sqrt();
        let momentum = mdot_kg_s * (v_exit - flight_velocity_m_s);
        NozzleResult { thrust_n: momentum, exit_velocity_m_s: v_exit, choked: false }
    } else {
        let t_exit = tt_k * (2.0 / (gamma + 1.0));
        let v_exit = (gamma * r_specific * t_exit).max(0.0).sqrt();
        let p_exit = pt_pa * (2.0 / (gamma + 1.0)).powf(gamma / (gamma - 1.0));
        let rho_exit = p_exit / (r_specific * t_exit);
        let area = if rho_exit > 1e-9 && v_exit > 1e-6 { mdot_kg_s / (rho_exit * v_exit) } else { 0.0 };
        let momentum = mdot_kg_s * (v_exit - flight_velocity_m_s);
        let pressure_term = (p_exit - ambient_pressure_pa) * area;
        NozzleResult { thrust_n: momentum + pressure_term, exit_velocity_m_s: v_exit, choked: true }
    }
}

fn exit_density_and_velocity(tt_k: f64, pt_pa: f64, ambient_pressure_pa: f64, gamma: f64, r_specific: f64) -> (f64, f64) {
    let pr_available = pt_pa / ambient_pressure_pa.max(1.0);
    let pr_critical = critical_pressure_ratio(gamma);
    if pr_available <= pr_critical {
        let t_exit = tt_k * gas::temperature_ratio_from_pressure_ratio(ambient_pressure_pa / pt_pa, gamma);
        let cp = gamma * r_specific / (gamma - 1.0);
        let v_exit = (2.0 * cp * (tt_k - t_exit)).max(0.0).sqrt();
        (ambient_pressure_pa / (r_specific * t_exit.max(1.0)), v_exit)
    } else {
        let t_exit = tt_k * (2.0 / (gamma + 1.0));
        let v_exit = (gamma * r_specific * t_exit).max(0.0).sqrt();
        let p_exit = pt_pa * (2.0 / (gamma + 1.0)).powf(gamma / (gamma - 1.0));
        (p_exit / (r_specific * t_exit.max(1.0)), v_exit)
    }
}

pub fn mass_flow_capacity(tt_k: f64, pt_pa: f64, ambient_pressure_pa: f64, area_m2: f64, gamma: f64, r_specific: f64) -> f64 {
    if tt_k <= 1.0 || pt_pa <= ambient_pressure_pa.max(0.0) || area_m2 <= 0.0 {
        return 0.0;
    }
    let (rho_exit, v_exit) = exit_density_and_velocity(tt_k, pt_pa, ambient_pressure_pa, gamma, r_specific);
    area_m2 * rho_exit * v_exit
}

pub fn design_area_m2(mdot_kg_s: f64, tt_k: f64, pt_pa: f64, ambient_pressure_pa: f64, gamma: f64, r_specific: f64) -> f64 {
    let (rho_exit, v_exit) = exit_density_and_velocity(tt_k, pt_pa, ambient_pressure_pa, gamma, r_specific);
    if rho_exit > 1e-9 && v_exit > 1e-6 {
        mdot_kg_s / (rho_exit * v_exit)
    } else {
        0.0
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use super::super::gas::{GAMMA_AIR, R_AIR};

    #[test]
    fn static_run_up_gives_positive_forward_thrust() {
        let r = thrust(100.0, 900.0, 250_000.0, 101325.0, 0.0, GAMMA_AIR, R_AIR);
        assert!(r.thrust_n > 0.0, "{:?}", r);
    }

    #[test]
    fn higher_total_pressure_gives_more_thrust() {
        let low = thrust(100.0, 900.0, 200_000.0, 101325.0, 0.0, GAMMA_AIR, R_AIR);
        let high = thrust(100.0, 900.0, 400_000.0, 101325.0, 0.0, GAMMA_AIR, R_AIR);
        assert!(high.thrust_n > low.thrust_n);
    }

    #[test]
    fn forward_flight_reduces_net_thrust_via_ram_drag() {
        let static_thrust = thrust(100.0, 900.0, 250_000.0, 101325.0, 0.0, GAMMA_AIR, R_AIR).thrust_n;
        let in_flight = thrust(100.0, 900.0, 250_000.0, 101325.0, 200.0, GAMMA_AIR, R_AIR).thrust_n;
        assert!(in_flight < static_thrust);
    }

    #[test]
    fn an_unchoked_nozzle_produces_a_cooler_subsonic_exit_and_positive_thrust() {
        let r = thrust(50.0, 900.0, 150_000.0, 101_325.0, 0.0, GAMMA_AIR, R_AIR);
        assert!(!r.choked);
        assert!(r.exit_velocity_m_s > 0.0 && r.exit_velocity_m_s.is_finite());
        assert!(r.thrust_n > 0.0, "{:?}", r);
    }

    #[test]
    fn a_high_pressure_ratio_chokes_the_nozzle() {
        let r = thrust(100.0, 1400.0, 2_000_000.0, 22_632.0, 230.0, GAMMA_AIR, R_AIR);
        assert!(r.choked);
        assert!(r.thrust_n > 0.0);
    }

    #[test]
    fn no_flow_gives_no_thrust() {
        let r = thrust(0.0, 900.0, 250_000.0, 101325.0, 0.0, GAMMA_AIR, R_AIR);
        assert!(r.thrust_n.abs() < 1e-9);
    }

    #[test]
    fn design_area_round_trips_through_mass_flow_capacity() {
        let (mdot, tt, pt, ambient) = (95.0, 1400.0, 2_000_000.0, 22_632.0);
        let area = design_area_m2(mdot, tt, pt, ambient, GAMMA_AIR, R_AIR);
        assert!(area > 0.0 && area.is_finite());
        let back = mass_flow_capacity(tt, pt, ambient, area, GAMMA_AIR, R_AIR);
        assert!((back - mdot).abs() / mdot < 1e-9, "{back} vs {mdot}");
    }

    #[test]
    fn design_area_round_trips_when_the_design_point_is_subsonic() {
        let (mdot, tt, pt, ambient) = (40.0, 900.0, 150_000.0, 101_325.0);
        let area = design_area_m2(mdot, tt, pt, ambient, GAMMA_AIR, R_AIR);
        let back = mass_flow_capacity(tt, pt, ambient, area, GAMMA_AIR, R_AIR);
        assert!((back - mdot).abs() / mdot < 1e-9, "{back} vs {mdot}");
    }

    #[test]
    fn a_hotter_inlet_temperature_reduces_capacity_at_a_fixed_area() {
        let area = 0.5;
        let cool = mass_flow_capacity(1200.0, 2_000_000.0, 22_632.0, area, GAMMA_AIR, R_AIR);
        let hot = mass_flow_capacity(1600.0, 2_000_000.0, 22_632.0, area, GAMMA_AIR, R_AIR);
        assert!(hot < cool, "{hot} vs {cool}");
    }

    #[test]
    fn zero_area_or_no_flow_conditions_give_zero_capacity() {
        assert_eq!(mass_flow_capacity(900.0, 250_000.0, 101_325.0, 0.0, GAMMA_AIR, R_AIR), 0.0);
        assert_eq!(mass_flow_capacity(900.0, 90_000.0, 101_325.0, 0.5, GAMMA_AIR, R_AIR), 0.0);
    }
}
