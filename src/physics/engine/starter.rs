//! The pneumatic (bleed-air) air-turbine starter: `engines.cfg`'s own
//! `starter_type = 2 ; Bleed Air`. Real air starters are a torque-speed
//! device, high stall torque falling off roughly linearly to zero at a
//! self-sustaining cutoff speed; this is what turns the HP spool during
//! `EngineState::Starting`/`Restarting`, before combustion can carry it the
//! rest of the way. Hung starts (the starter alone cannot reach
//! self-sustaining speed against the compressor's own drag if the supply
//! is weak) and hot starts (too much fuel scheduled for the airflow the
//! starter has produced so far) both emerge from this torque and
//! `combustor.rs`'s energy balance; nothing here is a scripted outcome.
//!
//! No public torque figure exists for the Trent 900's starter, so its
//! stall torque is derived from a generic large-turbofan pneumatic
//! starter's peak shaft power and the cutoff speed below, via the standard
//! result that a linear torque-speed curve peaks in power at half its
//! cutoff speed: `T_stall = 4 * P_peak / omega_cutoff`. Small/medium
//! turbofan starters are commonly cited around 100-150 kW, but this
//! model's own HP compressor design power (`params::inertia`'s
//! `hpc_design`-scale figure, tens of MW at 100% speed, scaling with the
//! cube of corrected speed — see `compressor.rs`) demands roughly 450-500
//! kW to motor the core through the compressor's own drag up to the 20%
//! N3 combustion floor with a working margin; `PEAK_POWER_W` is sized to
//! that requirement (self-consistent with this model's own compressor
//! curve) rather than taken from small-engine starter data, since a
//! four-engine A380-class powerplant's starter is plausibly larger than a
//! narrowbody's regardless.
use super::params::N3_DESIGN_RPM;

/// Peak shaft power of the pneumatic starter, watts. Derived (see module
/// docs), not measured: sized against this model's own HP compressor drag
/// curve so the starter can reach the light-off floor with margin. No
/// Trent-900-specific figure is public.
pub const PEAK_POWER_W: f64 = 600_000.0;

/// The HP spool speed, as a fraction of `N3_DESIGN_RPM`, at which the start
/// valve closes and the starter (a sprag clutch) stops contributing. Large
/// turbofans keep the starter assisting well past light-off, to about half
/// core speed: the A320's CFM56 start valve closes at 50% N2 (A320 FCOM,
/// engine start). No Trent 900 figure is public, so this generic
/// large-turbofan value is used. An earlier 30% left the core to climb from
/// there to self-sustaining speed on combustion alone, which a fuel-air
/// limited start cannot do.
pub const CUTOFF_N3_FRAC: f64 = 0.50;

fn cutoff_omega_rad_s() -> f64 {
    (CUTOFF_N3_FRAC * N3_DESIGN_RPM) * std::f64::consts::PI / 30.0
}

fn stall_torque_n_m() -> f64 {
    4.0 * PEAK_POWER_W / cutoff_omega_rad_s()
}

/// Starter torque on the HP spool, N·m, at the given HP spool speed
/// (RPM) and supply availability (0 = no bleed pressure, 1 = full rated
/// supply — e.g. a weak/low-pressure ground cart or a partly-spooled APU
/// would pass less than 1, which is exactly how a hung start emerges here
/// rather than being a separate scripted case).
pub fn torque_n_m(n3_rpm: f64, supply_fraction: f64) -> f64 {
    let omega = n3_rpm.max(0.0) * std::f64::consts::PI / 30.0;
    let cutoff = cutoff_omega_rad_s();
    if omega >= cutoff {
        return 0.0;
    }
    stall_torque_n_m() * supply_fraction.clamp(0.0, 1.0) * (1.0 - omega / cutoff)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn stall_torque_is_highest_at_zero_speed() {
        let stall = torque_n_m(0.0, 1.0);
        let partway = torque_n_m(N3_DESIGN_RPM * CUTOFF_N3_FRAC * 0.5, 1.0);
        assert!(stall > partway);
        assert!(partway > 0.0);
    }

    #[test]
    fn torque_reaches_zero_at_the_cutoff_speed() {
        let at_cutoff = torque_n_m(N3_DESIGN_RPM * CUTOFF_N3_FRAC, 1.0);
        assert!(at_cutoff.abs() < 1e-6);
        let past_cutoff = torque_n_m(N3_DESIGN_RPM, 1.0);
        assert!(past_cutoff.abs() < 1e-6);
    }

    #[test]
    fn a_weak_supply_gives_proportionally_less_torque() {
        let full = torque_n_m(0.0, 1.0);
        let half = torque_n_m(0.0, 0.5);
        assert!((half - full / 2.0).abs() < 1e-6);
    }
}
