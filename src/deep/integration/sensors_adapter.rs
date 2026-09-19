//! `EnvironmentTruth` -> the primitive `(tas_ms, sat_c, lwc_gm3)` triple
//! `deep::sensors`' pitot/TAT/AoA-vane models take as their `step`
//! arguments directly (`pitot::PitotProbe::step`, `tat_probe::TatProbe::
//! step`, `aoa_vane::AoaVane::step` all read exactly these three plus
//! their own true-pressure/true-AoA inputs, which belong to the flight
//! model, not weather -- see each field's doc below).
//!
//! Unit note: `deep::sensors`' probes take LWC in **g/m^3**
//! (`lwc_gm3`, e.g. `pitot.rs`'s own doc: "typical continuous-maximum
//! icing envelopes"), while `deep::fire_ice::icing::IcingEnvironment` (and
//! this crate's `weather_truth::lwc_kg_m3_from_conditions`) uses **kg/m^3**
//! -- the two areas chose different units independently (each
//! self-contained per the brief's hard rule 2) and this adapter is exactly
//! where that conversion belongs, tested below so the factor of 1000 is
//! never silently wrong in either direction.

use super::weather_truth::{dominant_cloud, lwc_kg_m3_from_conditions, EnvironmentTruth};

/// What every probe model in `deep::sensors` needs from the environment,
/// in each model's own native units (see module doc for the LWC unit).
#[derive(Clone, Copy, Debug, Default)]
pub struct ProbeEnvironment {
    pub tas_ms: f64,
    pub sat_c: f64,
    pub lwc_gm3: f64,
}

pub fn probe_environment(truth: &EnvironmentTruth) -> ProbeEnvironment {
    let cloud = truth.weather.as_ref().and_then(dominant_cloud);
    let lwc_kg_m3 = lwc_kg_m3_from_conditions(truth.sat_c, cloud);
    ProbeEnvironment { tas_ms: truth.tas_ms, sat_c: truth.sat_c, lwc_gm3: lwc_kg_m3 * 1000.0 }
}

/// True total/static pressure at the pitot tip, from ambient (static)
/// pressure and Mach, the standard compressible pitot-static relation
/// (`Pt/Ps = (1 + 0.2*M^2)^3.5`, e.g. Anderson, *Fundamentals of
/// Aerodynamics*, ch. 8; the same relation `deep::sensors::adr`'s own
/// module doc cites for the reverse computation) -- `deep::sensors::pitot::
/// PitotProbe::step` wants the *true* pressures a perfect probe would see,
/// which is exactly this: X-Plane/the airframe's real ambient state, not
/// yet run through any probe fault.
pub fn true_total_pressure_pa(ambient_pressure_pa: f64, mach: f64) -> f64 {
    let m = mach.max(0.0);
    ambient_pressure_pa.max(0.0) * (1.0 + 0.2 * m * m).powf(3.5)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::deep::integration::weather_truth::mach_from_tas_sat;
    use crate::deep::sensors::pitot::{PitotFaults, PitotProbe};
    use crate::deep::sensors::tat_probe::{TatProbe, TatProbeFaults};
    use crate::xp::{WeatherCloudLayer, WeatherSample};

    fn icing_truth() -> EnvironmentTruth {
        let mut clouds = [WeatherCloudLayer::default(); 3];
        clouds[0] = WeatherCloudLayer { cloud_type: 1.0, coverage: 1.0, alt_base_m: 0.0, alt_top_m: 3000.0 };
        EnvironmentTruth {
            sat_c: -10.0,
            leading_edge_c: -8.0,
            ambient_pressure_pa: 80_000.0,
            tas_ms: 100.0,
            precipitation_on_aircraft_ratio: 0.0,
            weather: Some(WeatherSample { precip_rate_alt: 0.3, precip_rate: 0.3, turbulence_alt: 0.0, clouds, detailed: true }),
        }
    }

    #[test]
    fn lwc_conversion_is_exactly_a_factor_of_a_thousand_from_fire_ices_kg_m3() {
        let truth = icing_truth();
        let probe_env = probe_environment(&truth);
        let cloud = truth.weather.as_ref().and_then(dominant_cloud);
        let kg_m3 = lwc_kg_m3_from_conditions(truth.sat_c, cloud);
        assert!((probe_env.lwc_gm3 - kg_m3 * 1000.0).abs() < 1e-12);
        assert!(probe_env.lwc_gm3 > 0.0);
    }

    #[test]
    fn clear_and_warm_gives_zero_lwc_for_the_probes_too() {
        let mut truth = icing_truth();
        truth.sat_c = 20.0;
        assert_eq!(probe_environment(&truth).lwc_gm3, 0.0);
    }

    #[test]
    fn true_total_pressure_matches_ambient_at_zero_mach() {
        assert!((true_total_pressure_pa(101_325.0, 0.0) - 101_325.0).abs() < 1e-6);
    }

    #[test]
    fn true_total_pressure_exceeds_ambient_and_grows_with_mach() {
        let low = true_total_pressure_pa(80_000.0, 0.3);
        let high = true_total_pressure_pa(80_000.0, 0.6);
        assert!(low > 80_000.0);
        assert!(high > low);
    }

    #[test]
    fn a_pitot_probe_driven_from_the_derived_environment_reads_the_derived_total_pressure_when_healthy() {
        let truth = icing_truth();
        let probe_env = probe_environment(&truth);
        let mach = mach_from_tas_sat(truth.tas_ms, truth.sat_c);
        let total = true_total_pressure_pa(truth.ambient_pressure_pa, mach);
        let mut probe = PitotProbe::new(total);
        let out = probe.step(total, truth.ambient_pressure_pa, probe_env.tas_ms, probe_env.sat_c, probe_env.lwc_gm3, true, &PitotFaults::default(), 1.0);
        assert!((out.sensed_total_pressure_pa - total).abs() < 1.0, "a healthy, powered probe should track true total pressure");
    }

    #[test]
    fn a_tat_probe_driven_from_the_derived_environment_produces_a_finite_reading() {
        let truth = icing_truth();
        let probe_env = probe_environment(&truth);
        let mach = mach_from_tas_sat(truth.tas_ms, truth.sat_c);
        let mut tat = TatProbe::new(truth.sat_c);
        let out = tat.step(probe_env.sat_c, mach, probe_env.tas_ms, probe_env.lwc_gm3, true, &TatProbeFaults::default(), 1.0);
        assert!(out.sensed_tat_c.is_finite());
        assert!(out.sensed_tat_c >= probe_env.sat_c, "total temperature must be at or above static (kinetic heating never cools)");
    }
}
