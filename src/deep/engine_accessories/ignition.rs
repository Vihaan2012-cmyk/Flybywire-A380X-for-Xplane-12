//! Ignition: two independent high-energy capacitor-discharge exciters (A/B),
//! each driving its own igniter plug, the standard redundant AC turbine
//! ignition arrangement (both plugs normally fire together for a start;
//! either chain alone can light the engine).
//!
//! An exciter charges a capacitor from the aircraft AC bus through a
//! charging resistor (an RC circuit) until the capacitor reaches its
//! trigger voltage, then dumps its stored energy through the igniter plug
//! and starts charging again -- so its spark rate is not a scripted
//! frequency, it falls out of the RC charge time, and a failing exciter
//! (less charging current) recharges slower and sparks less often, never
//! at a fixed reduced rate assigned by a table. Plug erosion widens the
//! spark gap, which raises the voltage needed to break the gap down
//! (Paschen-law-type behaviour: breakdown voltage rises with gap length at
//! a given pressure); once that exceeds the exciter's fixed peak output
//! voltage, the plug stops firing altogether -- not a gradual derate, a
//! hard cutoff exactly like a real worn-out igniter that simply will not
//! strike, which is also why an igniter is normally life-limited on erosion
//! rather than run to that point.
//!
//! No Trent-900 exciter/igniter figures are public. `CAPACITOR_ENERGY_J`
//! (**GENERIC**, ~12-20 J is the commonly quoted range for high-energy AC
//! turbine ignition exciters, e.g. Champion/Unison ignition-system data
//! sheets), `CHARGE_TIME_CONSTANT_S` (**GENERIC**, sized for a ~2-4 Hz
//! healthy spark rate, typical of that class of exciter) and the breakdown-
//! voltage range (**GENERIC**, ~10 kV new to ~22 kV badly eroded, against a
//! ~20 kV typical exciter peak output) are all derived/typical figures, not
//! measured Trent 900 data.

/// Aircraft AC ignition bus, V (115 V AC, ARINC 700-series standard).
pub const V_BUS_V: f64 = 115.0;
/// Stored discharge energy at full charge, J (**GENERIC**, see module docs).
pub const CAPACITOR_ENERGY_J: f64 = 18.0;
/// RC charge time constant at full exciter health, s (**GENERIC**).
const CHARGE_TIME_CONSTANT_S: f64 = 0.15;
/// Trigger threshold as a fraction of the asymptotic charge voltage.
const THRESHOLD_FRAC: f64 = 0.95;
/// Breakdown voltage required for a new plug and a fully-eroded one, V, and
/// the exciter's fixed peak output voltage, V (**GENERIC**, see module docs).
const BREAKDOWN_NEW_V: f64 = 10_000.0;
const BREAKDOWN_ERODED_V: f64 = 22_000.0;
const EXCITER_PEAK_OUTPUT_V: f64 = 20_000.0;

/// The breakdown voltage a plug's gap needs at a given erosion, V.
pub fn breakdown_voltage_v(erosion: f64) -> f64 {
    let e = erosion.clamp(0.0, 1.0);
    BREAKDOWN_NEW_V + e * (BREAKDOWN_ERODED_V - BREAKDOWN_NEW_V)
}

/// Whether the exciter's fixed peak output can still break this plug's gap
/// down at all.
pub fn can_fire(erosion: f64) -> bool {
    breakdown_voltage_v(erosion) <= EXCITER_PEAK_OUTPUT_V
}

/// Steady-state spark rate, Hz: zero if unpowered, if the exciter has
/// failed outright, or if erosion has put the gap's breakdown voltage
/// beyond the exciter's reach; otherwise the reciprocal of the RC charge
/// time to the trigger threshold, which a failing (but not dead) exciter
/// stretches out by charging more slowly.
pub fn spark_rate_hz(powered: bool, erosion: f64, exciter_failure: f64) -> f64 {
    let health = 1.0 - exciter_failure.clamp(0.0, 1.0);
    if !powered || health <= 1e-6 || !can_fire(erosion) {
        return 0.0;
    }
    let tau = CHARGE_TIME_CONSTANT_S / health;
    let period_s = -tau * (1.0 - THRESHOLD_FRAC).ln();
    1.0 / period_s
}

/// Faults the ignition system can carry, 0 (healthy) .. 1 (fully failed),
/// one pair per independent exciter/igniter chain.
#[derive(Clone, Copy, Debug, Default)]
pub struct IgnitionFaults {
    pub exciter_a_failure: f64,
    pub exciter_b_failure: f64,
    /// Spark-gap erosion on each plug, accumulated by whatever wear model
    /// tracks total spark count/hours elsewhere; supplied here as a
    /// fraction like every other fault in this directory.
    pub igniter_a_erosion: f64,
    pub igniter_b_erosion: f64,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct IgnitionState {
    pub chain_a_hz: f64,
    pub chain_b_hz: f64,
    pub chain_a_firing: bool,
    pub chain_b_firing: bool,
    /// True only if *neither* chain can produce a spark: with two
    /// independent exciter/igniter chains, this is what an actual loss of
    /// ignition capability requires.
    pub no_ignition_available: bool,
}

/// One step. `powered` is true whenever either the start sequence or a
/// crew/EEC continuous-ignition selection has energised the exciters (both
/// chains are normally powered together; this module does not decide which
/// flight phases select continuous ignition, that belongs to whatever calls
/// it).
pub fn step(powered: bool, faults: &IgnitionFaults) -> IgnitionState {
    let a_hz = spark_rate_hz(powered, faults.igniter_a_erosion, faults.exciter_a_failure);
    let b_hz = spark_rate_hz(powered, faults.igniter_b_erosion, faults.exciter_b_failure);
    IgnitionState {
        chain_a_hz: a_hz,
        chain_b_hz: b_hz,
        chain_a_firing: a_hz > 0.0,
        chain_b_firing: b_hz > 0.0,
        no_ignition_available: powered && a_hz <= 0.0 && b_hz <= 0.0,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn unpowered_gives_no_sparks_and_no_nan() {
        let s = step(false, &IgnitionFaults::default());
        assert_eq!(s.chain_a_hz, 0.0);
        assert!(!s.no_ignition_available);
    }

    #[test]
    fn a_healthy_new_plug_sparks_at_a_positive_rate_when_powered() {
        let s = step(true, &IgnitionFaults::default());
        assert!(s.chain_a_hz > 0.0 && s.chain_b_hz > 0.0);
        assert!(s.chain_a_firing && s.chain_b_firing);
    }

    #[test]
    fn a_failed_exciter_stops_its_own_chain_but_not_the_other() {
        let s = step(true, &IgnitionFaults { exciter_a_failure: 1.0, ..Default::default() });
        assert_eq!(s.chain_a_hz, 0.0);
        assert!(s.chain_b_hz > 0.0);
        assert!(!s.no_ignition_available, "the B chain alone still gives ignition");
    }

    #[test]
    fn both_exciters_failed_means_no_ignition_available() {
        let s = step(true, &IgnitionFaults { exciter_a_failure: 1.0, exciter_b_failure: 1.0, ..Default::default() });
        assert!(s.no_ignition_available);
    }

    #[test]
    fn a_badly_eroded_plug_cannot_break_down_and_stops_firing() {
        assert!(can_fire(0.0));
        assert!(!can_fire(1.0));
        let s = step(true, &IgnitionFaults { igniter_a_erosion: 1.0, ..Default::default() });
        assert_eq!(s.chain_a_hz, 0.0);
        assert!(s.chain_b_hz > 0.0);
    }

    #[test]
    fn a_partially_failing_exciter_sparks_slower_not_just_less_energetically() {
        let healthy = spark_rate_hz(true, 0.0, 0.0);
        let degraded = spark_rate_hz(true, 0.0, 0.6);
        assert!(degraded > 0.0 && degraded < healthy);
    }

    #[test]
    fn breakdown_voltage_rises_monotonically_with_erosion() {
        assert!(breakdown_voltage_v(1.0) > breakdown_voltage_v(0.5));
        assert!(breakdown_voltage_v(0.5) > breakdown_voltage_v(0.0));
    }
}
