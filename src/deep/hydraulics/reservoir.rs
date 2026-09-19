//! Hydraulic reservoir: the fluid store the pumps draw from and returns/case
//! drains flow back to, air-pressurised (a "bootstrap" reservoir) so the
//! engine-driven pumps see a positive inlet pressure rather than pure
//! suction lift, which large transport aircraft need to avoid cavitating
//! their high-pressure pumps at altitude and in descent.
//!
//! Volumes match FlyByWire's own A380 model
//! (`a380_systems/src/hydraulic/mod.rs`, `A380HydraulicReservoirFactory`):
//! green reservoir 12 US gal max (`Volume::new::<gallon>(12.)`), yellow 12.7
//! US gal, both with a `PressureSwitch` at 25/21.76 psi relative used for
//! the low air pressure caution (reused verbatim here as this model's own
//! low pressure warning thresholds), a 5 L low-level warning threshold, and
//! a hard `MIN_USABLE_VOLUME_GAL` = 0.2 gal floor (`fbw-common`'s own
//! `Reservoir::MIN_USABLE_VOLUME_GAL`, `hydraulic/mod.rs` line 2278) below
//! which the pump inlet truly runs dry -- reused here as `unusable_m3`,
//! distinct from the 5 L figure, which is only ever a warning threshold on
//! the real system. FlyByWire's own reservoir holds air pressure fixed at
//! a flat `Pressure::new::<psi>(50.)` (`hydraulic/mod.rs` line 2311, no
//! further physics); this model reuses that same 50 psi as its nominal
//! regulated pressure (`NOMINAL_BOOST_PA`) but derives *loss* of it from
//! causes FlyByWire does not model at this level.
//!
//! The bootstrap air supply is driven by cabin/bleed-air pressurisation
//! (`pressurization_supply_fraction` below), not by this same circuit's own
//! hydraulic pressure: a reservoir pressurised *from* the pump it feeds
//! would need pressure to already exist before a pump can produce any (the
//! pump's own cavitation model, `pump::cavitation_efficiency`, is exactly
//! zero at zero inlet gauge pressure) -- an unresolvable startup deadlock.
//! Real bootstrap reservoirs on transport aircraft avoid exactly this by
//! drawing their air side from cabin/bleed air rather than from the
//! circuit they pressurise, which is the physical justification for
//! decoupling it here too.

use super::fluid;

pub const PSI_PA: f64 = 6894.757;
pub const LITER_M3: f64 = 1.0e-3;
pub const GALLON_M3: f64 = 3.785_411_784e-3;

/// FlyByWire's own A380 reservoir low air pressure caution thresholds
/// (`PressureSwitch::new(Pressure::new::<psi>(25.), Pressure::new::<psi>(21.76), ...)`),
/// a Schmitt trigger: warns below the low value, clears only above the high
/// value, so the caution does not chatter right at the threshold.
pub const LOW_PRESSURE_WARN_PSI: f64 = 25.0;
pub const LOW_PRESSURE_CLEAR_PSI: f64 = 21.76;

/// GENERIC nominal regulated reservoir air (gauge) pressure while the
/// bootstrap pressurisation source (system pressure through a reducing
/// piston) is available -- see module doc.
const NOMINAL_BOOST_PA: f64 = 50.0 * PSI_PA;

/// Faults a reservoir can carry.
#[derive(Clone, Copy, Debug, Default)]
pub struct ReservoirFaults {
    /// A crack/seal failure losing fluid straight to the bay, sized as an
    /// orifice area, m^2 (0 healthy).
    pub leak_area_m2: f64,
    /// Loss of the bootstrap air pressurisation source (failed pressurising
    /// valve, air line leak): 0 healthy .. 1 no boost at all, reservoir air
    /// relaxes toward ambient regardless of system pressure.
    pub pressurization_loss: f64,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct ReservoirOutputs {
    pub fluid_volume_m3: f64,
    pub fill_fraction: f64,
    /// Gauge pressure available at the pump inlet, Pa -- feeds
    /// `pump::cavitation_efficiency`'s air-pressure map directly (ramps to
    /// zero as level or pressurisation is lost, the physical reason a
    /// starved/depressurised reservoir cavitates the pumps rather than a
    /// scripted "pump off" symptom).
    pub inlet_air_pressure_pa: f64,
    pub low_pressure_warning: bool,
    /// Fluid quantity below the 5 L warning threshold (a caution, not yet a
    /// physical starvation -- see module doc).
    pub low_level_warning: bool,
    pub leaked_m3_s: f64,
}

#[derive(Clone, Debug)]
pub struct Reservoir {
    capacity_m3: f64,
    /// The true hard floor (`fbw-common`'s `MIN_USABLE_VOLUME_GAL`): below
    /// this the pump inlet runs dry regardless of any warning light.
    unusable_m3: f64,
    /// The A380 factory's 5 L low-level *warning* threshold -- informational
    /// only, well above `unusable_m3`.
    low_level_warn_m3: f64,
    /// State.
    fluid_volume_m3: f64,
    low_pressure_switch_on: bool,
}

impl Reservoir {
    pub fn new(capacity_m3: f64, unusable_m3: f64, low_level_warn_m3: f64, initial_fluid_m3: f64) -> Self {
        Self {
            capacity_m3: capacity_m3.max(1e-6),
            unusable_m3: unusable_m3.max(0.0),
            low_level_warn_m3: low_level_warn_m3.max(0.0),
            fluid_volume_m3: initial_fluid_m3.clamp(0.0, capacity_m3),
            low_pressure_switch_on: false,
        }
    }

    /// `fbw-common`'s `Reservoir::MIN_USABLE_VOLUME_GAL` = 0.2 gal.
    const MIN_USABLE_GAL: f64 = 0.2;

    pub fn a380_green() -> Self {
        Self::new(GALLON_M3 * 12.0, GALLON_M3 * Self::MIN_USABLE_GAL, LITER_M3 * 5.0, GALLON_M3 * 12.0)
    }
    pub fn a380_yellow() -> Self {
        Self::new(GALLON_M3 * 12.7, GALLON_M3 * Self::MIN_USABLE_GAL, LITER_M3 * 5.0, GALLON_M3 * 12.7)
    }

    pub fn fluid_volume_m3(&self) -> f64 {
        self.fluid_volume_m3
    }

    /// Integrates the reservoir's fluid balance and reports the pump inlet
    /// conditions. `inflow_m3_s` is everything returning to it (case
    /// drains, return lines, relief valve dumps, network leaks already
    /// tallied by `network::Network::step`); `outflow_m3_s` is what the
    /// pump(s) actually drew from it this step (their commanded intake,
    /// *before* any cavitation derating -- the derating is this reservoir's
    /// own inlet pressure output feeding back into the pump model, not a
    /// second starvation cliff here).
    pub fn step(&mut self, inflow_m3_s: f64, outflow_m3_s: f64, pressurization_supply_fraction: f64, faults: &ReservoirFaults, dt_s: f64) -> ReservoirOutputs {
        let dt = dt_s.max(0.0);

        let air_pa_before_leak = self.boost_pa(pressurization_supply_fraction, faults);
        let density = fluid::density_kg_m3(60.0);
        let leak_area = faults.leak_area_m2.max(0.0);
        let leak_dp = air_pa_before_leak.max(0.0);
        let leaked_m3_s = if leak_area > 0.0 && leak_dp > 0.0 { 0.61 * leak_area * (2.0 * leak_dp / density).sqrt() } else { 0.0 };

        let net = inflow_m3_s - outflow_m3_s - leaked_m3_s;
        self.fluid_volume_m3 = (self.fluid_volume_m3 + net * dt).clamp(0.0, self.capacity_m3);

        let inlet_air_pressure_pa = self.boost_pa(pressurization_supply_fraction, faults);
        let low_psi = inlet_air_pressure_pa / PSI_PA;
        if self.low_pressure_switch_on {
            if low_psi > LOW_PRESSURE_CLEAR_PSI {
                self.low_pressure_switch_on = false;
            }
        } else if low_psi < LOW_PRESSURE_WARN_PSI {
            self.low_pressure_switch_on = true;
        }

        ReservoirOutputs {
            fluid_volume_m3: self.fluid_volume_m3,
            fill_fraction: self.fluid_volume_m3 / self.capacity_m3,
            inlet_air_pressure_pa,
            low_pressure_warning: self.low_pressure_switch_on,
            low_level_warning: self.fluid_volume_m3 < self.low_level_warn_m3,
            leaked_m3_s,
        }
    }

    /// Reservoir air (gauge) pressure: `NOMINAL_BOOST_PA` scaled by the
    /// cabin/bleed-air pressurisation supply available
    /// (`pressurization_supply_fraction`, 0..1, normally 1.0), lost further
    /// to `pressurization_loss` (a faulted pressurising valve/air line), and
    /// ramped out over one "margin" band above the *true* usable floor
    /// (`unusable_m3`, not the 5 L warning threshold) as the fluid level
    /// approaches it -- a low reservoir's outlet draws air progressively,
    /// exactly like a fuel tank pump inlet unporting
    /// (`physics::fluids::unporting_factor`'s same smooth-ramp-not-a-cliff
    /// approach, re-derived here to stay self-contained).
    fn boost_pa(&self, pressurization_supply_fraction: f64, faults: &ReservoirFaults) -> f64 {
        let pressurization_fraction = pressurization_supply_fraction.clamp(0.0, 1.0) * (1.0 - faults.pressurization_loss.clamp(0.0, 1.0));
        let margin_m3 = (self.capacity_m3 * 0.1).max(1e-6);
        let level_factor = ((self.fluid_volume_m3 - self.unusable_m3) / margin_m3).clamp(0.0, 1.0);
        NOMINAL_BOOST_PA * pressurization_fraction * level_factor
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn healthy_full_reservoir_with_pressurization_supplied_shows_full_boost() {
        let mut res = Reservoir::a380_green();
        let out = res.step(0.0, 0.0, 1.0, &ReservoirFaults::default(), 0.02);
        assert!((out.inlet_air_pressure_pa - NOMINAL_BOOST_PA).abs() < 1.0);
        assert!(!out.low_pressure_warning);
    }

    #[test]
    fn draining_the_reservoir_below_unusable_collapses_inlet_pressure() {
        let unusable = GALLON_M3 * Reservoir::MIN_USABLE_GAL;
        let mut res = Reservoir::new(GALLON_M3 * 12.0, unusable, LITER_M3 * 5.0, unusable); // right at the true floor
        let out = res.step(0.0, 0.0, 1.0, &ReservoirFaults::default(), 0.02);
        assert!(out.inlet_air_pressure_pa < 1.0, "at the unusable line there should be ~no usable head: {}", out.inlet_air_pressure_pa);
        assert!(out.low_level_warning, "well below the 5 L warning threshold too");
    }

    #[test]
    fn losing_pressurization_drops_boost_even_when_full() {
        let mut res = Reservoir::a380_green();
        let faults = ReservoirFaults { pressurization_loss: 1.0, ..Default::default() };
        let out = res.step(0.0, 0.0, 1.0, &faults, 0.02);
        assert_eq!(out.inlet_air_pressure_pa, 0.0);
        assert!(out.low_pressure_warning);
    }

    #[test]
    fn no_pressurization_supply_also_drops_boost_even_with_no_fault() {
        let mut res = Reservoir::a380_green();
        let out = res.step(0.0, 0.0, 0.0, &ReservoirFaults::default(), 0.02);
        assert_eq!(out.inlet_air_pressure_pa, 0.0);
    }

    #[test]
    fn low_pressure_switch_has_hysteresis() {
        let mut res = Reservoir::a380_green();
        let low_faults = ReservoirFaults { pressurization_loss: 1.0, ..Default::default() };
        let out = res.step(0.0, 0.0, 1.0, &low_faults, 0.02);
        assert!(out.low_pressure_warning);
        // Restore a pressurisation supply fraction that gives a boost between
        // 21.76 and 25 psi -- hysteresis must keep the warning latched.
        let mid_psi = (LOW_PRESSURE_WARN_PSI + LOW_PRESSURE_CLEAR_PSI) / 2.0;
        let needed_fraction = mid_psi * PSI_PA / NOMINAL_BOOST_PA;
        let out2 = res.step(0.0, 0.0, needed_fraction, &ReservoirFaults::default(), 0.02);
        assert!(out2.low_pressure_warning, "must stay latched between clear and warn thresholds");
    }

    #[test]
    fn mass_balance_inflow_minus_outflow() {
        let mut res = Reservoir::a380_green();
        let start = res.fluid_volume_m3();
        // Draw down (no return) for a while.
        for _ in 0..100 {
            res.step(0.0, 1.0e-5, 1.0, &ReservoirFaults::default(), 0.02);
        }
        assert!(res.fluid_volume_m3() < start);
        let drained = start - res.fluid_volume_m3();
        assert!((drained - 1.0e-5 * 100.0 * 0.02).abs() / drained < 0.05);
    }

    #[test]
    fn a_leak_drains_the_reservoir_even_with_balanced_flows() {
        let mut healthy = Reservoir::a380_green();
        let mut leaking = Reservoir::a380_green();
        let leak_faults = ReservoirFaults { leak_area_m2: 5e-6, ..Default::default() };
        for _ in 0..500 {
            healthy.step(1.0e-5, 1.0e-5, 1.0, &ReservoirFaults::default(), 0.02);
            leaking.step(1.0e-5, 1.0e-5, 1.0, &leak_faults, 0.02);
        }
        assert!(leaking.fluid_volume_m3() < healthy.fluid_volume_m3());
    }

    #[test]
    fn never_overfills_or_goes_negative() {
        let mut res = Reservoir::a380_green();
        for _ in 0..1000 {
            res.step(1.0, 0.0, 1.0, &ReservoirFaults::default(), 0.1);
        }
        assert!(res.fluid_volume_m3() <= GALLON_M3 * 12.0 + 1e-9);
        for _ in 0..1000 {
            res.step(0.0, 1.0, 1.0, &ReservoirFaults::default(), 0.1);
        }
        assert!(res.fluid_volume_m3() >= 0.0);
    }

    #[test]
    fn no_nan_at_dt_zero() {
        let mut res = Reservoir::a380_green();
        let out = res.step(0.0, 0.0, 0.0, &ReservoirFaults::default(), 0.0);
        assert!(out.inlet_air_pressure_pa.is_finite());
        assert!(out.fill_fraction.is_finite());
    }
}
