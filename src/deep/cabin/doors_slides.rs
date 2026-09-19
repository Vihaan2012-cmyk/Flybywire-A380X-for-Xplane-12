//! ATA 52: doors and slides. A door seal leak as a pressurisation-interface
//! orifice, slide arm/disarm logic with a real gas-bottle pressure, door
//! not-latched sensors, and a cargo door actuator driven by hydraulic
//! pressure.
//!
//! `doors.rs` (crate root) already models the *animation* side of all
//! twenty MSFS interactive points (open percent, handle interlock at 0.8
//! psi differential); this module is deliberately independent of it (per
//! the deep systems brief's self-containment rule, this directory takes no
//! dependency on crate internals), taking door position/command as plain
//! `f64`/`bool` inputs so it can be wired to `doors.rs`'s own `Door::
//! open_percent` later without either side depending on the other's types.
//!
//! No FlyByWire source exists to port for slides, seals or a physical cargo
//! door actuator (see `deep/cabin/mod.rs`'s doc for the wider search); this
//! is a native addition.
//!
//! **Sourcing:**
//! - A door seal is a real leak path into the pressurisation model: modelled
//!   as a variable-area orifice, `mdot = Cd * A * sqrt(2 * rho * dP)`, the
//!   standard incompressible-orifice relation used throughout this crate's
//!   own physics (e.g. `physics::hydraulics`'s orifices); `Cd = 0.62` is the
//!   standard sharp-edged-orifice discharge coefficient. The fully-failed
//!   leak area (2 cm^2) is `GENERIC`: a large but plausible degraded-seal
//!   gap, not a measured A380 figure.
//! - Evacuation slides are widely documented (FAA/EASA Part 25.809-family
//!   public certification material, general aviation-safety literature) as
//!   gas-inflated (typically compressed nitrogen or CO2) from a bottle
//!   armed to the door so that opening an armed door fires the slide
//!   automatically ("armed" = girt bar attached to the sill), and as unable
//!   to un-deploy once fired — the same "fires and runs to completion"
//!   idea `oxygen.rs`'s chemical generator already models. Bottle charge
//!   pressure (3000 psi) is `GENERIC`, a plausible compressed-gas bottle
//!   charge (the same order as many industrial/aviation gas bottles);
//!   inflation volume and target pressure are also `GENERIC`.
//! - Door not-latched proximity sensors are standard aircraft-door
//!   instrumentation (multiple public MEL/AMM-adjacent descriptions of
//!   "door not locked" indication systems on transport aircraft); this
//!   module's threshold and fault model are `GENERIC`.
//! - A hydraulically actuated cargo door ram is modelled the same way this
//!   crate's own `physics::hydraulics` treats any actuator: velocity
//!   proportional to available pressure past a load pressure, rate-limited,
//!   with a jam fault that caps travel and a hydraulic-loss fault that
//!   removes the driving pressure. Ram area/rate figures are `GENERIC`.

/// Standard sharp-edged-orifice discharge coefficient (any fluid mechanics
/// reference table).
const ORIFICE_CD: f64 = 0.62;
/// GENERIC fully-failed door seal leak area, m^2 (module doc): 2 cm^2.
pub const SEAL_MAX_LEAK_AREA_M2: f64 = 2.0e-4;
/// Cabin air density used for the seal-leak orifice equation, kg/m^3
/// (standard sea-level-equivalent cabin air density at typical cabin
/// pressure/temperature; a simplification, not a live density calculation).
const CABIN_AIR_DENSITY_KG_M3: f64 = 1.05;

/// GENERIC slide bottle charge pressure, Pa (module doc: 3000 psi).
pub const SLIDE_BOTTLE_CHARGE_PA: f64 = 3000.0 * 6894.757;
/// GENERIC slide bottle physical volume, m^3 (a compact high-pressure gas
/// bottle sized to inflate a slide's structure).
const SLIDE_BOTTLE_VOLUME_M3: f64 = 0.02;
/// GENERIC: a slide needs at least this fraction of its charge pressure
/// remaining to inflate to a usable shape.
const SLIDE_MIN_USABLE_FRACTION: f64 = 0.5;
/// GENERIC bottle temperature assumption (cabin-ish; a slide bottle is
/// stowed in the door structure, not exposed to OAT directly).
const SLIDE_BOTTLE_TEMP_K: f64 = 288.0;
/// Standard dry-air/near-nitrogen specific gas constant, J/(kg*K) — nitrogen
/// specifically is 296.8 J/(kg*K) (standard, molar mass 28.0134 g/mol via
/// R/M); used for the slide bottle since nitrogen is the commonly cited
/// slide inflation gas.
const N2_SPECIFIC_GAS_CONSTANT: f64 = 296.8;
/// A slow leak fault empties the bottle over this many seconds at
/// `leak = 1.0`. GENERIC.
const SLIDE_LEAK_FULL_DISCHARGE_S: f64 = 3600.0 * 24.0;
/// The door must be open past this percent for an armed slide to fire, and
/// a "door open while armed" state to be considered a deployment trigger.
/// GENERIC (matches `doors.rs`'s own handle interlock being expressed in
/// the same 0-100 percent convention it already uses).
const DEPLOY_OPEN_PERCENT: f64 = 5.0;
/// Below this open percent, a latch sensor reads "latched". GENERIC.
const LATCH_OPEN_PERCENT: f64 = 0.5;

/// GENERIC cargo door actuator full-travel time at full hydraulic
/// pressure, s: a hydraulic ram's stroke time is set by its swept volume
/// divided by the control valve's flow, but since this module tracks door
/// position only as a 0..100 percent (matching `doors.rs`'s own
/// convention, not a physical stroke length), that ratio is folded directly
/// into one full-stroke time, chosen to match commonly reported
/// large-transport cargo door cycle times of roughly 15-20 s.
const ACTUATOR_FULL_STROKE_S: f64 = 18.0;
/// Nominal hydraulic system pressure, Pa (5000 psi, per this brief's own
/// "Aircraft" convention).
pub const HYDRAULIC_NOMINAL_PA: f64 = 5000.0 * 6894.757;

fn ideal_gas_pressure_pa(mass_kg: f64, volume_m3: f64, temp_k: f64, r_specific: f64) -> f64 {
    if volume_m3 <= 0. || temp_k <= 0. {
        return 0.;
    }
    mass_kg.max(0.) * r_specific * temp_k / volume_m3
}
fn ideal_gas_mass_kg(pressure_pa: f64, volume_m3: f64, temp_k: f64, r_specific: f64) -> f64 {
    if temp_k <= 0. {
        return 0.;
    }
    (pressure_pa.max(0.) * volume_m3 / (r_specific * temp_k)).max(0.)
}

/// Orifice mass flow, kg/s, for a differential pressure `dp_pa` (always
/// non-negative; flow direction is "out" by convention here).
pub fn orifice_flow_kg_s(area_m2: f64, dp_pa: f64, density_kg_m3: f64) -> f64 {
    if area_m2 <= 0.0 || dp_pa <= 0.0 {
        return 0.0;
    }
    ORIFICE_CD * area_m2 * (2.0 * density_kg_m3 * dp_pa).sqrt()
}

/// Faults one door/slide pair carries, each 0.0 (healthy) .. 1.0 (fully
/// failed).
#[derive(Clone, Copy, Debug, Default)]
pub struct DoorSlideFaults {
    /// The door seal: leak area grows toward `SEAL_MAX_LEAK_AREA_M2`.
    pub seal_leak: f64,
    /// The slide bottle's own slow leak.
    pub bottle_leak: f64,
    /// The not-latched proximity sensor: at 1.0 it freezes its last
    /// reading instead of tracking the real door position.
    pub latch_sensor_fault: f64,
    /// The cargo door actuator jammed: caps travel at whatever fraction of
    /// full travel it jammed at (0 = free, 1 = fully seized in place).
    pub actuator_jam: f64,
    /// Hydraulic pressure loss specific to this actuator's circuit
    /// (upstream of the aircraft's own green/yellow system pressure).
    pub hydraulic_loss: f64,
}

#[derive(Clone, Copy, Debug)]
pub struct DoorSlideInputs {
    /// This door's own open percent (0..100), as `doors.rs`'s `Door::
    /// open_percent` already represents it — read, not written, by this
    /// module.
    pub door_open_percent: f64,
    pub cabin_diff_pressure_pa: f64,
    pub slide_armed_commanded: bool,
    /// The green/yellow hydraulic system pressure available to a cargo
    /// door's actuator, Pa (0 if that system is depressurised).
    pub hydraulic_pressure_pa: f64,
    /// Commanded cargo door travel target, 0..100 percent (the ground crew/
    /// cockpit cargo door switch).
    pub cargo_door_target_percent: f64,
}

impl Default for DoorSlideInputs {
    fn default() -> Self {
        Self {
            door_open_percent: 0.0,
            cabin_diff_pressure_pa: 0.0,
            slide_armed_commanded: false,
            hydraulic_pressure_pa: HYDRAULIC_NOMINAL_PA,
            cargo_door_target_percent: 0.0,
        }
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub struct DoorSlideOutputs {
    pub seal_leak_kg_s: f64,
    pub slide_armed: bool,
    pub slide_deployed: bool,
    pub slide_bottle_pressure_pa: f64,
    pub slide_pressure_adequate: bool,
    pub latched_indication: bool,
    pub door_not_latched_disagree: bool,
    pub cargo_door_percent: f64,
    pub cargo_door_jammed: bool,
}

pub struct DoorSlide {
    slide_bottle_kg: f64,
    slide_deployed: bool,
    displayed_latched: bool,
    cargo_door_percent: f64,
}

impl DoorSlide {
    pub fn new() -> Self {
        let full_kg = ideal_gas_mass_kg(SLIDE_BOTTLE_CHARGE_PA, SLIDE_BOTTLE_VOLUME_M3, SLIDE_BOTTLE_TEMP_K, N2_SPECIFIC_GAS_CONSTANT);
        Self { slide_bottle_kg: full_kg, slide_deployed: false, displayed_latched: true, cargo_door_percent: 0.0 }
    }

    /// Ground service: repacks/resets the slide and its bottle.
    pub fn service(&mut self) {
        self.slide_bottle_kg = ideal_gas_mass_kg(SLIDE_BOTTLE_CHARGE_PA, SLIDE_BOTTLE_VOLUME_M3, SLIDE_BOTTLE_TEMP_K, N2_SPECIFIC_GAS_CONSTANT);
        self.slide_deployed = false;
    }

    pub fn step(&mut self, inputs: &DoorSlideInputs, faults: &DoorSlideFaults, dt: f64) -> DoorSlideOutputs {
        let dt = dt.max(0.0);

        // Door seal leak: an orifice from cabin pressure to ambient, area
        // scaling with the fault.
        let leak_area = faults.seal_leak.clamp(0.0, 1.0) * SEAL_MAX_LEAK_AREA_M2;
        let seal_leak_kg_s = orifice_flow_kg_s(leak_area, inputs.cabin_diff_pressure_pa.max(0.0), CABIN_AIR_DENSITY_KG_M3);

        // Slide bottle: a slow leak fault depletes it continuously; the
        // full-charge mass at `SLIDE_LEAK_FULL_DISCHARGE_S` per full fault.
        let full_kg = ideal_gas_mass_kg(SLIDE_BOTTLE_CHARGE_PA, SLIDE_BOTTLE_VOLUME_M3, SLIDE_BOTTLE_TEMP_K, N2_SPECIFIC_GAS_CONSTANT);
        if faults.bottle_leak > 0.0 {
            let leak_rate_kg_s = full_kg / SLIDE_LEAK_FULL_DISCHARGE_S * faults.bottle_leak.clamp(0.0, 1.0);
            self.slide_bottle_kg = (self.slide_bottle_kg - leak_rate_kg_s * dt).max(0.0);
        }
        let bottle_pa = ideal_gas_pressure_pa(self.slide_bottle_kg, SLIDE_BOTTLE_VOLUME_M3, SLIDE_BOTTLE_TEMP_K, N2_SPECIFIC_GAS_CONSTANT);
        let pressure_adequate = bottle_pa >= SLIDE_BOTTLE_CHARGE_PA * SLIDE_MIN_USABLE_FRACTION;

        // Armed + door opened past the threshold fires the slide; once
        // fired it stays deployed (a real inflated slide does not
        // re-stow itself), and firing draws down the bottle toward
        // ambient over the inflation (a real slide bottle empties almost
        // completely on firing — modelled as an immediate near-total draw,
        // since inflation itself is much faster than any tick this system
        // is meaningfully queried at).
        let armed = inputs.slide_armed_commanded;
        if armed && !self.slide_deployed && inputs.door_open_percent > DEPLOY_OPEN_PERCENT {
            self.slide_deployed = true;
            self.slide_bottle_kg *= 0.02; // spent, a small residual remains
        }

        // Not-latched sensor: true position from door_open_percent, with a
        // stuck-reading fault.
        let real_latched = inputs.door_open_percent < LATCH_OPEN_PERCENT;
        if faults.latch_sensor_fault < 0.5 {
            self.displayed_latched = real_latched;
        }
        let disagree = self.displayed_latched != real_latched;

        // Cargo door actuator: hydraulic ram velocity from available
        // pressure past a nominal load pressure, rate-limited by the valve's
        // max flow, jammed fraction caps total travel.
        let available_pa = (inputs.hydraulic_pressure_pa * (1.0 - faults.hydraulic_loss.clamp(0.0, 1.0))).max(0.0);
        let pressure_fraction = (available_pa / HYDRAULIC_NOMINAL_PA).clamp(0.0, 1.0);
        let max_travel_percent = 100.0 * (1.0 - faults.actuator_jam.clamp(0.0, 1.0));
        let target_percent = inputs.cargo_door_target_percent.clamp(0.0, max_travel_percent);
        let rate_percent_s = (100.0 / ACTUATOR_FULL_STROKE_S) * pressure_fraction;
        let step = (rate_percent_s * dt).max(0.0);
        if self.cargo_door_percent < target_percent {
            self.cargo_door_percent = (self.cargo_door_percent + step).min(target_percent).min(max_travel_percent);
        } else if self.cargo_door_percent > target_percent {
            self.cargo_door_percent = (self.cargo_door_percent - step).max(target_percent);
        }
        self.cargo_door_percent = self.cargo_door_percent.clamp(0.0, 100.0);

        DoorSlideOutputs {
            seal_leak_kg_s,
            slide_armed: armed,
            slide_deployed: self.slide_deployed,
            slide_bottle_pressure_pa: bottle_pa,
            slide_pressure_adequate: pressure_adequate,
            latched_indication: self.displayed_latched,
            door_not_latched_disagree: disagree,
            cargo_door_percent: self.cargo_door_percent,
            cargo_door_jammed: faults.actuator_jam > 0.0
                && self.cargo_door_percent >= max_travel_percent - 1e-6
                && inputs.cargo_door_target_percent > max_travel_percent,
        }
    }
}

impl Default for DoorSlide {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn healthy_inputs() -> DoorSlideInputs {
        DoorSlideInputs::default()
    }

    #[test]
    fn a_sealed_door_has_no_leak_and_a_failed_seal_leaks_more_at_higher_differential() {
        let mut d = DoorSlide::new();
        let healthy = d.step(&DoorSlideInputs { cabin_diff_pressure_pa: 50_000.0, ..healthy_inputs() }, &DoorSlideFaults::default(), 1.0);
        assert_eq!(healthy.seal_leak_kg_s, 0.0);

        let mut faulty = DoorSlide::new();
        let low_dp = faulty.step(&DoorSlideInputs { cabin_diff_pressure_pa: 10_000.0, ..healthy_inputs() }, &DoorSlideFaults { seal_leak: 1.0, ..Default::default() }, 1.0);
        let high_dp = faulty.step(&DoorSlideInputs { cabin_diff_pressure_pa: 50_000.0, ..healthy_inputs() }, &DoorSlideFaults { seal_leak: 1.0, ..Default::default() }, 1.0);
        assert!(low_dp.seal_leak_kg_s > 0.0);
        assert!(high_dp.seal_leak_kg_s > low_dp.seal_leak_kg_s, "higher differential should leak faster");
    }

    #[test]
    fn an_armed_slide_deploys_when_the_door_opens_and_stays_deployed() {
        let mut d = DoorSlide::new();
        let mut inputs = healthy_inputs();
        inputs.slide_armed_commanded = true;
        inputs.door_open_percent = 0.0;
        let out = d.step(&inputs, &DoorSlideFaults::default(), 1.0);
        assert!(!out.slide_deployed);

        inputs.door_open_percent = 50.0;
        let out = d.step(&inputs, &DoorSlideFaults::default(), 1.0);
        assert!(out.slide_deployed);

        inputs.door_open_percent = 0.0; // door closes again
        let out = d.step(&inputs, &DoorSlideFaults::default(), 1.0);
        assert!(out.slide_deployed, "a fired slide does not un-deploy");
    }

    #[test]
    fn a_disarmed_door_opening_does_not_deploy_the_slide() {
        let mut d = DoorSlide::new();
        let mut inputs = healthy_inputs();
        inputs.slide_armed_commanded = false;
        inputs.door_open_percent = 80.0;
        let out = d.step(&inputs, &DoorSlideFaults::default(), 1.0);
        assert!(!out.slide_deployed);
    }

    #[test]
    fn a_leaking_bottle_eventually_drops_below_the_usable_pressure_fraction() {
        let mut d = DoorSlide::new();
        let inputs = healthy_inputs();
        let mut out = DoorSlideOutputs::default();
        // Comfortably past the 50% mass point (the usable-pressure
        // threshold), not sitting right on that boundary.
        for _ in 0..(SLIDE_LEAK_FULL_DISCHARGE_S as usize * 7 / 10) {
            out = d.step(&inputs, &DoorSlideFaults { bottle_leak: 1.0, ..Default::default() }, 1.0);
        }
        assert!(!out.slide_pressure_adequate, "{}", out.slide_bottle_pressure_pa);
    }

    #[test]
    fn a_healthy_bottle_holds_its_charge_indefinitely() {
        let mut d = DoorSlide::new();
        let inputs = healthy_inputs();
        let mut out = DoorSlideOutputs::default();
        for _ in 0..3600 {
            out = d.step(&inputs, &DoorSlideFaults::default(), 1.0);
        }
        assert!(out.slide_pressure_adequate);
        assert!((out.slide_bottle_pressure_pa - SLIDE_BOTTLE_CHARGE_PA).abs() / SLIDE_BOTTLE_CHARGE_PA < 1e-6);
    }

    #[test]
    fn a_stuck_latch_sensor_disagrees_with_a_door_that_has_actually_opened() {
        let mut d = DoorSlide::new();
        let mut inputs = healthy_inputs();
        let faults = DoorSlideFaults { latch_sensor_fault: 1.0, ..Default::default() };
        d.step(&inputs, &faults, 1.0); // latches while closed: reads latched
        inputs.door_open_percent = 20.0;
        let out = d.step(&inputs, &faults, 1.0);
        assert!(out.latched_indication, "sensor stuck reading latched");
        assert!(out.door_not_latched_disagree, "but the door is really open");
    }

    #[test]
    fn a_healthy_cargo_door_actuator_reaches_a_commanded_target() {
        let mut d = DoorSlide::new();
        let mut inputs = healthy_inputs();
        inputs.cargo_door_target_percent = 100.0;
        let mut out = DoorSlideOutputs::default();
        for _ in 0..60 {
            out = d.step(&inputs, &DoorSlideFaults::default(), 1.0);
        }
        assert!((out.cargo_door_percent - 100.0).abs() < 1.0, "{}", out.cargo_door_percent);
    }

    #[test]
    fn a_jammed_actuator_cannot_reach_full_travel_and_a_lost_hydraulic_circuit_cannot_move_at_all() {
        let mut jammed = DoorSlide::new();
        let mut inputs = healthy_inputs();
        inputs.cargo_door_target_percent = 100.0;
        let mut jam_out = DoorSlideOutputs::default();
        for _ in 0..120 {
            jam_out = jammed.step(&inputs, &DoorSlideFaults { actuator_jam: 0.5, ..Default::default() }, 1.0);
        }
        assert!(jam_out.cargo_door_percent < 60.0, "{}", jam_out.cargo_door_percent);
        assert!(jam_out.cargo_door_jammed);

        let mut no_hyd = DoorSlide::new();
        let out = no_hyd.step(&inputs, &DoorSlideFaults { hydraulic_loss: 1.0, ..Default::default() }, 10.0);
        assert_eq!(out.cargo_door_percent, 0.0);
    }

    #[test]
    fn service_resets_the_slide_and_recharges_the_bottle() {
        let mut d = DoorSlide::new();
        d.slide_deployed = true;
        d.slide_bottle_kg *= 0.02;
        d.service();
        assert!(!d.slide_deployed);
        let out = d.step(&healthy_inputs(), &DoorSlideFaults::default(), 0.001);
        assert!(out.slide_pressure_adequate);
    }

    #[test]
    fn no_nan_at_rest_or_dt_zero() {
        let mut d = DoorSlide::new();
        let out = d.step(&healthy_inputs(), &DoorSlideFaults::default(), 0.0);
        assert!(!out.seal_leak_kg_s.is_nan());
        assert!(!out.slide_bottle_pressure_pa.is_nan());
        assert!(!out.cargo_door_percent.is_nan());
    }
}
