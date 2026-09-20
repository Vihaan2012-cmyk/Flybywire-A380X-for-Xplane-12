//! Generic airframe thermal network engine.
//!
//! Each [`Zone`] is a lumped **air node** (the free air volume) plus a
//! lumped **structure node** (the surrounding skin/frame/equipment mass),
//! coupled by an internal convective term -- the standard two-capacitance
//! simplification for a ventilated compartment (air responds fast, the
//! surrounding metal/composite mass responds slowly), used the same way
//! `physics::engine::hot_section.rs`'s multi-node model separates fast and
//! slow thermal capacities (read as this push's style precedent).
//! Energy balance per node: `d(T)/dt = sum(Q) / thermal_mass`, summed in
//! watts, exactly `physics::bays.rs`'s own convention, extended here from
//! one node per bay to two (air + structure) and from five fixed bays to
//! an arbitrary zone/link graph.
//!
//! Zones exchange heat three ways:
//! - **Conduction** ([`ConductionLink`]): structure-to-structure, `Q = UA
//!   * dT`, for physically adjacent airframe structure (a shared
//!   bulkhead/floor/fairing) -- the same conductive-link idea
//!   `physics::bays.rs`'s `WING_ROOT_TO_MAIN_AVIONICS_LINK_W_PER_K` already
//!   uses for one pair, generalised to any pair here.
//! - **Ventilation** ([`VentilationLink`]): a mass flow of air pulling one
//!   zone's air node toward a *reference* condition (another zone's air,
//!   or the outside), `Q = m_dot * cp * (T_ref - T_own)`. This is
//!   algebraically the same fan term `physics::bays.rs::step_bay` already
//!   uses (`fan_w = fan_eff * flow * cp * (temp - supply)`, i.e. `Q = -fan_w
//!   = flow * cp * (supply - temp)`), generalised so the "supply" can be
//!   another *simulated* zone (e.g. the cabin) rather than a hardcoded
//!   constant. It is a one-way, boundary-style exchange: the reference
//!   zone is not itself debited for the flow it supplies (matching
//!   `bays.rs`'s own precedent, where the cabin/outside reference is never
//!   cooled by feeding a bay) -- physically justified where the reference
//!   is a much larger, independently-conditioned reservoir (the pressurised
//!   cabin, or the outside atmosphere) than the zone it feeds.
//! - **Outside air**: every zone's *structure* node also exchanges with
//!   the outside atmosphere directly, `Q = h*A*(T_recovery - T_structure)`,
//!   using the same TAS-driven forced-convection correlation
//!   `physics::bays.rs::external_h_w_m2k`/`physics::fuel.rs`'s tank-skin
//!   convection already use (`h = 10 + 5*sqrt(TAS)`, reproduced here as an
//!   independent copy per this push's "self-contained module" rule), and
//!   the standard compressible recovery-temperature relation
//!   (`physics::fluids.rs::recovery_temperature_k`'s formula, also
//!   reproduced independently here) so ram heating and altitude both come
//!   through the one outside-air input.
//!
//! Solar load lands on the structure node too (`Q = flux * A *
//! exposure_fraction * absorptivity`), and smoke is advected by the exact
//! same ventilation flows as heat (see [`super::smoke`]).
//!
//! **Self-contained by design** (this push's hard rule 2): nothing here
//! imports from `crate::physics`, `crate::Vars` or any other workstream's
//! code, even where a function is conceptually identical to one that
//! already exists there (cited above) -- this module must compile and be
//! testable on its own regardless of what else is mid-edit elsewhere in
//! the crate, and integration (wiring a real system's fault into
//! `inject_heat_w`/`set_ventilation_health`, or reading a real X-Plane
//! ambient dataref into [`OutsideAir`]) is left to whoever couples this
//! network into the rest of the aircraft.
//!
//! No fixed `...Faults` struct: this is a generic, arbitrary-size network,
//! so the fault convention (`0.0 = healthy .. 1.0 = fully failed`) is
//! applied per-link/per-zone instead, through the same public interface
//! any other system uses in normal operation: [`VentilationLink::health`]
//! (1.0 = fully open/healthy fan or door, 0.0 = fully blocked/failed --
//! also legitimately driven by *normal* operational state, e.g. a landing
//! gear system setting a gear-bay door link's health to the door's own
//! open fraction), [`ThermalNetwork::inject_heat_w`] (any system dumping
//! watts into a zone: a duct leak, a fire, brake heat, an engine) and
//! [`ThermalNetwork::inject_smoke_kg_s`] (any system producing smoke).

use super::smoke;

/// Index of a [`Zone`] within a [`ThermalNetwork`]'s `zones` vector.
pub type ZoneId = usize;

/// Dry air specific heat at constant pressure, J/(kg*K). Standard textbook
/// value, sea-level/moderate-temperature range (same cited figure
/// `physics::bays.rs::CP_AIR_J_PER_KG_K` uses; reproduced independently
/// here per this module's self-contained-module rule).
pub const CP_AIR_J_PER_KG_K: f64 = 1005.0;

/// ICAO standard atmosphere sea-level air density, kg/m^3 (ICAO Doc 7488).
/// Used only to convert a zone's air *volume* into an air *mass* at
/// construction time; not re-derived per tick (a zone's air mass is fixed
/// once built, a standard simplification for a lumped-capacitance node).
pub const SEA_LEVEL_AIR_DENSITY_KG_M3: f64 = 1.225;

/// Ratio of specific heats for air (diatomic ideal gas), standard.
const GAMMA_AIR: f64 = 1.4;

/// Recovery factor for a turbulent boundary layer (standard aerodynamic
/// heating reference value, matching `physics::fluids.rs::
/// recovery_temperature_k`'s own doc comment).
const RECOVERY_FACTOR_TURBULENT: f64 = 0.9;

/// Typical solar absorptivity of painted aircraft aluminium skin.
/// **GENERIC**: commonly cited range for light/white aircraft paint is
/// about 0.2-0.5 (aerospace thermal-control literature); no A380-specific
/// figure is public. A mid-range value is used uniformly for every zone's
/// exterior skin.
const SOLAR_ABSORPTIVITY_TYPICAL: f64 = 0.3;

/// Fraction of bare-metal exterior convective heat transfer an intact
/// insulation blanket removes. **GENERIC**: aircraft thermal/acoustic
/// blankets are commonly described (MRO/insulation-industry literature)
/// as cutting exterior heat transfer by roughly two-thirds to three-
/// quarters versus bare skin; no A380-specific figure is public. A
/// zone's effective exterior UA is `bare_ua * (1 - insulation_effectiveness
/// * INSULATION_ATTENUATION)`, so a fully intact blanket
/// (`insulation_effectiveness = 1.0`) cuts exterior exchange by this
/// fraction, and a fully damaged/missing one (`0.0`) exposes bare metal.
const INSULATION_ATTENUATION: f64 = 0.7;

/// Absolute zero, deg C. The one hard physical floor every temperature in
/// this module is clamped to (mirrors `crate::invariants::Bound::
/// TemperatureFloor(-273.15)`'s convention, reproduced locally since this
/// module does not depend on crate internals).
const ABSOLUTE_ZERO_C: f64 = -273.15;

/// Never let a temperature update go non-finite or below absolute zero.
fn floor_temp_c(t: f64) -> f64 {
    if !t.is_finite() {
        return ABSOLUTE_ZERO_C;
    }
    t.max(ABSOLUTE_ZERO_C)
}

/// External forced-convection coefficient, W/(m^2*K): the same empirical
/// form `physics::bays.rs::external_h_w_m2k`/`physics::fuel.rs`'s tank
/// skin convection already use, reproduced independently here.
fn external_h_w_m2k(true_airspeed_m_s: f64) -> f64 {
    10.0 + 5.0 * true_airspeed_m_s.abs().sqrt()
}

/// Standard compressible-flow adiabatic-wall recovery temperature,
/// `T_recovery = T_static * (1 + r*(gamma-1)/2*M^2)` -- the same relation
/// `physics::fluids.rs::recovery_temperature_k` implements, reproduced
/// independently here (deg C in, deg C out; the formula itself works in
/// Kelvin internally since it is a ratio relation, but the +273.15 offset
/// must round-trip for a non-zero Mach to matter, so it is applied here
/// explicitly rather than skipped).
fn recovery_temperature_c(static_temp_c: f64, mach: f64, recovery_factor: f64) -> f64 {
    let static_k = static_temp_c + 273.15;
    let recovered_k = static_k * (1.0 + recovery_factor * (GAMMA_AIR - 1.0) / 2.0 * mach * mach);
    recovered_k - 273.15
}

/// ICAO International Standard Atmosphere static air temperature at
/// `altitude_m` geometric altitude (ICAO Doc 7488, standard atmosphere):
/// 15 C at sea level, -6.5 K/km lapse rate through the troposphere (up to
/// 11,000 m), isothermal -56.5 C through the lower stratosphere
/// (11,000-20,000 m) -- covers the full range of transport-category cruise
/// altitudes. A convenience for building [`OutsideAir`] from an altitude
/// input; callers with a real ambient-temperature sensor/dataref should
/// use that instead.
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

/// The flight condition every zone's exterior exchanges heat with.
#[derive(Clone, Copy, Debug)]
pub struct OutsideAir {
    /// Static (true) outside air temperature, deg C (e.g. from
    /// [`isa_static_temp_c`] or a real ambient-temperature dataref).
    pub static_temp_c: f64,
    pub mach: f64,
    pub true_airspeed_m_s: f64,
}
impl OutsideAir {
    /// Effective outside-air temperature a stagnating/turbulent-boundary
    /// surface actually sees, folding in ram/aerodynamic heating.
    pub fn recovery_temp_c(&self) -> f64 {
        recovery_temperature_c(self.static_temp_c, self.mach, RECOVERY_FACTOR_TURBULENT)
    }
}

/// One lumped zone: an air node and a structure node, each with their own
/// thermal mass, coupled by [`Zone::air_structure_ua_w_per_k`].
pub struct Zone {
    pub name: &'static str,
    pub air_temp_c: f64,
    pub structure_temp_c: f64,
    /// Air mass in the zone's free volume, kg (fixed at construction).
    pub air_mass_kg: f64,
    /// Combined structure/equipment/furnishings thermal mass, J/K.
    pub structure_thermal_mass_j_per_k: f64,
    /// Internal air<->structure convective coupling, W/K.
    pub air_structure_ua_w_per_k: f64,
    /// Exterior skin/fairing area exposed to outside air, m^2 (0 for a
    /// fully interior zone with no exterior boundary of its own).
    pub exterior_skin_area_m2: f64,
    /// Fraction (0..1) of `exterior_skin_area_m2` that receives direct
    /// solar load (top/forward-facing skin vs. shaded/underside skin).
    pub sun_exposure_fraction: f64,
    /// Condition of the thermal/acoustic insulation blanket between the
    /// structure node and the outside skin: `1.0` = intact blanket
    /// (attenuates exterior convective exchange, [`INSULATION_ATTENUATION`]),
    /// `0.0` = blanket missing/damaged (bare-metal exterior heat transfer).
    /// Aircraft thermal/acoustic insulation blankets are a real, damageable
    /// part (moisture-soaked or torn blankets are a known MRO finding);
    /// this is the model field `FailureDef`s for insulation damage act on.
    pub insulation_effectiveness: f64,
    /// Constant baseline heat dissipation (equipment, lighting, standing
    /// losses), W. Continuous fault/system inputs use `inject_heat_w`
    /// instead of changing this.
    pub baseline_heat_w: f64,
    /// Per-tick injected heat, W: accumulated by any number of
    /// `ThermalNetwork::inject_heat_w` calls, consumed and reset to 0 by
    /// the next `step`.
    injected_heat_w: f64,
    /// Smoke currently held in this zone's (well-mixed) air, kg.
    pub smoke_kg: f64,
    /// Per-tick injected smoke production rate, kg/s: same accumulate/
    /// consume/reset convention as `injected_heat_w`.
    injected_smoke_kg_s: f64,
    /// Settling/filtration loss rate, fraction of standing smoke mass per
    /// second. **GENERIC**, 0.0 by default (no settling modelled unless a
    /// caller sets one).
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

/// Structure-to-structure conductive link between two adjacent zones.
pub struct ConductionLink {
    pub a: ZoneId,
    pub b: ZoneId,
    pub ua_w_per_k: f64,
}

/// The condition a [`VentilationLink`] pulls a zone's air toward.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ZoneRef {
    Zone(ZoneId),
    OutsideAir,
}

/// A mass-flow ventilation path pulling `vented_zone`'s air toward
/// `reference` (another zone's air, or the outside).
pub struct VentilationLink {
    pub vented_zone: ZoneId,
    pub reference: ZoneRef,
    /// Flow at full health/fully open, kg/s.
    pub nameplate_flow_kg_s: f64,
    /// `0.0 = fully blocked/failed/closed .. 1.0 = fully
    /// open/healthy`. See the module doc's "no fixed Faults struct" note.
    pub health: f64,
}
impl VentilationLink {
    pub fn flow_kg_s(&self) -> f64 {
        self.nameplate_flow_kg_s.max(0.0) * self.health.clamp(0.0, 1.0)
    }
}

/// The airframe thermal network: an arbitrary set of zones connected by
/// conduction and ventilation links, stepped once per tick.
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

    /// Adds a ventilation link and returns its index (for later
    /// `set_ventilation_health` calls -- e.g. a fan-failure or gear-door
    /// fault driving that one path).
    pub fn add_ventilation_link(&mut self, vented_zone: ZoneId, reference: ZoneRef, nameplate_flow_kg_s: f64) -> usize {
        self.ventilation_links.push(VentilationLink { vented_zone, reference, nameplate_flow_kg_s, health: 1.0 });
        self.ventilation_links.len() - 1
    }

    pub fn set_ventilation_health(&mut self, link_index: usize, health: f64) {
        if let Some(l) = self.ventilation_links.get_mut(link_index) {
            l.health = health.clamp(0.0, 1.0);
        }
    }

    /// Heat-source interface (task step 2): any system injects watts into
    /// a zone. Accumulates across multiple calls in the same tick;
    /// consumed and reset by the next `step`.
    pub fn inject_heat_w(&mut self, zone: ZoneId, watts: f64) {
        if let Some(z) = self.zones.get_mut(zone) {
            z.injected_heat_w += watts;
        }
    }

    /// Smoke-source interface: any system injects a smoke production
    /// rate (kg/s) into a zone for this tick.
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

    /// Advances the whole network by `dt_s` real seconds, sub-stepping
    /// internally if any zone's fastest local time constant is stiff
    /// relative to `dt_s` (see `stable_substep_count`).
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

    /// Explicit Euler is only stable while `dt <= ~2 / rate` for each
    /// node's own fastest loss rate (`UA / thermal_mass`). Rather than
    /// track a live worst case, this bounds every UA term that could act
    /// on a node (its own air<->structure link, every ventilation flow
    /// touching it, every conduction link and a conservative upper-bound
    /// exterior convection coefficient) and picks a comfortably small
    /// (`0.2x`) fraction of the resulting fastest time constant as the
    /// per-substep size, matching this push's "sub-stepping where stiff"
    /// convention (see `physics::hydraulics`-style stiff ODE handling
    /// this crate already uses elsewhere for the same reason).
    fn stable_substep_count(&self, dt_s: f64) -> u32 {
        const CONSERVATIVE_MAX_EXTERNAL_H_W_M2K: f64 = 200.0; // upper bound near Mach 1 ram convection
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

    // -- 1. Conduction chain reaches a hand-solved steady state. --------

    #[test]
    fn conduction_chain_reaches_hand_solved_steady_state() {
        let mut net = ThermalNetwork::new();
        // High internal air<->structure UA so each zone's air and
        // structure nodes track each other closely (the lumped-node
        // approximation the hand solve below assumes).
        let z0 = net.add_zone(Zone::new("Source", 1.0, 1.0e5, 5000.0, 0.0, 0.0, 1000.0, 15.0));
        let z1 = net.add_zone(Zone::new("Sink", 1.0, 1.0e5, 5000.0, 5.0, 0.0, 0.0, 15.0));
        net.add_conduction_link(z0, z1, 20.0);

        let outside = calm_ground_air(15.0);
        // The slowest eigenvalue of this two-structure-node chain is
        // 6.1e-5 /s (tau = 16 000 s: C = 1e5 J/K per node against a 20 W/K
        // link and a 15 W/K exit), so 40 000 s left it 8% short of the
        // steady state it is asymptoting to. 150 000 s is ~9 tau.
        for _ in 0..150_000 {
            net.step(1.0, &outside, 0.0);
        }

        // Hand solve: at steady state all of z0's baseline heat must flow
        // out through the conduction link, then all the way out through
        // z1's exterior convection:
        //   ua_link*(T0-T1) = ext_ua*(T1-Tamb) = Q0
        // `ext_ua` is NOT h*A: every zone is built with an intact
        // insulation blanket (`insulation_effectiveness` = 1.0), and
        // `substep` applies `1 - insulation_effectiveness *
        // INSULATION_ATTENUATION` = 0.3 to the exterior convective term,
        // so the blanketed exit conductance is h*A*0.3 = 10*5*0.3 = 15 W/K
        // and the chain has to sit far hotter than bare metal would to
        // push the same 1000 W out.
        let ext_ua = 10.0 * 5.0 * (1.0 - INSULATION_ATTENUATION);
        let t1_predicted = 15.0 + 1000.0 / ext_ua; // 81.67 C
        let t0_predicted = t1_predicted + 1000.0 / 20.0; // 131.67 C
        // Both assertions read the *air* nodes. z1's air carries no source
        // of its own, so at steady state it sits exactly on its structure.
        // z0's air carries the 1000 W and has to be 1000/5000 = 0.2 K above
        // its own structure to push it across, comfortably inside the 1 K
        // tolerance below.

        assert!((net.air_temp_c(z0) - t0_predicted).abs() < 1.0, "z0 {} vs predicted {}", net.air_temp_c(z0), t0_predicted);
        assert!((net.air_temp_c(z1) - t1_predicted).abs() < 1.0, "z1 {} vs predicted {}", net.air_temp_c(z1), t1_predicted);
        assert!(net.air_temp_c(z0) > net.air_temp_c(z1), "heat must flow from the source zone to the sink zone");
    }

    // -- 2. Smoke: injection + ventilation to outside reaches a hand-solved
    //    steady-state concentration. -------------------------------------

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

        // Steady state: production = flow * concentration (outside is
        // clean, no decay) => concentration = produced / flow.
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

    // -- 3. Sun load raises a zone's steady-state structure temperature. -

    #[test]
    fn sun_exposure_raises_steady_state_structure_temperature() {
        let outside = calm_ground_air(15.0);
        const FLUX_W_M2: f64 = 800.0;

        let mut shaded = ThermalNetwork::new();
        let z_shaded = shaded.add_zone(Zone::new("Shaded", 5.0, 2.0e5, 100.0, 10.0, 0.0, 0.0, 15.0));
        let mut sunny = ThermalNetwork::new();
        let z_sunny = sunny.add_zone(Zone::new("Sunny", 5.0, 2.0e5, 100.0, 10.0, 0.5, 0.0, 15.0));

        // tau = C/ext_ua = 2e5/30 = 6670 s, so 20 000 s was only 3 tau and
        // still 5% short; 60 000 s is 9 tau.
        for _ in 0..60_000 {
            shaded.step(1.0, &outside, FLUX_W_M2);
            sunny.step(1.0, &outside, FLUX_W_M2);
        }

        assert!((shaded.structure_temp_c(z_shaded) - 15.0).abs() < 0.5, "no sun exposure: should stay near ambient, got {}", shaded.structure_temp_c(z_shaded));
        // Hand solve: absorbed solar power in = blanketed convection out.
        //   Q_sun = flux * A * sun_fraction * absorptivity
        //         = 800 * 10 * 0.5 * 0.3 = 1200 W
        //     (the old expression here dropped the 10 m^2 area from the
        //      absorbed power and then divided by 10 instead of by the
        //      exit conductance, which happened to be within 6x)
        //   ext_ua = h * A * (1 - insulation_effectiveness * ATTENUATION)
        //          = 10 * 10 * 0.3 = 30 W/K
        //     (solar lands on the skin, so the blanket behind it does not
        //      attenuate the absorbed flux, only the convective exchange)
        //   T = 15 + 1200/30 = 55 C
        let q_sun_w = FLUX_W_M2 * 10.0 * 0.5 * SOLAR_ABSORPTIVITY_TYPICAL;
        let ext_ua = 10.0 * 10.0 * (1.0 - INSULATION_ATTENUATION);
        let predicted_sunny = 15.0 + q_sun_w / ext_ua; // 55 C
        assert!((sunny.structure_temp_c(z_sunny) - predicted_sunny).abs() < 1.0, "sunny {} vs predicted {}", sunny.structure_temp_c(z_sunny), predicted_sunny);
        assert!(sunny.structure_temp_c(z_sunny) > shaded.structure_temp_c(z_shaded));
    }

    // -- 4. Ventilation health fault: blocking a zone's vent raises its
    //    steady-state temperature (the generic "coupling fault" case). --

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

    // -- 5. Numerically safe at rest / dt = 0. ---------------------------

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

    // -- 6. Sub-stepping keeps a stiff configuration stable at a large dt.

    #[test]
    fn substepping_keeps_a_stiff_zone_stable_at_a_large_timestep() {
        let mut net = ThermalNetwork::new();
        // A 50 J/K structure node against 200 W/K of coupling: tau = 0.25 s
        // against a single 100 s call, i.e. 400x stiff. A plain one-shot
        // Euler step would multiply the structure node's error by
        // (1 - 200/50*100) = -399 per step; only sub-stepping can land it
        // on the steady state below.
        //
        // The zone's 500 W of equipment dissipation lands on the *air*
        // node (that is what `baseline_heat_w` is: see `Zone`'s own doc and
        // `substep`, which adds it to `q_air`), so the heat has to reach
        // the outside through the air<->structure coupling and then the
        // skin. The previous version of this test set
        // `air_structure_ua_w_per_k` to 0.0, which left the air node with
        // no loss path at all (it just ramped) and the structure node
        // decoupled from the heat entirely, so its "predicted" 65 C could
        // never have been reached by any amount of sub-stepping. It also
        // needs a defined sink for the air node, so the zone is given a
        // ventilation path to outside as well.
        //
        // Hand solve, with theta = T - 15, U = air<->structure = 100 W/K,
        // E = structure<->outside = h*A*(1 - insulation*ATTENUATION), and
        // V = vent flow * cp:
        //   structure:  U*(theta_a - theta_s) = E*theta_s
        //   air:        500 = V*theta_a + U*(theta_a - theta_s)
        // With the blanket removed (bare skin) E = 10*10*1.0 = 100 W/K, so
        // theta_s = theta_a/2 and 500 = V*theta_a + 50*theta_a; choosing
        // V = 50 W/K gives theta_a = 5 and theta_s = 2.5, i.e. air 20 C and
        // structure 17.5 C.
        const U_W_PER_K: f64 = 100.0;
        const VENT_UA_W_PER_K: f64 = 50.0;
        let z0 = net.add_zone(Zone::new("Stiff", 0.01, 50.0, U_W_PER_K, 10.0, 0.0, 500.0, 15.0));
        net.zones[z0].insulation_effectiveness = 0.0; // bare skin: E = h*A
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

    // -- 7. Insulation damage: a damaged blanket lets a cold zone chill
    //    faster toward outside air than an intact one (a failure changing
    //    the outcome). --

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
        // Both must still be well above absolute zero and finite -- no
        // runaway from removing the attenuation factor.
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
