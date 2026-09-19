//! Hail: persistent, cumulative damage to every airframe/engine part hail
//! can reach, each with a physical consequence a consuming model reads
//! (drag, weather-radar attenuation, window-heat/visibility/leak, slat
//! jam risk, engine fan/compressor/flameout risk, probe/antenna
//! damage) -- plus a manual trigger and an optional random mode tied to a
//! convective-weather input. Unlike a bird strike (one discrete, usually
//! survivable, impact per target), a hail encounter is many stones over
//! seconds to minutes, so damage here is a running total per component
//! that never resets on its own, matching how a real aircraft comes out
//! of a hailstorm: dented and requiring an inspection, not instantly
//! healed once clear of the cell.
//!
//! ## Sources
//! - Hailstone size categories: the TORRO Hailstorm Intensity Scale (Webb,
//!   Elsom & Meaden, public), H0 5-9 mm through H10 >100 mm; the U.S.
//!   National Weather Service's severe-thunderstorm criterion, 25 mm
//!   (1 inch) diameter, is the boundary this module treats as "large hail".
//! - Radome hail resistance: a public radome-industry whitepaper figure
//!   (radome.at) states an A-grade radome withstands hailstones up to
//!   15 mm (TORRO H1) with no/negligible damage, a C-grade radome up to
//!   20 mm (H2); the 20 mm (H2) figure anchors the radome's damage scale.
//! - Hailstone terminal fall velocity: `v_t = 9 * D_cm^0.8` m/s, a
//!   commonly used meteorological approximation (order-of-magnitude form
//!   seen across hail-physics literature, e.g. Rasmussen & Heymsfield-
//!   style fits); also inverted below (`max_sustainable_diameter_mm`) for
//!   `weather_cells.rs`'s hail-growth estimate from a cell's updraft.
//! - Hailstone density ~900 kg/m^3: public value for graupel/hail ice
//!   (slightly below solid ice's 917 kg/m^3 owing to included air).
//! - Engine hail/rain ingestion is a certification requirement (14 CFR
//!   33.68 / EASA CS-E 790, "Rain and hail ingestion", public): engines
//!   must run continuously through a specified hail concentration without
//!   flameout, and the certification concern is explicitly worse at low
//!   power (idle/approach), which is why `flameout_risk_frac` below scales
//!   inversely with `n1_frac`.
//! - Every reference diameter/area/threshold beyond the two cited above,
//!   the accumulation-to-full-damage hit count, the window-heat/leak/
//!   CLmax/slat-jam/erosion/flameout curves and the random-mode size
//!   distribution are `GENERIC`, derived as documented at each constant:
//!   no size- or part-specific public test report exists for these.

use super::rng::Rng;

/// TORRO Hailstorm Intensity Scale category for a diameter, mm.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TorroClass {
    H0,
    H1,
    H2,
    H3,
    H4,
    H5,
    H6,
    H7,
    H8,
    H9,
    H10,
}

pub fn torro_class(diameter_mm: f64) -> TorroClass {
    let d = diameter_mm;
    if d < 10.0 {
        TorroClass::H0
    } else if d < 16.0 {
        TorroClass::H1
    } else if d < 21.0 {
        TorroClass::H2
    } else if d < 31.0 {
        TorroClass::H3
    } else if d < 41.0 {
        TorroClass::H4
    } else if d < 51.0 {
        TorroClass::H5
    } else if d < 61.0 {
        TorroClass::H6
    } else if d < 76.0 {
        TorroClass::H7
    } else if d < 91.0 {
        TorroClass::H8
    } else if d < 101.0 {
        TorroClass::H9
    } else {
        TorroClass::H10
    }
}

/// Ice/graupel density, kg/m^3 (public, slightly below solid ice's 917
/// kg/m^3 owing to included air).
const HAIL_DENSITY_KG_M3: f64 = 900.0;

fn hail_mass_kg(diameter_mm: f64) -> f64 {
    let r_m = diameter_mm.max(0.0) / 2000.0;
    (4.0 / 3.0) * std::f64::consts::PI * r_m * r_m * r_m * HAIL_DENSITY_KG_M3
}

/// GENERIC meteorological approximation for hail terminal fall speed, m/s
/// (see module doc).
fn terminal_velocity_ms(diameter_mm: f64) -> f64 {
    let d_cm = (diameter_mm.max(0.0) / 10.0).max(1e-6);
    9.0 * d_cm.powf(0.8)
}

/// Inverts `terminal_velocity_ms`: the largest hailstone diameter, mm, an
/// updraft of this speed can keep suspended long enough to keep growing
/// (a hailstone falls out -- and stops growing -- once its terminal
/// velocity exceeds the updraft carrying it, the standard qualitative
/// hail-growth picture in convective storm meteorology). Used by
/// `weather_cells.rs`.
pub(super) fn max_sustainable_diameter_mm(updraft_ms: f64) -> f64 {
    let v = updraft_ms.max(0.0);
    (v / 9.0).powf(1.0 / 0.8) * 10.0
}

/// A location hail can hit. Indices are 0-based: `Windshield(0..6)`,
/// `WingLeadingEdge(0..12)` (six chordwise segments per side, 0-5 left
/// root-to-tip then 6-11 right), `EngineInlet`/`Nacelle`(0..4), `Probe`
/// (0..6: pitot/static/AoA/TAT probes and small external antennas).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ImpactTarget {
    Radome,
    Windshield(u8),
    WingLeadingEdge(u8),
    EngineInlet(u8),
    Nacelle(u8),
    Probe(u8),
}

const N_WINDSHIELD: usize = 6;
const N_WING_LE: usize = 12;
const N_ENGINE: usize = 4;
const N_PROBE: usize = 6;
const N_NACELLE: usize = 4;

/// GENERIC frontal/exposed area, m^2, each target's energy spreads over.
fn area_m2(target: ImpactTarget) -> f64 {
    match target {
        ImpactTarget::Radome => 2.2,
        ImpactTarget::Windshield(_) => 0.45,
        ImpactTarget::WingLeadingEdge(_) => 0.35,
        // Fan face area, pi/4 * (2.95 m)^2 -- the Trent 972B-84's public
        // fan diameter, EASA TCDS E.012.
        ImpactTarget::EngineInlet(_) => std::f64::consts::PI / 4.0 * 2.95 * 2.95,
        ImpactTarget::Nacelle(_) => 1.5,
        ImpactTarget::Probe(_) => 0.01,
    }
}

/// Reference diameter, mm, this target's damage scale is anchored to (see
/// module doc for the two cited radome figures; the rest are `GENERIC`,
/// set relative to the radome by material toughness).
fn reference_diameter_mm(target: ImpactTarget) -> f64 {
    match target {
        ImpactTarget::Radome => 20.0,
        ImpactTarget::Windshield(_) | ImpactTarget::WingLeadingEdge(_) | ImpactTarget::EngineInlet(_) | ImpactTarget::Nacelle(_) => 30.0,
        // GENERIC: small probes/antennas are fragile next to any of the
        // above.
        ImpactTarget::Probe(_) => 15.0,
    }
}

/// Reference impact speed the certification-style diameters above are
/// generally quoted at: representative high-speed flight, matching this
/// crate's own `docs/physics/failures.md` VMO citation (390 kt, from
/// FlyByWire's public `flight_model.cfg`, matching the public A380 type's
/// certified VMO).
const REFERENCE_SPEED_MS: f64 = 390.0 * 0.514_444;

/// GENERIC: how many reference-severity direct hits a sustained encounter
/// needs before a target reaches "fully damaged" -- hail damage is
/// cumulative (many stones), not typically one catastrophic hit, while
/// still anchoring the overall scale to the same certification-style
/// reference diameter used for a single impact.
const HITS_FOR_FULL_DAMAGE: f64 = 15.0;

fn threshold_j_m2(target: ImpactTarget) -> f64 {
    let e_ref = 0.5 * hail_mass_kg(reference_diameter_mm(target)) * REFERENCE_SPEED_MS * REFERENCE_SPEED_MS;
    (e_ref / area_m2(target)) * HITS_FOR_FULL_DAMAGE
}

/// GENERIC: fraction of an engine-inlet hailstone's mass that reaches the
/// core flow path rather than the bypass duct (reusing the Trent 972B-84's
/// public ~8.7:1 bypass ratio as the flow-area split, the same reasoning
/// `bird_strike.rs` uses).
const ENGINE_CORE_INGESTION_FRACTION: f64 = 1.0 / (1.0 + 8.7);
/// GENERIC compressor-erosion scale, efficiency-fraction lost per kg of
/// ice/hail mass reaching the core, and its ceiling (matching
/// `volcanic_ash.rs`'s own erosion ceiling since the mechanism -- solid
/// particles scouring the compressor -- is the same).
const ENGINE_EROSION_PER_KG: f64 = 0.02;
const MAX_ENGINE_EROSION: f64 = 0.25;
/// GENERIC: ingested ice/hail mass, kg, a healthy engine at 100% N1
/// tolerates without a meaningfully elevated flameout risk; CS-E 790's
/// concern is specifically low-power ingestion (module doc), so this
/// tolerance is scaled down by `n1_frac` before use.
const FLAMEOUT_TOLERANCE_KG_AT_FULL_POWER: f64 = 0.05;
/// GENERIC: below this N1 fraction the flameout tolerance is floored
/// (an engine at true zero power has essentially no flame margin left to
/// scale to zero).
const MIN_N1_FOR_TOLERANCE: f64 = 0.15;

/// GENERIC: windshield cumulative severity above which the window-heat
/// system's conductive film (thinner and more fragile than the structural
/// panes themselves) is assumed faulted.
const WINDOW_HEAT_FAULT_SEVERITY: f64 = 0.3;
/// GENERIC: windshield severity above which cracking is assumed to have
/// breached the pane, opening a small leak; and the leak area, m^2, at
/// `severity = 1.0`.
const WINDSHIELD_LEAK_ONSET_SEVERITY: f64 = 0.9;
const WINDSHIELD_LEAK_AREA_AT_FULL_M2: f64 = 0.0005;

/// Persistent, cumulative hail damage -- one instance covers a whole
/// flight (or is loaded from the previous one): unlike a bird strike or a
/// lightning strike, hail damage does not resolve to a single number from
/// a single hit; it is a running energy-density total per part that this
/// state carries forward, including after the storm is behind the
/// aircraft (see module doc).
#[derive(Clone, Copy, Debug, Default)]
pub struct HailDamageState {
    radome_j_m2: f64,
    windshield_j_m2: [f64; N_WINDSHIELD],
    wing_le_j_m2: [f64; N_WING_LE],
    engine_fan_j_m2: [f64; N_ENGINE],
    engine_compressor_eff_loss_frac: [f64; N_ENGINE],
    nacelle_j_m2: [f64; N_NACELLE],
    probe_j_m2: [f64; N_PROBE],
}

/// What one hailstone hit (or `count` identical ones landing on the same
/// target this step) did, including the target's fresh cumulative state.
/// Fields not relevant to `target`'s kind read 0.0/false.
#[derive(Clone, Copy, Debug)]
pub struct HailOutcome {
    pub target: ImpactTarget,
    pub diameter_mm: f64,
    pub class: TorroClass,
    pub impact_speed_ms: f64,
    /// Energy of this call's `count` stones combined, J.
    pub impact_energy_j: f64,
    /// Cumulative severity for this specific target instance, 0..1,
    /// persistent.
    pub damage_frac: f64,
    /// Drag-coefficient increment from accumulated denting (`Radome`,
    /// `WingLeadingEdge`, `Nacelle`).
    pub drag_delta_cd: f64,
    /// Weather-radar beam attenuation/distortion fraction (`Radome` only).
    pub wxr_attenuation_frac: f64,
    /// The window-heat system's conductive film has faulted (`Windshield`
    /// only).
    pub window_heat_fault: bool,
    /// Forward-visibility loss from crazing/pitting (`Windshield` only).
    pub visibility_loss_frac: f64,
    /// Structural/pressurisation leak area, m^2, once cracking breaches
    /// the pane (`Windshield` only).
    pub leak_area_m2: f64,
    /// Max-lift-coefficient penalty from a damaged leading edge/slat
    /// (`WingLeadingEdge` only), negative or zero.
    pub clmax_delta: f64,
    /// Risk this segment's slat extend/retract mechanism has jammed
    /// (`WingLeadingEdge` only).
    pub slat_jam_risk_frac: f64,
    /// Fan blade damage from repeated ice impacts (`EngineInlet` only).
    pub fan_damage_frac: f64,
    /// Compressor efficiency loss from ice/hail ingested into the core
    /// (`EngineInlet` only), persistent.
    pub compressor_efficiency_loss_frac: f64,
    /// Flameout/roll-back risk from this hit's ingested mass, worse at
    /// low power (`EngineInlet` only, see module doc, CS-E 790).
    pub flameout_risk_frac: f64,
}

fn blank_outcome(target: ImpactTarget, diameter_mm: f64, impact_speed_ms: f64, impact_energy_j: f64) -> HailOutcome {
    HailOutcome {
        target,
        diameter_mm,
        class: torro_class(diameter_mm),
        impact_speed_ms,
        impact_energy_j,
        damage_frac: 0.0,
        drag_delta_cd: 0.0,
        wxr_attenuation_frac: 0.0,
        window_heat_fault: false,
        visibility_loss_frac: 0.0,
        leak_area_m2: 0.0,
        clmax_delta: 0.0,
        slat_jam_risk_frac: 0.0,
        fan_damage_frac: 0.0,
        compressor_efficiency_loss_frac: 0.0,
        flameout_risk_frac: 0.0,
    }
}

impl HailDamageState {
    pub fn new() -> Self {
        Self::default()
    }

    /// Adds `count` identical `diameter_mm` hailstones hitting `target`
    /// this step (at true airspeed `tas_ms`) to that target's persistent
    /// damage, and returns the fresh cumulative outcome. `n1_frac` (0..1+)
    /// only matters for `EngineInlet`.
    pub fn strike(&mut self, target: ImpactTarget, diameter_mm: f64, tas_ms: f64, count: u32, n1_frac: f64) -> HailOutcome {
        let diameter_mm = diameter_mm.max(0.0);
        let n = count.max(1) as f64;
        let single_mass_kg = hail_mass_kg(diameter_mm);
        let v_fall = terminal_velocity_ms(diameter_mm);
        let v = tas_ms.max(0.0);
        let v_rel = (v * v + v_fall * v_fall).sqrt();
        // N separate stone impacts add their energies; they are not one
        // combined-mass impact the way a bird flock is treated.
        let single_energy_j = 0.5 * single_mass_kg * v_rel * v_rel;
        let impact_energy_j = single_energy_j * n;
        let mut out = blank_outcome(target, diameter_mm, v_rel, impact_energy_j);
        let energy_density_j_m2 = impact_energy_j / area_m2(target);

        match target {
            ImpactTarget::Radome => {
                self.radome_j_m2 += energy_density_j_m2;
                let d = (self.radome_j_m2 / threshold_j_m2(target)).min(1.0);
                out.damage_frac = d;
                // GENERIC: a dented/cracked radome scatters and attenuates
                // the radar beam roughly in proportion to its own damage.
                out.wxr_attenuation_frac = d;
                out.drag_delta_cd = 0.02 * d * d;
            }
            ImpactTarget::Windshield(i) => {
                let idx = i as usize % N_WINDSHIELD;
                self.windshield_j_m2[idx] += energy_density_j_m2;
                let d = (self.windshield_j_m2[idx] / threshold_j_m2(target)).min(1.0);
                out.damage_frac = d;
                out.window_heat_fault = d > WINDOW_HEAT_FAULT_SEVERITY;
                out.visibility_loss_frac = d;
                out.leak_area_m2 = if d > WINDSHIELD_LEAK_ONSET_SEVERITY {
                    WINDSHIELD_LEAK_AREA_AT_FULL_M2 * (d - WINDSHIELD_LEAK_ONSET_SEVERITY) / (1.0 - WINDSHIELD_LEAK_ONSET_SEVERITY)
                } else {
                    0.0
                };
            }
            ImpactTarget::WingLeadingEdge(i) => {
                let idx = i as usize % N_WING_LE;
                self.wing_le_j_m2[idx] += energy_density_j_m2;
                let d = (self.wing_le_j_m2[idx] / threshold_j_m2(target)).min(1.0);
                out.damage_frac = d;
                out.drag_delta_cd = 0.02 * d * d;
                // GENERIC: up to a 5% CLmax loss at full damage (leading-
                // edge/slat contamination-equivalence heuristic: a dented
                // slat disrupts boundary-layer control near stall the same
                // qualitative way ice contamination does).
                out.clmax_delta = -0.05 * d;
                // GENERIC: the slat's own drive mechanism (torque tube,
                // track, fittings) only risks jamming once cumulative
                // denting is well past cosmetic.
                out.slat_jam_risk_frac = ((d - 0.5) / 0.5).clamp(0.0, 1.0);
            }
            ImpactTarget::EngineInlet(i) => {
                let idx = i as usize % N_ENGINE;
                self.engine_fan_j_m2[idx] += energy_density_j_m2;
                let fan_d = (self.engine_fan_j_m2[idx] / threshold_j_m2(target)).min(1.0);
                out.fan_damage_frac = fan_d;
                out.damage_frac = fan_d;

                let ingested_mass_kg = single_mass_kg * n * ENGINE_CORE_INGESTION_FRACTION;
                let erosion = ENGINE_EROSION_PER_KG * ingested_mass_kg;
                self.engine_compressor_eff_loss_frac[idx] = (self.engine_compressor_eff_loss_frac[idx] + erosion).min(MAX_ENGINE_EROSION);
                out.compressor_efficiency_loss_frac = self.engine_compressor_eff_loss_frac[idx];

                let effective_n1 = n1_frac.max(MIN_N1_FOR_TOLERANCE);
                let tolerance_kg = FLAMEOUT_TOLERANCE_KG_AT_FULL_POWER * effective_n1;
                let mass_term = (ingested_mass_kg / tolerance_kg.max(1e-9)) * 0.5;
                let fan_term = fan_d * 0.3;
                let erosion_term = (self.engine_compressor_eff_loss_frac[idx] / MAX_ENGINE_EROSION) * 0.2;
                out.flameout_risk_frac = (mass_term + fan_term + erosion_term).min(1.0);
            }
            ImpactTarget::Nacelle(i) => {
                let idx = i as usize % N_NACELLE;
                self.nacelle_j_m2[idx] += energy_density_j_m2;
                let d = (self.nacelle_j_m2[idx] / threshold_j_m2(target)).min(1.0);
                out.damage_frac = d;
                out.drag_delta_cd = 0.015 * d * d;
            }
            ImpactTarget::Probe(i) => {
                let idx = i as usize % N_PROBE;
                self.probe_j_m2[idx] += energy_density_j_m2;
                out.damage_frac = (self.probe_j_m2[idx] / threshold_j_m2(target)).min(1.0);
            }
        }
        out
    }
}

/// GENERIC: mean of an exponential hailstone-diameter distribution inside
/// an encountered hail shaft -- most hail is small (TORRO H0/H1), with a
/// long tail reaching severe (>=25 mm, the NWS criterion) sizes rarely.
const MEAN_DIAMETER_MM: f64 = 8.0;

fn sample_diameter_mm(rng: &mut Rng) -> f64 {
    MEAN_DIAMETER_MM * -(rng.unit().max(1e-9)).ln()
}

/// GENERIC background hail-encounter rate, per second, at
/// `hail_intensity = 1.0` (flying through an active hail shaft): far rarer
/// than ordinary rain but common enough that most airline pilots report
/// at least one hail encounter over a career; not fitted to a published
/// rate.
const BASE_RATE_PER_S_AT_FULL_INTENSITY: f64 = 1.0 / 30.0;

/// The aircraft-side inputs a hail step needs beyond intensity/dt: true
/// airspeed and each engine's current N1 fraction (`EngineInlet`'s
/// flameout-risk term).
#[derive(Clone, Copy, Debug)]
pub struct HailFlightState {
    pub tas_ms: f64,
    pub n1_frac: [f64; N_ENGINE],
}

fn engine_n1_for(target: ImpactTarget, state: &HailFlightState) -> f64 {
    match target {
        ImpactTarget::EngineInlet(i) => state.n1_frac.get(i as usize % N_ENGINE).copied().unwrap_or(0.5),
        _ => 0.5,
    }
}

/// Hail model: the persistent damage state plus a manually armed strike
/// and an optional background random mode.
#[derive(Default)]
pub struct HailModel {
    pub damage: HailDamageState,
    armed: Option<(ImpactTarget, Option<f64>, u32)>,
    pub random_mode: bool,
}

impl HailModel {
    pub fn new() -> Self {
        Self::default()
    }

    /// Arms `count` (>=1) manual hailstones of `diameter_mm` (or
    /// randomised) on `target`; fires on the next `step`.
    pub fn trigger(&mut self, target: ImpactTarget, diameter_mm: Option<f64>, count: u32) {
        self.armed = Some((target, diameter_mm, count.max(1)));
    }

    /// `hail_intensity` (0..1) is a weather-model input. Random mode may
    /// hit several targets in the same shaft, so this returns a `Vec`.
    pub fn step(&mut self, hail_intensity: f64, state: &HailFlightState, dt_s: f64, rng: &mut Rng) -> Vec<HailOutcome> {
        let mut out = Vec::new();
        if let Some((target, diameter, count)) = self.armed.take() {
            let d = diameter.unwrap_or_else(|| sample_diameter_mm(rng));
            let n1 = engine_n1_for(target, state);
            out.push(self.damage.strike(target, d, state.tas_ms, count, n1));
        }
        if self.random_mode {
            let p = BASE_RATE_PER_S_AT_FULL_INTENSITY * hail_intensity.clamp(0.0, 1.0) * dt_s.max(0.0);
            if rng.chance(p) {
                let d = sample_diameter_mm(rng);
                let candidates = [
                    ImpactTarget::Radome,
                    ImpactTarget::Windshield(0),
                    ImpactTarget::WingLeadingEdge(0),
                    ImpactTarget::WingLeadingEdge(6),
                    ImpactTarget::EngineInlet(0),
                    ImpactTarget::EngineInlet(1),
                    ImpactTarget::Nacelle(0),
                    ImpactTarget::Probe(0),
                ];
                for t in candidates {
                    // GENERIC: not every part in the shaft is necessarily
                    // hit by a stone this size each step; independent
                    // per-target chance proportional to hail density.
                    if rng.chance(0.5 * hail_intensity.clamp(0.0, 1.0)) {
                        let count = 1 + rng.below(4) as u32;
                        let n1 = engine_n1_for(t, state);
                        out.push(self.damage.strike(t, d, state.tas_ms, count, n1));
                    }
                }
            }
        }
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn flight(tas_ms: f64, n1: f64) -> HailFlightState {
        HailFlightState { tas_ms, n1_frac: [n1; N_ENGINE] }
    }

    #[test]
    fn torro_classification_matches_the_published_bands() {
        assert_eq!(torro_class(5.0), TorroClass::H0);
        assert_eq!(torro_class(15.0), TorroClass::H1);
        assert_eq!(torro_class(20.0), TorroClass::H2);
        assert_eq!(torro_class(101.0), TorroClass::H10);
    }

    #[test]
    fn max_sustainable_diameter_round_trips_terminal_velocity() {
        let d = 25.0;
        let v = terminal_velocity_ms(d);
        assert!((max_sustainable_diameter_mm(v) - d).abs() < 1e-6);
        assert_eq!(max_sustainable_diameter_mm(0.0), 0.0);
    }

    #[test]
    fn no_nan_or_negative_at_zero_speed_and_zero_size() {
        let mut s = HailDamageState::new();
        let out = s.strike(ImpactTarget::Radome, 0.0, 0.0, 1, 1.0);
        assert_eq!(out.impact_energy_j, 0.0);
        assert_eq!(out.damage_frac, 0.0);
        assert!(!out.impact_speed_ms.is_nan());
    }

    #[test]
    fn a_stronger_longer_higher_airspeed_encounter_damages_more() {
        let mut mild = HailDamageState::new();
        let mut severe = HailDamageState::new();
        // Mild: a few small stones at low speed.
        let mild_out = mild.strike(ImpactTarget::Radome, 8.0, 100.0, 3, 1.0);
        // Severe: bigger stones, higher airspeed, more of them (longer
        // encounter, modelled as a bigger `count` this step).
        let severe_out = severe.strike(ImpactTarget::Radome, 25.0, 250.0, 2, 1.0);
        assert!(severe_out.impact_energy_j > mild_out.impact_energy_j);
        assert!(severe_out.damage_frac > mild_out.damage_frac);
        assert!(severe_out.damage_frac < 1.0, "test needs headroom to show further accumulation");
        // A second, equally severe encounter on top of the first damages
        // it further still (cumulative, not reset).
        let again = severe.strike(ImpactTarget::Radome, 25.0, 250.0, 2, 1.0);
        assert!(again.damage_frac > severe_out.damage_frac);
    }

    #[test]
    fn damage_persists_after_the_storm_is_left_behind() {
        let mut s = HailDamageState::new();
        let during = s.strike(ImpactTarget::WingLeadingEdge(2), 20.0, 220.0, 8, 1.0);
        assert!(during.damage_frac > 0.0);
        // "After the storm": zero-size/zero-speed hit (nothing new lands),
        // damage must not have healed.
        let after = s.strike(ImpactTarget::WingLeadingEdge(2), 0.0, 0.0, 1, 1.0);
        assert_eq!(after.damage_frac, during.damage_frac);
    }

    #[test]
    fn radome_damage_drives_both_wxr_attenuation_and_drag() {
        let mut s = HailDamageState::new();
        let out = s.strike(ImpactTarget::Radome, 20.0, REFERENCE_SPEED_MS, 15, 1.0);
        assert!((out.damage_frac - 1.0).abs() < 1e-6);
        assert_eq!(out.wxr_attenuation_frac, out.damage_frac);
        assert!(out.drag_delta_cd > 0.0);
    }

    #[test]
    fn severe_windshield_damage_faults_window_heat_and_eventually_leaks() {
        let mut s = HailDamageState::new();
        let mild = s.strike(ImpactTarget::Windshield(0), 10.0, 150.0, 2, 1.0);
        assert!(!mild.window_heat_fault);
        assert_eq!(mild.leak_area_m2, 0.0);
        let severe = s.strike(ImpactTarget::Windshield(0), 30.0, REFERENCE_SPEED_MS, 15, 1.0);
        assert!(severe.window_heat_fault);
        assert!(severe.leak_area_m2 > 0.0);
        assert!(severe.visibility_loss_frac > mild.visibility_loss_frac);
    }

    #[test]
    fn wing_leading_edge_damage_costs_clmax_and_can_risk_a_slat_jam() {
        let mut s = HailDamageState::new();
        let light = s.strike(ImpactTarget::WingLeadingEdge(0), 8.0, 150.0, 2, 1.0);
        assert_eq!(light.slat_jam_risk_frac, 0.0);
        let heavy = s.strike(ImpactTarget::WingLeadingEdge(0), 30.0, REFERENCE_SPEED_MS, 15, 1.0);
        assert!(heavy.clmax_delta < light.clmax_delta);
        assert!(heavy.slat_jam_risk_frac > 0.0);
    }

    #[test]
    fn engine_ingestion_hurts_worse_at_low_power_than_high_power() {
        let mut low = HailDamageState::new();
        let mut high = HailDamageState::new();
        let low_out = low.strike(ImpactTarget::EngineInlet(0), 20.0, 100.0, 5, 0.2);
        let high_out = high.strike(ImpactTarget::EngineInlet(0), 20.0, 100.0, 5, 1.0);
        assert!(low_out.flameout_risk_frac > high_out.flameout_risk_frac);
        // Fan damage and compressor erosion themselves do not depend on
        // N1 (they are struck by the ice regardless of power setting).
        assert_eq!(low_out.fan_damage_frac, high_out.fan_damage_frac);
        assert_eq!(low_out.compressor_efficiency_loss_frac, high_out.compressor_efficiency_loss_frac);
        assert!(high_out.compressor_efficiency_loss_frac > 0.0);
    }

    #[test]
    fn compressor_erosion_never_exceeds_its_ceiling() {
        let mut s = HailDamageState::new();
        let mut out = s.strike(ImpactTarget::EngineInlet(1), 40.0, 250.0, 20, 1.0);
        for _ in 0..50 {
            out = s.strike(ImpactTarget::EngineInlet(1), 40.0, 250.0, 20, 1.0);
        }
        assert!(out.compressor_efficiency_loss_frac <= MAX_ENGINE_EROSION + 1e-9);
        assert!(out.flameout_risk_frac <= 1.0);
    }

    #[test]
    fn probe_damage_accumulates_from_small_hits() {
        let mut s = HailDamageState::new();
        let a = s.strike(ImpactTarget::Probe(0), 6.0, 150.0, 1, 1.0);
        let b = s.strike(ImpactTarget::Probe(0), 6.0, 150.0, 1, 1.0);
        assert!(b.damage_frac > a.damage_frac);
    }

    #[test]
    fn nacelle_denting_adds_drag() {
        let mut s = HailDamageState::new();
        let out = s.strike(ImpactTarget::Nacelle(0), 25.0, 220.0, 10, 1.0);
        assert!(out.drag_delta_cd > 0.0);
    }

    #[test]
    fn manual_trigger_fires_once_with_the_requested_size_and_count() {
        let mut model = HailModel::new();
        model.trigger(ImpactTarget::WingLeadingEdge(3), Some(35.0), 5);
        let mut rng = Rng::new(1);
        let first = model.step(0.0, &flight(150.0, 1.0), 1.0, &mut rng);
        assert_eq!(first.len(), 1);
        assert_eq!(first[0].diameter_mm, 35.0);
        assert!(model.step(0.0, &flight(150.0, 1.0), 1.0, &mut rng).is_empty());
    }

    #[test]
    fn random_mode_needs_hail_intensity() {
        let mut model = HailModel::new();
        model.random_mode = true;
        let mut rng = Rng::new(2);
        for _ in 0..500_000 {
            assert!(model.step(0.0, &flight(200.0, 1.0), 1.0, &mut rng).is_empty());
        }
    }

    #[test]
    fn random_mode_eventually_strikes_in_a_hail_shaft() {
        let mut model = HailModel::new();
        model.random_mode = true;
        let mut rng = Rng::new(3);
        let mut any = false;
        for _ in 0..200_000 {
            if !model.step(1.0, &flight(200.0, 1.0), 1.0, &mut rng).is_empty() {
                any = true;
                break;
            }
        }
        assert!(any);
    }
}
