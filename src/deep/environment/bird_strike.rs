//! Bird strike: encounter probability by altitude/phase/season/time of day,
//! bird mass classes from certification standards, impact-point selection
//! weighted by each target's frontal area, and impact energy -> damage per
//! target. A manual trigger API (bird size, target, fire now / at an AGL
//! altitude / during a flight-phase window) plus an optional random mode
//! for background encounters.
//!
//! ## Sources
//! - Altitude/phase distribution: FAA National Wildlife Strike Database,
//!   *Wildlife Strikes to Civil Aircraft in the United States, 1990-2022*
//!   (faa.gov/sites/faa.gov/files/Wildlife-Strike-Report-1990-2022.pdf):
//!   92% of strikes at or below 3,500 ft AGL, 82% at or below 1,500 ft, 71%
//!   at or below 500 ft; about 61% of strikes happen during arrival
//!   (descent/approach/landing roll), 36% during departure (takeoff
//!   run/climb), 3% en route.
//! - Bird masses: EASA CS-25.631 (primary structure, 8 lb / 3.63 kg bird at
//!   Vc), CS-25.775(b) (windshield/flight-deck window, 4 lb / 1.81 kg),
//!   CS-E 800 / 14 CFR 33.76 (engine ingestion: small bird 85 g, one per
//!   0.032 m^2 of inlet throat area up to 16; medium bird ~0.70 kg (1.5 lb),
//!   1-3 birds; large bird 3.65 kg / 8 lb, single bird, for engines with an
//!   inlet throat area at or above 3.9 m^2 -- the Trent 972B-84's class,
//!   fan diameter 2.95 m per EASA TCDS E.012, easily clears that threshold).
//! - Test/reference speed: this crate's own `docs/physics/failures.md`
//!   cites VMO = 390 kt from FlyByWire's public `flight_model.cfg` (which
//!   matches the public A380 type's certified VMO); used here as the
//!   generic worst-case impact speed reference for certification energies
//!   quoted "at Vc", since Vc itself is not separately published.
//! - Season/time-of-day/species-mix multipliers are `GENERIC`: the report
//!   above documents that strikes peak in the Northern Hemisphere autumn
//!   (fledgling and migratory activity, Aug-Oct) and that avoidance is
//!   harder at night, but publishes no exact multiplier curve, so the
//!   shapes below are a defensible order-of-magnitude approximation, not a
//!   fitted regression.
//! - Fan tip speed, target frontal areas (other than the fan, sourced
//!   above) and the damage-fraction curves are `GENERIC`, derived as
//!   documented at each constant; no Trent 900 bird-ingestion test report
//!   is public.

use super::rng::Rng;

/// CS-E 800 / 14 CFR 33.76 single-bird ingestion masses, kg.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BirdClass {
    Small,
    Medium,
    Large,
}

impl BirdClass {
    pub fn mass_kg(self) -> f64 {
        match self {
            BirdClass::Small => 0.085,
            BirdClass::Medium => 0.70,
            BirdClass::Large => 3.65,
        }
    }
}

/// A location on the airframe a bird can hit, each with its own damage
/// model. Indices are 0-based (`EngineInlet(0..4)`, four engines;
/// `WingLeadingEdge(0..12)`, six chordwise-adjacent segments per side,
/// 0-5 left root-to-tip then 6-11 right root-to-tip).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ImpactTarget {
    EngineInlet(u8),
    Windshield(u8),
    Radome,
    WingLeadingEdge(u8),
    NoseGear,
    PitotAoaProbe(u8),
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Phase {
    TakeoffRun,
    Climb,
    EnRoute,
    Descent,
    Approach,
    LandingRoll,
    Taxi,
}

/// What the caller tells this model about the aircraft each step; owned by
/// the flight-model/ADIRS side, read-only here.
#[derive(Clone, Copy, Debug)]
pub struct FlightState {
    pub tas_ms: f64,
    pub altitude_agl_m: f64,
    pub phase: Phase,
    /// 1..=12.
    pub month: u8,
    pub night: bool,
    pub gear_down: bool,
    /// Fan speed as a fraction of 100% N1, one per engine.
    pub n1_frac: [f64; 4],
}

/// Fan face diameter, m (EASA TCDS E.012, Trent 972B-84).
const FAN_DIAMETER_M: f64 = 2.95;
/// GENERIC: no Trent 900 fan shaft speed is public; 2500 rpm is a typical
/// large-turbofan fan-shaft design-speed order of magnitude, giving the
/// blade tip speed at 100% N1 the bird's closing velocity adds to.
const FAN_TIP_SPEED_AT_100_N1_MS: f64 = std::f64::consts::PI * FAN_DIAMETER_M * (2500.0 / 60.0);
/// Trent 972B-84 bypass ratio, public Rolls-Royce data (~8.7:1), used below
/// to split fan-face flow area between bypass duct and core.
const BYPASS_RATIO: f64 = 8.7;
/// Generic worst-case impact speed reference, m/s (VMO 390 kt, see module
/// doc).
const CERT_SPEED_MS: f64 = 390.0 * 0.514_444;

const WINDSHIELD_PANELS: u8 = 6;
/// GENERIC: typical wide-body flight-deck window pane area, m^2.
const WINDSHIELD_PANEL_AREA_M2: f64 = 0.45;
/// GENERIC: nose radome frontal projection, m^2.
const RADOME_AREA_M2: f64 = 2.2;
const WING_LE_SEGMENTS_PER_SIDE: u8 = 6;
/// GENERIC: segment span times the leading edge's exposed capture depth.
const WING_LE_SEGMENT_AREA_M2: f64 = 0.35;
/// GENERIC: nose gear leg plus wheel frontal area, extended.
const NOSE_GEAR_AREA_M2: f64 = 0.9;
/// GENERIC count: pitot, static, AoA and TAT probes on the forward fuselage.
const PITOT_PROBES: u8 = 6;
/// GENERIC: a few cm^2 probe cross-section, m^2.
const PROBE_AREA_M2: f64 = 0.0008;

fn engine_inlet_area_m2() -> f64 {
    std::f64::consts::PI / 4.0 * FAN_DIAMETER_M * FAN_DIAMETER_M
}

/// Every possible target with its frontal-area weight, given the current
/// state (the nose gear only presents area when extended).
fn targets(state: &FlightState) -> Vec<(ImpactTarget, f64)> {
    let mut v = Vec::new();
    for i in 0..4u8 {
        v.push((ImpactTarget::EngineInlet(i), engine_inlet_area_m2()));
    }
    for i in 0..WINDSHIELD_PANELS {
        v.push((ImpactTarget::Windshield(i), WINDSHIELD_PANEL_AREA_M2));
    }
    v.push((ImpactTarget::Radome, RADOME_AREA_M2));
    for i in 0..(2 * WING_LE_SEGMENTS_PER_SIDE) {
        v.push((ImpactTarget::WingLeadingEdge(i), WING_LE_SEGMENT_AREA_M2));
    }
    if state.gear_down {
        v.push((ImpactTarget::NoseGear, NOSE_GEAR_AREA_M2));
    }
    for i in 0..PITOT_PROBES {
        v.push((ImpactTarget::PitotAoaProbe(i), PROBE_AREA_M2));
    }
    v
}

/// Picks a target at random, weighted by each candidate's frontal area (a
/// bird crossing the flight path at a random point is more likely to hit
/// whatever presents the biggest cross-section).
fn pick_target(state: &FlightState, rng: &mut Rng) -> ImpactTarget {
    let list = targets(state);
    let total: f64 = list.iter().map(|(_, a)| a).sum();
    let mut r = rng.unit() * total.max(1e-9);
    for (t, a) in &list {
        if r < *a {
            return *t;
        }
        r -= a;
    }
    list.last().map(|(t, _)| *t).unwrap_or(ImpactTarget::Radome)
}

/// What a strike (or a flock's worth of birds converging on one target) did.
#[derive(Clone, Copy, Debug)]
pub struct StrikeOutcome {
    pub target: ImpactTarget,
    pub bird: BirdClass,
    /// Birds of this target's group that hit here.
    pub bird_count: u32,
    pub impact_speed_ms: f64,
    pub impact_energy_j: f64,
    /// `EngineInlet` only: fan blade damage, 0 none .. 1 destructive.
    pub fan_damage_frac: f64,
    /// `EngineInlet` only: fraction of the ingested mass that reaches the
    /// IP compressor's core flow path rather than the bypass duct.
    pub core_ingestion_frac: f64,
    pub windshield_crack: bool,
    pub windshield_penetrated: bool,
    pub radome_damage_frac: f64,
    /// `WingLeadingEdge` only: local drag-coefficient increment from the
    /// dent's flow separation.
    pub leading_edge_dent_drag_delta_cd: f64,
    pub nose_gear_damage_frac: f64,
    pub probe_blocked: bool,
}

fn resolve(target: ImpactTarget, bird: BirdClass, bird_count: u32, state: &FlightState) -> StrikeOutcome {
    let n = bird_count.max(1) as f64;
    let mass_kg = bird.mass_kg() * n;
    let v = state.tas_ms.max(0.0);
    let impact_energy_j = 0.5 * mass_kg * v * v;
    let mut out = StrikeOutcome {
        target,
        bird,
        bird_count: bird_count.max(1),
        impact_speed_ms: v,
        impact_energy_j,
        fan_damage_frac: 0.0,
        core_ingestion_frac: 0.0,
        windshield_crack: false,
        windshield_penetrated: false,
        radome_damage_frac: 0.0,
        leading_edge_dent_drag_delta_cd: 0.0,
        nose_gear_damage_frac: 0.0,
        probe_blocked: false,
    };
    match target {
        ImpactTarget::EngineInlet(i) => {
            let n1 = state.n1_frac.get(i as usize).copied().unwrap_or(0.0).clamp(0.0, 1.2);
            let tip_speed = FAN_TIP_SPEED_AT_100_N1_MS * n1;
            // The blade sees the bird closing at the vector sum of the
            // aircraft's speed and the blade's own tangential speed.
            let v_rel = (v * v + tip_speed * tip_speed).sqrt();
            let e_blade = 0.5 * mass_kg * v_rel * v_rel;
            // Reference: CS-E 800's large-bird (8 lb) ingestion test, which
            // an engine must survive without a hazardous (uncontained,
            // high-energy-release) failure -- used here as the energy at
            // which fan damage is judged total, not as a literal pass/fail
            // bound.
            let e_cert = 0.5 * BirdClass::Large.mass_kg() * CERT_SPEED_MS * CERT_SPEED_MS;
            out.fan_damage_frac = (e_blade / e_cert).min(1.0);
            // A bird crossing the fan face at a random radius ends up in
            // the core annulus with probability equal to the core's share
            // of the fan-face flow, approximated by the bypass split.
            out.core_ingestion_frac = 1.0 / (1.0 + BYPASS_RATIO);
        }
        ImpactTarget::Windshield(_) => {
            // CS-25.775(b): windshield must not be penetrated by a 4 lb
            // bird at Vc.
            let e_cert = 0.5 * 1.81 * CERT_SPEED_MS * CERT_SPEED_MS;
            let ratio = impact_energy_j / e_cert;
            out.windshield_crack = ratio > 0.6;
            out.windshield_penetrated = ratio > 1.0;
        }
        ImpactTarget::Radome => {
            // GENERIC: no radome-specific bird-strike certification energy
            // is public. The radome is not a pressure boundary and is
            // generally the airframe's weakest bird-strike point, so half
            // the windshield's CS-25.775(b) reference energy is used as an
            // order-of-magnitude "fully damaged" threshold.
            let e_cert = 0.5 * (0.5 * 1.81 * CERT_SPEED_MS * CERT_SPEED_MS);
            out.radome_damage_frac = (impact_energy_j / e_cert).min(1.0);
        }
        ImpactTarget::WingLeadingEdge(_) => {
            // CS-25.631: primary structure must survive an 8 lb bird at Vc.
            let e_cert = 0.5 * 3.63 * CERT_SPEED_MS * CERT_SPEED_MS;
            let dent_frac = (impact_energy_j / e_cert).min(1.0);
            // GENERIC: a dent's drag increment scaling with roughly the
            // square of its depth fraction is an order-of-magnitude
            // separated-flow heuristic, not a cited figure.
            out.leading_edge_dent_drag_delta_cd = 0.02 * dent_frac * dent_frac;
        }
        ImpactTarget::NoseGear => {
            let e_cert = 0.5 * 3.63 * CERT_SPEED_MS * CERT_SPEED_MS;
            out.nose_gear_damage_frac = (impact_energy_j / e_cert).min(1.0);
        }
        ImpactTarget::PitotAoaProbe(_) => {
            // A probe's frontal area is tiny next to any bird class; any
            // strike is assumed to deform or block it outright.
            out.probe_blocked = mass_kg > 0.0;
        }
    }
    out
}

/// When a manually armed strike fires.
#[derive(Clone, Copy, Debug)]
pub enum TriggerCondition {
    /// Fires on the next `step`.
    Now,
    /// Fires the first `step` at or below this AGL altitude, m.
    AtAltitudeAglM(f64),
    /// Fires the first `step` in this flight phase.
    DuringPhase(Phase),
}

/// A caller-scripted strike, queued with [`BirdStrikeModel::arm`].
#[derive(Clone, Copy, Debug)]
pub struct ScriptedStrike {
    pub bird: BirdClass,
    /// Birds in the flock (>=1); split across the frontal-area-weighted
    /// targets that get hit, unless `target` pins a single one.
    pub bird_count: u32,
    /// `None` picks a target (or several, for a flock) by frontal area.
    pub target: Option<ImpactTarget>,
    pub condition: TriggerCondition,
}

/// Splits a flock across targets: each bird independently picks a target
/// weighted by frontal area (unless `forced_target` pins them all to one),
/// then birds landing on the same target are resolved together.
fn strike_group(bird: BirdClass, bird_count: u32, forced_target: Option<ImpactTarget>, state: &FlightState, rng: &mut Rng) -> Vec<StrikeOutcome> {
    let n = bird_count.max(1);
    if let Some(t) = forced_target {
        return vec![resolve(t, bird, n, state)];
    }
    let mut counts: Vec<(ImpactTarget, u32)> = Vec::new();
    for _ in 0..n {
        let t = pick_target(state, rng);
        match counts.iter_mut().find(|(tt, _)| *tt == t) {
            Some(e) => e.1 += 1,
            None => counts.push((t, 1)),
        }
    }
    counts.into_iter().map(|(t, c)| resolve(t, bird, c, state)).collect()
}

/// Piecewise density built directly from the FAA report's three cumulative
/// percentiles (92% <=3500ft, 82% <=1500ft, 71% <=500ft), density assumed
/// uniform within each band since the report gives no finer resolution,
/// relative to the lowest band's density (the level the base rate below is
/// calibrated at).
fn altitude_relative_risk(agl_m: f64) -> f64 {
    let ft = (agl_m / 0.3048).max(0.0);
    let d0 = 0.71 / 500.0;
    let d1 = 0.11 / 1000.0;
    let d2 = 0.10 / 2000.0;
    let d = if ft <= 500.0 {
        d0
    } else if ft <= 1500.0 {
        d1
    } else if ft <= 3500.0 {
        d2
    } else {
        // Beyond the report's resolution: GENERIC exponential tail so
        // cruise altitude is not literally zero-risk.
        d2 * (-(ft - 3500.0) / 5000.0).exp()
    };
    d / d0
}

/// GENERIC raised cosine peaking in September (Northern Hemisphere
/// fledgling/migration season) with a trough in March; amplitude chosen so
/// the annual mean is exactly 1.0.
fn season_multiplier(month: u8) -> f64 {
    let m = (month.clamp(1, 12) as f64 - 9.0) / 12.0;
    1.0 + 0.5 * (2.0 * std::f64::consts::PI * m).cos()
}

/// GENERIC: reduced avoidance opportunity (pilot and bird alike) at night.
fn time_multiplier(night: bool) -> f64 {
    if night {
        1.2
    } else {
        1.0
    }
}

/// GENERIC base rate, per second, for an aircraft at or below 500 ft AGL,
/// before phase/season/time multipliers: calibrated to the industry order
/// of magnitude of "on the order of one reportable strike per a few
/// thousand flight hours" for a large transport, not a fitted rate.
const BASE_RATE_PER_S: f64 = 3.0e-6;

fn strike_probability_per_s(state: &FlightState) -> f64 {
    // GENERIC: bird activity is denser right at the airfield surface than
    // a low pass elsewhere at the same AGL altitude.
    let ground_ops = matches!(state.phase, Phase::TakeoffRun | Phase::LandingRoll | Phase::Taxi);
    let ground_mult = if ground_ops { 1.5 } else { 1.0 };
    BASE_RATE_PER_S * altitude_relative_risk(state.altitude_agl_m) * ground_mult * season_multiplier(state.month) * time_multiplier(state.night)
}

/// Bird strike model: manually armed scripted events plus an optional
/// background random mode.
#[derive(Default)]
pub struct BirdStrikeModel {
    armed: Vec<ScriptedStrike>,
    pub random_mode: bool,
}

impl BirdStrikeModel {
    pub fn new() -> Self {
        Self::default()
    }

    /// Queues a manually scripted strike; it fires the first `step` whose
    /// flight state satisfies its `condition`.
    pub fn arm(&mut self, strike: ScriptedStrike) {
        self.armed.push(strike);
    }

    /// Advances by `dt_s`, firing any armed strikes whose condition now
    /// holds and, in random mode, possibly generating a background one.
    /// Returns one [`StrikeOutcome`] per distinct target hit this step.
    pub fn step(&mut self, state: &FlightState, dt_s: f64, rng: &mut Rng) -> Vec<StrikeOutcome> {
        let mut out = Vec::new();
        let mut i = 0;
        while i < self.armed.len() {
            let fire = match self.armed[i].condition {
                TriggerCondition::Now => true,
                TriggerCondition::AtAltitudeAglM(a) => state.altitude_agl_m <= a,
                TriggerCondition::DuringPhase(p) => state.phase == p,
            };
            if fire {
                let s = self.armed.remove(i);
                out.extend(strike_group(s.bird, s.bird_count, s.target, state, rng));
            } else {
                i += 1;
            }
        }
        if self.random_mode {
            let p = strike_probability_per_s(state) * dt_s.max(0.0);
            if rng.chance(p) {
                // GENERIC: most strikes are a single bird; occasionally a
                // small flock, consistent with the database's own
                // "number struck" field being overwhelmingly 1 with a
                // long tail.
                let flock = 1 + if rng.chance(0.2) { 1 } else { 0 } + if rng.chance(0.05) { rng.below(3) as u32 } else { 0 };
                // GENERIC species-class mix: small birds dominate strike
                // counts, medium next, large (geese, raptors) rarest.
                let class = if rng.chance(0.6) {
                    BirdClass::Small
                } else if rng.chance(0.8) {
                    BirdClass::Medium
                } else {
                    BirdClass::Large
                };
                out.extend(strike_group(class, flock, None, state, rng));
            }
        }
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn state(phase: Phase, agl_m: f64, tas_ms: f64) -> FlightState {
        FlightState { tas_ms, altitude_agl_m: agl_m, phase, month: 6, night: false, gear_down: agl_m < 100.0, n1_frac: [0.9; 4] }
    }

    #[test]
    fn bird_masses_match_the_certification_figures() {
        assert!((BirdClass::Small.mass_kg() - 0.085).abs() < 1e-9);
        assert!((BirdClass::Medium.mass_kg() - 0.70).abs() < 1e-9);
        assert!((BirdClass::Large.mass_kg() - 3.65).abs() < 1e-9);
    }

    #[test]
    fn altitude_risk_falls_off_with_height_matching_the_faa_bands() {
        let low = altitude_relative_risk(0.0);
        let mid = altitude_relative_risk(1000.0 / 3.281);
        let high = altitude_relative_risk(3000.0 / 3.281);
        let cruise = altitude_relative_risk(35000.0 / 3.281);
        assert_eq!(low, 1.0);
        assert!(mid < low && high < mid && cruise < high);
        assert!(cruise > 0.0, "never literally zero at cruise");
    }

    #[test]
    fn no_nan_at_rest() {
        // Genuinely at rest: stopped *and* with the fans stopped. The
        // helper's default state leaves N1 at 90%, which is not "at rest" --
        // the blade a bird meets there is still closing on it at 90% of fan
        // tip speed, and the model is right to shred it (see the companion
        // assertion below). With nothing moving at all there is no relative
        // velocity, so no energy and no damage.
        let mut s = state(Phase::Taxi, 0.0, 0.0);
        s.n1_frac = [0.0; 4];
        let out = resolve(ImpactTarget::EngineInlet(0), BirdClass::Large, 1, &s);
        assert_eq!(out.impact_energy_j, 0.0);
        assert!(!out.fan_damage_frac.is_nan());
        assert_eq!(out.fan_damage_frac, 0.0);
    }

    #[test]
    fn a_stationary_aircraft_with_the_fan_turning_still_damages_the_fan() {
        // The closing speed a fan blade sees is the vector sum of the
        // aircraft's speed and the blade's own tangential speed, so a bird
        // walked into a running engine on the stand is still a fan-blade
        // event even though the airframe's own impact energy is zero.
        let mut s = state(Phase::Taxi, 0.0, 0.0);
        s.n1_frac = [0.9; 4];
        let out = resolve(ImpactTarget::EngineInlet(0), BirdClass::Large, 1, &s);
        assert_eq!(out.impact_energy_j, 0.0, "the airframe is not moving");
        assert!(out.fan_damage_frac > 0.0, "but the blades are");
    }

    #[test]
    fn a_large_bird_at_high_speed_and_n1_destroys_the_fan_but_a_small_one_barely_scratches_it() {
        let cruise = state(Phase::Climb, 8000.0, 250.0);
        let big = resolve(ImpactTarget::EngineInlet(0), BirdClass::Large, 1, &cruise);
        let small = resolve(ImpactTarget::EngineInlet(0), BirdClass::Small, 1, &cruise);
        assert_eq!(big.fan_damage_frac, 1.0);
        assert!(small.fan_damage_frac < 0.2, "{}", small.fan_damage_frac);
    }

    #[test]
    fn higher_n1_makes_the_same_bird_worse() {
        let mut fast_fan = state(Phase::Climb, 3000.0, 150.0);
        fast_fan.n1_frac = [1.0; 4];
        let mut slow_fan = fast_fan;
        slow_fan.n1_frac = [0.3; 4];
        let a = resolve(ImpactTarget::EngineInlet(0), BirdClass::Medium, 1, &fast_fan);
        let b = resolve(ImpactTarget::EngineInlet(0), BirdClass::Medium, 1, &slow_fan);
        assert!(a.fan_damage_frac > b.fan_damage_frac);
    }

    #[test]
    fn core_ingestion_fraction_matches_the_bypass_split() {
        let s = state(Phase::Climb, 3000.0, 150.0);
        let out = resolve(ImpactTarget::EngineInlet(0), BirdClass::Small, 1, &s);
        assert!((out.core_ingestion_frac - 1.0 / 9.7).abs() < 1e-9);
    }

    #[test]
    fn windshield_survives_a_small_bird_at_approach_speed_but_not_a_large_one_at_cruise() {
        let approach = state(Phase::Approach, 300.0, 80.0);
        let mild = resolve(ImpactTarget::Windshield(0), BirdClass::Small, 1, &approach);
        assert!(!mild.windshield_crack && !mild.windshield_penetrated);

        let cruise = state(Phase::Climb, 8000.0, 250.0);
        let severe = resolve(ImpactTarget::Windshield(0), BirdClass::Large, 1, &cruise);
        assert!(severe.windshield_crack && severe.windshield_penetrated);
    }

    #[test]
    fn a_probe_strike_always_blocks_it() {
        let s = state(Phase::Climb, 3000.0, 150.0);
        let out = resolve(ImpactTarget::PitotAoaProbe(0), BirdClass::Small, 1, &s);
        assert!(out.probe_blocked);
    }

    #[test]
    fn target_selection_is_weighted_by_frontal_area() {
        let s = state(Phase::Climb, 3000.0, 150.0);
        let mut rng = Rng::new(99);
        let (mut engine_hits, mut probe_hits) = (0u32, 0u32);
        for _ in 0..20_000 {
            match pick_target(&s, &mut rng) {
                ImpactTarget::EngineInlet(_) => engine_hits += 1,
                ImpactTarget::PitotAoaProbe(_) => probe_hits += 1,
                _ => {}
            }
        }
        // Each engine inlet (~6.8 m^2) is thousands of times a probe's
        // (~0.0008 m^2) frontal area, so engines should dominate probes by
        // a wide, unambiguous margin.
        assert!(engine_hits > probe_hits * 100, "engines {engine_hits} probes {probe_hits}");
    }

    #[test]
    fn a_scripted_strike_fires_now_exactly_once() {
        let mut model = BirdStrikeModel::new();
        model.arm(ScriptedStrike { bird: BirdClass::Large, bird_count: 1, target: Some(ImpactTarget::EngineInlet(2)), condition: TriggerCondition::Now });
        let mut rng = Rng::new(1);
        let s = state(Phase::Climb, 3000.0, 150.0);
        let first = model.step(&s, 1.0, &mut rng);
        assert_eq!(first.len(), 1);
        assert!(matches!(first[0].target, ImpactTarget::EngineInlet(2)));
        let second = model.step(&s, 1.0, &mut rng);
        assert!(second.is_empty());
    }

    #[test]
    fn a_scripted_strike_waits_for_its_altitude() {
        let mut model = BirdStrikeModel::new();
        model.arm(ScriptedStrike { bird: BirdClass::Medium, bird_count: 1, target: Some(ImpactTarget::Radome), condition: TriggerCondition::AtAltitudeAglM(500.0) });
        let mut rng = Rng::new(2);
        let high = state(Phase::Descent, 2000.0, 150.0);
        assert!(model.step(&high, 1.0, &mut rng).is_empty());
        let low = state(Phase::Descent, 400.0, 100.0);
        assert_eq!(model.step(&low, 1.0, &mut rng).len(), 1);
    }

    #[test]
    fn a_scripted_strike_waits_for_its_phase() {
        let mut model = BirdStrikeModel::new();
        model.arm(ScriptedStrike { bird: BirdClass::Small, bird_count: 3, target: None, condition: TriggerCondition::DuringPhase(Phase::LandingRoll) });
        let mut rng = Rng::new(3);
        let cruise = state(Phase::EnRoute, 10000.0, 250.0);
        assert!(model.step(&cruise, 1.0, &mut rng).is_empty());
        let rolling = state(Phase::LandingRoll, 0.0, 40.0);
        let out = model.step(&rolling, 1.0, &mut rng);
        assert!(!out.is_empty());
        let total: u32 = out.iter().map(|o| o.bird_count).sum();
        assert_eq!(total, 3);
    }

    #[test]
    fn random_mode_eventually_produces_a_strike_at_a_high_forced_rate() {
        let mut model = BirdStrikeModel::new();
        model.random_mode = true;
        let mut rng = Rng::new(5);
        let s = state(Phase::Approach, 200.0, 80.0);
        let mut any = false;
        // Many one-second steps at the lowest, riskiest band should
        // eventually trigger at least one background strike.
        for _ in 0..2_000_000 {
            if !model.step(&s, 1.0, &mut rng).is_empty() {
                any = true;
                break;
            }
        }
        assert!(any);
    }
}
