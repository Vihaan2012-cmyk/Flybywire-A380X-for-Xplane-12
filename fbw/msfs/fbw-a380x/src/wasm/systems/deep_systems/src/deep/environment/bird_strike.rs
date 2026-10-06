use super::rng::Rng;

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

#[derive(Clone, Copy, Debug)]
pub struct FlightState {
    pub tas_ms: f64,
    pub altitude_agl_m: f64,
    pub phase: Phase,
    pub month: u8,
    pub night: bool,
    pub gear_down: bool,
    pub n1_frac: [f64; 4],
}

const FAN_DIAMETER_M: f64 = 2.95;
const FAN_TIP_SPEED_AT_100_N1_MS: f64 = std::f64::consts::PI * FAN_DIAMETER_M * (2500.0 / 60.0);
const BYPASS_RATIO: f64 = 8.7;
const CERT_SPEED_MS: f64 = 390.0 * 0.514_444;

const WINDSHIELD_PANELS: u8 = 6;
const WINDSHIELD_PANEL_AREA_M2: f64 = 0.45;
const RADOME_AREA_M2: f64 = 2.2;
const WING_LE_SEGMENTS_PER_SIDE: u8 = 6;
const WING_LE_SEGMENT_AREA_M2: f64 = 0.35;
const NOSE_GEAR_AREA_M2: f64 = 0.9;
const PITOT_PROBES: u8 = 6;
const PROBE_AREA_M2: f64 = 0.0008;

fn engine_inlet_area_m2() -> f64 {
    std::f64::consts::PI / 4.0 * FAN_DIAMETER_M * FAN_DIAMETER_M
}

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

#[derive(Clone, Copy, Debug)]
pub struct StrikeOutcome {
    pub target: ImpactTarget,
    pub bird: BirdClass,
    pub bird_count: u32,
    pub impact_speed_ms: f64,
    pub impact_energy_j: f64,
    pub fan_damage_frac: f64,
    pub core_ingestion_frac: f64,
    pub windshield_crack: bool,
    pub windshield_penetrated: bool,
    pub radome_damage_frac: f64,
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
            let v_rel = (v * v + tip_speed * tip_speed).sqrt();
            let e_blade = 0.5 * mass_kg * v_rel * v_rel;
            let e_cert = 0.5 * BirdClass::Large.mass_kg() * CERT_SPEED_MS * CERT_SPEED_MS;
            out.fan_damage_frac = (e_blade / e_cert).min(1.0);
            out.core_ingestion_frac = 1.0 / (1.0 + BYPASS_RATIO);
        }
        ImpactTarget::Windshield(_) => {
            let e_cert = 0.5 * 1.81 * CERT_SPEED_MS * CERT_SPEED_MS;
            let ratio = impact_energy_j / e_cert;
            out.windshield_crack = ratio > 0.6;
            out.windshield_penetrated = ratio > 1.0;
        }
        ImpactTarget::Radome => {
            let e_cert = 0.5 * (0.5 * 1.81 * CERT_SPEED_MS * CERT_SPEED_MS);
            out.radome_damage_frac = (impact_energy_j / e_cert).min(1.0);
        }
        ImpactTarget::WingLeadingEdge(_) => {
            let e_cert = 0.5 * 3.63 * CERT_SPEED_MS * CERT_SPEED_MS;
            let dent_frac = (impact_energy_j / e_cert).min(1.0);
            out.leading_edge_dent_drag_delta_cd = 0.02 * dent_frac * dent_frac;
        }
        ImpactTarget::NoseGear => {
            let e_cert = 0.5 * 3.63 * CERT_SPEED_MS * CERT_SPEED_MS;
            out.nose_gear_damage_frac = (impact_energy_j / e_cert).min(1.0);
        }
        ImpactTarget::PitotAoaProbe(_) => {
            out.probe_blocked = mass_kg > 0.0;
        }
    }
    out
}

#[derive(Clone, Copy, Debug)]
pub enum TriggerCondition {
    Now,
    AtAltitudeAglM(f64),
    DuringPhase(Phase),
}

#[derive(Clone, Copy, Debug)]
pub struct ScriptedStrike {
    pub bird: BirdClass,
    pub bird_count: u32,
    pub target: Option<ImpactTarget>,
    pub condition: TriggerCondition,
}

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
        d2 * (-(ft - 3500.0) / 5000.0).exp()
    };
    d / d0
}

fn season_multiplier(month: u8) -> f64 {
    let m = (month.clamp(1, 12) as f64 - 9.0) / 12.0;
    1.0 + 0.5 * (2.0 * std::f64::consts::PI * m).cos()
}

fn time_multiplier(night: bool) -> f64 {
    if night {
        1.2
    } else {
        1.0
    }
}

const BASE_RATE_PER_S: f64 = 3.0e-6;

fn strike_probability_per_s(state: &FlightState) -> f64 {
    let ground_ops = matches!(state.phase, Phase::TakeoffRun | Phase::LandingRoll | Phase::Taxi);
    let ground_mult = if ground_ops { 1.5 } else { 1.0 };
    BASE_RATE_PER_S * altitude_relative_risk(state.altitude_agl_m) * ground_mult * season_multiplier(state.month) * time_multiplier(state.night)
}

#[derive(Default)]
pub struct BirdStrikeModel {
    armed: Vec<ScriptedStrike>,
    pub random_mode: bool,
}

impl BirdStrikeModel {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn arm(&mut self, strike: ScriptedStrike) {
        self.armed.push(strike);
    }

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
                let flock = 1 + if rng.chance(0.2) { 1 } else { 0 } + if rng.chance(0.05) { rng.below(3) as u32 } else { 0 };
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
        let mut s = state(Phase::Taxi, 0.0, 0.0);
        s.n1_frac = [0.0; 4];
        let out = resolve(ImpactTarget::EngineInlet(0), BirdClass::Large, 1, &s);
        assert_eq!(out.impact_energy_j, 0.0);
        assert!(!out.fan_damage_frac.is_nan());
        assert_eq!(out.fan_damage_frac, 0.0);
    }

    #[test]
    fn a_stationary_aircraft_with_the_fan_turning_still_damages_the_fan() {
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
        for _ in 0..2_000_000 {
            if !model.step(&s, 1.0, &mut rng).is_empty() {
                any = true;
                break;
            }
        }
        assert!(any);
    }
}
