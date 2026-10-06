use deep_systems::wear::{Wear, WearStore};
use systems::simulation::{InitContext, Read, SimulatorReader, SimulatorWriter, VariableIdentifier, Write};

pub(super) const TGT_OVERTEMP_LIMIT_UNTRIMMED_C: f64 = 991.0;
pub(super) const OVERTEMP_ALLOWANCE_S: f64 = 20.0;
const LIFE_PER_S_AT_LIMIT: f64 = 0.1 / OVERTEMP_ALLOWANCE_S;
const DOUBLING_INTERVAL_C: f64 = 10.0;

pub(super) fn engine_key(engine: usize) -> String {
    format!("engine-{}", engine + 1)
}

pub(super) fn life_rate_per_s(tgt_untrimmed_c: f64, seconds_above_limit: f64) -> f64 {
    if tgt_untrimmed_c <= TGT_OVERTEMP_LIMIT_UNTRIMMED_C || seconds_above_limit <= OVERTEMP_ALLOWANCE_S {
        return 0.;
    }
    LIFE_PER_S_AT_LIMIT * 2f64.powf((tgt_untrimmed_c - TGT_OVERTEMP_LIMIT_UNTRIMMED_C) / DOUBLING_INTERVAL_C)
}

pub(super) struct HotSectionLife {
    damage_ids: [u64; 4],
    release_ids: [u64; 4],
    seconds_above_limit: [f64; 4],
}

impl HotSectionLife {
    pub(super) fn new() -> Self {
        let registry = deep_systems::deep::registry();
        let id = |engine: usize, fragment: &str| -> u64 {
            let component = format!("72_turb.blade_damage_{}", engine + 1);
            registry
                .failures
                .iter()
                .find(|f| f.component == component && f.model_field.contains(fragment))
                .map(|f| f.id)
                .unwrap_or_else(|| panic!("no {fragment} failure on {component}"))
        };
        Self {
            damage_ids: std::array::from_fn(|e| id(e, "damage_frac")),
            release_ids: std::array::from_fn(|e| id(e, "release_frac")),
            seconds_above_limit: [0.; 4],
        }
    }

    pub(super) fn repair(&mut self, engine: usize, wear: &mut WearStore) {
        let key = engine_key(engine);
        let w = wear.get(&key);
        wear.set(&key, Wear { thermal_stress_integral: 0., degradation_fraction: 0., ..w });
        self.seconds_above_limit[engine] = 0.;
    }

    pub(super) fn update(&mut self, tgt_untrimmed_c: [f64; 4], running: [bool; 4], dt_s: f64, wear: &mut WearStore) {
        for engine in 0..4 {
            if running[engine] && tgt_untrimmed_c[engine] > TGT_OVERTEMP_LIMIT_UNTRIMMED_C {
                self.seconds_above_limit[engine] += dt_s;
            } else {
                self.seconds_above_limit[engine] = 0.;
            }
            let rate = life_rate_per_s(tgt_untrimmed_c[engine], self.seconds_above_limit[engine]);
            if rate > 0. {
                let key = engine_key(engine);
                let used = wear.get(&key).degradation_fraction;
                let delta = (rate * dt_s).min(1. - used).max(0.);
                wear.accumulate(&key, 0., 0, dt_s, delta);
            }
        }
    }

    pub(super) fn derived(&self, wear: &WearStore) -> Vec<(u64, f64)> {
        let mut out = Vec::new();
        for engine in 0..4 {
            let used = wear.get(&engine_key(engine)).degradation_fraction.clamp(0., 1.);
            if used > 0. {
                out.push((self.damage_ids[engine], used));
            }
            if used >= 1. {
                out.push((self.release_ids[engine], 1.));
            }
        }
        out
    }
}

pub(super) struct HotSection {
    life: HotSectionLife,
    life_used_ids: [VariableIdentifier; 4],
    repair_id: VariableIdentifier,
    repair: f64,
}

impl HotSection {
    pub(super) fn new(context: &mut InitContext) -> Self {
        Self {
            life: HotSectionLife::new(),
            life_used_ids: std::array::from_fn(|e| context.get_identifier(format!("DEEP_ENG_{}_HOT_SECTION_LIFE_USED", e + 1))),
            repair_id: context.get_identifier("DEEP_ENG_HOT_SECTION_REPAIR".to_owned()),
            repair: 0.,
        }
    }

    pub(super) fn read(&mut self, reader: &mut SimulatorReader) {
        self.repair = reader.read(&self.repair_id);
    }

    pub(super) fn update(&mut self, tgt_untrimmed_c: [f64; 4], running: [bool; 4], dt_s: f64, wear: &mut WearStore) {
        let repair = self.repair.round() as i64;
        for engine in 0..4 {
            if repair == -1 || repair == engine as i64 + 1 {
                self.life.repair(engine, wear);
            }
        }
        self.life.update(tgt_untrimmed_c, running, dt_s, wear);
    }

    pub(super) fn derived(&self, wear: &WearStore) -> Vec<(u64, f64)> {
        self.life.derived(wear)
    }

    pub(super) fn write(&self, writer: &mut SimulatorWriter, wear: &WearStore) {
        for (engine, id) in self.life_used_ids.iter().enumerate() {
            writer.write(id, wear.get(&engine_key(engine)).degradation_fraction.clamp(0., 1.));
        }
        if self.repair != 0. {
            writer.write(&self.repair_id, 0.);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const RUNNING: [bool; 4] = [true; 4];

    fn hold(life: &mut HotSectionLife, wear: &mut WearStore, tgt_c: f64, running: [bool; 4], seconds: f64) {
        for _ in 0..(seconds * 10.).round() as usize {
            life.update([tgt_c, 900., 900., 900.], running, 0.1, wear);
        }
    }

    #[test]
    fn normal_takeoff_and_even_a_long_hot_climb_cost_no_life() {
        let mut life = HotSectionLife::new();
        let mut wear = WearStore::default();
        hold(&mut life, &mut wear, 985., RUNNING, 3600.);
        assert_eq!(wear.get("engine-1").degradation_fraction, 0., "below the 991 C untrimmed over-temperature limit nothing is consumed");
        assert!(life.derived(&wear).is_empty());
    }

    #[test]
    fn the_twenty_second_over_temperature_allowance_is_free_and_resets_per_event() {
        let mut life = HotSectionLife::new();
        let mut wear = WearStore::default();
        for _ in 0..5 {
            hold(&mut life, &mut wear, 1010., RUNNING, 19.);
            hold(&mut life, &mut wear, 950., RUNNING, 5.);
        }
        assert_eq!(wear.get("engine-1").degradation_fraction, 0.);
    }

    #[test]
    fn time_past_the_allowance_consumes_life_faster_the_hotter_it_runs() {
        let mut warm_life = HotSectionLife::new();
        let mut warm = WearStore::default();
        hold(&mut warm_life, &mut warm, 995., RUNNING, 30.);
        let mut hot_life = HotSectionLife::new();
        let mut hot = WearStore::default();
        hold(&mut hot_life, &mut hot, 1015., RUNNING, 30.);
        let (w, h) = (warm.get("engine-1").degradation_fraction, hot.get("engine-1").degradation_fraction);
        assert!(w > 0. && h > 3.5 * w, "warm {w}, hot {h}");
        assert_eq!(warm.get("engine-2").degradation_fraction, 0., "only the hot engine is charged");
    }

    #[test]
    fn a_used_up_hot_section_releases_blades_and_a_repair_restores_it() {
        let mut life = HotSectionLife::new();
        let mut wear = WearStore::default();
        hold(&mut life, &mut wear, 1031., RUNNING, 60.);
        assert_eq!(wear.get("engine-1").degradation_fraction, 1.);
        let derived = life.derived(&wear);
        assert_eq!(derived.len(), 2, "blade damage plus blade release: {derived:?}");
        life.repair(0, &mut wear);
        assert!(life.derived(&wear).is_empty());
        assert_eq!(wear.get("engine-1").degradation_fraction, 0.);
    }

    #[test]
    fn a_stopped_engine_is_never_charged() {
        let mut life = HotSectionLife::new();
        let mut wear = WearStore::default();
        hold(&mut life, &mut wear, 1100., [false; 4], 600.);
        assert_eq!(wear.get("engine-1").degradation_fraction, 0.);
    }
}
