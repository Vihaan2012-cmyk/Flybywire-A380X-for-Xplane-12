//! Random failures: an MTBF-based engine over a curated subset of the
//! catalogue (`failures.rs`/`failures::extra`), user-configurable including
//! off, pausing when X-Plane pauses (hyperrealism physics workstream 6).
//!
//! No public per-LRU MTBF figure exists for the A380's individual
//! components (Airbus does not publish reliability data at that level), so
//! the hazard rates below are generic, order-of-magnitude civil-aviation
//! reliability figures of the kind used in ARP4761-style safety
//! assessments, clearly marked here rather than presented as sourced:
//! - avionics/electronic LRU (FADEC channel, sensor, resolver): ~25,000
//!   flight-hour MTBF, the low end of the 10,000-50,000 FH band commonly
//!   quoted for certified avionics line-replaceable units.
//! - hydraulic/fuel pump, valve: ~6,000 FH, the low end of the 4,000-8,000
//!   FH band commonly quoted for aircraft hydraulic pumps.
//! - mechanical/structural (bearing wear, tyre burst arming): ~15,000 FH.
//! These are deliberately conservative (short) so the feature is
//! exercisable in a reasonable play session at the default rate multiplier;
//! the multiplier (`Config::rate_multiplier`) lets the Study panel scale
//! them, including to zero (equivalent to disabling).
//!
//! The failure model is the standard constant-hazard-rate (exponential)
//! reliability model: for a component with mean time between failures
//! `mtbf` hours, the probability of failing in a short interval `dt` hours
//! is `1 - exp(-dt/mtbf) ≈ dt/mtbf`. Each tick draws one uniform random
//! number per still-healthy component and fails it if the draw is under
//! that probability, using a seeded PRNG for deterministic tests and replay.

#![allow(dead_code)] // Study-panel API, wired up by the lead (see the workstream 6 report).

use std::collections::BTreeSet;

/// A small, fast, seedable PRNG (xorshift64*, Marsaglia 2003 / Vigna's
/// 2014 multiplier): no external dependency, and fully deterministic from
/// its seed, which is what the persisted/test-seeded behaviour needs.
#[derive(Clone, Copy, Debug, serde::Serialize, serde::Deserialize)]
pub struct Rng(u64);

impl Rng {
    pub fn new(seed: u64) -> Self {
        Self(if seed == 0 { 0x9E3779B97F4A7C15 } else { seed })
    }

    fn next_u64(&mut self) -> u64 {
        let mut x = self.0;
        x ^= x >> 12;
        x ^= x << 25;
        x ^= x >> 27;
        self.0 = x;
        x.wrapping_mul(0x2545_F491_4F6C_DD1D)
    }

    /// Uniform in `[0, 1)`.
    fn next_f64(&mut self) -> f64 {
        (self.next_u64() >> 11) as f64 / (1u64 << 53) as f64
    }
}

/// One component this engine can fail, with its assumed MTBF (flight
/// hours) and the catalogue id it arms.
struct Component {
    id: u64,
    mtbf_hours: f64,
}

/// The curated subset (not the whole catalogue: recurring cosmetic/duplicate
/// sensor ids would make the same handful of components dominate every
/// draw). One representative id per failure family, spread across the
/// catalogue this workstream added plus a few of FlyByWire's own that are
/// plausible random events (a generator, a hydraulic pump).
fn components() -> Vec<Component> {
    let mut v = vec![
        // FlyByWire's own catalogue: single-point electrical/hydraulic
        // failures a real fleet does see at random.
        Component { id: 24_020, mtbf_hours: 25_000. }, // Generator 1
        Component { id: 24_021, mtbf_hours: 25_000. }, // Generator 2
        Component { id: 29_010, mtbf_hours: 6_000. },  // EDP 1a overheat
        Component { id: 29_012, mtbf_hours: 6_000. },  // EDP 2a overheat
    ];
    for x in crate::failures::extra::extra_failures() {
        // Some ATA32/72/34 ids are documented in failures.rs (gear()'s and
        // exceedances()'s own comments) as armed exclusively by damage.rs's
        // brake-energy/EGT-creep/touchdown model, not spontaneous hazards:
        // ATA32 in full (tyre burst 32_100-32_104, brake wear-out
        // 32_110-32_113, and the touchdown exceedances 32_120-32_123),
        // ATA72's EGT/creep-triggered bearing/compressor/turbine ids
        // (72_000-72_011), and 34_120 (VMO/MMO overspeed, the one ATA34 id
        // that is not an ADIRS sensor). Drawing these here let the MTBF
        // engine arm "overweight landing" or a tyre burst on an aircraft
        // that never moved (the symptom in docs/briefs/debug.md): excluded.
        let mtbf = match x.ata {
            24 => 25_000., // bus short
            28 | 29 | 36 => 6_000.,  // pumps, valves, ducts
            34 if x.id != 34_120 => 25_000., // ADIRS sensors (34_120 excluded, see above)
            49 => 12_000., // APU
            77 => 15_000., // sensors (ATA72 excluded, see above)
            73 | 74 | 76 | 78 | 79 | 80 => 20_000., // FADEC/fuel/ignition/reverser/oil/start
            _ => continue,
        };
        v.push(Component { id: x.id, mtbf_hours: mtbf });
    }
    v
}

/// User configuration (Study panel, persisted).
#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Config {
    pub enabled: bool,
    /// Multiplies every component's hazard rate; 0 behaves like `enabled =
    /// false` without losing the setting, 1 is the modelled rate above, >1
    /// exercises the feature faster for testing.
    pub rate_multiplier: f64,
}

impl Default for Config {
    fn default() -> Self {
        Self { enabled: false, rate_multiplier: 1.0 }
    }
}

pub struct RandomFailures {
    pub config: Config,
    rng: Rng,
    components: Vec<Component>,
    /// The `app_settings::generation()` this `config` was last synced
    /// from (`app_settings.rs`'s `xphfbw.randomFailures`/`failureRate`),
    /// so `apply_requests` only resyncs when xphfbw.json actually
    /// reloaded, not every tick.
    app_settings_generation: u64,
}

impl RandomFailures {
    pub fn new(seed: u64) -> Self {
        Self { config: Config::default(), rng: Rng::new(seed), components: components(), app_settings_generation: 0 }
    }

    pub fn restore(&mut self, config: Config, rng: Rng) {
        self.config = config;
        self.rng = rng;
    }

    pub fn snapshot(&self) -> (Config, Rng) {
        (self.config, self.rng)
    }

    /// Apply the Study panel's queued configuration change, if any, else
    /// resync from the app's `xphfbw.randomFailures`/`failureRate`
    /// settings if they changed since last checked (`app_settings.rs`:
    /// live). Call once per tick, same pattern as `mel::Mel::apply_requests`.
    pub fn apply_requests(&mut self) {
        if let Some(config) = take_config_request() {
            self.config = config;
            self.app_settings_generation = crate::app_settings::generation();
            return;
        }
        self.sync_from_app_settings(crate::app_settings::current(), crate::app_settings::generation());
    }

    /// Resyncs `self.config` from `settings` if `generation` is newer than
    /// the one last applied (`app_settings::generation()` only bumps on an
    /// actual xphfbw.json reload, not every poll). Split out from
    /// `apply_requests` so the generation-diffing logic is unit-testable
    /// without the real process-wide settings watcher.
    fn sync_from_app_settings(&mut self, settings: crate::app_settings::AppSettings, generation: u64) {
        if generation != self.app_settings_generation {
            self.app_settings_generation = generation;
            self.config = Config { enabled: settings.random_failures, rate_multiplier: settings.failure_rate };
        }
    }

    /// One tick's draw. `delta_hours` is real elapsed time in hours (the
    /// caller passes `0.0` while X-Plane is paused, which this function
    /// already leaves as a no-op with `enabled` true or false, since `0.0`
    /// hours draws with probability `0`; callers should still skip calling
    /// this while paused to avoid advancing the RNG state for no reason).
    /// `already_active` is `failures::active_ids()` unioned with whatever
    /// the MEL already has deferred: a component already failed or
    /// deferred does not draw again. Returns the newly-triggered ids, which
    /// the caller should pass to `failures::set_active(id, true)`.
    pub fn update(&mut self, delta_hours: f64, already_active: &BTreeSet<u64>) -> Vec<u64> {
        if !self.config.enabled || self.config.rate_multiplier <= 0.0 || delta_hours <= 0.0 {
            return Vec::new();
        }
        let mut triggered = Vec::new();
        for c in &self.components {
            if already_active.contains(&c.id) {
                continue;
            }
            let lambda = self.config.rate_multiplier / c.mtbf_hours;
            let p = 1.0 - (-lambda * delta_hours).exp();
            if self.rng.next_f64() < p {
                triggered.push(c.id);
            }
        }
        triggered
    }
}

/// The Study panel's random-failures configuration page: queue a new
/// `Config` (enabled flag and rate multiplier) for `RandomFailures::
/// apply_requests` to pick up next tick. Queued rather than applied
/// immediately for the same reason `mel::request_defer` is: the page has
/// no direct access to the running `RandomFailures` (owned by `Plugin`).
static CONFIG_REQUEST: std::sync::Mutex<Option<Config>> = std::sync::Mutex::new(None);

pub fn request_config(config: Config) {
    if let Ok(mut r) = CONFIG_REQUEST.lock() {
        *r = Some(config);
    }
}

fn take_config_request() -> Option<Config> {
    CONFIG_REQUEST.lock().ok().and_then(|mut r| r.take())
}

/// Test-isolation helper (see `scenarios::reset_global_state`): drops any
/// queued MTBF-engine config request left over from a previous test.
#[cfg(any(test, feature = "test-support"))]
pub fn reset_for_tests() {
    if let Ok(mut r) = CONFIG_REQUEST.lock() {
        *r = None;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn disabled_never_triggers() {
        // `failures::STATE` is process-wide and these tests arm into it;
        // without this lock they race whatever else is asserting on a
        // magnitude at the time (it wiped breakers.rs's own 29_103
        // expectation in a full-suite run).
        let _serial = crate::failures::tests::SERIAL.lock().unwrap_or_else(|e| e.into_inner());

        let mut rf = RandomFailures::new(1);
        rf.config.enabled = false;
        let active = BTreeSet::new();
        for _ in 0..10_000 {
            assert!(rf.update(1.0, &active).is_empty());
        }
    }

    #[test]
    fn the_same_seed_produces_the_same_sequence() {
        let _serial = crate::failures::tests::SERIAL.lock().unwrap_or_else(|e| e.into_inner());

        let active = BTreeSet::new();
        let run = |seed| {
            let mut rf = RandomFailures::new(seed);
            rf.config.enabled = true;
            rf.config.rate_multiplier = 5000.0; // fast, so the test is short
            let mut out = Vec::new();
            for _ in 0..200 {
                out.extend(rf.update(1.0, &active));
            }
            out
        };
        assert_eq!(run(42), run(42));
    }

    #[test]
    fn a_higher_rate_multiplier_triggers_sooner() {
        let _serial = crate::failures::tests::SERIAL.lock().unwrap_or_else(|e| e.into_inner());

        let active = BTreeSet::new();
        let time_to_first = |multiplier: f64| {
            let mut rf = RandomFailures::new(7);
            rf.config.enabled = true;
            rf.config.rate_multiplier = multiplier;
            for hour in 0..100_000 {
                if !rf.update(1.0, &active).is_empty() {
                    return hour;
                }
            }
            100_000
        };
        assert!(time_to_first(1000.0) <= time_to_first(1.0));
    }

    #[test]
    fn an_already_active_component_never_re_triggers() {
        let _serial = crate::failures::tests::SERIAL.lock().unwrap_or_else(|e| e.into_inner());

        let mut rf = RandomFailures::new(3);
        rf.config.enabled = true;
        rf.config.rate_multiplier = 1_000_000.0;
        let mut active = BTreeSet::new();
        active.insert(24_020);
        active.insert(24_021);
        active.insert(29_010);
        active.insert(29_012);
        for x in crate::failures::extra::extra_failures() {
            active.insert(x.id);
        }
        assert!(rf.update(1.0, &active).is_empty());
    }

    /// Symptom (docs/briefs/debug.md): "failure 32123 (Overweight landing)
    /// activated" on a parked aircraft. damage.rs's own touchdown gate was
    /// fixed separately; this is the random scheduler's half of the same
    /// bug -- the MTBF engine must never be able to draw an id that
    /// failures.rs documents as armed only by damage.rs (every ATA32 id:
    /// tyre burst, brake wear-out, and the touchdown exceedances; ATA72's
    /// EGT/creep-triggered bearing/compressor/turbine ids; 34_120 VMO/MMO
    /// overspeed), since none of those can ever be caused by simply ticking
    /// time forward on a stationary airframe.
    #[test]
    fn damage_rs_armed_ids_are_never_drawn_by_the_mtbf_engine() {
        let _serial = crate::failures::tests::SERIAL.lock().unwrap_or_else(|e| e.into_inner());

        let damage_armed = |id: u64| {
            (32_100..=32_123).contains(&id) || (72_000..=72_011).contains(&id) || id == 34_120
        };
        for c in components() {
            assert!(!damage_armed(c.id), "component {} should not be in the random-failure engine", c.id);
        }
        // End-to-end: even with an enormous rate multiplier over a long
        // stretch of ticks, nothing in `damage_armed` ever comes out.
        let mut rf = RandomFailures::new(11);
        rf.config.enabled = true;
        rf.config.rate_multiplier = 1_000_000.0;
        let active = BTreeSet::new();
        for _ in 0..1000 {
            for id in rf.update(1.0, &active) {
                assert!(!damage_armed(id), "drew damage.rs-armed id {id}");
            }
        }
    }

    #[test]
    fn zero_or_negative_delta_never_triggers() {
        let _serial = crate::failures::tests::SERIAL.lock().unwrap_or_else(|e| e.into_inner());

        let mut rf = RandomFailures::new(9);
        rf.config.enabled = true;
        rf.config.rate_multiplier = 1_000_000.0;
        let active = BTreeSet::new();
        assert!(rf.update(0.0, &active).is_empty());
    }

    fn app_settings(random_failures: bool, failure_rate: f64) -> crate::app_settings::AppSettings {
        crate::app_settings::AppSettings { random_failures, failure_rate, ..Default::default() }
    }

    #[test]
    fn syncs_from_app_settings_only_when_the_generation_moved() {
        let _serial = crate::failures::tests::SERIAL.lock().unwrap_or_else(|e| e.into_inner());

        let mut rf = RandomFailures::new(1);
        assert_eq!(rf.config, Config::default());

        // Generation 0 -> 0: RandomFailures::new()'s starting generation is
        // 0 too, so this is a no-op (matches "nothing reloaded yet").
        rf.sync_from_app_settings(app_settings(true, 5.0), 0);
        assert_eq!(rf.config, Config::default(), "same generation: no resync");

        // Generation moved: a real xphfbw.json reload happened, so this
        // is applied live.
        rf.sync_from_app_settings(app_settings(true, 5.0), 1);
        assert_eq!(rf.config, Config { enabled: true, rate_multiplier: 5.0 });

        // Generation unchanged since: the Study panel (if it had written
        // directly to self.config) would not be stomped every tick.
        rf.config.rate_multiplier = 99.0;
        rf.sync_from_app_settings(app_settings(true, 5.0), 1);
        assert_eq!(rf.config.rate_multiplier, 99.0, "same generation again: no resync");

        // A second real reload (generation 1 -> 2) applies again.
        rf.sync_from_app_settings(app_settings(false, 1.0), 2);
        assert_eq!(rf.config, Config { enabled: false, rate_multiplier: 1.0 });
    }

    #[test]
    fn apply_requests_prefers_a_queued_study_panel_request_over_app_settings() {
        let _serial = crate::failures::tests::SERIAL.lock().unwrap_or_else(|e| e.into_inner());

        let mut rf = RandomFailures::new(5);
        request_config(Config { enabled: true, rate_multiplier: 42.0 });
        rf.apply_requests();
        assert_eq!(rf.config, Config { enabled: true, rate_multiplier: 42.0 });
    }
}
