//! The live layer: what turns the models in this directory from code that
//! compiles into systems that run.
//!
//! Every area under `deep/` models its physics as free functions and small
//! structs with no dependency on `crate::Vars` or X-Plane (`docs/deep/
//! BRIEF.md` hard rule 2), which is what let eighteen areas be written
//! independently. The cost is that nothing owns an instance of any of it:
//! there is no `HydraulicNetwork` anywhere in the running plugin, only the
//! type. This module is the seam that fixes that, without giving the areas
//! the dependency the rule exists to prevent.
//!
//! Three pieces:
//!
//! * [`Truth`] -- everything an area may read about the rest of the
//!   simulation this tick. The plugin fills it once per frame from
//!   X-Plane, from its own engine model and from FlyByWire's published
//!   variables; areas read it and never reach for a variable themselves.
//! * [`Faults`] -- the armed magnitude of every failure, by the id
//!   `deep::api` assigned it. A snapshot, taken by the plugin from
//!   `crate::failures`, so an area never depends on the failure system
//!   either. Every failure in the catalogue is a *continuous* magnitude in
//!   0..1, so an area asks for a number, not for whether something is
//!   "broken".
//! * [`Area`] -- what an area's live system implements: step yourself
//!   forward, then publish what the rest of the aircraft can see.
//!
//! [`Deep`] owns one live system per area and is what `Plugin::tick`
//! actually calls.
//!
//! ## Publishing
//!
//! An area publishes through a closure (`&mut dyn FnMut(&str, f64)`)
//! rather than by being handed the variable registry. That keeps the rule
//! intact -- the area names a variable and a value, and the plugin decides
//! what a variable *is* -- and it means the same area can publish into a
//! test harness, into a recording, or into nothing at all, without a
//! second code path. The names are the ones each area's `registry.rs`
//! already cites in its ECAM triggers, so an alert's trigger and the value
//! it reads cannot drift apart.
//!
//! ## Ordering
//!
//! Areas are stepped in the order they appear in [`Deep::tick`], and an
//! area reads the previous frame's values of anything another area
//! publishes. That one-frame lag is deliberate and is the same lag
//! `extra_backend_fbw.rs` already accepts on the reverser force path: it
//! makes the areas independent of each other's order, which is what keeps
//! them separately testable. At 30-60 Hz it is below the time constant of
//! every process modelled here.

use std::collections::BTreeMap;

use super::integration::weather_truth::EnvironmentTruth;

/// Everything the deep areas read about the rest of the simulation.
///
/// Filled once per frame by the plugin. Anything an area needs that is not
/// here has to be added here first -- that is the point, so there is one
/// list of what the models depend on rather than eighteen.
#[derive(Clone, Debug)]
pub struct Truth {
    /// This frame's length. Never zero, never negative; the plugin clamps
    /// it, because X-Plane hands out a zero `dt` on the frame a flight
    /// loads and a very large one after a pause.
    pub dt_s: f64,
    /// Real weather and atmosphere, as `integration::weather_truth` reads
    /// it from X-Plane.
    pub environment: EnvironmentTruth,
    pub altitude_ft: f64,
    pub on_ground: bool,
    /// Per engine, 1-4: fan speed as a fraction of take-off N1, and
    /// whether the core is turning and lit.
    pub engine_n1_frac: [f64; 4],
    pub engine_running: [bool; 4],
    /// Per engine: bleed air available at the pylon, from this crate's own
    /// engine model (`physics::engine`'s IP8/HP6 port outputs).
    pub engine_bleed_pressure_pa: [f64; 4],
    pub engine_bleed_temp_k: [f64; 4],
    /// APU: running, and its bleed available at the valve.
    pub apu_running: bool,
    pub apu_bleed_pressure_pa: f64,
    /// Bus voltages FlyByWire's own electrical system publishes, so the
    /// areas that consume power agree with what the crew sees on the ELEC
    /// page rather than running a second, disagreeing electrical model.
    pub ac_bus_volts: [f64; 4],
    pub dc_bus_volts: [f64; 2],
    /// Hydraulic system pressures, green and yellow, Pa.
    pub hydraulic_pressure_pa: [f64; 2],
}

impl Default for Truth {
    /// A cold aircraft on the ground at ISA sea level: what every area
    /// sees before the plugin has filled a single frame. Not all-zero --
    /// zero ambient pressure is a vacuum, and several areas divide by it.
    fn default() -> Self {
        Self {
            dt_s: 1.0 / 30.0,
            environment: EnvironmentTruth {
                sat_c: 15.0,
                leading_edge_c: 15.0,
                ambient_pressure_pa: 101_325.0,
                tas_ms: 0.0,
                precipitation_on_aircraft_ratio: 0.0,
                weather: None,
            },
            altitude_ft: 0.0,
            on_ground: true,
            engine_n1_frac: [0.0; 4],
            engine_running: [false; 4],
            engine_bleed_pressure_pa: [101_325.0; 4],
            engine_bleed_temp_k: [288.15; 4],
            apu_running: false,
            apu_bleed_pressure_pa: 101_325.0,
            ac_bus_volts: [0.0; 4],
            dc_bus_volts: [0.0; 2],
            hydraulic_pressure_pa: [0.0; 2],
        }
    }
}

/// How badly each failure is armed, by `deep::api` failure id.
///
/// Absent means healthy. Every magnitude is clamped to 0..1 on the way in,
/// so an area can use it as a fraction without checking.
#[derive(Clone, Debug, Default)]
pub struct Faults(BTreeMap<u64, f64>);

impl Faults {
    pub fn from_pairs(pairs: impl IntoIterator<Item = (u64, f64)>) -> Self {
        Self(pairs.into_iter().map(|(id, m)| (id, m.clamp(0.0, 1.0))).collect())
    }

    /// This failure's magnitude, 0 if it is not armed at all.
    pub fn get(&self, id: u64) -> f64 {
        self.0.get(&id).copied().unwrap_or(0.0)
    }

    /// Whether anything at all is armed -- areas with an expensive
    /// healthy-case shortcut can check this first.
    pub fn any(&self) -> bool {
        self.0.values().any(|&m| m > 0.0)
    }
}

/// One area's live system.
pub trait Area {
    /// A short name for diagnostics and the frame-time breakdown.
    fn name(&self) -> &'static str;

    /// Advance this area by `truth.dt_s`, with the failures armed as
    /// given.
    fn tick(&mut self, truth: &Truth, faults: &Faults);

    /// Publish what the rest of the aircraft can see: the variables this
    /// area's `registry.rs` names in its ECAM triggers, plus anything the
    /// EFB's Study pages read. Called after every area has ticked.
    fn publish(&self, out: &mut dyn FnMut(&str, f64));
}

/// Every area's live system, owned in one place.
///
/// Areas are added here as each grows a live system; an area with no
/// entry yet is simply not stepped, which is why this is a list rather
/// than eighteen named fields.
#[derive(Default)]
pub struct Deep {
    areas: Vec<Box<dyn Area>>,
    truth: Truth,
}

impl Deep {
    /// Every area that has a live system, constructed cold.
    pub fn new() -> Self {
        Self { areas: Vec::new(), truth: Truth::default() }
    }

    pub fn with_area(mut self, area: Box<dyn Area>) -> Self {
        self.areas.push(area);
        self
    }

    /// The truth as of the last tick, for anything that needs to read back
    /// what the areas were given.
    pub fn truth(&self) -> &Truth {
        &self.truth
    }

    /// Step every area, then publish. `publish` runs after every area has
    /// ticked so that no area can see half a frame.
    pub fn tick(&mut self, truth: Truth, faults: &Faults, out: &mut dyn FnMut(&str, f64)) {
        self.truth = truth;
        for area in &mut self.areas {
            area.tick(&self.truth, faults);
        }
        for area in &self.areas {
            area.publish(out);
        }
    }

    pub fn area_names(&self) -> Vec<&'static str> {
        self.areas.iter().map(|a| a.name()).collect()
    }

    /// Every variable name the areas publish, without stepping anything.
    ///
    /// `Plugin` calls this once at startup so it can resolve each name to a
    /// `VariableIdentifier` there rather than in the frame loop: at 30-60 Hz
    /// a per-name `Vars::get` would allocate the name and its `A32NX_`
    /// prefix every frame, for every published value, forever.
    ///
    /// Publishing is a pure read of an area's own state (the trait takes
    /// `&self`), so calling it on cold areas has no effect on them beyond
    /// the values it reports, which are discarded here.
    pub fn published_names(&self) -> Vec<String> {
        let mut names = Vec::new();
        for area in &self.areas {
            area.publish(&mut |name, _| names.push(name.to_owned()));
        }
        names
    }
}

/// Every area that has grown a live system, in the order they are stepped.
///
/// Each area publishes `pub fn live_system() -> Box<dyn Area>` from its own
/// `src/deep/<area>/live.rs`; an area that has not grown one yet is simply
/// absent from this list and is not stepped (see [`Deep`]'s own doc). This
/// is the one place the list lives, so adding an area is one line here and
/// nothing in `lib.rs` changes.
///
/// Ordering is deliberate only in that it is fixed: an area reads the
/// previous frame's value of anything another area publishes, so no order
/// here can be wrong (see this module's "Ordering" note). The list is
/// alphabetical so that a new area has an obvious place to go.
pub fn all_areas() -> Deep {
    Deep::new()
        .with_area(crate::deep::apu::live::live_system())
        .with_area(crate::deep::avionics_network::live::live_system())
        .with_area(crate::deep::breakers::live::live_system())
        .with_area(crate::deep::cabin::live::live_system())
        .with_area(crate::deep::electrical::live::live_system())
        .with_area(crate::deep::engine_accessories::live::live_system())
        .with_area(crate::deep::environment::live::live_system())
        .with_area(crate::deep::fire_ice::live::live_system())
        .with_area(crate::deep::flight_controls::live::live_system())
        .with_area(crate::deep::fuel::live::live_system())
        .with_area(crate::deep::gear_structure::live::live_system())
        .with_area(crate::deep::hydraulics::live::live_system())
        .with_area(crate::deep::pneumatic_ducts::live::live_system())
        .with_area(crate::deep::sensors::live::live_system())
        .with_area(crate::deep::thermal_zones::live::live_system())
        .with_area(crate::deep::wiring::live::live_system())
    // Every area under `deep/` has one now. A new area adds its own line
    // here, alphabetically, and nothing else in the plugin changes.
}

#[cfg(test)]
mod tests {
    use super::*;

    #[derive(Default)]
    struct Counter {
        ticks: usize,
        last_dt: f64,
        leak: f64,
    }

    impl Area for Counter {
        fn name(&self) -> &'static str {
            "counter"
        }
        fn tick(&mut self, truth: &Truth, faults: &Faults) {
            self.ticks += 1;
            self.last_dt = truth.dt_s;
            self.leak = faults.get(11_021_001);
        }
        fn publish(&self, out: &mut dyn FnMut(&str, f64)) {
            out("TEST_COUNTER_TICKS", self.ticks as f64);
            out("TEST_COUNTER_LEAK", self.leak);
        }
    }

    #[test]
    fn a_default_truth_is_a_cold_aircraft_in_real_air_not_a_vacuum() {
        // Several areas divide by ambient pressure or density; the
        // before-the-first-frame state has to be somewhere an aircraft
        // could actually be.
        let t = Truth::default();
        assert!(t.environment.ambient_pressure_pa > 90_000.0);
        assert!(t.dt_s > 0.0);
        assert!(t.on_ground && !t.apu_running);
        assert_eq!(t.engine_running, [false; 4]);
    }

    #[test]
    fn an_area_is_ticked_with_the_truth_and_faults_and_then_publishes() {
        let mut deep = Deep::new().with_area(Box::new(Counter::default()));
        let faults = Faults::from_pairs([(11_021_001, 0.4)]);
        let mut published = BTreeMap::new();
        let truth = Truth { dt_s: 0.05, ..Truth::default() };
        deep.tick(truth, &faults, &mut |name, value| {
            published.insert(name.to_string(), value);
        });
        assert_eq!(published.get("TEST_COUNTER_TICKS"), Some(&1.0));
        assert_eq!(published.get("TEST_COUNTER_LEAK"), Some(&0.4));
        assert_eq!(deep.truth().dt_s, 0.05);
        assert_eq!(deep.area_names(), vec!["counter"]);
    }

    #[test]
    fn every_assembled_area_has_its_own_name_and_can_be_stepped_cold() {
        // The assembly is a hand-kept list, so the two things that can go
        // wrong with it are an area added twice and an area that cannot
        // survive its first frame (dt clamped to a real value, a cold
        // aircraft in real air, nothing armed).
        let mut deep = all_areas();
        let names = deep.area_names();
        let unique: std::collections::BTreeSet<_> = names.iter().collect();
        assert_eq!(unique.len(), names.len(), "an area is listed twice in all_areas(): {names:?}");
        let mut published = BTreeMap::new();
        deep.tick(Truth::default(), &Faults::default(), &mut |name, value| {
            assert!(value.is_finite(), "{name} published {value} on the first frame");
            published.insert(name.to_owned(), value);
        });
    }

    #[test]
    fn published_names_are_what_the_areas_actually_publish() {
        // The plugin resolves these to variable ids once at startup, so a
        // name reported here that the areas never publish would be a dead
        // variable, and one they publish but do not report would be
        // resolved in the frame loop instead.
        let deep = all_areas();
        let reported: std::collections::BTreeSet<String> = deep.published_names().into_iter().collect();
        let mut actual = std::collections::BTreeSet::new();
        for area in &deep.areas {
            area.publish(&mut |name, _| {
                actual.insert(name.to_owned());
            });
        }
        assert_eq!(reported, actual);
    }

    #[test]
    fn an_unarmed_failure_reads_healthy_and_magnitudes_are_bounded() {
        let f = Faults::from_pairs([(1, 2.5), (2, -1.0)]);
        assert_eq!(f.get(999), 0.0, "a failure nobody armed must read healthy, not absent");
        assert_eq!(f.get(1), 1.0, "magnitudes are fractions and cannot exceed fully failed");
        assert_eq!(f.get(2), 0.0);
        assert!(f.any());
        assert!(!Faults::default().any());
    }
}


