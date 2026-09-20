//! Cross-catalogue integration test: verifies every `protected_load` id in
//! [`super::catalog`] actually resolves against `deep::electrical`'s own
//! live load catalogue, and that every `deep::electrical` load has a
//! breaker protecting it here. Requested directly (not a self-initiated
//! exception to `docs/deep/BRIEF.md` hard rule 2's self-containment rule):
//! the coordinator explicitly asked for "an integration test module
//! (compiled at the lead's build) that cross-checks every protected_load id
//! against deep::electrical's real catalogue."
//!
//! **This module only compiles once the lead's build wires both areas into
//! `src/deep/mod.rs`** (`pub mod electrical;` and `pub mod breakers;`) --
//! until then, neither `deep::electrical` nor `deep::breakers` is reachable
//! from the crate root at all, so nothing in this crate references this
//! file's `crate::deep::electrical` path yet and it cannot be compiled in
//! isolation the way every other file in this directory can. Declared as
//! `#[cfg(test)] mod integration_test;` from `mod.rs` so it only affects
//! `cargo test`, never a normal build, and never runs before the lead
//! actually wires both areas in (a bare `cargo test` on today's
//! `deep::mod.rs`, which only declares `pub mod api;`, never reaches this
//! file's own module tree at all).
//!
//! If `deep::electrical`'s own module layout changes before that wiring
//! happens (e.g. `loads::build` is renamed, or moves under a different
//! path), update the two `use` paths below to match -- this file was
//! written against `src/deep/electrical/mod.rs`'s `pub mod loads; pub mod
//! network;` and `network::Network::{new, loads}` / `loads::build`, all
//! confirmed `pub` by reading that file this session.

#[cfg(test)]
mod tests {
    use std::collections::HashSet;

    use crate::deep::breakers::catalog;
    use crate::deep::electrical::{loads, network::Network};

    fn real_load_ids() -> HashSet<&'static str> {
        let mut net = Network::new();
        loads::build(&mut net);
        // `Network::loads` is `Vec<network::Load>`, each carrying its own
        // `spec: LoadSpec` with a `'static` `id` -- leaked once here is
        // unnecessary since `LoadSpec::id` is already `&'static str`, so
        // this collects those references directly (no allocation).
        net.loads.iter().map(|l| l.spec.id).collect()
    }

    #[test]
    fn every_protected_load_id_resolves_against_deep_electricals_real_catalogue() {
        let real_ids = real_load_ids();
        let mut dangling: Vec<&str> = Vec::new();
        for def in catalog::all() {
            if let Some(load_id) = def.protected_load {
                if !real_ids.contains(load_id) {
                    dangling.push(def.id);
                }
            }
        }
        assert!(dangling.is_empty(), "breakers protecting a load id deep::electrical no longer defines (renamed/removed upstream, this catalogue needs a re-sync pass): {dangling:?}");
    }

    #[test]
    fn every_deep_electrical_load_has_a_protecting_breaker_in_this_catalogue() {
        let real_ids = real_load_ids();
        let protected: HashSet<&str> = catalog::all().iter().filter_map(|d| d.protected_load).collect();
        let mut unprotected: Vec<&str> = real_ids.into_iter().filter(|id| !protected.contains(id)).collect();
        unprotected.sort_unstable();
        assert!(unprotected.is_empty(), "deep::electrical loads with no breaker in this catalogue yet (new loads landed upstream, this catalogue needs a re-sync pass): {unprotected:?}");
    }

    /// The reverse direction of the coverage check above, and the one that
    /// is *not* satisfied today.
    ///
    /// This catalogue is deliberately larger than `deep::electrical`'s load
    /// set: it carries real A380 breakers whose consumers that area does not
    /// model yet (each fuel pump's own control channel, the engine ignition
    /// exciters, the cargo door actuator controls, the APU ECU channels, the
    /// crew oxygen shutoff, ...). Those entries honestly carry
    /// `protected_load: None` rather than a fabricated load id -- see
    /// `catalog::tests::group_2_equipment_has_no_fabricated_load_and_is_honestly_documented`.
    ///
    /// That is a real, known gap in the *electrical load model*, not a bug in
    /// this catalogue, so it is named here instead of being asserted away.
    /// Pinning the count per ATA chapter means the gap can only move
    /// deliberately: modelling one of these consumers in
    /// `deep::electrical::loads` and linking it to its breaker lowers a
    /// number below, and adding a breaker with no modelled consumer raises
    /// one. Either way this test says exactly which chapter moved.
    #[test]
    fn breakers_protecting_no_modelled_load_are_a_known_named_gap() {
        // 128 of the catalogue's 399 breakers protect a consumer
        // `deep::electrical` has not modelled yet, by ATA chapter:
        const EXPECTED: [(u16, usize); 14] =
            [(21, 8), (23, 6), (24, 6), (26, 6), (28, 60), (29, 3), (31, 3), (33, 3), (35, 4), (36, 4), (49, 5), (52, 4), (73, 8), (74, 8)];
        let expected_total: usize = EXPECTED.iter().map(|(_, n)| n).sum();
        assert_eq!(expected_total, 128, "the per-chapter table above must add up to the stated total");

        let mut actual: std::collections::BTreeMap<u16, usize> = std::collections::BTreeMap::new();
        let mut total = 0usize;
        for def in catalog::all() {
            if def.protected_load.is_none() {
                *actual.entry(def.ata).or_insert(0) += 1;
                total += 1;
            }
        }
        let expected: std::collections::BTreeMap<u16, usize> = EXPECTED.iter().copied().collect();
        assert_eq!(
            actual, expected,
            "the set of breakers protecting no modelled deep::electrical load has moved; if that was deliberate (a consumer was modelled, or a new unmodelled breaker was added) update the table in this test, chapter by chapter"
        );
        assert_eq!(total, expected_total);

        // Every one of them must still be a real, described breaker -- an
        // unmodelled consumer is allowed, an undocumented one is not.
        for def in catalog::all() {
            if def.protected_load.is_none() {
                assert!(!def.consumer.is_empty(), "{} protects no modelled load and does not say what it does feed", def.id);
                assert!(!def.basis.is_empty(), "{} protects no modelled load and cites no basis for its rating", def.id);
            }
        }
    }

    #[test]
    fn every_protected_load_has_either_one_breaker_or_exactly_as_many_as_its_own_real_feed_count() {
        // A single-feed load has exactly one breaker claiming it; a real
        // dual-fed load (`network::Load::feeds`, `add_dual` in
        // `loads.rs`) has exactly two -- never any other count, which
        // would mean either a missing feed's breaker or two breakers
        // racing to gate the same physical feed.
        let mut net = Network::new();
        loads::build(&mut net);
        let mut claim_counts: std::collections::HashMap<&str, usize> = std::collections::HashMap::new();
        for def in catalog::all() {
            if let Some(load_id) = def.protected_load {
                *claim_counts.entry(load_id).or_insert(0) += 1;
            }
        }
        for load in &net.loads {
            let expected = load.feeds.len().max(1);
            let actual = claim_counts.get(load.spec.id).copied().unwrap_or(0);
            assert_eq!(actual, expected, "load {} has {} real feed(s) but {} breaker(s) claim it here", load.spec.id, expected, actual);
        }
    }
}
