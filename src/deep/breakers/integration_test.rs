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
