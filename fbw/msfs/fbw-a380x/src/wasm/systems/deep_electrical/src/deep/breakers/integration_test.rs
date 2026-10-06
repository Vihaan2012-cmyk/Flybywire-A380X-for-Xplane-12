#[cfg(test)]
mod tests {
    use std::collections::HashSet;

    use crate::deep::breakers::catalog;
    use crate::deep::electrical::{loads, network::Network};

    fn real_load_ids() -> HashSet<&'static str> {
        let mut net = Network::new();
        loads::build(&mut net);
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
    fn breakers_protecting_no_modelled_load_are_a_known_named_gap() {
        const EXPECTED: [(u16, usize); 1] = [(24, 3)];
        let expected_total: usize = EXPECTED.iter().map(|(_, n)| n).sum();
        assert_eq!(expected_total, 3, "the per-chapter table above must add up to the stated total");

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

        for def in catalog::all() {
            if def.protected_load.is_none() {
                assert!(!def.consumer.is_empty(), "{} protects no modelled load and does not say what it does feed", def.id);
                assert!(!def.basis.is_empty(), "{} protects no modelled load and cites no basis for its rating", def.id);
            }
        }
    }

    #[test]
    fn every_protected_load_has_either_one_breaker_or_exactly_as_many_as_its_own_real_feed_count() {
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
