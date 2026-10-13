//! One Study page per `deep` area (src/deep/*), generated entirely from
//! that area's own registration (`deep::api::Registry`, via
//! `deep::registry()`) and its own published variable names (its
//! `live.rs`'s `Area::publish`). Nothing here invents a diagram, a
//! grouping label or a description the registry/live layer does not
//! itself carry -- see `docs/deep/BRIEF.md` and this crate's own "don't
//! fake it" rule, which `web.rs`'s doc comment already states for the
//! rest of the Study tab.
//!
//! Eighteen areas register through `deep::registry()`
//! (`src/deep/mod.rs::registry`); one of them, `integration`, folds its
//! failures/components/alerts into other areas' components but has no
//! `live.rs` of its own, so its page's "published variables" section is
//! honestly empty rather than faked.

use std::collections::{BTreeMap, BTreeSet};
use std::sync::OnceLock;

use crate::deep::api::{Area, ComponentDef, EcamAlert, FailureDef, Registry};

use super::canvas::{group, num, Group};
use super::pages::tint;

/// Every area this Study tree has a page for, in the same order
/// `deep::mod.rs::registry()` registers them (alphabetical, `integration`
/// last since it owns no failures/components of its own -- only
/// contributions and extensions onto the other seventeen).
pub(crate) const AREA_LIST: &[(Area, &str)] = &[
    (Area::Apu, "APU"),
    (Area::AvionicsNetwork, "Avionics Network"),
    (Area::Breakers, "Breakers"),
    (Area::Cabin, "Cabin"),
    (Area::Electrical, "Electrical"),
    (Area::EngineAccessories, "Engine Accessories"),
    (Area::Environment, "Environment"),
    (Area::FireIce, "Fire & Ice"),
    (Area::FlightControls, "Flight Controls"),
    (Area::Fuel, "Fuel"),
    (Area::GearStructure, "Gear & Structure"),
    (Area::Hydraulics, "Hydraulics"),
    (Area::Oxygen, "Oxygen"),
    (Area::PneumaticDucts, "Pneumatic Ducts"),
    (Area::Sensors, "Sensors"),
    (Area::ThermalZones, "Thermal Zones"),
    (Area::Wiring, "Wiring"),
    (Area::Integration, "Integration"),
    (Area::AutoFlight, "Auto Flight"),
    (Area::Communications, "Communications"),
];

/// The whole `deep` registry, built once for this process (it is several
/// thousand definitions -- too much to redo on every `/study/pages`
/// request). Mirrors `web.rs`'s own `deep_components` cache and
/// `catalogue.rs`'s own `registry()`, kept separate from both: this one is
/// read by the native XPLM windows too, which never touch `web.rs`.
pub(crate) fn registry() -> &'static Registry {
    static REGISTRY: OnceLock<Registry> = OnceLock::new();
    REGISTRY.get_or_init(crate::deep::registry)
}

pub(crate) fn components_of(area: Area) -> Vec<&'static ComponentDef> {
    registry().components.iter().filter(|c| c.area == area).collect()
}

pub(crate) fn failures_of(area: Area) -> Vec<&'static FailureDef> {
    registry().failures.iter().filter(|f| f.area == area).collect()
}

/// The ECAM alerts this area's own failures can raise -- not necessarily
/// the alerts this area's `registry.rs` *announces* (`EcamAlert` carries
/// no owning area; only the area that calls `Registry::alert` for a given
/// key is really its owner, and the registry does not record that). What
/// is recoverable, honestly, is which failures reach an alert's `failures`
/// list, and a failure's own id carries its area exactly
/// (`deep::api::failure_id`'s own encoding: id / 1_000_000 is the area
/// code) -- the same encoding `FailureDef::area` is built from. An alert
/// with contributions from several areas (`Registry::contribute`) shows up
/// on more than one area's page, which is correct: a real cause modelled
/// in this area really can raise it.
pub(crate) fn alerts_of(area: Area) -> Vec<&'static EcamAlert> {
    let code = area as u64;
    registry().alerts.iter().filter(|a| a.failures.iter().any(|&id| id / 1_000_000 == code)).collect()
}

/// Every name this area's live system publishes, built once (cold: no
/// simulation is stepped, only `Area::publish`'s own pure read of a
/// freshly-constructed area, the same trick `deep::live::Deep::
/// published_names` uses). `integration` has no live system at all -- it
/// only registers failures/components/contributions -- so it publishes
/// nothing, honestly.
fn live_names_uncached(area: Area) -> Vec<String> {
    let sys: Option<Box<dyn crate::deep::live::Area>> = match area {
        Area::Apu => Some(crate::deep::apu::live::live_system()),
        Area::AutoFlight => Some(crate::deep::autoflight::live::live_system()),
        Area::AvionicsNetwork => Some(crate::deep::avionics_network::live::live_system()),
        Area::Breakers => Some(crate::deep::breakers::live::live_system()),
        Area::Cabin => Some(crate::deep::cabin::live::live_system()),
        Area::Communications => Some(crate::deep::communications::live::live_system()),
        Area::Electrical => Some(crate::deep::electrical::live::live_system()),
        Area::EngineAccessories => Some(crate::deep::engine_accessories::live::live_system()),
        Area::Environment => Some(crate::deep::environment::live::live_system()),
        Area::FireIce => Some(crate::deep::fire_ice::live::live_system()),
        Area::FlightControls => Some(crate::deep::flight_controls::live::live_system()),
        Area::Fuel => Some(crate::deep::fuel::live::live_system()),
        Area::GearStructure => Some(crate::deep::gear_structure::live::live_system()),
        Area::Hydraulics => Some(crate::deep::hydraulics::live::live_system()),
        Area::Oxygen => Some(crate::deep::oxygen::live::live_system()),
        Area::PneumaticDucts => Some(crate::deep::pneumatic_ducts::live::live_system()),
        Area::Sensors => Some(crate::deep::sensors::live::live_system()),
        Area::ThermalZones => Some(crate::deep::thermal_zones::live::live_system()),
        Area::Wiring => Some(crate::deep::wiring::live::live_system()),
        // `Integration` has no live.rs; `FlightModel`/`EngineCore` are
        // `deep::api::Area` codes with no directory under `deep/` at all
        // (not in `AREA_LIST`, never reached here).
        Area::Integration | Area::FlightModel | Area::EngineCore => None,
    };
    let Some(sys) = sys else { return Vec::new() };
    let mut names = Vec::new();
    sys.publish(&mut |n, _| names.push(n.to_owned()));
    names.sort();
    names
}

fn all_live_names() -> &'static BTreeMap<u64, Vec<String>> {
    static NAMES: OnceLock<BTreeMap<u64, Vec<String>>> = OnceLock::new();
    NAMES.get_or_init(|| AREA_LIST.iter().map(|&(a, _)| (a as u64, live_names_uncached(a))).collect())
}

/// Every name this area publishes, live values included at draw/JSON time
/// by whoever reads the snapshot for these fields (the XPLM window's own
/// `Snapshot`, or the web app's `/vars` poll against the names these
/// `Group`s carry).
pub(crate) fn live_names(area: Area) -> Vec<String> {
    all_live_names().get(&(area as u64)).cloned().unwrap_or_default()
}

/// How many of a name's leading `_`-separated tokens are common to every
/// name in the set -- the area's own prefix (`"APU"`, or the two-word
/// `"DEEP", "OXY"`), found from the names themselves rather than hard-coded
/// per area, so a page never claims a prefix its area does not actually use.
fn common_prefix_len(token_lists: &[Vec<&str>]) -> usize {
    let Some(first) = token_lists.first() else { return 0 };
    let mut n = 0;
    while let Some(tok) = first.get(n) {
        if token_lists.iter().all(|toks| toks.get(n) == Some(tok)) {
            n += 1;
        } else {
            break;
        }
    }
    n
}

/// The bucket a name falls into once its area's own leading prefix is
/// skipped: the next token, or the one after that when the next token is a
/// bare instance number (`"..._1_..."`), so `APU_GEN_1_OVERLOAD` and
/// `APU_GEN_2_OVERLOAD` land in the same "GEN" bucket as
/// `APU_GEN_1_POTENTIAL` rather than splitting on the engine number.
fn bucket_key(tokens: &[&str], skip: usize) -> String {
    let mut i = skip;
    while i < tokens.len() && !tokens[i].is_empty() && tokens[i].chars().all(|c| c.is_ascii_digit()) {
        i += 1;
    }
    tokens.get(i).or_else(|| tokens.last()).copied().unwrap_or("VAR").to_string()
}

/// This area's published variables as field-list boxes -- the exact same
/// `Group`/`Field` shape every other Study page (`pages::groups`,
/// `depth::extra`) draws from, so both the XPLM window (`pages::flow`) and
/// the web app (`web::groups_json`) render these without a second code
/// path. Grouped by the name's own structure (its area prefix skipped,
/// then its own next token -- see `bucket_key`), never by an invented
/// category: `integration` publishes nothing, so its list is empty, not
/// filled with placeholder boxes.
pub(crate) fn var_groups(area: Area) -> Vec<Group> {
    let names = live_names(area);
    if names.is_empty() {
        return Vec::new();
    }
    let token_lists: Vec<Vec<&str>> = names.iter().map(|n| n.split('_').collect()).collect();
    let shortest = token_lists.iter().map(Vec::len).min().unwrap_or(1);
    let skip = common_prefix_len(&token_lists).min(shortest.saturating_sub(1));

    let mut buckets: BTreeMap<String, Vec<&String>> = BTreeMap::new();
    for (name, toks) in names.iter().zip(token_lists.iter()) {
        buckets.entry(bucket_key(toks, skip)).or_default().push(name);
    }
    buckets
        .into_iter()
        .enumerate()
        .map(|(i, (key, members))| {
            let fields = members.into_iter().map(|n| num(n, n.clone(), "", 3)).collect();
            group(&key, tint(i), fields)
        })
        .collect()
}

/// Every published name across the eighteen areas, for the "orphaned
/// name" cross-check (`mod.rs`'s own tests): a name is claimed once it
/// appears in some area's [`var_groups`].
pub(crate) fn all_published_names() -> BTreeSet<String> {
    AREA_LIST.iter().flat_map(|&(a, _)| live_names(a)).collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn eighteen_areas_one_of_which_integration_publishes_nothing() {
        // 18 + AutoFlight + Communications (E-AIR-DESIGN.md's ECAM-
        // completeness pass, ATA 22/23).
        assert_eq!(AREA_LIST.len(), 20);
        assert!(live_names(Area::Integration).is_empty(), "integration has no live.rs of its own");
        assert!(!live_names(Area::Apu).is_empty());
    }

    /// Every name this file claims an area publishes is really published by
    /// that area's live system -- the same set `deep::live::all_areas()`
    /// (the plugin's own tick assembly) reports, so a Study page can never
    /// name a variable the running plugin does not actually resolve.
    #[test]
    fn every_claimed_name_matches_the_plugins_own_live_assembly() {
        let mine = all_published_names();
        let real: std::collections::BTreeSet<String> = crate::deep::live::all_areas().published_names().into_iter().collect();
        assert_eq!(mine, real, "deep_page's per-area names must be exactly deep::live::all_areas()'s published set");
    }

    #[test]
    fn var_groups_never_invents_a_box_for_an_area_with_no_live_system() {
        assert!(var_groups(Area::Integration).is_empty());
        assert!(!var_groups(Area::Wiring).is_empty());
    }

    #[test]
    fn components_failures_and_alerts_are_partitioned_by_the_failure_ids_own_area_code() {
        for &(area, _) in AREA_LIST {
            for f in failures_of(area) {
                assert_eq!(f.area, area, "{} carries the wrong area", f.id);
                assert_eq!(f.id / 1_000_000, area as u64, "failure_id must encode its own area");
            }
            for c in components_of(area) {
                assert_eq!(c.area, area);
            }
        }
    }
}
