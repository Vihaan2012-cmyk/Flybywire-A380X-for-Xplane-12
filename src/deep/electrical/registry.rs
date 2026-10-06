//! Registers every physical part, failure and ECAM alert this area's models
//! support with `crate::deep::api::Registry` (`docs/deep/BRIEF.md`'s
//! "Registering failures, components and ECAM alerts" section).
//!
//! Rather than hand-transcribing ~600 components/failures (every load,
//! breaker, contactor, diode and source instance this area's own
//! `loads::build`/`sources::Wiring::build` already construct), this module
//! builds one real `Network` + `Catalog` + `Wiring` the same way any other
//! caller would, then walks its actual contents -- so the registry can
//! never drift out of sync with the real catalogue (adding a load in
//! `loads.rs` registers itself automatically, with no second list to
//! remember to update).
//!
//! Every component gets one `ComponentDef` per real instance (the brief's
//! own "expanded per instance" instruction) with its model's own fault
//! fields as 0..1 `ParamDef`s, and one `FailureDef` per fault field, named
//! sequentially within its own ATA chapter.

use std::collections::HashMap;

use crate::deep::api::*;

use super::loads;
use super::network::Network;
use super::sources;

/// Sequential-per-ATA-chapter failure numbering (the brief's own "number n
/// sequentially per ATA chapter" rule).
struct AtaCounters(HashMap<u16, u16>);
impl AtaCounters {
    fn new() -> Self {
        Self(HashMap::new())
    }
    fn next(&mut self, ata: u16) -> u16 {
        let n = self.0.entry(ata).or_insert(0);
        *n += 1;
        *n
    }
}

fn param(name: &str, meaning: &str, healthy: f64) -> ParamDef {
    ParamDef { name: name.to_string(), meaning: meaning.to_string(), healthy }
}

/// Registers one component with an arbitrary set of `(field, meaning,
/// magnitude, effect)` fault channels, each becoming its own `ParamDef` +
/// `FailureDef` pair. `model_field` is built as
/// `deep::electrical::<module>::<Type>.faults.<field>` for every channel.
fn register_component(r: &mut Registry, counters: &mut AtaCounters, ata: u16, comp_id: String, name: String, model_type: &str, channels: &[(&str, &str, &str, &str)]) {
    let mut params = Vec::with_capacity(channels.len());
    let mut ids = Vec::with_capacity(channels.len());
    for &(field, meaning, _magnitude, _effect) in channels {
        params.push(param(field, meaning, 0.0));
        ids.push(failure_id(Area::Electrical, ata, counters.next(ata)));
    }
    r.component(ComponentDef { id: comp_id.clone(), area: Area::Electrical, ata, name: name.clone(), params, failures: ids.clone() });
    for (i, &(field, _meaning, magnitude, effect)) in channels.iter().enumerate() {
        r.failure(FailureDef {
            id: ids[i],
            area: Area::Electrical,
            ata,
            name: format!("{name}: {}", field.replace('_', " ")),
            component: comp_id.clone(),
            model_field: format!("deep::electrical::{model_type}.faults.{field}"),
            magnitude: magnitude.to_string(),
            effect: effect.to_string(),
        });
    }
}

/// The four fault channels every catalogue `network::Load` carries
/// (`network::LoadFaults`'s own doc has the full physical description).
const LOAD_CHANNELS: [(&str, &str, &str, &str); 4] = [
    ("open_circuit", "internal path opens: 0 healthy .. 1 fully open", "0..1, fraction of the load's own internal path opened", "draws proportionally less current and delivers proportionally less function; no current/no function at 1.0"),
    ("short_to_ground", "wiring/internal short limited by feeder wiring resistance: 0 none .. 1 dead short", "0..1, short severity; current = severity * bus_voltage / wiring_resistance_ohm", "extra unregulated current on top of the load's own demand, can overload its breaker or (if the breaker fails to trip) its bus feeder"),
    ("high_resistance", "degrading connection/winding, draws more current for the same output: 0 none .. 1 up to 50% extra", "0..1, fraction of a 50% extra-current ceiling", "load draws more current than rated for the same useful output, dissipated as heat inside the load"),
    ("intermittent", "chafed/loose connection dropout: 0 none .. 1 up to 2 Hz", "0..1, dropout rate up to 2 Hz", "the load drops off the bus at a rate proportional to severity, independent of breaker/bus health"),
];

/// The two fault channels every `network::Breaker` carries.
const BREAKER_CHANNELS: [(&str, &str, &str, &str); 2] = [
    ("fails_to_trip", "the thermal/magnetic element reaches trip threshold but the contacts never open: 0 none .. 1 always fails", "0..1, probability the trip attempt fails, re-evaluated each attempt", "the breaker stays closed under an overload/short that should have opened it, letting the fault continue to heat the bus/wiring downstream"),
    ("nuisance_trip", "trips with no real overload at all (a marginal element/loose connection): 0 none .. 1 trips in ~5 s with zero load", "0..1, continuous extra I^2t heat added regardless of current", "the breaker opens and de-energises its load/bus segment despite no genuine fault"),
];

/// The two fault channels every `network::Contactor` carries.
const CONTACTOR_CHANNELS: [(&str, &str, &str, &str); 2] = [
    ("fails_to_close", "commanded closed but the contacts never make: 0 none .. 1 always fails", "0..1, probability the close attempt fails, re-evaluated each tick commanded closed", "the bus/path this contactor should have connected stays de-energised"),
    ("welded_closed", "contacts fused together, closes and stays closed regardless of command: 0 none .. 1 always welded", "0..1, probability of the welded-shut state each tick", "the bus/path stays connected even when it should have been isolated (e.g. a bus tie that should have opened on a fault)"),
];

pub fn register(r: &mut Registry) {
    let mut counters = AtaCounters::new();
    let mut net = Network::new();
    let catalog = loads::build(&mut net);
    sources::Wiring::build(&mut net, 15.0);

    // ---- Every catalogue load: one component + 4 failures each.
    for load in &net.loads {
        let comp_id = format!("{:02}_elec.{}", load.spec.ata, load.spec.id);
        register_component(r, &mut counters, load.spec.ata, comp_id, load.spec.name.to_string(), "network::Load", &LOAD_CHANNELS);
    }

    // ---- Every breaker: one component + 2 failures each. ATA chapter
    // taken from the bus it protects is not meaningful (a breaker has no
    // ATA of its own beyond its load's); use ATA 24 (electrical) for every
    // breaker/contactor, matching a real aircraft's own convention that the
    // *protection* hardware itself is an ATA24 part regardless of what it
    // feeds.
    for b in &net.breakers {
        let comp_id = format!("24_elec.bkr.{}", b.id);
        register_component(r, &mut counters, 24, comp_id, format!("Breaker {}", b.id), "network::Breaker", &BREAKER_CHANNELS);
    }

    // ---- Every contactor: one component + 2 failures each.
    for c in &net.contactors {
        let comp_id = format!("24_elec.contactor.{}", c.id);
        register_component(r, &mut counters, 24, comp_id, format!("Contactor {}", c.id), "network::Contactor", &CONTACTOR_CHANNELS);
    }

    // ---- Every diode: one component + 1 failure each.
    for d in &net.diodes {
        let comp_id = format!("24_elec.diode.{}", d.id);
        register_component(r, &mut counters, 24, comp_id, format!("Diode {}", d.id), "network::Diode", &[("open_circuit", "junction destroyed by an over-current/reverse surge: 0 none .. 1 always open", "0..1, probability of the open-junction state each tick", "the one-way path this diode provided is gone; whatever it fed can no longer be reached through it")]);
    }

    // ---- Every bus: one component + 1 (short-to-ground) failure each.
    for bus in &net.buses {
        let comp_id = format!("24_elec.bus.{}", bus.id.label());
        register_component(r, &mut counters, 24, comp_id, format!("{} bus", bus.id.label()), "network::Bus", &[("short_to_ground", "a short from the busbar itself to structure, independent of any one load on it: 0 none .. 1 dead short", "0..1, short severity; conductance = severity / BUS_FAULT_RESISTANCE_OHM (0.03 ohm)", "collapses the bus's own voltage and can overload whatever feeder/tie breaker protects it")]);
    }

    // ---- Sources: one component per real instance, their own fault fields.
    for n in 1..=4u16 {
        let comp_id = format!("24_elec.vfg-{n}");
        register_component(
            r,
            &mut counters,
            24,
            comp_id,
            format!("GEN {n} (VFG)"),
            "sources::Vfg",
            &[
                ("winding_degradation", "aged/shorted stator windings: 0 healthy .. 1 reactance at 4x", "0..1, series reactance grows toward 4x its healthy value", "terminal voltage sags harder under load; a fully degraded machine may never reach rated voltage"),
                ("regulator_drift", "GCU voltage-regulation drift: 0 none .. 1 full +8V high-voltage drift", "0..1 (magnitude of a high-voltage drift; the model field is signed -1..1 in code)", "terminal no-load voltage drifts away from 115 V, feeding an over/under-voltage condition to everything on its bus"),
            ],
        );
    }
    for n in 1..=2u16 {
        let comp_id = format!("24_elec.apu-gen-{n}");
        register_component(
            r,
            &mut counters,
            24,
            comp_id,
            format!("APU GEN {n}"),
            "sources::ApuGenerator",
            &[
                ("winding_degradation", "aged/shorted stator windings: 0 healthy .. 1 reactance at 4x", "0..1, series reactance grows toward 4x its healthy value", "terminal voltage sags harder under load"),
                ("regulator_drift", "GCU voltage-regulation drift: 0 none .. 1 full +8V high-voltage drift", "0..1 (magnitude; signed -1..1 in code)", "terminal no-load voltage drifts away from 115 V"),
            ],
        );
    }
    for (n, id) in [(1, "tr-1"), (2, "tr-2")].into_iter().chain([(3, "tr-ess"), (4, "tr-apu")]) {
        let comp_id = format!("24_elec.{id}");
        register_component(
            r,
            &mut counters,
            24,
            comp_id,
            format!("TR {n}"),
            "sources::Tru",
            &[("winding_degradation", "winding/diode-bridge degradation: 0 healthy .. 1 resistance at 0.054 ohm (4x)", "0..1, internal resistance grows from 0.0135 to 0.054 ohm", "DC output sags harder under load, and the TRU runs hotter for the same delivered power")],
        );
    }
    for n in 1..=2u16 {
        let comp_id = format!("24_elec.bat-{n}");
        register_component(
            r,
            &mut counters,
            24,
            comp_id,
            format!("BAT {n}"),
            "sources::Battery",
            &[
                ("capacity_fade", "permanently degraded cell capacity: 0 healthy .. 1 up to 70% capacity lost", "0..1, fraction of MAX_CAPACITY_FADE (0.7) capacity lost", "less usable charge before the battery reads empty, shorter time-to-empty under the same load"),
                ("resistance_growth", "aged internal resistance: 0 healthy .. 1 up to 3x", "0..1, internal resistance grows toward 3x (AGED_RESISTANCE_MULTIPLIER)", "terminal voltage sags harder under load, less real power deliverable before hitting the max-power-transfer ceiling"),
            ],
        );
    }
    register_component(r, &mut counters, 24, "24_elec.static-inv".to_string(), "STATIC INVERTER".to_string(), "sources::StaticInverter", &[("efficiency_loss", "degraded switching efficiency: 0 healthy (85%) .. 1 floor (30%)", "0..1, efficiency interpolates from 0.85 down to 0.30", "less real power deliverable to AC_EMER for the same battery input, faster battery drain in an emergency configuration")]);
    register_component(r, &mut counters, 24, "24_elec.rat".to_string(), "RAT".to_string(), "sources::Rat", &[("jammed", "turbine fails to fully deploy/partially seized: 0 healthy .. 1 no power even when deployed", "0..1, fraction of aerodynamic power lost", "less (or, at 1.0, no) emergency electrical power available from the RAT in an all-generation-lost configuration")]);
    register_component(r, &mut counters, 24, "24_elec.gpu".to_string(), "GPU (ground power)".to_string(), "sources::GroundPower", &[("weak_cart", "a weak/miswired ground cart: 0 healthy .. 1 up to 5x reactance", "0..1, series reactance grows toward 5x", "the main AC buses the four external power contactors feed sag harder under load while on ground power")]);

    // ---- E-ELEC additions: `MISC_FAULTS`' own eighteen computer-health/
    // monitoring/lubrication-leak components (ata24 unwired-id pass,
    // `E:/fbw-debug/ecam/E-ELEC-DESIGN.md`). One component + one failure
    // each, following the same `register_component` shape as every source
    // above; `super::live::MISC_FAULTS` is the single list both this
    // registration and `super::live::route_failures` read, so they cannot
    // drift apart.
    for &(suffix, name, meaning) in &super::live::MISC_FAULTS {
        let comp_id = format!("24_elec.misc.{suffix}");
        register_component(r, &mut counters, 24, comp_id, name.to_string(), "misc::Health", &[("fault", meaning, "0..1 (boolean for every entry but the two drive-oil-leak channels, which are continuous, 0..1, exactly as `apu::oil::OilFaults::leak` already is)", "see this failure's own name/meaning for the specific effect")]);
    }

    // ---------------------------------------------------------------------
    // ECAM alerts. Trigger/procedure variable names follow this plugin's
    // existing `ELEC_*` convention (`circuits.rs`/`breakers.rs`); none of
    // them are published yet by this model (it is a self-contained,
    // integration-pending crate per this workstream's own hard rules) --
    // see PROGRESS.md's "Vars this model must publish" section for the
    // full list an integration layer needs to wire up.

    r.alert(
        EcamAlert::new("ELEC_GEN_1_FAULT", 24, "ELEC GEN 1 FAULT", Level::Caution, var("ELEC_GEN_1_FAULT").on())
            .confirm(1.0)
            .inhibit(&[Phase::LiftOff, Phase::Above80Kt])
            .step(line("GEN 1", "OFF").done(var("ELEC_GEN_1_PB_ON").off()))
            .step(line("IF SMOKE OR FUMES:", "").only_if(var("ELEC_GEN_1_SMOKE").on()))
            .status_line("GEN 1 FAULT")
            .inop_sys("GEN 1"),
    );
    r.alert(
        EcamAlert::new("ELEC_APU_GEN_FAULT", 24, "ELEC APU GEN FAULT", Level::Caution, var("ELEC_APU_GEN_FAULT").on())
            .confirm(1.0)
            .step(line("APU GEN", "OFF").done(var("ELEC_APU_GEN_PB_ON").off()))
            .status_line("APU GEN FAULT"),
    );
    r.alert(
        EcamAlert::new("ELEC_TR_1_FAULT", 24, "ELEC TR 1 FAULT", Level::Caution, var("ELEC_TR_1_FAULT").on())
            .confirm(1.0)
            .step(line("ELEC", "MONITOR DC BUS 1").colour("green"))
            .status_line("TR 1 FAULT")
            .inop_sys("TR 1"),
    );
    r.alert(
        EcamAlert::new("ELEC_BAT_1_FAULT", 24, "ELEC BAT 1 FAULT", Level::Caution, var("ELEC_BAT_1_FAULT").on())
            .confirm(2.0)
            .step(line("BAT 1", "OFF").done(var("ELEC_BAT_1_PB_ON").off()))
            .status_line("BAT 1 FAULT"),
    );
    r.alert(
        EcamAlert::new("ELEC_AC_BUS_1_FAULT", 24, "ELEC AC BUS 1 FAULT", Level::Warning, all(vec![var("ELEC_AC_1_BUS_POTENTIAL").lt(90.0), var("ELEC_AC_1_BUS_IS_POWERED").off()]))
            .confirm(0.5)
            .inhibit(&[Phase::LiftOff, Phase::Touchdown])
            .step(line("AC BUS 1", "CHECK").colour("green"))
            .status_line("AC BUS 1 FAULT")
            .inop_sys("AC BUS 1 consumers"),
    );
    r.alert(
        EcamAlert::new("ELEC_EMER_CONFIG", 24, "ELEC EMER CONFIG", Level::Warning, all(vec![var("ELEC_AC_1_BUS_IS_POWERED").off(), var("ELEC_AC_2_BUS_IS_POWERED").off(), var("ELEC_AC_3_BUS_IS_POWERED").off(), var("ELEC_AC_4_BUS_IS_POWERED").off()]))
            .confirm(3.0)
            .step(line("RAT", "MAN ON").done(var("ELEC_RAT_DEPLOYED").on()))
            .step(line("ELEC EMER CONFIG PROC", "APPLY").after(5.0))
            .status_line("EMER CONFIG")
            .inop_sys("Non-essential AC/DC buses"),
    );
    r.alert(
        EcamAlert::new("ELEC_GALLEY_SHED", 24, "ELEC GALLEY SHED", Level::Advisory, var("ELEC_GALLEY_SHED_ACTIVE").on()).status_line("GALLEY SHED"),
    );
    r.alert(
        EcamAlert::new("ELEC_COMMERCIAL_SHED", 24, "ELEC COMMERCIAL SHED", Level::Advisory, var("ELEC_COMMERCIAL_SHED_ACTIVE").on()).status_line("COML SHED"),
    );
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn registering_everything_produces_no_validation_errors() {
        let mut r = Registry::default();
        register(&mut r);
        let errors = r.validate_area();
        assert!(errors.is_empty(), "registry validation errors: {errors:?}");
    }

    #[test]
    fn registering_covers_a_large_real_catalogue_with_no_duplicate_ids() {
        let mut r = Registry::default();
        register(&mut r);
        assert!(r.components.len() > 200, "expected a substantial component set, got {}", r.components.len());
        assert!(r.failures.len() > 400, "expected a substantial failure set, got {}", r.failures.len());
        let mut failure_ids: Vec<u64> = r.failures.iter().map(|f| f.id).collect();
        let before = failure_ids.len();
        failure_ids.sort_unstable();
        failure_ids.dedup();
        assert_eq!(failure_ids.len(), before, "duplicate failure id");
        let mut comp_ids: Vec<&str> = r.components.iter().map(|c| c.id.as_str()).collect();
        let before_c = comp_ids.len();
        comp_ids.sort_unstable();
        comp_ids.dedup();
        assert_eq!(comp_ids.len(), before_c, "duplicate component id");
    }

    #[test]
    fn every_failure_id_carries_the_electrical_area_and_its_own_ata() {
        let mut r = Registry::default();
        register(&mut r);
        for f in &r.failures {
            assert_eq!(f.id / 1_000_000, Area::Electrical as u64);
            assert_eq!(f.id / 1_000 % 1_000, f.ata as u64);
        }
    }

    #[test]
    fn every_alert_names_only_registered_failures_or_none() {
        let mut r = Registry::default();
        register(&mut r);
        // The curated ECAM alerts above deliberately do not call
        // `.raised_by(...)` (their triggers are over published Vars this
        // model does not publish yet, not a specific failure id -- see
        // PROGRESS.md); confirm that choice does not trip validation.
        for a in &r.alerts {
            for id in &a.failures {
                assert!(r.failures.iter().any(|f| f.id == *id));
            }
        }
    }
}
