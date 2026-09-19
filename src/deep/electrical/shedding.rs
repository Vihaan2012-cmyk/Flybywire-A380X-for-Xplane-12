//! Load shedding: galley/commercial shed relays, an emergency-configuration
//! shed, a per-bus power budget, and the transient a bus-tie transfer
//! produces (`docs/deep/BRIEF.md` backlog item 5).
//!
//! **Shed priority** (real Airbus load-management convention): a single
//! generator/bus loss first sheds galley power (ovens/chillers/water
//! heaters -- large, non-essential, restartable loads) to bring demand back
//! within the remaining generators' capacity; a worse condition (most/all
//! main AC generation lost -- an emergency electrical configuration running
//! on battery/static inverter/RAT) sheds commercial cabin load (IFE) too,
//! keeping only [`super::loads::LoadCategory::Essential`] powered. Neither
//! relay touches `Essential` or `Other` loads -- exactly the two categories
//! the brief names.
//!
//! **The bus-transfer transient** is not a separate mechanism bolted on
//! here: it is the same [`super::network::Load`] inrush model every
//! catalogue entry already carries (`loads.rs`'s own `inrush_multiple`/
//! `inrush_duration_s`), which re-triggers for real the moment a load's own
//! bus voltage passes through zero and comes back -- exactly what happens
//! electrically when a bus is re-energised through a tie after its own
//! source drops out. `bus_transfer_produces_an_inrush_transient` below is
//! the test proving that emergent behaviour, not a new model.

use super::loads::{Catalog, LoadCategory};
use super::network::{bernoulli, fnv1a, BusId, Network};

/// `0.0` healthy.
/// - `fails_to_shed`: the relay's own contacts do not open on command (a
///   real stuck/welded-closed shed relay) -- the load stays powered when it
///   should have been shed.
/// - `sheds_when_not_commanded`: the relay's own contacts open on their
///   own (a real spurious/nuisance shed) -- the load is shed when it should
///   not be.
#[derive(Clone, Copy, Debug, Default)]
pub struct ShedRelayFaults {
    pub fails_to_shed: f64,
    pub sheds_when_not_commanded: f64,
}

/// What the caller's own power-management/electrical-health logic decided
/// this tick (this module owns none of that judgement -- see
/// [`power_budget`] for the one piece of decision support it does provide).
pub struct ShedInputs {
    /// A single generator/bus loss (or any other reason the aircraft's own
    /// power budget needs trimming) -- sheds galleys.
    pub galley_shed_commanded: bool,
    /// Most/all main AC generation lost -- sheds galleys *and* commercial
    /// (IFE) load, on top of `galley_shed_commanded`.
    pub emergency_config_commanded: bool,
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ShedOutputs {
    pub galley_shed: bool,
    pub commercial_shed: bool,
}

pub struct SheddingRelays {
    galley_faults: ShedRelayFaults,
    commercial_faults: ShedRelayFaults,
    tick: u64,
}

impl SheddingRelays {
    pub fn new() -> Self {
        Self { galley_faults: ShedRelayFaults::default(), commercial_faults: ShedRelayFaults::default(), tick: 0 }
    }

    pub fn set_galley_faults(&mut self, f: ShedRelayFaults) {
        self.galley_faults = f;
    }
    pub fn set_commercial_faults(&mut self, f: ShedRelayFaults) {
        self.commercial_faults = f;
    }

    fn resolve(&self, id: &'static str, should_shed: bool, faults: ShedRelayFaults) -> bool {
        let seed = fnv1a(id);
        if should_shed {
            !bernoulli(seed ^ 0xAAAA_AAAA_AAAA_AAAA, self.tick, faults.fails_to_shed.clamp(0.0, 1.0))
        } else {
            bernoulli(seed ^ 0xBBBB_BBBB_BBBB_BBBB, self.tick, faults.sheds_when_not_commanded.clamp(0.0, 1.0))
        }
    }

    /// Applies this tick's shed decision directly onto every catalogued
    /// load's own `commanded_on` (`network::Load`'s own per-load on/off
    /// input) -- this module needs no aircraft-specific knowledge of which
    /// load id is which, only `loads::Catalog`'s own category index.
    pub fn step(&mut self, net: &mut Network, catalog: &Catalog, inputs: &ShedInputs) -> ShedOutputs {
        self.tick = self.tick.wrapping_add(1);

        let galley_should_shed = inputs.galley_shed_commanded || inputs.emergency_config_commanded;
        let commercial_should_shed = inputs.emergency_config_commanded;

        let galley_shed = self.resolve("galley-shed-relay", galley_should_shed, self.galley_faults);
        let commercial_shed = self.resolve("commercial-shed-relay", commercial_should_shed, self.commercial_faults);

        for &i in &catalog.galley {
            net.loads[i].commanded_on = !galley_shed;
        }
        for &i in &catalog.commercial {
            net.loads[i].commanded_on = !commercial_shed;
        }

        ShedOutputs { galley_shed, commercial_shed }
    }
}

impl Default for SheddingRelays {
    fn default() -> Self {
        Self::new()
    }
}

// ---------------------------------------------------------------------
// Power budget: real delivered power (post-`Network::step`, so it reflects
// actual solved voltage/current, not nameplate ratings) against a caller-
// supplied source capacity -- the number a real load-management computer
// would compare against before deciding whether to command a shed.

#[derive(Clone, Copy, Debug, Default)]
pub struct PowerBudget {
    pub total_demand_w: f64,
    pub capacity_w: f64,
    pub margin_w: f64,
    pub overloaded: bool,
}

/// Total real power every load in `net` is actually drawing right now (real
/// delivered power, at the network's own solved bus voltages -- excludes
/// fault current, which is wasted heat, not useful load, the same
/// convention `NetworkReport::total_power_w` already uses), against
/// `capacity_w` (whatever generation the caller's own source-health logic
/// currently has available -- e.g. the sum of healthy VFGs' own
/// `RATED_TRUE_POWER_W`).
pub fn power_budget(net: &Network, capacity_w: f64) -> PowerBudget {
    let total_demand_w: f64 = net.loads.iter().map(|l| l.spec.power_factor * l.current_a * net.bus(l.spec.bus).voltage).sum();
    let margin_w = capacity_w - total_demand_w;
    PowerBudget { total_demand_w, capacity_w, margin_w, overloaded: margin_w < 0.0 }
}

/// The whole bus's own present current draw, A -- every load on it summed
/// (regardless of which breaker gates each one), the same quantity a real
/// feeder ammeter reads. Useful for watching a bus-transfer transient
/// directly (`bus_transfer_produces_an_inrush_transient` below).
pub fn bus_total_current_a(net: &Network, bus: BusId) -> f64 {
    net.loads.iter().filter(|l| l.spec.bus == bus).map(|l| l.current_a).sum()
}

/// `0.0`..`=1.0`: how loaded is this category's set of loads on their own
/// buses, relative to `capacity_w` -- primarily a diagnostic/telemetry
/// helper, but also what a real load-management page would show per
/// category on a power-budget display.
pub fn category_demand_w(net: &Network, catalog: &Catalog, category: LoadCategory) -> f64 {
    let indices: &[usize] = match category {
        LoadCategory::Essential => &catalog.essential,
        LoadCategory::Galley => &catalog.galley,
        LoadCategory::Commercial => &catalog.commercial,
        LoadCategory::Other => &catalog.other,
    };
    indices.iter().map(|&i| net.loads[i].spec.power_factor * net.loads[i].current_a * net.bus(net.loads[i].spec.bus).voltage).sum()
}

#[cfg(test)]
mod tests {
    use super::*;
    use super::super::loads;
    use super::super::network::{Breaker, Contactor, ContactorKind, FeedSource, LoadSpec, Source};

    fn network_with_catalog_and_source() -> (Network, Catalog) {
        let mut net = Network::new();
        let catalog = loads::build(&mut net);
        // Feed every bus in the catalogue with a strong, stiff source via a
        // dedicated contactor per bus, closed from the start -- enough to
        // run every load at close to its own rated voltage for a clean
        // power-budget/shed check without needing the full `sources.rs`
        // generator/TRU chain.
        for &bus in &super::super::network::ALL_BUS_IDS {
            let id: &'static str = Box::leak(format!("test-src-{}", bus.label()).into_boxed_str());
            let src = net.add_source(Source { id, open_circuit_v: bus.nominal_voltage(), resistance_ohm: 1.0e-4, frequency_hz: if bus.is_ac() { 400.0 } else { 0.0 } });
            let contactor_id: &'static str = Box::leak(format!("test-src-{}-line", bus.label()).into_boxed_str());
            let c = net.add_contactor(Contactor::new(contactor_id, ContactorKind::Feeder, FeedSource::Source(src), bus, 1.0e-4));
            net.contactors[c].commanded_closed = true;
        }
        (net, catalog)
    }

    #[test]
    fn galley_shed_commands_off_every_galley_load_and_nothing_else() {
        let (mut net, catalog) = network_with_catalog_and_source();
        net.step(1.0 / 60.0);
        let mut relays = SheddingRelays::new();
        let out = relays.step(&mut net, &catalog, &ShedInputs { galley_shed_commanded: true, emergency_config_commanded: false });
        assert!(out.galley_shed);
        assert!(!out.commercial_shed);
        for &i in &catalog.galley {
            assert!(!net.loads[i].commanded_on);
        }
        for &i in &catalog.commercial {
            assert!(net.loads[i].commanded_on, "commercial should not shed on a plain galley-shed command");
        }
        for &i in &catalog.essential {
            assert!(net.loads[i].commanded_on, "essential loads must never be shed by this relay");
        }
    }

    #[test]
    fn emergency_configuration_sheds_both_galley_and_commercial() {
        let (mut net, catalog) = network_with_catalog_and_source();
        net.step(1.0 / 60.0);
        let mut relays = SheddingRelays::new();
        let out = relays.step(&mut net, &catalog, &ShedInputs { galley_shed_commanded: false, emergency_config_commanded: true });
        assert!(out.galley_shed && out.commercial_shed);
        for &i in &catalog.galley {
            assert!(!net.loads[i].commanded_on);
        }
        for &i in &catalog.commercial {
            assert!(!net.loads[i].commanded_on);
        }
    }

    #[test]
    fn a_relay_that_always_fails_to_shed_never_sheds_its_category() {
        let (mut net, catalog) = network_with_catalog_and_source();
        net.step(1.0 / 60.0);
        let mut relays = SheddingRelays::new();
        relays.set_galley_faults(ShedRelayFaults { fails_to_shed: 1.0, sheds_when_not_commanded: 0.0 });
        let out = relays.step(&mut net, &catalog, &ShedInputs { galley_shed_commanded: true, emergency_config_commanded: false });
        assert!(!out.galley_shed, "a jammed shed relay should never actually shed");
        for &i in &catalog.galley {
            assert!(net.loads[i].commanded_on);
        }
    }

    #[test]
    fn a_relay_that_always_spuriously_sheds_sheds_even_when_not_commanded() {
        let (mut net, catalog) = network_with_catalog_and_source();
        net.step(1.0 / 60.0);
        let mut relays = SheddingRelays::new();
        relays.set_commercial_faults(ShedRelayFaults { fails_to_shed: 0.0, sheds_when_not_commanded: 1.0 });
        let out = relays.step(&mut net, &catalog, &ShedInputs { galley_shed_commanded: false, emergency_config_commanded: false });
        assert!(out.commercial_shed, "a welded-open shed relay should shed even with no command");
        for &i in &catalog.commercial {
            assert!(!net.loads[i].commanded_on);
        }
    }

    #[test]
    fn shedding_the_galleys_measurably_reduces_the_power_budget_demand() {
        let (mut net, catalog) = network_with_catalog_and_source();
        for _ in 0..3 {
            net.step(1.0 / 60.0);
        }
        let before = power_budget(&net, 200_000.0);
        let mut relays = SheddingRelays::new();
        relays.step(&mut net, &catalog, &ShedInputs { galley_shed_commanded: true, emergency_config_commanded: false });
        for _ in 0..3 {
            net.step(1.0 / 60.0);
        }
        let after = power_budget(&net, 200_000.0);
        assert!(after.total_demand_w < before.total_demand_w, "shedding galleys should reduce total demand: {} -> {}", before.total_demand_w, after.total_demand_w);
        assert!(after.margin_w > before.margin_w);
    }

    #[test]
    fn power_budget_flags_overload_when_demand_exceeds_capacity() {
        let (mut net, _catalog) = network_with_catalog_and_source();
        for _ in 0..3 {
            net.step(1.0 / 60.0);
        }
        let tiny_capacity = power_budget(&net, 1.0);
        assert!(tiny_capacity.overloaded);
        let huge_capacity = power_budget(&net, 10_000_000.0);
        assert!(!huge_capacity.overloaded);
    }

    #[test]
    fn bus_transfer_produces_an_inrush_transient() {
        // A motor-class load loses its bus, then the bus is re-energised
        // through a tie: the load's own inrush model (loads.rs's own
        // `inrush_multiple`/`inrush_duration_s`, network.rs's `Load::step`)
        // should show a real current spike above its later steady value --
        // the same physical transient a real bus transfer produces, emerging
        // from the existing per-load model rather than a new mechanism.
        let mut net = Network::new();
        let src = net.add_source(Source { id: "src", open_circuit_v: 28.0, resistance_ohm: 0.01, frequency_hz: 0.0 });
        let gc = net.add_contactor(Contactor::new("gc", ContactorKind::GeneratorLine, FeedSource::Source(src), BusId::Dc1, 0.01));
        net.contactors[gc].commanded_closed = true;
        let bkr = net.add_breaker(Breaker::new("bkr", 200.0, BusId::Dc1));
        let spec = LoadSpec {
            id: "motor",
            name: "Motor",
            ata: 24,
            bus: BusId::Dc1,
            rated_power_w: 500.0,
            power_factor: 0.85,
            min_operating_voltage: 0.0,
            inrush_multiple: 4.0,
            inrush_duration_s: 0.5,
            wiring_resistance_ohm: 0.05,
            rated_frequency_hz: 0.0,
            basis: "test",
        };
        net.add_load(spec, bkr);

        for _ in 0..30 {
            net.step(1.0 / 60.0);
        }
        let steady_before = net.loads[0].current_a;

        // Bus transfer: source drops (bus dies), then a moment later the
        // tie re-energises it via a second, independent source.
        net.command_contactor("gc", false);
        for _ in 0..30 {
            net.step(1.0 / 60.0);
        }
        assert_eq!(net.loads[0].current_a, 0.0, "load should be fully dead with its bus unpowered");

        let src2 = net.add_source(Source { id: "src2", open_circuit_v: 28.0, resistance_ohm: 0.01, frequency_hz: 0.0 });
        let gc2 = net.add_contactor(Contactor::new("gc2", ContactorKind::BusTie, FeedSource::Source(src2), BusId::Dc1, 0.01));
        net.contactors[gc2].commanded_closed = true;
        net.step(1.0 / 60.0);
        let transient_current = net.loads[0].current_a;

        for _ in 0..60 {
            net.step(1.0 / 60.0);
        }
        let steady_after = net.loads[0].current_a;

        assert!(transient_current > steady_after * 1.5, "bus transfer should show a real inrush transient: {transient_current} A vs steady {steady_after} A");
        assert!((steady_after - steady_before).abs() < steady_before * 0.2, "steady-state current should return close to its pre-transfer value: {steady_after} vs {steady_before}");
    }
}
