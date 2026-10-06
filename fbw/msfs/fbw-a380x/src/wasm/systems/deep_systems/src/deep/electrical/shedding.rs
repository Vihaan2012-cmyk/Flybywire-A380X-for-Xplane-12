use super::loads::{Catalog, LoadCategory};
use super::network::{bernoulli, fnv1a, BusId, Network};

#[derive(Clone, Copy, Debug, Default)]
pub struct ShedRelayFaults {
    pub fails_to_shed: f64,
    pub sheds_when_not_commanded: f64,
}

pub struct ShedInputs {
    pub galley_shed_commanded: bool,
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

#[derive(Clone, Copy, Debug, Default)]
pub struct PowerBudget {
    pub total_demand_w: f64,
    pub capacity_w: f64,
    pub margin_w: f64,
    pub overloaded: bool,
}

fn active_bus(l: &super::network::Load) -> Option<BusId> {
    l.active_feed.map(|fi| l.feeds[fi].bus)
}

pub fn power_budget(net: &Network, capacity_w: f64) -> PowerBudget {
    let total_demand_w: f64 = net
        .loads
        .iter()
        .filter_map(|l| active_bus(l).map(|bus| l.spec.power_factor * l.current_a * net.bus(bus).voltage))
        .sum();
    let margin_w = capacity_w - total_demand_w;
    PowerBudget { total_demand_w, capacity_w, margin_w, overloaded: margin_w < 0.0 }
}

pub fn bus_total_current_a(net: &Network, bus: BusId) -> f64 {
    net.loads.iter().filter(|l| active_bus(l) == Some(bus)).map(|l| l.current_a).sum()
}

pub fn category_demand_w(net: &Network, catalog: &Catalog, category: LoadCategory) -> f64 {
    let indices: &[usize] = match category {
        LoadCategory::Essential => &catalog.essential,
        LoadCategory::Galley => &catalog.galley,
        LoadCategory::Commercial => &catalog.commercial,
        LoadCategory::Other => &catalog.other,
    };
    indices
        .iter()
        .filter_map(|&i| active_bus(&net.loads[i]).map(|bus| net.loads[i].spec.power_factor * net.loads[i].current_a * net.bus(bus).voltage))
        .sum()
}

#[cfg(test)]
mod tests {
    use super::*;
    use super::super::loads;
    use super::super::network::{Breaker, Contactor, ContactorKind, FeedSource, LoadSpec, Source};

    fn network_with_catalog_and_source() -> (Network, Catalog) {
        let mut net = Network::new();
        let catalog = loads::build(&mut net);
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

    #[test]
    fn a_dual_fed_load_on_its_backup_feed_is_priced_and_metered_against_the_live_bus_not_the_dead_normal_one() {
        let mut net = Network::new();
        let normal_bus = BusId::Dc1;
        let backup_bus = BusId::DcEss;
        let src1 = net.add_source(Source { id: "src1", open_circuit_v: 28.0, resistance_ohm: 0.01, frequency_hz: 0.0 });
        let src2 = net.add_source(Source { id: "src2", open_circuit_v: 28.0, resistance_ohm: 0.01, frequency_hz: 0.0 });
        let c1 = net.add_contactor(Contactor::new("c1", ContactorKind::Feeder, FeedSource::Source(src1), normal_bus, 0.01));
        let c2 = net.add_contactor(Contactor::new("c2", ContactorKind::Feeder, FeedSource::Source(src2), backup_bus, 0.01));
        net.contactors[c1].commanded_closed = true;
        net.contactors[c2].commanded_closed = true;
        let normal_bkr = net.add_breaker(Breaker::new("normal-bkr", 20.0, normal_bus));
        let backup_bkr = net.add_breaker(Breaker::new("backup-bkr", 20.0, backup_bus));
        let spec = LoadSpec {
            id: "dual-fed-lru",
            name: "Dual-Fed LRU",
            ata: 34,
            bus: normal_bus,
            rated_power_w: 300.0,
            power_factor: 1.0,
            min_operating_voltage: 20.0,
            inrush_multiple: 1.0,
            inrush_duration_s: 0.0,
            wiring_resistance_ohm: 0.05,
            rated_frequency_hz: 0.0,
            basis: "test",
        };
        let feeds = vec![super::super::network::LoadFeed { bus: normal_bus, breaker: normal_bkr, priority: 0 }, super::super::network::LoadFeed { bus: backup_bus, breaker: backup_bkr, priority: 1 }];
        let i = net.add_load_multi_feed(spec, feeds);

        for _ in 0..10 {
            net.step(1.0 / 60.0);
        }
        assert_eq!(net.loads[i].active_feed, Some(0), "setup: healthy, it must be running on its normal (feed 0) bus");
        assert!(net.loads[i].current_a > 0.0, "setup: the load must be genuinely drawing current before the fault");

        net.breakers[normal_bkr].pull();
        for _ in 0..10 {
            net.step(1.0 / 60.0);
        }
        assert_eq!(net.loads[i].active_feed, Some(1), "the load must fail over onto its backup feed");
        let current = net.loads[i].current_a;
        assert!(current > 0.0, "the load must still be genuinely powered on its backup feed");

        assert_eq!(bus_total_current_a(&net, normal_bus), 0.0, "the dead normal bus must not still be credited with this load's current");
        assert!((bus_total_current_a(&net, backup_bus) - current).abs() < 1e-9, "the live backup bus must carry exactly this load's real current");

        let voltage_on_backup = net.bus(backup_bus).voltage;
        let this_loads_power_w = net.loads[i].spec.power_factor * current * voltage_on_backup;

        let budget = power_budget(&net, 200_000.0);
        assert!((budget.total_demand_w - this_loads_power_w).abs() < 1e-6, "power_budget must price this load's real power on its live backup bus, not at 0 on the dead normal one: {} vs {}", budget.total_demand_w, this_loads_power_w);
    }
}
