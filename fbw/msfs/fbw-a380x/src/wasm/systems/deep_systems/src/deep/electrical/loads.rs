use super::network::{Breaker, BusId, LoadFeed, LoadSpec, Network};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LoadCategory {
    Essential,
    Galley,
    Commercial,
    Other,
}

pub struct Catalog {
    pub galley: Vec<usize>,
    pub commercial: Vec<usize>,
    pub essential: Vec<usize>,
    pub other: Vec<usize>,
}

impl Catalog {
    fn new() -> Self {
        Self { galley: Vec::new(), commercial: Vec::new(), essential: Vec::new(), other: Vec::new() }
    }
    fn push(&mut self, category: LoadCategory, index: usize) {
        match category {
            LoadCategory::Essential => self.essential.push(index),
            LoadCategory::Galley => self.galley.push(index),
            LoadCategory::Commercial => self.commercial.push(index),
            LoadCategory::Other => self.other.push(index),
        }
    }
}

fn min_operating_voltage(bus: BusId) -> f64 {
    bus.nominal_voltage() * 0.85
}

fn wiring_resistance_ohm(bus: BusId) -> f64 {
    if bus.is_ac() {
        0.08
    } else {
        0.03
    }
}

fn avionics_spec(id: &'static str, name: &'static str, ata: u16, bus: BusId, watts: f64, basis: &'static str) -> LoadSpec {
    LoadSpec {
        id,
        name,
        ata,
        bus,
        rated_power_w: watts,
        power_factor: if bus.is_ac() { 0.95 } else { 1.0 },
        min_operating_voltage: min_operating_voltage(bus),
        inrush_multiple: 1.3,
        inrush_duration_s: 0.05,
        wiring_resistance_ohm: wiring_resistance_ohm(bus),
        rated_frequency_hz: 0.0,
        basis,
    }
}

fn motor_spec(id: &'static str, name: &'static str, ata: u16, bus: BusId, watts: f64, power_factor: f64, inrush_multiple: f64, inrush_duration_s: f64, basis: &'static str) -> LoadSpec {
    LoadSpec { id, name, ata, bus, rated_power_w: watts, power_factor, min_operating_voltage: min_operating_voltage(bus), inrush_multiple, inrush_duration_s, wiring_resistance_ohm: wiring_resistance_ohm(bus), rated_frequency_hz: 0.0, basis }
}

const VF_MOTOR_RATED_FREQUENCY_HZ: f64 = 800.0;

fn frequency_sensitive_motor_spec(id: &'static str, name: &'static str, ata: u16, bus: BusId, watts: f64, power_factor: f64, inrush_multiple: f64, inrush_duration_s: f64, rated_frequency_hz: f64, basis: &'static str) -> LoadSpec {
    let mut spec = motor_spec(id, name, ata, bus, watts, power_factor, inrush_multiple, inrush_duration_s, basis);
    spec.rated_frequency_hz = rated_frequency_hz;
    spec
}

fn resistive_spec(id: &'static str, name: &'static str, ata: u16, bus: BusId, watts: f64, basis: &'static str) -> LoadSpec {
    LoadSpec {
        id,
        name,
        ata,
        bus,
        rated_power_w: watts,
        power_factor: 1.0,
        min_operating_voltage: min_operating_voltage(bus),
        inrush_multiple: 1.5,
        inrush_duration_s: 2.0,
        rated_frequency_hz: 0.0,
        wiring_resistance_ohm: wiring_resistance_ohm(bus),
        basis,
    }
}

fn own_breaker(net: &mut Network, id: &'static str, rated_a: f64, bus: BusId) -> usize {
    net.add_breaker(Breaker::new(id, rated_a, bus))
}

fn add(net: &mut Network, cat: &mut Catalog, category: LoadCategory, spec: LoadSpec, rated_a: f64) {
    let bus = spec.bus;
    let bkr = own_breaker(net, spec.id, rated_a, bus);
    let idx = net.add_load(spec, bkr);
    cat.push(category, idx);
}

fn add_dual(net: &mut Network, cat: &mut Catalog, category: LoadCategory, spec: LoadSpec, rated_a: f64, second_bus: BusId) {
    let normal_bus = spec.bus;
    let id = spec.id;
    let normal_bkr_id: &'static str = Box::leak(format!("{id}-normal-bkr").into_boxed_str());
    let second_bkr_id: &'static str = Box::leak(format!("{id}-2nd-bkr").into_boxed_str());
    let normal_bkr = own_breaker(net, normal_bkr_id, rated_a, normal_bus);
    let second_bkr = own_breaker(net, second_bkr_id, rated_a, second_bus);
    let feeds = vec![LoadFeed { bus: normal_bus, breaker: normal_bkr, priority: 0 }, LoadFeed { bus: second_bus, breaker: second_bkr, priority: 1 }];
    let idx = net.add_load_multi_feed(spec, feeds);
    cat.push(category, idx);
}

#[allow(dead_code)]
fn add_triple(net: &mut Network, cat: &mut Catalog, category: LoadCategory, spec: LoadSpec, rated_a: f64, second_bus: BusId, third_bus: BusId) {
    let normal_bus = spec.bus;
    let id = spec.id;
    let bkr_id = |suffix: &str| -> &'static str { Box::leak(format!("{id}-{suffix}-bkr").into_boxed_str()) };
    let normal_bkr = own_breaker(net, bkr_id("normal"), rated_a, normal_bus);
    let second_bkr = own_breaker(net, bkr_id("2nd"), rated_a, second_bus);
    let third_bkr = own_breaker(net, bkr_id("3rd"), rated_a, third_bus);
    let feeds = vec![
        LoadFeed { bus: normal_bus, breaker: normal_bkr, priority: 0 },
        LoadFeed { bus: second_bus, breaker: second_bkr, priority: 1 },
        LoadFeed { bus: third_bus, breaker: third_bkr, priority: 2 },
    ];
    let idx = net.add_load_multi_feed(spec, feeds);
    cat.push(category, idx);
}

fn rated_current(spec: &LoadSpec) -> f64 {
    let base = spec.rated_power_w / (spec.bus.nominal_voltage() * spec.power_factor.max(0.1));
    crate::deep::breakers::catalog::standard_size(base * 1.25)
}

fn ata21(net: &mut Network, cat: &mut Catalog) {
    const FANS: [(&str, &str, BusId); 4] = [
        ("cab-fan-1", "CAB FAN 1", BusId::Ac1),
        ("cab-fan-2", "CAB FAN 2", BusId::Ac2),
        ("cab-fan-3", "CAB FAN 3", BusId::Ac3),
        ("cab-fan-4", "CAB FAN 4", BusId::Ac4),
    ];
    for (id, name, bus) in FANS {
        let spec = frequency_sensitive_motor_spec(id, name, 21, bus, 500.0, 0.85, 3.0, 1.0, VF_MOTOR_RATED_FREQUENCY_HZ, "breakers.rs::ata21 CAB FAN 1-4 (500 W typical large-transport recirculation fan motor, typical/derived); real bus from a380_systems/air_conditioning/mod.rs CabinFan::new; frequency-sensitive direct-drive induction motor, fan affinity laws vs the VFG's own variable output frequency, nameplate at VF_MOTOR_RATED_FREQUENCY_HZ");
        let a = rated_current(&spec);
        add(net, cat, LoadCategory::Other, spec, a);
    }
    let spec = motor_spec("hotair-1", "HOT AIR VALVE 1", 21, BusId::AcEss, 50.0, 0.8, 2.0, 0.3, "breakers.rs::ata21 HOT AIR VALVE 1 (50 W typical motor-operated valve actuator, typical/derived)");
    add(net, cat, LoadCategory::Other, spec.clone(), rated_current(&spec));
    let spec = motor_spec("hotair-2", "HOT AIR VALVE 2", 21, BusId::AcEss, 50.0, 0.8, 2.0, 0.3, "breakers.rs::ata21 HOT AIR VALVE 2");
    add(net, cat, LoadCategory::Other, spec.clone(), rated_current(&spec));
    let spec = motor_spec("fwd-isol-valve", "FWD CARGO ISOL VALVE", 21, BusId::Dc1, 50.0, 0.8, 2.0, 0.3, "breakers.rs::ata21 FWD CARGO ISOL VALVE (VCM Fwd's own primary channel, DC1/411PP -- ventilation_control_module.rs VentilationControlModule::new(.., VcmId::Fwd, [DirectCurrent(1), DirectCurrentEssential]), channel 1 backed by powered_by[0] and default-active; fixes/W161.md, matching fixes/W115.md -- was wired to DC2, VCM Aft's bus)");
    add(net, cat, LoadCategory::Other, spec.clone(), rated_current(&spec));
    let spec = frequency_sensitive_motor_spec("fwd-extract-fan", "FWD CARGO EXTRACT FAN", 21, BusId::Ac1, 150.0, 0.85, 3.0, 1.0, VF_MOTOR_RATED_FREQUENCY_HZ, "breakers.rs::ata21 FWD CARGO EXTRACT FAN (the fan's own dedicated bus, not the VCM's channel bus -- ventilation_control_module.rs ForwardCargoVentilationControlSystem::new(AlternatingCurrent(1)), really gated in receive_power/fwd_extraction_fan_is_on; a direct-drive VFG-fed induction motor like CAB FAN 1-4, not a DC valve actuator, so it takes the same frequency_sensitive_motor_spec treatment; fixes/W161.md, matching fixes/W115.md -- was wired to VCM Fwd's DC channel, which the fan does not draw from at all)");
    add(net, cat, LoadCategory::Other, spec.clone(), rated_current(&spec));
    let spec = motor_spec("bulk-isol-valve", "BULK CARGO ISOL VALVE", 21, BusId::Dc2, 50.0, 0.8, 2.0, 0.3, "breakers.rs::ata21 BULK CARGO ISOL VALVE (VCM Aft's own primary channel, DC2/214PP -- ventilation_control_module.rs VentilationControlModule::new(.., VcmId::Aft, [DirectCurrent(2), DirectCurrentEssential]), channel 1 backed by powered_by[0] and default-active; fixes/W161.md, matching fixes/W115.md -- was on DC_ESS, Aft's secondary/standby channel)");
    add(net, cat, LoadCategory::Other, spec.clone(), rated_current(&spec));
    let spec = frequency_sensitive_motor_spec("bulk-extract-fan", "BULK CARGO EXTRACT FAN", 21, BusId::Ac4, 150.0, 0.85, 3.0, 1.0, VF_MOTOR_RATED_FREQUENCY_HZ, "breakers.rs::ata21 BULK CARGO EXTRACT FAN (the fan's own dedicated bus, not the VCM's channel bus -- ventilation_control_module.rs BulkVentilationControlSystem::new(AlternatingCurrent(4)), really gated in receive_power/bulk_extraction_fan_is_on; a direct-drive VFG-fed induction motor like CAB FAN 1-4, not a DC valve actuator, so it takes the same frequency_sensitive_motor_spec treatment; fixes/W161.md, matching fixes/W115.md -- was wired to VCM Aft's DC_ESS channel, which the fan does not draw from at all)");
    add(net, cat, LoadCategory::Other, spec.clone(), rated_current(&spec));
    let spec = resistive_spec("cargo-heater", "BULK CARGO HEATER", 21, BusId::Ac2, 1000.0, "breakers.rs::ata21 BULK CARGO HEATER (1000 W typical cargo-bay heater element, typical/derived; mod.rs AirHeater::new(AC2))");
    add(net, cat, LoadCategory::Other, spec.clone(), rated_current(&spec));

    const FDAC: [(&str, &str, BusId); 4] = [("fdac-1a", "FDAC 1 CHANNEL 1", BusId::AcEss), ("fdac-1b", "FDAC 1 CHANNEL 2", BusId::Ac2), ("fdac-2a", "FDAC 2 CHANNEL 1", BusId::AcEss), ("fdac-2b", "FDAC 2 CHANNEL 2", BusId::Ac4)];
    for (id, name, bus) in FDAC {
        let spec = avionics_spec(id, name, 21, bus, 50.0, "breakers.rs::ata21 FDAC (50 W generic avionics LRU, typical/derived; mod.rs FullDigitalAGUController::new)");
        add(net, cat, LoadCategory::Essential, spec.clone(), rated_current(&spec));
    }
    const TADD: [(&str, &str, BusId); 2] = [("tadd-1", "TADD CHANNEL 1", BusId::Ac2), ("tadd-2", "TADD CHANNEL 2", BusId::Ac4)];
    for (id, name, bus) in TADD {
        let spec = avionics_spec(id, name, 21, bus, 50.0, "breakers.rs::ata21 TADD (mod.rs TrimAirDriveDevice::new)");
        add(net, cat, LoadCategory::Essential, spec.clone(), rated_current(&spec));
    }
    const VCM: [(&str, &str, BusId); 4] = [("vcm-fwd-1", "VCM FWD CHANNEL 1", BusId::Dc1), ("vcm-fwd-2", "VCM FWD CHANNEL 2", BusId::DcEss), ("vcm-aft-1", "VCM AFT CHANNEL 1", BusId::Dc2), ("vcm-aft-2", "VCM AFT CHANNEL 2", BusId::DcEss)];
    for (id, name, bus) in VCM {
        let spec = avionics_spec(id, name, 21, bus, 50.0, "breakers.rs::ata21 VCM (mod.rs VentilationControlModule::new)");
        add(net, cat, LoadCategory::Other, spec.clone(), rated_current(&spec));
    }
    const OCSM_AP: [(&str, &str, BusId); 4] = [("ocsm-1-ap", "OCSM 1 AUTO PARTITION", BusId::Dc1), ("ocsm-2-ap", "OCSM 2 AUTO PARTITION", BusId::Dc1), ("ocsm-3-ap", "OCSM 3 AUTO PARTITION", BusId::Dc2), ("ocsm-4-ap", "OCSM 4 AUTO PARTITION", BusId::Dc2)];
    for (id, name, bus) in OCSM_AP {
        let spec = avionics_spec(id, name, 21, bus, 50.0, "breakers.rs::ata21 OCSM auto-partition (mod.rs OutflowValveControlModule::new)");
        add(net, cat, LoadCategory::Essential, spec.clone(), rated_current(&spec));
    }
    const OCSM_CH: [(&str, &str, BusId); 8] = [
        ("ocsm-1a", "OCSM 1 CHANNEL 1", BusId::Dc1),
        ("ocsm-1b", "OCSM 1 CHANNEL 2", BusId::DcEss),
        ("ocsm-2a", "OCSM 2 CHANNEL 1", BusId::Dc1),
        ("ocsm-2b", "OCSM 2 CHANNEL 2", BusId::DcEss),
        ("ocsm-3a", "OCSM 3 CHANNEL 1", BusId::Dc2),
        ("ocsm-3b", "OCSM 3 CHANNEL 2", BusId::DcEss),
        ("ocsm-4a", "OCSM 4 CHANNEL 1", BusId::Dc2),
        ("ocsm-4b", "OCSM 4 CHANNEL 2", BusId::DcEss),
    ];
    for (id, name, bus) in OCSM_CH {
        let spec = avionics_spec(id, name, 21, bus, 50.0, "breakers.rs::ata21 OCSM channel");
        add(net, cat, LoadCategory::Essential, spec.clone(), rated_current(&spec));
    }
    let cpiom_bus = [BusId::Dc1, BusId::DcEss, BusId::DcEss, BusId::Dc2];
    for (app_idx, app) in ["AGS", "TCS", "VCS", "CPCS"].iter().enumerate() {
        for k in 0..4usize {
            let id: &'static str = Box::leak(format!("cpiom-b{}-{}", k + 1, app.to_lowercase()).into_boxed_str());
            let name: &'static str = Box::leak(format!("CPIOM B{} {} APP", k + 1, app).into_boxed_str());
            let basis: &'static str = Box::leak(format!("breakers.rs::ata21 CPIOM B{} {} APP (mod.rs CPIOM B bus map, ~line 1334; app group {})", k + 1, app, app_idx).into_boxed_str());
            let spec = avionics_spec(id, name, 21, cpiom_bus[k], 50.0, basis);
            add(net, cat, LoadCategory::Other, spec.clone(), rated_current(&spec));
        }
    }
    for pack in 1..=2u32 {
        for side in 1..=2u32 {
            let id: &'static str = Box::leak(format!("pack-{pack}-flow-valve-{side}").into_boxed_str());
            let name: &'static str = Box::leak(format!("PACK {pack} FLOW VALVE {side}").into_boxed_str());
            let basis: &'static str = Box::leak(format!("breakers.rs::ata21 PACK {pack} FLOW VALVE {side} (pneumatic.rs PackComplex ElectroPneumaticValve, DC_ESS)").into_boxed_str());
            let spec = motor_spec(id, name, 21, BusId::DcEss, 50.0, 0.8, 2.0, 0.3, basis);
            add(net, cat, LoadCategory::Other, spec.clone(), rated_current(&spec));
        }
    }
    for (id, name, bus) in [("avionics-fan-1", "AVIONICS BAY FAN 1", BusId::AcEss), ("avionics-fan-2", "AVIONICS BAY FAN 2", BusId::AcEssShed), ("avionics-fan-3", "AVIONICS BAY FAN 3", BusId::Ac1), ("avionics-fan-4", "AVIONICS BAY FAN 4", BusId::Ac2)] {
        let spec = frequency_sensitive_motor_spec(id, name, 21, bus, 300.0, 0.85, 3.0, 1.0, VF_MOTOR_RATED_FREQUENCY_HZ, "GENERIC: typical avionics-bay cooling blower motor (300 W), not individually named in breakers.rs but required to ventilate the LRU set it protects; direct-drive induction motor, frequency-sensitive, nameplate at VF_MOTOR_RATED_FREQUENCY_HZ");
        add(net, cat, LoadCategory::Other, spec.clone(), rated_current(&spec));
    }
}

fn ata26(net: &mut Network, cat: &mut Catalog) {
    let zones = ["ENG1", "ENG2", "ENG3", "ENG4", "APU", "MLGBAY"];
    for zone in zones {
        for loop_name in ["A", "B"] {
            let id: &'static str = Box::leak(format!("fire-loop-{}-{loop_name}", zone.to_lowercase()).into_boxed_str());
            let name: &'static str = Box::leak(format!("FIRE DET {zone} LOOP {loop_name}").into_boxed_str());
            let basis: &'static str = Box::leak(format!("breakers.rs::ata26 FIRE DET {zone} LOOP {loop_name} (20 W typical detection-loop controller electronics, typical/derived; fire_and_smoke_protection.rs DC_ESS/DC_HOT1)").into_boxed_str());
            let spec = avionics_spec(id, name, 26, BusId::DcEss, 20.0, basis);
            add(net, cat, LoadCategory::Essential, spec.clone(), rated_current(&spec));
        }
    }
}

fn ata27(net: &mut Network, cat: &mut Catalog) {
    let entries: [(&str, &str, u16); 11] = [
        ("rollout", "ROLLOUT", 22),
        ("fcu-1", "FCU 1", 22),
        ("fcu-2", "FCU 2", 22),
        ("prim-1", "PRIM 1", 27),
        ("prim-2", "PRIM 2", 27),
        ("prim-3", "PRIM 3", 27),
        ("sec-1", "SEC 1", 27),
        ("sec-2", "SEC 2", 27),
        ("sec-3", "SEC 3", 27),
        ("fcdc-1", "FCDC 1", 27),
        ("fcdc-2", "FCDC 2", 27),
    ];
    for (k, (id, name, ata)) in entries.into_iter().enumerate() {
        let normal_bus = if k % 2 == 0 { BusId::Dc1 } else { BusId::Dc2 };
        let spec = avionics_spec(id, name, ata, normal_bus, 100.0, "breakers.rs::ata27 flight-control/autoflight computer (100 W typical FCC-class LRU, typical/derived: FBW's C++ FCCs carry no Rust-side bus figure); real Airbus-style dual feed, normal DC bus + DC ESS backup, each on its own breaker, OR-ed internally");
        let a = rated_current(&spec);
        add_dual(net, cat, LoadCategory::Essential, spec, a, BusId::DcEss);
    }
}

fn ata32(net: &mut Network, cat: &mut Catalog) {
    let spec = avionics_spec("lgciu-1", "LGCIU 1", 32, BusId::DcEss, 50.0, "breakers.rs::ata32 LGCIU 1; real dual feed, DC ESS normal + DC2 backup, each on its own breaker");
    let a = rated_current(&spec);
    add_dual(net, cat, LoadCategory::Essential, spec, a, BusId::Dc2);
    let spec = avionics_spec("lgciu-2", "LGCIU 2", 32, BusId::Dc2, 50.0, "breakers.rs::ata32 LGCIU 2; real dual feed, DC2 normal + DC ESS backup, each on its own breaker");
    let a = rated_current(&spec);
    add_dual(net, cat, LoadCategory::Essential, spec, a, BusId::DcEss);

    const PUMPS: [(&str, &str, BusId); 4] = [("hyd-epump-ga", "HYD GREEN ELEC PUMP A", BusId::Ac3), ("hyd-epump-gb", "HYD GREEN ELEC PUMP B", BusId::Ac4), ("hyd-epump-ya", "HYD YELLOW ELEC PUMP A", BusId::AcEss), ("hyd-epump-yb", "HYD YELLOW ELEC PUMP B", BusId::Ac2)];
    for (id, name, bus) in PUMPS {
        let spec = motor_spec(id, name, 29, bus, 75.0 * 28.0, 0.85, 4.0, 0.5, "breakers.rs::ata32 electric hydraulic pump (FBW ELECTRIC_PUMP_MAX_CURRENT_AMPERE = 75 A, hydraulic/mod.rs:1750, real/FBW-sourced; 28 V-equivalent power rating, real/FBW current x reference DC voltage)");
        add(net, cat, LoadCategory::Other, spec.clone(), rated_current(&spec));

        let coil_id: &'static str = Box::leak(format!("{id}-coil").into_boxed_str());
        let coil_name: &'static str = Box::leak(format!("{name} CONTACTOR COIL").into_boxed_str());
        let coil_spec = avionics_spec(coil_id, coil_name, 29, BusId::DcEss, 20.0, "GENERIC: typical DC line-contactor holding-coil supply (~20 W), the low-power control circuit that energises the pump motor's own contactor, distinct from the motor's own high-current feed");
        add(net, cat, LoadCategory::Other, coil_spec.clone(), rated_current(&coil_spec));
    }
    let spec = motor_spec("autobrake-disarm-sol", "AUTOBRAKE DISARM SOLENOID", 32, BusId::Dc2, 56.0, 1.0, 3.0, 0.1, "breakers.rs::ata32 AUTOBRAKE DISARM SOLENOID (56 W typical small solenoid valve, typical/derived; autobrakes.rs DC2)");
    add(net, cat, LoadCategory::Other, spec.clone(), rated_current(&spec));

    const SENSORS: [&str; 12] = [
        "prox-uplock-gear-nose-1",
        "prox-downlock-gear-nose-2",
        "prox-uplock-gear-right-1",
        "prox-downlock-gear-right-2",
        "prox-uplock-gear-left-2",
        "prox-downlock-gear-left-1",
        "prox-uplock-door-nose-1",
        "prox-downlock-door-nose-2",
        "prox-uplock-door-right-2",
        "prox-downlock-door-right-1",
        "prox-uplock-door-left-2",
        "prox-downlock-door-left-1",
    ];
    for id in SENSORS {
        let name: &'static str = Box::leak(id.replace('-', " ").to_uppercase().into_boxed_str());
        let spec = avionics_spec(id, name, 32, BusId::DcEss, 5.0, "breakers.rs::ata32_gear_and_door_sensors proximity sensor (5 W typical target/pickup, typical/derived; LGCIU's own DC_ESS supply)");
        add(net, cat, LoadCategory::Essential, spec.clone(), rated_current(&spec));
    }
    const ACTUATORS: [&str; 6] = ["gear-actuator-nose", "gear-actuator-left", "gear-actuator-right", "gear-door-actuator-nose", "gear-door-actuator-left", "gear-door-actuator-right"];
    for id in ACTUATORS {
        let name: &'static str = Box::leak(id.replace('-', " ").to_uppercase().into_boxed_str());
        let spec = motor_spec(id, name, 32, BusId::DcEss, 75.0 * 28.0, 0.85, 3.0, 0.5, "breakers.rs::ata32_gear_and_door_sensors gear/door actuator control (same order of magnitude as the electric hydraulic pumps, typical/derived)");
        add(net, cat, LoadCategory::Other, spec.clone(), rated_current(&spec));
    }
}

fn ata34(net: &mut Network, cat: &mut Catalog) {
    const RAS: [(&str, &str, BusId); 3] = [("ra-sys-a", "RA SYS A", BusId::Ac1), ("ra-sys-b", "RA SYS B", BusId::Ac2), ("ra-sys-c", "RA SYS C", BusId::AcEss)];
    for (id, name, bus) in RAS {
        let spec = avionics_spec(id, name, 34, bus, 50.0, "breakers.rs::ata34 radio altimeter transceiver (navigation.rs A380RadioAltimeters)");
        add(net, cat, LoadCategory::Essential, spec.clone(), rated_current(&spec));
    }
    for (n, bus) in [(1, BusId::Ac1), (2, BusId::Ac2), (3, BusId::AcEss)] {
        let id: &'static str = Box::leak(format!("ra-ant-interrupt-{n}").into_boxed_str());
        let name: &'static str = Box::leak(format!("RA {n} ANTENNA INTERRUPT").into_boxed_str());
        let spec = avionics_spec(id, name, 34, bus, 10.0, "breakers.rs::ata34_ra_antennas antenna-coupling network (10 W class, typical/derived)");
        add(net, cat, LoadCategory::Essential, spec.clone(), rated_current(&spec));
        let id2: &'static str = Box::leak(format!("ra-ant-coupling-{n}").into_boxed_str());
        let name2: &'static str = Box::leak(format!("RA {n} ANTENNA DIRECT COUPLING").into_boxed_str());
        let spec2 = avionics_spec(id2, name2, 34, bus, 10.0, "breakers.rs::ata34_ra_antennas antenna-coupling network");
        add(net, cat, LoadCategory::Essential, spec2.clone(), rated_current(&spec2));
    }
    let spec = avionics_spec("egpwc", "EGPWC (TAWS)", 34, BusId::AcEss, 100.0, "breakers.rs::ata34_ra_antennas EGPWC (100 W typical flight-warning-class computer LRU, typical/derived; real AC_ESS bus, enhanced_gpwc/mod.rs)");
    add(net, cat, LoadCategory::Essential, spec.clone(), rated_current(&spec));
}

pub const FUEL_PUMPS: [(&str, &str, BusId); 21] = [
    ("fuel-pump-feed1-main", "FEED TK 1 MAIN PUMP", BusId::Ac4),
    ("fuel-pump-feed1-stby", "FEED TK 1 STBY PUMP", BusId::Ac2),
    ("fuel-pump-feed2-main", "FEED TK 2 MAIN PUMP", BusId::AcEss),
    ("fuel-pump-feed2-stby", "FEED TK 2 STBY PUMP", BusId::Ac3),
    ("fuel-pump-feed3-main", "FEED TK 3 MAIN PUMP", BusId::Ac3),
    ("fuel-pump-feed3-stby", "FEED TK 3 STBY PUMP", BusId::AcEss),
    ("fuel-pump-feed4-main", "FEED TK 4 MAIN PUMP", BusId::Ac2),
    ("fuel-pump-feed4-stby", "FEED TK 4 STBY PUMP", BusId::Ac4),
    ("fuel-pump-outer-left", "L OUTER TK PUMP", BusId::Ac2),
    ("fuel-pump-outer-right", "R OUTER TK PUMP", BusId::Ac2),
    ("fuel-pump-mid-fwd-left", "L MID TK FWD PUMP", BusId::Ac3),
    ("fuel-pump-mid-fwd-right", "R MID TK FWD PUMP", BusId::Ac3),
    ("fuel-pump-mid-aft-left", "L MID TK AFT PUMP", BusId::Ac1),
    ("fuel-pump-mid-aft-right", "R MID TK AFT PUMP", BusId::Ac1),
    ("fuel-pump-inner-fwd-left", "L INNER TK FWD PUMP", BusId::Ac4),
    ("fuel-pump-inner-fwd-right", "R INNER TK FWD PUMP", BusId::Ac4),
    ("fuel-pump-inner-aft-left", "L INNER TK AFT PUMP", BusId::Ac2),
    ("fuel-pump-inner-aft-right", "R INNER TK AFT PUMP", BusId::Ac2),
    ("fuel-pump-trim-left", "TRIM TK L PUMP", BusId::AcEss),
    ("fuel-pump-trim-right", "TRIM TK R PUMP", BusId::Ac2),
    ("fuel-pump-apu-feed", "APU FEED PUMP", BusId::DcEss),
];

pub const FUEL_VALVES: [(&str, &str, BusId); 21] = [
    ("fuel-valve-crossfeed-1", "CROSSFEED VALVE 1", BusId::Ac4),
    ("fuel-valve-crossfeed-2", "CROSSFEED VALVE 2", BusId::Ac4),
    ("fuel-valve-crossfeed-3", "CROSSFEED VALVE 3", BusId::Ac4),
    ("fuel-valve-crossfeed-4", "CROSSFEED VALVE 4", BusId::Ac4),
    ("fuel-valve-eng-lp-1", "ENG 1 LP VALVE", BusId::Dc1),
    ("fuel-valve-eng-lp-2", "ENG 2 LP VALVE", BusId::Dc2),
    ("fuel-valve-eng-lp-3", "ENG 3 LP VALVE", BusId::Dc1),
    ("fuel-valve-eng-lp-4", "ENG 4 LP VALVE", BusId::Dc2),
    ("fuel-valve-jettison-left", "JETTISON VALVE L", BusId::Dc1),
    ("fuel-valve-jettison-right", "JETTISON VALVE R", BusId::Dc2),
    ("fuel-valve-trim-inlet-1", "TRIM TK INLET VALVE 1", BusId::Dc1),
    ("fuel-valve-trim-inlet-2", "TRIM TK INLET VALVE 2", BusId::Dc2),
    ("fuel-valve-trim-iso-fwd", "TRIM LINE ISOL VALVE FWD", BusId::Dc1),
    ("fuel-valve-trim-iso-aft", "TRIM LINE ISOL VALVE AFT", BusId::DcEss),
    ("fuel-valve-outer-xfer-left", "L OUTER TK XFR VALVE", BusId::DcEss),
    ("fuel-valve-outer-xfer-right", "R OUTER TK XFR VALVE", BusId::DcEss),
    ("fuel-valve-mid-xfer-left", "L MID TK XFR VALVE", BusId::DcEss),
    ("fuel-valve-mid-xfer-right", "R MID TK XFR VALVE", BusId::DcEss),
    ("fuel-valve-inner-xfer-left", "L INNER TK XFR VALVE", BusId::DcEss),
    ("fuel-valve-inner-xfer-right", "R INNER TK XFR VALVE", BusId::DcEss),
    ("fuel-valve-apu-feed", "APU FEED/ISOL VALVE", BusId::DcEss),
];

fn ata28_fuel(net: &mut Network, cat: &mut Catalog) {
    for (id, name, bus) in FUEL_PUMPS {
        let spec = motor_spec(id, name, 28, bus, 600.0, 0.85, 3.0, 1.0, "bus per FlyByWire a380_systems/src/fuel/mod.rs A380FuelPump variant's FuelPumpProperties.powered_by, cross-checked against FCOM DSC-28-60 P4 Electrical Supply table (apu-feed pump: DC ESS GENERIC essential-bus assignment, the FCOM table's own APU row is drift-corrupted by the PDF extraction); wattage unchanged from this codebase's prior CIRCUIT_FUEL_PUMP precedent (600 W)");
        add(net, cat, LoadCategory::Essential, spec.clone(), rated_current(&spec));
    }
    for (id, name, bus) in FUEL_VALVES {
        let spec = motor_spec(id, name, 28, bus, 50.0, 0.8, 2.0, 0.3, "crossfeed/trim-inlet/trim-isolation/wing-transfer buses per FCOM DSC-28-60 P4 and DSC-28-70 P1/P2 Electrical Supply tables; engine LP/jettison/APU feed valve buses are GENERIC (table confirms dual-motor-different-supply design but not the exact bus pair); wattage unchanged from this codebase's prior CIRCUIT_FUEL_VALVE precedent (50 W)");
        add(net, cat, LoadCategory::Essential, spec.clone(), rated_current(&spec));
    }
}

fn ata33_lighting(net: &mut Network, cat: &mut Catalog) {
    const LIGHTS: [(&str, &str, BusId, f64); 12] = [
        ("light-landing", "LANDING LIGHTS", BusId::Ac1, 600.0),
        ("light-taxi", "TAXI LIGHTS", BusId::Ac2, 250.0),
        ("light-nav", "NAV LIGHTS", BusId::AcEssShed, 40.0),
        ("light-beacon", "BEACON LIGHTS", BusId::AcEssShed, 100.0),
        ("light-strobe", "STROBE LIGHTS", BusId::Ac3, 300.0),
        ("light-logo", "LOGO LIGHTS", BusId::Ac4, 150.0),
        ("light-wing", "WING LIGHTS", BusId::Ac1, 150.0),
        ("light-recognition", "RECOGNITION LIGHTS", BusId::DcBat, 40.0),
        ("light-cabin", "CABIN LIGHTS", BusId::AcGndFltSvc, 200.0),
        ("light-panel", "PANEL LIGHTS", BusId::DcEss, 30.0),
        ("light-pedestal", "PEDESTAL LIGHTS", BusId::DcEss, 20.0),
        ("light-glareshield", "GLARESHIELD LIGHTS", BusId::DcEss, 20.0),
    ];
    for (id, name, bus, watts) in LIGHTS {
        let spec = resistive_spec(id, name, 33, bus, watts, "circuits.rs CIRCUIT_LIGHT_* lumped per type (brief instruction); wattage = physics::electrical.rs::rated_watts, same precedent figure for this exact circuit type");
        add(net, cat, LoadCategory::Other, spec.clone(), rated_current(&spec));
    }
}

fn ata30_ice_protection(net: &mut Network, cat: &mut Catalog) {
    for (n, bus) in [(1, BusId::Ac1), (2, BusId::Ac2)] {
        let id: &'static str = Box::leak(format!("windshield-heat-{n}").into_boxed_str());
        let name: &'static str = Box::leak(format!("WINDSHIELD HEAT {n}").into_boxed_str());
        let spec = resistive_spec(id, name, 30, bus, 2000.0, "GENERIC: typical wide-body windshield electric anti-ice heating element (2000 W/side), no A380-specific public figure");
        add(net, cat, LoadCategory::Other, spec.clone(), rated_current(&spec));
    }
    for (n, bus) in [(1, BusId::Ac1), (2, BusId::Ac2), (3, BusId::AcEss)] {
        let id: &'static str = Box::leak(format!("pitot-heat-{n}").into_boxed_str());
        let name: &'static str = Box::leak(format!("PITOT HEAT {n}").into_boxed_str());
        let spec = resistive_spec(id, name, 30, bus, 600.0, "GENERIC precedent: physics::electrical.rs::rated_watts(\"CIRCUIT_PITOT_HEAT\") = 600 W, same figure reused here for a probe not otherwise modelled");
        add(net, cat, LoadCategory::Essential, spec.clone(), rated_current(&spec));
    }
    for (name_txt, id, bus) in [("AOA HEAT 1", "aoa-heat-1", BusId::Dc1), ("AOA HEAT 2", "aoa-heat-2", BusId::Dc2), ("TAT PROBE HEAT", "tat-heat", BusId::DcEss)] {
        let name: &'static str = Box::leak(name_txt.to_string().into_boxed_str());
        let spec = resistive_spec(id, name, 30, bus, 150.0, "GENERIC: typical small-probe (AOA vane / TAT) heating element, an order of magnitude below a pitot tube's own heater");
        add(net, cat, LoadCategory::Essential, spec.clone(), rated_current(&spec));
    }
}

fn ata25_galleys(net: &mut Network, cat: &mut Catalog) {
    const GALLEYS: [(&str, &str, BusId, f64); 6] = [
        ("galley-fwd-upper", "FWD UPPER GALLEY", BusId::Ac1, 8000.0),
        ("galley-aft-upper", "AFT UPPER GALLEY", BusId::Ac2, 8000.0),
        ("galley-fwd-main", "FWD MAIN DECK GALLEY", BusId::Ac3, 10000.0),
        ("galley-mid-main", "MID MAIN DECK GALLEY", BusId::Ac4, 10000.0),
        ("galley-aft-main", "AFT MAIN DECK GALLEY", BusId::Ac1, 10000.0),
        ("galley-lower", "LOWER DECK GALLEY LIFT", BusId::Ac2, 3000.0),
    ];
    for (id, name, bus, watts) in GALLEYS {
        let spec = resistive_spec(id, name, 25, bus, watts, "GENERIC: typical wide-body galley complex load (ovens, water heaters, chillers combined), no public per-zone A380 figure; the first load shed on a generator loss (`shedding.rs`)");
        add(net, cat, LoadCategory::Galley, spec.clone(), rated_current(&spec));
    }
}

fn ata44_ife(net: &mut Network, cat: &mut Catalog) {
    const ZONES: [(&str, &str, BusId, u32); 5] = [
        ("ife-upper-deck", "IFE UPPER DECK ZONE", BusId::AcGndFltSvc, 90),
        ("ife-main-fwd", "IFE MAIN DECK FWD ZONE", BusId::AcGndFltSvc, 120),
        ("ife-main-mid", "IFE MAIN DECK MID ZONE", BusId::AcGndFltSvc, 150),
        ("ife-main-aft", "IFE MAIN DECK AFT ZONE", BusId::AcGndFltSvc, 130),
        ("ife-server", "IFE SERVER RACK", BusId::Ac2, 1),
    ];
    for (id, name, bus, seats) in ZONES {
        let per_seat_w = if seats > 1 { 30.0 } else { 4000.0 };
        let watts = per_seat_w * seats as f64;
        let spec = avionics_spec(id, name, 44, bus, watts, "GENERIC: typical wide-body IFE seat-box power (~30 W/seat) x an approximate zone seat count, or a server-rack figure for the head-end; no public A380/FlyByWire figure");
        add(net, cat, LoadCategory::Commercial, spec.clone(), rated_current(&spec));
    }
}

fn avionics_misc(net: &mut Network, cat: &mut Catalog) {
    const DUAL_BOXES: [(&str, &str, u16, BusId, BusId, f64); 7] = [
        ("fms-1", "FMS 1", 34, BusId::Dc1, BusId::DcEss, 60.0),
        ("fms-2", "FMS 2", 34, BusId::Dc2, BusId::DcEss, 60.0),
        ("fms-3", "FMS 3", 34, BusId::Dc1, BusId::DcEss, 60.0),
        ("adirs-1", "ADIRU 1", 34, BusId::AcEss, BusId::DcEss, 80.0),
        ("adirs-2", "ADIRU 2", 34, BusId::Ac2, BusId::DcEss, 80.0),
        ("adirs-3", "ADIRU 3", 34, BusId::AcEssShed, BusId::DcEss, 80.0),
        ("tcas", "TCAS COMPUTER", 34, BusId::DcEss, BusId::Dc2, 70.0),
    ];
    for (id, name, ata, normal_bus, second_bus, watts) in DUAL_BOXES {
        let spec = avionics_spec(id, name, ata, normal_bus, watts, "GENERIC: typical avionics LRU class figure (same order of magnitude breakers.rs's own AVIONICS_LRU_W uses), no A380-specific public per-box wattage; real dual feed (normal + ESS/backup bus), each on its own breaker, OR-ed internally");
        let a = rated_current(&spec);
        add_dual(net, cat, LoadCategory::Essential, spec, a, second_bus);
    }
    const SINGLE_BOXES: [(&str, &str, u16, BusId, f64); 5] = [
        ("xpdr-1", "TRANSPONDER 1", 34, BusId::Dc1, 50.0),
        ("xpdr-2", "TRANSPONDER 2", 34, BusId::Dc2, 50.0),
        ("vhf-1", "VHF 1", 23, BusId::Dc1, 40.0),
        ("vhf-2", "VHF 2", 23, BusId::Dc2, 40.0),
        ("wxr", "WEATHER RADAR", 34, BusId::Ac1, 150.0),
    ];
    for (id, name, ata, bus, watts) in SINGLE_BOXES {
        let spec = avionics_spec(id, name, ata, bus, watts, "GENERIC: typical avionics LRU class figure (same order of magnitude breakers.rs's own AVIONICS_LRU_W uses), no A380-specific public per-box wattage; conventionally single-fed with a manual transfer switch, not an automatic OR");
        add(net, cat, LoadCategory::Essential, spec.clone(), rated_current(&spec));
    }
}

fn ata36_bleed(net: &mut Network, cat: &mut Catalog) {
    for n in 1..=4u32 {
        let bus = if n <= 2 { BusId::Dc1 } else { BusId::Dc2 };
        let id: &'static str = Box::leak(format!("bleed-eng-{n}").into_boxed_str());
        let name: &'static str = Box::leak(format!("BLEED ENG {n} VALVES").into_boxed_str());
        let basis: &'static str = Box::leak(format!("breakers.rs::ata36 BLEED ENG {n} (HP + pressure-regulating + fan-air valve, one shared feed, real Airbus-style single bleed CB; 3x50 W typical valve actuators, typical/derived)").into_boxed_str());
        let spec = motor_spec(id, name, 36, bus, 150.0, 0.8, 2.0, 0.3, basis);
        add(net, cat, LoadCategory::Other, spec.clone(), rated_current(&spec));
    }
}

fn ata24_power_sources_extra(net: &mut Network, cat: &mut Catalog) {
    let spec = avionics_spec(
        "ext-pwr-contactor",
        "EXTERNAL POWER CONTACTOR CONTROL",
        24,
        BusId::DcHot1,
        20.0,
        "breakers.rs::ata24_power_sources EXTERNAL POWER CONTACTOR CONTROL (GENERIC: typical small contactor-coil control circuit, real A380 external power system, no public per-part figure; same figure that catalogue's own breaker already cites)",
    );
    add(net, cat, LoadCategory::Other, spec.clone(), rated_current(&spec));
    for n in 1..=2u32 {
        let id: &'static str = Box::leak(format!("bat-charge-limiter-{n}").into_boxed_str());
        let name: &'static str = Box::leak(format!("BATTERY {n} CHARGE LIMITER").into_boxed_str());
        let basis: &'static str = Box::leak(format!("breakers.rs::ata24_power_sources BATTERY {n} CHARGE LIMITER (GENERIC: typical small charge-controller LRU control circuit; same figure that catalogue's own breaker already cites)").into_boxed_str());
        let spec = avionics_spec(id, name, 24, BusId::DcEss, 20.0, basis);
        add(net, cat, LoadCategory::Other, spec.clone(), rated_current(&spec));
    }
}

fn ata23_comms(net: &mut Network, cat: &mut Catalog) {
    let spec = avionics_spec("satcom", "SATCOM", 23, BusId::DcEss, 100.0, "breakers.rs::ata23_comms SATCOM (GENERIC: typical wide-body SATCOM transceiver LRU, real A380 equipment class, no public per-box figure; same figure that catalogue's own breaker already cites)");
    add(net, cat, LoadCategory::Other, spec.clone(), rated_current(&spec));
    let spec = avionics_spec("hf-1", "HF 1", 23, BusId::Dc1, 100.0, "breakers.rs::ata23_comms HF 1 (GENERIC: typical HF transceiver LRU; same figure that catalogue's own breaker already cites)");
    add(net, cat, LoadCategory::Other, spec.clone(), rated_current(&spec));
    let spec = avionics_spec("hf-2", "HF 2", 23, BusId::Dc2, 100.0, "breakers.rs::ata23_comms HF 2 (same class as HF 1)");
    add(net, cat, LoadCategory::Other, spec.clone(), rated_current(&spec));
    let spec = avionics_spec("acars-mu", "ACARS MU", 23, BusId::DcEss, 50.0, "breakers.rs::ata23_comms ACARS MU (GENERIC: typical avionics LRU class figure; same figure that catalogue's own breaker already cites)");
    add(net, cat, LoadCategory::Other, spec.clone(), rated_current(&spec));
    let spec = motor_spec("pa-amplifier", "PA AMPLIFIER", 23, BusId::Ac1, 200.0, 0.9, 1.3, 0.2, "breakers.rs::ata23_comms PA AMPLIFIER (GENERIC: typical wide-body PA amplifier power stage; same figure/power factor that catalogue's own breaker already cites)");
    add(net, cat, LoadCategory::Other, spec.clone(), rated_current(&spec));
    let spec = avionics_spec("interphone", "INTERPHONE", 23, BusId::DcEss, 50.0, "breakers.rs::ata23_comms INTERPHONE (GENERIC: typical avionics LRU class figure; same figure that catalogue's own breaker already cites)");
    add(net, cat, LoadCategory::Other, spec.clone(), rated_current(&spec));
}

fn ata31_recorders(net: &mut Network, cat: &mut Catalog) {
    let spec = avionics_spec("dfdr", "DFDR", 31, BusId::DcEss, 50.0, "breakers.rs::ata31_recorders DFDR (GENERIC: typical avionics LRU class figure, real mandatory A380 equipment; same figure that catalogue's own breaker already cites)");
    add(net, cat, LoadCategory::Essential, spec.clone(), rated_current(&spec));
    let spec = avionics_spec("cvr", "CVR", 31, BusId::DcEss, 50.0, "breakers.rs::ata31_recorders CVR (same class as DFDR, real mandatory A380 equipment)");
    add(net, cat, LoadCategory::Essential, spec.clone(), rated_current(&spec));
    let spec = avionics_spec("qar", "QAR", 31, BusId::Dc2, 30.0, "breakers.rs::ata31_recorders QAR (GENERIC: typical small avionics LRU class figure; same figure that catalogue's own breaker already cites)");
    add(net, cat, LoadCategory::Other, spec.clone(), rated_current(&spec));
}

fn ata31_ind_group(net: &mut Network, cat: &mut Catalog) {
    let spec = avionics_spec("capt-efis-bkup-ctl", "CAPT EFIS BKUP CTL", 31, BusId::DcEss, 30.0, "E-IND-DESIGN.md 311800002 (GENERIC: typical small avionics LRU class figure, same figure this file's other CDS-class entries cite; real A380 equipment, no public per-box wattage)");
    add(net, cat, LoadCategory::Essential, spec.clone(), rated_current(&spec));
    let spec = avionics_spec("fo-efis-bkup-ctl", "F/O EFIS BKUP CTL", 31, BusId::DcEss, 30.0, "E-IND-DESIGN.md 311800003 (same class as CAPT EFIS BKUP CTL)");
    add(net, cat, LoadCategory::Essential, spec.clone(), rated_current(&spec));
    let spec = avionics_spec("capt-efis-ctl-panel", "CAPT EFIS CTL PANEL", 31, BusId::DcEss, 30.0, "E-IND-DESIGN.md 311800004/006 (GENERIC: typical small avionics LRU class figure)");
    add(net, cat, LoadCategory::Essential, spec.clone(), rated_current(&spec));
    let spec = avionics_spec("fo-efis-ctl-panel", "F/O EFIS CTL PANEL", 31, BusId::DcEss, 30.0, "E-IND-DESIGN.md 311800005/006 (same class as CAPT EFIS CTL PANEL)");
    add(net, cat, LoadCategory::Essential, spec.clone(), rated_current(&spec));

    let spec = avionics_spec("capt-pfd-du", "CAPT PFD DU", 31, BusId::DcEss, 60.0, "E-IND-DESIGN.md 311800007/010 (GENERIC: typical display-unit-class avionics LRU figure); feed DC ESS per FlyByWire CdsDisplayUnit.tsx DisplayUnitToDCBus:42 (409PP)");
    add(net, cat, LoadCategory::Essential, spec.clone(), rated_current(&spec));
    let spec = avionics_spec("capt-nd-du", "CAPT ND DU", 31, BusId::DcEss, 60.0, "E-IND-DESIGN.md 311800008/011 (same class as CAPT PFD DU); dual feed DC ESS + DC 1 per FlyByWire CdsDisplayUnit.tsx DisplayUnitToDCBus:43 (415PP or 105PP)");
    let a = rated_current(&spec);
    add_dual(net, cat, LoadCategory::Essential, spec, a, BusId::Dc1);
    let spec = avionics_spec("capt-ewd-du", "CAPT EWD DU", 31, BusId::DcEss, 60.0, "E-IND-DESIGN.md 311800009 (same class as CAPT PFD DU); feed DC ESS per FlyByWire CdsDisplayUnit.tsx DisplayUnitToDCBus:48 (423PP)");
    add(net, cat, LoadCategory::Essential, spec.clone(), rated_current(&spec));
    let spec = avionics_spec("fo-pfd-du", "F/O PFD DU", 31, BusId::Dc2, 60.0, "E-IND-DESIGN.md 311800010 (same class as CAPT PFD DU); feed DC 2 per FlyByWire CdsDisplayUnit.tsx DisplayUnitToDCBus:45");
    add(net, cat, LoadCategory::Essential, spec.clone(), rated_current(&spec));
    let spec = avionics_spec("fo-nd-du", "F/O ND DU", 31, BusId::Dc1, 60.0, "E-IND-DESIGN.md 311800011 (same class as CAPT PFD DU); dual feed DC 1 + DC 2 per FlyByWire CdsDisplayUnit.tsx DisplayUnitToDCBus:46");
    let a = rated_current(&spec);
    add_dual(net, cat, LoadCategory::Essential, spec, a, BusId::Dc2);

    let spec = avionics_spec("kccu-capt", "CAPT KCCU", 31, BusId::DcEss, 40.0, "E-IND-DESIGN.md 313800001/003/005 (GENERIC: typical small avionics keyboard/cursor-control-unit LRU figure)");
    add(net, cat, LoadCategory::Essential, spec.clone(), rated_current(&spec));
    let spec = avionics_spec("kccu-fo", "F/O KCCU", 31, BusId::DcEss, 40.0, "E-IND-DESIGN.md 313800002/004/006 (same class as CAPT KCCU)");
    add(net, cat, LoadCategory::Essential, spec.clone(), rated_current(&spec));

    let spec = avionics_spec("cds-mailbox-capt", "CAPT CDS MAILBOX", 31, BusId::DcEss, 20.0, "E-IND-DESIGN.md 313800007 (GENERIC: typical small CDS peripheral LRU figure)");
    add(net, cat, LoadCategory::Essential, spec.clone(), rated_current(&spec));

    let spec = avionics_spec("hud", "HUD", 31, BusId::DcEss, 50.0, "E-IND-DESIGN.md 316800001/002 (GENERIC: typical HUD projector/combiner-unit avionics LRU figure)");
    add(net, cat, LoadCategory::Essential, spec.clone(), rated_current(&spec));

    let spec = avionics_spec("video-multiplexer", "VIDEO MULTIPLEXER", 31, BusId::DcEss, 30.0, "E-IND-DESIGN.md 318800001 (GENERIC: typical small avionics LRU class figure)");
    add(net, cat, LoadCategory::Essential, spec.clone(), rated_current(&spec));

    let spec = avionics_spec("recorder-accelerometer", "RECORDER ACCELEROMETER", 31, BusId::DcEss, 20.0, "E-IND-DESIGN.md 319800001 (GENERIC: typical small avionics accessory LRU figure, same ATA31-mandatory-equipment class as ata31_recorders' own CVR/DFDR/QAR)");
    add(net, cat, LoadCategory::Essential, spec.clone(), rated_current(&spec));
    let spec = avionics_spec("dfdau", "DFDAU", 31, BusId::DcEss, 50.0, "E-IND-DESIGN.md 319800004 (GENERIC: typical avionics LRU class figure, same ATA31-mandatory-equipment class as ata31_recorders' own CVR/DFDR/QAR)");
    add(net, cat, LoadCategory::Essential, spec.clone(), rated_current(&spec));
}

fn ata35_oxygen_extra(net: &mut Network, cat: &mut Catalog) {
    let spec = motor_spec("crew-o2-shutoff", "CREW OXYGEN SHUTOFF VALVE", 35, BusId::Dc1, 50.0, 0.8, 2.0, 0.3, "breakers.rs::ata35_oxygen CREW OXYGEN SHUTOFF VALVE (GENERIC: typical motor/solenoid-operated shutoff valve actuator, real A380 crew oxygen system; same figure that catalogue's own breaker already cites)");
    add(net, cat, LoadCategory::Essential, spec.clone(), rated_current(&spec));
    let spec = avionics_spec("pax-o2-gen-ctl", "PAX OXYGEN GENERATOR CONTROL", 35, BusId::DcEss, 30.0, "breakers.rs::ata35_oxygen PAX OXYGEN GENERATOR CONTROL (GENERIC: typical small control-circuit LRU figure; same figure that catalogue's own breaker already cites)");
    add(net, cat, LoadCategory::Essential, spec.clone(), rated_current(&spec));
    let spec = avionics_spec("o2-pressure-xducer", "OXYGEN PRESSURE TRANSDUCER", 35, BusId::DcEss, 5.0, "breakers.rs::ata35_oxygen OXYGEN PRESSURE TRANSDUCER (GENERIC: typical small pressure-transducer power draw; same figure that catalogue's own breaker already cites)");
    add(net, cat, LoadCategory::Essential, spec.clone(), rated_current(&spec));
}

fn ata49_apu_extra(net: &mut Network, cat: &mut Catalog) {
    let spec = avionics_spec("apu-ecu-a", "APU ECU CHANNEL A", 49, BusId::DcApu, 60.0, "breakers.rs::ata49_apu APU ECU CHANNEL A (GENERIC: typical dual-channel engine/APU controller LRU class figure, real PW980 APU has its own FADEC-class controller; same figure that catalogue's own breaker already cites)");
    add(net, cat, LoadCategory::Essential, spec.clone(), rated_current(&spec));
    let spec = avionics_spec("apu-ecu-b", "APU ECU CHANNEL B", 49, BusId::DcEss, 60.0, "breakers.rs::ata49_apu APU ECU CHANNEL B (same class as channel A, redundant bus feed)");
    add(net, cat, LoadCategory::Essential, spec.clone(), rated_current(&spec));
    let spec = motor_spec("apu-fuel-shutoff-valve", "APU FUEL SHUTOFF VALVE", 49, BusId::Dc1, 50.0, 0.8, 2.0, 0.3, "breakers.rs::ata49_apu APU FUEL SHUTOFF VALVE (GENERIC: typical motor/solenoid-operated shutoff valve actuator; same figure that catalogue's own breaker already cites)");
    add(net, cat, LoadCategory::Essential, spec.clone(), rated_current(&spec));
    let spec = avionics_spec("apu-start-contactor", "APU START CONTACTOR", 49, BusId::Dc1, 20.0, "breakers.rs::ata49_apu APU START CONTACTOR (GENERIC: typical contactor-coil control circuit; same figure that catalogue's own breaker already cites; transit-only, see electrical::live::TRANSIT_ONLY_NO_TRUTH_INPUT)");
    add(net, cat, LoadCategory::Essential, spec.clone(), rated_current(&spec));
}

fn ata73_74_engine(net: &mut Network, cat: &mut Catalog) {
    for n in 1..=4u32 {
        let bus_a = if n <= 2 { BusId::DcEss } else { BusId::Dc1 };
        let bus_b = if n <= 2 { BusId::Dc2 } else { BusId::DcEss };
        let id_a: &'static str = Box::leak(format!("fadec-{n}a").into_boxed_str());
        let name_a: &'static str = Box::leak(format!("FADEC {n} CHANNEL A").into_boxed_str());
        let basis_a: &'static str = Box::leak(format!("breakers.rs::ata7x_engine FADEC {n} CHANNEL A (GENERIC: typical dual-lane FADEC-class controller channel, real Trent 972B-84 architecture, no public per-channel electrical figure; same figure that catalogue's own breaker already cites)").into_boxed_str());
        let spec = avionics_spec(id_a, name_a, 73, bus_a, 80.0, basis_a);
        add(net, cat, LoadCategory::Essential, spec.clone(), rated_current(&spec));
        let id_b: &'static str = Box::leak(format!("fadec-{n}b").into_boxed_str());
        let name_b: &'static str = Box::leak(format!("FADEC {n} CHANNEL B").into_boxed_str());
        let basis_b: &'static str = Box::leak(format!("breakers.rs::ata7x_engine FADEC {n} CHANNEL B (same class as channel A, redundant bus feed)").into_boxed_str());
        let spec = avionics_spec(id_b, name_b, 73, bus_b, 80.0, basis_b);
        add(net, cat, LoadCategory::Essential, spec.clone(), rated_current(&spec));
    }
    for n in 1..=4u32 {
        let bus_a = if n % 2 == 1 { BusId::Dc1 } else { BusId::Dc2 };
        let bus_b = if n % 2 == 1 { BusId::Dc2 } else { BusId::Dc1 };
        let id_a: &'static str = Box::leak(format!("ignition-{n}a").into_boxed_str());
        let name_a: &'static str = Box::leak(format!("IGNITION {n} EXCITER A").into_boxed_str());
        let basis_a: &'static str = Box::leak(format!("breakers.rs::ata7x_engine IGNITION {n} EXCITER A (GENERIC: typical high-energy ignition exciter unit pulsed power class (~250 W), no public per-part figure; same figure/power factor that catalogue's own breaker already cites; transit-only, see electrical::live::TRANSIT_ONLY_NO_TRUTH_INPUT)").into_boxed_str());
        let spec = motor_spec(id_a, name_a, 74, bus_a, 250.0, 0.9, 2.0, 0.5, basis_a);
        add(net, cat, LoadCategory::Essential, spec.clone(), rated_current(&spec));
        let id_b: &'static str = Box::leak(format!("ignition-{n}b").into_boxed_str());
        let name_b: &'static str = Box::leak(format!("IGNITION {n} EXCITER B").into_boxed_str());
        let basis_b: &'static str = Box::leak(format!("breakers.rs::ata7x_engine IGNITION {n} EXCITER B (same class as exciter A, redundant lane on the opposite DC bus)").into_boxed_str());
        let spec = motor_spec(id_b, name_b, 74, bus_b, 250.0, 0.9, 2.0, 0.5, basis_b);
        add(net, cat, LoadCategory::Essential, spec.clone(), rated_current(&spec));
    }
}

fn ata26_extinguishing(net: &mut Network, cat: &mut Catalog) {
    for bottle in 1..=2u32 {
        for squib in 1..=2u32 {
            let bus = if bottle == 1 { BusId::Dc1 } else { BusId::Dc2 };
            let id: &'static str = Box::leak(format!("eng-fire-bottle-{bottle}-squib-{squib}").into_boxed_str());
            let name: &'static str = Box::leak(format!("ENG FIRE BOTTLE {bottle} SQUIB {squib}").into_boxed_str());
            let basis: &'static str = Box::leak(format!("breakers.rs::ata26_extinguishing ENG FIRE BOTTLE {bottle} SQUIB {squib} (GENERIC: typical one-shot pyrotechnic squib firing circuit, real wide-body cross-feed fire-extinguishing architecture; same figure that catalogue's own breaker already cites; transit-only/one-shot, see electrical::live::TRANSIT_ONLY_NO_TRUTH_INPUT)").into_boxed_str());
            let spec = avionics_spec(id, name, 26, bus, 20.0, basis);
            add(net, cat, LoadCategory::Essential, spec.clone(), rated_current(&spec));
        }
    }
    for squib in 1..=2u32 {
        let id: &'static str = Box::leak(format!("apu-fire-bottle-squib-{squib}").into_boxed_str());
        let name: &'static str = Box::leak(format!("APU FIRE BOTTLE SQUIB {squib}").into_boxed_str());
        let basis: &'static str = Box::leak(format!("breakers.rs::ata26_extinguishing APU FIRE BOTTLE SQUIB {squib} (same class as the engine bottle squibs; transit-only/one-shot)").into_boxed_str());
        let spec = avionics_spec(id, name, 26, BusId::DcApu, 20.0, basis);
        add(net, cat, LoadCategory::Essential, spec.clone(), rated_current(&spec));
    }
}

fn ata29_hydraulics_extra(net: &mut Network, cat: &mut Catalog) {
    let spec = avionics_spec("rat-deploy-solenoid", "RAT DEPLOY SOLENOID", 29, BusId::DcHot2, 100.0, "breakers.rs::ata29_hydraulics_extra RAT DEPLOY SOLENOID (GENERIC: typical deployment solenoid, hot-bus fed so it works with both engines/APU/main batteries down; same figure that catalogue's own breaker already cites; transit-only, gated in electrical::live against the real emergency/rat_deployed state)");
    add(net, cat, LoadCategory::Essential, spec.clone(), rated_current(&spec));
    let spec = motor_spec("ptu-control-valve", "PTU CONTROL VALVE", 29, BusId::DcEss, 50.0, 0.8, 2.0, 0.3, "breakers.rs::ata29_hydraulics_extra PTU CONTROL VALVE (GENERIC: typical motor/solenoid-operated valve actuator, real green/yellow hydraulic power-transfer-unit architecture; same figure that catalogue's own breaker already cites)");
    add(net, cat, LoadCategory::Essential, spec.clone(), rated_current(&spec));
}

fn ata52_doors(net: &mut Network, cat: &mut Catalog) {
    let spec = motor_spec("cargo-door-fwd-actuator-ctl", "FWD CARGO DOOR ACTUATOR CONTROL", 52, BusId::Dc1, 100.0, 0.8, 2.0, 0.3, "breakers.rs::ata52_doors FWD CARGO DOOR ACTUATOR CONTROL (GENERIC: typical powered cargo door actuator control circuit; same figure that catalogue's own breaker already cites; transit-only, see electrical::live::TRANSIT_ONLY_NO_TRUTH_INPUT)");
    add(net, cat, LoadCategory::Other, spec.clone(), rated_current(&spec));
    let spec = motor_spec("cargo-door-aft-actuator-ctl", "AFT CARGO DOOR ACTUATOR CONTROL", 52, BusId::Dc2, 100.0, 0.8, 2.0, 0.3, "breakers.rs::ata52_doors AFT CARGO DOOR ACTUATOR CONTROL (same class as the forward cargo door; transit-only)");
    add(net, cat, LoadCategory::Other, spec.clone(), rated_current(&spec));
}

fn ata33_emergency_lighting(net: &mut Network, cat: &mut Catalog) {
    let spec = avionics_spec("emer-lighting-charger-1", "EMER LIGHTING BATTERY CHARGER 1", 33, BusId::DcHot1, 100.0, "breakers.rs::ata33_emergency_lighting EMER LIGHTING BATTERY CHARGER 1 (GENERIC: typical NiCd/Li-ion emergency-lighting pack charger circuit; same figure that catalogue's own breaker already cites)");
    add(net, cat, LoadCategory::Other, spec.clone(), rated_current(&spec));
    let spec = avionics_spec("emer-lighting-charger-2", "EMER LIGHTING BATTERY CHARGER 2", 33, BusId::DcHot2, 100.0, "breakers.rs::ata33_emergency_lighting EMER LIGHTING BATTERY CHARGER 2 (same class as charger 1)");
    add(net, cat, LoadCategory::Other, spec.clone(), rated_current(&spec));
    let spec = resistive_spec("ext-service-lighting", "EXTERIOR SERVICE LIGHTING", 33, BusId::AcGndFltSvc, 100.0, "breakers.rs::ata33_emergency_lighting EXTERIOR SERVICE LIGHTING (GENERIC: typical ground-service floodlight circuit; same figure that catalogue's own breaker already cites)");
    add(net, cat, LoadCategory::Other, spec.clone(), rated_current(&spec));
}

fn position_indication_spec(id: &'static str, name: &'static str, ata: u16, bus: BusId, basis: &'static str) -> LoadSpec {
    LoadSpec { id, name, ata, bus, rated_power_w: 5.0, power_factor: 1.0, min_operating_voltage: min_operating_voltage(bus), inrush_multiple: 1.0, inrush_duration_s: 0.0, wiring_resistance_ohm: wiring_resistance_ohm(bus), rated_frequency_hz: 0.0, basis }
}

fn position_indication_load(net: &mut Network, cat: &mut Catalog, parent_id: &'static str, parent_name: &'static str, ata: u16, bus: BusId, basis_suffix: &'static str) {
    let id: &'static str = Box::leak(format!("{parent_id}-pos-ind").into_boxed_str());
    let name: &'static str = Box::leak(format!("{parent_name} POSITION IND").into_boxed_str());
    let basis: &'static str = Box::leak(format!("breakers.rs::ata_control_excitation_supplies position-indication microswitch/LVDT excitation circuit (GENERIC 5 W, same figure that catalogue's own breaker already cites); {basis_suffix}").into_boxed_str());
    let spec = position_indication_spec(id, name, ata, bus, basis);
    add(net, cat, LoadCategory::Essential, spec.clone(), rated_current(&spec));
}

fn position_indication_supplies(net: &mut Network, cat: &mut Catalog) {
    let valve_buses = [BusId::Dc1, BusId::Dc2, BusId::DcEss, BusId::DcBat];
    for (i, (id, name, _)) in FUEL_VALVES.into_iter().enumerate() {
        position_indication_load(net, cat, id, name, 28, valve_buses[i % valve_buses.len()], "pairs with this catalogue's own fuel valve actuator breaker");
    }
    position_indication_load(net, cat, "hotair-1", "HOT AIR VALVE 1", 21, BusId::AcEss, "pairs with HOT AIR VALVE 1's own actuator breaker");
    position_indication_load(net, cat, "hotair-2", "HOT AIR VALVE 2", 21, BusId::AcEss, "pairs with HOT AIR VALVE 2's own actuator breaker");
    position_indication_load(net, cat, "fwd-isol-valve", "FWD CARGO ISOL VALVE", 21, BusId::Dc1, "pairs with FWD CARGO ISOL VALVE's own actuator breaker (fixes/W161.md: DC1, VCM Fwd's own primary channel)");
    position_indication_load(net, cat, "bulk-isol-valve", "BULK CARGO ISOL VALVE", 21, BusId::Dc2, "pairs with BULK CARGO ISOL VALVE's own actuator breaker (fixes/W161.md: DC2, VCM Aft's own primary channel)");
    for pack in 1..=2u32 {
        for side in 1..=2u32 {
            let parent_id: &'static str = Box::leak(format!("pack-{pack}-flow-valve-{side}").into_boxed_str());
            let parent_name: &'static str = Box::leak(format!("PACK {pack} FLOW VALVE {side}").into_boxed_str());
            position_indication_load(net, cat, parent_id, parent_name, 21, BusId::DcEss, "pairs with its own PACK FLOW VALVE actuator breaker");
        }
    }
    for n in 1..=4u32 {
        let bus = if n <= 2 { BusId::Dc1 } else { BusId::Dc2 };
        let parent_id: &'static str = Box::leak(format!("bleed-eng-{n}").into_boxed_str());
        let parent_name: &'static str = Box::leak(format!("BLEED ENG {n} VALVES").into_boxed_str());
        position_indication_load(net, cat, parent_id, parent_name, 36, bus, "pairs with the shared BLEED ENG valve-set actuator breaker");
    }
    position_indication_load(net, cat, "ptu-control-valve", "PTU CONTROL VALVE", 29, BusId::DcEss, "pairs with PTU CONTROL VALVE's own actuator breaker");
    position_indication_load(net, cat, "apu-fuel-shutoff-valve", "APU FUEL SHUTOFF VALVE", 49, BusId::Dc1, "pairs with APU FUEL SHUTOFF VALVE's own actuator breaker");
    position_indication_load(net, cat, "crew-o2-shutoff", "CREW OXYGEN SHUTOFF VALVE", 35, BusId::Dc1, "pairs with CREW OXYGEN SHUTOFF VALVE's own actuator breaker");
    position_indication_load(net, cat, "cargo-door-fwd-actuator-ctl", "FWD CARGO DOOR", 52, BusId::Dc1, "pairs with the forward cargo door's own actuator-control breaker");
    position_indication_load(net, cat, "cargo-door-aft-actuator-ctl", "AFT CARGO DOOR", 52, BusId::Dc2, "pairs with the aft cargo door's own actuator-control breaker");
}

pub fn build(net: &mut Network) -> Catalog {
    let mut cat = Catalog::new();
    ata21(net, &mut cat);
    ata26(net, &mut cat);
    ata27(net, &mut cat);
    ata32(net, &mut cat);
    ata34(net, &mut cat);
    ata28_fuel(net, &mut cat);
    ata33_lighting(net, &mut cat);
    ata30_ice_protection(net, &mut cat);
    ata25_galleys(net, &mut cat);
    ata44_ife(net, &mut cat);
    avionics_misc(net, &mut cat);
    ata36_bleed(net, &mut cat);
    ata24_power_sources_extra(net, &mut cat);
    ata23_comms(net, &mut cat);
    ata31_recorders(net, &mut cat);
    ata31_ind_group(net, &mut cat);
    ata35_oxygen_extra(net, &mut cat);
    ata49_apu_extra(net, &mut cat);
    ata73_74_engine(net, &mut cat);
    ata26_extinguishing(net, &mut cat);
    ata29_hydraulics_extra(net, &mut cat);
    ata52_doors(net, &mut cat);
    ata33_emergency_lighting(net, &mut cat);
    position_indication_supplies(net, &mut cat);
    cat
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_full_catalogue_builds_without_duplicate_ids_and_every_load_has_its_own_breaker() {
        let mut net = Network::new();
        let cat = build(&mut net);
        let mut ids: Vec<&str> = net.loads.iter().map(|l| l.spec.id).collect();
        let before = ids.len();
        ids.sort_unstable();
        ids.dedup();
        assert_eq!(ids.len(), before, "duplicate load id in the catalogue");
        assert!(net.breakers.len() >= net.loads.len(), "every load needs at least one breaker: {} breakers, {} loads", net.breakers.len(), net.loads.len());
        for load in &net.loads {
            assert!(!load.feeds.is_empty(), "{} has no feeds at all", load.spec.id);
        }
        assert!(net.loads.len() > 200, "expected a substantial catalogue, got {}", net.loads.len());
        assert_eq!(cat.galley.len() + cat.commercial.len() + cat.essential.len() + cat.other.len(), net.loads.len());
    }

    #[test]
    fn every_load_has_a_positive_rating_and_a_cited_basis() {
        let mut net = Network::new();
        build(&mut net);
        for load in &net.loads {
            assert!(load.spec.rated_power_w > 0.0, "{} has no rated power", load.spec.id);
            assert!(!load.spec.basis.is_empty(), "{} has no basis citation", load.spec.id);
        }
    }

    #[test]
    fn galleys_and_ife_are_flagged_commercial_shed_candidates_not_essential() {
        let mut net = Network::new();
        let cat = build(&mut net);
        assert!(!cat.galley.is_empty());
        assert!(!cat.commercial.is_empty());
        for &i in &cat.galley {
            assert_eq!(net.loads[i].spec.ata, 25);
        }
        for &i in &cat.essential {
            assert_ne!(net.loads[i].spec.ata, 25, "a galley should never be marked essential");
        }
    }

    #[test]
    fn fuel_pumps_and_valves_are_each_their_own_load() {
        let mut net = Network::new();
        build(&mut net);
        let pumps = net.loads.iter().filter(|l| l.spec.id.starts_with("fuel-pump-")).count();
        let valves = net.loads.iter().filter(|l| l.spec.id.starts_with("fuel-valve-") && !l.spec.id.ends_with("-pos-ind")).count();
        let valve_pos_ind = net.loads.iter().filter(|l| l.spec.id.starts_with("fuel-valve-") && l.spec.id.ends_with("-pos-ind")).count();
        assert_eq!(pumps, 21);
        assert_eq!(valves, 21);
        assert_eq!(valve_pos_ind, 21);
    }

    #[test]
    fn every_closed_gap_id_is_a_real_load() {
        let mut net = Network::new();
        build(&mut net);
        let ids: std::collections::HashSet<&str> = net.loads.iter().map(|l| l.spec.id).collect();
        let mut expected: Vec<String> = vec![
            "ext-pwr-contactor".into(),
            "bat-charge-limiter-1".into(),
            "bat-charge-limiter-2".into(),
            "satcom".into(),
            "hf-1".into(),
            "hf-2".into(),
            "acars-mu".into(),
            "pa-amplifier".into(),
            "interphone".into(),
            "dfdr".into(),
            "cvr".into(),
            "qar".into(),
            "crew-o2-shutoff".into(),
            "pax-o2-gen-ctl".into(),
            "o2-pressure-xducer".into(),
            "apu-ecu-a".into(),
            "apu-ecu-b".into(),
            "apu-fuel-shutoff-valve".into(),
            "apu-start-contactor".into(),
            "rat-deploy-solenoid".into(),
            "ptu-control-valve".into(),
            "cargo-door-fwd-actuator-ctl".into(),
            "cargo-door-aft-actuator-ctl".into(),
            "emer-lighting-charger-1".into(),
            "emer-lighting-charger-2".into(),
            "ext-service-lighting".into(),
        ];
        for n in 1..=4 {
            expected.push(format!("fadec-{n}a"));
            expected.push(format!("fadec-{n}b"));
            expected.push(format!("ignition-{n}a"));
            expected.push(format!("ignition-{n}b"));
        }
        for bottle in 1..=2 {
            for squib in 1..=2 {
                expected.push(format!("eng-fire-bottle-{bottle}-squib-{squib}"));
            }
        }
        expected.push("apu-fire-bottle-squib-1".into());
        expected.push("apu-fire-bottle-squib-2".into());
        for (id, _, _) in FUEL_VALVES {
            expected.push(format!("{id}-pos-ind"));
        }
        for parent in ["hotair-1", "hotair-2", "fwd-isol-valve", "bulk-isol-valve", "ptu-control-valve", "apu-fuel-shutoff-valve", "crew-o2-shutoff", "cargo-door-fwd-actuator-ctl", "cargo-door-aft-actuator-ctl"] {
            expected.push(format!("{parent}-pos-ind"));
        }
        for pack in 1..=2 {
            for side in 1..=2 {
                expected.push(format!("pack-{pack}-flow-valve-{side}-pos-ind"));
            }
        }
        for n in 1..=4 {
            expected.push(format!("bleed-eng-{n}-pos-ind"));
        }
        assert_eq!(expected.len(), 86, "the expected list itself must total 86 (89 minus the 3 battery-output breakers; was 128/125 before the 60 generic fuel-valve-N-pos-ind ids were replaced by the 21 real fuel valves' own pos-ind ids)");
        let mut missing: Vec<&String> = expected.iter().filter(|id| !ids.contains(id.as_str())).collect();
        missing.sort();
        assert!(missing.is_empty(), "gap-closing ids with no real load: {missing:?}");

        for id in ["bat-1", "bat-2", "bat-apu"] {
            assert!(!ids.contains(id), "{id} is a source's own output breaker, not a consumer -- it must not have a Load");
        }
    }

    #[test]
    fn every_gap_closing_load_defaults_on_like_every_other_catalogue_entry() {
        let mut net = Network::new();
        build(&mut net);
        for id in ["ignition-1a", "eng-fire-bottle-1-squib-1", "apu-start-contactor", "cargo-door-fwd-actuator-ctl", "rat-deploy-solenoid"] {
            let idx = net.load_index(id).unwrap_or_else(|| panic!("{id} missing"));
            assert!(net.loads[idx].commanded_on, "{id} should default on in the raw catalogue; live.rs is what gates it");
        }
    }

    #[test]
    fn cargo_ventilation_loads_sit_on_their_real_bus() {
        let mut net = Network::new();
        build(&mut net);
        let bus_of = |id: &str| net.loads[net.load_index(id).unwrap_or_else(|| panic!("no load {id}"))].spec.bus.label();
        let freq_of = |id: &str| net.loads[net.load_index(id).unwrap_or_else(|| panic!("no load {id}"))].spec.rated_frequency_hz;
        assert_eq!(bus_of("fwd-isol-valve"), "DC1", "VCM Fwd's own primary channel is DC1 (411PP), not DC2 (that's VCM Aft's)");
        assert_eq!(bus_of("bulk-isol-valve"), "DC2", "VCM Aft's own primary channel is DC2 (214PP), matching fwd-isol-valve's primary-channel convention, not its DC_ESS standby channel");
        assert_eq!(bus_of("fwd-extract-fan"), "AC1", "the forward extraction fan's own dedicated bus (ForwardCargoVentilationControlSystem::new), not any VCM channel bus");
        assert_eq!(bus_of("bulk-extract-fan"), "AC4", "the bulk extraction fan's own dedicated bus (BulkVentilationControlSystem::new), not any VCM channel bus");
        assert_eq!(freq_of("fwd-extract-fan"), VF_MOTOR_RATED_FREQUENCY_HZ, "now a VFG-fed direct-drive fan motor like CAB FAN 1-4, not a frequency-insensitive DC valve actuator");
        assert_eq!(freq_of("bulk-extract-fan"), VF_MOTOR_RATED_FREQUENCY_HZ, "same as fwd-extract-fan");
        assert_eq!(freq_of("fwd-isol-valve"), 0.0, "the isolation valve is a DC motor-operated actuator, not a VFG-fed motor -- unchanged by this fix");
        assert_eq!(freq_of("bulk-isol-valve"), 0.0, "same as fwd-isol-valve");
    }

    #[test]
    fn vcm_channel_loads_sit_on_their_real_bus() {
        let mut net = Network::new();
        build(&mut net);
        let bus_of = |id: &str| net.loads[net.load_index(id).unwrap_or_else(|| panic!("no load {id}"))].spec.bus.label();
        assert_eq!(bus_of("vcm-fwd-1"), "DC1", "VCM Fwd's own primary channel is DC1 (411PP), not DC2 (that's VCM Aft's channel 1)");
        assert_eq!(bus_of("vcm-fwd-2"), "DC_ESS", "VCM Fwd's standby channel (109PP)");
        assert_eq!(bus_of("vcm-aft-1"), "DC2", "VCM Aft's own primary channel is DC2 (214PP)");
        assert_eq!(bus_of("vcm-aft-2"), "DC_ESS", "VCM Aft's standby channel (109PP), same physical bus as vcm-fwd-2");
    }
}
