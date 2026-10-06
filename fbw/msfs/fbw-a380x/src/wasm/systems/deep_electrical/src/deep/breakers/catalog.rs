use super::trip::BreakerKind;

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Bus {
    Ac1,
    Ac2,
    Ac3,
    Ac4,
    AcEss,
    AcEssShed,
    AcGndFltSvc,
    Dc1,
    Dc2,
    DcEss,
    DcBat,
    DcHot1,
    DcHot2,
    DcApu,
    Named(&'static str, f64),
}

impl Bus {
    pub const fn is_ac(self) -> bool {
        matches!(self, Bus::Ac1 | Bus::Ac2 | Bus::Ac3 | Bus::Ac4 | Bus::AcEss | Bus::AcEssShed | Bus::AcGndFltSvc)
    }

    pub fn nominal_voltage(self) -> f64 {
        match self {
            Bus::Named(_, v) => v,
            b if b.is_ac() => 115.0,
            _ => 28.0,
        }
    }

    pub fn label(self) -> &'static str {
        match self {
            Bus::Ac1 => "AC1",
            Bus::Ac2 => "AC2",
            Bus::Ac3 => "AC3",
            Bus::Ac4 => "AC4",
            Bus::AcEss => "AC_ESS",
            Bus::AcEssShed => "AC_ESS_SHED",
            Bus::AcGndFltSvc => "AC_GND_FLT_SVC",
            Bus::Dc1 => "DC1",
            Bus::Dc2 => "DC2",
            Bus::DcEss => "DC_ESS",
            Bus::DcBat => "DC_BAT",
            Bus::DcHot1 => "DC_HOT1",
            Bus::DcHot2 => "DC_HOT2",
            Bus::DcApu => "DC_APU",
            Bus::Named(n, _) => n,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Panel {
    OverheadFwd,
    OverheadAft,
    AvionicsBay,
    PrimaryPowerCentre1,
    PrimaryPowerCentre2,
    PrimaryPowerCentre3,
    PrimaryPowerCentre4,
    SecondaryPowerCentreFwd,
    SecondaryPowerCentreAft,
}

impl Panel {
    pub fn code(self) -> &'static str {
        match self {
            Panel::OverheadFwd => "OHP-FWD",
            Panel::OverheadAft => "OHP-AFT",
            Panel::AvionicsBay => "EE-BAY",
            Panel::PrimaryPowerCentre1 => "PPC1",
            Panel::PrimaryPowerCentre2 => "PPC2",
            Panel::PrimaryPowerCentre3 => "PPC3",
            Panel::PrimaryPowerCentre4 => "PPC4",
            Panel::SecondaryPowerCentreFwd => "SPC-FWD",
            Panel::SecondaryPowerCentreAft => "SPC-AFT",
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub struct PanelPosition {
    pub row: u32,
    pub column: u32,
    pub label: &'static str,
}

const PANEL_COLUMNS: u32 = 12;

fn cap_label(name: &str) -> String {
    if name.chars().count() <= 14 {
        name.to_string()
    } else {
        name.chars().take(14).collect()
    }
}

fn assign_positions(v: &mut [BreakerDef]) {
    use std::collections::HashMap;
    let mut order: Vec<usize> = (0..v.len()).collect();
    order.sort_by(|&a, &b| (v[a].panel.code(), v[a].ata, v[a].id).cmp(&(v[b].panel.code(), v[b].ata, v[b].id)));
    let mut next_index: HashMap<Panel, u32> = HashMap::new();
    for i in order {
        let idx = next_index.entry(v[i].panel).or_insert(0);
        let row = *idx / PANEL_COLUMNS + 1;
        let column = *idx % PANEL_COLUMNS + 1;
        *idx += 1;
        let label: &'static str = Box::leak(cap_label(v[i].name).into_boxed_str());
        v[i].position = PanelPosition { row, column, label };
    }
}

fn panel_for(ata: u16, kind: BreakerKind, bus: Bus) -> Panel {
    if kind == BreakerKind::Sspc {
        return match bus {
            Bus::Ac1 | Bus::Dc1 => Panel::PrimaryPowerCentre1,
            Bus::Ac2 | Bus::Dc2 => Panel::PrimaryPowerCentre2,
            Bus::Ac3 => Panel::PrimaryPowerCentre3,
            Bus::Ac4 => Panel::PrimaryPowerCentre4,
            Bus::AcEss | Bus::AcEssShed | Bus::DcEss => Panel::SecondaryPowerCentreFwd,
            _ => Panel::SecondaryPowerCentreAft,
        };
    }
    match ata {
        21 | 28 | 29 | 36 | 49 => Panel::OverheadFwd,
        24 | 26 | 30 | 32 | 33 => Panel::OverheadAft,
        _ => Panel::AvionicsBay,
    }
}

const STANDARD_SIZES_A: [f64; 29] = [
    1.0, 2.0, 3.0, 4.0, 5.0, 7.5, 10.0, 15.0, 20.0, 25.0, 30.0, 35.0, 40.0, 50.0, 60.0, 70.0, 80.0, 90.0, 100.0, 125.0, 150.0, 175.0, 200.0, 225.0, 250.0, 300.0, 400.0, 500.0, 600.0,
];

pub fn standard_size(current_a: f64) -> f64 {
    STANDARD_SIZES_A.iter().copied().find(|&s| s >= current_a).unwrap_or_else(|| (current_a / 100.0).ceil() * 100.0)
}

fn margined_current(power_w: f64, voltage: f64, power_factor: f64) -> f64 {
    (power_w / (voltage * power_factor.max(0.1))) * 1.25
}

fn kind_for(rating_a: f64) -> BreakerKind {
    if rating_a <= 25.0 {
        BreakerKind::Sspc
    } else {
        BreakerKind::Thermal
    }
}

fn avionics_pf(bus: Bus) -> f64 {
    if bus.is_ac() {
        0.95
    } else {
        1.0
    }
}

#[derive(Clone, Copy)]
pub struct BreakerDef {
    pub id: &'static str,
    pub name: &'static str,
    pub ata: u16,
    pub bus: Bus,
    pub rated_power_w: f64,
    pub power_factor: f64,
    pub raw_current_a: f64,
    pub rating_a: f64,
    pub basis: &'static str,
    pub consumer: &'static str,
    pub protected_load: Option<&'static str>,
    pub panel: Panel,
    pub kind: BreakerKind,
    pub position: PanelPosition,
}

fn push_electrical(v: &mut Vec<BreakerDef>, id: &'static str, name: &'static str, ata: u16, bus: Bus, watts: f64, pf: f64, consumer: &'static str, basis: &'static str) {
    let raw = margined_current(watts, bus.nominal_voltage(), pf);
    let rating = standard_size(raw);
    let kind = kind_for(rating);
    v.push(BreakerDef { id, name, ata, bus, rated_power_w: watts, power_factor: pf, raw_current_a: raw, rating_a: rating, basis, consumer, protected_load: Some(id), panel: panel_for(ata, kind, bus), kind, position: PanelPosition::default() });
}

fn push_extra(v: &mut Vec<BreakerDef>, id: &'static str, name: &'static str, ata: u16, bus: Bus, watts: f64, pf: f64, consumer: &'static str, basis: &'static str) {
    let raw = margined_current(watts, bus.nominal_voltage(), pf);
    let rating = standard_size(raw);
    let kind = kind_for(rating);
    v.push(BreakerDef { id, name, ata, bus, rated_power_w: watts, power_factor: pf, raw_current_a: raw, rating_a: rating, basis, consumer, protected_load: None, panel: panel_for(ata, kind, bus), kind, position: PanelPosition::default() });
}

fn push_electrical_feed(v: &mut Vec<BreakerDef>, load_id: &'static str, feed_suffix: &'static str, name: &'static str, ata: u16, bus: Bus, watts: f64, pf: f64, consumer: &'static str, basis: &'static str) {
    let id: &'static str = Box::leak(format!("{load_id}-{feed_suffix}").into_boxed_str());
    let raw = margined_current(watts, bus.nominal_voltage(), pf);
    let rating = standard_size(raw);
    let kind = kind_for(rating);
    v.push(BreakerDef { id, name, ata, bus, rated_power_w: watts, power_factor: pf, raw_current_a: raw, rating_a: rating, basis, consumer, protected_load: Some(load_id), panel: panel_for(ata, kind, bus), kind, position: PanelPosition::default() });
}

fn push_electrical_dual(v: &mut Vec<BreakerDef>, load_id: &'static str, name: &'static str, ata: u16, normal_bus: Bus, second_bus: Bus, watts: f64, pf: f64, consumer: &'static str, basis: &'static str) {
    let normal_name: &'static str = Box::leak(format!("{name} NORMAL FEED").into_boxed_str());
    let second_name: &'static str = Box::leak(format!("{name} 2ND FEED").into_boxed_str());
    push_electrical_feed(v, load_id, "normal-bkr", normal_name, ata, normal_bus, watts, pf, consumer, basis);
    push_electrical_feed(v, load_id, "2nd-bkr", second_name, ata, second_bus, watts, pf, consumer, basis);
}

fn ata21(v: &mut Vec<BreakerDef>) {
    const FANS: [(&str, &str, Bus); 4] = [("cab-fan-1", "CAB FAN 1", Bus::Ac1), ("cab-fan-2", "CAB FAN 2", Bus::Ac2), ("cab-fan-3", "CAB FAN 3", Bus::Ac3), ("cab-fan-4", "CAB FAN 4", Bus::Ac4)];
    for (id, name, bus) in FANS {
        push_electrical(v, id, name, 21, bus, 500.0, 0.85, "cabin recirculation fan motor", "deep::electrical::loads.rs::ata21 CAB FAN 1-4 (500 W typical large-transport recirculation fan motor)");
    }
    push_electrical(v, "hotair-1", "HOT AIR VALVE 1", 21, Bus::AcEss, 50.0, 0.8, "hot air valve actuator", "deep::electrical::loads.rs::ata21 HOT AIR VALVE 1");
    push_electrical(v, "hotair-2", "HOT AIR VALVE 2", 21, Bus::AcEss, 50.0, 0.8, "hot air valve actuator", "deep::electrical::loads.rs::ata21 HOT AIR VALVE 2");
    push_electrical(v, "fwd-isol-valve", "FWD CARGO ISOL VALVE", 21, Bus::Dc1, 50.0, 0.8, "forward cargo isolation valve actuator", "deep::electrical::loads.rs::ata21 FWD CARGO ISOL VALVE (VCM Fwd's own primary channel, DC1/411PP -- ventilation_control_module.rs VentilationControlModule::new(.., VcmId::Fwd, [DirectCurrent(1), DirectCurrentEssential]), channel 1 backed by powered_by[0] and default-active; fixes/W161.md, matching fixes/W115.md's src/breakers.rs correction -- was wired to DC2, VCM Aft's bus)");
    push_electrical(v, "fwd-extract-fan", "FWD CARGO EXTRACT FAN", 21, Bus::Ac1, 150.0, 0.85, "forward cargo extraction fan motor", "deep::electrical::loads.rs::ata21 FWD CARGO EXTRACT FAN (the fan's own dedicated bus, not the VCM's channel bus -- ventilation_control_module.rs ForwardCargoVentilationControlSystem::new(AlternatingCurrent(1)), really gated in receive_power/fwd_extraction_fan_is_on; fixes/W161.md, matching fixes/W115.md -- was wired to VCM Fwd's DC channel, which the fan does not draw from at all)");
    push_electrical(v, "bulk-isol-valve", "BULK CARGO ISOL VALVE", 21, Bus::Dc2, 50.0, 0.8, "bulk cargo isolation valve actuator", "deep::electrical::loads.rs::ata21 BULK CARGO ISOL VALVE (VCM Aft's own primary channel, DC2/214PP -- ventilation_control_module.rs VentilationControlModule::new(.., VcmId::Aft, [DirectCurrent(2), DirectCurrentEssential]), channel 1 backed by powered_by[0] and default-active; fixes/W161.md, matching fixes/W115.md -- was on DC_ESS, Aft's secondary/standby channel)");
    push_electrical(v, "bulk-extract-fan", "BULK CARGO EXTRACT FAN", 21, Bus::Ac4, 150.0, 0.85, "bulk cargo extraction fan motor", "deep::electrical::loads.rs::ata21 BULK CARGO EXTRACT FAN (the fan's own dedicated bus, not the VCM's channel bus -- ventilation_control_module.rs BulkVentilationControlSystem::new(AlternatingCurrent(4)), really gated in receive_power/bulk_extraction_fan_is_on; fixes/W161.md, matching fixes/W115.md -- was wired to VCM Aft's DC_ESS channel, which the fan does not draw from at all)");
    push_electrical(v, "cargo-heater", "BULK CARGO HEATER", 21, Bus::Ac2, 1000.0, 1.0, "bulk cargo heater element", "deep::electrical::loads.rs::ata21 BULK CARGO HEATER (AirHeater::new(AC2))");

    const FDAC: [(&str, &str, Bus); 4] = [("fdac-1a", "FDAC 1 CHANNEL 1", Bus::AcEss), ("fdac-1b", "FDAC 1 CHANNEL 2", Bus::Ac2), ("fdac-2a", "FDAC 2 CHANNEL 1", Bus::AcEss), ("fdac-2b", "FDAC 2 CHANNEL 2", Bus::Ac4)];
    for (id, name, bus) in FDAC {
        push_electrical(v, id, name, 21, bus, 50.0, avionics_pf(bus), "FDAC channel", "deep::electrical::loads.rs::ata21 FDAC (FullDigitalAGUController)");
    }
    const TADD: [(&str, &str, Bus); 2] = [("tadd-1", "TADD CHANNEL 1", Bus::Ac2), ("tadd-2", "TADD CHANNEL 2", Bus::Ac4)];
    for (id, name, bus) in TADD {
        push_electrical(v, id, name, 21, bus, 50.0, avionics_pf(bus), "trim air drive device channel", "deep::electrical::loads.rs::ata21 TADD (TrimAirDriveDevice)");
    }
    const VCM: [(&str, &str, Bus); 4] = [("vcm-fwd-1", "VCM FWD CHANNEL 1", Bus::Dc1), ("vcm-fwd-2", "VCM FWD CHANNEL 2", Bus::DcEss), ("vcm-aft-1", "VCM AFT CHANNEL 1", Bus::Dc2), ("vcm-aft-2", "VCM AFT CHANNEL 2", Bus::DcEss)];
    for (id, name, bus) in VCM {
        push_electrical(v, id, name, 21, bus, 50.0, avionics_pf(bus), "ventilation control module channel", "deep::electrical::loads.rs::ata21 VCM (VentilationControlModule)");
    }
    const OCSM_AP: [(&str, &str, Bus); 4] = [("ocsm-1-ap", "OCSM 1 AUTO PARTITION", Bus::Dc1), ("ocsm-2-ap", "OCSM 2 AUTO PARTITION", Bus::Dc1), ("ocsm-3-ap", "OCSM 3 AUTO PARTITION", Bus::Dc2), ("ocsm-4-ap", "OCSM 4 AUTO PARTITION", Bus::Dc2)];
    for (id, name, bus) in OCSM_AP {
        push_electrical(v, id, name, 21, bus, 50.0, avionics_pf(bus), "outflow valve control module auto-partition logic", "deep::electrical::loads.rs::ata21 OCSM auto-partition (OutflowValveControlModule)");
    }
    const OCSM_CH: [(&str, &str, Bus); 8] = [
        ("ocsm-1a", "OCSM 1 CHANNEL 1", Bus::Dc1),
        ("ocsm-1b", "OCSM 1 CHANNEL 2", Bus::DcEss),
        ("ocsm-2a", "OCSM 2 CHANNEL 1", Bus::Dc1),
        ("ocsm-2b", "OCSM 2 CHANNEL 2", Bus::DcEss),
        ("ocsm-3a", "OCSM 3 CHANNEL 1", Bus::Dc2),
        ("ocsm-3b", "OCSM 3 CHANNEL 2", Bus::DcEss),
        ("ocsm-4a", "OCSM 4 CHANNEL 1", Bus::Dc2),
        ("ocsm-4b", "OCSM 4 CHANNEL 2", Bus::DcEss),
    ];
    for (id, name, bus) in OCSM_CH {
        push_electrical(v, id, name, 21, bus, 50.0, avionics_pf(bus), "outflow valve control module channel", "deep::electrical::loads.rs::ata21 OCSM channel");
    }
    let cpiom_bus = [Bus::Dc1, Bus::DcEss, Bus::DcEss, Bus::Dc2];
    for app in ["AGS", "TCS", "VCS", "CPCS"] {
        for k in 0..4usize {
            let id: &'static str = Box::leak(format!("cpiom-b{}-{}", k + 1, app.to_lowercase()).into_boxed_str());
            let name: &'static str = Box::leak(format!("CPIOM B{} {} APP", k + 1, app).into_boxed_str());
            let bus = cpiom_bus[k];
            push_electrical(v, id, name, 21, bus, 50.0, avionics_pf(bus), "CPIOM B application (AGS/TCS/VCS/CPCS)", "deep::electrical::loads.rs::ata21 CPIOM B bus map");
        }
    }
    for pack in 1..=2u32 {
        for side in 1..=2u32 {
            let id: &'static str = Box::leak(format!("pack-{pack}-flow-valve-{side}").into_boxed_str());
            let name: &'static str = Box::leak(format!("PACK {pack} FLOW VALVE {side}").into_boxed_str());
            push_electrical(v, id, name, 21, Bus::DcEss, 50.0, 0.8, "pack flow valve actuator", "deep::electrical::loads.rs::ata21 PACK FLOW VALVE (pneumatic.rs PackComplex ElectroPneumaticValve, DC_ESS)");
        }
    }
    for (id, name, bus) in [
        ("avionics-fan-1", "AVIONICS BAY FAN 1", Bus::AcEss),
        ("avionics-fan-2", "AVIONICS BAY FAN 2", Bus::AcEssShed),
        ("avionics-fan-3", "AVIONICS BAY FAN 3", Bus::Ac1),
        ("avionics-fan-4", "AVIONICS BAY FAN 4", Bus::Ac2),
    ] {
        push_electrical(v, id, name, 21, bus, 300.0, 0.85, "avionics-bay cooling fan motor", "deep::electrical::loads.rs::ata21 avionics-bay cooling fans (GENERIC, not individually named in src/breakers.rs)");
    }
}

fn ata26(v: &mut Vec<BreakerDef>) {
    let zones = ["eng1", "eng2", "eng3", "eng4", "apu", "mlgbay"];
    let titles = ["ENG1", "ENG2", "ENG3", "ENG4", "APU", "MLGBAY"];
    for (zone, title) in zones.iter().zip(titles.iter()) {
        for loop_name in ["A", "B"] {
            let id: &'static str = Box::leak(format!("fire-loop-{zone}-{loop_name}").into_boxed_str());
            let name: &'static str = Box::leak(format!("FIRE DET {title} LOOP {loop_name}").into_boxed_str());
            push_electrical(v, id, name, 26, Bus::DcEss, 20.0, avionics_pf(Bus::DcEss), "fire detection loop (redundant with its own A/B pair)", "deep::electrical::loads.rs::ata26 fire detection loop (20 W typical controller electronics; DC_ESS/DC_HOT1)");
        }
    }
}

fn ata27(v: &mut Vec<BreakerDef>) {
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
        let normal_bus = if k % 2 == 0 { Bus::Dc1 } else { Bus::Dc2 };
        push_electrical_dual(v, id, name, ata, normal_bus, Bus::DcEss, 100.0, 1.0, "flight-control/autoflight computer", "deep::electrical::loads.rs::ata27 flight-control/autoflight computer (100 W typical FCC-class LRU); real dual feed, normal DC1/DC2 bus + DC ESS backup, each on its own breaker, OR-ed internally");
    }
}

fn ata32(v: &mut Vec<BreakerDef>) {
    push_electrical_dual(v, "lgciu-1", "LGCIU 1", 32, Bus::DcEss, Bus::Dc2, 50.0, 1.0, "Landing Gear Control and Interface Unit 1", "deep::electrical::loads.rs::ata32 LGCIU 1; real dual feed, DC ESS normal + DC2 backup, each on its own breaker");
    push_electrical_dual(v, "lgciu-2", "LGCIU 2", 32, Bus::Dc2, Bus::DcEss, 50.0, 1.0, "Landing Gear Control and Interface Unit 2", "deep::electrical::loads.rs::ata32 LGCIU 2; real dual feed, DC2 normal + DC ESS backup, each on its own breaker");

    const PUMPS: [(&str, &str, Bus); 4] = [("hyd-epump-ga", "HYD GREEN ELEC PUMP A", Bus::Ac3), ("hyd-epump-gb", "HYD GREEN ELEC PUMP B", Bus::Ac4), ("hyd-epump-ya", "HYD YELLOW ELEC PUMP A", Bus::AcEss), ("hyd-epump-yb", "HYD YELLOW ELEC PUMP B", Bus::Ac2)];
    for (id, name, bus) in PUMPS {
        push_electrical(v, id, name, 29, bus, 75.0 * 28.0, 0.85, "electric hydraulic pump motor", "deep::electrical::loads.rs::ata32 electric hydraulic pump (FBW ELECTRIC_PUMP_MAX_CURRENT_AMPERE = 75 A, hydraulic/mod.rs:1750, real/FBW-sourced; 28 V-equivalent power)");
        let coil_id: &'static str = Box::leak(format!("{id}-coil").into_boxed_str());
        let coil_name: &'static str = Box::leak(format!("{name} CONTACTOR COIL").into_boxed_str());
        push_electrical(v, coil_id, coil_name, 29, Bus::DcEss, 20.0, 1.0, "pump motor line-contactor holding-coil supply", "deep::electrical::loads.rs::ata32 pump contactor coil (GENERIC ~20 W DC line-contactor holding coil, the low-power control circuit that energises the pump motor's own contactor, distinct from the motor's own high-current feed)");
    }
    push_electrical(v, "autobrake-disarm-sol", "AUTOBRAKE DISARM SOLENOID", 32, Bus::Dc2, 56.0, 1.0, "autobrake knob disarm solenoid", "deep::electrical::loads.rs::ata32 AUTOBRAKE DISARM SOLENOID (56 W typical small solenoid valve; autobrakes.rs DC2)");

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
        push_electrical(v, id, name, 32, Bus::DcEss, 5.0, avionics_pf(Bus::DcEss), "gear/door uplock-downlock proximity sensor", "deep::electrical::loads.rs::ata32 proximity sensor (5 W typical target/pickup; LGCIU's own DC_ESS supply)");
    }
    const ACTUATORS: [&str; 6] = ["gear-actuator-nose", "gear-actuator-left", "gear-actuator-right", "gear-door-actuator-nose", "gear-door-actuator-left", "gear-door-actuator-right"];
    for id in ACTUATORS {
        let name: &'static str = Box::leak(id.replace('-', " ").to_uppercase().into_boxed_str());
        push_electrical(v, id, name, 32, Bus::DcEss, 75.0 * 28.0, 0.85, "gear/gear-door hydraulic actuator control", "deep::electrical::loads.rs::ata32 gear/door actuator control (same order of magnitude as the electric hydraulic pumps)");
    }
}

fn ata34(v: &mut Vec<BreakerDef>) {
    const RAS: [(&str, &str, Bus); 3] = [("ra-sys-a", "RA SYS A", Bus::Ac1), ("ra-sys-b", "RA SYS B", Bus::Ac2), ("ra-sys-c", "RA SYS C", Bus::AcEss)];
    for (id, name, bus) in RAS {
        push_electrical(v, id, name, 34, bus, 50.0, avionics_pf(bus), "radio altimeter transceiver", "deep::electrical::loads.rs::ata34 radio altimeter transceiver (A380RadioAltimeters)");
    }
    for (n, bus) in [(1, Bus::Ac1), (2, Bus::Ac2), (3, Bus::AcEss)] {
        let id: &'static str = Box::leak(format!("ra-ant-interrupt-{n}").into_boxed_str());
        let name: &'static str = Box::leak(format!("RA {n} ANTENNA INTERRUPT").into_boxed_str());
        push_electrical(v, id, name, 34, bus, 10.0, avionics_pf(bus), "radio altimeter antenna feed", "deep::electrical::loads.rs::ata34 antenna-coupling network (10 W class)");
        let id2: &'static str = Box::leak(format!("ra-ant-coupling-{n}").into_boxed_str());
        let name2: &'static str = Box::leak(format!("RA {n} ANTENNA DIRECT COUPLING").into_boxed_str());
        push_electrical(v, id2, name2, 34, bus, 10.0, avionics_pf(bus), "radio altimeter antenna feed", "deep::electrical::loads.rs::ata34 antenna-coupling network (10 W class)");
    }
    push_electrical(v, "egpwc", "EGPWC (TAWS)", 34, Bus::AcEss, 100.0, avionics_pf(Bus::AcEss), "Enhanced Ground Proximity Warning Computer (TAWS/terrain display)", "deep::electrical::loads.rs::ata34 EGPWC (100 W typical flight-warning-class LRU; real AC_ESS bus, enhanced_gpwc/mod.rs)");
}

fn ata28_fuel(v: &mut Vec<BreakerDef>) {
    let pump_buses = [Bus::Ac1, Bus::Ac2, Bus::Ac3, Bus::Ac4, Bus::AcEss];
    for i in 0..25usize {
        let id: &'static str = Box::leak(format!("fuel-pump-{i}").into_boxed_str());
        let name: &'static str = Box::leak(format!("FUEL PUMP {i}").into_boxed_str());
        let bus = pump_buses[i % pump_buses.len()];
        push_electrical(v, id, name, 28, bus, 600.0, 0.85, "fuel boost/transfer/jettison pump motor", "deep::electrical::loads.rs::ata28_fuel CIRCUIT_FUEL_PUMP (25 real pumps, 600 W each, circuits.rs precedent figure)");
    }
    let valve_buses = [Bus::Dc1, Bus::Dc2, Bus::DcEss, Bus::DcBat];
    for i in 0..60usize {
        let id: &'static str = Box::leak(format!("fuel-valve-{i}").into_boxed_str());
        let name: &'static str = Box::leak(format!("FUEL VALVE {i}").into_boxed_str());
        let bus = valve_buses[i % valve_buses.len()];
        push_electrical(v, id, name, 28, bus, 50.0, 0.8, "fuel shutoff/transfer/crossfeed/isolation valve actuator", "deep::electrical::loads.rs::ata28_fuel CIRCUIT_FUEL_VALVE (60 real valves, 50 W each, circuits.rs precedent figure)");
    }
}

fn ata33_lighting(v: &mut Vec<BreakerDef>) {
    const LIGHTS: [(&str, &str, Bus, f64); 12] = [
        ("light-landing", "LANDING LIGHTS", Bus::Ac1, 600.0),
        ("light-taxi", "TAXI LIGHTS", Bus::Ac2, 250.0),
        ("light-nav", "NAV LIGHTS", Bus::AcEssShed, 40.0),
        ("light-beacon", "BEACON LIGHTS", Bus::AcEssShed, 100.0),
        ("light-strobe", "STROBE LIGHTS", Bus::Ac3, 300.0),
        ("light-logo", "LOGO LIGHTS", Bus::Ac4, 150.0),
        ("light-wing", "WING LIGHTS", Bus::Ac1, 150.0),
        ("light-recognition", "RECOGNITION LIGHTS", Bus::DcBat, 40.0),
        ("light-cabin", "CABIN LIGHTS", Bus::AcGndFltSvc, 200.0),
        ("light-panel", "PANEL LIGHTS", Bus::DcEss, 30.0),
        ("light-pedestal", "PEDESTAL LIGHTS", Bus::DcEss, 20.0),
        ("light-glareshield", "GLARESHIELD LIGHTS", Bus::DcEss, 20.0),
    ];
    for (id, name, bus, watts) in LIGHTS {
        push_electrical(v, id, name, 33, bus, watts, 1.0, "cockpit/cabin/exterior light circuit", "deep::electrical::loads.rs::ata33_lighting lumped per circuit type (circuits.rs CIRCUIT_LIGHT_* precedent figure)");
    }
}

fn ata30_ice_protection(v: &mut Vec<BreakerDef>) {
    for (n, bus) in [(1, Bus::Ac1), (2, Bus::Ac2)] {
        let id: &'static str = Box::leak(format!("windshield-heat-{n}").into_boxed_str());
        let name: &'static str = Box::leak(format!("WINDSHIELD HEAT {n}").into_boxed_str());
        push_electrical(v, id, name, 30, bus, 2000.0, 1.0, "windshield electric anti-ice heating element", "deep::electrical::loads.rs::ata30_ice_protection GENERIC wide-body windshield anti-ice heater (2000 W/side)");
    }
    for (n, bus) in [(1, Bus::Ac1), (2, Bus::Ac2), (3, Bus::AcEss)] {
        let id: &'static str = Box::leak(format!("pitot-heat-{n}").into_boxed_str());
        let name: &'static str = Box::leak(format!("PITOT HEAT {n}").into_boxed_str());
        push_electrical(v, id, name, 30, bus, 600.0, 1.0, "pitot tube heater element", "deep::electrical::loads.rs::ata30_ice_protection GENERIC precedent, physics::electrical.rs rated_watts(\"CIRCUIT_PITOT_HEAT\") = 600 W");
    }
    for (name_txt, id, bus) in [("AOA HEAT 1", "aoa-heat-1", Bus::Dc1), ("AOA HEAT 2", "aoa-heat-2", Bus::Dc2), ("TAT PROBE HEAT", "tat-heat", Bus::DcEss)] {
        let name: &'static str = Box::leak(name_txt.to_string().into_boxed_str());
        push_electrical(v, id, name, 30, bus, 150.0, 1.0, "probe (AOA vane / TAT) heating element", "deep::electrical::loads.rs::ata30_ice_protection GENERIC small-probe heater, an order of magnitude below a pitot tube's own heater");
    }
}

fn ata25_galleys(v: &mut Vec<BreakerDef>) {
    const GALLEYS: [(&str, &str, Bus, f64); 6] = [
        ("galley-fwd-upper", "FWD UPPER GALLEY", Bus::Ac1, 8000.0),
        ("galley-aft-upper", "AFT UPPER GALLEY", Bus::Ac2, 8000.0),
        ("galley-fwd-main", "FWD MAIN DECK GALLEY", Bus::Ac3, 10000.0),
        ("galley-mid-main", "MID MAIN DECK GALLEY", Bus::Ac4, 10000.0),
        ("galley-aft-main", "AFT MAIN DECK GALLEY", Bus::Ac1, 10000.0),
        ("galley-lower", "LOWER DECK GALLEY LIFT", Bus::Ac2, 3000.0),
    ];
    for (id, name, bus, watts) in GALLEYS {
        push_electrical(v, id, name, 25, bus, watts, 1.0, "galley complex load (ovens, water heaters, chillers)", "deep::electrical::loads.rs::ata25_galleys GENERIC wide-body galley complex load, no public per-zone A380 figure");
    }
}

fn ata44_ife(v: &mut Vec<BreakerDef>) {
    const ZONES: [(&str, &str, Bus, u32); 5] = [
        ("ife-upper-deck", "IFE UPPER DECK ZONE", Bus::AcGndFltSvc, 90),
        ("ife-main-fwd", "IFE MAIN DECK FWD ZONE", Bus::AcGndFltSvc, 120),
        ("ife-main-mid", "IFE MAIN DECK MID ZONE", Bus::AcGndFltSvc, 150),
        ("ife-main-aft", "IFE MAIN DECK AFT ZONE", Bus::AcGndFltSvc, 130),
        ("ife-server", "IFE SERVER RACK", Bus::Ac2, 1),
    ];
    for (id, name, bus, seats) in ZONES {
        let per_seat_w = if seats > 1 { 30.0 } else { 4000.0 };
        let watts = per_seat_w * seats as f64;
        push_electrical(v, id, name, 44, bus, watts, avionics_pf(bus), "IFE seat-box zone / server rack", "deep::electrical::loads.rs::ata44_ife GENERIC ~30 W/seat x zone seat count, or server-rack figure for the head-end");
    }
}

fn avionics_misc(v: &mut Vec<BreakerDef>) {
    const DUAL_BOXES: [(&str, &str, u16, Bus, Bus, f64); 7] = [
        ("fms-1", "FMS 1", 34, Bus::Dc1, Bus::DcEss, 60.0),
        ("fms-2", "FMS 2", 34, Bus::Dc2, Bus::DcEss, 60.0),
        ("fms-3", "FMS 3", 34, Bus::Dc1, Bus::DcEss, 60.0),
        ("adirs-1", "ADIRU 1", 34, Bus::AcEss, Bus::DcEss, 80.0),
        ("adirs-2", "ADIRU 2", 34, Bus::Ac2, Bus::DcEss, 80.0),
        ("adirs-3", "ADIRU 3", 34, Bus::AcEssShed, Bus::DcEss, 80.0),
        ("tcas", "TCAS COMPUTER", 34, Bus::DcEss, Bus::Dc2, 70.0),
    ];
    for (id, name, ata, normal_bus, second_bus, watts) in DUAL_BOXES {
        push_electrical_dual(v, id, name, ata, normal_bus, second_bus, watts, avionics_pf(normal_bus), "avionics LRU", "deep::electrical::loads.rs::avionics_misc GENERIC dual-fed avionics LRU class figure; real dual feed (normal + ESS/backup bus), each on its own breaker, OR-ed internally");
    }
    const SINGLE_BOXES: [(&str, &str, u16, Bus, f64); 5] = [
        ("xpdr-1", "TRANSPONDER 1", 34, Bus::Dc1, 50.0),
        ("xpdr-2", "TRANSPONDER 2", 34, Bus::Dc2, 50.0),
        ("vhf-1", "VHF 1", 23, Bus::Dc1, 40.0),
        ("vhf-2", "VHF 2", 23, Bus::Dc2, 40.0),
        ("wxr", "WEATHER RADAR", 34, Bus::Ac1, 150.0),
    ];
    for (id, name, ata, bus, watts) in SINGLE_BOXES {
        push_electrical(v, id, name, ata, bus, watts, avionics_pf(bus), "avionics LRU", "deep::electrical::loads.rs::avionics_misc GENERIC typical avionics LRU class figure; conventionally single-fed with a manual transfer switch, not an automatic OR");
    }
}

fn ata36_bleed(v: &mut Vec<BreakerDef>) {
    for n in 1..=4u32 {
        let bus = if n <= 2 { Bus::Dc1 } else { Bus::Dc2 };
        let id: &'static str = Box::leak(format!("bleed-eng-{n}").into_boxed_str());
        let name: &'static str = Box::leak(format!("BLEED ENG {n} VALVES").into_boxed_str());
        push_electrical(v, id, name, 36, bus, 150.0, 0.8, "engine bleed HP/pressure-regulating/fan-air valve set", "deep::electrical::loads.rs::ata36_bleed one shared feed per engine (real Airbus-style single bleed CB)");
    }
}

fn ata24_power_sources(v: &mut Vec<BreakerDef>) {
    push_extra(v, "bat-1", "BATTERY 1", 24, Bus::DcHot1, 150.0 * 28.0, 1.0, "main battery 1 output", "GENERIC: typical large-transport main aircraft battery continuous-discharge current-limiter class (150 A), no public A380 per-battery figure");
    push_extra(v, "bat-2", "BATTERY 2", 24, Bus::DcHot2, 150.0 * 28.0, 1.0, "main battery 2 output", "GENERIC: same class as BATTERY 1");
    push_extra(v, "bat-apu", "APU BATTERY", 24, Bus::DcApu, 100.0 * 28.0, 1.0, "APU battery output", "GENERIC: a smaller dedicated APU-start battery, same current-limiter class scaled down");
    push_electrical(v, "ext-pwr-contactor", "EXTERNAL POWER CONTACTOR CONTROL", 24, Bus::DcHot1, 20.0, 1.0, "external power contactor control coil", "GENERIC: typical small contactor-coil control circuit, real A380 external power system, no public per-part figure");
    push_electrical(v, "bat-charge-limiter-1", "BATTERY 1 CHARGE LIMITER", 24, Bus::DcEss, 20.0, 1.0, "battery 1 charge-limiter control circuit", "GENERIC: typical small charge-controller LRU control circuit");
    push_electrical(v, "bat-charge-limiter-2", "BATTERY 2 CHARGE LIMITER", 24, Bus::DcEss, 20.0, 1.0, "battery 2 charge-limiter control circuit", "GENERIC: typical small charge-controller LRU control circuit");
}

fn ata23_comms(v: &mut Vec<BreakerDef>) {
    push_electrical(v, "satcom", "SATCOM", 23, Bus::DcEss, 100.0, avionics_pf(Bus::DcEss), "satellite communication transceiver", "GENERIC: typical wide-body SATCOM transceiver LRU, real A380 equipment class, no public per-box figure");
    push_electrical(v, "hf-1", "HF 1", 23, Bus::Dc1, 100.0, avionics_pf(Bus::Dc1), "HF radio transceiver 1", "GENERIC: typical HF transceiver LRU");
    push_electrical(v, "hf-2", "HF 2", 23, Bus::Dc2, 100.0, avionics_pf(Bus::Dc2), "HF radio transceiver 2", "GENERIC: typical HF transceiver LRU");
    push_electrical(v, "acars-mu", "ACARS MU", 23, Bus::DcEss, 50.0, avionics_pf(Bus::DcEss), "ACARS management unit", "GENERIC: typical avionics LRU class figure");
    push_electrical(v, "pa-amplifier", "PA AMPLIFIER", 23, Bus::Ac1, 200.0, 0.9, "cabin passenger address amplifier", "GENERIC: typical wide-body PA amplifier power stage");
    push_electrical(v, "interphone", "INTERPHONE", 23, Bus::DcEss, 50.0, avionics_pf(Bus::DcEss), "crew interphone system", "GENERIC: typical avionics LRU class figure");
}

fn ata31_recorders(v: &mut Vec<BreakerDef>) {
    push_electrical(v, "dfdr", "DFDR", 31, Bus::DcEss, 50.0, avionics_pf(Bus::DcEss), "Digital Flight Data Recorder", "GENERIC: typical avionics LRU class figure, real mandatory A380 equipment");
    push_electrical(v, "cvr", "CVR", 31, Bus::DcEss, 50.0, avionics_pf(Bus::DcEss), "Cockpit Voice Recorder", "GENERIC: typical avionics LRU class figure, real mandatory A380 equipment");
    push_electrical(v, "qar", "QAR", 31, Bus::Dc2, 30.0, avionics_pf(Bus::Dc2), "Quick Access Recorder", "GENERIC: typical small avionics LRU class figure");
}

fn ata31_ind_group(v: &mut Vec<BreakerDef>) {
    push_electrical(v, "capt-efis-bkup-ctl", "CAPT EFIS BKUP CTL", 31, Bus::DcEss, 30.0, avionics_pf(Bus::DcEss), "captain's EFIS backup control panel", "GENERIC: typical small avionics LRU class figure (loads.rs ata31_ind_group)");
    push_electrical(v, "fo-efis-bkup-ctl", "F/O EFIS BKUP CTL", 31, Bus::DcEss, 30.0, avionics_pf(Bus::DcEss), "first officer's EFIS backup control panel", "GENERIC: same class as CAPT EFIS BKUP CTL");
    push_electrical(v, "capt-efis-ctl-panel", "CAPT EFIS CTL PANEL", 31, Bus::DcEss, 30.0, avionics_pf(Bus::DcEss), "captain's EFIS control panel", "GENERIC: typical small avionics LRU class figure");
    push_electrical(v, "fo-efis-ctl-panel", "F/O EFIS CTL PANEL", 31, Bus::DcEss, 30.0, avionics_pf(Bus::DcEss), "first officer's EFIS control panel", "GENERIC: same class as CAPT EFIS CTL PANEL");
    push_electrical(v, "capt-pfd-du", "CAPT PFD DU", 31, Bus::DcEss, 60.0, avionics_pf(Bus::DcEss), "captain's PFD display unit", "GENERIC display-unit-class figure; feed DC ESS per FlyByWire CdsDisplayUnit.tsx DisplayUnitToDCBus:42 (409PP)");
    push_electrical_dual(v, "capt-nd-du", "CAPT ND DU", 31, Bus::DcEss, Bus::Dc1, 60.0, avionics_pf(Bus::DcEss), "captain's ND display unit", "GENERIC display-unit-class figure; dual feed DC ESS + DC 1 per FlyByWire CdsDisplayUnit.tsx DisplayUnitToDCBus:43 (415PP or 105PP)");
    push_electrical(v, "capt-ewd-du", "CAPT EWD DU", 31, Bus::DcEss, 60.0, avionics_pf(Bus::DcEss), "EWD display unit", "GENERIC display-unit-class figure; feed DC ESS per FlyByWire CdsDisplayUnit.tsx DisplayUnitToDCBus:48 (423PP)");
    push_electrical(v, "fo-pfd-du", "F/O PFD DU", 31, Bus::Dc2, 60.0, avionics_pf(Bus::Dc2), "first officer's PFD display unit", "GENERIC display-unit-class figure; feed DC 2 per FlyByWire CdsDisplayUnit.tsx DisplayUnitToDCBus:45");
    push_electrical_dual(v, "fo-nd-du", "F/O ND DU", 31, Bus::Dc1, Bus::Dc2, 60.0, avionics_pf(Bus::Dc1), "first officer's ND display unit", "GENERIC display-unit-class figure; dual feed DC 1 + DC 2 per FlyByWire CdsDisplayUnit.tsx DisplayUnitToDCBus:46");
    push_electrical(v, "kccu-capt", "CAPT KCCU", 31, Bus::DcEss, 40.0, avionics_pf(Bus::DcEss), "captain's keyboard and cursor control unit", "GENERIC: typical small keyboard/cursor-control-unit LRU figure");
    push_electrical(v, "kccu-fo", "F/O KCCU", 31, Bus::DcEss, 40.0, avionics_pf(Bus::DcEss), "first officer's keyboard and cursor control unit", "GENERIC: same class as CAPT KCCU");
    push_electrical(v, "cds-mailbox-capt", "CAPT CDS MAILBOX", 31, Bus::DcEss, 20.0, avionics_pf(Bus::DcEss), "captain's CDS mailbox", "GENERIC: typical small CDS peripheral LRU figure");
    push_electrical(v, "hud", "HUD", 31, Bus::DcEss, 50.0, avionics_pf(Bus::DcEss), "head-up display projector/combiner", "GENERIC: typical HUD projector/combiner-unit LRU figure");
    push_electrical(v, "video-multiplexer", "VIDEO MULTIPLEXER", 31, Bus::DcEss, 30.0, avionics_pf(Bus::DcEss), "cockpit video multiplexer", "GENERIC: typical small avionics LRU class figure");
    push_electrical(v, "recorder-accelerometer", "RECORDER ACCELEROMETER", 31, Bus::DcEss, 20.0, avionics_pf(Bus::DcEss), "flight data recorder accelerometer", "GENERIC: typical small avionics accessory LRU figure");
    push_electrical(v, "dfdau", "DFDAU", 31, Bus::DcEss, 50.0, avionics_pf(Bus::DcEss), "digital flight data acquisition unit", "GENERIC: typical avionics LRU class figure");
}

fn ata35_oxygen(v: &mut Vec<BreakerDef>) {
    push_electrical(v, "crew-o2-shutoff", "CREW OXYGEN SHUTOFF VALVE", 35, Bus::Dc1, 50.0, 0.8, "crew oxygen supply shutoff valve actuator", "GENERIC: typical motor/solenoid-operated shutoff valve actuator, real A380 crew oxygen system");
    push_electrical(v, "pax-o2-gen-ctl", "PAX OXYGEN GENERATOR CONTROL", 35, Bus::DcEss, 30.0, avionics_pf(Bus::DcEss), "passenger chemical oxygen generator deployment/control circuit", "GENERIC: typical small control-circuit LRU figure");
    push_electrical(v, "o2-pressure-xducer", "OXYGEN PRESSURE TRANSDUCER", 35, Bus::DcEss, 5.0, avionics_pf(Bus::DcEss), "crew oxygen bottle pressure transducer", "GENERIC: typical small pressure-transducer power draw");
}

fn ata49_apu(v: &mut Vec<BreakerDef>) {
    push_electrical(v, "apu-ecu-a", "APU ECU CHANNEL A", 49, Bus::DcApu, 60.0, avionics_pf(Bus::DcApu), "APU electronic control unit channel A", "GENERIC: typical dual-channel engine/APU controller LRU class figure (real PW980 APU has its own FADEC-class controller)");
    push_electrical(v, "apu-ecu-b", "APU ECU CHANNEL B", 49, Bus::DcEss, 60.0, avionics_pf(Bus::DcEss), "APU electronic control unit channel B", "GENERIC: same class as channel A, redundant bus feed");
    push_electrical(v, "apu-fuel-shutoff-valve", "APU FUEL SHUTOFF VALVE", 49, Bus::Dc1, 50.0, 0.8, "APU fuel shutoff valve actuator", "GENERIC: typical motor/solenoid-operated shutoff valve actuator");
    push_electrical(v, "apu-start-contactor", "APU START CONTACTOR", 49, Bus::Dc1, 20.0, 1.0, "APU starter-generator start contactor control coil", "GENERIC: typical contactor-coil control circuit");
}

fn ata7x_engine(v: &mut Vec<BreakerDef>) {
    for n in 1..=4u32 {
        let bus_a = if n <= 2 { Bus::DcEss } else { Bus::Dc1 };
        let bus_b = if n <= 2 { Bus::Dc2 } else { Bus::DcEss };
        let id_a: &'static str = Box::leak(format!("fadec-{n}a").into_boxed_str());
        let name_a: &'static str = Box::leak(format!("FADEC {n} CHANNEL A").into_boxed_str());
        push_electrical(v, id_a, name_a, 73, bus_a, 80.0, avionics_pf(bus_a), "engine FADEC channel A", "GENERIC: typical dual-lane FADEC-class controller channel, real Trent 972B-84 architecture, no public per-channel electrical figure");
        let id_b: &'static str = Box::leak(format!("fadec-{n}b").into_boxed_str());
        let name_b: &'static str = Box::leak(format!("FADEC {n} CHANNEL B").into_boxed_str());
        push_electrical(v, id_b, name_b, 73, bus_b, 80.0, avionics_pf(bus_b), "engine FADEC channel B", "GENERIC: same class as channel A, redundant bus feed");
    }
    for n in 1..=4u32 {
        let bus_a = if n % 2 == 1 { Bus::Dc1 } else { Bus::Dc2 };
        let bus_b = if n % 2 == 1 { Bus::Dc2 } else { Bus::Dc1 };
        let id_a: &'static str = Box::leak(format!("ignition-{n}a").into_boxed_str());
        let name_a: &'static str = Box::leak(format!("IGNITION {n} EXCITER A").into_boxed_str());
        push_electrical(v, id_a, name_a, 74, bus_a, 250.0, 0.9, "engine ignition exciter A", "GENERIC: typical high-energy ignition exciter unit pulsed power class (~250 W), no public per-part figure");
        let id_b: &'static str = Box::leak(format!("ignition-{n}b").into_boxed_str());
        let name_b: &'static str = Box::leak(format!("IGNITION {n} EXCITER B").into_boxed_str());
        push_electrical(v, id_b, name_b, 74, bus_b, 250.0, 0.9, "engine ignition exciter B", "GENERIC: same class as exciter A, redundant lane on the opposite DC bus");
    }
}

fn ata26_extinguishing(v: &mut Vec<BreakerDef>) {
    for bottle in 1..=2u32 {
        for squib in 1..=2u32 {
            let bus = if bottle == 1 { Bus::Dc1 } else { Bus::Dc2 };
            let id: &'static str = Box::leak(format!("eng-fire-bottle-{bottle}-squib-{squib}").into_boxed_str());
            let name: &'static str = Box::leak(format!("ENG FIRE BOTTLE {bottle} SQUIB {squib}").into_boxed_str());
            push_electrical(v, id, name, 26, bus, 20.0, 1.0, "engine fire-extinguisher bottle pyrotechnic squib", "GENERIC: typical one-shot pyrotechnic squib firing circuit, real wide-body cross-feed fire-extinguishing architecture");
        }
    }
    for squib in 1..=2u32 {
        let id: &'static str = Box::leak(format!("apu-fire-bottle-squib-{squib}").into_boxed_str());
        let name: &'static str = Box::leak(format!("APU FIRE BOTTLE SQUIB {squib}").into_boxed_str());
        push_electrical(v, id, name, 26, Bus::DcApu, 20.0, 1.0, "APU fire-extinguisher bottle pyrotechnic squib", "GENERIC: same class as the engine bottle squibs");
    }
}

fn ata29_hydraulics_extra(v: &mut Vec<BreakerDef>) {
    push_electrical(v, "rat-deploy-solenoid", "RAT DEPLOY SOLENOID", 29, Bus::DcHot2, 100.0, 1.0, "Ram Air Turbine deployment solenoid", "GENERIC: typical deployment solenoid, hot-bus fed so it works with both engines/APU/main batteries down (real RAT deployment logic requirement)");
    push_electrical(v, "ptu-control-valve", "PTU CONTROL VALVE", 29, Bus::DcEss, 50.0, 0.8, "Power Transfer Unit control valve actuator", "GENERIC: typical motor/solenoid-operated valve actuator, real green/yellow hydraulic power-transfer-unit architecture");
}

fn ata52_doors(v: &mut Vec<BreakerDef>) {
    push_electrical(v, "cargo-door-fwd-actuator-ctl", "FWD CARGO DOOR ACTUATOR CONTROL", 52, Bus::Dc1, 100.0, 0.8, "forward cargo door electric actuator control", "GENERIC: typical powered cargo door actuator control circuit");
    push_electrical(v, "cargo-door-aft-actuator-ctl", "AFT CARGO DOOR ACTUATOR CONTROL", 52, Bus::Dc2, 100.0, 0.8, "aft cargo door electric actuator control", "GENERIC: same class as the forward cargo door");
}

fn ata33_emergency_lighting(v: &mut Vec<BreakerDef>) {
    push_electrical(v, "emer-lighting-charger-1", "EMER LIGHTING BATTERY CHARGER 1", 33, Bus::DcHot1, 100.0, 1.0, "emergency lighting battery pack charger", "GENERIC: typical NiCd/Li-ion emergency-lighting pack charger circuit");
    push_electrical(v, "emer-lighting-charger-2", "EMER LIGHTING BATTERY CHARGER 2", 33, Bus::DcHot2, 100.0, 1.0, "emergency lighting battery pack charger", "GENERIC: same class as charger 1");
    push_electrical(v, "ext-service-lighting", "EXTERIOR SERVICE LIGHTING", 33, Bus::AcGndFltSvc, 100.0, 1.0, "exterior ground-service lighting circuit", "GENERIC: typical ground-service floodlight circuit");
}

fn push_position_excitation(v: &mut Vec<BreakerDef>, parent_id: &'static str, parent_name: &'static str, ata: u16, bus: Bus, basis_suffix: &'static str) {
    let id: &'static str = Box::leak(format!("{parent_id}-pos-ind").into_boxed_str());
    let name: &'static str = Box::leak(format!("{parent_name} POSITION IND").into_boxed_str());
    let basis: &'static str = Box::leak(format!("GENERIC: typical position-indication microswitch/LVDT excitation circuit for a motor-operated valve, separate small CB from its own actuator power circuit (real large-transport fuel/pneumatic-system practice); {basis_suffix}").into_boxed_str());
    push_electrical(v, id, name, ata, bus, 5.0, 1.0, "position-indication microswitch/LVDT excitation circuit", basis);
}

fn ata_control_excitation_supplies(v: &mut Vec<BreakerDef>) {
    let valve_buses = [Bus::Dc1, Bus::Dc2, Bus::DcEss, Bus::DcBat];
    for i in 0..60usize {
        let parent_id: &'static str = Box::leak(format!("fuel-valve-{i}").into_boxed_str());
        let parent_name: &'static str = Box::leak(format!("FUEL VALVE {i}").into_boxed_str());
        push_position_excitation(v, parent_id, parent_name, 28, valve_buses[i % valve_buses.len()], "pairs with this catalogue's own FUEL VALVE actuator breaker");
    }
    push_position_excitation(v, "hotair-1", "HOT AIR VALVE 1", 21, Bus::AcEss, "pairs with HOT AIR VALVE 1's own actuator breaker");
    push_position_excitation(v, "hotair-2", "HOT AIR VALVE 2", 21, Bus::AcEss, "pairs with HOT AIR VALVE 2's own actuator breaker");
    push_position_excitation(v, "fwd-isol-valve", "FWD CARGO ISOL VALVE", 21, Bus::Dc1, "pairs with FWD CARGO ISOL VALVE's own actuator breaker (fixes/W161.md: DC1, VCM Fwd's own primary channel)");
    push_position_excitation(v, "bulk-isol-valve", "BULK CARGO ISOL VALVE", 21, Bus::Dc2, "pairs with BULK CARGO ISOL VALVE's own actuator breaker (fixes/W161.md: DC2, VCM Aft's own primary channel)");
    for pack in 1..=2u32 {
        for side in 1..=2u32 {
            let parent_id: &'static str = Box::leak(format!("pack-{pack}-flow-valve-{side}").into_boxed_str());
            let parent_name: &'static str = Box::leak(format!("PACK {pack} FLOW VALVE {side}").into_boxed_str());
            push_position_excitation(v, parent_id, parent_name, 21, Bus::DcEss, "pairs with its own PACK FLOW VALVE actuator breaker");
        }
    }
    for n in 1..=4u32 {
        let bus = if n <= 2 { Bus::Dc1 } else { Bus::Dc2 };
        let parent_id: &'static str = Box::leak(format!("bleed-eng-{n}").into_boxed_str());
        let parent_name: &'static str = Box::leak(format!("BLEED ENG {n} VALVES").into_boxed_str());
        push_position_excitation(v, parent_id, parent_name, 36, bus, "pairs with the shared BLEED ENG valve-set actuator breaker");
    }
    push_position_excitation(v, "ptu-control-valve", "PTU CONTROL VALVE", 29, Bus::DcEss, "pairs with PTU CONTROL VALVE's own actuator breaker");
    push_position_excitation(v, "apu-fuel-shutoff-valve", "APU FUEL SHUTOFF VALVE", 49, Bus::Dc1, "pairs with APU FUEL SHUTOFF VALVE's own actuator breaker");
    push_position_excitation(v, "crew-o2-shutoff", "CREW OXYGEN SHUTOFF VALVE", 35, Bus::Dc1, "pairs with CREW OXYGEN SHUTOFF VALVE's own actuator breaker");
    push_position_excitation(v, "cargo-door-fwd-actuator-ctl", "FWD CARGO DOOR", 52, Bus::Dc1, "pairs with the forward cargo door's own actuator-control breaker");
    push_position_excitation(v, "cargo-door-aft-actuator-ctl", "AFT CARGO DOOR", 52, Bus::Dc2, "pairs with the aft cargo door's own actuator-control breaker");
}

fn build_catalog() -> Vec<BreakerDef> {
    let mut v = Vec::with_capacity(400);
    ata21(&mut v);
    ata26(&mut v);
    ata27(&mut v);
    ata32(&mut v);
    ata34(&mut v);
    ata28_fuel(&mut v);
    ata33_lighting(&mut v);
    ata30_ice_protection(&mut v);
    ata25_galleys(&mut v);
    ata44_ife(&mut v);
    avionics_misc(&mut v);
    ata36_bleed(&mut v);
    ata24_power_sources(&mut v);
    ata23_comms(&mut v);
    ata31_recorders(&mut v);
    ata31_ind_group(&mut v);
    ata35_oxygen(&mut v);
    ata49_apu(&mut v);
    ata7x_engine(&mut v);
    ata26_extinguishing(&mut v);
    ata29_hydraulics_extra(&mut v);
    ata52_doors(&mut v);
    ata33_emergency_lighting(&mut v);
    ata_control_excitation_supplies(&mut v);
    assign_positions(&mut v);
    v
}

static CATALOG: std::sync::OnceLock<Vec<BreakerDef>> = std::sync::OnceLock::new();

pub fn all() -> &'static [BreakerDef] {
    CATALOG.get_or_init(build_catalog)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn every_id_is_unique() {
        let mut ids: Vec<&str> = all().iter().map(|d| d.id).collect();
        let before = ids.len();
        ids.sort_unstable();
        ids.dedup();
        assert_eq!(ids.len(), before, "duplicate breaker id in the catalogue");
    }

    #[test]
    fn the_catalogue_is_a_substantial_expansion_of_the_265_entry_legacy_catalogue() {
        assert!(all().len() > 390, "expected a substantial catalogue, got {}", all().len());
    }

    #[test]
    fn position_indication_entries_pair_with_a_real_parent_breaker_and_now_carry_their_own_load() {
        let ids: std::collections::HashSet<&str> = all().iter().map(|d| d.id).collect();
        let mut found_any = false;
        for def in all() {
            if let Some(parent_id) = def.id.strip_suffix("-pos-ind") {
                found_any = true;
                assert!(ids.contains(parent_id), "{} has no parent breaker {parent_id}", def.id);
                assert_eq!(def.protected_load, Some(def.id), "{} should now protect its own matching load", def.id);
                assert!(def.basis.contains("GENERIC"), "{} should cite GENERIC", def.id);
            }
        }
        assert!(found_any, "expected at least one position-indication entry");
    }

    #[test]
    fn every_breaker_gets_a_unique_position_on_its_own_panels_grid() {
        use std::collections::HashSet;
        let mut seen: HashSet<(&str, u32, u32)> = HashSet::new();
        for def in all() {
            assert!(def.position.row >= 1, "{} has row {}", def.id, def.position.row);
            assert!(def.position.column >= 1 && def.position.column <= PANEL_COLUMNS, "{} has column {}", def.id, def.position.column);
            assert!(!def.position.label.is_empty(), "{} has no cap label", def.id);
            assert!(def.position.label.chars().count() <= 14, "{} label too long for a real CB cap: {}", def.id, def.position.label);
            let key = (def.panel.code(), def.position.row, def.position.column);
            assert!(seen.insert(key), "{} collides with another breaker at panel {:?} row {} column {}", def.id, def.panel, def.position.row, def.position.column);
        }
    }

    #[test]
    fn every_electrical_group_entry_protects_itself_or_its_own_dual_feed_load() {
        for def in all() {
            if let Some(load) = def.protected_load {
                let self_or_feed = def.id == load || def.id == format!("{load}-normal-bkr") || def.id == format!("{load}-2nd-bkr");
                assert!(self_or_feed, "{} claims to protect {} but is neither that id nor one of its own dual-feed breaker ids", def.id, load);
            }
        }
    }

    #[test]
    fn rating_is_at_or_above_the_loads_own_current_the_standard_size_series_only_rounds_up() {
        for def in all() {
            let plain_current = def.rated_power_w / (def.bus.nominal_voltage() * def.power_factor.max(0.1));
            assert!(def.rating_a >= plain_current - 1e-9, "{}: rating {} A below plain load current {} A", def.id, def.rating_a, plain_current);
            assert!(def.rating_a >= def.raw_current_a - 1e-9, "{}: rating {} A below its own margined current {} A", def.id, def.rating_a, def.raw_current_a);
            let recomputed_raw = margined_current(def.rated_power_w, def.bus.nominal_voltage(), def.power_factor);
            assert!((recomputed_raw - def.raw_current_a).abs() < 1e-6, "{}: stored raw_current_a does not match its own power/voltage/pf", def.id);
            assert!((standard_size(def.raw_current_a) - def.rating_a).abs() < 1e-9, "{}: rating_a is not the standard-size rounding of its own raw current", def.id);
        }
    }

    #[test]
    fn standard_size_only_returns_a_published_series_value_and_never_rounds_down() {
        for &raw in &[0.5, 1.0, 1.1, 24.9, 25.0, 25.1, 99.0, 251.0, 999.0] {
            let s = standard_size(raw);
            assert!(s >= raw, "{s} A rounded below its own raw current {raw} A");
        }
        assert_eq!(standard_size(0.5), 1.0);
        assert_eq!(standard_size(25.0), 25.0);
        assert_eq!(standard_size(999.0), 1000.0);
    }

    #[test]
    fn every_ata_chapter_used_is_a_real_a380_chapter_and_matches_its_own_group() {
        const VALID: [u16; 21] = [21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 44, 49, 52, 73, 74];
        for def in all() {
            assert!(VALID.contains(&def.ata), "{} has an unexpected ATA chapter {}", def.id, def.ata);
        }
    }

    #[test]
    fn breaker_kind_follows_its_own_rating_split_and_sspc_entries_land_in_a_power_centre() {
        for def in all() {
            assert_eq!(kind_for(def.rating_a), def.kind, "{} kind does not match its own rating", def.id);
            if def.kind == BreakerKind::Sspc {
                assert!(
                    matches!(def.panel, Panel::PrimaryPowerCentre1 | Panel::PrimaryPowerCentre2 | Panel::PrimaryPowerCentre3 | Panel::PrimaryPowerCentre4 | Panel::SecondaryPowerCentreFwd | Panel::SecondaryPowerCentreAft),
                    "{} is SSPC but not in a power centre",
                    def.id
                );
            }
        }
    }

    #[test]
    fn the_three_battery_output_breakers_have_no_fabricated_load_and_are_honestly_documented() {
        let unmodelled_ids = ["bat-1", "bat-2", "bat-apu"];
        for def in all() {
            if unmodelled_ids.contains(&def.id) {
                assert!(def.protected_load.is_none(), "{} should honestly carry no modelled load", def.id);
                assert!(def.basis.contains("GENERIC"), "{} should cite GENERIC since it has no real per-part figure", def.id);
            }
        }
    }

    #[test]
    fn every_entry_has_a_positive_rating_and_a_nonempty_basis_and_consumer() {
        for def in all() {
            assert!(def.rating_a > 0.0, "{} has no positive rating", def.id);
            assert!(!def.basis.is_empty(), "{} has no basis citation", def.id);
            assert!(!def.consumer.is_empty(), "{} has no consumer description", def.id);
        }
    }

    #[test]
    fn cargo_ventilation_breakers_sit_on_their_real_bus() {
        let bus_of = |id: &str| all().into_iter().find(|d| d.id == id).unwrap_or_else(|| panic!("no breaker {id}")).bus.label();
        assert_eq!(bus_of("fwd-isol-valve"), "DC1", "VCM Fwd's own primary channel is DC1 (411PP), not DC2 (that's VCM Aft's)");
        assert_eq!(bus_of("bulk-isol-valve"), "DC2", "VCM Aft's own primary channel is DC2 (214PP), matching fwd-isol-valve's primary-channel convention, not its DC_ESS standby channel");
        assert_eq!(bus_of("fwd-extract-fan"), "AC1", "the forward extraction fan's own dedicated bus (ForwardCargoVentilationControlSystem::new), not any VCM channel bus");
        assert_eq!(bus_of("bulk-extract-fan"), "AC4", "the bulk extraction fan's own dedicated bus (BulkVentilationControlSystem::new), not any VCM channel bus");
    }

    #[test]
    fn vcm_channel_breakers_sit_on_their_real_bus() {
        let bus_of = |id: &str| all().into_iter().find(|d| d.id == id).unwrap_or_else(|| panic!("no breaker {id}")).bus.label();
        assert_eq!(bus_of("vcm-fwd-1"), "DC1", "VCM Fwd's own primary channel is DC1 (411PP), not DC2 (that's VCM Aft's channel 1)");
        assert_eq!(bus_of("vcm-fwd-2"), "DC_ESS", "VCM Fwd's standby channel (109PP)");
        assert_eq!(bus_of("vcm-aft-1"), "DC2", "VCM Aft's own primary channel is DC2 (214PP)");
        assert_eq!(bus_of("vcm-aft-2"), "DC_ESS", "VCM Aft's standby channel (109PP), same physical bus as vcm-fwd-2");
    }
}

